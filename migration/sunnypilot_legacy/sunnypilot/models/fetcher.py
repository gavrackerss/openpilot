"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import time
from datetime import datetime
import re

import requests
from requests.exceptions import (SSLError, RequestException, HTTPError)
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.models.helpers import is_bundle_version_compatible

from cereal import custom


class ModelParser:
  """Handles parsing of model data into cereal objects"""

  @staticmethod
  def _parse_download_uri(download_uri_data) -> custom.ModelManagerSP.DownloadUri:
    download_uri = custom.ModelManagerSP.DownloadUri()
    download_uri.uri = download_uri_data.get("url")
    download_uri.sha256 = download_uri_data.get("sha256")
    return download_uri

  @staticmethod
  def _parse_artifact(artifact_data) -> custom.ModelManagerSP.Artifact:
    artifact = custom.ModelManagerSP.Artifact()
    artifact.fileName = artifact_data.get("file_name")
    artifact.downloadUri = ModelParser._parse_download_uri(artifact_data.get("download_uri", {}))
    # V23+ stores the digest of each separately downloadable model chunk.
    chunks = []
    for entry in artifact_data.get("chunks", []):
      chunk = custom.ModelManagerSP.Chunk()
      chunk.fileName = entry["file_name"]
      chunk.sha256 = entry["sha256"]
      chunks.append(chunk)
    artifact.chunks = chunks
    return artifact

  @staticmethod
  def _parse_model(model_data) -> custom.ModelManagerSP.Model:
    model = custom.ModelManagerSP.Model()

    model.type = model_data.get("type")
    model.artifact = ModelParser._parse_artifact(model_data.get("artifact", {}))
    if metadata := model_data.get("metadata"):
      model.metadata = ModelParser._parse_artifact(metadata)
    return model

  @staticmethod
  def _parse_overrides(overrides_data: dict[str, str]) -> list[custom.ModelManagerSP.Override]:
    overrides = []
    for key, value in overrides_data.items():
      override = custom.ModelManagerSP.Override()
      override.key = key
      override.value = value
      overrides.append(override)
    return overrides

  @staticmethod
  def _parse_bundle(bundle) -> custom.ModelManagerSP.ModelBundle:
    model_bundle = custom.ModelManagerSP.ModelBundle()
    model_bundle.index = int(bundle["index"])
    model_bundle.internalName = bundle["short_name"]
    model_bundle.displayName = bundle["display_name"]
    model_bundle.models = [ModelParser._parse_model(model) for model in bundle.get("models",[])]
    model_bundle.status = 0
    model_bundle.generation = int(bundle["generation"])
    model_bundle.environment = bundle["environment"]
    model_bundle.runner = bundle.get("runner", custom.ModelManagerSP.Runner.snpe)
    model_bundle.is20hz = bundle.get("is_20hz", False)
    model_bundle.minimumSelectorVersion = int(bundle["minimum_selector_version"])
    model_bundle.overrides = ModelParser._parse_overrides(bundle.get("overrides", {}))
    model_bundle.ref = bundle.get("ref")

    return model_bundle

  @staticmethod
  def parse_models(json_data: dict) -> list[custom.ModelManagerSP.ModelBundle]:
    found_bundles = [ModelParser._parse_bundle(bundle) for bundle in json_data.get("bundles", [])]
    return [bundle for bundle in found_bundles if is_bundle_version_compatible(bundle.to_dict())]


class ModelCache:
  """Handles caching of model data to avoid frequent remote fetches"""

  def __init__(self, params: Params, cache_timeout: int = int(3600 * 1e9), source: str = "legacy"):
    self.params = params
    self.cache_timeout = cache_timeout
    self._LAST_SYNC_KEY = "ModelManager_LastSyncTime" if source == "legacy" else "ModelManager_LastSyncTime_v23"
    self._CACHE_KEY = "ModelManager_ModelsCache" if source == "legacy" else "ModelManager_ModelsCache_v23"

  def _is_expired(self) -> bool:
    """Checks if the cache has expired"""
    current_time = int(time.monotonic() * 1e9)
    last_sync = self.params.get(self._LAST_SYNC_KEY) or 0
    return bool(last_sync == 0) or (current_time - last_sync) >= self.cache_timeout

  def get(self) -> tuple[dict, bool]:
    """
    Retrieves cached model data and expiration status atomically.
    Returns: Tuple of (cached_data, is_expired)
    If no cached data exists or on error, returns an empty dict
    """
    try:
      cached_data = self.params.get(self._CACHE_KEY)
      if not cached_data:
        return {}, True
      return cached_data, self._is_expired()
    except Exception as e:
      cloudlog.exception(f"Error retrieving cached model data: {str(e)}")
      return {}, True

  def set(self, data: dict) -> None:
    """Updates the cache with new model data"""
    self.params.put(self._CACHE_KEY, data)
    self.params.put(self._LAST_SYNC_KEY, int(time.monotonic() * 1e9))


class ModelFetcher:
  """Handles fetching and caching of model data from remote source.

  XNOR V238: network refreshes are clock-aware and backed off so early boot cannot
  hammer GitHub while DNS/TLS/time are still coming up. Cached data remains usable
  immediately, and an explicit UI refresh bypasses retry backoff once the wall clock
  is valid.
  """
  MODEL_URL = "https://raw.githubusercontent.com/sunnypilot/sunnypilot-models/refs/heads/gh-pages/docs/driving_models_v17.json"
  MODEL_URL_MODERN = "https://raw.githubusercontent.com/sunnypilot/sunnypilot-models/refs/heads/gh-pages/docs/driving_models_v23.json"
  # This branch is a 2026 build. A wall clock older than 2026-01-01 cannot be trusted
  # for HTTPS certificate validation. The upper bound catches obviously-corrupt RTCs.
  MIN_VALID_UNIX_TIME = 1767225600.0  # 2026-01-01T00:00:00Z
  MAX_VALID_UNIX_TIME = 4102444800.0  # 2100-01-01T00:00:00Z
  RETRY_BACKOFF_S = (10.0, 30.0, 60.0, 300.0)
  CLOCK_LOG_INTERVAL_S = 60.0

  def __init__(self, params: Params):
    self.params = params
    self.model_cache = ModelCache(params)
    self.modern_cache = ModelCache(params, source="modern")
    self.model_parser = ModelParser()
    self.catalog_available = False
    self._pending_force_refresh = False
    self._retry_index = 0
    self._next_retry_mono = 0.0
    self._last_clock_log_mono = -1e9

  def _wall_clock_valid(self) -> bool:
    now = time.time()
    return self.MIN_VALID_UNIX_TIME <= now <= self.MAX_VALID_UNIX_TIME

  def _consume_force_refresh(self) -> bool:
    """UI writes -1 to ModelManager_LastSyncTime to request an immediate refresh."""
    try:
      requested = int(self.params.get("ModelManager_LastSyncTime") or 0) < 0
    except Exception:
      requested = False
    if requested:
      # Consume the sentinel once. If the clock is not ready yet, _pending_force_refresh
      # keeps the request alive in memory until HTTPS can be attempted safely.
      self.params.put("ModelManager_LastSyncTime", 0)
      self._pending_force_refresh = True
    return requested

  def _log_clock_deferred(self) -> None:
    now = time.monotonic()
    if now - self._last_clock_log_mono >= self.CLOCK_LOG_INTERVAL_S:
      cloudlog.warning("Model manifest refresh deferred until system clock is valid")
      self._last_clock_log_mono = now

  def _schedule_retry(self) -> None:
    delay = self.RETRY_BACKOFF_S[min(self._retry_index, len(self.RETRY_BACKOFF_S) - 1)]
    self._next_retry_mono = time.monotonic() + delay
    self._retry_index = min(self._retry_index + 1, len(self.RETRY_BACKOFF_S) - 1)
    cloudlog.warning(f"Model manifest refresh failed; next retry in {int(delay)}s")

  def _reset_retry(self) -> None:
    self._retry_index = 0
    self._next_retry_mono = 0.0

  def _fetch_and_cache_models(self, model_url: str, model_cache: ModelCache) -> bool:
    """Fetch one manifest, preserving the last trusted manifest on errors."""
    try:
      response = requests.get(model_url, timeout=10)
      response.raise_for_status()
      json_data = response.json()
      # Never replace a good catalogue with incompatible or empty entries.
      if not isinstance(json_data, dict) or not self.model_parser.parse_models(json_data):
        raise ValueError(f"Model manifest has no compatible bundles: {model_url}")
      model_cache.set(json_data)
      cloudlog.info(f"Updated driving-model catalogue: {model_url}")
      return True
    except (RequestException, ValueError, KeyError, TypeError) as e:
      cloudlog.warning(f"Model catalogue download failed: {model_url}: {e}")
    except Exception:
      cloudlog.exception(f"Unexpected model catalogue error: {model_url}")
    return False

  @staticmethod
  def _recent_small_model(bundle: dict) -> bool:
    """Only contemporary standard (non-Chestnut) Gen12 20 Hz models.

    Do not treat missing is_big as False: fail closed for unknown catalogues.
    Keep the list naturally open to future entries in this pinned v23 series.
    """
    if bundle.get("is_big") is not False:
      return False
    if str(bundle.get("runner")) != "tinygrad" or str(bundle.get("generation")) != "12":
      return False
    if not bundle.get("is_20hz"):
      return False
    match = re.search(r"\(([A-Za-z]+ \d{1,2}, 2026)\)$", bundle.get("display_name", ""))
    if match is None:
      return False
    try:
      return datetime.strptime(match.group(1), "%B %d, %Y") >= datetime(2026, 6, 1)
    except ValueError:
      return False

  def get_available_bundles(self) -> list[custom.ModelManagerSP.ModelBundle]:
    """Compact Qualcomm-class model picker: recent standard models + fallback.

    The downloaded / active legacy bundle is retained and never invalidated
    just because earlier models have been hidden from the selector.
    """
    self._consume_force_refresh()
    legacy_data, legacy_expired = self.model_cache.get()
    modern_data, modern_expired = self.modern_cache.get()
    needs_legacy = legacy_expired or not legacy_data or self._pending_force_refresh
    needs_modern = modern_expired or not modern_data or self._pending_force_refresh

    if (needs_legacy or needs_modern) and self._wall_clock_valid():
      if time.monotonic() >= self._next_retry_mono or self._pending_force_refresh:
        self._pending_force_refresh = False
        all_ok = True
        if needs_legacy:
          all_ok = self._fetch_and_cache_models(self.MODEL_URL, self.model_cache) and all_ok
        if needs_modern:
          all_ok = self._fetch_and_cache_models(self.MODEL_URL_MODERN, self.modern_cache) and all_ok
        if all_ok:
          self._reset_retry()
        else:
          self._schedule_retry()
        legacy_data, _ = self.model_cache.get()
        modern_data, _ = self.modern_cache.get()
    elif needs_legacy or needs_modern:
      self._log_clock_deferred()

    # Do not reset an active model on partial catalogue failure.
    self.catalog_available = bool(legacy_data and modern_data)
    legacy_all = self.model_parser.parse_models(legacy_data)

    # Keep the current working OPM10V3 available as an explicit fallback.
    # Its v17 metadata and hashes must NOT be substituted by the v23 build
    # of the same ref (which has incompatible chunks and selector version).
    fallback_ref = "7d4c295c27bc3f72727c49ec73b98e45745b588d"
    legacy_fallback = next((b for b in legacy_all if b.ref == fallback_ref), None)

    active_raw = self.params.get("ModelManager_ActiveBundle") or {}
    active_ref = active_raw.get("ref") if isinstance(active_raw, dict) else None
    active_legacy = next((b for b in legacy_all if b.ref == active_ref), None)

    modern_raw = modern_data.get("bundles", []) if isinstance(modern_data, dict) else []
    recent_data = {"bundles": [b for b in modern_raw if self._recent_small_model(b)]}
    recent_models = self.model_parser.parse_models(recent_data)
    recent_models.sort(key=lambda b: b.index, reverse=True)

    result = []
    refs = set()
    # Explicit, stable, high-numbered indices avoid collisions with legacy
    # index values and prevent stale queued downloads selecting wrong models.
    for bundle in recent_models:
      bundle.index = 1000 + bundle.index
      if bundle.ref not in refs:
        result.append(bundle)
        refs.add(bundle.ref)

    for preserved in (legacy_fallback, active_legacy):
      if preserved is not None and preserved.ref not in refs:
        # Preserve its original index and *all* artifact metadata unchanged.
        result.append(preserved)
        refs.add(preserved.ref)
    return result

if __name__ == "__main__":
  params = Params()
  model_fetcher = ModelFetcher(params)
  bundles = model_fetcher.get_available_bundles()
  for bundle in bundles:
    for model in bundle.models:
      model_overrides = {override.key: override.value for override in bundle.overrides}
      # Print model details
      print(f"Bundle: {bundle.internalName}, Type: {model.type}, Status: {bundle.status}, Overrides: {model_overrides}")
      # Print artifact details
      print(f"Artifact: {model.artifact.fileName}, Download URI: {model.artifact.downloadUri.uri}")
      # Print metadata details
      print(f"Metadata: {model.metadata.fileName}, Download URI: {model.metadata.downloadUri.uri}")
