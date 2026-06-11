#!/usr/bin/env python3
import zmq
from typing import NoReturn

import cereal.messaging as messaging
from openpilot.common.logging_extra import SwagLogFileFormatter
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import get_file_handler

XNOR_V163_LOGMESSAGED_MSGQ_PUBLISH_CAP = True
MAX_PUBLISHED_LOG_BYTES = 16 * 1024


def cap_record_for_msgq(record: str) -> str:
  raw = record.encode("utf-8", errors="replace")
  if len(raw) <= MAX_PUBLISHED_LOG_BYTES:
    return record

  suffix = f"\n...[truncated by XNOR_V163_LOGMESSAGED_MSGQ_PUBLISH_CAP original_bytes={len(raw)}]"
  suffix_raw = suffix.encode("utf-8", errors="replace")
  keep = max(0, MAX_PUBLISHED_LOG_BYTES - len(suffix_raw))
  return raw[:keep].decode("utf-8", errors="replace") + suffix


def main() -> NoReturn:
  log_handler = get_file_handler()
  log_handler.setFormatter(SwagLogFileFormatter(None))
  log_level = 20  # logging.INFO

  ctx = zmq.Context.instance()
  sock = ctx.socket(zmq.PULL)
  sock.bind(Paths.swaglog_ipc())

  # and we publish them
  log_message_sock = messaging.pub_sock('logMessage')
  error_log_message_sock = messaging.pub_sock('errorLogMessage')

  try:
    while True:
      dat = b''.join(sock.recv_multipart())
      level = dat[0]
      record = dat[1:].decode("utf-8")
      if level >= log_level:
        log_handler.emit(record)

      publish_record = cap_record_for_msgq(record)

      # then we publish them
      msg = messaging.new_message(None, valid=True, logMessage=publish_record)
      log_message_sock.send(msg.to_bytes())

      if level >= 40:  # logging.ERROR
        msg = messaging.new_message(None, valid=True, errorLogMessage=publish_record)
        error_log_message_sock.send(msg.to_bytes())
  finally:
    sock.close()
    ctx.term()

    # can hit this if interrupted during a rollover
    try:
      log_handler.close()
    except ValueError:
      pass

if __name__ == "__main__":
  main()
