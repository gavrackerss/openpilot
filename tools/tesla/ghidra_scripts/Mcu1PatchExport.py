# Ghidra script: patch a Tesla MCU1 raw image, refresh its image CRC, and export
# both padded BIN and Motorola S-record files for bench flashing.
#
# Usage in Ghidra Script Manager after importing the raw image at base 0x40000000:
#   1. Edit PATCHES below, or pass script arguments:
#        out=/path/out-prefix patch=0x24d5c:000870f0 patch=0x6f549:0870f0
#   2. Run this script.
#
# Defaults are intentionally conservative: no bytes are changed unless PATCHES or
# patch= arguments are supplied. The script still exports a CRC-refreshed image.

from __future__ import print_function

import os


# ---- User-editable defaults -------------------------------------------------
FLASH_BASE = 0x40000000
IMAGE_SIZE = 0x200000
PAD_BYTE = 0xff

# Header fields observed in Tesla_MCU1_ConfigCheckBypass.bin. Override with
# crc_offset=, crc_start=, crc_end= script args if your loader/header differs.
CRC_OFFSET = 0x8
CRC_START = 0x0c
CRC_END = IMAGE_SIZE

# Add default bench patches here as (file_offset, hex_bytes) tuples, e.g.:
# PATCHES = [(0x24d5c, "00086170"), (0x6f549, "086170")]
PATCHES = []
# -----------------------------------------------------------------------------


def parse_int(value):
  value = str(value).strip()
  return int(value, 16) if value.lower().startswith("0x") else int(value, 10)


def get_args():
  parsed = {"patch": []}
  for arg in getScriptArgs():
    if "=" not in arg:
      continue
    key, value = arg.split("=", 1)
    key = key.strip().lower()
    value = value.strip()
    if key == "patch":
      parsed["patch"].append(value)
    else:
      parsed[key] = value
  return parsed


def file_addr(offset):
  return currentProgram.getAddressFactory().getDefaultAddressSpace().getAddress(FLASH_BASE + offset)


def read_image(size):
  data = bytearray([PAD_BYTE] * size)
  mem = currentProgram.getMemory()
  for off in range(size):
    addr = file_addr(off)
    if mem.contains(addr):
      data[off] = getByte(addr) & 0xff
  return data


def write_bytes(offset, data):
  for index, value in enumerate(bytearray(data)):
    setByte(file_addr(offset + index), value)


def crc32_ieee(data):
  crc = 0xffffffff
  for byte in bytearray(data):
    crc ^= byte
    for _ in range(8):
      if crc & 1:
        crc = (crc >> 1) ^ 0xedb88320
      else:
        crc >>= 1
      crc &= 0xffffffff
  return crc ^ 0xffffffff


def srec_checksum(count, address_bytes, payload):
  total = count + sum(address_bytes) + sum(bytearray(payload))
  return (~total) & 0xff


def srec_line(record_type, address, payload):
  if record_type in (0, 1, 9):
    addr_len = 2
  elif record_type in (2, 8):
    addr_len = 3
  else:
    addr_len = 4
  address_bytes = [(address >> (8 * shift)) & 0xff for shift in range(addr_len - 1, -1, -1)]
  count = addr_len + len(payload) + 1
  chk = srec_checksum(count, address_bytes, payload)
  return "S%d%02X%s%s%02X" % (
    record_type,
    count,
    "".join("%02X" % b for b in address_bytes),
    "".join("%02X" % b for b in bytearray(payload)),
    chk,
  )


def write_srec(path, image, base, chunk=32):
  with open(path, "w") as f:
    f.write(srec_line(0, 0, bytearray(b"MCU1PATCH")) + "\n")
    count = 0
    for off in range(0, len(image), chunk):
      payload = image[off:off + chunk]
      f.write(srec_line(3, base + off, payload) + "\n")
      count += 1
    f.write(srec_line(7, base, bytearray()) + "\n")
  return count


def main():
  args = get_args()
  image_size = parse_int(args.get("image_size", IMAGE_SIZE))
  crc_offset = parse_int(args.get("crc_offset", CRC_OFFSET))
  crc_start = parse_int(args.get("crc_start", CRC_START))
  crc_end = parse_int(args.get("crc_end", image_size))
  out_prefix = args.get("out", os.path.join(os.path.expanduser("~"), "Tesla_MCU1_ConfigCheckBypass_patched"))

  patches = list(PATCHES)
  for spec in args.get("patch", []):
    off_text, hex_text = spec.split(":", 1)
    patches.append((parse_int(off_text), hex_text.replace(" ", "").replace("_", "")))

  print("Reading 0x%x bytes from Ghidra memory at 0x%x" % (image_size, FLASH_BASE))
  image = read_image(image_size)

  for off, hex_text in patches:
    patch = bytearray.fromhex(hex_text)
    print("Patching file offset 0x%x VA 0x%x: %s" % (off, FLASH_BASE + off, hex_text))
    image[off:off + len(patch)] = patch
    write_bytes(off, patch)

  if not (0 <= crc_offset <= image_size - 4 and 0 <= crc_start <= crc_end <= image_size):
    raise ValueError("CRC offsets/range are outside image_size")

  crc = crc32_ieee(image[crc_start:crc_end])
  image[crc_offset:crc_offset + 4] = bytearray([(crc >> 24) & 0xff, (crc >> 16) & 0xff, (crc >> 8) & 0xff, crc & 0xff])
  write_bytes(crc_offset, image[crc_offset:crc_offset + 4])
  verify = crc32_ieee(image[crc_start:crc_end])

  bin_path = out_prefix + ".bin"
  srec_path = out_prefix + ".srec"
  with open(bin_path, "wb") as f:
    f.write(image)
  records = write_srec(srec_path, image, FLASH_BASE)

  print("Wrote %s (%d bytes, padded with 0x%02x)" % (bin_path, len(image), PAD_BYTE))
  print("Wrote %s (%d S3 data records)" % (srec_path, records))
  print("Header CRC at 0x%x = 0x%08x over [0x%x, 0x%x); verify=0x%08x" % (crc_offset, crc, crc_start, crc_end, verify))


main()
