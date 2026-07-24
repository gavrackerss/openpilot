// Ghidra Java script: patch a Tesla MCU1 raw image, refresh its image CRC, and
// export both padded BIN and Motorola S-record files for bench flashing.
//
// Usage in Ghidra Script Manager after importing the raw image at base 0x40000000:
//   1. Edit PATCHES below, or pass script arguments:
//        out=/path/out-prefix patch=0x24d5c:000870f0 patch=0x6f549:0870f0
//   2. Run this script.
//
// Defaults are intentionally conservative: no bytes are changed unless PATCHES
// or patch= arguments are supplied. The script still exports a CRC-refreshed image.

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.mem.Memory;

import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class Mcu1PatchExport extends GhidraScript {
  private static final long FLASH_BASE = 0x40000000L;
  private static final int IMAGE_SIZE = 0x200000;
  private static final int PAD_BYTE = 0xff;

  // Header fields observed in Tesla_MCU1_ConfigCheckBypass.bin. Override with
  // crc_offset=, crc_start=, crc_end= script args if your loader/header differs.
  private static final int CRC_OFFSET = 0x8;
  private static final int CRC_START = 0x0c;
  private static final int CRC_END = IMAGE_SIZE;

  // Add default bench patches here as {file_offset_hex, hex_bytes}, for example:
  // private static final String[][] PATCHES = {{"0x24d5c", "00086170"}, {"0x6f549", "086170"}};
  private static final String[][] PATCHES = {};

  private static class Patch {
    final int offset;
    final byte[] bytes;

    Patch(int offset, byte[] bytes) {
      this.offset = offset;
      this.bytes = bytes;
    }
  }

  @Override
  public void run() throws Exception {
    Map<String, List<String>> args = parseArgs(getScriptArgs());
    int imageSize = parseIntegerArg(firstArg(args, "image_size", Integer.toString(IMAGE_SIZE)));
    int crcOffset = parseIntegerArg(firstArg(args, "crc_offset", Integer.toString(CRC_OFFSET)));
    int crcStart = parseIntegerArg(firstArg(args, "crc_start", Integer.toString(CRC_START)));
    int crcEnd = parseIntegerArg(firstArg(args, "crc_end", Integer.toString(imageSize)));
    String outPrefix = firstArg(args, "out", System.getProperty("user.home") + "/Tesla_MCU1_ConfigCheckBypass_patched");

    List<Patch> patches = new ArrayList<>();
    for (String[] patch : PATCHES) {
      patches.add(new Patch(parseIntegerArg(patch[0]), parseHex(patch[1])));
    }
    for (String spec : args.getOrDefault("patch", new ArrayList<String>())) {
      String[] parts = spec.split(":", 2);
      if (parts.length != 2) {
        throw new IllegalArgumentException("patch argument must be offset:hexbytes, got: " + spec);
      }
      patches.add(new Patch(parseIntegerArg(parts[0]), parseHex(parts[1])));
    }

    println(String.format("Reading 0x%x bytes from Ghidra memory at 0x%x", imageSize, FLASH_BASE));
    byte[] image = readImage(imageSize);

    for (Patch patch : patches) {
      if (patch.offset < 0 || patch.offset + patch.bytes.length > image.length) {
        throw new IllegalArgumentException(String.format("Patch outside image: off=0x%x len=%d", patch.offset, patch.bytes.length));
      }
      println(String.format("Patching file offset 0x%x VA 0x%x: %s", patch.offset, FLASH_BASE + patch.offset, toHex(patch.bytes)));
      System.arraycopy(patch.bytes, 0, image, patch.offset, patch.bytes.length);
      writeBytes(patch.offset, patch.bytes);
    }

    if (!(0 <= crcOffset && crcOffset <= imageSize - 4 && 0 <= crcStart && crcStart <= crcEnd && crcEnd <= imageSize)) {
      throw new IllegalArgumentException("CRC offsets/range are outside image_size");
    }

    int crc = crc32Ieee(image, crcStart, crcEnd);
    image[crcOffset] = (byte) ((crc >>> 24) & 0xff);
    image[crcOffset + 1] = (byte) ((crc >>> 16) & 0xff);
    image[crcOffset + 2] = (byte) ((crc >>> 8) & 0xff);
    image[crcOffset + 3] = (byte) (crc & 0xff);
    writeBytes(crcOffset, Arrays.copyOfRange(image, crcOffset, crcOffset + 4));
    int verify = crc32Ieee(image, crcStart, crcEnd);

    String binPath = outPrefix + ".bin";
    String srecPath = outPrefix + ".srec";
    writeBin(binPath, image);
    int records = writeSrec(srecPath, image, FLASH_BASE, 32);

    println(String.format("Wrote %s (%d bytes, padded with 0x%02x)", binPath, image.length, PAD_BYTE));
    println(String.format("Wrote %s (%d S3 data records)", srecPath, records));
    println(String.format("Header CRC at 0x%x = 0x%08x over [0x%x, 0x%x); verify=0x%08x", crcOffset, crc, crcStart, crcEnd, verify));
  }

  private Map<String, List<String>> parseArgs(String[] scriptArgs) {
    Map<String, List<String>> parsed = new LinkedHashMap<>();
    for (String arg : scriptArgs) {
      int eq = arg.indexOf('=');
      if (eq < 0) {
        continue;
      }
      String key = arg.substring(0, eq).trim().toLowerCase();
      String value = arg.substring(eq + 1).trim();
      parsed.computeIfAbsent(key, unused -> new ArrayList<>()).add(value);
    }
    return parsed;
  }

  private String firstArg(Map<String, List<String>> args, String key, String defaultValue) {
    List<String> values = args.get(key);
    return values == null || values.isEmpty() ? defaultValue : values.get(values.size() - 1);
  }

  private int parseIntegerArg(String value) {
    String trimmed = value.trim().toLowerCase();
    if (trimmed.startsWith("0x")) {
      return (int) Long.parseLong(trimmed.substring(2), 16);
    }
    return Integer.parseInt(trimmed);
  }

  private byte[] parseHex(String hex) {
    String clean = hex.replace(" ", "").replace("_", "").trim();
    if ((clean.length() & 1) != 0) {
      throw new IllegalArgumentException("hex byte string has odd length: " + hex);
    }
    byte[] out = new byte[clean.length() / 2];
    for (int i = 0; i < out.length; i++) {
      out[i] = (byte) Integer.parseInt(clean.substring(i * 2, i * 2 + 2), 16);
    }
    return out;
  }

  private String toHex(byte[] bytes) {
    StringBuilder out = new StringBuilder(bytes.length * 2);
    for (byte value : bytes) {
      out.append(String.format("%02x", value & 0xff));
    }
    return out.toString();
  }

  private Address fileAddr(int offset) {
    return currentProgram.getAddressFactory().getDefaultAddressSpace().getAddress(FLASH_BASE + (long) offset);
  }

  private byte[] readImage(int size) throws Exception {
    byte[] data = new byte[size];
    Arrays.fill(data, (byte) PAD_BYTE);
    Memory memory = currentProgram.getMemory();
    for (int off = 0; off < size; off++) {
      Address addr = fileAddr(off);
      if (memory.contains(addr)) {
        data[off] = getByte(addr);
      }
      monitor.checkCancelled();
    }
    return data;
  }

  private void writeBytes(int offset, byte[] data) throws Exception {
    for (int i = 0; i < data.length; i++) {
      setByte(fileAddr(offset + i), data[i]);
    }
  }

  private int crc32Ieee(byte[] data, int start, int end) {
    int crc = 0xffffffff;
    for (int i = start; i < end; i++) {
      crc ^= data[i] & 0xff;
      for (int bit = 0; bit < 8; bit++) {
        if ((crc & 1) != 0) {
          crc = (crc >>> 1) ^ 0xedb88320;
        } else {
          crc >>>= 1;
        }
      }
    }
    return crc ^ 0xffffffff;
  }

  private void writeBin(String path, byte[] image) throws IOException {
    try (FileOutputStream out = new FileOutputStream(path)) {
      out.write(image);
    }
  }

  private int writeSrec(String path, byte[] image, long base, int chunk) throws IOException {
    int count = 0;
    try (BufferedWriter out = new BufferedWriter(new FileWriter(path))) {
      out.write(srecLine(0, 0, "MCU1PATCH".getBytes("US-ASCII")));
      out.newLine();
      for (int off = 0; off < image.length; off += chunk) {
        int len = Math.min(chunk, image.length - off);
        byte[] payload = Arrays.copyOfRange(image, off, off + len);
        out.write(srecLine(3, base + off, payload));
        out.newLine();
        count++;
      }
      out.write(srecLine(7, base, new byte[0]));
      out.newLine();
    }
    return count;
  }

  private String srecLine(int recordType, long address, byte[] payload) {
    int addrLen;
    if (recordType == 0 || recordType == 1 || recordType == 9) {
      addrLen = 2;
    } else if (recordType == 2 || recordType == 8) {
      addrLen = 3;
    } else {
      addrLen = 4;
    }

    byte[] addressBytes = new byte[addrLen];
    for (int i = 0; i < addrLen; i++) {
      int shift = 8 * (addrLen - 1 - i);
      addressBytes[i] = (byte) ((address >>> shift) & 0xff);
    }

    int count = addrLen + payload.length + 1;
    int sum = count;
    StringBuilder out = new StringBuilder();
    out.append('S').append(recordType).append(String.format("%02X", count));
    for (byte value : addressBytes) {
      int unsigned = value & 0xff;
      sum += unsigned;
      out.append(String.format("%02X", unsigned));
    }
    for (byte value : payload) {
      int unsigned = value & 0xff;
      sum += unsigned;
      out.append(String.format("%02X", unsigned));
    }
    out.append(String.format("%02X", (~sum) & 0xff));
    return out.toString();
  }
}
