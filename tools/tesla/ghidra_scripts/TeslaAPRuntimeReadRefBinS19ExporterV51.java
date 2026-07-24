// Export the active Ghidra program as a complete 2 MiB MPC5668G CFLASH image
// and as a Motorola S-record file matching the existing PEmicro backup layout.
//
// @category Tesla.Export
// @author OpenAI

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.address.AddressSpace;
import ghidra.program.model.mem.Memory;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.Locale;

public class TeslaAPRuntimeReadRefBinS19ExporterV51 extends GhidraScript {

    private static final long START = 0x00000000L;
    private static final long END = 0x001fffffL;
    private static final int IMAGE_SIZE = 0x00200000;
    private static final int RECORD_DATA_LENGTH = 16;

    private static final long DIRECT_REF_ADDRESS = 0x0006f549L;
    private static final byte[] DIRECT_REF_PATCHED = new byte[] {
        (byte) 0x08, (byte) 0x61, (byte) 0x70
    };

    private static final long DESCRIPTOR_READ_ADDRESS = 0x00024d5cL;
    private static final byte[] DESCRIPTOR_READ_PATCHED = new byte[] {
        (byte) 0x00, (byte) 0x08, (byte) 0x61, (byte) 0x70
    };

    private static final long SHIM_ADDRESS = 0x0007722CL;
    private static final byte[] SHIM_FAILURE_ACK_BYTES = new byte[] {
        (byte) 0x48, (byte) 0x10
    };
    private static final byte[] SHIM_ORIGINAL_BYTES = new byte[] {
        (byte) 0x48, (byte) 0x00
    };

    private static final long CHECK_CALL_ADDRESS = 0x00077226L;
    private static final byte[] CHECK_CALL_BYTES = new byte[] {
        (byte) 0x78, (byte) 0x01, (byte) 0x11, (byte) 0x1B
    };

    private static final long RUNTIME_PATCH_ADDRESS = 0x00087110L;
    private static final byte[] RUNTIME_PATCH_BYTES = new byte[] {
        (byte) 0x48, (byte) 0x20
    };

    private static final long AUTOPILOT_VALUE_ADDRESS = 0x0001cdbdL;
    private static final int AUTOPILOT_VALUE_ZERO = 0x30;
    private static final int AUTOPILOT_VALUE_TWO = 0x32;

    private static final long APP_HEADER_ADDRESS = 0x00020000L;
    private static final long EXPECTED_APP_HEADER_CRC = 0x38c63335L;
    private static final byte[] EXPECTED_APP_HEADER_PREFIX = new byte[] {
        (byte) 0x38, (byte) 0xc6, (byte) 0x33, (byte) 0x35,
        (byte) 0x00, (byte) 0x12, (byte) 0x92, (byte) 0x9a,
        (byte) 0xff, (byte) 0xed, (byte) 0x6d, (byte) 0x65
    };

    private Memory memory;
    private AddressSpace addressSpace;
    private int missingByteCount;

    @Override
    protected void run() throws Exception {
        if (currentProgram == null) {
            popup("No program is open.");
            return;
        }

        memory = currentProgram.getMemory();
        addressSpace = currentProgram.getAddressFactory().getDefaultAddressSpace();

        println("Tesla AP runtime-read ref V51 CFLASH BIN/S19 exporter (V50 boot-fail guarded)");
        println("Program: " + currentProgram.getName());
        println(String.format(Locale.ROOT, "Range: 0x%08X-0x%08X", START, END));

        byte[] directRef = readRange(DIRECT_REF_ADDRESS, DIRECT_REF_PATCHED.length, false);
        byte[] descriptorRef = readRange(DESCRIPTOR_READ_ADDRESS, DESCRIPTOR_READ_PATCHED.length, false);
        if (equalsBytes(directRef, DIRECT_REF_PATCHED) || equalsBytes(descriptorRef, DESCRIPTOR_READ_PATCHED)) {
            popup("Export stopped: the V50 AP runtime-read reference patch is present, and that patch boot-failed on bench.\n" +
                  "Revert 0x6F549 and 0x24D5C before exporting a flashable file.\n\n" +
                  String.format(Locale.ROOT, "0x%08X: %s\n", DIRECT_REF_ADDRESS, toHex(directRef)) +
                  String.format(Locale.ROOT, "0x%08X: %s", DESCRIPTOR_READ_ADDRESS, toHex(descriptorRef)));
            return;
        }

        byte[] shim = readRange(SHIM_ADDRESS, SHIM_FAILURE_ACK_BYTES.length, false);
        if (!equalsBytes(shim, SHIM_FAILURE_ACK_BYTES) && !equalsBytes(shim, SHIM_ORIGINAL_BYTES)) {
            popup("Export stopped: the 0x7722C ACK shim bytes are not recognized.\n" +
                  "Expected one of: 48 10 or 48 00\n" +
                  "Found:           " + toHex(shim));
            return;
        }

        byte[] checkCall = readRange(
            CHECK_CALL_ADDRESS, CHECK_CALL_BYTES.length, false);
        if (!equalsBytes(checkCall, CHECK_CALL_BYTES)) {
            popup("Export stopped: the original 0x77226 check call is not present.\n" +
                  "Expected: 78 01 11 1B\n" +
                  "Found:    " + toHex(checkCall));
            return;
        }

        byte[] runtimePatch = readRange(
            RUNTIME_PATCH_ADDRESS, RUNTIME_PATCH_BYTES.length, false);
        if (!equalsBytes(runtimePatch, RUNTIME_PATCH_BYTES)) {
            popup("Export stopped: the runtime autopilot=2 patch is not present at 0x87110.\n" +
                  "Expected: 48 20\n" +
                  "Found:    " + toHex(runtimePatch));
            return;
        }

        byte[] appHeader = readRange(APP_HEADER_ADDRESS, EXPECTED_APP_HEADER_PREFIX.length, false);
        long appHeaderCrc = readUnsignedInt(APP_HEADER_ADDRESS, false);
        if (appHeaderCrc != EXPECTED_APP_HEADER_CRC) {
            popup(String.format(Locale.ROOT,
                "Export stopped: application header CRC word at 0x%08X is 0x%08X, expected 0x%08X.",
                APP_HEADER_ADDRESS, appHeaderCrc, EXPECTED_APP_HEADER_CRC));
            return;
        }
        if (!equalsBytes(appHeader, EXPECTED_APP_HEADER_PREFIX)) {
            popup("Export stopped: the gateway application header does not match this firmware build.\n" +
                  "Expected: " + toHex(EXPECTED_APP_HEADER_PREFIX) + "\n" +
                  "Found:    " + toHex(appHeader));
            return;
        }

        int autopilotValue = readUnsignedByte(AUTOPILOT_VALUE_ADDRESS, false);
        if (autopilotValue != AUTOPILOT_VALUE_ZERO &&
            autopilotValue != AUTOPILOT_VALUE_TWO) {
            popup(String.format(Locale.ROOT,
                "Export stopped: the stored autopilot byte at 0x%08X is neither ASCII '0' nor '2'.\n" +
                "Found: %02X",
                AUTOPILOT_VALUE_ADDRESS, autopilotValue));
            return;
        }

        File outputDirectory = askDirectory("Choose firmware export directory", "Export");
        File binFile = new File(outputDirectory, "Tesla_MCU1_APRuntimeReadRefV51.bin");
        File s19File = new File(outputDirectory, "Tesla_MCU1_APRuntimeReadRefV51.S19");
        File reportFile = new File(outputDirectory, "Tesla_MCU1_APRuntimeReadRefV51_export_report.txt");

        if ((binFile.exists() || s19File.exists() || reportFile.exists()) &&
            !askYesNo("Overwrite existing exports?",
                "One or more output files already exist in:\n" + outputDirectory.getAbsolutePath() +
                "\n\nOverwrite them?")) {
            println("Export cancelled.");
            return;
        }

        missingByteCount = 0;
        byte[] image = readRange(START, IMAGE_SIZE, true);
        if (monitor.isCancelled()) {
            println("Export cancelled.");
            return;
        }

        writeBinary(binFile, image);
        writeS19(s19File, image);

        S19Validation validation = validateS19(s19File);
        String binSha256 = sha256(binFile);
        String s19Sha256 = sha256(s19File);

        writeReport(reportFile, binFile, s19File, validation, binSha256, s19Sha256,
            directRef, descriptorRef, shim, appHeader, appHeaderCrc, autopilotValue);

        println("Export complete:");
        println("  BIN:    " + binFile.getAbsolutePath());
        println("  S19:    " + s19File.getAbsolutePath());
        println("  Report: " + reportFile.getAbsolutePath());
        println("  BIN SHA-256: " + binSha256);
        println("  S19 SHA-256: " + s19Sha256);
        println("  S-record validation: " + (validation.valid ? "PASS" : "FAIL"));

        popup("Firmware export completed.\n\n" +
              "S19 validation: " + (validation.valid ? "PASS" : "FAIL") + "\n" +
              "Records: " + validation.totalRecords + "\n" +
              "Missing Ghidra bytes filled with FF: " + missingByteCount + "\n\n" +
              "Review the generated report before flashing.");
    }

    private byte[] readRange(long start, int length, boolean fillMissingWithFF) throws Exception {
        byte[] result = new byte[length];
        for (int i = 0; i < length; i++) {
            if ((i & 0x3fff) == 0) {
                monitor.setProgress(i);
                monitor.setMessage(String.format(Locale.ROOT,
                    "Reading CFLASH at 0x%08X", start + i));
                if (monitor.isCancelled()) {
                    return result;
                }
            }

            Address address = addressSpace.getAddress(start + i);
            try {
                if (!memory.contains(address)) {
                    throw new Exception("Address is not mapped");
                }
                result[i] = memory.getByte(address);
            }
            catch (Exception ex) {
                if (!fillMissingWithFF) {
                    throw new Exception(String.format(Locale.ROOT,
                        "Cannot read required byte at 0x%08X", start + i), ex);
                }
                result[i] = (byte) 0xff;
                missingByteCount++;
            }
        }
        return result;
    }

    private int readUnsignedByte(long address, boolean fillMissingWithFF) throws Exception {
        return readRange(address, 1, fillMissingWithFF)[0] & 0xff;
    }

    private long readUnsignedInt(long address, boolean fillMissingWithFF) throws Exception {
        byte[] bytes = readRange(address, 4, fillMissingWithFF);
        return ((long) (bytes[0] & 0xff) << 24) |
               ((long) (bytes[1] & 0xff) << 16) |
               ((long) (bytes[2] & 0xff) << 8) |
               ((long) (bytes[3] & 0xff));
    }

    private void writeBinary(File file, byte[] image) throws Exception {
        monitor.setMessage("Writing raw 2 MiB binary");
        BufferedOutputStream output = new BufferedOutputStream(new FileOutputStream(file));
        try {
            output.write(image);
        }
        finally {
            output.close();
        }
    }

    private void writeS19(File file, byte[] image) throws Exception {
        monitor.setMessage("Writing Motorola S-record file");
        BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(
            new FileOutputStream(file), StandardCharsets.US_ASCII));
        try {
            for (int offset = 0; offset < image.length; offset += RECORD_DATA_LENGTH) {
                if ((offset & 0x3fff) == 0) {
                    monitor.setProgress(offset);
                    monitor.setMessage(String.format(Locale.ROOT,
                        "Writing S-record at 0x%08X", offset));
                    if (monitor.isCancelled()) {
                        throw new Exception("Export cancelled while writing S19");
                    }
                }

                int dataLength = Math.min(RECORD_DATA_LENGTH, image.length - offset);
                writer.write(makeSRecord(offset, image, offset, dataLength));
                writer.write("\r\n");
            }
        }
        finally {
            writer.close();
        }
    }

    private String makeSRecord(int address, byte[] source, int sourceOffset, int dataLength) {
        int addressBytes = address <= 0xffff ? 2 : 3;
        char recordType = addressBytes == 2 ? '1' : '2';
        int count = addressBytes + dataLength + 1;
        int sum = count;

        StringBuilder line = new StringBuilder(2 + 2 + (addressBytes * 2) + (dataLength * 2) + 2);
        line.append('S').append(recordType);
        appendHexByte(line, count);

        for (int shift = (addressBytes - 1) * 8; shift >= 0; shift -= 8) {
            int value = (address >>> shift) & 0xff;
            appendHexByte(line, value);
            sum += value;
        }

        for (int i = 0; i < dataLength; i++) {
            int value = source[sourceOffset + i] & 0xff;
            appendHexByte(line, value);
            sum += value;
        }

        appendHexByte(line, (~sum) & 0xff);
        return line.toString();
    }

    private void appendHexByte(StringBuilder builder, int value) {
        final char[] hex = "0123456789ABCDEF".toCharArray();
        builder.append(hex[(value >>> 4) & 0x0f]);
        builder.append(hex[value & 0x0f]);
    }

    private S19Validation validateS19(File file) throws Exception {
        S19Validation result = new S19Validation();
        result.valid = true;
        int expectedAddress = 0;

        BufferedReader reader = new BufferedReader(new InputStreamReader(
            new FileInputStream(file), StandardCharsets.US_ASCII));
        try {
            String line;
            int lineNumber = 0;
            while ((line = reader.readLine()) != null) {
                lineNumber++;
                if (line.length() == 0) {
                    continue;
                }

                if (line.length() < 10 || line.charAt(0) != 'S') {
                    result.fail("Invalid record syntax at line " + lineNumber);
                    break;
                }

                char type = line.charAt(1);
                int addressBytes;
                if (type == '1') {
                    addressBytes = 2;
                    result.s1Records++;
                }
                else if (type == '2') {
                    addressBytes = 3;
                    result.s2Records++;
                }
                else {
                    result.fail("Unexpected record type S" + type + " at line " + lineNumber);
                    break;
                }

                int count = parseHexByte(line, 2);
                if (line.length() != 4 + (count * 2)) {
                    result.fail("Record length mismatch at line " + lineNumber);
                    break;
                }

                int sum = count;
                int address = 0;
                int cursor = 4;
                for (int i = 0; i < addressBytes; i++) {
                    int value = parseHexByte(line, cursor);
                    cursor += 2;
                    sum += value;
                    address = (address << 8) | value;
                }

                int dataLength = count - addressBytes - 1;
                if (dataLength != RECORD_DATA_LENGTH) {
                    result.fail("Unexpected data length at line " + lineNumber + ": " + dataLength);
                    break;
                }

                if (address != expectedAddress) {
                    result.fail(String.format(Locale.ROOT,
                        "Non-contiguous address at line %d: expected 0x%08X, found 0x%08X",
                        lineNumber, expectedAddress, address));
                    break;
                }

                for (int i = 0; i < dataLength; i++) {
                    int value = parseHexByte(line, cursor);
                    cursor += 2;
                    sum += value;
                }

                int checksum = parseHexByte(line, cursor);
                sum += checksum;
                if ((sum & 0xff) != 0xff) {
                    result.fail("Checksum failure at line " + lineNumber);
                    break;
                }

                if ((address <= 0xffff && type != '1') ||
                    (address > 0xffff && type != '2')) {
                    result.fail("Wrong record type for address at line " + lineNumber);
                    break;
                }

                result.totalRecords++;
                result.dataBytes += dataLength;
                expectedAddress += dataLength;
            }
        }
        finally {
            reader.close();
        }

        if (result.valid && result.dataBytes != IMAGE_SIZE) {
            result.fail("S19 data size is " + result.dataBytes + ", expected " + IMAGE_SIZE);
        }
        if (result.valid && result.totalRecords != IMAGE_SIZE / RECORD_DATA_LENGTH) {
            result.fail("S19 record count is " + result.totalRecords + ", expected " +
                (IMAGE_SIZE / RECORD_DATA_LENGTH));
        }
        if (result.valid && result.s1Records != 0x10000 / RECORD_DATA_LENGTH) {
            result.fail("Unexpected S1 record count: " + result.s1Records);
        }
        if (result.valid && result.s2Records != (IMAGE_SIZE - 0x10000) / RECORD_DATA_LENGTH) {
            result.fail("Unexpected S2 record count: " + result.s2Records);
        }

        return result;
    }

    private int parseHexByte(String value, int offset) {
        return Integer.parseInt(value.substring(offset, offset + 2), 16);
    }

    private String sha256(File file) throws Exception {
        MessageDigest digest = MessageDigest.getInstance("SHA-256");
        InputStream input = new BufferedInputStream(new FileInputStream(file));
        try {
            byte[] buffer = new byte[65536];
            int read;
            while ((read = input.read(buffer)) >= 0) {
                if (read > 0) {
                    digest.update(buffer, 0, read);
                }
            }
        }
        finally {
            input.close();
        }
        return toHexCompact(digest.digest());
    }

    private void writeReport(File reportFile, File binFile, File s19File,
            S19Validation validation, String binSha256, String s19Sha256,
            byte[] directRef, byte[] descriptorRef, byte[] shim, byte[] appHeader, long appHeaderCrc, int autopilotValue) throws Exception {

        BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(
            new FileOutputStream(reportFile), StandardCharsets.UTF_8));
        try {
            writer.write("TESLA MCU1 AP RUNTIME-READ REF V51 EXPORT REPORT\r\n");
            writer.write("=================================================\r\n\r\n");
            writer.write("Program: " + currentProgram.getName() + "\r\n");
            writer.write(String.format(Locale.ROOT,
                "Export range: 0x%08X-0x%08X\r\n", START, END));
            writer.write("Image size: " + IMAGE_SIZE + " bytes (2 MiB)\r\n");
            writer.write("Missing/unreadable bytes filled with FF: " + missingByteCount + "\r\n\r\n");

            writer.write("Required patch checks\r\n");
            writer.write("---------------------\r\n");
            writer.write(String.format(Locale.ROOT,
                "0x%08X V50 direct AP runtime-read reference: %s [PASS]\r\n",
                DIRECT_REF_ADDRESS, toHex(directRef)));
            writer.write(String.format(Locale.ROOT,
                "0x%08X V50 AP descriptor runtime-read reference: %s [PASS]\r\n",
                DESCRIPTOR_READ_ADDRESS, toHex(descriptorRef)));
            writer.write(String.format(Locale.ROOT,
                "0x%08X ACK shim/original bytes: %s [PASS]\r\n",
                SHIM_ADDRESS, toHex(shim)));
            writer.write(String.format(Locale.ROOT,
                "0x%08X runtime autopilot patch: %s [PASS]\r\n",
                RUNTIME_PATCH_ADDRESS, toHex(RUNTIME_PATCH_BYTES)));
            writer.write(String.format(Locale.ROOT,
                "0x%08X autopilot value: %02X ('%c') [PASS]\r\n",
                AUTOPILOT_VALUE_ADDRESS, autopilotValue, (char) autopilotValue));
            writer.write(String.format(Locale.ROOT,
                "0x%08X application header CRC word: 0x%08X [PASS]\r\n",
                APP_HEADER_ADDRESS, appHeaderCrc));
            writer.write(String.format(Locale.ROOT,
                "0x%08X application header prefix: %s [PASS]\r\n\r\n",
                APP_HEADER_ADDRESS, toHex(appHeader)));

            writer.write("Output files\r\n");
            writer.write("------------\r\n");
            writer.write("BIN: " + binFile.getAbsolutePath() + "\r\n");
            writer.write("BIN size: " + binFile.length() + "\r\n");
            writer.write("BIN SHA-256: " + binSha256 + "\r\n\r\n");
            writer.write("S19: " + s19File.getAbsolutePath() + "\r\n");
            writer.write("S19 size: " + s19File.length() + "\r\n");
            writer.write("S19 SHA-256: " + s19Sha256 + "\r\n\r\n");

            writer.write("Motorola S-record validation\r\n");
            writer.write("----------------------------\r\n");
            writer.write("Validation: " + (validation.valid ? "PASS" : "FAIL") + "\r\n");
            writer.write("Total records: " + validation.totalRecords + "\r\n");
            writer.write("S1 records: " + validation.s1Records + "\r\n");
            writer.write("S2 records: " + validation.s2Records + "\r\n");
            writer.write("Data bytes: " + validation.dataBytes + "\r\n");
            writer.write("Data bytes per record: " + RECORD_DATA_LENGTH + "\r\n");
            writer.write("Line endings: CRLF\r\n");
            writer.write("Header/termination records: none, matching the supplied PEmicro backups\r\n");
            if (!validation.valid) {
                writer.write("Failure: " + validation.failureReason + "\r\n");
            }

            writer.write("\r\nFLASHING CAUTION\r\n");
            writer.write("-----------------\r\n");
            writer.write("The S-record checksum validation only proves record integrity and address coverage.\r\n");
            writer.write("It does not solve or update the gateway application's internal integrity mechanism.\r\n");
            writer.write("Keep the verified original full backup available for recovery and use bench power.\r\n");
        }
        finally {
            writer.close();
        }
    }

    private boolean equalsBytes(byte[] left, byte[] right) {
        if (left == null || right == null || left.length != right.length) {
            return false;
        }
        for (int i = 0; i < left.length; i++) {
            if (left[i] != right[i]) {
                return false;
            }
        }
        return true;
    }

    private String toHex(byte[] bytes) {
        StringBuilder builder = new StringBuilder(bytes.length * 3);
        for (int i = 0; i < bytes.length; i++) {
            if (i != 0) {
                builder.append(' ');
            }
            appendHexByte(builder, bytes[i] & 0xff);
        }
        return builder.toString();
    }

    private String toHexCompact(byte[] bytes) {
        StringBuilder builder = new StringBuilder(bytes.length * 2);
        for (int i = 0; i < bytes.length; i++) {
            appendHexByte(builder, bytes[i] & 0xff);
        }
        return builder.toString().toLowerCase(Locale.ROOT);
    }

    private static class S19Validation {
        boolean valid;
        int totalRecords;
        int s1Records;
        int s2Records;
        int dataBytes;
        String failureReason = "";

        void fail(String reason) {
            valid = false;
            failureReason = reason;
        }
    }
}
