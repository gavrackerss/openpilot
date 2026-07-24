// Export the V48 autopilot publication-split firmware as a complete 2 MiB
// MPC5668G CFLASH BIN and Motorola S-record file.
//
// Required patched state:
//   stored internal.dat autopilot = ASCII '0'
//   runtime AP hard-code = 2
//   0x4004AA38 publication copy receives AP0
//   0x40047CAC and 0x40049DA4 receive AP2
//
// S-record layout matches the existing PEmicro backups:
//   16 data bytes per record
//   S1 records through address 0xFFFF
//   S2 records above 0xFFFF
//   CRLF line endings
//   no S0/header or termination record
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
import java.util.zip.CRC32;

public class TeslaAutopilotPublicationSplitBinS19ExporterV49
        extends GhidraScript {

    private static final long START = 0x00000000L;
    private static final long END = 0x001FFFFFL;
    private static final int IMAGE_SIZE = 0x00200000;
    private static final int RECORD_DATA_LENGTH = 16;

    private static final long STORED_AP_ADDRESS = 0x0001CDBDL;
    private static final int STORED_AP_ZERO = 0x30;

    private static final long RUNTIME_AP_ADDRESS = 0x00087110L;
    private static final byte[] RUNTIME_AP_TWO =
        bytes(0x48, 0x20);

    private static final long SPLIT_LOAD_ADDRESS = 0x00087120L;
    private static final byte[] SPLIT_LOAD_BYTES =
        bytes(0x48, 0x00, 0x48, 0x23);

    private static final long COPY2_INSERT_ADDRESS = 0x0008712CL;
    private static final byte[] COPY2_INSERT_BYTES =
        bytes(0x74, 0x66, 0x2E, 0x34);

    private static final long COPY3_INSERT_ADDRESS = 0x00087130L;
    private static final byte[] COPY3_INSERT_BYTES =
        bytes(0x74, 0x6A, 0x1E, 0xB8);

    private static final long APP_HEADER_ADDRESS = 0x00020000L;
    private static final long APP_CRC_ADDRESS = 0x00020000L;
    private static final long APP_SIZE_ADDRESS = 0x00020004L;
    private static final long APP_SIZE_INV_ADDRESS = 0x00020008L;

    private static final long EXPECTED_APP_SIZE = 0x0012929AL;
    private static final long EXPECTED_APP_SIZE_INV = 0xFFED6D65L;

    private static final long COMPENSATION_ADDRESS = 0x00125800L;

    private Memory memory;
    private AddressSpace addressSpace;
    private int missingByteCount;

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

    @Override
    protected void run() throws Exception {
        if (currentProgram == null) {
            popup("No program is open.");
            return;
        }

        memory = currentProgram.getMemory();
        addressSpace =
            currentProgram
                .getAddressFactory()
                .getDefaultAddressSpace();

        println(
            "Tesla AP publication-split BIN/S19 exporter V49"
        );

        println(
            "Program: " +
            currentProgram.getName()
        );

        println(String.format(
            Locale.ROOT,
            "Range: 0x%08X-0x%08X",
            START,
            END
        ));

        byte[] runtimeAp =
            requireBytes(
                RUNTIME_AP_ADDRESS,
                RUNTIME_AP_TWO,
                "runtime autopilot=2 hard-code"
            );

        if (runtimeAp == null) {
            return;
        }

        byte[] splitLoad =
            requireBytes(
                SPLIT_LOAD_ADDRESS,
                SPLIT_LOAD_BYTES,
                "AP0/AP2 split load"
            );

        if (splitLoad == null) {
            return;
        }

        byte[] copy2 =
            requireBytes(
                COPY2_INSERT_ADDRESS,
                COPY2_INSERT_BYTES,
                "AP2 insert for 0x40047CAC"
            );

        if (copy2 == null) {
            return;
        }

        byte[] copy3 =
            requireBytes(
                COPY3_INSERT_ADDRESS,
                COPY3_INSERT_BYTES,
                "AP2 insert for 0x40049DA4"
            );

        if (copy3 == null) {
            return;
        }

        int storedAp =
            readUnsignedByte(
                STORED_AP_ADDRESS,
                false
            );

        if (storedAp != STORED_AP_ZERO) {
            popup(String.format(
                Locale.ROOT,
                "Export stopped: stored autopilot at 0x%08X is not ASCII '0'.\n" +
                "Expected: 30\nFound:    %02X",
                STORED_AP_ADDRESS,
                storedAp
            ));

            return;
        }

        long appSize =
            readU32BE(
                APP_SIZE_ADDRESS
            );

        long appSizeInv =
            readU32BE(
                APP_SIZE_INV_ADDRESS
            );

        if (appSize != EXPECTED_APP_SIZE ||
            appSizeInv != EXPECTED_APP_SIZE_INV ||
            appSizeInv !=
                (
                    (~appSize) &
                    0xFFFFFFFFL
                )) {

            popup(String.format(
                Locale.ROOT,
                "Export stopped: unexpected application header.\n\n" +
                "Size:     0x%08X\n" +
                "Inverse:  0x%08X",
                appSize,
                appSizeInv
            ));

            return;
        }

        long storedAppCrc =
            readU32BE(
                APP_CRC_ADDRESS
            );

        byte[] application =
            readRange(
                APP_HEADER_ADDRESS,
                (int)appSize,
                false
            );

        long calculatedAppCrc =
            calculateApplicationCrc(
                application
            );

        if (storedAppCrc != calculatedAppCrc) {
            popup(String.format(
                Locale.ROOT,
                "Export stopped: application CRC is not valid.\n\n" +
                "Stored:     0x%08X\n" +
                "Calculated: 0x%08X",
                storedAppCrc,
                calculatedAppCrc
            ));

            return;
        }

        byte[] compensation =
            readRange(
                COMPENSATION_ADDRESS,
                4,
                false
            );

        File outputDirectory =
            askDirectory(
                "Choose publication-split export directory",
                "Export"
            );

        File binFile =
            new File(
                outputDirectory,
                "Tesla_MCU1_AP2_Runtime_AP0_PublicationSplit_V49.bin"
            );

        File s19File =
            new File(
                outputDirectory,
                "Tesla_MCU1_AP2_Runtime_AP0_PublicationSplit_V49.S19"
            );

        File reportFile =
            new File(
                outputDirectory,
                "Tesla_MCU1_AP2_Runtime_AP0_PublicationSplit_V49_export_report.txt"
            );

        if ((binFile.exists() ||
             s19File.exists() ||
             reportFile.exists()) &&
            !askYesNo(
                "Overwrite existing exports?",
                "One or more output files already exist in:\n" +
                outputDirectory.getAbsolutePath() +
                "\n\nOverwrite them?"
            )) {

            println("Export cancelled.");
            return;
        }

        missingByteCount = 0;

        monitor.initialize(
            IMAGE_SIZE
        );

        byte[] image =
            readRange(
                START,
                IMAGE_SIZE,
                true
            );

        if (monitor.isCancelled()) {
            println("Export cancelled.");
            return;
        }

        writeBinary(
            binFile,
            image
        );

        writeS19(
            s19File,
            image
        );

        S19Validation validation =
            validateS19(
                s19File
            );

        String binSha256 =
            sha256(
                binFile
            );

        String s19Sha256 =
            sha256(
                s19File
            );

        writeReport(
            reportFile,
            binFile,
            s19File,
            validation,
            binSha256,
            s19Sha256,
            storedAppCrc,
            calculatedAppCrc,
            compensation,
            runtimeAp,
            splitLoad,
            copy2,
            copy3,
            storedAp
        );

        println("Export complete:");

        println(
            "  BIN:    " +
            binFile.getAbsolutePath()
        );

        println(
            "  S19:    " +
            s19File.getAbsolutePath()
        );

        println(
            "  Report: " +
            reportFile.getAbsolutePath()
        );

        println(
            "  BIN SHA-256: " +
            binSha256
        );

        println(
            "  S19 SHA-256: " +
            s19Sha256
        );

        println(
            "  S-record validation: " +
            (
                validation.valid
                    ? "PASS"
                    : "FAIL"
            )
        );

        popup(
            "Publication-split firmware export completed.\n\n" +
            "BIN: " +
            binFile.getName() +
            "\nS19: " +
            s19File.getName() +
            "\n\nS19 validation: " +
            (
                validation.valid
                    ? "PASS"
                    : "FAIL"
            ) +
            "\nRecords: " +
            validation.totalRecords +
            "\nApplication CRC: 0x" +
            String.format(
                Locale.ROOT,
                "%08X",
                storedAppCrc
            ) +
            "\nMissing Ghidra bytes filled with FF: " +
            missingByteCount +
            "\n\nReview the generated report before flashing."
        );
    }

    private byte[] requireBytes(
            long address,
            byte[] expected,
            String description)
            throws Exception {

        byte[] actual =
            readRange(
                address,
                expected.length,
                false
            );

        if (!equalsBytes(
                actual,
                expected)) {

            popup(String.format(
                Locale.ROOT,
                "Export stopped: %s is not present at 0x%08X.\n\n" +
                "Expected: %s\n" +
                "Found:    %s",
                description,
                address,
                toHex(expected),
                toHex(actual)
            ));

            return null;
        }

        return actual;
    }

    private byte[] readRange(
            long start,
            int length,
            boolean fillMissingWithFF)
            throws Exception {

        byte[] result =
            new byte[length];

        for (int index = 0;
                index < length;
                index++) {

            if ((index & 0x3FFF) == 0) {
                monitor.setProgress(index);

                monitor.setMessage(String.format(
                    Locale.ROOT,
                    "Reading CFLASH at 0x%08X",
                    start +
                    index
                ));

                if (monitor.isCancelled()) {
                    return result;
                }
            }

            Address address =
                addressSpace.getAddress(
                    start +
                    index
                );

            try {
                if (!memory.contains(address)) {
                    throw new Exception(
                        "Address is not mapped"
                    );
                }

                result[index] =
                    memory.getByte(address);
            }
            catch (Exception exception) {
                if (!fillMissingWithFF) {
                    throw new Exception(String.format(
                        Locale.ROOT,
                        "Cannot read required byte at 0x%08X",
                        start +
                        index
                    ), exception);
                }

                result[index] =
                    (byte)0xFF;

                missingByteCount++;
            }
        }

        return result;
    }

    private int readUnsignedByte(
            long address,
            boolean fillMissingWithFF)
            throws Exception {

        return readRange(
            address,
            1,
            fillMissingWithFF
        )[0] &
        0xFF;
    }

    private long readU32BE(
            long address)
            throws Exception {

        byte[] value =
            readRange(
                address,
                4,
                false
            );

        return
            ((long)(value[0] & 0xFF) << 24) |
            ((long)(value[1] & 0xFF) << 16) |
            ((long)(value[2] & 0xFF) << 8) |
            ((long)(value[3] & 0xFF));
    }

    private long calculateApplicationCrc(
            byte[] application) {

        byte[] working =
            application.clone();

        working[0] = 0;
        working[1] = 0;
        working[2] = 0;
        working[3] = 0;

        CRC32 crc =
            new CRC32();

        crc.update(
            working
        );

        return crc.getValue() &
            0xFFFFFFFFL;
    }

    private void writeBinary(
            File file,
            byte[] image)
            throws Exception {

        monitor.setMessage(
            "Writing raw 2 MiB binary"
        );

        BufferedOutputStream output =
            new BufferedOutputStream(
                new FileOutputStream(file)
            );

        try {
            output.write(image);
        }
        finally {
            output.close();
        }
    }

    private void writeS19(
            File file,
            byte[] image)
            throws Exception {

        monitor.setMessage(
            "Writing Motorola S-record file"
        );

        BufferedWriter writer =
            new BufferedWriter(
                new OutputStreamWriter(
                    new FileOutputStream(file),
                    StandardCharsets.US_ASCII
                )
            );

        try {
            for (int offset = 0;
                    offset < image.length;
                    offset += RECORD_DATA_LENGTH) {

                if ((offset & 0x3FFF) == 0) {
                    monitor.setProgress(offset);

                    monitor.setMessage(String.format(
                        Locale.ROOT,
                        "Writing S-record at 0x%08X",
                        offset
                    ));

                    if (monitor.isCancelled()) {
                        throw new Exception(
                            "Export cancelled while writing S19"
                        );
                    }
                }

                int dataLength =
                    Math.min(
                        RECORD_DATA_LENGTH,
                        image.length -
                        offset
                    );

                writer.write(
                    makeSRecord(
                        offset,
                        image,
                        offset,
                        dataLength
                    )
                );

                writer.write("\r\n");
            }
        }
        finally {
            writer.close();
        }
    }

    private String makeSRecord(
            int address,
            byte[] source,
            int sourceOffset,
            int dataLength) {

        int addressBytes =
            address <= 0xFFFF
                ? 2
                : 3;

        char recordType =
            addressBytes == 2
                ? '1'
                : '2';

        int count =
            addressBytes +
            dataLength +
            1;

        int sum = count;

        StringBuilder line =
            new StringBuilder(
                2 +
                2 +
                (
                    addressBytes *
                    2
                ) +
                (
                    dataLength *
                    2
                ) +
                2
            );

        line.append('S');
        line.append(recordType);

        appendHexByte(
            line,
            count
        );

        for (int shift =
                (
                    addressBytes -
                    1
                ) *
                8;
                shift >= 0;
                shift -= 8) {

            int value =
                (
                    address >>>
                    shift
                ) &
                0xFF;

            appendHexByte(
                line,
                value
            );

            sum += value;
        }

        for (int index = 0;
                index < dataLength;
                index++) {

            int value =
                source[
                    sourceOffset +
                    index
                ] &
                0xFF;

            appendHexByte(
                line,
                value
            );

            sum += value;
        }

        appendHexByte(
            line,
            (~sum) &
            0xFF
        );

        return line.toString();
    }

    private void appendHexByte(
            StringBuilder builder,
            int value) {

        final char[] hex =
            "0123456789ABCDEF"
                .toCharArray();

        builder.append(
            hex[
                (
                    value >>>
                    4
                ) &
                0x0F
            ]
        );

        builder.append(
            hex[
                value &
                0x0F
            ]
        );
    }

    private S19Validation validateS19(
            File file)
            throws Exception {

        S19Validation result =
            new S19Validation();

        result.valid = true;

        int expectedAddress = 0;

        BufferedReader reader =
            new BufferedReader(
                new InputStreamReader(
                    new FileInputStream(file),
                    StandardCharsets.US_ASCII
                )
            );

        try {
            String line;
            int lineNumber = 0;

            while ((line =
                    reader.readLine()) !=
                    null) {

                lineNumber++;

                if (line.length() == 0) {
                    continue;
                }

                if (line.length() < 10 ||
                    line.charAt(0) != 'S') {

                    result.fail(
                        "Invalid record syntax at line " +
                        lineNumber
                    );

                    break;
                }

                char type =
                    line.charAt(1);

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
                    result.fail(
                        "Unexpected record type S" +
                        type +
                        " at line " +
                        lineNumber
                    );

                    break;
                }

                int count =
                    parseHexByte(
                        line,
                        2
                    );

                if (line.length() !=
                    4 +
                    (
                        count *
                        2
                    )) {

                    result.fail(
                        "Record length mismatch at line " +
                        lineNumber
                    );

                    break;
                }

                int sum = count;
                int address = 0;
                int cursor = 4;

                for (int index = 0;
                        index < addressBytes;
                        index++) {

                    int value =
                        parseHexByte(
                            line,
                            cursor
                        );

                    cursor += 2;
                    sum += value;

                    address =
                        (
                            address <<
                            8
                        ) |
                        value;
                }

                int dataLength =
                    count -
                    addressBytes -
                    1;

                if (dataLength !=
                    RECORD_DATA_LENGTH) {

                    result.fail(
                        "Unexpected data length at line " +
                        lineNumber +
                        ": " +
                        dataLength
                    );

                    break;
                }

                if (address !=
                    expectedAddress) {

                    result.fail(String.format(
                        Locale.ROOT,
                        "Non-contiguous address at line %d: " +
                        "expected 0x%08X, found 0x%08X",
                        lineNumber,
                        expectedAddress,
                        address
                    ));

                    break;
                }

                for (int index = 0;
                        index < dataLength;
                        index++) {

                    int value =
                        parseHexByte(
                            line,
                            cursor
                        );

                    cursor += 2;
                    sum += value;
                }

                int checksum =
                    parseHexByte(
                        line,
                        cursor
                    );

                sum += checksum;

                if ((sum & 0xFF) !=
                    0xFF) {

                    result.fail(
                        "Checksum failure at line " +
                        lineNumber
                    );

                    break;
                }

                if ((address <= 0xFFFF &&
                     type != '1') ||
                    (address > 0xFFFF &&
                     type != '2')) {

                    result.fail(
                        "Wrong record type for address at line " +
                        lineNumber
                    );

                    break;
                }

                result.totalRecords++;
                result.dataBytes +=
                    dataLength;
                expectedAddress +=
                    dataLength;
            }
        }
        finally {
            reader.close();
        }

        if (result.valid &&
            result.dataBytes !=
                IMAGE_SIZE) {

            result.fail(
                "S19 data size is " +
                result.dataBytes +
                ", expected " +
                IMAGE_SIZE
            );
        }

        if (result.valid &&
            result.totalRecords !=
                IMAGE_SIZE /
                RECORD_DATA_LENGTH) {

            result.fail(
                "S19 record count is " +
                result.totalRecords +
                ", expected " +
                (
                    IMAGE_SIZE /
                    RECORD_DATA_LENGTH
                )
            );
        }

        if (result.valid &&
            result.s1Records !=
                0x10000 /
                RECORD_DATA_LENGTH) {

            result.fail(
                "Unexpected S1 record count: " +
                result.s1Records
            );
        }

        if (result.valid &&
            result.s2Records !=
                (
                    IMAGE_SIZE -
                    0x10000
                ) /
                RECORD_DATA_LENGTH) {

            result.fail(
                "Unexpected S2 record count: " +
                result.s2Records
            );
        }

        return result;
    }

    private int parseHexByte(
            String value,
            int offset) {

        return Integer.parseInt(
            value.substring(
                offset,
                offset +
                2
            ),
            16
        );
    }

    private String sha256(
            File file)
            throws Exception {

        MessageDigest digest =
            MessageDigest.getInstance(
                "SHA-256"
            );

        InputStream input =
            new BufferedInputStream(
                new FileInputStream(file)
            );

        try {
            byte[] buffer =
                new byte[65536];

            int read;

            while ((read =
                    input.read(buffer)) >=
                    0) {

                if (read > 0) {
                    digest.update(
                        buffer,
                        0,
                        read
                    );
                }
            }
        }
        finally {
            input.close();
        }

        return toHexCompact(
            digest.digest()
        );
    }

    private void writeReport(
            File reportFile,
            File binFile,
            File s19File,
            S19Validation validation,
            String binSha256,
            String s19Sha256,
            long storedAppCrc,
            long calculatedAppCrc,
            byte[] compensation,
            byte[] runtimeAp,
            byte[] splitLoad,
            byte[] copy2,
            byte[] copy3,
            int storedAp)
            throws Exception {

        BufferedWriter writer =
            new BufferedWriter(
                new OutputStreamWriter(
                    new FileOutputStream(
                        reportFile
                    ),
                    StandardCharsets.UTF_8
                )
            );

        try {
            writer.write(
                "TESLA MCU1 AP PUBLICATION-SPLIT EXPORT REPORT\r\n"
            );

            writer.write(
                "============================================\r\n\r\n"
            );

            writer.write(
                "Program: " +
                currentProgram.getName() +
                "\r\n"
            );

            writer.write(String.format(
                Locale.ROOT,
                "Export range: 0x%08X-0x%08X\r\n",
                START,
                END
            ));

            writer.write(
                "Image size: " +
                IMAGE_SIZE +
                " bytes (2 MiB)\r\n"
            );

            writer.write(
                "Missing/unreadable bytes filled with FF: " +
                missingByteCount +
                "\r\n\r\n"
            );

            writer.write(
                "Required publication-split checks\r\n"
            );

            writer.write(
                "---------------------------------\r\n"
            );

            writer.write(String.format(
                Locale.ROOT,
                "0x%08X stored autopilot: %02X ('%c') [PASS]\r\n",
                STORED_AP_ADDRESS,
                storedAp,
                (char)storedAp
            ));

            writer.write(String.format(
                Locale.ROOT,
                "0x%08X runtime AP2: %s [PASS]\r\n",
                RUNTIME_AP_ADDRESS,
                toHex(runtimeAp)
            ));

            writer.write(String.format(
                Locale.ROOT,
                "0x%08X AP0/AP2 split load: %s [PASS]\r\n",
                SPLIT_LOAD_ADDRESS,
                toHex(splitLoad)
            ));

            writer.write(String.format(
                Locale.ROOT,
                "0x%08X copy-2 AP2 insert: %s [PASS]\r\n",
                COPY2_INSERT_ADDRESS,
                toHex(copy2)
            ));

            writer.write(String.format(
                Locale.ROOT,
                "0x%08X copy-3 AP2 insert: %s [PASS]\r\n",
                COPY3_INSERT_ADDRESS,
                toHex(copy3)
            ));

            writer.write(String.format(
                Locale.ROOT,
                "0x%08X application CRC stored:     0x%08X\r\n",
                APP_CRC_ADDRESS,
                storedAppCrc
            ));

            writer.write(String.format(
                Locale.ROOT,
                "Application CRC calculated:        0x%08X [PASS]\r\n",
                calculatedAppCrc
            ));

            writer.write(String.format(
                Locale.ROOT,
                "0x%08X CRC compensation bytes: %s\r\n\r\n",
                COMPENSATION_ADDRESS,
                toHex(compensation)
            ));

            writer.write(
                "Resulting AP representations\r\n"
            );

            writer.write(
                "----------------------------\r\n"
            );

            writer.write(
                "0x4004AA38 AP bits = 0\r\n"
            );

            writer.write(
                "0x40047CAC AP bits = 2\r\n"
            );

            writer.write(
                "0x40049DA4 AP bits = 2\r\n\r\n"
            );

            writer.write(
                "Output files\r\n"
            );

            writer.write(
                "------------\r\n"
            );

            writer.write(
                "BIN: " +
                binFile.getAbsolutePath() +
                "\r\n"
            );

            writer.write(
                "BIN size: " +
                binFile.length() +
                "\r\n"
            );

            writer.write(
                "BIN SHA-256: " +
                binSha256 +
                "\r\n\r\n"
            );

            writer.write(
                "S19: " +
                s19File.getAbsolutePath() +
                "\r\n"
            );

            writer.write(
                "S19 size: " +
                s19File.length() +
                "\r\n"
            );

            writer.write(
                "S19 SHA-256: " +
                s19Sha256 +
                "\r\n\r\n"
            );

            writer.write(
                "Motorola S-record validation\r\n"
            );

            writer.write(
                "----------------------------\r\n"
            );

            writer.write(
                "Validation: " +
                (
                    validation.valid
                        ? "PASS"
                        : "FAIL"
                ) +
                "\r\n"
            );

            writer.write(
                "Total records: " +
                validation.totalRecords +
                "\r\n"
            );

            writer.write(
                "S1 records: " +
                validation.s1Records +
                "\r\n"
            );

            writer.write(
                "S2 records: " +
                validation.s2Records +
                "\r\n"
            );

            writer.write(
                "Data bytes: " +
                validation.dataBytes +
                "\r\n"
            );

            writer.write(
                "Data bytes per record: " +
                RECORD_DATA_LENGTH +
                "\r\n"
            );

            writer.write(
                "Line endings: CRLF\r\n"
            );

            writer.write(
                "Header/termination records: none, matching PEmicro backups\r\n"
            );

            if (!validation.valid) {
                writer.write(
                    "Failure: " +
                    validation.failureReason +
                    "\r\n"
                );
            }

            writer.write(
                "\r\nDIAGNOSTIC CAUTION\r\n"
            );

            writer.write(
                "------------------\r\n"
            );

            writer.write(
                "This is an experimental split-state firmware image.\r\n"
            );

            writer.write(
                "Test only on a bench or with the vehicle stationary.\r\n"
            );

            writer.write(
                "Keep the verified original full backup available for recovery.\r\n"
            );
        }
        finally {
            writer.close();
        }
    }

    private boolean equalsBytes(
            byte[] left,
            byte[] right) {

        if (left == null ||
            right == null ||
            left.length !=
                right.length) {

            return false;
        }

        for (int index = 0;
                index < left.length;
                index++) {

            if (left[index] !=
                right[index]) {

                return false;
            }
        }

        return true;
    }

    private String toHex(
            byte[] value) {

        StringBuilder builder =
            new StringBuilder(
                value.length *
                3
            );

        for (int index = 0;
                index < value.length;
                index++) {

            if (index != 0) {
                builder.append(' ');
            }

            appendHexByte(
                builder,
                value[index] &
                0xFF
            );
        }

        return builder.toString();
    }

    private String toHexCompact(
            byte[] value) {

        StringBuilder builder =
            new StringBuilder(
                value.length *
                2
            );

        for (byte item : value) {
            appendHexByte(
                builder,
                item &
                0xFF
            );
        }

        return builder
            .toString()
            .toLowerCase(
                Locale.ROOT
            );
    }

    private static byte[] bytes(
            int... values) {

        byte[] result =
            new byte[
                values.length
            ];

        for (int index = 0;
                index < values.length;
                index++) {

            result[index] =
                (byte)(
                    values[index] &
                    0xFF
                );
        }

        return result;
    }
}
