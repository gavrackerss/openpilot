// Classify the raw 0x6f500-0x6f590 region after V54 showed nonsensical
// disassembly and only a tiny candidate function at 0x6f548.
//
// This is read-only and treats the area as possible data/table/encoded VLE rather
// than assuming it is normal function code.
//
// @category Tesla.Trace
// @author OpenAI

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.address.AddressSpace;
import ghidra.program.model.mem.Memory;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

public class TeslaAPRuntimeReadRawRegionClassifierV55 extends GhidraScript {

    private static final long REGION_START = 0x0006f480L;
    private static final long REGION_END = 0x0006f680L;
    private static final long DIRECT_REF_ADDRESS = 0x0006f549L;

    private static final long[] IMPORTANT_VALUES = new long[] {
        0x000870f0L, 0x00086170L, 0x00085b96L, 0x00087834L,
        0x000865beL, 0x0008657cL, 0x0008654eL,
        0x0006f520L, 0x0006f548L, 0x0006f549L,
        0x0004aa38L, 0x00047cacL, 0x00049da4L, 0x00049da0L
    };

    private static final String[] IMPORTANT_NAMES = new String[] {
        "autopilot_read", "replacement_read", "updateWhatYouSee_read", "forwardRadarHw_read",
        "deliveryStatus_read", "autopilotCameraType_read", "mcuFPGAVersion_read",
        "region_6f520", "candidate_6f548", "direct_ref_6f549",
        "ap_word_4aa38", "ap_word_47cac", "ap_word_49da4", "ap_word_49da0"
    };

    private Memory memory;
    private AddressSpace addressSpace;
    private BufferedWriter writer;

    @Override
    protected void run() throws Exception {
        if (currentProgram == null) {
            popup("No program is open.");
            return;
        }

        memory = currentProgram.getMemory();
        addressSpace = currentProgram.getAddressFactory().getDefaultAddressSpace();

        File outputDirectory = askDirectory("Choose V55 raw classifier output directory", "Trace");
        File reportFile = new File(outputDirectory, "Tesla_APRuntimeReadRawRegionClassifierV55.txt");
        writer = new BufferedWriter(new OutputStreamWriter(
            new FileOutputStream(reportFile), StandardCharsets.UTF_8));
        try {
            writeLine("TESLA AP RUNTIME-READ RAW REGION CLASSIFIER V55");
            writeLine("================================================");
            writeLine("Program: " + currentProgram.getName());
            writeLine(String.format(Locale.ROOT, "Region: 0x%08X-0x%08X", REGION_START, REGION_END));
            writeLine(String.format(Locale.ROOT, "Direct raw AP ref: 0x%08X", DIRECT_REF_ADDRESS));
            writeLine("");
            dumpAlignedViews();
            scanImportantValuesInRegion();
            scanWholeImageForRegionPointers();
        }
        finally {
            writer.close();
        }

        popup("V55 raw region classifier complete.\n\n" + reportFile.getAbsolutePath());
    }

    private void dumpAlignedViews() throws Exception {
        writeLine("8-BYTE ROW VIEW");
        writeLine("---------------");
        for (long off = REGION_START; off <= REGION_END; off += 8) {
            writeLine(String.format(Locale.ROOT, "0x%08X  %s  w0=%08X w1=%08X  low24=%06X/%06X  %s",
                off, rawBytes(off, 8), readU32(off), readU32(off + 4), readU32(off) & 0xffffffL,
                readU32(off + 4) & 0xffffffL, annotations(off, 8)));
        }
        writeLine("");

        writeLine("16-BYTE ROW VIEW");
        writeLine("----------------");
        for (long off = REGION_START; off <= REGION_END; off += 16) {
            writeLine(String.format(Locale.ROOT, "0x%08X  %s  %s", off, rawBytes(off, 16), annotations(off, 16)));
        }
        writeLine("");
    }

    private void scanImportantValuesInRegion() throws Exception {
        writeLine("IMPORTANT VALUE HITS INSIDE REGION");
        writeLine("----------------------------------");
        for (int i = 0; i < IMPORTANT_VALUES.length; i++) {
            long value = IMPORTANT_VALUES[i];
            byte[] low24 = new byte[] {
                (byte) ((value >>> 16) & 0xff),
                (byte) ((value >>> 8) & 0xff),
                (byte) (value & 0xff)
            };
            byte[] full32 = new byte[] {
                (byte) ((value >>> 24) & 0xff),
                (byte) ((value >>> 16) & 0xff),
                (byte) ((value >>> 8) & 0xff),
                (byte) (value & 0xff)
            };
            writeLine(IMPORTANT_NAMES[i] + " 0x" + String.format(Locale.ROOT, "%08X", value));
            scanPattern(REGION_START, REGION_END, low24, "low24");
            scanPattern(REGION_START, REGION_END, full32, "u32");
        }
        writeLine("");
    }

    private void scanWholeImageForRegionPointers() throws Exception {
        writeLine("WHOLE-IMAGE RAW POINTERS TO REGION STARTS");
        writeLine("-----------------------------------------");
        long[] targets = new long[] {0x0006f520L, 0x0006f548L, 0x0006f549L};
        for (long target : targets) {
            byte[] full32 = new byte[] {
                (byte) ((target >>> 24) & 0xff),
                (byte) ((target >>> 16) & 0xff),
                (byte) ((target >>> 8) & 0xff),
                (byte) (target & 0xff)
            };
            byte[] low24 = new byte[] {
                (byte) ((target >>> 16) & 0xff),
                (byte) ((target >>> 8) & 0xff),
                (byte) (target & 0xff)
            };
            writeLine(String.format(Locale.ROOT, "target 0x%08X", target));
            scanPattern(memory.getMinAddress().getOffset(), memory.getMaxAddress().getOffset(), full32, "u32");
            scanPattern(memory.getMinAddress().getOffset(), memory.getMaxAddress().getOffset(), low24, "low24");
        }
    }

    private void scanPattern(long start, long end, byte[] pattern, String label) throws Exception {
        int hits = 0;
        long last = end - pattern.length + 1;
        for (long off = start; off <= last; off++) {
            if (matches(off, pattern)) {
                writeLine(String.format(Locale.ROOT, "  %s hit 0x%08X", label, off));
                hits++;
            }
            if ((off & 0xffff) == 0 && monitor.isCancelled()) {
                break;
            }
        }
        if (hits == 0) {
            writeLine("  " + label + " hits: 0");
        }
    }

    private String annotations(long start, int length) throws Exception {
        StringBuilder result = new StringBuilder();
        for (int i = 0; i < IMPORTANT_VALUES.length; i++) {
            byte[] low24 = new byte[] {
                (byte) ((IMPORTANT_VALUES[i] >>> 16) & 0xff),
                (byte) ((IMPORTANT_VALUES[i] >>> 8) & 0xff),
                (byte) (IMPORTANT_VALUES[i] & 0xff)
            };
            for (long off = start; off <= start + length - low24.length; off++) {
                if (matches(off, low24)) {
                    if (result.length() != 0) {
                        result.append(", ");
                    }
                    result.append(IMPORTANT_NAMES[i]).append("@+").append(Long.toHexString(off - start));
                }
            }
        }
        return result.length() == 0 ? "" : result.toString();
    }

    private boolean matches(long offset, byte[] pattern) throws Exception {
        for (int i = 0; i < pattern.length; i++) {
            Address address = offsetAddress(offset + i);
            if (!memory.contains(address) || memory.getByte(address) != pattern[i]) {
                return false;
            }
        }
        return true;
    }

    private long readU32(long offset) throws Exception {
        long value = 0;
        for (int i = 0; i < 4; i++) {
            Address address = offsetAddress(offset + i);
            value <<= 8;
            if (memory.contains(address)) {
                value |= memory.getByte(address) & 0xff;
            }
        }
        return value;
    }

    private String rawBytes(long start, int length) throws Exception {
        StringBuilder line = new StringBuilder();
        for (int i = 0; i < length; i++) {
            if (i != 0) {
                line.append(' ');
            }
            Address address = offsetAddress(start + i);
            if (memory.contains(address)) {
                line.append(String.format(Locale.ROOT, "%02X", memory.getByte(address) & 0xff));
            }
            else {
                line.append("??");
            }
        }
        return line.toString();
    }

    private Address offsetAddress(long offset) {
        return addressSpace.getAddress(offset);
    }

    private void writeLine(String line) throws Exception {
        writer.write(line);
        writer.write("\r\n");
    }
}
