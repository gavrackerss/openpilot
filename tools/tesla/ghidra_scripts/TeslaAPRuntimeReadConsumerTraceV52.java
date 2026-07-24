// Trace the AP runtime-read callback consumer around the direct 0x0870f0
// reference at file/Ghidra offset 0x6f549.
//
// This is a read-only Ghidra report script. It does not patch bytes.
//
// @category Tesla.Trace
// @author OpenAI

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.address.AddressSpace;
import ghidra.program.model.listing.Data;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionManager;
import ghidra.program.model.listing.Instruction;
import ghidra.program.model.listing.InstructionIterator;
import ghidra.program.model.listing.Listing;
import ghidra.program.model.mem.Memory;
import ghidra.program.model.symbol.Reference;
import ghidra.program.model.symbol.ReferenceIterator;
import ghidra.program.model.symbol.ReferenceManager;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Set;

public class TeslaAPRuntimeReadConsumerTraceV52 extends GhidraScript {

    private static final long DIRECT_REF_ADDRESS = 0x0006f549L;
    private static final long CONTEXT_ADDRESS = 0x0006f520L;
    private static final long AP_READER_OFFSET = 0x000870f0L;

    private static final long[] INTERESTING_CALLBACKS = new long[] {
        0x00085b96L, // updateWhatYouSee
        0x00087834L, // forwardRadarHw
        0x000870f0L, // autopilot
        0x000865beL, // deliveryStatus
        0x0008657cL, // autopilotCameraType
        0x0008654eL  // mcuFPGAVersion
    };

    private static final String[] INTERESTING_NAMES = new String[] {
        "updateWhatYouSee",
        "forwardRadarHw",
        "autopilot",
        "deliveryStatus",
        "autopilotCameraType",
        "mcuFPGAVersion"
    };

    private Listing listing;
    private Memory memory;
    private AddressSpace addressSpace;
    private ReferenceManager referenceManager;
    private FunctionManager functionManager;
    private BufferedWriter writer;

    @Override
    protected void run() throws Exception {
        if (currentProgram == null) {
            popup("No program is open.");
            return;
        }

        listing = currentProgram.getListing();
        memory = currentProgram.getMemory();
        addressSpace = currentProgram.getAddressFactory().getDefaultAddressSpace();
        referenceManager = currentProgram.getReferenceManager();
        functionManager = currentProgram.getFunctionManager();

        File outputDirectory = askDirectory("Choose AP runtime-read trace output directory", "Trace");
        File reportFile = new File(outputDirectory, "Tesla_APRuntimeReadConsumerTraceV52.txt");

        writer = new BufferedWriter(new OutputStreamWriter(
            new FileOutputStream(reportFile), StandardCharsets.UTF_8));
        try {
            writeHeader();
            traceContainingFunction();
            traceRawReferences();
            traceInterestingCallbackRefs();
            traceNearbyStrings();
        }
        finally {
            writer.close();
        }

        println("AP runtime-read consumer trace complete: " + reportFile.getAbsolutePath());
        popup("AP runtime-read consumer trace complete.\n\n" + reportFile.getAbsolutePath());
    }

    private void writeHeader() throws Exception {
        writeLine("TESLA AP RUNTIME-READ CONSUMER TRACE V52");
        writeLine("========================================");
        writeLine("Program: " + currentProgram.getName());
        writeLine(String.format(Locale.ROOT, "Context address:      0x%08X", CONTEXT_ADDRESS));
        writeLine(String.format(Locale.ROOT, "Direct ref address:   0x%08X", DIRECT_REF_ADDRESS));
        writeLine(String.format(Locale.ROOT, "AP reader callback:   0x%08X", AP_READER_OFFSET));
        writeLine("");
        writeLine("Bytes around direct reference:");
        dumpBytes(CONTEXT_ADDRESS, 0x50);
        writeLine("");
    }

    private void traceContainingFunction() throws Exception {
        Address directAddress = toAddr(DIRECT_REF_ADDRESS);
        Function function = functionManager.getFunctionContaining(directAddress);
        writeLine("CONTAINING FUNCTION");
        writeLine("-------------------");
        if (function == null) {
            writeLine("No function contains 0x" + Long.toHexString(DIRECT_REF_ADDRESS));
            writeLine("");
            return;
        }

        writeFunctionSummary(function);
        writeLine("");
        writeLine("Callers of containing function:");
        traceCallers(function);
        writeLine("");
        writeLine("Callees / flow references from containing function:");
        traceCallees(function);
        writeLine("");
        writeLine("Suspicious instructions in containing function:");
        traceSuspiciousInstructions(function);
        writeLine("");
    }

    private void traceRawReferences() throws Exception {
        writeLine("RAW 24-BIT CALLBACK-OFFSET REFERENCES");
        writeLine("-------------------------------------");
        for (int i = 0; i < INTERESTING_CALLBACKS.length; i++) {
            byte[] pattern = low24Bytes(INTERESTING_CALLBACKS[i]);
            writeLine(String.format(Locale.ROOT, "%s 0x%06X pattern %s",
                INTERESTING_NAMES[i], INTERESTING_CALLBACKS[i], toHex(pattern)));
            int hits = scanMemoryForPattern(pattern, true);
            writeLine("  hits: " + hits);
        }
        writeLine("");
    }

    private void traceInterestingCallbackRefs() throws Exception {
        writeLine("GHIDRA REFERENCES TO INTERESTING CALLBACK ADDRESSES");
        writeLine("--------------------------------------------------");
        for (int i = 0; i < INTERESTING_CALLBACKS.length; i++) {
            Address callback = toAddr(INTERESTING_CALLBACKS[i]);
            writeLine(String.format(Locale.ROOT, "%s callback 0x%08X", INTERESTING_NAMES[i], INTERESTING_CALLBACKS[i]));
            ReferenceIterator refs = referenceManager.getReferencesTo(callback);
            int count = 0;
            while (refs.hasNext()) {
                Reference ref = refs.next();
                writeLine("  " + ref.getFromAddress() + " -> " + ref.getToAddress() + " " + ref.getReferenceType());
                count++;
            }
            if (count == 0) {
                writeLine("  no typed Ghidra refs");
            }
        }
        writeLine("");
    }

    private void traceNearbyStrings() throws Exception {
        writeLine("NEARBY DEFINED STRINGS / DATA");
        writeLine("-----------------------------");
        long min = Math.max(0, CONTEXT_ADDRESS - 0x8000);
        long max = CONTEXT_ADDRESS + 0x8000;
        int count = 0;
        for (Data data = listing.getDefinedDataAfter(toAddr(min)); data != null; data = listing.getDefinedDataAfter(data.getAddress())) {
            long offset = data.getAddress().getOffset();
            if (offset > max) {
                break;
            }
            Object value = data.getValue();
            if (value instanceof String) {
                writeLine(String.format(Locale.ROOT, "  0x%08X %s", offset, value));
                count++;
            }
            if (monitor.isCancelled()) {
                break;
            }
        }
        if (count == 0) {
            writeLine("  no defined strings in +/-0x8000 window");
        }
        writeLine("");
    }

    private void writeFunctionSummary(Function function) throws Exception {
        writeLine("Name:  " + function.getName());
        writeLine("Entry: " + function.getEntryPoint());
        writeLine("Body:  " + function.getBody());
    }

    private void traceCallers(Function function) throws Exception {
        ReferenceIterator refs = referenceManager.getReferencesTo(function.getEntryPoint());
        int count = 0;
        while (refs.hasNext()) {
            Reference ref = refs.next();
            Function caller = functionManager.getFunctionContaining(ref.getFromAddress());
            writeLine("  " + ref.getFromAddress() + " from " + functionName(caller) + " type=" + ref.getReferenceType());
            count++;
        }
        if (count == 0) {
            writeLine("  no callers found by typed references");
        }
    }

    private void traceCallees(Function function) throws Exception {
        Set<String> seen = new LinkedHashSet<>();
        InstructionIterator instructions = listing.getInstructions(function.getBody(), true);
        while (instructions.hasNext()) {
            Instruction instruction = instructions.next();
            for (Reference ref : instruction.getReferencesFrom()) {
                if (ref.getReferenceType().isCall() || ref.getReferenceType().isFlow()) {
                    Function callee = functionManager.getFunctionAt(ref.getToAddress());
                    String line = "  " + instruction.getAddress() + " " + instruction + " -> " +
                        ref.getToAddress() + " " + functionName(callee) + " type=" + ref.getReferenceType();
                    if (seen.add(line)) {
                        writeLine(line);
                    }
                }
            }
            if (monitor.isCancelled()) {
                break;
            }
        }
        if (seen.isEmpty()) {
            writeLine("  no call/flow refs found");
        }
    }

    private void traceSuspiciousInstructions(Function function) throws Exception {
        InstructionIterator instructions = listing.getInstructions(function.getBody(), true);
        int count = 0;
        while (instructions.hasNext()) {
            Instruction instruction = instructions.next();
            String text = instruction.toString().toLowerCase(Locale.ROOT);
            String mnemonic = instruction.getMnemonicString().toLowerCase(Locale.ROOT);
            if (isStore(mnemonic) || isCompare(mnemonic) || mentionsInterestingConstant(text)) {
                writeLine("  " + instruction.getAddress() + "  " + instruction);
                count++;
            }
            if (monitor.isCancelled()) {
                break;
            }
        }
        if (count == 0) {
            writeLine("  no suspicious stores/compares/constants found");
        }
    }

    private boolean isStore(String mnemonic) {
        return mnemonic.startsWith("st") || mnemonic.equals("stmw");
    }

    private boolean isCompare(String mnemonic) {
        return mnemonic.startsWith("cmp") || mnemonic.startsWith("cmpl") || mnemonic.equals("twi") || mnemonic.equals("tw");
    }

    private boolean mentionsInterestingConstant(String text) {
        return text.contains("870f0") || text.contains("86170") || text.contains("aa38") ||
               text.contains("7cac") || text.contains("9da4") || text.contains("9da0");
    }

    private int scanMemoryForPattern(byte[] pattern, boolean printHits) throws Exception {
        int hits = 0;
        Address min = memory.getMinAddress();
        Address max = memory.getMaxAddress();
        if (min == null || max == null) {
            return 0;
        }
        long start = min.getOffset();
        long end = max.getOffset() - pattern.length + 1;
        for (long offset = start; offset <= end; offset++) {
            if (matchesAt(offset, pattern)) {
                hits++;
                if (printHits) {
                    writeLine(String.format(Locale.ROOT, "  hit 0x%08X", offset));
                }
            }
            if ((offset & 0xffff) == 0) {
                monitor.setMessage(String.format(Locale.ROOT, "Scanning raw refs at 0x%08X", offset));
                if (monitor.isCancelled()) {
                    break;
                }
            }
        }
        return hits;
    }

    private boolean matchesAt(long offset, byte[] pattern) throws Exception {
        for (int i = 0; i < pattern.length; i++) {
            Address address = toAddr(offset + i);
            if (!memory.contains(address) || memory.getByte(address) != pattern[i]) {
                return false;
            }
        }
        return true;
    }

    private byte[] low24Bytes(long value) {
        return new byte[] {
            (byte) ((value >>> 16) & 0xff),
            (byte) ((value >>> 8) & 0xff),
            (byte) (value & 0xff)
        };
    }

    private void dumpBytes(long start, int length) throws Exception {
        for (int row = 0; row < length; row += 16) {
            StringBuilder line = new StringBuilder();
            line.append(String.format(Locale.ROOT, "0x%08X:", start + row));
            for (int i = 0; i < 16 && row + i < length; i++) {
                Address address = toAddr(start + row + i);
                if (memory.contains(address)) {
                    line.append(String.format(Locale.ROOT, " %02X", memory.getByte(address) & 0xff));
                }
                else {
                    line.append(" ??");
                }
            }
            writeLine(line.toString());
        }
    }

    private String functionName(Function function) {
        return function == null ? "<no function>" : function.getName() + "@" + function.getEntryPoint();
    }

    private Address toAddr(long offset) {
        return addressSpace.getAddress(offset);
    }

    private String toHex(byte[] bytes) {
        StringBuilder builder = new StringBuilder(bytes.length * 3);
        for (int i = 0; i < bytes.length; i++) {
            if (i != 0) {
                builder.append(' ');
            }
            int value = bytes[i] & 0xff;
            builder.append("0123456789ABCDEF".charAt((value >>> 4) & 0x0f));
            builder.append("0123456789ABCDEF".charAt(value & 0x0f));
        }
        return builder.toString();
    }

    private void writeLine(String line) throws Exception {
        writer.write(line);
        writer.write("\r\n");
    }
}
