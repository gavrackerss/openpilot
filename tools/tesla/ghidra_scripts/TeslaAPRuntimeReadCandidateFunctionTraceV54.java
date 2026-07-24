// Try multiple candidate function starts around the AP runtime-read direct-reference
// bytes when Ghidra cannot place 0x6f549 inside a function from 0x6f520.
//
// This changes only Ghidra analysis metadata (disassembly/function definitions),
// never firmware bytes.
//
// @category Tesla.Trace
// @author OpenAI

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.address.AddressSpace;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionManager;
import ghidra.program.model.listing.Instruction;
import ghidra.program.model.listing.Listing;
import ghidra.program.model.mem.Memory;
import ghidra.program.model.symbol.Reference;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

public class TeslaAPRuntimeReadCandidateFunctionTraceV54 extends GhidraScript {

    private static final long DIRECT_REF_ADDRESS = 0x0006f549L;
    private static final long TRACE_START = 0x0006f500L;
    private static final long TRACE_END = 0x0006f590L;

    private static final long[] CANDIDATE_STARTS = new long[] {
        0x0006f548L,
        0x0006f546L,
        0x0006f544L,
        0x0006f542L,
        0x0006f540L,
        0x0006f53cL,
        0x0006f538L,
        0x0006f530L,
        0x0006f520L,
        0x0006f500L
    };

    private Listing listing;
    private Memory memory;
    private AddressSpace addressSpace;
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
        functionManager = currentProgram.getFunctionManager();

        File outputDirectory = askDirectory("Choose V54 trace output directory", "Trace");
        File reportFile = new File(outputDirectory, "Tesla_APRuntimeReadCandidateFunctionTraceV54.txt");
        writer = new BufferedWriter(new OutputStreamWriter(
            new FileOutputStream(reportFile), StandardCharsets.UTF_8));
        try {
            writeLine("TESLA AP RUNTIME-READ CANDIDATE FUNCTION TRACE V54");
            writeLine("====================================================");
            writeLine("Program: " + currentProgram.getName());
            writeLine(String.format(Locale.ROOT, "Direct AP ref address: 0x%08X", DIRECT_REF_ADDRESS));
            writeLine("");

            Function initial = functionManager.getFunctionContaining(offsetAddress(DIRECT_REF_ADDRESS));
            writeLine("Initial containing function: " + functionName(initial));
            writeLine("");

            for (long candidate : CANDIDATE_STARTS) {
                tryCandidate(candidate);
                Function function = functionManager.getFunctionContaining(offsetAddress(DIRECT_REF_ADDRESS));
                if (function != null) {
                    writeLine("");
                    writeLine("SUCCESS: direct AP ref is now contained by " + functionName(function));
                    break;
                }
            }

            writeLine("");
            writeLine("Final containing function: " + functionName(functionManager.getFunctionContaining(offsetAddress(DIRECT_REF_ADDRESS))));
            writeLine("");
            writeLine("Instruction/raw window:");
            dumpInstructionOrBytesWindow();
        }
        finally {
            writer.close();
        }

        popup("V54 candidate function trace complete.\n\n" + reportFile.getAbsolutePath());
    }

    private void tryCandidate(long candidate) throws Exception {
        Address start = offsetAddress(candidate);
        writeLine(String.format(Locale.ROOT, "Candidate start 0x%08X", candidate));
        if (!memory.contains(start)) {
            writeLine("  not mapped");
            return;
        }
        if (listing.getInstructionAt(start) == null) {
            writeLine("  disassemble requested");
            disassemble(start);
        }
        Instruction instruction = listing.getInstructionAt(start);
        writeLine("  first instruction: " + (instruction == null ? "<none>" : instruction.toString()));

        Function existing = functionManager.getFunctionContaining(start);
        if (existing != null) {
            writeLine("  start already inside function: " + functionName(existing));
        }
        else {
            String name = String.format(Locale.ROOT, "trace_ap_runtime_candidate_%08x", candidate);
            writeLine("  createFunction " + name);
            createFunction(start, name);
        }

        Function containsDirect = functionManager.getFunctionContaining(offsetAddress(DIRECT_REF_ADDRESS));
        writeLine("  direct-ref containing function after candidate: " + functionName(containsDirect));
    }

    private void dumpInstructionOrBytesWindow() throws Exception {
        long cursor = TRACE_START;
        while (cursor <= TRACE_END) {
            Address address = offsetAddress(cursor);
            Instruction instruction = listing.getInstructionAt(address);
            if (instruction != null) {
                writeInstruction(instruction);
                cursor = Math.max(cursor + 1, instruction.getMaxAddress().getOffset() + 1);
            }
            else {
                writeLine(String.format(Locale.ROOT, "0x%08X  raw %s", cursor, rawBytes(cursor, 8)));
                cursor += 2;
            }
            if (monitor.isCancelled()) {
                break;
            }
        }
    }

    private void writeInstruction(Instruction instruction) throws Exception {
        long offset = instruction.getAddress().getOffset();
        String marker = (offset <= DIRECT_REF_ADDRESS && instruction.getMaxAddress().getOffset() >= DIRECT_REF_ADDRESS)
            ? "  <== contains raw 0x0870f0 ref"
            : "";
        writeLine(String.format(Locale.ROOT, "0x%08X%s  %s", offset, marker, instruction));
        for (Reference ref : instruction.getReferencesFrom()) {
            writeLine("    ref -> " + ref.getToAddress() + " type=" + ref.getReferenceType());
        }
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

    private String functionName(Function function) {
        return function == null ? "<none>" : function.getName() + " @ " + function.getEntryPoint() + " body=" + function.getBody();
    }

    private Address offsetAddress(long offset) {
        return addressSpace.getAddress(offset);
    }

    private void writeLine(String line) throws Exception {
        writer.write(line);
        writer.write("\r\n");
    }
}
