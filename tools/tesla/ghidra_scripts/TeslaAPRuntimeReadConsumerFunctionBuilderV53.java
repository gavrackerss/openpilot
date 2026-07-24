// Build/repair Ghidra analysis around the AP runtime-read consumer bytes near
// 0x6f520, then emit a focused linear instruction report.
//
// This script does not patch firmware bytes. It may create Ghidra analysis objects
// (instructions/functions) so later xrefs/callers can be recovered.
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

public class TeslaAPRuntimeReadConsumerFunctionBuilderV53 extends GhidraScript {

    private static final long SUSPECTED_FUNCTION_START = 0x0006f520L;
    private static final long DIRECT_REF_ADDRESS = 0x0006f549L;
    private static final long TRACE_START = 0x0006f480L;
    private static final long TRACE_END = 0x0006f680L;
    private static final String FUNCTION_NAME = "trace_ap_runtime_read_consumer_0006f520";

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

        Address functionStart = offsetAddress(SUSPECTED_FUNCTION_START);
        Address directRef = offsetAddress(DIRECT_REF_ADDRESS);

        println("Tesla AP runtime-read consumer function builder V53");
        ensureDisassembly(functionStart);
        Function function = functionManager.getFunctionContaining(directRef);
        if (function == null) {
            println("No containing function for 0x6f549; creating analysis function at 0x6f520.");
            createFunction(functionStart, FUNCTION_NAME);
            function = functionManager.getFunctionContaining(directRef);
        }

        File outputDirectory = askDirectory("Choose V53 trace output directory", "Trace");
        File reportFile = new File(outputDirectory, "Tesla_APRuntimeReadConsumerFunctionBuilderV53.txt");
        writer = new BufferedWriter(new OutputStreamWriter(
            new FileOutputStream(reportFile), StandardCharsets.UTF_8));
        try {
            writeLine("TESLA AP RUNTIME-READ CONSUMER FUNCTION BUILDER V53");
            writeLine("====================================================");
            writeLine("Program: " + currentProgram.getName());
            writeLine(String.format(Locale.ROOT, "Suspected function start: 0x%08X", SUSPECTED_FUNCTION_START));
            writeLine(String.format(Locale.ROOT, "Direct AP ref address:    0x%08X", DIRECT_REF_ADDRESS));
            writeLine("");
            if (function == null) {
                writeLine("Function status: still no function contains direct ref after createFunction attempt");
            }
            else {
                writeLine("Function status: " + function.getName() + " @ " + function.getEntryPoint());
                writeLine("Function body:   " + function.getBody());
            }
            writeLine("");
            writeLine("Linear instruction window:");
            dumpInstructionWindow();
        }
        finally {
            writer.close();
        }

        popup("V53 analysis trace complete.\n\n" + reportFile.getAbsolutePath());
    }

    private void ensureDisassembly(Address start) throws Exception {
        if (listing.getInstructionAt(start) == null) {
            println("No instruction at 0x6f520; asking Ghidra to disassemble from there.");
            disassemble(start);
        }
    }

    private void dumpInstructionWindow() throws Exception {
        Address cursor = offsetAddress(TRACE_START);
        Address end = offsetAddress(TRACE_END);
        int count = 0;
        while (cursor.compareTo(end) <= 0) {
            Instruction instruction = listing.getInstructionAt(cursor);
            if (instruction == null) {
                cursor = cursor.next();
                continue;
            }
            writeInstruction(instruction);
            cursor = instruction.getMaxAddress().next();
            count++;
            if (monitor.isCancelled()) {
                break;
            }
        }
        if (count == 0) {
            writeLine("  no instructions defined in window; check processor/language import settings");
            dumpBytes(TRACE_START, (int) (TRACE_END - TRACE_START));
        }
    }

    private void writeInstruction(Instruction instruction) throws Exception {
        StringBuilder line = new StringBuilder();
        long offset = instruction.getAddress().getOffset();
        line.append(String.format(Locale.ROOT, "0x%08X", offset));
        if (offset == DIRECT_REF_ADDRESS || (offset < DIRECT_REF_ADDRESS && instruction.getMaxAddress().getOffset() >= DIRECT_REF_ADDRESS)) {
            line.append("  <== contains raw 0x0870f0 ref");
        }
        line.append("  ").append(instruction.toString());
        writeLine(line.toString());
        for (Reference ref : instruction.getReferencesFrom()) {
            writeLine("    ref -> " + ref.getToAddress() + " type=" + ref.getReferenceType());
        }
    }

    private void dumpBytes(long start, int length) throws Exception {
        for (int row = 0; row < length; row += 16) {
            StringBuilder line = new StringBuilder();
            line.append(String.format(Locale.ROOT, "0x%08X:", start + row));
            for (int i = 0; i < 16 && row + i < length; i++) {
                Address address = offsetAddress(start + row + i);
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

    private Address offsetAddress(long offset) {
        return addressSpace.getAddress(offset);
    }

    private void writeLine(String line) throws Exception {
        writer.write(line);
        writer.write("\r\n");
    }
}
