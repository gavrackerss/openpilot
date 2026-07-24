// Toggle the AP runtime-read reference experiment discovered near 0x6f520.
//
// This script patches the active Ghidra program in-place. Export afterward with
// TeslaAPRuntimeReadRefBinS19ExporterV51.java.
//
// @category Tesla.Patch
// @author OpenAI

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.address.AddressSpace;
import ghidra.program.model.mem.Memory;

import java.util.Locale;

public class TeslaAPRuntimeReadRefPatchV50 extends GhidraScript {

    private static final long DIRECT_REF_ADDRESS = 0x0006f549L;
    private static final byte[] DIRECT_REF_ORIGINAL = new byte[] {
        (byte) 0x08, (byte) 0x70, (byte) 0xf0
    };
    private static final byte[] DIRECT_REF_PATCHED = new byte[] {
        (byte) 0x08, (byte) 0x61, (byte) 0x70
    };

    private static final long DESCRIPTOR_READ_ADDRESS = 0x00024d5cL;
    private static final byte[] DESCRIPTOR_READ_ORIGINAL = new byte[] {
        (byte) 0x00, (byte) 0x08, (byte) 0x70, (byte) 0xf0
    };
    private static final byte[] DESCRIPTOR_READ_PATCHED = new byte[] {
        (byte) 0x00, (byte) 0x08, (byte) 0x61, (byte) 0x70
    };

    private static final long AP_READER_ADDRESS = 0x000870f0L;
    private static final long REPLACEMENT_READER_ADDRESS = 0x00086170L;

    private Memory memory;
    private AddressSpace addressSpace;

    @Override
    protected void run() throws Exception {
        if (currentProgram == null) {
            popup("No program is open.");
            return;
        }

        memory = currentProgram.getMemory();
        addressSpace = currentProgram.getAddressFactory().getDefaultAddressSpace();

        println("Tesla AP runtime-read reference patch V50");
        println(String.format(Locale.ROOT,
            "Direct AP callback reference: 0x%08X 0x%06X -> 0x%06X",
            DIRECT_REF_ADDRESS, AP_READER_ADDRESS, REPLACEMENT_READER_ADDRESS));
        println(String.format(Locale.ROOT,
            "Descriptor AP callback word:   0x%08X 0x%08X -> 0x%08X",
            DESCRIPTOR_READ_ADDRESS, AP_READER_ADDRESS, REPLACEMENT_READER_ADDRESS));

        byte[] direct = readRange(DIRECT_REF_ADDRESS, DIRECT_REF_ORIGINAL.length);
        byte[] descriptor = readRange(DESCRIPTOR_READ_ADDRESS, DESCRIPTOR_READ_ORIGINAL.length);

        boolean alreadyPatched = equalsBytes(direct, DIRECT_REF_PATCHED) && equalsBytes(descriptor, DESCRIPTOR_READ_PATCHED);
        boolean original = equalsBytes(direct, DIRECT_REF_ORIGINAL) && equalsBytes(descriptor, DESCRIPTOR_READ_ORIGINAL);

        if (!alreadyPatched && !original) {
            popup("Patch stopped: current bytes do not match either the original or V50 patched state.\n\n" +
                  String.format(Locale.ROOT, "Direct 0x%08X expected %s or %s, found %s\n",
                      DIRECT_REF_ADDRESS, toHex(DIRECT_REF_ORIGINAL), toHex(DIRECT_REF_PATCHED), toHex(direct)) +
                  String.format(Locale.ROOT, "Descriptor 0x%08X expected %s or %s, found %s",
                      DESCRIPTOR_READ_ADDRESS, toHex(DESCRIPTOR_READ_ORIGINAL), toHex(DESCRIPTOR_READ_PATCHED), toHex(descriptor)));
            return;
        }

        if (original) {
            popup("V50 apply path is disabled: the 0x6F549/0x24D5C runtime-read reference patch boot-failed on bench.\n\n" +
                  "No bytes were changed. Keep this script only to restore an already-patched Ghidra program back to the original bytes.");
            println("V50 apply path disabled after bench boot failure; no bytes changed.");
            return;
        }

        String action = "RESTORE original AP runtime-read references";
        if (!askYesNo(action, "This modifies the open Ghidra program in-place and reverts the boot-failing V50 bytes. Continue?")) {
            println("Restore cancelled.");
            return;
        }

        writeRange(DIRECT_REF_ADDRESS, DIRECT_REF_ORIGINAL);
        writeRange(DESCRIPTOR_READ_ADDRESS, DESCRIPTOR_READ_ORIGINAL);

        println(action + " complete.");
        println(String.format(Locale.ROOT, "0x%08X: %s", DIRECT_REF_ADDRESS,
            toHex(readRange(DIRECT_REF_ADDRESS, DIRECT_REF_ORIGINAL.length))));
        println(String.format(Locale.ROOT, "0x%08X: %s", DESCRIPTOR_READ_ADDRESS,
            toHex(readRange(DESCRIPTOR_READ_ADDRESS, DESCRIPTOR_READ_ORIGINAL.length))));
        popup(action + " complete. Export with the known-good full CFLASH exporter if needed.");
    }

    private byte[] readRange(long start, int length) throws Exception {
        byte[] result = new byte[length];
        for (int i = 0; i < length; i++) {
            Address address = addressSpace.getAddress(start + i);
            if (!memory.contains(address)) {
                throw new Exception(String.format(Locale.ROOT, "Address 0x%08X is not mapped", start + i));
            }
            result[i] = memory.getByte(address);
        }
        return result;
    }

    private void writeRange(long start, byte[] bytes) throws Exception {
        for (int i = 0; i < bytes.length; i++) {
            Address address = addressSpace.getAddress(start + i);
            if (!memory.contains(address)) {
                throw new Exception(String.format(Locale.ROOT, "Address 0x%08X is not mapped", start + i));
            }
            memory.setByte(address, bytes[i]);
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
            int value = bytes[i] & 0xff;
            builder.append("0123456789ABCDEF".charAt((value >>> 4) & 0x0f));
            builder.append("0123456789ABCDEF".charAt(value & 0x0f));
        }
        return builder.toString();
    }
}
