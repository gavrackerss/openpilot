// TeslaToggleAutopilotPublicationSplitCRCNeutralV48.java
//
// Focused diagnostic toggle derived from V47.
//
// APPLY state:
//   stored internal.dat autopilot remains ASCII '0'
//   runtime hard-code at 0x87110 remains AP2
//   0x4004AA38 AP bits are written as 0
//   0x40047CAC AP bits are written as 2
//   0x40049DA4 AP bits are written as 2
//
// RESTORE state:
//   all three packed runtime words are written as AP2 again.
//
// The application CRC is preserved through the established four-byte
// compensation slot at 0x125800.
//
// Bench/stationary diagnostic use only.
//
// @category Tesla.ReverseEngineering
// @menupath Tools.Tesla.Toggle Autopilot Publication Split CRC Neutral V48

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.lang.Register;
import ghidra.program.model.mem.Memory;

import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.zip.CRC32;

public class TeslaToggleAutopilotPublicationSplitCRCNeutralV48
        extends GhidraScript {

    private static final long APP_START = 0x00020000L;
    private static final long APP_CRC_ADDR = 0x00020000L;
    private static final long APP_SIZE_ADDR = 0x00020004L;
    private static final long APP_SIZE_INV_ADDR = 0x00020008L;

    private static final long EXPECTED_APP_SIZE = 0x0012929AL;
    private static final long EXPECTED_APP_SIZE_INV = 0xFFED6D65L;

    private static final long PADDING_START = 0x00125086L;
    private static final long PADDING_END = 0x00125FFBL;
    private static final long COMP_ADDR = 0x00125800L;

    private static final long STORED_AP_ADDR = 0x0001CDBDL;
    private static final int STORED_AP_ZERO = 0x30;

    private static final long RUNTIME_AP_ADDR = 0x00087110L;
    private static final byte[] RUNTIME_AP_TWO =
        bytes(0x48, 0x20);

    /*
     * Original sequence:
     *
     *   0x87120  e_rlwinm r0,r0,0,29,31
     *   0x8712C  e_rlwimi r6,r0,5,24,26
     *   0x87130  e_rlwimi r10,r0,3,26,28
     *
     * Split sequence:
     *
     *   0x87120  se_li r0,0
     *   0x87122  se_li r3,2
     *   0x8712C  e_rlwimi r6,r3,5,24,26
     *   0x87130  e_rlwimi r10,r3,3,26,28
     *
     * The unchanged instruction at 0x87128 still uses r0, so it inserts AP0
     * into 0x4004AA38. The following two inserts use r3=2.
     */
    private static final PatchSite[] PATCH_SITES =
        new PatchSite[] {
            new PatchSite(
                0x00087120L,
                "load separate publication/runtime AP values",
                bytes(0x74, 0x00, 0x07, 0x7F),
                bytes(0x48, 0x00, 0x48, 0x23)
            ),
            new PatchSite(
                0x0008712CL,
                "insert AP2 into 0x40047CAC using r3",
                bytes(0x74, 0x06, 0x2E, 0x34),
                bytes(0x74, 0x66, 0x2E, 0x34)
            ),
            new PatchSite(
                0x00087130L,
                "insert AP2 into 0x40049DA4 using r3",
                bytes(0x74, 0x0A, 0x1E, 0xB8),
                bytes(0x74, 0x6A, 0x1E, 0xB8)
            )
        };

    private Memory memory;

    private static class PatchSite {
        final long address;
        final String description;
        final byte[] normal;
        final byte[] split;

        PatchSite(
                long address,
                String description,
                byte[] normal,
                byte[] split) {

            this.address = address;
            this.description = description;
            this.normal = normal;
            this.split = split;
        }
    }

    private static class Change {
        final PatchSite site;
        final byte[] before;
        final byte[] after;

        Change(
                PatchSite site,
                byte[] before,
                byte[] after) {

            this.site = site;
            this.before = before;
            this.after = after;
        }
    }

    @Override
    protected void run() throws Exception {
        println(
            "TeslaToggleAutopilotPublicationSplitCRCNeutralV48 loaded successfully."
        );

        if (currentProgram == null) {
            popup("No Ghidra program is open.");
            return;
        }

        memory = currentProgram.getMemory();

        List<String> failures =
            new ArrayList<String>();

        verifyRequiredBytes(failures);
        verifyHeaderAndCrc(failures);

        if (!failures.isEmpty()) {
            popup(
                "Publication-split toggle refused.\n\n" +
                joinFailures(failures)
            );
            return;
        }

        boolean normalState = true;
        boolean splitState = true;

        for (PatchSite site : PATCH_SITES) {
            byte[] actual =
                readBytes(
                    site.address,
                    site.normal.length
                );

            normalState &=
                same(actual, site.normal);

            splitState &=
                same(actual, site.split);
        }

        if (!normalState && !splitState) {
            popup(
                "The three patch sites are in a mixed or unknown state.\n\n" +
                stateSummary()
            );
            return;
        }

        boolean applying =
            normalState;

        List<Change> changes =
            new ArrayList<Change>();

        for (PatchSite site : PATCH_SITES) {
            byte[] before =
                readBytes(
                    site.address,
                    site.normal.length
                );

            byte[] after =
                applying
                    ? site.split
                    : site.normal;

            changes.add(
                new Change(
                    site,
                    before,
                    after
                )
            );
        }

        long appSize =
            readU32BE(
                APP_SIZE_ADDR
            );

        long storedCrc =
            readU32BE(
                APP_CRC_ADDR
            );

        byte[] currentImage =
            readBytes(
                APP_START,
                (int)appSize
            );

        verifyPaddingRun(currentImage);

        byte[] proposedImage =
            currentImage.clone();

        for (Change change : changes) {
            putBytes(
                proposedImage,
                change.site.address,
                change.after
            );
        }

        int compensationOffset =
            offsetInApplication(
                COMP_ADDR
            );

        byte[] compensationBefore =
            readBytes(
                COMP_ADDR,
                4
            );

        for (int index = 0;
                index < 4;
                index++) {

            proposedImage[
                compensationOffset +
                index
            ] = 0;
        }

        byte[] compensationAfter =
            solveFourBytePatch(
                proposedImage,
                compensationOffset,
                storedCrc
            );

        if (compensationAfter == null) {
            popup(
                "CRC compensation solver did not find a four-byte solution."
            );
            return;
        }

        System.arraycopy(
            compensationAfter,
            0,
            proposedImage,
            compensationOffset,
            4
        );

        long proposedCrc =
            calculateCrc(
                proposedImage
            );

        if (proposedCrc != storedCrc) {
            popup(String.format(
                Locale.ROOT,
                "Proposed patch is not CRC-neutral.\n\n" +
                "Required:   0x%08X\nCalculated: 0x%08X",
                storedCrc,
                proposedCrc
            ));
            return;
        }

        StringBuilder summary =
            new StringBuilder();

        summary.append(
            applying
                ? "APPLY publication split?\n\n"
                : "RESTORE all three AP runtime copies to 2?\n\n"
        );

        for (Change change : changes) {
            summary.append(
                String.format(
                    Locale.ROOT,
                    "0x%08X %-46s %s -> %s\n",
                    change.site.address,
                    change.site.description,
                    toHex(change.before),
                    toHex(change.after)
                )
            );
        }

        summary.append(
            "\nStored autopilot remains ASCII '0' at 0x1CDBD."
        );

        summary.append(
            "\nRuntime hard-code remains AP2 at 0x87110."
        );

        if (applying) {
            summary.append(
                "\n\nResulting writes:\n" +
                "  0x4004AA38 AP bits = 0\n" +
                "  0x40047CAC AP bits = 2\n" +
                "  0x40049DA4 AP bits = 2"
            );
        }
        else {
            summary.append(
                "\n\nResulting writes:\n" +
                "  all three packed AP representations = 2"
            );
        }

        summary.append(
            "\n\nCRC compensation:\n  " +
            toHex(compensationBefore) +
            " -> " +
            toHex(compensationAfter)
        );

        summary.append(String.format(
            Locale.ROOT,
            "\n\nStored/calculated application CRC remains 0x%08X.",
            storedCrc
        ));

        summary.append(
            "\n\nBench or stationary diagnostic use only."
        );

        if (!askYesNo(
                applying
                    ? "Apply AP publication split"
                    : "Restore normal AP runtime copies",
                summary.toString())) {

            println("Cancelled. Nothing was changed.");
            return;
        }

        Register vle =
            currentProgram
                .getProgramContext()
                .getRegister("VLE");

        if (vle == null) {
            vle =
                currentProgram
                    .getProgramContext()
                    .getRegister("vle");
        }

        if (vle == null) {
            popup(
                "No processor-context register named VLE/vle exists."
            );
            return;
        }

        int transaction =
            currentProgram.startTransaction(
                applying
                    ? "Apply AP publication split"
                    : "Restore AP publication copies"
            );

        boolean commit = false;

        try {
            for (Change change : changes) {
                Address start =
                    toAddr(
                        change.site.address
                    );

                Address end =
                    toAddr(
                        change.site.address +
                        change.after.length -
                        1
                    );

                clearListing(
                    start,
                    end
                );

                currentProgram
                    .getProgramContext()
                    .setValue(
                        vle,
                        start,
                        end,
                        BigInteger.ONE
                    );

                memory.setBytes(
                    start,
                    change.after
                );
            }

            Address compensationStart =
                toAddr(
                    COMP_ADDR
                );

            clearListing(
                compensationStart,
                toAddr(
                    COMP_ADDR +
                    3
                )
            );

            memory.setBytes(
                compensationStart,
                compensationAfter
            );

            memory.setBytes(
                toAddr(
                    APP_CRC_ADDR
                ),
                u32BE(
                    storedCrc
                )
            );

            for (Change change : changes) {
                disassemble(
                    toAddr(
                        change.site.address
                    )
                );
            }

            commit = true;
        }
        finally {
            currentProgram.endTransaction(
                transaction,
                commit
            );
        }

        if (!commit) {
            popup(
                "Patch transaction failed and was rolled back."
            );
            return;
        }

        List<String> verificationFailures =
            new ArrayList<String>();

        for (Change change : changes) {
            byte[] actual =
                readBytes(
                    change.site.address,
                    change.after.length
                );

            if (!same(
                    actual,
                    change.after)) {

                verificationFailures.add(
                    String.format(
                        Locale.ROOT,
                        "0x%08X expected %s, found %s.",
                        change.site.address,
                        toHex(change.after),
                        toHex(actual)
                    )
                );
            }
        }

        byte[] compensationActual =
            readBytes(
                COMP_ADDR,
                4
            );

        if (!same(
                compensationActual,
                compensationAfter)) {

            verificationFailures.add(
                "CRC compensation bytes did not verify."
            );
        }

        long finalCalculated =
            calculateCrc(
                readBytes(
                    APP_START,
                    (int)appSize
                )
            );

        long finalStored =
            readU32BE(
                APP_CRC_ADDR
            );

        if (finalCalculated != finalStored ||
                finalStored != storedCrc) {

            verificationFailures.add(
                String.format(
                    Locale.ROOT,
                    "Final CRC mismatch: stored 0x%08X, calculated 0x%08X, " +
                    "expected preserved 0x%08X.",
                    finalStored,
                    finalCalculated,
                    storedCrc
                )
            );
        }

        if (!verificationFailures.isEmpty()) {
            popup(
                "Patch was written but verification failed.\n\n" +
                joinFailures(
                    verificationFailures
                )
            );
            return;
        }

        popup(
            applying
                ? "AP publication split applied and CRC verified.\n\n" +
                  "0x4004AA38 will receive AP0 while the other two packed " +
                  "runtime representations receive AP2."
                : "Normal AP2 writes restored to all three packed runtime " +
                  "representations and CRC verified."
        );
    }

    private void verifyRequiredBytes(
            List<String> failures)
            throws Exception {

        byte[] runtime =
            readBytes(
                RUNTIME_AP_ADDR,
                RUNTIME_AP_TWO.length
            );

        if (!same(
                runtime,
                RUNTIME_AP_TWO)) {

            failures.add(
                "Runtime AP2 hard-code is missing at 0x87110. " +
                "Expected 48 20, found " +
                toHex(runtime) +
                "."
            );
        }

        int storedAp =
            memory.getByte(
                toAddr(
                    STORED_AP_ADDR
                )
            ) &
            0xFF;

        if (storedAp !=
                STORED_AP_ZERO) {

            failures.add(String.format(
                Locale.ROOT,
                "Stored autopilot must be ASCII '0' at 0x1CDBD. Found %02X.",
                storedAp
            ));
        }

        for (PatchSite site : PATCH_SITES) {
            byte[] actual =
                readBytes(
                    site.address,
                    site.normal.length
                );

            if (!same(actual, site.normal) &&
                    !same(actual, site.split)) {

                failures.add(
                    String.format(
                        Locale.ROOT,
                        "Unexpected bytes at 0x%08X %s: %s.",
                        site.address,
                        site.description,
                        toHex(actual)
                    )
                );
            }
        }
    }

    private void verifyHeaderAndCrc(
            List<String> failures)
            throws Exception {

        long appSize =
            readU32BE(
                APP_SIZE_ADDR
            );

        long appSizeInv =
            readU32BE(
                APP_SIZE_INV_ADDR
            );

        long storedCrc =
            readU32BE(
                APP_CRC_ADDR
            );

        if (appSize !=
                EXPECTED_APP_SIZE ||
            appSizeInv !=
                EXPECTED_APP_SIZE_INV ||
            appSizeInv !=
                (
                    (~appSize) &
                    0xFFFFFFFFL
                )) {

            failures.add(String.format(
                Locale.ROOT,
                "Unexpected application header: size 0x%08X, inverse 0x%08X.",
                appSize,
                appSizeInv
            ));

            return;
        }

        byte[] image =
            readBytes(
                APP_START,
                (int)appSize
            );

        long calculated =
            calculateCrc(
                image
            );

        if (storedCrc !=
                calculated) {

            failures.add(String.format(
                Locale.ROOT,
                "Application CRC mismatch before patching: " +
                "stored 0x%08X, calculated 0x%08X.",
                storedCrc,
                calculated
            ));
        }
    }

    private String stateSummary()
            throws Exception {

        StringBuilder result =
            new StringBuilder();

        for (PatchSite site : PATCH_SITES) {
            result.append(
                String.format(
                    Locale.ROOT,
                    "0x%08X %s\n",
                    site.address,
                    toHex(
                        readBytes(
                            site.address,
                            site.normal.length
                        )
                    )
                )
            );
        }

        return result.toString();
    }

    private void verifyPaddingRun(
            byte[] image) {

        int start =
            offsetInApplication(
                PADDING_START
            );

        int end =
            offsetInApplication(
                PADDING_END
            );

        int compensation =
            offsetInApplication(
                COMP_ADDR
            );

        for (int offset = start;
                offset <= end;
                offset++) {

            boolean isCompensation =
                offset >= compensation &&
                offset <
                    compensation +
                    4;

            if (!isCompensation &&
                    image[offset] != 0) {

                throw new IllegalStateException(
                    String.format(
                        Locale.ROOT,
                        "Expected zero padding at 0x%08X, found %02X.",
                        APP_START + offset,
                        image[offset] &
                        0xFF
                    )
                );
            }
        }
    }

    private void putBytes(
            byte[] image,
            long absoluteAddress,
            byte[] value) {

        int offset =
            offsetInApplication(
                absoluteAddress
            );

        System.arraycopy(
            value,
            0,
            image,
            offset,
            value.length
        );
    }

    private byte[] solveFourBytePatch(
            byte[] source,
            int offset,
            long target) {

        byte[] work =
            source.clone();

        work[0] = 0;
        work[1] = 0;
        work[2] = 0;
        work[3] = 0;

        for (int index = 0;
                index < 4;
                index++) {

            work[offset + index] = 0;
        }

        long base =
            rawCrc(work);

        long rhs =
            (
                target ^
                base
            ) &
            0xFFFFFFFFL;

        long[] basisVector =
            new long[32];

        int[] basisMask =
            new int[32];

        for (int variable = 0;
                variable < 32;
                variable++) {

            int byteIndex =
                variable /
                8;

            int bitInByte =
                7 -
                (
                    variable %
                    8
                );

            work[offset + byteIndex] =
                (byte)(
                    1 <<
                    bitInByte
                );

            long vector =
                (
                    rawCrc(work) ^
                    base
                ) &
                0xFFFFFFFFL;

            int mask =
                1 <<
                variable;

            work[offset + byteIndex] = 0;

            for (int bit = 31;
                    bit >= 0;
                    bit--) {

                long bitMask =
                    1L <<
                    bit;

                if ((vector & bitMask) ==
                        0) {

                    continue;
                }

                if (basisVector[bit] ==
                        0) {

                    basisVector[bit] =
                        vector;

                    basisMask[bit] =
                        mask;

                    vector = 0;
                    break;
                }

                vector ^=
                    basisVector[bit];

                mask ^=
                    basisMask[bit];
            }
        }

        long remaining = rhs;
        int solution = 0;

        for (int bit = 31;
                bit >= 0;
                bit--) {

            long bitMask =
                1L <<
                bit;

            if ((remaining & bitMask) ==
                    0) {

                continue;
            }

            if (basisVector[bit] ==
                    0) {

                return null;
            }

            remaining ^=
                basisVector[bit];

            solution ^=
                basisMask[bit];
        }

        if (remaining != 0) {
            return null;
        }

        byte[] patch =
            new byte[4];

        for (int variable = 0;
                variable < 32;
                variable++) {

            if ((solution &
                    (
                        1 <<
                        variable
                    )) ==
                    0) {

                continue;
            }

            int byteIndex =
                variable /
                8;

            int bitInByte =
                7 -
                (
                    variable %
                    8
                );

            patch[byteIndex] |=
                (byte)(
                    1 <<
                    bitInByte
                );
        }

        return patch;
    }

    private long calculateCrc(
            byte[] source) {

        byte[] image =
            source.clone();

        image[0] = 0;
        image[1] = 0;
        image[2] = 0;
        image[3] = 0;

        return rawCrc(
            image
        );
    }

    private long rawCrc(
            byte[] image) {

        CRC32 crc =
            new CRC32();

        crc.update(
            image
        );

        return crc.getValue() &
            0xFFFFFFFFL;
    }

    private int offsetInApplication(
            long absoluteAddress) {

        long offset =
            absoluteAddress -
            APP_START;

        if (offset < 0 ||
                offset >
                    Integer.MAX_VALUE) {

            throw new IllegalArgumentException(
                "Address outside application: 0x" +
                Long.toHexString(
                    absoluteAddress
                )
            );
        }

        return (int)offset;
    }

    private byte[] readBytes(
            long address,
            int length)
            throws Exception {

        byte[] result =
            new byte[length];

        int read =
            memory.getBytes(
                toAddr(
                    address
                ),
                result
            );

        if (read !=
                length) {

            throw new IllegalStateException(
                String.format(
                    Locale.ROOT,
                    "Read %d of %d bytes at 0x%08X.",
                    read,
                    length,
                    address
                )
            );
        }

        return result;
    }

    private long readU32BE(
            long address)
            throws Exception {

        byte[] value =
            readBytes(
                address,
                4
            );

        return
            ((long)(value[0] & 0xFF) << 24) |
            ((long)(value[1] & 0xFF) << 16) |
            ((long)(value[2] & 0xFF) << 8) |
            ((long)(value[3] & 0xFF));
    }

    private byte[] u32BE(
            long value) {

        return new byte[] {
            (byte)(
                (value >>> 24) &
                0xFF
            ),
            (byte)(
                (value >>> 16) &
                0xFF
            ),
            (byte)(
                (value >>> 8) &
                0xFF
            ),
            (byte)(
                value &
                0xFF
            )
        };
    }

    private static byte[] bytes(
            int... values) {

        byte[] result =
            new byte[
                values.length
            ];

        for (int index = 0;
                index <
                    values.length;
                index++) {

            result[index] =
                (byte)(
                    values[index] &
                    0xFF
                );
        }

        return result;
    }

    private boolean same(
            byte[] left,
            byte[] right) {

        if (left.length !=
                right.length) {

            return false;
        }

        for (int index = 0;
                index <
                    left.length;
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

        StringBuilder result =
            new StringBuilder();

        for (byte item : value) {
            if (result.length() != 0) {
                result.append(' ');
            }

            result.append(
                String.format(
                    Locale.ROOT,
                    "%02X",
                    item &
                    0xFF
                )
            );
        }

        return result.toString();
    }

    private String joinFailures(
            List<String> failures) {

        StringBuilder result =
            new StringBuilder();

        for (String failure : failures) {
            if (result.length() != 0) {
                result.append('\n');
            }

            result.append(
                "- "
            );

            result.append(
                failure
            );
        }

        return result.toString();
    }
}
