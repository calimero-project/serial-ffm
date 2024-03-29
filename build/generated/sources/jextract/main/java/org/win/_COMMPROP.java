// Generated by jextract

package org.win;

import java.lang.invoke.*;
import java.lang.foreign.*;
import java.nio.ByteOrder;
import java.util.*;
import java.util.function.*;
import java.util.stream.*;

import static java.lang.foreign.ValueLayout.*;
import static java.lang.foreign.MemoryLayout.PathElement.*;

/**
 * {@snippet lang=c :
 * struct _COMMPROP {
 *     WORD wPacketLength;
 *     WORD wPacketVersion;
 *     DWORD dwServiceMask;
 *     DWORD dwReserved1;
 *     DWORD dwMaxTxQueue;
 *     DWORD dwMaxRxQueue;
 *     DWORD dwMaxBaud;
 *     DWORD dwProvSubType;
 *     DWORD dwProvCapabilities;
 *     DWORD dwSettableParams;
 *     DWORD dwSettableBaud;
 *     WORD wSettableData;
 *     WORD wSettableStopParity;
 *     DWORD dwCurrentTxQueue;
 *     DWORD dwCurrentRxQueue;
 *     DWORD dwProvSpec1;
 *     DWORD dwProvSpec2;
 *     WCHAR wcProvChar[1];
 * }
 * }
 */
public class _COMMPROP {

    _COMMPROP() {
        // Suppresses public default constructor, ensuring non-instantiability,
        // but allows generated subclasses in same package.
    }

    private static final GroupLayout $LAYOUT = MemoryLayout.structLayout(
        Windows.C_SHORT.withName("wPacketLength"),
        Windows.C_SHORT.withName("wPacketVersion"),
        Windows.C_LONG.withName("dwServiceMask"),
        Windows.C_LONG.withName("dwReserved1"),
        Windows.C_LONG.withName("dwMaxTxQueue"),
        Windows.C_LONG.withName("dwMaxRxQueue"),
        Windows.C_LONG.withName("dwMaxBaud"),
        Windows.C_LONG.withName("dwProvSubType"),
        Windows.C_LONG.withName("dwProvCapabilities"),
        Windows.C_LONG.withName("dwSettableParams"),
        Windows.C_LONG.withName("dwSettableBaud"),
        Windows.C_SHORT.withName("wSettableData"),
        Windows.C_SHORT.withName("wSettableStopParity"),
        Windows.C_LONG.withName("dwCurrentTxQueue"),
        Windows.C_LONG.withName("dwCurrentRxQueue"),
        Windows.C_LONG.withName("dwProvSpec1"),
        Windows.C_LONG.withName("dwProvSpec2"),
        MemoryLayout.sequenceLayout(1, Windows.C_SHORT).withName("wcProvChar"),
        MemoryLayout.paddingLayout(2)
    ).withName("_COMMPROP");

    public static final GroupLayout layout() {
        return $LAYOUT;
    }

    private static final long wPacketLength$OFFSET = 0;
    private static final OfShort wPacketLength$LAYOUT = (OfShort)$LAYOUT.select(groupElement("wPacketLength"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * WORD wPacketLength
     * }
     */
    public static short wPacketLength(MemorySegment struct) {
        return struct.get(wPacketLength$LAYOUT, wPacketLength$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * WORD wPacketLength
     * }
     */
    public static void wPacketLength(MemorySegment struct, short fieldValue) {
        struct.set(wPacketLength$LAYOUT, wPacketLength$OFFSET, fieldValue);
    }

    private static final long wPacketVersion$OFFSET = 2;
    private static final OfShort wPacketVersion$LAYOUT = (OfShort)$LAYOUT.select(groupElement("wPacketVersion"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * WORD wPacketVersion
     * }
     */
    public static short wPacketVersion(MemorySegment struct) {
        return struct.get(wPacketVersion$LAYOUT, wPacketVersion$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * WORD wPacketVersion
     * }
     */
    public static void wPacketVersion(MemorySegment struct, short fieldValue) {
        struct.set(wPacketVersion$LAYOUT, wPacketVersion$OFFSET, fieldValue);
    }

    private static final long dwServiceMask$OFFSET = 4;
    private static final OfInt dwServiceMask$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwServiceMask"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwServiceMask
     * }
     */
    public static int dwServiceMask(MemorySegment struct) {
        return struct.get(dwServiceMask$LAYOUT, dwServiceMask$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwServiceMask
     * }
     */
    public static void dwServiceMask(MemorySegment struct, int fieldValue) {
        struct.set(dwServiceMask$LAYOUT, dwServiceMask$OFFSET, fieldValue);
    }

    private static final long dwReserved1$OFFSET = 8;
    private static final OfInt dwReserved1$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwReserved1"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwReserved1
     * }
     */
    public static int dwReserved1(MemorySegment struct) {
        return struct.get(dwReserved1$LAYOUT, dwReserved1$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwReserved1
     * }
     */
    public static void dwReserved1(MemorySegment struct, int fieldValue) {
        struct.set(dwReserved1$LAYOUT, dwReserved1$OFFSET, fieldValue);
    }

    private static final long dwMaxTxQueue$OFFSET = 12;
    private static final OfInt dwMaxTxQueue$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwMaxTxQueue"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwMaxTxQueue
     * }
     */
    public static int dwMaxTxQueue(MemorySegment struct) {
        return struct.get(dwMaxTxQueue$LAYOUT, dwMaxTxQueue$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwMaxTxQueue
     * }
     */
    public static void dwMaxTxQueue(MemorySegment struct, int fieldValue) {
        struct.set(dwMaxTxQueue$LAYOUT, dwMaxTxQueue$OFFSET, fieldValue);
    }

    private static final long dwMaxRxQueue$OFFSET = 16;
    private static final OfInt dwMaxRxQueue$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwMaxRxQueue"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwMaxRxQueue
     * }
     */
    public static int dwMaxRxQueue(MemorySegment struct) {
        return struct.get(dwMaxRxQueue$LAYOUT, dwMaxRxQueue$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwMaxRxQueue
     * }
     */
    public static void dwMaxRxQueue(MemorySegment struct, int fieldValue) {
        struct.set(dwMaxRxQueue$LAYOUT, dwMaxRxQueue$OFFSET, fieldValue);
    }

    private static final long dwMaxBaud$OFFSET = 20;
    private static final OfInt dwMaxBaud$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwMaxBaud"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwMaxBaud
     * }
     */
    public static int dwMaxBaud(MemorySegment struct) {
        return struct.get(dwMaxBaud$LAYOUT, dwMaxBaud$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwMaxBaud
     * }
     */
    public static void dwMaxBaud(MemorySegment struct, int fieldValue) {
        struct.set(dwMaxBaud$LAYOUT, dwMaxBaud$OFFSET, fieldValue);
    }

    private static final long dwProvSubType$OFFSET = 24;
    private static final OfInt dwProvSubType$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwProvSubType"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwProvSubType
     * }
     */
    public static int dwProvSubType(MemorySegment struct) {
        return struct.get(dwProvSubType$LAYOUT, dwProvSubType$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwProvSubType
     * }
     */
    public static void dwProvSubType(MemorySegment struct, int fieldValue) {
        struct.set(dwProvSubType$LAYOUT, dwProvSubType$OFFSET, fieldValue);
    }

    private static final long dwProvCapabilities$OFFSET = 28;
    private static final OfInt dwProvCapabilities$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwProvCapabilities"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwProvCapabilities
     * }
     */
    public static int dwProvCapabilities(MemorySegment struct) {
        return struct.get(dwProvCapabilities$LAYOUT, dwProvCapabilities$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwProvCapabilities
     * }
     */
    public static void dwProvCapabilities(MemorySegment struct, int fieldValue) {
        struct.set(dwProvCapabilities$LAYOUT, dwProvCapabilities$OFFSET, fieldValue);
    }

    private static final long dwSettableParams$OFFSET = 32;
    private static final OfInt dwSettableParams$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwSettableParams"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwSettableParams
     * }
     */
    public static int dwSettableParams(MemorySegment struct) {
        return struct.get(dwSettableParams$LAYOUT, dwSettableParams$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwSettableParams
     * }
     */
    public static void dwSettableParams(MemorySegment struct, int fieldValue) {
        struct.set(dwSettableParams$LAYOUT, dwSettableParams$OFFSET, fieldValue);
    }

    private static final long dwSettableBaud$OFFSET = 36;
    private static final OfInt dwSettableBaud$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwSettableBaud"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwSettableBaud
     * }
     */
    public static int dwSettableBaud(MemorySegment struct) {
        return struct.get(dwSettableBaud$LAYOUT, dwSettableBaud$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwSettableBaud
     * }
     */
    public static void dwSettableBaud(MemorySegment struct, int fieldValue) {
        struct.set(dwSettableBaud$LAYOUT, dwSettableBaud$OFFSET, fieldValue);
    }

    private static final long wSettableData$OFFSET = 40;
    private static final OfShort wSettableData$LAYOUT = (OfShort)$LAYOUT.select(groupElement("wSettableData"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * WORD wSettableData
     * }
     */
    public static short wSettableData(MemorySegment struct) {
        return struct.get(wSettableData$LAYOUT, wSettableData$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * WORD wSettableData
     * }
     */
    public static void wSettableData(MemorySegment struct, short fieldValue) {
        struct.set(wSettableData$LAYOUT, wSettableData$OFFSET, fieldValue);
    }

    private static final long wSettableStopParity$OFFSET = 42;
    private static final OfShort wSettableStopParity$LAYOUT = (OfShort)$LAYOUT.select(groupElement("wSettableStopParity"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * WORD wSettableStopParity
     * }
     */
    public static short wSettableStopParity(MemorySegment struct) {
        return struct.get(wSettableStopParity$LAYOUT, wSettableStopParity$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * WORD wSettableStopParity
     * }
     */
    public static void wSettableStopParity(MemorySegment struct, short fieldValue) {
        struct.set(wSettableStopParity$LAYOUT, wSettableStopParity$OFFSET, fieldValue);
    }

    private static final long dwCurrentTxQueue$OFFSET = 44;
    private static final OfInt dwCurrentTxQueue$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwCurrentTxQueue"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwCurrentTxQueue
     * }
     */
    public static int dwCurrentTxQueue(MemorySegment struct) {
        return struct.get(dwCurrentTxQueue$LAYOUT, dwCurrentTxQueue$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwCurrentTxQueue
     * }
     */
    public static void dwCurrentTxQueue(MemorySegment struct, int fieldValue) {
        struct.set(dwCurrentTxQueue$LAYOUT, dwCurrentTxQueue$OFFSET, fieldValue);
    }

    private static final long dwCurrentRxQueue$OFFSET = 48;
    private static final OfInt dwCurrentRxQueue$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwCurrentRxQueue"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwCurrentRxQueue
     * }
     */
    public static int dwCurrentRxQueue(MemorySegment struct) {
        return struct.get(dwCurrentRxQueue$LAYOUT, dwCurrentRxQueue$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwCurrentRxQueue
     * }
     */
    public static void dwCurrentRxQueue(MemorySegment struct, int fieldValue) {
        struct.set(dwCurrentRxQueue$LAYOUT, dwCurrentRxQueue$OFFSET, fieldValue);
    }

    private static final long dwProvSpec1$OFFSET = 52;
    private static final OfInt dwProvSpec1$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwProvSpec1"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwProvSpec1
     * }
     */
    public static int dwProvSpec1(MemorySegment struct) {
        return struct.get(dwProvSpec1$LAYOUT, dwProvSpec1$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwProvSpec1
     * }
     */
    public static void dwProvSpec1(MemorySegment struct, int fieldValue) {
        struct.set(dwProvSpec1$LAYOUT, dwProvSpec1$OFFSET, fieldValue);
    }

    private static final long dwProvSpec2$OFFSET = 56;
    private static final OfInt dwProvSpec2$LAYOUT = (OfInt)$LAYOUT.select(groupElement("dwProvSpec2"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD dwProvSpec2
     * }
     */
    public static int dwProvSpec2(MemorySegment struct) {
        return struct.get(dwProvSpec2$LAYOUT, dwProvSpec2$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD dwProvSpec2
     * }
     */
    public static void dwProvSpec2(MemorySegment struct, int fieldValue) {
        struct.set(dwProvSpec2$LAYOUT, dwProvSpec2$OFFSET, fieldValue);
    }

    private static final long wcProvChar$OFFSET = 60;
    private static final long wcProvChar$SIZE = 2;

    /**
     * Getter for field:
     * {@snippet lang=c :
     * WCHAR wcProvChar[1]
     * }
     */
    public static MemorySegment wcProvChar(MemorySegment struct) {
        return struct.asSlice(wcProvChar$OFFSET, wcProvChar$SIZE);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * WCHAR wcProvChar[1]
     * }
     */
    public static void wcProvChar(MemorySegment struct, MemorySegment fieldValue) {
        MemorySegment.copy(fieldValue, 0L, struct, wcProvChar$OFFSET, wcProvChar$SIZE);
    }

    public static MemorySegment asSlice(MemorySegment array, long index) {
        return array.asSlice(layout().byteSize() * index);
    }

    public static long sizeof() { return layout().byteSize(); }

    public static MemorySegment allocate(SegmentAllocator allocator) {
        return allocator.allocate(layout());
    }

    public static MemorySegment allocateArray(long elementCount, SegmentAllocator allocator) {
        return allocator.allocate(MemoryLayout.sequenceLayout(elementCount, layout()));
    }

    public static MemorySegment reinterpret(MemorySegment addr, Arena arena, Consumer<MemorySegment> cleanup) {
        return reinterpret(addr, 1, arena, cleanup);
    }

    public static MemorySegment reinterpret(MemorySegment addr, long elementCount, Arena arena, Consumer<MemorySegment> cleanup) {
        return addr.reinterpret(layout().byteSize() * elementCount, arena, cleanup);
    }
}

