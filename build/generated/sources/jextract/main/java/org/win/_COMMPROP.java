// Generated by jextract

package org.win;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
/**
 * {@snippet :
 * struct _COMMPROP {
 *     unsigned short wPacketLength;
 *     unsigned short wPacketVersion;
 *     unsigned long dwServiceMask;
 *     unsigned long dwReserved1;
 *     unsigned long dwMaxTxQueue;
 *     unsigned long dwMaxRxQueue;
 *     unsigned long dwMaxBaud;
 *     unsigned long dwProvSubType;
 *     unsigned long dwProvCapabilities;
 *     unsigned long dwSettableParams;
 *     unsigned long dwSettableBaud;
 *     unsigned short wSettableData;
 *     unsigned short wSettableStopParity;
 *     unsigned long dwCurrentTxQueue;
 *     unsigned long dwCurrentRxQueue;
 *     unsigned long dwProvSpec1;
 *     unsigned long dwProvSpec2;
 *     unsigned short wcProvChar[1];
 * };
 * }
 */
public class _COMMPROP {

    public static MemoryLayout $LAYOUT() {
        return constants$4.const$0;
    }
    public static VarHandle wPacketLength$VH() {
        return constants$4.const$1;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned short wPacketLength;
     * }
     */
    public static short wPacketLength$get(MemorySegment seg) {
        return (short)constants$4.const$1.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned short wPacketLength;
     * }
     */
    public static void wPacketLength$set(MemorySegment seg, short x) {
        constants$4.const$1.set(seg, 0L, x);
    }
    public static short wPacketLength$get(MemorySegment seg, long index) {
        return (short)constants$4.const$1.get(seg, index * sizeof());    }
    public static void wPacketLength$set(MemorySegment seg, long index, short x) {
        constants$4.const$1.set(seg, index * sizeof(), x);
    }
    public static VarHandle wPacketVersion$VH() {
        return constants$4.const$2;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned short wPacketVersion;
     * }
     */
    public static short wPacketVersion$get(MemorySegment seg) {
        return (short)constants$4.const$2.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned short wPacketVersion;
     * }
     */
    public static void wPacketVersion$set(MemorySegment seg, short x) {
        constants$4.const$2.set(seg, 0L, x);
    }
    public static short wPacketVersion$get(MemorySegment seg, long index) {
        return (short)constants$4.const$2.get(seg, index * sizeof());    }
    public static void wPacketVersion$set(MemorySegment seg, long index, short x) {
        constants$4.const$2.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwServiceMask$VH() {
        return constants$4.const$3;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwServiceMask;
     * }
     */
    public static int dwServiceMask$get(MemorySegment seg) {
        return (int)constants$4.const$3.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwServiceMask;
     * }
     */
    public static void dwServiceMask$set(MemorySegment seg, int x) {
        constants$4.const$3.set(seg, 0L, x);
    }
    public static int dwServiceMask$get(MemorySegment seg, long index) {
        return (int)constants$4.const$3.get(seg, index * sizeof());    }
    public static void dwServiceMask$set(MemorySegment seg, long index, int x) {
        constants$4.const$3.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwReserved1$VH() {
        return constants$4.const$4;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwReserved1;
     * }
     */
    public static int dwReserved1$get(MemorySegment seg) {
        return (int)constants$4.const$4.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwReserved1;
     * }
     */
    public static void dwReserved1$set(MemorySegment seg, int x) {
        constants$4.const$4.set(seg, 0L, x);
    }
    public static int dwReserved1$get(MemorySegment seg, long index) {
        return (int)constants$4.const$4.get(seg, index * sizeof());    }
    public static void dwReserved1$set(MemorySegment seg, long index, int x) {
        constants$4.const$4.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwMaxTxQueue$VH() {
        return constants$4.const$5;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwMaxTxQueue;
     * }
     */
    public static int dwMaxTxQueue$get(MemorySegment seg) {
        return (int)constants$4.const$5.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwMaxTxQueue;
     * }
     */
    public static void dwMaxTxQueue$set(MemorySegment seg, int x) {
        constants$4.const$5.set(seg, 0L, x);
    }
    public static int dwMaxTxQueue$get(MemorySegment seg, long index) {
        return (int)constants$4.const$5.get(seg, index * sizeof());    }
    public static void dwMaxTxQueue$set(MemorySegment seg, long index, int x) {
        constants$4.const$5.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwMaxRxQueue$VH() {
        return constants$5.const$0;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwMaxRxQueue;
     * }
     */
    public static int dwMaxRxQueue$get(MemorySegment seg) {
        return (int)constants$5.const$0.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwMaxRxQueue;
     * }
     */
    public static void dwMaxRxQueue$set(MemorySegment seg, int x) {
        constants$5.const$0.set(seg, 0L, x);
    }
    public static int dwMaxRxQueue$get(MemorySegment seg, long index) {
        return (int)constants$5.const$0.get(seg, index * sizeof());    }
    public static void dwMaxRxQueue$set(MemorySegment seg, long index, int x) {
        constants$5.const$0.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwMaxBaud$VH() {
        return constants$5.const$1;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwMaxBaud;
     * }
     */
    public static int dwMaxBaud$get(MemorySegment seg) {
        return (int)constants$5.const$1.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwMaxBaud;
     * }
     */
    public static void dwMaxBaud$set(MemorySegment seg, int x) {
        constants$5.const$1.set(seg, 0L, x);
    }
    public static int dwMaxBaud$get(MemorySegment seg, long index) {
        return (int)constants$5.const$1.get(seg, index * sizeof());    }
    public static void dwMaxBaud$set(MemorySegment seg, long index, int x) {
        constants$5.const$1.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwProvSubType$VH() {
        return constants$5.const$2;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwProvSubType;
     * }
     */
    public static int dwProvSubType$get(MemorySegment seg) {
        return (int)constants$5.const$2.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwProvSubType;
     * }
     */
    public static void dwProvSubType$set(MemorySegment seg, int x) {
        constants$5.const$2.set(seg, 0L, x);
    }
    public static int dwProvSubType$get(MemorySegment seg, long index) {
        return (int)constants$5.const$2.get(seg, index * sizeof());    }
    public static void dwProvSubType$set(MemorySegment seg, long index, int x) {
        constants$5.const$2.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwProvCapabilities$VH() {
        return constants$5.const$3;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwProvCapabilities;
     * }
     */
    public static int dwProvCapabilities$get(MemorySegment seg) {
        return (int)constants$5.const$3.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwProvCapabilities;
     * }
     */
    public static void dwProvCapabilities$set(MemorySegment seg, int x) {
        constants$5.const$3.set(seg, 0L, x);
    }
    public static int dwProvCapabilities$get(MemorySegment seg, long index) {
        return (int)constants$5.const$3.get(seg, index * sizeof());    }
    public static void dwProvCapabilities$set(MemorySegment seg, long index, int x) {
        constants$5.const$3.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwSettableParams$VH() {
        return constants$5.const$4;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwSettableParams;
     * }
     */
    public static int dwSettableParams$get(MemorySegment seg) {
        return (int)constants$5.const$4.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwSettableParams;
     * }
     */
    public static void dwSettableParams$set(MemorySegment seg, int x) {
        constants$5.const$4.set(seg, 0L, x);
    }
    public static int dwSettableParams$get(MemorySegment seg, long index) {
        return (int)constants$5.const$4.get(seg, index * sizeof());    }
    public static void dwSettableParams$set(MemorySegment seg, long index, int x) {
        constants$5.const$4.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwSettableBaud$VH() {
        return constants$5.const$5;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwSettableBaud;
     * }
     */
    public static int dwSettableBaud$get(MemorySegment seg) {
        return (int)constants$5.const$5.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwSettableBaud;
     * }
     */
    public static void dwSettableBaud$set(MemorySegment seg, int x) {
        constants$5.const$5.set(seg, 0L, x);
    }
    public static int dwSettableBaud$get(MemorySegment seg, long index) {
        return (int)constants$5.const$5.get(seg, index * sizeof());    }
    public static void dwSettableBaud$set(MemorySegment seg, long index, int x) {
        constants$5.const$5.set(seg, index * sizeof(), x);
    }
    public static VarHandle wSettableData$VH() {
        return constants$6.const$0;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned short wSettableData;
     * }
     */
    public static short wSettableData$get(MemorySegment seg) {
        return (short)constants$6.const$0.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned short wSettableData;
     * }
     */
    public static void wSettableData$set(MemorySegment seg, short x) {
        constants$6.const$0.set(seg, 0L, x);
    }
    public static short wSettableData$get(MemorySegment seg, long index) {
        return (short)constants$6.const$0.get(seg, index * sizeof());    }
    public static void wSettableData$set(MemorySegment seg, long index, short x) {
        constants$6.const$0.set(seg, index * sizeof(), x);
    }
    public static VarHandle wSettableStopParity$VH() {
        return constants$6.const$1;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned short wSettableStopParity;
     * }
     */
    public static short wSettableStopParity$get(MemorySegment seg) {
        return (short)constants$6.const$1.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned short wSettableStopParity;
     * }
     */
    public static void wSettableStopParity$set(MemorySegment seg, short x) {
        constants$6.const$1.set(seg, 0L, x);
    }
    public static short wSettableStopParity$get(MemorySegment seg, long index) {
        return (short)constants$6.const$1.get(seg, index * sizeof());    }
    public static void wSettableStopParity$set(MemorySegment seg, long index, short x) {
        constants$6.const$1.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwCurrentTxQueue$VH() {
        return constants$6.const$2;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwCurrentTxQueue;
     * }
     */
    public static int dwCurrentTxQueue$get(MemorySegment seg) {
        return (int)constants$6.const$2.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwCurrentTxQueue;
     * }
     */
    public static void dwCurrentTxQueue$set(MemorySegment seg, int x) {
        constants$6.const$2.set(seg, 0L, x);
    }
    public static int dwCurrentTxQueue$get(MemorySegment seg, long index) {
        return (int)constants$6.const$2.get(seg, index * sizeof());    }
    public static void dwCurrentTxQueue$set(MemorySegment seg, long index, int x) {
        constants$6.const$2.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwCurrentRxQueue$VH() {
        return constants$6.const$3;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwCurrentRxQueue;
     * }
     */
    public static int dwCurrentRxQueue$get(MemorySegment seg) {
        return (int)constants$6.const$3.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwCurrentRxQueue;
     * }
     */
    public static void dwCurrentRxQueue$set(MemorySegment seg, int x) {
        constants$6.const$3.set(seg, 0L, x);
    }
    public static int dwCurrentRxQueue$get(MemorySegment seg, long index) {
        return (int)constants$6.const$3.get(seg, index * sizeof());    }
    public static void dwCurrentRxQueue$set(MemorySegment seg, long index, int x) {
        constants$6.const$3.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwProvSpec1$VH() {
        return constants$6.const$4;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwProvSpec1;
     * }
     */
    public static int dwProvSpec1$get(MemorySegment seg) {
        return (int)constants$6.const$4.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwProvSpec1;
     * }
     */
    public static void dwProvSpec1$set(MemorySegment seg, int x) {
        constants$6.const$4.set(seg, 0L, x);
    }
    public static int dwProvSpec1$get(MemorySegment seg, long index) {
        return (int)constants$6.const$4.get(seg, index * sizeof());    }
    public static void dwProvSpec1$set(MemorySegment seg, long index, int x) {
        constants$6.const$4.set(seg, index * sizeof(), x);
    }
    public static VarHandle dwProvSpec2$VH() {
        return constants$6.const$5;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long dwProvSpec2;
     * }
     */
    public static int dwProvSpec2$get(MemorySegment seg) {
        return (int)constants$6.const$5.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long dwProvSpec2;
     * }
     */
    public static void dwProvSpec2$set(MemorySegment seg, int x) {
        constants$6.const$5.set(seg, 0L, x);
    }
    public static int dwProvSpec2$get(MemorySegment seg, long index) {
        return (int)constants$6.const$5.get(seg, index * sizeof());    }
    public static void dwProvSpec2$set(MemorySegment seg, long index, int x) {
        constants$6.const$5.set(seg, index * sizeof(), x);
    }
    public static MemorySegment wcProvChar$slice(MemorySegment seg) {
        return seg.asSlice(60, 2);
    }
    public static long sizeof() { return $LAYOUT().byteSize(); }
    public static MemorySegment allocate(SegmentAllocator allocator) { return allocator.allocate($LAYOUT()); }
    public static MemorySegment allocateArray(long len, SegmentAllocator allocator) {
        return allocator.allocate(MemoryLayout.sequenceLayout(len, $LAYOUT()));
    }
    public static MemorySegment ofAddress(MemorySegment addr, Arena scope) { return RuntimeHelper.asArray(addr, $LAYOUT(), 1, scope); }
}


