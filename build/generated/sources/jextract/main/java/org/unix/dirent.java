// Generated by jextract

package org.unix;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
/**
 * {@snippet :
 * struct dirent {
 *     unsigned long d_ino;
 *     long d_off;
 *     unsigned short d_reclen;
 *     unsigned char d_type;
 *     char d_name[256];
 * };
 * }
 */
public class dirent {

    public static MemoryLayout $LAYOUT() {
        return constants$9.const$3;
    }
    public static VarHandle d_ino$VH() {
        return constants$9.const$4;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned long d_ino;
     * }
     */
    public static long d_ino$get(MemorySegment seg) {
        return (long)constants$9.const$4.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned long d_ino;
     * }
     */
    public static void d_ino$set(MemorySegment seg, long x) {
        constants$9.const$4.set(seg, 0L, x);
    }
    public static long d_ino$get(MemorySegment seg, long index) {
        return (long)constants$9.const$4.get(seg, index * sizeof());    }
    public static void d_ino$set(MemorySegment seg, long index, long x) {
        constants$9.const$4.set(seg, index * sizeof(), x);
    }
    public static VarHandle d_off$VH() {
        return constants$9.const$5;
    }
    /**
     * Getter for field:
     * {@snippet :
     * long d_off;
     * }
     */
    public static long d_off$get(MemorySegment seg) {
        return (long)constants$9.const$5.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * long d_off;
     * }
     */
    public static void d_off$set(MemorySegment seg, long x) {
        constants$9.const$5.set(seg, 0L, x);
    }
    public static long d_off$get(MemorySegment seg, long index) {
        return (long)constants$9.const$5.get(seg, index * sizeof());    }
    public static void d_off$set(MemorySegment seg, long index, long x) {
        constants$9.const$5.set(seg, index * sizeof(), x);
    }
    public static VarHandle d_reclen$VH() {
        return constants$10.const$0;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned short d_reclen;
     * }
     */
    public static short d_reclen$get(MemorySegment seg) {
        return (short)constants$10.const$0.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned short d_reclen;
     * }
     */
    public static void d_reclen$set(MemorySegment seg, short x) {
        constants$10.const$0.set(seg, 0L, x);
    }
    public static short d_reclen$get(MemorySegment seg, long index) {
        return (short)constants$10.const$0.get(seg, index * sizeof());    }
    public static void d_reclen$set(MemorySegment seg, long index, short x) {
        constants$10.const$0.set(seg, index * sizeof(), x);
    }
    public static VarHandle d_type$VH() {
        return constants$10.const$1;
    }
    /**
     * Getter for field:
     * {@snippet :
     * unsigned char d_type;
     * }
     */
    public static byte d_type$get(MemorySegment seg) {
        return (byte)constants$10.const$1.get(seg, 0L);
    }
    /**
     * Setter for field:
     * {@snippet :
     * unsigned char d_type;
     * }
     */
    public static void d_type$set(MemorySegment seg, byte x) {
        constants$10.const$1.set(seg, 0L, x);
    }
    public static byte d_type$get(MemorySegment seg, long index) {
        return (byte)constants$10.const$1.get(seg, index * sizeof());    }
    public static void d_type$set(MemorySegment seg, long index, byte x) {
        constants$10.const$1.set(seg, index * sizeof(), x);
    }
    public static MemorySegment d_name$slice(MemorySegment seg) {
        return seg.asSlice(19, 256);
    }
    public static long sizeof() { return $LAYOUT().byteSize(); }
    public static MemorySegment allocate(SegmentAllocator allocator) { return allocator.allocate($LAYOUT()); }
    public static MemorySegment allocateArray(long len, SegmentAllocator allocator) {
        return allocator.allocate(MemoryLayout.sequenceLayout(len, $LAYOUT()));
    }
    public static MemorySegment ofAddress(MemorySegment addr, Arena scope) { return RuntimeHelper.asArray(addr, $LAYOUT(), 1, scope); }
}

