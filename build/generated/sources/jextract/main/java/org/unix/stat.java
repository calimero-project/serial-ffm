// Generated by jextract

package org.unix;

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
 * struct stat {
 *     __dev_t st_dev;
 *     __ino_t st_ino;
 *     __nlink_t st_nlink;
 *     __mode_t st_mode;
 *     __uid_t st_uid;
 *     __gid_t st_gid;
 *     int __pad0;
 *     __dev_t st_rdev;
 *     __off_t st_size;
 *     __blksize_t st_blksize;
 *     __blkcnt_t st_blocks;
 *     struct timespec st_atim;
 *     struct timespec st_mtim;
 *     struct timespec st_ctim;
 *     __syscall_slong_t __glibc_reserved[3];
 * }
 * }
 */
public class stat {

    stat() {
        // Should not be called directly
    }

    private static final GroupLayout $LAYOUT = MemoryLayout.structLayout(
        Linux.C_LONG.withName("st_dev"),
        Linux.C_LONG.withName("st_ino"),
        Linux.C_LONG.withName("st_nlink"),
        Linux.C_INT.withName("st_mode"),
        Linux.C_INT.withName("st_uid"),
        Linux.C_INT.withName("st_gid"),
        Linux.C_INT.withName("__pad0"),
        Linux.C_LONG.withName("st_rdev"),
        Linux.C_LONG.withName("st_size"),
        Linux.C_LONG.withName("st_blksize"),
        Linux.C_LONG.withName("st_blocks"),
        timespec.layout().withName("st_atim"),
        timespec.layout().withName("st_mtim"),
        timespec.layout().withName("st_ctim"),
        MemoryLayout.sequenceLayout(3, Linux.C_LONG).withName("__glibc_reserved")
    ).withName("stat");

    /**
     * The layout of this struct
     */
    public static final GroupLayout layout() {
        return $LAYOUT;
    }

    private static final OfLong st_dev$LAYOUT = (OfLong)$LAYOUT.select(groupElement("st_dev"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * __dev_t st_dev
     * }
     */
    public static final OfLong st_dev$layout() {
        return st_dev$LAYOUT;
    }

    private static final long st_dev$OFFSET = 0;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * __dev_t st_dev
     * }
     */
    public static final long st_dev$offset() {
        return st_dev$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * __dev_t st_dev
     * }
     */
    public static long st_dev(MemorySegment struct) {
        return struct.get(st_dev$LAYOUT, st_dev$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * __dev_t st_dev
     * }
     */
    public static void st_dev(MemorySegment struct, long fieldValue) {
        struct.set(st_dev$LAYOUT, st_dev$OFFSET, fieldValue);
    }

    private static final OfLong st_ino$LAYOUT = (OfLong)$LAYOUT.select(groupElement("st_ino"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * __ino_t st_ino
     * }
     */
    public static final OfLong st_ino$layout() {
        return st_ino$LAYOUT;
    }

    private static final long st_ino$OFFSET = 8;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * __ino_t st_ino
     * }
     */
    public static final long st_ino$offset() {
        return st_ino$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * __ino_t st_ino
     * }
     */
    public static long st_ino(MemorySegment struct) {
        return struct.get(st_ino$LAYOUT, st_ino$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * __ino_t st_ino
     * }
     */
    public static void st_ino(MemorySegment struct, long fieldValue) {
        struct.set(st_ino$LAYOUT, st_ino$OFFSET, fieldValue);
    }

    private static final OfLong st_nlink$LAYOUT = (OfLong)$LAYOUT.select(groupElement("st_nlink"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * __nlink_t st_nlink
     * }
     */
    public static final OfLong st_nlink$layout() {
        return st_nlink$LAYOUT;
    }

    private static final long st_nlink$OFFSET = 16;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * __nlink_t st_nlink
     * }
     */
    public static final long st_nlink$offset() {
        return st_nlink$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * __nlink_t st_nlink
     * }
     */
    public static long st_nlink(MemorySegment struct) {
        return struct.get(st_nlink$LAYOUT, st_nlink$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * __nlink_t st_nlink
     * }
     */
    public static void st_nlink(MemorySegment struct, long fieldValue) {
        struct.set(st_nlink$LAYOUT, st_nlink$OFFSET, fieldValue);
    }

    private static final OfInt st_mode$LAYOUT = (OfInt)$LAYOUT.select(groupElement("st_mode"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * __mode_t st_mode
     * }
     */
    public static final OfInt st_mode$layout() {
        return st_mode$LAYOUT;
    }

    private static final long st_mode$OFFSET = 24;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * __mode_t st_mode
     * }
     */
    public static final long st_mode$offset() {
        return st_mode$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * __mode_t st_mode
     * }
     */
    public static int st_mode(MemorySegment struct) {
        return struct.get(st_mode$LAYOUT, st_mode$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * __mode_t st_mode
     * }
     */
    public static void st_mode(MemorySegment struct, int fieldValue) {
        struct.set(st_mode$LAYOUT, st_mode$OFFSET, fieldValue);
    }

    private static final OfInt st_uid$LAYOUT = (OfInt)$LAYOUT.select(groupElement("st_uid"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * __uid_t st_uid
     * }
     */
    public static final OfInt st_uid$layout() {
        return st_uid$LAYOUT;
    }

    private static final long st_uid$OFFSET = 28;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * __uid_t st_uid
     * }
     */
    public static final long st_uid$offset() {
        return st_uid$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * __uid_t st_uid
     * }
     */
    public static int st_uid(MemorySegment struct) {
        return struct.get(st_uid$LAYOUT, st_uid$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * __uid_t st_uid
     * }
     */
    public static void st_uid(MemorySegment struct, int fieldValue) {
        struct.set(st_uid$LAYOUT, st_uid$OFFSET, fieldValue);
    }

    private static final OfInt st_gid$LAYOUT = (OfInt)$LAYOUT.select(groupElement("st_gid"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * __gid_t st_gid
     * }
     */
    public static final OfInt st_gid$layout() {
        return st_gid$LAYOUT;
    }

    private static final long st_gid$OFFSET = 32;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * __gid_t st_gid
     * }
     */
    public static final long st_gid$offset() {
        return st_gid$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * __gid_t st_gid
     * }
     */
    public static int st_gid(MemorySegment struct) {
        return struct.get(st_gid$LAYOUT, st_gid$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * __gid_t st_gid
     * }
     */
    public static void st_gid(MemorySegment struct, int fieldValue) {
        struct.set(st_gid$LAYOUT, st_gid$OFFSET, fieldValue);
    }

    private static final OfInt __pad0$LAYOUT = (OfInt)$LAYOUT.select(groupElement("__pad0"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * int __pad0
     * }
     */
    public static final OfInt __pad0$layout() {
        return __pad0$LAYOUT;
    }

    private static final long __pad0$OFFSET = 36;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * int __pad0
     * }
     */
    public static final long __pad0$offset() {
        return __pad0$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * int __pad0
     * }
     */
    public static int __pad0(MemorySegment struct) {
        return struct.get(__pad0$LAYOUT, __pad0$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * int __pad0
     * }
     */
    public static void __pad0(MemorySegment struct, int fieldValue) {
        struct.set(__pad0$LAYOUT, __pad0$OFFSET, fieldValue);
    }

    private static final OfLong st_rdev$LAYOUT = (OfLong)$LAYOUT.select(groupElement("st_rdev"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * __dev_t st_rdev
     * }
     */
    public static final OfLong st_rdev$layout() {
        return st_rdev$LAYOUT;
    }

    private static final long st_rdev$OFFSET = 40;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * __dev_t st_rdev
     * }
     */
    public static final long st_rdev$offset() {
        return st_rdev$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * __dev_t st_rdev
     * }
     */
    public static long st_rdev(MemorySegment struct) {
        return struct.get(st_rdev$LAYOUT, st_rdev$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * __dev_t st_rdev
     * }
     */
    public static void st_rdev(MemorySegment struct, long fieldValue) {
        struct.set(st_rdev$LAYOUT, st_rdev$OFFSET, fieldValue);
    }

    private static final OfLong st_size$LAYOUT = (OfLong)$LAYOUT.select(groupElement("st_size"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * __off_t st_size
     * }
     */
    public static final OfLong st_size$layout() {
        return st_size$LAYOUT;
    }

    private static final long st_size$OFFSET = 48;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * __off_t st_size
     * }
     */
    public static final long st_size$offset() {
        return st_size$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * __off_t st_size
     * }
     */
    public static long st_size(MemorySegment struct) {
        return struct.get(st_size$LAYOUT, st_size$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * __off_t st_size
     * }
     */
    public static void st_size(MemorySegment struct, long fieldValue) {
        struct.set(st_size$LAYOUT, st_size$OFFSET, fieldValue);
    }

    private static final OfLong st_blksize$LAYOUT = (OfLong)$LAYOUT.select(groupElement("st_blksize"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * __blksize_t st_blksize
     * }
     */
    public static final OfLong st_blksize$layout() {
        return st_blksize$LAYOUT;
    }

    private static final long st_blksize$OFFSET = 56;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * __blksize_t st_blksize
     * }
     */
    public static final long st_blksize$offset() {
        return st_blksize$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * __blksize_t st_blksize
     * }
     */
    public static long st_blksize(MemorySegment struct) {
        return struct.get(st_blksize$LAYOUT, st_blksize$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * __blksize_t st_blksize
     * }
     */
    public static void st_blksize(MemorySegment struct, long fieldValue) {
        struct.set(st_blksize$LAYOUT, st_blksize$OFFSET, fieldValue);
    }

    private static final OfLong st_blocks$LAYOUT = (OfLong)$LAYOUT.select(groupElement("st_blocks"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * __blkcnt_t st_blocks
     * }
     */
    public static final OfLong st_blocks$layout() {
        return st_blocks$LAYOUT;
    }

    private static final long st_blocks$OFFSET = 64;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * __blkcnt_t st_blocks
     * }
     */
    public static final long st_blocks$offset() {
        return st_blocks$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * __blkcnt_t st_blocks
     * }
     */
    public static long st_blocks(MemorySegment struct) {
        return struct.get(st_blocks$LAYOUT, st_blocks$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * __blkcnt_t st_blocks
     * }
     */
    public static void st_blocks(MemorySegment struct, long fieldValue) {
        struct.set(st_blocks$LAYOUT, st_blocks$OFFSET, fieldValue);
    }

    private static final GroupLayout st_atim$LAYOUT = (GroupLayout)$LAYOUT.select(groupElement("st_atim"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * struct timespec st_atim
     * }
     */
    public static final GroupLayout st_atim$layout() {
        return st_atim$LAYOUT;
    }

    private static final long st_atim$OFFSET = 72;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * struct timespec st_atim
     * }
     */
    public static final long st_atim$offset() {
        return st_atim$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * struct timespec st_atim
     * }
     */
    public static MemorySegment st_atim(MemorySegment struct) {
        return struct.asSlice(st_atim$OFFSET, st_atim$LAYOUT.byteSize());
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * struct timespec st_atim
     * }
     */
    public static void st_atim(MemorySegment struct, MemorySegment fieldValue) {
        MemorySegment.copy(fieldValue, 0L, struct, st_atim$OFFSET, st_atim$LAYOUT.byteSize());
    }

    private static final GroupLayout st_mtim$LAYOUT = (GroupLayout)$LAYOUT.select(groupElement("st_mtim"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * struct timespec st_mtim
     * }
     */
    public static final GroupLayout st_mtim$layout() {
        return st_mtim$LAYOUT;
    }

    private static final long st_mtim$OFFSET = 88;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * struct timespec st_mtim
     * }
     */
    public static final long st_mtim$offset() {
        return st_mtim$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * struct timespec st_mtim
     * }
     */
    public static MemorySegment st_mtim(MemorySegment struct) {
        return struct.asSlice(st_mtim$OFFSET, st_mtim$LAYOUT.byteSize());
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * struct timespec st_mtim
     * }
     */
    public static void st_mtim(MemorySegment struct, MemorySegment fieldValue) {
        MemorySegment.copy(fieldValue, 0L, struct, st_mtim$OFFSET, st_mtim$LAYOUT.byteSize());
    }

    private static final GroupLayout st_ctim$LAYOUT = (GroupLayout)$LAYOUT.select(groupElement("st_ctim"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * struct timespec st_ctim
     * }
     */
    public static final GroupLayout st_ctim$layout() {
        return st_ctim$LAYOUT;
    }

    private static final long st_ctim$OFFSET = 104;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * struct timespec st_ctim
     * }
     */
    public static final long st_ctim$offset() {
        return st_ctim$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * struct timespec st_ctim
     * }
     */
    public static MemorySegment st_ctim(MemorySegment struct) {
        return struct.asSlice(st_ctim$OFFSET, st_ctim$LAYOUT.byteSize());
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * struct timespec st_ctim
     * }
     */
    public static void st_ctim(MemorySegment struct, MemorySegment fieldValue) {
        MemorySegment.copy(fieldValue, 0L, struct, st_ctim$OFFSET, st_ctim$LAYOUT.byteSize());
    }

    private static final SequenceLayout __glibc_reserved$LAYOUT = (SequenceLayout)$LAYOUT.select(groupElement("__glibc_reserved"));

    /**
     * Layout for field:
     * {@snippet lang=c :
     * __syscall_slong_t __glibc_reserved[3]
     * }
     */
    public static final SequenceLayout __glibc_reserved$layout() {
        return __glibc_reserved$LAYOUT;
    }

    private static final long __glibc_reserved$OFFSET = 120;

    /**
     * Offset for field:
     * {@snippet lang=c :
     * __syscall_slong_t __glibc_reserved[3]
     * }
     */
    public static final long __glibc_reserved$offset() {
        return __glibc_reserved$OFFSET;
    }

    /**
     * Getter for field:
     * {@snippet lang=c :
     * __syscall_slong_t __glibc_reserved[3]
     * }
     */
    public static MemorySegment __glibc_reserved(MemorySegment struct) {
        return struct.asSlice(__glibc_reserved$OFFSET, __glibc_reserved$LAYOUT.byteSize());
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * __syscall_slong_t __glibc_reserved[3]
     * }
     */
    public static void __glibc_reserved(MemorySegment struct, MemorySegment fieldValue) {
        MemorySegment.copy(fieldValue, 0L, struct, __glibc_reserved$OFFSET, __glibc_reserved$LAYOUT.byteSize());
    }

    private static long[] __glibc_reserved$DIMS = { 3 };

    /**
     * Dimensions for array field:
     * {@snippet lang=c :
     * __syscall_slong_t __glibc_reserved[3]
     * }
     */
    public static long[] __glibc_reserved$dimensions() {
        return __glibc_reserved$DIMS;
    }
    private static final VarHandle __glibc_reserved$ELEM_HANDLE = __glibc_reserved$LAYOUT.varHandle(sequenceElement());

    /**
     * Indexed getter for field:
     * {@snippet lang=c :
     * __syscall_slong_t __glibc_reserved[3]
     * }
     */
    public static long __glibc_reserved(MemorySegment struct, long index0) {
        return (long)__glibc_reserved$ELEM_HANDLE.get(struct, 0L, index0);
    }

    /**
     * Indexed setter for field:
     * {@snippet lang=c :
     * __syscall_slong_t __glibc_reserved[3]
     * }
     */
    public static void __glibc_reserved(MemorySegment struct, long index0, long fieldValue) {
        __glibc_reserved$ELEM_HANDLE.set(struct, 0L, index0, fieldValue);
    }

    /**
     * Obtains a slice of {@code arrayParam} which selects the array element at {@code index}.
     * The returned segment has address {@code arrayParam.address() + index * layout().byteSize()}
     */
    public static MemorySegment asSlice(MemorySegment array, long index) {
        return array.asSlice(layout().byteSize() * index);
    }

    /**
     * The size (in bytes) of this struct
     */
    public static long sizeof() { return layout().byteSize(); }

    /**
     * Allocate a segment of size {@code layout().byteSize()} using {@code allocator}
     */
    public static MemorySegment allocate(SegmentAllocator allocator) {
        return allocator.allocate(layout());
    }

    /**
     * Allocate an array of size {@code elementCount} using {@code allocator}.
     * The returned segment has size {@code elementCount * layout().byteSize()}.
     */
    public static MemorySegment allocateArray(long elementCount, SegmentAllocator allocator) {
        return allocator.allocate(MemoryLayout.sequenceLayout(elementCount, layout()));
    }

    /**
     * Reinterprets {@code addr} using target {@code arena} and {@code cleanupAction} (if any).
     * The returned segment has size {@code layout().byteSize()}
     */
    public static MemorySegment reinterpret(MemorySegment addr, Arena arena, Consumer<MemorySegment> cleanup) {
        return reinterpret(addr, 1, arena, cleanup);
    }

    /**
     * Reinterprets {@code addr} using target {@code arena} and {@code cleanupAction} (if any).
     * The returned segment has size {@code elementCount * layout().byteSize()}
     */
    public static MemorySegment reinterpret(MemorySegment addr, long elementCount, Arena arena, Consumer<MemorySegment> cleanup) {
        return addr.reinterpret(layout().byteSize() * elementCount, arena, cleanup);
    }
}

