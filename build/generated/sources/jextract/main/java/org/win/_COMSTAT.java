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
 * struct _COMSTAT {
 *     DWORD fCtsHold : 1;
 *     DWORD fDsrHold : 1;
 *     DWORD fRlsdHold : 1;
 *     DWORD fXoffHold : 1;
 *     DWORD fXoffSent : 1;
 *     DWORD fEof : 1;
 *     DWORD fTxim : 1;
 *     DWORD fReserved : 25;
 *     DWORD cbInQue;
 *     DWORD cbOutQue;
 * }
 * }
 */
public class _COMSTAT {

    _COMSTAT() {
        // Suppresses public default constructor, ensuring non-instantiability,
        // but allows generated subclasses in same package.
    }

    private static final GroupLayout $LAYOUT = MemoryLayout.structLayout(
        MemoryLayout.paddingLayout(4),
        Windows.C_LONG.withName("cbInQue"),
        Windows.C_LONG.withName("cbOutQue")
    ).withName("_COMSTAT");

    public static final GroupLayout layout() {
        return $LAYOUT;
    }

    private static final long cbInQue$OFFSET = 4;
    private static final OfInt cbInQue$LAYOUT = (OfInt)$LAYOUT.select(groupElement("cbInQue"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD cbInQue
     * }
     */
    public static int cbInQue(MemorySegment struct) {
        return struct.get(cbInQue$LAYOUT, cbInQue$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD cbInQue
     * }
     */
    public static void cbInQue(MemorySegment struct, int fieldValue) {
        struct.set(cbInQue$LAYOUT, cbInQue$OFFSET, fieldValue);
    }

    private static final long cbOutQue$OFFSET = 8;
    private static final OfInt cbOutQue$LAYOUT = (OfInt)$LAYOUT.select(groupElement("cbOutQue"));

    /**
     * Getter for field:
     * {@snippet lang=c :
     * DWORD cbOutQue
     * }
     */
    public static int cbOutQue(MemorySegment struct) {
        return struct.get(cbOutQue$LAYOUT, cbOutQue$OFFSET);
    }

    /**
     * Setter for field:
     * {@snippet lang=c :
     * DWORD cbOutQue
     * }
     */
    public static void cbOutQue(MemorySegment struct, int fieldValue) {
        struct.set(cbOutQue$LAYOUT, cbOutQue$OFFSET, fieldValue);
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

