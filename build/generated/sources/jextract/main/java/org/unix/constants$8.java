// Generated by jextract

package org.unix;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
final class constants$8 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$8() {}
    static final FunctionDescriptor const$0 = FunctionDescriptor.of(RuntimeHelper.POINTER,
        JAVA_INT
    );
    static final MethodHandle const$1 = RuntimeHelper.downcallHandle(
        "strerror",
        constants$8.const$0
    );
    static final StructLayout const$2 = MemoryLayout.structLayout(
        JAVA_LONG.withName("tv_sec"),
        JAVA_LONG.withName("tv_usec")
    ).withName("timeval");
    static final VarHandle const$3 = constants$8.const$2.varHandle(MemoryLayout.PathElement.groupElement("tv_sec"));
    static final VarHandle const$4 = constants$8.const$2.varHandle(MemoryLayout.PathElement.groupElement("tv_usec"));
    static final StructLayout const$5 = MemoryLayout.structLayout(
        MemoryLayout.sequenceLayout(16, JAVA_LONG).withName("__fds_bits")
    ).withName("fd_set");
}


