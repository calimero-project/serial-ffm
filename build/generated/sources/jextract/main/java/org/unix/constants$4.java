// Generated by jextract

package org.unix;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
final class constants$4 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$4() {}
    static final VarHandle const$0 = constants$3.const$4.varHandle(MemoryLayout.PathElement.groupElement("tv_nsec"));
    static final MethodHandle const$1 = RuntimeHelper.downcallHandle(
        "stat",
        constants$1.const$0
    );
    static final FunctionDescriptor const$2 = FunctionDescriptor.of(JAVA_INT,
        JAVA_INT,
        JAVA_INT
    );
    static final MethodHandle const$3 = RuntimeHelper.downcallHandleVariadic(
        "fcntl",
        constants$4.const$2
    );
    static final FunctionDescriptor const$4 = FunctionDescriptor.of(JAVA_INT,
        RuntimeHelper.POINTER,
        JAVA_INT
    );
    static final MethodHandle const$5 = RuntimeHelper.downcallHandleVariadic(
        "open",
        constants$4.const$4
    );
}


