// Generated by jextract

package serial.ffm.linux;

import java.lang.invoke.VarHandle;
import java.lang.foreign.*;

import static java.lang.foreign.ValueLayout.*;
final class constants$14 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$14() {}
    static final VarHandle const$0 = constants$11.const$2.varHandle(PathElement.groupElement("port_high"));
    static final VarHandle const$1 = constants$11.const$2.varHandle(PathElement.groupElement("iomap_base"));
    static final StructLayout const$2 = MemoryLayout.structLayout(
        JAVA_INT.withName("cts"),
        JAVA_INT.withName("dsr"),
        JAVA_INT.withName("rng"),
        JAVA_INT.withName("dcd"),
        JAVA_INT.withName("rx"),
        JAVA_INT.withName("tx"),
        JAVA_INT.withName("frame"),
        JAVA_INT.withName("overrun"),
        JAVA_INT.withName("parity"),
        JAVA_INT.withName("brk"),
        JAVA_INT.withName("buf_overrun"),
        MemoryLayout.sequenceLayout(9, JAVA_INT).withName("reserved")
    ).withName("serial_icounter_struct");
    static final VarHandle const$3 = constants$14.const$2.varHandle(PathElement.groupElement("cts"));
    static final VarHandle const$4 = constants$14.const$2.varHandle(PathElement.groupElement("dsr"));
    static final VarHandle const$5 = constants$14.const$2.varHandle(PathElement.groupElement("rng"));
}
