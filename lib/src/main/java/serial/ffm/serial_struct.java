// Generated by jextract

// This struct is saved here, because we need it for TIOCGSERIAL,
// but that functionality is not supported on macOS.

package serial.ffm;

import java.lang.foreign.GroupLayout;
import java.lang.foreign.MemoryLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.VarHandle;

@SuppressWarnings("checkstyle:all")
class serial_struct {

	static final  GroupLayout $struct$LAYOUT = MemoryLayout.structLayout(
			ValueLayout.JAVA_INT.withName("type"),
			ValueLayout.JAVA_INT.withName("line"),
			ValueLayout.JAVA_INT.withName("port"),
			ValueLayout.JAVA_INT.withName("irq"),
			ValueLayout.JAVA_INT.withName("flags"),
			ValueLayout.JAVA_INT.withName("xmit_fifo_size"),
			ValueLayout.JAVA_INT.withName("custom_divisor"),
			ValueLayout.JAVA_INT.withName("baud_base"),
			ValueLayout.JAVA_SHORT.withName("close_delay"),
			ValueLayout.JAVA_BYTE.withName("io_type"),
			MemoryLayout.sequenceLayout(1, ValueLayout.JAVA_BYTE).withName("reserved_char"),
			ValueLayout.JAVA_INT.withName("hub6"),
			ValueLayout.JAVA_SHORT.withName("closing_wait"),
			ValueLayout.JAVA_SHORT.withName("closing_wait2"),
			MemoryLayout.paddingLayout(32),
			ValueLayout.ADDRESS.withName("iomem_base"),
			ValueLayout.JAVA_SHORT.withName("iomem_reg_shift"),
			MemoryLayout.paddingLayout(16),
			ValueLayout.JAVA_INT.withName("port_high"),
			ValueLayout.JAVA_LONG.withName("iomap_base")
			).withName("serial_struct");
	public static MemoryLayout $LAYOUT() {
		return serial_struct.$struct$LAYOUT;
	}
	static final VarHandle type$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("type"));
	public static VarHandle type$VH() {
		return serial_struct.type$VH;
	}
	public static int type$get(final MemorySegment seg) {
		return (int)serial_struct.type$VH.get(seg);
	}
	public static void type$set( final MemorySegment seg, final int x) {
		serial_struct.type$VH.set(seg, x);
	}
	public static int type$get(final MemorySegment seg, final long index) {
		return (int)serial_struct.type$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void type$set(final MemorySegment seg, final long index, final int x) {
		serial_struct.type$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle line$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("line"));
	public static VarHandle line$VH() {
		return serial_struct.line$VH;
	}
	public static int line$get(final MemorySegment seg) {
		return (int)serial_struct.line$VH.get(seg);
	}
	public static void line$set( final MemorySegment seg, final int x) {
		serial_struct.line$VH.set(seg, x);
	}
	public static int line$get(final MemorySegment seg, final long index) {
		return (int)serial_struct.line$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void line$set(final MemorySegment seg, final long index, final int x) {
		serial_struct.line$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle port$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("port"));
	public static VarHandle port$VH() {
		return serial_struct.port$VH;
	}
	public static int port$get(final MemorySegment seg) {
		return (int)serial_struct.port$VH.get(seg);
	}
	public static void port$set( final MemorySegment seg, final int x) {
		serial_struct.port$VH.set(seg, x);
	}
	public static int port$get(final MemorySegment seg, final long index) {
		return (int)serial_struct.port$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void port$set(final MemorySegment seg, final long index, final int x) {
		serial_struct.port$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle irq$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("irq"));
	public static VarHandle irq$VH() {
		return serial_struct.irq$VH;
	}
	public static int irq$get(final MemorySegment seg) {
		return (int)serial_struct.irq$VH.get(seg);
	}
	public static void irq$set( final MemorySegment seg, final int x) {
		serial_struct.irq$VH.set(seg, x);
	}
	public static int irq$get(final MemorySegment seg, final long index) {
		return (int)serial_struct.irq$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void irq$set(final MemorySegment seg, final long index, final int x) {
		serial_struct.irq$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle flags$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("flags"));
	public static VarHandle flags$VH() {
		return serial_struct.flags$VH;
	}
	public static int flags$get(final MemorySegment seg) {
		return (int)serial_struct.flags$VH.get(seg);
	}
	public static void flags$set( final MemorySegment seg, final int x) {
		serial_struct.flags$VH.set(seg, x);
	}
	public static int flags$get(final MemorySegment seg, final long index) {
		return (int)serial_struct.flags$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void flags$set(final MemorySegment seg, final long index, final int x) {
		serial_struct.flags$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle xmit_fifo_size$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("xmit_fifo_size"));
	public static VarHandle xmit_fifo_size$VH() {
		return serial_struct.xmit_fifo_size$VH;
	}
	public static int xmit_fifo_size$get(final MemorySegment seg) {
		return (int)serial_struct.xmit_fifo_size$VH.get(seg);
	}
	public static void xmit_fifo_size$set( final MemorySegment seg, final int x) {
		serial_struct.xmit_fifo_size$VH.set(seg, x);
	}
	public static int xmit_fifo_size$get(final MemorySegment seg, final long index) {
		return (int)serial_struct.xmit_fifo_size$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void xmit_fifo_size$set(final MemorySegment seg, final long index, final int x) {
		serial_struct.xmit_fifo_size$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle custom_divisor$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("custom_divisor"));
	public static VarHandle custom_divisor$VH() {
		return serial_struct.custom_divisor$VH;
	}
	public static int custom_divisor$get(final MemorySegment seg) {
		return (int)serial_struct.custom_divisor$VH.get(seg);
	}
	public static void custom_divisor$set( final MemorySegment seg, final int x) {
		serial_struct.custom_divisor$VH.set(seg, x);
	}
	public static int custom_divisor$get(final MemorySegment seg, final long index) {
		return (int)serial_struct.custom_divisor$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void custom_divisor$set(final MemorySegment seg, final long index, final int x) {
		serial_struct.custom_divisor$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle baud_base$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("baud_base"));
	public static VarHandle baud_base$VH() {
		return serial_struct.baud_base$VH;
	}
	public static int baud_base$get(final MemorySegment seg) {
		return (int)serial_struct.baud_base$VH.get(seg);
	}
	public static void baud_base$set( final MemorySegment seg, final int x) {
		serial_struct.baud_base$VH.set(seg, x);
	}
	public static int baud_base$get(final MemorySegment seg, final long index) {
		return (int)serial_struct.baud_base$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void baud_base$set(final MemorySegment seg, final long index, final int x) {
		serial_struct.baud_base$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle close_delay$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("close_delay"));
	public static VarHandle close_delay$VH() {
		return serial_struct.close_delay$VH;
	}
	public static short close_delay$get(final MemorySegment seg) {
		return (short)serial_struct.close_delay$VH.get(seg);
	}
	public static void close_delay$set( final MemorySegment seg, final short x) {
		serial_struct.close_delay$VH.set(seg, x);
	}
	public static short close_delay$get(final MemorySegment seg, final long index) {
		return (short)serial_struct.close_delay$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void close_delay$set(final MemorySegment seg, final long index, final short x) {
		serial_struct.close_delay$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle io_type$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("io_type"));
	public static VarHandle io_type$VH() {
		return serial_struct.io_type$VH;
	}
	public static byte io_type$get(final MemorySegment seg) {
		return (byte)serial_struct.io_type$VH.get(seg);
	}
	public static void io_type$set( final MemorySegment seg, final byte x) {
		serial_struct.io_type$VH.set(seg, x);
	}
	public static byte io_type$get(final MemorySegment seg, final long index) {
		return (byte)serial_struct.io_type$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void io_type$set(final MemorySegment seg, final long index, final byte x) {
		serial_struct.io_type$VH.set(seg.asSlice(index*sizeof()), x);
	}
	public static MemorySegment reserved_char$slice(final MemorySegment seg) {
		return seg.asSlice(35, 1);
	}
	static final VarHandle hub6$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("hub6"));
	public static VarHandle hub6$VH() {
		return serial_struct.hub6$VH;
	}
	public static int hub6$get(final MemorySegment seg) {
		return (int)serial_struct.hub6$VH.get(seg);
	}
	public static void hub6$set( final MemorySegment seg, final int x) {
		serial_struct.hub6$VH.set(seg, x);
	}
	public static int hub6$get(final MemorySegment seg, final long index) {
		return (int)serial_struct.hub6$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void hub6$set(final MemorySegment seg, final long index, final int x) {
		serial_struct.hub6$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle closing_wait$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("closing_wait"));
	public static VarHandle closing_wait$VH() {
		return serial_struct.closing_wait$VH;
	}
	public static short closing_wait$get(final MemorySegment seg) {
		return (short)serial_struct.closing_wait$VH.get(seg);
	}
	public static void closing_wait$set( final MemorySegment seg, final short x) {
		serial_struct.closing_wait$VH.set(seg, x);
	}
	public static short closing_wait$get(final MemorySegment seg, final long index) {
		return (short)serial_struct.closing_wait$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void closing_wait$set(final MemorySegment seg, final long index, final short x) {
		serial_struct.closing_wait$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle closing_wait2$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("closing_wait2"));
	public static VarHandle closing_wait2$VH() {
		return serial_struct.closing_wait2$VH;
	}
	public static short closing_wait2$get(final MemorySegment seg) {
		return (short)serial_struct.closing_wait2$VH.get(seg);
	}
	public static void closing_wait2$set( final MemorySegment seg, final short x) {
		serial_struct.closing_wait2$VH.set(seg, x);
	}
	public static short closing_wait2$get(final MemorySegment seg, final long index) {
		return (short)serial_struct.closing_wait2$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void closing_wait2$set(final MemorySegment seg, final long index, final short x) {
		serial_struct.closing_wait2$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle iomem_base$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("iomem_base"));
	public static VarHandle iomem_base$VH() {
		return serial_struct.iomem_base$VH;
	}
	public static MemorySegment iomem_base$get(final MemorySegment seg) {
		return (java.lang.foreign.MemorySegment)serial_struct.iomem_base$VH.get(seg);
	}
	public static void iomem_base$set( final MemorySegment seg, final MemorySegment x) {
		serial_struct.iomem_base$VH.set(seg, x);
	}
	public static MemorySegment iomem_base$get(final MemorySegment seg, final long index) {
		return (java.lang.foreign.MemorySegment)serial_struct.iomem_base$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void iomem_base$set(final MemorySegment seg, final long index, final MemorySegment x) {
		serial_struct.iomem_base$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle iomem_reg_shift$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("iomem_reg_shift"));
	public static VarHandle iomem_reg_shift$VH() {
		return serial_struct.iomem_reg_shift$VH;
	}
	public static short iomem_reg_shift$get(final MemorySegment seg) {
		return (short)serial_struct.iomem_reg_shift$VH.get(seg);
	}
	public static void iomem_reg_shift$set( final MemorySegment seg, final short x) {
		serial_struct.iomem_reg_shift$VH.set(seg, x);
	}
	public static short iomem_reg_shift$get(final MemorySegment seg, final long index) {
		return (short)serial_struct.iomem_reg_shift$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void iomem_reg_shift$set(final MemorySegment seg, final long index, final short x) {
		serial_struct.iomem_reg_shift$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle port_high$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("port_high"));
	public static VarHandle port_high$VH() {
		return serial_struct.port_high$VH;
	}
	public static int port_high$get(final MemorySegment seg) {
		return (int)serial_struct.port_high$VH.get(seg);
	}
	public static void port_high$set( final MemorySegment seg, final int x) {
		serial_struct.port_high$VH.set(seg, x);
	}
	public static int port_high$get(final MemorySegment seg, final long index) {
		return (int)serial_struct.port_high$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void port_high$set(final MemorySegment seg, final long index, final int x) {
		serial_struct.port_high$VH.set(seg.asSlice(index*sizeof()), x);
	}
	static final VarHandle iomap_base$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("iomap_base"));
	public static VarHandle iomap_base$VH() {
		return serial_struct.iomap_base$VH;
	}
	public static long iomap_base$get(final MemorySegment seg) {
		return (long)serial_struct.iomap_base$VH.get(seg);
	}
	public static void iomap_base$set( final MemorySegment seg, final long x) {
		serial_struct.iomap_base$VH.set(seg, x);
	}
	public static long iomap_base$get(final MemorySegment seg, final long index) {
		return (long)serial_struct.iomap_base$VH.get(seg.asSlice(index*sizeof()));
	}
	public static void iomap_base$set(final MemorySegment seg, final long index, final long x) {
		serial_struct.iomap_base$VH.set(seg.asSlice(index*sizeof()), x);
	}
	public static long sizeof() { return $LAYOUT().byteSize(); }
	public static MemorySegment allocate(final SegmentAllocator allocator) { return allocator.allocate($LAYOUT()); }
	public static MemorySegment allocateArray(final int len, final SegmentAllocator allocator) {
		return allocator.allocate(MemoryLayout.sequenceLayout(len, $LAYOUT()));
	}
//	public static MemorySegment ofAddress(final MemoryAddress addr, final MemorySession session) { return RuntimeHelper.asArray(addr, $LAYOUT(), 1, session); }
}

