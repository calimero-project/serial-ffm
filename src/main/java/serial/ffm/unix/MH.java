package serial.ffm.unix;

import static java.lang.invoke.MethodType.methodType;

import java.lang.foreign.Arena;
import java.lang.foreign.GroupLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.util.function.Consumer;

final class MH {
	private final MethodHandles.Lookup lookup;
	private final Class<?> refc;

	MH(final MethodHandles.Lookup lookup, final Class<?> refc) {
		this.lookup = lookup;
		this.refc = refc;
	}

	MethodHandle findStatic(final String name, final MethodType type) {
		try {
			return lookup.findStatic(refc, name, type);
		} catch (final ReflectiveOperationException e) {
			throw new AssertionError(e);
		}
	}

	MethodHandle allocate() { return findStatic("allocate", methodType(MemorySegment.class, SegmentAllocator.class)); }

	MethodHandle reinterpret() {
		return findStatic("reinterpret",
				methodType(MemorySegment.class, MemorySegment.class, Arena.class, Consumer.class));
	}

	MethodHandle layout() { return findStatic("layout", methodType(GroupLayout.class)); }
}
