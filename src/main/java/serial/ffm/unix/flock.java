// MIT License
//
// Copyright (c) 2026, 2026 B. Malinowsky
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package serial.ffm.unix;

import java.lang.foreign.Arena;
import java.lang.foreign.GroupLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.util.function.Consumer;

import serial.ffm.OS;

import static java.lang.invoke.MethodType.methodType;

public final class flock {
	private static final MethodHandle allocate;
	private static final MethodHandle l_type;
	private static final MethodHandle l_whence;

	static {
		final var mh = new MH(MethodHandles.lookup(), switch (OS.current()) {
			case Linux -> serial.ffm.linux.flock.class;
			case Mac -> serial.ffm.mac.flock.class;
			default -> throw new IllegalStateException();
		});
		allocate = mh.allocate();
		l_type   = mh.findStatic("l_type", methodType(void.class, MemorySegment.class, short.class));
		l_whence = mh.findStatic("l_whence", methodType(void.class, MemorySegment.class, short.class));
	}

	/**
	 * Allocate a segment of size {@code layout().byteSize()} using {@code allocator}
	 */
	public static MemorySegment allocate(final SegmentAllocator allocator) {
		try {
			return (MemorySegment) allocate.invokeExact(allocator);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Setter for field:
	 * {@snippet lang=c :
	 * short l_type
	 * }
	 */
	public static void l_type(final MemorySegment struct, final short fieldValue) {
		try {
			l_type.invokeExact(struct, fieldValue);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Setter for field:
	 * {@snippet lang=c :
	 * short l_whence
	 * }
	 */
	public static void l_whence(final MemorySegment struct, final short fieldValue) {
		try {
			l_whence.invokeExact(struct, fieldValue);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}
}
