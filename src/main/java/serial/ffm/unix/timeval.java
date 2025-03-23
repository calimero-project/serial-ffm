// MIT License
//
// Copyright (c) 2022, 2025 B. Malinowsky
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

import static java.lang.invoke.MethodType.methodType;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;

import serial.ffm.OS;

public final class timeval {
	private static final MethodHandle allocate;
	private static final MethodHandle tv_sec;
	private static final MethodHandle tv_usec;

	static {
		final var lookup = MethodHandles.lookup();
		final var alloc = methodType(MemorySegment.class, SegmentAllocator.class);
		final var setLong = methodType(void.class, MemorySegment.class, long.class);
		try {
			switch (OS.current()) {
				case Linux -> {
					allocate = lookup.findStatic(serial.ffm.linux.timeval.class, "allocate", alloc);
					tv_sec   = lookup.findStatic(serial.ffm.linux.timeval.class, "tv_sec", setLong);
					tv_usec  = lookup.findStatic(serial.ffm.linux.timeval.class, "tv_usec", setLong);
				}
				case Mac -> {
					allocate = lookup.findStatic(serial.ffm.mac.timeval.class, "allocate", alloc);
					tv_sec   = lookup.findStatic(serial.ffm.mac.timeval.class, "tv_sec", setLong);
					tv_usec  = lookup.findStatic(serial.ffm.mac.timeval.class, "tv_usec", methodType(void.class, MemorySegment.class, int.class));
				}
				default -> throw new IllegalStateException();
			}
		} catch (final ReflectiveOperationException e) {
			throw new AssertionError(e);
		}
	}

	/**
	 * Allocate a segment of size {@code layout().byteSize()} using {@code allocator}
	 */
	public static MemorySegment allocate(final SegmentAllocator allocator) {
		try {
			return (MemorySegment) allocate.invoke(allocator);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Setter for field:
	 * {@snippet lang=c :
	 * __time_t tv_sec
	 * }
	 */
	public static void tv_sec(final MemorySegment struct, final long fieldValue) {
		try {
			tv_sec.invoke(struct, fieldValue);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Setter for field:
	 * {@snippet lang=c :
	 * __suseconds_t tv_usec
	 * }
	 */
	public static void tv_usec(final MemorySegment struct, final int fieldValue) {
		try {
			tv_usec.invoke(struct, fieldValue);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}
}
