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

import java.lang.foreign.GroupLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;

import serial.ffm.OS;

public final class stat {
	private static final MethodHandle allocate;
	private static final MethodHandle layout;
	private static final MethodHandle st_nlink;

	static {
		final String arch = System.getProperty("os.arch");
		final boolean aarch64 = "aarch64".equals(arch);

		final var mh = new MH(MethodHandles.lookup(), switch (OS.current()) {
			case Linux -> aarch64 ? serial.ffm.linux.aarch64.stat.class : serial.ffm.linux.stat.class;
			case Mac -> serial.ffm.mac.stat.class;
			default -> throw new IllegalStateException();
		});
		allocate = mh.allocate();
		layout   = mh.layout();
		final Class<?> rtype = OS.current() == OS.Linux ? aarch64 ? int.class : long.class : short.class;
		st_nlink = mh.findStatic("st_nlink", methodType(rtype, MemorySegment.class));
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
	 * The layout of this struct
	 */
	public static GroupLayout layout() {
		try {
			return (GroupLayout) layout.invokeExact();
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Getter for field:
	 * {@snippet lang=c :
	 * __nlink_t st_nlink
	 * }
	 */
	public static long st_nlink(final MemorySegment struct) {
		try {
			return (long) st_nlink.invoke(struct);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}
}
