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

public final class termios {
	private static final MethodHandle allocate;
	private static final MethodHandle c_cflag;
	private static final MethodHandle c_cflag_set;
	private static final MethodHandle c_iflag;
	private static final MethodHandle c_iflag_set;
	private static final MethodHandle c_lflag;
	private static final MethodHandle c_oflag;
	private static final MethodHandle c_cc;

	static {
		final var lookup = MethodHandles.lookup();
		final var alloc = methodType(MemorySegment.class, SegmentAllocator.class);
		final var cc = methodType(MemorySegment.class, MemorySegment.class);
		try {
			switch (OS.current()) {
				case Linux -> {
					final var setInt = methodType(void.class, MemorySegment.class, int.class);
					final var getInt = methodType(int.class, MemorySegment.class);

					allocate    = lookup.findStatic(serial.ffm.linux.termios.class, "allocate", alloc);
					c_cflag     = lookup.findStatic(serial.ffm.linux.termios.class, "c_cflag", getInt);
					c_cflag_set = lookup.findStatic(serial.ffm.linux.termios.class, "c_cflag", setInt);
					c_iflag     = lookup.findStatic(serial.ffm.linux.termios.class, "c_iflag", getInt);
					c_iflag_set = lookup.findStatic(serial.ffm.linux.termios.class, "c_iflag", setInt);
					c_lflag     = lookup.findStatic(serial.ffm.linux.termios.class, "c_lflag", setInt);
					c_oflag     = lookup.findStatic(serial.ffm.linux.termios.class, "c_oflag", setInt);
					c_cc        = lookup.findStatic(serial.ffm.linux.termios.class, "c_cc", cc);
				}
				case Mac -> {
					final var setLong = methodType(void.class, MemorySegment.class, long.class);
					final var getLong = methodType(long.class, MemorySegment.class);

					allocate    = lookup.findStatic(serial.ffm.mac.termios.class, "allocate", alloc);
					c_cflag     = lookup.findStatic(serial.ffm.mac.termios.class, "c_cflag", getLong);
					c_cflag_set = lookup.findStatic(serial.ffm.mac.termios.class, "c_cflag", setLong);
					c_iflag     = lookup.findStatic(serial.ffm.mac.termios.class, "c_iflag", getLong);
					c_iflag_set = lookup.findStatic(serial.ffm.mac.termios.class, "c_iflag", setLong);
					c_lflag     = lookup.findStatic(serial.ffm.mac.termios.class, "c_lflag", setLong);
					c_oflag     = lookup.findStatic(serial.ffm.mac.termios.class, "c_oflag", setLong);
					c_cc        = lookup.findStatic(serial.ffm.mac.termios.class, "c_cc", cc);
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
	 * Getter for field:
	 * {@snippet lang=c :
	 * tcflag_t c_cflag
	 * }
	 */
	public static int c_cflag(final MemorySegment struct) {
		try {
			return (int) (long) c_cflag.invoke(struct);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Setter for field:
	 * {@snippet lang=c :
	 * tcflag_t c_cflag
	 * }
	 */
	public static void c_cflag(final MemorySegment struct, final int fieldValue) {
		try {
			c_cflag_set.invoke(struct, fieldValue);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Getter for field:
	 * {@snippet lang=c :
	 * tcflag_t c_iflag
	 * }
	 */
	public static int c_iflag(final MemorySegment struct) {
		try {
			return (int) (long) c_iflag.invoke(struct);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Setter for field:
	 * {@snippet lang=c :
	 * tcflag_t c_iflag
	 * }
	 */
	public static void c_iflag(final MemorySegment struct, final int fieldValue) {
		try {
			c_iflag_set.invoke(struct, fieldValue);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Setter for field:
	 * {@snippet lang=c :
	 * tcflag_t c_lflag
	 * }
	 */
	public static void c_lflag(final MemorySegment struct, final int fieldValue) {
		try {
			c_lflag.invoke(struct, fieldValue);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Setter for field:
	 * {@snippet lang=c :
	 * tcflag_t c_oflag
	 * }
	 */
	public static void c_oflag(final MemorySegment struct, final int fieldValue) {
		try {
			c_oflag.invoke(struct, fieldValue);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Getter for field:
	 * {@snippet lang=c :
	 * cc_t c_cc[32]
	 * }
	 */
	public static MemorySegment c_cc(final MemorySegment struct) {
		try {
			return (MemorySegment) c_cc.invoke(struct);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}
}
