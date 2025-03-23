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

import java.lang.foreign.Arena;
import java.lang.foreign.GroupLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.util.function.Consumer;

import serial.ffm.OS;

public final class dirent {
	private static final MethodHandle allocate;
	private static final MethodHandle reinterpret;
	private static final MethodHandle layout;
	private static final MethodHandle d_name;

	static {
		final var lookup = MethodHandles.lookup();
		final var alloc = methodType(MemorySegment.class, SegmentAllocator.class);
		final var getLayout = methodType(GroupLayout.class);
		final var name = methodType(MemorySegment.class, MemorySegment.class);
		final var reinterp = methodType(MemorySegment.class, MemorySegment.class, Arena.class, Consumer.class);
		try {
			switch (OS.current()) {
				case Linux -> {
					allocate    = lookup.findStatic(serial.ffm.linux.dirent.class, "allocate", alloc);
					reinterpret = lookup.findStatic(serial.ffm.linux.dirent.class, "reinterpret", reinterp);
					layout      = lookup.findStatic(serial.ffm.linux.dirent.class, "layout", getLayout);
					d_name      = lookup.findStatic(serial.ffm.linux.dirent.class, "d_name", name);
				}
				case Mac -> {
					allocate    = lookup.findStatic(serial.ffm.mac.dirent.class, "allocate", alloc);
					reinterpret = lookup.findStatic(serial.ffm.mac.dirent.class, "reinterpret", reinterp);
					layout      = lookup.findStatic(serial.ffm.mac.dirent.class, "layout", getLayout);
					d_name      = lookup.findStatic(serial.ffm.mac.dirent.class, "d_name", name);
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
	 * Reinterprets {@code addr} using target {@code arena} and {@code cleanupAction} (if any).
	 * The returned segment has size {@code layout().byteSize()}
	 */
	public static MemorySegment reinterpret(final MemorySegment addr, final Arena arena, final Consumer<MemorySegment> cleanup) {
		try {
			return (MemorySegment) reinterpret.invoke(addr, arena, cleanup);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * The layout of this struct
	 */
	public static GroupLayout layout() {
		try {
			return (GroupLayout) layout.invoke();
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}

	/**
	 * Getter for field:
	 * {@snippet lang=c :
	 * char d_name[256]
	 * }
	 */
	public static MemorySegment d_name(final MemorySegment struct) {
		try {
			return (MemorySegment) d_name.invoke(struct);
		} catch (final Throwable ex$) {
			throw new AssertionError("should not reach here", ex$);
		}
	}
}
