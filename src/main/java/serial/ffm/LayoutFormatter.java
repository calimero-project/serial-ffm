// MIT License
//
// Copyright (c) 2022, 2024 B. Malinowsky
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

package serial.ffm;

import java.io.StringWriter;
import java.lang.foreign.GroupLayout;
import java.lang.foreign.MemoryLayout;
import java.lang.foreign.MemoryLayout.PathElement;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.VarHandle;

/** Pretty-prints memory segments with layouts for debugging. */
final class LayoutFormatter {
	private LayoutFormatter() {}

	static String format(final MemorySegment seg, final MemoryLayout layout) {
		final var writer = new StringWriter();
		try {
			format(seg, layout, writer, "");
		}
		catch (final Exception e) {
			System.err.println(e.getMessage());
		}
		return writer.toString();
	}

	private static void format(final MemorySegment seg, final MemoryLayout layout, final StringWriter writer,
			final String indent) {
		if (layout instanceof final GroupLayout gl) {
			writer.write(indent + layout.name().get() + " {\n");
			for (final var member : gl.memberLayouts()) {
				if (member.name().isPresent()) {
					final String name = member.name().get();
					final long offset = gl.byteOffset(PathElement.groupElement(name));
					format(seg.asSlice(offset, member.byteSize()), member, writer, indent + "\t");
				}
			}
			writer.write(indent + "}\n");
		}
		else if (layout instanceof final ValueLayout vl) {
			final var name = vl.name().get();
			final VarHandle varHandle = vl.varHandle();
			final Object o = varHandle.get(seg, 0);
			writer.write(indent + name + "\t= " + o + "\n");
		}
	}
}
