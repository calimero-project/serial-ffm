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

package serial.ffm;

import java.util.List;
import java.util.Optional;
import java.util.StringJoiner;

// TODO maybe better provide info as map, containing only available key=value pairs, and merge with SerialPortId
record SerialPortInfo(
		String manufacturer, String description, String serialNumber, String driver, int vendorId, int productId,
		// win/mac
		String interfacePath, String instanceId,
		// win
		String friendlyName, List<String> hardwareIds) {

	SerialPortInfo {
		hardwareIds = List.copyOf(hardwareIds);
	}

	@Override
	public String toString() {
		final var joiner = new StringJoiner(", ");
		try {
			for (var component : getClass().getRecordComponents())
				Optional.ofNullable(component.getAccessor().invoke(this))
						.filter(v -> !(v instanceof List<?> l && l.isEmpty())) // omit empty list of hardwareIds
						.map(v -> component.getName() + "=" + v).ifPresent(joiner::add);
		} catch (final ReflectiveOperationException e) {
			throw new IllegalStateException(e);
		}
		return joiner.toString();
	}
}
