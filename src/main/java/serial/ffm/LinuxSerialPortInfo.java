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

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

final class LinuxSerialPortInfo {
	private LinuxSerialPortInfo() {}

	static SerialPortInfo read(final Optional<Path> sysfsDevicePath, final String driver) {
		final Optional<Path> usbDevice = sysfsDevicePath.flatMap(LinuxSerialPortInfo::findUsbInterface)
				.flatMap(LinuxSerialPortInfo::findUsbDevice);
		if (usbDevice.isEmpty())
			return new SerialPortInfo(null, null, null, driver, 0, 0, null, null, null, List.of());

		final var node = usbDevice.get();
		return new SerialPortInfo(
				readAttribute(node, "manufacturer").orElse(null),
				readAttribute(node, "product").orElse(null),
				readAttribute(node, "serial").orElse(null),
				driver,
				readAttribute(node, "idVendor").map(id ->Integer.parseUnsignedInt(id, 16)).orElse(0),
				readAttribute(node, "idProduct").map(id ->Integer.parseUnsignedInt(id, 16)).orElse(0),
				null, null, null, List.of());
	}

	private static Optional<Path> findUsbInterface(final Path devicePath) {
		for (Path path = devicePath; path != null; path = path.getParent())
			if (Files.isRegularFile(path.resolve("bInterfaceClass"))
					&& Files.isRegularFile(path.resolve("bInterfaceNumber"))
					&& Files.isRegularFile(path.resolve("bInterfaceSubClass")))
				return Optional.of(path);
		return Optional.empty();
	}

	private static Optional<Path> findUsbDevice(final Path interfaceNode) {
		final Path parent = interfaceNode.getParent();
		if (parent != null && Files.isRegularFile(parent.resolve("idVendor"))
				&& Files.isRegularFile(parent.resolve("idProduct"))
				&& Files.isRegularFile(parent.resolve("bDeviceClass")))
			return Optional.of(parent);
		return Optional.empty();
	}

	private static Optional<String> readAttribute(final Path node, final String attr) {
		try {
			final String value = Files.readString(node.resolve(attr)).trim();
			return value.isEmpty() ? Optional.empty() : Optional.of(value);
		}
		catch (final IOException ignore) {
			return Optional.empty();
		}
	}
}
