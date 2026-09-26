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
import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import serial.ffm.mac.Mac;

import static java.lang.foreign.ValueLayout.JAVA_INT;
import static java.lang.foreign.ValueLayout.JAVA_LONG;
import static serial.ffm.mac.Mac.KERN_SUCCESS;

// macOS only: use IOKit to enumerate serial ports
final class MacIOKit {
	private static final MemorySegment kCFAllocatorDefault = Mac.CFAllocatorGetDefault();

	record CFTypeRef(MemorySegment ref) implements AutoCloseable {
		@Override
		public void close() {
			if (!ref.equals(MemorySegment.NULL))
				Mac.CFRelease(ref);
		}
	}

	record IOObjHandle(int handle) implements AutoCloseable {
		@Override
		public void close() { Mac.IOObjectRelease(handle); }
	}

	private static final class UsbProperties {
		String manufacturer, product, serialNumber;
		int vendorId, productId, interfaceNumber;
	}


	private MacIOKit() {}

	static HashSet<SerialPortId> enumerate() throws IOException {
		final HashSet<SerialPortId> ports = new HashSet<>();
		try (var arena = Arena.ofConfined()) {
			final var dict = Mac.IOServiceMatching(Mac.kIOSerialBSDServiceValue());
			if (dict.equals(MemorySegment.NULL))
				return ports;

			final var iterPtr = arena.allocate(JAVA_INT);
			final int kr = Mac.IOServiceGetMatchingServices(0, dict, iterPtr);
			if (kr != KERN_SUCCESS())
				throw new IOException("error calling IOServiceGetMatchingServices: " + kr);
			try (final var iter = new IOObjHandle(iterPtr.get(JAVA_INT, 0))) {
				if (iter.handle() == 0)
					return ports;
				while (true) {
					try (var service = new IOObjHandle(Mac.IOIteratorNext(iter.handle()))) {
						if (service.handle() == 0)
							break;
						inspectPort(arena, service.handle()).ifPresent(ports::add);
					}
				}
			}
		}
		return ports;
	}

	private static Optional<SerialPortId> inspectPort(final Arena arena, final int service) {
		final String port = stringProperty(arena, service, Mac.kIOCalloutDeviceKey())
				.or(() -> stringProperty(arena, service, Mac.kIODialinDeviceKey())).orElse(null);
		if (port == null)
			return Optional.empty();
		final String ioClass = ioClass(arena, service).orElse(null);
		final String registryPath = registryPath(arena, service, Mac.kIOServiceClass()).orElse(null);
		final long registryEntryId = registryEntryId(arena, service);

		// USB serial drivers have the serial client below the USB interface; vendor/product ID can be found at a parent
		final var usb = new UsbProperties();
		walkParents(arena, new IOObjHandle(service), usb, 16); // cap ancestors at some arbitrary limit

		final String portName = portName(port);
		if (usb.product == null)
			usb.product = portName;
		final var info = new SerialPortInfo(usb.manufacturer, usb.product, usb.serialNumber, ioClass, usb.vendorId,
				usb.productId, registryPath, Long.toUnsignedString(registryEntryId), null, List.of());
		return Optional.of(new SerialPortId(portName, port, info));
	}

	private static void walkParents(final Arena arena, final IOObjHandle entry, final UsbProperties usb, final int remaining) {
		if (remaining <= 0)
			return;
		try (final var parent = parent(arena, entry, Mac.kIOServiceClass())) {
			if (parent.handle() == 0)
				return;
			inspectUsbProperties(arena, parent.handle(), usb);
			walkParents(arena, parent, usb, remaining - 1);
		}
	}

	private static IOObjHandle parent(final Arena arena, final IOObjHandle entry, final MemorySegment plane) {
		final var parent = arena.allocate(JAVA_INT);
		return new IOObjHandle(Mac.IORegistryEntryGetParentEntry(entry.handle(), plane, parent) == KERN_SUCCESS() ?
				parent.get(JAVA_INT, 0) : 0);
	}

	private static void inspectUsbProperties(final Arena arena, final int entry, final UsbProperties usb) {
		if (usb.vendorId == 0)
			usb.vendorId = intProperty(entry, Mac.kUSBVendorID(), arena).orElse(0);
		if (usb.productId == 0)
			usb.productId = intProperty(entry, Mac.kUSBProductID(), arena).orElse(0);
		if (usb.interfaceNumber == 0)
			usb.interfaceNumber = intProperty(entry, Mac.kUSBInterfaceNumber(), arena).orElse(0);
		if (usb.manufacturer == null)
			usb.manufacturer = stringProperty(arena, entry, Mac.kUSBVendorString()).orElse(null);
		if (usb.product == null)
			usb.product = stringProperty(arena, entry, Mac.kUSBProductString())
					.or(() -> stringProperty(arena, entry, Mac.kIOPropertyProductNameKey()))
					.orElse(null);
		if (usb.serialNumber == null)
			usb.serialNumber = stringProperty(arena, entry, Mac.kUSBSerialNumberString()).orElse(null);
	}

	private static Optional<String> stringProperty(final Arena arena, final int entry, final MemorySegment property) {
		try (final var key = cfString(property)) {
			if (key.ref().equals(MemorySegment.NULL))
				return Optional.empty();
			try (final var value = new CFTypeRef(Mac.IORegistryEntryCreateCFProperty(entry, key.ref(), kCFAllocatorDefault, 0))) {
				if (value.ref().equals(MemorySegment.NULL))
					return Optional.empty();
				return javaString(arena, value.ref());
			}
		}
	}

	private static Optional<Long> longProperty(final Arena arena, final int entry, final MemorySegment property) {
		try (final var key = cfString(property)) {
			if (key.ref().equals(MemorySegment.NULL))
				return Optional.empty();
			try (final var value = new CFTypeRef(Mac.IORegistryEntryCreateCFProperty(entry, key.ref(), kCFAllocatorDefault, 0))) {
				if (value.ref().equals(MemorySegment.NULL))
					return Optional.empty();
				return javaLong(arena, value.ref());
			}
		}
	}

	private static Optional<Integer> intProperty(final int entry, final MemorySegment property, final Arena arena) {
		return longProperty(arena, entry, property).map(Math::toIntExact);
	}

	private static Optional<String> ioClass(final Arena arena, final int entry) {
		final var className = arena.allocate(256);
		return Mac.IOObjectGetClass(entry, className) == KERN_SUCCESS() ?
				Optional.of(className.getString(0)) : Optional.empty();
	}

	private static Optional<String> registryPath(final Arena arena, final int entry, final MemorySegment plane) {
		final var path = arena.allocate(2048);
		return Mac.IORegistryEntryGetPath(entry, plane, path) == KERN_SUCCESS() ?
				Optional.of(path.getString(0)) : Optional.empty();
	}

	private static long registryEntryId(final Arena arena, final int entry) {
		final var entryId = arena.allocate(JAVA_LONG);
		return Mac.IORegistryEntryGetRegistryEntryID(entry, entryId) == KERN_SUCCESS() ? entryId.get(JAVA_LONG, 0) : 0;
	}

	private static CFTypeRef cfString(final MemorySegment cString) {
		return new CFTypeRef(Mac.CFStringCreateWithCString(kCFAllocatorDefault, cString, Mac.kCFStringEncodingUTF8()));
	}

	private static Optional<String> javaString(final Arena arena, final MemorySegment cfString) {
		if (Mac.CFGetTypeID(cfString) != Mac.CFStringGetTypeID())
			return Optional.empty();
		final var buffer = arena.allocate(1024);
		if (Mac.CFStringGetCString(cfString, buffer, buffer.byteSize(), Mac.kCFStringEncodingUTF8()) != 0)
			return Optional.of(buffer.getString(0));
		return Optional.empty();
	}

	private static Optional<Long> javaLong(final Arena arena, final MemorySegment cfNumber) {
		if (Mac.CFGetTypeID(cfNumber) != Mac.CFNumberGetTypeID())
			return Optional.empty();
		final var result = arena.allocate(JAVA_LONG);
		if (Mac.CFNumberGetValue(cfNumber, Mac.kCFNumberSInt64Type(), result) != 0)
			return Optional.of(result.get(JAVA_LONG, 0));
		return Optional.empty();
	}

	private static String portName(final String port) {
		final int lastSlash = port.lastIndexOf('/');
		return port.substring(lastSlash == -1 ? 0 : lastSlash + 1);
	}
}
