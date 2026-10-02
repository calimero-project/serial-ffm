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

import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.UUID;
import java.util.function.Predicate;
import java.util.regex.Pattern;

import serial.ffm.win.Windows;
import serial.ffm.win._DEVPROPKEY;
import serial.ffm.win._GUID;

import static java.lang.System.Logger.Level.DEBUG;
import static java.lang.System.Logger.Level.INFO;
import static java.nio.charset.StandardCharsets.UTF_16LE;
import static serial.ffm.win.Windows.CR_BUFFER_SMALL;
import static serial.ffm.win.Windows.CR_SUCCESS;

// Windows only: enumerate COM ports using CfgMgr32
final class WinCfgMgr {
	// Format IDs (GUIDs) for CfgMgr DevPropKeys
	private static final UUID GuidDevInterfaceComport        = UUID.fromString("86e0d1e0-8089-11d0-9ce4-08003e301f73");
	private static final UUID DeviceInterfacePropertiesFmtid = UUID.fromString("4c6bf15c-4c03-4aac-91f5-64c0f852bcf4");
	private static final UUID DeviceInterfaceFmtid           = UUID.fromString("026e516e-b814-414b-83cd-856d6fef4823");
	private static final UUID DeviceInstanceFmtid            = UUID.fromString("78c34fc8-104a-4aca-9ea4-524d52996e57");
	private static final UUID DeviceFmtid                    = UUID.fromString("a45c254e-df1c-4efd-8020-67d146a850e0");

	private record DevPropKey(UUID fmtid, int pid) {
		MemorySegment allocate(final Arena arena) {
			final var key = _DEVPROPKEY.allocate(arena);
			writeGuid(key.asSlice(_DEVPROPKEY.fmtid$offset()), fmtid);
			_DEVPROPKEY.pid(key, pid);
			return key;
		}
	}

	private static final DevPropKey DeviceInterface_FriendlyName    = new DevPropKey(DeviceInterfaceFmtid, 2);
	private static final DevPropKey DeviceInterface_Serial_PortName = new DevPropKey(DeviceInterfacePropertiesFmtid, 4);

	private static final DevPropKey Device_InstanceId   = new DevPropKey(DeviceInstanceFmtid, 256);

	private static final DevPropKey Device_DeviceDesc   = new DevPropKey(DeviceFmtid, 2);
	private static final DevPropKey Device_HardwareIds  = new DevPropKey(DeviceFmtid, 3);
	private static final DevPropKey Device_Service      = new DevPropKey(DeviceFmtid, 6);
	private static final DevPropKey Device_Manufacturer = new DevPropKey(DeviceFmtid, 13);
	private static final DevPropKey Device_FriendlyName = new DevPropKey(DeviceFmtid, 14);

	private static final Pattern UsbVidPidPattern = Pattern.compile("USB\\\\VID_([0-9A-Fa-f]{4})&PID_([0-9A-Fa-f]{4})");

	private static final System.Logger logger = System.getLogger("serial.ffm");


	private WinCfgMgr() {}

	static Set<SerialPortId> enumerate() {
		try (var arena = Arena.ofConfined()) {
			final var interfaceGuid = allocateGuid(arena, GuidDevInterfaceComport);
			final var interfaces = enumerateInterfaces(arena, interfaceGuid);

			final var ports = new HashSet<SerialPortId>();
			for (String interfacePath : interfaces)
				inspectPort(arena, interfacePath).ifPresent(ports::add);
			return ports;
		}
	}

	private static List<String> enumerateInterfaces(final Arena arena, final MemorySegment interfaceGuid) {
		while (true) {
			final var reqSize = arena.allocate(ValueLayout.JAVA_INT);
			int status = Windows.CM_Get_Device_Interface_List_SizeW(reqSize, interfaceGuid, MemorySegment.NULL,
					Windows.CM_GET_DEVICE_INTERFACE_LIST_PRESENT());
			throwOnError(status, "CM_Get_Device_Interface_List_SizeW");

			final int chars = reqSize.get(ValueLayout.JAVA_INT, 0);
			if (chars == 0)
				return List.of();

			final var buffer = arena.allocate((long) chars * 2);
			status = Windows.CM_Get_Device_Interface_ListW(interfaceGuid, MemorySegment.NULL, buffer, chars,
					Windows.CM_GET_DEVICE_INTERFACE_LIST_PRESENT());
			if (status == CR_BUFFER_SMALL())
				continue;

			throwOnError(status, "CM_Get_Device_Interface_ListW");
			return readMultiString(buffer);
		}
	}

	private static Optional<SerialPortId> inspectPort(final Arena arena, final String interfacePath) {
		try {
			final var interfaceName = arena.allocateFrom(interfacePath, UTF_16LE);

			final String interfaceFriendlyName = interfaceStringProperty(arena, interfaceName,
					DeviceInterface_FriendlyName).orElse(null);

			final var devInstIdValue = interfaceStringPropertyValue(arena, interfaceName, Device_InstanceId).orElse(null);
			if (devInstIdValue == null)
				return Optional.empty();
			// obtain device instance _handle_ to device node that is associated with specified device instance _ID_
			final var devInstSeg = arena.allocate(ValueLayout.JAVA_INT);
			final int status = Windows.CM_Locate_DevNodeW(devInstSeg, devInstIdValue.data(), Windows.CM_LOCATE_DEVNODE_NORMAL());
			if (status != CR_SUCCESS()) {
				logger.log(DEBUG, "locating device node for {0} failed: {1}", interfacePath, configRetMessage(status));
				return Optional.empty();
			}
			final int devInst = devInstSeg.get(ValueLayout.JAVA_INT, 0);

			final String portName = interfaceStringProperty(arena, interfaceName, DeviceInterface_Serial_PortName)
					.orElseGet(() -> portName(arena, devInst));
			if (portName == null)
				return Optional.empty();
			final String friendlyName = devNodeStringProperty(arena, devInst, Device_FriendlyName).orElse(interfaceFriendlyName);
			final String description = devNodeStringProperty(arena, devInst, Device_DeviceDesc).orElse(null);
			final String manufacturer = devNodeStringProperty(arena, devInst, Device_Manufacturer).orElse(null);
			final String service = devNodeStringProperty(arena, devInst, Device_Service).orElse(null);
			final List<String> hardwareIds = devNodeStringListProperty(arena, devInst, Device_HardwareIds);

			// maybe device is attached via usb
			final var devInstId = devInstIdValue.string();
			final UsbId usb = parseUsbIdentity(devInstId, hardwareIds);
			final var serialNumber = parseSerialNumber(devInstId).orElse(null);

			final var info = new SerialPortInfo(manufacturer, description, serialNumber, service, usb.vendorId(),
					usb.productId(), interfacePath, devInstId, friendlyName , hardwareIds);
			return Optional.of(new DefaultSerialPortId(portName, "\\\\.\\" + portName, info));
		}
		catch (final RuntimeException e) {
			logger.log(INFO, "error inspecting {0}", interfacePath, e);
			return Optional.empty();
		}
	}

	private static String portName(final Arena arena, final int devInst) {
		final var phKey = arena.allocate(ValueLayout.ADDRESS);
		int status = Windows.CM_Open_DevNode_Key(devInst, Windows.KEY_READ(), 0,
				Windows.RegDisposition_OpenExisting(), phKey, Windows.CM_REGISTRY_HARDWARE());
		if (status != CR_SUCCESS() || phKey.get(ValueLayout.ADDRESS, 0).equals(MemorySegment.NULL)) {
			logger.log(INFO, "opening registry for querying port name failed: {0}", configRetMessage(status));
			return null;
		}
		final var hKey = phKey.get(ValueLayout.ADDRESS, 0);
		try {
			final var valueName = arena.allocateFrom("PortName", UTF_16LE);
			final var type = arena.allocate(ValueLayout.JAVA_INT);
			final var dataSize = arena.allocate(ValueLayout.JAVA_INT);
			status = Windows.RegQueryValueExW(hKey, valueName, Windows.NULL(), type, Windows.NULL(), dataSize);
			if (status == Windows.ERROR_SUCCESS()) {
				if (type.get(ValueLayout.JAVA_INT, 0) != Windows.REG_SZ())
					return null;
				final var buffer = arena.allocate(dataSize.get(ValueLayout.JAVA_INT, 0));
				status = Windows.RegQueryValueExW(hKey, valueName, Windows.NULL(), type, buffer, dataSize);
				if (status == Windows.ERROR_SUCCESS())
					return buffer.getString(0, UTF_16LE);
			}
			logger.log(INFO, "querying port name failed: {0}", WinSerialPort.formatWinError(status));
		}
		finally {
			Windows.RegCloseKey(hKey);
		}
		return null;
	}

	private static MemorySegment allocateGuid(final Arena arena, final UUID uuid) {
		final var guid = _GUID.allocate(arena);
		writeGuid(guid, uuid);
		return guid;
	}

	// converts UUID to GUID (little-endian format)
	private static void writeGuid(final MemorySegment guid, final UUID uuid) {
		final long most = uuid.getMostSignificantBits();
		final long least = uuid.getLeastSignificantBits();

		_GUID.Data1(guid, (int) (most >>> 32));
		_GUID.Data2(guid, (short) (most >>> 16));
		_GUID.Data3(guid, (short) most);
		for (int i = 0; i < 8; i++)
			_GUID.Data4(guid, i, (byte) (least >>> (56 - i * 8)));
	}

	// returns a list of strings from a buffer in MULTI_SZ format
	private static List<String> readMultiString(final MemorySegment buffer) {
		final var strings = new ArrayList<String>();
		for (int offset = 0; offset < buffer.byteSize(); ) {
			final String s = buffer.getString(offset, UTF_16LE);
			if (s.isEmpty())
				break;
			strings.add(s);
			offset += 2 * s.length() + 2;
		}
		return strings;
	}

	private static void throwOnError(final int status, final String prefix) {
		if (status != CR_SUCCESS())
			throw new RuntimeException("%s: %s".formatted(prefix, configRetMessage(status)));
	}

	private record UsbId(int vendorId, int productId) {
		UsbId(final String vendorId, final String productId) {
			this(Integer.parseUnsignedInt(vendorId, 16), Integer.parseUnsignedInt(productId, 16));
		}
		static UsbId none() { return new UsbId(0, 0); }
	}

	private static UsbId parseUsbIdentity(final String instId, final List<String> hardwareIds) {
		var matcher = UsbVidPidPattern.matcher(instId);
		if (matcher.find())
			return new UsbId(matcher.group(1), matcher.group(2));
		for (String hwId : hardwareIds) {
			matcher = UsbVidPidPattern.matcher(hwId);
			if (matcher.find())
				return new UsbId(matcher.group(1), matcher.group(2));
		}
		return UsbId.none();
	}

	private static Optional<String> parseSerialNumber(final String instId) {
		if (!instId.startsWith("USB\\"))
			return Optional.empty();
		final int lastBackslash = instId.lastIndexOf('\\');
		if (lastBackslash == -1 || lastBackslash == instId.length() - 1)
			return Optional.empty();
		return Optional.of(instId.substring(lastBackslash + 1));
	}

	private static Optional<PropertyValue> interfaceStringPropertyValue(final Arena arena, final MemorySegment interfaceName,
			final DevPropKey propKey) {
		final var propertyKey = propKey.allocate(arena);
		final var value = getProperty(arena, (type, buffer, size) ->
				Windows.CM_Get_Device_Interface_PropertyW(interfaceName, propertyKey, type, buffer, size, 0));
		return value != null && value.type() == Windows.DEVPROP_TYPE_STRING() ? Optional.of(value) : Optional.empty();
	}

	private static Optional<String> interfaceStringProperty(final Arena arena, final MemorySegment interfaceName,
			final DevPropKey propKey) {
		return interfaceStringPropertyValue(arena, interfaceName, propKey).map(PropertyValue::string).filter(
				Predicate.not(String::isEmpty));
	}

	private static Optional<String> devNodeStringProperty(final Arena arena, final int devInst,
			final DevPropKey propKey) {
		final var value = devNodeProperty(arena, devInst, propKey);
		if (value == null || value.type() != Windows.DEVPROP_TYPE_STRING())
			return Optional.empty();

		final String result = value.string();
		return result.isEmpty() ? Optional.empty() : Optional.of(result);
	}

	private static List<String> devNodeStringListProperty(final Arena arena, final int devInst,
			final DevPropKey propKey) {
		final var value = devNodeProperty(arena, devInst, propKey);
		if (value == null || value.type() != Windows.DEVPROP_TYPE_STRING_LIST())
			return List.of();
		return value.stringList();
	}

	private static PropertyValue devNodeProperty(final Arena arena, final int devInst, final DevPropKey propKey) {
		final var propertyKey = propKey.allocate(arena);
		return getProperty(arena, (type, buffer, size) ->
				Windows.CM_Get_DevNode_PropertyW(devInst, propertyKey, type, buffer, size, 0));
	}

	@FunctionalInterface
	private interface PropertyAccessor {
		int get(MemorySegment propertyType, MemorySegment propertyBuffer, MemorySegment propertyBufferSize);
	}

	private record PropertyValue(int type, MemorySegment data) {
		String string() { return data.getString(0, StandardCharsets.UTF_16LE); }

		List<String> stringList() { return readMultiString(data); }
	}

	private static PropertyValue getProperty(final Arena arena, final PropertyAccessor getter) {
		final var propertyType = arena.allocate(ValueLayout.JAVA_INT);
		final var propertySize = arena.allocate(ValueLayout.JAVA_INT);

		// query required buffer size
		int status = getter.get(propertyType, MemorySegment.NULL, propertySize);
		if (status != CR_SUCCESS() && status != CR_BUFFER_SMALL()) {
			logger.log(DEBUG, "querying size of property value failed: {0}", configRetMessage(status));
			return null;
		}

		// retry in case size of property value changes between calls
		for (int attempt = 0; attempt < 5; attempt++) {
			final int size = propertySize.get(ValueLayout.JAVA_INT, 0);
			if (size <= 0)
				break;
			final var buffer = arena.allocate(size);
			status = getter.get(propertyType, buffer, propertySize);
			if (status == CR_SUCCESS())
				return new PropertyValue(propertyType.get(ValueLayout.JAVA_INT, 0),
						buffer.asSlice(0, propertySize.get(ValueLayout.JAVA_INT, 0)));
			if (status != CR_BUFFER_SMALL())
				break;
		}
		return null;
	}

	// formats a CfgMgr result (CR_) code into a human-readable msg, suffixed with its result code
	// we don't use CM_MapCrToWin32Err, as that mapping leads to very generic error msgs
	private static String configRetMessage(final int resultCode) {
		final String msg = switch (resultCode) {
			case 0x00 -> "success";
			case 0x01 -> "default value/action";
			case 0x02 -> "out of memory";
			case 0x03 -> "invalid pointer";
			case 0x04 -> "invalid flag";
			case 0x05 -> "invalid device node handle";
			case 0x06 -> "invalid resource descriptor handle";
			case 0x07 -> "invalid logical configuration handle";
			case 0x08 -> "invalid arbitrator handle";
			case 0x09 -> "invalid node list handle";
			case 0x0a -> "device node has configuration requirements";
			case 0x0b -> "invalid resource type or description";
//			case 0x0c -> "device loader VxD not found"; // not used (Win 95)
			case 0x0d -> "no such device node";
			case 0x0e -> "no more logical configurations";
			case 0x0f -> "no more resource descriptors";
			case 0x10 -> "device node already exists";
			case 0x11 -> "invalid range list handle";
			case 0x12 -> "invalid range";
			case 0x13 -> "general internal failure";
			case 0x14 -> "no such logical device instance";
			case 0x15 -> "creation of device node blocked";
//			case 0x16 -> "no system VM"; // not used (Win 95)
			case 0x17 -> "removal operation vetoed";
			case 0x18 -> "vetoed by application-level subsystem";
			case 0x19 -> "invalid system power type";
			case 0x1a -> "buffer too small";
			case 0x1b -> "no arbitrator registered";
			case 0x1c -> "no registry handle";
			case 0x1d -> "registry read/write error";
			case 0x1e -> "invalid device ID";
			case 0x1f -> "invalid configuration data";
			case 0x20 -> "invalid CfgMgr API";
			case 0x21 -> "device loader not ready";
			case 0x22 -> "restart required";
			case 0x23 -> "no more hardware profiles";
			case 0x24 -> "physical device not present";
			case 0x25 -> "no such value";
			case 0x26 -> "wrong type";
			case 0x27 -> "invalid configuration priority";
			case 0x28 -> "device cannot be disabled";
			case 0x29 -> "free resources";
			case 0x2a -> "query vetoed";
			case 0x2b -> "cannot share IRQ";
			case 0x2c -> "no dependent device/resource";
			case 0x2d -> "same resources";
			case 0x2e -> "no such registry key";
			case 0x2f -> "invalid machine name";
			case 0x30 -> "remote communication failure";
			case 0x31 -> "remove machine unavailable";
			case 0x32 -> "configuration manager services unavailable";
			case 0x33 -> "access denied";
			case 0x34 -> "call not implemented";
			case 0x35 -> "invalid property";
			case 0x36 -> "device interface active";
			case 0x37 -> "no such device interface";
			case 0x38 -> "invalid reference string";
			case 0x39 -> "invalid conflict list handle";
			case 0x3a -> "invalid index";
			case 0x3b -> "invalid structure size";
			default   -> "unknown error";
		};
		return "%s (0x%08x)".formatted(msg, resultCode);
	}
}
