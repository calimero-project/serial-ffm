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
import java.lang.invoke.MethodHandles;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

// Linux only: enumerate /sys/class/tty for serial ports
final class LinuxSysfsTty {
	static final Path SysClassTty = Path.of("/sys/class/tty");

	private static final System.Logger logger = System.getLogger(MethodHandles.lookup().lookupClass().getPackageName());

	private LinuxSysfsTty() {}

	static boolean available() {
		try {
			return "sysfs".equals(Files.getFileStore(SysClassTty).type());
		}
		catch (final IOException e) {
			return false;
		}
	}

	static Set<SerialPortId> enumerate() throws IOException {
		try (final var entries = Files.list(SysClassTty)) {
			return entries.map(LinuxSysfsTty::inspectPort).flatMap(Optional::stream).collect(Collectors.toSet());
		}
	}

	private static Optional<SerialPortId> inspectPort(final Path classPath) {
		final Path fileName = classPath.getFileName();
		final Path devPath = Path.of("/dev").resolve(fileName);

		// check if sysfs entry is actually here
		if (!Files.exists(devPath))
			return Optional.empty();

		try {
			final var driverOpt = filterSerialTty(classPath);
			if (driverOpt.isPresent())
				return Optional.of(new DefaultSerialPortId(fileName.toString(), devPath.toString(),
						LinuxSerialPortInfo.read(resolveSysfsDevicePath(classPath), driverOpt.get())));
		}
		catch (IOException | RuntimeException e) {
			logger.log(System.Logger.Level.DEBUG, "error inspecting " + classPath, e);
		}
		return Optional.empty();
	}

	// resolve device symlink to see which kernel device is responsible for this tty
	private static Optional<Path> resolveSysfsDevicePath(final Path classPath) throws IOException {
		final Path device = classPath.resolve("device");
		return Files.isSymbolicLink(device) ? Optional.of(device.toRealPath()) : Optional.empty();
	}

	// returns the driver name if it is a serial tty
	static Optional<String> filterSerialTty(final Path classPath) throws IOException {
		// enumerate /sys/class/tty, but verify that the entry really belongs to the tty subsystem
		if (!isTtySubsystem(classPath))
			return Optional.empty();

		// we can use the "active" attribute for identifying configured/active virtual consoles
		if (Files.isRegularFile(classPath.resolve("active")))
			return Optional.empty();

		// skip PTY master multiplex
		if (isPtmx(classPath))
			return Optional.empty();

		return findTtyDriver(classPath);
	}

	private static boolean isTtySubsystem(final Path classPath) throws IOException {
		final Path subsystem = classPath.resolve("subsystem");
		if (!Files.isSymbolicLink(subsystem))
			return false;
		final Path name = subsystem.toRealPath().getFileName();
		return name != null && "tty".equals(name.toString());
	}

	private static boolean isPtmx(final Path classPath) throws IOException {
		final Path uevent = classPath.resolve("uevent");
		final var lines = Files.readAllLines(uevent);
		return lines.contains("MAJOR=5") && lines.contains("MINOR=2");
	}

	private static Optional<String> findTtyDriver(final Path classPath) throws IOException {
		Path driver = classPath.resolve("driver");
		if (!Files.isSymbolicLink(driver))
			driver = classPath.resolve("device/driver");
		if (!Files.isSymbolicLink(driver))
			return Optional.empty();
		final Path name = driver.toRealPath().getFileName();
		return name != null ? Optional.of(name.toString()) : Optional.empty();
	}
}
