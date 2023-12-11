// MIT License
//
// Copyright (c) 2022, 2023 B. Malinowsky
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

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.lang.foreign.Arena;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledOnOs;
import org.junit.jupiter.api.condition.OS;
import org.junit.jupiter.api.io.TempDir;

@EnabledOnOs({ OS.LINUX, OS.MAC })
class UnixSerialPortTests {

	@TempDir
	private Path tmpDir;

	private UnixSerialPort port;

	private final Arena arena = Arena.ofConfined();


	@BeforeEach
	void setUp() {
		port = new UnixSerialPort();
	}

	@AfterEach
	void cleanup() {
		arena.close();
	}

	@Test
	void tryLink() throws IOException {
		final var file = tmpDir.resolve("fileToLink.txt");
		final var link = tmpDir.resolve("link.lnk");

		Files.createFile(file);

		final var forName = arena.allocateFrom(file.toString());
		final var linkName = arena.allocateFrom(link.toString());
		port.tryLink(forName, linkName);

		assertTrue(Files.exists(link));
	}

	@Test
	void ensureLock() throws IOException {
		final String portId = "/dev/ttyTestLock0";
		try {
			port.ensureLock(portId);
		}
		finally {
			port.releaseLock();
		}
	}

	@Test
	void portIdentifiers() {
		final var ports = UnixSerialPort.portIdentifiers();
		assertNotNull(ports);
		assertFalse(ports.isEmpty());
		System.out.println("found serial ports = " + ports);
	}
}
