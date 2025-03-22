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

package serial.ffm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Duration;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;

import serial.ffm.SerialPort.FlowControl;
import serial.ffm.SerialPort.Parity;
import serial.ffm.SerialPort.Status;
import serial.ffm.SerialPort.StopBits;
import serial.ffm.SerialPort.Timeouts;

class SerialPortTests {

	@TempDir
	private Path tmpDir;

	private final String portId = osPort();

	private static String osPort() {
		return switch (OS.current()) {
			case Mac     -> "/dev/tty.usbmodem14101";
			case Linux   -> "/dev/ttyACM0";
			case Windows -> "COM1";
			case Other   -> throw new UnsupportedOperationException("unsupported platform: " + OS.osName());
		};
	}

	private SerialPort port;

	@BeforeEach
	void setUp() throws Exception {
		Thread.sleep(Duration.ofMillis(400));
		port = SerialPort.open(portId);
	}

	@AfterEach
	void cleanup() {
		port.close();
	}

	@Test
	void existingPortExists() {
		assertTrue(SerialPort.portExists(portId));
	}

	@Test
	void nonexistingPortDoesntExist() {
		assertFalse(SerialPort.portExists("/dev/ttyXYZ"));
	}

	@Test
	void openPort() throws IOException {
		port.close();
		try (var __ = SerialPort.open(portId)) {}
		try (var __ = SerialPort.open(portId)) {}
		try (var __ = SerialPort.open(portId)) {}
	}

	@Test
	void close() {
		port.close();
	}

	@Test
	void openPortWithInvalidName() {
		try {
			assertThrows(IOException.class, () -> SerialPort.open("\\dev/ttys0"));
		}
		finally {
			port.close();
		}
	}

	@ParameterizedTest
	@ValueSource(ints = { 9_600, 19_200, 115_200 })
	void baudRate(final int baudrate) throws IOException, InterruptedException {
		Thread.sleep(Duration.ofMillis(1000));
		port.baudRate(baudrate);
		assertEquals(baudrate, port.baudRate());
	}

	@ParameterizedTest
	@ValueSource(ints = { 5, 6, 7, 8 })
	void dataBits(final int databits) throws IOException {
		port.dataBits(databits);
		assertEquals(databits, port.dataBits());
	}

	@ParameterizedTest
	@EnumSource(value = StopBits.class)
	void stopBits(final StopBits stopbits) throws IOException {
		port.stopBits(stopbits);
		assertEquals(stopbits, port.stopBits());
	}

	@ParameterizedTest
	@EnumSource(value = Parity.class)
	void parity(final Parity parity) throws IOException {
		port.parity(parity);
		assertEquals(parity, port.parity());
	}

	@ParameterizedTest
	@EnumSource(value = FlowControl.class)
	void flowControl(final FlowControl flowControl) throws IOException {
		port.flowControl(flowControl);
		assertEquals(flowControl, port.flowControl());
	}

	@Test
	void timeouts() throws IOException {
		final var set = new Timeouts(5, 0, 0, 0, 0);
		port.timeouts(set);
		final ReadWritePort rwport = (ReadWritePort) port;
		final var get = rwport.timeouts();
		assertEquals(set, get);
//		final var zeros = new Timeouts(0, 0, 0, 0, 0);
//		port.timeouts(zeros);
	}

	@Test
	void inputStream() {
		assertNotNull(port.inputStream());
	}

	@Test
	void outputStream() {
		assertNotNull(port.outputStream());
	}

	@Test
	void setEvents() throws IOException {
		final int eventMask = UnixSerialPort.EVENT_CTS | UnixSerialPort.EVENT_TXEMPTY;
		port.setEvents(eventMask, true);
	}

	@Test
	void waitEvent() throws IOException {
		final int eventMask = UnixSerialPort.EVENT_CTS | UnixSerialPort.EVENT_TXEMPTY | UnixSerialPort.EVENT_RING
				| UnixSerialPort.EVENT_BREAK | UnixSerialPort.EVENT_CTS | UnixSerialPort.EVENT_DSR
				| UnixSerialPort.EVENT_RLSD | UnixSerialPort.EVENT_RXCHAR | UnixSerialPort.EVENT_RXFLAG;
		port.setEvents(eventMask, true);
		// TODO commented out because waiting for events never returns
//		final int events = port.waitEvent();
//		((UnixSerialPort) port).dispatchEvents(events);
	}

	@Test
	void status() throws IOException {
		assertEquals(0, port.status(Status.AvailableInput));
		assertEquals(0, port.status(Status.Error));
		assertEquals(0, port.status(Status.Line));
	}

	@Test
	void read() throws IOException {
		try {
			final var timeouts = new Timeouts(5, 0, 1000, 0, 0);
			port.timeouts(timeouts);
			port.inputStream().read();
		}
		finally {
			port.close();
		}
	}

	@Test
	void write() throws IOException {
		port.outputStream().write(0);
	}

	@Test
	void writeArray() throws IOException {
		port.outputStream().write(new byte[]{ 1, 2, 3 });
	}

	@Test
	void chain() throws IOException {
		try (var __ = port
				.baudRate(19_200)
				.dataBits(8)
				.parity(Parity.Even)
				.stopBits(StopBits.One)
				.flowControl(FlowControl.None)) {
			/*final var in =*/ port.inputStream();
			/*final var out =*/ port.outputStream();
		}
	}
}
