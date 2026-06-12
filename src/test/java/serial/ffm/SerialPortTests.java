// MIT License
//
// Copyright (c) 2022, 2026 B. Malinowsky
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

import static org.junit.jupiter.api.AssertionFailureBuilder.assertionFailure;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Duration;
import java.util.EnumSet;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.function.Executable;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.api.util.RestoreSystemProperties;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;
import org.opentest4j.TestAbortedException;

import serial.ffm.SerialPort.FlowControl;
import serial.ffm.SerialPort.Parity;
import serial.ffm.SerialPort.Status;
import serial.ffm.SerialPort.StopBits;
import serial.ffm.SerialPort.Timeouts;

class SerialPortTests {

	@TempDir
	private Path tmpDir;

	private final String portId = osPort();

	static String osPort() {
		return switch (OS.current()) {
			case Mac     -> "/dev/tty.usbmodem21201";
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
		if (port != null)
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
	void openPort() throws IOException, InterruptedException {
		port.close();
		Thread.sleep(Duration.ofMillis(50));
		try (var _ = SerialPort.open(portId)) {}
		Thread.sleep(Duration.ofMillis(50));
		try (var _ = SerialPort.open(portId)) {}
		Thread.sleep(Duration.ofMillis(50));
		try (var _ = SerialPort.open(portId)) {}
	}

	@Test
	void close() {
		port.close();
	}

	@Test
	void useAfterClose() throws IOException, InterruptedException {
		port.close();
		port.close(); // idempotent, shouldn't throw
		assertThrows(PortClosedException.class, () -> port.baudRate());
		assertThrows(PortClosedException.class, () -> port.baudRate(19_200));
		assertThrows(PortClosedException.class, () -> port.dataBits());
		assertThrows(PortClosedException.class, () -> port.dataBits(8));
		assertThrows(PortClosedException.class, () -> port.flowControl());
		assertThrows(PortClosedException.class, () -> port.flowControl(FlowControl.None));
		assertThrows(PortClosedException.class, () -> port.parity());
		assertThrows(PortClosedException.class, () -> port.parity(Parity.None));
		assertThrows(PortClosedException.class, () -> port.status(Status.AvailableInput));
		assertThrows(PortClosedException.class, () -> port.status(Status.Error));
		assertThrows(PortClosedException.class, () -> port.status(Status.Line));
		assertThrows(PortClosedException.class, () -> port.stopBits());
		assertThrows(PortClosedException.class, () -> port.stopBits(StopBits.One));
		assertThrows(PortClosedException.class, () -> port.events(EnumSet.of(SerialPort.SerialEvent.Break), true));
		assertThrows(PortClosedException.class, () -> port.waitEvent());
		assertThrows(PortClosedException.class, () -> port.inputStream().available());
		assertThrows(PortClosedException.class, () -> port.inputStream().read());
		assertThrows(PortClosedException.class, () -> port.inputStream().read(new byte[4]));
		assertThrows(PortClosedException.class, () -> port.outputStream().flush());
		assertThrows(PortClosedException.class, () -> port.outputStream().write(1));
		assertThrows(PortClosedException.class, () -> port.outputStream().write(new byte[4]));
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
		final var actual = port.parity();
		if ((parity == Parity.Mark || parity == Parity.Space) && actual == Parity.None)
			throw new TestAbortedException("hardware with no " + parity + " parity support (CMSPAR)");
		assertEquals(parity, actual);
	}

	@ParameterizedTest
	@EnumSource(value = FlowControl.class)
	void flowControl(final FlowControl flowControl) throws IOException {
		port.flowControl(flowControl);
		final var actual = port.flowControl();
		if (flowControl == FlowControl.CtsRts && actual == FlowControl.None)
			throw new TestAbortedException("no RTS/CTS flow control support");
		assertEquals(flowControl, actual);
	}

	@ParameterizedTest
	@MethodSource("timeoutsSource")
	void timeouts(final Timeouts timeouts) throws IOException {
		port.timeouts(timeouts);
		assertEquals(timeouts, ((ReadWritePort) port).timeouts());
	}

	private static Stream<Timeouts> timeoutsSource() {
		return Stream.of(
				new Timeouts(Duration.ZERO, Duration.ZERO, Duration.ZERO, Duration.ZERO, Duration.ZERO),
				new Timeouts(Duration.ofMillis(5), Duration.ZERO, Duration.ZERO, Duration.ZERO, Duration.ZERO),
				new Timeouts(Duration.ZERO, Duration.ofMillis(200), Duration.ofMillis(30), Duration.ZERO, Duration.ZERO),
				new Timeouts(Duration.ZERO, Duration.ZERO, Duration.ZERO, Duration.ofMillis(50), Duration.ofMillis(5)),
				Timeouts.readInterval(Duration.ofSeconds(1)),
				Timeouts.readTotal(Duration.ofSeconds(2), Duration.ofMillis(100))
		);
	}

	private static final Duration delta = Duration.ofMillis(50);

	@Test
	void readIntervalTimeout() throws IOException {
		final var timeouts = Timeouts.readInterval(Duration.ofMillis(2000));
		port.timeouts(timeouts);
		final var rwport = (ReadWritePort) port;
		assertTimeoutPreemptively(timeouts.readInterval().plus(delta), rwport::read);
	}

	@Test
	void readTotalTimeout() throws IOException {
		final var timeouts = Timeouts.readTotal(Duration.ofMillis(1000), Duration.ofMillis(300));
		port.timeouts(timeouts);

		final var rwport = (ReadWritePort) port;
		final int bytes = 1;
		final var total = timeouts.readTotalConstant().plus(timeouts.readTotalMultiplier().multipliedBy(bytes));
		assertTimeoutPreemptively(total.plus(delta), rwport::read);
	}

	@Test
	void readIntervalAndTotalTimeout() throws IOException {
		final var timeouts = new Timeouts(Duration.ofMillis(200), Duration.ofMillis(70), Duration.ofMillis(20),
				Duration.ZERO, Duration.ZERO);
		port.timeouts(timeouts);

		final var data = new byte[10];
		final var interval = timeouts.readInterval();
		final var total = timeouts.readTotalConstant().plus(timeouts.readTotalMultiplier().multipliedBy(data.length));
		final var min = interval.compareTo(total) <= 0 ? interval : total;
		final var rwport = (ReadWritePort) port;
		assertTimeoutPreemptively(min.plus(delta), () -> rwport.readBytes(data, 0, data.length));
	}

	@Test
	void interruptible() throws IOException {
		final var readInterval = Timeouts.readInterval(Duration.ofMillis(5000));
		port.timeouts(readInterval);

		final var readerThread = Thread.currentThread();
		final var timeToInterrupt = Duration.ofMillis(700);
		Executors.newSingleThreadScheduledExecutor().schedule(readerThread::interrupt,
				timeToInterrupt.toMillis(), TimeUnit.MILLISECONDS);

		final var rwport = (ReadWritePort) port;
		assertThrowsWithTimeout(IOException.class, rwport::read, "interrupted", timeToInterrupt);
	}

	private static <T extends Throwable> void assertThrowsWithTimeout(final Class<T> expectedType,
			final Executable run, String msg, final Duration timeout) {
		final long start = System.nanoTime();
		assertThrows(expectedType, run, msg);
		assertWithinRange(timeout, System.nanoTime() - start);
	}

	private static void assertWithinRange(final Duration expected, final long actualNanos) {
		if (expected.minusNanos(actualNanos).abs().compareTo(delta) > 0)
			assertionFailure()
					.expected(expected + " ±" + delta)
					.actual(actualNanos)
					.trimStacktrace(Assertions.class)
					.buildAndThrow();
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
	void events() throws IOException, InterruptedException {
		port.events(EnumSet.noneOf(SerialPort.SerialEvent.class), true);
		port.events(EnumSet.allOf(SerialPort.SerialEvent.class), true);

		port.events(EnumSet.noneOf(SerialPort.SerialEvent.class), false);
		port.events(EnumSet.allOf(SerialPort.SerialEvent.class), false);
	}

	@Test
	void eventLooperEnabled() throws IOException, InterruptedException {
		port.events(EnumSet.of(SerialPort.SerialEvent.DataAvailable), true);
		final var rwport = (ReadWritePort) port;
		assertNotNull(rwport.eventLooper);
		Thread.sleep(Duration.ofMillis(50));
		assertTrue(rwport.eventLooper.isAlive());
	}

	@Test
	void eventLooperDisabled() throws IOException, InterruptedException {
		final var rwport = (ReadWritePort) port;
		assertNull(rwport.eventLooper);
		port.events(EnumSet.of(SerialPort.SerialEvent.DataAvailable), true);
		Thread.sleep(Duration.ofMillis(100));
		final var thread = rwport.eventLooper;
		port.events(EnumSet.allOf(SerialPort.SerialEvent.class), false);
		assertNull(rwport.eventLooper);
		Thread.sleep(Duration.ofMillis(50));
		assertFalse(thread.isAlive());
	}

	@Test
	void waitEvent() throws IOException {
		final int eventMask = UnixSerialPort.EVENT_CTS | UnixSerialPort.EVENT_TXEMPTY | UnixSerialPort.EVENT_RING
				| UnixSerialPort.EVENT_BREAK | UnixSerialPort.EVENT_CTS | UnixSerialPort.EVENT_DSR
				| UnixSerialPort.EVENT_RLSD | UnixSerialPort.EVENT_RXCHAR;
		port.setEvents(eventMask, true);
		// TODO commented out because waiting for events never returns
//		final int events = port.waitEvent();
//		((UnixSerialPort) port).dispatchEvents(events);
	}

	@Test
	void status() throws IOException {
		assertEquals(0, port.status(Status.AvailableInput));
		assertEquals(0, port.status(Status.Error));
		// Line status will have DTR/RTS set after port open, so just verify no exception
		port.status(Status.Line);
	}

	@Test
	void read() throws IOException {
		try {
			final var timeouts = new Timeouts(Duration.ofMillis(5), Duration.ofSeconds(1), Duration.ZERO,
					Duration.ZERO, Duration.ZERO);
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
	void flush() throws IOException {
		port.outputStream().flush();

		port.outputStream().write(new byte[]{ 1, 2, 3 });
		port.outputStream().flush();
	}

	@Test
	void chain() throws IOException {
		try (var _ = port
				.baudRate(19_200)
				.dataBits(8)
				.parity(Parity.Even)
				.stopBits(StopBits.One)
				.flowControl(FlowControl.None)) {
			/*final var in =*/ port.inputStream();
			/*final var out =*/ port.outputStream();
		}
	}

	@ParameterizedTest
	@ValueSource(strings = { "0", "10", "2147483647", "2147483648", "4294967295" })
	@RestoreSystemProperties
	void validWakeupInterval(final String value) {
		System.setProperty(ReadWritePort.wakeupIntervalKey, value);
		int timeout = ReadWritePort.wakeupInterval();
		assertEquals(value, Long.toUnsignedString(timeout & 0xffffffffL));
	}

	@ParameterizedTest
	@ValueSource(strings = { "-1", "0xff", "x", "4294967296" })
	@RestoreSystemProperties
	void invalidWakeupInterval(final String value) {
		System.setProperty(ReadWritePort.wakeupIntervalKey, value);
		int timeout = ReadWritePort.wakeupInterval();
		assertEquals(ReadWritePort.defaultWakeupInterval, timeout);
	}
}
