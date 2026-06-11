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

import static java.lang.System.Logger.Level.DEBUG;
import static java.lang.System.Logger.Level.TRACE;
import static java.lang.System.Logger.Level.WARNING;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.System.Logger;
import java.lang.System.Logger.Level;
import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandles;
import java.util.HexFormat;
import java.util.concurrent.locks.ReentrantLock;

abstract sealed class ReadWritePort implements SerialPort permits UnixSerialPort, WinSerialPort {
	private static final boolean debug = System.getProperty("serial.ffm.debug") != null;
	static boolean debug() {
		return debug;
	}

	// special cases for wakeup interval:
	// 0: WaitForSingleObject (Windows) and poll (Unix) always return immediately
	// 4294967295, i.e., -1: infinite timeout; WaitForSingleObject and poll return only when the object/fd is signaled/ready
	static final int wakeupInterval = wakeupInterval();
	static final int defaultWakeupInterval = 20; // ms
	static final String wakeupIntervalKey = "serial.ffm.wakeupInterval";
	static int wakeupInterval() {
		final String val = System.getProperty(wakeupIntervalKey);
		if (val != null && !val.isEmpty()) {
			try {
				final int i = Integer.parseUnsignedInt(val);
				System.getLogger("serial.ffm").log(DEBUG, "set wakeup interval to {0} ms", i);
				return i;
			}
			catch (final NumberFormatException e) {
				System.getLogger("serial.ffm").log(WARNING, "error reading property ''{0}'' (default to {1} ms): {2}",
						wakeupIntervalKey, defaultWakeupInterval, e.getMessage());
			}
		}
		return defaultWakeupInterval;
	}

	final Logger logger;
	final ReentrantLock lock = new ReentrantLock();

	private final InputStream is;
	private final OutputStream os;
	private final String portId;

	Thread eventLooper;

	ReadWritePort(final String portId) {
		logger = System.getLogger(MethodHandles.lookup().lookupClass().getPackageName() + ":" + portId);
		this.portId = portId;
		is = new PortInputStream(this);
		os = new PortOutputStream(this);
	}

	@Override
	public final SerialPort baudRate(final int baudrate) throws IOException {
		logger.log(TRACE, "set baud rate {0}", baudrate);
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			baudRate(arena, baudrate);
		}
		finally {
			lock.unlock();
		}
		return this;
	}

	abstract void baudRate(Arena arena, int baudrate) throws IOException;

	@Override
	public final int baudRate() throws IOException {
		logger.log(TRACE, "get baud rate");
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			return baudRate(arena);
		}
		finally {
			lock.unlock();
		}
	}

	abstract int baudRate(Arena arena) throws IOException;

	@Override
	public final SerialPort parity(final Parity parity) throws IOException {
		logger.log(TRACE, "set parity {0}", parity);
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			parity(arena, parity);
		}
		finally {
			lock.unlock();
		}
		return this;
	}

	abstract void parity(Arena arena, Parity parity) throws IOException;

	@Override
	public final Parity parity() throws IOException {
		logger.log(TRACE, "get parity");
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			return parity(arena);
		}
		finally {
			lock.unlock();
		}
	}

	abstract Parity parity(Arena arena) throws IOException;

	@Override
	public final SerialPort dataBits(final int databits) throws IOException {
		logger.log(TRACE, "set databits {0}", databits);
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			dataBits(arena, databits);
		}
		finally {
			lock.unlock();
		}
		return this;
	}

	abstract void dataBits(Arena arena, int databits) throws IOException;

	@Override
	public final int dataBits() throws IOException {
		logger.log(TRACE, "get databits");
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			return dataBits(arena);
		}
		finally {
			lock.unlock();
		}
	}

	abstract int dataBits(Arena arena) throws IOException;

	@Override
	public final SerialPort stopBits(final StopBits stopbits) throws IOException {
		logger.log(TRACE, "set stop bits {0}", stopbits);
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			stopBits(arena, stopbits);
		}
		finally {
			lock.unlock();
		}
		return this;
	}

	abstract void stopBits(Arena arena, StopBits stopbits) throws IOException;

	@Override
	public final StopBits stopBits() throws IOException {
		logger.log(TRACE, "get stop bits");
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			return stopBits(arena);
		}
		finally {
			lock.unlock();
		}
	}

	abstract StopBits stopBits(Arena arena) throws IOException;

	@Override
	public final SerialPort flowControl(final FlowControl flowControl) throws IOException {
		logger.log(TRACE, "set flow control {0}", flowControl);
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			flowControl(arena, flowControl);
		}
		finally {
			lock.unlock();
		}
		return this;
	}

	abstract void flowControl(Arena arena, FlowControl flowControl) throws IOException;

	@Override
	public final FlowControl flowControl() throws IOException {
		logger.log(TRACE, "get flow control");
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			return flowControl(arena);
		}
		finally {
			lock.unlock();
		}
	}

	abstract FlowControl flowControl(Arena arena) throws IOException;

	@Override
	public final SerialPort timeouts(final Timeouts timeouts) throws IOException {
		logger.log(TRACE, "set {0}", timeouts);
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			timeouts(arena, timeouts);
		}
		finally {
			lock.unlock();
		}
		return this;
	}

	abstract void timeouts(Arena arena, Timeouts timeouts) throws IOException;

	final Timeouts timeouts() throws IOException {
		logger.log(TRACE, "get timeouts");
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			return timeouts(arena);
		}
		finally {
			lock.unlock();
		}
	}

	abstract Timeouts timeouts(Arena arena) throws IOException;

	@Override
	public final InputStream inputStream() {
		return is;
	}

	@Override
	public final OutputStream outputStream() {
		return os;
	}

	@Override
	public final int status(final Status type) throws IOException {
		logger.log(TRACE, "get status {0}", type);
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			return status(arena, type);
		}
		finally {
			lock.unlock();
		}
	}

	abstract int status(Arena arena, Status type) throws IOException;

	@Override
	public void close() {
		if (isClosed())
			return;
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			close(arena);
		}
		finally {
			eventLooper.interrupt();
			lock.unlock();
		}
	}

	abstract void close(Arena arena);

	final String portName() { return portId; }

	void open(final String portId) throws IOException {
		logger.log(TRACE, "open {0}", portId);
		lock.lock();
		try (var arena = Arena.ofConfined()) {
			open(arena, portId);
			eventLooper = Thread.startVirtualThread(this::waitEventLoop);
		}
		finally {
			lock.unlock();
		}
	}

	abstract void open(Arena arena, String portId) throws IOException;

	final int read() throws IOException {
		try (var arena = Arena.ofConfined()) {
			final var buf = arena.allocate(1);
			final long start = System.nanoTime();
			final long ret = readBytes(arena, buf);
			if (ret == 0)
				return -1;
			final byte b = buf.get(ValueLayout.JAVA_BYTE, 0);
			final long elapsed = System.nanoTime() - start;
			logger.log(TRACE, "read data [{0} us] (length 1): {1}", elapsed / 1000, HexFormat.of().toHexDigits(b));
			return b & 0xff;
		}
	}

	final int readBytes(final byte[] bytes, final int offset, final int length) throws IOException {
		try (var arena = Arena.ofConfined()) {
//			logger.log(TRACE, "start read");
			final long start = System.nanoTime();
			final var buf = arena.allocate(length);
			final int r = readBytes(arena, buf);
			MemorySegment.copy(buf, ValueLayout.JAVA_BYTE, 0, bytes, offset, r);

			if (r > 0 && logger.isLoggable(TRACE)) {
				final long elapsed = System.nanoTime() - start;
				final String hex = HexFormat.ofDelimiter(" ").formatHex(bytes, offset, offset + r);
				logger.log(TRACE, "read data [{0} us] (length {1}): {2}", elapsed / 1000, r, hex);
			}

			return r;
		}
	}

	abstract int readBytes(Arena arena, MemorySegment bytes) throws IOException;

	final void write(final int bite) throws IOException {
		var hex = bite < 16 ? "0" + Integer.toHexString(bite) : Integer.toHexString(bite);
		logger.log(TRACE, "start write (length 1): {0}", hex);
		try (var arena = Arena.ofConfined()) {
			final var out = arena.allocateFrom(ValueLayout.JAVA_BYTE, (byte) bite);
			final int written = writeBytes(arena, out);
			if (written != 1)
				throw new IOException("write failed");
		}
		finally {
			logger.log(TRACE, "end write");
		}
	}

	final void writeBytes(final byte[] bytes, final int offset, final int length) throws IOException {
		final var hex = HexFormat.ofDelimiter(" ").formatHex(bytes, offset, offset + length);
		logger.log(TRACE, "start write (length {0}): {1}", length, hex);
		try (var arena = Arena.ofConfined()) {
			final var buf = arena.allocate(length);
			MemorySegment.copy(bytes, offset, buf, ValueLayout.JAVA_BYTE, 0, length);
			writeBytes(arena, buf);
		}
		finally {
			logger.log(TRACE, "end write");
		}
	}

	abstract int writeBytes(Arena arena, MemorySegment bytes) throws IOException;

	final void drain() throws IOException {
		logger.log(TRACE, "drain");
		lock.lock();
		try {
			doDrain();
		}
		finally {
			lock.unlock();
		}
	}

	abstract void doDrain() throws IOException;

	abstract boolean isClosed();

	void waitEventLoop() {
		logger.log(TRACE, "enter waitEventLoop");
		while (!isClosed()) {
			try {
				final int eventMask = waitEvent();
				dispatchEvents(eventMask);
			}
			catch (final IOException e) {
				if (!isClosed())
					logger.log(Level.WARNING, "waitEventLoop", e);
			}
		}
		logger.log(TRACE, "exit waitEventLoop");
	}

	abstract void dispatchEvents(int eventMask);

	static boolean isSet(final int mask, final int flag) {
		return (mask & flag) == flag;
	}

	@Override
	public String toString() {
		if (isClosed())
			return portId + " closed";
		try {
			return portId + " baudrate=" + baudRate() + ", parity=" + parity() + ", databits=" + dataBits()
			+ ", stopbits=" + stopBits() + ", " + timeouts();
		}
		catch (final IOException e) {
			return portId + " invalid port setup: " + e.getMessage();
		}
	}
}
