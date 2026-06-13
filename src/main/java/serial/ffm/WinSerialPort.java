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

import static java.lang.System.Logger.Level.ERROR;
import static java.lang.System.Logger.Level.TRACE;
import static java.lang.System.Logger.Level.WARNING;
import static java.util.Map.entry;

import java.io.IOException;
import java.lang.System.Logger.Level;
import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.EnumSet;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import serial.ffm.win.HKEY__;
import serial.ffm.win.Windows;
import serial.ffm.win._COMMPROP;
import serial.ffm.win._COMMTIMEOUTS;
import serial.ffm.win._COMSTAT;
import serial.ffm.win._DCB;
import serial.ffm.win._OVERLAPPED;

/**
 * Serial communication using Panama FFM to invoke the Windows Comm API. The implementation of this API
 * contains platform dependent code.
 */
final class WinSerialPort extends ReadWritePort {

	// error flags
	// #define CE_RXOVER 0x0001 // Receive Queue overflow
	// #define CE_OVERRUN 0x0002 // Receive Overrun Error
	// #define CE_RXPARITY 0x0004 // Receive Parity Error
	// #define CE_FRAME 0x0008 // Receive Framing error
	// #define CE_BREAK 0x0010 // Break Detected
	// #define CE_TXFULL 0x0100 // TX Queue is full
	// #define CE_MODE 0x8000 // Requested mode unsupported

	static {
		System.loadLibrary("Kernel32");
	}


	private record HANDLE(MemorySegment handle) {
		static final HANDLE Invalid = new HANDLE(Windows.INVALID_HANDLE_VALUE());

		static HANDLE of(final MemorySegment handle) { return new HANDLE(handle); }

		boolean invalid() { return handle().equals(Invalid.handle()); }

		boolean valid() { return !invalid(); }

		@Override
		public String toString() { return "0x" + Long.toUnsignedString(handle.address(), 16); }
	}

	// only call inside lock
	private volatile HANDLE h = HANDLE.Invalid;
	private int maxBaudRate;


	// port names currently listed in registry
	static Set<String> portNames() {
		final var logger = System.getLogger("serial.ffm");
		logger.log(TRACE, "query serial port names from registry");
		final var portNames = new TreeSet<String>();

		try (var arena = Arena.ofConfined()) {
			final var subKey = arena.allocateFrom("HARDWARE\\DEVICEMAP\\SERIALCOMM");
			final var serialCommKey = HKEY__.allocate(arena);

			int ret = Windows.RegOpenKeyExA(Windows.HKEY_LOCAL_MACHINE(), subKey, 0, Windows.KEY_READ(), serialCommKey);
			if (ret == Windows.ERROR_SUCCESS()) {
				final int size = 256;
				final var deviceName = arena.allocate(size);
				final var deviceNameLength = arena.allocate(ValueLayout.JAVA_INT, size);
				final var portName = arena.allocate(size);
				final var portNameLength = arena.allocate(ValueLayout.JAVA_INT, size);

				for (int i = 0;; i++) {
					// reinit chars with size on every iteration
					deviceNameLength.set(ValueLayout.JAVA_INT, 0, size);
					portNameLength.set(ValueLayout.JAVA_INT, 0, size);
					ret = Windows.RegEnumValueA(MemorySegment.ofAddress(HKEY__.unused(serialCommKey)), i, deviceName,
							deviceNameLength, Windows.NULL(), Windows.NULL(), portName, portNameLength);

					if (ret == Windows.ERROR_NO_MORE_ITEMS())
						break;

					if (ret == Windows.ERROR_SUCCESS()) {
						final String port = extractString(portName, portNameLength);
						logger.log(TRACE, "{0} = {1}", extractString(deviceName, deviceNameLength), port);
						portNames.add(port);
					}
					else {
						logger.log(WARNING, "RegEnumValueA error: {0}", formatWinError(ret));
					}
				}
				Windows.RegCloseKey(serialCommKey);
			}
			else {
				logger.log(TRACE, "RegOpenKeyExA error: {0}", formatWinError(ret));
			}
		}
		return portNames;
	}

	// don't assume the string in the registry is null-terminated
	private static String extractString(final MemorySegment string, final MemorySegment length) {
		int actualLength = length.get(ValueLayout.JAVA_INT, 0);
		final int terminator = string.get(ValueLayout.JAVA_BYTE, actualLength) == 0 ? 1 : 0;
		actualLength -= terminator;
		final var bytes = string.asSlice(0, actualLength).toArray(ValueLayout.JAVA_BYTE);
		return new String(bytes, StandardCharsets.UTF_8);
	}

	// TODO only for port exists check
	WinSerialPort() {
		super("");
	}

	WinSerialPort(final String portId) throws IOException {
		super(portId);
		open(portId);
	}

	@Override
	void open(final Arena arena, final String portId) throws IOException {
		final var lastError = arena.allocate(Win.captureStateLayout);
		final String name = portId.toUpperCase().startsWith("COM") ? "\\\\.\\" + portId : portId;
		final HANDLE h = open(arena, name, true, lastError);
		if (h.invalid())
			throwIoException(name, lastError);

		this.h = h;
		try {
			final int fileType = Win.GetFileType(lastError, h.handle());
			if (fileType != Windows.FILE_TYPE_CHAR() && fileType != Windows.FILE_TYPE_UNKNOWN())
				throwIoException(name, lastError);

			final var cp = _COMMPROP.allocate(arena);
			if (Win.GetCommProperties(lastError, h.handle(), cp) == 0)
				throwIoException(name, lastError);
			if (debug()) {
				// set rx and tx queues (rx will usually keep some default buffer, 4K or similar...)
				Win.SetupComm(lastError, h.handle(), 200, 200);
				if (Win.GetCommProperties(lastError, h.handle(), cp) == 0)
					throwIoException(name, lastError);
				final int tx = _COMMPROP.dwCurrentTxQueue(cp);
				final int rx = _COMMPROP.dwCurrentRxQueue(cp);
				logger.log(TRACE, "set queue tx {0} rx {1}", tx, rx);
			}
			if (Win.EscapeCommFunction(lastError, h.handle(), Windows.SETDTR()) == 0)
				throwIoException(lastError);

			maxBaudRate = _COMMPROP.dwMaxBaud(cp);
			logger.log(TRACE, "max baud rate = " + maxBaudRate);
		} catch (final IOException e) {
			close();
			throw e;
		}
	}

	@Override
	void baudRate(final Arena arena, final int baudrate) throws IOException {
		if (maxBaudRate > 0 && baudrate > maxBaudRate)
			throw new IOException("baud rate %d > max. allowed baud rate %d".formatted(baudrate, maxBaudRate));

		final var lastError = arena.allocate(Win.captureStateLayout);
		final var dcb = commState(arena);
		_DCB.BaudRate(dcb, baudrate);
		setDcbBits(dcb, "fAbortOnError", Windows.FALSE());
		if (Win.SetCommState(lastError, h.handle(), dcb) == 0)
			throwIoException(lastError);
	}

	@Override
	int baudRate(final Arena arena) throws IOException {
		final var dcb = commState(arena);
		return _DCB.BaudRate(dcb);
	}

	@Override
	void parity(final Arena arena, final Parity parity) throws IOException {
		final var dcb = commState(arena);
		_DCB.Parity(dcb, (byte) parity.value());
		setDcbBits(dcb, "fParity", parity.value() != Windows.NOPARITY() ? 1 : 0);
		setDcbBits(dcb, "fAbortOnError", Windows.FALSE());

		final char parityReplace = '\0';
		if (parity == Parity.None) {
			setDcbBits(dcb, "fErrorChar", 0);
			_DCB.ErrorChar(dcb, (byte) parityReplace);
		}
		else {
			setDcbBits(dcb, "fErrorChar", parityReplace != '\0' ? 1 : 0);
			_DCB.ErrorChar(dcb, (byte) parityReplace);
		}

		final var lastError = arena.allocate(Win.captureStateLayout);
		if (Win.SetCommState(lastError, h.handle(), dcb) == 0)
			throwIoException(lastError);
	}

	@Override
	Parity parity(final Arena arena) throws IOException {
		final var dcb = commState(arena);
		return Parity.values()[_DCB.Parity(dcb)];
	}

	@Override
	void dataBits(final Arena arena, final int databits) throws IOException {
		final var dcb = commState(arena);
		_DCB.ByteSize(dcb, (byte) databits);
		setDcbBits(dcb, "fAbortOnError", Windows.FALSE());
		final var lastError = arena.allocate(Win.captureStateLayout);
		if (Win.SetCommState(lastError, h.handle(), dcb) == 0)
			throwIoException(lastError);
	}

	@Override
	int dataBits(final Arena arena) throws IOException {
		final var dcb = commState(arena);
		return _DCB.ByteSize(dcb);
	}

	@Override
	void stopBits(final Arena arena, final StopBits stopbits) throws IOException {
		final var dcb = commState(arena);
		_DCB.StopBits(dcb, (byte) platformStopBits(stopbits));
		setDcbBits(dcb, "fAbortOnError", Windows.FALSE());
		final var lastError = arena.allocate(Win.captureStateLayout);
		if (Win.SetCommState(lastError, h.handle(), dcb) == 0)
			throwIoException(lastError);
	}

	@Override
	StopBits stopBits(final Arena arena) throws IOException {
		final var dcb = commState(arena);
		return genericStopBits(_DCB.StopBits(dcb));
	}

	@Override
	void flowControl(final Arena arena, final FlowControl flowControl) throws IOException {
		final var lastError = arena.allocate(Win.captureStateLayout);
		final var dcb = commState(arena);
		setDcbBits(dcb, "fOutxDsrFlow", Windows.FALSE());
		setDcbBits(dcb, "fDtrControl", Windows.DTR_CONTROL_DISABLE());
		setDcbBits(dcb, "fDsrSensitivity", Windows.FALSE());
		setDcbBits(dcb, "fTXContinueOnXoff", Windows.FALSE());
		setDcbBits(dcb, "fOutX", Windows.FALSE());
		setDcbBits(dcb, "fInX", Windows.FALSE());
		_DCB.XonChar(dcb, (byte) 0x11);
		_DCB.XoffChar(dcb, (byte) 0x13);
		switch (flowControl) {
			case CtsRts -> {
				setDcbBits(dcb, "fOutxCtsFlow", Windows.TRUE());
				setDcbBits(dcb, "fRtsControl", Windows.RTS_CONTROL_HANDSHAKE());
			}
			case None -> {
				setDcbBits(dcb, "fOutxCtsFlow", Windows.FALSE());
				setDcbBits(dcb, "fRtsControl", Windows.RTS_CONTROL_DISABLE());
			}
		}

		setDcbBits(dcb, "fAbortOnError", Windows.FALSE());
		if (Win.SetCommState(lastError, h.handle(), dcb) == 0)
			throwIoException(lastError);
	}

	@Override
	FlowControl flowControl(final Arena arena) throws IOException {
		final var dcb = commState(arena);
		final int flag = getDcbBits(dcb, "fOutxCtsFlow");
		return flag == 1 ? FlowControl.CtsRts : FlowControl.None;
	}

	private MemorySegment commState(final Arena arena) throws IOException {
		final var lastError = arena.allocate(Win.captureStateLayout);
		final var dcb = _DCB.allocate(arena);
		_DCB.DCBlength(dcb, (int) _DCB.sizeof());
		if (Win.GetCommState(lastError, h.handle(), dcb) == 0)
			throwIoException(lastError);
		return dcb;
	}

	@Override
	void close(final Arena arena) {
		logger.log(TRACE, "close handle {0}", h);
		final var handle = h.handle();
		h = HANDLE.Invalid;
		final var lastError = arena.allocate(Win.captureStateLayout);
		final boolean closed = Win.CloseHandle(lastError, handle) == Windows.TRUE();
		if (!closed)
			logger.log(ERROR, "closing serial port: {0}", formatWinError(Win.getLastError(lastError)));
	}

	@Override
	boolean isClosed() {
		return h.invalid();
	}

	private HANDLE open(final Arena arena, final String portId, final boolean overlapped, final MemorySegment lastError) {
		logger.log(TRACE, "open {0}", portId);
		final var h = Win.CreateFileA(lastError, arena.allocateFrom(portId),
				Windows.GENERIC_READ() | Windows.GENERIC_WRITE(), 0, Windows.NULL(), Windows.OPEN_EXISTING(),
				Windows.FILE_ATTRIBUTE_NORMAL() | (overlapped ? Windows.FILE_FLAG_OVERLAPPED() : 0),
				Windows.NULL());
		return HANDLE.of(h);
	}

	boolean portExists(final String portId) {
		try (var arena = Arena.ofConfined()) {
			final var lastError = arena.allocate(Win.captureStateLayout);
			final HANDLE h = open(arena, portId, false, lastError);
			// if handle is valid, port exists, otherwise it depends on last error: on existing, but
			// used port, we would get ERROR_ACCESS_DENIED or ERROR_SHARING_VIOLATION (or similar)
			if (h.valid())
				Win.CloseHandle(lastError, h.handle());
			else {
				final int error = Win.getLastError(lastError);
				if (error == Windows.ERROR_FILE_NOT_FOUND() || error == Windows.ERROR_PATH_NOT_FOUND())
					return false;
				logger.log(Level.DEBUG, "opening {0}: {1}", portId, formatWinError(error));
			}
		}
		return true;
	}

	private static int platformStopBits(final StopBits stopbits) {
		return switch (stopbits) {
			case One -> Windows.ONESTOPBIT();
			case Two -> Windows.TWOSTOPBITS();
		};
	}

	private static StopBits genericStopBits(final int stopbits) {
		if (stopbits == Windows.ONESTOPBIT())
			return StopBits.One;
		if (stopbits == Windows.TWOSTOPBITS())
			return StopBits.Two;
		throw new RuntimeException("unsupported stopbits " + stopbits);
	}

	private static final Map<String, Integer> dcbFlagOffset = Map.ofEntries(
			entry("fBinary", 0),
			entry("fParity", 1),
			entry("fOutxCtsFlow", 2),
			entry("fOutxDsrFlow", 3),
			entry("fDtrControl", 4),
			entry("fDsrSensitivity", 6),
			entry("fTXContinueOnXoff", 7),
			entry("fOutX", 8),
			entry("fInX", 9),
			entry("fErrorChar", 10),
			entry("fNull", 11),
			entry("fRtsControl", 12),
			entry("fAbortOnError", 14));

	private void setDcbBits(final MemorySegment dcb, final String flag, final int value) {
		final var flagOffset = dcbFlagOffset.get(flag);
		logger.log(TRACE, "set " + flag + " = " + value + ", offset = " + flagOffset);

		final int mask = switch (flag) {
			case "fDtrControl", "fRtsControl" -> 3;
			default -> 1;
		};
		final int bits = mask << flagOffset;

		final long field = dcb.get(ValueLayout.JAVA_INT, 8) & 0xffff_ffffL;
		final long set = value == 0 ? field & ~bits : field | (value << flagOffset);

		if (debug()) {
			logger.log(TRACE, "DCB flags =   b" + leftPad(Long.toBinaryString(field), '0', 32));
			logger.log(TRACE, "DCB set bit = b" + leftPad(Long.toBinaryString(set), '0', 32));
			final var sb = new StringBuilder("|       |       |       |        |");
			sb.setCharAt(32 - flagOffset, 'x');
			final String indicator = "              " + sb;
			logger.log(TRACE, indicator);
		}

		dcb.set(ValueLayout.JAVA_INT, 8, (int) set);
	}

	private int getDcbBits(final MemorySegment dcb, final String flag) {
		final var flagOffset = dcbFlagOffset.get(flag);
		logger.log(TRACE, "get " + flag + ", offset = " + flagOffset);

		final int mask = switch (flag) {
			case "fDtrControl", "fRtsControl" -> 3;
			default -> 1;
		};

		final long field = dcb.get(ValueLayout.JAVA_INT, 8) & 0xffff_ffffL;

		if (debug()) {
			logger.log(TRACE, "DCB flags =   b" + leftPad(Long.toBinaryString(field), '0', 32));
			final var sb = new StringBuilder("|       |       |       |        |");
			sb.setCharAt(32 - flagOffset, 'x');
			final String indicator = "              " + sb;
			logger.log(TRACE, indicator);
		}

		final long l = dcb.get(ValueLayout.JAVA_INT, 8) & 0xffff_ffffL;
		return (int) ((l >> flagOffset) & mask);
	}

	private static String leftPad(final String s, final char ch, final int pad) {
		return ("%" + pad + "s").formatted(s).replace(' ', ch);
	}

	//call *immediately* after read/write
	private void waitPendingIO(final MemorySegment lastError, /*OVERLAPPED* */ final MemorySegment overlapped,
			final MemorySegment transferred) throws IOException, InterruptedException {
//		logger.log(TRACE, "wait pending I/O");
		// the only error status tolerated is I/O pending
		final int error = Win.getLastError(lastError);
		if (error != Windows.NO_ERROR() && error != Windows.ERROR_IO_PENDING())
			// some I/O problem, throw error
			throwIoException(lastError);

		// wait for operation completion, and check result
		while (true) {
			if (Thread.interrupted())
				throw new InterruptedException();

			if (Win.GetOverlappedResultEx(lastError, h.handle(), overlapped, transferred, wakeupInterval, Windows.FALSE()) != 0)
				return; // completed successfully
			final int res = Win.getLastError(lastError);
			if (res == Windows.WAIT_TIMEOUT() || res == Windows.ERROR_IO_INCOMPLETE())
				continue;
			// res == WAIT_FAILED: communication or wait error
			throwIoException(lastError);
		}
	}

	@Override
	int writeBytes(final Arena arena, final MemorySegment bytes) throws IOException {
		final var lastError = arena.allocate(Win.captureStateLayout);
		final var event = Win.CreateEventA(lastError, Windows.NULL(), Windows.TRUE(), Windows.FALSE(), Windows.NULL());
		if (event.equals(Windows.NULL()))
			throwIoException(lastError);

		try {
			final var o = _OVERLAPPED.allocate(arena);
			_OVERLAPPED.hEvent(o, event);

			if (Win.WriteFile(lastError, h.handle(), bytes, (int) bytes.byteSize(), Windows.NULL(), o) != 0)
				return (int) bytes.byteSize();
			final var written = arena.allocateFrom(ValueLayout.JAVA_INT, 0);
			waitPendingIO(lastError, o, written);
			return written.get(ValueLayout.JAVA_INT, 0);
		}
		catch (final InterruptedException e) {
			Thread.currentThread().interrupt();
			throw new IOException("interrupted");
		}
		finally {
			Win.CloseHandle(lastError, event);
		}
	}

	@Override
	int readBytes(final Arena arena, final MemorySegment bytes) throws IOException {
		final var lastError = arena.allocate(Win.captureStateLayout);
		final var event = Win.CreateEventA(lastError, Windows.NULL(), Windows.TRUE(), Windows.FALSE(), Windows.NULL());
		if (event.equals(Windows.NULL()))
			throwIoException(lastError);

		final var o = _OVERLAPPED.allocate(arena);
		_OVERLAPPED.hEvent(o, event);

		try {
			if (Win.ReadFile(lastError, h.handle(), bytes, (int) bytes.byteSize(), Windows.NULL(), o) != 0)
				return (int) bytes.byteSize();
			final var read = arena.allocateFrom(ValueLayout.JAVA_INT, 0);
			waitPendingIO(lastError, o, read);
			return read.get(ValueLayout.JAVA_INT, 0);
		}
		catch (final InterruptedException e) {
			Thread.currentThread().interrupt();
			throw new IOException("interrupted");
		}
		finally {
			Win.CloseHandle(lastError, event);
		}
	}

	@Override
	void doDrain() throws IOException {
		try (var arena = Arena.ofConfined()) {
			final var lastError = arena.allocate(Win.captureStateLayout);
			if (Win.FlushFileBuffers(lastError, h.handle()) == 0)
				throwIoException(lastError);
		}
	}

	@Override
	public void setEvents(final int eventMask, final boolean enable) throws IOException {
		logger.log(TRACE, "set event: mask 0x{0}, enable={1}",  Integer.toUnsignedString(eventMask, 16), enable);
		try (var arena = Arena.ofConfined()) {
			final var lastError = arena.allocate(Win.captureStateLayout);
			/*DWORD*/ final var mask = arena.allocate(ValueLayout.JAVA_INT);
			if (Win.GetCommMask(lastError, h.handle(), mask) == 0)
				throwIoException(lastError);

			final int m = mask.get(ValueLayout.JAVA_INT, 0);
			final int set = enable ? m | eventMask : m & ~eventMask;
			if (Win.SetCommMask(lastError, h.handle(), set) == 0)
				throwIoException(lastError);
			enableEventLooper(set != 0);
		}
	}

	@Override
	public void events(final EnumSet<SerialEvent> events, final boolean enable) throws IOException {
		if (isClosed())
			throwIoException(Windows.ERROR_INVALID_HANDLE());
		logger.log(TRACE, "{0} events: {1}", enable ? "enable" : "disable", events);
		int mask = 0;
		for (final var event : events) {
			mask |= switch (event) {
				case DataAvailable -> Windows.EV_RXCHAR();
				case OutputEmpty -> Windows.EV_TXEMPTY();

				case ClearToSend -> Windows.EV_CTS();
				case DataSetReady -> Windows.EV_DSR();
				case CarrierDetect -> Windows.EV_RLSD();

				case Break -> Windows.EV_BREAK();
				case Error -> Windows.EV_ERR();
				case Ring -> Windows.EV_RING();
			};
		}
		setEvents(mask, enable);
	}

	@Override
	public EnumSet<SerialEvent> waitEvent() throws IOException, InterruptedException {
		logger.log(TRACE, "wait comm event");
		try (var arena = Arena.ofConfined()) {
			final var lastError = arena.allocate(Win.captureStateLayout);
			final var event = Win.CreateEventA(lastError, Windows.NULL(), Windows.TRUE(), Windows.FALSE(), Windows.NULL());
			if (event.equals(Windows.NULL()))
				throwIoException(lastError);

			final /*DWORD*/ MemorySegment eventMask;
			try {
				final var o = _OVERLAPPED.allocate(arena);
				_OVERLAPPED.hEvent(o, event);
				eventMask = arena.allocateFrom(ValueLayout.JAVA_INT, 0);
				if (Win.WaitCommEvent(lastError, h.handle(), eventMask, o) == 0) {
					final /*DWORD*/ var unused = arena.allocateFrom(ValueLayout.JAVA_INT, 0);
					waitPendingIO(lastError, o, unused);
				}
			}
			finally {
				Win.CloseHandle(lastError, event);
			}
			// note if event mask was changed while waiting for event we return 0
			return translateEvents(eventMask.get(ValueLayout.JAVA_INT, 0));
		}
	}

	@Override
	int status(final Arena arena, final Status type) throws IOException {
		final var lastError = arena.allocate(Win.captureStateLayout);
		/*DWORD*/ final var value = arena.allocate(ValueLayout.JAVA_INT);
		int ret;
		final int status = switch (type) {
			case Line -> {
				// line status bit field
				// 0x0010 : CTS (clear-to-send) signal is on
				// 0x0020 : DSR (data-set-ready) signal is on
				// 0x0040 : ring indicator signal is on
				// 0x0080 : RLSD (receive-line-signal-detect) signal is on
				ret = Win.GetCommModemStatus(lastError, h.handle(), value);
				yield value.get(ValueLayout.JAVA_INT, 0);
			}
			case AvailableInput -> {
				final var stat = _COMSTAT.allocate(arena);
				ret = Win.ClearCommError(lastError, h.handle(), value, stat);
				yield _COMSTAT.cbInQue(stat);
			}
			case Error -> {
				ret = Win.ClearCommError(lastError, h.handle(), value, Windows.NULL());
				final int errors = value.get(ValueLayout.JAVA_INT, 0);
				commErrors(errors);
				yield errors;
			}
		};
		if (ret == 0)
			throwIoException(lastError);
		return status;
	}

	@Override
	void timeouts(final Arena arena, final Timeouts timeouts) throws IOException {
		final var lastError = arena.allocate(Win.captureStateLayout);
		final var to = _COMMTIMEOUTS.allocate(arena);
		_COMMTIMEOUTS.ReadIntervalTimeout(to, (int) timeouts.readInterval().toMillis());
		_COMMTIMEOUTS.ReadTotalTimeoutMultiplier(to, (int) timeouts.readTotalMultiplier().toMillis());
		_COMMTIMEOUTS.ReadTotalTimeoutConstant(to, (int) timeouts.readTotalConstant().toMillis());
		_COMMTIMEOUTS.WriteTotalTimeoutMultiplier(to, (int) timeouts.writeTotalMultiplier().toMillis());
		_COMMTIMEOUTS.WriteTotalTimeoutConstant(to, (int) timeouts.writeTotalConstant().toMillis());
		if (Win.SetCommTimeouts(lastError, h.handle(), to) == 0)
			throwIoException(lastError);
	}

	@Override
	Timeouts timeouts(final Arena arena) throws IOException {
		final var lastError = arena.allocate(Win.captureStateLayout);
		final var to = _COMMTIMEOUTS.allocate(arena);
		if (Win.GetCommTimeouts(lastError, h.handle(), to) == 0)
			throwIoException(lastError);
		return new Timeouts(Duration.ofMillis(_COMMTIMEOUTS.ReadIntervalTimeout(to)),
				Duration.ofMillis(_COMMTIMEOUTS.ReadTotalTimeoutConstant(to)),
				Duration.ofMillis(_COMMTIMEOUTS.ReadTotalTimeoutMultiplier(to)),
				Duration.ofMillis(_COMMTIMEOUTS.WriteTotalTimeoutConstant(to)),
				Duration.ofMillis(_COMMTIMEOUTS.WriteTotalTimeoutMultiplier(to)));
	}

	private EnumSet<SerialEvent> translateEvents(final int eventMask) {
		final var events = EnumSet.noneOf(SerialEvent.class);
		// data events
		if (isSet(eventMask, Windows.EV_RXCHAR()))
			events.add(SerialEvent.DataAvailable);

		// pin events
		if (isSet(eventMask, Windows.EV_CTS()))
			events.add(SerialEvent.ClearToSend);
		if (isSet(eventMask, Windows.EV_DSR()))
			events.add(SerialEvent.DataSetReady);
		if (isSet(eventMask, Windows.EV_RLSD()))
			events.add(SerialEvent.CarrierDetect);
		if (isSet(eventMask, Windows.EV_BREAK()))
			events.add(SerialEvent.Break);
		if (isSet(eventMask, Windows.EV_RING()))
			events.add(SerialEvent.Ring);

		// error event
		if (isSet(eventMask, Windows.EV_ERR()))
			events.add(SerialEvent.Error);
		return events;
	}

	private void commErrors(final int errors) {
		if (isSet(errors, Windows.CE_RXOVER()))
			logger.log(TRACE, "CE_RXOVER");
		if (isSet(errors, Windows.CE_OVERRUN()))
			logger.log(TRACE, "CE_OVERRUN");
		if (isSet(errors, Windows.CE_RXPARITY()))
			logger.log(TRACE, "CE_RXPARITY");
		if (isSet(errors, Windows.CE_FRAME()))
			logger.log(TRACE, "CE_FRAME");
		if (isSet(errors, Windows.CE_BREAK()))
			logger.log(TRACE, "CE_BREAK");
	}

	private static String formatWinError(/*DWORD*/ final int error) {
		final int size = 256;

		try (var arena = Arena.ofConfined()) {
			final var lastError = arena.allocate(Win.captureStateLayout);
			final var buf = arena.allocate(ValueLayout.JAVA_BYTE, size);
			int len = Win.FormatMessageA(lastError,
					Windows.FORMAT_MESSAGE_FROM_SYSTEM() | Windows.FORMAT_MESSAGE_IGNORE_INSERTS()
					| Windows.FORMAT_MESSAGE_MAX_WIDTH_MASK(),
					Windows.NULL(), error, 0, buf, size, Windows.NULL());
			while (len > 3 && (buf.get(ValueLayout.JAVA_BYTE, len - 1) == ' '
					|| buf.get(ValueLayout.JAVA_BYTE, len - 1) == '.'))
				--len;
			buf.set(ValueLayout.JAVA_BYTE, len, (byte) 0);
			return buf.getString(0) + " (error 0x" + Integer.toUnsignedString(error, 16) + ")";
		}
	}

	private void throwIoException(final MemorySegment lastError) throws IOException {
		throwIoException(Win.getLastError(lastError));
	}

	private void throwIoException(final String s, final MemorySegment lastError) throws IOException {
		final int error = Win.getLastError(lastError);
		if (error == Windows.ERROR_INVALID_HANDLE())
			throw s != null ? new PortClosedException(portName(), s) : new PortClosedException(portName());
		final String errmsg = formatWinError(error);
		throw new IOException(s != null ? s + ": " + errmsg : errmsg);
	}

	private void throwIoException(final int error) throws IOException {
		if (error == Windows.ERROR_INVALID_HANDLE())
			throw new PortClosedException(portName());
		throw new IOException(formatWinError(error));
	}
}
