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
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

import org.win.HKEY__;
import org.win.Windows;
import org.win.Windows_1;
import org.win.Windows_13;
import org.win.Windows_2;
import org.win.Windows_5;
import org.win.Windows_6;
import org.win.Windows_7;
import org.win.Windows_8;
import org.win.Windows_9;
import org.win._COMMPROP;
import org.win._COMMTIMEOUTS;
import org.win._COMSTAT;
import org.win._DCB;
import org.win._OVERLAPPED;

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
	static List<String> portNames() {
		final var logger = System.getLogger("serial.ffm");
		logger.log(TRACE, "query serial port names from registry");
		final var portNames = new ArrayList<String>();

		try (var arena = Arena.openConfined()) {
			final var subKey = arena.allocateUtf8String("HARDWARE\\DEVICEMAP\\SERIALCOMM");
			final var serialCommKey = HKEY__.allocate(arena);

			int ret = Windows_7.RegOpenKeyExA(Windows_13.HKEY_LOCAL_MACHINE(), subKey, 0, Windows_8.KEY_READ(), serialCommKey);
			if (ret == Windows_9.ERROR_SUCCESS()) {
				final int size = 256;
				final var deviceName = arena.allocate(size);
				final var deviceNameLength = arena.allocate(ValueLayout.JAVA_INT, size);
				final var portName = arena.allocate(size);
				final var portNameLength = arena.allocate(ValueLayout.JAVA_INT, size);

				for (int i = 0;; i++) {
					// reinit chars with size on every iteration
					deviceNameLength.set(ValueLayout.JAVA_INT, 0, size);
					portNameLength.set(ValueLayout.JAVA_INT, 0, size);
					ret = Windows_7.RegEnumValueA(MemorySegment.ofAddress(HKEY__.unused$get(serialCommKey)), i, deviceName,
							deviceNameLength, Windows_8.NULL(), Windows_8.NULL(), portName, portNameLength);

					if (ret == Windows_9.ERROR_NO_MORE_ITEMS())
						break;

					if (ret == Windows_9.ERROR_SUCCESS()) {
						final String port = extractString(portName, portNameLength);
						logger.log(TRACE, "{0} = {1}", extractString(deviceName, deviceNameLength), port);
						portNames.add(port);
					}
					else {
						logger.log(WARNING, "RegEnumValueA error: {0}", formatWinError(ret));
					}
				}
				Windows_7.RegCloseKey(serialCommKey);
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
		final String name = portId.toUpperCase().startsWith("COM") ? "\\\\.\\" + portId : portId;
		final HANDLE h = open(name, true, null);
		if (h.invalid())
			throw newIoException(name);

		this.h = h;
		try {
			final int fileType = Windows_5.GetFileType(h.handle());
			if (fileType != Windows_2.FILE_TYPE_CHAR() && fileType != Windows_2.FILE_TYPE_UNKNOWN())
				throw newIoException(name);

			final var cp = _COMMPROP.allocate(arena);
			if (Windows_6.GetCommProperties(h.handle(), cp) == 0)
				throw newIoException(name);
			if (debug()) {
				// set rx and tx queues (rx will usually keep some default buffer, 4K or similar...)
				Windows_6.SetupComm(h.handle(), 200, 200);
				if (Windows_6.GetCommProperties(h.handle(), cp) == 0)
					throw newIoException(name);
				final int tx = _COMMPROP.dwCurrentTxQueue$get(cp);
				final int rx = _COMMPROP.dwCurrentRxQueue$get(cp);
				logger.log(TRACE, "set queue tx {0} rx {1}", tx, rx);
			}
			if (Windows_6.EscapeCommFunction(h.handle(), Windows_2.SETDTR()) == 0)
				throw newIoException();

			maxBaudRate = _COMMPROP.dwMaxBaud$get(cp);
			System.out.println("max baud rate = " + maxBaudRate);
		} catch (final IOException e) {
			close();
			throw e;
		}
	}

	@Override
	void baudRate(final Arena arena, final int baudrate) throws IOException {
		if (maxBaudRate > 0 && baudrate > maxBaudRate)
			throw new IOException("baud rate %d > max. allowed baud rate %d".formatted(baudrate, maxBaudRate));

		final var dcb = commState(arena);
		_DCB.BaudRate$set(dcb, baudrate);
		setDcbBits(dcb, "fAbortOnError", Windows.FALSE());
		if (Windows_6.SetCommState(h.handle(), dcb) == 0)
			throw newIoException();
	}

	@Override
	int baudRate(final Arena arena) throws IOException {
		final var dcb = commState(arena);
		return _DCB.BaudRate$get(dcb);
	}

	@Override
	void parity(final Arena arena, final Parity parity) throws IOException {
		final var dcb = commState(arena);
		_DCB.Parity$set(dcb, (byte) parity.value());
		setDcbBits(dcb, "fParity", parity.value() != Windows_2.NOPARITY() ? 1 : 0);
		setDcbBits(dcb, "fAbortOnError", Windows.FALSE());

		final char parityReplace = '\0';
		if (parity == Parity.None) {
			setDcbBits(dcb, "fErrorChar", 0);
			_DCB.ErrorChar$set(dcb, (byte) parityReplace);
		}
		else {
			setDcbBits(dcb, "fErrorChar", parityReplace != '\0' ? 1 : 0);
			_DCB.ErrorChar$set(dcb, (byte) parityReplace);
		}

		if (Windows_6.SetCommState(h.handle(), dcb) == 0)
			throw newIoException();
	}

	@Override
	Parity parity(final Arena arena) throws IOException {
		final var dcb = commState(arena);
		return Parity.values()[_DCB.Parity$get(dcb)];
	}

	@Override
	void dataBits(final Arena arena, final int databits) throws IOException {
		final var dcb = commState(arena);
		_DCB.ByteSize$set(dcb, (byte) databits);
		setDcbBits(dcb, "fAbortOnError", Windows.FALSE());
		if (Windows_6.SetCommState(h.handle(), dcb) == 0)
			throw newIoException();
	}

	@Override
	int dataBits(final Arena arena) throws IOException {
		final var dcb = commState(arena);
		return _DCB.ByteSize$get(dcb);
	}

	@Override
	void stopBits(final Arena arena, final StopBits stopbits) throws IOException {
		final var dcb = commState(arena);
		_DCB.StopBits$set(dcb, (byte) platformStopBits(stopbits));
		setDcbBits(dcb, "fAbortOnError", Windows.FALSE());
		if (Windows_6.SetCommState(h.handle(), dcb) == 0)
			throw newIoException();
	}

	@Override
	StopBits stopBits(final Arena arena) throws IOException {
		final var dcb = commState(arena);
		return genericStopBits(_DCB.StopBits$get(dcb));
	}

	@Override
	void flowControl(final Arena arena, final FlowControl flowControl) throws IOException {
		final var dcb = commState(arena);
		setDcbBits(dcb, "fOutxDsrFlow", Windows.FALSE());
		setDcbBits(dcb, "fDtrControl", Windows_2.DTR_CONTROL_DISABLE());
		setDcbBits(dcb, "fDsrSensitivity", Windows.FALSE());
		setDcbBits(dcb, "fTXContinueOnXoff", Windows.FALSE());
		setDcbBits(dcb, "fOutX", Windows.FALSE());
		setDcbBits(dcb, "fInX", Windows.FALSE());
		_DCB.XonChar$set(dcb, (byte) 0);
		_DCB.XoffChar$set(dcb, (byte) 0);
		switch (flowControl) {
			case CtsRts -> {
				setDcbBits(dcb, "fOutxCtsFlow", Windows.TRUE());
				setDcbBits(dcb, "fRtsControl", Windows_2.RTS_CONTROL_HANDSHAKE());
			}
			case None -> {
				setDcbBits(dcb, "fOutxCtsFlow", Windows.FALSE());
				setDcbBits(dcb, "fRtsControl", Windows_2.RTS_CONTROL_DISABLE());
			}
		}

		setDcbBits(dcb, "fAbortOnError", Windows.FALSE());
		if (Windows_6.SetCommState(h.handle(), dcb) == 0)
			throw newIoException();
	}

	@Override
	FlowControl flowControl(final Arena arena) throws IOException {
		final var dcb = commState(arena);
		final int flag = getDcbBits(dcb, "fOutxCtsFlow");
		return flag == 1 ? FlowControl.CtsRts : FlowControl.None;
	}

	private MemorySegment commState(final Arena arena) throws IOException {
		final var dcb = _DCB.allocate(arena);
		_DCB.DCBlength$set(dcb, (int) _DCB.sizeof());
		if (Windows_6.GetCommState(h.handle(), dcb) == 0)
			throw newIoException();
		return dcb;
	}

	// any open input/output stream accessing this port becomes unusable
	@Override
	public void close() {
		if (h.valid()) {
			logger.log(TRACE, "close handle {0}", h);
			final boolean closed = Windows_6.CloseHandle(h.handle()) == Windows.TRUE();
			h = HANDLE.Invalid;
			if (!closed)
				logger.log(ERROR, "closing serial port: {0}", formatWinError(QUERY_GETLASTERROR));
		}
	}

	@Override
	boolean isClosed() {
		return h.invalid();
	}

	private HANDLE open(final String portId, final boolean overlapped, /*DWORD*/ final AtomicInteger lastError) {
		logger.log(TRACE, "open {0}", portId);
		try (var arena = Arena.openConfined()) {
			// clearing last error not necessary, but jvm/debugger sometimes have set some error code
//			Windows.SetLastError(0);
			final var h = Windows_5.CreateFileA(arena.allocateUtf8String(portId),
					Windows_8.GENERIC_READ() | Windows_8.GENERIC_WRITE(), 0, Windows_8.NULL(), Windows_2.OPEN_EXISTING(),
					Windows_1.FILE_ATTRIBUTE_NORMAL() | (overlapped ? Windows_2.FILE_FLAG_OVERLAPPED() : 0),
					Windows_8.NULL());
			if (lastError != null)
				lastError.set(Windows_5.GetLastError());
			return HANDLE.of(h);
		}
	}

	boolean portExists(final String portId) {
		/*DWORD*/ final var error = new AtomicInteger();
		final HANDLE h = open(portId, false, error);
		// if handle is valid, port exists, otherwise it depends on last error: on existing, but
		// used port, we would get ERROR_ACCESS_DENIED or ERROR_SHARING_VIOLATION (or similar)
		if (h.valid())
			Windows_6.CloseHandle(h.handle());
		else if (error.get() == Windows_9.ERROR_FILE_NOT_FOUND() || error.get() == Windows_9.ERROR_PATH_NOT_FOUND())
			return false;
		else
			logger.log(Level.DEBUG, "opening {0}: {1}", portId, formatWinError(error.get()));

		return true;
	}

	private static int platformStopBits(final StopBits stopbits) {
		return switch (stopbits) {
			case One -> Windows_2.ONESTOPBIT();
			case Two -> Windows_2.TWOSTOPBITS();
		};
	}

	private static StopBits genericStopBits(final int stopbits) {
		if (stopbits == Windows_2.ONESTOPBIT())
			return StopBits.One;
		if (stopbits == Windows_2.TWOSTOPBITS())
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
	private static void waitPendingIO(final HANDLE h, /*OVERLAPPED* */ final MemorySegment overlapped,
		final MemorySegment transferred) throws IOException {
//		logger.log(TRACE, "wait pending I/O");
		// the only status tolerated is I/O pending
		if (Windows_5.GetLastError() != Windows_9.NO_ERROR() && Windows_5.GetLastError() != Windows_9.ERROR_IO_PENDING())
			// some I/O problem, throw error
			throw newIoException();

		// wait for operation completion, and check result
		/*DWORD*/ final int res = Windows_6.WaitForSingleObject(_OVERLAPPED.hEvent$get(overlapped), Windows_9.INFINITE());
		if (res == Windows_9.WAIT_OBJECT_0()
				&& Windows_6.GetOverlappedResult(h.handle(), overlapped, transferred, Windows.FALSE()) != 0) {
			// completed successfully
			return;
		}
		// communication or wait error
		throw newIoException();
	}

	private static int write(final Arena arena, final HANDLE h, final MemorySegment bytes) throws IOException {
		final var event = Windows_6.CreateEventA(Windows_8.NULL(), Windows.TRUE(), Windows.FALSE(), Windows_8.NULL());
		if (event.equals(Windows_8.NULL()))
			throw newIoException();

		final var written = arena.allocate(ValueLayout.JAVA_INT, 0);
		try {
			final var o = _OVERLAPPED.allocate(arena);
			_OVERLAPPED.hEvent$set(o, event);

			if (Windows_6.WriteFile(h.handle(), bytes, (int) bytes.byteSize(), written, o) == 0)
				waitPendingIO(h, o, written);
			Windows_6.FlushFileBuffers(h.handle());
		}
		finally {
			Windows_6.CloseHandle(event);
		}
		return (int) bytes.byteSize();
		// in overlapped mode, the param for number of bytes written is useless
		// return written.get(ValueLayout.JAVA_INT, 0);
	}

	@Override
	int writeBytes(final Arena arena, final MemorySegment bytes) throws IOException {
		return write(arena, h, bytes);
	}

	private static int read(final Arena arena, final HANDLE h, final MemorySegment bytes) throws IOException {
		final var event = Windows_6.CreateEventA(Windows_8.NULL(), Windows.TRUE(), Windows.FALSE(), Windows_8.NULL());
		if (event.equals(Windows_8.NULL()))
			throw newIoException();

		final var o = _OVERLAPPED.allocate(arena);
		_OVERLAPPED.hEvent$set(o, event);

		final /*DWORD*/ var read = arena.allocate(ValueLayout.JAVA_INT, 0);
		try {
			if (Windows_6.ReadFile(h.handle(), bytes, (int) bytes.byteSize(), read, o) == 0)
				waitPendingIO(h, o, read);
		}
		finally {
			Windows_6.CloseHandle(event);
		}
		return (int) bytes.byteSize();
		// in overlapped mode, the param for number of bytes read is useless
		// return read.get(ValueLayout.JAVA_INT, 0);
	}

	@Override
	int readBytes(final Arena arena, final MemorySegment bytes) throws IOException {
		return read(arena, h, bytes);
	}

	@Override
	public void setEvents(final int eventMask, final boolean enable) throws IOException {
		logger.log(TRACE, "set event: mask 0x{0}, enable={1}",  Integer.toUnsignedString(eventMask, 16), enable);
		try (var arena = Arena.openConfined()) {
			/*DWORD*/ final var mask = arena.allocate(ValueLayout.JAVA_INT, 0);
			if (Windows_6.GetCommMask(h.handle(), mask) == 0)
				throw newIoException();

			final int m = mask.get(ValueLayout.JAVA_INT, 0);
			final int set = enable ? m | eventMask : m & ~eventMask;
			if (Windows_6.SetCommMask(h.handle(), set) == 0)
				throw newIoException();
		}
	}

	@Override
	public int waitEvent() throws IOException {
		logger.log(TRACE, "wait comm event");
		try (var arena = Arena.openConfined()) {
			final var event = Windows_6.CreateEventA(Windows_8.NULL(), Windows.TRUE(), Windows.FALSE(), Windows_8.NULL());
			if (event.equals(Windows_8.NULL()))
				throw newIoException();

			final var o = _OVERLAPPED.allocate(arena);
			_OVERLAPPED.hEvent$set(o, event);
			final /*DWORD*/ var eventMask = arena.allocate(ValueLayout.JAVA_INT, 0);
			final /*DWORD*/ var undefined = arena.allocate(ValueLayout.JAVA_INT, 0);
			if (Windows_6.WaitCommEvent(h.handle(), eventMask, o) == 0)
				waitPendingIO(h, o, undefined);
			Windows_6.CloseHandle(_OVERLAPPED.hEvent$get(o));
			// note if event mask was changed while waiting for event we return 0
			return eventMask.get(ValueLayout.JAVA_INT, 0);
		}
	}

	@Override
	int status(final Arena arena, final Status type) throws IOException {
		/*DWORD*/ final var value = arena.allocate(ValueLayout.JAVA_INT, 0);
		int ret = 0;
		final int status = switch (type) {
			case Line -> {
				// line status bit field
				// 0x0010 : CTS (clear-to-send) signal is on
				// 0x0020 : DSR (data-set-ready) signal is on
				// 0x0040 : ring indicator signal is on
				// 0x0080 : RLSD (receive-line-signal-detect) signal is on
				ret = Windows_6.GetCommModemStatus(h.handle(), value);
				yield value.get(ValueLayout.JAVA_INT, 0);
			}
			case AvailableInput -> {
				final var stat = _COMSTAT.allocate(arena);
				ret = Windows_6.ClearCommError(h.handle(), value, stat);
				yield _COMSTAT.cbInQue$get(stat);
			}
			case Error -> {
				ret = Windows_6.ClearCommError(h.handle(), value, Windows_8.NULL());
				final int errors = value.get(ValueLayout.JAVA_INT, 0);
				commErrors(errors);
				yield errors;
			}
		};
		if (ret == 0)
			throw newIoException();
		return status;
	}

	@Override
	void timeouts(final Arena arena, final Timeouts timeouts) throws IOException {
		final var to = _COMMTIMEOUTS.allocate(arena);
		_COMMTIMEOUTS.ReadIntervalTimeout$set(to, timeouts.readInterval());
		_COMMTIMEOUTS.ReadTotalTimeoutMultiplier$set(to, timeouts.readTotalMultiplier());
		_COMMTIMEOUTS.ReadTotalTimeoutConstant$set(to, timeouts.readTotalConstant());
		_COMMTIMEOUTS.WriteTotalTimeoutMultiplier$set(to, timeouts.writeTotalMultiplier());
		_COMMTIMEOUTS.WriteTotalTimeoutConstant$set(to, timeouts.writeTotalConstant());
		if (Windows_6.SetCommTimeouts(h.handle(), to) == 0)
			throw newIoException();
	}

	@Override
	Timeouts timeouts(final Arena arena) throws IOException {
		final var to = _COMMTIMEOUTS.allocate(arena);
		if (Windows_6.GetCommTimeouts(h.handle(), to) == 0)
			throw newIoException();
		return new Timeouts(_COMMTIMEOUTS.ReadIntervalTimeout$get(to),
				_COMMTIMEOUTS.ReadTotalTimeoutMultiplier$get(to), _COMMTIMEOUTS.ReadTotalTimeoutConstant$get(to),
				_COMMTIMEOUTS.WriteTotalTimeoutMultiplier$get(to), _COMMTIMEOUTS.WriteTotalTimeoutConstant$get(to));
	}

	@Override
	void dispatchEvents(final int eventMask) {
		// data events
		if (isSet(eventMask, Windows_2.EV_RXCHAR()))
			logger.log(TRACE, "EV_RXCHAR");
		if (isSet(eventMask, Windows_2.EV_RXFLAG()))
			logger.log(TRACE, "EV_RXFLAG");

		// pin events
		if (isSet(eventMask, Windows_2.EV_CTS()))
			logger.log(TRACE, "EV_CTS");
		if (isSet(eventMask, Windows_2.EV_DSR()))
			logger.log(TRACE, "EV_DSR");
		if (isSet(eventMask, Windows_2.EV_RLSD()))
			logger.log(TRACE, "EV_RLSD");
		if (isSet(eventMask, Windows_2.EV_BREAK()))
			logger.log(TRACE, "EV_BREAK");
		if (isSet(eventMask, Windows_2.EV_RING()))
			logger.log(TRACE, "EV_RING");

		// error event
		if (isSet(eventMask, Windows_2.EV_ERR()))
			logger.log(TRACE, "EV_ERR");
	}

	private void commErrors(final int errors) {
		if (isSet(errors, Windows_2.CE_RXOVER()))
			logger.log(TRACE, "CE_RXOVER");
		if (isSet(errors, Windows_2.CE_OVERRUN()))
			logger.log(TRACE, "CE_OVERRUN");
		if (isSet(errors, Windows_2.CE_RXPARITY()))
			logger.log(TRACE, "CE_RXPARITY");
		if (isSet(errors, Windows_2.CE_FRAME()))
			logger.log(TRACE, "CE_FRAME");
		if (isSet(errors, Windows_2.CE_BREAK()))
			logger.log(TRACE, "CE_BREAK");
	}

	private static final int QUERY_GETLASTERROR = -1;

	private static String formatWinError(/*DWORD*/ final int error) {
		final int err = error == QUERY_GETLASTERROR ? Windows_5.GetLastError() : error;
		final int size = 256;

		try (var arena = Arena.openConfined()) {
			final var buf = arena.allocateArray(ValueLayout.JAVA_BYTE, size);
			int len = Windows_6.FormatMessageA(
					Windows_2.FORMAT_MESSAGE_FROM_SYSTEM() | Windows_2.FORMAT_MESSAGE_IGNORE_INSERTS()
					| Windows_2.FORMAT_MESSAGE_MAX_WIDTH_MASK(),
					Windows_8.NULL(), err, 0, buf, size, Windows_8.NULL());
			while (len > 3 && (buf.get(ValueLayout.JAVA_BYTE, len - 1) == ' '
					|| buf.get(ValueLayout.JAVA_BYTE, len - 1) == '.'))
				--len;
			buf.set(ValueLayout.JAVA_BYTE, len, (byte) 0);
			return buf.getUtf8String(0) + " (error 0x" + Integer.toUnsignedString(err, 16) + ")";
		}
	}

	private static IOException newIoException() throws IOException {
		return newIoException(null, QUERY_GETLASTERROR);
	}

	private static IOException newIoException(final String s) {
		return newIoException(s, QUERY_GETLASTERROR);
	}

	private static IOException newIoException(final String s, /*DWORD*/ final int error) {
		final String errmsg = formatWinError(error);
		return new IOException(s != null ? s + ": " + errmsg : errmsg);
	}
}
