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
import static java.lang.System.Logger.Level.INFO;
import static java.lang.System.Logger.Level.TRACE;
import static java.lang.System.Logger.Level.WARNING;

import java.io.IOException;
import java.lang.foreign.Arena;
import java.lang.foreign.MemoryLayout.PathElement;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.lang.invoke.VarHandle;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.util.HashSet;
import java.util.HexFormat;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;

import org.unix.Linux;
import org.unix.dirent;
import org.unix.fd_set;
import org.unix.stat;
import org.unix.termios;
import org.unix.timeval;


/**
 * Serial communication using Panama FFM to invoke Linux/macOS C functions. The implementation of this API
 * contains platform dependent code.
 */
final class UnixSerialPort extends ReadWritePort {

	// Any Character received
	static final int EVENT_RXCHAR = 0x0001;
	// Received certain character
	static final int EVENT_RXFLAG = 0x0002;
	// Transmit Queue Empty
	static final int EVENT_TXEMPTY = 0x0004;
	// CTS changed state
	static final int EVENT_CTS = 0x0008;
	// DSR changed state
	static final int EVENT_DSR = 0x0010;
	// RLSD changed state
	static final int EVENT_RLSD = 0x0020;
	// BREAK received
	static final int EVENT_BREAK = 0x0040;
	// Line status error occurred
	private static final int EVENT_ERR = 0x0080;
	// Ring signal detected
	static final int EVENT_RING = 0x0100;

	static final int EVENT_DTR = 0x0200;
	static final int EVENT_RTS = 0x0400;


	private static final String lockDir = "/var/lock/";
	private static final String lckPrefix = "LCK..";
	private static final String pidPrefix = "PID..";

	private record fd_t(int value) {
		static final fd_t Invalid = new fd_t(-1);

		static fd_t of(final int fd) {
			if (fd == -1) return Invalid;
			return new fd_t(fd);
		}
	}

	private volatile fd_t fd = fd_t.Invalid;
	private String lockedPort = "";

	private volatile int currentEventMask;
	private volatile int ioctlEventMask;


	// only necessary on platforms like macOS which don't support waiting for events
	private final Duration eventPollInterval = Duration.ofMillis(50);
	private volatile int polledLineStatus;
	private volatile int polledErrorStatus;
	private volatile int polledAvailableStatus;

	private int receiveTimeout = 100; // ms


//	private static final List<String> defaultPortPrefixes = List.of("/dev/ttyS", "/dev/ttyACM", "/dev/ttyUSB", "/dev/ttyAMA");

	static Set<String> portIdentifiers() {
//		final var ports = new HashSet<String>();
//		defaultPortPrefixes.forEach(
//				p -> IntStream.range(0, 20).mapToObj(i -> p + i).filter(SerialPort::portExists).forEach(ports::add));
//		ports.addAll(checkPortsDir("/dev"));
//		return ports;
		return checkPortsDir("/dev");
	}

	private static Set<String> checkPortsDir(final String dir) {
		try (var arena = Arena.ofConfined()) {
			final var logger = System.getLogger(MethodHandles.lookup().lookupClass().getPackageName());
			final var ports = new HashSet<String>();

			var /*DIR*/ addr = Linux.opendir(arena.allocateFrom(dir));
			if (addr.equals(Linux.NULL())) {
				logger.log(WARNING, "can't open {0}: ", dir, errnoMsg());
				return ports;
			}

			final var dirp = addr;
			while (!(addr = Linux.readdir(dirp)).equals(Linux.NULL())) {
				final var entry = dirent.ofAddress(addr, arena);
				final String name = dirent.d_name$slice(entry).getString(0);
				// ignore entries '.' and '..'
				if (name.charAt(0) == '.' && name.length() <= 2)
					continue;

				if (debug())
					logger.log(TRACE, LayoutFormatter.format(entry, dirent.$LAYOUT()));

				final String filename = dir + "/" + name;
				logger.log(TRACE, "test {0}", filename);
				final var cfilename = arena.allocateFrom(filename);
				final var stbuf = stat.allocate(arena);
				if (Linux.stat(cfilename, stbuf) == -1) {
					logger.log(WARNING, "stat failed for {0}: {1}", filename, errnoMsg());
					continue;
				}
				if (SerialPort.portExists(filename))
					ports.add(filename);
			}

			Linux.closedir(dirp);
			return ports;
		}
	}



	// TODO only for port exists function
	UnixSerialPort() {
		super("");
	}

	UnixSerialPort(final String portId) throws IOException {
		super(portId);
		open(portId);

		final int eventMask = UnixSerialPort.EVENT_CTS | UnixSerialPort.EVENT_TXEMPTY | UnixSerialPort.EVENT_RING
				| UnixSerialPort.EVENT_BREAK | UnixSerialPort.EVENT_CTS | UnixSerialPort.EVENT_DSR
				| UnixSerialPort.EVENT_RLSD | UnixSerialPort.EVENT_RXCHAR | UnixSerialPort.EVENT_RXFLAG;

		setEvents(eventMask, true);
	}

	@Override
	void baudRate(final Arena arena, final int baudrate) throws IOException {
		final var options = tcgetattr(arena);
		final int termiosBaudrate = termiosBaudrate(baudrate);
		Linux.cfsetispeed(options, termiosBaudrate);
		Linux.cfsetospeed(options, termiosBaudrate);
		tcsetattr(options);
	}

	@Override
	int baudRate(final Arena arena) throws IOException {
		final var options = tcgetattr(arena);
		return genericBaudrate(Linux.cfgetispeed(options));
	}

	@Override
	void parity(final Arena arena, final Parity parity) throws IOException {
		final var options = tcgetattr(arena);
		setTermiosParity(options, parity);
		// XXX (c_iflag & INPCK) && !(c_iflag & IGNPAR))
		tcsetattr(options);
	}

	@Override
	Parity parity(final Arena arena) throws IOException {
		final var options = tcgetattr(arena);
		final long cflag = termios.c_cflag$get(options);
		return genericParity(cflag);
	}

	@Override
	void dataBits(final Arena arena, final int databits) throws IOException {
		final var options = tcgetattr(arena);
		setTermiosDataBits(options, databits);
		tcsetattr(options);
	}

	@Override
	int dataBits(final Arena arena) throws IOException {
		final var options = tcgetattr(arena);
		final long cflag = termios.c_cflag$get(options);
		return genericDataBits(cflag);
	}

	@Override
	void stopBits(final Arena arena, final StopBits stopbits) throws IOException {
		final var options = tcgetattr(arena);
		setTermiosStopBits(options, stopbits);
		tcsetattr(options);
	}

	@Override
	StopBits stopBits(final Arena arena) throws IOException {
		final var options = tcgetattr(arena);
		return genericStopBits(options);
	}

	@Override
	void flowControl(final Arena arena, final FlowControl flowControl) throws IOException {
		final var options = tcgetattr(arena);
		long iflag = termios.c_iflag$get(options);
		// disable SW flow ctrl
		iflag &= ~(Linux.IXON() | Linux.IXOFF() | Linux.IXANY());
		termios.c_iflag$set(options, (int) iflag);
		long cflag = termios.c_cflag$get(options);
		cflag = switch (flowControl) {
			case CtsRts -> cflag | HW_FLOWCTL;
			case None -> cflag & ~HW_FLOWCTL;
		};
		termios.c_cflag$set(options, (int) cflag);
		tcsetattr(options);
	}

	@Override
	FlowControl flowControl(final Arena arena) throws IOException {
		final var options = tcgetattr(arena);
		final long cflag = termios.c_cflag$get(options);
		return genericFlowControl(cflag);
	}

	@Override
	void open(final Arena arena, final String portId) throws IOException {
		final AtomicInteger error = new AtomicInteger();
		fd = openPort(arena, portId, true, error);
		if (error.get() == Linux.EBUSY())
			throw newException(Linux.EBUSY());

		if (debug()) {
// #if !defined TXSETIHOG
			final int TXSETIHOG = (('X' << 8) + 9);
			final int TXSETOHOG = (('X' << 8) + 10);

			// set rx and tx queues
			final var size = arena.allocate(Linux.C_INT, 200);
			// ??? how to set buffers
			if (Linux.ioctl(fd.value(), TXSETIHOG, size.address()) == -1)
				perror("TXSETIHOG");
			if (Linux.ioctl(fd.value(), TXSETOHOG, size.address()) == -1)
				perror("TXSETOHOG");

			final String out = "set queue tx %d rx %d".formatted(0, 0);
			logger.log(TRACE, out);
		}

		final var status = arena.allocate(Linux.C_INT);
		int ret = Linux.ioctl(fd.value(), Linux.TIOCMGET(), status.address());
		if (ret != -1) {
			status.set(Linux.C_INT, 0, status.get(Linux.C_INT, 0) | Linux.TIOCM_DTR());
			ret = Linux.ioctl(fd.value(), Linux.TIOCMSET(), status.address());
		}
		if (ret == -1) {
			final int err = errno();
			closePort();
			throw newException(err);
		}
	}

	// any open input/output stream accessing this port becomes unusable
	@Override
	public void close() {
		lock.lock();
		try {
			if (!closePort())
				logger.log(ERROR, "closing serial port: {0}", errnoMsg());
		}
		catch (final IOException e) {
			logger.log(ERROR, "closing serial port", e);
		}
		finally {
			eventLooper.interrupt();
			lock.unlock();
		}
	}

	@Override
	boolean isClosed() {
		return fd == fd_t.Invalid;
	}

	private static final long TIOCGSERIAL = 0x541E;
	private static final long PORT_UNKNOWN = 0;

	// this will open the port, alternatives might be to use /dev/serial or /proc/tty
	boolean portExists(final String portId) {
		final var error = new AtomicInteger();
		try (var arena = Arena.ofConfined()) {
			final fd_t fd = openPort(arena, portId, false, error);

			boolean valid = false;

			if (definedTIOCGSERIAL()) {
				final var info = serial_struct.allocate(arena);
				if (Linux.ioctl(fd.value(), TIOCGSERIAL, info.address()) == 0) {
					if (serial_struct.type$get(info) != PORT_UNKNOWN)
						valid = true;
					else if (hasDevSerialLink(arena, portId))
						valid = true;
				}
				else
					logger.log(TRACE, "check port type: {0}", errnoMsg());
			}
			else {
				final var status = arena.allocate(Linux.C_INT);
				final int ret = Linux.ioctl(fd.value(), Linux.TIOCMGET(), status.address());
				valid = ret == 0;
				logger.log(TRACE, "check port type: {0}", errnoMsg());
			}

			closePort(fd.value);
			return valid;
		}
		catch (final IOException e) {
			if (error.get() == Linux.ENOENT())
				return false;

			logger.log(TRACE, "{0}", e.getMessage());
			return false;
		}
	}

	private static String createLockName(final String dir, final String namePrefix, final String name) {
		return dir + namePrefix + name;
	}

	boolean tryLink(final MemorySegment forName, final MemorySegment linkName) {
		if (Linux.link(forName, linkName) == -1) {
			logger.log(TRACE, "create link {0} failed ({1}), try stat", linkName.getString(0), errnoMsg());
			try (var arena = Arena.ofConfined()) {
				// we are nevertheless successful if lock count states 2 locks
				final var seg = stat.allocate(arena);
				if (Linux.stat(forName, seg) == -1) {
					logger.log(TRACE, "stat {0} failed: {1}", forName, errnoMsg());
					return false;
				}
				if (debug())
					logger.log(TRACE, LayoutFormatter.format(seg, stat.$LAYOUT()));
				if (stat.st_nlink$get(seg) != 2)
					return false;
			}
		}
		Linux.unlink(forName);
		return true;
	}

	private static int readPid(final Path filename) throws IOException {
		try {
			final String s = Files.readString(filename);
			return Integer.parseInt(s);
		}
		catch (final NumberFormatException e) {
			throw new IOException(e);
		}
	}

	// try to create PID..pid and write our pid into it
	// create LCK..name as link to pid file
	// all files are located in /var/lock/
	boolean ensureLock(final String port) {
		String p = port;
		if (p.startsWith("/dev/") && p.length() > 5)
			p = p.substring(5);
		final long myPid = ProcessHandle.current().pid();
		final String pidFile = createLockName(lockDir, pidPrefix, "" + myPid);
		try (var arena = Arena.ofConfined()) {
			final var mpidFile = arena.allocateFrom(pidFile);
			final fd_t fd = fd_t.of(Linux.open(mpidFile, Linux.O_RDWR() | Linux.O_EXCL() | Linux.O_CREAT(), 0644));
			if (fd.equals(fd_t.Invalid)) {
				logger.log(WARNING, "open {0}", pidFile);
				return false;
			}

			final var mstrPid = arena.allocateFrom(Long.toString(myPid));
			/*ssize_t wr = */Linux.write(fd.value(), mstrPid, Linux.strlen(mstrPid));
			Linux.close(fd.value());
			// additional locking check, because O_EXCL in open is only
			// supported when using NFSv3 or later on kernel 2.6 or later
			// (see man open(2) )
			final String lckFile = createLockName(lockDir, lckPrefix, p);
			final var mlckFile = arena.allocateFrom(lckFile);
			if (tryLink(mpidFile, mlckFile)) {
				lockedPort = port;
				return true;
			}
			boolean locked = false;
			try {
				// lock file exists, read its pid
				// if we cannot read in the pid, we cannot lock
				final int pid = readPid(Path.of(lckFile));
				if (pid == myPid) {
					lockedPort = port;
					locked = true;
				}
				// check the read pid, if stale, try to remove lock file
				else if (Linux.kill(pid, 0) == -1 && errno() != Linux.EPERM()) {
					Linux.unlink(mlckFile);
					if (tryLink(mpidFile, mlckFile)) {
						lockedPort = port;
						return true;
					}
				}
			}
			catch (final IOException e) {
				logger.log(WARNING, "error reading pid from {0}: {1}", lckFile, e.toString());
			}
			// we might already have locked the port
			Linux.unlink(mpidFile);
			return locked;
		}
	}

	void releaseLock() throws IOException {
		if (lockedPort.isEmpty())
			return;
		final int idx = lockedPort.lastIndexOf('/');
		final String name = idx == -1 ? lockedPort : lockedPort.substring(idx + 1);
		final var lockFile = createLockName(lockDir, lckPrefix, name);
//		logger.log(TRACE, "release lock {0}", lockFile);
		lockedPort = "";
		final int pid = readPid(Path.of(lockFile));
		if (pid != -1 && pid == Linux.getpid())
			Files.deleteIfExists(Path.of(lockFile));
	}

	private boolean setPortDefaults(final int fd) {
		try (var arena = Arena.ofConfined()) {
			final var options = termios.allocate(arena);
			if (Linux.tcgetattr(fd, options) == -1) {
				logger.log(WARNING, "setting port defaults, tcgetattr: {0}", errnoMsg());
				return false;
			}
			Linux.cfsetispeed(options, Linux.B9600());
			Linux.cfsetospeed(options, Linux.B9600());
			long cflag = termios.c_cflag$get(options);
			cflag |= Linux.CREAD() | Linux.CLOCAL();
			cflag &= ~Linux.CSIZE();
			cflag |= Linux.CS8();
			termios.c_cflag$set(options, (int) cflag);
			termios.c_iflag$set(options, Linux.INPCK());
			termios.c_lflag$set(options, 0);
			termios.c_oflag$set(options, 0);
			termios.c_cc$slice(options).set(Linux.C_CHAR, Linux.VMIN(), (byte) 0);
			termios.c_cc$slice(options).set(Linux.C_CHAR, Linux.VTIME(), (byte) 0);
			if (Linux.tcsetattr(fd, Linux.TCSANOW(), options) == -1) {
				logger.log(WARNING, "setting port defaults, tcsetattr: {0}", errnoMsg());
				return false;
			}
			Linux.fcntl(fd, Linux.F_SETOWN(), Linux.getpid());
		}
		return true;
	}

	private fd_t openPort(final Arena arena, final String portId, final boolean configurePort,
		final AtomicInteger lastError) throws IOException {
		fd_t fd;
//		errno(0);
		int error = 0;

		if (ensureLock(portId)) {
			final var port = arena.allocateFrom(portId);

			do {
				// we set the port exclusive below, not here
				fd = fd_t.of(Linux.open(port, /*O_EXCL |*/Linux.O_RDWR() | Linux.O_NOCTTY() | Linux.O_NONBLOCK()));
				if (!fd.equals(fd_t.Invalid))
					break;
			}
			while (errno() == Linux.EINTR());
		}
		else
			throw new IOException("obtaining port lock for " + portId + ": " + errnoMsg());
		// check if someone else has opened the port
		if (fd.equals(fd_t.Invalid)) {
			lastError.set(errno());
			releaseLock();
			throw new IOException("opening " + portId + ": " + errnoMsg());
		}
		if (errno() == Linux.EBUSY()) {
			logger.log(TRACE, "busy {0}", fd);
			lastError.set(errno());
			releaseLock();
			return fd;
		}

		// we continue if we are not able to set exclusive mode
		if (Linux.ioctl(fd.value(), Linux.TIOCEXCL()) == -1) {
			error = errno();
			perror("set exclusive");
		}

		if (configurePort) {
			// we continue if we are not able to save old port settings
			// TODO actually restore them on close
			final var saved = termios.allocate(arena);
			if (Linux.tcgetattr(fd.value(), saved) == -1) {
				error = errno();
				perror("save old port settings");
			}

			setPortDefaults(fd.value);
		}

		lastError.set(error);
		return fd;
	}

	private boolean hasDevSerialLink(final Arena arena, final String portId) {
		final String dir = "/dev/serial/by-id";
		var /*DIR*/ addr = Linux.opendir(arena.allocateFrom(dir));
		if (addr.equals(Linux.NULL())) {
			logger.log(WARNING, "can't open {0}: ", dir, errnoMsg());
			return false;
		}

		final var dirp = addr;
		while (!(addr = Linux.readdir(dirp)).equals(Linux.NULL())) {
			final var entry = dirent.ofAddress(addr, arena);
			final String name = dirent.d_name$slice(entry).getString(0);
			// ignore entries '.' and '..'
			if (name.charAt(0) == '.' && name.length() <= 2)
				continue;

			final String filename = dir + "/" + name;
			final var cfilename = arena.allocateFrom(filename);
			final var stbuf = stat.allocate(arena);
			if (Linux.stat(cfilename, stbuf) == -1) {
				logger.log(WARNING, "stat failed for {0}: {1}", filename, errnoMsg());
				continue;
			}

			final var cresolved = arena.allocate(Linux.PATH_MAX());
			Linux.realpath(cfilename, cresolved);
			final String resolved = cresolved.getString(0);

			if (portId.equals(resolved)) {
				logger.log(TRACE, "{0} -> {1}", name, resolved);
				Linux.closedir(dirp);
				return true;
			}
		}
		Linux.closedir(dirp);
		return false;
	}

	private static boolean definedTIOCGSERIAL() {
		return OS.current() == OS.Linux;
	}

	private boolean closePort() throws IOException {
		try {
			return closePort(fd.value());
		}
		finally {
			fd = fd_t.Invalid;
		}
	}

	private boolean closePort(final int fd) throws IOException {
		if (fd == fd_t.Invalid.value())
			return true;
		logger.log(TRACE, "close handle {0}", fd);
		boolean closed;
		do {
			closed = Linux.close(fd) == 0;
			if (closed)
				break;
		}
		while (errno() == Linux.EINTR());

		releaseLock();

		return closed;
	}

	private void tcsetattr(final MemorySegment options) throws IOException {
		if (Linux.tcsetattr(fd.value(), Linux.TCSANOW(), options) == -1) {
			logger.log(WARNING, "tcsetattr: {0}", errnoMsg());
			throw newException(errno());
		}
	}

	private MemorySegment tcgetattr(final Arena arena) throws IOException {
		final var options = termios.allocate(arena);
		if (Linux.tcgetattr(fd.value(), options) == -1)
			throw newException(errno());
		return options;
	}

	// CSTOPB set corresponds to 2 stop bits, default is 1 stop bit
	private static void setTermiosStopBits(final MemorySegment cflags, /*uint8_t*/ final StopBits stopbits) {
		long flags = termios.c_cflag$get(cflags);

		flags &= ~Linux.CSTOPB();
		switch (stopbits) {
			case Two:
				flags |= Linux.CSTOPB();
				break;
//		case STOPBITS_15: /* does not exist */
//			break;
			case One:
				break;
			default:
				break;
		}
		termios.c_cflag$set(cflags, (int) flags);
	}

	private static StopBits genericStopBits(final MemorySegment cflags) {
		final long flags = termios.c_cflag$get(cflags);
		final int stopbits = (int) (flags & Linux.CSTOPB());
		if (stopbits == Linux.CSTOPB())
			return StopBits.Two;
		return StopBits.One;
	}

	// non-existing on macOS
	private static final int CMSPAR = 010000000000;

	private static boolean markSpaceParitySupport() {
		return OS.current() == OS.Linux;
	}

	private void setTermiosParity(final MemorySegment flags, final Parity parity) {
		long cflags = termios.c_cflag$get(flags);
		cflags &= ~(Linux.PARENB() | Linux.PARODD() | CMSPAR);

		switch (parity) {
			case None:
				break;
			case Even:
				cflags |= Linux.PARENB();
				break;
			case Odd:
				cflags |= Linux.PARENB() | Linux.PARODD();
				break;
			case Mark:
				if ((cflags & Linux.CS7()) == Linux.CS7()) {
					logger.log(TRACE, "setting mark parity for 7M1");
					// for mark parity use 7 data bits, 2 stop bits
					cflags &= ~Linux.CSIZE();
					cflags |= Linux.CSTOPB() | Linux.CS7();
				}
				else if ((cflags & Linux.CS8()) == Linux.CS8()) {
					logger.log(TRACE, "setting mark parity for 8M1");
					if (markSpaceParitySupport())
						cflags |= CMSPAR | Linux.PARENB() | Linux.PARODD();
					else {
						// emulate with 8N2
						cflags |= Linux.CSTOPB();
					}
				}
				break;
			case Space:
				if ((cflags & Linux.CS8()) == Linux.CS8()) {
					if (markSpaceParitySupport()) {
						cflags |= CMSPAR | Linux.PARENB();
						cflags &= ~Linux.PARODD();
					}
					else {
						// NYI emulate 8S1
					}
				}
		}
		termios.c_cflag$set(flags, (int) cflags);
	}

	private static Parity genericParity(final long cflags) {
		if (markSpaceParitySupport()) {
			if ((cflags & Linux.PARENB()) == Linux.PARENB() && (cflags & Linux.PARODD()) == Linux.PARODD()
					&& (cflags & CMSPAR) == CMSPAR)
				return Parity.Mark;
			if ((cflags & Linux.PARENB()) == Linux.PARENB() && (cflags & CMSPAR) == CMSPAR)
				return Parity.Space;
		}

		if ((cflags & Linux.PARENB()) != 0 && (cflags & Linux.PARODD()) != 0)
			return Parity.Odd;
		if ((cflags & Linux.PARENB()) != 0)
			return Parity.Even;

		return Parity.None;
	}

	private static int termiosBaudrate(final /*uint*/ int baudrate) {
		return switch (baudrate) {
			case 0 -> Linux.B0();
			case 50 -> Linux.B50();
			case 75 -> Linux.B75();
			case 110 -> Linux.B110();
			case 134 -> Linux.B134();
			case 150 -> Linux.B150();
			case 200 -> Linux.B200();
			case 300 -> Linux.B300();
			case 600 -> Linux.B600();
			case 1200 -> Linux.B1200();
			case 1800 -> Linux.B1800();
			case 2400 -> Linux.B2400();
			case 4800 -> Linux.B4800();
			case 9600 -> Linux.B9600();
//#ifdef B14400
// undefined on Linux
//		case 14400:
//			return Linux.B14400();
//#endif /* B14400 */
			case 19200 -> Linux.B19200();
//#ifdef B28800
// undefined on Linux
//		case 28800:
//			return Linux.B28800();
//#endif /* B28800 */
			case 38400 -> Linux.B38400();
//#ifdef B57600 // MacOS X does not define this Baud rate
			case 57600 -> Linux.B57600();
//#endif // B57600
//#ifdef B115200
			case 115200 -> Linux.B115200();
//#endif /*  B115200 */
//#ifdef B230400
			case 230400 -> Linux.B230400();
//#endif /* B230400 */
//#ifdef B460800
//    case 460800:
//        return Linux.B460800();
//#endif /* B460800 */
//#ifdef B500000
//    case 500000:
//        return Linux.B500000();
//#endif /* B500000 */
//#ifdef B576000
//    case 576000:
//        return Linux.B576000();
//#endif /* B57600 */
//#ifdef B921600
//    case 921600:
//        return Linux.B921600();
//#endif /* B921600 */
//#ifdef B1000000
//    case 1000000:
//        return Linux.B1000000();
//#endif /* B1000000 */
//#ifdef B1152000
//    case 1152000:
//        return Linux.B1152000();
//#endif /* B1152000 */
//#ifdef B1500000
//    case 1500000:
//        return Linux.B1500000();
//#endif /* B1500000 */
//#ifdef B2000000
//    case 2000000:
//        return Linux.B2000000();
//#endif /* B2000000 */
//#ifdef B2500000
//    case 2500000:
//        return Linux.B2500000();
//#endif /* B2500000 */
//#ifdef B3000000
//    case 3000000:
//        return Linux.B3000000();
//#endif /* B3000000 */
//#ifdef B3500000
//    case 3500000:
//        return Linux.B3500000();
//#endif /* B3500000 */
//#ifdef B4000000
//    case 4000000:
//        return Linux.B4000000();
//#endif /* B4000000 */
			default -> -1;
		};
	}

	private static int genericBaudrate(final long baudrate) {
		if (baudrate == Linux.B0())
			return 0;
		else if (baudrate == Linux.B50())
			return 50;
		else if (baudrate == Linux.B75())
			return 75;
		else if (baudrate == Linux.B110())
			return 110;
		else if (baudrate == Linux.B134())
			return 134;
		else if (baudrate == Linux.B150())
			return 150;
		else if (baudrate == Linux.B200())
			return 200;
		else if (baudrate == Linux.B300())
			return 300;
		else if (baudrate == Linux.B600())
			return 600;
		else if (baudrate == Linux.B1200())
			return 1200;
		else if (baudrate == Linux.B1800())
			return 1800;
		else if (baudrate == Linux.B2400())
			return 2400;
		else if (baudrate == Linux.B4800())
			return 4800;
		else if (baudrate == Linux.B9600())
			return 9600;
//		else if (baudrate == Linux.B14400())
//			return 14400;
		else if (baudrate == Linux.B19200())
			return 19200;
//		else if (baudrate == Linux.B28800())
//			return 28800;
		else if (baudrate == Linux.B38400())
			return 38400;
		else if (baudrate == Linux.B57600())
			return 57600;
		else if (baudrate == Linux.B115200())
			return 115200;
		else if (baudrate == Linux.B230400())
			return 230400;
		else
			return -1;

//#endif /* B230400 */
//#ifdef B460800
//    case Linux.B460800():
//        return 460800;
//#endif /* B460800 */
//#ifdef B500000
//    case Linux.B500000():
//        return 500000;
//#endif /* B500000 */
//#ifdef B576000
//    case Linux.B576000():
//        return 576000;
//#endif /* B576000 */
//#ifdef B921600
//    case Linux.B921600():
//        return 921600;
//#endif /* B921600 */
//#ifdef B1000000
//    case Linux.B1000000():
//        return 1000000;
//#endif /* B1000000 */
//#ifdef B1152000
//    case Linux.B1152000():
//        return 1152000;
//#endif /* B1152000 */
//#ifdef B1500000
//    case Linux.B1500000():
//        return 1500000;
//#endif /* B1500000 */
//#ifdef B2000000
//    case Linux.B2000000():
//        return 2000000;
//#endif /* B2000000 */
//#ifdef B2500000
//    case Linux.B2500000():
//        return 2500000;
//#endif /* B2500000 */
//#ifdef B3000000
//    case Linux.B3000000():
//        return 3000000;
//#endif /* B3000000 */
//#ifdef B3500000
//    case Linux.B3500000():
//        return 3500000;
//#endif /* B3500000 */
//#ifdef B4000000
//    case Linux.B4000000():
//        return 4000000;
//#endif /* B4000000 */
//    default:
//        return -1;
//    }
	}

	private static void setTermiosDataBits(final MemorySegment flags, /*uint8_t*/ final int databits) {
		long cflags = termios.c_cflag$get(flags);
		cflags &= ~Linux.CSIZE();
		cflags |= switch (databits) {
			case 5 -> Linux.CS5();
			case 6 -> Linux.CS6();
			case 7 -> Linux.CS7();
			default -> Linux.CS8();
		};
		termios.c_cflag$set(flags, (int) cflags);
	}

	private static /*uint8_t*/ int genericDataBits(final long cflags) {
		final long i = cflags & Linux.CSIZE();
		if (i == Linux.CS5())
			return 5;
		else if (i == Linux.CS6())
			return 6;
		else if (i == Linux.CS7())
			return 7;
		else if (i == Linux.CS8())
			return 8;
		else
			return -1;
	}

	private static final int HW_FLOWCTL;
	static {
		// check for the existence of the two HW flow control identifiers
//	#if defined CNEW_RTSCTS
//		#define HW_FLOWCTL CNEW_RTSCTS
//	#elif defined CRTSCTS
//		#define HW_FLOWCTL CRTSCTS
//	#else
//		#define HW_FLOWCTL 0
//	#endif

		HW_FLOWCTL = Linux.CRTSCTS();
	}

	// NYI SW flow control missing
	private static /*uint*/ FlowControl genericFlowControl(final long cflags) {
		if ((cflags & HW_FLOWCTL) != 0)
			return FlowControl.CtsRts;
		return FlowControl.None;
	}

	private static boolean drain(final fd_t fd) {
		int ret;
		do {
			ret = Linux.tcdrain(fd.value());
		}
		while (ret != 0 && errno() == Linux.EINTR());
		return ret == 0;
	}

	@Override
	int readBytes(final Arena arena, final MemorySegment bytes) throws IOException {
		final long r = read(arena, fd(), bytes);
		if (r == -1)
			throw newException(errno());
		return (int) r;
	}

	private fd_t fd() throws IOException {
		final var lfd = fd;
		if (lfd.equals(fd_t.Invalid))
			throw newException(Linux.EBADF());
		return lfd;
	}

	private long read(final Arena arena, final fd_t fd, final MemorySegment buffer) {
		final long start = System.nanoTime();

		final int maxFd = fd.value() + 1;
		long offset = 0;
		long remaining = buffer.byteSize();

		final var input = fd_set.allocate(arena);
		while (remaining > 0) {
			input.fill((byte) 0);
			FD_SET(fd, input);

			final var timeout = timeval.allocate(arena);
			timeval.tv_sec$set(timeout, 0);
			timeval.tv_usec$set(timeout, receiveTimeout * 1000);
			final int n = Linux.select(maxFd, input, MemorySegment.NULL, MemorySegment.NULL, timeout);
			if (n == -1) {
				perror("select failed");
				// EINTR: simply continue with the byte counting loop, since we have to calculate
				// a new timeout
				if (errno() == Linux.EINTR())
					;
				else
					// e.g., EBADF
					break;
			}
			else if (n == 0) { // read timeout
//					trace("read timeout");
				break;
			}
			else {
				// read received bytes from stream into buffer
				if (FD_ISSET(fd, input)) {

					// get number of bytes that are immediately available for reading:
					// if 0, this indicates other errors, e.g., disconnected usb adapter
					final /*size_t*/ var nread = arena.allocate(ValueLayout.JAVA_LONG, 0);
					Linux.ioctl(fd.value(), Linux.FIONREAD(), nread.address());
					if (nread.get(ValueLayout.JAVA_LONG, 0) == 0)
						return -1;

					final long ret = Linux.read(fd.value(), MemorySegment.ofAddress(buffer.address() + offset), remaining);
					if (ret == -1) {
						perror("read");
						// retry if error is EAGAIN or Linux.EINTR(), otherwise bail out
						//#ifdef EWOULDBLOCK
						if (errno() == Linux.EWOULDBLOCK()) // alias for EAGAIN
							;
						else
							//#endif // EWOULDBLOCK
							if (errno() == Linux.EAGAIN() || errno() == Linux.EINTR())
								;
							else
								break;
					}
					else {
						offset += ret;
						remaining -= ret;
					}
				}
			}
		}
		if (debug()) {
			if (offset > 0) {
				final long end = System.nanoTime();
				final long diff = end - start;
				final byte[] data = buffer.asSlice(0, offset).toArray(ValueLayout.JAVA_BYTE);
				final String hex = HexFormat.ofDelimiter(" ").formatHex(data);
				logger.log(TRACE, "read data [{0} us] (length {1}): {2}", diff / 1000, offset, hex);
			}
		}
		return offset;
	}

	@Override
	int writeBytes(final Arena arena, final MemorySegment bytes) throws IOException {
		return (int) write(fd, bytes);
	}

	private long write(final fd_t fd, /*uint8_t*/ final MemorySegment buf) throws IOException {
		final long written = Linux.write(fd.value(), buf, buf.byteSize());
		if (written < 0)
			throw newException(errno());
		else if (!drain(fd))
			throw newException(errno());
		logger.log(TRACE, "end write");
		return written;
	}

	private static /*uint*/ long isInputWaiting(final Arena arena, final fd_t fd) throws IOException {
		Linux.fcntl(fd.value(), Linux.F_SETFL(), Linux.O_NONBLOCK());
		/*uint*/ final var bytes = arena.allocate(ValueLayout.JAVA_INT);
		if (Linux.ioctl(fd.value(), Linux.FIONREAD(), bytes.address()) == -1) {

			//#if defined FIORDCHK
			if (definedFIORDCHK()) {
				//        final int rdchk = Linux.ioctl(fd, Linux.FIORDCHK, 0);
				//        if (rdchk > -1)
				//            bytes = rdchk;
				//        else
				//        	throw newException(errno());
			} // FIORDCHK
			else
				throw newException(errno());
		}
		return bytes.get(ValueLayout.JAVA_INT, 0);
	}

	private static boolean definedFIORDCHK() {
		return false;
	}

	@Override
	public void setEvents(final int eventMask, final boolean enable) {
		int events = 0;
		// XXX assign those
		if ((eventMask & EVENT_RXCHAR) != 0)
			events |= 0;
		if ((eventMask & EVENT_RXFLAG) != 0)
			events |= 0;
		if ((eventMask & EVENT_TXEMPTY) != 0)
			events |= 0;

		if ((eventMask & EVENT_CTS) != 0)
			events |= Linux.TIOCM_CTS();
		if ((eventMask & EVENT_DSR) != 0)
			events |= Linux.TIOCM_DSR();
		if ((eventMask & EVENT_RLSD) != 0)
			events |= Linux.TIOCM_CAR();
		// XXX assign those
		if ((eventMask & EVENT_BREAK) != 0)
			events |= 0;
		if ((eventMask & EVENT_ERR) != 0)
			events |= 0;
		if ((eventMask & EVENT_RING) != 0)
			events |= Linux.TIOCM_RNG();

		if (enable) {
			currentEventMask |= eventMask;
			ioctlEventMask |= events;
		}
		else {
			currentEventMask &= ~eventMask;
			ioctlEventMask &= ~events;
		}

		logger.log(TRACE, "set events {0}", Integer.toUnsignedString(currentEventMask, 16));
	}

	private int cts, dsr, rng, dcd;
	private int rx, tx;
	private int frame, overrun, parity, brk;
	private int buf_overrun;

	private /*uint*/ long genericLineEvents(final MemorySegment icount) {
		/*uint*/ long events = 0;
		if (serial_icounter_struct.rx$get(icount) != rx) {
			rx = serial_icounter_struct.rx$get(icount);
			events |= EVENT_RXCHAR;
			events |= EVENT_RXFLAG;
		}
		if (serial_icounter_struct.cts$get(icount) != cts) {
			cts = serial_icounter_struct.cts$get(icount);
			events |= EVENT_CTS;
		}
		if (serial_icounter_struct.dsr$get(icount) != dsr) {
			dsr = serial_icounter_struct.dsr$get(icount);
			events |= EVENT_DSR;
		}
		if (serial_icounter_struct.dcd$get(icount) != dcd) {
			dcd = serial_icounter_struct.dcd$get(icount);
			events |= EVENT_RLSD;
		}
		if (serial_icounter_struct.brk$get(icount) != brk) {
			brk = serial_icounter_struct.brk$get(icount);
			events |= EVENT_BREAK;
		}
		if (serial_icounter_struct.frame$get(icount) != frame
				|| serial_icounter_struct.buf_overrun$get(icount) != buf_overrun
				|| serial_icounter_struct.parity$get(icount) != parity) {
			frame = serial_icounter_struct.frame$get(icount);
			buf_overrun = serial_icounter_struct.buf_overrun$get(icount);
			parity = serial_icounter_struct.parity$get(icount);
			events |= EVENT_ERR;
		}
		if (serial_icounter_struct.rng$get(icount) != rng) {
			rng = serial_icounter_struct.rng$get(icount);
			events |= EVENT_RING;
		}
		return events;
	}

	// not defined on macOS X
	private static final int TIOCMIWAIT = 0x545C;
	private static final int TIOCGICOUNT = 0x545D;

	private MemorySegment queryInterruptCounters() throws IOException {
		logger.log(TRACE, "queryInterruptCounters");
		try (var arena = Arena.ofConfined()) {
			final var icount = serial_icounter_struct.allocate(arena);
			int ret;
			do {
				ret = Linux.ioctl(fd.value(), TIOCGICOUNT, icount);
			}
			while (ret == -1 && errno() == Linux.EINTR());
			if (ret == -1)
				throw newException(errno());
			return icount;
		}
	}

	private static final int TIOCSERGETLSR = 0x5459;	/* Get line status register */
	private static final int TIOCSER_TEMT = 0x01;		/* Transmitter physically empty */

	private boolean lsr() {
		try (var arena = Arena.ofConfined()) {
			final var lsr = arena.allocate(1);
			if (Linux.ioctl(fd.value(), TIOCSERGETLSR, lsr) == -1)
				return false;
			// output buffer empty?
			return (lsr.get(ValueLayout.JAVA_BYTE, 0) & TIOCSER_TEMT) != 0;
		}
	}

	@Override
	public int waitEvent() throws IOException {
		logger.log(TRACE, "enter wait event");
		try {
			if (OS.current() == OS.Mac) {
				logger.log(INFO, "waitEvent on macOS, not expected to work yet");
				return polledWaitEvent();
			}

			final int mask = ioctlEventMask;
			int ret;
			do {
				ret = Linux.ioctl(fd.value(), TIOCMIWAIT, mask);
				if (ret == -1 && errno() == Linux.EINTR())
					logger.log(TRACE, "waitEvent interrupted");
			}
			while (ret == -1 && errno() == Linux.EINTR());
			if (ret == -1)
				throw newException(errno());

			final int empty = lsr() ? EVENT_TXEMPTY : 0;
			// NYI Win behavior: if event mask was changed while waiting for event we return 0
			return empty | (int) genericLineEvents(queryInterruptCounters());
		}
		finally {
			logger.log(TRACE, "exit wait event");
		}
	}

	private int polledWaitEvent() throws IOException {
		try (var arena = Arena.ofConfined()) {
			final int maxFd = fd().value() + 1;
			final var fdset = fd_set.allocate(arena);
			final var timeout = timeval.allocate(arena);

			while (true) {
				fdset.fill((byte) 0);
				timeval.tv_sec$set(timeout, (int) (eventPollInterval.toSeconds()));
				timeval.tv_usec$set(timeout, eventPollInterval.toNanosPart() / 1000);
				int ret;
				do {
					ret = Linux.select(maxFd, fdset, Linux.NULL(), Linux.NULL(), timeout);
				} while (ret == -1 && errno() == Linux.EINTR());

				if ((currentEventMask & (EVENT_CTS | EVENT_DSR | EVENT_RING | EVENT_RLSD)) != 0) {
					final int lineStatus = status(Status.Line);
					if ((polledLineStatus != lineStatus)) {
						int events = 0;
						if ((polledLineStatus & LINE_CTS) != (lineStatus & LINE_CTS))
							events |= EVENT_CTS;
						if ((polledLineStatus & LINE_DSR) != (lineStatus & LINE_DSR))
							events |= EVENT_DSR;
						if ((polledLineStatus & LINE_RING) != (lineStatus & LINE_RING))
							events |= EVENT_RING;
						if ((polledLineStatus & LINE_DCD) != (lineStatus & LINE_DCD))
							events |= EVENT_RLSD;
						if ((polledLineStatus & DTR) != (lineStatus & DTR))
							events |= EVENT_DTR;
						if ((polledLineStatus & RTS) != (lineStatus & RTS))
							events |= EVENT_RTS;

						polledLineStatus = lineStatus;
						return events;
					}
				}

				if ((currentEventMask & EVENT_ERR) != 0) {
					final int errorStatus = status(Status.Error);
					if (polledErrorStatus != errorStatus) {
						final int events = 0;

						polledErrorStatus = errorStatus;
						return events;
					}
				}

				if ((currentEventMask & (EVENT_RXCHAR | EVENT_RXFLAG)) != 0) {
					final int availStatus = status(Status.AvailableInput);
					if (polledAvailableStatus != availStatus) {
						final int events = currentEventMask & (EVENT_RXCHAR | EVENT_RXFLAG);
						polledAvailableStatus = availStatus;
						return events;
					}
				}

//				try {
//					Thread.sleep(eventPollInterval);
//				}
//				catch (final InterruptedException e) {
//					Thread.currentThread().interrupt();
//					throw new InterruptedIOException();
//				}
			}
		}
	}

	@Override
	int status(final Arena arena, final Status type) throws IOException {
		// clear communication error and get device status
		return (int) /*uint*/ switch (type) {
			case Line -> {
				/*uint*/ final var mstatus = arena.allocate(ValueLayout.JAVA_INT);
				final int ret = Linux.ioctl(fd.value(), Linux.TIOCMGET(), mstatus.address());
				if (ret == -1)
					throw newException(errno());

				long v = 0;
				final long status = mstatus.get(ValueLayout.JAVA_INT, 0);
				if ((status & Linux.TIOCM_DTR()) != 0)
					v |= DTR;
				if ((status & Linux.TIOCM_RTS()) != 0)
					v |= RTS;
				if ((status & Linux.TIOCM_CTS()) != 0)
					v |= LINE_CTS;
				if ((status & Linux.TIOCM_DSR()) != 0)
					v |= LINE_DSR;
				if ((status & Linux.TIOCM_RNG()) != 0)
					v |= LINE_RING;
				if ((status & Linux.TIOCM_CAR()) != 0)
					v |= LINE_DCD;
				yield v;
			}
			case AvailableInput -> isInputWaiting(arena, fd);
			case Error -> 0; // XXX implement error status
			// 0x0010 : detected a break condition
			// 0x0008 : detected a framing error
			// 0x0002 : character-buffer overrun
			// 0x0001 : input buffer overflow (no room in the input buffer, or character received after EOF)
			// 0x0004 : detected a parity error
		};
	}

	// for readIntervalTimeout, so it has same behavior as in windows when set to this specific value
	private static final /*uint*/ int MAXDWORD = 0xffff_ffff;

	@Override
	void timeouts(final Arena arena, final Timeouts timeouts) throws IOException {
		final int readIntervalTimeout = timeouts.readInterval();
		final int readTotalTimeoutMultiplier = timeouts.readTotalMultiplier();
		final int readTotalTimeoutConstant = timeouts.readTotalConstant();
//		final int writeTotalTimeoutMultiplier = timeouts.writeTotalMultiplier();
//		final int writeTotalTimeoutConstant = timeouts.writeTotalConstant();

		// VMIN specifies the minimum number of characters to read. If set to 0, then the VTIME
		// value specifies the time to wait for every character read. If VMIN is non-zero, VTIME
		// specifies the time to wait for the first character read. If a character is read within
		// the time given, read will block until all VMIN characters are read. That is, once the first
		// character is read, the serial interface driver expects to receive an entire packet of
		// characters (VMIN bytes total). If no character is read within the time allowed, then the
		// call to read returns 0. VTIME specifies the amount of time to wait for incoming characters
		// in tenths of seconds. If VTIME is 0, read waits indefinitely unless the NDELAY option is set.

		final var options = termios.allocate(arena);
		if (Linux.tcgetattr(fd.value(), options) == -1)
			throw newException(errno());

		int vmin = 0;
		int vtime = 0;
		if (readTotalTimeoutMultiplier == 0 && readTotalTimeoutConstant == 0 && readIntervalTimeout == MAXDWORD) {
			// setting: return immediately from read, even if no characters read
			vmin = 0;
			vtime = 0;
			logger.log(TRACE, "set timeouts: return immediately from read, even if no characters read");
		}
		else if (readTotalTimeoutMultiplier == 0 && readTotalTimeoutConstant == 0) {
			// setting: no total timeouts used for read operations, block for next char
			vmin = 1;
			vtime = 0;
			logger.log(TRACE, "set timeouts: no total timeouts used for read operations, block for next char");
		}
		else if (readIntervalTimeout > 0) {
			// setting: enforce an inter-character timeout after reading the first char
			vmin = 255;
			vtime = (readIntervalTimeout / 100);
			logger.log(TRACE, "set timeouts: enforce an inter-character timeout after reading the first char");
		}
		else if (readTotalTimeoutConstant > 0 && readTotalTimeoutMultiplier == 0 && readIntervalTimeout == 0) {
			// setting: set a maximum timeout to wait for next character
			vmin = 0;
			vtime = (readTotalTimeoutConstant / 100);
			if (vtime == 0)
				vtime = 1;
			receiveTimeout = readTotalTimeoutConstant;
			logger.log(TRACE, "set timeouts: set a maximum timeout to wait for next character: {0} ms", receiveTimeout);
		}
		else {
			// XXX hmm
			// if we're here we have: multiplier > 0 or totalConstant > 0 or interval = 0
		}

		termios.c_cc$slice(options).set(ValueLayout.JAVA_BYTE, Linux.VMIN(), (byte) vmin);
		termios.c_cc$slice(options).set(ValueLayout.JAVA_BYTE, Linux.VTIME(), (byte) vtime);

		if (Linux.tcsetattr(fd.value(), Linux.TCSANOW(), options) == -1) {
			logger.log(WARNING, "set timeouts, tcsetattr: {0}", errnoMsg());
			throw newException(errno());
		}
	}

	@Override
	Timeouts timeouts(final Arena arena) throws IOException {
		final var options = termios.allocate(arena);
		if (Linux.tcgetattr(fd.value(), options) == -1) {
			logger.log(WARNING, "get timeouts, tcgetattr {0}", errnoMsg());
			throw newException(errno());
		}

		final byte vmin = termios.c_cc$slice(options).get(ValueLayout.JAVA_BYTE, Linux.VMIN());
		final byte vtime = termios.c_cc$slice(options).get(ValueLayout.JAVA_BYTE, Linux.VTIME());

		/*uint*/ int readIntervalTimeout = 0;
		/*uint*/ int readTotalTimeoutMultiplier = 0;
		/*uint*/ int readTotalTimeoutConstant = 0;
		/*uint*/ final int writeTotalTimeoutMultiplier = 0;
		/*uint*/ final int writeTotalTimeoutConstant = 0;

		if (vmin > 0 && vtime == 0) {
			readIntervalTimeout = readTotalTimeoutMultiplier = readTotalTimeoutConstant = 0;
		}
		else if (vmin == 0 && vtime > 0) {
			readTotalTimeoutConstant = vtime * 100;
			readIntervalTimeout = readTotalTimeoutMultiplier = 0;
		}
		else if (vmin > 0 && vtime > 0) {
			readIntervalTimeout = vtime * 100;
		}
		else if (vmin == 0 && vtime == 0) {
			readIntervalTimeout = MAXDWORD;
			readTotalTimeoutConstant = readTotalTimeoutMultiplier = 0;
		}
		return new Timeouts(readIntervalTimeout, readTotalTimeoutMultiplier, readTotalTimeoutConstant,
				writeTotalTimeoutMultiplier, writeTotalTimeoutConstant);
	}

	@Override
	void dispatchEvents(final int eventMask) {
		// data events
		if (isSet(eventMask, EVENT_CTS))
			logger.log(TRACE, "EVENT_CTS");
		if (isSet(eventMask, EVENT_DSR))
			logger.log(TRACE, "EVENT_DSR");
		if (isSet(eventMask, EVENT_RING))
			logger.log(TRACE, "EVENT_RING");
		if (isSet(eventMask, EVENT_RLSD))
			logger.log(TRACE, "EVENT_RLSD");
		if (isSet(eventMask, EVENT_RXCHAR))
			logger.log(TRACE, "EVENT_RXCHAR");
		if (isSet(eventMask, EVENT_RXFLAG))
			logger.log(TRACE, "EVENT_RXFLAG");
		if (isSet(eventMask, EVENT_BREAK))
			logger.log(TRACE, "EVENT_BREAK");
		if (isSet(eventMask, EVENT_DTR))
			logger.log(TRACE, "EVENT_DTR");
		if (isSet(eventMask, EVENT_RTS))
			logger.log(TRACE, "EVENT_RTS");
	}

	private static IOException newException(final int error) {
		return new IOException(errnoMsg(error) + " (" + error + ")");
	}


	private static final int __DARWIN_NBBY = 8;                               /* bits in a byte */
	private static final int __DARWIN_NFDBITS = /*sizeof(__int32_t)*/ 4 * __DARWIN_NBBY; /* bits per mask */

	private static final VarHandle fds_bits;
	static {
		final String name = OS.current() == OS.Mac ? "fds_bits" : "__fds_bits";
		final var valueLayout = (ValueLayout) fd_set.$LAYOUT()
				.select(PathElement.groupElement(name)).select(PathElement.sequenceElement());
		fds_bits = valueLayout.arrayElementVarHandle();
	}

	private static boolean FD_ISSET(final fd_t fd, final MemorySegment fdset) {
		// it's a inline header function, therefore panama doesn't find it at runtime
//		return Linux.__darwin_fd_isset(fd.value(), fdset) != 0;

//		__darwin_fd_isset(int _fd, const struct fd_set *_p)
//		{
//			return _p->fds_bits[(unsigned long)_fd / __DARWIN_NFDBITS] & ((__int32_t)(((unsigned long)1) << ((unsigned long)_fd % __DARWIN_NFDBITS)));
//		}

		final long rawfd = fd.value & 0xffff_ffffL;
		final long element = (long) fds_bits.get(fdset, rawfd / __DARWIN_NFDBITS);
		final long bit = element & (1 << (rawfd % __DARWIN_NFDBITS));
		return bit != 0;
	}

	private static void FD_SET(final fd_t fd, final MemorySegment fdset) {
		// it's a inline header function, therefore panama doesn't find it at runtime
//		Linux.__darwin_fd_set(fd.value(), fdset);

//		__darwin_fd_set(int _fd, struct fd_set *const _p)
//		{
//			(_p->fds_bits[(unsigned long)_fd / __DARWIN_NFDBITS] |= ((__int32_t)(((unsigned long)1) << ((unsigned long)_fd % __DARWIN_NFDBITS))));
//		}

		final long rawfd = fd.value & 0xffff_ffffL;
		fds_bits.getAndBitwiseOr(fdset, rawfd / __DARWIN_NFDBITS, 1 << (rawfd % __DARWIN_NFDBITS));
	}

	private void perror(final String msg) {
		final String err = errnoMsg();
		logger.log(WARNING, "{0}: {1}", msg, err);
	}

	private static final MethodHandle errno;
	static {
		final String name = OS.current() == OS.Mac ? "__error" : "__errno_location";
		try {
			errno = MethodHandles.lookup().findStatic(Linux.class, name, MethodType.methodType(MemorySegment.class));
		}
		catch (NoSuchMethodException | IllegalAccessException e) {
			throw new AssertionError("should not reach here", e);
		}
	}

	private static int errno() {
		try {
			final var addr = (MemorySegment) errno.invokeExact();
			return addr.get(ValueLayout.JAVA_INT, 0);
		}
		catch (final Throwable e) {
			throw new AssertionError("should not reach here", e);
		}
	}

	private static void errno(final int error) {
		try {
			final var addr = (MemorySegment) errno.invokeExact();
			addr.set(ValueLayout.JAVA_INT, 0, error);
		}
		catch (final Throwable e) {
			throw new AssertionError("should not reach here", e);
		}
	}

	private static String errnoMsg() {
		return errnoMsg(errno());
	}

	private static String errnoMsg(final int error) {
		// use POSIX strerror (i.e., retrieve error string pointer and copy it over)
		// instead of XSI/GNU strerror_r functions
		// while strerror_r is thread-safe, it is not available on all systems
		//String str = strerror_r(error, msg, 100);
		final MemorySegment str = Linux.strerror(error);
		return str.getString(0);
	}
}
