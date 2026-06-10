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
import static java.lang.System.Logger.Level.ERROR;
import static java.lang.System.Logger.Level.TRACE;
import static java.lang.System.Logger.Level.WARNING;
import static serial.ffm.Unix.errno;

import java.io.IOException;
import java.lang.foreign.Arena;
import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.Linker;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandles;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.atomic.AtomicInteger;

import serial.ffm.linux.Linux;
import serial.ffm.linux.serial_icounter_struct;
import serial.ffm.linux.serial_struct;
import serial.ffm.unix.dirent;
import serial.ffm.unix.flock;
import serial.ffm.unix.pollfd;
import serial.ffm.unix.stat;
import serial.ffm.unix.termios;


/**
 * Serial communication using Panama FFM to invoke Linux/macOS C functions. The implementation of this API
 * contains platform dependent code.
 */
final class UnixSerialPort extends ReadWritePort {

	// Any Character received
	static final int EVENT_RXCHAR = 0x0001;
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

	static final boolean supportsArbitraryBaudRates;
	static {
		supportsArbitraryBaudRates = switch (OS.current()) {
			case Linux -> {
				try {
					final var linker = Linker.nativeLinker();
					final var dh = linker.defaultLookup().find("gnu_get_libc_version")
							.map(sym -> linker.downcallHandle(sym, FunctionDescriptor.of(ValueLayout.ADDRESS)));
					if (dh.isEmpty())
						yield false;
					final var version = ((MemorySegment) dh.get().invoke()).reinterpret(10).getString(0);
					final var parts = version.split("\\.");
					if (parts.length > 1) {
						final int major = Integer.parseUnsignedInt(parts[0]);
						final int minor = Integer.parseUnsignedInt(parts[1]);
						// the GNU C library v2.42 switched to supporting arbitrary Baud rates
						if (major > 2 || (major == 2 && minor >= 42))
							yield true;
					}
				}
				catch (final Throwable t) {
					t.printStackTrace();
				}
				yield false;
			}
			case Mac, Windows, Other -> false;
		};
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

	private Timeouts timeouts = Timeouts.readInterval(Duration.ZERO);


	static Set<String> portIdentifiers() {
		return checkPortsDir("/dev");
	}

	private static Set<String> checkPortsDir(final String dir) {
		try (var arena = Arena.ofConfined()) {
			final var logger = System.getLogger(MethodHandles.lookup().lookupClass().getPackageName());

			var /*DIR*/ addr = Linux.opendir(arena.allocateFrom(dir));
			if (addr.equals(Linux.NULL())) {
				logger.log(WARNING, "cannot open ''{0}'': ", dir, errnoMsg());
				return Set.of();
			}

			final var ports = new TreeSet<String>();
			final var dirp = addr;
			while (!(addr = Linux.readdir(dirp)).equals(Linux.NULL())) {
				final MemorySegment entry = dirent.reinterpret(addr, arena, null);
				final String name = dirent.d_name(entry).getString(0);
				// ignore entries '.' and '..'
				if (name.charAt(0) == '.' && name.length() <= 2)
					continue;

				if (debug())
					logger.log(TRACE, LayoutFormatter.format(entry, dirent.layout()));

				final String filename = dir + "/" + name;
				logger.log(TRACE, "test {0}", filename);
				final var cfilename = arena.allocateFrom(filename);
				final var stbuf = stat.allocate(arena);
				if (Linux.stat(cfilename, stbuf) == -1) {
					logger.log(WARNING, "stat failed for ''{0}'': {1}", filename, errnoMsg());
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
				| UnixSerialPort.EVENT_RLSD | UnixSerialPort.EVENT_RXCHAR;

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
		final long cflag = termios.c_cflag(options);
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
		final long cflag = termios.c_cflag(options);
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
		long iflag = termios.c_iflag(options);
		// disable SW flow ctrl
		iflag &= ~(Unix.IXON | Unix.IXOFF | Unix.IXANY);
		termios.c_iflag(options, (int) iflag);
		long cflag = termios.c_cflag(options);
		cflag = switch (flowControl) {
			case CtsRts -> cflag | HW_FLOWCTL;
			case None -> cflag & ~HW_FLOWCTL;
		};
		termios.c_cflag(options, (int) cflag);
		tcsetattr(options);
	}

	@Override
	FlowControl flowControl(final Arena arena) throws IOException {
		final var options = tcgetattr(arena);
		final long cflag = termios.c_cflag(options);
		return genericFlowControl(cflag);
	}

	@Override
	void open(final Arena arena, final String portId) throws IOException {
		final AtomicInteger error = new AtomicInteger();
		fd = openPort(arena, portId, true, error);
		if (error.get() == Unix.EBUSY)
			throwIOException(Unix.EBUSY);

		if (debug()) {
// #if !defined TXSETIHOG
			final int TXSETIHOG = (('X' << 8) + 9);
			final int TXSETOHOG = (('X' << 8) + 10);

			// set rx and tx queues
			final var size = arena.allocate(Linux.C_INT, 200);
			// ??? how to set buffers
			if (Linux.ioctl.makeInvoker(Linux.C_POINTER).apply(fd.value(), TXSETIHOG, size) == -1)
				warnErrno("TXSETIHOG");
			if (Linux.ioctl.makeInvoker(Linux.C_POINTER).apply(fd.value(), TXSETOHOG, size) == -1)
				warnErrno("TXSETOHOG");

			final String out = "set queue tx %d rx %d".formatted(0, 0);
			logger.log(TRACE, out);
		}

		final var status = arena.allocate(Linux.C_INT);
		int ret = Linux.ioctl.makeInvoker(Linux.C_POINTER).apply(fd.value(), Unix.TIOCMGET, status);
		if (ret != -1) {
			status.set(Linux.C_INT, 0, status.get(Linux.C_INT, 0) | Unix.TIOCM_DTR);
			ret = Linux.ioctl.makeInvoker(Linux.C_POINTER).apply(fd.value(), Unix.TIOCMSET, status);
		}
		if (ret == -1) {
			final int err = errno();
			closePort();
			throwIOException(err);
		}
	}

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
				if (Linux.ioctl.makeInvoker(Linux.C_POINTER).apply(fd.value(), TIOCGSERIAL, info) == 0) {
					if (serial_struct.type(info) != PORT_UNKNOWN)
						valid = true;
					else if (hasDevSerialLink(arena, portId))
						valid = true;
				}
				else
					logger.log(TRACE, "check port type: {0}", errnoMsg());
			}
			else {
				final var status = arena.allocate(Linux.C_INT);
				final int ret = Linux.ioctl.makeInvoker(Linux.C_POINTER).apply(fd.value(), Unix.TIOCMGET, status);
				valid = ret == 0;
				logger.log(TRACE, "check port type: {0}", errnoMsg());
			}

			closePort(fd.value);
			return valid;
		}
		catch (final IOException e) {
			if (error.get() == Unix.ENOENT)
				return false;
			if (error.get() == Unix.EBUSY || error.get() == Unix.EPERM || error.get() == Unix.EACCES)
				return true;
			logger.log(TRACE, "{0}", e.getMessage());
			return false;
		}
	}

	private static String createLockName(final String dir, final String namePrefix, final String name) {
		return dir + namePrefix + name;
	}

	boolean tryLink(final MemorySegment forName, final MemorySegment linkName) {
		if (Linux.link(forName, linkName) == -1) {
			logger.log(TRACE, "failed to create link {0} ({1}), try stat {2}", linkName.getString(0), errnoMsg(),
					forName.getString(0));
			try (var arena = Arena.ofConfined()) {
				// we are nevertheless successful if lock count states 2 locks
				final var seg = stat.allocate(arena);
				if (Linux.stat(forName, seg) == -1) {
					logger.log(TRACE, "stat failed for {0}: {1}", forName, errnoMsg());
					return false;
				}
				if (debug())
					logger.log(TRACE, LayoutFormatter.format(seg, stat.layout()));
				if (stat.st_nlink(seg) != 2)
					return false;
			}
		}
		Linux.unlink(forName);
		return true;
	}

	private static int readPid(final Path filename) throws IOException {
		try {
			final String s = Files.readString(filename).trim();
			return Integer.parseInt(s);
		}
		catch (final NumberFormatException e) {
			throw new IOException("invalid pid in lock file '" + filename + "'", e);
		}
	}

	// try to create PID..pid and write our pid into it
	// create LCK..name as link to pid file
	// all files are located in /var/lock/
	void ensureLock(final String port) throws IOException {
		String p = port;
		if (p.startsWith("/dev/") && p.length() > 5)
			p = p.substring(5);
		final long myPid = ProcessHandle.current().pid();
		final String pidFile = createLockName(lockDir, pidPrefix, "" + myPid);
		try (var arena = Arena.ofConfined()) {
			final var mpidFile = arena.allocateFrom(pidFile);
			final fd_t fd = fd_t.of(Linux.open.makeInvoker(Linux.C_INT).apply(mpidFile, Unix.O_RDWR | Unix.O_EXCL | Unix.O_CREAT, 0644));
			if (fd.equals(fd_t.Invalid)) {
				if (errno() == Unix.EEXIST)
					throw new IOException("port '" + port + "' is already locked");
				throw new IOException("cannot create lock file '" + pidFile + "': " + errnoMsg());
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
				return;
			}
			final int pid;
			try {
				// lock file exists, read its pid
				// if we cannot read in the pid, we cannot lock
				pid = readPid(Path.of(lckFile));
				// we might already have locked the port
				if (pid == myPid) {
					lockedPort = port;
					return;
				}
				// check the read pid, if stale, try to remove lock file
				else if (Linux.kill(pid, 0) == -1 && errno() != Unix.EPERM) {
					Linux.unlink(mlckFile);
					if (tryLink(mpidFile, mlckFile)) {
						lockedPort = port;
						return;
					}
				}
			}
			catch (final IOException e) {
				throw new IOException("cannot acquire lock for port '" + port + "': " + e.getMessage());
			}
			finally {
				Linux.unlink(mpidFile);
			}
			throw new IOException("cannot acquire lock for port '" + port + "': lock held by process " + pid);
		}
	}

	void releaseLock() {
		if (lockedPort.isEmpty())
			return;
		final int idx = lockedPort.lastIndexOf('/');
		final String name = idx == -1 ? lockedPort : lockedPort.substring(idx + 1);
		final var lockFile = createLockName(lockDir, lckPrefix, name);
		lockedPort = "";
		final var path = Path.of(lockFile);
		if (Files.notExists(path))
			return;
		try {
			final int pid = readPid(path);
			if (pid == Linux.getpid()) {
				logger.log(TRACE, "release lock ''{0}''", lockFile);
				Files.delete(path);
			}
		}
		catch (final IOException e) {
			logger.log(DEBUG, "cannot release lock ''{0}'': {1}", lockFile, e.getMessage());
		}
	}

	private boolean setPortDefaults(final int fd) {
		try (var arena = Arena.ofConfined()) {
			final var options = termios.allocate(arena);
			if (Linux.tcgetattr(fd, options) == -1) {
				logger.log(WARNING, "setting port defaults, tcgetattr: {0}", errnoMsg());
				return false;
			}
			Linux.cfsetispeed(options, Unix.B9600);
			Linux.cfsetospeed(options, Unix.B9600);
			long cflag = termios.c_cflag(options);
			cflag |= Unix.CREAD | Unix.CLOCAL;
			cflag &= ~Unix.CSIZE;
			cflag |= Unix.CS8;
			termios.c_cflag(options, (int) cflag);
			termios.c_iflag(options, Unix.INPCK);
			termios.c_lflag(options, 0);
			termios.c_oflag(options, 0);
			// configure polling read
			termios.c_cc(options).set(Linux.C_CHAR, Unix.VMIN, (byte) 0);
			termios.c_cc(options).set(Linux.C_CHAR, Unix.VTIME, (byte) 0);
			if (Linux.tcsetattr(fd, Unix.TCSANOW, options) == -1) {
				logger.log(WARNING, "setting port defaults, tcsetattr: {0}", errnoMsg());
				return false;
			}
			Linux.fcntl.makeInvoker(Linux.C_INT).apply(fd, Unix.F_SETOWN, Linux.getpid());
		}
		return true;
	}

	private fd_t openPort(final Arena arena, final String portId, final boolean configurePort,
			final AtomicInteger lastError) throws IOException {
		fd_t fd;
//		errno(0);
		int error = 0;

		ensureLock(portId);
		final var port = arena.allocateFrom(portId);
		do {
			// we set the port exclusive below, not here
			fd = fd_t.of(Linux.open.makeInvoker().apply(port, /*O_EXCL |*/Unix.O_RDWR | Unix.O_NOCTTY | Unix.O_NONBLOCK));
			if (!fd.equals(fd_t.Invalid))
				break;
		}
		while (errno() == Unix.EINTR);

		// check if someone else has opened the port
		if (fd.equals(fd_t.Invalid)) {
			lastError.set(errno());
			releaseLock();
			throw new IOException("failed to open port '" + portId + "': " + errnoMsg());
		}
		if (errno() == Unix.EBUSY) {
			logger.log(TRACE, "busy {0}", fd);
			lastError.set(errno());
			releaseLock();
			return fd;
		}

		// we continue if we are not able to set exclusive mode
		if (Linux.ioctl.makeInvoker().apply(fd.value(), Unix.TIOCEXCL) == -1) {
			error = errno();
			warnErrno("set exclusive");
		}

		logger.log(TRACE, "acquire fcntl lock");
		final var lock = flock.allocate(arena);
		flock.l_type(lock, Unix.F_WRLCK);
		flock.l_whence(lock, Unix.SEEK_SET);
		if (Linux.fcntl.makeInvoker(Linux.C_POINTER).apply(fd.value(), Unix.F_SETLK, lock) == -1) {
			error = errno();
			warnErrno("fcntl lock");
			closePort(fd.value());
			fd = fd_t.Invalid;
		}
		else if (configurePort) {
			// we continue if we are not able to save old port settings
			// TODO actually restore them on close
			final var saved = termios.allocate(arena);
			if (Linux.tcgetattr(fd.value(), saved) == -1) {
				error = errno();
				warnErrno("save old port settings");
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
			logger.log(WARNING, "failed to enumerate ''{0}'': ", dir, errnoMsg());
			return false;
		}

		final var dirp = addr;
		while (!(addr = Linux.readdir(dirp)).equals(Linux.NULL())) {
			final var entry = dirent.reinterpret(addr, arena, null);
			final String name = dirent.d_name(entry).getString(0);
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

			final var cresolved = arena.allocate(Unix.PATH_MAX);
			if (Linux.realpath(cfilename, cresolved) == Linux.NULL())
				logger.log(WARNING, "realpath failed for {0}: {1}", filename, errnoMsg());
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
		final int closeFd = fd.value();
		fd = fd_t.Invalid;
		return closePort(closeFd);
	}

	private boolean closePort(final int fd) throws IOException {
		if (fd == fd_t.Invalid.value())
			return true;
		logger.log(TRACE, "close fd {0}", fd);
		boolean closed;
		do {
			closed = Linux.close(fd) == 0;
			if (closed)
				break;
		}
		while (errno() == Unix.EINTR);

		releaseLock();

		return closed;
	}

	private void tcsetattr(final MemorySegment options) throws IOException {
		if (Linux.tcsetattr(fd.value(), Unix.TCSANOW, options) == -1)
			throwIOException("tcsetattr", errno());
	}

	private MemorySegment tcgetattr(final Arena arena) throws IOException {
		final var options = termios.allocate(arena);
		if (Linux.tcgetattr(fd.value(), options) == -1)
			throwIOException("tcgetattr", errno());
		return options;
	}

	// CSTOPB set corresponds to 2 stop bits, default is 1 stop bit
	private static void setTermiosStopBits(final MemorySegment cflags, /*uint8_t*/ final StopBits stopbits) {
		long flags = termios.c_cflag(cflags);

		flags &= ~Unix.CSTOPB;
		switch (stopbits) {
			case Two:
				flags |= Unix.CSTOPB;
				break;
//		case STOPBITS_15: /* does not exist */
//			break;
			case One:
				break;
			default:
				break;
		}
		termios.c_cflag(cflags, (int) flags);
	}

	private static StopBits genericStopBits(final MemorySegment cflags) {
		final long flags = termios.c_cflag(cflags);
		final int stopbits = (int) (flags & Unix.CSTOPB);
		if (stopbits == Unix.CSTOPB)
			return StopBits.Two;
		return StopBits.One;
	}

	// non-existing on macOS
	private static final int CMSPAR = 010000000000;

	private static boolean markSpaceParitySupport() {
		return OS.current() == OS.Linux;
	}

	private void setTermiosParity(final MemorySegment flags, final Parity parity) {
		long cflags = termios.c_cflag(flags);
		cflags &= ~(Unix.PARENB | Unix.PARODD | CMSPAR);

		switch (parity) {
			case None:
				break;
			case Even:
				cflags |= Unix.PARENB;
				break;
			case Odd:
				cflags |= Unix.PARENB | Unix.PARODD;
				break;
			case Mark:
				if ((cflags & Unix.CS7) == Unix.CS7) {
					logger.log(TRACE, "setting mark parity for 7M1");
					// for mark parity use 7 data bits, 2 stop bits
					cflags &= ~Unix.CSIZE;
					cflags |= Unix.CSTOPB | Unix.CS7;
				}
				else if ((cflags & Unix.CS8) == Unix.CS8) {
					logger.log(TRACE, "setting mark parity for 8M1");
					if (markSpaceParitySupport())
						cflags |= CMSPAR | Unix.PARENB | Unix.PARODD;
					else {
						// emulate with 8N2
						cflags |= Unix.CSTOPB;
					}
				}
				break;
			case Space:
				if ((cflags & Unix.CS8) == Unix.CS8) {
					if (markSpaceParitySupport()) {
						cflags |= CMSPAR | Unix.PARENB;
						cflags &= ~Unix.PARODD;
					}
					else {
						// NYI emulate 8S1
					}
				}
		}
		termios.c_cflag(flags, (int) cflags);
	}

	private static Parity genericParity(final long cflags) {
		if (markSpaceParitySupport()) {
			if ((cflags & Unix.PARENB) == Unix.PARENB && (cflags & Unix.PARODD) == Unix.PARODD
					&& (cflags & CMSPAR) == CMSPAR)
				return Parity.Mark;
			if ((cflags & Unix.PARENB) == Unix.PARENB && (cflags & CMSPAR) == CMSPAR)
				return Parity.Space;
		}

		if ((cflags & Unix.PARENB) != 0 && (cflags & Unix.PARODD) != 0)
			return Parity.Odd;
		if ((cflags & Unix.PARENB) != 0)
			return Parity.Even;

		return Parity.None;
	}

	private static int termiosBaudrate(final /*uint*/ int baudrate) {
		if (supportsArbitraryBaudRates)
			return baudrate;
		return switch (baudrate) {
			case 0 -> Unix.B0;
			case 50 -> Unix.B50;
			case 75 -> Unix.B75;
			case 110 -> Unix.B110;
			case 134 -> Unix.B134;
			case 150 -> Unix.B150;
			case 200 -> Unix.B200;
			case 300 -> Unix.B300;
			case 600 -> Unix.B600;
			case 1200 -> Unix.B1200;
			case 1800 -> Unix.B1800;
			case 2400 -> Unix.B2400;
			case 4800 -> Unix.B4800;
			case 9600 -> Unix.B9600;
//#ifdef B14400
// undefined on Linux
//		case 14400:
//			return Unix.B14400)
//#endif /* B14400 */
			case 19200 -> Unix.B19200;
//#ifdef B28800
// undefined on Linux
//		case 28800:
//			return Unix.B28800)
//#endif /* B28800 */
			case 38400 -> Unix.B38400;
//#ifdef B57600 // MacOS X does not define this Baud rate
			case 57600 -> Unix.B57600;
//#endif // B57600
//#ifdef B115200
			case 115200 -> Unix.B115200;
//#endif /*  B115200 */
//#ifdef B230400
			case 230400 -> Unix.B230400;
//#endif /* B230400 */
//#ifdef B460800
//    case 460800:
//        return Unix.B460800)
//#endif /* B460800 */
//#ifdef B500000
//    case 500000:
//        return Unix.B500000)
//#endif /* B500000 */
//#ifdef B576000
//    case 576000:
//        return Unix.B576000)
//#endif /* B57600 */
//#ifdef B921600
//    case 921600:
//        return Unix.B921600)
//#endif /* B921600 */
//#ifdef B1000000
//    case 1000000:
//        return Unix.B1000000)
//#endif /* B1000000 */
//#ifdef B1152000
//    case 1152000:
//        return Unix.B1152000)
//#endif /* B1152000 */
//#ifdef B1500000
//    case 1500000:
//        return Unix.B1500000)
//#endif /* B1500000 */
//#ifdef B2000000
//    case 2000000:
//        return Unix.B2000000)
//#endif /* B2000000 */
//#ifdef B2500000
//    case 2500000:
//        return Unix.B2500000)
//#endif /* B2500000 */
//#ifdef B3000000
//    case 3000000:
//        return Unix.B3000000)
//#endif /* B3000000 */
//#ifdef B3500000
//    case 3500000:
//        return Unix.B3500000)
//#endif /* B3500000 */
//#ifdef B4000000
//    case 4000000:
//        return Unix.B4000000)
//#endif /* B4000000 */
			default -> -1;
		};
	}

	private static int genericBaudrate(final long baudrate) {
		if (supportsArbitraryBaudRates)
			return (int) baudrate;
		if (baudrate == Unix.B0)
			return 0;
		else if (baudrate == Unix.B50)
			return 50;
		else if (baudrate == Unix.B75)
			return 75;
		else if (baudrate == Unix.B110)
			return 110;
		else if (baudrate == Unix.B134)
			return 134;
		else if (baudrate == Unix.B150)
			return 150;
		else if (baudrate == Unix.B200)
			return 200;
		else if (baudrate == Unix.B300)
			return 300;
		else if (baudrate == Unix.B600)
			return 600;
		else if (baudrate == Unix.B1200)
			return 1200;
		else if (baudrate == Unix.B1800)
			return 1800;
		else if (baudrate == Unix.B2400)
			return 2400;
		else if (baudrate == Unix.B4800)
			return 4800;
		else if (baudrate == Unix.B9600)
			return 9600;
//		else if (baudrate == Unix.B14400)
//			return 14400;
		else if (baudrate == Unix.B19200)
			return 19200;
//		else if (baudrate == Unix.B28800)
//			return 28800;
		else if (baudrate == Unix.B38400)
			return 38400;
		else if (baudrate == Unix.B57600)
			return 57600;
		else if (baudrate == Unix.B115200)
			return 115200;
		else if (baudrate == Unix.B230400)
			return 230400;
		else
			return -1;

//#endif /* B230400 */
//#ifdef B460800
//    case Unix.B460800():
//        return 460800;
//#endif /* B460800 */
//#ifdef B500000
//    case Unix.B500000():
//        return 500000;
//#endif /* B500000 */
//#ifdef B576000
//    case Unix.B576000():
//        return 576000;
//#endif /* B576000 */
//#ifdef B921600
//    case Unix.B921600():
//        return 921600;
//#endif /* B921600 */
//#ifdef B1000000
//    case Unix.B1000000():
//        return 1000000;
//#endif /* B1000000 */
//#ifdef B1152000
//    case Unix.B1152000():
//        return 1152000;
//#endif /* B1152000 */
//#ifdef B1500000
//    case Unix.B1500000():
//        return 1500000;
//#endif /* B1500000 */
//#ifdef B2000000
//    case Unix.B2000000():
//        return 2000000;
//#endif /* B2000000 */
//#ifdef B2500000
//    case Unix.B2500000():
//        return 2500000;
//#endif /* B2500000 */
//#ifdef B3000000
//    case Unix.B3000000():
//        return 3000000;
//#endif /* B3000000 */
//#ifdef B3500000
//    case Unix.B3500000():
//        return 3500000;
//#endif /* B3500000 */
//#ifdef B4000000
//    case Unix.B4000000():
//        return 4000000;
//#endif /* B4000000 */
//    default:
//        return -1;
//    }
	}

	private static void setTermiosDataBits(final MemorySegment flags, /*uint8_t*/ final int databits) {
		long cflags = termios.c_cflag(flags);
		cflags &= ~Unix.CSIZE;
		cflags |= switch (databits) {
			case 5 -> Unix.CS5;
			case 6 -> Unix.CS6;
			case 7 -> Unix.CS7;
			default -> Unix.CS8;
		};
		termios.c_cflag(flags, (int) cflags);
	}

	private static /*uint8_t*/ int genericDataBits(final long cflags) {
		final long i = cflags & Unix.CSIZE;
		if (i == Unix.CS5)
			return 5;
		else if (i == Unix.CS6)
			return 6;
		else if (i == Unix.CS7)
			return 7;
		else if (i == Unix.CS8)
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

		HW_FLOWCTL = Unix.CRTSCTS;
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
		while (ret != 0 && errno() == Unix.EINTR);
		return ret == 0;
	}

	@Override
	int readBytes(final Arena arena, final MemorySegment bytes) throws IOException {
		lock.lock();
		try {
			final long r = read(arena, fd(), bytes);
			if (r == -2)
				throw new IOException("interrupted");
			if (r == -1)
				throwIOException(errno());
			return (int) r;
		}
		finally {
			lock.unlock();
		}
	}

	private fd_t fd() throws IOException {
		final var lfd = fd;
		if (lfd.equals(fd_t.Invalid))
			throwIOException(Unix.EBADF);
		return lfd;
	}

	private long read(final Arena arena, final fd_t fd, final MemorySegment buffer) {
		if (Thread.currentThread().isInterrupted())
			return -2;

		long offset = 0;
		long remaining = buffer.byteSize();

		final long readInterval = timeouts.readInterval().toMillis();
		final long totalTimeout = timeouts.readTotalConstant().toMillis()
				+ timeouts.readTotalMultiplier().toMillis() * remaining;

		final var pfd = pollfd.allocate(arena);
		pollfd.fd(pfd, fd.value());
		pollfd.events(pfd, (short) Unix.POLLIN);

		long pollTimeoutCount = 0;
		final long start = System.nanoTime();
		while (remaining > 0) {
			final long elapsed = (System.nanoTime() - start) / 1_000_000;
			long pollTimeout = 0;

			if (readInterval > 0) {
				// check whether we reached interval timeout for next byte
				final long remainingInterval = readInterval - elapsed;
				if (remainingInterval < 0)
					break;
				pollTimeout = remainingInterval;
			}
			if (totalTimeout > 0) {
				// check whether we reached total timeout for the requested bytes
				final long remainingTotal = totalTimeout - elapsed;
				if (remainingTotal <= 0)
					break;
				// if both interval and total timeout are configured, we use whatever times out first
				pollTimeout = pollTimeout > 0 ? Math.min(pollTimeout, remainingTotal) : remainingTotal;
			}

			pollTimeout = pollTimeout > 0 ? Math.min(pollTimeout, wakeupInterval) : wakeupInterval;
			final boolean nonblocking = false;
			if (!nonblocking) {
				final int n;
				lock.unlock();
				try {
					n = Unix.poll(pfd, 1, Math.toIntExact(pollTimeout));
				}
				finally {
					lock.lock();
				}

				if (Thread.currentThread().isInterrupted())
					return -2;

				if (n == -1) {
					warnErrno("poll failed");
					if (errno() == Unix.EINTR)
						continue;
					break; // e.g., EBADF
				}
				else if (n == 0) { // poll timeout
					if (debug()) {
						++pollTimeoutCount;
						// don't flood the log, so apply rate-limited trace logging of poll timeouts
						final int pollTimeoutLogThrottle = 100; // ms
						if (pollTimeoutCount * pollTimeout >= pollTimeoutLogThrottle) {
							logger.log(TRACE, "read poll timeout ({0} timeouts)", pollTimeoutCount);
							pollTimeoutCount = 0;
						}
					}
					continue;
				}
			}

			final int events = pollfd.revents(pfd);
			final boolean pollin = (events & Unix.POLLIN) != 0;
			final boolean pollhup = (events & Unix.POLLHUP) != 0;
			final boolean pollerr = (events & Unix.POLLERR) != 0;
			final boolean pollnval = (events & Unix.POLLNVAL) != 0;
			if (events != 0) {
				logger.log(TRACE, "poll returned events: {0}{1}{2}{3}",
						pollin ? "POLLIN "  : "",
						pollhup ? "POLLHUP " : "",
						pollerr ? "POLLERR " : "",
						pollnval ? "POLLNVAL" : "");
			}

			if (pollnval) { // fd not open
				logger.log(WARNING, "poll failed: fd not open");
				break;
			}
			else if (pollin || pollhup || pollerr) {
				// read received bytes from stream into buffer
				// on hang up or error, try to read remaining data (if any)

				// get number of bytes that are immediately available for reading:
				// if 0, this indicates other errors, e.g., disconnected usb adapter
				final var nread = arena.allocateFrom(ValueLayout.JAVA_LONG, 0);
				Linux.ioctl.makeInvoker(Linux.C_POINTER).apply(fd.value(), Unix.FIONREAD, nread);
				if (nread.get(ValueLayout.JAVA_LONG, 0) == 0)
					return -1;

				final long ret = Linux.read(fd.value(), MemorySegment.ofAddress(buffer.address() + offset), remaining);
				if (ret == -1) {
					// retry if error is EAGAIN/EWOULDBLOCK or EINTR, otherwise bail out
					final int errno = errno();
					if (errno == Unix.EWOULDBLOCK || errno == Unix.EAGAIN || errno == Unix.EINTR)
						continue;
					warnErrno("read");
					break;
				}
				else {
					offset += ret;
					remaining -= ret;
				}
			}
		}

		return offset;
	}

	@Override
	int writeBytes(final Arena arena, final MemorySegment bytes) throws IOException {
		lock.lock();
		try {
			final long written = Linux.write(fd.value(), bytes, bytes.byteSize());
			if (written < 0)
				throwIOException(errno());
			return (int) written;
		}
		finally {
			lock.unlock();
		}
	}

	@Override
	void doDrain() throws IOException {
		if (!drain(fd))
			throwIOException(errno());
	}

	private /*uint*/ long isInputWaiting(final Arena arena) throws IOException {
		Linux.fcntl.makeInvoker(Linux.C_INT).apply(fd.value(), Unix.F_SETFL, Unix.O_NONBLOCK);
		/*uint*/ final var bytes = arena.allocate(ValueLayout.JAVA_INT);
		if (Linux.ioctl.makeInvoker(Linux.C_POINTER).apply(fd.value(), Unix.FIONREAD, bytes) == -1) {

			//#if defined FIORDCHK
			if (definedFIORDCHK()) {
				//        final int rdchk = Linux.ioctl(fd, Unix.FIORDCHK, 0);
				//        if (rdchk > -1)
				//            bytes = rdchk;
				//        else
				//        	throw newException(errno());
			} // FIORDCHK
			else
				throwIOException(errno());
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
		if ((eventMask & EVENT_TXEMPTY) != 0)
			events |= 0;

		if ((eventMask & EVENT_CTS) != 0)
			events |= Unix.TIOCM_CTS;
		if ((eventMask & EVENT_DSR) != 0)
			events |= Unix.TIOCM_DSR;
		if ((eventMask & EVENT_RLSD) != 0)
			events |= Unix.TIOCM_CAR;
		// XXX assign those
		if ((eventMask & EVENT_BREAK) != 0)
			events |= 0;
		if ((eventMask & EVENT_ERR) != 0)
			events |= 0;
		if ((eventMask & EVENT_RING) != 0)
			events |= Unix.TIOCM_RNG;

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
		if (serial_icounter_struct.rx(icount) != rx) {
			rx = serial_icounter_struct.rx(icount);
			events |= EVENT_RXCHAR;
		}
		if (serial_icounter_struct.cts(icount) != cts) {
			cts = serial_icounter_struct.cts(icount);
			events |= EVENT_CTS;
		}
		if (serial_icounter_struct.dsr(icount) != dsr) {
			dsr = serial_icounter_struct.dsr(icount);
			events |= EVENT_DSR;
		}
		if (serial_icounter_struct.dcd(icount) != dcd) {
			dcd = serial_icounter_struct.dcd(icount);
			events |= EVENT_RLSD;
		}
		if (serial_icounter_struct.brk(icount) != brk) {
			brk = serial_icounter_struct.brk(icount);
			events |= EVENT_BREAK;
		}
		if (serial_icounter_struct.frame(icount) != frame
				|| serial_icounter_struct.buf_overrun(icount) != buf_overrun
				|| serial_icounter_struct.parity(icount) != parity) {
			frame = serial_icounter_struct.frame(icount);
			buf_overrun = serial_icounter_struct.buf_overrun(icount);
			parity = serial_icounter_struct.parity(icount);
			events |= EVENT_ERR;
		}
		if (serial_icounter_struct.rng(icount) != rng) {
			rng = serial_icounter_struct.rng(icount);
			events |= EVENT_RING;
		}
		return events;
	}

	// not defined on macOS
	private static final int TIOCMIWAIT = 0x545C;
	private static final int TIOCGICOUNT = 0x545D;

	private MemorySegment queryInterruptCounters() throws IOException {
		logger.log(TRACE, "queryInterruptCounters");
		try (var arena = Arena.ofConfined()) {
			final var icount = serial_icounter_struct.allocate(arena);
			int ret;
			do {
				ret = Linux.ioctl.makeInvoker(Linux.C_POINTER).apply(fd.value(), TIOCGICOUNT, icount);
			}
			while (ret == -1 && errno() == Unix.EINTR);
			if (ret == -1)
				throwIOException(errno());
			return icount;
		}
	}

	private static final int TIOCSERGETLSR = 0x5459;	/* Get line status register */
	private static final int TIOCSER_TEMT = 0x01;		/* Transmitter physically empty */

	private boolean lsr() {
		try (var arena = Arena.ofConfined()) {
			final var lsr = arena.allocate(1);
			if (Linux.ioctl.makeInvoker(Linux.C_POINTER).apply(fd.value(), TIOCSERGETLSR, lsr) == -1)
				return false;
			// output buffer empty?
			return (lsr.get(ValueLayout.JAVA_BYTE, 0) & TIOCSER_TEMT) != 0;
		}
	}

	@Override
	public int waitEvent() throws IOException {
		logger.log(TRACE, "enter wait event");
		try {
			if (OS.current() == OS.Mac || OS.current() == OS.Linux) {
				logger.log(DEBUG, "use polledWaitEvent, waitEvent not working yet");
				return polledWaitEvent();
			}

			final int mask = ioctlEventMask;
			int ret;
			do {
				ret = Linux.ioctl.makeInvoker(Linux.C_INT).apply(fd.value(), TIOCMIWAIT, mask);
				if (ret == -1 && errno() == Unix.EINTR)
					logger.log(TRACE, "waitEvent interrupted");
			}
			while (ret == -1 && errno() == Unix.EINTR);
			if (ret == -1)
				throwIOException(errno());

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
			final var pfd = pollfd.allocate(arena);
			pollfd.fd(pfd, fd.value());
			pollfd.events(pfd, (short) 0);

			while (true) {
				int ret;
				do {
					ret = Unix.poll(pfd, 1, (int) eventPollInterval.toMillis());
				} while (ret == -1 && errno() == Unix.EINTR);

				if (isClosed())
					throwIOException(Unix.EBADF);

				if ((currentEventMask & (EVENT_CTS | EVENT_DSR | EVENT_RING | EVENT_RLSD)) != 0) {
					final int lineStatus = status(arena, Status.Line);
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
					final int errorStatus = status(arena, Status.Error);
					if (polledErrorStatus != errorStatus) {
						final int events = 0;

						polledErrorStatus = errorStatus;
						return events;
					}
				}

				if ((currentEventMask & EVENT_RXCHAR) != 0) {
					final int availStatus = status(arena, Status.AvailableInput);
					if (polledAvailableStatus != availStatus) {
						polledAvailableStatus = availStatus;
						return EVENT_RXCHAR;
					}
				}
			}
		}
	}

	@Override
	int status(final Arena arena, final Status type) throws IOException {
		// clear communication error and get device status
		return (int) /*uint*/ switch (type) {
			case Line -> {
				/*uint*/ final var mstatus = arena.allocate(ValueLayout.JAVA_INT);
				final int ret = Linux.ioctl.makeInvoker(Linux.C_POINTER).apply(fd.value(), Unix.TIOCMGET, mstatus);
				if (ret == -1)
					throwIOException(errno());

				long v = 0;
				final long status = mstatus.get(ValueLayout.JAVA_INT, 0);
				if ((status & Unix.TIOCM_DTR) != 0)
					v |= DTR;
				if ((status & Unix.TIOCM_RTS) != 0)
					v |= RTS;
				if ((status & Unix.TIOCM_CTS) != 0)
					v |= LINE_CTS;
				if ((status & Unix.TIOCM_DSR) != 0)
					v |= LINE_DSR;
				if ((status & Unix.TIOCM_RNG) != 0)
					v |= LINE_RING;
				if ((status & Unix.TIOCM_CAR) != 0)
					v |= LINE_DCD;
				yield v;
			}
			case AvailableInput -> isInputWaiting(arena);
			case Error -> {
				if (isClosed())
					throwIOException(Unix.EBADF);
				// XXX implement error status
				yield 0;
				// 0x0010 : detected a break condition
				// 0x0008 : detected a framing error
				// 0x0002 : character-buffer overrun
				// 0x0001 : input buffer overflow (no room in the input buffer, or character received after EOF)
				// 0x0004 : detected a parity error
			}
		};
	}

	// for readIntervalTimeout, so it has same behavior as in windows when set to this specific value
	private static final /*uint*/ int MAXDWORD = 0xffff_ffff;

	@Override
	void timeouts(final Arena arena, final Timeouts timeouts) {
		this.timeouts = timeouts;
	}

	@Override
	Timeouts timeouts(final Arena arena) {
		return timeouts;
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
		if (isSet(eventMask, EVENT_BREAK))
			logger.log(TRACE, "EVENT_BREAK");
		if (isSet(eventMask, EVENT_DTR))
			logger.log(TRACE, "EVENT_DTR");
		if (isSet(eventMask, EVENT_RTS))
			logger.log(TRACE, "EVENT_RTS");
	}

	private void throwIOException(final int errno) throws IOException {
		if (errno == Unix.EBADF)
			throw new PortClosedException(portName());
		throw new IOException(errnoMsg(errno) + " (" + errno + ")");
	}

	private void throwIOException(final String msg, final int errno) throws IOException {
		if (errno == Unix.EBADF)
			throw new PortClosedException(portName(), msg);
		throw new IOException(msg + ": " + errnoMsg(errno) + " (" + errno + ")");
	}

	private void warnErrno(final String msg) {
		final String err = errnoMsg();
		logger.log(WARNING, "{0}: {1}", msg, err);
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
