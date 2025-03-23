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

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;

import serial.ffm.linux.Linux;

import serial.ffm.mac.Mac;

final class Unix {
	static final int PATH_MAX = OS.current() == OS.Linux ? Linux.PATH_MAX() : Mac.PATH_MAX();

	static final int EINTR       = OS.current() == OS.Linux ? Linux.EINTR() : Mac.EINTR();
	static final int EBADF       = OS.current() == OS.Linux ? Linux.EBADF() : Mac.EBADF();
	static final int ENOENT      = OS.current() == OS.Linux ? Linux.ENOENT() : Mac.ENOENT();
	static final int EACCES      = OS.current() == OS.Linux ? Linux.EACCES() : Mac.EACCES();
	static final int EPERM       = OS.current() == OS.Linux ? Linux.EPERM() : Mac.EPERM();
	static final int EBUSY       = OS.current() == OS.Linux ? Linux.EBUSY() : Mac.EBUSY();
	static final int EAGAIN      = OS.current() == OS.Linux ? Linux.EAGAIN() : Mac.EAGAIN();
	static final int EWOULDBLOCK = OS.current() == OS.Linux ? Linux.EWOULDBLOCK() : Mac.EWOULDBLOCK();

	static final int CREAD   = OS.current() == OS.Linux ? Linux.CREAD() : Mac.CREAD();
	static final int CLOCAL  = OS.current() == OS.Linux ? Linux.CLOCAL() : Mac.CLOCAL();
	static final int CSIZE   = OS.current() == OS.Linux ? Linux.CSIZE() : Mac.CSIZE();
	static final int CSTOPB  = OS.current() == OS.Linux ? Linux.CSTOPB() : Mac.CSTOPB();
	static final int CS5     = OS.current() == OS.Linux ? Linux.CS5() : Mac.CS5();
	static final int CS6     = OS.current() == OS.Linux ? Linux.CS6() : Mac.CS6();
	static final int CS7     = OS.current() == OS.Linux ? Linux.CS7() : Mac.CS7();
	static final int CS8     = OS.current() == OS.Linux ? Linux.CS8() : Mac.CS8();
	static final int PARENB  = OS.current() == OS.Linux ? Linux.PARENB() : Mac.PARENB();
	static final int PARODD  = OS.current() == OS.Linux ? Linux.PARODD() : Mac.PARODD();
	static final int INPCK   = OS.current() == OS.Linux ? Linux.INPCK() : Mac.INPCK();
	static final int VMIN    = OS.current() == OS.Linux ? Linux.VMIN() : Mac.VMIN();
	static final int VTIME   = OS.current() == OS.Linux ? Linux.VTIME() : Mac.VTIME();
	static final int TCSANOW = OS.current() == OS.Linux ? Linux.TCSANOW() : Mac.TCSANOW();

	static final int TIOCM_CTS = OS.current() == OS.Linux ? Linux.TIOCM_CTS() : Mac.TIOCM_CTS();
	static final int TIOCM_DSR = OS.current() == OS.Linux ? Linux.TIOCM_DSR() : Mac.TIOCM_DSR();
	static final int TIOCM_CAR = OS.current() == OS.Linux ? Linux.TIOCM_CAR() : Mac.TIOCM_CAR();
	static final int TIOCM_RNG = OS.current() == OS.Linux ? Linux.TIOCM_RNG() : Mac.TIOCM_RNG();
	static final int TIOCM_DTR = OS.current() == OS.Linux ? Linux.TIOCM_DTR() : Mac.TIOCM_DTR();

	static final int IXANY = OS.current() == OS.Linux ? Linux.IXANY() : Mac.IXANY();
	static final int IXON  = OS.current() == OS.Linux ? Linux.IXON() : Mac.IXON();
	static final int IXOFF = OS.current() == OS.Linux ? Linux.IXOFF() : Mac.IXOFF();

	static final int F_SETOWN  = OS.current() == OS.Linux ? Linux.F_SETOWN() : Mac.F_SETOWN();
	static final int F_SETFL   = OS.current() == OS.Linux ? Linux.F_SETFL() : Mac.F_SETFL();
	static final long FIONREAD = OS.current() == OS.Linux ? Linux.FIONREAD() : Mac.FIONREAD();

	static final int CRTSCTS = OS.current() == OS.Linux ? Linux.CRTSCTS() : Mac.CRTSCTS();

	static final int  TIOCEXCL  = OS.current() == OS.Linux ? Linux.TIOCEXCL() : Mac.TIOCEXCL();
	static final int  TIOCM_RTS = OS.current() == OS.Linux ? Linux.TIOCM_RTS() : Mac.TIOCM_RTS();
	static final long TIOCMGET  = OS.current() == OS.Linux ? Linux.TIOCMGET() : Mac.TIOCMGET();
	static final long TIOCMSET  = OS.current() == OS.Linux ? Linux.TIOCMSET() : Mac.TIOCMSET();

	static final int O_NONBLOCK = OS.current() == OS.Linux ? Linux.O_NONBLOCK() : Mac.O_NONBLOCK();
	static final int O_RDWR     = OS.current() == OS.Linux ? Linux.O_RDWR() : Mac.O_RDWR();
	static final int O_CREAT    = OS.current() == OS.Linux ? Linux.O_CREAT() : Mac.O_CREAT();
	static final int O_EXCL     = OS.current() == OS.Linux ? Linux.O_EXCL() : Mac.O_EXCL();
	static final int O_NOCTTY   = OS.current() == OS.Linux ? Linux.O_NOCTTY() : Mac.O_NOCTTY();

	static final int B0      = OS.current() == OS.Linux ? Linux.B0() : Mac.B0();
	static final int B50     = OS.current() == OS.Linux ? Linux.B50() : Mac.B50();
	static final int B75     = OS.current() == OS.Linux ? Linux.B75() : Mac.B75();
	static final int B110    = OS.current() == OS.Linux ? Linux.B110() : Mac.B110();
	static final int B134    = OS.current() == OS.Linux ? Linux.B134() : Mac.B134();
	static final int B150    = OS.current() == OS.Linux ? Linux.B150() : Mac.B150();
	static final int B200    = OS.current() == OS.Linux ? Linux.B200() : Mac.B200();
	static final int B300    = OS.current() == OS.Linux ? Linux.B300() : Mac.B300();
	static final int B600    = OS.current() == OS.Linux ? Linux.B600() : Mac.B600();
	static final int B1200   = OS.current() == OS.Linux ? Linux.B1200() : Mac.B1200();
	static final int B1800   = OS.current() == OS.Linux ? Linux.B1800() : Mac.B1800();
	static final int B2400   = OS.current() == OS.Linux ? Linux.B2400() : Mac.B2400();
	static final int B4800   = OS.current() == OS.Linux ? Linux.B4800() : Mac.B4800();
	static final int B9600   = OS.current() == OS.Linux ? Linux.B9600() : Mac.B9600();
	static final int B19200  = OS.current() == OS.Linux ? Linux.B19200() : Mac.B19200();
	static final int B38400  = OS.current() == OS.Linux ? Linux.B38400() : Mac.B38400();
	static final int B57600  = OS.current() == OS.Linux ? Linux.B57600() : Mac.B57600();
	static final int B115200 = OS.current() == OS.Linux ? Linux.B115200() : Mac.B115200();
	static final int B230400 = OS.current() == OS.Linux ? Linux.B230400() : Mac.B230400();


	private static final MemorySegment errno = OS.current() == OS.Linux ? Linux.__errno_location() : Mac.__error();

	static int errno() {
		return errno.get(ValueLayout.JAVA_INT, 0);
	}

	static void errno(final int error) {
		errno.set(ValueLayout.JAVA_INT, 0, error);
	}
}
