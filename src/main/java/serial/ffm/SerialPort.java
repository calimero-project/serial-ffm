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

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public interface SerialPort extends AutoCloseable {
	enum StopBits {
		One, Two;

		int value() { return ordinal() + 1; }
	}

	enum Parity {
		None, Odd, Even, Mark, Space;

		int value() { return ordinal(); }
	}

	enum FlowControl {
		None, CtsRts;

		int value() { return ordinal(); }
	}

	enum Status {
		Error, AvailableInput, Line
	}

	record Timeouts(int readInterval, int readTotalMultiplier, int readTotalConstant, int writeTotalMultiplier,
			int writeTotalConstant) {}


	static boolean portExists(final String portId) {
		return switch (OS.current()) {
			case Windows    -> new WinSerialPort().portExists(portId);
			case Linux, Mac -> new UnixSerialPort().portExists(portId);
			case Other      -> throw new RuntimeException("unsupported platform '" + OS.osName() + "'");
		};
	}

	static SerialPort open(final String portId) throws IOException {
		return switch (OS.current()) {
			case Windows    -> new WinSerialPort(portId);
			case Linux, Mac -> new UnixSerialPort(portId);
			case Other      -> throw new RuntimeException("unsupported platform '" + OS.osName() + "'");
		};
	}

	SerialPort baudRate(int baudrate) throws IOException;

	int baudRate() throws IOException;

	SerialPort dataBits(int databits) throws IOException;

	int dataBits() throws IOException;

	SerialPort stopBits(StopBits stopbits) throws IOException;

	StopBits stopBits() throws IOException;

	SerialPort parity(Parity parity) throws IOException;

	Parity parity() throws IOException;

	SerialPort flowControl(FlowControl flowControl) throws IOException;

	FlowControl flowControl() throws IOException;

	SerialPort timeouts(Timeouts timeouts) throws IOException;

	InputStream inputStream();

	OutputStream outputStream();

	void setEvents(int eventMask, boolean enable) throws IOException;

	int waitEvent() throws IOException;

	// line status bit field
	long LINE_CTS = 0x0010;    	// CTS (clear-to-send) signal is on
	long LINE_DSR = 0x0020;    	// DSR (data-set-ready) signal is on
	long LINE_RING = 0x0040;   	// ring indicator signal is on
	long LINE_DCD = 0x0080;    	// DCD (data carrier detect), TIOCM_CAR, aka (receive-line-signal-detect) signal is on

	long DTR = 0x0002;
	long RTS = 0x0004;

	int status(Status type) throws IOException;

	@Override
	void close();
}
