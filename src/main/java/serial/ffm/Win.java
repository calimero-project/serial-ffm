// MIT License
//
// Copyright (c) 2026, 2026 B. Malinowsky
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

import java.lang.foreign.Linker;
import java.lang.foreign.MemoryLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.StructLayout;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.util.Arrays;
import java.util.stream.Collectors;

import serial.ffm.win.Windows;

/**
 * Declaration of Windows API functions which capture the state of GetLastError().
 */
final class Win {
	static final StructLayout captureStateLayout = Linker.Option.captureStateLayout();
	private static final VarHandle getLastError = captureStateLayout.varHandle(MemoryLayout.PathElement.groupElement("GetLastError"));

	static int getLastError(final MemorySegment capturedState) { return (int) getLastError.get(capturedState, 0); }

	private static final Linker linker = Linker.nativeLinker();
	private static final Linker.Option ccs = Linker.Option.captureCallState("GetLastError");

	private static final MethodHandle CreateFileA = linker.downcallHandle(Windows.CreateFileA$address(), Windows.CreateFileA$descriptor(), ccs);
	private static final MethodHandle FlushFileBuffers = linker.downcallHandle(Windows.FlushFileBuffers$address(), Windows.FlushFileBuffers$descriptor(), ccs);
	private static final MethodHandle GetFileType = linker.downcallHandle(Windows.GetFileType$address(), Windows.GetFileType$descriptor(), ccs);
	private static final MethodHandle ReadFile = linker.downcallHandle(Windows.ReadFile$address(), Windows.ReadFile$descriptor(), ccs);
	private static final MethodHandle WriteFile = linker.downcallHandle(Windows.WriteFile$address(), Windows.WriteFile$descriptor(), ccs);
	private static final MethodHandle CloseHandle = linker.downcallHandle(Windows.CloseHandle$address(), Windows.CloseHandle$descriptor(), ccs);
	private static final MethodHandle GetOverlappedResultEx = linker.downcallHandle(Windows.GetOverlappedResultEx$address(), Windows.GetOverlappedResultEx$descriptor(), ccs);
	private static final MethodHandle CreateEventA = linker.downcallHandle(Windows.CreateEventA$address(), Windows.CreateEventA$descriptor(), ccs);
	private static final MethodHandle ClearCommError = linker.downcallHandle(Windows.ClearCommError$address(), Windows.ClearCommError$descriptor(), ccs);
	private static final MethodHandle SetupComm = linker.downcallHandle(Windows.SetupComm$address(), Windows.SetupComm$descriptor(), ccs);
	private static final MethodHandle EscapeCommFunction = linker.downcallHandle(Windows.EscapeCommFunction$address(), Windows.EscapeCommFunction$descriptor(), ccs);
	private static final MethodHandle GetCommMask = linker.downcallHandle(Windows.GetCommMask$address(), Windows.GetCommMask$descriptor(), ccs);
	private static final MethodHandle GetCommProperties = linker.downcallHandle(Windows.GetCommProperties$address(), Windows.GetCommProperties$descriptor(), ccs);
	private static final MethodHandle GetCommModemStatus = linker.downcallHandle(Windows.GetCommModemStatus$address(), Windows.GetCommModemStatus$descriptor(), ccs);
	private static final MethodHandle GetCommState = linker.downcallHandle(Windows.GetCommState$address(), Windows.GetCommState$descriptor(), ccs);
	private static final MethodHandle GetCommTimeouts = linker.downcallHandle(Windows.GetCommTimeouts$address(), Windows.GetCommTimeouts$descriptor(), ccs);
	private static final MethodHandle SetCommMask = linker.downcallHandle(Windows.SetCommMask$address(), Windows.SetCommMask$descriptor(), ccs);
	private static final MethodHandle SetCommState = linker.downcallHandle(Windows.SetCommState$address(), Windows.SetCommState$descriptor(), ccs);
	private static final MethodHandle SetCommTimeouts = linker.downcallHandle(Windows.SetCommTimeouts$address(), Windows.SetCommTimeouts$descriptor(), ccs);
	private static final MethodHandle WaitCommEvent = linker.downcallHandle(Windows.WaitCommEvent$address(), Windows.WaitCommEvent$descriptor(), ccs);
	private static final MethodHandle FormatMessageA = linker.downcallHandle(Windows.FormatMessageA$address(), Windows.FormatMessageA$descriptor(), ccs);


	private static final boolean traceDowncalls = Boolean.getBoolean("jextract.trace.downcalls");

	private static void traceDowncall(final String name, final Object... args) {
		final String traceArgs = Arrays.stream(args)
				.map(Object::toString)
				.collect(Collectors.joining(", "));
		System.out.printf("%s(%s)\n", name, traceArgs);
	}

	/**
	 * {@snippet lang=c :
	 * HANDLE CreateFileA(LPCSTR lpFileName, DWORD dwDesiredAccess, DWORD dwShareMode, LPSECURITY_ATTRIBUTES lpSecurityAttributes, DWORD dwCreationDisposition, DWORD dwFlagsAndAttributes, HANDLE hTemplateFile)
	 * }
	 */
	static MemorySegment CreateFileA(final MemorySegment lastError, final MemorySegment lpFileName, final int dwDesiredAccess, final int dwShareMode, final MemorySegment lpSecurityAttributes, final int dwCreationDisposition, final int dwFlagsAndAttributes, final MemorySegment hTemplateFile) {
		try {
			if (traceDowncalls)
				traceDowncall("CreateFileA", lpFileName, dwDesiredAccess, dwShareMode, lpSecurityAttributes, dwCreationDisposition, dwFlagsAndAttributes, hTemplateFile);
			return (MemorySegment) CreateFileA.invokeExact(lastError, lpFileName, dwDesiredAccess, dwShareMode, lpSecurityAttributes, dwCreationDisposition, dwFlagsAndAttributes, hTemplateFile);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL FlushFileBuffers(HANDLE hFile)
	 * }
	 */
	static int FlushFileBuffers(final MemorySegment lastError, final MemorySegment hFile) {
		try {
			if (traceDowncalls)
				traceDowncall("FlushFileBuffers", hFile);
			return (int) FlushFileBuffers.invokeExact(lastError, hFile);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * DWORD GetFileType(HANDLE hFile)
	 * }
	 */
	static int GetFileType(final MemorySegment lastError, final MemorySegment hFile) {
		try {
			if (traceDowncalls)
				traceDowncall("GetFileType", hFile);
			return (int) GetFileType.invokeExact(lastError, hFile);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL ReadFile(HANDLE hFile, LPVOID lpBuffer, DWORD nNumberOfBytesToRead, LPDWORD lpNumberOfBytesRead, LPOVERLAPPED lpOverlapped)
	 * }
	 */
	static int ReadFile(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpBuffer, final int nNumberOfBytesToRead, final MemorySegment lpNumberOfBytesRead, final MemorySegment lpOverlapped) {
		try {
			if (traceDowncalls)
				traceDowncall("ReadFile", hFile, lpBuffer, nNumberOfBytesToRead, lpNumberOfBytesRead, lpOverlapped);
			return (int) ReadFile.invokeExact(lastError, hFile, lpBuffer, nNumberOfBytesToRead, lpNumberOfBytesRead, lpOverlapped);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL WriteFile(HANDLE hFile, LPCVOID lpBuffer, DWORD nNumberOfBytesToWrite, LPDWORD lpNumberOfBytesWritten, LPOVERLAPPED lpOverlapped)
	 * }
	 */
	static int WriteFile(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpBuffer, final int nNumberOfBytesToWrite, final MemorySegment lpNumberOfBytesWritten, final MemorySegment lpOverlapped) {
		try {
			if (traceDowncalls)
				traceDowncall("WriteFile", hFile, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped);
			return (int) WriteFile.invokeExact(lastError, hFile, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL CloseHandle(HANDLE hObject)
	 * }
	 */
	static int CloseHandle(final MemorySegment lastError, final MemorySegment hObject) {
		try {
			if (traceDowncalls)
				traceDowncall("CloseHandle", hObject);
			return (int) CloseHandle.invokeExact(lastError, hObject);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL GetOverlappedResultEx(HANDLE hFile, LPOVERLAPPED lpOverlapped, LPDWORD lpNumberOfBytesTransferred, DWORD dwMilliseconds, BOOL bAlertable)
	 * }
	 */
	public static int GetOverlappedResultEx(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpOverlapped, final MemorySegment lpNumberOfBytesTransferred, final int dwMilliseconds, final int bAlertable) {
		try {
			if (traceDowncalls)
				traceDowncall("GetOverlappedResultEx", hFile, lpOverlapped, lpNumberOfBytesTransferred, dwMilliseconds, bAlertable);
			return (int) GetOverlappedResultEx.invokeExact(lastError, hFile, lpOverlapped, lpNumberOfBytesTransferred, dwMilliseconds, bAlertable);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * HANDLE CreateEventA(LPSECURITY_ATTRIBUTES lpEventAttributes, BOOL bManualReset, BOOL bInitialState, LPCSTR lpName)
	 * }
	 */
	static MemorySegment CreateEventA(final MemorySegment lastError, final MemorySegment lpEventAttributes, final int bManualReset, final int bInitialState, final MemorySegment lpName) {
		try {
			if (traceDowncalls)
				traceDowncall("CreateEventA", lpEventAttributes, bManualReset, bInitialState, lpName);
			return (MemorySegment) CreateEventA.invokeExact(lastError, lpEventAttributes, bManualReset, bInitialState, lpName);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL ClearCommError(HANDLE hFile, LPDWORD lpErrors, LPCOMSTAT lpStat)
	 * }
	 */
	static int ClearCommError(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpErrors, final MemorySegment lpStat) {
		try {
			if (traceDowncalls)
				traceDowncall("ClearCommError", hFile, lpErrors, lpStat);
			return (int) ClearCommError.invokeExact(lastError, hFile, lpErrors, lpStat);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL SetupComm(HANDLE hFile, DWORD dwInQueue, DWORD dwOutQueue)
	 * }
	 */
	static int SetupComm(final MemorySegment lastError, final MemorySegment hFile, final int dwInQueue, final int dwOutQueue) {
		try {
			if (traceDowncalls)
				traceDowncall("SetupComm", hFile, dwInQueue, dwOutQueue);
			return (int) SetupComm.invokeExact(lastError, hFile, dwInQueue, dwOutQueue);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL EscapeCommFunction(HANDLE hFile, DWORD dwFunc)
	 * }
	 */
	static int EscapeCommFunction(final MemorySegment lastError, final MemorySegment hFile, final int dwFunc) {
		try {
			if (traceDowncalls)
				traceDowncall("EscapeCommFunction", hFile, dwFunc);
			return (int) EscapeCommFunction.invokeExact(lastError, hFile, dwFunc);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL GetCommMask(HANDLE hFile, LPDWORD lpEvtMask)
	 * }
	 */
	static int GetCommMask(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpEvtMask) {
		try {
			if (traceDowncalls)
				traceDowncall("GetCommMask", hFile, lpEvtMask);
			return (int) GetCommMask.invokeExact(lastError, hFile, lpEvtMask);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL GetCommProperties(HANDLE hFile, LPCOMMPROP lpCommProp)
	 * }
	 */
	static int GetCommProperties(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpCommProp) {
		try {
			if (traceDowncalls)
				traceDowncall("GetCommProperties", hFile, lpCommProp);
			return (int) GetCommProperties.invokeExact(lastError, hFile, lpCommProp);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL GetCommModemStatus(HANDLE hFile, LPDWORD lpModemStat)
	 * }
	 */
	static int GetCommModemStatus(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpModemStat) {
		try {
			if (traceDowncalls)
				traceDowncall("GetCommModemStatus", hFile, lpModemStat);
			return (int) GetCommModemStatus.invokeExact(lastError, hFile, lpModemStat);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL GetCommState(HANDLE hFile, LPDCB lpDCB)
	 * }
	 */
	static int GetCommState(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpDCB) {
		try {
			if (traceDowncalls)
				traceDowncall("GetCommState", hFile, lpDCB);
			return (int) GetCommState.invokeExact(lastError, hFile, lpDCB);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL GetCommTimeouts(HANDLE hFile, LPCOMMTIMEOUTS lpCommTimeouts)
	 * }
	 */
	static int GetCommTimeouts(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpCommTimeouts) {
		try {
			if (traceDowncalls)
				traceDowncall("GetCommTimeouts", hFile, lpCommTimeouts);
			return (int) GetCommTimeouts.invokeExact(lastError, hFile, lpCommTimeouts);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL SetCommMask(HANDLE hFile, DWORD dwEvtMask)
	 * }
	 */
	static int SetCommMask(final MemorySegment lastError, final MemorySegment hFile, final int dwEvtMask) {
		try {
			if (traceDowncalls)
				traceDowncall("SetCommMask", hFile, dwEvtMask);
			return (int) SetCommMask.invokeExact(lastError, hFile, dwEvtMask);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL SetCommState(HANDLE hFile, LPDCB lpDCB)
	 * }
	 */
	static int SetCommState(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpDCB) {
		try {
			if (traceDowncalls)
				traceDowncall("SetCommState", hFile, lpDCB);
			return (int) SetCommState.invokeExact(lastError, hFile, lpDCB);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL SetCommTimeouts(HANDLE hFile, LPCOMMTIMEOUTS lpCommTimeouts)
	 * }
	 */
	static int SetCommTimeouts(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpCommTimeouts) {
		try {
			if (traceDowncalls)
				traceDowncall("SetCommTimeouts", hFile, lpCommTimeouts);
			return (int) SetCommTimeouts.invokeExact(lastError, hFile, lpCommTimeouts);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * BOOL WaitCommEvent(HANDLE hFile, LPDWORD lpEvtMask, LPOVERLAPPED lpOverlapped)
	 * }
	 */
	static int WaitCommEvent(final MemorySegment lastError, final MemorySegment hFile, final MemorySegment lpEvtMask, final MemorySegment lpOverlapped) {
		try {
			if (traceDowncalls)
				traceDowncall("WaitCommEvent", hFile, lpEvtMask, lpOverlapped);
			return (int) WaitCommEvent.invokeExact(lastError, hFile, lpEvtMask, lpOverlapped);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}

	/**
	 * {@snippet lang=c :
	 * DWORD FormatMessageA(DWORD dwFlags, LPCVOID lpSource, DWORD dwMessageId, DWORD dwLanguageId, LPSTR lpBuffer, DWORD nSize, va_list *Arguments)
	 * }
	 */
	static int FormatMessageA(final MemorySegment lastError, final int dwFlags, final MemorySegment lpSource, final int dwMessageId, final int dwLanguageId, final MemorySegment lpBuffer, final int nSize, final MemorySegment Arguments) {
		try {
			if (traceDowncalls)
				traceDowncall("FormatMessageA", dwFlags, lpSource, dwMessageId, dwLanguageId, lpBuffer, nSize, Arguments);
			return (int) FormatMessageA.invokeExact(lastError, dwFlags, lpSource, dwMessageId, dwLanguageId, lpBuffer, nSize, Arguments);
		} catch (Error | RuntimeException e) {
			throw e;
		} catch (final Throwable t) {
			throw new AssertionError("should not reach here", t);
		}
	}
}
