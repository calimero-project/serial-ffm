//#define _POSIX_C_SOURCE 200809L

// macOS: disable for dirent and stat, which otherwise contain additional fields
// and create mismatch during runtime
// not necessary for macOS Tahoe 26
//#define __DARWIN_64_BIT_INO_T 0

#include <unistd.h> // write
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <string.h> // strerror
#include <errno.h>
#include <sys/select.h>
#include <signal.h> // kill

#include <limits.h>
#include <dirent.h>
#include <stdlib.h>

// not available on macOS
#ifdef __linux__
#include <linux/serial.h> // serial_icounter_struct, serial_struct
#endif
