/**
 * Minimal/stub implementation for newlibc system calls for
 * bare metal embedded system programming.
 *
 * Minimal implementation of the following functions:
 * - _isatty/_stat/_fstat: regard the streams STDIN/STDOUT/STDERR
 * - _read: reading STDIN is routed to a serial port (STM32/STHAL specific)
 * - _write: writing STDOUT or STDERR is routed to a serial port (STM32/STHAL specific)
 * - _sbrk: increase heap size until it gets too close to the stack
 *
 * Other functions, which are mainly process and file system related,
 * are implemented as dumb stubs and return an error code.
 *
 * The serial port used here is accessed via the external variable
 * syscall_UART_handle_ptr which must be defined elsewhere; the same applies
 * to the configuration of the serial port.
 *
 * Refer to https://sourceware.org/newlib/libc.html#Syscalls
 * Based on the newlibc example.
 */

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>


#if defined STM32F1
# include <stm32f1xx_hal.h>
#elif defined STM32F2
# include <stm32f2xx_hal.h>
#elif defined STM32F4
# include <stm32f4xx_hal.h>
#endif

#ifndef SYSCALL_UART
extern UART_HandleTypeDef *syscall_UART_handle_ptr;
# define SYSCALL_UART
#endif


#define SYSCALL_FATTR __attribute__((weak))

#undef errno
extern int errno;

char *__env[1] = { 0 };
char **environ = __env;



/***************************************************************************/
/* Pure stub implementations which do not deliver useful functionality.    */
/***************************************************************************/

/**
 * Stub for transfering control to a new process.
 */
SYSCALL_FATTR int _execve(char *name, char **argv, char **env) {
    errno = ENOMEM;
    return -1;
}

/**
 * Stub for creating a new process.
 */
SYSCALL_FATTR int _fork() {
    errno = EAGAIN;
    return -1;
}

/**
 * Stub for getting the current process id.
 */
SYSCALL_FATTR int _getpid() {
    return 1;
}

/**
 * Stub for sending a signal to a process.
 */
SYSCALL_FATTR int _kill(int pid, int sig) {
    errno = EINVAL;
    return (-1);
}

/**
 * Stub for waiting for a child process.
 */
SYSCALL_FATTR int _wait(int *status) {
    errno = ECHILD;
    return -1;
}

/**
 * Stub for getting timing information for the current process.
 */
SYSCALL_FATTR clock_t _times(struct tms *buf) {
    return -1;
}

/**
 * Stub for closing a file.
 */
SYSCALL_FATTR int _close(int file) {
    return -1;
}

/**
 * Stub for setting the position in a file.
 */
SYSCALL_FATTR int _lseek(int file, int ptr, int dir) {
    return 0;
}

/**
 * Stub for establishing a new name for an existing file.
 */
SYSCALL_FATTR int _link(char *old, char *new) {
    errno = EMLINK;
    return -1;
}

/**
 * Stub for removing a file's directory entry.
 */
SYSCALL_FATTR int _unlink(char *name) {
    errno = ENOENT;
    return -1;
}


/***************************************************************************/
/* Minimal implementations of limited functionality.                       */
/***************************************************************************/

/**
 * Minimal implementation to determine whether a stream is a terminal.
 * Here, STDIN/STDOUT/STDERR are, and querying other streams results
 * in an error.
 */
SYSCALL_FATTR int _isatty(int file) {
    switch (file){
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;
    default:
        //errno = ENOTTY;
        errno = EBADF;
        return 0;
    }
}

/**
 * Minimal implementation for getting the status of a file. Always
 * returns 0 with st->st_mode=S_IFCHR (is character special file).
 */
SYSCALL_FATTR int _stat(const char *filepath, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/**
 * Minimal implementation for getting the status of an open file. Always
 * returns 0 with st->st_mode=S_IFCHR (is character special file).
 */
SYSCALL_FATTR int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/**
 * Minimal implemetation for reading a character from a file. Reading
 * STDIN reads form the UART, and reading other files results in an error.
 */
SYSCALL_FATTR int _read(int file, char *ptr, int len) {
    int result;
    switch (file) {
    case STDIN_FILENO:
        HAL_UART_Receive(syscall_UART_handle_ptr, (uint8_t *)ptr, len, 100);
        result = len - syscall_UART_handle_ptr->RxXferCount; /* number of transfers */
        break;
    default:
        errno = EBADF;
        result = -1; /* indicate an error */
    }
    return result;
}

/**
 * Minimal implementation for writing a string to a file. Writing
 * STDOUT or STDERR writes to the UART, and writing other destinations
 * results in an error.
 */
SYSCALL_FATTR int _write(int file, char *ptr, int len) {
    switch (file) {
    case STDOUT_FILENO:
    case STDERR_FILENO:
        HAL_UART_Transmit(syscall_UART_handle_ptr, (uint8_t *)ptr, len, 100);
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return len;
}

/**
 * Minimal implemantation to increase program data space.
 * Malloc and related functions depend on this.
 */
SYSCALL_FATTR caddr_t _sbrk(int incr) {
    extern char _ebss;   /* symbol defined by the linker */
    extern char _estack; /* symbol defined by the linker */
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0)
    {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

    /* determine the maximum address available for the Heap */
    /* Start heapMax determination with the current stack pointer:
       If the heap grows into the stack, this is obviously a major
       problem. As we cannot control stack growth into the heap, we
       require at least 128 bytes between stackCurrent and heapMax. */
    char *heapMax = (char *) __get_MSP() - 128;
    /* In FreeRTOS (and probaby also in similar RTOSs), the stack
       usually lies below the heap in a BSS area managed by the OS.
       Therefore in this situation, heapMax is limited by the end
       of RAM. */
    if ( (char *) __get_MSP() < prev_heap_end )
    {
        /* Either the stack completely lies under the heap, or
           stack and heap already have collided. */
        heapMax = &_estack; /* this symbol is defined at the end of RAM */
    }

    if (heap_end + incr > heapMax)
    {
        _write (STDERR_FILENO, "\nHeap and heapMax collision\n", 28);
        errno = ENOMEM;
        return (caddr_t) -1;
        // Alternative implementation: exit(-1) to stop the system
    }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
}

/**
 * Minimal implemantation for exiting the current process by writing
 * a message to the serial console and entering an infinite loop.
 *
 * On a concrete board, it would make a lot of sense to toggle a
 * status LED here; in a product, you might want to reboot.
 */
SYSCALL_FATTR void _exit(int status) {
    _write(1, "\nterminated\n", 12);
    while (1) {
        ; /* nothing to do */
    }
}
