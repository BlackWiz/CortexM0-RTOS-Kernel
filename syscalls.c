/** @file syscalls.c
 *
 * @brief Newlib system call stubs for bare-metal ARM
 *
 * Minimal implementations of system calls required by newlib.
 * These prevent linking errors when using standard C library.
 *
 * @par Dissertation Project
 * M.Tech - Building a Real-Time Operating System
 */

#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>

/* Forward declarations */
int _close(int fd);
void _exit(int status);
int _fstat(int fd, struct stat *st);
int _isatty(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char *ptr, int len);
int _write(int fd, char *ptr, int len);
void* _sbrk(int incr);
int _getpid(void);
int _kill(int pid, int sig);

/**
 * @brief Exit function - loops forever on bare-metal
 */
void _exit(int status) {
    (void)status;
    while (1) {
        /* Infinite loop */
    }
}

/**
 * @brief Close file descriptor (not supported)
 */
int _close(int fd) {
    (void)fd;
    return -1;
}

/**
 * @brief File status (treat all files as character devices)
 */
int _fstat(int fd, struct stat *st) {
    (void)fd;
    st->st_mode = S_IFCHR;
    return 0;
}

/**
 * @brief Check if file descriptor is a TTY
 */
int _isatty(int fd) {
    (void)fd;
    return 1;
}

/**
 * @brief Seek within file (not supported)
 */
int _lseek(int fd, int ptr, int dir) {
    (void)fd;
    (void)ptr;
    (void)dir;
    return 0;
}

/**
 * @brief Read from file (not supported)
 */
int _read(int fd, char *ptr, int len) {
    (void)fd;
    (void)ptr;
    (void)len;
    return 0;
}

/**
 * @brief Write to file (could redirect to UART in future)
 */
int _write(int fd, char *ptr, int len) {
    (void)fd;
    (void)ptr;
    /* Could implement UART output here */
    return len;
}

/**
 * @brief Get process ID (return dummy value)
 */
int _getpid(void) {
    return 1;
}

/**
 * @brief Kill process (not supported)
 */
int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

/**
 * @brief Allocate memory from heap (not supported - use fixed stacks)
 */
void* _sbrk(int incr) {
    extern char _end; /* Defined by linker script */
    static char *heap_end = NULL;

    (void)incr;
    
    if (heap_end == NULL) {
        heap_end = &_end;
    }

    /* No dynamic heap allocation in RTOS */
    errno = ENOMEM;
    return (void *)-1;
}

/*** end of file ***/