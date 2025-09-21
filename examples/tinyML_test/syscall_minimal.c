#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>

// Linker symbols from linker script
extern char __heap_start__;
extern char __heap_end__;
static char *heap_end = &__heap_start__;

__attribute__((weak)) int _write(int file, const char *ptr, int len) {
    // For embedded: could redirect to UART/RTT, for now just return success
    (void)file;
    (void)ptr;
    return len;
}

int _close(int file) { 
    (void)file;
    return -1; 
}

int _fstat(int file, struct stat *st) { 
    (void)file;
    st->st_mode = S_IFCHR; 
    return 0; 
}

int _isatty(int file) { 
    (void)file;
    return 1; 
}

int _lseek(int file, int ptr, int dir) { 
    (void)file;
    (void)ptr;
    (void)dir;
    return 0; 
}

__attribute__((weak)) int _read(int file, char *ptr, int len) { 
    (void)file;
    (void)ptr;
    (void)len;
    return 0; 
}

int _kill(int pid, int sig) { 
    (void)pid;
    (void)sig;
    errno = EINVAL; 
    return -1; 
}

int _getpid(void) { 
    return 1; 
}

caddr_t _sbrk(int incr) {
    char *prev_heap_end = heap_end;
    if (heap_end + incr > &__heap_end__) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }
    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

void _exit(int status) {
    (void)status;
    while (1) { 
        // Infinite loop - system halt
    }
}
