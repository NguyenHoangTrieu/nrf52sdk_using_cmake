/* nrf_common.ld - Common definitions for nRF52840 */

MEMORY
{
  FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x100000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x40000
}

/* Stack and heap sizes */
__stack_size__ = DEFINED(__stack_size__) ? __stack_size__ : 8192;
__heap_size__ = DEFINED(__heap_size__) ? __heap_size__ : 8192;