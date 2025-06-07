/* Linker script for STM32F411XE microcontroller */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K  /* Single SRAM block */
}

INCLUDE stm32-common.ld