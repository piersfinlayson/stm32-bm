/* Linker script for 64KB flash version of the STM32F103 microcontroller */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K  /* SRAM1 = 112KB, SRAM2 = 16KB) */
    CCMRAM (rw) : ORIGIN = 0x10000000, LENGTH = 64K   /* Cannot execute from STM32F4 CCM RAM */
}

INCLUDE stm32-common.ld