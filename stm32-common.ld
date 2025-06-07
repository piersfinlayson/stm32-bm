/* Entry Point */
ENTRY(Reset_Handler)

/* Stack size */
_Min_Stack_Size = 0x400; /* 1KB */

SECTIONS
{
    /* Vector table and code must be in FLASH */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata :
    {
        . = ALIGN(4);
        KEEP(*(.rodata));
        KEEP(*(.rodata*));
        . = ALIGN(4);
    } >FLASH

    /* Data to be copied from flash to RAM */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    /* Uninitialized data */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >RAM

    /* Reserve stack space */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >RAM

    /* Remove information from standard libraries */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
}

/* Set initial stack pointer to end of RAM */
_estack = ORIGIN(RAM) + LENGTH(RAM);
