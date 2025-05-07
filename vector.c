// Vector table and reset handler.

// Copyright (C) 2025 Piers Finlayson <piers@piers.rocks>
//
// MIT License

#include <stdint.h>
#include <string.h>
#include "include.h"

// Forward declarations
void Reset_Handler(void);
void Default_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);

// Default exception/interrupt handlers
#define MemManage_Handler   Default_Handler
#define SVC_Handler         Default_Handler
#define DebugMon_Handler    Default_Handler
#define PendSV_Handler      Default_Handler
#define SysTick_Handler     Default_Handler

// Declare stack section
extern uint32_t _estack;

// Vector table - must be placed at the start of flash
__attribute__ ((section(".isr_vector"), used))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))&_estack,      // Initial stack pointer
    Reset_Handler,                 // Reset handler
    NMI_Handler,                   // NMI handler
    HardFault_Handler,             // Hard fault handler
    MemManage_Handler,             // MPU fault handler
    BusFault_Handler,              // Bus fault handler
    UsageFault_Handler,            // Usage fault handler
    0, 0, 0, 0,                    // Reserved
    SVC_Handler,                   // SVCall handler
    DebugMon_Handler,              // Debug monitor handler
    0,                             // Reserved
    PendSV_Handler,                // PendSV handler
    SysTick_Handler,               // SysTick handler

    // Peripheral interrupt handlers follow
};

// Variables defined by the linker.
//
// Note these are "labels" that mark memory addresses, not variables that
// store data.  The address of the label IS the address we're interested in.
// Hence we use & below to get the addresses that these labels represent.
extern uint32_t _sidata;    // Start of .data section in FLASH
extern uint32_t _sdata;     // Start of .data section in RAM
extern uint32_t _edata;     // End of .data section in RAM
extern uint32_t _sbss;      // Start of .bss section in RAM
extern uint32_t _ebss;      // End of .bss section in RAM
extern uint32_t _main_loop_start;
extern uint32_t _main_loop_end;

// Reset handler
void Reset_Handler(void) {
    // We use memcpy and memset because it's likely to be faster than anything
    // we could come up with.

    // Copy data section from flash to RAM
    memcpy(&_sdata, &_sidata, (unsigned int)((char*)&_edata - (char*)&_sdata));
    
    // Zero out bss section  
    memset(&_sbss, 0, (unsigned int)((char*)&_ebss - (char*)&_sbss));
    
    // Call the main function
    extern int main(void);
    main();
    
    // In case main returns
    while(1);
}

// Default handler for unhandled interrupts
void Default_Handler(void) {
    while(1);
}

// NMI_Handler - toggles PA0
void NMI_Handler(void) {
    // Enable GPIOA clock
    RCC_APB2ENR |= (1 << 2);
    
    // Configure as output (MODE=11, CNF=00)
    GPIOA_CRL &= ~(0xF << 0);  // Clear bits
    GPIOA_CRL |= (0x3 << 0);   // Set as output 50MHz push-pull
    
    while(1) {
        GPIOC_ODR ^= (1 << 0);
    }
}

// HardFault_Handler - toggles PA1
void HardFault_Handler(void) {
    // Enable GPIOA clock
    RCC_APB2ENR |= (1 << 2);
    
    // Configure as output (MODE=11, CNF=00)
    GPIOA_CRL &= ~(0xF << 4);  // Clear bits
    GPIOA_CRL |= (0x3 << 4);   // Set as output 50MHz push-pull
    
    while(1) {
        GPIOA_ODR ^= (1 << 1);
    }
}

// BusFault_Handler - toggles PA2
void BusFault_Handler(void) {
    // Enable GPIOA clock
    RCC_APB2ENR |= (1 << 2);
    
    // Configure as output (MODE=11, CNF=00)
    GPIOA_CRL &= ~(0xF << 8);   // Clear bits
    GPIOA_CRL |= (0x3 << 8);    // Set as output 50MHz push-pull
    
    while(1) {
        GPIOA_ODR ^= (1 << 2);
    }
}

// UsageFault_Handler - toggles PA3
void UsageFault_Handler(void) {
    // Enable GPIOA clock
    RCC_APB2ENR |= (1 << 2);
    
    // Configure as output (MODE=11, CNF=00)
    GPIOA_CRL &= ~(0xF << 12);  // Clear bits
    GPIOA_CRL |= (0x3 << 12);   // Set as output 50MHz push-pull
    
    while(1) {
        GPIOA_ODR ^= (1 << 3);
    }
}
