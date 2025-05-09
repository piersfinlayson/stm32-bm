// Main startup code (clock and GPIO initialisation) for the STM32F103

// Copyright (C) 2025 Piers Finlayson <piers@piers.rocks>
//
// MIT License

#include <stdint.h>
#include <string.h>
#include "include.h"
#include "SEGGER_RTT.h"

// Version and metadata strings to include in the binary
const char __attribute__((used)) build_date[] = __DATE__ " " __TIME__;
const char __attribute__((used)) version[] = "0.1.0";
#ifdef VARIANT
const char __attribute__((used)) target[] = VARIANT;
#endif
const char __attribute__((used)) license[] = "MIT License";
const char __attribute__((used)) repository[] = "https://github.com/piersfinlayson/stm32-bm";
const char __attribute__((used)) author[] = "Piers Finlayson <piers@piers.rocks>";
const char __attribute__((used)) project[] = "stm32-bm";

// Copies a function from flash to RAM
void copy_func_to_ram(void (*fn)(void), uint32_t ram_addr, size_t size) {
    // Copy the function to RAM
    memcpy((void*)ram_addr, (void*)((uint32_t)fn & ~1), size);
}

void execute_ram_func(uint32_t ram_addr) {
    // Execute the function in RAM
    void (*ram_func)(void) = (void(*)(void))(ram_addr | 1);
    ram_func();
}

// Most/all of this is likely unnecessary
void reset_rcc_registers(void) {
    // Set RCC_CR to reset value
    // Retain reserved and read-only bits
    // Set HSION and default HSI trim value
    uint32_t rcc_cr = RCC_CR;
    rcc_cr &= RCC_CR_RSVD_RO_MASK; 
    rcc_cr |= RCC_CR_HSION | (0x10 << 3);
    RCC_CR = rcc_cr;

    // Set RCC_CFGR to reset value
    // Retain reserved and read-only bits
    // Set SW to HSI (value 0)
    uint32_t rcc_cfgr = RCC_CFGR;
    rcc_cfgr &= RCC_CFGR_RSVD_RO_MASK;
    RCC_CFGR = rcc_cfgr;

    // Set RCC_CIR to reset value
    uint32_t rcc_cir = RCC_CIR;
    rcc_cir &= RCC_CIR_RSVD_RO_MASK;
    RCC_CIR = rcc_cir;

    // Set RCC_APB2RSTR to reset value
    uint32_t rcc_apb2rstr = RCC_APB2RSTR;
    rcc_apb2rstr &= RCC_APB2RSTR_RSVD_RO_MASK;
    RCC_APB2RSTR = rcc_apb2rstr;

    // Set RCC_APB1RSTR to reset value
    uint32_t rcc_apb1rstr = RCC_APB1RSTR;
    rcc_apb1rstr &= RCC_APB1RSTR_RSVD_RO_MASK;
    RCC_APB1RSTR = rcc_apb1rstr;

    // Set RCC_AHBENR to reset value
    uint32_t rcc_ahbenr = RCC_AHBENR;
    rcc_ahbenr &= RCC_AHBENR_RSVD_RO_MASK;
    rcc_ahbenr |= (1 << 4);  // FLITF clock enabled during sleep mode
    RCC_AHBENR = rcc_ahbenr;


    // Set RCC_APB2ENR to reset value
    uint32_t rcc_apb2enr = RCC_APB2ENR;
    rcc_apb2enr &= RCC_APB2ENR_RSVD_RO_MASK;
    RCC_APB2ENR = rcc_apb2enr;

    // Set RCC_APB1ENR to reset value
    uint32_t rcc_apb1enr = RCC_APB1ENR;
    rcc_apb1enr &= RCC_APB1ENR_RSVD_RO_MASK;
    RCC_APB1ENR = rcc_apb1enr;

    // Set RCC_BDCR to reset value
    uint32_t rcc_bdcr = RCC_BDCR;
    rcc_bdcr &= RCC_BDCR_RSVD_RO_MASK;
    RCC_BDCR = rcc_bdcr;
}

// May be unnecessary
void reset_afio_registers(void) {
    uint32_t afio_mapr = AFIO_MAPR;
    afio_mapr &= AFIO_MAPR_RSVD_RO_MASK;
    AFIO_MAPR = afio_mapr;
}

// Sets pins PA13 and PA14 to allow them to be used by RTT logging.
// Doesn't appear to be necessary.
void setup_swd(void) {
    // Enable GPIOA clock
    RCC_APB2ENR |= (1 << 2);

    // Configure PA13 and PA14
    uint32_t gpioa_crh = GPIOA_CRH;
    gpioa_crh &= ~(0b1111 << 20);  // Clear bits for PA13
    gpioa_crh |= (0b1000 << 20);   // Set as input with pullup/down
    gpioa_crh &= ~(0b1111 << 24);  // Clear bits for PA14
    gpioa_crh |= (0b1000 << 24);   // Set as input with pullup/down
    GPIOA_CRH = gpioa_crh;

    // Set PA14 as pull-down (ODR bit = 0)
    GPIOA_ODR &= ~(1 << 14);   // Pull-down for PA14
    GPIOA_ODR |= (1 << 13);    // Pull-up for PA13
}

// Sets up the MCO (clock output) on PA8, to the value provided
void setup_mco(uint8_t mco) {
    // Enable GPIOA clock
    RCC_APB2ENR |= (1 << 2);

    // Configure PA8 as output (MODE=11, CNF=00)
    uint32_t gpioa_crh = GPIOA_CRH;
    gpioa_crh &= ~(0b1111 << 0);  // Clear CND and MODE bits for PA8
    gpioa_crh |= (0b1011 << 0);   // Set as AF 50MHz push-pull
    GPIOA_CRH = gpioa_crh;

    // Set MCO bits in RCC_CFGR
    uint32_t rcc_cfgr = RCC_CFGR;
    rcc_cfgr &= ~RCC_CFGR_MCO_MASK;  // Clear MCO bits
    rcc_cfgr |= ((mco & 0b111) << 24);  // Set MCO bits
    RCC_CFGR = rcc_cfgr;
}

// Sets up the PLL multiplier to the value provided
void setup_pll_mul(uint8_t mul) {
    // Set PLL multiplier in RCC_CFGR
    uint32_t rcc_cfgr = RCC_CFGR;
    rcc_cfgr &= ~RCC_CFGR_PLLMULL_MASK;  // Clear PLLMUL bits
    rcc_cfgr |= ((mul & 0b1111) << 18);  // Set PLLMUL bits
    RCC_CFGR = rcc_cfgr;
}

// Sets up the PLL source to the value provided
void setup_pll_src(uint8_t src) {
    // Set PLL source in RCC_CFGR
    uint32_t rcc_cfgr = RCC_CFGR;
    rcc_cfgr &= ~RCC_CFGR_PLLSRC;  // Clear PLLSRC bit
    rcc_cfgr |= (src & 1) << 16;  // Set PLLSRC bit
    RCC_CFGR = rcc_cfgr;
}

// Sets up the PLL XTPRE to the value provided - this is the HSE divider
// for the PLL input clock
void setup_pll_xtpre(uint8_t xtpre) {
    // Set PLL XTPRE in RCC_CFGR
    uint32_t rcc_cfgr = RCC_CFGR;
    rcc_cfgr &= ~RCC_CFGR_PLLXTPRE_MASK;  // Clear PLLXTPRE bit
    rcc_cfgr |= (xtpre & 0b1) << 17;  // Set PLLXTPRE bit
    RCC_CFGR = rcc_cfgr;
}

// Enables the PLL and waits for it to be ready
void enable_pll(void) {
    // Enable PLL
    uint32_t rcc_cr = RCC_CR;
    rcc_cr |= RCC_CR_PLLON;  // Set PLLON bit
    RCC_CR = rcc_cr;

    // Wait for PLL to be ready
    while (!(RCC_CR & RCC_CR_PLLRDY));
}

// Enables the HSE and waits for it to be ready.  If driving the PLL, or
// SYSCLK directly, this must be done first.
void enable_hse(void) {
    // Enable HSE
    uint32_t rcc_cr = RCC_CR;
    rcc_cr |= RCC_CR_HSEON;  // Set HSEON bit
    RCC_CR = rcc_cr;

    // Wait for HSE to be ready
    while (!(RCC_CR & RCC_CR_HSERDY));
}

// Get HSI calibration value
uint8_t get_hsi_cal(void) {
    uint32_t rcc_cr = RCC_CR;
    return (rcc_cr & (0xff << 8)) >> 8;  // Return HSI trim value
}

// Sets the system clock to the value provided.  By default the system clock
// uses HSI.  This function cab be used to set it to HSE directly or to the
// PLL.
void set_clock(uint8_t clock) {
    // Set system clock to PLL
    uint32_t rcc_cfgr = RCC_CFGR;
    rcc_cfgr &= ~RCC_CFGR_SW_MASK;  // Clear SW bits
    rcc_cfgr |= (clock & 0b11);  // Set SW bits
    RCC_CFGR = rcc_cfgr;

    // Wait for system clock to be ready
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != ((clock & 0b11) << 2));
}

void toggle_pa4(void) {
    // Enable GPIOA clock
    RCC_APB2ENR |= (1 << 2);

    GPIOA_CRL &= ~(0xF << 16);  // Clear bits
    GPIOA_CRL |= (0x3 << 16);   // Set as output 50MHz push-pull

    // Configure as output (MODE=11, CNF=00)
    asm volatile (
        "mov r1, #1<<4\n"   // Will set PA4 high
        "mov r2, #1<<20\n"  // Will set PA4 low
        "1:\n"
        ".rept 100\n"
        "str r1, [%0]\n"  // Set PA4 high
        "str r2, [%0]\n"  // Set PA4 low
        ".endr\n"
        "b 1b\n"         // Loop forever
        : // No outputs
        : "r" (GPIOA_BASE + GPIO_BSRR_OFFSET)
        : "r1", "r2"  // Clobbered registers
    ); 
}

void trim_hsi(uint8_t trim) {
    // Set HSI trim value
    uint32_t rcc_cr = RCC_CR;
    rcc_cr &= ~RCC_CR_HSITRIM_MAX;  // Clear HSITRIM bits
    rcc_cr |= ((trim & 0b11111) << 3);
    RCC_CR = rcc_cr;

    // Wait for HSI to be ready
    while (!(RCC_CR & RCC_CR_HSIRDY));
}

// Main
int main(void) {
    reset_rcc_registers();

    reset_afio_registers();

#if DEBUG == 1
    setup_swd();
    LOG("Debugging enabled");
#endif

#if MCO == 1
    LOG("Setup MCO on PA8 (SYSCLK)");
    setup_mco(RCC_CFGR_MCO_SYSCLK);
#endif

#if HSI == 1
    uint8_t hsi_cal = get_hsi_cal();
    LOG("HSI calibration value: 0x%x", hsi_cal);
    
#if defined(HSI_TRIM)
    // No-op - system clock is HSI by default
    LOG("Trim HSI: 0x%x", HSI_TRIM);
    trim_hsi(HSI_TRIM);  // Max HSI trim value
#endif // HSI_TRIM
    LOG("HSI clock trim value: 0x%x", (RCC_CR & RCC_CR_HSITRIM_MAX) >> 3);
#endif // HSI

#if HSE == 1
    LOG("Enable HSE");
    enable_hse();
#endif // HSE

#if defined(PLL)
    LOG("Setup PLL");

    LOG("Setup PLL multiplier: 0x%x", PLL_MUL_VAL);
    setup_pll_mul(PLL_MUL_VAL);

#if HSI == 1
    uint8_t pll_src = RCC_CFGR_PLLSRC_HSI;
#elif HSE == 1
    uint8_t pll_src = RCC_CFGR_PLLSRC_HSE;
#else
#error("Invalid PLL source")
#endif
    LOG("Setup PLL source: %x", pll_src);
    setup_pll_src(pll_src);

    LOG("Enable PLL");
    enable_pll();

    LOG("Switch MCO to PLL/2: %x", RCC_CFGR_MCO_PLL_DIV2);
    setup_mco(RCC_CFGR_MCO_PLL_DIV2);

    // Before we can switch to the PLL we must configure bus prescalers

    // AHB = SYSCLK not divided
    RCC_CFGR &= ~RCC_CFGR_HPRE_MASK;
        
    // APB1 = HCLK/2 (max 36MHz)
    RCC_CFGR &= ~RCC_CFGR_PPRE1_MASK;
    RCC_CFGR |= RCC_CFGR_PPRE1_DIV2;
       
    // APB2 = HCLK not divided
    RCC_CFGR &= ~RCC_CFGR_PPRE2_MASK;

    // Set Flash latency to 2 wait states for 64-72MHz.  We need to do this
    // before we switch to the PLL as we're running from flash.
    FLASH_ACR = FLASH_ACR_LATENCY_2WS | FLASH_ACR_PRFTBE;

    LOG("Set system clock to PLL: %x", RCC_CFGR_SW_PLL);
    set_clock(RCC_CFGR_SW_PLL);
#else // PLL
#if HSE == 1
    set_clock(RCC_CFGR_SW_HSE);
#endif
#endif // PLL

    LOG("Switch MCO to SYSCLK: %x", RCC_CFGR_MCO_SYSCLK);
    setup_mco(RCC_CFGR_MCO_SYSCLK);

    LOG("System started");

#if EXECUTE_FROM_RAM == 1
    copy_func_to_ram(toggle_pa4, (uint32_t)RAM_FUNC, 1024);
    execute_ram_func((uint32_t)RAM_FUNC);
#else // EXECUTE_FROM_RAM == 0
    // Call the function directly
    toggle_pa4();
#endif // EXECUTE_FROM_RAM

    return 0;
}
