// probe - the smallest firmware for the Nucleo-G0B1RE: raw-register
// HSI16 -> PLL -> 64 MHz and a PA5 (LD4) blink, nothing else. The stm32g0
// analog of the AVR project's family_probe and the samc probe, with the
// same two jobs: on the desk it proves the whole new chain (toolchain
// flags, linker script, startup, vector table, OpenOCD over the ST-LINK)
// with zero brio code in the loop; at the bench it is the first thing
// flashed onto a NEW board - the LED says the chip runs, the SWD link
// works, and the clock really moved.
//
// This is the ONE app allowed to poke registers directly (it exists
// exactly to prove the layer below the stm32g0/ stratum; apps proper
// never touch registers - CLAUDE.md's layering rule). The sequence it
// proves is the stratum's Clock<pll, 64 MHz>::init() in miniature
// (RM0444 3.3.4, 5.2.4, 5.4.4):
//
//   1. FLASH_ACR.LATENCY = 2 first and read back - table 13: HCLK holds
//      to 24 MHz at 0 wait states, 48 at 1, 64 at 2, and 3.3.4 orders
//      the wait states adapted to the FUTURE frequency before the switch.
//   2. PLLCFGR: source HSI16 (16 MHz, on and ready out of reset), M = 1,
//      N = 8 (VCO 128 MHz, inside 96..344), R = 2 -> PLLRCLK 64 MHz,
//      PLLREN set; then PLLON and wait PLLRDY.
//   3. CFGR.SW = PLLRCLK and wait until SWS says so.
//   4. IOPENR.GPIOAEN (the port is DEAD without its clock, 5.2.17),
//      PA5 output, toggle with a busy loop sized for 64 MHz.
//
// The blink rate is the timing witness: about 1 Hz (4M turns of a
// ~7-cycle volatile loop per half period at 64 MHz - measured 0.44 s
// per half over SWD) means the PLL ran; a 4x slower blink means the
// core is still on HSI16.
//
// Wiring: none - LD4 on PA5 is on the board (UM2324's Nucleo-64
// arrangement; verified here by the pad moving).
//
// build: boards = g0b1re

#include "stm32g0xx.h"

#include <stdint.h>

int main()
{
    // 1. Wait states for 64 MHz, in force only when they read back.
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | FLASH_ACR_LATENCY_1;
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_1) {}

    // 2. The PLL, configured stopped.
    RCC->CR = RCC->CR & ~RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) != 0u) {}
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI |
                   (0u << RCC_PLLCFGR_PLLM_Pos) |     // M = 1
                   (8u << RCC_PLLCFGR_PLLN_Pos) |     // N = 8
                   (1u << RCC_PLLCFGR_PLLR_Pos) |     // R = 2
                   RCC_PLLCFGR_PLLREN;
    RCC->CR = RCC->CR | RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0u) {}

    // 3. SYSCLK from PLLRCLK.
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_1;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_1) {}

    // 4. PA5 as a push-pull output.
    RCC->IOPENR = RCC->IOPENR | RCC_IOPENR_GPIOAEN;
    (void)RCC->IOPENR;
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE5_Msk) | GPIO_MODER_MODE5_0;

    for (;;) {
        GPIOA->BSRR = GPIO_BSRR_BS5;
        for (volatile uint32_t i = 0; i < 4'000'000u; i = i + 1) {}
        GPIOA->BRR = GPIO_BRR_BR5;
        for (volatile uint32_t i = 0; i < 4'000'000u; i = i + 1) {}
    }
}
