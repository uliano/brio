// probe - the smallest firmware for the SAM C21 board: raw-register OSC48M
// to 48 MHz and a PB23 LED blink, nothing else. The samc21 analog of the
// AVR project's family_probe, with the same two jobs: on the desk it proves
// the whole new chain (toolchain flags, linker script, startup, vector
// table, OpenOCD flash) with zero brio code in the loop; at the bench it is
// the first thing flashed onto a NEW board - the LED says the chip runs,
// the SWD link works, and the clock really moved.
//
// This is the ONE app allowed to poke registers directly (it exists exactly
// to prove the layer below the samc/ stratum; apps proper never touch
// registers - see CLAUDE.md's layering rule). The clock sequence it proves
// is the stratum's future Clock<internal, 48 MHz>::init() in miniature:
//
//   1. NVMCTRL.CTRLB.RWS = 2 first - table 45-41 (DS60001479M): flash reads
//      hold to 19 MHz at 0 wait states, 38 at 1, 64 at 2; and 27.5.2 orders
//      the wait states adapted to the FUTURE frequency before the switch.
//   2. OSC48MDIV.DIV = 0 (reset value 0xB = /12 = 4 MHz; 0 = /1 = 48 MHz),
//      then wait OSC48MSYNCBUSY.OSC48MDIV. OSC48M is GCLK0's source out of
//      reset, i.e. requested, so erratum 1.2.2 (DS80000740S: DIV write
//      while unrequested wedges SYNCBUSY) does not bite on this path.
//
// Wiring: none - the LED on PB23 is on the board.
//
// build: boards = c21j

#include "samc21j18a.h"

#include <stdint.h>

namespace {

constexpr uint32_t led_mask = 1u << 23;   // PB23, group 1

void clock_48mhz()
{
    NVMCTRL_REGS->NVMCTRL_CTRLB =
        (NVMCTRL_REGS->NVMCTRL_CTRLB & ~NVMCTRL_CTRLB_RWS_Msk) |
        NVMCTRL_CTRLB_RWS(2);

    OSCCTRL_REGS->OSCCTRL_OSC48MDIV = OSCCTRL_OSC48MDIV_DIV(0);
    while ((OSCCTRL_REGS->OSCCTRL_OSC48MSYNCBUSY &
            OSCCTRL_OSC48MSYNCBUSY_OSC48MDIV_Msk) != 0u) {}
}

void delay_rough(uint32_t loops)
{
    for (volatile uint32_t i = 0; i < loops; i = i + 1) {}
}

} // namespace

int main()
{
    clock_48mhz();

    PORT_REGS->GROUP[1].PORT_DIRSET = led_mask;

    for (;;) {
        PORT_REGS->GROUP[1].PORT_OUTTGL = led_mask;
        // ~8 cycles per volatile loop turn at -Os; ~3M turns = ~0.5 s at
        // 48 MHz -> ~1 Hz blink. Roughly: the exact rate is the kernel
        // ticker's job, not this probe's.
        delay_rough(3'000'000u);
    }
}
