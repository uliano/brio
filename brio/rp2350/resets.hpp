/*
 * resets.hpp
 *
 * The subsystem reset controller (RESETS, datasheet 7.5): this chip's
 * gate on every peripheral. Every block the controller governs - the
 * UARTs, SPIs, I2Cs, PWM, both system timers, the ADC, both PLLs, the
 * three PIOs, the DMA, the IO and pad banks, the USB controller, the
 * SHA-256 and TRNG accelerators, the HSTX, SYSINFO - is HELD IN RESET at
 * power-up, and a register of a block in reset is not there to be
 * written. So "open the peripheral's clock", the first act of every
 * driver's init on the other families, is "release its reset" here, and
 * RESET_DONE is what says the release has taken.
 *
 * Three registers: RESET (a bit per block, set = in reset), WDSEL (which
 * blocks a watchdog event resets) and RESET_DONE (read-only). The writes
 * go through the SET/CLR aliases (device.hpp), so releasing one block
 * never disturbs another and no critical section is needed. The release
 * wait is bounded like every wait in this stratum: a block whose done
 * bit never rises answers false, and the caller knows.
 *
 * THE BIT MAP IS NOT THE RP2040'S. Twenty-nine blocks here against
 * twenty-seven there, and the order differs from the first bit on: this
 * chip has no RTC (the always-on timer in POWMAN took its place) and no
 * separate JTAG line in the same position, and it adds TIMER1, PIO2,
 * HSTX, SHA256 and TRNG. Every constant below is the device header's
 * own name, so the map is read and never retyped.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

namespace brio {

/// The blocks, as RESET bit masks (7.5.3).
struct ResetBlock {
    static constexpr uint32_t adc = RESETS_RESET_ADC_BITS;
    static constexpr uint32_t busctrl = RESETS_RESET_BUSCTRL_BITS;
    static constexpr uint32_t dma = RESETS_RESET_DMA_BITS;
    static constexpr uint32_t hstx = RESETS_RESET_HSTX_BITS;
    static constexpr uint32_t i2c0 = RESETS_RESET_I2C0_BITS;
    static constexpr uint32_t i2c1 = RESETS_RESET_I2C1_BITS;
    static constexpr uint32_t io_bank0 = RESETS_RESET_IO_BANK0_BITS;
    static constexpr uint32_t io_qspi = RESETS_RESET_IO_QSPI_BITS;
    static constexpr uint32_t jtag = RESETS_RESET_JTAG_BITS;
    static constexpr uint32_t pads_bank0 = RESETS_RESET_PADS_BANK0_BITS;
    static constexpr uint32_t pads_qspi = RESETS_RESET_PADS_QSPI_BITS;
    static constexpr uint32_t pio0 = RESETS_RESET_PIO0_BITS;
    static constexpr uint32_t pio1 = RESETS_RESET_PIO1_BITS;
    static constexpr uint32_t pio2 = RESETS_RESET_PIO2_BITS;
    static constexpr uint32_t pll_sys = RESETS_RESET_PLL_SYS_BITS;
    static constexpr uint32_t pll_usb = RESETS_RESET_PLL_USB_BITS;
    static constexpr uint32_t pwm = RESETS_RESET_PWM_BITS;
    static constexpr uint32_t sha256 = RESETS_RESET_SHA256_BITS;
    static constexpr uint32_t spi0 = RESETS_RESET_SPI0_BITS;
    static constexpr uint32_t spi1 = RESETS_RESET_SPI1_BITS;
    static constexpr uint32_t syscfg = RESETS_RESET_SYSCFG_BITS;
    static constexpr uint32_t sysinfo = RESETS_RESET_SYSINFO_BITS;
    static constexpr uint32_t tbman = RESETS_RESET_TBMAN_BITS;
    static constexpr uint32_t timer0 = RESETS_RESET_TIMER0_BITS;
    static constexpr uint32_t timer1 = RESETS_RESET_TIMER1_BITS;
    static constexpr uint32_t trng = RESETS_RESET_TRNG_BITS;
    static constexpr uint32_t uart0 = RESETS_RESET_UART0_BITS;
    static constexpr uint32_t uart1 = RESETS_RESET_UART1_BITS;
    static constexpr uint32_t usbctrl = RESETS_RESET_USBCTRL_BITS;
    /// Every block the controller governs.
    static constexpr uint32_t all = RESETS_RESET_DONE_BITS;
};

/// The controller, a monostate.
struct Resets {
    Resets() = delete;

    /// Release `blocks` from reset and wait for each one's RESET_DONE.
    /// True when every one reported ready within the bounded wait; a
    /// block already released is simply confirmed. A no-op on zero.
    static bool release(uint32_t blocks) {
        hw_clear(RESETS->RESET, blocks);
        return wait_done(blocks);
    }

    /// Put `blocks` into reset: their registers are gone until the next
    /// release(), and their state with them.
    static void hold(uint32_t blocks) { hw_set(RESETS->RESET, blocks); }

    /// hold() then release(): the way a driver's init starts a block
    /// from its reset state, whatever a previous life left in it - and
    /// on this chip a previous life is the ordinary case, since a
    /// debugger's reset request resets the cores alone.
    static bool cycle(uint32_t blocks) {
        hold(blocks);
        return release(blocks);
    }

    /// True when every one of `blocks` is out of reset and ready.
    static bool released(uint32_t blocks) {
        return (RESETS->RESET_DONE & blocks) == blocks;
    }

    /// True when every one of `blocks` is held in reset.
    static bool held(uint32_t blocks) { return (RESETS->RESET & blocks) == blocks; }

    /// Which blocks a watchdog event resets (WDSEL), as a whole word.
    static void watchdog_resets(uint32_t blocks) { RESETS->WDSEL = blocks; }
    static uint32_t watchdog_resets() { return RESETS->WDSEL; }

private:
    /// The done bits rise within a handful of cycles of the release; the
    /// budget is generous so that a genuinely stuck block is the only way
    /// to spend it.
    static bool wait_done(uint32_t blocks) {
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if (released(blocks)) {
                return true;
            }
        }
        return false;
    }
};

} // namespace brio
