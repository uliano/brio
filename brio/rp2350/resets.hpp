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
 *
 * AND BESIDE IT, THE POWER-ON STATE MACHINE (7.4), which is chapter 7's
 * MIDDLE tier: the SYSTEM resets, the ones a processor cannot run
 * without - the oscillators, the clock generators, the bus fabric, the
 * memories, SIO, the access controller, the processors themselves - each
 * released in sequence by hardware, every stage waiting for the one
 * before it. Software never drives that sequence; what software does
 * touch is WDSEL, the stage a watchdog event restarts from, and FRCE_OFF,
 * which holds a stage down. Both are read by `Psm` here rather than
 * spelled again in the two chapters that need them (the watchdog's
 * selection, and erratum RP2350-E19's guard before a reboot), and
 * FRCE_ON is deliberately absent: 7.4.2 calls it a development feature
 * that does nothing on a production device.
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

// ---- the power-on state machine (7.4) ------------------------------------

/// The stages of the sequence, in the order hardware releases them
/// (figure 27), as WDSEL / FRCE_OFF / DONE bit masks.
struct PsmStage {
    static constexpr uint32_t proc_cold = PSM_WDSEL_PROC_COLD_BITS;
    static constexpr uint32_t otp = PSM_WDSEL_OTP_BITS;
    static constexpr uint32_t rosc = PSM_WDSEL_ROSC_BITS;
    static constexpr uint32_t xosc = PSM_WDSEL_XOSC_BITS;
    static constexpr uint32_t resets = PSM_WDSEL_RESETS_BITS;
    static constexpr uint32_t clocks = PSM_WDSEL_CLOCKS_BITS;
    static constexpr uint32_t psm_ready = PSM_WDSEL_PSM_READY_BITS;
    static constexpr uint32_t busfabric = PSM_WDSEL_BUSFABRIC_BITS;
    static constexpr uint32_t rom = PSM_WDSEL_ROM_BITS;
    static constexpr uint32_t bootram = PSM_WDSEL_BOOTRAM_BITS;
    static constexpr uint32_t sram0 = PSM_WDSEL_SRAM0_BITS;
    static constexpr uint32_t sram1 = PSM_WDSEL_SRAM1_BITS;
    static constexpr uint32_t sram2 = PSM_WDSEL_SRAM2_BITS;
    static constexpr uint32_t sram3 = PSM_WDSEL_SRAM3_BITS;
    static constexpr uint32_t sram4 = PSM_WDSEL_SRAM4_BITS;
    static constexpr uint32_t sram5 = PSM_WDSEL_SRAM5_BITS;
    static constexpr uint32_t sram6 = PSM_WDSEL_SRAM6_BITS;
    static constexpr uint32_t sram7 = PSM_WDSEL_SRAM7_BITS;
    static constexpr uint32_t sram8 = PSM_WDSEL_SRAM8_BITS;
    static constexpr uint32_t sram9 = PSM_WDSEL_SRAM9_BITS;
    static constexpr uint32_t xip = PSM_WDSEL_XIP_BITS;
    static constexpr uint32_t sio = PSM_WDSEL_SIO_BITS;
    static constexpr uint32_t accessctrl = PSM_WDSEL_ACCESSCTRL_BITS;
    static constexpr uint32_t proc0 = PSM_WDSEL_PROC0_BITS;
    static constexpr uint32_t proc1 = PSM_WDSEL_PROC1_BITS;

    /// Every stage the sequencer governs.
    static constexpr uint32_t all = PSM_WDSEL_BITS;
    /// The two oscillators, which a reboot keeps running: a watchdog
    /// selection that included them would drop the crystal and make the
    /// reboot pay for its startup all over again.
    static constexpr uint32_t oscillators = rosc | xosc;
    /// What a watchdog event restarts by default here and in the SDK:
    /// the whole sequence but the oscillators, so the chip comes back
    /// through the bootrom as a power-on would.
    static constexpr uint32_t reboot = all & ~oscillators;
};

/// The sequencer, a monostate. It has no init: hardware ran it before
/// the first instruction of this image existed.
struct Psm {
    Psm() = delete;

    /// Which stage a watchdog event restarts the sequence from (WDSEL),
    /// as a whole word - the register has no read-modify-write hazard
    /// worth an alias here, one selection standing at a time.
    static void watchdog_resets(uint32_t stages) { PSM->WDSEL = stages & PsmStage::all; }
    static uint32_t watchdog_resets() { return PSM->WDSEL & PsmStage::all; }

    /// Which stages are DONE - the whole sequence, once the program is
    /// running, minus anything hold() is keeping down.
    static uint32_t done() { return PSM->DONE & PsmStage::all; }

    /// Hold `stages` in reset (FRCE_OFF). A stage below the one this
    /// program runs on stops the machine: the verb exists because
    /// erratum RP2350-E19 is about the register's CONTENTS at a reboot,
    /// and a driver that must guarantee them must be able to read and
    /// write them.
    static void hold(uint32_t stages) { hw_set(PSM->FRCE_OFF, stages & PsmStage::all); }
    static void release(uint32_t stages) { hw_clear(PSM->FRCE_OFF, stages & PsmStage::all); }
    static uint32_t held() { return PSM->FRCE_OFF & PsmStage::all; }
};

} // namespace brio
