/*
 * i2c.hpp
 *
 * The I2C block (RM0444 ch. 32) in the two strata every brio bus driver
 * has (docs/design/i2c-bus.md):
 *
 *   I2c<n>          the RESOURCE: which instance, where its registers
 *                   are, its APB clock and reset, its NVIC line, its
 *                   kernel-clock multiplexer, its DMAMUX request pair,
 *                   its EXTI wake line - and the whole register
 *                   description of the chapter in both roles, including
 *                   the timing arithmetic that is this chapter's hardest
 *                   part and the SMBus half that only some instances
 *                   have;
 *   I2cHost<...>    the TASK util/i2c_bus.hpp drives: one Request is one
 *                   BUS TENURE, with the descriptor shape every
 *                   target's I2cHost carries (docs/design/i2c-bus.md),
 *                   so a device client compiles here untouched;
 *   I2cClient<...>  the target role: address matching, the stretching
 *                   pump, the wake from Stop. A client is a PROTOCOL and
 *                   the protocol is the application's, so this half
 *                   decides nothing (the SpiClient position).
 *
 * SCOPE. The chapter whole, bar what docs/stm32g0/i2c.md declines with a
 * reason: both roles, 7- and 10-bit addressing on both sides, the second
 * own address with its seven mask codes, the general call, all three
 * speeds with the timing arithmetic BOTH WAYS, both filters, the
 * controller's transfer engine with RELOAD/TCR past 255 bytes and
 * AUTOEND against a software STOP, target byte control (SBC), the flags
 * and their W1C clears, every interrupt, the two DMA enables, the wake
 * from Stop with its EXTI line, and the SMBus half - the two device
 * addresses, the alert, the PEC engine and the three time-outs.
 *
 * FACTS THAT SHAPE THE CODE
 *
 *  - THE INSTANCES ARE NOT COPIES OF EACH OTHER (table 165): the
 *    INDEPENDENT CLOCK, the WAKE FROM STOP and the SMBus half belong to
 *    I2C1 always, to I2C2 only on the G0B1/G0C1 class, and to I2C3
 *    nowhere. The reserve derives all three from ONE header probe
 *    (RCC_CCIPR_I2C2SEL_Pos) and says why; the register description
 *    cannot be asked, because every header of the pack declares the same
 *    253 I2C_* macros for the one I2C_TypeDef every instance is mapped
 *    through. The SILICON can be asked, though, and this file gives the
 *    bench the verb: 32.9.6 makes I2C_TIMEOUTR and I2C_PECR "reserved,
 *    and their bits are forced by hardware to 0" on an instance without
 *    SMBus, so smbus_probe() writes and reads back.
 *
 *  - THE TIMING REGISTER IS THE CHAPTER. TIMINGR is five fields that
 *    together set the SCL period, the data hold time and the data setup
 *    time, and 32.4.5 gives each of them an inequality against the I2C
 *    standard's own numbers and the BUS's own edges. So the arithmetic
 *    goes both ways here: i2c_timing_for() SOLVES 32.4.5's conditions
 *    for a register value and i2c_scl_hz() PRICES one, and both are
 *    pinned at the bottom of this file against every cell of tables 172,
 *    173 and 174. Three things about it are worth knowing before reading
 *    the code:
 *      * tSCL IS NOT tSCLL + tSCLH. 32.4.9's own formula adds tSYNC1 +
 *        tSYNC2 - the SCL edge slopes, the filters and two to three
 *        kernel periods of synchronization - and the manual's example
 *        tables charge 250..1000 ns of it. A prediction that ignores it
 *        over-estimates the bus rate by up to 40 %. It is a BUS fact,
 *        not a chip one, so it is an ARGUMENT (I2cBusTiming::sync_ns),
 *        like the rise and fall times beside it.
 *      * SDADEL HAS NO +1 AND THE OTHER THREE DO: tSDADEL = SDADEL x
 *        tPRESC while tSCLDEL, tSCLL and tSCLH are all (field + 1) x
 *        tPRESC (32.9.5). One helper for all four would be wrong for
 *        exactly one of them.
 *      * THE KERNEL CLOCK HAS THREE FLOORS AND ALL THREE BIND: 32.4.3's
 *        tI2CCLK < (tLOW - tfilters)/4 and tI2CCLK < tHIGH; DS13560
 *        table 74's 2 / 9 / 18 MHz; and ES0548 2.10.1's 4 / 10 / 20 MHz,
 *        which is the strictest of the three in every mode. The chooser
 *        refuses below the erratum's floor and below 32.4.3's condition,
 *        and states the datasheet's as a constant so a bench can compare
 *        them.
 *
 *  - ES0548 2.10.2 IS LIVE AND IT COSTS THE VOCABULARY A CODE. In master
 *    mode a BERR can be raised spuriously, "detection of bus error has no
 *    effect on the I2C-bus transfer ... and any such transfer continues
 *    normally", and the workaround is to clear the flag and go on. So the
 *    host engine CLEARS BERR AND COUNTS IT (spurious_bus_errors()) and
 *    NEVER reports i2c_bus_error from it: on this silicon a master's BERR
 *    is not evidence. What the engine does report as i2c_bus_error is its
 *    own bounded wait running out - a tenure the peripheral never
 *    started. A CLIENT's BERR is untouched by the erratum and is
 *    reported.
 *
 *  - THE ONE VERB THAT DISABLES IS 32.4.6's PROCEDURE. Clearing PE
 *    releases both lines, resets the state machines, CR2's START, STOP,
 *    PECBYTE and NACK and every ISR flag - and KEEPS every configuration
 *    register (CR1's other bits, OAR1, OAR2, TIMINGR, TIMEOUTR and the
 *    rest of CR2). The chapter demands PE stay low for at least three APB
 *    cycles and prescribes the sequence write-0 / read-0 / write-1, so
 *    disable() is that sequence and there is no raw PE clear in the file.
 *    It is also the chapter's OWN deadlock escape (32.4.9: "restores the
 *    normal operation ... by toggling the PE bit"), which is why
 *    recover() is built on it.
 *
 *  - EVERY FIELD THE REGISTER DESCRIPTION GATES IS A VERB THAT REFUSES.
 *    The gates are not one rule but four: PE = 0 for TIMINGR (whole
 *    register), NOSTRETCH, ANFOFF, DNF and PECEN; START = 0 for CR2's
 *    SADD, RD_WRN, ADD10, HEAD10R and NBYTES; OA1EN = 0 for OAR1's own
 *    fields and OA2EN = 0 for OAR2's; TIMOUTEN = 0 for TIMEOUTA and TIDLE
 *    and TEXTEN = 0 for TIMEOUTB. Each verb checks ITS OWN gate rather
 *    than one blanket rule, because here - unlike the SPI's - the gates
 *    really differ. Whether the SILICON enforces any of them is a
 *    different question, and the bench asks it directly.
 *
 *  - THE VECTOR IS SHARED where the part has an I2C3: I2C2 and I2C3 sit
 *    on I2C2_3_IRQn (the reserve derives it from I2C3_BASE), so an app
 *    binds BRIO_STM32G0_I2C2_HANDLER and calls both instances' bodies.
 *    I2C1's line is its own everywhere - and it is also EXTI line 23, the
 *    wake, which is why arming the wake needs the EXTI's mask and not
 *    just WUPEN.
 *
 *  - THE WAKE FROM STOP IS TRIPLE-GATED AND THE HARDWARE ENFORCES ONE OF
 *    THE THREE: HSI16 must be the kernel clock (32.4.16), NOSTRETCH must
 *    be clear, and DNF must be 0000 - the last one is a real interlock on
 *    WUPEN and not just a caution. ES0548 2.2.4 adds a fourth from
 *    outside the chapter: a peripheral with clock-request capability does
 *    not wake the device when HSIDIV is anything but 000. wake_from_stop()
 *    checks what it can and states the rest.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/dma_engine.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/i2c_bus.hpp"

namespace brio {

// =============================================================================
// The vocabulary
// =============================================================================

/**
 * The three bus speeds, spelled the way every target's I2cSpeed is, so
 * a Request naming a speed compiles anywhere.
 *
 * THERE IS NO SPEED FIELD IN THIS SILICON: the mode is
 * entirely a matter of what TIMINGR holds (and, for Fm+, of the SYSCFG
 * drive bit the pads need). What the enum selects is which row of the
 * I2C standard the arithmetic below is solved against.
 */
enum class I2cSpeed : uint8_t {
    standard_100k,   ///< Sm, up to 100 kHz
    fast_400k,       ///< Fm, up to 400 kHz
    fast_plus_1m,    ///< Fm+, up to 1 MHz - needs the 20 mA pad drive
};

constexpr uint32_t i2c_speed_hz(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::standard_100k: return 100'000UL;
        case I2cSpeed::fast_400k: return 400'000UL;
        default: return 1'000'000UL;
    }
}

/// RCC_CCIPR's I2CnSEL codes (5.4.21). Code 11 is Reserved and refused.
/// Only an instance with the INDEPENDENT CLOCK has this field at all -
/// every other one runs on PCLK for both its kernel and its registers
/// (32.4.1), which is what makes i2c_has_independent_clock() a real
/// question and not a formality.
enum class I2cClock : uint8_t {
    pclk = 0,
    sysclk = 1,
    hsi16 = 2,   ///< the ONE choice a wake from Stop accepts (32.4.16)
};

constexpr bool i2c_clock_valid(I2cClock c) { return static_cast<uint8_t>(c) <= 2u; }

/// OAR1's OA1MODE / CR2's ADD10: the address width of one endpoint.
enum class I2cAddressMode : uint8_t { seven_bit = 0, ten_bit = 1 };

/**
 * OAR2's OA2MSK[2:0]: how many low bits of the SECOND own address are
 * "don't care" (32.9.4). `none` compares all seven; `low_1` masks OA2[1]
 * so OA2[7:2] are compared; ... ; `all` compares nothing and acknowledges
 * every 7-bit address.
 *
 * WITH ANY MASK BUT `none` THE RESERVED ADDRESSES STOP BEING MATCHED:
 * "as soon as OA2MSK != 0, the reserved I2C addresses (0b0000xxx and
 * 0b1111xxx) are not acknowledged, even if the comparison matches" - a
 * rule with no bit of its own, so it lives in this comment and in the
 * doc rather than in a guard that cannot see the wire.
 */
enum class I2cOa2Mask : uint8_t {
    none = 0, low_1 = 1, low_2 = 2, low_3 = 3,
    low_4 = 4, low_5 = 5, low_6 = 6, all = 7,
};

constexpr bool i2c_oa2_mask_valid(I2cOa2Mask m) { return static_cast<uint8_t>(m) <= 7u; }

/// How many address bits OA2 really compares under a mask - the readback
/// a suite checks a masked match against.
constexpr uint8_t i2c_oa2_compared_bits(I2cOa2Mask m) {
    return static_cast<uint8_t>(7u - static_cast<uint8_t>(m));
}

/// I2C_ISR, by name. Every bit is read-only but TXE and TXIS, which are
/// rs (TXE writable at any time to flush TXDR, TXIS only with NOSTRETCH
/// set). The reset value of the whole register is 0x1: TXE STANDS out of
/// reset and after every PE clear, which is why a transmitter that waits
/// for TXE before its first byte waits for nothing.
struct I2cFlag {
    static constexpr uint32_t txe = I2C_ISR_TXE;
    static constexpr uint32_t txis = I2C_ISR_TXIS;
    static constexpr uint32_t rxne = I2C_ISR_RXNE;
    static constexpr uint32_t addr = I2C_ISR_ADDR;
    static constexpr uint32_t nack = I2C_ISR_NACKF;
    static constexpr uint32_t stop = I2C_ISR_STOPF;
    static constexpr uint32_t transfer_complete = I2C_ISR_TC;
    static constexpr uint32_t transfer_reload = I2C_ISR_TCR;
    static constexpr uint32_t bus_error = I2C_ISR_BERR;
    static constexpr uint32_t arb_lost = I2C_ISR_ARLO;
    static constexpr uint32_t overrun = I2C_ISR_OVR;
    static constexpr uint32_t pec_error = I2C_ISR_PECERR;
    static constexpr uint32_t timeout = I2C_ISR_TIMEOUT;
    static constexpr uint32_t alert = I2C_ISR_ALERT;
    static constexpr uint32_t busy = I2C_ISR_BUSY;
    static constexpr uint32_t dir = I2C_ISR_DIR;
    /// Everything 32.4.17 puts behind ERRIE - six conditions, one enable.
    static constexpr uint32_t errors =
        bus_error | arb_lost | overrun | pec_error | timeout | alert;
};

/// I2C_ICR: the W1C clears, one per clearable flag. THE MASK IS NOT THE
/// ISR'S - ADDRCF, NACKCF and STOPCF sit at the ISR bits' own positions
/// but BERRCF..ALERTCF do too, so one mask of ICR bits is what the clear
/// verbs take. ADDRCF is DUAL-PURPOSE: it also clears CR2.START (32.9.8),
/// which is how a target that was addressed while its own START stood
/// gets out of controller mode.
struct I2cClear {
    static constexpr uint32_t addr = I2C_ICR_ADDRCF;
    static constexpr uint32_t nack = I2C_ICR_NACKCF;
    static constexpr uint32_t stop = I2C_ICR_STOPCF;
    static constexpr uint32_t bus_error = I2C_ICR_BERRCF;
    static constexpr uint32_t arb_lost = I2C_ICR_ARLOCF;
    static constexpr uint32_t overrun = I2C_ICR_OVRCF;
    static constexpr uint32_t pec_error = I2C_ICR_PECCF;
    static constexpr uint32_t timeout = I2C_ICR_TIMOUTCF;
    static constexpr uint32_t alert = I2C_ICR_ALERTCF;
    static constexpr uint32_t errors =
        bus_error | arb_lost | overrun | pec_error | timeout | alert;
    static constexpr uint32_t all = addr | nack | stop | errors;
};

/// CR1's seven interrupt enables (table 181). ERRIE alone covers all six
/// error conditions, which is why isr() hands the caller a masked status
/// rather than a decision.
struct I2cInterrupt {
    static constexpr uint32_t tx = I2C_CR1_TXIE;
    static constexpr uint32_t rx = I2C_CR1_RXIE;
    static constexpr uint32_t addr = I2C_CR1_ADDRIE;
    static constexpr uint32_t nack = I2C_CR1_NACKIE;
    static constexpr uint32_t stop = I2C_CR1_STOPIE;
    static constexpr uint32_t transfer_complete = I2C_CR1_TCIE;
    static constexpr uint32_t error = I2C_CR1_ERRIE;
    static constexpr uint32_t all = tx | rx | addr | nack | stop |
                                    transfer_complete | error;
};

// =============================================================================
// The filters (32.4.5, table 168)
// =============================================================================

/**
 * The two noise filters, which are part of the TIMING and not a
 * decoration: both delay the internal view of SCL and SDA, and both
 * therefore appear in 32.4.5's SDADEL/SCLDEL inequalities and in
 * 32.4.3's condition on the kernel clock.
 *
 * The analog filter is ENABLED AT RESET (ANFOFF = 0) and suppresses
 * spikes of at least 50 ns; its delay is tAF, which DS13560 table 75
 * prices at 50..260 ns. The digital one suppresses DNF kernel-clock
 * periods and its delay is exactly DNF x tI2CCLK - stable where the
 * analog one moves with temperature, voltage and process, which is
 * table 168's whole comparison.
 *
 * THE DIGITAL FILTER AND THE WAKE FROM STOP ARE EXCLUSIVE, and this one
 * the silicon enforces: WUPEN can be set only with DNF = 0000.
 */
struct I2cFilters {
    bool analog = true;    ///< ANFOFF == 0, the reset state
    uint8_t digital = 0;   ///< DNF[3:0], 0..15
};

constexpr bool i2c_filters_valid(const I2cFilters& f) { return f.digital <= 15u; }

/// DS13560 table 75, the two numbers 32.4.5's inequalities need and the
/// manual itself does not carry ("refer to the device datasheet for tAF
/// values").
inline constexpr uint16_t i2c_analog_filter_min_ns = 50;
inline constexpr uint16_t i2c_analog_filter_max_ns = 260;

// =============================================================================
// The bus, as the arithmetic sees it
// =============================================================================

/**
 * What the timing arithmetic needs that is NOT in the chip: the I2C
 * standard's own limits for the mode, plus the SCL detection budget the
 * chapter calls tSYNC1 + tSYNC2.
 *
 * `rise_ns`, `fall_ns` and `setup_ns` are table 169/171's tr(max),
 * tf(max) and tSU;DAT(min) - the values 32.4.5 tells the reader to use
 * "to make the device work reliably regardless of the application",
 * with its own escape: "use the SDA and SCL real transition time values
 * measured in the application to widen the scope of allowed SDADEL and
 * SCLDEL values". A caller with a measured bus states it.
 *
 * `sync_ns` is the one number with no standard behind it. 32.4.9 makes
 * tSCL = tSYNC1 + tSYNC2 + [(SCLH+1) + (SCLL+1)] x (PRESC+1) x tI2CCLK,
 * and the two tSYNC terms are the SCL slopes plus the filter delays plus
 * two to three kernel periods each; its floor is 4 x tI2CCLK, and the
 * example tables 172..174 assume 1000 ns for Sm, 750 for Fm and
 * 250..655 for Fm+. THOSE ARE THE DEFAULTS HERE, so that this file's
 * arithmetic reproduces the manual's own tables, and a caller who has
 * measured its bus overrides them.
 */
struct I2cBusTiming {
    uint16_t rise_ns;
    uint16_t fall_ns;
    uint16_t setup_ns;
    uint16_t sync_ns;
};

/// Table 171 and table 169's columns, plus tables 172..174's own tSYNC
/// assumption for the mode.
constexpr I2cBusTiming i2c_bus_timing(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::standard_100k: return {1000, 300, 250, 1000};
        case I2cSpeed::fast_400k: return {300, 300, 100, 750};
        default: return {120, 120, 50, 500};
    }
}

/// Table 171's tLOW(min) for the mode - what the SCL low period on the
/// WIRE must reach. Note it is NOT what tSCLL must reach: the wire's low
/// time is tSCLL plus the falling-edge detection delay, which is why the
/// manual's own Fm example programs tSCLL = 1250 ns against a tLOW
/// minimum of 1300.
constexpr uint32_t i2c_low_min_ns(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::standard_100k: return 4700;
        case I2cSpeed::fast_400k: return 1300;
        default: return 500;
    }
}
/// Table 171's tHIGH(min).
constexpr uint32_t i2c_high_min_ns(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::standard_100k: return 4000;
        case I2cSpeed::fast_400k: return 600;
        default: return 260;
    }
}
/// Table 169's tVD;DAT(max), the ceiling in SDADEL's upper bound.
constexpr uint32_t i2c_data_valid_max_ns(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::standard_100k: return 3450;
        case I2cSpeed::fast_400k: return 900;
        default: return 450;
    }
}

/**
 * ES0548 2.10.1's kernel-clock floor: with a transmitter honouring the
 * standard's own tSU;DAT the device samples the PREVIOUS SDA value
 * unless the kernel clock period fits inside that setup time, so the
 * erratum names 4 / 10 / 20 MHz for Sm / Fm / Fm+. It has no register
 * workaround, which makes it a REFUSAL in the chooser.
 */
constexpr uint32_t i2c_min_kernel_hz(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::standard_100k: return 4'000'000UL;
        case I2cSpeed::fast_400k: return 10'000'000UL;
        default: return 20'000'000UL;
    }
}

/**
 * DS13560 table 74's floor, which the erratum's beats in every mode
 * (2 / 9 / 18 MHz with the analog filter and DNF = 0; 9 / 16 with the
 * analog filter off and DNF = 1). Stated so a bench can compare the two
 * and a reader can see WHY the driver uses the other one - it is not
 * used by any refusal here.
 */
constexpr uint32_t i2c_datasheet_min_kernel_hz(I2cSpeed s, const I2cFilters& f) {
    const bool analog = f.analog;
    switch (s) {
        case I2cSpeed::standard_100k: return 2'000'000UL;
        case I2cSpeed::fast_400k: return 9'000'000UL;
        default: return analog ? 18'000'000UL : 16'000'000UL;
    }
}

// ---- the small arithmetic helpers -------------------------------------------

/// Kernel-clock cycles in `ns`, ROUNDED UP - the form a bound wants.
/// The intermediate is 64 bits because a long time-out at 64 MHz
/// overflows 32 (the house rule: arithmetic that can exceed the natural
/// width names it).
constexpr uint32_t i2c_ns_cycles_up(uint32_t kernel_hz, uint32_t ns) {
    const uint64_t p = static_cast<uint64_t>(ns) * kernel_hz;
    return static_cast<uint32_t>((p + 999'999'999ULL) / 1'000'000'000ULL);
}
/// The same, rounded DOWN - the form a term that is SUBTRACTED from a
/// bound wants, so that rounding never loosens the bound.
constexpr uint32_t i2c_ns_cycles_down(uint32_t kernel_hz, uint32_t ns) {
    const uint64_t p = static_cast<uint64_t>(ns) * kernel_hz;
    return static_cast<uint32_t>(p / 1'000'000'000ULL);
}
/// The same, rounded to NEAREST - the form a BUDGET wants (tSYNC is an
/// estimate, and rounding it either way is equally honest).
constexpr uint32_t i2c_ns_cycles_near(uint32_t kernel_hz, uint32_t ns) {
    const uint64_t p = static_cast<uint64_t>(ns) * kernel_hz;
    return static_cast<uint32_t>((p + 500'000'000ULL) / 1'000'000'000ULL);
}
constexpr uint32_t i2c_cycles_ns(uint32_t kernel_hz, uint32_t cycles) {
    if (kernel_hz == 0u) {
        return 0;
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(cycles) * 1'000'000'000ULL) /
                                 kernel_hz);
}

/**
 * 32.4.3's own conditions on the kernel clock period, which are separate
 * from the erratum's floor and bind independently:
 *     tI2CCLK < (tLOW - tfilters) / 4     and     tI2CCLK < tHIGH
 * with tfilters the sum of the analog and digital filter delays. The
 * digital filter's is DNF x tI2CCLK, so a large DNF at a fast mode can
 * fail this on its own - which is the point of checking it.
 */
constexpr bool i2c_clock_requirements_met(uint32_t kernel_hz, I2cSpeed s,
                                          const I2cFilters& f) {
    if (kernel_hz == 0u || !i2c_filters_valid(f)) {
        return false;
    }
    // IN PICOSECONDS, and the width is named because it must be: at
    // 20 MHz the two sides of 32.4.3's first condition are 200 and 240
    // nanoseconds apart, and rounding either into whole kernel cycles
    // loses the difference and refuses a clock the chapter allows (the
    // bench found exactly that at the Fm+ floor).
    const uint64_t ps = 1'000'000'000'000ULL / kernel_hz;
    const uint64_t low_ps = static_cast<uint64_t>(i2c_low_min_ns(s)) * 1000ULL;
    const uint64_t high_ps = static_cast<uint64_t>(i2c_high_min_ns(s)) * 1000ULL;
    uint64_t filters_ps = static_cast<uint64_t>(f.digital) * ps;
    if (f.analog) {
        filters_ps += static_cast<uint64_t>(i2c_analog_filter_max_ns) * 1000ULL;
    }
    if (low_ps <= filters_ps) {
        return false;
    }
    // tI2CCLK < (tLOW - tfilters) / 4   and   tI2CCLK < tHIGH
    return (4ULL * ps) < (low_ps - filters_ps) && ps < high_ps;
}

// =============================================================================
// TIMINGR, both ways (32.4.5, 32.4.10, 32.9.5)
// =============================================================================

/**
 * The five fields of I2C_TIMINGR as a value. The times they mean, with
 * tPRESC = (PRESC + 1) x tI2CCLK:
 *
 *     tSCLL   = (SCLL + 1)   x tPRESC     the SCL low  delay
 *     tSCLH   = (SCLH + 1)   x tPRESC     the SCL high delay
 *     tSCLDEL = (SCLDEL + 1) x tPRESC     the data SETUP stretch
 *     tSDADEL =  SDADEL      x tPRESC     the data HOLD delay - NO +1
 */
struct I2cTiming {
    uint8_t presc = 0;    ///< PRESC[3:0]
    uint8_t scll = 0;     ///< SCLL[7:0]
    uint8_t sclh = 0;     ///< SCLH[7:0]
    uint8_t sdadel = 0;   ///< SDADEL[3:0]
    uint8_t scldel = 0;   ///< SCLDEL[3:0]
};

constexpr bool i2c_timing_valid(const I2cTiming& t) {
    return t.presc <= 15u && t.sdadel <= 15u && t.scldel <= 15u;
}

constexpr uint32_t i2c_timingr(const I2cTiming& t) {
    return (static_cast<uint32_t>(t.presc & 0xFu) << I2C_TIMINGR_PRESC_Pos) |
           (static_cast<uint32_t>(t.scldel & 0xFu) << I2C_TIMINGR_SCLDEL_Pos) |
           (static_cast<uint32_t>(t.sdadel & 0xFu) << I2C_TIMINGR_SDADEL_Pos) |
           (static_cast<uint32_t>(t.sclh) << I2C_TIMINGR_SCLH_Pos) |
           (static_cast<uint32_t>(t.scll) << I2C_TIMINGR_SCLL_Pos);
}

constexpr I2cTiming i2c_timing_of(uint32_t reg) {
    return I2cTiming{
        static_cast<uint8_t>((reg & I2C_TIMINGR_PRESC_Msk) >> I2C_TIMINGR_PRESC_Pos),
        static_cast<uint8_t>((reg & I2C_TIMINGR_SCLL_Msk) >> I2C_TIMINGR_SCLL_Pos),
        static_cast<uint8_t>((reg & I2C_TIMINGR_SCLH_Msk) >> I2C_TIMINGR_SCLH_Pos),
        static_cast<uint8_t>((reg & I2C_TIMINGR_SDADEL_Msk) >> I2C_TIMINGR_SDADEL_Pos),
        static_cast<uint8_t>((reg & I2C_TIMINGR_SCLDEL_Msk) >> I2C_TIMINGR_SCLDEL_Pos),
    };
}

/// tSCLL, in kernel-clock cycles.
constexpr uint32_t i2c_scll_cycles(const I2cTiming& t) {
    return (static_cast<uint32_t>(t.scll) + 1u) * (static_cast<uint32_t>(t.presc) + 1u);
}
/// tSCLH, in kernel-clock cycles.
constexpr uint32_t i2c_sclh_cycles(const I2cTiming& t) {
    return (static_cast<uint32_t>(t.sclh) + 1u) * (static_cast<uint32_t>(t.presc) + 1u);
}
/// tSCLDEL, in kernel-clock cycles.
constexpr uint32_t i2c_scldel_cycles(const I2cTiming& t) {
    return (static_cast<uint32_t>(t.scldel) + 1u) * (static_cast<uint32_t>(t.presc) + 1u);
}
/// tSDADEL, in kernel-clock cycles - the field WITHOUT the +1 (32.9.5).
constexpr uint32_t i2c_sdadel_cycles(const I2cTiming& t) {
    return static_cast<uint32_t>(t.sdadel) * (static_cast<uint32_t>(t.presc) + 1u);
}

constexpr uint32_t i2c_scll_ns(uint32_t kernel_hz, const I2cTiming& t) {
    return i2c_cycles_ns(kernel_hz, i2c_scll_cycles(t));
}
constexpr uint32_t i2c_sclh_ns(uint32_t kernel_hz, const I2cTiming& t) {
    return i2c_cycles_ns(kernel_hz, i2c_sclh_cycles(t));
}
constexpr uint32_t i2c_scldel_ns(uint32_t kernel_hz, const I2cTiming& t) {
    return i2c_cycles_ns(kernel_hz, i2c_scldel_cycles(t));
}
constexpr uint32_t i2c_sdadel_ns(uint32_t kernel_hz, const I2cTiming& t) {
    return i2c_cycles_ns(kernel_hz, i2c_sdadel_cycles(t));
}

/**
 * 32.4.8's minimum SCL low stretch after every falling edge, which both
 * roles pay whether or not they have anything to say:
 *     [(SDADEL + SCLDEL + 1) x (PRESC + 1) + 1] x tI2CCLK
 * It is the floor under a target's response time and the reason a
 * generous SDADEL costs a fast bus real throughput.
 */
constexpr uint32_t i2c_min_stretch_cycles(const I2cTiming& t) {
    return (static_cast<uint32_t>(t.sdadel) + static_cast<uint32_t>(t.scldel) + 1u) *
               (static_cast<uint32_t>(t.presc) + 1u) + 1u;
}

/**
 * WHAT A REGISTER VALUE REALLY PRODUCES - 32.4.9's own formula:
 *     tSCL = tSYNC1 + tSYNC2 + [(SCLH + 1) + (SCLL + 1)] x tPRESC
 *
 * `sync_ns` is the caller's tSYNC1 + tSYNC2 budget (see I2cBusTiming).
 * With each example table's own footnoted budget this reproduces every
 * cell of tables 172, 173 and 174 exactly - the static_asserts at the
 * bottom of this file are that check, and they also catch the one cell
 * where the manual's printed tSCL does not follow from its own rows.
 */
constexpr uint32_t i2c_scl_period_cycles(uint32_t kernel_hz, const I2cTiming& t,
                                         uint32_t sync_ns) {
    return i2c_scll_cycles(t) + i2c_sclh_cycles(t) +
           i2c_ns_cycles_near(kernel_hz, sync_ns);
}

constexpr uint32_t i2c_scl_hz(uint32_t kernel_hz, const I2cTiming& t, uint32_t sync_ns) {
    const uint32_t cycles = i2c_scl_period_cycles(kernel_hz, t, sync_ns);
    return cycles == 0u ? 0u : kernel_hz / cycles;
}

/// The same with the mode's own default budget - the everyday form.
constexpr uint32_t i2c_scl_hz(uint32_t kernel_hz, const I2cTiming& t, I2cSpeed s) {
    return i2c_scl_hz(kernel_hz, t, i2c_bus_timing(s).sync_ns);
}

/**
 * The low:high split of the SCL period, per mode - and it is the
 * MANUAL'S OWN, read off tables 172..174 rather than invented: every Sm
 * column programs 20 low against 16 high units, every Fm column 10
 * against 4, and the Fm+ columns are near 2:1. Each satisfies table
 * 171's tLOW/tHIGH minima once the detection delay is added, which is
 * the only sense in which they can satisfy them - see i2c_low_min_ns().
 */
constexpr uint8_t i2c_low_share(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::standard_100k: return 5;
        case I2cSpeed::fast_400k: return 5;
        default: return 2;
    }
}
constexpr uint8_t i2c_high_share(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::standard_100k: return 4;
        case I2cSpeed::fast_400k: return 2;
        default: return 1;
    }
}

/// THE CHOOSER: solve 32.4.5's conditions and 32.4.9's period formula for
/// a register value, at this kernel clock, for this speed, on this bus.
///
/// The method is the chapter's own, in four steps:
///
///  1. The PERIOD in kernel cycles is kernel_hz / f_SCL, less the tSYNC
///     budget; what remains is split between tSCLL and tSCLH in the
///     mode's own ratio (above). Both halves are then rounded UP into
///     prescaler units, so the produced period is never SHORTER than
///     asked: a requested SCL is a CEILING.
///  2. SCLDEL comes from 32.4.5's setup condition, which is a LOWER
///     bound: (SCLDEL + 1) x tPRESC >= tr(max) + tSU;DAT(min).
///  3. SDADEL comes from its hold condition, also a lower bound:
///     SDADEL x tPRESC >= tf(max) + tHD;DAT(min) - tAF(min)
///                        - (DNF + 3) x tI2CCLK
///     with the tAF term present only while the analog filter is on, and
///     the whole right-hand side floored at zero.
///  4. PRESC IS WHAT MAKES 2 AND 3 FIT. SCLDEL and SDADEL are four bits
///     each, so a small prescaler can put the setup delay out of range
///     (at 64 MHz in Sm the setup needs 80 kernel cycles and the field
///     holds sixteen units) - which is exactly why the manual's own
///     tables use a coarse PRESC and small SCLL/SCLH. This function
///     therefore walks PRESC from 0 up and takes the FIRST value at which
///     every field fits, which gives the finest resolution the four-bit
///     delays allow.
///
/// WHAT IT DOES NOT DO is reproduce the manual's own register words: its
/// tables are hand-picked, and for the same times there are several legal
/// (PRESC, SCLL, SCLH) triples. What it reproduces is the TIMES - tSCLL,
/// tSCLH and tSCLDEL come out equal to the tables' at every kernel clock
/// they cover - while SDADEL comes out at the inequality's own minimum
/// where the tables are more generous. Both are stated in the doc and
/// pinned by the static_asserts below.
///
/// Returns nullopt when the kernel clock is below ES0548 2.10.1's floor
/// for this speed, when 32.4.3's own conditions are not met, when the
/// period leaves no room after the tSYNC budget, or when no prescaler
/// makes every field fit. A speed that cannot be produced is REFUSED and
/// never approximated: a bus run at a rate nobody asked for is a fault
/// the caller must see.
constexpr std::optional<I2cTiming> i2c_timing_for(uint32_t kernel_hz, I2cSpeed s,
                                                  const I2cFilters& filters,
                                                  const I2cBusTiming& bus) {
    if (kernel_hz < i2c_min_kernel_hz(s) || !i2c_filters_valid(filters)) {
        return {};
    }
    if (!i2c_clock_requirements_met(kernel_hz, s, filters)) {
        return {};
    }
    const uint32_t total = kernel_hz / i2c_speed_hz(s);
    const uint32_t sync = i2c_ns_cycles_near(kernel_hz, bus.sync_ns);
    if (total < sync + 2u) {
        return {};
    }
    const uint32_t budget = total - sync;
    const uint32_t den = static_cast<uint32_t>(i2c_low_share(s)) + i2c_high_share(s);
    const uint32_t low = (budget * i2c_low_share(s) + den - 1u) / den;
    if (low >= budget) {
        return {};
    }
    const uint32_t high = budget - low;

    // The two delay bounds, in KERNEL cycles (the prescaler divides them
    // below). Both are 32.4.5's, with tHD;DAT(min) = 0 on every non-SMBus
    // row of table 169 and the analog filter's term dropped when it is off.
    const uint32_t setup_cycles = i2c_ns_cycles_up(kernel_hz, bus.rise_ns) +
                                  i2c_ns_cycles_up(kernel_hz, bus.setup_ns);
    uint32_t hold_cycles = i2c_ns_cycles_up(kernel_hz, bus.fall_ns);
    const uint32_t subtract =
        (filters.analog ? i2c_ns_cycles_down(kernel_hz, i2c_analog_filter_min_ns) : 0u) +
        filters.digital + 3u;
    hold_cycles = hold_cycles > subtract ? hold_cycles - subtract : 0u;

    for (uint32_t p = 1; p <= 16u; ++p) {
        const uint32_t scldel_units = (setup_cycles + p - 1u) / p;
        if (scldel_units == 0u || scldel_units > 16u) {
            continue;   // the setup delay does not fit four bits at this prescaler
        }
        const uint32_t sdadel_units = (hold_cycles + p - 1u) / p;
        if (sdadel_units > 15u) {
            continue;
        }
        const uint32_t scll_units = (low + p - 1u) / p;
        const uint32_t sclh_units = (high + p - 1u) / p;
        if (scll_units == 0u || scll_units > 256u || sclh_units == 0u ||
            sclh_units > 256u) {
            continue;
        }
        return I2cTiming{static_cast<uint8_t>(p - 1u),
                         static_cast<uint8_t>(scll_units - 1u),
                         static_cast<uint8_t>(sclh_units - 1u),
                         static_cast<uint8_t>(sdadel_units),
                         static_cast<uint8_t>(scldel_units - 1u)};
    }
    return {};
}

/// The same with the mode's own standard limits and tSYNC budget - what
/// every caller that has not measured its bus wants.
constexpr std::optional<I2cTiming> i2c_timing_for(uint32_t kernel_hz, I2cSpeed s,
                                                  const I2cFilters& filters = {}) {
    return i2c_timing_for(kernel_hz, s, filters, i2c_bus_timing(s));
}

/**
 * Whether a timing value satisfies 32.4.5's SETUP condition at this
 * kernel clock - the one bound that must hold for a receiver to see a
 * settled SDA. Separate from the chooser so a hand-written TIMINGR (the
 * manual's own, CubeMX's) can be judged.
 */
constexpr bool i2c_setup_ok(uint32_t kernel_hz, const I2cTiming& t,
                            const I2cBusTiming& bus) {
    return i2c_scldel_cycles(t) >= i2c_ns_cycles_up(kernel_hz, bus.rise_ns) +
                                       i2c_ns_cycles_up(kernel_hz, bus.setup_ns);
}

/**
 * Whether a timing value satisfies 32.4.5's HOLD condition, lower bound
 * only. The chapter's UPPER bound (SDADEL <= [tVD;DAT(max) - tr(max) -
 * tAF(max) - (DNF+4) x tI2CCLK] / tPRESC) is deliberately NOT a refusal
 * anywhere in this file, because 32.4.5's own note says it "can be
 * violated when NOSTRETCH = 0, because the device stretches SCL low to
 * guarantee the set-up time" - which is every configuration this driver
 * produces. i2c_hold_upper_ok() exists so a bench can ask.
 */
constexpr bool i2c_hold_ok(uint32_t kernel_hz, const I2cTiming& t,
                           const I2cFilters& f, const I2cBusTiming& bus) {
    const uint32_t fall = i2c_ns_cycles_up(kernel_hz, bus.fall_ns);
    const uint32_t sub =
        (f.analog ? i2c_ns_cycles_down(kernel_hz, i2c_analog_filter_min_ns) : 0u) +
        f.digital + 3u;
    const uint32_t need = fall > sub ? fall - sub : 0u;
    return i2c_sdadel_cycles(t) >= need;
}

constexpr bool i2c_hold_upper_ok(uint32_t kernel_hz, const I2cTiming& t,
                                 const I2cFilters& f, const I2cBusTiming& bus,
                                 I2cSpeed s) {
    const uint32_t ceil_ns = i2c_data_valid_max_ns(s);
    const uint32_t budget = i2c_ns_cycles_down(kernel_hz, ceil_ns);
    const uint32_t sub = i2c_ns_cycles_up(kernel_hz, bus.rise_ns) +
                         (f.analog ? i2c_ns_cycles_up(kernel_hz, i2c_analog_filter_max_ns)
                                   : 0u) +
                         f.digital + 4u;
    if (budget <= sub) {
        return false;
    }
    return i2c_sdadel_cycles(t) <= (budget - sub);
}

// =============================================================================
// The SMBus time-outs (32.4.12, tables 175, 177..179)
// =============================================================================

/**
 * TIMEOUTA and TIMEOUTB, both ways. The three periods, all in kernel
 * clocks:
 *
 *     tTIMEOUT  = (TIMEOUTA + 1) x 2048 x tI2CCLK   (TIDLE = 0)
 *     tIDLE     = (TIMEOUTA + 1) x    4 x tI2CCLK   (TIDLE = 1)
 *     tLOW:EXT  = (TIMEOUTB + 1) x 2048 x tI2CCLK
 *
 * AND WHAT EACH ONE WATCHES IS THE POINT, because it is not the same
 * thing: TIMEOUTA with TIDLE = 0 watches SCL held low ON THE WIRE by
 * anybody, which is the only one of the three that can see a PEER's
 * hold; TIMEOUTA with TIDLE = 1 watches both lines high (bus idle
 * detection); and TIMEOUTB watches THIS peripheral's OWN cumulative
 * stretch - tLOW:MEXT in controller mode, tLOW:SEXT in target mode
 * (32.9.6's own field description). A block with only the second kind
 * could police nothing but its own hold; this one has both, and the
 * bench measures them apart.
 *
 * The register's own arithmetic note: 32.9.6 prints TIMEOUTB's formula
 * as "tLOW:EXT = (TIMEOUTB + TIDLE = 01) x 2048 x tI2CCLK", which is
 * mangled - 32.4.12 and table 178 both give (TIMEOUTB + 1) and that is
 * what is implemented.
 */
constexpr uint32_t i2c_timeout_divisor(bool idle) { return idle ? 4u : 2048u; }

/// The code for at least `us` microseconds, or nullopt when twelve bits
/// cannot reach that long at this kernel clock.
constexpr std::optional<uint16_t> i2c_timeout_code_for(uint32_t kernel_hz, uint32_t us,
                                                       bool idle) {
    if (kernel_hz == 0u || us == 0u) {
        return {};
    }
    const uint64_t ticks = static_cast<uint64_t>(us) * kernel_hz / 1'000'000ULL;
    const uint32_t div = i2c_timeout_divisor(idle);
    const uint64_t units = (ticks + div - 1u) / div;
    if (units == 0u || units > 4096u) {
        return {};
    }
    return static_cast<uint16_t>(units - 1u);
}

/// What a code really produces, in microseconds.
constexpr uint32_t i2c_timeout_us(uint32_t kernel_hz, uint16_t code, bool idle) {
    if (kernel_hz == 0u) {
        return 0;
    }
    const uint64_t ticks = (static_cast<uint64_t>(code) + 1u) * i2c_timeout_divisor(idle);
    return static_cast<uint32_t>(ticks * 1'000'000ULL / kernel_hz);
}

/// Table 175's own limits, for a caller sizing an SMBus time-out: the
/// clock-low time-out is 25..35 ms, the target's cumulative extend at
/// most 25 ms and the controller's at most 10 ms.
inline constexpr uint32_t i2c_smbus_timeout_min_us = 25'000;
inline constexpr uint32_t i2c_smbus_timeout_max_us = 35'000;
inline constexpr uint32_t i2c_smbus_low_sext_max_us = 25'000;
inline constexpr uint32_t i2c_smbus_low_mext_max_us = 10'000;

/// The three SMBus addresses the chapter's own enables answer to.
inline constexpr uint8_t i2c_smbus_host_address = 0x08;      ///< SMBHEN
inline constexpr uint8_t i2c_smbus_device_default = 0x61;    ///< SMBDEN
inline constexpr uint8_t i2c_smbus_alert_response = 0x0C;    ///< ALERTEN

// =============================================================================
// Pads
// =============================================================================

/**
 * The two lines, with the AF the DATASHEET gives each signal on each pad
 * (DS13560 tables 13..16). The same shape and the same standing caveat
 * as SpiPins and UartPins: NO HEADER SYMBOL CAN CHECK AN ALTERNATE
 * FUNCTION NUMBER, so what is checked here is that both pads are real
 * pins of this device and are not the same pin; that the AF really
 * carries I2Cn's signal there is the datasheet's claim and the bench's
 * proof.
 *
 * `smba` is the SMBus alert pad and may be left default-constructed - it
 * is claimed only by a program that arms the alert.
 *
 * A NOTE THAT IS THE BOARD'S AND NOT THE CHIP'S: some I2C-capable pads
 * of this family are marked `_s` in DS13560 table 11, "supplied from
 * VDDIO2 only" - PA9..PA12 among them. On a board whose VDDIO2 rail is
 * separate, those pads need that rail up (and PWR_CR2.IOSV set) before
 * they carry anything. Nothing in this header can see a rail; the doc
 * and the suite say so.
 */
struct I2cPins {
    PinSel scl = {};
    PinSel sda = {};
    PinSel smba = {};

    constexpr bool has_alert() const { return smba.valid(); }
};

constexpr bool i2c_pins_collide(const PinSel& a, const PinSel& b) {
    return a.valid() && b.valid() && a.port == b.port && a.pin == b.pin;
}

constexpr bool i2c_pins_valid(const I2cPins& p) {
    if (!p.scl.valid() || !p.sda.valid()) {
        return false;
    }
    const PinSel all[3] = {p.scl, p.sda, p.smba};
    for (uint8_t i = 0; i < 3u; ++i) {
        for (uint8_t j = static_cast<uint8_t>(i + 1u); j < 3u; ++j) {
            if (i2c_pins_collide(all[i], all[j])) {
                return false;
            }
        }
    }
    return true;
}

// =============================================================================
// The configuration
// =============================================================================

/**
 * Everything CR1 holds that is not an interrupt enable or a DMA enable,
 * plus TIMINGR - the thing configure() writes wholesale with PE clear.
 *
 * NOSTRETCH is a TARGET knob and 32.9.1 says in as many words that it
 * "must be kept cleared in controller mode", so i2c_config_valid()
 * refuses the pair rather than letting a host disable a stretch it needs
 * to answer its own address.
 */
struct I2cConfig {
    I2cTiming timing{};
    I2cFilters filters{};
    /// Target only: do not stretch SCL. 32.4.8 states the price - the
    /// data must be in TXDR before the first SCL pulse of its own byte,
    /// or OVR is raised and 0xFF goes out instead.
    bool no_stretch = false;
    /// Target byte control: NBYTES/RELOAD gate each received byte so
    /// software can ACK or NACK it (32.4.8). Incompatible with
    /// no_stretch, and the chapter says so.
    bool byte_control = false;
    bool general_call = false;      ///< GCEN: answer address 0x00
    bool wake_from_stop = false;    ///< WUPEN - needs HSI16 and DNF = 0
    bool smbus_host = false;        ///< SMBHEN: answer the host address
    bool smbus_device = false;      ///< SMBDEN: answer the default address
    bool smbus_alert = false;       ///< ALERTEN: the SMBA pad / ARA
    bool pec = false;               ///< PECEN: the CRC-8 engine
};

constexpr bool i2c_config_valid(const I2cConfig& c) {
    if (!i2c_timing_valid(c.timing) || !i2c_filters_valid(c.filters)) {
        return false;
    }
    // 32.4.8: "The target byte control mode is not compatible with
    // NOSTRETCH mode. Setting SBC when NOSTRETCH = 1 is not allowed."
    if (c.byte_control && c.no_stretch) {
        return false;
    }
    // 32.9.1: WUPEN can be set only with DNF = 0000 - a hardware
    // interlock, refused here so a caller learns it at the verb and not
    // from a bit that would not stick.
    if (c.wake_from_stop && c.filters.digital != 0u) {
        return false;
    }
    // 32.4.16: "Clock stretching must be enabled (NOSTRETCH = 0) to
    // ensure proper operation of the wake-up from Stop mode feature."
    if (c.wake_from_stop && c.no_stretch) {
        return false;
    }
    return true;
}

constexpr uint32_t i2c_cr1(const I2cConfig& c) {
    uint32_t v = 0;
    if (!c.filters.analog) {
        v |= I2C_CR1_ANFOFF;
    }
    v |= static_cast<uint32_t>(c.filters.digital) << I2C_CR1_DNF_Pos;
    if (c.no_stretch) {
        v |= I2C_CR1_NOSTRETCH;
    }
    if (c.byte_control) {
        v |= I2C_CR1_SBC;
    }
    if (c.general_call) {
        v |= I2C_CR1_GCEN;
    }
    if (c.wake_from_stop) {
        v |= I2C_CR1_WUPEN;
    }
    if (c.smbus_host) {
        v |= I2C_CR1_SMBHEN;
    }
    if (c.smbus_device) {
        v |= I2C_CR1_SMBDEN;
    }
    if (c.smbus_alert) {
        v |= I2C_CR1_ALERTEN;
    }
    if (c.pec) {
        v |= I2C_CR1_PECEN;
    }
    return v;
}

/**
 * The target's own addresses - OAR1 and OAR2 together, because they are
 * one decision (which addresses this endpoint answers to) split over two
 * registers with two different write gates.
 */
struct I2cAddressConfig {
    uint16_t own = 0;                             ///< OA1, 7 or 10 bits
    I2cAddressMode mode = I2cAddressMode::seven_bit;
    bool enable = true;                           ///< OA1EN
    uint8_t second = 0;                           ///< OA2, ALWAYS 7 bits
    I2cOa2Mask second_mask = I2cOa2Mask::none;
    bool second_enable = false;                   ///< OA2EN
};

constexpr bool i2c_address_valid(uint16_t addr, I2cAddressMode m) {
    return m == I2cAddressMode::ten_bit ? addr <= 0x3FFu : addr <= 0x7Fu;
}

constexpr bool i2c_address_config_valid(const I2cAddressConfig& c) {
    if (!i2c_address_valid(c.own, c.mode)) {
        return false;
    }
    if (c.second > 0x7Fu || !i2c_oa2_mask_valid(c.second_mask)) {
        return false;
    }
    return true;
}

// =============================================================================
// The resource
// =============================================================================

template <uint8_t n>
struct I2c {
    static_assert(i2c_present(n),
                  "brio I2c: this device has no such I2C instance (the device header "
                  "declares no I2Cn_BASE for it: I2C1 and I2C2 on every STM32G0, I2C3 "
                  "only on the G0B1/G0C1 class)");

    I2c() = delete;

    static constexpr uint8_t index = n;

    /// Table 165's three conditional rows. The reserve derives all three
    /// from ONE header probe and says why; smbus_probe() below is the
    /// silicon's own second opinion.
    static constexpr bool has_independent_clock = i2c_has_independent_clock(n);
    static constexpr bool has_smbus = i2c_has_smbus(n);
    static constexpr bool wakes_from_stop = i2c_wakes_from_stop(n);
    /// Every instance of this family does both address widths and all
    /// three speeds - table 165's unconditional rows, stated as
    /// constants so a bench can compare a table with a register.
    static constexpr bool has_ten_bit = true;
    static constexpr bool has_fast_plus = true;
    /// The EXTI line the wake raises, 0xFF for an instance that cannot.
    static constexpr uint8_t exti_line = i2c_exti_line(n);

    static I2C_TypeDef& regs() { return *reinterpret_cast<I2C_TypeDef*>(i2c_base(n)); }
    static constexpr IRQn_Type irq() { return i2c_irq(n); }

    /// The DMAMUX request ids of this instance (table 56), published BY
    /// THE PERIPHERAL - stm32g0/dma.hpp takes a plain number and knows
    /// nothing about I2Cs.
    static constexpr uint8_t dma_rx_request() { return i2c_dma_rx_request(n); }
    static constexpr uint8_t dma_tx_request() { return i2c_dma_tx_request(n); }
    /// The two data registers, which unlike the SPI's are SEPARATE - a
    /// transmit engine pours into TXDR and a receive one drains RXDR.
    static volatile void* tx_address() { return &regs().TXDR; }
    static volatile void* rx_address() { return &regs().RXDR; }

    // ---- clocks and reset ---------------------------------------------------

    /// The APB clock of the block. Dead registers without it.
    static void bus_clock(bool on) { Rcc::apb1_clock(i2c_bus_clock(n), on); }
    static bool bus_clock() { return Rcc::apb1_clock(i2c_bus_clock(n)); }

    /// Software reset through RCC: every register back to its reset
    /// value. The one way out of a state the chapter has no sequence
    /// for - and note that PE = 0 is NOT that (32.4.6 keeps every
    /// configuration register).
    static void reset() { Rcc::apb1_reset(i2c_bus_clock(n)); }

    /**
     * The kernel-clock multiplexer (RCC_CCIPR). False - with NOTHING
     * written - on an instance without the independent clock, which is
     * I2C3 always and I2C2 below the G0B1: there the kernel runs on
     * PCLK and there is no field to write (32.4.1).
     */
    static bool kernel_clock(I2cClock c) {
        constexpr uint8_t pos = i2c_clock_select_pos(n);
        if constexpr (pos == 0xFF) {
            (void)c;
            return false;
        } else {
            if (!i2c_clock_valid(c)) {
                return false;
            }
            Rcc::kernel_clock(pos, static_cast<uint8_t>(c));
            return true;
        }
    }
    static I2cClock kernel_clock() {
        constexpr uint8_t pos = i2c_clock_select_pos(n);
        if constexpr (pos == 0xFF) {
            return I2cClock::pclk;
        } else {
            return static_cast<I2cClock>(Rcc::kernel_clock(pos) & 0x3u);
        }
    }

    /**
     * What this instance's kernel really runs at, given the app's own
     * PCLK - the number every piece of arithmetic in this file is
     * against. HSI16 is a fixed 16 MHz; PCLK and SYSCLK are the same
     * rate in this stratum, because every Clock task pins HPRE and PPRE
     * at 1 (stm32g0/clock.hpp states the pinning).
     */
    static uint32_t kernel_hz(uint32_t pclk_hz) {
        return kernel_clock() == I2cClock::hsi16 ? 16'000'000UL : pclk_hz;
    }

    // ---- the enable, and 32.4.6's disable PROCEDURE --------------------------

    static bool enabled() { return (regs().CR1 & I2C_CR1_PE) != 0u; }

    static void enable() { regs().CR1 = regs().CR1 | I2C_CR1_PE; }

    /// 32.4.6's RESET, and the only way this driver turns the peripheral
    /// off: write PE = 0, check PE = 0, write PE = 1 is the chapter's own
    /// three-step (PE must stay low for at least three APB cycles), and
    /// this verb is its first two steps.
    ///
    /// WHAT IT DOES: releases SCL and SDA, resets the state machines,
    /// clears CR2's START, STOP, PECBYTE and NACK, and clears every ISR
    /// flag - TXE going back to 1, PECR to 0.
    /// WHAT IT KEEPS: every configuration register, which is CR1's other
    /// bits, OAR1, OAR2, TIMINGR, TIMEOUTR and the rest of CR2. That is
    /// why enable() alone brings the same peripheral back and why
    /// recover() needs no reconfiguration.
    ///
    /// Returns false only if PE would not read back clear (a bus clock
    /// that is off, and nothing else) - an engine that reports is worth
    /// more than one that hangs.
    static bool disable() {
        regs().CR1 = regs().CR1 & ~I2C_CR1_PE;
        uint32_t left = 1000u;
        while ((regs().CR1 & I2C_CR1_PE) != 0u && left-- != 0u) {
        }
        return left != 0u;
    }

    /// The whole of 32.4.6: off, checked, and on again.
    static bool cycle() {
        const bool ok = disable();
        enable();
        return ok;
    }

    // ---- configuration: PE = 0 for all of it --------------------------------

    /**
     * CR1's configuration half and the whole of TIMINGR in one pair of
     * stores. Refused - nothing written - while the instance is enabled
     * or the configuration breaks one of the chapter's own rules.
     *
     * The interrupt enables, the DMA enables and PE itself are NOT here:
     * they live on their own verbs because they are legal under a
     * running peripheral and this one is not.
     */
    static bool configure(const I2cConfig& c) {
        if (enabled() || !i2c_config_valid(c)) {
            return false;
        }
        if (c.wake_from_stop && !wakes_from_stop) {
            return false;   // table 165: this instance has no wake at all
        }
        if ((c.smbus_host || c.smbus_device || c.smbus_alert || c.pec) && !has_smbus) {
            return false;   // ... nor an SMBus half
        }
        const uint32_t keep = regs().CR1 & (I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN |
                                            I2cInterrupt::all);
        regs().CR1 = i2c_cr1(c) | keep;
        regs().TIMINGR = i2c_timingr(c.timing);
        return true;
    }

    /// TIMINGR alone. 32.9.5: "This register must be configured when the
    /// I2C peripheral is disabled (PE = 0)."
    static bool timing(const I2cTiming& t) {
        if (enabled() || !i2c_timing_valid(t)) {
            return false;
        }
        regs().TIMINGR = i2c_timingr(t);
        return true;
    }
    static I2cTiming timing() { return i2c_timing_of(regs().TIMINGR); }
    static uint32_t timing_reg() { return regs().TIMINGR; }

    /// ANFOFF and DNF, which 32.9.1 gates on PE = 0 one by one and
    /// 32.4.5 gates as one ("the filter configuration cannot be changed
    /// when the I2C peripheral is enabled").
    static bool filters(const I2cFilters& f) {
        if (enabled() || !i2c_filters_valid(f)) {
            return false;
        }
        uint32_t v = regs().CR1 & ~(I2C_CR1_ANFOFF | I2C_CR1_DNF);
        if (!f.analog) {
            v |= I2C_CR1_ANFOFF;
        }
        v |= static_cast<uint32_t>(f.digital) << I2C_CR1_DNF_Pos;
        regs().CR1 = v;
        return true;
    }
    static I2cFilters filters() {
        const uint32_t v = regs().CR1;
        return I2cFilters{(v & I2C_CR1_ANFOFF) == 0u,
                          static_cast<uint8_t>((v & I2C_CR1_DNF) >> I2C_CR1_DNF_Pos)};
    }

    /// NOSTRETCH (32.9.1: "can be programmed only when the I2C
    /// peripheral is disabled").
    static bool no_stretch(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR1 = on ? (regs().CR1 | I2C_CR1_NOSTRETCH)
                        : (regs().CR1 & ~I2C_CR1_NOSTRETCH);
        return true;
    }
    static bool no_stretch() { return (regs().CR1 & I2C_CR1_NOSTRETCH) != 0u; }

    /// SBC. 32.4.8 lets this one move under a running peripheral in two
    /// stated windows ("when the target is not addressed, or when
    /// ADDR = 1"), which no register can check - so the verb is open and
    /// the obligation is the caller's, stated here and in the doc.
    static void byte_control(bool on) {
        regs().CR1 = on ? (regs().CR1 | I2C_CR1_SBC) : (regs().CR1 & ~I2C_CR1_SBC);
    }
    static bool byte_control() { return (regs().CR1 & I2C_CR1_SBC) != 0u; }

    /// GCEN: acknowledge the general-call address 0x00.
    static void general_call(bool on) {
        regs().CR1 = on ? (regs().CR1 | I2C_CR1_GCEN) : (regs().CR1 & ~I2C_CR1_GCEN);
    }
    static bool general_call() { return (regs().CR1 & I2C_CR1_GCEN) != 0u; }

    // ---- the target's own addresses -----------------------------------------

    /**
     * OAR1 and OAR2 together. Each register gates its own fields on ITS
     * OWN enable bit (32.9.3/32.9.4: "can be written only when
     * OA1EN = 0" / "OA2EN = 0"), so each is cleared, written and
     * re-enabled here - which is the only sequence that lands.
     */
    static bool addresses(const I2cAddressConfig& c) {
        if (!i2c_address_config_valid(c)) {
            return false;
        }
        regs().OAR1 = 0;   // OA1EN low: the fields open
        uint32_t a1 = c.mode == I2cAddressMode::ten_bit
                          ? (static_cast<uint32_t>(c.own) & I2C_OAR1_OA1_Msk)
                          : (static_cast<uint32_t>(c.own & 0x7Fu) << 1);
        if (c.mode == I2cAddressMode::ten_bit) {
            a1 |= I2C_OAR1_OA1MODE;
        }
        if (c.enable) {
            a1 |= I2C_OAR1_OA1EN;
        }
        regs().OAR1 = a1;

        regs().OAR2 = 0;
        uint32_t a2 = (static_cast<uint32_t>(c.second & 0x7Fu) << 1) |
                      (static_cast<uint32_t>(c.second_mask) << I2C_OAR2_OA2MSK_Pos);
        if (c.second_enable) {
            a2 |= I2C_OAR2_OA2EN;
        }
        regs().OAR2 = a2;
        return true;
    }

    static uint32_t oar1() { return regs().OAR1; }
    static uint32_t oar2() { return regs().OAR2; }

    /// The 7-bit address OA1 answers to (meaningless in 10-bit mode,
    /// where own_address10() is the reading).
    static uint8_t own_address() {
        return static_cast<uint8_t>((regs().OAR1 & I2C_OAR1_OA1_Msk) >> 1) & 0x7Fu;
    }
    static uint16_t own_address10() {
        return static_cast<uint16_t>(regs().OAR1 & I2C_OAR1_OA1_Msk);
    }

    // ---- the controller's transfer engine (32.4.9) --------------------------

    /**
     * One CR2 word: the address, the direction, the byte count and the
     * end policy. Written WITHOUT the START bit, so a caller can set it
     * up and launch separately (which is what a repeated START at TC
     * needs).
     *
     * `reload` is 32.4.9's answer to a transfer longer than 255 bytes:
     * with it set the peripheral raises TCR and stretches SCL when
     * NBYTES runs out, and the next NBYTES write continues the SAME
     * tenure. AUTOEND has no effect while RELOAD is set - the chapter's
     * own caution, and the reason this function refuses the pair rather
     * than writing a bit the silicon ignores.
     */
    static bool transfer(uint16_t addr, bool read, uint8_t nbytes, bool auto_end,
                         bool reload = false,
                         I2cAddressMode mode = I2cAddressMode::seven_bit,
                         bool head10_read_only = false) {
        const std::optional<uint32_t> v =
            cr2_word(addr, read, nbytes, auto_end, reload, mode, head10_read_only);
        if (!v) {
            return false;
        }
        regs().CR2 = *v;
        return true;
    }

    /**
     * THE SAME WORD WITH START IN IT, IN ONE STORE - and this is not a
     * convenience but the only correct way to issue a REPEATED START.
     *
     * 32.9.2 gives START two meanings ("if the I2C is already in
     * controller mode with AUTOEND = 0, setting this bit generates a
     * repeated START condition ... after the end of the NBYTES
     * transfer") and says nothing about the ORDER of the two writes it
     * takes to get there. THE ORDER DECIDES, and the bench measured it:
     * at TC the previous transfer is finished but UNTERMINATED, so a CR2
     * store that raises AUTOEND before START is read as a request to end
     * THAT transfer - the peripheral sends the STOP at once and START,
     * written a cycle later, opens a whole new tenure instead of
     * restarting this one. An ISR trace shows it plainly: TC then STOPF
     * with no second address match at the client, where a repeated START
     * owes two.
     *
     * So the engine uses THIS verb and transfer() is for a caller that
     * means to prepare CR2 and launch later - legal exactly when the bus
     * is idle and there is no unterminated transfer for AUTOEND to act
     * on.
     */
    static bool transfer_now(uint16_t addr, bool read, uint8_t nbytes, bool auto_end,
                             bool reload = false,
                             I2cAddressMode mode = I2cAddressMode::seven_bit,
                             bool head10_read_only = false) {
        const std::optional<uint32_t> v =
            cr2_word(addr, read, nbytes, auto_end, reload, mode, head10_read_only);
        if (!v) {
            return false;
        }
        regs().CR2 = *v | I2C_CR2_START;
        return true;
    }

    /// The CR2 word the two verbs above store, or nullopt when the
    /// chapter refuses it: nothing of CR2 may move while START stands
    /// (32.9.2), an address must fit its mode, and AUTOEND has no effect
    /// under RELOAD so the pair is a caller's mistake and not a state.
    static std::optional<uint32_t> cr2_word(uint16_t addr, bool read, uint8_t nbytes,
                                            bool auto_end, bool reload,
                                            I2cAddressMode mode,
                                            bool head10_read_only) {
        if ((regs().CR2 & I2C_CR2_START) != 0u) {
            return {};
        }
        if (!i2c_address_valid(addr, mode) || (reload && auto_end)) {
            return {};
        }
        uint32_t v = mode == I2cAddressMode::ten_bit
                         ? (static_cast<uint32_t>(addr) & I2C_CR2_SADD_Msk)
                         : (static_cast<uint32_t>(addr & 0x7Fu) << 1);
        if (mode == I2cAddressMode::ten_bit) {
            v |= I2C_CR2_ADD10;
            if (head10_read_only) {
                v |= I2C_CR2_HEAD10R;
            }
        }
        if (read) {
            v |= I2C_CR2_RD_WRN;
        }
        v |= static_cast<uint32_t>(nbytes) << I2C_CR2_NBYTES_Pos;
        if (auto_end) {
            v |= I2C_CR2_AUTOEND;
        }
        if (reload) {
            v |= I2C_CR2_RELOAD;
        }
        return v;
    }

    /// Set START on whatever transfer() left in CR2. A repeated START is
    /// the SAME verb: 32.9.2 makes it one "if the I2C is already in
    /// controller mode with AUTOEND = 0 ... after the end of the NBYTES
    /// transfer", which is exactly the TC state.
    static void start() { regs().CR2 = regs().CR2 | I2C_CR2_START; }
    static bool starting() { return (regs().CR2 & I2C_CR2_START) != 0u; }

    /// A software STOP - what a software-end (AUTOEND = 0) tenure needs
    /// at TC when there is no repeated START to make. Clears TC.
    static void stop() { regs().CR2 = regs().CR2 | I2C_CR2_STOP; }

    /// The next chunk of a RELOAD transfer: writing a non-zero NBYTES
    /// clears TCR and releases the stretch (32.4.9). `reload` false on
    /// the LAST chunk hands the ending back to AUTOEND or to TC.
    static bool reload(uint8_t nbytes, bool more, bool auto_end) {
        if (nbytes == 0u || (more && auto_end)) {
            return false;
        }
        uint32_t v = regs().CR2 & ~(I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND);
        v |= static_cast<uint32_t>(nbytes) << I2C_CR2_NBYTES_Pos;
        if (more) {
            v |= I2C_CR2_RELOAD;
        }
        if (auto_end) {
            v |= I2C_CR2_AUTOEND;
        }
        regs().CR2 = v;
        return true;
    }

    static uint8_t nbytes() {
        return static_cast<uint8_t>((regs().CR2 & I2C_CR2_NBYTES) >> I2C_CR2_NBYTES_Pos);
    }
    static uint32_t cr2() { return regs().CR2; }

    /// CR2.NACK - the TARGET's acknowledge control (32.9.2 says so in as
    /// many words: "used only in target mode"). A controller receiver
    /// NACKs its last byte by itself, whatever this bit holds.
    static void nack_next(bool on) {
        regs().CR2 = on ? (regs().CR2 | I2C_CR2_NACK) : (regs().CR2 & ~I2C_CR2_NACK);
    }

    /// CR2.PECBYTE: transfer the PEC after NBYTES - 1 data bytes. No
    /// effect while RELOAD is set (32.4.12).
    static bool pec_byte(bool on) {
        if (!has_smbus) {
            return false;
        }
        regs().CR2 = on ? (regs().CR2 | I2C_CR2_PECBYTE) : (regs().CR2 & ~I2C_CR2_PECBYTE);
        return true;
    }
    static uint8_t pec() { return static_cast<uint8_t>(regs().PECR & I2C_PECR_PEC_Msk); }

    // ---- data ----------------------------------------------------------------

    [[gnu::always_inline]] static uint8_t data() {
        return static_cast<uint8_t>(regs().RXDR & 0xFFu);
    }
    [[gnu::always_inline]] static void data(uint8_t v) { regs().TXDR = v; }
    /// ISR.TXE is rs: writing 1 FLUSHES a byte that was loaded and is no
    /// longer wanted (32.4.8's own trick for a target that must replace
    /// the first byte of a new tenure).
    static void flush_tx() { regs().ISR = I2C_ISR_TXE; }

    // ---- flags, clears, interrupts ------------------------------------------

    [[gnu::always_inline]] static uint32_t flags() { return regs().ISR; }
    static bool flag(uint32_t mask) { return (regs().ISR & mask) != 0u; }
    /// The W1C clears. THE MASK IS ICR'S (I2cClear), not ISR's.
    static void clear(uint32_t icr_mask) { regs().ICR = icr_mask; }
    static void clear_all() { regs().ICR = I2cClear::all; }

    static bool busy() { return (regs().ISR & I2C_ISR_BUSY) != 0u; }
    /// Valid at an address match: true = the controller is READING, so
    /// this target transmits (32.9.7).
    static bool host_reads() { return (regs().ISR & I2C_ISR_DIR) != 0u; }
    /// The address that matched. In 10-bit mode it is the header plus
    /// the address's two MSBs, not the address (32.9.7).
    static uint8_t matched_address() {
        return static_cast<uint8_t>((regs().ISR & I2C_ISR_ADDCODE) >> I2C_ISR_ADDCODE_Pos);
    }

    static void interrupt(uint32_t mask, bool on) {
        regs().CR1 = on ? (regs().CR1 | mask) : (regs().CR1 & ~mask);
    }
    static uint32_t interrupts() { return regs().CR1 & I2cInterrupt::all; }

    /// The masked pending status: what this instance is really asking
    /// for. Zero when it is not the one asking, which is what makes a
    /// SHARED vector safe (I2C2 and I2C3 sit on one line).
    [[gnu::always_inline]] static uint32_t pending() {
        const uint32_t en = regs().CR1;
        const uint32_t st = regs().ISR;
        uint32_t served = 0;
        if ((en & I2C_CR1_TXIE) != 0u) {
            served |= st & I2C_ISR_TXIS;
        }
        if ((en & I2C_CR1_RXIE) != 0u) {
            served |= st & I2C_ISR_RXNE;
        }
        if ((en & I2C_CR1_ADDRIE) != 0u) {
            served |= st & I2C_ISR_ADDR;
        }
        if ((en & I2C_CR1_NACKIE) != 0u) {
            served |= st & I2C_ISR_NACKF;
        }
        if ((en & I2C_CR1_STOPIE) != 0u) {
            served |= st & I2C_ISR_STOPF;
        }
        if ((en & I2C_CR1_TCIE) != 0u) {
            served |= st & (I2C_ISR_TC | I2C_ISR_TCR);
        }
        if ((en & I2C_CR1_ERRIE) != 0u) {
            served |= st & I2cFlag::errors;
        }
        return served;
    }

    // ---- DMA -----------------------------------------------------------------

    /// TXDMAEN feeds TXDR on TXIS; RXDMAEN drains RXDR on RXNE (32.7).
    /// Neither the address nor the START is a DMA affair, so a DMA
    /// tenure still costs the framing interrupts.
    static void dma_transmit(bool on) {
        regs().CR1 = on ? (regs().CR1 | I2C_CR1_TXDMAEN) : (regs().CR1 & ~I2C_CR1_TXDMAEN);
    }
    static void dma_receive(bool on) {
        regs().CR1 = on ? (regs().CR1 | I2C_CR1_RXDMAEN) : (regs().CR1 & ~I2C_CR1_RXDMAEN);
    }
    static bool dma_transmit() { return (regs().CR1 & I2C_CR1_TXDMAEN) != 0u; }
    static bool dma_receive() { return (regs().CR1 & I2C_CR1_RXDMAEN) != 0u; }

    // ---- the SMBus half (32.4.11..15) ---------------------------------------

    /**
     * TIMEOUTR wholesale. The two halves have DIFFERENT write gates
     * (TIMEOUTA and TIDLE on TIMOUTEN = 0, TIMEOUTB on TEXTEN = 0), so
     * this verb clears both enables first and raises them last, which is
     * the only order in which every field lands.
     *
     * False - nothing written - on an instance without SMBus, where
     * 32.9.6 makes the whole register reserved and forced to zero.
     */
    static bool timeouts(uint16_t code_a, bool idle, bool enable_a, uint16_t code_b,
                         bool enable_b) {
        if (!has_smbus || code_a > 4095u || code_b > 4095u) {
            return false;
        }
        regs().TIMEOUTR = 0;   // both enables low: every field opens
        uint32_t v = static_cast<uint32_t>(code_a) << I2C_TIMEOUTR_TIMEOUTA_Pos;
        if (idle) {
            v |= I2C_TIMEOUTR_TIDLE;
        }
        v |= static_cast<uint32_t>(code_b) << I2C_TIMEOUTR_TIMEOUTB_Pos;
        regs().TIMEOUTR = v;
        if (enable_a) {
            v |= I2C_TIMEOUTR_TIMOUTEN;
        }
        if (enable_b) {
            v |= I2C_TIMEOUTR_TEXTEN;
        }
        regs().TIMEOUTR = v;
        return true;
    }
    static uint32_t timeouts() { return regs().TIMEOUTR; }

    /**
     * THE SILICON'S OWN ANSWER to table 165's SMBus column, and the
     * reason it can be asked at all: 32.9.6 says that on an instance
     * without SMBus "this register is reserved, and its bits are forced
     * by hardware to 0". So a value written into TIMEOUTR either reads
     * back or does not, and that is the peripheral saying which column
     * it is in - a second opinion on a table the device header cannot
     * supply. Leaves the register as it found it.
     */
    static bool smbus_probe() {
        const uint32_t saved = regs().TIMEOUTR;
        regs().TIMEOUTR = 0x0FFFu;   // TIMEOUTA all ones, both enables clear
        const bool answered = (regs().TIMEOUTR & I2C_TIMEOUTR_TIMEOUTA) != 0u;
        regs().TIMEOUTR = saved;
        return answered;
    }

    // ---- the wake from Stop (32.4.16) ---------------------------------------

    /**
     * WUPEN plus its EXTI line. The chapter's three conditions are
     * checked here as far as a register can see them:
     *   - the instance must HAVE the wake (table 165);
     *   - HSI16 must be the kernel clock, or "the HSI16 oscillator does
     *     not start upon receiving START condition";
     *   - DNF must be 0000 (a hardware interlock on WUPEN itself) and
     *     NOSTRETCH must be clear.
     * WHAT IT CANNOT CHECK is ES0548 2.2.4's condition from outside the
     * chapter: a peripheral with clock-request capability does not wake
     * the device at all while RCC's HSIDIV is anything but 000. The
     * caller owes that one, and the doc says so.
     *
     * The EXTI line is DIRECT (table 65): no trigger selection, no
     * pending bit of its own - the peripheral's ADDR match IS the
     * pending state - so all this does is unmask it.
     */
    static bool wake_from_stop(bool on) {
        if constexpr (!wakes_from_stop) {
            (void)on;
            return false;
        } else {
            if (on) {
                if (kernel_clock() != I2cClock::hsi16) {
                    return false;
                }
                const I2cFilters f = filters();
                if (f.digital != 0u || no_stretch()) {
                    return false;
                }
            }
            regs().CR1 = on ? (regs().CR1 | I2C_CR1_WUPEN)
                            : (regs().CR1 & ~I2C_CR1_WUPEN);
            (void)Exti::interrupt(exti_line, on);
            return true;
        }
    }
    static bool wake_from_stop() { return (regs().CR1 & I2C_CR1_WUPEN) != 0u; }

    // ---- the fast-mode-plus pad drive (SYSCFG_CFGR1) ------------------------

    /**
     * The 20 mA drive an Fm+ bus needs, in both of the flavours SYSCFG
     * offers: `instance_fast_plus()` raises every pad configured for
     * this instance, `pad_fast_plus()` raises one named pad that has a
     * bit of its own (PB6, PB7, PB8, PB9, PA9, PA10 - and nothing
     * else). 6.1.3 adds that with Fm+ on, the pad's OSPEEDR speed
     * control is IGNORED.
     *
     * The SYSCFG clock gate is opened here for the same reason
     * vref.hpp's init() and pin.hpp's ucpd_dead_battery() open it: the
     * register is behind it.
     *
     * A COLLISION WORTH KNOWING: PB9's bit is also irtim.hpp's
     * `pb9_high_sink()` - one bit, two uses, and a program driving an
     * infrared LED on PB9 cannot also run an Fm+ bus there.
     */
    static void instance_fast_plus(bool on) {
        constexpr uint32_t bit = i2c_instance_fmp_bit(n);
        if constexpr (bit != 0u) {
            Rcc::apb2_clock(RCC_APBENR2_SYSCFGEN, true);
            SYSCFG->CFGR1 = on ? (SYSCFG->CFGR1 | bit) : (SYSCFG->CFGR1 & ~bit);
        } else {
            (void)on;
        }
    }
    static bool instance_fast_plus() {
        constexpr uint32_t bit = i2c_instance_fmp_bit(n);
        if constexpr (bit != 0u) {
            return (SYSCFG->CFGR1 & bit) != 0u;
        } else {
            return false;
        }
    }
    /// False for a pad with no bit of its own - PA11 and PA12 among
    /// them, where the instance-wide bit is the only route.
    static bool pad_fast_plus(char port, uint8_t pin, bool on) {
        const uint32_t bit = i2c_pad_fmp_bit(port, pin);
        if (bit == 0u) {
            return false;
        }
        Rcc::apb2_clock(RCC_APBENR2_SYSCFGEN, true);
        SYSCFG->CFGR1 = on ? (SYSCFG->CFGR1 | bit) : (SYSCFG->CFGR1 & ~bit);
        return true;
    }

    // ---- the ISR body --------------------------------------------------------

    /// The masked pending status, for an app that owns the protocol
    /// itself. Zero when this instance is not asking - which is what
    /// makes the shared I2C2/I2C3 vector safe.
    [[gnu::always_inline]] static uint32_t isr() { return pending(); }

    static void release() {
        Nvic::disable(irq());
        (void)disable();
        bus_clock(false);
    }
};

// =============================================================================
// The engine slots
// =============================================================================

/*
 * `NoDmaEngine` comes from stm32g0/dma_engine.hpp, for the reason that
 * file states: a driver with an optional engine slot must not include
 * stm32g0/dma.hpp, or every program with a bus would carry the DMA
 * controller. This file reaches its engines only through their own
 * published names - start(), service(), abandon(), stop(), busy(),
 * flag_complete, flag_error - and never spells a DmaChannel.
 */

/// Both engines or neither, and never the same channel twice. UNLIKE THE
/// SPI'S the reason is not a shared completion: an I2C tenure completes
/// on STOPF whichever direction moved. It is that a write-then-read
/// request needs both directions in ONE tenure, so a host with only one
/// engine would have to switch mid-tenure between two pacing mechanisms.
template <typename Tx, typename Rx>
constexpr bool i2c_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::controller != Rx::controller || Tx::channel != Rx::channel;
    } else {
        return true;
    }
}

/// A DMA transfer error as a BusDone status: an engine-defined code
/// bus_master.hpp reserves. i2c_bus.hpp's four wire codes sit at
/// bus_engine_status + 0..3, so this one takes the next.
inline constexpr uint8_t i2c_dma_fault = bus_engine_status + 4;

// =============================================================================
// The host task
// =============================================================================

/**
 * I2cHost<n, pins, TxEngine, RxEngine>
 *
 * The transfer ENGINE util/i2c_bus.hpp drives (which owns arbitration,
 * the pending FIFO, replies and the per-bus timeout). This task owns the
 * wire: the address phase, the repeated START of a write-then-read, the
 * byte pump and the status vocabulary.
 *
 * THE TRANSACTION DESCRIPTOR IS EVERY TARGET'S -
 * {addr, tx span, tx_len, rx span, rx_len, reply, speed} - so a device
 * client written against the contract compiles here untouched. One
 * request is ONE BUS TENURE in the four shapes I2C devices use:
 *
 *   write            tx set, rx_len 0    S addr+W data... P
 *   read             tx_len 0, rx set    S addr+R data... P
 *   write-then-read  both                S addr+W tx... Sr addr+R rx... P
 *   probe            both empty          S addr+W P
 *
 * ALWAYS ASYNCHRONOUS: start() returns false and a TransferDone follows
 * from the ISR - even the probe, whose address phase IS the transaction.
 *
 * THE STATE MACHINE IS 32.4.9's, and the flags do most of it. A write
 * half runs with AUTOEND = 0 when a read half follows, so that TC fires
 * with SCL stretched and setting START there IS the repeated START
 * (32.9.2's own sentence); every other shape runs with AUTOEND = 1 and
 * the hardware sends the STOP. THE COMPLETION EDGE IS STOPF in every
 * case, including the NACK ones - a NACK makes the peripheral send a
 * STOP by itself - which is what makes one exit path serve the whole
 * vocabulary.
 *
 * THE VOCABULARY, PRODUCED ON THE WIRE:
 *   i2c_nack_addr   NACKF with no data byte yet moved in this half
 *   i2c_nack_data   NACKF after at least one data byte
 *   i2c_arb_lost    ARLO - another controller won; the silicon has
 *                   already released the bus and switched to target
 *                   mode, so no STOP is ours to send
 *   i2c_bus_error   the engine's own bounded wait ran out: a tenure the
 *                   peripheral never started
 *   i2c_dma_fault   a DMA channel reported an error mid-tenure
 *
 * AND i2c_bus_error IS NOT BERR, WHICH IS THE ERRATUM'S DOING. ES0548
 * 2.10.2: in master mode a BERR can be raised spuriously and "any such
 * transfer continues normally"; the workaround is to clear it and go on.
 * So this engine clears BERR, COUNTS it (spurious_bus_errors()) and does
 * not let it decide anything. On this silicon a master's BERR is not
 * evidence of a broken wire, and a driver that reported it would report
 * noise.
 *
 * THE TWO OPTIONAL DMA ENGINE SLOTS (the Uart's and the SpiHost's shape,
 * for the same reason: an engineless build must stay byte-identical, so
 * the slots default to NoDmaEngine and every DMA branch folds away under
 * `if constexpr`). With engines named, the DATA of every tenure rides
 * the DMA - the TX channel feeds TXDR on TXIS and the RX channel drains
 * RXDR on RXNE - while the address, the repeated START and the ending
 * stay on the interrupt, because 32.7 says the address "cannot be
 * transferred with DMA". The completion edge is STOPF either way.
 *
 * THE DMA CONTROLLER IS THE APP'S: Dma1::init() before any engined
 * init(). The engines arm CHANNELS of a controller somebody else owns.
 *
 * WHAT A CLOCK SWITCH DOES: TIMINGR is solved against the KERNEL clock,
 * so a port on PCLK or SYSCLK must be re-solved when the rate moves and
 * a port on HSI16 must not (rebase() does both). A transfer in flight is
 * the caller's problem - the bus AO is the natural place to enforce that
 * (it knows when the queue is empty) and util/bus_master.hpp's
 * PrepareSleep voter is the pattern; no engine can enforce it from below,
 * so it is stated and not pretended.
 *
 * ISR wiring (app glue, as usual):
 *   extern "C" void I2C1_IRQHandler() {
 *       if (I2cHw::isr()) { brio::post<I2cBusAo>(brio::TransferDone{I2cHw::status()}); }
 *   }
 */
template <uint8_t n, I2cPins pins, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine>
class I2cHost {
    using S = I2c<n>;

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from stm32g0/dma.hpp, or NoDmaEngine (the default)");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio I2cHost: name both DMA engines or neither - a write-then-read "
                  "tenure needs both directions inside one tenure, and one engine would "
                  "have to hand over to the byte pump mid-transfer");
    static_assert(i2c_engines_distinct<TxEngine, RxEngine>(),
                  "brio I2cHost: the two engines must ride two different DMA channels");
    static_assert(i2c_pins_valid(pins),
                  "brio I2cHost: these I2C pads are not a bus - SCL and SDA must both be "
                  "real pads of this device and no two signals may name the same pin");

    using SclPin = Pin<pins.scl.port, pins.scl.pin>;
    using SdaPin = Pin<pins.sda.port, pins.sda.pin>;

public:
    I2cHost() = delete;

    using Resource = S;
    static constexpr I2cPins pin_pads = pins;
    static constexpr bool has_engines = TxEngine::present;

    struct Request {
        uint8_t addr;          ///< 7-bit client address (unshifted)
        /// Bytes written after START, LENT until the reply lands (may be
        /// null if tx_len == 0).
        Borrowed<const uint8_t, Lease::reply> tx;
        uint8_t tx_len;
        /// Where the bytes read after the (repeated) START+R land; LENT
        /// until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint8_t rx_len;
        ReplyTo<I2cDone> reply;
        I2cSpeed speed = I2cSpeed::standard_100k;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle ----------------------------------------------------------

    /// Bring the instance up as a bus host.
    ///
    /// `clock` is the app's brio::Clock tag; the timing arithmetic is
    /// solved against the KERNEL clock, which is `select`ed here and may
    /// be PCLK, SYSCLK or HSI16 on an instance that has the multiplexer
    /// (I2C1 always, I2C2 on the G0B1/G0C1). HSI16 is worth choosing for
    /// two reasons and the bench uses both: it is the only kernel clock a
    /// wake from Stop accepts, and it keeps the bus alive at a core rate
    /// ES0548 2.10.1 would otherwise refuse - at a 2 MHz core NO speed of
    /// the vocabulary is legal on PCLK, and all three are on HSI16.
    ///
    /// `bus` is the BUS'S own edges and the SCL detection budget: a rise
    /// time the bus does not have lands tSCLL below the specification
    /// floor by the difference. Nullopt (the default) means "the
    /// standard's own numbers and the manual's own tSYNC budget for each
    /// mode", which
    /// reproduces tables 172..174.
    ///
    /// The three speeds' TIMINGR values are resolved here (and at
    /// rebase()); one this kernel clock cannot produce is marked
    /// unreachable - speed_ok() tells, and a Request naming it is
    /// answered i2c_rejected rather than run at a rate nobody asked
    /// for: refused, never silently slowed.
    ///
    /// Returns false when not even standard_100k is reachable, or when
    /// the boot configuration is refused.
    template <typename Clock>
    static bool init(Clock clock, I2cClock kernel = I2cClock::pclk,
                     const I2cFilters& f = {},
                     std::optional<I2cBusTiming> bus = {}) {
        static_assert(clock_follows<Clock, I2cHost>(),
                      "this I2cHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its TIMINGR table would go stale on a "
                      "clock change");
        Nvic::disable(S::irq());
        filters_ = f;
        bus_ = bus;
        S::bus_clock(true);
        S::reset();
        if (!S::kernel_clock(kernel) && kernel != I2cClock::pclk) {
            return false;   // this instance has no multiplexer: PCLK or nothing
        }
        on_hsi16_ = (kernel == I2cClock::hsi16);
        rebase(clock_hz(clock));
        if (!valid_[0]) {
            return false;   // not even a 100 kHz bus is legal at this kernel clock
        }
        applied_ = I2cSpeed::standard_100k;
        I2cConfig c{};
        c.timing = table_[0];
        c.filters = filters_;
        if (!S::configure(c)) {
            return false;
        }
        S::enable();
        if constexpr (has_engines) {
            // The engines are pointed at THIS instance's two data
            // registers and its own DMAMUX request lines. Unlike the
            // SPI's one DR, an I2C has a separate TXDR and RXDR, so the
            // two engines never share an address.
            TxEngine::arm(S::tx_address(), S::dma_tx_request());
            RxEngine::arm(S::rx_address(), S::dma_rx_request());
        }
        // The pads go to the peripheral only now. OPEN DRAIN is not an
        // option here but the definition of the bus: the pull-ups own
        // the idle level and a push-pull driver would fight them.
        SclPin::function(pins.scl.function,
                         {.open_drain = true, .speed = PinSpeed::very_high});
        SdaPin::function(pins.sda.function,
                         {.open_drain = true, .speed = PinSpeed::very_high});
        status_ = i2c_ok;
        phase_ = Phase::idle;
        berr_count_ = 0;
        // TCIE is NOT among them: start() arms it for the one tenure
        // shape that owns a TC and finish() takes it away again.
        S::interrupt(I2cInterrupt::rx | I2cInterrupt::tx | I2cInterrupt::nack |
                         I2cInterrupt::stop | I2cInterrupt::error,
                     true);
        S::interrupt(I2cInterrupt::transfer_complete, false);
        Nvic::enable(S::irq());
        return true;
    }

    /// The core clock changed (DynamicClock fan-out): re-solve the
    /// three speeds' TIMINGR values against the new KERNEL rate.
    ///
    /// A port on HSI16 FOLDS TO NOTHING - its kernel clock did not move -
    /// which is the whole reason a bus can outlive a rate change that
    /// would otherwise refuse it. A speed this kernel clock cannot
    /// produce is marked unreachable, not an error: speed_ok() answers,
    /// and only a request naming it is refused.
    ///
    /// THE BUS MUST BE IDLE: TIMINGR is PE-gated, so applying a new value
    /// costs a PE cycle, and a PE cycle mid-tenure releases both lines.
    ///
    /// VOID, because util/clock.hpp's ClockUser concept says so - the
    /// fan-out has no caller to report to. What a rate change can COST is
    /// asked afterwards, and speed_ok() is the verb: at 2 MHz on PCLK it
    /// answers false for all three speeds, which is the whole finding of
    /// this driver's ladder.
    static void rebase(uint32_t hz) {
        pclk_hz_ = hz;
        ker_hz_ = on_hsi16_ ? 16'000'000UL : hz;
        for (uint8_t i = 0; i < 3u; ++i) {
            const auto s = static_cast<I2cSpeed>(i);
            const auto t = bus_ ? i2c_timing_for(ker_hz_, s, filters_, *bus_)
                                : i2c_timing_for(ker_hz_, s, filters_);
            valid_[i] = t.has_value();
            table_[i] = t.value_or(I2cTiming{});
        }
        cs_rate_ = delay_rate(hz);
        // A rate change under a live bus would leave the registers on the
        // old value; re-apply the one in force if the peripheral is up.
        if (valid_[static_cast<uint8_t>(applied_)] && S::enabled()) {
            const I2cSpeed keep = applied_;
            applied_ = static_cast<I2cSpeed>(0xFFu);   // force apply() to act
            apply(keep);
        }
    }

    /// Can this kernel clock produce that speed at all? (ES0548 2.10.1's
    /// floors drop the fast end of the vocabulary first, and at a 2 MHz
    /// core on PCLK they drop all of it.)
    static bool speed_ok(I2cSpeed s) { return valid_[static_cast<uint8_t>(s)]; }
    /// What a speed really runs at on this bus - the produced rate, not
    /// the asked one.
    static uint32_t scl_hz(I2cSpeed s) {
        return bus_ ? i2c_scl_hz(ker_hz_, table_[static_cast<uint8_t>(s)], bus_->sync_ns)
                    : i2c_scl_hz(ker_hz_, table_[static_cast<uint8_t>(s)], s);
    }
    static I2cTiming timing_of(I2cSpeed s) { return table_[static_cast<uint8_t>(s)]; }
    static uint32_t kernel_hz() { return ker_hz_; }
    static uint32_t reference_hz() { return pclk_hz_; }

    /**
     * The Fm+ pad drive. A bus that runs fast_plus_1m needs it: the
     * 20 mA sink is what makes a 120 ns fall time reachable on a pad at
     * all, and without it a 1 MHz bus is a 1 MHz clock with edges that
     * do not arrive. Raised for the INSTANCE (which covers both pads
     * however they are bonded) and, where the pads have bits of their
     * own, for each pad as well.
     */
    static void fast_plus_drive(bool on) {
        S::instance_fast_plus(on);
        (void)S::pad_fast_plus(pins.scl.port, pins.scl.pin, on);
        (void)S::pad_fast_plus(pins.sda.port, pins.sda.pin, on);
    }
    static bool fast_plus_drive() { return S::instance_fast_plus(); }

    /// The engine is between tenures. BusMaster serializes, so this is a
    /// convenience for suites, not a lock.
    static bool idle() { return phase_ == Phase::idle; }
    /// How many times ES0548 2.10.2's spurious BERR has been swept since
    /// init(). Not a status and not an error - a counter, so a bench can
    /// say how often the erratum fires on this die.
    static uint16_t spurious_bus_errors() { return berr_count_; }

    // ---- the transfer -------------------------------------------------------

    /// Begin one bus tenure (called by I2cBus from main context).
    /// Returns false ALWAYS on success - the tenure runs on the ISR and a
    /// TransferDone{status()} follows - and true only for the degenerate
    /// failure the reply must not wait for: a speed this kernel clock
    /// cannot produce. That is the I2cHost contract
    /// (docs/design/i2c-bus.md).
    static bool start(const Request& r) {
        req_ = r;
        pos_ = 0;
        if (!speed_ok(r.speed)) {
            status_ = i2c_rejected;
            phase_ = Phase::idle;
            return true;
        }
        apply(r.speed);
        status_ = i2c_ok;
        S::clear(I2cClear::all);
        const bool opens_read = (r.tx_len == 0u && r.rx_len != 0u);
        phase_ = opens_read ? Phase::reading : Phase::writing;
        // AUTOEND on every shape but a write that a read half follows:
        // there the software end is what puts TC in our hands, and
        // setting START at TC IS the repeated START (32.9.2).
        const bool two_phase = (r.tx_len != 0u && r.rx_len != 0u);
        const uint8_t nbytes = opens_read ? r.rx_len : r.tx_len;
        // TCIE IS ARMED ONLY FOR THE TENURE THAT NEEDS IT - the
        // write-then-read, whose TC is where the repeated START is
        // issued. Every other shape ends on AUTOEND's own STOP and never
        // raises TC at all, and an armed TCIE with no owner is exactly
        // the storm the idle sweep above cannot clear its way out of.
        S::interrupt(I2cInterrupt::transfer_complete, two_phase);
        if constexpr (has_engines) {
            launch_dma();
        }
        if (!S::transfer_now(r.addr, opens_read, nbytes, !two_phase)) {
            status_ = i2c_bus_error;   // START stood: the peripheral is not ours
            phase_ = Phase::idle;
            return true;
        }
        return false;
    }

    /// The engine's completion status, read by the app glue for the
    /// TransferDone payload.
    static uint8_t status() { return status_; }

    /// The instance's interrupt body - call from its vector.
    /// Returns true when the tenure just completed: the edge on which the
    /// app's glue posts TransferDone to the bus AO.
    [[gnu::always_inline]] static bool isr() {
        const uint32_t p = S::pending();
        if (p == 0u) {
            return false;
        }
        if (phase_ == Phase::idle) {
            // A flag with no tenure owning it. SWEEP IT, or a level
            // holds the NVIC line and the handler storms.
            //
            // AND A SWEEP IS NOT ENOUGH ON THIS PERIPHERAL: TC and TCR
            // HAVE NO ICR BIT. 32.9.8's clear register has one bit for
            // ADDR, NACKF, STOPF and each of the six errors and NOTHING
            // for the two transfer-complete flags, because the chapter
            // clears them by a START, a STOP or an NBYTES write - none
            // of which is legal with no tenure in flight. So what cannot
            // be cleared must be DISARMED, or the level holds the vector
            // for ever.
            S::clear(I2cClear::all);
            // AND RXNE IS NOT IN THE ICR EITHER. 32.9.8's clear register
            // reaches ADDR, NACKF, STOPF and the six errors and nothing
            // else: TXIS, RXNE, TC and TCR are cleared by an ACCESS or by
            // PE. So a byte left in RXDR by a tenure that ended without
            // it - which is exactly what an aborted or recovered tenure
            // leaves - holds this vector for ever unless it is READ.
            if ((S::flags() & I2C_ISR_RXNE) != 0u) {
                (void)S::data();
            }
            if (S::pending() != 0u) {
                S::interrupt(I2cInterrupt::transfer_complete, false);
            }
            return false;
        }

        // ES0548 2.10.2 FIRST, and deliberately before anything else: a
        // spurious BERR in master mode must be cleared and the transfer
        // must go on. Counted, never reported.
        if ((p & I2cFlag::bus_error) != 0u) {
            S::clear(I2cClear::bus_error);
            ++berr_count_;
        }
        if ((p & I2cFlag::arb_lost) != 0u) {
            // The bus is no longer ours: the silicon has released the
            // lines and switched to target mode (32.4.17), so no STOP is
            // ours to send and no STOPF will come.
            S::clear(I2cClear::arb_lost);
            return finish(i2c_arb_lost);
        }
        if ((p & I2cFlag::overrun) != 0u) {
            S::clear(I2cClear::overrun);
        }
        if ((p & I2cFlag::nack) != 0u) {
            // The position tells which byte went unanswered: at pos_ == 0
            // of the opening half nothing has moved, so it was the
            // ADDRESS - the probe's answer and the nobody-home case
            // alike. The peripheral sends the STOP by itself, so this
            // only records and STOPF completes.
            S::clear(I2cClear::nack);
            // pos_ IS THE WHOLE TEST, and it works for both halves of a
            // write-then-read because the repeated START resets it: a
            // NACK with nothing moved in this half is a NACK on the
            // address that opened it.
            status_ = pos_ == 0u ? i2c_nack_addr : i2c_nack_data;
            // AND IT RETURNS. The STOP the peripheral sends by itself
            // arrives as its OWN interrupt a few microseconds later, and
            // a branch that fell through to the sweep at the bottom
            // would clear that STOPF the moment it set - leaving the
            // tenure with its status recorded, its flags gone and no
            // completion for ever. Measured: an address nobody answers
            // never answered at all, but only when NACKF and STOPF
            // happened to arrive separately, which made it look like an
            // order dependency between letters.
            return false;
        }
        // THE DATA COMES BEFORE THE END, and the order is not a taste.
        // RXNE and STOPF can stand TOGETHER - the last byte of a read is
        // in RXDR at the instant the automatic STOP goes out - and RXNE
        // is cleared by READING RXDR and by nothing else. A handler that
        // served STOPF first would lose that byte AND leave RXNE
        // standing on a level-driven vector, which is an endless
        // handler. Both were measured before this order was written.
        if ((p & I2cFlag::rxne) != 0u) {
            const uint8_t v = S::data();
            if (req_.rx.get() != nullptr && pos_ < req_.rx_len) {
                req_.rx.get()[pos_] = v;
            }
            ++pos_;
            return false;
        }
        if ((p & I2cFlag::txis) != 0u) {
            if (pos_ < req_.tx_len && req_.tx.get() != nullptr) {
                S::data(req_.tx.get()[pos_]);
            } else {
                S::data(0xFFu);
            }
            ++pos_;
            return false;
        }
        if ((p & I2cFlag::transfer_complete) != 0u) {
            // The write half is done and a read half follows: the
            // repeated START is the second CR2 write (32.4.9).
            phase_ = Phase::reading;
            pos_ = 0;
            // ONE STORE, START included - see transfer_now(). Two stores
            // put a STOP here instead of the repeated START.
            (void)S::transfer_now(req_.addr, true, req_.rx_len, true);
            return false;
        }
        if ((p & I2cFlag::stop) != 0u) {
            S::clear(I2cClear::stop);
            return finish(status_);
        }
        // THE LAST RESORT, and it is not decoration. Every branch above
        // either clears its own flag or is cleared by the register access
        // it makes; anything that reaches here is a flag this engine did
        // not expect, and on a level-driven vector an unexpected flag is
        // an INFINITE HANDLER - which starves the program so completely
        // that nothing can even report it (measured, twice, while this
        // driver was being written). PECERR, TIMEOUT and ALERT are the
        // ones that can get here: they belong to the SMBus half, they
        // ride the one ERRIE this engine arms for BERR and ARLO, and no
        // branch above owns them. So they are swept; and what a sweep
        // cannot reach is DISARMED.
        // AND THE SWEEP CLEARS ONLY WHAT NO BRANCH ABOVE OWNS. ADDR,
        // NACKF and STOPF belong to the tenure and are cleared where
        // they are served; a blanket ICR write here would take a flag
        // that set between the pending() read and this line and lose the
        // completion with it. What is left over is the SMBus half's
        // three - PECERR, TIMEOUT and ALERT - which ride this engine's
        // one ERRIE and which nothing above claims.
        S::clear(I2cClear::pec_error | I2cClear::timeout | I2cClear::alert);
        if ((S::flags() & I2C_ISR_RXNE) != 0u) {
            (void)S::data();
        }
        if (S::pending() != 0u) {
            S::interrupt(I2cInterrupt::transfer_complete | I2cInterrupt::error, false);
        }
        return false;
    }

    /// The DMA channels' interrupt body - call from whichever
    /// vector each channel reports on. Compiles away on an engineless
    /// host.
    ///
    /// A transfer error on either channel ends the tenure with
    /// i2c_dma_fault and a software STOP, so the bus is released rather
    /// than left to a channel that will never be served again.
    ///
    /// Returns true when the tenure just completed that way.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            const uint8_t tx = TxEngine::service();
            const uint8_t rx = RxEngine::service();
            if ((tx & TxEngine::flag_complete) != 0u) {
                (void)TxEngine::complete();
            }
            // The RECEIVE side needs no completion verb: its block IS
            // the Request's rx span, already filled in place, and the
            // tenure's end is STOPF either way. take() is read only so
            // the engine's own count does not carry into the next
            // tenure.
            if ((rx & RxEngine::flag_complete) != 0u) {
                (void)RxEngine::take();
            }
            if (((tx & TxEngine::flag_error) | (rx & RxEngine::flag_error)) != 0u) {
                if (phase_ != Phase::idle) {
                    (void)TxEngine::abandon();
                    RxEngine::stop();
                    S::stop();
                    return finish(i2c_dma_fault);
                }
            }
        }
        return false;
    }

    /// The classic bus unstick: nine SCL pulses and a STOP, by
    /// hand, open-drain, with the pads reclaimed from the peripheral for
    /// the duration. RECOVER() FIXES THE PERIPHERAL, THIS FIXES THE
    /// WIRE.
    ///
    /// Returns the number of pulses it took a stuck client to release SDA
    /// (0 = the wire was never stuck), or 0xFF when nine pulses and a
    /// STOP left SDA still low - a short, not a client.
    ///
    /// A HEALTHY WIRE IS LEFT ALONE: SDA already high means nothing is
    /// stuck, and zero pulses is both the answer and the action. Pulsing
    /// first and asking after would report "released at pulse 1" on a
    /// clean bus and put nine spurious clocks on it.
    static uint8_t unstick() {
        Nvic::disable(S::irq());
        // The pads back to GPIO, OPEN DRAIN with ODR high: releasing is
        // the pull-ups' job and driving low is ours.
        SclPin::set();
        SdaPin::set();
        SclPin::output(true, {.open_drain = true});
        SdaPin::output(true, {.open_drain = true});

        uint8_t released_at = 0xFF;
        bool free_now = SdaPin::read();
        if (!free_now) {
            uint8_t pulses = 0;
            for (uint8_t i = 0; i < 9u && released_at == 0xFF; ++i) {
                SclPin::clear();
                spin_half_bit();
                SclPin::set();
                spin_half_bit();
                ++pulses;
                if (SdaPin::read()) {
                    released_at = pulses;
                }
            }
            // A STOP: SDA low while SCL is high, then SDA released.
            SdaPin::clear();
            spin_half_bit();
            SclPin::set();
            spin_half_bit();
            SdaPin::set();
            spin_half_bit();
            free_now = SdaPin::read();
        } else {
            released_at = 0;
        }

        SclPin::function(pins.scl.function,
                         {.open_drain = true, .speed = PinSpeed::very_high});
        SdaPin::function(pins.sda.function,
                         {.open_drain = true, .speed = PinSpeed::very_high});
        S::clear(I2cClear::all);
        Nvic::enable(S::irq());
        if (!free_now) {
            return 0xFF;
        }
        return released_at == 0xFF ? 0u : released_at;
    }

    /// Put the ENGINE back where start() is legal - the verb a
    /// timed I2cBus calls on a tenure that never answered
    /// (util/bus_master.hpp).
    ///
    /// It is 32.4.6's PE cycle and nothing more, because 32.4.6 is
    /// exactly what the chapter offers for this: "restores the normal
    /// operation of the I2C peripheral in case of a deadlock, by toggling
    /// the PE bit". The cycle releases both lines, resets the state
    /// machines and clears every flag - and KEEPS every configuration
    /// register, so the timings, the filters and the pads survive it and
    /// nothing is reconfigured.
    ///
    /// THE WIRE IS NOT ITS JOB: a client still holding SDA makes the next
    /// tenure report, and unstick() is the wire's verb.
    ///
    /// Returns false when PE would not read back clear.
    static bool recover() {
        Nvic::disable(S::irq());
        phase_ = Phase::idle;
        if constexpr (has_engines) {
            (void)TxEngine::abandon();
            RxEngine::stop();
            TxEngine::arm(S::tx_address(), S::dma_tx_request());
            RxEngine::arm(S::rx_address(), S::dma_rx_request());
        }
        const bool ok = S::cycle();
        S::clear(I2cClear::all);
        Nvic::enable(S::irq());
        return ok;
    }

    static void release() {
        Nvic::disable(S::irq());
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        S::release();
        SclPin::release();
        SdaPin::release();
        phase_ = Phase::idle;
    }

private:
    enum class Phase : uint8_t { idle, writing, reading };

    /// ~5 us: a 100 kHz half bit, timed by a counted spin because this is
    /// a recovery path that must not depend on any timer being alive.
    static void spin_half_bit() {
        (void)delay_us(cs_rate_, 5);
    }

    /// The one exit of every tenure: the status set, the phase idled, and
    /// every flag swept - a leftover level storms the vector.
    static bool finish(uint8_t st) {
        status_ = st;
        phase_ = Phase::idle;
        // The one interrupt this engine arms per tenure goes away with
        // the tenure (see start()).
        S::interrupt(I2cInterrupt::transfer_complete, false);
        if constexpr (has_engines) {
            (void)TxEngine::abandon();
            RxEngine::stop();
            S::dma_transmit(false);
            S::dma_receive(false);
        }
        S::clear(I2cClear::all);
        if ((S::flags() & I2C_ISR_RXNE) != 0u) {
            (void)S::data();   // an unwanted byte is cleared by reading it
        }
        return true;
    }

    static void launch_dma() {
        if constexpr (has_engines) {
            if (req_.rx_len != 0u && req_.rx.get() != nullptr) {
                (void)RxEngine::start(req_.rx.get(), req_.rx_len);
                S::dma_receive(true);
                S::interrupt(I2cInterrupt::rx, false);
            }
            if (req_.tx_len != 0u && req_.tx.get() != nullptr) {
                (void)TxEngine::start(req_.tx.get(), req_.tx_len);
                S::dma_transmit(true);
                S::interrupt(I2cInterrupt::tx, false);
            }
        }
    }

    /// TIMINGR is PE-gated, so a speed change costs a PE cycle - cached,
    /// so a run of requests at one speed costs nothing (the spi.hpp
    /// apply() shape).
    static void apply(I2cSpeed s) {
        if (s == applied_) {
            return;
        }
        applied_ = s;
        (void)S::disable();
        (void)S::timing(table_[static_cast<uint8_t>(s)]);
        S::enable();
    }

    static inline Request req_{};
    static inline uint8_t pos_ = 0;
    static inline Phase phase_ = Phase::idle;
    static inline uint8_t status_ = i2c_ok;
    static inline uint16_t berr_count_ = 0;
    static inline I2cSpeed applied_ = I2cSpeed::standard_100k;
    static inline I2cTiming table_[3]{};
    static inline bool valid_[3]{};
    static inline I2cFilters filters_{};
    static inline std::optional<I2cBusTiming> bus_{};
    static inline uint32_t pclk_hz_ = 0;
    static inline uint32_t ker_hz_ = 0;
    static inline bool on_hsi16_ = false;
    static inline DelayRate cs_rate_{};
};

// =============================================================================
// The client task
// =============================================================================

/**
 * I2cClient<n, pins>
 *
 * The target role: the polled surface plus the ISR body, deliberately
 * thin - the SpiClient position, because a client is a protocol and the
 * protocol is the application's.
 *
 * WHAT IT ADDS over the raw I2c<n> is the address machinery in one place
 * and the two rules that are easy to get wrong:
 *
 *  - AN ADDRESS MATCH HOLDS THE BUS. With NOSTRETCH clear (the default
 *    and the only mode a wake from Stop accepts) SCL stays low from the
 *    match until ADDR is cleared, so answer_address() is what releases
 *    the wire - and a client that takes its time costs the controller
 *    exactly that.
 *  - WITH NOSTRETCH SET NOTHING WAITS. 32.4.8: the byte must be in TXDR
 *    before the first SCL pulse of its own transfer, or OVR is raised
 *    and 0xFF goes out in its place; on the receive side the byte must
 *    be read before the ninth pulse of the NEXT one. The mode is
 *    offered, its price is stated, and the bench measures it.
 *
 * The wake from Stop is armed here rather than in the resource because
 * it is a CLIENT's feature - only an addressed target wakes - and
 * because arming it means arming an EXTI line as well.
 */
template <uint8_t n, I2cPins pins>
class I2cClient {
    using S = I2c<n>;

    static_assert(i2c_pins_valid(pins),
                  "brio I2cClient: these I2C pads are not a bus - SCL and SDA must both "
                  "be real pads of this device and no two signals may name the same pin");

    using SclPin = Pin<pins.scl.port, pins.scl.pin>;
    using SdaPin = Pin<pins.sda.port, pins.sda.pin>;

public:
    I2cClient() = delete;

    using Resource = S;
    static constexpr I2cPins pin_pads = pins;

    /// Bring the instance up as a bus target.
    ///
    /// A CLIENT NEEDS TIMINGR TOO, and that surprises people: SDADEL and
    /// SCLDEL are the data hold and setup delays a TARGET applies, and
    /// 32.4.8 makes the minimum stretch after every falling edge
    /// [(SDADEL + SCLDEL + 1) x (PRESC + 1) + 1] kernel periods. SCLL and
    /// SCLH are the controller's alone and are ignored here. So `speed`
    /// names the row of the standard the delays are solved against - the
    /// FASTEST bus this target expects to sit on - and not a rate it
    /// generates.
    template <typename Clock>
    static bool init(Clock clock, const I2cAddressConfig& addr,
                     I2cSpeed speed = I2cSpeed::standard_100k,
                     I2cClock kernel = I2cClock::pclk, const I2cFilters& f = {},
                     bool no_stretch = false) {
        Nvic::disable(S::irq());
        S::bus_clock(true);
        S::reset();
        if (!S::kernel_clock(kernel) && kernel != I2cClock::pclk) {
            return false;
        }
        ker_hz_ = kernel == I2cClock::hsi16 ? 16'000'000UL : clock_hz(clock);
        const auto t = i2c_timing_for(ker_hz_, speed, f);
        if (!t) {
            return false;
        }
        I2cConfig c{};
        c.timing = *t;
        c.filters = f;
        c.no_stretch = no_stretch;
        if (!S::configure(c)) {
            return false;
        }
        if (!S::addresses(addr)) {
            return false;
        }
        S::enable();
        SclPin::function(pins.scl.function,
                         {.open_drain = true, .speed = PinSpeed::very_high});
        SdaPin::function(pins.sda.function,
                         {.open_drain = true, .speed = PinSpeed::very_high});
        Nvic::enable(S::irq());
        return true;
    }

    /// The general call, after init() - GCEN is not PE-gated.
    static void general_call(bool on) { S::general_call(on); }

    /// Target byte control: each received byte gated by NBYTES/RELOAD so
    /// software can ACK or NACK it. Needs stretching, and 32.4.8 wants
    /// NBYTES re-armed to 1 after every byte.
    static bool byte_control(bool on) {
        if (on && S::no_stretch()) {
            return false;   // 32.4.8: the two are not compatible
        }
        S::byte_control(on);
        return true;
    }

    // ---- the protocol surface ------------------------------------------------

    /// An address match is pending (SCL held meanwhile, unless
    /// NOSTRETCH).
    static bool addressed() { return S::flag(I2cFlag::addr); }
    /// The matched tenure's direction, valid at ADDR: true = the
    /// controller is reading, so this target transmits.
    static bool host_reads() { return S::host_reads(); }
    /// WHICH address matched (ADDCODE). In 10-bit mode it is the header
    /// plus the address's two MSBs (32.9.7), and a general call reads 0.
    static uint8_t matched_address() { return S::matched_address(); }

    /**
     * Answer the match - and RELEASE THE CLOCK, which is the part that
     * matters: with stretching on, SCL is low from the match until this
     * call.
     *
     * A TARGET CANNOT NACK ITS OWN ADDRESS on this peripheral: the
     * address has already been acknowledged by the time ADDR rises, and
     * the flag is a report rather than a decision point with an ACK bit
     * of its own. Refusing a tenure is therefore a DATA-phase act - NACK
     * the first byte - and this verb's only job is to let the wire go.
     */
    static void answer_address() { S::clear(I2cClear::addr); }

    /// Host write: a byte has arrived.
    static bool data_ready() { return S::flag(I2cFlag::rxne); }
    static uint8_t take() { return S::data(); }
    /// Host read: the shifter wants the next byte.
    static bool data_wanted() { return S::flag(I2cFlag::txis); }
    static void give(uint8_t v) { S::data(v); }
    /// Throw away a byte loaded for a tenure that is over - 32.4.8's own
    /// trick, and the reason a target that reuses TXDR between tenures
    /// does not send yesterday's byte.
    static void flush() { S::flush_tx(); }

    /// Under byte control: acknowledge or refuse the byte just taken,
    /// then release the stretch by re-arming NBYTES.
    static bool answer_byte(bool ack) {
        S::nack_next(!ack);
        return S::reload(1, true, false);
    }

    /// A STOP arrived - the end of a tenure. W1C.
    static bool stop_seen() { return S::flag(I2cFlag::stop); }
    static void clear_stop() { S::clear(I2cClear::stop); }
    /// The controller refused a byte this target transmitted: the tenure
    /// is over and the lines are already released (32.4.8).
    static bool host_nacked() { return S::flag(I2cFlag::nack); }
    static void clear_nack() { S::clear(I2cClear::nack); }
    /// NOSTRETCH mode's own failure: a byte that was not ready in time
    /// (0xFF went out), or one not read before the next arrived.
    static bool overrun() { return S::flag(I2cFlag::overrun); }
    static void clear_overrun() { S::clear(I2cClear::overrun); }

    static uint32_t flags() { return S::flags(); }
    static void clear(uint32_t icr_mask) { S::clear(icr_mask); }

    // ---- the wake from Stop --------------------------------------------------

    /**
     * Arm the address-match wake (32.4.16 + table 65's direct EXTI
     * line). Every condition the register can see is checked by the
     * resource's verb; what remains on the caller is ES0548 2.2.4 -
     * HSIDIV must be 000 or no clock-requesting peripheral wakes this
     * device at all.
     *
     * The chapter's own operating rule, which no driver can enforce: do
     * not enter Stop with a tenure in flight. "Only an ADDR interrupt
     * can wake the device up", so a Stop entered after an address match
     * and before the STOP condition never returns.
     */
    static bool wake_from_stop(bool on) {
        if (on) {
            S::interrupt(I2cInterrupt::addr, true);
        }
        return S::wake_from_stop(on);
    }
    static bool wake_from_stop() { return S::wake_from_stop(); }

    // ---- the ISR body --------------------------------------------------------

    /// The masked pending status (I2cFlag bits), 0 when this instance
    /// was not the one asking. The app's glue decides what to do with
    /// it - a client's protocol is the application's.
    [[gnu::always_inline]] static uint32_t isr() { return S::isr(); }

    static void interrupt(uint32_t mask, bool on) { S::interrupt(mask, on); }

    static void release() {
        Nvic::disable(S::irq());
        S::release();
        SclPin::release();
        SdaPin::release();
    }

private:
    static inline uint32_t ker_hz_ = 0;
};

// =============================================================================
// Compile-time pins of the chapter's own arithmetic
// =============================================================================

// THE READBACK AGAINST TABLES 172, 173 AND 174, cell by cell: every
// example register value, at its own kernel clock, with its own
// footnoted tSYNC budget, must price out at the column's own frequency.
// Sm 10 kHz first (PRESC/SCLL/SCLH = 0x1/0xC7/0xC3 at 8 MHz and so on).
static_assert(i2c_scl_hz(8'000'000UL, I2cTiming{0x1, 0xC7, 0xC3, 2, 4}, 1000u) == 10'000);
static_assert(i2c_scl_hz(16'000'000UL, I2cTiming{0x3, 0xC7, 0xC3, 2, 4}, 1000u) == 10'000);
static_assert(i2c_scl_hz(48'000'000UL, I2cTiming{0xB, 0xC7, 0xC3, 2, 4}, 1000u) == 10'000);
// Sm 100 kHz.
static_assert(i2c_scl_hz(8'000'000UL, I2cTiming{0x1, 0x13, 0xF, 2, 4}, 1000u) == 100'000);
static_assert(i2c_scl_hz(16'000'000UL, I2cTiming{0x3, 0x13, 0xF, 2, 4}, 1000u) == 100'000);
static_assert(i2c_scl_hz(48'000'000UL, I2cTiming{0xB, 0x13, 0xF, 2, 4}, 1000u) == 100'000);
// Fm 400 kHz.
static_assert(i2c_scl_hz(8'000'000UL, I2cTiming{0x0, 0x9, 0x3, 1, 3}, 750u) == 400'000);
static_assert(i2c_scl_hz(16'000'000UL, I2cTiming{0x1, 0x9, 0x3, 2, 3}, 750u) == 400'000);
static_assert(i2c_scl_hz(48'000'000UL, I2cTiming{0x5, 0x9, 0x3, 3, 3}, 750u) == 400'000);
// Fm+ - 500 kHz at 8 MHz (the manual offers no 1 MHz column there),
// 1000 kHz at 16 and 48.
static_assert(i2c_scl_hz(8'000'000UL, I2cTiming{0x0, 0x6, 0x3, 0, 1}, 655u) == 500'000);
static_assert(i2c_scl_hz(16'000'000UL, I2cTiming{0x0, 0x4, 0x2, 0, 2}, 500u) == 1'000'000);
// AND THE ONE CELL WHERE THE MANUAL DISAGREES WITH ITSELF: table 174's
// Fm+ column prints tSCL ~875 ns, but its own rows (tSCLL 500 ns, tSCLH
// 250 ns) and its own footnote (tSYNC = 250 ns) add up to 1000 ns, which
// is the 1000 kHz its column header asks for. The arithmetic follows the
// rows and the footnote.
static_assert(i2c_scl_hz(48'000'000UL, I2cTiming{0x5, 0x3, 0x1, 0, 1}, 250u) == 1'000'000);

// The field times of table 173's Sm 100 kHz column, to the nanosecond.
static_assert(i2c_scll_ns(16'000'000UL, I2cTiming{0x3, 0x13, 0xF, 2, 4}) == 5000);
static_assert(i2c_sclh_ns(16'000'000UL, I2cTiming{0x3, 0x13, 0xF, 2, 4}) == 4000);
static_assert(i2c_scldel_ns(16'000'000UL, I2cTiming{0x3, 0x13, 0xF, 2, 4}) == 1250);
// SDADEL HAS NO +1 and this is where that shows: 2 x 250 ns, not 3.
static_assert(i2c_sdadel_ns(16'000'000UL, I2cTiming{0x3, 0x13, 0xF, 2, 4}) == 500);

// THE CHOOSER at the three rates the bench's own ladder uses, with the
// standard's numbers. Every one lands inside the mode's band and never
// above it.
static_assert(i2c_timing_for(64'000'000UL, I2cSpeed::standard_100k).has_value());
static_assert(i2c_scl_hz(64'000'000UL, *i2c_timing_for(64'000'000UL,
                                                       I2cSpeed::standard_100k),
                         I2cSpeed::standard_100k) <= 100'000);
static_assert(i2c_scl_hz(64'000'000UL, *i2c_timing_for(64'000'000UL,
                                                       I2cSpeed::standard_100k),
                         I2cSpeed::standard_100k) >= 95'000);
static_assert(i2c_scl_hz(64'000'000UL, *i2c_timing_for(64'000'000UL, I2cSpeed::fast_400k),
                         I2cSpeed::fast_400k) == 400'000);
static_assert(i2c_scl_hz(64'000'000UL,
                         *i2c_timing_for(64'000'000UL, I2cSpeed::fast_plus_1m),
                         I2cSpeed::fast_plus_1m) == 1'000'000);
// The chosen values meet 32.4.5's own two conditions.
static_assert(i2c_setup_ok(64'000'000UL, *i2c_timing_for(64'000'000UL, I2cSpeed::fast_400k),
                           i2c_bus_timing(I2cSpeed::fast_400k)));
static_assert(i2c_hold_ok(64'000'000UL, *i2c_timing_for(64'000'000UL, I2cSpeed::fast_400k),
                          I2cFilters{}, i2c_bus_timing(I2cSpeed::fast_400k)));
// And they reproduce the manual's own tSCLL/tSCLH for Fm at 16 MHz.
static_assert(i2c_scll_ns(16'000'000UL, *i2c_timing_for(16'000'000UL,
                                                        I2cSpeed::fast_400k)) == 1250);
static_assert(i2c_sclh_ns(16'000'000UL, *i2c_timing_for(16'000'000UL,
                                                        I2cSpeed::fast_400k)) == 500);

// ES0548 2.10.1's floors, as refusals: 16 MHz cannot do Fm+, and 2 MHz
// cannot do ANY of the three - which is what makes the independent clock
// worth having on a board that scales its core down.
static_assert(!i2c_timing_for(16'000'000UL, I2cSpeed::fast_plus_1m).has_value());
static_assert(i2c_timing_for(16'000'000UL, I2cSpeed::fast_400k).has_value());
static_assert(!i2c_timing_for(2'000'000UL, I2cSpeed::standard_100k).has_value());
static_assert(!i2c_timing_for(2'000'000UL, I2cSpeed::fast_400k).has_value());
static_assert(!i2c_timing_for(2'000'000UL, I2cSpeed::fast_plus_1m).has_value());
// The erratum's floor beats the datasheet's in every mode (table 74).
static_assert(i2c_min_kernel_hz(I2cSpeed::standard_100k) >
              i2c_datasheet_min_kernel_hz(I2cSpeed::standard_100k, I2cFilters{}));
static_assert(i2c_min_kernel_hz(I2cSpeed::fast_400k) >
              i2c_datasheet_min_kernel_hz(I2cSpeed::fast_400k, I2cFilters{}));
static_assert(i2c_min_kernel_hz(I2cSpeed::fast_plus_1m) >
              i2c_datasheet_min_kernel_hz(I2cSpeed::fast_plus_1m, I2cFilters{}));
// A digital filter deep enough to eat the whole low period is refused by
// 32.4.3's own condition and not by an arbitrary bound.
static_assert(!i2c_timing_for(20'000'000UL, I2cSpeed::fast_plus_1m,
                              I2cFilters{true, 15}).has_value());

// THE SMBus TIME-OUTS against tables 177, 178 and 179, cell by cell.
static_assert(*i2c_timeout_code_for(8'000'000UL, 25'000u, false) == 0x61);
static_assert(*i2c_timeout_code_for(16'000'000UL, 25'000u, false) == 0xC3);
static_assert(*i2c_timeout_code_for(48'000'000UL, 25'000u, false) == 0x249);
static_assert(*i2c_timeout_code_for(8'000'000UL, 8'000u, false) == 0x1F);
static_assert(*i2c_timeout_code_for(48'000'000UL, 8'000u, false) == 0xBB);
// Table 178's 16 MHz row is the ONE cell of the three time-out tables
// this arithmetic does not reproduce exactly, and the difference is
// ST's rounding and not an error: 8 ms at 16 MHz needs 62.5 units, so
// the tightest code that covers it is 63 (8.064 ms) while the table
// prints 64 (0x3F, 8.192 ms - and the row's own product is not the 8 ms
// its caption claims either). This chooser gives at least what was
// asked and no more, so it lands one unit below the table.
static_assert(*i2c_timeout_code_for(16'000'000UL, 8'000u, false) == 0x3E);
static_assert(i2c_timeout_us(16'000'000UL, 0x3E, false) >= 8'000);
static_assert(i2c_timeout_us(16'000'000UL, 0x3E, false) <
              i2c_timeout_us(16'000'000UL, 0x3F, false));
static_assert(*i2c_timeout_code_for(8'000'000UL, 50u, true) == 0x63);
static_assert(*i2c_timeout_code_for(16'000'000UL, 50u, true) == 0xC7);
static_assert(*i2c_timeout_code_for(48'000'000UL, 50u, true) == 0x257);
// ... and back: what a code really produces. (The manual's own 48 MHz
// rows use 20.08 ns for a 20.833 ns period, so its printed 25 ms and
// 8 ms are really 24.6 ms and 7.9 ms - this arithmetic says so.)
static_assert(i2c_timeout_us(16'000'000UL, 0xC3, false) == 25'088);
static_assert(i2c_timeout_us(48'000'000UL, 0x249, false) == 25'002);
static_assert(i2c_timeout_us(16'000'000UL, 0xC7, true) == 50);
// Twelve bits do not reach a second at 64 MHz: refused, not truncated.
static_assert(!i2c_timeout_code_for(64'000'000UL, 1'000'000u, false).has_value());

} // namespace brio
