/*
 * fmpi2c.hpp
 *
 * The FAST-MODE PLUS I2C of the STM32F4 (RM0390 ch. 23) - the second and
 * entirely different I2C block some parts of this family carry beside the
 * three of stm32f4/i2c.hpp, in the two layers every brio bus driver has
 * (docs/design/i2c-bus.md):
 *
 *   FmpI2c<n>                  the RESOURCE: the register block, its APB1
 *                              gate and reset, its KERNEL CLOCK
 *                              MULTIPLEXER, its TWO vectors, its DMA
 *                              request pair - and the whole register
 *                              description of the chapter in both roles,
 *                              including the timing arithmetic that is
 *                              this chapter's hardest part and the SMBus
 *                              half with its two time-outs.
 *   FmpI2cHost<n, pins, ...>   the TASK util/i2c_bus.hpp's arbiter drives,
 *                              carrying THE SAME Request as this stratum's
 *                              I2cHost: a program moves from one block to
 *                              the other by changing one type.
 *   FmpI2cClient<n, pins>      the target role, a thin polled surface with
 *                              two ISR bodies - a client is a protocol and
 *                              the protocol is the application's.
 *
 * IT IS THE STM32G0'S I2C UNDER ANOTHER NAME. CR1/CR2 with SADD, NBYTES,
 * RELOAD and AUTOEND, one TIMINGR word, OAR1/OAR2 with the second
 * address's mask, ISR and ICR, PECR, TIMEOUTR, separate RXDR and TXDR -
 * register for register and bit for bit the block brio/stm32g0/i2c.hpp
 * drives. So this file IS that file, with the same verb names and the same
 * shapes, and every place it differs says why. What differs is this
 * family's own: TWO vectors instead of one (RM0390 table 142 splits the
 * events from the errors, as every other peripheral of this stratum does),
 * a kernel-clock multiplexer in RCC_DCKCFGR2 instead of RCC_CCIPR, the
 * Fm+ pad drive in SYSCFG_CFGR instead of SYSCFG_CFGR1, DMA STREAMS
 * instead of channels - and NO WAKE FROM STOP (below).
 *
 * WHY EVERY NAME HERE CARRIES THE Fmp PREFIX. This stratum already has an
 * `I2cSpeed`, an `I2cTiming`, an `I2cPins` and the rest, and they belong
 * to the OTHER block: the F446 carries both, a program may use both at
 * once, and one namespace cannot hold two spellings of one word. The
 * prefix is the price of that, and it is why `FmpI2cHost`'s Request is
 * still the arbiter's own - the descriptor's FIELDS are the contract, and
 * only the speed enum that one of them names is spelled anew.
 *
 * ONE INSTANCE, AND THE INDEX IS THERE FOR THE SECOND. Every part of this
 * family that has this block has exactly one (FMPI2C1), so the reserve
 * publishes its facts without an index and `FmpI2c<n>` static_asserts
 * n == 1. The index exists so the day a part arrives with two, no user of
 * this file is renamed - and so that the host task's parameter list is
 * this stratum's I2cHost's, position for position.
 *
 * THE NAME IS TWO NAMES. ES0298 2.12.10 records it as a documentation
 * erratum: "I2C4 and FMPI2C1 refer to the same peripheral instance", which
 * is why RM0390 6.3.27 calls the kernel-clock selector "I2C4 kernel clock
 * source selection" while every other page of the chapter says FMPI2C1.
 * This file uses the header's name throughout.
 *
 * WHAT TABLE 127 GIVES THIS PART AND WHAT IT WITHHOLDS. The F446's
 * implementation row has 7- and 10-bit addressing, all three speeds, the
 * independent clock and SMBus/PMBus - and a dash against WAKEUP FROM STOP
 * MODE. The device header agrees in the only way it can: there is no
 * FMPI2C_CR1_WUPEN on any header of the pack, and RM0390 23.7.1 makes bit
 * 18 of CR1 reserved. So `wakes_from_stop` is false, `wake_from_stop()`
 * refuses, and the G0's whole triple-gated wake - HSI16, NOSTRETCH, DNF -
 * has nothing to gate here. It is the ONE feature of that chapter this
 * silicon does not have.
 *
 * THE TIMING REGISTER IS THE CHAPTER, and the arithmetic goes both ways:
 * fmpi2c_timing_for() SOLVES 23.4.5's conditions for a register value and
 * fmpi2c_scl_hz() PRICES one, both pinned at the bottom of this file
 * against every cell of tables 134 and 135. Three things about it:
 *   - tSCL IS NOT tSCLL + tSCLH. 23.4.9's own formula adds tSYNC1 + tSYNC2
 *     - the SCL slopes, the filters and two to three kernel periods of
 *     synchronization each - and the manual's example tables charge
 *     500..1000 ns of it. It is a BUS fact and not a chip one, so it is an
 *     argument (FmpI2cBusTiming::sync_ns) beside the rise and fall times.
 *   - SDADEL HAS NO +1 AND THE OTHER THREE DO - and here the manual
 *     disagrees with itself: 23.7.5's field description gives tSDADEL =
 *     SDADEL x tPRESC while 23.4.5's prose gives SDADEL x tPRESC +
 *     tI2CCLK. Tables 134 and 135 print "2 x 250 ns = 500 ns" for SDADEL
 *     0x2 at PRESC 1 and 8 MHz, which is the REGISTER DESCRIPTION's
 *     formula; this file follows the register description and the tables.
 *   - THE KERNEL CLOCK HAS A FLOOR AND IT IS AN ERRATUM'S. ES0298 2.12.2
 *     (the F412's, F413's and F410's errata carry the same item) names
 *     4 / 10 / 20 MHz for Sm / Fm / Fm+, because below them the block
 *     samples the PREVIOUS SDA value; there is no register workaround, so
 *     the chooser REFUSES. 23.4.3's own two conditions are checked beside
 *     it and refuse independently.
 *
 * FOUR MORE ERRATA SHAPE THE CODE (ES0298 2.12, ten items in all):
 *  - 2.12.3, SPURIOUS BUS ERROR IN CONTROLLER MODE: "Detection of bus
 *    error has no effect on the I2C-bus transfer in controller mode and
 *    any such transfer continues normally." So the host CLEARS BERR,
 *    COUNTS it (spurious_bus_errors()) and never reports i2c_bus_error
 *    from it - on this silicon a controller's BERR is not evidence. A
 *    CLIENT's BERR is untouched by the erratum and is reported.
 *  - 2.12.8, TRANSMISSION STALLED AFTER THE FIRST BYTE: when the first
 *    byte is not already in TXDR, the second is written within two kernel
 *    cycles of its request, AND the ratio between the APB clock and the
 *    kernel clock is between 1.5 and 3, the peripheral stalls until PE is
 *    cycled. The ratio is arithmetic this file can do, so it is PUBLISHED
 *    rather than hidden - fmpi2c_transmit_stall_risk() and the host's
 *    transmit_stall_risk() - and the choice of kernel clock, which is the
 *    manual's own second workaround, is the application's. The DMA path
 *    takes the FIRST workaround by construction: the engines are started
 *    before the START bit, so TXDR is already loaded when the address goes
 *    out (23.4.16 asks for that order anyway).
 *  - 2.12.4, LAST-RECEIVED BYTE LOSS IN RELOAD MODE: with RELOAD set and
 *    NBYTES > 1 a receiver can lose the last byte. The host engine never
 *    sets RELOAD - a Request's spans are eight-bit lengths, so one tenure
 *    is at most 255 bytes and NBYTES alone carries it - and the resource's
 *    reload() verb carries the caution.
 *  - 2.12.1, A 10-BIT ADDRESS WHOSE FIRST BYTE IS NACKED wedges the
 *    controller with NACKF and START both standing. The Request is 7-bit,
 *    so the host cannot reach it; the resource publishes the state
 *    (ten_bit_nack_stuck()) and the escape is 23.4.6's PE cycle, which is
 *    cycle().
 * The remaining five are the application's and live in
 * docs/stm32f4/fmpi2c.md.
 *
 * THE ONE VERB THAT DISABLES IS 23.4.6's PROCEDURE. Clearing PE releases
 * both lines, resets the state machines, CR2's START, STOP, NACK and
 * PECBYTE and every status flag - and KEEPS every configuration register.
 * The chapter demands PE stay low for at least three APB cycles and
 * prescribes write-0 / check-0 / write-1, so disable() is its first two
 * steps and there is no raw PE clear in this file.
 *
 * EVERY FIELD THE REGISTER DESCRIPTION GATES IS A VERB THAT REFUSES, and
 * the gates are four and not one: PE = 0 for TIMINGR, NOSTRETCH, ANFOFF
 * and DNF; START = 0 for CR2's SADD, RD_WRN, ADD10, HEAD10R and NBYTES;
 * OA1EN = 0 for OAR1's own fields and OA2EN = 0 for OAR2's; TIMOUTEN = 0
 * for TIMEOUTA and TIDLE and TEXTEN = 0 for TIMEOUTB.
 *
 * THE PADS ARE THE APPLICATION'S, as every bus task of this stratum has
 * them: SCL, SDA and the optional SMBus alert as PinSels, each with the
 * alternate function the DATASHEET gives that signal on that pad - AF4 on
 * every FMPI2C1 pad of the F446 (DS10693 table 11: SCL on PC6, PD12, PD14
 * and PF14; SDA on PC7, PD13, PD15 and PF15; SMBA on PD11 and PF13). No
 * device header carries a pin table, so nothing here can check an AF: the
 * bench is the check. Both lines go out OPEN DRAIN - that IS the bus - and
 * the pull-ups are the board's.
 *
 * NOT COVERED YET is at the end of docs/stm32f4/fmpi2c.md.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/dma_engine.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/syscfg.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/i2c_bus.hpp"

namespace brio {

// =============================================================================
// The vocabulary - the half that names no register
// =============================================================================

/**
 * The three bus speeds. Table 127 gives this block all three, which is
 * what its name is about: the other I2C of this family stops at 400 kHz.
 *
 * THERE IS NO SPEED FIELD IN THIS SILICON: the mode is entirely a matter
 * of what TIMINGR holds, and - for Fm+ - of the 20 mA pad drive SYSCFG
 * offers. What the enum selects is which row of the I2C standard the
 * arithmetic below is solved against.
 */
enum class FmpI2cSpeed : uint8_t {
    standard_100k,   ///< Sm, up to 100 kHz
    fast_400k,       ///< Fm, up to 400 kHz
    fast_plus_1m,    ///< Fm+, up to 1 MHz - wants the 20 mA pad drive
};

inline constexpr uint8_t fmpi2c_speed_count = 3;

constexpr uint32_t fmpi2c_speed_hz(FmpI2cSpeed s) {
    switch (s) {
        case FmpI2cSpeed::standard_100k: return 100'000UL;
        case FmpI2cSpeed::fast_400k: return 400'000UL;
        default: return 1'000'000UL;
    }
}

/**
 * RCC_DCKCFGR2.FMPI2C1SEL's codes (RM0390 6.3.27, where the field is
 * spelled "I2C4 kernel clock source selection" - see ES0298 2.12.10).
 *
 * CODE 3 IS NOT RESERVED HERE, unlike the STM32G0's third code: the manual
 * gives it as "APB clock selected as FMPI2C1 clock (same as 00)", so a
 * readback of 3 is a readback of `pclk` and the reader below normalizes
 * it. The INDEPENDENT CLOCK is what makes this selector worth having: a
 * bus on the HSI keeps its rate through every change of the core's, and a
 * bus on SYSCLK gets a kernel clock far above what an APB prescaler
 * leaves.
 */
enum class FmpI2cClock : uint8_t {
    pclk = 0,     ///< the peripheral's own APB1 clock
    sysclk = 1,
    hsi = 2,      ///< 16 MHz, whatever the core is doing
};

constexpr bool fmpi2c_clock_valid(FmpI2cClock c) { return static_cast<uint8_t>(c) <= 2u; }

/// OAR1's OA1MODE / CR2's ADD10: the address width of one endpoint.
enum class FmpI2cAddressMode : uint8_t { seven_bit = 0, ten_bit = 1 };

/**
 * OAR2's OA2MSK[2:0]: how many low bits of the SECOND own address are
 * "don't care" (23.7.4). `none` compares all seven; `low_1` masks OA2[1]
 * so OA2[7:2] are compared; ... ; `all` acknowledges every 7-bit address.
 *
 * WITH ANY MASK BUT `none` THE RESERVED ADDRESSES STOP BEING MATCHED:
 * 23.7.4 says the reserved I2C addresses (0b0000xxx and 0b1111xxx) "are
 * not acknowledged even if the comparison matches" - a rule with no bit of
 * its own, so it lives in this comment and in the document rather than in
 * a guard that cannot see the wire.
 */
enum class FmpI2cOa2Mask : uint8_t {
    none = 0, low_1 = 1, low_2 = 2, low_3 = 3,
    low_4 = 4, low_5 = 5, low_6 = 6, all = 7,
};

constexpr bool fmpi2c_oa2_mask_valid(FmpI2cOa2Mask m) { return static_cast<uint8_t>(m) <= 7u; }

/// How many address bits OA2 really compares under a mask.
constexpr uint8_t fmpi2c_oa2_compared_bits(FmpI2cOa2Mask m) {
    return static_cast<uint8_t>(7u - static_cast<uint8_t>(m));
}

/**
 * The two noise filters (23.4.5), which are part of the TIMING and not a
 * decoration: both delay the internal view of SCL and SDA, so both appear
 * in 23.4.5's SDADEL/SCLDEL inequalities and in 23.4.3's condition on the
 * kernel clock.
 *
 * The analog filter is ENABLED AT RESET (ANFOFF = 0) and suppresses spikes
 * of at least 50 ns; the digital one suppresses DNF kernel-clock periods
 * and its delay is exactly DNF x tI2CCLK - stable where the analog one
 * moves with temperature, voltage and process, which is table 130's whole
 * comparison. Neither may be changed with the peripheral enabled.
 */
struct FmpI2cFilters {
    bool analog = true;    ///< ANFOFF == 0, the reset state
    uint8_t digital = 0;   ///< DNF[3:0], 0..15
};

constexpr bool fmpi2c_filters_valid(const FmpI2cFilters& f) { return f.digital <= 15u; }

/**
 * The analog filter's delay, tAF, which 23.4.5's inequalities need.
 *
 * THE MAXIMUM IS THE MANUAL'S: 23.4.3 says "Analog filter delay is maximum
 * 260 ns" and 23.4.5's SDA-rising-edge inequality writes the same 260 ns
 * literally. THE MINIMUM IS NOBODY'S: DS10693 has no tAF row at all - only
 * tSP, the width of the spikes the filter suppresses (50..90 ns), which is
 * a different quantity - so it is taken as ZERO here. That is the
 * conservative end of the only inequality tAF(min) appears in, where it is
 * SUBTRACTED from SDADEL's lower bound: a zero makes the required hold
 * delay longer, never shorter. (brio/stm32g0/i2c.hpp takes 50 ns there,
 * because DS13560 states a tAF for that part; this is the same arithmetic
 * with the one number its datasheet does not give.)
 */
inline constexpr uint16_t fmpi2c_analog_filter_min_ns = 0;
inline constexpr uint16_t fmpi2c_analog_filter_max_ns = 260;

/**
 * What the timing arithmetic needs that is NOT in the chip: the I2C
 * standard's own limits for the mode, plus the SCL detection budget the
 * chapter calls tSYNC1 + tSYNC2.
 *
 * `rise_ns`, `fall_ns` and `setup_ns` are table 131's tr(max), tf(max) and
 * tSU;DAT(min) - the values 23.4.5 tells the reader to use "whatever the
 * application", with its own escape: "The SDA and SCL transition time
 * values to be used are the ones in the application." A caller with a
 * measured bus states them.
 *
 * `sync_ns` is the one number with no standard behind it. 23.4.9 makes
 * tSCL = tSYNC1 + tSYNC2 + [(SCLH+1) + (SCLL+1)] x tPRESC, and the two
 * tSYNC terms are the SCL slopes plus the filter delays plus two to three
 * kernel periods each; their floor is 4 x tI2CCLK, and the example tables
 * assume 1000 ns for Sm, 750 for Fm and 500..655 for Fm+. THOSE ARE THE
 * DEFAULTS HERE, so this file's arithmetic reproduces the manual's own
 * tables, and a caller who has measured its bus overrides them.
 */
struct FmpI2cBusTiming {
    uint16_t rise_ns;
    uint16_t fall_ns;
    uint16_t setup_ns;
    uint16_t sync_ns;
};

/// Table 131's columns, with tables 134 and 135's own tSYNC assumption.
constexpr FmpI2cBusTiming fmpi2c_bus_timing(FmpI2cSpeed s) {
    switch (s) {
        case FmpI2cSpeed::standard_100k: return {1000, 300, 250, 1000};
        case FmpI2cSpeed::fast_400k: return {300, 300, 100, 750};
        default: return {120, 120, 50, 500};
    }
}

/// Table 133's tLOW(min) - what the SCL low period on the WIRE must reach.
/// It is NOT what tSCLL must reach: the wire's low time is tSCLL plus the
/// falling-edge detection delay, which is why the manual's own Fm example
/// programs tSCLL = 1250 ns against a tLOW minimum of 1300.
constexpr uint32_t fmpi2c_low_min_ns(FmpI2cSpeed s) {
    switch (s) {
        case FmpI2cSpeed::standard_100k: return 4700;
        case FmpI2cSpeed::fast_400k: return 1300;
        default: return 500;
    }
}
/// Table 133's tHIGH(min).
constexpr uint32_t fmpi2c_high_min_ns(FmpI2cSpeed s) {
    switch (s) {
        case FmpI2cSpeed::standard_100k: return 4000;
        case FmpI2cSpeed::fast_400k: return 600;
        default: return 260;
    }
}
/// Table 131's tVD;DAT(max), the ceiling in SDADEL's upper bound.
constexpr uint32_t fmpi2c_data_valid_max_ns(FmpI2cSpeed s) {
    switch (s) {
        case FmpI2cSpeed::standard_100k: return 3450;
        case FmpI2cSpeed::fast_400k: return 900;
        default: return 450;
    }
}

/**
 * ES0298 2.12.2's kernel-clock floor: with a transmitter honouring the
 * standard's own tSU;DAT the device samples the PREVIOUS SDA value unless
 * the kernel clock period fits inside that setup time, so the erratum
 * names 4 / 10 / 20 MHz for Sm / Fm / Fm+. It has no register workaround -
 * "increase the FMPI2C kernel clock frequency" is the whole of it - which
 * makes it a REFUSAL in the chooser and not a warning.
 */
constexpr uint32_t fmpi2c_min_kernel_hz(FmpI2cSpeed s) {
    switch (s) {
        case FmpI2cSpeed::standard_100k: return 4'000'000UL;
        case FmpI2cSpeed::fast_400k: return 10'000'000UL;
        default: return 20'000'000UL;
    }
}

/**
 * DS10693's own floor on the APB clock, note 2 of the I2C characteristics:
 * 2 MHz for standard mode, 4 MHz for fast mode. It is about fPCLK1 and not
 * about the kernel clock, and the erratum's floors beat it in every mode,
 * so nothing here refuses on it - it is stated as a constant so a bench can
 * compare the two.
 */
constexpr uint32_t fmpi2c_datasheet_min_pclk_hz(FmpI2cSpeed s) {
    return s == FmpI2cSpeed::standard_100k ? 2'000'000UL : 4'000'000UL;
}

/**
 * ES0298 2.12.8: with the first transmit byte not already in TXDR, the
 * second written within two kernel cycles of its request, and THE RATIO
 * BETWEEN THE APB CLOCK AND THE KERNEL CLOCK BETWEEN 1.5 AND 3, the
 * peripheral stalls in a state only a PE cycle or a reset leaves.
 *
 * The ratio is the one part of that a driver can compute, so it is
 * published: true means this pair of rates sits in the erratum's band, and
 * the two workarounds are the manual's own - load TXDR before the transfer
 * starts (which the DMA path does by construction), or choose a kernel
 * clock whose ratio is outside 1.5 .. 3.
 */
constexpr bool fmpi2c_transmit_stall_risk(uint32_t pclk_hz, uint32_t kernel_hz) {
    if (pclk_hz == 0u || kernel_hz == 0u) {
        return false;
    }
    // 1.5 <= pclk / kernel <= 3, in integers: the widths are named because
    // 3 x 180 MHz is past what a 16-bit accumulator holds and close enough
    // to 32 bits to be worth saying.
    const uint64_t p = pclk_hz;
    const uint64_t k = kernel_hz;
    return (2ULL * p >= 3ULL * k) && (p <= 3ULL * k);
}

// ---- the small arithmetic helpers -------------------------------------------

/// Kernel-clock cycles in `ns`, ROUNDED UP - the form a bound wants. The
/// intermediate is 64 bits because a long time-out at 180 MHz overflows 32
/// (the house rule: arithmetic that can exceed the natural width names it).
constexpr uint32_t fmpi2c_ns_cycles_up(uint32_t kernel_hz, uint32_t ns) {
    const uint64_t p = static_cast<uint64_t>(ns) * kernel_hz;
    return static_cast<uint32_t>((p + 999'999'999ULL) / 1'000'000'000ULL);
}
/// The same, rounded DOWN - the form a term that is SUBTRACTED from a bound
/// wants, so that rounding never loosens the bound.
constexpr uint32_t fmpi2c_ns_cycles_down(uint32_t kernel_hz, uint32_t ns) {
    const uint64_t p = static_cast<uint64_t>(ns) * kernel_hz;
    return static_cast<uint32_t>(p / 1'000'000'000ULL);
}
/// The same, rounded to NEAREST - the form a BUDGET wants (tSYNC is an
/// estimate, and rounding it either way is equally honest).
constexpr uint32_t fmpi2c_ns_cycles_near(uint32_t kernel_hz, uint32_t ns) {
    const uint64_t p = static_cast<uint64_t>(ns) * kernel_hz;
    return static_cast<uint32_t>((p + 500'000'000ULL) / 1'000'000'000ULL);
}
constexpr uint32_t fmpi2c_cycles_ns(uint32_t kernel_hz, uint32_t cycles) {
    if (kernel_hz == 0u) {
        return 0;
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(cycles) * 1'000'000'000ULL) / kernel_hz);
}

/**
 * 23.4.3's own conditions on the kernel clock period, which are separate
 * from the erratum's floor and bind independently:
 *     tI2CCLK < (tLOW - tfilters) / 4     and     tI2CCLK < tHIGH
 * with tfilters the sum of the analog and digital filter delays. The
 * digital filter's is DNF x tI2CCLK, so a large DNF at a fast mode can fail
 * this on its own - which is the point of checking it.
 */
constexpr bool fmpi2c_clock_requirements_met(uint32_t kernel_hz, FmpI2cSpeed s,
                                             const FmpI2cFilters& f) {
    if (kernel_hz == 0u || !fmpi2c_filters_valid(f)) {
        return false;
    }
    // IN PICOSECONDS, and the width is named because it must be: at the
    // Fm+ floor the two sides are two hundred-odd nanoseconds apart, and
    // rounding either into whole kernel cycles loses the difference and
    // refuses a clock the chapter allows.
    const uint64_t ps = 1'000'000'000'000ULL / kernel_hz;
    const uint64_t low_ps = static_cast<uint64_t>(fmpi2c_low_min_ns(s)) * 1000ULL;
    const uint64_t high_ps = static_cast<uint64_t>(fmpi2c_high_min_ns(s)) * 1000ULL;
    uint64_t filters_ps = static_cast<uint64_t>(f.digital) * ps;
    if (f.analog) {
        filters_ps += static_cast<uint64_t>(fmpi2c_analog_filter_max_ns) * 1000ULL;
    }
    if (low_ps <= filters_ps) {
        return false;
    }
    return (4ULL * ps) < (low_ps - filters_ps) && ps < high_ps;
}

// =============================================================================
// TIMINGR, both ways (23.4.5, 23.4.9, 23.7.5)
// =============================================================================

/**
 * The five fields of FMPI2C_TIMINGR as a value. The times they mean, with
 * tPRESC = (PRESC + 1) x tI2CCLK:
 *
 *     tSCLL   = (SCLL + 1)   x tPRESC     the SCL low  delay
 *     tSCLH   = (SCLH + 1)   x tPRESC     the SCL high delay
 *     tSCLDEL = (SCLDEL + 1) x tPRESC     the data SETUP stretch
 *     tSDADEL =  SDADEL      x tPRESC     the data HOLD delay - NO +1
 */
struct FmpI2cTiming {
    uint8_t presc = 0;    ///< PRESC[3:0]
    uint8_t scll = 0;     ///< SCLL[7:0]
    uint8_t sclh = 0;     ///< SCLH[7:0]
    uint8_t sdadel = 0;   ///< SDADEL[3:0]
    uint8_t scldel = 0;   ///< SCLDEL[3:0]
};

constexpr bool fmpi2c_timing_valid(const FmpI2cTiming& t) {
    return t.presc <= 15u && t.sdadel <= 15u && t.scldel <= 15u;
}

/// tSCLL, in kernel-clock cycles.
constexpr uint32_t fmpi2c_scll_cycles(const FmpI2cTiming& t) {
    return (static_cast<uint32_t>(t.scll) + 1u) * (static_cast<uint32_t>(t.presc) + 1u);
}
/// tSCLH, in kernel-clock cycles.
constexpr uint32_t fmpi2c_sclh_cycles(const FmpI2cTiming& t) {
    return (static_cast<uint32_t>(t.sclh) + 1u) * (static_cast<uint32_t>(t.presc) + 1u);
}
/// tSCLDEL, in kernel-clock cycles.
constexpr uint32_t fmpi2c_scldel_cycles(const FmpI2cTiming& t) {
    return (static_cast<uint32_t>(t.scldel) + 1u) * (static_cast<uint32_t>(t.presc) + 1u);
}
/// tSDADEL, in kernel-clock cycles - the field WITHOUT the +1 (23.7.5, and
/// tables 134 and 135's own printed values).
constexpr uint32_t fmpi2c_sdadel_cycles(const FmpI2cTiming& t) {
    return static_cast<uint32_t>(t.sdadel) * (static_cast<uint32_t>(t.presc) + 1u);
}

constexpr uint32_t fmpi2c_scll_ns(uint32_t kernel_hz, const FmpI2cTiming& t) {
    return fmpi2c_cycles_ns(kernel_hz, fmpi2c_scll_cycles(t));
}
constexpr uint32_t fmpi2c_sclh_ns(uint32_t kernel_hz, const FmpI2cTiming& t) {
    return fmpi2c_cycles_ns(kernel_hz, fmpi2c_sclh_cycles(t));
}
constexpr uint32_t fmpi2c_scldel_ns(uint32_t kernel_hz, const FmpI2cTiming& t) {
    return fmpi2c_cycles_ns(kernel_hz, fmpi2c_scldel_cycles(t));
}
constexpr uint32_t fmpi2c_sdadel_ns(uint32_t kernel_hz, const FmpI2cTiming& t) {
    return fmpi2c_cycles_ns(kernel_hz, fmpi2c_sdadel_cycles(t));
}

/**
 * 23.4.5's minimum SCL low stretch after every falling edge, which both
 * roles pay whether or not they have anything to say:
 *     [(SDADEL + SCLDEL + 1) x (PRESC + 1) + 1] x tI2CCLK
 * It is the floor under a target's response time and the reason a generous
 * SDADEL costs a fast bus real throughput.
 */
constexpr uint32_t fmpi2c_min_stretch_cycles(const FmpI2cTiming& t) {
    return (static_cast<uint32_t>(t.sdadel) + static_cast<uint32_t>(t.scldel) + 1u) *
               (static_cast<uint32_t>(t.presc) + 1u) + 1u;
}

/**
 * WHAT A REGISTER VALUE REALLY PRODUCES - 23.4.9's own formula:
 *     tSCL = tSYNC1 + tSYNC2 + [(SCLH + 1) + (SCLL + 1)] x tPRESC
 * `sync_ns` is the caller's tSYNC1 + tSYNC2 budget (see FmpI2cBusTiming).
 * With each table's own footnoted budget this reproduces every cell of
 * tables 134 and 135 exactly - the static_asserts at the bottom of this
 * file are that check.
 */
constexpr uint32_t fmpi2c_scl_period_cycles(uint32_t kernel_hz, const FmpI2cTiming& t,
                                            uint32_t sync_ns) {
    return fmpi2c_scll_cycles(t) + fmpi2c_sclh_cycles(t) +
           fmpi2c_ns_cycles_near(kernel_hz, sync_ns);
}

constexpr uint32_t fmpi2c_scl_hz(uint32_t kernel_hz, const FmpI2cTiming& t, uint32_t sync_ns) {
    const uint32_t cycles = fmpi2c_scl_period_cycles(kernel_hz, t, sync_ns);
    return cycles == 0u ? 0u : kernel_hz / cycles;
}

/// The same with the mode's own default budget - the everyday form.
constexpr uint32_t fmpi2c_scl_hz(uint32_t kernel_hz, const FmpI2cTiming& t, FmpI2cSpeed s) {
    return fmpi2c_scl_hz(kernel_hz, t, fmpi2c_bus_timing(s).sync_ns);
}

/**
 * The low:high split of the SCL period, per mode - and it is the MANUAL'S
 * OWN, read off tables 134 and 135 rather than invented: every Sm column
 * programs 20 low units against 16 high, every Fm column 10 against 4, and
 * the Fm+ column at 16 MHz 5 against 3. Each satisfies table 133's
 * tLOW/tHIGH minima once the detection delay is added, which is the only
 * sense in which they can satisfy them - see fmpi2c_low_min_ns().
 */
constexpr uint8_t fmpi2c_low_share(FmpI2cSpeed s) {
    switch (s) {
        case FmpI2cSpeed::standard_100k: return 5;
        case FmpI2cSpeed::fast_400k: return 5;
        default: return 5;
    }
}
constexpr uint8_t fmpi2c_high_share(FmpI2cSpeed s) {
    switch (s) {
        case FmpI2cSpeed::standard_100k: return 4;
        case FmpI2cSpeed::fast_400k: return 2;
        default: return 3;
    }
}

/**
 * THE CHOOSER: solve 23.4.5's conditions and 23.4.9's period formula for a
 * register value, at this kernel clock, for this speed, on this bus.
 *
 * The method is the chapter's own, in four steps:
 *  1. The PERIOD in kernel cycles is kernel_hz / f_SCL ROUNDED UP, less the
 *     tSYNC budget; what remains is split between tSCLL and tSCLH in the
 *     mode's own ratio (above), and both halves are rounded UP into
 *     prescaler units. Every rounding is in the same direction on purpose:
 *     a requested SCL is a CEILING and the produced bus is never faster
 *     than the one asked for, at a kernel rate that divides the speed or
 *     at one that does not.
 *  2. SCLDEL comes from 23.4.5's setup condition, a LOWER bound:
 *     (SCLDEL + 1) x tPRESC >= tr(max) + tSU;DAT(min).
 *  3. SDADEL comes from its hold condition, also a lower bound:
 *     SDADEL x tPRESC >= tf(max) + tHD;DAT(min) - tAF(min)
 *                        - (DNF + 3) x tI2CCLK
 *     with the tAF term present only while the analog filter is on and the
 *     whole right-hand side floored at zero.
 *  4. PRESC IS WHAT MAKES 2 AND 3 FIT. SCLDEL and SDADEL are four bits
 *     each, so a small prescaler can put the setup delay out of range (at
 *     180 MHz in Sm the setup needs 225 kernel cycles and the field holds
 *     sixteen units) - which is why the manual's own tables use a coarse
 *     PRESC and small SCLL/SCLH. This function walks PRESC from 0 up and
 *     takes the FIRST value at which every field fits, which gives the
 *     finest resolution the four-bit delays allow.
 *
 * WHAT IT DOES NOT DO is reproduce the manual's own register words: its
 * tables are hand-picked, and for the same times there are several legal
 * (PRESC, SCLL, SCLH) triples. What it reproduces is the TIMES.
 *
 * Returns nullopt when the kernel clock is below ES0298 2.12.2's floor for
 * this speed, when 23.4.3's own conditions are not met, when the period
 * leaves no room after the tSYNC budget, or when no prescaler makes every
 * field fit. A speed that cannot be produced is REFUSED and never
 * approximated: a bus run at a rate nobody asked for is a fault the caller
 * must see.
 */
constexpr std::optional<FmpI2cTiming> fmpi2c_timing_for(uint32_t kernel_hz, FmpI2cSpeed s,
                                                        const FmpI2cFilters& filters,
                                                        const FmpI2cBusTiming& bus) {
    if (kernel_hz < fmpi2c_min_kernel_hz(s) || !fmpi2c_filters_valid(filters)) {
        return {};
    }
    if (!fmpi2c_clock_requirements_met(kernel_hz, s, filters)) {
        return {};
    }
    const uint32_t speed_hz = fmpi2c_speed_hz(s);
    const uint32_t total = (kernel_hz + speed_hz - 1u) / speed_hz;
    const uint32_t sync = fmpi2c_ns_cycles_near(kernel_hz, bus.sync_ns);
    if (total < sync + 2u) {
        return {};
    }
    const uint32_t budget = total - sync;
    const uint32_t den = static_cast<uint32_t>(fmpi2c_low_share(s)) + fmpi2c_high_share(s);
    const uint32_t low = (budget * fmpi2c_low_share(s) + den - 1u) / den;
    if (low >= budget) {
        return {};
    }
    const uint32_t high = budget - low;

    // The two delay bounds, in KERNEL cycles (the prescaler divides them
    // below). Both are 23.4.5's, with tHD;DAT(min) = 0 on every non-SMBus
    // row of table 131 and the analog filter's term dropped when it is off.
    const uint32_t setup_cycles = fmpi2c_ns_cycles_up(kernel_hz, bus.rise_ns) +
                                  fmpi2c_ns_cycles_up(kernel_hz, bus.setup_ns);
    uint32_t hold_cycles = fmpi2c_ns_cycles_up(kernel_hz, bus.fall_ns);
    const uint32_t subtract =
        (filters.analog ? fmpi2c_ns_cycles_down(kernel_hz, fmpi2c_analog_filter_min_ns) : 0u) +
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
        if (scll_units == 0u || scll_units > 256u || sclh_units == 0u || sclh_units > 256u) {
            continue;
        }
        return FmpI2cTiming{static_cast<uint8_t>(p - 1u),
                            static_cast<uint8_t>(scll_units - 1u),
                            static_cast<uint8_t>(sclh_units - 1u),
                            static_cast<uint8_t>(sdadel_units),
                            static_cast<uint8_t>(scldel_units - 1u)};
    }
    return {};
}

/// The same with the mode's own standard limits and tSYNC budget - what
/// every caller that has not measured its bus wants.
constexpr std::optional<FmpI2cTiming> fmpi2c_timing_for(uint32_t kernel_hz, FmpI2cSpeed s,
                                                        const FmpI2cFilters& filters = {}) {
    return fmpi2c_timing_for(kernel_hz, s, filters, fmpi2c_bus_timing(s));
}

/**
 * Whether a timing value satisfies 23.4.5's SETUP condition at this kernel
 * clock - the one bound that must hold for a receiver to see a settled
 * SDA. Separate from the chooser so a hand-written TIMINGR (the manual's
 * own, CubeMX's) can be judged.
 */
constexpr bool fmpi2c_setup_ok(uint32_t kernel_hz, const FmpI2cTiming& t,
                               const FmpI2cBusTiming& bus) {
    return fmpi2c_scldel_cycles(t) >= fmpi2c_ns_cycles_up(kernel_hz, bus.rise_ns) +
                                          fmpi2c_ns_cycles_up(kernel_hz, bus.setup_ns);
}

/**
 * Whether a timing value satisfies 23.4.5's HOLD condition, lower bound
 * only. The chapter's UPPER bound is deliberately NOT a refusal anywhere in
 * this file, because 23.4.5's own note says it "can be violated when
 * NOSTRETCH=0, because the device stretches SCL low to guarantee the set-up
 * time" - which is every configuration this driver produces.
 * fmpi2c_hold_upper_ok() exists so a bench can ask.
 */
constexpr bool fmpi2c_hold_ok(uint32_t kernel_hz, const FmpI2cTiming& t,
                              const FmpI2cFilters& f, const FmpI2cBusTiming& bus) {
    const uint32_t fall = fmpi2c_ns_cycles_up(kernel_hz, bus.fall_ns);
    const uint32_t sub =
        (f.analog ? fmpi2c_ns_cycles_down(kernel_hz, fmpi2c_analog_filter_min_ns) : 0u) +
        f.digital + 3u;
    const uint32_t need = fall > sub ? fall - sub : 0u;
    return fmpi2c_sdadel_cycles(t) >= need;
}

constexpr bool fmpi2c_hold_upper_ok(uint32_t kernel_hz, const FmpI2cTiming& t,
                                    const FmpI2cFilters& f, const FmpI2cBusTiming& bus,
                                    FmpI2cSpeed s) {
    const uint32_t budget = fmpi2c_ns_cycles_down(kernel_hz, fmpi2c_data_valid_max_ns(s));
    const uint32_t sub = fmpi2c_ns_cycles_up(kernel_hz, bus.rise_ns) +
                         (f.analog ? fmpi2c_ns_cycles_up(kernel_hz, fmpi2c_analog_filter_max_ns)
                                   : 0u) +
                         f.digital + 4u;
    if (budget <= sub) {
        return false;
    }
    return fmpi2c_sdadel_cycles(t) <= (budget - sub);
}

// =============================================================================
// The SMBus time-outs (23.4.12, 23.4.13, 23.7.6)
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
 * anybody, which is the only one of the three that can see a PEER's hold;
 * TIMEOUTA with TIDLE = 1 watches both lines high (bus idle detection);
 * and TIMEOUTB watches THIS peripheral's OWN cumulative stretch -
 * tLOW:MEXT as controller, tLOW:SEXT as target (23.7.6's own field
 * description). A block with only the second kind could police nothing but
 * its own hold; this one has both.
 */
constexpr uint32_t fmpi2c_timeout_divisor(bool idle) { return idle ? 4u : 2048u; }

/// The code for at least `us` microseconds, or nullopt when twelve bits
/// cannot reach that long at this kernel clock.
constexpr std::optional<uint16_t> fmpi2c_timeout_code_for(uint32_t kernel_hz, uint32_t us,
                                                          bool idle) {
    if (kernel_hz == 0u || us == 0u) {
        return {};
    }
    const uint64_t ticks = static_cast<uint64_t>(us) * kernel_hz / 1'000'000ULL;
    const uint32_t div = fmpi2c_timeout_divisor(idle);
    const uint64_t units = (ticks + div - 1u) / div;
    if (units == 0u || units > 4096u) {
        return {};
    }
    return static_cast<uint16_t>(units - 1u);
}

/// What a code really produces, in microseconds.
constexpr uint32_t fmpi2c_timeout_us(uint32_t kernel_hz, uint16_t code, bool idle) {
    if (kernel_hz == 0u) {
        return 0;
    }
    const uint64_t ticks = (static_cast<uint64_t>(code) + 1u) * fmpi2c_timeout_divisor(idle);
    return static_cast<uint32_t>(ticks * 1'000'000ULL / kernel_hz);
}

/// The SMBus specification's own limits, for a caller sizing a time-out:
/// the clock-low time-out is 25..35 ms, the target's cumulative extend at
/// most 25 ms and the controller's at most 10 ms (23.4.11's table).
inline constexpr uint32_t fmpi2c_smbus_timeout_min_us = 25'000;
inline constexpr uint32_t fmpi2c_smbus_timeout_max_us = 35'000;
inline constexpr uint32_t fmpi2c_smbus_low_sext_max_us = 25'000;
inline constexpr uint32_t fmpi2c_smbus_low_mext_max_us = 10'000;

/// The three SMBus addresses the chapter's own enables answer to.
inline constexpr uint8_t fmpi2c_smbus_host_address = 0x08;      ///< SMBHEN
inline constexpr uint8_t fmpi2c_smbus_device_default = 0x61;    ///< SMBDEN
inline constexpr uint8_t fmpi2c_smbus_alert_response = 0x0C;    ///< ALERTEN

// =============================================================================
// Pads
// =============================================================================

/**
 * The two lines and the optional SMBus alert, each with the alternate
 * function the DATASHEET gives that signal on that pad (DS10693 table 11
 * for the F446: AF4 on every one of them). The standing caveat of every
 * bus task in this stratum applies: NO HEADER SYMBOL CAN CHECK AN
 * ALTERNATE FUNCTION NUMBER, so what is checked here is that both pads are
 * real pins of ports this part bonds and that no two signals name the same
 * pad; that the AF really carries FMPI2C1's signal there is the
 * datasheet's claim and the bench's proof.
 */
struct FmpI2cPins {
    PinSel scl{};
    PinSel sda{};
    PinSel smba{};

    constexpr bool has_alert() const { return smba.valid(); }
};

constexpr bool fmpi2c_pins_valid(const FmpI2cPins& p) {
    if (!p.scl.valid() || !p.sda.valid()) {
        return false;
    }
    const PinSel all[3] = {p.scl, p.sda, p.smba};
    for (uint8_t i = 0; i < 3u; ++i) {
        for (uint8_t j = static_cast<uint8_t>(i + 1u); j < 3u; ++j) {
            if (all[i].valid() && all[j].valid() && all[i].port == all[j].port &&
                all[i].pin == all[j].pin) {
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
 * plus TIMINGR - what configure() writes wholesale with PE clear.
 *
 * NOSTRETCH is a TARGET knob and 23.7.1 says in as many words that it
 * "must be kept cleared in master mode", so a host never sets it.
 */
struct FmpI2cConfig {
    FmpI2cTiming timing{};
    FmpI2cFilters filters{};
    /// Target only: do not stretch SCL. 23.4.8 states the price - the data
    /// must be in TXDR before the first SCL pulse of its own byte, or OVR
    /// is raised and 0xFF goes out instead (and ES0298 2.12.7 says even
    /// the OVR can go missing).
    bool no_stretch = false;
    /// Target byte control: NBYTES/RELOAD gate each received byte so
    /// software can ACK or NACK it. Incompatible with no_stretch.
    bool byte_control = false;
    bool general_call = false;      ///< GCEN: answer address 0x00
    bool smbus_host = false;        ///< SMBHEN: answer the host address
    bool smbus_device = false;      ///< SMBDEN: answer the default address
    bool smbus_alert = false;       ///< ALERTEN: the SMBA pad / ARA
    bool pec = false;               ///< PECEN: the CRC-8 engine
};

constexpr bool fmpi2c_config_valid(const FmpI2cConfig& c) {
    if (!fmpi2c_timing_valid(c.timing) || !fmpi2c_filters_valid(c.filters)) {
        return false;
    }
    // 23.4.8: the target byte control mode is not compatible with
    // NOSTRETCH mode.
    if (c.byte_control && c.no_stretch) {
        return false;
    }
    return true;
}

/**
 * The target's own addresses - OAR1 and OAR2 together, because they are
 * one decision (which addresses this endpoint answers to) split over two
 * registers with two different write gates.
 */
struct FmpI2cAddressConfig {
    uint16_t own = 0;                                   ///< OA1, 7 or 10 bits
    FmpI2cAddressMode mode = FmpI2cAddressMode::seven_bit;
    bool enable = true;                                 ///< OA1EN
    uint8_t second = 0;                                 ///< OA2, ALWAYS 7 bits
    FmpI2cOa2Mask second_mask = FmpI2cOa2Mask::none;
    bool second_enable = false;                         ///< OA2EN
};

constexpr bool fmpi2c_address_valid(uint16_t addr, FmpI2cAddressMode m) {
    return m == FmpI2cAddressMode::ten_bit ? addr <= 0x3FFu : addr <= 0x7Fu;
}

constexpr bool fmpi2c_address_config_valid(const FmpI2cAddressConfig& c) {
    if (!fmpi2c_address_valid(c.own, c.mode)) {
        return false;
    }
    return c.second <= 0x7Fu && fmpi2c_oa2_mask_valid(c.second_mask);
}

/// What init() takes besides the clock: which kernel clock to run on, the
/// noise filters, whether the port's own weak pull-ups are wanted (a bench
/// convenience on a bus with no pull-ups of its own - far too weak for a
/// real one), and the bus's measured edges. Every field has the value a
/// plain 100 kHz bus on the APB clock wants.
///
/// THE CLIENT TAKES THE SAME STRUCT, name notwithstanding: all four are
/// properties of the PORT and not of the role - a target's delays are
/// solved against the same kernel clock through the same filters over the
/// same wire.
struct FmpI2cHostConfig {
    FmpI2cClock kernel = FmpI2cClock::pclk;
    FmpI2cFilters filters{};
    bool internal_pull_up = false;
    std::optional<FmpI2cBusTiming> bus{};
};

// The register-facing half is compiled only where the device header
// declares the block - the F410, F412, F413/F423 and F446 of this pack
// (dac.hpp's shape, for the same reason).
#if defined(FMPI2C1_BASE)

// =============================================================================
// The register vocabulary
// =============================================================================

/// FMPI2C_ISR, by name (23.7.7). Every bit is read-only but TXE and TXIS,
/// which are rs (TXE writable at any time to flush TXDR, TXIS only with
/// NOSTRETCH set). The reset value of the whole register is 0x1: TXE
/// STANDS out of reset and after every PE clear, which is why a
/// transmitter that waits for TXE before its first byte waits for nothing.
struct FmpI2cFlag {
    FmpI2cFlag() = delete;
    static constexpr uint32_t txe = FMPI2C_ISR_TXE;
    static constexpr uint32_t txis = FMPI2C_ISR_TXIS;
    static constexpr uint32_t rxne = FMPI2C_ISR_RXNE;
    static constexpr uint32_t addr = FMPI2C_ISR_ADDR;
    static constexpr uint32_t nack = FMPI2C_ISR_NACKF;
    static constexpr uint32_t stop = FMPI2C_ISR_STOPF;
    static constexpr uint32_t transfer_complete = FMPI2C_ISR_TC;
    static constexpr uint32_t transfer_reload = FMPI2C_ISR_TCR;
    static constexpr uint32_t bus_error = FMPI2C_ISR_BERR;
    static constexpr uint32_t arb_lost = FMPI2C_ISR_ARLO;
    static constexpr uint32_t overrun = FMPI2C_ISR_OVR;
    static constexpr uint32_t pec_error = FMPI2C_ISR_PECERR;
    static constexpr uint32_t timeout = FMPI2C_ISR_TIMEOUT;
    static constexpr uint32_t alert = FMPI2C_ISR_ALERT;
    static constexpr uint32_t busy = FMPI2C_ISR_BUSY;
    static constexpr uint32_t dir = FMPI2C_ISR_DIR;
    /// THE VECTOR SPLIT (table 142), which is this family's own: seven
    /// conditions reach FMPI2C1_EV and six reach FMPI2C1_ER. The STM32G0
    /// puts all thirteen on one line; here a handler serves its own half
    /// and can say so, which is why the masks are named.
    static constexpr uint32_t events =
        txis | rxne | addr | nack | stop | transfer_complete | transfer_reload;
    static constexpr uint32_t errors =
        bus_error | arb_lost | overrun | pec_error | timeout | alert;
};

/// FMPI2C_ICR (23.7.8): the W1C clears, one per clearable flag. THE MASK IS
/// NOT THE ISR'S - the bits happen to sit at the same positions, but only
/// nine of them exist and TC, TCR, TXIS and RXNE have no clear bit at all
/// (they are cleared by an access, a START, a STOP or an NBYTES write).
/// ADDRCF is DUAL-PURPOSE: 23.7.8 says it "also clears the START bit in the
/// FMPI2C_CR2 register", which is how a target that was addressed while its
/// own START stood gets out of controller mode - and, per ES0298 2.12.6, is
/// the ONLY thing that clears START there.
struct FmpI2cClear {
    FmpI2cClear() = delete;
    static constexpr uint32_t addr = FMPI2C_ICR_ADDRCF;
    static constexpr uint32_t nack = FMPI2C_ICR_NACKCF;
    static constexpr uint32_t stop = FMPI2C_ICR_STOPCF;
    static constexpr uint32_t bus_error = FMPI2C_ICR_BERRCF;
    static constexpr uint32_t arb_lost = FMPI2C_ICR_ARLOCF;
    static constexpr uint32_t overrun = FMPI2C_ICR_OVRCF;
    static constexpr uint32_t pec_error = FMPI2C_ICR_PECCF;
    static constexpr uint32_t timeout = FMPI2C_ICR_TIMOUTCF;
    static constexpr uint32_t alert = FMPI2C_ICR_ALERTCF;
    static constexpr uint32_t errors =
        bus_error | arb_lost | overrun | pec_error | timeout | alert;
    static constexpr uint32_t all = addr | nack | stop | errors;
};

/// CR1's seven interrupt enables (table 142). ERRIE alone covers all six
/// error conditions, which is why the ISR bodies hand the caller a masked
/// status rather than a decision.
struct FmpI2cInterrupt {
    FmpI2cInterrupt() = delete;
    static constexpr uint32_t tx = FMPI2C_CR1_TXIE;
    static constexpr uint32_t rx = FMPI2C_CR1_RXIE;
    static constexpr uint32_t addr = FMPI2C_CR1_ADDRIE;
    static constexpr uint32_t nack = FMPI2C_CR1_NACKIE;
    static constexpr uint32_t stop = FMPI2C_CR1_STOPIE;
    static constexpr uint32_t transfer_complete = FMPI2C_CR1_TCIE;
    static constexpr uint32_t error = FMPI2C_CR1_ERRIE;
    static constexpr uint32_t all = tx | rx | addr | nack | stop | transfer_complete | error;
};

constexpr uint32_t fmpi2c_timingr(const FmpI2cTiming& t) {
    return (static_cast<uint32_t>(t.presc & 0xFu) << FMPI2C_TIMINGR_PRESC_Pos) |
           (static_cast<uint32_t>(t.scldel & 0xFu) << FMPI2C_TIMINGR_SCLDEL_Pos) |
           (static_cast<uint32_t>(t.sdadel & 0xFu) << FMPI2C_TIMINGR_SDADEL_Pos) |
           (static_cast<uint32_t>(t.sclh) << FMPI2C_TIMINGR_SCLH_Pos) |
           (static_cast<uint32_t>(t.scll) << FMPI2C_TIMINGR_SCLL_Pos);
}

constexpr FmpI2cTiming fmpi2c_timing_of(uint32_t reg) {
    return FmpI2cTiming{
        static_cast<uint8_t>((reg & FMPI2C_TIMINGR_PRESC_Msk) >> FMPI2C_TIMINGR_PRESC_Pos),
        static_cast<uint8_t>((reg & FMPI2C_TIMINGR_SCLL_Msk) >> FMPI2C_TIMINGR_SCLL_Pos),
        static_cast<uint8_t>((reg & FMPI2C_TIMINGR_SCLH_Msk) >> FMPI2C_TIMINGR_SCLH_Pos),
        static_cast<uint8_t>((reg & FMPI2C_TIMINGR_SDADEL_Msk) >> FMPI2C_TIMINGR_SDADEL_Pos),
        static_cast<uint8_t>((reg & FMPI2C_TIMINGR_SCLDEL_Msk) >> FMPI2C_TIMINGR_SCLDEL_Pos),
    };
}

constexpr uint32_t fmpi2c_cr1(const FmpI2cConfig& c) {
    uint32_t v = 0;
    if (!c.filters.analog) {
        v |= FMPI2C_CR1_ANFOFF;
    }
    v |= static_cast<uint32_t>(c.filters.digital) << FMPI2C_CR1_DNF_Pos;
    if (c.no_stretch) {
        v |= FMPI2C_CR1_NOSTRETCH;
    }
    if (c.byte_control) {
        v |= FMPI2C_CR1_SBC;
    }
    if (c.general_call) {
        v |= FMPI2C_CR1_GCEN;
    }
    if (c.smbus_host) {
        v |= FMPI2C_CR1_SMBHEN;
    }
    if (c.smbus_device) {
        v |= FMPI2C_CR1_SMBDEN;
    }
    if (c.smbus_alert) {
        v |= FMPI2C_CR1_ALERTEN;
    }
    if (c.pec) {
        v |= FMPI2C_CR1_PECEN;
    }
    return v;
}

// =============================================================================
// The resource
// =============================================================================

/**
 * One FMPI2C instance, register by register. Every verb stores what it
 * names and nothing else; a verb whose combination the chapter declares
 * wrong, or whose gate is closed, refuses (false, nothing written). The
 * tasks below are written on these verbs.
 */
template <uint8_t n>
struct FmpI2c {
    static_assert(n == 1 && fmpi2c_present(),
                  "brio FmpI2c: this device has no such FMPI2C instance - every part of this "
                  "family that carries the block carries exactly ONE (FMPI2C1, the same "
                  "peripheral the manuals also call I2C4), and the device header declares no "
                  "FMPI2C1_BASE on the rest; the index is here so a part with two would not "
                  "rename every user of this file");

    FmpI2c() = delete;

    static constexpr uint8_t number = n;

    /// Table 127's implementation row for the part whose manual was read.
    /// The SMBus half is present on the STM32F446; on the other parts that
    /// carry this block no manual on this desk says either way, so this is
    /// false there and means "not claimed", not "absent" - which is why no
    /// verb of this file refuses on it and smbus_probe() below is the
    /// silicon's own answer.
    static constexpr bool smbus_claimed = fmpi2c_smbus_claimed();
    /// Table 127's dash: this block has NO wake from Stop on any part of
    /// the pack, and the header agrees - there is no FMPI2C_CR1_WUPEN
    /// anywhere and 23.7.1 makes CR1's bit 18 reserved. The one feature of
    /// the STM32G0's I2C that is missing here.
    static constexpr bool wakes_from_stop = fmpi2c_has_wakeup();
    /// The unconditional rows of table 127, as constants so a bench can
    /// compare a table with a register.
    static constexpr bool has_ten_bit = true;
    static constexpr bool has_fast_plus = true;

    static constexpr IRQn_Type event_irq = fmpi2c_event_irq();
    static constexpr IRQn_Type error_irq = fmpi2c_error_irq();

    static FMPI2C_TypeDef& regs() {
        return *reinterpret_cast<FMPI2C_TypeDef*>(fmpi2c_base());
    }

    // ---- clocks and reset ----------------------------------------------------

    /// The APB1 clock of the block. Dead registers without it.
    static void bus_clock(bool on) { Rcc::apb1_clock(fmpi2c_clock_mask(), on); }
    static bool bus_clock() { return Rcc::apb1_clock(fmpi2c_clock_mask()); }

    /// Software reset through RCC: every register back to its reset value.
    /// The one way out of a state the chapter has no sequence for - and
    /// note that PE = 0 is NOT that (23.4.6 keeps every configuration
    /// register).
    static void reset() { Rcc::apb1_reset(fmpi2c_clock_mask()); }

    /// The kernel-clock multiplexer (RCC_DCKCFGR2.FMPI2C1SEL). False -
    /// nothing written - only for a code outside the field's three
    /// meanings.
    static bool kernel_clock(FmpI2cClock c) {
        if (!fmpi2c_clock_valid(c)) {
            return false;
        }
        return Rcc::fmpi2c_kernel_clock(static_cast<uint8_t>(c));
    }
    static FmpI2cClock kernel_clock() {
        const uint8_t code = Rcc::fmpi2c_kernel_clock();
        // 6.3.27: code 3 is "APB clock selected ... (same as 00)".
        return code == 3u ? FmpI2cClock::pclk : static_cast<FmpI2cClock>(code);
    }

    /// What this instance's kernel really runs at, given the app's own two
    /// rates - the number every piece of arithmetic in this file is
    /// against. The HSI is a fixed 16 MHz whatever the core is doing, which
    /// is the whole point of an independent clock.
    static uint32_t kernel_hz(uint32_t pclk_hz, uint32_t sysclk_hz) {
        switch (kernel_clock()) {
            case FmpI2cClock::sysclk: return sysclk_hz;
            case FmpI2cClock::hsi: return brio::hsi_hz;
            default: return pclk_hz;
        }
    }

    // ---- the enable, and 23.4.6's disable PROCEDURE ---------------------------

    static bool enabled() { return (regs().CR1 & FMPI2C_CR1_PE) != 0u; }

    static void enable() { regs().CR1 = regs().CR1 | FMPI2C_CR1_PE; }

    /// 23.4.6's SOFTWARE RESET, and the only way this driver turns the
    /// peripheral off: write PE = 0, check PE = 0, write PE = 1 is the
    /// chapter's own three-step (PE must stay low for at least three APB
    /// cycles), and this verb is its first two steps.
    ///
    /// WHAT IT DOES: releases SCL and SDA, resets the state machines,
    /// clears CR2's START, STOP, NACK and PECBYTE, and clears every status
    /// flag - BUSY and the six errors included, TXE going back to 1.
    /// WHAT IT KEEPS: every configuration register - CR1's other bits,
    /// OAR1, OAR2, TIMINGR, TIMEOUTR and the rest of CR2. That is why
    /// enable() alone brings the same peripheral back and why recover()
    /// needs no reconfiguration.
    ///
    /// Returns false only if PE would not read back clear (a bus clock that
    /// is off, and nothing else) - an engine that reports is worth more
    /// than one that hangs.
    static bool disable() {
        regs().CR1 = regs().CR1 & ~FMPI2C_CR1_PE;
        uint32_t left = 1000u;
        while ((regs().CR1 & FMPI2C_CR1_PE) != 0u && left-- != 0u) {
        }
        return left != 0u;
    }

    /// The whole of 23.4.6: off, checked, and on again.
    static bool cycle() {
        const bool ok = disable();
        enable();
        return ok;
    }

    // ---- configuration: PE = 0 for all of it ----------------------------------

    /**
     * CR1's configuration half and the whole of TIMINGR in one pair of
     * stores. Refused - nothing written - while the instance is enabled or
     * while the configuration breaks one of the chapter's own rules.
     *
     * The interrupt enables, the DMA enables and PE itself are NOT here:
     * they live on their own verbs because they are legal under a running
     * peripheral and this one is not.
     */
    static bool configure(const FmpI2cConfig& c) {
        if (enabled() || !fmpi2c_config_valid(c)) {
            return false;
        }
        const uint32_t keep = regs().CR1 & (FMPI2C_CR1_TXDMAEN | FMPI2C_CR1_RXDMAEN |
                                            FmpI2cInterrupt::all);
        regs().CR1 = fmpi2c_cr1(c) | keep;
        regs().TIMINGR = fmpi2c_timingr(c.timing);
        return true;
    }

    /// TIMINGR alone. 23.7.5: "This register must be configured when the
    /// FMPI2C is disabled (PE = 0)."
    static bool timing(const FmpI2cTiming& t) {
        if (enabled() || !fmpi2c_timing_valid(t)) {
            return false;
        }
        regs().TIMINGR = fmpi2c_timingr(t);
        return true;
    }
    static FmpI2cTiming timing() { return fmpi2c_timing_of(regs().TIMINGR); }
    static uint32_t timing_reg() { return regs().TIMINGR; }

    /// ANFOFF and DNF, which 23.7.1 gates on PE = 0 one by one and 23.4.5
    /// gates as one ("Changing the filter configuration is not allowed when
    /// the FMPI2C is enabled").
    static bool filters(const FmpI2cFilters& f) {
        if (enabled() || !fmpi2c_filters_valid(f)) {
            return false;
        }
        uint32_t v = regs().CR1 & ~(FMPI2C_CR1_ANFOFF | FMPI2C_CR1_DNF);
        if (!f.analog) {
            v |= FMPI2C_CR1_ANFOFF;
        }
        v |= static_cast<uint32_t>(f.digital) << FMPI2C_CR1_DNF_Pos;
        regs().CR1 = v;
        return true;
    }
    static FmpI2cFilters filters() {
        const uint32_t v = regs().CR1;
        return FmpI2cFilters{(v & FMPI2C_CR1_ANFOFF) == 0u,
                             static_cast<uint8_t>((v & FMPI2C_CR1_DNF) >> FMPI2C_CR1_DNF_Pos)};
    }

    /// NOSTRETCH (23.7.1: "can only be programmed when the I2C is disabled").
    static bool no_stretch(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR1 = on ? (regs().CR1 | FMPI2C_CR1_NOSTRETCH)
                        : (regs().CR1 & ~FMPI2C_CR1_NOSTRETCH);
        return true;
    }
    static bool no_stretch() { return (regs().CR1 & FMPI2C_CR1_NOSTRETCH) != 0u; }

    /// SBC. 23.4.8 lets this one move under a running peripheral in two
    /// stated windows ("when the slave is not addressed, or when ADDR = 1"),
    /// which no register can check - so the verb is open and the obligation
    /// is the caller's, stated here and in the document.
    static void byte_control(bool on) {
        regs().CR1 = on ? (regs().CR1 | FMPI2C_CR1_SBC) : (regs().CR1 & ~FMPI2C_CR1_SBC);
    }
    static bool byte_control() { return (regs().CR1 & FMPI2C_CR1_SBC) != 0u; }

    /// GCEN: acknowledge the general-call address 0x00.
    static void general_call(bool on) {
        regs().CR1 = on ? (regs().CR1 | FMPI2C_CR1_GCEN) : (regs().CR1 & ~FMPI2C_CR1_GCEN);
    }
    static bool general_call() { return (regs().CR1 & FMPI2C_CR1_GCEN) != 0u; }

    // ---- the target's own addresses -------------------------------------------

    /**
     * OAR1 and OAR2 together. Each register gates its own fields on ITS OWN
     * enable bit (23.7.3 / 23.7.4: "can be written only when OA1EN = 0" /
     * "OA2EN = 0"), so each is cleared, written and re-enabled here - which
     * is the only sequence that lands.
     */
    static bool addresses(const FmpI2cAddressConfig& c) {
        if (!fmpi2c_address_config_valid(c)) {
            return false;
        }
        regs().OAR1 = 0;   // OA1EN low: the fields open
        uint32_t a1 = c.mode == FmpI2cAddressMode::ten_bit
                          ? (static_cast<uint32_t>(c.own) & FMPI2C_OAR1_OA1_Msk)
                          : (static_cast<uint32_t>(c.own & 0x7Fu) << 1);
        if (c.mode == FmpI2cAddressMode::ten_bit) {
            a1 |= FMPI2C_OAR1_OA1MODE;
        }
        if (c.enable) {
            a1 |= FMPI2C_OAR1_OA1EN;
        }
        regs().OAR1 = a1;

        regs().OAR2 = 0;
        uint32_t a2 = (static_cast<uint32_t>(c.second & 0x7Fu) << 1) |
                      (static_cast<uint32_t>(c.second_mask) << FMPI2C_OAR2_OA2MSK_Pos);
        if (c.second_enable) {
            a2 |= FMPI2C_OAR2_OA2EN;
        }
        regs().OAR2 = a2;
        return true;
    }

    static uint32_t oar1() { return regs().OAR1; }
    static uint32_t oar2() { return regs().OAR2; }

    /// The 7-bit address OA1 answers to (meaningless in 10-bit mode, where
    /// own_address10() is the reading).
    static uint8_t own_address() {
        return static_cast<uint8_t>((regs().OAR1 & FMPI2C_OAR1_OA1_Msk) >> 1) & 0x7Fu;
    }
    static uint16_t own_address10() {
        return static_cast<uint16_t>(regs().OAR1 & FMPI2C_OAR1_OA1_Msk);
    }

    // ---- the controller's transfer engine (23.4.9) -----------------------------

    /**
     * One CR2 word: the address, the direction, the byte count and the end
     * policy. Written WITHOUT the START bit, so a caller can set it up and
     * launch separately.
     *
     * `reload` is 23.4.9's answer to a transfer longer than 255 bytes: with
     * it set the peripheral raises TCR and stretches SCL when NBYTES runs
     * out, and the next NBYTES write continues the SAME tenure. AUTOEND has
     * no effect while RELOAD is set - the chapter's own caution, and the
     * reason this function refuses the pair rather than writing a bit the
     * silicon ignores. AND ES0298 2.12.4 IS AGAINST IT: with RELOAD set and
     * NBYTES > 1 a receiver can lose the byte that raises TCR, so the
     * erratum's own advice is RELOAD with NBYTES = 1 or no RELOAD at all.
     */
    static bool transfer(uint16_t addr, bool read, uint8_t nbytes, bool auto_end,
                         bool reload = false,
                         FmpI2cAddressMode mode = FmpI2cAddressMode::seven_bit,
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
     * 23.7.2 gives START two meanings and says nothing about the ORDER of
     * the two writes it would take to get there in pieces. THE ORDER
     * DECIDES: at TC the previous transfer is finished but UNTERMINATED, so
     * a CR2 store that raises AUTOEND before START is read as a request to
     * end THAT transfer - the peripheral sends the STOP at once and START,
     * written a cycle later, opens a whole new tenure instead of restarting
     * this one. (Measured on the STM32G0's identical block, whose driver
     * carries the same verb and the same warning.)
     */
    static bool transfer_now(uint16_t addr, bool read, uint8_t nbytes, bool auto_end,
                             bool reload = false,
                             FmpI2cAddressMode mode = FmpI2cAddressMode::seven_bit,
                             bool head10_read_only = false) {
        const std::optional<uint32_t> v =
            cr2_word(addr, read, nbytes, auto_end, reload, mode, head10_read_only);
        if (!v) {
            return false;
        }
        regs().CR2 = *v | FMPI2C_CR2_START;
        return true;
    }

    /// The CR2 word the two verbs above store, or nullopt when the chapter
    /// refuses it: "Changing all the above bits is not allowed when START
    /// bit is set" (23.4.9), an address must fit its mode, and AUTOEND has
    /// no effect under RELOAD so the pair is a caller's mistake.
    static std::optional<uint32_t> cr2_word(uint16_t addr, bool read, uint8_t nbytes,
                                            bool auto_end, bool reload, FmpI2cAddressMode mode,
                                            bool head10_read_only) {
        if ((regs().CR2 & FMPI2C_CR2_START) != 0u) {
            return {};
        }
        if (!fmpi2c_address_valid(addr, mode) || (reload && auto_end)) {
            return {};
        }
        uint32_t v = mode == FmpI2cAddressMode::ten_bit
                         ? (static_cast<uint32_t>(addr) & FMPI2C_CR2_SADD_Msk)
                         : (static_cast<uint32_t>(addr & 0x7Fu) << 1);
        if (mode == FmpI2cAddressMode::ten_bit) {
            v |= FMPI2C_CR2_ADD10;
            if (head10_read_only) {
                v |= FMPI2C_CR2_HEAD10R;
            }
        }
        if (read) {
            v |= FMPI2C_CR2_RD_WRN;
        }
        v |= static_cast<uint32_t>(nbytes) << FMPI2C_CR2_NBYTES_Pos;
        if (auto_end) {
            v |= FMPI2C_CR2_AUTOEND;
        }
        if (reload) {
            v |= FMPI2C_CR2_RELOAD;
        }
        return v;
    }

    /// Set START on whatever transfer() left in CR2. A repeated START is
    /// the SAME verb (23.4.9's closing note: "The same procedure is applied
    /// for a Repeated Start condition. In this case BUSY=1").
    static void start() { regs().CR2 = regs().CR2 | FMPI2C_CR2_START; }
    static bool starting() { return (regs().CR2 & FMPI2C_CR2_START) != 0u; }

    /// A software STOP - what a software-end (AUTOEND = 0) tenure needs at
    /// TC when there is no repeated START to make. Clears TC.
    static void stop() { regs().CR2 = regs().CR2 | FMPI2C_CR2_STOP; }

    /// The next chunk of a RELOAD transfer: writing a non-zero NBYTES
    /// clears TCR and releases the stretch (23.4.9). `reload` false on the
    /// LAST chunk hands the ending back to AUTOEND or to TC. See
    /// transfer()'s note on ES0298 2.12.4 before using it with more than
    /// one byte.
    static bool reload(uint8_t nbytes, bool more, bool auto_end) {
        if (nbytes == 0u || (more && auto_end)) {
            return false;
        }
        uint32_t v = regs().CR2 & ~(FMPI2C_CR2_NBYTES | FMPI2C_CR2_RELOAD | FMPI2C_CR2_AUTOEND);
        v |= static_cast<uint32_t>(nbytes) << FMPI2C_CR2_NBYTES_Pos;
        if (more) {
            v |= FMPI2C_CR2_RELOAD;
        }
        if (auto_end) {
            v |= FMPI2C_CR2_AUTOEND;
        }
        regs().CR2 = v;
        return true;
    }

    static uint8_t nbytes() {
        return static_cast<uint8_t>((regs().CR2 & FMPI2C_CR2_NBYTES) >> FMPI2C_CR2_NBYTES_Pos);
    }
    static uint32_t cr2() { return regs().CR2; }

    /// CR2.NACK - the TARGET's acknowledge control (23.7.2 says so in as
    /// many words: "used in slave mode"). A controller receiver NACKs its
    /// last byte by itself, whatever this bit holds.
    static void nack_next(bool on) {
        regs().CR2 = on ? (regs().CR2 | FMPI2C_CR2_NACK) : (regs().CR2 & ~FMPI2C_CR2_NACK);
    }

    /// CR2.PECBYTE: transfer the PEC after NBYTES - 1 data bytes. No effect
    /// while RELOAD is set.
    static void pec_byte(bool on) {
        regs().CR2 = on ? (regs().CR2 | FMPI2C_CR2_PECBYTE)
                        : (regs().CR2 & ~FMPI2C_CR2_PECBYTE);
    }
    static uint8_t pec() { return static_cast<uint8_t>(regs().PECR & FMPI2C_PECR_PEC_Msk); }

    /**
     * ES0298 2.12.1's wedge, as a question: in 10-bit controller mode, a
     * first address byte nobody acknowledges leaves NACKF standing WITH
     * START, and no new transfer can be launched. The escape is the
     * erratum's own - wait for STOPF, then cycle() - and 23.4.9 adds the
     * other half of the same story: without ADDRCF the master re-launches
     * the address for ever.
     *
     * The host task below cannot reach this state (its Request is 7-bit),
     * so the resource publishes it for a program that addresses ten bits by
     * hand.
     */
    static bool ten_bit_nack_stuck() {
        return (regs().ISR & FMPI2C_ISR_NACKF) != 0u && starting();
    }

    // ---- data -------------------------------------------------------------------

    [[gnu::always_inline]] static uint8_t data() {
        return static_cast<uint8_t>(regs().RXDR & 0xFFu);
    }
    [[gnu::always_inline]] static void data(uint8_t v) { regs().TXDR = v; }
    /// ISR.TXE is rs: writing 1 FLUSHES a byte that was loaded and is no
    /// longer wanted (23.7.7's own trick for a target that must replace the
    /// first byte of a new tenure).
    static void flush_tx() { regs().ISR = FMPI2C_ISR_TXE; }

    /// The two data registers, which unlike the other I2C's are SEPARATE:
    /// a transmit engine pours into TXDR and a receive one drains RXDR.
    static volatile void* tx_address() { return &regs().TXDR; }
    static volatile void* rx_address() { return &regs().RXDR; }

    // ---- flags, clears, interrupts ----------------------------------------------

    [[gnu::always_inline]] static uint32_t flags() { return regs().ISR; }
    static bool flag(uint32_t mask) { return (regs().ISR & mask) != 0u; }
    /// The W1C clears. THE MASK IS ICR'S (FmpI2cClear), not ISR's.
    static void clear(uint32_t icr_mask) { regs().ICR = icr_mask; }
    static void clear_all() { regs().ICR = FmpI2cClear::all; }

    static bool busy() { return (regs().ISR & FMPI2C_ISR_BUSY) != 0u; }
    /// Valid at an address match: true = the controller is READING, so this
    /// target transmits (23.7.7).
    static bool host_reads() { return (regs().ISR & FMPI2C_ISR_DIR) != 0u; }
    /// The address that matched. In 10-bit mode it is the header plus the
    /// address's two MSBs, not the address (23.7.7).
    static uint8_t matched_address() {
        return static_cast<uint8_t>((regs().ISR & FMPI2C_ISR_ADDCODE) >>
                                    FMPI2C_ISR_ADDCODE_Pos);
    }

    static void interrupt(uint32_t mask, bool on) {
        regs().CR1 = on ? (regs().CR1 | mask) : (regs().CR1 & ~mask);
    }
    static uint32_t interrupts() { return regs().CR1 & FmpI2cInterrupt::all; }

    /// The masked pending status: what this instance is really asking for,
    /// over both vectors.
    [[gnu::always_inline]] static uint32_t pending() {
        const uint32_t en = regs().CR1;
        const uint32_t st = regs().ISR;
        uint32_t served = 0;
        if ((en & FMPI2C_CR1_TXIE) != 0u) {
            served |= st & FMPI2C_ISR_TXIS;
        }
        if ((en & FMPI2C_CR1_RXIE) != 0u) {
            served |= st & FMPI2C_ISR_RXNE;
        }
        if ((en & FMPI2C_CR1_ADDRIE) != 0u) {
            served |= st & FMPI2C_ISR_ADDR;
        }
        if ((en & FMPI2C_CR1_NACKIE) != 0u) {
            served |= st & FMPI2C_ISR_NACKF;
        }
        if ((en & FMPI2C_CR1_STOPIE) != 0u) {
            served |= st & FMPI2C_ISR_STOPF;
        }
        if ((en & FMPI2C_CR1_TCIE) != 0u) {
            served |= st & (FMPI2C_ISR_TC | FMPI2C_ISR_TCR);
        }
        if ((en & FMPI2C_CR1_ERRIE) != 0u) {
            served |= st & FmpI2cFlag::errors;
        }
        return served;
    }

    /// The half of it that reaches FMPI2C1_EV, and the half that reaches
    /// FMPI2C1_ER (table 142). A handler serves its own half: a flag it
    /// does not own belongs to the other vector and clearing it there would
    /// take a completion with it.
    [[gnu::always_inline]] static uint32_t pending_event() {
        return pending() & FmpI2cFlag::events;
    }
    [[gnu::always_inline]] static uint32_t pending_error() {
        return pending() & FmpI2cFlag::errors;
    }

    // ---- DMA ---------------------------------------------------------------------

    /// TXDMAEN feeds TXDR on TXIS; RXDMAEN drains RXDR on RXNE (23.4.16).
    /// Neither the address nor the START is a DMA affair, so a DMA tenure
    /// still costs the framing interrupts.
    static void dma_transmit(bool on) {
        regs().CR1 = on ? (regs().CR1 | FMPI2C_CR1_TXDMAEN)
                        : (regs().CR1 & ~FMPI2C_CR1_TXDMAEN);
    }
    static void dma_receive(bool on) {
        regs().CR1 = on ? (regs().CR1 | FMPI2C_CR1_RXDMAEN)
                        : (regs().CR1 & ~FMPI2C_CR1_RXDMAEN);
    }
    static bool dma_transmit() { return (regs().CR1 & FMPI2C_CR1_TXDMAEN) != 0u; }
    static bool dma_receive() { return (regs().CR1 & FMPI2C_CR1_RXDMAEN) != 0u; }

    // ---- the SMBus half (23.4.11 .. 23.4.15) --------------------------------------

    /**
     * TIMEOUTR wholesale. The two halves have DIFFERENT write gates
     * (TIMEOUTA and TIDLE on TIMOUTEN = 0, TIMEOUTB on TEXTEN = 0), so this
     * verb clears both enables first and raises them last, which is the only
     * order in which every field lands.
     *
     * Refused for a code past twelve bits and nothing else: where the SMBus
     * half is absent 23.7.6 makes the whole register "reserved and forced by
     * hardware to 0x00000000", so the SILICON refuses the write visibly and
     * smbus_probe() is how a program asks.
     */
    static bool timeouts(uint16_t code_a, bool idle, bool enable_a, uint16_t code_b,
                         bool enable_b) {
        if (code_a > 4095u || code_b > 4095u) {
            return false;
        }
        regs().TIMEOUTR = 0;   // both enables low: every field opens
        uint32_t v = static_cast<uint32_t>(code_a) << FMPI2C_TIMEOUTR_TIMEOUTA_Pos;
        if (idle) {
            v |= FMPI2C_TIMEOUTR_TIDLE;
        }
        v |= static_cast<uint32_t>(code_b) << FMPI2C_TIMEOUTR_TIMEOUTB_Pos;
        regs().TIMEOUTR = v;
        if (enable_a) {
            v |= FMPI2C_TIMEOUTR_TIMOUTEN;
        }
        if (enable_b) {
            v |= FMPI2C_TIMEOUTR_TEXTEN;
        }
        regs().TIMEOUTR = v;
        return true;
    }
    static uint32_t timeouts() { return regs().TIMEOUTR; }

    /**
     * THE SILICON'S OWN ANSWER to table 127's SMBus row, and the reason it
     * can be asked at all: 23.7.6 says that on an instance without SMBus
     * "this register is reserved, and its bits are forced by hardware to
     * 0x00000000". So a value written into TIMEOUTR either reads back or
     * does not, and that is the peripheral saying which column it is in - a
     * second opinion on a table this file carries for one part class only.
     * Leaves the register as it found it.
     */
    static bool smbus_probe() {
        const uint32_t saved = regs().TIMEOUTR;
        regs().TIMEOUTR = 0x0FFFu;   // TIMEOUTA all ones, both enables clear
        const bool answered = (regs().TIMEOUTR & FMPI2C_TIMEOUTR_TIMEOUTA) != 0u;
        regs().TIMEOUTR = saved;
        return answered;
    }

    // ---- the wake from Stop, which this block has not ------------------------------

    /**
     * Table 127's dash, as a verb that refuses. The STM32G0's identical
     * register file has WUPEN at CR1's bit 18 with three conditions on it;
     * here 23.7.1 makes that bit reserved, the header declares no
     * FMPI2C_CR1_WUPEN, and 23.5's table 141 says a Stop keeps the
     * registers and nothing more. So there is nothing to arm, and this verb
     * exists to say so where a portable program would ask.
     */
    static bool wake_from_stop(bool on) {
        (void)on;
        return false;
    }
    static bool wake_from_stop() { return false; }

    // ---- the fast-mode-plus pad drive (SYSCFG_CFGR) ----------------------------------

    /**
     * The 20 mA drive an Fm+ bus needs, which on this family is TWO BITS
     * IN SYSCFG - one for the SCL signal and one for SDA (RM0390 8.2.5's
     * SYSCFG_CFGR), not per pad and not per instance: whichever pad carries
     * FMPI2C1_SCL gets the drive when the SCL bit is set. The register is
     * behind SYSCFG's own clock gate, which stm32f4/syscfg.hpp opens.
     *
     * Without it a 1 MHz bus is a 1 MHz clock with edges that do not
     * arrive: the 20 mA sink is what makes a 120 ns fall time reachable on
     * a pad at all.
     */
    static void fast_plus_drive(bool scl, bool sda) { Syscfg::fast_mode_plus(scl, sda); }
    static bool fast_plus_scl() { return Syscfg::fast_mode_plus_scl(); }
    static bool fast_plus_sda() { return Syscfg::fast_mode_plus_sda(); }

    // ---- the ISR bodies ---------------------------------------------------------------

    /// The masked pending status of each vector, for an app that owns the
    /// protocol itself.
    [[gnu::always_inline]] static uint32_t isr() { return pending_event(); }
    [[gnu::always_inline]] static uint32_t error_isr() { return pending_error(); }

    static void release() {
        Nvic::disable(event_irq);
        Nvic::disable(error_irq);
        (void)disable();
        bus_clock(false);
    }
};

// =============================================================================
// The engine slots
// =============================================================================

/*
 * `NoDmaEngine` comes from stm32f4/dma_engine.hpp, for the reason that file
 * states: a driver with an optional engine slot must not include
 * stm32f4/dma.hpp, or every program with a bus would carry the DMA
 * controller. This file reaches its engines only through their own
 * published names - arm(), start(), service(), complete(), take(),
 * abandon(), stop(), busy(), flag_complete, flag_error - and never spells a
 * DmaStream.
 */

/// Two engines on one bus must not name the same STREAM: a stream moves
/// data one way and has one FIFO. UNLIKE THE SPI'S the reason both must be
/// present is not a shared completion - an I2C tenure completes on STOPF
/// whichever direction moved - but that a write-then-read Request needs
/// both directions inside ONE tenure.
template <typename Tx, typename Rx>
constexpr bool fmpi2c_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::controller != Rx::controller || Tx::stream != Rx::stream;
    } else {
        return true;
    }
}

/// Whether an engine sits on a cell of the request mapping that really
/// carries this block's transmit (or receive) request. True for an absent
/// engine, and FALSE on a part class whose request table was not read - a
/// refusal, never a guess.
template <typename E, bool transmit>
constexpr bool fmpi2c_engine_placed() {
    if constexpr (!E::present) {
        return true;
    } else {
        return fmpi2c_dma_placement_valid(transmit, E::controller, E::stream, E::channel);
    }
}

/// A DMA block the engines could not finish: the engine's own status code,
/// in the range util/bus_master.hpp leaves to engines and above the four
/// util/i2c_bus.hpp holds.
inline constexpr uint8_t fmpi2c_dma_fault = bus_engine_status + 4;

// =============================================================================
// The host task
// =============================================================================

/**
 * FmpI2cHost<n, pins, TxEngine, RxEngine>
 *
 * The transfer ENGINE util/i2c_bus.hpp drives (which owns arbitration, the
 * pending FIFO, replies and the per-bus timeout). This task owns the wire:
 * the address phase, the repeated START of a write-then-read, the byte pump
 * and the status vocabulary.
 *
 * THE TRANSACTION DESCRIPTOR IS EVERY TARGET'S -
 * {addr, tx span, tx_len, rx span, rx_len, reply, speed} - and the
 * PARAMETER LIST IS THIS STRATUM'S I2cHost'S, position for position, so a
 * program moves from the F1-lineage block to this one by changing one type
 * name. One request is ONE BUS TENURE in the four shapes I2C devices use:
 *
 *   write            tx set, rx_len 0    S addr+W data... P
 *   read             tx_len 0, rx set    S addr+R data... P
 *   write-then-read  both                S addr+W tx... Sr addr+R rx... P
 *   probe            both empty          S addr+W P
 *
 * ALWAYS ASYNCHRONOUS: start() returns false and a TransferDone follows
 * from the ISR - even the probe, whose address phase IS the transaction.
 *
 * THE STATE MACHINE IS 23.4.9's, and the flags do most of it. A write half
 * runs with AUTOEND = 0 when a read half follows, so that TC fires with SCL
 * stretched and setting START there IS the repeated START; every other
 * shape runs with AUTOEND = 1 and the hardware sends the STOP. THE
 * COMPLETION EDGE IS STOPF in every case, including the NACK ones - 23.4.9:
 * "If a NACK is received: the TXIS flag is not set, and a STOP condition is
 * automatically sent after the NACK reception" - which is what makes one
 * exit path serve the whole vocabulary.
 *
 * THE VOCABULARY, PRODUCED ON THE WIRE:
 *   i2c_nack_addr    NACKF with no data byte yet moved in this half
 *   i2c_nack_data    NACKF after at least one data byte
 *   i2c_arb_lost     ARLO - another controller won; the silicon has already
 *                    released the bus and switched to target mode, so no
 *                    STOP is ours to send
 *   i2c_rejected     a speed this kernel clock cannot produce - the one
 *                    synchronous answer, no byte moved
 *   fmpi2c_dma_fault a DMA stream reported an error mid-tenure
 *
 * AND THERE IS NO i2c_bus_error HERE, WHICH IS THE ERRATUM'S DOING. ES0298
 * 2.12.3: in controller mode a BERR can be raised spuriously and "any such
 * transfer continues normally"; the workaround is to clear it and go on. So
 * this engine clears BERR, COUNTS it (spurious_bus_errors()) and does not
 * let it decide anything. A tenure that never answers is the ARBITER's to
 * time out (i2c_timeout), and recover() is what it calls.
 *
 * THE TWO OPTIONAL DMA ENGINE SLOTS (the Uart's and the SpiHost's shape,
 * for the same reason: an engineless build must stay byte-identical, so the
 * slots default to NoDmaEngine and every DMA branch folds away under
 * `if constexpr`). With engines named, the DATA of every tenure rides the
 * DMA while the address, the repeated START and the ending stay on the
 * interrupt, because 23.4.16 says the address "cannot be transferred with
 * DMA". The engines are started BEFORE the START bit, which is what 23.4.16
 * asks for and, incidentally, the first of ES0298 2.12.8's two workarounds.
 *
 * THE DMA CONTROLLER IS THE APP'S: Dma<1>::init() before any engined
 * init(). The engines arm STREAMS of a controller somebody else owns.
 *
 * WHAT A CLOCK SWITCH DOES: TIMINGR is solved against the KERNEL clock, so
 * a port on PCLK or SYSCLK must be re-solved when the rate moves and a port
 * on the HSI must not (rebase() does both). A transfer in flight is the
 * caller's problem - the bus AO is the natural place to enforce that, and
 * util/bus_master.hpp's PrepareSleep voter is the pattern.
 *
 * ISR wiring (app glue, as usual):
 *   extern "C" void FMPI2C1_EV_IRQHandler() {
 *       if (Bus::isr()) { brio::post<BusAo>(brio::TransferDone{Bus::status()}); }
 *   }
 *   extern "C" void FMPI2C1_ER_IRQHandler() {
 *       if (Bus::error_isr()) { brio::post<BusAo>(brio::TransferDone{Bus::status()}); }
 *   }
 */
template <uint8_t n, FmpI2cPins pins, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine>
class FmpI2cHost {
    using S = FmpI2c<n>;

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / DmaRxEngine "
                  "from stm32f4/dma.hpp, or NoDmaEngine (the default)");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio FmpI2cHost: name both DMA engines or neither - a write-then-read "
                  "tenure needs both directions inside one tenure, and one engine would have "
                  "to hand over to the byte pump mid-transfer");
    static_assert(fmpi2c_engines_distinct<TxEngine, RxEngine>(),
                  "brio FmpI2cHost: the transmit and receive engines must use DIFFERENT DMA "
                  "streams - a stream carries one direction and has one FIFO");
    static_assert(fmpi2c_engine_placed<TxEngine, true>(),
                  "brio FmpI2cHost: the transmit engine's (controller, stream, channel) is not "
                  "the cell this block's transmit request is wired to - RM0390 table 28 gives "
                  "FMPI2C1_TX one cell and one only (DMA1 stream 5, channel 2), and a part "
                  "class whose request table was not read has no cells at all, where an engine "
                  "is refused rather than guessed");
    static_assert(fmpi2c_engine_placed<RxEngine, false>(),
                  "brio FmpI2cHost: the receive engine's (controller, stream, channel) is not "
                  "the cell this block's receive request is wired to - RM0390 table 28 gives "
                  "FMPI2C1_RX DMA1 stream 2, channel 2; see the transmit engine's message");
    static_assert(fmpi2c_pins_valid(pins),
                  "brio FmpI2cHost: these FMPI2C pads are not a bus - SCL and SDA must both be "
                  "named, must be real pins of ports this part bonds, and no two signals may "
                  "name the same pad");

    using SclPin = Pin<pins.scl.port, pins.scl.pin>;
    using SdaPin = Pin<pins.sda.port, pins.sda.pin>;
    using SmbaPin = Pin<pins.smba.valid() ? pins.smba.port : pins.scl.port,
                        pins.smba.valid() ? pins.smba.pin : pins.scl.pin>;

public:
    FmpI2cHost() = delete;

    using Resource = S;
    static constexpr FmpI2cPins pin_sel = pins;
    static constexpr bool has_engines = TxEngine::present;

    struct Request {
        uint8_t addr;   ///< 7-bit client address (unshifted)
        /// Bytes written after START, LENT until the reply lands (may be
        /// null if tx_len == 0).
        Borrowed<const uint8_t, Lease::reply> tx;
        uint8_t tx_len;
        /// Where the bytes read after the (repeated) START+R land; LENT
        /// until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint8_t rx_len;
        ReplyTo<I2cDone> reply;
        FmpI2cSpeed speed = FmpI2cSpeed::standard_100k;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle ----------------------------------------------------------------

    /**
     * Bring the instance up as a bus host.
     *
     * `clock` is the app's Clock tag; the timing arithmetic is solved
     * against the KERNEL clock, which cfg.kernel selects and may be this
     * instance's APB1 clock, SYSCLK or the HSI. The HSI is worth choosing
     * for what an independent clock is for: it keeps the bus alive across a
     * change of the core's rate, and it holds a speed ES0298 2.12.2 would
     * otherwise refuse at a slow core. What it costs on a 180 MHz part is
     * ES0298 2.12.8's ratio - transmit_stall_risk() answers.
     *
     * `cfg.bus` is the BUS'S own edges and the SCL detection budget: a rise
     * time the bus does not have lands tSCLL below the specification floor
     * by the difference. Nullopt (the default) means "the standard's own
     * numbers and the manual's own tSYNC budget for each mode", which
     * reproduces tables 134 and 135.
     *
     * The three speeds' TIMINGR values are resolved here (and at rebase());
     * one this kernel clock cannot produce is marked unreachable -
     * speed_ok() tells, and a Request naming it is answered i2c_rejected
     * rather than run at a rate nobody asked for.
     *
     * Returns false when not even standard_100k is reachable, or when the
     * boot configuration is refused.
     */
    template <typename Clock>
    static bool init(Clock clock, const FmpI2cHostConfig& cfg = {}) {
        static_assert(clock_follows<Clock, FmpI2cHost>(),
                      "this FmpI2cHost is initialized with a DynamicClock that does not list it "
                      "among its Users: its TIMINGR table would go stale on a clock change");
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        cfg_ = cfg;
        S::bus_clock(true);
        S::reset();
        if (!S::kernel_clock(cfg.kernel)) {
            return false;
        }
        rebase_rates(apb_hz(clock, false), clock_hz(clock));
        if (!valid_[0]) {
            return false;   // not even a 100 kHz bus is legal at this kernel clock
        }
        // THE PADS GO OVER FIRST, and BUSY is the reason - the same trap
        // this stratum's other I2C sets (docs/stm32f4/i2c.md). BUSY is set
        // "by hardware when a START condition is detected" on the
        // PERIPHERAL'S inputs, and those read low while a pad is not in its
        // alternate function. Here 23.4.6 gives a way out that the F1
        // lineage's block does not have - PE = 0 clears BUSY - so the order
        // is belt and braces: pads first, then the PE cycle at the end of
        // this function starts the machine over an idle wire.
        claim_pads();
        applied_ = FmpI2cSpeed::standard_100k;
        FmpI2cConfig c{};
        c.timing = table_[0];
        c.filters = cfg_.filters;
        if (!S::configure(c)) {
            return false;
        }
        S::enable();
        if constexpr (has_engines) {
            // The engines are pointed at THIS instance's two data registers.
            // Unlike the other I2C's one DR, this block has a separate TXDR
            // and RXDR, so the two engines never share an address.
            TxEngine::arm(S::tx_address());
            RxEngine::arm(S::rx_address());
        }
        status_ = i2c_ok;
        phase_ = Phase::idle;
        berr_count_ = 0;
        // TCIE is NOT among them: start() arms it for the one tenure shape
        // that owns a TC and finish() takes it away again.
        S::interrupt(FmpI2cInterrupt::rx | FmpI2cInterrupt::tx | FmpI2cInterrupt::nack |
                         FmpI2cInterrupt::stop | FmpI2cInterrupt::error,
                     true);
        S::interrupt(FmpI2cInterrupt::transfer_complete, false);
        Nvic::enable(S::event_irq);
        Nvic::enable(S::error_irq);
        return true;
    }

    /**
     * The core clock changed (DynamicClock fan-out): re-solve the three
     * speeds' TIMINGR values against the new KERNEL rate. `hz` is SYSCLK, as
     * every other task's rebase takes it - the APB1 divider in force is read
     * back from the RCC.
     *
     * A port on the HSI FOLDS TO NOTHING - its kernel clock did not move -
     * which is the whole reason a bus can outlive a rate change that would
     * otherwise refuse it. A speed this kernel clock cannot produce is
     * marked unreachable, not an error.
     *
     * THE BUS MUST BE IDLE: TIMINGR is PE-gated, so applying a new value
     * costs a PE cycle, and a PE cycle mid-tenure releases both lines.
     *
     * VOID, because util/clock.hpp's ClockUser concept says so - the fan-out
     * has no caller to report to. What a rate change can COST is asked
     * afterwards, and speed_ok() is the verb.
     */
    static void rebase(uint32_t hz) {
        const bool was_enabled = S::enabled();
        rebase_rates(hz / Rcc::apb1_divider(), hz);
        if (was_enabled && valid_[static_cast<uint8_t>(applied_)]) {
            const FmpI2cSpeed keep = applied_;
            applied_ = static_cast<FmpI2cSpeed>(0xFFu);   // force apply() to act
            apply(keep);
        }
    }

    /// Can this kernel clock produce that speed at all? (ES0298 2.12.2's
    /// floors drop the fast end of the vocabulary first: on the HSI's
    /// 16 MHz, Fm+ is gone.)
    static bool speed_ok(FmpI2cSpeed s) { return valid_[static_cast<uint8_t>(s)]; }
    /// What a speed really runs at on this bus - the produced rate, not the
    /// asked one.
    static uint32_t scl_hz(FmpI2cSpeed s) {
        return cfg_.bus
                   ? fmpi2c_scl_hz(ker_hz_, table_[static_cast<uint8_t>(s)], cfg_.bus->sync_ns)
                   : fmpi2c_scl_hz(ker_hz_, table_[static_cast<uint8_t>(s)], s);
    }
    static FmpI2cTiming timing_of(FmpI2cSpeed s) { return table_[static_cast<uint8_t>(s)]; }
    /// The rate the arithmetic divides: this instance's KERNEL clock.
    static uint32_t kernel_hz() { return ker_hz_; }
    /// The rate its registers are on, which is the other half of ES0298
    /// 2.12.8's ratio.
    static uint32_t reference_hz() { return pclk_hz_; }
    /// ES0298 2.12.8: whether the two rates in force sit in the band where a
    /// transmission can stall after its first byte.
    static bool transmit_stall_risk() {
        return fmpi2c_transmit_stall_risk(pclk_hz_, ker_hz_);
    }

    /// The Fm+ pad drive, which a fast_plus_1m bus needs. Both signals at
    /// once: SYSCFG's two bits are per SIGNAL and not per pad.
    static void fast_plus_drive(bool on) { S::fast_plus_drive(on, on); }
    static bool fast_plus_drive() { return S::fast_plus_scl() && S::fast_plus_sda(); }

    /// The engine is between tenures. BusMaster serializes, so this is a
    /// convenience for suites, not a lock.
    static bool idle() { return phase_ == Phase::idle; }
    /// How many times ES0298 2.12.3's spurious BERR has been swept since
    /// init(). Not a status and not an error - a counter, so a bench can say
    /// how often the erratum fires on this die.
    static uint16_t spurious_bus_errors() { return berr_count_; }

    /// Hand the SMBA pad to the peripheral - only for a program that has put
    /// the resource in SMBus alert mode. The task's own tenures never use it.
    static void claim_smba_pad(bool on) {
        if constexpr (pins.smba.valid()) {
            if (on) {
                SmbaPin::function(pins.smba.function, pad_config());
            } else {
                SmbaPin::release();
            }
        } else {
            (void)on;
        }
    }

    // ---- the transfer -------------------------------------------------------------

    /// Begin one bus tenure (called by I2cBus from main context). Returns
    /// false whenever the wire moves - the tenure runs on the ISR and a
    /// TransferDone{status()} follows - and true for the two failures that
    /// move nothing, answered through status(): a speed this kernel clock
    /// cannot produce (i2c_rejected), and a START still standing from a
    /// tenure the peripheral never closed (i2c_bus_error). That is the
    /// I2cHost contract (docs/design/i2c-bus.md).
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
        S::clear(FmpI2cClear::all);
        const bool opens_read = (r.tx_len == 0u && r.rx_len != 0u);
        phase_ = opens_read ? Phase::reading : Phase::writing;
        // AUTOEND on every shape but a write that a read half follows: there
        // the software end is what puts TC in our hands, and setting START
        // at TC IS the repeated START.
        const bool two_phase = (r.tx_len != 0u && r.rx_len != 0u);
        const uint8_t nbytes = opens_read ? r.rx_len : r.tx_len;
        // TCIE IS ARMED ONLY FOR THE TENURE THAT NEEDS IT - the
        // write-then-read, whose TC is where the repeated START is issued.
        // Every other shape ends on AUTOEND's own STOP and never raises TC,
        // and an armed TCIE with no owner is a storm the idle sweep cannot
        // clear its way out of (TC has no ICR bit).
        S::interrupt(FmpI2cInterrupt::transfer_complete, two_phase);
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

    /// The EVENT vector's body - call from FMPI2C1_EV. Returns true when the
    /// tenure just completed: the edge on which the app's glue posts
    /// TransferDone to the bus AO.
    [[gnu::always_inline]] static bool isr() {
        const uint32_t p = S::pending_event();
        if (p == 0u) {
            return false;
        }
        if (phase_ == Phase::idle) {
            // A flag with no tenure owning it. SWEEP IT, or a level holds
            // the NVIC line and the handler storms.
            //
            // AND A SWEEP IS NOT ENOUGH ON THIS PERIPHERAL: TC, TCR, TXIS
            // and RXNE HAVE NO ICR BIT. 23.7.8's clear register has one bit
            // for ADDR, NACKF, STOPF and each of the six errors and nothing
            // else, because the chapter clears the other four by an access,
            // a START, a STOP or an NBYTES write - none of which is legal
            // with no tenure in flight. So a byte left in RXDR is cleared by
            // READING it, and what cannot be cleared at all is DISARMED.
            S::clear(FmpI2cClear::all);
            if ((S::flags() & FMPI2C_ISR_RXNE) != 0u) {
                (void)S::data();
            }
            if (S::pending_event() != 0u) {
                S::interrupt(FmpI2cInterrupt::transfer_complete, false);
            }
            return false;
        }

        if ((p & FmpI2cFlag::nack) != 0u) {
            // The position tells which byte went unanswered: at pos_ == 0 of
            // the opening half nothing has moved, so it was the ADDRESS -
            // the probe's answer and the nobody-home case alike. The
            // peripheral sends the STOP by itself (23.4.9), so this only
            // records and STOPF completes.
            S::clear(FmpI2cClear::nack);
            // pos_ IS THE WHOLE TEST, and it works for both halves of a
            // write-then-read because the repeated START resets it: a NACK
            // with nothing moved in this half is a NACK on the address that
            // opened it.
            status_ = pos_ == 0u ? i2c_nack_addr : i2c_nack_data;
            // AND IT RETURNS. The STOP the peripheral sends by itself
            // arrives as its OWN interrupt a few microseconds later, and a
            // branch that fell through to the sweep at the bottom would
            // clear that STOPF the moment it set - leaving the tenure with
            // its status recorded, its flags gone and no completion for
            // ever.
            return false;
        }
        // THE DATA COMES BEFORE THE END, and the order is not a taste. RXNE
        // and STOPF can stand TOGETHER - the last byte of a read is in RXDR
        // at the instant the automatic STOP goes out - and RXNE is cleared
        // by READING RXDR and by nothing else. A handler that served STOPF
        // first would lose that byte AND leave RXNE standing on a
        // level-driven vector, which is an endless handler.
        if ((p & FmpI2cFlag::rxne) != 0u) {
            const uint8_t v = S::data();
            if (req_.rx.get() != nullptr && pos_ < req_.rx_len) {
                req_.rx.get()[pos_] = v;
            }
            ++pos_;
            return false;
        }
        if ((p & FmpI2cFlag::txis) != 0u) {
            if (pos_ < req_.tx_len && req_.tx.get() != nullptr) {
                S::data(req_.tx.get()[pos_]);
            } else {
                S::data(0xFFu);
            }
            ++pos_;
            return false;
        }
        if ((p & FmpI2cFlag::transfer_complete) != 0u) {
            // The write half is done and a read half follows: the repeated
            // START is the second CR2 write, in ONE store - see
            // transfer_now(). Two stores would put a STOP here instead.
            phase_ = Phase::reading;
            pos_ = 0;
            (void)S::transfer_now(req_.addr, true, req_.rx_len, true);
            return false;
        }
        if ((p & FmpI2cFlag::stop) != 0u) {
            S::clear(FmpI2cClear::stop);
            return finish(status_);
        }
        // THE LAST RESORT, and it is not decoration. On a level-driven
        // vector an unexpected flag is an INFINITE HANDLER, which starves
        // the program so completely that nothing can even report it. The
        // only event flags no branch above owns are ADDR and TCR, and
        // neither should be able to arrive: a host enables neither its own
        // address nor ADDRIE, so nothing can match, and it never sets
        // RELOAD, so TCR never rises. What cannot be cleared is therefore
        // DISARMED - TC and TCR have no ICR bit at all.
        //
        // AND ADDRCF IS DELIBERATELY NOT WRITTEN HERE, though it is the one
        // clear that would reach the other of the two: 23.7.8 makes it
        // clear CR2's START as well, which would abort the address phase of
        // the very tenure that is running.
        S::interrupt(FmpI2cInterrupt::transfer_complete, false);
        return false;
    }

    /**
     * The ERROR vector's body - call from FMPI2C1_ER. True when the tenure
     * just completed.
     *
     * A BUS ERROR does NOT end a tenure: ES0298 2.12.3 says one can be
     * raised where there was none and that the transfer "continues
     * normally", so the flag is cleared, counted, and the tenure runs on.
     * Lost arbitration ends it with the hardware having already released the
     * bus. The SMBus half's three - OVR, PECERR, TIMEOUT and ALERT - ride
     * the same ERRIE and are SWEPT rather than acted on: none of them can
     * arise in a plain controller tenure that armed no time-out, and a
     * tenure that stops answering belongs to the arbiter's timeout, which
     * ends it with i2c_timeout and calls recover().
     */
    [[gnu::always_inline]] static bool error_isr() {
        const uint32_t p = S::pending_error();
        if (p == 0u) {
            return false;
        }
        if ((p & FmpI2cFlag::bus_error) != 0u) {
            S::clear(FmpI2cClear::bus_error);
            ++berr_count_;
        }
        if ((p & FmpI2cFlag::arb_lost) != 0u) {
            // The bus is no longer ours: the silicon has released the lines
            // and switched to target mode, so no STOP is ours to send and no
            // STOPF will come.
            S::clear(FmpI2cClear::arb_lost);
            if (phase_ != Phase::idle) {
                return finish(i2c_arb_lost);
            }
            return false;
        }
        S::clear(FmpI2cClear::overrun | FmpI2cClear::pec_error | FmpI2cClear::timeout |
                 FmpI2cClear::alert);
        return false;
    }

    /**
     * The DMA streams' interrupt body - call from BOTH streams' vectors (one
     * vector per stream on this family, shared with nothing). Compiles away
     * on an engineless host.
     *
     * A transfer error on either stream ends the tenure with
     * fmpi2c_dma_fault and a software STOP, so the bus is released rather
     * than left to a stream that will never be served again. The receive
     * side needs no completion verb: its block IS the Request's rx span,
     * already filled in place, and the tenure's end is STOPF either way.
     *
     * Returns true when the tenure just completed that way.
     */
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            const uint8_t tx = TxEngine::service();
            const uint8_t rx = RxEngine::service();
            if ((tx & TxEngine::flag_complete) != 0u) {
                (void)TxEngine::complete();
            }
            if ((rx & RxEngine::flag_complete) != 0u) {
                (void)RxEngine::take();
            }
            if (((tx & TxEngine::flag_error) | (rx & RxEngine::flag_error)) != 0u) {
                if (phase_ != Phase::idle) {
                    put_engines_away();
                    S::stop();
                    return finish(fmpi2c_dma_fault);
                }
            }
        }
        return false;
    }

    /**
     * The classic bus unstick: nine SCL pulses and a STOP, by hand,
     * open-drain, with the pads reclaimed from the peripheral for the
     * duration. RECOVER() FIXES THE PERIPHERAL, THIS FIXES THE WIRE.
     *
     * Returns the number of pulses it took a stuck client to release SDA
     * (0 = the wire was never stuck), or 0xFF when nine pulses and a STOP
     * left SDA still low - a short, not a client.
     *
     * A HEALTHY WIRE IS LEFT ALONE: SDA already high means nothing is stuck,
     * and zero pulses is both the answer and the action.
     */
    static uint8_t unstick() {
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        // The pads back to GPIO, OPEN DRAIN with ODR high: releasing is the
        // pull-ups' job and driving low is ours.
        SclPin::output(true, pad_config());
        SdaPin::output(true, pad_config());

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

        claim_pads();
        S::clear(FmpI2cClear::all);
        Nvic::enable(S::event_irq);
        Nvic::enable(S::error_irq);
        if (!free_now) {
            return 0xFF;
        }
        return released_at == 0xFF ? 0u : released_at;
    }

    /**
     * Put the ENGINE back where start() is legal - the verb a timed I2cBus
     * calls on a tenure that never answered (util/bus_master.hpp).
     *
     * It is 23.4.6's PE cycle and nothing more, because 23.4.6 is exactly
     * what the chapter offers for this. The cycle releases both lines,
     * resets the state machines and clears every status flag - and KEEPS
     * every configuration register, so the timings, the filters and the pads
     * survive it and nothing is reconfigured. It is also the escape ES0298
     * 2.12.1 and 2.12.8 both prescribe.
     *
     * THE WIRE IS NOT ITS JOB: a client still holding SDA makes the next
     * tenure report, and unstick() is the wire's verb.
     *
     * Returns false when PE would not read back clear.
     */
    static bool recover() {
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        phase_ = Phase::idle;
        if constexpr (has_engines) {
            put_engines_away();
            TxEngine::arm(S::tx_address());
            RxEngine::arm(S::rx_address());
        }
        S::interrupt(FmpI2cInterrupt::transfer_complete, false);
        const bool ok = S::cycle();
        S::clear(FmpI2cClear::all);
        Nvic::enable(S::event_irq);
        Nvic::enable(S::error_irq);
        return ok;
    }

    static void release() {
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        S::release();
        SclPin::release();
        SdaPin::release();
        if constexpr (pins.smba.valid()) {
            SmbaPin::release();
        }
        phase_ = Phase::idle;
    }

private:
    enum class Phase : uint8_t { idle, writing, reading };

    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, uint8_t> &&
                   std::is_same_v<typename RxEngine::element, uint8_t>;
        } else {
            return true;
        }
    }(), "brio FmpI2cHost: the DMA engines must carry uint8_t elements - TXDR and RXDR are one "
         "byte wide");

    /// What both pads are configured as. OPEN DRAIN is not an option here
    /// but the definition of the bus: the pull-ups own the idle level and a
    /// push-pull driver would fight them. The internal pull-up is a BENCH
    /// convenience for a wire that has none of its own - some tens of
    /// kiloohms, far too weak for an I2C edge - and it is off by default.
    static constexpr PinConfig pad_config() {
        return PinConfig{.pull = PinPull::none, .open_drain = true,
                         .speed = PinSpeed::very_high};
    }
    static void claim_pads() {
        PinConfig c = pad_config();
        if (cfg_.internal_pull_up) {
            c.pull = PinPull::up;
        }
        SclPin::function(pins.scl.function, c);
        SdaPin::function(pins.sda.function, c);
    }

    /// The one exit of every tenure: the status set, the phase idled, and
    /// every flag swept - a leftover level storms the vector.
    static bool finish(uint8_t st) {
        status_ = st;
        phase_ = Phase::idle;
        // The one interrupt this engine arms per tenure goes away with the
        // tenure (see start()).
        S::interrupt(FmpI2cInterrupt::transfer_complete, false);
        if constexpr (has_engines) {
            put_engines_away();
        }
        S::clear(FmpI2cClear::all);
        if ((S::flags() & FMPI2C_ISR_RXNE) != 0u) {
            (void)S::data();   // an unwanted byte is cleared by reading it
        }
        return true;
    }

    static void launch_dma() {
        if constexpr (has_engines) {
            if (req_.rx_len != 0u && req_.rx.get() != nullptr) {
                (void)RxEngine::start(req_.rx.get(), req_.rx_len);
                S::dma_receive(true);
                S::interrupt(FmpI2cInterrupt::rx, false);
            }
            if (req_.tx_len != 0u && req_.tx.get() != nullptr) {
                (void)TxEngine::start(req_.tx.get(), req_.tx_len);
                S::dma_transmit(true);
                S::interrupt(FmpI2cInterrupt::tx, false);
            }
        }
    }

    static void put_engines_away() {
        if constexpr (has_engines) {
            (void)TxEngine::abandon();
            RxEngine::stop();
            S::dma_transmit(false);
            S::dma_receive(false);
            S::interrupt(FmpI2cInterrupt::rx | FmpI2cInterrupt::tx, true);
        }
    }

    /// TIMINGR is PE-gated, so a speed change costs a PE cycle - cached, so
    /// a run of requests at one speed costs nothing.
    static void apply(FmpI2cSpeed s) {
        if (s == applied_) {
            return;
        }
        applied_ = s;
        (void)S::disable();
        (void)S::timing(table_[static_cast<uint8_t>(s)]);
        S::enable();
    }

    static void rebase_rates(uint32_t pclk, uint32_t sysclk) {
        pclk_hz_ = pclk;
        ker_hz_ = S::kernel_hz(pclk, sysclk);
        for (uint8_t i = 0; i < fmpi2c_speed_count; ++i) {
            const FmpI2cSpeed s = static_cast<FmpI2cSpeed>(i);
            const auto t = cfg_.bus ? fmpi2c_timing_for(ker_hz_, s, cfg_.filters, *cfg_.bus)
                                    : fmpi2c_timing_for(ker_hz_, s, cfg_.filters);
            valid_[i] = t.has_value();
            table_[i] = t.value_or(FmpI2cTiming{});
        }
        delay_rate_ = delay_rate(sysclk);   // the busy-waits count CORE cycles
    }

    /// Half a standard-mode bit, for the unstick - timed by the core's own
    /// ruler because this is a recovery path that must not depend on the
    /// peripheral being alive.
    static void spin_half_bit() { (void)delay_us(delay_rate_, 5); }

    static inline Request req_{};
    static inline uint8_t pos_ = 0;
    static inline volatile Phase phase_ = Phase::idle;
    static inline uint8_t status_ = i2c_ok;
    static inline uint16_t berr_count_ = 0;
    static inline FmpI2cSpeed applied_ = FmpI2cSpeed::standard_100k;
    static inline FmpI2cTiming table_[fmpi2c_speed_count]{};
    static inline bool valid_[fmpi2c_speed_count]{};
    static inline FmpI2cHostConfig cfg_{};
    static inline uint32_t pclk_hz_ = 0;
    static inline uint32_t ker_hz_ = 0;
    static inline DelayRate delay_rate_{};
};

// =============================================================================
// The client task
// =============================================================================

/// What the client's ISR bodies report: one thing per call, the app's glue
/// acts on it. The same vocabulary as this stratum's I2cClient, so a
/// protocol written over one compiles over the other.
enum class FmpI2cClientEvent : uint8_t {
    none,
    addressed,       ///< ADDR: a tenure opened on one of the own addresses
    byte_received,   ///< RXNE: take() it
    byte_wanted,     ///< TXIS: give() the next
    stop,            ///< STOPF: the tenure ended
    nacked,          ///< NACKF: the controller refused a byte this client gave
    error,           ///< BERR, ARLO or OVR
};

/**
 * FmpI2cClient<n, pins>
 *
 * The target role: the polled surface plus two ISR bodies, deliberately
 * thin - the SpiClient position, because a client is a protocol and the
 * protocol is the application's.
 *
 * WHAT IT ADDS over the raw FmpI2c<n> is the address machinery in one place
 * and the two rules that are easy to get wrong:
 *
 *  - AN ADDRESS MATCH HOLDS THE BUS. With NOSTRETCH clear (the default) SCL
 *    stays low from the match until ADDR is cleared, so answer_address() is
 *    what releases the wire - and a client that takes its time costs the
 *    controller exactly that.
 *  - WITH NOSTRETCH SET NOTHING WAITS. 23.4.8: the byte must be in TXDR
 *    before the first SCL pulse of its own transfer, or OVR is raised and
 *    0xFF goes out in its place. ES0298 2.12.7 makes it worse - in a narrow
 *    window the byte goes out as 0xFF and the OVR flag is NOT set - so a
 *    target transmitter that gives up the stretch can lie silently. The
 *    default here keeps the stretch.
 *
 * A TARGET CANNOT NACK ITS OWN ADDRESS on this peripheral: the address has
 * already been acknowledged by the time ADDR rises, and the flag is a report
 * rather than a decision point. Refusing a tenure is a DATA-phase act -
 * answer_byte(false) under byte control, or a NACK on the first byte.
 */
template <uint8_t n, FmpI2cPins pins>
class FmpI2cClient {
    using S = FmpI2c<n>;

    static_assert(fmpi2c_pins_valid(pins),
                  "brio FmpI2cClient: these FMPI2C pads are not a bus - SCL and SDA must both "
                  "be named, must be real pins of ports this part bonds, and no two signals "
                  "may name the same pad");

    using SclPin = Pin<pins.scl.port, pins.scl.pin>;
    using SdaPin = Pin<pins.sda.port, pins.sda.pin>;

public:
    FmpI2cClient() = delete;

    using Resource = S;
    static constexpr FmpI2cPins pin_sel = pins;

    /**
     * Bring the instance up as a bus target.
     *
     * A CLIENT NEEDS TIMINGR TOO, and that surprises people: SDADEL and
     * SCLDEL are the data hold and setup delays a TARGET applies, and 23.4.5
     * makes the minimum stretch after every falling edge
     * [(SDADEL + SCLDEL + 1) x (PRESC + 1) + 1] kernel periods. SCLL and
     * SCLH are the controller's alone and are ignored here. So `speed` names
     * the row of the standard the delays are solved against - the FASTEST
     * bus this target expects to sit on - and not a rate it generates.
     */
    template <typename Clock>
    static bool init(Clock clock, const FmpI2cAddressConfig& addr,
                     FmpI2cSpeed speed = FmpI2cSpeed::standard_100k,
                     const FmpI2cHostConfig& cfg = {}, bool no_stretch = false) {
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        S::bus_clock(true);
        S::reset();
        if (!S::kernel_clock(cfg.kernel)) {
            return false;
        }
        ker_hz_ = S::kernel_hz(apb_hz(clock, false), clock_hz(clock));
        const auto t = cfg.bus ? fmpi2c_timing_for(ker_hz_, speed, cfg.filters, *cfg.bus)
                               : fmpi2c_timing_for(ker_hz_, speed, cfg.filters);
        if (!t) {
            return false;
        }
        FmpI2cConfig c{};
        c.timing = *t;
        c.filters = cfg.filters;
        c.no_stretch = no_stretch;
        if (!S::configure(c)) {
            return false;
        }
        if (!S::addresses(addr)) {
            return false;
        }
        S::enable();
        PinConfig pad{.pull = cfg.internal_pull_up ? PinPull::up : PinPull::none,
                      .open_drain = true, .speed = PinSpeed::very_high};
        SclPin::function(pins.scl.function, pad);
        SdaPin::function(pins.sda.function, pad);
        Nvic::enable(S::event_irq);
        Nvic::enable(S::error_irq);
        return true;
    }

    /// The kernel rate the delays were solved against.
    static uint32_t kernel_hz() { return ker_hz_; }

    /// The general call, after init() - GCEN is not PE-gated.
    static void general_call(bool on) { S::general_call(on); }

    /// Target byte control: each received byte gated by NBYTES/RELOAD so
    /// software can ACK or NACK it. Needs stretching, and 23.4.8 wants
    /// NBYTES re-armed to 1 after every byte.
    static bool byte_control(bool on) {
        if (on && S::no_stretch()) {
            return false;   // 23.4.8: the two are not compatible
        }
        S::byte_control(on);
        return true;
    }

    // ---- the protocol surface -----------------------------------------------------

    /// An address match is pending (SCL held meanwhile, unless NOSTRETCH).
    static bool addressed() { return S::flag(FmpI2cFlag::addr); }
    /// The matched tenure's direction, valid at ADDR: true = the controller
    /// is reading, so this target transmits.
    static bool host_reads() { return S::host_reads(); }
    /// WHICH address matched (ADDCODE). In 10-bit mode it is the header plus
    /// the address's two MSBs, and a general call reads 0.
    static uint8_t matched_address() { return S::matched_address(); }
    /// Answer the match - and RELEASE THE CLOCK, which is the part that
    /// matters: with stretching on, SCL is low from the match until this
    /// call.
    static void answer_address() { S::clear(FmpI2cClear::addr); }

    /// Host write: a byte has arrived.
    static bool data_ready() { return S::flag(FmpI2cFlag::rxne); }
    static uint8_t take() { return S::data(); }
    /// Host read: the shifter wants the next byte.
    static bool data_wanted() { return S::flag(FmpI2cFlag::txis); }
    static void give(uint8_t v) { S::data(v); }
    /// Throw away a byte loaded for a tenure that is over - 23.7.7's own
    /// trick, and the reason a target that reuses TXDR between tenures does
    /// not send yesterday's byte.
    static void flush() { S::flush_tx(); }

    /// Under byte control: acknowledge or refuse the byte just taken, then
    /// release the stretch by re-arming NBYTES.
    static bool answer_byte(bool ack) {
        S::nack_next(!ack);
        return S::reload(1, true, false);
    }

    /// A STOP arrived - the end of a tenure. W1C.
    static bool stop_seen() { return S::flag(FmpI2cFlag::stop); }
    static void clear_stop() { S::clear(FmpI2cClear::stop); }
    /// The controller refused a byte this target transmitted: the tenure is
    /// over and the lines are already released.
    static bool host_nacked() { return S::flag(FmpI2cFlag::nack); }
    static void clear_nack() { S::clear(FmpI2cClear::nack); }
    /// NOSTRETCH mode's own failure: a byte that was not ready in time
    /// (0xFF went out), or one not read before the next arrived - when it is
    /// flagged at all (ES0298 2.12.7).
    static bool overrun() { return S::flag(FmpI2cFlag::overrun); }
    static void clear_overrun() { S::clear(FmpI2cClear::overrun); }

    static uint32_t flags() { return S::flags(); }
    static void clear(uint32_t icr_mask) { S::clear(icr_mask); }
    static void interrupt(uint32_t mask, bool on) { S::interrupt(mask, on); }

    // ---- the ISR bodies -------------------------------------------------------------

    /// The EVENT vector's body: one event per call, the flag consumed where
    /// the chapter's sequence consumes it (ADDR and STOPF here; RXNE by
    /// take(), TXIS by give(), which the glue owes).
    [[gnu::always_inline]] static FmpI2cClientEvent service() {
        const uint32_t p = S::pending_event();
        if ((p & FmpI2cFlag::addr) != 0u) {
            host_reads_ = S::host_reads();
            answer_address();
            return FmpI2cClientEvent::addressed;
        }
        if ((p & FmpI2cFlag::nack) != 0u) {
            S::clear(FmpI2cClear::nack);
            return FmpI2cClientEvent::nacked;
        }
        if ((p & FmpI2cFlag::rxne) != 0u) {
            return FmpI2cClientEvent::byte_received;
        }
        if ((p & FmpI2cFlag::txis) != 0u) {
            return FmpI2cClientEvent::byte_wanted;
        }
        if ((p & FmpI2cFlag::stop) != 0u) {
            S::clear(FmpI2cClear::stop);
            return FmpI2cClientEvent::stop;
        }
        return FmpI2cClientEvent::none;
    }

    /// The ERROR vector's body. A CLIENT'S BERR IS REPORTED, unlike a
    /// controller's: ES0298 2.12.3's spurious one is a controller-mode
    /// erratum, and 23.7.7 says the flag "is not set during the address
    /// phase in slave mode" - a target's bus error is a misplaced START or
    /// STOP inside a tenure it was part of.
    [[gnu::always_inline]] static FmpI2cClientEvent error_service() {
        const uint32_t p = S::pending_error();
        if (p == 0u) {
            return FmpI2cClientEvent::none;
        }
        S::clear(FmpI2cClear::errors);
        return FmpI2cClientEvent::error;
    }

    /// The direction of the tenure the last `addressed` opened: true = the
    /// controller reads, this side transmits.
    static bool host_reads_last() { return host_reads_; }

    static void release() {
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        S::release();
        SclPin::release();
        SdaPin::release();
    }

private:
    static inline bool host_reads_ = false;
    static inline uint32_t ker_hz_ = 0;
};

#endif // FMPI2C1_BASE

// =============================================================================
// Compile-time pins of the chapter's own arithmetic
// =============================================================================

// THE READBACK AGAINST TABLES 134 AND 135, cell by cell: every example
// register value, at its own kernel clock, with its own footnoted tSYNC
// budget, must price out at the column's own frequency. Table 134 first
// (8 MHz), Sm 10 kHz and 100 kHz.
static_assert(fmpi2c_scl_hz(8'000'000UL, FmpI2cTiming{0x1, 0xC7, 0xC3, 2, 4}, 1000u) == 10'000);
static_assert(fmpi2c_scl_hz(8'000'000UL, FmpI2cTiming{0x1, 0x13, 0xF, 2, 4}, 1000u) == 100'000);
// Fm 400 kHz and Fm+ 500 kHz (the 8 MHz table offers no 1 MHz column).
static_assert(fmpi2c_scl_hz(8'000'000UL, FmpI2cTiming{0x0, 0x9, 0x3, 1, 3}, 750u) == 400'000);
static_assert(fmpi2c_scl_hz(8'000'000UL, FmpI2cTiming{0x0, 0x6, 0x3, 0, 1}, 655u) == 500'000);
// Table 135 (16 MHz): the same four columns, the last of them at 1 MHz.
static_assert(fmpi2c_scl_hz(16'000'000UL, FmpI2cTiming{0x3, 0xC7, 0xC3, 2, 4}, 1000u) == 10'000);
static_assert(fmpi2c_scl_hz(16'000'000UL, FmpI2cTiming{0x3, 0x13, 0xF, 2, 4}, 1000u) == 100'000);
static_assert(fmpi2c_scl_hz(16'000'000UL, FmpI2cTiming{0x1, 0x9, 0x3, 2, 3}, 750u) == 400'000);
static_assert(fmpi2c_scl_hz(16'000'000UL, FmpI2cTiming{0x0, 0x4, 0x2, 0, 2}, 500u) == 1'000'000);

// The field times of table 135's Sm 100 kHz column, to the nanosecond.
static_assert(fmpi2c_scll_ns(16'000'000UL, FmpI2cTiming{0x3, 0x13, 0xF, 2, 4}) == 5000);
static_assert(fmpi2c_sclh_ns(16'000'000UL, FmpI2cTiming{0x3, 0x13, 0xF, 2, 4}) == 4000);
static_assert(fmpi2c_scldel_ns(16'000'000UL, FmpI2cTiming{0x3, 0x13, 0xF, 2, 4}) == 1250);
// SDADEL HAS NO +1 and this is where that shows: 2 x 250 ns, not 3 - which
// is 23.7.5's formula and the table's own product, against 23.4.5's prose.
static_assert(fmpi2c_sdadel_ns(16'000'000UL, FmpI2cTiming{0x3, 0x13, 0xF, 2, 4}) == 500);
// ... and of table 134's Fm column, where tSDADEL is one prescaler unit.
static_assert(fmpi2c_scll_ns(8'000'000UL, FmpI2cTiming{0x0, 0x9, 0x3, 1, 3}) == 1250);
static_assert(fmpi2c_sclh_ns(8'000'000UL, FmpI2cTiming{0x0, 0x9, 0x3, 1, 3}) == 500);
static_assert(fmpi2c_sdadel_ns(8'000'000UL, FmpI2cTiming{0x0, 0x9, 0x3, 1, 3}) == 125);
static_assert(fmpi2c_scldel_ns(8'000'000UL, FmpI2cTiming{0x0, 0x9, 0x3, 1, 3}) == 500);
// 23.4.5's minimum stretch, for the same value: (1 + 3 + 1) x 1 + 1 = 6
// kernel periods, which is 750 ns at 8 MHz.
static_assert(fmpi2c_min_stretch_cycles(FmpI2cTiming{0x0, 0x9, 0x3, 1, 3}) == 6);

// THE CHOOSER at the three kernel clocks a 180 MHz part can put under this
// block: its APB1 (45 MHz), SYSCLK (180 MHz) and the HSI (16 MHz). Every
// one lands inside the mode's band and NEVER ABOVE IT - the period is
// rounded up before it is split, so a kernel rate the speed does not divide
// (45 MHz against 400 kHz) still gives a bus no faster than the one asked
// for.
static_assert(fmpi2c_timing_for(45'000'000UL, FmpI2cSpeed::standard_100k).has_value());
static_assert(fmpi2c_scl_hz(45'000'000UL,
                            *fmpi2c_timing_for(45'000'000UL, FmpI2cSpeed::standard_100k),
                            FmpI2cSpeed::standard_100k) <= 100'000);
static_assert(fmpi2c_scl_hz(45'000'000UL,
                            *fmpi2c_timing_for(45'000'000UL, FmpI2cSpeed::standard_100k),
                            FmpI2cSpeed::standard_100k) >= 99'000);
static_assert(fmpi2c_scl_hz(45'000'000UL,
                            *fmpi2c_timing_for(45'000'000UL, FmpI2cSpeed::fast_400k),
                            FmpI2cSpeed::fast_400k) <= 400'000);
static_assert(fmpi2c_scl_hz(45'000'000UL,
                            *fmpi2c_timing_for(45'000'000UL, FmpI2cSpeed::fast_400k),
                            FmpI2cSpeed::fast_400k) >= 390'000);
static_assert(fmpi2c_scl_hz(45'000'000UL,
                            *fmpi2c_timing_for(45'000'000UL, FmpI2cSpeed::fast_plus_1m),
                            FmpI2cSpeed::fast_plus_1m) == 1'000'000);
static_assert(fmpi2c_scl_hz(180'000'000UL,
                            *fmpi2c_timing_for(180'000'000UL, FmpI2cSpeed::standard_100k),
                            FmpI2cSpeed::standard_100k) == 100'000);
static_assert(fmpi2c_scl_hz(180'000'000UL,
                            *fmpi2c_timing_for(180'000'000UL, FmpI2cSpeed::fast_400k),
                            FmpI2cSpeed::fast_400k) == 400'000);
static_assert(fmpi2c_scl_hz(180'000'000UL,
                            *fmpi2c_timing_for(180'000'000UL, FmpI2cSpeed::fast_plus_1m),
                            FmpI2cSpeed::fast_plus_1m) <= 1'000'000);
// On the HSI the chooser reproduces the manual's own TIMES for Fm at
// 16 MHz - twenty kernel cycles low and eight high - with a finer prescaler
// than table 135's, which is the whole difference between solving the
// inequalities and hand-picking a row.
static_assert(fmpi2c_scll_ns(16'000'000UL,
                             *fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::fast_400k)) == 1250);
static_assert(fmpi2c_sclh_ns(16'000'000UL,
                             *fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::fast_400k)) == 500);
// The chosen values meet 23.4.5's own two conditions.
static_assert(fmpi2c_setup_ok(45'000'000UL,
                              *fmpi2c_timing_for(45'000'000UL, FmpI2cSpeed::fast_400k),
                              fmpi2c_bus_timing(FmpI2cSpeed::fast_400k)));
static_assert(fmpi2c_hold_ok(45'000'000UL,
                             *fmpi2c_timing_for(45'000'000UL, FmpI2cSpeed::fast_400k),
                             FmpI2cFilters{}, fmpi2c_bus_timing(FmpI2cSpeed::fast_400k)));

// ES0298 2.12.2's floors, as refusals: the HSI's 16 MHz cannot do Fm+, and
// a 2 MHz kernel cannot do any of the three - which is what makes the
// independent clock worth having on a board that scales its core down.
static_assert(!fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::fast_plus_1m).has_value());
static_assert(fmpi2c_timing_for(16'000'000UL, FmpI2cSpeed::fast_400k).has_value());
static_assert(!fmpi2c_timing_for(2'000'000UL, FmpI2cSpeed::standard_100k).has_value());
static_assert(!fmpi2c_timing_for(2'000'000UL, FmpI2cSpeed::fast_400k).has_value());
static_assert(!fmpi2c_timing_for(2'000'000UL, FmpI2cSpeed::fast_plus_1m).has_value());
// The erratum's floor beats the datasheet's APB floor in every mode.
static_assert(fmpi2c_min_kernel_hz(FmpI2cSpeed::standard_100k) >
              fmpi2c_datasheet_min_pclk_hz(FmpI2cSpeed::standard_100k));
static_assert(fmpi2c_min_kernel_hz(FmpI2cSpeed::fast_400k) >
              fmpi2c_datasheet_min_pclk_hz(FmpI2cSpeed::fast_400k));
// A digital filter deep enough to eat the whole low period is refused by
// 23.4.3's own condition and not by an arbitrary bound.
static_assert(!fmpi2c_timing_for(20'000'000UL, FmpI2cSpeed::fast_plus_1m,
                                 FmpI2cFilters{true, 15}).has_value());
static_assert(!fmpi2c_filters_valid(FmpI2cFilters{true, 16}));

// ES0298 2.12.8's band, on the rates a 180 MHz part offers: the HSI against
// a 45 MHz APB1 is a ratio of 2.8 and sits INSIDE it; the APB clock itself
// (ratio 1) and SYSCLK (ratio 0.25) do not.
static_assert(fmpi2c_transmit_stall_risk(45'000'000UL, 16'000'000UL));
static_assert(!fmpi2c_transmit_stall_risk(45'000'000UL, 45'000'000UL));
static_assert(!fmpi2c_transmit_stall_risk(45'000'000UL, 180'000'000UL));
// The band's two edges, exactly: 1.5 and 3 are both inside it.
static_assert(fmpi2c_transmit_stall_risk(24'000'000UL, 16'000'000UL));
static_assert(fmpi2c_transmit_stall_risk(48'000'000UL, 16'000'000UL));
static_assert(!fmpi2c_transmit_stall_risk(48'000'001UL, 16'000'000UL));

// THE SMBus TIME-OUTS against tables 138, 139 and 140, cell by cell.
static_assert(*fmpi2c_timeout_code_for(8'000'000UL, 25'000u, false) == 0x61);
static_assert(*fmpi2c_timeout_code_for(16'000'000UL, 25'000u, false) == 0xC3);
static_assert(*fmpi2c_timeout_code_for(8'000'000UL, 8'000u, false) == 0x1F);
// Table 139's 16 MHz row is the ONE cell of the three time-out tables this
// arithmetic does not reproduce exactly, and the difference is ST's
// rounding and not an error: 8 ms at 16 MHz needs 62.5 units, so the
// tightest code that covers it is 63 (8.064 ms) while the table prints 64
// (0x3F, 8.192 ms). This chooser gives at least what was asked and no more,
// so it lands one unit below the table.
static_assert(*fmpi2c_timeout_code_for(16'000'000UL, 8'000u, false) == 0x3E);
static_assert(fmpi2c_timeout_us(16'000'000UL, 0x3E, false) >= 8'000);
static_assert(fmpi2c_timeout_us(16'000'000UL, 0x3E, false) <
              fmpi2c_timeout_us(16'000'000UL, 0x3F, false));
static_assert(*fmpi2c_timeout_code_for(8'000'000UL, 50u, true) == 0x63);
static_assert(*fmpi2c_timeout_code_for(16'000'000UL, 50u, true) == 0xC7);
// ... and back: what a code really produces.
static_assert(fmpi2c_timeout_us(16'000'000UL, 0xC3, false) == 25'088);
static_assert(fmpi2c_timeout_us(8'000'000UL, 0x61, false) == 25'088);
static_assert(fmpi2c_timeout_us(16'000'000UL, 0xC7, true) == 50);
// Twelve bits do not reach a second at 45 MHz: refused, not truncated.
static_assert(!fmpi2c_timeout_code_for(45'000'000UL, 1'000'000u, false).has_value());
static_assert(fmpi2c_timeout_code_for(45'000'000UL, 150'000u, false).has_value());

// The addresses and their masks.
static_assert(fmpi2c_address_valid(0x7F, FmpI2cAddressMode::seven_bit));
static_assert(!fmpi2c_address_valid(0x80, FmpI2cAddressMode::seven_bit));
static_assert(fmpi2c_address_valid(0x3FF, FmpI2cAddressMode::ten_bit));
static_assert(!fmpi2c_address_valid(0x400, FmpI2cAddressMode::ten_bit));
static_assert(fmpi2c_oa2_compared_bits(FmpI2cOa2Mask::none) == 7);
static_assert(fmpi2c_oa2_compared_bits(FmpI2cOa2Mask::all) == 0);
static_assert(!fmpi2c_address_config_valid(FmpI2cAddressConfig{.own = 0x80}));
static_assert(fmpi2c_address_config_valid(
    FmpI2cAddressConfig{.own = 0x123, .mode = FmpI2cAddressMode::ten_bit, .second = 0x42}));

// The configuration's own two refusals.
static_assert(!fmpi2c_config_valid(FmpI2cConfig{.no_stretch = true, .byte_control = true}));
static_assert(fmpi2c_config_valid(FmpI2cConfig{.byte_control = true}));

// The pads.
static_assert(!fmpi2c_pins_valid(FmpI2cPins{}));
static_assert(!fmpi2c_pins_valid(FmpI2cPins{.scl = {'C', 6, PinFunction::af4},
                                            .sda = {'C', 6, PinFunction::af4}}));

} // namespace brio
