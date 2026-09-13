/*
 * i2c.hpp
 *
 * The I2C of the STM32F4 (RM0090 ch. 27, RM0390 ch. 24, RM0383 ch. 18 -
 * one chapter, three manuals, the same registers): up to three instances
 * of the F1 lineage's EVENT MACHINE, in the layers every brio bus driver
 * has (docs/design/i2c-bus.md):
 *
 *  I2c<n>                the RESOURCE: the register block, its APB1 gate
 *                        and its reset, its TWO vectors, and the whole
 *                        register description as verbs - the clock
 *                        arithmetic (FREQ, CCR under F/S and DUTY, TRISE),
 *                        the noise filters, 7- and 10-bit own addresses
 *                        with the dual address and the general call, the
 *                        control bits (START, STOP, ACK, POS), every flag
 *                        with the SEQUENCE that clears it, SMBus and PEC,
 *                        the DMA requests with LAST. It decides nothing.
 *  I2cHost<n, pins, ...> the ENGINE util/i2c_bus.hpp's I2cBus drives: the
 *                        Request is the other strata's VERBATIM, so an
 *                        application written over I2cBus on a Nucleo or a
 *                        Pico runs here unchanged.
 *  I2cClient<n, pins>    the target side, a thin polled surface with two
 *                        ISR bodies: a client is a protocol and which one
 *                        is the application's.
 *
 * THE MACHINE THIS CHAPTER PRESCRIBES is a set of software sequences,
 * each stretching SCL low until it completes (27.3.2 and 27.3.3): EV5 (SB
 * up: read SR1, write the address into DR), EV6 (ADDR up: read SR1 then
 * SR2), EV8 (TxE: write DR), EV8_2 (TxE and BTF: the last byte is out,
 * STOP or repeated START), EV7 (RxNE: read DR) - and a RECEIVE PROCEDURE
 * THAT DEPENDS ON THE COUNT: one byte wants ACK cleared BEFORE ADDR is
 * cleared and STOP set right after; two bytes want POS set with ACK, ACK
 * cleared after ADDR, and the pair read after one BTF; three or more run
 * on RxNE until three remain, then wait for BTF twice - the first clears
 * ACK and reads N-2, the second sets STOP and reads N-1 and N. All of it
 * is in `isr()`, phase by phase, and it is the reason ITBUFEN is switched
 * on and off DURING a tenure: TxE and RxNE interrupt only while the byte
 * pump needs them, BTF (under ITEVTEN alone) carries the rest.
 *
 * THE RATE IS THE INSTANCE'S OWN APB1 CLOCK, never SYSCLK, and this
 * chapter needs it stated THREE times over: FREQ carries it in whole
 * megahertz (2 to 50 - 27.6.2; below 2 MHz no speed is legal and below
 * 4 MHz Fm mode is not, and init() refuses both), CCR divides it into the
 * SCL period, and TRISE bounds the SCL feedback loop. The CCR arithmetic
 * is the F1 lineage's: standard mode divides by 2 x CCR, fast mode by
 * 3 x CCR at DUTY 2 and by 25 x CCR at DUTY 16/9, ROUNDED UP here so a bus
 * never runs faster than asked. TRISE IS AN ARGUMENT AND NOT AN
 * ASSUMPTION, as the bus edges are on the AVR's TWI: it is the wire's
 * maximum rise time in nanoseconds - the specification's own 1000 ns in Sm
 * and 300 ns in Fm by default - converted to APB periods and incremented
 * by one, which is what 27.6.9 asks for.
 *
 * THE NOISE FILTERS ARE NOT ON EVERY PART. I2C_FLTR - the analog filter's
 * off switch and the digital filter's length in APB periods - exists on
 * every part of the pack but the F405 class, and the reserve asks the
 * header for it (stm32f4/device_tables.hpp's `i2c_has_filter()`); the
 * verb refuses where the register is not there. Table 122's ceiling on
 * the digital filter is published as `i2c_digital_filter_max()` and is
 * ADVICE, not a refusal: it is the bound that keeps the data hold time
 * inside the bus specification with the analog filter off, and the
 * chapter itself allows higher values to a system that can take the
 * violation.
 *
 * THREE ERRATA OF THIS CHAPTER SHAPE THE CODE (ES0206 2.10, ES0298 2.11,
 * ES0287 2.9 - the same six items on all three parts):
 *  - A BUS ERROR CAN BE DETECTED SPURIOUSLY IN CONTROLLER MODE, and
 *    "detection of bus error has no effect on the I2C-bus transfer in
 *    controller mode". The manual says as much of a REAL one (27.3.4: in
 *    controller mode the lines are not released and the transfer is
 *    unaffected), so BERR is never a reason to abort a tenure here: the
 *    error body clears it, COUNTS it - `spurious_bus_errors()` - and lets
 *    the transfer run on. This is where this family's engine differs from
 *    the CH32V00x's, whose chapter has no such erratum.
 *  - A START CANNOT BE GENERATED AFTER A MISPLACED STOP, and the
 *    workaround is SWRST. That is exactly what `recover()` does, so a
 *    tenure that never answers and is timed out by the arbiter comes back
 *    through the errata's own escape.
 *  - THE REPEATED START'S SETUP TIME CAN BE VIOLATED in standard mode
 *    between 88 and 100 kHz. A write-then-read Request IS a repeated
 *    start, so the fact is published rather than hidden -
 *    `repeated_start_setup_at_risk(speed)` - and the workaround (Fm, or a
 *    slower Sm) is the application's to take.
 * Three more are the application's and live in docs/stm32f4/i2c.md: SMBus
 * 2.0 is not fully supported, a target transmitter with NOSTRETCH can
 * violate the data valid time, and a 5 V bus violates the rise times.
 *
 * TWO OPTIONAL DMA ENGINE SLOTS, the shape every brio bus task has: name
 * a stm32f4/dma.hpp DmaTxEngine and/or DmaRxEngine and the data phases
 * move without the CPU; name neither (the default) and every engine
 * branch disappears. A stream and a channel are not free - the request
 * mapping gives an instance one, two or three (controller, stream,
 * channel) cells and no others, all of them on DMA1 - so a slot is checked
 * against the reserve's I2C slice of those tables at compile time, and
 * refused on a part class whose manual was not read.
 *
 * THE PADS ARE THE APPLICATION'S, as `SpiPins` has them: SCL, SDA and the
 * optional SMBus alert as `PinSel`s, each with the alternate function the
 * DATASHEET gives that signal on that pad - AF4 for I2C1, I2C2 and I2C3,
 * with AF9 on the handful of pads (PB3, PB4, PB8, PB9 on some parts) that
 * only the datasheet's table can settle. The device header carries no pin
 * table at all, so nothing here can check an AF: the bench is the check.
 * Both lines go out OPEN DRAIN - that IS the bus - and the pull-ups are
 * external.
 *
 * NOT COVERED YET is at the end of docs/stm32f4/i2c.md.
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
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/i2c_bus.hpp"

namespace brio {

// =============================================================================
// The vocabulary
// =============================================================================

/// The SR1 bits, spelled once for the handlers and the suites (27.6.6).
struct I2cFlag {
    I2cFlag() = delete;
    static constexpr uint32_t start_bit = I2C_SR1_SB;
    static constexpr uint32_t address = I2C_SR1_ADDR;
    static constexpr uint32_t byte_finished = I2C_SR1_BTF;
    static constexpr uint32_t header10 = I2C_SR1_ADD10;
    static constexpr uint32_t stop = I2C_SR1_STOPF;
    static constexpr uint32_t rx_not_empty = I2C_SR1_RXNE;
    static constexpr uint32_t tx_empty = I2C_SR1_TXE;
    static constexpr uint32_t bus_error = I2C_SR1_BERR;
    static constexpr uint32_t arbitration_lost = I2C_SR1_ARLO;
    static constexpr uint32_t ack_failure = I2C_SR1_AF;
    static constexpr uint32_t overrun = I2C_SR1_OVR;
    static constexpr uint32_t pec_error = I2C_SR1_PECERR;
    static constexpr uint32_t timeout = I2C_SR1_TIMEOUT;
    static constexpr uint32_t smb_alert = I2C_SR1_SMBALERT;
    /// What ITERREN puts on the error vector - the seven rc_w0 bits.
    static constexpr uint32_t errors = bus_error | arbitration_lost | ack_failure | overrun |
                                       pec_error | timeout | smb_alert;
};

/// The two speeds 27.1 names. This block has no Fm+: 400 kHz is the top.
enum class I2cSpeed : uint8_t { standard_100k = 0, fast_400k = 1 };

inline constexpr uint8_t i2c_speed_count = 2;

constexpr uint32_t i2c_speed_hz(I2cSpeed s) {
    return s == I2cSpeed::fast_400k ? 400'000UL : 100'000UL;
}

/// Fast mode's two duty shapes (CCR.DUTY): Tlow/Thigh = 2, or 16/9.
enum class I2cDuty : uint8_t { ratio_2 = 0, ratio_16_9 = 1 };

/// What the three timing registers hold for one speed at one APB1 clock.
struct I2cTiming {
    uint8_t freq_mhz = 0;   ///< CR2.FREQ, the APB1 clock in whole MHz
    uint16_t ccr = 0;       ///< CCR with F/S and DUTY
    uint8_t trise = 0;      ///< TRISE, the rise time in APB periods plus one
};

/// 27.6.2's window for FREQ, and 27.3.2's floor per mode.
inline constexpr uint32_t i2c_min_pclk_hz = 2'000'000UL;
inline constexpr uint32_t i2c_min_fast_pclk_hz = 4'000'000UL;
inline constexpr uint32_t i2c_max_pclk_hz = 50'000'000UL;

/// CCR is twelve bits with a floor of 4 in Sm and 1 in Fm (27.6.8);
/// TRISE is six.
inline constexpr uint16_t i2c_ccr_mask = 0x0FFFu;
inline constexpr uint16_t i2c_ccr_min_standard = 4;
inline constexpr uint16_t i2c_ccr_min_fast = 1;
inline constexpr uint8_t i2c_trise_mask = 0x3Fu;

/// The I2C specification's maximum SCL rise times, which are what TRISE
/// is programmed from when the application states nothing else.
inline constexpr uint16_t i2c_standard_rise_ns = 1000;
inline constexpr uint16_t i2c_fast_rise_ns = 300;

/**
 * The chapter's arithmetic (27.6.8): standard mode SCL = pclk / (2 x CCR),
 * fast mode pclk / (3 x CCR) at DUTY 2 and pclk / (25 x CCR) at DUTY 16/9,
 * with CCR ROUNDED UP - a bus at most as fast as asked - and clamped to
 * the chapter's floor, which inside FREQ's own window never binds. TRISE
 * is `rise_ns` in whole APB periods plus one (27.6.9).
 *
 * Nullopt outside FREQ's window (2 to 50 MHz, and 4 MHz for Fm), when CCR
 * would not fit its twelve bits, or when TRISE would not fit its six -
 * every one of them a refusal rather than a value the register cannot
 * hold.
 */
constexpr std::optional<I2cTiming> i2c_timing_for(uint32_t pclk, I2cSpeed speed,
                                                  I2cDuty duty = I2cDuty::ratio_2,
                                                  uint16_t rise_ns = 0) {
    if (pclk < i2c_min_pclk_hz || pclk > i2c_max_pclk_hz) {
        return {};
    }
    const bool fast = speed == I2cSpeed::fast_400k;
    if (fast && pclk < i2c_min_fast_pclk_hz) {
        return {};
    }
    if (rise_ns == 0u) {
        rise_ns = fast ? i2c_fast_rise_ns : i2c_standard_rise_ns;
    }
    const uint32_t hz = i2c_speed_hz(speed);
    const uint32_t div = fast ? (duty == I2cDuty::ratio_16_9 ? 25UL : 3UL) * hz : 2UL * hz;
    uint32_t ccr = (pclk + div - 1UL) / div;
    const uint32_t floor = fast ? i2c_ccr_min_fast : i2c_ccr_min_standard;
    if (ccr < floor) {
        ccr = floor;   // slower than asked, never faster
    }
    if (ccr > i2c_ccr_mask) {
        return {};
    }
    // The product is nanoseconds times hertz: 1000 x 50e6 overflows 32
    // bits, so the accumulator names its width.
    const uint32_t trise =
        static_cast<uint32_t>((static_cast<uint64_t>(rise_ns) * pclk) / 1'000'000'000ULL) + 1UL;
    if (trise > i2c_trise_mask) {
        return {};
    }
    uint16_t bits = static_cast<uint16_t>(ccr);
    if (fast) {
        bits = static_cast<uint16_t>(bits | I2C_CCR_FS);
        if (duty == I2cDuty::ratio_16_9) {
            bits = static_cast<uint16_t>(bits | I2C_CCR_DUTY);
        }
    }
    return I2cTiming{static_cast<uint8_t>(pclk / 1'000'000UL), bits,
                     static_cast<uint8_t>(trise)};
}

/// What a timing really produces, for the record - the arithmetic's own
/// answer, which the rise time and the input filters lengthen a little on
/// the wire (27.6.8's closing note).
constexpr uint32_t i2c_scl_hz(uint32_t pclk, I2cTiming t) {
    const uint32_t ccr = t.ccr & i2c_ccr_mask;
    if (ccr == 0u) {
        return 0;
    }
    if ((t.ccr & I2C_CCR_FS) == 0u) {
        return pclk / (2UL * ccr);
    }
    return pclk / (((t.ccr & I2C_CCR_DUTY) != 0u ? 25UL : 3UL) * ccr);
}

/**
 * ES0206 2.10.4 (ES0298 2.11.4, ES0287 2.9.4): the setup time of a
 * REPEATED START can be violated in controller standard mode between
 * 88 and 100 kHz - independently of the APB clock, and only when the
 * target stretches the clock or the SCL rise time is above 300 ns. The
 * workaround is Fm mode or a standard-mode rate below 88 kHz. A
 * write-then-read Request is a repeated start, so the answer belongs
 * beside the bus and not only in a document.
 */
constexpr bool i2c_repeated_start_at_risk(I2cSpeed speed, uint32_t scl_hz_in_force) {
    return speed == I2cSpeed::standard_100k && scl_hz_in_force > 88'000UL;
}

/// The analog filter's off switch and the digital filter's length in APB
/// periods (27.6.10). Absent on the F405 class: every verb that writes it
/// refuses there.
struct I2cFilter {
    bool analog = true;     ///< ANOFF inverted: the reset state has it on
    uint8_t digital = 0;    ///< DNF[3:0], 0 = off
};

/**
 * Table 122's ceiling on DNF - the largest digital filter that keeps the
 * data hold time inside the bus specification with the analog filter off,
 * per APB1 band and mode. It is ADVICE and not a refusal: 27.3.5's own
 * note allows more "if the system can support maximum hold time
 * violation". Zero below the band the table starts at.
 */
constexpr uint8_t i2c_digital_filter_max(uint32_t pclk, I2cSpeed speed) {
    const bool fast = speed == I2cSpeed::fast_400k;
    if (pclk < 2'000'000UL) {
        return 0;
    }
    if (pclk <= 5'000'000UL) {
        return fast ? 0 : 2;
    }
    if (pclk <= 10'000'000UL) {
        return fast ? 0 : 12;
    }
    if (pclk <= 20'000'000UL) {
        return fast ? 1 : 15;
    }
    if (pclk <= 30'000'000UL) {
        return fast ? 7 : 15;
    }
    if (pclk <= 40'000'000UL) {
        return fast ? 13 : 15;
    }
    return 15;
}

/// A target's addresses (27.3.2): one 7- or 10-bit own address, an
/// optional second 7-bit one (ENDUAL), the general call.
struct I2cAddressConfig {
    uint16_t own = 0;                 ///< 7 bits, or 10 with ten_bit
    bool ten_bit = false;
    std::optional<uint8_t> second{};  ///< OAR2: a second 7-bit address
    bool general_call = false;
};

constexpr bool i2c_address_config_valid(const I2cAddressConfig& a) {
    if (a.ten_bit) {
        return a.own <= 0x3FFu && !a.second;   // ENDUAL is a 7-bit arrangement
    }
    return a.own <= 0x7Fu && (!a.second || *a.second <= 0x7Fu);
}

/// SMBus, as 27.3.7 configures it: the mode, which of the two device
/// types this side is, and the address resolution protocol.
struct I2cSmbusConfig {
    bool enabled = false;
    bool host = false;   ///< SMBTYPE: false = SMBus device, true = SMBus host
    bool arp = false;    ///< ENARP
};

/**
 * The pads an I2C link claims, each with the alternate function the
 * DATASHEET gives that signal on that pad (AF4 for the three instances,
 * with AF9 on a handful of pads). SMBA is optional and only means
 * anything in SMBus mode.
 */
struct I2cPins {
    PinSel scl{};
    PinSel sda{};
    PinSel smba{};
};

/// A bus needs both lines, and no two signals may name the same pad.
constexpr bool i2c_pins_valid(const I2cPins& p) {
    if (!p.scl.valid() || !p.sda.valid()) {
        return false;
    }
    const PinSel all[] = {p.scl, p.sda, p.smba};
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
// I2c<n>: the resource
// =============================================================================

/**
 * One I2C instance, register by register. Every verb stores what it names
 * and nothing else; a verb whose combination the chapter declares wrong,
 * or whose register this part has not, refuses (false, nothing written).
 * The tasks below are written on these verbs; a program that wants the
 * chapter beyond a bus transaction - SMBus, PEC, a 10-bit target - reaches
 * them here.
 */
template <uint8_t n>
struct I2c {
    static_assert(i2c_present(n),
                  "brio I2c: this device has no I2C instance of that number (the device "
                  "header declares no I2Cn_BASE for it; I2C1 and I2C2 are on every part of "
                  "this family, I2C3 on every one but the F410)");

    I2c() = delete;

    static constexpr uint8_t number = n;
    /// Every I2C of this family is APB1's - asked of the enable bit's own
    /// register (stm32f4/device_tables.hpp), not written down as a rule.
    static constexpr bool on_apb2 = i2c_on_apb2(n);
    static constexpr IRQn_Type event_irq = i2c_event_irq(n);
    static constexpr IRQn_Type error_irq = i2c_error_irq(n);
    /// Whether I2C_FLTR's fields exist on this part at all.
    static constexpr bool has_filter = i2c_has_filter();

    static I2C_TypeDef& regs() { return *reinterpret_cast<I2C_TypeDef*>(i2c_base(n)); }

    // ---- the gate and the reset ------------------------------------------------
    static void bus_clock(bool on) {
        if constexpr (on_apb2) {
            Rcc::apb2_clock(i2c_clock_mask(n), on);
        } else {
            Rcc::apb1_clock(i2c_clock_mask(n), on);
        }
    }
    static bool bus_clock() {
        if constexpr (on_apb2) {
            return Rcc::apb2_clock(i2c_clock_mask(n));
        } else {
            return Rcc::apb1_clock(i2c_clock_mask(n));
        }
    }
    /// Every register to its reset value through the RCC's reset line.
    static void reset() {
        if constexpr (on_apb2) {
            Rcc::apb2_reset(i2c_clock_mask(n));
        } else {
            Rcc::apb1_reset(i2c_clock_mask(n));
        }
    }

    // ---- the timing -----------------------------------------------------------
    /**
     * FREQ, CCR and TRISE, with PE clear on the way in: all three "must be
     * configured only when the I2C is disabled" (27.6.8, 27.6.9), and the
     * peripheral enable is left DOWN - the caller raises it when the whole
     * configuration stands.
     */
    static void timing(I2cTiming t) {
        I2C_TypeDef& r = regs();
        r.CR1 &= ~I2C_CR1_PE;
        r.CR2 = (r.CR2 & ~I2C_CR2_FREQ_Msk) | (static_cast<uint32_t>(t.freq_mhz) & I2C_CR2_FREQ_Msk);
        r.CCR = t.ccr;
        r.TRISE = t.trise;
    }
    static I2cTiming timing() {
        const I2C_TypeDef& r = regs();
        return I2cTiming{static_cast<uint8_t>(r.CR2 & I2C_CR2_FREQ_Msk),
                         static_cast<uint16_t>(r.CCR),
                         static_cast<uint8_t>(r.TRISE & i2c_trise_mask)};
    }

    /**
     * The two noise filters (27.3.5). Refused where the part has no FLTR
     * register at all, where the length would not fit its four bits, and
     * while PE stands - "DNF[3:0] must only be configured when the I2C is
     * disabled".
     */
    /// The register is not merely unwritten where the part has not got it:
    /// its FIELD IS NOT IN THE HEADER'S I2C_TypeDef at all on the F405
    /// class, so the two accesses are selected by the preprocessor - the
    /// one question only it can ask - while `has_filter` above is the
    /// VALUE a caller and a static_assert read (the reserve's rule).
    static bool filter(const I2cFilter& f) {
#if defined(I2C_FLTR_ANOFF)
        if (f.digital > 15u || enabled()) {
            return false;
        }
        regs().FLTR = (f.analog ? 0u : I2C_FLTR_ANOFF) | static_cast<uint32_t>(f.digital);
        return true;
#else
        (void)f;
        return false;
#endif
    }
    static I2cFilter filter() {
#if defined(I2C_FLTR_ANOFF)
        const uint32_t v = regs().FLTR;
        return I2cFilter{(v & I2C_FLTR_ANOFF) == 0u, static_cast<uint8_t>(v & I2C_FLTR_DNF)};
#else
        return I2cFilter{};
#endif
    }

    // ---- the own addresses ------------------------------------------------------
    /**
     * OAR1 and OAR2, refused by i2c_address_config_valid()'s rules. Bit 14
     * of OAR1 "should always be kept at 1 by software" (27.6.3) and every
     * write here keeps it.
     */
    static bool addresses(const I2cAddressConfig& a) {
        if (!i2c_address_config_valid(a)) {
            return false;
        }
        uint32_t o1 = 1u << 14;
        if (a.ten_bit) {
            o1 |= I2C_OAR1_ADDMODE | (static_cast<uint32_t>(a.own) & 0x3FFu);
        } else {
            o1 |= (static_cast<uint32_t>(a.own) & 0x7Fu) << 1;
        }
        regs().OAR1 = o1;
        regs().OAR2 = a.second ? (I2C_OAR2_ENDUAL | ((static_cast<uint32_t>(*a.second) & 0x7Fu) << 1))
                               : 0u;
        general_call(a.general_call);
        return true;
    }

    // ---- the peripheral -----------------------------------------------------------
    static void enable() { regs().CR1 |= I2C_CR1_PE; }
    /// 27.6.1: "in controller mode, this bit must not be reset before the
    /// end of the communication" - and every bit PE clears takes effect at
    /// the end of the current transfer.
    static void disable() { regs().CR1 &= ~I2C_CR1_PE; }
    static bool enabled() { return (regs().CR1 & I2C_CR1_PE) != 0u; }

    /// SWRST: the state machines and the flags back to reset, the
    /// configuration kept - 27.6.1's own verb for a bus that shows BUSY
    /// with no STOP in sight, and the workaround ES0206 2.10.3 prescribes
    /// for a START that will not come out. Pulsed here: set, then cleared.
    static void software_reset() {
        regs().CR1 |= I2C_CR1_SWRST;
        regs().CR1 &= ~I2C_CR1_SWRST;
    }

    // ---- the control bits ---------------------------------------------------------
    static void start() { cr1(I2C_CR1_START, true); }
    static bool starting() { return (regs().CR1 & I2C_CR1_START) != 0u; }
    static void stop() { cr1(I2C_CR1_STOP, true); }
    static bool stopping() { return (regs().CR1 & I2C_CR1_STOP) != 0u; }
    static void ack(bool on) { cr1(I2C_CR1_ACK, on); }
    /// POS: the ACK bit governs the NEXT byte to arrive rather than the
    /// current one - the two-byte receive's arrangement, and 27.6.1 says
    /// it is for that alone.
    static void pos(bool on) { cr1(I2C_CR1_POS, on); }
    static void general_call(bool on) { cr1(I2C_CR1_ENGC, on); }
    static void no_stretch(bool on) { cr1(I2C_CR1_NOSTRETCH, on); }

    // ---- SMBus and PEC (27.3.7, 27.3.9) --------------------------------------------
    static void smbus(const I2cSmbusConfig& s) {
        uint32_t v = regs().CR1 & ~(I2C_CR1_SMBUS | I2C_CR1_SMBTYPE | I2C_CR1_ENARP);
        if (s.enabled) { v |= I2C_CR1_SMBUS; }
        if (s.host) { v |= I2C_CR1_SMBTYPE; }
        if (s.arp) { v |= I2C_CR1_ENARP; }
        regs().CR1 = v;
    }
    static bool smbus() { return (regs().CR1 & I2C_CR1_SMBUS) != 0u; }
    /// ALERT: drives the SMBA pad low, and the alert response address is
    /// acknowledged.
    static void alert(bool on) { cr1(I2C_CR1_ALERT, on); }
    static void pec(bool on) { cr1(I2C_CR1_ENPEC, on); }
    static bool pec() { return (regs().CR1 & I2C_CR1_ENPEC) != 0u; }
    /// The frame after this one is the PEC: written after the TxE of the
    /// last byte in transmission, after the RxNE of the last byte in
    /// reception.
    static void pec_transfer() { cr1(I2C_CR1_PEC, true); }
    static bool pec_pending() { return (regs().CR1 & I2C_CR1_PEC) != 0u; }
    /// The internal PEC, in SR2's upper half.
    static uint8_t pec_value() {
        return static_cast<uint8_t>((regs().SR2 & I2C_SR2_PEC_Msk) >> I2C_SR2_PEC_Pos);
    }

    // ---- DMA ------------------------------------------------------------------------
    /// DMAEN with LAST: LAST makes the block NACK the byte after EOT_1, so
    /// a controller receiver's last byte is refused without the CPU
    /// (27.3.8). It must be set before the ADDR event, or during it.
    static void dma(bool on, bool last = false) {
        uint32_t v = regs().CR2 & ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
        if (on) { v |= I2C_CR2_DMAEN; }
        if (last) { v |= I2C_CR2_LAST; }
        regs().CR2 = v;
    }
    static volatile void* data_address() { return &regs().DR; }

    // ---- data and status -------------------------------------------------------------
    static void data(uint8_t v) { regs().DR = v; }
    static uint8_t data() { return static_cast<uint8_t>(regs().DR); }
    static uint32_t status1() { return regs().SR1; }
    static uint32_t status2() { return regs().SR2; }
    static bool flag(uint32_t mask) { return (regs().SR1 & mask) != 0u; }
    static bool busy() { return (regs().SR2 & I2C_SR2_BUSY) != 0u; }
    /// MSL: this side is the bus controller in the tenure under way.
    static bool host() { return (regs().SR2 & I2C_SR2_MSL) != 0u; }
    /// TRA: this side is transmitting in the tenure under way.
    static bool transmitting() { return (regs().SR2 & I2C_SR2_TRA) != 0u; }
    static bool second_address_matched() { return (regs().SR2 & I2C_SR2_DUALF) != 0u; }
    static bool general_call_matched() { return (regs().SR2 & I2C_SR2_GENCALL) != 0u; }
    static bool smbus_host_matched() { return (regs().SR2 & I2C_SR2_SMBHOST) != 0u; }
    static bool smbus_default_matched() { return (regs().SR2 & I2C_SR2_SMBDEFAULT) != 0u; }

    /// EV6/EV1: ADDR is cleared by reading SR1 then SR2, in that order.
    /// Returns SR2, which tells the direction (TRA) and which address
    /// matched. 27.6.7 warns that SR2 must be read ONLY when ADDR is found
    /// set or after STOPF is cleared, which is what the phases guarantee.
    static uint32_t clear_addr() {
        (void)regs().SR1;
        return regs().SR2;
    }
    /// EV4: STOPF is cleared by reading SR1 then WRITING CR1.
    static void clear_stopf() {
        (void)regs().SR1;
        regs().CR1 = regs().CR1;
    }
    /// The seven error flags are rc_w0: a zero written into the bit.
    static void clear_errors(uint32_t mask) {
        regs().SR1 = ~(mask & I2cFlag::errors);
    }

    // ---- interrupts (27.4) --------------------------------------------------------------
    static void event_interrupt(bool on) { cr2(I2C_CR2_ITEVTEN, on); }
    /// ITBUFEN: TxE and RxNE reach the event vector too. 27.3.8 forbids it
    /// while a DMA request serves the same direction.
    static void buffer_interrupt(bool on) { cr2(I2C_CR2_ITBUFEN, on); }
    static void error_interrupt(bool on) { cr2(I2C_CR2_ITERREN, on); }

private:
    static void cr1(uint32_t bit, bool on) {
        if (on) { regs().CR1 |= bit; } else { regs().CR1 &= ~bit; }
    }
    static void cr2(uint32_t bit, bool on) {
        if (on) { regs().CR2 |= bit; } else { regs().CR2 &= ~bit; }
    }
};

// =============================================================================
// The engine slots
// =============================================================================

/// Two engines on one bus must not name the same STREAM: a stream moves
/// data one way and has one FIFO.
template <typename Tx, typename Rx>
constexpr bool i2c_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::controller != Rx::controller || Tx::stream != Rx::stream;
    } else {
        return true;
    }
}

/// Whether an engine sits on a cell of the request mapping that really
/// carries instance `n`'s transmit (or receive) request. True for an
/// absent engine, and FALSE on a part class whose request table was not
/// read - a refusal, never a guess.
template <uint8_t n, typename E, bool transmit>
constexpr bool i2c_engine_placed() {
    if constexpr (!E::present) {
        return true;
    } else {
        return i2c_dma_placement_valid(n, transmit, E::controller, E::stream, E::channel);
    }
}

/// A DMA block the engines could not finish: the engine's own status code,
/// in the range util/bus_master.hpp leaves to engines and above the four
/// i2c_bus.hpp holds.
inline constexpr uint8_t i2c_dma_fault = bus_engine_status + 4;

/// What init() takes besides the clock: the shape of fast mode's duty
/// cycle, the wire's rise times (which is what TRISE is), and the noise
/// filters. Every field has the value a plain 100 kHz bus wants.
struct I2cHostConfig {
    I2cDuty duty = I2cDuty::ratio_2;
    uint16_t standard_rise_ns = i2c_standard_rise_ns;
    uint16_t fast_rise_ns = i2c_fast_rise_ns;
    I2cFilter filter{};
};

// =============================================================================
// I2cHost: the bus engine
// =============================================================================

/**
 * I2cHost<n, pins, TxEngine, RxEngine>
 *
 * The engine I2cBus (util/i2c_bus.hpp = BusMaster) drives. One Request is
 * one bus tenure: START, the address, tx_len bytes written, then - if
 * rx_len is not zero - a repeated START, the address again with the read
 * bit and rx_len bytes taken with the last one NACKed, then STOP. Both
 * spans empty is the PROBE: START, address, STOP - the ACK on the address
 * is the answer. Every outcome the wire can give comes back as
 * i2c_bus.hpp's codes through TransferDone{status()}.
 *
 * `speed` names a row of the timing table init() solves for the APB1
 * clock; a speed the clock cannot produce is answered i2c_rejected inside
 * start(), no byte moved - the one synchronous completion of an I2C
 * engine, and the arbiter replies with status() for it.
 *
 * THE ENGINE SLOTS carry a write phase of any length and a read phase of
 * two bytes or more (CR2.LAST makes the block NACK the last byte a receive
 * run takes); a ONE-BYTE READ stays on the pump, whose ACK-before-ADDR
 * sequence the DMA path cannot express. Both engines or neither: a
 * write-then-read tenure needs both directions inside one tenure.
 *
 * THE PADS GO OUT OPEN DRAIN at `very_high`, which is what an I2C pad is:
 * the peripheral drives the lines low and lets the external pull-ups
 * raise them, and the slew class only shapes the falling edge.
 */
template <uint8_t n, I2cPins pins, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine>
class I2cHost {
    using S = I2c<n>;

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from stm32f4/dma.hpp, or NoDmaEngine (the default)");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio I2cHost: name both DMA engines or neither - a write-then-read "
                  "tenure needs both directions inside one tenure");
    static_assert(i2c_engines_distinct<TxEngine, RxEngine>(),
                  "brio I2cHost: the transmit and receive engines must use DIFFERENT DMA "
                  "streams - a stream carries one direction and has one FIFO");
    static_assert(i2c_engine_placed<n, TxEngine, true>(),
                  "brio I2cHost: the transmit engine's (controller, stream, channel) is not "
                  "a cell this instance's transmit request is wired to - RM0090 table 43 and "
                  "its RM0390 / RM0383 twins, keyed per part class in "
                  "stm32f4/device_tables.hpp (a part class whose manual was not read has no "
                  "table, and an engine is refused there rather than guessed)");
    static_assert(i2c_engine_placed<n, RxEngine, false>(),
                  "brio I2cHost: the receive engine's (controller, stream, channel) is not a "
                  "cell this instance's receive request is wired to - see the transmit "
                  "engine's message");
    static_assert(i2c_pins_valid(pins),
                  "brio I2cHost: these I2C pads are not a bus - SCL and SDA must both be "
                  "named, must be real pins of ports this part bonds, and no two signals may "
                  "name the same pad");

    using SclPin = Pin<pins.scl.port, pins.scl.pin>;
    using SdaPin = Pin<pins.sda.port, pins.sda.pin>;
    using SmbaPin = Pin<pins.smba.valid() ? pins.smba.port : pins.scl.port,
                        pins.smba.valid() ? pins.smba.pin : pins.scl.pin>;

public:
    I2cHost() = delete;

    using Resource = S;
    static constexpr I2cPins pin_sel = pins;
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
        I2cSpeed speed = I2cSpeed::standard_100k;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle ----------------------------------------------------------------

    /**
     * Bring the instance up as a bus host. `clock` is the app's Clock tag,
     * and the rate this chapter's three timing registers divide is THE
     * INSTANCE'S OWN APB1 CLOCK, never SYSCLK. Both speeds are solved here
     * and at rebase(); one the clock cannot produce is marked unreachable
     * (speed_ok()) and a Request naming it is answered i2c_rejected. False
     * when not even standard_100k is legal - which is every APB1 clock
     * below 2 MHz (27.6.2).
     */
    template <typename Clock>
    static bool init(Clock clock, const I2cHostConfig& cfg = {}) {
        static_assert(clock_follows<Clock, I2cHost>(),
                      "this I2cHost is initialized with a DynamicClock that does not list it "
                      "among its Users: its timing table would go stale on a clock change");
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        cfg_ = cfg;
        S::bus_clock(true);
        S::reset();
        rebase_pclk(apb_hz(clock, S::on_apb2), clock_hz(clock));
        if (!valid_[0]) {
            return false;
        }
        // THE PADS GO OVER BEFORE ANYTHING IS CONFIGURED, and BUSY is the
        // reason. That flag is not a stored bit: it is set "on detection of
        // SDA or SCL low" and cleared only "on detection of a Stop
        // condition" (27.6.7) - and the level it watches is the
        // PERIPHERAL'S input, which reads LOW while the pad is not in its
        // alternate function. A block configured before its pads therefore
        // comes up BUSY over an idle wire, no STOP is ever going to arrive
        // to clear it, and 27.3.3's START - which waits for BUSY to fall -
        // never leaves. So: the pads first, then the software reset below
        // takes the state machine and BUSY with the lines already real.
        // OPEN DRAIN is the definition of the bus: the pull-ups own the
        // idle level (an internal one is far too weak for an I2C edge), and
        // a released alternate-function output drives nothing, so handing
        // the pads to a disabled peripheral cannot disturb the wire.
        claim_pads();
        applied_ = I2cSpeed::standard_100k;
        S::software_reset();
        S::timing(table_[0]);
        (void)S::filter(cfg_.filter);   // refused where the part has no FLTR
        S::enable();
        S::ack(false);
        if constexpr (has_engines) {
            TxEngine::arm(S::data_address());
            RxEngine::arm(S::data_address());
        }
        status_ = i2c_ok;
        phase_ = Phase::idle;
        S::event_interrupt(true);
        S::error_interrupt(true);
        S::buffer_interrupt(false);
        Nvic::enable(S::event_irq);
        Nvic::enable(S::error_irq);
        return true;
    }

    /// The core clock changed (DynamicClock fan-out): both rows solved
    /// again and the applied one rewritten. `hz` is SYSCLK, as every other
    /// task's rebase takes it - the APB1 divider in force is read back from
    /// the RCC. THE BUS MUST BE IDLE.
    static void rebase(uint32_t hz) {
        const uint32_t pclk = hz / (S::on_apb2 ? Rcc::apb2_divider() : Rcc::apb1_divider());
        const bool was_enabled = S::enabled();
        rebase_pclk(pclk, hz);
        if (was_enabled && valid_[static_cast<uint8_t>(applied_)]) {
            S::timing(table_[static_cast<uint8_t>(applied_)]);
            S::enable();
        }
    }

    static bool speed_ok(I2cSpeed s) { return valid_[static_cast<uint8_t>(s)]; }
    static I2cTiming timing_of(I2cSpeed s) { return table_[static_cast<uint8_t>(s)]; }
    static uint32_t scl_hz(I2cSpeed s) {
        return i2c_scl_hz(pclk_hz_, table_[static_cast<uint8_t>(s)]);
    }
    /// The rate FREQ states and CCR divides: this instance's APB1 clock.
    static uint32_t reference_hz() { return pclk_hz_; }
    /// The largest digital filter table 122 allows at the clock in force.
    static uint8_t digital_filter_max(I2cSpeed s) { return i2c_digital_filter_max(pclk_hz_, s); }
    /// ES0206 2.10.4: whether a repeated START at this speed sits in the
    /// window where its setup time can be violated.
    static bool repeated_start_setup_at_risk(I2cSpeed s) {
        return i2c_repeated_start_at_risk(s, scl_hz(s));
    }
    /// How many BUS ERRORS this driver has seen and NOT acted on - every
    /// one of them, since a controller's tenure is never ended by one
    /// (27.3.4) and ES0206 2.10.1 says one can be raised where there was
    /// none. The count is what makes the difference visible at all.
    static uint32_t spurious_bus_errors() { return spurious_berr_; }
    static void clear_spurious_bus_errors() { spurious_berr_ = 0; }

    // ---- the transfer ---------------------------------------------------------------

    /// Begin a tenure (I2cBus calls it from main context). False whenever
    /// the wire moves: the tenure completes on the event vector and a
    /// TransferDone{status()} follows. True only for the one refusal that
    /// moves nothing - a speed the clock cannot produce, answered
    /// i2c_rejected through status() - the I2cHost contract
    /// (docs/design/i2c-bus.md).
    static bool start(const Request& r) {
        req_ = r;
        pos_ = 0;
        if (!speed_ok(r.speed)) {
            status_ = i2c_rejected;
            phase_ = Phase::idle;
            return true;
        }
        status_ = i2c_ok;
        phase_ = (r.tx_len == 0u && r.rx_len != 0u) ? Phase::start_rx : Phase::start_tx;
        // The last tenure's STOP may still be on its way out: STOP stands
        // in CR1 until the condition is on the wire, and a client that
        // stretches the clock after the last acknowledge holds it there.
        // 27.6.1 forbids any CR1 write while STOP stands - a second STOP
        // request otherwise - so it is waited for before anything writes
        // CR1, a speed change's PE cycle included, BOUNDED IN TIME
        // (stop_drain_us). A clock held past the bound is a wedge: the
        // tenure PARKS - no START and no vector - for the per-bus timeout
        // to answer.
        if (!wait_for_us([] { return !S::stopping(); }, stop_drain_us)) {
            return false;
        }
        // BUSY clears a moment after our STOP is seen; a bus still busy
        // after bus_free_us is someone else's, and 27.3.3's START waits for
        // BUSY to fall by itself.
        (void)wait_for_us([] { return !S::busy(); }, bus_free_us);
        apply(r.speed);
        S::pos(false);
        S::ack(false);
        S::buffer_interrupt(false);
        S::start();
        return false;
    }

    /// The engine's completion status, for the TransferDone payload.
    static uint8_t status() { return status_; }

    /// The EVENT vector's body - call from the instance's I2Cn_EV handler.
    /// Returns true when the tenure just completed: the edge the app's glue
    /// posts TransferDone on.
    [[gnu::always_inline]] static bool isr() {
        const uint32_t s1 = S::status1();
        // STOPF is the TARGET half's flag - a STOP seen after a START this
        // host did not issue: another controller's, or the one unstick()
        // makes by hand - and ITEVTEN routes it here in every phase. Left
        // standing it re-enters this vector without end. Its sequence ends
        // in a CR1 write, which must not happen while START or STOP stands
        // (27.6.1), so it waits for them to leave - a bit time at most.
        if ((s1 & I2cFlag::stop) != 0u && !S::starting() && !S::stopping()) {
            S::clear_stopf();
        }
        switch (phase_) {
            case Phase::idle:
                // Nothing in flight: an ADDR here is the target half's too,
                // and its sequence (SR1 read above, then SR2) writes
                // nothing.
                if ((s1 & I2cFlag::address) != 0u) {
                    (void)S::status2();
                }
                return false;

            case Phase::start_tx:
            case Phase::start_rx:
                if ((s1 & I2cFlag::start_bit) == 0u) {
                    return false;
                }
                // EV5: the address, with the direction bit. Reading SR1 -
                // done above - then writing DR is what clears SB.
                if (phase_ == Phase::start_rx) {
                    prime_receive();
                    S::data(static_cast<uint8_t>((req_.addr << 1) | 1u));
                    phase_ = Phase::addr_rx;
                } else {
                    S::data(static_cast<uint8_t>(req_.addr << 1));
                    phase_ = Phase::addr_tx;
                }
                return false;

            case Phase::addr_tx:
                if ((s1 & I2cFlag::address) == 0u) {
                    return false;
                }
                if (req_.tx_len == 0u) {
                    // The probe: acknowledged, and that was the question.
                    (void)S::clear_addr();
                    S::stop();
                    return finish(i2c_ok);
                }
                if constexpr (has_engines) {
                    // 27.3.8: DMAEN before the ADDR flag is cleared.
                    S::dma(true, false);
                    (void)TxEngine::start(req_.tx.get(), req_.tx_len);
                    (void)S::clear_addr();
                    phase_ = Phase::tx_dma;
                    return false;
                }
                (void)S::clear_addr();
                phase_ = Phase::tx;
                S::buffer_interrupt(true);   // EV8_1 follows at once
                return false;

            case Phase::tx:
                if ((s1 & I2cFlag::tx_empty) != 0u && pos_ < req_.tx_len) {
                    S::data(req_.tx.get()[pos_]);
                    ++pos_;
                    if (pos_ >= req_.tx_len) {
                        S::buffer_interrupt(false);   // BTF carries the end
                    }
                    return false;
                }
                if ((s1 & I2cFlag::byte_finished) != 0u && pos_ >= req_.tx_len) {
                    return end_of_write();   // EV8_2
                }
                return false;

            case Phase::tx_dma:
                // The engine loaded every byte; BTF says the last one is out
                // on the wire - 27.3.8's own order (disable the requests,
                // then wait for BTF, then the STOP or the repeated START).
                if ((s1 & I2cFlag::byte_finished) != 0u && !dma_busy()) {
                    S::dma(false, false);
                    return end_of_write();
                }
                return false;

            case Phase::addr_rx:
                if ((s1 & I2cFlag::address) == 0u) {
                    return false;
                }
                return begin_receive();

            case Phase::rx:
                return receive_step(s1);

            case Phase::rx_dma:
                return false;   // dma_isr() ends it
        }
        return false;
    }

    /**
     * The ERROR vector's body - call from the instance's I2Cn_ER handler.
     * True when the tenure just completed.
     *
     * A NACK ends the tenure with i2c_nack_addr or i2c_nack_data by the
     * phase it landed in, and the STOP is ours to send (27.3.4). Lost
     * arbitration ends it with the hardware having released the bus. A BUS
     * ERROR does NOT end it: 27.3.4 says a controller's transfer is
     * unaffected by one, and ES0206 2.10.1 says one can be raised where
     * there was none - so the flag is cleared, counted and the tenure runs
     * on. Anything else (an overrun, a PEC error, an SMBus timeout or
     * alert) ends it as a bus error, because none of them can arise in a
     * plain I2C controller tenure.
     */
    [[gnu::always_inline]] static bool error_isr() {
        const uint32_t s1 = S::status1();
        const uint32_t errs = s1 & I2cFlag::errors;
        if (errs == 0u) {
            return false;
        }
        S::clear_errors(errs);
        if ((errs & I2cFlag::bus_error) != 0u) {
            ++spurious_berr_;
        }
        if (phase_ == Phase::idle) {
            return false;
        }
        if ((errs & I2cFlag::ack_failure) != 0u) {
            const bool on_address = phase_ == Phase::addr_tx || phase_ == Phase::addr_rx ||
                                    phase_ == Phase::start_tx || phase_ == Phase::start_rx;
            S::stop();   // 27.3.4: a controller that is NACKed sends the STOP
            if constexpr (has_engines) {
                put_engines_away();
            }
            return finish(on_address ? i2c_nack_addr : i2c_nack_data);
        }
        if ((errs & I2cFlag::arbitration_lost) != 0u) {
            if constexpr (has_engines) {
                put_engines_away();
            }
            return finish(i2c_arb_lost);   // the hardware released the bus already
        }
        if ((errs & ~(I2cFlag::bus_error)) != 0u) {
            S::stop();
            if constexpr (has_engines) {
                put_engines_away();
            }
            return finish(i2c_bus_error);
        }
        return false;   // a bus error alone: counted, and the tenure runs on
    }

    /// The DMA streams' interrupt body - call from BOTH streams' vectors
    /// (one vector per stream on this family, shared with nothing). The
    /// receive block completing ends a read phase (the STOP after it); the
    /// transmit block completing is only half the story (BTF on the event
    /// vector says the last byte is out). A transfer error ends the tenure
    /// with i2c_dma_fault. True when the tenure just completed.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            const uint8_t tx = TxEngine::service();
            if ((tx & TxEngine::flag_error) != 0u) {
                put_engines_away();
                S::stop();
                return finish(i2c_dma_fault);
            }
            if ((tx & TxEngine::flag_complete) != 0u) {
                (void)TxEngine::complete();
            }
            const uint8_t rx = RxEngine::service();
            if ((rx & RxEngine::flag_error) != 0u) {
                put_engines_away();
                S::stop();
                return finish(i2c_dma_fault);
            }
            if ((rx & RxEngine::flag_complete) != 0u && phase_ == Phase::rx_dma) {
                S::dma(false, false);
                S::stop();
                return finish(i2c_ok);
            }
        }
        return false;
    }

    /// The classic bus unstick: nine SCL pulses and a STOP, by hand,
    /// open-drain, with the pads reclaimed from the peripheral for the
    /// duration. RECOVER() FIXES THE PERIPHERAL, THIS FIXES THE WIRE.
    /// Returns the pulses it took a stuck client to release SDA (0 = the
    /// wire was never stuck), or 0xFF when nine pulses and a STOP left SDA
    /// low - a short, not a client. A healthy wire is left alone.
    static uint8_t unstick() {
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        SclPin::output(true, {.open_drain = true, .speed = PinSpeed::very_high});
        SdaPin::output(true, {.open_drain = true, .speed = PinSpeed::very_high});

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
        S::clear_errors(I2cFlag::errors);
        Nvic::enable(S::event_irq);
        Nvic::enable(S::error_irq);
        if (!free_now) {
            return 0xFF;
        }
        return released_at == 0xFF ? 0u : released_at;
    }

    /// Put the ENGINE back where start() is legal - the verb a timed
    /// I2cBus calls on a tenure that never answered: the engines put away,
    /// SWRST pulsed (the chapter's own verb for a controller stuck BUSY,
    /// and ES0206 2.10.3's workaround for a START that will not come out),
    /// the timing and the filters rewritten, PE back. The WIRE stays the
    /// application's (unstick()).
    static bool recover() {
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        if constexpr (has_engines) {
            put_engines_away();
        }
        phase_ = Phase::idle;
        S::buffer_interrupt(false);
        S::software_reset();
        S::timing(table_[static_cast<uint8_t>(applied_)]);
        (void)S::filter(cfg_.filter);
        S::enable();
        S::ack(false);
        S::event_interrupt(true);
        S::error_interrupt(true);
        Nvic::enable(S::event_irq);
        Nvic::enable(S::error_irq);
        return true;
    }

    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        S::disable();
        S::bus_clock(false);
        SclPin::release();
        SdaPin::release();
        if constexpr (pins.smba.valid()) {
            SmbaPin::release();
        }
    }

    /// Hand the SMBA pad to the peripheral - only for a program that has
    /// put the resource in SMBus mode. The task's own tenures never use it.
    static void claim_smba_pad(bool on) {
        if constexpr (pins.smba.valid()) {
            if (on) {
                SmbaPin::function(pins.smba.function,
                                  {.open_drain = true, .speed = PinSpeed::very_high});
            } else {
                SmbaPin::release();
            }
        } else {
            (void)on;
        }
    }

private:
    enum class Phase : uint8_t {
        idle, start_tx, start_rx, addr_tx, addr_rx, tx, tx_dma, rx, rx_dma,
    };

    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, uint8_t> &&
                   std::is_same_v<typename RxEngine::element, uint8_t>;
        } else {
            return true;
        }
    }(), "brio I2cHost: the DMA engines must carry uint8_t elements - this data register is "
         "one byte wide");

    static void claim_pads() {
        SclPin::function(pins.scl.function, {.open_drain = true, .speed = PinSpeed::very_high});
        SdaPin::function(pins.sda.function, {.open_drain = true, .speed = PinSpeed::very_high});
    }

    static bool finish(uint8_t st) {
        status_ = st;
        phase_ = Phase::idle;
        S::buffer_interrupt(false);
        return true;
    }

    /// EV8_2: the last written byte is out. A repeated START opens the read
    /// half, or the STOP ends the tenure.
    static bool end_of_write() {
        if (req_.rx_len != 0u) {
            phase_ = Phase::start_rx;
            S::start();
            return false;
        }
        S::stop();
        return finish(i2c_ok);
    }

    /// The ACK/POS arrangement the read phase's length wants, set BEFORE
    /// the address goes out (27.3.3's receive procedures).
    static void prime_receive() {
        if (req_.rx_len == 2u && !dma_serves_rx()) {
            S::pos(true);
            S::ack(true);
        } else {
            S::ack(req_.rx_len != 1u);
        }
    }

    /// EV6 on a read: the procedure by count.
    static bool begin_receive() {
        if constexpr (has_engines) {
            if (dma_serves_rx()) {
                S::dma(true, true);   // LAST: the block's last byte is NACKed
                (void)RxEngine::start(req_.rx.get(), req_.rx_len);
                (void)S::clear_addr();
                phase_ = Phase::rx_dma;
                return false;
            }
        }
        pos_ = 0;
        if (req_.rx_len == 1u) {
            // ACK is already clear (prime_receive): clear ADDR, then STOP at
            // once; the byte lands on RxNE.
            (void)S::clear_addr();
            S::stop();
            phase_ = Phase::rx;
            S::buffer_interrupt(true);
            return false;
        }
        if (req_.rx_len == 2u) {
            // POS and ACK were set: clear ADDR, then ACK off - the NACK
            // lands on the second byte; one BTF delivers both.
            (void)S::clear_addr();
            S::ack(false);
            phase_ = Phase::rx;
            return false;
        }
        (void)S::clear_addr();
        phase_ = Phase::rx;
        S::buffer_interrupt(req_.rx_len > 3u);   // RxNE while more than three remain
        return false;
    }

    /// The receive pump, per the count's procedure.
    static bool receive_step(uint32_t s1) {
        uint8_t* out = req_.rx.get();
        if (req_.rx_len == 1u) {
            if ((s1 & I2cFlag::rx_not_empty) == 0u) {
                return false;
            }
            out[0] = S::data();
            return finish(i2c_ok);
        }
        if (req_.rx_len == 2u) {
            if ((s1 & I2cFlag::byte_finished) == 0u) {
                return false;
            }
            S::stop();
            out[0] = S::data();
            out[1] = S::data();
            S::pos(false);
            return finish(i2c_ok);
        }
        const uint8_t remaining = static_cast<uint8_t>(req_.rx_len - pos_);
        if (remaining > 3u) {
            if ((s1 & I2cFlag::rx_not_empty) != 0u) {
                out[pos_] = S::data();
                ++pos_;
                if (req_.rx_len - pos_ == 3u) {
                    S::buffer_interrupt(false);   // BTF from here on
                }
            }
            return false;
        }
        if ((s1 & I2cFlag::byte_finished) == 0u) {
            return false;
        }
        if (remaining == 3u) {
            // N-2 in DR, N-1 in the shifter: NACK the one to come, read
            // N-2, which lets N in.
            S::ack(false);
            out[pos_] = S::data();
            ++pos_;
            return false;
        }
        // remaining == 2: N-1 in DR, N in the shifter.
        S::stop();
        out[pos_] = S::data();
        ++pos_;
        (void)wait_until([] { return S::flag(I2cFlag::rx_not_empty); });
        out[pos_] = S::data();
        ++pos_;
        return finish(i2c_ok);
    }

    static bool dma_serves_rx() {
        if constexpr (has_engines) {
            return req_.rx_len >= 2u;
        } else {
            return false;
        }
    }
    static bool dma_busy() {
        if constexpr (has_engines) {
            return TxEngine::busy();
        } else {
            return false;
        }
    }
    static void put_engines_away() {
        if constexpr (has_engines) {
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address());
            S::dma(false, false);
        }
    }

    /// A speed CHANGE costs a PE cycle: FREQ, CCR and TRISE are all
    /// enable-protected. Paid only when the speed actually moves.
    static void apply(I2cSpeed s) {
        if (s == applied_) {
            return;
        }
        applied_ = s;
        S::timing(table_[static_cast<uint8_t>(s)]);
        S::enable();
    }

    static void rebase_pclk(uint32_t pclk, uint32_t sysclk) {
        pclk_hz_ = pclk;
        for (uint8_t i = 0; i < i2c_speed_count; ++i) {
            const I2cSpeed s = static_cast<I2cSpeed>(i);
            const uint16_t rise =
                s == I2cSpeed::fast_400k ? cfg_.fast_rise_ns : cfg_.standard_rise_ns;
            const auto t = i2c_timing_for(pclk, s, cfg_.duty, rise);
            valid_[i] = t.has_value();
            table_[i] = t.value_or(I2cTiming{});
        }
        delay_rate_ = delay_rate(sysclk);   // the busy-waits count CORE cycles
    }

    /// How long start() waits for the last tenure's STOP to leave the wire.
    /// A STOP takes a bit period; what can hold it longer is a slow client
    /// stretching the clock after the last acknowledge while it digests a
    /// write, and five milliseconds - a fifth of SMBus's 25 ms clock-low
    /// limit - is the bound. What start() spends here is the kernel
    /// dispatch's time, which is why it is bounded at all.
    static constexpr uint16_t stop_drain_us = 5'000;

    /// How long start() then waits for BUSY to clear: our STOP seen by the
    /// bus takes a moment, another controller's traffic takes longer.
    static constexpr uint16_t bus_free_us = 100;

    /// Poll `pred` for at least `us` microseconds, timed on SysTick
    /// (stm32f4/delay.hpp); true as soon as it holds. delay_us() refuses
    /// when SysTick is not running, and the bound is then a count of polls
    /// - a bound still.
    template <typename Pred>
    static bool wait_for_us(Pred pred, uint16_t us) {
        for (uint16_t t = 0; t < us; ++t) {
            if (pred()) {
                return true;
            }
            (void)delay_us(delay_rate_, 1);
        }
        return pred();
    }

    template <typename Pred>
    static bool wait_until(Pred pred) {
        for (uint32_t spins = 400'000u; spins != 0u; --spins) {
            if (pred()) {
                return true;
            }
        }
        return false;
    }

    /// Half a standard-mode bit, for the unstick.
    static void spin_half_bit() { (void)delay_us(delay_rate_, 5); }

    static inline Request req_{};
    static inline uint8_t pos_ = 0;
    static inline volatile Phase phase_ = Phase::idle;
    static inline uint8_t status_ = i2c_ok;
    static inline I2cSpeed applied_ = I2cSpeed::standard_100k;
    static inline I2cHostConfig cfg_{};
    static inline uint32_t pclk_hz_ = 0;
    static inline uint32_t spurious_berr_ = 0;
    static inline I2cTiming table_[i2c_speed_count]{};
    static inline bool valid_[i2c_speed_count]{};
    static inline DelayRate delay_rate_{};
};

// =============================================================================
// I2cClient: the target side
// =============================================================================

/// What the client's ISR bodies report: one thing per call, the app's glue
/// acts on it.
enum class I2cClientEvent : uint8_t {
    none,
    addressed,       ///< ADDR: a tenure opened on one of the own addresses
    byte_received,   ///< RxNE: take() it
    byte_wanted,     ///< TxE: give() the next
    stop,            ///< STOPF: the tenure ended
    nacked,          ///< AF: the controller refused a byte this client gave
    error,           ///< BERR, ARLO or OVR
};

/**
 * I2cClient<n, pins>
 *
 * The target side. The peripheral matches the own address(es), ACKs it,
 * raises ADDR and STRETCHES SCL until the sequence that clears it runs
 * (27.3.2); every byte then comes on RxNE or is asked for on TxE, with the
 * clock stretched while the flag stands unless NOSTRETCH. A read tenure
 * ends with the controller's NACK (AF, on the error vector); a write
 * tenure with STOPF.
 *
 * NOSTRETCH IS A CHOICE WITH A PRICE, and ES0206 2.10.5 is the price: a
 * target transmitter with the clock stretch given up can violate the data
 * valid time without the overrun flag ever being set, so a byte that goes
 * out late goes out wrong and silently. The default here keeps the stretch.
 *
 *   extern "C" void I2C1_EV_IRQHandler() { act(Peer::service()); }
 *   extern "C" void I2C1_ER_IRQHandler() { act(Peer::error_service()); }
 */
template <uint8_t n, I2cPins pins>
class I2cClient {
    using S = I2c<n>;

    static_assert(i2c_pins_valid(pins),
                  "brio I2cClient: these I2C pads are not a bus - SCL and SDA must both be "
                  "named and distinct");

    using SclPin = Pin<pins.scl.port, pins.scl.pin>;
    using SdaPin = Pin<pins.sda.port, pins.sda.pin>;

public:
    I2cClient() = delete;

    using Resource = S;
    static constexpr I2cPins pin_sel = pins;

    /// Bring the instance up as a bus target. FREQ must still state the
    /// APB1 clock (the peripheral's own timings need it even as a target),
    /// so the same 2 MHz floor applies. `no_stretch` gives up the clock
    /// stretch, and with it the guarantee against an overrun.
    template <typename Clock>
    static bool init(Clock clock, const I2cAddressConfig& addr, bool no_stretch = false) {
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        S::bus_clock(true);
        S::reset();
        const auto t = i2c_timing_for(apb_hz(clock, S::on_apb2), I2cSpeed::standard_100k);
        if (!t) {
            return false;
        }
        S::timing(*t);
        if (!S::addresses(addr)) {
            return false;
        }
        S::no_stretch(no_stretch);
        S::enable();
        S::ack(true);   // answer the own address
        SclPin::function(pins.scl.function, {.open_drain = true, .speed = PinSpeed::very_high});
        SdaPin::function(pins.sda.function, {.open_drain = true, .speed = PinSpeed::very_high});
        S::event_interrupt(true);
        S::buffer_interrupt(true);
        S::error_interrupt(true);
        Nvic::enable(S::event_irq);
        Nvic::enable(S::error_irq);
        return true;
    }

    // ---- the protocol surface, polled --------------------------------------------

    /// An address match is pending (SCL held meanwhile, unless NOSTRETCH).
    static bool addressed() { return S::flag(I2cFlag::address); }
    /// Answer the match and release the clock: EV1's SR1-then-SR2 read.
    /// Returns true when the controller READS (this side transmits).
    static bool answer_address() { return (S::clear_addr() & I2C_SR2_TRA) != 0u; }
    /// Valid after answer_address(): which own address matched.
    static bool second_address_matched() { return S::second_address_matched(); }
    static bool general_call_matched() { return S::general_call_matched(); }
    /// Controller write: a byte has arrived.
    static bool data_ready() { return S::flag(I2cFlag::rx_not_empty); }
    static uint8_t take() { return S::data(); }
    /// Controller read: the shifter wants the next byte.
    static bool data_wanted() { return S::flag(I2cFlag::tx_empty); }
    static void give(uint8_t v) { S::data(v); }
    /// Whether to ACK the bytes that arrive from here on.
    static void acknowledge(bool on) { S::ack(on); }
    /// A STOP arrived - the end of a write tenure.
    static bool stop_seen() { return S::flag(I2cFlag::stop); }
    static void clear_stop() { S::clear_stopf(); }
    /// The controller refused a byte this client gave: the end of a read
    /// tenure (27.3.4: the hardware released the lines).
    static bool host_nacked() { return S::flag(I2cFlag::ack_failure); }
    static void clear_nack() { S::clear_errors(I2cFlag::ack_failure); }
    static bool overrun() { return S::flag(I2cFlag::overrun); }
    static void clear_overrun() { S::clear_errors(I2cFlag::overrun); }
    static uint32_t flags() { return S::status1(); }

    // ---- the ISR bodies ------------------------------------------------------------

    /// The EVENT vector's body: one event per call, the flag consumed where
    /// the chapter's sequence consumes it (ADDR and STOPF here; RxNE by
    /// take(), TxE by give(), which the glue owes).
    [[gnu::always_inline]] static I2cClientEvent service() {
        const uint32_t s1 = S::status1();
        if ((s1 & I2cFlag::address) != 0u) {
            host_reads_ = answer_address();
            return I2cClientEvent::addressed;
        }
        if ((s1 & I2cFlag::stop) != 0u) {
            S::clear_stopf();
            return I2cClientEvent::stop;
        }
        if ((s1 & I2cFlag::rx_not_empty) != 0u) {
            return I2cClientEvent::byte_received;
        }
        if ((s1 & I2cFlag::tx_empty) != 0u) {
            return I2cClientEvent::byte_wanted;
        }
        return I2cClientEvent::none;
    }

    /// The ERROR vector's body: the controller's NACK (the normal end of a
    /// read) or a real error, the flag cleared.
    [[gnu::always_inline]] static I2cClientEvent error_service() {
        const uint32_t s1 = S::status1();
        const uint32_t errs = s1 & I2cFlag::errors;
        if (errs == 0u) {
            return I2cClientEvent::none;
        }
        S::clear_errors(errs);
        return (errs & I2cFlag::ack_failure) != 0u ? I2cClientEvent::nacked
                                                   : I2cClientEvent::error;
    }

    /// The direction of the tenure the last `addressed` opened: true = the
    /// controller reads, this side transmits.
    static bool host_reads() { return host_reads_; }

    static void release() {
        Nvic::disable(S::event_irq);
        Nvic::disable(S::error_irq);
        S::disable();
        S::bus_clock(false);
        SclPin::release();
        SdaPin::release();
    }

private:
    static inline bool host_reads_ = false;
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

// 27.6.8 at the APB1 clock of a 180 MHz part: 45 MHz. Standard mode wants
// CCR 225 (45e6 / 200e3) and is exact; fast mode wants 38 at DUTY 2
// (45e6 / 1.2e6 = 37.5, rounded UP so the bus is slower than asked) and 5
// at 16/9.
static_assert(i2c_timing_for(45'000'000UL, I2cSpeed::standard_100k)->ccr == 225u);
static_assert(i2c_timing_for(45'000'000UL, I2cSpeed::standard_100k)->freq_mhz == 45u);
static_assert(i2c_scl_hz(45'000'000UL, *i2c_timing_for(45'000'000UL, I2cSpeed::standard_100k)) ==
              100'000UL);
static_assert(i2c_timing_for(45'000'000UL, I2cSpeed::fast_400k)->ccr == (I2C_CCR_FS | 38u));
static_assert(i2c_scl_hz(45'000'000UL, *i2c_timing_for(45'000'000UL, I2cSpeed::fast_400k)) ==
              394'736UL);
static_assert(i2c_timing_for(45'000'000UL, I2cSpeed::fast_400k, I2cDuty::ratio_16_9)->ccr ==
              (I2C_CCR_FS | I2C_CCR_DUTY | 5u));
// 27.6.8's note "it must be a multiple of 10 MHz to reach the 400 kHz
// maximum" is about the 16/9 DUTY, whose divisor is 25 x CCR: 40 MHz
// (a 160 MHz part) gives exactly 400 kHz there at CCR 4. At DUTY 2 the
// divisor is 3 x CCR, and what comes out exact is a multiple of 1.2 MHz -
// 42 MHz, a 168 MHz part's APB1, at CCR 35.
static_assert(i2c_timing_for(40'000'000UL, I2cSpeed::fast_400k, I2cDuty::ratio_16_9)->ccr ==
              (I2C_CCR_FS | I2C_CCR_DUTY | 4u));
static_assert(i2c_scl_hz(40'000'000UL, *i2c_timing_for(40'000'000UL, I2cSpeed::fast_400k,
                                                       I2cDuty::ratio_16_9)) == 400'000UL);
static_assert(i2c_timing_for(42'000'000UL, I2cSpeed::fast_400k)->ccr == (I2C_CCR_FS | 35u));
static_assert(i2c_scl_hz(42'000'000UL, *i2c_timing_for(42'000'000UL, I2cSpeed::fast_400k)) ==
              400'000UL);
// The F411's ceiling, 50 MHz of APB1 at 100 MHz of HCLK.
static_assert(i2c_timing_for(50'000'000UL, I2cSpeed::standard_100k)->ccr == 250u);
static_assert(i2c_timing_for(50'000'000UL, I2cSpeed::standard_100k)->trise == 51u);
// TRISE is the rise time in whole APB periods PLUS ONE (27.6.9): the
// chapter's own example is 1000 ns at 8 MHz, which is 0x09.
static_assert(i2c_timing_for(8'000'000UL, I2cSpeed::standard_100k)->trise == 9u);
static_assert(i2c_timing_for(45'000'000UL, I2cSpeed::standard_100k)->trise == 46u);
static_assert(i2c_timing_for(45'000'000UL, I2cSpeed::fast_400k)->trise == 14u);
// A rise time the application states, and one that would not fit the six
// bits of TRISE.
static_assert(i2c_timing_for(45'000'000UL, I2cSpeed::standard_100k, I2cDuty::ratio_2, 250)->trise ==
              12u);
static_assert(!i2c_timing_for(45'000'000UL, I2cSpeed::standard_100k, I2cDuty::ratio_2, 1500)
                   .has_value());
// FREQ's window (27.6.2) and 27.3.2's floor per mode.
static_assert(!i2c_timing_for(1'999'999UL, I2cSpeed::standard_100k).has_value());
static_assert(i2c_timing_for(2'000'000UL, I2cSpeed::standard_100k)->ccr == 10u);
static_assert(!i2c_timing_for(2'000'000UL, I2cSpeed::fast_400k).has_value());
static_assert(i2c_timing_for(4'000'000UL, I2cSpeed::fast_400k)->ccr == (I2C_CCR_FS | 4u));
static_assert(!i2c_timing_for(50'000'001UL, I2cSpeed::standard_100k).has_value());
// The chapter's CCR floors never bind inside that window: the slowest
// legal clock already asks for more than four.
static_assert((i2c_timing_for(2'000'000UL, I2cSpeed::standard_100k)->ccr & i2c_ccr_mask) >
              i2c_ccr_min_standard);

// ES0206 2.10.4's window: a repeated start in standard mode above 88 kHz.
static_assert(i2c_repeated_start_at_risk(I2cSpeed::standard_100k, 100'000UL));
static_assert(!i2c_repeated_start_at_risk(I2cSpeed::standard_100k, 80'000UL));
static_assert(!i2c_repeated_start_at_risk(I2cSpeed::fast_400k, 400'000UL));

// Table 122's ceiling on the digital filter, band by band.
static_assert(i2c_digital_filter_max(4'000'000UL, I2cSpeed::standard_100k) == 2u);
static_assert(i2c_digital_filter_max(4'000'000UL, I2cSpeed::fast_400k) == 0u);
static_assert(i2c_digital_filter_max(15'000'000UL, I2cSpeed::fast_400k) == 1u);
static_assert(i2c_digital_filter_max(45'000'000UL, I2cSpeed::fast_400k) == 15u);
static_assert(i2c_digital_filter_max(45'000'000UL, I2cSpeed::standard_100k) == 15u);

// The addresses.
static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x41}));
static_assert(!i2c_address_config_valid(I2cAddressConfig{.own = 0x80}));
static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x41, .second = 0x42}));
static_assert(!i2c_address_config_valid(I2cAddressConfig{.own = 0x41, .second = 0x90}));
static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x123, .ten_bit = true}));
static_assert(!i2c_address_config_valid(
    I2cAddressConfig{.own = 0x123, .ten_bit = true, .second = 0x10}));

// The pads.
static_assert(!i2c_pins_valid(I2cPins{}));
static_assert(!i2c_pins_valid(I2cPins{.scl = {'B', 8, PinFunction::af4},
                                      .sda = {'B', 8, PinFunction::af4}}));
static_assert(i2c_pins_valid(I2cPins{.scl = {'B', 8, PinFunction::af4},
                                     .sda = {'B', 9, PinFunction::af4}}));

} // namespace brio
