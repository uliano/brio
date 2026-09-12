/*
 * pwm.hpp
 *
 * The RP2040's PWM block (datasheet 4.5): eight identical SLICES, each
 * a 16-bit counter with a wrap value (TOP), two compare levels (CC A
 * and CC B) driving two outputs, an 8.4 fractional clock divider, a
 * phase-correct mode, an input mode in which the B pin GATES or CLOCKS
 * the counter (a duty or a frequency measured with no capture unit),
 * and one wrap event per slice that is an interrupt bit on the block's
 * single line and a DMA request. Three layers, the other strata's
 * arrangement:
 *
 *  - `Pwm` is the BLOCK: the reset gate, the global enable register
 *    that starts slices in lockstep, the four interrupt registers
 *    (raw, enable, force, status) and the ISR body.
 *  - `PwmSlice<n>` is the RESOURCE: one slice's CSR / DIV / TOP / CC /
 *    CTR, the configuration, the levels, the phase nudges, its wrap
 *    bit and its request. It decides nothing.
 *  - THE TASKS: `PwmOutput<pin, top>` (util/pwm_channel.hpp's
 *    PwmChannel), `PwmPair<pin_a, pin_b, top>` (A and its complement
 *    on B with a dead time by arithmetic - no dead-time unit exists
 *    here), `PwmEdgeCounter<pin_b>` and `PwmLevelCounter<pin_b>` (the
 *    B pin as the counter's clock or gate: a frequency and a duty
 *    measured over a window the caller times), `PwmPeriodicTick<n>`
 *    (the wrap as a tick).
 *
 * THE PINS: every GPIO is a PWM pin (table 515). GPIO n belongs to
 * slice (n / 2) mod 8 and is its A output when n is even, its B when
 * odd; GPIO 16..29 repeat slices 0..6, and the same output selected on
 * two GPIOs appears on both. ONLY A B PIN IS AN INPUT: in the level
 * and edge modes the B pin stops being an output and CC B is ignored,
 * and two B pins of one slice selected at once are ORed.
 *
 * THE PERIOD is (TOP + 1) counts, twice that in phase-correct mode
 * (the counter runs back down to 0), at a count rate of clk_sys over
 * the divider: 1..255 plus sixteenths, and 0 in the integer field is
 * 256. A level of 0 is a 0 % output with no pulse and a level of
 * TOP + 1 a 100 % output with no gap (4.5.2.2), which is why a
 * PwmOutput's `max` is TOP + 1. CC and TOP are DOUBLE BUFFERED: a write
 * lands at the next wrap (the 0-to-0 turn in phase-correct mode), so a
 * duty change is glitch-free by construction, and a DMA channel paced
 * by the wrap request streams one level per period.
 *
 * THE FREQUENCY MEASUREMENT'S RULE (4.5.2.5): the measured signal's low
 * and high periods must each exceed one clk_sys period, the edge
 * detector's own.
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "rp2040/device.hpp"

#include "rp2040/dma_engine.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/resets.hpp"
#include "util/clock.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// =============================================================================
// The vocabulary
// =============================================================================

inline constexpr uint8_t pwm_slice_count = 8;

/// Table 515: the slice and the channel of a GPIO.
constexpr uint8_t pwm_slice_of(uint8_t pin) { return static_cast<uint8_t>((pin >> 1) & 7u); }
constexpr bool pwm_pin_is_b(uint8_t pin) { return (pin & 1u) != 0u; }
constexpr bool pwm_pin_valid(uint8_t pin) { return pin < gpio_count; }

/// CSR.DIVMODE: what advances the counter.
enum class PwmDivMode : uint8_t {
    free_running = PWM_CH0_CSR_DIVMODE_VALUE_DIV,   ///< every divided clk_sys cycle
    level_high = PWM_CH0_CSR_DIVMODE_VALUE_LEVEL,   ///< the cycles the B pin is high
    rising_edge = PWM_CH0_CSR_DIVMODE_VALUE_RISE,   ///< once per rising edge on B
    falling_edge = PWM_CH0_CSR_DIVMODE_VALUE_FALL,  ///< once per falling edge on B
};

/// The 8.4 divider: `integer` 1..255, or 0 for 256; `frac` in
/// sixteenths.
struct PwmDivider {
    uint8_t integer = 1;
    uint8_t frac = 0;
    constexpr bool valid() const { return frac < 16u; }
    /// The divider in sixteenths of a cycle.
    constexpr uint32_t sixteenths() const {
        return static_cast<uint32_t>(integer == 0u ? 256u : integer) * 16u + frac;
    }
    constexpr uint32_t reg() const {
        return (static_cast<uint32_t>(integer) << PWM_CH0_DIV_INT_LSB) | (frac & PWM_CH0_DIV_FRAC_BITS);
    }
    constexpr bool operator==(const PwmDivider& o) const { return integer == o.integer && frac == o.frac; }
};

/// A divider from sixteenths: 16 (one) to 4111 (256 and fifteen
/// sixteenths); nullopt outside.
constexpr std::optional<PwmDivider> pwm_divider_of(uint32_t sixteenths) {
    if (sixteenths < 16u || sixteenths > 4111u) {
        return {};
    }
    const uint32_t integer = sixteenths / 16u;
    return PwmDivider{static_cast<uint8_t>(integer == 256u ? 0u : integer), static_cast<uint8_t>(sixteenths % 16u)};
}

/// One slice's configuration (CSR less EN, DIV, TOP).
struct PwmSliceConfig {
    PwmDivMode mode = PwmDivMode::free_running;
    PwmDivider divider{};
    uint16_t top = 0xFFFF;
    bool phase_correct = false;
    bool invert_a = false;
    bool invert_b = false;
};

constexpr bool pwm_slice_config_valid(const PwmSliceConfig& c) { return c.divider.valid(); }

constexpr uint32_t pwm_csr_of(const PwmSliceConfig& c) {
    uint32_t v = static_cast<uint32_t>(c.mode) << PWM_CH0_CSR_DIVMODE_LSB;
    if (c.phase_correct) { v |= PWM_CH0_CSR_PH_CORRECT_BITS; }
    if (c.invert_a) { v |= PWM_CH0_CSR_A_INV_BITS; }
    if (c.invert_b) { v |= PWM_CH0_CSR_B_INV_BITS; }
    return v;
}

/// The period of a free-running slice in SIXTEENTHS of a clk_sys cycle
/// (4.5.2.6): (TOP + 1) x (1 + phase_correct) x the divider.
constexpr uint32_t pwm_period_sixteenths(const PwmSliceConfig& c) {
    return (static_cast<uint32_t>(c.top) + 1u) * (c.phase_correct ? 2u : 1u) * c.divider.sixteenths();
}
/// The output frequency at `sys_hz`.
constexpr uint32_t pwm_output_hz(uint32_t sys_hz, const PwmSliceConfig& c) {
    const uint64_t period16 = pwm_period_sixteenths(c);
    return period16 == 0u ? 0u : static_cast<uint32_t>((static_cast<uint64_t>(sys_hz) * 16u) / period16);
}

/// The slice configuration that produces `hz` with the given TOP: the
/// divider is solved in sixteenths, rounded to the nearest; nullopt
/// when the rate wants a divider under one (a smaller TOP) or over 256
/// and fifteen sixteenths (a larger TOP, or the system timer). The
/// finest duty resolution is TOP 65534 at every rate the divider
/// reaches: `pwm_config_for(sys, hz)` with the default.
constexpr std::optional<PwmSliceConfig> pwm_config_for(uint32_t sys_hz, uint32_t hz, uint16_t top = 0xFFFE,
                                                       bool phase_correct = false) {
    if (hz == 0u || sys_hz == 0u) {
        return {};
    }
    const uint64_t counts = (static_cast<uint64_t>(top) + 1u) * (phase_correct ? 2u : 1u);
    const uint64_t sixteenths = (static_cast<uint64_t>(sys_hz) * 16u + (counts * hz) / 2u) / (counts * hz);
    if (sixteenths > 4111u) {
        return {};
    }
    const auto d = pwm_divider_of(static_cast<uint32_t>(sixteenths));
    if (!d) {
        return {};
    }
    return PwmSliceConfig{.mode = PwmDivMode::free_running, .divider = *d, .top = top, .phase_correct = phase_correct};
}

// =============================================================================
// The block
// =============================================================================

struct Pwm {
    Pwm() = delete;

    static constexpr uint32_t reset_bit = ResetBlock::pwm;
    static constexpr IRQn_Type irq() { return PWM_IRQ_WRAP_IRQn; }

    /// The block from its reset state: every slice disabled, DIV at
    /// one, TOP at 0xFFFF, the levels at zero.
    static bool reset() { return Resets::cycle(reset_bit); }
    static bool released() { return Resets::released(reset_bit); }
    static void hold() { Resets::hold(reset_bit); }

    /// The global enable (EN aliases every CSR.EN): the slices in
    /// `mask` started or stopped in the same cycle - lockstep.
    static void start(uint8_t mask) { hw_set(PWM->EN, mask); }
    static void stop(uint8_t mask) { hw_clear(PWM->EN, mask); }
    static uint8_t running() { return static_cast<uint8_t>(PWM->EN & PWM_EN_BITS); }

    // ---- the wrap interrupts, one bit per slice --------------------------------
    static void interrupts(uint8_t mask, bool on) {
        if (on) { hw_set(PWM->INTE, mask); } else { hw_clear(PWM->INTE, mask); }
    }
    static uint8_t interrupts() { return static_cast<uint8_t>(PWM->INTE & PWM_INTE_BITS); }
    static uint8_t raw_pending() { return static_cast<uint8_t>(PWM->INTR & PWM_INTR_BITS); }
    static uint8_t pending() { return static_cast<uint8_t>(PWM->INTS & PWM_INTS_BITS); }
    /// INTR is write-one-to-clear.
    static void clear_pending(uint8_t mask) { PWM->INTR = mask; }
    /// INTF: a wrap raised by software, for a handler under test.
    static void force(uint8_t mask, bool on) {
        if (on) { hw_set(PWM->INTF, mask); } else { hw_clear(PWM->INTF, mask); }
    }

    /// The ISR body: the raised-and-enabled slices, cleared here.
    [[gnu::always_inline]] static uint8_t isr() {
        const uint8_t up = pending();
        if (up != 0u) {
            clear_pending(up);
        }
        return up;
    }
};

// =============================================================================
// The resource
// =============================================================================

template <uint8_t n>
struct PwmSlice {
    static_assert(n < pwm_slice_count, "the RP2040 has eight PWM slices, 0..7");
    PwmSlice() = delete;

    static constexpr uint8_t index = n;
    static constexpr uint8_t bit = static_cast<uint8_t>(1u << n);
    static constexpr Dreq dreq = static_cast<Dreq>(static_cast<uint8_t>(Dreq::pwm_wrap0) + n);
    static constexpr uint32_t base = PWM_BASE + 0x14u * n;

    static volatile uint32_t& csr() { return reg_at(base, PWM_CH0_CSR_OFFSET); }
    static volatile uint32_t& div() { return reg_at(base, PWM_CH0_DIV_OFFSET); }
    static volatile uint32_t& ctr() { return reg_at(base, PWM_CH0_CTR_OFFSET); }
    static volatile uint32_t& cc() { return reg_at(base, PWM_CH0_CC_OFFSET); }
    static volatile uint32_t& top_reg() { return reg_at(base, PWM_CH0_TOP_OFFSET); }
    /// Where a DMA channel paced by the wrap pours one CC word (both
    /// levels, A low) per period.
    static volatile void* cc_address() { return &cc(); }
    static volatile void* top_address() { return &top_reg(); }

    /// The whole configuration FROM SCRATCH, the vendor's order: the
    /// slice disabled, its counter at zero, then DIV, TOP and the mode
    /// bits - the caller enables. A slice reconfigured while running
    /// keeps its counter, and a counter above a new, smaller TOP runs
    /// on to 65535 before it wraps (measured: half a millisecond of
    /// silence at divider one), and a mode flipped under it can leave
    /// the output stuck (measured, phase-correct to plain). Refused for
    /// an invalid divider. The levels are kept.
    static bool configure(const PwmSliceConfig& c) {
        if (!pwm_slice_config_valid(c)) {
            return false;
        }
        csr() = 0;
        ctr() = 0;
        div() = c.divider.reg();
        top_reg() = c.top;
        csr() = pwm_csr_of(c);
        return true;
    }
    static PwmSliceConfig config() {
        const uint32_t v = csr();
        const uint32_t d = div();
        return PwmSliceConfig{
            .mode = static_cast<PwmDivMode>((v & PWM_CH0_CSR_DIVMODE_BITS) >> PWM_CH0_CSR_DIVMODE_LSB),
            .divider = {static_cast<uint8_t>((d & PWM_CH0_DIV_INT_BITS) >> PWM_CH0_DIV_INT_LSB),
                        static_cast<uint8_t>(d & PWM_CH0_DIV_FRAC_BITS)},
            .top = static_cast<uint16_t>(top_reg() & PWM_CH0_TOP_BITS),
            .phase_correct = (v & PWM_CH0_CSR_PH_CORRECT_BITS) != 0u,
            .invert_a = (v & PWM_CH0_CSR_A_INV_BITS) != 0u,
            .invert_b = (v & PWM_CH0_CSR_B_INV_BITS) != 0u,
        };
    }

    static void enable(bool on) {
        if (on) { hw_set(csr(), PWM_CH0_CSR_EN_BITS); } else { hw_clear(csr(), PWM_CH0_CSR_EN_BITS); }
    }
    static bool enabled() { return (csr() & PWM_CH0_CSR_EN_BITS) != 0u; }

    // ---- the levels and the counter -------------------------------------------
    /// One channel's level (ch 0 = A, 1 = B), taken at the next wrap.
    static void level(uint8_t ch, uint16_t v) {
        if (ch == 0u) {
            hw_write_masked(cc(), v, PWM_CH0_CC_A_BITS);
        } else {
            hw_write_masked(cc(), static_cast<uint32_t>(v) << PWM_CH0_CC_B_LSB, PWM_CH0_CC_B_BITS);
        }
    }
    /// Both levels in one write.
    static void levels(uint16_t a, uint16_t b) { cc() = (static_cast<uint32_t>(b) << PWM_CH0_CC_B_LSB) | a; }
    static uint16_t level(uint8_t ch) {
        const uint32_t v = cc();
        return static_cast<uint16_t>(ch == 0u ? (v & PWM_CH0_CC_A_BITS) : (v >> PWM_CH0_CC_B_LSB));
    }
    static void top(uint16_t v) { top_reg() = v; }
    static uint16_t top() { return static_cast<uint16_t>(top_reg() & PWM_CH0_TOP_BITS); }
    static uint16_t counter() { return static_cast<uint16_t>(ctr() & PWM_CH0_CTR_BITS); }
    static void counter(uint16_t v) { ctr() = v; }

    /// One count forward or back while running (4.5.2.8): the bit
    /// clears when the enable pulse has been inserted or deleted, which
    /// needs the divider above one for an advance. True when it did.
    static bool advance_phase(uint32_t spins = 100'000u) { return nudge(PWM_CH0_CSR_PH_ADV_BITS, spins); }
    static bool retard_phase(uint32_t spins = 100'000u) { return nudge(PWM_CH0_CSR_PH_RET_BITS, spins); }

    // ---- the wrap ----------------------------------------------------------------
    static void interrupt(bool on) { Pwm::interrupts(bit, on); }
    static bool raw_pending() { return (Pwm::raw_pending() & bit) != 0u; }
    static bool pending() { return (Pwm::pending() & bit) != 0u; }
    static void clear_pending() { Pwm::clear_pending(bit); }

private:
    static bool nudge(uint32_t flag, uint32_t spins) {
        hw_set(csr(), flag);
        while ((csr() & flag) != 0u && spins-- != 0u) {
        }
        return (csr() & flag) == 0u;
    }
};

// =============================================================================
// The tasks
// =============================================================================

/**
 * PwmOutput<pin, top>: one output as a util/pwm_channel.hpp
 * PwmChannel, `max` = TOP + 1 (the 100 % level, 4.5.2.2). The
 * FREQUENCY BELONGS TO THE SLICE, the duty to the channel: setup()
 * configures the slice (its other output, if any, shares the period)
 * and hands the pad over; `attach()` hands the pad over alone, for the
 * second output of a slice already set up.
 */
template <uint8_t pin, uint16_t top = 0xFFFE>
struct PwmOutput {
    PwmOutput() = delete;
    static_assert(pwm_pin_valid(pin), "brio PwmOutput: no such GPIO");
    static_assert(top < 0xFFFFu, "brio PwmOutput: TOP 0xFFFE at most - the 100 % level is TOP + 1");

    using Slice = PwmSlice<pwm_slice_of(pin)>;
    static constexpr uint8_t channel = pwm_pin_is_b(pin) ? 1u : 0u;
    static constexpr uint16_t max = top + 1u;

    /// The slice free-running at `divider`, this level at zero, the
    /// pad on the PWM function, the slice enabled.
    static bool setup(PwmDivider divider = {}, bool phase_correct = false, bool invert = false) {
        PwmSliceConfig c = Slice::config();
        c.mode = PwmDivMode::free_running;
        c.divider = divider;
        c.top = top;
        c.phase_correct = phase_correct;
        if (channel == 0u) { c.invert_a = invert; } else { c.invert_b = invert; }
        if (!Slice::configure(c)) {
            return false;
        }
        attach();
        Slice::enable(true);
        return true;
    }
    /// The slice at `hz` (the divider solved for this TOP), else as
    /// setup().
    template <typename Clock>
    static bool setup_hz(Clock clock, uint32_t hz, bool phase_correct = false, bool invert = false) {
        const auto c = pwm_config_for(clock_hz(clock), hz, top, phase_correct);
        if (!c) {
            return false;
        }
        return setup(c->divider, phase_correct, invert);
    }
    /// This level at zero and the pad handed over; the slice untouched.
    static void attach() {
        Slice::level(channel, 0);
        Pin<pin>::function(PinFunction::pwm);
    }

    static void duty(uint16_t v) { Slice::level(channel, v > max ? max : v); }
    static uint16_t duty() { return Slice::level(channel); }

    static void release() {
        Pin<pin>::release();
        Slice::level(channel, 0);
    }
};

/**
 * PwmPair<pin_a, pin_b, top>: A and its COMPLEMENT on B, both pins of
 * one slice, with a dead time by arithmetic - no dead-time unit exists
 * on this block. A is high for the first `v` counts of the period; B
 * is INVERTED and its raw level is `v + dead`, so B is low for the
 * first `v + dead` counts and high for the rest: B rises `dead` counts
 * after A falls. In the plain mode A rises and B falls together at the
 * wrap, so the dead time is on ONE edge; in PHASE-CORRECT mode both
 * outputs are centred (A on the wrap, B on TOP) and the dead time
 * appears at both transitions - the mode a bridge takes. `max` = TOP
 * + 1, B's level clamped to it.
 */
template <uint8_t pin_a, uint8_t pin_b, uint16_t top = 0xFFFE>
struct PwmPair {
    PwmPair() = delete;
    static_assert(pwm_pin_valid(pin_a) && pwm_pin_valid(pin_b), "brio PwmPair: no such GPIO");
    static_assert(pwm_slice_of(pin_a) == pwm_slice_of(pin_b) && !pwm_pin_is_b(pin_a) && pwm_pin_is_b(pin_b),
                  "brio PwmPair: the pair is the A and the B output of ONE slice (table 515: GPIO 2k and 2k + 1)");
    static_assert(top < 0xFFFFu, "brio PwmPair: TOP 0xFFFE at most");

    using Slice = PwmSlice<pwm_slice_of(pin_a)>;
    static constexpr uint16_t max = top + 1u;

    static bool setup(PwmDivider divider = {}, uint16_t dead_time = 0, bool phase_correct = false) {
        if (dead_time > top) {
            return false;
        }
        dead_ = dead_time;
        if (!Slice::configure({.mode = PwmDivMode::free_running, .divider = divider, .top = top,
                               .phase_correct = phase_correct, .invert_a = false, .invert_b = true})) {
            return false;
        }
        Slice::levels(0, dead_time);
        Pin<pin_a>::function(PinFunction::pwm);
        Pin<pin_b>::function(PinFunction::pwm);
        Slice::enable(true);
        return true;
    }
    /// A high for v counts, B low for v + dead (inverted).
    static void duty(uint16_t v) {
        if (v > max) { v = max; }
        const uint32_t b = static_cast<uint32_t>(v) + dead_;
        Slice::levels(v, static_cast<uint16_t>(b > max ? max : b));
    }
    static uint16_t dead_time() { return dead_; }

    static void release() {
        Pin<pin_a>::release();
        Pin<pin_b>::release();
        Slice::enable(false);
    }

private:
    static inline uint16_t dead_ = 0;
};

/**
 * PwmEdgeCounter<pin_b>: the B pin as the counter's clock - one count
 * per rising (or falling) edge, divided by the slice's divider. Over a
 * window the caller times, the count is the frequency. The counter
 * wraps at TOP + 1 and the wrap is the slice's interrupt: a window
 * longer than 65536 edges counts them on the wrap.
 */
template <uint8_t pin_b>
struct PwmEdgeCounter {
    PwmEdgeCounter() = delete;
    static_assert(pwm_pin_valid(pin_b) && pwm_pin_is_b(pin_b),
                  "brio PwmEdgeCounter: only a B pin is an input (table 515: an odd GPIO)");
    using Slice = PwmSlice<pwm_slice_of(pin_b)>;

    static bool setup(bool falling = false, PwmDivider divider = {}, uint16_t top = 0xFFFF,
                      PinPull pull = PinPull::none) {
        if (!Slice::configure({.mode = falling ? PwmDivMode::falling_edge : PwmDivMode::rising_edge,
                               .divider = divider, .top = top})) {
            return false;
        }
        Slice::counter(0);
        Pin<pin_b>::function(PinFunction::pwm, {.pull = pull});
        return true;
    }
    static void run(bool on) { Slice::enable(on); }
    static void restart() { Slice::counter(0); }
    static uint16_t count() { return Slice::counter(); }
    static void release() {
        Slice::enable(false);
        Pin<pin_b>::release();
    }
};

/**
 * PwmLevelCounter<pin_b>: the B pin as the counter's gate - the
 * counter advances on every divided clk_sys cycle the pin is HIGH.
 * Over a window the caller times, the count against the window's
 * cycles is the duty.
 */
template <uint8_t pin_b>
struct PwmLevelCounter {
    PwmLevelCounter() = delete;
    static_assert(pwm_pin_valid(pin_b) && pwm_pin_is_b(pin_b),
                  "brio PwmLevelCounter: only a B pin is an input (table 515: an odd GPIO)");
    using Slice = PwmSlice<pwm_slice_of(pin_b)>;

    static bool setup(PwmDivider divider = {}, uint16_t top = 0xFFFF, PinPull pull = PinPull::none) {
        if (!Slice::configure({.mode = PwmDivMode::level_high, .divider = divider, .top = top})) {
            return false;
        }
        Slice::counter(0);
        Pin<pin_b>::function(PinFunction::pwm, {.pull = pull});
        return true;
    }
    static void run(bool on) { Slice::enable(on); }
    static void restart() { Slice::counter(0); }
    static uint16_t count() { return Slice::counter(); }
    static void release() {
        Slice::enable(false);
        Pin<pin_b>::release();
    }
};

/**
 * PwmPeriodicTick<n>: a slice's wrap as a periodic interrupt, no pad -
 * the block as a source of regular requests (4.5.2.7's last sentence),
 * and the pacing request a DMA channel takes (`Slice::dreq`).
 */
template <uint8_t n>
struct PwmPeriodicTick {
    PwmPeriodicTick() = delete;
    using Slice = PwmSlice<n>;

    static bool setup(PwmDivider divider, uint16_t top, bool interrupt = true) {
        if (!Slice::configure({.mode = PwmDivMode::free_running, .divider = divider, .top = top})) {
            return false;
        }
        Slice::counter(0);
        Slice::clear_pending();
        Slice::interrupt(interrupt);
        Slice::enable(true);
        return true;
    }
    template <typename Clock>
    static bool setup_hz(Clock clock, uint32_t hz, bool interrupt = true) {
        // The largest TOP the divider reaches at this rate: the divider
        // at one when it fits, else solved for TOP 0xFFFF.
        const uint64_t counts = (static_cast<uint64_t>(clock_hz(clock)) + hz / 2u) / hz;
        if (counts >= 1u && counts <= 0x10000u) {
            return setup({}, static_cast<uint16_t>(counts - 1u), interrupt);
        }
        const auto c = pwm_config_for(clock_hz(clock), hz, 0xFFFF);
        if (!c) {
            return false;
        }
        return setup(c->divider, 0xFFFF, interrupt);
    }
    static void stop() {
        Slice::interrupt(false);
        Slice::enable(false);
    }
    static constexpr uint8_t flag = Slice::bit;
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

static_assert(pwm_slice_of(0) == 0u && pwm_slice_of(13) == 6u && pwm_slice_of(15) == 7u && pwm_slice_of(16) == 0u &&
              pwm_slice_of(29) == 6u);
static_assert(!pwm_pin_is_b(12) && pwm_pin_is_b(13) && pwm_pin_is_b(17) && !pwm_pin_is_b(20));
static_assert(PwmDivider{1, 0}.sixteenths() == 16u && PwmDivider{0, 0}.sixteenths() == 4096u &&
              PwmDivider{2, 8}.sixteenths() == 40u);
static_assert(pwm_divider_of(40)->integer == 2u && pwm_divider_of(40)->frac == 8u && pwm_divider_of(4096)->integer == 0u &&
              !pwm_divider_of(15).has_value() && !pwm_divider_of(4112).has_value());
// 125 MHz: TOP 124 at one is 1 MHz; TOP 65534 at 1 kHz wants 1.907 -> 31
// sixteenths (1 and 15/16), which is 984 Hz - the finest duty at the
// nearest divider; 7.5 Hz is the floor at TOP 0xFFFF and 256.
static_assert(pwm_output_hz(125'000'000, {.top = 124}) == 1'000'000u);
static_assert(pwm_output_hz(125'000'000, {.top = 124, .phase_correct = true}) == 500'000u);
static_assert(pwm_config_for(125'000'000, 1000)->divider.sixteenths() == 31u);
static_assert(pwm_output_hz(125'000'000, *pwm_config_for(125'000'000, 1000)) == 984u);
static_assert(pwm_output_hz(125'000'000, {.divider = {0, 0}}) == 7u);
static_assert(!pwm_config_for(125'000'000, 1).has_value());
static_assert(pwm_config_for(125'000'000, 20'000, 6249)->divider == PwmDivider{1, 0});
static_assert(PwmOutput<13>::max == 0xFFFFu && PwmOutput<13, 999>::max == 1000u);
static_assert(PwmSlice<3>::dreq == Dreq::pwm_wrap3);
static_assert((pwm_csr_of({.mode = PwmDivMode::level_high, .phase_correct = true, .invert_b = true}) &
               (PWM_CH0_CSR_DIVMODE_BITS | PWM_CH0_CSR_PH_CORRECT_BITS | PWM_CH0_CSR_B_INV_BITS)) ==
              ((1u << PWM_CH0_CSR_DIVMODE_LSB) | PWM_CH0_CSR_PH_CORRECT_BITS | PWM_CH0_CSR_B_INV_BITS));

} // namespace brio
