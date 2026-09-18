/*
 * pwm.hpp
 *
 * The PWM block (datasheet 12.5): TWELVE identical SLICES, each a 16-bit
 * counter with a wrap value (TOP), two compare levels (CC A and CC B)
 * driving two outputs, an 8.4 fractional clock divider off clk_sys, a
 * phase-correct mode, an input mode in which the B pin GATES or CLOCKS
 * the counter (a duty or a frequency measured with no capture unit), and
 * one wrap event per slice that is a bit in the raw status, a DMA request
 * and - on this chip - a bit in EITHER OF TWO interrupt lines. Three
 * layers, the other strata's arrangement:
 *
 *  - `Pwm` is the BLOCK: the reset gate, the global enable register that
 *    starts slices in lockstep, the shared raw status and THE TWO SETS OF
 *    INTERRUPT REGISTERS, and an ISR body per line.
 *  - `PwmSlice<n>` is the RESOURCE: one slice's CSR / DIV / TOP / CC /
 *    CTR, the configuration, the levels, the phase nudges, its wrap bit
 *    and its request. It decides nothing.
 *  - THE TASKS: `PwmOutput<pin, top>` (util/pwm_channel.hpp's
 *    PwmChannel), `PwmPair<pin_a, pin_b, top>` (A and its complement on B
 *    with a dead time by arithmetic - no dead-time unit exists here),
 *    `PwmEdgeCounter<pin_b>` and `PwmLevelCounter<pin_b>` (the B pin as
 *    the counter's clock or gate: a frequency and a duty measured over a
 *    window the caller times), `PwmPeriodicTick<n, line>` (the wrap as a
 *    tick on one of the two lines).
 *
 * WHAT THIS CHIP ADDED (12.5.1.1). Four slices, 8..11, whose pads are
 * GPIO 32..47 - so they exist on both packages but reach a pad only on
 * the QFN-80, and on the QFN-60 they are REPEATING TIMERS with no output,
 * which is what the chapter's own note calls them. And a SECOND shared
 * interrupt line: INTR is one raw register, but INTE / INTF / INTS come
 * in two sets, IRQ0 and IRQ1, so two handlers can own disjoint sets of
 * slices. The line is a template parameter here for the reason the system
 * timer's alarm index is one: what makes a slice's wrap this handler's is
 * the app binding `isr_pwm_wrap_0` or `isr_pwm_wrap_1`, a name the linker
 * resolves, so the choice belongs to the build. A SLICE BELONGS TO ONE
 * LINE: enabled in both, whichever handler runs first clears its raw flag
 * and the other line's handler finds nothing.
 *
 * THE PINS (table 1130, and the GPIO function table of 9.4). Every GPIO
 * is a PWM pin. GPIO 0..31 belong to slices 0..7 - GPIO n to slice
 * (n / 2) mod 8, its A output when n is even, its B when odd, the pattern
 * repeating at GPIO 16 - and GPIO 32..47 belong to slices 8..11 the same
 * way, repeating at GPIO 40. The same output selected on two GPIOs
 * appears on both. ONLY A B PIN IS AN INPUT: in the level and edge modes
 * the B pin stops being an output and CC B is ignored, and two B pins of
 * one slice selected at once are ORed.
 *
 * THE PERIOD is (TOP + 1) counts, twice that in phase-correct mode (the
 * counter runs back down to 0), at a count rate of clk_sys over the
 * divider: 1..255 plus sixteenths, and 0 in the integer field is 256 -
 * WITH NO FRACTION, because 12.5.2.6 forbids a DIV_FRAC bit while DIV_INT
 * is 0, which puts the top of the range at 256 exactly. A level of 0 is a
 * 0 % output with no pulse and a level of TOP + 1 a 100 % output with no
 * gap (12.5.2.2), which is why a PwmOutput's `max` is TOP + 1. CC and TOP
 * are DOUBLE BUFFERED: a write lands at the next wrap (the 0-to-0 turn in
 * phase-correct mode), so a duty change is glitch-free by construction,
 * and a DMA channel paced by the wrap request streams one level per
 * period.
 *
 * THE FREQUENCY MEASUREMENT'S RULE (12.5.2.5): the measured signal's low
 * and high periods must each exceed one clk_sys period, the edge
 * detector's own.
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "rp2350/device.hpp"

#include "rp2350/core.hpp"
#include "rp2350/dma_engine.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/resets.hpp"
#include "util/clock.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// =============================================================================
// The vocabulary
// =============================================================================

inline constexpr uint8_t pwm_slice_count = 12;
/// The two interrupt lines of 12.5.2.7.
inline constexpr uint8_t pwm_irq_lines = 2;

/// Table 1130: the slice of a GPIO. GPIO 0..31 walk slices 0..7 twice,
/// GPIO 32..47 walk slices 8..11 twice.
constexpr uint8_t pwm_slice_of(uint8_t pin) {
    return pin < 32u ? static_cast<uint8_t>((pin >> 1) & 7u)
                     : static_cast<uint8_t>(8u + ((pin >> 1) & 3u));
}
/// The channel of a GPIO: B on an odd pin, A on an even one.
constexpr bool pwm_pin_is_b(uint8_t pin) { return (pin & 1u) != 0u; }
/// Whether THIS image's package brings the pad out (device.hpp's fact).
constexpr bool pwm_pin_valid(uint8_t pin) { return pin < gpio_count; }
/// Whether `slice` reaches a pad in this package: slices 8..11 do so on
/// the QFN-80 alone, and on the QFN-60 they are timers with no output
/// (12.5.2's note).
constexpr bool pwm_slice_has_pads(uint8_t slice) { return slice < 8u || gpio_count > 32u; }

/// CSR.DIVMODE: what advances the counter.
enum class PwmDivMode : uint8_t {
    free_running = PWM_CH0_CSR_DIVMODE_VALUE_DIV,   ///< every divided clk_sys cycle
    level_high = PWM_CH0_CSR_DIVMODE_VALUE_LEVEL,   ///< the cycles the B pin is high
    rising_edge = PWM_CH0_CSR_DIVMODE_VALUE_RISE,   ///< once per rising edge on B
    falling_edge = PWM_CH0_CSR_DIVMODE_VALUE_FALL,  ///< once per falling edge on B
};

/// The 8.4 divider: `integer` 1..255, or 0 for 256; `frac` in
/// sixteenths, and NONE when the integer field is 0 (12.5.2.6).
struct PwmDivider {
    uint8_t integer = 1;
    uint8_t frac = 0;
    constexpr bool valid() const { return frac < 16u && !(integer == 0u && frac != 0u); }
    /// The divider in sixteenths of a cycle.
    constexpr uint32_t sixteenths() const {
        return static_cast<uint32_t>(integer == 0u ? 256u : integer) * 16u + frac;
    }
    constexpr uint32_t reg() const {
        return (static_cast<uint32_t>(integer) << PWM_CH0_DIV_INT_LSB) | (frac & PWM_CH0_DIV_FRAC_BITS);
    }
    constexpr bool operator==(const PwmDivider& o) const { return integer == o.integer && frac == o.frac; }
};

/// A divider from sixteenths: 16 (one) to 4096 (256, and no fraction
/// above 255 and fifteen sixteenths); nullopt outside.
constexpr std::optional<PwmDivider> pwm_divider_of(uint32_t sixteenths) {
    if (sixteenths < 16u || sixteenths > 4096u) {
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
/// (12.5.2.6): (TOP + 1) x (1 + phase_correct) x the divider.
constexpr uint32_t pwm_period_sixteenths(const PwmSliceConfig& c) {
    return (static_cast<uint32_t>(c.top) + 1u) * (c.phase_correct ? 2u : 1u) * c.divider.sixteenths();
}
/// The output frequency at `sys_hz`.
constexpr uint32_t pwm_output_hz(uint32_t sys_hz, const PwmSliceConfig& c) {
    const uint64_t period16 = pwm_period_sixteenths(c);
    return period16 == 0u ? 0u : static_cast<uint32_t>((static_cast<uint64_t>(sys_hz) * 16u) / period16);
}

/// The slice configuration that produces `hz` with the given TOP: the
/// divider is solved in sixteenths, rounded to the nearest; nullopt when
/// the rate wants a divider under one (a smaller TOP) or over 256 (a
/// larger TOP, or a system timer). The finest duty resolution is TOP
/// 65534 at every rate the divider reaches: `pwm_config_for(sys, hz)`
/// with the default.
constexpr std::optional<PwmSliceConfig> pwm_config_for(uint32_t sys_hz, uint32_t hz, uint16_t top = 0xFFFE,
                                                       bool phase_correct = false) {
    if (hz == 0u || sys_hz == 0u) {
        return {};
    }
    const uint64_t counts = (static_cast<uint64_t>(top) + 1u) * (phase_correct ? 2u : 1u);
    const uint64_t sixteenths = (static_cast<uint64_t>(sys_hz) * 16u + (counts * hz) / 2u) / (counts * hz);
    if (sixteenths > 4096u) {
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
    /// Every slice's bit: twelve here, eight on the RP2040.
    static constexpr uint16_t all_slices = PWM_EN_BITS;

    /// The interrupt line `line` reaches the controller on.
    template <uint8_t line = 0>
    static constexpr IRQn_Type irq() {
        static_assert(line < pwm_irq_lines, "the RP2350's PWM has two interrupt lines, 0 and 1");
        return line == 0u ? PWM_IRQ_WRAP_0_IRQn : PWM_IRQ_WRAP_1_IRQn;
    }

    /// The block from its reset state: every slice disabled, DIV at one,
    /// TOP at 0xFFFF, the levels at zero. A CYCLE and not a release,
    /// because a processor reset on this chip leaves the peripherals as
    /// the previous image left them.
    static bool reset() { return Resets::cycle(reset_bit); }
    static bool released() { return Resets::released(reset_bit); }
    static void hold() { Resets::hold(reset_bit); }

    /// The global enable (EN aliases every CSR.EN): the slices in `mask`
    /// started or stopped in the same cycle - lockstep (12.5.2.8).
    static void start(uint16_t mask) { hw_set(PWM->EN, mask); }
    static void stop(uint16_t mask) { hw_clear(PWM->EN, mask); }
    static uint16_t running() { return static_cast<uint16_t>(PWM->EN & PWM_EN_BITS); }

    // ---- the raw status, shared by both lines ----------------------------------

    static uint16_t raw_pending() { return static_cast<uint16_t>(PWM->INTR & PWM_INTR_BITS); }
    /// INTR is write-one-to-clear.
    static void clear_pending(uint16_t mask) { PWM->INTR = mask; }

    // ---- the two interrupt lines, one register set each ------------------------

    template <uint8_t line = 0>
    static void interrupts(uint16_t mask, bool on) {
        static_assert(line < pwm_irq_lines, "the RP2350's PWM has two interrupt lines, 0 and 1");
        if (on) { hw_set(inte<line>(), mask); } else { hw_clear(inte<line>(), mask); }
    }
    template <uint8_t line = 0>
    static uint16_t interrupts() {
        static_assert(line < pwm_irq_lines, "the RP2350's PWM has two interrupt lines, 0 and 1");
        return static_cast<uint16_t>(inte<line>() & PWM_IRQ0_INTE_BITS);
    }
    /// The masked status of one line: (INTR & INTE) | INTF.
    template <uint8_t line = 0>
    static uint16_t pending() {
        static_assert(line < pwm_irq_lines, "the RP2350's PWM has two interrupt lines, 0 and 1");
        return static_cast<uint16_t>(ints<line>() & PWM_IRQ0_INTS_BITS);
    }
    /// INTF: a wrap raised by software on one line, for a handler under
    /// test. It sets no raw flag, so `clear_pending` does not take it
    /// back down - the force is cleared by writing it away.
    template <uint8_t line = 0>
    static void force(uint16_t mask, bool on) {
        static_assert(line < pwm_irq_lines, "the RP2350's PWM has two interrupt lines, 0 and 1");
        if (on) { hw_set(intf<line>(), mask); } else { hw_clear(intf<line>(), mask); }
    }

    /// The ISR body of one line: the raised-and-enabled slices, cleared
    /// here. A slice enabled on BOTH lines is served by whichever handler
    /// runs first, the raw flag being one.
    template <uint8_t line = 0>
    [[gnu::always_inline]] static uint16_t isr() {
        static_assert(line < pwm_irq_lines, "the RP2350's PWM has two interrupt lines, 0 and 1");
        const uint16_t up = pending<line>();
        if (up != 0u) {
            clear_pending(up);
        }
        return up;
    }

private:
    /// The three registers of line 1 sit one stride of twelve bytes past
    /// line 0's (12.5.3), read off the device header rather than typed.
    static constexpr uint32_t line_stride = PWM_IRQ1_INTE_OFFSET - PWM_IRQ0_INTE_OFFSET;
    static_assert(PWM_IRQ1_INTF_OFFSET - PWM_IRQ0_INTF_OFFSET == line_stride &&
                      PWM_IRQ1_INTS_OFFSET - PWM_IRQ0_INTS_OFFSET == line_stride,
                  "brio Pwm: the two interrupt register sets are not evenly spaced");

    template <uint8_t line>
    static volatile uint32_t& inte() {
        static_assert(line < pwm_irq_lines, "the RP2350's PWM has two interrupt lines, 0 and 1");
        return reg_at(PWM_BASE, PWM_IRQ0_INTE_OFFSET + line_stride * line);
    }
    template <uint8_t line>
    static volatile uint32_t& intf() {
        static_assert(line < pwm_irq_lines, "the RP2350's PWM has two interrupt lines, 0 and 1");
        return reg_at(PWM_BASE, PWM_IRQ0_INTF_OFFSET + line_stride * line);
    }
    template <uint8_t line>
    static volatile uint32_t& ints() {
        static_assert(line < pwm_irq_lines, "the RP2350's PWM has two interrupt lines, 0 and 1");
        return reg_at(PWM_BASE, PWM_IRQ0_INTS_OFFSET + line_stride * line);
    }
};

// =============================================================================
// The resource
// =============================================================================

template <uint8_t n>
struct PwmSlice {
    static_assert(n < pwm_slice_count, "the RP2350 has twelve PWM slices, 0..11");
    PwmSlice() = delete;

    static constexpr uint8_t index = n;
    static constexpr uint16_t bit = static_cast<uint16_t>(1u << n);
    static constexpr Dreq dreq = static_cast<Dreq>(static_cast<uint8_t>(Dreq::pwm_wrap0) + n);
    static constexpr uint32_t base = PWM_BASE + 0x14u * n;
    /// Whether this slice reaches a pad in this package (12.5.2's note:
    /// on the QFN-60 the four highest slices are timers alone).
    static constexpr bool has_pads = pwm_slice_has_pads(n);

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
    /// bits - the caller enables. It starts from scratch because a slice
    /// reconfigured under a running counter keeps that counter, and one
    /// left above a new and smaller TOP will not meet it on this lap.
    /// Refused for an invalid divider. The levels are kept.
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

    /// One count forward or back while running (12.5.2.8): the bit clears
    /// when the enable pulse has been inserted or deleted, which needs the
    /// divider above one for an advance. True when it did.
    static bool advance_phase(uint32_t spins = 100'000u) { return nudge(PWM_CH0_CSR_PH_ADV_BITS, spins); }
    static bool retard_phase(uint32_t spins = 100'000u) { return nudge(PWM_CH0_CSR_PH_RET_BITS, spins); }

    // ---- the wrap ----------------------------------------------------------------
    template <uint8_t line = 0>
    static void interrupt(bool on) { Pwm::interrupts<line>(bit, on); }
    template <uint8_t line = 0>
    static bool pending() { return (Pwm::pending<line>() & bit) != 0u; }
    static bool raw_pending() { return (Pwm::raw_pending() & bit) != 0u; }
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
 * PwmOutput<pin, top>: one output as a util/pwm_channel.hpp PwmChannel,
 * `max` = TOP + 1 (the 100 % level, 12.5.2.2). The FREQUENCY BELONGS TO
 * THE SLICE, the duty to the channel: setup() configures the slice (its
 * other output, if any, shares the period) and hands the pad over;
 * `attach()` hands the pad over alone, for the second output of a slice
 * already set up.
 */
template <uint8_t pin, uint16_t top = 0xFFFE>
struct PwmOutput {
    PwmOutput() = delete;
    static_assert(pwm_pin_valid(pin), "brio PwmOutput: this package has no such GPIO");
    static_assert(top < 0xFFFFu, "brio PwmOutput: TOP 0xFFFE at most - the 100 % level is TOP + 1");

    using Slice = PwmSlice<pwm_slice_of(pin)>;
    static constexpr uint8_t channel = pwm_pin_is_b(pin) ? 1u : 0u;
    static constexpr uint16_t max = top + 1u;

    /// The slice free-running at `divider`, this level at zero, the pad on
    /// the PWM function, the slice enabled.
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
        (void)Pin<pin>::function(PinFunction::pwm);
    }

    static void duty(uint16_t v) { Slice::level(channel, v > max ? max : v); }
    static uint16_t duty() { return Slice::level(channel); }

    static void release() {
        (void)Pin<pin>::release();
        Slice::level(channel, 0);
    }
};

/**
 * PwmPair<pin_a, pin_b, top>: A and its COMPLEMENT on B, both pins of one
 * slice, with a dead time by arithmetic - no dead-time unit exists on
 * this block. A is high for the first `v` counts of the period; B is
 * INVERTED and its raw level is `v + dead`, so B is low for the first
 * `v + dead` counts and high for the rest: B rises `dead` counts after A
 * falls. In the plain mode A rises and B falls together at the wrap, so
 * the dead time is on ONE edge; in PHASE-CORRECT mode both outputs are
 * centred (A on the wrap, B on TOP) and the dead time appears at both
 * transitions - the mode a bridge takes. `max` = TOP + 1, B's level
 * clamped to it.
 */
template <uint8_t pin_a, uint8_t pin_b, uint16_t top = 0xFFFE>
struct PwmPair {
    PwmPair() = delete;
    static_assert(pwm_pin_valid(pin_a) && pwm_pin_valid(pin_b), "brio PwmPair: this package has no such GPIO");
    static_assert(pwm_slice_of(pin_a) == pwm_slice_of(pin_b) && !pwm_pin_is_b(pin_a) && pwm_pin_is_b(pin_b),
                  "brio PwmPair: the pair is the A and the B output of ONE slice (table 1130: an even GPIO "
                  "and an odd one of the same slice)");
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
        (void)Pin<pin_a>::function(PinFunction::pwm);
        (void)Pin<pin_b>::function(PinFunction::pwm);
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
        (void)Pin<pin_a>::release();
        (void)Pin<pin_b>::release();
        Slice::enable(false);
    }

private:
    static inline uint16_t dead_ = 0;
};

/**
 * PwmEdgeCounter<pin_b>: the B pin as the counter's clock - one count per
 * rising (or falling) edge, divided by the slice's divider. Over a window
 * the caller times, the count is the frequency. The counter wraps at
 * TOP + 1 and the wrap is the slice's interrupt: a window longer than
 * 65536 edges counts them on the wrap.
 */
template <uint8_t pin_b>
struct PwmEdgeCounter {
    PwmEdgeCounter() = delete;
    static_assert(pwm_pin_valid(pin_b) && pwm_pin_is_b(pin_b),
                  "brio PwmEdgeCounter: only a B pin is an input (table 1130: an odd GPIO this package has)");
    using Slice = PwmSlice<pwm_slice_of(pin_b)>;

    static bool setup(bool falling = false, PwmDivider divider = {}, uint16_t top = 0xFFFF,
                      PinPull pull = PinPull::none) {
        if (!Slice::configure({.mode = falling ? PwmDivMode::falling_edge : PwmDivMode::rising_edge,
                               .divider = divider, .top = top})) {
            return false;
        }
        Slice::counter(0);
        return Pin<pin_b>::function(PinFunction::pwm, {.pull = pull});
    }
    static void run(bool on) { Slice::enable(on); }
    static void restart() { Slice::counter(0); }
    static uint16_t count() { return Slice::counter(); }
    static void release() {
        Slice::enable(false);
        (void)Pin<pin_b>::release();
    }
};

/**
 * PwmLevelCounter<pin_b>: the B pin as the counter's gate - the counter
 * advances on every divided clk_sys cycle the pin is HIGH. Over a window
 * the caller times, the count against the window's cycles is the duty.
 */
template <uint8_t pin_b>
struct PwmLevelCounter {
    PwmLevelCounter() = delete;
    static_assert(pwm_pin_valid(pin_b) && pwm_pin_is_b(pin_b),
                  "brio PwmLevelCounter: only a B pin is an input (table 1130: an odd GPIO this package has)");
    using Slice = PwmSlice<pwm_slice_of(pin_b)>;

    static bool setup(PwmDivider divider = {}, uint16_t top = 0xFFFF, PinPull pull = PinPull::none) {
        if (!Slice::configure({.mode = PwmDivMode::level_high, .divider = divider, .top = top})) {
            return false;
        }
        Slice::counter(0);
        return Pin<pin_b>::function(PinFunction::pwm, {.pull = pull});
    }
    static void run(bool on) { Slice::enable(on); }
    static void restart() { Slice::counter(0); }
    static uint16_t count() { return Slice::counter(); }
    static void release() {
        Slice::enable(false);
        (void)Pin<pin_b>::release();
    }
};

/**
 * PwmPeriodicTick<n, line>: a slice's wrap as a periodic interrupt on one
 * of the two lines, no pad - the block as a source of regular requests
 * (12.5.2.7's last paragraph, and 12.5.1.1's own reason for the second
 * line), and the pacing request a DMA channel takes (`Slice::dreq`). On
 * the QFN-60 the four highest slices reach no pad at all, which is what
 * they are for there.
 */
template <uint8_t n, uint8_t line = 0>
struct PwmPeriodicTick {
    PwmPeriodicTick() = delete;
    static_assert(line < pwm_irq_lines, "the RP2350's PWM has two interrupt lines, 0 and 1");
    using Slice = PwmSlice<n>;

    static bool setup(PwmDivider divider, uint16_t top, bool interrupt = true) {
        if (!Slice::configure({.mode = PwmDivMode::free_running, .divider = divider, .top = top})) {
            return false;
        }
        Slice::counter(0);
        Slice::clear_pending();
        Slice::template interrupt<line>(interrupt);
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
        Slice::template interrupt<line>(false);
        Slice::enable(false);
    }
    static constexpr uint16_t flag = Slice::bit;
    static constexpr IRQn_Type irq() { return Pwm::irq<line>(); }
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

// Table 1130, in both halves of the map and at both of its repeats.
static_assert(pwm_slice_of(0) == 0u && pwm_slice_of(13) == 6u && pwm_slice_of(15) == 7u &&
              pwm_slice_of(16) == 0u && pwm_slice_of(29) == 6u && pwm_slice_of(31) == 7u);
static_assert(pwm_slice_of(32) == 8u && pwm_slice_of(35) == 9u && pwm_slice_of(39) == 11u &&
              pwm_slice_of(40) == 8u && pwm_slice_of(44) == 10u && pwm_slice_of(47) == 11u);
static_assert(!pwm_pin_is_b(12) && pwm_pin_is_b(13) && pwm_pin_is_b(33) && !pwm_pin_is_b(46));
// The divider: one to 256, and no fraction on top of 256 (12.5.2.6).
static_assert(PwmDivider{1, 0}.sixteenths() == 16u && PwmDivider{0, 0}.sixteenths() == 4096u &&
              PwmDivider{2, 8}.sixteenths() == 40u);
static_assert(PwmDivider{0, 0}.valid() && !PwmDivider{0, 1}.valid() && !PwmDivider{1, 16}.valid());
static_assert(pwm_divider_of(40)->integer == 2u && pwm_divider_of(40)->frac == 8u &&
              pwm_divider_of(4096)->integer == 0u && pwm_divider_of(4096)->frac == 0u &&
              !pwm_divider_of(15).has_value() && !pwm_divider_of(4097).has_value());
// 150 MHz: TOP 149 at one is 1 MHz; TOP 65534 at 1 kHz wants 2.3125 -> 37
// sixteenths, which is 989 Hz - the finest duty at the nearest divider; 8
// Hz is the floor at TOP 0xFFFF and 256.
static_assert(pwm_output_hz(150'000'000, {.top = 149}) == 1'000'000u);
static_assert(pwm_output_hz(150'000'000, {.top = 149, .phase_correct = true}) == 500'000u);
static_assert(pwm_config_for(150'000'000, 1000)->divider.sixteenths() == 37u);
static_assert(pwm_output_hz(150'000'000, *pwm_config_for(150'000'000, 1000)) == 989u);
static_assert(pwm_output_hz(150'000'000, {.divider = {0, 0}}) == 8u);
static_assert(!pwm_config_for(150'000'000, 1).has_value());
static_assert(pwm_config_for(150'000'000, 20'000, 7499)->divider == PwmDivider{1, 0});
static_assert(PwmOutput<13>::max == 0xFFFFu && PwmOutput<13, 999>::max == 1000u);
// The twelve slices, their requests and their two lines.
static_assert(Pwm::all_slices == 0x0FFFu && PwmSlice<11>::bit == 0x0800u);
static_assert(PwmSlice<3>::dreq == Dreq::pwm_wrap3 && PwmSlice<11>::dreq == Dreq::pwm_wrap11);
static_assert(Pwm::irq<0>() == PWM_IRQ_WRAP_0_IRQn && Pwm::irq<1>() == PWM_IRQ_WRAP_1_IRQn);
static_assert(PwmSlice<0>::has_pads && PwmSlice<7>::has_pads);
static_assert((pwm_csr_of({.mode = PwmDivMode::level_high, .phase_correct = true, .invert_b = true}) &
               (PWM_CH0_CSR_DIVMODE_BITS | PWM_CH0_CSR_PH_CORRECT_BITS | PWM_CH0_CSR_B_INV_BITS)) ==
              ((1u << PWM_CH0_CSR_DIVMODE_LSB) | PWM_CH0_CSR_PH_CORRECT_BITS | PWM_CH0_CSR_B_INV_BITS));

} // namespace brio
