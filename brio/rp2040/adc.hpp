/*
 * adc.hpp
 *
 * The RP2040's ADC (datasheet 4.9): one 12-bit successive-approximation
 * converter over five inputs - four pads, GPIO 26..29, and the
 * temperature sensor -, a conversion of 96 cycles of its own clock
 * (clk_adc, 48 MHz from the USB PLL: 500 ksps), a one-shot start and a
 * free-running mode paced by a 16.8 divider, a round-robin over the
 * inputs, an eight-entry FIFO with a threshold that is one interrupt
 * and one DMA request, an optional shift to eight bits for a byte
 * buffer, an error flag per conversion. In the two strata every brio
 * target uses:
 *
 *  Adc            the RESOURCE, a monostate (the chip has one): the
 *                 clock, the power-up, the input select, the two
 *                 starts, the divider, the round-robin, the FIFO and
 *                 its flags, the interrupt, the DMA request - and
 *                 util/analog_sampler.hpp's converter surface
 *                 (select / start / selected / input_code).
 *
 *  AnalogIn<Pin>  a pad handed to its input: GPIO 26..29 are inputs 0..3
 *                 (4.9.2.1), stated here once; the claim turns the pad's
 *                 digital input buffer off and disables its output
 *                 (4.9's note: the converter reads the bare pad).
 *
 *  AdcInput       the five inputs by name, the temperature sensor the
 *                 fifth (AINSEL 4, 4.9.5).
 *
 * THE CLOCK IS THE CONVERTER'S OWN. clk_adc is a generator of its own
 * with an aux mux and no glitchless one (clock.hpp's Clocks::adc_select);
 * the chapter asks for 48 MHz, which only the USB PLL makes from the
 * crystal, so init() starts that PLL unless it is locked already. The
 * crystal itself (12 MHz on every board here) is the other source
 * offered: a conversion then takes 96 of ITS cycles, 8 us, and the
 * result is the same reading (measured) - the choice of a program that
 * would rather not run a second PLL. The rate is clk_adc / 96 with the
 * divider at zero, else clk_adc / (1 + INT + FRAC / 256), the converter
 * ignoring a start that arrives while it converts.
 *
 * THE REFERENCE IS THE ADC_VREF PIN and nothing selectable: on the
 * boards here the 3.3 V rail through a filter, and what the pin
 * carries is the application's to state - `ref_mv(Ref, board_mv)`.
 * util/analog.hpp's arithmetic takes it as an argument. The
 * temperature sensor is a diode's Vbe, 706 mV at 27 C with -1.721 mV
 * per degree (4.9.5): `adc_temperature_centi` is that line, and it
 * moves 4 C for a 1 % change of the reference, the chapter's own
 * warning.
 *
 * THE FIFO ENTRY is twelve bits with, under FCS.ERR, bit 15 flagging a
 * conversion that failed to converge - a sample to discard (4.9.2.4's
 * caution); under FCS.SHIFT the entry is the result's top eight bits,
 * the flag where it was. A DMA channel reads the FIFO register paced
 * by the request (Dreq::adc) at threshold one; the FIFO must be
 * drained after a run is stopped, and START_MANY cleared before READY
 * is polled (4.9.2.5). Erratum RP2040-E11: the DNL peaks at four codes
 * (512, 1536, 2560, 3584), the ENOB 8.7 bits - no workaround.
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "rp2040/device.hpp"

#include "rp2040/clock.hpp"
#include "rp2040/dma_engine.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/resets.hpp"
#include "util/analog.hpp"

namespace brio {

// =============================================================================
// The vocabulary
// =============================================================================

inline constexpr uint8_t adc_bits = 12;
inline constexpr uint32_t adc_steps = 1UL << adc_bits;   ///< util/analog.hpp's full scale
inline constexpr uint16_t adc_max_count = static_cast<uint16_t>(adc_steps - 1u);
inline constexpr uint8_t adc_input_count = 5;
inline constexpr uint8_t adc_fifo_depth = 8;
inline constexpr uint32_t adc_conversion_cycles = 96;       ///< of clk_adc (4.9.2)
inline constexpr uint32_t adc_nominal_clk_hz = 48'000'000;  ///< what the chapter asks for
inline constexpr uint8_t adc_first_pin = 26;                ///< GPIO 26..29 are inputs 0..3

/// The five inputs (CS.AINSEL).
enum class AdcInput : uint8_t { ain0 = 0, ain1 = 1, ain2 = 2, ain3 = 3, temperature = 4 };

/// util/analog.hpp's vocabulary on this target: the converter's
/// reference is the ADC_VREF pin and there is no other.
enum class Ref : uint8_t { vref_pin };

/// What the pin carries is the board's: 3300 on a Pico's filtered rail.
constexpr uint16_t ref_mv(Ref, uint16_t vref_pin_mv = 3300) { return vref_pin_mv; }

/// Where the converter's clock comes from (init()).
enum class AdcClock : uint8_t {
    pll_usb,   ///< 48 MHz, the chapter's rate: 500 ksps
    crystal,   ///< the crystal as it is: a conversion of 96 of its cycles
};

/// The pacing divider (DIV, 16.8): a start every 1 + INT + FRAC / 256
/// cycles of clk_adc, a start during a conversion ignored.
struct AdcDivider {
    uint16_t integer = 0;
    uint8_t frac = 0;
    constexpr uint32_t reg() const { return (static_cast<uint32_t>(integer) << ADC_DIV_INT_LSB) | frac; }
    constexpr bool operator==(const AdcDivider& o) const { return integer == o.integer && frac == o.frac; }
};

/// The divider for `rate_hz` at `clk_hz`, solved to the nearest 1/256,
/// with INT at least 96 (4.9.2.2's "n will be >= 96": a trigger that
/// lands while a conversion runs is ignored, and at a period of
/// exactly 96 cycles every other one does - measured, 250 ksps at DIV
/// 95). Nullopt above clk_hz / 97 - the back-to-back pace is DIV 0,
/// asked for as `AdcDivider{}` - or when INT would not fit sixteen
/// bits.
constexpr std::optional<AdcDivider> adc_divider_for(uint32_t clk_hz, uint32_t rate_hz) {
    if (rate_hz == 0u || clk_hz == 0u) {
        return {};
    }
    const uint64_t period256 = (static_cast<uint64_t>(clk_hz) * 256u + rate_hz / 2u) / rate_hz;
    if (period256 < 256u * (adc_conversion_cycles + 1u)) {
        return {};
    }
    const uint64_t integer = period256 / 256u - 1u;
    if (integer > 0xFFFFu) {
        return {};
    }
    return AdcDivider{static_cast<uint16_t>(integer), static_cast<uint8_t>(period256 % 256u)};
}

/// The free-running rate a divider produces at `clk_hz`: DIV 0 is the
/// conversion's own 96 cycles; a period of 96 cycles or less is
/// stretched to the first multiple of it PAST the conversion (the
/// trigger inside a conversion is ignored, the one at its last cycle
/// too); above 96 the period is the divider's.
constexpr uint32_t adc_rate_hz(uint32_t clk_hz, AdcDivider d) {
    uint64_t period256 = 256u + static_cast<uint64_t>(d.integer) * 256u + d.frac;
    if (d.integer == 0u && d.frac == 0u) {
        period256 = 256u * adc_conversion_cycles;
    } else if (period256 <= 256u * adc_conversion_cycles) {
        const uint64_t k = (256u * adc_conversion_cycles) / period256 + 1u;
        period256 *= k;
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(clk_hz) * 256u) / period256);
}

/// The temperature in hundredths of a degree Celsius from the sensor's
/// count at `ref_mv_` (4.9.5: T = 27 - (V - 0.706) / 0.001721).
constexpr int32_t adc_temperature_centi(uint16_t counts, uint16_t ref_mv_) {
    const int64_t uv = (static_cast<int64_t>(counts) * ref_mv_ * 1000) / static_cast<int64_t>(adc_steps);
    return static_cast<int32_t>(2700 - ((uv - 706'000) * 100) / 1721);
}

/// The FIFO's configuration (FCS).
struct AdcFifoConfig {
    bool enable = true;
    bool dreq = false;       ///< DREQ_EN: the request for a DMA channel
    bool error_flag = true;  ///< ERR: bit 15 of an entry marks a failed conversion
    bool shift = false;      ///< SHIFT: eight-bit entries for a byte buffer
    uint8_t threshold = 1;   ///< THRESH: the level that raises the interrupt and the request
};

/// CS's flags.
struct AdcFlag {
    static constexpr uint32_t ready = ADC_CS_READY_BITS;
    static constexpr uint32_t error = ADC_CS_ERR_BITS;
    static constexpr uint32_t error_sticky = ADC_CS_ERR_STICKY_BITS;
};

/// The fifth input's pin: a pad is an input when it is GPIO 26..29.
constexpr bool adc_pin_valid(uint8_t pin) { return pin >= adc_first_pin && pin < adc_first_pin + 4u; }
constexpr uint8_t adc_input_of(uint8_t pin) { return static_cast<uint8_t>(pin - adc_first_pin); }

/**
 * AnalogIn<Pin>: a pad handed to its input. `claim()` turns the pad's
 * digital input buffer off and disables its output (4.9's note);
 * `release()` puts the pad back at its reset state.
 */
template <class P>
struct AnalogIn {
    static_assert(adc_pin_valid(P::number), "brio AnalogIn: only GPIO 26..29 are ADC inputs on the RP2040 (inputs 0..3)");
    using pin = P;
    static constexpr uint8_t input = adc_input_of(P::number);

    static void claim() { P::analog(); }
    static void release() { P::release(); }
};

// =============================================================================
// The resource
// =============================================================================

/**
 * Adc: the one converter, a monostate.
 *
 *   using Vin = brio::AnalogIn<brio::Pin<26>>;   // input 0
 *   Adc::init(clock);                              // the USB PLL at 48 MHz, clk_adc on it
 *   Vin::claim();
 *   Adc::select(Vin{});
 *   const auto counts = Adc::read();
 */
struct Adc {
    Adc() = delete;

    static constexpr uint8_t inputs = adc_input_count;
    static constexpr uint8_t temperature_input = static_cast<uint8_t>(AdcInput::temperature);
    static constexpr IRQn_Type irq() { return ADC_IRQ_FIFO_IRQn; }
    static constexpr Dreq dreq = Dreq::adc;
    static constexpr uint32_t reset_bit = ResetBlock::adc;

    static ADC_Type& regs() { return *ADC; }
    static volatile void* fifo_address() { return &regs().FIFO; }

    // ---- lifecycle ----------------------------------------------------------

    /// Bring the converter up: its clock (the USB PLL started at 48 MHz
    /// from the crystal unless locked already, or the crystal as it is),
    /// the block out of reset, EN, READY waited for. `clock` is the
    /// app's Clock tag, the one truth of the crystal's rate. False when
    /// the PLL did not lock, the block did not come up or READY never
    /// rose; the FIFO is off, the sensor's bias off, the divider zero.
    template <typename Clock>
    static bool init(Clock clock, AdcClock src = AdcClock::pll_usb) {
        (void)clock;
        Nvic::disable(irq());
        if (src == AdcClock::pll_usb) {
            if (!PllUsb::locked()) {
                const PllConfig cfg = pll_config_for(Clock::xtal_hz, adc_nominal_clk_hz);
                if (!cfg.valid() || !PllUsb::init(cfg)) {
                    return false;
                }
            }
            Clocks::adc_select(AdcAux::pll_usb);
            clk_hz_ = adc_nominal_clk_hz;
        } else {
            Clocks::adc_select(AdcAux::xosc);
            clk_hz_ = Clock::xtal_hz;
        }
        if (!Resets::cycle(reset_bit)) {
            return false;
        }
        regs().CS = ADC_CS_EN_BITS;
        selected_ = 0;
        return wait_ready();
    }

    static uint32_t clock_hz() { return clk_hz_; }
    static void enable(bool on) {
        if (on) { hw_set(regs().CS, ADC_CS_EN_BITS); } else { hw_clear(regs().CS, ADC_CS_EN_BITS); }
    }
    static bool enabled() { return (regs().CS & ADC_CS_EN_BITS) != 0u; }
    /// CS.READY: no conversion in progress and the analogue side up.
    static bool ready() { return (regs().CS & ADC_CS_READY_BITS) != 0u; }
    static bool wait_ready(uint32_t spins = 1'000'000u) {
        while (!ready() && spins-- != 0u) {
        }
        return ready();
    }
    /// The temperature sensor's bias (CS.TS_EN), about 40 uA on ADC_AVDD.
    static void temperature_sensor(bool on) {
        if (on) { hw_set(regs().CS, ADC_CS_TS_EN_BITS); } else { hw_clear(regs().CS, ADC_CS_TS_EN_BITS); }
    }
    static bool temperature_sensor() { return (regs().CS & ADC_CS_TS_EN_BITS) != 0u; }

    static void release() {
        Nvic::disable(irq());
        interrupt(false);
        start_many(false);
        regs().CS = 0;
        Resets::hold(reset_bit);
    }

    // ---- util/analog_sampler.hpp's converter surface --------------------------

    template <class P>
    static constexpr uint8_t input_code(AnalogIn<P>) { return AnalogIn<P>::input; }
    static constexpr uint8_t input_code(AdcInput in) { return static_cast<uint8_t>(in); }
    static constexpr uint8_t input_code(uint8_t in) { return in; }

    /// CS.AINSEL, no settling time (4.9.2.1).
    template <class P>
    static void select(AnalogIn<P>) { select_input(AnalogIn<P>::input); }
    static void select(AdcInput in) { select_input(static_cast<uint8_t>(in)); }
    static void select_input(uint8_t in) {
        if (in >= inputs) {
            return;
        }
        hw_write_masked(regs().CS, static_cast<uint32_t>(in) << ADC_CS_AINSEL_LSB, ADC_CS_AINSEL_BITS);
        selected_ = in;
    }
    /// The input the converter will take next (AINSEL as the register
    /// holds it: the round-robin moves it after every conversion).
    static uint8_t selected() { return static_cast<uint8_t>((regs().CS & ADC_CS_AINSEL_BITS) >> ADC_CS_AINSEL_LSB); }
    /// The input the last select() named - what a one-shot result is.
    static uint8_t selected_input() { return selected_; }

    /// START_ONCE: one conversion, now. Void for the sampler.
    static void start() { hw_set(regs().CS, ADC_CS_START_ONCE_BITS); }
    /// The most recent result (RESULT), whatever started it.
    static uint16_t result() { return static_cast<uint16_t>(regs().RESULT & ADC_RESULT_BITS); }
    /// One conversion of the selected input, waited for: nullopt when
    /// READY never rose or the conversion errored (CS.ERR).
    static std::optional<uint16_t> read(uint32_t spins = 100'000u) {
        if (!wait_ready(spins)) {
            return {};
        }
        start();
        while (ready() && spins-- != 0u) {   // the flag drops as the conversion begins
        }
        if (!wait_ready(spins)) {
            return {};
        }
        if (error()) {
            return {};
        }
        return result();
    }

    // ---- free running -------------------------------------------------------------

    /// START_MANY: conversions at the divider's pace until cleared.
    static void start_many(bool on) {
        if (on) { hw_set(regs().CS, ADC_CS_START_MANY_BITS); } else { hw_clear(regs().CS, ADC_CS_START_MANY_BITS); }
    }
    static bool running() { return (regs().CS & ADC_CS_START_MANY_BITS) != 0u; }
    /// The pacing divider; writing it restarts the pace.
    static void divider(AdcDivider d) { regs().DIV = d.reg(); }
    static AdcDivider divider() {
        const uint32_t v = regs().DIV;
        return AdcDivider{static_cast<uint16_t>((v & ADC_DIV_INT_BITS) >> ADC_DIV_INT_LSB),
                          static_cast<uint8_t>(v & ADC_DIV_FRAC_BITS)};
    }
    /// The free-running rate at this converter's clock.
    static uint32_t rate_hz() { return adc_rate_hz(clk_hz_, divider()); }
    /// CS.RROBIN: the inputs cycled in free-running mode, one bit per
    /// input; 0 disables. AINSEL names the first.
    static void round_robin(uint8_t mask) {
        hw_write_masked(regs().CS, static_cast<uint32_t>(mask & 0x1Fu) << ADC_CS_RROBIN_LSB, ADC_CS_RROBIN_BITS);
    }
    static uint8_t round_robin() { return static_cast<uint8_t>((regs().CS & ADC_CS_RROBIN_BITS) >> ADC_CS_RROBIN_LSB); }

    /// Stop a free run the chapter's way (4.9.2.5): START_MANY cleared,
    /// READY polled for the last conversion, the FIFO drained.
    static bool stop_many(uint32_t spins = 100'000u) {
        start_many(false);
        const bool ok = wait_ready(spins);
        drain();
        return ok;
    }

    // ---- the errors ----------------------------------------------------------------

    /// CS.ERR: the most recent conversion failed to converge.
    static bool error() { return (regs().CS & ADC_CS_ERR_BITS) != 0u; }
    /// CS.ERR_STICKY: some conversion did, since the last clear. The
    /// clear is a PLAIN write of the register with the flag's one: the
    /// atomic set alias does not clear a write-one-to-clear bit of this
    /// block (measured on FCS.OVER).
    static bool error_seen() { return (regs().CS & ADC_CS_ERR_STICKY_BITS) != 0u; }
    static void clear_error_seen() {
        regs().CS = (regs().CS & ~(ADC_CS_START_ONCE_BITS | ADC_CS_ERR_STICKY_BITS)) | ADC_CS_ERR_STICKY_BITS;
    }

    // ---- the FIFO ------------------------------------------------------------------

    static void fifo(const AdcFifoConfig& c) {
        regs().FCS = (c.enable ? ADC_FCS_EN_BITS : 0u) | (c.dreq ? ADC_FCS_DREQ_EN_BITS : 0u) |
                     (c.error_flag ? ADC_FCS_ERR_BITS : 0u) | (c.shift ? ADC_FCS_SHIFT_BITS : 0u) |
                     ((static_cast<uint32_t>(c.threshold) << ADC_FCS_THRESH_LSB) & ADC_FCS_THRESH_BITS) |
                     ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS;   // the two sticky flags cleared
    }
    static bool fifo_enabled() { return (regs().FCS & ADC_FCS_EN_BITS) != 0u; }
    static uint8_t fifo_level() { return static_cast<uint8_t>((regs().FCS & ADC_FCS_LEVEL_BITS) >> ADC_FCS_LEVEL_LSB); }
    static bool fifo_empty() { return (regs().FCS & ADC_FCS_EMPTY_BITS) != 0u; }
    static bool fifo_full() { return (regs().FCS & ADC_FCS_FULL_BITS) != 0u; }
    /// A conversion completed with the FIFO full and was lost; sticky.
    static bool fifo_overflowed() { return (regs().FCS & ADC_FCS_OVER_BITS) != 0u; }
    static bool fifo_underflowed() { return (regs().FCS & ADC_FCS_UNDER_BITS) != 0u; }
    /// A plain write with the two ones: the set alias would not clear
    /// them (measured).
    static void clear_fifo_flags() {
        regs().FCS = (regs().FCS & ~(ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS)) | ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS;
    }
    /// One entry popped: the result in bits 11:0 (7:0 under SHIFT), the
    /// error flag in bit 15.
    static uint16_t pop() { return static_cast<uint16_t>(regs().FIFO & 0xFFFFu); }
    static constexpr uint16_t entry_error = ADC_FIFO_ERR_BITS;
    static constexpr uint16_t entry_value(uint16_t e) { return static_cast<uint16_t>(e & ADC_FIFO_VAL_BITS); }
    static constexpr bool entry_failed(uint16_t e) { return (e & ADC_FIFO_ERR_BITS) != 0u; }
    static void drain() {
        while (!fifo_empty()) {
            (void)regs().FIFO;
        }
    }

    // ---- the interrupt ---------------------------------------------------------------

    /// INTE.FIFO: the level at or above the threshold; it clears by
    /// draining below it.
    static void interrupt(bool on) {
        if (on) { hw_set(regs().INTE, ADC_INTE_FIFO_BITS); } else { hw_clear(regs().INTE, ADC_INTE_FIFO_BITS); }
    }
    static bool raw_pending() { return (regs().INTR & ADC_INTR_FIFO_BITS) != 0u; }
    static bool pending() { return (regs().INTS & ADC_INTS_FIFO_BITS) != 0u; }
    static void force(bool on) {
        if (on) { hw_set(regs().INTF, ADC_INTF_FIFO_BITS); } else { hw_clear(regs().INTF, ADC_INTF_FIFO_BITS); }
    }
    /// The ISR body: whether the FIFO is at its threshold - the handler
    /// pops until it is not.
    [[gnu::always_inline]] static bool isr() { return pending(); }

private:
    static inline uint32_t clk_hz_ = 0;
    static inline uint8_t selected_ = 0;
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

// 48 MHz: DIV 0 is 500 ksps; DIV 47999 is 1 ksps (4.9.2.2's example);
// 100 ksps wants 479; 500 ksps through the divider is refused (DIV 95
// collides: 250 ksps), so is a rate the sixteen bits cannot reach; a
// short divider stretches to its first multiple past the conversion.
static_assert(adc_rate_hz(48'000'000, {}) == 500'000u);
static_assert(adc_divider_for(48'000'000, 1000)->integer == 47999u && adc_divider_for(48'000'000, 1000)->frac == 0u);
static_assert(adc_divider_for(48'000'000, 100'000)->integer == 479u);
static_assert(adc_rate_hz(48'000'000, {479, 0}) == 100'000u);
static_assert(!adc_divider_for(48'000'000, 500'000).has_value());
static_assert(adc_divider_for(48'000'000, 480'000)->integer == 99u);
static_assert(!adc_divider_for(48'000'000, 700).has_value());
static_assert(adc_rate_hz(48'000'000, {95, 0}) == 250'000u);   // the collision
static_assert(adc_rate_hz(48'000'000, {99, 0}) == 480'000u);
static_assert(adc_rate_hz(48'000'000, {47, 0}) == 333'333u);   // 48 x 3 = 144 cycles
static_assert(adc_divider_for(48'000'000, 44'100)->frac == 111u);
static_assert(adc_rate_hz(12'000'000, {}) == 125'000u);
// 4.9.5's example: 891 counts at 3.3 V is 20.1 C (20.12 in hundredths).
static_assert(adc_temperature_centi(891, 3300) == 2012);
static_assert(adc_temperature_centi(876, 3300) > 2500 && adc_temperature_centi(876, 3300) < 2800);
static_assert(adc_pin_valid(26) && adc_pin_valid(29) && !adc_pin_valid(25) && !adc_pin_valid(30));
static_assert(adc_input_of(28) == 2u);
static_assert(ref_mv(Ref::vref_pin) == 3300u && ref_mv(Ref::vref_pin, 3000) == 3000u);
static_assert(adc_mv(2048, adc_steps, 3300) == 1650u);

} // namespace brio
