/*
 * adc.hpp
 *
 * The RP2350's ADC (datasheet 12.4): one 12-bit successive-approximation
 * converter over FIVE OR NINE inputs - four pads or eight, by package,
 * plus the temperature sensor - a conversion of 96 cycles of its own
 * clock (clk_adc, 48 MHz from the USB PLL: 500 ksps), a one-shot start
 * and a free run paced by a 16.8 divider, a round-robin over the inputs,
 * an eight-entry FIFO with a threshold that is one interrupt and one DMA
 * request, an optional shift to eight bits for a byte buffer, an error
 * flag per conversion.
 *
 *  Adc            the RESOURCE, a monostate (the chip has one): the
 *                 clock, the power-up, the input select, the two starts,
 *                 the divider, the round-robin, the FIFO and its flags,
 *                 the interrupt, the DMA request - and
 *                 util/analog_sampler.hpp's converter surface (select /
 *                 start / selected / input_code).
 *
 *  AnalogIn<Pin>  a pad handed to its input. The claim turns the pad's
 *                 digital input buffer OFF and its output driver off
 *                 (12.4's own instruction: IE low and OD high on an ADC
 *                 pad) and DROPS THE ISOLATION LATCH, which on this chip
 *                 is what makes the pad's pull-up or pull-down reach the
 *                 pad at all (9.7: the latch holds the output enable, the
 *                 output level AND the pull enables; the analogue tap and
 *                 the digital input are not isolated).
 *
 *  AdcInput       the inputs by name, the temperature sensor LAST - and
 *                 which number that is depends on the package.
 *
 * THE PACKAGE DECIDES THE INPUT MAP, and this is the first difference
 * from the RP2040. The QFN-60 brings out four inputs on GPIO 26..29 with
 * the sensor on AINSEL 4, exactly as the RP2040 did; THE QFN-80 BRINGS
 * OUT EIGHT, on GPIO 40..47, with the sensor on AINSEL 8 (12.4.2.1,
 * tables 1118 and 1119). AINSEL is therefore four bits wide here against
 * three, and RROBIN nine against five - and the register description of
 * AINSEL says the field "is corrected for the package option so only ADC
 * channels which are bonded are available, and in the correct order", so
 * the numbering above is the whole story on each package and there is no
 * hole in it. The stratum's reserve (device.hpp) states which package
 * this image is for; everything below reads it from there and never
 * spells 4 or 8 again. Since the die is one, a program built for the
 * larger package and run on the smaller one would put the sensor's
 * number on nothing, so init() READS SYSINFO.PACKAGE_SEL and refuses when
 * the silicon disagrees with the build.
 *
 * THE OTHER DIFFERENCE IS SILENT: erratum RP2040-E11's differential
 * nonlinearity spikes at codes 0x200, 0x600, 0xa00 and 0xe00 are GONE,
 * "improving the ADC's precision by around 0.5 ENOB" (12.4.1) over the
 * other chip's 8.7 bits - 9.2 by 12.4's feature list, 9 minimum and 9.5
 * typical by the electrical table. Appendix E has no ADC erratum at all
 * - and the erratum numbered E11 in THIS chip's sheet is an XIP one,
 * which is why an erratum is never cited here by its bare number.
 *
 * THE REFERENCE IS THE ADC SUPPLY PIN, and that is the third difference:
 * this chip has NO ADC_VREF pin. ADC_AVDD supplies the converter and is
 * its full scale (6.1.5, and the electrical table's input voltage range
 * 0 .. ADC_AVDD), so `Ref` has one enumerator, `avdd_pin`, and what that
 * pin carries is the application's to state - `ref_mv(Ref, board_mv)`.
 * The maximum voltage an ADC pad may see is IOVDD's and not
 * ADC_AVDD's: above IOVDD the ESD diodes leak (12.4, 14.9).
 *
 * THE CLOCK IS THE CONVERTER'S OWN. clk_adc is a generator with an aux
 * mux and no glitchless one (clock.hpp's Clocks::adc_select); the chapter
 * asks for 48 MHz, which the USB PLL makes from the crystal, so init()
 * starts that PLL unless it is locked ON THE RIGHT RATIO already - which
 * matters here more than it did on the other chip, because a processor
 * reset leaves the clock tree as the previous image built it. The crystal
 * is the other source offered: a conversion then takes 96 of ITS cycles.
 * The rate is clk_adc / 96 with the divider at zero, else clk_adc /
 * (1 + INT + FRAC / 256), the converter ignoring a start that arrives
 * while it converts.
 *
 * THE FIFO ENTRY is twelve bits with, under FCS.ERR, bit 15 flagging a
 * conversion that failed to converge - a sample to discard (12.4.3.4's
 * caution); under FCS.SHIFT the entry is the result's top eight bits, the
 * flag where it was. A DMA channel reads the FIFO register paced by the
 * request (Dreq::adc) at threshold one; after a run is stopped, READY is
 * polled for the last conversion and the FIFO drained (12.4.3.5).
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "rp2350/device.hpp"

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/dma_engine.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/resets.hpp"
#include "rp2350/sysinfo.hpp"
#include "util/analog.hpp"

namespace brio {

// =============================================================================
// The vocabulary
// =============================================================================

inline constexpr uint8_t adc_bits = 12;
inline constexpr uint32_t adc_steps = 1UL << adc_bits;   ///< util/analog.hpp's full scale
inline constexpr uint16_t adc_max_count = static_cast<uint16_t>(adc_steps - 1u);
inline constexpr uint8_t adc_fifo_depth = 8;
inline constexpr uint32_t adc_conversion_cycles = 96;       ///< of clk_adc (12.4.3)
inline constexpr uint32_t adc_nominal_clk_hz = 48'000'000;  ///< what the chapter asks for

/// The package's facts, from the stratum's reserve and nowhere else
/// (device.hpp): four pads and the sensor on AINSEL 4 in the QFN-60,
/// eight and the sensor on AINSEL 8 in the QFN-80 (12.4.2.1).
inline constexpr uint8_t adc_pad_inputs = package_adc_inputs(package);
inline constexpr uint8_t adc_first_pin = package_adc_base_pin(package);
inline constexpr uint8_t adc_temperature_code = adc_pad_inputs;
inline constexpr uint8_t adc_input_count = static_cast<uint8_t>(adc_pad_inputs + 1u);
/// One RROBIN bit per channel this package has, the sensor's included.
inline constexpr uint16_t adc_round_robin_mask =
    static_cast<uint16_t>((1u << adc_input_count) - 1u);

/**
 * The inputs by name (CS.AINSEL). The eight pad enumerators are the
 * DIE'S - every one of them exists in the register field on both
 * packages - and `temperature` is a TAG whose number is the package's,
 * which is why the sensor is never spelled as a digit here. An input the
 * package has not bonded is refused where it is named: by `AnalogIn`'s
 * pad at compile time, by `select()` at run time.
 */
enum class AdcInput : uint8_t {
    ain0 = 0, ain1 = 1, ain2 = 2, ain3 = 3,
    ain4 = 4, ain5 = 5, ain6 = 6, ain7 = 7,
    temperature = 0xFF,   ///< the tag; its code is adc_temperature_code
};

/// The AINSEL code of an input on THIS package.
constexpr uint8_t adc_code_of(AdcInput in) {
    return in == AdcInput::temperature ? adc_temperature_code : static_cast<uint8_t>(in);
}
/// Whether this package bonds the pad behind `in` (the sensor always
/// counts: it is on the die, not on a pin). The bound is the number of
/// PADS and not the number of channels, because in the smaller package
/// `ain4`'s number is the sensor's and naming it would select the diode.
constexpr bool adc_input_bonded(AdcInput in) {
    return in == AdcInput::temperature || static_cast<uint8_t>(in) < adc_pad_inputs;
}

/// util/analog.hpp's vocabulary on this target. There is no ADC_VREF pin
/// on this chip: the converter's full scale is its own supply (6.1.5).
enum class Ref : uint8_t { avdd_pin };

/// What the ADC_AVDD pin carries is the board's: 3300 on a 3.3 V rail.
constexpr uint16_t ref_mv(Ref, uint16_t avdd_mv = 3300) { return avdd_mv; }

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
    constexpr uint32_t reg() const {
        return (static_cast<uint32_t>(integer) << ADC_DIV_INT_LSB) | frac;
    }
    constexpr bool operator==(const AdcDivider& o) const {
        return integer == o.integer && frac == o.frac;
    }
};

/// The divider for `rate_hz` at `clk_hz`, solved to the nearest 1/256,
/// with INT at least 96 (12.4.3.2's "generally n will be >= 96": a
/// trigger that lands while a conversion runs is ignored). Nullopt above
/// clk_hz / 97 - the back-to-back pace is DIV 0, asked for as
/// `AdcDivider{}` - or when INT would not fit sixteen bits.
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
/// conversion's own 96 cycles; a period of 96 cycles or less is stretched
/// to the first multiple of it PAST the conversion, since the trigger
/// inside a conversion is ignored. Whether the trigger at the
/// conversion's LAST cycle is ignored too - which is what makes DIV 95 a
/// half rate rather than a full one - is the one boundary the chapter
/// does not state.
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
/// count at `ref_mv_` (12.4.6: T = 27 - (V - 0.706) / 0.001721).
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

    /// The field is four bits, the FIFO eight entries deep: a threshold
    /// past the depth is a request that can never be raised, so it is
    /// refused rather than written.
    constexpr bool valid() const { return threshold <= adc_fifo_depth; }
};

/// CS's flags, as masks for a program that wants the register whole.
struct AdcFlag {
    static constexpr uint32_t ready = ADC_CS_READY_BITS;
    static constexpr uint32_t error = ADC_CS_ERR_BITS;
    static constexpr uint32_t error_sticky = ADC_CS_ERR_STICKY_BITS;
};

/// Whether a pad is an ADC input in THIS package (tables 1118 / 1119).
constexpr bool adc_pin_valid(uint8_t pin) {
    return pin >= adc_first_pin && pin < adc_first_pin + adc_pad_inputs;
}
constexpr uint8_t adc_input_of(uint8_t pin) { return static_cast<uint8_t>(pin - adc_first_pin); }

/**
 * AnalogIn<Pin>: a pad handed to its input. `claim()` turns the pad's
 * digital input buffer off, disables its output driver and drops the
 * isolation latch (pin.hpp's `analog`), optionally with a pull;
 * `release()` puts the pad back at its reset state - isolated, input
 * disabled, pulled down.
 */
template <class P>
struct AnalogIn {
    static_assert(adc_pin_valid(P::number),
                  "brio AnalogIn: this pad is not an ADC input in the package this image is "
                  "built for - the QFN-60 converts GPIO26..GPIO29, the QFN-80 GPIO40..GPIO47 "
                  "(datasheet 12.4.2.1)");
    using pin = P;
    static constexpr uint8_t input = adc_input_of(P::number);

    static bool claim(PinPull pull = PinPull::none) { return P::analog(pull); }
    static bool release() { return P::release(); }
};

// =============================================================================
// The resource
// =============================================================================

/**
 * Adc: the one converter, a monostate.
 *
 *   using Vin = brio::AnalogIn<brio::Pin<40>>;   // input 0 in the QFN-80
 *   Adc::init(clock);                            // the USB PLL at 48 MHz, clk_adc on it
 *   Vin::claim();
 *   Adc::select(Vin{});
 *   const auto counts = Adc::read();
 */
struct Adc {
    Adc() = delete;

    /// The channels this package has, the sensor included.
    static constexpr uint8_t inputs = adc_input_count;
    /// The sensor's AINSEL code on this package - 4 or 8, never spelled.
    static constexpr uint8_t temperature_input = adc_temperature_code;
    static constexpr IRQn_Type irq() { return ADC_IRQ_FIFO_IRQn; }
    static constexpr Dreq dreq = Dreq::adc;
    static constexpr uint32_t reset_bit = ResetBlock::adc;

    static ADC_Type& regs() { return *ADC; }
    static volatile void* fifo_address() { return &regs().FIFO; }

    // ---- lifecycle ----------------------------------------------------------

    /// Whether the silicon is in the package this image was built for.
    /// The input map and the sensor's channel number are compile-time
    /// facts here, so a mismatch is not a thing a verb can work around.
    static bool package_matches() { return ChipId::package_sel() == package; }

    /// Bring the converter up: the package checked, then its clock (the
    /// USB PLL brought to 48 MHz from the crystal unless it is locked on
    /// that ratio already, or the crystal as it is), the block cycled
    /// through reset, EN, READY waited for. `clock` is the app's Clock
    /// tag, the one truth of the crystal's rate. False when the silicon
    /// is the other package, the PLL did not lock, the block did not come
    /// up or READY never rose; the FIFO is off, the sensor's bias off,
    /// the divider zero.
    ///
    /// THE CLOCK COMES BEFORE THE RESET, and the reset is a CYCLE: a
    /// processor reset on this chip leaves every peripheral as the
    /// previous image left it, so a driver that wants the reset state has
    /// to make it.
    template <typename Clock>
    static bool init(Clock clock, AdcClock src = AdcClock::pll_usb) {
        (void)clock;
        Irq::disable(irq());
        if (!package_matches()) {
            return false;
        }
        if (src == AdcClock::pll_usb) {
            const PllConfig cfg = pll_config_for(Clock::xtal_hz, adc_nominal_clk_hz);
            if (!cfg.valid()) {
                return false;
            }
            if (!PllUsb::locked() || !(PllUsb::config() == cfg)) {
                if (!PllUsb::init(cfg)) {
                    return false;
                }
            }
            if (!Clocks::adc_select(AdcAux::pll_usb)) {
                return false;
            }
            clk_hz_ = adc_nominal_clk_hz;
        } else {
            if (!Clocks::adc_select(AdcAux::xosc)) {
                return false;
            }
            clk_hz_ = Clock::xtal_hz;
        }
        if (!Resets::cycle(reset_bit)) {
            return false;
        }
        regs().CS = ADC_CS_EN_BITS;
        selected_ = 0;
        return wait_ready();
    }

    /// What init() put on clk_adc, in hertz - the rate every conversion
    /// is counted in.
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

    /// The block back in reset, its interrupt off, its line disabled.
    static void release() {
        Irq::disable(irq());
        interrupt(false);
        start_many(false);
        regs().CS = 0;
        Resets::hold(reset_bit);
    }

    // ---- util/analog_sampler.hpp's converter surface --------------------------

    template <class P>
    static constexpr uint8_t input_code(AnalogIn<P>) { return AnalogIn<P>::input; }
    static constexpr uint8_t input_code(AdcInput in) { return adc_code_of(in); }
    static constexpr uint8_t input_code(uint8_t in) { return in; }

    /// CS.AINSEL, no settling time (12.4.3.1). A channel this package has
    /// not got is not written.
    template <class P>
    static void select(AnalogIn<P>) { select_input(AnalogIn<P>::input); }
    /// A pad tag this package has not bonded is not written - and in the
    /// smaller package that includes `ain4`, whose number is the
    /// sensor's there.
    static void select(AdcInput in) {
        if (!adc_input_bonded(in)) {
            return;
        }
        select_input(adc_code_of(in));
    }
    static void select_input(uint8_t in) {
        if (in >= inputs) {
            return;
        }
        hw_write_masked(regs().CS, static_cast<uint32_t>(in) << ADC_CS_AINSEL_LSB, ADC_CS_AINSEL_BITS);
        selected_ = in;
    }
    /// The input the converter will take next (AINSEL as the register
    /// holds it: the round-robin moves it after every conversion).
    static uint8_t selected() {
        return static_cast<uint8_t>((regs().CS & ADC_CS_AINSEL_BITS) >> ADC_CS_AINSEL_LSB);
    }
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
    /// The pacing divider; writing either field restarts the pace.
    static void divider(AdcDivider d) { regs().DIV = d.reg(); }
    static AdcDivider divider() {
        const uint32_t v = regs().DIV;
        return AdcDivider{static_cast<uint16_t>((v & ADC_DIV_INT_BITS) >> ADC_DIV_INT_LSB),
                          static_cast<uint8_t>(v & ADC_DIV_FRAC_BITS)};
    }
    /// The free-running rate at this converter's clock.
    static uint32_t rate_hz() { return adc_rate_hz(clk_hz_, divider()); }

    /// CS.RROBIN: the inputs cycled in free-running mode, one bit per
    /// channel, AINSEL naming the first; 0 disables. NINE bits here
    /// against the RP2040's five. False, and nothing written, for a mask
    /// naming a channel this package has not got.
    static bool round_robin(uint16_t mask) {
        if ((mask & ~adc_round_robin_mask) != 0u) {
            return false;
        }
        hw_write_masked(regs().CS, static_cast<uint32_t>(mask) << ADC_CS_RROBIN_LSB, ADC_CS_RROBIN_BITS);
        return true;
    }
    /// The same, judged at COMPILE time - what a program with a constant
    /// set of inputs uses, so that a channel the package has not got is
    /// an error and not a false.
    template <uint16_t mask>
    static void round_robin() {
        static_assert((mask & ~adc_round_robin_mask) == 0u,
                      "brio Adc::round_robin: that mask names a channel this package has not got "
                      "- five in the QFN-60, nine in the QFN-80, the sensor's included");
        hw_write_masked(regs().CS, static_cast<uint32_t>(mask) << ADC_CS_RROBIN_LSB, ADC_CS_RROBIN_BITS);
    }
    static uint16_t round_robin() {
        return static_cast<uint16_t>((regs().CS & ADC_CS_RROBIN_BITS) >> ADC_CS_RROBIN_LSB);
    }

    /// Stop a free run the chapter's way (12.4.3.5): START_MANY cleared,
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
    /// CS.ERR_STICKY: some conversion did, since the last clear.
    static bool error_seen() { return (regs().CS & ADC_CS_ERR_STICKY_BITS) != 0u; }
    /// The clear is a PLAIN write of the register with the flag's one and
    /// START_ONCE masked out. A plain write is right whether or not the
    /// atomic set alias reaches a write-one-to-clear bit of this block -
    /// on the RP2040 it did not - so nothing here depends on the answer.
    static void clear_error_seen() {
        regs().CS = (regs().CS & ~(ADC_CS_START_ONCE_BITS | ADC_CS_ERR_STICKY_BITS)) |
                    ADC_CS_ERR_STICKY_BITS;
    }

    // ---- the FIFO ------------------------------------------------------------------

    /// The whole register written at once, the two sticky flags cleared
    /// with it. False, and nothing written, for a threshold past the
    /// FIFO's depth.
    static bool fifo(const AdcFifoConfig& c) {
        if (!c.valid()) {
            return false;
        }
        regs().FCS = (c.enable ? ADC_FCS_EN_BITS : 0u) | (c.dreq ? ADC_FCS_DREQ_EN_BITS : 0u) |
                     (c.error_flag ? ADC_FCS_ERR_BITS : 0u) | (c.shift ? ADC_FCS_SHIFT_BITS : 0u) |
                     ((static_cast<uint32_t>(c.threshold) << ADC_FCS_THRESH_LSB) & ADC_FCS_THRESH_BITS) |
                     ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS;
        return true;
    }
    static bool fifo_enabled() { return (regs().FCS & ADC_FCS_EN_BITS) != 0u; }
    static uint8_t fifo_level() {
        return static_cast<uint8_t>((regs().FCS & ADC_FCS_LEVEL_BITS) >> ADC_FCS_LEVEL_LSB);
    }
    static uint8_t fifo_threshold() {
        return static_cast<uint8_t>((regs().FCS & ADC_FCS_THRESH_BITS) >> ADC_FCS_THRESH_LSB);
    }
    static bool fifo_empty() { return (regs().FCS & ADC_FCS_EMPTY_BITS) != 0u; }
    static bool fifo_full() { return (regs().FCS & ADC_FCS_FULL_BITS) != 0u; }
    /// A conversion completed with the FIFO full and was lost; sticky.
    static bool fifo_overflowed() { return (regs().FCS & ADC_FCS_OVER_BITS) != 0u; }
    static bool fifo_underflowed() { return (regs().FCS & ADC_FCS_UNDER_BITS) != 0u; }
    /// A plain write with the two ones, for clear_error_seen()'s reason.
    static void clear_fifo_flags() {
        regs().FCS = (regs().FCS & ~(ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS)) | ADC_FCS_OVER_BITS |
                     ADC_FCS_UNDER_BITS;
    }
    /// One entry popped: the result in bits 11:0 (7:0 under SHIFT), the
    /// error flag in bit 15 wherever the result was shifted to.
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
    /// draining below it. One line, `isr_adc_fifo` on both architectures.
    static void interrupt(bool on) {
        if (on) { hw_set(regs().INTE, ADC_INTE_FIFO_BITS); } else { hw_clear(regs().INTE, ADC_INTE_FIFO_BITS); }
    }
    static bool interrupt() { return (regs().INTE & ADC_INTE_FIFO_BITS) != 0u; }
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

// 48 MHz: DIV 0 is 500 ksps; DIV 47999 is 1 ksps (12.4.3.2's example);
// 100 ksps wants 479; 500 ksps through the divider is refused (DIV 95
// would collide with the conversion's own 96 cycles), so is a rate the
// sixteen bits cannot reach; a short divider stretches to its first
// multiple past the conversion.
static_assert(adc_rate_hz(48'000'000, {}) == 500'000u);
static_assert(adc_divider_for(48'000'000, 1000)->integer == 47999u &&
              adc_divider_for(48'000'000, 1000)->frac == 0u);
static_assert(adc_divider_for(48'000'000, 100'000)->integer == 479u);
static_assert(adc_rate_hz(48'000'000, {479, 0}) == 100'000u);
static_assert(!adc_divider_for(48'000'000, 500'000).has_value());
static_assert(adc_divider_for(48'000'000, 480'000)->integer == 99u);
static_assert(!adc_divider_for(48'000'000, 700).has_value());
static_assert(adc_rate_hz(48'000'000, {95, 0}) == 250'000u);   // the boundary
static_assert(adc_rate_hz(48'000'000, {99, 0}) == 480'000u);
static_assert(adc_rate_hz(48'000'000, {47, 0}) == 333'333u);   // 48 x 3 = 144 cycles
static_assert(adc_divider_for(48'000'000, 44'100)->frac == 111u);
static_assert(adc_rate_hz(12'000'000, {}) == 125'000u);
// 12.4.6's example: 891 counts at 3.3 V is 20.1 C (20.12 in hundredths).
static_assert(adc_temperature_centi(891, 3300) == 2012);
static_assert(adc_temperature_centi(876, 3300) > 2500 && adc_temperature_centi(876, 3300) < 2800);
// The package's map, both ways, from the reserve alone.
static_assert(adc_pad_inputs == (package == Package::qfn60 ? 4u : 8u));
static_assert(adc_first_pin == (package == Package::qfn60 ? 26u : 40u));
static_assert(adc_input_count == adc_pad_inputs + 1u);
static_assert(adc_temperature_code == adc_pad_inputs);
static_assert(adc_round_robin_mask == (package == Package::qfn60 ? 0x1Fu : 0x1FFu));
static_assert(adc_code_of(AdcInput::ain0) == 0u);
static_assert(adc_code_of(AdcInput::temperature) == adc_temperature_code);
static_assert(adc_input_bonded(AdcInput::ain3) && adc_input_bonded(AdcInput::temperature));
static_assert(adc_input_bonded(AdcInput::ain7) == (package == Package::qfn80));
static_assert(adc_pin_valid(adc_first_pin) && adc_pin_valid(adc_first_pin + adc_pad_inputs - 1u));
static_assert(!adc_pin_valid(adc_first_pin - 1u) && !adc_pin_valid(adc_first_pin + adc_pad_inputs));
static_assert(adc_input_of(adc_first_pin + 2u) == 2u);
// Every channel of the die exists in the register field, on both
// packages: AINSEL is four bits and RROBIN nine.
static_assert((ADC_CS_AINSEL_BITS >> ADC_CS_AINSEL_LSB) == 0xFu);
static_assert((ADC_CS_RROBIN_BITS >> ADC_CS_RROBIN_LSB) == 0x1FFu);
static_assert(ref_mv(Ref::avdd_pin) == 3300u && ref_mv(Ref::avdd_pin, 1800) == 1800u);
static_assert(adc_mv(2048, adc_steps, 3300) == 1650u);
static_assert(AdcFifoConfig{.threshold = adc_fifo_depth}.valid());
static_assert(!AdcFifoConfig{.threshold = adc_fifo_depth + 1u}.valid());

} // namespace brio
