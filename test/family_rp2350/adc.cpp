// ADC family smoke TU: every verb of the converter and of AnalogIn, with
// the PACKAGE as the axis - the pad map, the sensor's channel number and
// the round-robin's width are all read from the stratum's reserve, so
// this file compiles for the QFN-60's four inputs as it does for the
// QFN-80's eight, and the pads above the fourth are instantiated only
// where the package brings them out.
#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "rp2350/adc.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/platform.hpp"
#include "util/analog_sampler.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;

static_assert(adc_bits == 12u);
static_assert(adc_steps == 4096u);
static_assert(adc_max_count == 4095u);
static_assert(adc_fifo_depth == 8u);
static_assert(adc_conversion_cycles == 96u);
static_assert(adc_nominal_clk_hz == 48'000'000u);

// The package's map and nothing spelled twice.
static_assert(adc_pad_inputs == package_adc_inputs(package));
static_assert(adc_first_pin == package_adc_base_pin(package));
static_assert(adc_input_count == adc_pad_inputs + 1u);
static_assert(adc_temperature_code == adc_pad_inputs);
static_assert(Adc::inputs == adc_input_count);
static_assert(Adc::temperature_input == adc_temperature_code);
static_assert(Adc::dreq == Dreq::adc);
static_assert(Adc::reset_bit == ResetBlock::adc);
static_assert(Adc::irq() == ADC_IRQ_FIFO_IRQn);

// The reference of this chip is the converter's own supply pin, and the
// millivolts are the board's.
static_assert(ref_mv(Ref::avdd_pin) == 3300u);
static_assert(ref_mv(Ref::avdd_pin, 3000) == 3000u);

/// The four inputs every package has, by their pads.
using In0 = AnalogIn<Pin<adc_first_pin>>;
using In1 = AnalogIn<Pin<adc_first_pin + 1u>>;
using In2 = AnalogIn<Pin<adc_first_pin + 2u>>;
using In3 = AnalogIn<Pin<adc_first_pin + 3u>>;

static_assert(In0::input == 0u && In3::input == 3u);
static_assert(Adc::input_code(In2{}) == 2u);
static_assert(Adc::input_code(AdcInput::temperature) == adc_temperature_code);
static_assert(Adc::input_code(static_cast<uint8_t>(1)) == 1u);

/// The four the QFN-80 adds - named only where the package has them.
template <uint8_t pin, bool bonded>
void high_input() {
    if constexpr (bonded) {
        using In = AnalogIn<Pin<pin>>;
        static_assert(In::input == pin - adc_first_pin);
        (void)In::claim();
        (void)In::claim(PinPull::up);
        Adc::select(In{});
        (void)Adc::input_code(In{});
        (void)In::release();
    }
}

void arithmetic() {
    // The divider both ways, and what it refuses.
    constexpr auto d = adc_divider_for(adc_nominal_clk_hz, 10'000u);
    static_assert(d.has_value());
    static_assert(adc_rate_hz(adc_nominal_clk_hz, *d) == 10'000u);
    static_assert(!adc_divider_for(adc_nominal_clk_hz, 0u).has_value());
    static_assert(!adc_divider_for(0u, 1000u).has_value());
    static_assert(adc_divider_for(adc_nominal_clk_hz, 1u).has_value() == false);   // past sixteen bits
    static_assert(AdcDivider{} == AdcDivider{0, 0});
    static_assert(AdcDivider{47999, 0}.reg() == (47999u << ADC_DIV_INT_LSB));
    static_assert(adc_temperature_centi(891, 3300) == 2012);
    static_assert(AdcFifoConfig{}.valid());
    static_assert(!AdcFifoConfig{.threshold = 15}.valid());
    static_assert(adc_pin_valid(adc_first_pin + adc_pad_inputs - 1u));
    static_assert(!adc_pin_valid(adc_first_pin + adc_pad_inputs));
    static_assert(adc_input_of(adc_first_pin + 1u) == 1u);
    static_assert(AdcFlag::ready == ADC_CS_READY_BITS);
    static_assert(AdcFlag::error == ADC_CS_ERR_BITS);
    static_assert(AdcFlag::error_sticky == ADC_CS_ERR_STICKY_BITS);
    static_assert(Adc::entry_error == ADC_FIFO_ERR_BITS);
    static_assert(Adc::entry_value(0x8123u) == 0x123u);
    static_assert(Adc::entry_failed(0x8123u) && !Adc::entry_failed(0x0123u));
}

void lifecycle() {
    (void)Adc::package_matches();
    (void)Adc::init(SysClock{});
    (void)Adc::init(SysClock{}, AdcClock::pll_usb);
    (void)Adc::init(SysClock{}, AdcClock::crystal);
    (void)Adc::clock_hz();
    Adc::enable(true);
    (void)Adc::enabled();
    (void)Adc::ready();
    (void)Adc::wait_ready();
    (void)Adc::wait_ready(10u);
    Adc::temperature_sensor(true);
    (void)Adc::temperature_sensor();
    (void)Adc::regs().CS;
    (void)Adc::fifo_address();
}

void conversions() {
    (void)In0::claim();
    (void)In1::claim(PinPull::down);
    Adc::select(In0{});
    Adc::select(AdcInput::ain1);
    Adc::select(AdcInput::temperature);
    Adc::select_input(0);
    (void)Adc::selected();
    (void)Adc::selected_input();
    Adc::start();
    (void)Adc::result();
    (void)Adc::read();
    (void)Adc::read(10u);

    Adc::start_many(true);
    (void)Adc::running();
    Adc::divider(AdcDivider{479, 0});
    Adc::divider(*adc_divider_for(adc_nominal_clk_hz, 1000u));
    (void)Adc::divider();
    (void)Adc::rate_hz();
    (void)Adc::round_robin(adc_round_robin_mask);
    (void)Adc::round_robin(static_cast<uint16_t>(0x0Fu));
    Adc::round_robin<adc_round_robin_mask>();
    Adc::round_robin<0x03u>();
    (void)Adc::round_robin();
    (void)Adc::stop_many();
    (void)Adc::stop_many(10u);

    (void)Adc::error();
    (void)Adc::error_seen();
    Adc::clear_error_seen();

    (void)In0::release();
    (void)In1::release();
}

void fifo_and_interrupt() {
    (void)Adc::fifo({});
    (void)Adc::fifo({.enable = true, .dreq = true, .error_flag = true, .shift = true, .threshold = 4});
    (void)Adc::fifo({.enable = false, .dreq = false, .error_flag = false, .shift = false, .threshold = 0});
    (void)Adc::fifo_enabled();
    (void)Adc::fifo_level();
    (void)Adc::fifo_threshold();
    (void)Adc::fifo_empty();
    (void)Adc::fifo_full();
    (void)Adc::fifo_overflowed();
    (void)Adc::fifo_underflowed();
    Adc::clear_fifo_flags();
    (void)Adc::pop();
    Adc::drain();

    Adc::interrupt(true);
    (void)Adc::interrupt();
    (void)Adc::raw_pending();
    (void)Adc::pending();
    Adc::force(true);
    Adc::force(false);
    (void)Adc::isr();
    Irq::enable(Adc::irq());
    Irq::disable(Adc::irq());
    Adc::release();
}

// The sampler over this converter, on inputs every package has.
struct Probe {
    using Event = std::variant<AnalogSample>;
    static inline EventQueue<Event, 4, Rp2350Platform<>> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};
using Sampler = AnalogSampler<Adc, Rp2350Platform<>, Subscribers<Probe>, AdcInput::temperature, In0{}, In3{}>;

void sampler() {
    static_assert(AnalogConverter<Adc>);
    static_assert(Sampler::input_count == 3u);
    Sampler::init();
    Sampler::start_every(10u);
    (void)Sampler::running_every();
    (void)Sampler::unknown_inputs();
    Sampler::stop();
    post<Sampler>(Sampled{1234u, Adc::temperature_input});
}

void adc() {
    arithmetic();
    lifecycle();
    conversions();
    fifo_and_interrupt();
    sampler();
    // Inputs 4..7 are the QFN-80's alone: the same pads that are GPIO
    // 44..47 there, and no pad at all in the smaller package.
    high_input<adc_first_pin + 4u, (adc_pad_inputs > 4u)>();
    high_input<adc_first_pin + 7u, (adc_pad_inputs > 7u)>();
}
