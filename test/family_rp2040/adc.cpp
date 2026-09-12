// ADC family smoke TU: the vocabulary and its arithmetic, the inputs,
// the resource's verbs, the sampler over it, a DMA engine on the FIFO.
#include "rp2040/adc.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/platform.hpp"
#include "util/analog_sampler.hpp"

using namespace brio;

static_assert(adc_bits == 12u && adc_steps == 4096u && adc_max_count == 4095u);
static_assert(Adc::temperature_input == 4u && Adc::inputs == 5u);
static_assert(adc_divider_for(48'000'000, 250'000)->integer == 191u);
static_assert(adc_temperature_centi(891, 3300) == 2012);
static_assert(AnalogIn<Pin<27>>::input == 1u);
static_assert(AnalogConverter<Adc>);
static_assert(SamplerInput<Adc, AdcInput::temperature> && SamplerInput<Adc, AnalogIn<Pin<26>>{}>);
static_assert(Adc::input_code(AnalogIn<Pin<29>>{}) == 3u && Adc::input_code(AdcInput::temperature) == 4u);
static_assert(Adc::entry_failed(0x8000u) && Adc::entry_value(0x8FFFu) == 0xFFFu);

using SysClock = Clock<ClockSource::pll, 125'000'000>;
using P = Rp2040Platform<>;
using In0 = AnalogIn<Pin<26>>;
struct Listener {
    using Event = AnalogSample;
    static inline EventQueue<Event, 4, P> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};
using Sampler = AnalogSampler<Adc, P, Subscribers<Listener>, AdcInput::temperature, In0{}>;
using Block = DmaRxEngine<7, uint16_t>;
using ByteBlock = DmaRxEngine<8, uint8_t>;

uint16_t words[64];
uint8_t bytes[64];

void adc_verbs() {
    constexpr SysClock clock;
    (void)Adc::init(clock);
    (void)Adc::init(clock, AdcClock::crystal);
    (void)Adc::clock_hz();
    Adc::enable(true);
    (void)Adc::enabled();
    (void)Adc::ready();
    (void)Adc::wait_ready();
    Adc::temperature_sensor(true);
    (void)Adc::temperature_sensor();
    In0::claim();
    Adc::select(In0{});
    Adc::select(AdcInput::temperature);
    Adc::select_input(2);
    (void)Adc::selected();
    (void)Adc::selected_input();
    Adc::start();
    (void)Adc::result();
    (void)Adc::read();
    Adc::divider(*adc_divider_for(Adc::clock_hz(), 10'000));
    (void)Adc::divider();
    (void)Adc::rate_hz();
    Adc::round_robin(0x13);
    (void)Adc::round_robin();
    Adc::fifo({.enable = true, .dreq = true, .error_flag = true, .shift = false, .threshold = 1});
    (void)Adc::fifo_enabled();
    (void)Adc::fifo_level();
    (void)Adc::fifo_empty();
    (void)Adc::fifo_full();
    (void)Adc::fifo_overflowed();
    (void)Adc::fifo_underflowed();
    Adc::clear_fifo_flags();
    Block::arm(Adc::fifo_address(), Adc::dreq);
    (void)Block::start(words, 64);
    ByteBlock::arm(Adc::fifo_address(), Adc::dreq);
    (void)ByteBlock::start(bytes, 64);
    Adc::start_many(true);
    (void)Adc::running();
    (void)Adc::stop_many();
    (void)Adc::pop();
    Adc::drain();
    (void)Adc::error();
    (void)Adc::error_seen();
    Adc::clear_error_seen();
    Adc::interrupt(true);
    (void)Adc::raw_pending();
    (void)Adc::pending();
    Adc::force(false);
    (void)Adc::isr();
    Sampler::init();
    In0::release();
    Adc::release();
    (void)PllUsb::locked();
    Clocks::adc_select(AdcAux::xosc, 2);
    (void)Clocks::adc_enabled();
    (void)Clocks::adc_source();
    Clocks::adc_stop();
    Pin<26>::analog();
}
