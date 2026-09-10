// USART family smoke TU: the one instance the stratum implements, its
// pads, the divisor arithmetic (BRR = pclk / baud, whole), the
// transport concepts and every verb.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"
#include "util/print.hpp"
#include "util/stream.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;
using Serial = Uart<1, P>;
using Wide = Uart<1, P, 128, 256>;

static_assert(usart_base_for(1) == 0x40013800 && usart_base_for(2) == 0);
static_assert(usart_pads_for(1).tx_port == 'D' && usart_pads_for(1).tx_pin == 5);
static_assert(usart_pads_for(1).rx_port == 'D' && usart_pads_for(1).rx_pin == 6);
static_assert(std::same_as<Serial::Tx, Pin<'D', 5>>);

// RM 16.6.3: the whole register is fclk/baud in sixteenths.
static_assert(Serial::divisor_for(48'000'000, 115200) == 417);
static_assert(Serial::divisor_for(8'000'000, 115200) == 69);     // the reset clock: +0.6%
static_assert(Serial::divisor_for(48'000'000, 3'000'000) == 16); // the top: legal
static_assert(Serial::divisor_for(48'000'000, 4'000'000) == 12); // below 16: init() refuses
static_assert(Serial::divisor_for(48'000'000, 0) == 0);

static_assert(ByteSink<Serial> && ByteSource<Serial> && ByteTransport<Serial>);
static_assert(ByteTransport<Wide>);

void usart_verbs() {
    constexpr SysClock clock;
    constexpr Serial serial;
    (void)Serial::init(clock, 115200);
    (void)Wide::init(clock, 9600);
    (void)Serial::isr();
    (void)Serial::write_byte(0x55);
    uint8_t b;
    (void)Serial::read_byte(b);
    (void)Serial::rx_pending();
    (void)Serial::tx_idle();
    (void)Serial::rx_overruns();
    (void)Serial::frame_errors();
    (void)Serial::parity_errors();
    (void)Serial::noise_errors();
    (void)Serial::hw_overruns();
    (void)Serial::actual_baud(SysClock::pclk_hz);
    Serial::clear_errors();
    Serial::rebase(24'000'000);
    print(serial, "x=", hex(0x1234u), " ", fixed(3.14f, 6, 2), " ", sci(0.00123f), crlf);
}
