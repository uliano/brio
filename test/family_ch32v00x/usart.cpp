// USART family smoke TU: both instances, their pads and columns, the
// divisor arithmetic (BRR = pclk / baud, whole), the chapter's
// vocabulary, every verb of the resource, the transport's options and
// concepts, and every verb of the transport.
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

static_assert(usart_base_for(1) == 0x40013800 && usart_base_for(3) == 0);
#if BRIO_CH32_HAS_USART2
static_assert(usart_base_for(2) == 0x40004400);
#else
static_assert(usart_base_for(2) == 0);   // the CH32V003 has one USART
#endif
static_assert(usart_pads_for(1).tx_port == 'D' && usart_pads_for(1).tx_pin == 5);
static_assert(usart_pads_for(1).rx_port == 'D' && usart_pads_for(1).rx_pin == 6);
static_assert(std::same_as<Serial::Tx, Pin<'D', 5>>);
// The remaps: USART1's column 1 swaps the pair on both parts; USART2
// lives remapped on the CH32V006 (its default TX is the reset pin) -
// column 3 on PD2/PD3 - and does not exist on the CH32V003, whose
// USART1 has four columns.
#if BRIO_CH32_HAS_USART2
using Swapped = Uart<1, P, 64, 64, NoDmaEngine, NoDmaEngine, 1>;
#else
using Swapped = Uart<1, P, 64, 64, NoDmaEngine, NoDmaEngine, 2>;   // the CH32V003 swaps the pair at code 2 (table 7-10)
#endif
static_assert(std::same_as<Swapped::Tx, Pin<'D', 6>> && std::same_as<Swapped::Rx, Pin<'D', 5>>);
#if BRIO_CH32_HAS_USART2
using Second = Uart<2, P, 64, 64, NoDmaEngine, NoDmaEngine, 3>;
static_assert(std::same_as<Second::Tx, Pin<'D', 2>> && std::same_as<Second::Rx, Pin<'D', 3>>);
static_assert(usart_irq_for(2) == Irq::usart2 && usart_clock_for(2) == rcc_pb2_usart2 && usart_on_pb2(2));
static_assert(!usart_remap_valid(2, 0) && usart_remap_valid(2, 6) && !usart_remap_valid(2, 7) &&
              !usart_remap_valid(1, 10));
#else
static_assert(!usart_remap_valid(2, 3) && usart_remap_valid(1, 3) && !usart_remap_valid(1, 4));
static_assert(usart_pads_for(1, 1).tx_port == 'D' && usart_pads_for(1, 1).tx_pin == 0);
static_assert(usart_pads_for(1, 3).tx_port == 'C' && usart_pads_for(1, 3).tx_pin == 0);
static_assert(usart_column_for(1, 3).ck.port == 'C' && usart_column_for(1, 3).ck.pin == 5);
#endif

// RM 16.6.3: the whole register is fclk/baud in sixteenths.
static_assert(Serial::divisor_for(48'000'000, 115200) == 417);
static_assert(Serial::divisor_for(8'000'000, 115200) == 69);     // the reset clock: +0.6%
static_assert(Serial::divisor_for(48'000'000, 3'000'000) == 16); // the top: legal
static_assert(Serial::divisor_for(48'000'000, 4'000'000) == 12); // below 16: init() refuses
static_assert(Serial::divisor_for(48'000'000, 0) == 0);

static_assert(ByteSink<Serial> && ByteSource<Serial> && ByteTransport<Serial>);
static_assert(ByteTransport<Wide>);

// The chapter's vocabulary, pinned.
static_assert(uart_format_valid({}) && !uart_format_valid({.bits = UartBits::seven}) &&
              uart_format_valid({.bits = UartBits::seven, .parity = UartParity::even}) &&
              !uart_format_valid({.bits = UartBits::nine, .parity = UartParity::odd}));
static_assert(usart_ctlr1_format({}) == 0 &&
              usart_ctlr1_format({.parity = UartParity::even}) == (usart_m | usart_pce) &&
              usart_ctlr1_format({.parity = UartParity::odd}) == (usart_m | usart_pce | usart_ps) &&
              usart_ctlr1_format({.bits = UartBits::seven, .parity = UartParity::even}) == usart_pce &&
              usart_ctlr1_format({.bits = UartBits::nine}) == usart_m);
static_assert(usart_ctlr2_stop(UartStop::two) == (2u << 12) && usart_ctlr2_stop(UartStop::one_and_half) == (3u << 12));
static_assert(uart_data_mask({.bits = UartBits::seven, .parity = UartParity::odd}) == 0x7F &&
              uart_data_mask({.bits = UartBits::nine}) == 0x1FF);
static_assert(usart_divisor_valid(16) && !usart_divisor_valid(15) && !usart_divisor_valid(0x10000));
static_assert(usart_dma_tx_channel(1) == 4 && usart_dma_rx_channel(1) == 5 &&
              usart_dma_tx_channel(2) == 6 && usart_dma_rx_channel(2) == 7);
static_assert(usart_column_for(1, 0).cts.port == 'D' && usart_column_for(1, 0).cts.pin == 3 &&
              usart_column_for(1, 0).rts.port == 'C' && usart_column_for(1, 0).rts.pin == 2);
static_assert(Usart<1>::has_synchronous == device::usart_has_synchronous &&
              Usart<1>::has_smartcard == device::usart_has_smartcard);
static_assert(Usart<1>::irq == Irq::usart1 && Usart<1>::dma_tx_channel == 4 && Usart<1>::dma_rx_channel == 5);

// The options: a single wire, the flow-control pair, a frame.
constexpr UartOptions wire_opts{.half_duplex = true};
constexpr UartOptions flow_opts{.format = {.parity = UartParity::even, .stop = UartStop::two}, .rts = true, .cts = true};
using WireUart = Uart<1, P, 64, 64, NoDmaEngine, NoDmaEngine, 1, wire_opts>;
using FlowUart = Uart<1, P, 64, 64, NoDmaEngine, NoDmaEngine, 0, flow_opts>;
using FlowCts = FlowUart::Cts;
using FlowRts = FlowUart::Rts;
static_assert(WireUart::options.half_duplex);
static_assert(std::same_as<FlowCts, Pin<'D', 3>>);
static_assert(std::same_as<FlowRts, Pin<'C', 2>>);
static_assert(ByteTransport<WireUart> && ByteTransport<FlowUart>);

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
    (void)Swapped::init(clock, 9600);
#if BRIO_CH32_HAS_USART2
    (void)Second::init(clock, 9600);
    (void)Second::isr();
#endif
    (void)WireUart::init(clock, 9600);
    (void)FlowUart::init(clock, 9600);
}

// Every verb of the resource, instantiated.
void usart_resource_verbs() {
    using R = Usart<1>;
    R::bus_clock(true);
    R::reset();
    R::remap(1);
    (void)R::enabled();
    R::enable(true);
    R::transmitter(true);
    R::receiver(true);
    (void)R::configure({.bits = UartBits::nine}, usart_divisor(48'000'000, 115200));
    (void)R::stop_bits(UartStop::one_and_half);
    (void)R::set_brr(417);
    (void)R::brr();
    (void)R::actual_baud(48'000'000);
    (void)R::mute_mode({.wake = MuteWake::address_mark, .address = 7});
    (void)R::mute();
    R::unmute();
    (void)R::muted();
    (void)R::lin({.break_11bit = true, .break_interrupt = true});
    R::lin_off();
    (void)R::lin_enabled();
    R::send_break();
    (void)R::break_pending();
    (void)R::half_duplex(true);
    (void)R::half_duplex();
    (void)R::irda({.low_power = true, .prescaler = 3});
    R::irda_off();
    (void)R::irda_enabled();
    (void)R::prescaler();
    R::flow_control(true, true);
    (void)R::rts_enabled();
    (void)R::cts_enabled();
    R::dma_transmit(true);
    R::dma_receive(true);
    R::interrupts(usart_idleie | usart_tcie | usart_peie, true);
    R::rxne_interrupt(true);
    R::txe_interrupt(true);
    (void)R::txe_interrupt();
    R::break_interrupt(true);
    R::cts_interrupt(true);
    R::error_interrupt(true);
    (void)R::status();
    (void)R::flag(usart_lbd | usart_cts);
    R::clear_flags(usart_tc | usart_lbd);
    R::clear_by_read();
    (void)R::tx_empty();
    (void)R::tx_complete();
    (void)R::rx_ready();
    R::write_data(0x55);
    R::write_word(0x155);
    (void)R::read_data();
    (void)R::read_word();
    (void)R::data_address();
}
