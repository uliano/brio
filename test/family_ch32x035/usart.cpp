// USART family smoke TU: the resource over the whole of RM ch. 14 and the
// transport on it, on the one instance every part offers - USART2, whose
// default pads PA2/PA3 every package bonds - and on the others where the
// part has them, each branch asked of device:: (a template, so a part
// without the instance never forms it).
#include "ch32x035/clock.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"
#include "util/stream.hpp"

using namespace brio;

using P = Ch32x035Platform<>;
using SysClock = Clock<ClockSource::internal, 48'000'000>;

// ---- the arithmetic ---------------------------------------------------------
static_assert(usart_divisor(48'000'000, 115200) == 417);
static_assert(usart_divisor(8'000'000, 115200) == 69);
static_assert(usart_divisor_valid(16) && !usart_divisor_valid(15));
static_assert(usart_actual_baud(48'000'000, 417) == 115107);
static_assert(usart_min_hz(3'000'000) == 48'000'000);   // the chapter's "up to 3Mbps"
static_assert(uart_format_valid(UartFormat{}));
static_assert(!uart_format_valid(UartFormat{UartBits::seven, UartParity::none, UartStop::one}));
static_assert(usart_ctlr1_format(UartFormat{UartBits::eight, UartParity::even, UartStop::one}) ==
              (usart_m | usart_pce));
static_assert(irda_valid(IrdaConfig{}) && !irda_valid(IrdaConfig{false, 2}));
static_assert(smartcard_valid(SmartcardConfig{}) && !smartcard_valid(SmartcardConfig{true, 0, 32}));

// ---- the instances ------------------------------------------------------------
static_assert(usart_bus_for(1) == Bus::pb2 && usart_bus_for(3) == Bus::pb1);
static_assert(usart_gate_for(4) == rcc_pb1_usart4);
static_assert(usart_irq_for(3) == Irq::usart3);
static_assert(Usart<2>::dma_tx_channel == 7 && Usart<2>::dma_rx_channel == 6);
static_assert(Usart<2>::is_full);
static_assert(usart_remap_valid(2, 0));
// USART3's code 1 and USART4's 1x0 land on the debug pads.
static_assert(usart_column_on_debug_port(3, 1) && usart_column_on_debug_port(4, 4));
static_assert(!usart_column_on_debug_port(2, 0));

using Serial = Uart<2, P>;
static_assert(ByteTransport<Serial>);
static_assert(Serial::pads.tx == Pad{'A', 2} && Serial::pads.rx == Pad{'A', 3});

template <uint8_t n>
void resource_verbs() {
    using U = Usart<n>;
    U::bus_clock(true);
    U::reset();
    (void)U::remap(0);
    (void)U::configure(UartFormat{}, 417);
    (void)U::stop_bits(UartStop::two);
    (void)U::set_brr(417);
    (void)U::brr();
    (void)U::enabled();
    U::enable(false);
    U::transmitter(true);
    U::receiver(true);
    (void)U::mute_mode(MuteConfig{MuteWake::address_mark, 3});
    (void)U::mute();
    U::unmute();
    (void)U::muted();
    (void)U::lin(LinConfig{true, true});
    U::send_break();
    (void)U::break_pending();
    (void)U::lin_enabled();
    U::lin_off();
    (void)U::half_duplex(true);
    (void)U::half_duplex();
    (void)U::half_duplex(false);
    (void)U::irda(IrdaConfig{true, 4});
    (void)U::irda_enabled();
    U::irda_off();
    (void)U::prescaler();
    (void)U::guard_time();
    (void)U::smartcard(SmartcardConfig{});
    (void)U::smartcard_enabled();
    U::smartcard_off();
    (void)U::synchronous(UsartSyncConfig{true, true, false});
    (void)U::synchronous_enabled();
    (void)U::synchronous_off();
    (void)U::flow_control(true, true);
    (void)U::rts_enabled();
    (void)U::cts_enabled();
    U::dma_transmit(false);
    U::dma_receive(false);
    (void)U::data_address();
    U::interrupts(usart_idleie, true);
    U::rxne_interrupt(true);
    U::txe_interrupt(false);
    (void)U::txe_interrupt();
    U::tc_interrupt(false);
    U::idle_interrupt(false);
    U::parity_interrupt(false);
    U::break_interrupt(false);
    U::cts_interrupt(false);
    U::error_interrupt(false);
    (void)U::status();
    (void)U::flag(usart_txe);
    U::clear_flags(usart_tc | usart_lbd);
    U::clear_by_read();
    (void)U::tx_empty();
    (void)U::tx_complete();
    (void)U::rx_ready();
    (void)U::read_word();
    U::write_word(0x155);
    (void)U::read_data();
    U::write_data(0x55);
    (void)U::take_errors();
    (void)U::actual_baud(48'000'000);
}

template <uint8_t n>
void resource_where_offered() {
    if constexpr (device::has_usart(n)) {
        resource_verbs<n>();
    }
}

void every_instance() {
    resource_where_offered<1>();
    resource_where_offered<2>();
    resource_where_offered<3>();
    resource_where_offered<4>();
}

void transport_verbs() {
    constexpr SysClock clock;
    constexpr Serial serial;
    (void)serial;
    (void)Serial::init(clock, 115200);
    (void)Serial::isr();
    (void)Serial::dma_isr();
    (void)Serial::harvest();
    (void)Serial::dma_faults();
    (void)Serial::write_byte('x');
    uint8_t b = 0;
    (void)Serial::read_byte(b);
    static const uint8_t bytes[3] = {1, 2, 3};
    (void)Serial::write(bytes, 3);
    (void)Serial::tx_idle();
    (void)Serial::rx_pending();
    (void)Serial::baud();
    (void)Serial::actual_baud(48'000'000);
    (void)Serial::rx_overruns();
    (void)Serial::hw_overruns();
    (void)Serial::frame_errors();
    (void)Serial::noise_errors();
    (void)Serial::parity_errors();
    Serial::clear_errors();
    static_assert(Serial::divisor_for(48'000'000, 115200) == 417);
    static_assert(Serial::min_hz_for(115200) == 1'843'200);
    static_assert(Serial::can_baud(48'000'000, 3'000'000) && !Serial::can_baud(8'000'000, 1'000'000));
    Serial::rebase(24'000'000);
    (void)Serial::set_baud(48'000'000, 57600);
    Serial::release();
}

// The options and a frame of its own: 7E1 with the flow-control pair and
// a single-wire port, on the default column where CTS (PA0) and RTS (PA1)
// are bonded on every package.
using Flow = Uart<2, P, 32, 32, UartFormat{UartBits::seven, UartParity::even, UartStop::one},
                  NoDmaEngine, NoDmaEngine, 0, UartOptions{false, true, true}>;
using Wire = Uart<2, P, 16, 16, UartFormat{}, NoDmaEngine, NoDmaEngine, 0,
                  UartOptions{true, false, false}>;

void options_verbs() {
    constexpr SysClock clock;
    (void)Flow::init(clock, 9600);
    (void)Wire::init(clock, 19200);
    Flow::release();
    Wire::release();
}

// Vector bindings, as an app writes them.
extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Serial::isr(); }
