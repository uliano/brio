// USART family smoke TU: the resource's verbs over the whole of RM
// chapter 18, the frame and divisor arithmetic at compile time, the
// columns the remap tables give each instance, and the interrupt-driven
// transport with its options, its engine slots and its vector binding.
//
// WHICH INSTANCE a part offers is the table's answer and it is not
// always the first n: the smallest package offers exactly one usart and
// it is USART2, because it bonds neither of USART1's pin pairs. So this
// TU names no instance number as a literal - it asks - and a neg TU
// proves that USART1 is refused there.
//
// WHICH INSTANCE IS A FULL USART is a second question with a second
// answer: the fourth serial port is a USART4 on the CH32V203C8 and a
// UART4 on the other device class, so the synchronous clock, the
// smartcard and the flow-control pair are asked for through
// device::usart_full() and never through an instance's number.
#include "ch32v203/clock.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/usart.hpp"
#include "util/stream.hpp"

using namespace brio;

using P = Ch32v203Platform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

/// The console's instance on this part: USART1 where the package bonds
/// its pads, USART2 otherwise - the only one every part of the family
/// offers.
inline constexpr uint8_t console_instance = device::has_usart(1) ? 1 : 2;
/// The HIGHEST instance this part offers, which is UART4 on the two
/// parts that have four and the console's own everywhere else: a second
/// resource to exercise, and the one whose vector number moves with the
/// device class.
inline constexpr uint8_t last_instance = device::has_usart(4) ? 4 : console_instance;
/// Whether the console's own column brings out its flow-control pads on
/// this package: two of the twenty-pin parts bring out neither.
inline constexpr bool console_flow_pads =
    pad_bonded(usart_column_for(console_instance, 0).cts) &&
    pad_bonded(usart_column_for(console_instance, 0).rts);

using Console = Usart<console_instance>;
using Last = Usart<last_instance>;
using Serial = Uart<console_instance, P>;
using Seven = Uart<console_instance, P, 32, 32, UartFormat{UartBits::seven, UartParity::even}>;

/// The transport wearing its two options: one wire, and the pair - the
/// second only where the package has the pads for it.
inline constexpr UartOptions wire_opts{.half_duplex = true};
inline constexpr UartOptions flow_opts{.rts = console_flow_pads, .cts = console_flow_pads};
using Wire = Uart<console_instance, P, 64, 64, UartFormat{}, NoDmaEngine, NoDmaEngine, 0, wire_opts>;
using Flow = Uart<console_instance, P, 64, 64, UartFormat{}, NoDmaEngine, NoDmaEngine, 0, flow_opts>;

// USART1 is the PB2 instance and the other three are PB1's, which is
// what decides the gate and the clock a divisor counts.
static_assert(usart_bus_for(1) == Bus::pb2);
static_assert(usart_bus_for(2) == Bus::pb1 && usart_bus_for(4) == Bus::pb1);
static_assert(usart_gate_for(1) == rcc_pb2_usart1 && usart_gate_for(4) == rcc_pb1_uart4);
static_assert(usart_irq_for(1) == Irq::usart1 && usart_irq_for(4) == Irq::uart4);
static_assert(usart_remap_of(1) == Remap::usart1 && usart_remap_of(4) == Remap::uart4);

// The pads are afio.hpp's columns and this file states no table of its
// own: the default column of the instances this part has, and UART4's
// PB0/PB1 - the CH32V203C8's own table, not the PC10/PC11 of the one
// the bigger class reads.
static_assert(!device::has_usart(1) ||
              (usart_pads_for(1).tx == Pad{'A', 9} && usart_pads_for(1).rx == Pad{'A', 10}));
static_assert(!device::has_usart(2) ||
              (usart_pads_for(2).tx == Pad{'A', 2} && usart_pads_for(2).rx == Pad{'A', 3}));
static_assert(!device::has_usart(3) || usart_pads_for(3).tx == Pad{'B', 10});
static_assert(!device::has_usart(4) || device::device_class != DeviceClass::v20x_d6 ||
              (usart_pads_for(4).tx == Pad{'B', 0} && usart_pads_for(4).rx == Pad{'B', 1}));
static_assert(!device::has_usart(4) || device::device_class == DeviceClass::v20x_d6 ||
              (usart_pads_for(4).tx == Pad{'C', 10} && usart_pads_for(4).rx == Pad{'C', 11}));
// The column carries five pads, and the remapped one is another five.
static_assert(!device::has_usart(1) || usart_column_for(1, 0).ck == Pad{'A', 8});
static_assert(!device::has_usart(1) || usart_column_for(1, 1).tx == Pad{'B', 6});
static_assert(!device::has_usart(2) || usart_column_for(2, 0).cts == Pad{'A', 0});
static_assert(!device::has_usart(2) || usart_column_for(2, 0).rts == Pad{'A', 1});
// A column exists where the device class has it and the package bonds a
// pad of it: USART2's second column is the other class's.
static_assert(!device::has_usart(console_instance) || usart_remap_valid(console_instance, 0));
static_assert(!usart_remap_valid(2, 1) || device::device_class != DeviceClass::v20x_d6);

// The fourth port is a full USART on the CH32V203C8 and a UART4 on the
// other two classes, which is what decides the clock, the smartcard and
// the pair.
static_assert(Console::is_full);
static_assert(!device::has_usart(4) ||
              device::usart_full(4) == (device::device_class == DeviceClass::v20x_d6));

// The frame: seven data bits exist only with a parity bit and nine only
// without, because the register counts the WORD.
static_assert(uart_format_valid(UartFormat{}));
static_assert(uart_format_valid(UartFormat{UartBits::seven, UartParity::odd}));
static_assert(!uart_format_valid(UartFormat{UartBits::seven, UartParity::none}));
static_assert(!uart_format_valid(UartFormat{UartBits::nine, UartParity::even}));
static_assert(usart_ctlr1_format(UartFormat{}) == 0u);
static_assert(usart_ctlr1_format(UartFormat{UartBits::nine, UartParity::none}) == usart_m);
static_assert(usart_ctlr1_format(UartFormat{UartBits::eight, UartParity::odd}) ==
              (usart_m | usart_pce | usart_ps));
static_assert(usart_ctlr2_stop(UartStop::two) == (2u << usart_stop_shift));
static_assert(usart_ctlr2_stop(UartStop::one_and_half) == (3u << usart_stop_shift));
static_assert(uart_data_mask(UartFormat{}) == 0xFFu);
static_assert(uart_data_mask(UartFormat{UartBits::nine, UartParity::none}) == 0x1FFu);
static_assert(uart_data_mask(UartFormat{UartBits::seven, UartParity::even}) == 0x7Fu);

// The divisor is pclk/baud in sixteenths, rounded to nearest, and below
// sixteen the generator has nothing to divide.
static_assert(usart_divisor(48'000'000UL, 115200) == 417u);
static_assert(usart_divisor(0u, 115200) == 0u);
static_assert(usart_divisor(115200, 0) == 0u);
static_assert(usart_divisor_valid(16u) && !usart_divisor_valid(15u));
static_assert(!usart_divisor_valid(0x10000u));
static_assert(usart_actual_baud(48'000'000UL, 417u) == 115107u);
static_assert(usart_actual_baud(48'000'000UL, 0u) == 0u);
static_assert(usart_min_hz(115200) == 1'843'200UL);
static_assert(Serial::can_baud(48'000'000UL, 115200) && !Serial::can_baud(48'000'000UL, 4'000'000UL));
static_assert(Serial::min_hz_for(9600) == 153'600UL);
static_assert(Serial::divisor_for(48'000'000UL, 9600) == 5000u);

// The modes' own vocabulary: what each config refuses before a register
// is touched.
static_assert(mute_valid(MuteConfig{MuteWake::address_mark, 15}));
static_assert(!mute_valid(MuteConfig{MuteWake::address_mark, 16}));
static_assert(irda_valid(IrdaConfig{.low_power = false, .prescaler = 1}));
static_assert(!irda_valid(IrdaConfig{.low_power = false, .prescaler = 2}));
static_assert(irda_valid(IrdaConfig{.low_power = true, .prescaler = 200}));
static_assert(!irda_valid(IrdaConfig{.low_power = true, .prescaler = 0}));
static_assert(smartcard_valid(SmartcardConfig{.clock_prescaler = 31}));
static_assert(!smartcard_valid(SmartcardConfig{.clock_prescaler = 32}));
static_assert(!smartcard_valid(SmartcardConfig{.clock_prescaler = 0}));

// The transport is a ByteTransport for util's services.
static_assert(ByteSink<Serial> && ByteSource<Serial>);

void usart_resource_verbs() {
    Console::bus_clock(true);
    Console::reset();
    (void)Console::remap(0);
    (void)Console::configure(UartFormat{}, 417);
    (void)Console::stop_bits(UartStop::two);
    (void)Console::set_brr(417);
    (void)Console::brr();
    Console::enable(true);
    (void)Console::enabled();
    Console::transmitter(true);
    Console::receiver(true);

    (void)Console::mute_mode({.wake = MuteWake::address_mark, .address = 5});
    (void)Console::mute();
    Console::unmute();
    (void)Console::muted();

    (void)Console::lin({.break_11bit = true, .break_interrupt = true});
    (void)Console::lin_enabled();
    Console::send_break();
    (void)Console::break_pending();
    Console::lin_off();

    (void)Console::half_duplex(true);
    (void)Console::half_duplex();
    (void)Console::half_duplex(false);

    (void)Console::irda({.low_power = true, .prescaler = 16});
    (void)Console::irda_enabled();
    Console::irda_off();
    (void)Console::prescaler();
    (void)Console::guard_time();

    (void)Console::smartcard({.nack = true, .guard_time = 16, .clock_prescaler = 4});
    (void)Console::smartcard_enabled();
    Console::smartcard_off();

    (void)Console::synchronous({.clock_idle_high = true, .capture_second_edge = true,
                                .last_bit_clock = true});
    (void)Console::synchronous_enabled();
    (void)Console::synchronous_off();

    (void)Console::flow_control(true, true);
    (void)Console::rts_enabled();
    (void)Console::cts_enabled();
    Console::dma_transmit(true);
    Console::dma_receive(true);
    (void)Console::data_address();

    Console::interrupts(usart_rxneie | usart_peie, true);
    Console::rxne_interrupt(true);
    Console::txe_interrupt(true);
    (void)Console::txe_interrupt();
    Console::tc_interrupt(true);
    Console::idle_interrupt(true);
    Console::parity_interrupt(true);
    Console::break_interrupt(true);
    Console::cts_interrupt(true);
    Console::error_interrupt(true);

    (void)Console::status();
    (void)Console::flag(usart_tc);
    Console::clear_flags(usart_tc | usart_rxne);
    Console::clear_by_read();
    (void)Console::tx_empty();
    (void)Console::tx_complete();
    (void)Console::rx_ready();
    (void)Console::read_word();
    Console::write_word(0x155);
    (void)Console::read_data();
    Console::write_data(0x55);
    (void)Console::take_errors();
    (void)Console::actual_baud(48'000'000UL);
    Console::enable(false);
    (void)Console::number;
    (void)Console::bus;
    (void)Console::irq;
    (void)Console::pads;
    (void)Console::dma_tx_channel;
    (void)Console::dma_rx_channel;
    (void)Console::regs();

    Last::bus_clock(true);
    (void)Last::is_full;
    Last::bus_clock(false);
    Pfic::enable(usart_irq_for(last_instance));
    Pfic::disable(usart_irq_for(last_instance));
}

void uart_task_verbs() {
    constexpr SysClock clock;

    (void)Serial::init(clock, 115200);
    (void)Seven::init(clock, 9600);
    (void)Wire::init(clock, 9600);
    (void)Flow::init(clock, 9600);
    (void)Serial::write_byte('x');
    static const uint8_t greeting[2] = {'h', 'i'};
    (void)Serial::write(greeting, 2);
    uint8_t got = 0;
    (void)Serial::read_byte(got);
    (void)Serial::rx_pending();
    (void)Serial::tx_idle();
    (void)Serial::baud();
    (void)Serial::actual_baud(48'000'000UL);
    (void)Serial::set_baud(48'000'000UL, 9600);
    Serial::rebase(24'000'000UL);
    (void)Serial::rx_overruns();
    (void)Serial::hw_overruns();
    (void)Serial::frame_errors();
    (void)Serial::noise_errors();
    (void)Serial::parity_errors();
    (void)Serial::dma_faults();
    Serial::clear_errors();
    (void)Serial::harvest();
    (void)Serial::isr();
    (void)Serial::number;
    (void)Serial::remap_code;
    (void)Serial::pads;
    (void)Serial::column;
    (void)Serial::options;
    (void)Serial::has_tx_engine;
    (void)Serial::has_rx_engine;
    (void)Serial::regs();
    Wire::release();
    Flow::release();
    Serial::release();
}

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
