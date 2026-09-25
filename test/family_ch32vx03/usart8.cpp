// mcu: ch32v303rc ch32v303vc
// UART5..UART8 family smoke TU: the four serial ports the CH32V303RC and
// VC carry beyond UART4 - every fact of the four instances (the bus, the
// gate, the vector at the class's own tail, the DMA2 slots of tables 11-3
// and 11-4, the columns of tables 10-28..10-31), the resource's verbs a
// UART has, the transport on each with and without its engines, and the
// two columns that land on the debug port's pads. And UART4 as this class
// has it: PC10/PC11 by default, a UART and not a USART, DMA2's.
//
// WHY THESE TWO PARTS: table 2-1-1 gives eight serial ports to the 256 KB
// CH32V303 and three to the 128 KB ones; the CH32V203 has four at most.
// A neg TU proves UART5 refused everywhere else.
#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"
#include "util/stream.hpp"

using namespace brio;

using P = Ch32vx03Platform<>;
using SysClock = Clock<ClockSource::pll, 144'000'000>;

using U4 = Usart<4>;
using U5 = Usart<5>;
using U6 = Usart<6>;
using U7 = Usart<7>;
using U8 = Usart<8>;

// ---- the instances' facts ---------------------------------------------------

static_assert(device::has_usart(5) && device::has_usart(6) && device::has_usart(7) &&
              device::has_usart(8));
// All five of the upper ports are UARTs: TX and RX, no CK, no CTS, no RTS.
static_assert(!U4::is_full && !U5::is_full && !U6::is_full && !U7::is_full && !U8::is_full);
// On PB1, every one of them - at addresses that are not in instance order.
static_assert(U5::bus == Bus::pb1 && U8::bus == Bus::pb1);
static_assert(usart_base_for(5) == pb1_base + 0x5000 && usart_base_for(6) == pb1_base + 0x1800 &&
              usart_base_for(7) == pb1_base + 0x1C00 && usart_base_for(8) == pb1_base + 0x2000);
static_assert(usart_gate_for(5) == rcc_pb1_uart5 && usart_gate_for(6) == rcc_pb1_uart6 &&
              usart_gate_for(7) == rcc_pb1_uart7 && usart_gate_for(8) == rcc_pb1_uart8);
// The vectors of this class's tail: UART4 68, UART5 69, UART6..8 87..89.
static_assert(static_cast<uint8_t>(U4::irq) == 68 && static_cast<uint8_t>(U5::irq) == 69);
static_assert(static_cast<uint8_t>(U6::irq) == 87 && static_cast<uint8_t>(U7::irq) == 88 &&
              static_cast<uint8_t>(U8::irq) == 89);
// The DMA2 slots: UART4 5 and 3, UART5 4 and 2, UART6 6 and 7, UART7 8 and
// 9, UART8 10 and 11 (tables 11-3 and 11-4).
static_assert(U4::dma_tx_slot == DmaSlot{2, 5} && U4::dma_rx_slot == DmaSlot{2, 3});
static_assert(U5::dma_tx_slot == DmaSlot{2, 4} && U5::dma_rx_slot == DmaSlot{2, 2});
static_assert(U6::dma_tx_slot == DmaSlot{2, 6} && U6::dma_rx_slot == DmaSlot{2, 7});
static_assert(U7::dma_tx_slot == DmaSlot{2, 8} && U7::dma_rx_slot == DmaSlot{2, 9});
static_assert(U8::dma_tx_slot == DmaSlot{2, 10} && U8::dma_rx_slot == DmaSlot{2, 11});
// The default columns, and the remap fields that move them.
static_assert(U4::pads.tx == Pad{'C', 10} && U4::pads.rx == Pad{'C', 11});
static_assert(U5::pads.tx == Pad{'C', 12} && U5::pads.rx == Pad{'D', 2});
static_assert(U6::pads.tx == Pad{'C', 0} && U6::pads.rx == Pad{'C', 1});
static_assert(U7::pads.tx == Pad{'C', 2} && U7::pads.rx == Pad{'C', 3});
static_assert(U8::pads.tx == Pad{'C', 4} && U8::pads.rx == Pad{'C', 5});
static_assert(usart_remap_of(5) == Remap::uart5 && usart_remap_of(8) == Remap::uart8);
static_assert(usart_remap_valid(5, 0) && usart_remap_valid(5, 1) && usart_remap_valid(8, 1));
// The third column is port E's, which only the LQFP100 bonds.
static_assert(usart_remap_valid(6, 2) == device::has_port('E'));
static_assert(!pad_bonded(usart_column_for(5, 0).ck) && !pad_bonded(usart_column_for(8, 0).cts));
// Two columns land on the debug port's pads, and every default one does
// not.
static_assert(usart_column_on_debug_port(8, 1) && usart_column_on_debug_port(3, 2));
static_assert(!usart_column_on_debug_port(5, 0) && !usart_column_on_debug_port(8, 0) &&
              !usart_column_on_debug_port(1, 0));

// ---- the transports ---------------------------------------------------------

using Port5 = Uart<5, P>;
using Port6 = Uart<6, P, 32, 32, UartFormat{UartBits::eight, UartParity::even}>;
using Port7 = Uart<7, P, 64, 64, UartFormat{}, NoDmaEngine, NoDmaEngine, 1>;
using Port8 = Uart<8, P>;
/// The two engines of each port, on the slots the table gives it.
using Dma5 = Uart<5, P, 128, 128, UartFormat{}, DmaTxEngine<2, 4>, DmaRxEngine<2, 2>>;
using Dma8 = Uart<8, P, 128, 128, UartFormat{}, DmaTxEngine<2, 10>, DmaRxEngine<2, 11>>;
using Dma4 = Uart<4, P, 128, 128, UartFormat{}, DmaTxEngine<2, 5>, DmaRxEngine<2, 3>>;
/// A column on the debug port's pads: it compiles, and init() refuses it
/// while the two-wire port is the probe's.
using OnProbe = Uart<8, P, 64, 64, UartFormat{}, NoDmaEngine, NoDmaEngine, 1>;

static_assert(ByteSink<Port5> && ByteSource<Port8>);
static_assert(Port7::remap_code == 1 && Port7::pads.tx == Pad{'A', 6});
static_assert(Dma5::has_tx_engine && Dma5::has_rx_engine);

void uart_resources() {
    U5::bus_clock(true);
    U5::reset();
    (void)U5::remap(0);
    (void)U5::configure(UartFormat{}, 625);
    U5::enable(true);
    U5::transmitter(true);
    U5::receiver(true);
    (void)U5::mute_mode({.wake = MuteWake::idle_line});
    (void)U5::lin({});
    U5::lin_off();
    (void)U5::half_duplex(true);
    (void)U5::half_duplex(false);
    (void)U5::irda({.low_power = false, .prescaler = 1});
    U5::irda_off();
    // The full USART's modes are REFUSED on a UART at compile time (the
    // negs); the questions about them answer false and cost nothing.
    (void)U5::smartcard_enabled();
    (void)U5::synchronous_enabled();
    (void)U5::rts_enabled();
    (void)U5::cts_enabled();
    U5::interrupts(usart_rxneie | usart_tcie, true);
    U5::error_interrupt(true);
    U5::dma_transmit(true);
    U5::dma_receive(true);
    (void)U5::status();
    (void)U5::take_errors();
    U5::write_data(0x55);
    (void)U5::read_data();
    U5::bus_clock(false);

    U6::bus_clock(true);
    (void)U6::configure(UartFormat{UartBits::nine, UartParity::none}, 625);
    U6::write_word(0x1AA);
    (void)U6::read_word();
    U6::bus_clock(false);
    U7::bus_clock(true);
    (void)U7::remap(1);
    U7::bus_clock(false);
    U8::bus_clock(true);
    (void)U8::set_brr(625);
    (void)U8::actual_baud(72'000'000UL);
    U8::bus_clock(false);
    U4::bus_clock(true);
    (void)U4::remap(2);
    U4::bus_clock(false);
}

void uart_tasks() {
    constexpr SysClock clock;
    (void)Port5::init(clock, 115200);
    (void)Port6::init(clock, 9600);
    (void)Port7::init(clock, 57600);
    (void)Port8::init(clock, 921600);
    (void)Port5::write_byte('5');
    uint8_t b = 0;
    (void)Port8::read_byte(b);
    (void)Port6::parity_errors();
    (void)Port8::set_baud(SysClock::hz, 19200);
    Port8::rebase(SysClock::hz);
    (void)Dma5::init(clock, 115200);
    (void)Dma5::harvest();
    (void)Dma5::dma_isr();
    (void)Dma8::init(clock, 115200);
    (void)Dma8::harvest();
    (void)Dma4::init(clock, 921600);
    (void)Dma4::dma_faults();
    (void)OnProbe::init(clock, 115200);
    Port5::release();
    Port6::release();
    Port7::release();
    Port8::release();
    Dma5::release();
    Dma8::release();
    Dma4::release();
    OnProbe::release();
}

// The vectors this class's tail gives the four upper ports, bound the way
// an application binds them.
extern "C" BRIO_CH32_INTERRUPT void uart5_handler() { (void)Port5::isr(); }
extern "C" BRIO_CH32_INTERRUPT void uart6_handler() { (void)Port6::isr(); }
extern "C" BRIO_CH32_INTERRUPT void uart7_handler() { (void)Port7::isr(); }
extern "C" BRIO_CH32_INTERRUPT void uart8_handler() { (void)Port8::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel4_handler() { (void)Dma5::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel2_handler() { (void)Dma5::dma_isr(); }
