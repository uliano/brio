// THE IP STRATUM THROUGH THE HAZARD3 COMPILER, over a chip that is not a
// chip: brio/pl011/uart.hpp instantiated on brio/host/sim_pl011.hpp - two
// register blocks in RAM, a reset that memsets them, an interrupt
// controller that counts, pads that remember. The RP2040's fixture makes
// the same proof with one compiler; this one makes it with a SECOND
// ARCHITECTURE'S, which is the thing this target adds - a file that had
// smuggled in anything of a Cortex-M, of an NVIC or of a vendor header
// would fail here and nowhere else.
//
// The arithmetic below is judged at the rate THIS chip runs: the IP file's
// divisor is pure arithmetic and owes no chip an answer, so the two
// families ask it different questions and both must be right.
#include "host/sim_pl011.hpp"
#include "pl011/uart.hpp"

using namespace brio;

using Clock = SimPl011Clock<150'000'000>;

constexpr SimPl011Pins pins0{.tx = 0, .rx = 1};
constexpr SimPl011Pins pins1{.tx = 2, .rx = 3};

using Block = Pl011Uart<SimPl011, 0>;
using Plain = Pl011Transport<SimPl011, 0, pins0, 64, 256, SimNoEngine, SimNoEngine>;
using Engined = Pl011Transport<SimPl011, 1, pins1, 64, 64, SimPl011Engine<0>, SimPl011Engine<1>>;

static_assert(Pl011Chip<SimPl011>);
static_assert(Pl011ChipClock<SimPl011, Clock>);
static_assert(!Plain::has_tx_engine && !Plain::has_rx_engine);
static_assert(Engined::has_tx_engine && Engined::has_rx_engine);

void pl011_over_no_chip() {
    constexpr Clock clock;
    (void)Block::reset();
    (void)Block::line_control({.bits = UartBits::seven, .parity = UartParity::odd}, true);
    (void)Block::divisor(UartDivisor{81, 24});
    Block::fifo_levels(UartFifoLevel::quarter, UartFifoLevel::seven_eighths);
    Block::interrupts(UartInterrupt::errors, true);
    Block::dma_requests(true, true);

    (void)Plain::init(clock, 115200);
    (void)Plain::isr();
    (void)Plain::write_byte(0x55);
    (void)Plain::tx_idle();
    Plain::release();

    (void)Engined::init(clock, 3'000'000);
    (void)Engined::dma_isr();
    (void)Engined::harvest();
    (void)Engined::dma_faults();
    Engined::release();
}

// 150 MHz is this chip's clk_peri: 115200 wants 81 + 24/64, which lands
// at 115207 baud - a different pair from the RP2040's 67 + 52/64, off the
// same function, and 6 ppm from the asked-for rate.
static_assert(uart_divisor(150'000'000, 115200).value().integer == 81);
static_assert(uart_divisor(150'000'000, 115200).value().fraction == 24);
static_assert(uart_actual_baud(150'000'000, {81, 24}) == 115207);
// Some rates are EXACT at this clock and were not at the RP2040's: 9600,
// 1 Mbaud and 3 Mbaud all divide 150 MHz with nothing left over.
static_assert(uart_actual_baud(150'000'000, uart_divisor(150'000'000, 9600).value()) == 9600);
static_assert(uart_actual_baud(150'000'000, uart_divisor(150'000'000, 3'000'000).value()) ==
              3'000'000);
// clk_peri / 16 is the nominal ceiling and takes divisor 1 + 0/64; the
// generator's own reach runs a little past it, to where the rounding
// would make the integer part zero.
static_assert(uart_divisor(150'000'000, 9'375'000).value().integer == 1);
static_assert(uart_divisor(150'000'000, 9'375'000).value().fraction == 0);
static_assert(uart_divisor(150'000'000, 9'448'818).has_value());
static_assert(!uart_divisor(150'000'000, 9'448'819).has_value());
// And the floor is where the integer part would overflow sixteen bits.
static_assert(uart_divisor(150'000'000, 144).has_value());
static_assert(!uart_divisor(150'000'000, 143).has_value());
static_assert(uart_min_hz(3'000'000) == 48'000'000u);
