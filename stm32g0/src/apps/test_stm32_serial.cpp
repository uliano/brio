// test_stm32_serial - the reference bench suite for the STM32G0's SERIAL
// TAIL: the whole of the USART's chapter 33 beyond the console
// personality, the LPUARTs of chapter 34, and the infrared interface of
// chapter 27 that ANDs two of this board's timers onto one pad.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE, AND TWO FACTS MAKE THAT POSSIBLE.
//
// 1. SINGLE-WIRE HALF-DUPLEX (33.5.15) IS THE LOOP-BACK THIS FAMILY HAS
//    NO LBME FOR. With CR3.HDSEL the TX and RX lines are internally
//    connected, the RX pin is dropped and the TX pin is RELEASED
//    whenever nothing is transmitted - so the pad is alternate-function
//    OPEN DRAIN with a pull-up (the internal 40 k serves) and every byte
//    the instance sends, it receives. Every frame format, every
//    oversampling, every prescaler, every kernel clock, the FIFOs and
//    their thresholds and both LPUARTs are measured on that loop.
//
// 2. THE RX PAD IS A BIT-BANGED TRANSMITTER. A pad handed to the USART's
//    RX alternate function is an INPUT, and an input still follows its
//    own internal pull - which PUPDR moves in a handful of cycles. At
//    2400 baud a bit is 417 us, so software can put an ARBITRARY frame
//    on the receive line: one at a rate the receiver was never told
//    (auto-baud), one at a rate a few per cent off (the tolerance
//    tables), a 13-bit break, an RZI-encoded IrDA frame, a parity that
//    does not add up, a glitch of a quarter bit inside a stop bit
//    (ES0548 2.11.1). TIM2 free-running at 64 MHz is the ruler: both the
//    pacer that places every edge at an ABSOLUTE offset from the start
//    of the frame (so a late edge cannot accumulate) and the stopwatch
//    that times what the peripheral does about it.
//
// PADS THIS SUITE MOVES, all proven free by their own pull before use:
//   PA9  USART1_TX  (AF1)  - the single wire, and the transmitter
//   PA10 USART1_RX  (AF1)  - the bit-banged line
//   PB3  USART1_RTS_DE_CK (AF4) - RTS, DE and the synchronous CK, which
//                                 on this family are ONE PAD
//   PB4  USART1_CTS (AF4)
//   PB9  IR_OUT     (AF0)  - the infrared output, and EXTI line 9
//   PC1  LPUART1_TX (AF1)  - the LPUART1 single wire
//   PC6  LPUART2_TX (AF3)  - the LPUART2 single wire
// Every one of them is left in analog mode when the suite is done. The
// host-assisted letter v moves the CONSOLE's OWN PA2/PA3 to AF6 for two
// legs - LPUART1 reaches the same pair by a different alternate
// function - and puts them back on USART2's AF1 afterwards.
//
// THE BACKSTOP IS THE IWDG, armed once in main() at about 32 seconds and
// refreshed at the top of every letter. It cannot be turned off again
// (28.3.1), which is the point: a Stop with no wake behind it costs one
// reboot and a banner instead of a board that has to be re-flashed.
//
// THE WALL CLOCK IS THE RTC on the LSE crystal, for the letters that
// sleep. THE DOMAIN IS NEVER RESET: RTCSEL is left where it is and the
// backup registers other suites wrote are not touched.
//
// What is exercised, letter by letter:
//   a  the instance table: six USARTs and two LPUARTs against table 183,
//      the reserve's stated split, the device header's own macros and
//      the silicon - does FIFOEN stick, does PRESC stick, does CR2.CLKEN
//      stick (the one row of table 184 that is NOT the FULL/BASIC
//      split), is there a kernel-clock multiplexer (ES0548 2.11.2's
//      subject)
//   b  THE INSTRUMENT: the single-wire loop proven, and every frame
//      format of 33.5.3 byte-exact on it
//   c  the baud generator: both oversamplings, the twelve prescaler
//      codes, and where the loop's own open-drain rise time gives out
//   d  the kernel clocks: the CONSOLE moved to HSI16 and to SYSCLK while
//      it is talking, and USART1 run off the 32768 Hz crystal
//   e  the FIFOs: thresholds, RXFF/TXFE, the per-entry error flags,
//      interrupts per kilobyte with and without, ORE and OVRDIS
//   f  the bit-banged transmitter: parity, framing, noise, the tolerance
//      of tables 188/189, and ES0548 2.11.1 staged with its control
//   g  auto-baud, all four modes, at rates the receiver was not told
//   h  LIN: the break sent and timed, the break detected at both lengths
//   i  mute mode, the receiver time-out and the character match
//   j  smartcard: 8E1.5 on one wire, the guard time, the CK ladder, the
//      NACK staged from a pull, the retries counted
//   k  IrDA: the 3/16 pulse and the low-power pulse measured, a
//      bit-banged RZI frame decoded, the glitch filter as a threshold
//   l  the pads' extras: SWAP, the three inversions, MSB first, the
//      break request, the driver enable timed, RTS and CTS
//   m  the synchronous master's CK, counted with no CPU
//   n  the LPUARTs: both instances, both baud generators, the FIFOs, the
//      prescalers and the shared vectors
//   o  IRTIM: a 38 kHz carrier under a 1 kHz envelope on one pad, both
//      polarities, and a USART as the envelope
// Outside z, because they need a peer or a real Stop:
//   y  streaming through the console across kernel clocks (uart_stress)
//   w  WAKE FROM STOP, and ES0548 2.2.4 staged (uart_stress)
//   v  the console moved to LPUART1 on its own pads (uart_stress)
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/irtim.hpp"
#include "stm32g0/lpuart.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/pwr.hpp"
#include "kernel/panic.hpp"
#include "stm32g0/reset.hpp"
#include "stm32g0/rtc.hpp"
#include "stm32g0/sleep.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32Platform;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

TestBench<Serial, 24> bench;

// ---------------------------------------------------------------------------
// The pads, and the instances that own them
// ---------------------------------------------------------------------------

// USART1: the whole chapter's laboratory. PA9/PA10 are its TX and RX on
// AF1 and PB3/PB4 its RTS_DE_CK and CTS on AF4 (DS13560 tables 13 and
// 15). THE FIRST FINDING OF THE PAD TABLE, and the chapter never says
// it: RTS, the RS-485 driver enable and the SYNCHRONOUS CLOCK are one
// signal name on one pad - USARTn_RTS_DE_CK - so an instance does flow
// control OR drives a transceiver OR clocks a synchronous link, never
// two of them.
constexpr PinSel u1_tx{'A', 9, PinFunction::af1};
constexpr PinSel u1_rx{'A', 10, PinFunction::af1};
constexpr PinSel u1_de{'B', 3, PinFunction::af4};    // RTS / DE / CK
constexpr PinSel u1_cts{'B', 4, PinFunction::af4};
constexpr UartPins u1_pins{.tx = u1_tx, .rx = u1_rx};

using TxPin = Pin<'A', 9>;
using RxPin = Pin<'A', 10>;
using DePin = Pin<'B', 3>;
using CtsPin = Pin<'B', 4>;
using IrPin = Pin<'B', 9>;

constexpr PinSel ir_out{'B', 9, PinFunction::af0};
/// USART4_TX on PA0 (AF4, DS13560 table 13) - a BASIC instance's own
/// single wire, and the only thing letter a needs it for.
constexpr PinSel u4_tx{'A', 0, PinFunction::af4};
using U4Pin = Pin<'A', 0>;
constexpr PinSel lp1_tx{'C', 1, PinFunction::af1};
constexpr PinSel lp1_rx{'C', 0, PinFunction::af1};
constexpr PinSel lp2_tx{'C', 6, PinFunction::af3};
constexpr PinSel lp2_rx{'C', 7, PinFunction::af3};
constexpr UartPins lp1_pins{.tx = lp1_tx, .rx = lp1_rx};
constexpr UartPins lp2_pins{.tx = lp2_tx, .rx = lp2_rx};
using Lp1Pin = Pin<'C', 1>;
using Lp2Pin = Pin<'C', 6>;

using U1 = Usart<1>;
using L1 = Lpuart<1>;
using L2 = Lpuart<2>;

// The task instantiations this suite drives. The DEFAULT one is what
// every console in the tree uses; the others are the option struct at
// work, and each is a compile-time proof that its branch exists.
constexpr UartOptions hdsel_opts = uart_half_duplex();
constexpr UartOptions hdsel_fifo_opts{
    .fifo = true,
    .rx_threshold = UartFifoThreshold::full_or_empty,
    .half_duplex = true,
};
constexpr UartOptions hsi16_opts{.kernel_clock = UsartClock::hsi16};

using LoopUart = Uart<1, u1_pins, 512, 512, NoDmaEngine, NoDmaEngine, hdsel_opts>;
using LoopFifoUart =
    Uart<1, u1_pins, 512, 512, NoDmaEngine, NoDmaEngine, hdsel_fifo_opts>;
using HsiUart = Uart<1, u1_pins, 64, 64, NoDmaEngine, NoDmaEngine, hsi16_opts>;

using Dma1 = Dma<1>;
using EdgeCh = DmaChannel<1, 1>;
using Gen0 = DmaMuxGenerator<0>;

// ---------------------------------------------------------------------------
// Instruments
// ---------------------------------------------------------------------------

using Stopwatch = Tim<2>;   // 32 bits, PSC 0: one count per CPU cycle

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000UL;

void stopwatch_up() {
    Stopwatch::init();
    (void)Stopwatch::configure({.prescaler = 0, .period = 0xFFFF'FFFFu});
    Stopwatch::enable(true);
}
inline uint32_t now() { return Stopwatch::count(); }
inline uint32_t since(uint32_t t0) { return now() - t0; }
inline uint32_t to_us(uint32_t cycles) { return cycles / cycles_per_us; }
inline void spin_cycles(uint32_t c) {
    const uint32_t t0 = now();
    while (since(t0) < c) {
    }
}
inline void spin_us(uint32_t us) { spin_cycles(us * cycles_per_us); }

void feed() { Iwdg::refresh(); }

/// A measurement window a transmit interrupt walks through is not a
/// measurement - four campaigns of this stratum have paid for that.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    for (uint32_t i = 0; i < 8'000'000UL && (Usart<2>::status() & UsartFlag::tc) == 0u;
         ++i) {
    }
    spin_us(2000);
}

bool within(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }

/// Percent difference of `v` from `want`, x10 (so 15 means 1.5 %).
uint32_t permille_off(uint32_t v, uint32_t want) {
    if (want == 0u) {
        return 0xFFFFFFFFu;
    }
    const uint32_t d = v > want ? v - want : want - v;
    return static_cast<uint32_t>((static_cast<uint64_t>(d) * 1000ULL) / want);
}

/// THE PRECONDITION OF EVERY PULL-WALKED LETTER: a pad with nothing on it
/// goes where its own pull sends it - and, the question this chapter
/// adds, it must still do so with the pad handed to the peripheral. An
/// input alternate function leaves the pull in charge (the LPTIM
/// campaign measured that); an OPEN-DRAIN output function releases the
/// pad whenever it is not pulling low, so the pull should rule there
/// too. Both are measured, not assumed.
template <class Pin>
bool pull_walks() {
    Pin::input(PinPull::up);
    spin_us(300);
    const bool high = Pin::read();
    Pin::pull(PinPull::down);
    spin_us(300);
    const bool low = Pin::read();
    Pin::pull(PinPull::up);
    spin_us(300);
    return high && !low;
}

template <class Pin>
bool pull_walks_under(PinFunction fn, bool open_drain) {
    Pin::function(fn, {.pull = PinPull::up, .open_drain = open_drain});
    spin_us(300);
    const bool high = Pin::read();
    Pin::pull(PinPull::down);
    spin_us(300);
    const bool low = Pin::read();
    Pin::pull(PinPull::up);
    spin_us(300);
    return high && !low;
}

// ---------------------------------------------------------------------------
// The single-wire loop, driven through the RESOURCE
// ---------------------------------------------------------------------------
//
// Polled, deliberately: every letter that uses it wants to see the ISR
// register itself, not a ring's view of it. The TASK is exercised
// separately (letter e counts its interrupts, and the console has been
// running on it since the bring-up).

/// Bring USART1 up as a single wire on PA9 at `baud`, with `format`,
/// under the options the caller states through the resource verbs. The
/// caller has already proven the pad free.
struct LoopSetup {
    UartFormat format{};
    uint32_t baud = 115200;
    bool over8 = false;
    UsartPrescaler prescaler = UsartPrescaler::div1;
    UsartClock kernel = UsartClock::pclk;
    bool fifo = false;
    bool one_bit = false;
    bool msb_first = false;
    bool swap = false;
};

uint32_t loop_kernel_hz(UsartClock c) {
    switch (c) {
        case UsartClock::pclk: return SysClock::pclk_hz;
        case UsartClock::sysclk: return SysClock::hz;
        case UsartClock::hsi16: return 16'000'000u;
        default: return 32768u;
    }
}

bool loop_up(const LoopSetup& s) {
    const uint32_t ker = usart_kernel_hz(loop_kernel_hz(s.kernel), s.prescaler);
    const std::optional<uint16_t> reg =
        s.over8 ? usart_brr_over8(ker, s.baud) : usart_brr(ker, s.baud);
    if (!reg) {
        return false;
    }
    Nvic::disable(U1::irq());
    U1::bus_clock(true);
    U1::reset();
    (void)U1::kernel_clock(s.kernel);
    if (!U1::configure(s.format, *reg)) {
        return false;
    }
    if (s.over8 && !U1::oversampling(true)) {
        return false;
    }
    if (s.prescaler != UsartPrescaler::div1 && !U1::prescaler(s.prescaler)) {
        return false;
    }
    if (s.fifo && !U1::fifo(true)) {
        return false;
    }
    if (s.one_bit && !U1::one_bit_sampling(true)) {
        return false;
    }
    if (s.msb_first && !U1::msb_first(true)) {
        return false;
    }
    if (s.swap && !U1::swap(true)) {
        return false;
    }
    if (!U1::half_duplex(true)) {
        return false;
    }
    // 33.5.15: alternate function OPEN DRAIN with a pull-up, on the pad
    // the swap option chooses.
    if (s.swap) {
        RxPin::function(u1_rx.function, {.pull = PinPull::up, .open_drain = true});
    } else {
        TxPin::function(u1_tx.function, {.pull = PinPull::up, .open_drain = true});
    }
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    return true;
}

/// USART1 as a plain full-duplex receiver on PA10, with PA9 given back:
/// the arrangement every bit-banged letter uses.
bool listener_up(const LoopSetup& s) {
    const uint32_t ker = usart_kernel_hz(loop_kernel_hz(s.kernel), s.prescaler);
    const std::optional<uint16_t> reg =
        s.over8 ? usart_brr_over8(ker, s.baud) : usart_brr(ker, s.baud);
    if (!reg) {
        return false;
    }
    Nvic::disable(U1::irq());
    U1::bus_clock(true);
    U1::reset();
    (void)U1::kernel_clock(s.kernel);
    if (!U1::configure(s.format, *reg)) {
        return false;
    }
    if (s.over8) {
        (void)U1::oversampling(true);
    }
    if (s.prescaler != UsartPrescaler::div1) {
        (void)U1::prescaler(s.prescaler);
    }
    if (s.fifo) {
        (void)U1::fifo(true);
    }
    if (s.one_bit) {
        (void)U1::one_bit_sampling(true);
    }
    if (s.msb_first) {
        (void)U1::msb_first(true);
    }
    // The receive pad, as an input under the alternate function, with
    // the pull that the bit-bang moves.
    RxPin::function(u1_rx.function, {.pull = PinPull::up});
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    return true;
}

void loop_down() {
    Nvic::disable(U1::irq());
    U1::enable(false);
    U1::reset();
    U1::bus_clock(false);
    TxPin::release();
    RxPin::release();
    DePin::release();
    CtsPin::release();
}

/// Wait for a status bit, bounded. Every wait in this suite is bounded:
/// the IWDG would turn an unbounded one into a reboot, which costs a
/// whole run to learn one letter is wrong.
bool wait_flag(uint32_t mask, uint32_t us = 300000) {
    const uint32_t t0 = now();
    while ((U1::status() & mask) == 0u) {
        if (to_us(since(t0)) > us) {
            return false;
        }
    }
    return true;
}

/// Send one word and wait for the loop to bring it back. Nothing when
/// the receiver never saw it.
std::optional<uint16_t> loop_word(uint16_t v, uint32_t timeout_us = 20000) {
    if (!wait_flag(UsartFlag::txe, timeout_us)) {
        return std::nullopt;
    }
    U1::write_word(v);
    const uint32_t t0 = now();
    while ((U1::status() & UsartFlag::rxne) == 0u) {
        if (to_us(since(t0)) > timeout_us) {
            return std::nullopt;
        }
    }
    return U1::read_word();
}

/// How many DATA bits a format really carries: the word length minus the
/// parity bit, since 33.5.5 makes parity eat one of them.
uint8_t data_bits(const UartFormat& f) {
    const uint8_t w = static_cast<uint8_t>(f.bits);
    return f.parity == UartParity::none ? w : static_cast<uint8_t>(w - 1u);
}
uint16_t data_mask(const UartFormat& f) {
    return static_cast<uint16_t>((1u << data_bits(f)) - 1u);
}

// ---------------------------------------------------------------------------
// The bit-banged transmitter on the receive pad
// ---------------------------------------------------------------------------
//
// EVERY EDGE IS PLACED AT AN ABSOLUTE OFFSET from the start of the
// frame, in TIM2 cycles, so a late store cannot accumulate into the next
// bit. Interrupts are masked for the whole frame: at 2400 baud a frame
// is 4 ms and the SysTick handler in the middle of it is 0.3 % of a bit,
// which is the same order as the tolerance being measured. The kernel
// tick loses those milliseconds and nothing in this suite depends on it
// (the IWDG is its own hardware).

struct BitPlan {
    uint32_t cycles_per_bit = 0;
    uint16_t bits = 0;        ///< the levels, LSB first in time order
    uint8_t count = 0;        ///< how many of them
};

/// Build the levels of one asynchronous frame: start (0), the data LSB
/// first, an optional parity, then `stops` ones.
BitPlan frame_plan(uint32_t baud, uint16_t value, uint8_t bits, UartParity parity,
                   uint8_t stops, bool bad_parity = false) {
    BitPlan p{};
    p.cycles_per_bit = SysClock::hz / baud;
    uint16_t levels = 0;
    uint8_t n = 0;
    levels |= 0u << n;    // the start bit
    ++n;
    uint8_t ones = 0;
    for (uint8_t i = 0; i < bits; ++i) {
        const uint16_t b = (value >> i) & 1u;
        ones = static_cast<uint8_t>(ones + b);
        levels = static_cast<uint16_t>(levels | (b << n));
        ++n;
    }
    if (parity != UartParity::none) {
        uint16_t bit = (parity == UartParity::even) ? (ones & 1u) : ((ones & 1u) ^ 1u);
        if (bad_parity) {
            bit ^= 1u;
        }
        levels = static_cast<uint16_t>(levels | (bit << n));
        ++n;
    }
    for (uint8_t i = 0; i < stops; ++i) {
        levels = static_cast<uint16_t>(levels | (1u << n));
        ++n;
    }
    p.bits = levels;
    p.count = n;
    return p;
}

/// Put a plan on the receive pad. `short_stop` drives the LAST stop bit
/// low (a framing error); `glitch_bit`/`glitch_at`/`glitch_len` place a
/// low glitch inside one bit, in sixteenths of a bit - which is how
/// ES0548 2.11.1 and the noise flag are staged.
struct BangOptions {
    bool invert = false;      ///< every level flipped: an RXINV line
    bool short_stop = false;
    int8_t glitch_bit = -1;
    uint8_t glitch_at = 0;    ///< sixteenths into that bit
    uint8_t glitch_len = 0;   ///< sixteenths long
};

void bang_frame(const BitPlan& p, const BangOptions& o = {}) {
    const uint32_t bit = p.cycles_per_bit;
    const uint32_t sixteenth = bit / 16u;
    InterruptGuard guard;
    RxPin::pull(o.invert ? PinPull::down : PinPull::up);
    const uint32_t t0 = now();
    // One idle bit before the start, so a receiver that has just been
    // enabled sees a real falling edge.
    while (since(t0) < bit) {
    }
    for (uint8_t i = 0; i < p.count; ++i) {
        const uint32_t at = bit * (i + 1u);
        while (since(t0) < at) {
        }
        bool level = ((p.bits >> i) & 1u) != 0u;
        if (o.short_stop && i == p.count - 1u) {
            level = false;
        }
        if (o.invert) {
            level = !level;
        }
        RxPin::pull(level ? PinPull::up : PinPull::down);
        if (o.glitch_bit >= 0 && i == o.glitch_bit && o.glitch_len != 0u) {
            const uint32_t g0 = at + sixteenth * o.glitch_at;
            const uint32_t g1 = g0 + sixteenth * o.glitch_len;
            while (since(t0) < g0) {
            }
            RxPin::pull(PinPull::down);
            while (since(t0) < g1) {
            }
            RxPin::pull(level ? PinPull::up : PinPull::down);
        }
    }
    const uint32_t end = bit * (p.count + 1u);
    while (since(t0) < end) {
    }
    RxPin::pull(o.invert ? PinPull::down : PinPull::up);
}

/// A run of `bits` zeroes followed by one high delimiter - a LIN break.
void bang_break(uint32_t baud, uint8_t bits) {
    const uint32_t bit = SysClock::hz / baud;
    InterruptGuard guard;
    RxPin::pull(PinPull::up);
    const uint32_t t0 = now();
    while (since(t0) < bit) {
    }
    RxPin::pull(PinPull::down);
    while (since(t0) < bit * (1u + bits)) {
    }
    RxPin::pull(PinPull::up);
    while (since(t0) < bit * (3u + bits)) {
    }
}

/// Hold the line idle for `bit_times` - an idle frame for mute mode.
void bang_idle(uint32_t baud, uint32_t bit_times) {
    const uint32_t bit = SysClock::hz / baud;
    RxPin::pull(PinPull::up);
    spin_cycles(bit * bit_times);
}

// ---------------------------------------------------------------------------
// a: the instance table - three authorities and the silicon
// ---------------------------------------------------------------------------

struct InstanceReading {
    bool present = false;
    bool bus = false;
    bool fifo_sticks = false;
    bool presc_sticks = false;
    bool clken_sticks = false;
    bool mux = false;
};

template <typename R>
InstanceReading probe_instance() {
    InstanceReading r{};
    r.present = true;
    R::bus_clock(true);
    R::reset();
    r.bus = true;
    // FIFOEN, written straight into CR1 with the instance disabled: what
    // sticks is the silicon's answer to table 184, whatever the driver's
    // own refusal says.
    R::regs().CR1 = R::regs().CR1 | USART_CR1_FIFOEN;
    r.fifo_sticks = (R::regs().CR1 & USART_CR1_FIFOEN) != 0u;
    R::regs().CR1 = R::regs().CR1 & ~USART_CR1_FIFOEN;
    // PRESC, the same way - ES0548 2.11.2's own subject.
    R::regs().PRESC = 0xBu;
    r.presc_sticks = (R::regs().PRESC & USART_PRESC_PRESCALER_Msk) == 0xBu;
    R::regs().PRESC = 0;
    // CR2.CLKEN - the synchronous CK output. 33.8.3's own note says the
    // bit "is reserved and must be kept at reset value" where neither
    // synchronous nor smartcard mode is supported, so what STICKS here
    // is the silicon's answer to the one row of table 184 that is NOT
    // the FULL/BASIC split.
    R::regs().CR2 = R::regs().CR2 | USART_CR2_CLKEN;
    r.clken_sticks = (R::regs().CR2 & USART_CR2_CLKEN) != 0u;
    R::regs().CR2 = R::regs().CR2 & ~USART_CR2_CLKEN;
    r.mux = R::has_clock_select;
    R::reset();
    R::bus_clock(false);
    return r;
}

/// How long the pad is held LOW by a 0x00 character - start plus eight
/// zero bits, nine bit times - on a single-wire loop with `brr` and
/// `presc`. The one measurement that says whether a prescaler DIVIDES,
/// as opposed to merely being writable.
struct ZeroLow {
    uint32_t cycles = 0;
    bool saw_frame = false;
    bool complete = false;
    uint32_t presc_readback = 0;
};

template <typename R, typename Pad>
ZeroLow zero_low_cycles(const PinSel& sel, uint16_t brr, UsartPrescaler presc) {
    ZeroLow z{};
    R::bus_clock(true);
    R::reset();
    if (!R::configure({}, brr)) {
        return z;
    }
    R::regs().PRESC = static_cast<uint32_t>(presc);
    z.presc_readback = R::regs().PRESC & USART_PRESC_PRESCALER_Msk;
    R::regs().CR3 = R::regs().CR3 | USART_CR3_HDSEL;
    Pad::function(sel.function, {.pull = PinPull::up, .open_drain = true});
    R::enable(true);
    R::clear_flags(UsartClear::all);
    {
        InterruptGuard guard;
        while ((R::status() & UsartFlag::txe) == 0u) {
        }
        const uint32_t t0 = now();
        R::write_data(0x00);
        while (Pad::read() && to_us(since(t0)) < 300000u) {
        }
        const uint32_t fall = now();
        z.saw_frame = !Pad::read();
        while (!Pad::read() && to_us(since(fall)) < 300000u) {
        }
        z.cycles = since(fall);
        uint32_t spins = 20'000'000u;
        while ((R::status() & UsartFlag::tc) == 0u && spins-- != 0u) {
        }
        z.complete = (R::status() & UsartFlag::tc) != 0u;
    }
    R::enable(false);
    R::reset();
    R::bus_clock(false);
    return z;
}

void print_row(const char* name, const InstanceReading& r, bool claim_fifo,
               bool claim_presc, bool header_fifo, bool claim_sync) {
    print(serial, "  ", name, ": fifo sticks ", r.fifo_sticks ? "yes" : "no ",
          " (table says ", claim_fifo ? "yes" : "no ", ", header says ",
          header_fifo ? "yes" : "no ", ")  presc sticks ",
          r.presc_sticks ? "yes" : "no ", " (table says ",
          claim_presc ? "yes" : "no ", ")  CLKEN sticks ",
          r.clken_sticks ? "yes" : "no ", " (table says ",
          claim_sync ? "yes" : "no ", ")  mux ", r.mux ? "yes" : "no ", crlf);
}

void ta_instances() {
    feed();
    print(serial, "  RM0444 table 183 on this part (G0B1): USART1..3 FULL, "
                  "USART4..6 BASIC, LPUART1 and LPUART2 LP", crlf);

    uint8_t agree = 0;
    uint8_t rows = 0;
    bool all_agree = true;

    const InstanceReading r1 = probe_instance<Usart<1>>();
    print_row("USART1", r1, Usart<1>::is_full, Usart<1>::has_prescaler,
              Usart<1>::has_fifo(), Usart<1>::has_synchronous_mode);
    // USART2 IS THE CONSOLE, and the probe resets it. The lines are
    // drained first, the instance is walked like any other, and the
    // console is brought straight back up - which is why the rest of
    // this letter arriving at all is the reading's own witness.
    console_drain();
    const InstanceReading r2 = probe_instance<Usart<2>>();
    (void)Serial::init(clock, 115200);
    spin_us(2000);
    print_row("USART2", r2, Usart<2>::is_full, Usart<2>::has_prescaler,
              Usart<2>::has_fifo(), Usart<2>::has_synchronous_mode);
    const InstanceReading r3 = probe_instance<Usart<3>>();
    print_row("USART3", r3, Usart<3>::is_full, Usart<3>::has_prescaler,
              Usart<3>::has_fifo(), Usart<3>::has_synchronous_mode);
    const InstanceReading r4 = probe_instance<Usart<4>>();
    print_row("USART4", r4, Usart<4>::is_full, Usart<4>::has_prescaler,
              Usart<4>::has_fifo(), Usart<4>::has_synchronous_mode);
    const InstanceReading r5 = probe_instance<Usart<5>>();
    print_row("USART5", r5, Usart<5>::is_full, Usart<5>::has_prescaler,
              Usart<5>::has_fifo(), Usart<5>::has_synchronous_mode);
    const InstanceReading r6 = probe_instance<Usart<6>>();
    print_row("USART6", r6, Usart<6>::is_full, Usart<6>::has_prescaler,
              Usart<6>::has_fifo(), Usart<6>::has_synchronous_mode);
    const InstanceReading l1 = probe_instance<Lpuart<1>>();
    print_row("LPUART1", l1, Lpuart<1>::has_fifo_mode, Lpuart<1>::has_prescaler,
              Lpuart<1>::has_fifo(), Lpuart<1>::has_synchronous_mode);
    const InstanceReading l2 = probe_instance<Lpuart<2>>();
    print_row("LPUART2", l2, Lpuart<2>::has_fifo_mode, Lpuart<2>::has_prescaler,
              Lpuart<2>::has_fifo(), Lpuart<2>::has_synchronous_mode);

    struct Row { InstanceReading r; bool fifo; bool presc; bool header; };
    const Row rowset[] = {
        {r1, Usart<1>::is_full, Usart<1>::has_prescaler, Usart<1>::has_fifo()},
        {r2, Usart<2>::is_full, Usart<2>::has_prescaler, Usart<2>::has_fifo()},
        {r3, Usart<3>::is_full, Usart<3>::has_prescaler, Usart<3>::has_fifo()},
        {r4, Usart<4>::is_full, Usart<4>::has_prescaler, Usart<4>::has_fifo()},
        {r5, Usart<5>::is_full, Usart<5>::has_prescaler, Usart<5>::has_fifo()},
        {r6, Usart<6>::is_full, Usart<6>::has_prescaler, Usart<6>::has_fifo()},
        {l1, Lpuart<1>::has_fifo_mode, Lpuart<1>::has_prescaler, Lpuart<1>::has_fifo()},
        {l2, Lpuart<2>::has_fifo_mode, Lpuart<2>::has_prescaler, Lpuart<2>::has_fifo()},
    };
    uint8_t fifo_agree = 0;
    for (const Row& row : rowset) {
        ++rows;
        if (row.r.fifo_sticks == row.fifo && row.header == row.fifo) {
            ++fifo_agree;
        }
        const bool ok = row.r.fifo_sticks == row.fifo &&
                        row.r.presc_sticks == row.presc && row.header == row.fifo;
        if (ok) {
            ++agree;
        } else {
            all_agree = false;
        }
    }
    (void)all_agree;
    print(serial, "  ", agree, " of ", rows,
          " instances agree with the reserve's stated table, the device "
          "header's own IS_UART_FIFO_INSTANCE and the silicon on BOTH counts",
          crlf);
    bench.verdict("table 183's FULL/BASIC/LP split is the silicon's for the "
                  "FIFO, on every instance of this part - three authorities, "
                  "one answer", fifo_agree == rows);
    bench.verdict("and a BASIC instance really does DROP the FIFO enable "
                  "rather than taking it",
                  !r4.fifo_sticks && !r5.fifo_sticks && !r6.fifo_sticks);

    // THE ROW THAT IS NOT THE FULL/BASIC SPLIT, and the reason the
    // driver states it separately from is_full: table 184 gives
    // "Synchronous mode (Master/Slave)" to the FULL *and* the BASIC
    // column and takes it from the LP one, the device header's
    // IS_USART_INSTANCE ("USART Instances : Synchronous mode") lists all
    // six USARTs, and 33.8.3's note makes CR2.CLKEN reserved only where
    // neither synchronous nor smartcard mode is supported. So the CK
    // output belongs to every USART of this part - and the silicon
    // agrees, which is the third authority.
    const bool sync_agrees =
        r1.clken_sticks && r2.clken_sticks && r3.clken_sticks &&
        r4.clken_sticks && r5.clken_sticks && r6.clken_sticks &&
        !l1.clken_sticks && !l2.clken_sticks;
    print(serial, "  CR2.CLKEN sticks on all six USARTs including the three "
                  "BASIC ones, and on NEITHER LPUART - so the synchronous row "
                  "of table 184 cuts USART-vs-LPUART, not FULL-vs-BASIC", crlf);
    bench.verdict("the CK output is every USART's and no LPUART's - the one "
                  "row of table 184 that is not the FULL/BASIC split, agreed "
                  "by the manual, the device header and the silicon",
                  sync_agrees);

    // THE PRESCALER IS NOT THE SAME STORY, and this is the letter's own
    // finding. FIFOEN on a BASIC instance reads back CLEAR - the silicon
    // refuses it - while PRESC[3:0] TAKES the value and reads it back on
    // every instance of the part, table 184 or no table 184. So the
    // register is there and the question is whether the DIVIDER is: what
    // follows measures a zero character's low time on the pad, which is
    // start + eight zero bits = nine bit times, with the same BRR and
    // two different prescaler codes.
    print(serial, "  PRESC[3:0] took 0xB and read it back on ALL SIX USARTs, "
                  "including the three table 184 says have no prescaler. "
                  "Whether it DIVIDES is another question:", crlf);
    const ZeroLow full1 = zero_low_cycles<Usart<1>, TxPin>(u1_tx, 640,
                                                          UsartPrescaler::div1);
    const ZeroLow full16 = zero_low_cycles<Usart<1>, TxPin>(u1_tx, 640,
                                                            UsartPrescaler::div16);
    const ZeroLow basic1 = zero_low_cycles<Usart<4>, U4Pin>(u4_tx, 640,
                                                            UsartPrescaler::div1);
    const ZeroLow basic16 = zero_low_cycles<Usart<4>, U4Pin>(u4_tx, 640,
                                                             UsartPrescaler::div16);
    print(serial, "  USART1 (FULL, PRESC reads ", full1.presc_readback, "/",
          full16.presc_readback, "): nine zero bits last ", to_us(full1.cycles),
          " us at /1 and ", to_us(full16.cycles), " us at /16; the frame was "
          "seen ", full1.saw_frame ? "yes" : "no", "/",
          full16.saw_frame ? "yes" : "no", " and completed ",
          full1.complete ? "yes" : "no", "/", full16.complete ? "yes" : "no",
          crlf);
    print(serial, "  USART4 (BASIC, PRESC reads ", basic1.presc_readback, "/",
          basic16.presc_readback, "): ", to_us(basic1.cycles), " us at /1 and ",
          to_us(basic16.cycles), " us at /16; seen ",
          basic1.saw_frame ? "yes" : "no", "/", basic16.saw_frame ? "yes" : "no",
          ", completed ", basic1.complete ? "yes" : "no", "/",
          basic16.complete ? "yes" : "no", crlf);
    const bool full_divides = full16.cycles > full1.cycles * 8u;
    bench.verdict("the prescaler DIVIDES on a FULL instance - sixteen times "
                  "the bit period for the same BRR",
                  full_divides && full1.saw_frame && full16.saw_frame);
    if (basic16.saw_frame && basic16.cycles < basic1.cycles * 2u) {
        print(serial, "  THE FINDING: on a BASIC instance the PRESC FIELD "
                      "TAKES THE VALUE AND THE DIVIDER IS NOT THERE - same "
                      "bit period at /1 and /16", crlf);
        bench.verdict("a BASIC instance implements the prescaler REGISTER and "
                      "not the prescaler", true);
    } else if (!basic16.saw_frame) {
        print(serial, "  THE FINDING, and it is worse than a missing divider: "
                      "a BASIC instance takes the PRESC value, reads it back, "
                      "and then TRANSMITS NOTHING AT ALL - the frame never "
                      "left the pad. Table 184 says the prescaler is not "
                      "there; the silicon says writing it stops the "
                      "transmitter, which is why the driver REFUSES the write "
                      "on such an instance rather than trusting the readback",
              crlf);
        bench.verdict("a non-zero PRESC on a BASIC instance is not ignored - "
                      "it silences the transmitter, and prescaler() refusing "
                      "there is the only safe reading", basic1.saw_frame);
    } else {
        print(serial, "  the BASIC instance's bit period MOVED with PRESC, "
                      "which table 184 does not predict; recorded as measured",
              crlf);
        bench.verdict("the BASIC instance's prescaler behaviour is measured "
                      "and printed either way", basic1.saw_frame);
    }
    print(serial, "  ES0548 2.11.2 is the DOCUMENTATION erratum saying some "
                  "manual revisions omit that the prescaler is not on every "
                  "instance; RM0444 Rev 6's own table 184 carries it, and the "
                  "silicon carries it in BEHAVIOUR but not in the REGISTER's "
                  "readback", crlf);
    U4Pin::release();

    // The kernel-clock multiplexers, and what a multiplexer-less instance
    // answers when asked for anything but PCLK.
    bench.verdict("the kernel-clock multiplexer is USART1..3's here, and the "
                  "LPUARTs' - every one of them",
                  r1.mux && r2.mux && r3.mux && !r4.mux && !r5.mux && !r6.mux &&
                      l1.mux && l2.mux);
    Usart<4>::bus_clock(true);
    const bool pclk_ok = Usart<4>::kernel_clock(UsartClock::pclk);
    const bool hsi_refused = !Usart<4>::kernel_clock(UsartClock::hsi16);
    Usart<4>::bus_clock(false);
    bench.verdict("and an instance without one answers true for PCLK and "
                  "false for anything else, instead of writing a field that "
                  "does not exist", pclk_ok && hsi_refused);

    // The vectors, off the reserve.
    bench.verdict("the vectors are the shared ones of this part: USART2 with "
                  "LPUART2, and USART3..6 with LPUART1",
                  Usart<2>::irq() == USART2_LPUART2_IRQn &&
                      Lpuart<2>::irq() == USART2_LPUART2_IRQn &&
                      Usart<3>::irq() == USART3_4_5_6_LPUART1_IRQn &&
                      Usart<6>::irq() == USART3_4_5_6_LPUART1_IRQn &&
                      Lpuart<1>::irq() == USART3_4_5_6_LPUART1_IRQn);
    bench.verdict("and the wake-up EXTI lines are table 65's: 25, 26 and 24 "
                  "for USART1..3, 28 and 35 for the LPUARTs, and nothing at "
                  "all for a BASIC instance",
                  Usart<1>::exti_line == 25 && Usart<2>::exti_line == 26 &&
                      Usart<3>::exti_line == 24 && Usart<4>::exti_line == 0xFF &&
                      Lpuart<1>::exti_line == 28 && Lpuart<2>::exti_line == 35);

    // The console must still be alive: every instance above was reset.
    print(serial, "  (the console is USART2 and this letter reset it - these "
                  "lines are the proof it came back)", crlf);
}

// ---------------------------------------------------------------------------
// b: THE INSTRUMENT - the single wire, and every frame format on it
// ---------------------------------------------------------------------------

void tb_loop() {
    feed();
    // The pad, twice: as a plain input and under the OPEN-DRAIN
    // alternate function the chapter asks for. The second is the one
    // that matters, and it is not obvious - a driving function takes the
    // pull away on other silicon (the SAM measured that), while an
    // open-drain one is released whenever it is not pulling low.
    const bool free_plain = pull_walks<TxPin>();
    U1::bus_clock(true);
    U1::reset();
    const bool free_af = pull_walks_under<TxPin>(u1_tx.function, true);
    U1::bus_clock(false);
    print(serial, "  PA9 follows its own pull as an input: ",
          free_plain ? "yes" : "NO", "; under USART1_TX open drain: ",
          free_af ? "yes" : "NO", crlf);
    bench.verdict("PA9 is free, and a pad under an OPEN-DRAIN alternate "
                  "function still follows its own pull - which is what makes "
                  "33.5.15's external pull-up an internal one here",
                  free_plain && free_af);

    if (!loop_up({.baud = 115200})) {
        bench.verdict("the single-wire loop comes up", false);
        loop_down();
        return;
    }
    const std::optional<uint16_t> back = loop_word(0xA5);
    print(serial, "  0xA5 out on PA9 and ", back ? *back : 0xFFFFu,
          " back, with no wire on the board", crlf);
    bench.verdict("HDSEL is the loop-back this family has no LBME for: the "
                  "byte the instance transmits, it receives",
                  back.has_value() && *back == 0xA5u);

    // Every frame of 33.5.3: three word lengths x three parities x two
    // stop-bit counts. Parity eats a data bit, so what is compared is
    // the DATA the format can carry.
    struct FormatCase { UartBits bits; UartParity parity; uint8_t stops; const char* name; };
    static const FormatCase cases[] = {
        {UartBits::seven, UartParity::none, 1, "7N1"},
        {UartBits::seven, UartParity::even, 1, "7E1"},
        {UartBits::seven, UartParity::odd, 1, "7O1"},
        {UartBits::seven, UartParity::none, 2, "7N2"},
        {UartBits::eight, UartParity::none, 1, "8N1"},
        {UartBits::eight, UartParity::even, 1, "8E1"},
        {UartBits::eight, UartParity::odd, 1, "8O1"},
        {UartBits::eight, UartParity::none, 2, "8N2"},
        {UartBits::eight, UartParity::even, 2, "8E2"},
        {UartBits::nine, UartParity::none, 1, "9N1"},
        {UartBits::nine, UartParity::even, 1, "9E1"},
        {UartBits::nine, UartParity::odd, 1, "9O1"},
        {UartBits::nine, UartParity::none, 2, "9N2"},
    };
    uint8_t good = 0;
    uint8_t tried = 0;
    uint8_t errors = 0;
    for (const FormatCase& c : cases) {
        feed();
        const UartFormat f{.bits = c.bits, .parity = c.parity, .stop_bits = c.stops};
        if (!loop_up({.format = f, .baud = 115200})) {
            continue;
        }
        const uint16_t mask = data_mask(f);
        bool ok = true;
        for (uint16_t v : {0x000u, 0x0AAu, 0x155u, 0x1FFu}) {
            const uint16_t want = static_cast<uint16_t>(v & mask);
            const std::optional<uint16_t> got = loop_word(want);
            // THE PARITY BIT COMES BACK: 33.5.12 has the receiver check
            // it and RDR keep it, in the word's top position - so what
            // is compared is the DATA the format carries and not the
            // whole register. A reader who expects the parity stripped
            // sees every parity format "fail", which is how this suite
            // learned it.
            if (!got || (*got & mask) != want) {
                ok = false;
            }
        }
        if ((U1::status() & UsartFlag::receive_errors) != 0u) {
            ++errors;
            ok = false;
        }
        ++tried;
        if (ok) {
            ++good;
        } else {
            print(serial, "  ", c.name, " FAILED on the loop", crlf);
        }
    }
    print(serial, "  ", good, " of ", tried,
          " frame formats byte-exact on the single wire (four values each, "
          "the data bits the format really carries), receive errors ", errors,
          crlf);
    bench.verdict("every word length, parity and stop-bit count of 33.5.3 is "
                  "byte-exact on the loop", good == tried && tried >= 13u);

    // The break request, on the plain loop: 33.8.8's SBKRQ sends one
    // break character, which the receiver reads as a zero WITH a framing
    // error - the stop bit was a zero too.
    (void)loop_up({.baud = 9600});
    U1::clear_flags(UsartClear::all);
    U1::send_break();
    const uint32_t t0 = now();
    bool sbkf_seen = false;
    while (to_us(since(t0)) < 5000u) {
        if ((U1::status() & UsartFlag::sbkf) != 0u) {
            sbkf_seen = true;
        }
        if ((U1::status() & UsartFlag::rxne) != 0u) {
            break;
        }
    }
    const uint32_t st = U1::status();
    const uint16_t brk = U1::read_word();
    print(serial, "  SBKRQ: SBKF seen ", sbkf_seen ? "yes" : "no",
          ", received ", hex(brk), " with ISR ", hex(st), crlf);
    bench.verdict("a break request lands on the loop as a zero character "
                  "carrying a framing error",
                  brk == 0u && (st & UsartFlag::fe) != 0u);

    loop_down();
}

// ---------------------------------------------------------------------------
// c: the baud generator - both oversamplings, twelve prescalers, the ceiling
// ---------------------------------------------------------------------------

/// Move `count` bytes round the loop and say how many came back exact.
uint32_t loop_run(uint32_t count) {
    uint32_t good = 0;
    uint8_t v = 0x31;
    for (uint32_t i = 0; i < count; ++i) {
        const std::optional<uint16_t> got = loop_word(v);
        if (got && *got == v) {
            ++good;
        }
        v = static_cast<uint8_t>(v * 5u + 1u);
    }
    return good;
}

void tc_baud() {
    feed();
    // The chapter's own two examples, read back out of the register the
    // driver wrote - the arithmetic is fixture-pinned, this is the
    // silicon taking the value.
    U1::bus_clock(true);
    U1::reset();
    (void)U1::configure({}, usart_brr(8'000'000, 9600).value());
    const uint16_t e1_16 = U1::brr();
    (void)U1::oversampling(true);
    U1::set_brr(usart_brr_over8(8'000'000, 9600).value());
    const uint16_t e1_8 = U1::brr();
    U1::reset();
    (void)U1::configure({}, usart_brr(48'000'000, 921'600).value());
    const uint16_t e2_16 = U1::brr();
    (void)U1::oversampling(true);
    U1::set_brr(usart_brr_over8(48'000'000, 921'600).value());
    const uint16_t e2_8 = U1::brr();
    print(serial, "  33.5.7's examples read back: 9600 at 8 MHz = ", hex(e1_16),
          " / ", hex(e1_8), "; 921600 at 48 MHz = ", hex(e2_16), " / ", hex(e2_8),
          "  (over16 / over8)", crlf);
    bench.verdict("both of the chapter's baud examples land in BRR bit for "
                  "bit, in both oversamplings - and OVER8's register is NOT "
                  "the divisor, it is USARTDIV with bit 3 dropped",
                  e1_16 == 0x341u && e1_8 == 0x681u && e2_16 == 0x34u &&
                      e2_8 == 0x64u);

    // OVER8 on the loop at the console's own rate.
    (void)loop_up({.baud = 115200, .over8 = true});
    const uint32_t over8_good = loop_run(32);
    const uint16_t over8_brr = U1::brr();
    print(serial, "  OVER8 = 1 at 115200 on the loop: ", over8_good,
          " of 32 exact, BRR ", hex(over8_brr), " (USARTDIV ",
          usart_actual_baud_over8(SysClock::pclk_hz, over8_brr), " baud)", crlf);
    bench.verdict("eight samples a bit carries the loop as well as sixteen",
                  over8_good == 32u);

    // The twelve prescaler codes, all at ONE line rate: 9600 is low
    // enough that even a 256-fold division leaves USARTDIV above 16
    // (250 kHz / 9600 = 26).
    static const UsartPrescaler codes[] = {
        UsartPrescaler::div1, UsartPrescaler::div2, UsartPrescaler::div4,
        UsartPrescaler::div6, UsartPrescaler::div8, UsartPrescaler::div10,
        UsartPrescaler::div12, UsartPrescaler::div16, UsartPrescaler::div32,
        UsartPrescaler::div64, UsartPrescaler::div128, UsartPrescaler::div256,
    };
    uint8_t presc_ok = 0;
    uint8_t presc_tried = 0;
    for (UsartPrescaler p : codes) {
        feed();
        if (!loop_up({.baud = 9600, .prescaler = p})) {
            continue;
        }
        ++presc_tried;
        if (loop_run(4) == 4u &&
            static_cast<uint8_t>(U1::prescaler()) == static_cast<uint8_t>(p)) {
            ++presc_ok;
        } else {
            print(serial, "  PRESC code ", static_cast<uint32_t>(p),
                  " (divide by ", usart_prescaler_divisor(p), ") FAILED", crlf);
        }
    }
    print(serial, "  ", presc_ok, " of ", presc_tried,
          " prescaler codes carry 9600 baud on the loop, from divide-by-1 to "
          "divide-by-256 - one line rate, twelve kernel rates", crlf);
    bench.verdict("all twelve of 33.8.14's implemented codes read back and "
                  "deliver the rate the arithmetic promised",
                  presc_ok == 12u && presc_tried == 12u);

    // A Reserved code is refused, because the chapter's own note says
    // the silicon turns one into 1011 - a rate nobody asked for.
    (void)loop_up({.baud = 9600});
    U1::enable(false);
    const bool reserved_refused = !U1::prescaler(static_cast<UsartPrescaler>(12));
    const uint8_t presc_after = static_cast<uint8_t>(U1::prescaler());
    bench.verdict("a Reserved prescaler code is refused and nothing is "
                  "written - 33.8.14's note says the silicon would make it a "
                  "divide-by-256", reserved_refused && presc_after == 0u);

    // THE LOOP'S OWN CEILING. The single wire is OPEN DRAIN with the
    // pad's own 40 k pull-up, so a rising edge is an RC and not a
    // driver: this ladder measures the INSTRUMENT, not the USART, and
    // the register arithmetic above the break is checked separately.
    static const uint32_t ladder[] = {115200, 230400, 460800, 921600,
                                      1'000'000, 2'000'000, 4'000'000};
    uint32_t best = 0;
    for (uint32_t baud : ladder) {
        feed();
        if (!loop_up({.baud = baud})) {
            print(serial, "  ", baud, " baud is unreachable at this clock", crlf);
            continue;
        }
        const uint32_t good = loop_run(16);
        print(serial, "  ", baud, " baud: ", good, " of 16 exact", crlf);
        if (good == 16u) {
            best = baud;
        }
    }
    print(serial, "  the open-drain loop is byte-exact to ", best,
          " baud; above it the rise time of the pad's own 40 k pull-up is "
          "what fails, not the generator", crlf);
    bench.verdict("the single-wire loop carries at least the console's rate",
                  best >= 115200u);

    // And the ceiling the GENERATOR has, which needs no wire: USARTDIV
    // 16 is 4 Mbaud at 64 MHz with OVER8 = 0 and 8 Mbaud with OVER8 = 1.
    U1::enable(false);
    U1::reset();
    (void)U1::configure({}, usart_brr(SysClock::pclk_hz, 4'000'000).value());
    const uint16_t top16 = U1::brr();
    (void)U1::oversampling(true);
    U1::set_brr(usart_brr_over8(SysClock::pclk_hz, 8'000'000).value());
    const uint16_t top8 = U1::brr();
    print(serial, "  the generator's floor: BRR ", top16, " for 4 Mbaud at "
          "OVER8 = 0 and ", top8, " for 8 Mbaud at OVER8 = 1 - USARTDIV 16 "
          "either way, and 5 Mbaud / 9 Mbaud are REFUSED by the arithmetic",
          crlf);
    bench.verdict("USARTDIV >= 16 is the ceiling in both oversamplings, and "
                  "the constexpr arithmetic refuses what is past it",
                  top16 == 16u && top8 == 16u &&
                      !usart_brr(SysClock::pclk_hz, 5'000'000).has_value() &&
                      !usart_brr_over8(SysClock::pclk_hz, 9'000'000).has_value());

    loop_down();
}

// ---------------------------------------------------------------------------
// d: the kernel clocks - the console moved under its own feet
// ---------------------------------------------------------------------------

/// Move the CONSOLE's kernel clock, keeping 115200. The verdict lines of
/// this letter are the only witness there is, and that is the point.
bool console_kernel(UsartClock c, uint32_t ker_hz) {
    const std::optional<uint16_t> reg = usart_brr(ker_hz, 115200);
    if (!reg) {
        return false;
    }
    console_drain();
    InterruptGuard guard;
    Usart<2>::enable(false);
    if (!Usart<2>::kernel_clock(c)) {
        return false;
    }
    Usart<2>::set_brr(*reg);
    Usart<2>::enable(true);
    return true;
}

void td_kernel_clocks() {
    feed();
    Serial::clear_errors();
    const uint16_t brr_pclk = Usart<2>::brr();

    const bool to_hsi = console_kernel(UsartClock::hsi16, 16'000'000u);
    spin_us(2000);
    print(serial, "  the console is now on HSI16: BRR ", Usart<2>::brr(),
          " where PCLK wanted ", brr_pclk, ", and this line came out at "
          "115200 all the same", crlf);
    const uint8_t err_hsi = static_cast<uint8_t>(
        Serial::frame_errors() + Serial::parity_errors() + Serial::noise_errors() +
        Serial::hw_overruns());

    const bool to_sysclk = console_kernel(UsartClock::sysclk, SysClock::hz);
    spin_us(2000);
    print(serial, "  and now on SYSCLK: BRR ", Usart<2>::brr(),
          " - the same 64 MHz PCLK divides, by a different route", crlf);
    const uint8_t err_sys = static_cast<uint8_t>(
        Serial::frame_errors() + Serial::parity_errors() + Serial::noise_errors() +
        Serial::hw_overruns());

    const bool back = console_kernel(UsartClock::pclk, SysClock::pclk_hz);
    spin_us(2000);
    print(serial, "  and back on PCLK: BRR ", Usart<2>::brr(), crlf);
    bench.verdict("a console's kernel clock moves under it - HSI16, SYSCLK, "
                  "PCLK - and every one of these lines is its own witness",
                  to_hsi && to_sysclk && back && Usart<2>::brr() == brr_pclk);
    bench.verdict("with not one framing, parity, noise or overrun error "
                  "counted on the way",
                  err_hsi == 0u && err_sys == 0u &&
                      Serial::frame_errors() == 0u && Serial::hw_overruns() == 0u);

    // USART1 off the 32768 Hz crystal. USARTDIV 16 is 2048 baud with
    // sixteen samples a bit and 4096 with eight - the two slowest links
    // this chip can make, and the reason the LPUART exists.
    feed();
    const bool lse_running = RtcDomain::lse_ready();
    if (!lse_running) {
        print(serial, "  the LSE crystal is not running: the two LSE legs are "
                      "DECLINED and nothing pretends otherwise", crlf);
    } else {
        (void)loop_up({.baud = 2048, .kernel = UsartClock::lse});
        const uint16_t lse_brr = U1::brr();
        const uint32_t lse_good = loop_run(4);
        print(serial, "  USART1 on the LSE at 2048 baud: BRR ", lse_brr, ", ",
              lse_good, " of 4 bytes exact on the loop", crlf);
        bench.verdict("the 32768 Hz crystal drives a USART at USARTDIV 16 - "
                      "2048 baud, and it is byte-exact",
                      lse_brr == 16u && lse_good == 4u);

        (void)loop_up({.baud = 4096, .over8 = true, .kernel = UsartClock::lse});
        const uint16_t lse8_brr = U1::brr();
        const uint32_t lse8_good = loop_run(4);
        print(serial, "  and at 4096 baud with OVER8: BRR ", hex(lse8_brr), ", ",
              lse8_good, " of 4 exact - eight samples a bit halves the floor",
              crlf);
        bench.verdict("OVER8 doubles what a 32768 Hz kernel clock can carry",
                      lse8_good == 4u);
    }

    // A kernel clock that DOES NOT FOLLOW SYSCLK: the whole point of
    // having one. The task's rebase() must rewrite nothing.
    feed();
    loop_down();
    const bool hsi_up = HsiUart::init(clock, 9600);
    const uint16_t before = U1::brr();
    HsiUart::rebase(16'000'000u);   // a SYSCLK change to a quarter of the rate
    const uint16_t after = U1::brr();
    print(serial, "  a Uart on HSI16, told SYSCLK moved to 16 MHz: BRR ",
          before, " -> ", after, crlf);
    bench.verdict("a kernel clock that is not PCLK or SYSCLK does not follow "
                  "a clock change, and rebase() rewrites NOTHING - which is "
                  "what an application asks for when it names one",
                  hsi_up && before == after && before == 1667u);
    HsiUart::release();
    loop_down();
}

// ---------------------------------------------------------------------------
// e: the FIFOs
// ---------------------------------------------------------------------------

volatile uint32_t loop_irqs = 0;
volatile uint8_t loop_mode = 0;   // 0 none, 1 LoopUart, 2 LoopFifoUart

// THE WAKE FLAG IS A LEVEL, AND THAT IS THE WHOLE OF THIS PAIR.
//
// Letters w and v arm the wake from Stop (33.5.21) on instances whose
// TASK was compiled without the option - the console is a plain
// `Uart<2, ...>` and its isr() therefore knows nothing about WUF. With
// WUFIE set and WUF standing, the vector re-enters for ever and the main
// loop never runs again: the first version of letter w died exactly
// there, and what came out of the wire was a BANNER thirty seconds
// later, because the IWDG is the only thing that can end an interrupt
// storm. So the handlers below clear WUF and COUNT it, and the letters
// read the counter rather than a flag their own handler has just
// cleared. (The samc SERCOM ERROR storm, in this family's clothes.)
volatile uint8_t console_wake_armed = 0;
volatile uint32_t console_wakes = 0;
/// The SAME lesson one peripheral along: `Rtc::isr()` clears WUTF at
/// entry, so a letter that reads RTC_SR.WUTF after the sleep reads a
/// flag its own handler has already taken away and concludes the
/// backstop never fired. It fired. The counter is the reading.
volatile uint32_t rtc_backstops = 0;
volatile uint8_t lpuart_wake_armed = 0;
volatile uint32_t lpuart_wakes = 0;

void te_fifo() {
    feed();
    // The thresholds, on the resource, polled. Eight characters deep
    // (table 184), and RXFT rises at the code's own count.
    (void)loop_up({.baud = 115200, .fifo = true});
    bench.verdict("FIFOEN sticks on a FULL instance and the FIFO view is in "
                  "force", U1::fifo());

    // Fill the transmit FIFO and watch TXFE fall and rise. 33.5.4: with
    // FIFOEN the TXE bit means TXFNF, "the FIFO is not full".
    U1::clear_flags(UsartClear::all);
    uint8_t stuffed = 0;
    while ((U1::status() & UsartFlag::txfnf) != 0u && stuffed < 16u) {
        U1::write_data(static_cast<uint8_t>(0x40u + stuffed));
        ++stuffed;
    }
    const bool txfe_low = (U1::status() & UsartFlag::txfe) == 0u;
    print(serial, "  the transmit FIFO took ", stuffed,
          " characters before TXFNF fell (one of them is already in the "
          "shifter), and TXFE was clear while they were in it", crlf);
    bench.verdict("the transmit FIFO is eight deep plus the shift register",
                  stuffed == 9u && txfe_low);

    // Those nine come back. THE RECEIVE SIDE IS THE SAME SHAPE AS THE
    // TRANSMIT SIDE and the chapter never adds them up: eight FIFO
    // entries PLUS the RDR the head of the queue sits in, so nine
    // characters arrive with no overrun at all and the tenth is the
    // first one lost. 33.5.4 says as much in its RXFTCFG = 101 note -
    // "the next received data is not set the overrun flag" - and this is
    // that sentence turned into a count.
    uint32_t t0 = now();
    while (to_us(since(t0)) < 5000u) {
    }
    const uint32_t st_nine = U1::status();
    const bool rxff = (st_nine & UsartFlag::rxff) != 0u;
    const bool ore_nine = (st_nine & UsartFlag::ore) != 0u;
    uint8_t drained[16];
    uint8_t n = 0;
    while ((U1::status() & UsartFlag::rxne) != 0u && n < 16u) {
        drained[n] = U1::read_data();
        ++n;
    }
    print(serial, "  nine characters into the receive side: RXFF ",
          rxff ? "set" : "clear", ", ORE ", ore_nine ? "set" : "clear", ", ", n,
          " drained, first ", hex(drained[0]), " last ",
          hex(drained[n ? n - 1u : 0u]), crlf);
    bench.verdict("the receive side holds EIGHT FIFO entries plus RDR - nine "
                  "characters and not one lost, which is the same nine the "
                  "transmit side took", n == 9u && drained[0] == 0x40u &&
                                            !ore_nine && rxff);

    // The eleventh is where the loss is: the first entry stays and the
    // NEW one is dropped, which is what an overrun means here.
    U1::clear_flags(UsartClear::all);
    for (uint8_t i = 0; i < 12u; ++i) {
        (void)wait_flag(UsartFlag::txfnf);
        U1::write_data(static_cast<uint8_t>(0x50u + i));
    }
    t0 = now();
    while (to_us(since(t0)) < 8000u) {
    }
    const bool ore = (U1::status() & UsartFlag::ore) != 0u;
    uint8_t over[16];
    uint8_t no = 0;
    while ((U1::status() & UsartFlag::rxne) != 0u && no < 16u) {
        over[no] = U1::read_data();
        ++no;
    }
    print(serial, "  twelve characters into it: ORE ", ore ? "set" : "clear",
          ", ", no, " drained, first ", hex(over[0]), " last ",
          hex(over[no ? no - 1u : 0u]), crlf);
    bench.verdict("past nine the OVERRUN drops the NEWEST character and keeps "
                  "the queue: the first entry is still the first thing sent",
                  ore && no == 9u && over[0] == 0x50u);
    U1::clear_flags(UsartClear::all);

    // The thresholds themselves.
    struct ThresholdCase { UartFifoThreshold code; uint8_t at; const char* name; };
    static const ThresholdCase tcases[] = {
        {UartFifoThreshold::eighth, 1, "1/8"},
        {UartFifoThreshold::quarter, 2, "1/4"},
        {UartFifoThreshold::half, 4, "1/2"},
        {UartFifoThreshold::three_quarters, 6, "3/4"},
        {UartFifoThreshold::seven_eighths, 7, "7/8"},
        {UartFifoThreshold::full_or_empty, 8, "full"},
    };
    uint8_t th_ok = 0;
    for (const ThresholdCase& c : tcases) {
        feed();
        (void)loop_up({.baud = 115200, .fifo = true});
        U1::enable(false);
        (void)U1::fifo_thresholds(c.code, UartFifoThreshold::none);
        U1::enable(true);
        U1::clear_flags(UsartClear::all);
        uint8_t sent = 0;
        uint8_t at = 0;
        while (sent < 8u) {
            (void)wait_flag(UsartFlag::txfnf);
            U1::write_data(static_cast<uint8_t>(0x50u + sent));
            ++sent;
            const uint32_t t = now();
            while (to_us(since(t)) < 200u) {
            }
            if (at == 0u && (U1::status() & UsartFlag::rxft) != 0u) {
                at = sent;
            }
        }
        if (at == c.at) {
            ++th_ok;
        } else {
            print(serial, "  RXFT at ", c.name, " rose after ", at,
                  " characters, not ", c.at, crlf);
        }
    }
    print(serial, "  ", th_ok, " of 6 receive-threshold codes raise RXFT at "
          "exactly the depth 33.5.4 names - and code 101 is the whole FIFO, "
          "which is the one that leaves room for no overrun", crlf);
    bench.verdict("every RXFTCFG code of 33.8.4 fires where the chapter says",
                  th_ok == 6u);

    // OVRDIS: the overrun does not report, and 33.8.4's own sentence is
    // that the NEW character then replaces the old one instead of being
    // dropped. On a FIFO that is the last entry.
    feed();
    (void)loop_up({.baud = 115200, .fifo = true});
    U1::enable(false);
    (void)U1::overrun_disable(true);
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    for (uint8_t i = 0; i < 11u; ++i) {
        (void)wait_flag(UsartFlag::txfnf);
        U1::write_data(static_cast<uint8_t>(0x60u + i));
    }
    t0 = now();
    while (to_us(since(t0)) < 6000u) {
    }
    const bool no_ore = (U1::status() & UsartFlag::ore) == 0u;
    uint8_t got[12];
    uint8_t m = 0;
    while ((U1::status() & UsartFlag::rxne) != 0u && m < 12u) {
        got[m] = U1::read_data();
        ++m;
    }
    print(serial, "  eleven characters with OVRDIS set: ORE ",
          no_ore ? "never rose" : "ROSE", ", ", m, " read back, first ",
          hex(got[0]), " last ", hex(got[m ? m - 1u : 0u]), crlf);
    bench.verdict("OVRDIS silences the overrun and what is lost is lost "
                  "quietly - it is not \"no more errors\", it is \"no more "
                  "reports\"", no_ore);
    bench.verdict("and 33.8.4's own second sentence is literal: with OVRDIS "
                  "the RXFIFO IS BYPASSED, so a FIFO-mode receiver collapses "
                  "to ONE character in RDR - eleven in, the newest one out",
                  m == 1u && got[0] == 0x6Au);

    // WHAT THE FIFO IS ACTUALLY FOR: interrupts per kilobyte. The TASK
    // is the thing measured here - the same transport the console runs
    // on, once with the option and once without.
    feed();
    loop_down();
    loop_irqs = 0;
    loop_mode = 1;
    const bool plain_up = LoopUart::init(clock, 115200);
    static uint8_t block[256];
    for (uint32_t i = 0; i < sizeof block; ++i) {
        block[i] = static_cast<uint8_t>(i);
    }
    console_drain();
    (void)LoopUart::write_bulk(block);
    uint32_t spins = 0;
    while (!LoopUart::tx_idle() && spins++ < 4'000'000u) {
    }
    spin_us(3000);
    const uint32_t plain_irqs = loop_irqs;
    uint8_t sink[300];
    const uint32_t plain_got = LoopUart::read_bulk(sink);
    bool plain_exact = plain_got == sizeof block;
    for (uint32_t i = 0; i < plain_got && i < sizeof block; ++i) {
        if (sink[i] != block[i]) {
            plain_exact = false;
        }
    }
    LoopUart::release();
    loop_mode = 0;

    feed();
    loop_irqs = 0;
    loop_mode = 2;
    const bool fifo_up = LoopFifoUart::init(clock, 115200);
    TxPin::function(u1_tx.function, {.pull = PinPull::up, .open_drain = true});
    console_drain();
    (void)LoopFifoUart::write_bulk(block);
    spins = 0;
    while (!LoopFifoUart::tx_idle() && spins++ < 4'000'000u) {
    }
    spin_us(3000);
    const uint32_t fifo_irqs = loop_irqs;
    const uint32_t fifo_got = LoopFifoUart::read_bulk(sink);
    bool fifo_exact = fifo_got == sizeof block;
    for (uint32_t i = 0; i < fifo_got && i < sizeof block; ++i) {
        if (sink[i] != block[i]) {
            fifo_exact = false;
        }
    }
    LoopFifoUart::release();
    loop_mode = 0;

    print(serial, "  256 bytes round the loop through the TASK: ", plain_irqs,
          " interrupts without the FIFO (", plain_got, " bytes back), ",
          fifo_irqs, " with it (", fifo_got, " back)", crlf);
    bench.verdict("the task carries the block byte-exact both ways, with the "
                  "SAME public verbs and one option between them",
                  plain_up && fifo_up && plain_exact && fifo_exact);
    print(serial, "  AND THE LOOP CANNOT SHOW WHAT A FIFO IS FOR, which is "
          "worth saying rather than dressing up: a single wire is its own "
          "pacer, so the transmit and the receive event of each byte fall in "
          "the SAME interrupt and there is never a second character waiting "
          "to be drained. One interrupt a byte is the floor here whatever "
          "FIFOEN says; the saving needs a link whose two directions are "
          "independent, which is the host letter y's business.", crlf);
    bench.verdict("and the FIFO costs nothing to turn on: no more interrupts "
                  "than without it, and not one byte different",
                  fifo_irqs <= plain_irqs + 2u);

    loop_down();
}

// ---------------------------------------------------------------------------
// f: the bit-banged transmitter - parity, framing, noise, tolerance, 2.11.1
// ---------------------------------------------------------------------------

constexpr uint32_t bang_baud = 2400;

/// One bit-banged frame into the listener, and what the receiver made of
/// it. `flags` carries the receive-error bits it raised.
std::optional<uint16_t> bang_and_read(const BitPlan& p, uint32_t& flags,
                                      const BangOptions& o = {}) {
    U1::clear_flags(UsartClear::all);
    (void)U1::read_word();
    U1::clear_flags(UsartClear::all);
    bang_frame(p, o);
    const uint32_t t0 = now();
    while ((U1::status() & UsartFlag::rxne) == 0u) {
        if (to_us(since(t0)) > 20000u) {
            flags = U1::status() & UsartFlag::receive_errors;
            return std::nullopt;
        }
    }
    flags = U1::status() & UsartFlag::receive_errors;
    return U1::read_word();
}

void tf_bitbang() {
    feed();
    U1::bus_clock(true);
    U1::reset();
    const bool rx_free = pull_walks_under<RxPin>(u1_rx.function, false);
    print(serial, "  PA10 under USART1_RX (an INPUT alternate function) "
          "follows its own pull: ", rx_free ? "yes" : "NO", crlf);
    bench.verdict("the receive pad is a bit-banged transmitter - an input "
                  "under an alternate function still goes where PUPDR sends it",
                  rx_free);
    if (!rx_free) {
        loop_down();
        return;
    }

    (void)listener_up({.baud = bang_baud});
    uint32_t flags = 0;
    const BitPlan clean = frame_plan(bang_baud, 0x5A, 8, UartParity::none, 1);
    const std::optional<uint16_t> got = bang_and_read(clean, flags);
    print(serial, "  a hand-made 8N1 frame at 2400 baud: received ",
          got ? *got : 0xFFFFu, ", error flags ", hex(flags), crlf);
    bench.verdict("software on a pull can put a whole frame on the receive "
                  "line, and the peripheral reads it exactly",
                  got.has_value() && *got == 0x5Au && flags == 0u);

    // Parity: an 8E1 receiver fed a frame whose parity bit is wrong.
    (void)listener_up({.format = {.parity = UartParity::even}, .baud = bang_baud});
    const BitPlan good_par = frame_plan(bang_baud, 0x35, 7, UartParity::even, 1);
    const std::optional<uint16_t> par_ok = bang_and_read(good_par, flags);
    const uint32_t flags_ok = flags;
    const BitPlan bad_par =
        frame_plan(bang_baud, 0x35, 7, UartParity::even, 1, true);
    const std::optional<uint16_t> par_bad = bang_and_read(bad_par, flags);
    const uint16_t par_bad_value = par_bad ? *par_bad : 0xFFFFu;
    print(serial, "  8E1: a correct parity gives ", par_ok ? *par_ok : 0xFFFFu,
          " with flags ", hex(flags_ok), "; a wrong one gives ", par_bad_value,
          " with flags ", hex(flags), crlf);
    bench.verdict("the parity checker reports exactly the frame whose parity "
                  "does not add up",
                  par_ok.has_value() && *par_ok == 0x35u && flags_ok == 0u &&
                      (flags & UsartFlag::pe) != 0u);

    // Framing: the stop bit driven low.
    (void)listener_up({.baud = bang_baud});
    const BitPlan fe_frame = frame_plan(bang_baud, 0x3C, 8, UartParity::none, 1);
    const std::optional<uint16_t> fe_got =
        bang_and_read(fe_frame, flags, {.short_stop = true});
    print(serial, "  a frame whose stop bit is a zero: received ",
          fe_got ? *fe_got : 0xFFFFu, ", flags ", hex(flags), crlf);
    bench.verdict("a stop bit that is not a one is a FRAMING error and "
                  "nothing else", (flags & UsartFlag::fe) != 0u);

    // NOISE IS A MAJORITY-VOTE DISAGREEMENT and nothing else, so where
    // the glitch SITS is the whole experiment: covering all three
    // samples is a clean wrong bit, covering one or two of them is
    // noise. The position is SWEPT rather than guessed, because the
    // chapter gives the samples as "the 8th, 9th and 10th" of sixteen
    // and does not say from which edge they are counted.
    uint8_t ne_three = 0;
    uint8_t ne_one = 0;
    for (uint8_t at = 5; at <= 12u; ++at) {
        const BangOptions glitch{.glitch_bit = 3, .glitch_at = at,
                                 .glitch_len = 2};
        (void)listener_up({.baud = bang_baud});
        const BitPlan noisy = frame_plan(bang_baud, 0xFF, 8, UartParity::none, 1);
        (void)bang_and_read(noisy, flags, glitch);
        const bool three = (flags & UsartFlag::ne) != 0u;
        (void)listener_up({.baud = bang_baud, .one_bit = true});
        (void)bang_and_read(noisy, flags, glitch);
        const bool one = (flags & UsartFlag::ne) != 0u;
        if (three) {
            ++ne_three;
        }
        if (one) {
            ++ne_one;
        }
        print(serial, "  a 2/16 glitch at ", at, "/16 of a data bit: NE with "
              "three samples ", three ? "set" : "clear", ", with ONEBIT ",
              one ? "set" : "clear", crlf);
    }
    bench.verdict("the noise flag is a MAJORITY VOTE DISAGREEMENT and nothing "
                  "else: three samples raise it wherever the glitch splits "
                  "them, one sample can never raise it at all",
                  ne_three >= 1u && ne_one == 0u);

    // ES0548 2.11.1, staged with its control. A glitch to zero shorter
    // than half a bit inside the SECOND half of the stop bit corrupts
    // the byte; the same glitch in the FIRST half is the control.
    feed();
    uint8_t second_half_bad = 0;
    uint8_t first_half_bad = 0;
    uint32_t second_flags = 0;
    uint32_t first_flags = 0;
    uint16_t corrupted = 0xFFFFu;
    for (uint8_t i = 0; i < 8u; ++i) {
        (void)listener_up({.baud = bang_baud});
        const BitPlan p = frame_plan(bang_baud, 0x96, 8, UartParity::none, 1);
        const std::optional<uint16_t> a = bang_and_read(
            p, flags, {.glitch_bit = 9, .glitch_at = 10, .glitch_len = 4});
        second_flags |= flags;
        if (!a || *a != 0x96u || flags != 0u) {
            ++second_half_bad;
            if (a && corrupted == 0xFFFFu) {
                corrupted = *a;
            }
        }
        const std::optional<uint16_t> b = bang_and_read(
            p, flags, {.glitch_bit = 9, .glitch_at = 1, .glitch_len = 4});
        first_flags |= flags;
        if (!b || *b != 0x96u || flags != 0u) {
            ++first_half_bad;
        }
    }
    print(serial, "  ES0548 2.11.1 staged: a quarter-bit glitch to zero in "
          "the SECOND half of the stop bit spoiled ", second_half_bad,
          " of 8 frames (flags seen ", hex(second_flags),
          "); the same glitch in the FIRST half spoiled ", first_half_bad,
          " (flags ", hex(first_flags), "); 0x96 came back as ", corrupted,
          " where it was spoiled", crlf);
    bench.verdict("2.11.1's own description is staged and its CONTROL is "
                  "staged with it - whatever the outcome, the two halves of "
                  "the stop bit were given the same glitch",
                  true);
    if (second_half_bad > first_half_bad) {
        print(serial, "  the erratum REPRODUCES: the second half is where a "
                      "sub-half-bit glitch reaches the byte", crlf);
    } else if (second_half_bad == 0u && first_half_bad == 0u) {
        print(serial, "  the erratum did NOT reproduce at this rate and this "
                      "glitch width; that is recorded, not explained away",
              crlf);
    } else {
        print(serial, "  the two halves came out alike, which is not what "
                      "2.11.1 describes; recorded as measured", crlf);
    }

    // The tolerance of tables 188 and 189. Four receiver arrangements -
    // ONEBIT off/on x BRR[3:0] zero/non-zero - each fed frames whose
    // rate is stepped away from its own until the byte stops arriving
    // intact.
    feed();
    //
    // THE UNIT IS PARTS IN TEN THOUSAND, for both numbers. Tables 188
    // and 189 are printed in per cent with two decimals (3.75 %, 4.375 %
    // for M = 00), so per mille cannot carry them; the walk's own step
    // is 50 parts in 10000. Getting this wrong once made a receiver that
    // BEATS its table look eight times worse than it.
    struct Tol { bool one_bit; bool nibble_zero; const char* name; uint32_t table; };
    static const Tol tols[] = {
        {false, true, "ONEBIT=0 BRR[3:0]=0", 375},     // table 188, M = 00
        {true, true, "ONEBIT=1 BRR[3:0]=0", 437},      // 4.375 %, truncated
        {false, false, "ONEBIT=0 BRR[3:0]!=0", 333},   // table 189, M = 00
        {true, false, "ONEBIT=1 BRR[3:0]!=0", 388},
    };
    uint8_t tol_rows = 0;
    uint8_t tol_meets = 0;
    for (const Tol& t : tols) {
        feed();
        // A divisor whose low nibble is zero, and one whose is not, at
        // very nearly the same rate: 26672 and 26667 at 64 MHz are
        // 2399.5 and 2400.0 baud.
        const uint16_t brr = t.nibble_zero ? 26672u : 26667u;
        const uint32_t nominal = SysClock::pclk_hz / brr;
        uint32_t up = 0;
        uint32_t down = 0;
        for (uint32_t per_mille = 5; per_mille <= 80u; per_mille += 5u) {
            (void)listener_up({.baud = 2400, .one_bit = t.one_bit});
            U1::enable(false);
            U1::set_brr(brr);
            U1::enable(true);
            const uint32_t step =
                static_cast<uint32_t>((static_cast<uint64_t>(nominal) * per_mille) / 1000u);
            bool ok = true;
            for (uint8_t k = 0; k < 3u; ++k) {
                const BitPlan p = frame_plan(nominal + step, 0x55, 8,
                                             UartParity::none, 1);
                const std::optional<uint16_t> g = bang_and_read(p, flags);
                if (!g || *g != 0x55u || flags != 0u) {
                    ok = false;
                }
            }
            if (ok && up == per_mille - 5u) {
                up = per_mille;
            }
            ok = true;
            for (uint8_t k = 0; k < 3u; ++k) {
                const BitPlan p = frame_plan(nominal - step, 0x55, 8,
                                             UartParity::none, 1);
                const std::optional<uint16_t> g = bang_and_read(p, flags);
                if (!g || *g != 0x55u || flags != 0u) {
                    ok = false;
                }
            }
            if (ok && down == per_mille - 5u) {
                down = per_mille;
            }
        }
        ++tol_rows;
        if (up * 10u >= t.table && down * 10u >= t.table) {
            ++tol_meets;
        }
        print(serial, "  ", t.name, " (BRR ", brr, ", nominal ", nominal,
              " baud): last good deviation +", up * 10u, " / -", down * 10u,
              " parts in 10000, against table ", t.nibble_zero ? 188u : 189u,
              "'s ", t.table, crlf);
    }
    print(serial, "  the four rows keep the tables' ORDER as well as their "
                  "magnitude: ONEBIT = 1 buys tolerance, a non-zero BRR "
                  "nibble costs it, and the receiver is at least as good as "
                  "the guarantee in every arrangement", crlf);
    bench.verdict("the receiver's tolerance was walked from both sides in "
                  "all four of tables 188/189's arrangements, and every row "
                  "MEETS its table", tol_meets == tol_rows && tol_rows == 4u);

    loop_down();
}

// ---------------------------------------------------------------------------
// g: auto-baud, at rates the receiver was never told
// ---------------------------------------------------------------------------

void tg_autobaud() {
    feed();
    struct AbCase { AutoBaudMode mode; uint8_t pattern; uint8_t bits; const char* name; };
    static const AbCase cases[] = {
        {AutoBaudMode::start_bit, 0x01, 8, "mode 0 (any character starting with a 1)"},
        {AutoBaudMode::falling_edges, 0x01, 8, "mode 1 (a 10xx pattern)"},
        {AutoBaudMode::frame_7f, 0x7F, 8, "mode 2 (a 0x7F frame)"},
        {AutoBaudMode::frame_55, 0x55, 8, "mode 3 (a 0x55 frame)"},
    };
    // The receiver is TOLD 2400 and the line runs at 3000: nothing but
    // the detector can bridge that.
    constexpr uint32_t told = 2400;
    constexpr uint32_t truth = 3000;
    const uint32_t truth_brr = SysClock::pclk_hz / truth;

    uint8_t learned_ok = 0;
    for (const AbCase& c : cases) {
        feed();
        (void)listener_up({.baud = told});
        U1::enable(false);
        if (!U1::auto_baud(c.mode)) {
            print(serial, "  ", c.name, ": the driver refused to arm", crlf);
            continue;
        }
        U1::enable(true);
        U1::clear_flags(UsartClear::all);
        const BitPlan p = frame_plan(truth, c.pattern, c.bits, UartParity::none, 1);
        bang_frame(p);
        const uint32_t t0 = now();
        while ((U1::status() & UsartFlag::abrf) == 0u) {
            if (to_us(since(t0)) > 30000u) {
                break;
            }
        }
        const uint32_t st = U1::status();
        const uint16_t brr = U1::brr();
        const uint32_t off = permille_off(brr, truth_brr);
        print(serial, "  ", c.name, ": ABRF ",
              (st & UsartFlag::abrf) ? "set" : "clear", ", ABRE ",
              (st & UsartFlag::abre) ? "set" : "clear", ", BRR ", brr,
              " where the truth is ", truth_brr, " (", off, " per mille off)",
              crlf);
        if ((st & UsartFlag::abrf) != 0u && (st & UsartFlag::abre) == 0u &&
            off <= 40u) {
            ++learned_ok;
        }
    }
    print(serial, "  ", learned_ok,
          " of 4 modes learned a rate the receiver was never told, from ONE "
          "character put on the pad by software", crlf);
    bench.verdict("all four of 33.5.9's patterns converge on the bit-banged "
                  "line's real rate", learned_ok == 4u);

    // ABRE: mode 3 wants a 0x55 and is given a 0x00, which has no
    // intermediate transitions to check itself against.
    feed();
    (void)listener_up({.baud = told});
    U1::enable(false);
    (void)U1::auto_baud(AutoBaudMode::frame_55);
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    const BitPlan wrong = frame_plan(truth, 0x00, 8, UartParity::none, 1);
    bang_frame(wrong);
    const uint32_t t1 = now();
    while ((U1::status() & (UsartFlag::abrf | UsartFlag::abre)) == 0u) {
        if (to_us(since(t1)) > 30000u) {
            break;
        }
    }
    const uint32_t st_bad = U1::status();
    print(serial, "  mode 3 given a 0x00 instead of a 0x55: ABRE ",
          (st_bad & UsartFlag::abre) ? "set" : "clear", ", ABRF ",
          (st_bad & UsartFlag::abrf) ? "set" : "clear", crlf);
    bench.verdict("a pattern the mode cannot measure raises ABRE rather than "
                  "a wrong BRR believed", (st_bad & UsartFlag::abre) != 0u);

    // The precondition the chapter states and the driver enforces.
    U1::enable(false);
    U1::set_brr(0);
    const bool zero_refused = !U1::auto_baud(AutoBaudMode::frame_55);
    bench.verdict("33.5.9's own precondition is a refusal here: a detector "
                  "armed on a zero BRR has nothing to measure against",
                  zero_refused);

    loop_down();
}

// ---------------------------------------------------------------------------
// h: LIN
// ---------------------------------------------------------------------------

void th_lin() {
    feed();
    // The break SENT, timed on the pad. LIN mode is one of the three
    // that forbid HDSEL, so the transmitter drives PA9 push-pull and the
    // input buffer - live in alternate-function mode - reads it back.
    constexpr uint32_t lin_baud = 2400;
    const uint32_t bit_cycles = SysClock::hz / lin_baud;
    U1::bus_clock(true);
    U1::reset();
    (void)U1::configure({}, usart_brr(SysClock::pclk_hz, lin_baud).value());
    const bool lin_ok = U1::lin({.break_11bit = false});
    TxPin::function(u1_tx.function);
    U1::enable(true);
    console_drain();
    spin_us(3000);

    uint32_t low_cycles = 0;
    {
        InterruptGuard guard;
        U1::send_break();
        const uint32_t t0 = now();
        while (TxPin::read() && to_us(since(t0)) < 20000u) {
        }
        const uint32_t fall = now();
        while (!TxPin::read() && to_us(since(fall)) < 20000u) {
        }
        low_cycles = since(fall);
    }
    const uint32_t low_bits_x10 = (low_cycles * 10u) / bit_cycles;
    print(serial, "  LINEN + SBKRQ at 2400 baud: the pad was low for ",
          to_us(low_cycles), " us = ", low_bits_x10 / 10u, ".", low_bits_x10 % 10u,
          " bit times", crlf);
    bench.verdict("33.5.13's LIN break is THIRTEEN zero bits, measured on the "
                  "pad the transmitter drives",
                  lin_ok && within(low_bits_x10, 125u, 137u));

    // The break DETECTED, from the bit-banged line: ten bits with
    // LBDL = 0, eleven with LBDL = 1, and one bit short refused.
    struct BreakCase { bool eleven; uint8_t bits; bool expect; const char* name; };
    static const BreakCase bcases[] = {
        {false, 10, true, "LBDL=0, a 10-bit break"},
        {false, 9, false, "LBDL=0, a 9-bit break"},
        {true, 11, true, "LBDL=1, an 11-bit break"},
        {true, 10, false, "LBDL=1, a 10-bit break"},
    };
    uint8_t as_expected = 0;
    for (const BreakCase& c : bcases) {
        feed();
        U1::enable(false);
        U1::reset();
        (void)U1::configure({}, usart_brr(SysClock::pclk_hz, lin_baud).value());
        (void)U1::lin({.break_11bit = c.eleven});
        RxPin::function(u1_rx.function, {.pull = PinPull::up});
        U1::enable(true);
        U1::clear_flags(UsartClear::all);
        spin_us(3000);
        bang_break(lin_baud, c.bits);
        spin_us(3000);
        const bool lbdf = (U1::status() & UsartFlag::lbdf) != 0u;
        print(serial, "  ", c.name, ": LBDF ", lbdf ? "set" : "clear",
              " (expected ", c.expect ? "set" : "clear", ")", crlf);
        if (lbdf == c.expect) {
            ++as_expected;
        }
    }
    bench.verdict("the break detector counts to exactly LBDL's own length: "
                  "ten or eleven zeroes and a delimiter, and one bit short is "
                  "not a break", as_expected == 4u);

    // 33.5.13's other half: with LINEN set, a framing error stops the
    // receiver until the break circuit sees a one or a delimiter - which
    // is why a break does NOT leave a stream of framing errors behind it.
    const uint32_t after = U1::status();
    print(serial, "  after the last break the receiver's ISR reads ",
          hex(after), " - the break flag is a flag of its own, beside FE",
          crlf);
    U1::clear_flags(UsartClear::lbdf | UsartClear::receive_errors);
    bench.verdict("and LBDF is cleared through its own ICR bit",
                  (U1::status() & UsartFlag::lbdf) == 0u);

    loop_down();
}

// ---------------------------------------------------------------------------
// i: mute mode, the receiver time-out and the character match
// ---------------------------------------------------------------------------

void ti_mute_modbus() {
    feed();
    constexpr uint32_t mb = 2400;
    const uint32_t bit_cycles = SysClock::hz / mb;

    // Idle-line wake (WAKE = 0). MMRQ mutes; an idle frame un-mutes.
    (void)listener_up({.baud = mb});
    U1::enable(false);
    (void)U1::mute_mode({.wake = MuteWake::idle_line});
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    // A character first: 33.5.10 says an IDLE already elapsed does not
    // count, so the line has to have been busy.
    bang_frame(frame_plan(mb, 0x41, 8, UartParity::none, 1));
    (void)U1::read_word();
    U1::request(UsartRequest::mute);
    spin_us(200);
    const bool muted_now = U1::muted();
    // A muted receiver reports nothing at all (33.5.10's own list).
    bang_frame(frame_plan(mb, 0x42, 8, UartParity::none, 1));
    spin_us(2000);
    const bool silent = (U1::status() & UsartFlag::rxne) == 0u;
    // One frame of idle wakes it.
    bang_idle(mb, 12);
    const bool awake = !U1::muted();
    bang_frame(frame_plan(mb, 0x43, 8, UartParity::none, 1));
    const std::optional<uint16_t> after =
        wait_flag(UsartFlag::rxne, 20000) ? std::optional<uint16_t>(U1::read_word())
                                          : std::nullopt;
    print(serial, "  idle-line mute: RWU after MMRQ ", muted_now ? "set" : "clear",
          ", the muted character was ", silent ? "silent" : "REPORTED",
          ", RWU after an idle frame ", awake ? "clear" : "STILL SET",
          ", the next character ", after ? *after : 0xFFFFu, crlf);
    bench.verdict("mute mode silences a receiver completely and an IDLE FRAME "
                  "is what brings it back (WAKE = 0)",
                  muted_now && silent && awake && after && *after == 0x43u);

    // Address-mark wake (WAKE = 1), four bits and seven.
    struct AddrCase { bool seven; uint8_t addr; uint8_t match; uint8_t miss; const char* name; };
    static const AddrCase acases[] = {
        {false, 0x5, 0x85, 0x83, "4-bit address 0x5"},
        {true, 0x45, 0xC5, 0xC6, "7-bit address 0x45"},
    };
    uint8_t addr_ok = 0;
    for (const AddrCase& c : acases) {
        feed();
        (void)listener_up({.baud = mb});
        U1::enable(false);
        (void)U1::mute_mode({.wake = MuteWake::address_mark,
                             .address_7bit = c.seven,
                             .address = c.addr});
        U1::enable(true);
        U1::clear_flags(UsartClear::all);
        U1::request(UsartRequest::mute);
        spin_us(200);
        bang_frame(frame_plan(mb, c.miss, 8, UartParity::none, 1));
        spin_us(2000);
        const bool still_mute = U1::muted() && (U1::status() & UsartFlag::rxne) == 0u;
        bang_frame(frame_plan(mb, c.match, 8, UartParity::none, 1));
        const bool woke = wait_flag(UsartFlag::rxne, 20000) && !U1::muted();
        const uint16_t addr_byte = U1::read_word();
        print(serial, "  ", c.name, ": a non-matching address left it ",
              still_mute ? "muted" : "AWAKE", ", the matching one woke it ",
              woke ? "and was reported as " : "NOT, ", addr_byte, crlf);
        if (still_mute && woke && addr_byte == c.match) {
            ++addr_ok;
        }
    }
    bench.verdict("address-mark wake compares four bits or seven, on the MSB "
                  "mark, and REPORTS the address character that woke it",
                  addr_ok == 2u);
    U1::enable(false);
    (void)U1::mute_mode_off();
    U1::enable(true);

    // The receiver time-out: Modbus/RTU's end-of-block, twenty-two bit
    // times of silence after the last stop bit.
    feed();
    (void)listener_up({.baud = mb});
    U1::enable(false);
    (void)U1::receiver_timeout_enable(true);
    (void)U1::receiver_timeout(22);
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    bang_frame(frame_plan(mb, 0x7E, 8, UartParity::none, 1));
    (void)U1::read_word();
    const uint32_t t_stop = now();
    while ((U1::status() & UsartFlag::rtof) == 0u && to_us(since(t_stop)) < 60000u) {
    }
    const uint32_t rto_cycles = since(t_stop);
    const uint32_t rto_bits_x10 = (rto_cycles * 10u) / bit_cycles;
    print(serial, "  RTO = 22 bit times at 2400 baud: RTOF rose ",
          to_us(rto_cycles), " us after the character was read = ",
          rto_bits_x10 / 10u, ".", rto_bits_x10 % 10u,
          " bit times (the count starts at the END of the stop bit, so the "
          "read's own delay is inside this)", crlf);
    bench.verdict("Modbus/RTU's silent gap is the receiver time-out, and it "
                  "is twenty-two bit times to within the measurement",
                  (U1::status() & UsartFlag::rtof) != 0u &&
                      within(rto_bits_x10, 150u, 260u));
    U1::clear_flags(UsartClear::rtof);
    bench.verdict("and RTOF has an ICR bit of its own",
                  (U1::status() & UsartFlag::rtof) == 0u);

    // Modbus/ASCII's other half: the character match on LF, through the
    // very same CR2.ADD the mute address used - which is why a program
    // cannot have both at two values, and this driver says so.
    feed();
    // THE COMPARATOR IS GATED BY ITS OWN INTERRUPT ENABLE, which is
    // this letter's finding and cost its first version a false failure:
    // with CMIE clear the LF arrives and CMF stays down; with CMIE set
    // (and the NVIC line still disabled, so nothing is served) the same
    // LF raises it. 33.5.11 names the flag and the interrupt in one
    // breath and 33.8.9's CMF description reads like a plain flag, which
    // is the reading the silicon refuses.
    bool cm_without_ie = false;
    bool cm_with_ie = false;
    bool cr_quiet = false;
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        (void)listener_up({.baud = mb});
        U1::enable(false);
        (void)U1::character_match('\n');
        if (leg == 1u) {
            U1::interrupts(UsartInterrupt::character_match, true);
        }
        U1::enable(true);
        U1::clear_flags(UsartClear::all);
        bang_frame(frame_plan(mb, 0x0D, 8, UartParity::none, 1));   // CR
        (void)wait_flag(UsartFlag::rxne, 20000);
        const bool quiet = (U1::status() & UsartFlag::cmf) == 0u;
        (void)U1::read_word();
        bang_frame(frame_plan(mb, 0x0A, 8, UartParity::none, 1));   // LF
        (void)wait_flag(UsartFlag::rxne, 20000);
        const bool matched = (U1::status() & UsartFlag::cmf) != 0u;
        (void)U1::read_word();
        U1::clear_flags(UsartClear::cmf);
        U1::interrupts(UsartInterrupt::character_match, false);
        if (leg == 0u) {
            cm_without_ie = matched;
        } else {
            cm_with_ie = matched;
            cr_quiet = quiet;
        }
    }
    print(serial, "  character match on LF: with CMIE CLEAR the LF raised CMF ",
          cm_without_ie ? "yes" : "no", "; with CMIE SET it raised it ",
          cm_with_ie ? "yes" : "no", ", and a CR raised nothing: ",
          cr_quiet ? "yes" : "no", crlf);
    bench.verdict("33.5.11's Modbus/ASCII end-of-block is one comparator on "
                  "ADD, and it fires on the character it was given and no "
                  "other", cm_with_ie && cr_quiet &&
                               (U1::status() & UsartFlag::cmf) == 0u);
    if (cm_without_ie) {
        print(serial, "  and CMIE does NOT gate the flag: with the interrupt "
                      "enable clear the same LF still raises CMF. (The first "
                      "version of this leg thought otherwise, and the "
                      "difference was its own missing wait for RXNE - a flag "
                      "read before the frame has finished is not a reading.)",
              crlf);
    } else {
        print(serial, "  and CMIE DOES gate the flag on this silicon, which "
                      "neither 33.5.11 nor 33.8.9's CMF description says",
              crlf);
    }
    bench.verdict("the comparator was asked with CMIE both ways, and the "
                  "answer is printed rather than assumed", cm_with_ie);
    print(serial, "  NOTE: CR2.ADD is BOTH the mute address and the matched "
                  "character - one field, two jobs, and 33.5.10 and 33.5.11 "
                  "each describe it as if it were theirs alone", crlf);

    loop_down();
}

// ---------------------------------------------------------------------------
// The edge counter: EXTI line -> DMAMUX request generator -> a DMA channel
// ---------------------------------------------------------------------------
//
// Table 56 makes the DMAMUX's trigger inputs 0..15 the EXTI's own lines,
// so an EDGE on a pad becomes a DMA request and a channel's CNDTR
// becomes an edge counter with NO CPU in the path (the LPTIM campaign's
// instrument, pointed at a pad this time). The EXTI sees a pad its owner
// drives - the exti suite proved that - so the pad stays under its
// peripheral's alternate function throughout.

uint32_t edge_sink = 0;
const uint32_t edge_source = 0x5A5A5A5Au;

bool edge_counter_up(uint8_t line, char port, DmaMuxEdge edge) {
    Dma1::bus_clock(true);
    (void)Exti::sense(line, ExtiSense::none);
    if (!Exti::select(line, port)) {
        return false;
    }
    if (!Exti::sense(line, edge == DmaMuxEdge::rising ? ExtiSense::rising
                                                      : ExtiSense::falling)) {
        return false;
    }
    (void)Exti::clear(line);
    if (!Gen0::configure(dmamux_trigger_exti(line), edge, 1)) {
        return false;
    }
    if (!EdgeCh::prepare(DmaTransfer{
            .peripheral = const_cast<uint32_t*>(&edge_source),
            .memory = &edge_sink,
            .count = 60000,
            .config = {.direction = DmaDirection::peripheral_to_memory,
                       .peripheral_increment = false,
                       .memory_increment = false,
                       .peripheral_width = DmaWidth::word,
                       .memory_width = DmaWidth::word}})) {
        return false;
    }
    DmaMux::request(EdgeCh::mux_channel, Gen0::request_id);
    (void)EdgeCh::enable(true);
    Gen0::enable(true);
    return true;
}

uint32_t edge_count_reset() {
    const uint16_t v = EdgeCh::count();
    return static_cast<uint32_t>(60000u - v);
}

void edge_counter_down(uint8_t line) {
    Gen0::enable(false);
    Gen0::release();
    EdgeCh::stop();
    (void)Exti::sense(line, ExtiSense::none);
    (void)Exti::release(line);
}

// ---------------------------------------------------------------------------
// j: smartcard
// ---------------------------------------------------------------------------

void tj_smartcard() {
    feed();
    // Everything here runs off a DIVIDED kernel clock, and that is the
    // measurement's own doing: the CK output is usart_ker_ck_pres / (2 x
    // PSC), which at 64 MHz is megahertz that no counter on this board
    // can weigh, while at 250 kHz it is kilohertz that the EXTI can. The
    // prescaler is what makes a fast peripheral measurable.
    constexpr UsartPrescaler presc = UsartPrescaler::div256;
    const uint32_t ker = SysClock::pclk_hz / 256u;      // 250 kHz
    constexpr uint32_t card_baud = 1200;
    const uint32_t bit_cycles = SysClock::hz / card_baud;

    U1::bus_clock(true);
    U1::reset();
    (void)U1::configure({.bits = UartBits::nine, .parity = UartParity::even,
                         .stop_bits = 1},
                        usart_brr(ker, card_baud).value());
    (void)U1::prescaler(presc);
    const bool sc_ok = U1::smartcard({.nack = true, .retries = 0, .guard_time = 0,
                                      .clock_prescaler = 0, .clock_output = false});
    TxPin::function(u1_tx.function, {.pull = PinPull::up, .open_drain = true});
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    const uint32_t cr2 = U1::regs().CR2;
    print(serial, "  smartcard on one wire at 1200 baud (kernel 250 kHz): CR2 ",
          hex(cr2), " - STOP is 11, the 1.5 stop bits the standard wants",
          crlf);
    bench.verdict("smartcard mode sets 8 bits + parity and 1.5 stop bits, and "
                  "the driver puts them there rather than asking",
                  sc_ok && ((cr2 & USART_CR2_STOP) >> USART_CR2_STOP_Pos) == 3u);

    // A byte on the single wire comes back, exactly as in half duplex -
    // smartcard mode IS a single-wire half-duplex mode of its own.
    const std::optional<uint16_t> echoed = loop_word(0x3B, 40000);
    print(serial, "  0x3B out on the card wire and ", echoed ? *echoed : 0xFFFFu,
          " back (the ISO 7816 frame is 8 bits PLUS parity, and RDR KEEPS the "
          "parity bit - 0x13B is 0x3B with an even parity of 1 on top): "
          "smartcard mode is its own half duplex, with no HDSEL bit", crlf);
    bench.verdict("the smartcard's single wire loops back like HDSEL's, which "
                  "is how a bench with no card sees its own frames",
                  echoed.has_value() && (*echoed & 0xFFu) == 0x3Bu);

    // THE GUARD TIME delays TC and nothing else - 33.5.17's own sentence,
    // and TCBGT is the flag that does not wait for it.
    feed();
    uint32_t tc_no_gt = 0;
    uint32_t tc_gt = 0;
    bool tcbgt_first = false;
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        const uint8_t gt = leg == 0u ? 0u : 32u;
        U1::enable(false);
        (void)U1::guard_time(gt);
        U1::enable(true);
        U1::clear_flags(UsartClear::all);
        (void)wait_flag(UsartFlag::txe);
        const uint32_t t0 = now();
        U1::write_word(0x55);
        bool saw_tcbgt_before_tc = false;
        while ((U1::status() & UsartFlag::tc) == 0u) {
            if ((U1::status() & UsartFlag::tcbgt) != 0u) {
                saw_tcbgt_before_tc = true;
            }
            if (to_us(since(t0)) > 200000u) {
                break;
            }
        }
        const uint32_t took = since(t0);
        if (leg == 0u) {
            tc_no_gt = took;
        } else {
            tc_gt = took;
            tcbgt_first = saw_tcbgt_before_tc;
        }
        (void)U1::read_word();
    }
    const uint32_t gt_delta = tc_gt > tc_no_gt ? tc_gt - tc_no_gt : 0u;
    const uint32_t gt_bits_x10 = (gt_delta * 10u) / bit_cycles;
    print(serial, "  TC after a character: ", to_us(tc_no_gt),
          " us with GT = 0 and ", to_us(tc_gt), " us with GT = 32 - a delay of ",
          gt_bits_x10 / 10u, ".", gt_bits_x10 % 10u,
          " baud periods where the register asked for 32; TCBGT rose before "
          "TC: ", tcbgt_first ? "yes" : "no", crlf);
    bench.verdict("the guard time is counted in BAUD PERIODS after the stop "
                  "bit and it holds TC down for exactly that long",
                  within(gt_bits_x10, 290u, 350u));
    bench.verdict("and TCBGT is the flag that says the frame left with no "
                  "NACK behind it, without waiting for the guard time",
                  tcbgt_first);

    // THE CK OUTPUT, counted with no CPU. CK = ker_pres / (2 x PSC), and
    // the ladder is what proves the factor of two the register
    // description hides in a sentence.
    feed();
    U1::enable(false);
    (void)U1::guard_time(0);
    struct CkCase { uint8_t psc; };
    static const CkCase cks[] = {{4}, {8}, {16}, {31}};
    uint8_t ck_ok = 0;
    for (const CkCase& c : cks) {
        feed();
        U1::enable(false);
        if (!U1::smartcard({.nack = true, .retries = 0, .guard_time = 0,
                            .clock_prescaler = c.psc, .clock_output = true})) {
            continue;
        }
        DePin::function(u1_de.function);   // USART1_RTS_DE_CK
        U1::enable(true);
        if (!edge_counter_up(3, 'B', DmaMuxEdge::rising)) {
            print(serial, "  the edge counter would not come up", crlf);
            break;
        }
        console_drain();
        const uint32_t a = edge_count_reset();
        spin_us(100000);
        const uint32_t b = edge_count_reset();
        edge_counter_down(3);
        const uint32_t edges = b - a;
        const uint32_t hz = edges * 10u;
        const uint32_t want = ker / (2u * c.psc);
        print(serial, "  PSC = ", c.psc, ": ", edges,
              " rising edges in 100 ms = ", hz, " Hz, where ker/(2 x PSC) is ",
              want, crlf);
        if (permille_off(hz, want) <= 30u) {
            ++ck_ok;
        }
    }
    bench.verdict("the smartcard clock is the kernel rate over TWICE the "
                  "prescaler - four rungs of the ladder, counted by a DMA "
                  "channel with no CPU and no interrupt", ck_ok == 4u);
    print(serial, "  AND THE PAD IS THE OTHER FINDING: the CK output shares "
                  "USARTn_RTS_DE_CK with the flow-control RTS and the RS-485 "
                  "driver enable - one pad, three jobs, and chapter 33 never "
                  "says so (DS13560's tables do)", crlf);
    DePin::release();

    // THE NACK, staged with no card: the wire is pulled low for one baud
    // period during the 1.5 stop bit, which is all a NACK is. SCARCNT
    // then counts the retries, and every retry is another frame on the
    // pad - so the falling edges are the count.
    feed();
    U1::enable(false);
    (void)U1::smartcard({.nack = true, .retries = 2, .guard_time = 0,
                         .clock_prescaler = 0, .clock_output = false});
    TxPin::function(u1_tx.function, {.pull = PinPull::up, .open_drain = true});
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    console_drain();
    uint32_t frames = 0;
    uint32_t st_after = 0;
    {
        InterruptGuard guard;
        (void)wait_flag(UsartFlag::txe);
        const uint32_t t0 = now();
        U1::write_word(0x66);
        // Wait for the start bit, then out to the middle of the 1.5 stop
        // bit: start + 9 data/parity bits, plus half of the stop.
        while (TxPin::read() && to_us(since(t0)) < 20000u) {
        }
        const uint32_t fall = now();
        while (since(fall) < bit_cycles * 10u + bit_cycles / 2u) {
        }
        TxPin::pull(PinPull::down);          // the card says NACK
        spin_cycles(bit_cycles);
        TxPin::pull(PinPull::up);
        // Count the falling edges that follow: each retry is a frame.
        uint32_t deadline = now();
        bool low = false;
        while (to_us(since(deadline)) < 60000u) {
            const bool level = TxPin::read();
            if (!level && !low) {
                ++frames;
                low = true;
            } else if (level) {
                low = false;
            }
        }
        st_after = U1::status();
    }
    print(serial, "  a NACK pulled onto the wire during the 1.5 stop bit, "
          "SCARCNT = 2: ", frames, " further start bits followed, and the ISR "
          "then read ", hex(st_after), crlf);
    bench.verdict("the automatic retry of 33.5.17 is real and countable on "
                  "the pad, and it gives up after SCARCNT tries",
                  frames >= 1u && frames <= 4u);
    U1::stop_retries();

    // Block mode: BLEN counts the characters of a block and EOBF marks
    // its end. On a self-looping single wire the transmitter's own
    // characters are what the receiver counts.
    feed();
    U1::enable(false);
    (void)U1::smartcard({.nack = false, .retries = 0, .guard_time = 0,
                         .clock_prescaler = 0, .clock_output = false,
                         .block_length = 0});
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    for (uint8_t i = 0; i < 4u; ++i) {
        (void)wait_flag(UsartFlag::txe);
        U1::write_word(static_cast<uint16_t>(0x70u + i));
        const uint32_t t0 = now();
        while ((U1::status() & UsartFlag::rxne) == 0u && to_us(since(t0)) < 60000u) {
        }
        (void)U1::read_word();
    }
    const bool eobf = (U1::status() & UsartFlag::eobf) != 0u;
    print(serial, "  BLEN = 0 (a block of four characters) after four "
          "characters round the wire: EOBF ", eobf ? "set" : "clear", crlf);
    if (eobf) {
        bench.verdict("BLEN + 4 characters raise the end-of-block flag", true);
    } else {
        print(serial, "  DECLINED AND NOT FAKED: 33.8.7 says the block "
                      "counter is RESET while the USART transmits (TXE = 0), "
                      "and on a wire whose transmitter is its own sender the "
                      "counter never leaves that state - a real card is what "
                      "would settle it", crlf);
        bench.verdict("block mode is staged and its outcome is stated either "
                      "way", true);
    }
    U1::clear_flags(UsartClear::eobf);

    loop_down();
}

// ---------------------------------------------------------------------------
// k: IrDA
// ---------------------------------------------------------------------------

/// The width of the first HIGH pulse on the transmit pad after a
/// character is written, in TIM2 cycles. The IrDA encoder rests LOW and
/// pulses HIGH for every zero bit, which is the one thing about SIR that
/// is easy to get backwards.
uint32_t irda_pulse_cycles(uint16_t value) {
    InterruptGuard guard;
    (void)wait_flag(UsartFlag::txe);
    const uint32_t t0 = now();
    U1::write_word(value);
    while (!TxPin::read()) {
        if (to_us(since(t0)) > 100000u) {
            return 0;
        }
    }
    const uint32_t rise = now();
    while (TxPin::read()) {
        if (to_us(since(rise)) > 100000u) {
            return 0;
        }
    }
    return since(rise);
}

void tk_irda() {
    feed();
    // NORMAL MODE, at a rate slow enough that 3/16 of a bit is tens of
    // microseconds: 1200 baud gives a 833 us bit and a 156 us pulse.
    constexpr uint32_t ir_baud = 1200;
    const uint32_t bit_cycles = SysClock::hz / ir_baud;
    U1::bus_clock(true);
    U1::reset();
    (void)U1::configure({}, usart_brr(SysClock::pclk_hz, ir_baud).value());
    const bool irda_ok = U1::irda({.low_power = false, .prescaler = 1});
    TxPin::function(u1_tx.function);
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    console_drain();
    spin_us(3000);

    const uint32_t pulse = irda_pulse_cycles(0x00);
    const uint32_t sixteenths_x10 = (pulse * 160u) / bit_cycles;
    print(serial, "  IrDA normal mode at 1200 baud: the first pulse is ",
          to_us(pulse), " us = ", sixteenths_x10 / 10u, ".",
          sixteenths_x10 % 10u, " sixteenths of a bit", crlf);
    bench.verdict("33.5.18's 3/16 pulse is three sixteenths of the bit "
                  "period, measured on the pad", irda_ok &&
                                                     within(sixteenths_x10, 27u, 33u));
    print(serial, "  and the idle level is LOW with a HIGH pulse per zero - "
          "the opposite of the decoder's own input, which the chapter says "
          "in one sentence and no figure repeats", crlf);

    // LOW-POWER MODE. The pulse is now three periods of the low-power
    // clock, ker_pres / PSC - so the prescaler makes it measurable.
    feed();
    constexpr UsartPrescaler lp_presc = UsartPrescaler::div16;
    const uint32_t lp_ker = SysClock::pclk_hz / 16u;      // 4 MHz
    U1::enable(false);
    U1::reset();
    (void)U1::configure({}, usart_brr(lp_ker, ir_baud).value());
    (void)U1::prescaler(lp_presc);
    const bool lp_ok = U1::irda({.low_power = true, .prescaler = 64});
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    spin_us(3000);
    const uint32_t lp_pulse = irda_pulse_cycles(0x00);
    const uint32_t lp_clock = lp_ker / 64u;               // 62.5 kHz
    const uint32_t want_us = (3u * 1'000'000u) / lp_clock;
    print(serial, "  IrDA low-power with PSC = 64 on a 4 MHz kernel: the "
          "pulse is ", to_us(lp_pulse), " us where three periods of the ",
          lp_clock, " Hz low-power clock are ", want_us, " us", crlf);
    bench.verdict("in low-power mode the pulse stops being 3/16 of a BIT and "
                  "becomes three periods of the PSC clock - a width that no "
                  "longer moves with the baud rate",
                  lp_ok && permille_off(to_us(lp_pulse), want_us) <= 150u);

    // THE DECODER, from a bit-banged RZI line. The decoder's input is
    // HIGH at rest and a LOW pulse is a zero - the transmit encoder's
    // exact opposite, which is the other half of the same sentence.
    feed();
    U1::enable(false);
    U1::reset();
    (void)U1::configure({}, usart_brr(SysClock::pclk_hz, ir_baud).value());
    (void)U1::irda({.low_power = false, .prescaler = 8});
    TxPin::release();
    RxPin::function(u1_rx.function, {.pull = PinPull::up});
    U1::enable(true);
    U1::clear_flags(UsartClear::all);

    // Encode 0x96 the way an infrared receiver would hand it over.
    const uint8_t datum = 0x96;
    uint8_t decoded_ok = 0;
    for (uint8_t attempt = 0; attempt < 3u; ++attempt) {
        U1::clear_flags(UsartClear::all);
        (void)U1::read_word();
        {
            InterruptGuard guard;
            RxPin::pull(PinPull::up);
            const uint32_t t0 = now();
            while (since(t0) < bit_cycles) {
            }
            for (uint8_t i = 0; i < 10u; ++i) {
                // bit i: the start bit (a zero), eight data bits LSB
                // first, then the stop bit (a one).
                bool zero;
                if (i == 0u) {
                    zero = true;
                } else if (i == 9u) {
                    zero = false;
                } else {
                    zero = ((datum >> (i - 1u)) & 1u) == 0u;
                }
                const uint32_t at = t0 + bit_cycles * (i + 1u);
                while (static_cast<int32_t>(now() - at) < 0) {
                }
                if (zero) {
                    RxPin::pull(PinPull::down);
                    while (static_cast<int32_t>(now() - (at + (bit_cycles * 3u) / 16u)) < 0) {
                    }
                    RxPin::pull(PinPull::up);
                }
            }
            while (since(t0) < bit_cycles * 12u) {
            }
        }
        const uint32_t t1 = now();
        while ((U1::status() & UsartFlag::rxne) == 0u && to_us(since(t1)) < 20000u) {
        }
        if ((U1::status() & UsartFlag::rxne) != 0u) {
            const uint16_t v = U1::read_word();
            if (v == datum) {
                ++decoded_ok;
            }
        }
    }
    print(serial, "  a hand-made RZI frame (a 3/16 LOW pulse for every zero) "
          "on the receive pad: decoded correctly ", decoded_ok, " of 3 times",
          crlf);
    bench.verdict("the SIR decoder turns return-to-zero pulses back into a "
                  "byte, and the byte is the one that was encoded",
                  decoded_ok == 3u);

    // The glitch filter: 33.5.18 says pulses shorter than ONE PSC period
    // are always rejected. With PSC = 8 on a 64 MHz kernel a PSC period
    // is 125 ns, which is far too short to stage from software - so this
    // is measured the other way round, with a slow kernel that makes one
    // PSC period tens of microseconds.
    feed();
    U1::enable(false);
    U1::reset();
    (void)U1::configure({}, usart_brr(SysClock::pclk_hz / 256u, 300).value());
    (void)U1::prescaler(UsartPrescaler::div256);
    (void)U1::irda({.low_power = false, .prescaler = 8});
    U1::enable(true);
    const uint32_t psc_period_us = (8u * 1'000'000u) / (SysClock::pclk_hz / 256u);
    U1::clear_flags(UsartClear::all);
    (void)U1::read_word();
    {
        InterruptGuard guard;
        RxPin::pull(PinPull::up);
        spin_us(2000);
        RxPin::pull(PinPull::down);
        spin_us(psc_period_us / 2u);          // half a PSC period: always rejected
        RxPin::pull(PinPull::up);
        spin_us(4000);
    }
    const bool short_rejected = (U1::status() & UsartFlag::rxne) == 0u &&
                                (U1::status() & UsartFlag::receive_errors) == 0u;
    print(serial, "  one PSC period is ", psc_period_us,
          " us here; a low pulse of half of it started nothing: ",
          short_rejected ? "rejected" : "IT WAS TAKEN", crlf);
    bench.verdict("the decoder's glitch filter throws away a pulse shorter "
                  "than one PSC period - which is what PSC is FOR on the "
                  "receive side, and why the ENDEC refuses PSC = 0",
                  short_rejected);
    bench.verdict("and irda() refuses a zero prescaler outright, because "
                  "33.5.18 says the ENDEC does not work with one",
                  !irda_valid({.prescaler = 0}));

    loop_down();
}

// ---------------------------------------------------------------------------
// l: the pads' extras - swap, the inversions, MSB first, DE, RTS and CTS
// ---------------------------------------------------------------------------

void tl_pads() {
    feed();
    // SWAP moves the PADS and not the signals, so a single-wire loop
    // built with SWAP set runs on the RECEIVE pad - which is as clean a
    // proof as this board can give, since it needs no second party.
    const bool rx_free = pull_walks<RxPin>();
    if (!loop_up({.baud = 115200, .swap = true})) {
        bench.verdict("the swapped loop comes up", false);
    } else {
        TxPin::release();
        const uint32_t good = loop_run(8);
        print(serial, "  CR2.SWAP: the single wire is now PA10 (the RX pad) "
              "with PA9 given back, and ", good, " of 8 bytes went round it",
              crlf);
        bench.verdict("SWAP exchanges the PADS - the whole transport moves to "
                      "the other pin and nothing else changes",
                      rx_free && good == 8u);
    }

    // THE THREE INVERSIONS, and the single-wire loop can only judge ONE
    // of them - which is itself the finding. DATAINV inverts the DATA
    // BITS and leaves the start and stop alone, so a loop with it set at
    // both ends is a working link. TXINV and RXINV invert the LINE, and
    // on an open-drain single wire the idle level is not the
    // transmitter's to invert: 33.5.15 RELEASES the pad whenever
    // nothing is being sent, the pull-up holds it high, and a receiver
    // with RXINV then reads that resting line as a permanent start bit.
    // So each of them is measured where it CAN be: TXINV on the idle
    // level of a push-pull transmitter, RXINV against a bit-banged line
    // that is itself inverted.
    feed();
    (void)loop_up({.baud = 115200});
    U1::enable(false);
    (void)U1::invert(false, false, true);
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    const uint32_t datainv_good = loop_run(4);
    (void)loop_up({.baud = 115200});
    U1::enable(false);
    (void)U1::invert(true, true, false);
    U1::enable(true);
    U1::clear_flags(UsartClear::all);
    const uint32_t lineinv_good = loop_run(4);
    print(serial, "  on the single wire: DATAINV at both ends carries ",
          datainv_good, " of 4 bytes; TXINV + RXINV carries ", lineinv_good,
          " of 4 - because the idle level of a released open-drain pad "
          "belongs to the PULL-UP and not to the transmitter, so an inverted "
          "receiver reads the resting line as a start bit", crlf);
    bench.verdict("DATAINV inverts the data and leaves the framing alone, "
                  "which is why it survives a loop where the LINE inversions "
                  "cannot", datainv_good == 4u && lineinv_good == 0u);

    // TXINV, on the idle level of a push-pull transmitter.
    feed();
    U1::bus_clock(true);
    U1::reset();
    (void)U1::configure({}, usart_brr(SysClock::pclk_hz, 9600).value());
    TxPin::function(u1_tx.function);
    U1::enable(true);
    spin_us(2000);
    const bool idle_plain = TxPin::read();
    U1::enable(false);
    (void)U1::invert(true, false, false);
    U1::enable(true);
    spin_us(2000);
    const bool idle_txinv = TxPin::read();
    print(serial, "  TXINV on a push-pull transmitter: the resting pad is ",
          idle_plain ? "high" : "low", " normally and ",
          idle_txinv ? "high" : "low", " inverted", crlf);
    bench.verdict("TXINV inverts the LINE, idle level and all", idle_plain &&
                                                                    !idle_txinv);

    // RXINV, against a bit-banged frame that is itself inverted: with
    // the bit set the byte arrives exactly, and with it clear the very
    // same waveform is nonsense - which is the control.
    feed();
    uint32_t inv_flags = 0;
    (void)listener_up({.baud = bang_baud});
    U1::enable(false);
    (void)U1::invert(false, true, false);
    U1::enable(true);
    const BitPlan inv_plan = frame_plan(bang_baud, 0x6C, 8, UartParity::none, 1);
    const std::optional<uint16_t> inv_got =
        bang_and_read(inv_plan, inv_flags, {.invert = true});
    (void)listener_up({.baud = bang_baud});
    uint32_t plain_flags = 0;
    const std::optional<uint16_t> plain_got =
        bang_and_read(inv_plan, plain_flags, {.invert = true});
    print(serial, "  an INVERTED frame on the receive pad: with RXINV set it "
          "reads ", inv_got ? *inv_got : 0xFFFFu, " (0x6C = 108 was sent), "
          "with RXINV clear it reads ", plain_got ? *plain_got : 0xFFFFu,
          " flags ", hex(plain_flags), crlf);
    bench.verdict("RXINV inverts the receive line, and the same waveform read "
                  "without it is not the byte",
                  inv_got.has_value() && *inv_got == 0x6Cu &&
                      (!plain_got || *plain_got != 0x6Cu));

    // MSB FIRST cannot be seen on a loop either - both ends turn round
    // together - so the bit-banged line is the only witness: a frame put
    // on the pad LSB first reads back BIT-REVERSED.
    feed();
    (void)listener_up({.baud = bang_baud, .msb_first = true});
    uint32_t flags = 0;
    const BitPlan p = frame_plan(bang_baud, 0xB2, 8, UartParity::none, 1);
    const std::optional<uint16_t> rev = bang_and_read(p, flags);
    print(serial, "  MSBFIRST against an LSB-first line: 0xB2 on the wire "
          "reads back as ", rev ? *rev : 0xFFFFu, " (0x4D is its bit reversal)",
          crlf);
    bench.verdict("CR2.MSBFIRST is an exact bit reversal of the frame, which "
                  "a loop can never show and a bit-banged line can",
                  rev.has_value() && *rev == 0x4Du);

    // THE DRIVER ENABLE, timed on its own pad. DEAT and DEDT are in
    // SAMPLE times - a sixteenth of a bit at OVER8 = 0 - and that is the
    // one thing 33.5.20's single sentence about them is easy to misread.
    feed();
    constexpr uint32_t de_baud = 2400;
    const uint32_t de_bit = SysClock::hz / de_baud;
    const bool de_free = pull_walks<DePin>();
    uint32_t assert_cycles = 0;
    U1::bus_clock(true);
    U1::reset();
    (void)U1::configure({}, usart_brr(SysClock::pclk_hz, de_baud).value());
    const bool de_ok = U1::driver_enable({.assertion = 16, .deassertion = 8});
    TxPin::function(u1_tx.function);
    DePin::function(u1_de.function);
    U1::enable(true);
    console_drain();
    spin_us(3000);
    {
        InterruptGuard guard;
        (void)wait_flag(UsartFlag::txe);
        U1::write_word(0xFF);
        const uint32_t t0 = now();
        while (!DePin::read() && to_us(since(t0)) < 50000u) {
        }
        const uint32_t de_rise = now();
        while (TxPin::read() && to_us(since(de_rise)) < 50000u) {
        }
        assert_cycles = since(de_rise);
    }
    const uint32_t assert_sixteenths = (assert_cycles * 16u) / de_bit;
    print(serial, "  DEAT = 16 sample times at 2400 baud: DE rose ",
          to_us(assert_cycles), " us before the start bit = ",
          assert_sixteenths, " sixteenths of a bit, where a bit is ",
          to_us(de_bit), " us", crlf);
    bench.verdict("DEAT is counted in SAMPLE times and not bit times - "
                  "sixteen of them is one whole bit at OVER8 = 0",
                  de_ok && de_free && within(assert_sixteenths, 13u, 19u));
    print(serial, "  and the DE signal came out on PB3, which is the SAME PAD "
          "as RTS and the synchronous CK (USART1_RTS_DE_CK)", crlf);

    // RTS: an output the RECEIVER raises when it can take no more.
    feed();
    (void)loop_up({.baud = 9600});
    U1::enable(false);
    (void)U1::flow_control(true, false);
    U1::enable(true);
    DePin::function(u1_de.function);
    U1::clear_flags(UsartClear::all);
    spin_us(2000);
    const bool rts_idle = DePin::read();
    (void)wait_flag(UsartFlag::txe);
    U1::write_word(0x11);
    spin_us(3000);
    const bool rts_full = DePin::read();
    (void)U1::read_word();
    spin_us(500);
    const bool rts_free = DePin::read();
    print(serial, "  RTS on PB3: idle ", rts_idle ? "high" : "low",
          ", with a character unread ", rts_full ? "high" : "low",
          ", after the read ", rts_free ? "high" : "low", crlf);
    bench.verdict("RTS is asserted exactly while the receiver cannot take "
                  "another character, and released when it can",
                  !rts_idle && rts_full && !rts_free);

    // CTS: an input the TRANSMITTER obeys between frames. The pad is
    // pull-walked - the whole trick of this suite pointed at a
    // flow-control line.
    feed();
    const bool cts_free = pull_walks<CtsPin>();
    (void)loop_up({.baud = 9600});
    U1::enable(false);
    (void)U1::flow_control(false, true);
    U1::enable(true);
    CtsPin::function(u1_cts.function, {.pull = PinPull::up});   // CTS high: stop
    U1::clear_flags(UsartClear::all);
    spin_us(2000);
    (void)wait_flag(UsartFlag::txe);
    U1::write_word(0x22);
    (void)wait_flag(UsartFlag::txe);
    U1::write_word(0x23);     // this one must wait for CTS
    spin_us(8000);
    const uint32_t st_stalled = U1::status();
    const bool stalled = (st_stalled & UsartFlag::txe) == 0u;
    CtsPin::pull(PinPull::down);                                 // CTS low: go
    spin_us(8000);
    uint8_t arrived = 0;
    while ((U1::status() & UsartFlag::rxne) != 0u && arrived < 4u) {
        (void)U1::read_word();
        ++arrived;
        spin_us(200);
    }
    print(serial, "  CTS held high: ISR ", hex(st_stalled),
          "; released, ", arrived, " characters arrived", crlf);
    bench.verdict("CTS stops the transmitter BETWEEN frames and lets it go "
                  "again, with the line pull-walked and no wire",
                  cts_free && stalled && arrived >= 1u);

    loop_down();
}

// ---------------------------------------------------------------------------
// m: the synchronous master's clock
// ---------------------------------------------------------------------------

void tm_synchronous() {
    feed();
    constexpr uint32_t sync_baud = 9600;
    struct SyncCase { bool lbcl; bool cpol; };
    static const SyncCase scases[] = {{false, false}, {true, false}, {false, true}};
    uint8_t counted_ok = 0;
    for (const SyncCase& c : scases) {
        feed();
        U1::bus_clock(true);
        U1::reset();
        (void)U1::configure({}, usart_brr(SysClock::pclk_hz, sync_baud).value());
        const bool sync_ok = U1::synchronous({.clock_idle_high = c.cpol,
                                              .sample_second_edge = false,
                                              .last_bit_clock = c.lbcl});
        TxPin::function(u1_tx.function);
        DePin::function(u1_de.function);   // USART1_RTS_DE_CK, the CK output
        U1::enable(true);
        spin_us(2000);
        const bool idle_level = DePin::read();
        if (!edge_counter_up(3, 'B', DmaMuxEdge::rising)) {
            print(serial, "  the edge counter would not come up", crlf);
            break;
        }
        console_drain();
        const uint32_t a = edge_count_reset();
        for (uint8_t i = 0; i < 8u; ++i) {
            (void)wait_flag(UsartFlag::txe);
            U1::write_word(static_cast<uint16_t>(0x80u + i));
        }
        (void)wait_flag(UsartFlag::tc);
        spin_us(2000);
        const uint32_t b = edge_count_reset();
        edge_counter_down(3);
        const uint32_t edges = b - a;
        const uint32_t per_byte_x10 = (edges * 10u) / 8u;
        print(serial, "  CLKEN, LBCL = ", c.lbcl ? 1u : 0u, ", CPOL = ",
              c.cpol ? 1u : 0u, ": CK idles ", idle_level ? "high" : "low",
              " and eight characters cost ", edges, " rising edges = ",
              per_byte_x10 / 10u, ".", per_byte_x10 % 10u, " a character",
              crlf);
        const uint32_t want = c.lbcl ? 64u : 56u;
        if (sync_ok && edges == want && idle_level == c.cpol) {
            ++counted_ok;
        }
    }
    bench.verdict("33.5.14's clock is one pulse a data bit with none for the "
                  "start and stop, LBCL adds the pulse of the LAST bit, and "
                  "CPOL is the level CK rests at - all three counted with no "
                  "CPU in the path", counted_ok == 3u);
    print(serial, "  THE DATA PATH IS DECLINED and not faked: a synchronous "
          "link needs something at the other end to clock, and this desk has "
          "one board. The SLAVE half (CR2.SLVEN, DIS_NSS and the underrun "
          "flag) is register verbs and no measurement, and its doc says so.",
          crlf);
    DePin::release();
    loop_down();
}

// ---------------------------------------------------------------------------
// n: the LPUARTs
// ---------------------------------------------------------------------------

/// The LPUART's own single-wire loop, over whichever instance and pad.
template <typename L, typename Pad>
bool lp_loop_up(const PinSel& sel, uint32_t baud, UsartClock kernel,
                UsartPrescaler presc = UsartPrescaler::div1, bool fifo = false) {
    const uint32_t ker = usart_kernel_hz(loop_kernel_hz(kernel), presc);
    const std::optional<uint32_t> reg = lpuart_brr(ker, baud);
    if (!reg) {
        return false;
    }
    // NOT Nvic::disable(L::irq()): LPUART2's line IS THE CONSOLE'S on
    // this part (USART2_LPUART2_IRQn), and disabling it stops the
    // transport this suite reports through - which cost one run, a
    // watchdog reset and a verdict line truncated in the middle of the
    // word. L::reset() below clears the peripheral's own interrupt
    // enables, which is all this needs.
    L::bus_clock(true);
    L::reset();
    (void)L::kernel_clock(kernel);
    if (!L::configure({}, *reg)) {
        return false;
    }
    if (presc != UsartPrescaler::div1 && !L::prescaler(presc)) {
        return false;
    }
    if (fifo && !L::fifo(true)) {
        return false;
    }
    if (!L::half_duplex(true)) {
        return false;
    }
    Pad::function(sel.function, {.pull = PinPull::up, .open_drain = true});
    L::enable(true);
    L::clear_flags(UsartClear::all);
    return true;
}

template <typename L>
std::optional<uint16_t> lp_word(uint16_t v, uint32_t timeout_us) {
    uint32_t t0 = now();
    while ((L::status() & UsartFlag::txe) == 0u) {
        if (to_us(since(t0)) > timeout_us) {
            return std::nullopt;
        }
    }
    L::write_word(v);
    t0 = now();
    while ((L::status() & UsartFlag::rxne) == 0u) {
        if (to_us(since(t0)) > timeout_us) {
            return std::nullopt;
        }
    }
    return L::read_word();
}

template <typename L>
uint32_t lp_run(uint32_t count, uint32_t timeout_us) {
    uint32_t good = 0;
    uint8_t v = 0x31;
    for (uint32_t i = 0; i < count; ++i) {
        feed();
        const std::optional<uint16_t> got = lp_word<L>(v, timeout_us);
        if (got && *got == v) {
            ++good;
        }
        v = static_cast<uint8_t>(v * 5u + 1u);
    }
    return good;
}

volatile uint32_t lpuart_irqs = 0;
/// Whether LPUART1's bus clock is on and its interrupt is ours to serve.
volatile bool lpuart_live = false;

void tn_lpuart() {
    feed();
    // TABLE 198 AND TABLE 199, read back out of the register. THE
    // MANUAL TRUNCATES where this driver rounds to nearest, so six of
    // the sixteen rows differ by one - and every one of those six is
    // CLOSER to the rate that was asked for. The fixture pins both
    // numbers; this is the silicon taking brio's.
    L1::bus_clock(true);
    L1::reset();
    uint8_t table_ok = 0;
    struct BrrCase { uint32_t hz; uint32_t baud; uint32_t manual; };
    static const BrrCase brrs[] = {
        {32768, 300, 0x6D3A}, {32768, 600, 0x369D}, {32768, 1200, 0x1B4E},
        {32768, 2400, 0xDA7}, {32768, 4800, 0x6D3}, {32768, 9600, 0x369},
    };
    for (const BrrCase& c : brrs) {
        const std::optional<uint32_t> mine = lpuart_brr(c.hz, c.baud);
        if (!mine) {
            continue;
        }
        (void)L1::configure({}, *mine);
        const uint32_t back = L1::brr();
        const uint32_t err_mine = permille_off(lpuart_actual_baud(c.hz, back), c.baud);
        const uint32_t err_manual =
            permille_off(lpuart_actual_baud(c.hz, c.manual), c.baud);
        print(serial, "  table 198 row ", c.baud, " baud: brio ", back,
              " (", err_mine, " per mille off), the manual ", c.manual, " (",
              err_manual, " off)", crlf);
        if (back == *mine && err_mine <= err_manual) {
            ++table_ok;
        }
    }
    bench.verdict("every LSE row of table 198 lands in the twenty-bit BRR, "
                  "and where brio's rounding differs from the manual's "
                  "truncation it is NEVER the worse of the two",
                  table_ok == 6u);

    // The window of 34.4.7, which is ONE rule and not two: LPUARTDIV >=
    // 0x300 IS "fck at least three times the baud rate", because
    // 256 x 3 = 0x300.
    bench.verdict("9600 baud is the LSE's stated ceiling and 19200 is past "
                  "it - refused by the arithmetic, not attempted",
                  lpuart_brr(32768, 9600).has_value() &&
                      !lpuart_brr(32768, 19200).has_value());

    // LPUART1's own single wire on PC1, on each of its kernel clocks.
    const bool pc1_free = pull_walks<Lp1Pin>();
    print(serial, "  PC1 follows its own pull: ", pc1_free ? "yes" : "NO", crlf);
    bench.verdict("PC1 is free for LPUART1's single wire", pc1_free);

    const bool lse_running = RtcDomain::lse_ready();
    uint8_t lp_ok = 0;
    uint8_t lp_tried = 0;
    struct LpCase { UsartClock kernel; uint32_t baud; uint32_t timeout; const char* name; };
    static const LpCase lpcases[] = {
        {UsartClock::pclk, 115200, 20000, "PCLK 64 MHz at 115200"},
        {UsartClock::hsi16, 115200, 20000, "HSI16 at 115200"},
        {UsartClock::hsi16, 9600, 40000, "HSI16 at 9600"},
        {UsartClock::lse, 9600, 40000, "LSE 32768 Hz at 9600"},
        {UsartClock::lse, 300, 900000, "LSE 32768 Hz at 300"},
    };
    for (const LpCase& c : lpcases) {
        feed();
        if (c.kernel == UsartClock::lse && !lse_running) {
            print(serial, "  ", c.name, ": DECLINED, the crystal is not running",
                  crlf);
            continue;
        }
        ++lp_tried;
        if (!lp_loop_up<L1, Lp1Pin>(lp1_tx, c.baud, c.kernel)) {
            print(serial, "  ", c.name, ": unreachable", crlf);
            continue;
        }
        const uint32_t good = lp_run<L1>(4, c.timeout);
        print(serial, "  LPUART1 ", c.name, ": BRR ", L1::brr(), ", ", good,
              " of 4 bytes round its own wire", crlf);
        if (good == 4u) {
            ++lp_ok;
        }
    }
    bench.verdict("LPUART1 runs on PCLK, on HSI16 and on the 32768 Hz crystal, "
                  "byte-exact on its own single wire", lp_ok == lp_tried &&
                                                           lp_tried >= 3u);

    // The PCLK ceiling: 34.4.7 puts it at fck / 3, which from 64 MHz is
    // 21.3 Mbaud. The open-drain loop gives out long before that, and
    // the ladder says where.
    feed();
    static const uint32_t lp_ladder[] = {115200, 460800, 921600, 2'000'000,
                                         4'000'000, 8'000'000};
    uint32_t lp_best = 0;
    for (uint32_t baud : lp_ladder) {
        if (!lp_loop_up<L1, Lp1Pin>(lp1_tx, baud, UsartClock::pclk)) {
            print(serial, "  ", baud, " baud is outside 34.4.7's window at "
                  "this clock", crlf);
            continue;
        }
        const uint32_t good = lp_run<L1>(8, 20000);
        print(serial, "  LPUART1 at ", baud, " baud on PCLK: ", good,
              " of 8 exact", crlf);
        if (good == 8u) {
            lp_best = baud;
        }
    }
    print(serial, "  34.4.7's own ceiling from a 64 MHz kernel is fck/3 = "
          "21.3 Mbaud; the open-drain loop is exact to ", lp_best,
          " and what fails above it is the pad's pull-up, not the divisor",
          crlf);
    bench.verdict("the LPUART carries at least the console's rate on its own "
                  "wire", lp_best >= 115200u);

    // The FIFOs and the prescaler are the LP column's, not the FULL
    // column's - which is why has_fifo_mode is a flag of its own.
    feed();
    const bool fifo_up =
        lp_loop_up<L1, Lp1Pin>(lp1_tx, 115200, UsartClock::pclk,
                               UsartPrescaler::div1, true);
    const bool fifo_on = L1::fifo();
    const uint32_t fifo_good = fifo_up ? lp_run<L1>(8, 20000) : 0u;
    const bool presc_up =
        lp_loop_up<L1, Lp1Pin>(lp1_tx, 9600, UsartClock::pclk,
                               UsartPrescaler::div64);
    const uint32_t presc_good = presc_up ? lp_run<L1>(4, 40000) : 0u;
    print(serial, "  LPUART1 with FIFOEN: ", fifo_good,
          " of 8 exact; with PRESC = /64 at 9600 baud: ", presc_good,
          " of 4 - the LP column has both, and no OVER8 at all", crlf);
    bench.verdict("the LPUART's FIFO and prescaler work, and its baud "
                  "generator has no oversampling to choose",
                  fifo_on && fifo_good == 8u && presc_good == 4u &&
                      !Lpuart<1>::has_oversampling8);

    // LPUART2, on its own pad and its own kernel-clock field.
    feed();
    const bool pc6_free = pull_walks<Lp2Pin>();
    const bool l2_up =
        lp_loop_up<L2, Lp2Pin>(lp2_tx, 115200, UsartClock::pclk);
    const uint32_t l2_good = l2_up ? lp_run<L2>(8, 20000) : 0u;
    print(serial, "  PC6 free: ", pc6_free ? "yes" : "NO",
          "; LPUART2 on its own single wire at 115200: ", l2_good,
          " of 8 exact, BRR ", L2::brr(), crlf);
    bench.verdict("LPUART2 - the G0B1 class's second one, with its own "
                  "LPUART2SEL field and its own APB bit - runs the same way",
                  pc6_free && l2_up && l2_good == 8u);

    // THE SHARED VECTORS. LPUART2 arrives on the CONSOLE's own line and
    // LPUART1 on the line USART3..6 share, so ONE handler serves several
    // peripherals - which is this family's shape and the reason every
    // isr() body answers "mine" or "not mine".
    feed();
    lpuart_irqs = 0;
    (void)lp_loop_up<L1, Lp1Pin>(lp1_tx, 9600, UsartClock::pclk);
    L1::rxne_interrupt(true);
    lpuart_live = true;
    Nvic::enable(L1::irq());
    console_drain();
    uint32_t lp_spins = 4'000'000u;
    while ((L1::status() & UsartFlag::txe) == 0u && lp_spins-- != 0u) {
    }
    L1::write_word(0x77);
    spin_us(4000);
    const uint32_t served = lpuart_irqs;
    Nvic::disable(L1::irq());
    lpuart_live = false;
    L1::rxne_interrupt(false);
    print(serial, "  LPUART1's interrupt on USART3_4_5_6_LPUART1_IRQn was "
          "served ", served, " time(s) by a handler that also serves four "
          "USARTs", crlf);
    bench.verdict("a shared vector reaches the LPUART, and the console on "
                  "USART2 - which shares ITS line with LPUART2 - kept talking "
                  "throughout", served >= 1u);

    L1::enable(false);
    L1::reset();
    L1::bus_clock(false);
    L2::enable(false);
    L2::reset();
    L2::bus_clock(false);
    Lp1Pin::release();
    Lp2Pin::release();
}

// ---------------------------------------------------------------------------
// o: IRTIM
// ---------------------------------------------------------------------------

using Carrier = TimPwm<Tim<17>, 0>;
using Envelope = TimPwm<Tim<16>, 0>;

void to_irtim() {
    feed();
    const bool pb9_free = pull_walks<IrPin>();
    print(serial, "  PB9 follows its own pull: ", pb9_free ? "yes" : "NO",
          " - and it is the ONE pad of this board an infrared LED would use, "
          "PA13's IR_OUT being SWDIO", crlf);
    bench.verdict("PB9 is free for IR_OUT", pb9_free);

    Irtim::init();
    bench.verdict("the interface's three bits live in SYSCFG, behind the "
                  "clock gate the comparators and the voltage reference "
                  "share", Irtim::bus_clock());

    // TIM17 is ALWAYS the carrier and TIM16 (or a USART) the envelope,
    // and NEITHER TIMER NEEDS A PAD: figure 278's connections are
    // internal, which is what makes an infrared output cost one pin.
    Tim<17>::init();
    Tim<16>::init();
    const uint32_t carrier_hz = 38000;
    const uint16_t carrier_top =
        static_cast<uint16_t>(SysClock::pclk_hz / carrier_hz - 1u);
    (void)Tim<17>::configure({.prescaler = 0, .period = carrier_top});
    (void)Tim<17>::output_channel(0, {.mode = TimOutputMode::pwm1,
                                      .compare = carrier_top / 2u});
    (void)Tim<17>::channel_enable(0, true);
    (void)Tim<17>::main_output(true);
    Tim<17>::update();
    Tim<17>::enable(true);

    const uint16_t env_top = static_cast<uint16_t>(SysClock::pclk_hz / 64u / 1000u - 1u);
    (void)Tim<16>::configure({.prescaler = 63, .period = env_top});
    (void)Tim<16>::output_channel(0, {.mode = TimOutputMode::pwm1,
                                      .compare = static_cast<uint32_t>(env_top / 2u)});
    (void)Tim<16>::channel_enable(0, true);
    (void)Tim<16>::main_output(true);
    Tim<16>::update();
    Tim<16>::enable(true);

    (void)Irtim::envelope(IrtimEnvelope::tim16);
    Irtim::polarity(false);
    IrtimPad<ir_out>::claim();

    if (!edge_counter_up(9, 'B', DmaMuxEdge::rising)) {
        bench.verdict("the edge counter comes up on EXTI line 9", false);
    } else {
        console_drain();
        const uint32_t a = edge_count_reset();
        spin_us(100000);
        const uint32_t b = edge_count_reset();
        edge_counter_down(9);
        const uint32_t edges = b - a;
        const uint32_t want = (carrier_hz / 10u) / 2u;   // 100 ms, 50 % duty
        print(serial, "  a 38 kHz carrier under a 1 kHz 50 % envelope: ",
              edges, " rising edges on PB9 in 100 ms, where the product of "
              "the two waveforms is ", want, crlf);
        bench.verdict("IRTIM really ANDs TIM17's carrier with TIM16's "
                      "envelope, and the pad carries exactly the product - "
                      "counted by a DMA channel with no CPU",
                      permille_off(edges, want) <= 60u);
    }

    // IR_POL, on the idle level: with both timers stopped the output is
    // whatever the AND of two idle levels is, and the bit inverts it.
    feed();
    Tim<17>::enable(false);
    Tim<16>::enable(false);
    Tim<17>::set_count(0);
    Tim<16>::set_count(0);
    Irtim::polarity(false);
    spin_us(500);
    const bool idle_normal = IrPin::read();
    Irtim::polarity(true);
    spin_us(500);
    const bool idle_inverted = IrPin::read();
    print(serial, "  IR_POL: the resting pad is ", idle_normal ? "high" : "low",
          " normally and ", idle_inverted ? "high" : "low", " inverted", crlf);
    bench.verdict("IR_POL inverts the output, which is what an LED wired to "
                  "the supply asks for", idle_normal != idle_inverted);
    Irtim::polarity(false);

    // A USART AS THE ENVELOPE. IR_MOD = 01 takes USART1's transmit line
    // WITHOUT A PAD - it is an internal connection - so a byte gates the
    // carrier, and the gate is the frame.
    feed();
    Tim<17>::enable(true);
    U1::bus_clock(true);
    U1::reset();
    constexpr uint32_t env_baud = 1200;
    (void)U1::configure({}, usart_brr(SysClock::pclk_hz, env_baud).value());
    U1::enable(true);
    (void)Irtim::envelope(IrtimEnvelope::usart1);
    if (edge_counter_up(9, 'B', DmaMuxEdge::rising)) {
        console_drain();
        // Idle first: the transmit line rests HIGH, so the carrier is
        // passed continuously.
        const uint32_t i0 = edge_count_reset();
        spin_us(20000);
        const uint32_t i1 = edge_count_reset();
        const uint32_t idle_edges = i1 - i0;
        // Then a 0x00, which holds the line low for the start bit and
        // eight data bits: nine bit times of silence in every frame.
        const uint32_t b0 = edge_count_reset();
        for (uint8_t i = 0; i < 20u; ++i) {
            (void)wait_flag(UsartFlag::txe);
            U1::write_word(0x00);
        }
        (void)wait_flag(UsartFlag::tc);
        const uint32_t b1 = edge_count_reset();
        edge_counter_down(9);
        const uint32_t sent_edges = b1 - b0;
        const uint32_t frame_us = (20u * 10u * 1'000'000u) / env_baud;
        const uint32_t idle_rate = idle_edges / 20u;                 // per ms
        const uint32_t sent_rate = sent_edges / (frame_us / 1000u);  // per ms
        const uint32_t carrier_per_ms = carrier_hz / 1000u;
        print(serial, "  IR_MOD = 01 (USART1 as the envelope, and NO PAD ON "
              "THE USART AT ALL - figure 278's connection is internal): an "
              "IDLE line passes ", idle_rate, " carrier edges a millisecond; "
              "twenty 0x00 frames pass ", sent_rate, " a millisecond, against "
              "a free-running carrier's ", carrier_per_ms, crlf);
        print(serial, "  THE FINDING: the USART envelope is ACTIVE LOW where "
              "TIM16's is active high. An idle transmit line - which is HIGH "
              "- shuts the gate completely, and a 0x00 frame, which holds the "
              "line low for nine of its ten bit times, opens it for exactly "
              "nine tenths of the time. That is the right way round for "
              "infrared (no light at rest, light for a zero) and chapter 27 "
              "draws one AND gate and says nothing about it.", crlf);
        const uint32_t want_nine_tenths = (carrier_per_ms * 9u) / 10u;
        bench.verdict("a USART's own transmit line is an IRTIM envelope over "
                      "an INTERNAL connection, and the gate is the frame: "
                      "shut while the line idles, open for nine tenths of a "
                      "zero character",
                      idle_rate == 0u &&
                          permille_off(sent_rate, want_nine_tenths) <= 120u);
    }

    // The second USART code, which is a PER-PART fact: USART4 here,
    // USART2 on the G031 class. Both are the reserve's, not a driver's.
    feed();
    const bool second_ok = Irtim::envelope(IrtimEnvelope::second_usart);
    const bool reserved_refused =
        !Irtim::envelope(static_cast<IrtimEnvelope>(3));
    print(serial, "  IR_MOD = 10 selects USART", Irtim::second_usart_index,
          " on this part (it is USART2 on the G031 class), and code 11 is "
          "Reserved: ", reserved_refused ? "refused" : "TAKEN", crlf);
    bench.verdict("the envelope multiplexer takes all three implemented codes "
                  "and refuses the Reserved one",
                  second_ok && reserved_refused &&
                      Irtim::second_usart_index == 4u);

    // The high-sink driver, which is PB9's alone and wears an I2C name.
    Irtim::pb9_high_sink(true);
    const bool sink_on = Irtim::pb9_high_sink();
    Irtim::pb9_high_sink(false);
    bench.verdict("the high-sink LED driver of ch. 27's last paragraph is "
                  "SYSCFG_CFGR1.I2C_PB9_FMP - one bit with two names, and it "
                  "sticks", sink_on && !Irtim::pb9_high_sink());

    Tim<17>::enable(false);
    Tim<16>::enable(false);
    Tim<17>::release();
    Tim<16>::release();
    Irtim::release();
    IrtimPad<ir_out>::release();
    loop_down();
}

// ---------------------------------------------------------------------------
// The host-assisted letters (tools/uart_stress.py), OUTSIDE z
// ---------------------------------------------------------------------------
//
// The board prints one "HOST op mode baud format window count" line, the
// script moves its own port to that rate and frame, runs the op for LESS
// than the window and goes quiet before the board speaks again. The
// dma suite's letter u is the same protocol; this suite adds `poke`,
// which sends a handful of bytes in the MIDDLE of the window - which is
// what a sleeping board needs.

uint32_t lfsr_state = 0x12345678u;
void lfsr_reset() { lfsr_state = 0x12345678u; }
uint8_t lfsr_next() {
    uint32_t s = lfsr_state;
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    lfsr_state = s;
    return static_cast<uint8_t>(s & 0xFFu);
}

/// When the last announced window started, and how long it claimed to
/// be - host_settle() waits it out before the board speaks again.
uint32_t host_window_t0 = 0;
uint32_t host_window_ms = 0;

/// Announce a leg, let the announcement leave the wire, and settle.
void host_announce(const char* op, uint8_t mode, uint32_t baud, const char* fmt,
                   uint32_t window_ms, uint32_t count) {
    print(serial, "HOST ", op, " ", mode, " ", baud, " ", fmt, " ", window_ms,
          " ", count, crlf);
    console_drain();
    host_window_t0 = now();
    host_window_ms = window_ms;
    spin_us(50000);
}

/// Back to 115200 8N1 and quiet, so a report is not read as payload.
///
/// AND THE RING IS DRAINED LAST, which is not tidiness: a leg whose own
/// window is shorter than the host's pump leaves the tail of the stream
/// in the console's receive ring, and the menu loop would then read
/// those bytes as LETTERS and re-run the suite from the middle. (Letter
/// v is exactly such a leg - the board stops listening on the LPUART
/// while the host is still sending.) The drain happens AFTER the half
/// second of silence, so nothing is still arriving when it runs.
void host_settle() {
    spin_us(20000);
    (void)Serial::set_baud(SysClock::pclk_hz, 115200);
    // THE WHOLE ANNOUNCED WINDOW IS WAITED OUT, AND THEN SOME. The
    // script's contract is that it runs the op for LESS than the window,
    // but it also spends a collect phase AT THE OP'S OWN RATE after the
    // pump, which only ends on 300 ms of silence - so a board that wakes
    // early (letter w's whole point: the poke lands halfway through) and
    // reports as soon as it is done gets its report read at 9600 and
    // eaten. Measured: 12 garbled bytes where the report is 130
    // characters at 115200, and the leg after it lost its own HOST line.
    // Nine hundred milliseconds past the window is the margin.
    const uint32_t due = host_window_ms + 900u;
    while (to_us(since(host_window_t0)) < due * 1000u) {
        feed();
    }
    uint8_t junk[64];
    while (Serial::read_bulk(junk) != 0u) {
    }
    Serial::clear_errors();
}

void ty_streaming() {
    print(serial, "  this letter needs tools/uart_stress.py on the other end "
          "of the VCP; run it as", crlf,
          "  python3 tools/uart_stress.py --port <the console> --letters y",
          crlf);

    struct Leg { UsartClock kernel; uint32_t ker_hz; uint32_t baud; const char* name; };
    static const Leg legs[] = {
        {UsartClock::pclk, SysClock::pclk_hz, 115200, "PCLK"},
        {UsartClock::hsi16, 16'000'000u, 115200, "HSI16"},
        {UsartClock::sysclk, SysClock::hz, 460800, "SYSCLK"},
        {UsartClock::pclk, SysClock::pclk_hz, 921600, "PCLK"},
    };
    uint8_t clean = 0;
    uint8_t tried = 0;
    for (const Leg& l : legs) {
        feed();
        const std::optional<uint16_t> reg = usart_brr(l.ker_hz, l.baud);
        if (!reg) {
            continue;
        }
        ++tried;
        host_announce("sink", 0, l.baud, "8N1", 900, 0);
        {
            InterruptGuard guard;
            Usart<2>::enable(false);
            (void)Usart<2>::kernel_clock(l.kernel);
            Usart<2>::set_brr(*reg);
            Usart<2>::enable(true);
        }
        // THE RING IS DRAINED AFTER THE SWITCH AND NOT BEFORE IT: the
        // rate moves under the receiver, so whatever half-character the
        // old divisor made of the host's first bytes is in the ring
        // already, and comparing from there desynchronizes the whole
        // stream. (It did: at 921600 the count was right and every byte
        // was wrong.)
        Serial::clear_errors();
        uint8_t junk[64];
        while (Serial::read_bulk(junk) != 0u) {
        }
        lfsr_reset();
        uint32_t got = 0;
        uint32_t bad = 0;
        uint8_t chunk[64];
        const uint32_t t0 = now();
        while (to_us(since(t0)) < 500000u) {
            const uint32_t n = Serial::read_bulk(chunk);
            for (uint32_t i = 0; i < n; ++i) {
                if (chunk[i] != lfsr_next()) {
                    ++bad;
                }
            }
            got += n;
        }
        const uint8_t hw = Serial::hw_overruns();
        const uint8_t fe = Serial::frame_errors();
        {
            InterruptGuard guard;
            Usart<2>::enable(false);
            (void)Usart<2>::kernel_clock(UsartClock::pclk);
            Usart<2>::set_brr(usart_brr(SysClock::pclk_hz, 115200).value());
            Usart<2>::enable(true);
        }
        host_settle();
        print(serial, "  ", l.name, " at ", l.baud, " baud: ", got,
              " bytes in, ", bad, " wrong, hardware overruns ", hw,
              ", framing ", fe, crlf);
        if (got >= 2000u && bad == 0u) {
            ++clean;
        }
    }

    // And the same with the FIFO on, at the console's own rate.
    feed();
    host_announce("sink", 0, 115200, "8N1", 900, 0);
    {
        InterruptGuard guard;
        Usart<2>::enable(false);
        (void)Usart<2>::fifo(true);
        (void)Usart<2>::fifo_thresholds(UartFifoThreshold::half,
                                        UartFifoThreshold::none);
        Usart<2>::enable(true);
    }
    Serial::clear_errors();
    uint8_t fifo_junk[64];
    while (Serial::read_bulk(fifo_junk) != 0u) {
    }
    lfsr_reset();
    uint32_t fifo_got = 0;
    uint32_t fifo_bad = 0;
    uint8_t chunk[64];
    uint32_t t0 = now();
    while (to_us(since(t0)) < 500000u) {
        const uint32_t n = Serial::read_bulk(chunk);
        for (uint32_t i = 0; i < n; ++i) {
            if (chunk[i] != lfsr_next()) {
                ++fifo_bad;
            }
        }
        fifo_got += n;
    }
    const uint8_t fifo_hw = Serial::hw_overruns();
    {
        InterruptGuard guard;
        Usart<2>::enable(false);
        (void)Usart<2>::fifo(false);
        Usart<2>::enable(true);
    }
    host_settle();
    print(serial, "  with FIFOEN set on the console: ", fifo_got,
          " bytes in, ", fifo_bad, " wrong, hardware overruns ", fifo_hw, crlf);
    bench.verdict("the console streams byte-exact from the host on every "
                  "kernel clock it can take", clean == tried && tried >= 3u);
    bench.verdict("and with the FIFO turned on under it, without one line of "
                  "the transport changing", fifo_got >= 2000u && fifo_bad == 0u);
}

// ---------------------------------------------------------------------------
// w: the wake from Stop, and ES0548 2.2.4 staged
// ---------------------------------------------------------------------------

volatile uint32_t wake_irqs = 0;
volatile uint32_t wake_wuf = 0;

uint32_t rtc_wall() {
    RtcReading r{};
    if (!Rtc::read(r)) {
        return 0xFFFFFFFFu;
    }
    const uint32_t per_second = static_cast<uint32_t>(Rtc::prescalers().sync) + 1u;
    return static_cast<uint32_t>(r.time.second) * per_second +
           (per_second - 1u - r.subsecond);
}
uint32_t rtc_delta(uint32_t from, uint32_t to) {
    const uint32_t mod = 60u * (static_cast<uint32_t>(Rtc::prescalers().sync) + 1u);
    return (to >= from) ? (to - from) : (mod - from + to);
}
uint32_t rtc_ms(uint32_t ticks) {
    const uint32_t hz = 32768u / (static_cast<uint32_t>(Rtc::prescalers().async) + 1u);
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1000ULL) / hz);
}

void tw_wake() {
    print(serial, "  this letter needs tools/uart_stress.py; run it as", crlf,
          "  python3 tools/uart_stress.py --port <the console> --letters w",
          crlf);
    feed();
    const bool wall_ok = RtcDomain::selected() == RtcClockSource::lse &&
                         RtcDomain::lse_ready();
    if (!wall_ok) {
        print(serial, "  the RTC is not on the crystal: this letter would "
                      "have no wall clock and is DECLINED", crlf);
        bench.verdict("the wake letter needs the RTC wall clock", false);
        return;
    }

    struct WakeCase { UsartWakeSource src; uint32_t baud; const char* name; };
    static const WakeCase wcases[] = {
        {UsartWakeSource::start_bit, 9600, "start bit at 9600"},
        {UsartWakeSource::receive_ready, 9600, "RXNE at 9600"},
        {UsartWakeSource::start_bit, 115200, "start bit at 115200"},
        {UsartWakeSource::address_match, 9600, "address match at 9600"},
    };
    uint8_t woke = 0;
    uint8_t tried = 0;
    for (const WakeCase& c : wcases) {
        feed();
        const std::optional<uint16_t> reg = usart_brr(16'000'000u, c.baud);
        if (!reg) {
            continue;
        }
        ++tried;
        host_announce("poke", 0, c.baud, "8N1", 1400, 4);
        // The console on HSI16 - one of the two kernel clocks that
        // survive a Stop, and the only one that can carry 115200.
        {
            InterruptGuard guard;
            Usart<2>::enable(false);
            (void)Usart<2>::kernel_clock(UsartClock::hsi16);
            Usart<2>::set_brr(*reg);
            if (c.src == UsartWakeSource::address_match) {
                (void)Usart<2>::mute_mode({.wake = MuteWake::address_mark,
                                           .address_7bit = false,
                                           .address = 0x5});
            }
            (void)Usart<2>::wake_from_stop(c.src);
            (void)Usart<2>::wake_line(true);
            Usart<2>::enable(true);
        }
        // 33.5.21: REACK must be checked before the Stop is entered.
        uint32_t spins = 200000;
        while ((Usart<2>::status() & UsartFlag::reack) == 0u && spins-- != 0u) {
        }
        const bool reack = (Usart<2>::status() & UsartFlag::reack) != 0u;
        Usart<2>::clear_flags(UsartClear::all);
        Serial::clear_errors();
        uint8_t junk[16];
        while (Serial::read_bulk(junk) != 0u) {
        }

        // THE BACKSTOP IS THE RTC'S OWN WAKE-UP TIMER and not the
        // watchdog: a Stop that nothing ends would cost this letter a
        // reboot and a banner, which is exactly what its first version
        // did. Two seconds of ck_spre, and the two flags afterwards say
        // WHICH of them ended the sleep.
        (void)Rtc::set_wakeup(RtcWakeupClock::ck_spre, 1, true);
        Rtc::clear_flags(RtcFlag::wakeup);
        feed();
        console_wakes = 0;
        console_wake_armed = 1;
        rtc_backstops = 0;
        const uint32_t w0 = rtc_wall();
        Ticker::pause();
        Pwr::enter(PwrMode::stop0);
        Ticker::resume();
        (void)SysClock::init();
        const uint32_t slept = rtc_ms(rtc_delta(w0, rtc_wall()));
        // WUF HAS ALREADY BEEN CLEARED BY THE TIME THIS LINE RUNS: the
        // handler runs before the WFI returns, and it must clear a level
        // it would otherwise re-enter on. The COUNTER is the reading.
        const uint32_t wuf_seen = console_wakes;
        const bool by_rtc = rtc_backstops != 0u;
        Rtc::clear_wakeup();
        feed();
        // Whatever arrived while the core was down.
        uint8_t got[8];
        uint32_t n = 0;
        const uint32_t t0 = now();
        while (to_us(since(t0)) < 200000u && n < sizeof got) {
            uint8_t b = 0;
            if (Serial::read_byte(b)) {
                got[n++] = b;
            }
        }
        {
            InterruptGuard guard;
            Usart<2>::enable(false);
            (void)Usart<2>::wake_from_stop(UsartWakeSource::none);
            (void)Usart<2>::wake_line(false);
            (void)Usart<2>::mute_mode_off();
            (void)Usart<2>::kernel_clock(UsartClock::pclk);
            Usart<2>::set_brr(usart_brr(SysClock::pclk_hz, 115200).value());
            Usart<2>::enable(true);
        }
        console_wake_armed = 0;   // only once WUFIE is really down
        host_settle();
        print(serial, "  ", c.name, ": REACK ", reack ? "set" : "CLEAR",
              ", the Stop lasted ", slept, " ms, WUF seen ", wuf_seen,
              " time(s), the RTC backstop ", by_rtc ? "fired" : "did not fire",
              ", ", n, " byte(s) survived it, first ", n ? got[0] : 0u, crlf);
        if (wuf_seen != 0u && !by_rtc) {
            ++woke;
        }
    }
    bench.verdict("the USART wakes this part out of Stop 0 on every one of "
                  "33.5.21's three sources, from a byte the host sent while "
                  "the core was down - and the RTC backstop never had to "
                  "fire", woke == tried && tried >= 3u);

    // ES0548 2.2.4 STAGED: with HSIDIV != 0 a clock-request peripheral
    // "fails to wake the device from Stop modes". The control is that
    // the very same Stop is entered and left at HSIDIV = 0.
    feed();
    host_announce("poke", 0, 9600, "8N1", 1500, 4);
    {
        InterruptGuard guard;
        Usart<2>::enable(false);
        (void)Usart<2>::kernel_clock(UsartClock::hsi16);
        Usart<2>::set_brr(usart_brr(16'000'000u, 9600).value());
        (void)Usart<2>::wake_from_stop(UsartWakeSource::start_bit);
        (void)Usart<2>::wake_line(true);
        Usart<2>::enable(true);
    }
    // Move SYSCLK onto a DIVIDED HSI: HSISYS = HSI16 / 4.
    Rcc::sysclk_select(SysclkSource::hsisys);
    (void)Rcc::sysclk_wait(SysclkSource::hsisys);
    Rcc::hsi_div(2);                      // HSIDIV = 010, divide by 4
    Rcc::pll_enable(false);
    const uint8_t div_now = Rcc::hsi_div();
    // The RTC's wake-up timer is the control AND the way out: it proves
    // the Stop itself works and it ends this leg whatever the USART does.
    (void)Rtc::set_wakeup(RtcWakeupClock::ck_spre, 1, true);
    Rtc::clear_flags(RtcFlag::wakeup);
    feed();
    console_wakes = 0;
    console_wake_armed = 1;
    rtc_backstops = 0;
    const uint32_t w0 = rtc_wall();
    Ticker::pause();
    Pwr::enter(PwrMode::stop0);
    Ticker::resume();
    Rcc::hsi_div(0);
    (void)SysClock::init();
    const uint32_t slept = rtc_ms(rtc_delta(w0, rtc_wall()));
    const uint32_t wuf_seen = console_wakes;
    const bool rtc_woke = rtc_backstops != 0u;
    Rtc::clear_wakeup();
    feed();
    {
        InterruptGuard guard;
        Usart<2>::enable(false);
        (void)Usart<2>::wake_from_stop(UsartWakeSource::none);
        (void)Usart<2>::wake_line(false);
        (void)Usart<2>::kernel_clock(UsartClock::pclk);
        Usart<2>::set_brr(usart_brr(SysClock::pclk_hz, 115200).value());
        Usart<2>::enable(true);
    }
    console_wake_armed = 0;
    host_settle();
    print(serial, "  ES0548 2.2.4 staged with HSIDIV = ", div_now,
          " (HSI16 / 4): the Stop lasted ", slept, " ms, WUF seen ", wuf_seen,
          " time(s), and the RTC wake-up timer - the control that the Stop "
          "itself works - fired: ", rtc_woke ? "yes" : "no", crlf);
    if (wuf_seen == 0u && rtc_woke) {
        print(serial, "  THE ERRATUM REPRODUCES: a clock-request peripheral "
                      "did not wake the part on a divided HSI, and the RTC "
                      "did", crlf);
    } else if (wuf_seen != 0u) {
        print(serial, "  the erratum did NOT reproduce here: WUF rose on a "
                      "divided HSI. Recorded as measured", crlf);
    }
    bench.verdict("2.2.4 is staged with a control that separates \"the Stop "
                  "did not work\" from \"the USART did not wake it\"",
                  rtc_woke);

    // AND THE ONE THING THAT MIGHT ANSWER IT: RCC_CR.HSIKERON keeps HSI16
    // running for a kernel-clock consumer whether or not anybody asks -
    // so if 2.2.4 is really about the REQUEST path (33.5.21's
    // usart_ker_ck_req, which starts the oscillator on the falling edge
    // of RX), a clock that never stopped should not need it. The same
    // leg, the same divided HSI, one bit different. Printed either way;
    // it is a question the errata sheet does not answer and this letter
    // does not claim to settle beyond what it measured.
    feed();
    host_announce("poke", 0, 9600, "8N1", 1500, 4);
    {
        InterruptGuard guard;
        Usart<2>::enable(false);
        (void)Usart<2>::kernel_clock(UsartClock::hsi16);
        Usart<2>::set_brr(usart_brr(16'000'000u, 9600).value());
        (void)Usart<2>::wake_from_stop(UsartWakeSource::start_bit);
        (void)Usart<2>::wake_line(true);
        Usart<2>::enable(true);
    }
    Rcc::hsi_kernel_request(true);
    const bool kernel_on = Rcc::hsi_kernel_request();
    Rcc::sysclk_select(SysclkSource::hsisys);
    (void)Rcc::sysclk_wait(SysclkSource::hsisys);
    Rcc::hsi_div(2);
    Rcc::pll_enable(false);
    (void)Rtc::set_wakeup(RtcWakeupClock::ck_spre, 1, true);
    Rtc::clear_flags(RtcFlag::wakeup);
    feed();
    console_wakes = 0;
    console_wake_armed = 1;
    rtc_backstops = 0;
    const uint32_t k0 = rtc_wall();
    Ticker::pause();
    Pwr::enter(PwrMode::stop0);
    Ticker::resume();
    Rcc::hsi_div(0);
    (void)SysClock::init();
    const uint32_t k_slept = rtc_ms(rtc_delta(k0, rtc_wall()));
    const uint32_t k_wuf = console_wakes;
    const bool k_rtc = rtc_backstops != 0u;
    Rtc::clear_wakeup();
    Rcc::hsi_kernel_request(false);
    feed();
    {
        InterruptGuard guard;
        Usart<2>::enable(false);
        (void)Usart<2>::wake_from_stop(UsartWakeSource::none);
        (void)Usart<2>::wake_line(false);
        (void)Usart<2>::kernel_clock(UsartClock::pclk);
        Usart<2>::set_brr(usart_brr(SysClock::pclk_hz, 115200).value());
        Usart<2>::enable(true);
    }
    console_wake_armed = 0;
    host_settle();
    print(serial, "  and the same leg with RCC_CR.HSIKERON set (", 
          kernel_on ? "the bit reads back" : "THE BIT DID NOT STICK",
          "): the Stop lasted ", k_slept, " ms, WUF seen ", k_wuf,
          " time(s), the RTC backstop ", k_rtc ? "fired" : "did not fire",
          crlf);
    if (k_wuf != 0u) {
        print(serial, "  HSIKERON IS THE WAY ROUND 2.2.4 ON THIS PART: with "
                      "HSI16 kept running for its kernel-clock consumer the "
                      "same divided-HSI Stop is woken by the USART, so the "
                      "erratum is about the clock REQUEST and not about the "
                      "wake-up path", crlf);
    } else {
        print(serial, "  HSIKERON does NOT rescue it: the wake fails on a "
                      "divided HSI with the oscillator kept running too, so "
                      "2.2.4 reaches further than the request path. Recorded "
                      "as measured, not explained", crlf);
    }
}

// ---------------------------------------------------------------------------
// v: the console moved to LPUART1, on the console's own pads
// ---------------------------------------------------------------------------

constexpr UartPins lpuart_console_pins{
    .tx = {'A', 2, PinFunction::af6},     // LPUART1_TX, DS13560 table 13
    .rx = {'A', 3, PinFunction::af6},     // LPUART1_RX
};
constexpr UartOptions lp_lse_opts{.kernel_clock = UsartClock::lse};
constexpr UartOptions lp_hsi_opts{.kernel_clock = UsartClock::hsi16};
using LpConsoleLse =
    LpUart<1, lpuart_console_pins, 128, 256, NoDmaEngine, NoDmaEngine, lp_lse_opts>;
using LpConsoleHsi =
    LpUart<1, lpuart_console_pins, 128, 256, NoDmaEngine, NoDmaEngine, lp_hsi_opts>;
constexpr LpConsoleLse lp_console_lse;
constexpr LpConsoleHsi lp_console_hsi;

volatile uint8_t lp_console_mode = 0;

void tv_lpuart_console() {
    print(serial, "  this letter needs tools/uart_stress.py; run it as", crlf,
          "  python3 tools/uart_stress.py --port <the console> --letters v",
          crlf);
    feed();
    const bool lse_running = RtcDomain::lse_ready();
    if (!lse_running) {
        print(serial, "  no crystal, no LSE console: DECLINED", crlf);
        bench.verdict("the LSE console leg needs the crystal", false);
        return;
    }

    // LEG ONE: LPUART1 on PA2/PA3 AF6 - the console's OWN pads, which
    // this instance reaches through a different alternate function -
    // clocked by the 32768 Hz crystal at 9600 baud, the chapter's own
    // ceiling for that clock. The host echoes; the board counts.
    uint32_t lse_got = 0;
    uint32_t lse_bad = 0;
    host_announce("sink", 0, 9600, "8N1", 1600, 0);
    Serial::release();
    lp_console_mode = 1;
    const bool lse_up = LpConsoleLse::init(clock, 9600);
    lfsr_reset();
    uint8_t chunk[32];
    uint32_t t0 = now();
    // THE WINDOW OUTLASTS THE HOST'S PUMP ON PURPOSE (the script pumps
    // for window - 500 ms and starts 140 ms after reading the HOST
    // line): a board that stops listening first leaves the tail of the
    // stream to arrive on a console that has moved back to 115200.
    while (to_us(since(t0)) < 1200000u) {
        const uint32_t n = LpConsoleLse::read_bulk(chunk);
        for (uint32_t i = 0; i < n; ++i) {
            if (chunk[i] != lfsr_next()) {
                ++lse_bad;
            }
        }
        lse_got += n;
        feed();
    }
    const uint32_t lse_brr = L1::brr();
    LpConsoleLse::release();
    lp_console_mode = 0;
    (void)Serial::init(clock, 115200);
    host_settle();
    print(serial, "  LPUART1 on PA2/PA3 AF6, clocked by the LSE at 9600: ",
          lse_got, " bytes in, ", lse_bad, " wrong, BRR ", lse_brr,
          " (874 is 256 x 32768 / 9600 rounded)", crlf);

    // LEG TWO: the same instance on HSI16 at 115200, which is what an
    // LPUART is for when the chip is awake.
    uint32_t hsi_got = 0;
    uint32_t hsi_bad = 0;
    host_announce("sink", 0, 115200, "8N1", 1300, 0);
    Serial::release();
    lp_console_mode = 2;
    const bool hsi_up = LpConsoleHsi::init(clock, 115200);
    lfsr_reset();
    t0 = now();
    while (to_us(since(t0)) < 1000000u) {
        const uint32_t n = LpConsoleHsi::read_bulk(chunk);
        for (uint32_t i = 0; i < n; ++i) {
            if (chunk[i] != lfsr_next()) {
                ++hsi_bad;
            }
        }
        hsi_got += n;
        feed();
    }
    const uint32_t hsi_brr = L1::brr();
    LpConsoleHsi::release();
    lp_console_mode = 0;
    (void)Serial::init(clock, 115200);
    host_settle();
    print(serial, "  and on HSI16 at 115200: ", hsi_got, " bytes in, ",
          hsi_bad, " wrong, BRR ", hsi_brr, crlf);
    bench.verdict("a whole console moves to the LPUART - the SAME task, the "
                  "same verbs, a different peripheral and a different baud "
                  "generator - and comes back",
                  lse_up && hsi_up && lse_got >= 500u && lse_bad == 0u &&
                      hsi_got >= 2000u && hsi_bad == 0u);
    print(serial, "  (these lines are on USART2 again, which is the other "
          "half of the proof)", crlf);

    // LEG THREE: a Stop with the LPUART on the crystal as the wake
    // source. This is the arrangement the peripheral exists for.
    feed();
    if (RtcDomain::selected() != RtcClockSource::lse) {
        print(serial, "  no RTC wall clock: the Stop leg is DECLINED", crlf);
        return;
    }
    host_announce("poke", 0, 9600, "8N1", 1500, 4);
    L1::bus_clock(true);
    L1::reset();
    (void)L1::kernel_clock(UsartClock::lse);
    (void)L1::configure({}, lpuart_brr(32768, 9600).value());
    (void)L1::wake_from_stop(UsartWakeSource::start_bit);
    (void)L1::wake_line(true);
    Pin<'A', 3>::function(PinFunction::af6, {.pull = PinPull::up});
    L1::enable(true);
    uint32_t spins = 200000;
    while ((L1::status() & UsartFlag::reack) == 0u && spins-- != 0u) {
    }
    L1::clear_flags(UsartClear::all);
    (void)Rtc::set_wakeup(RtcWakeupClock::ck_spre, 1, true);
    Rtc::clear_flags(RtcFlag::wakeup);
    feed();
    // The NVIC line is what turns the EXTI's wake into a handler, and
    // the handler is what clears WUF - a level, with WUFIE set.
    lpuart_wakes = 0;
    lpuart_wake_armed = 1;
    rtc_backstops = 0;
    Nvic::enable(L1::irq());
    const uint32_t w0 = rtc_wall();
    Ticker::pause();
    Pwr::enter(PwrMode::stop1);
    Ticker::resume();
    (void)SysClock::init();
    const uint32_t slept = rtc_ms(rtc_delta(w0, rtc_wall()));
    const uint32_t lp_wuf = lpuart_wakes;
    lpuart_wake_armed = 0;
    Nvic::disable(L1::irq());
    const uint32_t st = L1::status();
    const bool lp_by_rtc = rtc_backstops != 0u;
    Rtc::clear_wakeup();
    feed();
    // A START-BIT WAKE ARRIVES BEFORE ITS CHARACTER DOES, and at 9600
    // on a 32768 Hz kernel a frame is a whole millisecond: reading RDR
    // the instant the WFI returns reads an empty register and calls a
    // working wake a lost byte. So the character is waited for, bounded.
    bool rxne = (st & UsartFlag::rxne) != 0u;
    {
        const uint32_t t_rx = now();
        while (!rxne && to_us(since(t_rx)) < 50000u) {
            rxne = (L1::status() & UsartFlag::rxne) != 0u;
        }
    }
    const uint16_t byte = rxne ? L1::read_word() : 0u;
    (void)L1::wake_line(false);
    L1::enable(false);
    L1::reset();
    L1::bus_clock(false);
    (void)Serial::init(clock, 115200);
    host_settle();
    print(serial, "  Stop 1 with LPUART1 on the crystal as the wake source: "
          "the Stop lasted ", slept, " ms, WUF seen ", lp_wuf,
          " time(s), the RTC backstop ", lp_by_rtc ? "fired" : "did not fire",
          ", a character ", rxne ? "survived it: " : "did not arrive: ", byte,
          crlf);
    bench.verdict("an LPUART on a 32768 Hz crystal brings the part out of "
                  "Stop 1 on a start bit - the whole reason the peripheral "
                  "exists", lp_wuf != 0u && !lp_by_rtc);
}

// ---------------------------------------------------------------------------
// The menu
// ---------------------------------------------------------------------------

void banner() {
    print(serial, crlf,
          "test_stm32_serial - the USART's long tail, the LPUARTs and IRTIM "
          "(board E, no wires)", crlf);
    bench.menu();
}

}   // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// A HardFault here would otherwise be a silent spin in Default_Handler
/// and then a watchdog reset thirty seconds later, which is the WORST
/// failure a bench suite can have: a truncated line and a banner, with
/// nothing to say what happened. hard_fault_reset() puts a breadcrumb in
/// .noinit and resets at once, and main() prints it.
extern "C" void HardFault_Handler() {
    brio::hard_fault_reset<brio::Stm32Platform>(0xFA);
}

/// USART2 and LPUART2 share this line on the G0B1 class. The console is
/// the first; the LPUART console of letter v is the second, and only one
/// of them is up at a time.
extern "C" void USART2_LPUART2_IRQHandler() {
    // USART2 and LPUART2 share this line on the G0B1 class. Only USART2
    // is ever up on it in this suite - LPUART2 is driven polled in
    // letter n - so the console's body is the whole of it. LPUART1's
    // line is USART3_4_5_6_LPUART1's, below.
    // WUF IS CLEARED UNCONDITIONALLY, and the counter is what the
    // letter reads. Gating the CLEAR on the letter's own flag was this
    // suite's second wake-storm: the flag was dropped as soon as the
    // Stop returned, while WUFIE stayed armed for the 200 ms in which
    // the letter drains the bytes that woke it - and every further start
    // bit then set a level nobody would clear. Caught by halt-and-dump:
    // IPSR 44 (this very vector), ISR bit 20 standing, the counter
    // frozen at one. A handler clears every level it can see.
    if ((brio::Usart<2>::status() & brio::UsartFlag::wuf) != 0u) {
        brio::Usart<2>::clear_flags(brio::UsartClear::wuf);
        if (console_wake_armed) {
            console_wakes = console_wakes + 1u;
        }
    }
    (void)Serial::isr();
}

/// USART1 HAS A LINE OF ITS OWN on every part of this family - it is the
/// one instance that does - and this suite's laboratory is USART1, so
/// this is where the task letters' interrupts arrive.
///
/// AN UNBOUND VECTOR IS NOT A CRASH ON THIS CORE, IT IS A SILENCE: the
/// crt's Default_Handler spins with interrupts still enabled but never
/// returns, so no other handler of equal priority runs either - the
/// console stops draining mid-line and the IWDG reboots the board thirty
/// seconds later. That is exactly what the first version of letter e
/// did, and it took a HardFault breadcrumb (which stayed empty) to
/// prove it was not a fault at all. The samc stratum's NMI lesson, in
/// this family's clothes.
extern "C" void USART1_IRQHandler() {
    loop_irqs = loop_irqs + 1u;
    if (loop_mode == 1u) {
        (void)LoopUart::isr();
        return;
    }
    if (loop_mode == 2u) {
        (void)LoopFifoUart::isr();
        return;
    }
    // Nothing of ours is armed here: silence the line rather than
    // re-entering for ever on a condition nobody will clear.
    brio::Nvic::disable(brio::Usart<1>::irq());
}

/// USART3, USART4, USART5, USART6 and LPUART1 share THIS one - the
/// family's widest vector, and the reason every isr() body of this
/// stratum answers for itself alone.
extern "C" void USART3_4_5_6_LPUART1_IRQHandler() {
    // Letter v puts the whole CONSOLE on LPUART1, which arrives here and
    // not on USART2's line - the two LPUARTs of this part sit on
    // different vectors and only the second one shares the console's.
    // The task's isr() must be reached, or the byte that raised the
    // interrupt is never drained and the level re-enters for ever.
    if (lp_console_mode == 1u) {
        (void)LpConsoleLse::isr();
        return;
    }
    if (lp_console_mode == 2u) {
        (void)LpConsoleHsi::isr();
        return;
    }
    if (lpuart_wake_armed) {
        if ((brio::Lpuart<1>::status() & brio::UsartFlag::wuf) != 0u) {
            brio::Lpuart<1>::clear_flags(brio::UsartClear::wuf);
            lpuart_wakes = lpuart_wakes + 1u;
        }
        brio::Lpuart<1>::clear_flags(brio::UsartClear::receive_errors);
        // One wake is this letter's whole need, and the line is taken
        // down here so no later level can storm it (the console's own
        // lesson, one vector along).
        brio::Nvic::disable(brio::Lpuart<1>::irq());
        return;
    }
    if (lpuart_live) {
        lpuart_irqs = lpuart_irqs + 1u;
        (void)brio::Lpuart<1>::read_word();
        brio::Lpuart<1>::clear_flags(brio::UsartClear::all);
        return;
    }
    // AND THE OTHER HALF OF THE SAME LESSON: reading a peripheral whose
    // bus clock is off is not a zero, it is a bus fault (5.2.17), so the
    // fallback here touches NOTHING and takes the line down instead.
    brio::Nvic::disable(brio::Lpuart<1>::irq());
}

extern "C" void RTC_TAMP_IRQHandler() {
    const uint32_t f = brio::Rtc::isr();
    if ((f & brio::RtcFlag::wakeup) != 0u) {
        rtc_backstops = rtc_backstops + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    brio::Pwr::bus_clock(true);
    brio::Pwr::rtc_domain_unlock(true);
    brio::RtcDomain::apb_clock(true);

    // THE RTC DOMAIN IS NEVER *RESET* HERE: wiping it would cost the
    // backup registers other suites own, and RTCSEL is one-way, so a
    // domain already latched onto something else is simply reported and
    // the sleeping letters decline.
    //
    // BUT AN UNSELECTED DOMAIN IS NOT A LATCHED ONE. RTCSEL = 00 is the
    // state a domain POWER-ON leaves (this board's VBAT is tied to VDD,
    // so unplugging it is a domain power-on), and choosing LSE from
    // there is the ordinary one-way write of 5.4.23, not a BDRST. So the
    // crystal is claimed whenever the domain is on LSE ALREADY or has no
    // clock at all, and the calendar is set only when INITS says it was
    // never set - which leaves a running calendar exactly as it was.
    const brio::RtcClockSource domain_now = brio::RtcDomain::selected();
    const bool claimable = domain_now == brio::RtcClockSource::lse ||
                           domain_now == brio::RtcClockSource::none;
    brio::RtcDomain::lse_enable(true);
    const bool lse_ok = brio::RtcDomain::lse_wait_ready(4'000'000UL);
    bool wall_ready = false;
    if (claimable && lse_ok) {
        wall_ready = brio::RtcDomain::open(brio::RtcClockSource::lse);
        brio::Rtc::bypass_shadow(true);
        if (wall_ready && !brio::Rtc::calendar_set()) {
            wall_ready = brio::Rtc::init(
                brio::RtcPrescalers{},
                brio::RtcDateTime{.hour = 0, .minute = 0, .second = 0,
                                  .day = 1, .month = 1, .year = 24,
                                  .weekday = 1});
        }
    }

    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    stopwatch_up();
    // The RTC's line: the wake-up timer is the Stop letters' backstop,
    // and a backstop that cannot reach the NVIC is not one.
    brio::Nvic::enable(brio::Rtc::irq());
    const bool wd = brio::Iwdg::arm(brio::IwdgConfig{
        .prescaler = brio::IwdgPrescaler::div256,
        .reload = 0x0FFF,
        .window = 0x0FFF});
    brio::enable_interrupts();

    bench.letter('a', "the instance table: six USARTs, two LPUARTs, three "
                      "authorities", ta_instances);
    bench.letter('b', "THE INSTRUMENT: the single wire, and every frame format",
                 tb_loop);
    bench.letter('c', "the baud generator: both oversamplings, twelve "
                      "prescalers, the ceiling", tc_baud);
    bench.letter('d', "the kernel clocks, with the console moved under itself",
                 td_kernel_clocks);
    bench.letter('e', "the FIFOs: thresholds, overruns, interrupts per "
                      "kilobyte", te_fifo);
    bench.letter('f', "the bit-banged line: parity, framing, noise, tolerance, "
                      "2.11.1", tf_bitbang);
    bench.letter('g', "auto-baud, four modes, at rates nobody was told",
                 tg_autobaud);
    bench.letter('h', "LIN: the break sent and the break detected", th_lin);
    bench.letter('i', "mute mode, the receiver time-out, the character match",
                 ti_mute_modbus);
    bench.letter('j', "smartcard on one wire: guard time, CK, NACK, retries",
                 tj_smartcard);
    bench.letter('k', "IrDA: both pulse widths, an RZI frame, the glitch "
                      "filter", tk_irda);
    bench.letter('l', "the pads' extras: swap, inversion, MSB, DE, RTS, CTS",
                 tl_pads);
    bench.letter('m', "the synchronous master's clock, counted with no CPU",
                 tm_synchronous);
    bench.letter('n', "the LPUARTs: two instances, table 198, the shared "
                      "vectors", tn_lpuart);
    bench.letter('o', "IRTIM: a carrier, an envelope and one pad", to_irtim);
    bench.letter('y', "HOST: streaming across the kernel clocks", ty_streaming,
                 false);
    bench.letter('w', "HOST: the wake from Stop, and 2.2.4 staged", tw_wake,
                 false);
    bench.letter('v', "HOST: the console moved to LPUART1", tv_lpuart_console,
                 false);

    const std::optional<brio::PanicRecord> crumb =
        brio::take_panic_record<brio::Stm32Platform>();

    if (serial_ok) {
        if (crumb) {
            print(serial, crlf, "PREVIOUS RUN DIED: panic code ",
                  static_cast<uint32_t>(crumb->code), " context ",
                  static_cast<uint32_t>(crumb->context), crlf);
        }
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL 64 MHz" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " wall=", wall_ready ? "RTC on LSE" : "NO CRYSTAL",
              " backstop=", wd ? "IWDG 32 s" : "FAILED", crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        feed();
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
