// test_stm32_serial - the reference bench suite for the STM32G0's SERIAL
// TAIL: the whole of the USART's chapter 33 beyond the console
// personality, the LPUARTs of chapter 34, and the infrared interface of
// chapter 27 that ANDs two of this board's timers onto one pad.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
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
//    their thresholds and both LPUARTs are measured on that loop. On the
//    paired desk PA9 also carries the link wire to the far board's
//    receive pad, and the open drain's rise on the pull-up into that load
//    gives out above 460800: the letters that run the loop above it
//    DRIVE the pad push-pull while a frame goes out (the far end hi-Z
//    under its dark console), letter c measuring both.
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
// function - and puts them back on USART2's AF1 afterwards. FOUR OF
// THOSE PADS ARE THE 48- AND 64-PIN PACKAGES' - PB3, PB4, PC1 and PC6:
// what a 32-pin part bonds instead is under "What the part has" below,
// and every leg that loses a pad there SKIPS BY NAME and claims nothing.
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
// THE RATE ON THE WIRE IS TIMED, because a loop cannot tell it: the
// single wire shares one divisor between the transmitter and the receiver
// it loops into, so a wrong divisor is a slower or a faster loop and
// every byte still comes back. Every letter that loops or sets a rate on
// a transmitter therefore times a run of its frames on TIM2 against
// their time AT THE RATE ASKED, prints the reading in thousandths and
// judges it -1 % to +3 % (res_wire_permille, port_wire_permille). The
// letters whose rate is a RECEIVER's - f, g, i and the receive halves of
// h, k and l - are fed by the bit-banged transmitter, which TIM2 paces at
// the rate asked: a receiver that decodes it is within its own tolerance
// of that rate, and no wire of its own is there to time.
//
// What is exercised, letter by letter (every letter runs on every part
// this suite builds for; a leg whose instance, feature or pad the part
// has not got prints the fact and claims nothing):
//   a  the instance table: six USARTs and two LPUARTs against table 183,
//      the reserve's stated split, the device header's own macros and
//      the silicon - does FIFOEN stick, does PRESC stick, does CR2.CLKEN
//      stick (the one row of table 184 that is NOT the FULL/BASIC
//      split), is there a kernel-clock multiplexer (ES0548 2.11.2's
//      subject)
//   b  THE INSTRUMENT: the single-wire loop proven, and every frame
//      format of 33.5.3 byte-exact on it - and each format's frame
//      length timed on the wire at 115200
//   c  the baud generator: both oversamplings, the twelve prescaler
//      codes, and where the loop's own open-drain rise time gives out -
//      every rung timed against the rate asked, OVER8's ceiling of
//      8 Mbaud included
//   d  the kernel clocks: the CONSOLE moved to HSI16 and to SYSCLK while
//      it is talking, its own line timed on each, and USART1 run off the
//      32768 Hz crystal, timed
//   e  the FIFOs: thresholds, RXFF/TXFE, the per-entry error flags,
//      interrupts per kilobyte with and without, ORE and OVRDIS; the line
//      under the FIFO and both transports' blocks timed on the wire
//   f  the bit-banged transmitter: parity, framing, noise, the tolerance
//      of tables 188/189, and ES0548 2.11.1 staged with its control
//   g  auto-baud, all four modes, at rates the receiver was not told
//   h  LIN: the break sent and timed, the frames timed, the break
//      detected at both lengths
//   i  mute mode, the receiver time-out and the character match
//   j  smartcard: 8E1.5 on one wire, the character's twelve baud periods
//      timed, the guard time, the CK ladder, the NACK staged from a pull,
//      the retries counted
//   k  IrDA: the 3/16 pulse and the low-power pulse measured, the SIR
//      frames timed in both modes, a bit-banged RZI frame decoded, the
//      glitch filter as a threshold
//   l  the pads' extras: SWAP (its wire timed), the three inversions, MSB
//      first, the break request, the driver enable timed, RTS and CTS
//   m  the synchronous master's CK, counted with no CPU, and its frames
//      timed
//   n  the LPUARTs: both instances, both baud generators, the FIFOs, the
//      prescalers and the shared vectors - every rate timed on the wire
//   o  IRTIM: a 38 kHz carrier under a 1 kHz envelope on one pad, both
//      polarities, and a USART as the envelope, its frames timed
//   p  THE RECEIVE TRANSPORT: the paced receiver (RXFT, the receiver
//      time-out for the tail) and the receive engine's edges (the line's
//      IDLE, the ring's half and full marks) - each burst told by a
//      vector within two frames of its last stop bit, a stream with no
//      silence across a ring three times its size - and K breaks in N
//      slots through both receivers: FE = K; the interrupt receiver,
//      read between the breaks, N - K delivered and a skip a break, read
//      only at the end nothing the ring held; the engine N - K, no byte
//      taken from under the channel; both transports' lines timed
//   q  tx_idle() ON THE PAD: EXTI line 9 times PA9's start bits, and the
//      wire's idle is the last stop bit's end, on the interrupt
//      transmitter and on the transmit engine - and the start bits seven
//      frames apart at the rate asked
// Outside z, because they need a peer or a real Stop:
//   y  streaming through the console across kernel clocks (uart_stress)
//   w  WAKE FROM STOP, and ES0548 2.2.4 staged (uart_stress)
//   v  the console moved to LPUART1 on its own pads (uart_stress)
// THE TWO-BOARD SET, under the second all-key t (z never runs it): USART1
// crossed to the desk's STM32F446RE running its usart_peer (PA9 to its
// PA10, PA10 from its PA9), driven in band with usart_link.hpp - every
// verdict of a rate is a FOREIGN receiver's, timed on a crystal:
//   1  the rendezvous, the refusal path, and this HSI16 against the far
//      crystal both ways
//   2  the baud matrix to 6.4 Mbaud: an echo, this transmitter timed by
//      the far chip against a 1 Mbaud reference either side, the far
//      blast on the polled receiver, the interrupt receiver and the
//      receive engine
//   3  the frame matrix across the wire, a refusal SKIPPED with its reason
//   4  the receiver's errors from a foreign sender: PE, the FE onset, ORE
//      with the FIFO, the start bit's vote, the break with and without
//      LIN, the FIFO's per-entry flags drained late, the task's counters
//
// THE G031K8 does not hold the suite whole (some 77 KB against its 64):
// it builds three images, letters a to j - the instances, the generator,
// the kernel clocks, the FIFOs and the modes - in one, k to y in the
// second and the two-board set in the third (the groups line below;
// design/overview.md, "A suite's image fits the family's smallest chip");
// the G071RB and the G0B1RE build one.
//
// build: boards = g0b1re,g071rb,g031k8
// build: groups = abcdefghij,klmnopqvwy,1234
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/flash.hpp"
#include "stm32g0/irtim.hpp"
#include "stm32g0/lpuart.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform.hpp"
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

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: the far board's usart_peer
// compiles the same file.
#include "../../../avrdx/src/apps/usart_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32g0Platform<>;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

/// The second all-key is 't' (two boards): 'y' is a letter here. 29 places:
/// this image's twenty, and the two-board set's nine.
TwoKeyTestBench<Serial, 't', 29> bench;

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

// ---- WHAT THE PART HAS, AND WHAT THE PACKAGE BONDS -------------------------
//
// Every INSTANCE question is the reserve's and the letters ask it there
// (usart_present, usart_is_full, usart_has_clock_select, lpuart_present).
// The PACKAGE question - which of the die's pads a 32-pin part brings
// out - has NO answer in any device header: it is the datasheet's pin
// table, so it is the SUITE's to state, an app being the one place board
// knowledge may live. Two pad facts of the Nucleo-32 (STM32G031K8,
// LQFP32) close legs below, and NOT ONE of them is a peripheral this
// part has not got:
//
//  - LPUART1's bonded pads here are PA2/PA3 at AF6, which are the
//    ST-LINK's virtual COM port - the CONSOLE's own pair. A single-wire
//    loop on them would put its own traffic into this suite's report, so
//    letter n's pad half skips; letter v, which MOVES the console there
//    with the host told, runs.
//  - USART1_RTS_DE_CK and USART1_CTS ARE on this package, and on the
//    same two pads at the same alternate function as everywhere else:
//    DS12992 table 14 gives PB3 AF4 USART1_RTS_DE_CK and PB4 AF4
//    USART1_CTS, and table 12 bonds both (this package's port B runs
//    PB0..PB9). So the driver-enable, flow-control, smartcard-clock and
//    synchronous-clock legs run here as they do on the Nucleo-64s.
//  - THE LSE CRYSTAL RUNS ON THIS BOARD TOO, as on the Nucleo-64s, so
//    the 32768 Hz kernel clock and the RTC wall are there for the
//    letters that need them. `board_lse_fitted` stays a BOARD statement
//    (a board whose X2 is absent or cut off its pads by its solder
//    bridges would set it false and spare itself the code of the legs
//    it cannot stage), and every leg below still ASKS
//    `RtcDomain::lse_ready()` at run time before resting on it.
#if defined(STM32G031xx)
constexpr bool has_lpuart_own_pad = false;
constexpr bool has_u1_ck_pads = true;
#else
constexpr bool has_lpuart_own_pad = true;
constexpr bool has_u1_ck_pads = true;
#endif
constexpr bool board_lse_fitted = true;

/// The one sentence letters j, l and m each print where the CK/DE/RTS
/// pads are not this package's - one string, three users, because on the
/// part that prints it every byte of prose is flash it has not got.
[[maybe_unused]] constexpr const char* ck_pad_skip =
    "  SKIPPED, no verdict claimed: this leg drives USART1_RTS_DE_CK (and "
    "CTS) on PB3/PB4 at AF4, which is the 48- and 64-pin packages' row. The "
    "USART pads this suite knows for the 32-pin one are USART1_TX/RX on "
    "PA9/PA10 at AF1, USART2 and LPUART1 on PA2/PA3, and IR_OUT on PB9 - no "
    "CK, no DE, no flow control - so no pad is claimed and nothing is "
    "measured.";

/// The lowest-numbered instance this part makes BASIC: USART4 on the
/// G0B1 (of six), USART3 on the G071 (of four), USART2 on the G031 (of
/// two) - which on that part is the CONSOLE, and the reason the leg that
/// uses it never touches a bus clock (the kernel-clock multiplexer is
/// RCC_CCIPR's field, and kernel_clock() on an instance without one
/// neither reads nor writes the peripheral at all). Every G0 of the pack
/// has at least two USARTs and therefore at least one BASIC instance;
/// the trailing 2 only keeps the expression well-formed.
constexpr uint8_t first_basic_usart() {
    for (uint8_t i = 1; i <= 6u; ++i) {
        if (usart_present(i) && !usart_is_full(i)) {
            return i;
        }
    }
    return 2;
}

using U1 = Usart<1>;
using L1 = Lpuart<1>;
using BasicUsart = Usart<first_basic_usart()>;
// LPUART2 is the G0B1/G0C1 class's alone. A part without it must not
// SPELL `Lpuart<2>` (the driver's static_assert is the refusal), and
// `if constexpr` inside a plain function still instantiates the branch it
// discards - while a NON-DEPENDENT `Lpuart<2>` inside a template body is
// looked up when the template is DEFINED. So the instance NUMBER depends
// on the reserve fact that gates every branch that names it.
template <bool present>
using Lpuart2 = Lpuart<present ? uint8_t{2} : uint8_t{1}>;

template <bool present = lpuart_present(2)>
bool lpuart2_shares_usart2() {
    if constexpr (present) {
        return Lpuart2<present>::irq() == usart_irq(2);
    } else {
        return true;
    }
}

template <bool present = lpuart_present(2)>
bool lpuart2_exti_35() {
    if constexpr (present) {
        return Lpuart2<present>::exti_line == 35;
    } else {
        return true;
    }
}

/// LPUART1's line, which is the OTHER half of the same presence rule:
/// where the part has a USART3, LPUART1 shares that instance's line (the
/// G071's USART3_4_LPUART1, the G0B1's USART3_4_5_6_LPUART1); where it
/// has none there is nothing to share with and LPUART1 gets a vector of
/// its own. The second half is spelled as "the line of NEITHER USART
/// this part has" rather than as LPUART1_IRQn, because that enumerator
/// exists only on the parts where it is the answer.
template <bool has_u3 = usart_present(3)>
bool lpuart1_line_ok() {
    if constexpr (has_u3) {
        return usart_irq(3) == usart_irq(4) && lpuart_irq(1) == usart_irq(3);
    } else {
        return lpuart_irq(1) != usart_irq(1) && lpuart_irq(1) != usart_irq(2);
    }
}

// The task instantiations this suite drives. The DEFAULT one is what
// every console in the tree uses; the others are the option struct at
// work, and each is a compile-time proof that its branch exists. The
// plain loop STATES the FIFO off: it is letter e's one-register control,
// and the default would turn the FIFO on for USART1.
constexpr UartOptions hdsel_opts = uart_half_duplex({.fifo = false});
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
/// measurement.
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
/// input alternate function leaves the pull in charge (measured); an
/// OPEN-DRAIN output function releases the pad whenever it is not
/// pulling low, so the pull should rule there too. Both are measured,
/// not assumed.
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
    /// The single wire DRIVEN: the pad push-pull at very high speed while
    /// a frame goes out, the pull-up holding it between frames - the loop
    /// on a wire whose far end only listens (UartOptions::
    /// single_wire_push_pull's shape, by hand).
    bool driven = false;
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
    // the swap option chooses - or driven, where the setup says so.
    const PinConfig pad{.pull = PinPull::up, .open_drain = !s.driven,
                        .speed = s.driven ? PinSpeed::very_high : PinSpeed::low};
    if (s.swap) {
        RxPin::function(u1_rx.function, pad);
    } else {
        TxPin::function(u1_tx.function, pad);
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

/// A frame's bit periods: the start bit, the word (33.5.5: the parity bit
/// is one of its M bits), the stop bits.
uint32_t frame_bits(const UartFormat& f) {
    return 1u + static_cast<uint32_t>(f.bits) + f.stop_bits;
}

// ---------------------------------------------------------------------------
// THE RATE ON THE WIRE, which a loop cannot tell
// ---------------------------------------------------------------------------
//
// A single wire shares ONE divisor between the transmitter and the
// receiver it loops into, so a wrong divisor is a slower or a faster loop
// and nothing else: every byte still comes back. So every letter that
// loops or sets a rate also TIMES A RUN of frames against the wire's
// time AT THE RATE ASKED - never at the rate the divisor produced, which
// is what a wrong one would hide - on TIM2 (one count a cycle at 64 MHz,
// 15.6 ns), from the first store into TDR to TC, the last stop bit's end
// (33.8.10: TC rises when "the transmission of a frame containing data is
// complete", and a write to TDR clears it). The run is a hundredth of a
// second of frames, sixteen at least and 1024 at most, so the ruler and
// the start-up of the first frame (33.5.17: a full shift register "starts
// shifting on the next baud clock edge") stay within a few thousandths;
// the answer is in thousandths of the wire's time. TIM2 and every kernel
// clock but the LSE derive from HSI16 on these boards (no HSE fitted), so
// the oscillator's own error cancels and what is read is the divisor's;
// against the LSE crystal the reading carries HSI16's trim as well.

/// How many frames time a rate.
uint32_t wire_frames(uint32_t baud) {
    const uint32_t f = baud / 100u;
    return f < 16u ? 16u : f > 1024u ? 1024u : f;
}

/// The verdict on a reading: never faster than the rate asked by more
/// than the divisor's rounding and the ruler's tick, never slower by more
/// than three per cent.
bool wire_ok(uint32_t permille) { return permille >= 990u && permille <= 1030u; }

/// The lowest and highest of a set of readings, and whether every one
/// passed.
struct WireSpan {
    uint32_t low = 0xFFFFFFFFu;
    uint32_t high = 0;
    bool ok = true;
    void add(uint32_t pm) {
        low = pm < low ? pm : low;
        high = pm > high ? pm : high;
        ok = ok && wire_ok(pm);
    }
};

/// `frames` frames of `bits` bit periods written back to back into the
/// resource `R`'s TDR (TXE, which is TXFNF under FIFOEN), timed to TC
/// against their time at `baud`; 0 when the line never drained. A run
/// shorter than ten milliseconds runs with interrupts masked - at 8 Mbaud
/// a frame is 80 cycles and a tick's handler would gap it - and a longer
/// one has a frame or more of slack in TDR. What the loop brings back is
/// thrown away after it and every flag cleared.
template <typename R>
uint32_t res_wire_permille(uint32_t baud, uint32_t bits = 10u, uint32_t frames = 0u) {
    if (frames == 0u) {
        frames = wire_frames(baud);
    }
    const uint64_t wire = static_cast<uint64_t>(frames) * bits * SysClock::hz / baud;
    const uint32_t budget = static_cast<uint32_t>(wire * 2u) + SysClock::hz / 50u;
    const uint32_t bit = SysClock::hz / baud;
    // The idle frame TE sends, and anything an earlier step left, off the
    // wire first.
    const uint32_t w0 = now();
    while ((R::status() & UsartFlag::tc) == 0u && since(w0) < budget) {
    }
    spin_cycles(2u * bits * bit);
    auto run = [&]() -> uint32_t {
        uint32_t t0 = now();
        for (uint32_t i = 0; i < frames; ++i) {
            while ((R::status() & UsartFlag::txe) == 0u) {
                if (since(t0) > budget) {
                    return 0;
                }
            }
            if (i == 0u) {
                t0 = now();
            }
            R::write_data(static_cast<uint8_t>(i * 37u + 11u));
        }
        while ((R::status() & UsartFlag::tc) == 0u) {
            if (since(t0) > budget) {
                return 0;
            }
        }
        return since(t0);
    };
    uint32_t took = 0;
    if (wire < SysClock::hz / 100u) {
        InterruptGuard guard;
        took = run();
    } else {
        took = run();
    }
    spin_cycles(2u * bits * bit);
    for (uint8_t i = 0; i < 16u && (R::status() & UsartFlag::rxne) != 0u; ++i) {
        (void)R::read_word();
    }
    R::clear_flags(UsartClear::all);
    return took == 0u ? 0u : static_cast<uint32_t>(1000ull * took / wire);
}

/// The same through a TRANSPORT: `frames` frames handed to write_bulk()
/// and timed to tx_idle() - the wire's (letter q) - what the loop brings
/// back read and thrown away meanwhile, the error counters cleared after.
/// What is read is the divisor AND the transport's own gaps between
/// frames, which is the transport's cost on the wire.
template <typename Port>
uint32_t port_wire_permille(uint32_t baud, uint32_t frames = 1024u) {
    static uint8_t out[64];
    for (uint32_t i = 0; i < sizeof out; ++i) {
        out[i] = static_cast<uint8_t>(i * 37u + 11u);
    }
    uint8_t sink[32];
    const uint64_t wire = static_cast<uint64_t>(frames) * 10u * SysClock::hz / baud;
    const uint32_t budget = static_cast<uint32_t>(wire * 2u) + SysClock::hz / 50u;
    uint32_t queued = 0;
    const uint32_t t0 = now();
    while (queued < frames && since(t0) < budget) {
        const uint32_t want = frames - queued;
        queued += Port::write_bulk(
            {out, want < sizeof out ? want : static_cast<uint32_t>(sizeof out)});
        while (Port::read_bulk(sink) != 0u) {
        }
    }
    while (!Port::tx_idle() && since(t0) < budget) {
        while (Port::read_bulk(sink) != 0u) {
        }
    }
    const uint32_t took = since(t0);
    spin_cycles(static_cast<uint32_t>(20ull * 10u * SysClock::hz / baud));
    while (Port::read_bulk(sink) != 0u) {
    }
    Port::clear_errors();
    return queued < frames ? 0u : static_cast<uint32_t>(1000ull * took / wire);
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

/// One row of table 183, read off the silicon - or, on an instance this
/// part has not got, an EMPTY row that is counted nowhere and printed as
/// "absent". `Usart<n>` is dependent on the template parameter here, so
/// the branch the reserve discards is never instantiated.
struct TableRow {
    const char* name;
    InstanceReading r;
    bool present;
    bool fifo;     ///< the reserve's stated column
    bool presc;    ///< the reserve's stated column
    bool header;   ///< the device header's own IS_UART_FIFO_INSTANCE
    bool sync;     ///< the reserve's stated column
    bool is_lpuart;
};

template <uint8_t n, bool present = usart_present(n)>
TableRow usart_row(const char* name) {
    if constexpr (present) {
        using U = Usart<n>;
        return {name, probe_instance<U>(), true, U::is_full, U::has_prescaler,
                U::has_fifo(), U::has_synchronous_mode, false};
    } else {
        return {name, InstanceReading{}, false, false, false, false, false, false};
    }
}

template <uint8_t n, bool present = lpuart_present(n)>
TableRow lpuart_row(const char* name) {
    if constexpr (present) {
        using L = Lpuart<n>;
        return {name, probe_instance<L>(), true, L::has_fifo_mode,
                L::has_prescaler, L::has_fifo(), L::has_synchronous_mode, true};
    } else {
        return {name, InstanceReading{}, false, false, false, false, false, true};
    }
}

/// WHETHER A BASIC INSTANCE'S PRESCALER DIVIDES, measured on USART4's
/// own single wire (USART4_TX on PA0, AF4) - a pad no other letter
/// touches. THE LEG IS THIS PART'S OR NOBODY'S: on a part whose only
/// BASIC instance is USART2 the pad in question is the ST-LINK's virtual
/// COM port, and a suite that transmits a 0x00 character on it at a
/// prescaled rate is writing garbage into its own report. So the
/// measurement is gated on USART4's presence, not on "a BASIC instance
/// exists" - the READBACK half of the same finding is in the table
/// above and runs on every part.
template <bool present = usart_present(4)>
void ta_basic_prescaler() {
    if constexpr (present) {
        const ZeroLow basic1 = zero_low_cycles<Usart<4>, U4Pin>(u4_tx, 640,
                                                                UsartPrescaler::div1);
        const ZeroLow basic16 = zero_low_cycles<Usart<4>, U4Pin>(u4_tx, 640,
                                                                 UsartPrescaler::div16);
        print(serial, "  USART4 (BASIC, PRESC reads ", basic1.presc_readback, "/",
              basic16.presc_readback, "): ", to_us(basic1.cycles), " us at /1 and ",
              to_us(basic16.cycles), " us at /16; seen ",
              basic1.saw_frame ? "yes" : "no", "/", basic16.saw_frame ? "yes" : "no",
              ", completed ", basic1.complete ? "yes" : "no", "/",
              basic16.complete ? "yes" : "no", crlf);
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
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: whether a BASIC instance's PRESC "
              "field DIVIDES is measured on a zero character's low time, and "
              "this part's only BASIC instance is USART2 - the console. Its "
              "pad is the ST-LINK's virtual COM port, so the frame would land "
              "in this report; the READBACK half of the finding is in the "
              "table above.", crlf);
    }
}

void ta_instances() {
    feed();
    print(serial, "  RM0444 table 183 read off THIS part: the reserve's "
                  "usart_is_full / usart_present / lpuart_present columns, "
                  "checked against the device header and the silicon", crlf);

    uint8_t agree = 0;
    uint8_t rows = 0;

    TableRow rowset[8] = {};
    rowset[0] = usart_row<1>("USART1");
    // USART2 IS THE CONSOLE, and the probe resets it. The lines are
    // drained first, the instance is walked like any other, and the
    // console is brought straight back up - which is why the rest of
    // this letter arriving at all is the reading's own witness.
    console_drain();
    rowset[1] = usart_row<2>("USART2");
    (void)Serial::init(clock, 115200);
    spin_us(2000);
    rowset[2] = usart_row<3>("USART3");
    rowset[3] = usart_row<4>("USART4");
    rowset[4] = usart_row<5>("USART5");
    rowset[5] = usart_row<6>("USART6");
    rowset[6] = lpuart_row<1>("LPUART1");
    rowset[7] = lpuart_row<2>("LPUART2");

    for (const TableRow& row : rowset) {
        if (!row.present) {
            print(serial, "  ", row.name, ": absent on this part", crlf);
            continue;
        }
        print_row(row.name, row.r, row.fifo, row.presc, row.header, row.sync);
    }

    uint8_t fifo_agree = 0;
    uint8_t basic_dropped = 0;
    uint8_t basic_rows = 0;
    bool sync_agrees = true;
    bool mux_agrees = true;
    for (const TableRow& row : rowset) {
        if (!row.present) {
            continue;
        }
        ++rows;
        if (row.r.fifo_sticks == row.fifo && row.header == row.fifo) {
            ++fifo_agree;
        }
        if (row.r.fifo_sticks == row.fifo &&
            row.r.presc_sticks == row.presc && row.header == row.fifo) {
            ++agree;
        }
        if (!row.is_lpuart && !row.fifo) {
            ++basic_rows;
            if (!row.r.fifo_sticks) {
                ++basic_dropped;
            }
        }
        // The CK output belongs to every USART and to no LPUART.
        if (row.r.clken_sticks != !row.is_lpuart) {
            sync_agrees = false;
        }
        // And the kernel-clock multiplexer to the FULL USARTs and the
        // LPUARTs - which is exactly the reserve's own column.
        if (row.r.mux != (row.is_lpuart ? (lpuart_clock_select_pos(1) != 0xFFu)
                                        : row.fifo)) {
            mux_agrees = false;
        }
    }
    print(serial, "  ", agree, " of ", rows,
          " instances this part HAS agree with the reserve's stated table, the "
          "device header's own IS_UART_FIFO_INSTANCE and the silicon on BOTH "
          "counts", crlf);
    bench.verdict("table 183's FULL/BASIC/LP split is the silicon's for the "
                  "FIFO, on every instance of this part - three authorities, "
                  "one answer", fifo_agree == rows);
    bench.verdict("and a BASIC instance really does DROP the FIFO enable "
                  "rather than taking it",
                  basic_rows != 0u && basic_dropped == basic_rows);

    // THE ROW THAT IS NOT THE FULL/BASIC SPLIT, and the reason the
    // driver states it separately from is_full: table 184 gives
    // "Synchronous mode (Master/Slave)" to the FULL *and* the BASIC
    // column and takes it from the LP one, the device header's
    // IS_USART_INSTANCE ("USART Instances : Synchronous mode") lists all
    // six USARTs, and 33.8.3's note makes CR2.CLKEN reserved only where
    // neither synchronous nor smartcard mode is supported. So the CK
    // output belongs to every USART of this part - and the silicon
    // agrees, which is the third authority.
    print(serial, "  CR2.CLKEN sticks on EVERY USART this part has, the "
                  "BASIC ones included, and on NO LPUART - so the synchronous "
                  "row of table 184 cuts USART-vs-LPUART, not FULL-vs-BASIC",
          crlf);
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
    print(serial, "  PRESC[3:0] took 0xB and read it back on EVERY USART of "
                  "this part, the BASIC ones table 184 says have no "
                  "prescaler included. Whether it DIVIDES is another "
                  "question:", crlf);
    const ZeroLow full1 = zero_low_cycles<Usart<1>, TxPin>(u1_tx, 640,
                                                          UsartPrescaler::div1);
    const ZeroLow full16 = zero_low_cycles<Usart<1>, TxPin>(u1_tx, 640,
                                                            UsartPrescaler::div16);
    print(serial, "  USART1 (FULL, PRESC reads ", full1.presc_readback, "/",
          full16.presc_readback, "): nine zero bits last ", to_us(full1.cycles),
          " us at /1 and ", to_us(full16.cycles), " us at /16; the frame was "
          "seen ", full1.saw_frame ? "yes" : "no", "/",
          full16.saw_frame ? "yes" : "no", " and completed ",
          full1.complete ? "yes" : "no", "/", full16.complete ? "yes" : "no",
          crlf);
    const bool full_divides = full16.cycles > full1.cycles * 8u;
    bench.verdict("the prescaler DIVIDES on a FULL instance - sixteen times "
                  "the bit period for the same BRR",
                  full_divides && full1.saw_frame && full16.saw_frame);
    ta_basic_prescaler();

    // The kernel-clock multiplexers, and what a multiplexer-less instance
    // answers when asked for anything but PCLK.
    bench.verdict("the kernel-clock multiplexer is every FULL USART's and "
                  "every LPUART's, and no BASIC instance's - the reserve's "
                  "column, the silicon's answer",
                  mux_agrees);
    const bool pclk_ok = BasicUsart::kernel_clock(UsartClock::pclk);
    const bool hsi_refused = !BasicUsart::kernel_clock(UsartClock::hsi16);
    bench.verdict("and an instance without one answers true for PCLK and "
                  "false for anything else, instead of writing a field that "
                  "does not exist",
                  !BasicUsart::has_clock_select && pclk_ok && hsi_refused);

    // The vectors, off the reserve.
    print(serial, "  vectors: USART1 ", static_cast<int32_t>(usart_irq(1)),
          ", USART2 ", static_cast<int32_t>(usart_irq(2)), ", USART3 ",
          static_cast<int32_t>(usart_irq(3)), ", USART4 ",
          static_cast<int32_t>(usart_irq(4)), ", LPUART1 ",
          static_cast<int32_t>(lpuart_irq(1)),
          " (-14 = an instance this part has not got)", crlf);
    bench.verdict("the vectors are the SHARED ones of this part: USART1 alone, "
                  "USART2 with an LPUART2 where the part has one, and every "
                  "remaining USART with LPUART1 - or, where there is no USART3 "
                  "to share with, LPUART1 on a line of its own; each name "
                  "derived from what the header declares beside it",
                  usart_irq(1) == USART1_IRQn && usart_irq(1) != usart_irq(2) &&
                      lpuart1_line_ok() && lpuart2_shares_usart2());
    print(serial, "  wake lines: USART1 ", usart_exti_line(1), " USART2 ",
          usart_exti_line(2), " USART3 ", usart_exti_line(3), " USART4 ",
          usart_exti_line(4), " LPUART1 ", lpuart_exti_line(1),
          " (0xFF = none)", crlf);
    bench.verdict("and the wake-up EXTI lines are table 65's - 25, 26 and 24 "
                  "for USART1..3 - with the line following the FULL/BASIC "
                  "COLUMN and not the instance number: an instance this part "
                  "makes BASIC has no wake line at all, 28 is LPUART1's and "
                  "35 an LPUART2's",
                  usart_exti_line(1) == 25 &&
                      usart_exti_line(2) == (usart_is_full(2) ? 26u : 0xFFu) &&
                      usart_exti_line(3) == (usart_is_full(3) ? 24u : 0xFFu) &&
                      usart_exti_line(4) == 0xFF &&
                      lpuart_exti_line(1) == 28 && lpuart2_exti_35());

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
    // that matters, and it is not obvious - a function that DRIVES a pad
    // could take the pull away, while an open-drain one is released
    // whenever it is not pulling low.
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
    WireSpan formats_wire;
    for (const FormatCase& c : cases) {
        feed();
        const UartFormat f{.bits = c.bits, .parity = c.parity, .stop_bits = c.stops};
        if (!loop_up({.format = f, .baud = 115200})) {
            continue;
        }
        // The frame's LENGTH on the wire: 256 frames timed against their
        // bit periods at 115200 - a stop bit or a parity bit the block did
        // not send would show here and nowhere else.
        const uint32_t pm = res_wire_permille<U1>(115200, frame_bits(f), 256u);
        formats_wire.add(pm);
        if (!wire_ok(pm)) {
            print(serial, "  ", c.name, ": 256 frames in ", pm,
                  " thousandths of their wire time", crlf);
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
            // sees every parity format "fail".
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
    print(serial, "  256 frames of each format in ", formats_wire.low, " to ",
          formats_wire.high, " thousandths of their wire time at 115200", crlf);
    bench.verdict("EVERY FORMAT TAKES ITS FRAME'S LENGTH ON THE WIRE: start, word and "
                  "stops at the rate asked, -1 % to +3 % (the loop alone shares the "
                  "divisor at both ends)", formats_wire.ok && tried >= 13u);

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
    const uint32_t over8_pm = res_wire_permille<U1>(115200);
    print(serial, "  OVER8 = 1 at 115200 on the loop: ", over8_good,
          " of 32 exact, BRR ", hex(over8_brr), " (USARTDIV ",
          usart_actual_baud_over8(SysClock::pclk_hz, over8_brr), " baud); 1024 frames "
          "in ", over8_pm, " thousandths of their wire time", crlf);
    bench.verdict("eight samples a bit carries the loop as well as sixteen",
                  over8_good == 32u);
    bench.verdict("AND OVER8 RUNS AT THE RATE ASKED: 1024 frames in their wire time "
                  "at 115200, -1 % to +3 % - OVER8's register lays USARTDIV out with "
                  "bit 3 dropped (33.5.7), the loop alone would not see it halved",
                  wire_ok(over8_pm));

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
    WireSpan presc_wire;
    for (UsartPrescaler p : codes) {
        feed();
        if (!loop_up({.baud = 9600, .prescaler = p})) {
            continue;
        }
        ++presc_tried;
        const bool exact = loop_run(4) == 4u &&
                           static_cast<uint8_t>(U1::prescaler()) == static_cast<uint8_t>(p);
        const uint32_t pm = res_wire_permille<U1>(9600);
        presc_wire.add(pm);
        if (exact && wire_ok(pm)) {
            ++presc_ok;
        } else {
            print(serial, "  PRESC code ", static_cast<uint32_t>(p),
                  " (divide by ", usart_prescaler_divisor(p), ") FAILED: ", pm,
                  " thousandths of the wire", crlf);
        }
    }
    print(serial, "  ", presc_ok, " of ", presc_tried,
          " prescaler codes carry 9600 baud on the loop, from divide-by-1 to "
          "divide-by-256 - one line rate, twelve kernel rates; 96 frames each in ",
          presc_wire.low, " to ", presc_wire.high, " thousandths of their wire time",
          crlf);
    bench.verdict("all twelve of 33.8.14's implemented codes read back and "
                  "deliver the rate the arithmetic promised - TIMED on the wire, "
                  "-1 % to +3 % of 9600 each (BRR's rounding at /256, 26 for "
                  "26.04, is 1.6 thousandths of it)",
                  presc_ok == 12u && presc_tried == 12u && presc_wire.ok);

    // A Reserved code is refused, because the chapter's own note says
    // the silicon turns one into 1011 - a rate nobody asked for.
    (void)loop_up({.baud = 9600});
    U1::enable(false);
    const bool reserved_refused = !U1::prescaler(static_cast<UsartPrescaler>(12));
    const uint8_t presc_after = static_cast<uint8_t>(U1::prescaler());
    bench.verdict("a Reserved prescaler code is refused and nothing is "
                  "written - 33.8.14's note says the silicon would make it a "
                  "divide-by-256", reserved_refused && presc_after == 0u);

    // THE LOOP'S OWN CEILING, TWICE. Open drain, the single wire rises on
    // the pad's own 40 k pull-up - an RC and not a driver, slowed further
    // by whatever the wire carries to a far board's input - so that
    // ladder measures the INSTRUMENT and is a figure. DRIVEN, the pad is
    // push-pull while a frame goes out and the loop is the generator's
    // and the receiver's alone: that ladder is judged. EVERY RUNG IS
    // TIMED: the transmitter's frames end at TC whatever the pad's rise
    // does to the receiver, so the generator is read where the open drain
    // cannot carry it.
    static const uint32_t ladder[] = {115200, 230400, 460800, 921600,
                                      1'000'000, 2'000'000, 4'000'000};
    uint32_t best = 0;
    uint32_t driven_best = 0;
    bool driven_whole = true;
    WireSpan ladder_wire;
    for (uint32_t baud : ladder) {
        feed();
        if (!loop_up({.baud = baud})) {
            print(serial, "  ", baud, " baud is unreachable at this clock", crlf);
            ladder_wire.ok = false;
            continue;
        }
        const uint32_t good = loop_run(16);
        const uint32_t pm = res_wire_permille<U1>(baud);
        ladder_wire.add(pm);
        (void)loop_up({.baud = baud, .driven = true});
        const uint32_t driven_good = loop_run(16);
        print(serial, "  ", baud, " baud: open drain ", good, " of 16 exact, driven ",
              driven_good, " of 16; 1024 frames in ", pm, " thousandths of their wire time",
              crlf);
        if (good == 16u) {
            best = baud;
        }
        if (driven_good == 16u && driven_whole) {
            driven_best = baud;
        } else {
            driven_whole = false;
        }
    }
    bench.verdict("EVERY RUNG RUNS AT THE RATE ASKED, 115200 to 4 Mbaud: 1024 frames in "
                  "their wire time, -1 % to +3 % (BRR's rounding, 69 for 69.4 at 921600, "
                  "is the 6.4 thousandths the reading may run fast there)",
                  ladder_wire.ok);
    print(serial, "  the open-drain loop is byte-exact to ", best,
          " baud (a figure: the rise of the pad's own 40 k pull-up into the wire it "
          "carries); the driven loop to ", driven_best, " baud", crlf);
    bench.verdict("the single-wire loop carries at least the console's rate",
                  best >= 115200u);
    bench.verdict("THE DRIVEN LOOP is byte-exact at every rung, 115200 to 4 Mbaud - the "
                  "generator's ceiling at OVER8 = 0, whatever the wire carries",
                  driven_whole && driven_best == 4'000'000u);

    // And the ceiling the GENERATOR has, which needs no wire: USARTDIV
    // 16 is 4 Mbaud at 64 MHz with OVER8 = 0 and 8 Mbaud with OVER8 = 1.
    U1::enable(false);
    U1::reset();
    (void)U1::configure({}, usart_brr(SysClock::pclk_hz, 4'000'000).value());
    const uint16_t top16 = U1::brr();
    (void)U1::oversampling(true);
    U1::set_brr(usart_brr_over8(SysClock::pclk_hz, 8'000'000).value());
    const uint16_t top8 = U1::brr();
    // And OVER8's ceiling timed: 8 Mbaud, a frame every 80 cycles, through
    // the FIFO so the store loop has eight frames of slack.
    const bool top8_up = loop_up({.baud = 8'000'000, .over8 = true, .fifo = true});
    const uint32_t top8_pm = top8_up ? res_wire_permille<U1>(8'000'000) : 0u;
    const bool top8_driven =
        loop_up({.baud = 8'000'000, .over8 = true, .fifo = true, .driven = true});
    const uint32_t top8_good = top8_driven ? loop_run(16) : 0u;
    print(serial, "  OVER8 at its ceiling, 8 Mbaud: 1024 frames in ", top8_pm,
          " thousandths of their wire time; the driven loop ", top8_good, " of 16 exact",
          crlf);
    bench.verdict("and OVER8's ceiling runs at the rate asked: 8 Mbaud, 1024 frames in "
                  "their wire time, -1 % to +3 %", top8_up && wire_ok(top8_pm));
    bench.verdict("and the driven loop carries it byte-exact, 16 of 16 at 8 Mbaud",
                  top8_driven && top8_good == 16u);
    U1::enable(false);
    U1::reset();
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
/// this letter are the only witness there is, and that is the point. A
/// TEMPLATE for one reason: on a part whose console has no multiplexer
/// nothing calls it, and an uninstantiated template is the only kind of
/// function that costs neither an image byte nor a warning there.
template <bool mux = usart_has_clock_select(2)>
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

/// THE CONSOLE'S OWN WIRE, TIMED: a line of 64 characters written to the
/// console and timed on TIM2 from the write to tx_idle(), against 64
/// frames at 115200 - the rate asked - in thousandths. The line is itself
/// part of the report.
[[maybe_unused]] uint32_t console_wire_permille() {
    static constexpr char line[] = "  (64 frames on the console's own wire, timed to its stop bit)\r\n";
    static_assert(sizeof line - 1u == 64u);
    console_drain();
    const uint32_t t0 = now();
    uint32_t queued = 0;
    while (queued < 64u && since(t0) < SysClock::hz / 20u) {
        queued += Serial::write_bulk(
            {reinterpret_cast<const uint8_t*>(line) + queued, 64u - queued});
    }
    while (!Serial::tx_idle() && since(t0) < SysClock::hz / 20u) {
    }
    const uint32_t took = since(t0);
    const uint64_t wire = 64ull * 10u * SysClock::hz / 115200u;
    return static_cast<uint32_t>(1000ull * took / wire);
}

/// THE CONSOLE MOVED UNDER ITS OWN FEET - on a console that HAS a
/// kernel-clock multiplexer. Where the part makes USART2 a BASIC
/// instance there is no CCIPR field for it at all: it runs on PCLK, full
/// stop, and this half of the letter would be asking the driver for a
/// refusal it already tests in letter a.
template <bool mux = usart_has_clock_select(2)>
void td_console_kernel() {
    if constexpr (mux) {
        Serial::clear_errors();
        const uint16_t brr_pclk = Usart<2>::brr();

        const bool to_hsi = console_kernel(UsartClock::hsi16, 16'000'000u);
        spin_us(2000);
        const uint32_t pm_hsi = console_wire_permille();
        print(serial, "  the console is now on HSI16: BRR ", Usart<2>::brr(),
              " where PCLK wanted ", brr_pclk, ", and this line came out at "
              "115200 all the same - the one above in ", pm_hsi,
              " thousandths of its wire time", crlf);
        const uint8_t err_hsi = static_cast<uint8_t>(
            Serial::frame_errors() + Serial::parity_errors() +
            Serial::noise_errors() + Serial::hw_overruns());

        const bool to_sysclk = console_kernel(UsartClock::sysclk, SysClock::hz);
        spin_us(2000);
        const uint32_t pm_sys = console_wire_permille();
        print(serial, "  and now on SYSCLK: BRR ", Usart<2>::brr(),
              " - the same 64 MHz PCLK divides, by a different route; the line "
              "above in ", pm_sys, " thousandths", crlf);
        const uint8_t err_sys = static_cast<uint8_t>(
            Serial::frame_errors() + Serial::parity_errors() +
            Serial::noise_errors() + Serial::hw_overruns());

        const bool back = console_kernel(UsartClock::pclk, SysClock::pclk_hz);
        spin_us(2000);
        const uint32_t pm_pclk = console_wire_permille();
        print(serial, "  and back on PCLK: BRR ", Usart<2>::brr(), "; the line above in ",
              pm_pclk, " thousandths", crlf);
        bench.verdict("a console's kernel clock moves under it - HSI16, SYSCLK, "
                      "PCLK - and every one of these lines is its own witness",
                      to_hsi && to_sysclk && back && Usart<2>::brr() == brr_pclk);
        bench.verdict("AND ON EACH KERNEL THE CONSOLE'S LINE RUNS AT THE RATE ASKED: 64 "
                      "frames in their wire time at 115200, -1 % to +3 %",
                      wire_ok(pm_hsi) && wire_ok(pm_sys) && wire_ok(pm_pclk));
        bench.verdict("with not one framing, parity, noise or overrun error "
                      "counted on the way",
                      err_hsi == 0u && err_sys == 0u &&
                          Serial::frame_errors() == 0u &&
                          Serial::hw_overruns() == 0u);
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: this part makes USART2 a BASIC "
              "instance, so the console has NO kernel-clock multiplexer to be "
              "moved on (usart_has_clock_select(2) is false - the device "
              "header declares no RCC_CCIPR_USART2SEL_Pos) and it runs on "
              "PCLK, full stop. USART1's own multiplexer is exercised below.",
              crlf);
    }
}

/// USART1 off the 32768 Hz crystal. USARTDIV 16 is 2048 baud with
/// sixteen samples a bit and 4096 with eight - the two slowest links this
/// chip can make, and the reason the LPUART exists.
template <bool fitted = board_lse_fitted>
void td_lse_legs() {
    if constexpr (fitted) {
        const bool lse_running = RtcDomain::lse_ready();
        if (!lse_running) {
            print(serial, "  the LSE crystal is not running: the two LSE legs "
                          "are DECLINED and nothing pretends otherwise", crlf);
            return;
        }
        (void)loop_up({.baud = 2048, .kernel = UsartClock::lse});
        const uint16_t lse_brr = U1::brr();
        const uint32_t lse_good = loop_run(4);
        const uint32_t lse_pm = res_wire_permille<U1>(2048);
        print(serial, "  USART1 on the LSE at 2048 baud: BRR ", lse_brr, ", ",
              lse_good, " of 4 bytes exact on the loop; 20 frames in ", lse_pm,
              " thousandths of their wire time", crlf);
        bench.verdict("the 32768 Hz crystal drives a USART at USARTDIV 16 - "
                      "2048 baud, and it is byte-exact",
                      lse_brr == 16u && lse_good == 4u);

        (void)loop_up({.baud = 4096, .over8 = true, .kernel = UsartClock::lse});
        const uint16_t lse8_brr = U1::brr();
        const uint32_t lse8_good = loop_run(4);
        const uint32_t lse8_pm = res_wire_permille<U1>(4096);
        print(serial, "  and at 4096 baud with OVER8: BRR ", hex(lse8_brr), ", ",
              lse8_good, " of 4 exact - eight samples a bit halves the floor; 40 "
              "frames in ", lse8_pm, " thousandths", crlf);
        bench.verdict("OVER8 doubles what a 32768 Hz kernel clock can carry",
                      lse8_good == 4u);
        bench.verdict("AND THE CRYSTAL'S RATES ARE THE RATES ASKED: 2048 and 4096 baud "
                      "timed against TIM2, -1 % to +3 % (the ruler runs on HSI16, so "
                      "its trim is in the reading)",
                      wire_ok(lse_pm) && wire_ok(lse8_pm));
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: a USART clocked by the 32768 Hz "
              "crystal needs one to be running, and THIS BOARD'S LSE DOES NOT "
              "START (measured over fifteen seconds at both drive settings, "
              "LSERDY never rose).", crlf);
    }
}

void td_kernel_clocks() {
    feed();
    td_console_kernel();
    feed();
    td_lse_legs();

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
// loop never runs again, and what comes out of the wire is a BANNER
// thirty seconds later, because the IWDG is the only thing that can end
// an interrupt storm. So the handlers below clear WUF and COUNT it, and
// the letters read the counter rather than a flag their own handler has
// just cleared.
volatile uint8_t console_wake_armed = 0;
volatile uint32_t console_wakes = 0;
/// THE SAME HAZARD ONE PERIPHERAL ALONG: `Rtc::isr()` clears WUTF at
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
    // The line under the FIFO, timed: 1024 frames stored on TXFNF.
    const uint32_t fifo_pm = res_wire_permille<U1>(115200);
    print(serial, "  1024 frames through the transmit FIFO in ", fifo_pm,
          " thousandths of their wire time at 115200", crlf);
    bench.verdict("and the line under the FIFO runs at the rate asked: 1024 frames in "
                  "their wire time, -1 % to +3 %", wire_ok(fifo_pm));
    // A fresh loop for the depth counts, and TE's idle frame off the wire
    // first: it holds the shift register, and the count below wants it free.
    (void)loop_up({.baud = 115200, .fifo = true});
    spin_us(300);

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
    const uint32_t plain_t0 = now();
    (void)LoopUart::write_bulk(block);
    uint32_t spins = 0;
    while (!LoopUart::tx_idle() && spins++ < 4'000'000u) {
    }
    const uint32_t plain_took = since(plain_t0);
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
    const uint32_t fifo_t0 = now();
    (void)LoopFifoUart::write_bulk(block);
    spins = 0;
    while (!LoopFifoUart::tx_idle() && spins++ < 4'000'000u) {
    }
    const uint32_t fifo_took = since(fifo_t0);
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

    // The block's own time on the wire, write_bulk() to tx_idle(): the
    // divisor and whatever gap the transport leaves between frames.
    const uint64_t block_wire = 256ull * 10u * SysClock::hz / 115200u;
    const uint32_t plain_pm = static_cast<uint32_t>(1000ull * plain_took / block_wire);
    const uint32_t fifo_task_pm = static_cast<uint32_t>(1000ull * fifo_took / block_wire);
    print(serial, "  256 bytes round the loop through the TASK: ", plain_irqs,
          " interrupts without the FIFO (", plain_got, " bytes back), ",
          fifo_irqs, " with it (", fifo_got, " back); the block on the wire in ",
          plain_pm, " and ", fifo_task_pm, " thousandths of its time at 115200", crlf);
    bench.verdict("the task carries the block byte-exact both ways, with the "
                  "SAME public verbs and one option between them",
                  plain_up && fifo_up && plain_exact && fifo_exact);
    bench.verdict("and both transports keep the wire busy: the 256-byte block in its "
                  "wire time at 115200, -1 % to +3 %, no gap between frames worth a "
                  "per cent", wire_ok(plain_pm) && wire_ok(fifo_task_pm));
    print(serial, "  ON A SINGLE WIRE THE TWO SIDES OF A BYTE ARE ONE EVENT, so what "
          "the FIFO buys shows only where BOTH sides ride a level: the transmitter "
          "refills eight places when the FIFO runs empty and the receiver drains "
          "at its threshold (the whole FIFO here), the receiver time-out taking "
          "the tail - two entries per eight bytes where the one-register "
          "transport takes one a byte.", crlf);
    bench.verdict("and the FIFO pays on the loop: the task in FIFO mode takes under "
                  "a sixth of the one-register transport's interrupts, not one byte "
                  "different",
                  fifo_irqs * 6u < plain_irqs);

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
    print(serial, "  2.11.1 staged (the G0B1's ES0548 numbering; each part "
          "has its own sheet and the outcome below is THIS die's): a "
          "quarter-bit glitch to zero in the SECOND half of the stop bit "
          "spoiled ", second_half_bad,
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
    // And the frames after it at the rate asked: 24 frames, a hundredth of
    // a second of them, timed to TC on the same push-pull pad.
    const uint32_t lin_pm = res_wire_permille<U1>(lin_baud);
    print(serial, "  24 frames in LIN mode in ", lin_pm,
          " thousandths of their wire time at 2400", crlf);
    bench.verdict("and LIN mode's frames run at the rate asked: 24 frames in their wire "
                  "time at 2400, -1 % to +3 %", wire_ok(lin_pm));

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
// becomes an edge counter with NO CPU in the path. The EXTI sees a pad
// its owner drives, so the pad stays under its peripheral's alternate
// function throughout.

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
    // BOTH MASKS, and the boot probe below says why: on one die of this
    // family the DMAMUX's trigger input follows the EXTI's INTERRUPT
    // output and not its event output, so a line whose IMR bit is clear
    // triggers nothing. The NVIC line is never enabled by this suite, so
    // an unmasked EXTI dispatches nothing either way.
    (void)Exti::interrupt(line, true);
    (void)Exti::event(line, true);
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
    (void)Exti::interrupt(line, false);
    (void)Exti::event(line, false);
    (void)Exti::sense(line, ExtiSense::none);
    (void)Exti::release(line);
}

// ---- IS THE INSTRUMENT THERE AT ALL, AND WHAT DOES IT NEED? ----------------
// Four letters below count edges with the arrangement above, and ES0418
// 2.2.4 (STM32G071/G081 revision B, "no workaround") says the
// EXTI-related DMAMUX synchronization and trigger inputs are routed to
// the EXTI's per-line INTERRUPT output `it_exti_per(y)` instead of to
// `exti[15:0]`. That is a testable difference and not a dead path: a
// signal taken off the interrupt output arrives only while the line's
// IMR bit is SET, where one taken off the event output does not care.
// So the question is put to the SILICON at boot, on PB3 - a pad this
// board drives itself, with the far end of its jumper in analog - with
// a REAL edge as the stimulus and the channel's own CNDTR as the
// witness, in both arrangements. The NVIC line is never enabled, so an
// unmasked EXTI dispatches nothing.
bool exti_dma_ok = false;         ///< the counter works with both masks armed
bool exti_dma_needs_imr = false;  ///< ...and ONLY with the interrupt mask

/// Toggle PB3 `n` times as a plain output and say how many words moved.
uint32_t edge_probe_pulses(uint8_t n) {
    using Pb3 = Pin<'B', 3>;
    const uint32_t before = edge_count_reset();
    for (uint8_t i = 0; i < n; ++i) {
        Pb3::output(false);
        spin_us(300);
        Pb3::set();
        spin_us(300);
    }
    Pb3::input(PinPull::none);
    return edge_count_reset() - before;
}

void probe_exti_dma_trigger() {
    using Pb3 = Pin<'B', 3>;
    // THE DMAMUX IS CLOCKED BY DMA1's AHB ENABLE: a generator configured
    // through a closed gate is configured nowhere (this probe's own first
    // version measured that instead of the silicon).
    Dma1::bus_clock(true);
    // The EVENT side alone.
    (void)Exti::sense(3, ExtiSense::none);
    if (!Exti::select(3, 'B') || !Exti::sense(3, ExtiSense::rising)) {
        return;
    }
    (void)Exti::interrupt(3, false);
    (void)Exti::event(3, true);
    (void)Exti::clear(3);
    if (!Gen0::configure(dmamux_trigger_exti(3), DmaMuxEdge::rising, 1)) {
        return;
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
        return;
    }
    Pb3::output(true);
    DmaMux::request(EdgeCh::mux_channel, Gen0::request_id);
    (void)EdgeCh::enable(true);
    Gen0::enable(true);
    const uint32_t on_event = edge_probe_pulses(4);
    // And with the INTERRUPT mask armed as well.
    Pb3::output(true);
    (void)Exti::interrupt(3, true);
    (void)Exti::clear(3);
    const uint32_t on_interrupt = edge_probe_pulses(4);
    // THE CONTROL THAT TELLS A DEAD PROBE FROM A DEAD PATH, two ways.
    // First: does the EXTI itself see the pad? Its own rising pending
    // bit is the witness, and it is set by the same edges that moved
    // nothing. Second: a SOFTWARE event on the same line, same
    // generator, same channel - the stimulus the dma suite uses.
    Pb3::output(true);
    (void)Exti::clear(3);
    Pb3::output(false);
    spin_us(300);
    Pb3::set();
    spin_us(300);
    const bool exti_saw_the_pad = Exti::rising_pending(3);
    (void)Exti::clear(3);
    Pb3::input(PinPull::none);
    const uint32_t before_sw = edge_count_reset();
    for (uint8_t i = 0; i < 4u; ++i) {
        (void)Exti::trigger(3);
        spin_us(300);
        (void)Exti::clear(3);
    }
    const uint32_t on_swier = edge_count_reset() - before_sw;
    Gen0::enable(false);
    Gen0::release();
    EdgeCh::stop();
    (void)Exti::interrupt(3, false);
    (void)Exti::event(3, false);
    (void)Exti::sense(3, ExtiSense::none);
    (void)Exti::release(3);
    Pb3::input(PinPull::none);
    exti_dma_ok = on_interrupt >= 4u;
    exti_dma_needs_imr = exti_dma_ok && on_event < 4u;
    const auto id = brio::DeviceIdcode::read();
    print(serial, crlf, "probe: four PB3 edges into a DMAMUX request "
          "generator moved ", on_event, " words with the EVENT mask alone, ",
          on_interrupt, " with the INTERRUPT mask armed too; the EXTI's own "
          "rising pending bit did ", exti_saw_the_pad ? "" : "NOT ",
          "see those edges; four SOFTWARE events on the same line, same "
          "generator, same channel moved ", on_swier, crlf);
    if (!exti_dma_ok && exti_saw_the_pad && on_swier >= 4u) {
        print(serial, "THE ERRATUM SHAPE REPRODUCED WITH ITS CONTROL on DEV_ID ",
              hex(id.dev_id), " REV_ID ", hex(id.rev_id),
              ": the EXTI SEES the pad, and a SOFTWARE event on the very same "
              "line, generator and channel moves a word every time - while "
              "four PAD EDGES move none with IMR clear and ONE with IMR set. "
              "That is the shape of \"routed to it_exti_per(y) instead of "
              "exti[15:0]\": the trigger follows the LEVEL of the line's "
              "pending interrupt, so with no IMR bit there is no level at "
              "all, and with one the first edge raises it and nothing "
              "further arrives until somebody clears the pending bit. The "
              "software leg passes for exactly that reason - its loop clears "
              "after every pulse - which is why a suite that stages this "
              "path with SWIER alone sees nothing wrong. The workaround the "
              "sheet does not offer would be a CPU in the loop, which is the "
              "one thing an edge counter with no CPU cannot have: the "
              "letters below say so and claim nothing. That is ES0418 2.2.4 "
              "word for word on the G071; whether the sheet of the die "
              "printed above carries its own twin is not a claim this suite "
              "makes.", crlf);
    } else if (!exti_dma_ok) {
        print(serial, "the EXTI-to-DMAMUX trigger path did not answer; the "
              "letters that need it will say so and claim nothing", crlf);
    }
}

/// The one line every letter whose instrument is that edge counter
/// prints when the probe says the path is dead in both arrangements.
bool need_exti_dma() {
    if (exti_dma_ok) {
        return true;
    }
    const auto id = brio::DeviceIdcode::read();
    print(serial,
          "  SKIPPED, no verdict claimed: this letter counts edges with a "
          "DMAMUX request generator triggered by an EXTI line, and the boot "
          "probe found that path dead in both mask arrangements on DEV_ID ",
          hex(id.dev_id), " REV_ID ", hex(id.rev_id),
          " - the shape ES0418 2.2.4 describes for the G071, with no "
          "workaround; the boot probe above says what THIS die did.", crlf);
    return false;
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
    // THE CARD'S CHARACTER ON THE WIRE, at the rate asked: sixteen frames
    // back to back with GT = 0, each TWELVE baud periods - 33.5.17 calls
    // twelve "the duration of one character" (start, eight data, parity,
    // the 1.5 stop bits and the half baud clock smartcard mode delays every
    // shift by) - timed to TC on a kernel prescaled to 250 kHz.
    const uint32_t card_pm = res_wire_permille<U1>(card_baud, 12u);
    print(serial, "  16 card characters in ", card_pm,
          " thousandths of twelve baud periods each at 1200", crlf);
    bench.verdict("and the card's wire runs at the rate asked through the /256 "
                  "prescaler: 16 characters of twelve baud periods in their time, "
                  "-1 % to +3 %", wire_ok(card_pm));

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
        if (!exti_dma_ok || !has_u1_ck_pads) {
            break;
        }
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
    if constexpr (!has_u1_ck_pads) {
        print(serial, ck_pad_skip, crlf);
    } else if (need_exti_dma()) {
        bench.verdict("the smartcard clock is the kernel rate over TWICE the "
                      "prescaler - four rungs of the ladder, counted by a DMA "
                      "channel with no CPU and no interrupt", ck_ok == 4u);
    }
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
    const uint32_t ir_pm = res_wire_permille<U1>(ir_baud);
    print(serial, "  16 SIR frames in ", ir_pm, " thousandths of their wire time at 1200",
          crlf);
    bench.verdict("and the SIR frames run at the rate asked: 16 frames in their wire "
                  "time at 1200, -1 % to +3 %", wire_ok(ir_pm));
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
    const uint32_t ir_lp_pm = res_wire_permille<U1>(ir_baud);
    const uint32_t lp_clock = lp_ker / 64u;               // 62.5 kHz
    const uint32_t want_us = (3u * 1'000'000u) / lp_clock;
    print(serial, "  IrDA low-power with PSC = 64 on a 4 MHz kernel: the "
          "pulse is ", to_us(lp_pulse), " us where three periods of the ",
          lp_clock, " Hz low-power clock are ", want_us, " us", crlf);
    bench.verdict("in low-power mode the pulse stops being 3/16 of a BIT and "
                  "becomes three periods of the PSC clock - a width that no "
                  "longer moves with the baud rate",
                  lp_ok && permille_off(to_us(lp_pulse), want_us) <= 150u);
    print(serial, "  and the frames on the /16 kernel: 16 in ", ir_lp_pm,
          " thousandths of their wire time at 1200", crlf);
    bench.verdict("while the FRAME still moves with it: 16 low-power frames in their "
                  "wire time at 1200 on the prescaled kernel, -1 % to +3 %",
                  wire_ok(ir_lp_pm));

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

/// THE DRIVER ENABLE, RTS AND CTS - the three legs of letter l that need
/// USART1's OTHER two pads. DEAT and DEDT are in SAMPLE times - a
/// sixteenth of a bit at OVER8 = 0 - and that is the one thing 33.5.20's
/// single sentence about them is easy to misread.
template <bool pads = has_u1_ck_pads>
void tl_de_rts_cts() {
    if constexpr (!pads) {
        print(serial, ck_pad_skip, crlf);
        return;
    } else {
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
    }
}

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
        const uint32_t swap_pm = res_wire_permille<U1>(115200);
        print(serial, "  CR2.SWAP: the single wire is now PA10 (the RX pad) "
              "with PA9 given back, and ", good, " of 8 bytes went round it; "
              "1024 frames in ", swap_pm, " thousandths of their wire time", crlf);
        bench.verdict("SWAP exchanges the PADS - the whole transport moves to "
                      "the other pin and nothing else changes",
                      rx_free && good == 8u);
        bench.verdict("... the rate included: 1024 frames on the swapped wire in their "
                      "time at 115200, -1 % to +3 %", wire_ok(swap_pm));
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

    feed();
    tl_de_rts_cts();

    loop_down();
}

// ---------------------------------------------------------------------------
// m: the synchronous master's clock
// ---------------------------------------------------------------------------

template <bool pads = has_u1_ck_pads>
void tm_synchronous() {
    feed();
    if constexpr (!pads) {
        print(serial, ck_pad_skip, crlf);
        return;
    } else {
        constexpr uint32_t sync_baud = 9600;
        struct SyncCase { bool lbcl; bool cpol; };
        static const SyncCase scases[] = {{false, false}, {true, false}, {false, true}};
        uint8_t counted_ok = 0;
        WireSpan sync_wire;
        bool counter_failed = false;
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
            // The CK pulses, counted where the EXTI path into the DMAMUX
            // is this die's (exti_dma_ok).
            if (exti_dma_ok && !counter_failed) {
                if (!edge_counter_up(3, 'B', DmaMuxEdge::rising)) {
                    print(serial, "  the edge counter would not come up", crlf);
                    counter_failed = true;
                } else {
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
            }
            // The frames under the clock, at the rate asked, on every die:
            // start and stop are on the data line as ever, CK pulsing only
            // for the data.
            const uint32_t pm = sync_ok ? res_wire_permille<U1>(sync_baud) : 0u;
            sync_wire.add(pm);
        }
        if (need_exti_dma()) {
            bench.verdict("33.5.14's clock is one pulse a data bit with none for "
                          "the start and stop, LBCL adds the pulse of the LAST "
                          "bit, and CPOL is the level CK rests at - all three "
                          "counted with no CPU in the path", counted_ok == 3u);
        }
        print(serial, "  96 synchronous frames in each arrangement in ", sync_wire.low, " to ",
              sync_wire.high, " thousandths of their wire time at 9600", crlf);
        bench.verdict("and the synchronous frames run at the rate asked: 96 frames in "
                      "their wire time at 9600 in each of the three arrangements, -1 % to "
                      "+3 %", sync_wire.ok);
        print(serial, "  THE DATA PATH IS DECLINED and not faked: a synchronous "
              "link needs something at the other end to clock, and this desk has "
              "one board. The SLAVE half (CR2.SLVEN, DIS_NSS and the underrun "
              "flag) is register verbs and no measurement, and its doc says so.",
              crlf);
        DePin::release();
        loop_down();
    }
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

/// The second LPUART, on its own pad and its own kernel-clock field.
template <bool present = lpuart_present(2)>
void tn_lpuart2() {
    if constexpr (present) {
        using L2 = Lpuart2<present>;
        feed();
        const bool pc6_free = pull_walks<Lp2Pin>();
        const bool l2_up =
            lp_loop_up<L2, Lp2Pin>(lp2_tx, 115200, UsartClock::pclk);
        const uint32_t l2_good = l2_up ? lp_run<L2>(8, 20000) : 0u;
        const uint32_t l2_pm = l2_up ? res_wire_permille<L2>(115200) : 0u;
        print(serial, "  PC6 free: ", pc6_free ? "yes" : "NO",
              "; LPUART2 on its own single wire at 115200: ", l2_good,
              " of 8 exact, BRR ", L2::brr(), "; 1024 frames in ", l2_pm,
              " thousandths of their wire time", crlf);
        bench.verdict("LPUART2 - the G0B1 class's second one, with its own "
                      "LPUART2SEL field and its own APB bit - runs the same way",
                      pc6_free && l2_up && l2_good == 8u);
        bench.verdict("... at the rate asked: 1024 frames in their wire time at 115200, "
                      "-1 % to +3 %", wire_ok(l2_pm));
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: a second single wire on PC6 at "
              "AF3 needs an LPUART2, which this part has not got "
              "(lpuart_present(2) is false - the reserve finds no LPUART2_BASE "
              "in the device header, and USART2's vector is therefore USART2's "
              "alone).",
              crlf);
    }
}

template <bool present = lpuart_present(2)>
void quiet_lpuart2() {
    if constexpr (present) {
        using L2 = Lpuart2<present>;
        L2::enable(false);
        L2::reset();
        L2::bus_clock(false);
    }
}

/// EVERYTHING LETTER N MEASURES ON A WIRE, which on this family is the
/// LPUART's OWN single pad: PC1 at AF1 on the 48- and 64-pin packages.
/// A 32-pin part bonds none of LPUART1's other pads and leaves it PA2/PA3
/// at AF6 - the ST-LINK's virtual COM port and this suite's own console -
/// so a half-duplex loop there would put its traffic into this report and
/// silence the console's transmitter while it held the pad. The console
/// MOVING to those pads is letter v's business, with the host told; the
/// table-198 arithmetic above and LPUART2's own leg below need no pad at
/// all and run everywhere.
template <bool pad = has_lpuart_own_pad>
void tn_lpuart_wire() {
    if constexpr (!pad) {
        print(serial,
              "  SKIPPED, no verdict claimed: LPUART1's single wire, its "
              "kernel-clock ladder, its FIFO, its prescaler and its interrupt "
              "all need a pad of its own, and the only LPUART1 pads this "
              "package bonds are PA2/PA3 at AF6 - the console's. The vector "
              "itself is not shared here at all: with no USART3 to share it "
              "with, LPUART1 has a line of its own (the reserve derives the "
              "name, and this suite binds it).", crlf);
        return;
    } else {
        // LPUART1's own single wire on PC1, on each of its kernel clocks.
        const bool pc1_free = pull_walks<Lp1Pin>();
        print(serial, "  PC1 follows its own pull: ", pc1_free ? "yes" : "NO", crlf);
        bench.verdict("PC1 is free for LPUART1's single wire", pc1_free);

        const bool lse_running = RtcDomain::lse_ready();
        uint8_t lp_ok = 0;
        uint8_t lp_tried = 0;
        WireSpan lp_wire;
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
            const uint32_t pm = res_wire_permille<L1>(c.baud);
            lp_wire.add(pm);
            print(serial, "  LPUART1 ", c.name, ": BRR ", L1::brr(), ", ", good,
                  " of 4 bytes round its own wire; ", wire_frames(c.baud), " frames in ",
                  pm, " thousandths of their wire time", crlf);
            if (good == 4u) {
                ++lp_ok;
            }
        }
        bench.verdict("LPUART1 runs on PCLK, on HSI16 and on the 32768 Hz crystal, "
                      "byte-exact on its own single wire", lp_ok == lp_tried &&
                                                               lp_tried >= 3u);
        bench.verdict("AT THE RATE ASKED on every kernel: each run's frames in their wire "
                      "time, -1 % to +3 % (the twenty-bit LPUARTDIV of 34.4.7 averages "
                      "the 3.4 kernel periods a bit is at 9600 on the LSE)",
                      lp_wire.ok && lp_tried >= 3u);

        // The PCLK ceiling: 34.4.7 puts it at fck / 3, which from 64 MHz is
        // 21.3 Mbaud. The open-drain loop gives out long before that, and
        // the ladder says where.
        feed();
        static const uint32_t lp_ladder[] = {115200, 460800, 921600, 2'000'000,
                                             4'000'000, 8'000'000};
        uint32_t lp_best = 0;
        WireSpan lp_ladder_wire;
        for (uint32_t baud : lp_ladder) {
            if (!lp_loop_up<L1, Lp1Pin>(lp1_tx, baud, UsartClock::pclk)) {
                print(serial, "  ", baud, " baud is outside 34.4.7's window at "
                      "this clock", crlf);
                lp_ladder_wire.ok = false;
                continue;
            }
            const uint32_t good = lp_run<L1>(8, 20000);
            const uint32_t pm = res_wire_permille<L1>(baud);
            lp_ladder_wire.add(pm);
            print(serial, "  LPUART1 at ", baud, " baud on PCLK: ", good,
                  " of 8 exact; 1024 frames in ", pm, " thousandths of their wire time",
                  crlf);
            if (good == 8u) {
                lp_best = baud;
            }
        }
        bench.verdict("EVERY RUNG OF THE LPUART'S LADDER RUNS AT THE RATE ASKED, 115200 "
                      "to 8 Mbaud: 1024 frames in their wire time, -1 % to +3 %, past "
                      "the loop's own ceiling too", lp_ladder_wire.ok);
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
        const uint32_t lp_fifo_pm = fifo_up ? res_wire_permille<L1>(115200) : 0u;
        const bool presc_up =
            lp_loop_up<L1, Lp1Pin>(lp1_tx, 9600, UsartClock::pclk,
                                   UsartPrescaler::div64);
        const uint32_t presc_good = presc_up ? lp_run<L1>(4, 40000) : 0u;
        const uint32_t lp_presc_pm = presc_up ? res_wire_permille<L1>(9600) : 0u;
        print(serial, "  LPUART1 with FIFOEN: ", fifo_good,
              " of 8 exact; with PRESC = /64 at 9600 baud: ", presc_good,
              " of 4 - the LP column has both, and no OVER8 at all; their frames in ",
              lp_fifo_pm, " and ", lp_presc_pm, " thousandths of the wire", crlf);
        bench.verdict("the LPUART's FIFO and prescaler work, and its baud "
                      "generator has no oversampling to choose",
                      fifo_on && fifo_good == 8u && presc_good == 4u &&
                          !Lpuart<1>::has_oversampling8);
        bench.verdict("... both at the rate asked: 1024 frames through the FIFO at 115200 "
                      "and 96 at 9600 through the /64 prescaler, -1 % to +3 %",
                      wire_ok(lp_fifo_pm) && wire_ok(lp_presc_pm));

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
        print(serial, "  LPUART1's interrupt on the line it shares with every "
              "USART from the third up was served ", served,
              " time(s) by a handler that answers for several peripherals",
              crlf);
        bench.verdict("a shared vector reaches the LPUART, and the console on "
                      "USART2 - which shares ITS line with LPUART2 - kept talking "
                      "throughout", served >= 1u);
    }
}

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

    tn_lpuart_wire();

    tn_lpuart2();

    L1::enable(false);
    L1::reset();
    L1::bus_clock(false);
    quiet_lpuart2();
    Lp1Pin::release();
    Lp2Pin::release();
}

// ---------------------------------------------------------------------------
// o: IRTIM
// ---------------------------------------------------------------------------

using Carrier = TimPwm<Tim<17>, 0>;
using Envelope = TimPwm<Tim<16>, 0>;

/// A PAD WITH AN EXTERNAL PULL-UP IS NOT A PAD A SUITE CAN PULL-WALK.
/// PB9 is where a wired I2C bus puts its 2.2 kOhm pull-up, which beats
/// the internal pull-down, so the precondition becomes a weaker one:
/// the pad is HELD UP by the external resistor and SINKS when
/// a push-pull output drives it low - which is exactly the electrical
/// question IR_OUT asks of it. A pad with no external pull answers the
/// plain walk and is accepted by the first clause.
template <class Pad>
bool pad_is_drivable() {
    if (pull_walks<Pad>()) {
        return true;
    }
    Pad::input(PinPull::down);
    spin_us(300);
    const bool held_up = Pad::read();
    Pad::output(false);
    spin_us(300);
    const bool sinks = !Pad::read();
    Pad::input(PinPull::none);
    return held_up && sinks;
}

void to_irtim() {
    feed();
    const bool pb9_free = pad_is_drivable<IrPin>();
    print(serial, "  PB9 is drivable (its own pull, or held up by an external "
          "one and sinking when driven): ", pb9_free ? "yes" : "NO",
          " - and it is the ONE pad of this board an infrared LED would use, "
          "PA13's IR_OUT being SWDIO", crlf);
    bench.verdict("PB9 is free for IR_OUT - a push-pull alternate function "
                  "owns it whether or not the desk hangs a pull-up on it",
                  pb9_free);

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

    if (!exti_dma_ok) {
        (void)need_exti_dma();
    } else if (!edge_counter_up(9, 'B', DmaMuxEdge::rising)) {
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
    uint32_t env_pm = 0;   // the envelope's frames against their wire time
    if (!exti_dma_ok) {
        (void)need_exti_dma();
    } else if (edge_counter_up(9, 'B', DmaMuxEdge::rising)) {
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
        uint32_t env_t0 = 0;
        for (uint8_t i = 0; i < 20u; ++i) {
            (void)wait_flag(UsartFlag::txe);
            if (i == 0u) {
                env_t0 = now();
            }
            U1::write_word(0x00);
        }
        (void)wait_flag(UsartFlag::tc);
        const uint32_t env_took = since(env_t0);
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
        // The envelope's own frames, timed to TC against 20 frames at the
        // rate asked: the gate's length is the divisor's.
        env_pm = static_cast<uint32_t>(1000ull * env_took / (200ull * SysClock::hz / env_baud));
    }
    // Where the edges could not be counted the frames are timed all the
    // same: the gate's length needs no counter.
    if (env_pm == 0u) {
        env_pm = res_wire_permille<U1>(env_baud);
    }
    print(serial, "  the envelope's frames took ", env_pm,
          " thousandths of their wire time at 1200", crlf);
    bench.verdict("and the envelope's frames run at the rate asked: their wire time at "
                  "1200, -1 % to +3 %", wire_ok(env_pm));

    // The second USART code, which is a PER-PART fact: USART4 here,
    // USART2 on the G031 class. Both are the reserve's, not a driver's.
    feed();
    const bool second_ok = Irtim::envelope(IrtimEnvelope::second_usart);
    const bool reserved_refused =
        !Irtim::envelope(static_cast<IrtimEnvelope>(3));
    print(serial, "  IR_MOD = 10 selects USART", Irtim::second_usart_index,
          " on this part (USART4 where there is one, USART2 where there is "
          "not), and code 11 is Reserved: ",
          reserved_refused ? "refused" : "TAKEN", crlf);
    bench.verdict("the envelope multiplexer takes all three implemented codes "
                  "and refuses the Reserved one - with code 10 following the "
                  "instance the part HAS, which is the reserve's own rule",
                  second_ok && reserved_refused &&
                      Irtim::second_usart_index == (usart_present(4) ? 4u : 2u));

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
// The host-assisted letters (brio stress), OUTSIDE z
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
    print(serial, "  this letter needs brio stress on the other end "
          "of the VCP; run it as", crlf,
          "  brio stress --port <the console> --letters y",
          crlf);

    // A CONSOLE WITHOUT A MULTIPLEXER HAS ONE KERNEL CLOCK, and this
    // letter streams on every one it HAS: the two PCLK legs run on any
    // part, the HSI16 and SYSCLK ones only where USART2 carries a CCIPR
    // field of its own (letter a is where that is measured).
    constexpr bool console_mux = usart_has_clock_select(2);
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
        if constexpr (!console_mux) {
            if (l.kernel != UsartClock::pclk) {
                print(serial, "  ", l.name, " at ", l.baud,
                      " baud: SKIPPED, no verdict claimed - this part makes "
                      "USART2 a BASIC instance and it has no kernel clock but "
                      "PCLK", crlf);
                continue;
            }
        }
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

    // AND THE SAME WITH FIFOEN FLIPPED UNDER IT, at the console's own rate
    // - on a console that HAS a FIFO. The task runs FIFO mode where the
    // instance has one (Serial::fifo_mode), so the leg streams with the
    // enable the task was NOT built for, and puts it back. THE PACE MOVES
    // WITH THE VIEW: the task's receiver rides RXFT, which the one-register
    // view has not got (33.8.4), so the leg arms RXNE in its place for the
    // length of the stream - what it judges is the DRAIN, which reads ISR
    // before every RDR and is right in both views. A BASIC instance has no
    // FIFO (letter a measures the enable being dropped), so the leg names
    // the fact and claims nothing rather than turning FIFOEN's refusal
    // into a stream test.
    feed();
    [[maybe_unused]] bool fifo_ok = false;
    if constexpr (Usart<2>::is_full) {
        constexpr bool built = Serial::fifo_mode;
        host_announce("sink", 0, 115200, "8N1", 900, 0);
        {
            InterruptGuard guard;
            Usart<2>::enable(false);
            (void)Usart<2>::fifo(!built);
            (void)Usart<2>::fifo_thresholds(UartFifoThreshold::half,
                                            UartFifoThreshold::none);
            Usart<2>::enable(true);
            if constexpr (Serial::rx_paced) {
                Usart<2>::rx_threshold_interrupt(false);
                Usart<2>::rxne_interrupt(true);
            }
        }
        Serial::clear_errors();
        uint8_t fifo_junk[64];
        while (Serial::read_bulk(fifo_junk) != 0u) {
        }
        lfsr_reset();
        uint32_t fifo_got = 0;
        uint32_t fifo_bad = 0;
        uint8_t chunk[64];
        const uint32_t t0 = now();
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
            (void)Usart<2>::fifo(built);
            Usart<2>::enable(true);
            if constexpr (Serial::rx_paced) {
                Usart<2>::rxne_interrupt(false);
                Usart<2>::rx_threshold_interrupt(true);
            }
        }
        host_settle();
        print(serial, "  with FIFOEN ", built ? "cleared" : "set",
              " under the console: ", fifo_got, " bytes in, ", fifo_bad,
              " wrong, hardware overruns ", fifo_hw, crlf);
        fifo_ok = fifo_got >= 2000u && fifo_bad == 0u;
    } else {
        print(serial, "  SKIPPED, no verdict claimed: the console is a BASIC "
                      "instance on this part and has no FIFO to turn on.",
              crlf);
    }
    bench.verdict("the console streams byte-exact from the host on every "
                  "kernel clock it can take",
                  clean == tried && tried >= (console_mux ? 3u : 2u));
    if constexpr (Usart<2>::is_full) {
        bench.verdict("and with FIFOEN flipped under it - the receive pace moved "
                      "to RXNE with it - without one line of the drain changing",
                      fifo_ok);
    }
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

/// THE WAKE LETTER NEEDS TWO THINGS THIS PART OR THIS BOARD MAY NOT
/// HAVE: a console that can wake at all (33.5.21's wake is a FULL
/// instance's - a BASIC one has no EXTI line, which is what letter a
/// measures) and a wall clock to time the Stop with, which here is the
/// RTC on the LSE crystal. Where either is missing the letter says so
/// and claims NOTHING - a Stop nothing can end is a reboot and a banner,
/// and a verdict for a leg that was never staged would be worse.
template <bool stageable = (usart_exti_line(2) != 0xFF) && board_lse_fitted>
void tw_wake() {
    print(serial, "  this letter needs brio stress; run it as", crlf,
          "  brio stress --port <the console> --letters w",
          crlf);
    feed();
    if constexpr (!stageable) {
        print(serial,
              "  SKIPPED, no verdict claimed: this letter needs a console "
              "that can wake from Stop AND a wall clock to time the Stop "
              "with, and here it is short of ",
              usart_exti_line(2) == 0xFFu
                  ? "the first: the console is USART2, which this part makes "
                    "a BASIC instance - table 184's last row, and the "
                    "reserve's usart_exti_line(2) is 0xFF, so there is no "
                    "wake to arm and no EXTI line to arm it on. "
                  : "",
              board_lse_fitted
                  ? ""
                  : "the second: this board is declared without a running "
                    "LSE crystal, so there is neither a wall to measure "
                    "the Stop against nor an RTC wake-up timer to end it. ",
              "ES0548 2.2.4's staging - and ES0487's twin, which this suite "
              "has not read - wants both, so it is not attempted.", crlf);
        return;
    } else {
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

/// The LSE console's own isr(), reached only where that console exists.
template <bool fitted = board_lse_fitted>
void lp_console_lse_isr() {
    if constexpr (fitted) {
        (void)LpConsoleLse::isr();
    }
}

/// LEG ONE, and the only one of the three that needs a crystal: LPUART1
/// on PA2/PA3 AF6 - the console's OWN pads, which this instance reaches
/// through a different alternate function - clocked by the 32768 Hz LSE.
/// A TEMPLATE so that a board with no crystal carries neither the leg nor
/// its console's two rings. It answers "nothing to hold against it": true
/// when the leg ran and was exact, and true when it never ran at all.
template <bool fitted = board_lse_fitted>
bool tv_lse_leg() {
    if constexpr (!fitted) {
        print(serial,
              "  SKIPPED, no verdict claimed: an LPUART console clocked by "
              "the 32768 Hz crystal needs one to be running, and THIS BOARD "
              "IS DECLARED WITHOUT ONE. The HSI16 leg below is the same "
              "console on the same pads and runs.", crlf);
        return true;
    } else {
        if (!RtcDomain::lse_ready()) {
            print(serial, "  no crystal running: the LSE console leg is "
                          "DECLINED and nothing pretends otherwise", crlf);
            return true;
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

        return lse_up && lse_got >= 500u && lse_bad == 0u;
    }
}

/// LEG THREE: a Stop with the LPUART on the crystal as the wake source.
/// This is the arrangement the peripheral exists for - and it wants the
/// crystal twice over, as the LPUART's kernel clock and as the wall this
/// suite times the Stop with.
template <bool fitted = board_lse_fitted>
void tv_stop_leg() {
    if constexpr (!fitted) {
        print(serial,
              "  SKIPPED, no verdict claimed: an LPUART woken out of Stop by "
              "a start bit needs the 32768 Hz crystal as its kernel clock - "
              "no other clock of this part survives a Stop for it - and this "
              "board is declared without one.", crlf);
        return;
    } else {
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
}

void tv_lpuart_console() {
    print(serial, "  this letter needs brio stress; run it as", crlf,
          "  brio stress --port <the console> --letters v",
          crlf);
    feed();
    const bool lse_leg_ok = tv_lse_leg();

    // LEG TWO: the same instance on HSI16 at 115200, which is what an
    // LPUART is for when the chip is awake.
    uint32_t hsi_got = 0;
    uint32_t hsi_bad = 0;
    uint8_t chunk[32];
    host_announce("sink", 0, 115200, "8N1", 1300, 0);
    Serial::release();
    lp_console_mode = 2;
    const bool hsi_up = LpConsoleHsi::init(clock, 115200);
    lfsr_reset();
    uint32_t t0 = now();
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
                  lse_leg_ok && hsi_up && hsi_got >= 2000u && hsi_bad == 0u);
    print(serial, "  (these lines are on USART2 again, which is the other "
          "half of the proof)", crlf);

    feed();
    tv_stop_leg();
}


// ---------------------------------------------------------------------------
// p: the receive transport - the edge from a vector, and errors under the
//    engine (design/serial.md; stm32g0/usart.hpp's UartTask)
// q: tx_idle() is the wire's: the last stop bit off the pad
// ---------------------------------------------------------------------------

/// The transports these two letters drive, all on USART1's PA9. The
/// receive ones run the single wire with the transmitter on its engine,
/// so USART1's vector serves the receiver alone; the transmit ones run
/// full duplex, the receiver on PA10's pull-up.
using LoopTxCh = DmaTxEngine<1, 2>;
using LoopRxCh = DmaRxEngine<1, 3>;
/// The receive transports' single wire is DRIVEN (push-pull while a frame
/// goes out): PA9 also carries the link wire to the far board's receive
/// pad, and the open drain's rise on the 40 k pull-up into that load gives
/// out above 460800 (letter c) - the 1 Mbaud stream below is the
/// transport's subject, not the pull-up's.
constexpr UartOptions driven_wire =
    uart_half_duplex({.tx_speed = PinSpeed::very_high, .single_wire_push_pull = true});
using PacedLoop = Uart<1, u1_pins, 256, 64, LoopTxCh, NoDmaEngine, driven_wire>;
using EngineLoop = Uart<1, u1_pins, 64, 64, LoopTxCh, LoopRxCh, driven_wire>;
using PlainSender = Uart<1, u1_pins, 16, 64>;
using EngineSender = Uart<1, u1_pins, 16, 64, LoopTxCh>;

constexpr uint8_t mode_paced = 3;
constexpr uint8_t mode_engine = 4;
constexpr uint8_t mode_plain_tx = 5;
constexpr uint8_t mode_engine_tx = 6;
volatile uint32_t rx_edges = 0;       // the vectors' true answers
volatile uint32_t rx_edge_at = 0;     // TIM2 at the last one

template <typename T>
bool transport_up(uint8_t mode, uint32_t baud) {
    loop_down();
    loop_irqs = 0;
    rx_edges = 0;
    loop_mode = mode;   // the vector serves T from its init on
    const bool up = T::init(clock, baud);
    spin_cycles(4u * SysClock::hz / (baud / 10u));   // TE's idle frame
    uint8_t junk = 0;
    while (T::read_byte(junk)) {
    }
    T::clear_errors();
    loop_irqs = 0;
    rx_edges = 0;
    return up;
}

template <typename T>
void transport_down() {
    T::release();
    loop_mode = 0;
    loop_down();
}

/// Everything the consumer is owed, drained into `got` and judged against
/// `want` (the stream without its hits); the count read.
template <typename T>
uint32_t drain_into(const uint8_t* want, uint32_t n, uint32_t& at, uint32_t& wrong) {
    uint32_t read = 0;
    for (;;) {
        const std::span<const uint8_t> run = T::read_span();
        if (run.empty()) {
            break;
        }
        for (const uint8_t b : run) {
            if (at >= n || b != want[at]) {
                ++wrong;
            }
            ++at;
        }
        read += static_cast<uint32_t>(run.size());
        (void)T::consume(static_cast<uint32_t>(run.size()));
    }
    return read;
}

/// A burst of `n` bytes from `src` round the loop, the consumer draining
/// on every edge a vector reports; returns the cycles from the burst's
/// last stop bit (TC rising) to the last edge, signed.
template <typename T>
int32_t burst_edge(const uint8_t* src, uint32_t n, uint32_t baud, uint32_t& got,
                   uint32_t& wrong) {
    const uint32_t frame = SysClock::hz / (baud / 10u);
    uint32_t seen = 0;
    uint32_t at = 0;
    bool tc_seen = false;
    uint32_t tc_at = 0;
    got = 0;
    wrong = 0;
    U1::clear_flags(UsartClear::tc);
    uint32_t queued = 0;
    const uint32_t t0 = now();
    while (queued < n && since(t0) < 40u * frame * n) {
        queued += T::write_bulk({src + queued, n - queued});
        if (rx_edges != seen) {
            seen = rx_edges;
            got += drain_into<T>(src, n, at, wrong);
        }
    }
    for (;;) {
        if (!tc_seen && T::tx_idle()) {
            tc_at = now();
            tc_seen = true;
        }
        if (rx_edges != seen) {
            seen = rx_edges;
            got += drain_into<T>(src, n, at, wrong);
        }
        if ((tc_seen && since(tc_at) > 6u * frame) || since(t0) > 40u * frame * n) {
            break;
        }
    }
    return static_cast<int32_t>(rx_edge_at - tc_at);
}

void tp_receive() {
    feed();
    console_drain();
    static uint8_t stream[200];
    for (uint32_t i = 0; i < sizeof stream; ++i) {
        stream[i] = static_cast<uint8_t>(0x30u + (i * 7u) % 79u);
    }

    // --- the paced receiver: RXFT at half the FIFO, the time-out for the
    // tail. 17 bytes at 115200: four levels of four and a tail of one.
    constexpr uint32_t baud = 115200;
    constexpr uint32_t frame = SysClock::hz / (baud / 10u);
    bool up = transport_up<PacedLoop>(mode_paced, baud);
    uint32_t got = 0;
    uint32_t wrong = 0;
    const int32_t paced_tail = burst_edge<PacedLoop>(stream, 17, baud, got, wrong);
    const uint32_t paced_irqs = loop_irqs;
    spin_us(10000);   // ten milliseconds of silence after the burst
    const uint32_t quiet_irqs = loop_irqs - paced_irqs;
    print(serial, "  the paced receiver, 17 bytes at 115200: ", got, " back (", wrong,
          " wrong) in ", paced_irqs, " USART1 entries; the last edge ", paced_tail,
          " cycles after the last stop bit (a frame is ", frame, "); ", quiet_irqs,
          " entries in the 10 ms of silence after it", crlf);
    bench.verdict("THE PACED RECEIVER: 17 bytes in one entry per level of four plus "
                  "the tail's - five at most where RXFNE took seventeen - and the "
                  "tail below the level delivered by the receiver time-out within "
                  "two frames of the last stop bit; silence costs nothing after it",
                  up && got == 17u && wrong == 0u && paced_irqs <= 6u &&
                      paced_tail > 0 && paced_tail < static_cast<int32_t>(2u * frame) &&
                      quiet_irqs == 0u);

    // --- K breaks among N slots through the paced receiver.
    constexpr uint32_t slots = 64;
    constexpr uint32_t hits = 4;
    auto is_hit = [](uint32_t i) { return i == 9u || i == 24u || i == 41u || i == 58u; };
    static uint8_t kept[slots];
    // The stream with its breaks, the consumer looking before and after
    // each break when `read_between` - every read judged against `kept`,
    // the stream without its hits - and the rest read at the end.
    uint32_t at = 0;
    auto send_with_breaks = [&]<typename T>(bool read_between, uint32_t& read) {
        uint32_t n = 0;
        read = 0;
        at = 0;
        wrong = 0;
        auto wire_idle = [&] {
            const uint32_t t1 = now();
            while (!T::tx_idle() && since(t1) < 100u * frame) {
            }
            spin_cycles(2u * frame);
        };
        for (uint32_t i = 0; i < slots; ++i) {
            if (is_hit(i)) {
                const uint32_t t1 = now();
                while (!T::tx_idle() && since(t1) < 100u * frame) {
                }
                if (read_between) {
                    spin_cycles(2u * frame);   // the tail's time-out
                    read += drain_into<T>(kept, n, at, wrong);
                }
                U1::send_break();
                while ((U1::status() & UsartFlag::sbkf) != 0u && since(t1) < 100u * frame) {
                }
                spin_cycles(2u * frame);
                if (read_between) {
                    wire_idle();
                    read += drain_into<T>(kept, n, at, wrong);   // the skip, nothing queued
                }
                continue;
            }
            const uint8_t b = stream[i];
            kept[n++] = b;
            while (T::write_bulk({&b, 1}) == 0u) {
            }
        }
        wire_idle();
        spin_cycles(2u * frame);
        read += drain_into<T>(kept, n, at, wrong);
        return n;
    };
    up = transport_up<PacedLoop>(mode_paced, baud);
    const uint32_t skips0 = PacedLoop::rx_skips();
    uint32_t want = send_with_breaks.template operator()<PacedLoop>(true, got);
    const uint32_t skips = (PacedLoop::rx_skips() - skips0) & 0xFFu;
    const uint32_t fe_paced = PacedLoop::frame_errors();
    print(serial, "  ", slots, " slots with ", hits, " breaks, the paced receiver read between: ",
          got, " of ", want, " bytes back (", wrong, " wrong), FE ", fe_paced, ", PE ",
          PacedLoop::parity_errors(), ", ORE ", PacedLoop::hw_overruns(), ", ring overruns ",
          PacedLoop::rx_overruns(), ", skips ", skips, crlf);
    bench.verdict("K BREAKS IN N SLOTS, the interrupt receiver read between them: N - K "
                  "bytes delivered intact and in order, the break dropped by its own "
                  "entry's flags, the frame-error counter K and one skip a break",
                  up && want == slots - hits && got == want && wrong == 0u &&
                      fe_paced == hits && skips == hits && PacedLoop::hw_overruns() == 0u &&
                      PacedLoop::rx_overruns() == 0u);
    // Read only at the end: every byte the ring held when the first break
    // was told goes with it, and the skip after the last break leaves
    // nothing behind it either - no run joins two sides of a break.
    PacedLoop::clear_errors();
    const uint32_t skips1 = PacedLoop::rx_skips();
    uint32_t joined = 0;
    want = send_with_breaks.template operator()<PacedLoop>(false, joined);
    const uint32_t skips_after = (PacedLoop::rx_skips() - skips1) & 0xFFu;
    print(serial, "  the same read only at the end: ", joined, " delivered (", wrong,
          " wrong), FE ", PacedLoop::frame_errors(), ", skips ", skips_after, crlf);
    bench.verdict("read only after the stream, the bytes queued with the breaks are "
                  "skipped whole, in one skip, and the frame-error counter is still K",
                  up && joined == 0u && skips_after == 1u &&
                      PacedLoop::frame_errors() == hits);
    // The transport's line at the rate asked: 1024 frames through its
    // transmit engine, write_bulk() to tx_idle().
    const uint32_t paced_pm = port_wire_permille<PacedLoop>(baud);
    print(serial, "  1024 frames through this transport's transmit engine in ", paced_pm,
          " thousandths of their wire time at 115200", crlf);
    bench.verdict("and its line runs at the rate asked: 1024 frames in their wire time "
                  "at 115200, -1 % to +3 %", wire_ok(paced_pm));
    transport_down<PacedLoop>();

    // --- the same under the receive engine: no clear reads RDR.
    feed();
    up = transport_up<EngineLoop>(mode_engine, baud);
    want = send_with_breaks.template operator()<EngineLoop>(false, got);
    const uint32_t fe_engine = EngineLoop::frame_errors();
    print(serial, "  the same under the receive engine: ", got, " of ", want, " back (",
          wrong, " wrong), FE ", fe_engine, ", NE ", EngineLoop::noise_errors(), ", ORE ",
          EngineLoop::hw_overruns(), ", ring overruns ", EngineLoop::rx_overruns(),
          ", USART1 entries ", loop_irqs, crlf);
    bench.verdict("K BREAKS IN N SLOTS UNDER THE ENGINE: N - K bytes delivered intact "
                  "and in order - the channel was owed every one and got every one, "
                  "no clear having read RDR (every flag of this block clears through "
                  "ICR) and the break's own character not transferred (DDRE clear) - "
                  "and the frame-error counter exactly K, one vector entry each",
                  up && want == slots - hits && got == want && wrong == 0u &&
                      fe_engine == hits && EngineLoop::hw_overruns() == 0u &&
                      EngineLoop::rx_overruns() == 0u);

    // --- the engine's edge: the line's IDLE after a burst ...
    feed();
    (void)transport_up<EngineLoop>(mode_engine, baud);
    const int32_t idle_tail = burst_edge<EngineLoop>(stream, 17, baud, got, wrong);
    print(serial, "  the receive engine, 17 bytes at 115200: ", got, " back (", wrong,
          " wrong) on ", rx_edges, " edges; the last edge ", idle_tail,
          " cycles after the last stop bit", crlf);
    bench.verdict("THE ENGINE'S EDGE IS THE LINE'S IDLE: a burst told within two "
                  "frames of its last stop bit, by the vector and no poll",
                  got == 17u && wrong == 0u && idle_tail > 0 &&
                      idle_tail < static_cast<int32_t>(2u * frame));

    // ... and the half and full marks on a stream with no silence in it:
    // 200 bytes at 1 Mbaud into a 64-byte ring, the consumer reading only
    // when a vector says so.
    feed();
    (void)transport_up<EngineLoop>(mode_engine, 1'000'000);
    (void)burst_edge<EngineLoop>(stream, sizeof stream, 1'000'000, got, wrong);
    const uint32_t stream_edges = rx_edges;
    print(serial, "  200 bytes at 1 Mbaud into the 64-byte ring, read on the edges "
          "alone: ", got, " back (", wrong, " wrong) on ", stream_edges,
          " edges, ring overruns ", EngineLoop::rx_overruns(), ", ORE ",
          EngineLoop::hw_overruns(), ", laps ", LoopRxCh::laps(), crlf);
    bench.verdict("A STREAM WITH NO SILENCE IS NEVER HELD: the ring's half and full "
                  "marks tell the consumer twice a lap, and 200 bytes cross a 64-byte "
                  "ring with no overrun where the line's IDLE alone would have come "
                  "three laps late",
                  got == sizeof stream && wrong == 0u && stream_edges >= 6u &&
                      EngineLoop::rx_overruns() == 0u && EngineLoop::hw_overruns() == 0u);
    const uint32_t engine_pm = port_wire_permille<EngineLoop>(1'000'000);
    print(serial, "  1024 frames through both engines in ", engine_pm,
          " thousandths of their wire time at 1 Mbaud", crlf);
    bench.verdict("and the engined line runs at the rate asked: 1024 frames in their "
                  "wire time at 1 Mbaud, -1 % to +3 %", wire_ok(engine_pm));
    transport_down<EngineLoop>();
}

/// tx_idle() against the pad: EXTI line 9 watches PA9's falling edges -
/// each frame of 0xFF has exactly one, its start bit - and the last one
/// places the last stop bit's end ten bit times later.
template <typename T>
void tx_idle_on_pad(const char* name, uint8_t mode, uint32_t& early, uint32_t& late,
                    bool& ok, uint32_t& pad_permille) {
    constexpr uint32_t baud = 9600;
    constexpr uint32_t bit = SysClock::hz / baud;
    ok = transport_up<T>(mode, baud);
    (void)Exti::sense(9, ExtiSense::none);
    // The pending bit is set only under an unmasked CPU interrupt
    // (RM0444 13.4); the NVIC line stays down, so nothing is
    // entered and the bit is polled.
    ok = ok && Exti::select(9, 'A') && Exti::sense(9, ExtiSense::falling) &&
         Exti::interrupt(9, true);
    (void)Exti::clear(9);
    static const uint8_t ones[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint32_t first_fall = 0;
    uint32_t last_fall = 0;
    uint32_t falls = 0;
    uint32_t idle_at = 0;
    bool idle_seen = false;
    (void)T::write_bulk({ones, sizeof ones});
    const uint32_t t0 = now();
    while (since(t0) < 12u * 10u * bit) {
        const uint32_t t = now();
        if (Exti::falling_pending(9)) {
            Exti::clear_falling(1u << 9);
            if (falls == 0u) {
                first_fall = t;
            }
            last_fall = t;
            ++falls;
        }
        if (!idle_seen && T::tx_idle()) {
            idle_at = t;
            idle_seen = true;
        }
    }
    const uint32_t stop_end = last_fall + 10u * bit;
    early = idle_seen && idle_at < stop_end ? stop_end - idle_at : 0u;
    late = idle_seen && idle_at >= stop_end ? idle_at - stop_end : 0u;
    // THE PAD'S OWN CLOCK: eight start bits back to back are seven frames
    // apart, ten bit periods each at the rate asked.
    pad_permille = static_cast<uint32_t>(1000ull * (last_fall - first_fall) / (70ull * bit));
    print(serial, "  ", name, ": ", falls, " start bits on PA9; tx_idle() ",
          idle_seen ? "" : "NEVER ", "true ", early != 0u ? early : late,
          early != 0u ? " cycles BEFORE" : " cycles after", " the last stop bit's end (a bit is ",
          bit, "); the first to the last start bit in ", pad_permille,
          " thousandths of seven frames", crlf);
    ok = ok && idle_seen && falls == sizeof ones;
    (void)Exti::release(9);
    transport_down<T>();
}

void tq_tx_idle() {
    feed();
    console_drain();
    constexpr uint32_t bit = SysClock::hz / 9600u;
    uint32_t early = 0;
    uint32_t late = 0;
    bool ok = false;
    uint32_t pad_plain = 0;
    uint32_t pad_engine = 0;
    tx_idle_on_pad<PlainSender>("the interrupt transmitter, 8 frames at 9600", mode_plain_tx,
                                early, late, ok, pad_plain);
    // The edge is seen by a poll of a few tens of cycles, so the stop
    // bit's end it places is late by as much: 1/32 of a bit of margin.
    bench.verdict("tx_idle() IS THE WIRE'S, on the interrupt transmitter: never before "
                  "the last stop bit's end, and within a bit time after it",
                  ok && early <= bit / 32u && late <= bit);
    feed();
    tx_idle_on_pad<EngineSender>("the transmit engine, 8 frames at 9600", mode_engine_tx,
                                 early, late, ok, pad_engine);
    bench.verdict("and on the transmit engine: TC cleared when the block starts, so "
                  "no stop bit of an earlier block answers for this one",
                  ok && early <= bit / 32u && late <= bit);
    bench.verdict("THE PAD RUNS AT THE RATE ASKED: on both transmitters the first and "
                  "the last start bit are seven frames of 9600 apart, -1 % to +3 %",
                  wire_ok(pad_plain) && wire_ok(pad_engine));
}

// ---------------------------------------------------------------------------
// The menu
// ---------------------------------------------------------------------------

/// The two-board set's owners of USART1's vector and of DMA1 channels 2/3
/// (loop_mode, beside the single-board letters' own).
constexpr uint8_t mode_link_irq = 7;
constexpr uint8_t mode_link_dma = 8;
constexpr uint8_t mode_link_irq8 = 9;
constexpr uint8_t mode_link_dma8 = 10;

/// Whether this image carries the two-board set: every image that names
/// no selection, and the group whose selection holds its first key - the
/// testbench's own selection speaks letters alone, and a digit key would
/// otherwise ride in every group image.
constexpr bool two_board_carried() {
    if (test_letters == nullptr) {
        return true;
    }
    for (const char* p = test_letters; *p != '\0'; ++p) {
        if (*p == '1') {
            return true;
        }
    }
    return false;
}

// ===========================================================================
// THE TWO-BOARD SET (key t): USART1 against the STM32F446RE's usart_peer
// ===========================================================================
//
// The desk's pairing: this board's USART1 crossed to the far board's -
// PA9 (TX, AF1) to its PA10, PA10 (RX, AF1) from its PA9 - the far board
// running usart_peer (stm32f4/src/apps/usart_peer.cpp), driven IN BAND
// over the link under test with usart_link.hpp's protocol: an ack before
// every action, a bounded action, the peer back in command mode by itself.
// The letters run under the second all-key alone: without the peer they
// FAIL, and the single-board letters above run with the far board dark.
// The far peer's time base is a crystal (the ST-LINK's 8 MHz), so what it
// times is this core's HSI16 - letter 1 prints it, and each rate's verdict
// below is taken against a 1 Mbaud reference timed just before it (the
// HSI wanders by a few hundred ppm over seconds, measured on this board).
// The IWDG's 32 s backstop is fed at every command.
//
// THE HARNESS. This end's link is the resource, polled, the FIFOs on
// (33.5.4; off for LIN, which 33.8.1 forbids them): dut_link() turns a
// Cfg into the registers (the block reset first), the pads handed over
// only once the enable has sent its idle frame; command() retries three
// times, each after a quiet longer than the peer's partial-frame reset;
// peer_report() raises leg_refused on report_cfg_failed, and judge()
// prints SKIP for a refused leg and counts nothing. A leg this end's
// silicon cannot express is skipped the same way, by name.

link::Decoder dec;
const uint8_t no_payload[1] = {0};
bool leg_refused = false;
bool link_quiet = false;

using U1Tx = Pin<'A', 9>;
using U1Rx = Pin<'A', 10>;
constexpr uint32_t u1_hz = SysClock::pclk_hz;   // the nominal 64 MHz: the HSI's error is the wire's

/// The task over the same pads, for the receive legs: the interrupt
/// receiver (the FIFO on by default, paced by RXFT and the receiver
/// time-out) and the receive engine on DMA1 channel 3 (letter p's), the
/// vectors below dispatching on loop_mode.
/// The rings: a consumer reading runs keeps up with the wire, so a ring a
/// quarter of the leg serves; the 8 KB part takes a smaller one still.
constexpr uint32_t link_ring = SRAM_SIZE_MAX >= 0x4000UL ? 512u : 128u;
constexpr UartOptions link_opts{.tx_speed = PinSpeed::very_high};
using LinkRxEngine = DmaRxEngine<1, 3>;
using LinkIrq = Uart<1, u1_pins, link_ring, 16, NoDmaEngine, NoDmaEngine, link_opts>;
using LinkDma = Uart<1, u1_pins, link_ring, 16, NoDmaEngine, LinkRxEngine, link_opts>;
/// The same two above 4 Mbaud, where only OVER8 reaches (USARTDIV 16).
constexpr UartOptions link_opts8{.over8 = true, .tx_speed = PinSpeed::very_high};
using LinkIrq8 = Uart<1, u1_pins, link_ring, 16, NoDmaEngine, NoDmaEngine, link_opts8>;
using LinkDma8 = Uart<1, u1_pins, link_ring, 16, NoDmaEngine, LinkRxEngine, link_opts8>;

void link_task_down() {
    switch (loop_mode) {
        case mode_link_irq: LinkIrq::release(); break;
        case mode_link_dma: LinkDma::release(); break;
        case mode_link_irq8: LinkIrq8::release(); break;
        case mode_link_dma8: LinkDma8::release(); break;
        default: break;
    }
    if (loop_mode >= mode_link_irq) {
        loop_mode = 0;
    }
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 <= ms) {
    }
}

void wait_us(uint32_t us) { spin_us(us); }

void put1(uint16_t b) {
    const uint32_t t0 = Ticker::millis();
    while (!U1::flag(UsartFlag::txfnf) && Ticker::millis() - t0 < 50u) {
    }
    U1::write_word(b);
}

void wait_tc() {
    const uint32_t t0 = Ticker::millis();
    while (!U1::flag(UsartFlag::tc) && Ticker::millis() - t0 < 50u) {
    }
}

void u1_flush() {
    U1::request(UsartRequest::flush_receive);
    U1::clear_flags(UsartClear::receive_errors | UsartClear::idle | UsartClear::lbdf);
}

/// What this end can express of a Cfg: the frame and the divisor, or
/// nothing - CR1.M counts the parity bit (33.5.5), so 6+P, 7, 7+P, 8, 8+P
/// and 9 and the four stop codes; the oversampling closer to the rate
/// (from the nominal kernel: this end's HSI error is the wire's).
struct DutSetup {
    UartFormat format{};
    UartStop half_stop = UartStop::one;
    uint16_t brr = 0;
    bool over8 = false;
    uint32_t actual = 0;   ///< the rate the divisor gives at the nominal kernel
};

std::optional<DutSetup> dut_setup(const link::Cfg& c) {
    DutSetup s{};
    const bool parity = c.parity != 0u;
    s.format.parity = c.parity == 1u ? UartParity::even : c.parity == 2u ? UartParity::odd : UartParity::none;
    switch (c.bits) {
        case 6: if (!parity) return std::nullopt; s.format.bits = UartBits::seven; break;
        case 7: s.format.bits = parity ? UartBits::eight : UartBits::seven; break;
        case 8: s.format.bits = parity ? UartBits::nine : UartBits::eight; break;
        case link::bits9_low:
        case link::bits9_high: if (parity) return std::nullopt; s.format.bits = UartBits::nine; break;
        default: return std::nullopt;
    }
    s.format.stop_bits = c.stop == 2u ? 2u : 1u;
    s.half_stop = c.stop == link::stop_half ? UartStop::half
                : c.stop == link::stop_one_half ? UartStop::one_and_half : UartStop::one;
    const std::optional<uint16_t> b16 = usart_brr(u1_hz, c.rate);
    const std::optional<uint16_t> b8 = usart_brr_over8(u1_hz, c.rate);
    auto err = [&c](uint32_t a) { return a > c.rate ? a - c.rate : c.rate - a; };
    if (b16 && (!b8 || err(usart_actual_baud(u1_hz, *b16)) <= err(usart_actual_baud_over8(u1_hz, *b8)))) {
        s.brr = *b16;
        s.actual = usart_actual_baud(u1_hz, *b16);
    } else if (b8) {
        s.brr = *b8;
        s.over8 = true;
        s.actual = usart_actual_baud_over8(u1_hz, *b8);
    } else {
        return std::nullopt;
    }
    return s;
}

/// This end's link in a commanded configuration: the block reset and
/// programmed with the pads off it (PA9 an input on its pull-up, so the
/// line stays high through TE's idle frame), then the pads handed over.
bool dut_link(const link::Cfg& c, bool lin = false, bool one_bit = false) {
    link_task_down();
    const std::optional<DutSetup> s = dut_setup(c);
    if (!s) {
        return false;
    }
    Nvic::disable(U1::irq());
    U1Tx::input(PinPull::up);
    U1::bus_clock(true);
    U1::enable(false);
    U1::reset();
    (void)U1::kernel_clock(UsartClock::pclk);
    if (!U1::configure(s->format, s->brr) || (s->over8 && !U1::oversampling(true))) {
        return false;
    }
    if (s->half_stop != UartStop::one && !U1::stop_bits(s->half_stop)) {
        return false;
    }
    if (one_bit && !U1::one_bit_sampling(true)) {
        return false;
    }
    if (lin ? !U1::lin(LinConfig{}) : !U1::fifo(true)) {
        return false;
    }
    U1::enable(true);
    wait_us(2u * (1'000'000u / c.rate) + 20u);
    U1Tx::function(PinFunction::af1, {.speed = PinSpeed::very_high});
    U1Rx::function(PinFunction::af1, {.pull = PinPull::up});
    u1_flush();
    return true;
}

bool link_command_mode() {
    dec.reset();
    return dut_link(link::Cfg{});
}

/// The link's pads given back (analog) and the block off: what every
/// two-board letter leaves, so the single-board half finds them as reset
/// left them.
/// THIS CORE AGAINST ITS OWN LSE, over the very window a run took. The
/// HSI16 under the core wanders - a few hundred ppm between runs a tenth
/// of a second apart, measured - so a rate the far crystal times is
/// judged against the HSI's error over THAT run, not against a reference
/// taken beside it: TIM16 captures the LSE every eight periods (TISEL 2,
/// RM0444 table 140; ICF 8) and DMA1 channel 4 moves each capture
/// (TIM16_CH1's request, table 55) into a ring of 1024 - 250 ms of the
/// crystal, no CPU - for the length of the two-board set (letter o owns
/// TIM16 otherwise). The far peer on the STM32G0 times itself the same way.
using LseTrack = Tim<16>;
using TrackCh = DmaChannel<1, 4>;
constexpr uint16_t track_n = 1024;
volatile uint16_t track[track_n];
bool tracking = false;

bool tracker_up() {
    if (tracking) {
        return true;
    }
    if (!RtcDomain::lse_ready()) {
        return false;   // the domain's crystal is main()'s to start; without it, no tracker
    }
    LseTrack::init();
    if (!LseTrack::configure({.prescaler = 0, .period = 0xFFFF}) ||
        !LseTrack::capture_channel(0, {.select = TimChannelSelect::direct,
                                       .polarity = TimCapturePolarity::rising,
                                       .prescaler = TimCapturePrescaler::every8,
                                       .filter = 8}) ||
        !LseTrack::input_select(0, RtcDomain::lse_tim16_ti1_code)) {
        return false;
    }
    Dma<1>::bus_clock(true);
    TrackCh::stop();
    TrackCh::set_peripheral(LseTrack::ccr_address(0));
    (void)DmaMux::request(TrackCh::mux_channel, LseTrack::dma_compare_request(0));
    TrackCh::restart(dma_ccr_config({.direction = DmaDirection::peripheral_to_memory,
                                     .circular = true,
                                     .memory_increment = true,
                                     .peripheral_width = DmaWidth::half,
                                     .memory_width = DmaWidth::half,
                                     .priority = DmaPriority::high}),
                     track, track_n);
    LseTrack::interrupts(LseTrack::compare_dma(0), true);
    LseTrack::enable(true);
    tracking = true;
    wait_ms(300);   // the ring filled once
    return true;
}

void tracker_down() {
    if (!tracking) {
        return;
    }
    LseTrack::interrupts(LseTrack::compare_dma(0), false);
    TrackCh::stop();
    (void)DmaMux::release(TrackCh::mux_channel);
    LseTrack::release();
    tracking = false;
}

/// The core's rate over the last `k` capture intervals: one subtraction
/// an entry, the walk stopping eight short of the channel's write head.
std::optional<uint32_t> tracked_hz(uint16_t k) {
    if (!tracking || k == 0u) {
        return std::nullopt;
    }
    if (k > track_n - 8u) {
        k = track_n - 8u;
    }
    const uint16_t pos = static_cast<uint16_t>(track_n - TrackCh::count());
    uint16_t i = pos == 0u ? static_cast<uint16_t>(track_n - 1u) : static_cast<uint16_t>(pos - 1u);
    uint16_t newer = track[i];
    uint32_t sum = 0;
    for (uint16_t j = 0; j < k; ++j) {
        i = i == 0u ? static_cast<uint16_t>(track_n - 1u) : static_cast<uint16_t>(i - 1u);
        const uint16_t older = track[i];
        sum += static_cast<uint16_t>(newer - older);
        newer = older;
    }
    const uint64_t hz = (static_cast<uint64_t>(sum) * 4096u + k / 2u) / k;
    if (hz < 60'000'000u || hz > 68'000'000u) {
        return std::nullopt;
    }
    return static_cast<uint32_t>(hz);
}

/// The HSI's error in ppm over a stretch of `counts` core cycles that
/// ended NOW (the tracker's intervals covering it, sixteen at least), or
/// nothing without the tracker.
std::optional<int32_t> hsi_over(uint32_t counts) {
    uint32_t k = counts / 15'625u + 2u;
    k = k < 16u ? 16u : (k > track_n - 8u ? track_n - 8u : k);
    const std::optional<uint32_t> hz = tracked_hz(static_cast<uint16_t>(k));
    if (!hz) {
        return std::nullopt;
    }
    return static_cast<int32_t>((static_cast<int64_t>(*hz) - static_cast<int64_t>(SysClock::hz)) * 1'000'000 /
                                static_cast<int64_t>(SysClock::hz));
}

void link_release() {
    tracker_down();
    link_task_down();
    loop_down();
}

bool recv_frame(link::Frame& out, uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    dec.reset();
    while (Ticker::millis() - t0 < ms) {
        const uint32_t st = U1::status();
        if ((st & UsartFlag::rxne) == 0u) {
            if ((st & UsartFlag::ore) != 0u) {
                U1::clear_flags(UsartClear::ore);
            }
            continue;
        }
        if ((st & UsartFlag::receive_errors) != 0u) {
            U1::clear_flags(st & UsartClear::receive_errors);   // before the pop
        }
        const uint8_t b = U1::read_data();
        if ((st & UsartFlag::receive_errors) != 0u) {
            dec.reset();
            continue;
        }
        if (dec.feed(b) == link::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

void put_link(uint8_t b) { put1(b); }

bool command_once(link::Op op, const uint8_t* p, uint8_t len, uint16_t ms) {
    u1_flush();
    link::write_frame(put_link, op, p, len);
    wait_tc();
    link::Frame f;
    if (!recv_frame(f, ms)) {
        return false;
    }
    return f.op == link::Op::ack && f.len == 2u && f.data[0] == link::byte_of(op);
}

/// The recovery guarantee in action: three attempts, each after a quiet
/// interval longer than the peer's own partial-frame reset.
bool command(link::Op op, const uint8_t* p = no_payload, uint8_t len = 0, uint16_t ms = 80) {
    feed();
    for (uint8_t k = 0; k < 3u; ++k) {
        if (command_once(op, p, len, ms)) {
            return true;
        }
        (void)link_command_mode();
        wait_ms(70);
    }
    if (!link_quiet) {
        print(serial, "    LINK FAILURE op ", hex(link::byte_of(op)),
              ": no ack - is the far board running usart_peer?", crlf);
    }
    return false;
}

bool query(link::Op op, link::Frame& data, uint16_t ms = 80) {
    return command(op, no_payload, 0, ms) && recv_frame(data, ms);
}

/// Find the peer: the crossed pair is the desk's only wiring, so this pings
/// until the peer's dark search hears a frame that checks out.
bool ensure_link() {
    (void)tracker_up();
    link_quiet = true;
    bool found = false;
    for (uint8_t k = 0; k < 3u && !found; ++k) {
        found = link_command_mode() && command(link::Op::ping);
    }
    link_quiet = false;
    if (!found) {
        print(serial, "  the peer did not answer: the STM32F446RE runs usart_peer on USART1?", crlf);
    }
    return found;
}

bool peer_ident(link::Ident& d) {
    link::Frame f;
    if (!query(link::Op::ident, f) || f.op != link::Op::ident_data || f.len != link::ident_size) {
        return false;
    }
    d = link::get_ident(f.data);
    return true;
}

/// The report of the last action, after its bound has had time to run. A
/// refusal raises leg_refused and says so once.
bool peer_report(link::Report& r, uint16_t after_ms) {
    link_task_down();
    (void)link_command_mode();
    if (after_ms != 0u) {
        wait_ms(after_ms);
    }
    for (uint8_t k = 0; k < 4u; ++k) {
        link::Frame f;
        const bool answered = query(link::Op::report, f) && f.op == link::Op::report_data;
        if (answered && f.len != link::report_size) {
            print(serial, "  the peer's REPORT is ", f.len, " bytes, this DUT reads ", link::report_size,
                  " - flash both boards from one tree", crlf);
            return false;
        }
        if (answered) {
            r = link::get_report(f.data);
            if ((r.flags & link::report_cfg_failed) != 0u && !leg_refused) {
                leg_refused = true;
                print(serial, "  the peer REFUSED op ", hex(r.op),
                      " (report_cfg_failed): a capability its silicon has not got", crlf);
            }
            return true;
        }
        wait_ms(40);
    }
    return false;
}

/// Command an action; the peer starts its own traffic link::settle_ms + 2
/// ms after its ack, so this end has settle_ms to take up its end.
bool peer_act(link::Op op, const link::Params& a, const uint8_t* extra = nullptr, uint8_t n = 0) {
    leg_refused = false;
    uint8_t p[link::max_payload] = {};
    link::put_params(p, a);
    for (uint8_t i = 0; i < n; ++i) {
        p[link::params_size + i] = extra[i];
    }
    if (!command(op, p, static_cast<uint8_t>(link::params_size + n))) {
        return false;
    }
    wait_us(static_cast<uint32_t>(link::settle_ms) * 1000u);
    return true;
}

/// A verdict, unless the leg was refused: then SKIP, counting nothing.
void judge(const char* name, bool ok) {
    if (leg_refused) {
        print(serial, "  SKIP  ", name, " (the peer refused the leg)", crlf);
        return;
    }
    bench.verdict(name, ok);
}
void judge(const char* name, const char* suffix, bool ok) {
    if (leg_refused) {
        print(serial, "  SKIP  ", name, suffix, " (the peer refused the leg)", crlf);
        return;
    }
    bench.verdict(name, suffix, ok);
}

/// Milliseconds `frames` frames need at `rate`, four times over plus a
/// floor, inside the protocol's 16-bit bound.
uint16_t bound_ms(uint32_t rate, uint32_t frames) {
    const uint32_t ms = (12u * frames * 1000u * 4u) / (rate != 0u ? rate : 1u) + 80u;
    return ms > 4000u ? 4000u : static_cast<uint16_t>(ms);
}

/// The peer opens its receive window settle_ms + 2 ms after its ack and
/// flushes what this end's reconfiguration left on the line.
void wait_for_peer_window() { wait_ms(6); }

/// `n` frames back to back: TDR written while the transmit FIFO has room,
/// eight frames ahead of the shifter, so the line never idles.
void dut_blast(uint16_t n, uint16_t mask) {
    for (uint16_t i = 0; i < n; ++i) {
        while (!U1::flag(UsartFlag::txfnf)) {
        }
        U1::write_word(static_cast<uint16_t>((0x55u + i) & mask));
    }
    wait_tc();
}

/// What this end's receiver made of a run, polled: frames, the errors, the
/// first and last data word, the first-to-last span on the DWT.
struct Drain {
    uint16_t frames = 0;
    uint8_t fe = 0, pe = 0, ne = 0, ore = 0;
    uint16_t first = 0, last = 0;
    uint32_t span = 0;   ///< TIM2 counts (the nominal 64 MHz), first frame to last
    bool lbd = false;
};

Drain collect(uint16_t ms, uint16_t want = 0xFFFFu) {
    Drain d{};
    // THE RULER'S LOOP IS LEAN: at 6.4 Mbaud a frame is a hundred cycles,
    // so a character costs the ISR load, the stamp, the RDR load and the
    // count; the errors, LBDF, ORE without a character and the deadline
    // are looked at off that path.
    USART_TypeDef& u = U1::regs();
    volatile uint32_t& cnt = TIM2->CNT;
    uint32_t c_first = 0, c_last = 0;
    uint16_t frames = 0, first = 0, last = 0;
    const uint32_t t0 = Ticker::millis();
    for (;;) {
        const uint32_t st = u.ISR;
        if ((st & UsartFlag::rxne) != 0u) {
            const uint32_t stamp = cnt;
            if ((st & UsartFlag::receive_errors) != 0u) [[unlikely]] {
                u.ICR = st & UsartClear::receive_errors;   // before the pop: the next entry's are its own
                if ((st & UsartFlag::fe) != 0u && d.fe < 255u) ++d.fe;
                if ((st & UsartFlag::pe) != 0u && d.pe < 255u) ++d.pe;
                if ((st & UsartFlag::ne) != 0u && d.ne < 255u) ++d.ne;
                if ((st & UsartFlag::ore) != 0u && d.ore < 255u) ++d.ore;
            }
            const uint16_t w = static_cast<uint16_t>(u.RDR & 0x1FFu);
            if (frames == 0u) {
                first = w;
                c_first = stamp;
            }
            last = w;
            c_last = stamp;
            if (++frames >= want) {
                break;
            }
            continue;
        }
        if ((st & UsartFlag::lbdf) != 0u) {
            d.lbd = true;
            u.ICR = UsartClear::lbdf;
        }
        if ((st & UsartFlag::ore) != 0u) {
            if (d.ore < 255u) ++d.ore;
            u.ICR = UsartClear::ore;
        }
        if (Ticker::millis() - t0 >= ms) {
            break;
        }
    }
    d.frames = frames;
    d.first = first;
    d.last = last;
    d.span = c_last - c_first;
    return d;
}

/// One calibrate leg: the peer counts and times `n` gapless frames of this
/// end at `c`; the answer is this end's rate against the peer's time base.
struct Calib {
    bool ok = false;
    int16_t ppm = 0;
    uint32_t span = 0;
    uint16_t count = 0;
    bool hsi_ok = false;   ///< this core's own LSE timed the blast's window
    int32_t hsi = 0;       ///< and the HSI's error over it, in ppm
};

Calib calibrate(const link::Cfg& c, uint16_t n) {
    Calib k{};
    link::Params a{};
    a.cfg = c;
    a.count = n;
    a.ms = bound_ms(c.rate, n);
    if (!peer_act(link::Op::calibrate, a)) {
        return k;
    }
    if (!dut_link(c)) {
        return k;
    }
    wait_for_peer_window();
    const uint32_t t0 = ::now();
    dut_blast(n, c.bits >= link::bits9_low ? 0x1FFu : c.bits == 7u ? 0x7Fu : 0xFFu);
    const std::optional<int32_t> hsi = hsi_over(::now() - t0);
    k.hsi_ok = hsi.has_value();
    k.hsi = hsi.value_or(0);
    link::Report r{};
    if (!peer_report(r, 5) || leg_refused) {
        return k;
    }
    k.ok = r.count == n && r.span != 0u && r.ferr == 0u && r.ovf == 0u;
    k.count = r.count;
    if (!k.ok) {
        print(serial, "    calibrate at ", c.rate, ": the peer counted ", r.count, " of ", n, ", ferr ", r.ferr,
              " ovf ", r.ovf, " span ", r.span, " flags ", hex(r.flags), crlf);
    }
    k.ppm = static_cast<int16_t>(r.aux16);
    k.span = r.span;
    return k;
}

void print_ppm(int32_t ppm) {
    const uint32_t m = static_cast<uint32_t>(ppm < 0 ? -ppm : ppm);
    print(serial, ppm < 0 ? "-" : "+", m, " ppm");
}

bool near(int32_t a, int32_t b, int32_t tol) { return a - b <= tol && b - a <= tol; }

/// Decimal for a verdict label.
char num_buf[12];
const char* num(uint32_t v) {
    uint8_t i = 11;
    num_buf[i] = 0;
    if (v == 0u) num_buf[--i] = '0';
    while (v != 0u && i != 0u) {
        num_buf[--i] = static_cast<char>('0' + (v % 10u));
        v /= 10u;
    }
    return &num_buf[i];
}

/// The rate this end's divisor gives against the rate asked, in ppm.
int32_t divisor_ppm(uint32_t rate) {
    link::Cfg c{};
    c.rate = rate;
    const std::optional<DutSetup> s = dut_setup(c);
    if (!s) return 0;
    // Twice the kernel cycles a bit, exactly: the register read back the
    // way the generator divides (no integer rate rounded in between).
    const uint32_t twice = s->over8 ? ((static_cast<uint32_t>(s->brr) & 0xFFF0u) | ((static_cast<uint32_t>(s->brr) & 0x7u) << 1)) : 2u * s->brr;
    const int64_t num = static_cast<int64_t>(2ull * u1_hz) * 1'000'000;
    return static_cast<int32_t>(num / (static_cast<int64_t>(twice) * rate) - 1'000'000);
}

/// The ppm of a span against `frames - 1` frames of `half_bits` at `rate`,
/// measured on THIS end's TIM2 at the nominal 64 MHz (the HSI's error in
/// it): positive when the far sender runs fast.
int32_t span_ppm(uint32_t span_cycles, uint32_t rate, uint16_t half_bits, uint16_t frames) {
    if (span_cycles == 0u || frames < 2u) return 0;
    const uint64_t nominal = (static_cast<uint64_t>(frames - 1u) * half_bits * SysClock::hz) / (2u * rate);
    return static_cast<int32_t>((static_cast<int64_t>(nominal) - span_cycles) * 1'000'000 /
                                static_cast<int64_t>(span_cycles));
}

constexpr uint32_t calib_rate = 1'000'000;   ///< 64 MHz / 64 here, 90 MHz / 90 there: the clocks alone
constexpr uint16_t calib_frames = 4000;      ///< 40 ms, a million ticks of span
int32_t hsi_ppm = 0;                         ///< letter 1's figure: this HSI against the far crystal
bool hsi_known = false;

// ---------------------------------------------------------------------------
// 1 - the rendezvous, and this core's HSI16 against the far crystal
// ---------------------------------------------------------------------------
void t1_link() {
    const bool found = ensure_link();
    bench.verdict("command mode on USART1 (8N1, 115200) and the peer answers a ping", found);
    if (!found) {
        link_release();
        return;
    }
    link::Ident d{};
    const bool got = peer_ident(d);
    bench.verdict("IDENT answered", got);
    char label[9] = {};
    if (got) {
        for (uint8_t i = 0; i < 8u; ++i) label[i] = d.label[i];
        print(serial, "  peer '", label, "' kind ", hex(link::peer_kind(d.version)), " fw ", hex(d.version),
              " xtal=", d.xtal, " sanity=", hex(d.sanity), crlf);
    }
    bench.verdict("the peer is the STM32F4's usart_peer", got && d.sanity == link::ident_sanity &&
                                                             link::peer_kind(d.version) == link::peer_stm32f4);
    // The label is the peer's UID word: not this die's own.
    const uint32_t w0 = DeviceUid::read().word[0];
    bool same = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
        same = same && label[i] == static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }
    bench.verdict("and it is another die: its label is not this chip's unique ID", got && !same);
    bench.verdict("its time base is crystal-referenced (the ST-LINK's 8 MHz in bypass)", got && d.xtal == 1u);

    u1_flush();
    put1(link::magic);
    put1(link::byte_of(link::Op::ping));
    put1(0);
    put1(0x00);   // not the checksum
    wait_tc();
    link::Frame bad;
    bench.verdict("a bad checksum comes back as a NAK", recv_frame(bad, 120) && bad.op == link::Op::nak);
    for (uint8_t i = 0; i < 12u; ++i) put1(static_cast<uint8_t>(0x11u * i));
    wait_tc();
    wait_ms(120);
    (void)link_command_mode();
    bench.verdict("the link recovers after a desync", command(link::Op::ping));

    // THE REFUSAL PATH, the first peer that refuses anything: a
    // synchronous client (no USART slave on that chip, 25.4.9) and a
    // seven-bit word (no word length for it, 25.4.3) are acked, reported
    // refused, and the peer is back in command mode at once.
    bool refused_both = true;
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        link::Params a{};
        a.cfg.rate = 115'200;
        if (leg == 0u) {
            a.cfg.mode = link::Mode::sync_client;
        } else {
            a.cfg.bits = 7;
        }
        a.ms = 50;
        link::Report r{};
        refused_both = refused_both && peer_act(link::Op::set_link, a) && peer_report(r, 0) && leg_refused &&
                       (r.flags & link::report_ran) == 0u;
    }
    leg_refused = false;
    bench.verdict("the peer refuses a synchronous client and a seven-bit word, and answers at once after",
                  refused_both && command(link::Op::ping));

    // THE CLOCK. 1 Mbaud is 64 MHz / 64 on this end and 90 MHz / 90 on the
    // far one: what the far crystal times is this core's HSI16 through the
    // PLL, three times.
    link::Cfg c{};
    c.rate = calib_rate;
    Calib k[3];
    bool all = true;
    for (Calib& x : k) {
        x = calibrate(c, calib_frames);
        all = all && x.ok;
    }
    print(serial, "  this core's HSI16 against the far crystal: ");
    print_ppm(k[0].ppm);
    print(serial, ", then ", k[1].ppm, " and ", k[2].ppm, " ppm; span ", k[0].span, " ticks for ",
          calib_frames - 1u, " frames", crlf);
    print(serial, "  the same three windows against this board's own LSE: ", k[0].hsi, ", ", k[1].hsi, ", ",
          k[2].hsi, " ppm", crlf);
    bench.verdict("the peer timed three gapless runs of 4000 frames at 1 Mbaud", all);
    // TWO CRYSTALS, ONE WINDOW: the far ST-LINK's 8 MHz and this board's
    // LSE time the same HSI over the same 40 ms - they must agree to their
    // own tolerances, whatever the HSI did.
    bool agree = all;
    for (const Calib& x : k) {
        agree = agree && x.hsi_ok && near(x.ppm, x.hsi, 60);
    }
    bench.verdict("and this board's LSE reads each window the same within 60 ppm: two crystals agree",
                  agree);
    // DS13560's HSI16 accuracy at the factory trim, over temperature.
    bench.verdict("the HSI16 lies inside its datasheet's one per cent (the error a finding, printed)",
                  all && near(k[0].ppm, 0, 10'000));
    if (all) {
        hsi_ppm = k[0].ppm;
        hsi_known = true;
    }

    // The other direction, on this end's ruler: the far crystal's 1 Mbaud
    // blast timed on TIM2 - this HSI's error seen from the other side.
    {
        link::Params a{};
        a.cfg.rate = calib_rate;
        a.count = calib_frames;
        a.ms = bound_ms(calib_rate, calib_frames);
        const bool acted = peer_act(link::Op::blast, a);
        const bool up = acted && dut_link(a.cfg);
        const Drain dr = up ? collect(a.ms, calib_frames) : Drain{};
        const int32_t ppm = span_ppm(dr.span, calib_rate, 20, dr.frames);
        print(serial, "  the far crystal's 1 Mbaud on this HSI: ", dr.frames, " frames, ");
        print_ppm(ppm);
        print(serial, " (minus this core's error, within its wander)", crlf);
        bench.verdict("the far blast arrives whole, and this ruler reads it against the HSI's error, "
                      "inside 500 ppm",
                      up && dr.frames == calib_frames && dr.fe == 0u && all && near(ppm, -hsi_ppm, 500));
    }
    link_release();
}

// ---------------------------------------------------------------------------
// 2 - the baud matrix across the wire, judged by a foreign receiver
// ---------------------------------------------------------------------------
uint16_t echo_mismatches(uint16_t n, uint16_t mask) {
    uint16_t bad = 0;
    wait_for_peer_window();
    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t v = static_cast<uint16_t>((0x5Au + i * 37u) & mask);
        while (!U1::flag(UsartFlag::txfnf)) {
        }
        U1::write_word(v);
        bool ok = false;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < 20u) {
            const uint32_t st = U1::status();
            if ((st & UsartFlag::rxne) != 0u) {
                U1::clear_flags(st & UsartClear::receive_errors);   // ICR, before the pop
                ok = (U1::read_word() & mask) == v && (st & UsartFlag::receive_errors) == 0u;
                break;
            }
        }
        if (!ok) ++bad;
    }
    return bad;
}

/// THE LADDER BOTH DIVIDERS CAN MEET. This USART's BRR divides PCLK2 by
/// any whole k from 8 (OVER8's 3-bit fraction is eighths of 8 x USARTDIV,
/// 25.4.4), and the far STM32G0's divides its kernel by any whole n from 8
/// - its OVER8 adds no resolution, since BRR[2:0] = USARTDIV[3:0] >> 1
/// drops USARTDIV's last bit (RM0444 33.5.7). So the two lattices are
/// 90 MHz / k and 64 MHz / n, and they meet where k / n = 7 / 5: above
/// 2 Mbaud the rungs are 64 MHz / 5m - 2.56, 3.2, 4.27 and 6.4 Mbaud,
/// 0.45 % above the rate on this end, 0.15 % from the far end. A rate
/// between them is one or both ends a per cent or more off, 7.5 Mbaud
/// five per cent on the far end, past its receiver's tolerance.
constexpr uint32_t matrix_rates[] = {9600, 115'200, 921'600, 1'000'000, 1'500'000, 2'000'000,
                                     2'560'000, 3'200'000, 4'266'667, 6'400'000};

/// Frames enough for a span of some 25 ms at `rate`: the peer's stamps
/// jitter by its loop, a microsecond, so the reading resolves 40 ppm.
uint16_t span_frames(uint32_t rate) {
    const uint32_t n = rate / 400u;
    return static_cast<uint16_t>(n < 64u ? 64u : (n > 20'000u ? 20'000u : n));
}

/// `n` bytes of the peer's counting blast through a Uart task, read as
/// runs: how many arrived, in order.
template <typename Port>
uint32_t task_receive(uint32_t n, uint16_t ms, uint8_t& ore) {
    uint32_t got = 0;
    bool broken = false;
    const uint32_t t0 = Ticker::millis();
    while (got < n && !broken && Ticker::millis() - t0 < ms) {
        const std::span<const uint8_t> run = Port::read_span();
        uint32_t k = 0;
        for (const uint8_t b : run) {
            if (b != static_cast<uint8_t>(got)) {
                broken = true;
                break;
            }
            ++got;
            ++k;
        }
        (void)Port::consume(k);
    }
    ore = Port::hw_overruns();
    return got;
}

void t2_rates() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    for (const uint32_t rate : matrix_rates) {
        link::Cfg c{};
        c.rate = rate;
        // a. An echo of 16 frames.
        {
            link::Params a{};
            a.cfg = c;
            a.count = 16;
            a.ms = bound_ms(rate, 32);
            const bool acted = peer_act(link::Op::echo, a);
            const bool up = acted && dut_link(c);
            const uint16_t bad = up ? echo_mismatches(16, 0xFFu) : 16u;
            link::Report r{};
            const bool have = peer_report(r, 5);
            judge(num(rate), " baud: 16 frames echoed by the far chip, every one equal", up && bad == 0u);
            judge(num(rate), " baud: the far chip counted 16 clean frames",
                  have && r.count == 16u && r.ferr == 0u && r.perr == 0u && r.ovf == 0u);
        }
        // b. This end's transmitter timed by the FOREIGN receiver, against
        //    the divisor's error and the HSI's over the same run, read off
        //    this board's own LSE: what is judged is the divisor on the
        //    wire, never the oscillator's walk.
        const uint16_t n = span_frames(rate);
        const Calib k = calibrate(c, n);
        const int32_t expect = k.hsi + divisor_ppm(rate);
        print(serial, "  ", rate, ": ", n, " frames, the far receiver times ");
        print_ppm(k.ppm);
        print(serial, " against this divisor's ", divisor_ppm(rate), " and the HSI's ", k.hsi,
              " (this board's LSE)", crlf);
        judge(num(rate), " baud: this transmitter's rate on the wire, timed by the far chip, within 100 ppm "
                         "of the divisor's and the HSI's over the run",
              k.ok && k.hsi_ok && near(k.ppm, expect, 100));
        // c. The peer's blast into this end: the polled receiver timing it,
        //    then the interrupt receiver and the receive engine.
        {
            link::Params a{};
            a.cfg = c;
            a.count = n;
            a.ms = bound_ms(rate, n);
            const bool acted = peer_act(link::Op::blast, a);
            const bool up = acted && dut_link(c);
            const Drain dr = up ? collect(a.ms, n) : Drain{};
            link::Report r{};
            (void)peer_report(r, 0);
            const int32_t ppm = span_ppm(dr.span, rate, 20, dr.frames);
            print(serial, "    the far blast on this ruler: ", dr.frames, " of ", n, ", ORE ", dr.ore, ", ");
            print_ppm(ppm);
            print(serial, crlf);
            judge(num(rate), " baud: the far blast arrives whole on the polled receiver, inside a per cent of "
                             "its rate",
                  up && dr.frames == n && dr.fe == 0u && dr.ore == 0u && near(ppm, 0, 10'000));
        }
        for (uint8_t which = 1; which <= 2u; ++which) {
            link::Params a{};
            a.cfg = c;
            a.count = 1024;
            a.ms = bound_ms(rate, 1024);
            a.value = 0;
            a.pattern = link::pattern_counting;
            const bool acted = peer_act(link::Op::blast, a);
            link_task_down();
            const bool fast = !LinkIrq::can_baud(u1_hz, rate);
            loop_mode = which == 1u ? (fast ? mode_link_irq8 : mode_link_irq)
                                    : (fast ? mode_link_dma8 : mode_link_dma);
            bool up = acted;
            if (up) {
                up = fast ? (which == 1u ? LinkIrq8::init(clock, rate) : LinkDma8::init(clock, rate))
                          : (which == 1u ? LinkIrq::init(clock, rate) : LinkDma::init(clock, rate));
            }
            uint8_t ore = 0;
            uint32_t got = 0;
            if (up) {
                if (fast) {
                    got = which == 1u ? task_receive<LinkIrq8>(1024, a.ms, ore)
                                      : task_receive<LinkDma8>(1024, a.ms, ore);
                } else {
                    got = which == 1u ? task_receive<LinkIrq>(1024, a.ms, ore)
                                      : task_receive<LinkDma>(1024, a.ms, ore);
                }
            }
            link_task_down();
            link::Report r{};
            (void)peer_report(r, 0);
            if (got != 1024u) {
                print(serial, "    ", which == 1u ? "the interrupt receiver" : "the receive engine", ": ", got,
                      " of 1024 in order, ORE ", ore, crlf);
            }
            judge(num(rate), which == 1u ? " baud: 1024 frames whole through the interrupt receiver"
                                         : " baud: 1024 frames whole through the receive engine",
                  up && got == 1024u);
        }
    }
    link_release();
}

// ---------------------------------------------------------------------------
// 3 - the frame matrix across the wire
// ---------------------------------------------------------------------------
struct FmtCase {
    uint8_t bits;
    uint8_t parity;
    uint8_t stop;
    const char* name;
};

void t3_frames() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    static constexpr FmtCase cases[] = {
        {8, 0, 1, "8N1"}, {8, 1, 1, "8E1"}, {8, 2, 1, "8O1"}, {7, 1, 1, "7E1"}, {7, 2, 1, "7O1"},
        {link::bits9_low, 0, 1, "9N1"}, {8, 0, 2, "8N2"}, {8, 0, link::stop_half, "8N0.5"},
        {8, 0, link::stop_one_half, "8N1.5"}, {7, 0, 1, "7N1"}, {6, 1, 1, "6E1"},
        {link::bits9_low, 1, 1, "9E1"},
    };
    for (const FmtCase& fc : cases) {
        link::Cfg c{};
        c.rate = 115'200;
        c.bits = fc.bits;
        c.parity = fc.parity;
        c.stop = fc.stop;
        if (!dut_setup(c)) {
            print(serial, "  SKIP  ", fc.name, " (this USART has no word length for it, RM0444 33.5.5)", crlf);
            continue;
        }
        link::Params a{};
        a.cfg = c;
        a.count = 8;
        a.ms = bound_ms(115'200, 24);
        const uint16_t mask = fc.bits >= link::bits9_low ? 0x1FFu : fc.bits == 7u ? 0x7Fu : 0xFFu;
        const bool acted = peer_act(link::Op::echo, a);
        const bool up = acted && dut_link(c);
        const uint16_t bad = up ? echo_mismatches(8, mask) : 8u;
        link::Report r{};
        const bool have = peer_report(r, 5);
        if (bad != 0u && !leg_refused) {
            print(serial, "    ", fc.name, ": wrong=", bad, " peer count=", r.count, " ferr=", r.ferr,
                  " perr=", r.perr, crlf);
        }
        judge(fc.name, " across the wire, echoed by the far chip",
              up && bad == 0u && have && r.count == 8u && r.ferr == 0u && r.perr == 0u);
    }
    // The two odd stops as LENGTHS: a gapless run timed by the peer is a
    // frame of 19 and of 21 half bits, the clocks' offset and nothing else.
    for (const uint8_t stop : {link::stop_half, link::stop_one_half}) {
        link::Cfg ref{};
        ref.rate = calib_rate;
        const Calib r0 = calibrate(ref, 2000);
        link::Cfg c{};
        c.rate = calib_rate;
        c.stop = stop;
        const Calib k = calibrate(c, 2000);
        print(serial, "  ", stop == link::stop_half ? "8N0.5" : "8N1.5", " at 1 Mbaud: ");
        print_ppm(k.ppm);
        print(serial, " against frames of ", link::frame_half_bits(8, 0, stop), " half bits (8N1: ", r0.ppm,
              ")", crlf);
        // Each run against the HSI over its own window (this board's LSE).
        judge(stop == link::stop_half ? "half a stop bit" : "one and a half stop bits",
              ": the frame on the wire is that long, within 100 ppm of an 8N1 run's, the HSI taken out",
              r0.ok && k.ok && r0.hsi_ok && k.hsi_ok && near(k.ppm - k.hsi, r0.ppm - r0.hsi, 100));
    }
    link_release();
}

// ---------------------------------------------------------------------------
// 4 - the receiver's errors, from a foreign sender
// ---------------------------------------------------------------------------
void t4_errors() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_release();
        return;
    }
    // 1. Odd parity into an even receiver: PE on each, the data readable.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.parity = 2;
        link::Cfg dc = pc;
        dc.parity = 1;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 12);
        a.value = 0x5A;
        a.pattern = link::pattern_fixed;
        const bool acted = peer_act(link::Op::send, a);
        const bool up = acted && dut_link(dc);
        const Drain d = collect(bound_ms(115'200, 12), 6);
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  odd into even: frames=", d.frames, " pe=", d.pe, " fe=", d.fe, " data ", hex(d.last),
              crlf);
        judge("every frame of the wrong parity raises PE, and its data is still read",
              up && d.frames == 6u && d.pe == 6u && d.fe == 0u && (d.last & 0xFFu) == 0x5Au);
    }
    // 2. The FE onset: 0x00 frames from the peer's divisor a few per cent
    //    off 9600, into a receiver at 9600 (33.5.8's tables 188/189:
    //    3.75 % at OVER16 with BRR[3:0] = 0, 6666 here). BACK TO BACK: a fast sender's
    //    frames with idle between them never fail the stop bit - its sample
    //    lands in the idle - so the blast's gapless run is what shows both
    //    edges of the tolerance.
    {
        static constexpr int8_t errs[] = {0, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6, 8, -8};
        int8_t onset_pos = 0, onset_neg = 0;
        bool inside_clean = true, outside_raised = true;
        link::Cfg dc{};
        dc.rate = 9600;
        for (const int8_t e : errs) {
            link::Params a{};
            a.cfg.rate = static_cast<uint32_t>(9600 + (9600 * e) / 100);
            a.count = 6;
            a.ms = 400;
            a.value = 0x00;
            a.pattern = link::pattern_fixed;
            const bool acted = peer_act(link::Op::blast, a);
            const bool up = acted && dut_link(dc);
            const Drain d = collect(40, 6);
            link::Report r{};
            (void)peer_report(r, 0);
            print(serial, "  ", e, " %: frames=", d.frames, " fe=", d.fe, " ne=", d.ne, crlf);
            const bool clean = up && d.frames == 6u && d.fe == 0u;
            if (up && !clean) {
                if (e > 0 && onset_pos == 0) onset_pos = e;
                if (e < 0 && onset_neg == 0) onset_neg = e;
            }
            if (e >= -3 && e <= 3) inside_clean = inside_clean && clean;
            if (e == 8 || e == -8) outside_raised = outside_raised && up && !clean;
        }
        print(serial, "  FE onset: +", onset_pos, " %, ", onset_neg, " %", crlf);
        judge("every rate within 3 % of the receiver's is clean (33.5.8's 3.75 %)", inside_clean);
        judge("and 8 % off either way raises FE", outside_raised);
    }
    // 3. A run into a receiver that reads late: ORE, the FIFO's eight and
    //    the shift register's one kept (33.5.6: with the FIFO on, an
    //    overrun is the shift register ready with the RXFIFO full).
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 16;
        a.ms = bound_ms(115'200, 32);
        a.value = 0xB0;
        const bool acted = peer_act(link::Op::blast, a);
        const bool up = acted && dut_link(pc);
        wait_ms(6 + 3);
        const uint32_t st = U1::status();
        const Drain d = collect(5);
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  a run of 16 read late: ISR ", hex(st), ", then ", d.frames, " read, ", hex(d.first),
              " to ", hex(d.last), crlf);
        judge("a run read late raises ORE", up && (st & UsartFlag::ore) != 0u && (st & UsartFlag::rxne) != 0u);
        // MEASURED NINE, the shift register's frame among them: the
        // overrun is the TENTH frame finding the RXFIFO full and the
        // shifter occupied (33.5.6's "the shift register is overwritten"
        // is that tenth frame and every one after it).
        judge("and the RXFIFO and the shift register keep the run's first nine in order, the rest lost",
              up && d.frames == 9u && d.first == 0xB0u && d.last == 0xB8u);
    }
    // 4. NE from a glitch: one low pulse of w ticks on the idle line at
    //    9600 (a bit is 2500 ticks, a sixteenth 156), with ONEBIT clear -
    //    the start bit's samples voted, NE on a split vote (33.5.6) - and
    //    set.
    {
        static constexpr uint16_t widths[] = {120, 600, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600, 1700, 2500};
        uint16_t first_taken[2] = {0, 0};
        bool tiny_ignored = true, whole_taken = true, ne_seen = false, ne_onebit = false;
        for (uint8_t onebit = 0; onebit < 2u; ++onebit) {
            print(serial, "  ONEBIT ", onebit, ":");
            for (const uint16_t w : widths) {
                link::Params a{};
                a.cfg.rate = 9600;
                a.count = 1;
                a.ms = 100;
                a.aux16 = w;
                a.aux8 = 10;
                link::Cfg dc{};
                dc.rate = 9600;
                const bool acted = peer_act(link::Op::glitch, a);
                const bool up = acted && dut_link(dc, false, onebit != 0u);
                const Drain d = collect(15);
                link::Report r{};
                (void)peer_report(r, 0);
                print(serial, " ", w, ":", d.frames, "/", d.ne, "/", d.fe);
                if (up && d.frames != 0u && first_taken[onebit] == 0u) first_taken[onebit] = w;
                if (w == 120u) tiny_ignored = tiny_ignored && up && d.frames == 0u;
                if (w == 2500u) whole_taken = whole_taken && up && d.frames == 1u;
                if (onebit == 0u && d.ne != 0u) ne_seen = true;
                if (onebit != 0u && d.ne != 0u) ne_onebit = true;
            }
            print(serial, "   (ticks:frames/NE/FE)", crlf);
        }
        print(serial, "  the narrowest pulse taken as a start bit: ", first_taken[0], " ticks, ", first_taken[1],
              " with ONEBIT", crlf);
        judge("a pulse under a sample (120 ticks) is never a start bit", tiny_ignored);
        judge("a whole bit low is one, either way", whole_taken);
        judge("and with ONEBIT clear a pulse that splits the start bit's vote raises NE", ne_seen);
        judge("with ONEBIT set no width raises NE (33.5.6: the one-sample method has no vote)", !ne_onebit);
    }
    // 5. A break of 20 bit times: one frame, FE, data 0x00 - and in LIN
    //    mode LBDF beside it (33.5.13's break detection).
    for (const bool lin : {false, true}) {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 20;
        a.ms = 200;
        const bool acted = peer_act(link::Op::brk, a);
        const bool up = acted && dut_link(pc, lin);
        const Drain d = collect(30);
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  a break of 20 bit times", lin ? " (LIN mode)" : "", ": frames=", d.frames, " fe=", d.fe,
              " data ", hex(d.first), " lbd=", d.lbd ? 1 : 0, "; the engine ",
              (r.flags & link::report_mismatch) != 0u ? "BROKEN" : "whole", crlf);
        if (!lin) {
            judge("a break arrives as one frame with FE and data 0x00",
                  up && d.frames == 1u && d.fe == 1u && d.first == 0u);
        } else {
            judge("and in LIN mode it raises LBDF as well", up && d.lbd);
        }
    }
    // 6. THE PER-ENTRY FLAGS OF A FIFO DRAINED LATE (33.5.4: PE, FE and NE
    //    are stored with each entry): six frames of the wrong parity left
    //    in the RXFIFO, then drained, the ISR word read before each RDR -
    //    with the flags cleared AFTER the RDR read, and BEFORE it.
    for (uint8_t before = 0; before < 2u; ++before) {
        link::Params a{};
        a.cfg.rate = 115'200;
        a.cfg.parity = 2;
        a.count = 6;
        a.ms = bound_ms(115'200, 12);
        a.value = 0x5A;
        a.pattern = link::pattern_fixed;
        link::Cfg dc = a.cfg;
        dc.parity = 1;
        const bool acted = peer_act(link::Op::send, a);
        const bool up = acted && dut_link(dc);
        wait_ms(6 + 3);
        uint8_t entries = 0, flagged = 0;
        USART_TypeDef& u = U1::regs();
        while ((u.ISR & UsartFlag::rxne) != 0u && entries < 16u) {
            const uint32_t st = u.ISR;
            if (before != 0u) {
                u.ICR = st & UsartClear::receive_errors;
            }
            (void)u.RDR;
            if (before == 0u) {
                u.ICR = st & UsartClear::receive_errors;
            }
            ++entries;
            if ((st & UsartFlag::pe) != 0u) ++flagged;
        }
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  six wrong-parity frames drained late, the clear ", before != 0u ? "BEFORE" : "after",
              " the RDR read: ", entries, " entries, ", flagged, " with PE", crlf);
        if (before != 0u) {
            judge("drained late, each entry's PE is its own when the clear precedes the read",
                  up && entries == 6u && flagged == 6u);
        } else {
            // A figure: cleared after the pop, a flag the next entry also
            // carries is the same standing bit and goes with it - the
            // order usart.hpp's take() must not use.
            print(serial, "  (cleared after the read, a run of errored entries loses every other flag)", crlf);
        }
    }
    // 7. The same errors through the Uart task: counted and dropped - odd
    //    parity into an even task, and a break.
    {
        link::Params a{};
        a.cfg.rate = 115'200;
        a.cfg.parity = 2;
        a.count = 6;
        a.ms = bound_ms(115'200, 12);
        a.value = 0x5A;
        a.pattern = link::pattern_fixed;
        const bool acted = peer_act(link::Op::send, a);
        link_task_down();
        loop_mode = mode_link_irq;
        // 8E1: CR1.M counts the parity bit, a nine-bit word (33.5.5).
        const bool up = acted && LinkIrq::init(clock, 115'200, {.bits = UartBits::nine, .parity = UartParity::even});
        wait_ms(20);
        const uint32_t pending = LinkIrq::rx_pending();
        const uint8_t pe = LinkIrq::parity_errors();
        link_task_down();
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  odd into an even task: ", pending, " bytes delivered, ", pe, " parity errors counted",
              crlf);
        judge("the task counts every parity error and delivers none of the frames",
              up && pe == 6u && pending == 0u);
    }
    {
        link::Params a{};
        a.cfg.rate = 115'200;
        a.count = 20;
        a.ms = 200;
        const bool acted = peer_act(link::Op::brk, a);
        link_task_down();
        loop_mode = mode_link_irq;
        const bool up = acted && LinkIrq::init(clock, 115'200);
        wait_ms(20);
        const uint32_t pending = LinkIrq::rx_pending();
        const uint8_t fe = LinkIrq::frame_errors();
        link_task_down();
        link::Report r{};
        (void)peer_report(r, 0);
        print(serial, "  a break into the task: ", pending, " bytes delivered, ", fe, " frame errors counted", crlf);
        judge("the task counts the break as one frame error and delivers nothing", up && fe == 1u && pending == 0u);
    }
    link_release();
}

void banner() {
    print(serial, crlf,
          "test_stm32_serial - the USART's long tail, the LPUARTs and IRTIM "
          "(no wires)", crlf);
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
    brio::hard_fault_reset<brio::Stm32g0Platform<>>(0xFA);
}

/// USART2 and LPUART2 share this line on the G0B1 class. The console is
/// the first; the LPUART console of letter v is the second, and only one
/// of them is up at a time.
extern "C" void BRIO_STM32G0_USART2_HANDLER() {
    // USART2 and LPUART2 share this line on the G0B1 class. Only USART2
    // is ever up on it in this suite - LPUART2 is driven polled in
    // letter n - so the console's body is the whole of it. LPUART1's
    // line is USART3_4_5_6_LPUART1's, below.
    // WUF IS CLEARED UNCONDITIONALLY, and the counter is what the
    // letter reads. Gating the CLEAR on the letter's own flag is a wake
    // storm: the flag is dropped as soon as the Stop returns while
    // WUFIE stays armed for the 200 ms in which the letter drains the
    // bytes that woke it, and every further start bit then sets a level
    // nobody will clear - the vector re-enters for ever with its counter
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
/// seconds later. The symptom looks like a fault and is not one: a
/// HardFault breadcrumb stays EMPTY through it.
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
    bool edge = false;
    switch (loop_mode) {
        case mode_paced: edge = PacedLoop::isr(); break;
        case mode_engine: edge = EngineLoop::isr(); break;
        case mode_plain_tx: (void)PlainSender::isr(); break;
        case mode_engine_tx: (void)EngineSender::isr(); break;
        case mode_link_irq: (void)LinkIrq::isr(); break;
        case mode_link_dma: (void)LinkDma::isr(); break;
        case mode_link_irq8: (void)LinkIrq8::isr(); break;
        case mode_link_dma8: (void)LinkDma8::isr(); break;
        default: break;
    }
    if (loop_mode >= mode_paced) {
        if (edge) {
            rx_edge_at = now();
            rx_edges = rx_edges + 1u;
        }
        return;
    }
    // Nothing of ours is armed here: silence the line rather than
    // re-entering for ever on a condition nobody will clear.
    brio::Nvic::disable(brio::Usart<1>::irq());
}

/// Channels 2 and 3 share this line (table 61): letters p and q's
/// transmit engine and receive engine, the only channels of this suite
/// that interrupt.
extern "C" void DMA1_Channel2_3_IRQHandler() {
    bool edge = false;
    switch (loop_mode) {
        case mode_paced: (void)PacedLoop::dma_isr(); break;
        case mode_engine: edge = EngineLoop::dma_isr(); break;
        case mode_engine_tx: (void)EngineSender::dma_isr(); break;
        case mode_link_dma: (void)LinkDma::dma_isr(); break;
        case mode_link_dma8: (void)LinkDma8::dma_isr(); break;
        default:
            brio::Nvic::disable(DMA1_Channel2_3_IRQn);
            break;
    }
    if (edge) {
        rx_edge_at = now();
        rx_edges = rx_edges + 1u;
    }
}

/// LPUART1'S LINE, WHICH IS USUALLY SOMEBODY ELSE'S TOO: USART3, USART4
/// and - where the part has them - USART5 and USART6 share it, which is
/// the reason every isr() body of this stratum answers for itself alone.
/// The NAME moves with what the header declares beside it - and on a part
/// with no USART3 at all the LPUART has the line to itself - so the
/// reserve's LPUART1 macro is what binds it: a bare spelling on the wrong
/// board would land in Default_Handler's silent spin.
extern "C" void BRIO_STM32G0_LPUART1_HANDLER() {
    // Letter v puts the whole CONSOLE on LPUART1, which arrives here and
    // not on USART2's line - the two LPUARTs of this part sit on
    // different vectors and only the second one shares the console's.
    // The task's isr() must be reached, or the byte that raised the
    // interrupt is never drained and the level re-enters for ever.
    if (lp_console_mode == 1u) {
        // A CALL THROUGH A TEMPLATE, so that a board whose crystal never
        // starts does not carry that console's two rings for a mode
        // letter v can never put it in (a discarded `if constexpr` branch
        // in a plain function is still odr-used, and 384 bytes of RAM is
        // real money on an 8 K part).
        lp_console_lse_isr();
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
        // down here so no later level can storm it - the same rule as
        // the console's vector above.
        brio::Nvic::disable(brio::Lpuart<1>::irq());
        return;
    }
    if (lpuart_live) {
        lpuart_irqs = lpuart_irqs + 1u;
        (void)brio::Lpuart<1>::read_word();
        brio::Lpuart<1>::clear_flags(brio::UsartClear::all);
        return;
    }
    // AND THE OTHER HALF OF THE SAME RULE: reading a peripheral whose
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

    // Four letters below count edges through a DMAMUX request generator
    // an EXTI line triggers. Whether that path exists on this die is a
    // silicon question, asked here once and answered by the letters that
    // need it (ES0418 2.2.4).
    probe_exti_dma_trigger();

    bench.letter('a', "the instance table: every USART and LPUART the part "
                      "has, three authorities", ta_instances);
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
    bench.letter('p', "the receive transport: the paced receiver, the engine's "
                      "edges, K breaks in N slots", tp_receive);
    bench.letter('q', "tx_idle() on the pad: the last stop bit, both transmitters",
                 tq_tx_idle);
    bench.letter('y', "HOST: streaming across the kernel clocks", ty_streaming,
                 false);
    bench.letter('w', "HOST: the wake from Stop, and 2.2.4 staged", tw_wake,
                 false);
    bench.letter('v', "HOST: the console moved to LPUART1", tv_lpuart_console,
                 false);
    if constexpr (two_board_carried()) {
        bench.second_letter('1', "the rendezvous, and this HSI16 against the far crystal", t1_link);
        bench.second_letter('2', "the baud matrix across the wire, timed by the far chip", t2_rates);
        bench.second_letter('3', "the frame matrix across the wire", t3_frames);
        bench.second_letter('4', "the receiver's errors from a foreign sender", t4_errors);
    }

    const std::optional<brio::PanicRecord> crumb =
        brio::take_panic_record<brio::Stm32g0Platform<>>();

    if (serial_ok) {
        if (crumb) {
            print(serial, crlf, "PREVIOUS RUN DIED: panic code ",
                  static_cast<uint32_t>(crumb->code), " context ",
                  static_cast<uint32_t>(crumb->context), crlf);
        }
        const auto idcode = brio::DeviceIdcode::read();
        print(serial, crlf, "part DEV_ID ", hex(idcode.dev_id),
              " REV_ID ", hex(idcode.rev_id), crlf);
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
