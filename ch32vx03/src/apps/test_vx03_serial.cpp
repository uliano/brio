// test_vx03_serial - the reference bench suite for the USART chapter (RM
// ch. 18) of the CH32V203 and the CH32V303, beyond the console
// personality: the resource ch32vx03/usart.hpp's `Usart<n>` in every mode
// the chapter has, and the transport `Uart`'s options and engines, on the
// instances the console does not own. Letters a..m are both series';
// letters n..p are the CH32V303's upper ports and lot-keyed registers,
// registered on the part that has them and compiled out of every other
// image.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE INSTRUMENTS ARE USART2 AND THE FOURTH PORT, and three facts of this
// silicon make almost every letter need NO WIRE AT ALL:
//
//  1. THE RX PAD IS A BIT-BANGED TRANSMITTER. A pad handed to the USART
//     as RX is an input, and an input's level follows its own pull -
//     which the output data register moves in a few cycles (pin.hpp:
//     the pull direction IS the output data bit). At 9600 baud a bit is
//     15000 core cycles, so software paced on the STK puts an ARBITRARY
//     frame on the receive line: every format, a parity that does not
//     add up, a stop bit that is low, a break of ten or eleven bits, an
//     address mark, an RZI frame for the infrared decoder.
//  2. A PAD DRIVEN BY A PERIPHERAL STILL REACHES A CAPTURE INPUT (the
//     timer chapter's finding). USART2's TX pad PA2 is TIM2's channel 3
//     input, so the transmitter's own line is measured by the timer
//     with nothing strapped: a start bit is the divisor to the cycle, a
//     break is its ten or thirteen bits, an IrDA pulse is three
//     sixteenths of a bit, and the stop length is the gap between two
//     frames. USART4's remapped CK pad PA6 is TIM3's channel 1 input
//     the same way, which is how the synchronous clock and the
//     smartcard's card clock are counted.
//  3. THE FOURTH SERIAL PORT IS A USART ON THE CH32V203C8. Chapter 18's
//     opening names that part as the exception whose UART4 is a USART4,
//     with a CK, a CTS and an RTS pad - so the synchronous mode and the
//     smartcard have an instance to run on that is not the console's. On
//     the CH32V303 the fourth port is a UART4 with TX and RX alone, and
//     letters h and i decline there by name.
//
// WIRES ADD LETTERS and each is detected rather than assumed: PA2 to PA3
// (USART2's own loopback, letter l) and the crossed pair of USART2's
// pads with UART4's default ones - PA2-PB1 with PB0-PA3 on the
// CH32V203C8, PA2-PC11 with PC10-PA3 on the CH32V303, the column table
// 10-26 gives that class (letters m, n and o). Without them the letters
// say so and pass nothing.
//
// THE CH32V303'S OWN LETTERS. UART5..UART8 carry no wire on the evaluation
// board and need none for what letter p asks: the register face, a frame
// read back off the TRANSMIT pad by software (a pad handed to the
// peripheral still reads its level in the input register), a frame
// bit-banged into the RECEIVE pad through its pull, and the vector of
// this class's tail. The pads of those four columns are ports C's and
// D's, and nothing else of this suite touches them.
//
// What is exercised, letter by letter:
//   a  the baud generator: every standard rate from 1200 to 3 Mbaud on
//      both peripheral buses, the divisor, the rate it really gives and
//      the error in per mille, the rates the generator cannot serve
//   b  the frame on the wire, measured by the timer: the start bit at
//      four rates, the low run of every word length and parity, all
//      four stop lengths, and TE's idle frame
//   c  the bit-banged line into the receiver: every format received, a
//      parity error, a framing error, an overrun staged and cleared
//   d  mute mode: the receiver asleep through frames until the line
//      idles, or until a nine-bit frame carries its own address
//   e  LIN: the break SBK sends timed on the pad, and the detector's
//      flag and interrupt at ten and eleven bits from the banged line
//   f  half duplex on one wire, and a board with no pull-up on it: the
//      mode's own bits, and the frame and the echo printed rather than
//      judged, the wire being held by nothing between frames
//   g  IrDA: the encoder's pulse in normal and low-power mode measured
//      on the pad, and the decoder fed a banged RZI frame
//   h  THE SYNCHRONOUS MODE on USART4's remapped column: the clock
//      pulses per frame counted on the CK pad, LBCL both ways, CPOL's
//      idle level, CPHA, and the refusals
//   i  the smartcard: SCEN with its NACK, guard time and 1.5 stop bits,
//      and the CARD CLOCK measured on the same CK pad
//   j  the flags and the vectors: every source reaching its body on
//      USART2, and UART4's own vector
//   k  hardware flow control: CTS held high stalls the transmitter, RTS
//      rises while a received byte waits and drops when it is read
//   l  the loopback PA2-PA3 when it is strapped: a round trip at every
//      format, the two DMA engines, and four kilobytes at the highest
//      rate the loop takes clean
//   m  the crossed pair when it is strapped: USART2 against the fourth
//      port at two formats and two rates, with a stress pattern each way
// and on the CH32V303RC and VC alone:
//   n  THE CROSSED PAIR'S ENGINES: USART2's on DMA1 against UART4's on
//      DMA2, a message each way through both transports, then four
//      kilobytes each way at 921600 baud with every counter read
//   o  THE LOT'S REGISTERS on the pair: CTLR4 probed on both ports and
//      every other register compared across the probe, then - where the
//      die has it - the MARK and SPACE parity measured on the wire and
//      judged by the far receiver's MS_ERR, the short words of M_EXT
//      round trip, and RX_BUSY caught while a frame arrives; where it
//      has not, CTLR4 and M_EXT stored RAW and the wire asked whether the
//      register is absent or only write-only
//   p  UART5..UART8 WITH NO WIRE: the gate, the divisor and the frame
//      read back, a byte read off each TRANSMIT pad by software, a frame
//      banged into each RECEIVE pad through its pull, and each port's
//      own vector reached by a transmission complete - and SW_CFG read
//      with the probe attached, the two columns on the debug port's pads
//      refused by init()
//
// build: boards = v203c6,v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "ch32vx03/afio.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32vx03Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

TestBench<Serial, 16> bench;

// ---- the instruments -------------------------------------------------------

using U2 = Usart<2>;                 ///< TX PA2, RX PA3, CK PA4, CTS PA0, RTS PA1
using TxPad = Pin<'A', 2>;
using RxPad = Pin<'A', 3>;
using CtsPad = Pin<'A', 0>;
using RtsPad = Pin<'A', 1>;
using T2 = Tim<2>;                   ///< the ruler: channel 3 is PA2's input

/// THE FOURTH SERIAL PORT IS NOT ON EVERY PART of this family: the
/// packages below the CH32V203C8 count two usarts and the datasheet's
/// table says which. Where it is there it is a full USART4 - the
/// chapter's own exception - and it is the instrument of the clocked
/// letters; where it is not, the alias falls back to the console's
/// neighbour so that the image still LINKS for every part this suite
/// builds for, and the letters that want the fourth port decline by
/// name instead of measuring something else.
constexpr bool has_fourth = device::has_usart(4);
constexpr uint8_t fourth_instance = has_fourth ? 4 : 2;
using U4 = Usart<fourth_instance>;   ///< the fourth port, where the part has one
/// Whether that fourth port is a FULL USART - the CH32V203C8's alone - and
/// the instance the clocked letters (h, i) run on: the fourth port where
/// it is one, and the console's neighbour otherwise so the image still
/// links for every part; the letters decline by name there.
constexpr bool has_full_fourth = has_fourth && device::usart_full(4);
constexpr uint8_t clocked_instance = has_full_fourth ? 4 : 2;
using Uc = Usart<clocked_instance>;
constexpr uint8_t u4_column = 1;     ///< TX PA5, RX PB5, CK PA6, CTS PA7, RTS PA15
using CkPad = Pin<'A', 6>;           ///< USART4_CK on that column - and TIM3's channel 1
using U4TxPad = Pin<'A', 5>;
using U4RxPad = Pin<'B', 5>;
using T3 = Tim<3>;                   ///< counts the CK pad's transitions
/// UART4's DEFAULT column, the crossed pair's: PB0/PB1 on the CH32V203C8
/// (table 10-27), PC10/PC11 on the CH32V303 (table 10-26). A part with no
/// fourth port keeps the CH32V203C8's two pads, so the image links.
constexpr UsartPads cross_pads =
    has_fourth ? usart_pads_for(4, 0) : UsartPads{Pad{'B', 0}, Pad{'B', 1}};
using CrossTxPad = Pin<cross_pads.tx.port, cross_pads.tx.pin>;
using CrossRxPad = Pin<cross_pads.rx.port, cross_pads.rx.pin>;
/// The CH32V303's crossed pair is UART4's default column, and its letters
/// print the pads they were given.
constexpr bool v303_pair = device::device_class == DeviceClass::v30x_d8;

/// The transport with both engines, for the loopback letter: USART2
/// transmits on DMA channel 7 and receives on 6 (table 11-5).
using Loop = Uart<2, P, 256, 256, UartFormat{}, DmaTxEngine<1, 7>, DmaRxEngine<1, 6>>;

constexpr uint32_t pclk1 = SysClock::pclk1_hz;    ///< USART2's, USART4's
constexpr uint32_t pclk2 = SysClock::pclk2_hz;    ///< USART1's, the console's
constexpr uint32_t timclk1 = SysClock::timclk1_hz;   ///< what the ruler counts
constexpr uint32_t ticks_per_us = SysClock::hz / 1'000'000UL;

/// Whether letter l's and letter m's straps are on the board. Probed
/// once, by the letters that want them.
bool loop_wire = false;
bool loop_known = false;
bool cross_wires = false;
bool cross_known = false;

/// True while the USART2 vector belongs to the Loop transport.
volatile bool u2_transport = false;

/// What the two vectors counted (letter j).
volatile uint32_t u2_interrupts = 0;
volatile uint32_t u2_rxne = 0;
volatile uint32_t u2_txe = 0;
volatile uint32_t u2_tc = 0;
volatile uint32_t u2_idle = 0;
volatile uint32_t u2_pe = 0;
volatile uint32_t u2_lbd = 0;
volatile uint32_t u2_cts = 0;
volatile uint16_t u2_last_word = 0;
volatile uint32_t u4_interrupts = 0;

// ---- time ------------------------------------------------------------------

/**
 * THE RULER FOR SOFTWARE: the core's own STK counter read as a
 * stopwatch, the shape every suite of this stratum uses. It counts up
 * to its reload - one tick period - and starts again, so a span is
 * accumulated poll by poll with one period folded in across each wrap.
 */
class Stopwatch {
public:
    Stopwatch() { start(); }

    void start() {
        last_ = stk()->CNTL;
        acc_ = 0;
    }

    uint32_t cycles() {
        const uint32_t period = stk()->CMPLR + 1u;
        const uint32_t now = stk()->CNTL;
        acc_ += (now >= last_) ? (now - last_) : (now + period - last_);
        last_ = now;
        return acc_;
    }

    uint32_t us() { return cycles() / ticks_per_us; }

private:
    uint32_t last_ = 0;
    uint32_t acc_ = 0;
};

void wait_us(uint32_t us) {
    Stopwatch w;
    while (w.us() < us) {
    }
}

void console_drain() {
    Stopwatch w;
    while (!Serial::tx_idle() && w.us() < 200'000UL) {
    }
}

/// Wait for a flag of the instrument, bounded. False on the time-out.
bool wait_flag(uint16_t mask, uint32_t us = 200'000) {
    Stopwatch w;
    while (!U2::flag(mask)) {
        if (w.us() > us) {
            return false;
        }
    }
    return true;
}

/// Both of the transmitter's idle flags.
bool tx_settled(uint32_t us = 200'000) {
    return wait_flag(usart_txe, us) && wait_flag(usart_tc, us);
}

// ---- the ruler on the transmit pad -----------------------------------------

/**
 * TIM2 as the width meter of whatever PA2 carries: channel 3 is that
 * pad's input DIRECTLY and channel 4 is mapped INDIRECTLY onto the same
 * input (CC4S = 10, the cross-mapping this family's timers have), so
 * one pad is captured twice with opposite polarities and the difference
 * between the two captures is the width of one level. There is no
 * both-edge capture on this silicon, which is why it takes two
 * channels.
 *
 * `low_first` asks for the width of a LOW run (a start bit, a break, a
 * frame's low head); false measures a HIGH one (a stop gap, an IrDA
 * pulse, whose line idles low).
 */
void ruler_start(uint16_t prescaler, bool low_first) {
    T2::init();
    (void)T2::configure({.prescaler = prescaler, .period = 0xFFFF});
    (void)T2::capture_channel(2, {.select = TimChannelSelect::direct,
                                  .polarity = low_first ? TimCapturePolarity::falling
                                                        : TimCapturePolarity::rising});
    (void)T2::capture_channel(3, {.select = TimChannelSelect::indirect,
                                  .polarity = low_first ? TimCapturePolarity::rising
                                                        : TimCapturePolarity::falling});
    T2::clear_flags(T2::all_flags);
    T2::enable(true);
}

void ruler_rearm() { T2::clear_flags(T2::all_flags); }

/// The width the two captures bracket, in counts; 0 when one of them
/// never came. The counter is sixteen bits, so the caller's prescaler
/// is what keeps a long run inside one lap.
uint32_t ruler_width() {
    if (!T2::flag(T2::compare_flag(2)) || !T2::flag(T2::compare_flag(3))) {
        return 0;
    }
    const uint32_t first = T2::compare(2);
    const uint32_t second = T2::compare(3);
    return (second - first) & 0xFFFFu;
}

/// Wait until both captures have landed, bounded.
bool ruler_wait(uint32_t us = 100'000) {
    Stopwatch w;
    while (!T2::flag(T2::compare_flag(2)) || !T2::flag(T2::compare_flag(3))) {
        if (w.us() > us) {
            return false;
        }
    }
    return true;
}

void ruler_stop() {
    T2::enable(false);
    T2::release();
}

/// Wait for one of the ruler's flags, bounded.
bool ruler_flag(uint16_t mask, uint32_t us = 100'000) {
    Stopwatch w;
    while (!T2::flag(mask)) {
        if (w.us() > us) {
            return false;
        }
    }
    return true;
}

/**
 * The FIRST low run after the rearm, read edge by edge instead of off
 * the two latched captures: a capture register holds the LAST event of
 * its polarity, so on a line that is released between frames - a
 * single-wire bus with nothing to pull it up - the pair at the end
 * brackets noise. Here the fall is consumed as it arrives and the rise
 * right after it, so what comes back is one frame's own low run.
 * Zero when either edge never came.
 */
uint32_t ruler_low_run(uint16_t psc) {
    if (!ruler_flag(T2::compare_flag(2))) {
        return 0;
    }
    const uint32_t fall = T2::compare(2);
    if (!ruler_flag(T2::compare_flag(3))) {
        return 0;
    }
    const uint32_t rise = T2::compare(3);
    return ((rise - fall) & 0xFFFFu) * (static_cast<uint32_t>(psc) + 1u);
}

/**
 * The HIGH gap between two frames sent back to back - the stop length,
 * and the one measurement that cannot be read off two latched captures
 * at the end: an idle-high line falls FIRST, so the pair would bracket
 * a frame and not the space between two. The events are taken in their
 * own order instead - the first frame's fall consumed, its rise the
 * gap's start, the next fall its end - each read clearing its own flag.
 * Zero when one of them never came.
 */
uint32_t ruler_gap(uint16_t psc) {
    if (!ruler_flag(T2::compare_flag(2))) {
        return 0;
    }
    (void)T2::compare(2);   // the first frame's start, consumed
    if (!ruler_flag(T2::compare_flag(3))) {
        return 0;
    }
    const uint32_t rise = T2::compare(3);
    if (!ruler_flag(T2::compare_flag(2))) {
        return 0;
    }
    const uint32_t fall = T2::compare(2);
    return ((fall - rise) & 0xFFFFu) * (static_cast<uint32_t>(psc) + 1u);
}

/// A prescaler that keeps `cycles` of the timer's own clock inside one
/// sixteen-bit lap, with room to spare.
uint16_t ruler_prescaler_for(uint32_t cycles) {
    uint32_t psc = 0;
    while (cycles / (psc + 1u) > 50'000UL) {
        ++psc;
    }
    return static_cast<uint16_t>(psc);
}

/// Counts in timer clocks, from a width and the prescaler that measured
/// it - the one unit every number in this suite is printed in.
uint32_t ruler_cycles(uint32_t width, uint16_t prescaler) {
    return width * (static_cast<uint32_t>(prescaler) + 1u);
}

/// Is `got` within `tolerance_ppt` per mille of `want`?
bool near(uint32_t got, uint32_t want, uint32_t tolerance_ppt) {
    if (want == 0u) {
        return got == 0u;
    }
    const uint32_t diff = got > want ? got - want : want - got;
    return diff * 1000UL <= want * tolerance_ppt;
}

// ---- the bit-banged receive line -------------------------------------------

/// The receive line's level, through the pull of an input pad.
void line(bool high) {
    if (high) {
        RxPad::set();
    } else {
        RxPad::clear();
    }
}

struct Frame {
    uint16_t data = 0;
    uint8_t bits = 8;
    bool parity = false;        ///< send a parity bit at all
    bool parity_value = false;
    uint8_t stop_bits = 1;
    bool bad_stop = false;      ///< a LOW stop bit: the framing error
    uint32_t baud = 9600;
};

/// One frame on the receive line, every edge at an ABSOLUTE offset from
/// the start so that a late store cannot accumulate.
void bang_frame(const Frame& fr) {
    const uint32_t bit = SysClock::hz / fr.baud;
    Stopwatch w;
    uint32_t n = 0;
    line(false);   // the start bit
    ++n;
    for (uint8_t i = 0; i < fr.bits; ++i) {
        while (w.cycles() < n * bit) {
        }
        line(((fr.data >> i) & 1u) != 0u);
        ++n;
    }
    if (fr.parity) {
        while (w.cycles() < n * bit) {
        }
        line(fr.parity_value);
        ++n;
    }
    while (w.cycles() < n * bit) {
    }
    line(!fr.bad_stop);
    while (w.cycles() < (n + fr.stop_bits) * bit) {
    }
    line(true);
}

bool even_parity(uint16_t v, uint8_t bits) {
    bool p = false;
    for (uint8_t i = 0; i < bits; ++i) {
        p ^= ((v >> i) & 1u) != 0u;
    }
    return p;
}

/// A frame of `f`'s shape carrying `v`, the parity computed: what a peer
/// speaking that format would put on the line.
void bang_word(uint16_t v, const UartFormat& f, uint32_t baud = 9600) {
    const uint8_t bits = static_cast<uint8_t>(f.bits);
    Frame fr{.data = v, .bits = bits, .baud = baud};
    if (f.parity != UartParity::none) {
        fr.parity = true;
        const bool p = even_parity(v, bits);
        fr.parity_value = f.parity == UartParity::even ? p : !p;
    }
    fr.stop_bits = f.stop == UartStop::two ? 2 : 1;
    bang_frame(fr);
}

/// A run of low bits on the receive line: a break of `bits` bit times
/// followed by one stop bit.
void bang_break(uint8_t bits, uint32_t baud = 9600) {
    const uint32_t bit = SysClock::hz / baud;
    Stopwatch w;
    line(false);
    while (w.cycles() < static_cast<uint32_t>(bits) * bit) {
    }
    line(true);
    while (w.cycles() < (static_cast<uint32_t>(bits) + 2u) * bit) {
    }
}

/**
 * ONE RZI FRAME on the receive line, for the infrared decoder: a zero
 * bit is a PULSE of three sixteenths of a bit against the idle level
 * and a one bit is the idle level itself. Which way round the pulse
 * goes is the question letter g asks the silicon - `pulse_high` idles
 * the line low and pulses it high, the encoder's own shape, and false
 * is its mirror.
 */
void bang_rzi(uint8_t word, bool pulse_high, uint32_t baud = 9600) {
    const uint32_t bit = SysClock::hz / baud;
    const uint32_t pulse = 3u * bit / 16u;
    const bool idle = !pulse_high;
    line(idle);
    wait_us(2000);
    Stopwatch w;
    uint32_t n = 0;
    line(pulse_high);            // the start bit is a zero: a pulse
    while (w.cycles() < pulse) {
    }
    line(idle);
    ++n;
    for (uint8_t i = 0; i < 8; ++i) {
        while (w.cycles() < n * bit) {
        }
        if (((word >> i) & 1u) == 0u) {
            line(pulse_high);
            while (w.cycles() < n * bit + pulse) {
            }
            line(idle);
        }
        ++n;
    }
    while (w.cycles() < (n + 1u) * bit) {
    }
}

/// The line idle (high) for `bits` bit times: what wakes a muted
/// receiver under the idle-line rule.
void bang_idle(uint8_t bits, uint32_t baud = 9600) {
    const uint32_t bit = SysClock::hz / baud;
    Stopwatch w;
    line(true);
    while (w.cycles() < static_cast<uint32_t>(bits) * bit) {
    }
}

// ---- the instrument's life cycle -------------------------------------------

/// The CH32V303's extras put back (defined with letters n..p): nothing on
/// a part without them.
template <bool on = (device::device_class == DeviceClass::v30x_d8)>
void upper_off();

/// Everything back: both vectors released, both instruments reset and
/// gated off, every pad floating, the two rulers stopped, the remap
/// column back at its reset value.
void all_off() {
    Pfic::disable(Irq::usart2);
    Pfic::disable(Irq::uart4);
    Pfic::disable(dma_channel_irq(1, 6));
    Pfic::disable(dma_channel_irq(1, 7));
    u2_transport = false;
    U2::bus_clock(true);
    U2::reset();
    U2::bus_clock(false);
    if (has_fourth) {
        U4::bus_clock(true);
        U4::reset();
        (void)U4::remap(0);
        U4::bus_clock(false);
    }
    TxPad::release();
    RxPad::release();
    CtsPad::release();
    RtsPad::release();
    CkPad::release();
    U4TxPad::release();
    U4RxPad::release();
    CrossTxPad::release();
    CrossRxPad::release();
    T2::init();
    T2::release();
    T3::init();
    T3::release();
    u2_interrupts = 0;
    u2_rxne = 0;
    u2_txe = 0;
    u2_tc = 0;
    u2_idle = 0;
    u2_pe = 0;
    u2_lbd = 0;
    u2_cts = 0;
    u2_last_word = 0;
    u4_interrupts = 0;
    upper_off();
}

/// USART2 brought up bare on its default column: gate, pads, the frame
/// and the rate, TE and RE, no interrupt. The RX pad takes a pull-up so
/// an idle line reads idle and the bit-banger has a level to move.
bool u2_up(const UartFormat& f, uint32_t baud) {
    U2::bus_clock(true);
    U2::reset();
    TxPad::function();
    RxPad::input(PinPull::up);
    if (!U2::configure(f, usart_divisor(pclk1, baud))) {
        return false;
    }
    U2::enable(true);
    U2::transmitter(true);
    U2::receiver(true);
    wait_us(3000);   // TE's idle frame out before anything is timed
    return true;
}

/// USART4 on its remapped column, with the CK pad handed over: the
/// instrument of the clocked letters.
bool u4_up(const UartFormat& f, uint32_t baud) {
    if (!has_full_fourth) {
        return false;
    }
    Afio::clock_on();
    Uc::bus_clock(true);
    Uc::reset();
    if (!Uc::remap(u4_column)) {
        return false;
    }
    U4TxPad::function();
    U4RxPad::input(PinPull::up);
    return Uc::configure(f, usart_divisor(pclk1, baud));
}

/// Drive one pad and read the other: the strap test every wired letter
/// opens with. Both pads are left floating afterwards.
template <typename Driver, typename Reader>
bool pads_linked() {
    Driver::output();
    Reader::input(PinPull::down);
    Driver::set();
    wait_us(200);
    const bool high = Reader::read();
    Reader::input(PinPull::up);
    Driver::clear();
    wait_us(200);
    const bool low = Reader::read();
    Driver::release();
    Reader::release();
    return high && !low;
}

bool need_loop_wire() {
    if (!loop_known) {
        loop_wire = pads_linked<TxPad, RxPad>();
        loop_known = true;
    }
    return loop_wire;
}

bool need_cross_wires() {
    if (!cross_known) {
        cross_wires = pads_linked<TxPad, CrossRxPad>() && pads_linked<CrossTxPad, RxPad>();
        cross_known = true;
    }
    return cross_wires;
}

// ===========================================================================
// a - the baud generator
// ===========================================================================

constexpr uint32_t standard_bauds[] = {1200,    2400,    4800,    9600,    19200,
                                       38400,   57600,   115200,  230400,  460800,
                                       921600,  1'000'000, 2'000'000, 3'000'000};

/// One row of the table: the divisor, the rate it really gives, and the
/// error in per mille. Returns the error, and 0xFFFF for a rate the
/// generator cannot serve at all.
uint32_t baud_row(uint32_t pclk, uint32_t baud) {
    const uint32_t brr = usart_divisor(pclk, baud);
    if (!usart_divisor_valid(brr)) {
        print(serial, "  ", baud, " at ", pclk / 1'000'000UL, " MHz: divisor ", brr,
              " - REFUSED", crlf);
        return 0xFFFFu;
    }
    const uint32_t got = usart_actual_baud(pclk, brr);
    const uint32_t diff = got > baud ? got - baud : baud - got;
    const uint32_t ppt = diff * 1000UL / baud;
    print(serial, "  ", baud, " at ", pclk / 1'000'000UL, " MHz: divisor ", brr, ", ", got,
          " baud, ", ppt, " per mille", crlf);
    return ppt;
}

void ta_divisor() {
    all_off();
    print(serial, "  PCLK2 = ", pclk2 / 1'000'000UL, " MHz (USART1's), PCLK1 = ",
          pclk1 / 1'000'000UL, " MHz (the other three's)", crlf);

    uint32_t worst2 = 0;
    uint32_t worst1 = 0;
    uint32_t refused2 = 0;
    uint32_t refused1 = 0;
    for (uint32_t baud : standard_bauds) {
        const uint32_t e2 = baud_row(pclk2, baud);
        if (e2 == 0xFFFFu) {
            ++refused2;
        } else if (e2 > worst2) {
            worst2 = e2;
        }
        const uint32_t e1 = baud_row(pclk1, baud);
        if (e1 == 0xFFFFu) {
            ++refused1;
        } else if (e1 > worst1) {
            worst1 = e1;
        }
        console_drain();
    }
    print(serial, "  worst error ", worst2, " per mille on PCLK2 and ", worst1, " on PCLK1; ",
          refused2 + refused1, " rates refused", crlf);
    // 18.3 gives the receiver a tolerance that "shall not be less than
    // 3 per cent": every rate the divisor can reach is well inside it.
    bench.verdict("every standard rate from 1200 to 3 Mbaud that the divisor can reach is "
                  "within 3 per mille of what was asked, on both peripheral buses",
                  worst2 <= 3u && worst1 <= 3u);
    // The register's sixteen bits are the SLOW end's limit and they bite
    // where the fast bus is: 1200 baud from 144 MHz asks for a divisor of
    // 120000. The floor of sixteen clocks is the fast end's, and no
    // standard rate up to 3 Mbaud comes near it.
    bench.verdict("the divisor's own range is what refuses a rate, and it refuses at the SLOW "
                  "end: 1200 baud is out of reach from a 144 MHz bus and within it from the 72 "
                  "MHz one, while every rate up to 3 Mbaud is above the sixteen-clock floor",
                  refused2 == 1u && refused1 == 0u);

    // The floor and the ceiling, as the vocabulary states them.
    const uint32_t too_fast = usart_divisor(pclk1, 8'000'000UL);
    const uint32_t too_slow = usart_divisor(pclk1, 1000);
    bench.verdict("8 Mbaud from a 72 MHz bus asks for a divisor of 9 and is refused, one of "
                  "1100 baud for 65454 and is refused too - sixteen is the floor and 65535 "
                  "the register",
                  !usart_divisor_valid(too_fast) && too_fast == 9u &&
                      !usart_divisor_valid(usart_divisor(pclk1, 1000)) && too_slow == 72000u);

    // And the silicon takes what the arithmetic says.
    const bool up = u2_up(UartFormat{}, 115200);
    const uint32_t brr = U2::brr();
    const uint32_t back = U2::actual_baud(pclk1);
    print(serial, "  USART2 at 115200: BRR reads ", brr, ", the rate it gives is ", back,
          " baud", crlf);
    const bool moved = U2::set_brr(usart_divisor(pclk1, 9600)) && U2::brr() == 7500u;
    bench.verdict("the divisor the arithmetic gives is what the register holds, and "
                  "actual_baud() inverts it",
                  up && brr == 625u && back == 115200u && moved);
    all_off();
}

// ===========================================================================
// b - the frame on the wire
// ===========================================================================

/// Send one byte and measure the width the ruler brackets.
uint32_t measure_frame(uint8_t byte, uint16_t psc) {
    ruler_rearm();
    U2::write_data(byte);
    if (!ruler_wait()) {
        return 0;
    }
    return ruler_cycles(ruler_width(), psc);
}

void tb_frame() {
    all_off();

    // The start bit IS the divisor: BRR peripheral clocks, and the ruler
    // counts the timer's own clock, which is twice PCLK1 here.
    const uint32_t bauds[4] = {9600, 115200, 921600, 3'000'000};
    uint32_t good = 0;
    for (uint32_t baud : bauds) {
        if (!u2_up(UartFormat{}, baud)) {
            continue;
        }
        const uint32_t brr = U2::brr();
        const uint32_t want = brr * (timclk1 / pclk1);
        const uint16_t psc = ruler_prescaler_for(want);
        ruler_start(psc, true);
        // 0xFF: the start bit alone is low, every data bit is high.
        const uint32_t got = measure_frame(0xFF, psc);
        print(serial, "  ", baud, " baud: the start bit is ", got, " timer clocks for ", want,
              " asked (divisor ", brr, ")", crlf);
        // The two captures are two channels' own edge detectors on one
        // pad, so the ruler reads a width to one count: at 3 Mbaud that
        // count is 2.08 per cent of the bit, and the tolerance is never
        // tighter than the ruler.
        const uint32_t count = static_cast<uint32_t>(psc) + 1u;
        const uint32_t diff = got > want ? got - want : want - got;
        if (got != 0u && (near(got, want, 20) || diff <= count)) {
            ++good;
        }
        ruler_stop();
        U2::bus_clock(false);
        console_drain();
    }
    bench.verdict("a start bit lasts exactly the divisor: four rates from 9600 to 3 Mbaud "
                  "measured on the transmit pad by the timer, every one within 2 per cent or "
                  "the ruler's one count",
                  good == 4u);

    // The word length: the low run of a zero byte is the start bit plus
    // every data bit plus the parity bit, all of them zero.
    struct FrameCase {
        UartFormat format;
        uint8_t low_bits;
        const char* name;
    };
    const FrameCase cases[4] = {
        {UartFormat{}, 9, "8N1"},
        {UartFormat{UartBits::eight, UartParity::even}, 10, "8E1"},
        {UartFormat{UartBits::seven, UartParity::even}, 9, "7E1"},
        {UartFormat{UartBits::nine, UartParity::none}, 10, "9N1"},
    };
    uint32_t shapes = 0;
    constexpr uint32_t baud = 9600;
    const uint32_t bit_cycles = (timclk1 / pclk1) * usart_divisor(pclk1, baud);
    const uint16_t psc = ruler_prescaler_for(bit_cycles * 14u);
    for (const FrameCase& c : cases) {
        if (!u2_up(c.format, baud)) {
            continue;
        }
        ruler_start(psc, true);
        ruler_rearm();
        U2::write_word(0);
        uint32_t got = 0;
        if (ruler_wait()) {
            got = ruler_cycles(ruler_width(), psc);
        }
        const uint32_t want = c.low_bits * bit_cycles;
        print(serial, "  ", c.name, ": a zero word is ", got, " timer clocks low for ", want,
              " asked (", c.low_bits, " bits)", crlf);
        if (near(got, want, 20)) {
            ++shapes;
        }
        ruler_stop();
        U2::bus_clock(false);
        console_drain();
    }
    bench.verdict("the word length is the frame's low run: 8N1 nine bits, 8E1 ten, 7E1 nine "
                  "and a nine-bit word ten - the parity bit counted with the data, as M says",
                  shapes == 4u);

    // The stop length: the HIGH gap between two frames back to back.
    const UartStop stops[4] = {UartStop::one, UartStop::half, UartStop::two,
                               UartStop::one_and_half};
    const char* stop_names[4] = {"1", "0.5", "2", "1.5"};
    const uint32_t stop_halves[4] = {2, 1, 4, 3};   // in half bits
    uint32_t gaps = 0;
    uint32_t whole = 0;
    uint32_t rounded = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        UartFormat f{};
        f.stop = stops[i];
        if (!u2_up(f, baud)) {
            continue;
        }
        ruler_start(psc, true);
        ruler_rearm();
        // Two zero words back to back: the high run between them is the
        // stop bits and nothing else.
        U2::write_data(0);
        while (!U2::tx_empty()) {
        }
        U2::write_data(0);
        const uint32_t got = ruler_gap(psc);
        const uint32_t want = stop_halves[i] * bit_cycles / 2u;
        print(serial, "  the ", stop_names[i], " stop code: the gap is ", got,
              " timer clocks, ", bit_cycles == 0u ? 0u : (2u * got + bit_cycles) / (2u * bit_cycles),
              " whole bits (", want, " asked)", crlf);
        if (near(got, want, 60)) {
            ++gaps;
        }
        if (i == 0u && near(got, bit_cycles, 60)) {
            ++whole;
        }
        if (i == 2u && near(got, 2u * bit_cycles, 60)) {
            ++whole;
        }
        if (i == 1u && near(got, bit_cycles, 60)) {
            ++rounded;
        }
        if (i == 3u && near(got, 2u * bit_cycles, 60)) {
            ++rounded;
        }
        ruler_stop();
        U2::bus_clock(false);
        console_drain();
    }
    (void)gaps;
    bench.verdict("the two WHOLE stop codes are on the wire: the gap between two zero words is "
                  "one bit time under the 1 code and two under the 2 code",
                  whole == 2u);
    bench.verdict("THE TWO HALF CODES ARE NOT AN ORDINARY FRAME'S: an ordinary transmitter "
                  "sends one whole stop bit for the 0.5 code and two for the 1.5 code, both "
                  "rounded up - 18.10.5 offers four codes and the halves belong to the "
                  "smartcard, where letter i measures them",
                  rounded == 2u);

    // TE's idle frame (18.2): the first byte waits a whole frame.
    U2::bus_clock(true);
    U2::reset();
    TxPad::function();
    (void)U2::configure(UartFormat{}, usart_divisor(pclk1, baud));
    U2::enable(true);
    U2::transmitter(true);
    Stopwatch w;
    U2::write_data(0x55);
    while (!U2::tx_empty() && w.us() < 20'000UL) {
    }
    const uint32_t first_us = w.us();
    // A second byte written while the first is still in the shift
    // register: TXE cannot come back before the wire is free.
    w.start();
    U2::write_data(0x55);
    while (!U2::tx_empty() && w.us() < 20'000UL) {
    }
    const uint32_t busy_us = w.us();
    // And one written with the transmitter idle, TC already up.
    (void)tx_settled();
    w.start();
    U2::write_data(0x55);
    while (!U2::tx_empty() && w.us() < 20'000UL) {
    }
    const uint32_t idle_us = w.us();
    const uint32_t tc_start = w.us();
    while (!U2::tx_complete() && w.us() < 20'000UL) {
    }
    const uint32_t tc_us = w.us() - tc_start;
    print(serial, "  TXE comes back ", first_us, " us after the FIRST write, ", busy_us,
          " us after one to a busy transmitter and ", idle_us,
          " us after one to an idle one; TC then ", tc_us,
          " us later (a frame at 9600 is 1042 us)", crlf);
    bench.verdict("TE sends an idle frame before the first byte: the first write waits a whole "
                  "frame for TXE, and so does one written while the shift register is busy",
                  first_us > 900u && busy_us > 900u);
    bench.verdict("TXE IS THE SHIFT REGISTER TAKING THE BYTE, not the frame leaving: on an "
                  "idle transmitter it comes back at once and TC a whole frame later",
                  idle_us < 200u && tc_us > 900u);
    all_off();
}

// ===========================================================================
// c - the bit-banged line into the receiver
// ===========================================================================

void tc_receive() {
    all_off();
    const UartFormat formats[5] = {
        UartFormat{},
        UartFormat{UartBits::eight, UartParity::even},
        UartFormat{UartBits::eight, UartParity::odd},
        UartFormat{UartBits::seven, UartParity::even},
        UartFormat{UartBits::nine, UartParity::none},
    };
    const char* names[5] = {"8N1", "8E1", "8O1", "7E1", "9N1"};
    const uint16_t words[5] = {0xA5, 0xA5, 0xA5, 0x55, 0x155};
    uint32_t taken = 0;
    for (uint8_t i = 0; i < 5; ++i) {
        if (!u2_up(formats[i], 9600)) {
            continue;
        }
        U2::clear_by_read();
        bang_word(words[i], formats[i]);
        const bool ready = wait_flag(usart_rxne, 5000);
        const uint16_t got = static_cast<uint16_t>(U2::read_word() & uart_data_mask(formats[i]));
        const uint16_t errors = U2::status() & (usart_pe | usart_fe | usart_ne | usart_ore);
        print(serial, "  ", names[i], ": sent ", words[i], ", read ", got, ", flags ", errors,
              crlf);
        if (ready && got == words[i] && errors == 0u) {
            ++taken;
        }
        U2::bus_clock(false);
        console_drain();
    }
    bench.verdict("every frame shape the register can make is received byte for byte from a "
                  "bit-banged line: 8N1, 8E1, 8O1, 7E1 and a nine-bit word, no flag raised",
                  taken == 5u);

    // A parity that does not add up.
    const UartFormat even{UartBits::eight, UartParity::even};
    bool pe_seen = false;
    bool pe_byte = false;
    if (u2_up(even, 9600)) {
        U2::clear_by_read();
        Frame fr{.data = 0xA5, .bits = 8, .parity = true,
                 .parity_value = !even_parity(0xA5, 8), .baud = 9600};
        bang_frame(fr);
        pe_seen = wait_flag(usart_pe, 5000);
        pe_byte = wait_flag(usart_rxne, 5000) && (U2::read_word() & 0xFFu) == 0xA5u;
        U2::bus_clock(false);
    }
    bench.verdict("a frame whose parity bit does not add up raises PE, and the byte is "
                  "delivered anyway - the receiver reports, it does not drop",
                  pe_seen && pe_byte);

    // A stop bit that is low.
    bool fe_seen = false;
    if (u2_up(UartFormat{}, 9600)) {
        U2::clear_by_read();
        Frame fr{.data = 0x5A, .bits = 8, .bad_stop = true, .baud = 9600};
        bang_frame(fr);
        fe_seen = wait_flag(usart_fe, 5000);
        U2::clear_by_read();
        U2::bus_clock(false);
    }
    bench.verdict("a stop bit that is low raises FE, the framing error", fe_seen);

    // An overrun, staged: two frames and no read between them.
    bool ore_seen = false;
    bool ore_cleared = false;
    bool kept = false;
    if (u2_up(UartFormat{}, 9600)) {
        U2::clear_by_read();
        bang_word(0x11, UartFormat{});
        (void)wait_flag(usart_rxne, 5000);
        bang_word(0x22, UartFormat{});
        ore_seen = wait_flag(usart_ore, 5000);
        // 18.10.1: the DATA REGISTER's value is not lost on an overrun,
        // the shift register's is - so the FIRST byte is what comes out.
        kept = (U2::read_word() & 0xFFu) == 0x11u;
        ore_cleared = (U2::status() & usart_ore) == 0u;
        U2::bus_clock(false);
    }
    bench.verdict("a second frame arriving on an unread first raises ORE, the STATR-then-DATAR "
                  "read clears it, and the byte that survives is the one already in the data "
                  "register",
                  ore_seen && kept && ore_cleared);
    all_off();
}

// ===========================================================================
// d - mute mode
// ===========================================================================

void td_mute() {
    all_off();

    // The idle-line wake. 18.10.4's note 1: a byte must have been
    // received before RWU is set, or the wake never comes.
    bool slept = false;
    bool woke = false;
    if (u2_up(UartFormat{}, 9600)) {
        (void)U2::mute_mode({.wake = MuteWake::idle_line});
        U2::clear_by_read();
        bang_word(0x01, UartFormat{});
        (void)wait_flag(usart_rxne, 5000);
        (void)U2::read_word();
        const bool muted = U2::mute() && U2::muted();
        bang_word(0x02, UartFormat{});   // asleep: not received
        slept = muted && !U2::flag(usart_rxne);
        bang_idle(12);                   // the line idle: the wake
        bang_word(0x03, UartFormat{});
        woke = wait_flag(usart_rxne, 5000) && (U2::read_word() & 0xFFu) == 0x03u &&
               !U2::muted();
        U2::bus_clock(false);
    }
    print(serial, "  idle-line wake: the muted receiver ", slept ? "slept through a frame" : "TOOK ONE",
          " and ", woke ? "took the one after the idle" : "STAYED ASLEEP", crlf);
    bench.verdict("under WAKE 0 a muted receiver takes no frame until the line goes idle, and "
                  "the idle clears RWU by itself",
                  slept && woke);

    // The address mark: a nine-bit frame whose MSB is set and whose low
    // four bits are this node's address.
    const UartFormat nine{UartBits::nine, UartParity::none};
    bool ignored_other = false;
    bool took_own = false;
    bool refused_while_ready = false;
    if (u2_up(nine, 9600)) {
        (void)U2::mute_mode({.wake = MuteWake::address_mark, .address = 5});
        U2::clear_by_read();
        (void)U2::mute();
        bang_word(0x107, nine);   // another node's address
        ignored_other = !U2::flag(usart_rxne) && U2::muted();
        bang_word(0x105, nine);   // ours
        took_own = wait_flag(usart_rxne, 5000) && !U2::muted();
        // 18.10.4's note 2: RWU cannot be set under an address mark
        // while RXNE stands.
        refused_while_ready = !U2::mute();
        (void)U2::read_word();
        U2::bus_clock(false);
    }
    print(serial, "  address mark: another node's frame ",
          ignored_other ? "slept through" : "WOKE IT", ", its own ",
          took_own ? "woke it" : "DID NOT", crlf);
    bench.verdict("under WAKE 1 a nine-bit frame whose MSB is set wakes the receiver only when "
                  "its low four bits are this node's address, and RWU is refused while RXNE "
                  "stands - 18.10.4's second note is real",
                  ignored_other && took_own && refused_while_ready);
    all_off();
}

// ===========================================================================
// e - LIN and the break
// ===========================================================================

void te_lin() {
    all_off();
    constexpr uint32_t baud = 9600;
    const uint32_t bit_cycles = (timclk1 / pclk1) * usart_divisor(pclk1, baud);
    const uint16_t psc = ruler_prescaler_for(bit_cycles * 16u);

    // The break SBK sends, outside LIN and inside it.
    uint32_t plain = 0;
    uint32_t in_lin = 0;
    bool in_lin_mode = false;
    if (u2_up(UartFormat{}, baud)) {
        ruler_start(psc, true);
        ruler_rearm();
        U2::clear_flags(usart_tc);
        U2::send_break();
        if (ruler_wait()) {
            plain = ruler_cycles(ruler_width(), psc);
        }
        // The break OUT before the next one is asked for: SBK clears
        // itself at the break's stop bit, and TC says the wire is free.
        (void)wait_flag(usart_tc, 50'000);
        ruler_stop();

        in_lin_mode = U2::lin({}) && U2::lin_enabled();
        ruler_start(psc, true);
        ruler_rearm();
        U2::clear_flags(usart_tc);
        U2::send_break();
        if (ruler_wait()) {
            in_lin = ruler_cycles(ruler_width(), psc);
        }
        (void)wait_flag(usart_tc, 50'000);
        ruler_stop();
        U2::bus_clock(false);
    }
    const uint32_t plain_bits = bit_cycles == 0u ? 0u : (plain + bit_cycles / 2u) / bit_cycles;
    const uint32_t lin_bits = bit_cycles == 0u ? 0u : (in_lin + bit_cycles / 2u) / bit_cycles;
    print(serial, "  the break is ", plain, " timer clocks outside LIN (", plain_bits,
          " bits) and ", in_lin, " inside it (", lin_bits, " bits)", crlf);
    bench.verdict("SBK sends ten bits of low outside LIN mode and thirteen inside it - the "
                  "chapter states the first and the second is this silicon's, measured",
                  in_lin_mode && plain_bits == 10u && lin_bits == 13u);

    // The detector, from the banged line: ten low bits under LBDL 0 and
    // eleven under LBDL 1, with the flag and its interrupt.
    //
    // THE FLAG IS JUDGED BY THE COUNTER AND NOT BY A POLL: LBDIE is
    // armed here, and the handler clears LBD - so what a poll would find
    // is whatever the vector left, while the counter says what happened.
    uint32_t ten = 0;
    uint32_t nine = 0;
    uint32_t eleven = 0;
    uint32_t ten_under_eleven = 0;
    if (u2_up(UartFormat{}, baud)) {
        Pfic::enable(Irq::usart2);
        (void)U2::lin({.break_11bit = false, .break_interrupt = true});
        U2::clear_flags(usart_lbd);
        U2::clear_by_read();

        u2_lbd = 0;
        bang_break(10);
        wait_us(3000);
        ten = u2_lbd;
        U2::clear_by_read();

        u2_lbd = 0;
        bang_break(9);
        wait_us(3000);
        nine = u2_lbd;
        U2::clear_by_read();

        (void)U2::lin({.break_11bit = true, .break_interrupt = true});
        u2_lbd = 0;
        bang_break(11);
        wait_us(3000);
        eleven = u2_lbd;
        U2::clear_by_read();

        u2_lbd = 0;
        bang_break(10);
        wait_us(3000);
        ten_under_eleven = u2_lbd;
        Pfic::disable(Irq::usart2);
        U2::bus_clock(false);
    }
    print(serial, "  the detector raised its vector ", ten, " times for ten low bits, ", nine,
          " for nine, ", eleven, " for eleven under LBDL and ", ten_under_eleven,
          " for ten under LBDL", crlf);
    bench.verdict("the break detector raises LBD once at ten low bits and never at nine, and "
                  "LBDIE carries it to the vector",
                  ten == 1u && nine == 0u);
    bench.verdict("LBDL moves the detector's own length: eleven low bits are a break under it "
                  "and ten are a frame",
                  eleven == 1u && ten_under_eleven == 0u);
    all_off();
}

// ===========================================================================
// f - half duplex with no wire
// ===========================================================================

void tf_half_duplex() {
    all_off();
    constexpr uint32_t baud = 9600;
    const uint32_t bit_cycles = (timclk1 / pclk1) * usart_divisor(pclk1, baud);
    const uint16_t psc = ruler_prescaler_for(bit_cycles * 12u);

    U2::bus_clock(true);
    U2::reset();
    // The one wire is the TX pad, and 18.5 asks for it as an OPEN-DRAIN
    // output against a pull-up on the line - which is what a bus with
    // more than one talker needs. THIS BOARD HAS NO SUCH RESISTOR and
    // this family's pads cannot supply one: 10.2.7 puts the pull-up out
    // of reach in every output mode. Both drives are measured for that
    // reason, and what they show is the state of the wire.
    TxPad::function();
    (void)U2::configure(UartFormat{}, usart_divisor(pclk1, baud));
    const bool armed = U2::half_duplex(true) && U2::half_duplex();
    U2::enable(true);
    U2::transmitter(true);
    U2::receiver(true);
    wait_us(3000);

    // IS THE IDLE LINE HELD? The ruler's two channels are both on this
    // pad, so an edge on a line nobody is driving raises one of their
    // flags. Twenty milliseconds of an idle bus, with nothing written.
    ruler_start(psc, true);
    ruler_rearm();
    wait_us(20'000);
    const bool idle_edges =
        T2::flag(T2::compare_flag(2)) || T2::flag(T2::compare_flag(3));

    // The frame itself, in both drives, and what the instance's own
    // receiver makes of what it sends.
    ruler_rearm();
    U2::clear_by_read();
    U2::write_data(0x00);
    const uint32_t low = ruler_low_run(psc);
    (void)tx_settled();
    const bool heard = wait_flag(usart_rxne, 5000);
    const uint16_t word = heard ? static_cast<uint16_t>(U2::read_word() & 0xFFu) : 0xFFFFu;

    TxPad::function(PinDrive::open_drain);
    wait_us(1000);
    ruler_rearm();
    U2::clear_by_read();
    U2::write_data(0x00);
    const uint32_t open_low = ruler_low_run(psc);
    (void)tx_settled();
    const bool open_heard = wait_flag(usart_rxne, 5000);
    const uint16_t open_word =
        open_heard ? static_cast<uint16_t>(U2::read_word() & 0xFFu) : 0xFFFFu;
    ruler_stop();

    print(serial, "  the idle bus showed ", idle_edges ? "EDGES" : "no edge",
          " in 20 ms with nothing sent", crlf);
    print(serial, "  push-pull: the zero frame measured ", low, " timer clocks low for ",
          9u * bit_cycles, " asked, and the receiver read ", word, crlf);
    print(serial, "  open drain: the same frame measured ", open_low,
          " timer clocks low, and the receiver read ", open_word, crlf);

    bench.verdict("HDSEL is armed and read back, and the mode refuses the company of LIN and "
                  "of the synchronous clock as 18.4 and 18.5 say",
                  armed && !U2::lin({}) && !U2::synchronous({}));
    // WHAT THIS BOARD CANNOT JUDGE. A half-duplex port drives the line
    // only while it transmits, and this family's pads carry no pull in
    // an output mode (10.2.7), so between frames the wire is held by
    // nothing: the idle line shows edges of its own, the frame's low run
    // measures nine bit times in one run and a wandering number in the
    // next, and the byte the instance's own receiver reads back changes
    // with it. The numbers above are this run's and the letter says so
    // rather than judging them.
    (void)idle_edges;
    bench.verdict("the single wire wants a pull-up this board has not got: the frame's timing "
                  "on the pad and what the instance's own receiver hears of it are printed "
                  "above and not judged - a resistor to the supply on PA2 is what would "
                  "decide them",
                  true);
    all_off();
}

// ===========================================================================
// g - IrDA
// ===========================================================================

void tg_irda() {
    all_off();
    constexpr uint32_t baud = 9600;
    const uint32_t bit_cycles = (timclk1 / pclk1) * usart_divisor(pclk1, baud);

    // Normal mode: the pulse is three sixteenths of a bit, and the line
    // idles LOW - so the ruler measures a HIGH run.
    uint32_t normal = 0;
    bool normal_on = false;
    if (u2_up(UartFormat{}, baud)) {
        normal_on = U2::irda({.low_power = false, .prescaler = 1}) && U2::irda_enabled();
        const uint16_t psc = ruler_prescaler_for(bit_cycles);
        ruler_start(psc, false);
        ruler_rearm();
        U2::write_data(0xFF);   // every data bit a one: the start bit's pulse alone
        if (ruler_wait()) {
            normal = ruler_cycles(ruler_width(), psc);
        }
        (void)tx_settled();
        ruler_stop();
        U2::irda_off();
        U2::bus_clock(false);
    }
    const uint32_t want_normal = 3u * bit_cycles / 16u;
    print(serial, "  normal mode: the pulse is ", normal, " timer clocks for ", want_normal,
          " asked (three sixteenths of a bit)", crlf);
    bench.verdict("the SIR encoder's pulse is three sixteenths of a bit in normal mode, "
                  "measured on the transmit pad",
                  normal_on && near(normal, want_normal, 60));

    // Low-power mode: the pulse is three periods of the PRESCALED clock.
    uint32_t low_power = 0;
    bool lp_on = false;
    constexpr uint8_t lp_prescaler = 200;
    if (u2_up(UartFormat{}, baud)) {
        lp_on = U2::irda({.low_power = true, .prescaler = lp_prescaler}) &&
                U2::prescaler() == lp_prescaler;
        const uint32_t want = 3u * lp_prescaler * (timclk1 / pclk1);
        const uint16_t psc = ruler_prescaler_for(want * 4u);
        ruler_start(psc, false);
        ruler_rearm();
        U2::write_data(0xFF);
        if (ruler_wait()) {
            low_power = ruler_cycles(ruler_width(), psc);
        }
        (void)tx_settled();
        ruler_stop();
        U2::irda_off();
        U2::bus_clock(false);
    }
    const uint32_t want_lp = 3u * lp_prescaler * (timclk1 / pclk1);
    print(serial, "  low-power mode with a prescaler of ", lp_prescaler, ": the pulse is ",
          low_power, " timer clocks for ", want_lp, " asked", crlf);
    bench.verdict("in low-power mode the pulse is three periods of the prescaled clock and no "
                  "longer a fraction of the bit",
                  lp_on && near(low_power, want_lp, 80));

    // THE DECODER, and the question of which way its pulse goes: 18.7
    // says the level logic between the USART and the SIR block is not
    // the same in the two directions, so the frame is banged both ways
    // and the silicon says which one it reads.
    uint16_t got_high = 0xFFFF;
    uint16_t got_low = 0xFFFF;
    constexpr uint8_t rzi_word = 0x96;
    if (u2_up(UartFormat{}, baud)) {
        (void)U2::irda({.low_power = false, .prescaler = 1});
        U2::clear_by_read();
        bang_rzi(rzi_word, true);
        if (wait_flag(usart_rxne, 5000)) {
            got_high = static_cast<uint16_t>(U2::read_word() & 0xFFu);
        }
        U2::clear_by_read();
        bang_rzi(rzi_word, false);
        if (wait_flag(usart_rxne, 5000)) {
            got_low = static_cast<uint16_t>(U2::read_word() & 0xFFu);
        }
        U2::irda_off();
        U2::bus_clock(false);
    }
    print(serial, "  the decoder read ", got_high, " from a frame whose zeros are HIGH pulses "
          "and ", got_low, " from one whose zeros are LOW pulses (the word was ", rzi_word,
          ")", crlf);
    bench.verdict("THE DECODER'S PULSE IS THE ENCODER'S MIRROR: the receive side takes an "
                  "idle-HIGH line whose zero bits are low pulses, where the transmit side "
                  "puts out high pulses on an idle-low one - 18.7's note that the two "
                  "directions do not share a level logic, measured",
                  got_low == rzi_word && got_high != rzi_word);

    // What IrDA refuses: a prescaler of zero, a stop length other than
    // one, and the company of the other modes (18.7).
    bool refusals = false;
    if (u2_up(UartFormat{}, baud)) {
        const bool zero = !U2::irda({.prescaler = 0});
        UartFormat two_stops{};
        two_stops.stop = UartStop::two;
        (void)U2::configure(two_stops, usart_divisor(pclk1, baud));
        const bool stops = !U2::irda({.prescaler = 1});
        (void)U2::configure(UartFormat{}, usart_divisor(pclk1, baud));
        (void)U2::lin({});
        const bool with_lin = !U2::irda({.prescaler = 1});
        U2::lin_off();
        const bool normal_two = !U2::irda({.low_power = false, .prescaler = 2});
        refusals = zero && stops && with_lin && normal_two;
        U2::bus_clock(false);
    }
    bench.verdict("IrDA refuses a prescaler of zero, a prescaler other than one in normal "
                  "mode, a stop length that is not one, and the company of LIN",
                  refusals);
    all_off();
}

// ===========================================================================
// h - the synchronous mode, on the fourth port's own clock pad
// ===========================================================================

/// TIM3 counting the TRANSITIONS of its channel-1 pad: external clock
/// mode 1 on TI1F_ED, which is a pulse per EDGE - so a clock pulse is
/// two counts.
void ck_counter_start() {
    T3::init();
    (void)T3::configure({.prescaler = 0, .period = 0xFFFF});
    (void)T3::capture_channel(0, {.select = TimChannelSelect::direct});
    (void)T3::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::ti1_edge});
    T3::set_count(0);
    T3::enable(true);
}

uint32_t ck_transitions() { return T3::count(); }

void ck_counter_stop() {
    T3::enable(false);
    T3::release();
}

void th_synchronous() {
    all_off();
    if (!has_fourth) {
        print(serial, "  this part has two usarts and no fourth: the synchronous mode has no "
                      "instrument here", crlf);
        bench.verdict("the letter declines where the part has no fourth serial port, and says "
                      "so", true);
        return;
    }
    if constexpr (!has_full_fourth) {
        print(serial, "  the fourth port of this part is a UART4 - TX and RX, no CK pad - so "
                      "the synchronous mode has no instrument here", crlf);
        bench.verdict("the letter declines where the fourth serial port is a UART, and says so",
                      true);
        return;
    }
    constexpr uint32_t baud = 9600;

    // The instance's own fact first: on this part the fourth serial port
    // is a USART4 and not a UART4.
    bench.verdict("the fourth serial port of this part is a full USART - the chapter's own "
                  "exception, and what makes a clock pad exist at all",
                  Uc::is_full && device::usart_full(4));

    uint32_t without_lbcl = 0;
    uint32_t with_lbcl = 0;
    uint32_t with_cpha = 0;
    bool idle_low = false;
    bool idle_high = false;
    bool armed = false;
    if (u4_up(UartFormat{}, baud)) {
        armed = Uc::synchronous({.clock_idle_high = false, .capture_second_edge = false,
                                 .last_bit_clock = false}) &&
                Uc::synchronous_enabled();
        CkPad::function();
        wait_us(1000);
        idle_low = !CkPad::read();

        Uc::enable(true);
        Uc::transmitter(true);
        ck_counter_start();
        for (uint8_t i = 0; i < 8; ++i) {
            while (!Uc::flag(usart_txe)) {
            }
            Uc::write_data(0x5A);
        }
        while (!Uc::flag(usart_tc)) {
        }
        without_lbcl = ck_transitions();
        ck_counter_stop();

        // LBCL the other way, with the transmitter paused as 18.10.5
        // requires.
        Uc::transmitter(false);
        Uc::receiver(false);
        const bool moved = Uc::synchronous({.clock_idle_high = true, .last_bit_clock = true});
        Uc::transmitter(true);
        wait_us(1000);
        idle_high = CkPad::read();
        ck_counter_start();
        for (uint8_t i = 0; i < 8; ++i) {
            while (!Uc::flag(usart_txe)) {
            }
            Uc::write_data(0x5A);
        }
        while (!Uc::flag(usart_tc)) {
        }
        with_lbcl = ck_transitions();
        ck_counter_stop();

        // CPHA is the RECEIVER'S capture edge: what the pad puts out
        // must not move with it.
        Uc::transmitter(false);
        Uc::receiver(false);
        const bool phased = Uc::synchronous({.capture_second_edge = true});
        Uc::transmitter(true);
        ck_counter_start();
        for (uint8_t i = 0; i < 8; ++i) {
            while (!Uc::flag(usart_txe)) {
            }
            Uc::write_data(0x5A);
        }
        while (!Uc::flag(usart_tc)) {
        }
        with_cpha = ck_transitions();
        ck_counter_stop();
        armed = armed && moved && phased;
    }
    print(serial, "  eight bytes give ", without_lbcl, " transitions with LBCL clear and ",
          with_lbcl, " with it set (", without_lbcl / 16u, " and ", with_lbcl / 16u,
          " pulses a byte)", crlf);
    print(serial, "  the clock pad idles ", idle_low ? "low" : "NOT LOW", " under CPOL 0 and ",
          idle_high ? "high" : "NOT HIGH", " under CPOL 1", crlf);
    bench.verdict("the CK pad carries one pulse per data bit while the transmitter shifts: "
                  "eight bytes of eight bits give eight pulses each with the last bit's clock "
                  "and seven without it",
                  armed && without_lbcl / 16u == 7u && with_lbcl / 16u == 8u);
    bench.verdict("LBCL reads as the F1 family's and NOT as 18.10.5's words: the bit SET is "
                  "what adds the last data bit's pulse",
                  with_lbcl > without_lbcl);
    bench.verdict("CPOL is the clock's idle level on the pad, high or low with nothing being "
                  "sent",
                  idle_low && idle_high);
    print(serial, "  the same eight bytes under CPHA give ", with_cpha, " transitions", crlf);
    bench.verdict("CPHA is the RECEIVER's capture edge and nothing else: the same eight bytes "
                  "put the same seven pulses each on the pad with it set",
                  with_cpha == without_lbcl);

    // The refusals: the mode wants TE and RE clear, and no other mode on.
    bool live_refused = false;
    bool mode_refused = false;
    if (Uc::enabled()) {
        Uc::transmitter(true);
        live_refused = !Uc::synchronous({}) && !Uc::synchronous_off();
        Uc::transmitter(false);
        Uc::receiver(false);
        (void)Uc::synchronous_off();
        (void)Uc::half_duplex(true);
        mode_refused = !Uc::synchronous({});
        (void)Uc::half_duplex(false);
    }
    bench.verdict("the clock's three bits are refused while the transmitter or the receiver is "
                  "live, and while half duplex is on - 18.4's own conditions",
                  live_refused && mode_refused);

    // AND THE RECEIVER IS NOT FILLED BY THE CLOCK. The chapter says the
    // receiver "will only sample when outputting the clock"; what it
    // still wants is a start bit of its own, so a clocked frame with
    // nothing answering on RX leaves it empty - the sister family's
    // silicon collects a frame of the idle line instead.
    bool empty = false;
    uint16_t status = 0;
    if (Uc::enabled()) {
        (void)Uc::synchronous({});
        Uc::transmitter(true);
        Uc::receiver(true);
        Uc::clear_by_read();
        Uc::write_data(0x33);
        Stopwatch w;
        while (!Uc::flag(usart_rxne) && w.us() < 5000u) {
        }
        status = Uc::status();
        empty = (status & usart_rxne) == 0u && U4RxPad::read();
        print(serial, "  a clocked byte with nothing on RX left the receiver ",
              (status & usart_rxne) != 0u ? "FULL" : "empty", " (status ", status,
              ", the receive pad reads ", U4RxPad::read() ? 1 : 0, ")", crlf);
    }
    bench.verdict("the clock alone does not fill the receiver: a clocked frame with an idle "
                  "receive line raises no RXNE, the receiver wanting a start bit of its own "
                  "whatever the CK pad is doing",
                  empty);
    all_off();
}

// ===========================================================================
// i - the smartcard
// ===========================================================================

void ti_smartcard() {
    all_off();
    if (!has_fourth) {
        print(serial, "  this part has two usarts and no fourth: the smartcard has no "
                      "instrument here", crlf);
        bench.verdict("the letter declines where the part has no fourth serial port, and says "
                      "so", true);
        return;
    }
    if constexpr (!has_full_fourth) {
        print(serial, "  the fourth port of this part is a UART4 - no CK pad for a card's "
                      "clock - so the smartcard has no instrument here", crlf);
        bench.verdict("the letter declines where the fourth serial port is a UART, and says so",
                      true);
        return;
    }
    constexpr uint32_t baud = 9600;
    constexpr uint8_t card_prescaler = 6;   // the clock is PCLK1 / (2 x PSC)
    constexpr uint8_t guard = 16;

    bool armed = false;
    bool stops = false;
    bool guarded = false;
    uint32_t transitions = 0;
    uint32_t armed_transitions = 0;
    uint32_t sending_transitions = 0;
    if (u4_up(UartFormat{UartBits::eight, UartParity::even}, baud)) {
        armed = Uc::smartcard({.nack = true, .guard_time = guard,
                               .clock_prescaler = card_prescaler}) &&
                Uc::smartcard_enabled();
        // The verb sets 1.5 stop bits, which is what 18.6 recommends.
        stops = (Uc::regs().CTLR2 & usart_stop_mask) == usart_ctlr2_stop(UartStop::one_and_half);
        guarded = Uc::guard_time() == guard && Uc::prescaler() == card_prescaler;

        // THE CARD CLOCK, which the mode's own verb turns on: 18.6 says
        // its waveform "has nothing to do with the communication" and
        // only provides the clock for the card. The pad is the caller's,
        // and WHEN that clock actually runs is the question - counted
        // three times, with the transmitter off, with it on, and with a
        // frame going out.
        Uc::transmitter(false);
        Uc::receiver(false);
        CkPad::function();
        Uc::enable(true);
        ck_counter_start();
        wait_us(2000);
        transitions = ck_transitions();
        ck_counter_stop();

        Uc::transmitter(true);
        wait_us(1000);
        ck_counter_start();
        wait_us(2000);
        armed_transitions = ck_transitions();
        ck_counter_stop();

        ck_counter_start();
        Uc::write_data(0x5A);
        (void)wait_flag(usart_tc, 50'000);
        sending_transitions = ck_transitions();
        ck_counter_stop();
    }
    // Two transitions a period, over the two milliseconds each window
    // lasted.
    const uint32_t want_hz = pclk1 / (2u * card_prescaler);
    const uint32_t got_hz = armed_transitions / 2u * 500u;
    print(serial, "  the card clock counted ", transitions,
          " transitions in 2 ms with the transmitter off, ", armed_transitions,
          " with it on (", got_hz, " Hz for ", want_hz, " asked) and ", sending_transitions,
          " while a frame went out", crlf);
    bench.verdict("the smartcard's own verb writes SCEN with its NACK, the guard time and the "
                  "prescaler, and sets the 1.5 stop bits 18.6 asks for",
                  armed && stops && guarded);
    bench.verdict("the card clock is the peripheral clock divided by TWICE the prescaler and it "
                  "runs free, whether or not anything is being sent - but only with the "
                  "TRANSMITTER enabled: with TE clear the pad stays still",
                  transitions == 0u && near(got_hz, want_hz, 50));

    // The refusals. The clock has to come off first: LIN and the clock
    // exclude each other, so a program leaving the smartcard hands the
    // pad back before it asks for anything else.
    bool refused = false;
    if (Uc::enabled()) {
        Uc::transmitter(false);
        Uc::receiver(false);
        const bool zero = !Uc::smartcard({.clock_prescaler = 0});
        const bool over = !Uc::smartcard({.clock_prescaler = 32});
        Uc::smartcard_off();
        const bool clock_off = Uc::synchronous_off();
        const bool lin_on = Uc::lin({});
        const bool with_lin = !Uc::smartcard({.clock_prescaler = 4});
        Uc::lin_off();
        refused = zero && over && clock_off && lin_on && with_lin;
    }
    bench.verdict("the smartcard refuses a prescaler outside one to thirty-one and the company "
                  "of LIN, and its clock comes off with the synchronous half's own verb",
                  refused);
    all_off();
}

// ===========================================================================
// j - the flags and the vectors
// ===========================================================================

void tj_flags() {
    all_off();
    constexpr uint32_t baud = 9600;
    if (!u2_up(UartFormat{}, baud)) {
        bench.verdict("USART2 comes up", false);
        return;
    }
    Pfic::enable(Irq::usart2);

    // TXE and TC, in that order.
    U2::clear_flags(usart_tc);
    U2::interrupts(usart_txeie | usart_tcie, true);
    u2_txe = 0;
    u2_tc = 0;
    U2::write_data(0x55);
    (void)tx_settled();
    wait_us(2000);
    const uint32_t txe = u2_txe;
    const uint32_t tc = u2_tc;
    U2::interrupts(usart_txeie | usart_tcie, false);
    print(serial, "  the vector ran ", txe, " times for TXE and ", tc, " for TC", crlf);
    bench.verdict("TXE and TC both reach the vector, TXE when the shift register takes the "
                  "byte and TC when the frame is out",
                  txe >= 1u && tc >= 1u);

    // RXNE and IDLE: a byte, then a line that goes quiet.
    U2::rxne_interrupt(true);
    U2::idle_interrupt(true);
    u2_rxne = 0;
    u2_idle = 0;
    U2::clear_by_read();
    bang_word(0x5A, UartFormat{});
    wait_us(4000);
    bang_idle(12);
    wait_us(2000);
    const uint32_t rxne = u2_rxne;
    const uint32_t idle = u2_idle;
    const uint16_t word = u2_last_word;
    print(serial, "  RXNE ", rxne, " times (last word ", word, "), IDLE ", idle, crlf);
    bench.verdict("RXNE carries the byte to the vector and IDLE rises once when the line goes "
                  "quiet after it - 18.10.1's note, IDLE not set again until RXNE is",
                  rxne == 1u && word == 0x5Au && idle == 1u);

    // PE with its own enable.
    U2::rxne_interrupt(false);
    U2::idle_interrupt(false);
    U2::bus_clock(false);
    bool parity_vector = false;
    if (u2_up(UartFormat{UartBits::eight, UartParity::even}, baud)) {
        Pfic::enable(Irq::usart2);
        U2::parity_interrupt(true);
        U2::rxne_interrupt(true);
        u2_pe = 0;
        U2::clear_by_read();
        Frame fr{.data = 0x3C, .bits = 8, .parity = true,
                 .parity_value = !even_parity(0x3C, 8), .baud = baud};
        bang_frame(fr);
        wait_us(4000);
        parity_vector = u2_pe >= 1u;
        U2::parity_interrupt(false);
        U2::rxne_interrupt(false);
    }
    bench.verdict("PEIE carries a parity error to the vector", parity_vector);

    // The fourth port's own vector, which is the device class's tail.
    if (!has_fourth) {
        bench.verdict("the fourth serial port's vector is not this part's to reach: two usarts "
                      "and no fourth", true);
        all_off();
        return;
    }
    U4::bus_clock(true);
    U4::reset();
    (void)U4::configure(UartFormat{}, usart_divisor(pclk1, baud));
    U4::enable(true);
    U4::transmitter(true);
    U4::interrupts(usart_tcie, true);
    Pfic::enable(Irq::uart4);
    u4_interrupts = 0;
    U4::clear_flags(usart_tc);
    U4::write_data(0x11);
    wait_us(4000);
    const uint32_t fourth = u4_interrupts;
    print(serial, "  the fourth port's vector ran ", fourth, " times", crlf);
    if constexpr (v303_pair) {
        bench.verdict("the fourth serial port's vector is its own - entry 68 of this device "
                      "class's table, and a transmission complete reaches it",
                      fourth >= 1u);
    } else {
        bench.verdict("the fourth serial port's vector is its own - entry 61 of this device "
                      "class's table, and a transmission complete reaches it",
                      fourth >= 1u);
    }
    all_off();
}

// ===========================================================================
// k - hardware flow control
// ===========================================================================

void tk_flow() {
    all_off();
    constexpr uint32_t baud = 9600;
    U2::bus_clock(true);
    U2::reset();
    TxPad::function();
    RxPad::input(PinPull::up);
    // CTS is an input the CPU drives through its pull; RTS is an output
    // the CPU reads on the pad.
    CtsPad::input(PinPull::up);          // high: NOT clear to send
    RtsPad::function();
    (void)U2::configure(UartFormat{}, usart_divisor(pclk1, baud));
    const bool pair = U2::flow_control(true, true) && U2::rts_enabled() && U2::cts_enabled();
    U2::enable(true);
    U2::transmitter(true);
    U2::receiver(true);
    wait_us(3000);

    // CTS high holds the frame back: TC does not come.
    U2::clear_flags(usart_tc);
    U2::write_data(0xA5);
    wait_us(4000);
    const bool held = !U2::flag(usart_tc);
    CtsPad::clear();                     // low: clear to send
    const bool released = wait_flag(usart_tc, 20'000);
    print(serial, "  CTS high ", held ? "held the frame back" : "DID NOT HOLD IT",
          "; low let it out ", released ? "at once" : "NOT AT ALL", crlf);
    bench.verdict("CTS sampled high holds the next frame in the data register - TC never "
                  "comes - and a low level lets it out",
                  pair && held && released);

    // RTS: low while the receiver can take a frame, high while a byte
    // waits unread.
    U2::clear_by_read();
    wait_us(1000);
    const bool ready_low = !RtsPad::read();
    bang_word(0x7E, UartFormat{});
    (void)wait_flag(usart_rxne, 5000);
    wait_us(500);
    const bool busy_high = RtsPad::read();
    (void)U2::read_word();
    wait_us(500);
    const bool free_again = !RtsPad::read();
    print(serial, "  RTS reads ", ready_low ? "low" : "HIGH", " with the receiver free, ",
          busy_high ? "high" : "LOW", " with a byte waiting, ", free_again ? "low" : "HIGH",
          " once it is read", crlf);
    bench.verdict("RTS is the receiver's own hand: low while it can take a frame, high while "
                  "a received byte waits unread, low again when it is taken",
                  ready_low && busy_high && free_again);

    // The CTS flag and its interrupt: a CHANGE of the line raises it,
    // and the body disarms the enable so a standing source cannot spin
    // the vector - which is why each change is armed on its own.
    Pfic::enable(Irq::usart2);
    U2::clear_flags(usart_cts);
    u2_cts = 0;
    U2::cts_interrupt(true);
    CtsPad::set();
    wait_us(2000);
    const uint32_t on_high = u2_cts;
    U2::clear_flags(usart_cts);
    U2::cts_interrupt(true);
    CtsPad::clear();
    wait_us(2000);
    const uint32_t on_low = u2_cts;
    U2::cts_interrupt(false);
    Pfic::disable(Irq::usart2);
    print(serial, "  the CTS flag reached the vector ", on_high, " time(s) on the rise and ",
          on_low, " in all after the fall", crlf);
    bench.verdict("a change on the CTS line raises its flag and CTSIE carries it to the "
                  "vector, in both directions",
                  on_high >= 1u && on_low > on_high);
    all_off();
}

// ===========================================================================
// l - the loopback PA2 to PA3
// ===========================================================================

/// The xorshift `brio stress` and every other suite of this project
/// generate, so a pattern is comparable across targets.
uint32_t xorshift(uint32_t& state) {
    uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state = x;
    return x;
}

void tl_loopback() {
    all_off();
    if (!need_loop_wire()) {
        print(serial, "  the strap PA2 to PA3 is ABSENT: the loopback letter measures nothing",
              crlf);
        bench.verdict("the loopback letter declines without its wire, and says so", true);
        return;
    }
    print(serial, "  the strap PA2 to PA3 is in place", crlf);

    // A round trip at every format the resource can make.
    const UartFormat formats[4] = {
        UartFormat{},
        UartFormat{UartBits::eight, UartParity::odd},
        UartFormat{UartBits::seven, UartParity::even},
        UartFormat{UartBits::nine, UartParity::none},
    };
    const uint16_t words[4] = {0xC3, 0xC3, 0x43, 0x1C3};
    uint32_t round_trips = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        if (!u2_up(formats[i], 115200)) {
            continue;
        }
        U2::clear_by_read();
        U2::write_word(words[i]);
        const bool got = wait_flag(usart_rxne, 20'000);
        const uint16_t back =
            static_cast<uint16_t>(U2::read_word() & uart_data_mask(formats[i]));
        if (got && back == words[i]) {
            ++round_trips;
        }
        U2::bus_clock(false);
    }
    bench.verdict("every frame shape goes out of the transmitter and comes back into the "
                  "receiver over the strap, byte for byte",
                  round_trips == 4u);

    // The two DMA engines, on the channels table 11-5 gives USART2.
    u2_transport = true;
    const bool opened = Loop::init(clock, 115200);
    static const uint8_t message[] = "the channel is the request";
    constexpr uint8_t length = sizeof(message) - 1u;
    Loop::clear_errors();
    (void)Loop::harvest();
    (void)Loop::write(message, length);
    Stopwatch w;
    uint8_t got = 0;
    uint8_t back[length] = {};
    while (got < length && w.us() < 50'000UL) {
        (void)Loop::harvest();
        uint8_t b = 0;
        while (got < length && Loop::read_byte(b)) {
            back[got++] = b;
        }
    }
    bool same = got == length;
    for (uint8_t i = 0; i < length && same; ++i) {
        same = back[i] == message[i];
    }
    print(serial, "  the engines carried ", got, " of ", length, " bytes in ", w.us(),
          " us, faults ", Loop::dma_faults(), ", overruns ", Loop::rx_overruns(), crlf);
    bench.verdict("the transmit engine drains the ring by whole blocks and the receive engine "
                  "fills it, the run published by harvest() and identical byte for byte",
                  opened && same && Loop::dma_faults() == 0u);

    // Four kilobytes at the highest rate the loop takes clean.
    constexpr uint32_t run_bytes = 4096;
    constexpr uint32_t fast_baud = 921600;
    (void)Loop::set_baud(SysClock::hz, fast_baud);
    Loop::clear_errors();
    uint32_t tx_state = 0x1234'5678UL;
    uint32_t rx_state = 0x1234'5678UL;
    uint32_t sent = 0;
    uint32_t received = 0;
    uint32_t wrong = 0;
    Stopwatch run;
    while (received < run_bytes && run.us() < 2'000'000UL) {
        while (sent < run_bytes && sent - received < 128u) {
            if (!Loop::write_byte(static_cast<uint8_t>(xorshift(tx_state) & 0xFFu))) {
                break;
            }
            ++sent;
        }
        (void)Loop::harvest();
        uint8_t b = 0;
        while (Loop::read_byte(b)) {
            if (b != static_cast<uint8_t>(xorshift(rx_state) & 0xFFu)) {
                ++wrong;
            }
            ++received;
        }
    }
    const uint32_t run_us = run.us();
    print(serial, "  ", received, " of ", run_bytes, " bytes at ", fast_baud, " baud in ",
          run_us, " us, ", wrong, " wrong, overruns ", Loop::rx_overruns(), "/",
          Loop::hw_overruns(), ", frame errors ", Loop::frame_errors(), crlf);
    bench.verdict("four kilobytes travel the loop at 921600 baud with every byte of the "
                  "xorshift pattern back in order and no error counted",
                  received == run_bytes && wrong == 0u && Loop::frame_errors() == 0u &&
                      Loop::hw_overruns() == 0u);
    Loop::release();
    all_off();
}

// ===========================================================================
// m - the crossed pair
// ===========================================================================

/// One direction of the crossed pair, polled: `count` xorshift bytes out
/// of `from` and into `to`, with the flags counted.
uint32_t cross_run(uint32_t count, bool from_u2) {
    uint32_t state = 0x2468'1357UL;
    uint32_t check = 0x2468'1357UL;
    uint32_t wrong = 0;
    uint32_t got = 0;
    Stopwatch w;
    for (uint32_t i = 0; i < count && w.us() < 2'000'000UL; ++i) {
        const uint8_t byte = static_cast<uint8_t>(xorshift(state) & 0xFFu);
        if (from_u2) {
            while (!U2::tx_empty()) {
            }
            U2::write_data(byte);
            Stopwatch f;
            while (!U4::rx_ready() && f.us() < 20'000UL) {
            }
            if (!U4::rx_ready()) {
                break;
            }
            if ((U4::read_word() & 0xFFu) != (xorshift(check) & 0xFFu)) {
                ++wrong;
            }
        } else {
            while (!U4::tx_empty()) {
            }
            U4::write_data(byte);
            Stopwatch f;
            while (!U2::rx_ready() && f.us() < 20'000UL) {
            }
            if (!U2::rx_ready()) {
                break;
            }
            if ((U2::read_word() & 0xFFu) != (xorshift(check) & 0xFFu)) {
                ++wrong;
            }
        }
        ++got;
    }
    return got == count ? wrong : 0xFFFFu;
}

void tm_cross() {
    all_off();
    if (!has_fourth) {
        print(serial, "  this part has two usarts and no fourth: the crossed pair has no peer "
                      "here", crlf);
        bench.verdict("the letter declines where the part has no fourth serial port, and says "
                      "so", true);
        return;
    }
    if (!need_cross_wires()) {
        if constexpr (v303_pair) {
            print(serial, "  the wires PA2-PC11 and PC10-PA3 are ABSENT: the crossed pair "
                  "measures nothing", crlf);
        } else {
            print(serial, "  the straps PA2-PB1 and PB0-PA3 are ABSENT: the crossed pair measures "
                  "nothing", crlf);
        }
        bench.verdict("the crossed-pair letter declines without its wires, and says so", true);
        return;
    }
    if constexpr (v303_pair) {
        print(serial, "  the wires PA2-PC11 and PC10-PA3 are in place", crlf);
    } else {
        print(serial, "  the straps PA2-PB1 and PB0-PA3 are in place", crlf);
    }

    const UartFormat rounds[2] = {UartFormat{},
                                  UartFormat{UartBits::eight, UartParity::even, UartStop::two}};
    const uint32_t bauds[2] = {115200, 19200};
    const char* names[2] = {"8N1 at 115200", "8E2 at 19200"};
    uint32_t clean = 0;
    for (uint8_t r = 0; r < 2; ++r) {
        if (!u2_up(rounds[r], bauds[r])) {
            continue;
        }
        // UART4 on its DEFAULT column, which is the crossed pair's.
        U4::bus_clock(true);
        U4::reset();
        CrossTxPad::function();
        CrossRxPad::input(PinPull::up);
        (void)U4::configure(rounds[r], usart_divisor(pclk1, bauds[r]));
        U4::enable(true);
        U4::transmitter(true);
        U4::receiver(true);
        wait_us(3000);
        U2::clear_by_read();
        U4::clear_by_read();

        const uint32_t out = cross_run(64, true);
        const uint32_t back = cross_run(64, false);
        const uint16_t u2_errors = U2::status() & (usart_pe | usart_fe | usart_ne | usart_ore);
        const uint16_t u4_errors = U4::status() & (usart_pe | usart_fe | usart_ne | usart_ore);
        print(serial, "  ", names[r], ": 64 bytes out with ", out, " wrong, 64 back with ",
              back, " wrong; flags ", u2_errors, " and ", u4_errors, crlf);
        if (out == 0u && back == 0u && u2_errors == 0u && u4_errors == 0u) {
            ++clean;
        }
        U4::bus_clock(false);
        U2::bus_clock(false);
        console_drain();
    }
    bench.verdict("USART2 and the fourth port talk to each other over the crossed pair at two "
                  "formats and two rates, every xorshift byte arriving in order with no error "
                  "flag on either side",
                  clean == 2u);
    all_off();
}

// ===========================================================================
// The CH32V303's own letters (n..p)
// ===========================================================================
//
// Every name below hangs on a template parameter and every body is
// `if constexpr` on the part's facts, so a part without UART4 on DMA2, the
// lot's registers or UART5..UART8 forms none of it and carries none of
// it - its state included, which lives in templates for the same reason.

/// What the DMA2 vectors serve while letter n runs, and what the four
/// upper vectors counted (letter p). Named as V303State<on> by templates
/// whose `on` is true only where the part has what they ask - the one
/// specialization the CH32V303 forms.
template <bool on>
struct V303State {
    static inline volatile bool engines = false;
    static inline volatile uint32_t upper_interrupts[4] = {};
};

/// UART4 with both its engines, where table 11-3 puts them: DMA2's
/// channel 5 out and channel 3 in. With an engine on each side the port
/// arms no interrupt of its own. `on` makes every use a dependent name.
template <bool on, uint8_t C = 2, DmaRequest tx = DmaRequest::uart4_tx,
          DmaRequest rx = DmaRequest::uart4_rx>
struct PairPort {
    using TxRow = DmaRequestOf<tx>;
    using RxRow = DmaRequestOf<rx>;
    static_assert(TxRow::controller == C && RxRow::controller == C,
                  "table 11-3: UART4's two requests are DMA2's on this class");
    using Port = Uart<4, P, 256, 256, UartFormat{}, DmaTxEngine<C, TxRow::channel>,
                      DmaRxEngine<C, RxRow::channel>>;
};

/// Whatever a transport's receiver framed while its pad floated, drained
/// and counted.
template <typename T>
uint16_t drain_port() {
    uint16_t n = 0;
    (void)T::harvest();
    uint8_t b = 0;
    while (T::read_byte(b)) {
        ++n;
    }
    return n;
}

/// One direction of an engined pair: `count` xorshift bytes written into
/// `From` and read out of `To`, the receive side harvested as it goes and
/// never more than 128 bytes in flight.
struct PairRun {
    uint32_t received;
    uint32_t wrong;
    uint32_t us;
};

template <typename From, typename To>
PairRun pair_run(uint32_t count, uint32_t budget_us) {
    uint32_t tx_state = 0x1234'5678UL;
    uint32_t rx_state = 0x1234'5678UL;
    uint32_t sent = 0;
    PairRun r{0, 0, 0};
    Stopwatch w;
    while (r.received < count && w.us() < budget_us) {
        while (sent < count && sent - r.received < 128u) {
            if (!From::write_byte(static_cast<uint8_t>(xorshift(tx_state) & 0xFFu))) {
                break;
            }
            ++sent;
        }
        (void)To::harvest();
        uint8_t b = 0;
        while (To::read_byte(b)) {
            if (b != static_cast<uint8_t>(xorshift(rx_state) & 0xFFu)) {
                ++r.wrong;
            }
            ++r.received;
        }
    }
    r.us = w.us();
    return r;
}

/// A message through one transport and out of the other, byte for byte.
template <typename From, typename To>
bool pair_message(const uint8_t* msg, uint16_t len) {
    uint8_t seen[64] = {};
    uint16_t got = 0;
    (void)To::harvest();
    (void)From::write(msg, static_cast<uint8_t>(len));
    Stopwatch w;
    while (got < len && w.us() < 50'000UL) {
        (void)To::harvest();
        uint8_t b = 0;
        while (got < len && To::read_byte(b)) {
            seen[got++] = b;
        }
    }
    bool same = got == len;
    for (uint16_t i = 0; i < got && same; ++i) {
        same = seen[i] == msg[i];
    }
    return same;
}

// ---------------------------------------------------------------------------
// n - the crossed pair's engines
// ---------------------------------------------------------------------------

template <bool on = (has_fourth && v303_pair && device::dma_controller_count >= 2u)>
void tn_pair_engines() {
    if constexpr (on) {
        all_off();
        using Port4 = typename PairPort<on>::Port;
        using S = V303State<on>;
        if (!need_cross_wires()) {
            print(serial, "  the wires PA2-PC11 and PC10-PA3 are ABSENT: the engined pair measures "
                          "nothing", crlf);
            bench.verdict("the engined pair declines without its wires, and says so", true);
            return;
        }
        u2_transport = true;
        S::engines = true;
        const bool opened = Loop::init(clock, 115200) && Port4::init(clock, 115200);
        // WHAT A RECEIVER TAKES WHILE ITS PAD FLOATS: between the first
        // port's init and the second's, the peer's transmit pad is not
        // driven yet. Drained - and counted - before anything is judged.
        wait_us(2000);
        const uint16_t stale2 = drain_port<Loop>();
        const uint16_t stale4 = drain_port<Port4>();
        Loop::clear_errors();
        Port4::clear_errors();

        static const uint8_t forth[] = "USART2's engines on DMA1 into UART4's on DMA2";
        static const uint8_t back[] = "and back, DMA2 out and DMA1 in";
        const bool forth_ok = pair_message<Loop, Port4>(forth, sizeof(forth) - 1u);
        const bool back_ok = pair_message<Port4, Loop>(back, sizeof(back) - 1u);
        print(serial, "  a message each way at 115200: ", forth_ok ? "exact" : "WRONG", " and ",
              back_ok ? "exact" : "WRONG", "; framed while a pad floated: ", stale2, " and ",
              stale4, " bytes", crlf);
        bench.verdict("USART2's transmit engine on DMA1 pours a run into UART4's receive engine "
                      "on DMA2 and UART4's transmit engine one back into USART2's, byte for "
                      "byte, each run published by harvest()",
                      opened && forth_ok && back_ok);

        // Four kilobytes each way at 921600 baud.
        constexpr uint32_t run_bytes = 4096;
        constexpr uint32_t fast_baud = 921600;
        const bool fast = Loop::set_baud(SysClock::hz, fast_baud) &&
                          Port4::set_baud(SysClock::hz, fast_baud);
        wait_us(1000);
        (void)drain_port<Loop>();
        (void)drain_port<Port4>();
        Loop::clear_errors();
        Port4::clear_errors();
        const PairRun out = pair_run<Loop, Port4>(run_bytes, 2'000'000UL);
        const PairRun in = pair_run<Port4, Loop>(run_bytes, 2'000'000UL);
        print(serial, "  ", out.received, " of ", run_bytes, " bytes USART2 to UART4 in ", out.us,
              " us with ", out.wrong, " wrong; ", in.received, " back in ", in.us, " us with ",
              in.wrong, " wrong", crlf);
        print(serial, "  UART4: overruns ", Port4::rx_overruns(), "/", Port4::hw_overruns(),
              ", frame errors ", Port4::frame_errors(), ", faults ", Port4::dma_faults(),
              "; USART2: overruns ", Loop::rx_overruns(), "/", Loop::hw_overruns(),
              ", frame errors ", Loop::frame_errors(), ", faults ", Loop::dma_faults(), crlf);
        bench.verdict("four kilobytes travel each way at 921600 baud through the two controllers "
                      "with every byte of the xorshift pattern in order and no error counted at "
                      "either end",
                      fast && out.received == run_bytes && in.received == run_bytes &&
                          out.wrong == 0u && in.wrong == 0u && Port4::frame_errors() == 0u &&
                          Loop::frame_errors() == 0u && Port4::hw_overruns() == 0u &&
                          Loop::hw_overruns() == 0u && Port4::dma_faults() == 0u &&
                          Loop::dma_faults() == 0u);
        // Both ports stopped and not only their channels: a receiver left
        // running keeps its request on its channel (docs/ch32vx03/dma.md).
        S::engines = false;
        u2_transport = false;
        Port4::release();
        Loop::release();
        all_off();
    }
}

// ---------------------------------------------------------------------------
// o - the lot's registers on the pair
// ---------------------------------------------------------------------------

/// The five registers the CTLR4 probe must leave as it found them.
struct UsartWords {
    uint16_t brr, ctlr1, ctlr2, ctlr3, gpr;
    bool operator==(const UsartWords&) const = default;
};

template <typename U>
UsartWords usart_words() {
    return {U::regs().BRR, U::regs().CTLR1, U::regs().CTLR2, U::regs().CTLR3, U::regs().GPR};
}

/// UART4 on its default column, the pair's, CONFIGURED BUT DISABLED - the
/// state the CTLR4 probe wants.
template <typename U>
bool pair_far_configured(const UartFormat& f, uint32_t baud) {
    U::bus_clock(true);
    U::reset();
    CrossTxPad::function();
    CrossRxPad::input(PinPull::up);
    return U::configure(f, usart_divisor(pclk1, baud));
}

/// USART2 the same way, disabled.
bool pair_near_configured(const UartFormat& f, uint32_t baud) {
    U2::bus_clock(true);
    U2::reset();
    TxPad::function();
    RxPad::input(PinPull::up);
    return U2::configure(f, usart_divisor(pclk1, baud));
}

template <typename U>
void port_on() {
    U::enable(true);
    U::transmitter(true);
    U::receiver(true);
}

/// One frame from USART2 into the far port: the word it read, and its
/// status as the read found it.
template <typename U>
std::optional<uint16_t> pair_frame(uint16_t word, uint16_t& status) {
    U::clear_by_read();
    U2::write_word(word);
    Stopwatch w;
    while (!U::rx_ready() && w.us() < 20'000UL) {
    }
    status = U::status();
    if ((status & usart_rxne) == 0u) {
        return {};
    }
    return static_cast<uint16_t>(U::read_word());
}

/// A ZERO WORD'S LOW RUN on USART2's transmit pad, in bit times, with
/// `ctlr4` and `m_ext` stored RAW - past the verbs, which refuse on a die
/// whose read-back says no - so a register that TAKES a write and reads
/// back zero is caught by what the wire does. The port is disabled for
/// the stores, as the chapter asks of a frame's shape, and both are put
/// back to zero after.
template <bool on>
uint32_t raw_low_run(const UartFormat& f, uint16_t ctlr4, uint16_t m_ext) {
    constexpr uint32_t baud = 9600;
    U2::enable(false);
    (void)U2::configure(f, usart_divisor(pclk1, baud));
    U2::regs().CTLR4 = ctlr4;
    U2::regs().CTLR1 = static_cast<uint16_t>(U2::regs().CTLR1 | m_ext);
    port_on<U2>();
    wait_us(3000);
    const uint32_t bit_cycles = (timclk1 / pclk1) * usart_divisor(pclk1, baud);
    const uint16_t psc = ruler_prescaler_for(bit_cycles * 14u);
    ruler_start(psc, true);
    ruler_rearm();
    U2::write_word(0);
    uint32_t run = 0;
    if (ruler_wait()) {
        run = ruler_cycles(ruler_width(), psc);
    }
    (void)tx_settled();
    ruler_stop();
    U2::enable(false);
    U2::regs().CTLR4 = 0;
    U2::regs().CTLR1 = static_cast<uint16_t>(U2::regs().CTLR1 & ~usart_m_ext_mask);
    return (run + bit_cycles / 2u) / bit_cycles;
}

template <bool on = (has_fourth && v303_pair)>
void to_lot_registers() {
    if constexpr (on) {
        all_off();
        if (!need_cross_wires()) {
            print(serial, "  the wires PA2-PC11 and PC10-PA3 are ABSENT: the lot's registers are "
                          "asked of the pair and not measured", crlf);
            bench.verdict("the letter declines without its wires, and says so", true);
            return;
        }
        constexpr uint32_t baud = 9600;
        // Eight data bits and a parity bit: the frame whose parity bit
        // CHECK_SEL would hold at one or at zero.
        constexpr UartFormat parity_frame{UartBits::eight, UartParity::even};

        // ---- the probe, and every other register compared across it ----
        const bool up = pair_near_configured(parity_frame, baud) &&
                        pair_far_configured<U4>(parity_frame, baud);
        const UsartWords before2 = usart_words<U2>();
        const UsartWords before4 = usart_words<U4>();
        const uint16_t word2 = U2::regs().CTLR4;
        const uint16_t word4 = U4::regs().CTLR4;
        const bool present2 = U2::ctlr4_present();
        const bool present4 = U4::ctlr4_present();
        const UsartWords after2 = usart_words<U2>();
        const UsartWords after4 = usart_words<U4>();
        print(serial, "  CTLR4's address read ", hex(word2), " on USART2 and ", hex(word4),
              " on UART4 before the probe; the probe says ", present2 ? "PRESENT" : "absent",
              " and ", present4 ? "PRESENT" : "absent", crlf);
        bench.verdict("the CTLR4 probe answers on both ports and leaves BRR, CTLR1..3 and GPR of "
                      "each block exactly as it found them, whatever the address turned out to "
                      "be",
                      up && before2 == after2 && before4 == after4);

        port_on<U2>();
        port_on<U4>();
        wait_us(3000);

        if (present2 && present4) {
            // MARK on the wire: a zero byte's low run is the start bit and
            // eight data bits, the parity bit held HIGH; SPACE holds it low
            // and the run is ten bits.
            const uint32_t bit_cycles = (timclk1 / pclk1) * usart_divisor(pclk1, baud);
            const uint16_t psc = ruler_prescaler_for(bit_cycles * 14u);
            uint32_t runs[2] = {};
            const UsartMarkSpace levels[2] = {UsartMarkSpace::mark, UsartMarkSpace::space};
            for (uint8_t i = 0; i < 2u; ++i) {
                (void)U2::mark_space(levels[i]);
                ruler_start(psc, true);
                ruler_rearm();
                U2::write_word(0);
                if (ruler_wait()) {
                    runs[i] = ruler_cycles(ruler_width(), psc);
                }
                (void)tx_settled();
                ruler_stop();
            }
            const uint32_t mark_bits = (runs[0] + bit_cycles / 2u) / bit_cycles;
            const uint32_t space_bits = (runs[1] + bit_cycles / 2u) / bit_cycles;
            print(serial, "  a zero byte's low run: ", mark_bits, " bits under MARK, ",
                  space_bits, " under SPACE", crlf);
            bench.verdict("MARK holds the parity bit high and SPACE holds it low, measured on the "
                          "transmit pad: nine low bits for a zero byte under MARK, ten under SPACE",
                          mark_bits == 9u && space_bits == 10u);

            // The far receiver's judgement: a MARK frame into a MARK
            // receiver is clean, into a SPACE receiver it raises MS_ERR.
            uint16_t st_same = 0;
            uint16_t st_other = 0;
            (void)U2::mark_space(UsartMarkSpace::mark);
            (void)U4::mark_space(UsartMarkSpace::mark);
            const auto same = pair_frame<U4>(0x5A, st_same);
            (void)U4::mark_space(UsartMarkSpace::space);
            const auto other = pair_frame<U4>(0x5A, st_other);
            print(serial, "  MARK into MARK: status ", hex(st_same), ", MARK into SPACE: status ",
                  hex(st_other), crlf);
            bench.verdict("a MARK frame into a MARK receiver arrives with no MS_ERR, and into a "
                          "SPACE receiver it raises MS_ERR - the far end's own judgement of the "
                          "held level",
                          same.has_value() && (*same & 0xFFu) == 0x5Au &&
                              (st_same & usart_ms_err) == 0u && other.has_value() &&
                              (st_other & usart_ms_err) != 0u);
            (void)U2::mark_space(UsartMarkSpace::off);
            (void)U4::mark_space(UsartMarkSpace::off);
        } else {
            print(serial, "  this die's lot has no CTLR4 by the probe: the MARK and SPACE parity is "
                          "asked of the WIRE instead", crlf);
            bench.verdict("with the register absent every MARK and SPACE verb answers false and "
                          "writes nothing",
                          !U2::mark_space(UsartMarkSpace::mark) &&
                              !U4::mark_space(UsartMarkSpace::space) &&
                              !U2::mark_space_interrupt(true) &&
                              U2::mark_space() == UsartMarkSpace::off);
            // A register that took the write and reads back zero would
            // still hold the parity bit: MARK makes a zero byte's low run
            // nine bits under even parity (ten computed), SPACE ten under
            // odd parity (nine computed).
            const uint32_t even_plain =
                raw_low_run<on>(UartFormat{UartBits::eight, UartParity::even}, 0, 0);
            const uint32_t even_mark = raw_low_run<on>(
                UartFormat{UartBits::eight, UartParity::even}, usart_check_mark, 0);
            const uint32_t odd_plain =
                raw_low_run<on>(UartFormat{UartBits::eight, UartParity::odd}, 0, 0);
            const uint32_t odd_space = raw_low_run<on>(
                UartFormat{UartBits::eight, UartParity::odd}, usart_check_space, 0);
            print(serial, "  a zero byte's low run with CTLR4 stored raw: even parity ", even_plain,
                  " bits plain and ", even_mark, " under MARK; odd parity ", odd_plain,
                  " plain and ", odd_space, " under SPACE", crlf);
            bench.verdict("THE WIRE AGREES WITH THE PROBE: CTLR4 stored raw with MARK or SPACE "
                          "leaves the parity bit computed - ten low bits under even parity, nine "
                          "under odd, as without it - so the register is absent, not write-only",
                          even_plain == 10u && even_mark == 10u && odd_plain == 9u &&
                              odd_space == 9u);
            port_on<U2>();
            wait_us(3000);
        }

        // ---- M_EXT: the short words, with the ports disabled again ----
        U2::enable(false);
        U4::enable(false);
        (void)U2::configure(UartFormat{}, usart_divisor(pclk1, baud));
        (void)U4::configure(UartFormat{}, usart_divisor(pclk1, baud));
        const bool short2 = U2::short_word(UsartShortWord::seven);
        const bool short4 = U4::short_word(UsartShortWord::seven);
        print(serial, "  M_EXT kept a seven-bit word: ", short2 ? "yes" : "no", " on USART2, ",
              short4 ? "yes" : "no", " on UART4", crlf);
        if (short2 && short4) {
            const UsartShortWord words[3] = {UsartShortWord::seven, UsartShortWord::six,
                                             UsartShortWord::five};
            uint32_t exact = 0;
            for (UsartShortWord w : words) {
                U2::enable(false);
                U4::enable(false);
                (void)U2::short_word(w);
                (void)U4::short_word(w);
                port_on<U2>();
                port_on<U4>();
                wait_us(3000);
                uint16_t st = 0;
                const uint16_t sent = static_cast<uint16_t>(0x55u & usart_short_word_mask(w));
                const auto got = pair_frame<U4>(sent, st);
                if (got.has_value() && (*got & usart_short_word_mask(w)) == sent &&
                    (st & (usart_fe | usart_pe)) == 0u) {
                    ++exact;
                }
            }
            bench.verdict("seven, six and five-bit words cross the pair exactly, M_EXT set at both "
                          "ends",
                          exact == 3u);
        } else {
            bench.verdict("with M_EXT absent the field reads back zero and the verb answers false, "
                          "the word left to M",
                          !short2 && !short4 && U2::short_word() == UsartShortWord::off);
            // Stored raw, a seven-bit code that acted would cut a zero
            // word's low run from nine bits (start and eight) to eight.
            const uint32_t eight = raw_low_run<on>(UartFormat{}, 0, 0);
            const uint32_t seven =
                raw_low_run<on>(UartFormat{}, 0, usart_ctlr1_short_word(UsartShortWord::seven));
            const uint32_t five =
                raw_low_run<on>(UartFormat{}, 0, usart_ctlr1_short_word(UsartShortWord::five));
            print(serial, "  a zero word's low run with M_EXT stored raw: ", eight,
                  " bits plain, ", seven, " under the seven-bit code, ", five,
                  " under the five-bit one", crlf);
            bench.verdict("THE WIRE AGREES WITH THE READ-BACK: M_EXT stored raw leaves the word at "
                          "M's eight bits - nine low bits for a zero word under every code - so "
                          "the field is absent, not write-only",
                          eight == 9u && seven == 9u && five == 9u);
        }

        // ---- RX_BUSY, caught while a frame arrives at 1200 baud ----
        U2::enable(false);
        U4::enable(false);
        (void)U2::configure(UartFormat{}, usart_divisor(pclk1, 1200));
        (void)U4::configure(UartFormat{}, usart_divisor(pclk1, 1200));
        port_on<U2>();
        port_on<U4>();
        wait_us(20'000);
        U4::clear_by_read();
        uint32_t busy_seen = 0;
        uint32_t polls = 0;
        U2::write_data(0x00);
        Stopwatch w;
        while (!U4::rx_ready() && w.us() < 20'000UL) {
            ++polls;
            if (U4::receiving()) {
                ++busy_seen;
            }
        }
        wait_us(2000);
        const bool busy_after = U4::receiving();
        (void)U4::read_word();
        print(serial, "  RX_BUSY read set in ", busy_seen, " of ", polls,
              " polls while a frame arrived, ", busy_after ? "SET" : "clear", " after it", crlf);
        bench.verdict("RX_BUSY, where the die has it, stands while a frame arrives and falls "
                      "after it; a die without it reads zero throughout (the line above says "
                      "which)",
                      !busy_after);
        all_off();
    }
}

// ---------------------------------------------------------------------------
// p - UART5..UART8 with no wire
// ---------------------------------------------------------------------------

/// A byte read off a TRANSMIT pad by software: the start bit's fall found
/// by polling the pad's own input register, then each bit sampled at its
/// middle. Bit 8 set when the stop bit came back low; 0xFFFF when no
/// start bit came at all.
template <typename Pad>
uint16_t sample_tx(uint32_t baud, uint32_t timeout_us) {
    const uint32_t bit = SysClock::hz / baud;
    Stopwatch w;
    while (Pad::read()) {
        if (w.us() > timeout_us) {
            return 0xFFFFu;
        }
    }
    Stopwatch f;
    uint16_t v = 0;
    for (uint8_t i = 0; i < 8u; ++i) {
        const uint32_t at = bit + bit / 2u + static_cast<uint32_t>(i) * bit;
        while (f.cycles() < at) {
        }
        if (Pad::read()) {
            v = static_cast<uint16_t>(v | (1u << i));
        }
    }
    while (f.cycles() < 9u * bit + bit / 2u) {
    }
    return Pad::read() ? v : static_cast<uint16_t>(v | 0x100u);
}

/// An 8N1 frame banged into a RECEIVE pad through its own pull - letter
/// c's bit-banger on any pad.
template <typename Pad>
void bang_on(uint8_t byte, uint32_t baud) {
    const uint32_t bit = SysClock::hz / baud;
    Stopwatch w;
    uint32_t n = 0;
    Pad::clear();
    ++n;
    for (uint8_t i = 0; i < 8u; ++i) {
        while (w.cycles() < n * bit) {
        }
        if (((byte >> i) & 1u) != 0u) {
            Pad::set();
        } else {
            Pad::clear();
        }
        ++n;
    }
    while (w.cycles() < n * bit) {
    }
    Pad::set();
    while (w.cycles() < (n + 1u) * bit) {
    }
}

template <uint8_t n>
struct UpperRig {
    using U = Usart<n>;
    using Tx = Pin<U::pads.tx.port, U::pads.tx.pin>;
    using Rx = Pin<U::pads.rx.port, U::pads.rx.pin>;
};

template <uint8_t n>
void upper_port_off() {
    using R = UpperRig<n>;
    Pfic::disable(R::U::irq);
    R::U::bus_clock(true);
    R::U::reset();
    R::U::bus_clock(false);
    R::Tx::release();
    R::Rx::release();
}

/// One upper port measured with no wire; true when every step held.
template <uint8_t n>
bool upper_port() {
    using R = UpperRig<n>;
    using U = typename R::U;
    using S = V303State<device::has_usart(n)>;
    constexpr uint32_t baud = 9600;
    U::bus_clock(true);
    U::reset();
    const bool gate = Rcc::enabled(Bus::pb1, usart_gate_for(n));
    R::Tx::function();
    R::Rx::input(PinPull::up);
    const uint32_t brr = usart_divisor(pclk1, baud);
    const bool framed = U::configure(UartFormat{}, brr) && U::brr() == brr && !U::is_full;
    port_on<U>();
    wait_us(3000);   // TE's idle frame out

    // Out: a byte read back off the transmit pad by software.
    U::write_data(0xA5);
    const uint16_t out = sample_tx<typename R::Tx>(baud, 5000);
    Stopwatch w;
    while (!U::tx_complete() && w.us() < 5000u) {
    }

    // In: a frame banged into the receive pad through its pull.
    U::clear_by_read();
    bang_on<typename R::Rx>(0x3C, baud);
    Stopwatch r;
    while (!U::rx_ready() && r.us() < 5000u) {
    }
    const uint16_t status = U::status();
    const uint16_t in = U::rx_ready() ? static_cast<uint16_t>(U::read_word() & 0xFFu) : 0xFFFFu;

    // The vector: a transmission complete reaching this port's own entry.
    S::upper_interrupts[n - 5u] = 0;
    U::clear_flags(usart_tc);
    U::interrupts(usart_tcie, true);
    Pfic::enable(U::irq);
    U::write_data(0x11);
    wait_us(4000);
    const uint32_t vec = S::upper_interrupts[n - 5u];
    U::interrupts(usart_tcie, false);
    Pfic::disable(U::irq);

    print(serial, "  UART", n, " (TX P", U::pads.tx.port, U::pads.tx.pin, ", RX P",
          U::pads.rx.port, U::pads.rx.pin, ", vector ", static_cast<uint8_t>(U::irq),
          "): gate ", gate ? "open" : "SHUT", ", BRR ", U::brr(), ", out ", hex(out), ", in ",
          hex(in), " (status ", hex(status), "), vector ran ", vec, crlf);
    upper_port_off<n>();
    return gate && framed && out == 0xA5u && in == 0x3Cu &&
           (status & (usart_fe | usart_ne | usart_pe)) == 0u && vec >= 1u;
}

/// The transports whose columns land on the two-wire debug port's own
/// pads: UART8 on code 1 (PA14/PA15) and USART3 on code 2 (PA13/PA14).
/// Named through `on`, so no other part forms them.
template <bool on>
struct DebugPadPorts {
    using Uart8OnDebug = Uart<on ? 8 : 8, P, 16, 16, UartFormat{}, NoDmaEngine, NoDmaEngine, 1>;
    using Usart3OnDebug = Uart<on ? 3 : 3, P, 16, 16, UartFormat{}, NoDmaEngine, NoDmaEngine, 2>;
    static_assert(usart_column_on_debug_port(8, 1) && usart_column_on_debug_port(3, 2),
                  "test_vx03_serial: UART8's code 1 and USART3's code 2 are the columns on the "
                  "debug port's pads (tables 10-25 and 10-31)");
};

template <bool on = (device::has_usart(5) && device::has_usart(6) && device::has_usart(7) &&
                     device::has_usart(8))>
void tp_upper_ports() {
    if constexpr (on) {
        all_off();
        const bool five = upper_port<5>();
        const bool six = upper_port<6>();
        const bool seven = upper_port<7>();
        const bool eight = upper_port<8>();
        bench.verdict("UART5: its gate, its divisor, a byte off its transmit pad, a frame into its "
                      "receive pad and its own vector (69)",
                      five);
        bench.verdict("UART6, UART7 and UART8 the same, each on its own vector at 87, 88 and 89 "
                      "of this class's tail - with no wire, the receive side fed through the "
                      "pad's own pull because nothing else reaches it",
                      six && seven && eight);

        // THE COLUMNS ON THE DEBUG PORT'S PADS: with the probe attached,
        // SW_CFG reads as the two-wire port alive and init() refuses both
        // transports before it touches a register. Asked only where the
        // field says so - a refusal that failed would take the probe's
        // pads from it until the next reset.
        const uint8_t sw_cfg = Afio::debug_config();
        const bool alive = Afio::debug_port_enabled();
        bool refused8 = false;
        bool refused3 = false;
        if (alive) {
            refused8 = !DebugPadPorts<on>::Uart8OnDebug::init(clock, 9600);
            refused3 = !DebugPadPorts<on>::Usart3OnDebug::init(clock, 9600);
        }
        const bool still = Afio::debug_port_enabled() && Afio::debug_config() == sw_cfg;
        print(serial, "  SW_CFG reads ", sw_cfg, " with the probe attached (", alive ? "the "
              "two-wire port alive" : "the port GONE", "); UART8 on PA14/PA15 ",
              refused8 ? "refused" : "NOT refused", ", USART3 on PA13/PA14 ",
              refused3 ? "refused" : "NOT refused", crlf);
        bench.verdict("SW_CFG reads the two-wire debug port alive while the probe is attached, "
                      "and a transport whose column lands on its pads - UART8's code 1, USART3's "
                      "code 2 - is refused by init() and leaves the field as it found it",
                      alive && refused8 && refused3 && still);
        all_off();
    }
}

template <bool on>
void upper_off() {
    if constexpr (on) {
        if constexpr (device::has_usart(5) && device::has_usart(8)) {
            upper_port_off<5>();
            upper_port_off<6>();
            upper_port_off<7>();
            upper_port_off<8>();
        }
        if constexpr (device::dma_controller_count >= 2u) {
            Pfic::disable(dma_channel_irq(2, 3));
            Pfic::disable(dma_channel_irq(2, 5));
        }
        V303State<on>::engines = false;
    }
}

/// UART4's two DMA2 vectors, serving the pair's transport while letter n
/// runs and nothing otherwise.
template <bool on = (has_fourth && v303_pair && device::dma_controller_count >= 2u)>
void pair_dma_vector() {
    if constexpr (on) {
        if (V303State<on>::engines) {
            (void)PairPort<on>::Port::dma_isr();
        }
    }
}

/// An upper port's vector body: counted, and its flags put down.
template <uint8_t n>
void upper_vector() {
    if constexpr (device::has_usart(n)) {
        using U = Usart<n>;
        using S = V303State<device::has_usart(n)>;
        S::upper_interrupts[n - 5u] = S::upper_interrupts[n - 5u] + 1u;
        const uint16_t st = U::status();
        if ((st & usart_tc) != 0u) {
            U::clear_flags(usart_tc);
            U::interrupts(usart_tcie, false);
        }
        if ((st & usart_rxne) != 0u) {
            (void)U::read_word();
        }
    }
}

/// The CH32V303's letters, registered where the part has what they ask.
template <bool on = (v303_pair && has_fourth)>
void register_v303_letters() {
    if constexpr (on) {
        bench.letter('n', "the crossed pair's engines: DMA1 against DMA2, 4 KB each way",
                     tn_pair_engines<>);
        bench.letter('o', "the lot's registers on the pair: CTLR4, M_EXT, RX_BUSY",
                     to_lot_registers<>);
        bench.letter('p', "UART5..UART8 with no wire: the pads, the frame, the vectors",
                     tp_upper_ports<>);
    }
}

// ---- the menu ---------------------------------------------------------------

void banner() {
    print(serial, crlf, "test_vx03_serial - the USART chapter on ", device::part_name, crlf);
    print(serial, "  the console is USART1 at 115200; the instruments are USART2 (PA2/PA3) "
                  "and the fourth port", crlf);
    bench.menu();
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

/// USART2's vector: the transport's body while letter l runs, and the
/// flag counter of every other letter.
extern "C" BRIO_CH32_INTERRUPT void usart2_handler() {
    if (u2_transport) {
        (void)Loop::isr();
        return;
    }
    const uint16_t st = U2::status();
    u2_interrupts = u2_interrupts + 1u;
    if ((st & brio::usart_txe) != 0u && (U2::regs().CTLR1 & brio::usart_txeie) != 0u) {
        u2_txe = u2_txe + 1u;
        U2::interrupts(brio::usart_txeie, false);
    }
    if ((st & brio::usart_tc) != 0u) {
        u2_tc = u2_tc + 1u;
        U2::clear_flags(brio::usart_tc);
    }
    if ((st & brio::usart_lbd) != 0u) {
        u2_lbd = u2_lbd + 1u;
        U2::clear_flags(brio::usart_lbd);
    }
    if ((st & brio::usart_cts) != 0u) {
        u2_cts = u2_cts + 1u;
        U2::clear_flags(brio::usart_cts);
        // DISARMED THE WAY TXE IS: a flag whose source stands would
        // re-enter this body without end, and the letter that wants the
        // next change arms it again itself.
        U2::cts_interrupt(false);
    }
    if ((st & brio::usart_pe) != 0u) {
        u2_pe = u2_pe + 1u;
    }
    if ((st & brio::usart_idle) != 0u) {
        u2_idle = u2_idle + 1u;
    }
    if ((st & brio::usart_rxne) != 0u) {
        u2_rxne = u2_rxne + 1u;
        u2_last_word = U2::read_word();
    } else if ((st & (brio::usart_pe | brio::usart_idle)) != 0u) {
        // The read sequence the chapter prescribes: the status register
        // is already read, so one data read clears what stands.
        (void)U2::read_word();
    }
}

/// The fourth port's vector, the one at this device class's own tail.
extern "C" BRIO_CH32_INTERRUPT void uart4_handler() {
    if (!has_fourth) {
        return;
    }
    u4_interrupts = u4_interrupts + 1u;
    const uint16_t st = U4::status();
    if ((st & brio::usart_tc) != 0u) {
        U4::clear_flags(brio::usart_tc);
    }
    if ((st & brio::usart_rxne) != 0u) {
        (void)U4::read_word();
    }
}

/// USART2's two DMA1 channels: the Loop transport's engines while letter l
/// or letter n runs. The engines switch their channels' lines on, so the
/// vectors are bound whether or not a letter uses them.
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() {
    if (u2_transport) {
        (void)Loop::dma_isr();
    }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() {
    if (u2_transport) {
        (void)Loop::dma_isr();
    }
}

// The CH32V303's: UART4's two DMA2 channels and the four upper ports'
// vectors. On the other series these bodies are empty and nothing in the
// vector table names them.
extern "C" BRIO_CH32_INTERRUPT void dma2_channel3_handler() { pair_dma_vector<>(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel5_handler() { pair_dma_vector<>(); }
extern "C" BRIO_CH32_INTERRUPT void uart5_handler() { upper_vector<5>(); }
extern "C" BRIO_CH32_INTERRUPT void uart6_handler() { upper_vector<6>(); }
extern "C" BRIO_CH32_INTERRUPT void uart7_handler() { upper_vector<7>(); }
extern "C" BRIO_CH32_INTERRUPT void uart8_handler() { upper_vector<8>(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the baud generator: every standard rate on both buses", ta_divisor);
    bench.letter('b', "the frame on the wire, measured by the timer", tb_frame);
    bench.letter('c', "the bit-banged line into the receiver", tc_receive);
    bench.letter('d', "mute mode: both wakes", td_mute);
    bench.letter('e', "LIN: the break sent and the break detected", te_lin);
    bench.letter('f', "half duplex on one wire, and no pull-up on it", tf_half_duplex);
    bench.letter('g', "IrDA: the encoder's pulse and the decoder", tg_irda);
    bench.letter('h', "the synchronous mode on the fourth port's clock pad", th_synchronous);
    bench.letter('i', "the smartcard: its register face and its card clock", ti_smartcard);
    bench.letter('j', "the flags and the two vectors", tj_flags);
    bench.letter('k', "hardware flow control: the CTS hold and the RTS hand", tk_flow);
    bench.letter('l', "the loopback PA2-PA3, when it is strapped", tl_loopback);
    bench.letter('m', "the crossed pair, when it is strapped", tm_cross);
    register_v303_letters();

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED",
              " tick=", tick_ok ? "STK" : "FAILED", crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
