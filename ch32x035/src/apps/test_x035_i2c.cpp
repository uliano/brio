// test_x035_i2c - the reference bench suite for the CH32X035's I2C
// chapter: ch32x035/i2c.hpp over RM ch. 15, the host side against the
// desk's partner, the event machine under its two vectors, the DMA
// engines on channels 6 and 7, the client half addressed by a foreign
// controller, and util/i2c_bus.hpp's arbiter with not one line changed.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// THE CONSOLE IS USART2 on PA2/PA3 over the probe's serial. The bus is
// I2C1 on its column 010 - SCL PC16, SDA PC17, the USB pads, the one
// column of the CH32X035F8U6 off the debug port - wired to the desk's
// CH32V006F8U6, which runs `twi_peer` on its column 0 (both WeAct boards,
// docs/boards/weact-ch32x035f8.md):
//
//   SCL   PC16  <->  the CH32V006's PC2
//   SDA   PC17  <->  the CH32V006's PC1
//   a 1.5 kOhm resistor from each line to this board's PB3, and GND
//
// THE PULL-UPS HANG FROM PB3, NEVER FROM VCC: PC17 is this board's boot
// pin - a high level on it at power-up enters WCH's loader (the CH32X035
// evaluation board reference's section 5, a rule of the chip's loader that
// the WeAct board's DOWNLOAD key uses too) - so the bus is pulled up only
// while an image that uses it drives PB3 high, and PB3 floats from every
// reset. This
// image drives it high, push-pull, from its boot to its end, the supply
// checked first; letter a lets it float for the one measurement that
// asks what the USB pads' own pulls do. 1.5 kOhm at 3.3 V sinks 2.2 mA a
// line, and PB3 sources at most about 4.4 mA with both lines held low.
//
// THE PEER is commanded IN BAND over the bus under test through
// avrxt/src/apps/twi_link.hpp, included by relative path: one source of
// truth for the wire format, whatever the peer's architecture. Its
// command-mode client answers ONE address (0x6B) with no general call and
// no second address.
//
// THE HOST RUNS ON ITS TWO VECTORS (events on i2c1_ev, errors on i2c1_er);
// a letter that wants a tenure waited out spins on a flag the handler
// sets. Three I2cHost instantiations share the one instance - the plain
// one and the one on DMA channels 6 and 7, both in the default placement,
// and letter x's plain one with its event pump in SRAM (util/placement.hpp's
// InRam) - and a flag says which owns the vectors, since tasks over one
// peripheral share its registers and not their statics. The event
// vector's body is timed on the STK at every entry (letter x reads the
// sums).
//
// What is exercised, letter by letter:
//   a  THE BLOCK: the lot probe first (present(), FREQ, CKCFGR and OADDR1
//      written and read back, PE, a START on the idle bus raising SB), the
//      reset values, the arithmetic, the word past CKCFGR (no rise-time
//      register), the SMBus bits the register description does not have,
//      the enable protection measured; each line's RISE from the pad; and
//      THE USB PADS' PULLS under the alternate function with the rail off
//   b  THE SCAN: every address 0x08..0x77 probed with the empty request
//   c  THE PEER: the twi_link command channel, its ident, ten round trips
//   d  the tenure shapes against the peer, the four receive procedures,
//      the counted serve ended once its read is over, 0x7A and the
//      general call refused inside start() with nothing on the wire
//   e  the vocabulary against the peer (nack_addr from a deaf client,
//      nack_data at a commanded byte) and commanded stretching priced
//   f  the two speeds against a second chip, byte-exact both ways, each
//      write timed and its SCL judged against the rate asked, fast mode at
//      DUTY 16/9 too - and a rung at 1 MHz past the chapter, measured and
//      not judged
//   g  THE DMA ENGINES on channels 6 and 7 under I2cEngine::dma: the same
//      shapes, a read at 400 kHz besides, the interrupts they take - one
//      completion a block, the write's handing its half back to the event
//      vector - their SCL timed
//   h  THE KERNEL against the peer: I2cBus over I2cHost, the NACK in its
//      place, the rejection, both votes, a held SDA and a held clock
//      answered in their place, the recover()ed engine after
//   i  THE REFUSAL, wireless: a speed the clock cannot make is answered
//      i2c_rejected inside start() and delivered through the arbiter
//   n  THE ENGINE A REQUEST NAMES (Request::engine): `dma` at one byte
//      each way and a 1 + 1, every half on the engines; `interrupt` at
//      sixteen touching no channel; `dma` on the engineless host refused with nothing on the wire; the
//      NACKs on the address and on data under `dma`
//   j  ARBITRATION in both directions, the losing round again under
//      I2cEngine::dma, a round repeated when the wire shows two tenures and
//      no race (race_missed()), the misses printed, and THIS BOARD AS THE CLIENT of the
//      peer's own controller - written to, its stretch one byte late
//      measured, and read from, the byte a transmitter is asked beyond the
//      closing NACK dropped by flush()
//   k  THE STUCK BUS: the peer holding SDA, unstick() counting the clocks
//   p  THE DEAD CHANNEL, staged by a timer's compare request held on DMA
//      channel 6 or 7 with no wire: a write and a read over it never
//      completing with the event vector unstormed, each
//      landing whole once the timer is reset; and release(), its gate closed
//      with no reset, leaving the next owner nothing of the block's own request
//   q  THE SIBLINGS' QUIRKS RE-MEASURED on this die: a START into a held
//      SDA, raw; how long the host's own STOP stands in CTLR1; a CTLR1
//      store under it; STOPF on a STOP the client half was not addressed in
//   r  THE REPEATED START of a write-then-read, requested on the TxE of the
//      last written byte: one to four written then four read, both speeds,
//      pump and engines (both halves on them, each block's completion
//      counted) - then the last byte refused
//   u  A BUSY LEFT STANDING over an idle wire - a START by hand and the lines
//      let go with no STOP condition - and start()'s SWRST taking it out
//   v  THIS BOARD AS THE CLIENT FROM ITS VECTORS: letter j's two client
//      cases with the interrupt enables up, service() and error_service()
//      on i2c1_ev and i2c1_er - one entry per event, the body timed
//   x  THE COST: every tenure shape on the pump and on the engines at 400
//      kHz, its wall time against the wire's, its vector entries and the
//      cycles inside them, start()'s own; the engines' arithmetic against
//      the pump (b, F, E and the length from which `dma` costs the core
//      less) at 400 and at 100 kHz; the pump's shapes again with its entry
//      in SRAM, the two placements side by side
//
//   t  THE TWO BOUNDS OF start() AGAINST THEIR NAMES: bus_free_us with SCL held
//      and over an idle wire, stop_drain_us run out by a client holding the
//      clock, both speeds, start() answering i2c_timeout, raw and through the
//      arbiter with a per-bus timeout and with none, the next request on the wire
//      behind it; and both bounds with the STK stopped, a count of polls
//   w  A REQUEST INTO A TENURE ANOTHER CONTROLLER IS CLOCKING: the peer's long
//      write, the post's moment swept, both speeds: no reset taken, and at 20 kHz
//      the other controller's tenure not ended in i2c_bus_error
//   y  THE ENGINES' CHANNELS STARVED: a memory-to-memory block at very_high above
//      them, up before the START, caught at the third byte and on a read's second-
//      last byte, writes and reads: the status, the far end's count, the next read's
//      bytes, the event vector's entries
//
// THE SUPPLY at boot: the voltage detector at 3.0 V (pwr.hpp); an image
// that drives a link pad from a sagging rail leaves PB3 floating, refuses
// every letter and says so.
//
// With the peer attached `z` outlasts `brio run`'s default 60 s (the
// peer's command windows are hundreds of milliseconds each): pass
// `--timeout 600`.
//
// THE SUITE IS TWO IMAGES (design/overview.md, "A suite's image fits the
// family's smallest chip"): whole it is over the 62 KB every part of the
// series has, so test_x035_i2c-1 carries the single-host wire letters and
// test_x035_i2c-2 the kernel, the two-host, the dead-channel, the stuck-BUSY,
// the vectors' client, the cost, the bounds' and the live tenure's letters.
// build: boards = x035f8
// build: groups = abcdefgiknry,hjpquvxtw
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "ch32x035/afio.hpp"
#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/i2c.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/pwr.hpp"
#include "ch32x035/tim.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/i2c_bus.hpp"
#include "util/placement.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/test_wait.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED (see the header comment).
#include "../../../avrxt/src/apps/twi_link.hpp"

using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, P>;
constexpr Serial serial;

using H = I2c<1>;
using Host = I2cHost<1>;
using DmaHost = I2cHost<1, i2c1_default_pins, DmaTxEngine<6>, DmaRxEngine<7>>;
/// Letter x's twin of Host, its event pump placed in SRAM.
using RamHost = I2cHost<1, i2c1_default_pins, NoDmaEngine, NoDmaEngine, InRam>;
using Client = I2cClient<1>;
using SclPin = Pin<'C', 16>;
using SdaPin = Pin<'C', 17>;
using Rail = Pin<'B', 3>;
static_assert(i2c1_default_pins.remap == 2u && i2c1_default_pins.scl == SclPin::pad &&
                  i2c1_default_pins.sda == SdaPin::pad,
              "the CH32X035F8's one column off the debug port is I2C1_RM 010, PC16/PC17");

TestBench<Serial> bench;

/// An address nobody on this bus answers.
constexpr uint8_t nobody_addr = 0x23;

/// Which task owns the instance's vectors.
enum class Owner : uint8_t { host, dma_host, ram_host, bus_ao };
volatile Owner owner = Owner::host;
/// The arbiter with no per-bus timeout, letter t's, owns the host's completions.
volatile bool bus_ao_untimed = false;

/// Letter w's client half is up: the event vector serves it, ahead of the host.
volatile bool live_client = false;
volatile uint16_t live_count = 0;
volatile uint16_t live_sum = 0;
volatile bool host_done = false;
volatile uint32_t dma6_entries = 0;   // the transmit channel's vector
volatile uint32_t dma7_entries = 0;   // the receive channel's
volatile uint32_t host_isr_entries = 0;
volatile uint32_t error_isr_entries = 0;
/// The event vector's body on the STK, summed and its longest.
volatile uint32_t ev_cycles = 0;
volatile uint32_t ev_max = 0;
/// The two DMA vectors' bodies on the STK, summed (letter x).
volatile uint32_t dma_cycles = 0;
/// The host's storm budget: a vector that fires this many times inside
/// one tenure is silenced and the fact reported.
constexpr uint32_t host_isr_budget = 60'000;
volatile bool host_stormed = false;
volatile uint32_t storms = 0;

bool supply_ok = true;

uint8_t tx_buf[255];
uint8_t rx_buf[255];

/// A tenure that never answered: the suite's own word, not a wire code.
constexpr uint8_t no_answer = 200;

/// The suite's ruler (util/test_wait.hpp), and the clock of its timed
/// spans: the kernel ticker's own cycles(), the tick count and the STK
/// counter's position composed across the restart (ch32x035/ticker.hpp), a
/// plain call - a time-out needs no inlined read. It wraps at 2^32, so only
/// differences are used; a read of it is some 76 cycles (docs/ch32x035/platform.md).
struct Ruler {
    static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};

/// ONE LOAD OF THE STK COUNTER, for the one span shorter than a read of
/// the Ruler: letter a's rise of a line, tens of cycles judged under a
/// 400 kHz bit, where the Ruler's own 76 cycles would be most of the figure. The
/// difference folds one restart at the compare, so it holds for a span
/// under one tick period (48000 cycles), which rise_of() checks on the
/// Ruler read around it.
uint32_t stk_stamp() { return stk()->CNTL; }
uint32_t stk_span(uint32_t from, uint32_t to) {
    const uint32_t period = stk()->CMPLR + 1u;
    return to >= from ? to - from : to + period - from;
}

/// A fixed span on the ruler, in milliseconds: a settle, not a time-out.
void settle_ms(uint32_t ms) { brio::spin_us<Ruler>(ms * 1000u); }

/// Every console byte off the wire, bounded (util/test_wait.hpp);
/// false when the console did not go idle within 200 ms.
bool console_drain() { return brio::console_drain<Ruler, Serial>(); }

/// The vectors' completion flag: the one condition every tenure's wait asks,
/// one object so that those waits share one copy of the loop.
constexpr auto host_answered = [] { return host_done; };

/// THE TWO RC CLOCKS' RATIO. A span the peer times on its own clock (a
/// commanded hold) is read here on this board's: this HSI within -1.7 to
/// +1.6 per cent of its name (CH32X035 datasheet V1.7, table 3-9, 0 to
/// 70 C; docs/ch32x035/clock.md), the desk's peer, the CH32V006F8U6 on its
/// WeAct board, which fits no crystal (docs/boards/weact-ch32v006f8.md), on
/// its HSI fallback within 2.0 per cent (CH32V006 datasheet V2.0, table
/// 3-11, -10 to 70 C; docs/ch32v00x/sleep.md). So a span of the peer's reads
/// here between 0.983 / 1.02 = 0.9637 and 1.016 / 0.98 = 1.0367 of its name:
/// in thousandths, rounded outward. A peer on a crystal narrows it.
constexpr uint32_t rc_lo_permille = 963;
constexpr uint32_t rc_hi_permille = 1037;
/// A span of `us` on the peer's clock, at its shortest and its longest here.
constexpr uint32_t peer_lo_us(uint32_t us) { return us * rc_lo_permille / 1000u; }
constexpr uint32_t peer_hi_us(uint32_t us) { return (us * rc_hi_permille + 999u) / 1000u; }
static_assert(peer_lo_us(15'000) == 14'445 && peer_hi_us(15'000) == 15'555, "the arithmetic beside the bounds");

/// THE WAIT FOR A TENURE'S ANSWER through host_tenure() and dma_tenure():
/// 100 ms. The longest tenure they wait out is stand_stop()'s write held
/// 45 ms by the peer, up to peer_hi_us(45'000) = 46665 us here, and its three
/// bytes' 27 clocks at the slowest rate letter f passes (13.3 us), 360 us:
/// 47.1 ms; letter x's write of 255 at 100 kHz is 256 x 9 clocks, 30.7 ms at
/// that rate. 100 ms is twice the longest; a tenure that never completes
/// (letter p's dead channel) is answered by this bound and recover().
constexpr uint32_t tenure_budget_us = 100'000;

/// THE RAIL: PB3 driven high, push-pull - the bus's pull-ups (the header).
void rail_on() { Rail::output(true); }
void rail_off() { Rail::release(); }

void clear_counts() {
    host_isr_entries = 0;
    error_isr_entries = 0;
    ev_cycles = 0;
    ev_max = 0;
    dma_cycles = 0;
    host_stormed = false;
}

/// Put the plain host back where a letter can use it.
void host_ready() {
    owner = Owner::host;
    (void)Host::init(clock);
    if (H::busy()) {
        (void)Host::unstick();
        (void)Host::recover();
    }
}

/// One tenure through the TASK, waited out. Returns the status. The plain
/// host has no engines: `engine` is the interrupt unless a letter asks for
/// the refusal by name.
uint8_t host_tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
                    I2cSpeed speed, I2cEngine engine = I2cEngine::interrupt) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    r.engine = engine;
    host_done = false;
    clear_counts();
    if (Host::start(r)) {
        return Host::status();
    }
    if (!brio::wait_until<Ruler>(host_answered, tenure_budget_us)) {
        print(serial, "    STALL: STAR1=", hex(H::status1()), " STAR2=", hex(H::status2()),
              " CTLR1=", hex(H::regs().CTLR1), " CTLR2=", hex(H::regs().CTLR2), " ev entries ",
              host_isr_entries, " er ", error_isr_entries, host_stormed ? " STORMED" : "", crlf);
        (void)Host::recover();
        return no_answer;
    }
    return Host::status();
}

/// The same through the DMA host, on the engine named: the engines unless
/// the caller asks for the interrupt by name.
uint8_t dma_tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
                   I2cSpeed speed, I2cEngine engine = I2cEngine::dma) {
    DmaHost::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    r.engine = engine;
    host_done = false;
    clear_counts();
    if (DmaHost::start(r)) {
        return DmaHost::status();
    }
    if (!brio::wait_until<Ruler>(host_answered, tenure_budget_us)) {
        print(serial, "    STALL (dma): STAR1=", hex(H::status1()), " STAR2=", hex(H::status2()),
              " ch6 flags=", hex(DmaChannel<6>::flags()), " ch7 flags=", hex(DmaChannel<7>::flags()),
              " ev entries ", host_isr_entries, crlf);
        (void)DmaHost::recover();
        return no_answer;
    }
    return DmaHost::status();
}

/// The wire has pull-ups: both lines read high with nothing driving. Port
/// C's gate opened first - a read through a shut gate is not the pads'
/// (docs/ch32x035/pin.md).
bool wire_pulled_up() {
    SclPin::P::clock_on();
    SdaPin::P::clock_on();
    return SclPin::read() && SdaPin::read();
}

/// THE WIRE'S RISE TIME, in HCLK cycles: the pad pulled low for a while -
/// a push-pull output over a clear output bit, the one way this series
/// pulls a pad low by hand - then released as a floating input, and the
/// cycles from the release's return until it reads high counted (the
/// configuration store is the release, a few instructions before that
/// return, so a rise shorter than those reads as zero polls). 0xFFFF =
/// never rose in the budget, or rose past one tick period.
struct Rise {
    uint32_t cycles;
    uint32_t polls;
};

template <class Pd>
Rise rise_of() {
    Pd::output(false);
    (void)delay_us(clock, 20);
    (void)Pd::input();
    // The Ruler read OUTSIDE the two stamps: what it spans holds the stamps'
    // span, so under one tick period there the fold of stk_span() is right.
    const uint32_t r0 = Ruler::now();
    const uint32_t t0 = stk_stamp();
    // The count of reads is the instrument (a read of the pad a turn), and
    // its bound: 65535 turns of a few cycles each, about 10 ms, past which the line
    // never rose and the figure reads 0xFFFF, failing the verdict on it.
    uint32_t n = 0;
    while (!Pd::read() && n < 0xFFFFu) {
        ++n;
    }
    const uint32_t t1 = stk_stamp();
    const bool within_period = Ruler::now() - r0 < stk()->CMPLR + 1u;
    return Rise{n >= 0xFFFFu || !within_period ? 0xFFFFu : stk_span(t0, t1), n};
}

/// Both lines driven low, then given to `release` (the alternate function,
/// or a pulled input), the output data bit as `outdr` says, and read two
/// milliseconds later: does anything pull them up?
struct Levels {
    bool scl;
    bool sda;
};

template <bool af>
Levels released_levels(bool outdr) {
    SclPin::output(false);
    SdaPin::output(false);
    (void)delay_us(clock, 20);
    if constexpr (af) {
        SclPin::function();
        SdaPin::function();
        if (outdr) {
            SclPin::set();
            SdaPin::set();
        } else {
            SclPin::clear();
            SdaPin::clear();
        }
    } else {
        (void)SclPin::input(PinPull::up);
        (void)SdaPin::input(PinPull::up);
    }
    (void)delay_us(clock, 2000);
    return Levels{SclPin::read(), SdaPin::read()};
}

// ===========================================================================
// The peer's command channel (twi_link)
// ===========================================================================

using twilink::Op;

constexpr I2cSpeed link_speed = I2cSpeed::standard_100k;

uint8_t frame_buf[twilink::max_payload + 4];
uint8_t resp_buf[twilink::response_bytes];
twilink::Decoder dec;
bool link_quiet = false;

void link_ready() { host_ready(); }

bool send_frame(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    twilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof frame_buf) {
                frame_buf[n++] = b;
            }
        },
        op, p, len);
    return host_tenure(twilink::command_addr, frame_buf, n, nullptr, 0, link_speed) == i2c_ok;
}

bool recv_frame(twilink::Frame& out) {
    dec.reset();
    for (uint8_t i = 0; i < twilink::response_bytes; ++i) {
        resp_buf[i] = 0;
    }
    if (host_tenure(twilink::command_addr, nullptr, 0, resp_buf, twilink::response_bytes,
                    link_speed) != i2c_ok) {
        return false;
    }
    for (uint8_t i = 0; i < twilink::response_bytes; ++i) {
        if (dec.feed(resp_buf[i]) == twilink::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

bool command_once(Op op, const uint8_t* p, uint8_t len) {
    if (!send_frame(op, p, len)) {
        return false;
    }
    settle_ms(2);
    twilink::Frame f;
    if (!recv_frame(f)) {
        return false;
    }
    return f.op == Op::ack && f.len == 2 && f.data[0] == twilink::byte_of(op);
}

const uint8_t no_payload[1] = {0};

/// Three attempts with the peer's own recovery bound between them.
bool command(Op op, const uint8_t* p = no_payload, uint8_t len = 0) {
    for (uint8_t k = 0; k < 3; ++k) {
        if (command_once(op, p, len)) {
            settle_ms(twilink::arm_ms);
            return true;
        }
        link_ready();
        settle_ms(400);
    }
    if (!link_quiet) {
        print(serial, "    LINK FAILURE op ", hex(twilink::byte_of(op)),
              ": the CH32V006 must be running `twi_peer`; check the two wires, the "
              "resistors to PB3 and the GND.",
              crlf);
    }
    return false;
}

bool query(Op op, twilink::Frame& data) {
    if (!command(op)) {
        return false;
    }
    settle_ms(2);
    return recv_frame(data);
}

bool peer_report(twilink::Report& r) {
    for (uint8_t k = 0; k < 4; ++k) {
        twilink::Frame f;
        if (query(Op::report, f) && f.op == Op::report_data && f.len == twilink::report_size) {
            r = twilink::get_report(f.data);
            return true;
        }
        settle_ms(60);
    }
    return false;
}

/// The extended repertoire's `counters` (twi_link.hpp): what the last
/// action's report has no field for. False from a peer that naks it.
bool peer_counters(twilink::Counters& c) {
    for (uint8_t k = 0; k < 2; ++k) {
        twilink::Frame f;
        if (query(Op::counters, f) && f.op == Op::counters_data && f.len == twilink::counters_size) {
            c = twilink::get_counters(f.data);
            return true;
        }
        settle_ms(60);
    }
    return false;
}

bool peer_act(Op op, const twilink::Params& a) {
    uint8_t p[twilink::params_size];
    twilink::put_params(p, a);
    return command(op, p, twilink::params_size);
}

/// A ping of the peer has been answered in this boot.
bool peer_seen = false;

/// Is the peer there at all? Every letter that needs it asks first, so an
/// absent peer is a named outcome and never a hang.
bool ensure_link() {
    link_quiet = true;
    link_ready();
    for (uint8_t k = 0; k < 3; ++k) {
        if (command(Op::ping)) {
            link_quiet = false;
            peer_seen = true;
            return true;
        }
    }
    link_quiet = false;
    print(serial,
          "  THE PEER DID NOT ANSWER. The CH32V006 must be running `twi_peer`; its console "
          "'0' forces the command-mode client back. Check the wires in this file's header.",
          crlf);
    return false;
}

/// Every wire letter asks first. No pull-ups is a skip with the reason. A
/// peer never heard in this boot is absent: the letter declines with the
/// reason and claims no verdict. A peer that answered earlier in this boot
/// and is silent now FAILS the letter.
bool need_peer() {
    if (!wire_pulled_up()) {
        bench.skip("SCL/SDA read low with the rail on: the resistors to PB3 are not on the wire");
        return false;
    }
    if (ensure_link()) {
        return true;
    }
    if (!peer_seen) {
        bench.skip("peer absent: no ping answered in this boot (twi_peer)");
        return false;
    }
    bench.verdict("the peer, which answered earlier in this boot, still answers a ping (the CH32V006 "
                  "running twi_peer)",
                  false);
    return false;
}

twilink::Params serve_params(uint16_t count, uint16_t ms, uint8_t seed) {
    twilink::Params a{};
    a.count = count;
    a.ms = ms;
    a.addr = twilink::dut_addr;
    a.seed = seed;
    a.pattern = twilink::pattern_counting;
    return a;
}

// ---- the rate on the wire ---------------------------------------------------
//
// A rate asked of the host is judged on the wire: a tenure of `periods`
// SCL periods (nine a byte, the address's included) timed on the ticker's
// cycle count from start() to its completion, START, STOP and the
// software around them inside - which only lowers the reading. The
// bracket: never above the rate asked by more than 2 % (a divisor too
// small would be), and within a quarter below it.

uint32_t scl_of(uint32_t cycles, uint32_t periods) {
    if (cycles == 0u) {
        return 0u;
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(periods) * SysClock::hz) / cycles);
}

bool scl_bracket(uint32_t measured, uint32_t asked) {
    return measured <= asked + asked / 50u && measured >= asked - asked / 4u;
}

// ===========================================================================
// a - the block
// ===========================================================================

void ta_block() {
    owner = Owner::host;
    Pfic::disable(H::event_irq());
    Pfic::disable(H::error_irq());

    // ---- THE LOT PROBE (the datasheet's table 2-1 note 3) ----
    const bool present = H::present();
    H::bus_clock(true);
    H::reset();
    const uint16_t r_c1 = H::regs().CTLR1, r_c2 = H::regs().CTLR2, r_o1 = H::regs().OADDR1,
                   r_o2 = H::regs().OADDR2, r_s1 = H::status1(), r_ck = H::regs().CKCFGR;
    H::regs().CTLR2 = 48u;
    H::regs().CKCFGR = static_cast<uint16_t>(i2c_fs | 40u);
    H::regs().OADDR1 = static_cast<uint16_t>(0x48u << 1);
    const uint16_t freq = static_cast<uint16_t>(H::regs().CTLR2 & i2c_freq_mask);
    const uint16_t ck = H::regs().CKCFGR;
    const uint16_t o1 = H::regs().OADDR1;
    print(serial, "  reset: CTLR1=", hex(r_c1), " CTLR2=", hex(r_c2), " OADDR1=", hex(r_o1), " OADDR2=",
          hex(r_o2), " STAR1=", hex(r_s1), " CKCFGR=", hex(r_ck), crlf);
    print(serial, "  written: FREQ 48 reads ", freq, ", CKCFGR 0x8028 reads ", hex(ck),
          ", OADDR1 0x0090 reads ", hex(o1), "; present() says ", present ? "PRESENT" : "ABSENT", crlf);
    bench.verdict("THE LOT PROBE: the block holds what it is given - FREQ, CKCFGR, OADDR1 - "
                  "and present() agrees",
                  present && freq == 48u && ck == (i2c_fs | 40u) && o1 == (0x48u << 1));
    bench.verdict("the reset values are table 15-1's (all zero)",
                  r_c1 == 0u && r_c2 == 0u && r_o1 == 0u && r_o2 == 0u && r_s1 == 0u && r_ck == 0u);

    // The word past CKCFGR: table 15-1 lists no RTR.
    volatile uint16_t* const past = reinterpret_cast<volatile uint16_t*>(i2c1_base + 0x20);
    const uint16_t past0 = *past;
    *past = 0x0011u;
    const uint16_t past1 = *past;
    print(serial, "  the word past CKCFGR (0x40005420): reads ", hex(past0), ", written 0x0011 reads ",
          hex(past1), crlf);
    bench.verdict("NO RISE-TIME REGISTER: the word past CKCFGR holds nothing written to it",
                  past1 != 0x0011u);
    // The SMBus bits CTLR1 has not got (3:1 reserved), and ENARP that it has.
    H::regs().CTLR1 = static_cast<uint16_t>((1u << 1) | (1u << 3) | i2c_enarp);
    const uint16_t smb = H::regs().CTLR1;
    H::regs().CTLR1 = 0;
    print(serial, "  CTLR1 written 0x001A (bits 1, 3 and ENARP): reads ", hex(smb), crlf);
    bench.verdict("CTLR1's bits 1 and 3 - the F1 lineage's SMBUS and SMBTYPE - are reserved "
                  "here and read zero; ENARP takes its bit",
                  (smb & 0x000Au) == 0u && (smb & i2c_enarp) != 0u);

    // The arithmetic at this clock.
    const auto sm = i2c_timing_for(SysClock::hz, I2cSpeed::standard_100k);
    const auto fm = i2c_timing_for(SysClock::hz, I2cSpeed::fast_400k);
    print(serial, "  100k: FREQ=", sm->freq_mhz, " CKCFGR=", hex(sm->ckcfgr), " -> ",
          i2c_scl_hz(SysClock::hz, *sm), " Hz; 400k: CKCFGR=", hex(fm->ckcfgr), " -> ",
          i2c_scl_hz(SysClock::hz, *fm), " Hz", crlf);
    bench.verdict("both speeds resolve at 48 MHz, exactly (CCR 240 and 40)",
                  sm->ckcfgr == 240u && fm->ckcfgr == (i2c_fs | 40u));
    bench.verdict("below 4 MHz nothing is legal (15.9.2's FREQ window), 4 MHz is",
                  !i2c_timing_for(3'000'000, I2cSpeed::standard_100k).has_value() &&
                      i2c_timing_for(4'000'000, I2cSpeed::standard_100k).has_value());

    // THE ENABLE PROTECTION, MEASURED: what does a CKCFGR or FREQ write
    // do with PE set? The chapter orders them before PE and says nothing
    // of a lock.
    H::timing(*sm);
    H::enable();
    H::regs().CKCFGR = static_cast<uint16_t>(i2c_fs | 40u);
    const uint16_t ck_pe = H::regs().CKCFGR;
    H::regs().CTLR2 = static_cast<uint16_t>((H::regs().CTLR2 & ~i2c_freq_mask) | 24u);
    const uint16_t freq_pe = static_cast<uint16_t>(H::regs().CTLR2 & i2c_freq_mask);
    print(serial, "  with PE set: CKCFGR written 0x8028 reads ", hex(ck_pe), ", FREQ written 24 reads ",
          freq_pe, " -> ", ck_pe == (i2c_fs | 40u) ? "no lock on CKCFGR" : "CKCFGR LOCKED",
          freq_pe == 24u ? ", none on FREQ" : ", FREQ LOCKED", crlf);
    // A measurement, printed: either answer is a finding, so no verdict.
    H::disable();
    H::regs().OADDR1 = static_cast<uint16_t>((1u << 14) | (0x48u << 1));
    print(serial, "  OADDR1 with bit 14 written: reads ", hex(H::regs().OADDR1), crlf);
    bench.verdict("OADDR1's bit 14 is reserved here (reads zero), not the F1's fixed one",
                  (H::regs().OADDR1 & (1u << 14)) == 0u);

    // ---- A START ON THE IDLE BUS (the rail on) ----
    H::reset();
    H::timing(*sm);
    (void)H::remap(i2c1_default_pins.remap);
    SclPin::function();
    SdaPin::function();
    H::enable();
    (void)delay_us(clock, 50);
    const uint16_t s2_idle = H::status2();
    H::start();
    // The count of STAR1 reads is the printed figure (an instrument, a read a
    // turn), and its bound: 100000 reads of some ten cycles, about 20 ms, past
    // which SB never rose and the verdict below fails on STAR1.
    uint32_t polls = 0;
    while ((H::status1() & i2c_sb) == 0u && polls < 100'000u) {
        ++polls;
    }
    const uint16_t s1_sb = H::status1();
    const uint16_t s2_sb = H::status2();
    H::stop();
    (void)delay_us(clock, 100);
    const uint16_t s2_after = H::status2();
    print(serial, "  idle bus: STAR2=", hex(s2_idle), "; START -> STAR1=", hex(s1_sb), " STAR2=", hex(s2_sb),
          " after ", polls, " polls; STOP behind it -> STAR2=", hex(s2_after), crlf);
    bench.verdict("a START on the idle bus raises SB with MSL and BUSY, and a STOP behind it "
                  "gives the bus back",
                  (s2_idle & i2c_busy) == 0u && (s1_sb & i2c_sb) != 0u &&
                      (s2_sb & (i2c_msl | i2c_busy)) == (i2c_msl | i2c_busy) &&
                      (s2_after & (i2c_msl | i2c_busy)) == 0u);
    H::reset();

    // ---- each line's rise from the pad, the rail on ----
    const Rise scl = rise_of<SclPin>();
    const Rise sda = rise_of<SdaPin>();
    print(serial, "  rise from low, released, the rail on: SCL ", scl.cycles, " cycles (", scl.polls,
          " polls), SDA ", sda.cycles, " cycles (", sda.polls, " polls) at 48 MHz - a 400 kHz bit is 120", crlf);
    // THE BOUND is the claim's own quantity, a 400 kHz bit (2.5 us, 120
    // cycles): the figure holds the stamps' own instructions (a rise faster
    // than them reads as zero polls), so it reads high, never low. The I2C
    // specification's 300 ns for a fast-mode rise is 14 cycles, under the
    // instrument's own figure, and is not judged here.
    constexpr uint32_t bit_400k = SysClock::hz / 400'000u;
    bench.verdict("with the rail on, each line rises from the pad within a 400 kHz bit, the stamps' own "
                  "cycles included (the 1.5 kOhm resistors to PB3)",
                  scl.cycles < bit_400k && sda.cycles < bit_400k);

    // ---- THE USB PADS' PULLS under the alternate function, the rail OFF ----
    // (RM 8.3.2.4: "only pull-up mode is set here", the GPIO configuration
    // deciding whether it acts.) The two lines are tied through the two
    // resistors and the floating PB3, so what is measured is the PAIR.
    rail_off();
    H::bus_clock(true);
    Afio::usb_pad_pull_ups(UsbPadPull::none, UsbPadPull::none);
    const Levels none_af = released_levels<true>(true);
    Afio::usb_pad_pull_ups(UsbPadPull::r1k5, UsbPadPull::r1k5);
    const Levels af1 = released_levels<true>(true);
    const Levels af0 = released_levels<true>(false);
    const Levels in = released_levels<false>(true);
    Afio::usb_pad_pull_ups(UsbPadPull::r10k, UsbPadPull::r10k);
    const Levels af10 = released_levels<true>(true);
    const Levels in10 = released_levels<false>(true);
    Afio::usb_pad_pull_ups(UsbPadPull::weak, UsbPadPull::weak);   // the reset modes
    const Levels af_weak = released_levels<true>(true);
    SclPin::release();
    SdaPin::release();
    H::bus_clock(false);
    rail_on();
    (void)delay_us(clock, 200);
    const auto hl = [](bool v) { return v ? "high" : "low"; };
    print(serial, "  the rail OFF, both lines driven low then released; read 2 ms later (SCL/SDA):", crlf);
    print(serial, "    modes none, alternate function:           ", hl(none_af.scl), "/", hl(none_af.sda), crlf);
    print(serial, "    modes 1.5k, alternate function, OUTDR 1:  ", hl(af1.scl), "/", hl(af1.sda),
          "; OUTDR 0: ", hl(af0.scl), "/", hl(af0.sda), crlf);
    print(serial, "    modes 10k,  alternate function:           ", hl(af10.scl), "/", hl(af10.sda), crlf);
    print(serial, "    modes weak, alternate function:           ", hl(af_weak.scl), "/", hl(af_weak.sda), crlf);
    print(serial, "    modes 1.5k, PULLED INPUT:                 ", hl(in.scl), "/", hl(in.sda),
          "; modes 10k: ", hl(in10.scl), "/", hl(in10.sda), crlf);
    const bool act = af1.scl && af1.sda;
    // A measurement, printed: either answer is a finding, so no verdict.
    print(serial, "  -> the USB pads' pull-ups ", act ? "ACT" : "do NOT act", " under the I2C's alternate function",
          crlf);
    bench.verdict("the control: the 1.5 kOhm and 10 kOhm modes pull both pads up as PULLED INPUTS",
                  in.scl && in.sda && in10.scl && in10.sda);
    bench.verdict("with no pull mode the released pair stays low (nothing else holds the bus up with "
                  "the rail off)",
                  !none_af.scl && !none_af.sda);
    print(serial, "  the rail back on: the wire reads ", wire_pulled_up() ? "high" : "LOW", crlf);
}

// ===========================================================================
// b - the scan
// ===========================================================================

void tb_scan() {
    if (!wire_pulled_up()) {
        bench.skip("no pull-ups on the wire");
        return;
    }
    host_ready();
    uint8_t found[8];
    uint8_t n_found = 0;
    uint8_t nacks = 0;
    uint8_t others = 0;
    const uint32_t t0 = Ticker::millis();
    for (uint8_t a = 0x08; a <= 0x77; ++a) {
        const uint8_t st = host_tenure(a, nullptr, 0, nullptr, 0, I2cSpeed::standard_100k);
        if (st == i2c_ok) {
            if (n_found < 8u) {
                found[n_found] = a;
            }
            ++n_found;
        } else if (st == i2c_nack_addr) {
            ++nacks;
        } else {
            ++others;
        }
    }
    const uint32_t took = Ticker::millis() - t0;
    print(serial, "  112 addresses probed in ", took, " ms: ", n_found, " answered, ", nacks,
          " nobody-home, ", others, " other", crlf);
    for (uint8_t i = 0; i < n_found && i < 8u; ++i) {
        print(serial, "    ", hex(found[i]),
              found[i] == twilink::command_addr ? "  <- twi_peer's command address" : "", crlf);
    }
    bench.verdict("every probe ended one way or the other (no stall, no other status)", others == 0u);
    bench.verdict("an address nobody answers is i2c_nack_addr - the scanner's probe result",
                  nacks >= 100u);
    bench.verdict("the peer's command address answered, and it alone",
                  n_found == 1u && found[0] == twilink::command_addr);
}

// ===========================================================================
// c - the peer link
// ===========================================================================

void tc_peer_link() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping over the two wires - every command is TWO TENURES "
                  "of the engine under test",
                  true);
    twilink::Frame f;
    const bool got = query(Op::ident, f) && f.op == Op::ident_data && f.len == twilink::ident_size;
    if (got) {
        const auto id = twilink::get_ident(f.data);
        char label[9] = {};
        for (uint8_t i = 0; i < 8; ++i) {
            label[i] = id.label[i];
        }
        print(serial, "  peer: label '", label, "' xtal=", id.xtal, " sanity=", hex(id.sanity),
              " fw=", hex(id.version), crlf);
        bench.verdict("ident comes back and it IS twi_peer on a CH32V006 (the sanity byte, kind 0x06)",
                      id.sanity == twilink::ident_sanity &&
                          twilink::peer_kind(id.version) == twilink::peer_ch32v00x);
    } else {
        bench.verdict("ident comes back", false);
    }
    uint8_t good = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (command(Op::ping)) {
            ++good;
        }
    }
    print(serial, "  ", good, " of 10 command round trips answered at ", Host::scl_hz(link_speed) / 1000u,
          " kHz", crlf);
    bench.verdict("the channel is steady over ten command round trips", good == 10u);
}

// ===========================================================================
// d - the tenure shapes against the peer
// ===========================================================================

void td_shapes() {
    if (!need_peer()) {
        return;
    }
    const twilink::Params a = serve_params(64, 250, 0x30);
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve command", false);
        return;
    }
    uint8_t w[8];
    for (uint8_t i = 0; i < 8; ++i) {
        w[i] = static_cast<uint8_t>(0x11u * (i + 1u));
    }
    const uint8_t ws = host_tenure(twilink::dut_addr, w, 8, nullptr, 0, link_speed);
    for (uint8_t i = 0; i < 8; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t rs = host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 8, link_speed);
    uint8_t rmism = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        if (rx_buf[i] != twilink::pattern_value(a.pattern, a.seed, i)) {
            ++rmism;
        }
    }
    for (uint8_t i = 0; i < 4; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0xA0u + i);
        rx_buf[i] = 0xEE;
    }
    const uint8_t cs = host_tenure(twilink::dut_addr, tx_buf, 4, rx_buf, 4, link_speed);
    settle_ms(300);
    twilink::Report r{};
    const bool rep = peer_report(r);
    twilink::Counters k{};
    const bool kc = peer_counters(k);
    print(serial, "  write=", ws, " read=", rs, " mism=", rmism, " combined=", cs, "; peer: count=",
          r.count, " addr_hits=", r.addr_hits, " first=", hex(r.first), " sum=", hex(r.sum),
          "; the peer's target dropped ", k.drops, " byte(s) at the closing NACKs of the action (",
          kc ? "counters answered" : "NO counters", ")", crlf);
    bench.verdict("a write tenure, a read tenure and the combined write-then-read all complete "
                  "i2c_ok against a SECOND CHIP",
                  ws == i2c_ok && rs == i2c_ok && cs == i2c_ok);
    bench.verdict("the bytes read back are the peer's own pattern, byte-exact", rmism == 0u);
    bench.verdict("the peer accounts every byte of the three tenures (8 written, 8 served, then 4 "
                  "and 4)",
                  rep && r.count == 24u);
    bench.verdict("and the COMBINED tenure hit the client's address machinery TWICE - the repeated "
                  "START, counted from the far end",
                  rep && r.addr_hits == 4u);
    if (kc) {
        bench.verdict("THE TARGET ASKED ONE BYTE MORE: each of the two read tenures left one byte in the "
                      "peer's DATAR past this host's closing NACK, and the peer's flush() dropped it (its "
                      "counters) - the bytes read above are exact all the same",
                      k.drops == 2u && rmism == 0u);
    } else {
        bench.skip("the peer does not serve the counters query: the drops not judged");
    }

    bool counts_ok = true;
    for (uint8_t n = 1; n <= 4u; ++n) {
        const twilink::Params c = serve_params(64, 250, static_cast<uint8_t>(0x40u + n));
        if (!peer_act(Op::serve, c)) {
            counts_ok = false;
            break;
        }
        for (uint8_t i = 0; i < 4; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint8_t st = host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, n, link_speed);
        uint8_t mism = 0;
        for (uint8_t i = 0; i < n; ++i) {
            if (rx_buf[i] != twilink::pattern_value(c.pattern, c.seed, i)) {
                ++mism;
            }
        }
        settle_ms(300);
        twilink::Report cr{};
        const bool crep = peer_report(cr);
        print(serial, "  read of ", n, ": status=", st, " mism=", mism, " peer count=", cr.count, crlf);
        if (st != i2c_ok || mism != 0u || !crep || cr.count != n) {
            counts_ok = false;
        }
    }
    bench.verdict("reads of ONE, TWO, THREE and FOUR bytes - the chapter's four receive procedures - "
                  "each byte-exact and each closed at the right byte (the peer counts no extra)",
                  counts_ok);

    // THE COUNTED SERVE (twi_link.hpp's `count`): a serve armed for eight
    // bytes and a deadline far beyond them must end once the read that
    // moved the eighth is over - every byte the peer's own, the last one
    // included, and the peer back in command mode long before its
    // deadline. The bound: the read takes under a millisecond at 100 kHz,
    // the peer's exit wants forty microseconds of wire at rest and its
    // re-init well under a millisecond, and the poll below looks every
    // 5 ms (a ping the serving peer does not answer fails at its address
    // byte), so the counted exit is seen within some 10 ms and a serve that
    // waited for its deadline at no less than 2000 ms; half the deadline
    // parts the two with a factor of a hundred on the early side.
    {
        constexpr uint32_t deadline_ms = 2000;
        const twilink::Params cnt = serve_params(8, deadline_ms, 0x58);
        if (peer_act(Op::serve, cnt)) {
            for (uint8_t i = 0; i < 8; ++i) {
                rx_buf[i] = 0xEE;
            }
            const uint8_t st = host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 8, link_speed);
            // The peer polled by pings until one is answered, within the
            // deadline and half a second more; the verdict judges both.
            const uint32_t t0 = Ticker::millis();
            const bool back = brio::wait_until<Ruler>(
                [] {
                    if (command_once(Op::ping, no_payload, 0)) {
                        return true;
                    }
                    link_ready();
                    settle_ms(5);
                    return false;
                },
                (deadline_ms + 500u) * 1000u);
            const uint32_t back_ms = Ticker::millis() - t0;
            uint8_t mism = 0;
            for (uint8_t i = 0; i < 8; ++i) {
                if (rx_buf[i] != twilink::pattern_value(cnt.pattern, cnt.seed, i)) {
                    ++mism;
                }
            }
            twilink::Report cr{};
            const bool crep = peer_report(cr);
            print(serial, "  counted serve of 8: read status=", st, " mism=", mism, " last=",
                  hex(rx_buf[7]), "; the peer back in ", back_ms, " ms of its ", deadline_ms,
                  " ms deadline, its count=", cr.count, " stops=", cr.stops, " flags=",
                  hex(cr.flags), crlf);
            bench.verdict("a COUNTED SERVE ends once the read that moved its count is over: the "
                          "eight bytes the peer's own, the last included, and the peer back in "
                          "command mode well inside half its deadline, not timed out",
                          st == i2c_ok && mism == 0u && back && back_ms < deadline_ms / 2u &&
                              crep && cr.count == 8u &&
                              (cr.flags & twilink::report_timed_out) == 0u);
        } else {
            bench.verdict("the peer accepted the counted serve", false);
        }
    }

    // The addresses no Request may name (util/i2c_bus.hpp): 0x7A, a 10-bit
    // prefix this block would send as a header, and the general call,
    // refused by start() itself - true, i2c_rejected, nothing armed -
    // against a peer at dut_addr ANSWERING THE GENERAL CALL, so a byte of
    // either on the wire would be an address match on its count; the
    // write to dut_addr after them is its one match and its two bytes.
    twilink::Params g = serve_params(64, 250, 0);
    g.flags = twilink::flag_general_call;
    if (peer_act(Op::serve, g)) {
        uint8_t gw[2] = {0x5A, 0xA5};
        Host::Request bad{};
        bad.addr = 0x7A;
        bad.tx = lend<Lease::reply>(static_cast<const uint8_t*>(gw));
        bad.tx_len = 2;
        bad.speed = link_speed;
        const bool sync_prefix = Host::start(bad);
        const uint8_t st_prefix = Host::status();
        bad.addr = 0x00;
        const bool sync_call = Host::start(bad);
        const uint8_t st_call = Host::status();
        const uint8_t own = host_tenure(twilink::dut_addr, gw, 2, nullptr, 0, link_speed);
        settle_ms(300);
        twilink::Report gr{};
        const bool grep = peer_report(gr);
        print(serial, "  0x7A: ", sync_prefix ? "refused at start()" : "STARTED", " ", st_prefix, "; 0x00: ",
              sync_call ? "refused at start()" : "STARTED", " ", st_call, "; then dut_addr ", own,
              "; peer: addr_hits=", gr.addr_hits, " count=", gr.count, " last_addr=", hex(gr.last_addr), crlf);
        bench.verdict("0x7A and the general call answered i2c_rejected inside start(), nothing on the wire: "
                      "the peer, answering the general call, took only the write to its own address",
                      sync_prefix && st_prefix == i2c_rejected && sync_call && st_call == i2c_rejected &&
                          own == i2c_ok && grep && gr.addr_hits == 1u && gr.count == 2u &&
                          gr.last_addr == twilink::dut_addr);
    } else {
        bench.verdict("the peer accepted the general-call serve", false);
    }
}

// ===========================================================================
// e - the vocabulary on the wire, and commanded stretching
// ===========================================================================

void te_vocabulary() {
    if (!need_peer()) {
        return;
    }
    twilink::Params a = serve_params(8, 600, 0);
    a.flags = twilink::flag_deaf;
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the deaf serve", false);
        return;
    }
    uint8_t w[4] = {1, 2, 3, 4};
    const uint8_t deaf = host_tenure(twilink::dut_addr, w, 4, nullptr, 0, link_speed);
    const uint8_t probe = host_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, link_speed);
    settle_ms(700);

    twilink::Params n = serve_params(16, 600, 0);
    n.nack_at = 3;
    if (!peer_act(Op::serve, n)) {
        bench.verdict("the peer accepted the nack-at serve", false);
        return;
    }
    uint8_t w6[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
    const uint8_t nack = host_tenure(twilink::dut_addr, w6, 6, nullptr, 0, link_speed);
    settle_ms(700);
    twilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  deaf=", deaf, " probe=", probe, " data-nack=", nack, " peer flags=", hex(r.flags),
          " count=", r.count, crlf);
    bench.verdict("an address nobody answers reports i2c_nack_addr - on a board that is otherwise "
                  "alive",
                  deaf == i2c_nack_addr && probe == i2c_nack_addr);
    bench.verdict("a commanded NACK on the 3rd data byte reports i2c_nack_data",
                  nack == i2c_nack_data && rep && (r.flags & twilink::report_nacked) != 0u);

    const twilink::Params base = serve_params(128, 700, 0);
    if (!peer_act(Op::serve, base)) {
        bench.verdict("the peer accepted the baseline serve", false);
        return;
    }
    uint8_t w8[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint32_t t0 = Ticker::millis();
    bool base_ok = true;
    for (uint8_t k = 0; k < 8; ++k) {
        if (host_tenure(twilink::dut_addr, w8, 8, nullptr, 0, link_speed) != i2c_ok) {
            base_ok = false;
        }
    }
    const uint32_t base_ms = Ticker::millis() - t0;
    settle_ms(750);

    twilink::Params st = serve_params(64, 150, 0);
    st.hold_us = 2000;
    if (!peer_act(Op::serve, st)) {
        bench.verdict("the peer accepted the stretched serve", false);
        return;
    }
    // The span judged behind a console that went idle, and failed behind one
    // that did not (its vector inside the span).
    const bool drained = console_drain();
    const uint32_t c0 = Ticker::cycles();
    const uint8_t s2 = host_tenure(twilink::dut_addr, w8, 8, nullptr, 0, link_speed);
    const uint32_t stretched = Ticker::cycles() - c0;
    settle_ms(300);
    twilink::Report sr{};
    const bool srep = peer_report(sr);
    // THE PEER STRETCHES ONE BYTE LATE (its header): a byte left in its
    // DATAR stalls the bus only once the NEXT one has been taken into the
    // shifter and acknowledged, so of an 8-byte write the address's hold
    // and those of the first six bytes are on the wire, and the last two
    // fall after the tenure: seven holds of 2 ms.
    //
    // THE WINDOW FOR SEVEN HOLDS, the wire subtracted rather than rounded
    // away: seven holds of 2 ms on the peer's clock, peer_lo_us(14'000) = 13482
    // to peer_hi_us(14'000) = 14518 us here; the wire's 81 SCL periods (the
    // address and eight bytes) at the rates letter f passes, 2 % above 100 kHz
    // (9.8 us) to a quarter below it (13.3 us), 794 to 1080 us; above them the
    // START, the STOP and the software around them, under 25 us (letter j's
    // arithmetic, race_behind_us), and the peer's turn at each of its eight
    // stretches (its polled loop seeing the byte before the hold begins, and
    // the hold's own last look: a few flag reads and a ruler read, under 5 us
    // at 48 MHz), 40 us - allowed 200 in all. Seven holds read 14276 to
    // 15798 us; six at most 12444 + 1280 = 13724, eight at least 15408 + 794 =
    // 16202, both outside.
    constexpr uint32_t seven_lo_us = peer_lo_us(14'000) + 794u;
    constexpr uint32_t seven_hi_us = peer_hi_us(14'000) + 1080u + 200u;
    static_assert(seven_lo_us == 14'276 && seven_hi_us == 15'798, "the arithmetic above");
    static_assert(peer_hi_us(12'000) + 1280u < seven_lo_us && peer_lo_us(16'000) + 794u > seven_hi_us,
                  "six and eight holds fall outside the window");
    const uint32_t stretched_us = stretched / (SysClock::hz / 1'000'000u);
    print(serial, "  8 x 8 bytes unstretched: ", base_ms, " ms; ONE 8-byte tenure at 2 ms per byte: ",
          stretched_us, " us, seven holds read ", seven_lo_us, " to ", seven_hi_us, " us; peer count=",
          sr.count, crlf);
    bench.verdict("the unstretched baseline tenures all complete i2c_ok", base_ok);
    bench.verdict("a client stretching every data byte by 2 ms stretches the WALL TIME by the seven holds "
                  "a target one byte late puts on the wire, and the tenure still completes i2c_ok - "
                  "stretching is flow control",
                  drained && s2 == i2c_ok && stretched_us >= seven_lo_us && stretched_us <= seven_hi_us &&
                      srep && sr.count >= 8u);
}

// ===========================================================================
// f - the two speeds against a second chip
// ===========================================================================

void tf_speeds() {
    if (!need_peer()) {
        return;
    }
    struct Rung {
        I2cSpeed speed;
        const char* name;
    };
    const Rung rungs[] = {{I2cSpeed::standard_100k, "100k"}, {I2cSpeed::fast_400k, "400k"}};
    uint8_t exact = 0;
    uint8_t bracketed = 0;
    bool drained = true;
    for (uint8_t i = 0; i < 2u; ++i) {
        const twilink::Params a = serve_params(64, 250, static_cast<uint8_t>(0x50u + i * 0x10u));
        if (!peer_act(Op::serve, a)) {
            bench.verdict("the peer accepted the serve for this rung", false);
            return;
        }
        for (uint8_t k = 0; k < 8; ++k) {
            tx_buf[k] = static_cast<uint8_t>(0xC0u + k);
            rx_buf[k] = 0xEE;
        }
        // A console still sending would put its vector inside the timed write:
        // the rate is judged behind a console that went idle, and fails behind
        // one that did not.
        drained = console_drain() && drained;
        const uint32_t c0 = Ticker::cycles();
        const uint8_t ws = host_tenure(twilink::dut_addr, tx_buf, 8, nullptr, 0, rungs[i].speed);
        const uint32_t cycles = Ticker::cycles() - c0;
        const uint8_t rs = host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 8, rungs[i].speed);
        uint8_t mism = 0;
        for (uint8_t k = 0; k < 8; ++k) {
            if (rx_buf[k] != twilink::pattern_value(a.pattern, a.seed, k)) {
                ++mism;
            }
        }
        settle_ms(300);
        const bool ok = ws == i2c_ok && rs == i2c_ok && mism == 0u;
        const uint32_t measured = scl_of(cycles, 81u);
        const uint32_t asked = i2c_speed_hz(rungs[i].speed);
        print(serial, "  ", rungs[i].name, ": SCL asked ", asked / 1000u, " kHz (the chooser's ",
              Host::scl_hz(rungs[i].speed) / 1000u, "), an 8-byte write took ", cycles,
              " cycles -> about ", measured / 1000u, " kHz on the wire; write=", ws, " read=", rs,
              " mism=", mism, crlf);
        if (ok) {
            ++exact;
        }
        if (ws == i2c_ok && scl_bracket(measured, asked)) {
            ++bracketed;
        }
    }
    bench.verdict("Standard mode and Fast mode both carry a write and a read byte-exact between "
                  "TWO SEPARATE CHIPS",
                  exact == 2u);
    bench.verdict("and each runs the bus at the rate asked: an 8-byte write's SCL never above 100 or "
                  "400 kHz by more than 2 % and within a quarter below it",
                  drained && bracketed == 2u);

    // DUTY 16/9: fast mode with the low phase sixteen parts of twenty-five.
    // CCR = 48 MHz / (25 x 400 kHz) = 4.8, rounded UP to 5 so the bus never
    // runs faster than asked (the engine's rule): SCL = 48 MHz / 125 = 384
    // kHz, the chooser's own figure. The write timed as above and judged
    // against the CHOOSER's rate by the same bracket; CKCFGR read back.
    {
        const twilink::Params dp = serve_params(64, 250, 0x68);
        if (peer_act(Op::serve, dp)) {
            (void)Host::init(clock, I2cDuty::ratio_16_9);
            for (uint8_t k = 0; k < 8; ++k) {
                tx_buf[k] = static_cast<uint8_t>(0xA0u + k);
                rx_buf[k] = 0xEE;
            }
            // The rate judged behind a console that went idle (as above).
            const bool duty_drained = console_drain();
            const uint32_t c0 = Ticker::cycles();
            const uint8_t ws = host_tenure(twilink::dut_addr, tx_buf, 8, nullptr, 0, I2cSpeed::fast_400k);
            const uint32_t cycles = Ticker::cycles() - c0;
            const uint16_t ckcfgr = H::regs().CKCFGR;
            const uint32_t chooser = Host::scl_hz(I2cSpeed::fast_400k);
            const uint8_t rs = host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 8, I2cSpeed::fast_400k);
            (void)Host::init(clock);
            uint8_t mism = 0;
            for (uint8_t k = 0; k < 8; ++k) {
                if (rx_buf[k] != twilink::pattern_value(dp.pattern, dp.seed, k)) {
                    ++mism;
                }
            }
            settle_ms(300);
            const uint32_t measured = scl_of(cycles, 81u);
            print(serial, "  400k at DUTY 16/9: CKCFGR ", hex(ckcfgr), ", the chooser's ", chooser / 1000u,
                  " kHz, an 8-byte write took ", cycles, " cycles -> about ", measured / 1000u,
                  " kHz on the wire; write=", ws, " read=", rs, " mism=", mism, crlf);
            bench.verdict("FAST MODE AT DUTY 16/9 carries a write and a read byte-exact, CKCFGR holding F/S, "
                          "DUTY and CCR 5, the write's SCL bracketed against the chooser's 384 kHz",
                          ws == i2c_ok && rs == i2c_ok && mism == 0u && ckcfgr == (i2c_fs | i2c_duty | 5u) &&
                              chooser == 384'000u && duty_drained && scl_bracket(measured, chooser));
        } else {
            bench.verdict("the peer accepted the serve for the DUTY 16/9 rung", false);
        }
    }

    // PAST THE CHAPTER: CKCFGR at fast mode's CCR 16 - 1 MHz at 48 MHz -
    // which 15.1's two speeds do not include. Measured, not judged.
    const twilink::Params fp = serve_params(64, 250, 0x70);
    if (peer_act(Op::serve, fp)) {
        (void)host_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, I2cSpeed::fast_400k);
        H::disable();
        H::regs().CKCFGR = static_cast<uint16_t>(i2c_fs | 16u);
        H::enable();
        for (uint8_t k = 0; k < 8; ++k) {
            tx_buf[k] = static_cast<uint8_t>(0x90u + k);
            rx_buf[k] = 0xEE;
        }
        // Measured, not judged: a console that did not drain is said beside it.
        const bool past_drained = console_drain();
        const uint32_t c0 = Ticker::cycles();
        const uint8_t ws = host_tenure(twilink::dut_addr, tx_buf, 8, nullptr, 0, I2cSpeed::fast_400k);
        const uint32_t cycles = Ticker::cycles() - c0;
        const uint8_t rs = host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 8, I2cSpeed::fast_400k);
        uint8_t mism = 0;
        for (uint8_t k = 0; k < 8; ++k) {
            if (rx_buf[k] != twilink::pattern_value(fp.pattern, fp.seed, k)) {
                ++mism;
            }
        }
        host_ready();
        settle_ms(300);
        twilink::Report r{};
        const bool rep = peer_report(r);
        print(serial, "  past the chapter, CCR 16 (1 MHz asked): an 8-byte write took ", cycles,
              " cycles -> about ", scl_of(cycles, 81u) / 1000u, " kHz; write=", ws, " read=", rs,
              " mism=", mism, "; peer sum ", hex(r.sum), rep ? "" : " (no report)", " (",
              hex(static_cast<uint16_t>(0x90u * 8u + 28u)), " sent) - measured, not judged",
              past_drained ? "" : " (the console had not drained)", crlf);
    }
    bench.verdict("the command channel survives the ladder", command(Op::ping));
}

// ===========================================================================
// g - the DMA engines against the peer
// ===========================================================================

void tg_dma() {
    if (!need_peer()) {
        return;
    }
    const twilink::Params a = serve_params(128, 400, 0x60);
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve", false);
        return;
    }
    owner = Owner::dma_host;
    (void)DmaHost::init(clock);
    for (uint8_t i = 0; i < 16; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x80u + i);
        rx_buf[i] = 0xEE;
    }
    dma6_entries = 0;
    dma7_entries = 0;
    const uint32_t tx_faults0 = DmaTxEngine<6>::faults();
    const uint32_t rx_faults0 = DmaRxEngine<7>::faults();
    // A console still sending would put its vector inside the timed tenures:
    // the rates are judged behind a console that went idle, and fail behind one
    // that did not. Nothing prints between here and the last timed tenure but a
    // STALL line, whose tenure fails the verdict already.
    const bool drained = console_drain();
    uint32_t c0 = Ticker::cycles();
    const uint8_t ws = dma_tenure(twilink::dut_addr, tx_buf, 16, nullptr, 0, link_speed);
    const uint32_t w_scl = scl_of(Ticker::cycles() - c0, 153u);
    const uint32_t w_entries = host_isr_entries;
    c0 = Ticker::cycles();
    const uint8_t rs = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 16, link_speed);
    const uint32_t r_scl = scl_of(Ticker::cycles() - c0, 153u);
    const uint32_t r_entries = host_isr_entries;
    uint8_t mism = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (rx_buf[i] != twilink::pattern_value(a.pattern, a.seed, i)) {
            ++mism;
        }
    }
    for (uint8_t i = 0; i < 4; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t cs = dma_tenure(twilink::dut_addr, tx_buf, 4, rx_buf, 4, link_speed);
    const uint8_t r2 = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf + 8, 2, link_speed);
    const uint8_t r1 = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf + 12, 1, link_speed);
    for (uint8_t i = 0; i < 16; ++i) {
        rx_buf[16 + i] = 0xEE;
    }
    c0 = Ticker::cycles();
    const uint8_t fs = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf + 16, 16, I2cSpeed::fast_400k);
    const uint32_t f_scl = scl_of(Ticker::cycles() - c0, 153u);
    uint8_t fmism = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (rx_buf[16 + i] != twilink::pattern_value(a.pattern, a.seed, static_cast<uint16_t>(23u + i))) {
            ++fmism;
        }
    }
    const uint32_t e6 = dma6_entries;
    const uint32_t e7 = dma7_entries;
    const uint32_t tx_faults = DmaTxEngine<6>::faults() - tx_faults0;
    const uint32_t rx_faults = DmaRxEngine<7>::faults() - rx_faults0;
    DmaHost::release();
    host_ready();
    settle_ms(450);
    twilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  DMA: write16=", ws, " (", w_entries, " event entries) read16=", rs, " (", r_entries,
          ") mism=", mism, " combined=", cs, " read2=", r2, " read1=", r1, " read16 at 400k=", fs,
          " mism=", fmism, crlf);
    print(serial, "  peer count=", r.count, " addr_hits=", r.addr_hits, "; faults this run tx=", tx_faults,
          " rx=", rx_faults, "; DMA interrupts ch6=", e6, " ch7=", e7, crlf);
    print(serial, "  SCL on the engines: the 16-byte write ", w_scl / 1000u, " kHz and read ", r_scl / 1000u,
          " kHz (100 asked), the read at 400k ", f_scl / 1000u, " kHz", crlf);
    bench.verdict("a 16-byte write and a 16-byte read through the DMA engines complete i2c_ok, "
                  "byte-exact",
                  ws == i2c_ok && rs == i2c_ok && mism == 0u);
    bench.verdict("the combined tenure, the two-byte read (POS's NACK) and the one-byte read (its NACK "
                  "and STOP from the ADDR entry) all complete on the engines",
                  cs == i2c_ok && r2 == i2c_ok && r1 == i2c_ok);
    bench.verdict("a 16-byte read at 400 kHz on the engines completes i2c_ok, the serve's next sixteen "
                  "bytes exact",
                  fs == i2c_ok && fmism == 0u);
    bench.verdict("the peer accounts every byte (16 + 16 + 4 + 4 + 2 + 1 + 16 = 59), and this run counts "
                  "no transfer fault",
                  rep && r.count == 59u && tx_faults == 0u && rx_faults == 0u);
    bench.verdict("each write block interrupts once, its completion handing the half back (the 16-byte "
                  "write and the combined's write half; SB, ADDR and BTF on the event vector), each of "
                  "the five engined reads once (SB and ADDR on the event vector)",
                  e6 == 2u && e7 == 5u && w_entries == 3u && r_entries == 2u);
    bench.verdict("and the engines run the bus at the rate asked: SCL never above 100 or 400 kHz by more "
                  "than 2 % and within a quarter below it",
                  drained && ws == i2c_ok && rs == i2c_ok && fs == i2c_ok && scl_bracket(w_scl, 100'000u) &&
                      scl_bracket(r_scl, 100'000u) && scl_bracket(f_scl, 400'000u));
}

// ===========================================================================
// h - the kernel against the peer
// ===========================================================================

namespace kl {

constexpr uint32_t bus_timeout_ticks = ticks_from_ms<P>(20);
static_assert(P::ticks_per_second == 1000u, "the letters count milliseconds as ticks");
using I2cArb = I2cBus<Host, P, 4, BusPassThrough, bus_timeout_ticks>;
/// The arbiter with no per-bus timeout, `timeout_ticks` 0: letter t's alone.
using I2cArbUntimed = I2cBus<Host, P, 4>;

uint8_t out_a[4];
uint8_t out_b[4];

class Probe {
public:
    using Event = std::variant<I2cDone, SleepVote>;
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies[8];
    static inline uint8_t n = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;

    static void init() { clear_tally(); }
    static void clear_tally() {
        n = 0;
        rejected = 0;
        votes = 0;
        last_vote = false;
    }

    static void dispatch(const Event& e) {
        brio::match(
            e,
            [](const I2cDone& d) {
                if (n < 8u) {
                    replies[n] = d.status;
                }
                ++n;
                if (d.status == i2c_rejected) {
                    ++rejected;
                }
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
            });
    }
};

using BusKernel = std::conditional_t<selection_carries(test_letters, 't'), Tenuto<P, Probe, I2cArb, I2cArbUntimed>,
                                     Tenuto<P, Probe, I2cArb>>;

void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

/// The kernel pumped until `want` replies came or `ms` ran out, on the
/// kernel tick (a tick a millisecond); the caller judges Probe::n.
void pump_until(uint8_t want, uint32_t ms) {
    (void)brio::wait_ticks<P>(
        [want] {
            pump();
            return Probe::n >= want;
        },
        ms);
    pump();
}

/// The kernel pumped for `ms`, whatever comes: a span of work, not a wait.
void drain(uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        pump();
    }
}

Host::Request request(uint8_t addr, const uint8_t* tx) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = 4;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    r.rx_len = 0;
    r.speed = I2cSpeed::fast_400k;
    r.reply = reply_to<Probe, I2cDone>();
    return r;
}

}  // namespace kl

void fill_pattern(uint8_t* p, uint16_t n, uint8_t seed) {
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + i * 7u + (i >> 3));
    }
}

void th_kernel() {
    if (!need_peer()) {
        return;
    }
    const twilink::Params a = serve_params(512, 900, 0x70);
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve the kernel letter drives", false);
        return;
    }
    kl::BusKernel::init_all();
    owner = Owner::bus_ao;

    fill_pattern(kl::out_a, 4, 0x01);
    fill_pattern(kl::out_b, 4, 0x02);
    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::I2cArb>(kl::request((i == 2u) ? nobody_addr : twilink::dut_addr, kl::out_a));
    }
    kl::pump_until(4, 400);
    print(serial, "  four queued tenures: replies ", kl::Probe::n, " [", kl::Probe::replies[0], " ",
          kl::Probe::replies[1], " ", kl::Probe::replies[2], " ", kl::Probe::replies[3], "]", crlf);
    bench.verdict("four tenures through I2cBus against a SECOND CHIP, four replies - util/i2c_bus.hpp "
                  "and util/bus_master.hpp unchanged on this core",
                  kl::Probe::n == 4u);
    bench.verdict("... in order, with the NACK from an address nobody answers delivered IN ITS PLACE",
                  kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_ok &&
                      kl::Probe::replies[2] == i2c_nack_addr && kl::Probe::replies[3] == i2c_ok);

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_b));
    }
    kl::pump_until(6, 400);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n, ", rejected ",
          kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately, and every request is still "
                  "answered exactly once",
                  kl::Probe::rejected != 0u && kl::Probe::n == 6u);

    kl::drain(100);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(PrepareSleep{.depth = SleepDepth::standby, .reply = reply_to<kl::Probe, SleepVote>()});
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep", kl::Probe::votes == 1u && kl::Probe::last_vote);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    post<kl::I2cArb>(PrepareSleep{.depth = SleepDepth::standby, .reply = reply_to<kl::Probe, SleepVote>()});
    kl::pump();
    kl::drain(150);
    bench.verdict("a BUSY bus votes against it", kl::Probe::votes == 1u && !kl::Probe::last_vote);

    // THE TIMED BUS, with the wedge held by the OTHER BOARD.
    owner = Owner::host;
    settle_ms(400);
    twilink::Params h{};
    h.ms = 300;
    const bool armed = peer_act(Op::hold_sda, h);
    owner = Owner::bus_ao;
    if (!armed) {
        bench.verdict("the peer accepted the hold_sda command", false);
        owner = Owner::host;
        return;
    }
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    const uint32_t t0 = Ticker::ticks();
    kl::pump_until(1, 400);
    const uint32_t took = Ticker::ticks() - t0;
    const bool still_low = !SdaPin::read();
    print(serial, "  the wedged tenure answered ", kl::Probe::n, " with status ", kl::Probe::replies[0],
          " after ", took, " ms; SDA still low at the reply: ", still_low ? "yes" : "no", " -> ",
          kl::Probe::replies[0] == i2c_arb_lost
              ? "i2c_arb_lost, the ENGINE's own wire code"
              : (kl::Probe::replies[0] == i2c_timeout ? "i2c_timeout, the ARBITER's" : "another code"),
          crlf);
    // THE ARBITER'S LIMIT, in ticks from the post: its one-shot is armed at the
    // request's dispatch, inside the pump's first turn, with the deadline the
    // tick count then plus bus_timeout_ticks (kernel/time_event.hpp), and
    // matures at that count; the pump sees it in its next turn, recover() and
    // the reply's dispatch are microseconds (ch32x035/i2c.hpp). So `took` is
    // bus_timeout_ticks, plus one when a tick lands between the reading of t0
    // and the arm, plus one when one lands between the maturity and the
    // reply's reading: 22 at most. A loss the engine reports itself
    // (i2c_arb_lost) answers in under a millisecond, well inside it.
    constexpr uint32_t answered_by_ticks = kl::bus_timeout_ticks + 2u;
    bench.verdict("a tenure into a wire a FOREIGN CHIP holds down is answered IN ITS PLACE - never "
                  "silence and never i2c_ok - at the arbiter's own limit or sooner",
                  kl::Probe::n == 1u && kl::Probe::replies[0] != i2c_ok && took <= answered_by_ticks);

    owner = Owner::host;
    settle_ms(400);
    link_ready();
    const bool re = peer_act(Op::serve, serve_params(64, 250, 0x90));
    owner = Owner::bus_ao;
    kl::drain(20);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    kl::pump_until(1, 300);
    print(serial, "  after the release: replies ", kl::Probe::n, " status ", kl::Probe::replies[0], crlf);
    bench.verdict("THE SAME BUS AO carries the next tenure to i2c_ok after the wedge",
                  re && kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_ok);

    // THE CLOCK HELD past the limit: the F1 lineage's I2C has no clock-low
    // timeout, so the per-bus timeout must answer it.
    owner = Owner::host;
    settle_ms(400);
    link_ready();
    twilink::Params held = serve_params(16, 300, 0);
    held.hold_us = 45'000;
    const bool armed_clock = peer_act(Op::serve, held);
    owner = Owner::bus_ao;
    if (!armed_clock) {
        bench.verdict("the peer accepted the 45 ms stretch", false);
        owner = Owner::host;
        return;
    }
    kl::drain(5);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    const uint32_t t1 = Ticker::ticks();
    kl::pump_until(1, 400);
    const uint32_t took_clock = Ticker::ticks() - t1;
    print(serial, "  the tenure into a held CLOCK answered ", kl::Probe::n, " with status ",
          kl::Probe::replies[0], " after ", took_clock, " ms (limit 20, hold 45); SCL at the reply: ",
          SclPin::read() ? "high" : "LOW", crlf);
    // THE SAME LIMIT as the wedge's above: the one-shot armed at the
    // request's dispatch, after t1 was read, matures bus_timeout_ticks on
    // (never sooner), recover() waits for nothing (ch32x035/i2c.hpp), so
    // `took_clock` is bus_timeout_ticks to answered_by_ticks.
    bench.verdict("a client holding the CLOCK past the limit is answered i2c_timeout ON THE ARBITER'S "
                  "CLOCK - the one wedge the silicon cannot answer by itself",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_timeout &&
                      took_clock >= kl::bus_timeout_ticks && took_clock <= answered_by_ticks);
    kl::drain(100);
    owner = Owner::host;
    settle_ms(400);
    link_ready();
    const bool re2 = peer_act(Op::serve, serve_params(64, 250, 0x91));
    owner = Owner::bus_ao;
    kl::drain(20);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    kl::pump_until(1, 300);
    print(serial, "  after the clock came back: replies ", kl::Probe::n, " status ", kl::Probe::replies[0],
          crlf);
    bench.verdict("the recover()ed engine carries the next tenure to i2c_ok through the same arbiter",
                  re2 && kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_ok);
    owner = Owner::host;
    settle_ms(300);
    host_ready();
}

// ===========================================================================
// i - the refusal, wireless
// ===========================================================================

/// A speed the clock in force cannot make is refused INSIDE start() and
/// reaches the requester through the arbiter as i2c_rejected, with the
/// wire and the vector untouched. The clock itself is not moved: rebase()
/// is told a rate below 15.9.2's window, which empties the timing table.
void ti_refusal() {
    host_ready();
    kl::BusKernel::init_all();
    owner = Owner::bus_ao;
    Host::rebase(3'000'000UL);
    bench.verdict("told 3 MHz, speed_ok says no to both speeds (below 15.9.2's window)",
                  !Host::speed_ok(I2cSpeed::standard_100k) && !Host::speed_ok(I2cSpeed::fast_400k));
    kl::Probe::clear_tally();
    const uint32_t entries = host_isr_entries;
    fill_pattern(kl::out_a, 4, 0x05);
    post<kl::I2cArb>(kl::request(nobody_addr, kl::out_a));
    kl::pump_until(1, 100);
    print(serial, "  the request at 400 kHz replied ", kl::Probe::replies[0], " (i2c_rejected = ",
          i2c_rejected, "), STAR2=", hex(H::status2()), ", event vector entries ",
          host_isr_entries - entries, crlf);
    bench.verdict("answered i2c_rejected inside start() and delivered through the arbiter, no START "
                  "issued (MSL clear) and the vector never entered",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_rejected &&
                      host_isr_entries == entries && (H::status2() & i2c_msl) == 0u);
    bench.verdict("the arbiter's own tally stays at zero (its rejection is the full queue's)",
                  kl::I2cArb::rejected_count() == 0u);
    Host::rebase(SysClock::hz);
    bench.verdict("told 48 MHz again, both speeds are back",
                  Host::speed_ok(I2cSpeed::standard_100k) && Host::speed_ok(I2cSpeed::fast_400k));
    owner = Owner::host;
}

// ===========================================================================
// j - arbitration, and this board as the client
// ===========================================================================

/// A START CONDITION BY HAND: SDA pulled low from the port while SCL is
/// released high - a push-pull output over a clear bit, this series having
/// no open-drain GPIO. BUSY follows the WIRE at both ends.
void inject_start() { SdaPin::output(false); }
/// AND THE STOP THAT CLOSES IT: SDA given back to the peripheral, open
/// drain by itself, so it rises while SCL is still high.
void inject_stop() { SdaPin::function(); }

void hold_bus_busy(uint32_t us) {
    inject_start();
    brio::spin_us<Ruler>(us);
    inject_stop();
}

/// ONE ROUND OF THE RACE, and what says whether it happened. The two STARTs
/// are set within microseconds of each other, not at once: when one of them
/// is set once the other is on the wire it waits for that tenure's STOP, and
/// the wire carries two tenures one after the other, each a NACK on its
/// address, with nothing to arbitrate. Which of them this board can read:
/// its own tenure answered late (`done_us`, behind the peer's) or another
/// tenure on the wire after its own (`tail`, the peer's behind this one).
struct RaceRound {
    uint8_t status;
    uint32_t done_us;   ///< from the start() to the tenure's answer
    bool tail;
};

/// THE BOUND ON `done_us`. A tenure is nine SCL periods to its NACK. Letter f
/// judges the rate at most 2 % above 100 kHz and at most a quarter below it,
/// so 9 x 9.8 = 88 us at the shortest and 9 x 13.3 = 120 us at the longest.
/// Alone on the wire, this board's answer comes within 120 us plus the START
/// and the vector, about 25 us in all (a loss at the first bit answers in 30
/// at most): under 145. Behind the peer's tenure it is two tenures, 176 us at the
/// shortest. The bound lies between the two.
constexpr uint32_t race_behind_us = 160;
/// THE WATCH FOR A TENURE AFTER THIS BOARD'S. A START held behind this
/// board's STOP goes out within tens of microseconds and pulls SCL low
/// within a bit of that; 200 us covers both and is twice a whole tenure.
constexpr uint32_t race_tail_us = 200;
/// THE ATTEMPTS A ROUND MAY TAKE. A miss was observed in 2 of 93 rounds on the
/// CH32V006 and about 1 of 50 on the CH32X035 (the rate each run prints,
/// RaceTally). Taken as one in ten, for the odds to hold even if a board has a
/// worse spell, five misses in a row are 1e-5 a round, so three rounds a run
/// over forty runs falsely fail with odds of about one in a thousand.
constexpr uint8_t race_attempts = 5;

/// The rounds of this run of the letter and how many of them missed.
struct RaceTally {
    uint8_t rounds = 0;
    uint8_t missed = 0;
};

/// THE ROUND MISSED: both tenures answered with a NACK on their address, no
/// loss on either side, and the wire shows two tenures. Both NACKs on one
/// tenure is a real race judged wrong - neither controller reading the bit
/// it lost - and is not a miss: the verdict below fails on it.
bool race_missed(const RaceRound& round, bool reported, const twilink::Report& peer) {
    return round.status == i2c_nack_addr && reported && peer.mstatus == i2c_nack_addr &&
           (peer.flags & twilink::report_arblost) == 0u && (round.done_us > race_behind_us || round.tail);
}

/// Own STOP first: the lines high together, which it takes about a bit
/// period to reach. Then any low SCL within race_tail_us is a further tenure.
bool race_tail_seen() {
    if (!brio::wait_until<Ruler>([] { return SclPin::read() && SdaPin::read(); }, 50)) {
        return true;   // a line low past any STOP of this board's
    }
    return brio::wait_until<Ruler>([] { return !SclPin::read(); }, race_tail_us);
}

/// ONE TENURE STARTED INTO A BUSY BUS: a START condition by hand, the
/// peer given time to see BUSY, this host's START set, then the STOP that
/// frees the bus - both STARTs leave on it and the wired-AND decides.
template <typename Hst = Host>
RaceRound arb_tenure(uint8_t addr, const uint8_t* tx, uint8_t len, uint32_t lead_us,
                   I2cEngine engine = I2cEngine::interrupt) {
    typename Hst::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = len;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    r.rx_len = 0;
    r.speed = link_speed;
    r.engine = engine;
    host_done = false;
    clear_counts();
    inject_start();
    brio::spin_us<Ruler>(lead_us);
    const bool sync = Hst::start(r);
    inject_stop();
    const uint32_t c0 = Ticker::cycles();
    if (sync) {
        return {Hst::status(), 0u, false};
    }
    // 200 ms, far past either tenure (under 0.2 ms each); the answer read at
    // most one turn of the wait late (a flag read and a ruler read, under
    // 2 us), inside race_behind_us's margins of 15 us each side.
    const bool answered = brio::wait_until<Ruler>(host_answered, 200'000);
    const uint32_t done_us = (Ticker::cycles() - c0) / (SysClock::hz / 1'000'000u);
    if (!answered) {
        print(serial, "    the race never answered: STAR1=", hex(H::status1()), " STAR2=", hex(H::status2()),
              " CTLR1=", hex(H::regs().CTLR1), crlf);
        (void)Hst::recover();
        return {no_answer, done_us, false};
    }
    const bool tail = race_tail_seen();
    return {Hst::status(), done_us, tail};
}

void tj_arbitration() {
    if (!need_peer()) {
        return;
    }
    struct Round {
        uint8_t dut_target;
        uint8_t peer_target;
        bool dut_should_lose;
        const char* name;
    };
    const Round rounds[] = {
        {twilink::command_addr, twilink::dut_addr, true,
         "this board -> 0x6B against the peer -> 0x2C (this board must lose)"},
        {twilink::low_addr, twilink::dut_addr, false,
         "this board -> 0x11 against the peer -> 0x2C (the peer must lose)"},
    };
    uint8_t as_expected = 0;
    RaceTally tally;
    const auto arb_action = [](uint8_t target) {
        twilink::Params a{};
        a.ms = 600;
        a.addr = twilink::dut_addr;
        a.target = target;
        a.count = 4;
        a.seed = 0x40;
        a.aux16 = 120;
        return a;
    };
    for (uint8_t i = 0; i < 2u; ++i) {
        RaceRound round{};
        twilink::Report r{};
        bool rep = false;
        for (uint8_t attempt = 0; attempt < race_attempts; ++attempt) {
            if (!peer_act(Op::arb, arb_action(rounds[i].peer_target))) {
                bench.verdict("the peer accepted the arbitration command", false);
                return;
            }
            host_ready();
            for (uint8_t k = 0; k < 5; ++k) {
                tx_buf[k] = static_cast<uint8_t>(0xD0u + k);
            }
            round = arb_tenure(rounds[i].dut_target, tx_buf, 5, 350);
            settle_ms(700);
            rep = peer_report(r);
            const bool missed = race_missed(round, rep, r);
            ++tally.rounds;
            print(serial, "  ", rounds[i].name, crlf, "    this board: status=", round.status, " after ",
                  round.done_us, " us", round.tail ? ", a tenure after it" : "", "; peer: flags=", hex(r.flags),
                  " mstatus=", r.mstatus, missed ? " - NO RACE, two tenures, again" : "", crlf);
            if (!missed) {
                break;
            }
            ++tally.missed;
        }
        const bool dut_lost = round.status == i2c_arb_lost;
        const bool peer_lost = rep && (r.flags & twilink::report_arblost) != 0u;
        if (rounds[i].dut_should_lose ? (dut_lost && !peer_lost) : (!dut_lost && peer_lost)) {
            ++as_expected;
        }
        if (rounds[i].dut_should_lose) {
            bench.verdict("aiming at the HIGHER address, this controller loses the wired-AND and reports "
                          "i2c_arb_lost, the peer's controller keeping the bus",
                          dut_lost && rep && !peer_lost && (r.flags & twilink::report_host_ran) != 0u);
        } else {
            bench.verdict("aiming at the LOWER address, this controller keeps the bus (nobody answers 0x11: "
                          "nack_addr) and it is the PEER that reports the loss",
                          !dut_lost && round.status == i2c_nack_addr && peer_lost);
        }
    }
    bench.verdict("arbitration was observed in BOTH directions", as_expected == 2u);

    // The losing round again with the request naming the engines: the loss
    // falls in the address, before any block starts, and is the same
    // i2c_arb_lost - no transmit block started or thrown away.
    {
        RaceRound dround{};
        twilink::Report dr{};
        bool drep = false;
        bool quiet = false;
        for (uint8_t attempt = 0; attempt < race_attempts; ++attempt) {
            if (!peer_act(Op::arb, arb_action(twilink::dut_addr))) {
                bench.verdict("the peer accepted the arbitration command", false);
                return;
            }
            owner = Owner::dma_host;
            (void)DmaHost::init(clock);
            const uint32_t faults0 = DmaTxEngine<6>::faults();
            dma6_entries = 0;
            dround = arb_tenure<DmaHost>(twilink::command_addr, tx_buf, 5, 350, I2cEngine::dma);
            quiet = dma6_entries == 0u && DmaTxEngine<6>::faults() == faults0 && !DmaChannel<6>::enabled();
            DmaHost::release();
            host_ready();
            settle_ms(700);
            drep = peer_report(dr);
            const bool missed = race_missed(dround, drep, dr);
            ++tally.rounds;
            print(serial, "  the same loss under I2cEngine::dma: status=", dround.status, " after ", dround.done_us,
                  " us", dround.tail ? ", a tenure after it" : "", ", channel 6 ", quiet ? "untouched" : "TOUCHED",
                  "; peer: flags=", hex(dr.flags), missed ? " - NO RACE, two tenures, again" : "", crlf);
            if (!missed) {
                break;
            }
            ++tally.missed;
        }
        bench.verdict("under dma the same loss is i2c_arb_lost, the peer's controller keeping the bus, and no "
                      "transmit block was started or thrown away",
                      dround.status == i2c_arb_lost && quiet && drep &&
                          (dr.flags & twilink::report_arblost) == 0u && (dr.flags & twilink::report_host_ran) != 0u);
    }
    print(serial, "  the race: ", tally.missed, " of ", tally.rounds, " rounds missed it (two tenures, nothing "
          "arbitrated, not judged), at most ", race_attempts - 1u, " a judged round", crlf);

    // ---- THIS BOARD AS THE CLIENT of the peer's own controller ----
    link_ready();
    twilink::Params c{};
    c.ms = 800;
    c.addr = twilink::dut_addr;
    c.target = twilink::dut_addr;
    c.count = 6;
    c.seed = 0xB0;
    c.aux16 = 400;
    if (!peer_act(Op::arb, c)) {
        bench.verdict("the peer accepted the host command that addresses this board", false);
        return;
    }
    Host::release();
    const bool up = Client::init(clock, {.own = twilink::dut_addr},
                                 {.no_stretch = false, .interrupts = false});
    hold_bus_busy(400);

    uint8_t got[16];
    uint8_t n_got = 0;
    uint8_t hits = 0;
    uint8_t stops = 0;
    bool wrote_to_us = false;
    uint16_t late_s1 = 0;
    bool late_seen = false;
    // The client polled within the peer's 800 ms action and 100 ms more; what
    // it took is judged below, so the wait's own answer is not.
    (void)brio::wait_until<Ruler>([&] {
        const I2cClientEvent e = Client::service();
        if (e == I2cClientEvent::addressed) {
            ++hits;
            wrote_to_us = !Client::host_reads();
        } else if (e == I2cClientEvent::byte_received) {
            if (!late_seen) {
                // THE STRETCH, ONE BYTE LATE: the first byte left standing in
                // DATAR for two milliseconds - what has the shifter done?
                brio::spin_us<Ruler>(2000);
                late_s1 = H::status1();
                late_seen = true;
            }
            const uint8_t v = Client::take();
            if (n_got < sizeof got) {
                got[n_got] = v;
            }
            ++n_got;
        } else if (e == I2cClientEvent::stop) {
            ++stops;
            if (n_got >= c.count) {
                return true;
            }
        } else if (Client::host_nacked()) {
            Client::clear_nack();
        }
        return false;
    }, 900'000);
    uint8_t mism = 0;
    for (uint8_t i = 0; i < n_got && i < c.count; ++i) {
        if (got[i] != twilink::pattern_value(twilink::pattern_counting, c.seed, i)) {
            ++mism;
        }
    }
    print(serial, "  as a CLIENT: init=", up ? 1 : 0, " address matches=", hits, " bytes=", n_got,
          " mism=", mism, " stops=", stops, "; STAR1 two ms after the first byte was left in DATAR: ",
          hex(late_s1), (late_s1 & (i2c_rxne | i2c_btf)) == (i2c_rxne | i2c_btf)
                            ? " (RxNE AND BTF: the next byte is in the shifter, SCL held)"
                            : "",
          crlf);
    bench.verdict("the client answered its own address to a REAL controller on another board, in a "
                  "write tenure",
                  up && hits >= 1u && wrote_to_us);
    bench.verdict("... and took every byte the peer's controller wrote, byte-exact, ending on its STOP",
                  n_got >= c.count && mism == 0u && stops >= 1u);
    bench.verdict("THE STRETCH ONE BYTE LATE: a byte left in DATAR lets the NEXT one into the shifter, "
                  "and only then is SCL held (RxNE and BTF both up)",
                  late_seen && (late_s1 & (i2c_rxne | i2c_btf)) == (i2c_rxne | i2c_btf));
    Client::release();
    settle_ms(300);

    // ---- THIS BOARD AS THE CLIENT, READ FROM: the transmitter ----
    // Two reads of six bytes back to back (the extended repertoire's
    // host_read); each read tenure serves its own run (0xC0.., then
    // 0xE0..), so a byte left in DATAR past the first read's closing NACK
    // would open the second read in place of 0xE0.
    host_ready();
    twilink::Params rd{};
    rd.ms = 900;
    rd.addr = twilink::dut_addr;
    rd.target = twilink::dut_addr;
    rd.count = 6;
    rd.aux16 = 400;
    rd.aux8 = 2;
    if (!peer_act(Op::host_read, rd)) {
        bench.verdict("the peer accepted the host_read command (the extended repertoire)", false);
        host_ready();
        return;
    }
    Host::release();
    const bool up2 = Client::init(clock, {.own = twilink::dut_addr}, {.no_stretch = false, .interrupts = false});
    hold_bus_busy(400);
    uint8_t read_tenures = 0;
    uint8_t gave = 0;
    uint8_t nacks = 0;
    uint8_t drops = 0;
    bool serving = false;
    uint8_t seed = 0;
    uint8_t i_tx = 0;
    uint16_t given_sum = 0;
    // ONE STAR1 SNAPSHOT A PASS, every decision taken on it: the closing
    // NACK is a LEVEL that may rise between two separate reads of the
    // register, and a pass that cleared the error flags on a second read
    // would swallow it uncounted. Polled within the peer's 900 ms action and
    // 100 ms more; what was served is judged below, so the wait's own answer
    // is not.
    (void)brio::wait_until<Ruler>([&] {
        const uint16_t s1 = H::status1();
        if ((s1 & i2c_addr) != 0u) {
            serving = Client::answer_address();
            if (serving) {
                seed = static_cast<uint8_t>(0xC0u + 0x20u * read_tenures);
                ++read_tenures;
                i_tx = 0;
            }
        } else if ((s1 & i2c_af) != 0u) {
            // THE CLOSING NACK: the byte the shifter asked for beyond it is
            // thrown away here, before the next tenure can see it.
            H::clear_errors(static_cast<uint16_t>(s1 & i2c_errors));
            serving = false;
            ++nacks;
            if (Client::flush()) {
                ++drops;
                given_sum = static_cast<uint16_t>(given_sum - static_cast<uint8_t>(seed + i_tx - 1u));
            }
        } else if (serving && (s1 & i2c_txe) != 0u) {
            const uint8_t v = static_cast<uint8_t>(seed + i_tx);
            Client::give(v);
            given_sum = static_cast<uint16_t>(given_sum + v);
            ++i_tx;
            ++gave;
        } else if ((s1 & i2c_stopf) != 0u) {
            Client::clear_stop();
            if (nacks >= 2u) {
                return true;
            }
        } else if ((s1 & i2c_errors) != 0u) {
            H::clear_errors(static_cast<uint16_t>(s1 & i2c_errors));
        }
        return false;
    }, 1'000'000);
    Client::release();
    settle_ms(300);
    host_ready();
    twilink::Report rr{};
    const bool rrep = peer_report(rr);
    uint16_t want_sum = 0;
    for (uint8_t k = 0; k < 2u; ++k) {
        for (uint8_t i = 0; i < 6u; ++i) {
            want_sum = static_cast<uint16_t>(want_sum + static_cast<uint8_t>(0xC0u + 0x20u * k + i));
        }
    }
    print(serial, "  as a CLIENT, read from: init=", up2 ? 1 : 0, " read tenures=", read_tenures,
          " bytes given=", gave, " closing NACKs=", nacks, " dropped by flush()=", drops, "; the peer read ",
          rr.count, " bytes in ", rr.addr_hits, " reads, status ", rr.mstatus, ", sum ", hex(rr.sum), " (",
          hex(want_sum), "), first ", hex(rr.first), " (0xC0), the second read's first ", hex(rr.aux0),
          " (0xE0); this side's sum of what left, the drops given back, ", hex(given_sum), crlf);
    bench.verdict("the client served a REAL controller's two reads as a transmitter, every byte the peer "
                  "took its own run (count, sum and both first bytes), and what this side counts as sent - "
                  "the drops given back - is what the peer read",
                  up2 && rrep && rr.mstatus == i2c_ok && rr.count == 12u && rr.addr_hits == 2u &&
                      rr.sum == want_sum && rr.first == 0xC0u && rr.aux0 == 0xE0u && given_sum == want_sum);
    bench.verdict("THE TRANSMITTER ASKED ONE BYTE MORE: at each closing NACK one byte stood in DATAR, and "
                  "flush() dropped it - the second read opened on its own first byte",
                  read_tenures == 2u && nacks == 2u && drops == 2u && rr.aux0 == 0xE0u);
    bench.verdict("the command channel is back after the role swaps", command(Op::ping));
    // The collision case (twi_link's `coll`, two clients on ONE address
    // with a third party reading) has no third party on this desk: the
    // document's gap list says so.
}

// ===========================================================================
// v - this board as the client, served from its vectors
// ===========================================================================
//
// Letter j's two client cases again with `I2cClientOptions::interrupts` up:
// every flag reaches the program through i2c1_ev and i2c1_er, and the
// bodies `service()` and `error_service()` decide - the glue below owes
// take() on byte_received, give() on byte_wanted and flush() on the closing
// NACK. ONE ENTRY PER EVENT is the shape the bodies promise: each entry
// consumes the one flag it reports (ADDR and STOPF in the body, RxNE by
// take(), TxE by give(), AF in error_service()), so an entry that finds
// nothing (`none`) is the defect a level left standing would make. The
// event body is timed on the STK at every entry (the entry's own
// prologue outside the stamps).

/// The vectors' tally for the client case (letter v), zeroed per case.
struct ClientTally {
    uint32_t entries;
    uint32_t err_entries;
    uint32_t cycles;
    uint32_t cycles_max;
    uint16_t given_sum;
    uint8_t addressed;
    uint8_t reads;
    uint8_t rx;
    uint8_t tx;
    uint8_t stops;
    uint8_t nacks;
    uint8_t drops;
    uint8_t nones;
    uint8_t errors;
    uint8_t seed;
    uint8_t i_tx;
    uint8_t got[16];
};
volatile bool client_live = false;
ClientTally ct{};
/// A client storm is silenced past this many entries in one case (a
/// tenure of six bytes takes about ten).
constexpr uint32_t client_entry_budget = 10'000;

/// The event vector's client glue: one event, one action.
[[gnu::always_inline]] inline void client_event() {
    switch (Client::service()) {
        case I2cClientEvent::addressed:
            ++ct.addressed;
            if (Client::host_reads()) {
                ct.seed = static_cast<uint8_t>(0xC0u + 0x20u * ct.reads);
                ++ct.reads;
                ct.i_tx = 0;
            }
            break;
        case I2cClientEvent::byte_received: {
            const uint8_t v = Client::take();
            if (ct.rx < sizeof ct.got) {
                ct.got[ct.rx] = v;
            }
            ++ct.rx;
            break;
        }
        case I2cClientEvent::byte_wanted: {
            const uint8_t v = static_cast<uint8_t>(ct.seed + ct.i_tx);
            Client::give(v);
            ct.given_sum = static_cast<uint16_t>(ct.given_sum + v);
            ++ct.i_tx;
            ++ct.tx;
            break;
        }
        case I2cClientEvent::stop: ++ct.stops; break;
        default: ++ct.nones; break;
    }
}

/// The error vector's client glue: the closing NACK flushes the byte the
/// shifter asked for beyond it and gives it back to the tally.
[[gnu::always_inline]] inline void client_error() {
    const I2cClientEvent e = Client::error_service();
    if (e == I2cClientEvent::nacked) {
        ++ct.nacks;
        if (Client::flush()) {
            ++ct.drops;
            ct.given_sum = static_cast<uint16_t>(ct.given_sum - static_cast<uint8_t>(ct.seed + ct.i_tx - 1u));
        }
    } else if (e == I2cClientEvent::error) {
        ++ct.errors;
    } else {
        ++ct.nones;
    }
}

/// One client case: the peer's host action armed, this board's client up
/// ON ITS VECTORS, the window opened by hand, and the vectors left to it
/// until `done` says the case is over or `ms` runs out.
template <typename Done>
bool client_case(Op op, const twilink::Params& a, uint32_t ms, Done done) {
    if (!peer_act(op, a)) {
        return false;
    }
    Host::release();
    ct = ClientTally{};
    client_live = true;
    const bool up = Client::init(clock, {.own = twilink::dut_addr}, {.no_stretch = false, .interrupts = true});
    hold_bus_busy(400);
    // The vectors write the tally; done() reads it, and the caller judges it,
    // so the wait's own answer is not.
    (void)brio::wait_until<Ruler>(
        [&] {
            asm volatile("" ::: "memory");
            return done();
        },
        ms * 1000u);
    Client::release();
    client_live = false;
    settle_ms(300);
    host_ready();
    return up;
}

void tv_client_vectors() {
    if (!need_peer()) {
        return;
    }
    // ---- WRITTEN TO: six bytes from the peer's controller ----
    link_ready();
    twilink::Params c{};
    c.ms = 800;
    c.addr = twilink::dut_addr;
    c.target = twilink::dut_addr;
    c.count = 6;
    c.seed = 0xB0;
    c.aux16 = 400;
    const bool wup = client_case(Op::arb, c, 900, [&] { return ct.rx >= c.count && ct.stops >= 1u; });
    const ClientTally w = ct;
    uint8_t mism = 0;
    for (uint8_t i = 0; i < w.rx && i < c.count; ++i) {
        if (w.got[i] != twilink::pattern_value(twilink::pattern_counting, c.seed, i)) {
            ++mism;
        }
    }
    print(serial, "  written to, from the vectors: init=", wup ? 1 : 0, " addressed=", w.addressed, " bytes=", w.rx,
          " mism=", mism, " stops=", w.stops, " nones=", w.nones, " errors=", w.errors, "; event entries ",
          w.entries, ", the body ", w.entries != 0u ? w.cycles / w.entries : 0u, " cycles on average, ",
          w.cycles_max, " at most", crlf);
    bench.verdict("FROM ITS VECTORS the client answered a real controller's write: its address, every byte "
                  "byte-exact, the STOP seen",
                  wup && w.addressed >= 1u && w.rx == c.count && mism == 0u && w.stops >= 1u);
    bench.verdict("one entry per event: no entry found nothing, no error", w.nones == 0u && w.errors == 0u);

    // ---- READ FROM: two reads of six, each read its own run ----
    twilink::Params rd{};
    rd.ms = 900;
    rd.addr = twilink::dut_addr;
    rd.target = twilink::dut_addr;
    rd.count = 6;
    rd.aux16 = 400;
    rd.aux8 = 2;
    const bool rup = client_case(Op::host_read, rd, 1000, [&] { return ct.nacks >= 2u; });
    const ClientTally r = ct;
    twilink::Report rr{};
    const bool rrep = peer_report(rr);
    uint16_t want_sum = 0;
    for (uint8_t k = 0; k < 2u; ++k) {
        for (uint8_t i = 0; i < 6u; ++i) {
            want_sum = static_cast<uint16_t>(want_sum + static_cast<uint8_t>(0xC0u + 0x20u * k + i));
        }
    }
    print(serial, "  read from, from the vectors: init=", rup ? 1 : 0, " reads=", r.reads, " given=", r.tx,
          " closing NACKs=", r.nacks, " dropped=", r.drops, " nones=", r.nones, " errors=", r.errors,
          "; the peer read ", rr.count, " bytes in ", rr.addr_hits, " reads, status ", rr.mstatus, ", sum ",
          hex(rr.sum), " (", hex(want_sum), "), first ", hex(rr.first), ", the second read's first ",
          hex(rr.aux0), "; event entries ", r.entries, " (the body ",
          r.entries != 0u ? r.cycles / r.entries : 0u, " cycles on average, ", r.cycles_max,
          " at most), error entries ", r.err_entries, crlf);
    bench.verdict("FROM ITS VECTORS the client served a real controller's two reads as a transmitter, every "
                  "byte the peer took its own run, and what the tally counts as sent - the drops given back - "
                  "is what the peer read",
                  rup && rrep && rr.mstatus == i2c_ok && rr.count == 12u && rr.addr_hits == 2u &&
                      rr.sum == want_sum && rr.first == 0xC0u && rr.aux0 == 0xE0u && r.given_sum == want_sum);
    bench.verdict("the closing NACK on the error vector each time, and flush() there dropped the byte asked "
                  "beyond it - the second read opened on its own first byte",
                  r.reads == 2u && r.nacks == 2u && r.drops == 2u && rr.aux0 == 0xE0u);
    bench.verdict("one entry per event: no entry found nothing, no error", r.nones == 0u && r.errors == 0u);
    bench.verdict("the command channel is back after the role swaps", command(Op::ping));
}

// ===========================================================================
// k - the stuck bus
// ===========================================================================

void tk_unstick() {
    if (!need_peer()) {
        return;
    }
    const uint8_t clean = Host::unstick();
    print(serial, "  a healthy bus: unstick() returns ", clean, crlf);
    bench.verdict("unstick() on a clean bus clocks nothing and says so", clean == 0u);
    twilink::Params h{};
    h.ms = 400;
    h.aux8 = 4;
    if (!peer_act(Op::hold_sda, h)) {
        bench.verdict("the peer accepted the hold_sda command", false);
        return;
    }
    settle_ms(5);
    const bool held = !SdaPin::read();
    clear_counts();
    const uint32_t storms_before = storms;
    const uint8_t pulses = Host::unstick();
    print(serial, "  SDA held by the peer: ", held ? "yes" : "no", "; unstick() clocked ", pulses,
          " pulse(s) before it let go", crlf);
    bench.verdict("the peer really held SDA low", held);
    // THE COUNT is the peer's: twi_peer polls SCL for falling edges and lets
    // SDA go on the fourth (h.aux8), its turn a few instructions against the
    // 5 us low half of each of unstick()'s pulses; unstick() reads SDA a half
    // bit after letting SCL rise, by when the line, 1.5 kOhm over the bus's
    // picofarads, is high. So it reads released after the fourth pulse.
    bench.verdict("unstick() clocked until the client released - the peer's four falling edges asked, four "
                  "pulses",
                  pulses == h.aux8);
    settle_ms(450);
    print(serial, "  the event vector, idle, after the hand-made START and STOP: ", host_isr_entries,
          " entries, STAR1=", hex(H::status1()), crlf);
    bench.verdict("the STOPF the hand-made STOP may leave is cleared - no storm on the event vector",
                  host_isr_entries <= 2u && storms == storms_before && (H::status1() & i2c_stopf) == 0u);
    bench.verdict("with the wire free again unstick() clocks nothing", Host::unstick() == 0u);
    bench.verdict("and the command channel is back", command(Op::ping));
}

// ===========================================================================
// u - a BUSY left standing over an idle wire
// ===========================================================================
//
// 15.9.1's own case for SWRST: "when no stop condition is detected on the
// bus but the busy bit is 1". STAGED BY HAND on the two pads: a START (SDA
// low under a high SCL, every node's BUSY up), SCL taken low, SDA let go
// while SCL is low - no STOP, the clock is low - and SCL let go: both lines
// high and no STOP condition ever on the wire, so BUSY stands over an idle
// bus. (The STOP made with the block's gate closed does not do it: BUSY
// read 0 on the gate's reopening - the flag follows the wire.) The
// engine's start() waits 100 us for BUSY, no read finding a line low, takes
// SWRST and counts it (wait_bus_free()); the tenure behind it completes.

void tu_unwedge() {
    if (!need_peer()) {
        return;
    }
    bool staged = true;
    bool taken_out = true;
    for (const I2cSpeed speed : {link_speed, I2cSpeed::fast_400k}) {
        host_ready();
        const uint16_t before = Host::unwedges();
        inject_start();
        (void)delay_us(clock, 20);
        const bool busy_in = H::busy();
        SclPin::output(false);
        (void)delay_us(clock, 20);
        inject_stop();   // SDA let go under a LOW clock: no STOP condition
        (void)delay_us(clock, 20);
        SclPin::function();
        (void)delay_us(clock, 50);
        const bool lines_high = SclPin::read() && SdaPin::read();
        const bool busy_left = H::busy();
        const uint8_t st = host_tenure(twilink::command_addr, nullptr, 0, nullptr, 0, speed);
        const uint16_t taken = static_cast<uint16_t>(Host::unwedges() - before);
        settle_ms(2);
        const bool busy_after = H::busy();
        print(serial, "  a START by hand: BUSY ", busy_in ? 1 : 0, "; SDA let go under a low clock, then SCL: both lines ",
              lines_high ? "high" : "NOT high", ", BUSY ", busy_left ? 1 : 0, "; the probe of the peer at ", i2c_speed_hz(speed) / 1000u,
              " kHz: status ", st, ", unwedges +", taken, ", BUSY 2 ms after ", busy_after ? 1 : 0, crlf);
        staged = staged && busy_in && lines_high && busy_left;
        taken_out = taken_out && taken == 1u && st == i2c_ok && !busy_after;
    }
    host_ready();
    bench.verdict("the STAGING: a START seen (BUSY up), no STOP ever on the wire - both lines high and BUSY "
                  "standing over the idle bus",
                  staged);
    bench.verdict("start() TOOK IT OUT, at both speeds: one unwedge counted, and the probe of the peer behind it completed "
                  "i2c_ok, BUSY down after",
                  taken_out);
    bench.verdict("and the command channel is back", command(Op::ping));
}

// ===========================================================================
// t - the two bounds of start() against their names
// ===========================================================================
//
// THE INSTRUMENT. Ticker::cycles() around start(), a count being one cycle of HCLK;
// the read's own cost is inside every figure alike (a read of it is some 76 cycles), so a
// figure reads high by it, never low. The rows with the STK stopped time on TIM2 at 2 us a
// count, the instrument's resolution there; they run first, because they measure the
// poll the other rows' ceilings use.
//
// What brackets a figure. Below: the name, bus_free_us or stop_drain_us in cycles,
// since the wait ends no earlier. Above (over_of()): the same plus
//  - what start() costs around the wait on a free bus, its two first looks, which the
//    free bus's row measures (`free`, the ruler's read inside it as in every row);
//  - three polls: the wait reads the counter once a poll, so it ends up to one poll
//    after the bound, and the wait's entry (poll_for_us()'s call, the rate's multiply,
//    the counter's two loads) and its last look after the loop hold fewer instructions
//    and peripheral loads than two polls (the release listing, the watch's: some 30
//    instructions and five peripheral loads with wait_bus_free()'s own frame, against
//    two polls' 44 and eight; the drain's wait is shaped alike). THE POLL is measured
//    in this run with the STK stopped, on TIM2, an independent counter: the span over
//    the budget's count of polls, which holds the wait's entry and last look too, so it
//    reads long;
//  - one tick's step (tick_step()), measured in this run: a tick landing in the last
//    poll lengthens it by its handler.
constexpr uint32_t bound_per_us = SysClock::hz / 1'000'000u;
constexpr uint32_t bus_free_named = 100;
constexpr uint32_t stop_drain_named = 5'000;
/// With the STK stopped: the budget is the bound over 8 cycles a poll.
constexpr uint32_t budget_of(uint32_t us) { return us * bound_per_us / 8u; }

/// THE TICK'S COST TO A POLLED LOOP, measured in this image: the ruler read
/// back to back for three tick periods behind a drained console; the longest
/// step between two reads is one read plus the tick's handler with its entry
/// and exit, nothing else being armed. 0 when the console did not drain.
uint32_t tick_step() {
    if (!console_drain()) {
        return 0;
    }
    constexpr uint32_t period = SysClock::hz / Ticker::ticks_per_second;
    uint32_t last = Ticker::cycles();
    const uint32_t t0 = last;
    uint32_t worst = 0;
    while (last - t0 < 3u * period) {
        const uint32_t now = Ticker::cycles();
        worst = now - last > worst ? now - last : worst;
        last = now;
    }
    return worst;
}

/// The ceiling's terms over the name, in cycles (the block comment above).
constexpr uint32_t over_of(uint32_t free_hi, uint32_t poll, uint32_t step) { return free_hi + 3u * poll + step; }

Host::Request nobody_probe() {
    Host::Request r{};
    r.addr = nobody_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    r.speed = link_speed;
    return r;
}

/// What the last start_cycles() returned: true when start() answered the request itself.
bool start_answered = false;
/// Every timed call of the letter so far behind a console that went idle:
/// one that did not drain would put its vector inside the timed call, so that
/// call is not made, its figure reads 0 (under every floor) and this goes
/// false, failing every verdict on a timed figure.
bool start_drained = true;

uint32_t start_cycles(const Host::Request& r) {
    host_done = false;
    host_isr_entries = 0;
    if (!console_drain()) {
        start_drained = false;
        start_answered = false;
        return 0;
    }
    const uint32_t c0 = Ticker::cycles();
    start_answered = Host::start(r);
    return Ticker::cycles() - c0;
}

/// A write of two bytes the client holds, its reply waited for: behind it the
/// client still holds SCL with the host's STOP standing. The hold is 45 ms, over
/// either bound with the counter running or stopped.
bool stand_stop(I2cSpeed speed) {
    host_ready();
    settle_ms(5);
    twilink::Params held{};
    held.count = 16;
    held.ms = 200;
    held.addr = twilink::dut_addr;
    held.hold_us = 45'000;
    if (!peer_act(Op::serve, held)) {
        return false;
    }
    host_ready();
    return host_tenure(twilink::dut_addr, tx_buf, 2, nullptr, 0, speed) == i2c_ok && H::stopping();
}

/// A probe of the peer's command channel: an address phase and nothing else, the
/// peer's serve keeping its client elsewhere until its deadline.
Host::Request command_probe(I2cSpeed speed) {
    Host::Request r = kl::request(twilink::command_addr, kl::out_a);
    r.tx_len = 0;
    r.speed = speed;
    return r;
}

/// One request through an arbiter: its one reply and the time to it, post to reply.
struct ArbAnswer {
    uint8_t replies;
    uint8_t status;
    uint32_t us;
};

ArbAnswer ask_arbiter(const Host::Request& r) {
    kl::Probe::clear_tally();
    const uint32_t c0 = Ticker::cycles();
    post<kl::I2cArb>(r);
    kl::pump_until(1, 100);
    const uint32_t us = (Ticker::cycles() - c0) / bound_per_us;
    return {kl::Probe::n, kl::Probe::replies[0], us};
}

ArbAnswer ask_untimed(const Host::Request& r, uint32_t ms) {
    kl::Probe::clear_tally();
    const uint32_t c0 = Ticker::cycles();
    post<kl::I2cArbUntimed>(r);
    kl::pump_until(1, ms);
    const uint32_t us = (Ticker::cycles() - c0) / bound_per_us;
    return {kl::Probe::n, kl::Probe::replies[0], us};
}

/// A request through whichever arbiter the round names.
ArbAnswer ask_of(bool timed, const Host::Request& r, uint32_t ms) {
    return timed ? ask_arbiter(r) : ask_untimed(r, ms);
}

/// THE RUN-OUT THROUGH THE ARBITER, one row: a write of two the client holds, its
/// reply taken, and a request posted right behind it, under that hold's second half
/// with the host's STOP standing. It is answered by start() itself: i2c_timeout, one
/// reply, CTLR1's STOP still up and no START, BUSY up, no reset and no vector. The
/// hold over, the STOP is out and BUSY down with nothing done, and the next request
/// is on the wire. Under the 20 ms per-bus timeout the client holds 15 ms, or the
/// write itself would meet the timeout; with none it holds 45.
///
/// THE BOUND, post to reply: stop_drain_us, then `over` (over_of(): what start()
/// costs around the wait, three polls, a tick's step), then the kernel's turn, which
/// is read here on a request refused with nothing on the wire: the same post,
/// dispatch and reply with no wait in them. One microsecond for the truncation of
/// each. The next request's time to its reply, the hold over, is printed and not
/// judged: it holds the probe's vector entries, which this letter does not time; its
/// one reply and its status are judged.
bool run_out_row(I2cSpeed speed, bool timed, uint32_t over) {
    const uint16_t hold_us = timed ? 15'000 : 45'000;
    constexpr uint8_t stands = 3;
    uint32_t lo = 0xFFFFFFFFu;
    uint32_t hi = 0;
    uint32_t turn_hi = 0;
    uint32_t next_hi = 0;
    uint8_t clean = 0;
    for (uint8_t k = 0; k < stands; ++k) {
        host_ready();
        settle_ms(5);
        twilink::Params held{};
        held.count = 16;
        held.ms = 200;
        held.addr = twilink::dut_addr;
        held.hold_us = hold_us;
        const bool accepted = peer_act(Op::serve, held);
        kl::BusKernel::init_all();
        owner = Owner::bus_ao;
        bus_ao_untimed = !timed;
        Host::Request refused = command_probe(speed);
        refused.addr = 0x00;
        const ArbAnswer turn = ask_of(timed, refused, 10);
        Host::Request w = kl::request(twilink::dut_addr, kl::out_a);
        w.tx_len = 2;
        w.speed = speed;
        const ArbAnswer through = ask_of(timed, w, 150);
        const bool stood = H::stopping();
        const uint16_t resets0 = Host::unwedges();
        host_isr_entries = 0;
        // A console that did not drain would put its vector inside the timed
        // answer: such a row is not sound.
        const bool drained = console_drain();
        const ArbAnswer a = ask_of(timed, command_probe(speed), 30);
        const uint16_t c1 = H::regs().CTLR1;
        const bool untouched = (c1 & i2c_stop) != 0u && (c1 & i2c_start) == 0u && H::busy() && host_isr_entries == 0u;
        kl::drain(hold_us / 1000u + 2u);
        const bool freed = !H::stopping() && !H::busy();
        const ArbAnswer next = ask_of(timed, command_probe(speed), 30);
        kl::drain(2);
        bus_ao_untimed = false;
        owner = Owner::host;
        const uint32_t ceiling = stop_drain_named + over / bound_per_us + 1u + turn.us + 1u;
        lo = a.us < lo ? a.us : lo;
        hi = a.us > hi ? a.us : hi;
        turn_hi = turn.us > turn_hi ? turn.us : turn_hi;
        next_hi = next.us > next_hi ? next.us : next_hi;
        if (drained && accepted && turn.replies == 1u && turn.status == i2c_rejected && through.replies == 1u &&
            through.status == i2c_ok &&
            stood && a.replies == 1u && a.status == i2c_timeout && a.us >= stop_drain_named && a.us <= ceiling && untouched &&
            freed && next.replies == 1u && (next.status == i2c_ok || next.status == i2c_nack_addr) &&
            Host::unwedges() == resets0) {
            ++clean;
        }
        host_ready();
        settle_ms(220);
    }
    print(serial, "  a run-out through the arbiter, ", timed ? "a 20 ms timeout" : "no timeout", ", ", i2c_speed_hz(speed) / 1000u,
          " kHz: i2c_timeout in ", lo, " to ", hi, " us, the kernel's turn ", turn_hi, " us, the next on the wire in ", next_hi,
          " us at most, sound ", clean, " of ", stands, crlf);
    return clean == stands;
}

/// Requests waiting behind the tenure when its reply comes: each spends the bound in
/// one dispatch of the arbiter (begin_waiting()), and each is answered. A write of two
/// under a 15 ms hold and two probes queued behind it: 15 ms, then 5 and 5. The bound
/// is run_out_row()'s twice over and the write's, printed with what it took: the hold
/// on the peer's clock, the write's three bytes at the slowest rate letter f passes
/// (360 us, tenure_budget_us's arithmetic), the two run-outs with their `over`, and
/// three of the kernel's turns, one a reply, read as run_out_row() reads it.
bool run_out_queue(uint32_t over) {
    constexpr uint32_t hold_us = 15'000;
    host_ready();
    settle_ms(5);
    twilink::Params held{};
    held.count = 16;
    held.ms = 200;
    held.addr = twilink::dut_addr;
    held.hold_us = hold_us;
    const bool accepted = peer_act(Op::serve, held);
    kl::BusKernel::init_all();
    owner = Owner::bus_ao;
    Host::Request refused = command_probe(link_speed);
    refused.addr = 0x00;
    const ArbAnswer turn = ask_arbiter(refused);
    Host::Request w = kl::request(twilink::dut_addr, kl::out_a);
    w.tx_len = 2;
    w.speed = link_speed;
    kl::Probe::clear_tally();
    // A console that did not drain would put its vector inside the timed
    // replies: the row is then not sound.
    const bool drained = console_drain();
    const uint32_t c0 = Ticker::cycles();
    post<kl::I2cArb>(w);
    post<kl::I2cArb>(command_probe(link_speed));
    post<kl::I2cArb>(command_probe(link_speed));
    kl::pump_until(3, 60);
    const uint32_t us = (Ticker::cycles() - c0) / bound_per_us;
    const uint8_t n = kl::Probe::n;
    kl::drain(hold_us / 1000u + 2u);
    const bool freed = !H::stopping() && !H::busy();
    owner = Owner::host;
    print(serial, "  two requests waiting behind a 15 ms hold: replies ", n, ", status ", kl::Probe::replies[0], " ",
          kl::Probe::replies[1], " ", kl::Probe::replies[2], ", all in ", us, " us, the kernel's turn ", turn.us,
          " us", crlf);
    // The hold is the peer's, timed on its clock: peer_lo_us(15'000) = 14445 to
    // peer_hi_us(15'000) = 15555 us here (the two RC clocks' ratio); the two
    // stop_drain_us bounds and the rest are this board's own, on this clock.
    const bool ok = drained && accepted && turn.replies == 1u && turn.status == i2c_rejected && n == 3u &&
                    kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_timeout &&
                    kl::Probe::replies[2] == i2c_timeout && us >= peer_lo_us(hold_us) + 2u * stop_drain_named &&
                    us <= peer_hi_us(hold_us) + 360u + 2u * (stop_drain_named + over / bound_per_us + 1u) +
                              3u * (turn.us + 1u) &&
                    freed;
    host_ready();
    settle_ms(220);
    return ok;
}

/// A BUSY left standing by hand: a START, SDA let go under a low SCL, SCL let go.
bool stand_busy() {
    host_ready();
    (void)delay_us(clock, 50);
    inject_start();
    (void)delay_us(clock, 20);
    SclPin::output(false);
    (void)delay_us(clock, 20);
    inject_stop();
    (void)delay_us(clock, 20);
    SclPin::function();
    (void)delay_us(clock, 50);
    return SclPin::read() && SdaPin::read() && H::busy();
}

void tt_bounds() {
    if (!need_peer()) {
        return;
    }
    start_drained = true;
    // ---- first, with the STK stopped: each wait on its count of polls, and the poll ----
    // The STK stopped stops the suite's Ruler too, so the span is TIM2's, a 16-bit
    // count at 2 us started from zero: its update flag still down at the end says it
    // did not wrap (131 ms), so the count is the whole span. Each wait ends on its
    // budget of polls (budget_of(), at 8 cycles a poll, the least a poll can take), so
    // never under the time named. The span over that count is THE POLL the running
    // rows' ceilings take (over_of()), the wait's entry and its last look spread over
    // it, so it reads long: rounded up, and one cycle more for the count's quantum, 2 us
    // over 600 polls.
    using PollTimer = Tim<2>;
    constexpr uint32_t poll_timer_us = 2;
    PollTimer::init();
    (void)PollTimer::configure({.prescaler = static_cast<uint16_t>(poll_timer_us * bound_per_us - 1u), .period = 0xFFFF});
    PollTimer::enable(true);
    // A console that did not drain would put its vector inside the timed call:
    // the call is then not made, its figure reads 0, under every floor, and
    // start_drained goes false.
    bool wrapped = false;
    const auto stopped_us = [&wrapped]() -> uint32_t {
        if (!console_drain()) {
            start_drained = false;
            return 0;
        }
        stk()->CTLR = stk()->CTLR & ~stk_ste;
        PollTimer::set_count(0);
        PollTimer::clear_flags(PollTimer::update_flag);
        host_done = false;
        host_isr_entries = 0;
        (void)Host::start(nobody_probe());
        const uint16_t t1 = static_cast<uint16_t>(PollTimer::count());
        wrapped = wrapped || PollTimer::flag(PollTimer::update_flag);
        stk()->CTLR = stk()->CTLR | stk_ste;
        return poll_timer_us * t1;
    };
    host_ready();
    SclPin::output(false);
    (void)delay_us(clock, 20);
    const uint32_t free_budget_us = stopped_us();
    SclPin::function();
    (void)delay_us(clock, 20);
    (void)Host::recover();
    const bool wire_idle = stand_busy();
    const uint32_t watch_budget_us = stopped_us();
    settle_ms(1);
    (void)Host::recover();
    const bool stood = stand_stop(link_speed);
    const uint32_t drain_budget_us = stopped_us();
    settle_ms(60);
    (void)Host::recover();
    settle_ms(200);
    PollTimer::enable(false);
    const uint32_t free_polls = budget_of(bus_free_named);
    const uint32_t drain_polls = budget_of(stop_drain_named);
    const auto per_poll = [](uint32_t us, uint32_t polls) { return (us * bound_per_us + polls - 1u) / polls + 1u; };
    const uint32_t poll_free = per_poll(free_budget_us, free_polls);
    const uint32_t poll_watch = per_poll(watch_budget_us, free_polls);
    const uint32_t poll_drain = per_poll(drain_budget_us, drain_polls);
    uint32_t poll = poll_free > poll_watch ? poll_free : poll_watch;
    poll = poll_drain > poll ? poll_drain : poll;
    print(serial, "  the STK stopped: bus_free_us ends after ", free_polls, " polls in ", free_budget_us, " us, ",
          poll_free, " cycles a poll with SCL low, ", watch_budget_us, " us and ", poll_watch,
          " over an idle wire (BUSY up ", wire_idle ? 1 : 0, ", the reset inside); stop_drain_us after ", drain_polls,
          " polls in ", drain_budget_us, " us, ", poll_drain, " cycles a poll (rounded up, one more for the count)",
          wrapped ? "; TIM2 WRAPPED" : "", crlf);
    bench.verdict("with the STK stopped each wait still ends, on its count of polls: never under the time named, "
                  "the span TIM2 counted whole",
                  start_drained && stood && wire_idle && !wrapped && free_budget_us >= bus_free_named &&
                      watch_budget_us >= bus_free_named && drain_budget_us >= stop_drain_named);
    // THE TICK'S STEP, the last of the ceilings' terms.
    const uint32_t step = tick_step();
    if (step == 0u) {
        start_drained = false;
    }
    print(serial, "  a tick's step on a polled loop: ", step, " cycles (0 = the console did not drain)", crlf);
    // ---- the free bus: what start() costs around the wait ----
    constexpr uint8_t tries = 8;
    uint32_t free_lo = 0xFFFFFFFFu;
    uint32_t free_hi = 0;
    host_ready();
    for (uint8_t k = 0; k < tries; ++k) {
        settle_ms(2);
        const uint32_t c = start_cycles(nobody_probe());
        free_lo = c < free_lo ? c : free_lo;
        free_hi = c > free_hi ? c : free_hi;
        settle_ms(2);
    }
    print(serial, "  free bus: start() ", free_lo, " to ", free_hi, " cycles, the ruler's read inside", crlf);
    const uint32_t over = over_of(free_hi, poll, step);
    // ---- bus_free_us, BUSY held by SCL low: the bound alone ----
    uint32_t held_lo = 0xFFFFFFFFu;
    uint32_t held_hi = 0;
    bool held_kept = true;
    for (uint8_t k = 0; k < tries; ++k) {
        host_ready();
        const uint16_t resets0 = Host::unwedges();
        SclPin::output(false);
        (void)delay_us(clock, 20);
        const uint32_t c = start_cycles(nobody_probe());
        held_lo = c < held_lo ? c : held_lo;
        held_hi = c > held_hi ? c : held_hi;
        held_kept = held_kept && Host::unwedges() == resets0;
        SclPin::function();
        (void)delay_us(clock, 20);
        (void)Host::recover();
        settle_ms(1);
    }
    print(serial, "  bus_free_us ", bus_free_named, " with SCL held low: start() ", held_lo, " to ", held_hi, " cycles, ",
          (held_hi - free_lo) / bound_per_us, " us at most over the free bus's", crlf);
    bench.verdict("bus_free_us means what it says: with SCL held low start() spends its 100 us and no more than the "
                  "free bus's cost, three polls and a tick's step over them, no reset taken",
                  start_drained && held_lo >= bus_free_named * bound_per_us &&
                      held_hi <= bus_free_named * bound_per_us + over && held_kept);
    // ---- bus_free_us, BUSY standing over an idle wire: the bound and the reset ----
    uint32_t stand_lo = 0xFFFFFFFFu;
    uint32_t stand_hi = 0;
    bool stand_kept = true;
    for (uint8_t k = 0; k < tries; ++k) {
        const bool staged = stand_busy();
        const uint16_t resets0 = Host::unwedges();
        const uint32_t c = start_cycles(nobody_probe());
        stand_lo = c < stand_lo ? c : stand_lo;
        stand_hi = c > stand_hi ? c : stand_hi;
        stand_kept = stand_kept && staged && Host::unwedges() == resets0 + 1u;
        settle_ms(1);
    }
    print(serial, "  bus_free_us ", bus_free_named, " over an idle wire: start() ", stand_lo, " to ", stand_hi,
          " cycles, the reset inside", crlf);
    /// The reset's stores: SWRST up and down, the timing, PE, ACK and CTLR2, some
    /// ten read-modify-writes through the peripheral bus.
    constexpr uint32_t reset_cycles = 300;
    bench.verdict("over an idle wire the same 100 us, the reset's stores on top, one reset counted each time",
                  start_drained && stand_lo >= bus_free_named * bound_per_us &&
                      stand_hi <= bus_free_named * bound_per_us + over + reset_cycles && stand_kept);
    // ---- stop_drain_us, run out by a client holding the clock ----
    for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
        uint32_t lo = 0xFFFFFFFFu;
        uint32_t hi = 0;
        bool answered = true;
        constexpr uint8_t stands = 3;
        for (uint8_t k = 0; k < stands; ++k) {
            answered = answered && stand_stop(speed);
            const uint32_t c = start_cycles(nobody_probe());
            lo = c < lo ? c : lo;
            hi = c > hi ? c : hi;
            const uint16_t c1 = H::regs().CTLR1;
            answered = answered && start_answered && Host::status() == i2c_timeout && (c1 & i2c_stop) != 0u &&
                       (c1 & i2c_start) == 0u && host_isr_entries == 0u;
            settle_ms(60);   // the hold over, the STOP out
            (void)Host::recover();
            settle_ms(200);  // the peer's deadline
        }
        print(serial, "  stop_drain_us ", stop_drain_named, ", a client holding the clock at ", i2c_speed_hz(speed) / 1000u,
              " kHz: start() ", lo, " to ", hi, " cycles, ", hi / bound_per_us, " us at most, answered ", answered ? "yes" : "NO", crlf);
        bench.verdict("stop_drain_us means what it says: a client holding the clock runs it out at its 5 ms, and start() "
                      "answers: true, i2c_timeout, no START, no vector",
                      start_drained && answered && lo >= stop_drain_named * bound_per_us &&
                          hi <= stop_drain_named * bound_per_us + over);
    }
    // ---- the same through the arbiter: what a requester gets ----
    bool timed_sound = true;
    bool untimed_sound = true;
    for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
        timed_sound = run_out_row(speed, true, over) && timed_sound;
        untimed_sound = run_out_row(speed, false, over) && untimed_sound;
    }
    bench.verdict("A RUN-OUT WITH A PER-BUS TIMEOUT is answered i2c_timeout at about 5 ms, by start(), under the hold: no "
                  "reset, no recover(), and the next request is on the wire behind it",
                  timed_sound);
    bench.verdict("A RUN-OUT WITH NO PER-BUS TIMEOUT is answered the same, and the next request is on the wire behind it",
                  untimed_sound);
    bench.verdict("two requests waiting behind the hold are each answered i2c_timeout, one bound each",
                  run_out_queue(over));
    host_ready();
    bench.verdict("and the command channel is back", command(Op::ping));
}

// ===========================================================================
// w - a request into a tenure another controller is clocking
// ===========================================================================
//
// start() waits bus_free_us for BUSY with both lines watched, and takes SWRST only
// when BUSY stood for all of it and no read found a line low (wait_bus_free()). One
// read could not tell: on a tenure another controller is clocking both lines read
// high in SCL's high phase on a 1, which the fraction below prints. The verdict is a
// count of zero, and what makes it mean something is that the bound ran out in every
// round, judged too; what a poll can miss is the page's arithmetic. The letter stages
// that two ways, this board's request a probe of nobody posted into a tenure of
// the peer's controller, the moment swept by 7 us behind the STOP that opens its
// START:
//  - AT 100 AND 400 kHz, 24 BYTES of 0x55. A tenure past nine clocks needs an
//    acknowledge and no third chip is on the wire, so this board's own client half
//    gives it from the event vector. start()'s own store of CTLR1 drops ACK, so the
//    peer's tenure ends on the byte behind the request either way: its NACK is the
//    staging's. 7 is prime to the 10 us bit and lands on five phases of the 2.5 us
//    one, 18 to 235 us behind, the peer's START being on the wire by the first; the
//    225 bits are 2250 us and 562 us, so the bound ends inside them.
//  - AT 20 kHz, ITS PROBE OF NOBODY: nine clocks of 50 us, 450 us, this board not
//    addressed, its block a bystander as it would be beside a third chip, the moment
//    40 to 257 us behind. What the peer's tenure ends with is then the reset's doing.
// Counted: the resets taken, what the peer's tenure ended with and what this request
// was answered, the tables apart by whether the round took a reset. Beside it, the
// fraction of a tenure both lines stand high: the thread reading both lines through
// one whole tenure of the same bits, no request posted.

struct LiveTally {
    uint8_t rounds = 0;
    uint8_t ran_out = 0;
    uint8_t resets = 0;
    uint8_t stalls = 0;
    uint8_t stall_standing = 0;
    uint16_t taken = 0;
    uint8_t status[2][6] = {};  ///< this request's, by round without and with a reset
    uint8_t peer[2][6] = {};    ///< the peer's tenure's, the same
};

/// A status as a row of the tables above: i2c_ok, nack_addr, nack_data, arb_lost,
/// bus_error, anything else.
uint8_t status_row(uint8_t st) {
    return st == i2c_ok ? 0u : st == i2c_nack_addr ? 1u : st == i2c_nack_data ? 2u : st == i2c_arb_lost ? 3u : st == i2c_bus_error ? 4u : 5u;
}

void print_row(const char* name, const uint8_t (&row)[6]) {
    print(serial, name, ": ok ", row[0], ", nack_addr ", row[1], ", nack_data ", row[2], ", arb_lost ", row[3], ", bus_error ", row[4],
          ", other ", row[5]);
}

/// The foreign tenures: `arb`'s aux8 at the peer (its header): 0, 1 and 2 as the
/// names say, 3 a probe at 50 kHz, and 100 kHz under any other value.
enum class Foreign : uint8_t { bytes_100k = 0, bytes_400k = 1, probe_20k = 2, probe_50k = 3, probe_100k = 4 };

/// The peer's tenure armed, and for its bytes the client half up; its START follows
/// the STOP of the window made here.
bool live_arm(Foreign kind) {
    twilink::Params a{};
    a.ms = 200;
    a.addr = twilink::dut_addr;
    a.aux16 = 120;
    a.aux8 = static_cast<uint8_t>(kind);
    a.pattern = twilink::pattern_fixed;
    if (kind != Foreign::bytes_100k && kind != Foreign::bytes_400k) {
        a.target = nobody_addr;
    } else {
        a.target = twilink::dut_addr;
        a.count = 24;
        a.seed = 0x55;
    }
    if (!peer_act(Op::arb, a)) {
        return false;
    }
    host_ready();
    live_count = 0;
    live_sum = 0;
    error_isr_entries = 0;
    if (kind == Foreign::bytes_100k || kind == Foreign::bytes_400k) {
        H::regs().OADDR1 = static_cast<uint16_t>(twilink::dut_addr << 1);
        H::ack(true);
        live_client = true;
        H::event_interrupt(true);
        H::buffer_interrupt(true);
    }
    host_done = false;
    host_isr_entries = 0;
    host_stormed = false;
    return true;
}

void live_round(LiveTally& t, Foreign kind, uint32_t delay) {
    if (!live_arm(kind)) {
        return;
    }
    const uint16_t resets0 = Host::unwedges();
    const Host::Request q = nobody_probe();
    inject_start();
    brio::spin_us<Ruler>(350);
    inject_stop();
    brio::spin_us<Ruler>(delay);
    const uint32_t c0 = Ticker::cycles();
    const bool sync = Host::start(q);
    const uint32_t c1 = Ticker::cycles();
    // 10 ms: over twenty of the peer's tenures (its longest, 24 bytes and its
    // address at 100 kHz, is 2.25 ms, the 20 kHz probe 0.45 ms).
    const bool answered = sync || brio::wait_until<Ruler>(host_answered, 10'000);
    const uint8_t st = answered ? Host::status() : no_answer;
    live_client = false;
    if (!answered) {
        // Unanswered in 10 ms, over 20 tenures of the peer's. What stands then is
        // the state a per-bus timeout would answer: START in CTLR1, BUSY over both
        // lines high.
        t.stall_standing = static_cast<uint8_t>(t.stall_standing + ((H::regs().CTLR1 & i2c_start) != 0u && H::busy() && SclPin::read() && SdaPin::read() ? 1u : 0u));
        ++t.stalls;
        (void)Host::recover();
    }
    const uint8_t reset = Host::unwedges() != resets0 ? 1u : 0u;
    t.resets = static_cast<uint8_t>(t.resets + reset);
    t.ran_out = static_cast<uint8_t>(t.ran_out + (c1 - c0 >= bus_free_named * bound_per_us ? 1u : 0u));
    t.taken = static_cast<uint16_t>(t.taken + live_count);
    ++t.rounds;
    const uint8_t row = status_row(st);
    t.status[reset][row] = static_cast<uint8_t>(t.status[reset][row] + 1u);
    H::regs().OADDR1 = 0;
    settle_ms(3);
    host_ready();
    twilink::Report r{};
    const uint8_t peer_row = peer_report(r) ? status_row(r.mstatus) : uint8_t{5};
    t.peer[reset][peer_row] = static_cast<uint8_t>(t.peer[reset][peer_row] + 1u);
}

/// One whole tenure read by the thread, no request posted: how many reads found
/// both lines high.
void live_fraction(Foreign kind, uint32_t& reads, uint32_t& both) {
    reads = 0;
    both = 0;
    if (!live_arm(kind)) {
        return;
    }
    inject_start();
    brio::spin_us<Ruler>(350);
    inject_stop();
    brio::spin_us<Ruler>(4);
    // The tenure alone: from the first low SCL to 200 us of both lines high, which
    // no tenure here shows (a clock's high phase is at most 25 us), those reads off.
    // The first low SCL waited for 100 us at most: a tenure that never came leaves
    // the reads below on a quiet wire, counted off as the trailing run.
    const uint32_t c0 = Ticker::cycles();
    (void)brio::wait_until<Ruler>([] { return !SclPin::read(); }, 100);
    uint32_t c_low = Ticker::cycles();
    uint32_t run = 0;
    while (Ticker::cycles() - c_low < 200u * bound_per_us && Ticker::cycles() - c0 < 5'000u * bound_per_us) {
        const bool scl = SclPin::read();
        const bool sda = SdaPin::read();
        ++reads;
        if (scl && sda) {
            ++both;
            ++run;
        } else {
            c_low = Ticker::cycles();
            run = 0;
        }
    }
    reads -= run;
    both -= run;
    live_client = false;
    H::regs().OADDR1 = 0;
    settle_ms(3);
    host_ready();
}

/// THE RAW QUESTION UNDER A STALL: START stored by hand, the vectors dark, into the
/// peer's probe of nobody, this block a bystander: the lines read just before the
/// store, then what becomes of it inside 3 ms, over a whole tenure: SB, or nothing
/// with BUSY standing over both lines high. The moment swept inside the tenure.
void live_raw_row(Foreign kind, uint32_t base, uint32_t step) {
    uint8_t sb[4] = {};
    uint8_t stuck[4] = {};
    uint8_t other[4] = {};
    for (uint8_t k = 0; k < 64u; ++k) {
        if (!live_arm(kind)) {
            continue;
        }
        Pfic::disable(H::event_irq());
        Pfic::disable(H::error_irq());
        inject_start();
        brio::spin_us<Ruler>(350);
        inject_stop();
        brio::spin_us<Ruler>(base + step * (k % 32u));
        const uint8_t where = static_cast<uint8_t>((SclPin::read() ? 2u : 0u) + (SdaPin::read() ? 1u : 0u));
        H::start();
        // 3 ms: over the slowest foreign tenure, the 20 kHz probe's ten clocks of 50 us.
        if (brio::wait_until<Ruler>([] { return (H::status1() & i2c_sb) != 0u; }, 3'000)) {
            ++sb[where];
        } else if (H::busy() && SclPin::read() && SdaPin::read()) {
            ++stuck[where];
        } else {
            ++other[where];
        }
        (void)Host::recover();
        settle_ms(3);
        host_ready();
    }
    for (uint8_t w = 0; w < 4u; ++w) {
        print(serial, "    raw START stored at SCL ", w >> 1, " SDA ", w & 1u, ": SB ", sb[w], ", BUSY standing over both high ", stuck[w], ", else ", other[w], crlf);
    }
}

void tw_live_tenure() {
    if (!need_peer()) {
        return;
    }
    print(serial, "  a raw START into the peer's probe of nobody at 100 kHz, 20 to 51 us behind its START", crlf);
    live_raw_row(Foreign::probe_100k, 20, 1);
    print(serial, "  at 50 kHz, 30 to 123 us behind", crlf);
    live_raw_row(Foreign::probe_50k, 30, 3);
    print(serial, "  at 20 kHz, 30 to 247 us behind", crlf);
    live_raw_row(Foreign::probe_20k, 30, 7);
    bool answered = true;
    bool ran_out = true;
    bool no_reset = true;
    bool bystander_clean = true;
    for (const Foreign kind : {Foreign::bytes_100k, Foreign::bytes_400k, Foreign::probe_20k}) {
        LiveTally t;
        constexpr uint8_t rounds = 64;
        for (uint8_t k = 0; k < rounds; ++k) {
            live_round(t, kind, (kind == Foreign::probe_20k ? 40u : 18u) + 7u * (k % 32u));
        }
        uint32_t reads = 0;
        uint32_t both = 0;
        live_fraction(kind, reads, both);
        print(serial, "  into the peer's ", kind == Foreign::probe_20k ? "probe of nobody at 20" : kind == Foreign::bytes_400k ? "24 bytes at 400" : "24 bytes at 100",
              " kHz: ", t.rounds, " posted, the bound ran out in ",
              t.ran_out, ", RESETS TAKEN ", t.resets, ", unanswered in 10 ms ", t.stalls, " (START and BUSY standing over both lines high in ",
              t.stall_standing, "), bytes taken ", t.taken, crlf);
        print(serial, "    both lines high in ", both, " of ", reads, " reads through one tenure", crlf);
        print_row("    this request, no reset", t.status[0]);
        print(serial, crlf);
        print_row("    this request, a reset", t.status[1]);
        print(serial, crlf);
        print_row("    the peer's tenure, no reset", t.peer[0]);
        print(serial, crlf);
        print_row("    the peer's tenure, a reset", t.peer[1]);
        print(serial, crlf);
        answered = answered && t.rounds == rounds;
        ran_out = ran_out && t.ran_out == t.rounds;
        no_reset = no_reset && t.resets == 0u;
        if (kind == Foreign::probe_20k) {
            bystander_clean = t.peer[0][4] == 0u && t.peer[1][4] == 0u;
        }
    }
    host_ready();
    bench.verdict("every round accounted for: answered on the wire or unanswered, recover() freeing the block, the peer "
                  "reporting",
                  answered);
    bench.verdict("the request ran its whole bound into the tenure, in every round of the three kinds", ran_out);
    bench.verdict("INTO A LIVE TENURE NO RESET IS TAKEN: the watch found a line low, in every round of the three kinds", no_reset);
    bench.verdict("and at 20 kHz, this block a bystander, the other controller's tenure is not ended in i2c_bus_error",
                  bystander_clean);
    bench.verdict("and the command channel is back", command(Op::ping));
}

// ===========================================================================
// p - the dead channel and the release pulse
// ===========================================================================
//
// A DMA channel's requests are an OR (9.2.3), and on this die a raised
// request is HELD until the channel takes it or its peripheral's reset line
// is pulsed (docs/ch32x035/dma.md, "A released requester"): the next owner
// takes one stray item and the channel then serves nothing more - a DEAD
// CHANNEL. The timers' compare requests reach channels 6 and 7 (TIM1's
// channel 3 and TIM2's channel 2, dma_engine.hpp's table), so a request
// raised there with no channel enabled to take it, its gate closed without
// a reset, stages a dead channel under the I2C host's engines with no wire
// of its own.

using DeadTx = Tim<1>;   ///< its channel 3's compare request: DMA channel 6
using DeadRx = Tim<2>;   ///< its channel 2's compare request: DMA channel 7
constexpr uint8_t dead_tx_ch = 2;
constexpr uint8_t dead_rx_ch = 1;
static_assert(DeadTx::compare_dma_channel(dead_tx_ch) == 6 && DeadRx::compare_dma_channel(dead_rx_ch) == 7,
              "9.2.3: TIM1_CH3 rides channel 6 with I2C1_TX, TIM2_CH2 channel 7 with I2C1_RX");

/// A compare request RAISED AND HELD: CCxDE set and the counter run from
/// zero at HCLK past one match of the channel's compare (10 counts in, a
/// period of 1000: some fifty microseconds, two matches), stopped, and the
/// gate closed with no reset. (The software compare event, SWEVGR's CCxG,
/// left the next owner nothing on this die: the match is the request.)
template <class T>
void hold_compare_request(uint8_t ch) {
    T::bus_clock(true);
    TimRegs& r = T::regs();
    r.PSC = 0;
    r.ATRLR = 999;
    r.CHCVR[ch] = 10;
    r.CNT = 0;
    r.DMAINTENR = static_cast<uint16_t>(1u << (9u + ch));
    r.CTLR1 = tim_cen;
    (void)delay_us(clock, 50);
    r.CTLR1 = 0;
    T::bus_clock(false);
}

/// The release contract on the timer: the reset line, then the gate.
template <class T>
void free_compare_request() {
    T::bus_clock(true);
    T::reset();
    T::bus_clock(false);
}

/// The NEXT OWNER of a channel: a transmit block of four bytes into a RAM
/// cell, request-paced (MEM2MEM clear), asked by nothing - any item it moves
/// is a request still standing on the channel's OR. Its count read 300 us
/// after the enable (a held request is taken within a few bus cycles).
/// Programmed by hand in the name of the channel's owner in this image, the
/// I2C's request on it (one owner a channel, util/dma_owner.hpp): the timers
/// program no channel, they raise a request on its OR.
volatile uint8_t sink_cell = 0;
template <uint8_t ch>
uint16_t next_owner_moves() {
    static const uint8_t four[4] = {1, 2, 3, 4};
    using C = DmaChannel<ch, DmaOwner<DmaPeriph::i2c, 1, ch == 6u ? DmaSignal::tx : DmaSignal::rx>>;
    C::stop();
    (void)C::prepare(DmaTransfer{.peripheral = &sink_cell, .memory = const_cast<uint8_t*>(four), .count = 4,
                                 .config = {.direction = DmaDirection::memory_to_peripheral,
                                            .memory_increment = true, .priority = DmaPriority::high}});
    C::enable(true);
    (void)delay_us(clock, 300);
    const uint16_t moved = static_cast<uint16_t>(4u - C::count());
    C::stop();
    return moved;
}

/// A write of 32 on the engines over a channel 6 that TIM1 holds dead: the
/// staging of both the transmit fault and the block's own held request
/// (TxE raised under DMAEN on a channel that never answered it). The engine
/// is left as the tenure left it, TIM1 reset.
uint8_t dead_write() {
    hold_compare_request<DeadTx>(dead_tx_ch);
    (void)peer_act(Op::serve, serve_params(0, 300, 0x10));
    owner = Owner::dma_host;
    (void)DmaHost::init(clock);
    for (uint8_t i = 0; i < 32; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x20u + i);
    }
    const uint8_t st = dma_tenure(twilink::dut_addr, tx_buf, 32, nullptr, 0, link_speed);
    free_compare_request<DeadTx>();
    return st;
}

/// The bytes the peer's serve counted, once its window is over (0xFFFF: no
/// report). The plain host carries the command.
uint16_t peer_took() {
    settle_ms(350);
    twilink::Report r{};
    return peer_report(r) ? r.count : uint16_t{0xFFFF};
}

/// A read of 16 on the engines over a channel 7 that TIM2 holds dead, left
/// STALLED: no recover(), the block as the stall left it (RxNE and BTF
/// standing under DMAEN), TIM2 still holding.
void stall_dead_read() {
    hold_compare_request<DeadRx>(dead_rx_ch);
    (void)peer_act(Op::serve, serve_params(0, 300, 0x40));
    owner = Owner::dma_host;
    (void)DmaHost::init(clock);
    DmaHost::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.tx_len = 0;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.rx_len = 16;
    r.speed = link_speed;
    r.engine = I2cEngine::dma;
    host_done = false;
    clear_counts();
    (void)DmaHost::start(r);
    // Left stalled on purpose: 20 ms, thirteen times the read's 1.5 ms at
    // 100 kHz, lets a read that would complete do so; the caller reads what
    // the next owner takes, so the wait's own answer is not judged.
    (void)brio::wait_until<Ruler>(host_answered, 20'000);
}

void tp_dead_channel() {
    if (!need_peer()) {
        return;
    }
    // THE CONTROL: nothing raised on either channel.
    const uint16_t control6 = next_owner_moves<6>();
    const uint16_t control7 = next_owner_moves<7>();

    // 1. THE DEAD TRANSMIT CHANNEL. The hold proved first by a next owner
    // (which takes the one stray item and leaves the channel dead), then a
    // write of 32 over it: the channel moves the first byte at its enable
    // and nothing more, BTF stands with the event line down, and nothing
    // answers but a bound outside the host (here dma_tenure's wait, then
    // recover()) - never i2c_ok over a short write.
    hold_compare_request<DeadTx>(dead_tx_ch);
    const uint16_t held6 = next_owner_moves<6>();
    free_compare_request<DeadTx>();
    const uint8_t dws = dead_write();
    const uint32_t dw_entries = host_isr_entries;
    const bool dw_storm = host_stormed;
    DmaHost::release();
    host_ready();
    const uint16_t dead_took = peer_took();
    // The same write with the channel live again.
    (void)peer_act(Op::serve, serve_params(0, 300, 0x10));
    owner = Owner::dma_host;
    (void)DmaHost::init(clock);
    const uint8_t lws = dma_tenure(twilink::dut_addr, tx_buf, 32, nullptr, 0, link_speed);
    DmaHost::release();
    host_ready();
    const uint16_t live_took = peer_took();
    print(serial, "  next owner, nothing raised: ch6 ", control6, " ch7 ", control7, "; after TIM1's held request: ",
          held6, " (of 4)", crlf);
    print(serial, "  the write of 32 over the dead channel 6: status ", dws, " (no answer ", no_answer,
          "), the peer took ", dead_took, ", event entries ", dw_entries, dw_storm ? ", STORMED" : "",
          "; with the channel live: status ", lws, ", the peer took ", live_took, crlf);
    bench.verdict("with nothing raised, the next owner of channel 6 or 7 moves nothing", control6 == 0u &&
                                                                                        control7 == 0u);
    bench.verdict("TIM1's compare request, raised and its gate closed with no reset, is HELD: the next owner "
                  "of channel 6 takes one stray item",
                  held6 == 1u);
    bench.verdict("A WRITE OVER THE DEAD CHANNEL never completes - no status claimed, never i2c_ok over a short "
                  "write - its event vector not stormed (at most 4 entries), the far end short of the 32",
                  dws == no_answer && dw_entries <= 4u && !dw_storm && dead_took < 32u);
    bench.verdict("with TIM1 reset the same write lands whole, i2c_ok", lws == i2c_ok && live_took == 32u);

    // 2. THE DEAD RECEIVE CHANNEL: TIM2's request held on channel 7 under
    // an engined read of 16. The channel takes one stray item at its enable
    // and serves no RxNE after it, so the read never completes: the event
    // line is down for the block (the driver), so its vector is NOT stormed
    // by the standing RxNE and BTF - the entries stay at the opening's (SB
    // and ADDR: two, three with a re-entry; a storm would be 60 000) - and
    // the caller's bound answers it (here dma_tenure's tenure_budget_us,
    // 100 ms against the 1.5 ms the read takes at 100 kHz, then recover()).
    hold_compare_request<DeadRx>(dead_rx_ch);
    (void)peer_act(Op::serve, serve_params(0, 300, 0x40));
    owner = Owner::dma_host;
    (void)DmaHost::init(clock);
    for (uint8_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0xEE;
    }
    dma7_entries = 0;
    const uint8_t drs = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 16, link_speed);
    const uint32_t dr_entries = host_isr_entries;
    const uint32_t dr_err = error_isr_entries;
    const bool dr_storm = host_stormed;
    const uint32_t dr_dma = dma7_entries;
    DmaHost::release();
    free_compare_request<DeadRx>();
    host_ready();
    settle_ms(350);
    (void)peer_act(Op::serve, serve_params(0, 300, 0x40));
    owner = Owner::dma_host;
    (void)DmaHost::init(clock);
    for (uint8_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t lrs = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 16, link_speed);
    uint8_t mism = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (rx_buf[i] != twilink::pattern_value(twilink::pattern_counting, 0x40, i)) {
            ++mism;
        }
    }
    DmaHost::release();
    host_ready();
    settle_ms(350);
    print(serial, "  the read of 16 over the dead channel 7: status ", drs, " (no answer ", no_answer,
          "), event entries ", dr_entries, ", error entries ", dr_err, ", channel 7 entries ", dr_dma,
          dr_storm ? ", STORMED" : "", "; with the channel live: status ", lrs, " mism ", mism, crlf);
    bench.verdict("A READ OVER THE DEAD CHANNEL never completes - no status claimed - and its event vector "
                  "is not stormed: the line is down for the block (at most 4 entries, the opening's)",
                  drs == no_answer && dr_entries <= 4u && !dr_storm);
    bench.verdict("after recover(), release() and TIM2's reset the same read is byte-exact", lrs == i2c_ok &&
                                                                                             mism == 0u);

    // 3. THE BLOCK'S OWN REQUEST, let go. A read over the dead
    // channel 7 stalls with RxNE and BTF standing under DMAEN - the I2C's
    // own request raised on a channel that never answered it - and TIM2 is
    // then reset, so whatever the next owner of channel 7 takes is the
    // I2C's. Four ways of letting go, each on a fresh stall: none (the
    // vectors off, the block live: the request still asserted, which proves
    // the staging raised one), the gate closed alone, PE cleared and the
    // gate closed (no reset), and release() - PE cleared and the gate
    // closed, no reset - which must leave the next owner nothing.
    uint16_t moved[4] = {};
    for (uint8_t way = 0; way < 4u; ++way) {
        stall_dead_read();
        Pfic::disable(H::event_irq());
        Pfic::disable(H::error_irq());
        if (way == 1u) {
            H::bus_clock(false);
        } else if (way == 2u) {
            H::disable();
            H::bus_clock(false);
        } else if (way == 3u) {
            DmaHost::release();
        }
        free_compare_request<DeadRx>();
        moved[way] = next_owner_moves<7>();
        H::bus_clock(true);
        H::reset();
        DmaHost::release();
        host_ready();
        (void)peer_took();
    }
    print(serial, "  the block's own request on channel 7 (a read stalled under DMAEN), the next owner moved: "
                  "left live ",
          moved[0], ", the gate closed ", moved[1], ", PE cleared and the gate closed ", moved[2],
          ", after release() ", moved[3], " (of 4)", crlf);
    bench.verdict("the stalled read raised the block's own request on channel 7: left live, the next owner "
                  "takes an item",
                  moved[0] >= 1u);
    bench.verdict("release() - PE cleared and the gate closed, no reset - leaves the next owner nothing",
                  moved[3] == 0u);
    bench.verdict("and no way of letting go hands the next owner more than the one item a held request is",
                  moved[1] <= 1u && moved[2] <= 1u);
    bench.verdict("and the command channel is back", command(Op::ping));
}

// ===========================================================================
// y - the engines' channels starved
// ===========================================================================
//
// The event line is down for a DMA block in either direction, so a channel a load holds
// still is seen by nobody: SCL stands at BTF until the channel moves the next byte, and the
// tenure answers behind the load with every byte. The load is the worst this DMA offers:
// channel 1 at very_high, above the engines' `high`, memory to memory, load_beats words from
// a cell to a cell. A memory-to-memory channel has no request to wait for and gives the
// arbiter no gap (RM 9.1, 9.2.3), so the channels below it move nothing until its block is over.
// It ends by itself, so even a vector re-entering for its whole length gives the suite back.
//
// A row is one tenure at link_speed, the load up before the START or, `late_us` non-zero,
// that long into it, then a read of next_n on the same host, no reset between: what the far
// end gave past the tenure's bytes shows there, and in its count. THE COUNT'S BOUND is exact:
// the target counts a byte as it loads it and stops at the host's NACK, so a tenure of tx_n +
// rx_n bytes and the next read count tx_n + rx_n + next_n, an integer with no instrument
// between (a row with no load is its control). THE ENTRIES' BOUND: SB, ADDR and a write's
// BTF, three; a write-then-read's address phases and the TxE that requests its repeated START
// five; a read's opening two; one more for a re-entry of a line still high. A level
// re-entering without end would take host_isr_budget, which the vector counts and stops.
// THE WINDOW: LAST's NACK follows the channel, so a read of three or more is exposed while
// its second-last byte is on the wire. From the START's request a read's byte k has arrived
// at (k + 1) x 90 us, nine clocks a byte at 100 kHz, the address the first: byte 6 of 8 at
// 630 us, byte 7 at 720; window_us lands in the middle, a request's few cycles early or late
// against a window 90 us wide. What a load there does is printed and its reply judged.
// THE LATE VECTOR: no load, the interrupts masked across a write of 8's end. Its block
// completes as byte 7 goes into the shifter, some (1 + 6) x 90 = 630 us in, and its last
// byte ends at (1 + 8) x 90 = 810 us: masked from 500 us for 500 us, the completion is
// served behind the last byte, BTF up and true.
namespace ly {
using Load = DmaChannel<1>;
constexpr uint16_t load_beats = 30000;
constexpr uint8_t next_n = 4;
constexpr uint16_t third_byte_us = 300;   // the address and two bytes, 27 clocks at 100 kHz
constexpr uint16_t window_us = 675;
constexpr uint16_t masked_from_us = 500;
constexpr uint16_t masked_us = 500;
alignas(4) volatile uint32_t load_cell[2];
/// THE LOAD'S LENGTH: six HCLK cycles an item from SRAM at every width
/// (docs/ch32x035/dma.md, measured), 30000 x 6 = 180000 cycles, 3.75 ms at
/// 48 MHz; 20 ms is over five times that.
constexpr uint32_t load_budget_us = 20'000;

void load_start() {
    Load::stop();
    (void)Load::prepare(DmaTransfer{.peripheral = &load_cell[0], .memory = &load_cell[1], .count = load_beats,
                                    .config = {.direction = DmaDirection::peripheral_to_memory,
                                               .memory_to_memory = true, .memory_increment = false,
                                               .peripheral_width = DmaWidth::word, .memory_width = DmaWidth::word,
                                               .priority = DmaPriority::very_high}});
    Load::enable(true);
}

struct Row {
    uint16_t rows = 0;
    uint16_t ok = 0;         ///< i2c_ok with every byte
    uint16_t short_ok = 0;   ///< i2c_ok without them: the defect
    uint16_t fault = 0;      ///< i2c_dma_fault
    uint16_t nack = 0;       ///< i2c_nack_data
    uint16_t other = 0;      ///< any other answer, or none
    uint16_t early = 0;      ///< answered with the load up
    uint16_t more = 0;       ///< the far end gave more than the tenure asked
    uint16_t next_bad = 0;   ///< the read behind it not i2c_ok, or its bytes not consecutive
    uint32_t entries = 0;    ///< the most event entries a tenure took
    uint16_t stormed = 0;
    uint16_t took_min = 0xFFFFu;
    uint16_t took_max = 0;
};

/// One tenure of tx_n written and rx_n read under the load (or none), then the read behind
/// it. The peer serves a short window; the plain host reads its report afterwards.
void row(uint8_t tx_n, uint8_t rx_n, uint16_t late_us, bool load, Row& out, uint8_t nack_at = 0, bool mask = false) {
    const uint8_t seed = 0x40;
    twilink::Params a = serve_params(0, 40, seed);
    a.nack_at = nack_at;
    (void)peer_act(Op::serve, a);
    owner = Owner::dma_host;
    (void)DmaHost::init(clock);
    for (uint8_t i = 0; i < tx_n; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x20u + i);
    }
    for (uint8_t i = 0; i < rx_n; ++i) {
        rx_buf[i] = 0xEE;
    }
    DmaHost::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.tx_len = tx_n;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.rx_len = rx_n;
    r.speed = link_speed;
    r.engine = I2cEngine::dma;
    host_done = false;
    clear_counts();
    if (load && late_us == 0u) {
        load_start();
    }
    const bool at_once = DmaHost::start(r);
    if (load && late_us != 0u) {
        (void)delay_us(clock, late_us);
        load_start();
    }
    if (mask) {
        (void)delay_us(clock, masked_from_us);
        InterruptGuard guard;
        (void)delay_us(clock, masked_us);
    }
    // The load's end or the tenure's answer, whichever comes first; `early` below and
    // the tenure's own wait judge which it was, so this wait's answer is not.
    if (load) {
        (void)brio::wait_until<Ruler>([] { return Load::count() == 0u || host_done; }, load_budget_us);
    }
    // Beats landing as the load's last one does may answer between the two looks: only a
    // count still standing behind the answer says it came with the load up.
    const bool early = load && host_done && Load::count() != 0u;
    // 20 ms: the longest tenure here, a write of 8 then a read of 2, is ten bytes and
    // two addresses, 108 clocks at 100 kHz, under 1.5 ms at the slowest rate letter f
    // passes, behind a load of 3.75 ms at most.
    const bool answered = at_once || brio::wait_until<Ruler>(host_answered, 20'000);
    const uint8_t st = answered ? DmaHost::status() : no_answer;
    const uint32_t entries = host_isr_entries;
    const bool stormed = host_stormed;
    // Load::stop() below ends the block whether or not it ran out.
    if (load) {
        (void)brio::wait_until<Ruler>([] { return Load::count() == 0u; }, load_budget_us);
    }
    Load::stop();
    if (!answered) {
        (void)DmaHost::recover();
    }
    uint8_t next[next_n] = {};
    const uint8_t next_st = dma_tenure(twilink::dut_addr, nullptr, 0, next, next_n, link_speed);
    DmaHost::release();
    host_ready();
    settle_ms(60);
    twilink::Report rep{};
    const uint16_t took = peer_report(rep) ? rep.count : uint16_t{0xFFFF};
    const uint16_t asked = static_cast<uint16_t>(tx_n + rx_n + next_n);
    bool whole = took >= asked;
    for (uint8_t i = 0; i < rx_n; ++i) {
        whole = whole && rx_buf[i] == twilink::pattern_value(twilink::pattern_counting, seed, i);
    }
    bool next_good = next_st == i2c_ok;
    for (uint8_t i = 1; i < next_n; ++i) {
        next_good = next_good && next[i] == static_cast<uint8_t>(next[i - 1u] + 1u);
    }
    out.rows = static_cast<uint16_t>(out.rows + 1u);
    if (st == i2c_ok) {
        if (whole) {
            out.ok = static_cast<uint16_t>(out.ok + 1u);
        } else {
            out.short_ok = static_cast<uint16_t>(out.short_ok + 1u);
        }
    } else if (st == i2c_dma_fault) {
        out.fault = static_cast<uint16_t>(out.fault + 1u);
    } else if (st == i2c_nack_data) {
        out.nack = static_cast<uint16_t>(out.nack + 1u);
    } else {
        out.other = static_cast<uint16_t>(out.other + 1u);
    }
    out.early = static_cast<uint16_t>(out.early + (early ? 1u : 0u));
    out.more = static_cast<uint16_t>(out.more + (took != 0xFFFFu && took > asked ? 1u : 0u));
    out.next_bad = static_cast<uint16_t>(out.next_bad + (next_good ? 0u : 1u));
    out.stormed = static_cast<uint16_t>(out.stormed + (stormed ? 1u : 0u));
    out.entries = entries > out.entries ? entries : out.entries;
    out.took_min = took < out.took_min ? took : out.took_min;
    out.took_max = took > out.took_max ? took : out.took_max;
}

void print_row(const char* what, const Row& r) {
    print(serial, "  ", what, ", ", r.rows, " rows: i2c_ok whole ", r.ok, ", i2c_ok short ", r.short_ok,
          ", i2c_dma_fault ", r.fault, ", i2c_nack_data ", r.nack, ", else ", r.other, "; with the load up ", r.early,
          "; event entries at most ", r.entries, r.stormed != 0u ? ", STORMED" : "", "; the far end counted ", r.took_min, " to ", r.took_max,
          ", more than asked in ", r.more, ", the next read off in ", r.next_bad, crlf);
}

/// Every byte, answered i2c_ok behind the load, nothing more off the far end than asked, the
/// read behind it clean, the event vector within `entries`.
bool exact(const Row& r, uint16_t asked, uint32_t entries) {
    return r.rows != 0u && r.ok == r.rows && r.early == 0u && r.stormed == 0u && r.next_bad == 0u &&
           r.entries <= entries && r.took_min == asked && r.took_max == asked;
}
}  // namespace ly

void ty_starved() {
    if (!need_peer()) {
        return;
    }
    using ly::next_n;
    using ly::third_byte_us;
    ly::Row plain_w{}, plain_r{}, plain_r2{}, wr2{}, wr8{}, late_w{}, late_wn{}, late_wr{}, held_w{}, held_wn{}, rd1{}, rd2{}, rd8{}, late_r{},
        window{};
    for (uint8_t k = 0; k < 8u; ++k) {
        ly::row(8, 0, 0, false, plain_w);
        ly::row(0, 8, 0, false, plain_r);
        ly::row(0, 2, 0, false, plain_r2);
        ly::row(2, 0, 0, true, wr2);
        ly::row(8, 0, 0, true, wr8);
        ly::row(8, 0, third_byte_us, true, late_w);
        ly::row(8, 0, third_byte_us, true, late_wn, 8);
        ly::row(8, 2, third_byte_us, true, late_wr);
        ly::row(8, 0, 0, false, held_w, 0, true);
        ly::row(8, 0, 0, false, held_wn, 8, true);
        ly::row(0, 1, 0, true, rd1);
        ly::row(0, 2, 0, true, rd2);
        ly::row(0, 8, 0, true, rd8);
        ly::row(0, 8, third_byte_us, true, late_r);
        ly::row(0, 8, ly::window_us, true, window);
    }
    ly::print_row("no load, a write of 8", plain_w);
    ly::print_row("no load, a read of 8", plain_r);
    ly::print_row("no load, a read of 2", plain_r2);
    ly::print_row("the load from the start, a write of 2", wr2);
    ly::print_row("the load from the start, a write of 8", wr8);
    ly::print_row("the load from the third byte, a write of 8", late_w);
    ly::print_row("the same, its last byte refused", late_wn);
    ly::print_row("the load from the third byte, a write of 8 then a read of 2", late_wr);
    ly::print_row("no load, the interrupts masked across a write of 8's end", held_w);
    ly::print_row("the same, its last byte refused", held_wn);
    ly::print_row("the load from the start, a read of 1", rd1);
    ly::print_row("the load from the start, a read of 2", rd2);
    ly::print_row("the load from the start, a read of 8", rd8);
    ly::print_row("the load from the third byte, a read of 8", late_r);
    ly::print_row("the load from the seventh byte, a read of 8", window);
    bench.verdict("with no load every tenure is whole, the far end counting what was asked",
                  ly::exact(plain_w, 8u + next_n, 3u) && ly::exact(plain_r, 8u + next_n, 2u) &&
                      ly::exact(plain_r2, 2u + next_n, 2u));
    bench.verdict("A WRITE STARVED FROM ITS THIRD BYTE answers i2c_ok behind the load, every byte at the far end, no "
                  "fault, the event vector at most 4 entries",
                  ly::exact(late_w, 8u + next_n, 4u) && late_w.fault == 0u);
    bench.verdict("THE SAME WRITE WITH ITS LAST BYTE REFUSED answers i2c_nack_data in every row, never i2c_ok",
                  late_wn.rows != 0u && late_wn.ok == 0u && late_wn.short_ok == 0u && late_wn.nack == late_wn.rows);
    bench.verdict("A WRITE STARVED FROM ITS START: the same", ly::exact(wr2, 2u + next_n, 4u) &&
                                                                  ly::exact(wr8, 8u + next_n, 4u));
    bench.verdict("A WRITE THEN A READ OF TWO, starved from the third byte: every byte both ways, at most 6 entries",
                  ly::exact(late_wr, 8u + 2u + next_n, 6u));
    bench.verdict("A WRITE'S COMPLETION SERVED BEHIND ITS LAST BYTE: every byte, i2c_ok; the last byte refused, "
                  "i2c_nack_data in every row",
                  ly::exact(held_w, 8u + next_n, 3u) && held_wn.rows != 0u && held_wn.nack == held_wn.rows);
    bench.verdict("A READ OF TWO STARVED FROM ITS START: the reply exact and THE FAR END GIVES TWO, the next read "
                  "taking the next byte",
                  ly::exact(rd2, 2u + next_n, 3u));
    bench.verdict("A READ OF ONE OR EIGHT STARVED, from its start or its third byte: exact, the far end giving what "
                  "was asked",
                  ly::exact(rd1, 1u + next_n, 3u) && ly::exact(rd8, 8u + next_n, 3u) &&
                      ly::exact(late_r, 8u + next_n, 3u));
    // What the far end gave past the eight is printed, not judged: how many
    // bytes the controller clocks while the channel is held there - DATAR
    // and the shift register full, the NACK armed by the channel's own
    // reads - is not given by the chapter, and the far end counts a byte as
    // it loads it, before the controller clocks it out.
    print(serial, "  a load starting on the second-last byte of a read of 8: the far end counted ", window.took_min,
          " to ", window.took_max, " (", 8u + next_n, " asked), more than asked in ", window.more, " of ",
          window.rows, " rows", crlf);
    bench.verdict("A READ OF EIGHT CAUGHT ON ITS SEVENTH BYTE: the reply exact, every row i2c_ok with its eight "
                  "bytes",
                  window.rows != 0u && window.ok == window.rows);
    bench.verdict("and the command channel is back", command(Op::ping));
}


// ===========================================================================
// q - the siblings' quirks, re-measured on this die
// ===========================================================================

void tq_quirks() {
    if (!need_peer()) {
        return;
    }
    // ---- 1. A START into a held SDA, raw: answered ARLO (the CH32V006),
    // or let out at the controller's tick (the CH32V203 and CH32V303)?
    // Twice: with the block enabled when the peer took the line - it saw
    // that START condition, as the engine meets a wedge - and with the
    // block brought up over a line already held. ----
    struct Raw {
        bool held;
        uint16_t s2_before;
        uint32_t sb_at, arlo_at, start_gone_at, addr_arlo_at;
        uint16_t s1, s2, c1;
    };
    const auto raw_start = [](bool reinit) {
        Raw w{};
        if (reinit) {
            (void)Host::init(clock);
        }
        Pfic::disable(H::event_irq());
        Pfic::disable(H::error_irq());
        (void)delay_us(clock, 2000);
        w.held = !SdaPin::read();
        w.s2_before = H::status2();
        H::start();
        const uint32_t c0 = Ticker::cycles();
        while (Ticker::cycles() - c0 < 48u * 50'000u) {
            const uint16_t s1 = H::status1();
            const uint32_t now = (Ticker::cycles() - c0) / 48u;
            if ((s1 & i2c_sb) != 0u && w.sb_at == 0u) {
                w.sb_at = now + 1u;
            }
            if ((s1 & i2c_arlo) != 0u && w.arlo_at == 0u) {
                w.arlo_at = now + 1u;
            }
            if (!H::starting() && w.start_gone_at == 0u) {
                w.start_gone_at = now + 1u;
            }
            if (w.sb_at != 0u || w.arlo_at != 0u) {
                break;
            }
        }
        w.s1 = H::status1();
        w.s2 = H::status2();
        w.c1 = H::regs().CTLR1;
        if (w.sb_at != 0u) {
            // EVT5 into the held line: the address of letter h's tenure.
            H::data(static_cast<uint8_t>(twilink::dut_addr << 1));
            // ARLO timed to its rise, 2 ms at most (0 = none), the reading late
            // by one turn of the wait at most (a flag read and a ruler read).
            const std::optional<uint32_t> arlo =
                brio::cycles_until<Ruler>([] { return H::flag(i2c_arlo); }, 2'000);
            w.addr_arlo_at = arlo.has_value() ? *arlo / bound_per_us + 1u : 0u;
        }
        return w;
    };
    const auto say = [](const char* name, const Raw& w) {
        print(serial, "  1. ", name, ": SDA held ", w.held ? "yes" : "NO", ", STAR2 before ", hex(w.s2_before),
              "; START set -> ");
        if (w.arlo_at != 0u) {
            print(serial, "ARLO at ", w.arlo_at, " us");
        } else if (w.sb_at != 0u) {
            print(serial, "SB at ", w.sb_at, " us, the address written into the held line -> ARLO at ",
                  w.addr_arlo_at, " us (0 = none in 2 ms)");
        } else {
            print(serial, "nothing in 50 ms");
        }
        print(serial, "; STAR1=", hex(w.s1), " STAR2=", hex(w.s2), " CTLR1=", hex(w.c1), crlf);
    };
    twilink::Params h{};
    h.ms = 300;
    if (!peer_act(Op::hold_sda, h)) {
        bench.verdict("the peer accepted the hold_sda command", false);
        return;
    }
    const Raw seen = raw_start(false);
    (void)Host::recover();
    settle_ms(400);
    (void)Host::unstick();
    host_ready();
    if (!peer_act(Op::hold_sda, h)) {
        bench.verdict("the peer accepted the second hold_sda command", false);
        return;
    }
    const Raw over = raw_start(true);
    (void)Host::recover();
    settle_ms(400);
    (void)Host::unstick();
    host_ready();
    say("the block up when the peer took SDA", seen);
    say("the block brought up over the held line", over);
    // WHAT A START INTO A HELD SDA DOES - SB after some tens of microseconds,
    // ARLO, or a START left pending - the chapter does not say, and the
    // siblings answer differently (the header): the habit is printed above,
    // and the driver's outcome under it is letter h's verdict. What is
    // judged is the arbitration the chapter does state: once SB has risen,
    // the address's first bit is a 1 the held line reads 0, and the
    // controller loses arbitration on it - within the START's hold and that
    // bit's low and high phases, under two SCL periods at the slowest rate
    // letter f passes (13.3 us at 100 kHz, 26.7 us), plus the microsecond of
    // the reading's rounding.
    constexpr uint32_t arlo_within_us = 2u * 40u / 3u + 1u;   // two periods of 13.3 us, rounded down, +1
    if (!seen.held || !over.held) {
        bench.skip("the far chip did not hold SDA at both STARTs: the arbitration not staged");
    } else if (seen.sb_at == 0u) {
        bench.skip("SB never rose into the held line here: the address's arbitration not staged");
    } else {
        bench.verdict("SB up into the held line, the address's first one bit loses arbitration within two SCL "
                      "periods at the slowest rate letter f passes - the engine's i2c_arb_lost of letter h",
                      seen.addr_arlo_at != 0u && seen.addr_arlo_at <= arlo_within_us);
    }

    // ---- 2. How long the host's own STOP stands in CTLR1 after the
    // completion edge, at both speeds. ----
    const twilink::Params a = serve_params(0, 400, 0x20);
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve", false);
        return;
    }
    uint32_t stand[2] = {};
    uint8_t sts[2] = {};
    for (uint8_t i = 0; i < 2u; ++i) {
        const I2cSpeed sp = i == 0u ? I2cSpeed::standard_100k : I2cSpeed::fast_400k;
        (void)host_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, sp);   // the speed applied
        settle_ms(1);
        Host::Request r{};
        r.addr = twilink::dut_addr;
        tx_buf[0] = 0x5A;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        r.tx_len = 1;
        r.speed = sp;
        host_done = false;
        // The completion edge, 10 ms at most: a write of one is two bytes, 18
        // clocks, under 0.25 ms at the slowest rate letter f passes. A write
        // never answered is no_answer, failing the verdict, its STOP not timed.
        // The edge is seen at most one turn of the wait late (a flag read and a
        // ruler read, some 60 cycles), which shortens the figure by as much.
        // The STOP looked at once behind it - 0 when it no longer stood - then
        // timed to its fall, 1 ms at most (0xFFFFFFFF: it never fell).
        if (Host::start(r) || brio::wait_until<Ruler>(host_answered, 10'000)) {
            const bool stood = H::stopping();
            const std::optional<uint32_t> gone =
                brio::cycles_until<Ruler>([] { return !H::stopping(); }, 1'000);
            stand[i] = stood ? gone.value_or(0xFFFF'FFFFu) : 0u;
            sts[i] = Host::status();
        } else {
            (void)Host::recover();
            sts[i] = no_answer;
        }
    }
    print(serial, "  2. the host's STOP stands in CTLR1 for ", stand[0], " cycles at 100 kHz and ", stand[1],
          " at 400 kHz after the completion edge (a bit is 480 and 120); status ", sts[0], "/", sts[1], crlf);
    // THE CEILING: the vector stores STOP before it raises the completion
    // edge, and STOP is cleared when the STOP condition is detected
    // (15.9.1): the rest of the last clock's low phase, then the high phase
    // before SDA rises - one SCL period at most, at the slowest rate letter f
    // passes (13.3 us at 100 kHz, 640 cycles; 3.3 us at 400 kHz, 160). The
    // reading adds one turn of cycles_until() at most (the fall seen late; the
    // edge seen late only shortens it): a composed ruler read, 76 cycles
    // (docs/ch32x035/platform.md), and a register load, under twice the read.
    constexpr uint32_t stop_turn = 2u * 76u;
    constexpr uint32_t stop_100k = SysClock::hz / 75'000u + stop_turn;    // 100 kHz a quarter slower
    constexpr uint32_t stop_400k = SysClock::hz / 300'000u + stop_turn;   // 400 kHz a quarter slower
    bench.verdict("the host's own STOP stands in CTLR1 after the completion edge at 100 kHz, and at both speeds "
                  "for no more than one SCL period at the slowest rate letter f passes and the reading's turn - "
                  "what start()'s bounded wait spends",
                  sts[0] == i2c_ok && sts[1] == i2c_ok && stand[0] > 0u && stand[0] <= stop_100k &&
                      stand[1] <= stop_400k);

    // ---- 3. A CTLR1 store while that STOP stands: a second STOP request? ----
    {
        Host::Request r{};
        r.addr = twilink::dut_addr;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        r.tx_len = 1;
        r.speed = I2cSpeed::standard_100k;
        (void)host_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, I2cSpeed::standard_100k);
        settle_ms(1);
        host_done = false;
        // The completion edge, 10 ms at most (as above); a write never answered
        // leaves the store below on a host still in its tenure, and the next
        // tenure, judged, shows it.
        const bool edge = Host::start(r) || brio::wait_until<Ruler>(host_answered, 10'000);
        const uint16_t at = H::regs().CTLR1;
        H::regs().CTLR1 = static_cast<uint16_t>(H::regs().CTLR1 | i2c_ack);   // a read-modify-write
        settle_ms(1);
        const uint16_t c1_later = H::regs().CTLR1;
        const uint16_t s2_later = H::status2();
        H::ack(false);
        const uint8_t next = host_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, I2cSpeed::standard_100k);
        print(serial, "  3. a CTLR1 read-modify-write with STOP standing (CTLR1 ", hex(at), "): a ms later CTLR1 ",
              hex(c1_later), " STAR2 ", hex(s2_later), ", the next tenure ", next,
              edge ? "" : " (the write before it NEVER ANSWERED)", crlf);
        bench.verdict("a CTLR1 store under the host's own STOP was measured and reported (either answer "
                      "is a finding), and the next tenure ran",
                      edge && next == i2c_ok);
        if (next != i2c_ok) {
            (void)Host::recover();
            (void)Host::unstick();
        }
    }
    settle_ms(450);

    // ---- 4. STOPF on a STOP the client half was not addressed in ----
    host_ready();
    Pfic::disable(H::event_irq());
    (void)H::status1();
    const uint16_t pre = H::status1();
    hold_bus_busy(30);   // a START and a STOP by hand, no address
    (void)delay_us(clock, 30);
    const uint16_t host_s1 = H::status1();
    H::ack(true);   // the client half acknowledging, as a target would
    hold_bus_busy(30);
    (void)delay_us(clock, 30);
    const uint16_t ack_s1 = H::status1();
    if ((ack_s1 & i2c_stopf) != 0u || (host_s1 & i2c_stopf) != 0u) {
        H::clear_stopf();
    }
    H::ack(false);
    print(serial, "  4. a START and STOP by hand, nobody addressed: STAR1 ", hex(pre), " -> ", hex(host_s1),
          " with ACK clear, ", hex(ack_s1), " with ACK set", crlf);
    bench.verdict("STOPF on an unaddressed STOP was measured with ACK clear and set, and cleared",
                  (H::status1() & i2c_stopf) == 0u);
    host_ready();
    bench.verdict("the command channel is back", command(Op::ping));
}

// ===========================================================================
// r - the repeated START of a write-then-read, acknowledged and refused
// ===========================================================================

uint8_t either_tenure(bool dma, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
                      I2cSpeed speed) {
    return dma ? dma_tenure(twilink::dut_addr, tx, tx_len, rx, rx_len, speed)
               : host_tenure(twilink::dut_addr, tx, tx_len, rx, rx_len, speed);
}

void bring_up(bool dma) {
    if (dma) {
        owner = Owner::dma_host;
        (void)DmaHost::init(clock);
    }
}
void put_down(bool dma) {
    if (dma) {
        DmaHost::release();
        host_ready();
    }
}

void tr_restart() {
    if (!need_peer()) {
        return;
    }
    bool ack_ok = true;
    bool refuse_status = true;
    bool refuse_clean = true;
    bool refuse_next = true;
    for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
        for (const bool dma : {false, true}) {
            twilink::Params a = serve_params(0, 300,
                                             static_cast<uint8_t>(0x50u + (dma ? 8u : 0u) +
                                                                  static_cast<uint8_t>(speed)));
            if (!peer_act(Op::serve, a)) {
                bench.verdict("the peer accepted the serve", false);
                return;
            }
            bring_up(dma);
            dma6_entries = 0;
            dma7_entries = 0;
            uint16_t sum = 0;
            uint8_t mism = 0;
            uint8_t sts[4] = {};
            uint32_t ev[4] = {};
            for (uint8_t n = 1; n <= 4u; ++n) {
                for (uint8_t i = 0; i < n; ++i) {
                    tx_buf[i] = static_cast<uint8_t>(0x31u * n + i);
                    sum = static_cast<uint16_t>(sum + tx_buf[i]);
                }
                for (uint8_t i = 0; i < 4u; ++i) {
                    rx_buf[i] = 0xEE;
                }
                sts[n - 1u] = either_tenure(dma, tx_buf, n, rx_buf, 4, speed);
                ev[n - 1u] = host_isr_entries;
                for (uint8_t i = 0; i < 4u; ++i) {
                    if (rx_buf[i] != twilink::pattern_value(a.pattern, a.seed,
                                                            static_cast<uint16_t>(4u * (n - 1u) + i))) {
                        ++mism;
                    }
                }
            }
            // Under the engines both halves of each of the four ride them:
            // one completion of each block a tenure.
            const uint32_t e6 = dma6_entries;
            const uint32_t e7 = dma7_entries;
            put_down(dma);
            settle_ms(350);
            twilink::Report r{};
            const bool rep = peer_report(r);
            const bool good = rep && sts[0] == i2c_ok && sts[1] == i2c_ok && sts[2] == i2c_ok &&
                              sts[3] == i2c_ok && mism == 0u && r.aux0 == 10u && r.sum == sum &&
                              r.addr_hits == 8u && e6 == (dma ? 4u : 0u) && e7 == (dma ? 4u : 0u);
            ack_ok = ack_ok && good;
            print(serial, "  ", i2c_speed_hz(speed) / 1000u, " kHz ", dma ? "engines" : "pump   ",
                  " acknowledged: status ", sts[0], "/", sts[1], "/", sts[2], "/", sts[3], ", event entries ",
                  ev[0], "/", ev[1], "/", ev[2], "/", ev[3], ", DMA entries ", e6, "+", e7, ", read mism ", mism, "; peer took ", r.aux0,
                  " (10) sum ", hex(r.sum), " (", hex(sum), ") address hits ", r.addr_hits, " (8)",
                  good ? "" : "  <- WRONG", crlf);

            for (uint8_t n = 1; n <= 4u; ++n) {
                twilink::Params f = serve_params(0, 150, 0);
                f.nack_at = n;
                if (!peer_act(Op::serve, f)) {
                    bench.verdict("the peer accepted the nack-at serve", false);
                    return;
                }
                bring_up(dma);
                for (uint8_t i = 0; i < n; ++i) {
                    tx_buf[i] = static_cast<uint8_t>(0x70u + i);
                }
                const uint8_t st = either_tenure(dma, tx_buf, n, rx_buf, 4, speed);
                const uint16_t c1 = H::regs().CTLR1;
                settle_ms(1);
                const uint16_t c1_later = H::regs().CTLR1;
                const uint16_t s2_later = H::status2();
                const uint8_t next = either_tenure(dma, nullptr, 0, nullptr, 0, speed);
                put_down(dma);
                settle_ms(200);
                twilink::Report fr{};
                const bool frep = peer_report(fr);
                const bool st_ok = st == i2c_nack_data && frep && (fr.flags & twilink::report_nacked) != 0u;
                const bool clean = (c1_later & (i2c_start | i2c_stop)) == 0u && (s2_later & i2c_msl) == 0u;
                refuse_status = refuse_status && st_ok;
                refuse_clean = refuse_clean && clean;
                // The next tenure, a probe of the same address a millisecond
                // later, must COMPLETE on the wire: i2c_ok, or - the far end
                // being a CH32 target that keeps its acknowledge down after
                // the refused byte until its window closes (twi_peer's
                // header) - i2c_nack_addr; a stall or a wedge is neither.
                refuse_next = refuse_next && (next == i2c_ok || next == i2c_nack_addr);
                print(serial, "  ", i2c_speed_hz(speed) / 1000u, " kHz ", dma ? "engines" : "pump   ", " ", n,
                      " written, the last refused: status ", st, ", CTLR1 at the end ", hex(c1), " then ",
                      hex(c1_later), ", STAR2 ", hex(s2_later), ", the next tenure ", next, "; peer flags ",
                      hex(fr.flags), " address hits ", fr.addr_hits,
                      st_ok && clean && (next == i2c_ok || next == i2c_nack_addr) ? "" : "  <- WRONG", crlf);
            }
        }
    }
    bench.verdict("one to four bytes written then four read, both speeds, pump and engines (both halves on "
                  "them, each block's completion once a tenure): every written byte taken by the peer - "
                  "a POLLED CH32 target - every read byte its pattern",
                  ack_ok);
    bench.verdict("the last written byte refused: i2c_nack_data, the peer's own NACK", refuse_status);
    bench.verdict("and the controller lets the bus go: no START or STOP left standing, MSL clear",
                  refuse_clean);
    bench.verdict("the next tenure on the same host runs to a wire outcome - i2c_ok, or the address "
                  "refused by a peer still deaf from its refusal - never a stall", refuse_next);
}

// ===========================================================================
// n - the engine a request names
// ===========================================================================

/// The two channels' configuration word and count, read together: what a
/// tenure that touches no channel leaves exactly as it found.
struct Channels {
    uint32_t cfgr6, cntr6, cfgr7, cntr7;
    bool operator==(const Channels&) const = default;
};
Channels channels_now() {
    return Channels{DmaChannel<6>::regs().CFGR, DmaChannel<6>::regs().CNTR, DmaChannel<7>::regs().CFGR,
                    DmaChannel<7>::regs().CNTR};
}

/// Arm a counting serve at the peer and hand the vectors to the DMA host.
bool engine_serve(uint8_t seed, uint16_t ms, uint8_t nack_at = 0) {
    twilink::Params a{};
    a.count = 0;   // the deadline ends it
    a.ms = ms;
    a.addr = twilink::dut_addr;
    a.seed = seed;
    a.pattern = twilink::pattern_counting;
    a.nack_at = nack_at;
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve", false);
        return false;
    }
    bring_up(true);
    dma6_entries = 0;
    dma7_entries = 0;
    return true;
}

/// The serve's report once its window is over, the plain host carrying the
/// command again.
bool engine_report(twilink::Report& r, uint16_t ms) {
    put_down(true);
    settle_ms(ms);
    return peer_report(r);
}

/// WHICH ENGINE MOVES A HALF IS THE REQUEST'S TO NAME (ch32x035/i2c.hpp,
/// util/i2c_bus.hpp's I2cEngine). Five rows, each against the peer's
/// counting serve, each judged by its statuses, its bytes, the peer's
/// count and sum, and the DMA vectors' entries (a completion of the
/// transmit block on channel 6, of the receive block on channel 7):
///   1. `dma` at the shortest lengths the silicon allows, one byte each
///      way and a register read of 1 + 1, every half on the engines;
///   2. `interrupt` at sixteen bytes each way and a 4 + 4: no channel
///      touched - no entry, CFGR and CNTR as init() left them;
///   3. `dma` on the host with no engines: i2c_rejected inside start(),
///      the vector never entered, nothing on the wire, the next request
///      exact;
///   4. the protocol's edges under `dma`: an address nobody answers, a
///      write and a read, i2c_nack_addr; a NACK on the third of six bytes
///      written, i2c_nack_data, the transmit block put away; the next
///      tenure i2c_ok.
void tn_engine() {
    if (!need_peer()) {
        return;
    }
    // ---- 1. dma at one byte ----
    if (!engine_serve(0x70, 300)) {
        return;
    }
    const uint32_t txf0 = DmaTxEngine<6>::faults();
    const uint32_t rxf0 = DmaRxEngine<7>::faults();
    tx_buf[0] = 0xA1;
    tx_buf[1] = 0xA2;
    rx_buf[0] = 0xEE;
    rx_buf[1] = 0xEE;
    const uint8_t w1 = dma_tenure(twilink::dut_addr, tx_buf, 1, nullptr, 0, link_speed);
    const uint32_t w1_e6 = dma6_entries;
    const uint8_t r1 = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 1, link_speed);
    const uint32_t r1_e7 = dma7_entries;
    const uint8_t wr = dma_tenure(twilink::dut_addr, tx_buf + 1, 1, rx_buf + 1, 1, link_speed);
    const uint32_t e6 = dma6_entries;
    const uint32_t e7 = dma7_entries;
    const uint32_t faults = (DmaTxEngine<6>::faults() - txf0) + (DmaRxEngine<7>::faults() - rxf0);
    const bool bytes = rx_buf[0] == twilink::pattern_value(twilink::pattern_counting, 0x70, 0) &&
                       rx_buf[1] == twilink::pattern_value(twilink::pattern_counting, 0x70, 1);
    twilink::Report a{};
    const bool arep = engine_report(a, 350);
    print(serial, "  1. dma: write 1 ", w1, " (ch6 ", w1_e6, "), read 1 ", r1, " (ch7 ", r1_e7, "), 1+1 ", wr,
          "; ch6 ", e6, " ch7 ", e7, " faults ", faults, "; read ", hex(rx_buf[0]), " ", hex(rx_buf[1]),
          "; peer count ", a.count, " sum ", hex(a.sum), " hits ", a.addr_hits, crlf);
    bench.verdict("dma at ONE BYTE: a one-byte write, a one-byte read and a 1 + 1 register read on the "
                  "engines - each block's completion once a half (ch6 1, 2; ch7 1, 2), no fault - "
                  "i2c_ok, the read bytes the peer's, the written ones taken (count 4, sum 0x143, "
                  "4 address matches)",
                  w1 == i2c_ok && r1 == i2c_ok && wr == i2c_ok && w1_e6 == 1u && r1_e7 == 1u && e6 == 2u &&
                      e7 == 2u && faults == 0u && bytes && arep && a.count == 4u && a.sum == 0x143u &&
                      a.addr_hits == 4u);

    // ---- 2. interrupt at sixteen bytes touches no channel ----
    if (!engine_serve(0x30, 300)) {
        return;
    }
    for (uint8_t i = 0; i < 16; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x10u + i);
        rx_buf[i] = 0xEE;
    }
    const Channels before = channels_now();
    const uint8_t iw = dma_tenure(twilink::dut_addr, tx_buf, 16, nullptr, 0, link_speed, I2cEngine::interrupt);
    const uint32_t iw_ev = host_isr_entries;
    const uint8_t ir = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 16, link_speed, I2cEngine::interrupt);
    const uint32_t ir_ev = host_isr_entries;
    const uint8_t ic = dma_tenure(twilink::dut_addr, tx_buf, 4, rx_buf + 16, 4, link_speed, I2cEngine::interrupt);
    const bool untouched = channels_now() == before;
    const uint32_t ie = dma6_entries + dma7_entries;
    uint8_t imism = 0;
    for (uint8_t i = 0; i < 20; ++i) {
        if (rx_buf[i] != twilink::pattern_value(twilink::pattern_counting, 0x30, i)) {
            ++imism;
        }
    }
    twilink::Report b{};
    const bool brep = engine_report(b, 350);
    print(serial, "  2. interrupt: write 16 ", iw, " (", iw_ev, " event entries), read 16 ", ir, " (", ir_ev,
          "), 4+4 ", ic, "; DMA entries ", ie, ", channels ", untouched ? "untouched" : "TOUCHED", ", mism ",
          imism, "; peer count ", b.count, crlf);
    bench.verdict("interrupt at SIXTEEN BYTES on the host with engines bound touches no channel: no DMA "
                  "entry, both channels' CFGR and CNTR as init() left them, one event entry a byte or "
                  "more - and the three tenures i2c_ok, exact (count 40)",
                  iw == i2c_ok && ir == i2c_ok && ic == i2c_ok && untouched && ie == 0u && iw_ev >= 16u &&
                      ir_ev >= 16u && imism == 0u && brep && b.count == 40u);

    // ---- 3. dma on the host with no engines ----
    twilink::Params q{};
    q.count = 0;
    q.ms = 250;
    q.addr = twilink::dut_addr;
    if (!peer_act(Op::serve, q)) {
        bench.verdict("the peer accepted the serve", false);
        return;
    }
    host_ready();
    tx_buf[0] = 0x11;
    tx_buf[1] = 0x22;
    const uint8_t rej = host_tenure(twilink::dut_addr, tx_buf, 2, nullptr, 0, link_speed, I2cEngine::dma);
    const uint32_t rej_ev = host_isr_entries;
    const bool rej_msl = (H::status2() & i2c_msl) != 0u;
    const uint8_t after = host_tenure(twilink::dut_addr, tx_buf, 2, nullptr, 0, link_speed);
    settle_ms(300);
    twilink::Report d{};
    const bool drep = peer_report(d);
    print(serial, "  3. dma on the engineless host: ", rej, " (i2c_rejected ", i2c_rejected, "), event entries ",
          rej_ev, rej_msl ? ", MSL SET" : "", "; the next ", after, "; peer hits ", d.addr_hits, " count ",
          d.count, crlf);
    bench.verdict("dma on the host with NO ENGINES: i2c_rejected inside start(), the vector never entered, "
                  "no START (MSL clear), nothing on the wire - the peer saw the next request alone, "
                  "exact",
                  rej == i2c_rejected && rej_ev == 0u && !rej_msl && after == i2c_ok && drep &&
                      d.addr_hits == 1u && d.count == 2u && d.sum == 0x33u);

    // ---- 4. the protocol's edges under dma ----
    if (!engine_serve(0x60, 400, 3)) {
        return;
    }
    const uint32_t txf5 = DmaTxEngine<6>::faults();
    for (uint8_t i = 0; i < 6; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x61u + i);
    }
    const uint8_t na_w = dma_tenure(nobody_addr, tx_buf, 6, nullptr, 0, link_speed);
    const uint8_t na_r = dma_tenure(nobody_addr, nullptr, 0, rx_buf, 4, link_speed);
    const uint8_t nd = dma_tenure(twilink::dut_addr, tx_buf, 6, nullptr, 0, link_speed);
    const bool ch6_off = !DmaChannel<6>::enabled();
    const uint32_t nd_faults = DmaTxEngine<6>::faults() - txf5;
    twilink::Report e{};
    const bool erep = engine_report(e, 450);
    // The peer's own NACK at the third byte holds its acknowledge down
    // until the window closes (twi_peer's header): the next tenure is
    // judged on a fresh serve.
    if (!engine_serve(0x60, 250)) {
        return;
    }
    const uint8_t nx = dma_tenure(twilink::dut_addr, tx_buf, 6, nullptr, 0, link_speed);
    twilink::Report f{};
    const bool frep = engine_report(f, 300);
    print(serial, "  4. dma edges: nobody, write ", na_w, " read ", na_r, "; NACK at the 3rd byte ", nd,
          " (channel 6 ", ch6_off ? "off" : "STILL ON", ", faults ", nd_faults, ", peer flags ", hex(e.flags),
          "); the next ", nx, " (peer count ", f.count, ")", crlf);
    bench.verdict("the protocol's edges under dma: an address nobody answers is i2c_nack_addr writing and "
                  "reading; a NACK on the third byte written is i2c_nack_data, the peer's own, the "
                  "transmit block put away (channel 6 off, counted once); the next write i2c_ok, whole",
                  na_w == i2c_nack_addr && na_r == i2c_nack_addr && nd == i2c_nack_data && ch6_off &&
                      nd_faults == 1u && erep && (e.flags & twilink::report_nacked) != 0u && nx == i2c_ok &&
                      frep && f.count == 6u);
    host_ready();
}

// ===========================================================================
// x - the cost
// ===========================================================================

struct Cost {
    uint8_t status;
    uint32_t wall;      ///< cycles, start() to the completion edge
    uint32_t start;     ///< cycles inside start()
    uint32_t entries;   ///< event vector entries
    uint32_t body;      ///< cycles inside the event vector, summed
    uint32_t longest;   ///< the longest event vector entry, in cycles
    uint32_t dma;       ///< DMA vector entries
    uint32_t dma_body;  ///< cycles inside the DMA vectors, summed
    uint32_t after;     ///< event vector entries in the millisecond after the edge
    /// What the tenure cost the core between the stamps: start(), the
    /// event entries and the DMA entries (each entry's hardware prologue
    /// and epilogue lie outside, docs/ch32x035/platform.md).
    uint32_t core() const { return start + body + dma_body; }
};

/// What an entry costs the core outside the stamps: the hardware
/// prologue's entry and the exit of a minimal handler, 36 and 34 cycles
/// (docs/ch32x035/platform.md, the interrupt round trip).
constexpr uint32_t entry_trip = 70;

/// A tenure's cost to the core: Cost::core() and the trip of every entry.
uint32_t core_cost(const Cost& c) { return c.core() + entry_trip * (c.entries + c.dma); }

template <typename Hst>
Cost cost_of(uint8_t tx_len, uint8_t rx_len, I2cEngine engine = I2cEngine::interrupt,
             I2cSpeed speed = I2cSpeed::fast_400k) {
    typename Hst::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.rx_len = rx_len;
    r.speed = speed;
    r.engine = engine;
    Cost c{};
    // A console still sending would put its vector inside the timed tenure:
    // the shape is then not run, and its status reads no_answer, failing every
    // verdict that asks i2c_ok of it.
    if (!console_drain()) {
        c.status = no_answer;
        return c;
    }
    host_done = false;
    clear_counts();
    dma6_entries = 0;
    dma7_entries = 0;
    const uint32_t c0 = Ticker::cycles();
    const bool sync = Hst::start(r);
    c.start = Ticker::cycles() - c0;
    if (!sync) {
        // The status below says whether it came; the wall is read at most one
        // turn of the wait late (a flag read and a ruler read).
        (void)brio::wait_until<Ruler>(host_answered, tenure_budget_us);
    }
    c.wall = Ticker::cycles() - c0;
    c.status = host_done || sync ? Hst::status() : no_answer;
    c.entries = host_isr_entries;
    c.body = ev_cycles;
    c.longest = ev_max;
    c.dma = dma6_entries + dma7_entries;
    c.dma_body = dma_cycles;
    (void)delay_us(clock, 1000);
    c.after = host_isr_entries - c.entries;
    return c;
}

/// The longest event vector entry over the shapes letter x printed: the
/// family's longest handler with every driver in the flash
/// (docs/ch32x035/platform.md), a hold of main context beside any polled
/// loop.
uint32_t longest_entry = 0;

void print_cost(const char* name, const Cost& c, uint32_t wire) {
    const uint32_t rest = c.wall > wire ? c.wall - wire : 0u;
    print(serial, "    ", name, ": status ", c.status, ", wall ", c.wall, " = wire ", wire, " + rest ", rest,
          "; start() ", c.start, "; event entries ", c.entries, " (", c.entries != 0u ? c.body / c.entries : 0u,
          " cycles each, ", c.longest, " at most), DMA ", c.dma, " (", c.dma_body, " cycles), core ", c.core(),
          " (", core_cost(c), " with the entries' trips), after the edge ", c.after, crlf);
    if (c.longest > longest_entry) {
        longest_entry = c.longest;
    }
}

/// THE ENGINE'S ARITHMETIC for one direction at one speed, in core cycles
/// (core_cost()): b the pump's cycles a byte, the slope between its 16- and
/// 255-byte tenures; F the pump's fixed part, its 16-byte tenure less
/// sixteen b; E the engines' cost of a one-byte tenure, which does not grow
/// with the length (the 16-byte one printed beside it). The engines cost
/// the core fewer cycles from the smallest N with E <= F + N x b; 0 when
/// they already do at no byte. A read of one or two bytes is a procedure of
/// its own on the pump, compared as measured (the lines above). Printed,
/// not judged: docs/ch32x035/i2c.md's guidance is read from these lines.
void engine_arithmetic(const char* what, const Cost& p16, const Cost& p255, const Cost& e1, const Cost& e16) {
    const int32_t b = (static_cast<int32_t>(core_cost(p255)) - static_cast<int32_t>(core_cost(p16))) / 239;
    const int32_t f = static_cast<int32_t>(core_cost(p16)) - 16 * b;
    const int32_t e = static_cast<int32_t>(core_cost(e1));
    const int32_t n = e <= f || b <= 0 ? 0 : (e - f + b - 1) / b;
    print(serial, "    ", what, ": b ", b, ", F ", f, ", E ", e, " (", core_cost(e16), " at 16 bytes), the engines "
          "cheaper from N = ", n, crlf);
}

void tx_cost() {
    if (!need_peer()) {
        return;
    }
    if (!peer_act(Op::serve, serve_params(0, 4000, 0x10))) {
        bench.verdict("the peer accepted the serve", false);
        return;
    }
    const uint32_t served_at = Ticker::millis();
    for (uint16_t i = 0; i < 255u; ++i) {
        tx_buf[i] = static_cast<uint8_t>(i);
    }
    longest_entry = 0;
    host_ready();
    // The wire's own time at the SCL period in force: the slope of two
    // writes on the engines, 255 and 16 bytes, 239 x 9 SCL periods apart.
    owner = Owner::dma_host;
    (void)DmaHost::init(clock);
    constexpr I2cEngine dma = I2cEngine::dma;
    (void)cost_of<DmaHost>(1, 0, dma);   // the speed applied
    const Cost e255 = cost_of<DmaHost>(255, 0, dma);
    const Cost e16 = cost_of<DmaHost>(16, 0, dma);
    const uint32_t period = (e255.wall - e16.wall) / (239u * 9u);
    const auto wire = [&](uint32_t bytes) { return bytes * 9u * period; };
    const Cost e1 = cost_of<DmaHost>(1, 0, dma);
    const Cost e2 = cost_of<DmaHost>(2, 0, dma);
    const Cost er1 = cost_of<DmaHost>(0, 1, dma);
    const Cost er2 = cost_of<DmaHost>(0, 2, dma);
    const Cost er16 = cost_of<DmaHost>(0, 16, dma);
    const Cost er255 = cost_of<DmaHost>(0, 255, dma);
    const Cost ereg1 = cost_of<DmaHost>(1, 1, dma);
    const Cost ereg = cost_of<DmaHost>(1, 4, dma);
    DmaHost::release();
    host_ready();
    (void)cost_of<Host>(1, 0);
    const Cost w1 = cost_of<Host>(1, 0);
    const Cost w2 = cost_of<Host>(2, 0);
    const Cost w16 = cost_of<Host>(16, 0);
    const Cost w255 = cost_of<Host>(255, 0);
    const Cost r1 = cost_of<Host>(0, 1);
    const Cost r2 = cost_of<Host>(0, 2);
    const Cost r16 = cost_of<Host>(0, 16);
    const Cost r255 = cost_of<Host>(0, 255);
    const Cost reg1 = cost_of<Host>(1, 1);
    const Cost reg4 = cost_of<Host>(1, 4);
    const Cost probe = cost_of<Host>(0, 0);
    print(serial, "  at 400 kHz, the SCL period from the engines' slope: ", period, " cycles (",
          period != 0u ? SysClock::hz / period / 1000u : 0u, " kHz)", crlf);
    print(serial, "  the pump:", crlf);
    print_cost("write 1      ", w1, wire(2));
    print_cost("write 2      ", w2, wire(3));
    print_cost("write 16     ", w16, wire(17));
    print_cost("write 255    ", w255, wire(256));
    print_cost("read 1       ", r1, wire(2));
    print_cost("read 2       ", r2, wire(3));
    print_cost("read 16      ", r16, wire(17));
    print_cost("read 255     ", r255, wire(256));
    print_cost("reg 1 + 1    ", reg1, wire(4));
    print_cost("reg 1 + 4    ", reg4, wire(7));
    print_cost("probe        ", probe, wire(1));
    print(serial, "  the engines (I2cEngine::dma):", crlf);
    print_cost("write 1      ", e1, wire(2));
    print_cost("write 2      ", e2, wire(3));
    print_cost("write 16     ", e16, wire(17));
    print_cost("write 255    ", e255, wire(256));
    print_cost("read 1       ", er1, wire(2));
    print_cost("read 2       ", er2, wire(3));
    print_cost("read 16      ", er16, wire(17));
    print_cost("read 255     ", er255, wire(256));
    print_cost("reg 1 + 1    ", ereg1, wire(4));
    print_cost("reg 1 + 4    ", ereg, wire(7));
    print(serial, "  the engines against the pump at 400 kHz, core cycles:", crlf);
    engine_arithmetic("write", w16, w255, e1, e16);
    engine_arithmetic("read ", r16, r255, er1, er16);
    const uint32_t pump_body = w255.entries != 0u ? w255.body / w255.entries : 0u;
    print(serial, "  the pump's entry a byte: ", pump_body, " cycles on average over the 255-byte write, ",
          r16.entries != 0u ? r16.body / r16.entries : 0u, " over the 16-byte read; the engined 255-byte "
          "write at ", e255.wall * 1000u / wire(256), " thousandths of its wire, the core in the vector ",
          e255.body, " cycles of ", e255.wall, crlf);
    print(serial, "  the longest event entry of every shape above: ", longest_entry, " cycles, stamps included",
          crlf);
    const uint32_t longest_flash = longest_entry;
    // THE SAME PUMP IN SRAM: the plain host's twin, its event entry placed
    // (util/placement.hpp's InRam), every pump shape again.
    Host::release();
    owner = Owner::ram_host;
    (void)RamHost::init(clock);
    longest_entry = 0;
    (void)cost_of<RamHost>(1, 0);
    const Cost m1 = cost_of<RamHost>(1, 0);
    const Cost m16 = cost_of<RamHost>(16, 0);
    const Cost m255 = cost_of<RamHost>(255, 0);
    const Cost mr1 = cost_of<RamHost>(0, 1);
    const Cost mr2 = cost_of<RamHost>(0, 2);
    const Cost mr16 = cost_of<RamHost>(0, 16);
    const Cost mreg4 = cost_of<RamHost>(1, 4);
    const Cost mprobe = cost_of<RamHost>(0, 0);
    RamHost::release();
    host_ready();
    print(serial, "  the pump with its entry in SRAM:", crlf);
    print_cost("write 1      ", m1, wire(2));
    print_cost("write 16     ", m16, wire(17));
    print_cost("write 255    ", m255, wire(256));
    print_cost("read 1       ", mr1, wire(2));
    print_cost("read 2       ", mr2, wire(3));
    print_cost("read 16      ", mr16, wire(17));
    print_cost("reg 1 + 4    ", mreg4, wire(7));
    print_cost("probe        ", mprobe, wire(1));
    const uint32_t ram_body = m255.entries != 0u ? m255.body / m255.entries : 0u;
    const uint32_t ram_read = mr16.entries != 0u ? mr16.body / mr16.entries : 0u;
    print(serial, "  the pump's entry a byte, flash | SRAM: ", pump_body, " | ", ram_body,
          " cycles over the 255-byte write, ", r16.entries != 0u ? r16.body / r16.entries : 0u, " | ", ram_read,
          " over the 16-byte read; the longest entry ", longest_flash, " | ", longest_entry, crlf);
    const bool ram_ok = m1.status == i2c_ok && m16.status == i2c_ok && m255.status == i2c_ok &&
                        mr1.status == i2c_ok && mr2.status == i2c_ok && mr16.status == i2c_ok &&
                        mreg4.status == i2c_ok && mprobe.status == i2c_ok;
    bench.verdict("the pump in SRAM: every shape i2c_ok, the same entries as from the flash",
                  ram_ok && m16.entries == w16.entries && m255.entries == w255.entries &&
                      mr16.entries == r16.entries);
    // From SRAM behind this suite's vector, which stays in the flash: the
    // vector's own path (the storm budget, the owner, the stamps) is the
    // flash's, the trampoline's body SRAM's. How much shorter the entry runs
    // there is the fetch's - the flash's wait states against SRAM's, behind
    // peripheral loads the placement does not touch - which no written figure
    // bounds: the two are printed above, side by side, and not judged.
    longest_entry = longest_flash;
    print(serial, "  unwedges so far (15.9.1's stuck BUSY taken out by start()): plain ", Host::unwedges(),
          ", engined ", DmaHost::unwedges(), crlf);
    // THE SAME ARITHMETIC AT 100 kHz, inside the same serve: its client
    // answers at either speed, and its command address only after the
    // window.
    bool slow_ok = true;
    constexpr I2cSpeed slow = I2cSpeed::standard_100k;
    owner = Owner::dma_host;
    (void)DmaHost::init(clock);
    (void)cost_of<DmaHost>(1, 0, dma, slow);   // the speed applied
    const Cost se1 = cost_of<DmaHost>(1, 0, dma, slow);
    const Cost se16 = cost_of<DmaHost>(16, 0, dma, slow);
    const Cost ser1 = cost_of<DmaHost>(0, 1, dma, slow);
    const Cost ser16 = cost_of<DmaHost>(0, 16, dma, slow);
    DmaHost::release();
    host_ready();
    (void)cost_of<Host>(1, 0, I2cEngine::interrupt, slow);
    const Cost sw16 = cost_of<Host>(16, 0, I2cEngine::interrupt, slow);
    const Cost sw255 = cost_of<Host>(255, 0, I2cEngine::interrupt, slow);
    const Cost sr16 = cost_of<Host>(0, 16, I2cEngine::interrupt, slow);
    const Cost sr255 = cost_of<Host>(0, 255, I2cEngine::interrupt, slow);
    for (const Cost* c : {&se1, &se16, &ser1, &ser16, &sw16, &sw255, &sr16, &sr255}) {
        slow_ok = slow_ok && c->status == i2c_ok;
    }
    print(serial, "  the engines against the pump at 100 kHz, core cycles:", crlf);
    engine_arithmetic("write", sw16, sw255, se1, se16);
    engine_arithmetic("read ", sr16, sr255, ser1, ser16);
    const bool all_ok = w1.status == i2c_ok && w2.status == i2c_ok && w16.status == i2c_ok &&
                        w255.status == i2c_ok && r1.status == i2c_ok && r2.status == i2c_ok &&
                        r16.status == i2c_ok && r255.status == i2c_ok && reg1.status == i2c_ok &&
                        reg4.status == i2c_ok && probe.status == i2c_ok && e1.status == i2c_ok &&
                        e2.status == i2c_ok && e16.status == i2c_ok && e255.status == i2c_ok &&
                        er1.status == i2c_ok && er2.status == i2c_ok && er16.status == i2c_ok &&
                        er255.status == i2c_ok && ereg1.status == i2c_ok && ereg.status == i2c_ok && slow_ok;
    bench.verdict("every shape completed i2c_ok, at both speeds", all_ok);
    bench.verdict("the period is the rate asked: within 2 % above and a quarter below 400 kHz",
                  period != 0u && scl_bracket(SysClock::hz / period, 400'000u));
    bench.verdict("a one- and a two-byte write take THREE event entries (SB, ADDR loading both bytes, "
                  "BTF), and a register read 1 + 1 takes no TxE entry",
                  w1.entries == 3u && w2.entries == 3u && reg1.entries == 5u);
    bench.verdict("the pump is one entry a byte past the first two: a write of N takes N + 1",
                  w16.entries == 17u && w255.entries == 256u);
    bench.verdict("the engines: a write of any length three event entries (SB, ADDR, BTF) and one DMA "
                  "interrupt, its completion handing the half back; a read of any length two and one",
                  e1.entries == 3u && e16.entries == 3u && e255.entries == 3u && e1.dma == 1u && e16.dma == 1u &&
                      e255.dma == 1u && er1.entries == 2u && er16.entries == 2u && er255.entries == 2u &&
                      er1.dma == 1u && er16.dma == 1u && er255.dma == 1u);
    bench.verdict("no vector re-entered after a completion edge (the event line is the tenure's)",
                  w1.after == 0u && w16.after == 0u && e255.after == 0u && r16.after == 0u);
    bench.verdict("the engined 255-byte write runs at the wire's own time (x 1.00 within 2 %)",
                  e255.wall <= wire(256) + wire(256) / 50u);
    // The serve's window waited out, so the next letter finds the peer's
    // command address answering.
    while (Ticker::millis() - served_at < 4300u) {
    }
}

void banner() {
    print(serial, crlf, "test_x035_i2c on ", device::part_name, " - I2C1 (RM ch. 15) against the CH32V006's twi_peer",
          crlf);
    print(serial, "  PC16 SCL / PC17 SDA, 1.5 kOhm to PB3 (the rail, ", supply_ok ? "driven high" : "FLOATING",
          "); the wire reads ", wire_pulled_up() ? "high" : "LOW", crlf);
    print(serial, "  supply at boot: ", supply_ok ? "above 3.0 V" : "UNDER 3.0 V, every letter refused", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_VECTOR(usart2_handler) { (void)Serial::isr(); }

/// The completion to the arbiter that owns the host (letter t has two).
[[gnu::always_inline]] inline void post_done() {
    if constexpr (brio::selection_carries(brio::test_letters, 't')) {
        if (bus_ao_untimed) {
            brio::post<kl::I2cArbUntimed>(brio::TransferDone{Host::status()});
            return;
        }
    }
    brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
}

BRIO_CH32_VECTOR(i2c1_ev_handler) {
    if (client_live) {
        const uint32_t c0 = brio::stk()->CNTL;
        ct.entries = ct.entries + 1u;
        if (ct.entries > client_entry_budget) {
            H::event_interrupt(false);
            H::buffer_interrupt(false);
            return;
        }
        client_event();
        const uint32_t c1 = brio::stk()->CNTL;
        const uint32_t d = c1 >= c0 ? c1 - c0 : c1 + brio::stk()->CMPLR + 1u - c0;
        ct.cycles = ct.cycles + d;
        if (d > ct.cycles_max) {
            ct.cycles_max = d;
        }
        return;
    }
    if (live_client) {
        // Letter w's client half, for the peer's long tenure: ADDR cleared, a byte
        // taken. The host's own body runs behind it: only its SB is for it.
        const uint16_t s1 = H::status1();
        if ((s1 & i2c_addr) != 0u) {
            (void)H::status2();
        }
        if ((s1 & i2c_rxne) != 0u) {
            const uint8_t v = H::data();
            live_count = live_count + 1u;
            live_sum = live_sum + v;
        }
    }
    const uint32_t t0 = brio::stk()->CNTL;
    host_isr_entries = host_isr_entries + 1u;
    if (host_isr_entries > host_isr_budget) {
        host_stormed = true;
        storms = storms + 1u;
        H::event_interrupt(false);
        H::buffer_interrupt(false);
        return;
    }
    if (owner == Owner::dma_host) {
        if (DmaHost::isr()) {
            host_done = true;
        }
    } else if (owner == Owner::ram_host) {
        if (RamHost::isr()) {
            host_done = true;
        }
    } else if (owner == Owner::bus_ao) {
        if (Host::isr()) {
            post_done();
        }
    } else if (Host::isr()) {
        host_done = true;
    }
    const uint32_t t1 = brio::stk()->CNTL;
    const uint32_t d = t1 >= t0 ? t1 - t0 : t1 + brio::stk()->CMPLR + 1u - t0;
    ev_cycles = ev_cycles + d;
    if (d > ev_max) {
        ev_max = d;
    }
}

BRIO_CH32_VECTOR(i2c1_er_handler) {
    if (client_live) {
        ct.err_entries = ct.err_entries + 1u;
        client_error();
        return;
    }
    error_isr_entries = error_isr_entries + 1u;
    if (owner == Owner::dma_host) {
        if (DmaHost::error_isr()) {
            host_done = true;
        }
    } else if (owner == Owner::ram_host) {
        if (RamHost::error_isr()) {
            host_done = true;
        }
    } else if (owner == Owner::bus_ao) {
        if (Host::error_isr()) {
            post_done();
        }
    } else if (Host::error_isr()) {
        host_done = true;
    }
}

/// Both channels' vectors: the host's body timed on the STK, as the event
/// vector's is.
[[gnu::always_inline]] inline void dma_vector_body() {
    const uint32_t t0 = brio::stk()->CNTL;
    if (owner == Owner::dma_host && DmaHost::dma_isr()) {
        host_done = true;
    }
    const uint32_t t1 = brio::stk()->CNTL;
    dma_cycles = dma_cycles + (t1 >= t0 ? t1 - t0 : t1 + brio::stk()->CMPLR + 1u - t0);
}
BRIO_CH32_VECTOR(dma1_channel6_handler) {
    dma6_entries = dma6_entries + 1u;
    dma_vector_body();
}
BRIO_CH32_VECTOR(dma1_channel7_handler) {
    dma7_entries = dma7_entries + 1u;
    dma_vector_body();
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // THE SUPPLY before any link pad is driven (the desk's rule), and the
    // rail only then.
    brio::Pwr::pvd(brio::PvdLevel::v3_0);
    (void)brio::delay_us(clock, 200);
    supply_ok = !brio::Pwr::supply_low();
    if (supply_ok) {
        rail_on();
    }

    bench.letter('a', "THE BLOCK: the lot probe, the registers, no RTR, the SMBus bits, the rise, "
                      "the USB pads' pulls", ta_block);
    bench.letter('b', "THE SCAN: 112 addresses probed, nobody-home as i2c_nack_addr", tb_scan);
    bench.letter('c', "THE PEER: the twi_link command channel, ident, ten round trips", tc_peer_link);
    bench.letter('d', "the tenure shapes, the four receive procedures, the reserved addresses refused",
                 td_shapes);
    bench.letter('e', "the vocabulary on the wire, and commanded stretching priced", te_vocabulary);
    bench.letter('f', "the two speeds against a second chip, timed; a rung past the chapter", tf_speeds);
    bench.letter('g', "THE DMA ENGINES on channels 6 and 7 against the peer", tg_dma);
    bench.letter('n', "THE ENGINE A REQUEST NAMES: dma at one byte, interrupt touching no channel, dma "
                      "refused without engines, the edges under dma",
                 tn_engine);
    bench.letter('h', "THE KERNEL: I2cBus over I2cHost, the rejection, the votes, a held SDA and a held "
                      "clock answered in their place", th_kernel);
    bench.letter('i', "THE REFUSAL, wireless: i2c_rejected inside start() through the arbiter", ti_refusal);
    bench.letter('j', "ARBITRATION both ways, and this board as the peer's CLIENT", tj_arbitration);
    bench.letter('k', "the stuck bus: the peer holding SDA, unstick() counting", tk_unstick);
    bench.letter('p', "THE DEAD CHANNEL: a held request under the engines' write and read, and release() over "
                      "pulse", tp_dead_channel);
    bench.letter('y', "THE ENGINES' CHANNELS STARVED: a memory copy above them, the entries counted", ty_starved);
    bench.letter('q', "the siblings' quirks re-measured on this die", tq_quirks);
    bench.letter('u', "a BUSY left standing over an idle wire, taken out by start()", tu_unwedge);
    bench.letter('v', "THIS BOARD AS THE CLIENT FROM ITS VECTORS: written to and read from by the peer's "
                      "controller", tv_client_vectors);
    bench.letter('r', "THE REPEATED START on the last byte's TxE, acknowledged and refused", tr_restart);
    bench.letter('t', "THE TWO BOUNDS OF start() against their names, and with the STK stopped", tt_bounds);
    bench.letter('w', "A REQUEST INTO A TENURE ANOTHER CONTROLLER IS CLOCKING: the resets taken, what each side saw",
                 tw_live_tenure);
    bench.letter('x', "THE COST: every shape on the pump and the engines at 400 kHz", tx_cost);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=",
                    tick_ok ? "STK" : "FAILED", " supply=", supply_ok ? "above 3.0 V" : "LOW", brio::crlf);
        if (!supply_ok) {
            brio::print(serial, "SUPPLY UNDER 3.0 V (the PVD at PLS 10): every letter refused, PB3 and the "
                                "bus pads left floating", brio::crlf);
        }
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
        brio::print(serial, static_cast<char>(c), brio::crlf);
        if (c == '?') {
            banner();
        } else if (!supply_ok) {
            brio::print(serial, "refused: the supply is under 3.0 V", brio::crlf);
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
