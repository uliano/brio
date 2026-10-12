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
// CH32V006K8U6, which runs `twi_peer` on its column 3:
//
//   SCL   PC16  <->  the CH32V006's PB5
//   SDA   PC17  <->  the CH32V006's PB6
//   a 1.5 kOhm resistor from each line to this board's PB3, and GND
//
// THE PULL-UPS HANG FROM PB3, NEVER FROM VCC: PC17 is this board's boot
// pin - a high level on it at power-up enters WCH's loader (the board's
// reference, section 5) - so the bus is pulled up only while an image
// that uses it drives PB3 high, and PB3 floats from every reset. This
// image drives it high, push-pull, from its boot to its end, the supply
// checked first; letter a lets it float for the one measurement that
// asks what the USB pads' own pulls do. 1.5 kOhm at 3.3 V sinks 2.2 mA a
// line, and PB3 sources at most about 4.4 mA with both lines held low.
//
// THE PEER is commanded IN BAND over the bus under test through
// avrdx/src/apps/twi_link.hpp, included by relative path: one source of
// truth for the wire format, whatever the peer's architecture. Its
// command-mode client answers ONE address (0x6B) with no general call and
// no second address.
//
// THE HOST RUNS ON ITS TWO VECTORS (events on i2c1_ev, errors on i2c1_er);
// a letter that wants a tenure waited out spins on a flag the handler
// sets. Two I2cHost instantiations share the one instance - the plain one
// and the one on DMA channels 6 and 7 - and a flag says which owns the
// vectors, since two tasks over one peripheral share its registers and
// not their statics. The event vector's body is timed on the STK at every
// entry (letter x reads the sums).
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
//      the general call
//   e  the vocabulary against the peer (nack_addr from a deaf client,
//      nack_data at a commanded byte) and commanded stretching priced
//   f  the two speeds against a second chip, byte-exact both ways, each
//      write timed and its SCL judged against the rate asked - and a rung
//      at 1 MHz past the chapter, measured and not judged
//   g  THE DMA ENGINES on channels 6 and 7: the same shapes, a read at
//      400 kHz besides, the interrupts they take, their SCL timed
//   h  THE KERNEL against the peer: I2cBus over I2cHost, the NACK in its
//      place, the rejection, both votes, a held SDA and a held clock
//      answered in their place, the recover()ed engine after
//   i  THE REFUSAL, wireless: a speed the clock cannot make is answered
//      i2c_rejected inside start() and delivered through the arbiter
//   j  ARBITRATION in both directions, and THIS BOARD AS THE CLIENT of the
//      peer's own controller - written to, its stretch one byte late
//      measured, and read from, the byte a transmitter is asked beyond the
//      closing NACK dropped by flush()
//   k  THE STUCK BUS: the peer holding SDA, unstick() counting the clocks
//   q  THE SIBLINGS' QUIRKS RE-MEASURED on this die: a START into a held
//      SDA, raw; how long the host's own STOP stands in CTLR1; a CTLR1
//      store under it; STOPF on a STOP the client half was not addressed in
//   r  THE REPEATED START of a write-then-read, requested on the TxE of the
//      last written byte: one to four written then four read, both speeds,
//      pump and engines - then the last byte refused
//   x  THE COST: every tenure shape on the pump and on the engines at 400
//      kHz, its wall time against the wire's, its vector entries and the
//      cycles inside them, start()'s own
//
// THE SUPPLY at boot: the voltage detector at 3.0 V (pwr.hpp); an image
// that drives a link pad from a sagging rail leaves PB3 floating, refuses
// every letter and says so.
//
// With the peer attached `z` outlasts `brio run`'s default 60 s (the
// peer's command windows are hundreds of milliseconds each): pass
// `--timeout 600`.
//
// build: boards = x035f8
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
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED (see the header comment).
#include "../../../avrdx/src/apps/twi_link.hpp"

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
enum class Owner : uint8_t { host, dma_host, bus_ao };
volatile Owner owner = Owner::host;

volatile bool host_done = false;
volatile uint32_t dma6_entries = 0;   // the transmit channel's vector
volatile uint32_t dma7_entries = 0;   // the receive channel's
volatile uint32_t host_isr_entries = 0;
volatile uint32_t error_isr_entries = 0;
/// The event vector's body on the STK, summed and its longest.
volatile uint32_t ev_cycles = 0;
volatile uint32_t ev_max = 0;
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

uint32_t cycles_now() { return stk()->CNTL; }
uint32_t cycles_between(uint32_t from, uint32_t to) {
    const uint32_t period = stk()->CMPLR + 1u;
    return to >= from ? to - from : to + period - from;
}

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

void spin_us(uint32_t us) {
    while (us >= 500u) {
        (void)delay_us(clock, 500u);
        us -= 500u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 200);
}

/// THE RAIL: PB3 driven high, push-pull - the bus's pull-ups (the header).
void rail_on() { Rail::output(true); }
void rail_off() { Rail::release(); }

void clear_counts() {
    host_isr_entries = 0;
    error_isr_entries = 0;
    ev_cycles = 0;
    ev_max = 0;
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

/// Wait for the vector's completion edge, bounded in time.
bool wait_done(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (!host_done && Ticker::millis() - t0 < ms) {
    }
    return host_done;
}

/// One tenure through the TASK, waited out. Returns the status.
uint8_t host_tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
                    I2cSpeed speed) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    host_done = false;
    clear_counts();
    if (Host::start(r)) {
        return Host::status();
    }
    if (!wait_done(100)) {
        print(serial, "    STALL: STAR1=", hex(H::status1()), " STAR2=", hex(H::status2()),
              " CTLR1=", hex(H::regs().CTLR1), " CTLR2=", hex(H::regs().CTLR2), " ev entries ",
              host_isr_entries, " er ", error_isr_entries, host_stormed ? " STORMED" : "", crlf);
        (void)Host::recover();
        return no_answer;
    }
    return Host::status();
}

/// The same through the DMA host.
uint8_t dma_tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
                   I2cSpeed speed) {
    DmaHost::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    host_done = false;
    clear_counts();
    if (DmaHost::start(r)) {
        return DmaHost::status();
    }
    if (!wait_done(100)) {
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
/// never rose in the budget.
struct Rise {
    uint32_t cycles;
    uint32_t polls;
};

template <class Pd>
Rise rise_of() {
    Pd::output(false);
    (void)delay_us(clock, 20);
    (void)Pd::input();
    const uint32_t t0 = cycles_now();
    uint32_t n = 0;
    while (!Pd::read() && n < 0xFFFFu) {
        ++n;
    }
    const uint32_t t1 = cycles_now();
    return Rise{n >= 0xFFFFu ? 0xFFFFu : cycles_between(t0, t1), n};
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

bool ensure_link() {
    link_quiet = true;
    link_ready();
    for (uint8_t k = 0; k < 3; ++k) {
        if (command(Op::ping)) {
            link_quiet = false;
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

/// Every wire letter asks first: no pull-ups is a skip with the reason,
/// a silent peer a failure.
bool need_peer() {
    if (!wire_pulled_up()) {
        print(serial, "  SKIPPED, no verdict claimed: SCL/SDA read low with the rail on - the "
                      "resistors to PB3 are not on the wire.",
              crlf);
        return false;
    }
    if (ensure_link()) {
        return true;
    }
    bench.verdict("the peer answers on the command address (the CH32V006 running twi_peer)",
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
    bench.verdict("the enable protection was measured and reported (either answer is a finding)", true);
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
    bench.verdict("with the rail on, each line rises from the pad in well under a 400 kHz bit "
                  "(the 1.5 kOhm resistors to PB3)",
                  scl.cycles < 60u && sda.cycles < 60u);

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
    print(serial, "  -> the USB pads' pull-ups ", act ? "ACT" : "do NOT act", " under the I2C's alternate function",
          crlf);
    bench.verdict("the control: the 1.5 kOhm and 10 kOhm modes pull both pads up as PULLED INPUTS",
                  in.scl && in.sda && in10.scl && in10.sda);
    bench.verdict("with no pull mode the released pair stays low (nothing else holds the bus up with "
                  "the rail off)",
                  !none_af.scl && !none_af.sda);
    bench.verdict("THE USB PADS' PULLS UNDER THE ALTERNATE FUNCTION were measured, every mode and both "
                  "output-data levels (either answer is a finding)",
                  true);
    print(serial, "  the rail back on: the wire reads ", wire_pulled_up() ? "high" : "LOW", crlf);
}

// ===========================================================================
// b - the scan
// ===========================================================================

void tb_scan() {
    if (!wire_pulled_up()) {
        print(serial, "  SKIPPED, no verdict claimed: no pull-ups on the wire.", crlf);
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
        print(serial, "  (the peer does not serve the counters query: the drops are not judged)", crlf);
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

    twilink::Params g = serve_params(64, 250, 0);
    g.flags = twilink::flag_general_call;
    if (peer_act(Op::serve, g)) {
        uint8_t gw[2] = {0x5A, 0xA5};
        const uint8_t gs = host_tenure(0x00, gw, 2, nullptr, 0, link_speed);
        settle_ms(300);
        twilink::Report gr{};
        const bool grep = peer_report(gr);
        print(serial, "  general call: status=", gs, " peer last_addr=", hex(gr.last_addr), " count=",
              gr.count, crlf);
        bench.verdict("a GENERAL CALL write reaches the peer at address 0x00",
                      gs == i2c_ok && grep && gr.count == 2u && gr.last_addr == 0x00u);
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
    const uint32_t hold = 2u * (SysClock::hz / 1000u);
    const uint32_t holds = (stretched + hold / 2u) / hold;
    print(serial, "  8 x 8 bytes unstretched: ", base_ms, " ms; ONE 8-byte tenure at 2 ms per byte: ",
          stretched / (SysClock::hz / 1000u), " ms = ", holds, " holds of 2 ms on the wire; peer count=",
          sr.count, crlf);
    bench.verdict("the unstretched baseline tenures all complete i2c_ok", base_ok);
    bench.verdict("a client stretching every data byte by 2 ms stretches the WALL TIME by the seven holds "
                  "a target one byte late puts on the wire, and the tenure still completes i2c_ok - "
                  "stretching is flow control",
                  s2 == i2c_ok && holds == 7u && srep && sr.count >= 8u);
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
        console_drain();
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
                  bracketed == 2u);

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
        console_drain();
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
              hex(static_cast<uint16_t>(0x90u * 8u + 28u)), " sent) - measured, not judged", crlf);
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
    console_drain();
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
    console_drain();
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
          ") mism=", mism, " combined=", cs, " read2=", r2, " read1(pump)=", r1, " read16 at 400k=", fs,
          " mism=", fmism, crlf);
    print(serial, "  peer count=", r.count, " addr_hits=", r.addr_hits, "; faults this run tx=", tx_faults,
          " rx=", rx_faults, "; DMA interrupts ch6=", e6, " ch7=", e7, crlf);
    print(serial, "  SCL on the engines: the 16-byte write ", w_scl / 1000u, " kHz and read ", r_scl / 1000u,
          " kHz (100 asked), the read at 400k ", f_scl / 1000u, " kHz", crlf);
    bench.verdict("a 16-byte write and a 16-byte read through the DMA engines complete i2c_ok, "
                  "byte-exact",
                  ws == i2c_ok && rs == i2c_ok && mism == 0u);
    bench.verdict("the combined tenure, the two-byte read (LAST's NACK) and the one-byte read (the "
                  "pump's) all complete",
                  cs == i2c_ok && r2 == i2c_ok && r1 == i2c_ok);
    bench.verdict("a 16-byte read at 400 kHz on the engines completes i2c_ok, the serve's next sixteen "
                  "bytes exact",
                  fs == i2c_ok && fmism == 0u);
    bench.verdict("the peer accounts every byte (16 + 16 + 4 + 4 + 2 + 1 + 16 = 59), and this run counts "
                  "no transfer fault",
                  rep && r.count == 59u && tx_faults == 0u && rx_faults == 0u);
    bench.verdict("no write block interrupts (BTF ends a write: SB, ADDR and BTF, three event entries), "
                  "each of the four engined reads once (SB and ADDR on the event vector)",
                  e6 == 0u && e7 == 4u && w_entries == 3u && r_entries == 2u);
    bench.verdict("and the engines run the bus at the rate asked: SCL never above 100 or 400 kHz by more "
                  "than 2 % and within a quarter below it",
                  ws == i2c_ok && rs == i2c_ok && fs == i2c_ok && scl_bracket(w_scl, 100'000u) &&
                      scl_bracket(r_scl, 100'000u) && scl_bracket(f_scl, 400'000u));
}

// ===========================================================================
// h - the kernel against the peer
// ===========================================================================

namespace kl {

constexpr uint32_t bus_timeout_ticks = ticks_from_ms<P>(20);
using I2cArb = I2cBus<Host, P, 4, BusPassThrough, bus_timeout_ticks>;

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

using BusKernel = Tenuto<P, Probe, I2cArb>;

void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

void pump_until(uint8_t want, uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        pump();
        if (Probe::n >= want) {
            break;
        }
    }
    pump();
}

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
    bench.verdict("a tenure into a wire a FOREIGN CHIP holds down is answered IN ITS PLACE - never "
                  "silence and never i2c_ok - at the arbiter's own limit or sooner",
                  kl::Probe::n == 1u && kl::Probe::replies[0] != i2c_ok && took <= 40u);

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
    bench.verdict("a client holding the CLOCK past the limit is answered i2c_timeout ON THE ARBITER'S "
                  "CLOCK - the one wedge the silicon cannot answer by itself",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_timeout && took_clock >= 20u &&
                      took_clock <= 30u);
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
    spin_us(us);
    inject_stop();
}

/// ONE TENURE STARTED INTO A BUSY BUS: a START condition by hand, the
/// peer given time to see BUSY, this host's START set, then the STOP that
/// frees the bus - both STARTs leave on it and the wired-AND decides.
uint8_t arb_tenure(uint8_t addr, const uint8_t* tx, uint8_t len, uint32_t lead_us) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = len;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    r.rx_len = 0;
    r.speed = link_speed;
    host_done = false;
    clear_counts();
    inject_start();
    spin_us(lead_us);
    const bool sync = Host::start(r);
    inject_stop();
    if (sync) {
        return Host::status();
    }
    if (!wait_done(200)) {
        print(serial, "    the race never answered: STAR1=", hex(H::status1()), " STAR2=", hex(H::status2()),
              " CTLR1=", hex(H::regs().CTLR1), crlf);
        (void)Host::recover();
        return no_answer;
    }
    return Host::status();
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
    for (uint8_t i = 0; i < 2u; ++i) {
        twilink::Params a{};
        a.ms = 600;
        a.addr = twilink::dut_addr;
        a.target = rounds[i].peer_target;
        a.count = 4;
        a.seed = 0x40;
        a.aux16 = 120;
        if (!peer_act(Op::arb, a)) {
            bench.verdict("the peer accepted the arbitration command", false);
            return;
        }
        host_ready();
        for (uint8_t k = 0; k < 5; ++k) {
            tx_buf[k] = static_cast<uint8_t>(0xD0u + k);
        }
        const uint8_t st = arb_tenure(rounds[i].dut_target, tx_buf, 5, 350);
        settle_ms(700);
        twilink::Report r{};
        const bool rep = peer_report(r);
        const bool dut_lost = st == i2c_arb_lost;
        const bool peer_lost = rep && (r.flags & twilink::report_arblost) != 0u;
        print(serial, "  ", rounds[i].name, crlf, "    this board: status=", st, "; peer: flags=",
              hex(r.flags), " mstatus=", r.mstatus, crlf);
        if (rounds[i].dut_should_lose ? dut_lost : (!dut_lost && peer_lost)) {
            ++as_expected;
        }
        if (rounds[i].dut_should_lose) {
            bench.verdict("aiming at the HIGHER address, this controller loses the wired-AND and reports "
                          "i2c_arb_lost",
                          dut_lost);
        } else {
            bench.verdict("aiming at the LOWER address, this controller keeps the bus (nobody answers 0x11: "
                          "nack_addr) and it is the PEER that reports the loss",
                          !dut_lost && st == i2c_nack_addr && peer_lost);
        }
    }
    bench.verdict("arbitration was observed in BOTH directions", as_expected == 2u);

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
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 900u) {
        const I2cClientEvent e = Client::service();
        if (e == I2cClientEvent::addressed) {
            ++hits;
            wrote_to_us = !Client::host_reads();
        } else if (e == I2cClientEvent::byte_received) {
            if (!late_seen) {
                // THE STRETCH, ONE BYTE LATE: the first byte left standing in
                // DATAR for two milliseconds - what has the shifter done?
                spin_us(2000);
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
                break;
            }
        } else if (Client::host_nacked()) {
            Client::clear_nack();
        }
    }
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
    const uint32_t t1 = Ticker::millis();
    // ONE STAR1 SNAPSHOT A PASS, every decision taken on it: the closing
    // NACK is a LEVEL that may rise between two separate reads of the
    // register, and a pass that cleared the error flags on a second read
    // would swallow it uncounted.
    while (Ticker::millis() - t1 < 1000u) {
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
                break;
            }
        } else if ((s1 & i2c_errors) != 0u) {
            H::clear_errors(static_cast<uint16_t>(s1 & i2c_errors));
        }
    }
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
    bench.verdict("unstick() clocked until the client released - the peer's four falling edges "
                  "asked, a handful of pulses, never 0xFF",
                  pulses >= 1u && pulses <= 9u);
    settle_ms(450);
    print(serial, "  the event vector, idle, after the hand-made START and STOP: ", host_isr_entries,
          " entries, STAR1=", hex(H::status1()), crlf);
    bench.verdict("the STOPF the hand-made STOP may leave is cleared - no storm on the event vector",
                  host_isr_entries <= 2u && storms == storms_before && (H::status1() & i2c_stopf) == 0u);
    bench.verdict("with the wire free again unstick() clocks nothing", Host::unstick() == 0u);
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
            const uint32_t a0 = Ticker::cycles();
            while (Ticker::cycles() - a0 < 48u * 2000u) {
                if (H::flag(i2c_arlo)) {
                    w.addr_arlo_at = (Ticker::cycles() - a0) / 48u + 1u;
                    break;
                }
            }
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
    bench.verdict("a START into a wire a foreign chip holds, from a block that saw the line taken, is "
                  "NOT refused: SB rises within a fraction of a millisecond, and the address's first one "
                  "bit loses arbitration - the engine's i2c_arb_lost of letter h",
                  seen.held && seen.arlo_at == 0u && seen.sb_at != 0u && seen.sb_at < 500u &&
                      seen.addr_arlo_at != 0u && seen.addr_arlo_at < 500u);
    bench.verdict("from a block brought up over the held line the START stays PENDING - no SB, no ARLO "
                  "in 50 ms, BUSY standing: the per-bus timeout's case",
                  over.held && over.sb_at == 0u && over.arlo_at == 0u && (over.c1 & i2c_start) != 0u &&
                      (over.s2 & i2c_busy) != 0u);

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
        (void)Host::start(r);
        while (!host_done) {
        }
        const uint32_t e0 = Ticker::cycles();
        while (H::stopping() && Ticker::cycles() - e0 < 48'000u) {
        }
        stand[i] = Ticker::cycles() - e0;
        sts[i] = Host::status();
    }
    print(serial, "  2. the host's STOP stands in CTLR1 for ", stand[0], " cycles at 100 kHz and ", stand[1],
          " at 400 kHz after the completion edge (a bit is 480 and 120); status ", sts[0], "/", sts[1], crlf);
    bench.verdict("the host's own STOP stands in CTLR1 after the completion edge, for about one bit - "
                  "what start()'s bounded wait spends",
                  sts[0] == i2c_ok && sts[1] == i2c_ok && stand[0] > 0u && stand[0] < 4800u &&
                      stand[1] < 2400u);

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
        (void)Host::start(r);
        while (!host_done) {
        }
        const uint16_t at = H::regs().CTLR1;
        H::regs().CTLR1 = static_cast<uint16_t>(H::regs().CTLR1 | i2c_ack);   // a read-modify-write
        settle_ms(1);
        const uint16_t c1_later = H::regs().CTLR1;
        const uint16_t s2_later = H::status2();
        H::ack(false);
        const uint8_t next = host_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, I2cSpeed::standard_100k);
        print(serial, "  3. a CTLR1 read-modify-write with STOP standing (CTLR1 ", hex(at), "): a ms later CTLR1 ",
              hex(c1_later), " STAR2 ", hex(s2_later), ", the next tenure ", next, crlf);
        bench.verdict("a CTLR1 store under the host's own STOP was measured and reported (either answer "
                      "is a finding), and the next tenure ran",
                      next == i2c_ok);
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
            put_down(dma);
            settle_ms(350);
            twilink::Report r{};
            const bool rep = peer_report(r);
            const bool good = rep && sts[0] == i2c_ok && sts[1] == i2c_ok && sts[2] == i2c_ok &&
                              sts[3] == i2c_ok && mism == 0u && r.aux0 == 10u && r.sum == sum &&
                              r.addr_hits == 8u;
            ack_ok = ack_ok && good;
            print(serial, "  ", i2c_speed_hz(speed) / 1000u, " kHz ", dma ? "engines" : "pump   ",
                  " acknowledged: status ", sts[0], "/", sts[1], "/", sts[2], "/", sts[3], ", event entries ",
                  ev[0], "/", ev[1], "/", ev[2], "/", ev[3], ", read mism ", mism, "; peer took ", r.aux0,
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
    bench.verdict("one to four bytes written then four read, both speeds, pump and engines: every "
                  "written byte taken by the peer - a POLLED CH32 target - every read byte its pattern",
                  ack_ok);
    bench.verdict("the last written byte refused: i2c_nack_data, the peer's own NACK", refuse_status);
    bench.verdict("and the controller lets the bus go: no START or STOP left standing, MSL clear",
                  refuse_clean);
    bench.verdict("the next tenure on the same host runs to a wire outcome - i2c_ok, or the address "
                  "refused by a peer still deaf from its refusal - never a stall", refuse_next);
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
    uint32_t after;     ///< event vector entries in the millisecond after the edge
};

template <typename Hst>
Cost cost_of(uint8_t tx_len, uint8_t rx_len) {
    typename Hst::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.rx_len = rx_len;
    r.speed = I2cSpeed::fast_400k;
    console_drain();
    host_done = false;
    clear_counts();
    dma6_entries = 0;
    dma7_entries = 0;
    Cost c{};
    const uint32_t c0 = Ticker::cycles();
    const bool sync = Hst::start(r);
    c.start = Ticker::cycles() - c0;
    if (!sync) {
        (void)wait_done(100);
    }
    c.wall = Ticker::cycles() - c0;
    c.status = host_done || sync ? Hst::status() : no_answer;
    c.entries = host_isr_entries;
    c.body = ev_cycles;
    c.longest = ev_max;
    c.dma = dma6_entries + dma7_entries;
    (void)delay_us(clock, 1000);
    c.after = host_isr_entries - c.entries;
    return c;
}

/// The longest event vector entry over the shapes letter x printed: the
/// figure the CH32X035 SPI host's default hold-off is drawn from
/// (brio/ch32x035/spi.hpp, spi_default_hold_off_cycles).
uint32_t longest_entry = 0;

void print_cost(const char* name, const Cost& c, uint32_t wire) {
    const uint32_t rest = c.wall > wire ? c.wall - wire : 0u;
    print(serial, "    ", name, ": status ", c.status, ", wall ", c.wall, " = wire ", wire, " + rest ", rest,
          "; start() ", c.start, "; event entries ", c.entries, " (", c.entries != 0u ? c.body / c.entries : 0u,
          " cycles each, ", c.longest, " at most), DMA ", c.dma, ", after the edge ", c.after, crlf);
    if (c.longest > longest_entry) {
        longest_entry = c.longest;
    }
}

void tx_cost() {
    if (!need_peer()) {
        return;
    }
    if (!peer_act(Op::serve, serve_params(0, 2500, 0x10))) {
        bench.verdict("the peer accepted the serve", false);
        return;
    }
    for (uint16_t i = 0; i < 255u; ++i) {
        tx_buf[i] = static_cast<uint8_t>(i);
    }
    longest_entry = 0;
    host_ready();
    // The wire's own time at the SCL period in force: the slope of two
    // writes on the engines, 255 and 16 bytes, 239 x 9 SCL periods apart.
    owner = Owner::dma_host;
    (void)DmaHost::init(clock);
    (void)cost_of<DmaHost>(1, 0);   // the speed applied
    const Cost e255 = cost_of<DmaHost>(255, 0);
    const Cost e16 = cost_of<DmaHost>(16, 0);
    const uint32_t period = (e255.wall - e16.wall) / (239u * 9u);
    const auto wire = [&](uint32_t bytes) { return bytes * 9u * period; };
    const Cost er16 = cost_of<DmaHost>(0, 16);
    const Cost er255 = cost_of<DmaHost>(0, 255);
    const Cost ereg = cost_of<DmaHost>(1, 4);
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
    print_cost("reg 1 + 1    ", reg1, wire(4));
    print_cost("reg 1 + 4    ", reg4, wire(7));
    print_cost("probe        ", probe, wire(1));
    print(serial, "  the engines:", crlf);
    print_cost("write 16     ", e16, wire(17));
    print_cost("write 255    ", e255, wire(256));
    print_cost("read 16      ", er16, wire(17));
    print_cost("read 255     ", er255, wire(256));
    print_cost("reg 1 + 4    ", ereg, wire(7));
    const uint32_t pump_body = w255.entries != 0u ? w255.body / w255.entries : 0u;
    print(serial, "  the pump's entry a byte: ", pump_body, " cycles on average over the 255-byte write, ",
          r16.entries != 0u ? r16.body / r16.entries : 0u, " over the 16-byte read; the engined 255-byte "
          "write at ", e255.wall * 1000u / wire(256), " thousandths of its wire, the core in the vector ",
          e255.body, " cycles of ", e255.wall, crlf);
    print(serial, "  the longest event entry of every shape above: ", longest_entry, " cycles, stamps included",
          crlf);
    print(serial, "  unwedges so far (15.9.1's stuck BUSY taken out by start()): plain ", Host::unwedges(),
          ", engined ", DmaHost::unwedges(), crlf);
    const bool all_ok = w1.status == i2c_ok && w2.status == i2c_ok && w16.status == i2c_ok &&
                        w255.status == i2c_ok && r1.status == i2c_ok && r2.status == i2c_ok &&
                        r16.status == i2c_ok && reg1.status == i2c_ok && reg4.status == i2c_ok &&
                        probe.status == i2c_ok && e16.status == i2c_ok && e255.status == i2c_ok &&
                        er16.status == i2c_ok && er255.status == i2c_ok && ereg.status == i2c_ok;
    bench.verdict("every shape completed i2c_ok", all_ok);
    bench.verdict("the period is the rate asked: within 2 % above and a quarter below 400 kHz",
                  period != 0u && scl_bracket(SysClock::hz / period, 400'000u));
    bench.verdict("a one- and a two-byte write take THREE event entries (SB, ADDR loading both bytes, "
                  "BTF), and a register read 1 + 1 takes no TxE entry",
                  w1.entries == 3u && w2.entries == 3u && reg1.entries == 5u);
    bench.verdict("the pump is one entry a byte past the first two: a write of N takes N + 1",
                  w16.entries == 17u && w255.entries == 256u);
    bench.verdict("the engines: a write of any length three event entries and no DMA interrupt, a read "
                  "two and one DMA interrupt",
                  e16.entries == 3u && e255.entries == 3u && e16.dma == 0u && e255.dma == 0u &&
                      er16.entries == 2u && er255.entries == 2u && er16.dma == 1u && er255.dma == 1u);
    bench.verdict("no vector re-entered after a completion edge (the event line is the tenure's)",
                  w1.after == 0u && w16.after == 0u && e255.after == 0u && r16.after == 0u);
    bench.verdict("the engined 255-byte write runs at the wire's own time (x 1.00 within 2 %)",
                  e255.wall <= wire(256) + wire(256) / 50u);
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

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Serial::isr(); }

extern "C" BRIO_CH32_INTERRUPT void i2c1_ev_handler() {
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
    } else if (owner == Owner::bus_ao) {
        if (Host::isr()) {
            brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
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

extern "C" BRIO_CH32_INTERRUPT void i2c1_er_handler() {
    error_isr_entries = error_isr_entries + 1u;
    if (owner == Owner::dma_host) {
        if (DmaHost::error_isr()) {
            host_done = true;
        }
    } else if (owner == Owner::bus_ao) {
        if (Host::error_isr()) {
            brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
        }
    } else if (Host::error_isr()) {
        host_done = true;
    }
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() {
    dma6_entries = dma6_entries + 1u;
    if (owner == Owner::dma_host && DmaHost::dma_isr()) {
        host_done = true;
    }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() {
    dma7_entries = dma7_entries + 1u;
    if (owner == Owner::dma_host && DmaHost::dma_isr()) {
        host_done = true;
    }
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
    bench.letter('d', "the tenure shapes, the four receive procedures, the general call", td_shapes);
    bench.letter('e', "the vocabulary on the wire, and commanded stretching priced", te_vocabulary);
    bench.letter('f', "the two speeds against a second chip, timed; a rung past the chapter", tf_speeds);
    bench.letter('g', "THE DMA ENGINES on channels 6 and 7 against the peer", tg_dma);
    bench.letter('h', "THE KERNEL: I2cBus over I2cHost, the rejection, the votes, a held SDA and a held "
                      "clock answered in their place", th_kernel);
    bench.letter('i', "THE REFUSAL, wireless: i2c_rejected inside start() through the arbiter", ti_refusal);
    bench.letter('j', "ARBITRATION both ways, and this board as the peer's CLIENT", tj_arbitration);
    bench.letter('k', "the stuck bus: the peer holding SDA, unstick() counting", tk_unstick);
    bench.letter('q', "the siblings' quirks re-measured on this die", tq_quirks);
    bench.letter('r', "THE REPEATED START on the last byte's TxE, acknowledged and refused", tr_restart);
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
