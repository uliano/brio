// test_vx03_i2c - the reference bench suite for the I2C chapter of the
// CH32V203 and the CH32V303: ch32vx03/i2c.hpp over RM ch. 19, the host
// engine under its two vectors, the client half addressed by a real
// controller, the DMA engines, and util/i2c_bus.hpp's arbiter with not
// one line changed - every letter on I2C1, and every far end a second
// chip running `twi_peer`. I2C2 reaches no wire on the desk this suite is
// written for (its pads carry a serial link there): its block is
// compiled in the stratum's fixture and declared in docs/ch32vx03/i2c.md.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE CLOCK IS 96 MHz AND THAT IS THE CHAPTER'S DOING. CTLR2.FREQ
// states PB1 in whole megahertz and RM 19.12.2 confines the field to
// 4..60 MHz, so the 72 MHz PB1 reaches at the top of this tree has no
// legal timing at all: a program with an I2C keeps HCLK at or below 120
// MHz. 96 MHz gives PB1 = 48 MHz (CCR 240 and 40, exactly 100 and 400
// kHz) and TIM4 = 96 MHz, which is the ruler below.
//
// WHAT IT MEASURES WITH. Two instruments, and neither is a scope:
//
//  - TIM4's CHANNEL 1 IS SCL'S OWN PAD. PB6 is I2C1's clock line and
//    TIM4_CH1 on its default column at once, and a pad driven by one
//    peripheral still reaches another's input on this family (the SPI
//    chapter's finding). In PWM input mode the channel pair measures
//    the PERIOD and the HIGH TIME of every SCL cycle with no wire; in
//    external clock mode 1 the same channel COUNTS SCL edges, which is
//    what prices an unstick().
//
//  - A PEER BOARD running `twi_peer` (a board of the other series of
//    this stratum, the desk partner - the CH32V303 of a CH32V203 and the
//    other way round - or a board of another family; the peer's ident
//    says which), commanded IN BAND over the bus under test through
//    avrdx/src/apps/twi_link.hpp, included by relative path: one source
//    of truth for the wire format on every architecture. Three wires,
//    both boards at 3.3 V:
//
//      PB6 (SCL)  <->  the peer's SCL
//      PB7 (SDA)  <->  the peer's SDA
//      GND        <->  GND
//
//    THE PULL-UPS ARE NOT THIS BOARD'S. An open-drain pad has no pull
//    in any output mode on this family (the pin chapter), so nothing
//    here holds the bus up - and nothing on a CH32 peer does either,
//    for the same reason: the wire wants resistors (the desk's pair
//    carries a 2.2 kOhm pair, docs/boards/), and letter a measures what
//    they are worth in nanoseconds of rise. The peer's command-mode
//    client answers ONE address (0x6B) with no general call and no
//    second address, which is why every wireless letter can run with it
//    attached.
//
//    Every peer letter asks for a ping first and SKIPS (with no
//    verdict claimed) when three command retries fail: a suite that
//    hangs on an absent instrument is worse than one that says so.
//
// THE PADS. PB6 and PB7 - and, in letter p alone, USART2's PA2 (its
// transmitter) and PA3 (its receiver), claimed by that letter's
// transport and released at its end. NEVER TOUCHED: PA9/PA10 (the
// console), PA13/PA14 (the debug port), PA11/PA12 (the USB pads),
// PC14/PC15 and the crystal's pins, PB10..PB15 (the pair's serial and SPI
// links on the CH32V203, the SPI2-SPI3 jumpers on the CH32V303), the rest
// of PA0..PA8 - and PB2, the LED, left undriven, as in every suite of this
// target.
//
// What is exercised, letter by letter:
//   a  THE BLOCK AND THE ARITHMETIC: the reset values, the three
//      timing registers at both speeds and both duties, the enable
//      protection measured, the wire's RISE TIME from the pad, and SCL
//      itself measured by the capture while the host probes - TIM4's
//      clock read against the core's counter first, since the timer runs
//      on PB1 with the divisor - and judged against the rate asked
//   b  THE HOST WITH NO ANSWER: an absent address as i2c_nack_addr,
//      the peer's command address as i2c_ok, BUSY before and after,
//      then the STUCK BUS the peer makes and unstick() with its clocks
//      counted by the timer
//   c  THE PEER: the twi_link command channel, ident, ten round trips
//   d  the tenure shapes against the peer, the four receive
//      procedures, the counted serve ending at the STOP of the read that
//      moved its count, the general call, the repeated START counted at
//      the far end
//   e  the vocabulary on the wire (nack_addr from a deaf client,
//      nack_data at a commanded byte) and commanded stretching priced
//   f  the speeds against a second chip, each timed on the wire and
//      judged against the rate asked
//   g  THE DMA ENGINES on channels 6 and 7
//   h  THE KERNEL: I2cBus over I2cHost, the NACK in its place, the
//      rejection, both votes, a wedge the peer holds and the per-bus
//      timeout that answers it
//   i  ARBITRATION in both directions, and THIS BOARD AS THE CLIENT
//      of the peer's own controller
//   j  the flags, both vectors, the client's addresses and the PE
//      cycle that drops an unclocked byte
//   k  a ten-second stress with the error counters at both ends
//   l  THIS BOARD'S TARGET under the peer's controller: its second
//      address and the general call written to, and THE TARGET AS A
//      TRANSMITTER - the peer's host reading one, two, three, four and
//      eight bytes, twice each, and every byte the shifter asked for
//      beyond the closing NACK dropped by flush()
//   n  A FAR TARGET STUCK MID-BYTE: the peer's target serving zeros, left
//      holding SDA low by this host taken off the bus through its reset
//      line, and unstick()'s clocks counted on the pad by the timer
//   o  THE REFUSED LAST BYTE: the peer's target refusing the last written
//      byte of a write-then-read (one to four written, two read) - the
//      NACK arriving with the repeated START already requested - through
//      the pump and the engines, at 100 and 400 kHz, the core at 96, 48
//      and 8 MHz (the console and the tick re-made at each, and back at
//      96): the status, the controller two milliseconds on (START, STOP,
//      MSL), the next tenure on the same host, the far end's own word that
//      it refused; each count also acknowledged whole, the control; and at
//      each core a write of sixteen through the pump and the engines timed
//      on the core's counter against the rate CCR makes, PB1 another rate
//      at each
//   p  THE HAND-OVER ON CHANNEL 7, the OR of USART2's transmit request and
//      I2C1's receive one: USART2's transport with its transmit engine
//      run and released, then I2C1's DMA read of sixteen from the peer
//      judged byte-exact, both owners timed against the rate asked; and
//      the mirror - I2C1's DMA read abandoned with RxNE standing under
//      DMAEN and the host released, then USART2's run judged out whole in
//      its wire time at 115200 on the core's counter
//   q  A CHANNEL THAT DOES NOT SERVE, staged by a request held on it by a
//      peripheral gated without its reset (TIM3's on channel 6, USART2's
//      on 7): a DMA write answered i2c_dma_fault with the far target short
//      of the block, and a DMA read with the event vector entered for the
//      START and the address alone, the tenure answered by the caller's
//      bound and recover(); each again with the request let go by the
//      holder's reset line, whole
// and by name only, not in z, against the peer:
//   r  THE LATE REPEATED START: a write of one to four bytes and a
//      one-byte read with the repeated START requested AFTER BTF, the
//      order the engine does not use, driven through the resource by
//      hand - once as soon as BTF is seen and once with SCL held 20 us
//      past it - and the engine's own order beside each as the control;
//      the peer's tally says how many of the written bytes its target
//      received. It names a target as losing the last byte or not and
//      judges neither answer: it passes when the peer answered
//
// With the peer attached `z` outlasts `brio run`'s default 60 s (the
// peer's command windows are hundreds of milliseconds each): pass
// `--timeout 400`.
//
// THE 32 KB TIER CANNOT HOLD THIS CHAPTER WHOLE. Built for the
// CH32V203C6 (the family's small-part link guard) the whole suite is
// some seven kilobytes over that part's flash, and the letters' prose
// is not for shortening: the chapter's own surface - two speeds with
// two duty shapes, four receive procedures, two DMA engines, the
// arbiter, both roles and the arbitration - is simply bigger than the
// tier. So the four 32 KB parts build it as one image per GROUP of
// letters (the groups line below; design/overview.md, "A suite's image
// fits the family's smallest chip"). The CH32V203C8 builds two images of
// its own (the groups.v203c8 line); the CH32V303 builds one.
// build: boards = v203c6,v203c8,v303vc
// build: groups = ab,cd,ef,gh,ijkr,ln,o,pq
// build: groups.v203c8 = abcdefgh,ijklnopqr
// build: monitor_speed = 115200

#include <stdint.h>

#include <type_traits>
#include <variant>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/i2c.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrdx/src/apps/twi_link.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
using SysClock = Clock<ClockSource::pll, 96'000'000>;
constexpr SysClock clock;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;

TestBench<Serial> bench;

using H = I2c<1>;
using Host = I2cHost<1>;
using DmaHost = I2cHost<1, i2c_default_pins<1>, DmaTxEngine<1, 6>, DmaRxEngine<1, 7>>;
using Client = I2cClient<1>;
using SclPin = Pin<'B', 6>;
using SdaPin = Pin<'B', 7>;

/// The ruler on SCL's own pad.
using Meter = Tim<4>;
using SclMeter = TimPeriodMeter<Meter>;

/// An address nobody on this bus answers.
constexpr uint8_t nobody_addr = 0x23;

/// A tenure that never answered: the suite's own word, not a wire code.
constexpr uint8_t no_answer = 200;

/// WHO IS HOLDING THE CLOCK, asked of a stalled tenure. THE RCC RESET
/// AND NOT PE: clearing PE does not let a stretching machine's pads go
/// on this silicon (measured - a master parked at SB kept SCL down with
/// PE clear), while the block's reset line provably does, so a level
/// that survives the reset is the far end's. The peripheral is left in
/// pieces; every stall path recovers it in the next breath.
bool clock_held_by_the_far_end() {
    if (SclPin::read()) {
        return false;
    }
    H::reset();
    (void)delay_us(clock, 20);
    return !SclPin::read();
}

/// How many distinct samples a stalled tenure's trail keeps.
constexpr uint8_t trail_len = 8;

volatile bool host_done = false;
volatile bool dma_host_live = false;
volatile bool bus_ao_live = false;
volatile bool client_live = false;
volatile uint32_t host_isr_entries = 0;
volatile uint32_t error_isr_entries = 0;
/// The host's storm budget: a vector that fires this many times inside
/// one tenure is silenced and the fact reported.
constexpr uint32_t host_isr_budget = 60'000;
volatile bool host_stormed = false;
volatile uint32_t storms = 0;
volatile uint16_t storm_s1 = 0;
volatile uint16_t storm_s2 = 0;

uint8_t tx_buf[64];
uint8_t rx_buf[64];

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

void spin_us(uint16_t us) {
    for (uint16_t i = 0; i < us; ++i) {
        (void)delay_us(clock, 1);
    }
}

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 500);
}

// ---------------------------------------------------------------------------
// The host, waited out
// ---------------------------------------------------------------------------

/// Put the plain host back where a letter can use it.
void host_ready() {
    dma_host_live = false;
    bus_ao_live = false;
    client_live = false;
    (void)Host::init(clock);
    if (H::busy()) {
        (void)Host::unstick();
        (void)Host::recover();
    }
}

/// One tenure through the TASK, waited out. Returns the status.
uint8_t host_tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx,
                    uint8_t rx_len, I2cSpeed speed) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    host_done = false;
    host_isr_entries = 0;
    host_stormed = false;
    if (Host::start(r)) {
        return Host::status();
    }
    // A STALL IS A REPORT, and what a wedged tenure needs is not one
    // snapshot but its TRAJECTORY: the status word with the two lines
    // beside it, sampled through the wait and kept whenever it changes.
    uint16_t trail[trail_len] = {};
    uint8_t trail_n = 0;
    uint16_t last_sample = 0xFFFF;
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
        const uint16_t sample = static_cast<uint16_t>(
            (H::status1() & 0x1FFFu) | (SclPin::read() ? 0x8000u : 0u) |
            (SdaPin::read() ? 0x4000u : 0u));
        if (sample != last_sample) {
            last_sample = sample;
            trail[trail_n % trail_len] = sample;
            ++trail_n;
        }
    }
    if (!host_done) {
        print(serial, "    STALL: STAR1=", hex(H::status1()), " STAR2=", hex(H::status2()),
              " CTLR1=", hex(H::regs().CTLR1), " CTLR2=", hex(H::regs().CTLR2), " SCL=",
              SclPin::read() ? "high" : "LOW", " SDA=", SdaPin::read() ? "high" : "LOW",
              " ev entries ", host_isr_entries, " er ", error_isr_entries,
              host_stormed ? " STORMED" : "", "; storms so far ", storms, crlf);
        print(serial, "      the clock, with this peripheral held in its reset: ",
              clock_held_by_the_far_end() ? "STILL LOW - the far end holds it"
                                          : "free - it was this end's",
              crlf);
        print(serial, "      the wait's LAST samples (bit 15 = SCL, bit 14 = SDA, the rest "
                      "STAR1), ", trail_n, " changes in all:");
        const uint8_t first = trail_n > trail_len ? static_cast<uint8_t>(trail_n % trail_len)
                                                  : uint8_t{0};
        const uint8_t count = trail_n > trail_len ? trail_len : trail_n;
        for (uint8_t i = 0; i < count; ++i) {
            print(serial, " ", hex(trail[(first + i) % trail_len]));
        }
        print(serial, crlf);
        (void)Host::recover();
        return no_answer;
    }
    return Host::status();
}

/// The same through the DMA host.
uint8_t dma_tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx,
                   uint8_t rx_len, I2cSpeed speed) {
    DmaHost::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    host_done = false;
    host_isr_entries = 0;
    if (DmaHost::start(r)) {
        return DmaHost::status();
    }
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL (dma): STAR1=", hex(H::status1()), " STAR2=",
              hex(H::status2()), " CTLR2=", hex(H::regs().CTLR2), " SCL=",
              SclPin::read() ? "high" : "LOW", " SDA=", SdaPin::read() ? "high" : "LOW",
              " ch6 flags=", hex(DmaChannel<1, 6>::flags()), " ch7 flags=",
              hex(DmaChannel<1, 7>::flags()), " ch7 ", DmaChannel<1, 7>::enabled() ? "on" : "OFF",
              " with ", DmaChannel<1, 7>::remaining(), " left; ev entries ", host_isr_entries,
              crlf);
        print(serial, "      the clock, with this peripheral held in its reset: ",
              clock_held_by_the_far_end() ? "STILL LOW - the far end holds it"
                                          : "free - it was this end's",
              crlf);
        (void)DmaHost::recover();
        return no_answer;
    }
    return DmaHost::status();
}

// ---------------------------------------------------------------------------
// The wire itself
// ---------------------------------------------------------------------------

/// Both lines read high with nothing driving: somebody holds the bus up.
bool wire_pulled_up() { return SclPin::read() && SdaPin::read(); }

/// THE WIRE'S RISE TIME, in HCLK cycles: the pad pulled low as an
/// open-drain output for a while, released as a floating input, and the
/// cycles until it reads high counted - the pull-up's strength against
/// the wire's capacitance, which a meter at rest cannot tell (a 40 kOhm
/// pull reads 3.3 V too, and misses a 400 kHz bit). 0xFFFF = never rose
/// within the budget.
template <class Pad>
uint32_t rise_cycles() {
    Pad::output(false, PinDrive::open_drain);
    (void)delay_us(clock, 20);
    // The STK reloads every tick: the difference is taken modulo the
    // period, which holds for any rise under a millisecond.
    const uint32_t period = stk()->CMPLR + 1u;
    const uint32_t t0 = stk()->CNTL;
    Pad::input(PinPull::none);
    uint32_t n = 0;
    while (!Pad::read() && n < 0xFFFFu) {
        ++n;
    }
    const uint32_t t1 = stk()->CNTL;
    Pad::release();
    return n >= 0xFFFFu ? 0xFFFFu : (t1 >= t0 ? t1 - t0 : t1 + period - t0);
}

/// Nanoseconds from HCLK cycles, rounded.
uint32_t cycles_to_ns(uint32_t cycles) {
    return (cycles * 1000UL + (SysClock::hz / 1'000'000UL) / 2u) / (SysClock::hz / 1'000'000UL);
}

/// TIM4 in PWM INPUT MODE on PB6: channel 1 captures the rising edges
/// (the period) and channel 2 the falling ones (the high time), the
/// counter reset on every rising edge. The pad stays the I2C's.
bool meter_arm() {
    Meter::init();
    return Meter::remap(0) && SclMeter::setup(0, 0, false);
}

/// The same channel as an EDGE COUNTER: external clock mode 1 on TI1,
/// so the count IS the number of SCL rising edges.
bool edges_arm() {
    Meter::init();
    if (!Meter::remap(0) || !Meter::configure({.prescaler = 0, .period = 0xFFFF})) {
        return false;
    }
    if (!Meter::capture_channel(0, {.select = TimChannelSelect::direct,
                                    .polarity = TimCapturePolarity::rising,
                                    .prescaler = TimCapturePrescaler::every,
                                    .filter = 0,
                                    .enable = true})) {
        return false;
    }
    if (!Meter::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::ti1})) {
        return false;
    }
    Meter::set_count(0);
    Meter::enable(true);
    return true;
}

void meter_off() { Meter::enable(false); }

/// Nanoseconds from TIM4 ticks (the timer counts at timclk1_hz).
uint32_t ticks_to_ns(uint32_t ticks) {
    return (ticks * 1000UL) / (SysClock::timclk1_hz / 1'000'000UL);
}

// ---------------------------------------------------------------------------
// The rate on the wire
// ---------------------------------------------------------------------------

/// TIM4'S OWN CLOCK ON THE CORE'S COUNTER. TIM4 counts a clock of PB1, the
/// bus whose rate the I2C's FREQ and CCR divide, so an SCL period read in
/// its ticks cannot tell a wrong PB1 - the period and the ruler would move
/// together. The STK counts HCLK: TIM4 free-running at a hundredth of its
/// clock for 20 ms of HCLK is the clock the ticks are read against - the
/// window's overshoot one turn of the loop, some ten cycles of 1.9 million.
uint32_t meter_clock_hz() {
    Meter::init();
    (void)Meter::configure({.prescaler = 99, .period = 0xFFFF});
    Meter::set_count(0);
    const uint32_t c0 = Ticker::cycles();
    Meter::enable(true);
    while (Ticker::cycles() - c0 < SysClock::hz / 50u) {
    }
    const uint32_t n = Meter::count();
    Meter::enable(false);
    Meter::release();
    return n * 100u * 50u;
}

/// The rate a speed ASKS for: 100 or 400 kHz, whatever CCR rounds it to.
constexpr uint32_t asked_hz(I2cSpeed speed) {
    return speed == I2cSpeed::standard_100k ? 100'000u : 400'000u;
}

/// SCL's mean rate over a tenure of `periods` SCL periods that took
/// `cycles` of a core at `core_hz`, in thousandths of the rate asked: the
/// START, the STOP and every clock either end stretched are inside it, so
/// it reads low by them and never high. Every core here is a whole number
/// of SCL periods at both rates, so the arithmetic stays in 32 bits.
uint32_t scl_permille(uint32_t cycles, uint32_t periods, I2cSpeed speed, uint32_t core_hz) {
    if (cycles == 0u) {
        return 0;
    }
    return periods * (core_hz / asked_hz(speed)) * 1000u / cycles;
}

/// The verdict on an SCL reading: never above the rate asked by more than
/// two per cent, and within a quarter below it - the pull-ups' rise, the
/// conditions and the stretching the difference.
bool scl_ok(uint32_t permille) { return permille <= 1020u && permille >= 750u; }

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
              ": the peer board must be running `twi_peer`; check the two SCL/SDA wires, "
              "the pull-ups and the GND.",
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
          "  THE PEER DID NOT ANSWER. The peer board must be running `twi_peer`; its "
          "console '0' forces the command-mode client back. Check the two wires in this "
          "file's header.",
          crlf);
    return false;
}

/// Every wire letter asks first and declines with the reason.
bool need_peer() {
    if (!wire_pulled_up()) {
        print(serial, "  SKIPPED, no verdict claimed: SCL/SDA read low with nothing driving "
                      "- no pull-ups on the wire, so no peer board is connected (this board "
                      "carries none).",
              crlf);
        return false;
    }
    if (ensure_link()) {
        return true;
    }
    bench.verdict("the peer answers on the command address (the peer board running twi_peer)",
                  false);
    return false;
}

// ===========================================================================
// a - the block, the arithmetic, the wire
// ===========================================================================

void ta_block() {
    dma_host_live = false;
    bus_ao_live = false;
    client_live = false;
    Pfic::disable(H::event_irq());
    Pfic::disable(H::error_irq());
    H::bus_clock(true);
    H::reset();
    print(serial, "  reset: CTLR1=", hex(H::regs().CTLR1), " CTLR2=", hex(H::regs().CTLR2),
          " OADDR1=", hex(H::regs().OADDR1), " STAR1=", hex(H::status1()), " CKCFGR=",
          hex(H::regs().CKCFGR), " RTR=", hex(H::regs().RTR), crlf);
    bench.verdict("the reset values are tables 19-1's: every register zero but RTR, whose "
                  "reset value is 2",
                  H::regs().CTLR1 == 0u && H::regs().CTLR2 == 0u && H::regs().CKCFGR == 0u &&
                      H::regs().OADDR1 == 0u && H::regs().RTR == 0x0002u);

    // ---- the arithmetic at this clock, in its three registers ----
    const auto sm = i2c_timing_for(SysClock::pclk1_hz, I2cSpeed::standard_100k);
    const auto fm = i2c_timing_for(SysClock::pclk1_hz, I2cSpeed::fast_400k);
    const auto f169 =
        i2c_timing_for(SysClock::pclk1_hz, I2cSpeed::fast_400k, I2cDuty::ratio_16_9);
    print(serial, "  PB1 = ", SysClock::pclk1_hz / 1'000'000u, " MHz; 100k: FREQ=",
          sm->freq_mhz, " CCR=", sm->ckcfgr & i2c_ccr_mask, " TRISE=", sm->trise, " -> ",
          i2c_scl_hz(SysClock::pclk1_hz, *sm), " Hz", crlf);
    print(serial, "  400k duty 2: CKCFGR=", hex(fm->ckcfgr), " TRISE=", fm->trise, " -> ",
          i2c_scl_hz(SysClock::pclk1_hz, *fm), " Hz; duty 16/9: CKCFGR=", hex(f169->ckcfgr),
          " -> ", i2c_scl_hz(SysClock::pclk1_hz, *f169), " Hz", crlf);
    bench.verdict("both speeds resolve at 48 MHz of PB1, exactly (CCR 240 and 40, TRISE 49 "
                  "and 15)",
                  sm->ckcfgr == 240u && sm->trise == 49u && fm->ckcfgr == (i2c_fs | 40u) &&
                      fm->trise == 15u && f169->ckcfgr == (i2c_fs | i2c_duty | 5u));
    bench.verdict("PB1 at the stratum's own ceiling of 72 MHz has NO legal timing - FREQ is "
                  "six bits and 19.12.2 stops at 60 MHz, which is this peripheral's ceiling "
                  "on the whole tree",
                  !i2c_timing_for(72'000'000UL, I2cSpeed::standard_100k).has_value() &&
                      i2c_timing_for(60'000'000UL, I2cSpeed::standard_100k).has_value());
    bench.verdict("the address refusals: 0x80 as a 7-bit address, a second address under "
                  "10-bit mode",
                  !i2c_address_config_valid({.own = 0x80}) &&
                      !i2c_address_config_valid(
                          {.own = 0x123, .ten_bit = true, .second = 0x10}));

    // ---- THE ENABLE PROTECTION, MEASURED ----
    // The chapter orders FREQ, CKCFGR and RTR before PE and says of RTR
    // alone that it "can only be set when PE is cleared" (19.12.9). What
    // the other two do under PE is a measurement.
    H::timing(*sm);
    H::enable();
    H::regs().CKCFGR = static_cast<uint16_t>(i2c_fs | 40u);
    const uint16_t ck_pe = H::regs().CKCFGR;
    H::regs().CTLR2 = static_cast<uint16_t>((H::regs().CTLR2 & ~i2c_freq_mask) | 24u);
    const uint16_t freq_pe = static_cast<uint16_t>(H::regs().CTLR2 & i2c_freq_mask);
    H::regs().RTR = 33u;
    const uint16_t rtr_pe = static_cast<uint16_t>(H::regs().RTR & i2c_trise_mask);
    print(serial, "  with PE set: CKCFGR written 0x8028 reads ", hex(ck_pe),
          ", FREQ written 24 reads ", freq_pe, ", RTR written 33 reads ", rtr_pe, crlf);
    print(serial, "  -> ",
          (ck_pe == (i2c_fs | 40u)) ? "CKCFGR TAKES a write under PE" : "CKCFGR is LOCKED",
          "; ", (freq_pe == 24u) ? "FREQ takes it" : "FREQ is locked", "; ",
          (rtr_pe == 33u) ? "RTR takes it too (against 19.12.9's own sentence)"
                          : "RTR is LOCKED, as 19.12.9 says",
          crlf);
    bench.verdict("the enable protection was measured field by field and reported (either "
                  "answer is a finding)",
                  true);
    H::disable();

    // ---- the addresses, where they land ----
    (void)H::addresses({.own = 0x2C, .second = 0x39, .general_call = true});
    print(serial, "  OADDR1=", hex(H::regs().OADDR1), " OADDR2=", hex(H::regs().OADDR2),
          " CTLR1=", hex(H::regs().CTLR1), crlf);
    bench.verdict("the own address lands shifted, the second under ENDUAL, the general call "
                  "in CTLR1",
                  H::regs().OADDR1 == (0x2Cu << 1) &&
                      H::regs().OADDR2 == (i2c_endual | (0x39u << 1)) &&
                      (H::regs().CTLR1 & i2c_engc) != 0u);
    (void)H::addresses({.own = 0x123, .ten_bit = true});
    print(serial, "  10-bit: OADDR1=", hex(H::regs().OADDR1), crlf);
    bench.verdict("a 10-bit own address lands whole under ADDMODE (19.12.3)",
                  H::regs().OADDR1 == (i2c_addmode | 0x123u));
    H::reset();

    // ---- THE WIRE: what holds it up, and how fast ----
    const bool up = wire_pulled_up();
    print(serial, "  the wire at rest: SCL ", SclPin::read() ? "high" : "LOW", ", SDA ",
          SdaPin::read() ? "high" : "LOW", " - the pads pull neither up (resistors on the "
          "board or a peer's pads do, where anything does)", crlf);
    if (up) {
        const uint32_t scl_rise = rise_cycles<SclPin>();
        const uint32_t sda_rise = rise_cycles<SdaPin>();
        print(serial, "  rise from low, released: SCL ", scl_rise, " cycles (",
              cycles_to_ns(scl_rise), " ns), SDA ", sda_rise, " cycles (",
              cycles_to_ns(sda_rise), " ns) - an UPPER BOUND, the pad's own mode switch "
              "inside it", crlf);
        print(serial, "  -> the wire's pull-up is ",
              cycles_to_ns(scl_rise) > 1000u ? "WEAK: a microsecond and more is an internal "
                                               "pull of tens of kiloohms, and fast mode "
                                               "would lose its high half to it"
                                             : "a RESISTOR: tens of kiloohms would take a "
                                               "microsecond and more, and the high halves "
                                               "measured below say the same",
              crlf);
        bench.verdict("both lines rise when released - something on the wire holds the bus up, "
                      "and the number above says how strongly",
                      scl_rise < 0xFFFFu && sda_rise < 0xFFFFu);
    } else {
        print(serial, "  no pull-up on the wire: the rise time cannot be measured and the "
                      "wire letters will decline.",
              crlf);
    }

    // ---- SCL ITSELF, measured on its own pad while the host probes ----
    const uint32_t tim_hz = meter_clock_hz();
    print(serial, "  TIM4 counts ", tim_hz / 1000u, " kHz on the core's counter (",
          SysClock::timclk1_hz / 1000u, " by the clock tree)", crlf);
    host_ready();
    if (!meter_arm()) {
        bench.verdict("TIM4's channel 1 takes SCL's pad in PWM input mode", false);
        return;
    }
    bench.verdict("TIM4's channel 1 takes SCL's own pad (PB6) in PWM input mode - a pad "
                  "driven by one peripheral still reaches another's input",
                  true);
    struct Rung {
        I2cSpeed speed;
        I2cDuty duty;
        const char* name;
    };
    const Rung rungs[] = {{I2cSpeed::standard_100k, I2cDuty::ratio_2, "100k     "},
                          {I2cSpeed::fast_400k, I2cDuty::ratio_2, "400k d2  "},
                          {I2cSpeed::fast_400k, I2cDuty::ratio_16_9, "400k d16/9"}};
    uint8_t measured = 0;
    uint8_t on_rate = 0;
    for (uint8_t i = 0; i < 3u; ++i) {
        (void)Host::init(clock, rungs[i].duty);
        // A probe of an address nobody answers: nine SCL pulses, and the
        // last complete cycle is what the capture registers hold.
        const uint8_t st = host_tenure(nobody_addr, nullptr, 0, nullptr, 0, rungs[i].speed);
        const uint32_t period = SclMeter::period_ticks();
        const uint32_t high = SclMeter::width_ticks();
        const uint32_t low = period > high ? period - high : 0u;
        const uint32_t asked = Host::scl_hz(rungs[i].speed);
        const uint32_t seen = period != 0u ? tim_hz / period : 0u;
        const uint32_t pm = 1000u * seen / asked_hz(rungs[i].speed);
        print(serial, "  ", rungs[i].name, ": the arithmetic gives ", asked / 1000u,
              " kHz, measured ", seen / 1000u, " kHz - ", pm, " thousandths of the ",
              asked_hz(rungs[i].speed) / 1000u, " asked (period ", ticks_to_ns(period),
              " ns, high ", ticks_to_ns(high), " ns, low ", ticks_to_ns(low), " ns), probe=", st,
              crlf);
        if (period != 0u && high != 0u && st == i2c_nack_addr) {
            ++measured;
        }
        if (period != 0u && scl_ok(pm)) {
            ++on_rate;
        }
    }
    // The high time is where the wire shows: the block releases SCL and
    // the pull-up has to carry it up, so what a capture sees as HIGH is
    // short by the rise, and the low half is the one the silicon drives.
    const uint32_t nominal_high_ns =
        (1000UL * (i2c_timing_for(SysClock::pclk1_hz, I2cSpeed::standard_100k)->ckcfgr &
                   i2c_ccr_mask)) /
        (SysClock::pclk1_hz / 1'000'000UL);
    print(serial, "  at 100 kHz the register asks for ", nominal_high_ns,
          " ns of high time: what the pad shows above is that less the rise the pull-up "
          "costs",
          crlf);
    bench.verdict("every rung was measured ON THE PAD with no wire and no scope, and each "
                  "probe reported nobody-home",
                  measured == 3u);
    bench.verdict("AND SCL RUNS AT THE RATE ASKED: each rung's period, read against TIM4's "
                  "clock as the core's counter measures it, never above 100 or 400 kHz by more "
                  "than 2 % and within a quarter below - the timer alone shares PB1 with the "
                  "divisor and could not tell",
                  on_rate == 3u);
    meter_off();
    (void)Host::init(clock);
}

// ===========================================================================
// b - the host with no answer, and the stuck bus
// ===========================================================================

void tb_no_answer() {
    host_ready();
    const bool idle_before = !H::busy();
    const uint8_t absent = host_tenure(nobody_addr, nullptr, 0, nullptr, 0, link_speed);
    // BUSY IS THE WIRE AND NOT A REGISTER BIT (19.12.7: "SDA or SCL has a
    // low level", cleared when the STOP is DETECTED), so the question is
    // not whether it is clear the instant the engine answers but HOW LONG
    // the STOP takes to leave: the answer comes back in microseconds.
    uint16_t stop_us = 0;
    while (stop_us < 1000u && H::busy()) {
        (void)delay_us(clock, 1);
        ++stop_us;
    }
    const bool idle_after = !H::busy();
    uint8_t w[2] = {0xA5, 0x5A};
    const uint8_t absent_write = host_tenure(nobody_addr, w, 2, nullptr, 0, link_speed);
    print(serial, "  an absent address: probe=", absent, " write=", absent_write,
          " (i2c_nack_addr = ", i2c_nack_addr, "); BUSY before the tenure ",
          idle_before ? 0 : 1, ", and it cleared ", stop_us,
          " us after the engine answered", crlf);
    bench.verdict("an address nobody answers is i2c_nack_addr, for a probe and for a write "
                  "alike - the scanner's result",
                  absent == i2c_nack_addr && absent_write == i2c_nack_addr);
    bench.verdict("and the bus goes back to IDLE within a bit time: the STOP the error path "
                  "issues really goes out, and BUSY follows the WIRE",
                  idle_before && idle_after && stop_us < 100u);

    if (!need_peer()) {
        return;
    }
    const uint8_t there = host_tenure(twilink::command_addr, nullptr, 0, nullptr, 0,
                                      link_speed);
    print(serial, "  the peer's command address probed: ", there, " (i2c_ok = ", i2c_ok, ")",
          crlf);
    bench.verdict("the peer's command address answers the empty tenure - the probe is a real "
                  "address scan, on a board that is there",
                  there == i2c_ok);

    // ---- THE STUCK BUS: the peer holds SDA down from its own port ----
    const uint8_t clean = Host::unstick();
    bench.verdict("unstick() on a healthy bus clocks nothing and says so", clean == 0u);

    twilink::Params h{};
    h.ms = 400;
    h.aux8 = 4;   // released after four SCL falling edges
    h.aux16 = 0;
    if (!peer_act(Op::hold_sda, h)) {
        bench.verdict("the peer accepted the hold_sda command", false);
        return;
    }
    // The peer enters its action when the command's ack read completes, so
    // the hold appears a moment later: it is WAITED FOR, and the wait is
    // the measurement.
    uint16_t held_after_ms = 0;
    while (held_after_ms < 100u && SdaPin::read()) {
        settle_ms(1);
        ++held_after_ms;
    }
    const bool held = !SdaPin::read();
    const uint8_t wedged = host_tenure(twilink::command_addr, nullptr, 0, nullptr, 0,
                                       link_speed);
    print(serial, "  SDA went low ", held_after_ms, " ms after the command and is held: ",
          held ? "yes" : "no", "; a tenure into it answered ", wedged, crlf);
    bench.verdict("the peer really held SDA low", held);
    bench.verdict("a tenure into a wire a foreign chip holds down is answered IN ITS PLACE - "
                  "never silence and never i2c_ok",
                  wedged != i2c_ok && wedged != no_answer);

    // The clocks unstick() makes, COUNTED on the pad by TIM4.
    if (!edges_arm()) {
        bench.verdict("TIM4's channel 1 counts SCL's edges", false);
        return;
    }
    host_isr_entries = 0;
    const uint32_t storms_before = storms;
    Meter::set_count(0);
    const uint8_t pulses = Host::unstick();
    const uint32_t edges = Meter::count();
    meter_off();
    print(serial, "  unstick() reported ", pulses, " pulse(s) and the timer counted ", edges,
          " SCL rising edges on the pad", crlf);
    bench.verdict("unstick() clocked until the client released - a handful of pulses, never "
                  "0xFF",
                  pulses >= 1u && pulses <= 9u);
    bench.verdict("... and the timer counted the same clocks on the pad, plus the STOP's own "
                  "rise: the count is a MEASUREMENT of the remedy, not its own report",
                  edges >= pulses && edges <= static_cast<uint32_t>(pulses) + 2u);
    // THE STOP unstick() MAKES BY HAND raises STOPF on this instance's own
    // target half, and the event vector is the TENURE's (raised by start(),
    // down at the completion: docs/ch32vx03/i2c.md), so the flag stands in
    // silence until the next tenure's first entry takes it.
    settle_ms(450);
    const uint32_t idle_entries = host_isr_entries;
    const bool stopf_left = (H::status1() & i2c_stopf) != 0u;
    print(serial, "  the event vector, idle, after the hand-made STOP: ", idle_entries,
          " entries, STOPF ", stopf_left ? "standing" : "down", " (STAR1=", hex(H::status1()),
          ")", crlf);
    bench.verdict("the STOPF the hand-made STOP leaves stands in silence: the event vector, "
                  "down between tenures, never enters for it - no storm",
                  idle_entries == 0u && storms == storms_before);
    bench.verdict("with the wire free again unstick() clocks nothing", Host::unstick() == 0u);
    const bool back = command(Op::ping);
    print(serial, "  after the next tenures: STOPF ",
          (H::status1() & i2c_stopf) != 0u ? "STILL STANDING" : "taken", crlf);
    bench.verdict("and the command channel is back, its first tenure's vector taking the "
                  "standing STOPF",
                  back && (H::status1() & i2c_stopf) == 0u);
}

// ===========================================================================
// c - the peer link
// ===========================================================================

void tc_peer_link() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping over the two wires - and every command is TWO "
                  "TENURES of the engine under test, a write carrying the frame and a read "
                  "collecting the answer",
                  true);

    twilink::Frame f;
    const bool got =
        query(Op::ident, f) && f.op == Op::ident_data && f.len == twilink::ident_size;
    if (got) {
        const auto id = twilink::get_ident(f.data);
        char label[9] = {};
        for (uint8_t i = 0; i < 8; ++i) {
            label[i] = id.label[i];
        }
        print(serial, "  peer: label '", label, "' xtal=", id.xtal, " sanity=",
              hex(id.sanity), " fw=", hex(id.version), crlf);
        bench.verdict("ident comes back and it IS twi_peer (the sanity byte), from a SECOND "
                      "BOARD of another architecture speaking the same wire format",
                      id.sanity == twilink::ident_sanity);
    } else {
        bench.verdict("ident comes back", false);
    }

    uint8_t good = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (command(Op::ping)) {
            ++good;
        }
    }
    print(serial, "  ", good, " of 10 command round trips answered at ",
          Host::scl_hz(link_speed) / 1000u, " kHz", crlf);
    bench.verdict("the channel is steady over ten command round trips", good == 10u);
    // The twenty-byte read of the answer is itself the answer to a
    // question the peer's own header asks: if this block's BTF stretch
    // fell BEFORE the ninth pulse, the closing NACK could never be
    // delivered and the channel would wedge on its last byte.
    bench.verdict("... which is where the target's BTF stretch falls: AFTER the ninth pulse, "
                  "or a twenty-byte read could not be closed by the controller's NACK",
                  good == 10u);
}

// ===========================================================================
// d - the tenure shapes against the peer
// ===========================================================================

void td_shapes() {
    if (!need_peer()) {
        return;
    }
    twilink::Params a{};
    a.count = 64;
    a.ms = 250;
    a.addr = twilink::dut_addr;
    a.seed = 0x30;
    a.pattern = twilink::pattern_counting;
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
    print(serial, "  write=", ws, " read=", rs, " mism=", rmism, " combined=", cs,
          "; peer: count=", r.count, " addr_hits=", r.addr_hits, " first=", hex(r.first),
          " sum=", hex(r.sum), crlf);
    bench.verdict("a write tenure, a read tenure and the combined write-then-read all "
                  "complete i2c_ok against a SECOND CHIP",
                  ws == i2c_ok && rs == i2c_ok && cs == i2c_ok);
    bench.verdict("the bytes read back are the peer's own pattern, byte-exact", rmism == 0u);
    // WHAT THE FAR END COUNTS IS WHAT IT HANDED ITS OWN REGISTER, and a
    // target transmitter of this lineage is asked for one byte more than
    // the controller takes (its shifter runs one ahead), so the bytes it
    // RECEIVED are exact while the ones it SERVED may be one or two over.
    print(serial, "  the peer's own tally: ", r.aux0, " received, ", r.aux1,
          " served (12 and 12 on the wire; a target transmitter is asked for one byte more "
          "than the controller takes)", crlf);
    bench.verdict("the peer received every byte this board wrote, exactly (8 then 4)",
                  rep && r.aux0 == 12u);
    bench.verdict("... and served at least every byte this board read - the few it counts "
                  "over are the ones its shifter is asked for ahead of the wire, which no "
                  "controller ever clocks",
                  rep && r.aux1 >= 12u && r.count >= 24u);
    bench.verdict("and the COMBINED tenure hit the client's address machinery TWICE - the "
                  "repeated START, counted from the far end",
                  rep && r.addr_hits == 4u);

    // THE COUNTS THE RECEIVE PROCEDURE SWITCHES ON: one byte, two, three
    // and four are four different sequences in isr().
    bool counts_ok = true;
    for (uint8_t n = 1; n <= 4u; ++n) {
        twilink::Params c{};
        c.count = 64;
        c.ms = 250;
        c.addr = twilink::dut_addr;
        c.seed = static_cast<uint8_t>(0x40u + n);
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
        print(serial, "  read of ", n, ": status=", st, " mism=", mism, " peer count=",
              cr.count, crlf);
        if (st != i2c_ok || mism != 0u || !crep || cr.count < n) {
            counts_ok = false;
        }
    }
    bench.verdict("reads of ONE, TWO, THREE and FOUR bytes - the chapter's four receive "
                  "procedures - each byte-exact, and the far end served at least each one",
                  counts_ok);

    // THE COUNTED SERVE (twi_link.hpp's `count`): a serve armed for eight
    // bytes and a deadline far beyond them must end at the STOP of the read
    // that moved the eighth - every byte the peer's own, the last one
    // included, and the peer back in command mode long before its
    // deadline. The bound: the read takes under a millisecond at either
    // speed and the peer's re-init a few more, the poll below looks every
    // 5 ms, so the counted exit is seen within some 10 ms and a serve that
    // waited for its deadline at no less than 2000 ms; half the deadline
    // parts the two with a factor of a hundred on the early side.
    {
        constexpr uint32_t deadline_ms = 2000;
        twilink::Params cnt{};
        cnt.count = 8;
        cnt.ms = deadline_ms;
        cnt.addr = twilink::dut_addr;
        cnt.seed = 0x58;
        cnt.pattern = twilink::pattern_counting;
        if (peer_act(Op::serve, cnt)) {
            for (uint8_t i = 0; i < 8; ++i) {
                rx_buf[i] = 0xEE;
            }
            const uint8_t st = host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 8, link_speed);
            const uint32_t t0 = Ticker::millis();
            bool back = false;
            while (!back && Ticker::millis() - t0 < deadline_ms + 500u) {
                back = command_once(Op::ping, no_payload, 0);
                if (!back) {
                    link_ready();
                    settle_ms(5);
                }
            }
            const uint32_t back_ms = Ticker::millis() - t0;
            settle_ms(twilink::arm_ms);
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
            bench.verdict("a COUNTED SERVE ends at the STOP of the read that moved its count: "
                          "the eight bytes the peer's own, the last included, and the peer "
                          "back in command mode well inside half its deadline, not timed out",
                          st == i2c_ok && mism == 0u && back && back_ms < deadline_ms / 2u &&
                              crep && cr.count == 8u &&
                              (cr.flags & twilink::report_timed_out) == 0u);
        } else {
            bench.verdict("the peer accepted the counted serve", false);
        }
    }

    twilink::Params g{};
    g.count = 64;
    g.ms = 250;
    g.addr = twilink::dut_addr;
    g.flags = twilink::flag_general_call;
    if (peer_act(Op::serve, g)) {
        uint8_t gw[2] = {0x5A, 0xA5};
        const uint8_t gs = host_tenure(0x00, gw, 2, nullptr, 0, link_speed);
        settle_ms(300);
        twilink::Report gr{};
        const bool grep = peer_report(gr);
        print(serial, "  general call: status=", gs, " peer last_addr=", hex(gr.last_addr),
              " count=", gr.count, crlf);
        bench.verdict("a GENERAL CALL write reaches the peer at address 0x00",
                      gs == i2c_ok && grep && gr.count == 2u && gr.last_addr == 0x00u);
    } else {
        bench.verdict("the peer accepted the general-call serve", false);
    }
}

// ===========================================================================
// e - the vocabulary on the wire, and commanded stretching
// ===========================================================================

/// How many SCL low spans of a tenure the profile keeps.
constexpr uint8_t stretch_spans = 12;

/// THE STRETCH ON THE WIRE: one write's SCL low spans longer than 100 us,
/// in core cycles, in order - read from the pad by polling it from the
/// loop that waits for the tenure (the vectors do the tenure), so a span
/// is measured to a poll's few cycles.
struct StretchProfile {
    uint8_t status = no_answer;
    uint8_t n = 0;
    uint32_t span[stretch_spans] = {};
};

StretchProfile stretched_write(const uint8_t* w, uint8_t n) {
    StretchProfile p{};
    Host::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(w);
    r.tx_len = n;
    r.speed = link_speed;
    host_done = false;
    host_isr_entries = 0;
    if (Host::start(r)) {
        p.status = Host::status();
        return p;
    }
    const uint32_t min_span = SysClock::hz / 10'000u;   // 100 us
    const uint32_t c0 = Ticker::cycles();
    bool low = false;
    uint32_t fell = 0;
    while (!host_done && Ticker::cycles() - c0 < SysClock::hz / 10u) {
        const bool now_low = !SclPin::read();
        if (now_low && !low) {
            fell = Ticker::cycles();
        } else if (!now_low && low) {
            const uint32_t span = Ticker::cycles() - fell;
            if (span >= min_span) {
                if (p.n < stretch_spans) {
                    p.span[p.n] = span;
                }
                ++p.n;
            }
        }
        low = now_low;
    }
    if (!host_done) {
        (void)Host::recover();
        return p;
    }
    p.status = Host::status();
    return p;
}

void te_vocabulary() {
    if (!need_peer()) {
        return;
    }
    twilink::Params a{};
    a.count = 8;
    a.ms = 600;
    a.addr = twilink::dut_addr;
    a.flags = twilink::flag_deaf;
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the deaf serve", false);
        return;
    }
    uint8_t w[4] = {1, 2, 3, 4};
    const uint8_t deaf = host_tenure(twilink::dut_addr, w, 4, nullptr, 0, link_speed);
    const uint8_t probe = host_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, link_speed);
    settle_ms(700);

    twilink::Params n{};
    n.count = 16;
    n.ms = 600;
    n.addr = twilink::dut_addr;
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
    print(serial, "  deaf=", deaf, " probe=", probe, " data-nack=", nack, " peer flags=",
          hex(r.flags), " count=", r.count, crlf);
    bench.verdict("an address nobody answers reports i2c_nack_addr - on a board that is "
                  "otherwise alive and listening",
                  deaf == i2c_nack_addr && probe == i2c_nack_addr);
    bench.verdict("a commanded NACK on the 3rd data byte reports i2c_nack_data - the "
                  "wire-level vocabulary is REAL statuses across two architectures",
                  nack == i2c_nack_data && rep && (r.flags & twilink::report_nacked) != 0u);

    twilink::Params base{};
    base.count = 128;   // never reached: the deadline ends the serve
    base.ms = 700;
    base.addr = twilink::dut_addr;
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

    twilink::Params st{};
    st.count = 64;
    st.ms = 150;
    st.addr = twilink::dut_addr;
    st.hold_us = 2000;
    if (!peer_act(Op::serve, st)) {
        bench.verdict("the peer accepted the stretched serve", false);
        return;
    }
    t0 = Ticker::millis();
    const StretchProfile prof = stretched_write(w8, 8);
    const uint8_t s2 = prof.status;
    const uint32_t stretched_ms = Ticker::millis() - t0;
    print(serial, "  the clock held low ON THE WIRE, span by span (us):");
    for (uint8_t i = 0; i < prof.n && i < stretch_spans; ++i) {
        print(serial, " ", prof.span[i] / (SysClock::hz / 1'000'000u));
    }
    print(serial, "; ", prof.n, " span(s) of more than 100 us", crlf);
    settle_ms(300);
    twilink::Report sr{};
    const bool srep = peer_report(sr);
    print(serial, "  8 x 8 bytes unstretched: ", base_ms,
          " ms; ONE 8-byte tenure at 2 ms per byte: ", stretched_ms, " ms; peer count=",
          sr.count, crlf);
    bench.verdict("the unstretched baseline tenures all complete i2c_ok", base_ok);
    // WHERE A POLLED CH32 TARGET'S HOLD LANDS ON THE WIRE (twi_peer's header:
    // the hold is spent at the address match and before each byte taken,
    // and the receiver stretches one byte late). The address match holds
    // SCL for the whole hold, h, plus the ninth clock's low half (T/2) and
    // the far loop's pass to the flag and from the hold to the release -
    // under two periods together; a hold before byte k is taken
    // overlaps byte k+1 entering the shifter (at most nine periods), and
    // SCL is held from that byte's BTF to the take: h - 9T to h; the hold
    // before byte 7 holds the clock only after this host's completion (its
    // BTF on byte 8) and the last falls after the tenure entirely. So a
    // write of 8 shows the address span and SIX byte spans inside the
    // tenure. The slack: the far end's hold is delay_us() in 500 us chunks,
    // each a fraction of a microsecond late, and this end polls the pad in
    // under one. Measured over twenty runs: 2008 to 2014 us at the address,
    // 1919 to 1929 at the bytes.
    constexpr uint32_t h_us = 2000;
    constexpr uint32_t t_us = 10;   // one period at 100 kHz
    constexpr uint32_t slack_us = 5;
    const uint32_t per_us = SysClock::hz / 1'000'000u;
    bool spans_ok = prof.n == 7u;
    for (uint8_t i = 0; i < prof.n && i < stretch_spans && spans_ok; ++i) {
        const uint32_t us = prof.span[i] / per_us;
        spans_ok = i == 0u ? (us + slack_us >= h_us && us <= h_us + 2u * t_us + slack_us)
                           : (us + slack_us >= h_us - 9u * t_us && us <= h_us + slack_us);
    }
    bench.verdict("a target stretching by 2 ms holds SCL where its model says - the address "
                  "match for the whole hold, then six byte spans of the hold less the byte the "
                  "shifter takes meanwhile (1910 to 2000 us, the derivation above) - and the "
                  "tenure still completes i2c_ok: stretching is flow control and the controller "
                  "simply waits",
                  s2 == i2c_ok && spans_ok && srep && sr.count >= 8u);
}

// ===========================================================================
// f - the speeds against a second chip, each timed on the wire
// ===========================================================================

void tf_speeds() {
    if (!need_peer()) {
        return;
    }
    struct Rung {
        I2cSpeed speed;
        I2cDuty duty;
        const char* name;
    };
    const Rung rungs[] = {{I2cSpeed::standard_100k, I2cDuty::ratio_2, "100k      "},
                          {I2cSpeed::fast_400k, I2cDuty::ratio_2, "400k d2   "},
                          {I2cSpeed::fast_400k, I2cDuty::ratio_16_9, "400k d16/9"}};
    uint8_t exact = 0;
    uint8_t on_rate = 0;
    const uint32_t tim_hz = meter_clock_hz();
    if (!meter_arm()) {
        bench.verdict("TIM4's channel 1 takes SCL's pad in PWM input mode", false);
        return;
    }
    for (uint8_t i = 0; i < 3u; ++i) {
        (void)Host::init(clock, rungs[i].duty);
        twilink::Params a{};
        a.count = 64;
        a.ms = 250;
        a.addr = twilink::dut_addr;
        a.seed = static_cast<uint8_t>(0x50u + i * 0x10u);
        a.pattern = twilink::pattern_counting;
        if (!peer_act(Op::serve, a)) {
            bench.verdict("the peer accepted the serve for this rung", false);
            break;
        }
        for (uint8_t k = 0; k < 8; ++k) {
            tx_buf[k] = static_cast<uint8_t>(0xC0u + k);
            rx_buf[k] = 0xEE;
        }
        console_drain();   // the capture is of the BUS, with no print in it
        const uint8_t ws = host_tenure(twilink::dut_addr, tx_buf, 8, nullptr, 0,
                                       rungs[i].speed);
        const uint32_t period = SclMeter::period_ticks();
        const uint32_t high = SclMeter::width_ticks();
        const uint8_t rs = host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 8,
                                       rungs[i].speed);
        uint8_t mism = 0;
        for (uint8_t k = 0; k < 8; ++k) {
            if (rx_buf[k] != twilink::pattern_value(a.pattern, a.seed, k)) {
                ++mism;
            }
        }
        settle_ms(300);
        const bool ok = ws == i2c_ok && rs == i2c_ok && mism == 0u;
        const uint32_t seen = period != 0u ? tim_hz / period : 0u;
        const uint32_t pm = 1000u * seen / asked_hz(rungs[i].speed);
        if (period != 0u && scl_ok(pm)) {
            ++on_rate;
        }
        print(serial, "  ", rungs[i].name, ": asked ",
              Host::scl_hz(rungs[i].speed) / 1000u, " kHz, on the wire ", seen / 1000u,
              " kHz, ", pm, " thousandths of the rate asked (high ", ticks_to_ns(high), " ns of ",
              ticks_to_ns(period),
              " ns); write=", ws, " read=", rs, " mism=", mism, " -> ",
              ok ? "byte-exact both ways" : "NOT exact", crlf);
        if (ok) {
            ++exact;
        }
    }
    meter_off();
    (void)Host::init(clock);
    print(serial, "  ", exact, " of 3 rungs byte-exact against the peer", crlf);
    bench.verdict("Standard mode and Fast mode in BOTH duty shapes carry a write and a read "
                  "byte-exact between TWO SEPARATE CHIPS - which is the highest rate this "
                  "chapter has, and the wire carries it",
                  exact == 3u);
    bench.verdict("and each rung's SCL, read against TIM4's clock on the core's counter, is never "
                  "above the rate asked by more than 2 % and within a quarter below it",
                  on_rate == 3u);
    bench.verdict("the command channel survives the ladder", command(Op::ping));
}

// ===========================================================================
// g - the DMA engines
// ===========================================================================

/// One serve armed for one measurement, and the count of bytes the far
/// end reports for it. A serve per shape is not decoration: THE
/// INSTRUMENT'S TARGET HALF IS RE-ARMED BY THE COMMAND that arms the
/// serve, and measured on this desk it needs that between the reads it
/// closes - the stall report's own question, which end is holding the
/// clock, answered the far end's more than once before each shape got
/// its own serve. Every other letter here arms one too.
struct PeerServe {
    bool armed = false;
    uint16_t received = 0;
    uint16_t served = 0;
};

PeerServe arm_serve(uint8_t seed) {
    twilink::Params a{};
    a.count = 0;   // no early exit: the deadline alone ends it
    a.ms = 300;
    a.addr = twilink::dut_addr;
    a.seed = seed;
    a.pattern = twilink::pattern_counting;
    PeerServe s{};
    s.armed = peer_act(Op::serve, a);
    return s;
}

bool collect_serve(PeerServe& s) {
    settle_ms(350);   // past the serve's own deadline: the peer is back
    twilink::Report r{};
    if (!peer_report(r)) {
        return false;
    }
    s.received = r.aux0;
    s.served = r.aux1;
    return true;
}

/// THE TWO HOSTS SHARE THE INSTANCE AND NOT THEIR STATICS: the command
/// channel is the plain engine's and the measurements are the engined
/// one's, so the vectors are handed over between them and never guessed.
void dma_host_up() {
    dma_host_live = true;
    (void)DmaHost::init(clock);
}

void dma_host_down() {
    DmaHost::release();
    dma_host_live = false;
    host_ready();
}

void tg_dma() {
    if (!need_peer()) {
        return;
    }
    for (uint8_t i = 0; i < 16; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x80u + i);
        rx_buf[i] = 0xEE;
    }
    DmaTxEngine<1, 6>::clear_faults();   // the counters live for the whole boot
    DmaRxEngine<1, 7>::clear_faults();

    // 1. sixteen bytes each way, the engines carrying both phases.
    PeerServe one = arm_serve(0x60);
    dma_host_up();
    const uint8_t ws = one.armed ? dma_tenure(twilink::dut_addr, tx_buf, 16, nullptr, 0,
                                              link_speed)
                                 : no_answer;
    const uint8_t rs = one.armed ? dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 16,
                                              link_speed)
                                 : no_answer;
    dma_host_down();
    uint8_t mism = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (rx_buf[i] != twilink::pattern_value(twilink::pattern_counting, 0x60, i)) {
            ++mism;
        }
    }
    const bool got1 = collect_serve(one);

    // 2. the combined write-then-read, both engines inside one tenure.
    for (uint8_t i = 0; i < 4; ++i) {
        rx_buf[i] = 0xEE;
    }
    PeerServe two = arm_serve(0x70);
    dma_host_up();
    const uint8_t cs = two.armed ? dma_tenure(twilink::dut_addr, tx_buf, 4, rx_buf, 4,
                                              link_speed)
                                 : no_answer;
    dma_host_down();
    const bool got2 = collect_serve(two);

    // 3. the two-byte read, which the receive engine takes under LAST.
    PeerServe three = arm_serve(0x80);
    dma_host_up();
    const uint8_t r2 = three.armed ? dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf + 8, 2,
                                                link_speed)
                                   : no_answer;
    dma_host_down();
    const bool got3 = collect_serve(three);

    // 4. the one-byte read, which no engine can express and the pump
    //    carries even on an engined host.
    PeerServe four = arm_serve(0x90);
    dma_host_up();
    const uint8_t r1 = four.armed ? dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf + 12, 1,
                                               link_speed)
                                  : no_answer;
    dma_host_down();
    const bool got4 = collect_serve(four);

    const uint16_t received = static_cast<uint16_t>(one.received + two.received +
                                                    three.received + four.received);
    const uint16_t served = static_cast<uint16_t>(one.served + two.served + three.served +
                                                  four.served);
    print(serial, "  DMA: write16=", ws, " read16=", rs, " mism=", mism, " combined=", cs,
          " read2=", r2, " read1(pump)=", r1, "; the peer received ", received,
          " bytes and served ", served, " (20 and 23 on the wire), faults tx=",
          DmaTxEngine<1, 6>::faults(), " rx=", DmaRxEngine<1, 7>::faults(), crlf);
    // 19.12.1's own case, counted: a DMA-served read can end with the
    // wire idle and BUSY standing, and the engine takes the chapter's
    // SWRST out of it before the next START.
    print(serial, "  the stuck-BUSY remedy fired ", DmaHost::unwedges(), " time(s) on the "
                  "engined host and ", Host::unwedges(), " on the plain one", crlf);
    bench.verdict("a 16-byte write and a 16-byte read through the engines on channels 6 and "
                  "7 complete i2c_ok, byte-exact",
                  ws == i2c_ok && rs == i2c_ok && mism == 0u);
    bench.verdict("the combined tenure, the two-byte read (LAST's NACK) and the one-byte "
                  "read (which stays on the pump) all complete",
                  cs == i2c_ok && r2 == i2c_ok && r1 == i2c_ok);
    bench.verdict("the peer received the 20 bytes the engines wrote, exactly, and served "
                  "the 23 they read, with no transfer fault on either channel",
                  got1 && got2 && got3 && got4 && received == 20u && served >= 23u &&
                      DmaTxEngine<1, 6>::faults() == 0u && DmaRxEngine<1, 7>::faults() == 0u);
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
    twilink::Params a{};
    a.count = 512;   // never reached: the deadline is the exit
    a.ms = 900;
    a.addr = twilink::dut_addr;
    a.seed = 0x70;
    a.pattern = twilink::pattern_counting;
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve the kernel letter drives", false);
        return;
    }
    bench.verdict("the peer accepted the serve the kernel letter drives", true);

    kl::BusKernel::init_all();
    bus_ao_live = true;

    fill_pattern(kl::out_a, 4, 0x01);
    fill_pattern(kl::out_b, 4, 0x02);
    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::I2cArb>(
            kl::request((i == 2u) ? nobody_addr : twilink::dut_addr, kl::out_a));
    }
    kl::pump_until(4, 400);
    print(serial, "  four queued tenures: replies ", kl::Probe::n, " [", kl::Probe::replies[0],
          " ", kl::Probe::replies[1], " ", kl::Probe::replies[2], " ", kl::Probe::replies[3],
          "]", crlf);
    bench.verdict("four tenures through I2cBus against a SECOND CHIP, four replies - "
                  "util/i2c_bus.hpp and util/bus_master.hpp with not one line changed for "
                  "this architecture",
                  kl::Probe::n == 4u);
    bench.verdict("... in order, with the NACK from an address nobody answers delivered IN "
                  "ITS PLACE as a reply",
                  kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_ok &&
                      kl::Probe::replies[2] == i2c_nack_addr &&
                      kl::Probe::replies[3] == i2c_ok);

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_b));
    }
    kl::pump_until(6, 400);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n,
          ", rejected ", kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately",
                  kl::Probe::rejected != 0u);
    bench.verdict("... and every request is still answered exactly once", kl::Probe::n == 6u);

    kl::drain(100);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep",
                  kl::Probe::votes == 1u && kl::Probe::last_vote);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    post<kl::I2cArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    kl::drain(150);
    print(serial, "  the vote from a BUSY bus: ", kl::Probe::votes, " vote(s), last ",
          kl::Probe::last_vote ? "yes" : "no", crlf);
    bench.verdict("a BUSY bus votes against it",
                  kl::Probe::votes == 1u && !kl::Probe::last_vote);

    // ---- THE CLOCK HELD past the limit, by the OTHER BOARD ----
    // The F1 lineage's I2C has no clock-low time-out outside SMBus mode,
    // so the one thing that can answer a client holding SCL is the
    // arbiter's own per-bus limit.
    bus_ao_live = false;
    settle_ms(400);
    link_ready();
    twilink::Params held{};
    held.count = 16;
    held.ms = 300;
    held.addr = twilink::dut_addr;
    held.hold_us = 45'000;
    const bool armed_clock = peer_act(Op::serve, held);
    bus_ao_live = true;
    if (!armed_clock) {
        bench.verdict("the peer accepted the 45 ms stretch", false);
        bus_ao_live = false;
        return;
    }
    kl::drain(5);
    kl::Probe::clear_tally();
    const uint8_t stale_before = kl::I2cArb::stale_events();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    const uint32_t t1 = Ticker::ticks();
    kl::pump_until(1, 400);
    const uint32_t took = Ticker::ticks() - t1;
    print(serial, "  the tenure into a held CLOCK answered ", kl::Probe::n, " with status ",
          kl::Probe::replies[0], " after ", took, " ms (limit 20, hold 45); SCL at the "
          "reply: ", SclPin::read() ? "high" : "LOW", crlf);
    bench.verdict("a client holding the CLOCK past the limit is answered i2c_timeout ON THE "
                  "ARBITER'S CLOCK - the one wedge this silicon cannot answer by itself",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_timeout && took >= 20u &&
                      took <= 40u);
    kl::drain(100);
    bus_ao_live = false;
    settle_ms(400);
    link_ready();
    twilink::Params after{};
    after.count = 64;
    after.ms = 250;
    after.addr = twilink::dut_addr;
    after.seed = 0x91;
    const bool re = peer_act(Op::serve, after);
    bus_ao_live = true;
    kl::drain(20);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    kl::pump_until(1, 300);
    print(serial, "  after the clock came back: replies ", kl::Probe::n, " status ",
          kl::Probe::replies[0], ", stale events ",
          kl::I2cArb::stale_events() - stale_before, crlf);
    bench.verdict("the recover()ed engine carries the next tenure to i2c_ok through the same "
                  "arbiter",
                  re && kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_ok);
    bus_ao_live = false;
    settle_ms(300);
    host_ready();
}

// ===========================================================================
// i - arbitration, and this board as the client
// ===========================================================================

/// A START CONDITION BY HAND: SDA pulled low from the port while SCL is
/// released high. Both ends' BUSY follows the WIRE (19.12.7), so this is
/// what opens the rendezvous window - and nothing but a STOP closes it.
void inject_start() { SdaPin::output(false, PinDrive::open_drain); }

/// AND THE STOP THAT CLOSES IT: SDA given back to the peripheral, so it
/// rises while SCL is still high. BUSY clears at both ends, and every
/// START that was set while the bus was busy leaves together.
void inject_stop() { SdaPin::function(PinDrive::open_drain); }

/// The rendezvous with no tenure of this board's own: the window is
/// opened, held for `us`, and closed - which is all the peer's host
/// action needs to arm and go.
void hold_bus_busy(uint16_t us) {
    inject_start();
    spin_us(us);
    inject_stop();
}

/// ONE TENURE STARTED INTO A BUSY BUS, which is where two controllers'
/// STARTs can meet: this board makes a START condition by hand, gives the
/// peer time to see BUSY, sets its own START and then makes the STOP that
/// frees the bus. A START set on a busy bus waits for that STOP or for its
/// controller's tick (docs/ch32vx03/i2c.md); ours is set a microsecond
/// before the STOP, so the STOP is what releases it, and the peer's leaves
/// on the same STOP - held until it by a controller that holds a START,
/// set on it by a CH32 peer, which watches the pads because its own tick
/// would release it inside the window - and both reach the wire within
/// the 5 us a released START takes, where the wired-AND decides. The
/// engine's own stuck-BUSY remedy cannot interfere: it is refused while a
/// line is low, which is exactly the state this arranges.
uint8_t arb_tenure(uint8_t addr, const uint8_t* tx, uint8_t len, uint16_t lead_us) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = len;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    r.rx_len = 0;
    r.speed = link_speed;
    host_done = false;
    host_isr_entries = 0;
    inject_start();
    spin_us(lead_us);
    const bool sync = Host::start(r);
    inject_stop();
    if (sync) {
        return Host::status();
    }
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    the race never answered: STAR1=", hex(H::status1()), " STAR2=",
              hex(H::status2()), " CTLR1=", hex(H::regs().CTLR1), " SCL=",
              SclPin::read() ? "high" : "LOW", crlf);
        (void)Host::recover();
        return no_answer;
    }
    return Host::status();
}

void ti_arbitration() {
    if (!need_peer()) {
        return;
    }

    // ---- both hosts racing, in both directions ----
    struct Round {
        uint8_t dut_target;   ///< where THIS board's controller aims
        uint8_t peer_target;  ///< where the peer's aims
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
        a.aux16 = 120;   // the lead-in after BUSY, microseconds
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
        print(serial, "  ", rounds[i].name, crlf, "    this board: status=", st,
              "; peer: flags=", hex(r.flags), " ran=",
              (rep && (r.flags & twilink::report_host_ran) != 0u) ? 1 : 0, " mstatus=",
              r.mstatus, crlf);
        if (rounds[i].dut_should_lose ? dut_lost : (!dut_lost && peer_lost)) {
            ++as_expected;
        }
        if (rounds[i].dut_should_lose) {
            bench.verdict("aiming at the HIGHER address, this controller loses the wired-AND "
                          "and reports i2c_arb_lost - the hardware released the bus and the "
                          "engine says so",
                          dut_lost);
        } else {
            bench.verdict("aiming at the LOWER address, this controller keeps the bus: its "
                          "own address went out (nobody answers 0x11, so the tenure ends "
                          "nack_addr) and it is the PEER that reports the loss",
                          !dut_lost && st == i2c_nack_addr && peer_lost);
        }
    }
    print(serial, "  ", as_expected, " of 2 rounds went as the wired-AND prescribes", crlf);
    bench.verdict("arbitration was observed in BOTH directions - which board aims at the "
                  "lower address decides the winner, and both ends report it",
                  as_expected == 2u);

    // ---- THIS BOARD AS THE CLIENT of the peer's own controller ----
    link_ready();
    twilink::Params c{};
    c.ms = 800;
    c.addr = twilink::dut_addr;
    c.target = twilink::dut_addr;
    c.count = 6;
    c.seed = 0xB0;
    c.aux16 = 400;   // time for this end to put its client up
    if (!peer_act(Op::arb, c)) {
        bench.verdict("the peer accepted the host command that addresses this board", false);
        return;
    }
    // The host goes away and the client takes the pads; the peer is
    // waiting for the bus to go busy, which this end then makes by hand.
    Host::release();
    client_live = true;
    const bool up = Client::init(clock, {.own = twilink::dut_addr},
                                 {.no_stretch = false, .interrupts = false});
    hold_bus_busy(400);

    uint8_t got[16];
    uint8_t n_got = 0;
    uint8_t hits = 0;
    uint8_t stops = 0;
    bool wrote_to_us = false;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 900u) {
        const I2cClientEvent e = Client::service();
        if (e == I2cClientEvent::addressed) {
            ++hits;
            wrote_to_us = !Client::host_reads();
        } else if (e == I2cClientEvent::byte_received) {
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
    print(serial, "  as a CLIENT: init=", up ? 1 : 0, " address matches=", hits, " bytes=",
          n_got, " mism=", mism, " stops=", stops, "; first=",
          n_got != 0u ? hex(got[0]) : hex(uint8_t{0}), crlf);
    bench.verdict("the client answered its own address to a REAL controller on another "
                  "board, in a write tenure",
                  up && hits >= 1u && wrote_to_us);
    bench.verdict("... and took every byte the peer's controller wrote, byte-exact, ending "
                  "on its STOP",
                  n_got >= c.count && mism == 0u && stops >= 1u);
    Client::release();
    client_live = false;
    settle_ms(300);
    host_ready();
    bench.verdict("the command channel is back after the role swap", command(Op::ping));

    // The collision case (twi_link's `coll`, two clients on ONE address
    // with a third party reading) has no third party on this bench: an
    // instance is a host or a client, never both, so the two boards can
    // only ever be one of each. The document's gap list says so.
}

// ===========================================================================
// j - the flags, the vectors, the client's addresses, the PE cycle
// ===========================================================================

void tj_flags() {
    host_ready();

    // ---- the flags a tenure with nobody home raises, and their clears ----
    host_isr_entries = 0;
    error_isr_entries = 0;
    const uint8_t st = host_tenure(nobody_addr, nullptr, 0, nullptr, 0, link_speed);
    print(serial, "  a probe of an absent address: status=", st, ", event vector ",
          host_isr_entries, " entries, error vector ", error_isr_entries, ", STAR1=",
          hex(H::status1()), " STAR2=", hex(H::status2()), crlf);
    bench.verdict("BOTH vectors carried the tenure: the address went out on the event one "
                  "and the NACK came back on the error one",
                  st == i2c_nack_addr && host_isr_entries >= 1u && error_isr_entries >= 1u);
    bench.verdict("the error flags are write-zero-to-clear and the error path left none "
                  "standing",
                  (H::status1() & i2c_all_errors) == 0u);

    // AF by hand: set it, read it, clear it the way 19.12.6 says.
    H::regs().STAR1 = static_cast<uint16_t>(H::status1() | i2c_af);
    const bool af_settable = (H::status1() & i2c_af) != 0u;
    H::clear_errors(i2c_af);
    print(serial, "  AF written as a one: ", af_settable ? "stands" : "ignored",
          "; after the write-zero clear: ", (H::status1() & i2c_af) != 0u ? "stands" : "gone",
          crlf);
    bench.verdict("a status flag of this register is RW0: a one never sets it and a zero "
                  "clears it",
                  !af_settable && (H::status1() & i2c_af) == 0u);

    // ---- the client's own addresses, read back ----
    client_live = true;
    (void)Client::init(clock, {.own = twilink::dut_addr,
                               .second = twilink::shared_addr,
                               .general_call = true},
                       {.no_stretch = false, .interrupts = false});
    const uint16_t o1 = H::regs().OADDR1;
    const uint16_t o2 = H::regs().OADDR2;
    const bool gc = (H::regs().CTLR1 & i2c_engc) != 0u;
    print(serial, "  client: OADDR1=", hex(o1), " OADDR2=", hex(o2), " ENGC=", gc ? 1 : 0,
          " CTLR2=", hex(H::regs().CTLR2), " (the three interrupt enables down: this client "
          "is polled)",
          crlf);
    bench.verdict("the dual address and the general call are armed together, and a POLLED "
                  "client leaves all three interrupt enables down",
                  o1 == (twilink::dut_addr << 1) &&
                      o2 == (i2c_endual | (twilink::shared_addr << 1)) && gc &&
                      (H::regs().CTLR2 & (i2c_itevten | i2c_itbufen | i2c_iterren)) == 0u);

    // ---- THE PE CYCLE: what it drops and what it spares ----
    // A target transmitter is asked for one byte more than the
    // controller takes, and the chapter has no verb that throws the
    // loaded byte away. A PE cycle does - and the question a bench has
    // to answer is whether the address and the timing survive it.
    const uint16_t ckcfgr_before = H::regs().CKCFGR;
    const uint16_t rtr_before = H::regs().RTR;
    const uint16_t ctlr2_before = H::regs().CTLR2;
    const bool txe_idle = Client::data_wanted();
    H::data(0x5A);
    const bool txe_loaded = Client::data_wanted();
    const bool dropped = Client::flush();
    const bool txe_after = Client::data_wanted();
    print(serial, "  the transmit-empty flag of a client nobody is reading: ",
          txe_idle ? "up" : "down", " at rest, ", txe_loaded ? "up" : "down",
          " with a byte written into DATAR, ", txe_after ? "up" : "down",
          " after the PE cycle; flush() ", dropped ? "took the byte" : "found nothing", crlf);
    bench.verdict("with a byte standing in DATAR the transmit-empty flag is DOWN, and "
                  "flush() is the only verb the chapter leaves for taking that byte back - "
                  "a PE cycle, which it ran",
                  !txe_loaded && dropped);
    print(serial, "  across the PE cycle: OADDR1=", hex(H::regs().OADDR1), " CKCFGR=",
          hex(H::regs().CKCFGR), " RTR=", hex(H::regs().RTR), " CTLR2=",
          hex(H::regs().CTLR2), " ACK=", (H::regs().CTLR1 & i2c_ack) != 0u ? 1 : 0, crlf);
    bench.verdict("... and the PE cycle spares the configuration: the own address, both "
                  "timing registers and CTLR2 are what they were, and ACK is put back up",
                  H::regs().OADDR1 == o1 && H::regs().OADDR2 == o2 &&
                      H::regs().CKCFGR == ckcfgr_before && H::regs().RTR == rtr_before &&
                      H::regs().CTLR2 == ctlr2_before && (H::regs().CTLR1 & i2c_ack) != 0u);

    // ---- SMBus, with no SMBus device on the wire ----
    H::smbus(true, true);
    H::arp(true);
    const bool smbus_on = (H::regs().CTLR1 & i2c_smbus) != 0u &&
                          (H::regs().CTLR1 & i2c_smbtype) != 0u &&
                          (H::regs().CTLR1 & i2c_enarp) != 0u;
    H::pec(true);
    const bool pec_on = (H::regs().CTLR1 & i2c_enpec) != 0u;
    const uint8_t pec_value = H::pec_value();
    H::smbus(false);
    H::pec(false);
    H::arp(false);
    print(serial, "  SMBus mode, host type and ARP read back: ", smbus_on ? "yes" : "no",
          "; ENPEC: ", pec_on ? "yes" : "no", "; the PEC register reads ", hex(pec_value),
          " (no SMBus device on this bench - the bits are all that can be judged)", crlf);
    bench.verdict("the SMBus half of CTLR1 takes every bit the chapter gives it, and the "
                  "PEC register answers - what those bits DO needs a device this bench has "
                  "not got",
                  smbus_on && pec_on);
    Client::release();
    client_live = false;
    host_ready();
}

// ===========================================================================
// k - the stress
// ===========================================================================

void tk_stress() {
    if (!need_peer()) {
        return;
    }
    twilink::Params a{};
    a.count = 0;   // no early exit: the deadline alone ends the serve
    a.ms = 10'500;
    a.addr = twilink::dut_addr;
    a.seed = 0x10;
    a.pattern = twilink::pattern_counting;
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the ten-second serve", false);
        return;
    }
    for (uint8_t i = 0; i < 16; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x20u + i);
    }
    uint32_t tenures = 0, bytes = 0, bad = 0, mism = 0;
    error_isr_entries = 0;
    console_drain();   // the ten seconds are the bus's, not the console's
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 10'000u) {
        for (uint8_t i = 0; i < 16; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint8_t ws = host_tenure(twilink::dut_addr, tx_buf, 16, nullptr, 0,
                                       I2cSpeed::fast_400k);
        const uint8_t rs = host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 16,
                                       I2cSpeed::fast_400k);
        if (ws != i2c_ok || rs != i2c_ok) {
            ++bad;
        } else {
            bytes += 32u;
        }
        tenures += 2u;
    }
    const uint32_t took = Ticker::millis() - t0;
    settle_ms(800);
    twilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  ", tenures, " tenures in ", took, " ms at 400 kHz: ", bytes,
          " bytes moved, ", bad, " tenures not i2c_ok, ", mism, " byte mismatches; the peer "
          "counted ", r.addr_hits, " address matches and ", r.count,
          " bytes (both its own 16-bit counters, wrapped), flags=", hex(r.flags), crlf);
    bench.verdict("ten seconds of back-to-back tenures at fast mode with not one failure at "
                  "this end",
                  bad == 0u && tenures > 100u);
    bench.verdict("... and the peer saw no bus error and no arbitration loss of its own",
                  rep && (r.flags & (twilink::report_buserr | twilink::report_arblost)) == 0u);
    bench.verdict("the command channel is unchanged after the stress", command(Op::ping));
}

// ===========================================================================
// r - the late repeated START, by name
// ===========================================================================
//
// THE ENGINE REQUESTS A WRITE-THEN-READ'S REPEATED START WHILE THE LAST
// WRITTEN BYTE STILL SHIFTS (ch32vx03/i2c.hpp's header), because a CH32
// target was measured losing that byte when the START came after BTF.
// This letter drives the OTHER order on purpose - the engine keeps no
// hook for it, so the tenure runs through the resource by hand, every
// flag polled - and asks the far end how many of the written bytes it
// received. The bytes written are 0x01, 0x02, 0x04 and 0x08, so the
// peer's running sum of what it received is also the SET of what it
// received. What a target does with the late START is the measurement:
// the letter judges nothing about it.

/// Poll STAR1 until a flag of `want` or an error flag rises: the word
/// seen, or zero when the bound ran out.
uint16_t await_star1(uint16_t want) {
    for (uint32_t i = 0; i < 400'000UL; ++i) {
        const uint16_t s1 = H::status1();
        if ((s1 & (want | i2c_errors)) != 0u) {
            return s1;
        }
    }
    return 0;
}

/// A hand-driven tenure's failure in i2c_bus.hpp's words: the error flags
/// cleared and the STOP issued where the chapter leaves it to the host
/// (19.5) - an arbitration loss has released the bus already.
uint8_t by_hand_failed(uint16_t s1, bool on_address) {
    if (s1 == 0u) {
        return no_answer;
    }
    H::clear_errors(s1);
    if ((s1 & i2c_arlo) != 0u) {
        return i2c_arb_lost;
    }
    H::stop();
    if ((s1 & i2c_af) != 0u) {
        return on_address ? i2c_nack_addr : i2c_nack_data;
    }
    return i2c_bus_error;
}

/// Whether a polled wait ended on the flag it waited for, and not on an
/// error or on its bound.
bool arrived(uint16_t s1, uint16_t want) {
    return (s1 & i2c_errors) == 0u && (s1 & want) != 0u;
}

/**
 * ONE WRITE-THEN-READ IN THE ORDER THE ENGINE DOES NOT USE: the address
 * and every written byte on their flags (EVT5, EVT6, EVT8), then BTF
 * waited for - the last byte and its acknowledge done, this end holding
 * SCL low - and `hold_us` more, and only then the repeated START; behind
 * it a ONE-byte read by 19.3's procedure for one (ACK clear before ADDR is
 * cleared, STOP right after). The engine's two vectors are silenced for
 * the tenure, and host_ready() gives them back.
 */
uint8_t late_start_tenure(uint8_t addr, const uint8_t* w, uint8_t n, uint16_t hold_us,
                          uint8_t& got) {
    Pfic::disable(H::event_irq());
    Pfic::disable(H::error_irq());
    H::event_interrupt(false);
    H::buffer_interrupt(false);
    H::error_interrupt(false);
    H::clear_errors(i2c_all_errors);
    H::pos(false);
    H::ack(false);
    H::start();
    uint16_t s1 = await_star1(i2c_sb);
    if (!arrived(s1, i2c_sb)) {
        return by_hand_failed(s1, true);
    }
    H::data(static_cast<uint8_t>(addr << 1));
    s1 = await_star1(i2c_addr);
    if (!arrived(s1, i2c_addr)) {
        return by_hand_failed(s1, true);
    }
    (void)H::clear_addr();
    for (uint8_t i = 0; i < n; ++i) {
        s1 = await_star1(i2c_txe);
        if (!arrived(s1, i2c_txe)) {
            return by_hand_failed(s1, false);
        }
        H::data(w[i]);
    }
    // EVT8_2 - TxE and BTF, the last byte out and acknowledged - and only
    // now the repeated START.
    s1 = await_star1(i2c_btf);
    if (!arrived(s1, i2c_btf)) {
        return by_hand_failed(s1, false);
    }
    if (hold_us != 0u) {
        (void)delay_us(clock, hold_us);
    }
    H::start();
    s1 = await_star1(i2c_sb);
    if (!arrived(s1, i2c_sb)) {
        return by_hand_failed(s1, true);
    }
    H::data(static_cast<uint8_t>((addr << 1) | 1u));
    s1 = await_star1(i2c_addr);
    if (!arrived(s1, i2c_addr)) {
        return by_hand_failed(s1, true);
    }
    (void)H::clear_addr();
    H::stop();
    s1 = await_star1(i2c_rxne);
    if (!arrived(s1, i2c_rxne)) {
        return by_hand_failed(s1, false);
    }
    got = H::data();
    return i2c_ok;
}

void tr_late_start() {
    if (!need_peer()) {
        return;
    }
    // WHICH TARGET this is, in its own words: the high byte of the firmware
    // version names the peer's port (twi_link.hpp), the label the board.
    twilink::Frame f;
    if (query(Op::ident, f) && f.op == Op::ident_data && f.len == twilink::ident_size) {
        const auto id = twilink::get_ident(f.data);
        char label[9] = {};
        for (uint8_t i = 0; i < 8; ++i) {
            label[i] = id.label[i];
        }
        print(serial, "  the target: label '", label, "', fw ", hex(id.version),
              " (the high byte names the peer's port, twi_link.hpp)", crlf);
    }

    struct Order {
        bool by_hand;
        uint16_t hold_us;
        const char* name;
    };
    const Order orders[3] = {{true, 0, "the START after BTF, at once "},
                             {true, 20, "the START after BTF, +20 us  "},
                             {false, 0, "the engine's order (control) "}};
    const uint8_t w[4] = {0x01, 0x02, 0x04, 0x08};
    uint8_t reports = 0;
    uint8_t last_in[3] = {};
    uint8_t last_out[3] = {};
    for (uint8_t n = 1; n <= 4u; ++n) {
        for (uint8_t k = 0; k < 3u; ++k) {
            const Order& o = orders[k];
            host_ready();
            const PeerServe s = arm_serve(static_cast<uint8_t>(0x40u + 0x10u * k + n));
            uint8_t got = 0xEE;
            uint8_t st = no_answer;
            if (s.armed) {
                if (o.by_hand) {
                    st = late_start_tenure(twilink::dut_addr, w, n, o.hold_us, got);
                    for (uint16_t t = 0; t < 1000u && H::stopping(); ++t) {
                        (void)delay_us(clock, 1);
                    }
                    host_ready();
                } else {
                    rx_buf[0] = 0xEE;
                    st = host_tenure(twilink::dut_addr, w, n, rx_buf, 1, link_speed);
                    got = rx_buf[0];
                }
            }
            settle_ms(350);   // past the serve's own deadline: the peer is back
            twilink::Report r{};
            const bool rep = s.armed && peer_report(r);
            print(serial, "  ", n, n == 1u ? " byte,  " : " bytes, ", o.name, ": status ", st,
                  ", read ", hex(got));
            if (!rep) {
                print(serial, "; no report from the peer", crlf);
                continue;
            }
            ++reports;
            const bool last = (r.sum & (1u << (n - 1u))) != 0u;
            if (last) {
                ++last_in[k];
            } else {
                ++last_out[k];
            }
            print(serial, "; the target took ", r.aux0, " of ", n, " (the set ", hex(r.sum),
                  ", the last byte ", last ? "in" : "MISSING", "), ", r.addr_hits,
                  " address matches", crlf);
        }
    }
    for (uint8_t k = 0; k < 3u; ++k) {
        print(serial, "  ", orders[k].name, ": the last byte in ", last_in[k], " and MISSING ",
              last_out[k], " of the four counts", crlf);
    }
    bench.verdict("the peer answered - the counts above name its target as losing the last "
                  "written byte before a late repeated START or not, and neither answer is "
                  "judged",
                  reports != 0u);
}

// ===========================================================================
// l..q - the subjects a second chip on the wire judges
// ===========================================================================
//
// Letters l to q run against the peer like every letter above: the
// chip's I2C2 reaches no wire on the desk this suite is written for (its
// pads PB10/PB11 carry a serial link there), so every far end below is
// the peer's - a target the core under test does not serve, polled by
// its own core, or a controller of its own.

/// THIS BOARD'S SECOND ADDRESS (dual addressing) for letter l: no letter
/// of the single-board half sends it, and it is neither the command
/// address nor one the peer takes.
constexpr uint8_t dut_second = 0x4B;
static_assert(dut_second != twilink::command_addr && dut_second != twilink::dut_addr &&
                  dut_second != twilink::low_addr && dut_second != twilink::deaf_addr &&
                  dut_second != twilink::shared_addr && dut_second != nobody_addr,
              "the second address is one nobody else on this bus answers");

/// What this board's POLLED target did while the peer's controller
/// addressed it: the bytes it took, the ones its shifter asked for, and
/// the events that framed them.
struct TargetLog {
    uint8_t in[16];
    uint8_t in_n;
    uint8_t served;
    uint8_t addressed;
    uint8_t reads;
    uint8_t second;
    uint8_t general;
    uint8_t stops;
    uint8_t nacks;
    uint8_t flushed;
};

/// The rendezvous of every peer host action (twi_link.hpp's `arb` and
/// `host_read`): the plain host gone, this board's target up at its own
/// address - and at the second one and the general call when asked - and
/// the bus made busy by hand and freed, the STOP on which the peer's
/// controller starts. Then the target is POLLED until `tenures` read or
/// write tenures have closed or the bound runs out. A read tenure gives
/// `seed`, `seed + 1`, ... from its first byte, the pattern the peer
/// checks.
TargetLog serve_peer_host(bool dual, uint8_t tenures, uint8_t seed, uint32_t bound_ms) {
    TargetLog log{};
    Host::release();
    client_live = true;
    (void)Client::init(clock,
                       dual ? I2cAddressConfig{.own = twilink::dut_addr,
                                               .second = dut_second,
                                               .general_call = true}
                            : I2cAddressConfig{.own = twilink::dut_addr},
                       {.no_stretch = false, .interrupts = false});
    hold_bus_busy(400);
    uint8_t pos = 0;
    uint8_t closed = 0;
    // A tenure is OPEN from its address match to its close - a read's
    // NACK, a write's STOP. The hand-made STOP above raised STOPF on this
    // target too (letter b) and closes nothing.
    bool open = false;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < bound_ms && closed < tenures) {
        if (Client::data_ready()) {
            const uint8_t v = Client::take();
            if (log.in_n < sizeof log.in) {
                log.in[log.in_n] = v;
            }
            ++log.in_n;
            continue;
        }
        if (Client::addressed()) {
            ++log.addressed;
            open = true;
            if (Client::answer_address()) {
                ++log.reads;
                pos = 0;
            }
            if (Client::second_address_matched()) {
                ++log.second;
            }
            if (Client::general_call_matched()) {
                ++log.general;
            }
            continue;
        }
        if (Client::host_nacked()) {
            // A read tenure's close: the byte the shifter asked for beyond
            // the NACK is dropped, and the peer's STOP follows.
            Client::clear_nack();
            ++log.nacks;
            if (Client::flush()) {
                ++log.flushed;
            }
            if (open) {
                ++closed;
                open = false;
            }
            continue;
        }
        if (Client::data_wanted() && log.reads > log.nacks) {
            Client::give(twilink::pattern_value(twilink::pattern_counting, seed, pos));
            ++pos;
            ++log.served;
            continue;
        }
        if (Client::stop_seen()) {
            Client::clear_stop();
            ++log.stops;
            if (open) {
                ++closed;   // a write tenure closes on its STOP
                open = false;
            }
            continue;
        }
        const uint16_t errs = static_cast<uint16_t>(H::status1() & i2c_errors & ~i2c_af);
        if (errs != 0u) {
            H::clear_errors(errs);
        }
    }
    Client::release();
    client_live = false;
    return log;
}

// ---------------------------------------------------------------------------
// l - this board's target under the peer's controller
// ---------------------------------------------------------------------------

void tl_target() {
    if (!need_peer()) {
        return;
    }

    // ---- THE SECOND ADDRESS AND THE GENERAL CALL, written by the peer ----
    const uint8_t w_seed = 0x21;
    struct Aim {
        uint8_t target;
        const char* name;
    };
    const Aim aims[2] = {{dut_second, "the second address"}, {0x00, "the general call"}};
    uint8_t hits[2] = {};
    bool bytes_ok[2] = {};
    for (uint8_t k = 0; k < 2u; ++k) {
        link_ready();
        twilink::Params a{};
        a.ms = 800;
        a.addr = twilink::dut_addr;
        a.target = aims[k].target;
        a.count = 2;
        a.seed = w_seed;
        a.aux16 = 400;   // time for this end to put its target up
        if (!peer_act(Op::arb, a)) {
            bench.verdict("the peer accepted the host command that addresses this board", false);
            return;
        }
        const TargetLog log = serve_peer_host(true, 1, 0, 900);
        hits[k] = k == 0u ? log.second : log.general;
        bytes_ok[k] = log.in_n == 2u && log.in[0] == w_seed &&
                      log.in[1] == static_cast<uint8_t>(w_seed + 1u) && log.addressed == 1u;
        print(serial, "  ", aims[k].name, " (", hex(aims[k].target), "): ", log.addressed,
              " match(es), DUALF ", log.second, ", GENCALL ", log.general, ", ", log.in_n,
              " byte(s) taken, ", log.stops, " STOP(s)", crlf);
        settle_ms(200);
    }
    host_ready();
    bench.verdict("the second address and the general call each open a write tenure on this "
                  "board's target from ANOTHER CHIP's controller, the bytes taken exact, and the "
                  "target says which one matched (DUALF, GENCALL)",
                  hits[0] == 1u && bytes_ok[0] && hits[1] == 1u && bytes_ok[1]);

    // ---- THE TARGET AS A TRANSMITTER, read by the peer's controller ----
    // The counts the peer's own receive procedure switches on, two reads
    // back to back each (twi_link.hpp's host_read).
    const uint8_t counts[5] = {1, 2, 3, 4, 8};
    uint8_t exact = 0;
    uint8_t over_all = 0;
    uint8_t flushed_all = 0;
    for (uint8_t c : counts) {
        link_ready();
        const uint8_t seed = static_cast<uint8_t>(0x60u + c);
        twilink::Params a{};
        a.ms = 800;
        a.addr = twilink::dut_addr;
        a.target = twilink::dut_addr;
        a.count = c;
        a.aux8 = 2;
        a.seed = seed;
        a.aux16 = 400;
        if (!peer_act(Op::host_read, a)) {
            print(serial, "  the peer refused host_read (a peer before the extended repertoire)",
                  crlf);
            break;
        }
        const TargetLog log = serve_peer_host(false, 2, seed, 900);
        settle_ms(200);
        host_ready();
        twilink::Report r{};
        const bool rep = peer_report(r);
        uint16_t want_sum = 0;
        for (uint8_t i = 0; i < c; ++i) {
            want_sum = static_cast<uint16_t>(
                want_sum + twilink::pattern_value(twilink::pattern_counting, seed, i));
        }
        want_sum = static_cast<uint16_t>(2u * want_sum);
        const bool ok = rep && r.mstatus == i2c_ok && r.addr_hits == 2u &&
                        r.count == static_cast<uint16_t>(2u * c) && r.sum == want_sum &&
                        r.first == seed && r.aux0 == seed;
        const uint8_t over = log.served > 2u * c ? static_cast<uint8_t>(log.served - 2u * c) : 0u;
        print(serial, "  read of ", c, " twice: the peer's host ", ok ? "exact" : "NOT exact",
              " (status ", r.mstatus, ", reads ", r.addr_hits, ", bytes ", r.count, ", sum ",
              hex(r.sum), " of ", hex(want_sum), "); this target served ", log.served,
              ", flushed ", log.flushed, " at ", log.nacks, " closing NACK(s)", crlf);
        if (ok) {
            ++exact;
        }
        over_all = static_cast<uint8_t>(over_all + over);
        flushed_all = static_cast<uint8_t>(flushed_all + log.flushed);
    }
    bench.verdict("THE TARGET AS A TRANSMITTER: another chip's controller read one, two, three, "
                  "four and eight bytes out of this board's data register, twice each, "
                  "byte-exact",
                  exact == 5u);
    bench.verdict("... and every byte the shifter asked for beyond the host's closing NACK was "
                  "dropped by flush()",
                  exact == 5u && flushed_all == over_all);
    bench.verdict("the command channel is back after the role swaps", command(Op::ping));
}

// ---------------------------------------------------------------------------
// n - a target stuck mid-byte, and unstick()
// ---------------------------------------------------------------------------

void tn_unstick() {
    if (!need_peer()) {
        return;
    }
    host_ready();
    const uint8_t clean = Host::unstick();
    bench.verdict("unstick() on a healthy bus clocks nothing and says so", clean == 0u);

    // A READ OF ZEROS, CUT OFF MID-BYTE: the peer's target serves 0x00 for
    // every byte, so it drives SDA low for every data bit, and this host
    // is taken off the bus through its reset line a few bit times into the
    // first byte - the one way this silicon lets a controller's pads go -
    // which leaves the far target holding SDA down with SCL released high.
    twilink::Params a{};
    a.ms = 400;
    a.addr = twilink::dut_addr;
    a.seed = 0x00;
    a.pattern = twilink::pattern_fixed;
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve the stuck target needs", false);
        return;
    }
    Host::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.tx_len = 0;
    r.rx = lend<Lease::reply>(rx_buf);
    r.rx_len = 4;
    r.speed = I2cSpeed::standard_100k;
    host_done = false;
    host_isr_entries = 0;
    (void)Host::start(r);
    // The event vector's SECOND entry is the address's ADDR, cleared there:
    // the first data byte starts as the far target answers its own ADDR,
    // and some four of its bits (10 us each at 100 kHz) go by before the cut.
    const uint32_t c0 = Ticker::cycles();
    while (host_isr_entries < 2u && Ticker::cycles() - c0 < SysClock::hz / 200u) {
    }
    const bool reached = host_isr_entries >= 2u;
    (void)delay_us(clock, 45);
    Pfic::disable(H::event_irq());
    Pfic::disable(H::error_irq());
    H::reset();
    (void)delay_us(clock, 20);
    const bool scl_high = SclPin::read();
    const bool sda_low = !SdaPin::read();
    print(serial, "  the host cut off ", reached ? "after its ADDR" : "BEFORE its ADDR",
          ": SCL ", scl_high ? "high" : "LOW", ", SDA ", sda_low ? "LOW - held" : "high", crlf);
    bench.verdict("ANOTHER CHIP's target holds SDA low mid-byte once its host is gone - the "
                  "stuck bus a reset controller leaves",
                  reached && scl_high && sda_low);

    // The remedy, its clocks COUNTED on the pad by TIM4.
    (void)Host::init(clock);
    (void)edges_arm();
    Meter::set_count(0);
    const uint8_t pulses = Host::unstick();
    const uint32_t edges = Meter::count();
    meter_off();
    const bool free_now = SclPin::read() && SdaPin::read();
    print(serial, "  unstick() reported ", pulses, " pulse(s) and TIM4 counted ", edges,
          " SCL rising edges on the pad; the bus is ", free_now ? "free" : "STILL HELD", crlf);
    bench.verdict("unstick() clocked the stuck target out - a handful of pulses, never "
                  "0xFF - and the timer counted them on the pad, plus the STOP's own rise",
                  pulses >= 1u && pulses <= 9u && edges >= pulses &&
                      edges <= static_cast<uint32_t>(pulses) + 2u && free_now);
    settle_ms(450);   // past the serve's deadline: the peer is back
    host_ready();
    twilink::Report pr{};
    const bool prep = peer_report(pr);
    print(serial, "  the peer's target: ", prep ? "reported" : "NO REPORT", ", served ", pr.aux1,
          ", ", pr.addr_hits, " address match(es), at the closing NACK STAR1-lo ",
          hex(pr.mstatus), crlf);

    // The bus is usable again: a read of four from the same far target.
    twilink::Params b{};
    b.ms = 300;   // the deadline alone ends it (arm_serve's reason)
    b.addr = twilink::dut_addr;
    b.seed = 0x71;
    bool exact = false;
    uint8_t after = no_answer;
    if (peer_act(Op::serve, b)) {
        for (uint8_t i = 0; i < 4u; ++i) {
            rx_buf[i] = 0xEE;
        }
        after = host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 4, link_speed);
        exact = after == i2c_ok;
        for (uint8_t i = 0; i < 4u && exact; ++i) {
            exact = rx_buf[i] == twilink::pattern_value(twilink::pattern_counting, 0x71, i);
        }
        settle_ms(350);
    }
    print(serial, "  the read of four after: status ", after, ", ", hex(rx_buf[0]), " ",
          hex(rx_buf[1]), " ", hex(rx_buf[2]), " ", hex(rx_buf[3]), crlf);
    bench.verdict("... and the same two chips carry a read of four byte-exact right after",
                  prep && exact);
}

// ---------------------------------------------------------------------------
// o - the refused last byte, at three core clocks
// ---------------------------------------------------------------------------

/// THE SLOWER CORES letter o also runs at: the PLL at 48 MHz (PB1 48, the
/// suite's own FREQ) and the HSI alone at 8 MHz (PB1 8; CTLR2.FREQ's
/// floor is 4 MHz, 19.12.2). The slower the core, the later the error
/// vector's answer to a NACK lands against what the controller does on
/// its own after it - which is the race this letter looks for.
using Core48 = Clock<ClockSource::pll, 48'000'000>;
using Core8 = Clock<ClockSource::internal, 8'000'000>;

/// What a run of refusals came to, over one core, one host, one speed.
struct RefusalTally {
    uint8_t refused;     ///< refusals tried
    uint8_t nack_data;   ///< answered i2c_nack_data
    uint8_t let_go;      ///< START and STOP down and MSL clear a millisecond on
    uint8_t next_ok;     ///< the probe that followed on the same host: i2c_ok
    uint8_t taken;       ///< refusals the far target reports as such (report_nacked)
    uint8_t acked_ok;    ///< the controls - every byte acknowledged - exact
    uint16_t first_bad_c1;   ///< CTLR1 and STAR2 of the first refusal not let go
    uint16_t first_bad_s2;
    uint8_t first_bad_st;
};

/// The refusing target's deadline: the action ends on it, since after a
/// refused byte the far target sees no Stop it was addressed in (19.12.6).
constexpr uint16_t refusal_ms = 40;

/// The far target armed for one write-then-read: `refuse` false
/// acknowledges all `n` written bytes and serves the two read, true
/// refuses the n-th - the last - written byte. No early exit: the
/// deadline alone ends the serve (arm_serve's reason).
bool arm_refusal(uint8_t n, bool refuse) {
    twilink::Params a{};
    a.ms = refusal_ms;
    a.addr = twilink::dut_addr;
    a.seed = 0x70;
    a.nack_at = refuse ? n : uint8_t{0};
    return peer_act(Op::serve, a);
}

/// One tenure through `HostT`, waited out on the tick for at most 20 ms.
template <typename HostT>
uint8_t refusal_run(uint8_t addr, uint8_t tx_len, uint8_t rx_len, I2cSpeed speed) {
    typename HostT::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx_buf);
    r.rx_len = rx_len;
    r.speed = speed;
    host_done = false;
    host_isr_entries = 0;
    if (HostT::start(r)) {
        return HostT::status();
    }
    const uint32_t t0 = Ticker::millis();
    while (!host_done && Ticker::millis() - t0 < 20u) {
    }
    return host_done ? HostT::status() : no_answer;
}

/// THE COMMAND CHANNEL IS THE PLAIN ENGINE'S: a letter measuring through
/// the engined one hands the instance over for each command and back.
template <typename HostT, typename Clk>
void measuring_host_up(const Clk& clk) {
    if constexpr (std::is_same_v<HostT, DmaHost>) {
        Host::release();
        dma_host_live = true;
        (void)DmaHost::init(clk);
    } else {
        dma_host_live = false;
        (void)Host::init(clk);
    }
}
template <typename HostT, typename Clk>
void command_host_up(const Clk& clk) {
    if constexpr (std::is_same_v<HostT, DmaHost>) {
        DmaHost::release();
        dma_host_live = false;
    }
    (void)Host::init(clk);
}

/// One to four bytes written then two read, at one speed through one
/// host: each count once acknowledged whole (the control) and four times
/// with its last written byte refused by the far target.
template <typename HostT, typename Clk>
RefusalTally refusal_point(const Clk& clk, I2cSpeed speed) {
    RefusalTally t{};
    for (uint8_t n = 1; n <= 4u; ++n) {
        command_host_up<HostT>(clk);
        if (arm_refusal(n, false)) {
            measuring_host_up<HostT>(clk);
            rx_buf[0] = 0xEE;
            rx_buf[1] = 0xEE;
            const uint8_t ack_st = refusal_run<HostT>(twilink::dut_addr, n, 2, speed);
            settle_ms(refusal_ms + 5u);
            command_host_up<HostT>(clk);
            twilink::Report r{};
            if (ack_st == i2c_ok && rx_buf[0] == 0x70u && rx_buf[1] == 0x71u &&
                peer_report(r) && r.aux0 == n) {
                ++t.acked_ok;
            }
        }
        for (uint8_t trial = 0; trial < 4u; ++trial) {
            command_host_up<HostT>(clk);
            if (!arm_refusal(n, true)) {
                continue;
            }
            measuring_host_up<HostT>(clk);
            const uint8_t st = refusal_run<HostT>(twilink::dut_addr, n, 2, speed);
            settle_ms(2);
            const uint16_t c1 = H::regs().CTLR1;
            const uint16_t s2 = H::status2();
            settle_ms(refusal_ms + 5u);   // past the far target's deadline
            // The next tenure on the same host: a probe of the command
            // address, which the far target answers again once its
            // deadline has put it back.
            const uint8_t next = refusal_run<HostT>(twilink::command_addr, 0, 0, speed);
            command_host_up<HostT>(clk);
            twilink::Report r{};
            const bool rep = peer_report(r);
            const bool gone = (c1 & (i2c_start | i2c_stop)) == 0u && (s2 & i2c_msl) == 0u;
            ++t.refused;
            t.nack_data = static_cast<uint8_t>(t.nack_data + (st == i2c_nack_data ? 1u : 0u));
            t.taken = static_cast<uint8_t>(
                t.taken + (rep && (r.flags & twilink::report_nacked) != 0u ? 1u : 0u));
            t.let_go = static_cast<uint8_t>(t.let_go + (gone ? 1u : 0u));
            t.next_ok = static_cast<uint8_t>(t.next_ok + (next == i2c_ok ? 1u : 0u));
            if ((!gone || next != i2c_ok) && t.first_bad_c1 == 0u) {
                t.first_bad_c1 = c1;
                t.first_bad_s2 = s2;
                t.first_bad_st = static_cast<uint8_t>(st << 4 | (next & 0x0Fu));
            }
            if (!gone || next != i2c_ok) {
                (void)Host::recover();
                (void)Host::unstick();
            }
        }
    }
    command_host_up<HostT>(clk);
    return t;
}

/// One host at one speed, its counts printed; true when every refusal
/// was answered i2c_nack_data with the bus let go and the next tenure
/// running, the far target reporting each refusal, and every control
/// exact.
template <typename HostT, typename Clk>
[[gnu::noinline]] bool refusal_report(const Clk& clk, const char* core, I2cSpeed speed) {
    constexpr bool engines = std::is_same_v<HostT, DmaHost>;
    const RefusalTally t = refusal_point<HostT>(clk, speed);
    print(serial, "  ", core, speed == I2cSpeed::fast_400k ? " 400 kHz " : " 100 kHz ",
          engines ? "engines" : "pump   ", ": acked exact ", t.acked_ok, "/4; refused ",
          t.refused, ": nack_data ", t.nack_data, " (the far target refused ", t.taken,
          "), let go ", t.let_go, ", next ok ", t.next_ok);
    if (t.first_bad_c1 != 0u) {
        print(serial, "; first stuck: status|next ", hex(t.first_bad_st), " CTLR1 ",
              hex(t.first_bad_c1), " STAR2 ", hex(t.first_bad_s2));
    }
    print(serial, crlf);
    console_drain();
    return t.acked_ok == 4u && t.refused == 16u && t.nack_data == t.refused &&
           t.taken == t.refused && t.let_go == t.refused && t.next_ok == t.refused;
}

/// THE RATE ON THE WIRE AT THIS CORE. A write of sixteen, every byte
/// acknowledged by the far target, through `HostT` at `speed`: SCL's mean
/// rate over its 153 periods (sixteen bytes and the address, nine clocks
/// each) on the core's counter, in thousandths of the rate asked - PB1 is
/// another rate at each core, and the core's counter is the one ruler here
/// that does not run on it. 0 when the far target did not take it whole.
template <typename HostT, typename Clk>
uint32_t refusal_rate(const Clk& clk, I2cSpeed speed) {
    for (uint8_t i = 0; i < 16u; ++i) {
        tx_buf[16u + i] = static_cast<uint8_t>(0x35u + 7u * i);
    }
    command_host_up<HostT>(clk);
    twilink::Params a{};
    a.ms = refusal_ms;
    a.addr = twilink::dut_addr;
    if (!peer_act(Op::serve, a)) {
        return 0;
    }
    measuring_host_up<HostT>(clk);
    typename HostT::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf + 16));
    r.tx_len = 16;
    r.speed = speed;
    host_done = false;
    const uint32_t c0 = Ticker::cycles();
    uint8_t st = no_answer;
    if (HostT::start(r)) {
        st = HostT::status();
    } else {
        while (!host_done && Ticker::cycles() - c0 < Clk::hz / 50u) {
        }
        st = host_done ? HostT::status() : no_answer;
    }
    const uint32_t took = Ticker::cycles() - c0;
    settle_ms(refusal_ms + 5u);
    command_host_up<HostT>(clk);
    twilink::Report pr{};
    const bool whole = st == i2c_ok && peer_report(pr) && pr.aux0 == 16u;
    return whole ? scl_permille(took, 9u * 17u, speed, Clk::hz) : 0u;
}

/// The longest service of a host vector, hardware entry included, in core
/// cycles: 164 for the event vector's longest entry and 165 for the error
/// one's, measured by bench_vx03's letter i (docs/ch32vx03/i2c.md), and a
/// margin for the entry itself. The image runs from the zero-wait window,
/// so a cycle count holds at every core clock.
constexpr uint32_t service_cycles = 180;

/**
 * THE BRACKET OF A TENURE'S MEAN RATE on the core's counter (17 bytes of
 * nine clocks: a write or a read of 16 and its address), DERIVED:
 *
 *  - the rate MADE is CCR's, `made_hz` (the engine's scl_hz()): the rate
 *    asked at 48 MHz of PB1, and at 8 MHz 100 kHz exact but 400 kHz CCR 7,
 *    381 kHz - rounded up, never fast; SCL and the counter share HCLK, so
 *    no clock error enters;
 *  - ABOVE: the interval may end half a period short of the 153 counted
 *    (the completion is the last byte's BTF, on the ninth clock's
 *    falling edge), so a reading is at most 153 / 152.5 of the rate made;
 *  - BELOW: the interval also holds the START's lead (SB a period and a
 *    half after START at 400 kHz, one at 100 - the engine's figures) and
 *    the completion's own half period, 2.5 periods, plus FOUR vector
 *    services that hold the bus or the completion back (SB, ADDR, the
 *    channel's or the last TxE's, BTF), each at most `service_cycles` of
 *    the core under test - 0.75 of a period at 400 kHz on a 96 MHz core,
 *    nine at 8 MHz. The far target, a polled core of its own at 96 MHz,
 *    takes or gives each byte within a byte time and stretches nothing it
 *    has not been told to. Anything slower is a clock someone held.
 */
bool mean_rate_ok(uint32_t permille, I2cSpeed speed, uint32_t core_hz, uint32_t made_hz) {
    const uint64_t asked = asked_hz(speed);
    // The rate made, in thousandths of the rate asked, times 1530: the
    // reading's scale.
    const uint64_t made = 1530ULL * 1000u * made_hz / asked;
    // The floor's denominator in tenths of a period.
    const uint64_t den10 = 1530u + 25u + (40ULL * service_cycles * made_hz) / core_hz;
    const bool above_ok = static_cast<uint64_t>(permille) * 1525u <= made;
    const bool below_ok = static_cast<uint64_t>(permille) * den10 >= made;
    return above_ok && below_ok;
}

template <typename Clk>
bool refusal_core(const char* core, bool& on_rate) {
    constexpr Clk clk;
    console_drain();
    const bool up = Clk::init();
    (void)Serial::init(clk, 115200);
    (void)Ticker::init(clk);
    print(serial, "  the core at ", core, " (", up ? "up" : "FAILED", "), PB1 ",
          Clk::pclk1_hz / 1'000'000u, " MHz", crlf);
    (void)Host::init(clk);
    const uint32_t pm100 = refusal_rate<Host>(clk, I2cSpeed::standard_100k);
    const uint32_t pm400 = refusal_rate<Host>(clk, I2cSpeed::fast_400k);
    const uint32_t dm100 = refusal_rate<DmaHost>(clk, I2cSpeed::standard_100k);
    const uint32_t dm400 = refusal_rate<DmaHost>(clk, I2cSpeed::fast_400k);
    print(serial, "  ", core, " a write of 16 in thousandths of the rate asked: the pump ", pm100,
          " at 100 kHz and ", pm400, " at 400 kHz, the engines ", dm100, " and ", dm400, crlf);
    (void)Host::init(clk);
    const uint32_t made100 = Host::scl_hz(I2cSpeed::standard_100k);
    const uint32_t made400 = Host::scl_hz(I2cSpeed::fast_400k);
    print(serial, "  ", core, " CCR makes ", made100, " Hz and ", made400, " Hz", crlf);
    on_rate = mean_rate_ok(pm100, I2cSpeed::standard_100k, Clk::hz, made100) &&
              mean_rate_ok(pm400, I2cSpeed::fast_400k, Clk::hz, made400) &&
              mean_rate_ok(dm100, I2cSpeed::standard_100k, Clk::hz, made100) &&
              mean_rate_ok(dm400, I2cSpeed::fast_400k, Clk::hz, made400);
    bool clean = up;
    clean = refusal_report<Host>(clk, core, I2cSpeed::standard_100k) && clean;
    clean = refusal_report<Host>(clk, core, I2cSpeed::fast_400k) && clean;
    clean = refusal_report<DmaHost>(clk, core, I2cSpeed::standard_100k) && clean;
    clean = refusal_report<DmaHost>(clk, core, I2cSpeed::fast_400k) && clean;
    command_host_up<Host>(clk);
    return clean;
}

void to_refusal() {
    if (!need_peer()) {
        return;
    }
    for (uint8_t i = 0; i < 4u; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0xC1u + i);
    }
    bool fast_rate = false;
    bool mid_rate = false;
    bool slow_rate = false;
    bool again_rate = false;
    const bool fast = refusal_core<SysClock>("96 MHz", fast_rate);
    const bool mid = refusal_core<Core48>("48 MHz", mid_rate);
    const bool slow = refusal_core<Core8>(" 8 MHz", slow_rate);
    (void)refusal_core<SysClock>("96 MHz again", again_rate);
    host_ready();
    bench.verdict("the last written byte refused by ANOTHER CHIP's target at 96 MHz: "
                  "i2c_nack_data, the bus let go, the next tenure ok",
                  fast);
    bench.verdict("... and at 48 MHz", mid);
    bench.verdict("... and at 8 MHz", slow);
    bench.verdict("AND SCL RUNS AT THE RATE ASKED AT EVERY CORE - PB1 at 48 and at 8 MHz: a "
                  "write of 16 through the pump and through the engines inside the bracket "
                  "derived above, on the core's counter",
                  fast_rate && mid_rate && slow_rate);
}

// ---------------------------------------------------------------------------
// p - the hand-over on channel 7; q - a channel that does not serve
// ---------------------------------------------------------------------------

/// THE OTHER OWNER OF CHANNEL 7. Table 11-5 wires USART2's transmit
/// request and I2C1's receive request to the same channel of DMA1 - an OR
/// - so USART2's transport with its transmit engine there is what letter
/// p hands the channel to and takes it from. USART2's pads (PA2, PA3) are
/// claimed by that transport for the letter and released at its end.
constexpr uint8_t shared_channel = DmaRequestOf<DmaRequest::usart2_tx>::channel;
static_assert(shared_channel == DmaRequestOf<DmaRequest::i2c1_rx>::channel,
              "table 11-5: USART2's transmit request and I2C1's receive request share a "
              "channel");
using Handed = Uart<2, P, 64, 64, UartFormat{}, DmaTxEngine<1, shared_channel>>;
using SharedCh = DmaChannel<1, shared_channel>;
using Ch6 = DmaChannel<1, DmaRequestOf<DmaRequest::i2c1_tx>::channel>;
/// True while channel 7's vector and USART2's are that transport's.
volatile bool handed_live = false;

/// A REQUEST HELD ON A CHANNEL BY A PERIPHERAL WHOSE CLOCK IS GATED - the
/// state a release that does not pulse its block's reset leaves
/// (docs/ch32vx03/dma.md, "A released requester") - made here at the
/// register level, on purpose, to stage a channel that does not serve.
/// Channel 7: USART2 with DMAT and an idle transmitter, its TXE request
/// raised at once, then the gate closed. Channel 6: TIM3's channel 1
/// compare with CC1DE (table 11-5's TIM3_CH1 row), one match, the counter
/// stopped and the gate closed. Neither touches a pad, and each is let go
/// by its block's reset line, which acts with the clock gated.
void hold_channel7() {
    using U = Usart<2>;
    U::bus_clock(true);
    U::reset();
    U::dma_transmit(true);
    U::transmitter(true);
    U::enable(true);
    (void)delay_us(clock, 20);
    U::bus_clock(false);
}
void free_channel7() { Usart<2>::reset(); }

using Holder6 = Tim<3>;
static_assert(DmaRequestOf<DmaRequest::tim3_ch1>::channel ==
                  DmaRequestOf<DmaRequest::i2c1_tx>::channel,
              "table 11-5: TIM3's channel 1 request and I2C1's transmit request share a "
              "channel");
void hold_channel6() {
    Holder6::init();
    (void)Holder6::configure(TimConfig{.prescaler = 0, .period = 0xFFFF});
    (void)Holder6::set_compare(0, 16);
    Holder6::interrupts(Holder6::compare_dma(0), true);
    Holder6::enable(true);
    (void)delay_us(clock, 20);
    Holder6::enable(false);
    Holder6::bus_clock(false);
}
void free_channel6() { Holder6::reset(); }

/// A run out through USART2's transmit engine, waited for: true when the
/// port drained it within `budget_ms`; `took` what it took in core cycles,
/// from the queue to tx_idle() - the last stop bit's end.
bool handed_run(const uint8_t* run, uint16_t n, uint32_t budget_ms, uint32_t& took) {
    const uint32_t c0 = Ticker::cycles();
    (void)Handed::write_bulk(std::span<const uint8_t>(run, n));
    const uint32_t t0 = Ticker::millis();
    while (!Handed::tx_idle() && Ticker::millis() - t0 < budget_ms) {
    }
    took = Ticker::cycles() - c0;
    return Handed::tx_idle();
}

/// A run's time in thousandths of its wire: `n` 8N1 frames at 115200,
/// the rate both of letter p's runs ask, and one more - the idle frame TE
/// sends before the first byte after an init (18.2), which both runs follow.
uint32_t handed_permille(uint32_t cycles, uint16_t n) {
    const uint64_t wire = 10ULL * (n + 1u) * SysClock::hz / 115200u;
    return wire == 0u ? 0u : static_cast<uint32_t>(1000ULL * cycles / wire);
}

/// What both of letter p's directions send through USART2.
constexpr uint8_t handed_note[] = "channel 7 is the OR of USART2's and I2C1's requests";
constexpr uint16_t handed_len = sizeof(handed_note) - 1u;

/// One DMA read of `n` from the far target, waited out.
uint8_t engined_read(uint8_t n, I2cSpeed speed, uint32_t& cycles) {
    for (uint8_t i = 0; i < n; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint32_t c0 = Ticker::cycles();
    const uint8_t st = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf, n, speed);
    cycles = Ticker::cycles() - c0;
    return st;
}

void tp_handover() {
    if (!need_peer()) {
        return;
    }

    // THE OUTGOING OWNER: USART2's transport with its transmit engine, a
    // run out through channel 7, then released. With DMAT set and the
    // transmitter idle its request stands on the channel the moment the
    // run is out - at init() already.
    host_ready();
    handed_live = true;
    const bool up = Handed::init(clock, 115200);
    uint32_t took = 0;
    const bool sent = handed_run(handed_note, handed_len, 20, took);
    Handed::release();
    handed_live = false;

    // THE INCOMING OWNER: I2C1's DMA host, a read of sixteen on the same
    // channel from the far target.
    const PeerServe s = arm_serve(0x27);
    dma_host_up();
    host_stormed = false;
    uint32_t rd_cycles = 0;
    const uint8_t rd = s.armed ? engined_read(16, I2cSpeed::fast_400k, rd_cycles) : no_answer;
    const uint32_t rd_pm = scl_permille(rd_cycles, 9u * 17u, I2cSpeed::fast_400k, SysClock::hz);
    bool rd_exact = rd == i2c_ok;
    for (uint8_t i = 0; i < 16u && rd_exact; ++i) {
        rd_exact = rx_buf[i] == twilink::pattern_value(twilink::pattern_counting, 0x27, i);
    }
    const uint16_t left7 = SharedCh::remaining();
    dma_host_down();
    PeerServe got = s;
    (void)collect_serve(got);
    print(serial, "  USART2's run of ", handed_len, sent ? " sent" : " NOT SENT",
          " and the port released; then I2C1's DMA read of 16 on channel ", shared_channel,
          ": status ", rd, rd_exact ? ", byte-exact" : ", NOT exact", ", the channel's count ",
          left7, host_stormed ? ", the event vector STORMED" : "", crlf);
    print(serial, "  in thousandths of the rate asked: USART2's run ",
          handed_permille(took, handed_len), " of its wire at 115200, the read's SCL ", rd_pm,
          " of 400 kHz", crlf);
    bench.verdict("THE HAND-OVER: USART2's transmit engine released, then I2C1's DMA read of "
                  "sixteen bytes from another chip on the same channel 7, byte-exact - the "
                  "port's release() left no request held on the channel's OR",
                  up && sent && rd_exact && left7 == 0u);
    bench.verdict("and both owners run at the rate asked: USART2's run in its wire time at "
                  "115200, -1 % to +3 %, and the read's SCL inside the mean-rate bracket "
                  "letter o derives",
                  handed_permille(took, handed_len) >= 990u &&
                      handed_permille(took, handed_len) <= 1030u &&
                      mean_rate_ok(rd_pm, I2cSpeed::fast_400k, SysClock::hz,
                                   Host::scl_hz(I2cSpeed::fast_400k)));

    // THE MIRROR: I2C1's read abandoned mid-block with RxNE standing under
    // DMAEN - its channel stopped under it, the far target serving on until
    // the data register and the shifter both hold a byte - and the host
    // released. I2C1's block is not touched again until USART2 has run, its
    // reset being what the question is about.
    Usart<2>::reset();
    const PeerServe m = arm_serve(0x51);
    dma_host_up();
    DmaHost::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.tx_len = 0;
    r.rx = lend<Lease::reply>(rx_buf);
    r.rx_len = 32;
    r.speed = I2cSpeed::standard_100k;
    host_done = false;
    host_isr_entries = 0;
    if (m.armed) {
        (void)DmaHost::start(r);
    }
    // A millisecond into the block - some ten bytes at 100 kHz - the channel
    // is stopped under it: the next byte lands in DATAR and the one after it
    // in the shifter, BTF holding the clock.
    settle_ms(1);
    DmaRxEngine<1, shared_channel>::stop();
    settle_ms(3);
    const uint16_t star1 = H::status1();
    const uint16_t ctlr2 = H::regs().CTLR2;
    const bool standing = (star1 & i2c_rxne) != 0u && (ctlr2 & i2c_dmaen) != 0u;
    dma_host_live = false;
    DmaHost::release();

    handed_live = true;
    const bool up2 = Handed::init(clock, 115200);
    uint32_t took2 = 0;
    const bool mirrored = handed_run(handed_note, handed_len, 20, took2);
    const uint16_t faults = Handed::dma_faults();
    Handed::release();
    handed_live = false;
    // The far target was left mid-byte by the released host: the wire is
    // clocked free before the command channel needs it.
    (void)Host::init(clock);
    (void)Host::unstick();
    settle_ms(350);   // past the serve's deadline: the peer is back
    host_ready();
    // 52 frames of ten bits at 115200 baud: 4.5 ms on the wire.
    const uint32_t pm2 = handed_permille(took2, handed_len);
    print(serial, "  I2C1's read stopped under its block (STAR1 ", hex(star1), ", CTLR2 ",
          hex(ctlr2), "), the host released; then USART2's run of ", handed_len, " on channel ",
          shared_channel, ": ", mirrored ? "out" : "NOT OUT", " in ", pm2,
          " thousandths of its wire time, the channel's count ", SharedCh::remaining(),
          ", faults ", faults, crlf);
    bench.verdict("THE MIRROR: I2C1's DMA read abandoned with RxNE standing under DMAEN and "
                  "the host released, then USART2's transmit engine on channel 7 sends its "
                  "whole run in its wire time at 115200, -1 % to +3 % on the core's counter - "
                  "the host's release() left no request held",
                  m.armed && standing && up2 && mirrored && pm2 >= 990u && pm2 <= 1030u &&
                      faults == 0u);
    bench.verdict("the command channel is back", command(Op::ping));
}

void tq_dead_channel() {
    if (!need_peer()) {
        return;
    }

    // A WRITE OVER A TRANSMIT CHANNEL THAT DOES NOT SERVE: TIM3's request
    // held on channel 6, the gate closed under it. The channel moves one
    // item on its enable and waits for that request to fall, which it never
    // does: BTF comes with the block's count standing.
    for (uint8_t i = 0; i < 32u; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x30u + i);
    }
    PeerServe w = arm_serve(0x00);
    hold_channel6();
    dma_host_up();
    const uint8_t wr = w.armed ? dma_tenure(twilink::dut_addr, tx_buf, 32, nullptr, 0,
                                            I2cSpeed::fast_400k)
                               : no_answer;
    const uint16_t left6 = Ch6::remaining();
    free_channel6();
    dma_host_down();
    (void)collect_serve(w);
    print(serial, "  a DMA write of 32 over channel 6 held by TIM3: status ", wr,
          wr == i2c_dma_fault ? " (i2c_dma_fault)" : "", ", the far target took ", w.received,
          ", the channel's count ", left6, crlf);
    bench.verdict("A WRITE OVER A CHANNEL THAT DOES NOT SERVE is answered i2c_dma_fault, its "
                  "STOP on the wire - never i2c_ok with the bytes missing: BTF with the "
                  "block's count standing is a stalled channel",
                  wr == i2c_dma_fault && w.received < 32u);
    PeerServe w2 = arm_serve(0x00);
    dma_host_up();
    const uint8_t wr2 = w2.armed ? dma_tenure(twilink::dut_addr, tx_buf, 32, nullptr, 0,
                                              I2cSpeed::fast_400k)
                                 : no_answer;
    dma_host_down();
    (void)collect_serve(w2);
    bench.verdict("... and with the held request let go by TIM3's reset line, the same "
                  "write lands whole",
                  wr2 == i2c_ok && w2.received == 32u);

    // A READ OVER A RECEIVE CHANNEL THAT DOES NOT SERVE: USART2's request
    // held on channel 7. RxNE and BTF stand with nothing to take them, and
    // the event vector must not be what hears them: the tenure is the
    // caller's bound to answer, and recover() - the verb the per-bus
    // timeout calls - puts the host back.
    PeerServe rs = arm_serve(0x63);
    hold_channel7();
    dma_host_up();
    host_stormed = false;
    uint8_t rd = no_answer;
    uint32_t entries = 0;
    if (rs.armed) {
        DmaHost::Request r{};
        r.addr = twilink::dut_addr;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
        r.tx_len = 0;
        r.rx = lend<Lease::reply>(rx_buf);
        r.rx_len = 16;
        r.speed = I2cSpeed::fast_400k;
        host_done = false;
        host_isr_entries = 0;
        if (DmaHost::start(r)) {
            rd = DmaHost::status();
        } else {
            const uint32_t t0 = Ticker::millis();
            while (!host_done && Ticker::millis() - t0 < 50u) {
            }
            rd = host_done ? DmaHost::status() : no_answer;
        }
        entries = host_isr_entries;
        if (rd == no_answer) {
            (void)DmaHost::recover();
        }
    }
    const bool stormed = host_stormed;
    print(serial, "  a DMA read of 16 over channel 7 held by USART2: status ", rd,
          rd == no_answer ? " (no answer: the caller's 50 ms bound)" : "", ", the event vector ",
          entries, " entries", stormed ? " - STORMED, silenced by the suite" : "",
          ", the channel's count ", SharedCh::remaining(), crlf);
    bench.verdict("A READ OVER A CHANNEL THAT DOES NOT SERVE leaves the thread running: the "
                  "event vector entered for the START and the address alone, no storm, "
                  "and the tenure answered by the caller's bound and recover()",
                  rd == no_answer && !stormed && entries <= 4u);

    // The far target, mid-byte when the host was reset, clocked free; the
    // channel freed; the same read again.
    (void)DmaHost::unstick();
    free_channel7();
    dma_host_down();
    (void)collect_serve(rs);
    PeerServe rs2 = arm_serve(0x63);
    dma_host_up();
    uint32_t cycles = 0;
    const uint8_t rd2 = rs2.armed ? engined_read(16, I2cSpeed::fast_400k, cycles) : no_answer;
    bool exact = rd2 == i2c_ok;
    for (uint8_t i = 0; i < 16u && exact; ++i) {
        exact = rx_buf[i] == twilink::pattern_value(twilink::pattern_counting, 0x63, i);
    }
    dma_host_down();
    (void)collect_serve(rs2);
    bench.verdict("... and with the held request let go by USART2's reset line, a read of "
                  "sixteen byte-exact",
                  exact);
    bench.verdict("the command channel is back", command(Op::ping));
}

// ===========================================================================
// the banner
// ===========================================================================

void banner() {
    print(serial, crlf, "test_vx03_i2c - the I2C of RM ch. 19 on I2C1 (PB6/PB7), on ",
          device::part_name, crlf,
          "  the bus goes to a peer board running `twi_peer` (command address ",
          hex(twilink::command_addr), ") - every letter but a and j skips when it does not "
          "answer",
          crlf, "  PB1 runs at ", SysClock::pclk1_hz / 1'000'000u,
          " MHz: CTLR2.FREQ cannot state more than 60 (19.12.2)", crlf,
          "  the wire now: SCL ", SclPin::read() ? "high" : "LOW", ", SDA ",
          SdaPin::read() ? "high" : "LOW", crlf);
    bench.menu();
}

}  // namespace

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_LEAF_VECTOR(usart1_handler) { (void)Serial::isr(); }

BRIO_CH32_VECTOR(i2c1_ev_handler) {
    host_isr_entries = host_isr_entries + 1u;
    if (host_isr_entries > host_isr_budget) {
        host_stormed = true;
        storms = storms + 1u;
        storm_s1 = H::status1();
        storm_s2 = H::status2();
        H::event_interrupt(false);
        H::buffer_interrupt(false);
        return;
    }
    if (client_live) {
        return;   // the client half is POLLED here: its enables are down
    }
    if (dma_host_live) {
        if (DmaHost::isr()) {
            host_done = true;
        }
        return;
    }
    // One body for the arbiter's tenures and the letters' own.
    if (Host::isr()) {
        if (bus_ao_live) {
            brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
        } else {
            host_done = true;
        }
    }
}

BRIO_CH32_VECTOR(i2c1_er_handler) {
    error_isr_entries = error_isr_entries + 1u;
    if (client_live) {
        return;
    }
    if (dma_host_live) {
        if (DmaHost::error_isr()) {
            host_done = true;
        }
        return;
    }
    // One body for the arbiter's tenures and the letters' own.
    if (Host::error_isr()) {
        if (bus_ao_live) {
            brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
        } else {
            host_done = true;
        }
    }
}


BRIO_CH32_VECTOR(dma1_channel6_handler) {
    if (dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
    }
}
BRIO_CH32_VECTOR(dma1_channel7_handler) {
    if (handed_live) {
        (void)Handed::dma_isr();
        return;
    }
    if (dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
    }
}
// USART2's: letter p's transport, its interrupt receiver idle.
BRIO_CH32_LEAF_VECTOR(usart2_handler) { (void)Handed::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the block: the reset values, the three timing registers, the enable "
                      "protection, the wire's rise time and SCL measured on its own pad",
                 ta_block);
    bench.letter('b', "the host with no answer, and the STUCK BUS with unstick()'s clocks "
                      "counted",
                 tb_no_answer);
    bench.letter('c', "THE PEER: the twi_link command channel, ident, ten round trips",
                 tc_peer_link);
    bench.letter('d', "the tenure shapes against the peer, the four receive procedures, the "
                      "general call",
                 td_shapes);
    bench.letter('e', "the vocabulary on the wire, and commanded stretching priced",
                 te_vocabulary);
    bench.letter('f', "the two speeds and both duty shapes against a second chip, each "
                      "timed on the wire",
                 tf_speeds);
    bench.letter('g', "THE DMA ENGINES on channels 6 and 7 against the peer", tg_dma);
    bench.letter('h', "THE KERNEL: I2cBus over I2cHost, the rejection, the votes, a held "
                      "clock answered by the per-bus timeout",
                 th_kernel);
    bench.letter('i', "ARBITRATION both ways, and THIS BOARD AS THE CLIENT of the peer's "
                      "controller",
                 ti_arbitration);
    bench.letter('j', "the flags, both vectors, the client's addresses and the PE cycle",
                 tj_flags);
    bench.letter('k', "a ten-second stress at fast mode with the counters at both ends",
                 tk_stress);
    bench.letter('l', "THIS BOARD'S TARGET under the peer's controller: the second address, the "
                      "general call, and the target as a TRANSMITTER",
                 tl_target);
    bench.letter('n', "a far target stuck mid-byte by a host taken off the bus, and unstick()'s "
                      "clocks counted",
                 tn_unstick);
    bench.letter('o', "THE REFUSED LAST BYTE from the peer's target, the core at 96, 48 and 8 MHz",
                 to_refusal);
    bench.letter('p', "THE HAND-OVER on channel 7: USART2 released, then I2C1's DMA read; "
                      "and the mirror",
                 tp_handover);
    bench.letter('q', "a channel that does not serve: a DMA write answered i2c_dma_fault, a "
                      "DMA read with the event vector quiet",
                 tq_dead_channel);
    bench.letter('r', "THE LATE REPEATED START against the peer: the START after BTF by hand, "
                      "the target's tally printed and nothing judged",
                 tr_late_start, false);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL96" : "FAILED",
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
