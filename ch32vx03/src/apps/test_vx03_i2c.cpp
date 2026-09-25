// test_vx03_i2c - the reference bench suite for the I2C chapter of the
// CH32V203 and the CH32V303: ch32vx03/i2c.hpp over RM ch. 19, the host
// engine under its two vectors, the client half addressed by a real
// controller, the DMA engines, and util/i2c_bus.hpp's arbiter with not
// one line changed. Letters a..k are both series'; letters l..n are the
// CH32V303 evaluation board's, where the chip's two controllers share one
// bus, registered on that series' parts and compiled out of every other
// image.
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
//  - A PEER BOARD running `twi_peer` (a Nucleo-F446RE; the peer's
//    ident says which), commanded IN BAND over the bus under test
//    through avrdx/src/apps/twi_link.hpp, included by relative path:
//    one source of truth for the wire format on every architecture.
//    Three wires, both boards at 3.3 V:
//
//      PB6 (SCL)  <->  the peer's SCL
//      PB7 (SDA)  <->  the peer's SDA
//      GND        <->  GND
//
//    THE PULL-UPS ARE NOT THIS BOARD'S. An open-drain pad has no pull
//    in any output mode on this family (the pin chapter), so nothing
//    here holds the bus up: the peer's two pads do, and letter a
//    measures what they are worth in nanoseconds of rise. The peer's
//    command-mode client answers ONE address (0x6B) with no general
//    call and no second address, which is why every wireless letter
//    can run with it attached.
//
//    Every peer letter asks for a ping first and SKIPS (with no
//    verdict claimed) when three command retries fail: a suite that
//    hangs on an absent instrument is worse than one that says so.
//
//  - ON THE CH32V303 EVALUATION BOARD, THE CHIP'S OWN I2C2. That board
//    wires I2C2's pads to I2C1's - PB10 to PB6, PB11 to PB7 - and puts a
//    4.7 kOhm pull-up on each line, so one chip's two controllers share
//    a bus: letters l..n make one the host and the other the target,
//    the target POLLED from the loop that waits for the host (its clock
//    stretching holds the bus while the loop comes round). The wires are
//    looked for before a byte moves, and a peer letter that finds the
//    bus pulled up by them and no peer answering declines instead of
//    failing.
//
// THE PADS. PB6 and PB7 - and on the CH32V303 PB10 and PB11, I2C2's -
// and nothing else. NEVER TOUCHED: PA9/PA10 (the console), PA13/PA14
// (the debug port), PA11/PA12 (the USB pads), PC14/PC15 and PD0/PD1
// (the crystals), PB10/PB11 on the CH32V203 (I2C2's pads, which carry
// another link on that bench), PB12..PB15 (the SPI link), PA0..PA8
// (other phases' straps) - and PB2, the LED, toggled per command as
// every suite of this target does.
//
// What is exercised, letter by letter:
//   a  THE BLOCK AND THE ARITHMETIC: the reset values, the three
//      timing registers at both speeds and both duties, the enable
//      protection measured, the wire's RISE TIME from the pad, and SCL
//      itself measured by the capture while the host probes
//   b  THE HOST WITH NO ANSWER: an absent address as i2c_nack_addr,
//      the peer's command address as i2c_ok, BUSY before and after,
//      then the STUCK BUS the peer makes and unstick() with its clocks
//      counted by the timer
//   c  THE PEER: the twi_link command channel, ident, ten round trips
//   d  the tenure shapes against the peer, the four receive
//      procedures, the general call, the repeated START counted at the
//      far end
//   e  the vocabulary on the wire (nack_addr from a deaf client,
//      nack_data at a commanded byte) and commanded stretching priced
//   f  the speeds against a second chip, each timed on the wire
//   g  THE DMA ENGINES on channels 6 and 7
//   h  THE KERNEL: I2cBus over I2cHost, the NACK in its place, the
//      rejection, both votes, a wedge the peer holds and the per-bus
//      timeout that answers it
//   i  ARBITRATION in both directions, and THIS BOARD AS THE CLIENT
//      of the peer's own controller
//   j  the flags, both vectors, the client's addresses and the PE
//      cycle that drops an unclocked byte
//   k  a ten-second stress with the error counters at both ends
// and on the CH32V303 alone, over the board's two wires:
//   l  THE SELF-LINK: I2C1 the host, I2C2 the target - the probe, an
//      absent address, a write, reads of one, two, three, four and
//      eight bytes (the host's receive procedures, and THE TARGET AS A
//      TRANSMITTER), a write-then-read, at 100 kHz and at 400 kHz in both
//      duty shapes; the second address and the general call; the DMA
//      host's write, read and write-then-read
//   m  the same shapes with the roles swapped: I2C2 the host
//   n  A TARGET STUCK MID-BYTE: the chip's own target left holding SDA
//      low by a host taken off the bus through its reset line, and
//      unstick()'s clocks counted on the pad by the timer
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
// fits the family's smallest chip") and every other part as one image.
// build: boards = v203c6,v203c8,v303vc
// build: groups = abcd,efgh,ijk
// build: monitor_speed = 115200

#include <stdint.h>

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
using Led = Pin<'B', 2>;

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

/// Whether this part is the one whose board wires I2C2 to I2C1 (letters
/// l..n below), and the question a peer letter asks when no peer
/// answered on a pulled-up bus.
constexpr bool self_link_part =
    device::device_class == DeviceClass::v30x_d8 && device::i2c_count >= 2u;
template <bool on = self_link_part>
bool self_link_present();

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
    // A bus pulled up by THIS board's resistors, on the wires to its own
    // I2C2, has no peer on it by construction: the letter declines.
    if (self_link_present()) {
        print(serial, "  SKIPPED, no verdict claimed: the pull-ups are this board's own, on the "
                      "wires to its own I2C2 (letters l..n) - no peer board is on this bus.",
              crlf);
        return false;
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
    for (uint8_t i = 0; i < 3u; ++i) {
        (void)Host::init(clock, rungs[i].duty);
        // A probe of an address nobody answers: nine SCL pulses, and the
        // last complete cycle is what the capture registers hold.
        const uint8_t st = host_tenure(nobody_addr, nullptr, 0, nullptr, 0, rungs[i].speed);
        const uint32_t period = SclMeter::period_ticks();
        const uint32_t high = SclMeter::width_ticks();
        const uint32_t low = period > high ? period - high : 0u;
        const uint32_t asked = Host::scl_hz(rungs[i].speed);
        const uint32_t seen = period != 0u ? SysClock::timclk1_hz / period : 0u;
        print(serial, "  ", rungs[i].name, ": asked ", asked / 1000u, " kHz, measured ",
              seen / 1000u, " kHz (period ", ticks_to_ns(period), " ns, high ",
              ticks_to_ns(high), " ns, low ", ticks_to_ns(low), " ns), probe=", st, crlf);
        if (period != 0u && high != 0u && st == i2c_nack_addr) {
            ++measured;
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
    settle_ms(450);
    const uint32_t idle_entries = host_isr_entries;
    print(serial, "  the event vector, idle, after the hand-made STOP: ", idle_entries,
          " entries, STAR1=", hex(H::status1()), crlf);
    bench.verdict("the STOPF the hand-made STOP leaves is taken once and cleared - no storm "
                  "on the event vector",
                  idle_entries <= 2u && storms == storms_before &&
                      (H::status1() & i2c_stopf) == 0u);
    bench.verdict("with the wire free again unstick() clocks nothing", Host::unstick() == 0u);
    bench.verdict("and the command channel is back", command(Op::ping));
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
    const uint8_t s2 = host_tenure(twilink::dut_addr, w8, 8, nullptr, 0, link_speed);
    const uint32_t stretched_ms = Ticker::millis() - t0;
    settle_ms(300);
    twilink::Report sr{};
    const bool srep = peer_report(sr);
    print(serial, "  8 x 8 bytes unstretched: ", base_ms,
          " ms; ONE 8-byte tenure at 2 ms per byte: ", stretched_ms, " ms; peer count=",
          sr.count, crlf);
    bench.verdict("the unstretched baseline tenures all complete i2c_ok", base_ok);
    bench.verdict("a client stretching every data byte by 2 ms stretches the WALL TIME the "
                  "model predicts (16 ms or more for 8 bytes) and the tenure still completes "
                  "i2c_ok - stretching is flow control and the controller simply waits",
                  s2 == i2c_ok && stretched_ms >= 16u && srep && sr.count >= 8u);
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
        const uint32_t seen = period != 0u ? SysClock::timclk1_hz / period : 0u;
        print(serial, "  ", rungs[i].name, ": asked ",
              Host::scl_hz(rungs[i].speed) / 1000u, " kHz, on the wire ", seen / 1000u,
              " kHz (high ", ticks_to_ns(high), " ns of ", ticks_to_ns(period),
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

/// ONE TENURE STARTED INTO A BUSY BUS, which is the only way two
/// controllers' STARTs can meet: a START set while the bus is busy is
/// HELD BY THE HARDWARE until the bus frees, so this board makes a START
/// condition by hand, waits for the peer to see BUSY and arm its own
/// START behind ours, sets ours, and then makes the STOP that frees the
/// bus - both STARTs leave together and the wired-AND decides. The
/// engine's own stuck-BUSY remedy cannot interfere: it is refused while
/// a line is low, which is exactly the state this arranges.
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
// The CH32V303's self-link: the chip's two controllers on one bus (l..n)
// ===========================================================================
//
// On the CH32V303 evaluation board I2C2's pads are wired to I2C1's -
// PB10 to PB6 (SCL) and PB11 to PB7 (SDA) - with a 4.7 kOhm pull-up on
// each line: ONE CHIP'S TWO CONTROLLERS ON ONE BUS, one the host and the
// other the target. On the CH32V203 board those two pads carry another
// link and are never touched, so these letters are registered on the
// CH32V303's parts alone - and there too the two wires are looked for
// before a byte moves. THE TARGET IS POLLED from the loop that waits for
// the host's tenure (I2cClientOptions::interrupts false): its clock
// stretching holds the bus while the loop comes round, which is what lets
// one core be both ends. Every name below hangs on the letters' template
// parameter, so a part without I2C2 forms none of it - its state included.

/// The target's addresses on the self-link: its own and its second
/// (dual addressing); the general call is the third.
constexpr uint8_t self_addr = 0x3A;
constexpr uint8_t self_second = 0x4B;

/// Drive one pad, read the other against the OPPOSITE pull; both left
/// floating afterwards.
template <typename Driver, typename Reader>
bool pads_linked() {
    Reader::input(PinPull::down);
    Driver::output(true);
    (void)delay_us(clock, 20);
    const bool high = Reader::read();
    Reader::input(PinPull::up);
    Driver::clear();
    (void)delay_us(clock, 20);
    const bool low = !Reader::read();
    Driver::release();
    Reader::release();
    return high && low;
}

/// What a POLLED target did: the bytes it took, the ones it was asked to
/// give - its shifter asks one ahead of the wire - and the events that
/// framed them.
struct TargetLog {
    uint8_t in[32];
    uint8_t in_n;
    uint8_t served;
    uint8_t addressed;
    uint8_t reads;
    uint8_t second;
    uint8_t general;
    uint8_t stops;
    uint8_t nacks;
    uint8_t flushed;
    uint8_t errors;
};

/// The tenure shapes one host and one polled target exchange, judged at
/// both ends.
struct SelfShapes {
    uint8_t probe;
    uint8_t absent;
    uint8_t write;
    bool write_exact;
    uint8_t reads_ok;      ///< of the five receive counts, byte-exact
    uint8_t combined;
    bool combined_exact;
    uint8_t served_over;   ///< bytes the target gave beyond what the host read
    uint8_t flushed;
    uint8_t addressed;
};

template <bool on>
struct Self {
    using H2 = I2c<on ? 2 : 2>;
    using Host2 = I2cHost<on ? 2 : 2>;
    using Client2 = I2cClient<on ? 2 : 2>;
    using Scl2 = Pin<'B', on ? 10 : 10>;
    using Sda2 = Pin<'B', on ? 11 : 11>;

    /// True while I2C2's vectors belong to letter m's host.
    static inline volatile bool host2_live = false;
    static inline TargetLog log{};
    /// What the target transmits: one byte per ask, a pattern from a seed
    /// that restarts with every read tenure.
    static inline uint8_t seed = 0x60;
    static inline uint8_t pos = 0;

    static uint8_t value(uint8_t s, uint8_t i) { return static_cast<uint8_t>(s + 0x1Du * i); }

    static void clear() {
        log = TargetLog{};
        pos = 0;
    }

    /// The two wires, each driven from I2C1's pad and read at I2C2's
    /// against the opposite pull. The board's pull-ups sit on the same
    /// net, so a push-pull high and a push-pull low are what decide.
    static bool wired() {
        const bool scl = pads_linked<SclPin, Scl2>();
        const bool sda = pads_linked<SdaPin, Sda2>();
        print(serial, "  the wires PB6-PB10 ", scl ? "in place" : "ABSENT", ", PB7-PB11 ",
              sda ? "in place" : "ABSENT", crlf);
        return scl && sda;
    }

    /// Both controllers back to their reset state with their gates shut,
    /// the four pads floating, every vector of the two silenced.
    static void all_released() {
        host2_live = false;
        client_live = false;
        dma_host_live = false;
        bus_ao_live = false;
        Pfic::disable(H::event_irq());
        Pfic::disable(H::error_irq());
        Pfic::disable(H2::event_irq());
        Pfic::disable(H2::error_irq());
        H::bus_clock(true);
        H::reset();
        H::bus_clock(false);
        H2::bus_clock(true);
        H2::reset();
        H2::bus_clock(false);
        SclPin::release();
        SdaPin::release();
        Scl2::release();
        Sda2::release();
    }

    /// One pass of the target's polled surface: the event the ISR body
    /// would report, acted on the way 19.4 prescribes - and the closing
    /// NACK of a read followed by flush(), which drops the byte the
    /// shifter asked for and the host never clocked.
    template <typename C>
    static void poll() {
        switch (C::service()) {
            case I2cClientEvent::addressed:
                ++log.addressed;
                if (C::host_reads()) {
                    ++log.reads;
                    pos = 0;
                }
                if (C::second_address_matched()) {
                    ++log.second;
                }
                if (C::general_call_matched()) {
                    ++log.general;
                }
                break;
            case I2cClientEvent::byte_received: {
                const uint8_t v = C::take();
                if (log.in_n < sizeof log.in) {
                    log.in[log.in_n] = v;
                }
                ++log.in_n;
                break;
            }
            case I2cClientEvent::byte_wanted:
                C::give(value(seed, pos));
                ++pos;
                ++log.served;
                break;
            case I2cClientEvent::stop:
                ++log.stops;
                break;
            default:
                break;
        }
        const I2cClientEvent err = C::error_service();
        if (err == I2cClientEvent::nacked) {
            ++log.nacks;
            if (C::flush()) {
                ++log.flushed;
            }
        } else if (err == I2cClientEvent::error) {
            ++log.errors;
        }
    }

    /// One tenure of `HostT` into a POLLED target `C`: started, then
    /// waited out with the target served on every pass, and the target
    /// polled a while longer so it sees the STOP or the NACK that closed
    /// the tenure.
    template <typename HostT, typename C>
    static uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx,
                          uint8_t rx_len, I2cSpeed speed) {
        typename HostT::Request r{};
        r.addr = addr;
        r.tx = lend<Lease::reply>(tx);
        r.tx_len = tx_len;
        r.rx = lend<Lease::reply>(rx);
        r.rx_len = rx_len;
        r.speed = speed;
        host_done = false;
        host_isr_entries = 0;
        uint8_t st = no_answer;
        if (HostT::start(r)) {
            st = HostT::status();
        } else {
            const uint32_t t0 = Ticker::millis();
            while (!host_done && Ticker::millis() - t0 < 50u) {
                poll<C>();
            }
            st = host_done ? HostT::status() : no_answer;
        }
        const uint32_t t1 = Ticker::millis();
        while (Ticker::millis() - t1 < 2u) {
            poll<C>();
        }
        if (st == no_answer) {
            print(serial, "    STALL: the host's tenure never answered; recovered", crlf);
            (void)HostT::recover();
        }
        return st;
    }

    template <typename HostT, typename C>
    static SelfShapes shapes(I2cSpeed speed) {
        SelfShapes s{};
        clear();
        s.probe = tenure<HostT, C>(self_addr, nullptr, 0, nullptr, 0, speed);
        s.absent = tenure<HostT, C>(nobody_addr, nullptr, 0, nullptr, 0, speed);

        // A write of eight: the target takes them all, then the STOP.
        for (uint8_t i = 0; i < 8u; ++i) {
            tx_buf[i] = static_cast<uint8_t>(0x11u * (i + 1u));
        }
        clear();
        s.write = tenure<HostT, C>(self_addr, tx_buf, 8, nullptr, 0, speed);
        bool same = log.in_n == 8u && log.stops >= 1u;
        for (uint8_t i = 0; i < 8u && same; ++i) {
            same = log.in[i] == tx_buf[i];
        }
        s.write_exact = same;

        // THE TARGET AS A TRANSMITTER, at the counts the host's receive
        // procedure switches on: one, two, three, four and eight bytes.
        const uint8_t counts[5] = {1, 2, 3, 4, 8};
        uint8_t over = 0;
        uint8_t flushed = 0;
        for (uint8_t c : counts) {
            clear();
            seed = static_cast<uint8_t>(0x60u + c);
            for (uint8_t i = 0; i < 8u; ++i) {
                rx_buf[i] = 0xEE;
            }
            const uint8_t st = tenure<HostT, C>(self_addr, nullptr, 0, rx_buf, c, speed);
            bool exact = st == i2c_ok;
            for (uint8_t i = 0; i < c && exact; ++i) {
                exact = rx_buf[i] == value(seed, i);
            }
            if (exact) {
                ++s.reads_ok;
            }
            over = static_cast<uint8_t>(over + (log.served > c ? log.served - c : 0u));
            flushed = static_cast<uint8_t>(flushed + log.flushed);
        }
        s.served_over = over;
        s.flushed = flushed;

        // Write-then-read: two bytes out, a repeated START, four back.
        clear();
        seed = 0x90;
        tx_buf[0] = 0x5A;
        tx_buf[1] = 0xA5;
        for (uint8_t i = 0; i < 4u; ++i) {
            rx_buf[i] = 0xEE;
        }
        s.combined = tenure<HostT, C>(self_addr, tx_buf, 2, rx_buf, 4, speed);
        bool cx = s.combined == i2c_ok && log.in_n == 2u && log.in[0] == 0x5Au &&
                  log.in[1] == 0xA5u;
        for (uint8_t i = 0; i < 4u && cx; ++i) {
            cx = rx_buf[i] == value(0x90, i);
        }
        s.combined_exact = cx;
        s.addressed = log.addressed;
        return s;
    }

    static bool shapes_exact(const SelfShapes& s) {
        return s.probe == i2c_ok && s.absent == i2c_nack_addr && s.write == i2c_ok &&
               s.write_exact && s.reads_ok == 5u && s.combined_exact;
    }

    static void print_shapes(const char* rung, const SelfShapes& s, uint32_t scl_asked) {
        print(serial, "    ", rung, ": probe=", s.probe, " absent=", s.absent, " write=",
              s.write, s.write_exact ? " (8 taken exactly)" : " (NOT exact)",
              "; reads of 1/2/3/4/8 exact: ", s.reads_ok, " of 5; write-then-read=",
              s.combined, s.combined_exact ? " exact" : " NOT exact", " with ", s.addressed,
              " address matches; SCL asked ", scl_asked / 1000u, " kHz", crlf);
        print(serial, "      the target gave ", s.served_over,
              " byte(s) beyond what the host read across the five reads, and flush() dropped ",
              s.flushed, crlf);
    }
};

/// The three rungs every self-link letter runs.
struct SelfRung {
    I2cSpeed speed;
    I2cDuty duty;
    const char* name;
};
constexpr SelfRung self_rungs[3] = {{I2cSpeed::standard_100k, I2cDuty::ratio_2, "100k      "},
                                    {I2cSpeed::fast_400k, I2cDuty::ratio_2, "400k d2   "},
                                    {I2cSpeed::fast_400k, I2cDuty::ratio_16_9, "400k d16/9"}};

// ---------------------------------------------------------------------------
// l - I2C1 the host, I2C2 the target
// ---------------------------------------------------------------------------

template <bool on = self_link_part>
void tl_self_link() {
    if constexpr (on) {
        using L = Self<on>;
        using C = typename L::Client2;
        L::all_released();
        if (!L::wired()) {
            bench.verdict("the self-link wants its two wires, and says so", true);
            return;
        }
        print(serial, "  the bus now: SCL ", SclPin::read() ? "high" : "LOW", ", SDA ",
              SdaPin::read() ? "high" : "LOW", " - the board's own pull-ups", crlf);

        // The target: I2C2 with its own address, a second one and the
        // general call, POLLED.
        const bool target_up =
            C::init(clock, {.own = self_addr, .second = self_second, .general_call = true},
                    {.no_stretch = false, .interrupts = false});

        uint8_t exact = 0;
        uint8_t over_all = 0;
        uint8_t flushed_all = 0;
        for (const SelfRung& g : self_rungs) {
            (void)Host::init(clock, g.duty);
            const SelfShapes s = L::template shapes<Host, C>(g.speed);
            L::print_shapes(g.name, s, Host::scl_hz(g.speed));
            console_drain();
            if (L::shapes_exact(s)) {
                ++exact;
            }
            over_all = static_cast<uint8_t>(over_all + s.served_over);
            flushed_all = static_cast<uint8_t>(flushed_all + s.flushed);
        }
        bench.verdict("I2C1 as the host and I2C2 as the target on the chip's own wires: the "
                      "probe, an absent address, an eight-byte write, reads of one, two, three, "
                      "four and eight bytes and a write-then-read, byte-exact at 100 kHz and at "
                      "400 kHz in both duty shapes",
                      target_up && exact == 3u);
        bench.verdict("THE TARGET AS A TRANSMITTER: every byte a read took came out of I2C2's "
                      "own data register, and every byte its shifter asked for beyond the "
                      "host's NACK was dropped by flush()",
                      target_up && exact == 3u && flushed_all == over_all);

        // The second address and the general call reach the target.
        (void)Host::init(clock);
        L::clear();
        const uint8_t w2[2] = {0x21, 0x43};
        const uint8_t st_second = L::template tenure<Host, C>(self_second, w2, 2, nullptr, 0,
                                                              I2cSpeed::standard_100k);
        const uint8_t second_hits = L::log.second;
        const bool second_bytes =
            L::log.in_n == 2u && L::log.in[0] == 0x21u && L::log.in[1] == 0x43u;
        L::clear();
        const uint8_t st_gc =
            L::template tenure<Host, C>(0x00, w2, 2, nullptr, 0, I2cSpeed::standard_100k);
        const uint8_t gc_hits = L::log.general;
        const bool gc_bytes = L::log.in_n == 2u;
        print(serial, "  the second address: ", st_second, " with ", second_hits,
              " DUALF match(es); the general call: ", st_gc, " with ", gc_hits,
              " GENCALL match(es)", crlf);
        bench.verdict("the second address and the general call each open a write tenure on the "
                      "target, which says which one matched",
                      st_second == i2c_ok && second_hits == 1u && second_bytes &&
                          st_gc == i2c_ok && gc_hits == 1u && gc_bytes);

        // The DMA host on channels 6 and 7 against the same target.
        C::release();
        (void)C::init(clock, {.own = self_addr}, {.no_stretch = false, .interrupts = false});
        Host::release();
        dma_host_live = true;
        (void)DmaHost::init(clock);
        for (uint8_t i = 0; i < 16u; ++i) {
            tx_buf[i] = static_cast<uint8_t>(0x80u + 3u * i);
            rx_buf[i] = 0xEE;
        }
        L::clear();
        L::seed = 0x33;
        const uint8_t dw = L::template tenure<DmaHost, C>(self_addr, tx_buf, 16, nullptr, 0,
                                                          I2cSpeed::fast_400k);
        bool dw_exact = L::log.in_n == 16u;
        for (uint8_t i = 0; i < 16u && dw_exact; ++i) {
            dw_exact = L::log.in[i] == tx_buf[i];
        }
        L::clear();
        const uint8_t dr = L::template tenure<DmaHost, C>(self_addr, nullptr, 0, rx_buf, 16,
                                                          I2cSpeed::fast_400k);
        bool dr_exact = dr == i2c_ok;
        for (uint8_t i = 0; i < 16u && dr_exact; ++i) {
            dr_exact = rx_buf[i] == L::value(0x33, i);
        }
        // And the two halves in one tenure: three bytes out on channel 6,
        // the repeated START, eight back on channel 7.
        L::clear();
        L::seed = 0x44;
        for (uint8_t i = 0; i < 8u; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint8_t dc = L::template tenure<DmaHost, C>(self_addr, tx_buf, 3, rx_buf, 8,
                                                          I2cSpeed::fast_400k);
        bool dc_exact = dc == i2c_ok && L::log.in_n == 3u;
        for (uint8_t i = 0; i < 3u && dc_exact; ++i) {
            dc_exact = L::log.in[i] == tx_buf[i];
        }
        for (uint8_t i = 0; i < 8u && dc_exact; ++i) {
            dc_exact = rx_buf[i] == L::value(0x44, i);
        }
        dma_host_live = false;
        DmaHost::release();
        print(serial, "  the DMA host at 400 kHz: a write of 16 ", dw,
              dw_exact ? " exact" : " NOT exact", ", a read of 16 ", dr,
              dr_exact ? " exact" : " NOT exact", ", a write of 3 then a read of 8 ", dc,
              dc_exact ? " exact" : " NOT exact", " (the target took ", L::log.in_n,
              "), unwedged ", DmaHost::unwedges(), " time(s)", crlf);
        bench.verdict("the DMA host - channels 6 and 7 - writes sixteen bytes into the chip's "
                      "own target, reads sixteen back, and carries a write-then-read whole, "
                      "byte-exact",
                      dw == i2c_ok && dw_exact && dr_exact && dc_exact);
        C::release();
        L::all_released();
        host_ready();
    }
}

// ---------------------------------------------------------------------------
// m - I2C2 the host, I2C1 the target
// ---------------------------------------------------------------------------

template <bool on = self_link_part>
void tm_self_swapped() {
    if constexpr (on) {
        using L = Self<on>;
        using H2 = typename L::Host2;
        L::all_released();
        if (!L::wired()) {
            bench.verdict("the swapped self-link wants its two wires, and says so", true);
            return;
        }
        client_live = true;   // I2C1's vectors stay quiet: its target is polled
        const bool target_up = Client::init(clock, {.own = self_addr},
                                            {.no_stretch = false, .interrupts = false});
        uint8_t exact = 0;
        for (const SelfRung& g : self_rungs) {
            L::host2_live = true;
            (void)H2::init(clock, g.duty);
            const SelfShapes s = L::template shapes<H2, Client>(g.speed);
            L::print_shapes(g.name, s, H2::scl_hz(g.speed));
            console_drain();
            if (L::shapes_exact(s)) {
                ++exact;
            }
        }
        bench.verdict("I2C2 ON A WIRE AS THE HOST, I2C1 as its target: the same shapes, "
                      "byte-exact at 100 kHz and at 400 kHz in both duty shapes",
                      target_up && exact == 3u);
        L::host2_live = false;
        H2::release();
        Client::release();
        L::all_released();
        host_ready();
    }
}

// ---------------------------------------------------------------------------
// n - a target stuck mid-byte, and unstick()
// ---------------------------------------------------------------------------

template <bool on = self_link_part>
void tn_unstick() {
    if constexpr (on) {
        using L = Self<on>;
        using C = typename L::Client2;
        L::all_released();
        if (!L::wired()) {
            bench.verdict("the stuck target wants the two wires, and says so", true);
            return;
        }
        (void)C::init(clock, {.own = self_addr}, {.no_stretch = false, .interrupts = false});
        (void)Host::init(clock);
        const uint8_t clean = Host::unstick();
        bench.verdict("unstick() on a healthy bus clocks nothing and says so", clean == 0u);

        // A READ OF ZEROS, CUT OFF MID-BYTE: the target drives SDA low for
        // every data bit, and the host is taken off the bus through its
        // reset line a few bit times into the first byte - the one way this
        // silicon lets a controller's pads go - which leaves the target
        // holding SDA down with SCL released high.
        L::clear();
        L::seed = 0x00;
        Host::Request r{};
        r.addr = self_addr;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
        r.tx_len = 0;
        r.rx = lend<Lease::reply>(rx_buf);
        r.rx_len = 4;
        r.speed = I2cSpeed::standard_100k;
        host_done = false;
        (void)Host::start(r);
        // The target served until its first byte is in the shifter; then
        // the host clocks some four of its bits (10 us each at 100 kHz).
        const uint32_t t0 = Ticker::millis();
        while (L::log.served == 0u && Ticker::millis() - t0 < 20u) {
            L::template poll<C>();
        }
        (void)delay_us(clock, 40);
        Pfic::disable(H::event_irq());
        Pfic::disable(H::error_irq());
        H::reset();
        (void)delay_us(clock, 20);
        const bool scl_high = SclPin::read();
        const bool sda_low = !SdaPin::read();
        print(serial, "  the host cut off ", L::log.served == 0u ? "BEFORE the byte" : "mid-byte",
              ": SCL ", scl_high ? "high" : "LOW", ", SDA ", sda_low ? "LOW - held" : "high",
              crlf);
        bench.verdict("the chip's own target holds SDA low mid-byte once its host is gone - a "
                      "stuck bus with no foreign chip on it",
                      L::log.served != 0u && scl_high && sda_low);

        // The remedy, its clocks COUNTED on the pad by TIM4.
        (void)Host::init(clock);
        (void)edges_arm();
        Meter::set_count(0);
        const uint8_t pulses = Host::unstick();
        const uint32_t edges = Meter::count();
        meter_off();
        // The target took the pulses as the rest of its byte and the ninth
        // clock as the host's NACK: its closing error is taken and the byte
        // its shifter asked for dropped, the way every read of letter l
        // ends.
        for (uint16_t i = 0; i < 200u; ++i) {
            L::template poll<C>();
        }
        const bool free_now = SclPin::read() && SdaPin::read();
        print(serial, "  unstick() reported ", pulses, " pulse(s) and TIM4 counted ", edges,
              " SCL rising edges on the pad; the bus is ", free_now ? "free" : "STILL HELD",
              "; the target saw ", L::log.nacks, " NACK(s)", crlf);
        bench.verdict("unstick() clocked the stuck target out - a handful of pulses, never "
                      "0xFF - and the timer counted them on the pad, plus the STOP's own rise",
                      pulses >= 1u && pulses <= 9u && edges >= pulses &&
                          edges <= static_cast<uint32_t>(pulses) + 2u && free_now);

        // The bus is usable again: a read of four, byte-exact.
        (void)Host::recover();
        L::clear();
        L::seed = 0x71;
        for (uint8_t i = 0; i < 4u; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint8_t after = L::template tenure<Host, C>(self_addr, nullptr, 0, rx_buf, 4,
                                                          I2cSpeed::standard_100k);
        bool exact = after == i2c_ok;
        for (uint8_t i = 0; i < 4u && exact; ++i) {
            exact = rx_buf[i] == L::value(0x71, i);
        }
        bench.verdict("... and the same two controllers carry a read of four byte-exact right "
                      "after",
                      exact);
        C::release();
        L::all_released();
        host_ready();
    }
}

/// The CH32V303's letters, registered where the part has the self-link.
template <bool on = self_link_part>
void register_self_link_letters() {
    if constexpr (on) {
        bench.letter('l', "THE SELF-LINK: I2C1 the host, I2C2 the target, on the chip's own "
                          "wires",
                     tl_self_link<>);
        bench.letter('m', "the self-link swapped: I2C2 the host, I2C1 the target",
                     tm_self_swapped<>);
        bench.letter('n', "a target stuck mid-byte, and unstick()'s clocks counted",
                     tn_unstick<>);
    }
}

/// Whether the wire's pull-ups are the self-link's - asked by a peer
/// letter whose peer did not answer, so it declines instead of failing.
template <bool on>
bool self_link_present() {
    if constexpr (on) {
        return Self<on>::wired();
    } else {
        return false;
    }
}

/// I2C2's two vectors: letter m's host, and nothing on any other part.
template <bool on = self_link_part>
void host2_event() {
    if constexpr (on) {
        if (Self<on>::host2_live && Self<on>::Host2::isr()) {
            host_done = true;
        }
    }
}
template <bool on = self_link_part>
void host2_error() {
    if constexpr (on) {
        if (Self<on>::host2_live && Self<on>::Host2::error_isr()) {
            host_done = true;
        }
    }
}

// ===========================================================================
// the banner
// ===========================================================================

void banner() {
    print(serial, crlf, "test_vx03_i2c - the I2C of RM ch. 19 on I2C1 (PB6/PB7), on ",
          device::part_name, crlf,
          "  the bus goes to a peer board running `twi_peer` (command address ",
          hex(twilink::command_addr), ") - letters b..k skip when it does not answer - or, "
          "on the CH32V303 evaluation board, to the chip's own I2C2 (letters l..n)",
          crlf, "  PB1 runs at ", SysClock::pclk1_hz / 1'000'000u,
          " MHz: CTLR2.FREQ cannot state more than 60 (19.12.2)", crlf,
          "  the wire now: SCL ", SclPin::read() ? "high" : "LOW", ", SDA ",
          SdaPin::read() ? "high" : "LOW", crlf);
    bench.menu();
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

extern "C" BRIO_CH32_INTERRUPT void i2c1_ev_handler() {
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
    if (bus_ao_live) {
        if (Host::isr()) {
            brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
        }
        return;
    }
    if (Host::isr()) {
        host_done = true;
    }
}

extern "C" BRIO_CH32_INTERRUPT void i2c1_er_handler() {
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
    if (bus_ao_live) {
        if (Host::error_isr()) {
            brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
        }
        return;
    }
    if (Host::error_isr()) {
        host_done = true;
    }
}

// I2C2's vectors: letter m's host on the CH32V303, empty on every other
// part.
extern "C" BRIO_CH32_INTERRUPT void i2c2_ev_handler() { host2_event<>(); }
extern "C" BRIO_CH32_INTERRUPT void i2c2_er_handler() { host2_error<>(); }

extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() {
    if (dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
    }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() {
    if (dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
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
    register_self_link_letters();

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
