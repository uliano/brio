// test_ch32_i2c - the reference bench suite for the CH32V00x's I2C
// chapter: ch32v00x/i2c.hpp over RM ch. 15, the host side against a
// PEER BOARD, the event machine under its two vectors, the DMA
// engines, and util/i2c_bus.hpp's arbiter with not one line changed.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// ONE INSTRUMENT: THE PEER BUS. This part has ONE I2C, so there is no
// self-link; every wire letter talks to a second board running
// `twi_peer` - on this desk the CH32X035F8U6 evaluation board (the peer
// names itself in its ident) - commanded IN BAND over the bus under test
// through avrxt/src/apps/twi_link.hpp, included by relative path: one
// source of truth for the wire format, whatever the peer's
// architecture. The peer's command-mode client answers ONE address
// (0x6B) with no general call and no second address.
//
// THE BUS IS I2C1 ON ITS COLUMN 3 on the CH32V006 (AFIO table 7-13-1,
// code 011), which leaves SPI1's column 0 whole; the CH32V003, whose
// remap table has no such column and which is off the desk, keeps its
// default pads (PC2 SCL, PC1 SDA). On the CH32V006:
//
//   SCL   PB5  <->  the CH32X035's PC16
//   SDA   PB6  <->  the CH32X035's PC17
//   GND   <->  GND, both boards at 3.3 V
//
// THE PULL-UPS ARE THE PEER'S: this module carries none, an
// alternate-function open-drain pad has no internal pull on this
// family, and the CH32X035 hangs a 1.5 kOhm resistor from each line to
// its PB3, which its I2C images drive high for their whole run and
// nothing drives at a reset. With no peer image up both lines read low
// and every wire letter declines with the reason - the lines read through
// their port's gate, which the suite opens before it asks (shut, the port
// reads zero, and right after a reset nothing else has opened it).
//
// THE HOST RUNS ON ITS TWO VECTORS (events on i2c1_ev, errors on
// i2c1_er); a letter that wants a tenure waited out spins on a flag
// the handler sets. Two I2cHost instantiations share the one instance
// - the plain one and the one on DMA channels 6 and 7 - and a flag
// says which owns the vectors, since two tasks over one peripheral
// share its registers and not their statics.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the reset values, the timing arithmetic -
//      and, with a peer on the wire, each line's RISE TIME from the
//      pad (a meter at rest cannot tell a pull-up from a leakage)
//      and the refusals, the enable protection MEASURED field by
//      field (what CKCFGR and FREQ do with PE set)
//   b  THE SCAN: every address 0x08..0x77 probed with the empty
//      request, the answers listed, nobody-home measured as
//      i2c_nack_addr - and the wire's pull-ups asked about first
//   c  THE PEER: the twi_link command channel, its ident, ten command
//      round trips - each one a write and a read of the engine under
//      test
//   d  the tenure shapes against the peer - write, read, write-then-
//      read with the repeated START counted from the far end, the
//      counted serve ended once its read is over, 0x7A and the general
//      call refused inside start() with nothing on the wire
//   e  the vocabulary against the peer (nack_addr from a deaf client,
//      nack_data at a commanded byte) and commanded stretching priced
//   f  the two speeds against a second chip, byte-exact both ways, each
//      write timed and its SCL judged against the rate asked: never above
//      it by more than 2 %, within a quarter below it
//   g  THE DMA ENGINES: the same shapes on channels 6 and 7 under
//      I2cEngine::dma, a read at 400 kHz besides, and the interrupts they
//      take - one completion a block, the write's handing its half back
//      to the event vector; the faults judged are the ones this run
//      counted; the SCL of the engined tenures timed and judged against
//      the rate asked
//   n  THE ENGINE A REQUEST NAMES (Request::engine): `dma` at one byte
//      each way and a 1 + 1, every half on the engines; `interrupt` at
//      sixteen touching no channel; `dma` on the engineless host refused with nothing on the wire; the
//      NACKs on the address and on data under `dma`
//   h  THE KERNEL against the peer: I2cBus (= BusMaster) over I2cHost,
//      the NACK in its place, the rejection, both votes, and two
//      wedges the PEER holds: SDA, which this silicon answers by itself
//      (a START into a held line is ARLO), and the CLOCK past the limit,
//      which only the per-bus timeout answers - then the recover()ed
//      engine carrying the next tenure
//   i  THE REFUSAL, wireless: a speed the clock cannot make is answered
//      i2c_rejected inside start() and delivered through the arbiter,
//      the wire and the vector untouched
//   j  TWO HOSTS ON ONE WIRE: arbitration decided in both directions
//      against the peer's controller, the losing round again under
//      I2cEngine::dma, a round repeated when the wire shows two tenures and
//      no race (race_missed()), the misses printed; a START set into a tenure the peer's
//      controller is clocking, raw and through the engine; start()'s watch of both
//      lines judged by requests posted into a tenure longer than its bound, at
//      100, 400 and 20 kHz, no reset taken; and THIS BOARD
//      AS THE CLIENT of that controller - written to (the one-byte-late
//      stretch measured) and read from (the client as a transmitter, the
//      byte it is asked beyond the closing NACK dropped by flush())
//   k  THE STUCK BUS: the peer holding SDA, unstick() counting the
//      clocks until it lets go, and the STOPF its hand-made STOP leaves
//      cleared with no entry of the event vector
//   p  THE DEAD CHANNEL, staged by a timer's compare request held on DMA
//      channel 6 or 7 with no wire: a write over it ending i2c_dma_fault, a
//      read over it never completing with its event vector unstormed, each
//      landing whole once the timer is reset; and the block's own request
//      left by a stalled read, let go four ways - release(), its gate
//      closed with no reset, leaving the next owner of channel 7 nothing
//   q  THE CONTROLLER'S OWN HABITS, raw, the vectors dark: how long BTF
//      stands under a STOP or a repeated START requested at EVT8_2; a
//      repeated START requested while a byte shifts, that byte refused -
//      generated all the same; and the withdrawal at that NACK the engine
//      declines, raced at both speeds
//   r  THE REPEATED START OF A WRITE-THEN-READ, requested on the TxE of
//      the last written byte: one to four bytes written then four read,
//      at both speeds, through the pump and the engines (both halves on
//      them, each block's completion counted), the peer counting
//      and summing what it took - then the peer REFUSING that last byte:
//      i2c_nack_data, no START or STOP left standing, MSL clear, and the
//      next tenure on the same host ending on the wire
//   v  THIS BOARD AS THE CLIENT FROM ITS VECTORS: letter j's two client
//      cases with the interrupt enables up, service() and error_service()
//      on i2c1_ev and i2c1_er - one entry per event, the body timed
//   l  THE FIRST BYTES AT THE ENTRY THAT CLEARS ADDR: TxE at ADDR, after its
//      clear and behind each store, raw, and two stores three ways against the
//      peer's tally; the host's writes of 1 to 4 and its 1 + 1 and 2 + 1 by
//      their entries; a refused last byte never i2c_ok; the first byte refused
//      with the second committed, and the next write answering for itself
//   u  A BUSY LEFT STANDING OVER AN IDLE WIRE: every way of making it tried and
//      counted, what takes it down, what a request meets raw and through the
//      arbiter - SWRST behind the watch of both lines, then the wire - with a
//      per-bus timeout and with none, and the way that takes no hand on the
//      wire: a client holding the clock past the per-bus timeout
//   t  THE TWO BOUNDS OF start() AGAINST THEIR NAMES: bus_free_us with BUSY held
//      and with BUSY standing over an idle wire, stop_drain_us run out by a
//      client holding the clock, at both speeds, start() answering i2c_timeout,
//      raw and through the arbiter with a per-bus timeout and with none, the next
//      request on the wire behind it; both bounds with the STK stopped, a count of
//      polls timed on a timer; and a raw START into a tenure at 100, 50 and 20
//      kHz, this block a bystander
//   y  THE ENGINES' CHANNELS STARVED: a memory-to-memory block at very_high above
//      them, up before the START and caught at the third byte, writes and reads:
//      the status, the far end's count, the event vector's entries
//
// With the peer attached `z` outlasts `brio run`'s default 60 s (the
// peer's command windows are hundreds of milliseconds each): pass
// `--timeout 400`.
//
// build: boards = v006k8,v003f4
// build: groups = ai,bcw,d,e,fk,g,q,r,l
// build: groups.v006k8 = abcdefgikrwv,hjpq,nlut,y
// (the kernel letter h, the two-host letters j and v, the dead-channel
// letter p, the engine letter n, the standing BUSY's letter u, the bounds'
// letter t and the starved channels' letter y are in no group of the CH32V003
// build: h, j, n and u each alone are over the part's 15 KB and their prose is
// not for shortening, v, p, t and y are born with the part's return to the desk
// - they run on the CH32V006.
// There the suite whole is over the 40 KB of program flash, so it builds
// as four images: the single-host wire letters with the client from its
// vectors; the kernel with the two-host, the dead-channel and the raw letters;
// the engine letter with the first bytes, the standing BUSY and the bounds; and
// the starved channels, each image left over 5 % of the flash)
// build: monitor_speed = 115200

#include <stdint.h>

#include <type_traits>
#include <variant>

#include "ch32v00x/afio.hpp"
#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/i2c.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/tim.hpp"
#include "ch32v00x/usart.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/post.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED (see the header comment).
#include "../../../avrxt/src/apps/twi_link.hpp"
#include "held_request.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;

/// THE BUS'S COLUMN (the header): column 3 on the CH32V006, the default
/// pads on the CH32V003.
constexpr I2cPins bus_pins =
    device::part == Ch32Part::v006 ? i2c1_pins_for(3) : i2c1_default_pins;
static_assert(device::part != Ch32Part::v006 ||
                  (bus_pins.scl == Pad{'B', 5} && bus_pins.sda == Pad{'B', 6}),
              "AFIO table 7-13-1: I2C1's code 011 is SCL PB5, SDA PB6");

using H = I2c<1>;
using Host = I2cHost<1, bus_pins>;
using DmaHost = I2cHost<1, bus_pins, DmaTxEngine<6>, DmaRxEngine<7>>;
using Client = I2cClient<1, bus_pins>;
using SclPin = Pin<bus_pins.scl.port, bus_pins.scl.pin>;
using SdaPin = Pin<bus_pins.sda.port, bus_pins.sda.pin>;

TestBench<Serial> bench;

/// An address nobody on this bus answers.
constexpr uint8_t nobody_addr = 0x23;

volatile bool host_done = false;
volatile bool dma_host_live = false;
/// Does this image carry a letter that brings the DMA host up (g, j, n, p,
/// r, u)? An image without one binds no DMA host in its vectors: on the
/// CH32V003 the host's engine paths are a few hundred bytes of its 15 KB.
constexpr bool dma_letters =
    selection_carries(test_letters, 'g') || selection_carries(test_letters, 'j') ||
    selection_carries(test_letters, 'n') || selection_carries(test_letters, 'p') ||
    selection_carries(test_letters, 'r') || selection_carries(test_letters, 'u') ||
    selection_carries(test_letters, 'y');
/// Does this image carry a letter that runs the arbiter (h, i, t, u)? An image
/// without one binds no arbiter in its vectors: the host's body inlined a
/// second time is a few hundred bytes of the CH32V003's 15 KB.
constexpr bool arbiter_letters = selection_carries(test_letters, 'h') || selection_carries(test_letters, 'i') ||
                                 selection_carries(test_letters, 't') || selection_carries(test_letters, 'u');
/// Does it carry a letter that runs the arbiter with no per-bus timeout (t, u)?
constexpr bool untimed_letters = selection_carries(test_letters, 't') || selection_carries(test_letters, 'u');
volatile uint32_t dma6_entries = 0;   // the transmit channel's vector
volatile uint32_t dma7_entries = 0;   // the receive channel's
volatile bool bus_ao_live = false;
/// The arbiter with no per-bus timeout, letters t and u, owns the host's completions.
volatile bool bus_ao_untimed = false;
/// Does this image carry the client half that answers the peer's long tenure
/// (letter j's watch)?
constexpr bool live_client_letters = selection_carries(test_letters, 'j');
/// That client half is up: the event vector serves it, ahead of the host.
volatile bool live_client = false;
volatile uint16_t live_count = 0;
volatile uint16_t live_sum = 0;
volatile uint32_t host_isr_entries = 0;
volatile uint32_t error_isr_entries = 0;
/// The host's storm budget: a vector that fires this many times inside
/// one tenure is silenced and the fact reported.
constexpr uint32_t host_isr_budget = 60'000;
volatile bool host_stormed = false;
/// What the storm guard saw when it last fired - wherever that was, in a
/// tenure or between two - so a STALL line can say why the vector was off.
volatile uint32_t storms = 0;
volatile uint16_t storm_s1 = 0;
volatile uint16_t storm_s2 = 0;

uint8_t tx_buf[64];
uint8_t rx_buf[64];

/// A tenure that never answered: the suite's own word, not a wire code.
constexpr uint8_t no_answer = 200;

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 500);
}

/// Put the plain host back where a letter can use it.
void host_ready() {
    dma_host_live = false;
    bus_ao_live = false;
    (void)Host::init(clock);
    if (H::busy()) {
        (void)Host::unstick();
        (void)Host::recover();
    }
}

/// One tenure through the TASK, waited out. Returns the status. The plain
/// host has no engines: `engine` is the interrupt unless a letter asks for
/// the refusal by name.
uint8_t host_tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx,
                    uint8_t rx_len, I2cSpeed speed, I2cEngine engine = I2cEngine::interrupt) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    r.engine = engine;
    host_done = false;
    host_isr_entries = 0;
    host_stormed = false;
    if (Host::start(r)) {
        return Host::status();
    }
    for (uint32_t i = 0; i < 600'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL: STAR1=", hex(H::status1()), " STAR2=", hex(H::status2()),
              " CTLR1=", hex(H::regs().CTLR1), " CTLR2=", hex(H::regs().CTLR2), " ev entries ",
              host_isr_entries, " er ", error_isr_entries, host_stormed ? " STORMED" : "",
              "; storms so far ", storms, " (last at STAR1=", hex(storm_s1), " STAR2=", hex(storm_s2),
              ")", crlf);
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
    host_isr_entries = 0;
    host_stormed = false;
    if (DmaHost::start(r)) {
        return DmaHost::status();
    }
    for (uint32_t i = 0; i < 600'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL (dma): STAR1=", hex(H::status1()), " STAR2=", hex(H::status2()),
              " ch6 flags=", hex(DmaChannel<6>::flags()), " ch7 flags=", hex(DmaChannel<7>::flags()),
              " ev entries ", host_isr_entries, crlf);
        (void)DmaHost::recover();
        return no_answer;
    }
    return DmaHost::status();
}

/// The wire has pull-ups: both lines read high with nothing driving.
/// THE BUS PORT'S GATE IS OPENED FIRST: a read through a shut gate answers
/// zero, and right after a reset nothing in this image has opened it -
/// the console is on port D, and only a letter that configures a pad
/// (the host's init(), the rise probe) opens the bus's port as a side
/// effect. A pad's mode is left alone, so the read is the same whether the
/// host owns the pads or nobody does.
bool wire_pulled_up() {
    SclPin::P::clock_on();
    SdaPin::P::clock_on();
    return SclPin::read() && SdaPin::read();
}

/// THE WIRE'S RISE TIME, in HCLK cycles: the pad pulled low as an
/// open-drain output for a while, released as an input, and the cycles
/// until it reads high counted - the pull-up's strength against the
/// wire's capacitance, which a meter at rest cannot tell (a 40 kOhm
/// internal pull reads 3.3 V too, and misses a 100 kHz bit). 0xFFFF =
/// never rose within the budget.
template <class P>
uint32_t rise_cycles() {
    P::output(false, PinDrive::open_drain);
    (void)delay_us(clock, 20);
    // The STK reloads every tick: the difference is taken modulo the
    // period, which holds for any rise under a millisecond.
    const uint32_t period = stk()->CMP + 1u;
    const uint32_t t0 = stk()->CNT;
    P::input(PinPull::none);
    uint32_t n = 0;
    while (!P::read() && n < 0xFFFFu) {
        ++n;
    }
    const uint32_t t1 = stk()->CNT;
    P::release();
    return n >= 0xFFFFu ? 0xFFFFu : (t1 >= t0 ? t1 - t0 : t1 + period - t0);
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
              ": the peer board must be running `twi_peer`; check the two SCL/SDA "
              "wires, the pull-ups and the GND.",
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
          "  THE PEER DID NOT ANSWER. The peer board must be running `twi_peer`; its "
          "console '0' forces the command-mode client back. Check the two wires in "
          "this file's header.",
          crlf);
    return false;
}

/// Every wire letter asks first and declines with the reason.
bool need_peer() {
    if (!wire_pulled_up()) {
        print(serial, "  SKIPPED, no verdict claimed: SCL/SDA read low - no pull-ups on the "
                      "wire, so no peer board is connected (this module carries none).",
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

// ---- the rate on the wire ---------------------------------------------------
//
// A rate asked of the host is judged on the wire: a tenure of `periods`
// SCL periods (nine a byte, the address's included) timed on the ticker's
// cycle count from start() to its completion, START, STOP and the
// software around them inside - which only lowers the reading. The
// bracket: never above the rate asked by more than 2 % (a divisor too
// small would be), and within a quarter below it: the pull-ups' rise
// (letter a measures it from the pad), the bytes' turnaround and the
// tenure's two ends all add to the run.

/// In 32 bits (the CH32V003 has no multiplier to spend on a 64-bit
/// division): `periods` x HCLK / 100 fits for a tenure of 89 bytes.
uint32_t scl_of(uint32_t cycles, uint32_t periods) {
    if (cycles == 0u) {
        return 0u;
    }
    const uint32_t q = periods * (SysClock::hz / 100u);
    return (q / cycles) * 100u + ((q % cycles) * 100u) / cycles;
}

bool scl_bracket(uint32_t measured, uint32_t asked) {
    return measured <= asked + asked / 50u && measured >= asked - asked / 4u;
}

// ===========================================================================
// a - the block, wireless
// ===========================================================================

void ta_block() {
    dma_host_live = false;
    bus_ao_live = false;
    Pfic::disable(H::event_irq());
    Pfic::disable(H::error_irq());
    H::bus_clock(true);
    H::reset();
    print(serial, "  reset: CTLR1=", hex(H::regs().CTLR1), " CTLR2=", hex(H::regs().CTLR2),
          " OADDR1=", hex(H::regs().OADDR1), " STAR1=", hex(H::status1()), " STAR2=",
          hex(H::status2()), " CKCFGR=", hex(H::regs().CKCFGR), crlf);
    bench.verdict("the reset values are table 15-1's (all zero)",
                  H::regs().CTLR1 == 0u && H::regs().CTLR2 == 0u && H::regs().CKCFGR == 0u &&
                      H::status1() == 0u);

    // The arithmetic at this clock.
    const auto sm = i2c_timing_for(SysClock::hz, I2cSpeed::standard_100k);
    const auto fm = i2c_timing_for(SysClock::hz, I2cSpeed::fast_400k);
    print(serial, "  100k: FREQ=", sm->freq_mhz, " CKCFGR=", hex(sm->ckcfgr), " -> ",
          i2c_scl_hz(SysClock::hz, *sm), " Hz; 400k: CKCFGR=", hex(fm->ckcfgr), " -> ",
          i2c_scl_hz(SysClock::hz, *fm), " Hz", crlf);
    bench.verdict("both speeds resolve at 48 MHz, exactly (CCR 240 and 40)",
                  sm->ckcfgr == 240u && fm->ckcfgr == (i2c_fs | 40u));
    bench.verdict("below 8 MHz nothing is legal (15.10.2's FREQ window)",
                  !i2c_timing_for(6'000'000, I2cSpeed::standard_100k).has_value());
    bench.verdict("the address refusals: 0x80 as a 7-bit address, a second address under "
                  "10-bit mode",
                  !i2c_address_config_valid({.own = 0x80}) &&
                      !i2c_address_config_valid({.own = 0x123, .ten_bit = true, .second = 0x10}));

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
          freq_pe, crlf);
    print(serial, "  -> ", (ck_pe == (i2c_fs | 40u)) ? "CKCFGR TAKES a write under PE (no lock)"
                                                       : "CKCFGR is LOCKED under PE",
          "; ", (freq_pe == 24u) ? "FREQ takes it too" : "FREQ is locked", crlf);
    bench.verdict("the enable protection was measured and reported (either answer is a "
                  "finding)",
                  true);
    H::disable();

    // The own address with bit 14 kept, the second address, the general call.
    (void)H::addresses({.own = 0x48, .second = 0x49, .general_call = true});
    print(serial, "  OADDR1=", hex(H::regs().OADDR1), " OADDR2=", hex(H::regs().OADDR2),
          " CTLR1=", hex(H::regs().CTLR1), crlf);
    bench.verdict("the own address lands shifted, the second under ENDUAL, the general call "
                  "in CTLR1",
                  H::regs().OADDR1 == (0x48u << 1) && H::regs().OADDR2 == (i2c_endual | (0x49u << 1)) &&
                      (H::regs().CTLR1 & i2c_engc) != 0u);
    // The F1 lineage's "bit 14 kept at one" is not this register's:
    // written, it reads back as the reserved bit it is.
    H::regs().OADDR1 = static_cast<uint16_t>((1u << 14) | (0x48u << 1));
    print(serial, "  OADDR1 with bit 14 written: reads ", hex(H::regs().OADDR1), crlf);
    bench.verdict("OADDR1's bit 14 is reserved here (reads zero), not the F1's fixed one",
                  (H::regs().OADDR1 & (1u << 14)) == 0u);
    H::reset();
    print(serial, "  pull-ups on the wire: ", wire_pulled_up() ? "yes (a peer is connected)"
                                                                 : "NO - both lines low",
          crlf);
    if (wire_pulled_up()) {
        const uint32_t scl = rise_cycles<SclPin>();
        const uint32_t sda = rise_cycles<SdaPin>();
        print(serial, "  rise from low, released: SCL ", scl, " cycles (", scl / (SysClock::hz / 1'000'000u),
              " us), SDA ", sda, " cycles (", sda / (SysClock::hz / 1'000'000u), " us) - a 100 kHz bit is 10 us",
              crlf);
    }
}

// ===========================================================================
// w - the wire probe (outside z)
// ===========================================================================

/// Each line held low by this board for half a second, then released:
/// the other end's own reading says whether the wire reaches it - a
/// peer's console, a meter, a scope. No verdict.
void tw_wire_probe() {
    print(serial, "  holding SDA low for 500 ms...", crlf);
    console_drain();
    SdaPin::output(false, PinDrive::open_drain);
    settle_ms(500);
    SdaPin::release();
    print(serial, "  released; holding SCL low for 500 ms...", crlf);
    console_drain();
    SclPin::output(false, PinDrive::open_drain);
    settle_ms(500);
    SclPin::release();
    print(serial, "  released. Rise after each: SDA ", rise_cycles<SdaPin>(), " cycles, SCL ", rise_cycles<SclPin>(),
          " cycles (a probe: no verdict)", crlf);
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
    if (others != 0u) {
        // What "other" was, for the record: the last probe's status and
        // the vectors' entry counts (i2c_bus_error = 4, i2c_arb_lost = 5,
        // i2c_timeout = 6, i2c_rejected = 7 in util/i2c_bus.hpp's order).
        const uint8_t st = host_tenure(0x77, nullptr, 0, nullptr, 0, I2cSpeed::standard_100k);
        print(serial, "    the last probe's status ", st, " (ok ", i2c_ok, " nack_addr ", i2c_nack_addr,
              " bus_error ", i2c_bus_error, " arb_lost ", i2c_arb_lost, " timeout ", i2c_timeout, "), ev entries ",
              host_isr_entries, " er ", error_isr_entries, " STAR1=", hex(H::status1()), " STAR2=",
              hex(H::status2()), " CTLR1=", hex(H::regs().CTLR1), " PCFR1=", hex(afio_pcfr1()), crlf);
    }
    for (uint8_t i = 0; i < n_found && i < 8u; ++i) {
        print(serial, "    ", hex(found[i]), (found[i] == twilink::command_addr)
                                                 ? "  <- twi_peer's command address"
                                                 : "",
              crlf);
    }
    bench.verdict("every probe ended one way or the other (no stall, no other status)",
                  others == 0u);
    bench.verdict("an address nobody answers is i2c_nack_addr - the scanner's probe result",
                  nacks >= 100u);
    bench.verdict("the peer's command address answered (twi_peer in command mode)",
                  n_found >= 1u);
}

// ===========================================================================
// c - the peer link
// ===========================================================================

void tc_peer_link() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping over the two wires - every command is TWO "
                  "TENURES of the engine under test",
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
              " fw=", hex(id.version), " (kind ", hex(twilink::peer_kind(id.version)),
              twilink::peer_kind(id.version) == twilink::peer_ch32x035 ? ", the CH32X035's)" : ")",
              crlf);
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
    twilink::Counters k{};
    const bool kc = peer_counters(k);
    print(serial, "  write=", ws, " read=", rs, " mism=", rmism, " combined=", cs,
          "; peer: count=", r.count, " addr_hits=", r.addr_hits, " first=", hex(r.first),
          " sum=", hex(r.sum), "; the peer's target dropped ", k.drops, " byte(s) at the closing "
          "NACKs of the action (", kc ? "counters answered" : "NO counters", ")", crlf);
    bench.verdict("a write tenure, a read tenure and the combined write-then-read all complete "
                  "i2c_ok against a SECOND CHIP",
                  ws == i2c_ok && rs == i2c_ok && cs == i2c_ok);
    bench.verdict("the bytes read back are the peer's own pattern, byte-exact", rmism == 0u);
    bench.verdict("the peer accounts every byte of the three tenures (8 written, 8 served, "
                  "then 4 and 4)",
                  rep && r.count == 24u);
    bench.verdict("and the COMBINED tenure hit the client's address machinery TWICE - the "
                  "repeated START, counted from the far end",
                  rep && r.addr_hits == 4u);
    if (kc) {
        bench.verdict("THE TARGET ASKED ONE BYTE MORE: each of the two read tenures left one byte in "
                      "the peer's DATAR past this host's closing NACK, and the peer's flush() dropped "
                      "it (its counters) - the bytes read above are exact all the same",
                      k.drops == 2u && rmism == 0u);
    } else {
        print(serial, "  (the peer does not serve the counters query: the drops are not judged)", crlf);
    }

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
        print(serial, "  read of ", n, ": status=", st, " mism=", mism, " peer count=", cr.count,
              crlf);
        if (st != i2c_ok || mism != 0u || !crep || cr.count != n) {
            counts_ok = false;
        }
    }
    bench.verdict("reads of ONE, TWO, THREE and FOUR bytes - the chapter's four receive "
                  "procedures - each byte-exact and each closed at the right byte (the "
                  "peer counts no extra)",
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
    // refused by start() itself - i2c_rejected is that refusal's code
    // alone, a tenure on the wire ending with one of the wire's - against
    // a peer at dut_addr ANSWERING THE GENERAL CALL, so a byte of either
    // on the wire would be an address match on its count; the write to
    // dut_addr after them is its one match and its two bytes. Through
    // host_tenure(), which a CH32V003 image has room for where two more
    // inlined start()s it has not.
    twilink::Params g{};
    g.count = 64;
    g.ms = 250;
    g.addr = twilink::dut_addr;
    g.flags = twilink::flag_general_call;
    if (peer_act(Op::serve, g)) {
        uint8_t gw[2] = {0x5A, 0xA5};
        const uint8_t st_prefix = host_tenure(0x7A, gw, 2, nullptr, 0, link_speed);
        const uint8_t st_call = host_tenure(0x00, gw, 2, nullptr, 0, link_speed);
        const uint8_t own = host_tenure(twilink::dut_addr, gw, 2, nullptr, 0, link_speed);
        settle_ms(300);
        twilink::Report gr{};
        const bool grep = peer_report(gr);
        print(serial, "  0x7A ", st_prefix, ", 0x00 ", st_call, ", then dut_addr ", own, "; peer: addr_hits=",
              gr.addr_hits, " count=", gr.count, " last_addr=", hex(gr.last_addr), crlf);
        bench.verdict("0x7A and the general call answered i2c_rejected, nothing on the wire: the peer, "
                      "answering the general call, took only the write to its own address",
                      st_prefix == i2c_rejected && st_call == i2c_rejected && own == i2c_ok && grep &&
                          gr.addr_hits == 1u && gr.count == 2u && gr.last_addr == twilink::dut_addr);
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
                  "otherwise alive",
                  deaf == i2c_nack_addr && probe == i2c_nack_addr);
    bench.verdict("a commanded NACK on the 3rd data byte reports i2c_nack_data - the wire-level "
                  "vocabulary is REAL statuses across two architectures",
                  nack == i2c_nack_data && rep && (r.flags & twilink::report_nacked) != 0u);

    twilink::Params base{};
    base.count = 128;
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
    const uint32_t c0 = Ticker::cycles();
    const uint8_t s2 = host_tenure(twilink::dut_addr, w8, 8, nullptr, 0, link_speed);
    const uint32_t stretched = Ticker::cycles() - c0;
    settle_ms(300);
    twilink::Report sr{};
    const bool srep = peer_report(sr);
    // A CH32 TARGET STRETCHES ONE BYTE LATE (the peer's header): a byte left
    // in its DATAR stalls the bus only once the NEXT one has been taken into
    // the shifter and acknowledged, so of an 8-byte write the address's hold
    // and those of the first six bytes are on the wire, and the last two
    // fall after the tenure: seven holds of 2 ms. A peer of another lineage
    // that stretches at every byte would put eight.
    const uint32_t hold = 2u * (SysClock::hz / 1000u);
    const uint32_t holds = (stretched + hold / 2u) / hold;
    print(serial, "  8 x 8 bytes unstretched: ", base_ms, " ms; ONE 8-byte tenure at 2 ms per byte: ",
          stretched / (SysClock::hz / 1000u), " ms = ", holds, " holds of 2 ms on the wire; peer count=",
          sr.count, crlf);
    bench.verdict("the unstretched baseline tenures all complete i2c_ok", base_ok);
    bench.verdict("a client stretching every data byte by 2 ms stretches the WALL TIME by the seven "
                  "holds a target one byte late puts on the wire, and the tenure still completes "
                  "i2c_ok - stretching is flow control",
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
        twilink::Params a{};
        a.count = 64;
        a.ms = 250;
        a.addr = twilink::dut_addr;
        a.seed = static_cast<uint8_t>(0x50u + i * 0x10u);
        a.pattern = twilink::pattern_counting;
        if (!peer_act(Op::serve, a)) {
            bench.verdict("the peer accepted the serve for this rung", false);
            return;
        }
        for (uint8_t k = 0; k < 8; ++k) {
            tx_buf[k] = static_cast<uint8_t>(0xC0u + k);
            rx_buf[k] = 0xEE;
        }
        // The tenure timed on the ticker's cycle count: 9 x (N + 1) SCL
        // periods plus a START and a STOP, so the period is what the
        // chooser predicts when the client does not stretch.
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
        // 8 data bytes + the address: 81 SCL periods, plus START/STOP.
        const uint32_t scl_hz_measured = scl_of(cycles, 81u);
        const uint32_t asked = i2c_speed_hz(rungs[i].speed);
        print(serial, "  ", rungs[i].name, ": SCL asked ", asked / 1000u, " kHz (the chooser's ",
              Host::scl_hz(rungs[i].speed) / 1000u, "), an 8-byte write took ", cycles,
              " cycles -> about ", scl_hz_measured / 1000u, " kHz on the wire; write=", ws, " read=", rs,
              " mism=", mism, " -> ", ok ? "byte-exact both ways" : "NOT exact", crlf);
        if (ok) {
            ++exact;
        }
        if (ws == i2c_ok && scl_bracket(scl_hz_measured, asked)) {
            ++bracketed;
        }
    }
    bench.verdict("Standard mode and Fast mode both carry a write and a read byte-exact between "
                  "TWO SEPARATE CHIPS",
                  exact == 2u);
    bench.verdict("and each runs the bus at the rate asked: an 8-byte write's SCL never above 100 or "
                  "400 kHz by more than 2 % and within a quarter below it (the pull-ups' rise, the "
                  "turnaround between bytes)",
                  bracketed == 2u);
    bench.verdict("the command channel survives the ladder", command(Op::ping));
}

// ===========================================================================
// g - the DMA engines against the peer
// ===========================================================================

void tg_dma() {
    if (!need_peer()) {
        return;
    }
    twilink::Params a{};
    a.count = 128;
    a.ms = 400;
    a.addr = twilink::dut_addr;
    a.seed = 0x60;
    a.pattern = twilink::pattern_counting;
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve", false);
        return;
    }
    // The DMA host over the same instance: the plain host's vectors are
    // handed over by the flag.
    dma_host_live = true;
    (void)DmaHost::init(clock);
    for (uint8_t i = 0; i < 16; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x80u + i);
        rx_buf[i] = 0xEE;
    }
    dma6_entries = 0;
    dma7_entries = 0;
    // What THIS run counts: the engines' fault counters are the image's,
    // and other letters (r's refusals through the engines) add to them.
    const uint32_t tx_faults0 = DmaTxEngine<6>::faults();
    const uint32_t rx_faults0 = DmaRxEngine<7>::faults();
    // The write and the read timed on the ticker's cycle count, 153 SCL
    // periods each (seventeen bytes, the address's included).
    console_drain();
    uint32_t c0 = Ticker::cycles();
    const uint8_t ws = dma_tenure(twilink::dut_addr, tx_buf, 16, nullptr, 0, link_speed);
    const uint32_t w_scl = scl_of(Ticker::cycles() - c0, 153u);
    c0 = Ticker::cycles();
    const uint8_t rs = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 16, link_speed);
    const uint32_t r_scl = scl_of(Ticker::cycles() - c0, 153u);
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
    // And at 400 kHz: a 16-byte read on the engines, timed, its bytes the
    // serve's next sixteen (the pattern runs on across the reads: 16 + 4 +
    // 2 + 1 taken before it).
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
    dma_host_live = false;
    // The plain host is the command channel's, and the DMA host's
    // release() left the instance off: it is brought back before the
    // report is asked for.
    host_ready();
    settle_ms(450);
    twilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  DMA: write16=", ws, " read16=", rs, " mism=", mism, " combined=", cs,
          " read2=", r2, " read1=", r1, " read16 at 400k=", fs, " mism=", fmism, "; peer count=",
          r.count, " addr_hits=", r.addr_hits, " faults this run tx=", tx_faults, " rx=", rx_faults,
          "; DMA interrupts ch6=", e6, " ch7=", e7, crlf);
    print(serial, "  SCL on the engines: the 16-byte write ", w_scl / 1000u, " kHz and read ", r_scl / 1000u,
          " kHz (100 asked), the read at 400k ", f_scl / 1000u, " kHz", crlf);
    bench.verdict("a 16-byte write and a 16-byte read through the DMA engines complete i2c_ok, "
                  "byte-exact",
                  ws == i2c_ok && rs == i2c_ok && mism == 0u);
    bench.verdict("the combined tenure, the two-byte read (LAST's NACK) and the one-byte read "
                  "(its NACK and STOP from the ADDR entry) all complete on the engines",
                  cs == i2c_ok && r2 == i2c_ok && r1 == i2c_ok);
    bench.verdict("a 16-byte read at 400 kHz on the engines completes i2c_ok, the serve's next "
                  "sixteen bytes exact",
                  fs == i2c_ok && fmism == 0u);
    bench.verdict("the peer accounts every byte (16 + 16 + 4 + 4 + 2 + 1 + 16 = 59), and this run "
                  "counts no transfer fault",
                  rep && r.count == 59u && tx_faults == 0u && rx_faults == 0u);
    bench.verdict("each write block interrupts once, its completion handing the half back (the "
                  "16-byte write, the combined's write half), each of the five engined reads once",
                  e6 == 2u && e7 == 5u);
    bench.verdict("and the engines run the bus at the rate asked: SCL never above 100 or 400 kHz by "
                  "more than 2 % and within a quarter below it",
                  ws == i2c_ok && rs == i2c_ok && fs == i2c_ok && scl_bracket(w_scl, 100'000u) &&
                      scl_bracket(r_scl, 100'000u) && scl_bracket(f_scl, 400'000u));
    host_ready();
}

// ===========================================================================
// h - the kernel against the peer
// ===========================================================================

namespace kl {

constexpr uint32_t bus_timeout_ticks = ticks_from_ms<P>(20);
using I2cArb = I2cBus<Host, P, 4, BusPassThrough, bus_timeout_ticks>;
/// The arbiter with no per-bus timeout, `timeout_ticks` 0: letters t and u.
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

using BusKernel = std::conditional_t<untimed_letters, Tenuto<P, Probe, I2cArb, I2cArbUntimed>,
                                     Tenuto<P, Probe, I2cArb>>;

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

// ===========================================================================
// i - the refusal, wireless
// ===========================================================================

/// A speed the clock in force cannot make is refused INSIDE start() and
/// reaches the requester through the arbiter as i2c_rejected - the one
/// synchronous completion of an I2C engine - with the wire and the
/// vector untouched. The clock itself is not moved: rebase() is told a
/// rate below 15.10.2's window, which empties the timing table, and
/// told 48 MHz again after.
void ti_refusal() {
    host_ready();
    kl::BusKernel::init_all();
    bus_ao_live = true;
    Host::rebase(6'000'000UL);
    bench.verdict("told 6 MHz, speed_ok says no to both speeds (below 15.10.2's window)",
                  !Host::speed_ok(I2cSpeed::standard_100k) && !Host::speed_ok(I2cSpeed::fast_400k));
    kl::Probe::clear_tally();
    const uint32_t entries = host_isr_entries;
    fill_pattern(kl::out_a, 4, 0x05);
    post<kl::I2cArb>(kl::request(nobody_addr, kl::out_a));
    kl::pump_until(1, 100);
    // STAR2's BUSY is the WIRE's level (both lines read low with no
    // pull-ups on this module); MSL is ours - it would stand had a
    // START gone out.
    print(serial, "  the request at 400 kHz replied ", kl::Probe::replies[0], " (i2c_rejected = ",
          i2c_rejected, "), STAR2=", hex(H::status2()), " (BUSY is the wire's level), event vector "
          "entries ", host_isr_entries - entries, crlf);
    bench.verdict("answered i2c_rejected inside start() and delivered through the arbiter, no "
                  "START issued (MSL clear) and the vector never entered",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_rejected &&
                      host_isr_entries == entries && (H::status2() & i2c_msl) == 0u);
    bench.verdict("the arbiter's own tally stays at zero (its rejection is the full queue's)",
                  kl::I2cArb::rejected_count() == 0u);
    Host::rebase(SysClock::hz);
    bench.verdict("told 48 MHz again, both speeds are back",
                  Host::speed_ok(I2cSpeed::standard_100k) && Host::speed_ok(I2cSpeed::fast_400k));
    bus_ao_live = false;
}

void th_kernel() {
    if (!need_peer()) {
        return;
    }
    twilink::Params a{};
    a.count = 512;
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
        post<kl::I2cArb>(kl::request((i == 2u) ? nobody_addr : twilink::dut_addr, kl::out_a));
    }
    kl::pump_until(4, 400);
    print(serial, "  four queued tenures: replies ", kl::Probe::n, " [", kl::Probe::replies[0], " ",
          kl::Probe::replies[1], " ", kl::Probe::replies[2], " ", kl::Probe::replies[3], "]", crlf);
    bench.verdict("four tenures through I2cBus against a SECOND CHIP, four replies - "
                  "util/i2c_bus.hpp and util/bus_master.hpp unchanged on this architecture",
                  kl::Probe::n == 4u);
    bench.verdict("... in order, with the NACK from an address nobody answers delivered IN ITS "
                  "PLACE as a reply",
                  kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_ok &&
                      kl::Probe::replies[2] == i2c_nack_addr && kl::Probe::replies[3] == i2c_ok);

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_b));
    }
    kl::pump_until(6, 400);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n, ", rejected ",
          kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately", kl::Probe::rejected != 0u);
    bench.verdict("... and every request is still answered exactly once", kl::Probe::n == 6u);

    kl::drain(100);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep", kl::Probe::votes == 1u && kl::Probe::last_vote);
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
    bench.verdict("a BUSY bus votes against it", kl::Probe::votes == 1u && !kl::Probe::last_vote);

    // THE TIMED BUS, with the wedge held by the OTHER BOARD.
    bus_ao_live = false;
    settle_ms(400);
    twilink::Params h{};
    h.ms = 300;
    h.aux8 = 0;
    h.aux16 = 0;
    const bool armed = peer_act(Op::hold_sda, h);
    bus_ao_live = true;
    if (!armed) {
        bench.verdict("the peer accepted the hold_sda command", false);
        return;
    }
    bench.verdict("the peer accepted the hold_sda command", true);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    const uint32_t t0 = Ticker::ticks();
    kl::pump_until(1, 400);
    const uint32_t took = Ticker::ticks() - t0;
    const bool still_low = !SdaPin::read();
    print(serial, "  the wedged tenure answered ", kl::Probe::n, " with status ", kl::Probe::replies[0],
          " after ", took, " ms; SDA still low at the reply: ", still_low ? "yes" : "no", crlf);
    print(serial, "  the answer is ",
          kl::Probe::replies[0] == i2c_arb_lost
              ? "i2c_arb_lost - the ENGINE's own wire code"
              : (kl::Probe::replies[0] == i2c_timeout ? "i2c_timeout - the ARBITER's"
                                                      : (kl::Probe::replies[0] == i2c_bus_error
                                                             ? "i2c_bus_error - the engine's"
                                                             : "another code")),
          crlf);
    bench.verdict("a tenure into a wire a FOREIGN CHIP holds down is answered IN ITS PLACE - "
                  "never silence and never i2c_ok",
                  kl::Probe::n == 1u && kl::Probe::replies[0] != i2c_ok);
    bench.verdict("... at the arbiter's own limit or sooner, never at the wedge's length", took <= 40u);

    bus_ao_live = false;
    settle_ms(400);
    link_ready();
    twilink::Params again{};
    again.count = 64;
    again.ms = 250;
    again.addr = twilink::dut_addr;
    again.seed = 0x90;
    const bool re = peer_act(Op::serve, again);
    bus_ao_live = true;
    kl::drain(20);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    kl::pump_until(1, 300);
    print(serial, "  after the release: replies ", kl::Probe::n, " status ", kl::Probe::replies[0], crlf);
    bench.verdict("THE SAME BUS AO carries the next tenure to i2c_ok after the wedge",
                  re && kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_ok);

    // THE CLOCK HELD past the limit: the wedge this silicon cannot answer
    // by itself - the F1 lineage's I2C has no clock-low timeout - so the
    // per-bus timeout must. The peer stretches the first data byte of a
    // write for 45 ms under the arbiter's 20.
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
    const uint32_t took_clock = Ticker::ticks() - t1;
    const bool scl_low = !SclPin::read();
    print(serial, "  the tenure into a held CLOCK answered ", kl::Probe::n, " with status ",
          kl::Probe::replies[0], " after ", took_clock, " ms (limit 20, hold 45); SCL still low at "
          "the reply: ", scl_low ? "yes" : "no", crlf);
    bench.verdict("a client holding the CLOCK past the limit is answered i2c_timeout ON THE "
                  "ARBITER'S CLOCK - the one wedge the silicon cannot answer by itself",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_timeout && took_clock >= 20u &&
                      took_clock <= 30u);
    kl::drain(100);
    bus_ao_live = false;
    settle_ms(400);
    link_ready();
    twilink::Params after{};
    after.count = 64;
    after.ms = 250;
    after.addr = twilink::dut_addr;
    after.seed = 0x91;
    const bool re2 = peer_act(Op::serve, after);
    bus_ao_live = true;
    kl::drain(20);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    kl::pump_until(1, 300);
    print(serial, "  after the clock came back: replies ", kl::Probe::n, " status ", kl::Probe::replies[0],
          ", stale events ", kl::I2cArb::stale_events() - stale_before, crlf);
    bench.verdict("the recover()ed engine carries the next tenure to i2c_ok through the same "
                  "arbiter",
                  re2 && kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_ok);
    bus_ao_live = false;
    settle_ms(300);
}

// ===========================================================================
// r - the repeated START of a write-then-read, acknowledged and refused
// ===========================================================================

/// One write-then-read through the plain host or the DMA host.
uint8_t either_tenure(bool dma, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
                      I2cSpeed speed) {
    return dma ? dma_tenure(twilink::dut_addr, tx, tx_len, rx, rx_len, speed)
               : host_tenure(twilink::dut_addr, tx, tx_len, rx, rx_len, speed);
}

/// The DMA host takes the vectors (the plain host keeps the command
/// channel, so it is brought back before the peer is spoken to again).
void bring_up(bool dma) {
    if (dma) {
        dma_host_live = true;
        (void)DmaHost::init(clock);
    }
}
void put_down(bool dma) {
    if (dma) {
        DmaHost::release();
        dma_host_live = false;
        host_ready();
    }
}

/// THE REPEATED START IS REQUESTED ON THE TxE OF THE LAST WRITTEN BYTE
/// (ch32v00x/i2c.hpp): one to four bytes written, then four read, at both
/// speeds, through the pump and through the engines. Acknowledged, the
/// peer counts and sums every written byte and the read is its pattern;
/// with the LAST written byte refused, the tenure is i2c_nack_data, START
/// is not left standing, the controller lets the bus go, and the next
/// tenure on the same host runs.
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
            // Acknowledged: one serve for the four tenures, counted at the end.
            twilink::Params a{};
            a.count = 0;   // the deadline ends it: a count would end it inside a tenure
            a.ms = 300;
            a.addr = twilink::dut_addr;
            a.seed = static_cast<uint8_t>(0x50u + (dma ? 8u : 0u) + static_cast<uint8_t>(speed));
            a.pattern = twilink::pattern_counting;
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
                uint8_t m = 0;
                for (uint8_t i = 0; i < 4u; ++i) {
                    if (rx_buf[i] != twilink::pattern_value(a.pattern, a.seed,
                                                            static_cast<uint16_t>(4u * (n - 1u) + i))) {
                        ++m;
                    }
                }
                if (m != 0u) {
                    print(serial, "    read after ", n, " written: ", hex(rx_buf[0]), " ",
                          hex(rx_buf[1]), " ", hex(rx_buf[2]), " ", hex(rx_buf[3]), " (seed ",
                          hex(a.seed), ")", crlf);
                }
                mism = static_cast<uint8_t>(mism + m);
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
                  " acknowledged: status ", sts[0], "/", sts[1], "/", sts[2], "/", sts[3],
                  ", event entries ", ev[0], "/", ev[1], "/", ev[2], "/", ev[3], ", DMA entries ", e6, "+",
                  e7, ", read mism ", mism,
                  "; peer took ", r.aux0, " (10) sum ", hex(r.sum), " (", hex(sum), ") address hits ",
                  r.addr_hits, " (8)", good ? "" : "  <- WRONG", crlf);

            // Refused: the LAST written byte NACKed, one serve a point.
            for (uint8_t n = 1; n <= 4u; ++n) {
                twilink::Params f{};
                f.count = 0;
                f.ms = 150;
                f.addr = twilink::dut_addr;
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
                twilink::Report r{};
                const bool rep = peer_report(r);
                const bool st_ok = st == i2c_nack_data && rep &&
                                   (r.flags & twilink::report_nacked) != 0u;
                const bool clean = (c1_later & (i2c_start | i2c_stop)) == 0u &&
                                   (s2_later & i2c_msl) == 0u;
                refuse_status = refuse_status && st_ok;
                refuse_clean = refuse_clean && clean;
                // The next tenure, a probe of the same address a millisecond
                // later, must COMPLETE on the wire: i2c_ok, or - the far end
                // a CH32 target whose acknowledge may stay down after the
                // refused byte until its window closes (twi_peer's header) -
                // i2c_nack_addr; a stall or a wedge is neither.
                const bool next_ok = next == i2c_ok || next == i2c_nack_addr;
                refuse_next = refuse_next && next_ok;
                print(serial, "  ", i2c_speed_hz(speed) / 1000u, " kHz ", dma ? "engines" : "pump   ",
                      " ", n, " written, the last refused: status ", st, ", CTLR1 at the end ",
                      hex(c1), " then ", hex(c1_later), ", STAR2 ", hex(s2_later),
                      ", the next tenure ", next, "; peer flags ", hex(r.flags), " address hits ",
                      r.addr_hits,
                      st_ok && clean && next_ok ? "" : "  <- WRONG", crlf);
            }
        }
    }
    bench.verdict("one to four bytes written then four read, both speeds, pump and engines (both "
                  "halves on them, each block's completion once a tenure): every written byte taken by "
                  "the peer - a POLLED CH32 target - every read byte its pattern",
                  ack_ok);
    bench.verdict("the last written byte refused: i2c_nack_data, the peer's own NACK",
                  refuse_status);
    bench.verdict("and the controller lets the bus go: no START or STOP left standing, MSL clear",
                  refuse_clean);
    bench.verdict("the next tenure on the same host runs to a wire outcome - i2c_ok, or the "
                  "address refused by a peer still deaf from its refusal - never a stall",
                  refuse_next);
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

/// WHICH ENGINE MOVES A HALF IS THE REQUEST'S TO NAME (ch32v00x/i2c.hpp,
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
    // THE ENTRIES ARE COUNTED: the write SB, ADDR with the first two bytes, 14
    // TxE and BTF; the read SB, ADDR, 13 RxNE down to three left and two BTF.
    // host_isr_entries counts the event vector alone, so a tick adds nothing.
    bench.verdict("interrupt at SIXTEEN BYTES on the host with engines bound touches no channel: no DMA "
                  "entry, both channels' CFGR and CNTR as init() left them, 17 event entries for the "
                  "write and 17 for the read - and the three tenures i2c_ok, exact (count 40)",
                  iw == i2c_ok && ir == i2c_ok && ic == i2c_ok && untouched && ie == 0u && iw_ev == 17u &&
                      ir_ev == 17u && imism == 0u && brep && b.count == 40u);

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
// j - two hosts on one wire, and this board as the peer's client
// ===========================================================================

/// A START CONDITION BY HAND: SDA pulled low from the port - an open-drain
/// output over a clear bit - while SCL stays released high. Every block on
/// the wire, this one's included, sees the bus taken.
void inject_start() { SdaPin::output(false, PinDrive::open_drain); }
/// AND THE STOP THAT CLOSES IT: SDA given back to the peripheral, so it
/// rises while SCL is still high.
void inject_stop() { SdaPin::function(PinDrive::open_drain); }

/// The host's bus_free_us, by the header's name: a suite cannot reach the
/// private constant, and letter t judges it against this one.
constexpr uint32_t bus_free_us_named = 100;

void spin_us(uint32_t us) {
    while (us >= 500u) {
        (void)delay_us(clock, 500u);
        us -= 500u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

void hold_bus_busy(uint32_t us) {
    inject_start();
    spin_us(us);
    inject_stop();
}

/// The peer's host action aimed at `target`, its start() run on the STOP
/// that closes this board's window (twi_peer's header).
twilink::Params host_action(uint8_t target, uint8_t count, uint8_t seed, uint16_t ms) {
    twilink::Params a{};
    a.ms = ms;
    a.addr = twilink::dut_addr;
    a.target = target;
    a.count = count;
    a.seed = seed;
    a.aux16 = 120;
    return a;
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
    const uint32_t c0 = Ticker::cycles();
    constexpr uint32_t per_us = SysClock::hz / 1'000'000u;
    while (!(SclPin::read() && SdaPin::read())) {
        if (Ticker::cycles() - c0 > 50u * per_us) {
            return true;   // a line low past any STOP of this board's
        }
    }
    const uint32_t c1 = Ticker::cycles();
    while (Ticker::cycles() - c1 < race_tail_us * per_us) {
        if (!SclPin::read()) {
            return true;
        }
    }
    return false;
}

/// ONE TENURE RACED AGAINST THE PEER'S: the window by hand, held long
/// enough for the peer to have seen it, then the STOP that frees the bus
/// and this host's start() at once - the peer's runs on the same STOP, so
/// both STARTs are set on a FREE bus within microseconds of each other.
/// Not set INSIDE the window: this block answers a START into a busy bus
/// with ARLO at once (the document), which would lose every race.
template <typename Hst = Host>
RaceRound race_tenure(uint8_t addr, const uint8_t* tx, uint8_t len, I2cEngine engine = I2cEngine::interrupt) {
    typename Hst::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = len;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    r.rx_len = 0;
    r.speed = link_speed;
    r.engine = engine;
    host_done = false;
    host_isr_entries = 0;
    host_stormed = false;
    inject_start();
    spin_us(350);
    inject_stop();
    // Between the STOP and start(), so that its few cycles also narrow the
    // gap to the peer's start(), this end being the earlier: no round of 276
    // missed with the stamp here, one of 121 with it before the STOP.
    const uint32_t c0 = Ticker::cycles();
    const bool sync = Hst::start(r);
    if (sync) {
        return {Hst::status(), 0u, false};
    }
    for (uint32_t i = 0; i < 600'000UL && !host_done; ++i) {
    }
    const uint32_t done_us = (Ticker::cycles() - c0) / (SysClock::hz / 1'000'000u);
    if (!host_done) {
        print(serial, "    the race never answered: STAR1=", hex(H::status1()), " STAR2=", hex(H::status2()),
              " CTLR1=", hex(H::regs().CTLR1), crlf);
        (void)Hst::recover();
        return {no_answer, done_us, false};
    }
    const bool tail = race_tail_seen();
    return {Hst::status(), done_us, tail};
}

/// What a START set into a live tenure did, raw: the first microsecond of
/// SB, ARLO and BERR after the store, BUSY's fall, the flags at the end.
struct LiveStart {
    uint32_t sb_at, arlo_at, berr_at, stop_at, last_stop_at;
    uint8_t stops;
    uint16_t s1, s2;
};

/// THE PEER'S CONTROLLER CLOCKING, THIS ONE'S START SET INTO IT: the
/// window by hand, its STOP - the peer's START follows within a few
/// microseconds and its address to nobody takes about ninety at 100 kHz -
/// then `delay` microseconds and the START bit stored with the vectors
/// dark, the flags polled for three milliseconds.
LiveStart start_into_live(uint32_t delay) {
    LiveStart w{};
    Pfic::disable(H::event_irq());
    Pfic::disable(H::error_irq());
    inject_start();
    spin_us(350);
    inject_stop();
    spin_us(delay);
    H::start();
    const uint32_t c0 = Ticker::cycles();
    constexpr uint32_t per_us = SysClock::hz / 1'000'000u;
    // THE PEER'S STOP ON THE PADS: SDA rising while SCL stands high - the
    // one edge a data bit never makes.
    bool prev_scl = SclPin::read();
    bool prev_sda = SdaPin::read();
    while (Ticker::cycles() - c0 < 3000u * per_us) {
        const uint16_t s1 = H::status1();
        const bool scl = SclPin::read();
        const bool sda = SdaPin::read();
        const uint32_t now = (Ticker::cycles() - c0) / per_us + 1u;
        if (scl && prev_scl && sda && !prev_sda) {
            if (w.stop_at == 0u) {
                w.stop_at = now;
            }
            w.last_stop_at = now;
            ++w.stops;
        }
        prev_scl = scl;
        prev_sda = sda;
        if ((s1 & i2c_sb) != 0u && w.sb_at == 0u) {
            w.sb_at = now;
        }
        if ((s1 & i2c_arlo) != 0u && w.arlo_at == 0u) {
            w.arlo_at = now;
        }
        if ((s1 & i2c_berr) != 0u && w.berr_at == 0u) {
            w.berr_at = now;
        }
    }
    w.s1 = H::status1();
    w.s2 = H::status2();
    return w;
}

/// What one request posted into the peer's long tenure came to.
struct LiveRound {
    uint8_t status;       ///< this board's request
    uint8_t peer_status;  ///< the peer's tenure, its report
    bool peer_reported;
    bool reset;           ///< start() counted a reset
    bool ran_out;         ///< start() spent the whole of bus_free_us
    uint16_t taken;       ///< data bytes this board's client half took
    uint16_t sum;         ///< and their sum
};

/// The three foreign tenures: `arb`'s aux8 at the peer (its header).
enum class Foreign : uint8_t { bytes_100k = 0, bytes_400k = 1, probe_20k = 2 };

/// A TENURE LONGER THAN THE BOUND, two ways. AT 100 AND 400 kHz its controller
/// writes 24 bytes of 0x55, whose acknowledges this board's own client half gives,
/// the only other chip on the wire: it answers from the event vector (the vector's
/// glue below), ADDR cleared, each byte taken, the clock free, and start()'s one
/// store of CTLR1 drops ACK, so the tenure ends on the byte behind the request, a
/// NACK that is the staging's. AT 20 kHz its probe of nobody, nine clocks of 50 us,
/// this block a bystander as beside a third chip. `delay` is the post's moment
/// behind the peer's START.
LiveRound live_round(Foreign kind, uint32_t delay) {
    LiveRound out{};
    twilink::Params a = host_action(kind == Foreign::probe_20k ? nobody_addr : twilink::dut_addr,
                                    kind == Foreign::probe_20k ? 0 : 24, 0x55, 200);
    a.pattern = twilink::pattern_fixed;
    a.aux8 = static_cast<uint8_t>(kind);
    if (!peer_act(Op::arb, a)) {
        out.status = no_answer;
        return out;
    }
    host_ready();
    Host::Request q{};
    q.addr = nobody_addr;
    q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    q.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    q.speed = link_speed;
    live_count = 0;
    live_sum = 0;
    if (kind != Foreign::probe_20k) {
        H::regs().OADDR1 = static_cast<uint16_t>(twilink::dut_addr << 1);
        H::ack(true);
        live_client = true;
        H::interrupts(i2c_itevten | i2c_itbufen, 0);
    }
    host_done = false;
    host_isr_entries = 0;
    host_stormed = false;
    const uint16_t resets0 = Host::unwedges();
    inject_start();
    spin_us(350);
    inject_stop();
    spin_us(delay);
    const uint32_t c0 = Ticker::cycles();
    const bool sync = Host::start(q);
    const uint32_t c1 = Ticker::cycles();
    constexpr uint32_t per_us = SysClock::hz / 1'000'000u;
    while (!sync && !host_done && Ticker::cycles() - c1 < 10'000u * per_us) {
    }
    out.status = sync || host_done ? Host::status() : no_answer;
    live_client = false;
    if (!sync && !host_done) {
        (void)Host::recover();
    }
    out.reset = Host::unwedges() != resets0;
    out.ran_out = c1 - c0 >= uint32_t{bus_free_us_named} * per_us;
    out.taken = live_count;
    out.sum = live_sum;
    H::regs().OADDR1 = 0;
    settle_ms(3);
    twilink::Report r{};
    out.peer_reported = peer_report(r);
    out.peer_status = r.mstatus;
    return out;
}

void tj_two_hosts() {
    if (!need_peer()) {
        return;
    }
    // ---- 1. ARBITRATION, both directions ----
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
    for (uint8_t i = 0; i < 2u; ++i) {
        RaceRound round{};
        twilink::Report r{};
        bool rep = false;
        for (uint8_t attempt = 0; attempt < race_attempts; ++attempt) {
            if (!peer_act(Op::arb, host_action(rounds[i].peer_target, 4, 0x40, 600))) {
                bench.verdict("the peer accepted the arbitration command", false);
                return;
            }
            host_ready();
            for (uint8_t k = 0; k < 5; ++k) {
                tx_buf[k] = static_cast<uint8_t>(0xD0u + k);
            }
            round = race_tenure(rounds[i].dut_target, tx_buf, 5);
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
    bench.verdict("arbitration was decided in BOTH directions against a second chip's controller",
                  as_expected == 2u);

    // The losing round again with the request naming the engines: the loss
    // falls in the address, before any block starts, and is the same
    // i2c_arb_lost - no transmit block started or thrown away.
    RaceRound dround{};
    twilink::Report dr{};
    bool drep = false;
    bool dma_quiet = false;
    for (uint8_t attempt = 0; attempt < race_attempts; ++attempt) {
        if (!peer_act(Op::arb, host_action(twilink::dut_addr, 4, 0x40, 600))) {
            bench.verdict("the peer accepted the arbitration command", false);
            return;
        }
        host_ready();
        bring_up(true);
        const uint32_t lost_faults0 = DmaTxEngine<6>::faults();
        dma6_entries = 0;
        dround = race_tenure<DmaHost>(twilink::command_addr, tx_buf, 5, I2cEngine::dma);
        dma_quiet = dma6_entries == 0u && DmaTxEngine<6>::faults() == lost_faults0 &&
                    !DmaChannel<6>::enabled();
        put_down(true);
        settle_ms(700);
        drep = peer_report(dr);
        const bool missed = race_missed(dround, drep, dr);
        ++tally.rounds;
        print(serial, "  the same loss under I2cEngine::dma: status=", dround.status, " after ", dround.done_us, " us",
              dround.tail ? ", a tenure after it" : "", ", channel 6 ", dma_quiet ? "untouched" : "TOUCHED",
              "; peer: flags=", hex(dr.flags), missed ? " - NO RACE, two tenures, again" : "", crlf);
        if (!missed) {
            break;
        }
        ++tally.missed;
    }
    bench.verdict("under dma the same loss is i2c_arb_lost, the peer's controller keeping the bus, and no "
                  "transmit block was started or thrown away",
                  dround.status == i2c_arb_lost && dma_quiet && drep &&
                      (dr.flags & twilink::report_arblost) == 0u && (dr.flags & twilink::report_host_ran) != 0u);
    print(serial, "  the race: ", tally.missed, " of ", tally.rounds, " rounds missed it (two tenures, nothing "
          "arbitrated, not judged), at most ", race_attempts - 1u, " a judged round", crlf);

    // ---- 2. A START INTO A TENURE ANOTHER HOST IS CLOCKING ----
    // The peer's controller probes an address nobody answers (S, nine
    // clocks, P: about 95 us at 100 kHz); this one's START is set thirty
    // microseconds into it, inside the address byte.
    link_ready();
    if (!peer_act(Op::arb, host_action(nobody_addr, 0, 0, 600))) {
        bench.verdict("the peer accepted the host command into nobody", false);
        return;
    }
    host_ready();
    const LiveStart w = start_into_live(30);
    (void)Host::recover();
    settle_ms(700);
    twilink::Report lr{};
    const bool lrep = peer_report(lr);
    print(serial, "  a START set 30 us into the peer's tenure, raw: SB at ", w.sb_at, " us, ARLO at ",
          w.arlo_at, " us, BERR at ", w.berr_at, " us (0 = never), the peer's STOP on the pads at ",
          w.stop_at, " us (", w.stops, " SDA rise(s) under a high SCL seen, the last at ", w.last_stop_at,
          " us); STAR1=", hex(w.s1), " STAR2=", hex(w.s2), "; the peer's controller: flags=",
          hex(lr.flags), " status=", lr.mstatus, crlf);
    const bool raw_resolved = w.sb_at != 0u || w.arlo_at != 0u || w.berr_at != 0u;
    print(serial, "  -> ",
          w.arlo_at != 0u && (w.sb_at == 0u || w.arlo_at < w.sb_at)
              ? "ANSWERED ARLO - the live wire refused this START"
              : (w.sb_at != 0u && w.stop_at != 0u && w.sb_at > w.stop_at
                     ? "HELD until the peer's STOP freed the bus, then sent (SB after the STOP's edge)"
                     : (w.sb_at != 0u ? "SB before any STOP was seen - the START went out on a live wire"
                                      : "no SB and no ARLO")),
          crlf);
    bench.verdict("a START set into a tenure another controller is clocking is resolved by this block "
                  "within 3 ms (SB, ARLO or BERR) - either answer is the finding printed above",
                  raw_resolved);
    bench.verdict("... and the other controller's tenure is not disturbed: the peer's probe of nobody "
                  "ends i2c_nack_addr, neither lost nor in error",
                  lrep && (lr.flags & twilink::report_host_ran) != 0u && lr.mstatus == i2c_nack_addr &&
                      (lr.flags & (twilink::report_arblost | twilink::report_buserr)) == 0u);

    // Through the ENGINE, its start() thirty microseconds into the same
    // tenure: the bounded BUSY wait (the document) meets the peer's STOP. The
    // verdict below ("never a stall") is of this 100 kHz tenure: under a slower
    // clock the request mostly stalls, the 20 kHz row of 2b.
    link_ready();
    if (!peer_act(Op::arb, host_action(nobody_addr, 0, 0, 600))) {
        bench.verdict("the peer accepted the host command into nobody", false);
        return;
    }
    host_ready();
    {
        Host::Request q{};
        q.addr = nobody_addr;
        q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        q.tx_len = 0;
        q.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
        q.rx_len = 0;
        q.speed = link_speed;
        host_done = false;
        inject_start();
        spin_us(350);
        inject_stop();
        spin_us(30);
        const uint32_t c0 = Ticker::cycles();
        const bool sync = Host::start(q);
        const uint32_t start_us = (Ticker::cycles() - c0) / (SysClock::hz / 1'000'000u);
        for (uint32_t i = 0; i < 600'000UL && !sync && !host_done; ++i) {
        }
        const uint8_t est = sync || host_done ? Host::status() : no_answer;
        if (!sync && !host_done) {
            (void)Host::recover();
        }
        settle_ms(700);
        twilink::Report er{};
        const bool erep = peer_report(er);
        print(serial, "  the ENGINE's start() 30 us into the peer's tenure: returned after ", start_us,
              " us, status ", est, "; the peer's controller: flags=", hex(er.flags), " status=", er.mstatus,
              crlf);
        bench.verdict("through the engine the same START ends on a wire outcome of its own (never a stall), "
                      "and the peer's tenure ends i2c_nack_addr, undisturbed",
                      est != no_answer && est != i2c_ok && erep && er.mstatus == i2c_nack_addr &&
                          (er.flags & (twilink::report_arblost | twilink::report_buserr)) == 0u);
    }

    // ---- 2b. THE WATCH: a request into a tenure longer than the bound ----
    // BUSY up through the whole of bus_free_us and a clock on the wire: SWRST
    // must not be taken. The post's moment swept by 7 us over 32 rounds, 7 being
    // prime to the bit at 100 kHz (10 us) and landing on five phases of it at
    // 400 kHz (2.5 us) and prime to the 50 us bit at 20 kHz. The sweep starts behind
    // the peer's START: its start() runs on the STOP, 5 us, and SB and the first
    // low SCL follow within half a bit, so 18 us at 100 and 400 kHz and 40 at 20. It
    // ends at 235 and 257 us, the bound's 100 us inside the 24 bytes and the address,
    // 2250 us and 562, and the probe's 450. 0x55 is half ones, so SDA stands high in
    // half the bits' high phases.
    {
        link_ready();
        for (const Foreign kind : {Foreign::bytes_100k, Foreign::bytes_400k, Foreign::probe_20k}) {
            constexpr uint8_t rounds = 32;
            uint8_t ran_out = 0;
            uint8_t resets = 0;
            uint8_t answered = 0;
            uint8_t stalled = 0;
            uint8_t last_other = i2c_ok;
            uint8_t peer_clean = 0;
            uint16_t taken = 0;
            uint16_t sum_bad = 0;
            for (uint8_t k = 0; k < rounds; ++k) {
                const LiveRound w = live_round(kind, (kind == Foreign::probe_20k ? 40u : 18u) + 7u * k);
                ran_out = static_cast<uint8_t>(ran_out + (w.ran_out ? 1u : 0u));
                resets = static_cast<uint8_t>(resets + (w.reset ? 1u : 0u));
                answered = static_cast<uint8_t>(answered + (w.status == i2c_nack_addr ? 1u : 0u));
                stalled = static_cast<uint8_t>(stalled + (w.status == no_answer ? 1u : 0u));
                if (w.status != i2c_nack_addr && w.status != no_answer) {
                    last_other = w.status;
                }
                peer_clean = static_cast<uint8_t>(peer_clean + (w.peer_reported && (kind == Foreign::probe_20k ? w.peer_status == i2c_nack_addr : (w.peer_status == i2c_nack_data || w.peer_status == i2c_ok)) ? 1u : 0u));
                taken = static_cast<uint16_t>(taken + w.taken);
                sum_bad = static_cast<uint16_t>(sum_bad + (w.sum != static_cast<uint16_t>(w.taken * 0x55u) ? 1u : 0u));
            }
            print(serial, "  a request into the peer's ", kind == Foreign::probe_20k ? "probe at 20" : kind == Foreign::bytes_400k ? "24 bytes at 400" : "24 bytes at 100",
                  " kHz: ", rounds, " posted, the bound ran out in ", ran_out, ", resets taken ", resets, ", its probe of nobody i2c_nack_addr in ",
                  answered, ", another wire status in ", rounds - answered - stalled, " (the last ", last_other, "), unanswered in 10 ms ", stalled, ", the peer's tenure clean in ", peer_clean, ", bytes taken ", taken, ", sums off ", sum_bad, crlf);
            // At 100 and 400 kHz this request ends on the wire: a NACK of its probe in all but
            // a few rounds of the hundreds run, another wire status in those, printed, its START
            // meeting the end of the peer's tenure, which then may end i2c_bus_error too (1 round
            // of 576): counted, not judged, there being no reset in it. At 20 kHz this request
            // stands (the page's finding: a START into a clock slower than 100 kHz), and the
            // peer's tenure, this block a bystander, is judged whole.
            bench.verdict("THE WATCH holds: BUSY up through the whole bound with a clock on the wire takes no SWRST, the "
                          "request ends on the wire at 100 and 400 kHz, a bystander leaves the peer's tenure whole",
                          resets == 0u && ran_out == rounds && sum_bad == 0u &&
                              (kind == Foreign::probe_20k ? peer_clean == rounds : stalled == 0u));
        }
        host_ready();
    }

    // ---- 3. THIS BOARD AS THE CLIENT, WRITTEN TO by the peer's controller ----
    link_ready();
    twilink::Params c = host_action(twilink::dut_addr, 6, 0xB0, 800);
    c.aux16 = 400;
    if (!peer_act(Op::arb, c)) {
        bench.verdict("the peer accepted the host command that addresses this board", false);
        return;
    }
    Host::release();
    bool up = Client::init(clock, {.own = twilink::dut_addr}, {.no_stretch = false, .interrupts = false});
    hold_bus_busy(400);
    uint8_t got[16];
    uint8_t n_got = 0;
    uint8_t hits = 0;
    uint8_t stops = 0;
    bool wrote_to_us = false;
    uint16_t late_s1 = 0;
    bool late_seen = false;
    uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 900u) {
        if (Client::data_ready()) {
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
        } else if (Client::addressed()) {
            ++hits;
            wrote_to_us = !Client::answer_address();
        } else if (Client::stop_seen()) {
            Client::clear_stop();
            ++stops;
            if (n_got >= c.count) {
                break;
            }
        } else if ((H::status1() & i2c_errors) != 0u) {
            H::clear_errors(static_cast<uint16_t>(H::status1() & i2c_errors));
        }
    }
    uint8_t mism = 0;
    for (uint8_t i = 0; i < n_got && i < c.count; ++i) {
        if (got[i] != twilink::pattern_value(twilink::pattern_counting, c.seed, i)) {
            ++mism;
        }
    }
    print(serial, "  as a CLIENT, written to: init=", up ? 1 : 0, " address matches=", hits, " bytes=", n_got,
          " mism=", mism, " stops=", stops, "; STAR1 two ms after the first byte was left in DATAR: ",
          hex(late_s1),
          (late_s1 & (i2c_rxne | i2c_btf)) == (i2c_rxne | i2c_btf)
              ? " (RxNE AND BTF: the next byte is in the shifter, SCL held)"
              : "",
          crlf);
    bench.verdict("the client answered its own address to a REAL controller on another board, in a write "
                  "tenure",
                  up && hits >= 1u && wrote_to_us);
    bench.verdict("... and took every byte the peer's controller wrote, byte-exact, ending on its STOP",
                  n_got >= c.count && mism == 0u && stops >= 1u);
    bench.verdict("THE STRETCH ONE BYTE LATE: a byte left in DATAR lets the NEXT one into the shifter, and "
                  "only then is SCL held (RxNE and BTF both up)",
                  late_seen && (late_s1 & (i2c_rxne | i2c_btf)) == (i2c_rxne | i2c_btf));
    Client::release();
    settle_ms(300);

    // ---- 4. THIS BOARD AS THE CLIENT, READ FROM: the transmitter ----
    // Two reads of six bytes back to back; each read tenure serves its own
    // run (0xC0.., then 0xE0..), so a byte left in DATAR past the first
    // read's closing NACK would open the second read in place of 0xE0.
    host_ready();
    twilink::Params rd = host_action(twilink::dut_addr, 6, 0, 900);
    rd.aux16 = 400;
    rd.aux8 = 2;
    if (!peer_act(Op::host_read, rd)) {
        bench.verdict("the peer accepted the host_read command (the extended repertoire)", false);
        host_ready();
        return;
    }
    Host::release();
    up = Client::init(clock, {.own = twilink::dut_addr}, {.no_stretch = false, .interrupts = false});
    hold_bus_busy(400);
    uint8_t read_tenures = 0;
    uint8_t gave = 0;
    uint8_t nacks = 0;
    uint8_t drops = 0;
    bool serving = false;
    uint8_t seed = 0;
    uint8_t i_tx = 0;
    uint16_t given_sum = 0;
    t0 = Ticker::millis();
    // ONE STAR1 SNAPSHOT A PASS, every decision taken on it: the closing
    // NACK is a LEVEL that may rise between two separate reads of the
    // register, and a pass that cleared the error flags on a second read
    // would swallow it uncounted.
    while (Ticker::millis() - t0 < 1000u) {
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
    print(serial, "  as a CLIENT, read from: init=", up ? 1 : 0, " read tenures=", read_tenures, " bytes given=",
          gave, " closing NACKs=", nacks, " dropped by flush()=", drops, "; the peer read ", rr.count,
          " bytes in ", rr.addr_hits, " reads, status ", rr.mstatus, ", sum ", hex(rr.sum), " (", hex(want_sum),
          "), first ", hex(rr.first), " (0xC0), the second read's first ", hex(rr.aux0),
          " (0xE0); this side's sum of what left, the drops given back, ", hex(given_sum), crlf);
    bench.verdict("the client served a REAL controller's two reads as a transmitter, every byte the "
                  "peer took its own run (count, sum and both first bytes), and what this side counts "
                  "as sent - the drops given back - is what the peer read",
                  up && rrep && rr.mstatus == i2c_ok && rr.count == 12u && rr.addr_hits == 2u &&
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
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms && !done()) {
        asm volatile("" ::: "memory");   // the vectors write the tally
    }
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
    twilink::Params c = host_action(twilink::dut_addr, 6, 0xB0, 800);
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
    twilink::Params rd = host_action(twilink::dut_addr, 6, 0, 900);
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
// y - the engines' channels starved
// ===========================================================================
//
// A write on the engines keeps the event vector up, and BTF is its level: a transmit
// channel a load holds still lets the byte in the shifter end with DATAR empty, BTF up and
// the count standing. The load is the worst this DMA offers: channel 1 at very_high,
// above the engines' `high`, memory to memory, load_beats words from a cell to a cell. A
// memory-to-memory channel has no request to wait for and gives the arbiter no gap
// (RM 9.2.1), so the channels below it move nothing until its block is over. It ends by
// itself, so even a vector re-entering for its whole length gives the suite back.
//
// A row is one tenure at link_speed, the load up before the START or, `late`, 300 us into
// it: the address and two bytes, 27 clocks at 100 kHz. What it counts: the answer's
// status, whether it came with the load up, the event vector's entries, the far end's
// count or the bytes read compared. THE ENTRIES' BOUND: SB and ADDR, then the BTF that
// ends the tenure on a fault or the plain end's after the load, four with one re-entry of
// a line still high (the same bound letter p holds a dead channel to); a level re-entering
// without end would take host_isr_budget, which the vector counts and stops.
namespace ly {
using Load = DmaChannel<1>;
constexpr uint16_t load_beats = 30000;
alignas(4) volatile uint32_t load_cell[2];

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
    uint16_t other = 0;      ///< any other answer, or none
    uint16_t early = 0;      ///< answered with the load up
    uint32_t entries = 0;    ///< the most event entries a row took
    uint16_t stormed = 0;
    uint16_t took_min = 0xFFFFu;
    uint16_t took_max = 0;
};

/// One tenure of `n`, a write or a read, under the load (or none). The peer serves a
/// short window; the plain host reads its report afterwards.
void row(uint8_t n, bool write, bool late, bool load, Row& out) {
    const uint8_t seed = 0x40;
    twilink::Params a{};
    a.count = 0;
    a.ms = 40;
    a.addr = twilink::dut_addr;
    a.seed = seed;
    a.pattern = twilink::pattern_counting;
    (void)peer_act(Op::serve, a);
    dma_host_live = true;
    (void)DmaHost::init(clock);
    for (uint8_t i = 0; i < n; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x20u + i);
        rx_buf[i] = 0xEE;
    }
    DmaHost::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(write ? static_cast<const uint8_t*>(tx_buf) : static_cast<const uint8_t*>(nullptr));
    r.tx_len = write ? n : uint8_t{0};
    r.rx = lend<Lease::reply>(write ? static_cast<uint8_t*>(nullptr) : static_cast<uint8_t*>(rx_buf));
    r.rx_len = write ? uint8_t{0} : n;
    r.speed = link_speed;
    r.engine = I2cEngine::dma;
    host_done = false;
    host_isr_entries = 0;
    host_stormed = false;
    if (load && !late) {
        load_start();
    }
    const bool at_once = DmaHost::start(r);
    if (load && late) {
        (void)delay_us(clock, 300);
        load_start();
    }
    while (load && Load::count() != 0u && !host_done) {
    }
    // Beats landing as the load's last one does may answer between the two looks: only a
    // count still standing behind the answer says it came with the load up.
    const bool early = load && host_done && Load::count() != 0u;
    for (uint32_t i = 0; i < 600'000UL && !host_done && !at_once; ++i) {
    }
    const bool answered = at_once || host_done;
    const uint8_t st = answered ? DmaHost::status() : no_answer;
    const uint32_t entries = host_isr_entries;
    const bool stormed = host_stormed;
    while (load && Load::count() != 0u) {
    }
    Load::stop();
    if (!answered) {
        (void)DmaHost::recover();
    }
    DmaHost::release();
    host_ready();
    settle_ms(60);
    twilink::Report rep{};
    const uint16_t took = peer_report(rep) ? rep.count : uint16_t{0xFFFF};
    bool whole = write ? took == n : true;
    if (!write) {
        for (uint8_t i = 0; i < n; ++i) {
            whole = whole && rx_buf[i] == twilink::pattern_value(twilink::pattern_counting, seed, i);
        }
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
    } else {
        out.other = static_cast<uint16_t>(out.other + 1u);
    }
    out.early = static_cast<uint16_t>(out.early + (early ? 1u : 0u));
    out.stormed = static_cast<uint16_t>(out.stormed + (stormed ? 1u : 0u));
    out.entries = entries > out.entries ? entries : out.entries;
    out.took_min = took < out.took_min ? took : out.took_min;
    out.took_max = took > out.took_max ? took : out.took_max;
}

void print_row(const char* what, uint8_t n, const Row& r) {
    print(serial, "  ", what, " of ", n, ", ", r.rows, " rows: i2c_ok whole ", r.ok, ", i2c_ok short ", r.short_ok,
          ", i2c_dma_fault ", r.fault, ", else ", r.other, "; with the load up ", r.early, "; event entries at most ",
          r.entries, r.stormed != 0u ? ", STORMED" : "", "; the far end counted ", r.took_min, " to ", r.took_max, crlf);
}
}  // namespace ly

void ty_starved() {
    if (!need_peer()) {
        return;
    }
    ly::Row plain_w{}, plain_r{}, wr2{}, wr8{}, late_w{}, rd1{}, rd2{}, rd8{}, late_r{};
    for (uint8_t k = 0; k < 8u; ++k) {
        ly::row(8, true, false, false, plain_w);
        ly::row(8, false, false, false, plain_r);
        ly::row(2, true, false, true, wr2);
        ly::row(8, true, false, true, wr8);
        ly::row(8, true, true, true, late_w);
        ly::row(1, false, false, true, rd1);
        ly::row(2, false, false, true, rd2);
        ly::row(8, false, false, true, rd8);
        ly::row(8, false, true, true, late_r);
    }
    ly::print_row("no load, a write", 8, plain_w);
    ly::print_row("no load, a read", 8, plain_r);
    ly::print_row("the load from the start, a write", 2, wr2);
    ly::print_row("the load from the start, a write", 8, wr8);
    ly::print_row("the load from the third byte, a write", 8, late_w);
    ly::print_row("the load from the start, a read", 1, rd1);
    ly::print_row("the load from the start, a read", 2, rd2);
    ly::print_row("the load from the start, a read", 8, rd8);
    ly::print_row("the load from the third byte, a read", 8, late_r);
    const auto never_short = [](const ly::Row& r) {
        return r.rows != 0u && r.short_ok == 0u && r.other == 0u && r.stormed == 0u && r.entries <= 4u;
    };
    bench.verdict("with no load every tenure is whole", plain_w.ok == plain_w.rows && plain_r.ok == plain_r.rows);
    bench.verdict("A WRITE STARVED FROM ITS THIRD BYTE ends i2c_dma_fault with the load up, in every row, the far end "
                  "short of the eight, the event vector at most 4 entries: the status says what the wire did",
                  never_short(late_w) && late_w.fault == late_w.rows && late_w.early == late_w.rows &&
                      late_w.took_max < 8u);
    bench.verdict("A WRITE STARVED FROM ITS START: every byte, or a status that says otherwise, at most 4 entries",
                  never_short(wr2) && never_short(wr8));
    bench.verdict("A READ STARVED, from its start or its third byte: every byte, or a status that says otherwise, "
                  "at most 4 entries",
                  never_short(rd1) && never_short(rd2) && never_short(rd8) && never_short(late_r));
    bench.verdict("and the command channel is back", command(Op::ping));
}


// ===========================================================================
// q - the controller's own habits, raw, against this pairing's target
// ===========================================================================

/// The instance as a controller with the vectors DARK and the timing of
/// `speed` in force: the host's init() for the pads and the column, then
/// the enables and the two PFIC lines down and CKCFGR rewritten under a
/// PE cycle (timing() clears PE itself).
void raw_up(I2cSpeed speed) {
    (void)Host::init(clock);
    Pfic::disable(H::event_irq());
    Pfic::disable(H::error_irq());
    H::interrupts(0, i2c_itevten | i2c_itbufen | i2c_iterren);
    H::timing(*i2c_timing_for(SysClock::hz, speed));
    H::enable();
}

/// Cycles until `mask` rises in STAR1, or 0xFFFFFFFF past `us`.
uint32_t raw_wait(uint16_t mask, uint32_t us) {
    const uint32_t c0 = Ticker::cycles();
    const uint32_t budget = us * (SysClock::hz / 1'000'000u);
    while ((H::status1() & mask) == 0u) {
        if (Ticker::cycles() - c0 > budget) {
            return 0xFFFF'FFFFu;
        }
    }
    return Ticker::cycles() - c0;
}

/// START, the address with the write bit, ADDR cleared, `first` loaded:
/// the opening every experiment below shares. False when a step never came.
bool raw_open(uint8_t first) {
    H::start();
    if (raw_wait(i2c_sb, 2000) == 0xFFFF'FFFFu) {
        return false;
    }
    H::data(static_cast<uint8_t>(twilink::dut_addr << 1));
    if (raw_wait(i2c_addr, 2000) == 0xFFFF'FFFFu) {
        return false;
    }
    (void)H::clear_addr();
    H::data(first);
    return true;
}

/// Put the instance back for the command channel, whatever a raw
/// experiment left on the wire.
void raw_down() {
    (void)Host::recover();
    host_ready();
    settle_ms(5);
}

void tq_controller() {
    if (!need_peer()) {
        return;
    }
    bool btf_ok = true;
    bool sr_ok = true;
    uint8_t wedged = 0;
    for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
        const uint32_t bit = SysClock::hz / i2c_speed_hz(speed);
        // ---- 1. BTF while the requested STOP or repeated START goes out ----
        twilink::Params a{};
        a.ms = 300;
        a.addr = twilink::dut_addr;
        if (!peer_act(Op::serve, a)) {
            bench.verdict("the peer accepted the serve", false);
            return;
        }
        uint32_t btf_stop = 0xFFFF'FFFFu;
        uint32_t btf_sr = 0xFFFF'FFFFu;
        uint32_t sb_sr = 0xFFFF'FFFFu;
        raw_up(speed);
        if (raw_open(0x5A) && raw_wait(i2c_txe, 2000) != 0xFFFF'FFFFu) {
            H::data(0xA5);
            if (raw_wait(i2c_btf, 2000) != 0xFFFF'FFFFu) {
                H::stop();
                const uint32_t c0 = Ticker::cycles();
                while (H::flag(i2c_btf) && Ticker::cycles() - c0 < 20u * bit) {
                }
                btf_stop = Ticker::cycles() - c0;
            }
        }
        settle_ms(1);
        if (raw_open(0x5B) && raw_wait(i2c_txe, 2000) != 0xFFFF'FFFFu) {
            H::data(0xA6);
            if (raw_wait(i2c_btf, 2000) != 0xFFFF'FFFFu) {
                H::start();
                const uint32_t c0 = Ticker::cycles();
                while (H::flag(i2c_btf) && Ticker::cycles() - c0 < 20u * bit) {
                }
                btf_sr = Ticker::cycles() - c0;
                sb_sr = raw_wait(i2c_sb, 2000);
                if (sb_sr != 0xFFFF'FFFFu) {
                    // Close behind the repeated START: the address read back
                    // to nobody, refused, and the STOP.
                    H::data(static_cast<uint8_t>((nobody_addr << 1) | 1u));
                    (void)raw_wait(i2c_af, 2000);
                    H::clear_errors(i2c_af);
                    H::stop();
                }
            }
        }
        raw_down();
        settle_ms(320);
        print(serial, "  ", i2c_speed_hz(speed) / 1000u, " kHz (a bit is ", bit, " cycles): BTF stood ",
              btf_stop, " cycles after a STOP requested at EVT8_2, ", btf_sr, " after a repeated START "
              "(SB ", sb_sr, " cycles after BTF fell)", crlf);
        // Standing, and clearing by itself once the condition is out: the
        // 20-bit window above is the poll's bound, a few bits the answer.
        btf_ok = btf_ok && btf_stop != 0u && btf_stop < 4u * bit && btf_sr != 0u && btf_sr < 4u * bit &&
                 sb_sr != 0xFFFF'FFFFu;

        // ---- 2. a repeated START requested while a refused byte shifts ----
        for (const bool withdraw : {false, true}) {
            twilink::Params f{};
            f.ms = 300;
            f.addr = twilink::dut_addr;
            f.nack_at = 1;
            if (!peer_act(Op::serve, f)) {
                bench.verdict("the peer accepted the nack-at serve", false);
                return;
            }
            raw_up(speed);
            uint32_t af_at = 0xFFFF'FFFFu;
            uint32_t sb_at = 0xFFFF'FFFFu;
            uint16_t c1_later = 0;
            uint16_t s2_later = 0;
            if (raw_open(0x77) && raw_wait(i2c_txe, 2000) != 0xFFFF'FFFFu) {
                // The repeated START requested while the byte shifts - the
                // engine's order for a write-then-read's last byte.
                H::start();
                const uint32_t c0 = Ticker::cycles();
                af_at = raw_wait(i2c_af, 2000);
                if (withdraw && af_at != 0xFFFF'FFFFu) {
                    // THE WITHDRAWAL the engine declines: one store clearing
                    // START and setting STOP at the NACK.
                    H::control(i2c_stop, i2c_start);
                } else if (af_at != 0xFFFF'FFFFu) {
                    const uint32_t s = raw_wait(i2c_sb, 2000);
                    sb_at = s == 0xFFFF'FFFFu ? s : Ticker::cycles() - c0 - af_at;
                }
                H::clear_errors(i2c_af);
                if (!withdraw && sb_at != 0xFFFF'FFFFu) {
                    // The engine's close: the void write - the address with
                    // the write bit - and the STOP behind it.
                    H::data(static_cast<uint8_t>(twilink::dut_addr << 1));
                    H::stop();
                }
                settle_ms(1);
                c1_later = H::regs().CTLR1;
                s2_later = H::status2();
            }
            raw_down();
            settle_ms(320);
            if (!withdraw) {
                print(serial, "  ", i2c_speed_hz(speed) / 1000u, " kHz, the byte refused with a repeated START "
                      "requested: AF ", af_at, " cycles after the request, SB ", sb_at, " cycles after AF", crlf);
                sr_ok = sr_ok && af_at != 0xFFFF'FFFFu && sb_at != 0xFFFF'FFFFu;
            } else {
                const bool held = (c1_later & i2c_stop) != 0u && (s2_later & i2c_msl) != 0u;
                print(serial, "  ", i2c_speed_hz(speed) / 1000u, " kHz, START withdrawn and STOP set in one store "
                      "at the NACK: a millisecond later CTLR1 ", hex(c1_later), " STAR2 ", hex(s2_later), " -> ",
                      held ? "STOP LEFT STANDING over a controller that holds the bus"
                           : "the STOP went out, the bus free",
                      crlf);
                if (held) {
                    ++wedged;
                }
            }
        }
    }
    bench.verdict("BTF stands while a STOP or repeated START requested at EVT8_2 goes out and falls by "
                  "itself within a few bits, both speeds - the requested repeated START then raising SB",
                  btf_ok);
    bench.verdict("a repeated START requested while a byte shifts survives that byte's NACK: SB rises "
                  "after AF, both speeds",
                  sr_ok);
    bench.verdict("the withdrawal at that NACK races the controller: at one speed at least it left STOP "
                  "standing over a controller that holds the bus - why the engine touches nothing there",
                  wedged != 0u);
    bench.verdict("and the command channel is back", command(Op::ping));
}

// ===========================================================================
// l - the first bytes of a write, at the entry that clears ADDR
// ===========================================================================
//
// The host's ADDR entry loads the shifter and the data register (15.3, EVT8_1
// then EVT8) instead of raising the buffer vector for them. What the silicon
// does there is asked raw first, the vectors dark; then the host's own
// tenures are judged by their entries, their statuses and the peer's tally.

/// START and the address with the write bit, the vectors dark, up to ADDR:
/// STAR1 as it stood there, 0xFFFF when a step never came.
uint16_t raw_to_addr() {
    H::start();
    if (raw_wait(i2c_sb, 2000) == 0xFFFF'FFFFu) {
        return 0xFFFF;
    }
    H::data(static_cast<uint8_t>(twilink::dut_addr << 1));
    if (raw_wait(i2c_addr, 2000) == 0xFFFF'FFFFu) {
        return 0xFFFF;
    }
    return H::status1();
}

/// Three ways of storing two bytes behind ADDR's clear: TxE polled before the
/// second store, the host's order; one STAR1 read between the stores, its
/// answer ignored; the two stores back to back. A store over a byte not yet
/// in the shifter would replace it, and the peer's tally says so.
enum class Load : uint8_t { polled, one_read, back_to_back };

struct LoadRow {
    uint16_t at_addr = 0;        ///< STAR1 at ADDR
    uint16_t after_clear = 0;    ///< STAR1 right after ADDR's clear
    uint16_t after_second = 0;   ///< STAR1 right after the second store
    uint16_t polls_max = 0;      ///< STAR1 reads that found TxE down behind the first store
    uint8_t done = 0;            ///< tenures that reached BTF and their STOP
    uint16_t sum = 0;            ///< what the peer should have summed
};

constexpr uint8_t load_tenures = 8;

LoadRow load_row(Load how, I2cSpeed speed) {
    LoadRow row{};
    raw_up(speed);
    for (uint8_t k = 0; k < load_tenures; ++k) {
        const uint8_t b1 = static_cast<uint8_t>(0x10u + k);
        const uint8_t b2 = static_cast<uint8_t>(0x80u + k);
        row.at_addr = raw_to_addr();
        if (row.at_addr == 0xFFFFu) {
            break;
        }
        (void)H::clear_addr();
        row.after_clear = H::status1();
        H::data(b1);
        if (how == Load::polled) {
            uint16_t polls = 0;
            while ((H::status1() & i2c_txe) == 0u && polls < 2000u) {
                ++polls;
            }
            row.polls_max = polls > row.polls_max ? polls : row.polls_max;
        } else if (how == Load::one_read) {
            (void)H::status1();
        }
        H::data(b2);
        row.after_second = H::status1();
        row.sum = static_cast<uint16_t>(row.sum + b1 + b2);
        if (raw_wait(i2c_btf, 2000) == 0xFFFF'FFFFu) {
            H::stop();
            break;
        }
        H::stop();
        ++row.done;
        settle_ms(1);
    }
    return row;
}

/// One serve's worth of the host's writes of `n` bytes, judged by the entries
/// each took, its status and the peer's count, sum and first byte. THE ENTRIES
/// ARE COUNTED, NOT BOUNDED: SB, ADDR and BTF, and one TxE a byte past the two
/// the ADDR entry loads: 3, 3, 4, 5 for 1, 2, 3, 4 bytes. One more is a STAR1
/// read that found TxE down and left the byte to the buffer vector.
bool load_writes(I2cSpeed speed) {
    twilink::Params a{};
    a.ms = 200;
    a.addr = twilink::dut_addr;
    if (!peer_act(Op::serve, a)) {
        return false;
    }
    bool good = true;
    uint16_t sum = 0;
    uint8_t took = 0;
    uint32_t ev[4] = {};
    uint8_t sts[4] = {};
    for (uint8_t n = 1; n <= 4u; ++n) {
        for (uint8_t i = 0; i < n; ++i) {
            tx_buf[i] = static_cast<uint8_t>(0x21u * n + i);
            sum = static_cast<uint16_t>(sum + tx_buf[i]);
        }
        took = static_cast<uint8_t>(took + n);
        sts[n - 1u] = host_tenure(twilink::dut_addr, tx_buf, n, nullptr, 0, speed);
        ev[n - 1u] = host_isr_entries;
        good = good && sts[n - 1u] == i2c_ok && ev[n - 1u] == (n < 3u ? 3u : n + 1u);
    }
    host_ready();
    settle_ms(220);
    twilink::Report r{};
    good = good && peer_report(r) && r.aux0 == took && r.sum == sum && r.addr_hits == 4u;
    print(serial, "  ", i2c_speed_hz(speed) / 1000u, " kHz, writes of 1, 2, 3, 4: status ", sts[0], "/", sts[1], "/", sts[2],
          "/", sts[3], ", event entries ", ev[0], "/", ev[1], "/", ev[2], "/", ev[3], " (3/3/4/5); the peer took ", r.aux0,
          " (", took, ") sum ", hex(r.sum), " (", hex(sum), ")", good ? "" : "  <- WRONG", crlf);
    return good;
}

/// A write of `len` with byte `nack_at` refused, its first bytes committed by
/// the ADDR entry, then what the next tenure does. THE STALE BYTE CHOSEN TO
/// TELL: a refusal of the first byte leaves the second in the data register,
/// and `stale` is an address. Had it opened the next write, that write's
/// answer would be that address's, not its own. The peer stays deaf after its
/// refusal until its window closes (twi_peer's header), so the next write is
/// set behind that window.
struct RefusedRow {
    uint8_t status = no_answer;
    uint16_t s1 = 0;
    uint16_t c1 = 0;
    uint8_t next = no_answer;
    uint8_t took = 0;
    uint16_t sum = 0;
    bool nacked = false;
};

RefusedRow refused_row(uint8_t len, uint8_t nack_at, uint8_t stale, uint8_t next_addr, I2cSpeed speed) {
    RefusedRow row{};
    twilink::Params f{};
    f.ms = 60;
    f.addr = twilink::dut_addr;
    f.nack_at = nack_at;
    if (!peer_act(Op::serve, f)) {
        return row;
    }
    for (uint8_t i = 0; i < len; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x30u + i);
    }
    tx_buf[nack_at] = stale;   // the byte behind the refused one, when there is one
    row.status = host_tenure(twilink::dut_addr, tx_buf, len, nullptr, 0, speed);
    settle_ms(1);
    row.s1 = H::status1();
    row.c1 = H::regs().CTLR1;
    settle_ms(80);   // the serve's window over, the command address back
    if (next_addr == twilink::command_addr) {
        // A write to the command address, its bytes judged by the peer's own
        // decoder, and the read of its acknowledgement.
        row.next = command_once(Op::ping, no_payload, 0) ? i2c_ok : no_answer;
    } else {
        row.next = host_tenure(next_addr, tx_buf, 1, nullptr, 0, link_speed);
    }
    twilink::Report r{};
    if (peer_report(r)) {
        row.took = r.aux0;
        row.sum = r.sum;
        row.nacked = (r.flags & twilink::report_nacked) != 0u;
    }
    return row;
}

void tl_first_bytes() {
    if (!need_peer()) {
        return;
    }
    bool order_ok = true;
    bool flags_ok = true;
    uint16_t polls_max = 0;
    bool writes_ok = true;
    bool wr_ok = true;
    bool stop_ok = true;
    bool first_refused_ok = true;
    bool stale_ok = true;
    for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
        const uint32_t khz = i2c_speed_hz(speed) / 1000u;
        // ---- 1. the silicon, raw ----
        for (const Load how : {Load::polled, Load::one_read, Load::back_to_back}) {
            twilink::Params a{};
            a.ms = 100;
            a.addr = twilink::dut_addr;
            if (!peer_act(Op::serve, a)) {
                bench.verdict("the peer accepted the serve", false);
                return;
            }
            const LoadRow row = load_row(how, speed);
            raw_down();
            settle_ms(120);
            twilink::Report r{};
            const bool rep = peer_report(r);
            const bool exact = rep && row.done == load_tenures && r.aux0 == 2u * load_tenures && r.sum == row.sum;
            print(serial, "  ", khz, " kHz raw, ",
                  how == Load::polled ? "TxE polled: " : how == Load::one_read ? "one read:   " : "back to back:",
                  " STAR1 at ADDR ", hex(row.at_addr), ", after its clear ", hex(row.after_clear), ", after the second store ",
                  hex(row.after_second), "; TxE found down ", row.polls_max, " times; the peer took ", r.aux0, " (",
                  2u * load_tenures, ") sum ", hex(r.sum), " (", hex(row.sum), ")", exact ? "" : "  <- LOST", crlf);
            flags_ok = flags_ok && (row.at_addr & i2c_txe) != 0u && (row.after_clear & i2c_txe) != 0u &&
                       (row.after_second & i2c_txe) == 0u;
            if (how == Load::polled) {
                order_ok = order_ok && exact;
                polls_max = row.polls_max > polls_max ? row.polls_max : polls_max;
            }
        }
        // ---- 2. the host: plain writes ----
        writes_ok = load_writes(speed) && writes_ok;
        // ---- 3. the host: a write before a read ----
        // SB, ADDR with the index and the repeated START, SB, ADDR, RxNE: 5.
        // Two written: one TxE more, for the repeated START: 6.
        {
            twilink::Params a{};
            a.ms = 100;
            a.addr = twilink::dut_addr;
            a.seed = 0x60;
            a.pattern = twilink::pattern_counting;
            if (!peer_act(Op::serve, a)) {
                bench.verdict("the peer accepted the serve", false);
                return;
            }
            tx_buf[0] = 0x4A;
            tx_buf[1] = 0x4B;
            rx_buf[0] = 0xEE;
            const uint8_t st1 = host_tenure(twilink::dut_addr, tx_buf, 1, rx_buf, 1, speed);
            const uint32_t ev1 = host_isr_entries;
            const uint8_t got1 = rx_buf[0];
            rx_buf[0] = 0xEE;
            const uint8_t st2 = host_tenure(twilink::dut_addr, tx_buf, 2, rx_buf, 1, speed);
            const uint32_t ev2 = host_isr_entries;
            const uint8_t got2 = rx_buf[0];
            host_ready();
            settle_ms(120);
            twilink::Report r{};
            const bool good = peer_report(r) && st1 == i2c_ok && st2 == i2c_ok && ev1 == 5u && ev2 == 6u &&
                              got1 == twilink::pattern_value(a.pattern, a.seed, 0) &&
                              got2 == twilink::pattern_value(a.pattern, a.seed, 1) && r.aux0 == 3u &&
                              r.sum == 0x4Au + 0x4Au + 0x4Bu;
            wr_ok = wr_ok && good;
            print(serial, "  ", khz, " kHz, 1 + 1 and 2 + 1: status ", st1, "/", st2, ", event entries ", ev1, "/", ev2,
                  " (5/6), read ", hex(got1), " ", hex(got2), ", the peer took ", r.aux0, " (3) sum ", hex(r.sum),
                  good ? "" : "  <- WRONG", crlf);
        }
        // ---- 4. the STOP's place: a refused last byte is never i2c_ok ----
        for (const uint8_t n : {uint8_t{1}, uint8_t{2}}) {
            const RefusedRow row = refused_row(n, n, 0x00, twilink::command_addr, speed);
            const bool good = row.status == i2c_nack_data && row.nacked && row.took == n;
            stop_ok = stop_ok && good;
            print(serial, "  ", khz, " kHz, ", n, " written, the last refused: status ", row.status, ", the peer took ",
                  row.took, ", the next write ", row.next, good ? "" : "  <- WRONG", crlf);
        }
        // ---- 5. the first byte refused, the second committed in DATAR ----
        // The refused byte is the peer's one: its count and sum hold it alone
        // if the committed byte never went out. Two aims for the stale byte:
        // the next write at nobody with the stale byte the peer's command
        // address (a stale opening would be acknowledged, an i2c_ok); the next
        // write at the command address, a whole frame, with the stale byte
        // nobody's (a stale opening would be refused, the frame never taken).
        for (const uint8_t len : {uint8_t{2}, uint8_t{4}}) {
            const uint8_t stale_cmd = static_cast<uint8_t>(twilink::command_addr << 1);
            const uint8_t stale_nobody = static_cast<uint8_t>(nobody_addr << 1);
            const RefusedRow a = refused_row(len, 1, stale_cmd, nobody_addr, speed);
            const RefusedRow b = refused_row(len, 1, stale_nobody, twilink::command_addr, speed);
            const bool one = a.status == i2c_nack_data && b.status == i2c_nack_data && a.took == 1u && b.took == 1u &&
                             a.sum == 0x30u && b.sum == 0x30u && a.nacked && b.nacked;
            const bool stale = a.next == i2c_nack_addr && b.next == i2c_ok;
            first_refused_ok = first_refused_ok && one;
            stale_ok = stale_ok && stale;
            print(serial, "  ", khz, " kHz, ", len, " written, the first refused: status ", a.status, "/", b.status,
                  ", STAR1 ", hex(a.s1), " CTLR1 ", hex(a.c1), " a millisecond later; the peer took ", a.took, "/", b.took,
                  " sum ", hex(a.sum), "/", hex(b.sum), "; the next write, the stale byte another address's: ", a.next,
                  " (i2c_nack_addr ", i2c_nack_addr, ") and ", b.next, " (i2c_ok ", i2c_ok, ")", one && stale ? "" : "  <- WRONG",
                  crlf);
        }
    }
    print(serial, "  TxE found down behind the first store, the host's order: at most ", polls_max, " STAR1 reads", crlf);
    bench.verdict("the flags as the chapter draws them: TxE up at ADDR and after its clear (EVT8_1), down behind the "
                  "second store, both speeds",
                  flags_ok);
    bench.verdict("two bytes stored behind ADDR's clear, TxE read between, byte-exact at the peer, both speeds", order_ok);
    bench.verdict("the host's plain writes of 1, 2, 3, 4: i2c_ok, byte-exact, SB, ADDR and BTF and one entry a byte past "
                  "the two the ADDR entry loads, both speeds",
                  writes_ok);
    bench.verdict("the host's 1 + 1 and 2 + 1: i2c_ok, byte-exact, five and six entries, both speeds", wr_ok);
    bench.verdict("the STOP stays at BTF: a last byte refused is i2c_nack_data, never i2c_ok, both speeds", stop_ok);
    bench.verdict("the first byte refused with the second committed: i2c_nack_data, and the peer never saw the second, "
                  "both speeds",
                  first_refused_ok);
    bench.verdict("the committed byte opens no next write: that write answers for its own address, its frame whole, both speeds", stale_ok);
    bench.verdict("and the command channel is back", command(Op::ping));
}

// ===========================================================================
// u - a BUSY left standing over an idle wire
// ===========================================================================
//
// 15.10.7: BUSY is "low level exists in SDA or SCL", cleared when a stop bit is
// detected, and 15.10.1 names SWRST for "the stop condition is not detected on
// the bus but the bus bit is 1". start() takes it when BUSY stood for the whole
// of bus_free_us and no read found a line low; the letter says what a request
// then meets. Stages by hand on the two pads, then the way nothing but the
// wire's own life produces it: a client holding the clock past the per-bus
// timeout, recover() run under its low SCL, the client letting go. The third
// way, the pull-ups' reset - the peer board restarted, both lines down and up -
// cannot be staged from here; what the letter's entry finds, and the first
// request behind it, is what the last one left (the block must have been up).

struct Staged {
    bool busy_mid;
    bool busy_end;
    bool lines_high;
    bool busy_later;
};

enum class Stage : uint8_t {
    start_no_stop,       ///< START by hand, SDA let go under a low clock, then SCL
    scl_pulse,           ///< SCL alone low and let go, SDA high throughout
    sda_pulse,           ///< SDA alone: a START and its STOP, the control
    both_low_sda_first,  ///< both low over recover(), SDA let go first, then SCL
    both_low_scl_first,  ///< both low over recover(), SCL let go first: a STOP, the control
    scl_low_recover,     ///< SCL alone low over recover(), then let go
    pe_over_start,       ///< START by hand, PE down, no STOP, PE up
    gate_over_start,     ///< START by hand, gate shut, no STOP, gate open
    reset_over_start,    ///< START by hand, the reset line, no STOP
    tenure_recovered,    ///< a real tenure cut mid-byte by recover()
    count,
};

const char* stage_name(Stage s) {
    switch (s) {
        case Stage::start_no_stop: return "START, SDA up under a low SCL, SCL up";
        case Stage::scl_pulse: return "SCL alone low, up";
        case Stage::sda_pulse: return "SDA alone low, up (a STOP, the control)";
        case Stage::both_low_sda_first: return "both low over recover(), SDA up, SCL up";
        case Stage::both_low_scl_first: return "both low over recover(), SCL up, SDA up (a STOP, the control)";
        case Stage::scl_low_recover: return "SCL low over recover(), up";
        case Stage::pe_over_start: return "START, PE down, no STOP, PE up";
        case Stage::gate_over_start: return "START, gate shut, no STOP, gate open";
        case Stage::reset_over_start: return "START, the reset line, no STOP";
        case Stage::tenure_recovered: return "a tenure cut mid-byte by recover()";
        default: return "?";
    }
}

/// A line pulled low from its pad, and given back to the peripheral; a line
/// the suite holds for twenty microseconds, over any bit's edge at both speeds.
template <class Line>
void line_low() {
    Line::output(false, PinDrive::open_drain);
    (void)delay_us(clock, 20);
}
template <class Line>
void line_up() {
    Line::function(PinDrive::open_drain);
    (void)delay_us(clock, 20);
}

Staged stage_busy(Stage s) {
    Staged out{};
    host_ready();
    (void)delay_us(clock, 50);
    const auto no_stop = [] {
        line_low<SclPin>();
        line_up<SdaPin>();
        line_up<SclPin>();
    };
    switch (s) {
        case Stage::start_no_stop:
            line_low<SdaPin>();
            out.busy_mid = H::busy();
            no_stop();
            break;
        case Stage::scl_pulse:
            line_low<SclPin>();
            out.busy_mid = H::busy();
            line_up<SclPin>();
            break;
        case Stage::sda_pulse:
            line_low<SdaPin>();
            out.busy_mid = H::busy();
            line_up<SdaPin>();
            break;
        case Stage::both_low_sda_first:
        case Stage::both_low_scl_first:
            line_low<SdaPin>();
            line_low<SclPin>();
            (void)Host::recover();
            (void)delay_us(clock, 20);
            out.busy_mid = H::busy();
            if (s == Stage::both_low_sda_first) {
                line_up<SdaPin>();
                line_up<SclPin>();
            } else {
                line_up<SclPin>();
                line_up<SdaPin>();
            }
            break;
        case Stage::scl_low_recover:
            line_low<SclPin>();
            (void)Host::recover();
            (void)delay_us(clock, 20);
            out.busy_mid = H::busy();
            line_up<SclPin>();
            break;
        case Stage::pe_over_start:
            line_low<SdaPin>();
            H::disable();
            out.busy_mid = H::busy();
            no_stop();
            H::enable();
            break;
        case Stage::gate_over_start:
            line_low<SdaPin>();
            H::bus_clock(false);
            no_stop();
            H::bus_clock(true);
            out.busy_mid = H::busy();
            break;
        case Stage::reset_over_start:
            line_low<SdaPin>();
            H::reset();
            H::timing(*i2c_timing_for(SysClock::hz, link_speed));
            H::enable();
            (void)delay_us(clock, 20);
            out.busy_mid = H::busy();
            no_stop();
            break;
        case Stage::tenure_recovered: {
            Host::Request r{};
            r.addr = twilink::command_addr;
            r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
            r.tx_len = 8;
            r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
            r.rx_len = 0;
            r.speed = link_speed;
            host_done = false;
            (void)Host::start(r);
            (void)delay_us(clock, 300);   // at 100 kHz, inside the second data byte
            out.busy_mid = H::busy();
            (void)Host::recover();
            break;
        }
        default:
            break;
    }
    (void)delay_us(clock, 50);
    out.lines_high = SclPin::read() && SdaPin::read();
    out.busy_end = H::busy();
    settle_ms(2);
    out.busy_later = H::busy();
    return out;
}

/// The probe of the peer's command address a request names here.
Host::Request command_probe(I2cSpeed speed) {
    Host::Request r = kl::request(twilink::command_addr, kl::out_a);
    r.tx_len = 0;
    r.speed = speed;
    return r;
}

/// One request through the arbiter: its one reply and the time to it. THE
/// BOUND ON A TIMEOUT: the per-bus timeout is 20 ms of ticks of 1 ms, armed in
/// the dispatch that calls start(), so it fires between 19 and 20 ms behind its
/// arming, the dispatch's own wait comes before that, and the tick and the
/// kernel's loop add under a millisecond: from 19 to 22 ms.
struct ArbAnswer {
    uint8_t replies;
    uint8_t status;
    uint32_t us;
};
constexpr uint32_t arb_timeout_min_us = 19'000;
constexpr uint32_t arb_timeout_max_us = 22'000;

/// THE BOUND ON start() OVER A BUSY LEFT STANDING. The watch's 100 us, its last
/// poll and the reset's stores, under 10 us; the tenure behind it, a probe: at
/// 100 kHz nine SCL periods at the slowest rate letter f passes, 120 us, with
/// the START, the STOP and the vector's entries, 25 (letter j's arithmetic): 255
/// us in all. At 400 kHz 9 x 3.33 = 30 us and the same 25: 165 us. A PE cycle
/// first, for the other speed, is two stores.
constexpr uint32_t standing_100k_us = 255;
constexpr uint32_t standing_400k_us = 165;
/// THROUGH AN ARBITER: the same, the kernel's turn and a tick's entry beside it.
/// The post waits for the dispatch and the reply for the probe's turn, under
/// 2 ms between them on a loop with nothing else to do (the first reply's
/// figure is printed).
constexpr uint32_t standing_arb_us = 2'000;

ArbAnswer ask_arbiter(const Host::Request& r) {
    kl::Probe::clear_tally();
    const uint32_t c0 = Ticker::cycles();
    post<kl::I2cArb>(r);
    kl::pump_until(1, 100);
    const uint32_t us = (Ticker::cycles() - c0) / (SysClock::hz / 1'000'000u);
    return {kl::Probe::n, kl::Probe::replies[0], us};
}

ArbAnswer ask_untimed(const Host::Request& r, uint32_t ms) {
    kl::Probe::clear_tally();
    const uint32_t c0 = Ticker::cycles();
    post<kl::I2cArbUntimed>(r);
    kl::pump_until(1, ms);
    const uint32_t us = (Ticker::cycles() - c0) / (SysClock::hz / 1'000'000u);
    return {kl::Probe::n, kl::Probe::replies[0], us};
}

bool timed_out(const ArbAnswer& a) {
    return a.replies == 1u && a.status == i2c_timeout && a.us >= arb_timeout_min_us && a.us <= arb_timeout_max_us;
}

void tu_unwedge() {
    (void)wire_pulled_up();   // the bus port's gate
    const bool entry_standing = H::enabled() && H::busy() && SclPin::read() && SdaPin::read();
    print(serial, "  at the letter's entry, as the wire left it: the block ", H::enabled() ? "up" : "not up", ", BUSY ",
          H::busy() ? 1 : 0, ", lines SCL ", SclPin::read() ? 1 : 0, " SDA ", SdaPin::read() ? 1 : 0, crlf);
    if (entry_standing) {
        // The first request behind a far board's reset, with the block as the wire
        // left it: the peer's command channel probed through what the wire left.
        const uint16_t resets0 = Host::unwedges();
        const uint8_t st = host_tenure(twilink::command_addr, nullptr, 0, nullptr, 0, link_speed);
        print(serial, "  a request over it: status ", st, ", resets taken ", Host::unwedges() - resets0, ", BUSY ",
              H::busy() ? 1 : 0, crlf);
    }
    if (!need_peer()) {
        return;
    }
    // ---- 1. the stagings, and how often each leaves it ----
    constexpr uint8_t tries = 8;
    uint8_t sibling_way = 0;
    uint8_t scl_alone = 0;
    uint8_t controls = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(Stage::count); ++i) {
        const Stage s = static_cast<Stage>(i);
        uint8_t mid = 0;
        uint8_t standing = 0;
        uint8_t later = 0;
        for (uint8_t k = 0; k < tries; ++k) {
            const Staged w = stage_busy(s);
            mid = static_cast<uint8_t>(mid + (w.busy_mid ? 1u : 0u));
            standing = static_cast<uint8_t>(standing + (w.busy_end && w.lines_high ? 1u : 0u));
            later = static_cast<uint8_t>(later + (w.busy_later && w.lines_high ? 1u : 0u));
            (void)Host::recover();
            settle_ms(1);
        }
        print(serial, "  ", stage_name(s), ": BUSY in the middle ", mid, " of ", tries, ", standing over both lines high ",
              standing, ", still 2 ms later ", later, crlf);
        if (s == Stage::start_no_stop) {
            sibling_way = later;
        } else if (s == Stage::scl_pulse) {
            scl_alone = later;
        } else if (s == Stage::sda_pulse || s == Stage::both_low_scl_first) {
            controls = static_cast<uint8_t>(controls + later);
        }
    }
    // ---- 2. what clears it, the wire and the block left to themselves ----
    uint8_t swrst_cleared = 0;
    for (uint8_t way = 0; way < 4u; ++way) {
        uint8_t cleared = 0;
        for (uint8_t k = 0; k < tries; ++k) {
            const Staged w = stage_busy(k % 2u == 0u ? Stage::start_no_stop : Stage::scl_pulse);
            if (way == 0u) {
                hold_bus_busy(20);
            } else if (way == 1u) {
                H::disable();
                H::enable();
            } else if (way == 2u) {
                H::software_reset();
                H::timing(*i2c_timing_for(SysClock::hz, link_speed));
                H::enable();
            } else {
                settle_ms(50);
            }
            (void)delay_us(clock, 20);
            cleared = static_cast<uint8_t>(cleared + (w.busy_end && !H::busy() ? 1u : 0u));
            (void)Host::recover();
        }
        if (way == 2u) {
            swrst_cleared = cleared;
        }
        print(serial, "  ",
              way == 0u   ? "a START and a STOP on the wire"
              : way == 1u ? "a PE cycle"
              : way == 2u ? "SWRST"
                          : "fifty milliseconds alone",
              " took BUSY down ", cleared, " of ", tries, crlf);
    }
    // What SWRST takes with it, read straight behind the pulse, then put back.
    {
        host_ready();
        H::software_reset();
        const uint16_t c1 = H::regs().CTLR1;
        const uint16_t c2 = H::regs().CTLR2;
        const uint16_t ck = H::regs().CKCFGR;
        host_ready();
        print(serial, "  SWRST leaves CTLR1 ", hex(c1), " CTLR2 ", hex(c2), " CKCFGR ", hex(ck), crlf);
    }
    // ---- 3. what a request meets, raw on the host: the watch, SWRST, the wire ----
    bool request_clean = true;
    for (const I2cSpeed speed : {link_speed, I2cSpeed::fast_400k}) {
        for (const Stage st : {Stage::start_no_stop, Stage::scl_pulse}) {
            uint32_t start_hi = 0;
            uint32_t done_hi = 0;
            uint32_t ev_hi = 0;
            uint8_t clean = 0;
            for (uint8_t k = 0; k < tries; ++k) {
                const Staged w = stage_busy(st);
                const uint16_t resets0 = Host::unwedges();
                const Host::Request r = command_probe(speed);
                host_done = false;
                host_isr_entries = 0;
                error_isr_entries = 0;
                constexpr uint32_t per_us = SysClock::hz / 1'000'000u;
                const uint32_t c0 = Ticker::cycles();
                const bool sync = Host::start(r);
                const uint32_t start_us = (Ticker::cycles() - c0) / per_us;
                while (!host_done && Ticker::cycles() - c0 < 30'000u * per_us) {
                }
                const uint32_t done_us = (Ticker::cycles() - c0) / per_us;
                const bool answered = host_done;
                const uint32_t ev = host_isr_entries;
                const uint32_t er = error_isr_entries;
                const uint8_t status = Host::status();
                if (!answered) {
                    (void)Host::recover();
                }
                settle_ms(1);
                start_hi = start_us > start_hi ? start_us : start_hi;
                done_hi = done_us > done_hi ? done_us : done_hi;
                ev_hi = ev > ev_hi ? ev : ev_hi;
                const uint32_t limit = speed == I2cSpeed::fast_400k ? standing_400k_us : standing_100k_us;
                if (w.busy_end && !sync && answered && status == i2c_ok && er == 0u && done_us <= limit &&
                    Host::unwedges() == resets0 + 1u && !H::busy()) {
                    ++clean;
                }
            }
            print(serial, "  ", i2c_speed_hz(speed) / 1000u, " kHz (", speed == link_speed ? "in force" : "a PE cycle first",
                  ") over \"", stage_name(st), "\": i2c_ok with one reset ", clean, " of ", tries, ", start() at most ",
                  start_hi, " us, answered at most ", done_hi, " us, event entries at most ", ev_hi, crlf);
            request_clean = request_clean && clean == tries;
        }
    }
    // ---- 4. through the arbiter ----
    (void)stage_busy(Stage::start_no_stop);
    kl::BusKernel::init_all();
    bus_ao_live = true;
    const ArbAnswer wedged = ask_arbiter(command_probe(link_speed));
    const ArbAnswer behind = ask_arbiter(command_probe(link_speed));
    print(serial, "  through the arbiter: ", wedged.replies, " reply, status ", wedged.status, " after ", wedged.us,
          " us; the next request ", behind.status, " after ", behind.us, " us", crlf);
    kl::drain(5);
    bus_ao_live = false;
    // ---- 4b. the same with no per-bus timeout ----
    (void)stage_busy(Stage::start_no_stop);
    kl::BusKernel::init_all();
    bus_ao_live = true;
    bus_ao_untimed = true;
    const ArbAnswer wedged_untimed = ask_untimed(command_probe(link_speed), 100);
    kl::drain(2);
    bus_ao_untimed = false;
    bus_ao_live = false;
    print(serial, "  no per-bus timeout, over it: ", wedged_untimed.replies, " reply, status ", wedged_untimed.status,
          " after ", wedged_untimed.us, " us", crlf);
    // A client holding the clock, then letting go, under the arbiter with no
    // timeout: nothing cuts the tenure, so no BUSY is left behind it. The peer
    // stretches each byte 45 ms; the deadline of its serve covers the two holds
    // a write of two bytes meets, and the second falls behind the reply, its
    // STOP standing under it (the client takes the last byte one byte late). The
    // request posted right behind that reply runs stop_drain_us out and start()
    // answers it (letter t derives the bound and judges both speeds).
    host_ready();
    settle_ms(10);
    bool held_through = false;
    bool behind_answered = false;
    {
        twilink::Params held{};
        held.count = 16;
        held.ms = 200;
        held.addr = twilink::dut_addr;
        held.hold_us = 45'000;
        bool accepted = peer_act(Op::serve, held);
        kl::BusKernel::init_all();
        bus_ao_live = true;
        bus_ao_untimed = true;
        Host::Request w = kl::request(twilink::dut_addr, kl::out_a);
        w.tx_len = 2;
        w.speed = link_speed;
        const ArbAnswer through = ask_untimed(w, 150);
        // The request right behind the reply, while the client still holds.
        const ArbAnswer behind_hold = ask_untimed(command_probe(link_speed), 150);
        print(serial, "  no per-bus timeout, a client holding the clock 45 ms: accepted ", accepted ? 1 : 0, ", the write ",
              through.replies, " reply, status ", through.status, " after ", through.us,
              " us; a request behind it under the hold: ", behind_hold.replies, " reply, status ", behind_hold.status, " after ",
              behind_hold.us, " us; BUSY ", H::busy() ? 1 : 0, ", STOP standing ", H::stopping() ? 1 : 0, crlf);
        // THE BOUND ON THE WRITE'S REPLY: the 45 ms hold at the address, the three
        // bytes' 27 clocks at the slowest rate letter f passes, 360 us, and the
        // kernel's turn of standing_arb_us.
        held_through = accepted && through.replies == 1u && through.status == i2c_ok && through.us >= 45'000u &&
                       through.us <= 45'000u + 360u + standing_arb_us;
        // THE BOUND ON THE ANSWER BEHIND IT: stop_drain_us and the kernel's turn.
        behind_answered = behind_hold.replies == 1u && behind_hold.status == i2c_timeout && behind_hold.us >= 5'000u &&
                          behind_hold.us <= 5'000u + standing_arb_us;
        kl::drain(2);
        bus_ao_untimed = false;
        bus_ao_live = false;
        host_ready();
        settle_ms(220);
    }
    // ---- 5. no hand on the wire: a client holding the clock past the limit ----
    // The peer stretches the address for 45 ms under the arbiter's 20: its
    // i2c_timeout and recover() come under a low SCL, and SWRST leaves BUSY up
    // over it. The client lets go with no STOP. The first request behind it
    // meets what the stagings left and takes the reset; the second a clean block.
    bool held_seen = true;
    bool held_next_ok = true;
    for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
        host_ready();
        settle_ms(10);
        twilink::Params held{};
        held.count = 16;
        held.ms = 200;
        held.addr = twilink::dut_addr;
        held.hold_us = 45'000;
        if (!peer_act(Op::serve, held)) {
            bench.verdict("the peer accepted the 45 ms stretch", false);
            return;
        }
        kl::BusKernel::init_all();
        bus_ao_live = true;
        kl::drain(5);
        Host::Request r = kl::request(twilink::dut_addr, kl::out_a);
        r.speed = speed;
        const ArbAnswer first = ask_arbiter(r);
        const bool scl_low = !SclPin::read();
        const bool busy_low = H::busy();
        kl::drain(100);   // the hold's 45 ms over, and the address's byte alone left to the peer
        const bool lines_high = SclPin::read() && SdaPin::read();
        const bool busy_idle = H::busy();
        const uint16_t resets0 = Host::unwedges();
        const ArbAnswer second = ask_arbiter(command_probe(speed));
        const bool second_reset = Host::unwedges() == resets0 + 1u;
        kl::drain(2);
        const ArbAnswer third = ask_arbiter(command_probe(speed));
        kl::drain(2);
        bus_ao_live = false;
        print(serial, "  a client holding the clock 45 ms, ", i2c_speed_hz(speed) / 1000u, " kHz: status ", first.status,
              " after ", first.us, " us, SCL low at the reply ", scl_low ? 1 : 0, ", BUSY ", busy_low ? 1 : 0,
              "; the client gone, lines high ", lines_high ? 1 : 0, ", BUSY ", busy_idle ? 1 : 0,
              "; the request behind it: status ", second.status, " after ", second.us, " us, a reset ",
              second_reset ? 1 : 0, "; the next: status ", third.status, " after ", third.us, " us", crlf);
        // A wire outcome: i2c_nack_addr while the peer's serve keeps its client
        // on the other address, i2c_ok once it is back.
        const auto on_wire = [](const ArbAnswer& a) {
            return a.replies == 1u && (a.status == i2c_ok || a.status == i2c_nack_addr) && a.us < standing_arb_us;
        };
        held_seen = held_seen && timed_out(first) && scl_low && lines_high && busy_idle && on_wire(second) && second_reset;
        held_next_ok = held_next_ok && on_wire(third);
        settle_ms(250);
    }
    // ---- 6. reads, the way it was seen on another die: rate, not a verdict ----
    twilink::Params a{};
    a.ms = 200;
    a.addr = twilink::dut_addr;
    uint16_t reads = 0;
    uint16_t standing = 0;
    for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
        for (const bool dma : {false, true}) {
            host_ready();
            if (!peer_act(Op::serve, a)) {
                bench.verdict("the peer accepted the serve", false);
                return;
            }
            bring_up(dma);
            for (const uint8_t n : {uint8_t{1}, uint8_t{2}, uint8_t{3}, uint8_t{4}, uint8_t{16}}) {
                for (uint8_t k = 0; k < 4u; ++k) {
                    if (either_tenure(dma, nullptr, 0, rx_buf, n, speed) == i2c_ok) {
                        (void)delay_us(clock, 300);   // the STOP on the wire, over the slowest bit by far
                        ++reads;
                        standing = static_cast<uint16_t>(standing + (H::busy() ? 1u : 0u));
                    }
                }
            }
            put_down(dma);
            settle_ms(220);
        }
    }
    print(serial, "  reads of 1, 2, 3, 4 and 16 on the pump and on the engines, both speeds: BUSY standing 300 us after ",
          standing, " of ", reads, crlf);
    bench.verdict("the state is made by hand: a START with no STOP, and SCL low alone, each leaves BUSY up over both lines "
                  "high, every time; a STOP takes it down, every time",
                  sibling_way == tries && scl_alone == tries && controls == 0u);
    bench.verdict("SWRST takes it down, every time", swrst_cleared == tries);
    bench.verdict("A REQUEST at either speed takes SWRST behind the watch and is on the wire: i2c_ok, one reset counted, "
                  "inside the derived bound, BUSY down behind it",
                  request_clean);
    bench.verdict("THROUGH THE ARBITER: one reply, i2c_ok, no timeout spent, and the next request i2c_ok",
                  wedged.replies == 1u && wedged.status == i2c_ok && wedged.us < standing_arb_us && behind.replies == 1u &&
                      behind.status == i2c_ok);
    bench.verdict("WITH NO PER-BUS TIMEOUT the request over it is answered: one reply, i2c_ok",
                  wedged_untimed.replies == 1u && wedged_untimed.status == i2c_ok && wedged_untimed.us < standing_arb_us);
    bench.verdict("WITH NO PER-BUS TIMEOUT a client holding the clock, then letting go, cuts nothing: its write ends i2c_ok "
                  "behind the hold",
                  held_through);
    bench.verdict("and the request posted behind that reply, the client still holding, is answered i2c_timeout at 5 ms",
                  behind_answered);
    bench.verdict("NO HAND ON THE WIRE: a client holding the clock past the limit leaves it behind recover(), and the "
                  "request behind that takes the reset and reaches the wire, no second i2c_timeout",
                  held_seen);
    bench.verdict("and the one behind that reaches the wire", held_next_ok);
    host_ready();
    bench.verdict("and the command channel is back", command(Op::ping));
}

// ===========================================================================
// t - the two bounds of start() against their names
// ===========================================================================
//
// THE INSTRUMENT. Ticker::cycles() around start(), a count being 20.8 ns of HCLK;
// a read of it is 57 cycles (bench_ch32_i2c's ruler, on the same call), and the
// figures keep it: they read high by that, never low. The row with the STK stopped
// times on TIM2 at 1 us a count, the instrument's resolution there.
//
// What brackets a figure. Below: the name, bus_free_us x 48 or stop_drain_us x 48
// cycles, since the wait ends no earlier. Above: the same plus what start() costs
// around the wait, which the free bus's row prints (`free`), one poll (poll_max,
// below) because the wait looks at the counter once a poll, one tick's entry of
// 220 cycles (the host's page) because a tick may land in that last poll, and the
// ruler's 57.
constexpr uint32_t bound_per_us = SysClock::hz / 1'000'000u;
constexpr uint32_t stop_drain_named_us = 5'000;
/// The longest poll, in cycles: the watch's, 63 measured with SCL low (the stopped
/// row's figure), the path that reads both lines high about the same by the listing.
constexpr uint32_t poll_max = 80;
constexpr uint32_t tick_entry = 220;
constexpr uint32_t ruler = 57;
/// With the STK stopped: the budget is the bound over 8 cycles a poll.
constexpr uint32_t budget_of(uint32_t us) { return us * bound_per_us / 8u; }

/// What the last start_cycles() returned: true when start() answered the request itself.
bool start_answered = false;

uint32_t start_cycles(const Host::Request& r) {
    host_done = false;
    host_isr_entries = 0;
    console_drain();
    const uint32_t c0 = Ticker::cycles();
    start_answered = Host::start(r);
    return Ticker::cycles() - c0;
}

/// A write of two bytes the client holds, its reply waited for: behind it the
/// client still holds SCL with the host's STOP standing (one byte late, the
/// page's finding). The hold is 45 ms, over either bound with the counter
/// running or stopped.
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
/// THE BOUND, post to reply: stop_drain_us, then `over` (what start() costs around
/// the wait, a poll, a tick's entry, the ruler), then the kernel's turn, which is
/// read here on a request refused with nothing on the wire: the same post, dispatch
/// and reply with no wait in them. One microsecond for the truncation of each.
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
        bus_ao_live = true;
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
        console_drain();
        const ArbAnswer a = ask_of(timed, command_probe(speed), 30);
        const uint16_t c1 = H::regs().CTLR1;
        const bool untouched = (c1 & i2c_stop) != 0u && (c1 & i2c_start) == 0u && H::busy() && host_isr_entries == 0u;
        kl::drain(hold_us / 1000u + 2u);
        const bool freed = !H::stopping() && !H::busy();
        const ArbAnswer next = ask_of(timed, command_probe(speed), 30);
        kl::drain(2);
        bus_ao_untimed = false;
        bus_ao_live = false;
        const uint32_t ceiling = stop_drain_named_us + over / bound_per_us + 1u + turn.us + 1u;
        lo = a.us < lo ? a.us : lo;
        hi = a.us > hi ? a.us : hi;
        turn_hi = turn.us > turn_hi ? turn.us : turn_hi;
        next_hi = next.us > next_hi ? next.us : next_hi;
        if (accepted && turn.replies == 1u && turn.status == i2c_rejected && through.replies == 1u && through.status == i2c_ok &&
            stood && a.replies == 1u && a.status == i2c_timeout && a.us >= stop_drain_named_us && a.us <= ceiling && untouched &&
            freed && next.replies == 1u && (next.status == i2c_ok || next.status == i2c_nack_addr) && next.us < standing_arb_us &&
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
/// is run_out_row()'s twice over and the write's, printed with what it took.
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
    bus_ao_live = true;
    Host::Request w = kl::request(twilink::dut_addr, kl::out_a);
    w.tx_len = 2;
    w.speed = link_speed;
    kl::Probe::clear_tally();
    const uint32_t c0 = Ticker::cycles();
    post<kl::I2cArb>(w);
    post<kl::I2cArb>(command_probe(link_speed));
    post<kl::I2cArb>(command_probe(link_speed));
    kl::pump_until(3, 60);
    const uint32_t us = (Ticker::cycles() - c0) / bound_per_us;
    const uint8_t n = kl::Probe::n;
    kl::drain(hold_us / 1000u + 2u);
    const bool freed = !H::stopping() && !H::busy();
    bus_ao_live = false;
    print(serial, "  two requests waiting behind a 15 ms hold: replies ", n, ", status ", kl::Probe::replies[0], " ",
          kl::Probe::replies[1], " ", kl::Probe::replies[2], ", all in ", us, " us", crlf);
    const bool ok = accepted && n == 3u && kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_timeout &&
                    kl::Probe::replies[2] == i2c_timeout && us >= hold_us + 2u * stop_drain_named_us &&
                    us <= hold_us + 360u + 2u * (stop_drain_named_us + over / bound_per_us + 1u) + standing_arb_us && freed;
    host_ready();
    settle_ms(220);
    return ok;
}

/// THE RAW QUESTION UNDER A STALL: START stored by hand, the vectors dark, into the
/// peer's probe of nobody, this block a bystander: the lines read just before the
/// store, then what becomes of it inside 3 ms, over a whole tenure: SB, or nothing
/// with BUSY standing over both lines high. The moment swept inside the tenure.
void live_raw_row(uint8_t aux8, uint32_t base, uint32_t step) {
    uint8_t sb[4] = {};
    uint8_t stuck[4] = {};
    uint8_t other[4] = {};
    for (uint8_t k = 0; k < 64u; ++k) {
        twilink::Params a = host_action(nobody_addr, 0, 0x55, 200);
        a.aux8 = aux8;
        if (!peer_act(Op::arb, a)) {
            continue;
        }
        host_ready();
        Pfic::disable(H::event_irq());
        Pfic::disable(H::error_irq());
        inject_start();
        spin_us(350);
        inject_stop();
        spin_us(base + step * (k % 32u));
        const uint8_t where = static_cast<uint8_t>((SclPin::read() ? 2u : 0u) + (SdaPin::read() ? 1u : 0u));
        H::start();
        const uint32_t c0 = Ticker::cycles();
        bool got = false;
        while (!got && Ticker::cycles() - c0 < 3000u * (SysClock::hz / 1'000'000u)) {
            got = (H::status1() & i2c_sb) != 0u;
        }
        if (got) {
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

void tt_bounds() {
    if (!need_peer()) {
        return;
    }
    // ---- the free bus: what start() costs around the wait ----
    constexpr uint8_t tries = 8;
    uint32_t free_lo = 0xFFFFFFFFu;
    uint32_t free_hi = 0;
    host_ready();
    for (uint8_t k = 0; k < tries; ++k) {
        settle_ms(1);
        const uint32_t c = start_cycles(command_probe(link_speed));
        free_lo = c < free_lo ? c : free_lo;
        free_hi = c > free_hi ? c : free_hi;
        kl::drain(1);
        settle_ms(1);
    }
    print(serial, "  free bus: start() ", free_lo, " to ", free_hi, " cycles, the ruler's ", ruler, " inside", crlf);
    const uint32_t over = free_hi + poll_max + tick_entry + ruler;
    // ---- bus_free_us, BUSY held by SCL low: the bound alone ----
    uint32_t held_lo = 0xFFFFFFFFu;
    uint32_t held_hi = 0;
    bool held_kept = true;
    for (uint8_t k = 0; k < tries; ++k) {
        host_ready();
        const uint16_t resets0 = Host::unwedges();
        line_low<SclPin>();
        const uint32_t c = start_cycles(command_probe(link_speed));
        held_lo = c < held_lo ? c : held_lo;
        held_hi = c > held_hi ? c : held_hi;
        held_kept = held_kept && Host::unwedges() == resets0;
        line_up<SclPin>();
        (void)Host::recover();
        settle_ms(1);
    }
    print(serial, "  bus_free_us ", bus_free_us_named, " with SCL held low: start() ", held_lo, " to ", held_hi, " cycles, ",
          (held_hi - free_lo) / bound_per_us, " us at most over the free bus's", crlf);
    bench.verdict("bus_free_us means what it says: with SCL held low start() spends its 100 us and no more than the "
                  "poll, the tick and the ruler over them, no reset taken",
                  held_lo >= bus_free_us_named * bound_per_us && held_hi <= bus_free_us_named * bound_per_us + over && held_kept);
    // ---- bus_free_us, BUSY standing over an idle wire: the bound and the reset ----
    uint32_t stand_lo = 0xFFFFFFFFu;
    uint32_t stand_hi = 0;
    bool stand_kept = true;
    for (uint8_t k = 0; k < tries; ++k) {
        const Staged w = stage_busy(Stage::start_no_stop);
        const uint16_t resets0 = Host::unwedges();
        const uint32_t c = start_cycles(command_probe(link_speed));
        stand_lo = c < stand_lo ? c : stand_lo;
        stand_hi = c > stand_hi ? c : stand_hi;
        stand_kept = stand_kept && w.busy_end && Host::unwedges() == resets0 + 1u;
        settle_ms(1);
    }
    print(serial, "  bus_free_us ", bus_free_us_named, " over an idle wire: start() ", stand_lo, " to ", stand_hi,
          " cycles, the reset inside", crlf);
    /// The reset's stores: SWRST up and down, the timing, PE, ACK and CTLR2, some
    /// ten read-modify-writes at 12 to 20 cycles each.
    constexpr uint32_t reset_cycles = 200;
    bench.verdict("over an idle wire the same 100 us, the reset's stores on top, one reset counted each time",
                  stand_lo >= bus_free_us_named * bound_per_us &&
                      stand_hi <= bus_free_us_named * bound_per_us + over + reset_cycles && stand_kept);
    // ---- stop_drain_us, run out by a client holding the clock ----
    constexpr uint32_t stop_drain_named = stop_drain_named_us;
    // Each floor in this letter is the name on a figure that holds start()'s own
    // cycles and the ruler, so it would pass a wait that much short; poll_for_us()'s
    // sum is what keeps the wait itself from ending early.
    for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
        uint32_t lo = 0xFFFFFFFFu;
        uint32_t hi = 0;
        bool answered = true;
        constexpr uint8_t stands = 3;
        for (uint8_t k = 0; k < stands; ++k) {
            answered = answered && stand_stop(speed);
            const uint32_t c = start_cycles(command_probe(link_speed));
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
                      answered && lo >= stop_drain_named * bound_per_us && hi <= stop_drain_named * bound_per_us + over);
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
    // ---- both with the STK stopped: the poll budget, on TIM2 at 1 us a count ----
    using Ruler = Tim<2>;
    Ruler::init();
    (void)Ruler::configure({.prescaler = static_cast<uint16_t>(bound_per_us - 1u), .period = 0xFFFF});
    Ruler::enable(true);
    const auto stopped_us = [] {
        console_drain();
        stk()->CTLR = stk()->CTLR & ~stk_ste;
        const uint16_t t0 = Ruler::count();
        host_done = false;
        host_isr_entries = 0;
        (void)Host::start(command_probe(link_speed));
        const uint16_t t1 = Ruler::count();
        stk()->CTLR = stk()->CTLR | stk_ste;
        return static_cast<uint32_t>(static_cast<uint16_t>(t1 - t0));
    };
    host_ready();
    line_low<SclPin>();
    const uint32_t free_budget_us = stopped_us();
    line_up<SclPin>();
    (void)Host::recover();
    const Staged idle_wire = stage_busy(Stage::start_no_stop);
    const uint32_t watch_budget_us = stopped_us();
    settle_ms(1);
    (void)Host::recover();
    const bool stood = stand_stop(link_speed);
    const uint32_t drain_budget_us = stopped_us();
    settle_ms(60);
    (void)Host::recover();
    settle_ms(200);
    Ruler::enable(false);
    const uint32_t free_polls = budget_of(bus_free_us_named);
    const uint32_t drain_polls = budget_of(stop_drain_named);
    print(serial, "  the STK stopped: bus_free_us ends after ", free_polls, " polls in ", free_budget_us, " us, ",
          free_budget_us * bound_per_us / free_polls, " cycles a poll with SCL low, ", watch_budget_us, " us and ",
          watch_budget_us * bound_per_us / free_polls, " over an idle wire (BUSY up ", idle_wire.busy_end ? 1 : 0, ", the reset inside); stop_drain_us after ", drain_polls, " polls in ",
          drain_budget_us, " us, ", drain_budget_us * bound_per_us / drain_polls, " cycles a poll", crlf);
    bench.verdict("with the STK stopped each wait still ends, on its count of polls: never under the time named, never "
                  "over the budget at the longest poll",
                  stood && free_budget_us >= bus_free_us_named && free_budget_us <= free_polls * poll_max / bound_per_us + 1u &&
                      drain_budget_us >= stop_drain_named && drain_budget_us <= drain_polls * poll_max / bound_per_us + 1u);
    print(serial, "  a raw START into the peer's probe of nobody at 100 kHz, 20 to 51 us behind its START", crlf);
    live_raw_row(4, 20, 1);
    print(serial, "  at 50 kHz, 30 to 123 us behind", crlf);
    live_raw_row(3, 30, 3);
    print(serial, "  at 20 kHz, 30 to 247 us behind", crlf);
    live_raw_row(2, 30, 7);
    host_ready();
    bench.verdict("and the command channel is back", command(Op::ping));
}

// ===========================================================================
// p - the dead channel and the release pulse
// ===========================================================================
//
// A DMA channel's requests are an OR (RM 8.2), and on this die a raised
// request is HELD until the channel takes it or its peripheral's reset line
// is pulsed - measured on the CH32X035 and the CH32V203/V303
// (docs/ch32x035/dma.md, docs/ch32vx03/dma.md, "A released requester"), and
// asked of this die here: the next owner takes one stray item and the
// channel then serves nothing more - a DEAD CHANNEL. The timers' compare
// requests reach channels 6 and 7 (TIM1's channel 3 and TIM2's channel 2,
// table 8-2), so a request
// raised there with no channel enabled to take it, its gate closed without
// a reset, stages a dead channel under the I2C host's engines with no wire
// of its own.

/// What a serve of the peer's is armed with (the twi_link `serve` action).
twilink::Params serve_params(uint16_t count, uint16_t ms, uint8_t seed) {
    twilink::Params a{};
    a.count = count;
    a.ms = ms;
    a.addr = twilink::dut_addr;
    a.seed = seed;
    a.pattern = twilink::pattern_counting;
    return a;
}

// I2C1's two requests: the owners of channels 6 and 7 in this image (one
// owner a channel, util/dma_owner.hpp) - the engined host claims them, and the
// next owner below programs either channel by hand in their name. The timers
// program no channel: they raise a request on its OR.
using I2cTxOwner = DmaOwner<DmaPeriph::i2c, 1, DmaSignal::tx>;
using I2cRxOwner = DmaOwner<DmaPeriph::i2c, 1, DmaSignal::rx>;
using DeadTx = Tim<1>;   ///< its channel 3's compare request: DMA channel 6
using DeadRx = Tim<2>;   ///< its channel 2's compare request: DMA channel 7
constexpr uint8_t dead_tx_ch = 2;
constexpr uint8_t dead_rx_ch = 1;
// Table 8-2: TIM1_CH3 rides channel 6 with I2C1_TX (and USART2_TX), TIM2_CH2
// channel 7 with I2C1_RX (and USART2_RX); the timers' header carries no
// table for its compare requests, so the two numbers are the table's.

// The staging itself - a compare request held, freed, and the next owner
// of a channel counted - is the suites' shared held_request.hpp.

/// A write of 32 on the engines over a channel 6 that TIM1 holds dead: the
/// staging of both the transmit fault and the block's own held request
/// (TxE raised under DMAEN on a channel that never answered it). The engine
/// is left as the tenure left it, TIM1 reset.
uint8_t dead_write() {
    held::hold_compare_request<DeadTx>(dead_tx_ch, clock);
    (void)peer_act(Op::serve, serve_params(0, 300, 0x10));
    dma_host_live = true;
    (void)DmaHost::init(clock);
    for (uint8_t i = 0; i < 32; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0x20u + i);
    }
    const uint8_t st = dma_tenure(twilink::dut_addr, tx_buf, 32, nullptr, 0, link_speed);
    held::free_compare_request<DeadTx>();
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
    held::hold_compare_request<DeadRx>(dead_rx_ch, clock);
    (void)peer_act(Op::serve, serve_params(0, 300, 0x40));
    dma_host_live = true;
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
    host_isr_entries = 0;
    host_stormed = false;
    if (!DmaHost::start(r)) {
        settle_ms(20);
    }
}

void tp_dead_channel() {
    if (!need_peer()) {
        return;
    }
    // THE CONTROL: nothing raised on either channel.
    const uint16_t control6 = held::next_owner_moves<6, I2cTxOwner>(clock);
    const uint16_t control7 = held::next_owner_moves<7, I2cRxOwner>(clock);

    // 1. THE DEAD TRANSMIT CHANNEL. The hold proved first by a next owner
    // (which takes the one stray item and leaves the channel dead), then a
    // write of 32 over it: the channel moves the first byte at its enable
    // and nothing more, BTF comes with the count standing, and the engine
    // must answer i2c_dma_fault - never i2c_ok over a short write.
    held::hold_compare_request<DeadTx>(dead_tx_ch, clock);
    const uint16_t held6 = held::next_owner_moves<6, I2cTxOwner>(clock);
    held::free_compare_request<DeadTx>();
    const uint8_t dws = dead_write();
    const uint32_t dw_entries = host_isr_entries;
    DmaHost::release();
    host_ready();
    const uint16_t dead_took = peer_took();
    // The same write with the channel live again.
    (void)peer_act(Op::serve, serve_params(0, 300, 0x10));
    dma_host_live = true;
    (void)DmaHost::init(clock);
    const uint8_t lws = dma_tenure(twilink::dut_addr, tx_buf, 32, nullptr, 0, link_speed);
    DmaHost::release();
    host_ready();
    const uint16_t live_took = peer_took();
    print(serial, "  next owner, nothing raised: ch6 ", control6, " ch7 ", control7, "; after TIM1's held request: ",
          held6, " (of 4)", crlf);
    print(serial, "  the write of 32 over the dead channel 6: status ", dws, " (i2c_dma_fault ", i2c_dma_fault,
          "), the peer took ", dead_took, ", event entries ", dw_entries, "; with the channel live: status ", lws,
          ", the peer took ", live_took, crlf);
    bench.verdict("with nothing raised, the next owner of channel 6 or 7 moves nothing", control6 == 0u &&
                                                                                        control7 == 0u);
    bench.verdict("TIM1's compare request, raised and its gate closed with no reset, is HELD: the next owner "
                  "of channel 6 takes one stray item",
                  held6 == 1u);
    bench.verdict("A WRITE OVER THE DEAD CHANNEL ends i2c_dma_fault (BTF with the transmit count standing), "
                  "never i2c_ok over a short write - and the far end took fewer than the 32",
                  dws == i2c_dma_fault && dead_took < 32u);
    bench.verdict("with TIM1 reset the same write lands whole, i2c_ok", lws == i2c_ok && live_took == 32u);

    // 2. THE DEAD RECEIVE CHANNEL: TIM2's request held on channel 7 under
    // an engined read of 16. The channel takes one stray item at its enable
    // and serves no RxNE after it, so the read never completes: the event
    // line is down for the block (the driver), so its vector is NOT stormed
    // by the standing RxNE and BTF - the entries stay at the opening's (SB
    // and ADDR: two, three with a re-entry; a storm would be 60 000) - and
    // the caller's bound answers it (here dma_tenure's spin, some 50 ms,
    // against the 1.5 ms the read takes at 100 kHz, then recover()).
    held::hold_compare_request<DeadRx>(dead_rx_ch, clock);
    (void)peer_act(Op::serve, serve_params(0, 300, 0x40));
    dma_host_live = true;
    (void)DmaHost::init(clock);
    for (uint8_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0xEE;
    }
    dma7_entries = 0;
    const uint32_t err0 = error_isr_entries;
    const uint8_t drs = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 16, link_speed);
    const uint32_t dr_entries = host_isr_entries;
    const uint32_t dr_err = error_isr_entries - err0;
    const bool dr_storm = host_stormed;
    const uint32_t dr_dma = dma7_entries;
    DmaHost::release();
    held::free_compare_request<DeadRx>();
    host_ready();
    settle_ms(350);
    (void)peer_act(Op::serve, serve_params(0, 300, 0x40));
    dma_host_live = true;
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
        held::free_compare_request<DeadRx>();
        moved[way] = held::next_owner_moves<7, I2cRxOwner>(clock);
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
// k - the stuck bus
// ===========================================================================

void tk_unstick() {
    if (!need_peer()) {
        return;
    }
    const uint8_t clean = Host::unstick();
    print(serial, "  a healthy bus: unstick() returns ", clean, crlf);
    bench.verdict("unstick() on a clean bus clocks nothing and says so", clean == 0u);

    // The peer holds SDA low from its port and lets go after four SCL
    // falling edges: unstick() clocks until the client releases.
    twilink::Params h{};
    h.ms = 400;
    h.aux8 = 4;
    h.aux16 = 0;
    if (!peer_act(Op::hold_sda, h)) {
        bench.verdict("the peer accepted the hold_sda command", false);
        return;
    }
    settle_ms(5);
    const bool held = !SdaPin::read();
    // The STOP unstick() makes by hand is seen by the instance's own
    // slave half, which raises STOPF with nothing in flight: unstick()
    // clears it, and the event line - down between tenures - must not
    // re-enter for it at all. Counted from before the call.
    host_isr_entries = 0;
    const uint32_t storms_before = storms;
    const uint8_t pulses = Host::unstick();
    print(serial, "  SDA held by the peer: ", held ? "yes" : "no", "; unstick() clocked ", pulses,
          " pulse(s) before it let go", crlf);
    bench.verdict("the peer really held SDA low", held);
    bench.verdict("unstick() clocked until the client released - a handful of pulses, never 0xFF",
                  pulses >= 1u && pulses <= 9u);
    settle_ms(450);
    const uint32_t idle_entries = host_isr_entries;
    print(serial, "  the event vector, idle, after the hand-made STOP: ", idle_entries, " entries, STAR1=",
          hex(H::status1()), crlf);
    bench.verdict("the STOPF the hand-made STOP leaves is cleared - no storm on the event "
                  "vector",
                  idle_entries <= 2u && storms == storms_before && (H::status1() & i2c_stopf) == 0u);
    const uint8_t after = Host::unstick();
    bench.verdict("with the wire free again unstick() clocks nothing", after == 0u);
    bench.verdict("and the command channel is back", command(Op::ping));
}

void banner() {
    print(serial, crlf, "test_ch32_i2c - ", device::part_name, " I2C (RM ch. 15) against a peer board", crlf);
    print(serial, "  P", bus_pins.scl.port, bus_pins.scl.pin, " SCL / P", bus_pins.sda.port, bus_pins.sda.pin,
          " SDA (I2C1 column ", bus_pins.remap, ") -> the peer running `twi_peer` (command address ",
          hex(twilink::command_addr), "), its pull-ups, a shared GND, both at 3.3 V", crlf);
    print(serial, "  pull-ups seen on the wire now: ", wire_pulled_up() ? "yes" : "NO", crlf);
    bench.menu();
}

} // namespace

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_VECTOR(usart1_handler) { (void)Serial::isr(); }

/// The completion to the arbiter that owns the host (letters t and u have two).
[[gnu::always_inline]] inline void post_done() {
    if constexpr (untimed_letters) {
        if (bus_ao_untimed) {
            brio::post<kl::I2cArbUntimed>(brio::TransferDone{Host::status()});
            return;
        }
    }
    brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
}

BRIO_CH32_VECTOR(i2c1_ev_handler) {
    // The client's glue only where letter v is carried (a group image of
    // the CH32V003 carries none of it).
    if (brio::selection_carries(brio::test_letters, 'v') && client_live) {
        const uint32_t t0 = brio::stk()->CNT;
        ct.entries = ct.entries + 1u;
        if (ct.entries > client_entry_budget) {
            H::event_interrupt(false);
            H::buffer_interrupt(false);
            return;
        }
        client_event();
        const uint32_t t1 = brio::stk()->CNT;
        const uint32_t d = t1 >= t0 ? t1 - t0 : t1 + brio::stk()->CMP + 1u - t0;
        ct.cycles = ct.cycles + d;
        if (d > ct.cycles_max) {
            ct.cycles_max = d;
        }
        return;
    }
    if (live_client_letters && live_client) {
        // Letter j's client half, for the peer's long tenure: ADDR cleared, a byte
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
    if (dma_letters && dma_host_live) {
        if (DmaHost::isr()) {
            host_done = true;
        }
        return;
    }
    if (arbiter_letters && bus_ao_live) {
        if (Host::isr()) {
            post_done();
        }
        return;
    }
    if (Host::isr()) {
        host_done = true;
    }
}

BRIO_CH32_VECTOR(i2c1_er_handler) {
    if (brio::selection_carries(brio::test_letters, 'v') && client_live) {
        ct.err_entries = ct.err_entries + 1u;
        client_error();
        return;
    }
    error_isr_entries = error_isr_entries + 1u;
    if (dma_letters && dma_host_live) {
        if (DmaHost::error_isr()) {
            host_done = true;
        }
        return;
    }
    if (arbiter_letters && bus_ao_live) {
        if (Host::error_isr()) {
            post_done();
        }
        return;
    }
    if (Host::error_isr()) {
        host_done = true;
    }
}

/// Both channels' vectors call this one body, out of line: the host's
/// dma_isr() inlined twice costs a CH32V003 group image some two hundred
/// bytes of its 15 KB, and nothing here times the vector.
[[gnu::noinline]] void dma_vector_body() {
    if (dma_letters && dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
    }
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

    bench.letter('a', "the block, wireless: reset values, the arithmetic, the refusals, "
                      "the enable protection measured", ta_block);
    bench.letter('b', "THE SCAN: 112 addresses probed, nobody-home as i2c_nack_addr", tb_scan);
    bench.letter('c', "THE PEER: the twi_link command channel, ident, ten round trips",
                 tc_peer_link);
    bench.letter('d', "the tenure shapes against the peer, the four receive procedures, "
                      "the reserved addresses refused", td_shapes);
    bench.letter('e', "the vocabulary on the wire, and commanded stretching priced",
                 te_vocabulary);
    bench.letter('f', "the two speeds against a second chip, timed", tf_speeds);
    bench.letter('g', "THE DMA ENGINES on channels 6 and 7 against the peer", tg_dma);
    bench.letter('n', "THE ENGINE A REQUEST NAMES: dma at one byte, interrupt touching no channel, dma "
                      "refused without engines, the edges under dma",
                 tn_engine);
    bench.letter('h', "THE KERNEL: I2cBus over I2cHost, the rejection, the votes, a held SDA "
                      "and a held clock answered in their place", th_kernel);
    bench.letter('i', "THE REFUSAL, wireless: a speed the clock cannot make, i2c_rejected "
                      "inside start() through the arbiter", ti_refusal);
    bench.letter('j', "TWO HOSTS ON ONE WIRE: arbitration both ways, a START into a live tenure, this "
                      "board as the peer's client - written to and read from", tj_two_hosts);
    bench.letter('k', "the stuck bus: the peer holding SDA, unstick() counting", tk_unstick);
    bench.letter('p', "THE DEAD CHANNEL: a held request under the engines' write and read, and release() over "
                      "pulse", tp_dead_channel);
    bench.letter('v', "THIS BOARD AS THE CLIENT FROM ITS VECTORS: written to and read from by the peer's "
                      "controller", tv_client_vectors);
    bench.letter('y', "THE ENGINES' CHANNELS STARVED: a memory copy above them, the entries counted", ty_starved);
    bench.letter('q', "the controller's own habits, raw: BTF under a requested STOP, the repeated START "
                      "past a refused byte, the withdrawal's race", tq_controller);
    bench.letter('r', "THE REPEATED START on the last byte's TxE: 1 to 4 written then read, "
                      "acknowledged and the last refused, both speeds, pump and engines",
                 tr_restart);
    bench.letter('l', "THE FIRST BYTES at the entry that clears ADDR: the flags raw, the host's writes by their entries, "
                      "a byte committed behind a refused one", tl_first_bytes);
    bench.letter('u', "A BUSY LEFT STANDING over an idle wire: how it is made, what a request meets, a client holding the "
                      "clock past the limit", tu_unwedge);
    bench.letter('t', "THE TWO BOUNDS OF start(): bus_free_us and stop_drain_us against their names, and with the STK stopped",
                 tt_bounds);
    bench.letter('w', "the wire probe: each line held low for 500 ms, for the other end to read (no verdict)",
                 tw_wire_probe, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
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
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
