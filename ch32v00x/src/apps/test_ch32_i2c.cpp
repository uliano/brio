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
// `twi_peer` (an STM32G0 Nucleo, a SAM C21 or an AVR - the peer names
// itself in its ident), commanded IN BAND over the bus under test
// through avrdx/src/apps/twi_link.hpp, included by relative path: one
// source of truth for the wire format, whatever the peer's
// architecture. The peer's command-mode client answers ONE address
// (0x6B) with no general call and no second address.
//
//   SCL   PC2  <->  the peer's SCL (a Nucleo-G0: PB8)
//   SDA   PC1  <->  the peer's SDA (a Nucleo-G0: PB9)
//   GND   <->  GND, both boards at 3.3 V
//
// THE PULL-UPS ARE THE PEER'S: this module carries none, an
// alternate-function open-drain pad has no internal pull on this
// family, and the Nucleo's two-wire node has its 2.2 kOhm pair. With
// no peer on the desk both lines read low and every wire letter
// declines with the reason.
//
// THE HOST RUNS ON ITS TWO VECTORS (events on i2c1_ev, errors on
// i2c1_er); a letter that wants a tenure waited out spins on a flag
// the handler sets. Two I2cHost instantiations share the one instance
// - the plain one and the one on DMA channels 6 and 7 - and a flag
// says which owns the vectors, since two tasks over one peripheral
// share its registers and not their statics.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the reset values, the timing arithmetic
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
//      general call
//   e  the vocabulary against the peer (nack_addr from a deaf client,
//      nack_data at a commanded byte) and commanded stretching priced
//   f  the two speeds against a second chip, byte-exact both ways
//   g  THE DMA ENGINES: the same shapes on channels 6 and 7
//   h  THE KERNEL against the peer: I2cBus (= BusMaster) over I2cHost,
//      the NACK in its place, the rejection, both votes, and the
//      wedge the PEER holds answered by the per-bus timeout
//   i  THE REFUSAL, wireless: a speed the clock cannot make is answered
//      i2c_rejected inside start() and delivered through the arbiter,
//      the wire and the vector untouched
//   k  THE STUCK BUS: the peer holding SDA, and unstick() counting
//      the clocks until it lets go
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/i2c.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED (see the header comment).
#include "../../../avrdx/src/apps/twi_link.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'C', 0>;

using H = I2c<1>;
using Host = I2cHost<1>;
using DmaHost = I2cHost<1, i2c1_default_pins, DmaTxEngine<6>, DmaRxEngine<7>>;
using SclPin = Pin<'C', 2>;
using SdaPin = Pin<'C', 1>;

TestBench<Serial> bench;

/// An address nobody on this bus answers.
constexpr uint8_t nobody_addr = 0x23;

volatile bool host_done = false;
volatile bool dma_host_live = false;
volatile bool bus_ao_live = false;
volatile uint32_t host_isr_entries = 0;
volatile uint32_t error_isr_entries = 0;
/// The host's storm budget: a vector that fires this many times inside
/// one tenure is silenced and the fact reported.
constexpr uint32_t host_isr_budget = 60'000;
volatile bool host_stormed = false;

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
    for (uint32_t i = 0; i < 600'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL: STAR1=", hex(H::status1()), " STAR2=", hex(H::status2()),
              " CTLR1=", hex(H::regs().CTLR1), " ev entries ", host_isr_entries, " er ",
              error_isr_entries, host_stormed ? " STORMED" : "", crlf);
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
bool wire_pulled_up() {
    return SclPin::read() && SdaPin::read();
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
              " fw=", hex(id.version), crlf);
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
    print(serial, "  write=", ws, " read=", rs, " mism=", rmism, " combined=", cs,
          "; peer: count=", r.count, " addr_hits=", r.addr_hits, " first=", hex(r.first),
          " sum=", hex(r.sum), crlf);
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
    t0 = Ticker::millis();
    const uint8_t s2 = host_tenure(twilink::dut_addr, w8, 8, nullptr, 0, link_speed);
    const uint32_t stretched_ms = Ticker::millis() - t0;
    settle_ms(300);
    twilink::Report sr{};
    const bool srep = peer_report(sr);
    print(serial, "  8 x 8 bytes unstretched: ", base_ms, " ms; ONE 8-byte tenure at 2 ms per byte: ",
          stretched_ms, " ms; peer count=", sr.count, crlf);
    bench.verdict("the unstretched baseline tenures all complete i2c_ok", base_ok);
    bench.verdict("a client stretching every data byte by 2 ms stretches the WALL TIME (16 ms "
                  "or more for 8 bytes) and the tenure still completes i2c_ok - stretching is "
                  "flow control",
                  s2 == i2c_ok && stretched_ms >= 16u && srep && sr.count >= 8u);
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
        // The tenure timed on the STK: 9 x (N + 1) SCL periods plus a
        // START and a STOP, so the period is what the chooser predicts
        // when the client does not stretch.
        console_drain();
        const uint32_t t0 = Ticker::ticks();
        const uint32_t c0 = stk()->CNT;
        const uint8_t ws = host_tenure(twilink::dut_addr, tx_buf, 8, nullptr, 0, rungs[i].speed);
        const uint32_t c1 = stk()->CNT;
        const uint32_t t1 = Ticker::ticks();
        const uint32_t period = stk()->CMP + 1u;
        const uint32_t cycles = (t1 - t0) * period + c1 - c0;
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
        const uint32_t scl_hz_measured = cycles != 0u ? (81UL * SysClock::hz) / cycles : 0u;
        print(serial, "  ", rungs[i].name, ": SCL asked ", Host::scl_hz(rungs[i].speed) / 1000u,
              " kHz, an 8-byte write took ", cycles, " cycles -> about ", scl_hz_measured / 1000u,
              " kHz on the wire; write=", ws, " read=", rs, " mism=", mism, " -> ",
              ok ? "byte-exact both ways" : "NOT exact", crlf);
        if (ok) {
            ++exact;
        }
    }
    bench.verdict("Standard mode and Fast mode both carry a write and a read byte-exact between "
                  "TWO SEPARATE CHIPS",
                  exact == 2u);
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
    const uint8_t ws = dma_tenure(twilink::dut_addr, tx_buf, 16, nullptr, 0, link_speed);
    const uint8_t rs = dma_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 16, link_speed);
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
    DmaHost::release();
    dma_host_live = false;
    settle_ms(450);
    twilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  DMA: write16=", ws, " read16=", rs, " mism=", mism, " combined=", cs,
          " read2=", r2, " read1(pump)=", r1, "; peer count=", r.count, " addr_hits=", r.addr_hits,
          " faults tx=", DmaTxEngine<6>::faults(), " rx=", DmaRxEngine<7>::faults(), crlf);
    bench.verdict("a 16-byte write and a 16-byte read through the DMA engines complete i2c_ok, "
                  "byte-exact",
                  ws == i2c_ok && rs == i2c_ok && mism == 0u);
    bench.verdict("the combined tenure, the two-byte read (LAST's NACK) and the one-byte read "
                  "(the pump's) all complete",
                  cs == i2c_ok && r2 == i2c_ok && r1 == i2c_ok);
    bench.verdict("the peer accounts every byte (16 + 16 + 4 + 4 + 2 + 1 = 43), no transfer fault",
                  rep && r.count == 43u && DmaTxEngine<6>::faults() == 0u &&
                      DmaRxEngine<7>::faults() == 0u);
    host_ready();
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

using BusKernel = Kernel<P, Probe, I2cArb>;

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
    bench.verdict("THE SAME BUS AO carries the next tenure to i2c_ok after recover()",
                  re && kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_ok);
    bus_ao_live = false;
    settle_ms(300);
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
    const uint8_t pulses = Host::unstick();
    print(serial, "  SDA held by the peer: ", held ? "yes" : "no", "; unstick() clocked ", pulses,
          " pulse(s) before it let go", crlf);
    bench.verdict("the peer really held SDA low", held);
    bench.verdict("unstick() clocked until the client released - a handful of pulses, never 0xFF",
                  pulses >= 1u && pulses <= 9u);
    settle_ms(450);
    const uint8_t after = Host::unstick();
    bench.verdict("with the wire free again unstick() clocks nothing", after == 0u);
    bench.verdict("and the command channel is back", command(Op::ping));
}

void banner() {
    print(serial, crlf, "test_ch32_i2c - CH32V006K8 I2C (RM ch. 15) against a peer board", crlf);
    print(serial, "  PC2 SCL / PC1 SDA -> the peer running `twi_peer` (command address ",
          hex(twilink::command_addr), "), its pull-ups, a shared GND, both at 3.3 V", crlf);
    print(serial, "  pull-ups seen on the wire now: ", wire_pulled_up() ? "yes" : "NO", crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

extern "C" BRIO_CH32_INTERRUPT void i2c1_ev_handler() {
    host_isr_entries = host_isr_entries + 1u;
    if (host_isr_entries > host_isr_budget) {
        host_stormed = true;
        H::event_interrupt(false);
        H::buffer_interrupt(false);
        return;
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

    bench.letter('a', "the block, wireless: reset values, the arithmetic, the refusals, "
                      "the enable protection measured", ta_block);
    bench.letter('b', "THE SCAN: 112 addresses probed, nobody-home as i2c_nack_addr", tb_scan);
    bench.letter('c', "THE PEER: the twi_link command channel, ident, ten round trips",
                 tc_peer_link);
    bench.letter('d', "the tenure shapes against the peer, the four receive procedures, "
                      "the general call", td_shapes);
    bench.letter('e', "the vocabulary on the wire, and commanded stretching priced",
                 te_vocabulary);
    bench.letter('f', "the two speeds against a second chip, timed", tf_speeds);
    bench.letter('g', "THE DMA ENGINES on channels 6 and 7 against the peer", tg_dma);
    bench.letter('h', "THE KERNEL: I2cBus over I2cHost, the rejection, the votes, the "
                      "wedge answered by the timeout", th_kernel);
    bench.letter('i', "THE REFUSAL, wireless: a speed the clock cannot make, i2c_rejected "
                      "inside start() through the arbiter", ti_refusal);
    bench.letter('k', "the stuck bus: the peer holding SDA, unstick() counting", tk_unstick);

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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
