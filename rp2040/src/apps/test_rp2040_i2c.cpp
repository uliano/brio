// test_rp2040_i2c - the reference bench suite for the RP2040's I2C
// (rp2040/i2c.hpp over datasheet 4.3, the Synopsys DW_apb_i2c): the
// host on a wire to the chip's OTHER instance as the client, every
// tenure shape, the vocabulary produced on the wire, the three speeds
// measured, the DMA engines, the roles inverted, and util/i2c_bus.hpp's
// arbiter with not one line changed.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// ONE INSTRUMENT: THE SELF-LINK. I2C0 hosts and I2C1 listens on ONE
// BOARD, two wires between the two instances' pads and the board's
// pull-ups on them:
//
//   GP12 (I2C0 SDA)  <->  GP14 (I2C1 SDA)     4.7 kOhm to 3V3
//   GP13 (I2C0 SCL)  <->  GP15 (I2C1 SCL)     4.7 kOhm to 3V3
//
// The client answers 0x42 and is served from ITS OWN interrupt: what
// the host writes is taken there, what the host reads is given there
// (up to the FIFO's depth ahead), and every tenure edge the block
// reports is counted. Both instances' interrupts run on one core, so
// the client's service delays the host's and the other way round; the
// wire lets both. With the pull-ups absent both lines read low and
// every wire letter declines with the reason.
//
// THE RULER IS THE SYSTEM TIMER (rp2040/timer.hpp). A bus rate is
// measured with no pad to spare: a tenure of N bytes is 9 x (N + 1)
// SCL periods, and the stopwatch runs around the whole tenure.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: table 279 as the driver states it, the
//      timing arithmetic (the three speeds at 125 MHz, table 450's
//      floors reproduced), the reset state, the FIFO depths the block
//      reports, the refusals, the enable and the disable through
//      IC_ENABLE_STATUS
//   b  THE WIRE: the pull-ups asked about first; then THE SCAN, every
//      address 0x08..0x77 probed, exactly one answer at 0x42, nobody-
//      home measured as i2c_nack_addr and timed
//   c  the tenure shapes: write, read, write-then-read with the
//      repeated START counted from the far end, the probe as this
//      silicon serves it (one byte read), a 200-byte write in one
//      tenure through the pump
//   d  the vocabulary on the wire: nack_addr from a deaf client on a
//      write and on the probe; nack_data from a client refusing data;
//      the client's clock stretch priced; the flush of stale bytes at
//      a read
//   e  THE THREE SPEEDS byte-exact both ways and MEASURED on the ruler
//   f  THE DMA ENGINES: a 64-byte read, a write-then-read, and the
//      short reads that stay on the pump
//   g  THE KERNEL: I2cBus (= BusMaster) over I2cHost, the NACK in its
//      place, the rejection, both votes, and THE TIMED BUS: a client
//      holding the clock answered by the arbiter's per-bus timeout,
//      the recover()ed engine carrying the next tenure
//   h  THE STUCK BUS: unstick() reading a healthy wire, then SDA held
//      low by a GPIO, then healthy again
//   i  THE ROLES INVERT on the same two wires: I2C1 hosts, I2C0 listens
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/i2c.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;
using P = Rp2040Platform<>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Led = Pin<25>;

TestBench<Serial, 16> bench;

// I2C0 hosts on GP13/GP12; I2C1 listens on GP15/GP14. And the other
// way round.
constexpr I2cPins host_pins{.scl = 13, .sda = 12};
constexpr I2cPins client_pins{.scl = 15, .sda = 14};
using Host = I2cHost<0, host_pins>;
using DmaHost = I2cHost<0, host_pins, DmaTxEngine<4, uint16_t>, DmaRxEngine<5>>;
using Client = I2cClient<1, client_pins>;
using HostB = I2cHost<1, client_pins>;
using ClientA = I2cClient<0, host_pins>;
constexpr uint8_t client_address = 0x42;
constexpr uint8_t nobody = 0x43;

// Which task owns each instance's vector at the moment.
enum class I2c0Owner : uint8_t { none, host, dma_host, client_a };
enum class I2c1Owner : uint8_t { none, client, host_b };
volatile I2c0Owner i2c0_owner = I2c0Owner::none;
volatile I2c1Owner i2c1_owner = I2c1Owner::none;
volatile uint32_t i2c0_isr_entries = 0;
volatile uint32_t i2c1_isr_entries = 0;
volatile bool transfer_done = false;
volatile uint8_t transfer_status = 0;
bool bus_ao_live = false;

uint32_t us_now() { return Timer::now_low(); }
void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}

uint8_t tx_buf[256];
uint8_t rx_buf[256];

void fill_pattern(uint8_t* p, uint16_t n, uint8_t seed) {
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + 3u * i);
    }
}
bool same(const uint8_t* a, const uint8_t* b, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

// ---- the client, served from its interrupt ----------------------------------
uint8_t client_answers[256];
uint8_t client_heard[256];
volatile uint16_t client_given = 0;    // answers given so far
volatile uint16_t client_count = 0;    // bytes heard
// Everything the ISR reads and the letters write is VOLATILE: a store
// to a plain global with no opaque call after it is sunk past an
// inlined tenure, and the ISR reads the old value (measured: every
// answer 0xFF, a stretch that never happened).
volatile uint16_t client_limit = 0;    // answers to give in all (0xFF filler after)
volatile uint16_t client_stops = 0;
volatile uint16_t client_restarts = 0;
volatile uint16_t client_read_reqs = 0;
volatile uint16_t client_nacks = 0;
volatile uint16_t client_flushes = 0;
volatile uint16_t client_errors = 0;
volatile uint16_t client_overruns = 0;
volatile uint16_t client_gcalls = 0;
volatile uint32_t client_stretch_us = 0;   // spent before every give
volatile bool client_wedged = false;       // never answers a read request: SCL held
volatile uint8_t client_ahead = 1;         // answers written per read request
volatile uint32_t client_stamp[16];    // us at each read request, for the diagnostics
volatile uint8_t client_stamp_n = 0;

template <typename C>
void client_serve(I2cClientEvent ev) {
    switch (ev) {
        case I2cClientEvent::byte_received:
            while (C::data_ready()) {
                const uint8_t b = C::take();
                if (client_count < 256u) {
                    client_heard[client_count] = b;
                }
                client_count = client_count + 1u;
            }
            break;
        case I2cClientEvent::byte_wanted:
            client_read_reqs = client_read_reqs + 1u;
            if (client_stamp_n < 16u) {
                client_stamp[client_stamp_n] = us_now();
                client_stamp_n = client_stamp_n + 1u;
            }
            if (client_wedged) {
                C::interrupts(I2cInterrupt::rd_req, false);   // the request stands, SCL with it
                break;
            }
            if (client_stretch_us != 0u) {
                spin_us(client_stretch_us);
            }
            for (uint8_t k = 0; k < client_ahead && C::writable(); ++k) {
                const uint8_t b = client_given < client_limit ? client_answers[client_given] : 0xFFu;
                C::give(b);
                client_given = client_given + 1u;
            }
            C::clear_read_request();
            break;
        case I2cClientEvent::stop: client_stops = client_stops + 1u; break;
        case I2cClientEvent::restart: client_restarts = client_restarts + 1u; break;
        case I2cClientEvent::nacked: client_nacks = client_nacks + 1u; break;
        case I2cClientEvent::flushed: client_flushes = client_flushes + 1u; break;
        case I2cClientEvent::overrun: client_overruns = client_overruns + 1u; break;
        case I2cClientEvent::general_call: client_gcalls = client_gcalls + 1u; break;
        case I2cClientEvent::error: client_errors = client_errors + 1u; break;
        default: break;
    }
}

void client_reset_tally() {
    client_given = 0;
    client_count = 0;
    client_stops = 0;
    client_restarts = 0;
    client_read_reqs = 0;
    client_nacks = 0;
    client_flushes = 0;
    client_errors = 0;
    client_overruns = 0;
    client_gcalls = 0;
    client_stretch_us = 0;
    client_wedged = false;
    client_ahead = 1;
    client_stamp_n = 0;
    for (uint16_t i = 0; i < 256u; ++i) {
        client_heard[i] = 0xEE;
    }
}

/// Bring the client up on `C` at 0x42 with `limit` answers of pattern
/// `seed`, its interrupts on.
template <typename C>
bool client_ready(uint16_t limit, uint8_t seed, I2cSpeed speed = I2cSpeed::fast_plus_1m) {
    client_reset_tally();
    client_limit = limit;
    for (uint16_t i = 0; i < 256u; ++i) {
        client_answers[i] = static_cast<uint8_t>(seed + 5u * i);
    }
    if (!C::init(clock, {.address = client_address, .speed = speed})) {
        return false;
    }
    C::interrupts(C::events, true);
    return true;
}

bool client_up() {
    i2c1_owner = I2c1Owner::none;
    const bool ok = client_ready<Client>(0, 0);
    i2c1_owner = I2c1Owner::client;
    return ok;
}
void client_down() {
    i2c1_owner = I2c1Owner::none;
    Client::release();
}

// ---- the host's tenures -----------------------------------------------------------
void host_ready() {
    i2c0_owner = I2c0Owner::none;
    bus_ao_live = false;
    (void)Host::init(clock);
    i2c0_owner = I2c0Owner::host;
}

// What a tenure that never answered left behind (diagnostics).
uint32_t dma_tx_remaining = 0;
uint32_t dma_tx_credits = 0;
uint32_t dma_rx_remaining = 0;
uint32_t dma_rx_credits = 0;
uint32_t i2c0_raw = 0;
uint32_t i2c0_status = 0;
uint8_t i2c0_rxflr = 0;
uint8_t i2c0_txflr = 0;
uint32_t i2c0_abort = 0;
template <typename H>
void snapshot() {
    using S = typename H::Resource;
    i2c0_raw = S::raw_pending();
    i2c0_status = S::flags();
    i2c0_rxflr = S::rx_count();
    i2c0_txflr = S::tx_count();
    i2c0_abort = S::abort_source();
    if constexpr (H::has_engines) {
        dma_tx_remaining = DmaChannel<4>::count();
        dma_tx_credits = DmaChannel<4>::debug_credits();
        dma_rx_remaining = DmaChannel<5>::count();
        dma_rx_credits = DmaChannel<5>::debug_credits();
    }
}

/// One tenure through a host, waited out. The status, or 0xFE for a
/// tenure that never answered (the engine recover()ed).
template <typename H>
uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
               I2cSpeed speed = I2cSpeed::fast_400k, uint32_t wait_us = 200'000u) {
    typename H::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    transfer_done = false;
    i2c0_isr_entries = 0;
    if (H::start(r)) {
        return H::status();
    }
    const uint32_t t0 = us_now();
    while (!transfer_done && us_now() - t0 < wait_us) {
    }
    if (!transfer_done) {
        snapshot<H>();
        (void)H::recover();
        return 0xFEu;
    }
    return transfer_status;
}

/// Whether the wire is fitted: both lines at the pull-ups' level with
/// every peripheral off them.
bool wire_present() {
    Pin<12>::input(PinPull::none);
    Pin<13>::input(PinPull::none);
    Pin<14>::input(PinPull::none);
    Pin<15>::input(PinPull::none);
    spin_us(100);
    const bool high = Pin<12>::read() && Pin<13>::read() && Pin<14>::read() && Pin<15>::read();
    Pin<12>::release();
    Pin<13>::release();
    Pin<14>::release();
    Pin<15>::release();
    return high;
}
bool wire_or_decline() {
    if (wire_present()) {
        return true;
    }
    bench.verdict("DECLINED: the self-link's pull-ups are not fitted (GP12..GP15 do not read high with "
                  "every peripheral released) - wire I2C0 x I2C1 with two 4.7 kOhm to 3V3",
                  false);
    return false;
}

// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    bench.verdict("table 279 as the driver states it: I2C0 SDA on 0/4/../28 and SCL on 1/5/../29, I2C1 "
                  "SDA on 2/6/../26 and SCL on 3/7/../27",
                  i2c_sda_pin(0, 0) && i2c_sda_pin(0, 28) && i2c_scl_pin(0, 1) && i2c_scl_pin(0, 29) &&
                      i2c_sda_pin(1, 2) && i2c_sda_pin(1, 26) && i2c_scl_pin(1, 3) && i2c_scl_pin(1, 27) &&
                      !i2c_sda_pin(0, 2) && !i2c_scl_pin(1, 13) && !i2c_sda_pin(1, 30) &&
                      !i2c_pins_valid(0, {.scl = 12, .sda = 13}));
    for (uint8_t s = 0; s < i2c_speed_count; ++s) {
        const auto sp = static_cast<I2cSpeed>(s);
        const auto t = i2c_timing_for(SysClock::hz, sp);
        print(serial, "  ", i2c_speed_hz(sp) / 1000u, " kHz at ", SysClock::hz / 1'000'000u, " MHz: HCNT ",
              t ? t->hcnt : 0u, " LCNT ", t ? t->lcnt : 0u, " SPKLEN ", t ? t->spklen : 0u, " hold ",
              t ? t->sda_tx_hold : 0u, " setup ", t ? t->sda_setup : 0u, " -> ",
              t ? i2c_scl_hz(SysClock::hz, *t) : 0u, " Hz on an ideal wire", crlf);
    }
    bench.verdict("the timing arithmetic: the three speeds solved at 125 MHz produce their asked rate on "
                  "an ideal wire (the SPKLEN + 8 cycles the block adds subtracted from the counts)",
                  i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::standard_100k)) == 100'000u &&
                      i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::fast_400k)) >= 399'000u &&
                      i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::fast_plus_1m)) == 1'000'000u);
    bench.verdict("table 450's floors reproduced: fast mode at 12 MHz is (LCNT 15, HCNT 6), standard at 2.7 "
                  "MHz is (12, 6), fast-mode-plus needs 32 MHz, a hertz below each is refused",
                  i2c_timing_for(12'000'000u, I2cSpeed::fast_400k)->lcnt == 15u &&
                      i2c_timing_for(12'000'000u, I2cSpeed::fast_400k)->hcnt == 6u &&
                      i2c_timing_for(2'700'000u, I2cSpeed::standard_100k)->lcnt == 12u &&
                      !i2c_timing_for(11'999'999u, I2cSpeed::fast_400k).has_value() &&
                      !i2c_timing_for(31'999'999u, I2cSpeed::fast_plus_1m).has_value() &&
                      i2c_timing_for(32'000'000u, I2cSpeed::fast_plus_1m).has_value());
    using S = DwApbI2c<0>;
    const bool reset_ok = S::reset();
    const uint32_t con = S::con();
    print(serial, "  I2C0 after reset: IC_CON=", hex(con), " SPKLEN=", S::timing(I2cSpeed::fast_400k).spklen,
          " SDA_HOLD=", S::timing(I2cSpeed::fast_400k).sda_tx_hold, " enabled=", S::enabled(), " status=", hex(S::flags()),
          " IC_COMP_PARAM_1=", hex(S::regs().IC_COMP_PARAM_1), crlf);
    bench.verdict("the block's reset state: a host with its client half disabled, fast speed code, "
                  "restarts enabled (IC_CON 0x65), the spike filter at 7, the hold at 1, disabled, both "
                  "FIFOs empty - and the component parameter register reading zero, as 4.3.17 says",
                  reset_ok && con == 0x65u && S::timing(I2cSpeed::fast_400k).spklen == 7u &&
                      S::timing(I2cSpeed::fast_400k).sda_tx_hold == 1u && !S::enabled() && S::tx_empty() &&
                      !S::rx_not_empty() && S::regs().IC_COMP_PARAM_1 == 0u);
    // The transmit FIFO's depth, measured: entries queued under
    // TX_CMD_BLOCK until the FIFO says full, then flushed by a disable.
    S::enable();
    S::command_block(true);
    uint8_t depth = 0;
    while (S::tx_not_full() && depth < 64u) {
        S::push(i2c_write_entry(depth));
        ++depth;
    }
    const uint8_t counted = S::tx_count();
    const bool over = (S::raw_pending() & I2cInterrupt::tx_over) == 0u;
    S::push(i2c_write_entry(0xFF));   // one too many
    const bool over_now = (S::raw_pending() & I2cInterrupt::tx_over) != 0u;
    (void)S::disable();
    S::command_block(false);
    S::clear_all();
    print(serial, "  the transmit FIFO under TX_CMD_BLOCK: full after ", depth, " entries (TXFLR ", counted, "), TX_OVER ",
          over_now ? "raised" : "NOT raised", " on the next; after the disable TXFLR ", S::tx_count(), crlf);
    bench.verdict("the transmit FIFO is sixteen deep (4.3.1), measured under TX_CMD_BLOCK: full at sixteen, "
                  "TX_OVER on the seventeenth, flushed by the disable",
                  depth == 16u && counted == 16u && over && over_now && S::tx_count() == 0u);
    const bool cfg = S::configure({.speed = I2cSpeed::fast_plus_1m}) &&
                     S::timing(*i2c_timing_for(SysClock::hz, I2cSpeed::fast_plus_1m)) && S::target(0x42);
    const I2cTiming back = S::timing(I2cSpeed::fast_plus_1m);
    bench.verdict("a configuration, a timing and a target are taken with the block disabled and read back",
                  cfg && back.hcnt == 36u && back.lcnt == 74u && S::target() == 0x42u &&
                      (S::con() & I2C_IC_CON_SPEED_BITS) == (2u << I2C_IC_CON_SPEED_LSB));
    S::enable();
    const bool running = S::running();
    const bool refused = !S::configure({}) && !S::timing(back) && !S::target(0x10) && !S::own_address(0x11);
    (void)S::target(0x10);   // ignored by the hardware too
    bench.verdict("enabled: IC_ENABLE_STATUS reports it, and the configuration, the timing, the target "
                  "and the own address are refused",
                  running && refused && S::target() == 0x42u);
    const uint32_t t0 = us_now();
    const bool off = S::disable();
    const uint32_t took = us_now() - t0;
    print(serial, "  disable on an idle bus: ", off ? "reported" : "NOT reported", " in ", took, " us", crlf);
    bench.verdict("the disable of an idle block is reported by IC_ENABLE_STATUS within a few microseconds",
                  off && took < 20u);
    S::hold();
}

// =============================================================================
// b - the wire and the scan
// =============================================================================
void tb_scan() {
    if (!wire_or_decline()) {
        return;
    }
    bench.verdict("the self-link's pull-ups hold both lines high with every peripheral released", true);
    (void)client_up();
    host_ready();
    uint8_t found = 0;
    uint8_t first = 0;
    uint32_t nobody_us = 0;
    uint8_t nobody_status = 0;
    for (uint8_t a = 0x08; a <= 0x77u; ++a) {
        const uint32_t t0 = us_now();
        const uint8_t st = tenure<Host>(a, nullptr, 0, nullptr, 0);
        const uint32_t took = us_now() - t0;
        if (st == i2c_ok) {
            if (found == 0u) {
                first = a;
            }
            ++found;
            print(serial, "  ", hex(a), " answers (", took, " us)", crlf);
        } else if (a == nobody) {
            nobody_us = took;
            nobody_status = st;
        }
    }
    print(serial, "  scan 0x08..0x77 at 400 kHz: ", found, " answer(s); nobody home at 0x43 is status ", nobody_status,
          " in ", nobody_us, " us; the client saw ", client_read_reqs, " read request(s), ", client_stops, " stop(s)",
          crlf);
    bench.verdict("the scan finds exactly one device, at 0x42", found == 1u && first == client_address);
    bench.verdict("nobody home is i2c_nack_addr, answered inside a tenure's time (under 100 us at 400 kHz)",
                  nobody_status == i2c_nack_addr && nobody_us < 100u);
    bench.verdict("the probe is a one-byte read on this silicon: the client saw one read request and one STOP",
                  client_read_reqs == 1u && client_stops == 1u);
    Host::release();
    client_down();
}

// =============================================================================
// c - the tenure shapes
// =============================================================================
void tc_shapes() {
    if (!wire_or_decline()) {
        return;
    }
    (void)client_up();
    host_ready();
    // A write.
    client_reset_tally();
    fill_pattern(tx_buf, 8, 0x10);
    uint8_t st = tenure<Host>(client_address, tx_buf, 8, nullptr, 0);
    spin_us(200);
    print(serial, "  write 8: status ", st, ", the client heard ", client_count, ", stops ", client_stops, ", ",
          same(tx_buf, client_heard, 8) ? "byte-exact" : "MISMATCH", crlf);
    bench.verdict("a write of eight bytes: i2c_ok, the client hears eight byte-exact and one STOP",
                  st == i2c_ok && client_count == 8u && client_stops == 1u && same(tx_buf, client_heard, 8));
    // A read.
    client_reset_tally();
    client_limit = 8;
    for (uint16_t i = 0; i < 8; ++i) {
        rx_buf[i] = 0xEE;
    }
    st = tenure<Host>(client_address, nullptr, 0, rx_buf, 8);
    spin_us(200);
    print(serial, "  read 8: status ", st, ", read requests ", client_read_reqs, ", host NACK seen ", client_nacks,
          ", stops ", client_stops, ", ", same(client_answers, rx_buf, 8) ? "byte-exact" : "MISMATCH", crlf);
    if (!same(client_answers, rx_buf, 8)) {
        print(serial, "    got ");
        for (uint16_t i = 0; i < 8; ++i) {
            print(serial, hex(rx_buf[i]), " ");
        }
        print(serial, " given ", client_given, " flushes ", client_flushes, " errors ", client_errors, " isr entries ",
              i2c1_isr_entries, crlf);
    }
    bench.verdict("a read of eight bytes: i2c_ok, byte-exact, the client served eight read requests, saw the "
                  "host's NACK once and one STOP",
                  st == i2c_ok && same(client_answers, rx_buf, 8) && client_read_reqs == 8u && client_nacks == 1u &&
                      client_stops == 1u);
    // A write-then-read.
    client_reset_tally();
    client_limit = 6;
    tx_buf[0] = 0xA0;
    tx_buf[1] = 0xA1;
    for (uint16_t i = 0; i < 6; ++i) {
        rx_buf[i] = 0xEE;
    }
    st = tenure<Host>(client_address, tx_buf, 2, rx_buf, 6);
    spin_us(200);
    print(serial, "  write 2 then read 6: status ", st, ", heard ", client_count, ", restarts ", client_restarts,
          ", stops ", client_stops, ", ", same(client_answers, rx_buf, 6) ? "byte-exact" : "MISMATCH", crlf);
    bench.verdict("a write-then-read: the two bytes heard, ONE repeated START seen from the far end, six bytes "
                  "back exact, one STOP",
                  st == i2c_ok && client_count == 2u && client_heard[0] == 0xA0u && client_restarts == 1u &&
                      same(client_answers, rx_buf, 6) && client_stops == 1u);
    // The probe.
    client_reset_tally();
    st = tenure<Host>(client_address, nullptr, 0, nullptr, 0);
    spin_us(200);
    bench.verdict("the probe of a present client: i2c_ok, served as one read request and no byte written",
                  st == i2c_ok && client_read_reqs == 1u && client_count == 0u);
    // A long write through the pump.
    client_reset_tally();
    fill_pattern(tx_buf, 200, 0x30);
    const uint32_t t0 = us_now();
    st = tenure<Host>(client_address, tx_buf, 200, nullptr, 0);
    const uint32_t took = us_now() - t0;
    spin_us(200);
    print(serial, "  write 200: status ", st, " in ", took, " us, ", i2c0_isr_entries, " host interrupts, the client heard ",
          client_count, ", overruns ", client_overruns, ", ", same(tx_buf, client_heard, 200) ? "byte-exact" : "MISMATCH",
          crlf);
    bench.verdict("a 200-byte write in ONE tenure through the pump (the FIFO refilled at its half): exact, "
                  "no overrun at the client, in fewer interrupts than bytes",
                  st == i2c_ok && client_count == 200u && same(tx_buf, client_heard, 200) && client_overruns == 0u &&
                      i2c0_isr_entries < 200u && client_stops == 1u);
    Host::release();
    client_down();
}

// =============================================================================
// d - the vocabulary on the wire
// =============================================================================
void td_vocabulary() {
    if (!wire_or_decline()) {
        return;
    }
    host_ready();
    // Deaf: no client at all.
    fill_pattern(tx_buf, 4, 0x50);
    const uint8_t deaf_w = tenure<Host>(client_address, tx_buf, 4, nullptr, 0);
    const uint8_t deaf_p = tenure<Host>(client_address, nullptr, 0, nullptr, 0);
    const uint8_t deaf_r = tenure<Host>(client_address, nullptr, 0, rx_buf, 4);
    print(serial, "  deaf client: write ", deaf_w, ", probe ", deaf_p, ", read ", deaf_r, crlf);
    bench.verdict("a deaf client is i2c_nack_addr on a write, on the probe and on a read",
                  deaf_w == i2c_nack_addr && deaf_p == i2c_nack_addr && deaf_r == i2c_nack_addr);
    // The client refusing data.
    (void)client_up();
    const bool nack_armed = Client::acknowledge(false);
    client_reset_tally();
    const uint8_t refused = tenure<Host>(client_address, tx_buf, 4, nullptr, 0);
    spin_us(200);
    const uint16_t heard_refused = client_count;
    (void)Client::acknowledge(true);
    client_reset_tally();
    const uint8_t taken = tenure<Host>(client_address, tx_buf, 4, nullptr, 0);
    spin_us(200);
    print(serial, "  the client refusing data: status ", refused, ", the client stored ", heard_refused,
          " byte(s); acknowledging again: status ", taken, ", stored ", client_count, crlf);
    bench.verdict("a client refusing data (IC_SLV_DATA_NACK_ONLY) answers i2c_nack_data on the first byte, "
                  "which it does not store; acknowledging again the write lands whole",
                  nack_armed && refused == i2c_nack_data && heard_refused == 0u && taken == i2c_ok && client_count == 4u);
    // The stretch priced.
    client_reset_tally();
    client_limit = 8;
    const uint32_t t0 = us_now();
    const uint8_t plain = tenure<Host>(client_address, nullptr, 0, rx_buf, 8);
    const uint32_t plain_us = us_now() - t0;
    client_reset_tally();
    client_limit = 8;
    client_stretch_us = 100;
    const uint32_t t1 = us_now();
    const uint8_t stretched = tenure<Host>(client_address, nullptr, 0, rx_buf, 8);
    const uint32_t stretched_us = us_now() - t1;
    client_stretch_us = 0;
    print(serial, "  an 8-byte read: ", plain_us, " us plain, ", stretched_us, " us with the client holding SCL 100 us "
          "before every byte (read requests ", client_read_reqs, ", given ", client_given, ", first bytes ", hex(rx_buf[0]),
          " ", hex(rx_buf[1]), ")", crlf);
    bench.verdict("the client's clock stretch (RD_REQ holds SCL until a byte is given) is priced on the wire: "
                  "100 us before each of eight bytes lengthens the read by 700 us or more",
                  plain == i2c_ok && stretched == i2c_ok && stretched_us >= plain_us + 700u && stretched_us < plain_us + 2000u);
    // Stale bytes flushed at the next read.
    client_reset_tally();
    client_limit = 8;
    client_ahead = 4;   // four queued per request: two too many for a two-byte read
    const uint8_t two = tenure<Host>(client_address, nullptr, 0, rx_buf, 2);
    spin_us(200);
    const uint16_t flushes_after_two = client_flushes;
    const uint8_t r0 = rx_buf[0];
    const uint8_t r1 = rx_buf[1];
    client_ahead = 1;
    client_given = 0;
    const uint8_t again = tenure<Host>(client_address, nullptr, 0, rx_buf, 2);
    spin_us(200);
    print(serial, "  a two-byte read against four queued: status ", two, " [", hex(r0), " ", hex(r1), "], flushes ",
          flushes_after_two, "; the next read: status ", again, " [", hex(rx_buf[0]), " ", hex(rx_buf[1]), "]", crlf);
    bench.verdict("bytes queued beyond what the host takes are flushed at its NACK and the client told "
                  "(ABRT_SLVFLUSH_TXFIFO); the next read starts clean",
                  two == i2c_ok && r0 == client_answers[0] && r1 == client_answers[1] && flushes_after_two >= 1u &&
                      again == i2c_ok && rx_buf[0] == client_answers[0]);
    Host::release();
    client_down();
}

// =============================================================================
// e - the three speeds, measured
// =============================================================================
void te_speeds() {
    if (!wire_or_decline()) {
        return;
    }
    (void)client_up();
    host_ready();
    uint8_t exact = 0;
    uint8_t in_bracket = 0;
    for (uint8_t s = 0; s < i2c_speed_count; ++s) {
        const auto sp = static_cast<I2cSpeed>(s);
        client_reset_tally();
        client_limit = 16;
        fill_pattern(tx_buf, 16, static_cast<uint8_t>(0x60u + s));
        for (uint16_t i = 0; i < 16; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint8_t w = tenure<Host>(client_address, tx_buf, 16, nullptr, 0, sp);
        spin_us(100);
        const bool w_ok = w == i2c_ok && client_count == 16u && same(tx_buf, client_heard, 16);
        const uint8_t r = tenure<Host>(client_address, nullptr, 0, rx_buf, 16, sp);
        spin_us(100);
        const bool r_ok = r == i2c_ok && same(client_answers, rx_buf, 16);
        // The rate: a 64-byte write is 9 x 65 SCL periods plus START and STOP.
        fill_pattern(tx_buf, 64, 0x70);
        const uint32_t t0 = us_now();
        const uint8_t m = tenure<Host>(client_address, tx_buf, 64, nullptr, 0, sp);
        const uint32_t took = us_now() - t0;
        const uint32_t scl_hz = took == 0u ? 0u : static_cast<uint32_t>(9ULL * 65u * 1'000'000u / took);
        const uint32_t asked = i2c_speed_hz(sp);
        const bool bracket = m == i2c_ok && scl_hz <= asked + asked / 50u && scl_hz >= asked - asked / 4u;
        print(serial, "  ", asked / 1000u, " kHz: write ", w_ok ? "exact" : "WRONG", ", read ", r_ok ? "exact" : "WRONG",
              "; 64 bytes in ", took, " us -> SCL ", scl_hz / 1000u, " kHz (", Host::scl_hz(sp) / 1000u,
              " on an ideal wire)", bracket ? "" : "  OUT OF BRACKET", crlf);
        if (w_ok && r_ok) {
            ++exact;
        }
        if (bracket) {
            ++in_bracket;
        }
    }
    bench.verdict("the three speeds, 100 kHz, 400 kHz and 1 MHz: sixteen bytes byte-exact both ways at each",
                  exact == 3u);
    bench.verdict("the SCL rate measured on the ruler is never above the asked rate and within a quarter below "
                  "it (the pull-ups' rise time is the difference)",
                  in_bracket == 3u);
    Host::release();
    client_down();
}

// =============================================================================
// f - the DMA engines
// =============================================================================
void tf_dma() {
    if (!wire_or_decline()) {
        return;
    }
    (void)client_up();
    i2c0_owner = I2c0Owner::none;
    (void)DmaHost::init(clock);
    i2c0_owner = I2c0Owner::dma_host;
    client_reset_tally();
    client_limit = 64;
    client_ahead = 8;
    for (uint16_t i = 0; i < 64; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint32_t t0 = us_now();
    const uint8_t st = tenure<DmaHost>(client_address, nullptr, 0, rx_buf, 64, I2cSpeed::fast_400k, 20'000u);
    const uint32_t took = us_now() - t0;
    spin_us(200);
    print(serial, "  64 bytes read on the engines at 400 kHz: status ", st, " in ", took, " us, ", i2c0_isr_entries,
          " host interrupts, read requests ", client_read_reqs, ", ", same(client_answers, rx_buf, 64) ? "byte-exact" : "MISMATCH",
          ", nacks ", client_nacks, ", stops ", client_stops, crlf);
    if (st != i2c_ok) {
        uint8_t landed = 0;
        while (landed < 64u && rx_buf[landed] != 0xEEu) {
            ++landed;
        }
        print(serial, "    landed ", landed, " bytes; tx channel remaining ", dma_tx_remaining, " credits ", dma_tx_credits,
              ", rx channel remaining ", dma_rx_remaining, " credits ", dma_rx_credits, "; I2C0 raw=", hex(i2c0_raw),
              " status=", hex(i2c0_status), " rxflr=", i2c0_rxflr, " txflr=", i2c0_txflr, " abort=", hex(i2c0_abort), crlf);
    }
    bench.verdict("a 64-byte read on the two engines: the commands poured from a fixed cell, the bytes "
                  "collected, byte-exact, the host's own interrupt untouched by the bytes",
                  st == i2c_ok && same(client_answers, rx_buf, 64) && i2c0_isr_entries <= 2u && client_stops == 1u);
    client_reset_tally();
    client_limit = 32;
    client_ahead = 8;
    tx_buf[0] = 0x01;
    tx_buf[1] = 0x02;
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st2 = tenure<DmaHost>(client_address, tx_buf, 2, rx_buf, 32, I2cSpeed::fast_400k);
    spin_us(200);
    bench.verdict("a write-then-read on an engined host: the write on the pump, the repeated START, 32 "
                  "bytes back on the engines, exact",
                  st2 == i2c_ok && client_count == 2u && client_restarts == 1u && same(client_answers, rx_buf, 32));
    client_reset_tally();
    client_limit = 8;
    const uint8_t st3 = tenure<DmaHost>(client_address, nullptr, 0, rx_buf, 2, I2cSpeed::fast_400k);
    spin_us(100);
    const bool two_ok = st3 == i2c_ok && rx_buf[0] == client_answers[0] && rx_buf[1] == client_answers[1];
    client_reset_tally();
    const uint8_t st4 = tenure<DmaHost>(client_address, nullptr, 0, nullptr, 0, I2cSpeed::fast_400k);
    client_reset_tally();
    const uint8_t st5 = tenure<DmaHost>(nobody, nullptr, 0, rx_buf, 16, I2cSpeed::fast_400k);
    bench.verdict("a two-byte read and the probe stay on the pump of an engined host; a deaf client on an "
                  "engined read is i2c_nack_addr with the engines put away",
                  two_ok && st4 == i2c_ok && st5 == i2c_nack_addr);
    client_reset_tally();
    client_limit = 16;
    client_ahead = 8;
    const uint8_t st6 = tenure<DmaHost>(client_address, nullptr, 0, rx_buf, 16, I2cSpeed::fast_400k);
    spin_us(100);
    bench.verdict("... and the next engined read after that is clean", st6 == i2c_ok && same(client_answers, rx_buf, 16));
    // The bursts: the request is a level the DMA banks credits on while
    // the FIFO is empty; a burst past the FIFO's depth would drop
    // commands (TX_OVER) and shorten the tenure. Eight reads of 255.
    uint8_t exact = 0;
    bool over = false;
    for (uint8_t k = 0; k < 8u; ++k) {
        client_reset_tally();
        client_limit = 255;
        client_ahead = 8;
        for (uint16_t i = 0; i < 255; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint8_t s7 = tenure<DmaHost>(client_address, nullptr, 0, rx_buf, 255, I2cSpeed::fast_plus_1m, 50'000u);
        spin_us(100);
        over = over || (DwApbI2c<0>::raw_pending() & I2cInterrupt::tx_over) != 0u;
        if (s7 == i2c_ok && same(client_answers, rx_buf, 255)) {
            ++exact;
        }
    }
    print(serial, "  eight 255-byte reads on the engines at 1 MHz: ", exact, " exact, TX_OVER ", over ? "RAISED" : "never raised",
          crlf);
    bench.verdict("the engine's bursts never overrun the sixteen-deep FIFO: eight 255-byte reads at 1 MHz exact, "
                  "TX_OVER never raised",
                  exact == 8u && !over);
    i2c0_owner = I2c0Owner::none;
    DmaHost::release();
    client_down();
}

// =============================================================================
// g - the kernel over the bus
// =============================================================================
namespace kl {

constexpr uint32_t timeout_ticks = ticks_from_ms<P>(20);
using I2cArb = I2cBus<Host, P, 4, BusPassThrough, timeout_ticks>;

uint8_t out_a[8];
uint8_t in_a[8];

class Probe {
public:
    using Event = std::variant<I2cDone, SleepVote>;
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies[8];
    static inline uint32_t reply_at[8];
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
        match(e,
              [](const I2cDone& d) {
                  if (n < 8u) {
                      replies[n] = d.status;
                      reply_at[n] = Ticker::ticks();
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

Host::Request request(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
                      I2cSpeed speed = I2cSpeed::fast_400k) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    r.reply = reply_to<Probe, I2cDone>();
    return r;
}

}  // namespace kl

void tg_kernel() {
    if (!wire_or_decline()) {
        return;
    }
    (void)client_up();
    host_ready();
    kl::BusKernel::init_all();
    bus_ao_live = true;
    client_limit = 8;
    fill_pattern(kl::out_a, 8, 0x01);
    post<kl::I2cArb>(kl::request(client_address, kl::out_a, 8, nullptr, 0));
    post<kl::I2cArb>(kl::request(client_address, nullptr, 0, kl::in_a, 8));
    post<kl::I2cArb>(kl::request(client_address, kl::out_a, 2, kl::in_a, 4));
    post<kl::I2cArb>(kl::request(client_address, nullptr, 0, nullptr, 0));
    kl::pump_until(4, 300);
    print(serial, "  four queued tenures (write, read, write-then-read, probe): replies ", kl::Probe::n, " [",
          kl::Probe::replies[0], " ", kl::Probe::replies[1], " ", kl::Probe::replies[2], " ", kl::Probe::replies[3], "]",
          crlf);
    bench.verdict("four tenures through I2cBus, four replies - util/i2c_bus.hpp and util/bus_master.hpp "
                  "unchanged on this architecture",
                  kl::Probe::n == 4u);
    bench.verdict("... every one i2c_ok",
                  kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_ok && kl::Probe::replies[2] == i2c_ok &&
                      kl::Probe::replies[3] == i2c_ok);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(client_address, kl::out_a, 4, nullptr, 0));
    post<kl::I2cArb>(kl::request(nobody, kl::out_a, 4, nullptr, 0));
    post<kl::I2cArb>(kl::request(client_address, kl::out_a, 4, nullptr, 0));
    kl::pump_until(3, 300);
    print(serial, "  a NACK in its place: replies [", kl::Probe::replies[0], " ", kl::Probe::replies[1], " ",
          kl::Probe::replies[2], "]", crlf);
    bench.verdict("the NACK of a deaf address is delivered in its place, the tenures around it untouched",
                  kl::Probe::n == 3u && kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_nack_addr &&
                      kl::Probe::replies[2] == i2c_ok);
    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::I2cArb>(kl::request(client_address, kl::out_a, 8, nullptr, 0));
    }
    kl::pump_until(6, 300);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n, ", rejected ", kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately, and every request is still answered "
                  "exactly once",
                  kl::Probe::rejected != 0u && kl::Probe::n == 6u);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(PrepareSleep{.depth = SleepDepth::standby, .reply = reply_to<kl::Probe, SleepVote>()});
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep", kl::Probe::votes == 1u && kl::Probe::last_vote);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(client_address, kl::out_a, 8, nullptr, 0));
    post<kl::I2cArb>(PrepareSleep{.depth = SleepDepth::standby, .reply = reply_to<kl::Probe, SleepVote>()});
    kl::pump();
    kl::pump_until(2, 100);
    bench.verdict("a BUSY bus votes against it", kl::Probe::votes == 1u && !kl::Probe::last_vote);
    // The timed bus: the client holds the clock.
    kl::Probe::clear_tally();
    client_reset_tally();
    client_limit = 8;
    client_wedged = true;
    const uint32_t t0 = Ticker::ticks();
    post<kl::I2cArb>(kl::request(client_address, nullptr, 0, kl::in_a, 8));
    kl::pump_until(1, 200);
    const uint32_t answered_ms = kl::Probe::n != 0u ? kl::Probe::reply_at[0] - t0 : 0u;
    const bool scl_low = !Pin<13>::read();
    print(serial, "  a read against a client holding SCL: reply ", kl::Probe::n != 0u ? kl::Probe::replies[0] : 0xFFu,
          " after ", answered_ms, " ms, SCL ", scl_low ? "still low" : "released", crlf);
    bench.verdict("THE TIMED BUS: a client holding the clock (a read request never served) is answered "
                  "i2c_timeout at the arbiter's 20 ms with SCL still held - no silicon on this chip answers a "
                  "held clock",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_timeout && answered_ms >= 19u && answered_ms <= 25u);
    // The client freed and re-armed; the recover()ed engine carries on.
    client_wedged = false;
    Client::clear_read_request();
    client_down();
    (void)client_up();
    client_limit = 8;
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(client_address, kl::out_a, 8, nullptr, 0));
    post<kl::I2cArb>(kl::request(client_address, nullptr, 0, kl::in_a, 8));
    kl::pump_until(2, 300);
    bench.verdict("... and the recover()ed engine carries the next two tenures clean",
                  kl::Probe::n == 2u && kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_ok &&
                      same(client_answers, kl::in_a, 8));
    bus_ao_live = false;
    Host::release();
    client_down();
}

// =============================================================================
// h - the stuck bus
// =============================================================================
void th_unstick() {
    if (!wire_or_decline()) {
        return;
    }
    host_ready();
    const uint8_t healthy = Host::unstick();
    // SDA held low by a GPIO at the far end: a short, as far as nine
    // clocks can tell.
    Pin<14>::output(false);
    spin_us(10);
    const uint8_t held = Host::unstick();
    Pin<14>::release();
    spin_us(10);
    const uint8_t again = Host::unstick();
    (void)client_up();
    const uint8_t probe = tenure<Host>(client_address, nullptr, 0, nullptr, 0);
    print(serial, "  unstick: healthy ", healthy, ", SDA held by a GPIO ", hex(held), ", released ", again,
          "; the probe after: ", probe, crlf);
    bench.verdict("unstick() reads the wire before clocking it: 0 on a healthy bus, 0xFF when SDA stays low "
                  "through nine clocks and a STOP, 0 again once released - and the bus works after it",
                  healthy == 0u && held == 0xFFu && again == 0u && probe == i2c_ok);
    Host::release();
    client_down();
}

// =============================================================================
// i - the roles invert
// =============================================================================
void ti_roles() {
    if (!wire_or_decline()) {
        return;
    }
    i2c0_owner = I2c0Owner::none;
    (void)client_ready<ClientA>(16, 0x90);
    i2c0_owner = I2c0Owner::client_a;
    i2c1_owner = I2c1Owner::none;
    (void)HostB::init(clock);
    i2c1_owner = I2c1Owner::host_b;
    fill_pattern(tx_buf, 16, 0x80);
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t w = tenure<HostB>(client_address, tx_buf, 16, nullptr, 0);
    spin_us(200);
    const bool heard = client_count == 16u && same(tx_buf, client_heard, 16);
    const uint8_t r = tenure<HostB>(client_address, nullptr, 0, rx_buf, 16);
    spin_us(200);
    print(serial, "  I2C1 hosting on GP15/14, I2C0 listening on GP13/12: write ", w, " (", heard ? "exact" : "WRONG",
          "), read ", r, " (", same(client_answers, rx_buf, 16) ? "exact" : "WRONG", ")", crlf);
    bench.verdict("the roles invert on the same two wires: sixteen bytes exact both ways",
                  w == i2c_ok && heard && r == i2c_ok && same(client_answers, rx_buf, 16));
    i2c1_owner = I2c1Owner::none;
    HostB::release();
    i2c0_owner = I2c0Owner::none;
    ClientA::release();
}

// =============================================================================
// x - a traced read (diagnostics, outside z)
// =============================================================================
void tx_trace() {
    if (!wire_or_decline()) {
        return;
    }
    (void)client_up();
    host_ready();
    using C = DwApbI2c<1>;
    print(serial, "  client after init: raw=", hex(C::raw_pending()), " status=", hex(C::flags()), " txflr=", C::tx_count(),
          " con=", hex(C::con()), " sar=", hex(C::own_address()), crlf);
    for (uint8_t round = 0; round < 3u; ++round) {
        client_reset_tally();
        client_limit = 4;
        client_stretch_us = round == 2u ? 100u : 0u;
        for (uint16_t i = 0; i < 4; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint32_t t0 = us_now();
        const uint8_t st = tenure<Host>(client_address, nullptr, 0, rx_buf, 4);
        const uint32_t t1 = us_now();
        spin_us(300);
        print(serial, "  round ", round, " (stretch ", client_stretch_us, "): status ", st, " in ", t1 - t0, " us, got ",
              hex(rx_buf[0]), " ", hex(rx_buf[1]), " ", hex(rx_buf[2]), " ", hex(rx_buf[3]), "; requests ", client_read_reqs,
              " given ", client_given, " nacks ", client_nacks, " stops ", client_stops, " flushes ", client_flushes,
              " errors ", client_errors, "; client raw=", hex(C::raw_pending()), " txflr=", C::tx_count(), crlf);
        print(serial, "    request stamps from t0:");
        for (uint8_t i = 0; i < client_stamp_n; ++i) {
            print(serial, " ", client_stamp[i] - t0);
        }
        print(serial, crlf);
    }
    Host::release();
    client_down();
}

void banner() {
    print(serial, crlf, "test_rp2040_i2c - the RP2040 I2C (datasheet 4.3, DW_apb_i2c): I2C0 hosts on GP13/GP12, "
          "I2C1 listens at 0x42 on GP15/GP14; clk_sys=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_i2c0() {
    i2c0_isr_entries = i2c0_isr_entries + 1u;
    switch (i2c0_owner) {
        case I2c0Owner::host:
            if (Host::isr()) {
                transfer_status = Host::status();
                transfer_done = true;
                if (bus_ao_live) {
                    brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
                }
            }
            break;
        case I2c0Owner::dma_host:
            if (DmaHost::isr()) {
                transfer_status = DmaHost::status();
                transfer_done = true;
            }
            break;
        case I2c0Owner::client_a:
            client_serve<ClientA>(ClientA::service());
            break;
        default:
            brio::DwApbI2c<0>::interrupts_only(0);
            brio::DwApbI2c<0>::clear_all();
            break;
    }
}
extern "C" void isr_i2c1() {
    i2c1_isr_entries = i2c1_isr_entries + 1u;
    switch (i2c1_owner) {
        case I2c1Owner::client:
            client_serve<Client>(Client::service());
            break;
        case I2c1Owner::host_b:
            if (HostB::isr()) {
                transfer_status = HostB::status();
                transfer_done = true;
            }
            break;
        default:
            brio::DwApbI2c<1>::interrupts_only(0);
            brio::DwApbI2c<1>::clear_all();
            break;
    }
}
extern "C" void isr_dma_0() {
    if (i2c0_owner == I2c0Owner::dma_host && DmaHost::dma_isr()) {
        transfer_status = DmaHost::status();
        transfer_done = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless", ta_block);
    bench.letter('b', "the wire and the scan", tb_scan);
    bench.letter('c', "the tenure shapes", tc_shapes);
    bench.letter('d', "the vocabulary on the wire", td_vocabulary);
    bench.letter('e', "the three speeds, measured", te_speeds);
    bench.letter('f', "the DMA engines", tf_dma);
    bench.letter('g', "the kernel: I2cBus over I2cHost, the timed bus", tg_kernel);
    bench.letter('h', "the stuck bus", th_unstick);
    bench.letter('i', "the roles invert", ti_roles);
    bench.letter('x', "a traced read (diagnostics, outside z)", tx_trace, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
                    " dma=", dma_ok ? "released" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
