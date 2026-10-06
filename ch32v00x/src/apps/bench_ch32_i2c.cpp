// bench_ch32_i2c - letter i of the CH32V00x's benchmark (design/
// benchmark.md, util/bench.hpp) in an image of its own: the I2C host
// (ch32v00x/i2c.hpp) against a PEER BOARD, one bench line per tenure
// shape, size, speed and engine, in the grammar every family prints.
//
// WHY AN IMAGE OF ITS OWN: bench_ch32 fills nearly all of the CH32V006's
// 40 KB of program flash (ch32v00x/ld/ch32v006k8.ld stops the image short of
// the NV partition), and this letter - two instantiations of the host,
// the pump's and the engines', and the peer's command channel - is about
// ten more. The ruler, the idle adapter, the meters and the console are
// bench_ch32's, spelled the same way, so a line here reads as a line
// there; letter r prints the instrument's cost for this image.
//
// NOT A TEST: every letter's one verdict is "ran", and letter i's "ran"
// is false when the peer did not answer or its count of the bytes moved
// disagrees with this side's.
//
// THE BOARD AND THE WIRE. The CH32V006K8U6 at 48 MHz (the HSI doubled by
// the PLL), the console USART1 on PD5/PD6 through the WCH-Link at 115200.
// I2C1 on its default pads, PC2 SCL and PC1 SDA, to a PEER BOARD running
// `twi_peer` (an STM32G0 Nucleo on the bench, its PB8/PB9, its 2.2 kOhm
// pull-ups - the module carries none), commanded in band over the same
// two wires through avrdx/src/apps/twi_link.hpp: the peer SERVES at 0x2C
// for a bounded window, counting the bytes it takes and gives and summing
// those it takes, its answers the protocol's counting pattern; after the
// window its report is held against this side's tally.
//
// What each letter prints:
//
//   r  the instrument's cost: one ruler read and one stamp pair (enter()
//      and leave() of a meter bound to nothing), each the average of 100
//      from a fresh tick, n=0 and wire=0 - what the reader subtracts.
//   i  THE I2C HOST AGAINST THE PEER, at 100 kHz and 400 kHz (this block
//      has no Fm+): the SCL period MEASURED as the slope of two engined
//      writes of 16 and 255 bytes, then every op through the pump
//      (`i2c.*`, I2cHost<1>) and through the engines (`i2c.*.dma`,
//      DmaTxEngine<6> + DmaRxEngine<7>): i2c.write and i2c.read of 1, 2,
//      16 and 255 bytes, i2c.wr (one byte written, a repeated START, 1, 2
//      and 16 read; n counts both), i2c.probe (0x2C) and i2c.probe.nack
//      (0x23, nobody), i2c.write.nack (1 and 2 bytes to 0x23: the NACK
//      named from the address). Each the best of 8 runs, every byte read
//      judged against the peer's pattern - a run whose tenure failed is
//      not taken, nor one whose wall is under the wire, which no tenure
//      can be: such a wall is a fault of the instrument, counted, printed
//      and failing the letter; the line's irq and isr are
//      I2C1's two vectors, channels 6 and 7 and the tick; a second line
//      gives the wire in cycles, "the rest" above it (the fixed cost of
//      the tenure) and start() alone (the best of 8, the completion
//      waited outside the clock).
//
// THE WIRE: nine SCL periods a byte with its acknowledge (the address
// included), one for the START and the STOP together and one more for a
// repeated START, at the period measured at that speed; the wire field
// is the tenure's bytes over that time, so x = wall over the bus's own
// time. A probe carries no data byte: n=0 and x is "-".
//
// THE INSTRUMENT'S COST: a Stopwatch holds about one ruler read beyond
// the operation, a metered handler about one stamp pair beyond its body
// (letter r prints both); the hardware prologue's entry and exit lie
// outside the stamps (docs/ch32v00x/platform.md).
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/i2c.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// The peer protocol: the wire format every twi_peer speaks, shared by
// relative path, not copied.
#include "../../../avrdx/src/apps/twi_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

/// The STK's cycle count, bench_ch32's ruler.
struct Ruler {
    [[gnu::always_inline, gnu::flatten]] static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

using Plat = BenchIdle<Ch32v00xPlatform<>, Ruler>;
using Meter = IsrMeter<Ruler, Plat>;

using Serial = Uart<1, Plat>;   // USART1 on PD5/PD6
constexpr Serial serial;
constexpr uint32_t console_baud = 115200;

TestBench<Serial> bench;

Meter tick_meter;    // the STK's vector
Meter usart_meter;   // USART1's vector
Meter i2c_meter;     // I2C1's two vectors and DMA channels 6 and 7
Meter probe_meter;   // bound to nothing: letter r's stamp line

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    while (!Serial::Resource::tx_complete()) {
    }
}

void loop_wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 <= ms) {
    }
}

void fresh_tick() {
    const uint32_t t = Ticker::ticks();
    while (Ticker::ticks() == t) {
    }
}

// ---------------------------------------------------------------------------
// r - the instrument's cost
// ---------------------------------------------------------------------------
volatile uint32_t ruler_sink = 0;

void tr_costs() {
    console_drain();
    fresh_tick();
    Stopwatch<Ruler> sw;
    sw.start();
    for (uint8_t i = 0; i < 100u; ++i) {
        ruler_sink = Ruler::now();
    }
    const uint32_t reads = sw.elapsed();
    bench_line(serial, "ruler", 0, BenchSample{reads / 100u, reads / 100u, 0, 0}, Ruler::hz(), 0);
    fresh_tick();
    const uint32_t isr0 = probe_meter.cycles();
    sw.start();
    for (uint8_t i = 0; i < 100u; ++i) {
        probe_meter.enter();
        probe_meter.leave();
        __asm__ volatile("" ::: "memory");
    }
    const uint32_t pairs = sw.elapsed();
    bench_line(serial, "stamp", 0,
               BenchSample{pairs / 100u, pairs / 100u, 1, (probe_meter.cycles() - isr0) / 100u},
               Ruler::hz(), 0);
    bench.verdict("ran", true);
}

uint8_t ram[512];

// ---------------------------------------------------------------------------
// i - the I2C host against the peer board (ch32v00x/i2c.hpp over
//     design/i2c-bus.md)
// ---------------------------------------------------------------------------
using I2cPump = I2cHost<1>;
using I2cDma = I2cHost<1, i2c1_default_pins, DmaTxEngine<6>, DmaRxEngine<7>>;
using I2cSclPin = Pin<'C', 2>;
using I2cSdaPin = Pin<'C', 1>;
constexpr uint8_t i2c_nobody = 0x23;

/// Which host owns I2C1's vectors and channels 6 and 7 (two tasks over one
/// peripheral share its registers and not their statics); the vectors
/// read it.
enum class I2cMode : uint8_t { none, pump, dma };
volatile I2cMode i2c_mode = I2cMode::none;
volatile bool i2c_done = false;

/// The bytes the peer took and gave since its serve began, for the report
/// it is held to at the end of each speed, and the sum of those it took.
uint32_t peer_took = 0;
uint32_t peer_gave = 0;
uint16_t peer_sum = 0;
uint8_t peer_seed = 0;

BenchCounters i2c_counters() { return bench_counters<Plat>(tick_meter, i2c_meter); }

template <typename Host>
void i2c_bring_up(I2cMode mode) {
    i2c_mode = I2cMode::none;
    (void)Host::init(clock);
    i2c_mode = mode;
}

/// One tenure through `Host`, waited out on the idle path; false when it
/// never answered (the host then recover()ed).
template <typename Host>
bool i2c_run(const typename Host::Request& r) {
    i2c_done = false;
    if (Host::start(r)) {
        return true;
    }
    const uint32_t t0 = Ticker::millis();
    for (;;) {
        Plat::CriticalSection cs;
        if (i2c_done) {
            return true;
        }
        if (Ticker::millis() - t0 > 40u) {
            (void)Host::recover();
            return false;
        }
        Plat::idle();
    }
}

/// One tenure on the plain host, the peer's command channel's way.
uint8_t link_tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len) {
    I2cPump::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = I2cSpeed::standard_100k;
    return i2c_run<I2cPump>(r) ? I2cPump::status() : uint8_t{200};
}

/// One command to the peer (test_ch32_i2c's channel, compact): the frame
/// written, then the ack read back; three attempts.
bool peer_command(twilink::Op op, const uint8_t* p, uint8_t len) {
    uint8_t frame[twilink::max_payload + 4];
    uint8_t resp[twilink::response_bytes];
    for (uint8_t k = 0; k < 3u; ++k) {
        uint8_t n = 0;
        twilink::write_frame([&](uint8_t b) { frame[n++] = b; }, op, p, len);
        if (link_tenure(twilink::command_addr, frame, n, nullptr, 0) == i2c_ok) {
            loop_wait_ms(2);
            if (link_tenure(twilink::command_addr, nullptr, 0, resp, sizeof resp) == i2c_ok) {
                twilink::Decoder dec;
                for (const uint8_t b : resp) {
                    if (dec.feed(b) == twilink::Decoder::Result::frame) {
                        const twilink::Frame& f = dec.frame();
                        if (f.op == twilink::Op::ack && f.data[0] == twilink::byte_of(op)) {
                            loop_wait_ms(twilink::arm_ms);
                            return true;
                        }
                    }
                }
            }
        }
        loop_wait_ms(400);
    }
    return false;
}

/// The report of the peer's last action: one more command, its data
/// frame collected by a second read.
bool peer_report(twilink::Report& out) {
    if (!peer_command(twilink::Op::report, nullptr, 0)) {
        return false;
    }
    loop_wait_ms(2);
    uint8_t resp[twilink::response_bytes];
    if (link_tenure(twilink::command_addr, nullptr, 0, resp, sizeof resp) != i2c_ok) {
        return false;
    }
    twilink::Decoder dec;
    for (const uint8_t b : resp) {
        if (dec.feed(b) == twilink::Decoder::Result::frame &&
            dec.frame().op == twilink::Op::report_data) {
            out = twilink::get_report(dec.frame().data);
            return true;
        }
    }
    return false;
}

enum class I2cShape : uint8_t { write, read, wr, probe_ack, probe_nack, write_nack };

/// The SCL periods a tenure holds: nine a byte with its acknowledge (the
/// address included), one for the START and the STOP together, one more
/// for a repeated START.
constexpr uint32_t i2c_periods(I2cShape k, uint8_t n) {
    switch (k) {
        case I2cShape::write:
        case I2cShape::read: return 9u * (1u + n) + 1u;
        case I2cShape::wr: return 9u * (2u + 1u + n) + 2u;
        default: return 10u;
    }
}

/// The measured SCL period of each speed, in sixteenths of a cycle.
uint32_t i2c_period_x16[2] = {};

/// Walls the best-of filters refused for being under the bus's own time:
/// an instrument fault, never a measurement, so the letter does not pass
/// with one in it.
uint32_t impossible_walls = 0;

uint8_t* i2c_tx() { return ram; }
uint8_t* i2c_rx() { return ram + 256; }

template <typename Host>
typename Host::Request i2c_request(I2cShape k, uint8_t n, I2cSpeed speed) {
    typename Host::Request r{};
    const bool nobody = k == I2cShape::probe_nack || k == I2cShape::write_nack;
    r.addr = nobody ? i2c_nobody : twilink::dut_addr;
    r.speed = speed;
    if (k == I2cShape::write || k == I2cShape::wr || k == I2cShape::write_nack) {
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(i2c_tx()));
        r.tx_len = k == I2cShape::wr ? 1u : n;
    }
    if (k == I2cShape::read || k == I2cShape::wr) {
        r.rx = lend<Lease::reply>(i2c_rx());
        r.rx_len = n;
    }
    return r;
}

/// One op of letter i: the best of 8 runs, the bench line, then the wire,
/// the fixed cost and start() alone (its best of 8, the completion
/// waited outside the clock).
template <typename Host>
void run_i2c(const char* op, I2cShape k, uint8_t n, I2cSpeed speed) {
    const typename Host::Request r = i2c_request<Host>(k, n, speed);
    const uint8_t s_ix = static_cast<uint8_t>(speed);
    // The bus's own time for this tenure: a wall below it is impossible,
    // and the best of the runs takes no such wall and no failed tenure.
    const uint32_t wire = i2c_periods(k, n) * i2c_period_x16[s_ix] / 16u;
    console_drain();
    BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
    bool ok = true;
    bool data = true;
    uint8_t below = 0;
    uint32_t lowest = 0xFFFF'FFFFu;
    for (uint8_t run = 0; run < 8u; ++run) {
        for (uint32_t i = 0; i < n; ++i) {
            i2c_tx()[i] = static_cast<uint8_t>(0x40u + run + i);
        }
        const BenchCounters before = i2c_counters();
        Stopwatch<Ruler> sw;
        sw.start();
        const bool done = i2c_run<Host>(r);
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, before, i2c_counters());
        const bool nobody = k == I2cShape::probe_nack || k == I2cShape::write_nack;
        const bool st = done && Host::status() == (nobody ? i2c_nack_addr : i2c_ok);
        ok = ok && st;
        if (st && !nobody) {
            for (uint8_t i = 0; i < r.tx_len; ++i) {
                peer_sum = static_cast<uint16_t>(peer_sum + i2c_tx()[i]);
            }
            peer_took += r.tx_len;
            for (uint8_t i = 0; i < r.rx_len; ++i) {
                data = data && i2c_rx()[i] ==
                                   twilink::pattern_value(twilink::pattern_counting, peer_seed,
                                                          static_cast<uint16_t>(peer_gave + i));
            }
            peer_gave += r.rx_len;
        }
        if (s.wall < wire) {
            ++below;
            lowest = s.wall < lowest ? s.wall : lowest;
        } else if (st && s.wall < best.wall) {
            best = s;
        }
    }
    impossible_walls += below;
    uint32_t launch = 0xFFFF'FFFFu;
    for (uint8_t run = 0; run < 8u; ++run) {
        i2c_done = false;
        uint32_t d = 0;
        bool sync = false;
        {
            Plat::CriticalSection cs;
            const uint32_t t0 = Ruler::now();
            sync = Host::start(r);
            d = Ruler::now() - t0;
        }
        if (!sync) {
            const uint32_t t0 = Ticker::millis();
            for (;;) {
                Plat::CriticalSection cs;
                if (i2c_done || Ticker::millis() - t0 > 40u) {
                    break;
                }
                Plat::idle();
            }
            if (!i2c_done) {
                (void)Host::recover();
            }
        }
        if (i2c_done && Host::status() == i2c_ok) {
            for (uint8_t i = 0; i < r.tx_len; ++i) {
                peer_sum = static_cast<uint16_t>(peer_sum + i2c_tx()[i]);
            }
            peer_took += r.tx_len;
            peer_gave += r.rx_len;
        }
        if (d < launch) {
            launch = d;
        }
    }
    const uint32_t bytes = k == I2cShape::wr ? 1u + n : k == I2cShape::probe_nack ? 0u : n;
    const uint32_t wire_bps =
        bytes == 0u || wire == 0u
            ? 0u
            : static_cast<uint32_t>(static_cast<uint64_t>(bytes) * Ruler::hz() / wire);
    bench_line(serial, op, bytes, best, Ruler::hz(), wire_bps);
    print(serial, "  the wire ", wire, " cycles (", i2c_periods(k, n), " periods), the rest ",
          best.wall > wire ? best.wall - wire : 0u, ", start() ", launch,
          data ? ", data exact" : ", DATA WRONG", ok ? "" : ", A TENURE FAILED");
    if (below != 0u) {
        print(serial, ", ", below, " WALLS BELOW THE WIRE REFUSED (the lowest ", lowest, ")");
    }
    print(serial, crlf);
}

/// The SCL period at `speed`, measured as the slope of two engined
/// writes, 16 and 255 bytes, the best of 4 each.
void measure_period(I2cSpeed speed) {
    uint32_t walls[2] = {};
    uint8_t j = 0;
    for (const uint8_t n : {uint8_t{16}, uint8_t{255}}) {
        const I2cDma::Request r = i2c_request<I2cDma>(I2cShape::write, n, speed);
        // The register's own SCL rate is the fastest the bus can run (CCR
        // rounded up, the rise time only adding): a wall under that wire
        // is impossible and refused like a failed tenure.
        const uint32_t floor = i2c_periods(I2cShape::write, n) * (Ruler::hz() / I2cDma::scl_hz(speed));
        uint32_t best = 0xFFFF'FFFFu;
        for (uint8_t run = 0; run < 4u; ++run) {
            Stopwatch<Ruler> sw;
            sw.start();
            const bool ok = i2c_run<I2cDma>(r) && I2cDma::status() == i2c_ok;
            const uint32_t wall = sw.elapsed();
            if (ok) {
                for (uint8_t i = 0; i < n; ++i) {
                    peer_sum = static_cast<uint16_t>(peer_sum + i2c_tx()[i]);
                }
                peer_took += n;
            }
            if (wall < floor) {
                ++impossible_walls;
            } else if (ok && wall < best) {
                best = wall;
            }
        }
        walls[j++] = best;
    }
    const uint8_t s_ix = static_cast<uint8_t>(speed);
    i2c_period_x16[s_ix] = (walls[1] - walls[0]) * 16u / (9u * 239u);
    const uint32_t p16 = i2c_period_x16[s_ix] == 0u ? 1u : i2c_period_x16[s_ix];
    print(serial, "  ", i2c_speed_hz(speed) / 1000u, " kHz: the measured SCL period ", p16 / 16u,
          " and ", p16 % 16u, "/16 cycles = ", Ruler::hz() * 16u / p16,
          " Hz; the register's own ", I2cPump::scl_hz(speed), " Hz", crlf);
}

template <typename Host>
void i2c_ops(const char* const names[6], I2cSpeed speed) {
    for (const uint8_t n : {uint8_t{1}, uint8_t{2}, uint8_t{16}, uint8_t{255}}) {
        run_i2c<Host>(names[0], I2cShape::write, n, speed);
    }
    for (const uint8_t n : {uint8_t{1}, uint8_t{2}, uint8_t{16}, uint8_t{255}}) {
        run_i2c<Host>(names[1], I2cShape::read, n, speed);
    }
    for (const uint8_t n : {uint8_t{1}, uint8_t{2}, uint8_t{16}}) {
        run_i2c<Host>(names[2], I2cShape::wr, n, speed);
    }
    run_i2c<Host>(names[3], I2cShape::probe_ack, 0, speed);
    run_i2c<Host>(names[4], I2cShape::probe_nack, 0, speed);
    for (const uint8_t n : {uint8_t{1}, uint8_t{2}}) {
        run_i2c<Host>(names[5], I2cShape::write_nack, n, speed);
    }
}

void ti_i2c() {
    print(serial, "  I2C1 the host on PC2 SCL / PC1 SDA (the pump, and DmaTxEngine<6> + "
                  "DmaRxEngine<7>), the PEER BOARD's twi_peer serving at 0x2C, the wire's "
                  "pull-ups the peer's",
          crlf);
    I2cSclPin::input();
    I2cSdaPin::input();
    loop_wait_ms(1);
    if (!I2cSclPin::read() || !I2cSdaPin::read()) {
        print(serial, "  SKIPPED: SCL/SDA read low at rest - no pull-ups, no peer on the wire",
              crlf);
        bench.verdict("ran", false);
        return;
    }
    static const char* const pump_names[6] = {"i2c.write",      "i2c.read",
                                              "i2c.wr",         "i2c.probe",
                                              "i2c.probe.nack", "i2c.write.nack"};
    static const char* const dma_names[6] = {"i2c.write.dma",      "i2c.read.dma",
                                             "i2c.wr.dma",         "i2c.probe.dma",
                                             "i2c.probe.nack.dma", "i2c.write.nack.dma"};
    bool all = true;
    impossible_walls = 0;
    for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
        // The peer serves at 0x2C for a bounded window, counting what it
        // takes and what it gives; the window is waited out after the
        // ops and its report held against this side's tally.
        const uint16_t window_ms = speed == I2cSpeed::standard_100k ? 9000u : 5000u;
        i2c_bring_up<I2cPump>(I2cMode::pump);
        twilink::Params a{};
        a.count = 0;
        a.ms = window_ms;
        a.addr = twilink::dut_addr;
        a.seed = static_cast<uint8_t>(0x20u + static_cast<uint8_t>(speed));
        a.pattern = twilink::pattern_counting;
        uint8_t pp[twilink::params_size];
        twilink::put_params(pp, a);
        if (!peer_command(twilink::Op::serve, pp, twilink::params_size)) {
            print(serial, "  THE PEER DID NOT ANSWER: the peer board must run twi_peer", crlf);
            all = false;
            break;
        }
        const uint32_t served_at = Ticker::millis();
        peer_took = 0;
        peer_gave = 0;
        peer_sum = 0;
        peer_seed = a.seed;
        i2c_bring_up<I2cDma>(I2cMode::dma);
        measure_period(speed);
        i2c_bring_up<I2cPump>(I2cMode::pump);
        i2c_ops<I2cPump>(pump_names, speed);
        i2c_bring_up<I2cDma>(I2cMode::dma);
        i2c_ops<I2cDma>(dma_names, speed);
        const uint32_t used = Ticker::millis() - served_at;
        i2c_bring_up<I2cPump>(I2cMode::pump);
        while (Ticker::millis() - served_at < window_ms + 300u) {
        }
        twilink::Report rep{};
        const bool got = peer_report(rep);
        const bool agree = got && rep.count == static_cast<uint16_t>(peer_took + peer_gave) &&
                           rep.sum == peer_sum;
        all = all && agree;
        print(serial, "  the peer's report: ", got ? "" : "NONE, ", "count ", rep.count, " sum ",
              hex(rep.sum), " address matches ", rep.addr_hits, "; this side's tally ",
              peer_took + peer_gave, " (", peer_took, " taken, ", peer_gave, " given) sum ",
              hex(peer_sum), agree ? " - AGREE" : " - DISAGREE", "; the ops took ", used,
              " of the window's ", window_ms, " ms", crlf);
    }
    i2c_mode = I2cMode::none;
    I2cPump::release();
    if (impossible_walls != 0u) {
        print(serial, "  ", impossible_walls, " WALLS UNDER THE WIRE were refused: the ruler or the "
                      "harness misread a tenure",
              crlf);
    }
    bench.verdict("ran", all && impossible_walls == 0u);
}

void banner() {
    print(serial, crlf, "bench_ch32_i2c - ", device::part_name,
          " (clk=48 MHz PLL, ruler=STK cycles at 48 MHz, console=USART1 ", console_baud,
          " 8N1): letter i of the benchmark in an image of its own", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" BRIO_CH32_INTERRUPT void systick_handler() {
    tick_meter.enter();
    brio::Ticker::tick();
    tick_meter.leave();
}

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() {
    usart_meter.enter();
    (void)Serial::isr();
    usart_meter.leave();
}

// I2C1's event and error vectors and the engines' channels 6 and 7: the
// host letter i brought up, or nothing.
extern "C" BRIO_CH32_INTERRUPT void i2c1_ev_handler() {
    i2c_meter.enter();
    bool done = false;
    if (i2c_mode == I2cMode::pump) {
        done = I2cPump::isr();
    } else if (i2c_mode == I2cMode::dma) {
        done = I2cDma::isr();
    }
    i2c_meter.leave();
    if (done) {
        i2c_done = true;
    }
}
extern "C" BRIO_CH32_INTERRUPT void i2c1_er_handler() {
    i2c_meter.enter();
    bool done = false;
    if (i2c_mode == I2cMode::pump) {
        done = I2cPump::error_isr();
    } else if (i2c_mode == I2cMode::dma) {
        done = I2cDma::error_isr();
    }
    i2c_meter.leave();
    if (done) {
        i2c_done = true;
    }
}
namespace {
[[gnu::always_inline]] inline void i2c_dma_vector() {
    i2c_meter.enter();
    bool done = false;
    if (i2c_mode == I2cMode::dma) {
        done = I2cDma::dma_isr();
    }
    i2c_meter.leave();
    if (done) {
        i2c_done = true;
    }
}
} // namespace
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() { i2c_dma_vector(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { i2c_dma_vector(); }

int main() {
    const bool clock_ok = SysClock::init();   // HSI x2 -> 48 MHz
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('r', "the instrument's cost: a ruler read, a stamp pair", tr_costs);
    bench.letter('i', "the I2C host against the peer board: write, read, register read, probe; "
                      "pump and engines at 100 k and 400 k",
                 ti_i2c);

    if (!serial_ok) {
        for (;;) {
        }
    }
    brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED", " tick=",
                tick_ok ? "STK" : "FAILED", brio::crlf);
    banner();
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            Plat::CriticalSection cs;
            if (Serial::rx_pending() == 0u) {
                Plat::idle();
            }
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
        bench.prompt();
    }
}
