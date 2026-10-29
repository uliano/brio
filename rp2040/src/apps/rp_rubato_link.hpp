// rp_rubato_link.hpp - THE LINK SUITE UNDER RUBATO OF THE TWO RASPBERRY PI
// CHIPS, written once over the ARM PL022 and PL011 (brio/pl022/spi.hpp,
// brio/pl011/uart.hpp) and bound by test_rp2040_rubato_link and
// test_rp2350_rubato_link: this chip's SPI1 and UART1 stacks run inside
// Rubato, the preemptive kernel, against the desk partner's unchanged
// `spi_peer` (rp_spi_peer.hpp, a PIO client) and `uart_peer`
// (rp_uart_peer.hpp, a fixed-role echo at 115200), while the pack's highest
// object preempts them at prime periods. Every verdict is the traffic
// judged byte-exact.
//
// TWO SHARED BODIES, ONE ABOVE THE OTHER. What names no register at all -
// the pack of seven objects, the passes and the pressure's verdict, the
// numbered lines and their borrower, the spi_link.hpp window client, letter
// w's bookkeeping, the waits, the UART legs - is every family's
// (avrxt/src/apps/rubato_link.hpp, whose header describes each object and
// the passes). What the two chips share because they carry the same two
// IP blocks - the PL022 host's paths and the shapes letter p climbs, the
// PL011's interrupt receiver under LineReader, the uart_peer's mode line -
// is this file's, as rp_spi_pair.hpp and rp_uart_pair.hpp are for the
// Tenuto suites: one register description, so one body, not a copy.
//
// What the binding states before it includes this file, inside its own
// anonymous namespace with `using namespace brio` in force, having included
// what rubato_link.hpp asks at file scope:
//
//   SysClock, clock, P, core_hz, cycles_per_us, Ruler, spin()
//   Serial, serial, bench    the console (UART0 on GP0/GP1) and its
//                            TwoKeyTestBench
//   Host                     SpiHost<1, {sck, tx, rx}> with the half-word
//                            engines on DMA channels 6 and 7, line 1
//   CsPin, SckPin, TxPin     the link's select (a GPIO), clock and transmit
//                            pads as Pin types
//   ModeLine                 this board's end of the uart_peer's mode line
//   uart_link_pins           UART1's two link pads
//   pressure_name, peer_name what the prints call the pressure's source
//                            and the partner board
//   partner_kind,            spi_link's kind byte of the peer, and its
//   partner_sys_hz           clk_sys, which sets letter p's ceiling
//   core1_pass               whether letters o and t run a pass with
//                            Pressure fed by core 1 (the RP2040's alone)
//   pressure_core1           the flag the binding's pressure verbs read:
//                            the pass in progress is core 1's
//
// and defines after it pressure_arm/go/halt/quiet (rubato_link.hpp's
// declarations), pressure_silent() with its verdict's text, and
// console_banner(); its glue binds isr_spi1 (Host::isr, a TransferDone to
// the arbiter on true), isr_dma_1 (Host::dma_isr, the same), isr_uart1
// (link_rx_vector() below) and isr_uart0 (the console, a post to the
// Console).
//
// THE SPI HOST. SpiHost<1> on the pair's wires with half-word engines, the
// one host every path goes through, under util/spi_bus.hpp's arbiter. The
// PL022's 8-deep FIFOs are the write-ahead on every path (pl022/spi.hpp,
// "The FIFOs are the write-ahead"): the polled loop in its receive shape
// primes eight frames and from there writes one for each one it reads, the
// pump fills while fewer than eight are in flight, so whatever holds either
// off, the frames that can land in the receive FIFO are never more than it
// holds. The polled loop's waits are a budget counted in loop turns, which
// a preemption does not spend. So the prediction here is the STM32G0's,
// not the STM32F4's: STRICT at every judged rung under every pressure,
// and the overrun flag (SSPRIS.RORRIS, which nothing on these paths
// clears) never set at a reply. The transmit-only shape (run_write), the
// one that lets the receive FIFO overrun by design, is taken only for a
// write longer than the FIFO with no in buffer at a divisor under four,
// which no window here asks. MSb FIRST ONLY: the PL022 has no bit-order
// field, so the matrix has no host-LSb rows; its LSb rows are the client
// LSb against this host's MSb, an exact two-way bit reversal the PIO
// client serves (rp_spi_peer.hpp). The host stays InFlash, the default:
// a cold request is longer from the flash and the XIP cache's misses are
// printed with the matrix's rate, cleared at every pass (the counters
// saturate at 32 bits, rp2040/flash.hpp's Xip).
//
// THE UART. Uart<1> on GP4/GP5 with the INTERRUPT receiver: RXIM at the
// receive FIFO's half (16 of 32) and RTIM after 32 bit periods of a quiet
// line, the handler draining the FIFO into a SkipRing of 1024
// (pl011/uart.hpp). Not the receive engine: under it no vector ends a
// burst - the time-out needs a character waiting and the channel's single
// request leaves none - so a partial run waits for harvest(), which
// LineReader never calls. Where the STM32 families tell the port at the
// idle line and at a ring's half lap, this one tells it at a FIFO level
// and at its time-out. The receive stream's position the borrower reads is
// the link vector's entry count.
//
// THE UART KEY IS REDUCED (the peer, and the gap it leaves). The partner's
// `uart_peer` is a fixed-role firmware at 115200 8N1 whose role a mode
// line picks - low ECHO, high SEND - with no command channel, no report
// and no rate change (rp_uart_peer.hpp). So key y runs at 115200 alone,
// with the mode line held low, judged at this end by the borrower's parse
// and the port's counters; letter u, the blast judged by a peer's report,
// cannot run. At 115200 a character is 86.8 us: a 3000-cycle preemption
// (24 us at 125 MHz, 20 at 150) is a quarter of one, so neither the 32-deep
// FIFO nor the ring is stressed by it; what the key proves under
// preemption is the stack's logic - the edges, the drain, the loan - not
// its headroom. The full key needs a usart_link peer on these chips.
//
// THE WIRES, fixed on the desk (docs/boards/weact-rp2040.md,
// docs/boards/weact-rp2350b.md): SPI1 SCK, the two data lines and the
// select (1.5 kOhm to the RP2040's 3V3), UART1 crossed, the two mode lines
// crossed, the I2C pair (unused here, the RP2350's sense of its partner's
// rail), two grounds. Every letter brings its pads up and gives them back:
// nothing here drives a link pad between letters.

#pragma once

// =============================================================================
// The names rubato_link.hpp asks of a family, the RP chips' alike
// =============================================================================

/// The command channel's SCK: clk_peri / 128 - 977 kHz on the RP2040, 1.17
/// MHz on the RP2350, far under the PIO client's ceiling.
constexpr SpiClock link_clock = SpiClocks::div128;
constexpr const char* bus_clock_name = "clk_peri";
static_assert(SysClock::pclk_hz == SysClock::hz, "clk_peri is clk_sys: a frame's cycles are bits x the divisor");

/// UART1 with the interrupt receiver (the file header): a ring of 1024 for
/// what the borrower holds the reader off of, 256 to transmit.
using LinkPort = Uart<1, uart_link_pins, 1024, 256>;
/// The link vector's receive entries: the borrower's view of the stream.
volatile uint32_t link_rx_entries = 0;
/// The Uart task owns UART1's vector while a leg runs.
volatile bool link_live = false;

/// What a leg's transport counted (init() clears the error counters; the
/// skip epoch is read as a difference).
struct PortCounters {
    uint8_t ring_overruns = 0, oe = 0, fe = 0, pe = 0, be = 0;
    uint32_t skips = 0;
    bool clean() const { return ring_overruns == 0u && oe == 0u && fe == 0u && pe == 0u && be == 0u && skips == 0u; }
};

// THE TARGET-FREE HALF IS SHARED, NOT COPIED: every family's link suite
// compiles the same body around its own engines. The PL022 has no
// bit-order field (pl022/spi.hpp) and no bit-order verb, so the shared
// body clocks every window MSb first and this file posts no LSb window.
#include "../../../avrxt/src/apps/rubato_link.hpp"

// =============================================================================
// What the shared body asks of this family
// =============================================================================

uint32_t rx_stream_position() { return link_rx_entries; }

uint32_t division(SpiClock c) { return c.divisor(); }

/// The host's overrun flag at a reply: the receive shapes never let it rise
/// (the file header) and nothing on their paths clears it, so a set flag is
/// a frame lost since the last reply; it is cleared here so that each reply
/// speaks for its own transaction.
volatile uint32_t ovr_at_reply = 0;
void client_replied() {
    if ((Host::Resource::raw_pending() & SpiInterrupt::overrun) != 0u) {
        ovr_at_reply = ovr_at_reply + 1u;
        Host::Resource::clear_pending(SpiInterrupt::overrun);
    }
}

uint32_t skips_at_up = 0;

bool task_up(uint32_t rate) {
    link_live = true;
    if (!LinkPort::init(clock, rate)) {
        link_live = false;
        return false;
    }
    skips_at_up = LinkPort::rx_skips();
    return true;
}

void task_down() {
    LinkPort::release();
    link_live = false;
}

PortCounters read_port_counters() {
    PortCounters c{};
    c.ring_overruns = LinkPort::rx_overruns();
    c.oe = LinkPort::hw_overruns();
    c.fe = LinkPort::frame_errors();
    c.pe = LinkPort::parity_errors();
    c.be = LinkPort::break_errors();
    c.skips = LinkPort::rx_skips() - skips_at_up;
    return c;
}

void print_port(const PortCounters& c) {
    print(serial, "ring overruns ", c.ring_overruns, ", OE ", c.oe, ", FE ", c.fe, ", PE ", c.pe, ", BE ", c.be,
          ", skips ", c.skips);
}

/// The link vector's body: every entry counted (the stream's position),
/// the port's edge posted to the reader while a leg runs.
[[gnu::always_inline]] inline void link_rx_vector() {
    link_rx_entries = link_rx_entries + 1u;
    if (link_live && LinkPort::isr()) {
        post<LinePort>(RxActivity{});
    }
}

// =============================================================================
// The UART key's peer: the mode line, no command channel
// =============================================================================

namespace ul {

/// The peer's one rate (rp_uart_peer.hpp's link_baud).
constexpr uint32_t peer_baud = 115200;

/// THE MODE LINE LOW: the peer in ECHO by the time this returns. A peer
/// that was SENDING finishes its 64-byte burst first (5.6 ms) and reads the
/// line again within one turn of its loop; 20 ms covers both with this
/// end's port still down, so no byte of the burst is taken here
/// (rp_uart_pair.hpp's peer_echo()).
void echo_mode() {
    (void)ModeLine::output(false);
    wait_ms(20);
}

/// The mode line given back: the peer's pull-down holds it low.
void release() { (void)ModeLine::release(); }

/// The shared body's command: the one action this peer has, ECHO.
bool peer_act(link::Op, const link::Params&) {
    echo_mode();
    return true;
}

/// This peer keeps no report: none, and false (the letters judge at this
/// end and never read one).
bool peer_report(link::Report& r, uint16_t) {
    r = link::Report{};
    return false;
}

/// Is the peer there? One numbered line through the whole stack and back
/// within 100 ms (a line is 6 ms of wire there and back at 115200): a named
/// failure, never a hang.
bool ensure() {
    echo_mode();
    LineSink::reset(60000);
    bool back = false;
    if (task_up(peer_baud)) {
        wait_ms(2);
        const uint8_t n = line_with_eol(60000);
        back = write_line("\n", 1) && write_line(line_buf, n) &&
               wait_until([] { return LineSink::good >= 1u; }, 100);
        task_down();
    }
    if (!back) {
        print(serial, "  THE PEER DID NOT ANSWER: ", peer_name, " must be running `uart_peer`", crlf);
        bench.verdict("the peer echoes a line (the partner running uart_peer, the mode line low)", false);
        release();
    }
    return back;
}

}  // namespace ul

// =============================================================================
// The pads of the SPI key
// =============================================================================

/// SPI1 up for a letter: the host on its pads, the select a GPIO idling
/// high. Every letter of key z starts here and ends in spi_down().
bool spi_up() {
    const bool ok = Host::init(clock);
    (void)CsPin::output(true);
    return ok;
}

/// SPI1's pads back to their reset state, the block in reset: the select
/// held high by its pull-up to the RP2040's 3V3 meanwhile.
void spi_down() {
    Host::release();
    (void)CsPin::release();
}

// =============================================================================
// The core-1 pass (letters o and t, where the binding has it)
// =============================================================================

/// The core-1 pass: Pressure fed through the inbox's doorbell at 97 us, its
/// dispatch spinning 3000 cycles, the source named in its label (the shared
/// body's PassSpec) and judged by the shared arithmetic - the system
/// timer's microseconds and clk_sys share the crystal.
constexpr PassSpec core1_spec{97, 3000, "core 1's doorbell"};

/// The XIP cache's misses since its counters were cleared, both cores'.
/// The hits are read first, so the reads' own fetches land on the accesses'
/// side as hits.
uint32_t xip_misses() {
    const uint32_t hits = Xip::hits();
    return Xip::accesses() - hits;
}

// =============================================================================
// n - the peer's command channel
// =============================================================================

void tn_peer_link() {
    if (!spi_up()) {
        bench.verdict("SPI1 up on the link's pads", false);
        spi_down();
        return;
    }
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        const bool there = sl::need_peer();
        spilink::Ident d{};
        const bool got = there && sl::peer_ident(d);
        uint8_t pings = 0;
        for (uint8_t i = 0; there && i < 10u; ++i) {
            if (sl::command(spilink::Op::ping)) {
                ++pings;
            }
        }
        const PassStats st = pass_end(s);
        print_pass(s, st);
        if (got) {
            print(serial, "    peer: label '");
            for (uint8_t i = 0; i < 8u && d.label[i] != '\0'; ++i) {
                print(serial, d.label[i]);
            }
            print(serial, "' sanity=", hex(d.sanity), " fw=", hex(d.version), " xtal=", d.xtal, "; ", pings,
                  " of 10 pings", crlf);
        }
        bench.verdict(pass_label(s), "ident is spi_peer's (the sanity byte), the partner's kind, on its crystal",
                      got && d.sanity == spilink::ident_sanity && spilink::peer_kind(d.version) == partner_kind &&
                          d.xtal == 1u);
        bench.verdict(pass_label(s), "ten more frames answered, every one through the arbiter", pings == 10u);
        judge_pressure(s, st);
    }
    spi_down();
}

// =============================================================================
// o - the matrix
// =============================================================================

/// The matrix's rate: clk_peri / 32 (3.9 and 4.7 MHz) on the engines, 32
/// frames a window, past dma_min_frames on both chips.
constexpr SpiClock matrix_rate = SpiClocks::div32;
static_assert(Host::dma_min_frames <= 32u && Host::wide_engines, "the matrix's windows ride the engines");

/// One pass of the matrix: the four modes x two widths x the client MSb and
/// LSb (the LSb rows an exact two-way reversal against this MSb-only host),
/// 32 frames a window; the rate of the windows that came back exact.
void matrix_pass(const PassSpec& s, uint64_t& control_bytes, uint64_t& control_cycles, bool control) {
    pass_begin(s);
    Xip::reset_counters();
    if (!sl::need_peer()) {
        (void)pass_end(s);
        return;
    }
    uint8_t good = 0, total = 0;
    uint32_t bytes = 0, cycles = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        for (uint8_t wide = 0; wide < 2u; ++wide) {
            for (uint8_t dord = 0; dord < 2u; ++dord) {
                sl::Exchange e{};
                e.cfg.mode = m;
                e.cfg.dord = dord;
                e.host_mode = static_cast<SpiMode>(m);
                e.wide = wide != 0u;
                e.rate = matrix_rate;
                e.count = 32;
                e.seed_a = static_cast<uint8_t>(0x13u + 7u * total);
                e.seed_b = static_cast<uint8_t>(0x57u + 5u * total);
                const sl::Outcome o = sl::run(e);
                ++total;
                if (sl::exact(e, o)) {
                    ++good;
                    bytes += e.wide ? 64u : 32u;
                    cycles += o.cycles;
                } else {
                    sl::print_outcome(e, o);
                }
            }
        }
    }
    const bool ping = sl::command(spilink::Op::ping);
    const uint32_t misses = xip_misses();
    const PassStats st = pass_end(s);
    print_pass(s, st);
    const uint32_t rate_x10 =
        cycles != 0u ? static_cast<uint32_t>(static_cast<uint64_t>(bytes) * core_hz / 100u / cycles) : 0u;
    if (control) {
        control_bytes = bytes;
        control_cycles = cycles;
    }
    const uint32_t vs = control_cycles != 0u && cycles != 0u && bytes == control_bytes
                            ? static_cast<uint32_t>(control_cycles * 1000u / cycles)
                            : 0u;
    print(serial, "    ", good, " of ", total, " exchanges exact; the data windows carried ", bytes, " bytes in ",
          cycles / cycles_per_us, " us, ", rate_x10 / 10u, ".", rate_x10 % 10u, " kB/s (", vs,
          " permille of the control's rate); XIP misses over the pass ", misses, crlf);
    bench.verdict(pass_label(s), "four modes x two widths x the client MSb and LSb: every window byte-exact at "
                                 "both ends on the DMA engines",
                  good == total && total == 16u);
    bench.verdict(pass_label(s), "the command channel still answers after the matrix", ping);
    judge_pressure(s, st);
}

void to_matrix() {
    if (!spi_up()) {
        bench.verdict("SPI1 up on the link's pads", false);
        spi_down();
        return;
    }
    uint64_t control_bytes = 0, control_cycles = 0;
    for (uint8_t p = 0; p < pass_count; ++p) {
        matrix_pass(passes[p], control_bytes, control_cycles, p == 0u);
    }
    if constexpr (core1_pass) {
        pressure_core1 = true;
        matrix_pass(core1_spec, control_bytes, control_cycles, false);
        pressure_core1 = false;
    }
    spi_down();
}

// =============================================================================
// p - the ladders
// =============================================================================

/// An even divisor of clk_peri as the PL022's pair: CPSDVSR 2, SCR the rest.
constexpr SpiClock divisor_clock(uint16_t d) { return SpiClock{2, static_cast<uint8_t>(d / 2u - 1u)}; }

/// THE CEILING JUDGED, the PIO client's (rp_spi_pair.hpp's header derives
/// it): the peer's answer bit leaves its pad four of ITS clk_sys cycles
/// after the SCK edge that asks for it - two of input synchronizer, the
/// WAIT that sees the edge, the OUT - and this host samples it half an SCK
/// period after that edge, so the half period must clear 4 / partner_sys_hz
/// plus 15 ns of two pads and the wire: the smallest even divisor that
/// does, in picoseconds to keep the arithmetic exact. 12 on the RP2040
/// against the RP2350's 150 MHz (48 ns against 41.7), 16 on the RP2350
/// against the RP2040's 125 MHz (53.3 ns against 47).
constexpr uint16_t judged_divisor() {
    const uint64_t need_ps = 4'000'000'000'000ull / partner_sys_hz + 15'000ull;
    for (uint16_t d = 2; d <= 512u; d = static_cast<uint16_t>(d + 2u)) {
        const uint64_t half_ps = 1'000'000'000'000ull * d / (2ull * SysClock::pclk_hz);
        if (half_ps >= need_ps) {
            return d;
        }
    }
    return 512;
}
constexpr uint16_t judged_top = judged_divisor();

/// The rungs: every one at or below the ceiling judged, and the next two
/// faster printed with their counts, not judged.
constexpr uint16_t rung_list[] = {256, 128, 64, 32, 24, 16, 14, 12, 10, 8};

enum class Path : uint8_t { polled8, polled16, pump8, pump16, engines8, engines_polled16 };
constexpr uint8_t path_count = 6;
const char* path_name(Path p) {
    switch (p) {
        case Path::polled8: return "polled loop, 8-bit, 9 frames";
        case Path::polled16: return "polled loop, 16-bit, 32 frames off the engines";
        case Path::pump8: return "pump, 8-bit, 9 frames";
        case Path::pump16: return "pump, 16-bit, 32 frames off the engines";
        case Path::engines8: return "DMA engines, 8-bit, 32 frames";
        default: return "DMA engines under a polled request, 16-bit, 32 frames";
    }
}

/// THE SHAPES, each from the host's own rule (pl022/spi.hpp, engines_serve):
/// a data phase under dma_min_frames takes the polled loop or the pump; a
/// 16-bit one whose buffers are not half-word aligned takes them at any
/// length (the DMA would move a half-word across an odd address wrongly).
/// So 9 frames - one past the FIFO, so the polled loop refills once and the
/// pump twice - and 32 16-bit frames from an odd address, 24 refills.
static_assert(9u < Host::dma_min_frames, "letter p's 9-frame windows take the polled loop and the pump");
static_assert(32u >= Host::dma_min_frames, "letter p's 32-frame aligned windows ride the engines");

/// A frame's core cycles at divisor `d`: bits x the divisor, clk_peri =
/// clk_sys.
uint32_t frame_cycles(uint16_t d, bool wide) { return (wide ? 16u : 8u) * static_cast<uint32_t>(d); }

/// The informational rungs' outcomes, printed after the pass: how many
/// paths, and which (bit i the path i of Path).
uint8_t info_exact[2] = {};
uint8_t info_total[2] = {};
uint8_t info_mask[2] = {};

bool climb(Path path, uint8_t seed) {
    bool all = true;
    uint8_t k = 0;
    uint8_t info = 0;
    for (const uint16_t d : rung_list) {
        const bool judged = d >= judged_top;
        if (!judged && info >= 2u) {
            break;
        }
        sl::Exchange e{};
        e.rate = divisor_clock(d);
        e.polled = path == Path::polled8 || path == Path::polled16 || path == Path::engines_polled16;
        e.wide = path == Path::polled16 || path == Path::pump16 || path == Path::engines_polled16;
        e.odd = path == Path::polled16 || path == Path::pump16;
        e.count = path == Path::polled8 || path == Path::pump8 ? 9u : 32u;
        e.seed_a = static_cast<uint8_t>(seed + 9u * k);
        e.seed_b = static_cast<uint8_t>(seed + 0x40u + 5u * k);
        ++k;
        const sl::Outcome o = sl::run(e);
        const bool ok = sl::exact(e, o);
        if (!judged) {
            ++info_total[info];
            if (ok) {
                ++info_exact[info];
                info_mask[info] = static_cast<uint8_t>(info_mask[info] | (1u << static_cast<uint8_t>(path)));
            }
            ++info;
            continue;
        }
        if (!ok) {
            print(serial, "    ", path_name(path), " (a frame ", frame_cycles(d, e.wide), " cycles):", crlf);
            sl::print_outcome(e, o);
        }
        all = all && ok;
    }
    return all;
}

void tp_ladders() {
    if (!spi_up()) {
        bench.verdict("SPI1 up on the link's pads", false);
        spi_down();
        return;
    }
    uint16_t info_div[2] = {};
    {
        uint8_t i = 0;
        for (const uint16_t d : rung_list) {
            if (d < judged_top && i < 2u) {
                info_div[i++] = d;
            }
        }
    }
    print(serial, "  every path keeps at most 8 frames in flight (the FIFOs); the engines take a phase of ",
          Host::dma_min_frames, " frames or more; judged to clk_peri/", judged_top, " (", SysClock::pclk_hz / judged_top / 1000u,
          " kHz, the PIO client's ceiling at ", partner_sys_hz / 1'000'000u, " MHz); /", info_div[0], " and /",
          info_div[1], " printed, not judged", crlf);
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!sl::need_peer()) {
            (void)pass_end(s);
            spi_down();
            return;
        }
        ovr_at_reply = 0;
        info_exact[0] = info_exact[1] = 0;
        info_total[0] = info_total[1] = 0;
        info_mask[0] = info_mask[1] = 0;
        bool ok[path_count];
        for (uint8_t i = 0; i < path_count; ++i) {
            ok[i] = climb(static_cast<Path>(i), static_cast<uint8_t>(0x21u + 0x0Cu * i));
        }
        const uint32_t ovr = ovr_at_reply;
        const bool ping = sl::command(spilink::Op::ping);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    the host's overrun flag at a reply over the pass: ", ovr, "; /", info_div[0], " exact on ",
              info_exact[0], " of ", info_total[0], " paths (mask ", hex(info_mask[0]), "), /", info_div[1], " on ",
              info_exact[1], " of ", info_total[1], " (mask ", hex(info_mask[1]), "); the mask's bits from 0: polled "
              "8-bit, polled 16-bit, pump 8-bit, pump 16-bit, engines, engines polled",
              crlf);
        // A dispatch above the arbiter holds the polled loop off for its
        // whole length; with the FIFO as the write-ahead that is a gap on
        // the bus and never a lost frame (the file header), so the verdict
        // under pressure is the control's.
        bench.verdict(pass_label(s), "THE POLLED LOOP, 8-bit: 9 frames byte-exact both ways at every rung to the "
                                     "ceiling",
                      ok[0]);
        bench.verdict(pass_label(s), "THE POLLED LOOP, 16-bit: 32 frames off the engines byte-exact at every rung",
                      ok[1]);
        bench.verdict(pass_label(s), "THE PUMP, 8-bit: 9 frames byte-exact at every rung", ok[2]);
        bench.verdict(pass_label(s), "THE PUMP, 16-bit: 32 frames off the engines byte-exact at every rung", ok[3]);
        bench.verdict(pass_label(s), "THE DMA ENGINES, 8-bit: 32 frames byte-exact at every rung", ok[4]);
        bench.verdict(pass_label(s), "THE DMA ENGINES under a polled request, 16-bit: 32 frames byte-exact at every "
                                     "rung",
                      ok[5]);
        bench.verdict(pass_label(s), "the host's overrun flag never set at a reply: no frame lost", ovr == 0u);
        bench.verdict(pass_label(s), "the command channel survives the ladders", ping);
        judge_pressure(s, st);
    }
    spi_down();
}

// =============================================================================
// w - the arbiter under pressure
// =============================================================================

void tw_arbiter() {
    if (!spi_up()) {
        bench.verdict("SPI1 up on the link's pads", false);
        spi_down();
        return;
    }
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        const bool pressing = s.period_us != 0u;
        pass_begin(s);
        if (!sl::need_peer()) {
            (void)pass_end(s);
            spi_down();
            return;
        }
        const uint16_t frames = static_cast<uint16_t>(w::client_n * w::client_len + w::minor_n * w::minor_len +
                                                      (pressing ? w::press_n * w::press_len : 0u));
        spilink::Params a{};
        a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
        a.count = frames;
        a.ms = 2000;
        a.seed_a = w::seed_a;
        a.seed_b = w::seed_b;
        a.pattern = spilink::pattern_fixed;
        const bool acted = sl::peer_act(spilink::Op::exchange, a);
        for (auto& r : w::client_rx) for (uint8_t& b : r) b = w::untouched;
        for (auto& r : w::minor_rx) for (uint8_t& b : r) b = w::untouched;
        for (auto& r : w::press_rx) for (uint8_t& b : r) b = w::untouched;
        w::clear(w::client);
        w::clear(w::minor);
        w::clear(w::press);
        Pressure::outstanding = false;
        const uint8_t rejected0 = SpiArb::rejected_count();
        const uint8_t stale0 = SpiArb::stale_events();
        const uint32_t t0 = Ruler::now();
        bool done = false;
        if (acted) {
            // The window: the mode primed with the select high (a CPOL flip
            // inside an open window is an edge for the client), the 30 us
            // the protocol pays on each side of it. The bus is idle here: no
            // request is outstanding.
            sl::settle();
            Host::prime(SpiMode::mode0, w::rate);
            CsPin::clear();
            wait_us(30);
            w::live = true;
            w::press_live = pressing;
            post<Client>(Kick{});
            post<Minor>(Kick{});
            done = wait_until(
                [pressing] {
                    return w::client.replies >= w::client_n && w::minor.replies >= w::minor_n &&
                           (!pressing || w::press.replies >= w::press_n);
                },
                2000);
            w::press_live = false;
            w::live = false;
            wait_us(30);
            CsPin::set();
        }
        const uint32_t took = Ruler::now() - t0;
        spilink::Report r{};
        const bool rep = acted && sl::peer_report(r);
        uint16_t host_bad = 0;
        for (auto& row : w::client_rx) for (uint8_t b : row) host_bad = static_cast<uint16_t>(host_bad + (b != w::seed_b));
        for (auto& row : w::minor_rx) for (uint8_t b : row) host_bad = static_cast<uint16_t>(host_bad + (b != w::seed_b));
        if (pressing) {
            for (auto& row : w::press_rx) for (uint8_t b : row) host_bad = static_cast<uint16_t>(host_bad + (b != w::seed_b));
        }
        const uint8_t rejected = static_cast<uint8_t>(SpiArb::rejected_count() - rejected0);
        const uint8_t stale = static_cast<uint8_t>(SpiArb::stale_events() - stale0);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    ", took / cycles_per_us, " us; Client ", w::client.replies, "/", w::client.posted,
              " replies (", w::client.ok, " ok, ", w::client.out_of_order, " out of order), Minor ",
              w::minor.replies, "/", w::minor.posted, " (", w::minor.ok, " ok, ", w::minor.out_of_order,
              " out of order), Pressure ", w::press.replies, "/", w::press.posted, " (", w::press.ok, " ok, ",
              w::press.out_of_order, " out of order); timeouts ",
              w::client.timeouts + w::minor.timeouts + w::press.timeouts, ", rejected ", rejected, ", stale ", stale,
              "; host bytes wrong ", host_bad, "; peer count ", r.count, " of ", frames, " mism ", r.mism, crlf);
        if (host_bad != 0u) {
            print(serial, "    windows not the peer's:");
            for (uint8_t k = 0; k < w::client_n; ++k) {
                if (!w::complete(w::client_rx[k], w::client_len)) {
                    print(serial, " C", k, "=");
                    for (uint8_t b : w::client_rx[k]) print(serial, hex(b), ",");
                }
            }
            for (uint8_t k = 0; k < w::minor_n; ++k) {
                if (!w::complete(w::minor_rx[k], w::minor_len)) {
                    print(serial, " M", k);
                }
            }
            print(serial, "; peer's first wrong byte ", r.idx, " got ", hex(r.got), crlf);
        }
        const bool all_ok = done && w::client.ok == w::client_n && w::minor.ok == w::minor_n &&
                            (!pressing || w::press.ok == w::press_n);
        bench.verdict(pass_label(s), "every request answered spi_ok, to its own requester, once", all_ok &&
                      w::client.replies == w::client_n && w::minor.replies == w::minor_n &&
                      w::press.replies == (pressing ? w::press_n : 0u));
        bench.verdict(pass_label(s), "every reply in its requester's order (the Client's before its next "
                                     "window starts)",
                      w::client.out_of_order == 0u && w::minor.out_of_order == 0u && w::press.out_of_order == 0u);
        bench.verdict(pass_label(s), "nothing rejected, the per-bus timeout never fired (no spi_timeout, no "
                                     "stale event)",
                      rejected == 0u && stale == 0u && w::client.timeouts == 0u && w::minor.timeouts == 0u &&
                          w::press.timeouts == 0u && w::client.rejected == 0u && w::minor.rejected == 0u &&
                          w::press.rejected == 0u);
        bench.verdict(pass_label(s), "the peer counted every frame exact, and every frame back is the peer's",
                      rep && r.count == frames && r.mism == 0u && host_bad == 0u);
        judge_pressure(s, st);
    }
    spi_down();
}

// =============================================================================
// i - dark
// =============================================================================

/// The binding's: its pressure source stopped, its line quiet (and on the
/// RP2040 core 1 parked), and the verdict's name that says so.
bool pressure_silent();
const char* pressure_silent_text();

void ti_dark() {
    bench.verdict("every object above the Console idle: every queue empty",
                  Pressure::queue.empty() && LineSink::queue.empty() && Client::queue.empty() &&
                      SpiArb::queue.empty() && Minor::queue.empty() && LinePort::queue.empty());
    bench.verdict(pressure_silent_text(), pressure_silent());
    bench.verdict("no time event is armed (the arbiter's timeout among them)", !TE::next_deadline().has_value());
    bench.verdict("SPI1 given back: the block disabled, its clock and transmit pads and the select at no function "
                  "and not outputs",
                  !Host::Resource::enabled() && SckPin::function() == PinFunction::none &&
                      TxPin::function() == PinFunction::none && CsPin::function() == PinFunction::none &&
                      !CsPin::is_output());
    bench.verdict("UART1 given back and the mode line released: none of the three an output, the Uart task down",
                  !link_live && Pin<uart_link_pins.tx.pin>::function() == PinFunction::none && !ModeLine::is_output());
    bench.verdict("the LED's pad GP25 is not an output", !Pin<25>::is_output());
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
}

// =============================================================================
// t, v - the UART, reduced
// =============================================================================

constexpr uint16_t echo_lines = 200;

/// A leg's time against its wire's.
void print_echo(const char* what, const LegResult& g) {
    print(serial, "    ", what, " at ", ul::peer_baud, ": ", g.bytes, " bytes in ", g.cycles / cycles_per_us,
          " us (the wire's ", wire_cycles(g.bytes, ul::peer_baud) / cycles_per_us, " us, ",
          g.cycles != 0u ? static_cast<uint32_t>(static_cast<uint64_t>(wire_cycles(g.bytes, ul::peer_baud)) * 1000u /
                                                 g.cycles)
                         : 0u,
          " permille of it)", crlf);
}

void echo_pass(const PassSpec& s, uint8_t p) {
    pass_begin(s);
    if (!ul::ensure()) {
        (void)pass_end(s);
        return;
    }
    LineSink::spin_cycles = 0;
    const LegResult g = echo_leg(ul::peer_baud, echo_lines, static_cast<uint16_t>(7u * p));
    const PassStats st = pass_end(s);
    ul::release();
    print_echo("echo", g);
    print_counters(g.c);
    print_sink();
    print_pass(s, st);
    // The leg judged at this end: the peer keeps no report.
    judge_echo(s, ul::peer_baud, echo_lines, g, false);
    judge_pressure(s, st);
}

void tt_lines() {
    for (uint8_t p = 0; p < pass_count; ++p) {
        echo_pass(passes[p], p);
    }
    if constexpr (core1_pass) {
        pressure_core1 = true;
        echo_pass(core1_spec, pass_count);
        pressure_core1 = false;
    }
}

/// THE BORROWER'S SPIN AND THE WINDOW, at 115200. The sink holds each line
/// for sixteen character times, one receive level of the FIFO (1.39 ms), so
/// the link vector lands inside nearly every one of its dispatches and the
/// reader lends a line while the next arrives; the shortest line is 18
/// characters (1.56 ms), so the sink still keeps up with the stream. The
/// window holds the writer at five lines and the line in flight, 5 x 71 +
/// 70 = 425 characters at most, under the peer's 512-byte rings - all a
/// backlog at the echo could ever be - and under this end's ring of 1024.
/// Pressure inside the borrower: 200 dispatches of 1.39 ms, 0.28 s, are 714
/// periods of the longest pressure (389 us): a pass with none has a chance
/// of e^-714.
constexpr uint32_t sink_spin = static_cast<uint32_t>(16ull * 10u * core_hz / ul::peer_baud);
constexpr uint8_t loan_window = 5;
static_assert(loan_window * 71u + 70u < 512u, "the window keeps the writer inside the peer's rings");

void tv_loan() {
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!ul::ensure()) {
            (void)pass_end(s);
            return;
        }
        LineSink::spin_cycles = sink_spin;
        const LegResult g = echo_leg(ul::peer_baud, echo_lines, static_cast<uint16_t>(3000u + 13u * p), loan_window);
        LineSink::spin_cycles = 0;
        const PassStats st = pass_end(s);
        ul::release();
        print_echo("echo, the sink slow", g);
        print_counters(g.c);
        print_sink();
        print(serial, "    the sink's dispatches took ", LineSink::busy / 1000u, " k cycles; the expected ticks "
              "inside them ", s.period_us != 0u ? LineSink::busy / (static_cast<uint32_t>(s.period_us) * cycles_per_us) : 0u,
              crlf);
        print_pass(s, st);
        judge_echo(s, ul::peer_baud, echo_lines, g, false);
        bench.verdict(pass_label(s), "every line lent alone (none waiting at the sink's entry, the sink never "
                                     "nested), the two buffers strictly alternating",
                      LineSink::waiting == 0u && LineSink::nested == 0u && LineSink::same_buffer == 0u &&
                          LineSink::distinct <= 2u && LineSink::lines >= echo_lines);
        if (s.period_us != 0u) {
            bench.verdict(pass_label(s), "Pressure landed inside the borrower's dispatch", st.land[sink_index] != 0u);
        }
        judge_pressure(s, st);
    }
}
