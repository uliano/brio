// rp_spi_peer.hpp - THE spi_peer OF THE TWO RASPBERRY PI CHIPS, written once
// and bound by rp2040/src/apps/spi_peer.cpp and rp2350/src/apps/spi_peer.cpp:
// a scriptable SPI CLIENT for the far end of a two-board SPI test, commanded
// IN BAND over the very bus both are testing through
// avrdx/src/apps/spi_link.hpp.
//
// ONE BODY, TWO CHIPS: everything below is the PIO's, the PL022's and the
// protocol's; what is a chip's - the four pads, the PIO's window onto them,
// the clock, the console, the ruler, the label, the partner-rail guard -
// the binding file states before it includes this one, inside its own
// anonymous namespace with `using namespace brio` in force:
//
//   SysClock, clock          the app's Clock type and object
//   Console, console         the console UART type and object
//   sck_gpio, rx_gpio,       the link's four pads as SPI1 names them on this
//   tx_gpio, cs_gpio         chip: the clock, this chip's RECEIVE pad (the
//                            host's MOSI while this end is the client), its
//                            TRANSMIT pad (the answer line), the select
//   SckPin, RxPin, TxPin,    the same four as Pin types
//   CsPin
//   Blk, Sm, pio_function    the PIO block, its machine 0, the pad function
//                            that hands a pad to that block
//   pio_index(gpio)          a system GPIO as the block numbers it
//                            (constexpr), and open_pio_window() - whatever
//                            the block needs to see the four pads
//   Host                     SpiHost<1, {sck, tx, rx, cs}>: the PL022 host
//                            on the same four pads, the select its own
//                            frame signal (THE ROLES INVERT below)
//   PeerTx, PeerRx           half-word DMA engines for the exchange
//   us_now()                 the microsecond ruler (the system timer)
//   peer_kind, peer_label    spi_link's kind byte and the Ident label
//   link_text                the pads, for the banner
//   xtal_ok                  the crystal came up (Ident.xtal)
//   link_pads_free()         may a link pad be driven now (the RP2350's
//                            partner-rail guard; true on the RP2040)
//
// WHY THE CLIENT IS A PIO PROGRAM AND NOT THE PL022. spi_link's command
// channel is mode 0 under a GPIO select held for a whole frame of bytes,
// and the PL022 as a client takes ONE frame per select window whenever
// SPH = 0 (RP2040 4.4.3.10 and 4.4.3.12: the select freezes the slave's
// shift register, so its host must pulse it between frames). A state
// machine has no such rule: it shifts on SCK edges and frames on the
// select only where its program says.
//
// THE PROGRAMS, one per mode, written from the PIO chapter (RP2040 3.4,
// RP2350 11.4) and loaded side by side at boot - six instructions in modes
// 0 and 2, five in modes 1 and 3, twenty-two of the thirty-two slots. With
// L the level SCK takes at a frame's leading edge (high for CPOL = 0, low
// for CPOL = 1):
//
//   SPH = 0   out pins,1 ; wait 0 cs ; [wait L sck ; in pins,1 ;
//             wait !L sck ; out pins,1]
//   SPH = 1   wait 0 cs ; [wait L sck ; out pins,1 ; wait !L sck ;
//             in pins,1]
//
// the brackets being the wrap, which costs no cycle. SPH = 0 puts the
// first bit on the line before the select falls and every later one at
// the trailing edge - the last trailing edge of a frame putting out the
// next frame's first bit, so a window of many frames runs back to back -
// and samples at the leading edge; SPH = 1 changes the line at the leading
// edge and samples at the trailing one. The frame is the shift registers'
// own count: autopull and autopush at the frame's width (4..16 in the
// program, 8 and 16 on this protocol), the bit order the shift direction
// - MSb first shifting left, LSb first shifting right. A frame written to
// the transmit FIFO as a byte or a half-word lands REPLICATED across the
// word (RP2040 2.1.4, RP2350 2.1.5: narrow writes are replicated), so its
// bits are where either direction shifts from; a received frame sits in
// the word's low bits shifting left and in its high bits shifting right,
// which is where the receive engine reads it (the FIFO's address plus 3
// for a byte, plus 2 for a half-word).
//
// THE SELECT FRAMES THE WINDOW, NOT THE FRAME. The program waits for the
// select once, at its entry, and from there follows SCK alone. So every
// window is ARMED by the loop with the select high - the machine stopped,
// restarted (its shift counters and its input register cleared), its
// output register emptied, its PC at the entry - and a window the host
// closes in mid-frame leaves nothing behind for the next one. The dark
// command listener is re-armed the same way at every rising edge of the
// select the loop sees.
//
// THE COST, AND THE CEILING IT SETS. The machine runs at clk_sys, every
// input through the two-flop synchronizer (RP2040 3.5.6.3). An answer bit
// leaves the pad some four machine cycles after the SCK edge that asks for
// it - two of synchronizer, the WAIT that sees the edge, the OUT - and the
// far host samples it half an SCK period after that edge, which is the
// client's ceiling (the suites measure it: docs/rp2040/spi.md,
// docs/rp2350/spi.md). The receive path is shorter by the OUT and is not
// the bound. Nothing of either path runs on the core: a window is the
// machine and two DMA channels (the default serve) or the core's polled
// pump feeding and draining the FIFOs (`spare_polled_pump`).
//
// WHAT THIS PEER MAKES OF THE PROTOCOL:
//
//  - EVERY WINDOW IS PRELOADED. The first answer is in the transmit FIFO
//    before the select falls, so every regime aligns rx[i] = P_B(i), the
//    table's exact row (regime_buffer_wait's on every peer; normal and
//    buffer included here - this client has no shift register left over
//    to lead).
//  - THE EXCHANGE ON THE DMA ENGINES BY DEFAULT: the transmit channel
//    pours the answers into the FIFO and the receive channel takes the
//    frames out, both at the frame's beat, no interrupt; the loop watches
//    the select and the receive count. `spare_polled_pump` asks for the
//    core's loop instead, `spare_answer_only` for the answers alone (the
//    receive side off, the report counting the answers loaded),
//    `spare_receive_only` for the frames alone with the answer line DARK.
//  - THE EXTENDED EXCHANGE AS FAR AS THE SILICON GOES: `spare_wide`
//    (16-bit frames: the thresholds at sixteen, the engines in half-word
//    beats) and `spare_receive_only` are served and raise report_extended;
//    `spare_crc` is REFUSED (report_cfg_failed, no flag) - a state machine
//    has no CRC unit to witness the host's with.
//  - flag_expect_reversed is judged as asked; flag_wrcol, flag_feed_tx and
//    flag_skip_write name conditions of a buffered shift register this
//    client has not got and are ignored. Op::mspi asks for a USART in
//    host-SPI mode, which neither chip has: report_cfg_failed.
//  - sink_slow: the receive FIFO filled and never read for the deadline;
//    the machine STALLS at the IN that finds it full (RXSTALL), so what is
//    lost is whatever the host clocked after that - count = the frames the
//    FIFO kept, sum = 1 when the stall was seen, got / exp = the machine's
//    FDEBUG bits before and after the drain, aux0..aux3 the first four.
//  - THE REPORT'S OWN FIELDS (spi_link.hpp leaves them to the peer):
//    aux0 = the machine's FDEBUG bits after the window (bit 0 RXSTALL - a
//    frame arrived on a full receive FIFO, also raised as report_bufovf -,
//    bit 1 RXUNDER, bit 2 TXOVER, bit 3 TXSTALL - the answers ran out),
//    aux1 = the milliseconds from the arming to the window's close (255 =
//    nothing came), aux2 bit 0 = the select read low once the window was
//    over, bit 1 = the host closed it, bit 2 = the DMA engines served it;
//    aux3 = 0. host_burst's: aux0 = the burst's time in tens of
//    microseconds, aux1 = the division over 4, aux3 = the host's status.
//
// THE DARK LISTENER: the answer pad belongs to the PIO at all times and
// the program drives it only through its direction, which is OUTPUT for
// one answer or exchange window after a frame that CHECKED OUT and INPUT
// otherwise, the pad itself carrying no pull (the far host's own pull-up
// holds the wire). An unknown op is dropped in silence, a bad checksum
// nak'ed only while ENGAGED.
//
// THE ROLES INVERT for Op::host_burst: the four pads go to the PL022 as a
// HOST whose select is the block's OWN FRAME SIGNAL on the CS pad
// (SpiHost's `frame_select`) - pulsed between frames in modes 0 and 2,
// held through the burst in modes 1 and 3 (RP2040 4.4.3.10 to 4.4.3.13) -
// which is what a PL022 client needs in the modes it frames on its select.
// The burst is ONE polled request of `count` frames back to back at the
// commanded division of clk_peri (spi_link's aux16: 4..128, 0 = 32), 8- or
// 16-bit (spare_wide), MSb first - the PL022 has no other order, so dord
// LSb is refused. ss_pulse drives the CS pad low from SIO for its
// microseconds.
//
// COEXISTENCE: in command mode the machine reads its three inputs and
// drives nothing; a select that never falls never even starts the program.

#pragma once

#include <stdint.h>

#include <span>

#include "kernel/borrowed.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrdx/src/apps/spi_link.hpp"

using spilink::Op;

constexpr uint16_t firmware_version = static_cast<uint16_t>((peer_kind << 8) | 0x01u);   ///< Ident

bool trace = false;
bool client_up = false;
/// The console's 'b': the four pads' input synchronizers bypassed (RP2040
/// 3.5.6.3, INPUT_SYNC_BYPASS) - an experiment on the ceiling, off by
/// default.
bool sync_bypassed = false;

spilink::Decoder decoder;
spilink::Report last_report;

uint32_t commands = 0, actions = 0, naks = 0, serves = 0, resyncs = 0;
uint32_t last_byte_us = 0;
/// Engaged until this microsecond (0 = never addressed); the distance is
/// read signed, so the ruler's wrap is harmless for spi_link's 4 s.
uint32_t engaged_until_us = 0;
bool is_engaged = false;

bool engaged() { return is_engaged && static_cast<int32_t>(engaged_until_us - us_now()) > 0; }
void engage() {
    is_engaged = true;
    engaged_until_us = us_now() + static_cast<uint32_t>(spilink::engage_ms) * 1000u;
}

// ---- waits ------------------------------------------------------------------------

void hold_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}
bool expired(uint32_t t0_us, uint32_t ms) { return us_now() - t0_us >= ms * 1000u; }

/// The select is high (the host has closed its window), within `ms`.
bool wait_cs_high(uint32_t ms) {
    const uint32_t t0 = us_now();
    while (!CsPin::read()) {
        if (expired(t0, ms)) {
            return false;
        }
    }
    return true;
}

// ---- the programs ---------------------------------------------------------------------

constexpr uint8_t sck_i = pio_index(sck_gpio);
constexpr uint8_t rx_i = pio_index(rx_gpio);
constexpr uint8_t tx_i = pio_index(tx_gpio);
constexpr uint8_t cs_i = pio_index(cs_gpio);
static_assert(sck_i < 32u && rx_i < 32u && tx_i < 32u && cs_i < 32u, "the four pads inside the PIO's window");

/// SPH = 0 (modes 0 and 2): the header's six instructions.
constexpr PioProgram<6> sph0_program(bool cpol) {
    const bool lead = !cpol;
    PioProgram<6> p{};
    p.code = {pio_out(PioOut::pins, 1),
              pio_wait(false, PioWaitOn::gpio, cs_i),
              pio_wait(lead, PioWaitOn::gpio, sck_i),
              pio_in(PioIn::pins, 1),
              pio_wait(!lead, PioWaitOn::gpio, sck_i),
              pio_out(PioOut::pins, 1)};
    p.wrap_bottom = 2;
    p.wrap_top = 5;
    return p;
}
/// SPH = 1 (modes 1 and 3): the header's five.
constexpr PioProgram<5> sph1_program(bool cpol) {
    const bool lead = !cpol;
    PioProgram<5> p{};
    p.code = {pio_wait(false, PioWaitOn::gpio, cs_i),
              pio_wait(lead, PioWaitOn::gpio, sck_i),
              pio_out(PioOut::pins, 1),
              pio_wait(!lead, PioWaitOn::gpio, sck_i),
              pio_in(PioIn::pins, 1)};
    p.wrap_bottom = 1;
    p.wrap_top = 4;
    return p;
}
constexpr PioProgram<6> mode0_program = sph0_program(false);
constexpr PioProgram<5> mode1_program = sph1_program(false);
constexpr PioProgram<6> mode2_program = sph0_program(true);
constexpr PioProgram<5> mode3_program = sph1_program(true);
/// Where each mode's program sits: side by side from slot 0.
constexpr uint8_t mode_offset[4] = {0, 6, 11, 17};

bool programs_loaded = false;

bool load_programs() {
    (void)Blk::reset();
    open_pio_window();
    programs_loaded = Blk::load(mode0_program, mode_offset[0]) && Blk::load(mode1_program, mode_offset[1]) &&
                      Blk::load(mode2_program, mode_offset[2]) && Blk::load(mode3_program, mode_offset[3]);
    return programs_loaded;
}

// ---- the client's window -------------------------------------------------------------

/// What a window is: the mode, the order, the frame's width, and which of
/// the two directions run.
struct Window {
    uint8_t mode = 0;
    bool lsb = false;
    uint8_t bits = 8;
    bool answer = false;   ///< the answer line driven, the transmit FIFO fed
    bool receive = true;   ///< the receive FIFO taken
};

/// The machine's four registers for a window, the wrap points the
/// program's at its offset.
PioSmConfig window_config(const Window& w, bool autopull) {
    PioSmConfig c{};
    c.clock = PioClockDiv{1, 0};
    const uint8_t at = mode_offset[w.mode & 3u];
    const bool sph0 = (w.mode & 1u) == 0u;
    c.wrap_bottom = static_cast<uint8_t>(at + (sph0 ? 2u : 1u));
    c.wrap_top = static_cast<uint8_t>(at + (sph0 ? 5u : 4u));
    c.out_base = tx_i;
    c.out_count = 1;
    c.set_base = tx_i;
    c.set_count = 1;
    c.in_base = rx_i;
    c.out_shift_right = w.lsb;
    c.in_shift_right = w.lsb;
    c.pull_threshold = w.bits;
    c.push_threshold = w.bits;
    c.autopull = autopull;
    c.autopush = w.receive;
    // One direction alone takes both FIFOs' storage: eight frames deep.
    c.fifo_join = !w.answer ? PioFifoJoin::rx : (!w.receive ? PioFifoJoin::tx : PioFifoJoin::none);
    return c;
}

/// The pads to the PIO, the answer line's direction set first.
void pads_to_pio(bool drive) {
    Sm::pin_directions(tx_i, 1, drive);
    (void)SckPin::function(pio_function, {});
    (void)RxPin::function(pio_function, {});
    (void)CsPin::function(pio_function, {});
    (void)TxPin::function(pio_function, {});
}

/// ARM a window with the select high: the machine stopped and restarted
/// at its mode's entry, the output register EMPTIED - a restart clears the
/// shift counters, which leaves the register counting as full, so one OUT
/// NULL, 32 with autopull off empties it and the first OUT of the window
/// pulls - and only then autopull as the window asks. The FIFOs are left
/// empty; the caller fills the transmit side and enables the machine.
bool arm(const Window& w) {
    Sm::enable(false);
    PioSmConfig c = window_config(w, false);
    bool ok = false;
    switch (w.mode & 3u) {
        case 0: ok = Sm::init(mode0_program, mode_offset[0], c); break;
        case 1: ok = Sm::init(mode1_program, mode_offset[1], c); break;
        case 2: ok = Sm::init(mode2_program, mode_offset[2], c); break;
        default: ok = Sm::init(mode3_program, mode_offset[3], c); break;
    }
    if (!ok) {
        return false;
    }
    (void)Sm::exec_wait(pio_out(PioOut::null, 32));
    if (w.answer) {
        c = window_config(w, true);
        ok = Sm::configure(c);
    }
    Sm::clear_fifo_debug();
    Sm::pin_directions(tx_i, 1, w.answer);
    return ok;
}

/// Back to the entry with nothing else changed: the command listener's
/// resynchronization at a rising select.
void resync() {
    Sm::enable(false);
    Sm::restart();
    (void)Sm::exec(pio_jmp(PioJmp::always, mode_offset[0]));
    Sm::enable(true);
    ++resyncs;
}

/// COMMAND MODE: mode 0, MSb first, 8-bit frames, the receive FIFO alone
/// (eight deep), the answer line DARK. Everything returns here.
bool go_dark() {
    (void)wait_cs_high(50);
    const bool ok = arm({.mode = 0, .lsb = false, .bits = 8, .answer = false, .receive = true});
    pads_to_pio(false);
    Sm::enable(true);
    decoder.reset();
    return ok;
}

/// The select's window, from the arming: true once it has been low and is
/// high again, false at the deadline. `saw_low` says whether it ever fell.
bool window_closed(uint32_t t0, uint32_t ms, bool& saw_low) {
    for (;;) {
        const bool low = !CsPin::read();
        if (low) {
            saw_low = true;
        } else if (saw_low) {
            return true;
        }
        if (expired(t0, ms)) {
            return false;
        }
    }
}

/// A frame as the transmit FIFO wants it pushed by the core: replicated,
/// so it is where either shift direction takes it from - as a narrow DMA
/// write lands.
uint32_t replicated(uint16_t f, uint8_t bits) {
    return bits <= 8u ? static_cast<uint32_t>(f & 0xFFu) * 0x01010101u : static_cast<uint32_t>(f) * 0x00010001u;
}
/// A popped word's frame: in the low bits shifting left, the high bits
/// shifting right.
uint16_t frame_of(uint32_t word, const Window& w) {
    const uint32_t mask = (1u << w.bits) - 1u;
    return static_cast<uint16_t>((w.lsb ? word >> (32u - w.bits) : word) & mask);
}

/// The machine's FDEBUG bits, packed (the header's aux0).
uint8_t fdebug_bits() {
    return static_cast<uint8_t>((Sm::rx_stalled() ? 1u : 0u) | (Sm::rx_underflowed() ? 2u : 0u) |
                                (Sm::tx_overflowed() ? 4u : 0u) | (Sm::tx_stalled() ? 8u : 0u));
}

// ---- answering ------------------------------------------------------------------------

/// Serve ONE answer window: the frame padded with zeros to the most the
/// host clocks, pushed by the core - the answer-only window's transmit
/// FIFO is eight deep - the window ending when the host raises the select.
void serve(const uint8_t* buf, uint8_t n) {
    (void)wait_cs_high(50);   // the command window must close first
    uint8_t padded[spilink::answer_bytes] = {};
    for (uint8_t i = 0; i < n && i < spilink::answer_bytes; ++i) {
        padded[i] = buf[i];
    }
    const Window w{.mode = 0, .lsb = false, .bits = 8, .answer = true, .receive = false};
    if (!arm(w)) {
        (void)go_dark();
        return;
    }
    uint16_t sent = 0;
    while (sent < spilink::answer_bytes && !Sm::tx_full()) {
        Sm::push(replicated(padded[sent++], 8));
    }
    Sm::enable(true);
    const uint32_t t0 = us_now();
    bool saw_low = false;
    for (;;) {
        while (sent < spilink::answer_bytes && !Sm::tx_full()) {
            Sm::push(replicated(padded[sent++], 8));
        }
        const bool low = !CsPin::read();
        if (low) {
            saw_low = true;
        } else if (saw_low) {
            break;
        }
        if (expired(t0, spilink::serve_ms)) {
            break;
        }
    }
    ++serves;
    (void)go_dark();
}

void answer(Op op, const uint8_t* p, uint8_t len) {
    uint8_t buf[spilink::max_payload + 4];
    uint8_t n = 0;
    spilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof buf) {
                buf[n++] = b;
            }
        },
        op, p, len);
    serve(buf, n);
}

void ack(Op op, uint8_t sum, bool good) {
    const uint8_t p[2] = {spilink::byte_of(op), sum};
    answer(good ? Op::ack : Op::nak, p, 2);
}

// ---- the exchange ---------------------------------------------------------------------

constexpr uint16_t exchange_cap = 256;
alignas(4) uint16_t x_out[exchange_cap];
alignas(4) uint16_t x_in[exchange_cap];
uint8_t* out_bytes() { return reinterpret_cast<uint8_t*>(x_out); }
uint8_t* in_bytes() { return reinterpret_cast<uint8_t*>(x_in); }

/// Judge `frames` received frames against the host's stream - their BYTES
/// in memory order (spi_link.hpp's extended accounting).
void judge(spilink::Report& r, const spilink::Params& a, uint16_t frames, bool wide) {
    const bool reversed = (a.flags & spilink::flag_expect_reversed) != 0;
    const uint8_t* const got = in_bytes();
    const uint16_t bytes = static_cast<uint16_t>(wide ? frames * 2u : frames);
    for (uint16_t i = 0; i < bytes; ++i) {
        const uint8_t want = spilink::wide_byte(a.pattern, a.seed_a, i, wide, reversed);
        if (got[i] != want) {
            if (r.mism == 0) {
                r.idx = static_cast<uint8_t>(i < 255u ? i : 255u);
                r.got = got[i];
                r.exp = want;
            }
            ++r.mism;
        }
        r.sum = static_cast<uint16_t>(r.sum + got[i]);
    }
    r.count = frames;
}

/// THE DMA SERVE: both engines bound to the machine's FIFOs at the frame's
/// beat, the receive one at the FIFO's address plus the offset where a
/// right-shifted frame sits; the transmit run started first, so the FIFO
/// holds the first answers when the machine starts. Returns the frames in.
template <typename T>
uint16_t dma_serve(const Window& w, uint16_t n, uint32_t ms, bool& closed) {
    if (w.receive) {
        volatile uint8_t* const at = static_cast<volatile uint8_t*>(Sm::rx_address()) +
                                     (w.lsb ? 4u - sizeof(T) : 0u);
        PeerRx::arm(at, Sm::dreq_rx, true, DmaReport::errors);
        (void)PeerRx::start(std::span<T>(reinterpret_cast<T*>(x_in), n));
    }
    if (w.answer) {
        PeerTx::arm(Sm::tx_address(), Sm::dreq_tx, false, DmaReport::errors);
        (void)PeerTx::start(std::span<const T>(reinterpret_cast<const T*>(x_out), n));
    }
    // The transmit FIFO full before the machine runs: the preload.
    const uint32_t t_fill = us_now();
    while (w.answer && Sm::tx_level() < (n < 4u ? n : 4u) && us_now() - t_fill < 100u) {
    }
    Sm::enable(true);
    const uint32_t t0 = us_now();
    bool saw_low = false;
    for (;;) {
        if (w.receive) {
            (void)PeerRx::take();
            if (PeerRx::full()) {
                break;
            }
        }
        const bool low = !CsPin::read();
        if (low) {
            saw_low = true;
        } else if (saw_low) {
            closed = true;
            break;
        }
        if (expired(t0, ms)) {
            break;
        }
    }
    if (w.receive) {
        (void)PeerRx::take();
    }
    const uint16_t got = w.receive ? static_cast<uint16_t>(PeerRx::taken()) : 0u;
    PeerTx::stop();
    PeerRx::stop();
    return got;
}

/// THE POLLED SERVE: the core feeds and drains the FIFOs itself.
uint16_t polled_serve(const Window& w, uint16_t n, uint32_t ms, bool& closed) {
    const bool wide = w.bits > 8u;
    const uint8_t* const o8 = out_bytes();
    uint8_t* const i8 = in_bytes();
    uint16_t sent = 0;
    uint16_t got = 0;
    auto out_frame = [&](uint16_t k) -> uint16_t {
        return wide ? x_out[k] : o8[k];
    };
    while (w.answer && sent < n && !Sm::tx_full()) {
        Sm::push(replicated(out_frame(sent++), w.bits));
    }
    Sm::enable(true);
    const uint32_t t0 = us_now();
    bool saw_low = false;
    for (;;) {
        while (w.answer && sent < n && !Sm::tx_full()) {
            Sm::push(replicated(out_frame(sent++), w.bits));
        }
        while (w.receive && !Sm::rx_empty()) {
            const uint16_t f = frame_of(Sm::pop(), w);
            if (got < n) {
                if (wide) {
                    x_in[got] = f;
                } else {
                    i8[got] = static_cast<uint8_t>(f);
                }
                ++got;
            }
        }
        if (w.receive && got >= n) {
            break;
        }
        const bool low = !CsPin::read();
        if (low) {
            saw_low = true;
        } else if (saw_low) {
            closed = true;
            break;
        }
        if (expired(t0, ms)) {
            break;
        }
    }
    return w.receive ? got : sent;
}

/// The workhorse: the host sends P_A(i), this client answers P_B(i), in
/// ONE window of `count` frames under the host's held select.
spilink::Report run_exchange(const spilink::Params& a) {
    spilink::Report r{};
    const bool wide = (a.spare & spilink::spare_wide) != 0;
    const bool crc = (a.spare & spilink::spare_crc) != 0;
    const bool receive_only = (a.spare & spilink::spare_receive_only) != 0;
    const bool answer_only = !receive_only && (a.spare & spilink::spare_answer_only) != 0;
    const bool polled = (a.spare & spilink::spare_polled_pump) != 0;
    const uint16_t n = a.count < exchange_cap ? a.count : exchange_cap;
    if (n == 0u || crc) {
        // A state machine has no CRC unit to witness the host's with.
        r.flags |= spilink::report_cfg_failed;
        (void)go_dark();
        return r;
    }
    r.flags |= spilink::report_extended;
    {
        spilink::Stream out(a.pattern, a.seed_b);
        uint8_t* const o = out_bytes();
        const uint16_t bytes = static_cast<uint16_t>(wide ? 2u * n : n);
        for (uint16_t i = 0; i < bytes; ++i) {
            o[i] = out.next();
        }
        uint8_t* const in = in_bytes();
        for (uint16_t i = 0; i < sizeof x_in; ++i) {
            in[i] = 0;
        }
    }
    (void)wait_cs_high(50);
    const Window w{.mode = static_cast<uint8_t>(a.cfg.mode & 3u),
                   .lsb = a.cfg.dord != 0,
                   .bits = static_cast<uint8_t>(wide ? 16u : 8u),
                   .answer = !receive_only,
                   .receive = !answer_only};
    if (!arm(w)) {
        r.flags |= spilink::report_cfg_failed;
        (void)go_dark();
        return r;
    }
    const uint32_t t0 = us_now();
    bool closed = false;
    uint16_t got = 0;
    if (polled) {
        got = polled_serve(w, n, a.ms, closed);
    } else {
        got = wide ? dma_serve<uint16_t>(w, n, a.ms, closed) : dma_serve<uint8_t>(w, n, a.ms, closed);
        r.aux2 |= 0x04;
    }
    if (!closed) {
        bool saw_low = true;
        closed = window_closed(us_now(), 20, saw_low);
    }
    const uint32_t dt = (us_now() - t0) / 1000u;
    Sm::enable(false);
    r.aux0 = fdebug_bits();
    if (Sm::rx_stalled()) {
        r.flags |= spilink::report_bufovf;
    }
    r.aux1 = got == 0u && !answer_only ? uint8_t{255} : (dt > 254u ? uint8_t{254} : static_cast<uint8_t>(dt));
    if (!CsPin::read()) {
        r.aux2 |= 0x01;
    }
    if (closed) {
        r.aux2 |= 0x02;
    }
    if (answer_only) {
        r.count = polled ? got : n;   // the answers loaded; nothing was received
    } else {
        judge(r, a, got, wide);
    }
    if (r.count < a.count) {
        r.flags |= spilink::report_timed_out;
    }
    (void)go_dark();
    return r;
}

/// The loss semantics: the receive FIFO (four deep, not joined) never read
/// for the deadline.
spilink::Report run_sink_slow(const spilink::Params& a) {
    spilink::Report r{};
    (void)wait_cs_high(50);
    const Window w{.mode = static_cast<uint8_t>(a.cfg.mode & 3u), .lsb = a.cfg.dord != 0, .bits = 8,
                   .answer = true, .receive = true};
    if (!arm(w)) {
        r.flags |= spilink::report_cfg_failed;
        (void)go_dark();
        return r;
    }
    Sm::pin_directions(tx_i, 1, false);   // nothing to answer with: the line dark
    Sm::enable(true);
    const uint32_t t0 = us_now();
    while (!expired(t0, a.ms)) {
        if (Sm::rx_stalled()) {
            r.sum = 1;
        }
    }
    r.flags |= spilink::report_timed_out;   // by construction: the deadline IS the exit
    if (Sm::rx_stalled()) {
        r.sum = 1;
        r.flags |= spilink::report_bufovf;
    }
    r.got = fdebug_bits();
    Sm::enable(false);
    uint8_t vals[4] = {};
    uint8_t kept = 0;
    while (!Sm::rx_empty() && kept < 8u) {
        const uint8_t v = static_cast<uint8_t>(frame_of(Sm::pop(), w));
        if (kept < 4u) {
            vals[kept] = v;
        }
        ++kept;
    }
    r.exp = fdebug_bits();
    r.count = kept;
    r.aux0 = vals[0];
    r.aux1 = vals[1];
    r.aux2 = vals[2];
    r.aux3 = vals[3];
    (void)go_dark();
    return r;
}

/// The second driver on the shared select wire, from SIO.
spilink::Report run_ss_pulse(const spilink::Params& a) {
    spilink::Report r{};
    Sm::enable(false);
    if (a.aux8 != 0u) {
        hold_us(static_cast<uint32_t>(a.aux8) * 1000u);
    }
    (void)CsPin::output(false);
    hold_us(a.aux16 != 0u ? a.aux16 : 1000u);
    (void)CsPin::input(PinPull::none);
    r.aux0 = 1;
    r.count = 1;
    (void)go_dark();
    return r;
}

/// THE ROLES INVERT: the PL022 hosts one burst of `count` frames back to
/// back on the four pads, its own frame signal the select.
spilink::Report run_host_burst(const spilink::Params& a) {
    spilink::Report r{};
    const bool wide = (a.spare & spilink::spare_wide) != 0;
    const uint16_t cap = wide ? exchange_cap / 2u : exchange_cap;
    const uint16_t n = a.count < cap ? a.count : cap;
    uint32_t div = a.aux16 != 0u ? a.aux16 : 32u;
    if (div < 2u) {
        div = 2u;
    }
    div &= ~1u;
    const SpiClock rate{2, static_cast<uint8_t>(div / 2u - 1u)};
    Sm::enable(false);
    Sm::pin_directions(tx_i, 1, false);
    // The PL022 is MSb first and nothing else.
    if (a.cfg.dord != 0 || n == 0u || div > 512u || !Host::init(clock)) {
        r.flags |= spilink::report_cfg_failed;
        Host::release();
        (void)go_dark();
        return r;
    }
    r.aux1 = static_cast<uint8_t>(div >> 2);   // /4 to fit a byte, as on every peer
    const SpiMode mode = static_cast<SpiMode>(a.cfg.mode & 3u);
    const SpiDataSize bits = wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
    // The mode primed before the burst: a CPOL flip is an edge on the wire.
    Host::prime(mode, rate, bits);
    {
        spilink::Stream out(a.pattern, a.seed_a);
        uint8_t* const o = out_bytes();
        const uint16_t bytes = static_cast<uint16_t>(wide ? 2u * n : n);
        for (uint16_t i = 0; i < bytes; ++i) {
            o[i] = out.next();
        }
        uint8_t* const in = in_bytes();
        for (uint16_t i = 0; i < sizeof x_in; ++i) {
            in[i] = 0xEE;
        }
    }
    hold_us(static_cast<uint32_t>(a.aux8 != 0u ? a.aux8 : 20u) * 1000u);
    Host::Request q{};
    q.cs = {};   // the frame signal is the select
    q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out_bytes()));
    q.rx = lend<Lease::reply>(in_bytes());
    q.len = n;
    q.clock = rate;
    q.mode = mode;
    q.bits = bits;
    q.polled = true;
    const uint32_t t0 = us_now();
    (void)Host::start(q);
    const uint32_t took = us_now() - t0;
    r.aux3 = Host::status();
    {
        spilink::Stream want(a.pattern, a.seed_b);
        const uint8_t* const in = in_bytes();
        const uint16_t bytes = static_cast<uint16_t>(wide ? 2u * n : n);
        for (uint16_t i = 0; i < bytes; ++i) {
            const uint8_t e = want.next();
            if (in[i] != e) {
                if (r.mism == 0u) {
                    r.idx = static_cast<uint8_t>(i < 255u ? i : 255u);
                    r.got = in[i];
                    r.exp = e;
                }
                ++r.mism;
            }
            r.sum = static_cast<uint16_t>(r.sum + in[i]);
        }
        r.count = Host::status() == spi_ok ? n : 0u;
    }
    r.aux0 = static_cast<uint8_t>(took > 255u * 10u ? 255u : took / 10u);   // tens of microseconds
    if (r.count < a.count) {
        r.flags |= spilink::report_timed_out;
    }
    Host::release();
    (void)go_dark();
    return r;
}

// ---- the command handler ------------------------------------------------------------------

void handle(const spilink::Frame& f) {
    const Op op = f.op;
    ++commands;
    engage();
    if (trace) {
        print(console, "  [cmd op=", hex(spilink::byte_of(op)), " len=", f.len, "]", crlf);
    }
    if (op == Op::ping) {
        ack(op, f.sum, true);
        return;
    }
    if (op == Op::ident) {
        ack(op, f.sum, true);
        spilink::Ident d{};
        for (uint8_t i = 0; i < 8u; ++i) {
            d.label[i] = peer_label[i];
        }
        d.xtal = xtal_ok ? 1u : 0u;
        d.sanity = spilink::ident_sanity;
        d.version = firmware_version;
        uint8_t p[spilink::ident_size];
        spilink::put_ident(p, d);
        answer(Op::ident_data, p, spilink::ident_size);
        return;
    }
    if (op == Op::report) {
        ack(op, f.sum, true);
        uint8_t p[spilink::report_size];
        spilink::put_report(p, last_report);
        answer(Op::report_data, p, spilink::report_size);
        return;
    }
    if (f.len < spilink::params_size) {
        ack(op, f.sum, false);
        ++naks;
        return;
    }
    ack(op, f.sum, true);
    ++actions;
    const spilink::Params a = spilink::get_params(f.data);
    const uint32_t started = us_now();
    spilink::Report r{};
    switch (op) {
        case Op::exchange: r = run_exchange(a); break;
        case Op::sink_slow: r = run_sink_slow(a); break;
        case Op::ss_pulse: r = run_ss_pulse(a); break;
        case Op::host_burst: r = run_host_burst(a); break;
        // Op::mspi: no USART of either chip has a host-SPI mode.
        default: r.flags |= spilink::report_cfg_failed; break;
    }
    if ((r.flags & spilink::report_cfg_failed) == 0) {
        r.flags |= spilink::report_ran;
    }
    const uint32_t took = (us_now() - started) / 1000u;
    r.ms = took > 255u ? uint8_t{255} : static_cast<uint8_t>(took);
    r.op = spilink::byte_of(op);
    last_report = r;
    engage();
    (void)go_dark();
}

// ---- the console ----------------------------------------------------------------------------

void help() {
    print(console, "spi_peer: ? help | i status | 0 back to the dark client | 3 trace | b the input "
                   "synchronizers bypassed or not (an experiment)", crlf);
}

void status() {
    print(console, "  ", link_text, ": client ", client_up ? "up" : "OFF", ", machine at ", Sm::address(),
          ", FIFO levels tx ", Sm::tx_level(), " rx ", Sm::rx_level(), ", FDEBUG bits ", hex(fdebug_bits()),
          ", synchronizers ", sync_bypassed ? "BYPASSED" : "on", crlf);
    print(console, "  CS reads ", CsPin::read() ? "high" : "LOW", ", SCK ", SckPin::read() ? "high" : "low",
          ", the answer line ", TxPin::status().oe_to_pad ? "DRIVEN" : "dark", crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks, " served=", serves,
          " resyncs=", resyncs, engaged() ? "  (ENGAGED)" : "  (dark, mute)", crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count,
          " mism=", last_report.mism, " sum=", hex(last_report.sum), " flags=", hex(last_report.flags),
          " idx=", last_report.idx, " got=", hex(last_report.got), " exp=", hex(last_report.exp), crlf);
    print(console, "               aux=", hex(last_report.aux0), " ", hex(last_report.aux1), " ",
          hex(last_report.aux2), " ", hex(last_report.aux3), " ms=", last_report.ms, crlf);
}

/// The DMA line: both engines are bound to report a bus error alone, so
/// anything here is cleared and counted, never dispatched.
volatile uint32_t dma_vectors = 0;
[[gnu::always_inline]] inline void peer_dma_vector() {
    dma_vectors = dma_vectors + 1u;
    (void)PeerTx::service();
    (void)PeerRx::service();
}

/// Bring the client up: the programs, the dark listener.
bool bring_up() {
    client_up = load_programs() && go_dark();
    return client_up;
}

/// The loop: the console, then - with the link pads free - the command
/// listener: every frame the machine pushed fed to the decoder, and a
/// rising select re-arming the machine at its entry.
[[noreturn]] void peer_loop() {
    uint32_t asked = us_now();
    bool cs_was_low = false;
    for (;;) {
        uint8_t c = 0;
        if (Console::read_byte(c) && c != '\r' && c != '\n') {
            print(console, static_cast<char>(c), crlf);
            if (c == '?') {
                help();
            } else if (c == 'i') {
                status();
            } else if (c == '0') {
                is_engaged = false;
                if (link_pads_free()) {
                    (void)bring_up();
                }
                print(console, client_up ? "  dark client restored, engagement dropped" : "  the client stays off",
                      crlf);
            } else if (c == '3') {
                trace = !trace;
                print(console, trace ? "  trace on" : "  trace off", crlf);
            } else if (c == 'b') {
                sync_bypassed = !sync_bypassed;
                const uint32_t mask = (1u << sck_i) | (1u << rx_i) | (1u << cs_i);
                Blk::sync_bypass(sync_bypassed ? mask : 0u);
                print(console, sync_bypassed ? "  SCK, RX and CS synchronizers BYPASSED" : "  synchronizers on",
                      crlf);
            } else {
                print(console, "? for help", crlf);
            }
            print(console, "> ");
        }
        if (!client_up) {
            if (us_now() - asked >= 1'000'000u) {
                asked = us_now();
                if (link_pads_free() && bring_up()) {
                    print(console, crlf, "  the link pads are free: the dark client is up", crlf, "> ");
                }
            }
            continue;
        }
        if (decoder.partial() && us_now() - last_byte_us > 100'000u) {
            decoder.reset();
        }
        bool handled = false;
        while (!handled && !Sm::rx_empty()) {
            const uint8_t b = static_cast<uint8_t>(Sm::pop() & 0xFFu);
            last_byte_us = us_now();
            switch (decoder.feed(b)) {
                case spilink::Decoder::Result::frame:
                    if (spilink::is_command(decoder.frame().op)) {
                        handle(decoder.frame());
                        handled = true;
                    }
                    break;
                case spilink::Decoder::Result::bad_checksum:
                    if (spilink::is_command(decoder.pending_op()) && engaged()) {
                        ack(decoder.pending_op(), decoder.frame().sum, false);
                        ++naks;
                        handled = true;
                    }
                    break;
                default: break;
            }
        }
        if (handled) {
            cs_was_low = false;
            continue;
        }
        const bool low = !CsPin::read();
        if (cs_was_low && !low && Sm::rx_empty()) {
            resync();
        }
        cs_was_low = low;
    }
}
