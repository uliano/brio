// rp_spi_pair.hpp - THE TWO-BOARD SPI LETTERS OF THE TWO RASPBERRY PI CHIPS,
// written once over the ARM PL022 (brio/pl022/spi.hpp) and included by
// test_rp2040_spi and test_rp2350_spi: this chip's SPI1 the DUT - the host
// on its link pads, and for letters r and m the CLIENT - the desk partner
// running `spi_peer` (rp_spi_peer.hpp: a PIO client, the roles inverted on
// its PL022) commanded IN BAND over the very bus under test through
// avrdx/src/apps/spi_link.hpp - the RP2040 against the RP2350's peer, the
// RP2350 (both halves) against the RP2040's.
//
// ONE BODY, TWO CHIPS: what is a chip's - the pads, the clock, the ruler,
// the platform, the partner's name, kind and clock - the including suite
// states before it includes this file, inside its own anonymous namespace
// with `using namespace brio` in force, having included kernel/,
// util/spi_bus.hpp, util/power.hpp, util/testbench.hpp and spi_link.hpp at
// file scope:
//
//   SysClock, clock, P, Serial, serial, bench
//   LinkHost, LinkDmaHost    SpiHost<1, {sck, tx, rx}>: the pump and the
//                            polled loop; the same with half-word engines on
//                            DMA channels 6 and 7, reporting on DMA_IRQ_1
//   LinkClient               SpiClient<1, {sck, tx, rx, cs}>
//   SckPin, TxPin, RxPin,    the four link pads; CsPin is this end's GPIO
//   CsPin                    select while it hosts, its client's select pad
//   us_now()                 the microsecond ruler
//   partner_kind,            spi_link's kind byte of the peer, its name, and
//   partner_text,            its clk_sys - what the peer's PIO client runs
//   partner_sys_hz           at, which sets the ceiling letter p judges to
//   CountBlk, CountSm,       a PIO block and machine the letters may take, the
//   count_cs_index,          select pad as that block numbers it, and what
//   open_count_window()      the block needs to see it (the window counter)
//   client_pad_follows_select  this chip's nSSPOE reaches the client's
//                            answer pad (the RP2350's) or not (the RP2040's),
//                            which letter m judges
//
// and binds isr_spi1 to `pair_spi_isr()` and isr_dma_1 to
// `pair_dma_isr()`.
//
// THE LINK'S RULES (docs/boards/weact-rp2040.md, weact-rp2350b.md): the
// select carries 1.5 kOhm to the RP2040's 3V3; the partner's peer is DARK
// in command mode - its answer line an input with no pull, so an undriven
// wire reads this end's own pull-up, the host receive pad's; nothing here
// drives a link pad but SPI1's four, and every letter gives them back.
//
// THE CLIENT'S CEILING (letter p). The peer's answer bit leaves its pad
// about four of ITS clk_sys cycles after the SCK edge that asks for it (two
// of input synchronizer, the WAIT that sees the edge, the OUT -
// rp_spi_peer.hpp), and this host samples it half an SCK period after that
// edge. With 15 ns for the two pads and the wire (each pad's 4 mA output
// into some 15 pF of wire and pad, RP2040 5.5.3), the half period must be
// at least 4 / partner_sys_hz + 15 ns: 47 ns against the RP2040's 125 MHz
// (10.6 MHz), 41.7 ns against the RP2350's 150 MHz (12 MHz). The letter
// judges every rung of the ladder whose half period clears that sum and
// prints the rest - where the wire really breaks is a finding, not a bound.

#pragma once

using spilink::Op;

/// The command channel's SCK: clk_peri / 128 - 977 kHz on the RP2040, 1.17
/// MHz on the RP2350, far under any ceiling of the peer's client.
constexpr SpiClock link_clock = SpiClocks::div128;
/// A frame that never answered: the suite's own code, not a wire one.
constexpr uint8_t no_answer = 0xFE;

// ---- the vector's owner --------------------------------------------------------------

enum class PairOwner : uint8_t { none, host, dma_host, bus_ao, client };
volatile PairOwner pair_owner = PairOwner::none;
volatile bool pair_done = false;
volatile uint8_t pair_status = 0;
volatile uint32_t pair_isr_entries = 0;
/// Letter w's wedge: the pump's ISR body runs and its completion is NOT
/// posted to the arbiter - the lost completion the per-bus timeout answers.
volatile bool pair_swallow = false;

// ---- shared waits ----------------------------------------------------------------------

/// Spin until `done()` or `us` microseconds have passed; true when done.
template <typename F>
bool wait_until(F done, uint32_t us) {
    const uint32_t t0 = us_now();
    while (!done()) {
        if (us_now() - t0 >= us) {
            return done();
        }
    }
    return true;
}
void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}
void settle_ms(uint32_t ms) { spin_us(ms * 1000u); }

// ---- the pads ----------------------------------------------------------------------------

/// Every link pad back to its reset state (the select held high by its
/// pull-up meanwhile): how each letter leaves the wire.
void pair_release() {
    pair_owner = PairOwner::none;
    LinkHost::release();
    LinkDmaHost::release();
    LinkClient::release();
    (void)CsPin::release();
}

/// SPI1 as the command channel's host, the CS pad its GPIO select.
bool link_command_mode() {
    pair_owner = PairOwner::none;
    LinkClient::release();
    const bool ok = LinkHost::init(clock);
    (void)CsPin::output(true);
    pair_owner = PairOwner::host;
    return ok;
}
bool link_dma_mode() {
    pair_owner = PairOwner::none;
    const bool ok = LinkDmaHost::init(clock);
    (void)CsPin::output(true);
    pair_owner = PairOwner::dma_host;
    return ok;
}

/// The hold around each select window: the protocol owns the select for
/// these windows (the Request carries a null PinRef) and pays 30 us on each
/// side, for the client's loop to see the edge.
void link_hold() { spin_us(30); }

/// One request through a host, polled; the status or no_answer.
template <typename H>
uint8_t polled(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate, SpiMode mode, SpiDataSize bits) {
    typename H::Request r{};
    r.cs = {};
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = n;
    r.clock = rate;
    r.mode = mode;
    r.bits = bits;
    r.polled = true;
    return H::start(r) ? H::status() : no_answer;
}

/// One protocol window: prime, select, hold, the transaction, hold,
/// deselect. THE MODE IS PRIMED BEFORE THE SELECT FALLS - a CPOL flip in an
/// open window is an edge the selected client counts.
template <typename H = LinkHost>
uint8_t link_xfer(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate = link_clock,
                  SpiMode mode = SpiMode::mode0, SpiDataSize bits = SpiDataSize::bits8) {
    if (n == 0u) {
        return spi_ok;
    }
    H::prime(mode, rate, bits);
    spin_us(10);
    CsPin::clear();
    link_hold();
    const uint8_t st = polled<H>(tx, rx, n, rate, mode, bits);
    link_hold();
    CsPin::set();
    return st;
}

// ---- the command channel ------------------------------------------------------------------

uint8_t frame_buf[spilink::max_payload + 8];
uint8_t answer_buf[spilink::answer_bytes];
uint8_t raw_seen[16];
uint8_t raw_n = 0;
spilink::Decoder dec;
bool link_quiet = false;

void send_frame(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    spilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof frame_buf) {
                frame_buf[n++] = b;
            }
        },
        op, p, len);
    (void)link_xfer(frame_buf, nullptr, n);
}

/// One answer window: `answer_bytes` dummies, then the decoder walks what
/// came back (the peer pads its frame with zeros).
bool recv_frame(spilink::Frame& out) {
    dec.reset();
    raw_n = 0;
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        answer_buf[i] = 0xEE;
    }
    (void)link_xfer(nullptr, answer_buf, spilink::answer_bytes);
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        if (raw_n < sizeof raw_seen) {
            raw_seen[raw_n++] = answer_buf[i];
        }
        if (dec.feed(answer_buf[i]) == spilink::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

void settle() { settle_ms(spilink::settle_ms); }

bool command_once(Op op, const uint8_t* p, uint8_t len) {
    send_frame(op, p, len);
    settle();
    spilink::Frame f;
    if (!recv_frame(f)) {
        return false;
    }
    return f.op == Op::ack && f.len == 2 && f.data[0] == spilink::byte_of(op);
}

const uint8_t no_payload[1] = {0};

/// Three attempts, each separated by longer than the peer's own answer
/// window bound - the protocol's recovery guarantee.
bool command(Op op, const uint8_t* p = no_payload, uint8_t len = 0) {
    for (uint8_t k = 0; k < 3u; ++k) {
        if (command_once(op, p, len)) {
            return true;
        }
        (void)link_command_mode();
        settle_ms(spilink::serve_ms + 100u);
    }
    if (!link_quiet) {
        print(serial, "    LINK FAILURE op ", hex(spilink::byte_of(op)), ": the answer window carried");
        for (uint8_t i = 0; i < raw_n; ++i) {
            print(serial, " ", hex(raw_seen[i]));
        }
        print(serial, crlf, "      ", partner_text, " must be running spi_peer; its console '0' forces the dark "
                            "client back.", crlf);
    }
    return false;
}

bool query(Op op, spilink::Frame& data) {
    if (!command(op)) {
        return false;
    }
    settle();
    return recv_frame(data);
}

bool peer_ident(spilink::Ident& d) {
    spilink::Frame f;
    if (!query(Op::ident, f) || f.op != Op::ident_data || f.len != spilink::ident_size) {
        return false;
    }
    d = spilink::get_ident(f.data);
    return true;
}

bool peer_report(spilink::Report& r) {
    (void)link_command_mode();
    for (uint8_t k = 0; k < 4u; ++k) {
        spilink::Frame f;
        if (query(Op::report, f) && f.op == Op::report_data && f.len == spilink::report_size) {
            r = spilink::get_report(f.data);
            return true;
        }
        settle_ms(60);
    }
    return false;
}

bool peer_act(Op op, const spilink::Params& a) {
    uint8_t p[spilink::params_size];
    spilink::put_params(p, a);
    if (!command(op, p, spilink::params_size)) {
        return false;
    }
    settle();
    return true;
}

/// Is the peer there at all? Every letter that needs it asks first, so an
/// absent peer is a NAMED failure and never a hang.
bool need_peer() {
    link_quiet = true;
    (void)link_command_mode();
    bool up = false;
    for (uint8_t k = 0; k < 2u && !up; ++k) {
        up = command(Op::ping);
    }
    link_quiet = false;
    if (!up) {
        print(serial, "  THE PEER DID NOT ANSWER: ", partner_text, " must be running spi_peer.", crlf);
        bench.verdict("the peer answers a ping (the partner running spi_peer)", false);
        pair_release();
    }
    return up;
}

// ---- one exchange, commanded and then clocked ---------------------------------------------

/// One exchange: one window of `count` frames under this end's GPIO select,
/// held for the whole of it - the shape a PL022 client cannot take in modes
/// 0 and 2 and the PIO client takes in all four.
struct Exchange {
    spilink::Cfg cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    SpiClock rate = link_clock;
    uint16_t count = 16;
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
    uint8_t pattern = spilink::pattern_prbs;
    uint8_t flags = 0;
    uint8_t spare = 0;
    bool wide = false;      ///< 16-bit frames (spilink::spare_wide), `count` frames
    uint16_t ms = 400;
    bool engines = false;   ///< this end clocks the data on its DMA engines
};

/// The most BYTES an exchange carries.
constexpr uint16_t max_exchange = 256;
alignas(4) uint8_t xtx[max_exchange];
alignas(4) uint8_t xrx[max_exchange];

uint16_t exchange_bytes(const Exchange& e) {
    const uint16_t b = static_cast<uint16_t>(e.wide ? 2u * e.count : e.count);
    return b < max_exchange ? b : max_exchange;
}

/// The PL022 host is MSb first and nothing else: a client asked LSb first
/// is a reversal the peer is told to expect.
bool reversed(const Exchange& e) { return e.cfg.dord != 0; }

uint8_t last_status = 0;

bool do_exchange(const Exchange& e) {
    spilink::Params a{};
    a.cfg = e.cfg;
    a.count = e.count;
    a.ms = e.ms;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = e.pattern;
    a.flags = e.flags;
    a.spare = static_cast<uint8_t>(e.spare | (e.wide ? spilink::spare_wide : 0u));
    if (reversed(e)) {
        a.flags |= spilink::flag_expect_reversed;
    }
    if (!peer_act(Op::exchange, a)) {
        return false;
    }
    spilink::Stream out(e.pattern, e.seed_a);
    for (uint16_t i = 0; i < max_exchange; ++i) {
        xtx[i] = 0;
        xrx[i] = 0xEE;
    }
    const uint16_t bytes = exchange_bytes(e);
    const uint16_t n = static_cast<uint16_t>(e.wide ? bytes / 2u : bytes);
    const SpiDataSize bits = e.wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
    for (uint16_t i = 0; i < bytes; ++i) {
        xtx[i] = out.next();
    }
    const SpiMode mode = static_cast<SpiMode>(e.cfg.mode & 3u);
    if (e.engines) {
        (void)link_dma_mode();
        last_status = link_xfer<LinkDmaHost>(xtx, xrx, n, e.rate, mode, bits);
    } else {
        last_status = link_xfer(xtx, xrx, n, e.rate, mode, bits);
    }
    (void)link_command_mode();
    return true;
}

struct Verify {
    uint16_t mism = 0;
    uint8_t idx = 0xFF;
    uint8_t got = 0;
    uint8_t exp = 0;
};

Verify verify_rx(const Exchange& e) {
    Verify v{};
    const uint16_t bytes = exchange_bytes(e);
    for (uint16_t i = 0; i < bytes; ++i) {
        const uint8_t exp = spilink::wide_byte(e.pattern, e.seed_b, i, e.wide, reversed(e));
        if (xrx[i] != exp) {
            if (v.mism == 0) {
                v.idx = static_cast<uint8_t>(i < 255u ? i : 255u);
                v.got = xrx[i];
                v.exp = exp;
            }
            ++v.mism;
        }
    }
    return v;
}

/// Every byte this host read back is `level`: what a dark answer line reads.
bool read_all(const Exchange& e, uint8_t level) {
    const uint16_t bytes = exchange_bytes(e);
    for (uint16_t i = 0; i < bytes; ++i) {
        if (xrx[i] != level) {
            return false;
        }
    }
    return true;
}

bool exchange_exact(const Exchange& e, Verify& v, spilink::Report& r) {
    const bool ran = do_exchange(e);
    const bool rep = ran && peer_report(r);
    v = verify_rx(e);
    return ran && rep && last_status == spi_ok && v.mism == 0 && r.mism == 0 && r.count == e.count;
}

void print_miss(const Verify& v, const spilink::Report& r) {
    print(serial, " (host mism ", v.mism);
    if (v.mism != 0u) {
        print(serial, " idx ", v.idx, " got ", hex(v.got), " exp ", hex(v.exp));
    }
    print(serial, "; client ", r.count, " frames mism ", r.mism);
    if (r.mism != 0u) {
        print(serial, " idx ", r.idx, " got ", hex(r.got), " exp ", hex(r.exp));
    }
    print(serial, " flags ", hex(r.flags), " aux ", hex(r.aux0), "/", hex(r.aux2), ")");
}

/// An even divisor of clk_peri as the PL022's pair: CPSDVSR 2, SCR the rest.
constexpr SpiClock divisor_clock(uint16_t d) { return SpiClock{2, static_cast<uint8_t>(d / 2u - 1u)}; }

/// THE CEILING letter p judges to (the file header): the smallest even
/// divisor whose half SCK period clears four of the partner's cycles and
/// 15 ns of pads and wire, in picoseconds to keep the arithmetic exact.
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

/// THE RUNGS LETTER r JUDGES: a division of the far host's clk_peri (its
/// clk_sys, the peers' clock tree) whose SCK this chip's PL022 client
/// admits - its clk_peri at least twelve times the client's clock, the input
/// double-synchronized (docs/rp2040/spi.md, docs/rp2350/spi.md). /16 is 7.81
/// MHz from the RP2040 host against the RP2350 client's 12.5 MHz ceiling,
/// 9.375 MHz from the RP2350 host against the RP2040 client's 10.42; /8 is
/// over either (15.6 and 18.75 MHz).
constexpr bool client_admits(uint16_t d) {
    return 12ull * partner_sys_hz <= static_cast<uint64_t>(SysClock::pclk_hz) * d;
}

// =============================================================================
// n - THE PEER, and its dark listener
// =============================================================================

void tn_peer() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping over the four wires", true);
    spilink::Ident d{};
    const bool got = peer_ident(d);
    print(serial, "  peer: label '");
    for (uint8_t i = 0; i < 8u && d.label[i] != '\0'; ++i) {
        print(serial, d.label[i]);
    }
    print(serial, "' xtal=", d.xtal, " sanity=", hex(d.sanity), " fw=", hex(d.version), crlf);
    bench.verdict("ident comes back and it IS spi_peer, the partner's (kind and sanity byte), on its crystal",
                  got && d.sanity == spilink::ident_sanity && spilink::peer_kind(d.version) == partner_kind &&
                      d.xtal == 1u);
    uint8_t pings = 0;
    for (uint8_t i = 0; i < 10u; ++i) {
        if (command(Op::ping)) {
            ++pings;
        }
    }
    print(serial, "  ", pings, " of 10 pings at ", LinkHost::sck_hz(link_clock) / 1000u,
          " kHz SCK, mode 0, one protocol frame per select window", crlf);
    bench.verdict("the channel is steady over ten frames", pings == 10u);

    // THE DARK LISTENER: thirty-two bytes that are no frame (0x00..0x1F, no
    // magic among them) clocked into the peer's command listener under a
    // held select, the answer line read back - the peer drives nothing, so
    // the wire reads this end's own pull-up - and then a ping.
    for (uint8_t i = 0; i < 32u; ++i) {
        xtx[i] = i;
        xrx[i] = 0x00;
    }
    (void)link_xfer(xtx, xrx, 32);
    bool dark = true;
    for (uint8_t i = 0; i < 32u; ++i) {
        dark = dark && xrx[i] == 0xFFu;
    }
    settle_ms(20);
    const bool after = command(Op::ping);
    print(serial, "  noise into the dark listener: the answer line read ", dark ? "0xFF throughout" : "SOMETHING",
          ", a ping after it ", after ? "answered" : "LOST", crlf);
    bench.verdict("THE DARK LISTENER: thirty-two bytes of noise under a held select wake nothing - the answer line "
                  "reads this end's pull-up throughout - and the next command is answered",
                  dark && after);
    pair_release();
}

// =============================================================================
// o - the matrix against the peer
// =============================================================================

void to_matrix() {
    if (!need_peer()) {
        return;
    }
    // EVERY MODE, both client orders, both widths, both of the peer's
    // serves: sixteen frames in ONE held window each, what this host SENDS
    // judged by the client's report and what it READS judged here.
    uint8_t exact = 0;
    uint8_t runs = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        for (uint8_t o = 0; o < 2u; ++o) {
            for (uint8_t w = 0; w < 2u; ++w) {
                print(serial, "  mode ", m, o != 0u ? " client LSb" : " client MSb", w != 0u ? " 16-bit:" : "  8-bit:");
                for (uint8_t serve = 0; serve < 2u; ++serve) {
                    Exchange e{};
                    e.cfg.mode = m;
                    e.cfg.dord = o;
                    e.wide = w != 0u;
                    e.spare = serve != 0u ? spilink::spare_polled_pump : 0;
                    e.seed_a = static_cast<uint8_t>(0x13u + m + 4u * o + 8u * w + 16u * serve);
                    e.seed_b = static_cast<uint8_t>(0x57u + m + 4u * o + 8u * w + 16u * serve);
                    Verify v{};
                    spilink::Report r{};
                    const bool ok = exchange_exact(e, v, r) && (r.flags & spilink::report_extended) != 0u;
                    ++runs;
                    exact = static_cast<uint8_t>(exact + (ok ? 1u : 0u));
                    print(serial, serve != 0u ? "  polled " : "  dma ", ok ? "exact" : "BAD");
                    if (!ok) {
                        print_miss(v, r);
                    }
                }
                print(serial, crlf);
            }
        }
    }
    bench.verdict("THE MATRIX: this host's frames reach the foreign client exact and its answers come back exact - "
                  "four modes, the client MSb and LSb first (the LSb rows an exact two-way bit reversal), 8- and "
                  "16-bit frames, its DMA and its polled serve, sixteen frames under ONE held select each",
                  runs == 32u && exact == runs);

    // RECEIVE-ONLY: the client listens with its answer line DARK, so what
    // this host reads is its own pull-up; ANSWER-ONLY: the client loads its
    // answers and reads nothing; THE CRC: refused by a client with no unit.
    uint8_t ro_ok = 0;
    for (uint8_t w = 0; w < 2u; ++w) {
        Exchange e{};
        e.cfg.mode = 3;
        e.wide = w != 0u;
        e.spare = spilink::spare_receive_only;
        e.seed_a = static_cast<uint8_t>(0x61u + w);
        const bool ran = do_exchange(e);
        spilink::Report r{};
        const bool rep = ran && peer_report(r);
        const bool ok = rep && r.count == e.count && r.mism == 0u && read_all(e, 0xFF) &&
                        (r.flags & spilink::report_extended) != 0u;
        ro_ok = static_cast<uint8_t>(ro_ok + (ok ? 1u : 0u));
        print(serial, "  receive-only, ", w != 0u ? "16" : "8", "-bit: the client heard ", r.count, " mism ", r.mism,
              ", this host read ", read_all(e, 0xFF) ? "0xFF throughout" : "SOMETHING", crlf);
    }
    bench.verdict("RECEIVE-ONLY (spare_receive_only), both widths: the client hears every frame exact with its "
                  "answer line dark - this host reads its own pull-up throughout",
                  ro_ok == 2u);
    Exchange ao{};
    ao.cfg.mode = 1;
    ao.spare = spilink::spare_answer_only;
    ao.seed_b = 0x7C;
    Verify av{};
    spilink::Report ar{};
    const bool ao_ran = do_exchange(ao);
    const bool ao_rep = ao_ran && peer_report(ar);
    av = verify_rx(ao);
    print(serial, "  answer-only: this host read mism ", av.mism, ", the client loaded ", ar.count, crlf);
    bench.verdict("ANSWER-ONLY (spare_answer_only): the answers alone come back exact", ao_rep && av.mism == 0u &&
                                                                                       ar.count == ao.count);
    Exchange cr{};
    cr.spare = spilink::spare_crc;
    cr.count = 8;
    spilink::Report crr{};
    const bool cr_rep = do_exchange(cr) && peer_report(crr);
    bench.verdict("THE CRC (spare_crc) is refused by the PIO client - report_cfg_failed, no report_extended: a "
                  "state machine has no CRC unit to witness this host's with",
                  cr_rep && (crr.flags & spilink::report_cfg_failed) != 0u &&
                      (crr.flags & spilink::report_extended) == 0u);
    (void)link_command_mode();
    bench.verdict("and the command channel still works after all of that", command(Op::ping));
    pair_release();
}

// =============================================================================
// p - the rate ladder against the peer
// =============================================================================

void tp_ladder() {
    if (!need_peer()) {
        return;
    }
    // Even divisors of clk_peri from 256 to 2, on the polled host (32
    // frames) and on the engines (64), against the peer's DMA serve - and
    // the engines against its polled serve, whose own ceiling is the core's
    // loop. Every rung printed; the verdicts on the rungs the header's
    // ceiling admits.
    constexpr uint16_t rungs[] = {256, 128, 64, 32, 24, 20, 16, 14, 12, 10, 8, 6, 4, 2};
    constexpr uint16_t judged = judged_divisor();
    struct Serve {
        bool engines;
        bool polled_peer;
        const char* name;
    };
    const Serve serves[3] = {{false, false, "polled host / peer DMA "},
                             {true, false, "engined host / peer DMA"},
                             {true, true, "engined host / peer polled"}};
    uint16_t broke[3] = {0, 0, 0};   // the slowest divisor that broke, 0 = none
    bool judged_ok[2] = {true, true};
    print(serial, "  the ceiling judged: divisor ", judged, " = ", SysClock::pclk_hz / judged / 1000u,
          " kHz (half period >= 4 partner cycles + 15 ns at ", partner_sys_hz / 1'000'000u, " MHz)", crlf);
    for (uint8_t s = 0; s < 3u; ++s) {
        print(serial, "  ", serves[s].name, ":");
        for (const uint16_t d : rungs) {
            Exchange e{};
            e.rate = divisor_clock(d);
            e.count = serves[s].engines ? 64 : 32;
            e.engines = serves[s].engines;
            e.spare = serves[s].polled_peer ? spilink::spare_polled_pump : 0;
            e.seed_a = static_cast<uint8_t>(0x21u + d);
            e.seed_b = static_cast<uint8_t>(0x84u + d);
            Verify v{};
            spilink::Report r{};
            const bool ok = exchange_exact(e, v, r);
            print(serial, " /", d, ok ? " ok" : " BAD");
            if (!ok) {
                print(serial, "(h", v.mism, " c", r.mism, "/", r.count, ")");
                if (broke[s] == 0u) {
                    broke[s] = d;
                }
                if (s < 2u && d >= judged) {
                    judged_ok[s] = false;
                }
            }
        }
        print(serial, crlf);
    }
    print(serial, "  first break: polled host /", broke[0], ", engines /", broke[1], ", against the peer's polled "
                  "serve /", broke[2], " (0 = none)", crlf);
    bench.verdict("THE POLLED HOST is exact against the PIO client at every rung up to the judged ceiling",
                  judged_ok[0]);
    bench.verdict("THE ENGINES ON THE WIRE are exact against it at every rung up to the judged ceiling",
                  judged_ok[1]);
    (void)link_command_mode();
    bench.verdict("the command channel survives the climb", command(Op::ping));
    pair_release();
}

// =============================================================================
// q - THE KERNEL against the peer
// =============================================================================

namespace pk {

/// A 100-tick per-bus timeout (100 ms of the ticker): three orders of
/// magnitude above the longest transaction here (8 frames at /128, 65 us),
/// so only a completion that never comes meets it (letter w).
using SpiArb = SpiBus<LinkHost, P, 4, BusPassThrough, 100>;

class Probe {
public:
    using Event = std::variant<SpiDone>;
    static inline EventQueue<Event, 8, P> queue;
    static inline uint8_t replies[4];
    static inline uint8_t n = 0;

    static void init() { n = 0; }
    static void dispatch(const Event& e) {
        match(e, [](const SpiDone& d) {
            if (n < 4u) {
                replies[n] = d.status;
            }
            ++n;
        });
    }
};

using BusKernel = Tenuto<P, Probe, SpiArb>;

void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

LinkHost::Request request(const uint8_t* tx, uint8_t* rx, bool polled) {
    LinkHost::Request r{};
    r.cs = {};   // the letter holds the select across the four
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = 8;
    r.mode = SpiMode::mode0;
    r.clock = link_clock;
    r.polled = polled;
    r.reply = reply_to<Probe, SpiDone>();
    return r;
}

}  // namespace pk

void tq_kernel() {
    if (!need_peer()) {
        return;
    }
    constexpr uint16_t total = 4u * 8u;
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = total;
    a.ms = 800;
    a.seed_a = 0x3B;
    a.seed_b = 0x6D;
    a.pattern = spilink::pattern_prbs;
    const bool acted = peer_act(Op::exchange, a);
    bench.verdict("the peer accepted the exchange the kernel letter clocks", acted);
    if (!acted) {
        pair_release();
        return;
    }
    static uint8_t qtx[4][8];
    static uint8_t qrx[4][8];
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint8_t i = 0; i < 4u; ++i) {
            for (uint8_t k = 0; k < 8u; ++k) {
                qtx[i][k] = out.next();
                qrx[i][k] = 0xEE;
            }
        }
    }
    pk::BusKernel::init_all();
    pair_owner = PairOwner::bus_ao;
    LinkHost::prime(SpiMode::mode0, link_clock);
    spin_us(10);
    CsPin::clear();
    link_hold();
    for (uint8_t i = 0; i < 4u; ++i) {
        post<pk::SpiArb>(pk::request(qtx[i], qrx[i], i == 2u));
    }
    (void)wait_until(
        [] {
            pk::pump();
            return pk::Probe::n >= 4u;
        },
        800'000u);
    pk::pump();
    link_hold();
    CsPin::set();
    pair_owner = PairOwner::host;
    uint16_t mism = 0;
    {
        spilink::Stream want(a.pattern, a.seed_b);
        for (uint8_t i = 0; i < 4u; ++i) {
            for (uint8_t k = 0; k < 8u; ++k) {
                if (qrx[i][k] != want.next()) {
                    ++mism;
                }
            }
        }
    }
    const bool replies_ok = pk::Probe::n == 4u && pk::Probe::replies[0] == spi_ok &&
                            pk::Probe::replies[1] == spi_ok && pk::Probe::replies[2] == spi_ok &&
                            pk::Probe::replies[3] == spi_ok;
    spilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  kernel: ", pk::Probe::n, " replies; host mism=", mism, "; the peer read ", r.count, " of ", total,
          " with mism=", r.mism, " flags=", hex(r.flags), crlf);
    bench.verdict("FOUR TRANSACTIONS QUEUED FROM ONE DISPATCH (one polled among three pumped) come back in order, "
                  "every one spi_ok, and the bytes on the wire are what each request lent - read back by A SECOND "
                  "CHIP, util/spi_bus.hpp and kernel/ unchanged",
                  replies_ok && mism == 0u && rep && r.count == total && r.mism == 0u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the kernel's window", command(Op::ping));
    pair_release();
}

// =============================================================================
// r - THE ROLES INVERT: this chip's PL022 as the CLIENT, the peer's as host
// =============================================================================

alignas(4) uint8_t ctx[256];
alignas(4) uint8_t crx[256];

// THE WINDOW COUNTER: a PIO machine of this chip watching the select pad -
// a PIO sees every pad whoever owns it (RP2040 2.19.2, the PIO row of the
// function table; RP2350 9.4) - and counting the windows that CLOSE, one a
// rising edge however short the pulse: two cycles of the machine at
// clk_sys resolve it, where the core's loop above would sample it once a
// turn. X starts at all ones and every close decrements it.
constexpr PioProgram<3> window_program = [] {
    PioProgram<3> p{};
    p.code = {pio_wait(false, PioWaitOn::gpio, count_cs_index), pio_wait(true, PioWaitOn::gpio, count_cs_index),
              pio_jmp(PioJmp::x_dec, 0)};
    return p;
}();
/// THE PULSE GAUGE beside it, the second machine: every time the select
/// rises inside a burst it counts, two cycles a turn (a JMP Y-- and a JMP
/// PIN), until the select falls again, and pushes the count - a high pulse
/// of about 2n machine cycles, two of synchronizer and the loop's own two
/// the uncertainty, pushes n (a push that finds the FIFO full is dropped,
/// never stalls). The core's loop keeps the smallest (gauge_take).
constexpr PioProgram<7> gauge_program = [] {
    PioProgram<7> p{};
    p.code = {pio_wait(false, PioWaitOn::gpio, count_cs_index), pio_wait(true, PioWaitOn::gpio, count_cs_index),
              pio_mov(PioMovTo::y, PioMovFrom::null, PioMovOp::invert), pio_jmp(PioJmp::y_dec, 4),
              pio_jmp(PioJmp::pin, 3), pio_mov(PioMovTo::isr, PioMovFrom::y, PioMovOp::invert),
              pio_push(false, false)};
    return p;
}();
constexpr uint8_t gauge_offset = 3;
bool counter_loaded = false;
uint32_t gauge_min = 0xFFFFFFFFu;
uint32_t gauge_n = 0;

/// The counter and the gauge armed: loaded once, then restarted - X at all
/// ones, the gauge's push after its MOV (the wrap carries it to 0) - with
/// the select high.
void count_windows() {
    if (!counter_loaded) {
        (void)CountBlk::reset();
        open_count_window();
        counter_loaded = CountBlk::load(window_program, 0) && CountBlk::load(gauge_program, gauge_offset);
    }
    PioSmConfig c{};
    (void)CountSm::init(window_program, 0, c);
    (void)CountSm::exec_wait(pio_mov(PioMovTo::x, PioMovFrom::null, PioMovOp::invert));
    PioSmConfig g{};
    g.jmp_pin = count_cs_index;
    g.fifo_join = PioFifoJoin::rx;
    (void)GaugeSm::init(gauge_program, gauge_offset, g);
    gauge_min = 0xFFFFFFFFu;
    gauge_n = 0;
    CountSm::enable(true);
    GaugeSm::enable(true);
}
/// The gauge's counts drained, the smallest kept.
[[gnu::always_inline]] inline void gauge_take() {
    while (!GaugeSm::rx_empty()) {
        const uint32_t v = GaugeSm::pop();
        gauge_min = v < gauge_min ? v : gauge_min;
        ++gauge_n;
    }
}
/// The windows closed since count_windows(): X read back through the ISR.
uint32_t windows_seen() {
    GaugeSm::enable(false);
    gauge_take();
    CountSm::enable(false);
    (void)CountSm::exec_wait(pio_mov(PioMovTo::isr, PioMovFrom::x));
    (void)CountSm::exec_wait(pio_push(false, false));
    const auto x = CountSm::pop_wait(1000u);
    return x ? ~*x : 0xFFFFu;
}

struct ClientRun {
    bool ran = false;
    uint16_t got = 0;        ///< frames received
    uint16_t mism = 0;       ///< bytes received wrong
    uint8_t idx = 0xFF, read = 0, want = 0;
    bool oe_deselected = false;   ///< the answer pad driven before the burst
    bool oe_selected = false;     ///< ... and while the far host clocked
    bool overrun = false;
    uint16_t windows = 0;         ///< select windows that closed, counted by a PIO machine
    uint32_t pulse_min = 0;       ///< the shortest high pulse inside the burst, in gauge counts
    uint32_t pulses = 0;          ///< how many the gauge measured
    spilink::Report peer{};
};

/// How a client run treats its answer line.
enum class Answer : uint8_t { driven, sod, released };

/// THE CLIENT'S LOOP, one status read a turn serving both FIFOs: at /16 of
/// the far host's 125 or 150 MHz a frame is 853 to 1024 ns back to back, 107
/// cycles of this core's at the least, and the receive FIFO is eight frames
/// deep. The bound is counted in turns, never a clock: some 300 ms at one
/// turn per ten cycles. IT RUNS FROM SRAM, the program's own `.fast_text`
/// (util/placement.hpp), flattened so that nothing it calls is fetched from
/// the flash: out of the flash, the loop's first turns of a burst fetch its
/// lines through the XIP cache, some fifty cycles a line on the RP2040, while
/// the far host clocks on - the first role-inverted burst after a flash once
/// overran (55 frames of 64) and once ran dry after the eight preloaded and
/// one more; with the cache emptied before every burst and only this loop in
/// SRAM, the wait for the select and the pad read before it, cold in the
/// flash, still left the loop behind at /32 in modes 1 and 3. So the WAIT for
/// the far host's select is in here too, with the read of the answer pad's
/// enable it is followed by: nothing between the select falling and the
/// loop's first turn is fetched from the flash. The wait is bounded in turns
/// as the loop is (some 200 ms). Returns the frames received; `sent` the
/// answers written, `oe_selected` the pad's output enable once the window
/// opened.
[[gnu::section(".fast_text"), gnu::noinline, gnu::flatten]] uint16_t serve_client(bool wide, uint16_t count,
                                                                                uint16_t& sent,
                                                                                bool& oe_selected) {
    using S = Pl022<1>;
    for (uint32_t spins = SysClock::hz / 50u; CsPin::read() && spins != 0u; --spins) {
    }
    oe_selected = TxPin::status().oe_to_pad;
    uint16_t got = 0;
    uint16_t out = sent;
    uint32_t turns = SysClock::hz / 33u;
    while (got < count && turns-- != 0u) {
        const uint32_t sr = S::flags();
        if ((sr & SpiFlag::rx_not_empty) != 0u) {
            const uint16_t f = S::read_data();
            if (wide) {
                crx[2u * got] = static_cast<uint8_t>(f);
                crx[2u * got + 1u] = static_cast<uint8_t>(f >> 8);
            } else {
                crx[got] = static_cast<uint8_t>(f);
            }
            ++got;
        }
        if ((sr & SpiFlag::tx_not_full) != 0u && out < count) {
            const uint16_t v = wide ? static_cast<uint16_t>(ctx[2u * out] | (static_cast<uint16_t>(ctx[2u * out + 1u]) << 8))
                                    : ctx[out];
            S::write_data(v);
            ++out;
        }
        gauge_take();
    }
    sent = out;
    return got;
}

/// One burst with this chip as the client: `count` frames back to back in
/// `mode`, the peer's PL022 host at `div` of its clk_peri, its select the
/// block's frame signal. The client preloads its FIFO with the select high
/// and keeps it fed from serve_client().
ClientRun client_burst(uint8_t mode, bool wide, uint16_t count, uint16_t div, uint8_t seed_a, uint8_t seed_b,
                       Answer answer = Answer::driven) {
    ClientRun out{};
    spilink::Params a{};
    a.count = count;
    a.ms = 500;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = 40;    // lead-in ms: time for this end to become a client
    a.aux16 = div;
    a.cfg.apply = 1;
    a.cfg.mode = mode;
    a.spare = wide ? spilink::spare_wide : 0u;
    if (!peer_act(Op::host_burst, a)) {
        return out;
    }
    pair_owner = PairOwner::none;
    LinkHost::release();
    (void)CsPin::release();
    const uint8_t bits = wide ? 16u : 8u;
    const bool ok = LinkClient::init(clock, {.mode = static_cast<SpiMode>(mode), .bits = bits,
                                             .drive_output = answer != Answer::released});
    if (!ok) {
        return out;
    }
    LinkClient::interrupts(SpiInterrupt::all, false);
    if (answer == Answer::sod) {
        LinkClient::sod(true);
    }
    spilink::Stream answers(a.pattern, a.seed_b);
    const uint16_t bytes = static_cast<uint16_t>(wide ? 2u * count : count);
    for (uint16_t i = 0; i < bytes; ++i) {
        ctx[i] = answers.next();
        crx[i] = 0xEE;
    }
    auto frame_at = [&](uint16_t k) -> uint16_t {
        return wide ? static_cast<uint16_t>(ctx[2u * k] | (static_cast<uint16_t>(ctx[2u * k + 1u]) << 8)) : ctx[k];
    };
    out.oe_deselected = TxPin::status().oe_to_pad;
    count_windows();
    LinkClient::enable(frame_at(0));
    uint16_t sent = 1;
    while (sent < count && sent < LinkClient::frames_ahead && LinkClient::writable()) {
        LinkClient::write(frame_at(sent++));
    }
    // The pad's output enable once the far host has the select down - the
    // first window open - is read in serve_client(), after its wait.
    bool oe_selected = false;
    const uint16_t got = serve_client(wide, count, sent, oe_selected);
    out.oe_selected = oe_selected;
    (void)wait_until([] { return CsPin::read(); }, 100'000u);
    out.windows = static_cast<uint16_t>(windows_seen());
    out.pulse_min = gauge_min;
    out.pulses = gauge_n;
    out.overrun = LinkClient::overrun();
    out.got = got;
    out.ran = true;
    LinkClient::release();
    {
        spilink::Stream want(a.pattern, a.seed_a);
        const uint16_t rb = static_cast<uint16_t>(wide ? 2u * got : got);
        for (uint16_t i = 0; i < rb; ++i) {
            const uint8_t exp = want.next();
            if (crx[i] != exp) {
                if (out.mism == 0u) {
                    out.idx = static_cast<uint8_t>(i < 255u ? i : 255u);
                    out.read = crx[i];
                    out.want = exp;
                }
                ++out.mism;
            }
        }
    }
    settle_ms(20);
    (void)peer_report(out.peer);
    return out;
}

void print_client(const ClientRun& c, uint16_t count) {
    print(serial, " received ", c.got, "/", count, " mism ", c.mism);
    if (c.mism != 0u) {
        print(serial, " (idx ", c.idx, " got ", hex(c.read), " exp ", hex(c.want), ")");
    }
    print(serial, "; the host read ", c.peer.count, " mism ", c.peer.mism);
    if (c.peer.mism != 0u) {
        print(serial, " (idx ", c.peer.idx, " got ", hex(c.peer.got), " exp ", hex(c.peer.exp), ")");
    }
    print(serial, "; the burst took ", 10u * c.peer.aux0, " us, ", c.windows, " windows");
    if (c.overrun) {
        print(serial, " OVERRUN");
    }
}

void tr_roles() {
    if (!need_peer()) {
        return;
    }
    // EVERY MODE, 8- and 16-bit frames, 64 frames back to back at three of
    // the peer's divisions - what this client RECEIVES judged here, what it
    // ANSWERS judged by the peer's report. In modes 0 and 2 the peer's frame
    // signal pulses between the frames (the PL022 client's need: its select
    // freezes the shift register under SPH = 0); in modes 1 and 3 it is held
    // through the burst, a multi-frame window.
    // EVERY RUNG IS JUDGED: each is one the client admits (client_admits()).
    // /16 is the RP2350 client's rung that lost a frame now and then until
    // its answer line got its series resistor at the pad (wire 3,
    // docs/boards/weact-rp2350b.md): the answer's own edge reached the
    // select and SCK inputs beside it (letter x counts it).
    constexpr uint16_t divs[3] = {64, 32, 16};
    static_assert(client_admits(64) && client_admits(32) && client_admits(16),
                  "letter r's verdicts name /64, /32 and /16 as rungs the client admits");
    uint8_t exact = 0, runs = 0, framed = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        for (uint8_t w = 0; w < 2u; ++w) {
            print(serial, "  mode ", m, w != 0u ? " 16-bit:" : "  8-bit:");
            for (const uint16_t d : divs) {
                const ClientRun c = client_burst(m, w != 0u, 64, d, static_cast<uint8_t>(0x2Cu + m + 4u * w + d),
                                                 static_cast<uint8_t>(0x71u + m + 4u * w + d));
                const bool ok = c.ran && c.got == 64u && c.mism == 0u && c.peer.count == 64u && c.peer.mism == 0u &&
                                !c.overrun;
                ++runs;
                exact = static_cast<uint8_t>(exact + (ok ? 1u : 0u));
                // The burst's time, the peer's ruler in tens of microseconds.
                print(serial, " /", d, ok ? " ok" : " BAD", " ", c.windows, "w ", 10u * c.peer.aux0, "us");
                if (!ok) {
                    print_client(c, 64);
                }
                // The window counter resolves a pulse of two machine cycles;
                // the frame signal's is about half an SCK period, 48 to 64 ns
                // at /16 (measured on either end, docs/rp2040/spi.md and
                // docs/rp2350/spi.md): six of the counting end's cycles at
                // the least, three times what it resolves (letter x).
                {
                    const uint16_t want = (m & 1u) == 0u ? 64u : 1u;
                    framed = static_cast<uint8_t>(framed + (c.windows == want ? 1u : 0u));
                }
            }
            print(serial, crlf);
        }
    }
    print(serial, "  (w = select windows the client saw close; us = the burst on the peer's ruler, against a wire "
                  "of 64 x 8 or 16 SCK periods - plus 1.5 a frame in modes 0 and 2)", crlf);
    bench.verdict("THE HOST'S OWN FRAME SIGNAL ON THE WIRE, counted by a PIO machine: a window per frame in modes 0 "
                  "and 2, ONE window for the 64 back-to-back frames in modes 1 and 3 (4.4.3.10 to 4.4.3.13), at /64, "
                  "/32 and /16",
                  framed == 24u);
    bench.verdict("THIS CHIP'S PL022 AS CLIENT, under a foreign PL022 host's own frame signal, exact both ways in "
                  "every mode, 8- and 16-bit, 64 frames back to back at /64, /32 and /16 of the host's clk_peri - "
                  "every rung the client admits - more than one frame per select in modes 0 and 2 on the terms "
                  "SPH = 0 sets",
                  runs == 24u && exact == runs);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the role swaps", command(Op::ping));
    pair_release();
}

// =============================================================================
// m - THE PAD FACTS with a foreign host
// =============================================================================

void tm_pads() {
    if (!need_peer()) {
        return;
    }
    // The client's answer pad in three states, each under a burst of the
    // peer's: handed to the block (driven), handed to the block with SOD
    // set, and released (the dark listener) - the pad's output enable read
    // where the pad sees it (GPIO_STATUS.OETOPAD), what the far host read,
    // and what this client heard meanwhile.
    const ClientRun dr = client_burst(3, false, 32, 32, 0x41, 0x92, Answer::driven);
    const ClientRun so = client_burst(3, false, 32, 32, 0x42, 0x93, Answer::sod);
    const ClientRun rl = client_burst(3, false, 32, 32, 0x43, 0x94, Answer::released);
    print(serial, "  driven:   pad ", dr.oe_deselected ? "driven" : "released", " deselected, ",
          dr.oe_selected ? "driven" : "released", " selected;");
    print_client(dr, 32);
    print(serial, crlf, "  SOD:      pad ", so.oe_deselected ? "driven" : "released", " deselected, ",
          so.oe_selected ? "driven" : "released", " selected;");
    print_client(so, 32);
    print(serial, " sum ", hex(so.peer.sum), crlf, "  released: pad ", rl.oe_deselected ? "driven" : "released",
          " deselected, ", rl.oe_selected ? "driven" : "released", " selected;");
    print_client(rl, 32);
    print(serial, " sum ", hex(rl.peer.sum), crlf);
    // A steady byte v over 32 frames is 32 x v in the report's 16-bit sum.
    if constexpr (client_pad_follows_select) {
        bench.verdict("A CLIENT'S ANSWER PAD FOLLOWS ITS SELECT: released while deselected, driven while selected - "
                      "nSSPOE reaches the pad on this chip (12.3.1)",
                      dr.ran && !dr.oe_deselected && dr.oe_selected);
        bench.verdict("and the far host reads its answers exact", dr.peer.count == 32u && dr.peer.mism == 0u);
        bench.verdict("UNDER SOD the pad is released selected or not - the far host reads its own pull-up, 0xFF "
                      "throughout - and the client still hears the host's frames exact",
                      so.ran && !so.oe_deselected && !so.oe_selected && so.peer.count == 32u &&
                          so.peer.sum == 32u * 0xFFu && so.got == 32u && so.mism == 0u);
    } else {
        bench.verdict("A CLIENT'S ANSWER PAD IS DRIVEN WHILE DESELECTED, as while selected: nSSPOE reaches no pad on "
                      "this chip (4.4.3.14), so the pad's output enable is the function's",
                      dr.ran && dr.oe_deselected && dr.oe_selected);
        bench.verdict("and the far host reads its answers exact", dr.peer.count == 32u && dr.peer.mism == 0u);
        // The answers' stream starts at its seed (pattern_value's prbs at 0).
        bench.verdict("UNDER SOD the pad stays driven and the far host reads the client's FIRST answer in every "
                      "frame - SOD keeps the later ones off the wire, not the pad - while the client still hears "
                      "the host's frames exact",
                      so.ran && so.oe_deselected && so.oe_selected && so.peer.count == 32u &&
                          so.peer.sum == 32u * 0x93u && so.got == 32u && so.mism == 0u);
    }
    bench.verdict("RELEASED (the dark listener) the pad is not driven, the far host reads its own pull-up - 0xFF "
                  "throughout - and the client hears exact",
                  rl.ran && !rl.oe_deselected && !rl.oe_selected && rl.peer.count == 32u &&
                      rl.peer.sum == 32u * 0xFFu && rl.got == 32u && rl.mism == 0u);
    (void)link_command_mode();
    bench.verdict("the command channel is back", command(Op::ping));
    pair_release();
}

// =============================================================================
// s - THE CLIENT SERVED FROM ITS VECTOR: the ladder to the ceiling, the
// receive timeout's tail, the overrun
// =============================================================================

// THE VECTOR'S STATE: what the ISR body below reads and writes. The vector
// is entered on RXIM (the receive FIFO at half - four frames), RTIM (a
// frame waiting with the wire idle 32 bit periods: the tail under the
// half) and RORIM (the overrun); TXIM stays masked - every receive entry
// refills the transmit FIFO, which the four frames just received have
// half emptied.
volatile uint16_t cv_got = 0;
volatile uint16_t cv_sent = 0;
volatile uint16_t cv_count = 0;
volatile bool cv_wide = false;
volatile bool cv_drain = true;
volatile uint16_t cv_rx_entries = 0;
volatile uint16_t cv_timeout_entries = 0;
volatile uint16_t cv_overrun_entries = 0;

[[gnu::always_inline]] inline uint16_t ctx_frame(uint16_t k, bool wide) {
    return wide ? static_cast<uint16_t>(ctx[2u * k] | (static_cast<uint16_t>(ctx[2u * k + 1u]) << 8)) : ctx[k];
}

/// The client's ISR body: the sources counted, the receive FIFO drained,
/// the transmit FIFO refilled to its depth. IN SRAM, like serve_client():
/// from the flash its first entry after a quiet spell fetches its lines
/// through the XIP cache - 6000 to 8600 cycles cold on the RP2040
/// (docs/rp2040/platform.md) - while the far host clocks on: measured,
/// the first entry let the receive FIFO overflow at /64 and the transmit
/// FIFO run dry at /16. Only the vector's own lines stay in the flash.
[[gnu::section(".fast_text"), gnu::noinline, gnu::flatten]] void client_vector_serve() {
    using S = Pl022<1>;
    const uint32_t up = LinkClient::isr();
    if ((up & SpiInterrupt::overrun) != 0u) {
        cv_overrun_entries = static_cast<uint16_t>(cv_overrun_entries + 1u);
    }
    if ((up & SpiInterrupt::rx_timeout) != 0u) {
        cv_timeout_entries = static_cast<uint16_t>(cv_timeout_entries + 1u);
    } else if ((up & SpiInterrupt::rx) != 0u) {
        cv_rx_entries = static_cast<uint16_t>(cv_rx_entries + 1u);
    }
    if (!cv_drain) {
        return;
    }
    const bool wide = cv_wide;
    uint16_t got = cv_got;
    while (S::rx_not_empty()) {
        const uint16_t f = S::read_data();
        if (got < cv_count) {
            if (wide) {
                crx[2u * got] = static_cast<uint8_t>(f);
                crx[2u * got + 1u] = static_cast<uint8_t>(f >> 8);
            } else {
                crx[got] = static_cast<uint8_t>(f);
            }
        }
        ++got;
    }
    cv_got = got;
    uint16_t sent = cv_sent;
    while (sent < cv_count && S::tx_not_full()) {
        S::write_data(ctx_frame(sent, wide));
        ++sent;
    }
    cv_sent = sent;
}

/// One burst of the peer's host with this chip's client served FROM ITS
/// VECTOR (no loop on the core): `count` frames at `div` of the far
/// clk_peri. `drain` false leaves the receive FIFO alone (the overrun run):
/// the vector counts, takes nothing, refills nothing.
ClientRun client_vector_burst(uint8_t mode, bool wide, uint16_t count, uint16_t div, uint8_t seed_a, uint8_t seed_b,
                              bool drain = true) {
    ClientRun out{};
    spilink::Params a{};
    a.count = count;
    a.ms = 500;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = 40;
    a.aux16 = div;
    a.cfg.apply = 1;
    a.cfg.mode = mode;
    a.spare = wide ? spilink::spare_wide : 0u;
    if (!peer_act(Op::host_burst, a)) {
        return out;
    }
    pair_owner = PairOwner::none;
    LinkHost::release();
    (void)CsPin::release();
    const uint8_t bits = wide ? 16u : 8u;
    if (!LinkClient::init(clock, {.mode = static_cast<SpiMode>(mode), .bits = bits})) {
        return out;
    }
    LinkClient::interrupts(SpiInterrupt::all, false);
    spilink::Stream answers(a.pattern, a.seed_b);
    const uint16_t bytes = static_cast<uint16_t>(wide ? 2u * count : count);
    for (uint16_t i = 0; i < bytes; ++i) {
        ctx[i] = answers.next();
        crx[i] = 0xEE;
    }
    count_windows();
    LinkClient::enable(ctx_frame(0, wide));
    uint16_t sent = 1;
    while (sent < count && sent < LinkClient::frames_ahead && LinkClient::writable()) {
        LinkClient::write(ctx_frame(sent++, wide));
    }
    cv_got = 0;
    cv_sent = drain ? sent : count;
    cv_count = count;
    cv_wide = wide;
    cv_drain = drain;
    cv_rx_entries = 0;
    cv_timeout_entries = 0;
    cv_overrun_entries = 0;
    pair_owner = PairOwner::client;
    LinkClient::interrupts(drain ? (SpiInterrupt::rx | SpiInterrupt::rx_timeout | SpiInterrupt::overrun)
                                 : SpiInterrupt::overrun,
                           true);
    // The select falls within the peer's lead-in (40 ms); in modes 0 and 2
    // it pulses between frames, so the burst's end is the vector's count
    // (the tail included: RTIM 32 bit periods after the last frame, under
    // 6 us at /64) - or, for a client that takes nothing, 5 ms, twenty
    // times a burst of 24 frames at /64.
    (void)wait_until([] { return !CsPin::read(); }, 200'000u);
    if (drain) {
        (void)wait_until([] { return cv_got >= cv_count; }, 300'000u);
    } else {
        spin_us(5000);
    }
    spin_us(200);
    LinkClient::interrupts(SpiInterrupt::all, false);
    pair_owner = PairOwner::none;
    out.windows = static_cast<uint16_t>(windows_seen());
    out.overrun = LinkClient::overrun();
    if (drain) {
        out.got = cv_got;
    } else {
        // What the FIFO kept, read out after the burst.
        uint16_t got = 0;
        while (const auto f = LinkClient::poll()) {
            if (got < count) {
                if (wide) {
                    crx[2u * got] = static_cast<uint8_t>(*f);
                    crx[2u * got + 1u] = static_cast<uint8_t>(*f >> 8);
                } else {
                    crx[got] = static_cast<uint8_t>(*f);
                }
            }
            ++got;
        }
        out.got = got;
    }
    out.ran = true;
    LinkClient::release();
    {
        spilink::Stream want(a.pattern, a.seed_a);
        const uint16_t rb = static_cast<uint16_t>(wide ? 2u * (out.got < count ? out.got : count)
                                                       : (out.got < count ? out.got : count));
        for (uint16_t i = 0; i < rb; ++i) {
            const uint8_t exp = want.next();
            if (crx[i] != exp) {
                if (out.mism == 0u) {
                    out.idx = static_cast<uint8_t>(i < 255u ? i : 255u);
                    out.read = crx[i];
                    out.want = exp;
                }
                ++out.mism;
            }
        }
    }
    settle_ms(20);
    (void)peer_report(out.peer);
    return out;
}

void ts_client_vector() {
    if (!need_peer()) {
        return;
    }
    // THE LADDER: every even division of the far host's clk_peri from /64
    // down to the last this client admits (client_admits(): the far SCK no
    // faster than this chip's clk_peri / 12), modes 0 and 3, 8- and 16-bit,
    // 67 frames - sixteen receive entries of four and a TAIL OF THREE the
    // receive timeout serves (the FIFO never reaches half again). The two
    // rungs past the ceiling are printed, not judged.
    // The vector keeps up when its entry, four reads and four writes come
    // inside the four frames the transmit FIFO still holds: at the fastest
    // admitted rung (12.5 MHz, 8-bit) that is 2.56 us, 320 cycles of the
    // RP2040's clk_sys, 384 of the RP2350's.
    // THE FIRST BURST IS COLD: the vector's own lines and the veneer into
    // SRAM are fetched through the XIP cache at its first entry. Printed,
    // not judged; every burst after it runs with them cached.
    {
        const ClientRun w = client_vector_burst(3, false, 67, 64, 0x11, 0x22);
        const bool ok = w.ran && w.got == 67u && w.mism == 0u && w.peer.count == 67u && w.peer.mism == 0u;
        print(serial, "  the first burst, cold (mode 3, 8-bit, /64): ", ok ? "ok" : "BAD", " ", cv_rx_entries, "+",
              cv_timeout_entries);
        if (!ok) {
            print_client(w, 67);
        }
        print(serial, crlf);
    }
    uint8_t runs = 0, exact = 0, tails = 0;
    uint16_t last_admitted = 0;
    for (uint16_t d = 64; d >= 8u; d = static_cast<uint16_t>(d == 64u ? 32u : (d == 32u ? 16u : d - 2u))) {
        const bool judged = client_admits(d);
        if (!judged && d + 4u < last_admitted) {
            break;
        }
        print(serial, "  /", d, judged ? "" : " (past the ceiling, printed)", ":");
        for (uint8_t m = 0; m < 4u; m = static_cast<uint8_t>(m + 3u)) {
            for (uint8_t w = 0; w < 2u; ++w) {
                const ClientRun c = client_vector_burst(m, w != 0u, 67, d, static_cast<uint8_t>(0x19u + m + 2u * w + d),
                                                        static_cast<uint8_t>(0x63u + m + 2u * w + d));
                const bool ok = c.ran && c.got == 67u && c.mism == 0u && c.peer.count == 67u && c.peer.mism == 0u &&
                                !c.overrun && cv_overrun_entries == 0u;
                print(serial, " m", m, w != 0u ? "/16 " : "/8 ", ok ? "ok" : "BAD", " ", cv_rx_entries, "+",
                      cv_timeout_entries);
                if (!ok) {
                    print_client(c, 67);
                }
                if (judged) {
                    ++runs;
                    exact = static_cast<uint8_t>(exact + (ok ? 1u : 0u));
                }
                if (d == 64u) {
                    // At /64 a frame is 3.4 us or more and an entry from
                    // SRAM a microsecond: the sixteenth level entry is over
                    // before the 65th frame lands, so the last three are
                    // RTIM's. Faster, an entry may still be draining when
                    // they land and take them itself (printed).
                    tails = static_cast<uint8_t>(tails + (cv_rx_entries == 16u && cv_timeout_entries == 1u ? 1u : 0u));
                }
            }
        }
        print(serial, crlf);
        if (judged) {
            last_admitted = d;
        }
    }
    print(serial, "  (entries: receive-at-half + receive timeout; ", runs, " judged runs down to /", last_admitted, ")",
          crlf);
    bench.verdict("THE CLIENT FROM ITS VECTOR, every rung it admits down to the ceiling: 67 frames exact both ways in "
                  "modes 0 and 3, 8- and 16-bit, no overrun - one entry per four frames",
                  runs != 0u && exact == runs);
    bench.verdict("THE RECEIVE TIMEOUT'S TAIL: at /64 the 67 frames are sixteen level entries of four and ONE "
                  "receive-timeout entry for the three under the FIFO's half, in each of the four runs",
                  tails == 4u);
    // THE OVERRUN: 24 frames into a client that takes nothing - its eight
    // answers preloaded, RORIM alone unmasked.
    const ClientRun o = client_vector_burst(3, false, 24, 64, 0x2D, 0x8E, false);
    print(serial, "  nothing taken: the FIFO kept ", o.got, " frames (", o.mism, " wrong), overrun ", o.overrun,
          ", the vector entered ", cv_overrun_entries, " times on RORIM; the host read its first mismatch at ",
          o.peer.idx, crlf);
    // One RORIM entry a frame lost: the vector clears ROR (isr()) in about
    // a microsecond and the next frame lands 3.4 us later at /64.
    bench.verdict("THE CLIENT'S OVERRUN: a client that takes nothing keeps the FIFO's first eight frames exact and "
                  "enters its vector on RORIM once for each of the sixteen lost; the far host reads the eight "
                  "preloaded answers exact",
                  o.ran && o.got == 8u && o.mism == 0u && cv_overrun_entries == 16u &&
                      (o.peer.mism == 0u || o.peer.idx >= 8u));
    (void)link_command_mode();
    bench.verdict("the command channel is back after the client's runs", command(Op::ping));
    pair_release();
}

// =============================================================================
// w - THE WEDGE: a completion that never comes, the per-bus timeout,
// recover()
// =============================================================================

void tw_wedge() {
    if (!need_peer()) {
        return;
    }
    // The select stays HIGH: the peer is not addressed, the transactions
    // run against nobody (its answer line dark, this end reading the pull-up)
    // and only their completions matter. The first is pumped with its
    // completion swallowed (the ISR body run, nothing posted): the arbiter's
    // 100-tick timeout must answer it spi_timeout and recover() the engine.
    // The bound: the time event is armed at the start and matures on the
    // 100th tick, the first of which comes anywhere within a tick: 99 to
    // 100 ms, plus the pump's turn - judged 99 to 102 ms.
    static uint8_t wtx[4][8];
    static uint8_t wrx[4][8];
    for (uint8_t i = 0; i < 4u; ++i) {
        for (uint8_t k = 0; k < 8u; ++k) {
            wtx[i][k] = static_cast<uint8_t>(0x30u + 8u * i + k);
        }
    }
    pk::BusKernel::init_all();
    pair_owner = PairOwner::bus_ao;
    pair_swallow = true;
    const uint32_t t0 = us_now();
    post<pk::SpiArb>(pk::request(wtx[0], wrx[0], false));
    (void)wait_until(
        [] {
            pk::pump();
            return pk::Probe::n >= 1u;
        },
        800'000u);
    const uint32_t took_us = us_now() - t0;
    const uint8_t first = pk::Probe::replies[0];
    const uint8_t first_n = pk::Probe::n;
    pair_swallow = false;
    pk::Probe::n = 0;
    for (uint8_t i = 0; i < 4u; ++i) {
        post<pk::SpiArb>(pk::request(wtx[i], wrx[i], i == 2u));
    }
    (void)wait_until(
        [] {
            pk::pump();
            return pk::Probe::n >= 4u;
        },
        800'000u);
    pk::pump();
    uint8_t ok = 0;
    for (uint8_t i = 0; i < 4u; ++i) {
        ok = static_cast<uint8_t>(ok + (pk::Probe::replies[i] == spi_ok ? 1u : 0u));
    }
    const uint8_t after_n = pk::Probe::n;
    pair_owner = PairOwner::host;
    print(serial, "  the swallowed completion: ", first_n, " reply, status ", first, " after ", took_us,
          " us; then ", after_n, " replies, ", ok, " spi_ok", crlf);
    bench.verdict("A COMPLETION THAT NEVER COMES is answered spi_timeout in its place by the arbiter's per-bus "
                  "timeout, 99 to 102 ms after the request (100 ticks)",
                  first_n == 1u && first == spi_timeout && took_us >= 99'000u && took_us <= 102'000u);
    bench.verdict("and recover() puts the engine back where start() is legal: the next four transactions on the same "
                  "bus AO (three pumped, one polled) run to spi_ok",
                  after_n == 4u && ok == 4u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the wedge", command(Op::ping));
    pair_release();
}

// =============================================================================
// x - (outside z) the slip statistics
// =============================================================================

void tx_slips() {
    if (!need_peer()) {
        return;
    }
    constexpr uint16_t judged = judged_divisor();
    const uint16_t divs[2] = {128, judged};
    for (const uint16_t d : divs) {
        print(serial, "  /", d, " (", SysClock::pclk_hz / d / 1000u, " kHz), each mode ten times, 32 frames on the "
                      "engines: bursts not exact (first bad host/client)", crlf);
        uint8_t total = 0;
        for (uint8_t m = 0; m < 4u; ++m) {
            uint8_t slips = 0;
            print(serial, "  mode ", m, ":");
            for (uint8_t k = 0; k < 10u; ++k) {
                Exchange e{};
                e.cfg.mode = m;
                e.rate = divisor_clock(d);
                e.count = 32;
                e.engines = true;
                e.seed_a = static_cast<uint8_t>(0x13u + m + 7u * k);
                e.seed_b = static_cast<uint8_t>(0x57u + m + 11u * k);
                Verify v{};
                spilink::Report r{};
                if (!exchange_exact(e, v, r)) {
                    ++slips;
                    print(serial, " [", v.idx, "/", r.idx, r.count != e.count ? "!" : "", "]");
                }
            }
            print(serial, "  -> ", slips, " of 10", crlf);
            total = static_cast<uint8_t>(total + slips);
        }
        print(serial, "  ", total, " of 40 bursts slipped (a probe: no verdict)", crlf);
    }
    // THE ROLES INVERTED, the same count: this chip's client under the
    // peer's host at /32 and /16, each mode twenty times in 16-bit frames,
    // the bursts not exact and the bursts whose select windows were not the
    // frame signal's own count (a frame each in modes 0 and 2, one in 1 and
    // 3) printed, with the shortest high pulse the gauge saw.
    for (const uint16_t d : {uint16_t{32}, uint16_t{16}}) {
        print(serial, "  the roles inverted, /", d, " of the peer's clk_peri, 64 16-bit frames, each mode twenty "
                      "times:", crlf);
        for (uint8_t m = 0; m < 4u; ++m) {
            uint8_t bad = 0, odd = 0;
            uint32_t shortest = 0xFFFFFFFFu, pulses = 0;
            for (uint8_t k = 0; k < 20u; ++k) {
                const ClientRun c = client_burst(m, true, 64, d, static_cast<uint8_t>(0x15u + m + 3u * k),
                                                 static_cast<uint8_t>(0x6Bu + m + 5u * k));
                const bool ok = c.ran && c.got == 64u && c.mism == 0u && c.peer.count == 64u &&
                                c.peer.mism == 0u && !c.overrun;
                const uint16_t want = (m & 1u) == 0u ? 64u : 1u;
                if (!ok) {
                    ++bad;
                    print(serial, "    mode ", m, " run ", k, ":");
                    print_client(c, 64);
                    print(serial, crlf);
                }
                if (c.windows != want) {
                    ++odd;
                }
                if (c.pulses != 0u && c.pulse_min < shortest) {
                    shortest = c.pulse_min;
                }
                pulses += c.pulses;
            }
            print(serial, "  mode ", m, ": ", bad, " of 20 not exact, ", odd, " of 20 with another window count; ",
                  pulses, " high pulses inside the bursts, the shortest ",
                  shortest == 0xFFFFFFFFu ? 0u : 2u * shortest, " machine cycles at ", SysClock::hz / 1'000'000u,
                  " MHz", crlf);
        }
    }
    // WHOSE EDGE? Mode 3, 16-bit frames at /16, forty bursts with this
    // client's answer line driven and forty with it released (the dark
    // listener): what the client RECEIVES and the window count judged
    // alike. A count that falls with the answer line dark names this end's
    // own output as the aggressor.
    print(serial, "  mode 3, 16-bit, /16, forty bursts each: the client's answer line driven, then released", crlf);
    for (uint8_t dark = 0; dark < 2u; ++dark) {
        uint8_t bad = 0, odd = 0;
        for (uint8_t k = 0; k < 40u; ++k) {
            const ClientRun c = client_burst(3, true, 64, 16, static_cast<uint8_t>(0x25u + 3u * k),
                                             static_cast<uint8_t>(0x4Bu + 5u * k),
                                             dark != 0u ? Answer::released : Answer::driven);
            const bool ok = c.ran && c.got == 64u && c.mism == 0u && !c.overrun;
            bad = static_cast<uint8_t>(bad + (ok ? 0u : 1u));
            odd = static_cast<uint8_t>(odd + (c.windows == 1u ? 0u : 1u));
        }
        print(serial, dark != 0u ? "    released: " : "    driven:   ", bad, " of 40 received wrong, ", odd,
              " of 40 with another window count", crlf);
    }
    (void)link_command_mode();
    print(serial, "  the command channel afterwards: ", command(Op::ping) ? "answers" : "DEAD", crlf);
    pair_release();
}

// ---- the vectors --------------------------------------------------------------------------

[[gnu::always_inline]] inline void pair_spi_isr() {
    pair_isr_entries = pair_isr_entries + 1u;
    switch (pair_owner) {
        case PairOwner::host:
            if (LinkHost::isr()) {
                pair_status = LinkHost::status();
                pair_done = true;
            }
            break;
        case PairOwner::bus_ao:
            if (LinkHost::isr() && !pair_swallow) {
                post<pk::SpiArb>(TransferDone{LinkHost::status()});
            }
            break;
        case PairOwner::client: client_vector_serve(); break;
        case PairOwner::dma_host:
            if (LinkDmaHost::isr()) {
                pair_status = LinkDmaHost::status();
                pair_done = true;
            }
            break;
        default: Pl022<1>::interrupts(SpiInterrupt::all, false); break;
    }
}

[[gnu::always_inline]] inline void pair_dma_isr() {
    if (pair_owner == PairOwner::dma_host && LinkDmaHost::dma_isr()) {
        pair_status = LinkDmaHost::status();
        pair_done = true;
    }
}
