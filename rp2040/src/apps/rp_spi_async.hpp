// rp_spi_async.hpp - LETTER f OF THE TWO RASPBERRY PI SPI SUITES, THE
// ASYNCHRONOUS WAY ON THE LOOP-BACK, written once over the ARM PL022
// (brio/pl022/spi.hpp) and included by test_rp2040_spi and test_rp2350_spi:
// an asynchronous request is the DMA engines' (docs/design/spi-bus.md, "Two
// ways to run a request"), so this letter judges the three things that
// rule adds to a host - the refusal on a host with no engines, the short
// request run polled inside start() at the edge the host's count draws, and
// a command phase that repays a block of its own chained to its data block
// from the vector - and MEASURES the figures the count is drawn from (the
// chip traits' Pl022Costs), E - F as one difference through one call. AND
// LETTER g, A POLLED WAIT THAT RUNS OUT, and LETTER h, A DMA BLOCK THE BUS
// REFUSES: what the host leaves after each and the next request with no
// recover() from the program, on the same host, on the loop-back (below, above
// tg_stall and th_fault).
//
// What the including suite states before it includes this file, inside its
// own anonymous namespace with `using namespace brio` in force:
//
//   SysClock, clock, serial, bench
//   Host, DmaHost        SpiHost<0, {18, 19, 16}> without engines, and the
//                        same with half-word engines reporting on DMA_IRQ_0
//   CsPin                the loop-back's select, GP17
//   tx_buf, rx_buf       256 bytes each, aligned to a word
//   fill_pattern(), same(), host_ready(), xfer<H>()
//   spi0_owner           Spi0Owner::none / host / dma_host: whose vectors
//   spi0_isr_entries, dma_isr_entries, transfer_done, transfer_status
//                        the two vectors' counts and the DMA line's flag,
//                        written by the suite's glue
//   cycles_now()         a ruler of clk_sys cycles
//   us_now()             the microsecond ruler
//   Xip                  the XIP cache's counters
//   xip_line_cycles      what one miss adds to a wall, at the boot's XIP setting
//   dma_host_ready()     DmaHost brought up, its vector the DMA line's owner
//   bus_ao_live          the DMA line's glue posts TransferDone to kl::SpiArb
//   kl::SpiArb           the arbiter over DmaHost
//   bus_hole             an address the DMA's bus port gets a bus error from,
//                        reading and writing, from the chip's own memory map
//   sram_base, sram_end  the chip's SRAM, the read that runs off its end
//
// THE MEASUREMENT (Pl022Costs, pl022/spi.hpp's "THE SHORT ASYNCHRONOUS
// REQUEST"). E - F is what decides the count, and it is taken AS ONE
// DIFFERENCE through ONE CALL (xfer<DmaHost>), so the call's own cycles -
// the Request built, the counters cleared, the ruler's reads, the wait's
// turn - cancel: E, the engines' fixed cost, is a 32-frame asynchronous
// block's wall less its 32 frames on the wire (the engines run at the wire,
// letter d); F, the polled loop's fixed part, is an 8-frame polled request's
// wall less 8 frames at the loop's own pace, that pace the difference of a
// 24- and an 8-frame request. Both at clk_peri / 4 in mode 3 (a byte frame
// 32 cycles, no frame gap), the best of fifteen in three rounds 3 ms
// apart (xfer_cycles()), the code warm. A read (an in buffer) and a write (none) each. And the two
// loops' TURNS, the frames' cost where the loop and not the wire sets the
// pace: the difference of a 48- and a 16-frame polled request at clk_peri /
// 2 in mode 3 (a byte frame 16 cycles), a read for the receive loop and a
// write for the transmit-only one (taken there on a run past the FIFO) -
// through the host without engines.
//
// A WALL IS TAKEN NET OF ITS XIP MISSES, and the misses are printed beside E and
// F. The cache is two-way, so three lines of one transaction in one set miss on
// every pass and no warm-up cures them: the same host code then reads a wall
// longer by one line's fill a miss. Measured on the RP2040, the same host code in
// several layouts of this suite: E 1230 raw in one, its misses not counted, and net
// of 92 cycles a miss 765 to 771 over runs counting 2 to 10 misses. That is the
// image's and not the code's, and these figures describe the code, so a run's wall
// counts less `xip_line_cycles` a miss. Not every miss costs a wall: one run in
// thirteen on Hazard3 counted four misses on a wall of no extra length, E 246 for 510
// when the least NET wall was taken (a handler landing inside the wire's time would
// read so; which one it was is not established). So the run taken is the one with
// the FEWEST misses and of those the least wall, and only what every run pays is
// taken off. The
// tolerance the verdict judges them by stays a quarter of the constant. The
// counters themselves are one off at some call sites (a fetch between the two
// resets, or between the two reads, which hit and access counts see apart): a window
// with no request in it, through the very same instructions, reads that error, and
// it is taken off (xfer_cycles()).

#pragma once

/// One asynchronous request on a host, and the way it ran: inside start()
/// or on the engines, the DMA line's entries, the status.
struct AsyncRun {
    bool inside = false;
    uint8_t status = 0xFE;
    uint32_t dma_entries = 0;
};

template <typename H>
AsyncRun async_once(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len, SpiMode mode,
                    SpiClock rate) {
    typename H::Request r{};
    r.cs = CsPin::ref();
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = mode;
    r.clock = rate;
    r.bits = SpiDataSize::bits8;
    r.engine = SpiEngine::dma;
    transfer_done = false;
    spi0_isr_entries = 0;
    dma_isr_entries = 0;
    AsyncRun run{};
    run.inside = H::start(r);
    if (run.inside) {
        run.status = H::status();
    } else {
        const uint32_t t0 = us_now();
        while (!transfer_done && us_now() - t0 < 200'000u) {
        }
        run.status = transfer_done ? transfer_status : 0xFEu;
    }
    run.dma_entries = dma_isr_entries;
    return run;
}

/// The best wall of one request through xfer<H>, in clk_sys cycles: three
/// rounds of five, 3 ms apart, each after one run thrown away. The best is
/// the run with the fewest XIP misses and of those the least wall; the
/// misses go in `least_misses` and the wall comes back net of them (the
/// file header). A round's first two windows hold no request and the
/// second reads the counters' own error; the first request is thrown away.
/// Spread in time on purpose: one run of the letter in about
/// thirty read every engined block of a round some 185 cycles long, the next
/// round clean - a state of a few milliseconds this measurement does not judge.
inline uint32_t least_misses = 0;

template <typename H = DmaHost>
uint32_t xfer_cycles(uint16_t frames, bool read, bool polled, SpiClock rate = SpiClocks::div4) {
    uint32_t least = 0xFFFF'FFFFu;
    uint32_t fewest = 0xFFFF'FFFFu;
    for (uint8_t round = 0; round < 3u; ++round) {
        const uint32_t pause = us_now();
        while (us_now() - pause < 3000u) {
        }
        int32_t error = 0;
        for (uint8_t k = 0; k < 8u; ++k) {
            Xip::reset_counters();
            const uint32_t t0 = cycles_now();
            if (k >= 2u) {
                (void)xfer<H>(nullptr, 0, tx_buf, read ? rx_buf : nullptr, frames, SpiMode::mode3, rate,
                              SpiDataSize::bits8, polled);
            }
            const uint32_t took = cycles_now() - t0;
            const uint32_t hits = Xip::hits();
            const int32_t counted = static_cast<int32_t>(Xip::accesses() - hits);
            if (k < 2u) {
                error = counted;
                continue;
            }
            const uint32_t misses = counted > error ? static_cast<uint32_t>(counted - error) : 0u;
            if (k != 2u && (misses < fewest || (misses == fewest && took < least))) {
                least = took;
                fewest = misses;
            }
        }
    }
    least_misses = fewest;
    const uint32_t filled = fewest * xip_line_cycles;
    return least > filled ? least - filled : 0u;
}

/// E - F for a read or a write, and the polled loop's frame at clk_peri / 4
/// (the file header); 0 where a wall came out shorter than its frames.
struct Measured {
    int32_t e_less_f = 0;
    uint32_t engines_fixed = 0;
    uint32_t polled_fixed = 0;
    uint32_t polled_frame_x10 = 0;
    uint32_t engined_misses = 0;
    uint32_t polled_misses = 0;
};

Measured measure_costs(bool read) {
    // A byte frame at clk_peri / 4 in mode 3: 32 SSPCLK cycles, scaled to
    // the core's (clk_peri is clk_sys on both suites' clocks).
    const uint32_t wire = pl022_frame_cycles(pl022_core_per_sspclk_q8(SysClock::hz, SysClock::pclk_hz),
                                             SpiClocks::div4, SpiMode::mode3, SpiDataSize::bits8);
    const uint32_t engined = xfer_cycles(32, read, false);
    const uint32_t engined_misses = least_misses;
    const uint32_t p8 = xfer_cycles(8, read, true);
    const uint32_t polled_misses = least_misses;
    const uint32_t p24 = xfer_cycles(24, read, true);
    Measured m{};
    m.engined_misses = engined_misses;
    m.polled_misses = polled_misses;
    m.engines_fixed = engined > 32u * wire ? engined - 32u * wire : 0u;
    const uint32_t frame_x10 = p24 > p8 ? (p24 - p8) * 10u / 16u : 0u;
    m.polled_frame_x10 = frame_x10;
    m.polled_fixed = p8 * 10u > 8u * frame_x10 ? (p8 * 10u - 8u * frame_x10) / 10u : 0u;
    m.e_less_f = static_cast<int32_t>(m.engines_fixed) - static_cast<int32_t>(m.polled_fixed);
    return m;
}

/// A polled loop's frame at clk_peri / 2, in tenths of a cycle, on the host
/// without engines (the file header).
uint32_t measure_turn(bool read) {
    const uint32_t t16 = xfer_cycles<Host>(16, read, true, SpiClocks::div2);
    const uint32_t t48 = xfer_cycles<Host>(48, read, true, SpiClocks::div2);
    return t48 > t16 ? (t48 - t16) * 10u / 32u : 0u;
}

/// A measured E - F within a quarter of the constant the host carries: the
/// count it draws is then within a quarter of the run's own, and at the
/// edge either way of a request costs the core the same within that
/// fraction of E - F - the precision a choice between two ways of near cost
/// needs (an order of magnitude would do; a quarter catches a constant that
/// no longer describes the code).
bool within_a_quarter(int32_t measured, uint32_t constant) {
    if (measured <= 0) {
        return false;
    }
    const uint32_t m = static_cast<uint32_t>(measured);
    return 4u * m >= 3u * constant && 4u * m <= 5u * constant;
}

/// A loop's turn, measured in tenths, against the constant: the same
/// quarter, the turn entering a frame's cost where the wire is shorter.
bool turn_within_a_quarter(uint32_t measured_x10, uint32_t constant) {
    return 4u * measured_x10 >= 30u * constant && 4u * measured_x10 <= 50u * constant;
}

void tf_async() {
    // ---- the refusal ----------------------------------------------------------
    host_ready();
    Host::loopback(true);
    Host::prime(SpiMode::mode0, SpiClocks::div16);
    const uint32_t cr0 = Pl022<0>::regs().SSPCR0;
    fill_pattern(tx_buf, 16, 0x2B);
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0xEE;
    }
    // The select driven LOW first: a host that touched it would leave it
    // high (start() asserts it and releases it at the end).
    CsPin::clear();
    const AsyncRun refused = async_once<Host>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3, SpiClocks::div2);
    const bool select_untouched = !CsPin::read_out();
    CsPin::set();
    const uint32_t sr = Pl022<0>::flags();
    bool rx_untouched = true;
    for (uint16_t i = 0; i < 16; ++i) {
        rx_untouched = rx_untouched && rx_buf[i] == 0xEEu;
    }
    print(serial, "  the engineless host, an asynchronous request: inside start() ", refused.inside, ", status ",
          refused.status, ", the select ", select_untouched ? "untouched" : "DRIVEN", ", SSPCR0 ",
          hex(Pl022<0>::regs().SSPCR0), " (was ", hex(cr0), "), SSPSR ", hex(sr), ", SPI interrupts ",
          spi0_isr_entries, crlf);
    bench.verdict("AN ASYNCHRONOUS REQUEST ON A HOST WITH NO ENGINES is refused inside start(): spi_rejected, the "
                  "select never driven, the control word as it was (no CPOL flip for the request's mode 3), no "
                  "frame clocked (both FIFOs empty, the block idle), nothing written to the in buffer, no interrupt",
                  !Host::has_dma && refused.inside && refused.status == spi_rejected && select_untouched &&
                      Pl022<0>::regs().SSPCR0 == cr0 &&
                      (sr & (SpiFlag::tx_empty | SpiFlag::tx_not_full | SpiFlag::rx_not_empty | SpiFlag::busy)) ==
                          (SpiFlag::tx_empty | SpiFlag::tx_not_full) &&
                      rx_untouched && spi0_isr_entries == 0u);
    const uint8_t polled_st = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3, SpiClocks::div2,
                                         SpiDataSize::bits8, true);
    bench.verdict("and the same request POLLED runs on that host, byte-exact",
                  polled_st == spi_ok && same(tx_buf, rx_buf, 16));
    const uint32_t read_turn_x10 = measure_turn(true);
    const uint32_t write_turn_x10 = measure_turn(false);
    Host::release();

    // ---- the short request, at the host's count -------------------------------
    spi0_owner = Spi0Owner::none;
    (void)DmaHost::init(clock);
    (void)CsPin::output(true);
    DmaHost::loopback(true);
    spi0_owner = Spi0Owner::dma_host;
    struct Shape {
        SpiClock rate;
        SpiMode mode;
        bool read;
        const char* name;
    };
    const Shape shapes[] = {{SpiClocks::div2, SpiMode::mode3, true, "a read at /2, mode 3"},
                            {SpiClocks::div4, SpiMode::mode0, true, "a read at /4, mode 0"},
                            {SpiClocks::div2, SpiMode::mode3, false, "a write at /2, mode 3"},
                            {SpiClocks::div8, SpiMode::mode0, true, "a read at /8, mode 0"}};
    uint8_t split_ok = 0;
    for (const Shape& s : shapes) {
        const uint16_t n = DmaHost::async_min_frames(s.rate, s.mode, SpiDataSize::bits8, s.read);
        bool ok = n >= 1u && n <= 255u;
        AsyncRun below{};
        AsyncRun at{};
        bool exact_below = true;
        bool exact_at = true;
        if (ok) {
            for (uint8_t edge = 0; edge < 2u; ++edge) {
                const uint16_t k = static_cast<uint16_t>(edge == 0u ? n - 1u : n);
                fill_pattern(tx_buf, k, static_cast<uint8_t>(0x31u + 7u * edge + n));
                for (uint16_t i = 0; i < k; ++i) {
                    rx_buf[i] = 0xEE;
                }
                if (k == 0u) {
                    continue;   // a count of one has no shorter request
                }
                const AsyncRun run =
                    async_once<DmaHost>(nullptr, 0, tx_buf, s.read ? rx_buf : nullptr, k, s.mode, s.rate);
                const bool exact = !s.read || same(tx_buf, rx_buf, k);
                if (edge == 0u) {
                    below = run;
                    exact_below = exact;
                } else {
                    at = run;
                    exact_at = exact;
                }
            }
            const bool below_ok = n == 1u || (below.inside && below.status == spi_ok && below.dma_entries == 0u &&
                                              exact_below);
            const bool at_ok = !at.inside && at.status == spi_ok && at.dma_entries == 1u && exact_at;
            ok = below_ok && at_ok;
        }
        print(serial, "  ", s.name, ": the host's count ", n, " frames; ", n - 1u, " ran ",
              below.inside ? "inside start()" : "ON THE ENGINES", " (", below.dma_entries, " DMA interrupts), ", n,
              " ", at.inside ? "INSIDE start()" : "on the engines", " (", at.dma_entries, ")",
              exact_below && exact_at ? "" : ", NOT EXACT", crlf);
        if (ok) {
            ++split_ok;
        }
    }
    bench.verdict("THE SHORT ASYNCHRONOUS REQUEST, at four shapes (reads at /2, /4 and /8, a write at /2): one frame "
                  "under the host's count runs polled and completes inside start() with no DMA interrupt, the count "
                  "itself rides the engines with one, every read byte-exact",
                  split_ok == 4u);

    // ---- the command block ------------------------------------------------------
    // A command phase that repays a block of its own rides the engines first
    // and its completion launches the data block from the vector: two DMA
    // interrupts. A one-frame command runs polled inside start() ahead of
    // the data block: one.
    const uint16_t long_cmd = DmaHost::async_min_frames(SpiClocks::div2, SpiMode::mode3, SpiDataSize::bits8, false);
    const uint8_t cmd_len = static_cast<uint8_t>(long_cmd < 128u ? long_cmd : 128u);
    fill_pattern(tx_buf, 128, 0x5D);
    fill_pattern(tx_buf + 128, 32, 0x6E);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    const AsyncRun chained = async_once<DmaHost>(tx_buf, cmd_len, tx_buf + 128, rx_buf, 32, SpiMode::mode3,
                                                 SpiClocks::div2);
    const bool chained_exact = same(tx_buf + 128, rx_buf, 32);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    const AsyncRun single = async_once<DmaHost>(tx_buf, 1, tx_buf + 128, rx_buf, 32, SpiMode::mode3, SpiClocks::div2);
    const bool single_exact = same(tx_buf + 128, rx_buf, 32);
    print(serial, "  a command phase of ", cmd_len, " frames then 32 data frames at /2: ", chained.dma_entries,
          " DMA interrupts, status ", chained.status, chained_exact ? ", exact" : ", NOT EXACT",
          "; a command of 1 then the same 32: ", single.dma_entries, ", status ", single.status,
          single_exact ? ", exact" : ", NOT EXACT", crlf);
    bench.verdict("A COMMAND PHASE THAT REPAYS A BLOCK OF ITS OWN rides the engines before the data block, chained "
                  "from the vector (two DMA interrupts); a one-frame command runs polled ahead of the data block "
                  "(one); both data phases byte-exact, their command echoes discarded",
                  long_cmd <= 128u && !chained.inside && chained.status == spi_ok && chained.dma_entries == 2u &&
                      chained_exact && !single.inside && single.status == spi_ok && single.dma_entries == 1u &&
                      single_exact);

    // ---- the costs ----------------------------------------------------------------
    const Measured rd = measure_costs(true);
    const Measured wr = measure_costs(false);
    print(serial, "  measured (clk_sys cycles), a read:  E ", rd.engines_fixed, ", F ", rd.polled_fixed, ", E - F ",
          rd.e_less_f, " (the host's ", DmaHost::async_read_budget_cycles, "), the loop's frame at /4 ",
          rd.polled_frame_x10 / 10u, ".", rd.polled_frame_x10 % 10u, ", its turn at /2 ", read_turn_x10 / 10u, ".",
          read_turn_x10 % 10u, " (the host's ", DmaHost::costs.read_turn, "); XIP misses E ", rd.engined_misses,
          ", F ", rd.polled_misses, crlf);
    print(serial, "                      a write: E ", wr.engines_fixed, ", F ", wr.polled_fixed, ", E - F ",
          wr.e_less_f, " (the host's ", DmaHost::async_write_budget_cycles, "), the loop's frame at /4 ",
          wr.polled_frame_x10 / 10u, ".", wr.polled_frame_x10 % 10u, ", its turn at /2 ", write_turn_x10 / 10u, ".",
          write_turn_x10 % 10u, " (the host's ", DmaHost::costs.write_turn, "); XIP misses E ", wr.engined_misses,
          ", F ", wr.polled_misses, crlf);
    bench.verdict("THE FIGURES THE COUNT IS DRAWN FROM, measured through one call and net of the XIP misses printed "
                  "beside them: E - F for a read and for a write, and the two polled loops' turns at clk_peri / 2, "
                  "each within a quarter of the constant the host carries (the chip traits' Pl022Costs)",
                  within_a_quarter(rd.e_less_f, DmaHost::async_read_budget_cycles) &&
                      within_a_quarter(wr.e_less_f, DmaHost::async_write_budget_cycles) &&
                      turn_within_a_quarter(read_turn_x10, DmaHost::costs.read_turn) &&
                      turn_within_a_quarter(write_turn_x10, DmaHost::costs.write_turn));
    spi0_owner = Spi0Owner::none;
    DmaHost::release();
}

/// The frames just written all back in the receive FIFO, as far as `limit_us`
/// will wait: the shared wait of letters g and h, on the ruler, whose differences
/// do not mind its wrap. RNE with BSY clear: a frame has come back and none is
/// still shifting.
bool wait_frames_landed(uint32_t limit_us) {
    const uint32_t t0 = us_now();
    while (!Pl022<0>::rx_not_empty() || Pl022<0>::busy()) {
        if (us_now() - t0 >= limit_us) {
            return false;
        }
    }
    return true;
}

/// SSPSR as the block's reset leaves it: both FIFOs empty, the transmit one
/// not full, nothing shifting (the letter a verdict on the block's reset state).
bool fifos_clean() {
    return (Pl022<0>::flags() & (SpiFlag::tx_empty | SpiFlag::tx_not_full | SpiFlag::rx_not_empty | SpiFlag::busy)) ==
           (SpiFlag::tx_empty | SpiFlag::tx_not_full);
}

/// A stall staged with what a reset cures: at /2 in mode 0, on the loop-back, the
/// block's enable taken down under the host. The rate goes first so that a request
/// at the same word finds nothing to change and leaves the enable down. With
/// `stand` three frames are left in the receive FIFO first, for the transmit-only
/// shape, which reads nothing as it goes: the receive shape would take them for
/// its own echoes, and needs the FIFO empty to run out. False when the frames did
/// not stand, and then the staging is what failed.
template <typename H>
bool stage_stall(bool stand) {
    H::prime(SpiMode::mode0, SpiClocks::div2);
    bool landed = true;
    if (stand) {
        Pl022<0>::write_data(0x11u);
        Pl022<0>::write_data(0x22u);
        Pl022<0>::write_data(0x33u);
        // Three frames at /2 are under 1 us on these clocks: 1000 us is a wait
        // that only a dead block spends.
        landed = wait_frames_landed(1000u);
    }
    Pl022<0>::enable(false);
    return landed && Pl022<0>::rx_not_empty() == stand;
}

/// A polled write of nine frames at /2 on 8-bit frames, stalled: the one shape
/// that reads nothing as it goes, so what stands in the receive FIFO still
/// stands when its budget runs out and the transmit FIFO fills. The budget is
/// 200 000 turns a frame and one more, 2 000 000 turns.
template <typename H>
uint8_t stalled_write() {
    return xfer<H>(nullptr, 0, tx_buf, nullptr, 9, SpiMode::mode0, SpiClocks::div2, SpiDataSize::bits8, true);
}

/// `rx_buf` cleared and `frames` of a pattern in `tx_buf`: what the next request
/// must read back, in front of which a stale frame would show.
void next_pattern(uint16_t frames, uint8_t seed) {
    fill_pattern(tx_buf, frames, seed);
    for (uint16_t i = 0; i < frames; ++i) {
        rx_buf[i] = 0xEE;
    }
}

/// The next request after a stall or a fault, polled or asynchronous on the
/// engines, and whether it was exact. 64 frames at /16 repay the engines at
/// every count the host draws (letter f prints them), which `dma_entries` of
/// one confirms.
bool next_exact(bool asynchronous, uint8_t seed) {
    next_pattern(64, seed);
    if (asynchronous) {
        const AsyncRun run =
            async_once<DmaHost>(nullptr, 0, tx_buf, rx_buf, 64, SpiMode::mode3, SpiClocks::div16);
        return !run.inside && run.status == spi_ok && run.dma_entries == 1u && same(tx_buf, rx_buf, 64);
    }
    return xfer<DmaHost>(nullptr, 0, tx_buf, rx_buf, 64, SpiMode::mode3, SpiClocks::div16, SpiDataSize::bits8, true) ==
               spi_ok &&
           same(tx_buf, rx_buf, 64);
}

// THE QUEUE, letters g and h: one client posts two requests from one dispatch
// through the arbiter, which starts the second in the dispatch that answers the
// first, before the client has run - the case only the host itself can serve,
// since nothing of the program runs between them.
namespace queued {

struct Go {};

class Client {
public:
    using Event = std::variant<Go, SpiDone>;
    static inline EventQueue<Event, 6, P> queue;
    static inline DmaHost::Request first = [] { DmaHost::Request r{}; r.engine = SpiEngine::dma; return r; }();
    static inline DmaHost::Request second = [] { DmaHost::Request r{}; r.engine = SpiEngine::dma; return r; }();
    static inline uint8_t replies[2] = {0xFEu, 0xFEu};
    static inline uint8_t n = 0;

    static void init() { n = 0; }
    static void dispatch(const Event& e) {
        match(e,
              [](const Go&) {
                  post<kl::SpiArb>(first);
                  post<kl::SpiArb>(second);
              },
              [](const SpiDone& d) {
                  if (n < 2u) {
                      replies[n] = d.status;
                  }
                  ++n;
              });
    }
};

using Kernel = Tenuto<P, Client, kl::SpiArb>;

DmaHost::Request request(const uint8_t* tx, uint8_t* rx, uint16_t len, SpiMode mode, SpiClock rate, bool polled) {
    DmaHost::Request r{};
    r.cs = CsPin::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = mode;
    r.clock = rate;
    r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
    r.reply = reply_to<Client, SpiDone>();
    return r;
}

/// The two requests through the kernel, and both replies in, as far as 1 s will
/// wait: the first may be a stall of some 250 ms and the second has 64 frames
/// to go, so 1 s is what only a lost reply spends.
bool run(const DmaHost::Request& one, const DmaHost::Request& two) {
    Client::first = one;
    Client::second = two;
    Kernel::init_all();
    bus_ao_live = true;
    post<Client>(Go{});
    const uint32_t t0 = us_now();
    while (Client::n < 2u && us_now() - t0 < 1'000'000u) {
        (void)Kernel::step();
    }
    (void)Kernel::step();
    bus_ao_live = false;
    return Client::n == 2u;
}

}  // namespace queued

// g - A POLLED WAIT THAT RUNS OUT. No clock is stopped: the block's enable is
// taken down under the host (stage_stall()), so frames written to the transmit
// FIFO stay there and the loop's flag never rises, which is a stall on silicon
// with no clock tree touched, and one the block's reset cures. The budget is
// 2 000 000 turns at some ten cycles a turn: under 200 ms by a margin, printed.
// Judged, with no recover() anywhere in the letter: spi_stalled and never the
// engines' code, the select released (the request drove it low and it reads
// high), the block clean and enabled at the word applied - the frames that stood
// in the receive FIFO and those the request left in the transmit FIFO gone, by
// the reset - and then the next request exact, polled on the host with no engines,
// polled and asynchronous on the host with them. Every stall path: a command phase
// polled ahead of an engined data block too, which answers inside start(), with no
// block launched. And queued through the arbiter behind a stall, both ways.
void tg_stall() {
    host_ready();
    Host::loopback(true);
    const bool stood = stage_stall<Host>(true);
    fill_pattern(tx_buf, 16, 0x7C);
    const uint32_t t0 = us_now();
    const uint8_t st = stalled_write<Host>();
    const uint32_t took = us_now() - t0;
    const bool released = CsPin::read_out();
    const bool clean = fifos_clean();
    const bool enabled = Pl022<0>::enabled();
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st_after = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0, SpiClocks::div16,
                                        SpiDataSize::bits8, true);
    const bool exact_after = same(tx_buf, rx_buf, 16);
    print(serial, "  the enable down under a polled write of nine frames: three frames stood in the receive FIFO ",
          stood ? "yes" : "NO", ", status ", st, " in ", took, " us, select released ", released ? "yes" : "NO",
          ", both FIFOs clean ", clean ? "yes" : "NO", ", enabled ", enabled ? "yes" : "NO",
          "; no recover(), then 16 frames polled status ", st_after, exact_after ? " exact" : " NOT EXACT", crlf);
    bench.verdict("A POLLED WAIT THAT RUNS OUT answers spi_stalled, not the engines' code: the select released, the "
                  "block reset and enabled, the frames that stood in its two FIFOs gone",
                  stood && st == spi_stalled && st != spi_dma_fault && released && clean && enabled);
    bench.verdict("and the next polled request, with no recover(), is exact", st_after == spi_ok && exact_after);
    Host::release();

    // The same on the host with engines, the next request each way.
    dma_host_ready();
    DmaHost::loopback(true);
    bool both_ways = true;
    for (uint8_t way = 0; way < 2u; ++way) {
        const bool stood_w = stage_stall<DmaHost>(true);
        const uint8_t st_w = stalled_write<DmaHost>();
        const bool released_w = CsPin::read_out();
        const bool clean_w = fifos_clean();
        const bool exact_w = next_exact(way == 1u, static_cast<uint8_t>(0x13u + 0x16u * way));
        print(serial, "  on the host with engines: three frames stood ", stood_w ? "yes" : "NO", ", status ", st_w,
              ", select released ", released_w ? "yes" : "NO", ", both FIFOs clean ", clean_w ? "yes" : "NO",
              "; no recover(), the next request ", way == 1u ? "asynchronous " : "polled ",
              exact_w ? "exact" : "NOT EXACT", crlf);
        both_ways = both_ways && stood_w && st_w == spi_stalled && released_w && clean_w && exact_w;
    }
    bench.verdict("on the host with engines the same stall leaves the next request exact, polled and asynchronous, "
                  "with no recover()",
                  both_ways);

    // A command phase polled ahead of an engined data block: one command frame
    // does not repay a block of its own, and 64 frames at /2 repay the engines
    // (letter f's counts), so this request runs its command through the receive
    // shape, whose budget is the command's one frame and one more, 400 000 turns.
    const bool stood_c = stage_stall<DmaHost>(false);
    cmd_buf[0] = 0x5A;
    const AsyncRun cmd = async_once<DmaHost>(cmd_buf, 1, tx_buf, rx_buf, 64, SpiMode::mode0, SpiClocks::div2);
    const bool cmd_ok = stood_c && cmd.inside && cmd.status == spi_stalled && cmd.dma_entries == 0u &&
                        CsPin::read_out() && fifos_clean() && Pl022<0>::enabled();
    const bool cmd_next = next_exact(true, 0x47);
    print(serial, "  a command frame polled ahead of an engined block, the enable down: inside start() ",
          cmd.inside ? "yes" : "NO", ", status ", cmd.status, ", ", cmd.dma_entries, " DMA interrupts; the next "
          "request ", cmd_next ? "exact" : "NOT EXACT", crlf);
    bench.verdict("A COMMAND PHASE POLLED AHEAD OF AN ENGINED BLOCK that runs out takes the same path: spi_stalled "
                  "inside start(), no block launched, the block reset, the next asynchronous request exact",
                  cmd_ok && cmd_next);

    // Queued: the second request starts in the dispatch that answered the first.
    uint8_t queued_ok = 0;
    for (uint8_t way = 0; way < 2u; ++way) {
        next_pattern(64, static_cast<uint8_t>(0x61u + way));
        const bool stood_q = stage_stall<DmaHost>(true);
        const bool both = queued::run(
            queued::request(tx_buf + 128, nullptr, 9, SpiMode::mode0, SpiClocks::div2, true),
            queued::request(tx_buf, rx_buf, 64, SpiMode::mode3, SpiClocks::div16, way == 0u));
        const bool exact = same(tx_buf, rx_buf, 64);
        print(serial, "  queued through SpiBus behind a stall, the second ", way == 0u ? "polled" : "asynchronous",
              ": ", queued::Client::n, " replies [", queued::Client::replies[0], " ", queued::Client::replies[1], "]",
              exact ? ", the second's data exact" : ", the second's data NOT EXACT", crlf);
        if (stood_q && both && queued::Client::replies[0] == spi_stalled && queued::Client::replies[1] == spi_ok &&
            exact) {
            ++queued_ok;
        }
    }
    bench.verdict("QUEUED BEHIND A STALL through SpiBus, two requests from one dispatch: replies in order, spi_stalled "
                  "then spi_ok, the second's data exact polled and on the engines, no recover()",
                  queued_ok == 2u);
    spi0_owner = Spi0Owner::none;
    DmaHost::release();
}

// h - A DMA BLOCK THE BUS REFUSES. An asynchronous request whose buffer the bus
// refuses: a bus error is the DMA's own failure and never the core's, and it is
// what spi_dma_fault answers. Three stagings, each 64 frames at /128 so that a
// fault can find frames on their way. A hole for the out buffer: the transmit
// channel's first read fails, with nothing on its way. An out buffer running off
// the end of the SRAM, 16 frames short of it: a READ, which cannot land anywhere,
// failing with frames already written. A hole for the in buffer: the receive
// channel's write fails on the first frame back, the transmit channel having filled
// its FIFO. A frame at /128 is 1024 SSPCLK cycles and the eight standing are 8192
// of them, so in the last two the transmit FIFO is not yet empty when the program
// looks after the interrupt, which takes far less, and what the handler did not do
// shows: the frames stand, SSE up. Judged, with no recover() anywhere in the letter:
// dma_isr() answered true (transfer_done is set on that edge alone), spi_dma_fault,
// the select released; then the next request, polled or asynchronous, exact: no
// frame of the faulted block in front of it, which is what the reset owed by the
// interrupt and paid by that start() is for. Six rounds. And queued through the
// arbiter behind the read that runs off the end, both ways.
//
// A WRITE THE BUS REFUSES MUST ALSO LAND NOWHERE, which the error flag does not
// prove: the chapter says an address that fails to decode reaches no device, and
// the census (ram_census()) says it of the SRAM on this silicon. A write of a half-word
// nobody else writes, 32 frames of it, is staged at the hole, and every half-word
// of the SRAM holding that value is counted per kilobyte before and after: a block
// whose count grew is a landing, printed by address. Half-words and not bytes, because
// the stack and the live counters pass through any byte value at 1 in 256 of the time and
// through this one at 1 in 65536 a word.
//
// A WINDOW FRAMED BY HAND after a fault, on the loop-back: the select null in the
// Request, prime(), the select low, start(), the select high. Exact is not enough
// here, since the loop-back has no selected device: what a hand-framed window owes
// that device is a block already clean when the select falls, so SSPSR is read
// between prime() and the select, with frames on their way from the fault. Letter v
// of rp_spi_pair.hpp judges the same window through a device's own eyes.
struct FaultRun {
    AsyncRun run{};
    bool standing = false;
    bool enabled = false;
    bool released = false;
};

enum class BadBuffer : uint8_t { out_hole, out_end, in_hole };

/// `frames` frames ending at the SRAM's last byte and then the unmapped: where a
/// read off the end starts.
const uint8_t* sram_tail(uint16_t frames) { return reinterpret_cast<const uint8_t*>(sram_end - frames); }

FaultRun fault_once(BadBuffer bad) {
    const auto* out = bad == BadBuffer::out_hole  ? reinterpret_cast<const uint8_t*>(bus_hole)
                      : bad == BadBuffer::out_end ? sram_tail(16)
                                                  : tx_buf;
    uint8_t* in = bad == BadBuffer::in_hole ? reinterpret_cast<uint8_t*>(bus_hole) : rx_buf;
    fill_pattern(tx_buf, 64, 0xC3);
    FaultRun f{};
    f.run = async_once<DmaHost>(nullptr, 0, out, in, 64, SpiMode::mode3, SpiClocks::div128);
    f.standing = !Pl022<0>::tx_empty();
    f.enabled = Pl022<0>::enabled();
    f.released = CsPin::read_out();
    return f;
}

/// What the owed reset costs the request that pays it, in clk_sys cycles: a
/// polled 8-frame request's wall after a fault, less the same request's wall
/// right after it, the least of three - printed, judged by nothing.
uint32_t owed_reset_cycles() {
    uint32_t least = 0xFFFF'FFFFu;
    for (uint8_t k = 0; k < 3u; ++k) {
        (void)fault_once(BadBuffer::out_end);
        const uint32_t t0 = cycles_now();
        (void)xfer<DmaHost>(nullptr, 0, tx_buf, nullptr, 8, SpiMode::mode3, SpiClocks::div128, SpiDataSize::bits8, true);
        const uint32_t t1 = cycles_now();
        (void)xfer<DmaHost>(nullptr, 0, tx_buf, nullptr, 8, SpiMode::mode3, SpiClocks::div128, SpiDataSize::bits8, true);
        const uint32_t t2 = cycles_now();
        const uint32_t paid = t1 - t0;
        const uint32_t clean = t2 - t1;
        const uint32_t net = paid > clean ? paid - clean : 0u;
        least = net < least ? net : least;
    }
    return least;
}

/// The half-word only the census's staging writes, and the SRAM counted for it,
/// a count per kilobyte: 8 ms or so of reads, nothing else running but the tick.
constexpr uint16_t census_mark = 0x5BA7u;
constexpr uint32_t census_blocks = (sram_end - sram_base) / 1024u;
uint16_t census_before[census_blocks];

uint16_t census_count(uint32_t block) {
    const auto* p = reinterpret_cast<const volatile uint16_t*>(sram_base + block * 1024u);
    uint16_t n = 0;
    for (uint32_t i = 0; i < 512u; ++i) {
        n = static_cast<uint16_t>(n + (p[i] == census_mark ? 1u : 0u));
    }
    return n;
}

/// The write the bus refuses, and every SRAM block whose count of the mark grew
/// across it. The mark stands in `tx_buf` before the first count and stays there,
/// so it adds nothing. Answers how many blocks grew.
uint32_t ram_census() {
    for (uint16_t i = 0; i < 32u; ++i) {
        tx_buf[2u * i] = static_cast<uint8_t>(census_mark);
        tx_buf[2u * i + 1u] = static_cast<uint8_t>(census_mark >> 8);
    }
    for (uint32_t b = 0; b < census_blocks; ++b) {
        census_before[b] = census_count(b);
    }
    const uint8_t st = xfer<DmaHost>(nullptr, 0, tx_buf, reinterpret_cast<uint8_t*>(bus_hole), 32, SpiMode::mode3,
                                     SpiClocks::div128, SpiDataSize::bits16, false);
    uint32_t grown = 0;
    for (uint32_t b = 0; b < census_blocks; ++b) {
        const uint16_t now = census_count(b);
        if (now > census_before[b]) {
            ++grown;
            print(serial, "  the mark appeared in the SRAM's block at ", hex(sram_base + b * 1024u), ": ", now, " for ",
                  census_before[b], crlf);
        }
    }
    print(serial, "  a 16-bit write of the mark at ", hex(bus_hole), ": status ", st, ", ", census_blocks,
          " KB of SRAM counted before and after, ", grown, " blocks grew", crlf);
    return st == spi_dma_fault ? grown : 0xFFFF'FFFFu;
}

void th_fault() {
    dma_host_ready();
    DmaHost::loopback(true);
    struct Round {
        BadBuffer bad;
        const char* name;
        bool frames_on_their_way;
        bool next_async;
    };
    const Round rounds[] = {{BadBuffer::out_hole, "out buffer in the hole", false, false},
                            {BadBuffer::out_end, "out buffer off the SRAM's end", true, false},
                            {BadBuffer::in_hole, "in buffer in the hole", true, false},
                            {BadBuffer::out_hole, "out buffer in the hole", false, true},
                            {BadBuffer::out_end, "out buffer off the SRAM's end", true, true},
                            {BadBuffer::in_hole, "in buffer in the hole", true, true}};
    uint8_t rounds_ok = 0;
    uint8_t standing_ok = 0;
    uint8_t seed = 0x35u;
    for (const Round& r : rounds) {
        const FaultRun f = fault_once(r.bad);
        const bool edge = !f.run.inside && transfer_done && f.run.status == spi_dma_fault && f.released;
        const bool exact = next_exact(r.next_async, seed);
        seed = static_cast<uint8_t>(seed + 11u);
        print(serial, "  ", r.name, ": status ", f.run.status, ", ", f.run.dma_entries,
              " DMA interrupts, select released ", f.released ? "yes" : "NO", ", frames still in the transmit FIFO ",
              f.standing ? "yes" : "no", ", enabled ", f.enabled ? "yes" : "NO", "; no recover(), the next request ",
              r.next_async ? "asynchronous " : "polled ", exact ? "exact" : "NOT EXACT", crlf);
        if (edge && exact && f.enabled) {
            ++rounds_ok;
        }
        if (r.frames_on_their_way && f.standing) {
            ++standing_ok;
        }
    }
    bench.verdict("A DMA BLOCK THE BUS REFUSES, a read or a write: dma_isr() answers true, spi_dma_fault, the select "
                  "released, and the next request, polled or asynchronous, with no recover(), is exact, no frame of "
                  "the faulted block in front of it",
                  rounds_ok == 6u);
    bench.verdict("the interrupt decides and does not reset: after a fault with frames on their way the block is still "
                  "enabled with frames standing in its transmit FIFO, for the next start() to reset",
                  standing_ok == 4u);
    print(serial, "  the owed reset paid by a polled request: ", owed_reset_cycles(), " clk_sys cycles", crlf);

    // A window framed by hand, polled and asynchronous.
    uint8_t framed_ok = 0;
    for (uint8_t way = 0; way < 2u; ++way) {
        const FaultRun f = fault_once(BadBuffer::out_end);
        DmaHost::prime(SpiMode::mode3, SpiClocks::div16);
        const bool clean = fifos_clean();
        next_pattern(64, static_cast<uint8_t>(0xB3u + way));
        CsPin::clear();
        const uint8_t st = xfer<DmaHost>(nullptr, 0, tx_buf, rx_buf, 64, SpiMode::mode3, SpiClocks::div16,
                                         SpiDataSize::bits8, way == 0u, PinRef{});
        const bool held = !CsPin::read_out();
        CsPin::set();
        const bool exact = same(tx_buf, rx_buf, 64);
        print(serial, "  a window framed by hand after a fault, ", way == 0u ? "polled" : "asynchronous", ": frames stood ",
              f.standing ? "yes" : "NO", ", both FIFOs clean after prime() ", clean ? "yes" : "NO", ", status ", st,
              ", the select untouched ", held ? "yes" : "NO", exact ? ", exact" : ", NOT EXACT", crlf);
        if (f.run.status == spi_dma_fault && f.standing && clean && st == spi_ok && held && exact) {
            ++framed_ok;
        }
    }
    bench.verdict("A WINDOW FRAMED BY HAND AFTER A FAULT: prime() pays the reset, both FIFOs clean before the select "
                  "falls with frames on their way before it; the request's null select leaves the program's alone; "
                  "exact, polled and asynchronous, no recover()",
                  framed_ok == 2u);
    bench.verdict("A WRITE THE BUS REFUSES LANDS NOWHERE IN THE SRAM: spi_dma_fault, and no block of the SRAM holds the "
                  "written half-word once more than it did",
                  ram_census() == 0u);

    // Queued: the second request starts in the dispatch that answers the fault.
    // The fault is the read off the end: the frames on their way with nothing to land.
    uint8_t queued_ok = 0;
    for (uint8_t way = 0; way < 2u; ++way) {
        next_pattern(64, static_cast<uint8_t>(0x91u + way));
        const bool both = queued::run(
            queued::request(sram_tail(16), rx_buf + 128, 64, SpiMode::mode3, SpiClocks::div128, false),
            queued::request(tx_buf, rx_buf, 64, SpiMode::mode3, SpiClocks::div16, way == 0u));
        const bool exact = same(tx_buf, rx_buf, 64);
        print(serial, "  queued through SpiBus behind a fault, the second ", way == 0u ? "polled" : "asynchronous",
              ": ", queued::Client::n, " replies [", queued::Client::replies[0], " ", queued::Client::replies[1], "]",
              exact ? ", the second's data exact" : ", the second's data NOT EXACT", crlf);
        if (both && queued::Client::replies[0] == spi_dma_fault && queued::Client::replies[1] == spi_ok && exact) {
            ++queued_ok;
        }
    }
    bench.verdict("QUEUED BEHIND A FAULT through SpiBus, two requests from one dispatch: replies in order, "
                  "spi_dma_fault then spi_ok, the second's data exact polled and on the engines, no recover()",
                  queued_ok == 2u);
    spi0_owner = Spi0Owner::none;
    DmaHost::release();
}
