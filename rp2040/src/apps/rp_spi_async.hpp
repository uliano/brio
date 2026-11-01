// rp_spi_async.hpp - LETTER f OF THE TWO RASPBERRY PI SPI SUITES, THE
// ASYNCHRONOUS WAY ON THE LOOP-BACK, written once over the ARM PL022
// (brio/pl022/spi.hpp) and included by test_rp2040_spi and test_rp2350_spi:
// an asynchronous request is the DMA engines' (docs/design/spi-bus.md, "Two
// ways to run a request"), so this letter judges the three things that
// rule adds to a host - the refusal on a host with no engines, the short
// request run polled inside start() at the edge the host's count draws, and
// a command phase that repays a block of its own chained to its data block
// from the vector - and MEASURES the figures the count is drawn from (the
// chip traits' Pl022Costs), E - F as one difference through one call.
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
// through the host WITHOUT engines, where a polled read cannot ride them.

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
    r.polled = false;
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
/// rounds of five, 3 ms apart, each after one run thrown away. Spread in
/// time on purpose: one run of the letter in about thirty read every
/// engined block of a round some 185 cycles long, the next round clean -
/// a state of a few milliseconds this measurement has not explained, and
/// does not judge.
template <typename H = DmaHost>
uint32_t xfer_cycles(uint16_t frames, bool read, bool polled, SpiClock rate = SpiClocks::div4) {
    uint32_t least = 0xFFFF'FFFFu;
    for (uint8_t round = 0; round < 3u; ++round) {
        const uint32_t pause = us_now();
        while (us_now() - pause < 3000u) {
        }
        for (uint8_t k = 0; k < 6u; ++k) {
            const uint32_t t0 = cycles_now();
            (void)xfer<H>(nullptr, 0, tx_buf, read ? rx_buf : nullptr, frames, SpiMode::mode3, rate,
                          SpiDataSize::bits8, polled);
            const uint32_t took = cycles_now() - t0;
            if (k != 0u && took < least) {
                least = took;
            }
        }
    }
    return least;
}

/// E - F for a read or a write, and the polled loop's frame at clk_peri / 4
/// (the file header); 0 where a wall came out shorter than its frames.
struct Measured {
    int32_t e_less_f = 0;
    uint32_t engines_fixed = 0;
    uint32_t polled_fixed = 0;
    uint32_t polled_frame_x10 = 0;
};

Measured measure_costs(bool read) {
    // A byte frame at clk_peri / 4 in mode 3: 32 SSPCLK cycles, scaled to
    // the core's (clk_peri is clk_sys on both suites' clocks).
    const uint32_t wire = pl022_frame_cycles(pl022_core_per_sspclk_q8(SysClock::hz, SysClock::pclk_hz),
                                             SpiClocks::div4, SpiMode::mode3, SpiDataSize::bits8);
    const uint32_t engined = xfer_cycles(32, read, false);
    const uint32_t p8 = xfer_cycles(8, read, true);
    const uint32_t p24 = xfer_cycles(24, read, true);
    Measured m{};
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
          read_turn_x10 % 10u, " (the host's ", DmaHost::costs.read_turn, ")", crlf);
    print(serial, "                      a write: E ", wr.engines_fixed, ", F ", wr.polled_fixed, ", E - F ",
          wr.e_less_f, " (the host's ", DmaHost::async_write_budget_cycles, "), the loop's frame at /4 ",
          wr.polled_frame_x10 / 10u, ".", wr.polled_frame_x10 % 10u, ", its turn at /2 ", write_turn_x10 / 10u, ".",
          write_turn_x10 % 10u, " (the host's ", DmaHost::costs.write_turn, ")", crlf);
    bench.verdict("THE FIGURES THE COUNT IS DRAWN FROM, measured through one call: E - F for a read and for a "
                  "write, and the two polled loops' turns at clk_peri / 2, each within a quarter of the constant "
                  "the host carries (the chip traits' Pl022Costs)",
                  within_a_quarter(rd.e_less_f, DmaHost::async_read_budget_cycles) &&
                      within_a_quarter(wr.e_less_f, DmaHost::async_write_budget_cycles) &&
                      turn_within_a_quarter(read_turn_x10, DmaHost::costs.read_turn) &&
                      turn_within_a_quarter(write_turn_x10, DmaHost::costs.write_turn));
    spi0_owner = Spi0Owner::none;
    DmaHost::release();
}
