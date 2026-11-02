// rp_spi_async.hpp - LETTER f OF THE TWO RASPBERRY PI SPI SUITES, THE ENGINE A
// REQUEST NAMES, ON THE LOOP-BACK, written once over the ARM PL022
// (brio/pl022/spi.hpp) and included by test_rp2040_spi and test_rp2350_spi. A
// request names its engine and the host does what it names, so this letter
// judges the rules that adds to a host - the refusal on a host with no engines,
// SpiEngine::dma on the engines at the shortest length, what they cannot carry
// refused, the command rule - and MEASURES what each engine costs the core, E and
// F, for the guidance each family's page gives. AND LETTER t, WHEN A REQUEST ENDS
// THE WIRE HAS ENDED, on the SCK pad itself (below, above tt_wire_end). AND LETTER
// g, A POLLED WAIT THAT RUNS OUT, and LETTER h, A DMA BLOCK THE BUS REFUSES: what
// the host leaves after each and the next request with no recover() from the
// program, on the same host, on the loop-back (below, above tg_stall and th_fault).
//
// What the including suite states before it includes this file, inside its
// own anonymous namespace with `using namespace brio` in force:
//
//   SysClock, clock, serial, bench
//   Host, DmaHost        SpiHost<0, {18, 19, 16}> without engines, and the
//                        same with half-word engines reporting on DMA_IRQ_0
//   ByteDmaHost          the same with byte engines, for the refusal: it is
//                        brought up and asked, and no block ever runs on it
//   CsPin                the loop-back's select, GP17
//   DcPin                a D/C line, GP22
//   SckPad               the host's clock pad, GP18, read as an input
//   tx_buf, rx_buf       256 bytes each, aligned to a word
//   fill_pattern(), same(), host_ready(), xfer<H>()
//   spi0_owner           Spi0Owner::none / host / dma_host / wire_probe: whose
//                        vectors, the last the DMA line's for letter t
//   spi0_isr_entries, dma_isr_entries, transfer_done, transfer_status
//                        the two vectors' counts and the DMA line's flag,
//                        written by the suite's glue
//   wire_probe           what the DMA line's glue reads around dma_isr() for
//                        letter t (the suite's glue, wire_probe_vector() below)
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
// THE MEASUREMENT. E and F are taken through ONE CALL (xfer<DmaHost>), so the
// call's own cycles - the Request built, the counters cleared, the ruler's reads,
// the wait's turn - land in both and cancel from their difference: E, the engines'
// fixed cost, is a 32-frame block's wall less its 32 frames on the wire (the engines
// run at the wire, letter d); F, the polled loop's fixed part, is the intercept of
// two polled lengths, an 8-frame request's wall less 8 frames at the loop's own
// pace, that pace the difference of a 24- and an 8-frame request. Both at clk_peri /
// 4 in mode 3 (a byte frame 32 cycles, no frame gap), the best of fifteen in three
// rounds 3 ms apart (xfer_cycles()), the code warm. A read (an in buffer) and a
// write (none) each. And the two loops' TURNS, the frames' cost where the loop and
// not the wire sets the pace: the difference of a 48- and a 16-frame polled request
// at clk_peri / 2 in mode 3 (a byte frame 16 cycles), a read for the receive loop
// and a write for the transmit-only one (taken there on a run past the FIFO) -
// through the host without engines. They are printed and judged by nothing: nothing
// in the driver reads them, and the page's guidance is drawn from them.
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
// taken off. The counters themselves are one off at some call sites (a fetch between
// the two resets, or between the two reads, which hit and access counts see apart):
// a window with no request in it, through the very same instructions, reads that
// error, and it is taken off (xfer_cycles()).

#pragma once

static_assert(SysClock::pclk_hz == SysClock::hz, "clk_peri is clk_sys: a frame's cycles are bits x the divisor");

/// One request naming the engines on a host, and the way it ran: inside start()
/// or on them, the DMA line's entries, the status.
struct AsyncRun {
    bool inside = false;
    uint8_t status = 0xFE;
    uint32_t dma_entries = 0;
};

template <typename H>
AsyncRun async_once(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len, SpiMode mode,
                    SpiClock rate, SpiDataSize bits = SpiDataSize::bits8, PinRef dc = PinRef{}) {
    typename H::Request r{};
    r.cs = CsPin::ref();
    r.dc = dc;
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = mode;
    r.clock = rate;
    r.bits = bits;
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

/// E and F for a read or a write, and the polled loop's frame at clk_peri / 4
/// (the file header); 0 where a wall came out shorter than its frames.
struct Measured {
    uint32_t engines_fixed = 0;
    uint32_t polled_fixed = 0;
    uint32_t polled_frame_x10 = 0;
    uint32_t engined_misses = 0;
    uint32_t polled_misses = 0;
};

Measured measure_costs(bool read) {
    // A byte frame at clk_peri / 4 in mode 3: 32 SSPCLK cycles, and an SSPCLK
    // cycle is a core cycle on both suites' clocks.
    const uint32_t wire = 8u * SpiClocks::div4.divisor();
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
    return m;
}

/// A polled loop's frame at clk_peri / 2, in tenths of a cycle, on the host
/// without engines (the file header).
uint32_t measure_turn(bool read) {
    const uint32_t t16 = xfer_cycles<Host>(16, read, true, SpiClocks::div2);
    const uint32_t t48 = xfer_cycles<Host>(48, read, true, SpiClocks::div2);
    return t48 > t16 ? (t48 - t16) * 10u / 32u : 0u;
}

/// `n` bytes of the in buffer set to what no transfer here writes.
void untouched_rx(uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        rx_buf[i] = 0xEE;
    }
}
bool rx_untouched(uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (rx_buf[i] != 0xEEu) {
            return false;
        }
    }
    return true;
}

/// SSPSR with nothing on its way: both FIFOs empty, the block idle.
bool block_quiet() {
    return (Pl022<0>::flags() & (SpiFlag::tx_empty | SpiFlag::tx_not_full | SpiFlag::rx_not_empty | SpiFlag::busy)) ==
           (SpiFlag::tx_empty | SpiFlag::tx_not_full);
}

/// A dma request the engines cannot carry, asked with the select LOW and the D/C
/// HIGH, in a mode that would flip SCK's idle level: refused inside start(), with
/// spi_rejected, and nothing touched: a host that drove the select would leave it
/// high and one that drove the D/C for a command would leave it low, SSPCR0 still
/// says the mode before, no frame stands in a FIFO and the block is idle, and the in
/// buffer holds what it held.
template <typename H>
bool refused_whole(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len, const char* what) {
    H::prime(SpiMode::mode0, SpiClocks::div16);
    const uint32_t cr0 = Pl022<0>::regs().SSPCR0;
    untouched_rx(64);
    CsPin::clear();
    DcPin::set();
    const AsyncRun run =
        async_once<H>(cmd, cmd_len, tx, rx, len, SpiMode::mode3, SpiClocks::div2, SpiDataSize::bits16, DcPin::ref());
    const bool cs_low = !CsPin::read_out();
    const bool dc_high = DcPin::read_out();
    CsPin::set();
    const bool quiet = block_quiet();
    const bool word = Pl022<0>::regs().SSPCR0 == cr0;
    const bool buffer = rx_untouched(64);
    print(serial, "  ", what, ": inside start() ", run.inside, ", status ", run.status, ", the select ",
          cs_low ? "untouched" : "DRIVEN", ", the D/C ", dc_high ? "untouched" : "DRIVEN", ", the control word ",
          word ? "as it was" : "MOVED", ", the block ", quiet ? "quiet" : "NOT QUIET", ", ", run.dma_entries,
          " DMA interrupts", crlf);
    return run.inside && run.status == spi_rejected && cs_low && dc_high && word && quiet && buffer &&
           run.dma_entries == 0u;
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
    bool rx_clean = true;
    for (uint16_t i = 0; i < 16; ++i) {
        rx_clean = rx_clean && rx_buf[i] == 0xEEu;
    }
    print(serial, "  the engineless host, a request naming dma: inside start() ", refused.inside, ", status ",
          refused.status, ", the select ", select_untouched ? "untouched" : "DRIVEN", ", SSPCR0 ",
          hex(Pl022<0>::regs().SSPCR0), " (was ", hex(cr0), "), SSPSR ", hex(sr), ", SPI interrupts ",
          spi0_isr_entries, crlf);
    bench.verdict("A REQUEST NAMING dma ON A HOST WITH NO ENGINES is refused inside start(): spi_rejected, the "
                  "select never driven, the control word as it was (no CPOL flip for the request's mode 3), no "
                  "frame clocked (both FIFOs empty, the block idle), nothing written to the in buffer, no interrupt",
                  !Host::has_dma && refused.inside && refused.status == spi_rejected && select_untouched &&
                      Pl022<0>::regs().SSPCR0 == cr0 &&
                      (sr & (SpiFlag::tx_empty | SpiFlag::tx_not_full | SpiFlag::rx_not_empty | SpiFlag::busy)) ==
                          (SpiFlag::tx_empty | SpiFlag::tx_not_full) &&
                      rx_clean && spi0_isr_entries == 0u);
    const uint8_t polled_st = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3, SpiClocks::div2,
                                         SpiDataSize::bits8, true);
    bench.verdict("and the same request POLLED runs on that host, byte-exact",
                  polled_st == spi_ok && same(tx_buf, rx_buf, 16));
    const uint32_t read_turn_x10 = measure_turn(true);
    const uint32_t write_turn_x10 = measure_turn(false);
    Host::release();

    // ---- dma at the shortest length -------------------------------------------
    // One frame, read and written, at the fastest rate and at the next: 16 and 32
    // cycles of wire in mode 3, where the loop's turn and its fixed part cost the
    // core far less than the engines' launch and interrupt (the figures below), so
    // that nothing but the request's word can put it on them.
    spi0_owner = Spi0Owner::none;
    (void)DmaHost::init(clock);
    (void)CsPin::output(true);
    (void)DcPin::output(true);
    DmaHost::loopback(true);
    spi0_owner = Spi0Owner::dma_host;
    struct Shape {
        SpiClock rate;
        SpiMode mode;
        bool read;
        const char* name;
    };
    const Shape shapes[] = {{SpiClocks::div2, SpiMode::mode3, true, "a read at /2, mode 3"},
                            {SpiClocks::div2, SpiMode::mode3, false, "a write at /2, mode 3"},
                            {SpiClocks::div4, SpiMode::mode0, true, "a read at /4, mode 0"},
                            {SpiClocks::div4, SpiMode::mode0, false, "a write at /4, mode 0"}};
    uint8_t shortest_ok = 0;
    for (const Shape& s : shapes) {
        fill_pattern(tx_buf, 1, 0x31);
        rx_buf[0] = 0xEE;
        const AsyncRun run = async_once<DmaHost>(nullptr, 0, tx_buf, s.read ? rx_buf : nullptr, 1, s.mode, s.rate);
        const bool exact = !s.read || rx_buf[0] == tx_buf[0];
        print(serial, "  one frame, ", s.name, ": ", run.inside ? "INSIDE start()" : "on the engines", ", ",
              run.dma_entries, " DMA interrupts, status ", run.status, exact ? ", exact" : ", NOT EXACT", crlf);
        if (!run.inside && run.status == spi_ok && run.dma_entries == 1u && exact && CsPin::read_out()) {
            ++shortest_ok;
        }
    }
    bench.verdict("dma MEANS DMA AT THE SHORTEST LENGTH: one frame, read and written, at clk_peri / 2 and / 4, where the "
                  "loop would have cost the core less: start() answers false, dma_isr() is its end, one DMA interrupt, "
                  "the read exact, the select released",
                  shortest_ok == 4u);

    // ---- what the engines cannot carry ------------------------------------------
    fill_pattern(tx_buf, 64, 0x47);
    uint8_t refused_ok = 0;
    const auto refusal = [&refused_ok](const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                                       const char* what) {
        if (refused_whole<DmaHost>(cmd, cmd_len, tx, rx, len, what)) {
            ++refused_ok;
        }
    };
    refusal(nullptr, 0, tx_buf + 1, rx_buf, 16, "16-bit frames, the out buffer off a half-word");
    refusal(nullptr, 0, tx_buf, rx_buf + 1, 16, "16-bit frames, the in buffer off a half-word");
    refusal(nullptr, 0, tx_buf + 1, rx_buf + 1, 16, "16-bit frames, both off a half-word");
    refusal(nullptr, 0, tx_buf + 1, nullptr, 16, "16-bit frames, a write off a half-word");
    refusal(nullptr, 0, nullptr, rx_buf + 1, 16, "16-bit frames, a read off a half-word");
    refusal(tx_buf + 1, 4, nullptr, nullptr, 0, "16-bit frames, a command alone off a half-word");
    refusal(tx_buf, 2, tx_buf + 1, rx_buf, 16, "16-bit frames, a command and then data off a half-word");
    bench.verdict("WHAT THE ENGINES CANNOT CARRY IS REFUSED: 16-bit frames with a buffer off a half-word, the out buffer, "
                  "the in buffer, both, a write, a read, a command alone, a command ahead of such data: spi_rejected inside "
                  "start(), the select and the D/C never moved, the control word as it was, the block quiet and the in "
                  "buffer untouched, no interrupt",
                  refused_ok == 7u);
    fill_pattern(tx_buf, 32, 0x53);
    untouched_rx(32);
    const AsyncRun after_refusal =
        async_once<DmaHost>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3, SpiClocks::div16, SpiDataSize::bits16);
    print(serial, "  the next request, 16-bit frames on aligned buffers: status ", after_refusal.status, ", ",
          after_refusal.dma_entries, " DMA interrupts", same(tx_buf, rx_buf, 32) ? ", exact" : ", NOT EXACT", crlf);
    bench.verdict("and the next request is exact: 16-bit frames on aligned buffers ride the engines, one DMA interrupt",
                  !after_refusal.inside && after_refusal.status == spi_ok && after_refusal.dma_entries == 1u &&
                      same(tx_buf, rx_buf, 32));
    spi0_owner = Spi0Owner::none;
    DmaHost::release();

    // Byte engines carry 8-bit frames and refuse 16-bit ones, however aligned. No
    // block runs on this host, so its vector is nobody's.
    (void)ByteDmaHost::init(clock);
    (void)CsPin::output(true);
    ByteDmaHost::loopback(true);
    fill_pattern(tx_buf, 32, 0x5A);
    const bool byte_refused = refused_whole<ByteDmaHost>(nullptr, 0, tx_buf, rx_buf, 16, "16-bit frames on byte engines, aligned");
    bench.verdict("BYTE ENGINES refuse 16-bit frames the same way, on buffers however aligned", byte_refused);
    ByteDmaHost::release();

    // ---- the command rule -------------------------------------------------------
    // The command in the loop, the data on the engines: one DMA interrupt whatever
    // the command's length, the longest a Request's field carries among them, and
    // D/C high by the time start() answers false. A command with no data phase
    // rides the engines: start() false, one interrupt, D/C low until it ends and
    // high after it, where a polled command leaves it too. A command off a half-word
    // ahead of 16-bit data runs in the loop, a frame at a time, and the data rides.
    spi0_owner = Spi0Owner::none;
    (void)DmaHost::init(clock);
    (void)CsPin::output(true);
    DmaHost::loopback(true);
    spi0_owner = Spi0Owner::dma_host;
    fill_pattern(tx_buf, 200, 0x5D);
    fill_pattern(tx_buf + 200, 32, 0x6E);
    uint8_t command_ok = 0;
    const uint8_t command_lengths[] = {1u, 2u, 8u, 9u, 128u, 200u};
    for (const uint8_t cmd_len : command_lengths) {
        untouched_rx(32);
        (void)DcPin::output(true);
        const AsyncRun run = async_once<DmaHost>(tx_buf, cmd_len, tx_buf + 200, rx_buf, 32, SpiMode::mode3,
                                                 SpiClocks::div4, SpiDataSize::bits8, DcPin::ref());
        const bool dc_high = DcPin::read_out();
        const bool exact = same(tx_buf + 200, rx_buf, 32);
        print(serial, "  a command of ", cmd_len, " then 32 data frames at /4: ", run.inside ? "INSIDE start()" : "on the engines",
              ", ", run.dma_entries, " DMA interrupts, status ", run.status, ", D/C ", dc_high ? "high" : "LOW",
              exact ? ", exact" : ", NOT EXACT", crlf);
        if (!run.inside && run.status == spi_ok && run.dma_entries == 1u && dc_high && exact) {
            ++command_ok;
        }
    }
    bench.verdict("THE COMMAND RULE: a command then data, at any length of command, the command in the loop and the data "
                  "on the engines: start() false, ONE DMA interrupt, D/C high, the data exact, the command's echoes "
                  "discarded",
                  command_ok == 6u);
    // The command alone, its frames long enough to be seen on their way: D/C low
    // when start() answers false, with the block still shifting.
    (void)DcPin::output(true);
    DmaHost::prime(SpiMode::mode3, SpiClocks::div64);
    DcPin::set();
    transfer_done = false;
    dma_isr_entries = 0;
    DmaHost::Request alone{};
    alone.cs = CsPin::ref();
    alone.dc = DcPin::ref();
    alone.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    alone.cmd_len = 16;
    alone.mode = SpiMode::mode3;
    alone.clock = SpiClocks::div64;
    alone.engine = SpiEngine::dma;
    const bool alone_inside = DmaHost::start(alone);
    const bool alone_dc_during = DcPin::read_out();
    const bool alone_cs_during = CsPin::read_out();
    const uint32_t alone_t0 = us_now();
    while (!transfer_done && us_now() - alone_t0 < 200'000u) {
    }
    print(serial, "  a command of 16 frames alone at /64: ", alone_inside ? "INSIDE start()" : "on the engines", ", D/C on its way ",
          alone_dc_during ? "HIGH" : "low", ", the select ", alone_cs_during ? "HIGH" : "low", ", ", dma_isr_entries,
          " DMA interrupts, status ", transfer_status, ", D/C after it ", DcPin::read_out() ? "high" : "LOW", ", the select ",
          CsPin::read_out() ? "high" : "LOW", crlf);
    bench.verdict("A COMMAND ALONE rides the engines as the whole request: start() false, D/C low and the select low on "
                  "its way, one DMA interrupt, then both high",
                  !alone_inside && transfer_done && transfer_status == spi_ok && dma_isr_entries == 1u && !alone_dc_during &&
                      !alone_cs_during && DcPin::read_out() && CsPin::read_out());
    // A command at no alignment in particular ahead of 16-bit data.
    fill_pattern(tx_buf, 64, 0x71);
    untouched_rx(32);
    const AsyncRun odd_command = async_once<DmaHost>(tx_buf + 33, 2, tx_buf, rx_buf, 16, SpiMode::mode3, SpiClocks::div16,
                                                     SpiDataSize::bits16, DcPin::ref());
    print(serial, "  a command off a half-word ahead of 16-bit data: ", odd_command.inside ? "INSIDE start()" : "on the engines",
          ", ", odd_command.dma_entries, " DMA interrupts, status ", odd_command.status,
          same(tx_buf, rx_buf, 32) ? ", exact" : ", NOT EXACT", crlf);
    bench.verdict("A COMMAND OFF A HALF-WORD ahead of 16-bit data is carried: the command in the loop, the data on the "
                  "engines, exact",
                  !odd_command.inside && odd_command.status == spi_ok && odd_command.dma_entries == 1u &&
                      same(tx_buf, rx_buf, 32));
    (void)DcPin::release();

    // ---- what each engine costs the core --------------------------------------
    const Measured rd = measure_costs(true);
    const Measured wr = measure_costs(false);
    print(serial, "  measured (clk_sys cycles), a read:  E ", rd.engines_fixed, ", F ", rd.polled_fixed,
          ", the loop's frame at /4 ", rd.polled_frame_x10 / 10u, ".", rd.polled_frame_x10 % 10u, ", its turn at /2 ",
          read_turn_x10 / 10u, ".", read_turn_x10 % 10u, "; XIP misses E ", rd.engined_misses, ", F ", rd.polled_misses,
          crlf);
    print(serial, "                      a write: E ", wr.engines_fixed, ", F ", wr.polled_fixed,
          ", the loop's frame at /4 ", wr.polled_frame_x10 / 10u, ".", wr.polled_frame_x10 % 10u, ", its turn at /2 ",
          write_turn_x10 / 10u, ".", write_turn_x10 % 10u, "; XIP misses E ", wr.engined_misses, ", F ", wr.polled_misses,
          crlf);
    spi0_owner = Spi0Owner::none;
    DmaHost::release();
}

// t - WHEN A REQUEST ENDS, THE WIRE HAS ENDED, ON THE SCK PAD ITSELF. The rule: the
// select rises after the last SCK edge, and what tells a caller the request is over
// comes after that edge too, on both engines, also for a caller framing the select
// itself. Right after the end - start() answering true, or dma_isr() answering true,
// in the handler - the SCK pad's input is read: it must already be at the mode's
// idle level.
//
// THE INSTRUMENT AND ITS BOUND. What an early end leaves on the wire in modes 0 and 2
// is the rest of the last bit: half a bit, the divisor / 2 core cycles on these clocks
// (clk_peri is clk_sys), SCK off its idle level throughout. The pad's read comes
// some tens of core cycles after the loop's last flag, some hundreds after the
// channel's completion (an interrupt's entry, the flash), through the input's
// synchronizer, two or three cycles more. So an early end cannot be missed while half
// a bit outlasts that, and the rungs are chosen from the slowest down: at clk_peri /
// 65024 half a bit is 32512 cycles, a hundred times anything between the end and the
// read; at / 1024 it is 512. Below that the instrument goes blind by degrees, and the
// letter prints each rung's count so that where it goes blind can be read off the code
// without the wait: the document states it. Every rung is judged, since with the wait
// in the driver no read may find SCK off its idle level on any of them.
//
// The pad is shown to be read, every mode: at its idle level after prime(), both levels
// seen in the middle of a frame. The loop-back is off: this is the wire's letter, and
// SPI0's pads carry no wire. The in buffer reads the receive pad's pull-up.
//
// A COMMAND PHASE ENDS THE SAME WAY, which is what puts D/C right at the boundary:
// D/C rises after the polled loop's last wait and nothing else lies between, so a
// polled command with no data, read at its end, judges the same code. And start()'s
// own wall says it on the way to the engines: a command frame then data on them, at
// the slowest rung: an early end would hand the D/C and the launch over half a bit
// before the frame's last edge, 32512 cycles short.
//
// WHAT THE WAIT COSTS: at each rung the handler's own cycles around dma_isr(), the
// least of four, read and printed for modes 0 and 1: where it stops falling, vector
// entry has covered the wait, which is one read of SSPSR from that rung up.

/// One rung of the ladder, with its name for the print.
struct WireRung {
    SpiClock rate;
    uint16_t divisor_shown;
};
constexpr WireRung wire_rungs[] = {{SpiClock{254, 255}, 65024}, {SpiClock{16, 255}, 4096}, {SpiClock{4, 255}, 1024},
                                   {SpiClocks::div256, 256},    {SpiClocks::div128, 128},  {SpiClocks::div64, 64},
                                   {SpiClocks::div32, 32},      {SpiClocks::div16, 16},    {SpiClocks::div8, 8},
                                   {SpiClocks::div4, 4},        {SpiClocks::div2, 2}};
constexpr uint8_t wire_rung_count = sizeof(wire_rungs) / sizeof(wire_rungs[0]);

/// A spin bound for the two figures read after an end, in turns: one slowest frame's
/// worth, which only a block that stopped clocking spends.
constexpr uint32_t wire_spin_bound = 1'000'000u;

/// The end of one request as the pad and the block read it.
struct WireEnd {
    bool inside = false;
    uint8_t status = 0xFE;
    bool sck = false;              ///< the pad, right after the end
    bool busy = false;             ///< SSPSR.BSY, right after that
    uint32_t to_idle = 0;          ///< cycles from the end until the pad read idle
    uint32_t to_clear = 0;         ///< cycles from the end until BSY read clear
    uint32_t vector_cycles = 0;    ///< the handler's, around dma_isr()
    bool select_high = false;
};

/// What the DMA line's glue leaves for the letter when it owns the vector.
volatile bool probe_idle_level = false;
WireEnd wire_probe{};

/// The two figures, read right after an end at the idle level asked: the pad's way
/// there and BSY's. Zero, one read each, when the wire has ended.
void read_end(WireEnd& e, bool idle) {
    e.sck = SckPad::read();
    e.busy = Pl022<0>::busy();
    const uint32_t t0 = cycles_now();
    for (uint32_t n = wire_spin_bound; SckPad::read() != idle && n != 0u; --n) {
    }
    const uint32_t t1 = cycles_now();
    for (uint32_t n = wire_spin_bound; Pl022<0>::busy() && n != 0u; --n) {
    }
    const uint32_t t2 = cycles_now();
    e.to_idle = t1 - t0;
    e.to_clear = t2 - t0;
}

/// The DMA line's vector while letter t owns it: dma_isr() timed, the pad read right
/// after it answered true and before anything else.
[[gnu::always_inline]] inline void wire_probe_vector() {
    const uint32_t t0 = cycles_now();
    const bool ended = DmaHost::dma_isr();
    const uint32_t t1 = cycles_now();
    if (ended) {
        wire_probe.vector_cycles = t1 - t0;
        read_end(wire_probe, probe_idle_level);
        transfer_status = DmaHost::status();
        transfer_done = true;
    }
}

/// One request, ended, in a window on the program's own select when `framed`: its
/// end read in the handler on the engines and right after start() in the loop.
WireEnd wire_once(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len, SpiClock rate,
                  SpiMode mode, bool polled, bool framed, uint32_t* start_cycles = nullptr) {
    DmaHost::Request r{};
    r.cs = framed ? PinRef{} : CsPin::ref();
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = mode;
    r.clock = rate;
    r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
    const bool idle = spi_mode_cpol(mode);
    DmaHost::prime(mode, rate);
    if (framed) {
        CsPin::clear();
    }
    probe_idle_level = idle;
    transfer_done = false;
    wire_probe = WireEnd{};
    const uint32_t c0 = cycles_now();
    const bool inside = DmaHost::start(r);
    const uint32_t c1 = cycles_now();
    if (start_cycles != nullptr) {
        *start_cycles = c1 - c0;
    }
    WireEnd e{};
    if (inside) {
        read_end(e, idle);
        e.status = DmaHost::status();
    } else {
        const uint32_t t0 = us_now();
        while (!transfer_done && us_now() - t0 < 200'000u) {
        }
        e = wire_probe;
        e.status = transfer_done ? transfer_status : 0xFEu;
    }
    e.inside = inside;
    e.select_high = CsPin::read_out();
    if (framed) {
        // The select the program framed stays where it put it.
        e.select_high = !e.select_high;
        CsPin::set();
    }
    return e;
}

void tt_wire_end() {
    spi0_owner = Spi0Owner::none;
    bus_ao_live = false;
    (void)DmaHost::init(clock);
    (void)CsPin::output(true);
    spi0_owner = Spi0Owner::wire_probe;
    fill_pattern(tx_buf, 16, 0x39);

    // ---- the pad is read -------------------------------------------------------
    uint8_t idle_seen = 0;
    uint8_t both_seen = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        const SpiMode mode = static_cast<SpiMode>(m);
        DmaHost::prime(mode, SpiClock{16, 255});
        const bool idle = spi_mode_cpol(mode);
        if (SckPad::read() == idle) {
            ++idle_seen;
        }
        // Two frames at this rung, sampled from main while they run: thousands of
        // samples a bit period, so neither level can be missed.
        DmaHost::Request r{};
        r.cs = CsPin::ref();
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        r.len = 2;
        r.mode = mode;
        r.clock = SpiClock{16, 255};
        r.engine = SpiEngine::dma;
        transfer_done = false;
        bool seen_high = false;
        bool seen_low = false;
        if (!DmaHost::start(r)) {
            const uint32_t t0 = us_now();
            while (!transfer_done && us_now() - t0 < 200'000u) {
                if (SckPad::read()) {
                    seen_high = true;
                } else {
                    seen_low = true;
                }
            }
        }
        if (seen_high && seen_low) {
            ++both_seen;
        }
    }
    print(serial, "  the SCK pad read: at the idle level after prime() in ", idle_seen, " of 4 modes, both levels seen "
          "inside a frame in ", both_seen, " of 4", crlf);
    bench.verdict("THE INSTRUMENT: the SCK pad reads the mode's idle level after prime(), and both levels inside a frame, "
                  "in the four modes",
                  idle_seen == 4u && both_seen == 4u);

    // ---- the ladder --------------------------------------------------------------
    // Four modes x read and write x the request's select and the program's, on each
    // engine; a frame pair a run, so that the frame gap of modes 0 and 2 stands too.
    uint32_t early[2] = {};
    uint32_t bad_status[2] = {};
    uint32_t wrong_way[2] = {};
    uint32_t select_wrong[2] = {};
    for (uint8_t k = 0; k < wire_rung_count; ++k) {
        const WireRung& g = wire_rungs[k];
        uint8_t rung_early[2][4] = {};
        uint32_t idle_max[2][4] = {};
        uint32_t clear_max[2][4] = {};
        for (uint8_t way = 0; way < 2u; ++way) {
            const bool polled = way == 0u;
            for (uint8_t m = 0; m < 4u; ++m) {
                const SpiMode mode = static_cast<SpiMode>(m);
                for (uint8_t shape = 0; shape < 4u; ++shape) {
                    const bool read = (shape & 1u) == 0u;
                    const bool framed = (shape & 2u) != 0u;
                    for (uint8_t i = 0; i < 4u; ++i) {
                        rx_buf[i] = 0xEE;
                    }
                    const WireEnd e = wire_once(nullptr, 0, tx_buf, read ? rx_buf : nullptr, 2, g.rate, mode, polled, framed);
                    if (e.sck != spi_mode_cpol(mode)) {
                        ++early[way];
                        ++rung_early[way][m];
                    }
                    if (e.status != spi_ok || (read && (rx_buf[0] != 0xFFu || rx_buf[1] != 0xFFu))) {
                        ++bad_status[way];
                    }
                    if (e.inside != polled) {
                        ++wrong_way[way];
                    }
                    if (!e.select_high) {
                        ++select_wrong[way];
                    }
                    idle_max[way][m] = e.to_idle > idle_max[way][m] ? e.to_idle : idle_max[way][m];
                    clear_max[way][m] = e.to_clear > clear_max[way][m] ? e.to_clear : clear_max[way][m];
                }
            }
        }
        print(serial, "  /", g.divisor_shown, ": ends with SCK off its idle level, polled ", rung_early[0][0], " ",
              rung_early[0][1], " ", rung_early[0][2], " ", rung_early[0][3], ", dma ", rung_early[1][0], " ",
              rung_early[1][1], " ", rung_early[1][2], " ", rung_early[1][3], " of 4 by mode; the pad idle / BSY clear after "
              "the end at most, polled mode 0 ",
              idle_max[0][0], " / ", clear_max[0][0], ", mode 1 ", idle_max[0][1], " / ", clear_max[0][1], ", dma mode 0 ",
              idle_max[1][0], " / ", clear_max[1][0], ", mode 1 ", idle_max[1][1], " / ", clear_max[1][1], " cycles", crlf);
    }
    print(serial, "  over the ladder: early ends polled ", early[0], ", dma ", early[1], "; bad status or data ",
          bad_status[0], " / ", bad_status[1], "; the other way ", wrong_way[0], " / ", wrong_way[1], "; select wrong ",
          select_wrong[0], " / ", select_wrong[1], crlf);
    bench.verdict("THE POLLED LOOP'S END IS THE WIRE'S: right after start() answers true SCK is at its idle level, every "
                  "mode, read and write, the request's select and the program's, every rung from clk_peri / 65024 to / 2",
                  early[0] == 0u && bad_status[0] == 0u && wrong_way[0] == 0u);
    bench.verdict("THE ENGINES' END IS THE WIRE'S: right after dma_isr() answers true, in the handler, SCK is at its idle "
                  "level, the same combinations",
                  early[1] == 0u && bad_status[1] == 0u && wrong_way[1] == 0u);
    bench.verdict("the request's select is high at its end on both engines, and the program's still low until it raises it",
                  select_wrong[0] == 0u && select_wrong[1] == 0u);

    // ---- a command ends the same way --------------------------------------------
    uint8_t command_ends = 0;
    uint32_t polled_wall[4] = {};
    uint32_t onto_engines_wall[4] = {};
    const uint32_t frame_cycles = 8u * SpiClock{254, 255}.divisor();
    for (uint8_t m = 0; m < 4u; ++m) {
        const SpiMode mode = static_cast<SpiMode>(m);
        const WireEnd alone = wire_once(tx_buf, 1, nullptr, nullptr, 0, SpiClock{254, 255}, mode, true, false, &polled_wall[m]);
        const WireEnd ahead =
            wire_once(tx_buf, 1, tx_buf, nullptr, 1, SpiClock{254, 255}, mode, false, false, &onto_engines_wall[m]);
        // A frame's last edge comes eight periods after the write that starts it with
        // SPH = 1, eight and a half with SPH = 0, whose clock starts half a period in
        // (measured: this letter's walls, polled, on both chips). A wall of less is an
        // end ahead of that edge.
        const uint32_t floor = spi_mode_cpha(mode) ? frame_cycles : frame_cycles + frame_cycles / 16u;
        if (alone.sck == spi_mode_cpol(mode) && alone.status == spi_ok && ahead.status == spi_ok && !ahead.inside &&
            polled_wall[m] >= floor && onto_engines_wall[m] >= floor) {
            ++command_ends;
        }
    }
    print(serial, "  one command frame at clk_peri / 65024 (", frame_cycles, " cycles of eight periods): start()'s wall, polled "
          "alone ", polled_wall[0], " ", polled_wall[1], " ", polled_wall[2], " ", polled_wall[3], "; ahead of data on the "
          "engines ", onto_engines_wall[0], " ", onto_engines_wall[1], " ", onto_engines_wall[2], " ", onto_engines_wall[3],
          " by mode", crlf);
    bench.verdict("A COMMAND'S END IS THE WIRE'S, so D/C flips between the phases on the wire: at clk_peri / 65024 a polled "
                  "command alone ends with SCK idle, and start() on the way to the engines lasts the whole frame",
                  command_ends == 4u);

    // ---- what the wait costs --------------------------------------------------------
    for (uint8_t k = 0; k < wire_rung_count; ++k) {
        const WireRung& g = wire_rungs[k];
        uint32_t least[2] = {0xFFFF'FFFFu, 0xFFFF'FFFFu};
        for (uint8_t way = 0; way < 2u; ++way) {
            for (uint8_t i = 0; i < 4u; ++i) {
                const WireEnd e = wire_once(nullptr, 0, tx_buf, rx_buf, 1, g.rate, way == 0u ? SpiMode::mode0 : SpiMode::mode1,
                                            false, false);
                least[way] = e.vector_cycles < least[way] ? e.vector_cycles : least[way];
            }
        }
        print(serial, "  /", g.divisor_shown, ": the handler's cycles around dma_isr(), the least of four, mode 0 ", least[0],
              ", mode 1 ", least[1], crlf);
    }
    spi0_owner = Spi0Owner::none;
    DmaHost::release();
    (void)CsPin::output(true);
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

/// The next request after a stall or a fault, polled or on the engines, and
/// whether it was exact: `dma_entries` of one says it ran on them.
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

    // A command phase polled ahead of an engined data block: this request runs its
    // command through the receive shape, whose budget is the command's one frame
    // and one more, 400 000 turns.
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
