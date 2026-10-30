// rp_rubato_i2c.hpp - THE I2C SUITE UNDER RUBATO OF THE TWO RASPBERRY PI
// CHIPS, written once over the Synopsys DW_apb_i2c (brio/dw_apb_i2c/i2c.hpp)
// and bound by test_rp2040_rubato_i2c and test_rp2350_rubato_i2c: this
// chip's I2C host and its DMA engines under util/i2c_bus.hpp's arbiter, run
// inside Rubato, the preemptive kernel, against the desk partner's unchanged
// `twi_peer` (rp_twi_peer.hpp), while the pack's highest object preempts it
// at prime periods. Every verdict is the traffic judged byte-exact and every
// status the one the wire produced.
//
// TWO SHARED BODIES, ONE ABOVE THE OTHER. What names no register - the pack
// of five objects, the command channel through the arbiter, the passes,
// the letters n o p h i x - is every family's (avrxt/src/apps/rubato_i2c.hpp,
// whose header describes each object and each letter). What the two chips
// share because they carry the same IP block and the same peer - the host's
// paths and the shapes, this peer's arithmetic, the in-tenure stretch and
// its instrument, the RP versions of letters q and w, letter s - is this
// file's, as rp_i2c_pair.hpp is for the Tenuto suites: one register
// description, so one body, not a copy.
//
// What the binding states before it includes this file, inside its own
// anonymous namespace with `using namespace brio` in force, having included
// at file scope what rubato_i2c.hpp asks and twi_link_client.hpp:
//
//   SysClock, clock, P, core_hz, cycles_per_us, Ruler
//   Serial, serial, bench    the console (UART0 on GP0/GP1) and its TestBench
//   Host, TxEngine, RxEngine the link's I2cHost with its two engines (the
//                            transmit engine of uint16_t command words, the
//                            receive engine of bytes, both on DMA_IRQ_0)
//   SdaPin, SclPin           the link's two pads as Pin types
//   peer_kind, pressure_name, peer_name   (rubato_i2c.hpp's)
//   core1_pass               whether letters c and e (o and x with every
//                            pressure pass fed by core 1) exist: the RP2040's
//   pressure_core1           the flag its pressure verbs read
//   host_kept                true when the host is brought up once at boot
//                            and kept (the RP2040); false when it is brought
//                            up by rp_bring_up() at every pass and given
//                            back by rp_letter_end() (the RP2350, whose rule
//                            is that no link pad is driven before the
//                            partner's rail is read)
//   rp_bring_up()            the family's bring-up before the wire
//                            (I2cTraits::bring_up below)
//   rp_letter_end()          what follows every letter
//
// and defines after it the pressure verbs, console_banner() and
// binding_dark() (declared below). Its glue calls rp_landing_stamp() before
// the shared body's pressure_landed() in the pressure's vector, binds the
// I2C vector (Host::isr(), a TransferDone to the arbiter on true,
// done_from_dma false) and isr_dma_0 (Host::dma_isr(), the same, done_from_dma
// true), and registers every letter through rp_letter<>.
//
// THE HOST. The DW block's command FIFO, sixteen entries, written in runs:
// start() pushes every entry the FIFO has room for and only then unmasks the
// sources; the pump refills at half the transmit FIFO (TX_EMPTY) and takes
// the bytes at half the receive one (RX_FULL); every tenure ends on its own
// STOP_DET. A read phase of three bytes or more rides the engines - the
// first command and the last the pump's, the middle block poured by the
// transmit engine, every byte collected by the receive engine - and a write
// phase stays on the pump (a byte buffer cannot carry the entries' flags).
// The probe is a ONE-BYTE READ: the block has no address-only frame
// (dw_apb_i2c/i2c.hpp's header), so the peer gives it a byte and counts it.
// The address and the speed go into the block under a disable/enable cycle,
// only when they move.
//
// WHERE PREEMPTION REACHES A TENURE HERE, AND HOW FAR. start()'s first push
// starts the tenure: the block takes the entry, puts out the START and the
// address, and while start() goes on pushing the rest the wire is already
// moving. A preemption landing in start() after its k-th push leaves the
// block k entries to clock - the address and k bytes, (1 + k) byte times -
// before it needs entry k + 1; a preemption longer than that empties the
// FIFO with no STOP in it, and the block holds SCL low, mid-tenure, until
// start() resumes and pushes again (docs/dw_apb_i2c/README.md: "an empty
// command FIFO with no STOP holds it too"). So the stretch is not every
// landing with the master active: it needs a preemption longer than two
// byte times at the least (k = 1). One Pressure dispatch with its 3000
// cycles and the road into it is some 28 us on the RP2040 and 23 on the
// RP2350, against two byte times of 45 us at 400 kHz - so at 400 kHz no
// landing stretches - and of 18 us at 1 MHz, where a landing after the
// first push, or the second on the RP2040, does. Letter s measures
// start()'s fill, states the landings expected and judges the 400 kHz case;
// the stretch instrument (stretch_check(), at the end of every spin) counts
// the stretches wherever they happen, and every pass prints them.
//
// THE PEER (rp_twi_peer.hpp), and what it changes in three letters:
//
//  - THE PROBE COUNTS: it is a one-byte read, so `probe_bytes` is 1 (letter
//    w's tallies), letter o's table leaves it out (the shared table's
//    index arithmetic counts read bytes alone: a probe there would shift
//    every read after it by one pattern index and the serve's count by
//    one), and letter q's last case finds the probe after the refused
//    write served one byte, not none - the RP version of q below.
//  - A READ IS SERVED A FIFO AHEAD: up to sixteen bytes given at each read
//    request, those the host does not take flushed at its closing NACK and
//    given back to the tally. The counted serve ends when the bytes RECEIVED
//    and GIVEN reach the count and a STOP has been seen (run_serve's
//    top-of-loop check), so while a read runs its tally stands up to fifteen
//    bytes above what the wire moved: a count reached by a read's gives
//    ends the serve in the middle of that read. Every counted serve here
//    therefore ends on a write of sixteen bytes or more after its last read
//    - letter o's table does (46 bytes of writes after the last read), and
//    letter w's closing write is sixteen bytes in the RP version below.
//  - THE STRETCH IS A READ'S: `hold_us` holds before every byte the client
//    GIVES (its read request holds SCL) and never on a write (a byte written
//    lands in the sixteen-deep receive FIFO), so letter p's stretched read
//    of 8 carries eight holds and its write none.

#pragma once

// =============================================================================
// The names the shared body asks, the two chips alike
// =============================================================================

/// The stretch instrument, defined below: called at the end of every spin.
void stretch_check();

/// A busy wait of at least `cycles`, then the stretch instrument: the one
/// spin above the arbiter is Pressure's (the shared body's), so a spin that
/// ends with start() still in progress is Pressure's dispatch having
/// preempted it.
[[gnu::always_inline]] inline void spin(uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (Ruler::now() - t0 < cycles) {
    }
    stretch_check();
}

/// The three speeds both chips' DW block solves at its clk_sys, which the
/// Tenuto suites' letter f judges both ways against this peer.
constexpr I2cSpeed speeds[] = {I2cSpeed::standard_100k, I2cSpeed::fast_400k, I2cSpeed::fast_plus_1m};
constexpr uint32_t speed_hz(I2cSpeed s) { return i2c_speed_hz(s); }

/// What the shared body changes for this host and this peer (the file
/// header).
struct Shape {
    uint8_t tx_len;
    uint8_t rx_len;
    const char* path;
};
struct I2cTraits {
    /// Letter o's table, each shape with the path this host takes: a read
    /// phase of three or more rides the engines, a shorter one and every
    /// write phase the pump. No probe (the file header: the shared table
    /// counts read bytes alone); the plain writes last, 46 bytes after the
    /// last read, more than the peer's FIFO-ahead can stand its tally above
    /// the wire's.
    static constexpr Shape shapes[] = {
        {0, 1, "pump: one read entry carrying STOP, the byte taken at STOP_DET"},
        {0, 2, "pump: two read entries, the bytes at STOP_DET"},
        {0, 3, "engines: the pump's first and last entries, the middle one poured, every byte collected"},
        {0, 8, "engines"},
        {0, 32, "engines: the commands poured through the 16-deep FIFO on its level request"},
        {1, 1, "pump both halves: the write entry, the read entry with RESTART and STOP"},
        {1, 2, "pump both halves"},
        {2, 8, "pump write, engine read (its first entry with RESTART)"},
        {1, 0, "pump: one entry carrying STOP"},
        {2, 0, "pump"},
        {3, 0, "pump"},
        {8, 0, "pump: one fill"},
        {32, 0, "pump: a fill of sixteen, then refills at TX_EMPTY, half the FIFO"},
    };
    static_assert(Host::has_engines, "the table names engine paths");
    /// The probe is a one-byte read the peer counts (the file header).
    static constexpr uint8_t probe_bytes = 1;
    /// The peer holds before every byte it gives and never on a write: a
    /// read of 8 carries eight holds, each overlapping the host's own SCL
    /// low phase (LCNT + 1 cycles, 6 us at 100 kHz), so the read lasts at
    /// least 8 x (1000 - 6) us plus its nine byte times on the wire (810 us)
    /// - 8762 us - against the bound's 8 x 1000; the write of 8 none.
    static constexpr uint8_t stretch_write_holds = 0;
    static constexpr uint8_t stretch_read_holds = 8;
    /// Letter x's write: sixteen entries, one fill of the command FIFO.
    static constexpr uint8_t fill_tx = 16;
    static_assert(fill_tx == Host::Resource::fifo_depth, "letter x's write is exactly one fill");
    static bool bring_up() { return rp_bring_up(); }
};

// THE TARGET-FREE HALF IS SHARED, NOT COPIED: every family's I2C suite
// compiles the same body around its own host. Its letters q and w are
// replaced by the RP versions below (the file header), so the shared two
// stay uncalled, and -Wunused-function -Werror would refuse a function of
// an unnamed namespace left uncalled: the warning is silenced over the
// include alone (a [[maybe_unused]] on the shared letters is proposed in
// the lane's report).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "../../../avrxt/src/apps/rubato_i2c.hpp"
#pragma GCC diagnostic pop

// =============================================================================
// The stretch instrument and the landing stamp
// =============================================================================

/// The binding's half of letter i (its pressure source, and where the host
/// is given back between letters its pads and its lines): null when dark.
const char* binding_dark();

/// Stretches found since the last pass's print: a spin that ended inside
/// start(), before its mask store (IC_INTR_MASK is zero from the last
/// tenure's finish() to start()'s last store), with the master active and
/// its command FIFO empty - the block holding SCL with no STOP pushed. A
/// one-entry tenure preempted between its only push and the mask store
/// reads the same while its STOP goes out; that window is a few
/// instructions, and letters x and s judge sixteen-entry writes, whose FIFO
/// outlasts any preemption it does not stretch.
volatile uint32_t stretches = 0;
/// The ruler at the pressure's last landing, and the longest a preemption
/// of start() lasted, from the landing to the end of Pressure's spin (the
/// road back into start() not counted).
volatile uint32_t t_land = 0;
volatile uint32_t held_off_max = 0;

void stretch_check() {
    if (!in_start) {
        return;
    }
    using S = Host::Resource;
    const uint32_t held = Ruler::now() - t_land;
    if (held > held_off_max) {
        held_off_max = held;
    }
    if (S::interrupts() == 0u && S::host_active() && S::tx_empty()) {
        stretches = stretches + 1u;
    }
}

/// The pressure's vector, before pressure_landed(): when it landed.
[[gnu::always_inline]] inline void rp_landing_stamp() { t_land = Ruler::now(); }

/// The landing record without its post, for a pressure whose post another
/// hand made (the RP2040's doorbell, whose inbox drain posts the Tick):
/// pressure_landed()'s record, the one place the two are apart - the shared
/// body keeps them in one function (a split is proposed in the lane's
/// report).
[[gnu::always_inline]] inline void i2c_note_landing() {
    rp_landing_stamp();
    const uint8_t c = P::ceiling;
    const uint8_t slot = c < landing_slots - 1u ? c : static_cast<uint8_t>(landing_slots - 1u);
    landed[slot] = landed[slot] + 1u;
    if (arb_event == arb_done) {
        landed_completion = landed_completion + 1u;
        if (done_from_dma) {
            landed_dma_completion = landed_dma_completion + 1u;
        }
    }
    if (in_start) {
        landed_in_start = landed_in_start + 1u;
        if (host_on_wire()) {
            landed_marked = landed_marked + 1u;
        }
    }
    pressure_irqs = pressure_irqs + 1u;
}

// =============================================================================
// What the shared body asks of this family
// =============================================================================

/// IC_STATUS.MST_ACTIVITY: the host's state machine is not idle - from the
/// first entry's START to its STOP.
bool host_on_wire() { return Host::Resource::host_active(); }

const char* host_dark() {
    if (const char* why = binding_dark()) {
        return why;
    }
    if (!Host::idle()) {
        return "the host's engine is not between tenures";
    }
    if constexpr (host_kept) {
        using S = Host::Resource;
        // finish() leaves the mask at zero and the engines' finish takes
        // both DMA request enables down (dw_apb_i2c/i2c.hpp); IC_DMA_CR has
        // no getter in the IP header, so it is read here (a getter is
        // proposed in the lane's report).
        if (S::interrupts() != 0u) {
            return "the host's interrupt mask is not at rest (zero between tenures)";
        }
        if (S::regs().IC_DMA_CR != 0u) {
            return "a DMA request enable of the host stands";
        }
        if (TxEngine::busy() || !RxEngine::idle()) {
            return "an engine's channel holds a block";
        }
        if (S::activity() || !S::tx_empty()) {
            return "the block reports activity or a command waiting";
        }
        if (!SdaPin::read() || !SclPin::read()) {
            return "a line reads low on its pad";
        }
    }
    return nullptr;
}

void print_host() {
    // The hits first, so the reads' own fetches land on the accesses' side
    // as hits; cleared after (the counters saturate at 32 bits).
    const uint32_t hits = Xip::hits();
    const uint32_t misses = Xip::accesses() - hits;
    Xip::reset_counters();
    print(serial, "; XIP misses ", misses, "; stretches inside start() ", stretches, " (start() held off ",
          held_off_max, " cycles at most); DMA faults ", TxEngine::faults(), "/", RxEngine::faults(), " (since boot)");
    stretches = 0;
    held_off_max = 0;
}

/// Every letter is registered through this: the XIP cache's two counters
/// cleared as the letter starts (they saturate, rp2040/flash.hpp's Xip), so
/// that a letter added later cannot forget, and the binding's letter end.
template <void (*letter)()>
void rp_letter() {
    Xip::reset_counters();
    letter();
    rp_letter_end();
}

// =============================================================================
// q - the vocabulary under pressure, the RP version
// =============================================================================

/// The shared body's tq_vocabulary() with ONE expectation changed: the
/// probe after the refused 2+4 is a one-byte read here, which the peer
/// serves, so its report counts `probe_bytes` bytes given where the shared
/// letter counts none (the read half of the refused tenure still never
/// began: the address matches are the tenure's and the probe's, two). The
/// shared letter's verdict is `r.aux1 == 0`; a parameter for it is proposed
/// in the lane's report.
void tq_vocabulary_rp() {
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        begin(s);
        if (!need_peer()) {
            (void)pass_end(s);
            return;
        }
        constexpr uint16_t deadline = 120;
        // A deaf client: the probe and a write meet nobody at dut_addr; the
        // peer is parked at deaf_addr, which answers - the next tenure.
        twilink::Params deaf{};
        deaf.ms = deadline;
        deaf.addr = twilink::dut_addr;
        deaf.flags = twilink::flag_deaf;
        uint8_t probe = no_reply, wdeaf = no_reply, next1 = no_reply;
        bool a1 = serve(deaf);
        const uint32_t t1 = Ruler::now();
        if (a1) {
            const uint8_t w4[4] = {1, 2, 3, 4};
            probe = arb_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, speeds[1]);
            wdeaf = arb_tenure(twilink::dut_addr, w4, 4, nullptr, 0, speeds[1]);
            next1 = arb_tenure(twilink::deaf_addr, nullptr, 0, nullptr, 0, speeds[1]);
            wait_past(t1, deadline);
        }
        // The third byte of six refused.
        twilink::Params n3{};
        n3.ms = deadline;
        n3.addr = twilink::dut_addr;
        n3.nack_at = 3;
        uint8_t nack3 = no_reply, next2 = no_reply;
        const bool a2 = serve(n3);
        const uint32_t t2 = Ruler::now();
        if (a2) {
            const uint8_t w6[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
            nack3 = arb_tenure(twilink::dut_addr, w6, 6, nullptr, 0, speeds[1]);
            next2 = arb_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, speeds[1]);
            wait_past(t2, deadline);
        }
        // The LAST written byte of a 2+4 write-then-read refused: the
        // controller lets go, no read half follows (the peer counts the
        // tenure's one address match, and the probe after it - which on
        // this block is a one-byte read, served).
        twilink::Params n2{};
        n2.ms = deadline;
        n2.addr = twilink::dut_addr;
        n2.nack_at = 2;
        uint8_t nack2 = no_reply, next3 = no_reply;
        const bool a3 = serve(n2);
        const uint32_t t3 = Ruler::now();
        if (a3) {
            const uint8_t w2[2] = {0x77, 0x88};
            for (uint8_t k = 0; k < 4u; ++k) {
                rx_buf[k] = 0xEE;
            }
            nack2 = arb_tenure(twilink::dut_addr, w2, 2, rx_buf, 4, speeds[1]);
            next3 = arb_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0, speeds[1]);
            wait_past(t3, deadline);
        }
        twilink::Report r{};
        const bool rep = a3 && report(r);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    deaf: probe ", probe, ", write ", wdeaf, ", then deaf_addr's probe ", next1,
              "; the third of six refused: ", nack3, ", then the probe ", next2, "; the last of a 2+4's write "
              "refused: ",
              nack2, ", then the probe ", next3, crlf);
        if (rep) {
            print_report(r);
        }
        bench.verdict(pass_label(s), "a deaf client: the probe and a write answered i2c_nack_addr, the next tenure "
                                     "i2c_ok",
                      a1 && probe == i2c_nack_addr && wdeaf == i2c_nack_addr && next1 == i2c_ok);
        bench.verdict(pass_label(s), "the third byte of six refused: i2c_nack_data, the next tenure i2c_ok",
                      a2 && nack3 == i2c_nack_data && next2 == i2c_ok);
        bench.verdict(pass_label(s), "the last written byte of a 2+4 refused: i2c_nack_data, the controller let go "
                                     "(no read half: the one byte served is the probe's), the next tenure i2c_ok",
                      a3 && nack2 == i2c_nack_data && next3 == i2c_ok && rep && r.addr_hits == 2u &&
                          (r.flags & twilink::report_nacked) != 0u && r.aux1 == probe_bytes);
        judge_pressure(s, st);
    }
}

// =============================================================================
// w - the arbiter with three requesters of three priorities, the RP version
// =============================================================================

/// The shared body's tw_arbiter() with ONE change: the closing write is
/// sixteen bytes, not one. The counted serve must end on writes the peer's
/// FIFO-ahead cannot overtake (the file header): during the last read
/// before the close the peer's tally stands up to fifteen bytes above the
/// wire's, and a closing write of one byte would let that tally reach the
/// count with the read still running. A closing length is proposed as a
/// trait in the lane's report.
constexpr uint8_t closing_len = 16;
static_assert(closing_len >= Host::Resource::fifo_depth, "the close outlasts the peer's FIFO-ahead");
uint8_t closing_buf[closing_len];

void tw_arbiter_rp() {
    for (uint8_t& b : closing_buf) {
        b = closing_byte;
    }
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        const bool pressing = s.period_us != 0u;
        begin(s);
        if (!need_peer()) {
            (void)pass_end(s);
            return;
        }
        const uint16_t bytes = static_cast<uint16_t>(w::client_n * (1u + w::client_len) + w::minor_n * w::minor_len +
                                                     (pressing ? w::press_n * probe_bytes : 0u) + closing_len);
        const uint16_t sum = static_cast<uint16_t>(w::client_n * w::client_byte +
                                                   w::minor_n * w::minor_len * w::minor_byte +
                                                   closing_len * closing_byte);
        const uint16_t hits = static_cast<uint16_t>(2u * w::client_n + w::minor_n + (pressing ? w::press_n : 0u) + 1u);
        twilink::Params a{};
        a.count = bytes;
        a.ms = 3000;
        a.addr = twilink::dut_addr;
        a.seed = w::seed;
        a.pattern = twilink::pattern_fixed;
        const bool acted = serve(a);
        for (auto& row : w::client_rx) {
            for (uint8_t& b : row) {
                b = w::untouched;
            }
        }
        w::clear(w::client);
        w::clear(w::minor);
        w::clear(w::press);
        Pressure::outstanding = false;
        const uint8_t rejected0 = I2cArb::rejected_count();
        const uint8_t stale0 = I2cArb::stale_events();
        const uint32_t t0 = Ruler::now();
        bool done = false;
        uint8_t closing = no_reply;
        if (acted) {
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
            closing = arb_tenure(twilink::dut_addr, closing_buf, closing_len, nullptr, 0, speeds[1]);
        }
        const uint32_t took = Ruler::now() - t0;
        twilink::Report r{};
        const bool rep = acted && report(r);
        uint16_t host_bad = 0;
        for (const auto& row : w::client_rx) {
            for (const uint8_t b : row) {
                host_bad = static_cast<uint16_t>(host_bad + (b != w::seed));
            }
        }
        const uint8_t rejected = static_cast<uint8_t>(I2cArb::rejected_count() - rejected0);
        const uint8_t stale = static_cast<uint8_t>(I2cArb::stale_events() - stale0);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    ", us_of(took), " us; Client ", w::client.replies, "/", w::client.posted, " replies (",
              w::client.ok, " ok, ", w::client.out_of_order, " out of order), Minor ", w::minor.replies, "/",
              w::minor.posted, " (", w::minor.ok, " ok), Pressure ", w::press.replies, "/", w::press.posted, " (",
              w::press.ok, " ok); timeouts ", w::client.timeouts + w::minor.timeouts + w::press.timeouts,
              ", rejected ", rejected, ", stale ", stale, "; read bytes wrong ", host_bad, "; the closing write of ",
              closing_len, " ", closing, crlf);
        if (rep) {
            print_report(r);
        }
        bench.verdict(pass_label(s), "every request answered i2c_ok, to its own requester, once",
                      done && w::client.ok == w::client_n && w::minor.ok == w::minor_n &&
                          w::client.replies == w::client_n && w::minor.replies == w::minor_n &&
                          w::press.replies == (pressing ? w::press_n : 0u) &&
                          w::press.ok == (pressing ? w::press_n : 0u) && closing == i2c_ok);
        bench.verdict(pass_label(s), "every reply in its requester's order (the Client's slot k complete and k + 1 "
                                     "untouched at reply k)",
                      w::client.out_of_order == 0u && w::minor.out_of_order == 0u && w::press.out_of_order == 0u);
        bench.verdict(pass_label(s), "nothing rejected, the per-bus timeout never fired (no i2c_timeout, no stale "
                                     "event)",
                      rejected == 0u && stale == 0u && w::client.timeouts == 0u && w::minor.timeouts == 0u &&
                          w::press.timeouts == 0u && w::client.rejected == 0u && w::minor.rejected == 0u &&
                          w::press.rejected == 0u);
        bench.verdict(pass_label(s), "the peer's count, sum and address matches exact, every read byte the peer's",
                      rep && r.count == bytes && r.sum == sum && r.addr_hits == hits && ran_to_count(r) &&
                          host_bad == 0u);
        judge_pressure(s, st);
    }
}

// =============================================================================
// s - (OUTSIDE z) start()'s fill measured, and the stretch by its arithmetic
// =============================================================================

/// Two byte times on the wire at `hz`, in core cycles: the address and the
/// first byte, what the block clocks after start()'s first push before it
/// needs the second entry - the shortest preemption that stretches.
constexpr uint32_t two_bytes(uint32_t hz) { return static_cast<uint32_t>(18ull * core_hz / hz); }

/// THE LONG SPIN of letter s's last run: a Pressure dispatch longer than a
/// whole write of sixteen at 1 MHz on the wire (the address and sixteen
/// bytes, 17 x 9 us = 153 us), so that every landing in start() after its
/// first push empties the FIFO before start() resumes: 160 us of cycles, at
/// the longest period (389 us), where the dispatch and its road still fit
/// a period.
constexpr uint16_t long_spin = static_cast<uint16_t>(160u * cycles_per_us);
static_assert(160u * cycles_per_us > 17u * 9u * cycles_per_us && 160u * cycles_per_us <= 0xFFFFu,
              "the long spin outlasts a 16-byte write at 1 MHz and fits a PassSpec");
static_assert(160u * cycles_per_us + 2000u < 389u * cycles_per_us, "the long spin and its road fit the period");
constexpr PassSpec long_spec{389, long_spin, nullptr};

struct FillOutcome {
    bool acted = false;
    bool exact = false;
    uint16_t bad = 0;
    uint32_t in_start = 0;
    uint32_t marked = 0;
    uint32_t stretched = 0;
    uint32_t held_max = 0;
    uint32_t worst = 0;
};

/// 400 writes of sixteen at `speed` under the pass `s`, against a serve
/// counted on them: the landings inside start(), those with the master
/// active, the stretches, the longest hold-off, every write's status and
/// the peer's tallies.
FillOutcome fill_run(const PassSpec& s, Speed speed) {
    constexpr uint16_t tenures = 400;
    FillOutcome o{};
    twilink::Params a{};
    a.count = static_cast<uint16_t>(tenures * fill_tx);
    a.ms = 6000;
    a.addr = twilink::dut_addr;
    o.acted = serve(a);
    uint16_t sum = 0;
    landed_in_start = 0;
    stretches = 0;
    held_off_max = 0;
    begin(s);
    for (uint16_t k = 0; o.acted && k < tenures; ++k) {
        for (uint8_t i = 0; i < fill_tx; ++i) {
            tx_buf[i] = static_cast<uint8_t>(k * 5u + i * 11u);
            sum = static_cast<uint16_t>(sum + tx_buf[i]);
        }
        if (arb_tenure(twilink::dut_addr, tx_buf, fill_tx, nullptr, 0, speed) != i2c_ok) {
            ++o.bad;
        }
        o.worst = Client::cycles > o.worst ? Client::cycles : o.worst;
    }
    o.in_start = landed_in_start;
    o.stretched = stretches;
    o.held_max = held_off_max;
    const PassStats st = pass_end(s);
    o.marked = st.marked;
    twilink::Report r{};
    const bool rep = o.acted && report(r);
    print_pass(s, st);
    print(serial, "  [", pass_label(s), "] at ", speed_hz(speed) / 1000u, " kHz: landed in start() ", o.in_start,
          ", with the master active ", o.marked, ", stretches ", o.stretched, "; start() held off ", o.held_max,
          " cycles at most (two byte times are ", two_bytes(speed_hz(speed)), "); the worst post to reply ",
          us_of(o.worst), " us; ", o.bad, " not i2c_ok", crlf);
    if (rep) {
        print_report(r);
    }
    o.exact = o.acted && o.bad == 0u && rep && r.count == a.count && r.sum == sum && r.addr_hits == tenures &&
              ran_to_count(r);
    judge_pressure(s, st);
    return o;
}

void ts_fill() {
    if (!need_peer()) {
        return;
    }
    // 1. start()'s own cycles, silent, for five shapes against a fixed
    // pattern served to a deadline (the reads' gives make a counted end
    // unsafe here, the file header).
    struct Cal {
        uint8_t tx;
        uint8_t rx;
        const char* what;
    };
    constexpr Cal cal[] = {
        {1, 0, "a write of 1: one entry"},
        {16, 0, "a write of 16: one fill"},
        {0, 0, "the probe: one read entry"},
        {0, 3, "a read of 3: one entry, then the engines launched inside start()"},
        {0, 16, "a read of 16: the same"},
    };
    constexpr uint8_t cal_n = static_cast<uint8_t>(std::size(cal));
    constexpr uint8_t reps = 32;
    twilink::Params a{};
    a.ms = 600;
    a.addr = twilink::dut_addr;
    a.seed = 0x5C;
    a.pattern = twilink::pattern_fixed;
    const bool acted = serve(a);
    const uint32_t t0 = Ruler::now();
    uint32_t avg[cal_n] = {};
    uint16_t bad = 0, mism = 0, sum = 0, given = 0, written = 0;
    for (uint8_t c = 0; acted && c < cal_n; ++c) {
        uint32_t total = 0;
        for (uint8_t k = 0; k < reps; ++k) {
            for (uint8_t i = 0; i < cal[c].tx; ++i) {
                tx_buf[i] = static_cast<uint8_t>(0x30u + i + k);
                sum = static_cast<uint16_t>(sum + tx_buf[i]);
            }
            for (uint8_t i = 0; i < cal[c].rx; ++i) {
                rx_buf[i] = 0xEE;
            }
            if (arb_tenure(twilink::dut_addr, tx_buf, cal[c].tx, rx_buf, cal[c].rx, speeds[1]) != i2c_ok) {
                ++bad;
            }
            for (uint8_t i = 0; i < cal[c].rx; ++i) {
                mism = static_cast<uint16_t>(mism + (rx_buf[i] != a.seed));
            }
            total += start_cycles;
            written = static_cast<uint16_t>(written + cal[c].tx);
            given = static_cast<uint16_t>(given + (cal[c].tx == 0u && cal[c].rx == 0u ? probe_bytes : cal[c].rx));
        }
        avg[c] = total / reps;
    }
    wait_past(t0, a.ms);
    twilink::Report r{};
    const bool rep = acted && report(r);
    print(serial, "  start()'s own cycles, silent, at ", speed_hz(speeds[1]) / 1000u, " kHz, ", reps, " each:", crlf);
    for (uint8_t c = 0; c < cal_n; ++c) {
        print(serial, "    ", cal[c].what, ": ", avg[c], crlf);
    }
    if (rep) {
        print_report(r);
    }
    bench.verdict("the calibration's tenures i2c_ok and byte-exact, the peer counting every byte written and given",
                  acted && bad == 0u && mism == 0u && rep && r.count == static_cast<uint16_t>(written + given) &&
                      r.sum == sum);
    // 2. The arithmetic. The fifteen pushes after the first are the part of
    // start() with the block already on the wire: the write of 16 less the
    // write of 1. Independent ticks would land there 400 x that / period
    // times in 400 writes.
    const uint32_t window = avg[1] > avg[0] ? avg[1] - avg[0] : 0u;
    const uint32_t period97 = 97u * cycles_per_us;
    print(serial, "  the fill after the first push: ", window, " cycles (", window / 15u,
          " a push); 400 writes at 97 us would land there about ", 400u * window / period97,
          " times with independent ticks; two byte times (the shortest preemption that stretches): ",
          two_bytes(400'000u), " cycles at 400 kHz, ", two_bytes(1'000'000u), " at 1 MHz", crlf);
    // 3. The three runs.
    const FillOutcome f400 = fill_run(passes[4], speeds[1]);
    bench.verdict(pass_label(passes[4]), "400 writes of 16 at 400 kHz: every one i2c_ok, the peer's tallies exact",
                  f400.exact);
    bench.verdict(pass_label(passes[4]), "at 400 kHz no landing stretched the tenure: every preemption of start() "
                                         "shorter than the two byte times the block clocks after its first entry",
                  f400.stretched == 0u && f400.held_max < two_bytes(400'000u));
    const FillOutcome f1m = fill_run(passes[4], speeds[2]);
    bench.verdict(pass_label(passes[4]), "400 writes of 16 at 1 MHz: every one i2c_ok, the peer's tallies exact, "
                                         "stretched or not",
                  f1m.exact);
    const FillOutcome flong = fill_run(long_spec, speeds[2]);
    bench.verdict(pass_label(long_spec), "400 writes of 16 at 1 MHz under a 160 us preemption: every one i2c_ok, "
                                         "the peer's tallies exact, stretched or not",
                  flong.exact);
    bench.verdict(pass_label(long_spec), "a stretch found only where a landing found the master active",
                  flong.stretched <= flong.marked && f1m.stretched <= f1m.marked);
}

// =============================================================================
// c, e - (the RP2040's) letters o and x with every pressure pass fed by core 1
// =============================================================================

/// The shared letter o with Pressure fed through core 1's doorbell in every
/// pressure pass (the binding's pressure verbs read the flag): core 1
/// fetching its program from the flash shares the XIP cache start() runs
/// from. The passes' labels name their period and spin; the source is this
/// letter's. Uncalled where the binding has no core-1 pass.
[[maybe_unused]] void tc_core1_shapes() {
    print(serial, "  every pressure pass below is fed by core 1's doorbell, not by ", pressure_name, crlf);
    pressure_core1 = true;
    to_shapes();
    pressure_core1 = false;
}

/// The shared letter x the same way.
[[maybe_unused]] void te_core1_fill() {
    print(serial, "  the pressure pass below is fed by core 1's doorbell, not by ", pressure_name, crlf);
    pressure_core1 = true;
    tx_fill();
    pressure_core1 = false;
}
