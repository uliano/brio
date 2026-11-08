// x035_link_dut.hpp - the DUT's half of usart_link.hpp on this stratum's
// desk link: the CH32X035's USART4 opened and handed back, and the
// reviewed helpers every suite that drives the far CH32V006's usart_peer
// goes through - the polled command path with its retries, the action and
// its report, the gapless blast, the calibrate leg that times this end's
// HSI against the far chip's clock. App-level bench tooling beside the suites
// that include it, as usart_link.hpp sits beside the peers; nothing in
// brio knows it exists.
//
// THE LINK (docs/boards/weact-ch32x035f8.md, docs/boards/weact-ch32v006f8.md):
// USART4 on its default column, PB0 (TX) and PB1 (RX), wired to the
// CH32V006F8U6's USART2 on column 3 - this PB0 to its PD3 (RX), this PB1 to
// its PD2 (TX). The far chip runs usart_peer at 48 MHz from its HSI's PLL,
// its board fitting no crystal (on a board that fits one, the crystal
// doubled - IDENT's xtal says which); this end runs the 48 MHz HSI, so at
// every rate the two ends divide 48 MHz by one divisor and differ by the
// two clocks' ratio alone, which calibrate() measures. Against an RC peer
// that ratio is all a calibrate gives: a RELATIVE leg (a rate judged
// against a reference leg taken back to back) holds, an absolute claim
// about this end's HSI does not, and the letters that make one decline
// when IDENT reads xtal = 0.
//
// HOW A SUITE INCLUDES IT: inside its own anonymous namespace, with `using
// namespace brio` in force, after it names its console (`Serial` and its
// tag `serial`), its bench (`bench`, a TestBench or TwoKeyTestBench), its
// clock (`SysClock` and `clock`), its kernel platform (`P`), its ruler
// (`Ruler`, util/test_wait.hpp's: now() the ticker's composed cycles(),
// a constexpr hz()) and the link's resource and pads (`U4` = Usart<4>,
// `Tx4` = Pin<'B', 0>, `Rx4` = Pin<'B', 1>), having included
// ch32x035/delay.hpp, pin.hpp, ticker.hpp, usart.hpp, util/print.hpp,
// util/testbench.hpp, util/test_wait.hpp and usart_link.hpp at file
// scope. Everything here is in namespace `x035link`: a Tenuto suite that
// is its only user says `using namespace x035link;` after the include; a
// suite whose own names could collide (the Rubato link suite, beside the
// link suites' shared body and its helpers) qualifies each call. Every helper is a
// polled loop run from main context or from a dispatch, with interrupts
// enabled (the ruler and the tick count stand on the kernel tick) - none
// is an interrupt body, none masks interrupts. Every wait is bounded, on
// the ruler (util/test_wait.hpp) or, where a gapless blast cannot afford
// the ruler's read, on the tick count.

#pragma once

namespace x035link {

// ---------------------------------------------------------------------------
// The port
// ---------------------------------------------------------------------------

/// USART4 open with the frame and rate, the port's reset line pulsed
/// first, the pads as the caller sets them after. The TX pad is parked as
/// a GPIO output high across the reset: a USART in reset drives its
/// alternate output low, an edge the far receiver takes for a start bit.
bool u4_open(const UartFormat& f, uint32_t rate) {
    Tx4::output(true);
    U4::bus_clock(true);
    U4::reset();
    if (!U4::configure(f, usart_divisor(SysClock::hz, rate))) {
        return false;
    }
    U4::enable(true);
    U4::transmitter(true);
    U4::receiver(true);
    return true;
}

/// STATR then DATAR, four times: what the receiver holds and its flags
/// gone.
void u4_flush() {
    for (uint8_t i = 0; i < 4u; ++i) {
        (void)U4::status();
        (void)U4::read_word();
    }
}

/// Everything of the link let go: the port reset and gated off, both pads
/// floating inputs - the far board's discipline between its answers.
[[maybe_unused]] void u4_release() {
    U4::bus_clock(true);
    U4::reset();
    U4::bus_clock(false);
    Tx4::release();
    Rx4::release();
}

// ---------------------------------------------------------------------------
// The command path
// ---------------------------------------------------------------------------

link::Decoder dec;
const uint8_t no_payload[1] = {0};
/// Raised by peer_report() when the peer refused the last action.
bool leg_refused = false;
bool link_quiet = false;

/// A wait of whole milliseconds on the tick count, from `ms` to `ms` + 1
/// of them (the wait starts inside a tick period): the protocol's waits on
/// the far chip, which counts its own milliseconds the same way, were
/// timed so. A wait whose condition never holds, so it lasts the whole
/// budget (util/test_wait.hpp); its answer is always false, and dropped.
void wait_ms(uint32_t ms) {
    (void)brio::wait_ticks<P>([] { return false; }, ms);
}

/// Set by ensure_link() once the peer has answered in this boot: what
/// tells an absent peer (a skip) from one gone silent (a failure).
bool peer_seen = false;

/// One byte into DATAR at its turn. TXE comes within one frame at any rate
/// the command path runs (115200: 87 us), so two to three ticks bound it
/// (on the tick count, one load a turn); a byte whose
/// turn never came is dropped, and the frame it belonged to then fails its
/// checksum or its answer's time-out - the command path's retry.
void put4(uint8_t b) {
    if (brio::wait_ticks<P>([] { return U4::tx_empty(); }, 2)) {
        U4::write_data(b);
    }
}

/// The transmitter's last frame off the wire (TC). One frame at the
/// slowest rate a caller transmits at (9600: 1.04 ms) many times over:
/// 50 ms. False when TC never came.
[[nodiscard]] bool wait_tc() {
    return brio::wait_until<Ruler>([] { return U4::tx_complete(); }, 50'000);
}

/// The DUT's end in a commanded configuration: the link pads the
/// function's TX and a pulled-up RX (a peer between answers drives
/// nothing until its wiring is proven, and a floating RX invents bytes).
bool dut_link(const link::Cfg& c) {
    UartFormat f{};
    switch (c.bits) {
        case 7: f.bits = UartBits::seven; break;
        case link::bits9_low:
        case link::bits9_high: f.bits = UartBits::nine; break;
        default: f.bits = UartBits::eight; break;
    }
    f.parity = c.parity == 1u ? UartParity::even : c.parity == 2u ? UartParity::odd : UartParity::none;
    f.stop = c.stop == 2u ? UartStop::two
           : c.stop == link::stop_half ? UartStop::half
           : c.stop == link::stop_one_half ? UartStop::one_and_half : UartStop::one;
    if (!u4_open(f, c.rate)) {
        return false;
    }
    // IrDA: normal mode, or with `txpl` named the low-power mode at that
    // prescaler - the peer's own reading of the field (usart_peer).
    if (c.mode == link::Mode::ircom &&
        !U4::irda(c.txpl != 0u ? IrdaConfig{true, c.txpl} : IrdaConfig{})) {
        return false;
    }
    // TE's idle frame first: a pad handed to the function the moment TE
    // rises shows the far receiver a low pulse (measured on the far
    // chip's USART, the same block's lineage), so two bit times and 20 us
    // pass before it.
    (void)delay_us(clock, 2u * (usart_divisor(SysClock::hz, c.rate) / 48u) + 20u);
    Tx4::function();
    (void)Rx4::input(PinPull::up);
    u4_flush();
    return true;
}

bool link_command_mode() {
    dec.reset();
    return dut_link(link::Cfg{});
}

/// A whole frame from the peer within `ms` milliseconds: each turn of the
/// wait takes the byte RXNE holds, if any, into the decoder (an errored
/// byte resets it).
bool recv_frame(link::Frame& out, uint16_t ms) {
    dec.reset();
    auto heard = [&out] {
        const uint16_t st = U4::status();
        if ((st & usart_rxne) == 0u) {
            return false;
        }
        const uint8_t b = U4::read_data();
        if ((st & (usart_fe | usart_pe | usart_ne)) != 0u) {
            dec.reset();
            return false;
        }
        if (dec.feed(b) == link::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
        return false;
    };
    // On the tick count, one load a turn as the loop always was: the frame
    // arrives at 115200 under the Rubato suites' pressure, and a ruler read
    // in every turn (a composed cycles() call) lengthened the turn that has
    // to catch each byte before the next one lands.
    return brio::wait_ticks<P>(heard, ms);
}

bool command_once(link::Op op, const uint8_t* p, uint8_t len, uint16_t ms) {
    u4_flush();
    link::write_frame(put4, op, p, len);
    if (!wait_tc()) {
        return false;
    }
    link::Frame f;
    if (!recv_frame(f, ms)) {
        return false;
    }
    return f.op == link::Op::ack && f.len == 2u && f.data[0] == link::byte_of(op);
}

/// The recovery guarantee in action: three attempts, each after a quiet
/// interval longer than the peer's own partial-frame reset.
bool command(link::Op op, const uint8_t* p = no_payload, uint8_t len = 0, uint16_t ms = 80) {
    for (uint8_t k = 0; k < 3u; ++k) {
        if (command_once(op, p, len, ms)) {
            return true;
        }
        (void)link_command_mode();
        wait_ms(70);
    }
    if (!link_quiet) {
        print(serial, "    LINK FAILURE op ", hex(link::byte_of(op)),
              ": no ack - is the far board running usart_peer?", crlf);
    }
    return false;
}

bool query(link::Op op, link::Frame& data, uint16_t ms = 80) {
    return command(op, no_payload, 0, ms) && recv_frame(data, ms);
}

/// Find the peer: full duplex is the desk's only wiring, so this pings
/// until the peer's dark search hears a frame that checks out.
bool ensure_link() {
    link_quiet = true;
    bool found = false;
    for (uint8_t k = 0; k < 3u && !found; ++k) {
        found = link_command_mode() && command(link::Op::ping);
    }
    link_quiet = false;
    if (!found) {
        print(serial, "  the peer did not answer: the CH32V006 runs usart_peer on the pair's USART2 column?", crlf);
    }
    peer_seen = peer_seen || found;
    return found;
}

/// Every letter that needs the peer asks first. A peer never heard in this
/// boot is absent: the letter declines with the reason and claims no
/// verdict. A peer that answered earlier in this boot and is silent now
/// FAILS the letter.
bool need_peer() {
    if (ensure_link()) {
        return true;
    }
    if (!peer_seen) {
        bench.skip("peer absent: no ping answered in this boot (usart_peer)");
        return false;
    }
    bench.verdict("the peer, which answered earlier in this boot, still answers a ping (the CH32V006 running "
                  "usart_peer)",
                  false);
    return false;
}

bool peer_ident(link::Ident& d) {
    link::Frame f;
    if (!query(link::Op::ident, f) || f.op != link::Op::ident_data || f.len != link::ident_size) {
        return false;
    }
    d = link::get_ident(f.data);
    return true;
}

/// The report of the last action, after its bound has had time to run.
/// A refusal raises leg_refused and says so once.
bool peer_report(link::Report& r, uint16_t after_ms) {
    (void)link_command_mode();
    if (after_ms != 0u) {
        wait_ms(after_ms);
    }
    for (uint8_t k = 0; k < 4u; ++k) {
        link::Frame f;
        const bool answered = query(link::Op::report, f) && f.op == link::Op::report_data;
        if (answered && f.len != link::report_size) {
            print(serial, "  the peer's REPORT is ", f.len, " bytes, this DUT reads ", link::report_size,
                  " - flash both boards from one tree", crlf);
            return false;
        }
        if (answered) {
            r = link::get_report(f.data);
            if ((r.flags & link::report_cfg_failed) != 0u && !leg_refused) {
                leg_refused = true;
                print(serial, "  the peer REFUSED op ", hex(r.op),
                      " (report_cfg_failed): a capability its silicon has not got", crlf);
            }
            return true;
        }
        wait_ms(40);
    }
    return false;
}

/// Command an action; the peer starts its own traffic link::settle_ms +
/// 2 ms after its ack, so the DUT has that window to take up its end.
bool peer_act(link::Op op, const link::Params& a, const uint8_t* extra = nullptr, uint8_t n = 0) {
    leg_refused = false;
    uint8_t p[link::max_payload] = {};
    link::put_params(p, a);
    for (uint8_t i = 0; i < n; ++i) {
        p[link::params_size + i] = extra[i];
    }
    if (!command(op, p, static_cast<uint8_t>(link::params_size + n))) {
        return false;
    }
    // settle_ms EXACTLY: the peer starts its own traffic settle_ms + 2 ms
    // after its ack in its ticks, as early as 4 ms, and this end's
    // reconfiguration must be done by then (a frame lost to the race was
    // measured with a tick-granular wait here).
    brio::spin_us<Ruler>(static_cast<uint32_t>(link::settle_ms) * 1000u);
    return true;
}

/// Milliseconds a frame count needs at a rate, four times over plus a floor.
uint16_t bound_ms(uint32_t rate, uint32_t frames) {
    const uint32_t ms = (12u * frames * 1000u * 4u) / (rate != 0u ? rate : 1u) + 80u;
    return ms > 4000u ? 4000u : static_cast<uint16_t>(ms);
}

/// The peer opens its window link::settle_ms + 2 ms after its ack and
/// flushes what the DUT's reconfiguration left on the line; peer_act()
/// returned settle_ms after the ack, so 6 ms more leave 4 ms of margin.
void wait_for_peer_window() { wait_ms(6); }

/// `n` frames back to back: DATAR written on TXE, a frame ahead of the
/// shifter, so the line never idles. Each frame's turn is bounded on the
/// tick count, not the ruler: at 3 Mbaud a frame is 160 cycles, and the
/// write must follow TXE inside the frame the shifter holds; a turn of the
/// tick wait is one load beside TXE's, where the ruler's composed read is
/// tens of cycles. Two whole tick periods (2 to 3 ms) are past a frame at
/// any rate the suites blast at (115200 and up: 87 us at most). False when a turn never
/// came or the last frame did not leave.
[[nodiscard]] bool dut_blast(uint16_t n, uint16_t mask) {
    for (uint16_t i = 0; i < n; ++i) {
        if (!brio::wait_ticks<P>([] { return U4::tx_empty(); }, 2u)) {
            return false;
        }
        U4::write_word(static_cast<uint16_t>((0x55u + i) & mask));
    }
    return wait_tc();
}

// ---------------------------------------------------------------------------
// The HSI against the far chip's clock
// ---------------------------------------------------------------------------

constexpr uint32_t calib_rate = 1'000'000;   ///< a divisor of 48: the HSI's error alone
constexpr uint16_t calib_frames = 4000;      ///< 40 ms, about a million ticks of span

/// One calibrate leg: the peer counts and times `n` gapless frames of the
/// DUT at `c`; the answer is the DUT's rate against the peer's clock - its
/// crystal where it runs one, its HSI otherwise (the file's header).
struct Calib {
    bool ok = false;
    int16_t ppm = 0;
    uint32_t span = 0;
    uint32_t blocks = 0;   ///< the transmit engine's blocks (a suite's engined variant)
    uint32_t u4 = 0;       ///< USART4's own vector entries during the blast (the same)
};

Calib calibrate(const link::Cfg& c, uint16_t n) {
    Calib k{};
    link::Params a{};
    a.cfg = c;
    a.count = n;
    a.ms = bound_ms(c.rate, n);
    if (!peer_act(link::Op::calibrate, a)) {
        return k;
    }
    if (!dut_link(c)) {
        return k;
    }
    wait_for_peer_window();
    const bool sent = dut_blast(n, c.bits >= link::bits9_low ? 0x1FFu : c.bits == 7u ? 0x7Fu : 0xFFu);
    link::Report r{};
    if (!peer_report(r, 5) || leg_refused) {
        return k;
    }
    k.ok = sent && r.count == n && r.span != 0u && r.ferr == 0u && r.ovf == 0u;
    if (!k.ok) {
        print(serial, "    calibrate at ", c.rate, ": the peer counted ", r.count, " of ", n, ", ferr ", r.ferr,
              " ovf ", r.ovf, " span ", r.span, " flags ", hex(r.flags), crlf);
    }
    k.ppm = static_cast<int16_t>(r.aux16);
    k.span = r.span;
    return k;
}

}  // namespace x035link
