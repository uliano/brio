// vx03_link_dut.hpp - the DUT's half of usart_link.hpp on this stratum's
// desk pair: the link's port, its pads, and the reviewed helpers every
// suite that drives the far chip's usart_peer goes through - the command
// path with its retries, the action and its report, the polled drain, the
// gapless runs timed at either end, the stamps' bound. App-level bench
// tooling beside the suites that include it, as usart_link.hpp sits
// beside the peers; nothing in brio knows it exists.
//
// THE LINK (docs/boards/weact-ch32v203c8.md, docs/boards/ch32v303-evt.md):
// USART3 on its default column (TX PB10, RX PB11) on the CH32V203C8,
// crossed to UART4 on its SECOND column (TX PB0, RX PB1) on the CH32V303 -
// each the other's DUT and peer. A part that is not one end of it keeps
// USART2's name, so a suite still links there, and registers none of its
// two-board letters (`present`).
//
// BOTH ENDS RUN ON THEIR CRYSTALS (8 MHz x18 = 144 MHz, PCLK1 72 MHz), so
// both divide one bus rate by one arithmetic and every rate is the same
// divisor at both ends; what neither divisor can tell is the two
// crystals' disagreement, which a suite measures from both sides.
//
// THE STAMP BOUND, every span's. A span is the first-to-last of a gapless
// run, stamped by a polled loop at each end of it - the far peer's
// run_receive, timed_blast() here - each stamp up to ONE TURN of its loop
// late (the release listings: the peer's idle turn is 12 instructions with
// one APB load, timed_blast()'s 9 with one - some 30 core cycles at most,
// bounded at 64) and a Report's span quantized to the 24 MHz tick (six
// cycles). So a span of S core cycles is good to (2 x 64 + 6) / S:
// span_bound_ppm().
//
// `Dut<Out, Clk>`: Out the suite's console (the print sink every message
// goes to), Clk its clock type. Monostate, as every suite's state is.

#pragma once

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usart.hpp"
#include "util/print.hpp"

// THE LINK PROTOCOL IS SHARED, NOT COPIED: pure encoding, not one register.
#include "../../../avrxt/src/apps/usart_link.hpp"

namespace vx03link {

inline constexpr bool v303 = brio::device::device_class == brio::DeviceClass::v30x_d8;
inline constexpr uint8_t wanted = v303 ? 4 : 3;
inline constexpr uint8_t wanted_column = v303 ? 1 : 0;
/// Whether this part is one end of the pair: the CH32V203C8's USART3, the
/// CH32V303RC's and VC's UART4 on its second column.
inline constexpr bool present = brio::device::has_usart(wanted) && brio::usart_remap_valid(wanted, wanted_column);
inline constexpr uint8_t port_n = present ? wanted : 2;
inline constexpr uint8_t column = present ? wanted_column : 0;
inline constexpr brio::UsartPads pads = brio::usart_pads_for(port_n, column);
/// The far chip's series, as its ident label spells it after the V.
inline constexpr char far_series[4] = {v303 ? '2' : '3', '0', '3', '\0'};

template <typename Out, typename Clk>
struct Dut {
    Dut() = delete;

    using W = brio::Usart<port_n>;
    using WTx = brio::Pin<pads.tx.port, pads.tx.pin>;
    using WRx = brio::Pin<pads.rx.port, pads.rx.pin>;
    static constexpr uint32_t wclk = brio::usart_bus_hz<port_n>(Clk{});
    static_assert(!present || wclk == 72'000'000u, "the link's port counts PCLK1, 72 MHz at 144 MHz");
    static constexpr uint32_t hclk = Clk::hz;
    static constexpr uint32_t calib_rate = 1'000'000;   ///< a divisor of 72 at 72 MHz: exact at both ends
    static constexpr uint16_t calib_frames = 4000;      ///< 40 ms of span

    static inline link::Decoder dec{};
    static inline const uint8_t no_payload[1] = {0};
    /// Raised by report() when the peer refused the last action.
    static inline bool leg_refused = false;
    static inline bool link_quiet = false;

    // ---- time ----------------------------------------------------------

    /// Whole milliseconds on the tick count.
    static void wait_ms(uint32_t ms) {
        const uint32_t t0 = brio::Ticker::millis();
        while (brio::Ticker::millis() - t0 < ms + 1u) {
        }
    }

    /// Exactly `n` core cycles from the composed counter.
    static void wait_cycles(uint32_t n) {
        const uint32_t c0 = brio::Ticker::cycles();
        while (brio::Ticker::cycles() - c0 < n) {
        }
    }

    // ---- the port ------------------------------------------------------

    /// One byte into the link's port, its data register's turn bounded: a
    /// port a leg left disabled never empties it (the command path brings
    /// the port to command mode first; this bound is what a slip costs).
    static void put(uint8_t b) {
        const uint32_t t0 = brio::Ticker::millis();
        while (!W::tx_empty() && brio::Ticker::millis() - t0 < 20u) {
        }
        W::write_data(b);
    }

    static void wait_tc() {
        const uint32_t t0 = brio::Ticker::millis();
        while (!W::tx_complete() && brio::Ticker::millis() - t0 < 50u) {
        }
    }

    /// STATR then DATAR, four times: what the receiver holds and its flags
    /// gone (18.10.1's clear).
    static void flush() {
        for (uint8_t i = 0; i < 4u; ++i) {
            (void)W::status();
            (void)W::read_word();
        }
    }

    /// The link's port with the frame and rate, its reset line pulsed
    /// first. The TX pad is parked as a GPIO output high across the reset:
    /// a port in reset, or with TE clear, does not drive its pad, and the
    /// CH32V303's reads low then (docs/ch32vx03/usart.md, the desk's
    /// instrument).
    static bool open(const brio::UartFormat& f, uint32_t rate) {
        WTx::output(true);
        W::bus_clock(true);
        W::reset();
        if (!W::remap(column) || !W::configure(f, brio::usart_divisor(wclk, rate))) {
            return false;
        }
        W::enable(true);
        W::transmitter(true);
        W::receiver(true);
        return true;
    }

    /// Everything of the link let go: the port reset and gated off, its
    /// column back at the reset code, both pads floating inputs (the far
    /// board's discipline is the same).
    static void release() {
        W::bus_clock(true);
        W::reset();
        (void)W::remap(0);
        W::bus_clock(false);
        WTx::release();
        WRx::release();
    }

    static brio::UartFormat format_of(const link::Cfg& c) {
        using brio::UartBits;
        using brio::UartParity;
        using brio::UartStop;
        brio::UartFormat f{};
        switch (c.bits) {
            case 7: f.bits = UartBits::seven; break;
            case link::bits9_low:
            case link::bits9_high: f.bits = UartBits::nine; break;
            default: f.bits = UartBits::eight; break;
        }
        f.parity = c.parity == 1u ? UartParity::even : c.parity == 2u ? UartParity::odd : UartParity::none;
        f.stop = c.stop == 2u                     ? UartStop::two
                 : c.stop == link::stop_half      ? UartStop::half
                 : c.stop == link::stop_one_half ? UartStop::one_and_half
                                                  : UartStop::one;
        return f;
    }

    /// This end in a commanded configuration: the TX pad the port's, the
    /// RX pad pulled up (a peer between answers drives nothing until its
    /// wiring is proven, and a floating RX invents bytes). No wait after
    /// TE: no low pulse shows as it rises on this family (usart.md).
    static bool configure(const link::Cfg& c) {
        if (!open(format_of(c), c.rate)) {
            return false;
        }
        if (c.mode == link::Mode::ircom && !W::irda(brio::IrdaConfig{})) {
            return false;
        }
        WTx::function();
        WRx::input(brio::PinPull::up);
        flush();
        return true;
    }

    static bool command_mode() {
        dec.reset();
        return configure(link::Cfg{});
    }

    // ---- the command path ------------------------------------------------

    static bool recv_frame(link::Frame& out, uint16_t ms) {
        const uint32_t t0 = brio::Ticker::millis();
        dec.reset();
        while (brio::Ticker::millis() - t0 < ms) {
            const uint16_t st = W::status();
            if ((st & brio::usart_rxne) == 0u) {
                continue;
            }
            const uint8_t b = W::read_data();
            if ((st & (brio::usart_fe | brio::usart_pe | brio::usart_ne)) != 0u) {
                dec.reset();
                continue;
            }
            if (dec.feed(b) == link::Decoder::Result::frame) {
                out = dec.frame();
                return true;
            }
        }
        return false;
    }

    static bool command_once(link::Op op, const uint8_t* p, uint8_t len, uint16_t ms) {
        flush();
        link::write_frame(put, op, p, len);
        wait_tc();
        link::Frame f;
        if (!recv_frame(f, ms)) {
            return false;
        }
        return f.op == link::Op::ack && f.len == 2u && f.data[0] == link::byte_of(op);
    }

    /// The recovery guarantee in action: three attempts, each after a
    /// quiet interval longer than the peer's own partial-frame reset.
    static bool command(link::Op op, const uint8_t* p = no_payload, uint8_t len = 0, uint16_t ms = 80) {
        for (uint8_t k = 0; k < 3u; ++k) {
            if (command_once(op, p, len, ms)) {
                return true;
            }
            (void)command_mode();
            wait_ms(70);
        }
        if (!link_quiet) {
            brio::print(Out{}, "    LINK FAILURE op ", brio::hex(link::byte_of(op)),
                        ": no ack - is the far board running usart_peer?", brio::crlf);
        }
        return false;
    }

    static bool query(link::Op op, link::Frame& data, uint16_t ms = 80) {
        return command(op, no_payload, 0, ms) && recv_frame(data, ms);
    }

    /// Find the peer: the crossed pair is the desk's only wiring, so this
    /// pings until the peer's dark search hears a frame that checks out.
    static bool ensure() {
        link_quiet = true;
        bool found = false;
        for (uint8_t k = 0; k < 3u && !found; ++k) {
            found = command_mode() && command(link::Op::ping);
        }
        link_quiet = false;
        if (!found) {
            brio::print(Out{}, "  the peer did not answer: the far chip runs usart_peer on the pair's link?",
                        brio::crlf);
        }
        return found;
    }

    static bool ident(link::Ident& d) {
        link::Frame f;
        if (!query(link::Op::ident, f) || f.op != link::Op::ident_data || f.len != link::ident_size) {
            return false;
        }
        d = link::get_ident(f.data);
        return true;
    }

    /// Whether an ident is the far chip's peer: this protocol's sanity,
    /// the kind 0x05, the label the other series' part number.
    static bool far_peer(const link::Ident& d) {
        return d.sanity == link::ident_sanity && link::peer_kind(d.version) == link::peer_ch32vx03 &&
               d.label[0] == 'V' && d.label[1] == far_series[0] && d.label[2] == far_series[1] &&
               d.label[3] == far_series[2];
    }

    /// The report of the last action, after its bound has had time to run.
    /// A refusal raises leg_refused and says so once.
    static bool report(link::Report& r, uint16_t after_ms) {
        (void)command_mode();
        if (after_ms != 0u) {
            wait_ms(after_ms);
        }
        for (uint8_t k = 0; k < 4u; ++k) {
            link::Frame f;
            const bool answered = query(link::Op::report, f) && f.op == link::Op::report_data;
            if (answered && f.len != link::report_size) {
                brio::print(Out{}, "  the peer's REPORT is ", f.len, " bytes, this DUT reads ", link::report_size,
                            " - flash both boards from one tree", brio::crlf);
                return false;
            }
            if (answered) {
                r = link::get_report(f.data);
                if ((r.flags & link::report_cfg_failed) != 0u && !leg_refused) {
                    leg_refused = true;
                    brio::print(Out{}, "  the peer REFUSED op ", brio::hex(r.op),
                                " (report_cfg_failed): a capability its silicon has not got", brio::crlf);
                }
                return true;
            }
            wait_ms(40);
        }
        return false;
    }

    /// Command an action, from command mode whatever the last leg left;
    /// the peer starts its own traffic link::settle_ms + 2 ms after its
    /// ack IN ITS TICKS, so this end takes up its end settle_ms after the
    /// ack exactly, on the cycle count.
    static bool act(link::Op op, const link::Params& a, const uint8_t* extra = nullptr, uint8_t n = 0) {
        leg_refused = false;
        (void)command_mode();
        uint8_t p[link::max_payload] = {};
        link::put_params(p, a);
        for (uint8_t i = 0; i < n; ++i) {
            p[link::params_size + i] = extra[i];
        }
        if (!command(op, p, static_cast<uint8_t>(link::params_size + n))) {
            return false;
        }
        wait_cycles(static_cast<uint32_t>(link::settle_ms) * (hclk / 1000u));
        return true;
    }

    /// Milliseconds a frame count needs at a rate, four times over plus a
    /// floor.
    static uint16_t bound_ms(uint32_t rate, uint32_t frames) {
        const uint32_t ms = (12u * frames * 1000u * 4u) / (rate != 0u ? rate : 1u) + 80u;
        return ms > 6000u ? 6000u : static_cast<uint16_t>(ms);
    }

    /// The peer opens its window link::settle_ms + 2 ms after its ack and
    /// flushes what this end's reconfiguration left on the line.
    static void wait_for_window() { wait_ms(6); }

    // ---- the drains and the runs -----------------------------------------

    /// `n` frames back to back: DATAR written on TXE, a frame ahead of the
    /// shifter, so the line never idles.
    static void blast(uint16_t n, uint16_t mask) {
        for (uint16_t i = 0; i < n; ++i) {
            while (!W::tx_empty()) {
            }
            W::write_word(static_cast<uint16_t>((0x55u + i) & mask));
        }
        wait_tc();
    }

    /// What this end's receiver made of a run, polled.
    struct Drain {
        uint16_t frames = 0;
        uint8_t fe = 0, pe = 0, ne = 0, ore = 0;
        uint16_t last = 0;
        uint16_t first = 0;
        uint16_t wrong = 0;   ///< frames whose word was not the one expected
        bool lbd = false;
    };

    static Drain collect(uint16_t ms, int32_t expect = -1) {
        Drain d{};
        const uint32_t t0 = brio::Ticker::millis();
        while (brio::Ticker::millis() - t0 < ms) {
            const uint16_t st = W::status();
            if ((st & brio::usart_rxne) == 0u) {
                continue;
            }
            const uint16_t w = W::read_word();
            if (d.frames == 0u) d.first = w;
            d.last = w;
            ++d.frames;
            if (expect >= 0 && w != static_cast<uint16_t>(expect)) ++d.wrong;
            if ((st & brio::usart_fe) != 0u) ++d.fe;
            if ((st & brio::usart_pe) != 0u) ++d.pe;
            if ((st & brio::usart_ne) != 0u) ++d.ne;
            if ((st & brio::usart_ore) != 0u) ++d.ore;
        }
        return d;
    }

    static uint16_t mask_of(const link::Cfg& c) {
        return c.bits >= link::bits9_low ? 0x1FFu : c.bits == 7u ? 0x7Fu : 0xFFu;
    }

    /// THE STAMP BOUND (the header) for a span of `frames` frames of
    /// `half_bits` half bits at `rate`, in ppm, rounded up.
    static uint32_t span_bound_ppm(uint32_t rate, uint32_t frames, uint32_t half_bits = 20) {
        constexpr uint64_t poll_cycles = 64;
        const uint64_t span =
            (static_cast<uint64_t>(frames - 1u) * half_bits * hclk) / (2u * static_cast<uint64_t>(rate));
        return span == 0u ? 1'000'000u : static_cast<uint32_t>(((2u * poll_cycles + 6u) * 1'000'000u) / span + 1u);
    }

    /// One timed run.
    struct Timed {
        bool ok = false;
        int32_t ppm = 0;
        uint32_t span = 0;
    };

    /// This end sends `n` gapless frames of `c`; the peer counts them (`op`
    /// calibrate - a crystal peer adopts nothing and reports this end's
    /// rate against its crystal - or sink) and reports the span on its
    /// clock; the answer is this end's rate against the peer's clock in ppm
    /// (calibrate_ppm over the frame's own half bits).
    static Timed timed_run(link::Op op, const link::Cfg& c, uint16_t n) {
        Timed k{};
        link::Params a{};
        a.cfg = c;
        a.count = n;
        a.ms = bound_ms(c.rate, n);
        if (!act(op, a) || !configure(c)) {
            return k;
        }
        wait_for_window();
        blast(n, mask_of(c));
        link::Report r{};
        if (!report(r, 5) || leg_refused) {
            return k;
        }
        k.ok = r.count == n && r.span != 0u && r.ferr == 0u && r.perr == 0u && r.ovf == 0u;
        if (!k.ok) {
            brio::print(Out{}, "    the peer counted ", r.count, " of ", n, " at ", c.rate, ", ferr ", r.ferr,
                        " perr ", r.perr, " ovf ", r.ovf, " span ", r.span, " flags ", brio::hex(r.flags),
                        brio::crlf);
        }
        k.span = r.span;
        k.ppm = link::calibrate_ppm(
            link::nominal_span(c.rate, link::frame_half_bits(c.bits, c.parity, c.stop), n), r.span);
        return k;
    }

    /// The other way: the peer blasts `n` gapless 8N1 frames at `rate` and
    /// THIS end stamps the first and the last on its composed cycle count -
    /// DATAR read first and the stamp after, as the peer does - the answer
    /// the peer's rate against this crystal in ppm (positive: the peer
    /// fast).
    static Timed timed_blast(uint32_t rate, uint16_t n) {
        Timed k{};
        link::Params a{};
        a.cfg.rate = rate;
        a.count = n;
        a.ms = bound_ms(rate, n);
        a.pattern = link::pattern_counting;
        if (!act(link::Op::blast, a)) {
            return k;
        }
        link::Cfg c{};
        c.rate = rate;
        if (!configure(c)) {
            return k;
        }
        uint16_t got = 0;
        bool clean = true;
        uint32_t first = 0, last = 0;
        const uint32_t t0 = brio::Ticker::millis();
        while (got < n && brio::Ticker::millis() - t0 < a.ms) {
            const uint16_t st = W::status();
            if ((st & brio::usart_rxne) == 0u) {
                continue;
            }
            (void)W::read_word();
            if (got == 0u) {
                first = brio::Ticker::cycles();
            } else if (got + 1u == n) {
                last = brio::Ticker::cycles();
            }
            clean = clean && (st & (brio::usart_fe | brio::usart_pe | brio::usart_ne | brio::usart_ore)) == 0u;
            ++got;
        }
        link::Report r{};
        (void)report(r, 2);
        k.ok = got == n && clean;
        if (!k.ok) {
            brio::print(Out{}, "    this end took ", got, " of the peer's ", n, " at ", rate,
                        clean ? "" : " with an error", brio::crlf);
            return k;
        }
        const uint64_t nominal = (static_cast<uint64_t>(n - 1u) * 10u * hclk + rate / 2u) / rate;
        const uint32_t measured = last - first;
        k.span = measured;
        k.ppm = static_cast<int32_t>((static_cast<int64_t>(nominal) - static_cast<int64_t>(measured)) * 1'000'000 /
                                     static_cast<int64_t>(measured));
        return k;
    }

    static Timed calibrate() {
        link::Cfg c{};
        c.rate = calib_rate;
        return timed_run(link::Op::calibrate, c, calib_frames);
    }

    /// What this end's divisor gives against the rate asked, in ppm: PCLK1
    /// over BRR against the baud.
    static int32_t divisor_ppm(uint32_t rate) {
        return link::rate_error_ppm(wclk, brio::usart_divisor(wclk, rate), rate);
    }

    static bool within(int32_t a, int32_t b, uint32_t tol) {
        const int64_t d = static_cast<int64_t>(a) - static_cast<int64_t>(b);
        return (d < 0 ? -d : d) <= static_cast<int64_t>(tol);
    }
};

}  // namespace vx03link
