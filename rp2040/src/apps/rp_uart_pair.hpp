// rp_uart_pair.hpp - THE TWO-BOARD SERIAL LETTERS OF THE TWO RASPBERRY PI
// SUITES, p and s, written once and included by test_rp2040_serial and
// test_rp2350_serial: each suite's far end is the other board's uart_peer
// (rp2040/src/apps/rp_uart_peer.hpp), whose role this suite chooses on a
// mode line it drives - low for echo, high for send.
//
// What the binding states before it includes this file, inside its own
// anonymous namespace with `using namespace brio` in force:
//
//   bench, serial               the TestBench and the console
//   Instrument                  Uart<1, ...> on the link's two pads
//   ModeLine                    this board's end of the mode line, a Pin
//   mode_line_text              its name, for the console
//   instrument_up(baud, fmt, loop)        the instrument up (loop false:
//                               on the pads)
//   link_round_trip(count, bits, budget_us, took)   the suite's round
//                               trip of `count` pattern bytes, masked to
//                               `bits`, on Instrument
//   Lfsr, us_now(), spin_us()   the pattern, the ruler, its wait
//
// and <bit> included at its top (std::popcount below).
//
// THE RATE IS 115200 8N1 at both ends, each from its own crystal: 115207
// baud on both chips' dividers (RP2040 67 + 52/64 at 125 MHz, RP2350
// 81 + 24/64 at 150 MHz), a frame of ten bit periods 86.80 us long.

constexpr uint32_t pair_baud = 115200;
constexpr uint32_t pair_real_baud = 115207;   // both dividers' (header)

/// `frames` frames of 8N1 at the dividers' rate, in microseconds.
constexpr uint32_t pair_frames_us(uint32_t frames) {
    return static_cast<uint32_t>(static_cast<uint64_t>(frames) * 10u * 1'000'000u / pair_real_baud);
}

/// THE MODE LINE LOW, and the peer in ECHO by the time it returns: a peer
/// that was SENDING finishes its burst first - 64 frames, 5.6 ms - and
/// reads the line again within one turn of its loop, microseconds; 20 ms
/// covers both with the instrument still down, so no byte of that burst is
/// taken here.
void peer_echo() {
    ModeLine::output(false);
    spin_us(20'000);
}

// -----------------------------------------------------------------------------
// p - the wire against the peer's echo
// -----------------------------------------------------------------------------
/// THE ROUND TRIP'S TIME, derived: this end transmits the 512 frames back
/// to back; the peer's receiver hands them over by its FIFO's level (16 of
/// 32, the transport's half), so it starts echoing a run of sixteen when the
/// run's last frame has landed, and its transmitter then stays busy to the
/// end - one run, sixteen frames, behind this one. The last echoed byte is
/// here 512 + 16 = 528 frames after the first write: 45830 us. At least
/// that less one frame (a byte is had before its stop bit has elapsed, and
/// the ruler steps by 1 us); at most that plus 1 ms for the handler entries
/// and the loop turns of both ends, each a few microseconds, and a crystal
/// pair within 100 ppm (5 us).
void tp_wire() {
    peer_echo();
    if (!instrument_up(pair_baud, {}, false)) {
        bench.verdict("the instrument on the wire", false);
        (void)ModeLine::release();
        return;
    }
    Instrument::clear_errors();
    uint32_t took = 0;
    const uint32_t good = link_round_trip(512, UartBits::eight, 2'000'000u, &took);
    const uint32_t expect = pair_frames_us(528);
    print(serial, "  512 bytes to the peer and back on the pair's wires: ", good, " exact in ", took,
          " us (528 frames: ", expect, " us); FE ", Instrument::frame_errors(), " PE ",
          Instrument::parity_errors(), " BE ", Instrument::break_errors(), " OE ",
          Instrument::hw_overruns(), crlf);
    bench.verdict("every byte comes back exact from the peer, no error counted",
                  good == 512u && Instrument::frame_errors() == 0u && Instrument::break_errors() == 0u &&
                      Instrument::parity_errors() == 0u && Instrument::hw_overruns() == 0u &&
                      Instrument::rx_overruns() == 0u);
    bench.verdict("THE TWO CRYSTALS AGREE ON THE RATE: the round trip in 528 frames' time at 115207 "
                  "baud, less one frame to a millisecond over (derived in the suite)",
                  good == 512u && took + pair_frames_us(1) >= expect && took <= expect + 1000u);
    Instrument::release();
    (void)ModeLine::release();
}

// -----------------------------------------------------------------------------
// s - the error attribution on the wire
// -----------------------------------------------------------------------------
/// THE SPLIT IS THE PATTERN'S, so it is computed and not bounded: 8N1's
/// eighth data bit is where 7E1 expects its parity bit, and 8N1's stop bit
/// is where 7E1 expects its own - so no frame error, and a parity error
/// exactly where the byte's eight bits hold an odd number of ones (even
/// parity over seven bits and the bit read as parity).
///
/// AND WHAT IS DELIVERED IS THE RECEIVE RING'S CONTRACT (util/ring.hpp's
/// SkipRing): a frame dropped for its error marks a drop, and the next look
/// discards everything queued with it, clean bytes included, so no run
/// handed out joins the two sides of a loss. The handler moves the FIFO a
/// level at a time - sixteen entries (the transport's half of 32), drained
/// in one entry long before the seventeenth frame lands 86.8 us later - and
/// this loop looks between two levels, so each run of sixteen of the 64 is
/// delivered whole, as its bytes' low seven bits, when it holds no odd
/// byte, and skipped whole - one skip - when it holds one.
void ts_wire_errors() {
    peer_echo();
    if (!instrument_up(pair_baud, {.bits = UartBits::seven, .parity = UartParity::even}, false)) {
        bench.verdict("the instrument at 7E1", false);
        (void)ModeLine::release();
        return;
    }
    constexpr uint32_t level = 16;   // the transport's receive level
    Lfsr pattern;
    uint8_t want[64];
    uint32_t wanted = 0;
    uint32_t odd = 0;
    uint32_t skipped_runs = 0;
    for (uint32_t run = 0; run < 64u / level; ++run) {
        uint8_t low7[level];
        bool clean = true;
        for (uint32_t i = 0; i < level; ++i) {
            const uint8_t b = pattern.next();
            low7[i] = static_cast<uint8_t>(b & 0x7Fu);
            if ((std::popcount(b) & 1) != 0) {
                ++odd;
                clean = false;
            }
        }
        if (clean) {
            for (uint32_t i = 0; i < level; ++i) {
                want[wanted++] = low7[i];
            }
        } else {
            ++skipped_runs;
        }
    }
    Instrument::clear_errors();
    const uint32_t skips0 = Instrument::rx_skips();
    print(serial, "  listening at 7E1; the peer is asked on ", mode_line_text,
          " for its 64 frames at 8N1 ...", crlf);
    ModeLine::set();   // the rising edge is the peer's cue to send
    const uint32_t t0 = us_now();
    uint32_t accepted = 0;
    uint32_t in_order = 0;
    // 64 frames are 5.6 ms on the wire and the peer starts within a turn of
    // its loop; a second is only the time-out of a peer that is not there.
    // Once every parity error and every expected byte is in, two more
    // milliseconds - a level's 1.39 ms and the receive timeout's 0.28 -
    // give anything else the chance to show.
    uint32_t done_at = 0;
    while (us_now() - t0 < 1'000'000u) {
        uint8_t in[64];
        const uint32_t n = Instrument::read_bulk(in);
        for (uint32_t i = 0; i < n; ++i) {
            if (accepted + i < wanted && in[i] == want[accepted + i] && in_order == accepted + i) {
                ++in_order;
            }
        }
        accepted += n;
        if (done_at == 0u && Instrument::parity_errors() >= odd && accepted >= wanted) {
            done_at = us_now();
        }
        if (done_at != 0u && us_now() - done_at >= 2000u) {
            break;
        }
    }
    const uint32_t took = (done_at != 0u ? done_at : us_now()) - t0;
    const uint32_t skips = (Instrument::rx_skips() - skips0) & 0xFFu;
    ModeLine::clear();
    print(serial, "  PE ", Instrument::parity_errors(), " FE ", Instrument::frame_errors(), " BE ",
          Instrument::break_errors(), " (the pattern has ", odd, " bytes of odd weight); ", accepted,
          " bytes delivered, ", in_order, " as the pattern's, ", skips, " skips (", skipped_runs,
          " of the 4 runs of 16 hold an odd byte); the last in ", took, " us from the cue", crlf);
    bench.verdict("THE PEER'S EIGHTH DATA BIT IS READ AS PARITY: a parity error counted and the frame "
                  "dropped exactly where the byte has an odd number of ones, and no frame error",
                  Instrument::parity_errors() == odd && Instrument::frame_errors() == 0u &&
                      Instrument::break_errors() == 0u);
    bench.verdict("and nothing is handed out joined across a drop: a run of sixteen with an odd byte "
                  "skipped whole, one skip, a run without delivered as its low seven bits",
                  accepted == wanted && in_order == wanted && skips == skipped_runs);

    // THE OTHER WAY ROUND, which delivers: a 7E1 frame IS an 8N1 frame whose
    // eighth data bit is the parity bit, so the peer's echo (8N1) returns
    // each one bit for bit, and here every one arrives with the right parity.
    peer_echo();
    Instrument::clear_errors();
    uint32_t echo_took = 0;
    const uint32_t echoed = link_round_trip(64, UartBits::seven, 1'000'000u, &echo_took);
    print(serial, "  64 frames sent at 7E1, echoed by the peer at 8N1: ", echoed, " back exact in ", echo_took,
          " us; PE ", Instrument::parity_errors(), " FE ", Instrument::frame_errors(), crlf);
    bench.verdict("a 7E1 frame is an 8N1 frame with the parity for its eighth bit: 64 sent, echoed by the "
                  "peer at 8N1, every one back exact with no error - this end's parity as a foreign "
                  "receiver reads it",
                  echoed == 64u && Instrument::parity_errors() == 0u && Instrument::frame_errors() == 0u);
    Instrument::release();
    (void)ModeLine::release();
}

// -----------------------------------------------------------------------------
// What the partner sees of a loop-back letter (used by t)
// -----------------------------------------------------------------------------
/// The instrument on its loop-back at 9600, eight frames of 0xFF sent, and
/// the TRANSMIT PAD watched through the bank meanwhile (`TxPad` the pad as a
/// Pin type, read through SIO whoever owns it): its falling edges counted
/// over twelve frames' time, and what the loop brought back. Whether the
/// PL011 under LBE still drives its UARTTXD pad decides whether the loop-back
/// letters put traffic on the pair's wire into the partner's receiver.
template <typename TxPad>
uint32_t loopback_pad_falls(uint32_t* looped) {
    constexpr uint32_t baud = 9600;
    if (!instrument_up(baud)) {
        *looped = 0;
        return 0xFFFFFFFFu;
    }
    spin_us(5000);
    static const uint8_t ones[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    bool level = TxPad::read();
    uint32_t falls = 0;
    (void)Instrument::write_bulk(ones);
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 12u * 10u * 1'000'000u / baud) {
        const bool now_level = TxPad::read();
        falls += (level && !now_level) ? 1u : 0u;
        level = now_level;
    }
    uint8_t back[16];
    *looped = Instrument::read_bulk(back);
    Instrument::release();
    return falls;
}
