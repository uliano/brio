// test_vx03_i2s - the reference bench suite for the I2S face of the
// CH32V303's SPI2 and SPI3: ch32vx03/spi.hpp's `I2s<n>` over RM 20.3,
// 20.4.8 and 20.4.9, one instance against the other on the evaluation
// board's own wires.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE WIRES. The face's three signals ride SPI pads - WS is NSS's, CK is
// SCK's, SD is MOSI's - and the board carries SPI2's column wired to
// SPI3's default one: PB12-PA15 (WS), PB13-PB3 (CK), PB15-PB5 (SD), with
// PB14-PB4 beside them unused here. Each is looked for before a letter
// that wants it runs (one end driven as a plain output, the other read
// against the opposite pull, both levels), and a letter says "no wire" and
// passes nothing without it. THE MASTER CLOCK IS LEFT OFF: I2S2's MCK pad
// PC6 carries the timers' wire on this board, so MCKOE is written and read
// back and no pad is handed to it.
//
// THE CLOCK. SYSCLK is the I2S clock on this class (figure 3-3), and the
// suite runs the PLL at 144 MHz: a 16-bit frame reaches 16 kHz at a
// divider of 281, the lowest the ladder allows being some 8.8 kHz - the
// frame rate is measured against the core's own counter, which the same
// SYSCLK drives, so the ratio is exact whatever the root's own error.
//
// THE PADS. PB12, PB13, PB15 and PA15, PB3, PB5 (the two columns' WS, CK
// and SD), PB14 and PB4 released. NEVER TOUCHED: PA9/PA10 (the console),
// PA13/PA14 (the debug port), PA11/PA12 (the USB pads), PC6 (the timers'
// wire, and I2S2's MCK) - and PB2, toggled per command as every suite of
// this target does.
//
// What is exercised, letter by letter:
//   a  THE REGISTER FACE with no wire: every configuration field of
//      I2S_CFGR and I2SPR written and read back on both instances, TXE on
//      a configured face before and after I2SE, the two faces switched by
//      I2SMOD, the prescaler refused while enabled, the clock read as
//      SYSCLK, and 20.3.3's frame rates at the usual audio rates with the
//      error each divider leaves
//   b  THE FOUR STANDARDS: I2S2 a host transmitting into I2S3 a client
//      receiving, Philips, MSB-justified, LSB-justified and PCM in both its
//      frames, a known pattern compared word for word after the receiver's
//      first word is found in it, the channel side followed and the first
//      word's side printed, the frame rate counted against the core's
//      counter - and a datum written before I2SE, followed to the wire
//   c  THE WIDTHS: a 16-bit datum in a 32-bit channel, a 24-bit one and a
//      32-bit one - two data-register accesses a channel for the last two
//      - under Philips, compared word for word, the 24-bit datum's low
//      byte counted where it reads zero
//   d  THE ROLES SWAPPED: I2S3 the host transmitting into I2S2 the client
//   e  THE OTHER DIRECTION: I2S2 a host RECEIVING from I2S3 a client
//      transmitting, the client fed one datum ahead, and the host stopped
//      the way 20.3.4.3 prescribes
//   f  THE FLAGS AND THE VECTORS: the overrun of a receiver nobody reads,
//      the underrun of a client transmitter nobody feeds, BSY low
//      throughout a master reception, and each flag reaching its
//      instance's vector under ERRIE
//   g  THE DMA REQUESTS: a block of half-words at 48 kHz, I2S2's transmit
//      request on DMA1's channel 5 and I2S3's receive request on DMA2's
//      channel 1 - the SPI face's two slots
//
// build: boards = v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/spi.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32vx03Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

TestBench<Serial, 12> bench;

using I2 = I2s<2>;
using I3 = I2s<3>;

/// The two columns: WS, CK and SD of each, MCK left unnamed.
constexpr I2sPins pins2 = i2s_default_pins<2>;
constexpr I2sPins pins3 = i2s_default_pins<3>;
using Ws2 = Pin<pins2.ws.port, pins2.ws.pin>;
using Ck2 = Pin<pins2.ck.port, pins2.ck.pin>;
using Sd2 = Pin<pins2.sd.port, pins2.sd.pin>;
using Ws3 = Pin<pins3.ws.port, pins3.ws.pin>;
using Ck3 = Pin<pins3.ck.port, pins3.ck.pin>;
using Sd3 = Pin<pins3.sd.port, pins3.sd.pin>;
/// The fourth wire of the SPI link, which the face does not use.
using Miso2 = Pin<'B', 14>;
using Miso3 = Pin<'B', 4>;

constexpr uint32_t i2s_hz = i2s_clock_hz(clock);
constexpr uint32_t ticks_per_us = SysClock::hz / 1'000'000UL;

/// What the two vectors counted (letter f).
volatile uint32_t vec2 = 0;
volatile uint32_t vec3 = 0;
volatile uint32_t vec2_flags = 0;
volatile uint32_t vec3_flags = 0;

// ---- time ------------------------------------------------------------------

/// The core's STK read as a stopwatch, the shape every suite of this
/// stratum uses: accumulated poll by poll, one period folded in across
/// each wrap.
class Stopwatch {
public:
    Stopwatch() { start(); }

    void start() {
        last_ = stk()->CNTL;
        acc_ = 0;
    }

    uint32_t cycles() {
        const uint32_t period = stk()->CMPLR + 1u;
        const uint32_t now = stk()->CNTL;
        acc_ += (now >= last_) ? (now - last_) : (now + period - last_);
        last_ = now;
        return acc_;
    }

    uint32_t us() { return cycles() / ticks_per_us; }

private:
    uint32_t last_ = 0;
    uint32_t acc_ = 0;
};

void wait_us(uint32_t us) {
    Stopwatch w;
    while (w.us() < us) {
    }
}

void console_drain() {
    Stopwatch w;
    while (!Serial::tx_idle() && w.us() < 200'000UL) {
    }
}

// ---- the wires -------------------------------------------------------------

/// Drive one pad, read the other against the OPPOSITE pull; both left
/// floating afterwards.
template <typename Driver, typename Reader>
bool pads_linked() {
    Reader::input(PinPull::down);
    Driver::output(true);
    wait_us(20);
    const bool high = Reader::read();
    Reader::input(PinPull::up);
    Driver::clear();
    wait_us(20);
    const bool low = !Reader::read();
    Driver::release();
    Reader::release();
    return high && low;
}

bool wired() {
    const bool ws = pads_linked<Ws2, Ws3>();
    const bool ck = pads_linked<Ck2, Ck3>();
    const bool sd = pads_linked<Sd2, Sd3>();
    print(serial, "  the wires WS PB12-PA15 ", ws ? "in place" : "ABSENT", ", CK PB13-PB3 ",
          ck ? "in place" : "ABSENT", ", SD PB15-PB5 ", sd ? "in place" : "ABSENT", crlf);
    return ws && ck && sd;
}

/// Both instances back to their reset state, their gates shut, every pad
/// floating, both vectors silenced.
void all_off() {
    Pfic::disable(I2::irq);
    Pfic::disable(I3::irq);
    I2::bus_clock(true);
    I2::reset();
    I2::bus_clock(false);
    I3::bus_clock(true);
    I3::reset();
    (void)I3::remap(0);
    I3::bus_clock(false);
    I2::release_pads<pins2>();
    I3::release_pads<pins3>();
    Miso2::release();
    Miso3::release();
    vec2 = 0;
    vec3 = 0;
    vec2_flags = 0;
    vec3_flags = 0;
}

// ---- the stream and its judge ---------------------------------------------

/// The pattern a transmitter pours, as an endless function of the word's
/// index: what a receiver that joined at any word can be judged against.
uint16_t pattern(uint32_t i) { return static_cast<uint16_t>(0x1357u + i * 0x0B3Du); }

constexpr uint16_t stream_words = 96;
uint16_t got[stream_words];
bool got_right[stream_words];

/// What a receiver's run compares to: how many words matched the
/// pattern after the first word was found in it, and where that was.
struct Judgement {
    uint16_t words;
    uint16_t matched;
    int32_t offset;
};

/// The received run judged against the pattern: its first word searched
/// for among the first 64 of the pattern, the rest compared from there.
/// `mask` covers what the width carries (a 24-bit datum's second word has
/// no low byte).
Judgement judge(uint16_t n, uint16_t second_mask = 0xFFFFu, bool two_per_channel = false) {
    Judgement j{n, 0, -1};
    for (uint32_t k = 0; k < 64u && j.offset < 0; ++k) {
        const uint16_t mask = (two_per_channel && (k & 1u) != 0u) ? second_mask : 0xFFFFu;
        if ((pattern(k) & mask) == (got[0] & mask)) {
            j.offset = static_cast<int32_t>(k);
        }
    }
    if (j.offset < 0) {
        return j;
    }
    for (uint16_t i = 0; i < n; ++i) {
        const uint32_t k = static_cast<uint32_t>(j.offset) + i;
        const uint16_t mask = (two_per_channel && (k & 1u) != 0u) ? second_mask : 0xFFFFu;
        if ((pattern(k) & mask) == (got[i] & mask)) {
            ++j.matched;
        }
    }
    return j;
}

/// Pour the pattern into `Tx` and collect `n` words out of `Rx`, both
/// polled - one datum every few thousand core cycles at the rates this
/// suite asks. The transmitter is fed without end so it never starves
/// while the receiver fills.
template <typename Tx, typename Rx>
uint16_t stream(uint16_t n, uint32_t budget_us) {
    uint32_t oi = 0;
    uint16_t ii = 0;
    Stopwatch w;
    while (ii < n && w.us() < budget_us) {
        if (Tx::tx_empty()) {
            Tx::data(pattern(oi++));
        }
        if (Rx::rx_ready()) {
            got_right[ii] = Rx::right_channel();
            got[ii] = Rx::data();
            ++ii;
        }
    }
    return ii;
}

/// The receiver's words counted over a window of the core's own counter,
/// the transmitter fed throughout: words per second.
template <typename Tx, typename Rx>
uint32_t word_rate(uint32_t window_us) {
    uint32_t oi = 0;
    uint32_t words = 0;
    Stopwatch w;
    while (w.us() < window_us) {
        if (Tx::tx_empty()) {
            Tx::data(pattern(oi++));
        }
        if (Rx::rx_ready()) {
            (void)Rx::data();
            ++words;
        }
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(words) * 1'000'000ULL) / window_us);
}

/// A host transmitter and a client receiver brought up on their columns:
/// the client configured and enabled first - it must be listening before
/// the host's clock starts - then the host.
template <typename Host, typename Client, I2sPins host_pins, I2sPins client_pins>
bool pair_up(const I2sConfig& host_cfg, const I2sConfig& client_cfg) {
    Host::bus_clock(true);
    Host::reset();
    Client::bus_clock(true);
    Client::reset();
    const bool pads = Client::template claim_pads<client_pins>(client_cfg.mode) &&
                      Host::template claim_pads<host_pins>(host_cfg.mode);
    const bool cfg = Client::configure(client_cfg) && Host::configure(host_cfg);
    Client::enable();
    return pads && cfg;
}

void pair_down() {
    (void)I2::disable();
    (void)I3::disable();
    all_off();
}

const char* standard_name(I2sStandard s, bool long_frame) {
    switch (s) {
        case I2sStandard::philips: return "Philips";
        case I2sStandard::msb_justified: return "MSB-justified";
        case I2sStandard::lsb_justified: return "LSB-justified";
        case I2sStandard::pcm: return long_frame ? "PCM long frame" : "PCM short frame";
    }
    return "?";
}

// ===========================================================================
// a - the register face
// ===========================================================================

/// A 16 kHz frame's divider for letter a's TXE check.
constexpr I2sPrescaler pre_face = *i2s_prescaler_for(i2s_hz, 16'000, false, false);

template <typename F>
bool face_round_trip(const I2sConfig& c) {
    F::bus_clock(true);
    F::reset();
    const bool took = F::configure(c);
    const bool back = (F::regs().I2SCFGR & 0x0FBFu) == i2s_i2scfgr_of(c) &&
                      (F::regs().I2SPR & 0x03FFu) == i2s_i2spr_of(c) && F::i2s_mode_selected() &&
                      F::mode() == c.mode && F::standard() == c.standard;
    return took && back;
}

void ta_face() {
    all_off();
    // Every field, on both instances.
    const I2sConfig shapes[4] = {
        I2sConfig{.mode = I2sMode::host_transmit, .div = 47},
        I2sConfig{.mode = I2sMode::host_receive, .standard = I2sStandard::msb_justified,
                  .data = I2sDataLength::bits24, .channel = I2sChannelLength::bits32,
                  .clock_idle_high = true, .div = 70, .odd = true},
        I2sConfig{.mode = I2sMode::client_transmit, .standard = I2sStandard::lsb_justified,
                  .data = I2sDataLength::bits32, .channel = I2sChannelLength::bits32},
        I2sConfig{.mode = I2sMode::host_transmit, .standard = I2sStandard::pcm,
                  .pcm_long_frame = true, .channel = I2sChannelLength::bits32,
                  .master_clock_out = true, .div = 255, .odd = true},
    };
    uint32_t round2 = 0;
    uint32_t round3 = 0;
    for (const I2sConfig& c : shapes) {
        if (face_round_trip<I2>(c)) {
            ++round2;
        }
        if (face_round_trip<I3>(c)) {
            ++round3;
        }
    }
    print(serial, "  four configurations read back whole: ", round2, " on I2S2, ", round3,
          " on I2S3 (MCKOE among them, no pad handed to it)", crlf);
    bench.verdict("every field of I2S_CFGR and I2SPR is written and read back on both instances, "
                  "the master clock's enable among them",
                  round2 == 4u && round3 == 4u);

    // TXE on a transmitter whose face is configured and not enabled, and
    // once it is. 20.3.6.2 says the flag is 0 while I2SE is 0; this letter
    // reads what the silicon holds, and letter b where a datum written
    // then goes.
    I2::bus_clock(true);
    I2::reset();
    const bool txe_at_reset = I2::tx_empty();
    (void)I2::configure(I2sConfig{.mode = I2sMode::host_transmit, .div = pre_face.div,
                                  .odd = pre_face.odd});
    const bool txe_before = I2::tx_empty();
    I2::enable();
    wait_us(100);
    const bool txe_after = I2::tx_empty();
    (void)I2::disable();
    const bool txe_disabled_again = I2::tx_empty();
    print(serial, "  TXE: ", txe_at_reset ? "1" : "0", " out of reset (the SPI face), ",
          txe_before ? "1" : "0", " with the I2S face configured and I2SE clear, ",
          txe_after ? "1" : "0", " once enabled, ", txe_disabled_again ? "1" : "0",
          " after the disable", crlf);
    bench.verdict("TXE READS 1 on a configured transmitting face with I2SE clear - not the 0 "
                  "20.3.6.2 states - and stays 1 through the enable and the disable",
                  txe_at_reset && txe_before && txe_after && txe_disabled_again);

    // The two faces, and the prescaler's own rule.
    I2::bus_clock(true);
    I2::reset();
    (void)I2::configure(I2sConfig{.div = 10});
    I2::enable();
    const bool refused_live = !I2::prescaler(20, true) && I2::prescaler_div() == 10u;
    (void)I2::disable();
    const bool taken_idle = I2::prescaler(20, true) && I2::prescaler_div() == 20u &&
                            I2::prescaler_odd();
    I2::select_spi_mode();
    const bool spi_again = !I2::i2s_mode_selected() && !I2::enabled();
    I2::bus_clock(false);
    bench.verdict("the prescaler is refused while the face is enabled and taken once it is not, "
                  "and I2SMOD hands the block back to the SPI face",
                  refused_live && taken_idle && spi_again);

    // SPI1 has no face: its I2S_CFGR is the one question the SPI face asks.
    Spi<1>::bus_clock(true);
    Spi<1>::reset();
    const bool spi1_bit = Spi<1>::i2s_config_writable();
    Spi<1>::bus_clock(false);
    print(serial, "  SPI1's I2SMOD ", spi1_bit ? "TAKES a write" : "reads back zero",
          " (table 20-1 gives SPI1 no I2SPR)", crlf);

    // The clock and the arithmetic.
    print(serial, "  the I2S clock is SYSCLK: ", i2s_hz, " Hz", crlf);
    const uint32_t rates[6] = {11'025, 16'000, 22'050, 32'000, 44'100, 48'000};
    uint32_t reached = 0;
    for (uint32_t fs : rates) {
        const auto p16 = i2s_prescaler_for(i2s_hz, fs, false, false);
        const auto p32 = i2s_prescaler_for(i2s_hz, fs, true, false);
        const auto pmck = i2s_prescaler_for(i2s_hz, fs, false, true);
        print(serial, "  ", fs, " Hz:");
        if (p16) {
            const uint32_t got_fs = i2s_fs_hz(i2s_hz, p16->div, p16->odd, false, false);
            print(serial, " 16-bit ", p16->div, p16->odd ? "+odd" : "", " -> ", got_fs);
            ++reached;
        }
        if (p32) {
            print(serial, ", 32-bit ", p32->div, p32->odd ? "+odd" : "", " -> ",
                  i2s_fs_hz(i2s_hz, p32->div, p32->odd, true, false));
        }
        if (pmck) {
            print(serial, ", with MCK ", pmck->div, pmck->odd ? "+odd" : "", " -> ",
                  i2s_fs_hz(i2s_hz, pmck->div, pmck->odd, false, true));
        }
        print(serial, crlf);
        console_drain();
    }
    bench.verdict("20.3.3's divider reaches every usual audio rate from 11025 Hz to 48 kHz in a "
                  "16-bit frame off a 144 MHz SYSCLK",
                  reached == 6u);
    all_off();
}

// ===========================================================================
// b - the four standards, host transmit into client receive
// ===========================================================================

struct Shape {
    I2sStandard standard;
    bool long_frame;
};

constexpr uint32_t fs_asked = 16'000;
/// The dividers of the two rates this suite runs, at compile time: a rate
/// the ladder cannot reach would not build.
constexpr I2sPrescaler pre16 = *i2s_prescaler_for(i2s_hz, fs_asked, false, false);
constexpr I2sPrescaler pre48 = *i2s_prescaler_for(i2s_hz, 48'000, false, false);

/// One run of the pair in a shape: the words matched, the channel side
/// alternating where the standard has one, and the rate the receiver
/// counted.
template <typename Host, typename Client, I2sPins host_pins, I2sPins client_pins>
bool shape_run(const Shape& s, I2sDataLength data, I2sChannelLength channel, bool print_rate) {
    const bool wide = data != I2sDataLength::bits16 || channel == I2sChannelLength::bits32;
    const auto pre = i2s_prescaler_for(i2s_hz, fs_asked, wide, false);
    if (!pre) {
        return false;
    }
    const I2sConfig host_cfg{.mode = I2sMode::host_transmit, .standard = s.standard,
                             .pcm_long_frame = s.long_frame, .data = data, .channel = channel,
                             .div = pre->div, .odd = pre->odd};
    const I2sConfig client_cfg{.mode = I2sMode::client_receive, .standard = s.standard,
                               .pcm_long_frame = s.long_frame, .data = data, .channel = channel};
    const bool up = pair_up<Host, Client, host_pins, client_pins>(host_cfg, client_cfg);
    // The host is enabled first and fed on TXE from its first ask - one
    // order for every run, whatever a disabled face does with a datum
    // written early (letter b measures that apart).
    Host::enable();
    const uint16_t n = stream<Host, Client>(stream_words, 200'000UL);
    const bool two = data != I2sDataLength::bits16;
    const uint16_t second_mask = data == I2sDataLength::bits24 ? 0xFF00u : 0xFFFFu;
    const Judgement j = judge(n, second_mask, two);
    // Each channel is one word, or two where the datum is wider than 16
    // bits; the side alternates per channel except under PCM.
    uint16_t side_ok = 0;
    const uint16_t per = two ? 2u : 1u;
    for (uint16_t i = per; i < n; i = static_cast<uint16_t>(i + per)) {
        if (got_right[i] != got_right[i - per]) {
            ++side_ok;
        }
    }
    // What the 24-bit datum's second access carries below its byte: the
    // low byte of every second word, counted where it reads zero.
    uint16_t low_zero = 0;
    uint16_t low_words = 0;
    if (data == I2sDataLength::bits24 && j.offset >= 0) {
        for (uint16_t i = 0; i < n; ++i) {
            if (((static_cast<uint32_t>(j.offset) + i) & 1u) != 0u) {
                ++low_words;
                if ((got[i] & 0x00FFu) == 0u) {
                    ++low_zero;
                }
            }
        }
    }
    uint32_t rate = 0;
    const uint32_t want = i2s_fs_hz(i2s_hz, host_cfg);
    if (print_rate) {
        // Half a second of words: one word either way is two frames a
        // second at most, inside the tolerance below.
        rate = word_rate<Host, Client>(500'000UL) / (two ? 4u : 2u);
    }
    (void)Host::disable();
    (void)Client::disable();
    print(serial, "    ", standard_name(s.standard, s.long_frame), ", ",
          data == I2sDataLength::bits16 ? 16 : data == I2sDataLength::bits24 ? 24 : 32,
          "-bit in ", wide ? 32 : 16, ": ", j.matched, " of ", j.words,
          " words matched from pattern word ", j.offset, ", the side flipped ", side_ok,
          " times");
    if (print_rate) {
        print(serial, ", ", rate, " frames a second counted for ", want, " asked");
    }
    print(serial, crlf);
    print(serial, "      the first word received on the ", got_right[0] ? "RIGHT" : "left",
          " channel");
    if (data == I2sDataLength::bits24) {
        print(serial, "; the low byte of the datum's second access zero in ", low_zero, " of ",
              low_words);
    }
    print(serial, crlf);
    console_drain();
    const bool sides = s.standard == I2sStandard::pcm || side_ok + 2u >= n / per;
    const bool rate_ok = !print_rate || (rate + 3u >= want && rate <= want + 3u);
    all_off();
    return up && n == stream_words && j.offset >= 0 && j.matched == n && sides && rate_ok;
}

void tb_standards() {
    all_off();
    if (!wired()) {
        bench.verdict("the standards want the three wires, and say so", true);
        return;
    }
    const Shape shapes[5] = {{I2sStandard::philips, false}, {I2sStandard::msb_justified, false},
                             {I2sStandard::lsb_justified, false}, {I2sStandard::pcm, false},
                             {I2sStandard::pcm, true}};
    uint32_t clean = 0;
    for (const Shape& s : shapes) {
        if (shape_run<I2, I3, pins2, pins3>(s, I2sDataLength::bits16, I2sChannelLength::bits16,
                                            s.standard == I2sStandard::philips)) {
            ++clean;
        }
    }
    bench.verdict("I2S2 a host transmitting into I2S3 a client receiving: Philips, MSB- and "
                  "LSB-justified and PCM in both frames carry the pattern word for word, the side "
                  "alternating where the standard has one, and the frame rate counted against the "
                  "core's counter is 20.3.3's",
                  clean == 5u);

    // A DATUM WRITTEN BEFORE THE ENABLE. TXE reads 1 on a configured face
    // with I2SE clear (letter a), so a write there is taken by something:
    // the receiver says whether it is the first word on the wire or lost.
    constexpr uint16_t preload = 0xA5C3u;
    const bool up = pair_up<I2, I3, pins2, pins3>(
        I2sConfig{.mode = I2sMode::host_transmit, .div = pre16.div, .odd = pre16.odd},
        I2sConfig{.mode = I2sMode::client_receive});
    I2::data(preload);
    I2::enable();
    const uint16_t n = stream<I2, I3>(stream_words, 200'000UL);
    (void)I2::disable();
    (void)I3::disable();
    const bool first_is_preload = n >= 2u && got[0] == preload;
    int32_t first_pattern = -1;
    for (uint32_t k = 0; k < 8u && first_pattern < 0; ++k) {
        if (n >= 2u && got[first_is_preload ? 1u : 0u] == pattern(k)) {
            first_pattern = static_cast<int32_t>(k);
        }
    }
    print(serial, "  a word written with I2SE clear, then the enable: the receiver's first word ",
          hex(got[0]), first_is_preload ? " - THE PRELOADED DATUM," : " - not the preloaded one,",
          " the pattern following from its word ", first_pattern, crlf);
    bench.verdict("a host transmitter's datum written while I2SE is clear is the first word on "
                  "the wire once the face is enabled, the pattern fed on TXE following from its "
                  "first word",
                  up && first_is_preload && first_pattern == 0);
    all_off();
}

// ===========================================================================
// c - the widths
// ===========================================================================

void tc_widths() {
    all_off();
    if (!wired()) {
        bench.verdict("the widths want the three wires, and say so", true);
        return;
    }
    const Shape philips{I2sStandard::philips, false};
    const bool w16in32 = shape_run<I2, I3, pins2, pins3>(philips, I2sDataLength::bits16,
                                                         I2sChannelLength::bits32, true);
    const bool w24 = shape_run<I2, I3, pins2, pins3>(philips, I2sDataLength::bits24,
                                                     I2sChannelLength::bits32, true);
    const bool w32 = shape_run<I2, I3, pins2, pins3>(philips, I2sDataLength::bits32,
                                                     I2sChannelLength::bits32, true);
    bench.verdict("a 16-bit datum in a 32-bit channel crosses in one access a channel",
                  w16in32);
    bench.verdict("24- and 32-bit data cross in two accesses a channel, the high half first, a "
                  "24-bit datum's last byte left out of the comparison as 20.3.2 leaves it",
                  w24 && w32);
    all_off();
}

// ===========================================================================
// d - the roles swapped
// ===========================================================================

void td_swapped() {
    all_off();
    if (!wired()) {
        bench.verdict("the swapped roles want the three wires, and say so", true);
        return;
    }
    const bool philips = shape_run<I3, I2, pins3, pins2>({I2sStandard::philips, false},
                                                         I2sDataLength::bits16,
                                                         I2sChannelLength::bits16, true);
    const bool pcm = shape_run<I3, I2, pins3, pins2>({I2sStandard::pcm, true},
                                                     I2sDataLength::bits16,
                                                     I2sChannelLength::bits32, false);
    bench.verdict("I2S3 a host transmitting into I2S2 a client receiving: Philips and PCM's "
                  "long frame word for word, the rate counted",
                  philips && pcm);
    all_off();
}

// ===========================================================================
// e - a host receiving from a client transmitting
// ===========================================================================

void te_host_receive() {
    all_off();
    if (!wired()) {
        bench.verdict("a host receiving wants the three wires, and says so", true);
        return;
    }
    const I2sConfig host_cfg{.mode = I2sMode::host_receive, .div = pre16.div, .odd = pre16.odd};
    const I2sConfig client_cfg{.mode = I2sMode::client_transmit};
    // The client transmitter is fed ONE DATUM AHEAD: its first word is in
    // the data register before the host's clock can arrive (20.3.5.1).
    const bool up = pair_up<I2, I3, pins2, pins3>(host_cfg, client_cfg);
    I3::data(pattern(0));
    uint32_t oi = 1;
    I2::enable();
    uint16_t ii = 0;
    uint32_t busy_seen = 0;
    Stopwatch w;
    while (ii < stream_words && w.us() < 200'000UL) {
        if (I3::tx_empty()) {
            I3::data(pattern(oi++));
        }
        if (I2::busy()) {
            ++busy_seen;
        }
        if (I2::rx_ready()) {
            got_right[ii] = I2::right_channel();
            got[ii] = I2::data();
            ++ii;
            // 20.3.4.3's stop, at the word it names: for a 16-bit datum in
            // a 16-bit channel, the second-to-last RXNE, then one I2S
            // clock period, then I2SE cleared.
            if (ii == stream_words - 1u) {
                const I2sReceiveStop stop = i2s_receive_stop(host_cfg);
                const uint32_t period_us =
                    1'000'000UL / i2s_bit_clock_hz(i2s_hz, host_cfg) + 1u;
                wait_us(period_us * stop.clock_periods);
                (void)I2::disable();
            }
        }
    }
    const Judgement j = judge(ii);
    const bool underrun = I3::underrun();
    print(serial, "  I2S2 host receiving from I2S3 client transmitting: ", ii, " words, ",
          j.matched, " matched from pattern word ", j.offset, "; BSY read set ", busy_seen,
          " times during the reception; the client ", underrun ? "UNDERRAN" : "never underran",
          crlf);
    print(serial, "  the client's first datum reached the host as its first word, on the ",
          got_right[0] ? "RIGHT" : "left", " channel", crlf);
    bench.verdict("a host receives what a client fed one datum ahead transmits, word for word, "
                  "and 20.3.4.3's stop ends the reception on the word it names",
                  up && j.offset >= 0 && j.matched + 1u >= ii && ii + 1u >= stream_words);
    bench.verdict("BSY stays low through a master reception (20.3.6.1)", busy_seen == 0u);
    pair_down();
}

// ===========================================================================
// f - the flags and the vectors
// ===========================================================================

void tf_flags() {
    all_off();
    if (!wired()) {
        bench.verdict("the flags want the three wires, and say so", true);
        return;
    }

    // The overrun: a receiver nobody reads.
    (void)pair_up<I2, I3, pins2, pins3>(
        I2sConfig{.mode = I2sMode::host_transmit, .div = pre16.div, .odd = pre16.odd},
        I2sConfig{.mode = I2sMode::client_receive});
    I3::error_interrupt(true);
    Pfic::enable(I3::irq);
    I2::enable();
    Stopwatch w;
    while (w.us() < 5000u) {
        if (I2::tx_empty()) {
            I2::data(0x5555u);
        }
    }
    const bool ovr = I3::overrun();
    const uint32_t ovr_vec = vec3;
    const uint32_t ovr_flags = vec3_flags;
    // The host stopped first: a stream still arriving would raise the
    // flag again between the two reads that clear it.
    (void)I2::disable();
    I3::clear_overrun();
    const bool ovr_cleared = !I3::overrun();
    print(serial, "  a receiver left unread: OVR ", ovr ? "set" : "NOT SET", ", its vector ran ",
          ovr_vec, " times with flags ", hex(ovr_flags), crlf);
    pair_down();

    // The underrun: a client transmitter nobody feeds.
    (void)pair_up<I2, I3, pins2, pins3>(
        I2sConfig{.mode = I2sMode::host_receive, .div = pre16.div, .odd = pre16.odd},
        I2sConfig{.mode = I2sMode::client_transmit});
    I3::error_interrupt(true);
    Pfic::enable(I3::irq);
    I2::enable();
    w.start();
    while (w.us() < 5000u) {
        if (I2::rx_ready()) {
            (void)I2::data();
        }
    }
    const bool udr = I3::underrun();
    const uint32_t udr_vec = vec3;
    const uint32_t udr_flags = vec3_flags;
    I3::clear_underrun();
    print(serial, "  a client transmitter never fed: UDR ", udr ? "set" : "NOT SET",
          ", its vector ran ", udr_vec, " times with flags ", hex(udr_flags), crlf);
    pair_down();

    // RXNE and TXE through the vectors of both instances.
    (void)pair_up<I2, I3, pins2, pins3>(
        I2sConfig{.mode = I2sMode::host_transmit, .div = pre16.div, .odd = pre16.odd},
        I2sConfig{.mode = I2sMode::client_receive});
    I2::txe_interrupt(true);
    I3::rxne_interrupt(true);
    Pfic::enable(I2::irq);
    Pfic::enable(I3::irq);
    I2::enable();
    wait_us(2000);
    const uint32_t tx_vec = vec2;
    const uint32_t rx_vec = vec3;
    print(serial, "  2 ms of a 16 kHz stream: I2S2's vector ran ", tx_vec, " times on TXE, "
          "I2S3's ", rx_vec, " times on RXNE", crlf);
    pair_down();

    bench.verdict("OVR stands on a receiver nobody reads, reaches the vector under ERRIE and "
                  "goes down with the DATAR-then-STATR read",
                  ovr && ovr_vec >= 1u && (ovr_flags & SpiFlag::overrun) != 0u && ovr_cleared);
    bench.verdict("UDR stands on a client transmitter nobody feeds and reaches the vector under "
                  "ERRIE",
                  udr && udr_vec >= 1u && (udr_flags & SpiFlag::underrun) != 0u);
    bench.verdict("TXE and RXNE reach their instances' vectors under their own enables, some "
                  "sixty of each in two milliseconds of a 16 kHz stream",
                  tx_vec >= 40u && rx_vec >= 40u);
    all_off();
}

// ===========================================================================
// g - the DMA requests
// ===========================================================================

constexpr uint16_t dma_words = 256;
uint16_t dma_out[dma_words];
uint16_t dma_in[dma_words];

void tg_dma() {
    all_off();
    if (!wired()) {
        bench.verdict("the DMA requests want the three wires, and say so", true);
        return;
    }
    using Out = DmaChannel<I2::dma_tx_slot.controller, I2::dma_tx_slot.channel>;
    using In = DmaChannel<I3::dma_rx_slot.controller, I3::dma_rx_slot.channel>;
    for (uint16_t i = 0; i < dma_words; ++i) {
        dma_out[i] = pattern(i);
        dma_in[i] = 0;
    }
    const I2sConfig host_cfg{.mode = I2sMode::host_transmit, .div = pre48.div, .odd = pre48.odd,
                             .dma_transmit = true};
    const I2sConfig client_cfg{.mode = I2sMode::client_receive, .dma_receive = true};
    const DmaChannelConfig to_block{.direction = DmaDirection::peripheral_to_memory,
                                    .peripheral_width = DmaWidth::half,
                                    .memory_width = DmaWidth::half,
                                    .priority = DmaPriority::very_high};
    const DmaChannelConfig from_block{.direction = DmaDirection::memory_to_peripheral,
                                      .peripheral_width = DmaWidth::half,
                                      .memory_width = DmaWidth::half,
                                      .priority = DmaPriority::high};
    const bool loaded_in = In::load({.peripheral = I3::data_address(), .memory = dma_in,
                                     .count = dma_words, .config = to_block});
    const bool up = pair_up<I2, I3, pins2, pins3>(host_cfg, client_cfg);
    const bool loaded_out = Out::load({.peripheral = I2::data_address(), .memory = dma_out,
                                       .count = dma_words, .config = from_block});
    Stopwatch w;
    I2::enable();
    while (!In::flag(DmaFlag::complete) && w.us() < 50'000UL) {
    }
    const uint32_t us = w.us();
    const bool done = In::flag(DmaFlag::complete);
    Out::stop();
    In::stop();
    pair_down();
    // The receiver may join the stream a word late: judged as any run is.
    for (uint16_t i = 0; i < stream_words; ++i) {
        got[i] = dma_in[i];
    }
    const Judgement j = judge(stream_words);
    print(serial, "  ", dma_words, " half-words at 48 kHz: ", done ? "the block completed" :
          "the block DID NOT complete", " in ", us, " us, the first ", stream_words, " judged: ",
          j.matched, " matched from pattern word ", j.offset, crlf);
    bench.verdict("I2S2's transmit request on DMA1's channel 5 and I2S3's receive request on "
                  "DMA2's channel 1 - the SPI face's two slots - carry a block of half-words word "
                  "for word",
                  up && loaded_in && loaded_out && done && j.offset >= 0 &&
                      j.matched == stream_words);
    all_off();
}

// ---- the menu ---------------------------------------------------------------

void banner() {
    print(serial, crlf, "test_vx03_i2s - the I2S face of SPI2 and SPI3 on ", device::part_name,
          crlf, "  WS PB12-PA15, CK PB13-PB3, SD PB15-PB5 (each looked for); MCK left off", crlf);
    bench.menu();
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

/// Each instance's vector: counted, the raised sources collected, and the
/// enable of what was raised dropped so a standing source cannot spin it.
extern "C" BRIO_CH32_INTERRUPT void spi2_handler() {
    const uint32_t up = I2::isr();
    vec2 = vec2 + 1u;
    vec2_flags = vec2_flags | up;
    if ((up & brio::SpiFlag::txe) != 0u) {
        I2::data(0x5A5Au);
    }
    if ((up & brio::SpiFlag::rxne) != 0u) {
        (void)I2::data();
    }
    if ((up & brio::SpiFlag::i2s_errors) != 0u) {
        I2::error_interrupt(false);
    }
}

extern "C" BRIO_CH32_INTERRUPT void spi3_handler() {
    const uint32_t up = I3::isr();
    vec3 = vec3 + 1u;
    vec3_flags = vec3_flags | up;
    if ((up & brio::SpiFlag::txe) != 0u) {
        I3::data(0x5A5Au);
    }
    if ((up & brio::SpiFlag::rxne) != 0u) {
        (void)I3::data();
    }
    if ((up & brio::SpiFlag::i2s_errors) != 0u) {
        I3::error_interrupt(false);
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the register face, the clock and the dividers, with no wire", ta_face);
    bench.letter('b', "the four standards: I2S2 host transmit into I2S3 client receive",
                 tb_standards);
    bench.letter('c', "the widths: 16 in 32, 24 in 32, 32 in 32", tc_widths);
    bench.letter('d', "the roles swapped: I2S3 host transmit into I2S2 client receive",
                 td_swapped);
    bench.letter('e', "a host receiving from a client fed one datum ahead", te_host_receive);
    bench.letter('f', "the flags and the vectors: OVR, UDR, TXE, RXNE", tf_flags);
    bench.letter('g', "the DMA requests: a block of half-words at 48 kHz", tg_dma);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED",
              " tick=", tick_ok ? "STK" : "FAILED", crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
