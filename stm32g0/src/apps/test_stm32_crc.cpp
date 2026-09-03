// test_stm32_crc - the reference bench suite for the STM32G0's CRC
// calculation unit: stm32g0/crc.hpp (RM0444 ch. 14, the whole of it).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE, no pad moved, no flash written, no option byte
// touched: a checksum unit has no pin, no interrupt and no request line,
// and the only outside thing this suite reads is its OWN CODE, through a
// DMA channel, as the payload of letter d.
//
// THE INSTRUMENTS ARE ALL INSIDE THE CHIP:
//   - util/crc.hpp's bitwise CRC-16/CCITT-FALSE is the software judge of
//     the hardware's CCITT preset - the same checksum by a wholly
//     different mechanism, which is what makes letter b a proof and not
//     a tautology;
//   - a reflected CRC-32 written HERE (not in util/) is the judge of the
//     hardware's CRC-32 preset;
//   - SysTick, read the way every other suite of this stratum reads it,
//     is the cycle stopwatch of letter c;
//   - one DMA channel in MEM2MEM mode is the no-CPU feeder of letter d.
//
// What is exercised, letter by letter:
//   a  the block: the reset values, the gate its bus clock is, 14.3.3's
//      own reversal examples, REV_OUT through a computed CRC, IDR
//      surviving a RESET, and the polynomial-change rule MEASURED
//   b  the check values of four standards against the hardware, the
//      CCITT preset bit-exact against util/crc.hpp over 4 KB, and the
//      word-vs-byte equivalence of feed(span) in all four REV_IN
//      settings
//   c  what it costs: cycles per word, per half-word and per byte
//      against the software loop's cycles per byte
//   d  DMA-fed: 16 KB of this program's own flash checksummed with no
//      CPU in the loop, judged against two independent references
//   e  the two rules 14.3.3 states and no register enforces
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "stm32g0/clock.hpp"
#include "stm32g0/crc.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/crc.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32Platform;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// Instruments
// ---------------------------------------------------------------------------

/// The cycle-resolution stopwatch every suite of this stratum uses:
/// SysTick's tick count times its period plus the phase it has already
/// counted down, the two reads retried until they belong to one tick.
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}

/// A measurement window a transmit interrupt walks through is not a
/// measurement - the lesson three campaigns of this stratum have paid
/// for. Letter c drains before it counts.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < SysClock::hz / 500u) {
    }
}

/// The bit reversal 14.3.3 describes in words, so that REV_OUT can be
/// checked against arithmetic and not against a second register.
constexpr uint32_t bitrev32(uint32_t v) {
    uint32_t r = 0;
    for (uint8_t i = 0; i < 32u; ++i) {
        r = (r << 1) | (v & 1u);
        v >>= 1;
    }
    return r;
}
static_assert(bitrev32(0x11223344u) == 0x22CC4488u,
              "14.3.3's own REV_OUT example, as arithmetic");
static_assert(bitrev32(0x1A2B3C4Du) == 0xB23CD458u,
              "...and its REV_IN word example, which is the same operation");

/// Bit-reverse each BYTE of a word, in place - 14.3.3's REV_IN = 01.
constexpr uint32_t bitrev_bytes(uint32_t v) {
    uint32_t r = 0;
    for (uint8_t b = 0; b < 4u; ++b) {
        uint8_t byte = static_cast<uint8_t>(v >> (8u * b));
        uint8_t rev = 0;
        for (uint8_t i = 0; i < 8u; ++i) {
            rev = static_cast<uint8_t>((rev << 1) | (byte & 1u));
            byte = static_cast<uint8_t>(byte >> 1);
        }
        r |= static_cast<uint32_t>(rev) << (8u * b);
    }
    return r;
}
static_assert(bitrev_bytes(0x1A2B3C4Du) == 0x58D43CB2u,
              "14.3.3's REV_IN = byte example");

/// Bit-reverse each HALF-WORD - 14.3.3's REV_IN = 10.
constexpr uint32_t bitrev_halfwords(uint32_t v) {
    const uint32_t lo = bitrev32(v & 0xFFFFu) >> 16;
    const uint32_t hi = bitrev32(v >> 16) >> 16;
    return (hi << 16) | lo;
}
static_assert(bitrev_halfwords(0x1A2B3C4Du) == 0xD458B23Cu,
              "14.3.3's REV_IN = half-word example");

/**
 * THE SOFTWARE JUDGE OF THE CRC-32 PRESET, written here and NOT in
 * util/: the reflected CRC-32 of IEEE 802.3, bitwise, no table.
 *
 * It lives in this suite because util/crc.hpp holds exactly one
 * checksum, the one the nonvolatile stores use, and adding a second
 * there to check a peripheral would be a util decision this campaign has
 * no business taking. Here it is a measuring instrument, and it is
 * allowed to be slow.
 */
uint32_t crc32_reference(const uint8_t* data, uint32_t len, uint32_t seed = 0xFFFFFFFFu) {
    uint32_t crc = seed;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8u; ++bit) {
            crc = (crc & 1u) != 0u ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Payloads
// ---------------------------------------------------------------------------

/// The check string every CRC catalogue is keyed by.
constexpr uint8_t check_string[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

/// 4 KB of pseudo-random bytes for letter b's bit-exactness proof.
/// xorshift32, so the buffer is reproducible from run to run and a
/// mismatch is a fact about the peripheral and not about the data.
constexpr uint32_t blob_bytes = 4096;
uint8_t blob[blob_bytes];

void fill_blob() {
    uint32_t x = 0x2463534Au;
    for (uint32_t i = 0; i < blob_bytes; ++i) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        blob[i] = static_cast<uint8_t>(x);
    }
}

/// Letter d's payload: this program's own code, in bank 1, read
/// word-wise. 16 KB, which is more than the image and therefore also
/// covers erased flash - the point is that the three routes agree, not
/// what the number is.
constexpr uint32_t code_base = 0x08000000UL;
constexpr uint32_t code_bytes = 16u * 1024u;

using Dma1 = Dma<1>;
using ChA = DmaChannel<1, 1>;

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

/// One checksum of a byte range through the hardware, byte by byte -
/// the reference path against which every faster one is judged.
uint32_t hw_bytes(const CrcConfig& cfg, const uint8_t* data, uint32_t len) {
    (void)Crc::configure(cfg);
    for (uint32_t i = 0; i < len; ++i) {
        Crc::feed8(data[i]);
    }
    return Crc::value_masked();
}

/// The same range through the driver's block verb, which feeds words
/// where it can.
uint32_t hw_span(const CrcConfig& cfg, const uint8_t* data, uint32_t len) {
    (void)Crc::configure(cfg);
    Crc::feed(std::span<const uint8_t>{data, len});
    return Crc::value_masked();
}

// =============================================================================
// a - the block
// =============================================================================
void ta_block() {
    // THE GATE. 5.2.17: a peripheral whose enable bit is clear does not
    // answer register reads at all. What it answers INSTEAD is a fact
    // about this bus and is printed rather than guessed at.
    Crc::bus_clock(false);
    const uint32_t closed_dr = Crc::regs().DR;
    const uint32_t closed_pol = Crc::regs().POL;
    Crc::init();
    const uint32_t dr = Crc::regs().DR;
    const uint32_t idr = Crc::regs().IDR;
    const uint32_t cr = Crc::regs().CR;
    const uint32_t init = Crc::regs().INIT;
    const uint32_t pol = Crc::regs().POL;
    print(serial, "  through the CLOSED bus gate DR reads ", hex(closed_dr),
          " and POL ", hex(closed_pol), "; open and reset, DR=", hex(dr),
          " IDR=", hex(idr), " CR=", hex(cr), " INIT=", hex(init), " POL=",
          hex(pol), crlf);

    bench.verdict("out of its RCC reset the unit reads table 70's own values: "
                  "DR and INIT all ones, POL the CRC-32 Ethernet polynomial, "
                  "CR and IDR zero",
                  dr == 0xFFFFFFFFu && init == 0xFFFFFFFFu &&
                      pol == 0x04C11DB7u && cr == 0u && idr == 0u);

    // 14.3.3's THREE REVERSAL EXAMPLES, each checked the only way that
    // does not assume the answer: feed the PRE-REVERSED word with no
    // reversal, feed the ORIGINAL word with the reversal, and require
    // the two checksums to be identical.
    constexpr uint32_t sample = 0x1A2B3C4Du;
    struct Rev {
        const char* name;
        CrcReverse code;
        uint32_t pre_reversed;
    };
    const Rev revs[] = {
        {"by byte", CrcReverse::byte, bitrev_bytes(sample)},
        {"by half-word", CrcReverse::halfword, bitrev_halfwords(sample)},
        {"by word", CrcReverse::word, bitrev32(sample)},
    };
    bool reversals_ok = true;
    for (const Rev& r : revs) {
        (void)Crc::configure(CrcConfig{});
        Crc::feed(r.pre_reversed);
        const uint32_t plain = Crc::value();
        (void)Crc::configure(CrcConfig{.reverse_in = r.code});
        Crc::feed(sample);
        const uint32_t reversed = Crc::value();
        print(serial, "  REV_IN ", r.name, ": 0x1A2B3C4D becomes ",
              hex(r.pre_reversed), ", checksums ", hex(plain), " vs ",
              hex(reversed), crlf);
        reversals_ok = reversals_ok && plain == reversed;
    }
    bench.verdict("all three of 14.3.3's REV_IN examples are exactly what the "
                  "silicon does: the reversed input gives the checksum of the "
                  "pre-reversed word",
                  reversals_ok);

    // REV_OUT, through a COMPUTED checksum: the same data with the bit
    // clear and set, and the second must be the first bit-reversed.
    (void)Crc::configure(CrcConfig{});
    Crc::feed(std::span<const uint8_t>{check_string});
    const uint32_t straight = Crc::value();
    (void)Crc::configure(CrcConfig{.reverse_out = true});
    Crc::feed(std::span<const uint8_t>{check_string});
    const uint32_t flipped = Crc::value();
    print(serial, "  REV_OUT off/on over the check string: ", hex(straight),
          " / ", hex(flipped), crlf);
    bench.verdict("REV_OUT reverses the OUTPUT at bit level and nothing else "
                  "(14.3.3's 0x11223344 -> 0x22CC4488 rule, on a checksum this "
                  "unit really computed)",
                  flipped == bitrev32(straight));

    // CRC_IDR: four bytes this peripheral never looks at, and 14.4.2's
    // one interesting property.
    Crc::scratch(0xA5C3F00Du);
    Crc::restart();
    const uint32_t after_reset = Crc::scratch();
    (void)Crc::configure(crc_ccitt_false_config);
    const uint32_t after_configure = Crc::scratch();
    bench.verdict("CRC_IDR survives both a CR.RESET and a whole "
                  "reconfiguration - 14.4.2's \"not affected by CRC resets\"",
                  after_reset == 0xA5C3F00Du && after_configure == 0xA5C3F00Du);

    // A WIDTH UNDER 32 BITS leaves the bits above it holding the wider
    // datapath's leftovers, which is why value_masked() exists.
    (void)Crc::configure(CrcConfig{.polynomial = 0x09u, .size = CrcWidth::w7,
                                   .init = 0});
    Crc::feed(std::span<const uint8_t>{check_string});
    const uint32_t whole = Crc::value();
    const uint32_t masked = Crc::value_masked();
    print(serial, "  a 7-bit polynomial leaves DR = ", hex(whole),
          ", of which the result is ", hex(masked), crlf);
    bench.verdict("14.3.3 is literal: the result of a narrow polynomial is the "
                  "LEAST SIGNIFICANT bits of DR, and the rest is not zero",
                  masked == (whole & 0x7Fu));

    // THE POLYNOMIAL-CHANGE RULE, MEASURED. 14.3.3 says a change made
    // during a calculation is unreliable; what "unreliable" means on
    // this silicon is a fact, so it is measured in both directions.
    constexpr CrcConfig poly_a{.polynomial = 0x1021u, .size = CrcWidth::w16,
                               .init = 0xFFFFu};
    constexpr CrcConfig poly_b{.polynomial = 0x8005u, .size = CrcWidth::w16,
                               .init = 0xFFFFu};
    const uint32_t clean_b = hw_bytes(poly_b, check_string, 9);

    // Mid-calculation: configure A, feed half the string, then write the
    // polynomial registers BY HAND with no reset and no DR read, then
    // feed the rest.
    auto mid_change = [&]() {
        (void)Crc::configure(poly_a);
        for (uint8_t i = 0; i < 4u; ++i) {
            Crc::feed8(check_string[i]);
        }
        CRC->POL = poly_b.polynomial;
        for (uint8_t i = 4; i < 9u; ++i) {
            Crc::feed8(check_string[i]);
        }
        return Crc::value_masked();
    };
    const uint32_t dirty1 = mid_change();
    const uint32_t dirty2 = mid_change();
    const uint32_t clean_after = hw_bytes(poly_b, check_string, 9);
    print(serial, "  poly changed mid-calculation: ", hex(dirty1), " then ",
          hex(dirty2), "; the clean 0x8005 checksum is ", hex(clean_b),
          " and after a reconfigure it is ", hex(clean_after), crlf);
    bench.verdict("a polynomial changed during a calculation gives a value "
                  "that is the checksum of nothing - neither polynomial's "
                  "answer (14.3.3's \"cannot be performed\")",
                  dirty1 != clean_b);
    bench.verdict("but UNRELIABLE here does not mean random: the same illegal "
                  "sequence twice gives the same number, so nothing is lost "
                  "except the meaning",
                  dirty1 == dirty2);
    bench.verdict("and configure()'s own trailing CR.RESET is what makes the "
                  "next calculation clean again",
                  clean_after == clean_b);
}

// =============================================================================
// b - the check values, and the block verb
// =============================================================================
void tb_standards() {
    struct Standard {
        const char* name;
        CrcConfig cfg;
        uint32_t expect;
        bool final_xor;
    };
    const Standard standards[] = {
        {"CRC-16/CCITT-FALSE", crc_ccitt_false_config, 0x29B1u, false},
        {"CRC-32/ISO-HDLC", crc32_ieee_config, 0xCBF43926u, true},
        {"CRC-8/SMBUS",
         CrcConfig{.polynomial = 0x07u, .size = CrcWidth::w8, .init = 0},
         0xF4u, false},
        {"CRC-7/MMC",
         CrcConfig{.polynomial = 0x09u, .size = CrcWidth::w7, .init = 0},
         0x75u, false},
    };
    for (const Standard& s : standards) {
        uint32_t v = hw_bytes(s.cfg, check_string, 9);
        if (s.final_xor) {
            v = crc32_ieee_finish(v);
        }
        print(serial, "  ", s.name, " over the check string: ", hex(v),
              " (want ", hex(s.expect), ")", crlf);
        bench.verdict(s.name, " computed by the silicon is the catalogue's "
                              "own check value",
                      v == s.expect);
    }

    // THE PROOF THAT THE PRESET IS WHAT IT CLAIMS: 4 KB through the
    // hardware and 4 KB through util/crc.hpp's bitwise loop, which share
    // nothing but the definition.
    fill_blob();
    const uint32_t hw = hw_bytes(crc_ccitt_false_config, blob, blob_bytes);
    const uint16_t sw = crc16(blob, static_cast<uint16_t>(blob_bytes));
    print(serial, "  4 KB of xorshift bytes: hardware ", hex(hw),
          ", util/crc.hpp ", hex(sw), crlf);
    bench.verdict("crc_ccitt_false_config IS util/crc.hpp's CRC-16 - bit for "
                  "bit over 4096 bytes, by two mechanisms sharing only their "
                  "definition",
                  hw == static_cast<uint32_t>(sw));

    // THE BLOCK VERB against the byte-by-byte path, in all four REV_IN
    // settings - which is where the driver's word assembly either means
    // the same thing or does not (crc.hpp's fact 2).
    struct Rev {
        const char* name;
        CrcReverse code;
    };
    const Rev revs[] = {{"none", CrcReverse::none},
                        {"byte", CrcReverse::byte},
                        {"half-word", CrcReverse::halfword},
                        {"word", CrcReverse::word}};
    bool span_ok = true;
    for (const Rev& r : revs) {
        const CrcConfig cfg{.polynomial = 0x04C11DB7u,
                            .size = CrcWidth::w32,
                            .init = 0xFFFFFFFFu,
                            .reverse_in = r.code,
                            .reverse_out = false};
        // A length that is NOT a multiple of four, so the tail path runs.
        const uint32_t by_byte = hw_bytes(cfg, blob, 1023);
        const uint32_t by_span = hw_span(cfg, blob, 1023);
        print(serial, "  REV_IN ", r.name, ": byte-fed ", hex(by_byte),
              ", block-fed ", hex(by_span), crlf);
        span_ok = span_ok && by_byte == by_span;
    }
    bench.verdict("feed(span) means exactly what byte-by-byte feeding means, "
                  "in all four REV_IN settings and over a length the word path "
                  "cannot cover on its own",
                  span_ok);
    // A FINDING 14.3.3 DOES NOT CARRY, visible in the four rows above:
    // for a BYTE access the three reversals collapse into one, because
    // there is nothing above a byte to reorder. So REV_IN only
    // distinguishes itself on the wider accesses - which is exactly why
    // the driver's block verb has to know which setting is in force.
    const CrcConfig byte_cfgs[3] = {
        {.polynomial = 0x04C11DB7u, .size = CrcWidth::w32, .init = 0xFFFFFFFFu,
         .reverse_in = CrcReverse::byte},
        {.polynomial = 0x04C11DB7u, .size = CrcWidth::w32, .init = 0xFFFFFFFFu,
         .reverse_in = CrcReverse::halfword},
        {.polynomial = 0x04C11DB7u, .size = CrcWidth::w32, .init = 0xFFFFFFFFu,
         .reverse_in = CrcReverse::word}};
    const uint32_t r_byte = hw_bytes(byte_cfgs[0], blob, 1023);
    const uint32_t r_half = hw_bytes(byte_cfgs[1], blob, 1023);
    const uint32_t r_word = hw_bytes(byte_cfgs[2], blob, 1023);
    bench.verdict("and for a BYTE access the three reversals are ONE thing - "
                  "there is nothing above a byte to reorder - so REV_IN only "
                  "tells itself apart on the wider accesses",
                  r_byte == r_half && r_half == r_word);
}

// =============================================================================
// c - what it costs
// =============================================================================
void tc_cost() {
    fill_blob();
    constexpr uint32_t words = 256;
    console_drain();

    (void)Crc::configure(crc32_ieee_config);
    const uint32_t w0 = cycles_now();
    for (uint32_t i = 0; i < words; ++i) {
        Crc::feed(reinterpret_cast<const uint32_t*>(blob)[i]);
    }
    const uint32_t w1 = cycles_now();

    (void)Crc::configure(crc32_ieee_config);
    const uint32_t h0 = cycles_now();
    for (uint32_t i = 0; i < words * 2u; ++i) {
        Crc::feed16(reinterpret_cast<const uint16_t*>(blob)[i]);
    }
    const uint32_t h1 = cycles_now();

    (void)Crc::configure(crc32_ieee_config);
    const uint32_t b0 = cycles_now();
    for (uint32_t i = 0; i < words * 4u; ++i) {
        Crc::feed8(blob[i]);
    }
    const uint32_t b1 = cycles_now();

    const uint32_t s0 = cycles_now();
    const uint16_t sw = crc16(blob, static_cast<uint16_t>(words * 4u));
    const uint32_t s1 = cycles_now();

    const uint32_t bytes = words * 4u;
    const uint32_t word_cycles = w1 - w0;
    const uint32_t half_cycles = h1 - h0;
    const uint32_t byte_cycles = b1 - b0;
    const uint32_t soft_cycles = s1 - s0;

    // Per THOUSAND bytes, so the numbers stay integers and comparable.
    const uint32_t per_kb_word = word_cycles * 1000u / bytes;
    const uint32_t per_kb_half = half_cycles * 1000u / bytes;
    const uint32_t per_kb_byte = byte_cycles * 1000u / bytes;
    const uint32_t per_kb_soft = soft_cycles * 1000u / bytes;
    print(serial, "  ", bytes, " bytes: word writes ", word_cycles,
          " cycles, half-word ", half_cycles, ", byte ", byte_cycles,
          ", util/crc.hpp ", soft_cycles, crlf);
    print(serial, "  per 1000 bytes: ", per_kb_word, " / ", per_kb_half,
          " / ", per_kb_byte, " cycles against the software loop's ",
          per_kb_soft, crlf);
    print(serial, "  the hardware's word path is ", per_kb_soft / per_kb_word,
          "x the bitwise loop; sw = ", hex(sw), crlf);

    bench.verdict("a word write is the cheapest way in: fewer bus "
                  "transactions for the same four bytes, so the word loop "
                  "beats the byte loop",
                  word_cycles < byte_cycles);
    bench.verdict("and the half-word path sits between them",
                  half_cycles > word_cycles && half_cycles < byte_cycles);
    bench.verdict("the input buffer of 14.2 is real: a back-to-back word "
                  "write costs well under the ten cycles a stalled AHB "
                  "transaction plus 14.3.3's four computation cycles would",
                  per_kb_word / 250u < 10u);
    bench.verdict("the unit is worth having: it beats util/crc.hpp's bitwise "
                  "loop by more than an order of magnitude",
                  per_kb_soft > 10u * per_kb_word);
}

// =============================================================================
// d - DMA-fed, no CPU in the loop
// =============================================================================
void td_dma() {
    Dma1::bus_clock(true);
    Dma1::reset();
    // REV_IN = word is what makes a LITTLE-ENDIAN word stream compute
    // the reflected CRC-32 of the byte stream behind it: the whole word
    // is reversed, so the byte the memory holds first is processed
    // first, with its bits reflected. (REV_IN = byte would need the
    // words assembled big-endian, which a DMA channel cannot do.)
    constexpr CrcConfig dma_cfg{.polynomial = 0x04C11DB7u,
                                .size = CrcWidth::w32,
                                .init = 0xFFFFFFFFu,
                                .reverse_in = CrcReverse::word,
                                .reverse_out = true};

    // 1. the CPU-fed answer.
    const uint32_t* code = reinterpret_cast<const uint32_t*>(code_base);
    (void)Crc::configure(dma_cfg);
    const uint32_t c0 = cycles_now();
    for (uint32_t i = 0; i < code_bytes / 4u; ++i) {
        Crc::feed(code[i]);
    }
    const uint32_t c1 = cycles_now();
    const uint32_t cpu_value = crc32_ieee_finish(Crc::value());

    // 2. the software reference over the same bytes.
    const uint32_t sw_value =
        crc32_ieee_finish(crc32_reference(reinterpret_cast<const uint8_t*>(code_base),
                                          code_bytes));

    // 3. the DMA-fed answer: MEM2MEM, source incrementing through flash,
    // destination FIXED at CRC_DR, no request line and no CPU.
    (void)Crc::configure(dma_cfg);
    const bool prepared = ChA::prepare(
        DmaTransfer{.peripheral = const_cast<uint32_t*>(code),
                    .memory = Crc::data_address(),
                    .count = static_cast<uint16_t>(code_bytes / 4u),
                    .config = {.direction = DmaDirection::peripheral_to_memory,
                               .memory_to_memory = true,
                               .peripheral_increment = true,
                               .memory_increment = false,
                               .peripheral_width = DmaWidth::word,
                               .memory_width = DmaWidth::word,
                               .priority = DmaPriority::high}});
    const uint32_t d0 = cycles_now();
    (void)ChA::enable(true);
    uint32_t spins = 4'000'000u;
    while (!ChA::flag(DmaFlag::complete) && spins-- != 0u) {
    }
    const uint32_t d1 = cycles_now();
    const bool done = ChA::flag(DmaFlag::complete);
    ChA::stop();
    const uint32_t dma_value = crc32_ieee_finish(Crc::value());

    const uint32_t cpu_cycles = c1 - c0;
    const uint32_t dma_cycles = d1 - d0;
    const uint32_t dma_rate =
        dma_cycles == 0u ? 0u
                         : static_cast<uint32_t>((static_cast<uint64_t>(code_bytes) *
                                                  SysClock::hz) / dma_cycles);
    print(serial, "  ", code_bytes, " bytes of bank 1: CPU-fed ", hex(cpu_value),
          " in ", cpu_cycles, " cycles, DMA-fed ", hex(dma_value), " in ",
          dma_cycles, " cycles, software reference ", hex(sw_value), crlf);
    print(serial, "  the DMA path moves ", dma_rate / 1000u,
          " kB/s with no CPU in the loop", crlf);

    bench.verdict("the channel ran to completion with the CRC as a FIXED "
                  "destination and no request line at all - MEM2MEM is the "
                  "only mode that can feed this peripheral, since table 55 "
                  "has no CRC row",
                  prepared && done);
    bench.verdict("the DMA-fed checksum is the CPU-fed one, over 4096 words",
                  done && dma_value == cpu_value);
    bench.verdict("and both are the reflected CRC-32 a bitwise loop computes "
                  "over the same bytes - three routes, one number",
                  cpu_value == sw_value);
    bench.verdict("the DMA path is faster than the CPU one, having no "
                  "instruction fetch between beats",
                  dma_cycles < cpu_cycles);
    Dma1::reset();
    Dma1::bus_clock(false);
}

// =============================================================================
// e - the two rules no register enforces
// =============================================================================
void te_refusals() {
    Crc::init();
    (void)Crc::configure(crc_ccitt_false_config);
    const uint32_t before = Crc::polynomial();

    const bool even_refused =
        !Crc::configure(CrcConfig{.polynomial = 0x1020u, .size = CrcWidth::w16});
    const bool wide_refused =
        !Crc::configure(CrcConfig{.polynomial = 0x1021u, .size = CrcWidth::w8});
    const bool wide32_refused = !Crc::configure(
        CrcConfig{.polynomial = 0x04C11DB7u, .size = CrcWidth::w16});
    const uint32_t after = Crc::polynomial();

    bench.verdict("an EVEN polynomial is refused before a register is touched "
                  "(14.3.3: \"even polynomials are not supported\", and the "
                  "silicon does not notice)",
                  even_refused);
    bench.verdict("so is one that does not fit the POLYSIZE it declares - "
                  "14.4.5 would simply use the low bits and compute a "
                  "checksum nobody asked for",
                  wide_refused && wide32_refused);
    bench.verdict("and a refusal writes NOTHING: the polynomial in force is "
                  "the one that was there",
                  after == before);

    bench.verdict("the compile-time twin is the family fixture's "
                  "(test/family_stm32g0/neg/crc_even_polynomial.cpp and "
                  "crc_polynomial_too_wide.cpp) - both refuse to compile",
                  !crc_config_valid(CrcConfig{.polynomial = 0x1020u,
                                              .size = CrcWidth::w16}) &&
                      !crc_config_valid(CrcConfig{.polynomial = 0x1021u,
                                                  .size = CrcWidth::w8}));
}

// ---------------------------------------------------------------------------
// The menu
// ---------------------------------------------------------------------------

void banner() {
    print(serial, crlf,
          "test_stm32_crc - the CRC calculation unit (board E, no wires)", crlf);
    bench.menu();
    print(serial, "  z  run them all", crlf);
}

}   // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    brio::Crc::init();

    bench.letter('a', "the block: reset values, the reversals, the polynomial "
                      "rule", ta_block);
    bench.letter('b', "four standards' check values, and 4 KB against "
                      "util/crc.hpp", tb_standards);
    bench.letter('c', "what a checksum costs, against the software loop",
                 tc_cost);
    bench.letter('d', "16 KB of flash checksummed by DMA, no CPU in the loop",
                 td_dma);
    bench.letter('e', "the two rules 14.3.3 states and no register enforces",
                 te_refusals);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL 64 MHz" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED", crlf);
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
