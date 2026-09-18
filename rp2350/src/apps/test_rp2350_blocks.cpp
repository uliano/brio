// test_rp2350_blocks - the reference bench suite for the four blocks
// this chip has and its predecessor had not: the SHA-256 accelerator
// (datasheet 12.13), the true random number generator (12.12), the
// one-time programmable array READ SIDE (chapter 13) and the mask ROM's
// public function table (5.4).
//
// ONE SOURCE, BOTH ARCHITECTURES. Every letter runs on the Cortex-M33
// pair and on the Hazard3 pair, built from the rp2350-arm-* and
// rp2350-riscv-* presets. Two verdicts MUST differ between them and say
// so in their own text: ARCHSEL_STATUS, which reads 0 when the bootrom
// entered an Arm image and 3 when it entered a RISC-V one, and the
// bootrom's own CPU_INFO word, which says the same thing from the other
// side. Everything else is one answer on both.
//
// NOTHING TO WIRE, AND NOTHING WRITTEN. The console is the Debug
// Probe's UART bridge on GP0 (TX) / GP1 (RX) = UART0. No letter here
// programs a single OTP bit, sets a software page lock or calls the
// bootrom's otp_access: the array is READ ONLY in this tree (there is
// no verb for the rest), so every letter is safe to run any number of
// times and `z` carries them all.
//
// THE JUDGE OF THE ACCELERATOR IS ARITHMETIC, NOT A TABLE. util/sha256.
// hpp is SHA-256 in constexpr C++, judged on the host against the FIPS
// 180-4 vectors (test/test_sha256); here the silicon is asked for the
// same digests and the two are compared word for word. A verdict that
// says "the block agrees with the definition" is therefore a comparison
// between two independent computations and not an assertion about one.
//
// What is exercised, letter by letter:
//   a  the four blocks' boot story: the chip's identity from SYSINFO and
//      from OTP side by side, the architecture from two registers, the
//      ROM's version and revision
//   b  SHA-256 against the published vectors, and THE WORD ORDER of
//      SUM0..SUM7 settled by measurement rather than by reading
//   c  SHA-256 over 4 kB: agreement with the twin, and the throughput of
//      the polled path in bytes per second and cycles per byte, beside
//      the software twin's own
//   d  the accelerator's edges: the write-error flag raised on purpose,
//      BSWAP both ways over the same bytes, a digest taken before any
//      data is the initial state
//   e  the generator: what the block says about itself, how long 192
//      bits take, and how often it discards a run on its own checks
//   f  a crude quality measurement on 4 kbit - monobit, byte spread, no
//      repeated block. A MEASUREMENT AND NOT A CERTIFICATION: it would
//      pass for a counter, and it is here to catch a source that is not
//      running at all
//   g  the generator's recovery: the internal soft reset and its
//      required delay, the settings back, entropy again afterwards
//   h  the OTP record of this board, printed: the identity rows, the
//      calibration rows, the package, and the critical and boot flags
//      decoded bit by bit - this is the letter whose OUTPUT is the point
//   j  the three read paths against each other: the ECC window, the raw
//      window decoded in software, and the guarded window on a row the
//      first two have just agreed about
//   k  the ROM's table: found from the well-known words, the same
//      function resolved on this half, get_sys_info decoded and
//      cross-checked against SYSINFO and OTP, the partition table's
//      absence reported as the code the chapter names
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <array>
#include <optional>
#include <span>

#include "rp2350/bootrom.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/otp.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/sha256.hpp"
#include "rp2350/sysinfo.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/trng.hpp"
#include "rp2350/uart.hpp"
#include "util/print.hpp"
#include "util/sha256.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000>;
constexpr SysClock clock;
using P = brio::Rp2350Platform<>;

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

using Led = Pin<25>;

TestBench<Serial> bench;

/// The microsecond ruler every measurement here is taken against: the
/// SIO's platform timer, which both architectures share and which rides
/// clk_ref and not clk_sys - so a throughput measured with it is not
/// measured with the clock it is measuring.
uint32_t us_now() { return Mtime::micros(); }

const char* arch_name() { return core_kind == CoreKind::hazard3 ? "Hazard3" : "Cortex-M33"; }

/// The message the standard's appendix B.1 uses, and the two-block one
/// of B.2 that forces the padding into a block of its own.
constexpr std::array<uint8_t, 3> msg_abc = {'a', 'b', 'c'};
constexpr std::array<uint8_t, 56> msg_two = {
    'a', 'b', 'c', 'd', 'b', 'c', 'd', 'e', 'c', 'd', 'e', 'f', 'd', 'e',
    'f', 'g', 'e', 'f', 'g', 'h', 'f', 'g', 'h', 'i', 'g', 'h', 'i', 'j',
    'h', 'i', 'j', 'k', 'i', 'j', 'k', 'l', 'j', 'k', 'l', 'm', 'k', 'l',
    'm', 'n', 'l', 'm', 'n', 'o', 'm', 'n', 'o', 'p', 'n', 'o', 'p', 'q',
};

/// The three answers, computed here by the constexpr twin - so they are
/// in the image as CONSTANTS and the comparison costs nothing.
constexpr Sha256Digest digest_empty = sha256(std::span<const uint8_t>{});
constexpr Sha256Digest digest_abc = sha256(std::span<const uint8_t>{msg_abc});
constexpr Sha256Digest digest_two = sha256(std::span<const uint8_t>{msg_two});

/// The block the throughput letter hashes: 4 kB, filled with something
/// that is not all one byte.
std::array<uint8_t, 4096> bulk;

/// Sixteen 192-bit collections: 3072 bits of entropy, which is what the
/// quality letter measures on.
constexpr uint8_t entropy_blocks = 16;
std::array<uint32_t, 6 * entropy_blocks> entropy;
uint8_t entropy_got = 0;
uint32_t entropy_us = 0;
uint32_t entropy_discarded = 0;

/// Fill the array above, timing it and counting what the block threw
/// away on its own checks. Two letters want it and either may be asked
/// for first, so it lives here rather than in one of them.
void collect_entropy() {
    const uint32_t before = Trng::recoverable_failures();
    const bool was_running = Trng::running();
    if (!was_running) {
        Trng::start();
    }
    const uint32_t t0 = us_now();
    entropy_got = 0;
    for (uint8_t i = 0; i < entropy_blocks; ++i) {
        const auto e = Trng::read_blocking();
        if (!e) {
            break;
        }
        for (uint32_t w = 0; w < 6; ++w) {
            entropy[6u * i + w] = e->words[w];
        }
        ++entropy_got;
    }
    entropy_us = us_now() - t0;
    entropy_discarded = Trng::recoverable_failures() - before;
    if (!was_running) {
        Trng::stop();
    }
}

void print_digest(const Sha256Digest& d) {
    for (uint32_t i = 0; i < sha256_digest_words; ++i) {
        print(serial, hex(d.words[i]));
    }
}

/// One word with its bytes reversed, for the one question the datasheet
/// does not answer: whether SUM0..SUM7 present H0..H7 as words or as
/// byte streams.
uint32_t swapped(uint32_t w) {
    return (w >> 24) | ((w >> 8) & 0x0000FF00u) | ((w << 8) & 0x00FF0000u) | (w << 24);
}

bool same_swapped(const Sha256Digest& a, const Sha256Digest& b) {
    for (uint32_t i = 0; i < sha256_digest_words; ++i) {
        if (a.words[i] != swapped(b.words[i])) {
            return false;
        }
    }
    return true;
}

uint32_t popcount32(uint32_t v) {
    uint32_t n = 0;
    while (v != 0u) {
        n += v & 1u;
        v >>= 1;
    }
    return n;
}

// =============================================================================
// a - the four blocks' boot story
// =============================================================================
void ta_identity() {
    const ChipId id = ChipId::read();
    print(serial, "  SYSINFO: manufacturer ", hex(id.manufacturer), " part ", hex(id.part),
          " revision ", hex(id.revision), ", package ",
          ChipId::package_sel() == Package::qfn80 ? 80 : 60, " pins, running ", arch_name(),
          crlf);
    bench.verdict("SYSINFO names Raspberry Pi's RP2350",
                  id.manufacturer == ChipId::raspberry_pi && id.part == ChipId::rp2350);

    const auto otp_id = Otp::chip_id();
    const auto gpios = Otp::num_gpios();
    print(serial, "  OTP: CHIPID ");
    if (otp_id) {
        print(serial, hex(static_cast<uint32_t>(*otp_id >> 32)),
              hex(static_cast<uint32_t>(*otp_id)));
    } else {
        print(serial, "unreadable");
    }
    print(serial, ", NUM_GPIOS ");
    if (gpios) {
        print(serial, *gpios);
    } else {
        print(serial, "unreadable");
    }
    print(serial, crlf);
    bench.verdict("page 0 answers: the array's identity rows are readable and not all-ones",
                  otp_id.has_value() && *otp_id != 0xFFFFFFFFFFFFFFFFull && *otp_id != 0);
    bench.verdict("the pad count the factory wrote agrees with SYSINFO's package bit",
                  gpios.has_value() &&
                      *gpios == package_gpio_count(ChipId::package_sel()));

    const OtpArchSel arch = Otp::archsel_status();
    print(serial, "  ARCHSEL_STATUS=", hex(arch.raw), " (core0 ",
          arch.core0 == OtpArchitecture::riscv ? "RISC-V" : "Arm", ", core1 ",
          arch.core1 == OtpArchitecture::riscv ? "RISC-V" : "Arm", ")", crlf);
    // THE ONE VERDICT THAT DIFFERS BETWEEN THE TWO HALVES, and the
    // reason it does: the bootrom switched both cores into the
    // architecture the image's own IMAGE_DEF named, so the register
    // answers 0 under an Arm image and 3 under a RISC-V one.
    if constexpr (core_kind == CoreKind::hazard3) {
        bench.verdict("ARCHSEL_STATUS reads 3 under a RISC-V image: the bootrom switched "
                      "BOTH cores, and this half is the one running",
                      arch.raw == 3u && arch.core0 == OtpArchitecture::riscv);
    } else {
        bench.verdict("ARCHSEL_STATUS reads 0 under an Arm image: neither core was switched, "
                      "which is the reset state of the register",
                      arch.raw == 0u && arch.core0 == OtpArchitecture::arm);
    }

    print(serial, "  bootrom: ");
    if (Bootrom::present()) {
        print(serial, "magic at 0x10, version ", Bootrom::version());
        const auto rev = Bootrom::git_revision();
        if (rev) {
            print(serial, ", revision ", hex(*rev));
        }
    } else {
        print(serial, "NOT FOUND");
    }
    print(serial, crlf);
    bench.verdict("the mask ROM's three magic bytes stand at 0x10, so the rest of the "
                  "fixed words may be trusted",
                  Bootrom::present());
    bench.verdict("the ROM reports a version, which is 2 on A2 silicon",
                  Bootrom::version() != 0u && Bootrom::version() != 0xFFu);
    bench.verdict("the ROM's git revision word is in the table and is not blank",
                  Bootrom::git_revision().has_value() &&
                      *Bootrom::git_revision() != 0u &&
                      *Bootrom::git_revision() != 0xFFFFFFFFu);

    bench.verdict("the accelerator and the generator are out of reset, released by their "
                  "own init",
                  Resets::released(ResetBlock::sha256 | ResetBlock::trng));
}

// =============================================================================
// b - SHA-256 against the published vectors
// =============================================================================
void tb_vectors() {
    const auto e = Sha256::hash(std::span<const uint8_t>{});
    const auto a = Sha256::hash(std::span<const uint8_t>{msg_abc});
    const auto t = Sha256::hash(std::span<const uint8_t>{msg_two});

    print(serial, "  empty  hw ");
    if (e) {
        print_digest(*e);
    } else {
        print(serial, "none");
    }
    print(serial, crlf, "         sw ");
    print_digest(digest_empty);
    print(serial, crlf);

    print(serial, "  'abc'  hw ");
    if (a) {
        print_digest(*a);
    } else {
        print(serial, "none");
    }
    print(serial, crlf, "         sw ");
    print_digest(digest_abc);
    print(serial, crlf);

    bench.verdict("the empty message: one padding block, and the block's answer is the "
                  "standard's own",
                  e.has_value() && *e == digest_empty);
    bench.verdict("'abc' (FIPS 180-4 B.1): 24 bits of message in one block",
                  a.has_value() && *a == digest_abc);
    bench.verdict("the 56-byte message (B.2): the padding lands in a SECOND block, which "
                  "is the case the tail arithmetic exists for",
                  t.has_value() && *t == digest_two);

    // THE WORD ORDER OF THE RESULT REGISTERS, measured. 12.13 says what
    // BSWAP does to the INPUT and nothing about how SUM0..SUM7 present
    // the state; this driver reads them as H0..H7, and if that were
    // wrong the digests above would differ from the twin by exactly a
    // byte swap per word. Both comparisons are reported, so one run
    // settles it either way.
    const bool direct = a.has_value() && *a == digest_abc;
    const bool byte_swapped = a.has_value() && same_swapped(digest_abc, *a);
    print(serial, "  SUM0..SUM7 read as H0..H7: direct match ", direct ? "yes" : "no",
          ", match after a byte swap per word ", byte_swapped ? "yes" : "no", crlf);
    bench.verdict("SUM0..SUM7 present the state as WORDS H0..H7, not as a byte stream: "
                  "the direct comparison is the one that holds",
                  direct && !byte_swapped);
}

// =============================================================================
// c - 4 kB, and what the polled path costs
// =============================================================================
void tc_throughput() {
    for (uint32_t i = 0; i < bulk.size(); ++i) {
        bulk[i] = static_cast<uint8_t>(i * 7u + 3u);
    }
    const std::span<const uint8_t> block{bulk};

    const uint32_t t0 = us_now();
    const auto hw = Sha256::hash(block);
    const uint32_t t1 = us_now();
    const Sha256Digest sw = sha256(block);
    const uint32_t t2 = us_now();

    const uint32_t hw_us = t1 - t0;
    const uint32_t sw_us = t2 - t1;
    const uint32_t bytes = static_cast<uint32_t>(bulk.size());
    const uint32_t hw_kbs = hw_us != 0u ? (bytes * 1000u) / hw_us : 0u;
    const uint32_t sw_kbs = sw_us != 0u ? (bytes * 1000u) / sw_us : 0u;
    // Cycles per byte, in hundredths, from the clock this image runs at.
    const uint32_t hw_cpb100 =
        bytes != 0u ? (hw_us * (SysClock::hz / 10'000u)) / bytes : 0u;

    print(serial, "  4096 bytes: accelerator ", hw_us, " us (", hw_kbs, " kB/s, ",
          hw_cpb100 / 100u, ".", (hw_cpb100 / 10u) % 10u, (hw_cpb100 % 10u),
          " clk_sys cycles a byte); software twin ", sw_us, " us (", sw_kbs, " kB/s)", crlf);
    print(serial, "  digest ");
    if (hw) {
        print_digest(*hw);
    } else {
        print(serial, "none");
    }
    print(serial, crlf);

    bench.verdict("the accelerator and the definition agree over 4 kB - 64 blocks, the "
                  "padding in a 65th",
                  hw.has_value() && *hw == sw);
    bench.verdict("the accelerator is faster than the software twin on the same bytes",
                  hw_us != 0u && sw_us > hw_us);
    // 12.13.2's ceiling is 121 clk_sys cycles a block, 1.89 a byte, and
    // that figure is the DMA's; a polled loop pays a status read per
    // word on top. The bar here is loose on purpose: what it catches is
    // a block that is not accelerating anything.
    bench.verdict("the polled path stays under sixteen clk_sys cycles a byte",
                  hw_cpb100 != 0u && hw_cpb100 < 1600u);
    bench.verdict("no word was lost along the way: the write-error flag is clear",
                  !Sha256::write_error());
}

// =============================================================================
// d - the accelerator's edges
// =============================================================================
void td_edges() {
    // A digest taken with nothing written since START is the INITIAL
    // state - the fractional square roots of the first eight primes -
    // because CSR.START forces SUM_VLD high at once.
    Sha256::clear_write_error();
    Sha256::start();
    const auto fresh = Sha256::digest();
    print(serial, "  after START, with nothing written: SUM0=",
          hex(fresh ? fresh->words[0] : 0u), crlf);
    bench.verdict("a digest taken before any data is the algorithm's initial state, which "
                  "is why hash() always writes at least the padding block",
                  fresh.has_value() && fresh->words[0] == sha256_initial_state[0] &&
                      fresh->words[7] == sha256_initial_state[7]);

    // THE WRITE-ERROR FLAG, raised on purpose: sixteen words fill a
    // block, the core goes busy for 57 cycles, and a seventeenth word
    // written with no handshake is dropped.
    Sha256::clear_write_error();
    Sha256::start();
    for (uint32_t i = 0; i < 16; ++i) {
        Sha256::write_word_unchecked(0u);
    }
    Sha256::write_word_unchecked(0u);
    const bool raised = Sha256::write_error();
    print(serial, "  a word written into the busy core: ERR_WDATA_NOT_RDY ",
          raised ? "raised" : "not raised", crlf);
    bench.verdict("a word written while the core is digesting is DROPPED and says so - "
                  "which is what makes the polled handshake necessary and checkable",
                  raised);
    Sha256::clear_write_error();
    bench.verdict("and the flag is write-one-to-clear", !Sha256::write_error());

    // BSWAP both ways over the same bytes. With it set, a word loaded
    // little-endian from the message is the standard's big-endian
    // message word; with it clear, the SAME word enters byte-reversed,
    // so the digest must differ - and must equal the digest of the
    // byte-reversed message.
    std::array<uint8_t, 64> one_block{};
    for (uint32_t i = 0; i < one_block.size(); ++i) {
        one_block[i] = static_cast<uint8_t>(i);
    }
    std::array<uint8_t, 64> reversed_words{};
    for (uint32_t i = 0; i < one_block.size(); i += 4) {
        reversed_words[i + 0] = one_block[i + 3];
        reversed_words[i + 1] = one_block[i + 2];
        reversed_words[i + 2] = one_block[i + 1];
        reversed_words[i + 3] = one_block[i + 0];
    }
    const auto with = Sha256::hash(std::span<const uint8_t>{one_block});
    Sha256::byte_swap(false);
    const auto without = Sha256::hash(std::span<const uint8_t>{one_block});
    Sha256::byte_swap(true);
    bench.verdict("BSWAP set is what makes a word loaded little-endian the standard's "
                  "message word",
                  with.has_value() && *with == sha256(std::span<const uint8_t>{one_block}));
    bench.verdict("BSWAP clear digests the same buffer with every message word reversed, "
                  "which is the byte-reversed message's own digest",
                  without.has_value() &&
                      *without == sha256(std::span<const uint8_t>{reversed_words}));
    bench.verdict("and the two are not the same digest",
                  with.has_value() && without.has_value() && !(*with == *without));

    print(serial, "  DMA request ", static_cast<uint32_t>(Sha256::dreq),
          " (12.6.4.1's table), DMA_SIZE ", static_cast<uint32_t>(Sha256::dma_size()), crlf);
    bench.verdict("the block's pacing request is the one the DREQ table names, and "
                  "DMA_SIZE is left at words",
                  Sha256::dreq == Dreq::sha256 && Sha256::dma_size() == Sha256DmaSize::words);
}

// =============================================================================
// e - the generator: what it is, and what it costs
// =============================================================================
void te_generator() {
    print(serial, "  RNG_VERSION=", hex(Trng::version()), " (EHR ",
          Trng::ehr_is_192_bits() ? 192 : 128, " bits, autocorrelation ",
          Trng::has_autocorrelation() ? "present" : "absent", ", CRNGT ",
          Trng::has_crngt() ? "present" : "absent", "); chain ", Trng::chain(),
          ", sample count ", Trng::sample_cycles(), ", debug control ",
          hex(Trng::debug_control()), crlf);
    bench.verdict("this IP carries the 192-bit entropy holding register the driver reads "
                  "six words out of",
                  Trng::ehr_is_192_bits());
    bench.verdict("the three entropy checks are present and none is bypassed",
                  Trng::has_autocorrelation() && Trng::has_crngt() &&
                      Trng::debug_control() == 0u);
    bench.verdict("the debug mode of the IP is off", !Trng::debug_mode());

    Trng::start();
    bench.verdict("the ring oscillator is running", Trng::running());

    collect_entropy();
    const uint32_t bits = static_cast<uint32_t>(entropy_got) * Trng::entropy_bits;
    const uint32_t per_block_us = entropy_got != 0u ? entropy_us / entropy_got : 0u;
    const uint32_t bits_per_s =
        entropy_us != 0u ? (bits * 1000u) / (entropy_us / 1000u + 1u) : 0u;
    print(serial, "  ", entropy_got, " collections of 192 bits in ", entropy_us, " us (",
          per_block_us, " us each, about ", bits_per_s, " bit/s); runs discarded on a "
          "health check: ", entropy_discarded, crlf);
    const Trng::AutocorrStats stats = Trng::autocorr_stats();
    print(serial, "  autocorrelation tests: ", stats.tries, " started, ", stats.failures,
          " failed; BIST counters ", hex(Trng::bist()[0]), " ", hex(Trng::bist()[1]), " ",
          hex(Trng::bist()[2]), crlf);

    bench.verdict("every collection asked for arrived within its bounded wait",
                  entropy_got == entropy_blocks);
    bench.verdict("the fatal check has not fired: AUTOCORR_ERR would stop the block until "
                  "it is reset",
                  (Trng::status() & TrngFlag::autocorr_error) == 0u);
    // 12.12.1 puts the yield at about 7.5 kb/s with the core at 150 MHz,
    // which would be some 25 ms a collection; 12.12.2's worked example at
    // chain 0..1 and sample count 20..25 says about 2 ms. Generation time
    // is NOT deterministic (12.12.4), so this bar is only a COMPLETION
    // check - the number beside it is the measurement.
    bench.verdict("a collection takes a measurable time and under a second",
                  per_block_us > 10u && per_block_us < 1'000'000u);
    Trng::stop();
    bench.verdict("and the source stops when it is asked to, which 12.12.3 asks for when "
                  "the block is idle",
                  !Trng::running());
}

// =============================================================================
// f - a crude quality measurement (not a certification)
// =============================================================================
void tf_quality() {
    // Letter e fills the array; this letter may be asked for on its own.
    if (entropy_got != entropy_blocks) {
        collect_entropy();
    }
    uint32_t ones = 0;
    uint32_t zero_words = 0;
    uint32_t repeats = 0;
    std::array<uint16_t, 16> nibble{};
    for (uint32_t i = 0; i < entropy.size(); ++i) {
        const uint32_t w = entropy[i];
        ones += popcount32(w);
        if (w == 0u) {
            ++zero_words;
        }
        if (i != 0u && w == entropy[i - 1]) {
            ++repeats;
        }
        for (uint32_t n = 0; n < 8; ++n) {
            ++nibble[(w >> (4u * n)) & 0xFu];
        }
    }
    const uint32_t bits = static_cast<uint32_t>(entropy.size()) * 32u;
    const uint32_t ones_permille = bits != 0u ? (ones * 1000u) / bits : 0u;
    uint16_t low = 0xFFFFu;
    uint16_t high = 0;
    for (uint32_t n = 0; n < 16; ++n) {
        if (nibble[n] < low) {
            low = nibble[n];
        }
        if (nibble[n] > high) {
            high = nibble[n];
        }
    }
    print(serial, "  ", bits, " bits: ", ones, " ones (", ones_permille / 10u, ".",
          ones_permille % 10u, " per cent), nibble counts ", low, "..", high,
          " against an even ", bits / 4u / 16u, ", repeated words ", repeats, crlf);

    bench.verdict("the collections this letter judges all arrived",
                  entropy_got == entropy_blocks);
    bench.verdict("the collections are not empty: no word of the 3072 bits is zero",
                  zero_words == 0u);
    bench.verdict("no word repeats the one before it",
                  repeats == 0u);
    // A monobit count this loose would pass for a counter, which is
    // exactly why the text says so: it is here to catch a source that is
    // stuck, not to say anything about entropy.
    bench.verdict("the monobit fraction is within a tenth of a half - a STUCK-SOURCE "
                  "check and not a statement about randomness",
                  ones_permille > 400u && ones_permille < 600u);
    bench.verdict("every one of the sixteen nibble values occurs", low > 0u);
}

// =============================================================================
// g - the generator's recovery path
// =============================================================================
void tg_recovery() {
    const uint32_t before_chain = Trng::chain();
    const uint32_t before_sample = Trng::sample_cycles();

    // The internal soft reset returns every setting to its reset value:
    // that is what makes it the only cure for AUTOCORR_ERR and what
    // makes recover() - the reset AND the settings again - the verb a
    // program calls.
    Trng::sw_reset();
    const uint32_t after_reset_sample = Trng::sample_cycles();
    print(serial, "  after TRNG_SW_RESET: SAMPLE_CNT1=", after_reset_sample,
          " (reset value 0xffff), chain=", Trng::chain(), crlf);
    bench.verdict("the IP's soft reset puts the settings back to their reset values, "
                  "SAMPLE_CNT1's included",
                  after_reset_sample == 0xFFFFu && after_reset_sample != before_sample);

    const bool back = Trng::recover();
    print(serial, "  after recover(): chain=", Trng::chain(), ", sample count ",
          Trng::sample_cycles(), crlf);
    bench.verdict("recover() is the reset and the settings again", back &&
                  Trng::chain() == before_chain && Trng::sample_cycles() == before_sample);

    Trng::start();
    const uint32_t t0 = us_now();
    const auto e = Trng::read_blocking();
    const uint32_t span = us_now() - t0;
    print(serial, "  first collection after the recovery: ", span, " us", crlf);
    bench.verdict("the source collects again after a soft reset", e.has_value());
    Trng::stop();

    // RST_BITS_COUNTER refuses while the source is enabled, which is
    // 12.12.5's own condition and the one place this block answers false
    // rather than writing into the wind.
    Trng::start();
    const bool refused = !Trng::reset_bit_counter();
    Trng::stop();
    const bool accepted = Trng::reset_bit_counter();
    bench.verdict("the bit counter refuses to reset while the source is enabled, and "
                  "takes it when the source is stopped",
                  refused && accepted);
}

// =============================================================================
// h - this board's OTP record, printed
// =============================================================================
void print_flags(uint32_t word, const char* const* names, const uint32_t* masks,
                 uint32_t count) {
    bool any = false;
    for (uint32_t i = 0; i < count; ++i) {
        if ((word & masks[i]) != 0u) {
            print(serial, any ? " " : "", names[i]);
            any = true;
        }
    }
    if (!any) {
        print(serial, "none");
    }
}

void th_otp_record() {
    print(serial, "  data window enabled: ", Otp::data_window_enabled() ? "yes" : "no",
          ", RMA flag ", Otp::rma_flag() ? "SET" : "clear", ", DBG=", hex(Otp::debug_status()),
          crlf);
    bench.verdict("the memory-mapped data windows are reachable: USR.DCTRL stands, so no "
                  "programming bridge owns the array",
                  Otp::data_window_enabled());
    bench.verdict("the decommissioning flag is clear, so pages 3..61 are as they were",
                  !Otp::rma_flag());

    const auto rid = Otp::random_id();
    const auto rosc = Otp::rosc_calib_khz();
    const auto lposc = Otp::lposc_calib_hz();
    const auto crc = Otp::info_crc();
    const auto devinfo = Otp::flash_devinfo();
    print(serial, "  RANDID ");
    if (rid) {
        for (uint32_t i = 8; i > 0; --i) {
            print(serial, hex((*rid)[i - 1]));
        }
    } else {
        print(serial, "unreadable");
    }
    print(serial, crlf, "  ROSC_CALIB ");
    if (rosc) {
        print(serial, *rosc, " kHz");
    } else {
        print(serial, "unreadable");
    }
    print(serial, ", LPOSC_CALIB ");
    if (lposc) {
        print(serial, *lposc, " Hz");
    } else {
        print(serial, "unreadable");
    }
    print(serial, ", INFO_CRC ");
    if (crc) {
        print(serial, hex(*crc));
    } else {
        print(serial, "unreadable");
    }
    print(serial, ", FLASH_DEVINFO ");
    if (devinfo) {
        print(serial, hex(*devinfo));
    } else {
        print(serial, "unreadable");
    }
    print(serial, crlf);
    bench.verdict("the factory's calibration rows carry plausible rates: the ring "
                  "oscillator in megahertz and the low-power one near 32 kHz",
                  rosc.has_value() && lposc.has_value() && *rosc > 1'000u &&
                      *rosc < 60'000u && *lposc > 10'000u && *lposc < 60'000u);
    bench.verdict("the chip information CRC row is written", crc.has_value() && *crc != 0u);

    // THE CRITICAL FLAGS, decoded bit by bit. These are exactly what
    // this project forbids a program to set, so printing them IS the
    // record of the board's standing state.
    static const char* const crit1_names[] = {"SECURE_BOOT_ENABLE", "SECURE_DEBUG_DISABLE",
                                              "DEBUG_DISABLE", "BOOT_ARCH",
                                              "GLITCH_DETECTOR_ENABLE"};
    static const uint32_t crit1_masks[] = {
        OtpCrit1::secure_boot_enable, OtpCrit1::secure_debug_disable, OtpCrit1::debug_disable,
        OtpCrit1::boot_arch, OtpCrit1::glitch_detector_enable};
    static const char* const crit0_names[] = {"ARM_DISABLE", "RISCV_DISABLE"};
    static const uint32_t crit0_masks[] = {OtpCrit0::arm_disable, OtpCrit0::riscv_disable};

    const auto c0 = Otp::crit0();
    const auto c1 = Otp::crit1();
    print(serial, "  CRIT0 (three-of-eight voted) ", hex(c0.value_or(0u)), ": ");
    print_flags(c0.value_or(0u), crit0_names, crit0_masks, 2);
    print(serial, crlf, "  CRIT1 (three-of-eight voted) ", hex(c1.value_or(0u)), ": ");
    print_flags(c1.value_or(0u), crit1_names, crit1_masks, 5);
    print(serial, crlf, "  CRITICAL register (hardware's own latch) ", hex(Otp::critical()),
          ", KEY_VALID ", hex(Otp::key_valid()), ", DEBUGEN ", hex(Otp::debugen()),
          ", DEBUGEN_LOCK ", hex(Otp::debugen_lock()), ", BOOTDIS ", hex(Otp::bootdis()),
          crlf);

    bench.verdict("neither architecture has been disabled: both halves of this suite can "
                  "still be flashed",
                  c0.has_value() &&
                      (*c0 & (OtpCrit0::arm_disable | OtpCrit0::riscv_disable)) == 0u);
    bench.verdict("secure boot is not enabled and no debug disable stands - which is why "
                  "the probe can still reach this board",
                  c1.has_value() &&
                      (*c1 & (OtpCrit1::secure_boot_enable | OtpCrit1::debug_disable |
                              OtpCrit1::secure_debug_disable)) == 0u);
    bench.verdict("the rows and the hardware's own latched copy agree about secure boot",
                  c1.has_value() &&
                      (((*c1 & OtpCrit1::secure_boot_enable) != 0u) ==
                       ((Otp::critical() & OtpCritical::secure_boot_enable) != 0u)));
    bench.verdict("no hardware access key is enrolled, so nothing of this array is hidden "
                  "behind one",
                  Otp::key_valid() == 0u);

    static const char* const bf0_names[] = {
        "DISABLE_BOOTSEL_UART_BOOT", "DISABLE_BOOTSEL_USB_PICOBOOT_IFC",
        "DISABLE_BOOTSEL_USB_MSD_IFC", "DISABLE_WATCHDOG_SCRATCH", "DISABLE_POWER_SCRATCH",
        "DISABLE_FLASH_BOOT", "ENABLE_OTP_BOOT", "FLASH_DEVINFO_ENABLE",
        "SINGLE_FLASH_BINARY", "ENABLE_BOOTSEL_LED"};
    static const uint32_t bf0_masks[] = {
        OtpBootFlags0::disable_bootsel_uart_boot,
        OtpBootFlags0::disable_bootsel_usb_picoboot_ifc,
        OtpBootFlags0::disable_bootsel_usb_msd_ifc,
        OtpBootFlags0::disable_watchdog_scratch,
        OtpBootFlags0::disable_power_scratch,
        OtpBootFlags0::disable_flash_boot,
        OtpBootFlags0::enable_otp_boot,
        OtpBootFlags0::flash_devinfo_enable,
        OtpBootFlags0::single_flash_binary,
        OtpBootFlags0::enable_bootsel_led};
    const auto bf0 = Otp::boot_flags0();
    const auto bf1 = Otp::boot_flags1();
    print(serial, "  BOOT_FLAGS0 (best of three) ", hex(bf0.value_or(0u)), ": ");
    print_flags(bf0.value_or(0u), bf0_names, bf0_masks, 10);
    print(serial, crlf, "  BOOT_FLAGS1 ", hex(bf1.value_or(0u)), ", USB_BOOT_FLAGS ",
          hex(Otp::usb_boot_flags().value_or(0u)), crlf);
    bench.verdict("no boot path has been closed: a blank device's boot flags are all zero "
                  "and this board's are too",
                  bf0.has_value() && *bf0 == 0u);

    // The page locks as the factory left them (13.5.5): page 0
    // read-only everywhere, pages 1 and 2 read-only to Non-secure, 62
    // and 63 with their own rules, everything else open.
    uint8_t secure_ro = 0;
    uint8_t secure_closed = 0;
    for (uint8_t page = 0; page < otp_page_count; ++page) {
        const OtpPageLock lock = Otp::page_lock(page);
        if (lock.secure == OtpLock::read_only) {
            ++secure_ro;
        } else if (lock.secure == OtpLock::inaccessible) {
            ++secure_closed;
        }
    }
    const OtpPageLock p0 = Otp::page_lock(0);
    print(serial, "  page locks: page 0 secure ", static_cast<uint32_t>(p0.secure),
          "/non-secure ", static_cast<uint32_t>(p0.non_secure), "; of 64 pages ", secure_ro,
          " are secure-read-only and ", secure_closed, " are inaccessible", crlf);
    bench.verdict("page 0 is read-only to Secure code, which is what the factory programs "
                  "after writing the chip information (13.5.5)",
                  p0.secure == OtpLock::read_only);
    bench.verdict("no page is closed to Secure reads, so every row this suite prints was "
                  "there to be read",
                  secure_closed == 0u);
}

// =============================================================================
// j - the three read paths against each other
// =============================================================================
void tj_read_paths() {
    // Over a run of the factory's own rows: the ECC window against the
    // raw window decoded here. They must agree row for row, and the
    // software decoder's verdict says whether any of them needed
    // repairing - which the hardware window would have done in silence.
    uint32_t checked = 0;
    uint32_t disagreed = 0;
    uint32_t corrected = 0;
    uint32_t uncorrectable = 0;
    for (uint16_t row = 0; row < 0x40u; ++row) {
        const auto r = Otp::read(row);
        if (!r) {
            continue;
        }
        ++checked;
        if (r->ecc == OtpEcc::corrected) {
            ++corrected;
        }
        if (r->ecc == OtpEcc::uncorrectable) {
            ++uncorrectable;
            continue;
        }
        if (Otp::ecc(row) != r->data) {
            ++disagreed;
        }
    }
    print(serial, "  rows 0x000..0x03f: ", checked, " read, ", disagreed,
          " where the ECC window and the software decode differ, ", corrected,
          " needed one bit repaired, ", uncorrectable, " beyond repair", crlf);
    bench.verdict("the hardware's ECC window and this driver's software decode of the raw "
                  "window agree on every row of page 0",
                  checked > 0u && disagreed == 0u);
    bench.verdict("no row of the factory's own page is beyond repair", uncorrectable == 0u);

    // The guarded window, on a row the two unguarded paths have just
    // agreed about - because a guarded read of a refused or broken row
    // is a BUS FAULT and not a value, and this suite does not stage one.
    const auto row0 = Otp::read(OtpRowId::chipid0);
    const bool safe = row0.has_value() && row0->ecc == OtpEcc::ok;
    if (safe) {
        const uint16_t guarded = Otp::ecc_guarded(OtpRowId::chipid0);
        const uint32_t guarded_raw = Otp::raw_guarded(OtpRowId::chipid0);
        print(serial, "  CHIPID0: ecc ", hex(Otp::ecc(OtpRowId::chipid0)), ", guarded ",
              hex(guarded), ", raw ", hex(Otp::raw(OtpRowId::chipid0)), ", raw guarded ",
              hex(guarded_raw), crlf);
        bench.verdict("a guarded read returns the same data as the unguarded one, its "
                      "extra checks being about failure and not about value",
                      guarded == row0->data && guarded_raw == row0->raw);
    } else {
        bench.verdict("CHIPID0 decodes cleanly, which is what makes a guarded read of it "
                      "safe to take",
                      false);
    }

    // A row nothing has ever written reads as zero through the ECC
    // window: an erased cell is a zero here, not a one, which is the
    // opposite of every flash array in this tree.
    const auto blank = Otp::read(0x0C0u);
    print(serial, "  first user row (0x0c0): ");
    if (blank) {
        print(serial, "ecc ", hex(blank->data), ", raw ", hex(blank->raw));
    } else {
        print(serial, "not readable");
    }
    print(serial, crlf);
    bench.verdict("the first user page is readable and unprogrammed: an OTP cell starts "
                  "at ZERO and goes to one, once",
                  blank.has_value() && blank->raw == 0u && blank->data == 0u);

    // And the row the driver's own arithmetic is judged by: the decoder
    // says a clean row is clean.
    bench.verdict("the software ECC decoder calls a factory row clean",
                  row0.has_value() && row0->ecc == OtpEcc::ok && !row0->inverted);
}

// =============================================================================
// k - the mask ROM's table
// =============================================================================
void tk_bootrom() {
    const void* table = Bootrom::table();
    const void* lookup_v = reinterpret_cast<const void*>(Bootrom::lookup_value());
    const void* lookup_e = reinterpret_cast<const void*>(Bootrom::lookup_entry());
    const void* sys = Bootrom::lookup_function(BootromCode::get_sys_info);
    const void* part = Bootrom::lookup_function(BootromCode::get_partition_table_info);
    const uint32_t table_a = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(table));
    const uint32_t sys_a = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(sys));

    print(serial, "  on ", arch_name(), ": table at ", hex(table_a), ", lookup_val ",
          hex(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(lookup_v))),
          ", lookup_entry ",
          hex(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(lookup_e))),
          ", get_sys_info ", hex(sys_a), ", get_partition_table_info ",
          hex(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(part))), crlf);

    bench.verdict("the entry table is inside the 32 kB ROM", table_a != 0u && table_a < 0x8000u);
    bench.verdict("both functions resolve on THIS half, out of the architecture's own "
                  "lookup: the table carries an entry per architecture and the driver "
                  "asks with the right flag",
                  sys != nullptr && part != nullptr);
    bench.verdict("and the addresses they resolve to are in the ROM", sys_a != 0u &&
                  (sys_a & ~1u) < 0x8000u);

    const auto info = Bootrom::sys_info();
    if (!info) {
        bench.verdict("get_sys_info answered", false);
        return;
    }
    print(serial, "  get_sys_info served ", hex(info->served), ": package_sel ",
          hex(info->package_sel), ", device id ",
          hex(static_cast<uint32_t>(info->device_id >> 32)),
          hex(static_cast<uint32_t>(info->device_id)), ", cpu ", info->cpu, ", critical ",
          hex(info->critical), ", flash devinfo ", hex(info->flash_dev_info), crlf);
    print(serial, "  boot random ", hex(info->boot_random[0]), " ", hex(info->boot_random[1]),
          " ", hex(info->boot_random[2]), " ", hex(info->boot_random[3]), "; boot info ",
          hex(info->boot_info[0]), " ", hex(info->boot_info[1]), crlf);

    bench.verdict("the ROM serves every flag this driver asks for",
                  info->served == SysInfoFlag::all);
    bench.verdict("the device id the ROM reports is the CHIPID rows of the array, read "
                  "the other way round",
                  info->has(SysInfoFlag::chip_info) && Otp::chip_id().has_value() &&
                      info->device_id == *Otp::chip_id());
    bench.verdict("and the CRITICAL word it reports is the OTP block's own register",
                  info->has(SysInfoFlag::critical) && info->critical == Otp::critical());
    // THE SECOND VERDICT THAT DIFFERS BETWEEN THE HALVES, from the other
    // side: the ROM reports the architecture of the core that called it.
    if constexpr (core_kind == CoreKind::hazard3) {
        bench.verdict("CPU_INFO reads 1 under a RISC-V image: the ROM is answering a hart",
                      info->has(SysInfoFlag::cpu_info) && info->cpu == 1u);
    } else {
        bench.verdict("CPU_INFO reads 0 under an Arm image: the ROM is answering a "
                      "Cortex-M33",
                      info->has(SysInfoFlag::cpu_info) && info->cpu == 0u);
    }
    bench.verdict("the per-boot random number the ROM made at start of day is not blank - "
                  "it is the TRNG's raw samples through the accelerator this suite also "
                  "drives (12.12.4.1)",
                  info->has(SysInfoFlag::boot_random) &&
                      (info->boot_random[0] | info->boot_random[1] | info->boot_random[2] |
                       info->boot_random[3]) != 0u);

    // The partition table: this image carries none, so the chapter's own
    // answer is a precondition error - which is a MEASUREMENT of the
    // wrapper's error path and not a failure.
    std::array<uint32_t, 8> out{};
    const int32_t rc = Bootrom::get_partition_table_info(
        out.data(), static_cast<uint32_t>(out.size()), PartitionInfoFlag::pt_info);
    print(serial, "  get_partition_table_info -> ", rc, crlf);
    bench.verdict("the partition table call answers the code 5.4.8.16 names when no table "
                  "has been loaded, or reports one if a table is there",
                  rc == BootromError::precondition_not_met || rc > 0);
}

void banner() {
    print(serial, crlf, "test_rp2350_blocks - the SHA-256 accelerator, the TRNG, the OTP "
          "read side and the bootrom table (datasheet 12.13, 12.12, 13, 5.4) on ",
          arch_name(), ", clk_sys=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------

extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool mtime_ok = brio::Mtime::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    const bool sha_ok = brio::Sha256::init();
    const bool trng_ok = brio::Trng::init();
    (void)Led::output(false);
    brio::enable_interrupts();

    bench.letter('a', "the four blocks' boot story", ta_identity);
    bench.letter('b', "SHA-256 against the published vectors", tb_vectors);
    bench.letter('c', "SHA-256 over 4 kB, and what it costs", tc_throughput);
    bench.letter('d', "the accelerator's edges", td_edges);
    bench.letter('e', "the generator, and what 192 bits cost", te_generator);
    bench.letter('f', "a crude quality measurement", tf_quality);
    bench.letter('g', "the generator's recovery path", tg_recovery);
    bench.letter('h', "this board's OTP record", th_otp_record);
    bench.letter('j', "the three OTP read paths against each other", tj_read_paths);
    bench.letter('k', "the mask ROM's table", tk_bootrom);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " mtime=", mtime_ok ? "1us" : "FAILED", " tick=", tick_ok ? "on" : "FAILED",
                    " sha256=", sha_ok ? "ready" : "FAILED", " trng=",
                    trng_ok ? "ready" : "FAILED", brio::crlf);
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
        brio::print(serial, static_cast<char>(c), brio::crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
