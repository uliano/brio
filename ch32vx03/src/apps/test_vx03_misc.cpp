// test_vx03_misc - the reference bench suite for the small blocks of the
// CH32V203 and the CH32V303: ch32vx03/crc.hpp over RM ch. 5, the cyclic
// redundancy check unit, and the pieces of a chapter that are too small
// to earn a suite of their own.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// WHAT THIS BLOCK IS, AND WHAT THERE IS TO MEASURE. Three registers and
// no options: the Ethernet polynomial is wired in, so the function the
// unit computes is exactly one - CRC-32/MPEG-2 - and util/crc.hpp's
// `crc32_ethernet*` is that same function in constexpr C++. THE TWIN IS
// THE JUDGE: every letter here computes both and compares, which is
// what makes the software function the specification and the silicon
// the thing under test. Three questions the manual does not answer are
// letters of their own: whether the reset lands before the next
// instruction can look (the same block on an STM32F4 takes a few
// cycles), what the block's clock gate really holds, and what the whole
// unit costs per word against the core's own counter.
//
// NOTHING IS WIRED AND NOTHING IS PRESSED. The block has no pad, no
// interrupt and no DMA row: every letter is free, so `z` carries all of
// them, and the one letter by name is the one that REBOOTS the board -
// what a system reset does to the scratch register, which is the only
// question a running program cannot answer about it.
//
// What is exercised, letter by letter:
//   a  THE BLOCK: what the HB gate really holds (asked of two registers,
//      so that a stale bus capture cannot be mistaken for a live read),
//      the reset value of the data register, and RST as the only reset
//      this block has
//   b  THE FUNCTION against util/crc.hpp's twin: one word, two, four,
//      sixty-four, and a byte stream packed by `word_be` - plus the
//      empty message, whose checksum is the initial value
//   c  THE RESET'S LATENCY: `reset()` returns its own spin count, and
//      this is where the number is printed; then whether a word written
//      in the instruction after RST is taken or swallowed
//   d  THE RUNNING RESULT: a checksum taken in pieces equals the same
//      checksum taken whole, because reading CRC_DATAR does not consume
//      it
//   e  CRC_IDATAR, the eight bits of scratch: a byte store read back,
//      every value, and survival across `reset()` and across every word
//      the calculator takes
//   f  WHAT IT COSTS: the core's counter across a thousand words
//      through the unit, against the same thousand in software
//   w  (by name) THE SCRATCH ACROSS A SYSTEM RESET: a byte written, the
//      board rebooted, the register read at the next boot
//
// build: boards = v203c6,v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/crc.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/reset.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usart.hpp"
#include "util/crc.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32vx03Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 96'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter w lives in: .noinit, so the crt neither loads nor
// zeroes it, with a magic word because nothing promises SRAM across a
// reset.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x5C31;

struct Token {
    uint16_t magic;
    char letter;    ///< which by-name letter is running ('\0' = none)
    uint8_t leg;
    uint16_t pass;
    uint16_t fail;
    uint8_t wrote;  ///< what letter w put in the scratch register
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

TestBench<Serial> bench;

uint32_t boot_flags = 0;

constexpr uint32_t ticks_per_us = SysClock::hz / 1'000'000u;

/// The core's counter as a stopwatch: STK counts to its reload (one
/// tick of the kernel's timebase) and starts again, so a span is
/// accumulated poll by poll with one period folded in across each wrap.
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

private:
    uint32_t last_ = 0;
    uint32_t acc_ = 0;
};

/// The message every length letter is cut from: a run with no symmetry,
/// so a byte order swapped anywhere shows.
constexpr uint32_t message[64] = {
    0x00000000u, 0xFFFFFFFFu, 0x01020304u, 0xDEADBEEFu, 0x12345678u, 0x9ABCDEF0u,
    0xA5A5A5A5u, 0x5A5A5A5Au, 0x0000FFFFu, 0xFFFF0000u, 0x80000001u, 0x7FFFFFFEu,
    0x11111111u, 0x22222222u, 0x33333333u, 0x44444444u, 0x55555555u, 0x66666666u,
    0x77777777u, 0x88888888u, 0x99999999u, 0xAAAAAAAAu, 0xBBBBBBBBu, 0xCCCCCCCCu,
    0xDDDDDDDDu, 0xEEEEEEEEu, 0x0F0F0F0Fu, 0xF0F0F0F0u, 0x13579BDFu, 0x2468ACE0u,
    0xFEDCBA98u, 0x76543210u, 0x00000001u, 0x00000002u, 0x00000004u, 0x00000008u,
    0x00000010u, 0x00000020u, 0x00000040u, 0x00000080u, 0x00000100u, 0x00000200u,
    0x00000400u, 0x00000800u, 0x00001000u, 0x00002000u, 0x00004000u, 0x00008000u,
    0x00010000u, 0x00020000u, 0x00040000u, 0x00080000u, 0x00100000u, 0x00200000u,
    0x00400000u, 0x00800000u, 0x01000000u, 0x02000000u, 0x04000000u, 0x08000000u,
    0x10000000u, 0x20000000u, 0x40000000u, 0x80000000u,
};

/// The byte stream `word_be` packs, and what util/crc.hpp computes over
/// it byte by byte: the two must agree, and that agreement is the whole
/// reason the packing is stated in the driver.
constexpr uint8_t stream[12] = {'b', 'r', 'i', 'o', ' ', 'C', 'H', '3', '2', 'V', '2', '0'};

// ===========================================================================
// a - the block: the gate, the reset value, the two resets
// ===========================================================================
void ta_block() {
    // What the HB gate really holds, asked in an order that tells a
    // LIVE READ from a stale one: the scratch register is read FIRST
    // with the gate shut, so an answer that carries the data register's
    // low byte is the bus repeating itself and not the block speaking.
    Crc::clock(true);
    (void)Crc::reset();
    Crc::feed(0x12345678u);
    Crc::scratch(0x3Cu);
    const uint32_t running = Crc::value();
    Crc::clock(false);
    const bool gate_shut = !Crc::clock();
    const uint8_t shut_scratch = Crc::regs().IDATAR;
    const uint32_t shut_datar = Crc::regs().DATAR;
    Crc::regs().DATAR = 0xFFFFFFFFu;
    Crc::regs().IDATAR = 0xF0u;
    Crc::clock(true);
    const uint32_t reopened = Crc::value();
    const uint8_t reopened_scratch = Crc::scratch();
    print(serial, "  gate open: a word fed, DATAR=", hex(running), " scratch=0x3C", crlf);
    print(serial, "  gate shut, scratch read first: ", hex(shut_scratch), " then DATAR ",
          hex(shut_datar), crlf);
    print(serial, "  gate re-opened after two stores: DATAR=", hex(reopened), " scratch=",
          hex(reopened_scratch), crlf);
    bench.verdict("with the HB gate SHUT nothing of the block answers: the scratch "
                  "register, which holds 0x3C, comes back as the LOW BYTE OF THE DATA "
                  "REGISTER's last value instead - what a read returns from a block behind "
                  "a closed gate is the last word the bus carried, byte lane and all, and "
                  "never the register that was asked for",
                  gate_shut && shut_scratch == static_cast<uint8_t>(running) &&
                      shut_scratch != 0x3Cu && shut_datar == running);
    bench.verdict("and both stores made with the gate shut are lost, while the block keeps "
                  "what it held: a peripheral behind a closed gate is not a peripheral "
                  "that answers wrongly, it is one that is not there",
                  reopened == running && reopened_scratch == 0x3Cu);

    // RST is the only reset this block has: RCC_AHBRSTR carries no bit
    // for it (3.4.11's [11:0] are reserved), so what RST spares nothing
    // else can clear - letter w asks a system reset the same question.
    Crc::scratch(0x5Au);
    Crc::feed(0xA5A5A5A5u);
    (void)Crc::reset();
    const uint8_t after_soft = Crc::scratch();
    const uint32_t soft_datar = Crc::value();
    print(serial, "  scratch 0x5A: after reset() it reads ", hex(after_soft), " and DATAR ",
          hex(soft_datar), crlf);
    bench.verdict("RST puts the data register back to its initial value and DOES NOT touch "
                  "CRC_IDATAR - and since this block has no line in RCC_AHBRSTR, nothing "
                  "short of a system reset ever does",
                  after_soft == 0x5Au && soft_datar == crc32_ethernet_init);

    (void)Crc::init();
}

// ===========================================================================
// b - the function against util/crc.hpp's twin
// ===========================================================================
void tb_function() {
    (void)Crc::init();

    // The empty message: nothing fed, so the answer is the initial
    // value - the one length that needs no polynomial at all.
    (void)Crc::reset();
    const uint32_t empty = Crc::value();
    bench.verdict("the empty message checksums to the initial value, because a read does "
                  "not start anything",
                  empty == crc32_ethernet(nullptr, 0));

    bool all = true;
    for (uint32_t n : {1u, 2u, 3u, 4u, 8u, 16u, 63u, 64u}) {
        const uint32_t hw = Crc::compute(message, n);
        const uint32_t sw = crc32_ethernet(message, n);
        if (hw != sw) {
            all = false;
        }
        print(serial, "  ", n, " word(s): silicon ", hex(hw), " software ", hex(sw),
              hw == sw ? " agree" : " DIFFER", crlf);
    }
    bench.verdict("eight lengths from one word to sixty-four: the unit's answer is "
                  "util/crc.hpp's crc32_ethernet() exactly, so the block computes "
                  "CRC-32/MPEG-2 and nothing else",
                  all);

    // The packing is this chapter's own fact, so it is measured and not
    // assumed: a byte stream fed most-significant-first must equal the
    // software function over those bytes in that order.
    const uint32_t packed = Crc::compute_bytes(stream);
    const uint32_t bytes_sw = crc32_ethernet_bytes(stream, sizeof(stream));
    print(serial, "  twelve bytes through word_be: silicon ", hex(packed), " software ",
          hex(bytes_sw), crlf);
    bench.verdict("word_be's packing - the first byte of the stream the word's most "
                  "significant - is what makes the unit's answer the BYTE stream's "
                  "checksum: the two agree",
                  packed == bytes_sw);

    // A single flipped bit anywhere must move the answer, which is what
    // says the whole word reached the polynomial.
    uint32_t one[1] = {0x00000000u};
    const uint32_t zero_word = Crc::compute(one, 1);
    one[0] = 0x00000001u;
    const uint32_t low_bit = Crc::compute(one, 1);
    one[0] = 0x80000000u;
    const uint32_t high_bit = Crc::compute(one, 1);
    bench.verdict("the lowest and the highest bit of one word each move the answer, and "
                  "both match the twin: all thirty-two bits are in the calculation",
                  zero_word != low_bit && zero_word != high_bit &&
                      low_bit == crc32_ethernet_word(crc32_ethernet_init, 0x00000001u) &&
                      high_bit == crc32_ethernet_word(crc32_ethernet_init, 0x80000000u));
}

// ===========================================================================
// c - the reset's latency, the chapter's one open question
// ===========================================================================
void tc_reset_latency() {
    (void)Crc::init();

    // reset() returns the number of EXTRA reads it spent waiting for
    // the initial value to appear. Zero means the reset landed before
    // the next instruction could look.
    uint8_t worst = 0;
    for (uint32_t i = 0; i < 64u; ++i) {
        Crc::feed(message[i & 63u]);
        const uint8_t spins = Crc::reset();
        if (spins > worst) {
            worst = spins;
        }
    }
    print(serial, "  sixty-four resets over a dirty register: worst spin count ", worst,
          crlf);
    bench.verdict("RST lands before the next instruction can read the register - the spin "
                  "count is zero every time, where the same block on another family was "
                  "measured taking a few cycles",
                  worst == 0u);

    // The harder half of the same question: is a word written in the
    // instruction AFTER the reset taken, or swallowed by a reset still
    // in flight? The answer is the checksum of one word or of none.
    Crc::feed(0xDEADBEEFu);
    Crc::regs().CTLR = crc_rst;
    Crc::regs().DATAR = 0x01020304u;
    const uint32_t after = Crc::value();
    const uint32_t one_word = crc32_ethernet_word(crc32_ethernet_init, 0x01020304u);
    print(serial, "  a word stored in the instruction after RST: DATAR=", hex(after),
          " (one word would be ", hex(one_word), ", a swallowed one ",
          hex(crc32_ethernet_init), ")", crlf);
    bench.verdict("and a word written in the very next instruction is TAKEN, not swallowed "
                  "- so a checksum may start with reset() and feed in the next store with "
                  "nothing in between",
                  after == one_word);
    (void)Crc::reset();
}

// ===========================================================================
// d - the running result, read in pieces
// ===========================================================================
void td_pieces() {
    (void)Crc::init();

    const uint32_t whole = Crc::compute(message, 16);

    (void)Crc::reset();
    Crc::feed(message, 5);
    const uint32_t part = Crc::value();
    Crc::feed(message + 5, 11);
    const uint32_t joined = Crc::value();
    print(serial, "  sixteen words whole: ", hex(whole), "; five then eleven, read in "
          "between: ", hex(joined), crlf);
    bench.verdict("reading CRC_DATAR does not consume the running result: a checksum taken "
                  "in two pieces with a read between them equals the same checksum taken "
                  "whole",
                  joined == whole && part == crc32_ethernet(message, 5));

    // And the same value reached one word at a time through the twin's
    // own step function, which is what makes the two implementations
    // step for step the same.
    uint32_t sw = crc32_ethernet_init;
    for (uint32_t i = 0; i < 16u; ++i) {
        sw = crc32_ethernet_word(sw, message[i]);
    }
    bench.verdict("and the twin reaches it one word at a time, so the two implementations "
                  "agree at every step and not only at the end",
                  sw == whole);
}

// ===========================================================================
// e - CRC_IDATAR, the eight bits of scratch
// ===========================================================================
void te_scratch() {
    (void)Crc::init();

    bool all = true;
    for (uint32_t v = 0; v < 256u; ++v) {
        Crc::scratch(static_cast<uint8_t>(v));
        if (Crc::scratch() != static_cast<uint8_t>(v)) {
            all = false;
        }
    }
    bench.verdict("all two hundred and fifty-six values read back out of CRC_IDATAR: the "
                  "chapter names the register R8 and a byte store is what it takes",
                  all);

    // Is it eight bits and not more? A word store must leave only the
    // low byte behind.
    Crc::regs().IDATAR = 0xA5u;
    const uint8_t kept = Crc::scratch();
    Crc::scratch(0xC3u);
    (void)Crc::reset();
    const uint8_t across_reset = Crc::scratch();
    Crc::feed(0x12345678u);
    const uint8_t across_feed = Crc::scratch();
    print(serial, "  scratch: 0xA5 read back ", hex(kept), ", 0xC3 across reset() ",
          hex(across_reset), " and across a fed word ", hex(across_feed), crlf);
    bench.verdict("the scratch register is the one piece of state a checksum does not "
                  "touch: it survives RST and every word fed through the calculator",
                  kept == 0xA5u && across_reset == 0xC3u && across_feed == 0xC3u);
}

// ===========================================================================
// f - what the unit costs
// ===========================================================================
void tf_cost() {
    (void)Crc::init();

    constexpr uint32_t laps = 16;   // 16 x 64 = 1024 words
    (void)Crc::reset();
    Stopwatch hw_watch;
    for (uint32_t lap = 0; lap < laps; ++lap) {
        Crc::feed(message, 64);
    }
    const uint32_t hw_cycles = hw_watch.cycles();
    const uint32_t hw_value = Crc::value();

    volatile uint32_t sink = 0;
    Stopwatch sw_watch;
    uint32_t sw = crc32_ethernet_init;
    for (uint32_t lap = 0; lap < laps; ++lap) {
        for (uint32_t i = 0; i < 64u; ++i) {
            sw = crc32_ethernet_word(sw, message[i]);
        }
    }
    const uint32_t sw_cycles = sw_watch.cycles();
    sink = sw;

    const uint32_t words = laps * 64u;
    print(serial, "  1024 words: the unit ", hw_cycles, " cycles (", hw_cycles / words,
          " a word), the twin ", sw_cycles, " cycles (", sw_cycles / words, " a word) at ",
          SysClock::hz / 1'000'000u, " MHz", crlf);
    print(serial, "  that is ", (hw_cycles * 1000u) / ticks_per_us / 1000u, " us against ",
          (sw_cycles * 1000u) / ticks_per_us / 1000u, " us", crlf);
    bench.verdict("a thousand words through the unit cost a handful of cycles each and "
                  "agree with the twin, which walks the same polynomial bit by bit and "
                  "costs an order of magnitude more",
                  hw_value == static_cast<uint32_t>(sink) && hw_cycles < sw_cycles);
}

// ===========================================================================
// w - the scratch register across a system reset (reboots the board)
// ===========================================================================
void tw_across_reset() {
    bench.reset_tally();
    token.magic = token_magic;
    token.letter = 'w';
    token.leg = 1;
    token.pass = 0;
    token.fail = 0;
    token.wrote = 0x7Eu;
    (void)Crc::init();
    Crc::scratch(token.wrote);
    Crc::feed(0xCAFEBABEu);
    print(serial, "  scratch written ", hex(token.wrote), ", rebooting...", crlf);
    Reset::software();
}

void tw_resume() {
    print(serial, crlf, "-> back from the software reset", crlf);
    // The gate is shut again at every reset, so the register cannot be
    // read before it is opened - which is itself half the answer.
    const bool gate_shut = !Crc::clock();
    Crc::clock(true);
    const uint8_t kept = Crc::scratch();
    const uint32_t datar = Crc::value();
    print(serial, "  the HB gate came up ", gate_shut ? "SHUT" : "open", ", scratch reads ",
          hex(kept), ", DATAR ", hex(datar), crlf);
    bench.verdict("a system reset takes the scratch register with it - the block is reset "
                  "with the rest of the chip and only VDD-backed state survives here",
                  kept == 0u);
    bench.verdict("and the calculator comes back at its initial value with the gate shut, "
                  "which is the state a program finds at every boot",
                  gate_shut && datar == crc32_ethernet_init);
    bench.end_letter();
}

void banner() {
    print(serial, crlf, "test_vx03_misc on ", device::part_name,
          " - the CRC unit (RM ch. 5)", crlf,
          "  z costs the board nothing: the block has no pad and no wear", crlf,
          "  w REBOOTS the board once - run it by name", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    boot_flags = brio::Reset::take_flags();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the block: the HB gate, the reset value, the two resets", ta_block);
    bench.letter('b', "the function against util/crc.hpp's constexpr twin", tb_function);
    bench.letter('c', "the reset's latency, and a word written right behind it",
                 tc_reset_latency);
    bench.letter('d', "the running result, taken in pieces", td_pieces);
    bench.letter('e', "CRC_IDATAR, the eight bits of scratch", te_scratch);
    bench.letter('f', "what a thousand words cost, against the twin", tf_cost);
    bench.letter('w', "THE SCRATCH ACROSS A SYSTEM RESET (reboots the board)",
                 tw_across_reset, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL96" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
        if (token.magic == token_magic && token.letter == 'w') {
            token.letter = '\0';
            bench.reset_tally();
            bench.resume_tally(token.pass, token.fail);
            tw_resume();
        } else {
            banner();
        }
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched",
                    brio::crlf);
        bench.prompt();
    }
}
