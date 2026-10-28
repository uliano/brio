// test_ch32_dma - the reference bench suite for the CH32V00x's DMA
// chapter: the seven channels' controller, a channel's refusals and
// flags, memory-to-memory blocks at the three widths, the transfer
// error, the interrupt - and the two engine slots of ch32v00x/usart.hpp
// exercised by THIS SUITE'S OWN CONSOLE, which transmits on channel 4
// and receives on channel 5: every line printed here left the ring on
// a DMA block, every keystroke arrived through the channel's circular
// ring.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE. The console is the WCH-Link's own serial.
//
// What is exercised, letter by letter:
//   a  the block and a channel as found, and the refusals: a config the
//      chapter forbids, a store into an enabled channel
//   b  memory-to-memory at byte, half-word and word widths: the copy
//      exact, CNTR counting down to zero, the completion flag, GIF
//      alongside it, the flags cleared by the write-one register
//   c  the interrupt: a block completing on channel 1 counted in its
//      own handler, with the channel's ISR body reading only the armed
//      flags
//   d  a transfer error: RM 8.2.1 promises TEIF and an automatic EN drop
//      for a block that reads or writes a reserved address; this letter
//      probes a handful of holes in the map, read and written, and a write
//      into the flash (locked, onto a word already zero, so nothing can
//      change), and REPORTS what the silicon does, judging only that each
//      block ended one way or the other
//   e  the console's transmit engine: a burst longer than the TX ring
//      printed through it, the blocks counted, no fault, and the run
//      timed against its wire at the rate asked - the host reading the
//      console would not see a divisor a few per cent off
//   f  the console's receive engine: what the ring holds is what was
//      typed - this letter asks for a line and echoes it
//   g  the copy engine on channel 7: copy and fill at the three beats
//      exact, polled and by its own interrupt, its refusals - a
//      half-word run off its boundary (RM 8.3.6), a count of zero or
//      over CNTR's, a block on a busy engine - and abandon()
//   h  UNEQUAL WIDTHS (RM 8.2.2, table 8-1): four items copied at every
//      pair of source and target widths, the nine cases, each target byte
//      against the table - a narrow source zero-filled into a wide
//      target, a wide one truncated to its low bytes
//   i  THE LEVEL AGAINST THE CHANNEL NUMBER (8.2.1): two memory-to-memory
//      blocks started one store apart, channel 1 at low and channel 7 at
//      very_high - which one the arbiter serves read off the counts in
//      quiet windows - and the control, one level, the lower number first
//
// build: boards = v006k8,v003f4
// build: groups = abcefg,dhi
// (on the CH32V003 the suite is two images, test_ch32_dma-1 and -2: whole
// it overflows the part's 15 KB)
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include <span>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/nvm.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

// The console on both engines: USART1 transmits on channel 4, receives
// on channel 5 (RM table 8-2).
using Serial = Uart<1, P, 64, 128, DmaTxEngine<4>, DmaRxEngine<5>>;
constexpr Serial serial;

// The channel the memory-to-memory letters use: 1, the ADC's, idle here.
using Copier = DmaChannel<1>;
// The copy engine's: 7, I2C1's and USART2's receive, idle here.
using CopyEngine = DmaCopyEngine<7>;

TestBench<Serial> bench;

volatile uint32_t copier_completions = 0;
volatile uint32_t copier_errors = 0;
volatile uint32_t tx_blocks = 0;
volatile uint32_t engine_completions = 0;

alignas(4) uint8_t src8[64];
alignas(4) uint8_t dst8[64];
uint16_t src16[32];
uint16_t dst16[32];
uint32_t src32[16];
uint32_t dst32[16];

bool wait_complete(uint32_t polls = 1'000'000u) {
    for (uint32_t i = 0; i < polls; ++i) {
        if (Copier::flag(DmaFlag::complete) || Copier::flag(DmaFlag::error)) {
            return true;
        }
    }
    return false;
}

// One memory-to-memory block: `count` items of `width` from src to dst.
bool copy_block(volatile void* src, volatile void* dst, uint16_t count, DmaWidth width) {
    Copier::stop();
    const bool loaded = Copier::load(DmaTransfer{
        .peripheral = src,
        .memory = dst,
        .count = count,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = true,
                   .peripheral_increment = true,
                   .memory_increment = true,
                   .peripheral_width = width,
                   .memory_width = width,
                   .priority = DmaPriority::high},
    });
    return loaded && wait_complete();
}

// ---------------------------------------------------------------------------
// a - the block and the refusals
// ---------------------------------------------------------------------------
void ta_block() {
    Dma::open();
    print(serial, "  HBPCENR=", hex(rcc()->HBPCENR), " INTFR=", hex(Dma::flags()),
          " ch1 CFGR=", hex(Copier::regs().CFGR), crlf);
    bench.verdict("the block's gate is open (DMA1EN)", (rcc()->HBPCENR & rcc_hb_dma1) != 0u);
    Copier::stop();
    bench.verdict("channel 1 is disabled as found", !Copier::enabled());
    bench.verdict("circular + memory-to-memory is refused (RM 8.2.1)",
                  !Copier::configure({.circular = true, .memory_to_memory = true}));
    bench.verdict("a zero count is refused", !Copier::set_count(0));
    bench.verdict("a transfer with a null end is refused",
                  !Copier::load(DmaTransfer{.peripheral = nullptr, .memory = dst8, .count = 4}));
    // A configure into an ENABLED channel is refused: start a long copy
    // and try to reconfigure under it.
    for (uint32_t i = 0; i < 64u; ++i) { src8[i] = static_cast<uint8_t>(i); }
    (void)Copier::load(DmaTransfer{.peripheral = src8, .memory = dst8, .count = 64,
                                   .config = {.memory_to_memory = true, .peripheral_increment = true}});
    const bool refused = !Copier::configure({.memory_to_memory = true}) &&
                         !Copier::set_count(8);
    (void)wait_complete();
    bench.verdict("a store into an enabled channel is refused (its fields are "
                  "read-only then)",
                  refused || !Copier::enabled());   // or the block was already done
    Copier::stop();
    bench.verdict("stop() disables and clears", !Copier::enabled() && Copier::flags() == 0u);
}

// ---------------------------------------------------------------------------
// b - memory to memory at three widths
// ---------------------------------------------------------------------------
void tb_copies() {
    for (uint32_t i = 0; i < 64u; ++i) { src8[i] = static_cast<uint8_t>(i * 7u + 1u); dst8[i] = 0; }
    for (uint32_t i = 0; i < 32u; ++i) { src16[i] = static_cast<uint16_t>(i * 771u + 5u); dst16[i] = 0; }
    for (uint32_t i = 0; i < 16u; ++i) { src32[i] = i * 0x01010101u + 0xA5u; dst32[i] = 0; }

    bool ok = copy_block(src8, dst8, 64, DmaWidth::byte);
    bench.verdict("64 bytes copied memory to memory", ok && memcmp(src8, dst8, 64) == 0);
    bench.verdict("CNTR counted down to zero", Copier::count() == 0u);
    bench.verdict("the completion flag is up, and GIF beside it",
                  Copier::flag(DmaFlag::complete) && Copier::flag(DmaFlag::global));
    bench.verdict("no error", !Copier::flag(DmaFlag::error));
    Copier::clear(DmaFlag::all);
    bench.verdict("the write-one clear takes every flag down", Copier::flags() == 0u);

    ok = copy_block(src16, dst16, 32, DmaWidth::half);
    bench.verdict("32 half-words copied", ok && memcmp(src16, dst16, 64) == 0);
    ok = copy_block(src32, dst32, 16, DmaWidth::word);
    bench.verdict("16 words copied", ok && memcmp(src32, dst32, 64) == 0);

    // The rate: 64 bytes at each width timed on the STK (HCLK cycles),
    // the tick held off so the count is the block's alone.
    for (const DmaWidth w : {DmaWidth::byte, DmaWidth::half, DmaWidth::word}) {
        const uint16_t n = w == DmaWidth::byte ? 64u : w == DmaWidth::half ? 32u : 16u;
        Copier::stop();
        uint32_t cycles = 0;
        {
            InterruptGuard guard;
            const uint32_t t0 = stk()->CNT;
            (void)Copier::load(DmaTransfer{.peripheral = src8, .memory = dst8, .count = n,
                                           .config = {.memory_to_memory = true, .peripheral_increment = true,
                                                      .memory_increment = true, .peripheral_width = w,
                                                      .memory_width = w}});
            (void)wait_complete();
            const uint32_t t1 = stk()->CNT;
            cycles = t1 >= t0 ? t1 - t0 : t1 + (stk()->CMP + 1u) - t0;
        }
        print(serial, "  64 bytes as ", n, " item(s): ", cycles, " HCLK cycles from load to TCIF", crlf);
    }

    // A fixed source spread over a run: the peripheral side not incrementing.
    const uint32_t pattern = 0xDEADBEEFu;
    Copier::stop();
    (void)Copier::load(DmaTransfer{.peripheral = const_cast<uint32_t*>(&pattern), .memory = dst32, .count = 16,
                                   .config = {.memory_to_memory = true, .peripheral_increment = false,
                                              .memory_increment = true, .peripheral_width = DmaWidth::word,
                                              .memory_width = DmaWidth::word}});
    (void)wait_complete();
    bool filled = true;
    for (uint32_t i = 0; i < 16u; ++i) { filled = filled && dst32[i] == pattern; }
    bench.verdict("a non-incrementing source fills the run with one word", filled);
    Copier::stop();
}

// ---------------------------------------------------------------------------
// c - the interrupt
// ---------------------------------------------------------------------------
void tc_interrupt() {
    copier_completions = 0;
    copier_errors = 0;
    Copier::stop();
    Copier::arm(DmaFlag::complete | DmaFlag::error, true);
    Pfic::enable(Copier::irq());
    for (uint8_t round = 0; round < 5u; ++round) {
        Copier::enable(false);
        (void)Copier::load(DmaTransfer{.peripheral = src8, .memory = dst8, .count = 16,
                                       .config = {.memory_to_memory = true, .peripheral_increment = true}});
        (void)delay_us(clock, 200);
    }
    Pfic::disable(Copier::irq());
    print(serial, "  5 blocks: ", copier_completions, " completions in the handler, ",
          copier_errors, " errors", crlf);
    bench.verdict("every completion reached the channel's handler", copier_completions == 5u);
    bench.verdict("and no error did", copier_errors == 0u);
    bench.verdict("the handler cleared the flags as it went", Copier::flags() == 0u);
    Copier::stop();
}

// ---------------------------------------------------------------------------
// d - a transfer error
// ---------------------------------------------------------------------------
/// A word of the image's own read-only data that reads ZERO: the target of
/// the flash write. The flash is locked (FLASH_CTLR.LOCK from reset, and
/// nothing in this image unlocks it), and a programmed bit can only go from
/// one to zero, so a write of zero onto it changes nothing whatever the
/// controller does with it.
[[gnu::used]] const uint32_t flash_zero_word = 0;

/// One block of `count` items, memory to memory, between `src` and `dst`
/// at `width`, on channel 1: how it ended.
struct Probe {
    bool ended;
    uint32_t flags;
    bool enabled;
    uint16_t left;
};
Probe probe_block(volatile void* src, volatile void* dst, DmaWidth width, uint16_t count = 4) {
    Copier::stop();
    const bool loaded = Copier::load(DmaTransfer{
        .peripheral = src, .memory = dst, .count = count,
        .config = {.memory_to_memory = true, .peripheral_increment = true, .memory_increment = true,
                   .peripheral_width = width, .memory_width = width}});
    const bool ended = loaded && wait_complete(200'000u);
    return {ended, Copier::flags(), Copier::enabled(), static_cast<uint16_t>(Copier::count())};
}

void td_error() {
    // Holes in the map (RM 1.2): between the peripheral buses and past
    // them, and the top of the address space. What the controller does
    // with a read from each, and with a write into each, is the finding.
    constexpr uint32_t holes[] = {0x50000000u, 0x60000000u, 0x30000000u, 0x40030000u, 0xF0000000u};
    uint8_t errors = 0, completions = 0, hung = 0;
    const auto tally = [&](const Probe& p) {
        if (!p.ended) { ++hung; }
        else if ((p.flags & DmaFlag::error) != 0u) { ++errors; }
        else { ++completions; }
    };
    for (const uint32_t hole : holes) {
        memset(dst8, 0x55, 4);
        const Probe p = probe_block(reinterpret_cast<volatile void*>(hole), dst8, DmaWidth::byte);
        print(serial, "  read from ", hex(hole), ": flags=", hex(p.flags), " EN=", p.enabled,
              " CNTR=", p.left, " data=", hex(dst8[0]), crlf);
        tally(p);
    }
    for (const uint32_t hole : holes) {
        memset(src8, 0x55, 4);
        const Probe p = probe_block(src8, reinterpret_cast<volatile void*>(hole), DmaWidth::byte);
        print(serial, "  write into ", hex(hole), ": flags=", hex(p.flags), " EN=", p.enabled,
              " CNTR=", p.left, crlf);
        tally(p);
    }
    // The flash, a legal destination by 8.2.1's list: ONE word of zero
    // (from the SRAM) written onto the image's own zero word, the flash
    // locked - a block of one item, so no other word of the image is
    // offered to it.
    static uint32_t zero = 0;
    const Probe f = probe_block(&zero, const_cast<uint32_t*>(&flash_zero_word), DmaWidth::word, 1);
    const uint32_t word_after = *static_cast<const volatile uint32_t*>(&flash_zero_word);
    print(serial, "  write into the flash at ", hex(reinterpret_cast<uintptr_t>(&flash_zero_word)),
          " (locked: FLASH_CTLR=", hex(flash_ctl()->CTLR), "): flags=", hex(f.flags), " EN=", f.enabled,
          " CNTR=", f.left, crlf);
    // And with the program/erase controller's lock OPEN and no operation
    // armed (no program or erase bit): the same word written through the
    // array's own address and through its 0x0800 0000 alias, which is the
    // one the chapter's programming sequence writes; STATR read after each
    // for the controller's own refusal (WRPRTERR). Locked again at once.
    const uint32_t word_addr = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&flash_zero_word));
    const uint32_t alias_addr = 0x08000000u + (word_addr & 0x00FFFFFFu);
    (void)Flash::unlock();
    const bool opened = (flash_ctl()->CTLR & flash_lock) == 0u;
    flash_ctl()->STATR = flash_wrprterr | flash_eop;   // write-1-clear
    const Probe fu = probe_block(&zero, reinterpret_cast<volatile void*>(word_addr), DmaWidth::word, 1);
    const uint32_t st_u = flash_ctl()->STATR;
    const Probe fa = probe_block(&zero, reinterpret_cast<volatile void*>(alias_addr), DmaWidth::word, 1);
    const uint32_t st_a = flash_ctl()->STATR;
    Flash::lock();
    const bool relocked = (flash_ctl()->CTLR & flash_lock) != 0u;
    const uint32_t word_end = *static_cast<const volatile uint32_t*>(&flash_zero_word);
    print(serial, "  the lock open (", opened ? "opened" : "NOT OPENED", "), no operation armed: at ", hex(word_addr),
          " flags=", hex(fu.flags), " EN=", fu.enabled, " STATR=", hex(st_u), "; at the alias ", hex(alias_addr),
          " flags=", hex(fa.flags), " EN=", fa.enabled, " STATR=", hex(st_a), "; locked again ", relocked, crlf);
    Copier::stop();
    print(serial, "  ", errors, " error(s), ", completions, " completion(s), ", hung,
          " hung, over 5 holes read and written", crlf);
    bench.verdict("every block from or into a hole ENDED (no channel left hanging)", hung == 0u);
    if (errors == 0u) {
        print(serial, "  -> NO HOLE RAISES TEIF ON THIS SILICON: a DMA read or write of unmapped "
                      "space completes as a normal block", crlf);
    }
    bench.verdict("the outcome was one of the two the chapter allows for every hole "
                  "(an error, or a completion the silicon chose to give)",
                  errors + completions == 10u);
    const bool flash_error = f.ended && (f.flags & DmaFlag::error) != 0u;
    if (flash_error) {
        print(serial, "  -> A WRITE INTO THE LOCKED FLASH RAISES TEIF: EN ", f.enabled ? "STILL SET" : "cleared",
              ", ", f.left, " item(s) left", crlf);
    }
    bench.verdict("the write into the locked flash ENDED one of the two ways, and the word is still zero",
                  f.ended && word_after == 0u);
    bench.verdict("and with the lock open, through the array and its alias: each ENDED, the word still zero, "
                  "the lock closed again",
                  opened && fu.ended && fa.ended && word_end == 0u && relocked);
}

// ---------------------------------------------------------------------------
// e - the console's transmit engine
// ---------------------------------------------------------------------------
void te_tx_engine() {
    // The line idle first: what the menu printed is out.
    while (!Serial::tx_idle()) {
    }
    const uint32_t blocks_before = tx_blocks;
    // Longer than the 128-byte TX ring: print() blocks on the full ring
    // and the engine drains it run by run. Timed on the ticker's cycle
    // count from the first byte handed over to tx_idle() - TC after the
    // last block - against 2720 bits at the rate asked: a console the
    // host reads at 115200 tolerates a divisor a few per cent off, the
    // run does not.
    const uint32_t t0 = Ticker::cycles();
    for (uint8_t line = 0; line < 4u; ++line) {
        print(serial, "  0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef", crlf);
    }
    while (!Serial::tx_idle()) {
    }
    const uint32_t took = Ticker::cycles() - t0;
    // 272 frames of ten bits at 115200 (BRR 417 for 416.7: the divisor's
    // own rounding is 0.8 thousandths slow).
    // In 32 bits (the CH32V003 has no multiplier to spend on a 64-bit
    // division): a run over 4 M cycles, three and a half times its wire,
    // reads as 9999 rather than overflow.
    constexpr uint32_t wire = static_cast<uint32_t>(2720ull * SysClock::hz / 115200u);
    const uint32_t permille = took < 4'000'000u ? 1000u * took / wire : 9999u;
    (void)delay_us(clock, 500);
    const uint32_t blocks = tx_blocks - blocks_before;
    const bool drained = Serial::tx_idle();   // before the next print refills it
    print(serial, "  272 bytes through the transmit engine in ", blocks, " block(s), faults ",
          Serial::dma_faults(), "; the run in ", permille, " thousandths of its wire time at 115200", crlf);
    bench.verdict("a burst longer than the ring went out in blocks", blocks >= 2u);
    bench.verdict("with no transfer fault", Serial::dma_faults() == 0u);
    bench.verdict("and the ring drained to empty", drained);
    bench.verdict("at the rate asked: 2720 bits at 115200 within -1 % to +3 % of their wire time",
                  permille >= 990u && permille <= 1030u);
}

// ---------------------------------------------------------------------------
// f - the console's receive engine
// ---------------------------------------------------------------------------
void tf_rx_engine() {
    print(serial, "  type up to 32 characters and Enter within 15 s:", crlf, "  > ");
    char line[33];
    uint8_t n = 0;
    const uint32_t t0 = Ticker::ticks();
    bool done = false;
    while (!done && Ticker::ticks() - t0 < 15'000u) {
        uint8_t c;
        while (Serial::read_byte(c)) {
            if (c == '\n' || c == '\r') {
                done = n > 0u;
            } else if (n < 32u) {
                line[n++] = static_cast<char>(c);
            }
        }
    }
    line[n] = '\0';
    print(serial, crlf, "  read ", n, " byte(s): \"", line, "\"", crlf);
    bench.verdict("a line arrived through the receive engine", done);
    bench.verdict("with no software overrun and no fault",
                  Serial::rx_overruns() == 0u && Serial::dma_faults() == 0u);
}

// ---------------------------------------------------------------------------
// g - the copy engine
// ---------------------------------------------------------------------------
/// The engine idle again, bounded: busy() reads the flags while a block
/// runs, polled or not.
bool engine_wait() {
    for (uint32_t i = 0; i < 1'000'000u; ++i) {
        if (!CopyEngine::busy()) {
            return true;
        }
    }
    return false;
}

void tg_copy_engine() {
    CopyEngine::arm(DmaPriority::high, false);   // polled: no interrupt
    for (uint32_t i = 0; i < 64u; ++i) { src8[i] = static_cast<uint8_t>(i * 5u + 3u); dst8[i] = 0; }
    for (uint32_t i = 0; i < 32u; ++i) { src16[i] = static_cast<uint16_t>(i * 1031u + 7u); dst16[i] = 0; }
    for (uint32_t i = 0; i < 16u; ++i) { src32[i] = i * 0x01020304u + 0x5Au; dst32[i] = 0; }

    bool ok = CopyEngine::copy(dst8, src8, 64u) && engine_wait();
    bench.verdict("64 bytes copied in byte beats, polled", ok && memcmp(src8, dst8, 64) == 0);
    ok = CopyEngine::copy(dst16, src16, 32u) && engine_wait();
    bench.verdict("32 half-words copied in half-word beats", ok && memcmp(src16, dst16, 64) == 0);
    ok = CopyEngine::copy(dst32, src32, 16u) && engine_wait();
    bench.verdict("16 words copied in word beats", ok && memcmp(src32, dst32, 64) == 0);

    static const uint32_t word_cell = 0xC0DE'F00Du;
    static const uint16_t half_cell = 0xBEEFu;
    ok = CopyEngine::fill(dst32, &word_cell, 16u) && engine_wait();
    bool filled = ok;
    for (uint32_t i = 0; i < 16u; ++i) { filled = filled && dst32[i] == word_cell; }
    ok = CopyEngine::fill(dst16, &half_cell, 32u) && engine_wait();
    for (uint32_t i = 0; i < 32u; ++i) { filled = filled && ok && dst16[i] == half_cell; }
    bench.verdict("a fill from one cell, words and half-words: every element the cell", filled);

    // The odd address passes through an empty asm, so that the refusal is
    // the engine's run-time check and not one the compiler folded.
    uint8_t* odd = dst8 + 1;
    __asm__ volatile("" : "+r"(odd));
    bench.verdict("a half-word run off its boundary is refused (8.3.6: bit 0 would be ignored)",
                  !CopyEngine::copy(reinterpret_cast<uint16_t*>(odd), src16, 4u));
    bench.verdict("a block of no element, or of more than CNTR counts, is refused",
                  !CopyEngine::copy(dst8, src8, 0u) && !CopyEngine::copy(dst8, src8, 65536u));
    const bool first = CopyEngine::copy(dst8, src8, 64u);
    const bool second = CopyEngine::copy(dst8, src8, 64u);
    const bool was_busy = CopyEngine::busy();
    (void)engine_wait();
    bench.verdict("a block on a busy engine is refused", first && (!second || !was_busy));
    // abandon(): a fill of 64 bytes in byte beats (some 380 cycles of the
    // controller's) stopped where it stands, CNTR holding what was left.
    for (uint32_t i = 0; i < 64u; ++i) { dst8[i] = 0; }
    const bool long_started = CopyEngine::fill(dst8, &src8[0], 64u);
    const bool stopped = CopyEngine::abandon();
    const bool idle_after = !CopyEngine::busy() && !CopyEngine::abandon();
    print(serial, "  abandon(): started ", long_started, ", stopped ", stopped, ", CNTR left ",
          DmaChannel<7>::count(), crlf);
    bench.verdict("abandon() stops a running block and frees the engine; a second one finds nothing",
                  long_started && idle_after);
    bench.verdict("no transfer fault, polled", CopyEngine::faults() == 0u);

    // By its own interrupt: five blocks, each ended by the vector.
    CopyEngine::arm(DmaPriority::high, true);
    engine_completions = 0;
    uint8_t started = 0;
    for (uint8_t round = 0; round < 5u; ++round) {
        if (CopyEngine::copy(dst32, src32, 16u)) {
            ++started;
        }
        (void)delay_us(clock, 100);
    }
    print(serial, "  5 blocks by the interrupt: ", started, " started, ", engine_completions,
          " completions in the handler, faults ", CopyEngine::faults(), crlf);
    bench.verdict("every block started and ended in the channel's handler",
                  started == 5u && engine_completions == 5u && !CopyEngine::busy());
    (void)CopyEngine::abandon();
}

// ---------------------------------------------------------------------------
// h - unequal widths (table 8-1)
// ---------------------------------------------------------------------------
/// The width in bytes.
constexpr uint8_t bytes_of(DmaWidth w) {
    return w == DmaWidth::byte ? 1u : w == DmaWidth::half ? 2u : 4u;
}

void th_widths() {
    // The source bytes as the table names them, B0 at the lowest address;
    // the target painted with a value no case writes, four guard bytes
    // past the widest target (4 items x 4 bytes).
    alignas(4) static uint8_t src[16];
    alignas(4) static uint8_t dst[20];
    for (uint8_t i = 0; i < 16u; ++i) {
        src[i] = static_cast<uint8_t>(0xB0u + i);
    }
    constexpr DmaWidth widths[] = {DmaWidth::byte, DmaWidth::half, DmaWidth::word};
    uint8_t exact = 0;
    for (const DmaWidth sw : widths) {
        for (const DmaWidth tw : widths) {
            memset(dst, 0xEE, sizeof dst);
            Copier::stop();
            const bool loaded = Copier::load(DmaTransfer{
                .peripheral = src, .memory = dst, .count = 4,
                .config = {.memory_to_memory = true, .peripheral_increment = true, .memory_increment = true,
                           .peripheral_width = sw, .memory_width = tw}});
            const bool ended = loaded && wait_complete();
            __asm__ volatile("" ::: "memory");
            // Table 8-1: item i is read at i x the source's width and
            // written at i x the target's, little-endian; a narrower source
            // is zero-filled above, a wider one keeps its low bytes.
            const uint8_t sb = bytes_of(sw);
            const uint8_t tb = bytes_of(tw);
            bool ok = ended && (Copier::flags() & DmaFlag::error) == 0u;
            for (uint8_t item = 0; item < 4u; ++item) {
                for (uint8_t k = 0; k < tb; ++k) {
                    const uint8_t want = k < sb ? src[item * sb + k] : 0u;
                    ok = ok && dst[item * tb + k] == want;
                }
            }
            for (uint8_t k = static_cast<uint8_t>(4u * tb); k < sizeof dst; ++k) {
                ok = ok && dst[k] == 0xEEu;   // nothing past the fourth item
            }
            print(serial, "  ", 8u * sb, " -> ", 8u * tb, ":");
            for (uint8_t k = 0; k < 4u * tb; ++k) {
                print(serial, " ", hex(dst[k]));
            }
            print(serial, ok ? "  as the table" : "  NOT AS THE TABLE", crlf);
            if (ok) {
                ++exact;
            }
        }
    }
    Copier::stop();
    bench.verdict("the nine pairs of source and target widths land as table 8-1 spells them: a narrow "
                  "source zero-filled, a wide one truncated to its low bytes, nothing past the fourth item",
                  exact == 9u);
}

// ---------------------------------------------------------------------------
// i - the level against the channel number
// ---------------------------------------------------------------------------
/// Two memory-to-memory blocks of 1024 words between fixed cells (no
/// increment on either side: the arbiter's order is in the counts, not
/// in the data), `first` enabled and `second` one store later, interrupts
/// held off: `first`'s count once both run, both counts after each of
/// four quiet windows - a loop of the core's own, no access to the
/// controller - and `first`'s when `second` completes. Each item is about
/// six HCLK cycles (letter b), a block some 6000; a window is about a
/// hundred cycles, so four of them fall inside the second block whichever
/// channel the arbiter serves.
struct Race {
    uint16_t at_start;
    uint16_t a[4];
    uint16_t b[4];
    uint16_t left;

    /// The first moved nothing across the windows while the second moved
    /// (an item in flight when the second came lands before the first
    /// window closes).
    bool first_held() const {
        return a[0] == a[3] && at_start - a[0] <= 1 && b[0] > b[1] && b[1] > b[2] && b[2] != 0u;
    }
};

volatile uint32_t race_cell_a = 0x11111111u;
volatile uint32_t race_cell_b = 0;
volatile uint32_t race_cell_c = 0x22222222u;
volatile uint32_t race_cell_d = 0;

template <uint8_t first_ch, uint8_t second_ch>
Race race(DmaPriority first_level, DmaPriority second_level) {
    using A = DmaChannel<first_ch>;
    using B = DmaChannel<second_ch>;
    constexpr uint16_t words = 1024;
    A::stop();
    B::stop();
    (void)A::prepare(DmaTransfer{.peripheral = &race_cell_a, .memory = &race_cell_b, .count = words,
                                 .config = {.memory_to_memory = true, .memory_increment = false,
                                            .peripheral_width = DmaWidth::word, .memory_width = DmaWidth::word,
                                            .priority = first_level}});
    (void)B::prepare(DmaTransfer{.peripheral = &race_cell_c, .memory = &race_cell_d, .count = words,
                                 .config = {.memory_to_memory = true, .memory_increment = false,
                                            .peripheral_width = DmaWidth::word, .memory_width = DmaWidth::word,
                                            .priority = second_level}});
    Race r{};
    {
        InterruptGuard guard;
        A::regs().CFGR = A::regs().CFGR | dma_cfgr_en;
        B::regs().CFGR = B::regs().CFGR | dma_cfgr_en;
        r.at_start = static_cast<uint16_t>(A::count());
        for (uint8_t k = 0; k < 4u; ++k) {
            for (uint32_t i = 0; i < 40u; ++i) {
                __asm__ volatile("nop");
            }
            r.a[k] = static_cast<uint16_t>(A::count());
            r.b[k] = static_cast<uint16_t>(B::count());
        }
        // Bounded: 2 x 1024 items at about six cycles each is some 12 000
        // cycles; a million turns of a poll is far beyond it.
        for (uint32_t i = 0; i < 1'000'000u && (B::flags() & DmaFlag::complete) == 0u; ++i) {
        }
        r.left = static_cast<uint16_t>(A::count());
        for (uint32_t i = 0; i < 1'000'000u && (A::flags() & DmaFlag::complete) == 0u; ++i) {
        }
    }
    print(serial, "    ch", first_ch, " (PL ", static_cast<uint8_t>(first_level), ") first, ch", second_ch,
          " (PL ", static_cast<uint8_t>(second_level), ") next: ch", first_ch, " had ", r.at_start,
          " of 1024 left once both were on", crlf, "      after four quiet windows ch", first_ch, " ", r.a[0],
          " ", r.a[1], " ", r.a[2], " ", r.a[3], ", ch", second_ch, " ", r.b[0], " ", r.b[1], " ", r.b[2],
          " ", r.b[3], "; ch", first_ch, " ", r.left, " when ch", second_ch, " completed", crlf);
    A::stop();
    B::stop();
    return r;
}

void ti_levels() {
    // A higher level on the HIGHER channel number: does the level decide?
    const Race lv = race<1, 7>(DmaPriority::low, DmaPriority::very_high);
    // The control, one level, the higher number first: the lower number
    // takes over (the tie rule letter u of test_ch32_serial measured).
    const Race tie = race<7, 1>(DmaPriority::medium, DmaPriority::medium);
    bench.verdict("a higher level wins over a lower channel number: channel 1 at low, first, moved nothing "
                  "in the quiet windows while channel 7's very_high block ran",
                  lv.first_held());
    bench.verdict("at one level the lower number wins mid-block: channel 7, first, moved nothing in the "
                  "windows while channel 1's block ran",
                  tie.first_held());
}

void banner() {
    print(serial, crlf, "test_ch32_dma - ", device::part_name, " (console on DMA channels 4 and 5)", crlf);
    bench.menu();
}

} // namespace

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_VECTOR(usart1_handler) { (void)Serial::isr(); }
/// Channel 4 is the transmit engine's alone: every entry is a block's
/// completion (dma_isr()'s answer is the RECEIVE edge, none of its).
BRIO_CH32_VECTOR(dma1_channel4_handler) {
    tx_blocks = tx_blocks + 1u;
    (void)Serial::dma_isr();
}
BRIO_CH32_VECTOR(dma1_channel5_handler) { (void)Serial::dma_isr(); }
BRIO_CH32_VECTOR(dma1_channel7_handler) {
    if (CopyEngine::isr() != 0u) {
        engine_completions = engine_completions + 1u;
    }
}
BRIO_CH32_VECTOR(dma1_channel1_handler) {
    const uint32_t f = Copier::isr();
    if ((f & brio::DmaFlag::complete) != 0u) { copier_completions = copier_completions + 1u; }
    if ((f & brio::DmaFlag::error) != 0u) { copier_errors = copier_errors + 1u; }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the block and the refusals", ta_block);
    bench.letter('b', "memory to memory at three widths", tb_copies);
    bench.letter('c', "the interrupt", tc_interrupt);
    bench.letter('d', "a transfer error", td_error);
    bench.letter('e', "the console's transmit engine", te_tx_engine);
    bench.letter('f', "the console's receive engine (asks for a line)", tf_rx_engine, false);
    bench.letter('g', "the copy engine: three beats, copy and fill, polled and by interrupt, refusals",
                 tg_copy_engine);
    bench.letter('h', "unequal widths: the nine pairs of table 8-1", th_widths);
    bench.letter('i', "the level against the channel number", ti_levels);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
