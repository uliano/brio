// test_x035_dma - the reference bench suite for the CH32X035's DMA
// controller (RM ch. 9): the eight channels' block as found, a channel's
// refusals and flags, memory-to-memory blocks at the three widths, the
// eight vectors, the transfer error, circular mode, the release contract's
// held request, the arbiter's order between two channels, what the core's
// Sleep does to a working channel, the copy engine - and the two engine
// slots of ch32x035/usart.hpp exercised by THIS SUITE'S OWN CONSOLE, which
// transmits on channel 7 and receives on channel 6: every line printed
// here left the ring on a DMA block, every keystroke arrived through the
// channel's circular ring. Letter m is the benchmark grammar's row
// (docs/design/benchmark.md): copy, fill and a paced block.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE, AND NO PAD DRIVEN. The console is USART2 on PA2/PA3
// over the probe's serial. The paced loads are USART3's TRANSMITTER, whose
// request (TXE under DMAT, channel 2) comes at its frame rate: its column
// is never handed a pad (no Pin is formed for it), so the frames go
// nowhere - this package bonds none of its default column's pads anyway.
// The desk's link pads (PB0, PB1, PA4..PA7, PB3) are not touched.
//
// What is exercised, letter by letter:
//   a  the block as found - DMA1EN closed at reset, opened by the console's
//      engines - and the refusals: a config the chapter forbids, a store
//      into an enabled channel
//   b  memory to memory at byte, half-word and word widths: the copy exact,
//      CNTR down to zero, TCIF with GIF, the write-one clear; EN STANDING
//      after a completed block; a half-word block on an odd address moving
//      the even one's bytes (9.3.5/9.3.6); the rate at each width
//   c  the vectors: a block on each of channels 1..5 and 8 ended by its own
//      handler (6 and 7 are the console's engines'), the eighth at entry 44
//   d  a transfer error: 9.2.1 promises TEIF and an automatic EN drop for a
//      reserved address; holes in the map are read and written, and one
//      word written into the flash (locked, onto a word already zero, so
//      nothing can change), and what the silicon does is REPORTED, the
//      verdict only that each block ended
//   e  the console's transmit engine: a burst longer than its ring printed
//      through it, the blocks counted, no fault, timed against its wire
//   f  the console's receive engine: a typed line (asks; not in z)
//   g  the copy engine: copy and fill at the three beats, polled and by its
//      own interrupt, its refusals and abandon()
//   h  circular mode: USART3's transmitter paces a circular block of four
//      bytes for twelve laps, HTIF and TCIF each lap, CNTR reloaded
//   i  THE RELEASE CONTRACT: USART3's transmit request raised under DMAT and
//      released four ways - its enable, the block, the gate, the reset
//      line - then channel 2 bound by its next owner: does it move a stray
//      item at its first enable?
//   j  the arbiter: two memory-to-memory blocks on channels 1 and 8, at two
//      levels and at one, their order read off the counts
//   k  THE CORE'S SLEEP: a copy and a paced block started before the
//      platform's idle() - does the controller move while the core sleeps?
//   m  the benchmark row: copy, fill and copy from the flash at 16, 256 and
//      4096 bytes, and 1024 bytes paced by USART3 at 1 Mbaud - the lines of
//      docs/design/benchmark.md's grammar
//   n  UNEQUAL WIDTHS (RM 9.2.2, table 9-1): four items copied at every pair
//      of source and target widths, the nine cases, each target byte against
//      the table - a narrow source zero-filled into a wide target, a wide one
//      truncated to its low bytes
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include <span>

#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

// The console on both engines: USART2 transmits on channel 7 and receives
// on channel 6 (ch32x035/dma_engine.hpp's table).
using Serial = Uart<2, P, 64, 128, UartFormat{}, DmaTxEngine<7>, DmaRxEngine<6>>;
constexpr Serial serial;

/// The STK's cycle count as the bench's ruler: flatten pulls the ticker's
/// read into this function and always_inline pulls it into every stamp.
struct Ruler {
    [[gnu::always_inline, gnu::flatten]] static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

using Plat = BenchIdle<P, Ruler>;
using Meter = IsrMeter<Ruler, Plat>;

TestBench<Serial> bench;

// The one-shot channel the letters program by hand: 1 (the ADC's, USART4's
// transmit - neither running here).
using Copier = DmaChannel<1>;
// The copy engine: 5 (USART1's receive, TIM1's update - neither here).
using CopyEngine = DmaCopyEngine<5>;
// USART3's transmit request lands on channel 2: the paced loads' engine.
using U3 = Usart<3>;
using Paced = DmaTxEngine<2>;

uint32_t dma_gate_at_boot = 0;   // RCC_AHBPCENR before the console opened anything

volatile uint32_t vector_hits[9] = {};   // per channel, by the bound handlers
volatile uint32_t engine_completions = 0;
volatile uint32_t tx_blocks = 0;
volatile uint32_t laps = 0;
volatile uint32_t half_laps = 0;
volatile bool copy_done = false;
volatile bool paced_done = false;
/// What each test channel's vector does: count, or end a copy / a paced
/// block, or serve a circular lap.
enum class Role : uint8_t { count, copy_engine, paced, circular };
volatile Role role2 = Role::count;
volatile Role role5 = Role::count;

Meter tick_meter;
Meter usart_meter;
Meter dma_meter;

alignas(4) uint8_t src8[64];
alignas(4) uint8_t dst8[64];
uint16_t src16[32];
uint16_t dst16[32];
uint32_t src32[16];
uint32_t dst32[16];

/// The two big buffers the timed letters move: 4096 bytes each.
constexpr uint32_t big_bytes = 4096;
alignas(4) uint8_t big_a[big_bytes];
alignas(4) uint8_t big_b[big_bytes];

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 200);
}

/// Poll channel 1's flags. The barrier on the way out: the controller wrote
/// memory behind the compiler's back, which must not answer a read after
/// this from what it stored before the block.
bool wait_flag(uint32_t mask, uint32_t polls = 1'000'000u) {
    for (uint32_t i = 0; i < polls; ++i) {
        if ((Copier::flags() & mask) != 0u) {
            __asm__ volatile("" ::: "memory");
            return true;
        }
    }
    __asm__ volatile("" ::: "memory");
    return false;
}

bool copy_block(volatile void* src, volatile void* dst, uint16_t count, DmaWidth width) {
    Copier::stop();
    const bool loaded = Copier::load(DmaTransfer{
        .peripheral = src,
        .memory = dst,
        .count = count,
        .config = {.memory_to_memory = true,
                   .peripheral_increment = true,
                   .memory_increment = true,
                   .peripheral_width = width,
                   .memory_width = width,
                   .priority = DmaPriority::high},
    });
    return loaded && wait_flag(DmaFlag::complete | DmaFlag::error);
}

/// The STK's own counter between two reads, taken masked: one period of the
/// tick (CMPLR + 1 cycles) long, so a span under a tick is exact.
uint32_t stk_span(uint32_t from, uint32_t to) {
    const uint32_t period = stk()->CMPLR + 1u;
    return to >= from ? to - from : to + period - from;
}

// ---------------------------------------------------------------------------
// a - the block as found, and the refusals
// ---------------------------------------------------------------------------
void ta_block() {
    print(serial, "  RCC_AHBPCENR at boot ", hex(dma_gate_at_boot), ", now ", hex(rcc()->AHBPCENR),
          "; INTFR ", hex(Dma::flags()), crlf);
    bench.verdict("DMA1EN is closed out of reset (RCC_AHBPCENR's reset value opens the USB blocks and SRAMEN)",
                  (dma_gate_at_boot & rcc_hb_dma1) == 0u);
    bench.verdict("and open now: the console's engines bound their channels", Dma::is_open());
    Copier::stop();
    bench.verdict("channel 1 is disabled as found", !Copier::enabled());
    bench.verdict("circular + memory-to-memory is refused (RM 9.2.1)",
                  !Copier::configure({.circular = true, .memory_to_memory = true}));
    bench.verdict("a zero count is refused", !Copier::set_count(0));
    bench.verdict("a transfer with a null end is refused",
                  !Copier::load(DmaTransfer{.peripheral = nullptr, .memory = dst8, .count = 4}));
    for (uint32_t i = 0; i < 64u; ++i) { src8[i] = static_cast<uint8_t>(i); }
    // A long block (64 bytes, a few hundred cycles), then a store under it.
    (void)Copier::load(DmaTransfer{.peripheral = src8, .memory = dst8, .count = 64,
                                   .config = {.memory_to_memory = true, .peripheral_increment = true}});
    const bool refused = !Copier::configure({.memory_to_memory = true}) && !Copier::set_count(8);
    (void)wait_flag(DmaFlag::complete);
    bench.verdict("a store into an enabled channel is refused (9.2.1's note: read-only then)", refused);
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
    bench.verdict("no error, and no half flag missed (HTIF up too)",
                  !Copier::flag(DmaFlag::error) && Copier::flag(DmaFlag::half));
    bench.verdict("EN STANDS after the completed block: only software clears it", Copier::enabled());
    Copier::clear(DmaFlag::all);
    bench.verdict("the write-one clear takes every flag down", Copier::flags() == 0u);

    ok = copy_block(src16, dst16, 32, DmaWidth::half);
    bench.verdict("32 half-words copied", ok && memcmp(src16, dst16, 64) == 0);
    ok = copy_block(src32, dst32, 16, DmaWidth::word);
    bench.verdict("16 words copied", ok && memcmp(src32, dst32, 64) == 0);

    // 9.3.5/9.3.6: a half-word access ignores bit 0. Programmed piecewise
    // (load() refuses the odd address): the source one byte past src16[0].
    for (uint32_t i = 0; i < 32u; ++i) { dst16[i] = 0; }
    Copier::stop();
    (void)Copier::configure({.memory_to_memory = true, .peripheral_increment = true, .memory_increment = true,
                             .peripheral_width = DmaWidth::half, .memory_width = DmaWidth::half});
    Copier::set_peripheral(reinterpret_cast<volatile uint8_t*>(src16) + 1);
    Copier::set_memory(dst16);
    (void)Copier::set_count(4);
    Copier::clear(DmaFlag::all);
    Copier::enable(true);
    (void)wait_flag(DmaFlag::complete | DmaFlag::error);
    print(serial, "  half-words from src16+1: ", hex(dst16[0]), " ", hex(dst16[1]), " (src16[0..1] ", hex(src16[0]),
          " ", hex(src16[1]), ")", crlf);
    bench.verdict("an odd address under a half-word width moves the even address's half-words (bit 0 ignored)",
                  dst16[0] == src16[0] && dst16[1] == src16[1] && dst16[3] == src16[3]);

    // The rate: 64 bytes at each width timed on the STK with every interrupt
    // held off, so the span is the block's and the poll's alone.
    uint32_t cycles[3] = {};
    uint8_t k = 0;
    for (const DmaWidth w : {DmaWidth::byte, DmaWidth::half, DmaWidth::word}) {
        const uint16_t n = w == DmaWidth::byte ? 64u : w == DmaWidth::half ? 32u : 16u;
        Copier::stop();
        (void)Copier::prepare(DmaTransfer{.peripheral = src8, .memory = dst8, .count = n,
                                          .config = {.memory_to_memory = true, .peripheral_increment = true,
                                                     .memory_increment = true, .peripheral_width = w,
                                                     .memory_width = w}});
        {
            InterruptGuard guard;
            const uint32_t t0 = stk()->CNTL;
            Copier::regs().CFGR = Copier::regs().CFGR | dma_cfgr_en;
            while ((Copier::flags() & DmaFlag::complete) == 0u) {
            }
            const uint32_t t1 = stk()->CNTL;
            cycles[k] = stk_span(t0, t1);
        }
        print(serial, "  64 bytes as ", n, " item(s): ", cycles[k], " HCLK cycles from the enable to TCIF", crlf);
        ++k;
    }
    // The slope between the widths is the controller's cycles an item.
    const uint32_t per_item_x10 = (cycles[0] - cycles[2]) * 10u / 48u;
    print(serial, "  -> ", per_item_x10 / 10u, ".", per_item_x10 % 10u,
          " cycles an item (bytes against words: 48 items apart)", crlf);
    bench.verdict("the width is the rate: words take the fewest cycles for the same bytes",
                  cycles[2] < cycles[1] && cycles[1] < cycles[0]);

    // A fixed source spread over a run: the peripheral side not incrementing.
    const uint32_t pattern = 0xDEADBEEFu;
    Copier::stop();
    (void)Copier::load(DmaTransfer{.peripheral = const_cast<uint32_t*>(&pattern), .memory = dst32, .count = 16,
                                   .config = {.memory_to_memory = true, .peripheral_increment = false,
                                              .memory_increment = true, .peripheral_width = DmaWidth::word,
                                              .memory_width = DmaWidth::word}});
    (void)wait_flag(DmaFlag::complete);
    bool filled = true;
    for (uint32_t i = 0; i < 16u; ++i) { filled = filled && dst32[i] == pattern; }
    bench.verdict("a non-incrementing source fills the run with one word", filled);
    Copier::stop();
}

// ---------------------------------------------------------------------------
// c - the vectors
// ---------------------------------------------------------------------------
template <uint8_t ch>
bool one_vector() {
    using C = DmaChannel<ch>;
    const uint32_t before = vector_hits[ch];
    C::stop();
    (void)C::prepare(DmaTransfer{.peripheral = src8, .memory = dst8, .count = 16,
                                 .config = {.memory_to_memory = true, .peripheral_increment = true}});
    C::arm(DmaFlag::complete | DmaFlag::error, true);
    Pfic::enable(C::irq());
    C::enable(true);
    (void)delay_us(clock, 200);
    Pfic::disable(C::irq());
    const bool hit = vector_hits[ch] == before + 1u && C::flags() == 0u;
    C::stop();
    return hit;
}

void tc_vectors() {
    role2 = Role::count;
    role5 = Role::count;
    const bool v1 = one_vector<1>();
    const bool v2 = one_vector<2>();
    const bool v3 = one_vector<3>();
    const bool v4 = one_vector<4>();
    const bool v5 = one_vector<5>();
    const bool v8 = one_vector<8>();
    print(serial, "  handlers: ch1 ", v1, " ch2 ", v2, " ch3 ", v3, " ch4 ", v4, " ch5 ", v5, " ch8 ", v8,
          " (6 and 7: the console's engines)", crlf);
    bench.verdict("channels 1..5 each ended a block in its own handler (entries 22..26), flags cleared there",
                  v1 && v2 && v3 && v4 && v5);
    bench.verdict("channel 8 in its own handler, at table 7-1's entry 44", v8);
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
    const bool ended = loaded && wait_flag(DmaFlag::complete | DmaFlag::error, 200'000u);
    return {ended, Copier::flags(), Copier::enabled(), static_cast<uint16_t>(Copier::count())};
}

void td_error() {
    // Holes in the map (RM 1.2's figure): between and past the buses, and
    // high above the memories. What the controller does with a read from
    // each, and with a write into each, is the finding.
    constexpr uint32_t holes[] = {0x30000000u, 0x40030000u, 0x50000000u, 0x60000000u, 0xA0000000u};
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
    // The flash, a legal destination by 9.2.1's list: ONE word of zero
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
    if ((flash_ctl()->CTLR & flash_lock) != 0u) {
        flash_ctl()->KEYR = 0x45670123u;   // KEY1, KEY2 (the FPEC keys, RM 20.3.2)
        flash_ctl()->KEYR = 0xCDEF89ABu;
    }
    const bool opened = (flash_ctl()->CTLR & flash_lock) == 0u;
    flash_ctl()->STATR = flash_wrprterr | flash_eop;   // write-1-clear
    const Probe fu = probe_block(&zero, reinterpret_cast<volatile void*>(word_addr), DmaWidth::word, 1);
    const uint32_t st_u = flash_ctl()->STATR;
    const Probe fa = probe_block(&zero, reinterpret_cast<volatile void*>(alias_addr), DmaWidth::word, 1);
    const uint32_t st_a = flash_ctl()->STATR;
    flash_ctl()->CTLR = flash_ctl()->CTLR | flash_lock;   // RW1: a one sets it
    const bool relocked = (flash_ctl()->CTLR & flash_lock) != 0u;
    const uint32_t word_end = *static_cast<const volatile uint32_t*>(&flash_zero_word);
    print(serial, "  the lock open (", opened ? "opened" : "NOT OPENED", "), no operation armed: at ", hex(word_addr),
          " flags=", hex(fu.flags), " EN=", fu.enabled, " STATR=", hex(st_u), "; at the alias ", hex(alias_addr),
          " flags=", hex(fa.flags), " EN=", fa.enabled, " STATR=", hex(st_a), "; locked again ", relocked, crlf);
    Copier::stop();
    print(serial, "  ", errors, " error(s), ", completions, " completion(s), ", hung,
          " hung, over 5 holes read and written", crlf);
    bench.verdict("every block from or into a hole ENDED (no channel left hanging)", hung == 0u);
    bench.verdict("each ended one of the two ways the chapter allows (an error, or a completion)",
                  errors + completions == 10u);
    if ((f.flags & DmaFlag::error) != 0u) {
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
    while (!Serial::tx_idle()) {
    }
    const uint32_t blocks_before = tx_blocks;
    const uint32_t t0 = Ticker::cycles();
    for (uint8_t line = 0; line < 4u; ++line) {
        print(serial, "  0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef", crlf);
    }
    while (!Serial::tx_idle()) {
    }
    const uint32_t took = Ticker::cycles() - t0;
    // 272 frames of ten bits at 115200 (BRR 417 for 416.7: 0.8 thousandths
    // slow from the divisor's rounding).
    constexpr uint32_t wire = static_cast<uint32_t>(2720ull * SysClock::hz / 115200u);
    const uint32_t permille = static_cast<uint32_t>(1000ull * took / wire);
    (void)delay_us(clock, 500);
    const uint32_t blocks = tx_blocks - blocks_before;
    const bool drained = Serial::tx_idle();
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
    bench.verdict("with no overrun and no fault", Serial::rx_overruns() == 0u && Serial::dma_faults() == 0u);
}

// ---------------------------------------------------------------------------
// g - the copy engine
// ---------------------------------------------------------------------------
bool engine_wait() {
    for (uint32_t i = 0; i < 1'000'000u; ++i) {
        if (!CopyEngine::busy()) {
            return true;
        }
    }
    return false;
}

void tg_copy_engine() {
    role5 = Role::copy_engine;
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

    uint8_t* odd = dst8 + 1;
    __asm__ volatile("" : "+r"(odd));
    bench.verdict("a half-word run off its boundary is refused (9.3.6: bit 0 would be ignored)",
                  !CopyEngine::copy(reinterpret_cast<uint16_t*>(odd), src16, 4u));
    bench.verdict("a block of no element, or of more than CNTR counts, is refused",
                  !CopyEngine::copy(dst8, src8, 0u) && !CopyEngine::copy(dst8, src8, 65536u));
    // A long block (4 KB in byte beats, some 25 000 cycles) so the second
    // start surely finds it running.
    const bool first = CopyEngine::copy(big_b, big_a, big_bytes);
    const bool second = CopyEngine::copy(dst8, src8, 64u);
    (void)engine_wait();
    bench.verdict("a block on a busy engine is refused", first && !second);
    const bool long_started = CopyEngine::fill(big_b, &big_a[0], big_bytes);
    const bool stopped = CopyEngine::abandon();
    const uint16_t left = DmaChannel<5>::count();
    const bool idle_after = !CopyEngine::busy() && !CopyEngine::abandon();
    print(serial, "  abandon(): started ", long_started, ", stopped ", stopped, ", CNTR left ", left, " of ",
          big_bytes, crlf);
    bench.verdict("abandon() stops a running block where it stands and frees the engine",
                  long_started && stopped && left > 0u && left < big_bytes && idle_after);
    bench.verdict("no transfer fault, polled", CopyEngine::faults() == 0u);

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
    role5 = Role::count;
}

// ---------------------------------------------------------------------------
// The paced load: USART3's transmitter, no pad
// ---------------------------------------------------------------------------
/// USART3 open at `baud` with DMAT: its TXE asks channel 2 for a byte every
/// frame. Its column is handed no pad. The block's reset line first.
void pacer_open(uint32_t baud) {
    U3::bus_clock(true);
    U3::reset();
    (void)U3::configure(UartFormat{}, usart_divisor(SysClock::hz, baud));
    U3::enable(true);
    U3::transmitter(true);
}

/// The release contract (docs/design/dma.md) on the pacer itself: the reset
/// line, then the gate.
void pacer_close() {
    U3::reset();
    U3::bus_clock(false);
}

// ---------------------------------------------------------------------------
// h - circular mode
// ---------------------------------------------------------------------------
void th_circular() {
    role2 = Role::circular;
    laps = 0;
    half_laps = 0;
    static const uint8_t lap4[4] = {0x11, 0x22, 0x33, 0x44};
    using C = DmaChannel<2>;
    C::stop();
    (void)C::prepare(DmaTransfer{.peripheral = &U3::regs().DATAR, .memory = const_cast<uint8_t*>(lap4), .count = 4,
                                 .config = {.direction = DmaDirection::memory_to_peripheral, .circular = true,
                                            .memory_increment = true, .priority = DmaPriority::high}});
    C::arm(DmaFlag::complete | DmaFlag::half | DmaFlag::error, true);
    Pfic::enable(C::irq());
    pacer_open(1'000'000);
    C::enable(true);
    U3::dma_transmit(true);
    // Twelve laps of four frames at 1 Mbaud: 480 us; stop after 12 and a
    // bit, the channel running all the while.
    const uint32_t t0 = Ticker::cycles();
    while (laps < 12u && Ticker::cycles() - t0 < 48'000u * 5u) {
    }
    U3::dma_transmit(false);
    const uint16_t cntr = C::count();
    const bool still_on = C::enabled();
    C::stop();
    Pfic::disable(C::irq());
    pacer_close();
    print(serial, "  12 laps of 4 at 1 Mbaud: ", laps, " completions, ", half_laps, " half marks, CNTR at the stop ",
          cntr, ", EN ", still_on, crlf);
    bench.verdict("the circular block ran lap after lap with no CPU in the path (12 completions)", laps >= 12u);
    bench.verdict("a half mark each lap beside the completion", half_laps >= 12u && half_laps <= laps + 1u);
    bench.verdict("CNTR reloaded each lap (1..4 at the stop) and EN never dropped", cntr >= 1u && cntr <= 4u && still_on);
    role2 = Role::count;
}

// ---------------------------------------------------------------------------
// i - the release contract: a held request
// ---------------------------------------------------------------------------
/// The next owner of channel 2: a transmit block of four bytes into a RAM
/// cell, request-paced (MEM2MEM clear) - nothing on this channel asks once
/// USART3 is released, so any item it moves is a stray request still
/// standing on the channel's OR. Answers how many items it moved.
volatile uint8_t sink_cell = 0;
uint16_t next_owner_moves() {
    static const uint8_t four[4] = {1, 2, 3, 4};
    using C = DmaChannel<2>;
    C::stop();
    (void)C::prepare(DmaTransfer{.peripheral = &sink_cell, .memory = const_cast<uint8_t*>(four), .count = 4,
                                 .config = {.direction = DmaDirection::memory_to_peripheral,
                                            .memory_increment = true, .priority = DmaPriority::high}});
    C::enable(true);
    (void)delay_us(clock, 300);
    const uint16_t moved = static_cast<uint16_t>(4u - C::count());
    C::stop();
    return moved;
}

/// USART3 brought up with DMAT and TXE standing: its request raised on
/// channel 2 with no channel enabled to take it.
void raise_request() {
    DmaChannel<2>::stop();
    pacer_open(115'200);
    U3::dma_transmit(true);
    (void)delay_us(clock, 50);
}

void ti_held_request() {
    // The control: the block reset and never asked.
    pacer_open(115'200);
    pacer_close();
    const uint16_t control = next_owner_moves();

    // 1. DMAT cleared, the block clocked and enabled.
    raise_request();
    U3::dma_transmit(false);
    const uint16_t dmat_off = next_owner_moves();
    pacer_close();

    // 2. UE and TE cleared with DMAT, the clock on.
    raise_request();
    U3::dma_transmit(false);
    U3::transmitter(false);
    U3::enable(false);
    const uint16_t ue_off = next_owner_moves();
    pacer_close();

    // 3. The gate closed with DMAT still set, no reset.
    raise_request();
    U3::bus_clock(false);
    const uint16_t gated = next_owner_moves();
    pacer_close();

    // 4. THE CONTRACT: the reset line pulsed, then the gate.
    raise_request();
    U3::reset();
    U3::bus_clock(false);
    const uint16_t reset = next_owner_moves();

    print(serial, "  the next owner of channel 2 moved: control ", control, ", after DMAT cleared ", dmat_off,
          ", after UE/TE/DMAT cleared ", ue_off, ", after the gate closed ", gated, ", after the reset line ",
          reset, " (of 4)", crlf);
    bench.verdict("with no request ever raised, the next owner moves nothing", control == 0u);
    bench.verdict("after the reset line, the next owner moves nothing: the release contract holds here",
                  reset == 0u);
    bench.verdict("every way of letting go ended with the channel at most one stray item down",
                  dmat_off <= 1u && ue_off <= 1u && gated <= 1u);
    if (dmat_off + ue_off + gated != 0u) {
        print(serial, "  -> A RAISED REQUEST OUTLIVES its peripheral's release on this die unless the block is "
                      "reset", crlf);
    } else {
        print(serial, "  -> this die withdraws a raised request on every release measured", crlf);
    }
    DmaChannel<2>::stop();
}

// ---------------------------------------------------------------------------
// j - the arbiter
// ---------------------------------------------------------------------------
/// Two memory-to-memory blocks of 512 words, `first` enabled then `second`
/// on the next store, interrupts held off: the first's count once both are
/// on, and when the second completes. Equal, the first moved nothing during
/// the second's block; zero, it finished before the second did.
struct Race {
    uint16_t at_start;
    uint16_t a[4];   ///< the first's count after each of four quiet windows
    uint16_t b[4];   ///< and the second's
    uint16_t left;

    /// The first moved nothing across the windows while the second moved
    /// (its item in flight when the second came lands before the first
    /// window closes).
    bool first_held() const {
        return a[0] == a[3] && at_start - a[0] <= 1 && b[0] > b[1] && b[1] > b[2] && b[2] != 0u;
    }
};

template <uint8_t first_ch, uint8_t second_ch>
Race race(DmaPriority first_level, DmaPriority second_level) {
    using A = DmaChannel<first_ch>;
    using B = DmaChannel<second_ch>;
    constexpr uint16_t words = 512;
    A::stop();
    B::stop();
    (void)A::prepare(DmaTransfer{.peripheral = big_a, .memory = big_b, .count = words,
                                 .config = {.memory_to_memory = true, .peripheral_increment = true,
                                            .peripheral_width = DmaWidth::word, .memory_width = DmaWidth::word,
                                            .priority = first_level}});
    (void)B::prepare(DmaTransfer{.peripheral = big_a + 2048, .memory = big_b + 2048, .count = words,
                                 .config = {.memory_to_memory = true, .peripheral_increment = true,
                                            .peripheral_width = DmaWidth::word, .memory_width = DmaWidth::word,
                                            .priority = second_level}});
    Race r{};
    {
        InterruptGuard guard;
        A::regs().CFGR = A::regs().CFGR | dma_cfgr_en;
        B::regs().CFGR = B::regs().CFGR | dma_cfgr_en;
        r.at_start = A::count();
        // A window with NO access to the controller (a loop of the core's
        // own), then both counts: what the arbiter did undisturbed by a
        // poll of its register port.
        for (uint8_t k = 0; k < 4u; ++k) {
            for (uint32_t i = 0; i < 40u; ++i) {
                __asm__ volatile("nop");
            }
            r.a[k] = static_cast<uint16_t>(A::count());
            r.b[k] = static_cast<uint16_t>(B::count());
        }
        while ((B::flags() & DmaFlag::complete) == 0u) {
        }
        r.left = A::count();
        while ((A::flags() & DmaFlag::complete) == 0u) {
        }
    }
    print(serial, "    ch", first_ch, " (PL ", static_cast<uint8_t>(first_level), ") first, ch", second_ch, " (PL ",
          static_cast<uint8_t>(second_level), ") next: ch", first_ch, " had ", r.at_start,
          " of 512 left once both were on", crlf, "      after four quiet windows ch", first_ch, " ", r.a[0], " ",
          r.a[1], " ", r.a[2], " ", r.a[3], ", ch", second_ch, " ", r.b[0], " ", r.b[1], " ", r.b[2], " ", r.b[3],
          "; ch", first_ch, " ", r.left, " when ch", second_ch, "'s completion was seen (INTFR polled)", crlf);
    A::stop();
    B::stop();
    return r;
}

void tj_arbiter() {
    // A higher level on the HIGHER channel number: the level decides.
    const Race lv = race<1, 8>(DmaPriority::low, DmaPriority::very_high);
    // Equal levels, the higher number first: the lower number takes over.
    const Race tie = race<8, 1>(DmaPriority::medium, DmaPriority::medium);
    // Equal levels, the lower number first: the higher number waits.
    const Race low_first = race<1, 8>(DmaPriority::medium, DmaPriority::medium);
    bench.verdict("a higher level wins over a lower channel number: in a quiet window the low channel 1 moved "
                  "nothing while channel 8's very_high block ran",
                  lv.first_held());
    bench.verdict("at one level the lower number wins, mid-block: channel 8, first, moved nothing in the window "
                  "while channel 1's block ran",
                  tie.first_held());
    bench.verdict("and channel 1, first at one level, keeps the bus: channel 8 moved nothing in the windows",
                  low_first.b[3] == 512u && low_first.a[0] > low_first.a[1] && low_first.left == 0u);
}

// ---------------------------------------------------------------------------
// k - the core's Sleep
// ---------------------------------------------------------------------------
/// One plain idle() with interrupts masked as the kernel calls it: back at
/// the next interrupt (the tick at the latest) - or at once, while a DMA
/// channel works (the platform's guard).
void one_idle() {
    P::CriticalSection cs;
    P::idle();
}

/// THE PLATFORM'S SLEEP WITHOUT ITS GUARD: the same WFE sequence
/// ch32x035/platform.hpp's idle() runs (the store, the unmask, the wfi, the
/// latch consumed), entered whatever the controller is doing - the one way
/// this suite can watch a channel across a sleep the platform refuses.
void raw_sleep() {
    InterruptGuard masked;
    const uint32_t wfe = (pfic_sctlr() | sctlr_wfitowfe | sctlr_sevonpend) & ~(sctlr_setevent | sctlr_sleeponexit);
    pfic_sctlr() = wfe;
    enable_interrupts();
    __asm__ volatile("wfi" ::: "memory");
    pfic_sctlr() = wfe | sctlr_setevent;
    __asm__ volatile("wfi" ::: "memory");
}

/// 256 bytes paced by USART3's transmitter at 1 Mbaud (2.56 ms), the core
/// waiting for the completion by `wait`; the wall in thousandths of the
/// pace.
template <typename Wait>
uint32_t paced_permille(Wait wait) {
    role2 = Role::paced;
    Paced::arm(&U3::regs().DATAR);
    pacer_open(1'000'000);
    U3::dma_transmit(true);
    paced_done = false;
    const uint32_t p0 = Ticker::cycles();
    (void)Paced::start(std::span<const uint8_t>(big_a, 256));
    while (!paced_done) {
        wait();
    }
    const uint32_t paced_wall = Ticker::cycles() - p0;
    U3::dma_transmit(false);
    while (!U3::tx_complete()) {
    }
    pacer_close();
    Paced::stop();
    role2 = Role::count;
    return static_cast<uint32_t>(1000ull * paced_wall / (2560ull * SysClock::hz / 1'000'000u));
}

void tk_sleep() {
    // THE SILICON: a copy of 4096 bytes in byte beats (some 25 000 cycles at
    // six an item) started right after a tick, with NO interrupt of its
    // own, the core put to sleep without the platform's guard until the
    // next interrupt; the count read at the wake.
    CopyEngine::arm(DmaPriority::low, false);
    role5 = Role::copy_engine;
    raw_sleep();                                 // land just after a tick
    (void)CopyEngine::copy(big_b, big_a, big_bytes);
    const uint32_t t0 = stk()->CNTL;
    raw_sleep();
    const uint32_t t1 = stk()->CNTL;
    const uint16_t left = DmaChannel<5>::count();
    (void)engine_wait();
    print(serial, "  4096-byte copy, the core asleep (no guard) ", stk_span(t0, t1), " cycles: ", left,
          " items left at the wake", crlf);
    bench.verdict("IN SLEEP THE CONTROLLER MOVES NOTHING: the copy stood still while the core slept (this die)",
                  left > 3000u);
    const uint32_t raw_paced = paced_permille([] { raw_sleep(); });
    print(serial, "  256 bytes paced at 1 Mbaud, the core sleeping between interrupts (no guard): ", raw_paced,
          " thousandths of the pace", crlf);
    bench.verdict("and a request-paced block crawls: it moves only while the core is awake", raw_paced > 2000u);

    // THE PLATFORM: idle() asks the controller first and does not sleep
    // over a working channel - the same copy idles to its own interrupt.
    CopyEngine::arm(DmaPriority::low, true);
    copy_done = false;
    raw_sleep();
    const uint32_t c0 = Ticker::cycles();
    (void)CopyEngine::copy(big_b, big_a, big_bytes);
    bool slept_through = false;
    for (;;) {
        P::CriticalSection cs;
        if (copy_done) {
            break;
        }
        slept_through = slept_through || !Dma::any_working();
        P::idle();
    }
    const uint32_t wall = Ticker::cycles() - c0;
    print(serial, "  the same copy idling through the platform to its own interrupt: ", wall, " cycles", crlf);
    bench.verdict("the platform's idle() does not sleep over it: ended within a tick, at the controller's rate",
                  wall < 40'000u && memcmp(big_a, big_b, big_bytes) == 0);
    const uint32_t guarded_paced = paced_permille([] { one_idle(); });
    print(serial, "  256 bytes paced at 1 Mbaud, idling through the platform: ", guarded_paced,
          " thousandths of the pace", crlf);
    bench.verdict("a paced block idled through the platform keeps its pace (within 2 per cent)",
                  guarded_paced >= 990u && guarded_paced <= 1020u);
    (void)slept_through;
    // This suite's console receives through a CIRCULAR channel, which is
    // always working: the program holds itself awake (the platform's file
    // header), and idle() returns at once on every turn.
    const uint32_t i0 = Ticker::ticks();
    uint32_t turns = 0;
    while (Ticker::ticks() - i0 < 20u) {
        one_idle();
        ++turns;
    }
    print(serial, "  the console's receive ring at work: 20 ticks of idle() took ", turns, " turns", crlf);
    bench.verdict("a receive ring holds the program awake: idle() returns at once, many turns a tick",
                  Dma::any_working() && turns > 200u);
    role5 = Role::count;
}

// ---------------------------------------------------------------------------
// m - the benchmark row
// ---------------------------------------------------------------------------
BenchCounters counters() { return bench_counters<Plat>(tick_meter, usart_meter, dma_meter); }

template <typename Launch>
BenchSample dma_sample(volatile bool& done, Launch launch) {
    done = false;
    const BenchCounters before = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    launch();
    for (;;) {
        Plat::CriticalSection cs;
        if (done) {
            break;
        }
        Plat::idle();
    }
    const uint32_t wall = sw.elapsed();
    return bench_sample(wall, before, counters());
}

template <typename Launch>
BenchSample dma_best(uint8_t runs, volatile bool& done, Launch launch) {
    BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < runs; ++run) {
        const BenchSample s = dma_sample(done, launch);
        if (s.wall < best.wall) {
            best = s;
        }
    }
    return best;
}

enum class BlockOp : uint8_t { copy, copy_flash, fill };

template <BlockOp op>
BenchSample block_run(uint32_t n) {
    const uint32_t words = n / 4u;
    // The cell IN SRAM: a const one would sit in the flash, and the fill
    // would be copy.flash's rate.
    static uint32_t fill_cell = 0xA5A5'5A5Au;
    return dma_best(8, copy_done, [words] {
        if constexpr (op == BlockOp::copy) {
            (void)CopyEngine::copy(reinterpret_cast<uint32_t*>(big_b), reinterpret_cast<const uint32_t*>(big_a), words);
        } else if constexpr (op == BlockOp::copy_flash) {
            (void)CopyEngine::copy(reinterpret_cast<uint32_t*>(big_b),
                                   reinterpret_cast<const uint32_t*>(flash_array_base), words);
        } else {
            (void)CopyEngine::fill(reinterpret_cast<uint32_t*>(big_b), &fill_cell, words);
        }
    });
}

template <BlockOp op>
void block_lines(const char* name) {
    constexpr uint32_t wire = 4u * Ruler::hz();   // one word item a cycle
    const BenchSample s16 = block_run<op>(16);
    const BenchSample s256 = block_run<op>(256);
    const BenchSample sbig = block_run<op>(big_bytes);
    bench_line(serial, name, 16, s16, Ruler::hz(), wire);
    bench_line(serial, name, 256, s256, Ruler::hz(), wire);
    bench_line(serial, name, big_bytes, sbig, Ruler::hz(), wire);
    const uint32_t tenths = (sbig.wall - s256.wall) * 10u / ((big_bytes - 256u) / 4u);
    const uint32_t share = tenths * 4u / 10u;
    const uint32_t fixed = s16.wall > share ? s16.wall - share : 0u;
    print(serial, "  ", name, ": ", tenths / 10u, '.', tenths % 10u, " cycles an item of 4 bytes, fixed ", fixed,
          " cycles a block (the instrument inside)", crlf);
    console_drain();
}

void tm_bench() {
    for (uint32_t i = 0; i < big_bytes; ++i) {
        big_a[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    role5 = Role::copy_engine;
    CopyEngine::arm(DmaPriority::low, true);
    console_drain();
    print(serial, "  every line to the completion interrupt, the core idling through the platform; wire = one "
                  "word item a cycle", crlf);
    console_drain();
    block_lines<BlockOp::copy>("copy");
    block_lines<BlockOp::copy_flash>("copy.flash");
    block_lines<BlockOp::fill>("fill");
    print(serial, "  copy faults: ", CopyEngine::faults(), crlf);

    // Paced: 1024 bytes into USART3's DATAR at 1 Mbaud - one byte a frame of
    // ten bits, 100 000 B/s.
    role2 = Role::paced;
    Paced::arm(&U3::regs().DATAR);
    pacer_open(1'000'000);
    U3::dma_transmit(true);
    console_drain();
    const BenchSample s = dma_sample(paced_done, [] { (void)Paced::start(std::span<const uint8_t>(big_a, 1024)); });
    U3::dma_transmit(false);
    while (!U3::tx_complete()) {
    }
    pacer_close();
    Paced::stop();
    bench_line(serial, "paced", 1024, s, Ruler::hz(), 100'000);
    role2 = Role::count;
    role5 = Role::count;
    bench.verdict("ran", true);
}

void banner() {
    print(serial, crlf, "test_x035_dma - ", device::part_name, " (console on DMA channels 7 and 6)", crlf);
    bench.menu();
}

} // namespace

BRIO_CH32_VECTOR(systick_handler) {
    tick_meter.enter();
    brio::Ticker::tick();
    tick_meter.leave();
}
BRIO_CH32_VECTOR(usart2_handler) {
    usart_meter.enter();
    (void)Serial::isr();
    usart_meter.leave();
}
/// Channel 7 is the transmit engine's alone: every entry is a block's
/// completion (dma_isr()'s answer is the RECEIVE edge, none of its).
BRIO_CH32_VECTOR(dma1_channel7_handler) {
    tx_blocks = tx_blocks + 1u;
    usart_meter.enter();
    (void)Serial::dma_isr();
    usart_meter.leave();
}
BRIO_CH32_VECTOR(dma1_channel6_handler) {
    usart_meter.enter();
    (void)Serial::dma_isr();
    usart_meter.leave();
}
BRIO_CH32_VECTOR(dma1_channel1_handler) {
    if (brio::DmaChannel<1>::isr() != 0u) { vector_hits[1] = vector_hits[1] + 1u; }
}
BRIO_CH32_VECTOR(dma1_channel2_handler) {
    dma_meter.enter();
    switch (role2) {
        case Role::paced:
            if ((Paced::service() & Paced::flag_complete) != 0u) {
                (void)Paced::complete();
                paced_done = true;
            }
            break;
        case Role::circular: {
            const uint32_t f = brio::DmaChannel<2>::isr();
            if ((f & brio::DmaFlag::half) != 0u) { half_laps = half_laps + 1u; }
            if ((f & brio::DmaFlag::complete) != 0u) { laps = laps + 1u; }
            break;
        }
        default:
            if (brio::DmaChannel<2>::isr() != 0u) { vector_hits[2] = vector_hits[2] + 1u; }
            break;
    }
    dma_meter.leave();
}
BRIO_CH32_VECTOR(dma1_channel3_handler) {
    if (brio::DmaChannel<3>::isr() != 0u) { vector_hits[3] = vector_hits[3] + 1u; }
}
BRIO_CH32_VECTOR(dma1_channel4_handler) {
    if (brio::DmaChannel<4>::isr() != 0u) { vector_hits[4] = vector_hits[4] + 1u; }
}
BRIO_CH32_VECTOR(dma1_channel5_handler) {
    dma_meter.enter();
    if (role5 == Role::copy_engine) {
        if (CopyEngine::isr() != 0u) {
            engine_completions = engine_completions + 1u;
            copy_done = true;
        }
    } else if (brio::DmaChannel<5>::isr() != 0u) {
        vector_hits[5] = vector_hits[5] + 1u;
    }
    dma_meter.leave();
}
BRIO_CH32_VECTOR(dma1_channel8_handler) {
    if (brio::DmaChannel<8>::isr() != 0u) { vector_hits[8] = vector_hits[8] + 1u; }
}

namespace {

// ---------------------------------------------------------------------------
// n - unequal widths (table 9-1)
// ---------------------------------------------------------------------------
/// The width in bytes.
constexpr uint8_t bytes_of(DmaWidth w) {
    return w == DmaWidth::byte ? 1u : w == DmaWidth::half ? 2u : 4u;
}

void tn_widths() {
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
            const bool ended = loaded && wait_flag(DmaFlag::complete | DmaFlag::error);
            // Table 9-1: item i is read at i x the source's width and
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
    bench.verdict("the nine pairs of source and target widths land as table 9-1 spells them: a narrow "
                  "source zero-filled, a wide one truncated to its low bytes, nothing past the fourth item",
                  exact == 9u);
}

} // namespace

int main() {
    dma_gate_at_boot = brio::rcc()->AHBPCENR;
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the block as found and the refusals", ta_block);
    bench.letter('b', "memory to memory at three widths, EN standing, the odd address", tb_copies);
    bench.letter('c', "the eight vectors", tc_vectors);
    bench.letter('d', "a transfer error", td_error);
    bench.letter('e', "the console's transmit engine", te_tx_engine);
    bench.letter('f', "the console's receive engine (asks for a line)", tf_rx_engine, false);
    bench.letter('g', "the copy engine: three beats, copy and fill, polled and by interrupt, refusals",
                 tg_copy_engine);
    bench.letter('h', "circular mode, paced by USART3's transmitter", th_circular);
    bench.letter('i', "the release contract: a held request", ti_held_request);
    bench.letter('j', "the arbiter: levels and the channel number", tj_arbiter);
    bench.letter('k', "the core's Sleep over a working channel", tk_sleep);
    bench.letter('m', "the benchmark row: copy, fill, paced", tm_bench);
    bench.letter('n', "unequal widths: the nine pairs of table 9-1", tn_widths);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=",
                    tick_ok ? "STK" : "FAILED", brio::crlf);
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
