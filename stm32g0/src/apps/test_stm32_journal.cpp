// test_stm32_journal - util/nv_journal.hpp in the STM32G0's storage attic
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console; `z` runs them all and prints the "ALL: N pass, M fail"
// line tools/bench.py judges. Board E (STM32G0B1RE), no wires.
//
// WHAT IT IS ABOUT. stm32g0/nvm_flash.hpp gives the whole of physical
// BANK 2 to storage - the linker script hands the compiler bank 1 alone -
// and partitions it into two classes: pages 0..125 are the block heap's
// (util/nv_heap.hpp, exercised by test_stm32_nvm) and the top two pages
// are the value journal's, two 2 Kbyte halves that ping-pong. This suite
// runs the journal on the real silicon and, in letter e, runs BOTH at
// once - which is the partition's whole point and the thing a host test
// cannot say anything about.
//
// AND IT IS THE JOURNAL'S THIRD SILICON. util/nv_journal.hpp was written
// on the samc's RWWEE array (256-byte erase unit, 64-byte cell) and
// host-swept over three geometries, one of which - 2048/8 - was chosen
// because it is THIS part's. So the first thing letter a checks is that
// the geometry the host suite has been sweeping all along is the one the
// silicon really has.
//
// What is exercised, letter by letter:
//
//   a  the partition and the mount: the constants, the region, and a
//      read-only boot that costs no cycle.
//   b  the round trip: the byte core, the typed twin, latest-wins, and
//      the three refusals (oversize, one id too many, a short reader).
//   c  the halves ping-pong on silicon: churn until a collection
//      happens, watch the active half change, and prove every value
//      came through it and through a fresh mount afterwards.
//   d  what it costs, and THE NO-STALL CLAIM: the turns of a bank-1
//      polling loop the CPU completes inside a bank-2 page erase, which
//      is what makes an ordinary save() legal from the main loop here.
//   e  COEXISTENCE: a heap in the lower pages and the journal in the
//      attic, both mounted, the heap's block byte-exact while the
//      journal churns over it.
//   f  the panic reserve: after every completed save there is room for
//      one more maximum-size entry, and taking it costs NO erase.
//   w  the wear this run spent, in page erases, against a declared
//      budget. It PRINTS the numbers and then zeroes the counters, so
//      each z run reports its own budget and the letter is re-runnable.
//
//   p  the panic letter (by name only: it REBOOTS the board). A real
//      panic() whose reporter writes the breadcrumb into the journal
//      through the reserve, a reset, and take() at the next boot.
//   v  verify the survivors (by name only): the values letter b wrote,
//      after whatever happened in between - the letter to run after
//      reflashing another app and coming back.
//
// The suite is re-runnable in one power-on: nothing here is one-way.
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <array>
#include <optional>
#include <span>

#include "kernel/panic.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/flash.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/nvm_flash.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/reset.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/nv_heap.hpp"
#include "util/nv_journal.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter p lives in
//
// INLINE, and in .noinit, for the two reasons test_stm32_platform gives:
// the section must survive the crt, and gcc gives an inline variable with
// a section attribute the COMDAT group a plain one does not get. Its
// magic word is not decoration - RM0444 promises nothing about SRAM
// across a reset.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x4A21;
inline constexpr uint16_t token_canary = 0xC3A5;

struct Token {
    uint16_t magic;
    uint16_t canary;
    uint8_t leg;       ///< 1 = a panic was staged and is awaited
    uint8_t code;      ///< the PanicCode panic() was given
    uint8_t context;   ///< its context byte
    uint16_t pass;
    uint16_t fail;
    uint32_t free_before;   ///< cells free in the active half before the panic
    uint8_t via_fault;      ///< 1 = the HardFault body wrote the journal record
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

using Led = Pin<'A', 5>;   // LD4

TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// The media, with a meter on it
//
// The journal runs over the real MainFlashJournalZone; this wrapper only
// counts what passes through, because "the wear budget of a run" is a
// number the suite has to be able to PRINT rather than estimate. The
// static_asserts below are what keep it honest: every constant is the
// real medium's, so nothing about the journal's geometry is different
// for being metered.
// ---------------------------------------------------------------------------
struct MeteredZone {
    MeteredZone() = delete;

    static constexpr uint32_t erase_size = MainFlashJournalZone::erase_size;
    static constexpr uint32_t write_cell = MainFlashJournalZone::write_cell;
    static constexpr uint32_t flash_end = MainFlashJournalZone::flash_end;
    static constexpr uint8_t zone_count = MainFlashJournalZone::zone_count;

    static std::array<FlashZone, zone_count> zones() {
        return MainFlashJournalZone::zones();
    }
    static void read(uint32_t addr, std::span<uint8_t> dst) {
        ++reads;
        MainFlashJournalZone::read(addr, dst);
    }
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        ++programs;
        return MainFlashJournalZone::program(addr, src);
    }
    static bool erase(uint32_t addr) {
        ++erases;
        return MainFlashJournalZone::erase(addr);
    }
    static uint32_t build_id() { return MainFlashJournalZone::build_id(); }

    static void clear() { reads = programs = erases = 0; }

    static inline uint32_t reads = 0;
    static inline uint32_t programs = 0;
    static inline uint32_t erases = 0;
};

static_assert(FlashMedia<MeteredZone>);
static_assert(MeteredZone::erase_size == 2048u);
static_assert(MeteredZone::write_cell == 8u);

/// Six 32-byte values in two 2 Kbyte halves. The journal's own
/// static_assert then performs (6 + 2) x 6 cells <= 256 cells, which is
/// this geometry's whole point: one PAGE per half is already generous
/// where the samc needed two rows for the same six values.
using Journal = NvJournal<MeteredZone, 6, 32, 1>;
Journal journal;

/// The heap letter e runs beside it, in the OTHER partition. It is the
/// real MainFlash, unmetered: this suite only reads what it wrote, and
/// test_stm32_nvm is where the heap is under test.
using Heap = NvHeap<MainFlash, 8, 2>;
Heap heap;
constexpr uint16_t heap_record = 0x4A02;

/// The id map. Six is max_ids, so this is all of them.
constexpr uint8_t id_bytes = 0;    ///< a plain byte string
constexpr uint8_t id_cal = 1;      ///< the typed value
constexpr uint8_t id_short = 2;    ///< a one-byte value
constexpr uint8_t id_full = 3;     ///< a maximum-size value
constexpr uint8_t id_churn = 4;    ///< what letters c, e and f rewrite
constexpr uint8_t id_panic = 5;    ///< the breadcrumb (letter p)

/// The typed value letter b stores and letter v checks.
struct Calibration {
    int32_t offset;
    uint16_t gain;
    uint8_t mode;
};
constexpr Calibration calibration{-31415, 4001, 0x5A};

/// The panic reporter: JournalPanic over this journal, at id_panic.
using Panic = JournalPanic<journal, id_panic>;

// ---------------------------------------------------------------------------
// A cycle-resolution stopwatch (the one test_stm32_platform uses)
// ---------------------------------------------------------------------------
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

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000UL;
uint32_t cycles_to_us(uint32_t c) { return c / cycles_per_us; }

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < SysClock::hz / 500u) {
    }
}

// ---------------------------------------------------------------------------
// Payload helpers
//
// VOLATILE for the reason every storage suite in this tree pays: they are
// compared against flash the CPU itself programmed through a side channel
// the optimizer cannot see, and gcc folds such a read-back into whatever
// was last stored.
// ---------------------------------------------------------------------------
volatile uint8_t out_buf[40];
volatile uint8_t in_buf[40];

uint8_t pattern(uint32_t i, uint8_t seed) {
    return static_cast<uint8_t>(seed ^ (i * 7u + 0x5Bu));
}

void fill(uint8_t len, uint8_t seed) {
    for (uint8_t i = 0; i < len; ++i) {
        out_buf[i] = pattern(i, seed);
    }
}

std::span<const uint8_t> out_span(uint8_t len) {
    return std::span<const uint8_t>(const_cast<const uint8_t*>(out_buf), len);
}
std::span<uint8_t> in_span(uint8_t len) {
    return std::span<uint8_t>(const_cast<uint8_t*>(in_buf), len);
}

/// Save a byte pattern under `id` and say whether it landed.
bool put(uint8_t id, uint8_t len, uint8_t seed) {
    fill(len, seed);
    return journal.save(id, out_span(len));
}

/// Read it back and compare, from flash, through the journal's index.
bool holds(uint8_t id, uint8_t len, uint8_t seed) {
    const std::optional<uint8_t> n = journal.load(id, in_span(32));
    if (!n || *n != len) {
        return false;
    }
    for (uint8_t i = 0; i < len; ++i) {
        if (in_buf[i] != pattern(i, seed)) {
            return false;
        }
    }
    return true;
}

const char* error_name(NvJournalError e) {
    switch (e) {
    case NvJournalError::none: return "none";
    case NvJournalError::not_mounted: return "not_mounted";
    case NvJournalError::too_big: return "too_big";
    case NvJournalError::too_many_ids: return "too_many_ids";
    case NvJournalError::no_room: return "no_room";
    case NvJournalError::media: return "media";
    case NvJournalError::gc_failed: return "gc_failed";
    }
    return "?";
}

void print_mount(const Journal::MountReport& r) {
    print(serial, "  mount: status=", static_cast<uint8_t>(r.status),
          " live=", r.live, " torn=", r.torn, " active=", r.active,
          " pending=", r.collect_pending ? 1u : 0u, " seq=", r.seq,
          " used=", r.used_cells, "/", Journal::half_cells, crlf);
}

/// Every letter starts from a journal that is mounted from FLASH, never
/// from what a previous letter left in RAM: a suite that trusts its own
/// memory proves nothing about persistence.
const Journal::MountReport& remount() { return journal.mount(); }

// =============================================================================
// a - the partition, and a read-only mount
// =============================================================================
void ta_partition() {
    const uint32_t abs_journal =
        MainFlashPartition::address_of(MainFlashPartition::journal_base);
    print(serial, "  bank 2 ",
          hex(MainFlashPartition::address_of(MainFlashPartition::storage_base)),
          " .. ",
          hex(MainFlashPartition::address_of(MainFlashPartition::storage_end)),
          "   heap up to ",
          hex(MainFlashPartition::address_of(MainFlash::flash_end)),
          "   journal ", hex(abs_journal), " .. ",
          hex(MainFlashPartition::address_of(MainFlashPartition::storage_end)),
          crlf);
    print(serial, "  halves ",
          hex(MainFlashPartition::address_of(Journal::half_base(0))), " and ",
          hex(MainFlashPartition::address_of(Journal::half_base(1))), ", ",
          Journal::half_bytes, " bytes = ", Journal::half_cells,
          " cells each; one entry is at most ", Journal::max_entry_bytes,
          " bytes (", Journal::max_entry_cells, " cells), reserve ",
          Journal::reserve_cells, " cells", crlf);

    // THE GEOMETRY THE HOST SUITE HAS BEEN SWEEPING. docs/design/
    // nv-journal.md names 2048/8 as a geometry test/test_nv_journal
    // already covers "because it is the STM32G0's" - this is the line
    // that turns that sentence from a prediction into a reading.
    bench.verdict("the erase unit is a 2048-byte PAGE and the program unit an "
                  "8-byte DOUBLE WORD - the widest split of the three targets",
                  Journal::erase_size == 2048u && Journal::write_cell == 8u);
    bench.verdict("the journal owns the top two pages of the bank",
                  MainFlashPartition::journal_bytes == 2u * Flash::page_size &&
                      MainFlashPartition::journal_base ==
                          MainFlashPartition::storage_end -
                              2u * Flash::page_size);
    bench.verdict("a half is ONE page, which is the smallest a half may be: "
                  "erasing it must not take the other half down",
                  Journal::half_bytes == Flash::page_size &&
                      Journal::half_cells == 256u);
    bench.verdict("the two regions do not overlap, the heap's map home "
                  "included",
                  Heap::map_home + 2u * Heap::erase_size <=
                      MainFlashPartition::journal_base);
    bench.verdict("the journal's halves fill the attic exactly",
                  Journal::journal_home == MainFlashPartition::journal_base &&
                      Journal::half_base(1) + Journal::half_bytes ==
                          MainFlashPartition::storage_end);
    bench.verdict("and the reserve is one maximum-size entry",
                  Journal::reserve_cells == Journal::max_entry_cells &&
                      Journal::max_entry_cells == 6u);

    // A mount is READ-ONLY: this is the boot path and it must cost no
    // cycle of endurance at all.
    const uint32_t erases0 = MeteredZone::erases;
    const uint32_t programs0 = MeteredZone::programs;
    const uint32_t t0 = cycles_now();
    const Journal::MountReport& r = remount();
    const uint32_t mount_us = cycles_to_us(cycles_now() - t0);
    print_mount(r);
    print(serial, "  the mount took ", mount_us, " us and read ",
          MeteredZone::reads, " time(s) in this letter", crlf);

    bench.verdict("the journal mounts", r.mounted());
    bench.verdict("A MOUNT IS READ-ONLY: no erase and no program at boot",
                  MeteredZone::erases == erases0 &&
                      MeteredZone::programs == programs0);
    bench.verdict("and it is fast enough to be on the boot path",
                  mount_us < 20'000u);
}

// =============================================================================
// b - the round trip
// =============================================================================
void tb_round_trip() {
    (void)remount();

    bench.verdict("a byte string saves", put(id_bytes, 20, 0x31));
    bench.verdict("and loads back byte for byte", holds(id_bytes, 20, 0x31));

    bench.verdict("the typed twin saves", journal.save(id_cal, calibration));
    const std::optional<Calibration> back = journal.load<Calibration>(id_cal);
    bench.verdict("and comes back field for field",
                  back && back->offset == calibration.offset &&
                      back->gain == calibration.gain &&
                      back->mode == calibration.mode);

    // LATEST-SEQ-WINS, and the older entry is still physically there:
    // that is the whole atomicity argument, so it is worth writing three
    // versions and reading the third.
    bench.verdict("a one-byte value saves", put(id_short, 1, 0x40));
    bench.verdict("rewriting it twice more leaves the LAST one in force",
                  put(id_short, 1, 0x41) && put(id_short, 1, 0x42) &&
                      holds(id_short, 1, 0x42));
    bench.verdict("a fresh mount agrees - the winner is decided from FLASH "
                  "and not from the RAM index",
                  remount().mounted() && holds(id_short, 1, 0x42));

    bench.verdict("a maximum-size value saves", put(id_full, 32, 0x53));
    bench.verdict("and loads whole", holds(id_full, 32, 0x53));

    // The three refusals, each with its own error code.
    fill(33, 0x60);
    bench.verdict("a payload one byte too big is refused",
                  !journal.save(id_bytes, out_span(33)) &&
                      journal.last_error() == NvJournalError::too_big);

    // max_ids is six and five are live (bytes, cal, short, full, and
    // whatever letter c or p left); the sixth is the panic id.
    (void)put(id_churn, 4, 0x70);
    (void)journal.save<PanicRecord>(id_panic, PanicRecord{0, 0, 0});
    fill(4, 0x71);
    const bool refused_id = !journal.save(0x20u, out_span(4)) &&
                            journal.last_error() == NvJournalError::too_many_ids;
    print(serial, "  live ids ", journal.count(), "/6, a seventh -> ",
          error_name(journal.last_error()), crlf);
    bench.verdict("a SEVENTH id is refused, and the refusal says which rule "
                  "it broke", refused_id);

    bench.verdict("a reader too small for the value gets NOTHING rather than "
                  "a short read", !journal.load(id_full, in_span(31)).has_value());
    bench.verdict("and an id that was never stored is simply absent",
                  !journal.has(0x30u) && !journal.load(0x30u, in_span(32)));
}

// =============================================================================
// c - the halves ping-pong
// =============================================================================
void tc_halves() {
    const Journal::MountReport& r0 = remount();
    const uint8_t half0 = r0.active;
    const uint32_t erases0 = MeteredZone::erases;
    print_mount(r0);

    // Churn one id until the half fills and a collection moves everything
    // to the other one. A 16-byte value is 4 cells, so a 256-cell half
    // takes about sixty of them - bounded well above that so the letter
    // fails loudly rather than looping.
    uint16_t saves = 0;
    uint8_t half1 = half0;
    bool all_saved = true;
    for (uint16_t i = 0; i < 400u && half1 == half0; ++i) {
        all_saved = all_saved && put(id_churn, 16, static_cast<uint8_t>(i));
        ++saves;
        half1 = journal.active_half();
    }
    print(serial, "  ", saves, " saves moved the active half ", half0, " -> ",
          half1, ", ", MeteredZone::erases - erases0, " page erase(s), seq now ",
          journal.sequence(), crlf);

    bench.verdict("every save landed", all_saved);
    bench.verdict("the halves really do ping-pong on silicon", half1 != half0);
    bench.verdict("a collection costs the erase of BOTH halves' pages - the "
                  "destination before the copy and the source after it",
                  MeteredZone::erases - erases0 == 2u);
    bench.verdict("the churned value is the last one written",
                  holds(id_churn, 16, static_cast<uint8_t>(saves - 1u)));

    // EVERY OTHER ID CAME THROUGH THE COLLECTION. That is what a
    // collection is: the latest value of every live id copied into the
    // fresh half.
    bench.verdict("the byte string survived the collection",
                  holds(id_bytes, 20, 0x31));
    bench.verdict("so did the typed value", [] {
        const std::optional<Calibration> c = journal.load<Calibration>(id_cal);
        return c && c->offset == calibration.offset && c->gain == calibration.gain;
    }());
    bench.verdict("and the maximum-size one", holds(id_full, 32, 0x53));

    // And from FLASH, not from the index the collection built in RAM.
    const Journal::MountReport& r1 = remount();
    print_mount(r1);
    bench.verdict("a fresh mount finds the new half active and nothing torn",
                  r1.mounted() && r1.active == half1 && r1.torn == 0u);
    bench.verdict("with no collection left pending", !r1.collect_pending);
    bench.verdict("and every value still readable through it",
                  holds(id_bytes, 20, 0x31) && holds(id_full, 32, 0x53) &&
                      holds(id_churn, 16, static_cast<uint8_t>(saves - 1u)));
}

// =============================================================================
// d - what it costs, and the no-stall claim
// =============================================================================
void td_cost() {
    (void)remount();

    // One ordinary save that does NOT collect: one program of a few
    // cells and nothing else.
    const uint32_t erases0 = MeteredZone::erases;
    fill(16, 0x88);
    const uint32_t t0 = cycles_now();
    const bool ok = journal.save(id_churn, out_span(16));
    const uint32_t save_us = cycles_to_us(cycles_now() - t0);
    const uint32_t save_erases = MeteredZone::erases - erases0;

    print(serial, "  a save of 16 bytes: ", save_us, " us, ", save_erases,
          " erase(s) - the entry is ",
          (Journal::header_bytes + 16u + Journal::write_cell - 1u) /
              Journal::write_cell,
          " double words of a 2048-byte page", crlf);
    bench.verdict("an ordinary save lands", ok);
    bench.verdict("and costs NO erase when the half has room - which is the "
                  "whole reason a journal is not a record rewritten in place",
                  save_erases == 0u);
    bench.verdict("its cost is the double words it programs plus the CRC over "
                  "them (~88 us a double word here)",
                  save_us >= 100u && save_us <= 3'000u);

    // A collection, timed: two page erases and one program per live id.
    const uint32_t before = journal.free_cells();
    const uint32_t t1 = cycles_now();
    const bool collected = journal.collect();
    const uint32_t collect_us = cycles_to_us(cycles_now() - t1);
    print(serial, "  a collection: ", collect_us, " us for ", journal.count(),
          " live id(s), free cells ", before, " -> ", journal.free_cells(),
          crlf);
    bench.verdict("a collection succeeds", collected);
    bench.verdict("and costs about two page erases plus one program per live "
                  "id (2 x 22 ms is the floor)",
                  collect_us >= 40'000u && collect_us <= 90'000u);
    bench.verdict("it hands the half back nearly empty",
                  journal.free_cells() > before);

    // THE NO-STALL CLAIM, and it is what makes an ordinary save legal
    // from the main loop on this target at all. RM0444 3.3.9: a program
    // or erase in bank 2 leaves reads of bank 1 running. The witness is
    // the number of turns flash.hpp's own wait loop - which lives in
    // bank 1 - completed while the attic was being erased. A stalled bus
    // would have allowed none.
    const uint32_t turns = Flash::last_wait_turns();
    print(serial, "  the last wait inside that collection spun ", turns,
          " times out of bank 1 while bank 2 was busy", crlf);
    bench.verdict("READ-WHILE-WRITE holds for the attic too: the CPU kept "
                  "executing throughout, which is what makes save() legal "
                  "from the loop here",
                  turns > 1'000u);

    // The reserve is restored behind an ordinary save. It is the
    // invariant, not a coincidence.
    bench.verdict("and the reserve is intact behind it", journal.reserve_intact());
}

// =============================================================================
// e - coexistence
// =============================================================================
void te_coexist() {
    const Heap::MountReport& hr = heap.mount();
    print(serial, "  heap: status=", static_cast<uint8_t>(hr.status),
          " survivors=", hr.survivors, " lost=", hr.lost, " free=",
          heap.free_pages(0), " pages", crlf);
    bench.verdict("the heap mounts in the SAME BANK, below the attic",
                  hr.mounted());

    // A block of this suite's own, so nothing here depends on
    // test_stm32_nvm having run first.
    constexpr uint32_t block_len = 240;
    for (uint32_t i = 0; i < block_len; ++i) {
        // out_buf is only 40 bytes; the block is written in chunks.
        (void)i;
    }
    bool wrote = false;
    uint32_t block_addr = 0;
    if (auto w = heap.alloc(heap_record, block_len)) {
        block_addr = w->address();
        wrote = true;
        for (uint32_t off = 0; off < block_len && wrote; off += 40u) {
            const uint8_t n =
                static_cast<uint8_t>(block_len - off < 40u ? block_len - off : 40u);
            for (uint8_t i = 0; i < n; ++i) {
                out_buf[i] = pattern(off + i, 0xB7);
            }
            wrote = w->append(out_span(n));
        }
        wrote = wrote && w->seal();
    }
    bench.verdict("a heap block is written beside the journal", wrote);
    print(serial, "  block ", hex(heap_record), " at ",
          hex(MainFlashPartition::address_of(block_addr)), ", ", block_len,
          " bytes", crlf);
    bench.verdict("and it landed BELOW the attic, where the heap's zone ends",
                  block_addr + block_len <= MainFlashPartition::journal_base);

    // Now churn the journal hard enough to collect at least once, and
    // check the block afterwards. The two users share one bank and one
    // FLASH_CR; what must not happen is either one reaching the other's
    // pages.
    const uint32_t erases0 = MeteredZone::erases;
    bool churned = true;
    for (uint16_t i = 0; i < 120u; ++i) {
        churned = churned && put(id_churn, 16, static_cast<uint8_t>(i ^ 0x5Au));
    }
    print(serial, "  120 journal saves cost ", MeteredZone::erases - erases0,
          " attic page erase(s)", crlf);
    bench.verdict("the journal churned through at least one collection",
                  churned && MeteredZone::erases - erases0 >= 2u);

    bool block_ok = false;
    if (const auto f = heap.find(heap_record)) {
        block_ok = f->length == block_len;
        for (uint32_t off = 0; off < block_len && block_ok; off += 40u) {
            const uint8_t n =
                static_cast<uint8_t>(block_len - off < 40u ? block_len - off : 40u);
            block_ok = f->read(off, in_span(n));
            for (uint8_t i = 0; i < n && block_ok; ++i) {
                block_ok = in_buf[i] == pattern(off + i, 0xB7);
            }
        }
    }
    bench.verdict("the heap's block is byte-exact after all of it - two "
                  "storage classes over one bank, neither reaching the other",
                  block_ok);

    // And the other way round: the heap mutating while journal values
    // stand.
    const uint8_t seed = 0xC9;
    (void)put(id_bytes, 20, seed);
    if (auto w = heap.rewrite(heap_record)) {
        for (uint32_t off = 0; off < block_len; off += 40u) {
            const uint8_t n =
                static_cast<uint8_t>(block_len - off < 40u ? block_len - off : 40u);
            for (uint8_t i = 0; i < n; ++i) {
                out_buf[i] = pattern(off + i, 0xB8);
            }
            (void)w->append(out_span(n));
        }
        (void)w->seal();
    }
    bench.verdict("a heap mutation leaves the journal's values untouched",
                  remount().mounted() && holds(id_bytes, 20, seed));
}

// =============================================================================
// f - the panic reserve
// =============================================================================
void tf_reserve() {
    (void)remount();

    // The invariant: after every completed ordinary save the half still
    // holds room for one more MAXIMUM-SIZE entry, in cells that are
    // already erased.
    bool always = true;
    uint32_t worst = Journal::half_cells;
    for (uint16_t i = 0; i < 80u; ++i) {
        (void)put(id_churn, 16, static_cast<uint8_t>(i));
        const uint32_t free = journal.free_cells();
        if (free < worst) {
            worst = free;
        }
        always = always && journal.reserve_intact();
    }
    print(serial, "  over 80 saves the half never fell below ", worst,
          " free cells; the reserve is ", Journal::reserve_cells, crlf);
    bench.verdict("EVERY completed save leaves the reserve intact - the room "
                  "save_reserved() spends is a guarantee and not a hope",
                  always && worst >= Journal::reserve_cells);

    // And spending it costs NO erase: that is what makes it legal from a
    // panic handler, where an erase would be an unbounded wait on a
    // dying supply.
    const uint32_t erases0 = MeteredZone::erases;
    const uint32_t free_before = journal.free_cells();
    fill(32, 0x9E);
    const uint32_t t0 = cycles_now();
    const bool spent = journal.save_reserved(id_full, out_span(32));
    const uint32_t us = cycles_to_us(cycles_now() - t0);
    print(serial, "  save_reserved: ", us, " us, ",
          MeteredZone::erases - erases0, " erase(s), free ", free_before,
          " -> ", journal.free_cells(), crlf);

    bench.verdict("save_reserved() lands", spent);
    bench.verdict("and costs NO erase at all - one bounded polled program",
                  MeteredZone::erases == erases0);
    bench.verdict("what it wrote is what a later mount reads",
                  remount().mounted() && holds(id_full, 32, 0x9E));

    // The next ordinary save collects early and puts the reserve back.
    (void)put(id_churn, 16, 0x01);
    bench.verdict("and the next ORDINARY save restores the reserve",
                  journal.reserve_intact());
}

// =============================================================================
// w - the wear this run spent
// =============================================================================
void tw_wear() {
    print(serial, "  attic page erases this run: ", MeteredZone::erases,
          ", programs: ", MeteredZone::programs, ", reads: ", MeteredZone::reads,
          crlf);
    print(serial, "  DS13560 table 49 gives a page 10000 cycles minimum, and "
          "the two halves wear evenly by construction", crlf);

    bench.verdict("a z run stays well inside a sane erase budget",
                  MeteredZone::erases <= 40u);
    MeteredZone::clear();
}

// =============================================================================
// p - a real panic, a real reset, and the breadcrumb in FLASH
// =============================================================================
void tp_panic() {
    (void)remount();
    token.magic = token_magic;
    token.canary = token_canary;
    token.code = static_cast<uint8_t>(PanicCode::assert_failed);
    token.context = 0x5E;
    token.pass = 0;
    token.fail = 0;
    token.via_fault = 0;
    bench.reset_tally();

    // Start from a journal whose reserve is intact and whose panic id
    // holds nothing: the point is that the FAILING program can still
    // write, not that it found room by luck.
    (void)journal.save<PanicRecord>(id_panic, PanicRecord{0, 0, 0});
    token.free_before = journal.free_cells();
    print(serial, "  reserve ", journal.reserve_intact() ? "intact" : "SPENT",
          ", ", token.free_before, " free cells; panicking now", crlf);
    console_drain();

    token.leg = 1;
    // WHAT ACTUALLY HAPPENS HERE, and it is worth knowing before reading
    // the next boot's verdicts: panic() writes the SRAM breadcrumb, then
    // executes break_here() - a BKPT - and on a board whose C_DEBUGEN
    // tools/bench.py has cleared that BKPT ESCALATES INTO HardFault
    // before the reporter runs. So the record that crosses the reset may
    // have been written by JournalPanic OR by the fault body, and the
    // next boot reports which.
    panic<Stm32Platform, Panic>(PanicCode::assert_failed, 0x5E);
}

void tp_resume() {
    bench.resume_tally(token.pass, token.fail);
    const Journal::MountReport& r = remount();
    print_mount(r);

    bench.verdict("the journal still mounts after a panic and a reset",
                  r.mounted());
    bench.verdict("nothing was torn by the reset - a half-written entry "
                  "fails its CRC and is stepped over, which is the atomicity "
                  "working",
                  r.torn == 0u);

    const std::optional<PanicRecord> pending = Panic::peek();
    print(serial, "  journal breadcrumb: ",
          pending ? "present" : "ABSENT", " (written by ",
          token.via_fault ? "the HardFault BODY" : "the Reporter",
          "); SRAM breadcrumb: ",
          Stm32Platform::panic_record().magic == panic_magic ? "present"
                                                             : "taken/absent",
          crlf);

    if (pending) {
        print(serial, "  code=", pending->code, " context=", hex(pending->context),
              crlf);
    }
    bench.verdict("A PANIC CROSSED A RESET IN FLASH: the record is in the "
                  "journal, where a power loss could not have taken it",
                  pending.has_value());
    // WHICH PATH WROTE IT is the finding, not an implementation detail.
    // On this board panic()'s BKPT escalates into HardFault before the
    // Reporter is called, so an application that wants a flash
    // breadcrumb has to bind the fault body - the samc campaign's
    // finding, confirmed on the third silicon.
    bench.verdict("and it was the FAULT BODY that wrote it, not the Reporter: "
                  "with C_DEBUGEN cleared, panic()'s BKPT escalates before "
                  "Reporter::report() is ever reached",
                  token.via_fault != 0u);
    bench.verdict("its code is the one panic() was given",
                  pending && pending->code == token.code);
    bench.verdict("and its context byte came through untouched",
                  pending && pending->context == token.context);

    // take() consumes it through the ORDINARY save, which is affordable
    // on the boot path and restores the reserve for the next failure.
    const std::optional<PanicRecord> taken = Panic::take();
    bench.verdict("take() returns it once", taken.has_value());
    bench.verdict("and a second call returns nothing", !Panic::pending());
    bench.verdict("the reserve is back for the next failure",
                  journal.reserve_intact());

    bench.verdict("the values written before the panic are all still there",
                  holds(id_bytes, 20, 0xC9) || holds(id_bytes, 20, 0x31));
    bench.verdict("the token crossed the reset intact",
                  token.magic == token_magic && token.canary == token_canary);

    token.leg = 0;
    token.magic = 0;
    bench.end_letter();
}

// =============================================================================
// v - the survivors, after whatever happened in between
// =============================================================================
void tv_verify() {
    const Journal::MountReport& r = remount();
    print_mount(r);
    print(serial, "  this image is build ", MeteredZone::build_id(), crlf);

    bench.verdict("the journal still mounts", r.mounted());
    bench.verdict("and nothing is torn", r.torn == 0u);
    bench.verdict("the byte string letter b or e wrote is still readable",
                  holds(id_bytes, 20, 0x31) || holds(id_bytes, 20, 0xC9));
    bench.verdict("the typed value is field for field what it was", [] {
        const std::optional<Calibration> c = journal.load<Calibration>(id_cal);
        return c && c->offset == calibration.offset &&
               c->gain == calibration.gain && c->mode == calibration.mode;
    }());
    bench.verdict("and the maximum-size value is whole",
                  holds(id_full, 32, 0x53) || holds(id_full, 32, 0x9E));

    // THE POINT OF THIS LETTER. tools/bench.py flashes through OpenOCD's
    // `program <elf> verify`, which erases only the sectors the image
    // occupies - and ld/stm32g0b1re.ld gives the image bank 1 alone. So
    // a reflash of a DIFFERENT app cannot touch the attic, and these
    // values are the proof.
    print(serial, "  (run this after flashing another app and coming back: "
          "the attic is in bank 2, which no image reaches)", crlf);
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf, "test_stm32_journal - util/nv_journal.hpp in the "
          "STM32G0B1RE's bank-2 attic, clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// THE PANIC PATH ON THIS BOARD IS THE FAULT PATH, and this body is where
/// the campaign's one piece of application glue lives.
///
/// panic() writes the SRAM breadcrumb and then executes break_here() - a
/// BKPT - which on a core whose C_DEBUGEN tools/bench.py has cleared
/// ESCALATES INTO HardFault before the Reporter is ever called. Measured:
/// with only kernel/panic.hpp's own composition, JournalPanic::report()
/// never runs and the journal comes up empty after the reset. So the
/// fault body carries the record into flash itself, through the SAME
/// bounded, erase-free path the reporter would have used
/// (save_reserved(), one program into cells the last ordinary save left
/// ready) - which is exactly what makes it legal here, with interrupts
/// masked for good and the board about to reset.
///
/// It refuses to write a second record over a standing one, the way
/// hard_fault_reset() refuses to overwrite the SRAM breadcrumb.
extern "C" void HardFault_Handler() {
    const brio::PanicRecord& r = brio::Stm32Platform::panic_record();
    if (r.magic == brio::panic_magic && !Panic::pending()) {
        Panic::report(static_cast<brio::PanicCode>(r.code), r.context);
        if (token.magic == token_magic) {
            token.via_fault = 1;
        }
    }
    brio::hard_fault_reset<brio::Stm32Platform>(0x5E);
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    brio::enable_interrupts();

    bench.letter('a', "the partition and a read-only mount", ta_partition);
    bench.letter('b', "the round trip and the three refusals", tb_round_trip);
    bench.letter('c', "the halves ping-pong on silicon", tc_halves);
    bench.letter('d', "what it costs, and the no-stall claim", td_cost);
    bench.letter('e', "coexistence: a heap below, the journal above",
                 te_coexist);
    bench.letter('f', "the panic reserve", tf_reserve);
    bench.letter('w', "the wear this run spent", tw_wear);
    bench.letter('p', "A REAL PANIC AND A RESET (reboots the board)", tp_panic,
                 false);
    bench.letter('v', "the survivors, after a reflash (not in z)", tv_verify,
                 false);

    if (serial_ok && token.magic == token_magic && token.leg != 0) {
        tp_resume();
        bench.prompt();
    } else if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=",
                    clock_ok ? "PLL64" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", " attic=",
                    brio::hex(brio::MainFlashPartition::address_of(
                        brio::MainFlashPartition::journal_base)),
                    brio::crlf);
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
