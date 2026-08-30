// test_samc_journal - util/nv_journal.hpp on the SAM C21's RWWEE attic
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// a serial console; `z` runs them all and prints the "ALL: N pass, M
// fail" line tools/bench.py judges. Board C (ATSAMC21J18A), no wires.
//
// WHAT IT IS ABOUT. samc/nvm_flash.hpp partitions the 8 KB RWWEE array
// into two storage classes: rows 0..27 are the block heap's
// (util/nv_heap.hpp) and rows 28..31 are the value journal's
// (util/nv_journal.hpp), two 512-byte halves that ping-pong. This suite
// runs the journal on the real silicon and, in letter e, runs BOTH at
// once - which is the partition's whole point and the thing a host test
// cannot say anything about.
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
//   d  what it costs, and the no-stall claim, in the shape
//      test_samc_nvm letter d gives it: the polling turns the CPU
//      completes INSIDE an RWWEE row erase.
//   e  COEXISTENCE: a heap in rows 0..27 and the journal in the attic,
//      both mounted, the heap's block byte-exact while the journal
//      churns over it.
//   f  the panic reserve: after every completed save there is room for
//      one more maximum-size entry, and taking it costs NO erase.
//   w  the wear this run spent, in row erases, against a declared
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
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <span>

#include "kernel/panic.hpp"
#include "samc/clock.hpp"
#include "samc/nvic.hpp"
#include "samc/nvm.hpp"
#include "samc/nvm_flash.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/reset.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
#include "util/nv_heap.hpp"
#include "util/nv_journal.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter p lives in
//
// INLINE, and in .noinit, for the two reasons test_samc_platform gives:
// the section must survive the crt, and gcc gives an inline variable with
// a section attribute the COMDAT group a plain one does not get. Its
// magic word is not decoration - table 18-1 has no SRAM row at all, so
// nothing promises this object survives anything.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x4A11;

struct Token {
    uint16_t magic;
    uint8_t leg;      ///< 0 = nothing pending, 1 = a panic was fired
    uint8_t code;     ///< what panic() was given
    uint8_t context;
    uint8_t path;     ///< 1 = the reporter ran, 2 = the fault body did
    uint16_t pass;    ///< letter p's tally so far
    uint16_t fail;
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;

using Led = Pin<'B', 23>;

TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// The media, with a meter on it
//
// The journal runs over the real RwweeJournalZone; this wrapper only
// counts what passes through, because "the wear budget of a run" is a
// number the suite has to be able to PRINT rather than estimate. The
// static_asserts below are what keep it honest: every constant is the
// real medium's, so nothing about the journal's geometry is different
// for being metered.
// ---------------------------------------------------------------------------
struct MeteredZone {
    MeteredZone() = delete;

    static constexpr uint32_t erase_size = RwweeJournalZone::erase_size;
    static constexpr uint32_t write_cell = RwweeJournalZone::write_cell;
    static constexpr uint32_t flash_end = RwweeJournalZone::flash_end;
    static constexpr uint8_t zone_count = RwweeJournalZone::zone_count;

    static std::array<FlashZone, zone_count> zones() {
        return RwweeJournalZone::zones();
    }
    static void read(uint32_t addr, std::span<uint8_t> dst) {
        ++reads;
        RwweeJournalZone::read(addr, dst);
    }
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        ++programs;
        return RwweeJournalZone::program(addr, src);
    }
    static bool erase(uint32_t addr) {
        ++erases;
        return RwweeJournalZone::erase(addr);
    }
    static uint32_t build_id() { return RwweeJournalZone::build_id(); }

    static void clear() {
        reads = 0;
        programs = 0;
        erases = 0;
    }

    static inline uint32_t reads = 0;
    static inline uint32_t programs = 0;
    static inline uint32_t erases = 0;
};

static_assert(FlashMedia<MeteredZone>);
static_assert(MeteredZone::erase_size == 256u);
static_assert(MeteredZone::write_cell == 64u);
static_assert(MeteredZone::flash_end == Nvm::rwwee_end);

/// Six 32-byte values in two 512-byte halves: the arithmetic the
/// journal's own static_assert performs is (6 + 2) x 1 cell <= 8 cells.
using Journal = NvJournal<MeteredZone, 6, 32, 2>;
Journal journal;

/// The heap letter e runs beside it, in the OTHER partition.
using Heap = NvHeap<RwweeFlash, 8, 2>;
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

/// The one row of the HEAP's share this suite is allowed to touch by
/// hand: the bottom one, which the allocator (top-down) reaches last.
constexpr uint32_t scratch_row = Nvm::rwwee_base;

// ---------------------------------------------------------------------------
// A cycle-resolution stopwatch (the one test_samc_nvm and test_samc_dma use)
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

uint32_t cycles_to_us(uint32_t cycles) { return cycles / (SysClock::hz / 1'000'000UL); }

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
// The buffers are VOLATILE for the reason the DMAC campaign paid for on
// this target: they are read back from flash the CPU itself programmed
// through a side channel the optimizer cannot see, and gcc will fold such
// a read-back into whatever was last stored.
// ---------------------------------------------------------------------------
volatile uint8_t out_buf[40];
volatile uint8_t in_buf[40];

void fill(uint8_t len, uint8_t seed) {
    for (uint8_t i = 0; i < len; ++i) {
        out_buf[i] = static_cast<uint8_t>(seed ^ (i * 7u + 0x5Bu));
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
        if (in_buf[i] != static_cast<uint8_t>(seed ^ (i * 7u + 0x5Bu))) {
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
    print(serial, "  RWWEE ", hex(Nvm::rwwee_base), " .. ", hex(Nvm::rwwee_end),
          "   heap ", hex(Nvm::rwwee_base), " .. ", hex(RwweeFlash::flash_end),
          "   journal ", hex(RwweePartition::journal_base), " .. ",
          hex(Nvm::rwwee_end), crlf);
    print(serial, "  halves ", hex(Journal::half_base(0)), " and ",
          hex(Journal::half_base(1)), ", ", Journal::half_bytes,
          " bytes = ", Journal::half_cells, " cells each; one entry is at most ",
          Journal::max_entry_bytes, " bytes", crlf);

    // The partition is a pair of CONSTANTS on this target - no linker
    // symbol reaches into the RWWEE array - which is the whole reason
    // the split can be decided once in a header.
    bench.verdict("the journal owns the top four rows of the array",
                  RwweePartition::journal_bytes == 4u * Nvm::row_size &&
                      RwweePartition::journal_base ==
                          Nvm::rwwee_end - 4u * Nvm::row_size);
    bench.verdict("the heap's zone stops exactly where the journal starts",
                  RwweeFlash::zones()[0].ceiling == RwweePartition::journal_base &&
                      RwweeFlash::zones()[0].floor == Nvm::rwwee_base);
    bench.verdict("the two regions do not overlap, map home included",
                  Heap::map_home + 2u * Nvm::row_size <=
                      RwweePartition::journal_base);
    bench.verdict("the journal's halves fill the attic exactly",
                  Journal::journal_home == RwweePartition::journal_base &&
                      Journal::half_base(1) + Journal::half_bytes ==
                          Nvm::rwwee_end);

    // A mount is READ-ONLY: this is the boot path and it must cost no
    // cycle of endurance at all.
    const uint32_t erases0 = MeteredZone::erases;
    const uint32_t programs0 = MeteredZone::programs;
    const uint32_t reads0 = MeteredZone::reads;
    const uint32_t t0 = cycles_now();
    const auto& r = remount();
    const uint32_t mount_us = cycles_to_us(cycles_now() - t0);
    print_mount(r);
    print(serial, "  the mount read ", MeteredZone::reads - reads0,
          " times, programmed ", MeteredZone::programs - programs0, ", erased ",
          MeteredZone::erases - erases0, ", in ", mount_us, " us", crlf);
    bench.verdict("the journal mounts", r.mounted());
    bench.verdict("MOUNTING COSTS NO ERASE AND NO PROGRAM",
                  MeteredZone::erases == erases0 &&
                      MeteredZone::programs == programs0);
    bench.verdict("and it is fast enough to be on the boot path",
                  mount_us < 5000u);
}

// =============================================================================
// b - the round trip and the refusals
// =============================================================================
void tb_roundtrip() {
    if (!remount().mounted()) {
        bench.verdict("the journal mounts", false);
        return;
    }

    bench.verdict("a byte string saves", put(id_bytes, 12, 0xA1));
    bench.verdict("and reads back byte-exact from flash",
                  holds(id_bytes, 12, 0xA1));

    bench.verdict("the typed twin saves a struct",
                  journal.save<Calibration>(id_cal, calibration));
    const std::optional<Calibration> got = journal.load<Calibration>(id_cal);
    bench.verdict("and hands it back field for field",
                  got && got->offset == calibration.offset &&
                      got->gain == calibration.gain &&
                      got->mode == calibration.mode);
    // A type of a different size reads as ABSENT, not as garbage: the
    // stored length has to match exactly.
    bench.verdict("a different type of a different size reads as absent",
                  !journal.load<uint32_t>(id_cal).has_value());

    bench.verdict("a one-byte value is a value", put(id_short, 1, 0x33));
    bench.verdict("and so is a maximum-size one", put(id_full, 32, 0x77));
    bench.verdict("both read back", holds(id_short, 1, 0x33) &&
                                        holds(id_full, 32, 0x77));

    // Latest-wins, on flash: five versions of one id, and the index is
    // still one id.
    bool latest = true;
    for (uint8_t k = 1; k <= 5; ++k) {
        latest = latest && put(id_churn, 20, static_cast<uint8_t>(k * 17u));
    }
    bench.verdict("five versions of one id leave one id", latest &&
                                                              journal.has(id_churn));
    bench.verdict("and the LATEST is the one that rules",
                  holds(id_churn, 20, static_cast<uint8_t>(5u * 17u)));

    // A short reader is refused, never served a truncated value.
    bench.verdict("a destination too small is refused, not truncated",
                  !journal.load(id_full, in_span(31)).has_value());

    // What can never fit is refused with a reason, and NOTHING is
    // written for it - not one cell, not one row.
    const uint32_t erases0 = MeteredZone::erases;
    const uint32_t programs0 = MeteredZone::programs;
    fill(33, 0x01);
    const bool oversize = !journal.save(id_bytes, out_span(33));
    print(serial, "  a 33-byte value -> ", error_name(journal.last_error()), crlf);
    bench.verdict("a value larger than max_payload is refused",
                  oversize && journal.last_error() == NvJournalError::too_big);
    bench.verdict("and a refusal writes nothing at all",
                  MeteredZone::erases == erases0 &&
                      MeteredZone::programs == programs0);

    // The sixth id, then the seventh.
    bench.verdict("the sixth id fits", put(id_panic, 4, 0x0F));
    const bool seventh = !put(6, 4, 0x0F);
    print(serial, "  a seventh id -> ", error_name(journal.last_error()), crlf);
    bench.verdict("max_ids is a declared limit and the seventh id is refused",
                  seventh && journal.last_error() == NvJournalError::too_many_ids);

    // The whole set survives a mount that re-reads it from flash.
    const auto& r = remount();
    print_mount(r);
    bench.verdict("a fresh mount finds every id again",
                  r.mounted() && r.live == 6 &&
                      holds(id_bytes, 12, 0xA1) && holds(id_short, 1, 0x33) &&
                      holds(id_full, 32, 0x77) &&
                      holds(id_churn, 20, static_cast<uint8_t>(5u * 17u)));
    const std::optional<Calibration> again = journal.load<Calibration>(id_cal);
    bench.verdict("the typed value too",
                  again && again->offset == calibration.offset &&
                      again->gain == calibration.gain);
}

// =============================================================================
// c - the halves ping-pong on silicon
// =============================================================================
void tc_collection() {
    const auto& r0 = remount();
    print_mount(r0);
    if (!r0.mounted()) {
        bench.verdict("the journal mounts", false);
        return;
    }
    // Letter b's values are the ones that must come through; if this
    // letter is run alone they are re-established here.
    if (!journal.has(id_bytes)) {
        (void)put(id_bytes, 12, 0xA1);
        (void)journal.save<Calibration>(id_cal, calibration);
        (void)put(id_short, 1, 0x33);
        (void)put(id_full, 32, 0x77);
    }

    const uint8_t first = journal.active_half();
    uint32_t collections = 0;
    uint8_t here = first;
    bool reserve_held = true;
    bool saved = true;
    const uint32_t erases0 = MeteredZone::erases;
    for (uint8_t k = 0; k < 10; ++k) {
        saved = saved && put(id_churn, 24, static_cast<uint8_t>(0xB0u + k));
        if (journal.active_half() != here) {
            ++collections;
            here = journal.active_half();
        }
        reserve_held = reserve_held && journal.reserve_intact();
    }
    print(serial, "  10 saves of one id: ", collections,
          " collections, active half ", first, " -> ", here, ", ",
          MeteredZone::erases - erases0, " row erases", crlf);

    bench.verdict("10 saves of a 24-byte value all land", saved);
    bench.verdict("A COLLECTION HAPPENED, more than once", collections >= 2u);
    bench.verdict("and the halves alternated", here != first || collections >= 2u);
    bench.verdict("the reserve was intact after every single one of them",
                  reserve_held);
    bench.verdict("the churned value is the last one written",
                  holds(id_churn, 24, static_cast<uint8_t>(0xB0u + 9u)));
    bench.verdict("and the values that were only carried are byte-exact",
                  holds(id_bytes, 12, 0xA1) && holds(id_short, 1, 0x33) &&
                      holds(id_full, 32, 0x77));

    // The survival path: re-read from flash, not from the RAM index.
    const auto& r = remount();
    print_mount(r);
    bench.verdict("a fresh mount after the collections finds everything",
                  r.mounted() && r.live >= 5 && !r.collect_pending &&
                      holds(id_churn, 24, static_cast<uint8_t>(0xB0u + 9u)) &&
                      holds(id_bytes, 12, 0xA1) && holds(id_full, 32, 0x77));
    bench.verdict("with no torn entry left behind by a clean run", r.torn == 0u);
}

// =============================================================================
// d - what it costs, and the no-stall claim
// =============================================================================
void td_cost() {
    if (!remount().mounted()) {
        bench.verdict("the journal mounts", false);
        return;
    }

    // THE NO-STALL CLAIM, in the shape test_samc_nvm letter d gives it:
    // the erase is issued by hand so that the POLLING TURNS between the
    // command and READY are the measurement. The target is the bottom
    // row of the HEAP's share, which the allocator (top-down) reaches
    // last - not a journal row, because the journal's own bookkeeping
    // must not be disturbed by a measurement.
    Nvm::clear_status();
    const uint32_t e0 = cycles_now();
    Nvm::address(NvmArray::rwwee, scratch_row);
    NVMCTRL_REGS->NVMCTRL_CTRLA = static_cast<uint16_t>(
        NVMCTRL_CTRLA_CMD(NVMCTRL_CTRLA_CMD_RWWEEER_Val) |
        NVMCTRL_CTRLA_CMDEX(NVMCTRL_CTRLA_CMDEX_KEY_Val));
    uint32_t spins = 0;
    while (!Nvm::ready()) {
        ++spins;
    }
    const uint32_t erase_us = cycles_to_us(cycles_now() - e0);
    const NvmError erased = Nvm::outcome();
    print(serial, "  one RWWEE row erase: ", erase_us, " us, and the CPU ran ",
          spins, " polling turns INSIDE it", crlf);
    bench.verdict("the row erases", erased == NvmError::none);
    bench.verdict("THE CPU IS NOT STALLED by an RWWEE erase (27.6.4.1)",
                  spins > 1000u);

    // THE BARE PAGE PROGRAM, on the row just erased and through the same
    // driver call the journal's media uses. This is the reference the
    // save below is weighed against: without it, a save's cost is a
    // number with nothing to compare to.
    uint8_t page[Nvm::page_size];
    for (uint32_t i = 0; i < Nvm::page_size; ++i) {
        page[i] = static_cast<uint8_t>(i * 3u + 0x11u);
    }
    const uint32_t p0 = cycles_now();
    const NvmError wrote = Nvm::program_page(
        NvmArray::rwwee, scratch_row,
        std::span<const uint8_t>(page, Nvm::page_size));
    const uint32_t page_us = cycles_to_us(cycles_now() - p0);
    bench.verdict("a bare page programs", wrote == NvmError::none);

    // WHAT ONE SAVE COSTS. Timed on save_reserved rather than on save,
    // and that is a measurement decision worth its sentence: an
    // ordinary save may fall due for a collection and then reports the
    // collection's cost, while save_reserved never collects BY
    // CONSTRUCTION - so every reading here is one page program plus the
    // journal's own work and nothing else. Five rounds, nothing printed
    // inside a window, the spread reported before any band is claimed.
    uint32_t best = 0xFFFFFFFFu;
    uint32_t worst = 0;
    bool all_landed = true;
    for (uint8_t k = 0; k < 5; ++k) {
        // The ordinary save restores the reserve the previous round
        // spent; its own cost is not part of this measurement. It goes
        // to the CHURN id, because letter v has to be able to check
        // letter b's values after everything else has run.
        all_landed = all_landed && put(id_churn, 32, static_cast<uint8_t>(k));
        fill(32, static_cast<uint8_t>(0xC0u + k));
        const uint32_t t0 = cycles_now();
        const bool ok = journal.save_reserved(id_churn, out_span(32));
        const uint32_t us = cycles_to_us(cycles_now() - t0);
        all_landed = all_landed && ok;
        best = us < best ? us : best;
        worst = us > worst ? us : worst;
    }
    print(serial, "  a bare page program: ", page_us,
          " us;  a save: ", best, " .. ", worst, " us over five", crlf);
    print(serial, "  the difference is the journal's own work - building a "
          "64-byte entry image and its BITWISE CRC-16 (util/crc.hpp)", crlf);
    bench.verdict("every save landed", all_landed);
    bench.verdict("a save is one page program plus its entry image, and the "
                  "page program is the larger half",
                  best > page_us && worst < 2u * page_us &&
                      worst - best <= page_us / 4u);

    // A collection is four row erases plus one page program per live
    // id, and that is what it must measure.
    const uint32_t before_erases = MeteredZone::erases;
    const uint32_t before_programs = MeteredZone::programs;
    const uint32_t c0 = cycles_now();
    const bool collected = journal.collect();
    const uint32_t collect_us = cycles_to_us(cycles_now() - c0);
    const uint32_t rows = MeteredZone::erases - before_erases;
    const uint32_t pages = MeteredZone::programs - before_programs;
    print(serial, "  a collection: ", collect_us, " us, ", rows, " row erases, ",
          pages, " page programs for ", journal.count(), " live ids", crlf);
    bench.verdict("a collection erases BOTH halves, two rows each",
                  collected && rows == 4u);
    bench.verdict("and copies exactly one page per live id",
                  pages == journal.count());
    bench.verdict("its cost is four erases plus the copies",
                  collect_us > 3u * erase_us &&
                      collect_us < 6u * erase_us + pages * 400u);

    // The software timebase keeps up through all of it, which is the
    // other half of the no-stall story: SysTick_Handler is code in the
    // MAIN array, and a main-array operation is what makes it go stale
    // (test_samc_nvm letter m).
    const uint32_t t0 = Ticker::ticks();
    const uint32_t k0 = cycles_now();
    for (uint8_t k = 0; k < 3; ++k) {
        (void)journal.collect();
    }
    const uint32_t sw_us = cycles_to_us(cycles_now() - k0);
    const uint32_t ticks_seen = Ticker::ticks() - t0;
    print(serial, "  three collections: stopwatch ", sw_us, " us, tick counter ",
          ticks_seen, " ms", crlf);
    bench.verdict("the software timebase keeps up with the stopwatch through "
                  "an RWWEE operation",
                  ticks_seen + 2u >= sw_us / 1000u &&
                      ticks_seen <= sw_us / 1000u + 2u);
}

// =============================================================================
// e - COEXISTENCE: the heap and the journal in one array
// =============================================================================
void te_coexistence() {
    const auto& jr = remount();
    print_mount(jr);
    const auto& hr = heap.mount();
    print(serial, "  heap: status=", static_cast<uint8_t>(hr.status),
          " survivors=", hr.survivors, " lost=", hr.lost, " seq=", hr.seq, crlf);
    bench.verdict("both mount over the same array", jr.mounted() && hr.mounted());

    // A block through the allocator's own writer, in the heap's share.
    constexpr uint32_t payload = 300;
    auto writer = heap.alloc(heap_record, payload);
    bench.verdict("the heap allocates a block", writer.has_value());
    if (!writer) {
        return;
    }
    const uint32_t block_addr = writer->address();
    bool appended = true;
    for (uint32_t i = 0; i < payload; ++i) {
        const uint8_t b = static_cast<uint8_t>(i * 5u + 0x21u);
        appended = appended && writer->append(std::span<const uint8_t>(&b, 1));
    }
    bench.verdict("the payload appends and seals", appended && writer->seal());
    bench.verdict("and the block landed BELOW the journal's attic",
                  block_addr + payload <= RwweePartition::journal_base);

    // Now churn the journal hard enough to collect, right over the top
    // of the block. This is the partition's whole point.
    const uint32_t erases_before = MeteredZone::erases;
    bool churned = true;
    for (uint8_t k = 0; k < 10; ++k) {
        churned = churned && put(id_churn, 28, static_cast<uint8_t>(0x40u + k));
    }
    print(serial, "  10 journal saves over the block: ",
          MeteredZone::erases - erases_before, " attic row erases", crlf);
    bench.verdict("the journal churns, collections and all", churned);
    bench.verdict("and it did erase attic rows while doing so",
                  MeteredZone::erases > erases_before);

    // The heap, re-read from flash: the block must be untouched.
    const auto& again = heap.mount();
    print(serial, "  heap re-mount: survivors=", again.survivors,
          " lost=", again.lost, crlf);
    bench.verdict("the heap re-mounts with nothing lost",
                  again.mounted() && again.lost == 0u);
    const std::optional<NvBlock<RwweeFlash>> found = heap.find(heap_record);
    bench.verdict("its block is still there, at the same address",
                  found && found->address == block_addr &&
                      found->length == payload);
    bool exact = found.has_value();
    if (found) {
        uint8_t got[64];
        for (uint32_t off = 0; off < payload && exact; off += sizeof got) {
            const uint32_t n = payload - off < sizeof got ? payload - off
                                                          : sizeof got;
            exact = found->read(off, std::span<uint8_t>(got, n));
            for (uint32_t i = 0; i < n && exact; ++i) {
                exact = got[i] == static_cast<uint8_t>((off + i) * 5u + 0x21u);
            }
        }
    }
    bench.verdict("AND ITS PAYLOAD IS BYTE-EXACT after the journal's churn",
                  exact);
    bench.verdict("while the journal still holds what it wrote",
                  holds(id_churn, 28, static_cast<uint8_t>(0x40u + 9u)));
}

// =============================================================================
// f - the panic reserve
// =============================================================================
void tf_reserve() {
    if (!remount().mounted()) {
        bench.verdict("the journal mounts", false);
        return;
    }

    // EIGHT rounds and not the host suite's three thousand, and the
    // reason is endurance: with six live ids in an eight-cell half a
    // collection falls due at every second save, so every round here
    // costs four real row erases. The exhaustive sweep is host work
    // (test_nv_journal); what silicon has to answer is whether the
    // guarantee holds over a real page program.
    const uint32_t erases0 = MeteredZone::erases;
    bool invariant = true;
    bool reserved_ok = true;
    bool no_erase = true;
    bool readback = true;
    for (uint8_t k = 0; k < 8; ++k) {
        if (!put(id_churn, static_cast<uint8_t>(k + 4u), static_cast<uint8_t>(k))) {
            invariant = false;
            break;
        }
        invariant = invariant && journal.reserve_intact();

        // The panic path, exercised exactly where a panic could happen:
        // a maximum-size entry, no collection, NO ERASE.
        fill(32, static_cast<uint8_t>(0xE0u + k));
        const uint32_t erases = MeteredZone::erases;
        reserved_ok = reserved_ok && journal.save_reserved(id_panic, out_span(32));
        no_erase = no_erase && MeteredZone::erases == erases;
        readback = readback && holds(id_panic, 32, static_cast<uint8_t>(0xE0u + k));
    }
    print(serial, "  8 rounds of save + save_reserved: ",
          MeteredZone::erases - erases0, " row erases, free ",
          journal.free_cells(), "/", Journal::half_cells, " cells", crlf);

    bench.verdict("EVERY COMPLETED SAVE LEFT ROOM FOR ONE MORE MAXIMUM-SIZE "
                  "ENTRY", invariant);
    bench.verdict("and save_reserved always found it", reserved_ok);
    bench.verdict("SPENDING THE RESERVE COSTS NO ERASE", no_erase);
    bench.verdict("what it wrote reads back every time", readback);

    // And it all survives a mount.
    const auto& r = remount();
    print_mount(r);
    bench.verdict("the last reserved write is there after a fresh mount",
                  r.mounted() &&
                      holds(id_panic, 32, static_cast<uint8_t>(0xE0u + 7u)));
}

// =============================================================================
// w - the wear this run spent
// =============================================================================
void tw_wear() {
    const uint32_t rows = MeteredZone::erases;
    const uint32_t pages = MeteredZone::programs;
    print(serial, "  this run spent ", rows, " row erases and ", pages,
          " page programs in the attic", crlf);
    print(serial, "  the attic is four rows at 100k cycles each (45-43), "
          "so a z run costs about ", rows / 4u, " cycles of each", crlf);

    // The budget is a DECLARED limit, not an observation: a change that
    // makes a z run cost twice as much endurance should say so here
    // rather than be discovered by a dead board.
    bench.verdict("a z run stays inside its declared wear budget", rows <= 240u);
    bench.verdict("and every erase was matched by work", pages > 0u);

    // Zeroing them is what makes this letter re-runnable: each z run
    // reports its OWN budget rather than the power-on's running total.
    MeteredZone::clear();
}

// =============================================================================
// v - verify the survivors (by name: the reflash choreography)
// =============================================================================
void tv_survivors() {
    const auto& r = remount();
    print_mount(r);
    print(serial, "  build id of the image that is running: ",
          hex(MeteredZone::build_id()), crlf);
    bench.verdict("the journal mounts after whatever happened in between",
                  r.mounted());
    bench.verdict("letter b's byte string is still there",
                  holds(id_bytes, 12, 0xA1));
    const std::optional<Calibration> cal = journal.load<Calibration>(id_cal);
    bench.verdict("its typed value came through field for field",
                  cal && cal->offset == calibration.offset &&
                      cal->gain == calibration.gain &&
                      cal->mode == calibration.mode);
    bench.verdict("the one-byte value too", holds(id_short, 1, 0x33));
    bench.verdict("and the maximum-size one", holds(id_full, 32, 0x77));
    bench.verdict("nothing was torn by the way", r.torn == 0u);
}

// =============================================================================
// p - a real panic into the journal, across a reset (by name: it reboots)
// =============================================================================
//
// The reporter that panic() is given writes the breadcrumb into the
// journal through the RESERVE and then resets. What actually happens on
// this board is worth knowing and the token records it: panic() calls
// break_here() BEFORE the reporter, break_here() is a BKPT, and with
// DHCSR.C_DEBUGEN cleared - which is what bench.py does as the last step
// of every SAM flash - a BKPT escalates to HardFault. So the fault body
// below is the path that runs, and it does exactly what the reporter
// would have: the reserve, then the reset. Both are bound, because which
// one runs is a property of the debug state and not of the program.

constexpr uint8_t panic_context = 0x5A;
using Panic = JournalPanic<journal, id_panic>;

void write_breadcrumb(PanicCode code, uint8_t context, uint8_t path) {
    token.path = path;
    Panic::report(code, context);
}

/// The panic Reporter: the journal, then the reset. Two reporters
/// chained, which is the composition kernel/panic.hpp describes.
struct JournalThenReset {
    static void report(PanicCode code, uint8_t context) {
        write_breadcrumb(code, context, 1);
        ResetReporter::report(code, context);
    }
};

void tp_panic() {
    if (!remount().mounted()) {
        bench.verdict("the journal mounts", false);
        return;
    }
    // A value that must still be there afterwards, and an ordinary save
    // to restore the reserve the panic path is about to spend.
    bench.verdict("a value is stored beside the breadcrumb",
                  put(id_bytes, 12, 0xA1));
    bench.verdict("and the reserve is intact before the panic",
                  journal.reserve_intact());

    token.magic = token_magic;
    token.leg = 1;
    token.code = static_cast<uint8_t>(PanicCode::assert_failed);
    token.context = panic_context;
    token.path = 0;
    token.pass = bench.passed();
    token.fail = bench.failed();

    print(serial, "  panic() through JournalPanic, then a reset ...", crlf);
    console_drain();
    panic<SamPlatform, JournalThenReset>(PanicCode::assert_failed,
                                         panic_context);
}

/// Everything after the reset. main() calls this instead of the banner.
void tp_resume() {
    bench.resume_tally(token.pass, token.fail);
    print(serial, crlf, "p (continued after the panic reset)", crlf);

    const ResetCause cause = Reset::cause();
    const auto& r = remount();
    print_mount(r);
    print(serial, "  reset cause=", static_cast<uint8_t>(cause),
          ", the breadcrumb was written by ",
          token.path == 2u ? "the HardFault body (BKPT escalated)"
                           : "the reporter itself",
          crlf);

    bench.verdict("the board came back from a system reset request",
                  cause == ResetCause::system_request);
    bench.verdict("the journal mounts and is not torn", r.mounted() &&
                                                            !r.collect_pending);
    bench.verdict("A PANIC RECORD IS WAITING IN FLASH", Panic::pending());
    const std::optional<PanicRecord> got = Panic::take();
    bench.verdict("take() hands it over", got.has_value());
    bench.verdict("its code is the one panic() was given",
                  got && got->code == token.code);
    bench.verdict("its context byte came through untouched",
                  got && got->context == token.context);
    bench.verdict("a second take() returns nothing", !Panic::take().has_value());
    bench.verdict("the value stored beside it is untouched",
                  holds(id_bytes, 12, 0xA1));
    bench.verdict("and taking it restored the reserve for the next failure",
                  journal.reserve_intact());

    token.leg = 0;
    token.magic = 0;
    bench.end_letter();
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf, "test_samc_journal - util/nv_journal.hpp on the RWWEE "
          "attic, clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// AN UNBOUND VECTOR HERE IS A SILENT DEATH: the crt's default handler is
// a spin loop, so the first console interrupt would park the CPU in it
// and the board would simply never say anything.
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

/// The other end of letter p. panic() writes the SRAM breadcrumb BEFORE
/// break_here(), so by the time a BKPT escalates to here the code and
/// context are already known - and this body finishes what the reporter
/// was going to do. It refuses to act on anything that is not a valid
/// record, so a fault from somewhere else still resets but writes no
/// breadcrumb it cannot vouch for.
extern "C" void HardFault_Handler() {
    const brio::PanicRecord& r = brio::SamPlatform::panic_record();
    if (r.magic == brio::panic_magic && token.magic == token_magic) {
        write_breadcrumb(static_cast<brio::PanicCode>(r.code), r.context, 2);
    }
    brio::Reset::software();
    for (;;) {
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    brio::enable_interrupts();

    bench.letter('a', "the partition, and a read-only mount", ta_partition);
    bench.letter('b', "the round trip and the refusals", tb_roundtrip);
    bench.letter('c', "the halves ping-pong on silicon", tc_collection);
    bench.letter('d', "what it costs, and the no-stall claim", td_cost);
    bench.letter('e', "coexistence: the heap and the journal in one array",
                 te_coexistence);
    bench.letter('f', "the panic reserve", tf_reserve);
    bench.letter('w', "the wear this run spent", tw_wear);
    bench.letter('v', "verify the survivors", tv_survivors, false);
    bench.letter('p', "a real panic into the journal, across a reset",
                 tp_panic, false);

    const bool resuming = token.magic == token_magic && token.leg == 1;
    (void)journal.mount();

    if (serial_ok) {
        if (resuming) {
            tp_resume();
        } else {
            print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
                  " tick=", tick_ok ? "SysTick" : "FAILED", crlf);
            banner();
        }
    }
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), brio::crlf);
        if (c == '?') {
            bench.menu();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
