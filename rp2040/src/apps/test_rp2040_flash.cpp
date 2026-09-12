// test_rp2040_flash - the reference bench suite for the RP2040's FLASH:
// rp2040/flash.hpp (the engine over the bootrom's functions, run from
// SRAM with the flash disconnected; the raw command; the XIP cache),
// rp2040/nvm_flash.hpp (the constant partition at the top of the chip
// and its two FlashMedia), util/nv_heap.hpp and util/nv_journal.hpp on
// their fifth silicon - the first where the storage is an EXTERNAL chip
// the core executes out of.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE. GP0/GP1 are the console's, GP25 the LED. Every
// letter but a and d writes flash, so the counts are kept small on
// purpose: a run of `z` costs a dozen sector erases out of the chip's
// endurance.
//
// THE RULER: the system timer (microseconds) around every window, and
// the ticker's count across it for the ticks it loses.
//
// What is exercised, letter by letter:
//   a  the engine and the partition: the bootrom's table found and its
//      six functions, the stage's CRC computed as the bootrom does,
//      the chip's JEDEC id (its capacity against the build's size),
//      its unique id and status register, the linker's boundary
//      against the storage floor, the two media's zones, the build id
//   b  one sector, one page: erased to 0xFF (timed), a page programmed
//      to a pattern (timed), read back exact through the cache and
//      past it, erased again - and the refusals: a misaligned address,
//      a short span, a source in the window, the media's bounds
//   c  THE QUESTION THE CHAPTER DOES NOT ANSWER: a page programmed a
//      second time between erases, clearing more bits - what the chip
//      holds afterwards, printed and judged for consistency
//   d  THE WINDOW AND THE CACHE: the ticks the ticker loses during a
//      sector erase, the console's bytes held meanwhile; the hit ratio
//      of a walk over one sector read twice, the flush, and the cache
//      coherent with the chip after a program
//   e  NvHeap on this media: mount, a block written through the
//      allocator's writer, sealed, found again after a re-mount and
//      read back byte-exact
//   f  NvJournal in the attic: typed save and load, an overwrite, a
//      collection, a re-mount that holds - and the panic reserve
//      (save_reserved() with no erase)
//
//   w  (by name only, NOT in z) WIPE the storage partition: the 64 KB
//      from the floor to the top erased in one run (the 64 KB block
//      command), timed, so the next run starts from virgin flash.
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include "rp2040/clock.hpp"
#include "rp2040/flash.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/nvm_flash.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/nv_journal.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Led = Pin<25>;

TestBench<Serial, 16> bench;

using Heap = NvHeap<QspiFlash, 8, 2>;
Heap heap;
using Journal = NvJournal<QspiFlashJournalZone, 6, 32, 2>;
Journal journal;

constexpr uint16_t heap_record = 0x0301;
constexpr uint8_t id_cal = 1;
constexpr uint8_t id_churn = 2;
constexpr uint8_t id_panic = 3;

struct Calibration {
    uint32_t gain;
    int16_t offset;
    uint8_t revision;
};

uint8_t page_buf[Flash::page_size];
uint8_t got_buf[Flash::page_size];

uint32_t us_now() { return Timer::now_low(); }

/// Does the range read `value` through the alias past the cache?
bool range_is(uint32_t addr, uint32_t bytes, uint8_t value) {
    const uint8_t* p = Flash::uncached_address(addr);
    for (uint32_t i = 0; i < bytes; ++i) {
        if (p[i] != value) {
            return false;
        }
    }
    return true;
}

void print_id(const std::array<uint8_t, 8>& id) {
    constexpr char digits[] = "0123456789ABCDEF";
    for (const uint8_t b : id) {
        print(serial, digits[b >> 4], digits[b & 0x0Fu]);
    }
}

// =============================================================================
// a - the engine and the partition
// =============================================================================
void ta_engine() {
    const bool up = Flash::init();
    print(serial, "  bootrom table: ", up ? "found" : "NOT FOUND", ", version ", Flash::rom_version(), "; stage CRC ",
          hex(Flash::stage_crc()), " computed, ", hex(Flash::stage_crc_stored()), " stored", crlf);
    bench.verdict("the bootrom's table is found by its magic and every one of the six flash functions by its code; the "
                  "stage's CRC computed as the bootrom does is the one stored in its last word",
                  up && Flash::ready() && Flash::stage_crc() == Flash::stage_crc_stored());
    if (!up) {
        return;
    }

    const auto jedec = Flash::jedec_id();
    const auto uid = Flash::unique_id();
    const auto sr = Flash::status_register();
    print(serial, "  JEDEC id: ");
    if (jedec) {
        print(serial, hex(jedec->manufacturer), " ", hex(jedec->type), " ", hex(jedec->capacity), " (",
              jedec->manufacturer == FlashJedecId::winbond ? "Winbond" : jedec->manufacturer == FlashJedecId::zetta ? "Zetta" : "?",
              ", ", jedec->bytes() / 1024u, " KB)");
    } else {
        print(serial, "refused");
    }
    print(serial, "; unique id: ");
    if (uid) {
        print_id(*uid);
    } else {
        print(serial, "refused");
    }
    print(serial, "; status register 1: ", sr ? hex(*sr) : hex(uint8_t{0}), crlf);
    bench.verdict("the chip answers 9Fh with a JEDEC id whose capacity is the build's flash size, 4Bh with a unique id "
                  "that is not all ones or zeros, and 05h with a status register that is neither busy nor write-enabled",
                  jedec && jedec->bytes() == Flash::size_bytes && uid && *uid != std::array<uint8_t, 8>{} &&
                      *uid != std::array<uint8_t, 8>{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} && sr && (*sr & 0x03u) == 0u);
    bench.verdict("the same exchange asked again answers the same id (the SSI back in XIP mode between the two)",
                  Flash::unique_id() == uid);
    bench.verdict("XIP is enabled and the cache serves after the exchanges (this code runs)", Xip::enabled());

    const uint32_t end = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__brio_flash_end)) - Flash::window;
    print(serial, "  linker stops at ", hex(end), ", storage floor ", hex(QspiFlashPartition::storage_base), ", journal attic ",
          hex(QspiFlashPartition::journal_base), "..", hex(QspiFlashPartition::storage_end), crlf);
    bench.verdict("the linker's boundary is the partition's floor",
                  end == QspiFlashPartition::storage_base && QspiFlashPartition::geometry_matches_silicon());
    const auto hz = QspiFlash::zones();
    const auto jz = QspiFlashJournalZone::zones();
    bench.verdict("the heap's zone is the 48 KB between the floor and the attic, the journal's the 16 KB attic",
                  hz[0].floor == 0x1F0000u && hz[0].ceiling == 0x1FC000u && hz[0].size() == 48u * 1024u &&
                      jz[0].floor == 0x1FC000u && jz[0].ceiling == 0x200000u);
    bench.verdict("the build id is the link's epoch, not zero", QspiFlash::build_id() != 0u);
}

// =============================================================================
// b - one sector, one page
// =============================================================================
void tb_page() {
    const uint32_t sector = QspiFlashPartition::storage_base;   // the partition's first sector

    uint32_t t0 = us_now();
    bool ok = Flash::erase_sector(sector);
    const uint32_t erase_us = us_now() - t0;
    const auto sr = Flash::status_register();
    print(serial, "  sector erase: ", ok ? "ok" : "refused", " in ", erase_us, " us; status register after: ", sr ? hex(*sr) : hex(uint8_t{0}),
          crlf);
    bench.verdict("the sector erase completes, the chip no longer busy when the window closes (the bootrom waits for it)",
                  ok && sr && (*sr & 0x01u) == 0u);
    bench.verdict("and the sector reads all 0xFF past the cache", range_is(sector, Flash::sector_size, 0xFF));

    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        page_buf[i] = static_cast<uint8_t>(i * 7u + 0x13u);
    }
    t0 = us_now();
    ok = Flash::program(sector, page_buf);
    const uint32_t program_us = us_now() - t0;
    print(serial, "  page program: ", ok ? "ok" : "refused", " in ", program_us, " us", crlf);
    bench.verdict("the page program completes", ok);
    Flash::read(sector, got_buf);
    bench.verdict("and reads back byte-exact through the cache", memcmp(got_buf, page_buf, Flash::page_size) == 0);
    bench.verdict("and past it", memcmp(Flash::uncached_address(sector), page_buf, Flash::page_size) == 0);
    bench.verdict("the rest of the sector is still 0xFF", range_is(sector + Flash::page_size, Flash::sector_size - Flash::page_size, 0xFF));
    bench.verdict("a sector erase is under 400 ms and a page program under 5 ms (the chip's own maxima), the window "
                  "masked that long",
                  erase_us < 400'000u && program_us < 5'000u);

    ok = Flash::erase_sector(sector);
    bench.verdict("erased again, all 0xFF", ok && range_is(sector, Flash::sector_size, 0xFF));

    // The engine's refusals, before any window opens.
    bench.verdict("program() refuses a misaligned address", !Flash::program(sector + 4u, page_buf));
    bench.verdict("and a span that is not whole pages", !Flash::program(sector, std::span<const uint8_t>(page_buf, 100)));
    bench.verdict("and a source in the flash window", !Flash::program(sector, std::span<const uint8_t>(Flash::address(0), Flash::page_size)));
    bench.verdict("and a page past the chip", !Flash::program(Flash::size_bytes, page_buf));
    bench.verdict("erase() refuses a misaligned address, a run of no sectors and one past the chip",
                  !Flash::erase(sector + 256u, Flash::sector_size) && !Flash::erase(sector, 0) &&
                      !Flash::erase(Flash::size_bytes - Flash::sector_size, 2u * Flash::sector_size));
    bench.verdict("command() refuses an exchange of unequal lengths or one longer than it carries",
                  !Flash::command(std::span<const uint8_t>(page_buf, 4), std::span<uint8_t>(got_buf, 3)) &&
                      !Flash::command(std::span<const uint8_t>(page_buf, Flash::max_command + 1u), std::span<uint8_t>(got_buf, Flash::max_command + 1u)));
    // And the media's bounds: the image is not programmable through them.
    bench.verdict("the heap's media refuses an address below the storage floor",
                  !QspiFlash::program(QspiFlashPartition::storage_base - Flash::sector_size, page_buf));
    bench.verdict("and one in the journal's attic", !QspiFlash::program(QspiFlashPartition::journal_base, page_buf));
    bench.verdict("the journal's media refuses an address below the attic", !QspiFlashJournalZone::program(QspiFlashPartition::storage_base, page_buf));
}

// =============================================================================
// c - a second program between erases?
// =============================================================================
void tc_reprogram() {
    const uint32_t page = QspiFlashPartition::storage_base + Flash::sector_size;   // the second sector's first page
    (void)Flash::erase_sector(page);

    // First program: the left half a pattern, the right half untouched.
    uint8_t first[Flash::page_size];
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        first[i] = i < 128u ? static_cast<uint8_t>(0xA5u ^ i) : 0xFFu;
    }
    const bool ok1 = Flash::program(page, first);
    Flash::read(page, got_buf);
    bench.verdict("first program: the left half written, the right half left at 0xFF",
                  ok1 && memcmp(got_buf, first, Flash::page_size) == 0);

    // Second program, NO ERASE: the left half repeated unchanged (every
    // bit already where it is), the right half now a pattern. Only bits
    // going 1 -> 0 are asked for, the physics' own direction.
    uint8_t second[Flash::page_size];
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        second[i] = i < 128u ? first[i] : static_cast<uint8_t>(0x3Cu ^ i);
    }
    const bool ok2 = Flash::program(page, second);
    Flash::read(page, got_buf);
    const bool second_exact = memcmp(got_buf, second, Flash::page_size) == 0;
    print(serial, "  second program without an erase: ", ok2 ? "ok" : "refused", ", whole page exact=", second_exact, crlf);
    if (second_exact) {
        print(serial, "  -> THE CHIP ACCEPTS A SECOND PROGRAM between erases when only more bits are cleared: the "
                      "page is the cell by the bootrom's program, not by the chip", crlf);
    } else {
        print(serial, "  -> the chip does NOT re-program a page cleanly: the page is the cell for good", crlf);
    }
    // And the AND of the two: a byte asked to go 0 -> 1 stays 0.
    uint8_t third[Flash::page_size];
    memset(third, 0xFF, sizeof third);
    third[0] = static_cast<uint8_t>(~first[0]);
    const bool ok3 = Flash::program(page, third);
    Flash::read(page, got_buf);
    bench.verdict("a third program asking bits back to 1 leaves the AND (0 stays 0): what the chip holds is what "
                  "every program cleared",
                  ok3 && got_buf[0] == static_cast<uint8_t>(first[0] & third[0]) && got_buf[1] == second[1]);
    bench.verdict("the sector erases clean afterwards", Flash::erase_sector(page) && range_is(page, Flash::sector_size, 0xFF));
}

// =============================================================================
// d - the window and the cache
// =============================================================================
void td_window() {
    const uint32_t sector = QspiFlashPartition::storage_base + 2u * Flash::sector_size;

    // The ticks the ticker loses: SysTick pends once however long the
    // window stays masked, so the count comes up one where the timer
    // says tens.
    const uint32_t ticks0 = Ticker::ticks();
    const uint32_t t0 = us_now();
    const bool ok = Flash::erase_sector(sector);
    const uint32_t erase_us = us_now() - t0;
    const uint32_t ticks = Ticker::ticks() - ticks0;
    print(serial, "  a sector erase of ", erase_us, " us: the ticker counted ", ticks, " ms of it", crlf);
    bench.verdict("during a sector erase the ticker counts at most two ticks (one that fell before the window, the one "
                  "that pended in it) where the timer says five milliseconds: the window is masked and the tick pends once",
                  ok && ticks <= 2u && erase_us / 1000u > ticks + 1u);
    bench.verdict("the console's bytes queued during the window come out after it (this line prints)", true);

    // The cache: a walk over one sector, cold then warm.
    Xip::flush();
    Xip::reset_counters();
    volatile uint32_t sink = 0;
    const uint32_t* p = reinterpret_cast<const uint32_t*>(Flash::address(sector));
    for (uint32_t i = 0; i < Flash::sector_size / 4u; ++i) {
        sink += p[i];
    }
    const uint32_t cold_acc = Xip::accesses();
    const uint32_t cold_hit = Xip::hits();
    Xip::reset_counters();
    for (uint32_t i = 0; i < Flash::sector_size / 4u; ++i) {
        sink += p[i];
    }
    const uint32_t warm_acc = Xip::accesses();
    const uint32_t warm_hit = Xip::hits();
    print(serial, "  one sector read word by word: cold ", cold_hit, "/", cold_acc, " hits, warm ", warm_hit, "/", warm_acc,
          " (the code's own fetches counted too)", crlf);
    bench.verdict("the cold walk misses at least once a line (eight words) and the warm one hits nearly everything",
                  cold_acc >= Flash::sector_size / 4u && cold_acc - cold_hit >= Flash::sector_size / 32u &&
                      warm_acc >= Flash::sector_size / 4u && warm_hit * 100u >= warm_acc * 95u);
    Xip::flush();
    Xip::reset_counters();
    sink += p[0];
    const uint32_t after_flush_hit = Xip::hits();
    const uint32_t after_flush_acc = Xip::accesses();
    bench.verdict("after a flush the first word misses again", after_flush_acc >= 1u && after_flush_hit < after_flush_acc);

    // Coherence: the cached view of a page is refreshed by the
    // operation's own flush.
    Flash::read(sector, got_buf);   // the cache now holds the erased page
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        page_buf[i] = static_cast<uint8_t>(0x80u + i);
    }
    const bool programmed = Flash::program(sector, page_buf);
    Flash::read(sector, got_buf);
    bench.verdict("a page read through the cache, then programmed, reads its new bytes through the cache (the "
                  "operation flushes on its way back)",
                  programmed && memcmp(got_buf, page_buf, Flash::page_size) == 0 &&
                      memcmp(Flash::uncached_address(sector), page_buf, Flash::page_size) == 0);
    bench.verdict("the sector erases clean afterwards", Flash::erase_sector(sector) && range_is(sector, Flash::sector_size, 0xFF));
    (void)sink;
}

// =============================================================================
// e - the heap
// =============================================================================
void te_heap() {
    const auto& r = heap.mount();
    print(serial, "  heap mount: status=", static_cast<uint8_t>(r.status), " survivors=", r.survivors, " lost=", r.lost,
          " seq=", r.seq, crlf);
    bench.verdict("the heap mounts over the page-celled media", r.mounted());
    if (!r.mounted()) {
        return;
    }

    constexpr uint32_t payload = 300;   // two cells' worth, with a tail
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
    const bool sealed = writer->seal();
    print(serial, "  block at ", hex(block_addr), ", ", payload, " bytes, sealed=", sealed, crlf);
    bench.verdict("the payload appends byte by byte and seals", appended && sealed);
    bench.verdict("the block landed in the heap's share",
                  block_addr >= QspiFlashPartition::storage_base && block_addr + payload <= QspiFlashPartition::heap_end);

    const auto& again = heap.mount();
    print(serial, "  re-mount: status=", static_cast<uint8_t>(again.status), " survivors=", again.survivors, " lost=", again.lost,
          " seq=", again.seq, crlf);
    bench.verdict("the heap re-mounts with the block surviving", again.mounted() && again.survivors >= 1u && again.lost == 0u);
    const std::optional<NvBlock<QspiFlash>> found = heap.find(heap_record);
    bench.verdict("the block is found again, at its address and length",
                  found && found->address == block_addr && found->length == payload);
    bool exact = found.has_value();
    if (found) {
        uint8_t got[64];
        for (uint32_t off = 0; off < payload && exact; off += sizeof got) {
            const uint32_t n = payload - off < sizeof got ? payload - off : sizeof got;
            exact = found->read(off, std::span<uint8_t>(got, n));
            for (uint32_t i = 0; i < n && exact; ++i) {
                exact = got[i] == static_cast<uint8_t>((off + i) * 5u + 0x21u);
            }
        }
    }
    bench.verdict("and its payload reads back byte-exact", exact);
}

// =============================================================================
// f - the journal
// =============================================================================
uint8_t churn_buf[32];

void tf_journal() {
    const auto& r = journal.mount();
    print(serial, "  journal mount: status=", static_cast<uint8_t>(r.status), " live=", r.live, " torn=", r.torn,
          " active=", r.active, " seq=", r.seq, " used=", r.used_cells, "/", Journal::half_cells, crlf);
    bench.verdict("the journal mounts in the attic (one-page cells, thirty-two a half of two sectors)",
                  r.mounted() && Journal::half_cells == 32u);
    if (!r.mounted()) {
        return;
    }

    const Calibration cal{123456u, -321, 7};
    bench.verdict("a typed value saves", journal.save<Calibration>(id_cal, cal));
    const std::optional<Calibration> got = journal.load<Calibration>(id_cal);
    bench.verdict("and loads back equal", got && got->gain == cal.gain && got->offset == cal.offset && got->revision == cal.revision);
    bench.verdict("a load of the wrong width is refused", !journal.load<uint32_t>(id_cal).has_value());

    // Enough saves to force a collection: a half holds thirty-two cells
    // and the reserve keeps one back, so churning a second id past that
    // point ping-pongs the halves.
    bool churned = true;
    uint8_t k = 0;
    for (; k < 40u; ++k) {
        memset(churn_buf, static_cast<uint8_t>(0x40u + k), sizeof churn_buf);
        churned = churned && journal.save(id_churn, churn_buf);
    }
    print(serial, "  40 saves of a 32-byte value: free ", journal.free_cells(), "/", Journal::half_cells,
          " cells, reserve intact=", journal.reserve_intact(), crlf);
    bench.verdict("forty saves through at least one collection all succeeded", churned);
    uint8_t back[32];
    const std::optional<uint8_t> n = journal.load(id_churn, back);
    bench.verdict("the last value is the one that holds",
                  n && *n == 32u && back[0] == static_cast<uint8_t>(0x40u + 39u) && back[31] == static_cast<uint8_t>(0x40u + 39u));
    const std::optional<Calibration> still = journal.load<Calibration>(id_cal);
    bench.verdict("and the calibration survived the collection", still && still->gain == cal.gain);

    // The panic reserve: a maximum-size entry with NO erase.
    memset(churn_buf, 0xE7, sizeof churn_buf);
    const uint32_t free_before = journal.free_cells();
    const uint32_t t0 = us_now();
    const bool reserved = journal.save_reserved(id_panic, churn_buf);
    const uint32_t reserved_us = us_now() - t0;
    print(serial, "  save_reserved(): ", reserved_us, " us", crlf);
    bench.verdict("save_reserved() lands in the cell every save left behind, one page program and no erase (under 5 ms)",
                  reserved && journal.free_cells() + 1u == free_before && reserved_us < 5'000u);
    const std::optional<uint8_t> pn = journal.load(id_panic, back);
    bench.verdict("and reads back", pn && *pn == 32u && back[0] == 0xE7u);

    const auto& again = journal.mount();
    print(serial, "  re-mount: status=", static_cast<uint8_t>(again.status), " live=", again.live, " torn=", again.torn,
          " seq=", again.seq, crlf);
    bench.verdict("a re-mount holds every value with nothing torn", again.mounted() && again.live == 3u && again.torn == 0u);
}

// =============================================================================
// w - wipe the storage (outside z)
// =============================================================================
void tw_wipe() {
    const uint32_t bytes = QspiFlashPartition::storage_end - QspiFlashPartition::storage_base;
    const uint32_t t0 = us_now();
    const bool ok = Flash::erase(QspiFlashPartition::storage_base, bytes);
    const uint32_t us = us_now() - t0;
    const auto sr = Flash::status_register();
    print(serial, "  ", bytes / 1024u, " KB erased in one run (the 64 KB block command): ", us, " us; status register after: ",
          sr ? hex(*sr) : hex(uint8_t{0}), crlf);
    bench.verdict("the storage partition is erased from the floor to the top, the chip idle when the window closes",
                  ok && sr && (*sr & 0x01u) == 0u && range_is(QspiFlashPartition::storage_base, bytes, 0xFF));
}

void banner() {
    print(serial, crlf, "test_rp2040_flash - the RP2040 flash: the bootrom's functions from SRAM, the partition at the top "
                        "of the chip, nothing to wire; clk_sys=",
          SysClock::hz, " Hz, flash ", Flash::size_bytes / 1024u, " KB", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the engine and the partition: the bootrom's table, the chip's ids", ta_engine);
    bench.letter('b', "one sector, one page: erased, programmed, erased; the refusals", tb_page);
    bench.letter('c', "a second program between erases?", tc_reprogram);
    bench.letter('d', "the window (the ticks lost) and the cache (hits, flush, coherence)", td_window);
    bench.letter('e', "the heap on a page-celled media", te_heap);
    bench.letter('f', "the journal in the attic, and its reserve", tf_journal);
    bench.letter('w', "WIPE the storage partition", tw_wipe, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
