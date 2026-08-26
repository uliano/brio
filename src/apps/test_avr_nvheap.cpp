// test_avr_nvheap - the flash block allocator test SUITE for the AVR
// DA/DB target: util/nv_heap.hpp over avrdx/nvm_flash.hpp, on the real
// flash of a real part.
//
// Reference test of those headers (docs/design/nv-heap.md and the
// NvmFlash section of docs/avrdx/nvm.md): keep it passing.
//
// THIS SUITE CHANGES THE CHIP, and unlike every other suite here it
// MEANS TO: the blocks it writes are supposed to still be there at the
// next power-on and after the next reflash. That is the whole point of
// the thing under test. What it touches is the free flash outside the
// image - the hole between the code and the read-only data, and the tail
// above the read-only data - plus the two map pages under FLASHEND. One
// run of z costs about a dozen page erases out of a 1000-cycle budget.
//
// IT NEEDS ITS FUSES. Under the shipping default (BOOTSIZE = 0) the
// whole flash is one BOOT section, no software can write any of it, and
// the middle zone collapses to nothing. The standing bench geometry is
// BOOTSIZE = 128 (BOOT = the first 64 KB, where all the code is) and
// CODESIZE = 0; tools/bench.py's `fuses` verb writes it.
//
// THE SURVIVAL IT DEMONSTRATES IS A TOOL CONVENTION, NOT SILICON. Pages
// outside the image survive a reflash because bench.py flashes with
// avrdude -D, which erases only the pages it writes. A chip erase
// (bench.py flash --erase, or any UPDI erase) takes everything: flash
// has no EESAVE twin. Letter v is written to judge BOTH outcomes and to
// say which one it is looking at.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800. No pin is driven, nothing to wire,
// no timer and no event channel: the heap is polled and the flash verbs
// halt the CPU by themselves.
//
// Test d RESETS THE BOARD on purpose and carries its verdicts across the
// reset in a .noinit token, so a run still ends with one ALL: line. It
// is always run last.
//
// Commands: ? | a mount, geometry and the round trip | b supersede and
// the map rotation | c rewrite in place | d persistence across a real
// software reset | v verify the survivors (the reflash choreography)
//   -> z = a..d
// Every single-letter run also closes with an ALL: line, so bench.py
// judges any of them with its default marker.

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>
#include <optional>
#include <span>

#include "avrdx/clock.hpp"
#include "avrdx/nvm.hpp"
#include "avrdx/nvm_flash.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/reset.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/nv_heap.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

/// The suite's reset-surviving breadcrumb: test d spans a real reset and
/// its verdicts, its tallies and what it expects to find have to cross
/// it, so a run can still close with one ALL: line.
///
/// inline on purpose: gcc gives an INLINE variable with a custom section
/// attribute a COMDAT group and a plain one none, and the two section
/// types cannot be merged - the platform's own panic record is a static
/// inline member, so this must be inline too.
struct Token {
    uint16_t magic;
    uint16_t canary;
    uint8_t phase;      ///< 0 = nothing pending, 1 = test d is mid-reset
    uint8_t in_all;     ///< the run was `z`: close with the ALL: line
    uint16_t all_pass;  ///< tallies banked by the tests before this one
    uint16_t all_fail;
    uint16_t d_pass;    ///< test d's own tallies so far
    uint16_t d_fail;
    uint32_t seq;       ///< the map sequence banked before the reset
    uint32_t address;   ///< where the block was, before the reset
    uint32_t build;     ///< the build id the map carried
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using P = AvrPlatform;
using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

/// The fuse geometry this image is built for; test a checks the claim
/// against the silicon.
using Layout = FlashLayout<128, 0>;

using Heap = NvHeap<NvmFlash, 8, 2>;
Heap heap;

// The blocks this suite owns, and how long each one is when the suite is
// done with it. Letter v knows this table and nothing else: it is what
// makes "did my tables survive?" a question with an answer.
constexpr uint16_t id_small = 0x11;      ///< a: one page
constexpr uint16_t id_wide = 0x12;       ///< a: two pages, ragged tail
constexpr uint16_t id_super = 0x21;      ///< b: superseded twice
constexpr uint16_t id_rewrite = 0x31;    ///< c: rewritten in place
constexpr uint16_t id_reset = 0x41;      ///< d: written before a reset

struct Known {
    uint16_t id;
    uint32_t len;
    const char* what;
};
constexpr Known known[] = {
    {id_small, 100, "a: one page"},
    {id_wide, 601, "a: two pages, odd tail"},
    {id_super, 200, "b: superseded"},
    {id_rewrite, 150, "c: rewritten"},
    {id_reset, 64, "d: across a reset"},
};

/// Every block this suite writes is a pure function of its id and its
/// length, so any later run - or any run after a reflash - can tell
/// "the bytes I wrote" from "some bytes".
uint8_t pattern_byte(uint16_t id, uint32_t len, uint32_t i) {
    return static_cast<uint8_t>(id * 31u + len * 3u + i * 7u + (i >> 5));
}

// ---- tiny test harness -------------------------------------------------------

uint16_t passed = 0, failed = 0;
bool in_all_run = false;

void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}

/// Everything printed so far is really out of the shifter - the last
/// thing to do before a reset takes the console with it.
void console_drain() {
    while (!Serial::tx_idle()) {
    }
    delay_us(clock, 2000);
}

const char* status_name(NvHeapStatus s) {
    switch (s) {
        case NvHeapStatus::unmounted: return "unmounted";
        case NvHeapStatus::ok: return "ok";
        case NvHeapStatus::empty: return "empty";
        case NvHeapStatus::bad_geometry: return "bad_geometry";
    }
    return "?";
}

// ---- the two things every test does -----------------------------------------

/// Write a whole block: alloc, append in chunks, seal. The chunking is
/// deliberately not a multiple of the program unit - the handle buffers
/// the ragged end, and that is part of what is being tested.
bool write_block(uint16_t id, uint32_t len, bool in_place = false) {
    std::optional<Heap::Writer> w =
        in_place ? heap.rewrite(id) : heap.alloc(id, len);
    if (!w) {
        return false;
    }
    uint8_t buf[37];
    uint32_t at = 0;
    while (at < len) {
        const uint32_t n = (len - at) < sizeof buf ? (len - at) : sizeof buf;
        for (uint32_t i = 0; i < n; ++i) {
            buf[i] = pattern_byte(id, len, at + i);
        }
        if (!w->append(std::span<const uint8_t>(buf, n))) {
            return false;
        }
        at += n;
    }
    return w->seal();
}

/// Read a block back and compare it, byte for byte, with the pattern it
/// should hold.
bool block_holds(uint16_t id, uint32_t len) {
    const std::optional<NvBlock<NvmFlash>> b = heap.find(id);
    if (!b || b->length != len) {
        return false;
    }
    uint8_t buf[64];
    uint32_t at = 0;
    while (at < len) {
        const uint32_t n = (len - at) < sizeof buf ? (len - at) : sizeof buf;
        if (!b->read(at, std::span<uint8_t>(buf, n))) {
            return false;
        }
        for (uint32_t i = 0; i < n; ++i) {
            if (buf[i] != pattern_byte(id, len, at + i)) {
                return false;
            }
        }
        at += n;
    }
    return true;
}

void report_mount(const Heap::MountReport& r) {
    print(serial, "  mount ", status_name(r.status), ": seq ", r.seq,
          ", build ", hex(r.build_id), ", ", r.survivors, " survivor(s), ",
          r.lost, " lost", crlf);
    for (uint8_t i = 0; i < r.survivors; ++i) {
        print(serial, "    survived id ", hex(r.survivor_ids[i]), crlf);
    }
    for (uint8_t i = 0; i < r.lost; ++i) {
        print(serial, "    LOST     id ", hex(r.lost_ids[i]), crlf);
    }
}

// ---- a: mount, geometry, and the round trip ----------------------------------

void ta_round_trip() {
    print(serial, "a - mount, the geometry the linker left, and one block "
                  "written and read back", crlf);

    verdict("the silicon carries the fuses this image is built for",
            Layout::matches_fuses());

    const std::array<FlashZone, 2> z = NvmFlash::zones();
    print(serial, "  image ends ", hex(Nvm::image_low_end()), ", rodata ",
          hex(Nvm::rodata_load_start()), "..", hex(Nvm::rodata_load_end()),
          ", BOOT ends ", hex(Nvm::boot_end()), crlf);
    print(serial, "  middle zone ", hex(z[0].floor), "..", hex(z[0].ceiling),
          " (", z[0].size() / NvmFlash::erase_size, " pages), tail zone ",
          hex(z[1].floor), "..", hex(z[1].ceiling), " (",
          z[1].size() / NvmFlash::erase_size, " pages)", crlf);
    print(serial, "  map home ", hex(Heap::map_home), "..",
          hex(NvmFlash::flash_end), ", one version is ", Heap::map_bytes,
          " bytes of ", NvmFlash::erase_size, ", build id ",
          hex(NvmFlash::build_id()), crlf);

    verdict("the middle zone starts at the BOOT boundary, which SPM cannot "
            "cross", z[0].floor == Nvm::boot_end());
    verdict("the middle zone ends below the read-only data",
            z[0].ceiling <= Nvm::rodata_load_start());
    verdict("the tail zone starts above the read-only data",
            z[1].floor >= Nvm::rodata_load_end());
    verdict("neither zone reaches into the map home",
            z[0].floor <= Heap::map_home && z[1].floor <= Heap::map_home);

    const Heap::MountReport& r = heap.mount();
    report_mount(r);
    verdict("the heap mounts", r.mounted());
    verdict("no listed block failed its checksum", r.lost == 0);
    if (!heap.mounted()) {
        print(serial, "  the heap refused to mount: nothing else can run", crlf);
        return;
    }
    print(serial, "  free: ", heap.free_pages(0), " middle page(s), ",
          heap.free_pages(1), " tail page(s)", crlf);

    const uint32_t seq_before = heap.sequence();

    // One page, then two pages with a payload that is not a whole number
    // of program units: the handle has to pad the last word itself.
    verdict("a one-page block is written and sealed",
            write_block(id_small, 100));
    verdict("and it reads back exactly", block_holds(id_small, 100));
    verdict("a two-page block with an odd tail is written and sealed",
            write_block(id_wide, 601));
    verdict("and it reads back exactly", block_holds(id_wide, 601));

    const std::optional<NvBlock<NvmFlash>> b = heap.find(id_wide);
    if (b) {
        uint8_t tail[4];
        print(serial, "  the wide block sits at ", hex(b->address), ", ",
              b->length, " bytes", crlf);
        verdict("it starts on a page boundary",
                b->address % NvmFlash::erase_size == 0);
        verdict("the last byte of the payload is readable",
                b->read(600, std::span<uint8_t>(tail, 1)));
        verdict("and a read that would leave the block is refused, not "
                "truncated", !b->read(600, std::span<uint8_t>(tail, 2)));
    }
    verdict("the map advanced one version per seal",
            heap.sequence() == seq_before + 2);

    // A fresh mount of the same flash sees exactly the same heap.
    Heap again;
    const Heap::MountReport& r2 = again.mount();
    verdict("a fresh mount finds both blocks", r2.status == NvHeapStatus::ok &&
                                                   r2.lost == 0);
    verdict("and the sequence it picks up is the one just published",
            r2.seq == heap.sequence());
}

// ---- b: supersede and the map rotation ---------------------------------------

void tb_supersede() {
    print(serial, "b - a second block under the same id, and the map pages "
                  "taking turns", crlf);
    if (!heap.mounted()) {
        (void)heap.mount();
    }
    if (!heap.mounted()) {
        verdict("the heap is mounted", false);
        return;
    }

    verdict("the first version of the block is written",
            write_block(id_super, 80));
    verdict("and it reads back exactly", block_holds(id_super, 80));
    const std::optional<NvBlock<NvmFlash>> first = heap.find(id_super);
    const uint32_t first_addr = first ? first->address : 0;
    const uint32_t seq_before = heap.sequence();
    const uint8_t page_before = heap.map_page();

    // The old block stays current for as long as the new one is not
    // sealed - that is the whole atomicity story, and it is observable
    // from here.
    std::optional<Heap::Writer> w = heap.alloc(id_super, 200);
    verdict("a second block under the same id is allocated", w.has_value());
    if (!w) {
        return;
    }
    print(serial, "  old block at ", hex(first_addr), ", new one at ",
          hex(w->address()), crlf);
    verdict("the new block does not land on the old one",
            w->address() != first_addr);
    verdict("the old block is still the one the heap serves",
            block_holds(id_super, 80));

    uint8_t buf[40];
    for (uint32_t at = 0; at < 200; at += sizeof buf) {
        for (uint32_t i = 0; i < sizeof buf; ++i) {
            buf[i] = pattern_byte(id_super, 200, at + i);
        }
        if (!w->append(std::span<const uint8_t>(buf, sizeof buf))) {
            break;
        }
    }
    verdict("the old block is STILL the one the heap serves, with the new "
            "one fully written", block_holds(id_super, 80));
    verdict("the seal publishes the new version", w->seal());
    verdict("and now the new block is the one served",
            block_holds(id_super, 200));
    verdict("one id, one block: the old entry is gone",
            heap.find(id_super)->address != first_addr);

    print(serial, "  map seq ", seq_before, " -> ", heap.sequence(),
          ", page ", page_before, " -> ", heap.map_page(), crlf);
    verdict("the seal published exactly one new version",
            heap.sequence() == seq_before + 1);
    verdict("the version did not land on the page that held the last one",
            heap.map_page() != page_before);

    // Three more mutations, watching the rotation alternate.
    bool alternates = true;
    uint8_t page = heap.map_page();
    for (uint8_t i = 0; i < 3; ++i) {
        if (!write_block(id_super, 200)) {
            alternates = false;
            break;
        }
        if (heap.map_page() == page) {
            alternates = false;
        }
        page = heap.map_page();
    }
    verdict("every published version lands on the page the other one is not "
            "using", alternates);
    verdict("the block still reads back exactly", block_holds(id_super, 200));

    Heap again;
    const Heap::MountReport& r = again.mount();
    verdict("a fresh mount agrees with the sequence in RAM",
            r.seq == heap.sequence() && r.lost == 0);
}

// ---- c: rewrite in place ------------------------------------------------------

void tc_rewrite() {
    print(serial, "c - refilling a block where it already is", crlf);
    if (!heap.mounted()) {
        (void)heap.mount();
    }
    if (!heap.mounted()) {
        verdict("the heap is mounted", false);
        return;
    }

    verdict("a block is written to be rewritten", write_block(id_rewrite, 300));
    const std::optional<NvBlock<NvmFlash>> before = heap.find(id_rewrite);
    verdict("and it is there", before.has_value());
    if (!before) {
        return;
    }
    const uint32_t addr = before->address;

    verdict("the rewrite is accepted", write_block(id_rewrite, 150, true));
    const std::optional<NvBlock<NvmFlash>> after = heap.find(id_rewrite);
    verdict("the block is still there", after.has_value());
    if (!after) {
        return;
    }
    print(serial, "  address ", hex(addr), " -> ", hex(after->address),
          ", length 300 -> ", after->length, crlf);
    verdict("at the very same address", after->address == addr);
    verdict("with the new length", after->length == 150);
    verdict("and the new contents", block_holds(id_rewrite, 150));
    verdict("a rewrite of an id the heap does not hold is refused",
            !heap.rewrite(0x7FFF).has_value());

    Heap again;
    (void)again.mount();
    verdict("a fresh mount reads the rewritten block back",
            again.find(id_rewrite).has_value() &&
                again.find(id_rewrite)->address == addr);
}

// ---- d: across a real reset ----------------------------------------------------

constexpr uint16_t token_magic = 0x4E48;   // "NH"
constexpr uint16_t token_canary = 0xC3A5;

void td_reset() {
    print(serial, "d - a block written, the board reset, the block still "
                  "there", crlf);
    if (!heap.mounted()) {
        (void)heap.mount();
    }
    if (!heap.mounted()) {
        verdict("the heap is mounted", false);
        return;
    }

    verdict("the block is written and sealed", write_block(id_reset, 64));
    verdict("and it reads back before the reset", block_holds(id_reset, 64));
    const std::optional<NvBlock<NvmFlash>> b = heap.find(id_reset);
    if (!b) {
        return;
    }

    token.magic = token_magic;
    token.canary = token_canary;
    token.phase = 1;
    token.in_all = in_all_run ? 1 : 0;
    token.d_pass = passed;
    token.d_fail = failed;
    token.seq = heap.sequence();
    token.address = b->address;
    // The version just published carries THIS image's build id, whatever
    // the version before it carried - after a reflash the two differ,
    // which is exactly what the field is for.
    token.build = NvmFlash::build_id();

    print(serial, "  block at ", hex(b->address), ", map seq ", token.seq,
          " - resetting the board now", crlf);
    console_drain();
    Reset::software();
}

/// The other half of test d, entered from main() when the token says a
/// reset is pending.
void td_resume() {
    passed = token.d_pass;
    failed = token.d_fail;
    in_all_run = token.in_all != 0;
    print(serial, crlf, "d (continued after the reset)", crlf);

    const ResetFlags flags = Reset::take_flags();
    verdict("the boot really came from a software reset", flags.software);
    verdict("the .noinit token crossed the reset intact",
            token.canary == token_canary);

    const Heap::MountReport& r = heap.mount();
    report_mount(r);
    verdict("the heap mounts after the reset", r.status == NvHeapStatus::ok);
    verdict("nothing was lost across the reset", r.lost == 0);
    verdict("the map version is the one published before the reset",
            r.seq == token.seq);
    print(serial, "  build id: map ", hex(r.build_id), ", banked ",
          hex(token.build), ", image ", hex(NvmFlash::build_id()), crlf);
    verdict("the build id in the map is this image's",
            r.build_id == NvmFlash::build_id());
    verdict("and it is the one banked before the reset",
            r.build_id == token.build);
    verdict("the block is where it was", heap.find(id_reset).has_value() &&
                                             heap.find(id_reset)->address ==
                                                 token.address);
    verdict("and it holds exactly what was written before the reset",
            block_holds(id_reset, 64));

    token.phase = 0;
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf);
    const uint16_t tp = static_cast<uint16_t>(token.all_pass + passed);
    const uint16_t tf = static_cast<uint16_t>(token.all_fail + failed);
    print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf, crlf);
}

// ---- v: what survived ----------------------------------------------------------

void tv_verify() {
    print(serial, "v - what is still in the flash", crlf);
    const Heap::MountReport& r = heap.mount();
    report_mount(r);
    verdict("the heap mounts", r.mounted());
    if (!heap.mounted()) {
        return;
    }
    verdict("no block listed by the map failed its checksum", r.lost == 0);

    uint8_t present = 0, absent = 0, wrong = 0;
    for (const Known& k : known) {
        const std::optional<NvBlock<NvmFlash>> b = heap.find(k.id);
        if (!b) {
            ++absent;
            print(serial, "  id ", hex(k.id), " ", k.what, ": ABSENT", crlf);
            continue;
        }
        const bool ok = block_holds(k.id, k.len);
        if (ok) {
            ++present;
        } else {
            ++wrong;
        }
        print(serial, "  id ", hex(k.id), " ", k.what, ": present at ",
              hex(b->address), ", ", b->length, " bytes, contents ",
              ok ? "EXACT" : "WRONG", crlf);
    }
    verdict("every block that is present holds exactly what this suite "
            "writes", wrong == 0);

    if (present == 0 && r.status == NvHeapStatus::empty && heap.count() == 0) {
        print(serial, "  STATE: clean slate - no map at all, the heap is "
                      "empty and mountable (a chip erase went through here)",
              crlf);
        verdict("a clean slate is a coherent state: empty, mountable, ready",
                true);
        verdict("and the flash takes a block straight away",
                write_block(id_small, 100) && block_holds(id_small, 100));
    } else {
        print(serial, "  STATE: tables present - ", present, " of ",
              static_cast<uint8_t>(sizeof known / sizeof known[0]),
              " known block(s) survived, ", absent, " absent", crlf);
        verdict("the surviving tables are exactly the ones the map lists",
                present == r.survivors && present + wrong == r.survivors);
    }
}

// ---- the menu ------------------------------------------------------------------

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'a', ta_round_trip}, {'b', tb_supersede}, {'c', tc_rewrite},
    {'d', td_reset},      {'v', tv_verify},
};
constexpr char all_keys[] = "abcd";

void run(TestFn fn) {
    passed = failed = 0;
    // A lone letter banks nothing before it: test d's resume adds its own
    // tally to these two and prints the closing lines.
    token.magic = token_magic;
    token.canary = token_canary;
    token.all_pass = 0;
    token.all_fail = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf);
    // A single letter closes with an ALL: line of its own, so bench.py's
    // default marker judges any of them.
    print(serial, "ALL: ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void run_set(const char* keys) {
    uint16_t tp = 0, tf = 0;
    in_all_run = true;
    for (const char* k = keys; *k != 0; ++k) {
        for (const Test& t : tests) {
            if (t.key != *k) continue;
            token.magic = token_magic;
            token.canary = token_canary;
            token.all_pass = tp;      // banked before the test that resets
            token.all_fail = tf;
            passed = failed = 0;
            t.fn();
            print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf);
            tp = static_cast<uint16_t>(tp + passed);
            tf = static_cast<uint16_t>(tf + failed);
        }
    }
    in_all_run = false;
    print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf, crlf);
}

void help() {
    print(serial, "test_avr_nvheap: a mount, geometry and the round trip | "
                  "b supersede and the map rotation | c rewrite in place | "
                  "d persistence across a REAL software reset    -> z = a..d",
          crlf,
          "  v verify the survivors: run it after a reflash to see which "
          "tables came through (-D keeps them, --erase does not)", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    sei();

    if (token.magic == token_magic && token.phase == 1) {
        td_resume();
    } else {
        token.magic = 0;
        auto board = board_id();
        if (board.empty()) board = "?";
        print(serial, crlf, "test_avr_nvheap - flash block allocator suite "
                            "(board ", board, ", clk=",
              xtal ? "XTAL" : "OSCHF", " 24 MHz, BOOTSIZE ", FUSE.BOOTSIZE,
              "/CODESIZE ", FUSE.CODESIZE, ", build ",
              hex(NvmFlash::build_id()), ")", crlf);
        help();
    }
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 'z' || c == 'Z') { run_set(all_keys); }
        else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
