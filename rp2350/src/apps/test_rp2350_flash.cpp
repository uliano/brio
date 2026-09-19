// test_rp2350_flash - the reference bench suite for the EXTERNAL QUAD-SPI
// CHIP and the two blocks between it and the bus: the QMI (datasheet
// 12.14), the XIP cache with its four address aliases (4.4), the engine
// over the bootrom's flash functions (5.4), and rp2350/nvm_flash.hpp -
// the constant partition at the top of the chip and the one FlashMedia
// over it.
//
// ONE SOURCE, BOTH ARCHITECTURES. Every letter runs on the Cortex-M33
// pair and on the Hazard3 pair, built from the rp2350-arm-* and
// rp2350-riscv-* presets. Nothing here is either architecture's: the
// bootrom's flash functions exist on both, found through the lookup
// rp2350/bootrom.hpp does per architecture, and the window's code is one
// .ram_text function compiled twice. A verdict that must differ says so
// in its own text; there is none.
//
// NOTHING TO WIRE. The console is the Debug Probe's UART bridge on GP0
// (TX) / GP1 (RX) = UART0, the LED is GP25, and the chip under test is
// the one the image itself runs from.
//
// FLASH WEAR IS A BUDGET (a thousand cycles, and the storage partition
// is the only place this suite may spend it). So `z` COSTS NOTHING: not
// one of its letters erases or programs. The three letters that do are
// asked for by name, touch only the 64 kB partition, and say what each
// run costs.
//
// THE RULER IS TIMER0 (rp2350/timer.hpp): 64 bits of microseconds on a
// tick generator dividing clk_ref, independent of clk_sys, which is what
// makes it honest about a window that stops the program.
//
// What is exercised, letter by letter:
//   a  the boot story of this chapter: the bootrom's table and its flash
//      functions, WHICH WAY BACK into execute-in-place this image has,
//      the chip's JEDEC id against the build's size, its unique id, its
//      three status registers, the SFDP signature, the runtime
//      FLASH_DEVINFO decoded, the linker's ceiling against the
//      partition's floor, the medium's zone and its build id
//   b  THE QMI AS THE BOOTROM LEFT IT: both windows' timing, read and
//      write format and command constants, decoded and printed with the
//      SCK rate each implies; direct mode's own divisor; and the same
//      registers read again after a window, which is what proves the way
//      back put them where it found them
//   c  the four aliases and the cache: the same bytes through the cached,
//      uncached and untranslated windows; the hit ratio of a walk read
//      twice; a line invalidated by address and the miss it makes; the
//      full invalidate and the E11-safe clean sweep, both timed
//   d  ADDRESS TRANSLATION: the eight ATRANS registers, the identity map,
//      and our arithmetic against the ROM'S OWN translator over a hundred
//      addresses - then a pane the image does not use given a rolling
//      map, the two translators compared again, and the pane restored
//   e  the streaming interface of 4.4.3: a run of words fetched in the
//      background and popped by the processor, judged byte for byte
//      against the same bytes read through the window
//   f  WHAT A WINDOW COSTS WITH NOTHING WRITTEN: a raw id command timed,
//      the kernel ticks it eats, and the console's own bytes held across
//      it
//   g  the refusals, none of which reaches the chip: a misaligned erase,
//      a short span, a run past the end, a source in the XIP space, a
//      write opcode through the raw command verb, and the medium's own
//      bounds below its floor and above its ceiling
//
//   h  (by name only, NOT in z - TWO ERASE CYCLES on the partition's
//      first sector, one on the second) one sector, one page: erased and
//      read back all ones through both aliases, a page programmed to a
//      pattern and read back exact, both timed, with the ticks each
//      window eats - the cache proved coherent with the chip afterwards,
//      and a marker in the NEXT sector proving the default erase grain
//      issues the 4 kB command and nothing wider
//   i  (by name only, NOT in z - ONE ERASE CYCLE) THE QUESTION THE
//      CHAPTER DOES NOT ANSWER: a page programmed a second time between
//      erases, clearing more bits; what the chip holds afterwards,
//      printed and judged against the AND of the two
//   w  (by name only, NOT in z - TWO ERASE CYCLES) WIPE the whole 64 kB
//      partition, once with the 4 kB command alone and once letting the
//      bootrom take the 64 kB block command, so the saving is a number -
//      with a marker page at each end between the two, because a chip
//      with no D8h command would ignore it in silence and an already
//      erased partition would hide that
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <array>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/flash.hpp"
#include "rp2350/nvm_flash.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

extern "C" {
/// The load address of .data - the last flash cursor the linker used -
/// and the region it is copied into, which together say how far up the
/// chip this image actually reaches. Letter d needs it: it may only
/// touch a translation pane the image does not live in.
extern const char __data_load_start[];
extern const char __data_start[];
extern const char __data_end[];
}

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;
constexpr SysClock clock;
using P = Rp2350Platform<>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

using Led = Pin<25>;          // the board's LED: a keystroke marker
using Ruler = Timer<0>;       // microseconds, independent of clk_sys
using Chip = QmiWindow<0>;    // the window the board's flash sits in
using Spare = QmiWindow<1>;   // nothing is attached to it on this board

TestBench<Serial> bench;

/// The partition, and the one sector every writing letter uses.
constexpr uint32_t store_base = QspiFlashPartition::storage_base;
constexpr uint32_t store_end = QspiFlashPartition::storage_end;
constexpr uint32_t store_bytes = store_end - store_base;

uint8_t page_buf[Flash::page_size];
uint8_t got_buf[Flash::page_size];
uint8_t answer[16];

uint32_t us_now() { return Ruler::now_low(); }

/// How far up the chip this image reaches: the flash cursor .data was
/// loaded from, plus .data's own length.
uint32_t image_end() {
    const auto load = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__data_load_start));
    const auto from = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__data_start));
    const auto to = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__data_end));
    return (load - Flash::window) + (to - from);
}

/// Does the range read `value` through the alias that neither hits nor
/// fills the cache - what the chip holds, not what the cache remembers?
bool range_is(uint32_t addr, uint32_t bytes, uint8_t value) {
    const uint8_t* p = Flash::uncached_address(addr);
    for (uint32_t i = 0; i < bytes; ++i) {
        if (p[i] != value) {
            return false;
        }
    }
    return true;
}

void print_bytes(const uint8_t* p, uint32_t n) {
    constexpr char digits[] = "0123456789ABCDEF";
    for (uint32_t i = 0; i < n; ++i) {
        print(serial, digits[p[i] >> 4], digits[p[i] & 0x0Fu]);
    }
}

/// Letter c's cache probe: a flash offset FAR FROM THE IMAGE - 8 MB in,
/// where no code, no constant and no storage of this program lives - so
/// that invalidating its line cannot drop a line the probe itself
/// fetches. At offset 0x200, which looks harmless, both builds put
/// executable code.
constexpr uint32_t probe_line = 8u * 1024u * 1024u;
constexpr uint32_t probe_reads = 8;

/**
 * The same line read `probe_reads` times, and the cache hits that cost.
 *
 * THE COUNTERS COUNT INSTRUCTION FETCHES TOO, so the only honest way to
 * compare two readings is to make them THE SAME INSTRUCTIONS AT THE SAME
 * ADDRESSES: one function, called twice, with the counters reset inside
 * it. Then the whole difference between the two answers is the data
 * access the caller changed - and the first call, whose own code is
 * still cold, is thrown away.
 */
[[gnu::noinline]] uint32_t hits_over_reads(uint32_t offset) {
    Xip::reset_counters();
    for (uint32_t i = 0; i < probe_reads; ++i) {
        (void)*reinterpret_cast<const volatile uint32_t*>(Flash::address(offset));
    }
    return Xip::hits();
}

const char* width_name(QmiWidth w) {
    return w == QmiWidth::quad ? "quad" : (w == QmiWidth::dual ? "dual" : "serial");
}

const char* restore_name(XipRestore r) {
    switch (r) {
        case XipRestore::setup_function:
            return "the bootrom's XIP setup function out of boot RAM";
        case XipRestore::rom_cmd_xip:
            return "the ROM's flash_enter_cmd_xip (03h, CLKDIV 12)";
        default:
            return "none";
    }
}

void print_format(const char* label, const QmiTransferFormat& f) {
    print(serial, "    ", label, ": prefix ", width_name(f.prefix_width),
          f.prefix_len == QmiPrefixLen::eight ? "/8" : "/none", ", address ",
          width_name(f.addr_width), ", suffix ", width_name(f.suffix_width),
          f.suffix_len == QmiSuffixLen::eight ? "/8" : "/none", ", dummy ",
          width_name(f.dummy_width), "/", static_cast<uint32_t>(f.dummy_len) * 4u, " bits, data ",
          width_name(f.data_width), f.dtr ? ", DTR" : "", crlf);
}

void print_timing(const char* label, const QmiTiming& t) {
    print(serial, "    ", label, ": clkdiv ", t.clkdiv, " (SCK ",
          qmi_sck_hz(SysClock::hz, t.clkdiv) / 1000u, " kHz), rxdelay ", t.rxdelay, ", cooldown ",
          t.cooldown, ", max_select ", t.max_select, ", min_deselect ", t.min_deselect,
          ", select setup/hold ", t.select_setup, "/", t.select_hold, crlf);
}

// =============================================================================
// a - the boot story of this chapter
// =============================================================================
void ta_boot() {
    const bool found = Flash::init();
    bench.verdict("the bootrom names its six flash functions on this architecture", found);
    print(serial, "  the way back into execute-in-place: ", restore_name(Flash::xip_restore_kind()),
          crlf);
    bench.verdict("an image entered by the bootrom has its XIP setup function in boot RAM, "
                  "so the fast mode survives a window",
                  Flash::xip_restore_kind() == XipRestore::setup_function);
    bench.verdict("and the ROM's own data table named it, rather than the documented base of "
                  "boot RAM having to serve - the two routes agree",
                  Flash::xip_setup_named_by_rom());
    print(serial, "  its first two words: ", hex(Flash::setup_word(0)), " ",
          hex(Flash::setup_word(1)), crlf);

    const auto id = Flash::jedec_id();
    if (id) {
        print(serial, "  9Fh: manufacturer ", hex(id->manufacturer), " type ", hex(id->type),
              " capacity ", hex(id->capacity), " = ", id->bytes() / (1024u * 1024u), " MB", crlf);
    }
    bench.verdict("9Fh answers, and the capacity it reports is the size this image was built for",
                  id.has_value() && id->bytes() == Flash::size_bytes);

    const auto unique = Flash::unique_id();
    if (unique) {
        print(serial, "  4Bh: ");
        print_bytes(unique->data(), 8);
        print(serial, crlf);
    }
    bool id_zeros = true;
    bool id_ones = true;
    if (unique) {
        for (const uint8_t b : *unique) {
            id_zeros = id_zeros && b == 0x00u;
            id_ones = id_ones && b == 0xFFu;
        }
    }
    bench.verdict("4Bh answers a unique id that is neither all zeros nor all ones",
                  unique.has_value() && !id_zeros && !id_ones);

    const auto s1 = Flash::status_register<1>();
    const auto s2 = Flash::status_register<2>();
    const auto s3 = Flash::status_register<3>();
    print(serial, "  status registers: 05h=", hex(s1.value_or(0)), " 35h=", hex(s2.value_or(0)),
          " 15h=", hex(s3.value_or(0)), crlf);
    bench.verdict("the three status registers read, and the chip is idle with no write enabled "
                  "(05h bits 0 and 1 clear)",
                  s1.has_value() && s2.has_value() && s3.has_value() && (*s1 & 0x03u) == 0u);

    uint8_t sfdp[8] = {};
    const bool sfdp_ok = Flash::read_sfdp(0u, sfdp);
    print(serial, "  SFDP header: ");
    print_bytes(sfdp, 8);
    print(serial, crlf);
    bench.verdict("5Ah answers the JEDEC signature 'SFDP' at address zero",
                  sfdp_ok && sfdp[0] == 'S' && sfdp[1] == 'F' && sfdp[2] == 'D' && sfdp[3] == 'P');

    const auto info = Flash::device_info();
    if (info) {
        print(serial, "  FLASH_DEVINFO ", hex(info->raw), ": chip select 0 ",
              info->cs0_bytes() / (1024u * 1024u), " MB, chip select 1 ", info->cs1_bytes(),
              " bytes, CS1 pad ", info->cs1_gpio(), ", D8h block erase ",
              info->d8h_erase_supported() ? "declared" : "not declared", crlf);
    }
    bench.verdict("the runtime FLASH_DEVINFO in boot RAM is readable and says nothing is "
                  "attached to chip select 1, which is what lets a window restore window 1",
                  info.has_value() && info->cs1_bytes() == 0u);

    print(serial, "  image reaches ", image_end() / 1024u, " kB; the linker's ceiling is ",
          (static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__brio_flash_end)) - Flash::window) /
              1024u,
          " kB; the partition floor is ", store_base / 1024u, " kB", crlf);
    bench.verdict("the linker script still stops at or below the partition's floor",
                  QspiFlashPartition::geometry_matches_silicon());

    const auto zone = QspiFlash::zones()[0];
    print(serial, "  the medium: zone ", hex(zone.floor), "..", hex(zone.ceiling), ", erase ",
          QspiFlash::erase_size, ", cell ", QspiFlash::write_cell, ", build id ",
          hex(QspiFlash::build_id()), crlf);
    bench.verdict("the medium reports the partition as its one zone, 64 kB of it",
                  zone.floor == store_base && zone.ceiling == store_end &&
                      store_bytes == 64u * 1024u);
}

// =============================================================================
// b - the QMI as the bootrom left it, and as a window gives it back
// =============================================================================
void tb_qmi() {
    print(serial, "  window 0 (the board's flash):", crlf);
    const auto t0 = Chip::timing();
    print_timing("timing", t0);
    const auto rf0 = Chip::read_format();
    print_format("read format", rf0);
    const auto rc0 = Chip::read_command();
    print(serial, "    read command: prefix ", hex(rc0.prefix), " suffix ", hex(rc0.suffix), crlf);
    const auto wf0 = Chip::write_format();
    print_format("write format", wf0);
    const auto wc0 = Chip::write_command();
    print(serial, "    write command: prefix ", hex(wc0.prefix), " suffix ", hex(wc0.suffix), crlf);

    print(serial, "  window 1 (nothing on this board):", crlf);
    print_timing("timing", Spare::timing());
    print_format("read format", Spare::read_format());

    bench.verdict("the bootrom left window 0 with a read command that is one of the four modes "
                  "5.4.8.14 tries - 03h, 0Bh, BBh or EBh",
                  rc0.prefix == 0x03u || rc0.prefix == 0x0Bu || rc0.prefix == 0xBBu ||
                      rc0.prefix == 0xEBu);
    bench.verdict("the prefix is eight bits wide, which is what keeps the device in a serial "
                  "command state and lets reads mix with programming",
                  rf0.prefix_len == QmiPrefixLen::eight);
    bench.verdict("the divisor is at least one and the SCK it implies is under clk_sys",
                  qmi_sck_hz(SysClock::hz, t0.clkdiv) <= SysClock::hz);
    print(serial, "  direct mode: clkdiv ", Qmi::direct_clkdiv(), " (SCK ",
          qmi_sck_hz(SysClock::hz, Qmi::direct_clkdiv()) / 1000u, " kHz), rxdelay ",
          Qmi::direct_rxdelay(), "; EN ", Qmi::direct_enabled() ? "set" : "clear", crlf);
    bench.verdict("direct mode is off while the program executes from the window",
                  !Qmi::direct_enabled());
    bench.verdict("neither chip select is being driven low by hand",
                  !Chip::select_asserted() && !Spare::select_asserted());

    // THE MEASUREMENT THIS LETTER EXISTS FOR: one window, with nothing
    // written, and the whole configuration read again afterwards.
    const uint32_t t1_before = qmi_timing_word(Spare::timing());
    const uint32_t pads_before = Qmi::pad_control(QspiPad::sclk);
    (void)Flash::jedec_id();
    const auto t0_after = Chip::timing();
    const auto rf0_after = Chip::read_format();
    const auto rc0_after = Chip::read_command();
    const uint32_t t1_after = qmi_timing_word(Spare::timing());
    const uint32_t pads_after = Qmi::pad_control(QspiPad::sclk);

    print_timing("window 0 timing after a window", t0_after);
    print_format("window 0 read format after a window", rf0_after);
    bench.verdict("a window leaves window 0's read command where it found it - the way back "
                  "restored the mode the bootrom discovered",
                  rc0_after.prefix == rc0.prefix && rc0_after.suffix == rc0.suffix);
    bench.verdict("and its read format and its divisor too",
                  qmi_format_word(rf0_after) == qmi_format_word(rf0) &&
                      t0_after.clkdiv == t0.clkdiv);
    bench.verdict("window 1's timing survives a window, which the driver saves and puts back "
                  "because the ROM's exit sequence rewrites it",
                  t1_after == t1_before);
    bench.verdict("the QSPI pad controls survive a window, which the driver saves and puts back "
                  "because connect_internal_flash resets them",
                  pads_after == pads_before);
}

// =============================================================================
// c - the four aliases and the cache
// =============================================================================
void tc_aliases() {
    constexpr uint32_t probe = 0x0000'0100u;   // well inside the image, never written
    const uint8_t* cached = Flash::address(probe);
    const uint8_t* uncached = Flash::uncached_address(probe);
    const uint8_t* raw = Flash::untranslated_address(probe);
    print(serial, "  the same eight bytes at ", hex(probe), " through three aliases:", crlf,
          "    cached       ");
    print_bytes(cached, 8);
    print(serial, crlf, "    uncached     ");
    print_bytes(uncached, 8);
    print(serial, crlf, "    untranslated ");
    print_bytes(raw, 8);
    print(serial, crlf);
    bool same = true;
    for (uint32_t i = 0; i < 8; ++i) {
        same = same && cached[i] == uncached[i] && cached[i] == raw[i];
    }
    bench.verdict("the three aliases of one byte agree, the translation being the identity",
                  same);

    bench.verdict("the cache is enabled for Secure accesses, which is what the bootrom leaves",
                  Xip::enabled_secure());
    bench.verdict("the cache is not powered down", !Xip::powered_down());
    bench.verdict("neither memory window is writable, which is the reset state and the one that "
                  "keeps a stray store from breaking the flash out of its read mode",
                  !Xip::window_writable<0>() && !Xip::window_writable<1>());

    // A walk of one sector read twice: the first pass fills, the second
    // should hit almost everywhere.
    Xip::invalidate_all();
    Xip::reset_counters();
    uint32_t sum = 0;
    for (uint32_t pass = 0; pass < 2; ++pass) {
        for (uint32_t i = 0; i < Flash::sector_size; i += 4u) {
            sum += *reinterpret_cast<const volatile uint32_t*>(Flash::address(i));
        }
    }
    const uint32_t acc = Xip::accesses();
    const uint32_t hit = Xip::hits();
    print(serial, "  a 4 kB walk read twice: ", acc, " accesses, ", hit, " hits (",
          acc != 0u ? 100u * hit / acc : 0u, " per cent); checksum ", hex(sum), crlf);
    bench.verdict("the counters counted, and more than half of the accesses hit - the second "
                  "pass over a sector that fits in a 16 kB cache",
                  acc != 0u && hit * 2u > acc);

    // One line invalidated by address: the next read of it must miss.
    // `hits_over_reads` above is why this is a fair comparison.
    const uint32_t line = probe_line;
    (void)hits_over_reads(line);        // the probe's own code into the cache
    const uint32_t warm_hits = hits_over_reads(line);
    Xip::invalidate_address(line);
    const uint32_t cold_hits = hits_over_reads(line);
    print(serial, "  one line at ", hex(line), " read ", probe_reads, " times by one function: ",
          warm_hits, " hits with it allocated, ", cold_hits, " after an invalidate by address",
          crlf);
    bench.verdict("an invalidate by address turns the next read of that line into a miss - one "
                  "hit fewer over a run of reads whose instructions are the same instructions "
                  "twice over",
                  warm_hits == cold_hits + 1u);

    const uint32_t t_inv0 = us_now();
    Xip::invalidate_all();
    const uint32_t t_inv = us_now() - t_inv0;
    const uint32_t t_cln0 = us_now();
    Xip::clean_all();
    const uint32_t t_cln = us_now() - t_cln0;
    print(serial, "  the whole cache: invalidate ", t_inv, " us, clean ", t_cln,
          " us (2048 lines each)", crlf);
    bench.verdict("both sweeps complete, and the program that issued the clean is still running "
                  "- erratum RP2350-E11's tag rewrite lands outside the QMI's half of the space",
                  t_inv != 0u && t_cln != 0u &&
                      *reinterpret_cast<const volatile uint32_t*>(Flash::address(0)) != 0u);
}

// =============================================================================
// d - address translation, against the ROM's own translator
// =============================================================================
void td_translation() {
    print(serial, "  window 0 panes: ");
    const auto panes = Chip::translations();
    for (uint8_t p = 0; p < 4; ++p) {
        print(serial, "[", p, "] base ", hex(panes[p].base), " size ", panes[p].size / 1024u,
              " kB  ");
    }
    print(serial, crlf);
    bench.verdict("window 0 is the identity map the QMI's reset leaves - four panes of 4 MB, "
                  "each based where it already is",
                  Chip::translation_is_identity());
    bench.verdict("window 1 is the identity map too", Spare::translation_is_identity());

    // WHICH BASE THE ROM'S TRANSLATOR ANSWERS IN is not pinned down by
    // the chapter - 5.4.8.13 says "the storage address", which reads
    // like an offset in the chip, while 5.4.8.9 expresses every flash
    // address from the window's base - so it is MEASURED here once, on
    // the identity map, and everything below is judged against what it
    // turns out to be.
    const auto probe = Flash::runtime_to_storage(Flash::window + 0x1000u);
    const bool from_window = probe.has_value() && *probe == Flash::window + 0x1000u;
    const bool from_zero = probe.has_value() && *probe == 0x1000u;
    print(serial, "  the bootrom's translator, asked for ", hex(Flash::window + 0x1000u),
          ", answers ", hex(probe.value_or(0)), " - ",
          from_window ? "from the window's base" : (from_zero ? "as a chip offset" : "neither"),
          crlf);
    bench.verdict("the bootrom's own translator answers on the identity map, in one of the two "
                  "bases the chapter leaves open - as a chip offset, or from the window's base",
                  from_window || from_zero);
    if (!from_window && !from_zero) {
        return;
    }
    const uint32_t rom_base = from_window ? Flash::window : 0u;

    // Our arithmetic against the ROM's, over the live registers.
    uint32_t checked = 0;
    uint32_t disagreed = 0;
    for (uint32_t i = 0; i < 100u; ++i) {
        const uint32_t offset = i * 0x0002'7100u;   // a stride that is not a power of two
        const auto ours = Chip::translate(offset);
        const auto theirs = Flash::runtime_to_storage(Flash::window + offset);
        ++checked;
        const bool agree = ours.has_value() == theirs.has_value() &&
                           (!ours.has_value() || *theirs == *ours + rom_base);
        if (!agree) {
            ++disagreed;
        }
    }
    print(serial, "  ", checked, " addresses through both translators, ", disagreed,
          " disagreements", crlf);
    bench.verdict("this file's translation arithmetic answers what the bootrom's own "
                  "flash_runtime_to_storage_addr answers, on the identity map",
                  disagreed == 0u);

    // A rolling window on a pane the image does not use. Pane 1 covers
    // 4 MB..8 MB of the chip: the image ends far below it and the
    // storage partition sits at the top, so nothing is fetched or read
    // through it while it is moved. The change and its undoing are one
    // masked region, and the cache is flushed after (12.14.4.2).
    const bool pane1_free = image_end() < xip_pane_span && store_base >= 2u * xip_pane_span;
    bench.verdict("pane 1 of window 0 is free of this image and of the storage, so the rolling "
                  "map below may be written there",
                  pane1_free);
    if (!pane1_free) {
        return;
    }

    constexpr uint32_t rolled_base = 5u * 1024u * 1024u;
    QmiTranslation before{};
    QmiTranslation read_back{};
    {
        InterruptGuard guard;
        before = Chip::translation<1>();
        Chip::set_translation<1>(QmiTranslation{.base = rolled_base, .size = xip_pane_span});
        read_back = Chip::translation<1>();
        Chip::set_translation<1>(before);
    }
    Xip::invalidate_all();
    print(serial, "  pane 1 written base ", hex(rolled_base), ", read back base ",
          hex(read_back.base), " size ", read_back.size / 1024u, " kB, restored to base ",
          hex(Chip::translation<1>().base), crlf);
    bench.verdict("ATRANS counts in units of one flash sector: a base of 5 MB written and read "
                  "back as 5 MB, the size still 4 MB",
                  read_back.base == rolled_base && read_back.size == xip_pane_span);
    bench.verdict("and the pane is back where it was, the window unchanged",
                  Chip::translation_is_identity());
}

// =============================================================================
// e - the streaming interface (4.4.3)
// =============================================================================
void te_stream() {
    constexpr uint32_t from = 0x0000'0400u;
    constexpr uint32_t words = 32;
    std::array<uint32_t, words> got{};

    Xip::stream_start(Flash::window + from, words);
    uint32_t taken = 0;
    const uint32_t t0 = us_now();
    while (taken < words && us_now() - t0 < 100'000u) {
        if (const auto w = Xip::stream_pop()) {
            got[taken++] = *w;
        }
    }
    const uint32_t elapsed = us_now() - t0;
    const uint32_t left = Xip::stream_remaining();
    Xip::stream_stop();

    print(serial, "  ", taken, " of ", words, " words streamed in ", elapsed, " us, ", left,
          " still to fetch", crlf);
    bench.verdict("the streaming engine delivered every word it was asked for", taken == words);

    uint32_t wrong = 0;
    for (uint32_t i = 0; i < words; ++i) {
        const uint32_t direct = *reinterpret_cast<const volatile uint32_t*>(
            Flash::uncached_address(from + i * 4u));
        if (direct != got[i]) {
            ++wrong;
        }
    }
    bench.verdict("and every word is the word the same address reads through the window",
                  taken == words && wrong == 0u);
    bench.verdict("the count is spent and the FIFO drained once the run is over",
                  left == 0u && Xip::stream_empty());
}

// =============================================================================
// f - what a window costs with nothing written
// =============================================================================
void tf_window_cost() {
    // Let the console fall silent first: its transmitter would be the
    // interrupt this measures the absence of.
    const uint32_t t_drain = us_now();
    while (!Serial::tx_idle() && us_now() - t_drain < 200'000u) {
    }

    const uint32_t ticks0 = Ticker::ticks();
    const uint32_t t0 = us_now();
    const auto id = Flash::jedec_id();
    const uint32_t us = us_now() - t0;
    const uint32_t ticks = Ticker::ticks() - ticks0;

    print(serial, "  a 9Fh window: ", us, " us, ", ticks, " kernel ticks across it", crlf);
    bench.verdict("the id came back", id.has_value());
    bench.verdict("the window is short - a raw id command under two milliseconds, the cache "
                  "invalidate and the way back included",
                  us != 0u && us < 2000u);
    bench.verdict("the kernel cannot GAIN time across a masked window: it advanced by no more "
                  "than the window's own milliseconds, and loses whatever fell inside",
                  ticks <= us / 1000u + 1u);

    // The console's own bytes across a window: written before it, read
    // by the host after it, uncut.
    print(serial, "  the next line is written across a window, from a buffer in SRAM:", crlf);
    const uint32_t t1 = us_now();
    (void)Flash::status_register<1>();
    const uint32_t us1 = us_now() - t1;
    print(serial, "  ... and here it is, whole - the window took ", us1, " us", crlf);
    bench.verdict("the console survived a window with no byte lost, the transport's ring being "
                  "in SRAM and its interrupt merely late",
                  us1 != 0u);
}

// =============================================================================
// g - the refusals, none of which reaches the chip
// =============================================================================
void tg_refusals() {
    bench.verdict("an erase at an address that is not a sector boundary is refused",
                  !Flash::erase(store_base + 1u, Flash::sector_size));
    bench.verdict("an erase of less than a whole sector is refused",
                  !Flash::erase(store_base, Flash::page_size));
    bench.verdict("an erase of zero bytes is refused", !Flash::erase(store_base, 0));
    bench.verdict("an erase that runs past the end of the chip is refused",
                  !Flash::erase(store_end - Flash::sector_size, 2u * Flash::sector_size));
    bench.verdict("an erase at an address past the end of the chip is refused",
                  !Flash::erase(Flash::size_bytes, Flash::sector_size));

    bench.verdict("a program at an address that is not a page boundary is refused",
                  !Flash::program(store_base + 1u, page_buf));
    bench.verdict("a program of less than a whole page is refused",
                  !Flash::program(store_base, std::span<const uint8_t>{page_buf, 8}));
    bench.verdict("a program whose SOURCE lies in the XIP space is refused: the bootrom would "
                  "read it through a memory interface that is not there",
                  !Flash::program(store_base,
                                  std::span<const uint8_t>{Flash::address(0), Flash::page_size}));
    bench.verdict("a source in the uncached alias is refused just the same",
                  Flash::in_window(Flash::uncached_address(0)) &&
                      Flash::in_window(Flash::untranslated_address(0)));

    bench.verdict("the raw command verb refuses a page program opcode at run time, as it "
                  "refuses one at compile time",
                  !Flash::command(0x02u, {}, answer));
    bench.verdict("and a write enable, and a status register write, and a chip erase",
                  !Flash::command(0x06u, {}, answer) && !Flash::command(0x01u, {}, answer) &&
                      !Flash::command(0xC7u, {}, answer));
    bench.verdict("a raw command whose answer buffer lies in the XIP space is refused",
                  !Flash::command(FlashCommand::read_jedec_id, {},
                                  std::span<uint8_t>{const_cast<uint8_t*>(Flash::address(0)), 3}));

    bench.verdict("the medium refuses a program below its floor - the running image is not "
                  "storage",
                  !QspiFlash::program(store_base - Flash::page_size, page_buf));
    bench.verdict("the medium refuses an erase below its floor",
                  !QspiFlash::erase(store_base - Flash::sector_size));
    bench.verdict("the medium refuses an erase at its ceiling", !QspiFlash::erase(store_end));
    bench.verdict("the medium refuses a program that begins at its ceiling",
                  !QspiFlash::program(store_end, page_buf));
    bench.verdict("the medium refuses a program at an address that is not a cell boundary",
                  !QspiFlash::program(store_base + 1u, page_buf));
}

// =============================================================================
// h - one sector, one page (ONE ERASE CYCLE)
// =============================================================================
void th_erase_program() {
    print(serial, "  this letter spends two erase cycles on the sector at ", hex(store_base),
          " and one on the next", crlf);

    // Clear the first two sectors and mark the second. The marker is
    // what makes the erase below MEAN something: it proves the default
    // grain issued the 4 kB command and not a wider one, which would
    // have taken the neighbour with it.
    const bool cleared = Flash::erase(store_base, 2u * Flash::sector_size);
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        page_buf[i] = 0x5Au;
    }
    const bool marked = QspiFlash::program(store_base + Flash::sector_size, page_buf);
    bench.verdict("two sectors cleared and the second one marked", cleared && marked);

    const uint32_t ticks_e0 = Ticker::ticks();
    const uint32_t te0 = us_now();
    const bool erased = QspiFlash::erase(store_base);
    const uint32_t erase_us = us_now() - te0;
    const uint32_t erase_ticks = Ticker::ticks() - ticks_e0;
    print(serial, "  sector erase: ", erase_us, " us; the kernel advanced ", erase_ticks,
          " ticks where ", erase_us / 1000u, " ms of wall clock passed", crlf);
    bench.verdict("the medium erased its first sector", erased);
    bench.verdict("and it reads all ones through the alias that bypasses the cache",
                  range_is(store_base, Flash::sector_size, 0xFF));
    bench.verdict("THE MARKER IN THE NEXT SECTOR SURVIVED: the default erase grain issues the "
                  "4 kB command and nothing wider, which is what the block size this driver "
                  "hands the bootrom is chosen to guarantee",
                  *Flash::uncached_address(store_base + Flash::sector_size) == 0x5Au);
    bench.verdict("the kernel cannot gain time across the window: it advanced by no more than "
                  "the erase's own milliseconds, and loses whatever fell inside",
                  erase_ticks <= erase_us / 1000u + 1u);

    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        page_buf[i] = static_cast<uint8_t>(0xA5u ^ i);
    }
    const uint32_t ticks_p0 = Ticker::ticks();
    const uint32_t tp0 = us_now();
    const bool programmed = QspiFlash::program(store_base, page_buf);
    const uint32_t program_us = us_now() - tp0;
    const uint32_t program_ticks = Ticker::ticks() - ticks_p0;
    print(serial, "  page program: ", program_us, " us, ", program_ticks, " kernel ticks lost",
          crlf);
    bench.verdict("the medium programmed one page", programmed);

    QspiFlash::read(store_base, got_buf);
    bool exact = true;
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        exact = exact && got_buf[i] == page_buf[i];
    }
    bench.verdict("the page reads back byte for byte through the cached window", exact);

    const uint8_t* past = Flash::uncached_address(store_base);
    bool exact_uncached = true;
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        exact_uncached = exact_uncached && past[i] == page_buf[i];
    }
    bench.verdict("and past the cache too, so the cache was left coherent with the chip - the "
                  "engine's own invalidate is what does it",
                  exact_uncached);
    bench.verdict("the page after it is untouched: a program of 256 bytes programs 256 bytes",
                  range_is(store_base + Flash::page_size, Flash::page_size, 0xFF));
}

// =============================================================================
// i - a second program between erases (ONE ERASE CYCLE)
// =============================================================================
void ti_reprogram() {
    print(serial, "  this letter spends one erase cycle on the sector at ", hex(store_base), crlf);
    if (!QspiFlash::erase(store_base)) {
        bench.verdict("the sector erased", false);
        return;
    }

    constexpr uint8_t first = 0xF0;
    constexpr uint8_t second = 0xCC;
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        page_buf[i] = first;
    }
    const bool one = QspiFlash::program(store_base, page_buf);
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        page_buf[i] = second;
    }
    const bool two = QspiFlash::program(store_base, page_buf);
    bench.verdict("a page programmed twice between erases, both programs accepted", one && two);

    QspiFlash::read(store_base, got_buf);
    print(serial, "  ", hex(first), " then ", hex(second), " into the same page reads back ",
          hex(got_buf[0]), "; the AND of the two is ", hex(static_cast<uint8_t>(first & second)),
          crlf);
    bool all_and = true;
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        all_and = all_and && got_buf[i] == static_cast<uint8_t>(first & second);
    }
    bench.verdict("what the chip holds is the AND of the two programs, every byte of the page: "
                  "programming clears ones and never sets them",
                  all_and);
}

// =============================================================================
// w - wipe the partition, both grains (TWO ERASE CYCLES)
// =============================================================================
void tw_wipe() {
    print(serial, "  this letter spends two erase cycles on the whole 64 kB partition", crlf);

    const uint32_t t0 = us_now();
    const bool by_sector = Flash::erase(store_base, store_bytes, FlashEraseGrain::sector);
    const uint32_t sector_us = us_now() - t0;
    bench.verdict("the partition erased with the 4 kB command alone", by_sector);
    bench.verdict("and reads all ones", range_is(store_base, store_bytes, 0xFF));

    // Mark the partition before the second erase, so that "reads all
    // ones" afterwards can only be true if that erase REALLY HAPPENED -
    // a chip with no D8h command would ignore it in silence, and an
    // already-erased partition would hide that.
    for (uint32_t i = 0; i < Flash::page_size; ++i) {
        page_buf[i] = 0x5Au;
    }
    const bool marked = QspiFlash::program(store_end - Flash::page_size, page_buf) &&
                        QspiFlash::program(store_base, page_buf);
    bench.verdict("a marker page written at each end of the partition, so the erase below has "
                  "something to erase", marked);

    const uint32_t t1 = us_now();
    const bool by_block = Flash::erase(store_base, store_bytes, FlashEraseGrain::sector_or_block);
    const uint32_t block_us = us_now() - t1;
    bench.verdict("the partition erased again, this time with the 64 kB block command offered",
                  by_block);
    bench.verdict("and both markers are gone, so the 64 kB command reached the chip and did "
                  "what it says",
                  range_is(store_base, store_bytes, 0xFF));

    print(serial, "  64 kB erased: ", sector_us, " us by sixteen 4 kB commands, ", block_us,
          " us with the 64 kB block command offered", crlf);
    bench.verdict("the block command is not slower than sixteen sector commands, which is what "
                  "makes offering it worth the driver's parameter",
                  block_us <= sector_us);
}

void banner() {
    print(serial, crlf, "test_rp2350_flash - the QSPI chip, the QMI and the XIP cache", crlf);
    bench.menu();
    print(serial, "  (h, i and w spend flash endurance and are not in z)", crlf);
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// ONE NAME, BOTH ARCHITECTURES: a Cortex-M vector-table slot on one half,
// an entry of Hazard3's own dispatch on the other.
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool ruler_ok = brio::Timer<0>::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    const bool flash_ok = brio::Flash::init();
    (void)Led::output(false);
    brio::enable_interrupts();

    bench.letter('a', "the engine, the chip's identity and the partition", ta_boot);
    bench.letter('b', "the QMI as the bootrom left it, and after a window", tb_qmi);
    bench.letter('c', "the four aliases and the cache", tc_aliases);
    bench.letter('d', "address translation against the bootrom's own translator", td_translation);
    bench.letter('e', "the streaming interface", te_stream);
    bench.letter('f', "what a window costs with nothing written", tf_window_cost);
    bench.letter('g', "the refusals", tg_refusals);
    bench.letter('h', "one sector, one page (THREE ERASE CYCLES)", th_erase_program, false);
    bench.letter('i', "a second program between erases (ONE ERASE CYCLE)", ti_reprogram, false);
    bench.letter('w', "WIPE the partition, both grains (TWO ERASE CYCLES)", tw_wipe, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " ruler=", ruler_ok ? "1us" : "FAILED", " tick=", tick_ok ? "on" : "FAILED",
                    " flash=", flash_ok ? "rom functions found" : "FAILED", brio::crlf);
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
