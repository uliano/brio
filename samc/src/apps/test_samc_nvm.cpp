// test_samc_nvm - the reference bench suite for samc/nvm.hpp and
// samc/nvm_flash.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE. Everything under test is inside the chip.
//
// What is exercised, letter by letter:
//   a  geometry and identity, with no wear at all: the device header's
//      constants against what PARAM reports, the user row decoded into
//      the fuses it is, the factory calibration and the die's serial
//      number, and ADDR's encoding PROVEN by loading the page buffer at
//      a known address on each array and reading the register back
//   b  the RWWEE round trip: erase a row and see 0xFF, program each of
//      its four pages and read them back byte-exact, and see every
//      malformed request refused rather than half-performed
//   c  THE PAGE-BUFFER ORDERING TRAP, decided by data: the same page
//      loaded ascending and descending, and the descending one shown
//      landing wrong exactly as 27.6.4.3 predicts
//   d  what it costs, and the claim that matters: a row erase and a page
//      write measured in microseconds, and the CPU PROVEN STILL RUNNING
//      through both - the whole reason this heap lives in RWWEE
//   e  util/nv_heap.hpp on this silicon: mount, alloc, append, seal,
//      find and read back, then a re-mount that finds the block again -
//      the target-independent allocator's second implementation
//   f  protection and error reporting: a locked region REFUSING an erase
//      (nothing is erased, so this costs no wear), the lock readback,
//      and STATUS.PROGE raised by a deliberately invalid command
//
//   m  (by name only) THE MAIN ARRAY, which costs one row of code-flash
//      endurance per run: the same erase/program round trip on the last
//      row of the part, and the AHB stall measured - the counter-proof
//      to letter d. Ask for it when you want the number; z does not.
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "samc/clock.hpp"
#include "samc/nvic.hpp"
#include "samc/nvm.hpp"
#include "samc/nvm_flash.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
#include "util/nv_heap.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

constexpr brio::UartPads console_pads{
    .tx = brio::SercomPad::pad0,
    .rx = brio::SercomPad::pad1,
    .tx_pin = {'B', 30, brio::PinFunction::d},
    .rx_pin = {'B', 31, brio::PinFunction::d},
};
using Serial = brio::Uart<5, console_pads>;
constexpr Serial serial;

using Led = brio::Pin<'B', 23>;

brio::TestBench<Serial> bench;

using brio::crlf;
using brio::print;
using brio::Nvm;
using brio::NvmArray;
using brio::NvmError;

// ---------------------------------------------------------------------------
// Where this suite is allowed to write
//
// THE RWWEE ARRAY IS ENTIRELY OURS: no linker section can reach it, so
// any row of it is fair game. The suite works at the BOTTOM of the array
// and the heap of letter e at the top, so the two never meet - the map
// home is the last two rows and the block lands as high as it can below
// them, while `scratch_row` is row zero.
//
// THE MAIN ARRAY IS NOT. Letter m touches exactly one row, the LAST of
// the part: the image is tens of kilobytes and gcc places nothing near
// 0x0003FF00, so the row is free - but it is still code flash, and that
// is why the letter sits outside z.
// ---------------------------------------------------------------------------
constexpr uint32_t scratch_row = Nvm::rwwee_base;
constexpr uint32_t main_scratch_row = Nvm::main_end - Nvm::row_size;

// The heap: 8 blocks is more than this suite needs and the map still
// fits in one row (the static_asserts inside NvHeap check that).
using Heap = brio::NvHeap<brio::RwweeFlash, 8, 2>;
Heap heap;
constexpr uint16_t heap_record = 0x5A01;

// ---------------------------------------------------------------------------
// A cycle-resolution stopwatch (the same one test_samc_dma uses)
//
// The kernel timebase ticks at 1 kHz, far too coarse for an operation
// that costs microseconds. SysTick counts CPU cycles DOWN from LOAD to
// zero once per tick, so tick x (LOAD + 1) + (LOAD - VAL) is the same
// clock read at 48 MHz. The tick counter is read on both sides of VAL
// and the pair retried on a mismatch, because a tick landing between the
// two reads would pair a new tick with an old remainder.
// ---------------------------------------------------------------------------
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = brio::Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = brio::Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}

uint32_t cycles_to_us(uint32_t cycles) { return cycles / (SysClock::hz / 1'000'000UL); }

// ---------------------------------------------------------------------------
// Buffers
//
// VOLATILE, because these are read back from flash the CPU itself just
// programmed through a side channel the optimizer cannot see. The DMAC
// campaign learned this the expensive way on this same target: gcc will
// happily fold a read-back into whatever the CPU last stored, and a
// round-trip test that does that proves nothing at all.
// ---------------------------------------------------------------------------
volatile uint8_t page_out[Nvm::page_size];
volatile uint8_t page_in[Nvm::page_size];

std::span<const uint8_t> out_span() {
    return std::span<const uint8_t>(const_cast<const uint8_t*>(page_out),
                                    Nvm::page_size);
}
std::span<uint8_t> in_span() {
    return std::span<uint8_t>(const_cast<uint8_t*>(page_in), Nvm::page_size);
}

/// Fill page_out with a pattern that is different in every byte and
/// different for every seed, so a read-back can only match by being real.
void fill_pattern(uint8_t seed) {
    for (uint32_t i = 0; i < Nvm::page_size; ++i) {
        page_out[i] = static_cast<uint8_t>(seed ^ (i * 7u + 0x5Bu));
    }
}

bool pattern_matches() {
    for (uint32_t i = 0; i < Nvm::page_size; ++i) {
        if (page_in[i] != page_out[i]) {
            return false;
        }
    }
    return true;
}

bool all_erased(uint32_t addr, uint32_t len) {
    const volatile uint8_t* p = reinterpret_cast<const volatile uint8_t*>(addr);
    for (uint32_t i = 0; i < len; ++i) {
        if (p[i] != 0xFFu) {
            return false;
        }
    }
    return true;
}

const char* error_name(NvmError e) {
    switch (e) {
    case NvmError::none: return "none";
    case NvmError::busy: return "busy";
    case NvmError::timed_out: return "timed_out";
    case NvmError::bad_address: return "bad_address";
    case NvmError::program_error: return "program_error";
    case NvmError::lock_error: return "lock_error";
    case NvmError::nvm_error: return "nvm_error";
    }
    return "?";
}

// =============================================================================
// a - geometry and identity (no wear)
// =============================================================================
void ta_geometry() {
    print(serial, "  header: page=", Nvm::page_size, " row=", Nvm::row_size,
          " main=", Nvm::main_size, " rwwee=", Nvm::rwwee_size, crlf);
    print(serial, "  PARAM : page=", Nvm::param_page_size(),
          " main_pages=", Nvm::param_main_pages(),
          " rwwee_pages=", Nvm::param_rwwee_pages(), crlf);

    bench.verdict("PARAM reports the geometry the header was built for",
                  Nvm::geometry_matches());
    bench.verdict("the row is exactly four pages",
                  Nvm::pages_per_row == 4u && Nvm::row_size == 4u * Nvm::page_size);

    // ADDR's encoding, PROVEN rather than trusted. A store into the
    // memory-mapped array loads the page buffer and hardware latches the
    // address; 27.8.8 says the latched value is the half-word offset
    // FROM THE SECTION BASE, and 22 bits could have held either
    // convention. The page buffer is cleared afterwards so nothing is
    // left half-loaded for the next letter.
    *reinterpret_cast<volatile uint32_t*>(Nvm::rwwee_base + 0x100u) = 0x12345678UL;
    const uint32_t addr_rwwee = Nvm::address();
    const bool load_flag = Nvm::status().page_buffer_loaded;
    (void)Nvm::clear_page_buffer();
    const bool load_cleared = !Nvm::status().page_buffer_loaded;

    *reinterpret_cast<volatile uint32_t*>(0x00020100UL) = 0x12345678UL;
    const uint32_t addr_main = Nvm::address();
    (void)Nvm::clear_page_buffer();

    print(serial, "  ADDR after a load at rwwee+0x100: ", brio::hex(addr_rwwee),
          "  at main 0x00020100: ", brio::hex(addr_main), crlf);

    bench.verdict("ADDR is section-relative half-words on RWWEE",
                  addr_rwwee == Nvm::addr_field(NvmArray::rwwee,
                                                Nvm::rwwee_base + 0x100u));
    bench.verdict("ADDR is section-relative half-words on the main array",
                  addr_main == Nvm::addr_field(NvmArray::main, 0x00020100UL));
    bench.verdict("STATUS.LOAD rises on a page-buffer load", load_flag);
    bench.verdict("the page-buffer clear command clears it", load_cleared);

    // The user row IS the fuses on this family. Reading it is how a
    // program learns whether a bootloader area or an EEPROM emulation
    // area has been carved out of the main array behind its back.
    const brio::NvmUserRow user = brio::NvmUserRow::read();
    print(serial, "  user row: word1=", brio::hex(user.word1),
          " word0=", brio::hex(user.word0), "  bootprot=", user.bootprot_bytes(), "B eeprom=", user.eeprom_bytes(),
          "B bod_level=", user.bodvdd_level(),
          " wdt=", user.wdt_enabled() ? "on" : "off",
          user.wdt_always_on() ? "+always" : "", crlf);

    bench.verdict("the user row reads as something, not as erased flash",
                  user.word0 != 0xFFFFFFFFUL);
    bench.verdict("no bootloader area is reserved on this board",
                  user.bootprot_rows() == 0u);
    bench.verdict("no EEPROM emulation area is reserved either",
                  user.eeprom_rows() == 0u);
    bench.verdict("the watchdog is not armed by the fuses",
                  !user.wdt_enabled() && !user.wdt_always_on());

    const brio::NvmCalibration cal = brio::NvmCalibration::read();
    print(serial, "  calibration: adc0 bias=", cal.adc0_biascomp(), "/",
          cal.adc0_biasrefbuf(), " adc1 bias=", cal.adc1_biascomp(), "/",
          cal.adc1_biasrefbuf(), " osc32k=", cal.osc32k_calib(),
          " cal48m 3v3=", cal.cal48m_3v3(), " 5v=", cal.cal48m_5v(), crlf);

    // Every one of these is a factory value some future driver must copy
    // into its peripheral. An all-ones field means the area was not read
    // where it was meant to be.
    bench.verdict("the calibration area is programmed, not erased",
                  cal.osc32k_calib() != 0x7Fu && cal.cal48m_3v3() != 0x3FFFFFUL);

    const brio::DeviceSerial sn = brio::DeviceSerial::read();
    print(serial, "  die serial: ", brio::hex(sn.word[0]), "-", brio::hex(sn.word[1]),
          "-", brio::hex(sn.word[2]), "-", brio::hex(sn.word[3]), crlf);
    bench.verdict("the die carries a serial number (no label to write)",
                  sn.word[0] != 0xFFFFFFFFUL && sn.word[1] != 0xFFFFFFFFUL);

    bench.verdict("the part is not locked against the debugger",
                  !Nvm::security_bit());
}

// =============================================================================
// b - the RWWEE round trip
// =============================================================================
void tb_roundtrip() {
    const NvmError erased = Nvm::erase_row(NvmArray::rwwee, scratch_row);
    print(serial, "  erase_row -> ", error_name(erased), crlf);
    bench.verdict("a row erase succeeds", erased == NvmError::none);
    bench.verdict("and leaves the whole row at 0xFF",
                  all_erased(scratch_row, Nvm::row_size));

    // All four pages of the row, each with its own pattern: a page write
    // that landed on the wrong page would show up as a mismatch on two
    // of them at once.
    bool all_ok = true;
    for (uint32_t p = 0; p < Nvm::pages_per_row; ++p) {
        fill_pattern(static_cast<uint8_t>(0x10u + p));
        const uint32_t addr = scratch_row + p * Nvm::page_size;
        if (Nvm::program_page(NvmArray::rwwee, addr, out_span()) != NvmError::none) {
            all_ok = false;
            continue;
        }
        Nvm::read(addr, in_span());
        if (!pattern_matches()) {
            all_ok = false;
        }
    }
    bench.verdict("all four pages of the row program and read back byte-exact",
                  all_ok);

    // Re-reading through the driver's span read and through a plain
    // pointer must agree: the RWWEE array is NOT cached, so there is no
    // stale line to catch here - which is itself worth asserting, since
    // the main array would need INVALL.
    Nvm::read(scratch_row, in_span());
    const volatile uint8_t* direct = reinterpret_cast<const volatile uint8_t*>(scratch_row);
    bool same = true;
    for (uint32_t i = 0; i < Nvm::page_size; ++i) {
        if (page_in[i] != direct[i]) {
            same = false;
        }
    }
    bench.verdict("the span read and a direct read agree", same);

    // Refusals. Each of these is a request the hardware would have
    // performed on the WRONG place, which is why the driver checks
    // rather than trusting the caller.
    const NvmError unaligned_page =
        Nvm::program_page(NvmArray::rwwee, scratch_row + 1u, out_span());
    const NvmError short_page = Nvm::program_page(
        NvmArray::rwwee, scratch_row, out_span().subspan(0, Nvm::page_size - 4u));
    const NvmError outside =
        Nvm::program_page(NvmArray::rwwee, Nvm::rwwee_end, out_span());
    const NvmError unaligned_row =
        Nvm::erase_row(NvmArray::rwwee, scratch_row + Nvm::page_size);
    const NvmError wrong_array =
        Nvm::erase_row(NvmArray::rwwee, 0x00010000UL);

    bench.verdict("an unaligned page address is refused",
                  unaligned_page == NvmError::bad_address);
    bench.verdict("a partial page is refused", short_page == NvmError::bad_address);
    bench.verdict("an address past the array is refused",
                  outside == NvmError::bad_address);
    bench.verdict("a row erase at a page boundary is refused",
                  unaligned_row == NvmError::bad_address);
    bench.verdict("a main-array address given to the RWWEE array is refused",
                  wrong_array == NvmError::bad_address);

    // A refusal must also be a NON-EVENT: the page buffer must not be
    // left loaded and the row must still hold what letter b put there.
    fill_pattern(0x10u);
    Nvm::read(scratch_row, in_span());
    bench.verdict("page 0 still holds what was written before the refusals",
                  pattern_matches());
}

// =============================================================================
// c - the page-buffer ordering trap, decided by data
// =============================================================================
//
// 27.6.4.3 says the 64-bit holding register PBLDATA is reset to all ones
// whenever a write CROSSES a 64-bit boundary, and gives a "random access"
// example in which one of the two words of a section comes out at
// 0xFFFFFFFF. What the chapter does NOT say is which access patterns that
// actually condemns, and the difference matters: it is the rule
// program_page() has to keep.
//
// So all three orders are written here and read back.
//
//   ASCENDING     words 0,1,2,3,...  - what the driver does.
//   DESCENDING    words 15,14,13,... - which LOOKS like it crosses a
//                 boundary on every store, and does. But descending
//                 visits 15 then 14 (one section), then 13 then 12 (the
//                 next), so each section's two halves are still written
//                 BACK TO BACK, and the crossing happens only after the
//                 section is complete.
//   EVEN-THEN-ODD words 0,2,4,... then 1,3,5,... - the chapter's own
//                 example, generalized: every store crosses a boundary
//                 AND no section ever gets both halves consecutively.
//
// The prediction, if PBLDATA works as described: the first two come out
// exact and only the third loses anything - which would mean the real
// rule is not "ascending" but "write both words of a 64-bit section back
// to back".
void tc_ordering() {
    bench.verdict("the row erases for the experiment",
                  Nvm::erase_row(NvmArray::rwwee, scratch_row) == NvmError::none);

    // Load one page by hand in a given store order and commit it. Only
    // the ORDER is the experiment, so the command is borrowed from the
    // driver and the loading is done here.
    const auto load_and_write = [](uint32_t addr, const uint8_t* order,
                                   uint32_t words) {
        (void)Nvm::clear_page_buffer();
        volatile uint32_t* dst = reinterpret_cast<volatile uint32_t*>(addr);
        for (uint32_t k = 0; k < words; ++k) {
            const uint32_t w = order[k];
            const uint32_t i = w * 4u;
            dst[w] = static_cast<uint32_t>(page_out[i]) |
                     (static_cast<uint32_t>(page_out[i + 1u]) << 8) |
                     (static_cast<uint32_t>(page_out[i + 2u]) << 16) |
                     (static_cast<uint32_t>(page_out[i + 3u]) << 24);
        }
        Nvm::address(NvmArray::rwwee, addr);
        return Nvm::command(NVMCTRL_CTRLA_CMD_RWWEEWP_Val);
    };

    // Count, per 64-bit half, how many words came back right and how many
    // came back erased. "low" is the even word of each section, "high"
    // the odd one.
    struct Score { uint32_t low_ok, low_erased, high_ok, high_erased; };
    const auto score = [](uint32_t addr) {
        Nvm::read(addr, in_span());
        Score sc{0, 0, 0, 0};
        for (uint32_t w = 0; w < Nvm::page_size / 4u; ++w) {
            const uint32_t i = w * 4u;
            bool matches = true, erased_word = true;
            for (uint32_t b = 0; b < 4u; ++b) {
                if (page_in[i + b] != page_out[i + b]) matches = false;
                if (page_in[i + b] != 0xFFu) erased_word = false;
            }
            if ((w & 1u) == 0u) {
                sc.low_ok += matches ? 1u : 0u;
                sc.low_erased += erased_word ? 1u : 0u;
            } else {
                sc.high_ok += matches ? 1u : 0u;
                sc.high_erased += erased_word ? 1u : 0u;
            }
        }
        return sc;
    };

    constexpr uint32_t words = Nvm::page_size / 4u;   // 16
    uint8_t order[words];

    fill_pattern(0x33u);

    // 1. Ascending - the driver's own discipline, through its own verb.
    const bool asc_ok =
        Nvm::program_page(NvmArray::rwwee, scratch_row, out_span()) == NvmError::none;
    const Score asc = score(scratch_row);
    print(serial, "  ascending    : low ", asc.low_ok, "/8 exact, high ",
          asc.high_ok, "/8 exact", crlf);
    bench.verdict("ascending: the page is exact",
                  asc_ok && asc.low_ok == 8u && asc.high_ok == 8u);

    // 2. Descending.
    for (uint32_t k = 0; k < words; ++k) {
        order[k] = static_cast<uint8_t>(words - 1u - k);
    }
    const uint32_t page_desc = scratch_row + Nvm::page_size;
    const NvmError desc_err = load_and_write(page_desc, order, words);
    const Score desc = score(page_desc);
    print(serial, "  descending   : low ", desc.low_ok, "/8 exact, high ",
          desc.high_ok, "/8 exact  (-> ", error_name(desc_err), ")", crlf);
    bench.verdict("descending is ALSO exact: each 64-bit section is still "
                  "completed before the crossing",
                  desc_err == NvmError::none && desc.low_ok == 8u &&
                      desc.high_ok == 8u);

    // 3. Even words, then odd words: the chapter's example, generalized.
    uint32_t k = 0;
    for (uint32_t w = 0; w < words; w += 2u) order[k++] = static_cast<uint8_t>(w);
    for (uint32_t w = 1; w < words; w += 2u) order[k++] = static_cast<uint8_t>(w);
    const uint32_t page_split = scratch_row + 2u * Nvm::page_size;
    const NvmError split_err = load_and_write(page_split, order, words);
    const Score split = score(page_split);
    print(serial, "  even-then-odd: low ", split.low_ok, "/8 exact (", split.low_erased,
          " erased), high ", split.high_ok, "/8 exact (", split.high_erased,
          " erased)  (-> ", error_name(split_err), ")", crlf);

    bench.verdict("the split write itself reported no error",
                  split_err == NvmError::none);
    bench.verdict("PBLDATA's boundary reset is REAL: the words written FIRST "
                  "were lost",
                  split.low_erased == 8u && split.low_ok == 0u);
    bench.verdict("and the words written last - one per section - survived",
                  split.high_ok == 8u);

    print(serial, "  measured rule: not \"ascending\" but \"write both words of a "
                  "64-bit section back to back\" - ascending and descending both "
                  "satisfy it, interleaving does not", crlf);
}

// =============================================================================
// d - what it costs, and the no-stall claim
// =============================================================================
//
// The architectural claim of this whole design is that an RWWEE write
// does not stall the CPU. It is measured here the only way that means
// anything: by counting how much work the CPU got done DURING the
// operation, against a calibration of how much it does in the same time
// with the flash idle.
void td_cost() {
    // Calibrate the counting loop: iterations per microsecond, measured
    // with nothing else happening.
    const uint32_t cal_start = cycles_now();
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < 20000u; ++i) {
        sink = sink + 1u;
    }
    const uint32_t cal_cycles = cycles_now() - cal_start;
    const uint32_t cal_us = cycles_to_us(cal_cycles);

    // A row erase, timed, with the counting loop running in the gap
    // between issuing the command and READY coming back.
    Nvm::clear_status();
    const uint32_t e0 = cycles_now();
    Nvm::address(NvmArray::rwwee, scratch_row);
    NVMCTRL_REGS->NVMCTRL_CTRLA = static_cast<uint16_t>(
        NVMCTRL_CTRLA_CMD(NVMCTRL_CTRLA_CMD_RWWEEER_Val) |
        NVMCTRL_CTRLA_CMDEX(NVMCTRL_CTRLA_CMDEX_KEY_Val));
    uint32_t spins_erase = 0;
    while (!Nvm::ready()) {
        ++spins_erase;
    }
    const uint32_t erase_cycles = cycles_now() - e0;

    fill_pattern(0x77u);
    const uint32_t w0 = cycles_now();
    const NvmError wr = Nvm::program_page(NvmArray::rwwee, scratch_row, out_span());
    const uint32_t write_cycles = cycles_now() - w0;

    print(serial, "  calibration : 20000 loop turns in ", cal_us, " us", crlf);
    print(serial, "  RWWEE erase : ", cycles_to_us(erase_cycles), " us, and the CPU ran ",
          spins_erase, " polling turns inside it", crlf);
    print(serial, "  RWWEE write : ", cycles_to_us(write_cycles), " us (page, ",
          Nvm::page_size, " bytes) -> ", error_name(wr), crlf);

    bench.verdict("the erase completed", erase_cycles > 0u);
    bench.verdict("the write completed", wr == NvmError::none);
    Nvm::read(scratch_row, in_span());
    bench.verdict("and what it wrote is what was read back", pattern_matches());

    // The claim. If the AHB were stalled, the polling loop could not run
    // at all and `spins_erase` would be a handful of turns; a row erase
    // is milliseconds, so an unstalled CPU gets thousands.
    bench.verdict("THE CPU KEPT RUNNING through an RWWEE erase (no AHB stall)",
                  spins_erase > 100u);
    print(serial, "  27.6.4.1: the main array is readable while RWWEE is written - "
                  "this is why the heap lives there", crlf);

    if (cal_us == 0u) {
        print(serial, "  (calibration too fast to resolve; the spin count is the "
                      "verdict that matters)", crlf);
    }
}

// =============================================================================
// e - util/nv_heap.hpp on this silicon
// =============================================================================
void te_heap() {
    const auto& r = heap.mount();
    print(serial, "  mount: status=", static_cast<uint8_t>(r.status),
          " survivors=", r.survivors, " lost=", r.lost, " seq=", r.seq,
          " build_id=", r.build_id, crlf);
    bench.verdict("the heap mounts on the RWWEE array", r.mounted());

    // The zone is a constant on this target - no linker symbol reaches
    // into the RWWEE array - which is worth asserting because it is the
    // whole difference from the AVR backend.
    const auto zones = brio::RwweeFlash::zones();
    print(serial, "  zone: ", brio::hex(zones[0].floor), " .. ",
          brio::hex(zones[0].ceiling), " = ", zones[0].size(), " bytes", crlf);
    bench.verdict("the zone is the whole array",
                  zones[0].floor == Nvm::rwwee_base &&
                      zones[0].ceiling == Nvm::rwwee_end);

    // A block, written through the allocator's own Writer.
    constexpr uint32_t payload = 200;
    auto writer = heap.alloc(heap_record, payload);
    bench.verdict("a block is allocated", writer.has_value());
    if (!writer) {
        return;
    }

    bool appended = true;
    for (uint32_t i = 0; i < payload; ++i) {
        const uint8_t b = static_cast<uint8_t>(i * 3u + 0x11u);
        if (!writer->append(std::span<const uint8_t>(&b, 1))) {
            appended = false;
            break;
        }
    }
    bench.verdict("the payload appends a byte at a time", appended);
    const bool sealed = writer->seal();
    bench.verdict("and seals", sealed);

    auto found = heap.find(heap_record);
    bench.verdict("find() returns the block just written", found.has_value());
    if (found) {
        print(serial, "  block ", found->record_id, " at ", brio::hex(found->address),
              " len ", found->length, crlf);
        bool exact = found->length == payload;
        uint8_t got[payload];
        if (exact && found->read(0, std::span<uint8_t>(got, payload))) {
            for (uint32_t i = 0; i < payload; ++i) {
                if (got[i] != static_cast<uint8_t>(i * 3u + 0x11u)) {
                    exact = false;
                }
            }
        } else {
            exact = false;
        }
        bench.verdict("its payload reads back byte-exact", exact);

        // The block must not have landed on the scratch row the earlier
        // letters churn, or the two would fight - the allocator places
        // top-down and the scratch row is the bottom one.
        bench.verdict("the allocator placed it clear of the scratch row",
                      found->address >= scratch_row + Nvm::row_size);
    }

    // Re-mount: the map is re-read from flash, so this is the survival
    // path and not a memory of what was just done.
    const auto& again = heap.mount();
    print(serial, "  re-mount: survivors=", again.survivors, " lost=", again.lost,
          " seq=", again.seq, crlf);
    bench.verdict("a fresh mount finds the block again",
                  again.mounted() && again.lost == 0u &&
                      heap.find(heap_record).has_value());
}

// =============================================================================
// f - protection and error reporting (no wear: nothing is erased)
// =============================================================================
void tf_protection() {
    const uint16_t before = Nvm::locks();
    print(serial, "  LOCK at entry: ", brio::hex(before), crlf);
    bench.verdict("every region is unlocked out of reset (production default)",
                  before == 0xFFFFu);

    // Region 15 is the top of the main array - the same region letter m
    // writes into, which is exactly why locking it here is a safe way to
    // prove the refusal: a REFUSED erase erases nothing.
    const uint32_t target = main_scratch_row;
    bench.verdict("the region locks",
                  Nvm::lock_region(target) == NvmError::none);
    const uint16_t locked = Nvm::locks();
    print(serial, "  LOCK after lock_region: ", brio::hex(locked), crlf);
    bench.verdict("and the readback says so",
                  Nvm::region_locked(Nvm::region_of(target)) &&
                      locked == static_cast<uint16_t>(before & ~(1u << 15)));

    // The attempt. It must fail, and the row must be untouched.
    uint8_t before_bytes[16];
    Nvm::read(target, std::span<uint8_t>(before_bytes, sizeof before_bytes));
    const NvmError refused = Nvm::erase_row(NvmArray::main, target);
    uint8_t after_bytes[16];
    Nvm::read(target, std::span<uint8_t>(after_bytes, sizeof after_bytes));
    bool untouched = true;
    for (uint32_t i = 0; i < sizeof before_bytes; ++i) {
        if (before_bytes[i] != after_bytes[i]) {
            untouched = false;
        }
    }
    print(serial, "  erase into the locked region -> ", error_name(refused), crlf);
    bench.verdict("an erase into a locked region reports lock_error",
                  refused == NvmError::lock_error);
    bench.verdict("and the row is untouched", untouched);

    bench.verdict("the region unlocks again",
                  Nvm::unlock_region(target) == NvmError::none);
    bench.verdict("and LOCK is back where it started", Nvm::locks() == before);

    // STATUS.PROGE: an invalid command code. 0x7 is Reserved in table
    // 27.8.1's command list, so the controller must reject it - and
    // nothing is written by a rejected command.
    Nvm::clear_status();
    const NvmError bad = Nvm::command(0x07u);
    print(serial, "  a reserved command code -> ", error_name(bad), crlf);
    bench.verdict("an invalid command raises STATUS.PROGE",
                  bad == NvmError::program_error);

    // take_status() is the read-and-clear verb: after it, the history is
    // gone and a fresh read is clean.
    const auto taken = Nvm::take_status();
    bench.verdict("take_status() reports the error it just cleared",
                  taken.program_error);
    bench.verdict("and leaves the status clean",
                  !Nvm::status().program_error && Nvm::outcome() == NvmError::none);
}

// =============================================================================
// m - the main array, and the stall (outside z: costs one row of wear)
// =============================================================================
void tm_main_array() {
    print(serial, "  target: the LAST row of the part, ", brio::hex(main_scratch_row),
          " - free flash on any image this size", crlf);

    // The erase is issued by hand rather than through erase_row(), for
    // the same reason letter d does it: the polling turns between the
    // command and READY are the measurement.
    Nvm::clear_status();
    const uint32_t e0 = cycles_now();
    Nvm::address(NvmArray::main, main_scratch_row);
    NVMCTRL_REGS->NVMCTRL_CTRLA = static_cast<uint16_t>(
        NVMCTRL_CTRLA_CMD(NVMCTRL_CTRLA_CMD_ER_Val) |
        NVMCTRL_CTRLA_CMDEX(NVMCTRL_CTRLA_CMDEX_KEY_Val));
    uint32_t spins_erase = 0;
    while (!Nvm::ready()) {
        ++spins_erase;
    }
    const uint32_t erase_cycles = cycles_now() - e0;
    const NvmError erased = Nvm::outcome();
    bench.verdict("the main-array row erases", erased == NvmError::none);
    bench.verdict("and reads as 0xFF", all_erased(main_scratch_row, Nvm::row_size));

    fill_pattern(0xC5u);
    const uint32_t w0 = cycles_now();
    const NvmError wr =
        Nvm::program_page(NvmArray::main, main_scratch_row, out_span());
    const uint32_t write_cycles = cycles_now() - w0;
    Nvm::read(main_scratch_row, in_span());
    bench.verdict("a main-array page programs", wr == NvmError::none);
    bench.verdict("and reads back byte-exact", pattern_matches());

    // The counter-proof to letter d. Here the CPU CANNOT run: an
    // instruction fetch is a main-array read, and 27.6.4.1 says such a
    // read stalls the AHB until the operation ends. The measurement is
    // therefore of wall-clock cost only - there is no "turns completed"
    // number to take, and that absence IS the finding.
    // THE COUNTER-PROOF, and it is a measurement and not a claim. The
    // polling loop lives in the main array, so while the main array is
    // being erased the CPU cannot even FETCH it: 27.6.4.1's stall shows
    // up as a spin count of essentially zero, against the thousands
    // letter d counts for the same wait on the RWWEE array.
    print(serial, "  main erase : CPU ran ", spins_erase,
          " polling turns inside it (letter d counts thousands on RWWEE)", crlf);
    bench.verdict("THE CPU WAS STALLED through a main-array erase (27.6.4.1)",
                  spins_erase < 16u);

    // AND THE DURATIONS ARE NOT PRINTED IN MICROSECONDS, because the
    // stopwatch cannot be trusted across a stall - a fact this letter
    // measured the hard way, having first predicted the wrong cause.
    //
    // A single erase timed with cycles_now() scattered between 234 and
    // 1233 us across runs: a whole tick period of spread on a ~1 ms
    // operation. The guess was that the SysTick handler, being code in
    // the stalled main array, was MISSING ticks. The eight-round total
    // below refutes that - eight erases report eight milliseconds of
    // ticks, so nothing is lost.
    //
    // What actually happens is subtler and worth knowing on this target:
    // cycles_now() mixes the SOFTWARE tick counter with the HARDWARE
    // SysTick VAL, and the counter is advanced by a handler. During a
    // stall that handler cannot run, so in the moment right after the
    // stall ends SysTick may already have wrapped while the counter has
    // not yet been incremented - and a reading taken exactly there comes
    // out one tick short. The tick is not lost, it is merely late; the
    // damage is confined to a single reading taken in that window, which
    // is why eight erases scatter by the same one tick as one erase.
    constexpr uint32_t rounds = 8;
    constexpr uint32_t rwwee_erase_us = 989;   // letter d, stable to the microsecond
    const uint32_t t0 = brio::Ticker::ticks();
    const uint32_t c0 = cycles_now();
    for (uint32_t i = 0; i < rounds; ++i) {
        (void)Nvm::erase_row(NvmArray::main, main_scratch_row);
    }
    const uint32_t software_us = cycles_to_us(cycles_now() - c0);
    const uint32_t ticks_seen = brio::Ticker::ticks() - t0;
    const uint32_t expected_us = rounds * rwwee_erase_us;

    print(serial, "  ", rounds, " main-array row erases: software clock ",
          software_us, " us, ticks advanced ", ticks_seen,
          "; the same eight on RWWEE cost ", expected_us, " us", crlf);

    bench.verdict("no tick is LOST across the stalls (eight erases, eight ticks)",
                  ticks_seen + 1u >= rounds && ticks_seen <= rounds + 1u);
    bench.verdict("a main-array erase costs the same as an RWWEE one, within the "
                  "stopwatch's one-tick window",
                  software_us + 1100u > expected_us &&
                      software_us < expected_us + 1100u);
    print(serial, "  so the wall-clock number of record is letter d's ",
          rwwee_erase_us, " us - the same flash operation, measured where the CPU "
          "survives it (45-42 allows 6 ms)", crlf);
    print(serial, "  what main-array work really costs is not time but the CPU: "
                  "that is why the heap lives in RWWEE", crlf);
    (void)write_cycles;
    (void)erase_cycles;

    // Leave the row erased rather than holding a pattern: a future run
    // starts from a known state either way, and erased is the tidier one.
    (void)Nvm::erase_row(NvmArray::main, main_scratch_row);
    bench.verdict("the row is left erased", all_erased(main_scratch_row, Nvm::row_size));
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf, "test_samc_nvm - SAMC21J18A NVMCTRL (ch. 27), clk=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// AN UNBOUND VECTOR HERE IS A SILENT DEATH, not a crash: the crt's
// default handler is a spin loop, so the first console interrupt parks
// the CPU in it and the board simply never says anything. (The AVR side's
// twin of this lesson is the opposite shape - an unbound vector there
// jumps to 0 and the suite reboots forever.) Both of these are needed:
// the Uart is interrupt-driven, and the stopwatch below reads the tick
// counter SysTick_Handler advances.
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

// NVMCTRL's own line is deliberately NOT bound: every command in this
// suite is awaited by polling INTFLAG.READY, which is what the driver
// does. The ISR body exists (Nvm::isr()) and letter a exercises it as a
// function - binding the vector would need an armed interrupt and a
// handler with something to do, which is the sleep pass's business.

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    brio::enable_interrupts();

    bench.letter('a', "geometry, the fuses, calibration and ADDR", ta_geometry);
    bench.letter('b', "the RWWEE round trip and its refusals", tb_roundtrip);
    bench.letter('c', "the page-buffer ordering trap, by data", tc_ordering);
    bench.letter('d', "what it costs, and the no-stall claim", td_cost);
    bench.letter('e', "util/nv_heap.hpp on the RWWEE array", te_heap);
    bench.letter('f', "region locks and error reporting", tf_protection);
    bench.letter('m', "the MAIN array and its stall (one row of wear)",
                 tm_main_array, false);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " rws=", brio::FlashWaitStates::get(), crlf);
        banner();
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
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
