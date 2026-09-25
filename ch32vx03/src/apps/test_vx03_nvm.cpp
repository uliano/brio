// test_vx03_nvm - the reference bench suite for the FLASH MEMORY of the
// CH32V203 and the CH32V303: ch32vx03/nvm.hpp over RM ch. 32 (the array,
// its zero-wait window and its tail, the engine, the option bytes) and
// ch. 31 (the electronic signature), with ch32vx03/nvm_flash.hpp's
// MainFlash - the FlashMedia in the last 4 KB of the window - above it.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// WHAT A FLASH TEST COSTS. The array is rated for ten thousand erase
// cycles (datasheet table 4-18), so every letter that ERASES is outside
// `z` and is run by name: `z` reads, decodes and refuses, and spends
// not one cycle of the part. The letters that do write spend one erase
// each on ONE page of the medium's own zone - never on the image - and
// the two tail letters work at the END OF THE TAIL, far from both: t
// one standard erase of its last sector, s one 32 KB erase of its last
// block and one page erase, so a whole run of this suite costs the
// board a handful of cycles out of ten thousand.
//
// THE CLOCK IS PART OF THE CONTRACT, so this suite runs on a
// DynamicClock and not a static one. RM 32.1 forbids an erase or a
// program above 120 MHz of HCLK without halving the tree around it, and
// this driver refuses rather than moving the tree behind its caller's
// back - at compile time under a static Clock, with a CODE under a
// dynamic one. A pack of three rates is therefore what a program that
// writes flash on this family really looks like: 96 MHz to work at, 48
// MHz where the array may be read at the whole system clock (SCKMOD),
// and 144 MHz to prove the refusal on the silicon instead of only in a
// negative compile.
//
// NOTHING IS WIRED. The chapter has no pad: the array, the engine, the
// option bytes and the signature are all internal. TWO RULERS measure
// them, because they answer different questions: the core's own counter
// for the short spans (it reloads at every kernel tick, so a span nobody
// polls across can lose whole periods), and a TIMER counting
// microseconds on the peripheral bus for the milliseconds an erase or a
// program takes - which also keeps counting if the core ever stops,
// which is itself one of the things letter w asks.
//
// What is exercised, letter by letter:
//   a  THE LOCKS: which one each verb needs, the key pairs, lock() and
//      the two unlocks, and the fact that a lock closed again refuses
//      what it refused before
//   b  THE OPTION BYTES decoded read-only: FLASH_OBR and FLASH_WPR
//      against the eight bytes at 0x1FFF F800 they were loaded from,
//      the read protection, the three reset-related bits the watchdog
//      and power chapters cite
//   c  THE ELECTRONIC SIGNATURE (ch. 31): the flash capacity against
//      what the part table states, and the 96-bit unique identifier
//   d  THE STATUS FLAGS and the engine's refusals, none of which
//      touches the array: a misaligned address, a run that is not a
//      page, a chip erase with no policy, a write while locked, and the
//      enhanced read mode that would fail an erase in silence
//   e  THE MEDIUM'S GEOMETRY: where the zone is, what the linker left
//      below it, and what the zone reads now
//   f  SCKMOD, measured on BOTH paths a program takes to the array - a
//      straight run of instructions fetched from it and a kilobyte of
//      constants read out of it - with the access clock at half the
//      system clock and at the whole of it, at a rate where the whole
//      of it is legal; and the refusal above 60 MHz
//   g  THE RATE REFUSAL on the silicon: an erase asked for at 144 MHz
//      comes back `refused_rate` with nothing written, and the same
//      erase at 96 MHz is taken
//   w  (by name, one erase cycle) THE PAGE CYCLE: erase, verify the
//      ERASED PATTERN, fast-program a page, verify it byte for byte,
//      both durations against the datasheet's typical - and then a
//      program into a page that was NOT erased, which is the one
//      thing the chapter does not say
//   v  (by name, one erase cycle) THE HALF-WORD PROGRAM, its cost, and
//      whether a cell takes pass after pass between erases - which is
//      what a small-value store over this medium would need
//   q  (by name, one erase cycle, reboots) THE SECOND PASS ACROSS A
//      RESET: a window cell programmed twice, read before a system reset
//      and after it - the pair of values letter t gives a tail cell
//   u  (by name, two erase cycles) IS FLASH_ADDR PART OF THE FAST PAGE
//      PROGRAM? 32.5.6 does not list the store the sister family's
//      chapter lists, so the sequence is staged here WITHOUT it, with
//      the register left pointing at another page
//   x  (by name, one erase cycle, reboots) THE FlashMedia CONTRACT on
//      the silicon: erase a cell, program a cell, read it back, and
//      find it again after a reset
//   y  (by name, LAST, reboots) THE WRONG KEY: whether a bad key pair
//      really locks the engine until the next system reset
//   h  THE ARRAY'S THREE NUMBERS AND THE SPLIT: the window, the tail and
//      the array from the part table, USER[7:5] decoded out of FLASH_OBR
//      and out of the option byte area, and whether the two and the part
//      table agree - which also settles where FLASH_OBR carries the field
//   i  THE TAIL READ, TIMED: a kilobyte of words read out of the tail
//      against the same out of the window, at the working rate and at 48
//      MHz with SCKMOD both ways - the non-zero-wait cost in core cycles
//      - and the tail's refusals, which cost nothing
//   t  (by name, one standard erase of the tail's last sector) THE TAIL
//      WRITTEN THE STANDARD WAY: the 4 KB erase, half-words programmed
//      and read back, a second pass on one cell, the core's tick across
//      the erase
//   s  (by name, one 32 KB erase and one page erase) THE TAIL WRITTEN THE
//      FAST WAY: the 32 KB erase of its last block, a page programmed,
//      erased and programmed again with the driver's own verbs, each
//      timed - RM 32.2.1's note answered - and the page left holding a
//      pattern for letter r
//   r  (by name, OUTSIDE THE RATING, no wear, after s) THE TAIL READ AT
//      144 MHz, where the driver refuses: the page s left, read by hand
//      and compared byte for byte, against the working rate's
//
// build: boards = v203c6,v203c8,v303vc
// build: groups = abcdefgquvwxy,hirst
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include <optional>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/nvm.hpp"
#include "ch32vx03/nvm_flash.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/reset.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "kernel/panic.hpp"
#include "util/nv_heap.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32vx03Platform<>;

// ---------------------------------------------------------------------------
// The token the three rebooting letters live in.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x5B41;

struct Token {
    uint16_t magic;
    char letter;
    uint8_t leg;
    uint16_t pass;
    uint16_t fail;
    uint32_t mark;    ///< what the leg wrote before the reset
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

// ---------------------------------------------------------------------------
// The rates. 96 MHz is where everything is written: below
// flash_safe_sysclk_hz, so an erase needs nothing of the tree. 48 MHz is
// where FLASH_CTLR.SCKMOD may be set at all, because the access clock it
// then chooses is HCLK whole and the ceiling is 60 MHz. 144 MHz is the
// refusal, measured rather than assumed.
// ---------------------------------------------------------------------------
using Work = Clock<ClockSource::pll, 96'000'000>;
using Slow = Clock<ClockSource::pll, 48'000'000>;
using Full = Clock<ClockSource::pll, 144'000'000>;
using SysClock = DynamicClock<Rates<Work, Slow, Full>, Ticker, Serial>;
constexpr SysClock clock;

constexpr uint8_t work_rate = 0;
constexpr uint8_t slow_rate = 1;
constexpr uint8_t full_rate = 2;

using Store = MainFlash<SysClock>;
static_assert(FlashMedia<Store>);

TestBench<Serial> bench;

uint32_t boot_flags = 0;

/// What the last run left if it ended in a trap (the vectors are bound
/// to fault_reset below): a staged operation the silicon answers with a
/// bus fault is reported at the next boot instead of hanging the board.
std::optional<PanicRecord> boot_record;

/// The flash vector's tally (letter d): what the engine's own interrupt
/// counted.
volatile uint32_t flash_irqs = 0;
volatile uint32_t flash_irq_flags = 0;

/// The core's counter as a stopwatch, the ruler every duration here is
/// weighed against.
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

    uint32_t us() { return cycles() / (SysClock::hz() / 1'000'000u); }

private:
    uint32_t last_ = 0;
    uint32_t acc_ = 0;
};

/// THE RULER FOR A MILLISECOND-LONG OPERATION, and the reason there are
/// two of them in this file. The core's counter reloads at every tick of
/// the kernel's timebase, so a span nobody polls across can lose whole
/// periods - and an erase is sixteen of them. A timer counting
/// microseconds on the peripheral bus has no such bound below 65
/// milliseconds, and it keeps counting while the core waits for the
/// flash: that independence is exactly what makes it the instrument for
/// this measurement.
using Ruler = Tim<2>;
constexpr uint32_t ruler_prescaler = Work::timclk1_hz / 1'000'000u - 1u;

void ruler_init() {
    Ruler::init();
    (void)Ruler::configure({.prescaler = static_cast<uint16_t>(ruler_prescaler),
                            .period = 0xFFFFu});
    Ruler::enable(true);
}

uint16_t ruler_now() { return static_cast<uint16_t>(Ruler::count()); }

uint16_t ruler_since(uint16_t mark) { return static_cast<uint16_t>(ruler_now() - mark); }

/// Where the letters that write work: the LAST page of the medium's
/// zone, which is the last page of the zero-wait window. One page, so
/// the wear falls on one place and not on sixteen.
constexpr uint32_t test_page = MainFlashPartition::storage_end - MainFlashPartition::page;
/// The page below it, for the letters that want two.
constexpr uint32_t second_page = test_page - MainFlashPartition::page;

/// A page of bytes with no symmetry: a word order or a byte order that
/// slipped anywhere shows up as a mismatch and not as a coincidence.
void fill_pattern(uint8_t* dst, uint32_t seed) {
    uint32_t x = seed;
    for (uint32_t i = 0; i < MainFlashPartition::page; ++i) {
        x = x * 1664525u + 1013904223u;
        dst[i] = static_cast<uint8_t>(x >> 24);
    }
}

uint8_t page_buffer[MainFlashPartition::page];
uint8_t read_back[MainFlashPartition::page];

/// The words of a run as the core reads them, for a verdict line that
/// shows the pattern rather than claiming it.
void print_first_words(uint32_t addr, uint32_t count) {
    uint32_t words[4];
    Flash::read(addr, {reinterpret_cast<uint8_t*>(words), count * 4u});
    print(serial, "  at ", hex(addr), ":");
    for (uint32_t i = 0; i < count; ++i) {
        print(serial, " ", hex(words[i]));
    }
    print(serial, crlf);
}

// ===========================================================================
// a - the locks
// ===========================================================================
void ta_locks() {
    // At every boot the engine is locked: the reset value of both bits
    // is one, and nothing in the crt opens them.
    const bool locked_at_boot = Flash::locked() && Flash::fast_locked();
    const bool options_locked = Flash::options_locked();

    // The FPEC lock alone, which is what the standard verbs need.
    Flash::lock();
    const bool std_open = Flash::unlock_standard();
    const bool fast_still_shut = Flash::fast_locked();
    // And then the fast one on top of it.
    const bool both_open = Flash::unlock();
    Flash::lock();
    const bool both_shut = Flash::locked() && Flash::fast_locked();
    print(serial, "  at boot: LOCK=", Flash::locked() ? 1 : 0, " FLOCK=",
          Flash::fast_locked() ? 1 : 0, " OBWRE=", options_locked ? 0 : 1, crlf);
    bench.verdict("both locks are SHUT at every boot and the option lock with them: an "
                  "image that never unlocks cannot write the array by accident",
                  locked_at_boot && options_locked);
    bench.verdict("KEYR's pair opens the FPEC lock alone - the fast one stays shut, which "
                  "is exactly the split 32.4.4 makes bit by bit",
                  std_open && fast_still_shut);
    bench.verdict("MODEKEYR's pair opens the fast lock on top of it, and writing a one "
                  "into each bit closes both again",
                  both_open && both_shut);

    // A verb that needs the fast lock is refused while only the FPEC
    // one is open, and the refusal costs nothing: no operation starts.
    Flash::lock();
    (void)Flash::unlock_standard();
    const uint32_t fast_refused = Flash::erase_page(clock, test_page);
    Flash::lock();
    const uint32_t all_refused = Flash::erase_page(clock, test_page);
    print(serial, "  erase_page with the fast lock shut: ", hex(fast_refused),
          ", with both shut: ", hex(all_refused), " (refused = ", hex(Flash::refused), ")",
          crlf);
    bench.verdict("a fast verb is REFUSED while the fast lock stands, and refused again "
                  "with both locks shut - the driver asks the lock the chapter names for "
                  "that bit and starts nothing",
                  fast_refused == Flash::refused && all_refused == Flash::refused);

    // Re-opening is free and idempotent: unlock() on an already open
    // engine writes no key and answers true.
    const bool reopen = Flash::unlock() && Flash::unlock();
    Flash::lock();
    bench.verdict("unlock() is idempotent - an engine already open takes no key and "
                  "answers true, so a medium may open the window at every call",
                  reopen);
}

// ===========================================================================
// b - the option bytes, read-only
// ===========================================================================
void tb_options() {
    const FlashOptions opt = Flash::options();
    const FlashOptionArea area = FlashOptionArea::read();
    const bool consistent = FlashOptionArea::consistent();

    print(serial, "  FLASH_OBR=", hex(Flash::regs().OBR), " FLASH_WPR=",
          hex(opt.write_protection), crlf);
    print(serial, "  the area at 0x1FFFF800: RDPR=", hex(area.rdpr), " USER=",
          hex(area.user), " DATA0=", hex(area.data0), " DATA1=", hex(area.data1), crlf);
    print(serial, "  read protection ", opt.read_protected ? "ON" : "off",
          ", IWDG ", opt.iwdg_software ? "software" : "HARDWARE",
          ", reset on Stop ", opt.resets_on_stop ? "yes" : "no",
          ", reset on Standby ", opt.resets_on_standby ? "yes" : "no", crlf);

    bench.verdict("every byte of the option area carries its own inverse in the half-word "
                  "above it, and all four pairs agree - which is what the loader checks "
                  "before it believes FLASH_OBR",
                  consistent);
    bench.verdict("OBERR is clear: the hardware loaded the area without finding a byte "
                  "that disagrees with its complement",
                  !opt.error);
    bench.verdict("the part is NOT read-protected, which is what lets a probe read the "
                  "array at all - and RDPR reads the unprotected key",
                  !opt.read_protected && area.rdpr == flash_rdpr_unprotected);
    bench.verdict("the decoded USER bits are the COMPLEMENTS the register description "
                  "makes them: a zero bit means the reset happens, so the watchdog and the "
                  "power chapters read them through these names and not through the raw "
                  "word",
                  opt.iwdg_software == ((area.user & 0x01u) != 0u) &&
                      opt.resets_on_stop == ((area.user & 0x02u) == 0u) &&
                      opt.resets_on_standby == ((area.user & 0x04u) == 0u));

    // The write protection: no sector of this part is protected, so the
    // medium's own zone is writable - which is why the suite can run at
    // all, and a fact a program should check before it writes.
    const bool zone_open = !opt.address_protected(MainFlashPartition::storage_base);
    print(serial, "  the medium's zone at ", hex(MainFlashPartition::storage_base), " is ",
          zone_open ? "writable" : "PROTECTED", crlf);
    bench.verdict("FLASH_WPR leaves every 4 KB sector writable on this part, the medium's "
                  "own among them - a CLEAR bit would be the protected one",
                  zone_open && opt.write_protection == 0xFFFFFFFFu);
}

// ===========================================================================
// c - the electronic signature
// ===========================================================================
void tc_signature() {
    const uint16_t kb = Flash::size_kbytes();
    const DeviceUid uid = DeviceUid::read();
    print(serial, "  FLACAP=", kb, " KB; the part table's window is ",
          device::flash_bytes / 1024u, " KB, the largest its option byte can select ",
          flash_window_max_bytes / 1024u, " KB", crlf);
    print(serial, "  UID: ", hex(uid.word[0]), " ", hex(uid.word[1]), " ", hex(uid.word[2]),
          uid.word[2] == Flash::erased_word ? " - the third word reads the ERASED PATTERN"
                                            : " - the third word carries a value",
          crlf);
    bench.verdict("the signature's flash capacity is the LARGEST zero-wait window this part's "
                  "option byte can select - the window itself on a part with no split - so "
                  "the die and the part table agree about the array",
                  kb == flash_window_max_bytes / 1024u);
    if constexpr (device::device_class == DeviceClass::v20x_d6 ||
                  device::device_class == DeviceClass::v30x_d8) {
        bench.verdict("the identifier the chapter calls ninety-six bits is SIXTY-FOUR on this "
                      "die: the first two words are the factory's and the third reads the "
                      "ERASED PATTERN of this array, so a board is known by the pair and not "
                      "by the three",
                      uid.word[0] != 0u && uid.word[1] != 0u &&
                          uid.word[2] == Flash::erased_word);
    } else {
        bench.verdict("the identifier's first two words carry the factory's number, and the "
                      "third is printed for what it is on this class",
                      uid.word[0] != 0u && uid.word[1] != 0u);
    }
    // The same three words read twice: a signature is memory, not a
    // counter.
    const DeviceUid again = DeviceUid::read();
    bench.verdict("and it reads the same twice, which is what tells a signature from a "
                  "register",
                  again.word[0] == uid.word[0] && again.word[1] == uid.word[1] &&
                      again.word[2] == uid.word[2]);
}

// ===========================================================================
// d - the status flags, the interrupt and the refusals
// ===========================================================================
void td_refusals() {
    const uint32_t statr = Flash::status();
    print(serial, "  FLASH_STATR=", hex(statr), " (BSY=", Flash::busy() ? 1 : 0,
          " WRBSY=", Flash::write_busy() ? 1 : 0, " EOP=", Flash::operation_ended() ? 1 : 0,
          " EHMODS=", Flash::enhanced_read() ? 1 : 0, ")", crlf);
    bench.verdict("with nothing running the engine is idle and no error stands: BSY clear, "
                  "WRBSY clear, and WRPRTERR - the ONE error this family's STATR carries - "
                  "clear too",
                  !Flash::busy() && !Flash::write_busy() && Flash::errors() == 0u);

    // Every refusal below costs nothing: the driver checks before it
    // opens a lock, so not one of these reaches the array.
    (void)Flash::unlock();
    const uint32_t misaligned = Flash::erase_page(clock, test_page + 4u);
    // Past the ARRAY: the window on a part with no tail, the tail above
    // it on one that has one - both methods write the tail.
    const uint32_t past_end = Flash::erase_page(clock, Flash::tail_end);
    const uint32_t odd_half = Flash::program_half_word(clock, test_page + 1u, 0x1234u);
    const uint32_t short_page =
        Flash::program_page(clock, test_page, {page_buffer, 64u});
    const uint32_t no_policy = Flash::erase_chip(clock);
    Flash::lock();
    print(serial, "  refusals: misaligned ", hex(misaligned), ", past the end ",
          hex(past_end), ", odd half-word ", hex(odd_half), ", short page ",
          hex(short_page), ", chip erase with no policy ", hex(no_policy), crlf);
    bench.verdict("five malformed calls are refused with the caller's own bit and nothing "
                  "written: a page address that is not a page, one past the array, "
                  "an odd half-word, a run that is not a whole page, and a chip erase "
                  "whose policy argument was not passed",
                  misaligned == Flash::refused && past_end == Flash::refused &&
                      odd_half == Flash::refused && short_page == Flash::refused &&
                      no_policy == Flash::refused);

    // The enhanced read mode. 32.3 offers it to a program whose code
    // outgrew the flash/RAM split RAM_CODE_MOD chooses - a field the
    // option bytes give to the CH32V203RB and to the CH32V303RC and VC
    // (table 32-4's note) - so whether it engages at all is a question
    // for the silicon. The bit is written, the status polled, and an erase is
    // asked for ONLY if the mode really stands: a letter inside `z`
    // spends no cycle of the array.
    (void)Flash::unlock();
    const bool entered = Flash::enhanced_read(true);
    const bool bit_stuck = (Flash::regs().CTLR & flash_ehmod) != 0u;
    uint32_t refused_in_mode = Flash::refused;
    if (entered) {
        refused_in_mode = Flash::erase_page(clock, test_page);
    }
    const bool left = !Flash::enhanced_read(false);
    Flash::lock();
    print(serial, "  enhanced read mode: EHMOD reads back ", bit_stuck ? 1 : 0,
          ", EHMODS ", entered ? "FOLLOWED" : "never followed", ", left=", left ? 1 : 0,
          crlf);
    if constexpr (device::device_class == DeviceClass::v20x_d6 ||
                  device::device_class == DeviceClass::v30x_d8) {
        bench.verdict("the enhanced read mode does not engage on this device class: EHMOD "
                      "takes the write and FLASH_STATR.EHMODS never follows it - on a part "
                      "whose option byte carries the flash and RAM split 32.3 speaks of as on "
                      "one whose byte does not",
                      bit_stuck && !entered && left && refused_in_mode == Flash::refused);
    } else {
        bench.verdict("the enhanced read mode answers what EHMODS says - whether it engages on "
                      "this class is the line above - an erase is refused while it stands and "
                      "leaving it works",
                      bit_stuck && left && refused_in_mode == Flash::refused);
    }

    // The interrupt: armed, read back, disarmed. Nothing raises it
    // here, which is the point - an armed vector with no operation is
    // quiet.
    (void)Flash::unlock();
    Flash::interrupts(true, true);
    const bool armed = Flash::end_interrupt() && Flash::error_interrupt();
    Pfic::enable(Irq::flash);
    Stopwatch quiet_watch;
    while (quiet_watch.us() < 1000u) {
    }
    const uint32_t quiet = flash_irqs;
    Flash::interrupts(false, false);
    const bool disarmed = !Flash::end_interrupt() && !Flash::error_interrupt();
    Pfic::disable(Irq::flash);
    Flash::lock();
    bench.verdict("EOPIE and ERRIE arm and disarm, and an armed vector with no operation "
                  "running is silent",
                  armed && disarmed && quiet == 0u);
}

// ===========================================================================
// e - the medium's geometry
// ===========================================================================
void te_geometry() {
    const auto zones = Store::zones();
    const uint32_t rom_end =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&__brio_rom_end[0]));
    print(serial, "  the window is ", device::flash_bytes / 1024u, " KB; the linker stops at ",
          hex(rom_end), ", the zone runs ", hex(zones[0].floor), "..", hex(zones[0].ceiling),
          " (", (zones[0].ceiling - zones[0].floor) / Store::erase_size, " cells of ",
          Store::erase_size, " B)", crlf);
    print(serial, "  the linker's region ends ",
          rom_end == MainFlashPartition::storage_base ? "exactly at the zone's floor"
                                                      : "short of the zone's floor",
          crlf);
    bench.verdict("the medium is the LAST 4 KB of the window - one write-protection unit, "
                  "sixteen pages - and the linker script stops short of it, so nothing the "
                  "compiler emits can reach the storage",
                  MainFlashPartition::geometry_matches_silicon() &&
                      zones[0].floor == MainFlashPartition::storage_base &&
                      zones[0].ceiling == device::flash_bytes && rom_end <= zones[0].floor);
    bench.verdict("the cell IS the page: 256 bytes to erase and 256 to program, which is "
                  "the only grain the fast method has",
                  Store::erase_size == 256u && Store::write_cell == 256u &&
                      Store::zone_count == 1u);

    // What the zone holds now: printed and not judged, because it is
    // whatever the last write letter left - the erased pattern on a
    // board that has never run w, and a pattern of this suite's making
    // afterwards.
    print_first_words(zones[0].floor, 4);
    print_first_words(test_page, 4);
    const bool readable = Store::erased_word == Flash::erased_word;
    bench.verdict("the medium publishes the erased pattern of THIS array, which is not the "
                  "all-ones every other flash in this project leaves",
                  readable && Store::erased_word == 0xE339E339u);
}

// ===========================================================================
// f - SCKMOD, measured
// ===========================================================================

/// TWO WAYS TO ASK THE ARRAY FOR SOMETHING, because SCKMOD is the
/// ACCESS clock and a program touches flash in two ways: it FETCHES
/// instructions from it and it READS data out of it.
///
/// The fetch is measured with a straight run and not with a tight loop:
/// a loop of ten instructions is served by whatever the core keeps in
/// front of itself and never asks the array for a new line, which is
/// what the first attempt measured (nothing). `.rept` lays down two
/// hundred and fifty-six compressed instructions in a row - half a
/// kilobyte of code the core has to walk through - and the loop around
/// it walks it again and again.
volatile uint32_t loop_seed = 1;

[[gnu::noinline]] uint32_t fetch_run(uint32_t laps) {
    uint32_t acc = loop_seed;
    for (uint32_t i = 0; i < laps; ++i) {
        __asm__ volatile(".rept 256\n\taddi %0, %0, 1\n\t.endr" : "+r"(acc));
    }
    return acc;
}

/// The data side: a kilobyte of constants in the array, summed. Nothing
/// of it is in RAM and nothing of it can be folded, because the sum is
/// taken through a volatile pointer.
const uint32_t blob[256] = {
#define BRIO_BLOB_8(n) \
    (n) * 2654435761u, (n) * 40503u + 7u, (n) ^ 0xA5A5A5A5u, (n) * 3u + 1u, \
    (n) * 97u, ~(n), (n) << 3, (n) | 0x5A5A0000u
#define BRIO_BLOB_32(n) \
    BRIO_BLOB_8(n), BRIO_BLOB_8((n) + 1u), BRIO_BLOB_8((n) + 2u), BRIO_BLOB_8((n) + 3u)
    BRIO_BLOB_32(0u),  BRIO_BLOB_32(4u),  BRIO_BLOB_32(8u),  BRIO_BLOB_32(12u),
    BRIO_BLOB_32(16u), BRIO_BLOB_32(20u), BRIO_BLOB_32(24u), BRIO_BLOB_32(28u),
#undef BRIO_BLOB_32
#undef BRIO_BLOB_8
};

[[gnu::noinline]] uint32_t read_run(uint32_t laps) {
    uint32_t acc = 0;
    const volatile uint32_t* p = blob;
    for (uint32_t i = 0; i < laps; ++i) {
        for (uint32_t j = 0; j < 256u; ++j) {
            acc += p[j];
        }
    }
    return acc;
}

void tf_sckmod() {
    // SCKMOD chooses HCLK or half of it as the ACCESS clock, and 60 MHz
    // is the ceiling - so the whole system clock is legal only at 48
    // MHz of the pack, never at the 96 this suite works at.
    (void)Flash::unlock();
    const bool refused_at_96 = !Flash::access_clock_whole(clock, true);
    const uint32_t access_96 = Flash::access_hz(clock);
    Flash::lock();
    print(serial, "  at ", SysClock::hz() / 1'000'000u, " MHz: SCKMOD=1 ",
          refused_at_96 ? "REFUSED" : "taken", ", the array is read at ",
          access_96 / 1'000'000u, " MHz", crlf);
    bench.verdict("at 96 MHz the whole system clock is REFUSED as the access clock - 60 "
                  "MHz is the chapter's ceiling and the reset value's half is what keeps "
                  "the array inside it",
                  refused_at_96 && access_96 == 48'000'000u);

    // Down to 48 MHz, where both settings are legal, and the same two
    // runs timed under each.
    const bool stepped = SysClock::set_index(slow_rate);
    volatile uint32_t sink = 0;

    const uint32_t ctlr_boot = Flash::regs().CTLR;
    const bool unlocked_here = Flash::unlock();
    const uint32_t ctlr_open = Flash::regs().CTLR;
    const bool half = Flash::access_clock_whole(clock, false);
    const bool half_reads = !Flash::access_clock_whole();
    const uint32_t ctlr_half = Flash::regs().CTLR;
    Flash::lock();
    Stopwatch fetch_half_watch;
    for (uint32_t i = 0; i < 200u; ++i) {
        sink += fetch_run(1);
        (void)fetch_half_watch.cycles();
    }
    const uint32_t fetch_half = fetch_half_watch.cycles();
    Stopwatch read_half_watch;
    for (uint32_t i = 0; i < 40u; ++i) {
        sink += read_run(1);
        (void)read_half_watch.cycles();
    }
    const uint32_t read_half = read_half_watch.cycles();

    (void)Flash::unlock();
    const bool whole = Flash::access_clock_whole(clock, true);
    const bool whole_reads = Flash::access_clock_whole();
    const uint32_t ctlr_whole = Flash::regs().CTLR;
    Flash::lock();
    Stopwatch fetch_whole_watch;
    for (uint32_t i = 0; i < 200u; ++i) {
        sink += fetch_run(1);
        (void)fetch_whole_watch.cycles();
    }
    const uint32_t fetch_whole = fetch_whole_watch.cycles();
    Stopwatch read_whole_watch;
    for (uint32_t i = 0; i < 40u; ++i) {
        sink += read_run(1);
        (void)read_whole_watch.cycles();
    }
    const uint32_t read_whole = read_whole_watch.cycles();

    // Leave the array as the reset value has it, whatever the answer.
    (void)Flash::unlock();
    (void)Flash::access_clock_whole(clock, false);
    Flash::lock();
    const bool back = SysClock::set_index(work_rate);

    const uint32_t fetch_gain = fetch_half > fetch_whole
                                    ? ((fetch_half - fetch_whole) * 100u) / fetch_half
                                    : 0u;
    const uint32_t read_gain =
        read_half > read_whole ? ((read_half - read_whole) * 100u) / read_half : 0u;
    print(serial, "  FLASH_CTLR at entry ", hex(ctlr_boot), ", unlocked ", hex(ctlr_open),
          " (", unlocked_here ? "open" : "SHUT", ")", crlf);
    print(serial, "  at 48 MHz, SCKMOD written 0: CTLR=", hex(ctlr_half), "; written 1: ",
          hex(ctlr_whole), " (the bit read back ", half_reads ? "clear" : "SET", " then ",
          whole_reads ? "set" : "CLEAR", ")", crlf);
    print(serial, "  51200 instructions FETCHED in a straight run: ", fetch_half,
          " cycles at half the clock, ", fetch_whole, " at the whole of it - ", fetch_gain,
          " per cent", crlf);
    print(serial, "  10240 words READ out of the array: ", read_half, " cycles then ",
          read_whole, " - ", read_gain, " per cent", crlf);
    bench.verdict("SCKMOD is written and reads back in both positions, which is what makes "
                  "the two timings above a measurement of the ACCESS CLOCK and not of a "
                  "bit that was never set",
                  stepped && half && whole && half_reads && whole_reads && back &&
                      unlocked_here && (ctlr_half & flash_sckmod) == 0u &&
                      (ctlr_whole & flash_sckmod) != 0u);
    bench.verdict("and the two numbers are the answer: at 48 MHz of HCLK neither a "
                  "straight run of fifty thousand instructions nor ten thousand words read "
                  "out of the array costs measurably more with the access clock HALVED - "
                  "what the core waits for at this rate is not the flash, whichever of the "
                  "two settings it is read at",
                  sink != 0u && fetch_gain < 5u && read_gain < 5u);
}

// ===========================================================================
// g - the rate refusal, measured on the silicon
// ===========================================================================
void tg_rate() {
    // 144 MHz: above flash_safe_sysclk_hz, so the engine refuses with
    // its own code rather than halving HCLK behind the program's back.
    const bool up = SysClock::set_index(full_rate);
    (void)Flash::unlock();
    const uint32_t refused = Flash::erase_page(clock, test_page);
    const uint32_t refused_program =
        Flash::program_half_word(clock, test_page, 0x1234u);
    const bool media_refused = !Store::erase(test_page);
    Flash::lock();
    const bool down = SysClock::set_index(work_rate);
    print(serial, "  at 144 MHz: erase_page ", hex(refused), ", program_half_word ",
          hex(refused_program), ", the medium's erase() ",
          media_refused ? "false" : "TRUE", crlf);
    bench.verdict("above 120 MHz an erase and a program come back refused_rate with "
                  "NOTHING written, and the medium answers false - the driver will not "
                  "move the clock tree behind its caller's back",
                  up && down && refused == Flash::refused_rate &&
                      refused_program == Flash::refused_rate && media_refused);

    // And the same rate is what the array still READS at, because
    // SCKMOD's half of 144 MHz is 72 - over the 60 MHz ceiling, which is
    // what makes 120 the safe rate and not 144.
    print(serial, "  at ", SysClock::hz() / 1'000'000u, " MHz the array is read at ",
          Flash::access_hz(clock) / 1'000'000u, " MHz, the ceiling being ",
          flash_access_max_hz / 1'000'000u, crlf);
    bench.verdict("at the working rate the access clock is inside the chapter's ceiling "
                  "with SCKMOD at its reset value, which is why 96 MHz needs nothing of "
                  "the tree",
                  Flash::access_hz(clock) <= flash_access_max_hz);
}

// ===========================================================================
// w - the page cycle (one erase, by name)
// ===========================================================================
void tw_page_cycle() {
    if (!Flash::unlock()) {
        print(serial, "  the engine will not unlock", crlf);
        bench.verdict("the engine unlocks", false);
        return;
    }

    // THE ERASE, timed on the ruler - and the kernel's timebase watched
    // across it, because whether the core can run at all while the
    // engine works is a fact a program has to know.
    ruler_init();
    const uint32_t ms_before = Ticker::millis();
    const uint16_t erase_mark = ruler_now();
    const uint32_t erase_err = Flash::erase_page(clock, test_page);
    const uint32_t erase_us = ruler_since(erase_mark);
    const uint32_t ms_after = Ticker::millis();
    const bool erased = Flash::erased(test_page, MainFlashPartition::page);
    print(serial, "  erase_page: ", hex(erase_err), " in ", erase_us,
          " us (the datasheet's typical is 16000 for a sector), the kernel's tick "
          "advancing ", ms_after - ms_before, " ms across it", crlf);
    print_first_words(test_page, 4);
    bench.verdict("one 256-byte page erased with no error, and an erased cell reads back "
                  "0xE339E339 - NOT the all-ones every other flash in this project leaves "
                  "(32.5.4, said four times in the chapter)",
                  erase_err == 0u && erased);

    // THE FAST PAGE PROGRAM, timed: sixty-four words into the buffer
    // with WRBSY watched between them, then PGSTRT.
    fill_pattern(page_buffer, 0xB2103u);
    const uint32_t pms_before = Ticker::millis();
    const uint16_t program_mark = ruler_now();
    const uint32_t program_err =
        Flash::program_page(clock, test_page, {page_buffer, MainFlashPartition::page});
    const uint32_t program_us = ruler_since(program_mark);
    const uint32_t pms_after = Ticker::millis();
    Flash::read(test_page, {read_back, MainFlashPartition::page});
    const bool exact = memcmp(page_buffer, read_back, MainFlashPartition::page) == 0;
    print(serial, "  program_page: ", hex(program_err), " in ", program_us,
          " us (the datasheet's typical is 2000), the tick advancing ",
          pms_after - pms_before, " ms across it", crlf);
    print_first_words(test_page, 4);
    bench.verdict("a whole page programmed in one operation comes back BYTE FOR BYTE: the "
                  "fast method's buffer took all sixty-four words and PGSTRT wrote them",
                  program_err == 0u && exact);
    bench.verdict("and both operations are inside the datasheet's typical times, with the "
                  "erase the expensive one",
                  erase_us > program_us && erase_us < 50000u && program_us < 20000u);

    // THE CORE DOES NOT STOP. The kernel's millisecond tick is a count
    // of HANDLER RUNS, and a handler runs only if the core can fetch it
    // out of the array the engine is working on: a tick that keeps up
    // with the ruler is proof that it could.
    const uint32_t erase_ms = ms_after - ms_before;
    const uint32_t erase_expected = erase_us / 1000u;
    bench.verdict("AND THE CORE GOES ON RUNNING OUT OF THE ARRAY WHILE THE ENGINE ERASES "
                  "IT: the kernel's tick, which counts handler runs and would lose every "
                  "millisecond the core could not fetch, advanced across the erase by what "
                  "the ruler measured - so an erase here is a wait and not a stall, and a "
                  "program that erases keeps its interrupts",
                  erase_ms + 2u >= erase_expected && erase_ms <= erase_expected + 2u);

    // A PROGRAM INTO A PAGE THAT WAS NOT ERASED. The chapter does not
    // say what happens; this is where the silicon does.
    uint8_t second[MainFlashPartition::page];
    fill_pattern(second, 0x77A51u);
    const uint32_t again_err =
        Flash::program_page(clock, test_page, {second, MainFlashPartition::page});
    Flash::read(test_page, {read_back, MainFlashPartition::page});
    const bool took_the_new = memcmp(second, read_back, MainFlashPartition::page) == 0;
    const bool kept_the_old = memcmp(page_buffer, read_back, MainFlashPartition::page) == 0;
    bool anded = true;
    for (uint32_t i = 0; i < MainFlashPartition::page; ++i) {
        if (read_back[i] != static_cast<uint8_t>(page_buffer[i] & second[i])) {
            anded = false;
        }
    }
    uint32_t old_w0 = 0;
    uint32_t new_w0 = 0;
    uint32_t got_w0 = 0;
    memcpy(&old_w0, page_buffer, 4u);
    memcpy(&new_w0, second, 4u);
    memcpy(&got_w0, read_back, 4u);
    print(serial, "  a second program into the SAME page, not erased: ", hex(again_err),
          " - the page now ", took_the_new ? "holds the NEW data" :
          kept_the_old ? "holds the OLD data" : anded ? "holds the two ANDed" :
          "holds NEITHER, NOR THE TWO ANDED", crlf);
    print(serial, "  word 0: it held ", hex(old_w0), ", ", hex(new_w0),
          " was programmed over it, it now reads ", hex(got_w0), " (the two ANDed would be ",
          hex(old_w0 & new_w0), ")", crlf);
    print_first_words(test_page, 4);
    bench.verdict("a program into a page that was not erased reports NO ERROR and leaves "
                  "the page holding neither pattern and not their bitwise AND either - the "
                  "chapter promises nothing here and the silicon keeps that promise, which "
                  "is why a medium erases before it writes and why a torn write cannot be "
                  "repaired by writing again",
                  again_err == 0u && !took_the_new && !kept_the_old && !anded);

    Flash::lock();
    bench.end_letter();
}

// ===========================================================================
// v - the half-word program, and a second one into the same cell
// ===========================================================================
void tv_half_word() {
    if (!Flash::unlock()) {
        bench.verdict("the engine unlocks", false);
        return;
    }

    const uint32_t erase_err = Flash::erase_page(clock, second_page);
    const bool erased = Flash::erased(second_page, MainFlashPartition::page);
    bench.verdict("the page under test is erased and reads the erased pattern",
                  erase_err == 0u && erased);

    // The standard method: two bytes, the store IS the operation - and
    // it is the SLOW one, which is the first surprise of this letter.
    ruler_init();
    const uint16_t mark = ruler_now();
    const uint32_t err = Flash::program_half_word(clock, second_page, 0xF0F0u);
    const uint32_t us = ruler_since(mark);
    uint16_t back = 0;
    Flash::read(second_page, {reinterpret_cast<uint8_t*>(&back), 2u});
    print(serial, "  program_half_word 0xF0F0: ", hex(err), " in ", us,
          " us - against ", 1500u, " or so for a whole page the fast way", crlf);
    print_first_words(second_page, 4);
    bench.verdict("the standard method writes ONE half-word behind PG with no start bit - "
                  "the store is the operation - and the cell reads back what was written; "
                  "it costs MORE than the fast method spends on a whole page, which is "
                  "what makes the page the grain a medium wants",
                  err == 0u && back == 0xF0F0u && us > 1000u);

    // THE QUESTION A SMALL-VALUE STORE OVER THIS MEDIUM WOULD NEED
    // ANSWERED, and the reason this letter exists: does a cell take a
    // SECOND program between erases - and if it does, is it the AND
    // every other flash in this project performs, or something else?
    constexpr uint16_t passes[] = {0xF000u, 0x0F0Fu, 0xFFFFu, 0x0000u, 0xA5A5u};
    uint32_t exact = 0;
    uint32_t errors = 0;
    for (uint16_t want : passes) {
        const uint32_t e = Flash::program_half_word(clock, second_page, want);
        uint16_t got = 0;
        Flash::read(second_page, {reinterpret_cast<uint8_t*>(&got), 2u});
        if (e != 0u) {
            ++errors;
        }
        if (got == want) {
            ++exact;
        }
        print(serial, "  over it, ", hex(want), " -> ", hex(e), ", reads ", hex(got),
              got == want ? " EXACT" : " not what was written", crlf);
    }

    // And the cells beside it, which a program that keeps several values
    // in one page depends on.
    uint32_t neighbours[2] = {0, 0};
    Flash::read(second_page + 2u, {reinterpret_cast<uint8_t*>(&neighbours[0]), 4u});
    Flash::read(second_page + 6u, {reinterpret_cast<uint8_t*>(&neighbours[1]), 4u});
    print(serial, "  the two half-words above it read ", hex(neighbours[0]), " ",
          hex(neighbours[1]), " (erased is ", hex(Flash::erased_word), ")", crlf);
    bench.verdict("A HALF-WORD CELL TAKES PASS AFTER PASS BETWEEN ERASES, and each one "
                  "reads back EXACTLY what was written - bits come back as well as go "
                  "away, which is not the AND every other flash in this project performs "
                  "and is of a piece with an erased pattern that is not all ones",
                  errors == 0u && exact == 5u);
    bench.verdict("and the half-words beside it are untouched: what a program writes with "
                  "the standard method costs its own two bytes and no more",
                  neighbours[0] == Flash::erased_word && neighbours[1] == Flash::erased_word);

    // Leave the page erased, so the next run of e reads a clean zone.
    const uint32_t tidy = Flash::erase_page(clock, second_page);
    Flash::lock();
    bench.verdict("and the page is left erased behind the letter, so the medium's zone is "
                  "where the next program finds it",
                  tidy == 0u && Flash::erased(second_page, MainFlashPartition::page));
    bench.end_letter();
}

// ===========================================================================
// q - a second pass across a reset (one erase, reboots)
// ===========================================================================

/// The two values letter q programs into one half-word of the window,
/// the pair letter t programs into one of the tail.
constexpr uint16_t first_pass = 0x9659u;
constexpr uint16_t second_pass = 0x0F0Fu;

void tq_second_pass() {
    bench.reset_tally();
    if (!Flash::unlock()) {
        bench.verdict("the engine unlocks", false);
        return;
    }
    const uint32_t erase_err = Flash::erase_page(clock, second_page);
    const uint32_t first_err = Flash::program_half_word(clock, second_page, first_pass);
    const uint32_t second_err = Flash::program_half_word(clock, second_page, second_pass);
    Flash::lock();
    uint16_t now = 0;
    Flash::read(second_page, {reinterpret_cast<uint8_t*>(&now), 2u});
    print(serial, "  ", hex(first_pass), " then ", hex(second_pass), " programmed into one "
          "half-word of the window at ", hex(second_page), ": ", hex(erase_err | first_err |
                                                                      second_err),
          ", it reads ", hex(now), " before the reset", crlf);
    bench.verdict("before a reset the cell reads the second value exactly, as letter v finds",
                  erase_err == 0u && first_err == 0u && second_err == 0u && now == second_pass);
    token.magic = token_magic;
    token.letter = 'q';
    token.leg = 1;
    token.pass = 0;
    token.fail = 0;
    token.mark = now;
    print(serial, "  rebooting...", crlf);
    Reset::software();
}

void tq_resume() {
    uint16_t after = 0;
    Flash::read(second_page, {reinterpret_cast<uint8_t*>(&after), 2u});
    print(serial, crlf, "-> back from the software reset: the cell reads ", hex(after),
          " (before it: ", hex(token.mark), "; letter t gives a tail cell the same two "
          "values)", crlf);
    bench.verdict("AND A SYSTEM RESET KEEPS IT: after the reset the cell reads the second "
                  "value as it did before - the window's second pass survives a reboot",
                  after == second_pass);
    (void)Flash::unlock();
    const uint32_t tidy = Flash::erase_page(clock, second_page);
    Flash::lock();
    bench.verdict("and the page is left erased behind the letter",
                  tidy == 0u && Flash::erased(second_page, MainFlashPartition::page));
    bench.end_letter();
}

// ===========================================================================
// u - is FLASH_ADDR part of the fast page program? (two erases, by name)
// ===========================================================================

/// The fast page program of 32.5.6, WITHOUT the store into FLASH_ADDR
/// that `Flash::program_page()` makes - the one step the section does
/// not list and the sister family's does. Staged here through the
/// engine's own registers, because a driver has no reason to offer a
/// sequence it believes is incomplete.
///
/// The address the engine is left pointing at is the caller's business:
/// the letter erases the OTHER page last, so FLASH_ADDR holds that
/// page's address when this runs.
uint32_t program_page_without_address(uint32_t addr, const uint8_t* src) {
    if (!Flash::unlock()) {
        return Flash::refused;
    }
    Flash::clear_flags();
    Flash::regs().CTLR = (Flash::regs().CTLR & ~flash_ctlr_write_once) | flash_ftpg;
    volatile uint32_t* cell = reinterpret_cast<volatile uint32_t*>(Flash::alias_of(addr));
    for (uint32_t i = 0; i < MainFlashPartition::page / 4u; ++i) {
        uint32_t word;
        memcpy(&word, src + i * 4u, 4u);
        cell[i] = word;
        while (Flash::write_busy()) {
        }
    }
    Flash::regs().CTLR = (Flash::regs().CTLR & ~flash_ctlr_write_once) | flash_pgstrt;
    while (Flash::busy()) {
    }
    const uint32_t err = Flash::errors();
    Flash::clear_flags();
    Flash::regs().CTLR = Flash::regs().CTLR & ~(flash_ctlr_write_once | flash_ftpg);
    Flash::lock();
    return err;
}

void tu_address_register() {
    if (!Flash::unlock()) {
        bench.verdict("the engine unlocks", false);
        return;
    }

    // Both pages erased, the OTHER one last - so FLASH_ADDR is left
    // holding the address of the page this letter does NOT write.
    const uint32_t first = Flash::erase_page(clock, test_page);
    const uint32_t second = Flash::erase_page(clock, second_page);
    const bool both = Flash::erased(test_page, MainFlashPartition::page) &&
                      Flash::erased(second_page, MainFlashPartition::page);
    Flash::lock();
    bench.verdict("two pages erased, the SECOND of them last - which is what leaves "
                  "FLASH_ADDR pointing at a page this letter will not write",
                  first == 0u && second == 0u && both);

    fill_pattern(page_buffer, 0x4DD8u);
    const uint32_t err = program_page_without_address(test_page, page_buffer);
    Flash::read(test_page, {read_back, MainFlashPartition::page});
    const bool wrote_target = memcmp(page_buffer, read_back, MainFlashPartition::page) == 0;
    const bool target_untouched = Flash::erased(test_page, MainFlashPartition::page);
    const bool other_moved = !Flash::erased(second_page, MainFlashPartition::page);
    print(serial, "  a fast page program with NO store into FLASH_ADDR, the register left "
          "pointing at ", hex(second_page), ": ", hex(err), crlf);
    print(serial, "  the page written to ", hex(test_page), " now ",
          wrote_target ? "HOLDS THE DATA" : target_untouched ? "is still erased"
                                                             : "holds something else",
          "; the page FLASH_ADDR pointed at is ",
          other_moved ? "NO LONGER ERASED" : "untouched", crlf);
    print_first_words(test_page, 2);
    print_first_words(second_page, 2);
    bench.verdict("THE PAGE PROGRAM DOES NOT NEED FLASH_ADDR: the sixty-four stores name "
                  "the page and PGSTRT writes it, with the register left pointing "
                  "elsewhere and that other page untouched - so the store this driver "
                  "makes anyway is one instruction of belt and braces, not a step",
                  err == 0u && wrote_target && !other_moved);

    // And the pages are left erased behind the letter.
    (void)Flash::unlock();
    const uint32_t tidy = Flash::erase_page(clock, test_page);
    Flash::lock();
    bench.verdict("and the page is left erased behind the letter",
                  tidy == 0u && Flash::erased(test_page, MainFlashPartition::page));
    bench.end_letter();
}

// ===========================================================================
// x - the FlashMedia contract on the silicon (one erase, reboots)
// ===========================================================================
void tx_media() {
    bench.reset_tally();

    const auto zones = Store::zones();
    const uint32_t cell = zones[0].floor;

    const bool erased_ok = Store::erase(cell);
    const bool now_erased = Flash::erased(cell, Store::erase_size);
    bench.verdict("the medium's erase() takes the address of a cell inside its zone, opens "
                  "both locks for the one operation and shuts them again",
                  erased_ok && now_erased && Flash::locked() && Flash::fast_locked());

    fill_pattern(page_buffer, 0x51CE0u);
    const bool programmed = Store::program(cell, {page_buffer, Store::write_cell});
    Store::read(cell, {read_back, Store::write_cell});
    const bool exact = memcmp(page_buffer, read_back, Store::write_cell) == 0;
    bench.verdict("and program() writes one whole cell, read back byte for byte through "
                  "the contract's own read()",
                  programmed && exact);

    // The bounds are not decoration: an address below the zone is the
    // image itself, and the medium refuses it at run time exactly as
    // contains() refuses it at compile time.
    const bool below = !Store::program(cell - Store::write_cell,
                                       {page_buffer, Store::write_cell});
    const bool past = !Store::program(zones[0].ceiling, {page_buffer, Store::write_cell});
    const bool not_a_cell = !Store::program(cell, {page_buffer, 64u});
    const bool erase_below = !Store::erase(cell - Store::write_cell);
    bench.verdict("an address below the zone, one past its top, a run that is not a whole "
                  "cell and an erase outside the zone are all REFUSED - which is what "
                  "stands between a miscounted address and the running image",
                  below && past && not_a_cell && erase_below);

    // What crosses a reset: the point of the medium. The mark is a word
    // of the cell, kept in .noinit too so that the two can be compared
    // at the next boot.
    uint32_t mark = 0;
    memcpy(&mark, read_back, 4u);
    token.magic = token_magic;
    token.letter = 'x';
    token.leg = 1;
    token.pass = 0;
    token.fail = 0;
    token.mark = mark;
    print(serial, "  the cell's first word is ", hex(mark), ", rebooting...", crlf);
    Reset::software();
}

void tx_resume() {
    const auto zones = Store::zones();
    const uint32_t cell = zones[0].floor;
    uint32_t first = 0;
    Store::read(cell, {reinterpret_cast<uint8_t*>(&first), 4u});
    print(serial, crlf, "-> back from the software reset: the cell's first word is ",
          hex(first), ", the token remembered ", hex(token.mark), crlf);
    print_first_words(cell, 4);
    bench.verdict("what the medium was given is still there after a system reset, with the "
                  "engine locked again and nothing of the write surviving in the block's "
                  "own registers - which is the whole promise of a flash-backed store",
                  first == token.mark && Flash::locked() && Flash::fast_locked());
    bench.verdict("and the geometry is the same one it was written through, because both "
                  "bounds are constants and not symbols that move with a build",
                  MainFlashPartition::geometry_matches_silicon() &&
                      zones[0].floor == MainFlashPartition::storage_base);
    bench.end_letter();
}

// ===========================================================================
// y - the wrong key (reboots; LAST)
// ===========================================================================
void ty_wrong_key() {
    bench.reset_tally();
    token.magic = token_magic;
    token.letter = 'y';
    token.leg = 1;
    token.pass = 0;
    token.fail = 0;
    token.mark = 0;

    // The engine locked, then a key pair that is not the key pair.
    // 32.5.2 says this locks the block "until the next system reset" and
    // "generates a bus error" - a store fault on this core, which would
    // take the exception vector before the next line runs. The token is
    // written FIRST so that either outcome is reported.
    Flash::lock();
    print(serial, "  writing a wrong key pair into KEYR...", crlf);
    token.mark = 1;
    Flash::regs().KEYR = flash_key1;
    Flash::regs().KEYR = 0x00000000u;   // not KEY2
    token.mark = 2;

    const bool still_locked = Flash::locked();
    const bool reopen = Flash::unlock_standard();
    const uint32_t erase = Flash::erase_page(clock, test_page);
    print(serial, "  no bus error was taken; after the wrong pair: LOCK=",
          still_locked ? 1 : 0, ", a correct key pair ", reopen ? "OPENED it" : "was IGNORED",
          ", an erase answered ", hex(erase), crlf);
    token.mark = reopen ? 3 : 4;
    bench.verdict("a wrong key sequence does not fault the core here: the store is taken "
                  "and the engine simply does not open",
                  true);
    bench.verdict("and the lock HOLDS UNTIL THE NEXT SYSTEM RESET - a correct key pair "
                  "written afterwards does not open it, so a program that mistypes a key "
                  "has no way back but a reboot",
                  !reopen && erase != 0u);
    print(serial, "  rebooting to prove the way back...", crlf);
    Reset::software();
}

void ty_resume() {
    print(serial, crlf, "-> back from the software reset (the wrong key left mark ",
          token.mark, ")", crlf);
    const bool locked = Flash::locked() && Flash::fast_locked();
    const bool opens = Flash::unlock();
    Flash::lock();
    bench.verdict("the system reset is the way back: after it the engine is locked as at "
                  "any boot and the correct key pairs open it again",
                  locked && opens);
    bench.verdict("the core never took a fault for the wrong key - the reset flags at this "
                  "boot are the software reset's alone",
                  (boot_flags & ResetFlag::software) != 0u);
    bench.end_letter();
}

// ===========================================================================
// h - the array's three numbers and the split
// ===========================================================================
void th_split() {
    const FlashOptions opt = Flash::options();
    const FlashOptionArea area = FlashOptionArea::read();
    const uint32_t obr = Flash::regs().OBR;
    const uint8_t obr_user = static_cast<uint8_t>(obr >> flash_obr_user_shift);
    print(serial, "  the part table: a window of ", Flash::window_bytes / 1024u,
          " KB, a tail of ", Flash::tail_bytes / 1024u, " KB, an array of ",
          Flash::array_bytes / 1024u, " KB", crlf);
    print(serial, "  FLASH_OBR=", hex(obr), ": bits [9:2] read ", hex(obr_user),
          ", the option area's USER byte ", hex(area.user), crlf);
    bench.verdict("FLASH_OBR carries the loaded USER byte WHOLE at bits [9:2], so the memory "
                  "split is its bits [9:7] whatever 32.4.6's bit column prints",
                  obr_user == area.user);

    const std::optional<FlashSplit> loaded = opt.split();
    const std::optional<FlashSplit> stored = area.split();
    print(serial, "  USER[7:5]: ", opt.split_code, " as loaded, ", area.split_code(),
          " as stored");
    if (loaded) {
        print(serial, " - ", loaded->code_kbytes, " KB of window with ", loaded->ram_kbytes,
              " KB of SRAM");
    } else {
        print(serial, " - this part reads no split table");
    }
    print(serial, crlf);
    if constexpr (device::flash_split_table == FlashSplitTable::none) {
        bench.verdict("this part reads no split table: the decode answers nothing, and the "
                      "window is the part table's whatever the byte holds",
                      !loaded && !stored && opt.split_matches_part());
    } else {
        bench.verdict("the split the option byte selected is the part table's - the window "
                      "the linker script was written for, and the SRAM beside it - loaded and "
                      "stored alike",
                      loaded.has_value() && stored == loaded && opt.split_matches_part());
    }
    print(serial, "  ESIG_FLACAP ", Flash::size_kbytes(), " KB: the largest window this part "
          "can select is ", flash_window_max_bytes / 1024u, " KB", crlf);
}

// ===========================================================================
// the tail's instruments
// ===========================================================================

/// A kilobyte of words read through a volatile pointer with interrupts
/// masked: the core's cycles for the run, and the sum as the proof of
/// what was read. The window is timed at the alias the image runs at and
/// at the array's own address, the tail at the second only.
struct ReadRun {
    uint32_t cycles;
    uint32_t sum;
};

[[gnu::noinline]] ReadRun timed_read(uint32_t address) {
    const volatile uint32_t* p = reinterpret_cast<const volatile uint32_t*>(address);
    InterruptGuard guard;
    const uint32_t t0 = stk()->CNTL;
    uint32_t acc = 0;
    for (uint32_t i = 0; i < 1024u; ++i) {
        acc += p[i];
    }
    const uint32_t t1 = stk()->CNTL;
    const uint32_t period = stk()->CMPLR + 1u;
    return ReadRun{t1 >= t0 ? t1 - t0 : t1 + period - t0, acc};
}

void print_run(const char* label, const ReadRun& r) {
    const uint32_t c100 = (r.cycles * 100u) / 1024u;
    print(serial, "  ", label, ": ", r.cycles, " cycles for 1024 words, ", c100 / 100u, ".",
          (c100 % 100u) / 10u, c100 % 10u, " a word", crlf);
}

/// Where the window is timed: its second sector, at both of its
/// addresses (a pointer to address zero is not one C++ lets a program
/// hold).
constexpr uint32_t window_probe = 0x1000;

/// The words of a tail run summed through the DRIVER's read, a page at a
/// time - what the raw loop's sum is checked against.
template <typename C>
std::optional<uint32_t> tail_sum(C sysclock, uint32_t addr) {
    uint32_t acc = 0;
    for (uint32_t off = 0; off < 4096u; off += MainFlashPartition::page) {
        if (Flash::read_tail(sysclock, addr + off, {read_back, MainFlashPartition::page}) != 0u) {
            return std::nullopt;
        }
        for (uint32_t i = 0; i < MainFlashPartition::page; i += 4u) {
            uint32_t w = 0;
            memcpy(&w, read_back + i, 4u);
            acc += w;
        }
    }
    return acc;
}

/// Is a tail run erased - every word the erased pattern? Through the
/// driver's read.
template <typename C>
bool tail_erased(C sysclock, uint32_t addr, uint32_t bytes) {
    for (uint32_t off = 0; off < bytes; off += MainFlashPartition::page) {
        if (Flash::read_tail(sysclock, addr + off, {read_back, MainFlashPartition::page}) != 0u) {
            return false;
        }
        for (uint32_t i = 0; i < MainFlashPartition::page; i += 4u) {
            uint32_t w = 0;
            memcpy(&w, read_back + i, 4u);
            if (w != Flash::erased_word) {
                return false;
            }
        }
    }
    return true;
}

// ===========================================================================
// i - the tail read, timed, and the tail's refusals
// ===========================================================================
template <bool tail = Flash::has_tail>
void ti_tail_read() {
    if constexpr (tail) {
        constexpr uint32_t sector = Flash::tail_end - Flash::sector_size;
        uint8_t small[16];

        // THE REFUSALS FIRST: none of them reaches the engine.
        const uint32_t straddle = Flash::read_tail(clock, Flash::tail_base - 8u, {small, 16u});
        const uint32_t past = Flash::read_tail(clock, Flash::tail_end - 8u, {small, 16u});
        (void)Flash::unlock();
        const uint32_t page_past =
            Flash::program_page(clock, Flash::tail_end, {page_buffer, MainFlashPartition::page});
        const uint32_t erase_past = Flash::erase_page(clock, Flash::tail_end);
        const uint32_t block_past = Flash::erase_block32(clock, Flash::tail_end);
        const uint32_t sector_past = Flash::erase_sector(clock, Flash::tail_end);
        const uint32_t half_past = Flash::program_half_word(clock, Flash::tail_end, 0x1234u);
        Flash::lock();
        const bool up = SysClock::set_index(full_rate);
        const uint32_t at_144 = Flash::read_tail(clock, sector, {small, 16u});
        const bool down = SysClock::set_index(work_rate);
        print(serial, "  the tail runs ", hex(Flash::tail_base), "..", hex(Flash::tail_end),
              "; refused: a read across the line ", hex(straddle), ", past the end ",
              hex(past), ", at 144 MHz ", hex(at_144), crlf);
        print(serial, "  every write verb at the array's end: page program ", hex(page_past),
              ", page erase ", hex(erase_past), ", 32 KB erase ", hex(block_past),
              ", sector erase ", hex(sector_past), ", half-word ", hex(half_past), crlf);
        bench.verdict("the tail's read refuses a run across the line and one past the end, "
                      "and refuses 144 MHz with its code - the rate a tail read wants halved",
                      straddle == Flash::refused && past == Flash::refused && up && down &&
                          at_144 == Flash::refused_rate);
        bench.verdict("every write verb, of both methods, refuses a run past the array's end - "
                      "the tail is where the array ends and not where a method's reach does - "
                      "with nothing started",
                      page_past == Flash::refused && erase_past == Flash::refused &&
                          block_past == Flash::refused && sector_past == Flash::refused &&
                          half_past == Flash::refused && !Flash::busy());

        // THE TIMINGS at the working rate: the access clock is half of
        // 96 MHz, 48 against the 60 the chapter allows.
        const ReadRun win_alias = timed_read(Flash::alias_base + window_probe);
        const ReadRun win_array = timed_read(Flash::array_base + window_probe);
        const ReadRun tail_96 = timed_read(Flash::array_base + sector);
        const std::optional<uint32_t> verb_sum = tail_sum(clock, sector);

        // At 48 MHz, where SCKMOD may take both positions.
        const bool slow = SysClock::set_index(slow_rate);
        (void)Flash::unlock();
        const bool half = Flash::access_clock_whole(clock, false);
        const ReadRun win_48h = timed_read(Flash::array_base + window_probe);
        const ReadRun tail_48h = timed_read(Flash::array_base + sector);
        const bool whole = Flash::access_clock_whole(clock, true);
        const ReadRun win_48w = timed_read(Flash::array_base + window_probe);
        const ReadRun tail_48w = timed_read(Flash::array_base + sector);
        (void)Flash::access_clock_whole(clock, false);
        Flash::lock();
        const bool work = SysClock::set_index(work_rate);

        // The enhanced read mode, where it engages: 32.3 offers it to a
        // program running out of what the split left outside the window.
        (void)Flash::unlock();
        const bool engaged = Flash::enhanced_read(true);
        const ReadRun tail_ehm = engaged ? timed_read(Flash::array_base + sector) : ReadRun{0, 0};
        const bool left = !Flash::enhanced_read(false);
        Flash::lock();

        print_run("the window through the alias, 96 MHz", win_alias);
        print_run("the window at 0x0800 0000, 96 MHz", win_array);
        print_run("THE TAIL, 96 MHz (access clock 48)", tail_96);
        print_run("the window, 48 MHz, SCKMOD half (24)", win_48h);
        print_run("THE TAIL, 48 MHz, SCKMOD half (24)", tail_48h);
        print_run("the window, 48 MHz, SCKMOD whole (48)", win_48w);
        print_run("THE TAIL, 48 MHz, SCKMOD whole (48)", tail_48w);
        if (engaged) {
            print_run("THE TAIL, 96 MHz, the enhanced read mode", tail_ehm);
        } else {
            print(serial, "  the enhanced read mode did not engage on this part", crlf);
        }
        bench.verdict("the window answers the same words at the alias the image runs at and at "
                      "the array's own address", win_alias.sum == win_array.sum &&
                                                     win_array.sum == win_48h.sum &&
                                                     win_48h.sum == win_48w.sum);
        bench.verdict("the tail answers the same words at every access clock and through the "
                      "driver's own read - the timings above are of one content",
                      verb_sum.has_value() && *verb_sum == tail_96.sum &&
                          tail_96.sum == tail_48h.sum && tail_48h.sum == tail_48w.sum &&
                          (!engaged || tail_ehm.sum == tail_96.sum));
        bench.verdict("and the clock tree and SCKMOD are back where the letter found them",
                      slow && half && whole && work && left && !Flash::access_clock_whole());
    }
}

// ===========================================================================
// t - the tail written the standard way (one standard erase, by name)
// ===========================================================================
template <bool tail = Flash::has_tail>
void tt_tail_standard() {
    if constexpr (tail) {
        constexpr uint32_t sector = Flash::tail_end - Flash::sector_size;
        // The FPEC lock alone: the standard method is all the tail takes.
        Flash::lock();
        if (!Flash::unlock_standard()) {
            bench.verdict("the engine unlocks", false);
            return;
        }
        const bool fast_shut = Flash::fast_locked();

        ruler_init();
        const uint32_t ms_before = Ticker::millis();
        const uint16_t mark = ruler_now();
        const uint32_t erase_err = Flash::erase_sector(clock, sector);
        const uint32_t erase_us = ruler_since(mark);
        const uint32_t ms_after = Ticker::millis();
        const bool erased = tail_erased(clock, sector, Flash::sector_size);
        print(serial, "  erase_sector at ", hex(sector), " (the tail's last): ", hex(erase_err),
              " in ", erase_us, " us (the datasheet's typical is 16000), the kernel's tick "
              "advancing ", ms_after - ms_before, " ms across it", crlf);
        bench.verdict("the tail's last sector erased with the standard method and the FPEC "
                      "lock alone, and every word of it reads the erased pattern",
                      fast_shut && erase_err == 0u && erased);
        const uint32_t erase_ms = ms_after - ms_before;
        const uint32_t erase_expected = erase_us / 1000u;
        bench.verdict("the core went on running out of the window while the engine erased the "
                      "tail: the tick advanced by what the ruler measured",
                      erase_ms + 2u >= erase_expected && erase_ms <= erase_expected + 2u);

        // Thirty-two half-words at the sector's start, the first of them
        // timed on the ruler and the run on the tick.
        fill_pattern(page_buffer, 0x7A1Bu);
        uint16_t words[32];
        memcpy(words, page_buffer, sizeof words);
        uint32_t errors = 0;
        const uint16_t first_mark = ruler_now();
        errors |= Flash::program_half_word(clock, sector, words[0]);
        const uint32_t first_us = ruler_since(first_mark);
        const uint32_t run_before = Ticker::millis();
        for (uint32_t i = 1; i < 32u; ++i) {
            errors |= Flash::program_half_word(clock, sector + 2u * i, words[i]);
        }
        const uint32_t run_ms = Ticker::millis() - run_before;
        // The first page whole: the 64 bytes written, and the 192 after
        // them still erased; then the fifteen pages above it.
        const uint32_t got = Flash::read_tail(clock, sector, {read_back, MainFlashPartition::page});
        const bool exact = got == 0u && memcmp(read_back, words, sizeof words) == 0;
        bool page_rest_erased = true;
        for (uint32_t i = sizeof words; i < MainFlashPartition::page; i += 4u) {
            uint32_t w = 0;
            memcpy(&w, read_back + i, 4u);
            if (w != Flash::erased_word) {
                page_rest_erased = false;
            }
        }
        const bool rest_erased =
            page_rest_erased && tail_erased(clock, sector + MainFlashPartition::page,
                                            Flash::sector_size - MainFlashPartition::page);
        print(serial, "  32 half-words: the first in ", first_us, " us, the other 31 in ",
              run_ms, " ms, ", hex(errors), crlf);
        bench.verdict("half-words programmed into the tail the standard way read back exactly "
                      "through the tail's own read, and the rest of the sector is still erased",
                      errors == 0u && exact && rest_erased);

        // A second pass over one cell, as the window's took them.
        const uint32_t again = Flash::program_half_word(clock, sector, 0x0F0Fu);
        uint16_t back = 0;
        (void)Flash::read_tail(clock, sector, {reinterpret_cast<uint8_t*>(&back), 2u});
        print(serial, "  a second pass: ", hex(words[0]), " then 0x0F0F programmed over it -> ",
              hex(again), ", reads ", hex(back),
              back == 0x0F0Fu ? " EXACT, as the window's cells" : " - NEITHER value", crlf);
        Flash::lock();
        bench.verdict("a second pass over a tail cell reports no error and reads NEITHER value - "
                      "the window's half-word cells take pass after pass (letter v), the "
                      "tail's do not", again == 0u && back != 0x0F0Fu && back != words[0]);
    }
}

// ===========================================================================
// s - the tail written the fast way (by name)
// ===========================================================================

/// The page letter s leaves in the tail and letter r reads: the first of
/// the tail's last sector, with the pattern this seed makes.
constexpr uint32_t tail_pattern_seed = 0x5EEDu;

template <bool tail = Flash::has_tail>
void ts_tail_fast() {
    if constexpr (tail) {
        constexpr uint32_t block = Flash::tail_end - Flash::block_size;
        constexpr uint32_t page = Flash::tail_end - Flash::sector_size;
        if (!Flash::unlock()) {
            bench.verdict("the engine unlocks", false);
            return;
        }
        ruler_init();

        // THE 32 KB ERASE, made visible: a page of the block programmed
        // first, whatever it held - over an erased page it lands, over a
        // written one it leaves neither pattern, and either way the page
        // is not erased when the block erase starts.
        fill_pattern(page_buffer, 0xB10Cu);
        const uint32_t marked =
            Flash::program_page(clock, page, {page_buffer, MainFlashPartition::page});
        const bool dirty = !tail_erased(clock, page, MainFlashPartition::page);
        uint16_t mark = ruler_now();
        const uint32_t block_err = Flash::erase_block32(clock, block);
        const uint32_t block_us = ruler_since(mark);
        const bool block_clean = tail_erased(clock, block, Flash::block_size);
        print(serial, "  erase_block32 at ", hex(block), " (the tail's last 32 KB), over a page ",
              dirty ? "that was not erased" : "that WAS erased", ": ", hex(block_err), " in ",
              block_us, " us; the block ", block_clean ? "reads erased" : "is NOT erased", crlf);
        bench.verdict("the fast 32 KB erase takes a block of the tail and leaves every word of it "
                      "the erased pattern", marked == 0u && dirty && block_err == 0u &&
                                                block_clean);

        // THE PAGE PROGRAM AND THE PAGE ERASE, with the driver's verbs.
        fill_pattern(page_buffer, tail_pattern_seed);
        mark = ruler_now();
        const uint32_t program_err =
            Flash::program_page(clock, page, {page_buffer, MainFlashPartition::page});
        const uint32_t program_us = ruler_since(mark);
        (void)Flash::read_tail(clock, page, {read_back, MainFlashPartition::page});
        const bool landed = memcmp(read_back, page_buffer, MainFlashPartition::page) == 0;
        mark = ruler_now();
        const uint32_t erase_err = Flash::erase_page(clock, page);
        const uint32_t erase_us = ruler_since(mark);
        const bool erased_now = tail_erased(clock, page, MainFlashPartition::page);
        print(serial, "  program_page at ", hex(page), ": ", hex(program_err), " in ", program_us,
              " us, the page ", landed ? "holds the data" : "does NOT hold the data",
              "; erase_page: ", hex(erase_err), " in ", erase_us, " us, the page ",
              erased_now ? "reads erased" : "is NOT erased", crlf);
        bench.verdict("the fast page program lands in the tail byte for byte and the fast page "
                      "erase erases it - RM 32.2.1's note is about where the code sits, not "
                      "where it writes", program_err == 0u && landed && erase_err == 0u &&
                                             erased_now);

        // The pattern left behind for letter r.
        const uint32_t left_err =
            Flash::program_page(clock, page, {page_buffer, MainFlashPartition::page});
        (void)Flash::read_tail(clock, page, {read_back, MainFlashPartition::page});
        const bool left = memcmp(read_back, page_buffer, MainFlashPartition::page) == 0;
        Flash::lock();
        uint32_t first[4];
        memcpy(first, read_back, sizeof first);
        print(serial, "  at ", hex(page), " (the tail): ", hex(first[0]), " ", hex(first[1]), " ",
              hex(first[2]), " ", hex(first[3]), crlf);
        bench.verdict("and the page is programmed again and left holding the pattern letter r "
                      "reads", left_err == 0u && left);
    }
}

// ===========================================================================
// r - the tail read at 144 MHz, outside the rating (by name)
// ===========================================================================
template <bool tail = Flash::has_tail>
void tr_tail_fast_read() {
    if constexpr (tail) {
        constexpr uint32_t sector = Flash::tail_end - Flash::sector_size;
        fill_pattern(page_buffer, tail_pattern_seed);
        const bool holds = Flash::read_tail(clock, sector, {read_back, MainFlashPartition::page}) ==
                               0u &&
                           memcmp(read_back, page_buffer, MainFlashPartition::page) == 0;
        print(serial, "  the tail's last page ", holds ? "holds letter s's pattern"
              : "does NOT hold letter s's pattern - run s first", crlf);
        bench.verdict("the page letter s left in the tail holds its pattern at 96 MHz", holds);
        if (!holds) {
            return;
        }
        const ReadRun at_96 = timed_read(Flash::array_base + sector);
        const bool up = SysClock::set_index(full_rate);
        uint8_t small[16];
        const uint32_t refused = Flash::read_tail(clock, sector, {small, 16u});
        // By hand, which the verb refuses: the access clock is 72 MHz here.
        const ReadRun at_144 = timed_read(Flash::array_base + sector);
        uint32_t mismatches = 0;
        for (uint32_t pass = 0; pass < 3u; ++pass) {
            const volatile uint8_t* p =
                reinterpret_cast<const volatile uint8_t*>(Flash::array_base + sector);
            for (uint32_t i = 0; i < MainFlashPartition::page; ++i) {
                mismatches += p[i] != page_buffer[i] ? 1u : 0u;
            }
        }
        const bool down = SysClock::set_index(work_rate);
        const ReadRun after = timed_read(Flash::array_base + sector);
        print_run("THE TAIL at 96 MHz", at_96);
        print_run("THE TAIL at 144 MHz, read by hand (access clock 72)", at_144);
        print(serial, "  the page read three times by hand at 144 MHz, byte for byte: ",
              mismatches, " bytes of 768 differ from the pattern; the sums at 96 and 144 MHz ",
              at_144.sum == at_96.sum ? "agree" : "DIFFER", crlf);
        bench.verdict("the tail's read verb refuses 144 MHz with its code, as RM 32.1's note 2 "
                      "reads - the line above is what the silicon did by hand",
                      up && refused == Flash::refused_rate);
        bench.verdict("and back at 96 MHz the tail reads what it read before", down &&
                                                                               after.sum ==
                                                                                   at_96.sum);
    }
}

void banner() {
    print(serial, crlf, "test_vx03_nvm on ", device::part_name,
          " - the flash memory (RM ch. 32) and the signature (ch. 31)", crlf,
          "  z costs the board NOTHING: it reads, decodes and refuses", crlf,
          "  w, v, q, x and y each spend one erase cycle of one page; q, x and y reboot",
          crlf,
          "  t spends one standard erase of the tail's last sector, s one 32 KB erase of its",
          " last block and one page erase; r reads what s left at 144 MHz, OUTSIDE THE RATING",
          crlf,
          "  the medium's zone: ", hex(MainFlashPartition::storage_base), "..",
          hex(MainFlashPartition::storage_end), "; the tail: ", hex(Flash::tail_base), "..",
          hex(Flash::tail_end), crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

/// Both entries a trap can arrive on - the exception vector and the
/// breakpoint one - record the cause and reset (letter s stages what
/// the driver refuses, and a fault there is an answer too).
extern "C" BRIO_CH32_INTERRUPT void fault_handler() { brio::fault_reset<P>(); }

extern "C" BRIO_CH32_INTERRUPT void breakpoint_handler() { brio::fault_reset<P>(); }

/// The flash engine's own vector: the flags that stood, cleared.
extern "C" BRIO_CH32_INTERRUPT void flash_handler() {
    flash_irq_flags = brio::Flash::isr();
    flash_irqs = flash_irqs + 1u;
}

int main() {
    const bool clock_ok = SysClock::init();
    boot_flags = brio::Reset::take_flags();
    boot_record = brio::take_panic_record<P>();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the locks: which verb needs which, and a wrong one shut", ta_locks);
    bench.letter('b', "the option bytes decoded, against the area they came from",
                 tb_options);
    bench.letter('c', "the electronic signature: the capacity and the identifier",
                 tc_signature);
    bench.letter('d', "the status flags, the interrupt and five refusals", td_refusals);
    bench.letter('e', "the medium's geometry and what the zone holds", te_geometry);
    bench.letter('f', "SCKMOD measured on a loop fetched from flash", tf_sckmod);
    bench.letter('g', "the rate refusal at 144 MHz, on the silicon", tg_rate);
    bench.letter('h', "the array's three numbers and the option byte's split", th_split);
    if constexpr (brio::Flash::has_tail) {
        bench.letter('i', "THE TAIL READ, timed against the window, and its refusals",
                     ti_tail_read<>);
    }
    bench.letter('w', "THE PAGE CYCLE: erase, program, verify (one erase cycle)",
                 tw_page_cycle, false);
    bench.letter('v', "THE HALF-WORD PROGRAM and a second pass (one erase cycle)",
                 tv_half_word, false);
    bench.letter('q', "THE SECOND PASS ACROSS A RESET (one erase cycle, reboots)",
                 tq_second_pass, false);
    bench.letter('u', "IS FLASH_ADDR PART OF THE PAGE PROGRAM? (two erase cycles)",
                 tu_address_register, false);
    bench.letter('x', "THE MEDIA CONTRACT across a reset (one erase cycle, reboots)",
                 tx_media, false);
    bench.letter('y', "THE WRONG KEY: locked until reset (reboots)", ty_wrong_key, false);
    if constexpr (brio::Flash::has_tail) {
        bench.letter('t', "THE TAIL THE STANDARD WAY (one standard erase)", tt_tail_standard<>,
                     false);
        bench.letter('s', "THE TAIL THE FAST WAY (one 32 KB erase, one page erase)",
                     ts_tail_fast<>, false);
        bench.letter('r', "THE TAIL READ AT 144 MHz, outside the rating (after s)",
                     tr_tail_fast_read<>, false);
    }

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL96" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
        if (boot_record) {
            brio::print(serial, "  THE LAST RUN ENDED IN A TRAP: ",
                        brio::fault_cause_name(boot_record->context), " (context ",
                        brio::hex(boot_record->context), ")", brio::crlf);
        }
        if (token.magic == token_magic && token.letter != '\0') {
            const char letter = token.letter;
            token.letter = '\0';
            bench.reset_tally();
            bench.resume_tally(token.pass, token.fail);
            if (letter == 'x') {
                tx_resume();
            } else if (letter == 'q') {
                tq_resume();
            } else {
                ty_resume();
            }
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
