// test_avr_nvm - the NVMCTRL test SUITE for the AVR DA/DB target:
// avrdx/nvm.hpp (Flash, EEPROM, User Row, Signature Row, the section
// protections and the vector-table invariant) and the three services
// built on it - util/nv_record.hpp, util/nv_writer.hpp and
// util/persistent_panic.hpp.
//
// Reference test of those headers (docs/avrdx/nvm.md): keep it passing.
//
// THIS SUITE CHANGES THE CHIP. It erases and rewrites Flash pages, it
// erases and rewrites EEPROM bytes, and one letter erases the User Row.
// Everything it touches is either scratch (the Flash hole between the
// image and its read-only data, which the driver's own region provider
// computes) or is put back before the test ends. The wear each run
// costs is stated in nvm.md; the short version is two or three erase
// cycles on a handful of Flash pages and a few dozen EEPROM writes.
//
// IT NEEDS ITS FUSES. Under the shipping default (BOOTSIZE = 0) the
// whole Flash is one BOOT section and no software can write any of it,
// so the Flash legs report the geometry and skip. The standing bench
// geometry is BOOTSIZE = 128 (BOOT = the first 64 KB, where all the
// code is) and CODESIZE = 0; tools/bench.py's `fuses` verb writes it.
//
// Times are COUNTED IN CLK_PER CYCLES: a TCB pair (TCB1+TCB2) cascaded
// into one 32-bit counter at CLK_PER is the stopwatch, latched by a
// software event. It keeps counting while the CPU is halted by a Flash
// operation, which is what makes the stall measurable at all - the RTC
// timebase is the second, independent witness.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800. No pin is driven, nothing to wire.
// Event channels 4 (the cascade's carry) and 5 (its snapshot).
//
// Test f RESETS THE BOARD four times on purpose and carries its own
// verdicts across the resets in a .noinit token, so `z` still ends with
// one ALL: line. It is always run last.
//
// Commands: ? | a identity and geometry | b EEPROM primitives and times
// | c the record, the writer AO and the endurance policy | d Flash in
// the scratch region | e the CPU stall | f protections and the
// persistent panic across four REAL resets | z = a..f
// Not in z, because each costs more than a test should cost per run:
// u the User Row write path (one erase cycle of the row)
// g the APPDATA legs and the multi-page-erase erratum, which need a
//   temporary CODESIZE fuse (see the message the test prints)

// build: monitor_speed = 460800
// build: flmap_lock = 0

#include <avr/interrupt.h>
#include <stdint.h>
#include <optional>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/nvm.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/reset.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "kernel/panic.hpp"
#include "util/nv_writer.hpp"
#include "util/persistent_panic.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

/// The suite's own reset-surviving breadcrumb: test f spans four resets
/// and its verdicts, its tallies and what it expects to find have to
/// cross them, so `z` can still close with one ALL: line.
///
/// inline on purpose: gcc gives an INLINE variable with a custom
/// section attribute a COMDAT group and a plain one none, and the two
/// section types cannot be merged - the platform's own panic_record_ is
/// a static inline member, so this must be inline too.
struct Token {
    uint16_t magic;
    uint16_t canary;
    uint8_t phase;      ///< which reset we are waiting for (0 = none pending)
    uint8_t in_all;     ///< the run was `z`: close with the ALL: line
    uint8_t code;       ///< the PanicCode written before the reset
    uint8_t context;    ///< its context byte
    uint16_t all_pass;  ///< tallies banked by the tests before this one
    uint16_t all_fail;
    uint16_t f_pass;    ///< test f's own tallies so far
    uint16_t f_fail;
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using P = AvrPlatform;
using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

/// The fuse geometry this image is built for. Every compile-time Flash
/// verb below is checked against it, and test a checks it against the
/// silicon.
using Layout = FlashLayout<128, 0>;

/// The EEPROM map this suite claims. The panic slot first (it is the
/// one thing another program would also want at a known place), the
/// settings record after it, then a run of raw bytes for the primitives
/// and the writer AO.
using Store = EepromStore;
using Panic = PersistentPanic<Store, 0>;

struct Settings {
    uint16_t rate;
    uint8_t mode;
    uint8_t flags;
};
using SettingsRecord = NvRecord<Settings, Store, 16>;
constexpr uint16_t raw_base = 32;      ///< scratch EEPROM for b
constexpr uint16_t writer_base = 96;   ///< scratch EEPROM for the writer AO

// ---- the instrument ----------------------------------------------------------
using Alarm = Tcb<0>;                  ///< the interrupt whose latency e measures
using WatchLo = Tcb<1>;
using WatchHi = Tcb<2>;
using Watch = CascadedCounter<WatchLo, WatchHi>;
using ChCarry = EventChannel<4>;
using ChSnap = EventChannel<5>;

constexpr uint32_t crystal_hz = SysClock::hz;
constexpr uint32_t cycles_per_us = crystal_hz / 1'000'000u;

uint32_t cycles_now() { return Watch::read(); }

void stopwatch_init() {
    Watch::init(TcbClock::div1, ChCarry{}, ChSnap{});
    Watch::reset();
}

uint32_t us_of(uint32_t cycles) { return cycles / cycles_per_us; }

/// The cost of two back-to-back stamps: what the fine measurements
/// subtract, so what is reported is the operation and not the ruler.
uint32_t stamp_cost() {
    uint32_t best = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < 8; ++i) {
        P::CriticalSection cs;
        const uint32_t a = cycles_now();
        const uint32_t b = cycles_now();
        if (b - a < best) best = b - a;
    }
    return best;
}

// ---- shared with the ISRs -----------------------------------------------------
volatile uint8_t alarm_mode = 0;   ///< 0 off, 1 = e's latency alarm
volatile uint32_t alarm_stamp = 0;
volatile bool alarm_ran = false;
volatile uint16_t nvm_interrupts = 0;

// ---- the writer AO ------------------------------------------------------------
using Writer = NvWriter<Store, P, 2>;

/// The AO that asks for a write and counts the answer.
struct Requester {
    using Event = NvDone;
    static inline EventQueue<Event, 4, P> queue;
    static inline uint8_t replies = 0;
    static inline NvDone last{};
    static void init() { replies = 0; last = NvDone{}; }
    static void dispatch(const Event& e) {
        ++replies;
        last = e;
    }
};

// ---- the breadcrumb token -----------------------------------------------------
constexpr uint16_t token_magic = 0x4E56;    // "NV"
constexpr uint16_t token_canary = 0xC3A5;

ResetFlags boot_reset{};
std::optional<PanicRecord> boot_record;
std::optional<PanicRecord> boot_nv_record;

/// A reporter that stores the breadcrumb where a power cycle cannot
/// reach it, and then ends the panic in a software reset.
struct SaveAndResetReporter {
    [[noreturn]] static void report(PanicCode code, uint8_t context) {
        Panic::report(code, context);
        Reset::software();
    }
};

// ---- tiny test harness --------------------------------------------------------
uint16_t passed = 0, failed = 0;
bool in_all_run = false;

void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    delay_us(clock, 2000);
}

void quiesce() {
    alarm_mode = 0;
    Alarm::enable_capt_interrupt(false);
    Alarm::disable();
    Nvm::enable_eeprom_ready_interrupt(false);
    Nvm::clear_command();
    Nvm::clear_error();
    Ticker::init();
    stopwatch_init();
}

const char* error_name(NvmError e) {
    switch (e) {
        case NvmError::none: return "none";
        case NvmError::invalid_command: return "invalid_command";
        case NvmError::write_protect: return "write_protect";
        case NvmError::command_collision: return "command_collision";
        case NvmError::ongoing_program: return "ongoing_program";
    }
    return "?";
}

const char* section_name(FlashSection s) {
    switch (s) {
        case FlashSection::boot: return "BOOT";
        case FlashSection::appcode: return "APPCODE";
        case FlashSection::appdata: return "APPDATA";
    }
    return "?";
}

/// The scratch pages this suite uses, as offsets in pages from the
/// region's start. Fixed on purpose: the wear budget in nvm.md counts
/// erase cycles per page, and that only means something if the same
/// pages are used every run.
constexpr uint16_t page_words = flash_page_size / 2;
constexpr uint16_t scratch_page_single = 0;   ///< d: the single-page leg
constexpr uint16_t scratch_page_time = 1;     ///< e: the stall measurement
constexpr uint16_t scratch_page_mp2 = 2;      ///< d: the five multi-page legs
constexpr uint16_t scratch_page_mp4 = 4;
constexpr uint16_t scratch_page_mp8 = 8;
constexpr uint16_t scratch_page_mp16 = 16;
constexpr uint16_t scratch_page_mp32 = 32;
constexpr uint16_t scratch_page_latency = 40;  ///< e: four pages, one erase each

uint32_t scratch_page(uint16_t index) {
    return Nvm::scratch_region().begin +
           static_cast<uint32_t>(index) * flash_page_size;
}

bool scratch_usable() {
    const FlashRange r = Nvm::scratch_region();
    return !r.empty() && r.size() >= 64u * flash_page_size;
}

// ---- a identity and geometry ---------------------------------------------------
void ta_identity() {
    print(serial, "a identity, geometry, the fuses and the data-space window", crlf);
    quiesce();

    print(serial, "  SIGROW device id ", hex(Sigrow::device_id(0)), " ",
          hex(Sigrow::device_id(1)), " ", hex(Sigrow::device_id(2)),
          " (0x1E97xx = a 128 KB AVR Dx), silicon rev ", hex(SYSCFG.REVID), crlf);
    verdict("the device id starts with Atmel's 0x1E", Sigrow::device_id(0) == 0x1Eu);
    verdict("and the second byte says 128 KB (0x97)", Sigrow::device_id(1) == 0x97u);
    print(serial, "  serial number");
    bool serial_varies = false;
    for (uint8_t i = 0; i < Sigrow::serial_bytes; ++i) {
        const uint8_t b = Sigrow::serial(i);
        if (b != 0x00 && b != 0xFF) serial_varies = true;
        print(serial, " ", hex(b));
    }
    print(serial, crlf);
    verdict("the serial number is programmed, not blank", serial_varies);
    print(serial, "  TEMPSENSE0 ", Sigrow::tempsense0(), ", TEMPSENSE1 ",
          Sigrow::tempsense1(), crlf);
    verdict("the temperature calibration is programmed",
            Sigrow::tempsense0() != 0xFFFFu && Sigrow::tempsense1() != 0xFFFFu);

    verdict("the Flash is the header's 128 KB in 512-byte pages",
            flash_size == 131072u && flash_page_size == 512u);
    verdict("the EEPROM is 512 bytes at 0x1400",
            eeprom_size == 512u && eeprom_base == 0x1400u);
    verdict("the User Row is 32 bytes at 0x1080",
            userrow_size == 32u && userrow_base == 0x1080u);

    // The fuses, and the geometry they produce.
    print(serial, "  fuses: BOOTSIZE ", FUSE.BOOTSIZE, ", CODESIZE ", FUSE.CODESIZE,
          ", SYSCFG0 ", hex(FUSE.SYSCFG0), " (EESAVE ",
          (FUSE.SYSCFG0 & FUSE_EESAVE_bm) ? "set" : "clear", ")", crlf);
    print(serial, "  -> BOOT ends at ", Nvm::boot_end(), ", APPCODE ends at ",
          Nvm::appcode_end(), "; address 0 is ",
          section_name(Nvm::section_of(0)), ", 0x10000 is ",
          section_name(Nvm::section_of(0x10000u)), crlf);
    const bool geometry = Layout::matches_fuses();
    verdict("the silicon's fuses are the ones this image is built for", geometry);
    if (!geometry) {
        print(serial, "  the Flash legs of d, e and f will SKIP. Write the bench "
                      "geometry with: bench.py fuses A bootsize=128 codesize=0", crlf);
    }
    verdict("code executes from BOOT, so BOOT is not writable",
            Nvm::section_of(0) == FlashSection::boot &&
                !Nvm::writable(0, flash_page_size));

    // The vector table invariant. src/glue/ivsel_boot.cpp set it in
    // .init3; every interrupt this suite has taken so far is the proof
    // that it had to be set.
    verdict("IVSEL is armed: the vectors are at the start of BOOT",
            Nvm::vectors_in_boot_armed());
    print(serial, "  (with BOOTSIZE ", FUSE.BOOTSIZE, " the hardware ",
          FUSE.BOOTSIZE == 0 ? "IGNORES the bit - the whole Flash is BOOT"
                             : "USES it, and without it every ISR would jump "
                               "into erased Flash",
          ")", crlf);

    // The scratch region.
    const FlashRange r = Nvm::scratch_region();
    print(serial, "  image: .text+.data load ends at ", Nvm::image_low_end(),
          ", .rodata loads at ", Nvm::rodata_load_start(), " .. ",
          Nvm::rodata_load_end(), crlf);
    print(serial, "  scratch region ", r.begin, " .. ", r.end, " (",
          r.size() / flash_page_size, " pages, ", r.size(), " bytes)", crlf);
    verdict("the read-only data really lives in its own Flash section, "
            "leaving a hole in the middle",
            Nvm::rodata_load_start() > Nvm::image_low_end() + flash_page_size);
    verdict("the scratch region starts at the BOOT boundary",
            !r.empty() ? r.begin == Nvm::boot_end() : !geometry);
    verdict("it is page aligned at both ends",
            r.begin % flash_page_size == 0 && r.end % flash_page_size == 0);
    verdict("it is big enough for the largest erase command (32 pages)",
            !geometry || scratch_usable());
    verdict("it is blank: nothing of the image is in it",
            !scratch_usable() || Nvm::flash_blank(r.begin, r.begin + 64u));

    // FLMAP. This image opts out of the project-wide lock so the field
    // can be moved here; every other image is linked with it locked.
    const uint8_t boot_map = Nvm::flmap();
    print(serial, "  FLMAP reads ", boot_map, " (the section .rodata was linked "
                  "into), FLMAPLOCK ", Nvm::flmap_locked() ? "SET" : "clear", crlf);
    verdict("the C runtime mapped the section .rodata was linked into",
            static_cast<uint32_t>(boot_map) * 32768u ==
                Nvm::rodata_load_start() / 32768u * 32768u);
    // The lock is one-way until a reset, so on a RERUN of this letter in
    // the same power-on it is already standing (this very letter set it).
    // The mobility half needs the field open: skip it then, instead of
    // failing tests whose precondition is a fresh boot.
    if (Nvm::flmap_locked()) {
        print(serial, "  FLMAPLOCK already set (an earlier run of this letter "
                      "locked it): the mobility half is skipped - reset or "
                      "reflash for the full letter", crlf);
    } else {
        verdict("this image was linked with FLMAPLOCK left open",
                !Nvm::flmap_locked());
        bool moved = true;
        for (uint8_t s = 0; s < 4; ++s) {
            if (!Nvm::set_flmap(s) || Nvm::flmap() != s) moved = false;
        }
        verdict("every one of the four sections can be selected while unlocked",
                moved);
        verdict("FLMAP is restored to the one .rodata needs",
                Nvm::set_flmap(boot_map) && Nvm::flmap() == boot_map);
    }
    Nvm::lock_flmap();
    verdict("FLMAPLOCK reads back after being set", Nvm::flmap_locked());
    verdict("and the field refuses to move from now on",
            !Nvm::set_flmap(boot_map == 0 ? 1 : 0) && Nvm::flmap() == boot_map);
    print(serial, "  the lock is one-way until a reset; every other brio image is "
                  "linked with it already set (-Wl,--defsym,__flmap_lock=1).", crlf);

    // The USERROW read side. Its write path costs an erase cycle of the
    // whole row, so it is letter u and not part of z.
    auto board = board_id();
    print(serial, "  USERROW label \"", board.empty() ? "?" : board.data(), "\"", crlf);
    verdict("the board carries a printable identity label", !board.empty());
    quiesce();
}

// ---- b EEPROM primitives and times ----------------------------------------------
void tb_eeprom() {
    print(serial, "b EEPROM: the two write commands, the erases, the times", crlf);
    quiesce();
    verdict("the controller is idle and error-free at the start",
            !Nvm::busy() && Nvm::error() == NvmError::none);
    verdict("EEREADY stands while the EEPROM is idle (it is a LEVEL flag)",
            Nvm::eeprom_ready_flag());

    // Erase-and-write: the honest "store this byte".
    const bool w0 = Nvm::eeprom_erase_write(raw_base, 0x5Au);
    const bool settled0 = Nvm::wait_eeprom();
    Nvm::clear_command();
    verdict("erase-and-write is accepted", w0 && settled0);
    verdict("and the byte reads back", Nvm::eeprom_read(raw_base) == 0x5Au);

    // A bare write can only clear bits.
    (void)Nvm::eeprom_write(raw_base, 0xFFu);
    (void)Nvm::wait_eeprom();
    Nvm::clear_command();
    verdict("a bare write cannot turn a zero back into a one",
            Nvm::eeprom_read(raw_base) == 0x5Au);
    (void)Nvm::eeprom_write(raw_base, 0x12u);
    (void)Nvm::wait_eeprom();
    Nvm::clear_command();
    verdict("but it can clear more bits (0x5A AND 0x12 = 0x12)",
            Nvm::eeprom_read(raw_base) == 0x12u);

    // Byte erase.
    (void)Nvm::eeprom_erase(raw_base);
    (void)Nvm::wait_eeprom();
    Nvm::clear_command();
    verdict("a byte erase leaves 0xFF", Nvm::eeprom_read(raw_base) == 0xFFu);

    // A block, then a 32-byte multi-byte erase over it.
    uint8_t pattern[32];
    for (uint8_t i = 0; i < 32; ++i) pattern[i] = static_cast<uint8_t>(0xA0 + i);
    const bool blk = Nvm::eeprom_write_block(raw_base, pattern, 32);
    bool block_ok = blk;
    for (uint8_t i = 0; i < 32; ++i) {
        if (Nvm::eeprom_read(static_cast<uint16_t>(raw_base + i)) != pattern[i]) {
            block_ok = false;
        }
    }
    verdict("a 32-byte block writes and reads back exactly", block_ok);
    (void)Nvm::eeprom_erase(raw_base, EepromErase::bytes32);
    (void)Nvm::wait_eeprom();
    Nvm::clear_command();
    bool all_blank = true;
    for (uint8_t i = 0; i < 32; ++i) {
        if (Nvm::eeprom_read(static_cast<uint16_t>(raw_base + i)) != 0xFFu) {
            all_blank = false;
        }
    }
    verdict("one 32-byte erase command clears all 32", all_blank);

    // The alignment rule: an erase span ignores the low address bits, so
    // the driver refuses an unaligned one instead of erasing elsewhere.
    verdict("an unaligned multi-byte erase is refused",
            !Nvm::eeprom_erase(static_cast<uint16_t>(raw_base + 4),
                               EepromErase::bytes8));
    verdict("and an offset past the array too",
            !Nvm::eeprom_write(eeprom_size, 0));

    // ---- times, against table 39-7 ----
    struct Row { const char* what; uint32_t cycles; };
    Row rows[4];
    uint32_t t0;

    // A bare write (EEWR) on an erased byte.
    (void)Nvm::eeprom_erase(raw_base);
    (void)Nvm::wait_eeprom();
    Nvm::clear_command();
    t0 = cycles_now();
    (void)Nvm::eeprom_write(raw_base, 0x00u);
    while (Nvm::eeprom_busy()) {
    }
    rows[0] = {"byte write (EEWR)", cycles_now() - t0};
    Nvm::clear_command();

    // Erase and write in one operation.
    t0 = cycles_now();
    (void)Nvm::eeprom_erase_write(raw_base, 0x77u);
    while (Nvm::eeprom_busy()) {
    }
    rows[1] = {"byte erase-and-write (EEERWR)", cycles_now() - t0};
    Nvm::clear_command();

    // A byte erase.
    t0 = cycles_now();
    (void)Nvm::eeprom_erase(raw_base);
    while (Nvm::eeprom_busy()) {
    }
    rows[2] = {"byte erase (EEBER)", cycles_now() - t0};
    Nvm::clear_command();

    // A 32-byte erase: the same command class over 32 times the data.
    t0 = cycles_now();
    (void)Nvm::eeprom_erase(raw_base, EepromErase::bytes32);
    while (Nvm::eeprom_busy()) {
    }
    rows[3] = {"32-byte erase (EEMBER32)", cycles_now() - t0};
    Nvm::clear_command();

    for (const Row& r : rows) {
        print(serial, "  ", r.what, ": ", us_of(r.cycles), " us (", r.cycles,
              " CLK_PER cycles)", crlf);
    }
    verdict("a bare byte write is the table's ~70 us",
            near(static_cast<int32_t>(us_of(rows[0].cycles)), 70, 25));
    verdict("erase-and-write is the table's ~10 ms",
            near(static_cast<int32_t>(us_of(rows[1].cycles)), 10070, 2000));
    verdict("a byte erase is the table's ~10 ms",
            near(static_cast<int32_t>(us_of(rows[2].cycles)), 10000, 2000));
    verdict("erasing 32 bytes costs the same as erasing one",
            near(static_cast<int32_t>(us_of(rows[3].cycles)),
                 static_cast<int32_t>(us_of(rows[2].cycles)), 1000));

    // The CPU is NOT halted by an EEPROM write: it is halted only by a
    // second NVM access while one is running (11.3.2.3.4).
    (void)Nvm::eeprom_erase(raw_base);
    (void)Nvm::wait_eeprom();
    Nvm::clear_command();
    uint32_t turns = 0;
    (void)Nvm::eeprom_erase_write(raw_base, 0x33u);
    while (Nvm::eeprom_busy()) {
        ++turns;
    }
    Nvm::clear_command();
    print(serial, "  the CPU turned a polling loop ", turns,
          " times while the byte was being written", crlf);
    verdict("the CPU keeps running through an EEPROM write", turns > 1000u);

    // ---- the error field ----
    Nvm::clear_error();
    verdict("ERROR reads none after a clear", Nvm::error() == NvmError::none);
    // A store into the array with no command selected: the controller
    // does nothing and says why (11.5.3).
    Nvm::clear_command();
    (void)Nvm::eeprom_poke(raw_base, 0x01u);
    const NvmError no_cmd = Nvm::error();
    print(serial, "  a store with no command selected -> ERROR ",
          error_name(no_cmd), crlf);
    verdict("storing with no command selected raises invalid_command",
            no_cmd == NvmError::invalid_command);
    verdict("and nothing was written", Nvm::eeprom_read(raw_base) == 0x33u);
    Nvm::clear_error();
    verdict("the error field clears by writing it to zero",
            Nvm::error() == NvmError::none);

    // Selecting a reserved code.
    Nvm::select(NvmCommand::reserved);
    const NvmError bad_cmd = Nvm::error();
    const NvmCommand read_back = Nvm::selected();
    Nvm::clear_command();
    Nvm::clear_error();
    print(serial, "  selecting the reserved code 0x7F -> CTRLA reads ",
          static_cast<uint8_t>(read_back), ", ERROR ", error_name(bad_cmd), crlf);

    // Tidy up: leave the scratch bytes erased.
    (void)Nvm::eeprom_erase(raw_base, EepromErase::bytes32);
    (void)Nvm::wait_eeprom();
    Nvm::clear_command();
    quiesce();
}

// ---- c the record, the writer AO, the endurance policy ---------------------------
void tc_services() {
    print(serial, "c the typed record, its endurance policy and the writer AO", crlf);
    quiesce();

    // Start from an erased record slot.
    for (uint16_t a = SettingsRecord::begin; a < SettingsRecord::end; ++a) {
        (void)Nvm::eeprom_erase(a);
        (void)Nvm::wait_eeprom();
    }
    Nvm::clear_command();
    verdict("an erased slot holds no record", !SettingsRecord::load().has_value());

    const Settings first{9600, 3, 0x0F};
    const uint16_t n1 = SettingsRecord::store(first);
    print(serial, "  first store wrote ", n1, " of ", nv_record_size<Settings>,
          " bytes", crlf);
    const std::optional<Settings> back = SettingsRecord::load();
    verdict("the record loads back", back.has_value());
    verdict("field by field", back && back->rate == first.rate &&
                                  back->mode == first.mode &&
                                  back->flags == first.flags);

    // The whole point: storing the same value costs nothing.
    const uint16_t n2 = SettingsRecord::store(first);
    print(serial, "  storing the same value again wrote ", n2, " bytes", crlf);
    verdict("an unchanged store writes NOTHING", n2 == 0);

    // One field changes: the field's bytes and the checksum, no more.
    Settings second = first;
    second.mode = 4;
    const uint16_t n3 = SettingsRecord::store(second);
    print(serial, "  changing one byte wrote ", n3, " bytes (the byte plus "
                  "whichever checksum bytes moved)", crlf);
    verdict("one changed field costs at most three bytes", n3 >= 1 && n3 <= 3);
    const std::optional<Settings> back2 = SettingsRecord::load();
    verdict("and the new value is what comes back", back2 && back2->mode == 4);

    // A corrupted payload byte is caught by the checksum, not returned.
    const uint8_t saved = Nvm::eeprom_read(SettingsRecord::begin + 4);
    (void)Nvm::eeprom_erase_write(SettingsRecord::begin + 4,
                                  static_cast<uint8_t>(saved ^ 0x01u));
    (void)Nvm::wait_eeprom();
    Nvm::clear_command();
    verdict("a single flipped payload bit invalidates the record",
            !SettingsRecord::load().has_value());
    (void)Nvm::eeprom_erase_write(SettingsRecord::begin + 4, saved);
    (void)Nvm::wait_eeprom();
    Nvm::clear_command();
    verdict("and putting the byte back makes it valid again",
            SettingsRecord::valid());

    // ---- the writer AO: one byte per interrupt, nothing waits ----
    for (uint8_t i = 0; i < 8; ++i) {
        (void)Nvm::eeprom_erase(static_cast<uint16_t>(writer_base + i));
        (void)Nvm::wait_eeprom();
    }
    Nvm::clear_command();

    static const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    Requester::init();
    Writer::init();
    nvm_interrupts = 0;
    sei();

    const uint32_t t_start = cycles_now();
    const uint32_t tick_start = P::now();
    uint32_t loop_turns = 0;
    post<Writer>(NvWrite{writer_base, lend<Lease::reply>(payload), sizeof payload,
                         reply_to<Requester, NvDone>()});
    // The main loop the kernel would be running: it must keep turning
    // for the whole transfer, and every byte must arrive on an
    // interrupt, not on a wait inside a dispatch.
    for (uint32_t guard = 0; guard < 20'000'000u && Requester::replies == 0;
         ++guard) {
        ++loop_turns;
        if (const std::optional<Writer::Event> e = Writer::queue.pop()) {
            Writer::dispatch(*e);
        }
        if (const std::optional<Requester::Event> r = Requester::queue.pop()) {
            Requester::dispatch(*r);
        }
    }
    const uint32_t elapsed = cycles_now() - t_start;
    const uint32_t ticks = P::now() - tick_start;

    print(serial, "  8 bytes in ", us_of(elapsed) / 1000u, " ms, ", nvm_interrupts,
          " NVMCTRL interrupts, ", loop_turns, " main-loop turns, ", ticks,
          " timebase ticks kept coming", crlf);
    verdict("the request was answered", Requester::replies == 1);
    verdict("with nv_ok", Requester::last.status == nv_ok);
    verdict("and all eight bytes counted as written", Requester::last.written == 8);
    bool bytes_ok = true;
    for (uint8_t i = 0; i < 8; ++i) {
        if (Nvm::eeprom_read(static_cast<uint16_t>(writer_base + i)) !=
            payload[i]) {
            bytes_ok = false;
        }
    }
    verdict("the bytes are in the EEPROM", bytes_ok);
    verdict("one interrupt per byte", nvm_interrupts == 8);
    verdict("the main loop kept turning throughout (no busy wait anywhere)",
            loop_turns > 10000u);
    verdict("the 1024 Hz timebase never stopped", ticks > 50u);
    verdict("the ready interrupt is disarmed at the end",
            (NVMCTRL.INTCTRL & NVMCTRL_EEREADY_bm) == 0);

    // The same request again: nothing differs, so nothing is written and
    // no interrupt is needed at all.
    Requester::init();
    nvm_interrupts = 0;
    post<Writer>(NvWrite{writer_base, lend<Lease::reply>(payload), sizeof payload,
                         reply_to<Requester, NvDone>()});
    for (uint32_t guard = 0; guard < 100000u && Requester::replies == 0; ++guard) {
        if (const std::optional<Writer::Event> e = Writer::queue.pop()) {
            Writer::dispatch(*e);
        }
        if (const std::optional<Requester::Event> r = Requester::queue.pop()) {
            Requester::dispatch(*r);
        }
    }
    verdict("an unchanged run is answered without a single write",
            Requester::replies == 1 && Requester::last.written == 0 &&
                nvm_interrupts == 0);

    // A request that leaves the store is answered, not attempted.
    Requester::init();
    post<Writer>(NvWrite{static_cast<uint16_t>(eeprom_size - 2), lend<Lease::reply>(payload), 8,
                         reply_to<Requester, NvDone>()});
    for (uint32_t guard = 0; guard < 100000u && Requester::replies == 0; ++guard) {
        if (const std::optional<Writer::Event> e = Writer::queue.pop()) {
            Writer::dispatch(*e);
        }
        if (const std::optional<Requester::Event> r = Requester::queue.pop()) {
            Requester::dispatch(*r);
        }
    }
    verdict("a request past the end of the store is refused with nv_bad_range",
            Requester::replies == 1 && Requester::last.status == nv_bad_range);
    quiesce();
}

// ---- d Flash in the scratch region ------------------------------------------------
void td_flash() {
    print(serial, "d Flash: erase, word write, ELPM read-back, the five "
                  "multi-page spans", crlf);
    quiesce();
    if (!scratch_usable()) {
        print(serial, "  no writable scratch region under these fuses "
                      "(BOOTSIZE ", FUSE.BOOTSIZE, "): SKIPPED. Write the bench "
                      "geometry with bench.py fuses A bootsize=128 codesize=0", crlf);
        return;
    }
    const uint32_t page = scratch_page(scratch_page_single);
    print(serial, "  working on the page at ", page, " (", section_name(
              Nvm::section_of(page)), ")", crlf);

    verdict("the page is inside APPCODE, which BOOT may write",
            Nvm::section_of(page) == FlashSection::appcode &&
                Nvm::writable(page, page + flash_page_size));
    verdict("a single-page erase is accepted", Nvm::erase(page));
    verdict("and the whole page reads 0xFF",
            Nvm::flash_blank(page, page + flash_page_size));
    verdict("no error was raised", Nvm::error() == NvmError::none);

    // Word writes and the ELPM read-back, at both ends of the page.
    verdict("a word write is accepted",
            Nvm::write_word(page, 0x1234u) &&
                Nvm::write_word(page + flash_page_size - 2, 0xBEEFu));
    verdict("both words read back through ELPM",
            Nvm::flash_read_word(page) == 0x1234u &&
                Nvm::flash_read_word(page + flash_page_size - 2) == 0xBEEFu);
    verdict("and byte by byte, low byte first",
            Nvm::flash_read(page) == 0x34u && Nvm::flash_read(page + 1) == 0x12u);
    verdict("a write only clears bits: 0xFFFF over 0x1234 changes nothing",
            Nvm::write_word(page, 0xFFFFu) &&
                Nvm::flash_read_word(page) == 0x1234u);
    verdict("but more bits can be cleared",
            Nvm::write_word(page, 0x1030u) &&
                Nvm::flash_read_word(page) == 0x1030u);

    // A block write under one command selection.
    uint8_t block[16];
    for (uint8_t i = 0; i < 16; ++i) block[i] = static_cast<uint8_t>(0x10 + i);
    const uint32_t at = page + 64;
    verdict("a 16-byte block write is accepted",
            Nvm::write_block(at, block, sizeof block));
    bool block_ok = true;
    for (uint8_t i = 0; i < 16; ++i) {
        if (Nvm::flash_read(at + i) != block[i]) block_ok = false;
    }
    verdict("and reads back byte for byte", block_ok);

    // The refusals.
    verdict("an odd word address is refused", !Nvm::write_word(page + 1, 0));
    verdict("a write into BOOT is refused by the driver", !Nvm::write_word(0, 0));
    verdict("an address past the device is refused",
            !Nvm::write_word(flash_size, 0));
    verdict("an unaligned page erase is refused", !Nvm::erase(page + 2));
    verdict("a 32-page erase at a 2-page boundary is refused",
            !Nvm::erase(page + flash_page_size * 2, FlashErase::pages32));
    verdict("and none of those raised a hardware error",
            Nvm::error() == NvmError::none);

    // The five multi-page spans, each on its own aligned block so the
    // wear is spread over the region instead of piling on page 0.
    struct Span { FlashErase span; uint16_t first_page; };
    const Span spans[5] = {
        {FlashErase::pages2, scratch_page_mp2},
        {FlashErase::pages4, scratch_page_mp4},
        {FlashErase::pages8, scratch_page_mp8},
        {FlashErase::pages16, scratch_page_mp16},
        {FlashErase::pages32, scratch_page_mp32},
    };
    bool all_spans = true;
    for (const Span& s : spans) {
        const uint32_t base = scratch_page(s.first_page);
        const uint32_t bytes =
            static_cast<uint32_t>(erase_pages(s.span)) * flash_page_size;
        // Mark the first and last page of the block, then erase it all
        // with ONE command and check both marks are gone.
        (void)Nvm::write_word(base, 0x0000u);
        (void)Nvm::write_word(base + bytes - 2, 0x0000u);
        const bool marked = Nvm::flash_read_word(base) == 0 &&
                            Nvm::flash_read_word(base + bytes - 2) == 0;
        const bool erased = Nvm::erase(base, s.span);
        const bool blank = Nvm::flash_blank(base, base + 4) &&
                           Nvm::flash_blank(base + bytes - 4, base + bytes);
        if (!marked || !erased || !blank) all_spans = false;
        print(serial, "  ", erase_pages(s.span), " pages at ", base, ": marked ",
              marked ? "yes" : "no", ", erased ", erased ? "yes" : "no",
              ", blank ", blank ? "yes" : "no", crlf);
    }
    verdict("all five multi-page erase spans take down exactly their block",
            all_spans);

    // The error the controller is supposed to raise when a write is
    // aimed at a section the CPU may not write. The guarded verb refuses
    // it; the unguarded one lets the silicon answer.
    Nvm::clear_error();
    verdict("erase() refuses a BOOT address before the silicon sees it",
            !Nvm::erase(0) && Nvm::error() == NvmError::none);
    (void)Nvm::erase_ignoring_protection(0);
    const NvmError wp = Nvm::error();
    print(serial, "  the same erase without the driver's guard -> ERROR ",
          error_name(wp), crlf);
    verdict("the silicon calls it write_protect", wp == NvmError::write_protect);
    verdict("and BOOT is untouched (this suite is still running)",
            Nvm::flash_read_word(0) != 0xFFFFu);
    Nvm::clear_error();

    // Leave the scratch blank.
    (void)Nvm::erase(page);
    quiesce();
}

// ---- e the CPU stall ---------------------------------------------------------------
void te_stall() {
    print(serial, "e what a Flash operation costs the rest of the system", crlf);
    quiesce();
    if (!scratch_usable()) {
        print(serial, "  no writable scratch region under these fuses: SKIPPED", crlf);
        return;
    }
    const uint32_t page = scratch_page(scratch_page_time);
    (void)Nvm::erase(page);
    const uint32_t cost = stamp_cost();

    // Two independent witnesses. The TCB cascade runs on CLK_PER and
    // keeps counting while the CPU is halted, so it measures WALL TIME.
    // A software counter only advances while the CPU runs, so the
    // difference between the two is the CPU time that was LOST.
    volatile uint32_t spin = 0;
    const uint32_t reference_start = cycles_now();
    for (uint32_t i = 0; i < 20000u; ++i) {
        spin = spin + 1;
    }
    const uint32_t reference = cycles_now() - reference_start - cost;
    const uint32_t cycles_per_turn = reference / 20000u;
    print(serial, "  two back-to-back stamps cost ", cost,
          " CLK_PER cycles (subtracted below); the reference loop turns once "
          "every ", cycles_per_turn, crlf);

    struct Leg { const char* what; uint32_t wall; uint32_t ticks; };
    Leg legs[2];

    uint32_t t0 = cycles_now();
    uint32_t k0 = P::now();
    (void)Nvm::write_word(page, 0x00FFu);
    legs[0] = {"word write (FLWR)", cycles_now() - t0 - cost, P::now() - k0};

    t0 = cycles_now();
    k0 = P::now();
    (void)Nvm::erase(page);
    legs[1] = {"page erase (FLPER)", cycles_now() - t0 - cost, P::now() - k0};

    for (const Leg& l : legs) {
        print(serial, "  ", l.what, ": ", us_of(l.wall), " us wall (", l.wall,
              " CLK_PER cycles), the 1024 Hz timebase advanced ", l.ticks,
              " tick(s)", crlf);
    }
    verdict("a word write is in the table's tens of microseconds",
            us_of(legs[0].wall) >= 60u && us_of(legs[0].wall) <= 120u);
    verdict("a page erase is the table's ~10 ms",
            near(static_cast<int32_t>(us_of(legs[1].wall)), 10000, 2500));

    // The software timebase is NOT a second witness of wall time: the
    // PIT keeps counting in its own clock domain, but its interrupt can
    // only be SERVICED once the CPU is given back, and one pending
    // interrupt is one interrupt however long it waited. A 10 ms erase
    // therefore costs the tick counter ten of its eleven ticks.
    print(serial, "  10 ms of wall time is ", (10000u * P::ticks_per_second) /
                                                  1'000'000u,
          " ticks of the 1024 Hz timebase, and the counter got ",
          legs[1].ticks, crlf);
    verdict("a software timebase LOSES a page erase: the pending PIT "
            "interrupt collapses into one", legs[1].ticks <= 2u);

    // The CPU really was halted: a loop that would have turned N times
    // in that span turned none of them.
    const uint32_t during = legs[1].wall;
    const uint32_t would_have = during / (cycles_per_turn ? cycles_per_turn : 1u);
    print(serial, "  the CPU executed nothing for ", us_of(during),
          " us: a loop that turns every ", cycles_per_turn,
          " cycles would have turned ", would_have, " times", crlf);
    verdict("a page erase halts the CPU for its whole duration",
            would_have > 1000u);

    // Interrupt latency across an operation. The alarm is due at a known
    // cycle; the ISR stamps the cascade itself, so what is measured is
    // how long the interrupt had to wait.
    Ticker::pause();
    alarm_mode = 1;
    (void)Alarm::init({.mode = TcbMode::periodic,
                       .clock = TcbClock::div1,
                       .compare = 24000});          // 1 ms
    Alarm::enable_capt_interrupt(true);

    uint32_t quiet_worst = 0, during_worst = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        cli();
        alarm_ran = false;
        Alarm::count(0);
        Alarm::clear_capt();
        uint32_t due = cycles_now() + 24000u;
        sei();
        while (!alarm_ran) {
        }
        uint32_t late = alarm_stamp - due;
        if (late > quiet_worst) quiet_worst = late;

        cli();
        alarm_ran = false;
        Alarm::count(0);
        Alarm::clear_capt();
        due = cycles_now() + 24000u;
        sei();
        // A different page each time: the wear budget in nvm.md counts
        // erase cycles per page, and four on one page would dominate it.
        (void)Nvm::erase(scratch_page(scratch_page_latency + i));
        while (!alarm_ran) {
        }
        late = alarm_stamp - due;
        if (late > during_worst) during_worst = late;
    }
    alarm_mode = 0;
    Alarm::enable_capt_interrupt(false);
    Alarm::disable();
    Ticker::resume();

    print(serial, "  interrupt latency: ", quiet_worst,
          " CLK_PER cycles on a quiet CPU, ", during_worst, " (", us_of(during_worst),
          " us) with a page erase in the way", crlf);
    verdict("an interrupt is answered in a handful of cycles when nothing "
            "is happening", quiet_worst < 100u);
    verdict("a page erase delays it by most of the erase time",
            during_worst > 24000u * 100u);
    print(serial, "  this is the price of writing Flash from a running system: "
                  "the whole machine stops, interrupts included.", crlf);

    (void)Nvm::erase(page);
    quiesce();
}

// ---- f protections and the persistent panic across real resets ---------------------
// This test spans four resets. Its state lives in `token`, a .noinit
// object, and every leg is entered from main() at boot.

void bank(uint8_t phase) {
    token.magic = token_magic;
    token.canary = token_canary;
    token.phase = phase;
    token.in_all = in_all_run ? 1 : 0;
    token.f_pass = passed;
    token.f_fail = failed;
}

void report_boot() {
    print(serial, "  boot: RSTFR=", hex(boot_reset.raw),
          boot_reset.power_on ? " POR" : "", boot_reset.brown_out ? " BOR" : "",
          boot_reset.external ? " EXT" : "", boot_reset.watchdog ? " WDT" : "",
          boot_reset.software ? " SW" : "", boot_reset.updi ? " UPDI" : "",
          boot_record ? "; RAM breadcrumb pending" : "; no RAM breadcrumb",
          boot_nv_record ? "; EEPROM breadcrumb pending" : "; no EEPROM breadcrumb",
          crlf);
}

void tf_protections() {
    print(serial, "f the one-way protections and a panic that survives "
                  "a reset in the EEPROM", crlf);
    report_boot();
    verdict("this boot names a reset source", boot_reset.any());
    verdict("nothing was left over from a previous run",
            !boot_record && !boot_nv_record);
    verdict("no protection bit is set after a reset",
            !Nvm::appcode_protected() && !Nvm::appdata_protected() &&
                !Nvm::boot_read_protected());
    print(serial, "  FLMAPLOCK is ", Nvm::flmap_locked() ? "SET" : "clear",
          " going in (test a sets it, and only a reset takes it off)", crlf);

    // Phase 1: panic into a reporter that stores the breadcrumb in the
    // EEPROM and then resets in software.
    token.code = static_cast<uint8_t>(PanicCode::assert_failed);
    token.context = 0x5A;
    bank(1);
    print(serial, "  panicking into a reporter that stores the record in the "
                  "EEPROM and resets (the board will reboot) ...", crlf);
    console_drain();
    panic<P, SaveAndResetReporter>(PanicCode::assert_failed, 0x5A);
}

void tf_resume() {
    passed = token.f_pass;
    failed = token.f_fail;
    in_all_run = token.in_all != 0;
    const uint8_t phase = token.phase;
    print(serial, crlf, "f (continued after reset ", phase, " of 3)", crlf);
    report_boot();

    if (phase == 1) {
        verdict("RSTFR names a SOFTWARE reset", boot_reset.software);
        verdict("the RAM breadcrumb survived", boot_record.has_value());
        verdict("and so did the EEPROM one", boot_nv_record.has_value());
        verdict("both carry the code panic() was given",
                boot_record && boot_nv_record &&
                    boot_record->code == token.code &&
                    boot_nv_record->code == token.code);
        verdict("and the context byte", boot_nv_record &&
                                            boot_nv_record->context == token.context);
        verdict("the EEPROM slot is consumed by the take: nothing is pending now",
                !Panic::pending());
        verdict("the reset cleared FLMAPLOCK as well", !Nvm::flmap_locked());

        // Phase 2: APPCODEWP. One-way until a reset, and the driver must
        // refuse a write to the protected section before the silicon
        // sees it.
        if (!scratch_usable()) {
            print(serial, "  no writable scratch region under these fuses: the "
                          "protection legs are SKIPPED", crlf);
        } else {
            const uint32_t page = scratch_page(scratch_page_single);
            verdict("a scratch write works before the protection is set",
                    Nvm::erase(page) && Nvm::write_word(page, 0x55AAu));
            Nvm::protect_appcode();
            verdict("APPCODEWP reads back", Nvm::appcode_protected());
            verdict("the driver now refuses every write to APPCODE",
                    !Nvm::write_word(page + 2, 0x1234u) && !Nvm::erase(page));
            verdict("and the word written before it is still there",
                    Nvm::flash_read_word(page) == 0x55AAu);
            Nvm::clear_error();
            (void)Nvm::erase_ignoring_protection(page);
            const NvmError e = Nvm::error();
            print(serial, "  the same erase without the guard -> ERROR ",
                  error_name(e), ", page ",
                  Nvm::flash_read_word(page) == 0x55AAu ? "intact" : "ERASED", crlf);
            verdict("the silicon refuses it too, with write_protect",
                    e == NvmError::write_protect);
            verdict("and the page is intact",
                    Nvm::flash_read_word(page) == 0x55AAu);
            Nvm::clear_error();
        }
        bank(2);
        print(serial, "  resetting to prove the protection is cleared by a "
                      "reset ...", crlf);
        console_drain();
        Reset::software();
    }

    if (phase == 2) {
        verdict("a reset cleared APPCODEWP", !Nvm::appcode_protected());
        if (scratch_usable()) {
            const uint32_t page = scratch_page(scratch_page_single);
            verdict("and the section is writable again", Nvm::erase(page));
        }

        // Phase 3: BOOTRP. It can only be written from code executing in
        // BOOT - which is all of this program - and it takes effect only
        // when execution LEAVES the BOOT section, which never happens
        // here. So it is set, read back, and cleared by the reset.
        Nvm::protect_boot_read();
        verdict("BOOTRP reads back after being set from BOOT code",
                Nvm::boot_read_protected());
        verdict("code in BOOT still reads BOOT (the protection acts on "
                "accesses from OTHER sections)",
                Nvm::flash_read_word(0) != 0xFFFFu);
        bank(3);
        print(serial, "  resetting to release BOOTRP ...", crlf);
        console_drain();
        Reset::software();
    }

    // phase 3: the last boot.
    verdict("a reset cleared BOOTRP", !Nvm::boot_read_protected());
    verdict("RSTFR names a SOFTWARE reset", boot_reset.software);
    verdict("no breadcrumb is left anywhere",
            !boot_record.has_value() && !boot_nv_record.has_value());
    verdict("the .noinit token crossed all three resets intact",
            token.canary == token_canary);
    verdict("IVSEL is still armed after every one of them",
            Nvm::vectors_in_boot_armed());
    token.magic = 0;
    token.phase = 0;
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
    if (in_all_run) {
        print(serial, "ALL: ", static_cast<uint16_t>(token.all_pass + passed),
              " pass, ", static_cast<uint16_t>(token.all_fail + failed), " fail",
              crlf);
    }
}

// ---- u the User Row write path -------------------------------------------------
// Not in z: the row is Flash-class storage and this test costs it one
// erase cycle, which is one of about a thousand.
void tu_userrow() {
    print(serial, "u the User Row write path (COSTS THE ROW ONE ERASE CYCLE)", crlf);
    quiesce();

    uint8_t saved[userrow_size];
    Nvm::userrow_read(0, saved, userrow_size);
    print(serial, "  the row before, so it can be put back by hand if this "
                  "run is interrupted:", crlf, "   ");
    for (uint8_t i = 0; i < userrow_size; ++i) {
        print(serial, " ", hex(saved[i]));
    }
    print(serial, crlf);
    console_drain();

    // A write into a byte that is already erased needs no erase: Flash
    // writes can always clear bits.
    uint8_t spare = 0xFF;
    for (uint8_t i = userrow_size; i-- > 0;) {
        if (saved[i] == 0xFFu) {
            spare = i;
            break;
        }
    }
    verdict("the row has an erased byte to write into", spare != 0xFFu);
    if (spare != 0xFFu) {
        verdict("writing one byte of the row is accepted",
                Nvm::userrow_write(spare, 0x5Au));
        verdict("and it reads back", Nvm::userrow_read(spare) == 0x5Au);
        verdict("no error was raised", Nvm::error() == NvmError::none);
        verdict("an offset past the 32 bytes is refused",
                !Nvm::userrow_write(userrow_size, 0));
    }

    // The erase takes the WHOLE row: there is only one page of it.
    verdict("erasing the row is accepted", Nvm::userrow_erase());
    bool blank = true;
    for (uint8_t i = 0; i < userrow_size; ++i) {
        if (Nvm::userrow_read(i) != 0xFFu) blank = false;
    }
    verdict("and it takes down all 32 bytes at once", blank);

    // Put the label back.
    const bool restored = Nvm::userrow_write_block(0, saved, userrow_size);
    bool same = restored;
    for (uint8_t i = 0; i < userrow_size; ++i) {
        if (Nvm::userrow_read(i) != saved[i]) same = false;
    }
    verdict("the row is written back byte by byte and matches what it held",
            same);
    verdict("board_id() reads the label again", !board_id().empty());
    print(serial, "  the label is \"", board_id().empty() ? "?" : board_id().data(),
          "\" again", crlf);
    quiesce();
}

// ---- g the APPDATA legs and the multi-page-erase erratum -------------------------
// Not in z: it needs a fuse geometry that this bench does not stand in.
void tg_appdata() {
    print(serial, "g APPDATA, APPDATAWP and the multi-page-erase erratum", crlf);
    quiesce();

    const uint32_t append = Nvm::appcode_end();
    const bool has_appdata = FUSE.BOOTSIZE != 0 && append < flash_size;
    print(serial, "  fuses: BOOTSIZE ", FUSE.BOOTSIZE, ", CODESIZE ", FUSE.CODESIZE,
          " -> APPCODE ends at ", append, ", APPDATA ",
          has_appdata ? "exists" : "does not exist", crlf);
    if (!has_appdata) {
        print(serial, "  SKIPPED. This test needs an APPDATA section ABOVE the "
                      "image. With .rodata loading at ", Nvm::rodata_load_start(),
              " .. ", Nvm::rodata_load_end(), ", write", crlf,
              "    bench.py fuses A bootsize=128 codesize=223", crlf,
              "  (APPEND = 223 * 512 = 114176: APPCODE ends mid-way through the "
              "last free page, so a 2-page erase block straddles the boundary "
              "- which is what the erratum needs), reset the board and run g "
              "again. Restore with bench.py fuses A codesize=0.", crlf);
        return;
    }
    verdict("an address above APPEND is APPDATA",
            Nvm::section_of(append) == FlashSection::appdata);
    verdict("and one below it is APPCODE",
            Nvm::section_of(append - 2) == FlashSection::appcode);

    // The two-page block that straddles the section boundary. Its first
    // page is APPCODE, its second is APPDATA - which is exactly the
    // shape the erratum is about.
    const uint32_t block = (append / (2u * flash_page_size)) * (2u * flash_page_size);
    const uint32_t lower = block;
    const uint32_t upper = block + flash_page_size;
    const bool straddles = Nvm::section_of(lower) == FlashSection::appcode &&
                           Nvm::section_of(upper) == FlashSection::appdata;
    print(serial, "  the 2-page erase block at ", block, " covers ", lower, " (",
          section_name(Nvm::section_of(lower)), ") and ", upper, " (",
          section_name(Nvm::section_of(upper)), ")", crlf);
    verdict("the block really straddles the section boundary", straddles);
    verdict("both pages are free Flash, above the image",
            lower > Nvm::rodata_load_end());
    if (!straddles || lower <= Nvm::rodata_load_end()) {
        print(serial, "  the geometry does not place a straddling block over free "
                      "Flash: the erratum leg is SKIPPED", crlf);
        return;
    }

    // Both pages writable while nothing is protected.
    verdict("both pages erase and write while nothing is protected",
            Nvm::erase(lower) && Nvm::erase(upper) &&
                Nvm::write_word(lower, 0x1111u) &&
                Nvm::write_word(upper, 0x2222u));

    // Protect APPDATA. The driver must refuse everything that reaches
    // into it, including the straddling multi-page erase.
    Nvm::protect_appdata();
    verdict("APPDATAWP reads back", Nvm::appdata_protected());
    verdict("the driver refuses a single-page erase in APPDATA",
            !Nvm::erase(upper));
    verdict("and refuses the straddling 2-page erase, whose FIRST page is "
            "legal", !Nvm::erase(block, FlashErase::pages2));
    verdict("while APPCODE is still writable",
            Nvm::erase(lower) && Nvm::write_word(lower, 0x3333u));

    // Now the silicon's own answer, on two pages nothing needs.
    Nvm::clear_error();
    (void)Nvm::erase_ignoring_protection(upper);
    const NvmError single = Nvm::error();
    const bool single_kept = Nvm::flash_read_word(upper) == 0x2222u;
    print(serial, "  unguarded SINGLE-page erase of the protected page -> ERROR ",
          error_name(single), ", page ", single_kept ? "intact" : "ERASED", crlf);
    verdict("the silicon honours APPDATAWP for a single-page erase",
            single == NvmError::write_protect && single_kept);

    Nvm::clear_error();
    (void)Nvm::erase_ignoring_protection(block, FlashErase::pages2);
    const NvmError multi = Nvm::error();
    const bool multi_kept = Nvm::flash_read_word(upper) == 0x2222u;
    print(serial, "  unguarded 2-PAGE erase over the same page -> ERROR ",
          error_name(multi), ", protected page ",
          multi_kept ? "intact" : "ERASED", crlf);
    verdict("DS80000915F 2.7.1 observed: the protected page is erased anyway "
            "because only the FIRST page of the range was checked", !multi_kept);
    print(serial, "  this is why Nvm::erase() validates every page of the range "
                  "itself - the refusals above are the work-around the errata "
                  "sheet does not have.", crlf);
    Nvm::clear_error();
    (void)Nvm::erase_ignoring_protection(lower);
    print(serial, "  APPDATAWP is one-way: only a reset clears it. Restore the "
                  "standing geometry with bench.py fuses A codesize=0.", crlf);
    quiesce();
}

// ---- the menu -----------------------------------------------------------------------

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'a', ta_identity}, {'b', tb_eeprom}, {'c', tc_services}, {'d', td_flash},
    {'e', te_stall},    {'f', tf_protections}, {'u', tu_userrow},
    {'g', tg_appdata},
};
constexpr char all_keys[] = "abcdef";

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void run_set(const char* keys) {
    uint16_t tp = 0, tf = 0;
    in_all_run = true;
    for (const char* k = keys; *k != 0; ++k) {
        for (const Test& t : tests) {
            if (t.key != *k) continue;
            token.all_pass = tp;      // banked before the test that may reset
            token.all_fail = tf;
            run(t.fn);
            tp = static_cast<uint16_t>(tp + passed);
            tf = static_cast<uint16_t>(tf + failed);
        }
    }
    in_all_run = false;
    print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
}

void help() {
    print(serial, "test_avr_nvm: a identity and geometry | b EEPROM primitives "
                  "and times | c record, writer AO, endurance | d Flash in the "
                  "scratch region | e the CPU stall | f protections and the "
                  "persistent panic across REAL resets    -> z = all of a..f",
          crlf,
          "  outside z (each costs more than a test should cost per run): "
          "u the User Row write path | g APPDATA and the multi-page-erase "
          "erratum (needs a temporary CODESIZE fuse)", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }

/// The EEPROM's ready interrupt. Its flag is a LEVEL: disabling the
/// interrupt here is not an optimization, it is the only thing that
/// stops the vector re-entering forever (11.5.4).
ISR(NVMCTRL_EE_vect) {
    brio::Nvm::eeready();
    nvm_interrupts = nvm_interrupts + 1;
    brio::post<Writer>(brio::NvReady{});
}

ISR(TCB0_INT_vect) {
    (void)Alarm::take_flags();
    if (alarm_mode == 1) {
        alarm_stamp = cycles_now();
        alarm_ran = true;
    }
}
// The cascade's two halves have vectors of their own; an unbound vector
// on this part is a jump to 0, which is a reset loop and not a crash.
ISR(TCB1_INT_vect) { (void)WatchLo::take_flags(); }
ISR(TCB2_INT_vect) { (void)WatchHi::take_flags(); }

int main() {
    // FIRST: the three things a later line would destroy. RSTFR
    // accumulates until someone clears it, and both breadcrumbs are
    // fetch-and-clear.
    boot_reset = Reset::take_flags();
    boot_record = take_panic_record<P>();
    boot_nv_record = Panic::take();

    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    stopwatch_init();
    sei();

    if (token.magic == token_magic && token.phase != 0) {
        tf_resume();
    } else {
        token.magic = 0;
        auto board = board_id();
        if (board.empty()) board = "?";
        print(serial, crlf, "test_avr_nvm - NVMCTRL test suite (board ", board,
              ", clk=", xtal ? "XTAL" : "OSCHF", " 24 MHz, silicon rev ",
              hex(SYSCFG.REVID), ", BOOTSIZE ", FUSE.BOOTSIZE, "/CODESIZE ",
              FUSE.CODESIZE, ")", crlf);
        report_boot();
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
