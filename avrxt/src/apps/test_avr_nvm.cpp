// test_avr_nvm - the NVMCTRL test SUITE for the avrxt target:
// avrxt/nvm.hpp (Flash, EEPROM, User Row, Signature Row, the section
// protections and the vector-table invariant) and the three services
// built on it - util/nv_record.hpp, util/nv_writer.hpp and
// util/persistent_panic.hpp.
//
// TWO ENGINES, ONE SUITE. On the AVR DA/DB (NVM version 2) every letter
// below runs as described. On the AVR EA (NVM version 3, EEPROM only:
// its flash and User Row self-programming are declined) the letters
// keep their keys and change their matter: a the EA's identity, the
// supply guard of DS80001048D 2.2.1 and the FLMAP window under CCP; b the
// page-buffer command model, the three page operations and their times;
// c the services on the store's run path (a run per page), each beside
// the byte path of a store without the run verb, the panic record's time
// on both; d and u print why they are skipped; e what an EEPROM operation
// costs the CPU and its interrupts, a record store's included; f the
// persistent panic and EEWP across two resets. No g, no m.
//
// Reference test of those headers (docs/avrxt/nvm.md): keep it passing.
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
// code is) and CODESIZE = 0; brio's `fuses` verb writes it. On the AVR
// EA the factory BOOTSIZE 0 stays, and the BODCFG fuse guards the EEPROM
// writes (letter a asserts it: brio fuses <board> bodcfg=0x4D).
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
// The verdicts' bounds are derived beside each one; times are counted
// in CLK_PER cycles of the board's crystal (24 MHz on the DA/DB boards,
// 20 MHz on the EA's).
//
// Test f RESETS THE BOARD three times on purpose (twice on the AVR EA)
// and carries its own verdicts across the resets in a .noinit token, so
// `z` still ends with one ALL: line. It is always run last.
//
// Commands: ? | a identity and geometry | b EEPROM primitives and times
// | c the record, the writer AO and the endurance policy | d Flash in
// the scratch region | e the CPU stall | f protections and the
// persistent panic across four REAL resets | z = a..f
// Not in z, because each costs more than a test should cost per run:
// u the User Row write path (one erase cycle of the row)
// g the APPDATA legs and the multi-page-erase erratum, which need a
//   temporary CODESIZE fuse (see the message the test prints)
// m (an AVR DA only, first after a reset) the data-space window's
//   protection check, DS80000882C 2.7.1: two erase cycles of one page

// build: boards = db48,da48,ea48
// build: monitor_speed = 460800
// build: flmap_lock = 0

#include <avr/interrupt.h>
#include <stdint.h>
#include <optional>

#include "avrxt/clock.hpp"
#include "avrxt/delay.hpp"
#include "avrxt/evsys.hpp"
#include "avrxt/nvm.hpp"
#include "avrxt/platform.hpp"
#include "avrxt/reset.hpp"
#include "avrxt/tcb.hpp"
#include "avrxt/ticker.hpp"
#include "avrxt/usart.hpp"
#include "avrxt/userrow.hpp"
#include "kernel/panic.hpp"
#include "util/nv_writer.hpp"
#include "util/persistent_panic.hpp"
#include "util/print.hpp"

#include "avr_board.hpp"

#if defined(NVMCTRL_FLBUSY_bm)
#include "avrxt/sleep.hpp"

/// The load address of the image's .rodata (the AVR64EA's linker script
/// defines it): letter a checks FLMAP against it.
extern "C" const char __rodata_load_start[];
#endif

// The board's HF clock: its source and rate are the board table's.
using SysClock = BoardClock;   // the board's HF source (avr_board.hpp)
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

#if !defined(NVMCTRL_FLBUSY_bm)
/// The fuse geometry this image is built for. Every compile-time Flash
/// verb below is checked against it, and test a checks it against the
/// silicon.
using Layout = FlashLayout<128, 0>;
#endif

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

#if defined(NVMCTRL_FLBUSY_bm)
/// The same EEPROM as a store WITHOUT the contract's run verb: the
/// writers serve it a byte at a time. It exists so the suite measures
/// the byte path beside the run path on one silicon.
struct EepromByteStore {
    EepromByteStore() = delete;
    static constexpr uint16_t size() { return EepromStore::size(); }
    static uint8_t read(uint16_t addr) { return EepromStore::read(addr); }
    static bool write(uint16_t addr, uint8_t value) { return EepromStore::write(addr, value); }
    static bool ready() { return EepromStore::ready(); }
    static bool wait_ready() { return EepromStore::wait_ready(); }
    static void finish() { EepromStore::finish(); }
    static void arm_ready_interrupt(bool on) { EepromStore::arm_ready_interrupt(on); }
    static bool ready_flag() { return EepromStore::ready_flag(); }
    static void clear_ready_flag() { EepromStore::clear_ready_flag(); }
};
static_assert(NvPacedStore<EepromByteStore> && !NvRunStore<EepromByteStore>);
static_assert(NvRunStore<Store>);
using BytePanic = PersistentPanic<EepromByteStore, 0>;

/// Letter e's record: twelve bytes straddling pages 13 (104..111) and
/// 14 (112..119), so its run path is two operations with a wait between.
struct Wide8 {
    uint8_t b[8];
};
constexpr uint16_t record_e = 108;
using RunRecordE = NvRecord<Wide8, Store, record_e>;
using ByteRecordE = NvRecord<Wide8, EepromByteStore, record_e>;
#endif

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
#if defined(NVMCTRL_FLBUSY_bm)
volatile bool alarm_saw_busy = false;   ///< e: an EEPROM operation ran at the stamp
volatile uint32_t alarm_last = 0;       ///< alarm_mode 2: the previous run's stamp
volatile uint32_t alarm_gap = 0;        ///< alarm_mode 2: the longest gap between runs
volatile uint8_t alarm_runs = 0;        ///< alarm_mode 2: runs so far
volatile uint8_t ready_target = 0;      ///< whose NvReady: 0 Writer, 1 ByteWriter
#endif
volatile uint16_t nvm_interrupts = 0;

// ---- the writer AO ------------------------------------------------------------
using Writer = NvWriter<Store, P, 2>;
#if defined(NVMCTRL_FLBUSY_bm)
using ByteWriter = NvWriter<EepromByteStore, P, 2>;
#endif

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
#if !defined(NVMCTRL_FLBUSY_bm)
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}
#endif

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    delay_us(clock, 2000);
}

void quiesce() {
#if defined(NVMCTRL_FLBUSY_bm)
    // CTRLA written while an operation runs is refused (nvm.hpp): let the
    // last one end before clear_command() below.
    (void)Nvm::wait_idle();
#endif
    alarm_mode = 0;
    Alarm::enable_capt_interrupt(false);
    Alarm::disable();
    Nvm::enable_eeprom_ready_interrupt(false);
    Nvm::clear_command();
    Nvm::clear_error();
    Ticker::init();
    stopwatch_init();
}

#if !defined(NVMCTRL_FLBUSY_bm)
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
#else
const char* error_name(NvmError e) {
    switch (e) {
        case NvmError::none: return "none";
        case NvmError::write_protect: return "write_protect";
        case NvmError::command_collision: return "command_collision";
        case NvmError::wrong_section: return "wrong_section";
    }
    return "?";
}
#endif

#if !defined(NVMCTRL_FLBUSY_bm)
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
constexpr uint16_t scratch_page_window = 44;   ///< m: the data-space window (DA only)

/// The DA family's errata carry the data-space window's protection bug
/// (DS80000882C 2.7.1); the DB's (DS80000915F) have no such item. The
/// die decides it (avrxt/device_tables.hpp).
constexpr bool window_erratum = erratum_da_window;

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
                      "geometry with: brio fuses <board> bootsize=128 codesize=0", crlf);
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

#else // the AVR EA: NVM version 3, the EEPROM alone

// ---- EA helpers -------------------------------------------------------------------

/// The EEPROM pages letter b works on: page 4 (32..39) and page 5
/// (40..47), inside the suite's raw area.
constexpr uint16_t page_a = raw_base;
constexpr uint16_t page_b = static_cast<uint16_t>(raw_base + eeprom_page_size);
/// Letter e's page (48..55) and letter f's (56..63).
constexpr uint16_t page_e = static_cast<uint16_t>(raw_base + 2 * eeprom_page_size);
constexpr uint16_t page_f = static_cast<uint16_t>(raw_base + 3 * eeprom_page_size);

bool page_reads(uint16_t at, const uint8_t* want, uint8_t n) {
    for (uint8_t i = 0; i < n; ++i) {
        if (Nvm::eeprom_read(static_cast<uint16_t>(at + i)) != want[i]) return false;
    }
    return true;
}
bool page_blank(uint16_t at) {
    for (uint8_t i = 0; i < eeprom_page_size; ++i) {
        if (Nvm::eeprom_read(static_cast<uint16_t>(at + i)) != 0xFFu) return false;
    }
    return true;
}
/// One 8-byte erase, waited out: the page reads 0xFF after.
bool erase_page(uint16_t at) {
    return Nvm::eeprom_erase(at, EepromErase::bytes8) && Nvm::wait_eeprom();
}
void print_page(const char* what, uint16_t at) {
    print(serial, "  ", what, " [", at, "..]:");
    for (uint8_t i = 0; i < eeprom_page_size; ++i) {
        print(serial, " ", hex(Nvm::eeprom_read(static_cast<uint16_t>(at + i))));
    }
    print(serial, crlf);
}
const char* command_name(NvmCommand c) {
    switch (c) {
        case NvmCommand::none: return "NOCMD";
        case NvmCommand::noop: return "NOOP";
        case NvmCommand::eeprom_page_write: return "EEPW";
        case NvmCommand::eeprom_page_erase_write: return "EEPERW";
        case NvmCommand::eeprom_page_erase: return "EEPER";
        case NvmCommand::eeprom_buffer_clear: return "EEPBCLR";
        case NvmCommand::reserved: return "reserved";
    }
    return "?";
}
/// |a - b| <= a / 100: two operation times that the silicon makes
/// equal, measured within one letter. The bound: both are timed by the
/// same crystal (50 ppm, common to both, so it cancels), each stamp
/// pair's own cost is the same in both legs, the wait loop polls every
/// ~8 cycles (a 0.01 % quantization of a 54 000-cycle erase), and the
/// controller's own timebase moves between two legs a few milliseconds
/// apart only with the die's temperature, which a letter does not move
/// by a degree (an internal RC oscillator drifts well under 0.5 % per
/// degree). One percent covers all of it twice over.
bool same_time(uint32_t a, uint32_t b) {
    const uint32_t d = a > b ? a - b : b - a;
    return d <= a / 100u;
}

// ---- a identity, geometry, the supply, the window ------------------------------
void ta_identity() {
    print(serial, "a identity, geometry, the fuses, the supply guard and the "
                  "data-space window", crlf);
    quiesce();

    print(serial, "  SIGROW device id ", hex(Sigrow::device_id(0)), " ",
          hex(Sigrow::device_id(1)), " ", hex(Sigrow::device_id(2)),
          " (0x1E961E = the AVR64EA), silicon rev ", hex(SYSCFG.REVID),
          " (0x21 = B1)", crlf);
    verdict("the device id starts with Atmel's 0x1E", Sigrow::device_id(0) == 0x1Eu);
    verdict("the second byte says 64 KB (0x96)", Sigrow::device_id(1) == 0x96u);
    verdict("and the third names the AVR64EA (0x1E)", Sigrow::device_id(2) == 0x1Eu);
    print(serial, "  serial number");
    bool serial_varies = false;
    for (uint8_t i = 0; i < Sigrow::serial_bytes; ++i) {
        const uint8_t b = Sigrow::serial(i);
        if (b != 0x00 && b != 0xFF) serial_varies = true;
        print(serial, " ", hex(b));
    }
    print(serial, crlf);
    verdict("the serial number is programmed, not blank", serial_varies);
    print(serial, "  TEMPSENSE0 (a signed gain) ", Sigrow::tempsense0(),
          ", TEMPSENSE1 ", Sigrow::tempsense1(), crlf);
    verdict("the temperature calibration is programmed",
            static_cast<uint16_t>(Sigrow::tempsense0()) != 0xFFFFu &&
                Sigrow::tempsense1() != 0xFFFFu);

    verdict("the Flash is the header's 64 KB in 128-byte pages",
            flash_size == 65536u && flash_page_size == 128u);
    verdict("the EEPROM is 512 bytes in 8-byte pages at 0x1400",
            eeprom_size == 512u && eeprom_page_size == 8u && eeprom_base == 0x1400u);
    verdict("the User Row is 64 bytes at 0x1080",
            userrow_size == 64u && userrow_base == 0x1080u);

    print(serial, "  fuses: BOOTSIZE ", FUSE.BOOTSIZE, ", CODESIZE ", FUSE.CODESIZE,
          ", SYSCFG0 ", hex(FUSE.SYSCFG0), " (EESAVE ",
          (FUSE.SYSCFG0 & FUSE_EESAVE_bm) ? "set" : "clear", "), BODCFG ",
          hex(FUSE.BODCFG), crlf);
    verdict("BOOTSIZE is the factory 0: the whole Flash is BOOT, and nothing "
            "in this image writes it", FUSE.BOOTSIZE == 0);

    // DS80001048D 2.2.1: on rev B1 an EEPROM write below 2.7 V may fail;
    // the BOD at BODLEVEL2 or above is the sheet's alternative.
    const uint8_t bod_a = BOD.CTRLA;
    print(serial, "  BOD: CTRLA ", hex(bod_a), " (ACTIVE ", (bod_a >> 2) & 3u,
          ", SLEEP ", bod_a & 3u, "), CTRLB ", hex(BOD.CTRLB), " (LVL ",
          BOD.CTRLB & 7u, ": 2 = BODLEVEL2, 2.60 V typical)", crlf);
    const bool guarded = Nvm::supply_guarded();
    verdict("DS80001048D 2.2.1: the BOD guards every EEPROM write "
            "(Nvm::supply_guarded)", guarded);
    if (!guarded) {
        print(serial, "  provision it: brio fuses <board> bodcfg=0x4D (BODLEVEL2, "
                      "continuous awake with the start-up wait, continuous asleep)", crlf);
    }

    verdict("IVSEL is armed (the hardware ignores it while the whole Flash is BOOT)",
            Nvm::vectors_in_boot_armed());

    // FLMAP. This image opts out of the project-wide lock so the field
    // can be moved here; .rodata lives in the upper 32 KB.
    const uint8_t boot_map = Nvm::flmap();
    const uint32_t rodata = pgm_get_far_address(__rodata_load_start);
    print(serial, "  FLMAP reads ", boot_map, ", FLMAPLOCK ",
          Nvm::flmap_locked() ? "SET" : "clear", "; .rodata loads at ", rodata, crlf);
    verdict("the C runtime mapped the half .rodata was linked into",
            static_cast<uint32_t>(boot_map & 1u) * 32768u == rodata / 32768u * 32768u);
    if (Nvm::flmap_locked()) {
        print(serial, "  FLMAPLOCK already set (an earlier run of this letter "
                      "locked it): the mobility half is skipped - reset or "
                      "reflash for the full letter", crlf);
    } else {
        verdict("this image was linked with FLMAPLOCK left open", !Nvm::flmap_locked());
        // A plain store, no key: FLMAP 1 -> 3, which maps the SAME half
        // on 64 KB (11.5.2), so .rodata stays readable whatever happens.
        const uint8_t before = NVMCTRL.CTRLB;
        NVMCTRL.CTRLB = static_cast<uint8_t>(before ^ 0x20u);
        const uint8_t after = NVMCTRL.CTRLB;
        print(serial, "  a plain store of ", hex(static_cast<uint8_t>(before ^ 0x20u)),
              " into CTRLB (", hex(before), ") reads back ", hex(after), crlf);
        verdict("a plain store does not move FLMAP: the whole of CTRLB is "
                "under CCP here (table 11-7)", after == before);
        bool moved = true;
        for (uint8_t s = 0; s < 4; ++s) {
            if (!Nvm::set_flmap(s) || Nvm::flmap() != s) moved = false;
        }
        const bool restored = Nvm::set_flmap(boot_map) && Nvm::flmap() == boot_map;
        verdict("the keyed set_flmap() selects every one of the four sections",
                moved);
        verdict("FLMAP is restored to the one .rodata needs", restored);
    }
    Nvm::lock_flmap();
    verdict("FLMAPLOCK reads back after being set", Nvm::flmap_locked());
    verdict("and the field refuses to move from now on",
            !Nvm::set_flmap(boot_map ^ 1u) && Nvm::flmap() == boot_map);

    // The User Row: read, never written from here.
    auto board = board_id();
    print(serial, "  USERROW label \"", board.empty() ? "?" : board.data(), "\"", crlf);
    verdict("the board carries a printable identity label", !board.empty());
    uint8_t row[4];
    Nvm::userrow_read(0, row, sizeof row);
    verdict("the driver reads the row the label is read from",
            !board.empty() && row[0] == static_cast<uint8_t>(board[0]) &&
                Nvm::userrow_read(0) == row[0]);
    quiesce();
}

// ---- b the EEPROM engine: the command model, the operations, the times -----------
void tb_eeprom() {
    print(serial, "b EEPROM: the page buffer's command model, the three page "
                  "operations, the chooser, the times and the engine's cost", crlf);
    quiesce();
    verdict("the controller is idle and error-free at the start",
            !Nvm::busy() && Nvm::error() == NvmError::none);
    verdict("EEREADY stands while the EEPROM is idle (it is a LEVEL flag)",
            Nvm::eeprom_ready_flag());
    verdict("an 8-byte erase leaves both pages blank",
            erase_page(page_a) && erase_page(page_b) && page_blank(page_a) &&
                page_blank(page_b));

    // ---- the command model ----
    // A store loads the buffer; the command written after it starts the
    // operation on the loaded bytes.
    Nvm::clear_command();
    Nvm::clear_error();
    Nvm::eeprom_load(page_a, 0x5Au);
    const bool started_by_store = Nvm::eeprom_busy();
    verdict("a store into the EEPROM starts nothing: it loads the page buffer",
            !started_by_store && Nvm::eeprom_read(page_a) == 0xFFu &&
                Nvm::error() == NvmError::none);
    Nvm::eeprom_commit(EepromOp::write);
    const bool started_by_command = Nvm::eeprom_busy();
    (void)Nvm::wait_eeprom();
    verdict("the command written after the load starts the operation at once",
            started_by_command);
    verdict("and writes the loaded byte", Nvm::eeprom_read(page_a) == 0x5Au);
    verdict("the command stays in CTRLA after the operation",
            Nvm::selected() == NvmCommand::eeprom_page_write);
    Nvm::eeprom_load(static_cast<uint16_t>(page_a + 1), 0x00u);
    verdict("a store with that command still selected only loads, too",
            !Nvm::eeprom_busy() &&
                Nvm::eeprom_read(static_cast<uint16_t>(page_a + 1)) == 0xFFu);
    // That load is a stray now. Every verb opens with EEPBCLR ...
    const bool w2 = Nvm::eeprom_write(static_cast<uint16_t>(page_a + 2), 0x33u);
    (void)Nvm::wait_eeprom();
    verdict("a verb opens with EEPBCLR: the stray load is not written",
            w2 && Nvm::eeprom_read(static_cast<uint16_t>(page_a + 1)) == 0xFFu &&
                Nvm::eeprom_read(static_cast<uint16_t>(page_a + 2)) == 0x33u);
    // ... because without it the stray rides along.
    Nvm::eeprom_load(static_cast<uint16_t>(page_a + 3), 0x00u);   // a stray
    Nvm::eeprom_load(static_cast<uint16_t>(page_a + 4), 0x44u);   // the intended byte
    Nvm::eeprom_commit(EepromOp::write);
    (void)Nvm::wait_eeprom();
    verdict("without EEPBCLR a stray load is written with the next operation",
            Nvm::eeprom_read(static_cast<uint16_t>(page_a + 3)) == 0x00u &&
                Nvm::eeprom_read(static_cast<uint16_t>(page_a + 4)) == 0x44u);
    // The AVR DA/DB order: the command first, then the store.
    (void)Nvm::eeprom_open();
    uint32_t t0 = cycles_now();
    Nvm::eeprom_commit(EepromOp::write);
    const bool empty_ran = Nvm::eeprom_busy();
    (void)Nvm::wait_eeprom();
    const uint32_t empty_cycles = cycles_now() - t0;
    Nvm::eeprom_load(static_cast<uint16_t>(page_a + 5), 0x55u);
    const bool started_late = Nvm::eeprom_busy();
    (void)Nvm::wait_eeprom();
    print(serial, "  a command on an empty buffer: busy ", empty_ran ? "yes" : "no",
          ", for ", us_of(empty_cycles), " us", crlf);
    verdict("the AVR DA/DB order - command, then store - writes nothing here",
            !started_late && Nvm::eeprom_read(static_cast<uint16_t>(page_a + 5)) == 0xFFu);
    (void)Nvm::eeprom_open();        // drop that load
    // CTRLA during an operation.
    Nvm::clear_error();
    (void)Nvm::eeprom_erase(page_b, EepromErase::bytes8);
    Nvm::clear_command();             // while the erase runs
    const NvmError collision = Nvm::error();
    const NvmCommand kept = Nvm::selected();
    (void)Nvm::wait_eeprom();
    print(serial, "  NOCMD written during an erase -> ERROR ", error_name(collision),
          ", CTRLA still ", command_name(kept), crlf);
    verdict("CTRLA written during an operation is refused with command_collision",
            collision == NvmError::command_collision);
    verdict("the command stays and the operation completes",
            kept == NvmCommand::eeprom_page_erase && page_blank(page_b));
    Nvm::clear_error();
    // One run fills a page: the loads made with EEPBCLR still selected
    // stay in the buffer.
    uint8_t pattern[8];
    for (uint8_t i = 0; i < 8; ++i) pattern[i] = static_cast<uint8_t>(0xA0u + i);
    const bool run8 = Nvm::eeprom_run(page_b, pattern, 8, EepromOp::write);
    (void)Nvm::wait_eeprom();
    verdict("one run writes all eight bytes of a page in one operation",
            run8 && page_reads(page_b, pattern, 8));
    verdict("and nothing in the sequence raised an error",
            Nvm::error() == NvmError::none);
    // The buffer is indexed by the low three address bits alone.
    (void)Nvm::eeprom_open();
    Nvm::eeprom_load(static_cast<uint16_t>(page_a + 6), 0x66u);   // page 4, offset 6
    Nvm::eeprom_load(static_cast<uint16_t>(page_b + 7), 0x77u);   // page 5, offset 7
    Nvm::eeprom_commit(EepromOp::erase_write);
    (void)Nvm::wait_eeprom();
    print_page("page 4", page_a);
    print_page("page 5", page_b);
    verdict("loads in two pages land in ONE page, the last load's, at the "
            "first load's offset",
            Nvm::eeprom_read(static_cast<uint16_t>(page_b + 6)) == 0x66u &&
                Nvm::eeprom_read(static_cast<uint16_t>(page_b + 7)) == 0x77u &&
                Nvm::eeprom_read(static_cast<uint16_t>(page_a + 6)) == 0xFFu);
    verdict("so a run that leaves its page is refused",
            !Nvm::eeprom_run(static_cast<uint16_t>(page_a + 6), pattern, 4,
                             EepromOp::write) &&
                !Nvm::eeprom_store_run(static_cast<uint16_t>(page_a + 6), pattern, 4));
    verdict("and so are an empty run and an offset past the array",
            !Nvm::eeprom_run(page_a, pattern, 0, EepromOp::write) &&
                !Nvm::eeprom_erase_write(eeprom_size, 0));
    verdict("an erase span that is not aligned to itself is refused",
            !Nvm::eeprom_erase(static_cast<uint16_t>(page_a + 4), EepromErase::bytes8) &&
                !Nvm::eeprom_erase(static_cast<uint16_t>(page_a + 1), EepromErase::bytes2));
    // A wake from Idle clears the buffer (11.3.5): the PIT wakes it.
    (void)Nvm::eeprom_open();
    Nvm::eeprom_load(static_cast<uint16_t>(page_a + 7), 0x00u);
    Sleep::enter(SleepMode::idle);
    Nvm::eeprom_commit(EepromOp::write);
    (void)Nvm::wait_eeprom();
    verdict("a wake from Idle clears the buffer: the load made before the "
            "sleep is not written",
            Nvm::eeprom_read(static_cast<uint16_t>(page_a + 7)) == 0xFFu);

    // ---- the three operations, and the chooser ----
    (void)erase_page(page_a);
    verdict("an erase-and-write stores a byte over a programmed one",
            Nvm::eeprom_write(page_a, 0x0Fu) && Nvm::wait_eeprom() &&
                Nvm::eeprom_erase_write(page_a, 0xA5u) && Nvm::wait_eeprom() &&
                Nvm::eeprom_read(page_a) == 0xA5u);
    verdict("a write stores a byte onto an erased one",
            Nvm::eeprom_write(static_cast<uint16_t>(page_a + 1), 0x12u) &&
                Nvm::wait_eeprom() &&
                Nvm::eeprom_read(static_cast<uint16_t>(page_a + 1)) == 0x12u);
    // EEPW over a programmed byte: outside the chapter's precondition, so
    // the chooser never does it - here only to say what the cell does.
    (void)Nvm::eeprom_write(static_cast<uint16_t>(page_a + 1), 0x5Au);
    (void)Nvm::wait_eeprom();
    print(serial, "  (a write of 0x5A over 0x12, outside the chapter's "
                  "precondition, reads back ",
          hex(Nvm::eeprom_read(static_cast<uint16_t>(page_a + 1))),
          "; 0x5A AND 0x12 = 0x12)", crlf);
    const uint8_t one_new = 0x21u;
    const uint8_t one_ff = 0xFFu;
    const uint8_t one_other = 0x5Au;
    const uint16_t c_at = static_cast<uint16_t>(page_a + 5);
    const bool s1 = Nvm::eeprom_store_run(c_at, &one_new, 1);
    const NvmCommand c1 = Nvm::selected();
    (void)Nvm::wait_eeprom();
    const bool v1 = Nvm::eeprom_read(c_at) == one_new;
    const bool s2 = Nvm::eeprom_store_run(c_at, &one_other, 1);
    const NvmCommand c2 = Nvm::selected();
    (void)Nvm::wait_eeprom();
    const bool v2 = Nvm::eeprom_read(c_at) == one_other;
    const bool s3 = Nvm::eeprom_store_run(c_at, &one_ff, 1);
    const NvmCommand c3 = Nvm::selected();
    (void)Nvm::wait_eeprom();
    const bool v3 = Nvm::eeprom_read(c_at) == one_ff;
    const bool s4 = Nvm::eeprom_store_run(c_at, &one_ff, 1);
    const bool busy4 = Nvm::eeprom_busy();
    print(serial, "  the chooser: onto 0xFF ", command_name(c1), ", over a byte ",
          command_name(c2), ", to 0xFF ", command_name(c3), crlf);
    verdict("store onto an erased byte is a write (EEPW)",
            s1 && v1 && c1 == NvmCommand::eeprom_page_write);
    verdict("store over a programmed byte is an erase-and-write (EEPERW)",
            s2 && v2 && c2 == NvmCommand::eeprom_page_erase_write);
    verdict("store of 0xFF is an erase (EEPER)",
            s3 && v3 && c3 == NvmCommand::eeprom_page_erase);
    verdict("store of what is already there starts nothing", s4 && !busy4);
    // A mixed page: some bytes equal, some erased, some programmed.
    uint8_t mixed[8];
    for (uint8_t i = 0; i < 8; ++i) {
        mixed[i] = Nvm::eeprom_read(static_cast<uint16_t>(page_a + i));
    }
    mixed[0] = 0x01u;   // over a programmed byte
    mixed[6] = 0x02u;   // onto an erased one
    const bool sm = Nvm::eeprom_store_run(page_a, mixed, 8);
    const NvmCommand cm = Nvm::selected();
    (void)Nvm::wait_eeprom();
    verdict("a mixed page run leaves exactly the bytes asked, in one EEPERW",
            sm && cm == NvmCommand::eeprom_page_erase_write &&
                page_reads(page_a, mixed, 8));
    // A block across the page boundary: one store run per page touched.
    uint8_t block[12];
    for (uint8_t i = 0; i < 12; ++i) block[i] = static_cast<uint8_t>(0x30u + i);
    const bool wb = Nvm::eeprom_write_block(static_cast<uint16_t>(page_a + 2), block,
                                            sizeof block);
    verdict("a 12-byte block across a page boundary writes and reads back",
            wb && page_reads(static_cast<uint16_t>(page_a + 2), block, sizeof block) &&
                !Nvm::eeprom_busy() && Nvm::selected() == NvmCommand::none);
    verdict("and a block past the array is refused",
            !Nvm::eeprom_write_block(static_cast<uint16_t>(eeprom_size - 4), block, 8));

    // ---- the times (table 35-8: 2 ms typical for a byte or page erase
    // and for a write, no maximum; an erase-and-write is not listed) ----
    const uint32_t cost = stamp_cost();
    uint8_t zeros[8] = {};
    struct Leg { const char* what; uint32_t cycles; };
    Leg legs[6];
    auto timed = [&](const char* what, bool started) -> Leg {
        const uint32_t start = cycles_now();
        (void)started;
        while (Nvm::eeprom_busy()) {
        }
        return Leg{what, cycles_now() - start};
    };
    // Each leg: the engine's call, then the wait; the call's own cycles
    // (tens, measured below) are not in the figure.
    (void)erase_page(page_b);
    legs[0] = timed("write 1 byte (EEPW)", Nvm::eeprom_write(page_b, 0x00u));
    legs[1] = timed("erase 1 byte (EEPER)", Nvm::eeprom_erase(page_b));
    legs[2] = timed("erase-and-write 1 byte (EEPERW)",
                    Nvm::eeprom_erase_write(page_b, 0x5Au));
    legs[3] = timed("erase 8 bytes (EEPER)", Nvm::eeprom_erase(page_b, EepromErase::bytes8));
    legs[4] = timed("write 8 bytes (EEPW)",
                    Nvm::eeprom_run(page_b, zeros, 8, EepromOp::write));
    legs[5] = timed("erase-and-write 8 bytes (EEPERW)",
                    Nvm::eeprom_run(page_b, pattern, 8, EepromOp::erase_write));
    for (const Leg& l : legs) {
        print(serial, "  ", l.what, ": ", us_of(l.cycles), " us (", l.cycles,
              " CLK_PER cycles)", crlf);
    }
    verdict("eight bytes cost what one does: write",
            same_time(legs[0].cycles, legs[4].cycles));
    verdict("eight bytes cost what one does: erase",
            same_time(legs[1].cycles, legs[3].cycles));
    verdict("eight bytes cost what one does: erase-and-write",
            same_time(legs[2].cycles, legs[5].cycles));
    verdict("an erase and a write take the same time",
            same_time(legs[1].cycles, legs[0].cycles));
    verdict("an erase-and-write is an erase plus a write",
            same_time(legs[2].cycles, legs[0].cycles + legs[1].cycles));
    verdict("and the page holds the run", page_reads(page_b, pattern, 8));

    // ---- the engine's own cost: CPU cycles from the call to its return,
    // the controller idle at the call (the operation runs on after).
    // Each call is timed with interrupts masked, like the stamp pair
    // whose cost is subtracted: a console interrupt inside the window
    // would be counted as the engine's. ----
    (void)erase_page(page_b);
    uint32_t a = 0, call1 = 0, call8 = 0, call_same = 0, call_store = 0;
    uint32_t clear_cost = 0, load_cost = 0;
    {
        P::CriticalSection cs;
        a = cycles_now();
        (void)Nvm::eeprom_erase_write(page_b, 0x11u);
        call1 = cycles_now() - a - cost;
    }
    (void)Nvm::wait_eeprom();
    {
        P::CriticalSection cs;
        a = cycles_now();
        (void)Nvm::eeprom_run(page_b, pattern, 8, EepromOp::erase_write);
        call8 = cycles_now() - a - cost;
    }
    (void)Nvm::wait_eeprom();
    {
        P::CriticalSection cs;
        a = cycles_now();
        (void)Nvm::eeprom_store_run(page_b, pattern, 8);
        call_same = cycles_now() - a - cost;
    }
    const uint8_t one = 0x11u;
    {
        P::CriticalSection cs;
        a = cycles_now();
        (void)EepromStore::write(page_b, one);
        call_store = cycles_now() - a - cost;
    }
    (void)Nvm::wait_eeprom();
    // The NVM accesses inside those calls, one at a time: the buffer
    // clear (its key and store, then the halt 11.3.2.4.5 puts at seven
    // cycles) and one buffer load (a store, one cycle by the instruction
    // set, plus whatever the bus adds).
    {
        P::CriticalSection cs;
        a = cycles_now();
        Nvm::issue(NvmCommand::eeprom_buffer_clear);
        clear_cost = cycles_now() - a - cost;
        a = cycles_now();
        Nvm::eeprom_load(page_b, 0xFFu);
        load_cost = cycles_now() - a - cost;
    }
    Nvm::issue(NvmCommand::eeprom_buffer_clear);
    Nvm::clear_command();
    print(serial, "  the engine's call, CPU cycles: one byte ", call1,
          ", an 8-byte run ", call8, ", an 8-byte store run with nothing to "
          "do ", call_same, ", EepromStore::write of one byte ", call_store,
          " (the operation itself runs on, the CPU free); inside them, an "
          "EEPBCLR costs ", clear_cost, " and one buffer load ", load_cost, crlf);
    // The bounds: the release listing counts 58 cycles for the one-byte
    // call (the EEPBCLR halt at the chapter's seven, a store at one) and
    // 143 for the 8-byte run; the NVM accesses add what the line above
    // measures. 100 and 250 hold both with room and still refuse the
    // three-call shape this path replaced (138 cycles for one byte).
    verdict("one operation's CPU cost is tens of cycles against its "
            "milliseconds", call1 < 100u && call_store < 100u && call8 < 250u);

    (void)erase_page(page_a);
    (void)erase_page(page_b);
    Nvm::clear_command();
    quiesce();
}

#endif // the EA's letters a and b

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
#if defined(NVMCTRL_FLBUSY_bm)
    uint32_t turn_last = t_start;
    uint32_t turn_gap = 0;   ///< the longest stretch between two turns
#endif
    post<Writer>(NvWrite{writer_base, lend<Lease::reply>(payload), sizeof payload,
                         reply_to<Requester, NvDone>()});
    // The main loop the kernel would be running: it must keep turning
    // for the whole transfer, and every byte must arrive on an
    // interrupt, not on a wait inside a dispatch.
    for (uint32_t guard = 0; guard < 20'000'000u && Requester::replies == 0;
         ++guard) {
        ++loop_turns;
#if defined(NVMCTRL_FLBUSY_bm)
        const uint32_t turn_now = cycles_now();
        if (turn_now - turn_last > turn_gap) turn_gap = turn_now - turn_last;
        turn_last = turn_now;
#endif
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
#if !defined(NVMCTRL_FLBUSY_bm)
    verdict("one interrupt per byte", nvm_interrupts == 8);
#else
    // The store takes a run: the eight bytes are one page, one operation.
    verdict("one interrupt for the page: the eight bytes are one operation",
            nvm_interrupts == 1);
#endif
#if !defined(NVMCTRL_FLBUSY_bm)
    verdict("the main loop kept turning throughout (no busy wait anywhere)",
            loop_turns > 10000u);
    verdict("the 1024 Hz timebase never stopped", ticks > 50u);
#else
    // A wait for an operation anywhere in a turn would show as a gap of
    // the operation (52 000 cycles for the shortest); a tenth of that
    // holds a dispatch, the two queue pops and an ISR or two.
    print(serial, "  the longest stretch between two main-loop turns: ", turn_gap,
          " cycles", crlf);
    verdict("the main loop kept turning throughout (no busy wait anywhere)",
            turn_gap < 5000u);
    // The PIT runs on OSC32K, within 10 % over the full range
    // (DS40002443A table 35-10), and each end of the window quantizes by
    // a tick: a held interrupt would lose two ticks per operation.
    const uint32_t expected_ticks = static_cast<uint32_t>(
        static_cast<uint64_t>(elapsed) * P::ticks_per_second / crystal_hz);
    verdict("the 1024 Hz timebase never stopped",
            ticks + 2u >= expected_ticks * 9u / 10u);
#endif
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
#if defined(NVMCTRL_FLBUSY_bm)
    // ---- the byte path: the same eight bytes through a store without the run verb ----
    // The writer above is on the store's run path: one operation, one
    // interrupt, one dispatch for the page. The same writer over
    // EepromByteStore (the same EEPROM, the run verb hidden) spends one
    // of each per byte: eight EEPWs onto the erased bytes.
    (void)Nvm::eeprom_erase(writer_base, EepromErase::bytes8);
    (void)Nvm::wait_eeprom();
    Nvm::clear_command();
    Requester::init();
    ByteWriter::init();
    ready_target = 1;
    nvm_interrupts = 0;
    const uint32_t byte_start = cycles_now();
    post<ByteWriter>(NvWrite{writer_base, lend<Lease::reply>(payload), sizeof payload,
                             reply_to<Requester, NvDone>()});
    for (uint32_t guard = 0; guard < 20'000'000u && Requester::replies == 0; ++guard) {
        if (const std::optional<ByteWriter::Event> e = ByteWriter::queue.pop()) {
            ByteWriter::dispatch(*e);
        }
        if (const std::optional<Requester::Event> r = Requester::queue.pop()) {
            Requester::dispatch(*r);
        }
    }
    const uint32_t byte_elapsed = cycles_now() - byte_start;
    ready_target = 0;
    bool byte_bytes = Requester::replies == 1 && Requester::last.status == nv_ok &&
                      Requester::last.written == 8;
    for (uint8_t i = 0; i < 8; ++i) {
        if (Nvm::eeprom_read(static_cast<uint16_t>(writer_base + i)) != payload[i]) {
            byte_bytes = false;
        }
    }
    print(serial, "  the same 8 bytes on the byte path: ", us_of(byte_elapsed), " us, ",
          nvm_interrupts, " interrupts (the run path: ", us_of(elapsed), " us, one)",
          crlf);
    verdict("the byte path stores the same eight bytes, one interrupt each",
            byte_bytes && nvm_interrupts == 8);
    // Both legs write onto erased bytes, so every operation is an EEPW of
    // page 12: the byte path's time is at least eight of the run path's
    // one; a quarter leaves a factor of two for the dispatches.
    verdict("the run path in under a quarter of the byte path's time",
            elapsed < byte_elapsed / 4u);

    // ---- the run path's CPU cost of one operation, masked ----
    // What the writer's dispatch and the record spend per run: the scan
    // (the first and last differing byte within the room) and the
    // store's write_run (the engine's store run: re-read, EEPBCLR, the
    // loads, the command). Eight differing bytes, onto erased cells.
    (void)Nvm::eeprom_erase(writer_base, EepromErase::bytes8);
    (void)Nvm::wait_eeprom();
    uint32_t t_scan = 0, t_start_run = 0;
    bool cost_ok = false;
    {
        const uint32_t ruler = stamp_cost();
        P::CriticalSection cs;
        const uint32_t c0 = cycles_now();
        const NvRunScan r = nv_scan_run<Store>(writer_base, payload, 0, sizeof payload);
        const uint32_t c1 = cycles_now();
        cost_ok = Store::write_run(static_cast<uint16_t>(writer_base + r.from),
                                   payload + r.from, r.len);
        const uint32_t c2 = cycles_now();
        t_scan = c1 - c0 - ruler;
        t_start_run = c2 - c1 - ruler;
        cost_ok = cost_ok && r.from == 0 && r.len == 8 && r.changed == 8;
        (void)Nvm::wait_eeprom();
    }
    Nvm::clear_command();
    print(serial, "  one run's CPU cost (8 bytes, masked): the scan ", t_scan,
          " cycles, write_run ", t_start_run, " cycles", crlf);
    verdict("the scan finds the whole page as one run and the store takes it",
            cost_ok && page_reads(writer_base, payload, 8));

    // ---- the panic record: one operation on the run path, eight on the byte path ----
    // Interrupts masked, as panic() leaves them. Page 0 is erased first
    // each time and the erase is the ruler: an EEPER and an EEPW of one
    // page take the same time to a few cycles (letter b), so the run
    // path is one ruler plus the reporter's own work, the byte path eight.
    uint32_t t_ref = 0, t_run = 0, t_byte = 0;
    bool panic_back = false, panic_byte_back = false;
    {
        P::CriticalSection cs;
        uint32_t t0 = cycles_now();
        (void)Nvm::eeprom_erase(0, EepromErase::bytes8);
        (void)Nvm::wait_eeprom();
        t_ref = cycles_now() - t0;
        t0 = cycles_now();
        Panic::report(PanicCode::assert_failed, 0x3C);
        t_run = cycles_now() - t0;
    }
    {
        const std::optional<PanicRecord> r = Panic::peek();
        panic_back = r && r->code == static_cast<uint8_t>(PanicCode::assert_failed) &&
                     r->context == 0x3C;
    }
    {
        P::CriticalSection cs;
        (void)Nvm::eeprom_erase(0, EepromErase::bytes8);
        (void)Nvm::wait_eeprom();
        const uint32_t t0 = cycles_now();
        BytePanic::report(PanicCode::assert_failed, 0x3C);
        t_byte = cycles_now() - t0;
    }
    {
        const std::optional<PanicRecord> r = Panic::take();
        panic_byte_back = r && r->code == static_cast<uint8_t>(PanicCode::assert_failed) &&
                          r->context == 0x3C && !Panic::pending();
    }
    print(serial, "  the panic record's report, interrupts masked: ", us_of(t_run),
          " us on the run path (", t_run, " cycles), ", us_of(t_byte),
          " us on the byte path (", t_byte, "); one operation of page 0: ",
          us_of(t_ref), " us (", t_ref, ")", crlf);
    verdict("the panic record reads back after the run path's report", panic_back);
    verdict("and after the byte path's, and is taken once", panic_byte_back);
    // One operation is t_ref to letter b's 1 % (same page, same kind of
    // operation) plus the reporter's work - the record built, its CRC,
    // eight reads, the store run: under 2000 cycles, 4 % of t_ref. Two
    // operations would be 1.98 t_ref at the least: 1.5 sits between.
    verdict("the run path's report is ONE operation",
            t_run + t_ref / 100u >= t_ref && t_run < t_ref + t_ref / 2u);
    // Eight operations, each t_ref to 1 %, each read of the next byte
    // held to the operation's end and 70..120 us beyond it (letter e: at
    // most 2400 cycles, 5 % of t_ref) seven times: under 8.4 t_ref; 9
    // leaves room, 7.5 sits a half operation below the floor.
    verdict("the byte path's is eight",
            t_byte > t_ref * 15u / 2u && t_byte < t_ref * 9u);
#endif
    quiesce();
}

#if !defined(NVMCTRL_FLBUSY_bm)
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

#else // the AVR EA

// ---- d Flash: declined on this part ------------------------------------------------
void td_flash() {
    print(serial, "d Flash: SKIPPED - flash self-programming is declined on the "
                  "AVR EA (the part keeps its small values in its EEPROM; "
                  "docs/avrxt/nvm.md)", crlf);
}

// ---- e what an EEPROM operation costs the rest of the system -----------------------
void te_stall() {
    print(serial, "e what an EEPROM operation costs the CPU and its interrupts", crlf);
    quiesce();
    const uint32_t cost = stamp_cost();
    (void)erase_page(page_e);

    // The CPU runs while the EEPROM is programmed, as long as it touches
    // no EEPROM: a loop that reads STATUS alone keeps turning.
    uint32_t turns = 0;
    uint32_t t0 = cycles_now();
    (void)Nvm::eeprom_erase_write(page_e, 0x33u);
    while (Nvm::eeprom_busy()) {
        ++turns;
    }
    const uint32_t op = cycles_now() - t0;
    print(serial, "  an erase-and-write of ", us_of(op), " us: a loop polling STATUS "
                  "turned ", turns, " times", crlf);
    // The loop (STATUS load, test, a 32-bit increment, the branch) is
    // about ten cycles: some 10 000 turns in a 107 000-cycle operation; a
    // halted CPU would turn at most once. 1000 sits a decade from both.
    verdict("the CPU keeps running through an EEPROM operation", turns > 1000u);

    // A READ of the EEPROM during an operation is a bus wait (11.3.2.1):
    // the instruction is suspended until the operation ends.
    t0 = cycles_now();
    (void)Nvm::eeprom_erase_write(page_e, 0x44u);
    const uint32_t issued = cycles_now();
    const uint8_t seen = Nvm::eeprom_read(static_cast<uint16_t>(page_e + 1));
    const uint32_t read_done = cycles_now();
    const bool idle_after_read = !Nvm::eeprom_busy();
    const uint32_t read_stall = read_done - issued - cost;
    const uint32_t rest = op - (issued - t0);
    print(serial, "  a read 1 us into it took ", us_of(read_stall), " us (", read_stall,
          " cycles; the operation had ", rest, " to go), and read ", hex(seen), crlf);
    // And right after EEBUSY drops: is the array readable at once?
    uint32_t after_worst = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        (void)Nvm::eeprom_erase_write(page_e, static_cast<uint8_t>(0x60u + i));
        while (Nvm::eeprom_busy()) {
        }
        const uint32_t c0 = cycles_now();
        (void)Nvm::eeprom_read(static_cast<uint16_t>(page_e + 1));
        const uint32_t took = cycles_now() - c0 - cost;
        if (took > after_worst) after_worst = took;
    }
    print(serial, "  a read right after EEBUSY drops took ", after_worst,
          " cycles at worst (4 tries)", crlf);
    // The read is held to the operation's end at least: its stall is no
    // shorter than the remainder of an operation as long as the one
    // above, to letter b's same_time() percent.
    verdict("a read meanwhile waits out the rest of the operation",
            read_stall + rest / 100u >= rest && idle_after_read);

    // A buffer LOAD during an operation halts the CPU likewise
    // (11.3.2.2's note). The load is a stray afterwards: dropped.
    t0 = cycles_now();
    (void)Nvm::eeprom_erase_write(page_e, 0x55u);
    const uint32_t issued2 = cycles_now();
    Nvm::eeprom_load(static_cast<uint16_t>(page_e + 2), 0xFFu);
    const uint32_t load_done = cycles_now();
    const bool idle_after_load = !Nvm::eeprom_busy();
    const uint32_t load_stall = load_done - issued2 - cost;
    const uint32_t rest2 = op - (issued2 - t0);
    (void)Nvm::eeprom_open();
    print(serial, "  a buffer load 1 us into it took ", us_of(load_stall), " us (",
          load_stall, " cycles; the operation had ", rest2, " to go)", crlf);
    verdict("a buffer load meanwhile waits it out too",
            load_stall + rest2 / 100u >= rest2 && idle_after_load);

    // And an interrupt waits with the instruction. The alarm falls due
    // one millisecond into an erase-and-write; the ISR stamps the cascade
    // itself, so what is measured is how late it ran.
    Ticker::pause();
    alarm_mode = 1;
    constexpr uint16_t one_ms = static_cast<uint16_t>(cycles_per_us * 1000u);
    (void)Alarm::init({.mode = TcbMode::periodic,
                       .clock = TcbClock::div1,
                       .compare = one_ms});
    Alarm::enable_capt_interrupt(true);
    uint32_t quiet_worst = 0, polling_worst = 0, reading_worst = 0;
    struct Probe { uint32_t late; uint32_t spins; uint32_t gap; bool busy; };
    Probe probes[3][3] = {};
    for (uint8_t i = 0; i < 3; ++i) {
        for (uint8_t leg = 0; leg < 3; ++leg) {
            cli();
            alarm_ran = false;
            Alarm::count(0);
            Alarm::clear_capt();
            const uint32_t due = cycles_now() + one_ms;
            sei();
            uint32_t last = 0, gap = 0, spins = 0;
            if (leg != 0) {
                (void)Nvm::eeprom_erase_write(page_e, static_cast<uint8_t>(leg + i));
                if (leg == 2) {
                    (void)Nvm::eeprom_read(static_cast<uint16_t>(page_e + 1));
                }
            }
            last = cycles_now();
            while (!alarm_ran) {
                const uint32_t t = cycles_now();
                if (t - last > gap) gap = t - last;
                last = t;
                ++spins;
            }
            // Taken now: the alarm is periodic and stamps again every
            // millisecond while the operation below is waited out.
            const uint32_t late = alarm_stamp - due;
            (void)Nvm::wait_eeprom();
            probes[i][leg] = Probe{late, spins, gap, alarm_saw_busy};
            uint32_t& worst = leg == 0 ? quiet_worst : (leg == 1 ? polling_worst : reading_worst);
            if (late > worst) worst = late;
        }
    }
    alarm_mode = 0;
    Alarm::enable_capt_interrupt(false);
    Alarm::disable();
    Ticker::resume();
    bool ran_during = true;   // every leg-1 stamp found the operation running
    uint32_t loop_gap = 0;    // the longest stretch between two waiting turns
    for (uint8_t i = 0; i < 3; ++i) {
        if (!probes[i][1].busy) ran_during = false;
        if (probes[i][1].gap > loop_gap) loop_gap = probes[i][1].gap;
    }
    print(serial, "  while the operation ran, the waiting loop's longest stretch "
                  "between two turns was ", loop_gap, " cycles", crlf);
    print(serial, "  interrupt latency, CLK_PER cycles: ", quiet_worst, " quiet, ",
          polling_worst, " with an operation running and the CPU polling STATUS, ",
          reading_worst, " (", us_of(reading_worst), " us) with the CPU reading "
          "the EEPROM", crlf);
    // Quiet: the response (5 cycles), the prologue's pushes before the
    // stamp and the instruction in flight - the DA/DB letter's own bound.
    verdict("an interrupt is answered in a handful of cycles when nothing "
            "is happening", quiet_worst < 100u);
    verdict("and just as fast while an EEPROM operation runs, the CPU free",
            polling_worst < 100u && ran_during);
    // Reading: the alarm (1 ms in) waits for the operation's end, some
    // 4.4 ms on: more than half the operation is a floor with room.
    verdict("but a read of the EEPROM holds it for the rest of the operation",
            reading_worst > op / 2u);

    // ---- a record store, on each path, under a 1 ms alarm ----
    // The record straddles two pages. On the store's run path it waits
    // for the EEPROM before every read, so nothing it does holds an
    // interrupt; on the byte path it reads the next byte while the last
    // one is being written, and that read holds the CPU and the alarm
    // to the operation's end (above). The ISR keeps the longest gap
    // between two of its runs; the main context reads no stopwatch
    // meanwhile (the cascade's latch is shared with the ISR).
    const auto worst_gap = [](bool run_path, uint8_t seed) -> uint32_t {
        Wide8 v{};
        for (uint8_t i = 0; i < sizeof v.b; ++i) {
            v.b[i] = static_cast<uint8_t>(seed + 17u * i);
        }
        (void)Nvm::eeprom_erase(104, EepromErase::bytes8);
        (void)Nvm::wait_eeprom();
        (void)Nvm::eeprom_erase(112, EepromErase::bytes8);
        (void)Nvm::wait_eeprom();
        cli();
        alarm_runs = 0;
        alarm_gap = 0;
        alarm_mode = 2;
        Alarm::count(0);
        Alarm::clear_capt();
        sei();
        while (alarm_runs < 2) {
        }
        const uint16_t written = run_path ? RunRecordE::store(v) : ByteRecordE::store(v);
        const uint8_t after = alarm_runs;
        while (static_cast<uint8_t>(alarm_runs - after) < 2u && alarm_runs != 0xFFu) {
        }
        cli();
        alarm_mode = 0;
        const uint32_t gap = alarm_gap;
        sei();
        const std::optional<Wide8> back = RunRecordE::load();
        const bool ok = written == nv_record_size<Wide8> && back &&
                        back->b[7] == static_cast<uint8_t>(seed + 17u * 7u);
        return ok ? gap : 0u;
    };
    Ticker::pause();
    (void)Alarm::init({.mode = TcbMode::periodic,
                       .clock = TcbClock::div1,
                       .compare = one_ms});
    Alarm::enable_capt_interrupt(true);
    console_drain();
    const uint32_t gap_run = worst_gap(true, 0x21u);
    const uint32_t gap_byte = worst_gap(false, 0x43u);
    Alarm::enable_capt_interrupt(false);
    Alarm::disable();
    Ticker::resume();
    (void)Nvm::eeprom_erase(104, EepromErase::bytes8);
    (void)Nvm::wait_eeprom();
    (void)Nvm::eeprom_erase(112, EepromErase::bytes8);
    (void)Nvm::wait_eeprom();
    print(serial, "  a 12-byte record over two pages under a 1 ms alarm: the "
                  "longest gap between two alarm runs ", gap_run, " cycles on the run "
                  "path, ", gap_byte, " (", us_of(gap_byte), " us) on the byte path "
                  "(the period: ", one_ms, ")", crlf);
    // A gap is the period plus this run's lateness minus the last's;
    // a lateness is under 100 cycles with the CPU free (the verdicts
    // above), so the gap stays under period + 100 when nothing holds
    // the alarm; 200 is the bound. A zero gap is a record that did not
    // store or load back.
    verdict("the run path's record store holds no interrupt",
            gap_run >= one_ms && gap_run < one_ms + 200u);
    // On the byte path each read of the next byte holds the alarm for
    // the rest of an EEPW onto an erased byte - 52 000 cycles less the
    // few hundred the store took to start it - so the longest gap is at
    // least that. `op` is the erase-and-write above, two EEPWs: the
    // period plus a quarter of it is 46 000, under the floor.
    verdict("the byte path's holds one for most of an operation",
            gap_byte > one_ms + op / 4u);

    (void)erase_page(page_e);
    Nvm::clear_command();
    quiesce();
}

#endif // the EA's letters d and e

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
#if !defined(NVMCTRL_FLBUSY_bm)
    verdict("no protection bit is set after a reset",
            !Nvm::appcode_protected() && !Nvm::appdata_protected() &&
                !Nvm::boot_read_protected());
#else
    verdict("EEWP is clear after a reset", !Nvm::eeprom_protected());
#endif
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
#if !defined(NVMCTRL_FLBUSY_bm)
    print(serial, crlf, "f (continued after reset ", phase, " of 3)", crlf);
#else
    print(serial, crlf, "f (continued after reset ", phase, " of 2)", crlf);
#endif
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

#if !defined(NVMCTRL_FLBUSY_bm)
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
#else
        // Phase 2: EEWP. One-way until a reset; the driver refuses before
        // the silicon sees anything, and the silicon refuses too.
        const uint16_t at = page_f;
        verdict("an EEPROM write works before the protection is set",
                Nvm::eeprom_erase_write(at, 0x55u) && Nvm::wait_eeprom() &&
                    Nvm::eeprom_read(at) == 0x55u);
        Nvm::protect_eeprom();
        verdict("EEWP reads back", Nvm::eeprom_protected());
        const uint8_t other = 0x12u;
        verdict("the driver now refuses every EEPROM write and erase",
                !Nvm::eeprom_erase_write(at, other) && !Nvm::eeprom_erase(at) &&
                    !Nvm::eeprom_store_run(at, &other, 1) && !Nvm::eeprom_open());
        // The raw steps, past the driver's check.
        Nvm::clear_error();
        (void)Nvm::wait_idle();
        Nvm::issue(NvmCommand::eeprom_buffer_clear);
        Nvm::eeprom_load(at, 0x00u);
        Nvm::eeprom_commit(EepromOp::erase_write);
        (void)Nvm::wait_eeprom();
        const NvmError e = Nvm::error();
        print(serial, "  an erase-and-write past the driver's guard -> ERROR ",
              error_name(e), ", byte ", hex(Nvm::eeprom_read(at)), crlf);
        verdict("the silicon refuses it too, with write_protect",
                e == NvmError::write_protect);
        Nvm::clear_error();
        Nvm::issue(NvmCommand::eeprom_buffer_clear);
        Nvm::eeprom_load(at, 0xFFu);
        Nvm::eeprom_commit(EepromOp::erase);
        (void)Nvm::wait_eeprom();
        const NvmError e2 = Nvm::error();
        verdict("an erase included, and the byte is intact",
                e2 == NvmError::write_protect && Nvm::eeprom_read(at) == 0x55u);
        Nvm::clear_error();
        Nvm::clear_command();
#endif
        bank(2);
        print(serial, "  resetting to prove the protection is cleared by a "
                      "reset ...", crlf);
        console_drain();
        Reset::software();
    }

#if defined(NVMCTRL_FLBUSY_bm)
    if (phase == 2) {
        verdict("a reset cleared EEWP", !Nvm::eeprom_protected());
        verdict("and the EEPROM is writable again",
                Nvm::eeprom_erase(page_f) && Nvm::wait_eeprom() &&
                    Nvm::eeprom_read(page_f) == 0xFFu);
        Nvm::clear_command();
        verdict("RSTFR names a SOFTWARE reset", boot_reset.software);
        verdict("no breadcrumb is left anywhere",
                !boot_record.has_value() && !boot_nv_record.has_value());
        verdict("the .noinit token crossed both resets intact",
                token.canary == token_canary);
        verdict("IVSEL is still armed after every one of them",
                Nvm::vectors_in_boot_armed());
    }
#else
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
#endif
    token.magic = 0;
    token.phase = 0;
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
    if (in_all_run) {
        print(serial, "ALL: ", static_cast<uint16_t>(token.all_pass + passed),
              " pass, ", static_cast<uint16_t>(token.all_fail + failed), " fail",
              crlf);
    }
}

#if !defined(NVMCTRL_FLBUSY_bm)
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


// ---- m the data-space window's protection check (DA: DS80000882C 2.7.1) -----------
// The erratum: the inter-section protection ignores FLMAP and compares only
// the offset into the 32 KB window, so BOOT is mirrored into every section
// the window maps - writes through the window work only with FLMAP = 0.
// On this geometry (BOOTSIZE = 128: BOOT is the first 64 KB) every offset
// of the window lands in BOOT, so a page erase through the window into
// the scratch region - APPCODE, which BOOT may write and which letter d
// erases through SPM - is predicted REFUSED with WRITEPROTECT, the page
// left as it was; a part without the erratum would erase it. The read half
// is the control: the same window READS the section it maps.
//
// Not in z, and run before letter a after a reset: a moves FLMAP and then
// locks it, and only a reset clears FLMAPLOCK. The window is moved with
// interrupts off and nothing between the two FLMAP writes may read
// .rodata, which the window normally shows (FLMAP 3). Wear: one erase and
// one word write to mark the page, one erase to leave it blank - two erase
// cycles of one scratch page per run.
void tm_window() {
    print(serial, "m the data-space window's protection check (DS80000882C 2.7.1): a page "
                  "erase through the window, into APPCODE", crlf);
    quiesce();
    if (!scratch_usable()) {
        print(serial, "  no writable scratch region under these fuses: SKIPPED", crlf);
        return;
    }
    if (Nvm::flmap_locked()) {
        verdict("FLMAPLOCK is clear (letter a sets it: run m first after a reset)", false);
        return;
    }
    const uint32_t page = scratch_page(scratch_page_window);
    const uint8_t boot_map = Nvm::flmap();
    const uint8_t map = static_cast<uint8_t>(page >> 15);       // its 32 KB section
    const uint16_t offset = static_cast<uint16_t>(page & 0x7FFFu);
    print(serial, "  the page at ", page, " (", section_name(Nvm::section_of(page)),
          "): window section ", map, ", offset ", offset, "; .rodata's section ", boot_map,
          crlf);
    verdict("the page is APPCODE, which BOOT may write",
            Nvm::section_of(page) == FlashSection::appcode &&
                Nvm::writable(page, page + flash_page_size));
    verdict("a marker 0xA55A is written through SPM",
            Nvm::erase(page) && Nvm::write_word(page, 0xA55Au) &&
                Nvm::flash_read_word(page) == 0xA55Au);

    volatile uint8_t* const window =
        reinterpret_cast<volatile uint8_t*>(MAPPED_PROGMEM_START + offset);
    cli();
    Nvm::clear_error();
    const bool moved = Nvm::set_flmap(map);
    const uint8_t lo = window[0];
    const uint8_t hi = window[1];
    Nvm::select(NvmCommand::flash_page_erase);
    window[0] = 0xFF;                        // the store is the erase request
    const bool idle = Nvm::wait_idle();
    Nvm::clear_command();
    const NvmError err = Nvm::error();
    const bool restored = Nvm::set_flmap(boot_map);
    sei();
    const uint16_t after = Nvm::flash_read_word(page);

    print(serial, "  through the window: read ", hex(hi), hex(lo), "; the erase left ERROR ",
          static_cast<uint8_t>(err), " (2 = WRITEPROTECT) and the page reading ", hex(after),
          crlf);
    verdict("FLMAP moved to the page's section and back", moved && restored &&
                                                           Nvm::flmap() == boot_map);
    verdict("the window READS the section it maps: the marker comes through it",
            lo == 0x5Au && hi == 0xA5u);
    verdict("the controller went idle again", idle);
    verdict("2.7.1: a page erase through the window into APPCODE is refused "
            "(WRITEPROTECT: the offset alone was checked, and it is BOOT's)",
            err == NvmError::write_protect);
    verdict("and the page still holds the marker", after == 0xA55Au);
    Nvm::clear_error();
    verdict("the page is left blank again through SPM", Nvm::erase(page) &&
                                                        Nvm::flash_read_word(page) == 0xFFFFu);
}

#else // the AVR EA

// ---- u the User Row: declined on this part -----------------------------------------
void tu_userrow() {
    print(serial, "u the User Row write path: SKIPPED - the AVR EA's CPU never "
                  "writes its User Row (its write path is the declined flash "
                  "one); UPDI writes it, even on a locked part "
                  "(docs/avrxt/nvm.md)", crlf);
}

#endif // the DA/DB's letters u, g, m and the EA's u

// ---- the menu -----------------------------------------------------------------------

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
#if !defined(NVMCTRL_FLBUSY_bm)
constexpr Test tests[] = {
    {'a', ta_identity}, {'b', tb_eeprom}, {'c', tc_services}, {'d', td_flash},
    {'e', te_stall},    {'f', tf_protections}, {'u', tu_userrow},
    {'g', tg_appdata},
};
#else
constexpr Test tests[] = {
    {'a', ta_identity}, {'b', tb_eeprom}, {'c', tc_services}, {'d', td_flash},
    {'e', te_stall},    {'f', tf_protections}, {'u', tu_userrow},
};
#endif
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

#if !defined(NVMCTRL_FLBUSY_bm)
void help() {
    print(serial, "test_avr_nvm: a identity and geometry | b EEPROM primitives "
                  "and times | c record, writer AO, endurance | d Flash in the "
                  "scratch region | e the CPU stall | f protections and the "
                  "persistent panic across REAL resets    -> z = all of a..f",
          crlf,
          "  outside z (each costs more than a test should cost per run): "
          "u the User Row write path | g APPDATA and the multi-page-erase "
          "erratum (needs a temporary CODESIZE fuse)", crlf);
    if constexpr (window_erratum) {
        print(serial, "  and, first after a reset: m the data-space window's protection "
                      "check (DS80000882C 2.7.1)", crlf);
    }
}
#else
void help() {
    print(serial, "test_avr_nvm (AVR EA: the EEPROM engine): a identity, the "
                  "supply guard, FLMAP | b the page buffer, the three operations, "
                  "the times | c record, writer AO, endurance, the run path beside "
                  "the byte path | d Flash (declined) | e what an EEPROM operation "
                  "costs the CPU | "
                  "f the persistent panic and EEWP across REAL resets    -> z = "
                  "all of a..f", crlf,
          "  outside z: u the User Row write path (declined)", crlf);
}
#endif

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }

/// The EEPROM's ready interrupt. Its flag is a LEVEL: disabling the
/// interrupt here is not an optimization, it is the only thing that
/// stops the vector re-entering forever (11.5.4).
#if !defined(NVMCTRL_FLBUSY_bm)
ISR(NVMCTRL_EE_vect) {
#else
ISR(NVMCTRL_EEREADY_vect) {
#endif
    brio::Nvm::eeready();
    nvm_interrupts = nvm_interrupts + 1;
#if defined(NVMCTRL_FLBUSY_bm)
    if (ready_target != 0) {
        brio::post<ByteWriter>(brio::NvReady{});
        return;
    }
#endif
    brio::post<Writer>(brio::NvReady{});
}

ISR(TCB0_INT_vect) {
    (void)Alarm::take_flags();
    if (alarm_mode == 1) {
        alarm_stamp = cycles_now();
#if defined(NVMCTRL_FLBUSY_bm)
        alarm_saw_busy = Nvm::eeprom_busy();
#endif
        alarm_ran = true;
    }
#if defined(NVMCTRL_FLBUSY_bm)
    if (alarm_mode == 2) {
        const uint32_t now = cycles_now();
        if (alarm_runs != 0 && now - alarm_last > alarm_gap) {
            alarm_gap = now - alarm_last;
        }
        alarm_last = now;
        if (alarm_runs != 0xFFu) alarm_runs = static_cast<uint8_t>(alarm_runs + 1u);
    }
#endif
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
#if !defined(NVMCTRL_FLBUSY_bm)
        print(serial, crlf, "test_avr_nvm - NVMCTRL test suite (board ", board,
              ", clk=", xtal ? (has_xoschf ? "XTAL" : "EXTCLK") : "OSCHF", " 24 MHz, silicon rev ",
              hex(SYSCFG.REVID), ", BOOTSIZE ", FUSE.BOOTSIZE, "/CODESIZE ",
              FUSE.CODESIZE, ")", crlf);
#else
        print(serial, crlf, "test_avr_nvm - NVMCTRL test suite, the EEPROM engine (board ",
              board, ", clk=", xtal ? "XTAL" : "OSCHF", " ", SysClock::hz / 1'000'000u,
              " MHz, silicon rev ", hex(SYSCFG.REVID), ", BODCFG ", hex(FUSE.BODCFG),
              ", supply ", Nvm::supply_guarded() ? "guarded" : "NOT GUARDED", ")", crlf);
#endif
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
#if !defined(NVMCTRL_FLBUSY_bm)
            if constexpr (window_erratum) {
                if (c == 'm') { run(tm_window); found = true; }
            }
#endif
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
