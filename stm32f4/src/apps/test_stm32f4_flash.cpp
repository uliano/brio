// test_stm32f4_flash - the reference bench suite for the STM32F4's
// EMBEDDED FLASH MEMORY INTERFACE: the sector map computed from the
// size register, the keyed lock, the program and erase engine with its
// four error flags, the interrupt, the option bytes, and the two
// numbers this family's flash is really about - how long the core is
// FROZEN while a sector erases, and what a second program into an
// already programmed word does.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The console is the board's own bridge and everything
// measured is inside the chip. The RULER IS TIM2, a free-running 32-bit
// microsecond counter: the kernel tick cannot time an erase, because
// SysTick's interrupt is not TAKEN while the core is stalled on the
// flash - which is itself one of the things the suite measures.
//
// WHAT IT COSTS. The flash of this family is good for 10 000 erase
// cycles a sector (DS10314 table 47). Letters e, f, g and h ERASE, and
// are therefore OUT of the z set and asked for by name: one full round
// of them spends four cycles of the test sector, one of each smaller
// free sector, and two writes of the configuration sector. THE TEST
// ZONE IS THE LAST SECTOR OF THE ARRAY and every letter checks first
// that it lies above the image's own last byte (the linker's .data load
// address plus its length); on a part where it does not, the letter
// says so and does nothing.
//
// Letter k is the one that cannot be undone by another letter: a wrong
// unlock key locks the flash interface until the next RESET. It is last
// in the menu, out of z, and after it the board wants a re-flash.
//
// What is exercised, letter by letter:
//   a  the array: the size register, the banks, the sector map walked
//      and checked against the total, the image's own sectors, the OTP
//      and the option bytes decoded
//   b  the lock: LOCK out of reset, the keyed unlock, its idempotence,
//      and an erase refused while it stands
//   c  the four malformed sequences, each aborted by the silicon with
//      its own flag and nothing written
//   d  the error interrupt: OPERR exists only while ERRIE is set, and
//      the vector with the ISR body behind it
//   e  erase and program the last sector: the blank check, a block
//      written and read back, the CRC over it, the timings   (WEAR)
//   f  a second program into a programmed word: the AND, and the one
//      that needs an erase                                   (WEAR)
//   g  the erase stall: one sector of each size, the turns the core
//      completed, the ART on and off, the tick's loss         (WEAR)
//   h  write protection through the option bytes: a sector protected,
//      the WRPERR it earns, and the way back                  (WEAR)
//   k  a wrong unlock key: the engine locked until reset      (LAST)
//
// build: boards = f429zi,f446re,f411ce
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <span>

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/flash.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/reset.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/tim.hpp"
#include "stm32f4/usart.hpp"
#include "util/crc.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#if defined(STM32F411xE)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
#elif defined(STM32F429xx)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
#else
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
#endif
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

#if defined(STM32F429xx)
using Led = Pin<'G', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#elif defined(STM32F411xE)
using Led = Pin<'C', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#else
using Led = Pin<'A', 5>;
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
#endif

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

/// TIM2, free-running at 1 MHz over 32 bits: the only ruler in the chip
/// that keeps counting while the core is frozen on the flash.
using Ruler = Tim<2>;
uint32_t us_now() { return Ruler::count(); }
uint32_t us_since(uint32_t t0) { return us_now() - t0; }

// The image's own last byte in the flash: .data's load address plus its
// length, which is where the linker stopped writing. Everything above it
// is fair game for an erase; everything below is the running program.
extern "C" {
extern uint32_t __data_load_start;
extern uint32_t __data_start;
extern uint32_t __data_end;
}
uint32_t image_end() {
    return reinterpret_cast<uint32_t>(&__data_load_start) +
           (reinterpret_cast<uint32_t>(&__data_end) - reinterpret_cast<uint32_t>(&__data_start));
}

/// The test zone: the LAST sector of the array, and only if the image
/// ends below it.
std::optional<FlashSector> test_sector() {
    const uint8_t n = Flash::sector_count();
    if (n == 0u) {
        return std::nullopt;
    }
    const std::optional<FlashSector> s = Flash::sector_at(static_cast<uint8_t>(n - 1u));
    if (!s || s->base < image_end()) {
        return std::nullopt;
    }
    return s;
}

/// Say why a wear letter cannot run, once, in the same words.
bool have_test_sector(const std::optional<FlashSector>& s) {
    if (s) {
        return true;
    }
    print(serial, "  no free sector to work in: the map is unknown on this part class, or "
                  "the image reaches the last sector", crlf);
    bench.verdict("a test sector above the image", false);
    return false;
}

/// The bytes of a word, little-endian, for the program verb's span.
struct Word {
    uint8_t byte[4];
    explicit Word(uint32_t v)
        : byte{static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8),
               static_cast<uint8_t>(v >> 16), static_cast<uint8_t>(v >> 24)} {}
    std::span<const uint8_t> span() const { return std::span<const uint8_t>{byte, 4}; }
};

uint32_t read_word(uint32_t addr) { return *reinterpret_cast<const volatile uint32_t*>(addr); }

/// The flags of a status mask, spelled.
void print_flags(uint32_t mask) {
    if (mask == 0u) {
        print(serial, "none");
        return;
    }
    if (mask & FlashFlag::refused) print(serial, "refused ");
    if (mask & FlashFlag::operation_error) print(serial, "OPERR ");
    if (mask & FlashFlag::write_protect_error) print(serial, "WRPERR ");
    if (mask & FlashFlag::alignment_error) print(serial, "PGAERR ");
    if (mask & FlashFlag::parallelism_error) print(serial, "PGPERR ");
    if (mask & FlashFlag::sequence_error) print(serial, "PGSERR ");
    if ((mask & FlashFlag::read_protect_error) != 0u) print(serial, "RDERR ");
    if (mask & FlashFlag::eop) print(serial, "EOP ");
}

// The FLASH vector's own counters, written by the ISR body's caller.
volatile uint32_t flash_irqs = 0;
volatile uint32_t flash_irq_flags = 0;

// The fault counters letter k arms.
volatile uint32_t bus_faults = 0;
volatile uint32_t hard_faults = 0;
volatile uint32_t fault_cfsr = 0;

// =============================================================================
// a - the array, its sectors and its option bytes
// =============================================================================
void ta_array() {
    const DeviceIdcode id = DeviceIdcode::read();
    print(serial, "  DEV_ID ", hex(id.dev_id), " flash ", flash_size_kbytes(), " KB, ",
          Flash::bank_count(), " bank(s) of ", Flash::bank_bytes() / 1024u, " KB, ",
          Flash::sector_count(), " sectors, image ends at ", hex(image_end()), crlf);
    bench.verdict("the size register and the array agree",
                  Flash::size_bytes() == static_cast<uint32_t>(flash_size_kbytes()) * 1024u);
    bench.verdict("this part class's chapter 3 was read, so there is a sector map",
                  Flash::geometry_known && Flash::sector_count() > 0u);

    // The map, walked: every sector printed, contiguous, and adding up.
    uint32_t at = Flash::base;
    bool contiguous = true;
    bool numbering = true;
    uint32_t total = 0;
    for (uint8_t i = 0; i < Flash::sector_count(); ++i) {
        const std::optional<FlashSector> s = Flash::sector_at(i);
        if (!s) {
            contiguous = false;
            break;
        }
        print(serial, "  sector ", s->number, " bank ", s->bank, "  ", hex(s->base), "..",
              hex(s->base + s->size - 1u), "  ", s->size / 1024u, " KB",
              s->base >= image_end() ? "  free" : "  image", crlf);
        if (s->base != at) {
            contiguous = false;
        }
        if (Flash::sector(s->number) != s) {
            numbering = false;
        }
        at += s->size;
        total += s->size;
    }
    bench.verdict("the sectors tile the array with no hole and no overlap", contiguous);
    bench.verdict("and they add up to the whole of it", total == Flash::size_bytes());
    bench.verdict("a sector looked up by the manual's number is the same sector", numbering);

    // The shape every chapter states: four of 16 KB, one of 64, the rest
    // of 128.
    const std::optional<FlashSector> s0 = Flash::sector_at(0);
    const std::optional<FlashSector> s3 = Flash::sector_at(3);
    const std::optional<FlashSector> s4 = Flash::sector_at(4);
    const std::optional<FlashSector> s5 = Flash::sector_at(5);
    bench.verdict("the first four sectors are 16 KB",
                  s0 && s3 && s0->size == 16u * 1024u && s3->size == 16u * 1024u);
    bench.verdict("the fifth is 64 KB", s4 && s4->size == 64u * 1024u);
    bench.verdict("and the rest are 128 KB", s5 && s5->size == 128u * 1024u);

    // sector_of() on the two ends of a sector and on the vector table.
    bench.verdict("the vector table is in sector 0",
                  Flash::sector_of(Flash::base) && Flash::sector_of(Flash::base)->number == 0u);
    const std::optional<FlashSector> last = Flash::sector_at(
        static_cast<uint8_t>(Flash::sector_count() - 1u));
    bench.verdict("the last byte of the array belongs to the last sector",
                  last && Flash::sector_of(Flash::base + Flash::size_bytes() - 1u) == last);
    bench.verdict("an address past the array belongs to no sector",
                  !Flash::sector_of(Flash::base + Flash::size_bytes()));
    bench.verdict("and in_main_flash() draws the same line",
                  Flash::in_main_flash(Flash::base + Flash::size_bytes() - 1u) &&
                      !Flash::in_main_flash(Flash::base + Flash::size_bytes()));

    const std::optional<FlashSector> zone = test_sector();
    if (zone) {
        print(serial, "  the test zone is sector ", zone->number, " at ", hex(zone->base), ", ",
              zone->size / 1024u, " KB", crlf);
    }
    bench.verdict("the last sector is above the image, so the wear letters have a zone",
                  zone.has_value());

    // The option bytes, decoded.
    print(serial, "  OPTCR ", hex(FlashOptions::raw()), " OPTCR1 ", hex(FlashOptions::raw_bank2()),
          " RDP ", hex(FlashOptions::rdp_code()), " level ",
          static_cast<uint8_t>(FlashOptions::rdp()), " BOR_LEV ", FlashOptions::bor_level(),
          FlashOptions::bor_off() ? " (off)" : "", crlf);
    print(serial, "  WDG_SW ", FlashOptions::iwdg_software() ? "software" : "HARDWARE",
          ", reset on Stop ", FlashOptions::reset_on_stop() ? "yes" : "no", ", on Standby ",
          FlashOptions::reset_on_standby() ? "yes" : "no", ", PCROP ",
          FlashOptions::pcrop_mode() ? "ON" : "off", ", dual-bank boot ",
          FlashOptions::dual_bank_boot() ? "on" : "off", crlf);
    bench.verdict("the part is at RDP level 0, which is where a debugger can reach it",
                  FlashOptions::rdp() == FlashRdpLevel::level0);
    bench.verdict("the watchdog option is the software one (no IWDG running since boot)",
                  FlashOptions::iwdg_software());
    bench.verdict("PCROP is off, so the nWRP bits mean write protection",
                  !FlashOptions::pcrop_mode());

    bool all_unprotected = true;
    for (uint8_t i = 0; i < Flash::sector_count(); ++i) {
        const std::optional<FlashSector> s = Flash::sector_at(i);
        if (s && FlashOptions::write_protected(s->number)) {
            all_unprotected = false;
            print(serial, "  sector ", s->number, " is WRITE PROTECTED", crlf);
        }
    }
    bench.verdict("no sector is write protected", all_unprotected);

    // The OTP: read-only here, and printed so a board that has one used
    // shows it.
    uint8_t otp[16] = {};
    const bool otp_read = Flash::read_otp(0, otp);
    print(serial, "  OTP[0..7] ");
    for (uint8_t i = 0; i < 8u; ++i) {
        print(serial, hex(otp[i]), " ");
    }
    uint8_t locked_blocks = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        if (Flash::otp_block_locked(i)) {
            ++locked_blocks;
        }
    }
    print(serial, " locked blocks ", locked_blocks, "/16", crlf);
    bench.verdict("the OTP area reads", otp_read);
    bench.verdict("a read past the OTP area is refused",
                  !Flash::read_otp(500, std::span<uint8_t>{otp, 16}));

    // The read side, against the clock task's own constants.
    print(serial, "  LATENCY ", FlashWaitStates::get(), " WS, prefetch ",
          FlashAccel::prefetch() ? "on" : "off", ", I-cache ",
          FlashAccel::icache() ? "on" : "off", ", D-cache ",
          FlashAccel::dcache() ? "on" : "off", crlf);
    bench.verdict("the latency is the one this rate needs",
                  FlashWaitStates::get() == SysClock::wait_states);
    bench.verdict("the ART accelerator is fully on", FlashAccel::prefetch() &&
                                                         FlashAccel::icache() &&
                                                         FlashAccel::dcache());
    bench.verdict("a cache reset is refused while the cache is enabled",
                  !FlashAccel::icache_reset() && !FlashAccel::dcache_reset());
    FlashAccel::flush_caches();
    bench.verdict("and flush_caches() leaves both caches as it found them",
                  FlashAccel::icache() && FlashAccel::dcache());
}

// =============================================================================
// b - the lock
// =============================================================================
void tb_lock() {
    // Whatever ran before, start from a locked engine: LOCK is a
    // write-one bit, so this is always possible.
    (void)Flash::lock();
    bench.verdict("FLASH_CR can be locked, and reads locked", Flash::locked());

    // An erase while it stands is refused BEFORE the flash is touched -
    // FlashFlag::refused and no hardware flag at all.
    const std::optional<FlashSector> zone = test_sector();
    const uint32_t locked_try =
        zone ? Flash::erase_sector(zone->number) : Flash::erase_sector(0xFFu);
    print(serial, "  an erase with LOCK set -> ");
    print_flags(locked_try);
    print(serial, crlf);
    bench.verdict("an erase on a locked engine is refused by the driver, not by the silicon",
                  locked_try == FlashFlag::refused);
    bench.verdict("and no error flag stands afterwards", Flash::errors() == 0u);

    const uint32_t t0 = us_now();
    const bool opened = Flash::unlock();
    const uint32_t unlock_us = us_since(t0);
    print(serial, "  the key pair took ", unlock_us, " us", crlf);
    bench.verdict("KEY1 then KEY2 clears LOCK", opened && !Flash::locked());
    bench.verdict("unlock() on an open engine is a no-op, not a second key pair",
                  Flash::unlock() && !Flash::locked());

    bench.verdict("lock() closes it again", Flash::lock() && Flash::locked());
    bench.verdict("and it opens again", Flash::unlock() && !Flash::locked());
    (void)Flash::lock();
}

// =============================================================================
// c - the four malformed sequences
// =============================================================================
void tc_missteps() {
    const std::optional<FlashSector> zone = test_sector();
    if (!have_test_sector(zone)) {
        return;
    }
    if (!Flash::unlock()) {
        bench.verdict("the engine opens", false);
        return;
    }
    const uint32_t at = zone->base;
    const uint32_t before = read_word(at);

    struct Case {
        Flash::Misstep step;
        const char* name;
        uint32_t expect;
    };
    const Case cases[] = {
        {Flash::Misstep::store_without_pg, "a store into the array with PG clear",
         FlashFlag::sequence_error},
        {Flash::Misstep::wrong_access_width, "a byte store while PSIZE says x32",
         FlashFlag::parallelism_error},
        {Flash::Misstep::write_protected_area, "a store into the system memory",
         FlashFlag::write_protect_error},
    };
    for (const Case& c : cases) {
        const uint32_t got = Flash::provoke(c.step, at);
        print(serial, "  ", c.name, " -> ");
        print_flags(got);
        print(serial, " (SR ", hex(Flash::last_status()), ")", crlf);
        bench.verdict("the silicon names it: ", c.name, (got & c.expect) != 0u);
    }

    // The fourth is a MEASUREMENT and not a claim: the core splits an
    // unaligned word store, so which flag the engine ends up raising is
    // the silicon's answer, not the manual's.
    const uint32_t unaligned = Flash::provoke(Flash::Misstep::unaligned_word, at);
    print(serial, "  a word store two bytes off its alignment -> ");
    print_flags(unaligned);
    print(serial, " (SR ", hex(Flash::last_status()), ")", crlf);
    bench.verdict("an unaligned word store is refused by the silicon, whichever flag it picks",
                  (unaligned & FlashFlag::errors) != 0u);

    print(serial, "  the target word was ", hex(before), " and is ", hex(read_word(at)), crlf);
    bench.verdict("NOT ONE of the four wrote anything: the operation is aborted, "
                  "which is why this letter costs no endurance",
                  read_word(at) == before);
    bench.verdict("the flags are cleared behind each attempt", Flash::errors() == 0u);
    (void)Flash::lock();
}

// =============================================================================
// d - the error interrupt
// =============================================================================
void td_interrupt() {
    const std::optional<FlashSector> zone = test_sector();
    if (!have_test_sector(zone)) {
        return;
    }
    if (!Flash::unlock()) {
        bench.verdict("the engine opens", false);
        return;
    }
    Nvic::enable(Flash::irq());

    // With ERRIE clear, 3.8.4 says OPERR is not even set.
    (void)Flash::interrupts(false, false);
    flash_irqs = 0;
    (void)Flash::provoke(Flash::Misstep::store_without_pg, zone->base);
    const uint32_t silent_sr = Flash::last_status();
    print(serial, "  ERRIE clear: SR ", hex(silent_sr), ", ", flash_irqs, " interrupt(s)", crlf);
    bench.verdict("PGSERR stands on its own", (silent_sr & FlashFlag::sequence_error) != 0u);
    bench.verdict("OPERR does NOT: 3.8.4 makes the flag itself conditional on ERRIE",
                  (silent_sr & FlashFlag::operation_error) == 0u);
    bench.verdict("and no interrupt is taken", flash_irqs == 0u);

    // With ERRIE set and the LINE MASKED, the same misstep raises OPERR
    // where the operation's own tail can still read it. Masked on
    // purpose: the handler clears the flags, and it would win the race
    // against the driver's own read.
    Nvic::disable(Flash::irq());
    bench.verdict("the two enables are written and read back",
                  Flash::interrupts(true, true) && Flash::end_of_operation_interrupt() &&
                      Flash::error_interrupt());
    (void)Flash::provoke(Flash::Misstep::store_without_pg, zone->base);
    print(serial, "  ERRIE set:   SR ", hex(Flash::last_status()), crlf);
    bench.verdict("OPERR is raised beside the cause", (Flash::last_status() &
                                                       FlashFlag::operation_error) != 0u);

    // The masked error latched the NVIC's pending bit all the same -
    // masking a line does not stop it being raised, only taken - so it
    // is taken down before the line is opened, or it would fire the
    // handler once for an error that is already history.
    bench.verdict("an error taken while the line was masked leaves the NVIC pending",
                  Nvic::pending(Flash::irq()));
    Nvic::clear_pending(Flash::irq());

    // And with the line open, the same misstep reaches the vector.
    flash_irqs = 0;
    flash_irq_flags = 0;
    Nvic::enable(Flash::irq());
    (void)Flash::provoke(Flash::Misstep::store_without_pg, zone->base);
    (void)delay_us(clock, 50);
    print(serial, "  the vector: ", flash_irqs, " interrupt(s), the body saw ");
    print_flags(flash_irq_flags);
    print(serial, crlf);
    bench.verdict("the FLASH vector fires on the error", flash_irqs >= 1u);
    bench.verdict("ONCE, and not twice: the level goes down with the flags the body "
                  "cleared, before the handler returns",
                  flash_irqs == 1u);
    bench.verdict("the body names what raised it",
                  (flash_irq_flags & (FlashFlag::sequence_error |
                                      FlashFlag::operation_error)) != 0u);
    bench.verdict("and nothing is left standing afterwards", Flash::errors() == 0u);

    (void)Flash::interrupts(false, false);
    bench.verdict("both enables go down again",
                  !Flash::end_of_operation_interrupt() && !Flash::error_interrupt());
    Nvic::disable(Flash::irq());
    (void)Flash::lock();
}

// =============================================================================
// e - erase and program the last sector (WEAR)
// =============================================================================
void te_round_trip() {
    const std::optional<FlashSector> zone = test_sector();
    if (!have_test_sector(zone)) {
        return;
    }
    if (!Flash::unlock()) {
        bench.verdict("the engine opens", false);
        return;
    }
    // EOPIE on with the LINE MASKED: the flag exists only while its
    // enable stands (3.8.4), and letter d is where the vector is shown.
    Nvic::disable(Flash::irq());
    (void)Flash::interrupts(true, false);

    const uint32_t t0 = us_now();
    const uint32_t erased = Flash::erase_sector(zone->number);
    const uint32_t erase_us = us_since(t0);
    print(serial, "  erasing sector ", zone->number, " (", zone->size / 1024u, " KB) took ",
          erase_us, " us, ", Flash::last_wait_turns(), " poll turns, SR ",
          hex(Flash::last_status()), crlf);
    bench.verdict("the erase reports no error", erased == 0u);
    bench.verdict("EOP is set, ITS ENABLE BEING WHAT MAKES IT EXIST",
                  (Flash::last_status() & FlashFlag::eop) != 0u);
    (void)Flash::interrupts(false, false);

    // AN ERASE DOES NOT INVALIDATE THE CACHES (3.5.4: "you have to make
    // sure that these data are rewritten before they are accessed"), so
    // the blank check is made twice - once as the caches stand, once
    // after the flush - and only the second is a statement about the
    // ARRAY.
    const bool blank_cached = Flash::blank(zone->base, zone->size);
    FlashAccel::flush_caches();
    const bool blank_true = Flash::blank(zone->base, zone->size);
    print(serial, "  the sector reads erased: ", blank_true ? "yes" : "NO",
          " (before the cache flush: ", blank_cached ? "yes" : "no", ")", crlf);
    bench.verdict("the whole sector reads erased", blank_true);

    // A block of 1 KB, programmed word by word at x32.
    static uint8_t pattern[1024];
    for (uint16_t i = 0; i < sizeof(pattern); ++i) {
        pattern[i] = static_cast<uint8_t>(i * 7u + (i >> 5));
    }
    const uint32_t t1 = us_now();
    const uint32_t wrote = Flash::program(zone->base, pattern);
    const uint32_t program_us = us_since(t1);
    print(serial, "  1024 bytes at x32 took ", program_us, " us (",
          program_us * 1000u / (sizeof(pattern) / 4u), " ns a word, ",
          Flash::last_wait_turns(), " poll turns on the last one)", crlf);
    bench.verdict("the block programs with no error", wrote == 0u);

    // The read-back is a statement about the ARRAY only with the caches
    // out of the way - letter f is where that matters and this is where
    // the habit belongs.
    FlashAccel::flush_caches();
    bool exact = true;
    for (uint16_t i = 0; i < sizeof(pattern); ++i) {
        if (*reinterpret_cast<const volatile uint8_t*>(zone->base + i) != pattern[i]) {
            exact = false;
        }
    }
    const uint16_t crc_written =
        crc16(reinterpret_cast<const uint8_t*>(zone->base), sizeof(pattern));
    const uint16_t crc_source = crc16(pattern, sizeof(pattern));
    print(serial, "  CRC-16 of the block in flash ", hex(crc_written), ", of the source ",
          hex(crc_source), crlf);
    bench.verdict("every byte reads back exactly as it was written", exact);
    bench.verdict("and the CRC over the two agrees", crc_written == crc_source);

    // The far end of the sector, to prove the whole of it is reachable.
    const uint32_t far = zone->base + zone->size - 4u;
    const Word marker{0xC0FFEE00UL};
    bench.verdict("a word at the sector's last address programs",
                  Flash::program(far, marker.span()) == 0u && read_word(far) == 0xC0FFEE00UL);
    bench.verdict("and everything between the block and it is still erased",
                  Flash::blank(zone->base + sizeof(pattern), zone->size - sizeof(pattern) - 4u));

    // The bounds the driver draws before the flash is touched.
    bench.verdict("a program off the parallelism's alignment is refused",
                  Flash::program(zone->base + 1u, marker.span()) == FlashFlag::refused);
    bench.verdict("a program past the end of the array is refused",
                  Flash::program(Flash::base + Flash::size_bytes(), marker.span()) ==
                      FlashFlag::refused);
    bench.verdict("an erase of a sector this part has not got is refused",
                  Flash::erase_sector(31) == FlashFlag::refused);
    (void)Flash::lock();
}

// =============================================================================
// f - a second program into a programmed word (WEAR)
// =============================================================================
void tf_second_write() {
    const std::optional<FlashSector> zone = test_sector();
    if (!have_test_sector(zone)) {
        return;
    }
    if (!Flash::unlock() || Flash::erase_sector(zone->number) != 0u) {
        bench.verdict("the test sector erases", false);
        return;
    }

    // 3.5.4: "Successive write operations are possible without the need
    // of an erase operation when changing bits from 1 to 0. Writing 1
    // requires a flash memory erase operation." Four pairs, each in a
    // 128-bit row of its own so that no two share a cache line - and
    // each read TWICE, because the same paragraph says a write "modifies
    // the data in the flash memory AND the data in the cache", which
    // makes a read-back through the D-cache a report of what was
    // WRITTEN and not of what the array now holds.
    struct Pair {
        uint32_t first;
        uint32_t second;
        const char* name;
    };
    static const Pair pairs[] = {
        {0xA5A5A5A5UL, 0x5A5A5A5AUL, "0xA5A5A5A5 then 0x5A5A5A5A"},
        {0xFFFF0000UL, 0x0000FFFFUL, "0xFFFF0000 then 0x0000FFFF"},
        {0x12345678UL, 0xFFFFFFFFUL, "0x12345678 then 0xFFFFFFFF"},
        {0x0F0F0F0FUL, 0x0F0F0F0FUL, "0x0F0F0F0F twice        "},
    };
    bool all_and = true;
    bool all_quiet = true;
    bool cache_lied = false;
    for (uint8_t i = 0; i < sizeof(pairs) / sizeof(pairs[0]); ++i) {
        const uint32_t w = zone->base + i * 16u;
        const Word one{pairs[i].first};
        const Word two{pairs[i].second};
        const uint32_t first_mask = Flash::program(w, one.span());
        const uint32_t after_first = read_word(w);   // and the line is now resident
        const uint32_t second_mask = Flash::program(w, two.span());
        const uint32_t cached = read_word(w);
        FlashAccel::flush_caches();
        const uint32_t held = read_word(w);
        print(serial, "  ", pairs[i].name, " -> the cache says ", hex(cached),
              ", the array holds ", hex(held), ", flags ");
        print_flags(first_mask | second_mask);
        print(serial, crlf);
        if (after_first != pairs[i].first) {
            all_and = false;
        }
        if (held != (pairs[i].first & pairs[i].second)) {
            all_and = false;
        }
        if (first_mask != 0u || second_mask != 0u) {
            all_quiet = false;
        }
        if (cached != held) {
            cache_lied = true;
        }
    }

    bench.verdict("A SECOND PROGRAM INTO A PROGRAMMED WORD IS ACCEPTED AND ANDS INTO IT: "
                  "the bits that were 0 stay 0, and writing a 1 over a 0 does nothing",
                  all_and);
    bench.verdict("and it raises NOTHING - no PROGERR, no refusal, whatever was there "
                  "before: there is no write-once cell on this family",
                  all_quiet);
    bench.verdict("SO THE READ-BACK IS THE ONLY WITNESS - and it has to be taken with "
                  "the caches flushed, because a write updates the D-cache with the "
                  "value WRITTEN while the array takes the AND (3.5.4)",
                  cache_lied);

    // The same thing without the cache in the way: with DCEN clear the
    // read is the array's own.
    const uint32_t w4 = zone->base + 64u;
    FlashAccel::dcache(false);
    const Word one{0xCCCC0F0FUL};
    const Word two{0x0F0FCCCCUL};
    (void)Flash::program(w4, one.span());
    (void)Flash::program(w4, two.span());
    const uint32_t bare = read_word(w4);
    FlashAccel::dcache(true);
    print(serial, "  with the D-cache off: 0xCCCC0F0F then 0x0F0FCCCC -> ", hex(bare), crlf);
    bench.verdict("with the D-cache off the read-back needs no flush and is the AND",
                  bare == (0xCCCC0F0FUL & 0x0F0FCCCCUL));
    (void)Flash::lock();
}

// =============================================================================
// g - the erase stall (WEAR)
// =============================================================================
void tg_stall() {
    const std::optional<FlashSector> zone = test_sector();
    if (!have_test_sector(zone)) {
        return;
    }
    if (!Flash::unlock()) {
        bench.verdict("the engine opens", false);
        return;
    }

    // One free sector of each SIZE the map has, smallest first: the
    // duration is the datasheet's tERASE and the poll turns say whether
    // the core lived through it.
    uint32_t sizes_seen = 0;
    for (uint8_t i = 0; i < Flash::sector_count(); ++i) {
        const std::optional<FlashSector> s = Flash::sector_at(i);
        if (!s || s->base < image_end()) {
            continue;
        }
        if ((sizes_seen & s->size) != 0u) {
            continue;   // a size already measured
        }
        sizes_seen |= s->size;
        const uint32_t ticks0 = Ticker::ticks();
        const uint32_t t0 = us_now();
        const uint32_t err = Flash::erase_sector(s->number);
        const uint32_t took = us_since(t0);
        const uint32_t ticks = Ticker::ticks() - ticks0;
        print(serial, "  sector ", s->number, " (bank ", s->bank, "), ", s->size / 1024u,
              " KB: ", took, " us, ", Flash::last_wait_turns(),
              " poll turns, the kernel tick advanced ", ticks, " ms", crlf);
        bench.verdict("the erase completes with no error", err == 0u);
        if (s->bank == 1u) {
            // The image is in bank 1 on every board of this bench, so
            // this erase stalls the fetches that would have served the
            // SysTick handler.
            bench.verdict("THE CORE IS FROZEN THROUGH IT: the kernel tick, whose interrupt "
                          "cannot be taken, loses all but a millisecond or two of the wall",
                          ticks <= 2u);
        } else {
            // Read-while-write (RM0090 3.6.5): the code is in the other
            // bank and the core never notices.
            bench.verdict("the erase is in the OTHER bank, so the core runs through it and "
                          "the tick keeps its time",
                          ticks + 5u >= took / 1000u);
        }
    }

    // The ART's part in it. The poll loop is the only code running, so
    // with the instruction cache holding it the core may keep turning
    // while the array is busy; with the accelerator off it cannot fetch
    // at all. Both are printed; the erase time itself is the array's and
    // should not care.
    const uint32_t t1 = us_now();
    (void)Flash::erase_sector(zone->number);
    const uint32_t with_art = us_since(t1);
    const uint32_t turns_art = Flash::last_wait_turns();

    FlashAccel::prefetch(false);
    FlashAccel::icache(false);
    FlashAccel::dcache(false);
    (void)FlashAccel::icache_reset();
    (void)FlashAccel::dcache_reset();
    const uint32_t t2 = us_now();
    (void)Flash::erase_sector(zone->number);
    const uint32_t without_art = us_since(t2);
    const uint32_t turns_bare = Flash::last_wait_turns();
    FlashAccel::enable_all();

    print(serial, "  the same ", zone->size / 1024u, " KB sector: ART on ", with_art, " us / ",
          turns_art, " turns, ART off ", without_art, " us / ", turns_bare, " turns", crlf);
    if (zone->bank == 1u) {
        bench.verdict("with the accelerator off the core completes NO poll turn: "
                      "every instruction fetch is stalled on the array",
                      turns_bare == 0u);
    } else {
        bench.verdict("the zone is in the other bank, so the core polls throughout "
                      "whatever the accelerator does",
                      turns_bare > 0u);
    }
    bench.verdict("the erase takes the same time either way, the accelerator being "
                  "the CPU's side of the interface and not the array's",
                  with_art + with_art / 10u >= without_art &&
                      without_art + without_art / 10u >= with_art);
    bench.verdict("the ART is back on afterwards",
                  FlashAccel::prefetch() && FlashAccel::icache() && FlashAccel::dcache());
    (void)Flash::lock();
}

// =============================================================================
// h - write protection through the option bytes (WEAR)
// =============================================================================
void th_write_protection() {
    const std::optional<FlashSector> zone = test_sector();
    if (!have_test_sector(zone)) {
        return;
    }
    if (FlashOptions::rdp() != FlashRdpLevel::level0 || FlashOptions::pcrop_mode()) {
        print(serial, "  the part is not at RDP level 0 with PCROP off: an option write "
                      "would not be the reversible one figure 4 draws", crlf);
        bench.verdict("the state an option write is reversible in", false);
        return;
    }
    if (!Flash::unlock() || Flash::erase_sector(zone->number) != 0u) {
        bench.verdict("the test sector erases", false);
        return;
    }
    bench.verdict("the option engine is locked out of reset", FlashOptions::locked());
    bench.verdict("and OPTKEY1 then OPTKEY2 opens it", FlashOptions::unlock());
    bench.verdict("a second unlock is a no-op, not a second key pair",
                  FlashOptions::unlock() && !FlashOptions::locked());

    const uint8_t n = zone->number;
    const uint32_t optcr_before = FlashOptions::raw();
    const uint32_t t0 = us_now();
    const bool on = FlashOptions::protect(n, true);
    const uint32_t protect_us = us_since(t0);
    print(serial, "  protecting sector ", n, " took ", protect_us, " us, OPTCR ",
          hex(optcr_before), " -> ", hex(FlashOptions::raw()), crlf);
    bench.verdict("the option loader takes the new nWRP", on && FlashOptions::write_protected(n));
    bench.verdict("and NOTHING ELSE in OPTCR moved - the RDP byte was never in the data path",
                  (FlashOptions::raw() & ~0x0FFF0000UL) == (optcr_before & ~0x0FFF0000UL));

    const Word value{0x1234ABCDUL};
    const uint32_t denied = Flash::program(zone->base, value.span());
    print(serial, "  a program into it -> ");
    print_flags(denied);
    print(serial, ", the word reads ", hex(read_word(zone->base)), crlf);
    bench.verdict("a program into a protected sector earns WRPERR",
                  (denied & FlashFlag::write_protect_error) != 0u);
    bench.verdict("and writes nothing", read_word(zone->base) == 0xFFFFFFFFUL);

    const uint32_t denied_erase = Flash::erase_sector(n);
    print(serial, "  an erase of it -> ");
    print_flags(denied_erase);
    print(serial, crlf);
    bench.verdict("an erase of a protected sector earns WRPERR too",
                  (denied_erase & FlashFlag::write_protect_error) != 0u);

    bench.verdict("the protection comes off again", FlashOptions::protect(n, false) &&
                                                        !FlashOptions::write_protected(n));
    bench.verdict("and the sector takes a word again",
                  Flash::program(zone->base, value.span()) == 0u &&
                      read_word(zone->base) == 0x1234ABCDUL);
    bench.verdict("OPTCR is back where it started", FlashOptions::raw() == optcr_before);

    bench.verdict("the option engine locks again", FlashOptions::lock() && FlashOptions::locked());
    bench.verdict("and a protect through a locked engine is refused",
                  !FlashOptions::protect(n, true));
    (void)Flash::lock();
}

// =============================================================================
// k - a wrong unlock key (LAST: the engine is dead until a reset)
// =============================================================================
void tk_wrong_key() {
    print(serial, "  AFTER THIS LETTER the flash interface answers nothing until the board "
                  "is reset: it is the last thing to run", crlf);
    // The store is a bus error and this core reports it imprecisely, so
    // the vector is armed first - with BusFault enabled it arrives
    // there, and with it disabled it would escalate to HardFault.
    Faults::enable(false, true, false);
    (void)Faults::take();
    bus_faults = 0;
    hard_faults = 0;
    fault_cfsr = 0;

    (void)Flash::lock();
    Flash::provoke_wrong_key();
    (void)delay_us(clock, 100);

    print(serial, "  the wrong key raised ", bus_faults, " bus fault(s) and ", hard_faults,
          " hard fault(s), CFSR ", hex(fault_cfsr), crlf);
    bench.verdict("LOCK still stands", Flash::locked());

    const bool reopened = Flash::unlock();
    print(serial, "  a CORRECT key pair afterwards -> ", reopened ? "opened" : "refused",
          ", LOCK ", Flash::locked() ? "set" : "clear", ", ", bus_faults, " bus fault(s) so far",
          crlf);
    bench.verdict("THE REGISTER IS LOCKED UNTIL THE NEXT RESET: even the right sequence "
                  "does not open it again",
                  !reopened && Flash::locked());

    const std::optional<FlashSector> zone = test_sector();
    const uint32_t after = zone ? Flash::erase_sector(zone->number) : FlashFlag::refused;
    bench.verdict("so every erase and program refuses from here on",
                  after == FlashFlag::refused);
    Faults::enable(false, false, false);
}

void banner() {
    print(serial, crlf, "test_stm32f4_flash - the sector map, the engine, the option bytes, "
          "the stall", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
#if defined(STM32F446xx)
extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
#else
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
#endif
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void FLASH_IRQHandler() {
    flash_irq_flags = brio::Flash::isr();
    flash_irqs = flash_irqs + 1u;
}

// Letter k's two catchers. An imprecise bus error is recoverable: the
// store that raised it is already dead, so the handler clears the sticky
// bits and returns to whatever the core had reached. A storm of them
// would be a wedged core, so after eight the board reboots instead.
extern "C" void BusFault_Handler() {
    fault_cfsr = brio::Faults::read().cfsr;
    brio::Faults::clear();
    bus_faults = bus_faults + 1u;
    if (bus_faults > 8u) {
        brio::Reset::software();
    }
}
extern "C" void HardFault_Handler() {
    fault_cfsr = brio::Faults::read().cfsr;
    brio::Faults::clear();
    hard_faults = hard_faults + 1u;
    if (hard_faults > 8u) {
        brio::Reset::software();
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    // The ruler: TIM2 free-running at 1 MHz over its whole 32 bits.
    const uint32_t tim_hz = Ruler::clock_hz(clock);
    Ruler::init();
    const bool ruler_ok = Ruler::configure(brio::TimConfig{
        .prescaler = static_cast<uint16_t>(tim_hz / 1'000'000u - 1u), .period = 0xFFFFFFFFUL});
    Ruler::enable(true);

    brio::enable_interrupts();

    bench.letter('a', "the array, its sectors and its option bytes", ta_array);
    bench.letter('b', "the lock and the keyed unlock", tb_lock);
    bench.letter('c', "the four malformed sequences, each aborted", tc_missteps);
    bench.letter('d', "the error interrupt and the ISR body", td_interrupt);
    bench.letter('e', "erase and program the last sector", te_round_trip, false);
    bench.letter('f', "a second program into a programmed word", tf_second_write, false);
    bench.letter('g', "the erase stall, per sector size and per ART", tg_stall, false);
    bench.letter('h', "write protection through the option bytes", th_write_protection, false);
    bench.letter('k', "a wrong unlock key: the engine locked until reset", tk_wrong_key, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", " ruler=",
                    ruler_ok ? "TIM2 1 MHz" : "FAILED", brio::crlf);
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
