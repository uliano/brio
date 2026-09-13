// test_stm32f4_platform - the reference bench suite for the STM32F4's
// PLATFORM: the running half (the PRIMASK critical section, the idle
// hook, the SysTick timebase), stm32f4/delay.hpp (the microsecond
// busy-wait), the clock tree as the registers hold it against what the
// Clock task's constants say, and the panic breadcrumb without a reset.
// The failing half - the reset causes, the watchdogs, the breadcrumb
// across a real reset - is the reset chapter's suite to come.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The console is the board's ST-LINK virtual COM port
// (or the probe's bridge on the black pill, as console.cpp says) and
// every clock this suite measures is inside the chip. THERE IS NO
// INDEPENDENT RULER on this family yet (the RTC and the timers are
// chapters of their own), so the SysTick tick judges delay_us in bulk
// and delay_us is judged for its at-least contract against SysTick's
// own counter; the tick's own rate is judged by the reload's arithmetic
// and, on the desk, by the console's uptime against a clock.
//
// What is exercised, letter by letter:
//   a  the boot story: the device id and revision, the unique id, the
//      flash size, no breadcrumb pending
//   b  the critical section: PRIMASK saved and restored, nesting, and
//      the idle hook returning on the tick
//   c  the SysTick timebase: the reload, ticks/millis/secs/now
//      coherence, and the VAL-delta accumulation that delay.hpp is
//      built on checked against the interrupt count
//   d  delay_us: a thousand 100 us waits against the kernel tick, never
//      early on SysTick's counter, the cap, the no-counter refusal
//   e  the clock tree: SWS, the PLL's lock, the HSE's mode, the
//      regulator scale and over-drive, the flash latency and the
//      accelerator, the APB dividers - each register against the task's
//      constant
//   g  the panic breadcrumb WITHOUT a reset: written, taken once, gone
//
// build: boards = f429zi,f446re,f411ce
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/panic.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/flash.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/pwr.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
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

std::optional<PanicRecord> boot_record;

/// Let the console's own transmitter fall silent: its interrupt would
/// otherwise wake every idle() below.
void console_drain() {
    const uint32_t t0 = Ticker::ticks();
    while (!Serial::tx_idle() && Ticker::ticks() - t0 < 200u) {
    }
}

// =============================================================================
// a - the boot story
// =============================================================================
void ta_boot() {
    const DeviceIdcode id = DeviceIdcode::read();
    const DeviceUid uid = DeviceUid::read();
    print(serial, "  DEV_ID ", hex(id.dev_id), " REV_ID ", hex(id.rev_id), " flash ",
          flash_size_kbytes(), " KB uid ", hex(uid.word[0]), "-", hex(uid.word[1]), "-",
          hex(uid.word[2]), crlf);
#if defined(STM32F429xx)
    bench.verdict("DEV_ID 0x419: an STM32F42x/F43x", id.dev_id == 0x419u);
    bench.verdict("the flash size register says 2048 KB", flash_size_kbytes() == 2048u);
#elif defined(STM32F446xx)
    bench.verdict("DEV_ID 0x421: an STM32F446", id.dev_id == 0x421u);
    bench.verdict("the flash size register says 512 KB", flash_size_kbytes() == 512u);
#elif defined(STM32F411xE)
    bench.verdict("DEV_ID 0x431: an STM32F411", id.dev_id == 0x431u);
    bench.verdict("the flash size register says 512 KB", flash_size_kbytes() == 512u);
#endif
    bench.verdict("the unique id is not blank", (uid.word[0] | uid.word[1] | uid.word[2]) != 0u);
    bench.verdict("no breadcrumb is pending on a clean start", !boot_record);
}

// =============================================================================
// b - the critical section and the idle hook
// =============================================================================
void tb_critical() {
    bench.verdict("interrupts are enabled when a letter runs", P::interrupts_enabled());

    bool inside = true, nested = true, after_inner = false, after_outer = false;
    {
        P::CriticalSection cs;
        inside = P::interrupts_enabled();
        {
            P::CriticalSection inner;
            nested = P::interrupts_enabled();
        }
        after_inner = P::interrupts_enabled();
    }
    after_outer = P::interrupts_enabled();
    bench.verdict("a critical section masks", !inside);
    bench.verdict("a nested one is still masked", !nested);
    bench.verdict("leaving the INNER scope does not unmask (PRIMASK is saved "
                  "and restored, not cleared)",
                  !after_inner);
    bench.verdict("leaving the outer scope unmasks", after_outer);

    // The tick keeps counting through a masked window - the interrupt is
    // pending, not lost - but a masked window LOSES ticks: SysTick's
    // interrupt is a pending bit, so five periods under the mask deliver
    // one tick when the mask lifts.
    const uint32_t period = SysTick->LOAD + 1u;
    const uint32_t t0 = Ticker::ticks();
    uint32_t cycles = 0;
    {
        P::CriticalSection cs;
        uint32_t last = SysTick->VAL;
        while (cycles < 5u * period) {
            const uint32_t now = SysTick->VAL;
            cycles += (last >= now) ? (last - now) : (last + period - now);
            last = now;
        }
    }
    // The pending interrupt is taken AFTER the mask lifts, not inside the
    // instruction that lifts it: give the core the ISB the architecture
    // asks for before reading what the handler delivered (PM0214 2.1.3).
    __ISB();
    const uint32_t through_mask = Ticker::ticks() - t0;
    print(serial, "  five SysTick periods under the mask advanced the tick by ", through_mask,
          " ms", crlf);
    bench.verdict("a masked window LOSES ticks (one delivered, the rest coalesce)",
                  through_mask >= 1u && through_mask <= 2u);

    // idle(): WFI, then unmask. Any enabled interrupt wakes it, so the
    // console has to be silent first.
    console_drain();
    const uint32_t t1 = Ticker::ticks();
    uint8_t calls = 0;
    while (Ticker::ticks() == t1 && calls < 20u) {
        P::idle();
        ++calls;
    }
    print(serial, "  with the console silent, ", calls, " idle() call(s) reached the next tick",
          crlf);
    bench.verdict("idle() sleeps and the tick is what brings it back", calls >= 1u && calls < 20u);
    bench.verdict("and it comes back with interrupts enabled", P::interrupts_enabled());
}

// =============================================================================
// c - the SysTick timebase
// =============================================================================
void tc_ticker() {
    const uint32_t reload = SysTick->LOAD;
    print(serial, "  SysTick CTRL=", hex(SysTick->CTRL), " LOAD=", reload, " (",
          SysClock::hz / Ticker::ticks_per_second - 1u, " expected)", crlf);
    bench.verdict("the reload is Clock::hz / tps - 1",
                  reload == SysClock::hz / Ticker::ticks_per_second - 1u);
    bench.verdict("the counter runs on the processor clock with its interrupt armed",
                  (SysTick->CTRL & (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                                    SysTick_CTRL_ENABLE_Msk)) ==
                      (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                       SysTick_CTRL_ENABLE_Msk));

    const uint32_t ticks = Ticker::ticks();
    const uint32_t ms = Ticker::millis();
    const uint32_t secs = Ticker::secs();
    TimeStamp stamp{};
    Ticker::now(stamp);
    print(serial, "  ticks=", ticks, " millis=", ms, " secs=", secs, " now=", stamp, crlf);
    bench.verdict("millis() is ticks() at 1000 Hz (no correction, no drift)",
                  ms >= ticks && ms - ticks <= 2u);
    bench.verdict("secs() is the whole-second part of the same counter",
                  secs == ticks / 1000u || secs == (ticks + 2u) / 1000u);
    bench.verdict("now() agrees with both counters",
                  stamp.seconds == secs || stamp.seconds == secs + 1u);

    // The VAL-delta accumulation delay.hpp is built on, against the
    // interrupt count over 200 wraps.
    const uint32_t target = 200;
    const uint32_t period = reload + 1u;
    uint32_t t1 = Ticker::ticks();
    while (Ticker::ticks() == t1) {
    }
    t1 = Ticker::ticks();
    uint32_t last = SysTick->VAL;
    uint32_t accumulated = 0;
    while (Ticker::ticks() - t1 < target) {
        const uint32_t now = SysTick->VAL;
        accumulated += (last >= now) ? (last - now) : (last + period - now);
        last = now;
    }
    const uint32_t expected = target * period;
    const uint32_t cerr = accumulated > expected ? accumulated - expected : expected - accumulated;
    print(serial, "  200 ticks = ", accumulated, " cycles by VAL deltas, ", expected,
          " expected, error ", cerr, " cycles", crlf);
    bench.verdict("the VAL-delta accumulation tracks the interrupt count over "
                  "200 wraps to under a period's worth of error",
                  cerr < period);
}

// =============================================================================
// d - delay_us on the SysTick counter
// =============================================================================
void td_delay() {
    const uint32_t period = SysTick->LOAD + 1u;
    const uint32_t cycles_per_us = delay_rate(SysClock::hz).cycles_per_us;

    // Each wait measured on SysTick's own counter (the same ruler the
    // wait reads, so this judges the ARITHMETIC: at least, never early).
    static const uint32_t spans[] = {5, 30, 100, 500, 900};
    bool all_at_least = true;
    for (uint8_t i = 0; i < sizeof(spans) / sizeof(spans[0]); ++i) {
        const uint32_t v0 = SysTick->VAL;
        const bool ok = delay_us(clock, spans[i]);
        const uint32_t v1 = SysTick->VAL;
        const uint32_t took = (v0 >= v1) ? (v0 - v1) : (v0 + period - v1);
        const uint32_t took_us = took / cycles_per_us;
        print(serial, "  delay_us(", spans[i], ") -> ", took_us, " us (", took, " cycles)", crlf);
        if (!ok || took < spans[i] * cycles_per_us) {
            all_at_least = false;
        }
    }
    bench.verdict("delay_us serves 5..900 us AT LEAST on its own counter", all_at_least);

    const uint32_t t0 = Ticker::ticks();
    for (uint16_t k = 0; k < 1000; ++k) {
        (void)delay_us(clock, 100u);
    }
    const uint32_t bulk_ms = Ticker::ticks() - t0;
    print(serial, "  1000 x delay_us(100) = ", bulk_ms, " ms of kernel tick (100 due)", crlf);
    bench.verdict("a thousand 100 us waits are 100 ms of kernel time, never less, "
                  "and the per-call overhead costs under 10%",
                  bulk_ms >= 100u && bulk_ms <= 110u);

    bool refused = true;
    for (uint8_t k = 0; k < 4; ++k) {
        refused = refused && !delay_us(clock, 1000u);
    }
    bench.verdict("a wait of one SysTick period or more is REFUSED (TimeEvent territory)",
                  refused);
    bench.verdict("a wait of 65536 us is refused too", !delay_us(clock, 65536u));
}

/// Whether this program's clock names the HSE in bypass (the Nucleo's
/// MCO) or as a crystal - a constexpr of the board file's line.
constexpr bool hse_is_bypass() {
#if defined(STM32F446xx)
    return true;
#else
    return false;
#endif
}

// =============================================================================
// e - the clock tree as the registers hold it
// =============================================================================
void te_clock() {
    print(serial, "  SWS=", static_cast<uint8_t>(Rcc::sysclk_status()), " PLL ",
          Rcc::pll_ready() ? "locked" : "OFF", " HSE ",
          Rcc::hse_ready() ? (Rcc::hse_bypassed() ? "bypass" : "crystal") : "OFF",
          " VOS scale", static_cast<uint8_t>(Pwr::scale()), " OD ",
          Pwr::over_drive_active() ? "on" : "off", " WS ", FlashWaitStates::get(),
          " PPRE1 /", Rcc::apb1_divider(), " PPRE2 /", Rcc::apb2_divider(), crlf);
    bench.verdict("SYSCLK is the PLL (SWS reads 10)", Rcc::sysclk_status() == SysclkSource::pll);
    bench.verdict("the PLL is locked", Rcc::pll_ready());
    bench.verdict("the HSE root is ready, in the mode the board file states",
                  Rcc::hse_ready() && Rcc::hse_bypassed() == hse_is_bypass());
    bench.verdict("the regulator scale is the one the ladder picked for this rate",
                  Pwr::scale() == SysClock::regime.scale);
    bench.verdict("over-drive is active exactly where the rate needs it",
                  Pwr::over_drive_active() == SysClock::regime.over_drive);
    bench.verdict("the flash latency is the task's wait states",
                  FlashWaitStates::get() == SysClock::wait_states);
    bench.verdict("the ART accelerator is on (prefetch, both caches)",
                  FlashAccel::prefetch() && FlashAccel::icache() && FlashAccel::dcache());
    bench.verdict("the APB dividers are the task's",
                  Rcc::apb1_divider() == SysClock::apb1_div && Rcc::apb2_divider() == SysClock::apb2_div);
    bench.verdict("HPRE is 1: HCLK is SYSCLK", Rcc::ahb_undivided());
    bench.verdict("PLLCFGR holds the ratio the task searched",
                  (RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos == SysClock::pll.m &&
                  (RCC->PLLCFGR & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos == SysClock::pll.n &&
                  ((RCC->PLLCFGR & RCC_PLLCFGR_PLLP_Msk) >> RCC_PLLCFGR_PLLP_Pos) == (SysClock::pll.p / 2u - 1u) &&
                  (RCC->PLLCFGR & RCC_PLLCFGR_PLLQ_Msk) >> RCC_PLLCFGR_PLLQ_Pos == SysClock::pll.q);
}

// =============================================================================
// g - the panic breadcrumb, no reset
// =============================================================================
void tg_breadcrumb() {
    print(serial, "  panic record at ", hex(reinterpret_cast<uintptr_t>(&P::panic_record())), crlf);
    bench.verdict("the .noinit record sits in the main SRAM",
                  reinterpret_cast<uintptr_t>(&P::panic_record()) >= 0x20000000u &&
                      reinterpret_cast<uintptr_t>(&P::panic_record()) < 0x20040000u);
    bench.verdict("nothing is pending", !take_panic_record<P>());
    P::panic_record() = PanicRecord{panic_magic, static_cast<uint8_t>(PanicCode::assert_failed), 0x42};
    const auto taken = take_panic_record<P>();
    bench.verdict("a written record is taken with its code and context",
                  taken && taken->code == static_cast<uint8_t>(PanicCode::assert_failed) &&
                      taken->context == 0x42);
    bench.verdict("and taken only once", !take_panic_record<P>());
}

void banner() {
    print(serial, crlf, "test_stm32f4_platform - the STM32F4 platform, SysTick, delay_us, "
          "the clock tree, the breadcrumb", crlf);
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

int main() {
    boot_record = brio::take_panic_record<P>();

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the boot story", ta_boot);
    bench.letter('b', "the critical section and the idle hook", tb_critical);
    bench.letter('c', "the SysTick timebase and its VAL arithmetic", tc_ticker);
    bench.letter('d', "delay_us on the SysTick counter", td_delay);
    bench.letter('e', "the clock tree against the task's constants", te_clock);
    bench.letter('g', "the panic breadcrumb, no reset", tg_breadcrumb);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
