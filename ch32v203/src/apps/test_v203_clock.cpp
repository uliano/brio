// test_v203_clock - the reference bench suite for the CH32V203's CLOCK
// TREE (RM ch. 3 with the EXTEN bits that belong to it): the roots, the
// PLL and its two input dividers, the switch, the four prescalers, the
// clock output, the security system, the peripheral gates - and the
// dynamic regime over all of it, with the kernel tick and the console
// itself rebased at every step.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. Every clock this suite touches is inside the chip or
// on the board's own 8 MHz crystal, the console is the probe's serial
// (USART1 on PA9/PA10) and the clock output takes a pad nothing else on
// the board uses.
//
// THE CONSOLE IS THE INSTRUMENT AND THE SUBJECT AT ONCE. Every letter
// that moves the rate moves it through a DynamicClock whose users are
// the ticker and this very port, so a letter whose console dies is a
// letter that failed: the PASS line you are reading came out of a
// divisor computed for the rate it names.
//
// HOW A RATE IS MEASURED. Nothing inside the chip can judge its own
// clock - every counter here is derived from the same HCLK - so the
// letters that measure a rate BRACKET a span of kernel ticks between
// two console lines and leave the judging to whoever timed those two
// lines. The tick is exact by construction (ticker.hpp), so the ratio
// between the span asked and the seconds that passed IS the rate's
// deviation. The verdict is only that the tree switched and the console
// survived; the number is in the log.
//
// What is exercised, letter by letter:
//   a  the tree as the registers hold it at boot: the root SYSCLK runs
//      on, the four prescaler codes, the USB divider, the PLL's source,
//      multiplier and input divider (the HSI's lives in EXTEN), and the
//      two rates that follow - with the flash's own rule stated, since
//      this family has no wait-state field at all
//   b  every static rate the family offers on this board, each one
//      reached, read back and bracketed for the host's clock to judge:
//      the bare HSI, the PLL on the HSI at four rates (one of them
//      through the HALVED HSI, which is the EXTEN bit), the bare
//      crystal and the PLL on the crystal at four more
//   c  the dynamic regime through its whole pack, up and down: every
//      switch taken, the tick still exact across it, delay_us following
//      the rate by index, and the console rebased at each step
//   d  the LSI: stopped, started, and the time to LSIRDY measured
//   e  the HSE: the crystal's start-up time to HSERDY measured with the
//      ready INTERRUPT armed over the same ramp, and the HSI stopped and
//      restarted while the tree runs off the crystal
//   f  the clock security system armed over a running crystal: CSSON
//      reads back, the flag stays clear and the non-maskable interrupt
//      never fires
//   g  the clock output on PA8: every source this class has, set and
//      read back, and the pad claimed and released
//   h  the peripheral gates: every enable the chapter lists opened and
//      closed with the register read back, and what the register does
//      with the gates of blocks this package has not got
//
// build: boards = v203c6,v203c8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v203/clock.hpp"
#include "ch32v203/delay.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/ticker.hpp"
#include "ch32v203/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32v203Platform<>;
using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

// ---------------------------------------------------------------------------
// The rates. Each is a static Clock task - the tuple of root, PLL ratio,
// HPRE, both peripheral prescalers, the ADC divider and the USB one -
// and the pack is how one image reaches them all. The board's crystal is
// 8 MHz, which is also the HSI's rate, so the two roots reach the same
// four PLL rates and the pack names each of them twice: that is what
// set_index() is for.
//
// 52 MHz is not decoration: it is the one rate here that the PLL can
// only make from the HALVED input (4 MHz times thirteen), so it is what
// exercises EXTEN_CTR.HSIPRE in its other position.
// ---------------------------------------------------------------------------
inline constexpr uint32_t xtal_hz = 8'000'000;

using PllHsi144 = Clock<ClockSource::pll, 144'000'000>;
using PllHsi96 = Clock<ClockSource::pll, 96'000'000>;
using PllHsi48 = Clock<ClockSource::pll, 48'000'000>;
using PllHsi52 = Clock<ClockSource::pll, 52'000'000>;
using HsiBare = Clock<ClockSource::internal, 8'000'000>;
using PllXtal144 = Clock<ClockSource::pll, 144'000'000, xtal_hz>;
using PllXtal96 = Clock<ClockSource::pll, 96'000'000, xtal_hz>;
using PllXtal72 = Clock<ClockSource::pll, 72'000'000, xtal_hz>;
using PllXtal48 = Clock<ClockSource::pll, 48'000'000, xtal_hz>;
using XtalBare = Clock<ClockSource::crystal, 8'000'000, xtal_hz>;

using SysClock = DynamicClock<Rates<PllHsi144, PllHsi96, PllHsi52, PllHsi48, HsiBare,
                                    PllXtal144, PllXtal96, PllXtal72, PllXtal48, XtalBare>,
                              Ticker, Serial>;
constexpr SysClock clock;

/// The pack's own names, in its order: the tuple says the rate and the
/// root, but not which oscillator the PLL is fed from, and that is the
/// half this suite is about.
constexpr const char* rate_names[] = {
    "PLL on the HSI",       "PLL on the HSI",       "PLL on the HALVED HSI",
    "PLL on the HSI",       "the bare HSI",         "PLL on the crystal",
    "PLL on the crystal",   "PLL on the crystal",   "PLL on the crystal",
    "the bare crystal",
};
constexpr uint8_t boot_rate = 0;               ///< PllHsi144, what main() enters
constexpr uint8_t xtal_144_rate = 5;           ///< PllXtal144, letter e's crystal tree
static_assert(sizeof(rate_names) / sizeof(rate_names[0]) == SysClock::rate_count);

TestBench<Serial> bench;

/// The non-maskable interrupt's tally (letter f): the clock security
/// system's own count, and anything else that took that vector.
volatile uint32_t css_events = 0;
volatile uint32_t nmi_others = 0;

/// The RCC vector's tally (letter e): how many times a root's ready flag
/// raised the line, and which flags the body found standing.
volatile uint32_t rcc_events = 0;
volatile uint32_t rcc_flags = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 200);   // the last byte's own time on the wire
}

const char* source_name(SysclkSource s) {
    switch (s) {
        case SysclkSource::hsi: return "HSI";
        case SysclkSource::hse: return "HSE";
        case SysclkSource::pll: return "PLL";
    }
    return "?";
}

const char* mco_name(McoSource s) {
    switch (s) {
        case McoSource::none:     return "none";
        case McoSource::sysclk:   return "SYSCLK";
        case McoSource::hsi:      return "HSI";
        case McoSource::hse:      return "HSE";
        case McoSource::pll_div2: return "PLL/2";
    }
    return "?";
}

/// Step a poll in 50 us units until it answers, and say how long it
/// took. The microsecond wait is the one instrument that follows the
/// rate by itself (delay.hpp dispatches on the pack's index), so this
/// reads the same at 8 MHz and at 144.
template <typename Fn>
uint32_t wait_us(Fn ready, uint32_t limit_us) {
    uint32_t spent = 0;
    while (!ready() && spent < limit_us) {
        (void)delay_us(clock, 50);
        spent += 50;
    }
    return spent;
}

/// Move to one rate of the pack, with the console drained first so no
/// byte is in the shift register when the divisor changes.
bool go_to(uint8_t i) {
    console_drain();
    return SysClock::set_index(i);
}

/// The two lines whoever timed them turns into a rate. Between them the
/// program does nothing but count its own ticks.
void bracket(uint8_t i, uint32_t span) {
    print(serial, "  rate ", i, " (", rate_names[i], ", ", SysClock::rate_hz(i),
          " Hz): bracket ", span, " ticks", crlf);
    console_drain();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < span) {
    }
    print(serial, "  rate ", i, " done", crlf);
}

/// What a rate must be able to say about itself after a switch.
bool rate_in_force(uint8_t i) {
    return SysClock::rate_index() == i && SysClock::hz() == SysClock::rate_hz(i) &&
           Rcc::sysclk_status() == SysClock::rate_source(i);
}

// ---------------------------------------------------------------------------
// a - the tree as the registers hold it
// ---------------------------------------------------------------------------
void ta_tree() {
    print(serial, "  SYSCLK on ", source_name(Rcc::sysclk_status()), ", HCLK ",
          SysClock::hz(), " Hz, PCLK1 ", SysClock::pclk1_hz(), " Hz, PCLK2 ",
          SysClock::pclk2_hz(), " Hz", crlf);
    print(serial, "  HPRE=", Rcc::hpre_code(), " (/", hpre_divider(Rcc::hpre_code()),
          ") PPRE1=", Rcc::ppre1_code(), " (/", ppre_divider(Rcc::ppre1_code()),
          ") PPRE2=", Rcc::ppre2_code(), " (/", ppre_divider(Rcc::ppre2_code()),
          ") ADCPRE=", Rcc::adc_prescaler(), " (/",
          adcpre_divider(Rcc::adc_prescaler()), ")", crlf);
    print(serial, "  USBPRE=", Rcc::usb_prescaler(), " (/",
          usbpre_divider(Rcc::usb_prescaler()), ") -> ", SysClock::usb_hz(),
          " Hz; ADC -> ", SysClock::adc_hz(), " Hz, in spec: ",
          SysClock::adc_in_spec(), crlf);
    print(serial, "  PLL from the ", Rcc::pll_from_hse() ? "HSE" : "HSI",
          ", input divided: ", Rcc::pll_input_divided(), " (the HSI's divider is "
          "EXTEN's HSIPRE), x", Rcc::pll_multiplier(), crlf);
    print(serial, "  HSI trim=", Rcc::hsi_trim(), " factory calibration=",
          hex(Rcc::hsi_calibration()), crlf);
    print(serial, "  no flash wait-state field exists on this family; what the "
                  "flash chapter asks for at this rate is HCLK halved around an "
                  "erase or a program: ", PllHsi144::flash_needs_halving, crlf);

    bench.verdict("the boot rate is in force, and the registers say so",
                  rate_in_force(boot_rate));
    bench.verdict("SWS reports the PLL", Rcc::sysclk_status() == SysclkSource::pll);
    bench.verdict("HPRE leaves SYSCLK undivided at the family ceiling",
                  Rcc::hpre_code() == PllHsi144::hpre_code && hpre_divider(Rcc::hpre_code()) == 1u);
    bench.verdict("PB1 is halved above its cap and PB2 is not",
                  ppre_divider(Rcc::ppre1_code()) == 2u && ppre_divider(Rcc::ppre2_code()) == 1u);
    bench.verdict("PCLK1 is half of HCLK and PCLK2 is all of it",
                  SysClock::pclk1_hz() == SysClock::hz() / 2u &&
                      SysClock::pclk2_hz() == SysClock::hz());
    bench.verdict("the USB divider was programmed before any USB gate could "
                  "open, and gives 48 MHz",
                  usbpre_divider(Rcc::usb_prescaler()) == 3u &&
                      SysClock::usb_hz() == 48'000'000UL);
    bench.verdict("the ADC prescaler is the slowest the field has, and even "
                  "that is above the converter's rating at this rate",
                  Rcc::adc_prescaler() == 3u && !SysClock::adc_in_spec());
    bench.verdict("the PLL is fed by the HSI, whole, times eighteen",
                  !Rcc::pll_from_hse() && !Rcc::pll_input_divided() &&
                      Rcc::pll_multiplier() == 18u);
    bench.verdict("the HSI's trim is the reset value, the centre of the field",
                  Rcc::hsi_trim() == 16u);
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() == t0) {
    }
    bench.verdict("the kernel tick is running on this tree",
                  Ticker::ticks() != t0);
}

// ---------------------------------------------------------------------------
// b - every static rate, bracketed for the host's clock
// ---------------------------------------------------------------------------
void tb_rates() {
    constexpr uint32_t span = 2000;   // ticks, i.e. 2 s if the tick is a ms
    print(serial, "  each rate is entered, read back and bracketed; the number "
                  "is the host's to compute from the two lines", crlf);

    for (uint8_t i = 0; i < SysClock::rate_count; ++i) {
        const bool ok = go_to(i);
        const bool in_force = rate_in_force(i);
        bracket(i, span);
        bench.verdict("the tree took this rate and this line came out of it: ",
                      rate_names[i], ok && in_force);
    }

    (void)go_to(boot_rate);
    bench.verdict("and back to the boot rate", rate_in_force(boot_rate));
}

// ---------------------------------------------------------------------------
// c - the pack up and down
// ---------------------------------------------------------------------------
void tc_dynamic() {
    constexpr uint32_t span = 200;
    bool up_ok = true;
    bool tick_ok = true;
    bool delay_ok = true;

    for (uint8_t i = 0; i < SysClock::rate_count; ++i) {
        up_ok = go_to(i) && rate_in_force(i) && up_ok;
        const uint32_t t0 = Ticker::ticks();
        while (Ticker::ticks() - t0 < span) {
        }
        tick_ok = (Ticker::ticks() - t0 == span) && tick_ok;
        delay_ok = delay_us(clock, 500) && delay_ok;
        print(serial, "  up   ", i, " -> ", SysClock::hz(), " Hz (", rate_names[i],
              "), PCLK1 ", SysClock::pclk1_hz(), " Hz", crlf);
    }
    bench.verdict("every rate of the pack entered in order, upwards through "
                  "the list",
                  up_ok);

    bool down_ok = true;
    for (uint8_t k = SysClock::rate_count; k > 0; --k) {
        const uint8_t i = static_cast<uint8_t>(k - 1u);
        const bool ok = go_to(i) && rate_in_force(i);
        const uint32_t t0 = Ticker::ticks();
        while (Ticker::ticks() - t0 < span) {
        }
        tick_ok = (Ticker::ticks() - t0 == span) && tick_ok;
        delay_ok = delay_us(clock, 500) && delay_ok;
        print(serial, "  down ", i, " -> ", SysClock::hz(), " Hz, ",
              ok ? "in force" : "NOT IN FORCE", crlf);
        down_ok = ok && down_ok;
    }
    bench.verdict("and back down through it, the console rebased at every step",
                  down_ok);
    bench.verdict("the tick counted exactly the span asked at every rate",
                  tick_ok);
    bench.verdict("delay_us served its wait at every rate, dispatching on the "
                  "pack's index",
                  delay_ok);

    bench.verdict("set() by rate refuses a rate the pack does not name",
                  !SysClock::set(37'000'000UL) && !SysClock::set_index(99));

    (void)go_to(boot_rate);
    bench.verdict("the boot rate is back", rate_in_force(boot_rate));
    bench.verdict("restore() is a readback when the rate is already in force",
                  SysClock::restore() && rate_in_force(boot_rate));
}

// ---------------------------------------------------------------------------
// d - the LSI
// ---------------------------------------------------------------------------
void td_lsi() {
    (void)Rcc::lsi_start();
    bench.verdict("the LSI starts and reports itself ready", Rcc::lsi_ready());

    Rcc::lsi_stop();
    const uint32_t fall = wait_us([] { return !Rcc::lsi_ready(); }, 20'000);
    print(serial, "  LSIRDY fell ", fall, " us after LSION was cleared", crlf);
    bench.verdict("stopping it drops LSIRDY", !Rcc::lsi_ready());

    Rcc::lsi_enable(true);
    const uint32_t rise = wait_us([] { return Rcc::lsi_ready(); }, 20'000);
    print(serial, "  LSIRDY rose ", rise, " us after LSION was set (the "
                  "datasheet's figure is 230 us with the LSE running and 5 ms "
                  "without)", crlf);
    bench.verdict("and starting it raises LSIRDY again", Rcc::lsi_ready());

    print(serial, "  its RATE is nominal only - ", device::lsi_min_hz, " to ",
          device::lsi_max_hz, " Hz, typically ", device::lsi_typ_hz,
          " - and nothing in this chapter can count it: the watchdog and the "
          "RTC are what will", crlf);
}

// ---------------------------------------------------------------------------
// e - the HSE, and the HSI stopped under a tree that does not need it
// ---------------------------------------------------------------------------
void te_hse() {
    Rcc::hse_stop();
    const uint32_t fall = wait_us([] { return !Rcc::hse_ready(); }, 20'000);
    print(serial, "  HSERDY fell ", fall, " us after HSEON was cleared", crlf);

    // The ready INTERRUPT, armed over the same ramp: the flag that is
    // about to rise is the one the RCC vector reports.
    Rcc::clear_interrupt_flags(rcc_intr_flags);
    rcc_events = 0;
    rcc_flags = 0;
    Rcc::ready_interrupts(rcc_hserdyie);
    Pfic::enable(Irq::rcc);

    Rcc::hse_enable(false);
    const uint32_t rise = wait_us([] { return Rcc::hse_ready(); }, 100'000);
    print(serial, "  HSERDY rose ", rise, " us after HSEON was set (the "
                  "datasheet's figure for an 8 MHz crystal is 2.5 ms)", crlf);
    bench.verdict("the board's crystal starts and HSERDY reads back",
                  Rcc::hse_ready());

    print(serial, "  the RCC vector ran ", rcc_events, " time(s), the body "
                  "finding flags ", hex(rcc_flags), crlf);
    bench.verdict("the ready interrupt reached the RCC vector, and the body "
                  "found the HSE's own flag standing",
                  rcc_events != 0u && (rcc_flags & rcc_hserdyf) != 0u);
    Rcc::ready_interrupts(0);
    Pfic::disable(Irq::rcc);
    bench.verdict("and the body left no flag behind, so the line does not "
                  "re-enter",
                  (Rcc::interrupt_flags() & rcc_hserdyf) == 0u);

    // The HSI is only a root while something uses it. Under a tree whose
    // PLL is fed by the crystal, nothing does - so it can be stopped,
    // which is the one way to prove the verb does something.
    const bool moved = go_to(xtal_144_rate) && rate_in_force(xtal_144_rate);
    bench.verdict("the tree moved onto the crystal's PLL", moved);
    Rcc::hsi_stop();
    (void)delay_us(clock, 100);
    const bool hsi_off = !Rcc::hsi_ready();
    print(serial, "  with SYSCLK on the crystal's PLL, HSION cleared -> "
                  "HSIRDY=", Rcc::hsi_ready(), crlf);
    const bool hsi_back = Rcc::hsi_start();
    bench.verdict("the HSI stops while the tree runs off the crystal", hsi_off);
    bench.verdict("and comes back on demand", hsi_back && Rcc::hsi_ready());

    (void)go_to(boot_rate);
    bench.verdict("the boot rate is back, which needs the HSI it just lost",
                  rate_in_force(boot_rate));
    Rcc::hse_stop();
}

// ---------------------------------------------------------------------------
// f - the clock security system
// ---------------------------------------------------------------------------
void tf_css() {
    const uint32_t before = css_events;
    const bool started = Rcc::hse_start(false);
    bench.verdict("the crystal is running, which is what arms the detector",
                  started);

    Rcc::clock_monitor(true);
    bench.verdict("CSSON reads back", Rcc::clock_monitor());
    bench.verdict("and no failure stands", !Rcc::clock_failed());

    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < 100u) {
    }
    print(serial, "  100 ms armed over a healthy crystal: CSS events ",
          css_events - before, ", other non-maskable interrupts ", nmi_others, crlf);
    bench.verdict("the non-maskable interrupt never fired", css_events == before);
    bench.verdict("and nothing else took that vector", nmi_others == 0u);

    Rcc::clock_monitor(false);
    bench.verdict("CSSON clears", !Rcc::clock_monitor());
    Rcc::hse_stop();

    print(serial, "  the failure itself is not staged here: it wants the "
                  "crystal killed on the board, and what the silicon does with "
                  "it - SYSCLK back on the HSI, the PLL off, the flag set - is "
                  "in the driver's header and not measured", crlf);
}

// ---------------------------------------------------------------------------
// g - the clock output
// ---------------------------------------------------------------------------
void tg_mco() {
    bench.verdict("this package brings the output pad out", Mco::has_pad);

    constexpr McoSource sources[] = {McoSource::sysclk, McoSource::hsi, McoSource::hse,
                                     McoSource::pll_div2};
    bool all_ok = true;
    for (const McoSource src : sources) {
        const bool claimed = Mco::init(src);
        const bool reads = Mco::source() == src;
        print(serial, "  ", mco_name(src), ": pad claimed ", claimed,
              ", multiplexer reads ", mco_name(Mco::source()), crlf);
        all_ok = claimed && reads && all_ok;
    }
    bench.verdict("every source this device class has is selected and reads "
                  "back",
                  all_ok);

    Mco::off();
    bench.verdict("off() takes the multiplexer to no clock and releases the pad",
                  Mco::source() == McoSource::none);

    print(serial, "  what comes OUT of the pad is not measured here: it wants a "
                  "counter, which arrives with the timers", crlf);
}

// ---------------------------------------------------------------------------
// h - the peripheral gates
// ---------------------------------------------------------------------------
struct Gate {
    const char* name;
    Bus bus;
    uint32_t mask;
};

constexpr Gate gates[] = {
    {"HB DMA1", Bus::hb, rcc_hb_dma1},
    {"HB SRAM", Bus::hb, rcc_hb_sram},
    {"HB CRC", Bus::hb, rcc_hb_crc},
    {"HB USBFS", Bus::hb, rcc_hb_usbfs},
    {"PB2 AFIO", Bus::pb2, rcc_pb2_afio},
    {"PB2 GPIOB", Bus::pb2, rcc_pb2_gpiob},
    {"PB2 GPIOC", Bus::pb2, rcc_pb2_gpioc},
    {"PB2 GPIOD", Bus::pb2, rcc_pb2_gpiod},
    {"PB2 ADC1", Bus::pb2, rcc_pb2_adc1},
    {"PB2 ADC2", Bus::pb2, rcc_pb2_adc2},
    {"PB2 TIM1", Bus::pb2, rcc_pb2_tim1},
    {"PB2 SPI1", Bus::pb2, rcc_pb2_spi1},
    {"PB1 TIM2", Bus::pb1, rcc_pb1_tim2},
    {"PB1 TIM3", Bus::pb1, rcc_pb1_tim3},
    {"PB1 TIM4", Bus::pb1, rcc_pb1_tim4},
    {"PB1 WWDG", Bus::pb1, rcc_pb1_wwdg},
    {"PB1 SPI2", Bus::pb1, rcc_pb1_spi2},
    {"PB1 USART2", Bus::pb1, rcc_pb1_usart2},
    {"PB1 USART3", Bus::pb1, rcc_pb1_usart3},
    {"PB1 UART4", Bus::pb1, rcc_pb1_uart4},
    {"PB1 I2C1", Bus::pb1, rcc_pb1_i2c1},
    {"PB1 I2C2", Bus::pb1, rcc_pb1_i2c2},
    {"PB1 USBD", Bus::pb1, rcc_pb1_usbd},
    {"PB1 CAN1", Bus::pb1, rcc_pb1_can1},
    {"PB1 BKP", Bus::pb1, rcc_pb1_bkp},
    {"PB1 PWR", Bus::pb1, rcc_pb1_pwr},
};

/// The gates of blocks this package has NOT got. Nothing says what a
/// gate does there, so the letter reports what it found instead of
/// judging it.
constexpr Gate absent_gates[] = {
    {"HB DMA2 (no such block on this family)", Bus::hb, rcc_hb_dma2},
    {"PB2 GPIOE (no such port on this series)", Bus::pb2, rcc_pb2_gpioe},
    {"PB1 TIM5 (the CH32V203RB's alone)", Bus::pb1, rcc_pb1_tim5},
};

void th_gates() {
    bool all_ok = true;
    uint32_t tried = 0;
    for (const Gate& g : gates) {
        const bool was = Rcc::enabled(g.bus, g.mask);
        Rcc::enable(g.bus, g.mask);
        const bool on = Rcc::enabled(g.bus, g.mask);
        Rcc::disable(g.bus, g.mask);
        const bool off = !Rcc::enabled(g.bus, g.mask);
        if (was) {
            Rcc::enable(g.bus, g.mask);
        }
        if (!on || !off) {
            print(serial, "  ", g.name, ": opened ", on, " closed ", off, crlf);
        }
        all_ok = on && off && all_ok;
        ++tried;
    }
    print(serial, "  ", tried, " gates opened and closed, each read back", crlf);
    bench.verdict("every gate the chapter lists for this part opens and closes, "
                  "read back one at a time",
                  all_ok);
    print(serial, "  USART1's and GPIOA's gates are the two left alone: this "
                  "console runs on them", crlf);

    for (const Gate& g : absent_gates) {
        const bool was = Rcc::enabled(g.bus, g.mask);
        Rcc::enable(g.bus, g.mask);
        const bool on = Rcc::enabled(g.bus, g.mask);
        Rcc::disable(g.bus, g.mask);
        if (was) {
            Rcc::enable(g.bus, g.mask);
        }
        print(serial, "  ", g.name, ": the bit holds a one: ", on, crlf);
    }
    // The two resets of a block that is not in use: pulsing them proves
    // the verb reaches the register, and CRC has nothing to lose.
    Rcc::enable(Bus::hb, rcc_hb_crc);
    Rcc::reset(Bus::hb, rcc_hb_crc);
    bench.verdict("a peripheral reset pulse leaves its gate where it was",
                  Rcc::enabled(Bus::hb, rcc_hb_crc));
    Rcc::disable(Bus::hb, rcc_hb_crc);
}

void banner() {
    print(serial, crlf, "test_v203_clock on ", device::part_name, " - the clock tree",
          crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

// The three vectors this program owns. The non-maskable one is the
// clock security system's: the driver's body says whether the CSS was
// the reason, and only this suite's own counters are touched here.
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

extern "C" BRIO_CH32_INTERRUPT void rcc_handler() {
    rcc_flags = rcc_flags | brio::Rcc::ready_isr();
    rcc_events = rcc_events + 1u;
}

extern "C" BRIO_CH32_INTERRUPT void nmi_handler() {
    if (brio::Rcc::css_isr()) {
        css_events = css_events + 1u;
    } else {
        nmi_others = nmi_others + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the tree as the registers hold it at boot", ta_tree);
    bench.letter('b', "every static rate, bracketed for the host's clock", tb_rates);
    bench.letter('c', "the dynamic pack, up and down", tc_dynamic);
    bench.letter('d', "the LSI and the time to LSIRDY", td_lsi);
    bench.letter('e', "the crystal's start-up, its ready interrupt, and the "
                      "HSI stopped", te_hse);
    bench.letter('f', "the clock security system over a healthy crystal", tf_css);
    bench.letter('g', "the clock output on its pad", tg_mco);
    bench.letter('h', "the peripheral gates, one at a time", th_gates);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=",
                    clock_ok ? "PLL144" : "FAILED", " tick=",
                    tick_ok ? "STK" : "FAILED", brio::crlf);
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched",
                    brio::crlf);
        bench.prompt();
    }
}
