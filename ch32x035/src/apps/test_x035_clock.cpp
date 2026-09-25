// test_x035_clock - the reference bench suite for the CLOCK TREE of the
// CH32X035 (RM ch. 3 with the flash's wait states of 20.3.1): the one
// root, the HSI and its trim, the HPRE ladder walked at run time with the
// kernel tick and the console itself rebased at every rung, the clock
// output, and the peripheral gates.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// NOTHING TO WIRE. The console is USART2 on PA2/PA3 over the probe's
// serial, and every clock this suite touches is inside the chip.
//
// THE CONSOLE IS THE INSTRUMENT AND THE SUBJECT AT ONCE. Every letter that
// moves the rate moves it through a DynamicClock whose users are the
// ticker and this very port, so a letter whose console dies is a letter
// that failed: a line read clean at a rung is the proof that HCLK landed
// within the UART's tolerance of what the clock type claims. A number in
// hertz is the host's to take - letter d brackets a span of ticks between
// two lines - or a counter's on the clock output, which the QFN20 does
// not bring out.
//
// NOTHING HERE ENTERS STOP OR STANDBY.
//
// What is exercised, letter by letter:
//   a  the tree as the registers hold it after init(): the HSI on and
//      ready, the trim at the centre, the factory calibration, HPRE at 1,
//      the flash's two wait states, the clock output off, the gates the
//      console opened, and RCC_AHBPCENR's own reset-value gates (the USB
//      blocks' clocks are ON out of reset)
//   b  the HPRE ladder, 48 -> 24 -> 16 -> 12 -> 9.6 -> 8 -> 6 -> 3 MHz and
//      back up: at each rung the code, the wait states the rate needs, two
//      hundred ticks exact, a 500 us wait served, and a console line
//   c  the HSI trim: one step each way, the console still reading, the
//      centre restored
//   d  the rate against the HOST's clock: a bracket of ticks at 48 MHz and
//      at the 8 MHz of reset, for whoever times the lines
//   e  the clock output: its source field written and read back for every
//      source, and its pad (PB9) claimed where the package bonds it - on
//      the QFN20 init() answers false, which is the verdict there
//   f  the peripheral gates: every enable of the chapter opened, read back
//      and closed where it was, and a reset pulse leaving its gate alone
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32x035Platform<>;
using Serial = Uart<2, P>;
constexpr Serial serial;
using Led = Pin<'A', 0>;

using Boot = Clock<ClockSource::internal, 48'000'000>;
using SysClock = DynamicClock<Boot, Ticker, Serial>;
constexpr SysClock clock;

TestBench<Serial> bench;

/// RCC_AHBPCENR as main() found it, before any driver touched it.
uint32_t boot_hb_gates = 0;

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 300);   // the last byte's own time on the wire
}

bool wait_states_follow(uint32_t hz) {
    return (flash_ctl()->ACTLR & flash_latency_mask) == flash_latency_for(hz);
}

// ---------------------------------------------------------------------------
// a - the tree as the registers hold it
// ---------------------------------------------------------------------------
void ta_tree() {
    print(serial, "  CTLR=", hex(rcc()->CTLR), " CFGR0=", hex(rcc()->CFGR0),
          " AHBPCENR at boot=", hex(boot_hb_gates), " ACTLR=", hex(flash_ctl()->ACTLR), crlf);
    print(serial, "  HSI calibration=", hex(Rcc::hsi_calibration()), " trim=", Rcc::hsi_trim(),
          " HPRE code=", Rcc::hpre_code(), " HCLK=", Rcc::hclk_hz(), " Hz", crlf);
    bench.verdict("the HSI is on and ready", Rcc::hsi_on() && Rcc::hsi_ready());
    bench.verdict("its trim sits at the centre (16)", Rcc::hsi_trim() == 16u);
    bench.verdict("HPRE is 1 after Boot::init(), the ladder's top", Rcc::hpre_code() == 0u);
    bench.verdict("the registers' HCLK is the clock type's", Rcc::hclk_hz() == SysClock::hz());
    bench.verdict("the flash holds two wait states for 48 MHz (RM 20.3.1)",
                  wait_states_follow(48'000'000));
    bench.verdict("the clock output carries nothing", Rcc::mco() == McoSource::none);
    bench.verdict("the console opened USART2's gate and port A's",
                  Rcc::enabled(Bus::pb1, rcc_pb1_usart2) && Rcc::enabled(Bus::pb2, rcc_pb2_gpioa));
    bench.verdict("RCC_AHBPCENR came out of reset as 0x00021004: the SRAM in Sleep, "
                  "the USBFS and the USB PD clocked",
                  boot_hb_gates == 0x00021004UL);
}

// ---------------------------------------------------------------------------
// b - the HPRE ladder
// ---------------------------------------------------------------------------
constexpr uint32_t ladder[] = {48'000'000, 24'000'000, 16'000'000, 12'000'000,
                               9'600'000,  8'000'000,  6'000'000,  3'000'000};
constexpr uint8_t rungs = sizeof(ladder) / sizeof(ladder[0]);

bool rung(uint32_t hz, const char* way) {
    console_drain();
    const bool set = SysClock::set(hz);
    const bool code = Rcc::hpre_code() == hpre_for(hsi_hz, hz);
    const bool waits = wait_states_follow(hz);
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < 200u) {
    }
    const bool ticks = Ticker::ticks() - t0 == 200u;
    const bool delay = delay_us(clock, 500);
    print(serial, "  ", way, " ", hz, " Hz: HPRE ", Rcc::hpre_code(), ", ",
          flash_ctl()->ACTLR & flash_latency_mask, " wait state(s), BRR ", Usart<2>::brr(),
          " -> ", Serial::actual_baud(hz), " baud", crlf);
    return set && code && waits && ticks && delay;
}

void tb_ladder() {
    bool down = true;
    for (uint8_t i = 0; i < rungs; ++i) {
        down = rung(ladder[i], "down") && down;
    }
    bench.verdict("down the ladder: every rung set, its code and its wait states, 200 "
                  "ticks exact and a 500 us wait served - and this line read clean",
                  down);
    bool up = true;
    for (uint8_t i = rungs; i > 0; --i) {
        up = rung(ladder[i - 1u], "up") && up;
    }
    bench.verdict("and back up to 48 MHz the same way", up);
    bench.verdict("a rate no divider reaches is refused, nothing changed",
                  !SysClock::set(7'000'000) && SysClock::hz() == 48'000'000);
}

// ---------------------------------------------------------------------------
// c - the HSI trim
// ---------------------------------------------------------------------------
void tc_trim() {
    const uint8_t centre = Rcc::hsi_trim();
    console_drain();
    Rcc::hsi_trim(static_cast<uint8_t>(centre + 1u));
    const uint8_t up = Rcc::hsi_trim();
    print(serial, "  trim ", centre, " -> ", up, ": this line left at the trimmed rate", crlf);
    console_drain();
    Rcc::hsi_trim(static_cast<uint8_t>(centre - 1u));
    const uint8_t down = Rcc::hsi_trim();
    print(serial, "  trim -> ", down, ": and this one too", crlf);
    console_drain();
    Rcc::hsi_trim(centre);
    bench.verdict("one step up takes", up == centre + 1u);
    bench.verdict("one step down takes", down == centre - 1u);
    bench.verdict("and the centre is back", Rcc::hsi_trim() == centre);
}

// ---------------------------------------------------------------------------
// d - the rate against the host's clock
// ---------------------------------------------------------------------------
void bracket(uint32_t span) {
    print(serial, "  bracket ", span, " ticks at ", SysClock::hz(), " Hz", crlf);
    console_drain();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < span) {
    }
    print(serial, "  bracket done", crlf);
    console_drain();
    const uint32_t t1 = Ticker::ticks();
    while (Ticker::ticks() - t1 < 5u) {
    }
}

void td_bracket() {
    bracket(3000);
    console_drain();
    const bool slow = SysClock::set(8'000'000);
    bracket(3000);
    console_drain();
    const bool back = SysClock::set(48'000'000);
    bench.verdict("the brackets ran at 48 MHz and at the reset rate, and the tree is back",
                  slow && back && SysClock::hz() == 48'000'000);
}

// ---------------------------------------------------------------------------
// e - the clock output
// ---------------------------------------------------------------------------
void te_mco() {
    Rcc::mco(McoSource::sysclk);
    const bool sysclk = Rcc::mco() == McoSource::sysclk;
    Rcc::mco(McoSource::hsi);
    const bool hsi = Rcc::mco() == McoSource::hsi;
    Rcc::mco(McoSource::none);
    const bool none = Rcc::mco() == McoSource::none;
    bench.verdict("the source field takes SYSCLK, the HSI and nothing, and reads each back",
                  sysclk && hsi && none);
    const bool claimed = Mco::init(McoSource::hsi);
    print(serial, "  this package ", Mco::has_pad ? "bonds" : "does not bond",
          " PB9; Mco::init() answered ", claimed, crlf);
    if constexpr (Mco::has_pad) {
        bench.verdict("the pad is claimed and the HSI is on it", claimed && Mco::source() == McoSource::hsi);
    } else {
        bench.verdict("with no PB9 on this package, init() answers false", !claimed);
    }
    Mco::off();
    bench.verdict("and off() leaves the output carrying nothing", Mco::source() == McoSource::none);
}

// ---------------------------------------------------------------------------
// f - the peripheral gates
// ---------------------------------------------------------------------------
struct Gate {
    Bus bus;
    uint32_t mask;
    const char* name;
};

constexpr Gate gates[] = {
    {Bus::hb, rcc_hb_dma1, "DMA1"},       {Bus::pb2, rcc_pb2_afio, "AFIO"},
    {Bus::pb2, rcc_pb2_gpiob, "IOPB"},    {Bus::pb2, rcc_pb2_gpioc, "IOPC"},
    {Bus::pb2, rcc_pb2_adc1, "ADC1"},     {Bus::pb2, rcc_pb2_tim1, "TIM1"},
    {Bus::pb2, rcc_pb2_spi1, "SPI1"},     {Bus::pb2, rcc_pb2_usart1, "USART1"},
    {Bus::pb1, rcc_pb1_tim2, "TIM2"},     {Bus::pb1, rcc_pb1_tim3, "TIM3"},
    {Bus::pb1, rcc_pb1_wwdg, "WWDG"},     {Bus::pb1, rcc_pb1_usart3, "USART3"},
    {Bus::pb1, rcc_pb1_usart4, "USART4"}, {Bus::pb1, rcc_pb1_i2c1, "I2C1"},
    {Bus::pb1, rcc_pb1_pwr, "PWR"},
};

void tf_gates() {
    bool all = true;
    for (const Gate& g : gates) {
        const bool was = Rcc::enabled(g.bus, g.mask);
        Rcc::enable(g.bus, g.mask);
        const bool on = Rcc::enabled(g.bus, g.mask);
        Rcc::disable(g.bus, g.mask);
        const bool off = !Rcc::enabled(g.bus, g.mask);
        if (was) {
            Rcc::enable(g.bus, g.mask);
        }
        print(serial, "  ", g.name, ": opens ", on, ", closes ", off, crlf);
        all = all && on && off;
    }
    bench.verdict("every gate of the chapter opens and closes, read back", all);
    // The pulse's subject is I2C1, a block nothing here uses. NEVER the
    // power controller's: a pulse on PWRRST left this part unreachable by
    // its debug port until its supply was cycled (clock.md), and the
    // driver refuses it now - which the third verdict checks.
    Rcc::enable(Bus::pb1, rcc_pb1_i2c1);
    const bool pulsed = Rcc::reset(Bus::pb1, rcc_pb1_i2c1);
    bench.verdict("a reset pulse leaves its gate where it was",
                  pulsed && Rcc::enabled(Bus::pb1, rcc_pb1_i2c1));
    Rcc::disable(Bus::pb1, rcc_pb1_i2c1);
    bench.verdict("and the power controller's reset line is REFUSED, nothing written",
                  !Rcc::reset(Bus::pb1, rcc_pb1_pwr));
}

void banner() {
    print(serial, crlf, "test_x035_clock on ", device::part_name, " - the clock tree", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Serial::isr(); }

int main() {
    boot_hb_gates = brio::rcc()->AHBPCENR;
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output(true);
    brio::enable_interrupts();

    bench.letter('a', "the tree as the registers hold it", ta_tree);
    bench.letter('b', "the HPRE ladder, down and up, the users rebased", tb_ladder);
    bench.letter('c', "the HSI trim", tc_trim);
    bench.letter('d', "the rate against the host's clock", td_bracket);
    bench.letter('e', "the clock output", te_mco);
    bench.letter('f', "the peripheral gates", tf_gates);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=",
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
