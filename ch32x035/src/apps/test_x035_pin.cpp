// test_x035_pin - the reference bench suite for the GPIO and AFIO of the
// CH32X035 (RM ch. 8): the five nibbles of this series in each of the
// three configuration registers, CFGHR configured through its RAM copy and
// compared with what the register reads, the levels and the set/reset
// halves of a 24-pin port, the pulls with the pull-down only some pads
// have, the remap field and the EXTI multiplexer read back, and the lock.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// NOTHING TO WIRE for `z`. The pads it drives are free on WCH's QFN20
// evaluation board - PA5, PA6 and PA7 and PB11 and PB12 go to its pin
// headers and nowhere else - and the one it only reads, PC14, is the
// board's CC1 line, which the board ties to ground through 5.1 kOhm at the
// USB-C connector: it is configured as an input and never driven, so a
// cable on that connector sees nothing change. The USB pads PC16/PC17 and
// the debug port's PC18/PC19 are not touched. Letter `w` wants a jumper
// PA4-PA5 and DETECTS it first; without it the letter says so and judges
// nothing. The LED on PA0 marks a keystroke and is judged on nothing.
//
// What is exercised, letter by letter:
//   a  the nibbles: every mode of this series into PA6 (CFGLR) and PB11
//      (CFGHR), and the inputs into PC14 (CFGXR), each read back - with
//      CFGHR's RAM copy compared against what the register itself reads,
//      the question WCH's own library does not ask the silicon
//   b  the levels: a driven pad read back through INDR, the set/reset
//      halves and the toggle on one pin and on three at once, the runtime
//      PinRef, the whole-port write
//   c  the pulls: up and down on a pad that has both, up on pads that have
//      no pull-down and the refusal of the down there
//   d  the high byte: PC14's output data bit set and cleared through BSXR
//      and BCR while it is a pulled input, and the level the board's CC
//      pull-down leaves it at (reported)
//   e  AFIO: the remap field of a peripheral no program uses here written
//      and read back, the debug port's field as it stands, and the EXTI
//      multiplexer's three codes written and read back
//   w  (a jumper PA4-PA5, detected first) the levels across a wire both
//      ways, and a pull against a driver
//   l  (by name only) THE LOCK: PB12's configuration frozen until the next
//      reset, a reconfiguration left without effect, the copy unchanged -
//      not in `z`, because nothing but a reset undoes it
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32x035/afio.hpp"
#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, P>;
constexpr Serial serial;
using Led = Pin<'A', 0>;

using Low = Pin<'A', 6>;      // CFGLR
using High = Pin<'B', 11>;    // CFGHR, through its copy
using Pulled = Pin<'A', 7>;   // a pad with both pulls
using NoDown = Pin<'B', 3>;   // a pad with the pull-up alone
using Cc1 = Pin<'C', 14>;     // CFGXR; the board's CC1, read only
using WireOut = Pin<'A', 4>;
using WireIn = Pin<'A', 5>;
using Locked = Pin<'B', 12>;

TestBench<Serial> bench;

void settle() { (void)delay_us(clock, 20); }

// ---------------------------------------------------------------------------
// a - the nibbles
// ---------------------------------------------------------------------------
void ta_nibbles() {
    bool low_ok = true;
    const uint32_t modes[] = {pin_nibble_analog, pin_nibble_floating, pin_nibble_pulled,
                              pin_nibble_output, pin_nibble_alternate};
    for (const uint32_t n : modes) {
        Port<'A'>::configure(6, n);
        low_ok = Low::nibble() == n && low_ok;
    }
    Low::release();
    bench.verdict("every nibble of this series lands in CFGLR and reads back", low_ok);

    bool high_ok = true;
    bool register_agrees = true;
    for (const uint32_t n : modes) {
        Port<'B'>::configure(11, n);
        high_ok = High::nibble() == n && high_ok;
        const uint32_t reg = Port<'B'>::cfghr_register();
        const uint32_t copy = Port<'B'>::cfghr_copy();
        print(serial, "  PB11 <- ", n, ": copy ", hex(copy), ", CFGHR reads ", hex(reg), crlf);
        register_agrees = reg == copy && register_agrees;
    }
    High::release();
    bench.verdict("every nibble lands in CFGHR's copy and is stored whole", high_ok);
    print(serial, "  chip id word ", hex(chip_id_word()), ": bits 7:4 = ",
          (chip_id_word() >> 4) & 0xFu, crlf);
    bench.verdict("CFGHR reads back what the copy says (WCH's library does not read it "
                  "where the chip id's bits 7:4 are zero - this is the silicon's answer)",
                  register_agrees);

    bool x_ok = true;
    for (const uint32_t n : {pin_nibble_analog, pin_nibble_floating, pin_nibble_pulled}) {
        Port<'C'>::configure(14, n);
        x_ok = Cc1::nibble() == n && x_ok;
    }
    Cc1::release();
    bench.verdict("the three inputs land in CFGXR (PC14, never driven) and read back", x_ok);
    bench.verdict("released pads are floating inputs, the reset state",
                  Low::nibble() == pin_nibble_floating && High::nibble() == pin_nibble_floating &&
                      Cc1::nibble() == pin_nibble_floating);
}

// ---------------------------------------------------------------------------
// b - the levels
// ---------------------------------------------------------------------------
void tb_levels() {
    Low::output(true);
    settle();
    const bool high = Low::read() && Low::read_out();
    Low::clear();
    settle();
    const bool low = !Low::read() && !Low::read_out();
    Low::toggle();
    settle();
    const bool toggled = Low::read();
    bench.verdict("a driven pad reads its level back through INDR, set and cleared", high && low);
    bench.verdict("the toggle's one BSHR store flips it", toggled);

    const PinRef r = Low::ref();
    r.clear();
    settle();
    const bool ref_low = !r.read();
    r.toggle();
    settle();
    const bool ref_high = r.read();
    bench.verdict("the runtime PinRef clears, toggles and reads the same pad", ref_low && ref_high);

    constexpr uint32_t three = (1UL << 5) | (1UL << 6) | (1UL << 7);
    Port<'A'>::configure(5, pin_nibble_output);
    Port<'A'>::configure(7, pin_nibble_output);
    Port<'A'>::out_set(three);
    settle();
    const bool all_set = (Port<'A'>::in() & three) == three;
    Port<'A'>::out_toggle(three);
    settle();
    const bool all_clear = (Port<'A'>::in() & three) == 0u;
    Port<'A'>::out_write((Port<'A'>::out() & ~three) | (1UL << 6));
    settle();
    const bool one = (Port<'A'>::in() & three) == (1UL << 6);
    Port<'A'>::out_clear(three);
    bench.verdict("three pins set in one BSHR store and toggled in another", all_set && all_clear);
    bench.verdict("the whole-port write drives what it names", one);
    Port<'A'>::configure_pins(three, pin_nibble_floating);
}

// ---------------------------------------------------------------------------
// c - the pulls
// ---------------------------------------------------------------------------
void tc_pulls() {
    const bool up_set = Pulled::input(PinPull::up);
    settle();
    const bool up = Pulled::read();
    const bool down_set = Pulled::input(PinPull::down);
    settle();
    const bool down = !Pulled::read();
    Pulled::release();
    print(serial, "  PA7 pulled up reads ", up, ", pulled down reads ", !down, crlf);
    bench.verdict("PA7 takes both pulls and each one moves the free pad", up_set && down_set && up && down);

    const bool nodown_up = NoDown::input(PinPull::up);
    settle();
    const bool pb3_up = NoDown::read();
    const bool nodown_down = NoDown::input(PinPull::down);
    NoDown::release();
    bench.verdict("PB3 takes the pull-up and reads high", nodown_up && pb3_up);
    bench.verdict("and refuses a pull-down it has not got, writing nothing",
                  !nodown_down && NoDown::nibble() == pin_nibble_pulled);
    NoDown::release();
}

// ---------------------------------------------------------------------------
// d - the high byte
// ---------------------------------------------------------------------------
void td_high_byte() {
    (void)Cc1::input(PinPull::up);   // OUTDR bit set through BSXR, then the pull
    const bool bit_set = (Port<'C'>::out() & Cc1::mask) != 0u;
    settle();
    const bool level_up = Cc1::read();
    Cc1::clear();                      // BCR's bit 14
    const bool bit_clear = (Port<'C'>::out() & Cc1::mask) == 0u;
    Cc1::set();                        // BSXR's bit 6 is PC14's set bit
    const bool bit_again = (Port<'C'>::out() & Cc1::mask) != 0u;
    Cc1::release();
    print(serial, "  PC14 pulled up reads ", level_up,
          " (the board's CC1 line has 5.1 kOhm to ground at the connector)", crlf);
    bench.verdict("PC14's output data bit is set by BSXR and cleared by BCR", bit_set && bit_clear && bit_again);
}

// ---------------------------------------------------------------------------
// e - AFIO
// ---------------------------------------------------------------------------
void te_afio() {
    // Code 1 puts TIM3's channels on PB4/PB5, which this package does not
    // bond; the timer is not clocked either, so the write moves nothing.
    const uint8_t was = Afio::remap_code(Remap::tim3);
    const bool wrote = Afio::remap(Remap::tim3, 1);
    const uint8_t now = Afio::remap_code(Remap::tim3);
    (void)Afio::remap(Remap::tim3, was);
    bench.verdict("TIM3's remap field takes a code and reads it back",
                  wrote && now == 1u && Afio::remap_code(Remap::tim3) == was);
    print(serial, "  SW_CFG = ", Afio::debug_config(), ", PCFR1 = ", hex(afio()->PCFR1), crlf);
    bench.verdict("the debug port is the probe's (SW_CFG 0xx)", Afio::debug_port_enabled());

    const char a0 = Afio::exti_source(5);
    const bool to_b = Afio::exti_source(5, 'B');
    const char b = Afio::exti_source(5);
    const bool to_c = Afio::exti_source(19, 'C');
    const char c = Afio::exti_source(19);
    (void)Afio::exti_source(5, 'A');
    (void)Afio::exti_source(19, 'A');
    print(serial, "  EXTICR1=", hex(afio()->EXTICR[0]), " EXTICR2=", hex(afio()->EXTICR[1]), crlf);
    bench.verdict("line 5 starts on port A (code 00)", a0 == 'A');
    bench.verdict("and takes port B (code 10) and port C on line 19 (code 11, EXTICR2)",
                  to_b && b == 'B' && to_c && c == 'C');
    bench.verdict("and both go back to port A", Afio::exti_source(5) == 'A' && Afio::exti_source(19) == 'A');
}

// ---------------------------------------------------------------------------
// w - across a wire, PA4-PA5
// ---------------------------------------------------------------------------
bool wire_present() {
    (void)WireIn::input(PinPull::down);
    WireOut::output(true);
    settle();
    const bool sees_high = WireIn::read();
    (void)WireIn::input(PinPull::up);
    WireOut::clear();
    settle();
    const bool sees_low = !WireIn::read();
    WireOut::release();
    WireIn::release();
    return sees_high && sees_low;
}

void tw_wire() {
    if (!wire_present()) {
        print(serial, "  no wire: PA4-PA5 is not strapped, nothing judged", crlf);
        return;
    }
    (void)WireIn::input();
    bool follows = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        const bool level = (i & 1u) != 0u;
        WireOut::output(level);
        settle();
        follows = WireIn::read() == level && follows;
    }
    bench.verdict("PA5 follows PA4 through the wire, eight edges", follows);
    (void)WireIn::input(PinPull::up);
    WireOut::output(false);
    settle();
    bench.verdict("a driver beats the other pad's pull", !WireIn::read());
    WireOut::release();
    WireIn::release();
}

// ---------------------------------------------------------------------------
// l - the lock
// ---------------------------------------------------------------------------
void tl_lock() {
    Locked::output(false);
    const uint32_t before = Locked::nibble();
    const bool took = Locked::lock();
    Port<'B'>::configure(12, pin_nibble_pulled);
    const uint32_t after = Locked::nibble();
    const uint32_t reg = (Port<'B'>::cfghr_register() >> 16) & 0xFu;
    print(serial, "  LCKR=", hex(Port<'B'>::regs().LCKR),
          " PB12 nibble before ", before, ", after a reconfiguration ", after,
          ", CFGHR's field reads ", reg, crlf);
    bench.verdict("the key sequence took (LCKK set, PB12's bit set)", took && Locked::locked());
    bench.verdict("a reconfiguration of the locked pin changed nothing, the copy included",
                  after == before);
}

void banner() {
    print(serial, crlf, "test_x035_pin on ", device::part_name, " - GPIO and AFIO", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output(true);
    brio::enable_interrupts();

    bench.letter('a', "the nibbles in the three configuration registers", ta_nibbles);
    bench.letter('b', "the levels, the set/reset halves, the toggle", tb_levels);
    bench.letter('c', "the pulls", tc_pulls);
    bench.letter('d', "the high byte: PC14's data bit through BSXR", td_high_byte);
    bench.letter('e', "AFIO: a remap field, the debug port, the EXTI multiplexer", te_afio);
    bench.letter('w', "across a jumper PA4-PA5, detected first", tw_wire);
    bench.letter('l', "the configuration lock, until the next reset", tl_lock, false);

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
