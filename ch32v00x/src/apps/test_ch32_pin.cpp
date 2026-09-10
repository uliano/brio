// test_ch32_pin - the reference bench suite for the CH32V00x's GPIO
// and EXTI: ch32v00x/pin.hpp over RM ch. 7 and ch32v00x/exti.hpp over
// 6.4, the pulls and the software trigger with NO WIRE, the levels,
// the open drain and the edges on ONE JUMPER - the same PD2 to PD4
// the timer suite uses - and util/input_scanner.hpp's InputScanner
// over a pin.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// THE JUMPER, for letters c, d and f:
//
//   PD2  <->  PD4
//
// main() probes it and the three letters decline with the reason when
// it is missing. PC0 is the LED; PD5/PD6 the console's, never claimed.
//
// What is exercised, letter by letter:
//   a  the ports, WIRELESS: the clock gates, the reset nibbles, every
//      nibble pin_nibble() spells read back, the BSHR/BCR atomics, the
//      toggle
//   b  THE PULLS, wireless: an input pulled up reads high and pulled
//      down low on four pads, the pull's direction being OUTDR's bit
//   c  THE LEVELS on the jumper: an output drives the other pad both
//      ways, an open-drain output against the far pad's pull-up, an
//      analog pad reading zero on INDR
//   d  THE EDGES on the jumper: PD4's line 4 counting rising, falling
//      and both edges of PD2's toggles, the flag as a poll and the
//      interrupt on exti7_0
//   e  THE SOFTWARE TRIGGER and the EVENT MODE, wireless: SWIEVR raising
//      a line's flag and its interrupt, a line in event mode ending a
//      WFE with no handler and no flag
//   f  THE SCANNER on the jumper: util/input_scanner.hpp's InputScanner
//      over PD4 with PD2 driving it, InputEdge events on the flips and
//      none on a held level
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/exti.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "util/input_scanner.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'C', 0>;

using Driver = Pin<'D', 2>;
using Sensor = Pin<'D', 4>;
using Line = ExtInt<'D', 4>;

TestBench<Serial> bench;

volatile uint32_t line_interrupts = 0;
volatile uint32_t line_flags_seen = 0;
bool jumper_present = false;

bool probe_jumper() {
    Sensor::input();
    Driver::output(true);
    (void)delay_us(clock, 5);
    const bool high = Sensor::read();
    Driver::clear();
    (void)delay_us(clock, 5);
    const bool low = !Sensor::read();
    Driver::release();
    Sensor::release();
    return high && low;
}

bool need_jumper() {
    if (jumper_present) {
        return true;
    }
    jumper_present = probe_jumper();
    if (!jumper_present) {
        print(serial, "  SKIPPED, no verdict claimed: no jumper between PD2 and PD4", crlf);
    }
    return jumper_present;
}

void all_off() {
    Pfic::disable(Irq::exti7_0);
    Line::interrupt(false);
    Line::event(false);
    Line::clear();
    Driver::release();
    Sensor::release();
    line_interrupts = 0;
    line_flags_seen = 0;
}

// ===========================================================================
// a - the ports, wireless
// ===========================================================================

void ta_ports() {
    all_off();
    // A port's clock gate: PB2PCENR's bit, opened by the first configuring verb.
    rcc()->PB2PCENR &= ~rcc_pb2_gpiob;
    const bool closed = (rcc()->PB2PCENR & rcc_pb2_gpiob) == 0u;
    Pin<'B', 0>::input();
    const bool opened = (rcc()->PB2PCENR & rcc_pb2_gpiob) != 0u;
    bench.verdict("a configuring verb opens the port's clock gate (GPIOB, closed, then PB0 configured)",
                  closed && opened);
    // The reset nibble is 0x4 (floating input) on every pin of a reset port.
    rcc()->PB2PRSTR |= rcc_pb2_gpiob;
    rcc()->PB2PRSTR &= ~rcc_pb2_gpiob;
    print(serial, "  GPIOB after its reset pulse: CFGLR=", hex(Port<'B'>::regs().CFGLR), " OUTDR=",
          hex(Port<'B'>::regs().OUTDR), " (port B bonds PB0..PB6: seven nibbles)", crlf);
    bench.verdict("a reset port holds 0x4 in every nibble it has (floating input, RM 7.3.1.1) - seven on "
                  "port B, whose eighth reads zero",
                  (Port<'B'>::regs().CFGLR & 0x0FFFFFFFu) == 0x04444444u);
    // Every nibble pin_nibble() spells, written to PD2 and read back.
    struct Case {
        const char* name;
        PinMode mode;
        PinDrive drive;
        uint32_t nibble;
    };
    const Case cases[] = {{"analog", PinMode::analog, PinDrive::push_pull, 0x0},
                          {"input", PinMode::input, PinDrive::push_pull, 0x4},
                          {"output push-pull", PinMode::output, PinDrive::push_pull, 0x1},
                          {"output open-drain", PinMode::output, PinDrive::open_drain, 0x5},
                          {"alternate push-pull", PinMode::alternate, PinDrive::push_pull, 0x9},
                          {"alternate open-drain", PinMode::alternate, PinDrive::open_drain, 0xD}};
    uint8_t right = 0;
    for (const Case& c : cases) {
        Port<'D'>::configure(2, pin_nibble(c.mode, c.drive));
        const uint32_t read = (Port<'D'>::regs().CFGLR >> 8) & 0xFu;
        if (read == c.nibble && pin_nibble(c.mode, c.drive) == c.nibble) {
            ++right;
        }
    }
    Driver::release();
    bench.verdict("the six nibbles land as spelled (MODE one bit, CNF two) and read back", right == 6u);
    // BSHR sets from its low half and clears from its high half; BCR
    // clears; the toggle is one BSHR store.
    Driver::output(false);
    Port<'D'>::out_set(Driver::mask);
    const bool set = Driver::read_out();
    Port<'D'>::regs().BSHR = Driver::mask << 16;
    const bool cleared_high = !Driver::read_out();
    Driver::set();
    Port<'D'>::out_clear(Driver::mask);
    const bool cleared_bcr = !Driver::read_out();
    Driver::toggle();
    const bool toggled = Driver::read_out();
    Driver::toggle();
    const bool back = !Driver::read_out();
    Driver::release();
    bench.verdict("BSHR sets from its low half and clears from its high half, BCR clears, toggle() flips",
                  set && cleared_high && cleared_bcr && toggled && back);
    print(serial, "  jumper PD2-PD4: ", jumper_present ? "present" : "ABSENT", crlf);
}

// ===========================================================================
// b - the pulls, wireless
// ===========================================================================

void tb_pulls() {
    all_off();
    struct PadCase {
        const char* name;
        bool (*up)();
        bool (*down)();
    };
    const PadCase pads[] = {
        {"PD2", [] { Pin<'D', 2>::input(PinPull::up); (void)delay_us(clock, 20); const bool r = Pin<'D', 2>::read(); Pin<'D', 2>::release(); return r; },
                [] { Pin<'D', 2>::input(PinPull::down); (void)delay_us(clock, 20); const bool r = Pin<'D', 2>::read(); Pin<'D', 2>::release(); return r; }},
        {"PD4", [] { Pin<'D', 4>::input(PinPull::up); (void)delay_us(clock, 20); const bool r = Pin<'D', 4>::read(); Pin<'D', 4>::release(); return r; },
                [] { Pin<'D', 4>::input(PinPull::down); (void)delay_us(clock, 20); const bool r = Pin<'D', 4>::read(); Pin<'D', 4>::release(); return r; }},
        {"PC4", [] { Pin<'C', 4>::input(PinPull::up); (void)delay_us(clock, 20); const bool r = Pin<'C', 4>::read(); Pin<'C', 4>::release(); return r; },
                [] { Pin<'C', 4>::input(PinPull::down); (void)delay_us(clock, 20); const bool r = Pin<'C', 4>::read(); Pin<'C', 4>::release(); return r; }},
        {"PC3", [] { Pin<'C', 3>::input(PinPull::up); (void)delay_us(clock, 20); const bool r = Pin<'C', 3>::read(); Pin<'C', 3>::release(); return r; },
                [] { Pin<'C', 3>::input(PinPull::down); (void)delay_us(clock, 20); const bool r = Pin<'C', 3>::read(); Pin<'C', 3>::release(); return r; }},
    };
    uint8_t right = 0;
    print(serial, "  pulled up / down:");
    for (const PadCase& p : pads) {
        const bool u = p.up();
        const bool d = p.down();
        print(serial, " ", p.name, "=", u, "/", d);
        if (u && !d) {
            ++right;
        }
    }
    print(serial, crlf);
    bench.verdict("four pads read high through their pull-up and low through their pull-down", right == 4u);
    // The pull IS OUTDR's bit: with CNF 10 the level follows what OUTDR holds.
    Driver::input(PinPull::up);
    const bool odr_up = Driver::read_out();
    Driver::input(PinPull::down);
    const bool odr_down = !Driver::read_out();
    const uint32_t nibble = (Port<'D'>::regs().CFGLR >> 8) & 0xFu;
    Driver::release();
    bench.verdict("the pull's direction is OUTDR's bit under CNF 10 (nibble 0x8)",
                  odr_up && odr_down && nibble == 0x8u);
}

// ===========================================================================
// c - the levels on the jumper
// ===========================================================================

void tc_levels() {
    if (!need_jumper()) {
        return;
    }
    all_off();
    Sensor::input();
    Driver::output(true);
    (void)delay_us(clock, 5);
    const bool high = Sensor::read();
    Driver::clear();
    (void)delay_us(clock, 5);
    const bool low = !Sensor::read();
    bench.verdict("a push-pull output drives the far pad high and low", high && low);
    // Open drain: the output pulls low, and RELEASES against the far
    // pad's pull-up.
    Sensor::input(PinPull::up);
    Driver::output(true, PinDrive::open_drain);
    (void)delay_us(clock, 20);
    const bool released = Sensor::read();
    Driver::clear();
    (void)delay_us(clock, 20);
    const bool pulled_low = !Sensor::read();
    Driver::set();
    (void)delay_us(clock, 20);
    const bool released_again = Sensor::read();
    bench.verdict("an open-drain output pulls the far pad low and lets its pull-up have it otherwise",
                  released && pulled_low && released_again);
    // A pad in analog mode: its input buffer off, INDR reads zero
    // whatever the level.
    Driver::output(true);
    Sensor::analog();
    (void)delay_us(clock, 5);
    const bool analog_reads_zero = !Sensor::read();
    Sensor::input();
    (void)delay_us(clock, 5);
    const bool input_reads_high = Sensor::read();
    print(serial, "  the far pad driven high: as analog INDR=", !analog_reads_zero, ", as input INDR=", input_reads_high,
          crlf);
    bench.verdict("an analog pad's input buffer is off (INDR zero with the pad high), an input's is on",
                  analog_reads_zero && input_reads_high);
    all_off();
}

// ===========================================================================
// d - the edges on the jumper
// ===========================================================================

uint32_t edges_of(bool rising, bool falling, uint8_t toggles) {
    Line::interrupt(false);
    Line::init(rising, falling);
    line_interrupts = 0;
    Line::clear();
    Line::interrupt(true);
    Pfic::enable(Irq::exti7_0);
    for (uint8_t i = 0; i < toggles; ++i) {
        Driver::toggle();
        (void)delay_us(clock, 50);
    }
    (void)delay_us(clock, 50);
    Pfic::disable(Irq::exti7_0);
    Line::interrupt(false);
    return line_interrupts;
}

void td_edges() {
    if (!need_jumper()) {
        return;
    }
    all_off();
    Sensor::input();
    Driver::output(false);
    // Ten toggles from low: five rising and five falling edges.
    const uint32_t r = edges_of(true, false, 10);
    const uint32_t f = edges_of(false, true, 10);
    const uint32_t b = edges_of(true, true, 10);
    print(serial, "  ten toggles of PD2 on PD4's line 4: rising ", r, ", falling ", f, ", both ", b, " interrupts",
          crlf);
    bench.verdict("line 4 counts five rising, five falling and ten of both edges, one interrupt each",
                  r == 5u && f == 5u && b == 10u);
    // The flag as a poll, no interrupt: it stands until cleared.
    Line::init(true, false);
    Line::clear();
    Driver::clear();
    Driver::set();
    (void)delay_us(clock, 5);
    const bool flag_up = Line::flag();
    Line::clear();
    const bool flag_down = !Line::flag();
    bench.verdict("the flag rises on the edge with the interrupt off and is cleared by writing one",
                  flag_up && flag_down);
    // The other port on the same line number: PC4 selected instead of
    // PD4, PD2's toggles reach nothing.
    Exti::port(4, 2);   // port C
    Line::clear();
    Line::interrupt(true);
    line_interrupts = 0;
    Pfic::enable(Irq::exti7_0);
    for (uint8_t i = 0; i < 6u; ++i) {
        Driver::toggle();
        (void)delay_us(clock, 50);
    }
    Pfic::disable(Irq::exti7_0);
    Line::interrupt(false);
    print(serial, "  line 4 routed to PC4 instead: ", line_interrupts, " interrupts from PD2's toggles", crlf);
    bench.verdict("a line is ONE pin number of ONE port (AFIO_EXTICR): routed to PC4 it hears nothing of PD4",
                  line_interrupts == 0u);
    all_off();
}

// ===========================================================================
// e - the software trigger and the event mode, wireless
// ===========================================================================

void te_soft_and_event() {
    all_off();
    // The software trigger on line 4, the pad untouched. A line enabled
    // NOWHERE (neither INTENR nor EVENR) raises no flag from SWIEVR
    // (measured); enabled in INTENR with the PFIC line closed, the flag
    // stands to be polled; with the PFIC line open, the interrupt.
    Line::init(true, false);
    Line::interrupt(false);
    Line::clear();
    Line::soft();
    const bool flag_unarmed = Line::flag();
    Line::clear();
    Line::interrupt(true);
    Line::soft();
    const bool flag_armed = Line::flag();
    Line::clear();
    Pfic::clear_pending(Irq::exti7_0);
    line_interrupts = 0;
    Pfic::enable(Irq::exti7_0);
    Line::soft();
    (void)delay_us(clock, 20);
    Line::soft();
    (void)delay_us(clock, 20);
    Pfic::disable(Irq::exti7_0);
    Line::interrupt(false);
    print(serial, "  SWIEVR on line 4: flag with the line enabled nowhere ", flag_unarmed, ", enabled in INTENR ",
          flag_armed, "; then ", line_interrupts, " interrupts from two triggers", crlf);
    bench.verdict("the software trigger raises the flag of a line enabled in INTENR - and nothing on a line "
                  "enabled nowhere - and, with the PFIC line open, one interrupt per trigger",
                  !flag_unarmed && flag_armed && line_interrupts == 2u);

    // EVENT MODE: the line in EVENR, not INTENR - a software trigger
    // ends a WFE with no handler run and no flag raised (6.4.2). The
    // ticker is paused so nothing else ends the wait.
    Line::interrupt(false);
    Line::event(true);
    Line::clear();
    Ticker::pause();
    Pfic::clear_pending(Irq::systick);
    // A trigger BEFORE the wait: the event is latched and the WFE
    // returns at once; the flag stays down.
    Line::soft();
    const uint32_t t0 = stk()->CNT;
    pfic_sctlr() = (pfic_sctlr() | sctlr_wfitowfe | sctlr_sevonpend) & ~sctlr_setevent;
    asm volatile("wfi");
    const uint32_t t1 = stk()->CNT;
    const bool flag_after = Line::flag();
    Ticker::resume();
    Line::event(false);
    print(serial, "  a line in event mode, triggered before a WFE: the WFE returned after ",
          static_cast<uint32_t>(t1 - t0) & 0xFFFFFFu, " cycles, the flag ", flag_after ? "UP" : "down", crlf);
    bench.verdict("a line in EVENT mode ends a WFE with no handler and no flag (6.4.2)",
                  !flag_after && (static_cast<uint32_t>(t1 - t0) & 0xFFFFFFu) < 1000u);
    all_off();
}

// ===========================================================================
// f - the scanner on the jumper
// ===========================================================================

namespace ks {

struct SensorInput {
    static bool read() { return Sensor::read(); }
};

class Probe {
public:
    using Event = std::variant<InputEdge>;
    static inline EventQueue<Event, 8, P> queue;
    static inline uint8_t edges = 0;
    static inline bool last_active = false;

    static void init() { edges = 0; }
    static void dispatch(const Event& e) {
        brio::match(e, [](const InputEdge& edge) {
            ++edges;
            last_active = edge.active;
        });
    }
};

using Scanner = InputScanner<P, Subscribers<Probe>, ScanConfig{.stable_samples = 3}, SensorInput>;
using Loop = Kernel<P, Probe, Scanner>;

void pump_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        TimeEvents<P>::process();
        while (Loop::step()) {
            TimeEvents<P>::process();
        }
    }
}

}  // namespace ks

void tf_scanner() {
    if (!need_jumper()) {
        return;
    }
    all_off();
    Sensor::input();
    Driver::output(false);
    ks::Loop::init_all();
    ks::Scanner::start_every(2);   // a sample every 2 ms, three to believe a level
    ks::pump_ms(20);
    const uint8_t at_start = ks::Probe::edges;
    Driver::set();
    ks::pump_ms(20);
    const uint8_t after_rise = ks::Probe::edges;
    const bool rise_active = ks::Probe::last_active;
    ks::pump_ms(40);
    const uint8_t held = ks::Probe::edges;
    Driver::clear();
    ks::pump_ms(20);
    const uint8_t after_fall = ks::Probe::edges;
    const bool fall_active = ks::Probe::last_active;
    // A glitch shorter than the debounce: one sample wide.
    Driver::set();
    (void)delay_us(clock, 500);
    Driver::clear();
    ks::pump_ms(20);
    const uint8_t after_glitch = ks::Probe::edges;
    ks::Scanner::stop();
    print(serial, "  scanner edges: at start ", at_start, ", after a rise ", after_rise, " (active ", rise_active,
          "), held 40 ms ", held, ", after a fall ", after_fall, " (active ", fall_active, "), after a 0.5 ms glitch ",
          after_glitch, crlf);
    bench.verdict("the InputScanner is silent at start and on a held level, publishes one InputEdge per flip "
                  "with the new state, and swallows a glitch shorter than its debounce",
                  at_start == 0u && after_rise == 1u && rise_active && held == 1u && after_fall == 2u &&
                      !fall_active && after_glitch == 2u);
    all_off();
}

void banner() {
    print(serial, crlf, "test_ch32_pin - CH32V006K8 GPIO (RM ch. 7) and EXTI (6.4)", crlf);
    print(serial, "  the jumper for c, d and f: PD2 <-> PD4; ", jumper_present ? "PRESENT" : "ABSENT", crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
extern "C" BRIO_CH32_INTERRUPT void exti7_0_handler() {
    const uint32_t f = brio::Exti::flags();
    line_flags_seen = line_flags_seen | f;
    if (Line::flag()) {
        Line::clear();
        line_interrupts = line_interrupts + 1u;
    } else {
        brio::exti()->INTFR = f;   // whatever else fired: cleared, uncounted
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();
    jumper_present = probe_jumper();

    bench.letter('a', "the ports, wireless: the gate, the reset nibbles, every nibble, BSHR/BCR", ta_ports);
    bench.letter('b', "the pulls, wireless: four pads up and down, the pull as OUTDR's bit", tb_pulls);
    bench.letter('c', "THE LEVELS on the jumper: push-pull, open-drain, an analog pad", tc_levels);
    bench.letter('d', "THE EDGES on the jumper: rising, falling, both; the flag; the port select", td_edges);
    bench.letter('e', "the software trigger and the EVENT mode ending a WFE, wireless", te_soft_and_event);
    bench.letter('f', "THE SCANNER on the jumper: InputScanner over PD4 driven by PD2", tf_scanner);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
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
