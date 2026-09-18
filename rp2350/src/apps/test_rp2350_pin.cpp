// test_rp2350_pin - the reference bench suite for the RP2350's GPIO
// (rp2350/pin.hpp, datasheet chapter 9): the function select and its
// four overrides, the pad's electrical controls, THE ISOLATION LATCHES
// this chip adds, the SIO path in both halves of the bank, THE PIN
// INTERRUPTS with their summary registers, the package's own edges, and
// ERRATUM RP2350-E9 shown on a pad and keyed on the stepping.
//
// Every letter runs on BOTH ARCHITECTURES from this one source and is
// judged the same way; the interrupt letter binds ONE NAME,
// `isr_io_bank0`, which the Arm crt puts in a vector table and the
// RISC-V one reaches through Hazard3's own dispatch.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// WHAT IS WIRED. The console is a Debug Probe's UART bridge on GP0 (TX)
// / GP1 (RX) = UART0. Four pads are wired to four others, one wire each
// - GP19 to GP8, GP11 to GP16, GP18 to GP10, GP17 to GP9 - and two pads
// are wired to two more WITH A REAL PULL-UP TO 3V3 ON THE NET: GP12 to
// GP14 and GP13 to GP15. Those pull-ups are the only thing on this bench
// that can hold a pad high without the chip's help, which is what makes
// them the witness in the pull letter. GP22 is free and is where the
// erratum is shown; GP40 is free and is where the high half of the bank
// is driven.
//
// THE RULER is the platform timer (rp2350/mtime.hpp), microseconds off
// clk_ref: it times the interrupt path. A microsecond is its grain, and
// the numbers below are read with that in mind.
//
// What is exercised, letter by letter:
//   a  the bank as this image finds it: the package the die reports
//      against the one the build states, the size of the bank, an
//      untouched pad's register printed, and release() putting a pad
//      back to the reset value the datasheet states
//   b  the four wires, both ways: every pad drives its neighbour and
//      reads it back, then the roles swap - a wire has no direction
//   c  the pulls: the two nets with a real pull-up on them, read high
//      with both ends listening and low when one end drives; then the
//      chip's own pull-up on a free pad
//   d  ERRATUM RP2350-E9, KEYED ON THE STEPPING: with its input buffer
//      enabled a free pad idles HIGH against its own pull-down, and the
//      errata sheet's own workaround - the buffer enabled for the read
//      alone - reads it LOW
//   e  the bus keeper: both pulls at once, holding the level the pad
//      last had
//   f  the pad's electrical controls: the four drive strengths, the slew
//      rate, the hysteresis, the input buffer and the output disable,
//      each written and read back, and the wire unmoved by any of them
//   g  the function select and the STATUS register: a pad handed to a
//      peripheral and taken back, what the console's own pads carry, and
//      what STATUS says actually reached the pad
//   h  the four overrides: the output forced high and low, the output
//      enable forced off, the input inverted UNDER SIO's own read, and
//      the interrupt forced from the multiplexer
//   i  THE PIN INTERRUPTS: an edge, both edges, a level that is not
//      latched, two pads at once, the summary registers, the forced
//      interrupt, and the path's latency on the ruler
//   j  THE ISOLATION LATCH: a pad that keeps its level, its direction
//      and its pull while the registers say otherwise, and takes all
//      three the moment the latch is opened
//   k  the package's edges: what this die bonds, what is refused at run
//      time, and the HIGH HALF of the bank driven and read through its
//      own registers
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/delay.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/sysinfo.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

using Led = Pin<25>;

// The four one-wire pairs, driver and listener.
using DriveA = Pin<19>;
using ListenA = Pin<8>;
using DriveB = Pin<11>;
using ListenB = Pin<16>;
using DriveC = Pin<18>;
using ListenC = Pin<10>;
using DriveD = Pin<17>;
using ListenD = Pin<9>;
// The two nets that carry a real pull-up to 3V3.
using PullNet0A = Pin<12>;
using PullNet0B = Pin<14>;
using PullNet1A = Pin<13>;
using PullNet1B = Pin<15>;
// A pad with nothing on it at all, and one in the high half of the bank.
using Free = Pin<22>;
using High = Pin<40>;

using IntListen = ExtInt<ListenA>;
using IntListen2 = ExtInt<ListenC>;

TestBench<Serial> bench;

uint32_t us_now() { return Mtime::micros(); }

// What the interrupt handler leaves behind. Volatile: the loop is the
// only reader and the handler the only writer.
volatile uint32_t isr_count = 0;
volatile uint32_t isr_us = 0;
volatile uint8_t isr_events = 0;
volatile uint32_t isr_count2 = 0;
volatile uint8_t isr_events2 = 0;

/// Whether a served set contains an event that clearing cannot end: a
/// LEVEL is not latched and stands while the pad holds it. The other
/// kind is a FORCED event, which the handler asks INTF about.
bool stands(PinEvents e) {
    return e.has(PinEvent::level_high) || e.has(PinEvent::level_low);
}

void isr_clear() {
    isr_count = 0;
    isr_us = 0;
    isr_events = 0;
    isr_count2 = 0;
    isr_events2 = 0;
}

/// Wait for the handler to have run at least once since `before`, up to
/// `budget` microseconds. The count it ended at.
uint32_t wait_isr(uint32_t before, uint32_t budget = 2000u) {
    const uint32_t t0 = us_now();
    while (isr_count == before && us_now() - t0 < budget) {
    }
    return isr_count;
}

/// Every pad this suite drives, back to the reset state - the shape a
/// letter leaves the bench in.
void release_all() {
    (void)DriveA::release();
    (void)ListenA::release();
    (void)DriveB::release();
    (void)ListenB::release();
    (void)DriveC::release();
    (void)ListenC::release();
    (void)DriveD::release();
    (void)ListenD::release();
    (void)PullNet0A::release();
    (void)PullNet0B::release();
    (void)PullNet1A::release();
    (void)PullNet1B::release();
    (void)Free::release();
    (void)High::release();
}

/// One wire, judged in one direction: `Out` drives, `In` listens.
template <typename Out, typename In>
void wire_both_levels(const char* name) {
    (void)Out::output();
    (void)In::input();
    Out::set();
    (void)delay_us(clock, 5);
    const bool high = In::read();
    Out::clear();
    (void)delay_us(clock, 5);
    const bool low = In::read();
    print(serial, "  ", name, ": driven high the far pad reads ", high ? 1 : 0,
          ", driven low ", low ? 1 : 0, crlf);
    bench.verdict(name, ": the far pad follows the near one", high && !low);
}

// -----------------------------------------------------------------------------
void ta_bank() {
    const Package sel = ChipId::package_sel();
    print(serial, "  the die reports the ", sel == Package::qfn60 ? "QFN-60" : "QFN-80",
          " package; this image was built for ", package_known ? "" : "no package in particular, so ",
          static_cast<uint32_t>(gpio_count), " pins, and the register files hold ",
          static_cast<uint32_t>(gpio_count_max), crlf);
    bench.verdict("the package the die reports is the one this image was built for",
                  !package_known || sel == package);
    print(serial, "  the pad input thresholds are set for ",
          Gpio::voltage() == PadVoltage::v3v3 ? "2.5 to 3.3 V" : "1.8 V", " (a board fact: both "
          "banks share one IOVDD, and no verb here writes it)", crlf);
    bench.verdict("the thresholds match the 3.3 V this board runs at",
                  Gpio::voltage() == PadVoltage::v3v3);

    // A pad nothing in this image has configured, printed as it stands -
    // a processor reset does not reset the pads, so what it holds is
    // whatever the last image left unless the flash went through the
    // rescue that does reset them.
    print(serial, "  GP", static_cast<uint32_t>(Free::number), "'s pad register as this boot "
          "found it: ", hex(Gpio::pad(Free::number)), " (the datasheet's reset value is 0x116: "
          "isolated, input buffer off, pulled down, 4 mA, hysteresis)", crlf);

    (void)Free::output();
    const uint32_t configured = Gpio::pad(Free::number);
    (void)Free::release();
    const uint32_t released = Gpio::pad(Free::number);
    print(serial, "  configured as an output it reads ", hex(configured), ", released ",
          hex(released), crlf);
    bench.verdict("a configuring verb drops the isolation latch and enables the input buffer",
                  (configured & PADS_BANK0_GPIO0_ISO_BITS) == 0u &&
                      (configured & PADS_BANK0_GPIO0_IE_BITS) != 0u);
    bench.verdict("and release() puts the pad back to the reset value the datasheet states",
                  released == PADS_BANK0_GPIO0_RESET);
    bench.verdict("a released pad has no owner either (FUNCSEL back to the null function)",
                  Free::function() == PinFunction::none);
}

// -----------------------------------------------------------------------------
void tb_wires() {
    wire_both_levels<DriveA, ListenA>("GP19 to GP8");
    wire_both_levels<DriveB, ListenB>("GP11 to GP16");
    wire_both_levels<DriveC, ListenC>("GP18 to GP10");
    wire_both_levels<DriveD, ListenD>("GP17 to GP9");
    // A wire has no direction: the same four, the other way round.
    wire_both_levels<ListenA, DriveA>("GP8 to GP19");
    wire_both_levels<ListenB, DriveB>("GP16 to GP11");
    wire_both_levels<ListenC, DriveC>("GP10 to GP18");
    wire_both_levels<ListenD, DriveD>("GP9 to GP17");
    release_all();
}

// -----------------------------------------------------------------------------
template <typename A, typename B>
void pulled_net(const char* name) {
    (void)A::input();
    (void)B::input();
    (void)delay_us(clock, 20);
    const bool idle_a = A::read();
    const bool idle_b = B::read();
    (void)A::output(false);
    (void)delay_us(clock, 20);
    const bool pulled_a = A::read();
    const bool pulled_b = B::read();
    (void)A::input();
    (void)delay_us(clock, 50);
    const bool back_a = A::read();
    const bool back_b = B::read();
    print(serial, "  ", name, ": both listening ", idle_a ? 1 : 0, idle_b ? 1 : 0,
          ", one end driving low ", pulled_a ? 1 : 0, pulled_b ? 1 : 0, ", released again ",
          back_a ? 1 : 0, back_b ? 1 : 0, crlf);
    bench.verdict(name, ": the external pull-up holds both pads high with nothing driving",
                  idle_a && idle_b);
    bench.verdict(name, ": one pad driving low takes the whole net down",
                  !pulled_a && !pulled_b);
    bench.verdict(name, ": released, the pull-up brings it back", back_a && back_b);
}

void tc_pulls() {
    pulled_net<PullNet0A, PullNet0B>("GP12 and GP14, with 3V3 through a resistor");
    pulled_net<PullNet1A, PullNet1B>("GP13 and GP15, the same");

    // The chip's own pull-up, on a pad with nothing else on it.
    (void)Free::input(PinPull::up);
    (void)delay_us(clock, 50);
    const bool up = Free::read();
    print(serial, "  GP22 with the chip's own pull-up: ", up ? 1 : 0, crlf);
    bench.verdict("the internal pull-up holds a free pad high", up);
    release_all();
}

// -----------------------------------------------------------------------------
void td_erratum_e9() {
    const uint8_t revision = ChipId::read().revision;
    const bool leaky = revision <= ChipId::revision_a2;   // A3 removes the path
    print(serial, "  this die's stepping reads ", hex(revision), " (A2 is 0x2), where the "
          "leakage path RP2350-E9 describes is ", leaky ? "LIVE" : "fixed", crlf);

    // The pad is driven HIGH and released with its own pull-down on and
    // the input buffer enabled: the errata sheet's first condition.
    (void)Free::output(true);
    (void)delay_us(clock, 20);
    (void)Free::input(PinPull::down);
    (void)delay_us(clock, 500);
    const bool after_high = Free::read();

    // And now the workaround: the same pull-down with the input buffer
    // OFF, read with the buffer enabled for the read alone.
    (void)Free::output(true);
    (void)delay_us(clock, 20);
    (void)Free::input({.pull = PinPull::down, .input_enable = false});
    (void)delay_us(clock, 500);
    const bool pulsed = Free::read_pulsed();

    // A pad driven LOW and released stays low even with the buffer on -
    // the leakage cannot lift a pad that is already below the undefined
    // region.
    (void)Free::output(false);
    (void)delay_us(clock, 20);
    (void)Free::input(PinPull::down);
    (void)delay_us(clock, 500);
    const bool after_low = Free::read();

    print(serial, "  GP22 pulled down, buffer on: after being driven high it reads ",
          after_high ? 1 : 0, ", after being driven low ", after_low ? 1 : 0,
          "; with the buffer pulsed for the read alone, after being driven high it reads ",
          pulsed ? 1 : 0, crlf);
    bench.verdict("with its input buffer enabled, a pad that was driven high and released "
                  "sits HIGH against its own pull-down - the erratum, on this stepping",
                  leaky ? after_high : !after_high);
    bench.verdict("the errata sheet's workaround reads the same pad LOW: the buffer is what "
                  "leaks, and read_pulsed() enables it for the read alone",
                  !pulsed);
    bench.verdict("a pad driven low and released stays low either way", !after_low);

    // WHAT THE BUFFER COSTS TO WAKE, since the workaround turns it on
    // and off around every read. The pad is held HIGH by its own
    // pull-up with the buffer off; the eight samples taken as fast as
    // SIO can be read say when the pad's own level first arrives - a
    // sample taken before the buffer is awake would read 0.
    (void)Free::input({.pull = PinPull::up, .input_enable = false});
    (void)delay_us(clock, 200);
    uint8_t samples = 0;
    Gpio::input_enable(Free::number, true);
    for (uint8_t i = 0; i < 8u; ++i) {
        samples = static_cast<uint8_t>(samples | (Free::read() ? (1u << i) : 0u));
    }
    Gpio::input_enable(Free::number, false);
    print(serial, "  the eight SIO samples right after the input buffer is enabled, on a pad "
          "its own pull-up holds high: ", hex(samples), " (bit 0 the first)", crlf);
    bench.verdict("the very first sample after the enable is already the pad's: the enable is "
                  "an APB write of four cycles and the read a single-cycle SIO one, so the "
                  "buffer is awake before it can be asked",
                  samples == 0xFFu);
    release_all();
}

// -----------------------------------------------------------------------------
void te_keeper() {
    (void)Free::output(true);
    (void)delay_us(clock, 20);
    (void)Free::input(PinPull::keeper);
    (void)delay_us(clock, 200);
    const bool held_high = Free::read();
    (void)Free::output(false);
    (void)delay_us(clock, 20);
    (void)Free::input(PinPull::keeper);
    (void)delay_us(clock, 200);
    const bool held_low = Free::read();
    print(serial, "  GP22 with both pulls on (the bus keeper): after high it reads ",
          held_high ? 1 : 0, ", after low ", held_low ? 1 : 0, crlf);
    bench.verdict("the bus keeper holds the level the pad last had, in both directions",
                  held_high && !held_low);
    release_all();
}

// -----------------------------------------------------------------------------
void tf_pad_controls() {
    // Every electrical control written and read back on a pad with a
    // listener on the other end of the wire, so that the wire can be
    // seen not to move.
    (void)ListenA::input();
    bool all_read_back = true;
    bool wire_held = true;
    for (uint8_t d = 0; d < 4u; ++d) {
        const PinDrive drive = static_cast<PinDrive>(d);
        const bool fast = (d & 1u) != 0u;
        const bool schmitt = (d & 2u) == 0u;
        (void)DriveA::output({.drive = drive, .slew_fast = fast, .schmitt = schmitt});
        DriveA::set();
        (void)delay_us(clock, 5);
        const uint32_t pad = Gpio::pad(DriveA::number);
        const uint32_t want = (static_cast<uint32_t>(d) << PADS_BANK0_GPIO0_DRIVE_LSB) |
                              (fast ? PADS_BANK0_GPIO0_SLEWFAST_BITS : 0u) |
                              (schmitt ? PADS_BANK0_GPIO0_SCHMITT_BITS : 0u) |
                              PADS_BANK0_GPIO0_IE_BITS;
        all_read_back = all_read_back && pad == want;
        wire_held = wire_held && ListenA::read();
    }
    print(serial, "  the four drive strengths with slew and hysteresis in turn: the pad "
          "register read back ", all_read_back ? "as written every time" : "WRONG",
          ", and the far pad stayed high throughout: ", wire_held ? "yes" : "no", crlf);
    bench.verdict("every drive strength, slew and hysteresis setting reads back as written",
                  all_read_back);
    bench.verdict("and none of them changes what the far end of a wire reads (this chip "
                  "cannot witness an edge rate, only its own registers)",
                  wire_held);

    // The output disable, which outranks whoever owns the pad. BOTH
    // pads' input buffers go off for this: on this stepping a pad whose
    // buffer is enabled and whose driver has let go leaks enough to hold
    // the whole net high against the far pad's pull-down (letter d), and
    // that is true of the DRIVING pad as much as of the listening one.
    Gpio::output_disable(DriveA::number, true);
    Gpio::input_enable(DriveA::number, false);
    (void)ListenA::input({.pull = PinPull::down, .input_enable = false});
    (void)delay_us(clock, 200);
    const bool driven_off = ListenA::read_pulsed();
    Gpio::output_disable(DriveA::number, false);
    (void)delay_us(clock, 20);
    const bool driven_on = ListenA::read_pulsed();
    print(serial, "  with the pad's OD bit set the far pad (pulled down, read through the "
          "errata's workaround) reads ", driven_off ? 1 : 0, "; cleared, ", driven_on ? 1 : 0,
          crlf);
    bench.verdict("the pad's own output disable stops the driver over SIO's head, and "
                  "clearing it hands the pad back",
                  !driven_off && driven_on);
    release_all();
}

// -----------------------------------------------------------------------------
void tg_function_and_status() {
    print(serial, "  the console's own pads: GP0 carries function ",
          static_cast<uint32_t>(Pin<0>::function()), ", GP1 function ",
          static_cast<uint32_t>(Pin<1>::function()), " (2 is the first UART column)", crlf);
    bench.verdict("the console's pads are held by the UART, not by SIO",
                  Pin<0>::function() == PinFunction::uart &&
                      Pin<1>::function() == PinFunction::uart);

    (void)DriveA::output(true);
    (void)ListenA::input();
    (void)delay_us(clock, 5);
    const PinStatus driving = DriveA::status();
    const PinStatus listening = ListenA::status();
    print(serial, "  GP19 driving high: STATUS says out ", driving.out_to_pad ? 1 : 0, ", oe ",
          driving.oe_to_pad ? 1 : 0, ", in ", driving.in_from_pad ? 1 : 0,
          "; GP8 listening: out ", listening.out_to_pad ? 1 : 0, ", oe ",
          listening.oe_to_pad ? 1 : 0, ", in ", listening.in_from_pad ? 1 : 0, crlf);
    bench.verdict("STATUS shows the level and the direction that reached the driving pad",
                  driving.out_to_pad && driving.oe_to_pad && driving.in_from_pad);
    bench.verdict("and on the listening pad it shows an input high and no driver",
                  !listening.oe_to_pad && listening.in_from_pad);

    // A pad handed to another function and taken back. GPIO_IN reads the
    // PAD whoever owns it, which is what makes the far end still visible.
    (void)DriveA::function(PinFunction::pio0);
    const PinFunction pio = DriveA::function();
    (void)DriveA::function(PinFunction::sio);
    DriveA::set();
    (void)delay_us(clock, 5);
    const bool back_under_sio = ListenA::read();
    print(serial, "  handed to PIO0 it reads function ", static_cast<uint32_t>(pio),
          ", back under SIO the wire follows again: ", back_under_sio ? 1 : 0, crlf);
    bench.verdict("a pad follows the function it is handed to, and comes back to SIO",
                  pio == PinFunction::pio0 && back_under_sio);
    release_all();
}

// -----------------------------------------------------------------------------
void th_overrides() {
    (void)DriveA::output(false);
    (void)ListenA::input();
    (void)delay_us(clock, 5);

    DriveA::out_override(PinOverride::high);
    (void)delay_us(clock, 5);
    const bool forced_high = ListenA::read();
    DriveA::out_override(PinOverride::low);
    (void)delay_us(clock, 5);
    const bool forced_low = ListenA::read();
    DriveA::out_override(PinOverride::invert);
    (void)delay_us(clock, 5);
    const bool inverted = ListenA::read();   // SIO says low, so the pad is high
    DriveA::out_override(PinOverride::pass);
    (void)delay_us(clock, 5);
    const bool passed = ListenA::read();
    print(serial, "  OUTOVER on GP19 with SIO saying low: forced high ", forced_high ? 1 : 0,
          ", forced low ", forced_low ? 1 : 0, ", inverted ", inverted ? 1 : 0, ", passed ",
          passed ? 1 : 0, crlf);
    bench.verdict("the output override forces, inverts and passes the level SIO drives",
                  forced_high && !forced_low && inverted && !passed);

    // The output enable override, witnessed through the far pad's own
    // pull-up (the chip's, this net having none of its own).
    DriveA::set();
    (void)ListenA::input(PinPull::up);
    DriveA::oe_override(PinOeOverride::disable);
    (void)delay_us(clock, 200);
    const bool floating = ListenA::read();
    DriveA::oe_override(PinOeOverride::pass);
    (void)delay_us(clock, 20);
    const bool driven = ListenA::read();
    DriveA::clear();
    (void)delay_us(clock, 20);
    const bool driven_low = ListenA::read();
    print(serial, "  OEOVER disable on GP19, GP8 pulled up: ", floating ? 1 : 0,
          "; the override passed again and GP19 driving low: ", driven_low ? 1 : 0, crlf);
    bench.verdict("the output-enable override takes the driver off the wire, and passing it "
                  "again puts it back",
                  floating && driven && !driven_low);

    // The INPUT override sits between the pad and the function - and SIO
    // is a function, so SIO's own GPIO_IN sees the inversion.
    ListenA::in_override(PinOverride::invert);
    (void)delay_us(clock, 5);
    const bool seen_inverted = ListenA::read();
    ListenA::in_override(PinOverride::high);
    const bool seen_forced = ListenA::read();
    ListenA::in_override(PinOverride::pass);
    (void)delay_us(clock, 5);
    const bool seen_plain = ListenA::read();
    const PinStatus at_pad = ListenA::status();
    print(serial, "  INOVER on GP8 with the wire low: inverted SIO reads ",
          seen_inverted ? 1 : 0, ", forced high ", seen_forced ? 1 : 0, ", passed ",
          seen_plain ? 1 : 0, "; STATUS's own in_from_pad ", at_pad.in_from_pad ? 1 : 0, crlf);
    bench.verdict("the input override inverts and forces what SIO reads, and STATUS still "
                  "reports the PAD",
                  seen_inverted && seen_forced && !seen_plain && !at_pad.in_from_pad);
    release_all();
}

// -----------------------------------------------------------------------------
void ti_interrupts() {
    (void)DriveA::output(false);
    (void)ListenA::input();
    (void)DriveC::output(false);
    (void)ListenC::input();
    (void)delay_us(clock, 20);
    isr_clear();
    Irq::enable(PinIrq::irq());

    // One edge, and the path timed on the ruler - thirty-two times, so
    // that what is reported is a spread and not one sample of a grain.
    bench.verdict("a pad with nothing armed raises nothing", IntListen::pending().none());
    uint32_t worst = 0;
    uint32_t best = 0xFFFF'FFFFu;
    uint32_t edges = 0;
    for (uint8_t i = 0; i < 32u; ++i) {
        DriveA::clear();
        (void)delay_us(clock, 20);
        isr_clear();
        (void)IntListen::arm(pin_events(PinEvent::edge_high));
        const uint32_t t0 = us_now();
        DriveA::set();
        if (wait_isr(0) == 1u) {
            ++edges;
            const uint32_t latency = isr_us - t0;
            worst = latency > worst ? latency : worst;
            best = latency < best ? latency : best;
        }
    }
    print(serial, "  thirty-two rising edges on GP19: ", edges, " served, the store to the "
          "handler's own timestamp took ", best, " to ", worst, " us (the ruler's grain is 1 us)"
          ", the last events ", static_cast<uint32_t>(isr_events), crlf);
    bench.verdict("every rising edge is served exactly once, and the path's own cost is under "
                  "the ruler's grain - the worst of the thirty-two is the one that waited for "
                  "another handler, since no interrupt nests over another here",
                  edges == 32u && best <= 1u && worst <= 20u &&
                      isr_events == static_cast<uint8_t>(PinEvent::edge_high));

    // The falling edge is NOT armed, so it must raise nothing.
    DriveA::clear();
    (void)delay_us(clock, 50);
    bench.verdict("the falling edge, which is not armed, raises nothing",
                  isr_count == 1u);

    // Both edges: two more interrupts for two more transitions.
    (void)IntListen::arm(pin_edges);
    DriveA::set();
    (void)wait_isr(1);
    DriveA::clear();
    (void)wait_isr(2);
    print(serial, "  with both edges armed, a high and a low made ",
          isr_count - 1u, " more interrupts, the last of them ",
          static_cast<uint32_t>(isr_events), crlf);
    bench.verdict("both edges armed serve both transitions, and the second is the falling one",
                  isr_count == 3u && isr_events == static_cast<uint8_t>(PinEvent::edge_low));

    // A LEVEL is not latched: it stands while the pad holds it, so a
    // handler that means to return must disarm it. Here the handler
    // disarms the pad it served, and the level raises exactly one.
    IntListen::disarm();
    isr_clear();
    DriveA::set();
    (void)delay_us(clock, 20);
    (void)IntListen::arm(pin_events(PinEvent::level_high));
    const uint32_t level_served = wait_isr(0);
    (void)delay_us(clock, 200);
    print(serial, "  a level that is already true fires at once (", level_served,
          " interrupt) and the handler disarming it stops there: ", isr_count, crlf);
    bench.verdict("a level event fires on a pad that already holds that level, and "
                  "disarming is what ends it (clearing cannot: a level is not latched)",
                  level_served == 1u && isr_count == 1u);

    // Two pads at once, and the summary registers that say which.
    isr_clear();
    IntListen::disarm();
    IntListen2::disarm();
    DriveA::clear();
    DriveC::clear();
    (void)delay_us(clock, 20);
    (void)IntListen::arm(pin_events(PinEvent::edge_high));
    (void)IntListen2::arm(pin_events(PinEvent::edge_high));
    DriveA::set();
    DriveC::set();
    (void)delay_us(clock, 200);
    print(serial, "  two pads armed and both driven: GP8 served ", isr_count, ", GP10 served ",
          isr_count2, crlf);
    bench.verdict("two pads' interrupts are two pads' interrupts, on one line",
                  isr_count >= 1u && isr_count2 >= 1u);

    // The summary register, this chip's own: one bit per PIN.
    IntListen::disarm();
    IntListen2::disarm();
    DriveA::clear();
    (void)delay_us(clock, 20);
    Irq::disable(PinIrq::irq());
    (void)IntListen::arm(pin_events(PinEvent::edge_high));
    DriveA::set();
    (void)delay_us(clock, 20);
    const uint32_t summary = PinIrq::summary(PinIrqTarget::proc0, 0);
    print(serial, "  with the line masked, the summary register reads ", hex(summary),
          " - bit ", static_cast<uint32_t>(ListenA::number), " for GP8", crlf);
    bench.verdict("the summary register names the pin that is asking, and no other",
                  summary == (1UL << ListenA::number));

    // The forced interrupt: the block's own INTF, with nothing on the
    // wire - AND AN EVENT NEITHER CLEARING NOR DISARMING CAN END. The
    // status register is "after masking AND FORCING", so a force
    // bypasses the enable; the handler unforces what it served, which
    // is the only way out, and this letter checks that INTF is empty
    // when it returns. The pad is DISARMED here first, to show that the
    // force reaches the line with no event armed at all.
    IntListen::clear();
    IntListen::disarm();
    isr_clear();
    Irq::enable(PinIrq::irq());
    IntListen::force(pin_events(PinEvent::edge_high));
    const uint32_t forced = wait_isr(0);
    const PinEvents standing = IntListen::forced();
    print(serial, "  an interrupt forced through INTF with the wire untouched and NOTHING "
          "armed: ", forced, " served, and INTF after the handler reads ",
          static_cast<uint32_t>(standing.bits), crlf);
    bench.verdict("a forced interrupt reaches the handler with no event armed - the status is "
                  "after masking AND forcing - and the handler's unforce is what ends it",
                  forced == 1u && standing.none());

    IntListen::disarm();
    IntListen2::disarm();
    Irq::disable(PinIrq::irq());
    PinIrq::disable_all();
    release_all();
}

// -----------------------------------------------------------------------------
void tj_isolation() {
    // The latch freezes what the switched core presents to the pad: the
    // level, the direction and the pull. It is the one thing in this
    // chapter the RP2040 has not, and this is what it does.
    (void)DriveA::output(true);
    (void)ListenA::input();
    (void)delay_us(clock, 20);
    const bool before = ListenA::read();

    DriveA::isolate(true);
    DriveA::clear();                       // the register says low
    (void)delay_us(clock, 50);
    const bool while_isolated = ListenA::read();
    const bool register_says = DriveA::read_out();

    DriveA::isolate(false);                // the latch becomes transparent
    (void)delay_us(clock, 50);
    const bool after = ListenA::read();
    print(serial, "  GP19 driving high, then isolated and told to go low: the far pad reads ",
          before ? 1 : 0, " -> ", while_isolated ? 1 : 0, " while SIO's own register says ",
          register_says ? 1 : 0, "; the latch opened, ", after ? 1 : 0, crlf);
    bench.verdict("an isolated pad keeps the level it had while the register says otherwise",
                  before && while_isolated && !register_says);
    bench.verdict("and it takes the new level the moment the latch is opened", !after);

    // The direction is latched too: isolated, an output enable that goes
    // away does not release the pad. Both buffers go off again, for the
    // reason letter d gives: a released pad with its buffer on holds the
    // net high by itself on this stepping.
    (void)DriveA::output(true, {.input_enable = false});
    (void)ListenA::input({.pull = PinPull::down, .input_enable = false});
    (void)delay_us(clock, 20);
    DriveA::isolate(true);
    Gpio::oe_clear(DriveA::mask);          // SIO says input, through SIO alone
    (void)delay_us(clock, 200);
    const bool still_driven = ListenA::read_pulsed();
    DriveA::isolate(false);
    (void)delay_us(clock, 200);
    const bool released = ListenA::read_pulsed();
    print(serial, "  isolated and told through SIO to stop driving, the far pad (pulled down) "
          "reads ", still_driven ? 1 : 0, "; the latch opened, ", released ? 1 : 0, crlf);
    bench.verdict("the output ENABLE is latched as well: an isolated pad keeps driving until "
                  "the latch is opened",
                  still_driven && !released);

    // AND A CONFIGURING VERB IS THE WAY OUT, by design: every one of
    // them writes the whole pad register, and ISO clear is part of the
    // value - which is why the direction above was changed through SIO
    // and not through Pin::input().
    DriveA::isolate(true);
    const bool isolated_before = DriveA::isolated();
    (void)DriveA::output(true);
    const bool isolated_after = DriveA::isolated();
    print(serial, "  isolated (", isolated_before ? 1 : 0, "), a configuring verb leaves it ",
          isolated_after ? 1 : 0, crlf);
    bench.verdict("a configuring verb drops the latch on its way: writing a pad IS taking it "
                  "out of isolation",
                  isolated_before && !isolated_after);
    release_all();
}

// -----------------------------------------------------------------------------
void tk_package() {
    print(serial, "  this build is for ", static_cast<uint32_t>(gpio_count),
          " pins; bonded(29) ", Gpio::bonded(29u) ? 1 : 0, ", bonded(40) ",
          Gpio::bonded(40u) ? 1 : 0, ", bonded(47) ", Gpio::bonded(47u) ? 1 : 0,
          ", bonded(48) ", Gpio::bonded(48u) ? 1 : 0, crlf);
    bench.verdict("every pad of this package answers, and the first one past the register "
                  "files does not",
                  Gpio::bonded(29u) && Gpio::bonded(gpio_count - 1u) && !Gpio::bonded(48u));
    bench.verdict("a pad past the bank is not written, and the caller is told",
                  !Gpio::function(48u, PinFunction::sio) && !Gpio::release(48u) &&
                      !Gpio::analog(48u));
    bench.verdict("a run-time pin reference past the bank is not valid, and does nothing",
                  !PinRef{48u}.valid() && !PinRef{}.valid() && PinRef{25u}.valid());

    // The high half of the bank: its own SIO registers, its own bit
    // positions, on a pad this package brings out and nothing is wired to.
    (void)High::output(true);
    (void)delay_us(clock, 5);
    const bool high_set = High::read() && High::read_out() && High::is_output();
    High::clear();
    (void)delay_us(clock, 5);
    const bool high_clear = !High::read() && !High::read_out();
    const uint32_t hi_word = Gpio::out_hi();
    print(serial, "  GP", static_cast<uint32_t>(High::number), ", in the high half of the "
          "bank: driven high it reads back high, driven low it reads low; GPIO_HI_OUT is ",
          hex(hi_word), ", its bit ", static_cast<uint32_t>(High::number - 32u), crlf);
    bench.verdict("a pad above GP31 is driven and read through the HIGH registers, at its "
                  "own bit",
                  high_set && high_clear && High::mask == (1UL << (High::number - 32u)));
    release_all();
}

void banner() {
    print(serial, crlf, "test_rp2350_pin - the RP2350's GPIO (datasheet 9), ",
          static_cast<uint32_t>(gpio_count), " pads, stepping ", hex(ChipId::read().revision),
          ", on ",
          core_kind == CoreKind::cortex_m33 ? "the Cortex-M33" : "Hazard3", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }

/// ONE NAME ON BOTH ARCHITECTURES: the bank's Secure interrupt line for
/// this core. The handler asks each armed pad what it served - which
/// clears the latched events - and disarms a LEVEL, which cannot be
/// cleared and would otherwise stand for ever.
extern "C" void isr_io_bank0() {
    const brio::PinEvents a = IntListen::served();
    if (!a.none()) {
        isr_us = us_now();
        isr_events = a.bits;
        isr_count = isr_count + 1u;
        const brio::PinEvents forced = IntListen::forced();
        if (!forced.none()) {
            IntListen::unforce(forced);   // a force outlives the enable
        } else if (stands(a)) {
            IntListen::disarm();          // a level outlives the clear
        }
    }
    const brio::PinEvents c = IntListen2::served();
    if (!c.none()) {
        isr_events2 = c.bits;
        isr_count2 = isr_count2 + 1u;
        const brio::PinEvents forced = IntListen2::forced();
        if (!forced.none()) {
            IntListen2::unforce(forced);
        } else if (stands(c)) {
            IntListen2::disarm();
        }
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool ruler_ok = brio::Mtime::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the bank, the package, a pad's reset state", ta_bank);
    bench.letter('b', "the four wires, driven and read both ways", tb_wires);
    bench.letter('c', "the pulls, against the two nets that carry a real pull-up", tc_pulls);
    bench.letter('d', "erratum RP2350-E9 on a free pad, keyed on the stepping", td_erratum_e9);
    bench.letter('e', "the bus keeper", te_keeper);
    bench.letter('f', "the pad's electrical controls, written and read back", tf_pad_controls);
    bench.letter('g', "the function select and the STATUS register", tg_function_and_status);
    bench.letter('h', "the four overrides of the IO multiplexer", th_overrides);
    bench.letter('i', "the pin interrupts: edges, a level, two pads, the summary, a force",
                 ti_interrupts);
    bench.letter('j', "the isolation latch", tj_isolation);
    bench.letter('k', "the package's edges and the high half of the bank", tk_package);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " ruler=", ruler_ok ? "1us" : "FAILED", " tick=", tick_ok ? "1kHz" : "FAILED",
                    brio::crlf);
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
