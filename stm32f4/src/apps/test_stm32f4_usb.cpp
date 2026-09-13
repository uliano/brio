// test_stm32f4_usb - the reference bench suite for the STM32F4's USB OTG
// controller in device mode (stm32f4/usb.hpp over RM0383 ch. 22) under
// the device stack (util/usb/device.hpp) and the CDC ACM class
// (util/usb/cdc.hpp): the core's own bring-up, the FIFO map, the
// enumeration a real host drives, the port as a byte transport.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// THE CONSOLE IS THE PROBE'S UART BRIDGE on USART1; the board's own USB
// connector goes to the host, which is the other end of every measure:
// Linux enumerates the device (the pid.codes test identity 1209:0001),
// binds cdc-acm, and the host letters talk to /dev/ttyACM* with pyserial.
//
// THE CLOCK IS 96 MHz AND NOT THE PART'S 100: the controller wants 48 MHz
// exactly on its own domain, which is the main PLL's Q output, and from a
// 25 MHz crystal 96 MHz is the rate whose ratio gives it.
//
// What is exercised, letter by letter:
//   a  the controller and the enumeration, no host action needed: the
//      core out of reset and identified, the FIFO map, the pull-up down,
//      then the host does what a host does - the reset, the descriptors,
//      the address, the configuration - and the suite watches the state
//      reach CONFIGURED, with the setups and stalls counted
//   b  the FIFO budget and the claims that are refused: endpoint zero,
//      a control or isochronous type, a packet past 64, and the budget
//      spent - each answered false with nothing written
//   c  the core's state registers: the mode, the enumerated speed, the
//      session, the address the driver keeps because the field does not
//      read back (ES0287 2.12.6)
//   d  the control endpoint's tally: the descriptors' own sizes, the
//      setups the enumeration took, the stalls the rules ask for
//   e  the start-of-frame counter: a frame a millisecond, measured
//   f  a reconnect through the soft disconnect: the pull-up dropped and
//      raised, the host's reset counted, the port configured again
//   g  suspend and resume: with the pull-up down there is no activity on
//      the lines, which is the suspend condition the chapter names
//   h  the clock gating of 22.8: the PHY clock stopped while suspended
//      and the port alive afterwards
//   i  the stall bits: endpoint zero's, a bulk endpoint's, and the
//      clear that puts the data toggle back
//
//   y  (HOST-ASSISTED, outside the all-key) ECHO: for ten seconds every
//      byte the host sends comes back; the host end judges it byte-exact
//   w  (host-assisted) THE LINE CODING AND DTR: the values as they
//      stand, ten seconds for the host to change them, the values again
//   v  (host-assisted) THROUGHPUT: three seconds of the device pouring
//      into the port, then two seconds counting what the host pours in
//   x  (host-assisted) NAK FLOW CONTROL: the device stops reading for
//      two seconds while the host writes a counting pattern, then drains
//      it and checks every byte followed the last - what the host saw as
//      NAKs cost no byte
//   t  (host-assisted) THE ZERO-LENGTH PACKET: exactly 1024 bytes - a
//      whole number of full packets - poured out, which the host's read
//      only returns from because a zero-length packet closes the run
//
// build: boards = f411ce
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "stm32f4/usb.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

namespace {

using namespace brio;

using P = Stm32f4Platform<>;
using SysClock = Clock<ClockSource::pll_hse, 96'000'000, 25'000'000>;
constexpr SysClock clock;
static_assert(SysClock::usb_hz == otg_clock_hz, "this rate must give the controller 48 MHz");

constexpr UartPins console_pins{
    .tx = {'A', 9, PinFunction::af7},
    .rx = {'A', 10, PinFunction::af7},
};
using Serial = Uart<1, console_pins>;
constexpr Serial serial;
using Led = Pin<'C', 13>;   // lit when low
using Usb = UsbFs;

TestBench<Serial, 20> bench;

using Port = UsbCdcAcm<Usb, P, 0, 1, 2, 2048, 512>;

struct Descriptors {
    static constexpr auto device =
        usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        usb_concat(usb_configuration_head(9 + Port::descriptor_bytes, Port::interface_count),
                   Port::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static constexpr auto manufacturer = usb_string_descriptor("brio");
    static constexpr auto product = usb_string_descriptor("test_stm32f4_usb");
    static constexpr auto serial_number = usb_string_descriptor("bench");
    static std::span<const uint8_t> string(uint8_t index) {
        switch (index) {
        case 0: return language;
        case 1: return manufacturer;
        case 2: return product;
        case 3: return serial_number;
        default: return {};
        }
    }
};
using Device = UsbDevice<Usb, Descriptors, Port>;

// ---- a ruler ------------------------------------------------------------------
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

uint32_t systick_period() { return SysTick->LOAD + 1u; }

/// A cycle count that spans ticks; it wraps in some forty seconds, which
/// is longer than anything measured here.
uint32_t cycles_now() {
    const uint32_t period = systick_period();
    uint32_t t0 = 0, v = 0, t1 = 0;
    do {
        t0 = Ticker::ticks();
        v = SysTick->VAL;
        t1 = Ticker::ticks();
    } while (t0 != t1);
    return t0 * period + (period - 1u - v);
}
uint32_t us_now() { return cycles_now() / cycles_per_us; }

void wait_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}

const char* state_name(UsbDeviceState s) {
    switch (s) {
    case UsbDeviceState::detached: return "detached";
    case UsbDeviceState::defaulted: return "default";
    case UsbDeviceState::addressed: return "addressed";
    case UsbDeviceState::configured: return "configured";
    }
    return "?";
}

/// Wait for the device to reach CONFIGURED; the microseconds it took, or
/// 0 on a timeout.
uint32_t wait_configured(uint32_t budget_us) {
    const uint32_t t0 = us_now();
    while (!Device::configured()) {
        if (us_now() - t0 > budget_us) {
            return 0;
        }
    }
    return us_now() - t0;
}

/// A host letter's clean start: whatever a previous letter left in the
/// rings is dropped, so the counts are this letter's own.
void settle_port() {
    uint8_t b = 0;
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 1'000'000u) {
        if (!Port::read_byte(b) && Port::tx_idle()) {
            break;
        }
    }
}

/// The controller and the stack from nothing: what letter a does and what
/// every letter that detaches the device does to put it back.
uint32_t bring_up(uint32_t budget_us) {
    Device::stop();
    Usb::release();
    if (!Usb::init(clock)) {
        return 0;
    }
    Device::start();
    return wait_configured(budget_us);
}

bool usb_up = false;

// =============================================================================
// a - the controller, and the host enumerating the device
// =============================================================================
void ta_enumerate() {
    Device::stop();
    Usb::release();
    const uint32_t t0 = us_now();
    usb_up = Usb::init(clock);
    const uint32_t init_us = us_now() - t0;
    print(serial, "  init: ", usb_up ? "ok" : "FAILED", " in ", init_us, " us; core id ",
          hex(Usb::core_id()), ", device mode=", Usb::in_device_mode(), ", session valid=",
          Usb::session_valid(), ", pulled up=", Usb::pulled_up(), crlf);
    bench.verdict("the core comes up in device mode with its identifier readable and the pull-up "
                  "still down (the stack raises it)",
                  usb_up && Usb::in_device_mode() && Usb::core_id() != 0u && !Usb::pulled_up());
    print(serial, "  FIFO RAM: ", Usb::fifo_words, " words, receive ", Usb::rx_words,
          ", endpoint zero's transmit ", Usb::ep0_tx_words, ", spoken for ", Usb::fifo_words_used(),
          ", endpoints ", Usb::endpoint_count, ", OUT slots ", Usb::out_slots, crlf);
    bench.verdict("the FIFO map fits the core's dedicated RAM with room left for the class's "
                  "transmit FIFOs",
                  Usb::fifo_words_used() == Usb::rx_words + Usb::ep0_tx_words &&
                      Usb::fifo_words_used() < Usb::fifo_words);
    bench.verdict("the descriptors: the device's 18 bytes, the configuration's 67 (two interfaces, "
                  "three endpoints), four strings",
                  Descriptors::device.size() == 18u && Descriptors::configuration.size() == 67u &&
                      Descriptors::configuration[4] == 2u && Descriptors::string(3).size() == 12u &&
                      Descriptors::string(4).empty());

    const uint32_t stalls_before = Device::stalls();
    const uint32_t setups_before = Device::setups();
    Device::start();
    const uint32_t took = wait_configured(3'000'000u);
    print(serial, "  start(): pull-up=", Usb::pulled_up(), "; the host took ", took,
          " us to configure: state=", state_name(Device::state()), " address=", Device::address(),
          " resets=", Device::resets(), " enumerations=", Usb::enumerations(),
          " setups=", Device::setups(), " stalls=", Device::stalls(),
          " last request=", Device::last_request(), " frame=", Usb::frame(), crlf);
    bench.verdict("start() raises the pull-up and the host enumerates the device to CONFIGURED "
                  "within three seconds",
                  Usb::pulled_up() && took != 0u && Device::configured());
    bench.verdict("the host reset the bus at least once, the core saw the reset end, and a "
                  "non-zero address was assigned",
                  Device::resets() >= 1u && Usb::enumerations() >= 1u && Device::address() != 0u);
    bench.verdict("enumeration took at least the four requests a host must make, and the stall "
                  "counter is small (a device qualifier or a string the device has not are stalled "
                  "by the rules)",
                  Device::setups() - setups_before >= 4u && Device::stalls() - stalls_before <= 8u);
    bench.verdict("the port is configured with its bulk OUT armed and nothing in flight",
                  Port::configured() && Port::out_armed() && !Port::in_armed() && Port::tx_idle());
    print(serial, "  after the class: spoken for ", Usb::fifo_words_used(), " of ", Usb::fifo_words,
          " words; the bulk OUT holds ", Usb::out_armed(2), " packets armed; receive entries ",
          Usb::rx_entries(), ", FIFO waits ", Usb::fifo_waits(), ", timeouts ", Usb::timeouts(),
          ", mode mismatches ", Usb::mode_mismatches(), crlf);
    bench.verdict("the class's three endpoints took their transmit FIFOs out of the budget",
                  Usb::fifo_words_used() > Usb::rx_words + Usb::ep0_tx_words &&
                      Usb::fifo_words_used() <= Usb::fifo_words);
    bench.verdict("no mode mismatch and no control-IN timeout during the enumeration",
                  Usb::mode_mismatches() == 0u && Usb::timeouts() == 0u);
}

// =============================================================================
// b - the FIFO budget, and the claims that are refused
// =============================================================================
void tb_budget() {
    Device::stop();
    Usb::release();
    const bool up = Usb::init(clock);
    const uint16_t at_rest = Usb::fifo_words_used();

    const bool zero_refused = !Usb::configure_endpoint(usb_ep_in(0), UsbEndpointType::bulk, 64);
    const bool control_refused = !Usb::configure_endpoint(usb_ep_in(1), UsbEndpointType::control, 64);
    const bool iso_refused =
        !Usb::configure_endpoint(usb_ep_in(1), UsbEndpointType::isochronous, 64);
    const bool wide_refused = !Usb::configure_endpoint(usb_ep_in(1), UsbEndpointType::bulk, 65);
    const bool past_last_refused =
        !Usb::configure_endpoint(usb_ep_in(Usb::endpoint_count), UsbEndpointType::bulk, 64);
    print(serial, "  at rest ", at_rest, " words; refusals: endpoint zero=", zero_refused,
          " control=", control_refused, " isochronous=", iso_refused, " 65 bytes=", wide_refused,
          " past the last endpoint=", past_last_refused, crlf);
    bench.verdict("a claim on endpoint zero, of a control or isochronous type, of a packet past "
                  "64 bytes, or on an endpoint number this core has not, is refused",
                  zero_refused && control_refused && iso_refused && wide_refused &&
                      past_last_refused);
    bench.verdict("a refused claim spends none of the budget", Usb::fifo_words_used() == at_rest);

    // Every IN endpoint this core has, claimed in turn: each takes its
    // own slice, and the sum is what the readback says.
    uint8_t taken = 0;
    for (uint8_t n = 1; n < Usb::endpoint_count; ++n) {
        if (Usb::configure_endpoint(usb_ep_in(n), UsbEndpointType::bulk, 64)) {
            ++taken;
        }
    }
    print(serial, "  ", taken, " IN endpoints claimed: ", Usb::fifo_words_used(), " words of ",
          Usb::fifo_words, crlf);
    bench.verdict("each IN endpoint takes one packet's worth of words - the register's minimum of "
                  "16 - out of the budget",
                  taken == Usb::endpoint_count - 1u &&
                      Usb::fifo_words_used() == at_rest + 16u * taken);
    Usb::deconfigure_endpoints();
    bench.verdict("deconfigure_endpoints gives every slice back", Usb::fifo_words_used() == at_rest);

    const uint32_t took = bring_up(3'000'000u);
    print(serial, "  brought up again in ", took, " us, state=", state_name(Device::state()), crlf);
    bench.verdict("the controller and the port come back after the poking (a first)",
                  up && took != 0u && Port::configured());
}

// =============================================================================
// c - the core's state registers
// =============================================================================
void tc_state() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    print(serial, "  mode=", Usb::in_device_mode() ? "device" : "host",
          " enumerated speed=", Usb::enumerated_speed(), " suspended=", Usb::suspended(),
          " erratic=", Usb::erratic_error(), " session valid=", Usb::session_valid(),
          " address=", Usb::address(), " (stack ", Device::address(), ")", crlf);
    bench.verdict("the core is a full-speed device, awake, with no erratic error",
                  Usb::in_device_mode() && Usb::enumerated_speed() == 3u && !Usb::suspended() &&
                      !Usb::erratic_error());
    bench.verdict("the VBUS pad is given away, so the core takes a session as valid at all times",
                  Usb::session_valid());
    bench.verdict("the address the driver keeps agrees with the stack's - the register's own field "
                  "is never read, because ES0287 2.12.6 says it does not read back",
                  Usb::address() == Device::address() && Usb::address() != 0u);
    print(serial, "  counters: receive entries ", Usb::rx_entries(), ", FIFO waits ",
          Usb::fifo_waits(), ", stalls ", Usb::stalls(), ", timeouts ", Usb::timeouts(),
          ", setup overruns ", Usb::setup_overruns(), ", early suspends ", Usb::early_suspends(),
          ", sessions ", Usb::sessions(), ", OTG events ", Usb::otg_events(), crlf);
    bench.verdict("the receive FIFO carried at least one entry per setup packet plus its stage-done "
                  "word",
                  Usb::rx_entries() >= 2u * Device::setups());
}

// =============================================================================
// d - the control endpoint's tally
// =============================================================================
void td_control() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    const UsbSetup s = Usb::setup();
    print(serial, "  the last setup packet: type ", hex(s.request_type), " request ", s.request,
          " value ", hex(s.value), " index ", s.index, " length ", s.length, crlf);
    bench.verdict("the last setup packet the core delivered is a whole one (the eight bytes came "
                  "out of the receive FIFO as one entry)",
                  Device::setups() > 0u && s.request == Device::last_request());
    print(serial, "  the stack: resets ", Device::resets(), " setups ", Device::setups(),
          " stalls ", Device::stalls(), " suspends ", Device::suspends(), " resumes ",
          Device::resumes(), crlf);
    bench.verdict("the configuration the host chose is the one and only one this device offers",
                  Device::state() == UsbDeviceState::configured &&
                      Descriptors::configuration[5] == 1u);
    bench.verdict("the device descriptor states one configuration and a 64-byte control endpoint",
                  Descriptors::device[7] == 64u && Descriptors::device[17] == 1u);
}

// =============================================================================
// e - the start-of-frame counter
// =============================================================================
void te_frames() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    const uint16_t f0 = Usb::frame();
    const uint32_t t0 = us_now();
    wait_us(100'000u);
    const uint32_t span = us_now() - t0;
    const uint16_t frames = static_cast<uint16_t>((Usb::frame() - f0) & 0x3FFFu);
    print(serial, "  ", frames, " frames in ", span, " us (", f0, " -> ", Usb::frame(), ")", crlf);
    bench.verdict("the host's start-of-frame counter advances a frame a millisecond",
                  frames >= 95u && frames <= 105u);
}

// =============================================================================
// f - a reconnect through the soft disconnect
// =============================================================================
void tf_reconnect() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    const uint32_t resets = Device::resets();
    const uint8_t address = Device::address();
    Device::stop();
    wait_us(500'000u);
    print(serial, "  the pull-up down for 500 ms: state=", state_name(Device::state()),
          " suspended=", Usb::suspended(), "; start() again...", crlf);
    Device::start();
    const uint32_t took = wait_configured(3'000'000u);
    print(serial, "  resets ", resets, " -> ", Device::resets(), ", address ", address, " -> ",
          Device::address(), ", configured in ", took, " us, state=", state_name(Device::state()),
          crlf);
    bench.verdict("dropping the pull-up detaches the device; raising it brings the host's bus reset "
                  "(counted) and a fresh enumeration",
                  Device::resets() > resets && took != 0u);
    bench.verdict("the port is configured again with its bulk OUT armed",
                  Port::configured() && Port::out_armed());
}

// =============================================================================
// g - suspend and resume
// =============================================================================
void tg_suspend() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    const uint32_t suspends = Device::suspends();
    const uint32_t early = Usb::early_suspends();
    // DETACHED IS NOT SUSPENDED. With the pull-up down the core is off
    // the bus and its suspend detector with it; the suspend condition
    // 22.16.4 names is an idle bus the device is still ATTACHED to.
    Device::stop();
    const uint32_t t0 = us_now();
    uint32_t detached_us = 0;
    while (us_now() - t0 < 200'000u) {
        if (Usb::suspended() && detached_us == 0u) {
            detached_us = us_now() - t0;
        }
    }
    print(serial, "  the pull-up down for 200 ms: SUSPSTS after ", detached_us,
          " us (0 = never), early suspends ", early, " -> ", Usb::early_suspends(), crlf);
    bench.verdict("a detached core reports no suspend: the condition is an idle bus the device is "
                  "still attached to",
                  detached_us == 0u && Usb::early_suspends() == early);

    // ATTACHED AND IGNORED IS SUSPENDED, and that is the window this
    // letter measures: the host takes some hundreds of milliseconds to
    // notice the pull-up and reset the bus, and until it does the lines
    // are idle with the device on them. Three milliseconds of that is
    // the early suspend, three more the suspend itself.
    const uint32_t t1 = us_now();
    Device::start();
    uint32_t suspended_us = 0;
    uint32_t left_us = 0;
    while (us_now() - t1 < 3'000'000u) {
        if (suspended_us == 0u && Usb::suspended()) {
            suspended_us = us_now() - t1;
        } else if (suspended_us != 0u && left_us == 0u && !Usb::suspended()) {
            left_us = us_now() - t1;
        }
        if (Device::configured()) {
            break;
        }
    }
    const uint32_t took = us_now() - t1;
    print(serial, "  the pull-up up: SUSPSTS at ", suspended_us, " us, gone at ", left_us,
          " us, configured at ", took, " us; early suspends ", early, " -> ",
          Usb::early_suspends(), ", stack suspends ", suspends, " -> ", Device::suspends(), crlf);
    bench.verdict("attached to a bus that is not talking to it yet, the core counts the idleness "
                  "and enters the suspended state, reporting the early suspend and the suspend",
                  suspended_us != 0u && Usb::early_suspends() > early &&
                      Device::suspends() > suspends);
    bench.verdict("the host's bus reset takes the core out of the suspended state and the port is "
                  "configured again",
                  left_us != 0u && !Usb::suspended() && Device::configured() && Port::configured());
}

// =============================================================================
// h - the clock gating of 22.8
// =============================================================================
void th_gating() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    // 22.8 allows the gating while the bus is SUSPENDED, and letter g
    // shows where that state can be had with no host to ask for it: in
    // the window between the pull-up going up and the host noticing it,
    // the device is on an idle bus and the core suspends after six
    // milliseconds. The gate goes on there.
    Device::stop();
    wait_us(100'000u);
    Device::start();
    const uint32_t t0 = us_now();
    while (!Usb::suspended() && us_now() - t0 < 100'000u) {
    }
    const bool suspended = Usb::suspended();
    const bool phy_awake_before = !Usb::phy_suspended();
    Usb::gate_clocks(true);
    const uint32_t t1 = us_now();
    uint32_t asleep_us = 0;
    while (us_now() - t1 < 10'000u) {
        if (Usb::phy_suspended()) {
            asleep_us = us_now() - t1;
            break;
        }
    }
    Usb::gate_clocks(false);
    print(serial, "  suspended=", suspended, ", PHY awake before the gate=", phy_awake_before,
          ", PHY suspended ", asleep_us, " us after it (0 = never)", crlf);
    bench.verdict("the PHY clock stopped while the bus is suspended, and the core reports the PHY "
                  "suspended",
                  suspended && phy_awake_before && asleep_us != 0u);
    wait_us(1'000u);
    const bool awake = !Usb::phy_suspended();
    const uint32_t took = wait_configured(3'000'000u);
    print(serial, "  ungated: PHY awake=", awake, ", configured in ", took, " us", crlf);
    bench.verdict("the clocks back on and the host enumerates the device through the same window",
                  awake && took != 0u && Port::configured());
}

// =============================================================================
// i - the stall bits
// =============================================================================
void ti_stall() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    const uint32_t before = Usb::stalls();
    Usb::stall(usb_ep_in(2), true);
    const bool in_stalled = Usb::stalled(usb_ep_in(2));
    Usb::stall(usb_ep_out(2), true);
    const bool out_stalled = Usb::stalled(usb_ep_out(2));
    print(serial, "  the bulk pair stalled: IN=", in_stalled, " OUT=", out_stalled, ", counter ",
          before, " -> ", Usb::stalls(), crlf);
    bench.verdict("a stall stands on both directions of a bulk endpoint and is readable back",
                  in_stalled && out_stalled && Usb::stalls() == before + 2u);
    Usb::stall(usb_ep_in(2), false);
    Usb::stall(usb_ep_out(2), false);
    const bool cleared = !Usb::stalled(usb_ep_in(2)) && !Usb::stalled(usb_ep_out(2));
    bench.verdict("clearing it puts both directions back", cleared);

    // Endpoint zero's stall is the one the silicon clears itself, on the
    // next setup token; a host that keeps talking to this port will send
    // one within milliseconds.
    Usb::stall(usb_ep_in(0), true);
    Usb::stall(usb_ep_out(0), true);
    const bool ep0_stalled = Usb::stalled(usb_ep_in(0));
    print(serial, "  endpoint zero stalled: ", ep0_stalled, crlf);
    bench.verdict("endpoint zero takes a stall on both directions", ep0_stalled);
    Usb::stall(usb_ep_in(0), false);
    Usb::stall(usb_ep_out(0), false);

    const uint32_t took = bring_up(3'000'000u);
    print(serial, "  brought up again in ", took, " us", crlf);
    bench.verdict("the port comes back after the poking", took != 0u && Port::configured());
}

// =============================================================================
// y - echo (host-assisted)
// =============================================================================
void ty_echo() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    settle_port();
    print(serial, "  echoing every byte for 10 s: send a stream on the port now", crlf);
    const uint32_t rx0 = Port::rx_bytes();
    const uint32_t tx0 = Port::tx_bytes();
    const uint32_t t0 = us_now();
    uint32_t echoed = 0;
    uint8_t buf[64];
    while (us_now() - t0 < 10'000'000u) {
        uint32_t n = 0;
        while (n < sizeof buf && Port::read_byte(buf[n])) {
            ++n;
        }
        uint32_t written = 0;
        while (written < n) {
            written += Port::write(buf + written, n - written);
        }
        echoed += n;
    }
    while (!Port::tx_idle() && us_now() - t0 < 11'000'000u) {
    }
    print(serial, "  echoed ", echoed, " bytes (received ", Port::rx_bytes() - rx0, ", sent ",
          Port::tx_bytes() - tx0, "), overruns ", Port::rx_overruns(), ", FIFO waits ",
          Usb::fifo_waits(), crlf);
    bench.verdict("every byte received went back, no overrun (the host end judges the bytes)",
                  echoed > 0u && Port::rx_bytes() - rx0 == echoed &&
                      Port::tx_bytes() - tx0 == echoed && Port::rx_overruns() == 0u);
}

// =============================================================================
// w - the line coding and DTR (host-assisted)
// =============================================================================
void tw_line() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    const UsbLineCoding before = Port::line_coding();
    const uint32_t changes = Port::coding_changes();
    const uint32_t states = Port::line_state_changes();
    print(serial, "  now: ", before.baud, "/", before.data_bits, "/", before.parity, "/",
          before.stop_bits, " dtr=", Port::dtr(),
          "; 10 s for the host to open the port at another rate and raise DTR", crlf);
    wait_us(10'000'000u);
    const UsbLineCoding after = Port::line_coding();
    print(serial, "  then: ", after.baud, "/", after.data_bits, "/", after.parity, "/",
          after.stop_bits, " dtr=", Port::dtr(), " rts=", Port::rts(), " coding changes ", changes,
          " -> ", Port::coding_changes(), ", line state changes ", states, " -> ",
          Port::line_state_changes(), crlf);
    bench.verdict("the host's SET_LINE_CODING landed through the control data stage and reads back",
                  Port::coding_changes() > changes);
    bench.verdict("the host's SET_CONTROL_LINE_STATE landed: DTR raised",
                  Port::line_state_changes() > states && Port::dtr());
}

// =============================================================================
// v - throughput (host-assisted)
// =============================================================================
void tv_throughput() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    settle_port();
    print(serial, "  device -> host for 3 s: read the port now", crlf);
    uint8_t pattern[64];
    uint8_t next = 0;
    const uint32_t tx0 = Port::tx_bytes();
    uint32_t t0 = us_now();
    while (us_now() - t0 < 3'000'000u) {
        for (uint8_t i = 0; i < 64u; ++i) {
            pattern[i] = next++;
        }
        uint32_t written = 0;
        while (written < 64u && us_now() - t0 < 3'000'000u) {
            written += Port::write(pattern + written, 64u - written);
        }
    }
    while (!Port::tx_idle() && us_now() - t0 < 4'000'000u) {
    }
    const uint32_t sent = Port::tx_bytes() - tx0;
    print(serial, "  sent ", sent, " bytes in 3 s: ", sent / 3000u, " KB/s", crlf);
    bench.verdict("the device pours more than 100 KB/s into the port", sent > 300'000u);

    print(serial, "  host -> device: write the port now, the two seconds count from the first byte",
          crlf);
    uint8_t b = 0;
    t0 = us_now();
    while (!Port::read_byte(b) && us_now() - t0 < 5'000'000u) {
    }
    const uint32_t rx0 = Port::rx_bytes();
    uint32_t drained = 0;
    t0 = us_now();
    while (us_now() - t0 < 2'000'000u) {
        if (Port::read_byte(b)) {
            ++drained;
        }
    }
    const uint32_t got = Port::rx_bytes() - rx0;
    print(serial, "  received ", got, " bytes in 2 s: ", got / 2000u, " KB/s, drained ", drained,
          ", overruns ", Port::rx_overruns(), crlf);
    bench.verdict("the device takes more than 100 KB/s from the port with no overrun",
                  got > 200'000u && Port::rx_overruns() == 0u);
}

// =============================================================================
// x - NAK flow control (host-assisted)
// =============================================================================
void tx_nak() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    settle_port();
    print(serial, "  send a counting pattern (0,1,2,... modulo 256) now: the device reads nothing "
                  "for 2 s, then drains for 8 s",
          crlf);
    // Nothing is read for two seconds: the receive ring fills, the class
    // stops re-arming the endpoint, and the host is NAKed - USB's own
    // flow control, which costs no byte.
    const uint32_t rx0 = Port::rx_bytes();
    uint32_t t0 = us_now();
    while (us_now() - t0 < 2'000'000u) {
    }
    const bool armed_none = Usb::out_armed(2) == 0u;
    const uint32_t held = Port::rx_bytes() - rx0;
    print(serial, "  after the 2 s pause: ", held, " bytes taken in of a ", Port::rx_capacity(),
          "-byte ring, the endpoint holds ", Usb::out_armed(2), " packets armed, overruns ",
          Port::rx_overruns(), crlf);

    uint8_t b = 0;
    bool first = true;
    uint8_t expected = 0;
    uint32_t got = 0;
    uint32_t breaks = 0;
    t0 = us_now();
    uint32_t last_byte_us = t0;
    while (us_now() - t0 < 8'000'000u) {
        if (!Port::read_byte(b)) {
            if (us_now() - last_byte_us > 1'000'000u && got != 0u) {
                break;   // the host has stopped and the ring is empty
            }
            continue;
        }
        last_byte_us = us_now();
        if (first) {
            first = false;
        } else if (b != expected) {
            ++breaks;
        }
        expected = static_cast<uint8_t>(b + 1u);
        ++got;
    }
    print(serial, "  drained ", got, " bytes, ", breaks, " breaks in the counting pattern, "
          "overruns ", Port::rx_overruns(), ", receive entries ", Usb::rx_entries(), crlf);
    bench.verdict("with nobody reading, the receive ring took one ringful and no more and the "
                  "endpoint ran out of armed packets - the host saw NAKs from there on",
                  armed_none && held >= Port::rx_capacity() - 8u * 64u &&
                      held <= Port::rx_capacity());
    bench.verdict("every byte followed the last through the NAK window: a NAK costs no byte",
                  got > 0u && breaks == 0u && Port::rx_overruns() == 0u);
}

// =============================================================================
// t - the zero-length packet that closes a run of full ones (host-assisted)
// =============================================================================
void tt_zlp() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    settle_port();
    constexpr uint32_t run = 1024;   // sixteen whole packets and not a byte more
    static_assert(run % 64u == 0u, "the point of this letter is a whole number of full packets");
    print(serial, "  pouring ", run, " bytes - a whole number of full packets - into the port", crlf);
    const uint32_t tx0 = Port::tx_bytes();
    uint8_t buf[64];
    uint8_t next = 0;
    const uint32_t t0 = us_now();
    uint32_t sent = 0;
    while (sent < run && us_now() - t0 < 3'000'000u) {
        const uint32_t want = run - sent < sizeof buf ? run - sent : sizeof buf;
        for (uint32_t i = 0; i < want; ++i) {
            buf[i] = next++;
        }
        uint32_t written = 0;
        while (written < want && us_now() - t0 < 3'000'000u) {
            written += Port::write(buf + written, want - written);
        }
        sent += written;
    }
    while (!Port::tx_idle() && us_now() - t0 < 4'000'000u) {
    }
    const uint32_t took = us_now() - t0;
    print(serial, "  sent ", Port::tx_bytes() - tx0, " bytes in ", took,
          " us, transmitter idle=", Port::tx_idle(), crlf);
    bench.verdict("the whole run went out and the transmitter went idle - the zero-length packet "
                  "after the last full one is what lets the host's read return (the host end "
                  "judges that it did)",
                  Port::tx_bytes() - tx0 == run && Port::tx_idle());
}

void banner() {
    print(serial, crlf,
          "test_stm32f4_usb - the STM32F4 OTG FS device controller under the CDC ACM stack; the "
          "port on the host is the other end; clk=",
          SysClock::hz, " Hz, usb=", SysClock::usb_hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void OTG_FS_IRQHandler() { Device::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output(true);   // this LED is lit when low
    brio::enable_interrupts();

    // The port is brought up at boot so the host has something to open
    // before any letter runs; letter a takes the controller down and
    // brings it back, so it measures the same bring-up from nothing.
    usb_up = Usb::init(clock);
    if (usb_up) {
        Device::start();
    }

    bench.letter('a', "the controller, and the host enumerating the device", ta_enumerate);
    bench.letter('b', "the FIFO budget and the claims that are refused", tb_budget);
    bench.letter('c', "the core's state registers", tc_state);
    bench.letter('d', "the control endpoint's tally", td_control);
    bench.letter('e', "the start-of-frame counter", te_frames);
    bench.letter('f', "a reconnect through the soft disconnect", tf_reconnect);
    bench.letter('g', "suspend and resume", tg_suspend);
    bench.letter('h', "the PHY clock gated while suspended", th_gating);
    bench.letter('i', "the stall bits", ti_stall);
    bench.letter('y', "echo for 10 s (host-assisted)", ty_echo, false);
    bench.letter('w', "the line coding and DTR from the host (host-assisted)", tw_line, false);
    bench.letter('v', "throughput both ways (host-assisted)", tv_throughput, false);
    bench.letter('x', "NAK flow control (host-assisted)", tx_nak, false);
    bench.letter('t', "the zero-length packet closing a full run (host-assisted)", tt_zlp, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL96" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", " usb=", usb_up ? "up" : "FAILED",
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
