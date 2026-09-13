// test_rp2040_usb - the reference bench suite for the RP2040's USB
// device controller (rp2040/usb.hpp over datasheet 4.1) under the
// device stack (util/usb/device.hpp) and the CDC ACM class
// (util/usb/cdc.hpp): enumeration against a real host, the port as a
// byte transport, the class requests, the bus reset.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// THE CONSOLE IS THE PROBE'S BRIDGE on UART0; the USB connector goes to
// the host, which is the other end of every measure: Linux enumerates
// the device (the pid.codes test identity 1209:0001), binds cdc-acm,
// and the host letters talk to /dev/ttyACM* with pyserial.
//
// What is exercised, letter by letter:
//   a  the controller and the enumeration, no host action needed: clk_usb
//      counted at 48 MHz, VBUS seen, the pull-up raised, then the host
//      does what a host does - the bus reset, the descriptors, the
//      address, the configuration - and the suite watches the state
//      reach CONFIGURED within a second, the address non-zero, the
//      setups counted, the class's endpoints armed
//
//   y  (HOST-ASSISTED, outside the all-key) ECHO: for ten seconds every
//      byte the host sends on the port comes back; the host end checks
//      the echo byte-exact and the letter reports the counts
//   w  (host-assisted) THE LINE CODING AND DTR: the values as they
//      stand, ten seconds for the host to change them, the values
//      again; the verdict is that they moved
//   v  (host-assisted) THROUGHPUT: three seconds of the device pouring
//      a counting pattern into the port as fast as the host takes it,
//      then three seconds counting what the host pours in
//   u  A RECONNECT, no host action needed: the pull-up dropped for half
//      a second and raised again - the host's bus reset counted, the
//      device enumerated afresh and configured
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2040/clock.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "rp2040/usb.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

namespace {

using namespace brio;

using P = Rp2040Platform<>;
using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Led = Pin<25>;

TestBench<Serial, 16> bench;

using Port = UsbCdcAcm<Usb, P, 0, 1, 2, 2048, 512>;

struct Descriptors {
    static constexpr auto device = usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        usb_concat(usb_configuration_head(9 + Port::descriptor_bytes, Port::interface_count), Port::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static constexpr auto manufacturer = usb_string_descriptor("brio");
    static constexpr auto product = usb_string_descriptor("test_rp2040_usb");
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

uint32_t us_now() { return Timer::now_low(); }
bool within(uint32_t got, uint32_t want, uint32_t per_mille) {
    const uint32_t tol = static_cast<uint32_t>(static_cast<uint64_t>(want) * per_mille / 1000u);
    return got + tol >= want && got <= want + tol;
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

/// A host letter's clean start: whatever a previous letter left in the
/// rings is dropped (the receive ring drained, the transmit ring waited
/// out for a second), so the counts are this letter's own.
void settle_port() {
    uint8_t b = 0;
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 1'000'000u) {
        if (!Port::read_byte(b) && Port::tx_idle()) {
            break;
        }
    }
}

/// Wait for the device to reach CONFIGURED; the microseconds it took, 0 on a timeout.
uint32_t wait_configured(uint32_t budget_us) {
    const uint32_t t0 = us_now();
    while (!Device::configured()) {
        if (us_now() - t0 > budget_us) {
            return 0;
        }
    }
    return us_now() - t0;
}

bool usb_up = false;

// =============================================================================
// a - the controller and the enumeration
// =============================================================================
void ta_enumerate() {
    Device::stop();
    Usb::release();
    const uint32_t t0 = us_now();
    usb_up = Usb::init(clock);
    const uint32_t init_us = us_now() - t0;
    const auto usb_hz = SysClock::count_hz(CountSource::clk_usb);
    print(serial, "  init: ", usb_up ? "ok" : "FAILED", " in ", init_us, " us; clk_usb counted ", usb_hz ? *usb_hz : 0u, " Hz, USB PLL locked=",
          PllUsb::locked(), ", VBUS detected=", Usb::vbus_detected(), ", connected=", Usb::connected(), ", pulled up=", Usb::pulled_up(), crlf);
    bench.verdict("the controller initializes with clk_usb counted at 48 MHz from the USB PLL, VBUS seen (forced: the boards wire no VBUS "
                  "pin), the pull-up still down",
                  usb_up && usb_hz && within(*usb_hz, usb_clk_hz, 1) && Usb::vbus_detected() && !Usb::pulled_up());
    bench.verdict("the descriptors: the device's 18 bytes, the configuration's 67 (two interfaces, three endpoints), four strings",
                  Descriptors::device.size() == 18u && Descriptors::configuration.size() == 67u && Descriptors::configuration[4] == 2u &&
                      Descriptors::string(3).size() == 12u && Descriptors::string(4).empty());

    const uint32_t stalls_before = Device::stalls();
    const uint32_t setups_before = Device::setups();
    Device::start();
    const uint32_t took = wait_configured(1'500'000u);
    print(serial, "  start(): pull-up=", Usb::pulled_up(), "; the host took ", took, " us to configure: state=", state_name(Device::state()),
          " address=", Device::address(), " resets=", Device::resets(), " setups=", Device::setups(), " stalls=", Device::stalls(),
          " last request=", Device::last_request(), " frame=", Usb::frame(), crlf);
    bench.verdict("start() raises the pull-up and the host enumerates the device to CONFIGURED within a second and a half",
                  Usb::pulled_up() && took != 0u && Device::configured());
    bench.verdict("the host reset the bus at least once and assigned a non-zero address", Device::resets() >= 1u && Device::address() != 0u);
    bench.verdict("enumeration took at least the four requests a host must make, and the stall counter is small (a device qualifier or "
                  "a string the device has not are stalled by the rules)",
                  Device::setups() - setups_before >= 4u && Device::stalls() - stalls_before <= 8u);
    bench.verdict("the port is configured with its bulk OUT armed and nothing in flight",
                  Port::configured() && Port::out_armed() && !Port::in_armed() && Port::tx_idle());
    bench.verdict("the controller handed out the notification's buffer, the bulk OUT's two halves and the bulk IN's past endpoint zero's",
                  Usb::buffers_used() == Usb::buffers_start + 4u * 64u);
    print(serial, "  line coding as found: ", Port::line_coding().baud, "/", Port::line_coding().data_bits, "/", Port::line_coding().parity, "/",
          Port::line_coding().stop_bits, " dtr=", Port::dtr(), " coding changes=", Port::coding_changes(), " frames seen=", Usb::frame(), crlf);
    // Ten milliseconds of frames: the host's start-of-frame counter moves.
    const uint16_t f0 = Usb::frame();
    const uint32_t t1 = us_now();
    while (us_now() - t1 < 10'000u) {
    }
    const uint16_t frames = static_cast<uint16_t>((Usb::frame() - f0) & 0x7FFu);
    bench.verdict("the host's start-of-frame counter advances a frame a millisecond", frames >= 9u && frames <= 11u);
    const uint32_t errors = Usb::take_errors();
    print(serial, "  SIE errors during enumeration: ", hex(errors), crlf);
    bench.verdict("no CRC, bit-stuff, overflow, timeout or data-sequence error", errors == 0u);
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
    print(serial, "  echoed ", echoed, " bytes (received ", Port::rx_bytes() - rx0, ", sent ", Port::tx_bytes() - tx0, "), overruns ",
          Port::rx_overruns(), ", SIE errors ", hex(Usb::take_errors()), crlf);
    bench.verdict("every byte received went back, no overrun (the host end judges the bytes)",
                  echoed > 0u && Port::rx_bytes() - rx0 == echoed && Port::tx_bytes() - tx0 == echoed && Port::rx_overruns() == 0u);
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
    print(serial, "  now: ", before.baud, "/", before.data_bits, "/", before.parity, "/", before.stop_bits, " dtr=", Port::dtr(),
          "; 10 s for the host to open the port at another rate and raise DTR", crlf);
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 10'000'000u) {
    }
    const UsbLineCoding after = Port::line_coding();
    print(serial, "  then: ", after.baud, "/", after.data_bits, "/", after.parity, "/", after.stop_bits, " dtr=", Port::dtr(), " rts=", Port::rts(),
          " coding changes ", changes, " -> ", Port::coding_changes(), ", line state changes ", states, " -> ", Port::line_state_changes(), crlf);
    bench.verdict("the host's SET_LINE_CODING landed through the control data stage and reads back", Port::coding_changes() > changes);
    bench.verdict("the host's SET_CONTROL_LINE_STATE landed: DTR raised", Port::line_state_changes() > states && Port::dtr());
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
    bench.verdict("the device pours more than 200 KB/s into the port", sent > 600'000u);

    print(serial, "  host -> device: write the port now, the two seconds count from the first byte", crlf);
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
    print(serial, "  received ", got, " bytes in 2 s: ", got / 2000u, " KB/s, drained ", drained, ", overruns ", Port::rx_overruns(), crlf);
    bench.verdict("the device takes more than 200 KB/s from the port with no overrun", got > 400'000u && Port::rx_overruns() == 0u);
}

// =============================================================================
// u - a reconnect: the pull-up dropped and raised, the host enumerates again
// =============================================================================
void tu_reconnect() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    const uint32_t resets = Device::resets();
    const uint8_t address = Device::address();
    Device::stop();
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 500'000u) {
    }
    print(serial, "  stopped for 500 ms: state=", state_name(Device::state()), "; start() again...", crlf);
    Device::start();
    const uint32_t took = wait_configured(2'000'000u);
    print(serial, "  resets ", resets, " -> ", Device::resets(), ", address ", address, " -> ", Device::address(), ", configured in ", took, " us, state=",
          state_name(Device::state()), crlf);
    bench.verdict("dropping the pull-up detaches the device; raising it again brings the host's bus reset (counted) and a fresh enumeration",
                  Device::resets() > resets && took != 0u);
    bench.verdict("the port is configured again with its bulk OUT armed", Port::configured() && Port::out_armed());
}

void banner() {
    print(serial, crlf, "test_rp2040_usb - the RP2040 USB device controller under the CDC ACM stack; the port on the host is the other end; clk_sys=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_usbctrl() { Device::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the controller, and the host enumerating the device", ta_enumerate);
    bench.letter('y', "echo for 10 s (host-assisted)", ty_echo, false);
    bench.letter('w', "the line coding and DTR from the host (host-assisted)", tw_line, false);
    bench.letter('v', "throughput both ways (host-assisted)", tv_throughput, false);
    bench.letter('u', "a reconnect: the pull-up dropped and raised", tu_reconnect);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
