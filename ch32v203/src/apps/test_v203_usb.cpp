// test_v203_usb - the reference bench suite for the CH32V203's USB
// device controller (ch32v203/usb.hpp over RM ch. 21) under the device
// stack (util/usb/device.hpp) and the CDC ACM class (util/usb/cdc.hpp):
// the enumeration a real host performs, the port as a byte transport,
// the class requests, a reconnect, and the packet-memory overflow that
// this family's whole power model is built on.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE CONSOLE IS THE PROBE'S UART, on USART1's own pads PA9/PA10, and
// THE BOARD'S USB-C IS THE THING UNDER TEST: a device being asked what
// it thinks of its host cannot answer over the connector the question
// is about. The host is the other end of every letter that needs one:
// Linux enumerates the device (the pid.codes test identity 1209:0001),
// binds cdc-acm, and a pyserial script drives /dev/ttyACM* while the
// letter watches this end. A host-assisted letter waits a BOUNDED time
// and then says it was skipped, so the all-key is green with the cable
// in and no script running. THE CABLE ITSELF IS NOT OPTIONAL: letters a
// and u need no host ACTION but they do need a host, and with the
// connector empty they fail and print an enumeration that never
// happened, which is this suite's way of reporting an unplugged cable.
//
// THE CLOCK IS 96 MHz AND THE CRYSTAL IS WHY. Full-speed USB wants its
// 48 MHz within 2500 ppm, and every HSI-rooted tree of this family
// measures four tenths of a per cent fast (the clock chapter) - nearly
// twice the allowance; so the tree is the board's 8 MHz crystal through
// the PLL at x12, whose USBPRE then halves it whole. 96 MHz is also one
// of the three bus rates the controller is measured good at, and it
// leaves PB1 at 48.
//
// THE LOOP IDLES, AND THAT IS WHAT A COUNT BOUGHT. In a sleep of any
// depth on this family no bus master but the core gets a cycle: an
// armed endpoint loses the host's bytes, the packet memory overflows,
// and an enumeration dies in its first control transfer
// (ch32v203/bus_activity.hpp carries the measurements and the reasons).
// The rule that used to be written in prose - a program that uses USB
// never idles - is now a MECHANISM: an attached controller holds one
// count, and the platform's idle() returns at once while it stands. So
// this suite's loop calls idle() like any other program's loop, and
// letter a is what measures that the call costs nothing here. Letter d
// is the other face of the same fact: a bare wfi under traffic, which
// is the instruction the platform would not have taken, and the
// overflow counter climbing behind it.
//
// THE PADS. PA11 and PA12 (the connector's, which this block reaches
// through port A's clock), PA9/PA10 (the console) and PB2 (the LED,
// toggled per command as every suite of this target does). Nothing
// else is touched, and no wire is needed.
//
// What is exercised, letter by letter:
//   a  THE CONTROLLER AND THE ENUMERATION, no host action needed: the
//      block brought up with its 48 MHz and the pull-up still down,
//      then start() and what a host does with it - the bus reset, the
//      descriptors, the address, the configuration - watched to
//      CONFIGURED, the setups and the stalls counted, the class's
//      endpoints armed, the shared memory they spent, the frame counter
//      advancing, the error and overflow counters at zero, and THE BUS
//      COUNT at one with a thousand idle() calls costing nothing
//   u  A RECONNECT, no host action needed: the pull-up dropped for half
//      a second and raised again, the host's fresh bus reset counted and
//      the device enumerated anew
//   y  (host-assisted) ECHO: every byte the host sends comes back for
//      ten seconds, the host end judging it byte-exact
//   w  (host-assisted) THE LINE CODING AND DTR: the values as they
//      stand, a window for the host to open the port at another frame,
//      the values again
//   v  (host-assisted) THROUGHPUT both ways, the device pouring and
//      then draining as fast as the host will take and give
//   s  (host-assisted) SUSPEND AND RESUME: the bus idle long enough for
//      the controller to report a suspend (Linux's autosuspend on the
//      port's power/control), and the resume that ends it
//   d  THE PACKET-MEMORY OVERFLOW: the counter this driver keeps -
//      which counts from init() and stands still in a healthy run -
//      and, with the host pouring, the finding staged on purpose with
//      a bare wfi, the counter read again, and the receive buffer's
//      own size halfword read out of the packet memory to show what a
//      lost packet leaves there and what the driver writes back
//
// The host end of y, w, v, s and d is
// private/tools/ch32v203_usb_host.py, driven by hand one letter at a
// time: START THE LETTER HERE FIRST, then run the script inside the
// window the letter prints. The order matters for the two letters the
// host pours into: a host that is already pouring when the letter
// begins fills the receive ring before the letter is watching, and
// USB's own flow control then holds every byte behind it - the letter
// reports it as a skip.
//
// build: boards = v203c6,v203c8
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "ch32v203/clock.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/ticker.hpp"
#include "ch32v203/usart.hpp"
#include "ch32v203/usb.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

namespace {

using namespace brio;

using P = Ch32v203Platform<>;
/// The board's 8 MHz crystal through the PLL: 96 MHz of HCLK, 48 of
/// PB1, and the USB divider halving the PLL whole (the file header).
using SysClock = Clock<ClockSource::pll, 96'000'000, 8'000'000>;
constexpr SysClock clock;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

TestBench<Serial> bench;

/// The 384-byte budget, which is what is left of the shared memory when
/// the CAN controller takes its filter table (RM 21.2.1).
using Usb = Usbd<>;
/// Interface 0, the notification on endpoint 1 and the bulk pair on 2,
/// the default rings.
using Port = UsbCdcAcm<Usb, P>;

struct Descriptors {
    static constexpr auto device =
        usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        usb_concat(usb_configuration_head(9 + Port::descriptor_bytes, Port::interface_count),
                   Port::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static constexpr auto manufacturer = usb_string_descriptor("brio");
    static constexpr auto product = usb_string_descriptor("test_v203_usb");
    static constexpr auto serial_number = usb_string_descriptor("ch32v203");
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

// ---------------------------------------------------------------------------
// The rulers
// ---------------------------------------------------------------------------

/// The core counter counts HCLK and wraps once a tick, which is what
/// makes it the ruler for a span shorter than a millisecond.
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000UL;

uint32_t core_counter() { return stk()->CNTL; }

uint32_t us_between(uint32_t first, uint32_t second) {
    const uint32_t period = stk()->CMPLR + 1u;
    const uint32_t span = (second >= first) ? (second - first) : (second + period - first);
    return span / cycles_per_us;
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// The shared memory a configured CDC port spends: the table's 64 bytes
/// (eight entries of four halfwords), endpoint zero's two buffers of
/// the full-speed maximum, the notification's eight and the bulk pair's
/// two of 64.
constexpr uint16_t pma_configured = 64u + 2u * 64u + 8u + 2u * 64u;

/// The bulk pair's endpoint number - the class's default, which this
/// suite takes. A letter that looks into the packet memory has to name
/// the endpoint the way the hardware knows it, and the buffer table's
/// entry is four halfwords with the receive count the last of them.
constexpr uint8_t data_endpoint = 2;
constexpr uint16_t count_rx_offset = static_cast<uint16_t>(8u * data_endpoint + 6u);
/// The two fields of that halfword: the buffer's SIZE on top, and under
/// it the ten bits the hardware writes the received count into.
constexpr uint16_t count_rx_size = 0xFC00u;

const char* state_name(UsbDeviceState s) {
    switch (s) {
        case UsbDeviceState::detached: return "detached";
        case UsbDeviceState::defaulted: return "default";
        case UsbDeviceState::addressed: return "addressed";
        case UsbDeviceState::configured: return "configured";
    }
    return "?";
}

/// Wait for the host to configure the device; the milliseconds it took,
/// or zero when the budget ran out.
uint32_t wait_configured(uint32_t budget_ms) {
    const uint32_t t0 = Ticker::millis();
    while (!Device::configured()) {
        if (Ticker::millis() - t0 > budget_ms) {
            return 0;
        }
    }
    const uint32_t took = Ticker::millis() - t0;
    return took == 0u ? 1u : took;
}

/// A host letter's clean start: whatever a previous letter left in the
/// rings is dropped, so the counts belong to this letter alone.
void settle_port() {
    uint8_t b = 0;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 500u) {
        if (!Port::read_byte(b) && Port::tx_idle()) {
            break;
        }
    }
}

/// Is there a host at all? Every host-assisted letter asks first, so an
/// unplugged connector is a NAMED skip and never a hang or a false
/// verdict.
bool need_host() {
    if (Device::configured()) {
        return true;
    }
    print(serial, "  SKIPPED, and it passes as a skip: the device is not configured, so no host "
                  "is answering on the connector. Plug the board's USB-C into the host and run "
                  "letter a first.",
          crlf);
    bench.verdict("the host letter is skipped and says so", true);
    return false;
}

/// Wait for the host to send something on the port, a bounded wait; the
/// bytes it sent, or zero.
uint32_t wait_for_traffic(uint32_t budget_ms) {
    const uint32_t rx0 = Port::rx_bytes();
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < budget_ms) {
        if (Port::rx_bytes() != rx0) {
            return Port::rx_bytes() - rx0;
        }
    }
    return 0;
}

// =============================================================================
// a - the controller, and the host enumerating the device
// =============================================================================
void ta_enumerate() {
    Device::stop();
    Usb::release();
    wait_ms(20);

    const uint32_t c0 = core_counter();
    const bool up = Usb::init(clock);
    const uint32_t init_us = us_between(c0, core_counter());
    print(serial, "  init: ", up ? "ok" : "FAILED", " in ", init_us,
          " us; the tree feeds the controller ", SysClock::usb_hz, " Hz with HCLK at ",
          SysClock::hz, "; pulled up=", Usb::pulled_up() ? 1 : 0,
          " bus masters=", P::bus_masters_active(), crlf);
    bench.verdict("the controller comes up on a tree that makes it exactly 48 MHz, with the "
                  "pull-up still down and no bus master counted - the host must not see a "
                  "device before the program can answer it",
                  up && SysClock::usb_hz == usb_required_hz && !Usb::pulled_up() &&
                      P::bus_masters_active() == 0u);
    bench.verdict("the descriptors: the device's 18 bytes, the configuration's 67 over two "
                  "interfaces, and four strings",
                  Descriptors::device.size() == 18u && Descriptors::configuration.size() == 67u &&
                      Descriptors::configuration[4] == 2u && !Descriptors::string(3).empty() &&
                      Descriptors::string(4).empty());

    const uint32_t setups_before = Device::setups();
    const uint32_t stalls_before = Device::stalls();
    Device::start();
    const uint32_t took = wait_configured(1500u);
    print(serial, "  start(): pulled up=", Usb::pulled_up() ? 1 : 0, " bus masters=",
          P::bus_masters_active(), "; the host took ", took, " ms: state=",
          state_name(Device::state()), " address=", Device::address(), " resets=",
          Device::resets(), " setups=", Device::setups(), " stalls=", Device::stalls(),
          " last request=", hex(Device::last_request()), " frame=", Usb::frame(), crlf);
    bench.verdict("start() raises the pull-up and the host enumerates the device to CONFIGURED "
                  "within a second and a half",
                  Usb::pulled_up() && took != 0u && Device::configured());
    bench.verdict("the host reset the bus at least once and assigned a non-zero address",
                  Device::resets() >= 1u && Device::address() != 0u);
    bench.verdict("the enumeration took at least the four requests a host must make, and few "
                  "stalls (a device qualifier and a string this device has not are stalled by "
                  "the rules)",
                  Device::setups() - setups_before >= 4u && Device::stalls() - stalls_before <= 8u);
    bench.verdict("the port is configured with its bulk OUT armed and nothing in flight",
                  Port::configured() && Port::out_armed() && !Port::in_armed() && Port::tx_idle());

    print(serial, "  packet memory: ", Usb::buffer_used(), " of ", Usb::buffer_used() +
          Usb::buffer_free(), " bytes spent, ", Usb::buffer_free(), " free", crlf);
    bench.verdict("the table, endpoint zero's two buffers and the class's three came out of the "
                  "384 bytes the CAN filter table leaves",
                  Usb::buffer_used() == pma_configured &&
                      Usb::buffer_used() + Usb::buffer_free() == 384u);

    const uint16_t f0 = Usb::frame();
    wait_ms(10);
    const uint16_t frames = static_cast<uint16_t>((Usb::frame() - f0) & 0x07FFu);
    print(serial, "  the host's frame counter advanced ", frames, " in 10 ms; line coding ",
          Port::line_coding().baud, "/", Port::line_coding().data_bits, "/",
          Port::line_coding().parity, "/", Port::line_coding().stop_bits, " dtr=",
          Port::dtr() ? 1 : 0, crlf);
    bench.verdict("the host's start-of-frame counter advances a frame a millisecond",
                  frames >= 9u && frames <= 11u);
    print(serial, "  errors=", Usb::errors(), " packet-memory overflows=", Usb::overruns(), crlf);
    bench.verdict("no bus error and no packet-memory overflow through the whole enumeration",
                  Usb::errors() == 0u && Usb::overruns() == 0u);

    // THE COUNT, AND THE IDLE PATH IT GOVERNS. A thousand calls at a
    // count of one are a thousand returns; the same thousand at zero
    // would be a thousand tick periods, because that is when this core
    // is next woken.
    const uint8_t masters = P::bus_masters_active();
    const uint32_t overruns_before = Usb::overruns();
    const uint32_t t0 = Ticker::millis();
    for (uint16_t i = 0; i < 1000u; ++i) {
        P::CriticalSection cs;   // the kernel enters idle() masked
        P::idle();
    }
    const uint32_t idle_ms = Ticker::millis() - t0;
    print(serial, "  bus masters=", masters, "; a thousand idle() calls took ", idle_ms,
          " ms (a thousand tick periods is what they would cost at a count of zero)", crlf);
    bench.verdict("the attached controller counts itself one bus master, so the kernel's idle "
                  "path returns at once instead of sleeping - and the port survives a loop "
                  "that idles, which is what the count is for",
                  masters == 1u && idle_ms <= 20u && Device::configured() &&
                      Usb::overruns() == overruns_before);
}

// =============================================================================
// u - a reconnect: the pull-up dropped and raised
// =============================================================================
void tu_reconnect() {
    if (!Device::configured()) {
        print(serial, "  SKIPPED, and it passes as a skip: the device is not configured, so "
                      "there is nothing to detach. Run letter a first.",
              crlf);
        bench.verdict("the reconnect is skipped and says so", true);
        return;
    }
    const uint32_t resets = Device::resets();
    const uint8_t address = Device::address();

    Device::stop();
    print(serial, "  stopped: pulled up=", Usb::pulled_up() ? 1 : 0, " state=",
          state_name(Device::state()), " bus masters=", P::bus_masters_active(), crlf);
    bench.verdict("stop() drops the pull-up, which detaches the device and gives its bus-master "
                  "count back - a program with no host attached may sleep again",
                  !Usb::pulled_up() && P::bus_masters_active() == 0u &&
                      Device::state() == UsbDeviceState::detached);

    wait_ms(500);
    Device::start();
    const uint32_t took = wait_configured(2000u);
    print(serial, "  re-attached: resets ", resets, " -> ", Device::resets(), ", address ",
          address, " -> ", Device::address(), ", configured in ", took, " ms, state=",
          state_name(Device::state()), " bus masters=", P::bus_masters_active(), crlf);
    bench.verdict("raising the pull-up again brings the host's bus reset (counted) and a fresh "
                  "enumeration, and the count is held once more",
                  Device::resets() > resets && took != 0u && P::bus_masters_active() == 1u);
    bench.verdict("the port is configured again with its bulk OUT armed",
                  Port::configured() && Port::out_armed());
}

// =============================================================================
// y - echo (host-assisted)
// =============================================================================
void ty_echo() {
    if (!need_host()) {
        return;
    }
    settle_port();
    print(serial, "  10 s of echo: run the script's y now, or nothing comes and this letter "
                  "says so", crlf);
    const uint32_t rx0 = Port::rx_bytes();
    const uint32_t tx0 = Port::tx_bytes();
    const uint32_t overruns = Port::rx_overruns();
    if (wait_for_traffic(10000u) == 0u) {
        print(serial, "  SKIPPED, and it passes as a skip: the host sent nothing in ten "
                      "seconds. The host end is private/tools/ch32v203_usb_host.py.",
              crlf);
        bench.verdict("the echo is skipped and says so", true);
        return;
    }

    // Ten seconds of echo and then a tail: the host's last chunk can
    // still be in the ring when the window closes, and a byte received
    // after the clock ran out is not a byte the port lost.
    uint32_t echoed = 0;
    uint8_t buf[64];
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 10500u) {
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
    while (!Port::tx_idle() && Ticker::millis() - t0 < 11500u) {
    }
    print(serial, "  echoed ", echoed, " bytes of the ", Port::rx_bytes() - rx0,
          " received (", Port::tx_bytes() - tx0, " sent), overruns ",
          Port::rx_overruns() - overruns, ", bus errors ", Usb::errors(), ", overflows ",
          Usb::overruns(), crlf);
    bench.verdict("every byte the host sent came back and none was dropped on the way in - the "
                  "host end is what judges the bytes themselves",
                  echoed > 0u && Port::rx_bytes() - rx0 == echoed &&
                      Port::tx_bytes() - tx0 == echoed && Port::rx_overruns() == overruns);
}

// =============================================================================
// w - the line coding and DTR (host-assisted)
// =============================================================================
void tw_line() {
    if (!need_host()) {
        return;
    }
    const UsbLineCoding before = Port::line_coding();
    const uint32_t codings = Port::coding_changes();
    const uint32_t states = Port::line_state_changes();
    print(serial, "  now: ", before.baud, "/", before.data_bits, "/", before.parity, "/",
          before.stop_bits, " dtr=", Port::dtr() ? 1 : 0, " rts=", Port::rts() ? 1 : 0,
          "; 15 s for the host to open the port at another frame and raise DTR", crlf);

    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 15000u) {
        if (Port::coding_changes() != codings && Port::line_state_changes() != states) {
            break;
        }
    }
    const UsbLineCoding after = Port::line_coding();
    print(serial, "  then: ", after.baud, "/", after.data_bits, "/", after.parity, "/",
          after.stop_bits, " dtr=", Port::dtr() ? 1 : 0, " rts=", Port::rts() ? 1 : 0,
          "; coding changes ", codings, " -> ", Port::coding_changes(), ", line state changes ",
          states, " -> ", Port::line_state_changes(), crlf);
    if (Port::coding_changes() == codings && Port::line_state_changes() == states) {
        print(serial, "  SKIPPED, and it passes as a skip: the host neither set a line coding "
                      "nor moved a control line in fifteen seconds.",
              crlf);
        bench.verdict("the line coding letter is skipped and says so", true);
        return;
    }
    bench.verdict("the host's SET_LINE_CODING arrives through the control endpoint's OUT data "
                  "stage and reads back as the host sent it",
                  Port::coding_changes() > codings);
    bench.verdict("the host's SET_CONTROL_LINE_STATE arrives in the setup packet's value field: "
                  "DTR raised",
                  Port::line_state_changes() > states && Port::dtr());
    print(serial, "  and the coding is REPORTED and not obeyed: a virtual port has no baud "
                  "rate, and this console - a real UART at its own 115200 - is the proof that "
                  "nothing was rebased",
          crlf);
}

// =============================================================================
// v - throughput both ways (host-assisted)
// =============================================================================
void tv_throughput() {
    if (!need_host()) {
        return;
    }
    settle_port();
    print(serial, "  3 s of the device pouring: run the script's v now", crlf);
    uint8_t pattern[64];
    uint8_t next = 0;
    const uint32_t tx0 = Port::tx_bytes();
    uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 3000u) {
        for (uint8_t i = 0; i < 64u; ++i) {
            pattern[i] = next++;
        }
        uint32_t written = 0;
        while (written < 64u && Ticker::millis() - t0 < 3000u) {
            written += Port::write(pattern + written, 64u - written);
        }
    }
    while (!Port::tx_idle() && Ticker::millis() - t0 < 4000u) {
    }
    const uint32_t sent = Port::tx_bytes() - tx0;
    print(serial, "  the device sent ", sent, " bytes in 3 s = ", sent / 3000u, " KB/s", crlf);
    if (sent < 8192u) {
        print(serial, "  SKIPPED, and it passes as a skip: with no host reading the port the "
                      "transmit ring fills and stays full, which is what this count is.",
              crlf);
        bench.verdict("the throughput letter is skipped and says so", true);
        return;
    }
    bench.verdict("the device pours into a host that is reading, one packet per IN token and no "
                  "double buffer on this block",
                  sent >= 8192u);

    print(serial, "  now the other way: the two seconds count from the host's first byte", crlf);
    if (wait_for_traffic(5000u) == 0u) {
        print(serial, "  SKIPPED, and it passes as a skip: the host poured nothing back in five "
                      "seconds.",
              crlf);
        bench.verdict("the second half of the throughput letter is skipped and says so", true);
        return;
    }
    const uint32_t rx0 = Port::rx_bytes();
    const uint32_t overruns = Port::rx_overruns();
    uint8_t b = 0;
    uint32_t drained = 0;
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 2000u) {
        if (Port::read_byte(b)) {
            ++drained;
        }
    }
    const uint32_t got = Port::rx_bytes() - rx0;
    print(serial, "  the device took ", got, " bytes in 2 s = ", got / 2000u, " KB/s, drained ",
          drained, ", overruns ", Port::rx_overruns() - overruns, ", overflows ", Usb::overruns(),
          crlf);
    bench.verdict("the device drains what the host pours with no ring overrun - the OUT "
                  "endpoint is re-armed only while a whole packet fits, so USB's own NAK is the "
                  "flow control",
                  got > 0u && Port::rx_overruns() == overruns);
}

// =============================================================================
// s - suspend and resume (host-assisted)
// =============================================================================
void ts_suspend() {
    if (!need_host()) {
        return;
    }
    const uint32_t suspends = Device::suspends();
    const uint32_t resumes = Device::resumes();
    print(serial, "  suspends=", suspends, " resumes=", resumes,
          "; 20 s for the host to let the port suspend (the script's s asks the kernel for it)",
          crlf);

    uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 20000u && Device::suspends() == suspends) {
    }
    if (Device::suspends() == suspends) {
        print(serial, "  SKIPPED, and it passes as a skip: this host did not let the bus go "
                      "idle for three milliseconds in twenty seconds. What would measure it is "
                      "a host whose port may autosuspend, or a hub whose downstream port can be "
                      "suspended by hand.",
              crlf);
        bench.verdict("the suspend letter is skipped and says so", true);
        return;
    }
    const uint8_t masters = P::bus_masters_active();
    print(serial, "  suspended: suspends=", Device::suspends(), " bus masters=", masters,
          "; 20 s for the resume", crlf);
    bench.verdict("the controller reports the bus going idle as a suspend event", true);
    bench.verdict("a suspended controller still holds its bus-master count, which is deliberate: "
                  "the count is held from the pull-up to the detach, and what a suspended "
                  "controller would let the core sleep through is a question for a meter",
                  masters == 1u);

    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 20000u && Device::resumes() == resumes) {
    }
    if (Device::resumes() == resumes) {
        print(serial, "  the resume did not come in twenty seconds; the port may need touching "
                      "at the host end", crlf);
        bench.verdict("the resume is not claimed and says so", true);
        return;
    }
    wait_ms(200);
    print(serial, "  resumed: resumes=", Device::resumes(), " state=", state_name(Device::state()),
          " frame=", Usb::frame(), " errors=", Usb::errors(), " overflows=", Usb::overruns(), crlf);
    bench.verdict("the resume is reported and the port is carrying frames again, configured as "
                  "it was",
                  Device::resumes() > resumes && Device::configured() && Port::configured());
}

// =============================================================================
// d - the packet-memory overflow counter
// =============================================================================
void td_overflow() {
    // THE COUNTER IS THE ATTACHMENT'S, so what this letter judges is a
    // WINDOW and not the total: an overflow staged here stands in the
    // count until the block is brought up again (letter a does that),
    // and a letter that judged the total would fail on its second run
    // for a reason that is the measurement itself.
    const uint32_t standing = Usb::overruns();
    print(serial, "  overflows=", standing, " bus errors=", Usb::errors(), " resets=",
          Device::resets(), " setups=", Device::setups(), " bus masters=",
          P::bus_masters_active(), crlf);
    print(serial, "  the counter saturates rather than wrapping, because a counter that wrapped "
                  "would report a healthy bus; the driver counts the event and re-arms the "
                  "receiver over a whole buffer, because the lost packet leaves its COUNT where "
                  "the buffer's SIZE was - the packet itself is gone and nothing is retried, so "
                  "a control transfer survives because the HOST retries it and a bulk burst "
                  "does not",
          crlf);
    wait_ms(100);
    bench.verdict("the packet-memory overflow counter does not move on a device the core stays "
                  "awake for",
                  Usb::overruns() == standing);

    if (!Device::configured()) {
        print(serial, "  SKIPPED, and it passes as a skip: staging an overflow wants a host "
                      "pouring bytes into a configured port.",
              crlf);
        bench.verdict("the staged overflow is skipped and says so", true);
        return;
    }
    settle_port();
    print(serial, "  to stage one: run the script's d now (it pours for twenty seconds; this "
                  "letter waits six for the first byte)", crlf);
    if (wait_for_traffic(6000u) == 0u) {
        print(serial, "  SKIPPED, and it passes as a skip: the host poured nothing in six "
                      "seconds, so the counter's other side is not measured here.",
              crlf);
        bench.verdict("the staged overflow is skipped and says so", true);
        return;
    }

    // AWAKE FIRST, as the reference: the same span, the same pour, the
    // core draining the ring as fast as the host fills it.
    uint8_t b = 0;
    uint32_t awake = 0;
    uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 50u) {
        if (Port::read_byte(b)) {
            ++awake;
        }
    }
    const uint32_t overruns_before = Usb::overruns();
    const uint32_t rx_before = Port::rx_bytes();

    // THEN ASLEEP, WITH THE BARE INSTRUCTION. The platform's idle()
    // would not sleep here at all - the attached controller's count
    // forbids it - which is exactly the mechanism this letter is
    // staging the reason for. So the sleep is written out: a wfi with
    // interrupts ENABLED, woken by the tick and by the controller's own
    // vector and going back to sleep between them. This is an
    // instrument and not an idiom; no driver and no program of this
    // stratum writes it.
    //
    // AND IT IS A TRUE WFI HERE. The platform arms WFITOWFE only on the
    // path that sleeps, and it does not take that path while a bus
    // master is counted - so the bit is clear with the controller
    // attached. If it were set the instruction would return at once,
    // the core would stay awake and the counter would not move: the
    // verdict below would FAIL, which is the safe direction for a bit
    // this program cannot see.
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 50u) {
        __asm__ volatile("wfi" ::: "memory");
    }
    const uint32_t overflowed = Usb::overruns() - overruns_before;
    const uint32_t arrived = Port::rx_bytes() - rx_before;
    print(serial, "  50 ms awake took ", awake, " bytes; 50 ms of wfi under the same pour took ",
          arrived, " and counted ", overflowed, " packet-memory overflows", crlf);
    // THE COUNTER IS THE MEASUREMENT AND THE BYTES ARE NOT: nothing
    // drains the receive ring while the core sleeps, so a full ring
    // would stop the bytes too, by flow control and not by loss. What
    // only a lost packet explains is the overflow.
    bench.verdict("A SLEEPING CORE COSTS THE CONTROLLER ITS PACKET MEMORY: under the same pour "
                  "that the awake window carried, the sleeping one counts overflows - the "
                  "measurement the bus-master count exists to make unreachable",
                  overflowed > 0u);

    // WHAT THE OVERFLOW DID TO THE ENDPOINT. The halfword that carries
    // the buffer's size is the one the hardware writes the received
    // count into, and a packet that never completes raises no
    // completion for any layer to answer - so without the driver's
    // repair this endpoint would be armed over a buffer of zero blocks
    // and would lose every packet after the first, for ever.
    const uint16_t count_rx = usbd_pma(count_rx_offset);
    print(serial, "  the bulk OUT buffer's COUNTn_RX reads ", hex(count_rx), ": its size field ",
          hex(static_cast<uint16_t>(count_rx & count_rx_size)), " where a whole ", Port::packet,
          "-byte buffer is ", hex(usbd_count_rx_for(Port::packet)), ", and a count of ",
          count_rx & 0x03FFu, " in the ten bits under it", crlf);
    bench.verdict("the driver writes that size back on every overflow, so the receiver is armed "
                  "over a whole buffer again and not over the zero blocks a lost packet leaves",
                  (count_rx & count_rx_size) == usbd_count_rx_for(Port::packet));

    // AND IT RECOVERS, which is the other half of the answer: the
    // driver counts and carries on, and the port is a port again as
    // soon as the core is awake.
    const uint32_t settled = Usb::overruns();
    uint32_t after = 0;
    t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 200u) {
        if (Port::read_byte(b)) {
            ++after;
        }
    }
    print(serial, "  awake again: ", after, " bytes in 200 ms, overflows ", settled, " -> ",
          Usb::overruns(), ", state=", state_name(Device::state()), crlf);
    bench.verdict("the driver counts the overflow and carries on: with the core awake the port "
                  "takes bytes again, still configured, and the counter stops climbing",
                  after > 0u && Usb::overruns() == settled && Device::configured());
}

// ===========================================================================
// The menu
// ===========================================================================
void banner() {
    print(serial, crlf, "test_v203_usb - the USB device controller of RM ch. 21 under the CDC "
                        "ACM stack; HCLK ",
          SysClock::hz, " Hz, the controller fed ", SysClock::usb_hz, crlf,
          "  the console is the probe's UART and the board's USB-C is the device: the host at "
          "its other end answers the letters y, w, v, s and d",
          crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

// The controller's low-priority line, which every event of a
// single-buffered device arrives on; the high-priority one, shared with
// the CAN's transmit, belongs to the double-buffered and isochronous
// endpoints this driver does not use.
extern "C" BRIO_CH32_INTERRUPT void usb_lp_can1_rx0_handler() { Device::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool usb_ok = Usb::init(clock);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();
    if (usb_ok) {
        Device::start();   // the pull-up: the host enumerates from here
    }

    bench.letter('a', "the controller, and the host enumerating the device", ta_enumerate);
    bench.letter('u', "a reconnect: the pull-up dropped and raised", tu_reconnect);
    bench.letter('y', "echo for 10 s (host-assisted)", ty_echo, false);
    bench.letter('w', "the line coding and DTR from the host (host-assisted)", tw_line, false);
    bench.letter('v', "throughput both ways (host-assisted)", tv_throughput, false);
    bench.letter('s', "suspend and resume (host-assisted)", ts_suspend, false);
    bench.letter('d', "the packet-memory overflow counter, and the sleep that fills it",
                 td_overflow);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "XT96" : "FAILED",
              " usb=", usb_ok ? "48MHz" : "FAILED", " tick=", tick_ok ? "STK" : "FAILED", crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            // THE LOOP IDLES (the file header): with the controller
            // attached the count is not zero and this returns at once,
            // and with it detached the core sleeps until the console or
            // the tick wakes it. The silicon's own state is what
            // decides, not a rule the program keeps.
            P::CriticalSection cs;
            P::idle();
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
