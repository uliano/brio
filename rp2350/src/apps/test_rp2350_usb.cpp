// test_rp2350_usb - the reference bench suite for the RP2350's USB
// device controller (rp2350/usb.hpp over datasheet 12.7) under the
// device stack (util/usb/device.hpp) and the CDC ACM class
// (util/usb/cdc.hpp): enumeration against a real host, the port as a
// byte transport, the class requests, the bus reset - and THE FIVE
// THINGS 12.7.2 ADDED to the RP2040's controller, each with a number
// beside its verdict. From one source on both of this chip's
// architectures.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// TWO ENDS, AND NO WIRE. The console is the probe's bridge on UART0
// (GP0 / GP1); the chip's own USB-C connector goes to the host, which is
// the other end of every measure: Linux enumerates the device (the
// pid.codes test identity 1209:0001), binds cdc-acm, and the host
// letters talk to /dev/ttyACM* with pyserial, as the RP2040's USB suite
// does. Nothing here needs a jumper or a button.
//
// THE CLOCK IS PART OF THE TEST. Erratum RP2350-E12 loses SIE flags
// across the clk_usb / clk_sys boundary unless clk_sys runs at least ten
// per cent above 48 MHz; this image runs at 150 MHz and letter a says
// so with the counted number.
//
// What is exercised, letter by letter:
//   a  THE CONTROLLER AND THE ENUMERATION, no host action needed beyond
//      the cable: clk_usb counted at 48 MHz, the E12 margin, the PHY's
//      isolation lifted, the muxing word with USBPHY_AS_GPIO clear, VBUS
//      seen, the pull-up raised - then the host does what a host does
//      (the bus reset, the descriptors, the address, the configuration)
//      and the suite watches the state reach CONFIGURED within a second
//      and a half, the setups counted, the class's endpoints armed, the
//      start-of-frame counter advancing a frame a millisecond
//   b  WHAT THIS CHIP ADDED: the two free-running 48 MHz timestamps
//      around the last start of frame and the microseconds since it, the
//      three state machines in SM_STATE, the per-endpoint error counters
//      after a clean enumeration, LINESTATE_TUNING at its reset value
//      with both device fixes on, the device state-machine watchdog
//      armed and read back, and the endpoint abort that the other chip's
//      erratum E2 made useless
//   c  A RECONNECT THROUGH THE PULL-UP: dropped for half a second and
//      raised again - the host's bus reset counted, the device
//      enumerated afresh
//   d  A RECONNECT THROUGH THE PHY'S ISOLATION: MAIN_CTRL.PHY_ISO set
//      for half a second, which is what a power-down of the switched
//      core domain leaves behind, then the controller brought up again
//      from scratch
//
//   y  (HOST-ASSISTED, outside the all-key) ECHO: for ten seconds every
//      byte the host sends on the port comes back; the host end checks
//      the echo byte-exact and the letter reports the counts
//   w  (host-assisted) THE LINE CODING AND DTR: the values as they
//      stand, ten seconds for the host to change them, the values again
//   v  (host-assisted) THROUGHPUT: three seconds of the device pouring a
//      counting pattern into the port as fast as the host takes it, then
//      two seconds counting what the host pours in
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/uart.hpp"
#include "rp2350/usb.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

namespace {

using namespace brio;

using P = Rp2350Platform<>;
using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
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
    static constexpr auto device =
        usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        usb_concat(usb_configuration_head(9 + Port::descriptor_bytes, Port::interface_count), Port::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static constexpr auto manufacturer = usb_string_descriptor("brio");
    static constexpr auto product = usb_string_descriptor("test_rp2350_usb");
    // The serial string names the half that is running, so that a host
    // with both images on its bus over a day can tell them apart.
    static constexpr auto serial_m33 = usb_string_descriptor("cortex-m33");
    static constexpr auto serial_hazard3 = usb_string_descriptor("hazard3");
    static std::span<const uint8_t> string(uint8_t index) {
        switch (index) {
        case 0: return language;
        case 1: return manufacturer;
        case 2: return product;
        case 3:
            if constexpr (core_kind == CoreKind::hazard3) {
                return serial_hazard3;
            } else {
                return serial_m33;
            }
        default: return {};
        }
    }
};
using Device = UsbDevice<Usb, Descriptors, Port>;

uint32_t us_now() { return Mtime::micros(); }

bool within(uint32_t got, uint32_t want, uint32_t per_mille) {
    const uint32_t tol = static_cast<uint32_t>(static_cast<uint64_t>(want) * per_mille / 1000u);
    return got + tol >= want && got <= want + tol;
}

void wait_us(uint32_t span) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < span) {
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
    // On a re-run the device goes down first; on the first run nothing
    // here touches the block, because a peripheral this program has not
    // taken out of reset answers a read with a bus error and the
    // bootrom's own state is not this suite's to assume.
    if (usb_up) {
        Device::stop();
        Usb::release();
    }
    const uint32_t t0 = us_now();
    usb_up = Usb::init(clock);
    const uint32_t init_us = us_now() - t0;
    const auto usb_hz = SysClock::count_hz(CountSource::clk_usb);
    const auto sys_hz = SysClock::count_hz(CountSource::clk_sys);
    print(serial, "  init: ", usb_up ? "ok" : "FAILED", " in ", init_us, " us; clk_usb counted ", usb_hz ? *usb_hz : 0u,
          " Hz, clk_sys ", sys_hz ? *sys_hz : 0u, " Hz, USB PLL locked=", PllUsb::locked(),
          ", VBUS detected=", Usb::vbus_detected(), ", connected=", Usb::connected(), ", pulled up=", Usb::pulled_up(),
          crlf);
    bench.verdict("the controller initializes with clk_usb counted at 48 MHz from the USB PLL, VBUS seen (forced: the "
                  "board wires no VBUS pin), the pull-up still down",
                  usb_up && usb_hz && within(*usb_hz, clk_usb_hz, 1) && Usb::vbus_detected() && !Usb::pulled_up());
    bench.verdict("erratum RP2350-E12: clk_sys runs at least ten per cent above clk_usb - 52.8 MHz is the edge and this "
                  "image is at 150 MHz",
                  usb_clk_sys_ok(SysClock::hz) && sys_hz && usb_clk_sys_ok(*sys_hz));

    print(serial, "  MAIN_CTRL=", hex(Usb::regs().MAIN_CTRL), " (PHY isolated=", Usb::phy_isolated(),
          ", controller enabled=", Usb::controller_enabled(), "), USB_MUXING=", hex(Usb::muxing()),
          " (phy as gpio=", Usb::phy_as_gpio(), "), SIE_CTRL=", hex(Usb::regs().SIE_CTRL), crlf);
    bench.verdict("the one new duty of 12.7.2: init() lifted MAIN_CTRL.PHY_ISO, which resets SET, and enabled the "
                  "controller in device mode",
                  !Usb::phy_isolated() && Usb::controller_enabled());
    bench.verdict("the muxing is written whole, so the PHY is the controller's and not the SIO's: TO_PHY and SOFTCON "
                  "alone, USBPHY_AS_GPIO clear",
                  !Usb::phy_as_gpio() &&
                      Usb::muxing() == (USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS));
    bench.verdict("SIE_CTRL is written whole too: the host's pull-downs, which reset SET on this chip and not on the "
                  "RP2040, are gone",
                  (Usb::regs().SIE_CTRL & USB_SIE_CTRL_PULLDOWN_EN_BITS) == 0u);

    bench.verdict("the descriptors: the device's 18 bytes, the configuration's 67 (two interfaces, three endpoints), "
                  "four strings",
                  Descriptors::device.size() == 18u && Descriptors::configuration.size() == 67u &&
                      Descriptors::configuration[4] == 2u && !Descriptors::string(3).empty() &&
                      Descriptors::string(4).empty());

    const uint32_t stalls_before = Device::stalls();
    const uint32_t setups_before = Device::setups();
    Device::start();
    const uint32_t took = wait_configured(1'500'000u);
    print(serial, "  start(): pull-up=", Usb::pulled_up(), "; the host took ", took,
          " us to configure: state=", state_name(Device::state()), " address=", Device::address(),
          " resets=", Device::resets(), " setups=", Device::setups(), " stalls=", Device::stalls(),
          " last request=", Device::last_request(), " frame=", Usb::frame(), crlf);
    bench.verdict("start() raises the pull-up and the host enumerates the device to CONFIGURED within a second and a "
                  "half",
                  Usb::pulled_up() && took != 0u && Device::configured());
    bench.verdict("the host reset the bus at least once and assigned a non-zero address",
                  Device::resets() >= 1u && Device::address() != 0u);
    bench.verdict("enumeration took at least the four requests a host must make, and the stall counter is small (a "
                  "device qualifier or a string the device has not are stalled by the rules)",
                  Device::setups() - setups_before >= 4u && Device::stalls() - stalls_before <= 8u);
    bench.verdict("the port is configured with its bulk OUT armed and nothing in flight",
                  Port::configured() && Port::out_armed() && !Port::in_armed() && Port::tx_idle());
    bench.verdict("the controller handed out the notification's buffer, the bulk OUT's two halves and the bulk IN's "
                  "past endpoint zero's",
                  Usb::buffers_used() == Usb::buffers_start + 4u * 64u);
    print(serial, "  line coding as found: ", Port::line_coding().baud, "/", Port::line_coding().data_bits, "/",
          Port::line_coding().parity, "/", Port::line_coding().stop_bits, " dtr=", Port::dtr(),
          " coding changes=", Port::coding_changes(), " frames seen=", Usb::frame(), crlf);
    // Ten milliseconds of frames: the host's start-of-frame counter moves.
    const uint16_t f0 = Usb::frame();
    wait_us(10'000u);
    const uint16_t frames = static_cast<uint16_t>((Usb::frame() - f0) & 0x7FFu);
    print(serial, "  start-of-frame counter advanced ", frames, " in 10 ms", crlf);
    bench.verdict("the host's start-of-frame counter advances a frame a millisecond", frames >= 9u && frames <= 11u);
    const uint32_t errors = Usb::take_errors();
    print(serial, "  SIE errors during enumeration: ", hex(errors), crlf);
    bench.verdict("no CRC, bit-stuff, overflow, timeout or data-sequence error", errors == 0u);
}

// =============================================================================
// b - what 12.7.2 added to this controller
// =============================================================================
void tb_additions() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }

    // The two 48 MHz timestamps: one free running, one latched at the
    // last start of frame. A live host sends one a millisecond, so the
    // distance between them is under 48 000 cycles of that domain.
    const uint32_t raw_a = Usb::sof_timestamp_raw();
    const uint32_t last_a = Usb::sof_timestamp_last();
    const uint32_t since_a = Usb::since_sof();
    wait_us(300u);
    const uint32_t raw_b = Usb::sof_timestamp_raw();
    const uint32_t since_b = Usb::since_sof();
    const uint32_t advanced = (raw_b + 0x20'0000u - raw_a) % 0x20'0000u;
    print(serial, "  SOF timestamps: raw ", raw_a, " -> ", raw_b, " (", advanced, " cycles over 300 us), last ", last_a,
          ", since_sof ", since_a, " then ", since_b, " cycles of 48 MHz", crlf);
    bench.verdict("the free-running 48 MHz counter advances about 14 400 cycles over 300 microseconds",
                  within(advanced, 14'400u, 50u));
    bench.verdict("the last start of frame is under a millisecond old both times - 48 000 cycles of that counter",
                  since_a < 48'000u && since_b < 48'000u);

    const uint32_t sm = Usb::sm_state();
    print(serial, "  SM_STATE=", hex(sm), ": main ", Usb::sm_main(), ", bus control ", Usb::sm_bus_control(),
          ", rx deserialiser ", Usb::sm_rx_deserialiser(), crlf);
    bench.verdict("SM_STATE reads inside its three fields and no bit above them",
                  (sm & ~static_cast<uint32_t>(USB_SM_STATE_BITS)) == 0u && Usb::sm_main() < 32u &&
                      Usb::sm_bus_control() < 8u && Usb::sm_rx_deserialiser() < 16u);

    const UsbEndpointErrors ep0 = Usb::endpoint_errors(0);
    const UsbEndpointErrors ep2 = Usb::endpoint_errors(2);
    print(serial, "  endpoint errors after enumeration: ep0 tx=", ep0.tx_count, " rx=", ep0.rx_transaction, "/",
          ep0.rx_sequence, ", ep2 tx=", ep2.tx_count, " rx=", ep2.rx_transaction, "/", ep2.rx_sequence,
          ", ENDPOINT_ERROR=", Usb::take_endpoint_error(), crlf);
    bench.verdict("the per-endpoint error counters this chip added are clear on a bus that enumerated cleanly",
                  !ep0.any() && !ep2.any());
    Usb::clear_endpoint_errors();
    bench.verdict("and the write-to-clear pair reads back clear after it",
                  !Usb::endpoint_errors(0).any() && !Usb::endpoint_errors(2).any());

    const uint32_t tuning = Usb::linestate_tuning();
    print(serial, "  LINESTATE_TUNING=", hex(tuning), " (buffer-control double read=",
          Usb::buffer_control_double_read_fix(), ", wake on any bus activity=", Usb::wake_on_any_bus_activity(), ")",
          crlf);
    bench.verdict("LINESTATE_TUNING stands at its reset value 0x0f8 - 12.7.2 says to leave it there, and this driver "
                  "has no verb that writes it",
                  tuning == 0xF8u && Usb::buffer_control_double_read_fix() && Usb::wake_on_any_bus_activity());

    // The device state-machine watchdog, armed WITHOUT the forced reset
    // so that it cannot disturb the very bus this suite is talking over:
    // the limit written with the enable low and read back, then the
    // fired flag WATCHED at the widest limit and at a millisecond's
    // worth of clk_usb cycles. Whether a quiet bus keeps the counter
    // reset is what the two numbers report; nothing here asserts it.
    const bool armed = Usb::arm_device_watchdog({.limit = 0x3'FFFFu, .reset_on_fire = false});
    const uint32_t limit = Usb::device_watchdog_limit();
    const bool standing = Usb::device_watchdog_armed();
    wait_us(20'000u);
    const bool fired_wide = Usb::take_device_watchdog_fired();
    (void)Usb::arm_device_watchdog({.limit = 48'000u, .reset_on_fire = false});
    wait_us(20'000u);
    const bool fired_narrow = Usb::take_device_watchdog_fired();
    Usb::disarm_device_watchdog();
    print(serial, "  device watchdog: armed=", armed, " limit read back ", limit, " standing=", standing,
          "; fired over 20 ms at the widest limit (262143 cycles)=", fired_wide, ", at 48000 cycles=", fired_narrow,
          ", disarmed=", !Usb::device_watchdog_armed(), crlf);
    bench.verdict("the device state-machine watchdog takes its 18-bit limit, reads it back, stands, disarms - and the "
                  "port is still up after it",
                  armed && limit == 0x3'FFFFu && standing && !Usb::device_watchdog_armed() && Port::configured());
    bench.verdict("a limit past eighteen bits is refused instead of truncated",
                  !Usb::arm_device_watchdog({.limit = 0x4'0000u}));

    // The abort path, on the notification endpoint the class never
    // writes: on the RP2040 erratum E2 left the abort standing, and this
    // chip fixes it.
    const uint8_t notify = usb_ep_in(1);
    Usb::abort(notify, true);
    uint32_t spins = 0;
    while (!Usb::abort_done(notify) && spins < 100'000u) {
        ++spins;
    }
    const bool done = Usb::abort_done(notify);
    Usb::abort(notify, false);
    print(serial, "  endpoint abort on the notification endpoint: done after ", spins, " spins, lifted=",
          !Usb::aborted(notify), ", done flag cleared=", !Usb::abort_done(notify), crlf);
    bench.verdict("an idle endpoint's abort completes and LIFTS - the RP2040's erratum E2 left it standing, and 12.7.2 "
                  "fixes it here",
                  done && !Usb::aborted(notify) && !Usb::abort_done(notify));
    bench.verdict("the port still works after it: configured, its bulk OUT armed", Port::configured() &&
                                                                                   Port::out_armed());
}

// =============================================================================
// c - a reconnect through the pull-up
// =============================================================================
void tc_reconnect() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    const uint32_t resets = Device::resets();
    const uint8_t address = Device::address();
    Device::stop();
    wait_us(500'000u);
    print(serial, "  stopped for 500 ms: state=", state_name(Device::state()), "; start() again...", crlf);
    Device::start();
    const uint32_t took = wait_configured(2'000'000u);
    print(serial, "  resets ", resets, " -> ", Device::resets(), ", address ", address, " -> ", Device::address(),
          ", configured in ", took, " us, state=", state_name(Device::state()), crlf);
    bench.verdict("dropping the pull-up detaches the device; raising it again brings the host's bus reset (counted) "
                  "and a fresh enumeration",
                  Device::resets() > resets && took != 0u);
    bench.verdict("the port is configured again with its bulk OUT armed", Port::configured() && Port::out_armed());
}

// =============================================================================
// d - a reconnect through the PHY's isolation, this chip's own way
// =============================================================================
void td_isolation() {
    if (!Device::configured()) {
        bench.verdict("the device is configured (run a first)", false);
        return;
    }
    const uint32_t resets = Device::resets();
    Usb::phy_isolate(true);
    const bool isolated = Usb::phy_isolated();
    const bool pulled_while_isolated = Usb::pulled_up();
    wait_us(500'000u);
    print(serial, "  PHY isolated for 500 ms (MAIN_CTRL=", hex(Usb::regs().MAIN_CTRL), ", pull-up bit still ",
          pulled_while_isolated, "): this is what a power-down of the switched core domain leaves behind", crlf);
    // The way back is the whole of init(), which is what the datasheet
    // asks for after a power-down event: the isolation lifted last, once
    // the controller is configured again.
    Device::stop();
    const bool up = Usb::init(clock);
    Device::start();
    // AND THE HOST IS NOT TOLD. Six seconds, not two, and the answer is
    // the same: no bus reset arrives. The isolation cuts the PHY from
    // the switched core domain, it does not take the pull-up off the
    // wire, so from the host's side nothing happened and there is
    // nothing to enumerate - while the device's own stack has been
    // reset to DEFAULT and no longer answers at the address the host is
    // still using. That silent half-open state is the finding, and the
    // reason a program coming back from a power-down must force the
    // reconnect itself.
    const uint32_t took = wait_configured(6'000'000u);
    print(serial, "  re-init=", up, ", PHY isolated=", Usb::phy_isolated(), ", resets ", resets, " -> ",
          Device::resets(), ", address=", Device::address(), ", configured in ", took, " us, state=",
          state_name(Device::state()), crlf);
    bench.verdict("MAIN_CTRL.PHY_ISO reads back set while it stands, and the bring-up sequence lifts it again",
                  isolated && up && !Usb::phy_isolated());
    bench.verdict("BUT THE ISOLATION IS INVISIBLE TO THE HOST: no bus reset arrives in six seconds and the device "
                  "is left in DEFAULT while the host goes on addressing the address it assigned, because isolating "
                  "the PHY does not take the pull-up off the wire",
                  Device::resets() == resets && took == 0u && !Port::configured());
    // The way back, and the only one: the device's own pull-up, down
    // long enough for the host to run its disconnect debounce.
    Device::stop();
    wait_us(500'000u);
    Device::start();
    const uint32_t back = wait_configured(4'000'000u);
    print(serial, "  the device forcing the reconnect afterwards: resets ", resets, " -> ", Device::resets(),
          ", address=", Device::address(), ", configured in ", back, " us, state=", state_name(Device::state()),
          crlf);
    bench.verdict("and the way back is the device's own pull-up, dropped long enough for the host to see it: a bus "
                  "reset, a new address and CONFIGURED again",
                  Device::resets() > resets && back != 0u && Port::configured() && Port::out_armed());
    const uint32_t errors = Usb::take_errors();
    const uint32_t again = Usb::take_errors();
    print(serial, "  SIE errors over the isolation round trip: ", hex(errors), ", and on a second read ", hex(again),
          crlf);
    bench.verdict("whatever the round trip raised, the error flags are write-to-clear: the second read shows none",
                  again == 0u);
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
    print(serial, "  echoed ", echoed, " bytes (received ", Port::rx_bytes() - rx0, ", sent ", Port::tx_bytes() - tx0,
          "), overruns ", Port::rx_overruns(), ", SIE errors ", hex(Usb::take_errors()), ", endpoint errors ",
          Usb::endpoint_errors(2).tx_count, crlf);
    bench.verdict("every byte received went back, no overrun (the host end judges the bytes)",
                  echoed > 0u && Port::rx_bytes() - rx0 == echoed && Port::tx_bytes() - tx0 == echoed &&
                      Port::rx_overruns() == 0u);
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
    print(serial, "  now: ", before.baud, "/", before.data_bits, "/", before.parity, "/", before.stop_bits,
          " dtr=", Port::dtr(), "; 10 s for the host to open the port at another rate and raise DTR", crlf);
    wait_us(10'000'000u);
    const UsbLineCoding after = Port::line_coding();
    print(serial, "  then: ", after.baud, "/", after.data_bits, "/", after.parity, "/", after.stop_bits,
          " dtr=", Port::dtr(), " rts=", Port::rts(), " coding changes ", changes, " -> ", Port::coding_changes(),
          ", line state changes ", states, " -> ", Port::line_state_changes(), crlf);
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
    print(serial, "  received ", got, " bytes in 2 s: ", got / 2000u, " KB/s, drained ", drained, ", overruns ",
          Port::rx_overruns(), crlf);
    bench.verdict("the device takes more than 200 KB/s from the port with no overrun",
                  got > 400'000u && Port::rx_overruns() == 0u);
}

void banner() {
    print(serial, crlf, "test_rp2350_usb - the RP2350 USB device controller under the CDC ACM stack on ",
          core_kind == CoreKind::hazard3 ? "Hazard3" : "Cortex-M33", "; the port on the host is the other end; clk_sys=",
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
    const bool mtime_ok = brio::Mtime::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the controller, and the host enumerating the device", ta_enumerate);
    bench.letter('b', "what 12.7.2 added: the timestamps, the state machines, the error counters, the watchdog, the "
                      "abort",
                 tb_additions);
    bench.letter('c', "a reconnect: the pull-up dropped and raised", tc_reconnect);
    bench.letter('d', "a reconnect: the PHY isolated and brought back", td_isolation);
    bench.letter('y', "echo for 10 s (host-assisted)", ty_echo, false);
    bench.letter('w', "the line coding and DTR from the host (host-assisted)", tw_line, false);
    bench.letter('v', "throughput both ways (host-assisted)", tv_throughput, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " us=", mtime_ok ? "MTIME" : "FAILED", " tick=", tick_ok ? "1 kHz" : "FAILED", brio::crlf);
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
