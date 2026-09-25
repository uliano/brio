// console_usb - the brio kernel console over the chip's own USB: the
// same three active objects as the console app (SerialPort, Console,
// Blinker), the transport a CDC ACM port instead of a UART - nothing
// above the transport changed. THE CONTROLLER IS THE PART'S: the
// CH32V203's USB device controller (ch32vx03/usb.hpp, RM ch. 21) where
// the part has one, and the host/device controller in device mode
// (ch32vx03/usbfs.hpp, RM ch. 23) on the CH32V303, which has no other -
// chosen by the part table's has_usbd, the one type alias below.
// The host sees a serial port (/dev/ttyACM* on Linux, no driver to
// install) and types the same commands:
//   HELP | LED ON|OFF|TOG | UPTIME | ERR | USB
// USB reports the device's state, address, the line coding the host set
// and DTR, and how much of the controller's buffer memory is spent (the
// device controller's shared packet memory, the host/device controller's
// pool); its "frame" is the host/device controller's frame count, which
// stays at zero there unless frame counting is switched on.
//
// The line coding is reported, not obeyed: a virtual port has no baud
// rate, so "115200" in a terminal is a number the device reads back.
// The identity is the pid.codes test pair 1209:0001, meant for exactly
// this.
//
// THE LOOP IS THE ORDINARY ONE, AND IT DOES NOT SLEEP HERE - which is
// no longer a rule this program keeps but a fact of the machine it runs
// on. With the core stopped the USB controller cannot reach its packet
// memory (measured: an armed bulk endpoint receives nothing, the
// packet-memory overflow counts up, and an enumeration never gets past
// its first control transfer), so an attached controller counts itself
// a bus master and the platform's idle path does not sleep while the
// count stands (ch32vx03/bus_activity.hpp). The kernel therefore runs
// its normal loop, idle path and all, and the silicon's own state is
// what decides.
//
// Wiring: on a WeAct CH32V203C8T6 core board the USB-C connector is on
// PA11/PA12, which is the USBD block's pair (the USBFS one on PB6/PB7
// is a different peripheral and a different chapter) - measured, the
// host's pull-downs hold those two pads and PB6/PB7 are free. On WCH's
// CH32V303 evaluation board it is the connector P14, on PA11/PA12 too,
// which that part gives its host/device controller. The clock must be
// 48, 96 or 144 MHz for either controller to be fed its 48. The LED is
// the board's on PB2 (active high on the WeAct board, low on WCH's).
//
// build: boards = v203c6,v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include <type_traits>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usb.hpp"
#include "ch32vx03/usbfs.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"
#include "util/trace.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

using P = brio::Ch32vx03Platform<>;
// The PLL fed by the board's 8 MHz CRYSTAL: full-speed USB wants its
// 48 MHz within 2500 ppm and an internal RC is not that accurate. x6 =
// 48 MHz, which the USB divider then takes whole.
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000, 8'000'000>;
constexpr SysClock clock;

namespace {

using Led = brio::Pin<'B', 2>;

// The device controller with its CAN-safe 384 bytes of packet memory
// where the part has one; the host/device controller with its buffers
// in RAM (the pool that never refuses a claim) where it has not.
using Usb = std::conditional_t<brio::device::has_usbd, brio::Usbd<>, brio::Usbfs<>>;
using Serial = brio::UsbCdcAcm<Usb, P>;       // interface 0, endpoints 1 (notification) and 2 (bulk)
constexpr Serial serial;

struct Descriptors {
    static constexpr auto device =
        brio::usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        brio::usb_concat(brio::usb_configuration_head(9 + Serial::descriptor_bytes, Serial::interface_count),
                         Serial::descriptors);
    static constexpr auto language = brio::usb_language_descriptor();
    static constexpr auto manufacturer = brio::usb_string_descriptor("brio");
    static constexpr auto product = brio::usb_string_descriptor("brio console");
    static constexpr auto serial_number = brio::usb_string_descriptor("ch32vx03");
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
using Device = brio::UsbDevice<Usb, Descriptors, Serial>;

// ---- events -----------------------------------------------------------------
struct Toggle {};
struct SetLed { enum class Mode : uint8_t { on, off, tog } mode; };

// ---- the blinker: owns the LED ----------------------------------------------
struct Blinker : brio::Fsm<Blinker, Toggle, SetLed> {
    static inline brio::EventQueue<Event, 2, P> queue;
    static inline brio::TimeEvent<P, Blinker, Toggle> heartbeat{Toggle{}};

    static void init() {
        Led::output();
        start(&beating);
    }

    static Status beating(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { heartbeat.arm_every(brio::ticks_from_ms<P>(500)); return handled(); },
            [](brio::Exit) { heartbeat.disarm(); return handled(); },
            [](Toggle) { Led::toggle(); return handled(); },
            [](SetLed s) { apply(s); return transition(&manual); },
            [](auto) { return unhandled(); });
    }

    static Status manual(const Event& e) {
        return brio::match(e,
            [](SetLed s) { apply(s); return handled(); },
            [](auto) { return unhandled(); });
    }

private:
    static void apply(SetLed s) {
        switch (s.mode) {
            case SetLed::Mode::on:  Led::set(); break;
            case SetLed::Mode::off: Led::clear(); break;
            case SetLed::Mode::tog: Led::toggle(); break;
        }
    }
};

// ---- the console --------------------------------------------------------------
struct Console : brio::Fsm<Console, brio::LineReceived> {
    static inline brio::EventQueue<Event, 2, P> queue;

    using Parser = brio::ConsoleCommandParser<4>;
    using Router = brio::CommandRouter<Serial, 4>;
    using Cmd = Router::CommandType;

    static void init() { start(&running); }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](brio::LineReceived l) { handle_line(l.line.get()); return handled(); },
            [](auto) { return unhandled(); });
    }

private:
    static void handle_line(char* line) {
        Cmd cmd;
        if (Parser::parse(line, cmd)) {
            if (!Router::dispatch(cmd, routes, route_count, serial)) {
                brio::print(serial, "unknown command (try HELP)", brio::crlf);
            }
        }
        brio::print(serial, "> ");
    }

    static void cmd_help(const Cmd&, Serial s) {
        brio::print(s, "commands: HELP | LED ON|OFF|TOG | UPTIME | ERR | USB", brio::crlf);
    }

    static void cmd_led(const Cmd& cmd, Serial s) {
        SetLed::Mode mode;
        const char* arg = (cmd.argument_count == 1) ? cmd.arguments[0] : "";
        if (brio::command_equals(arg, "ON")) {
            mode = SetLed::Mode::on;
        } else if (brio::command_equals(arg, "OFF")) {
            mode = SetLed::Mode::off;
        } else if (brio::command_equals(arg, "TOG")) {
            mode = SetLed::Mode::tog;
        } else {
            brio::print(s, "usage: LED ON|OFF|TOG", brio::crlf);
            return;
        }
        brio::post<Blinker>(SetLed{mode});
        brio::print(s, "OK", brio::crlf);
    }

    static void cmd_uptime(const Cmd&, Serial s) {
        brio::TimeStamp ts;
        brio::Ticker::now(ts);
        brio::print(s, "uptime: ", ts, brio::crlf);
    }

    static void cmd_usb(const Cmd&, Serial s) {
        brio::print(s, "state=", static_cast<uint8_t>(Device::state()),
                    " address=", Usb::address(),
                    " frame=", Usb::frame(),
                    " coding=", Serial::line_coding().baud,
                    " dtr=", Serial::dtr() ? 1 : 0,
                    " pma_used=", Usb::buffer_used(),
                    " pma_free=", Usb::buffer_free(),
                    brio::crlf);
    }

    static void cmd_err(const Cmd&, Serial s);

    static constexpr Router::Route routes[] = {
        {"HELP", cmd_help}, {"LED", cmd_led}, {"UPTIME", cmd_uptime}, {"ERR", cmd_err}, {"USB", cmd_usb},
    };
    static constexpr uint8_t route_count = sizeof(routes) / sizeof(routes[0]);
};

using SerialLines = brio::SerialPort<Serial, P, Console, 80>;

}  // namespace

// The bench instrument of this bring-up: one mark per USB interrupt.
brio::Trace<60, P> usb_trace;

namespace {

void Console::cmd_err(const Cmd&, Serial s) {
    brio::print(s, "rx_overruns=", Serial::rx_overruns(),
                " rx_bytes=", Serial::rx_bytes(),
                " tx_bytes=", Serial::tx_bytes(),
                " line_overflows=", SerialLines::line_overflows(),
                " q_drops=", SerialLines::queue.overflows(),
                " resets=", Device::resets(),
                " setups=", Device::setups(),
                " stalls=", Device::stalls(),
                " usb_errors=", Usb::errors(),
                brio::crlf);
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

// WHICH VECTOR IS THE PREPROCESSOR'S QUESTION, the one place this
// program asks it: a handler is a symbol the image defines or does not,
// and the two controllers raise two different lines, so the part table
// states has_usbd for the preprocessor as well (parts/ch32v203c8.hpp).
#if BRIO_CH32VX03_HAS_USBD
// The controller's low-priority line, which every event of a
// single-buffered device arrives on; the high-priority one (shared with
// the CAN's transmit) belongs to double-buffered and isochronous
// endpoints, which this driver does not use.
extern "C" BRIO_CH32_INTERRUPT void usb_lp_can1_rx0_handler() {
    usb_trace.stamp('i', brio::usbd()->ISTR);
    usb_trace.stamp('e', brio::usbd()->EPR[0].R);
    Device::isr();
    usb_trace.stamp('E', brio::usbd()->EPR[0].R);
    usb_trace.stamp('t', static_cast<uint16_t>(brio::usbd_pma(2)));
    usb_trace.stamp('r', static_cast<uint16_t>(brio::usbd_pma(6)));
    if (Serial::take_rx_edge()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}
#else
// The host/device controller's one line, which every event of this
// block in device mode arrives on: the flags and the status as the
// handler finds them, then the stack.
extern "C" BRIO_CH32_INTERRUPT void usbfs_handler() {
    usb_trace.stamp('f', Usb::flags());
    usb_trace.stamp('s', Usb::status());
    Device::isr();
    if (Serial::take_rx_edge()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}
#endif

int main()
{
    const bool clock_ok = SysClock::init();
    const bool usb_ok = Usb::init(clock);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();
    if (usb_ok) {
        Device::start();   // the pull-up: the host enumerates from here
    }

    // The banner waits for a terminal: the first line goes out once the
    // host has configured the port and raised DTR. A spin, because
    // there is no kernel yet to idle in - and the controller's own
    // count would keep that idle awake anyway.
    while (!Serial::configured() || !Serial::dtr()) {
    }
    brio::print(serial, brio::crlf, brio::device::part_name, " brio console over USB (clk=",
                clock_ok ? "XT48" : "FAILED", ", usb=", usb_ok ? "48MHz" : "FAILED",
                ", tick=", tick_ok ? "STK" : "FAILED", "), type HELP", brio::crlf, "> ");

    brio::Tenuto<P, Console, SerialLines, Blinker>::run();
}
