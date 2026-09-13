// console_usb - the brio kernel console over the chip's own USB: the
// same three active objects as the console app (SerialPort, Console,
// Blinker), the transport a CDC ACM port on the RP2040's controller
// instead of a UART - nothing above the transport changed. The host
// sees a serial port (/dev/ttyACM* on Linux, no driver to install) and
// types the same commands:
//   HELP | LED ON|OFF|TOG | UPTIME | ERR | USB
// USB reports the device's state, address, the line coding the host
// set and DTR.
//
// The line coding is reported, not obeyed: a virtual port has no baud
// rate, so "115200" in a terminal is a number the device reads back.
// Between keystrokes the CPU sleeps in WFI, woken by the tick or the
// controller's interrupt. The identity is the pid.codes test pair
// 1209:0001, meant for exactly this.
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/sysinfo.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/usb.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

using P = brio::Rp2040Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

namespace {

using Led = brio::Pin<25>;

using Serial = brio::UsbCdcAcm<brio::Usb, P>;   // interface 0, endpoints 1 (notification) and 2 (bulk)
constexpr Serial serial;

struct Descriptors {
    static constexpr auto device = brio::usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        brio::usb_concat(brio::usb_configuration_head(9 + Serial::descriptor_bytes, Serial::interface_count), Serial::descriptors);
    static constexpr auto language = brio::usb_language_descriptor();
    static constexpr auto manufacturer = brio::usb_string_descriptor("brio");
    static constexpr auto product = brio::usb_string_descriptor("brio console");
    static constexpr auto serial_number = brio::usb_string_descriptor("rp2040");
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
using Device = brio::UsbDevice<brio::Usb, Descriptors, Serial>;

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
        const brio::UsbLineCoding c = Serial::line_coding();
        brio::print(s, "state=", static_cast<uint8_t>(Device::state()), " address=", Device::address(), " resets=", Device::resets(),
                    " setups=", Device::setups(), " stalls=", Device::stalls(), " suspends=", Device::suspends(),
                    " line=", c.baud, "/", c.data_bits, "/", c.parity, "/", c.stop_bits, " dtr=", Serial::dtr(), " rts=", Serial::rts(),
                    " frame=", brio::Usb::frame(), brio::crlf);
    }

    static void cmd_err(const Cmd&, Serial s);

    static constexpr Router::Route routes[] = {
        {"HELP", cmd_help}, {"LED", cmd_led}, {"UPTIME", cmd_uptime}, {"ERR", cmd_err}, {"USB", cmd_usb},
    };
    static constexpr uint8_t route_count = sizeof(routes) / sizeof(routes[0]);
};

using SerialLines = brio::SerialPort<Serial, P, Console, 80>;

void Console::cmd_err(const Cmd&, Serial s) {
    brio::print(s, "rx_overruns=", Serial::rx_overruns(), " rx_bytes=", Serial::rx_bytes(), " tx_bytes=", Serial::tx_bytes(),
                " line_overflows=", SerialLines::line_overflows(), " q_drops=", SerialLines::queue.overflows(),
                " sie_errors=", brio::hex(brio::Usb::take_errors()), brio::crlf);
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_usbctrl() {
    Device::isr();
    if (Serial::take_rx_edge()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    (void)SysClock::init();
    const bool usb_ok = brio::Usb::init(clock);
    (void)brio::Ticker::init(clock);
    brio::enable_interrupts();
    if (usb_ok) {
        Device::start();   // the pull-up: the host enumerates from here
    }
    // The banner waits for a terminal: the first line goes out once the
    // host has configured the port and raised DTR.
    while (!Serial::configured() || !Serial::dtr()) {
        P::CriticalSection cs;
        P::idle();
    }
    const brio::ChipId id = brio::ChipId::read();
    brio::print(serial, brio::crlf, "RP2040 rev B", id.revision, " brio console over USB (clk=PLL125, usb=", usb_ok ? "48MHz" : "FAILED",
                "), type HELP", brio::crlf, "> ");
    brio::Tenuto<P, Console, SerialLines, Blinker>::run();
}
