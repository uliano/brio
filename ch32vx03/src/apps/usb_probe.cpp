// usb_probe - the USB device controller under a UART console: the
// instrument of this stratum's USB bring-up.
//
// The same stack as console_usb (UsbDevice over Usbd with a CDC ACM
// class), but the CONSOLE IS THE UART, on the probe's own serial pins -
// so the device can be asked what it is doing while the host tries to
// enumerate it, instead of the answer being read out of its registers
// with a debugger that halts the core to do it. Commands:
//
//   USB     the stack's state and counters, the controller's registers
//   PMA     the buffer description table and the two endpoint buffers
//   TRACE   one line per USB interrupt: the status register, the
//           endpoint register before and after the stack ran, and the
//           two counts (the ring holds the last fifteen interrupts)
//   MARK    clear the trace, to catch one enumeration attempt alone
//   CONN    drop the pull-up and raise it again - a re-attach the host
//           answers with a fresh enumeration, on demand
//   IDLE    what the loop does when it has nothing to do: SPIN, WFI
//           (the bare instruction with interrupts enabled) or WFE (the
//           platform's own idle path) - the question being which of
//           them the USB engine survives. WFE now measures the
//           MECHANISM rather than the failure: that path counts the
//           attached controller as a bus master and does not sleep
//           while it is there (ch32vx03/bus_activity.hpp), so what
//           still puts the core to sleep under a host is WFI alone
//
// Wiring: a WeAct CH32V203C8T6 core board, the probe's TX/RX on
// PA9/PA10 and its debug port on PA13/PA14, the board's USB-C in the
// host. The clock is the board's 8 MHz crystal through the PLL at 48
// MHz, which the USB divider then takes whole.
//
// build: boards = v203c6,v203c8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usart.hpp"
#include "ch32vx03/usb.hpp"
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
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000, 8'000'000>;
constexpr SysClock clock;

using Serial = brio::Uart<1, P>;
constexpr Serial serial;

using Usb = brio::Usbd<>;
using Cdc = brio::UsbCdcAcm<Usb, P, 0, 1, 2, 128, 128>;

namespace {

using Led = brio::Pin<'B', 2>;

struct Descriptors {
    static constexpr auto device =
        brio::usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        brio::usb_concat(brio::usb_configuration_head(9 + Cdc::descriptor_bytes, Cdc::interface_count),
                         Cdc::descriptors);
    static constexpr auto language = brio::usb_language_descriptor();
    static constexpr auto manufacturer = brio::usb_string_descriptor("brio");
    static constexpr auto product = brio::usb_string_descriptor("brio usb probe");
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

}  // namespace

using Device = brio::UsbDevice<Usb, Descriptors, Cdc>;

/// One mark per USB interrupt, five per entry (see the header).
brio::Trace<75, P> usb_trace;

namespace {

/// What the loop does with an empty queue (the IDLE command).
enum class IdleMode : uint8_t { spin, wfi, wfe };
volatile IdleMode idle_mode = IdleMode::spin;

struct Beat {};

struct Console : brio::Fsm<Console, brio::LineReceived, Beat> {
    static inline brio::EventQueue<Event, 4, P> queue;
    static inline brio::TimeEvent<P, Console, Beat> heartbeat{Beat{}};

    static void init() {
        Led::output();
        start(&running);
    }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                heartbeat.arm_every(brio::ticks_from_ms<P>(500));
                return handled();
            },
            [](brio::Exit) { heartbeat.disarm(); return handled(); },
            [](Beat) { Led::toggle(); return handled(); },
            [](brio::LineReceived l) { handle_line(l.line.get()); return handled(); },
            [](auto) { return unhandled(); });
    }

private:
    using Parser = brio::ConsoleCommandParser<4>;
    using Router = brio::CommandRouter<Serial, 4>;
    using Cmd = Router::CommandType;

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
        brio::print(s, "commands: HELP | USB | PMA | TRACE | MARK | CONN | IDLE SPIN|WFI|WFE", brio::crlf);
    }

    static void cmd_usb(const Cmd&, Serial s) {
        brio::print(s, "state=", static_cast<uint8_t>(Device::state()),
                    " addr=", Usb::address(),
                    " frame=", Usb::frame(),
                    " resets=", Device::resets(),
                    " setups=", Device::setups(),
                    " stalls=", Device::stalls(),
                    " req=", brio::hex(Device::last_request()),
                    brio::crlf);
        brio::print(s, "  errors=", Usb::errors(), " overruns=", Usb::overruns(),
                    " pma_used=", Usb::buffer_used(),
                    " cdc_cfg=", Cdc::configured() ? 1 : 0,
                    " dtr=", Cdc::dtr() ? 1 : 0,
                    " cdc_rx=", Cdc::rx_bytes(), " cdc_tx=", Cdc::tx_bytes(),
                    brio::crlf);
        brio::print(s, "  CNTR=", brio::hex(Usb::regs().CNTR),
                    " ISTR=", brio::hex(Usb::regs().ISTR),
                    " DADDR=", brio::hex(Usb::regs().DADDR),
                    " EPR0=", brio::hex(Usb::regs().EPR[0].R),
                    " EPR1=", brio::hex(Usb::regs().EPR[1].R),
                    " EPR2=", brio::hex(Usb::regs().EPR[2].R),
                    brio::crlf);
    }

    static void cmd_pma(const Cmd&, Serial s) {
        for (uint8_t n = 0; n < 3u; ++n) {
            brio::print(s, "  ep", n,
                        " tx=", brio::hex(brio::usbd_pma(static_cast<uint16_t>(8u * n + 0u))),
                        " count_tx=", brio::hex(brio::usbd_pma(static_cast<uint16_t>(8u * n + 2u))),
                        " rx=", brio::hex(brio::usbd_pma(static_cast<uint16_t>(8u * n + 4u))),
                        " count_rx=", brio::hex(brio::usbd_pma(static_cast<uint16_t>(8u * n + 6u))),
                        brio::crlf);
        }
        // The control endpoint's two buffers, four halfwords each: the
        // setup packet the host sent and the answer staged for it.
        brio::print(s, "  tx:");
        for (uint16_t i = 0; i < 4u; ++i) {
            brio::print(s, " ", brio::hex(brio::usbd_pma(static_cast<uint16_t>(64u + 2u * i))));
        }
        brio::print(s, brio::crlf, "  rx:");
        for (uint16_t i = 0; i < 4u; ++i) {
            brio::print(s, " ", brio::hex(brio::usbd_pma(static_cast<uint16_t>(128u + 2u * i))));
        }
        brio::print(s, brio::crlf);
    }

    static void cmd_trace(const Cmd&, Serial s) { usb_trace.dump(s); }

    static void cmd_mark(const Cmd&, Serial s) {
        usb_trace.clear();
        brio::print(s, "trace cleared", brio::crlf);
    }

    static void cmd_idle(const Cmd& cmd, Serial s) {
        const char* arg = (cmd.argument_count == 1) ? cmd.arguments[0] : "";
        if (brio::command_equals(arg, "SPIN")) {
            idle_mode = IdleMode::spin;
        } else if (brio::command_equals(arg, "WFI")) {
            idle_mode = IdleMode::wfi;
        } else if (brio::command_equals(arg, "WFE")) {
            idle_mode = IdleMode::wfe;
        } else {
            brio::print(s, "usage: IDLE SPIN|WFI|WFE", brio::crlf);
            return;
        }
        brio::print(s, "OK", brio::crlf);
    }

    static void cmd_conn(const Cmd&, Serial s) {
        // Long enough for the host to see the device go: a disconnect
        // is a hundred microseconds of idle on the line, and ten
        // milliseconds is past every doubt.
        Usb::connect(false);
        const uint32_t until = brio::Ticker::millis() + 10u;
        while (brio::Ticker::millis() < until) {
        }
        Usb::connect(true);
        brio::print(s, "re-attached", brio::crlf);
    }

    static constexpr Router::Route routes[] = {
        {"HELP", cmd_help}, {"USB", cmd_usb}, {"PMA", cmd_pma},
        {"TRACE", cmd_trace}, {"MARK", cmd_mark}, {"CONN", cmd_conn}, {"IDLE", cmd_idle},
    };
    static constexpr uint8_t route_count = sizeof(routes) / sizeof(routes[0]);
};

using SerialLines = brio::SerialPort<Serial, P, Console, 64>;


}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() {
    if (Serial::isr()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}

extern "C" BRIO_CH32_INTERRUPT void usb_lp_can1_rx0_handler() {
    usb_trace.stamp('i', brio::usbd()->ISTR);
    usb_trace.stamp('e', brio::usbd()->EPR[0].R);
    Device::isr();
    usb_trace.stamp('E', brio::usbd()->EPR[0].R);
    usb_trace.stamp('t', static_cast<uint16_t>(brio::usbd_pma(2)));
    usb_trace.stamp('r', static_cast<uint16_t>(brio::usbd_pma(6)));
}

int main()
{
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool usb_ok = Usb::init(clock);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();
    if (usb_ok) {
        Device::start();
    }
    if (serial_ok) {
        brio::print(serial, brio::crlf, brio::device::part_name, " usb probe (clk=",
                    clock_ok ? "XT48" : "FAILED", ", usb=", usb_ok ? "48MHz" : "FAILED",
                    ", tick=", tick_ok ? "STK" : "FAILED", "), type HELP", brio::crlf, "> ");
    }
    // THE LOOP DOES NOT SLEEP HERE, on purpose: this image is the
    // instrument of a bring-up, and whether the USB engine can reach
    // the packet memory while the core is in WFI is one of the things
    // it is asking. step() alone is run() without the idle.
    using Kernel = brio::Tenuto<P, Console, SerialLines>;
    Kernel::init_all();
    for (;;) {
        Kernel::step();
        switch (idle_mode) {
            case IdleMode::spin:
                break;
            case IdleMode::wfi:
                // The bare instruction with interrupts ENABLED: not the
                // kernel's idle (which enters masked), just the core
                // asleep between events.
                __asm__ volatile("wfi" ::: "memory");
                break;
            case IdleMode::wfe:
                // The kernel's own idle path, which does not sleep
                // while the controller is attached: this mode is the
                // mechanism under test and no longer a sleep.
                Kernel::idle_if_empty();
                break;
        }
    }
}
