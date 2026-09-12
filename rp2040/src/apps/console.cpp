// console - the brio kernel console: three active objects over UART0.
//
//   SerialPort   turns RX bytes into LineReceived events (ping-pong line
//              buffers, backpressure on the ring - see util/serial_port.hpp)
//   Console  parses and routes each line, replies via blocking print
//              (bounded by the wire rate), drives the blinker BY POSTING
//   Blinker  owns the LED: heartbeat time event at 1 Hz, manual
//              LED ON|OFF|TOG commands arrive as posted SetLed events
//
// Everything above the target glue is portable: the three AOs, their
// events, their queues and every kernel and util header below them.
// Only the glue lines are this target's - the clock type, the pin, the
// transport, and the vector bindings.
//
// Kernel pack order is a CONTRACT here: Console (line consumer) must
// precede SerialPort (line producer) so the ping-pong buffers are always
// free when SerialPort runs - see the scheduling contract in serial_port.hpp.
//
// Wiring: the Debug Probe's UART bridge on GP0 (TX) / GP1 (RX) = UART0
// under function 2, crossed. Connect at 115200 8N1 and type:
//   HELP | LED ON|OFF|TOG | UPTIME | ERR
//
// The LED is GP25, on a Pico and on a WeAct board alike.
//
// Between keystrokes the CPU sleeps in WFI, woken by the SysTick tick or
// the UART interrupt. No polling anywhere.
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/sysinfo.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/uart.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"

using P = brio::Rp2040Platform<>;

// The clock: the 12 MHz crystal through the PLL to 125 MHz, the ONE
// truth about clk_sys (rp2040/clock.hpp); clk_peri == clk_sys, and the
// UART's clock is clk_peri.
using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

namespace {

using Led = brio::Pin<25>;

constexpr brio::UartPins console_pins{
    .tx = {0, brio::PinFunction::uart},
    .rx = {1, brio::PinFunction::uart},
};

using Serial = brio::Uart<0, console_pins>;  // rings 64/256 (defaults)
constexpr Serial serial;                     // tag for print(serial, ...)

constexpr uint32_t console_baud = 115200;

// ---- events -----------------------------------------------------------------
struct Toggle {};                                  // Blinker heartbeat
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
            [](brio::Entry) {
                heartbeat.arm_every(brio::ticks_from_ms<P>(500));
                return handled();
            },
            [](brio::Exit) {
                heartbeat.disarm();
                return handled();
            },
            [](Toggle) {
                Led::toggle();
                return handled();
            },
            [](SetLed s) {
                apply(s);
                return transition(&manual);
            },
            [](auto) { return unhandled(); }
        );
    }

    static Status manual(const Event& e) {
        return brio::match(e,
            [](SetLed s) { apply(s); return handled(); },
            [](auto)     { return unhandled(); }
        );
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

// ---- the console: parses lines, replies, commands the blinker ---------------
struct Console : brio::Fsm<Console, brio::LineReceived> {
    static inline brio::EventQueue<Event, 2, P> queue;

    using Parser = brio::ConsoleCommandParser<4>;
    using Router = brio::CommandRouter<Serial, 4>;
    using Cmd = Router::CommandType;

    static void init() { start(&running); }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](brio::LineReceived l) {
                handle_line(l.line.get());
                return handled();
            },
            [](auto) { return unhandled(); }
        );
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
        brio::print(s, "commands: HELP | LED ON|OFF|TOG | UPTIME | ERR",
                    brio::crlf);
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

    static void cmd_err(const Cmd&, Serial s);   // needs SerialLines below

    static constexpr Router::Route routes[] = {
        {"HELP", cmd_help},
        {"LED", cmd_led},
        {"UPTIME", cmd_uptime},
        {"ERR", cmd_err},
    };
    static constexpr uint8_t route_count =
        sizeof(routes) / sizeof(routes[0]);
};

// ---- the serial line producer ----------------------------------------------
using SerialLines = brio::SerialPort<Serial, P, Console, 80>;

void Console::cmd_err(const Cmd&, Serial s) {
    brio::print(s,
                "rx_overruns=", Serial::rx_overruns(),
                " frame=", Serial::frame_errors(),
                " parity=", Serial::parity_errors(),
                " break=", Serial::break_errors(),
                " hw_overruns=", Serial::hw_overruns(),
                " line_overflows=", SerialLines::line_overflows(),
                " q_drops=", SerialLines::queue.overflows(),
                " baud=", Serial::actual_baud(SysClock::pclk_hz),
                brio::crlf);
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() {
    if (Serial::isr()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main()
{
    const bool clock_ok = SysClock::init();       // XOSC -> PLL -> 125 MHz
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // Guarded: print() BLOCKS until the transport accepts each byte, so
    // printing into a UART that failed to come up would never return -
    // the LED heartbeat below is then the only sign of life.
    if (serial_ok) {
        const brio::ChipId id = brio::ChipId::read();
        brio::print(serial, brio::crlf, "RP2040 rev B", id.revision, " brio console (clk=",
                    clock_ok ? "PLL125" : "FAILED", ", tick=",
                    tick_ok ? "SysTick" : "FAILED",
                    "), type HELP", brio::crlf, "> ");
    }

    brio::Kernel<P, Console, SerialLines, Blinker>::run();
}
