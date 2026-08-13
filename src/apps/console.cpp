// console - the console migrated to the brio kernel: three active
// objects replacing the old polling main loop.
//
//   SerialPort   turns RX bytes into LineReceived events (ping-pong line
//              buffers, backpressure on the ring - see util/serial_port.hpp)
//   Console  parses and routes each line, replies via blocking print
//              (bounded, ~us at 460800), drives the blinker BY POSTING
//   Blinker  owns the LED: heartbeat time event at 1 Hz, manual
//              LED ON|OFF|TOG commands arrive as posted SetLed events
//
// Kernel pack order is a CONTRACT here: Console (line consumer) must
// precede SerialPort (line producer) so the ping-pong buffers are always
// free when SerialPort runs - see the scheduling contract in serial_port.hpp.
//
// Connect with `pio device monitor -e console` (460800 8N1) and type:
//   HELP | LED ON|OFF|TOG | UPTIME | ERR
//
// Between keystrokes the CPU sleeps in IDLE, woken by the PIT tick or
// the USART interrupts. No polling anywhere.

#include <avr/interrupt.h>
#include <stdint.h>
#include <variant>

#include "avrdx/clock.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/uart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"

using P = brio::AvrPlatform;

namespace {

template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };

using Led = brio::Pin<'F', 2>;
using Serial = brio::Uart<2, brio::Route::alt1>;  // rings 64/256 (defaults)
constexpr Serial serial;                          // tag for print(serial, ...)

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

    // Heartbeat state: 1 Hz toggle until a manual command takes over.
    static Status beating(const Event& e) {
        return std::visit(overloaded{
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
                return transition(&manual);   // exit disarms the heartbeat
            },
            [](auto) { return unhandled(); },
        }, e);
    }

    // Manual state: the LED belongs to the console commands.
    static Status manual(const Event& e) {
        return std::visit(overloaded{
            [](SetLed s) { apply(s); return handled(); },
            [](auto)     { return unhandled(); },
        }, e);
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
        return std::visit(overloaded{
            [](brio::Entry) { return handled(); },
            [](brio::LineReceived l) {
                handle_line(l.line);
                return handled();
            },
            [](auto) { return unhandled(); },
        }, e);
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
        brio::post<Blinker>(SetLed{mode});   // addressed command, AO to AO
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
                " hw_overruns=", Serial::hw_overruns(),
                " line_overflows=", SerialLines::line_overflows(),
                " q_drops=", SerialLines::queue.overflows(),
                brio::crlf);
}

} // namespace

// ---- target glue ------------------------------------------------------------
ISR(USART2_RXC_vect) {
    if (Serial::rxc()) {                       // empty -> non-empty edge
        brio::post<SerialLines>(brio::RxActivity{});
    }
}
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }

int main() {
    const bool xtal = brio::init_clock_24mhz();
    Serial::init(460800);
    brio::Ticker::init();
    sei();

    brio::print(serial, brio::crlf, "AVR128DB48 brio console (clk=",
                xtal ? "XTAL" : "OSCHF", "), type HELP", brio::crlf, "> ");

    // Pack order = priority AND correctness: the line CONSUMER (Console)
    // must precede the PRODUCER (SerialLines) - scheduling contract.
    brio::Kernel<P, Console, SerialLines, Blinker>::run();
}
