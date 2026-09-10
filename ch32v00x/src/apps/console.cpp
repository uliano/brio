// console - the brio kernel console on the CH32V00x: three active
// objects over USART1, and the proof that the kernel this repository is
// built around runs unchanged on a RISC-V core with sixteen registers
// and eight kilobytes of RAM.
//
//   SerialPort  turns RX bytes into LineReceived events (ping-pong line
//               buffers, backpressure on the ring - util/serial_port.hpp)
//   Console     parses and routes each line, replies via blocking print,
//               owns a 1 Hz time event whose count says the kernel's
//               timers are alive, and drives the blinker BY POSTING
//   Blinker     owns the LED: heartbeat time event at 1 Hz, manual
//               LED ON|OFF|TOG commands arrive as posted SetLed events
//
// Everything above the glue is portable: the AOs, their events, their
// queues, and every kernel and util header below them - the same files
// the AVR, the SAM C21 and the STM32G0 compile. Only the last lines are
// this target's: the clock type, the transport, and the two vector
// bindings.
//
// Kernel pack order is a CONTRACT here: Console (line consumer) must
// precede SerialPort (line producer) so the ping-pong buffers are always
// free when SerialPort runs - see the scheduling contract in
// serial_port.hpp.
//
// Wiring: the WCH-Link's own serial pins are wired to PD5 (USART1_TX)
// and PD6 (USART1_RX) on this board, so the debug cable carries the
// console too; the module's LED is jumpered to PC0 (blink.cpp says
// why that pin). Connect at 115200 8N1 and type:
//   HELP | LED ON|OFF|TOG | UPTIME | BEATS | ERR
//
// Between keystrokes the CPU sleeps in WFI, woken by the system counter
// or the USART. No polling anywhere.
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"

using P = brio::Ch32v00xPlatform<>;

// The clock: HSI 24 MHz through the PLL, which on this family only
// doubles - 48 MHz, the top of the part's range and the ONE truth about
// HCLK every driver below derives from.
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using Serial = brio::Uart<1, P>;   // USART1 on PD5/PD6, rings 64/64
constexpr Serial serial;           // tag for print(serial, ...)

constexpr uint32_t console_baud = 115200;

using Led = brio::Pin<'C', 0>;     // the jumpered LED

// ---- events -----------------------------------------------------------------
struct Beat {};                                    // Console's own heartbeat
struct Toggle {};                                  // Blinker's heartbeat
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

// ---- the console: parses lines, replies, and counts its own heartbeat -------
struct Console : brio::Fsm<Console, brio::LineReceived, Beat> {
    static inline brio::EventQueue<Event, 4, P> queue;
    static inline brio::TimeEvent<P, Console, Beat> heartbeat{Beat{}};
    static inline uint32_t beats = 0;

    static void init() { start(&running); }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                heartbeat.arm_every(brio::ticks_from_ms<P>(1000));
                return handled();
            },
            [](brio::Exit) {
                heartbeat.disarm();
                return handled();
            },
            [](Beat) {
                ++beats;
                return handled();
            },
            [](brio::LineReceived l) {
                handle_line(l.line.get());
                return handled();
            },
            [](auto) { return unhandled(); }
        );
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
        brio::print(s, "commands: HELP | LED ON|OFF|TOG | UPTIME | BEATS | ERR",
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

    static void cmd_beats(const Cmd&, Serial s) {
        brio::print(s, "beats: ", beats, brio::crlf);
    }

    static void cmd_err(const Cmd&, Serial s);   // needs SerialLines below

    static constexpr Router::Route routes[] = {
        {"HELP", cmd_help},
        {"LED", cmd_led},
        {"UPTIME", cmd_uptime},
        {"BEATS", cmd_beats},
        {"ERR", cmd_err},
    };
    static constexpr uint8_t route_count = sizeof(routes) / sizeof(routes[0]);
};

// ---- the serial line producer ----------------------------------------------
using SerialLines = brio::SerialPort<Serial, P, Console, 64>;

void Console::cmd_err(const Cmd&, Serial s) {
    brio::print(s,
                "rx_overruns=", Serial::rx_overruns(),
                " frame=", Serial::frame_errors(),
                " parity=", Serial::parity_errors(),
                " noise=", Serial::noise_errors(),
                " hw_overruns=", Serial::hw_overruns(),
                " line_overflows=", SerialLines::line_overflows(),
                " q_drops=", SerialLines::queue.overflows(),
                " baud=", Serial::actual_baud(SysClock::pclk_hz),
                brio::crlf);
}

} // namespace

// ---- target glue ------------------------------------------------------------
// The two vectors this program owns. BRIO_CH32_INTERRUPT is the one
// spelling of the handler attribute on this target - the hardware
// prologue's or gcc's, whichever the image was built for (pfic.hpp).
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() {
    if (Serial::isr()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}

int main()
{
    const bool clock_ok = SysClock::init();            // HSI x2 -> 48 MHz
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // Guarded: print() BLOCKS until the transport accepts each byte, so
    // printing into a port that failed to come up would never return.
    if (serial_ok) {
        brio::print(serial, brio::crlf, "CH32V006K8 brio console (clk=",
                    clock_ok ? "PLL48" : "FAILED", ", tick=",
                    tick_ok ? "STK" : "FAILED",
                    "), type HELP", brio::crlf, "> ");
    }

    brio::Kernel<P, Console, SerialLines, Blinker>::run();
}
