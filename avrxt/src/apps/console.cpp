// console - a line console on the brio kernel: three active objects
// and no polling loop anywhere.
//
//   SerialPort   turns RX bytes into LineReceived events (ping-pong line
//              buffers, backpressure on the ring - see util/serial_port.hpp)
//   Console  parses and routes each line, replies via blocking print
//              (bounded, ~us at 460800), drives the lamp BY POSTING
//   Lamp     owns the LED, dark from boot: the LED ON|OFF|TOG
//              commands arrive as posted SetLed events
//
// Tenuto pack order is a CONTRACT here: Console (line consumer) must
// precede SerialPort (line producer) so the ping-pong buffers are always
// free when SerialPort runs - see the scheduling contract in serial_port.hpp.
//
// Connect at 460800 8N1 (`brio console <board>` names the
// device, any serial monitor - incl. VSCode's built-in one - works) and type:
//   HELP | LED ON|OFF|TOG | UPTIME | ERR
//
// Between keystrokes the CPU sleeps in IDLE, woken by the PIT tick or
// the USART interrupts. No polling anywhere.

// build: boards = db48,da48,ea48

#include <avr/interrupt.h>
#include <stdint.h>
#include <variant>

#include "avrxt/clock.hpp"
#include "avrxt/pin.hpp"
#include "avrxt/platform.hpp"
#include "avrxt/ticker.hpp"
#include "avrxt/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"

#include "avr_board.hpp"

using P = brio::AvrPlatform;

// The clock: the ONE truth about CLK_PER for every driver of this
// target (avrxt/clock.hpp): the board's HF source at its rate (the DB
// board's 24 MHz crystal, the DA board's 24 MHz external clock, the EA
// board's 20 MHz crystal - avr_board.hpp), OSCHF fallback at the same
// CLK_PER; `clock` is an empty tag passed to driver inits.
using SysClock = BoardClock;   // the board's HF source (avr_board.hpp)
constexpr SysClock clock;

namespace {

using Led = brio::Pin<'F', 2>;
// The board's console (avr_board.hpp); the vectors bound below are
// USART2's, which the static_assert keeps honest.
using Serial = brio::Uart<board_console_usart, board_console_route>;  // rings 64/256 (defaults)
static_assert(board_console_usart == 2, "console.cpp binds USART2's vectors");
constexpr Serial serial;                          // tag for print(serial, ...)

// ---- events -----------------------------------------------------------------
struct SetLed { enum class Mode : uint8_t { on, off, tog } mode; };

// ---- the lamp: owns the LED, dark until a command lights it -----------------
struct Lamp : brio::Fsm<Lamp, SetLed> {
    static inline brio::EventQueue<Event, 2, P> queue;

    static void init() {
        Led::output();      // OUT resets low: the LED (PF2 to ground) dark
        start(&steady);
    }

    static Status steady(const Event& e) {
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

// ---- the console: parses lines, replies, commands the lamp ------------------
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
        brio::post<Lamp>(SetLed{mode});   // addressed command, AO to AO
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
                " torn=", SerialLines::torn_lines(),
                " skips=", Serial::rx_skips(),
                " q_drops=", SerialLines::queue.overflows(),
                brio::crlf);
}

} // namespace

// ---- target glue ------------------------------------------------------------
// FLATTEN: post() is out of line wherever it has a second call site, and
// a call in an AVR vector makes it save the whole call-clobbered set;
// flattened, the post is inlined into this vector alone and the vector
// saves the registers it uses (docs/avrxt/usart.md, the receive cost).
ISR(USART2_RXC_vect, ISR_FLATTEN) {
    if (Serial::rxc()) {                       // empty -> non-empty edge
        brio::post<SerialLines>(brio::RxActivity{});
    }
}
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    brio::Ticker::init();
    sei();

    if constexpr (brio::avr_die == brio::AvrDie::avr64ea) {
        brio::print(serial, brio::crlf, "AVR64EA48 brio console (clk=",
                    xtal ? "XTAL" : "OSCHF", ", silicon rev ", brio::hex(SYSCFG.REVID),
                    "), type HELP", brio::crlf, "> ");
    } else {
        brio::print(serial, brio::crlf,
                    brio::has_xoschf ? "AVR128DB48 brio console (clk=" : "AVR128DA48 brio console (clk=",
                    xtal ? (brio::has_xoschf ? "XTAL" : "EXTCLK") : "OSCHF", "), type HELP",
                    brio::crlf, "> ");
    }

    // Pack order = priority AND correctness: the line CONSUMER (Console)
    // must precede the PRODUCER (SerialLines) - scheduling contract.
    brio::Tenuto<P, Console, SerialLines, Lamp>::run();
}
