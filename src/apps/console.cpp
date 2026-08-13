// console - interactive command console over USART2, full-stack validation
// of the brio framework: byte transport + line assembly + parsing + routing +
// software timers, all resolved at compile time (no virtual dispatch).
//
// Connect with `pio device monitor -e console` (460800 8N1, CRLF or LF line
// endings both accepted) and type:
//
//   HELP            list the commands
//   LED ON|OFF|TOG  drive the PF2 LED (heartbeat stops when driven manually)
//   UPTIME          RTC timestamp since boot
//   ERR             UART + line-assembly error counters
//
// A brio::Timer<Millis> blinks the LED at 1 Hz through a member-function
// callback bound with brio::bind<&Blinker::toggle> (compile-time trampoline).

#include <avr/io.h>
#include <avr/interrupt.h>
#include "avrdx/clock.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/uart.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/timer.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"

namespace {

using Led = brio::Pin<'F', 2>;
using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;  // zero-cost tag for print(serial, ...)

using Parser = brio::ConsoleCommandParser<4>;
using Router = brio::CommandRouter<Serial, 4>;
using Cmd = Router::CommandType;

brio::LineAssembler<80> line_in;  // no hardware touched: safe as a global

struct Blinker {
    bool enabled = true;
    void toggle() {
        if (enabled) {
            Led::toggle();
        }
    }
};
Blinker blinker;

brio::Timer<brio::Millis> heartbeat(500, true, brio::bind<&Blinker::toggle>(&blinker));

void cmd_help(const Cmd &, Serial s) {
    brio::print(s, "commands: HELP | LED ON|OFF|TOG | UPTIME | ERR", brio::crlf);
}

void cmd_led(const Cmd &cmd, Serial s) {
    if (cmd.argument_count != 1) {
        brio::print(s, "usage: LED ON|OFF|TOG", brio::crlf);
        return;
    }
    const char *arg = cmd.arguments[0];
    if (brio::command_equals(arg, "ON")) {
        blinker.enabled = false;
        Led::set();
    } else if (brio::command_equals(arg, "OFF")) {
        blinker.enabled = false;
        Led::clear();
    } else if (brio::command_equals(arg, "TOG")) {
        blinker.enabled = false;
        Led::toggle();
    } else {
        brio::print(s, "usage: LED ON|OFF|TOG", brio::crlf);
        return;
    }
    brio::print(s, "OK", brio::crlf);
}

void cmd_uptime(const Cmd &, Serial s) {
    brio::TimeStamp ts;
    brio::Ticker::now(ts);
    brio::print(s, "uptime: ", ts, brio::crlf);
}

void cmd_err(const Cmd &, Serial s) {
    brio::print(s,
              "rx_overruns=", Serial::rx_overruns(),
              " frame=", Serial::frame_errors(),
              " parity=", Serial::parity_errors(),
              " hw_overruns=", Serial::hw_overruns(),
              " line_overflows=", line_in.overflow_count(),
              brio::crlf);
}

constexpr Router::Route routes[] = {
    {"HELP", cmd_help},
    {"LED", cmd_led},
    {"UPTIME", cmd_uptime},
    {"ERR", cmd_err},
};
constexpr uint8_t route_count = sizeof(routes) / sizeof(routes[0]);

}  // namespace

ISR(USART2_RXC_vect) { Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }

int main() {
    const bool xtal = brio::init_clock_24mhz();
    Led::output();
    Serial::init(460800);
    brio::Ticker::init();
    sei();

    brio::print(serial, brio::crlf, "AVR128DB48 console (clk=",
              xtal ? "XTAL" : "OSCHF", "), type HELP", brio::crlf, "> ");
    heartbeat.start();

    Cmd cmd;
    for (;;) {
        uint8_t byte;
        while (Serial::read_byte(byte)) {
            if (char *line = line_in.push(byte)) {
                if (Parser::parse(line, cmd)) {
                    if (!Router::dispatch(cmd, routes, route_count, serial)) {
                        brio::print(serial, "unknown command (try HELP)", brio::crlf);
                    }
                }
                brio::print(serial, "> ");
            }
        }
        brio::Timer<brio::Millis>::check_all();
    }
}
