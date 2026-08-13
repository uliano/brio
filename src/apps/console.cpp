// console - interactive command console over USART2, full-stack validation
// of the dx framework: byte transport + line assembly + parsing + routing +
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
// A dx::Timer<Millis> blinks the LED at 1 Hz through a member-function
// callback bound with dx::bind<&Blinker::toggle> (compile-time trampoline).

#include <avr/io.h>
#include <avr/interrupt.h>
#include "clock.hpp"
#include "pin.hpp"
#include "uart.hpp"
#include "ticker.hpp"
#include "timer.hpp"
#include "print.hpp"
#include "proto/line_parser.hpp"

namespace {

using Led = dx::Pin<'F', 2>;
using Serial = dx::Uart<2, dx::Route::alt1>;
constexpr Serial serial;  // zero-cost tag for print(serial, ...)

using Parser = dx::ConsoleCommandParser<4>;
using Router = dx::CommandRouter<Serial, 4>;
using Cmd = Router::CommandType;

dx::LineAssembler<80> line_in;  // no hardware touched: safe as a global

struct Blinker {
    bool enabled = true;
    void toggle() {
        if (enabled) {
            Led::toggle();
        }
    }
};
Blinker blinker;

dx::Timer<dx::Millis> heartbeat(500, true, dx::bind<&Blinker::toggle>(&blinker));

void cmd_help(const Cmd &, Serial s) {
    dx::print(s, "commands: HELP | LED ON|OFF|TOG | UPTIME | ERR", dx::crlf);
}

void cmd_led(const Cmd &cmd, Serial s) {
    if (cmd.argument_count != 1) {
        dx::print(s, "usage: LED ON|OFF|TOG", dx::crlf);
        return;
    }
    const char *arg = cmd.arguments[0];
    if (dx::command_equals(arg, "ON")) {
        blinker.enabled = false;
        Led::set();
    } else if (dx::command_equals(arg, "OFF")) {
        blinker.enabled = false;
        Led::clear();
    } else if (dx::command_equals(arg, "TOG")) {
        blinker.enabled = false;
        Led::toggle();
    } else {
        dx::print(s, "usage: LED ON|OFF|TOG", dx::crlf);
        return;
    }
    dx::print(s, "OK", dx::crlf);
}

void cmd_uptime(const Cmd &, Serial s) {
    dx::TimeStamp ts;
    dx::Ticker::now(ts);
    dx::print(s, "uptime: ", ts, dx::crlf);
}

void cmd_err(const Cmd &, Serial s) {
    dx::print(s,
              "rx_overruns=", Serial::rx_overruns(),
              " frame=", Serial::frame_errors(),
              " parity=", Serial::parity_errors(),
              " hw_overruns=", Serial::hw_overruns(),
              " line_overflows=", line_in.overflow_count(),
              dx::crlf);
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
ISR(RTC_PIT_vect)    { dx::Ticker::pit(); }

int main() {
    const bool xtal = dx::init_clock_24mhz();
    Led::output();
    Serial::init(460800);
    dx::Ticker::init();
    sei();

    dx::print(serial, dx::crlf, "AVR128DB48 console (clk=",
              xtal ? "XTAL" : "OSCHF", "), type HELP", dx::crlf, "> ");
    heartbeat.start();

    Cmd cmd;
    for (;;) {
        uint8_t byte;
        while (Serial::read_byte(byte)) {
            if (char *line = line_in.push(byte)) {
                if (Parser::parse(line, cmd)) {
                    if (!Router::dispatch(cmd, routes, route_count, serial)) {
                        dx::print(serial, "unknown command (try HELP)", dx::crlf);
                    }
                }
                dx::print(serial, "> ");
            }
        }
        dx::Timer<dx::Millis>::check_all();
    }
}
