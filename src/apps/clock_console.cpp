// clock_console - the console app on a RUNTIME-VARIABLE clock: the
// bench test of brio::DynamicClock. Type CLOCK <div> and CLK_PER changes
// under the running program; if the console keeps talking, the rebase
// fan-out did its job (Serial drained its TX at the old rate and took a
// new BAUD before the switch), and if the LED keeps blinking at 1 Hz
// the kernel timebase (RTC/PIT, time events) did not even notice.
//
//   SysClock = DynamicClock<Boot, Serial>: Boot is the static 24 MHz
//   crystal configuration; Serial is the one clocked user here (a Twi
//   or Spi engine would be listed too). SysClock::set(hz) fans the new
//   rate out to the users, THEN reprograms the main prescaler. The
//   app speaks Hz; which prescaler produces them is the clock's detail.
//
// pio: monitor_speed = 115200
//
// Console @ 115200 (not 460800: the USART needs CLK_PER >= 16 x baud,
// so 115200 works down to 2 MHz; 460800 would stop at 8 MHz). CLOCK
// refuses a rate the clock cannot reach or the USART cannot follow.
//
// Commands: HELP | LED ON|OFF|TOG | UPTIME | ERR | CLOCK [<hz>|<n>M|<n>k]
// CLOCK alone prints the current rate; CLOCK 4M, CLOCK 500k, CLOCK
// 24000000 switch. Reachable rates are 24 MHz / {1,2,4,6,8,10,12,16,
// 24,32,48,64}; the console needs >= 2 MHz at 115200 (see below).
//
// A rate change happens INSIDE the Console dispatch, main context, in
// the middle of nothing: the reply is printed first (so it goes out at
// the old rate), then set() drains and switches. Bytes received during
// the switch may be garbled - type after the prompt comes back.

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

// The clock, runtime regime: Boot names the source (24 MHz crystal on
// PA0/PA1, OSCHF fallback), DynamicClock lists the users to rebase on
// every change. Declared after Serial below - it must know the type.
using Boot = brio::Clock<brio::ClockSource::crystal, 24'000'000>;

namespace {

using Led = brio::Pin<'F', 2>;
using Serial = brio::Uart<2, brio::Route::alt1>;  // rings 64/256 (defaults)
constexpr Serial serial;                          // tag for print(serial, ...)

using SysClock = brio::DynamicClock<Boot, Serial>;   // Serial rebased on set()
constexpr SysClock clock;
constexpr uint32_t baud = 115200;

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
                return transition(&manual);   // exit disarms the heartbeat
            },
            [](auto) { return unhandled(); }
        );
    }

    // Manual state: the LED belongs to the console commands.
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
        brio::print(s, "commands: HELP | LED ON|OFF|TOG | UPTIME | ERR | "
                       "CLOCK [<hz>|<n>M|<n>k]", brio::crlf);
    }

    // CLOCK: print the rate; CLOCK <rate>: switch CLK_PER to it. The
    // rate is Hz, or a number with M/k suffix (4M, 500k). The clock
    // itself decides whether it can reach it (source / a prescaler);
    // the app only checks the USART can keep the baud.
    static void cmd_clock(const Cmd& cmd, Serial s) {
        if (cmd.argument_count == 0) {
            brio::print(s, "CLK_PER = ", SysClock::hz(), " Hz", brio::crlf);
            return;
        }
        uint32_t next = 0;
        if (!parse_rate(cmd.arguments[0], next)) {
            brio::print(s, "usage: CLOCK [<hz>|<n>M|<n>k]  e.g. CLOCK 4M", brio::crlf);
            return;
        }
        if (!SysClock::can_run_at(next)) {
            brio::print(s, "refused: ", next, " Hz is not ", SysClock::source_hz,
                        " / an available prescaler", brio::crlf);
            return;
        }
        if (!Serial::can_baud(next, baud)) {
            brio::print(s, "refused: ", next, " Hz cannot do ", baud,
                        " baud (needs >= ", Serial::min_hz_for(baud), ")",
                        brio::crlf);
            return;
        }
        brio::print(s, "switching to ", next, " Hz", brio::crlf);
        // Reply is queued at the old rate; set() drains it (Serial::rebase
        // waits for TX idle), reprograms BAUD, then switches.
        SysClock::set(next);
        brio::print(s, "now at ", SysClock::hz(), " Hz", brio::crlf);
    }

    // "24000000", "24M", "500k" -> Hz. False on anything else.
    static bool parse_rate(const char* text, uint32_t& hz) {
        uint32_t v = 0;
        const char* p = text;
        if (*p < '0' || *p > '9') {
            return false;
        }
        while (*p >= '0' && *p <= '9') {
            v = v * 10u + static_cast<uint32_t>(*p - '0');
            ++p;
        }
        if (*p == 'M' || *p == 'm') { v *= 1'000'000u; ++p; }
        else if (*p == 'K' || *p == 'k') { v *= 1'000u; ++p; }
        if (*p != '\0') {
            return false;
        }
        hz = v;
        return true;
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
        {"CLOCK", cmd_clock},
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
    const bool xtal = SysClock::init();
    Serial::init(clock, baud);
    brio::Ticker::init();
    sei();

    brio::print(serial, brio::crlf, "AVR128DB48 brio clock console (src=",
                xtal ? "XTAL" : "OSCHF", ", ", SysClock::hz(), " Hz), type HELP", brio::crlf, "> ");

    // Pack order = priority AND correctness: the line CONSUMER (Console)
    // must precede the PRODUCER (SerialLines) - scheduling contract.
    brio::Kernel<P, Console, SerialLines, Blinker>::run();
}
