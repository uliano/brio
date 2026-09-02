// console - the brio kernel console on the STM32G0: three active objects
// over USART2, a port of the AVR and SAM projects' console.cpp.
//
//   SerialPort   turns RX bytes into LineReceived events (ping-pong line
//              buffers, backpressure on the ring - see util/serial_port.hpp)
//   Console  parses and routes each line, replies via blocking print
//              (bounded by the wire rate), drives the blinker BY POSTING
//   Blinker  owns the LED: heartbeat time event at 1 Hz, manual
//              LED ON|OFF|TOG commands arrive as posted SetLed events
//
// What did NOT change from the other two apps is the point: the three
// AOs, their events, their queues and every kernel and util header below
// them are the same source. Only the target glue differs - the clock
// type, the pin, the transport, and the vector binding.
//
// ONE VECTOR, SHARED. USART2 raises everything on one NVIC line that it
// shares with LPUART2 on this part (RM0444 table 61), so the handler
// name is USART2_LPUART2_IRQHandler and Uart::isr() sorts out what is
// pending; an app using LPUART2 as well would ask that one next.
//
// Kernel pack order is a CONTRACT here: Console (line consumer) must
// precede SerialPort (line producer) so the ping-pong buffers are always
// free when SerialPort runs - see the scheduling contract in serial_port.hpp.
//
// Wiring: none - the console is the Nucleo's own ST-LINK virtual COM
// port on PA2 (TX) / PA3 (RX) = USART2 AF1 (DS13560 table 13). Connect
// at 115200 8N1 and type:
//   HELP | LED ON|OFF|TOG | UPTIME | ERR
//
// Between keystrokes the CPU sleeps in WFI, woken by the SysTick tick or
// the USART interrupt. No polling anywhere.
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"

using P = brio::Stm32Platform;

// The clock: HSI16 through the PLL to 64 MHz, the ONE truth about SYSCLK
// (stm32g0/clock.hpp); PCLK == SYSCLK (both prescalers at 1), and the
// USART's kernel clock is PCLK.
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using Led = brio::Pin<'A', 5>;  // PA5 = LD4

// The console pads: USART2_TX on PA2 and USART2_RX on PA3, both at AF1
// (DS13560 table 13 - the datasheet is the authority; the device header
// carries no pin table to static_assert against, unlike the SAM DFP).
constexpr brio::UartPins console_pins{
    .tx = {'A', 2, brio::PinFunction::af1},
    .rx = {'A', 3, brio::PinFunction::af1},
};

using Serial = brio::Uart<2, console_pins>;  // rings 64/256 (defaults)
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
                " noise=", Serial::noise_errors(),
                " hw_overruns=", Serial::hw_overruns(),
                " line_overflows=", SerialLines::line_overflows(),
                " q_drops=", SerialLines::queue.overflows(),
                " baud=", Serial::actual_baud(SysClock::pclk_hz),
                brio::crlf);
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void USART2_LPUART2_IRQHandler() {
    if (Serial::isr()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main()
{
    const bool clock_ok = SysClock::init();       // HSI16 -> PLL -> 64 MHz
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // Guarded: print() BLOCKS until the transport accepts each byte, so
    // printing into a UART that failed to come up would never return -
    // the LED heartbeat below is then the only sign of life.
    if (serial_ok) {
        brio::print(serial, brio::crlf, "STM32G0B1RE brio console (clk=",
                    clock_ok ? "PLL64" : "FAILED", ", tick=",
                    tick_ok ? "SysTick" : "FAILED",
                    "), type HELP", brio::crlf, "> ");
    }

    brio::Kernel<P, Console, SerialLines, Blinker>::run();
}
