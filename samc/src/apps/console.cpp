// console - the brio kernel console on the SAM C21: three active objects
// over the SERCOM5 UART, a port of the AVR project's src/apps/console.cpp.
//
//   SerialPort   turns RX bytes into LineReceived events (ping-pong line
//              buffers, backpressure on the ring - see util/serial_port.hpp)
//   Console  parses and routes each line, replies via blocking print
//              (bounded by the wire rate), drives the blinker BY POSTING
//   Blinker  owns the LED: heartbeat time event at 1 Hz, manual
//              LED ON|OFF|TOG commands arrive as posted SetLed events
//
// What did NOT change from the AVR app is the point: the three AOs, their
// events, their queues and every kernel and util header below them are
// the same source. Only the target glue differs - the clock type, the pin,
// the transport, and the vector bindings.
//
// ONE VECTOR, NOT TWO. The AVR app binds USART2_RXC_vect and
// USART2_DRE_vect separately; a SERCOM raises everything on a single NVIC
// line, so SERCOM5_Handler is the whole binding and Uart::isr() sorts out
// what is pending (samc/sercom.hpp says why).
//
// Kernel pack order is a CONTRACT here: Console (line consumer) must
// precede SerialPort (line producer) so the ping-pong buffers are always
// free when SerialPort runs - see the scheduling contract in serial_port.hpp.
//
// Wiring: none - the console is the board's own CH340 USB bridge on
// PB30/PB31. Connect at 115200 8N1 (any serial monitor, including
// VSCode's built-in one) and type:
//   HELP | LED ON|OFF|TOG | UPTIME | ERR
//
// Between keystrokes the CPU sleeps in WFI, woken by the SysTick tick or
// the SERCOM interrupt. No polling anywhere.
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "samc/clock.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"

using P = brio::SamPlatform;

// The clock: the ONE truth about CLK_CPU for every driver of this target
// (samc/clock.hpp). OSC48M undivided; `clock` is an empty tag passed to
// driver inits, and the SERCOM's core clock is GCLK generator 0, i.e.
// this same rate.
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using Led = brio::Pin<'B', 23>;  // PB23, the board's only LED

// The console pads. PB30 and PB31 are SERCOM5's PAD[0] and PAD[1] through
// PMUX function D, and the CH340 bridge hangs off them.
//
// WHICH ONE IS TX IS NOT A CHOICE. CTRLA.TXPO (DS60001479M 31.8.1) puts
// TxD on SERCOM PAD[0] or PAD[2] and nowhere else, so of this pair only
// PAD[0] = PB30 can transmit: PB30 is TX, PB31 is RX, and swapping them
// is not a configuration the silicon offers (samc/sercom.hpp refuses it
// at compile time). If the bench ever shows the two crossed, the fix is a
// DIFFERENT pad pair - SERCOM5 also reaches PAD[0]/PAD[1] on PB02/PB03
// and PB16/PB17 (function D and C respectively), and PAD[2]/PAD[3] on
// other pins still - not a swap of these two.
constexpr brio::UartPads console_pads{
    .tx = brio::SercomPad::pad0,
    .rx = brio::SercomPad::pad1,
    .tx_pin = {'B', 30, brio::PinFunction::d},
    .rx_pin = {'B', 31, brio::PinFunction::d},
};

// The device header's own I/O multiplexing symbols must agree with the
// claim above - the half of the pad-to-pin mapping that IS checkable
// without a per-package pad table (samc/sercom.hpp, "PADS ARE NOT PINS").
static_assert(MUX_PB30D_SERCOM5_PAD0 ==
              static_cast<uint8_t>(console_pads.tx_pin.function));
static_assert(MUX_PB31D_SERCOM5_PAD1 ==
              static_cast<uint8_t>(console_pads.rx_pin.function));

using Serial = brio::Uart<5, console_pads>;  // rings 64/256 (defaults)
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
                " baud=", Serial::actual_baud(SysClock::hz),
                brio::crlf);
}

} // namespace

// ---- target glue ------------------------------------------------------------
// ONE vector for the whole peripheral: isr() reads INTFLAG against
// INTENSET and serves whatever is genuinely pending, returning the RX
// ring's empty -> non-empty edge that wakes the line assembler.
extern "C" void SERCOM5_Handler() {
    if (Serial::isr()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main()
{
    const bool clock_ok = SysClock::init();       // OSC48M -> GCLK0 -> 48 MHz
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // Guarded: print() BLOCKS until the transport accepts each byte, so
    // printing into a UART that failed to come up would never return -
    // the LED heartbeat below is then the only sign of life, which is
    // exactly the diagnosis one wants at that point.
    if (serial_ok) {
        brio::print(serial, brio::crlf, "SAMC21J18A brio console (clk=",
                    clock_ok ? "OSC48M" : "FAILED", ", tick=",
                    tick_ok ? "SysTick" : "FAILED",
                    "), type HELP", brio::crlf, "> ");
    }

    // Pack order = priority AND correctness: the line CONSUMER (Console)
    // must precede the PRODUCER (SerialLines) - scheduling contract.
    brio::Kernel<P, Console, SerialLines, Blinker>::run();
}
