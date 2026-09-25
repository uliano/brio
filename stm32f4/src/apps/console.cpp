// console - the brio kernel console: three active objects over a USART.
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
// transport, and the vector binding.
//
// Tenuto pack order is a CONTRACT here: Console (line consumer) must
// precede SerialPort (line producer) so the ping-pong buffers are always
// free when SerialPort runs - see the scheduling contract in serial_port.hpp.
//
// Wiring: none on the ST boards - the console is the board's own
// ST-LINK virtual COM port: USART1 on PA9 (TX) / PA10 (RX) at AF7 on the
// STM32F429I-DISC1 (UM1670, through SB11/SB15), USART2 on PA2 (TX) /
// PA3 (RX) at AF7 on the Nucleo-F446RE (UM1724), USART3 on PB10 (TX) /
// PB11 (RX) at AF7 on the 32F469IDISCOVERY (UM1932 4.11). On the black
// pill the console is USART1 on PA9/PA10 too, wired to a probe's UART
// bridge: the board's PA9 to the bridge's RX, PA10 to its TX. The AF
// numbers are the datasheets' (DS10693 table 11, the F429's table 12,
// the F411's table 9, DS11189 table 12). Connect at 115200 8N1 and type:
//   HELP | LED ON|OFF|TOG | UPTIME | CLK | ERR
//
// Between keystrokes the CPU sleeps in WFI, woken by the SysTick tick or
// the USART interrupt. No polling anywhere.
//
// build: boards = f429zi,f446re,f411ce,f469ni
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/flash.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/pwr.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"

using P = brio::Stm32f4Platform<>;

// The clock: HSE through the PLL to the part's ceiling, the ONE truth
// about SYSCLK (stm32f4/clock.hpp); the two APB rates follow from it and
// the USART's divisor divides its own bus's. HSE is the ST-LINK's 8 MHz
// MCO in bypass on the Nucleo-F446RE, the 8 MHz crystal X3 on the
// STM32F429I-DISC1 (UM1670 7.12.1: the MCO route needs SB18, open by
// default), the 8 MHz crystal X2 on the 32F469IDISCOVERY (UM1932 4.3.1),
// the 25 MHz crystal on the black pill.
#if defined(STM32F411xE)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
#elif defined(STM32F429xx) || defined(STM32F469xx)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
#else
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
#endif
constexpr SysClock clock;

namespace {

// One board carries one part (stm32f4/CMakeLists.txt's rule), so the
// part selects the LED, the console instance and its pads. Which device
// header is compiled is the one question only the preprocessor can ask.
#if defined(STM32F429xx)
using Led = brio::Pin<'G', 13>;   // LD3 on the STM32F429I-DISC1
constexpr brio::UartPins console_pins{
    .tx = {'A', 9, brio::PinFunction::af7},    // USART1_TX
    .rx = {'A', 10, brio::PinFunction::af7},   // USART1_RX
};
constexpr uint8_t console_instance = 1;
constexpr const char* banner_head = "STM32F429ZI brio console (clk=";
#elif defined(STM32F469xx)
using Led = brio::Pin<'G', 6>;    // LD1 on the 32F469IDISCOVERY, lit when low
constexpr brio::UartPins console_pins{
    .tx = {'B', 10, brio::PinFunction::af7},   // USART3_TX
    .rx = {'B', 11, brio::PinFunction::af7},   // USART3_RX
};
constexpr uint8_t console_instance = 3;
constexpr const char* banner_head = "STM32F469NI brio console (clk=";
#elif defined(STM32F411xE)
using Led = brio::Pin<'C', 13>;   // the black pill's LED, lit when low
constexpr brio::UartPins console_pins{
    .tx = {'A', 9, brio::PinFunction::af7},
    .rx = {'A', 10, brio::PinFunction::af7},
};
constexpr uint8_t console_instance = 1;
constexpr const char* banner_head = "STM32F411CE brio console (clk=";
#else
using Led = brio::Pin<'A', 5>;    // LD2 on the Nucleo-F446RE
constexpr brio::UartPins console_pins{
    .tx = {'A', 2, brio::PinFunction::af7},    // USART2_TX
    .rx = {'A', 3, brio::PinFunction::af7},    // USART2_RX
};
constexpr uint8_t console_instance = 2;
constexpr const char* banner_head = "STM32F446RE brio console (clk=";
#endif

using Serial = brio::Uart<console_instance, console_pins>;  // rings 64/256 (defaults)
constexpr Serial serial;                                    // tag for print(serial, ...)

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
        brio::print(s, "commands: HELP | LED ON|OFF|TOG | UPTIME | CLK | ERR",
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

    // What the tree really holds, read back from the registers: the
    // proof the clock task's sequence landed where its constants say.
    static void cmd_clk(const Cmd&, Serial s) {
        brio::print(s, "sysclk=", SysClock::hz, " pclk1=", SysClock::pclk1_hz,
                    " pclk2=", SysClock::pclk2_hz, " usb=", SysClock::usb_hz,
                    " sws=", static_cast<uint8_t>(brio::Rcc::sysclk_status()),
                    " pll=", brio::Rcc::pll_ready() ? "locked" : "OFF",
                    " hse=", brio::Rcc::hse_ready() ? (brio::Rcc::hse_bypassed() ? "bypass" : "crystal") : "OFF",
                    " vos=scale", static_cast<uint8_t>(brio::Pwr::scale()),
                    " od=", brio::Pwr::over_drive_active() ? "on" : "off",
                    " ws=", brio::FlashWaitStates::get(),
                    " ppre1=", brio::Rcc::apb1_divider(), " ppre2=", brio::Rcc::apb2_divider(),
                    brio::crlf);
    }

    static void cmd_err(const Cmd&, Serial s);   // needs SerialLines below

    static constexpr Router::Route routes[] = {
        {"HELP", cmd_help},
        {"LED", cmd_led},
        {"UPTIME", cmd_uptime},
        {"CLK", cmd_clk},
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
                " baud=", Serial::actual_baud(Serial::kernel_hz<SysClock>()),
                brio::crlf);
}

} // namespace

// ---- target glue ------------------------------------------------------------
// Every serial instance of this family has a vector of its own; the
// part decides which one the console binds.
#if defined(STM32F446xx)
extern "C" void USART2_IRQHandler() {
#elif defined(STM32F469xx)
extern "C" void USART3_IRQHandler() {
#else
extern "C" void USART1_IRQHandler() {
#endif
    if (Serial::isr()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main()
{
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // Guarded: print() BLOCKS until the transport accepts each byte, so
    // printing into a UART that failed to come up would never return -
    // the LED heartbeat below is then the only sign of life.
    if (serial_ok) {
        const brio::DeviceIdcode id = brio::DeviceIdcode::read();
        brio::print(serial, brio::crlf, banner_head,
                    clock_ok ? "PLL" : "FAILED", ", tick=",
                    tick_ok ? "SysTick" : "FAILED",
                    ", dev=", brio::hex(id.dev_id), " rev=", brio::hex(id.rev_id),
                    "), type HELP", brio::crlf, "> ");
    }

    brio::Tenuto<P, Console, SerialLines, Blinker>::run();
}
