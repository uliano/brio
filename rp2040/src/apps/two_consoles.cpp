// two_consoles - two kernels on two cores, each with a console of its
// own: core 0's over UART0 (GP0 / GP1, the Debug Probe's bridge), core
// 1's over UART1 (GP4 / GP5, a second bridge). The same three active
// objects as the console app on core 0 - SerialPort, Console, Blinker -
// and two of them again on core 1, where the LED command CROSSES THE
// BRIDGE: core 1 has no LED of its own (there is one, and core 0's
// Blinker owns it), so its console sends the SetLed to core 0's Blinker
// through util/inbox.hpp, and core 0's doorbell drain posts it.
//
// Type on either console at 115200 8N1:
//   HELP | LED ON|OFF|TOG | UPTIME | ERR | CORE
// UPTIME is the answering core's own ticker; CORE says which core
// answers. The two kernels share nothing but the bridge: each has its
// SysTick, its queues, its rings, its breadcrumb.
//
// Everything above the target glue is portable; the glue is the clock
// type, the pins, the two transports, the launch of core 1 and the
// vector bindings (the one SysTick vector ticks the ticker of the core
// that took it; UART1's line is enabled by core 1's init, in ITS NVIC).
//
// Kernel pack order is a CONTRACT: a Console (line consumer) precedes
// its SerialPort (line producer) on both cores.
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
#include "rp2040/multicore.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/sysinfo.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/uart.hpp"
#include "util/inbox.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"

using P0 = brio::Rp2040Platform<0>;
using P1 = brio::Rp2040Platform<1>;

using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

namespace {

using Led = brio::Pin<25>;

constexpr brio::UartPins console0_pins{
    .tx = {0, brio::PinFunction::uart},
    .rx = {1, brio::PinFunction::uart},
};
constexpr brio::UartPins console1_pins{
    .tx = {4, brio::PinFunction::uart},
    .rx = {5, brio::PinFunction::uart},
};
using Serial0 = brio::Uart<0, console0_pins>;
using Serial1 = brio::Uart<1, console1_pins>;
constexpr Serial0 serial0;
constexpr Serial1 serial1;
constexpr uint32_t console_baud = 115200;

// ---- events -----------------------------------------------------------------
struct Toggle {};
struct SetLed { enum class Mode : uint8_t { on, off, tog } mode; };

// ---- the blinker: core 0's, the one LED ---------------------------------------
struct Blinker : brio::Fsm<Blinker, Toggle, SetLed> {
    static inline brio::EventQueue<Event, 4, P0> queue;
    static inline brio::TimeEvent<P0, Blinker, Toggle> heartbeat{Toggle{}};

    static void init() {
        Led::output();
        start(&beating);
    }

    static Status beating(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { heartbeat.arm_every(brio::ticks_from_ms<P0>(500)); return handled(); },
            [](brio::Exit) { heartbeat.disarm(); return handled(); },
            [](Toggle) { Led::toggle(); return handled(); },
            [](SetLed s) { apply(s); return transition(&manual); },
            [](auto) { return unhandled(); });
    }

    static Status manual(const Event& e) {
        return brio::match(e,
            [](SetLed s) { apply(s); return handled(); },
            [](auto) { return unhandled(); });
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

// ---- a console on a core: parses lines, replies, commands the blinker ---------
// The LED command is the one line that knows which core it runs on: a
// post on the Blinker's core, a send across the bridge from the other.
template <typename Serial, typename P, uint8_t core>
struct ConsoleOn : brio::Fsm<ConsoleOn<Serial, P, core>, brio::LineReceived> {
    using Base = brio::Fsm<ConsoleOn<Serial, P, core>, brio::LineReceived>;
    using Event = typename Base::Event;
    using Status = typename Base::Status;
    static inline brio::EventQueue<Event, 2, P> queue;
    using Lines = brio::SerialPort<Serial, P, ConsoleOn, 80>;

    using Parser = brio::ConsoleCommandParser<4>;
    using Router = brio::CommandRouter<Serial, 4>;
    using Cmd = typename Router::CommandType;

    static void init() { Base::start(&running); }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return Base::handled(); },
            [](brio::LineReceived l) { handle_line(l.line.get()); return Base::handled(); },
            [](auto) { return Base::unhandled(); });
    }

    static void banner() {
        const brio::ChipId id = brio::ChipId::read();
        brio::print(Serial{}, brio::crlf, "RP2040 rev B", id.revision, " brio console on core ", core,
                    " (clk=", SysClock::hz, " Hz), type HELP", brio::crlf, "> ");
    }

private:
    static void handle_line(char* line) {
        Cmd cmd;
        if (Parser::parse(line, cmd)) {
            if (!Router::dispatch(cmd, routes, route_count, Serial{})) {
                brio::print(Serial{}, "unknown command (try HELP)", brio::crlf);
            }
        }
        brio::print(Serial{}, "> ");
    }

    static void cmd_help(const Cmd&, Serial s) {
        brio::print(s, "commands: HELP | LED ON|OFF|TOG | UPTIME | ERR | CORE", brio::crlf);
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
        if constexpr (core == 0) {
            brio::post<Blinker>(SetLed{mode});
            brio::print(s, "OK", brio::crlf);
        } else {
            const uint16_t dropped = brio::Inbox<Blinker>::overflows();
            brio::send<Blinker>(SetLed{mode});
            brio::print(s, brio::Inbox<Blinker>::overflows() == dropped ? "OK (sent to core 0)" : "the bridge is full", brio::crlf);
        }
    }

    static void cmd_uptime(const Cmd&, Serial s) {
        brio::TimeStamp ts;
        brio::CoreTicker<core>::now(ts);
        brio::print(s, "uptime: ", ts, " (core ", core, "'s ticker)", brio::crlf);
    }

    static void cmd_core(const Cmd&, Serial s) {
        brio::print(s, "core ", P::core_id(), " answers (this kernel is core ", core, "'s)", brio::crlf);
    }

    static void cmd_err(const Cmd&, Serial s) {
        brio::print(s, "rx_overruns=", Serial::rx_overruns(), " frame=", Serial::frame_errors(),
                    " parity=", Serial::parity_errors(), " break=", Serial::break_errors(),
                    " hw_overruns=", Serial::hw_overruns(), " line_overflows=", Lines::line_overflows(),
                    " q_drops=", Lines::queue.overflows(), " misposts=", queue.misposts(),
                    " baud=", Serial::actual_baud(SysClock::pclk_hz), brio::crlf);
    }

    static constexpr typename Router::Route routes[] = {
        {"HELP", cmd_help}, {"LED", cmd_led}, {"UPTIME", cmd_uptime}, {"ERR", cmd_err}, {"CORE", cmd_core},
    };
    static constexpr uint8_t route_count = sizeof(routes) / sizeof(routes[0]);
};

using Console0 = ConsoleOn<Serial0, P0, 0>;
using Console1 = ConsoleOn<Serial1, P1, 1>;
using Lines0 = Console0::Lines;
using Lines1 = Console1::Lines;

using K0 = brio::Kernel<P0, Console0, Lines0, Blinker>;
using K1 = brio::Kernel<P1, Console1, Lines1>;
using Drain0 = brio::Inboxes<Blinker>;   // what core 1 sends to core 0

/// Core 1's whole life: its ticker, its UART (the line enabled in its
/// own NVIC), its kernel.
[[noreturn]] void core1_entry() {
    (void)brio::CoreTicker<1>::init(clock);
    const bool serial_ok = Serial1::init(clock, console_baud);
    brio::enable_interrupts();
    if (serial_ok) {
        Console1::banner();
    }
    K1::run();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() {
    if (Serial0::isr()) {
        brio::post<Lines0>(brio::RxActivity{});
    }
}
extern "C" void isr_uart1() {
    if (Serial1::isr()) {
        brio::post<Lines1>(brio::RxActivity{});
    }
}
extern "C" void isr_systick() {
    if (P0::core_id() == 0u) {
        brio::CoreTicker<0>::tick();
    } else {
        brio::CoreTicker<1>::tick();
    }
}
extern "C" void isr_sio_proc0() { Drain0::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial0::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    const bool launched = brio::Core1::launch(&core1_entry);
    Drain0::enable();
    brio::enable_interrupts();

    if (serial_ok) {
        brio::print(serial0, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED",
                    " core1=", launched ? "launched" : "FAILED", brio::crlf);
        Console0::banner();
    }
    K0::run();
}
