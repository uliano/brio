// sampler - the ADC inside the kernel: brio::AnalogSampler owning ADC0,
// walking three inputs (the DAC loop on PD1, the die temperature, VDD/10)
// and publishing every result; two subscribers (a Monitor that prints,
// an Alarm that drives the LED on a threshold); a console to move the
// source, change the pace and the clock.
//
// Wiring: PD6 (DAC0 OUT) -> PD1 (AIN1), the jumper of test_avr_analog.
// Console @ 115200 on USART2 ALT1 (PF4/PF5). LED PF2.
//
// pio: monitor_speed = 115200
//
// Commands:
//   DAC <mv>              source voltage on PD6 (0..2047 mV, 2.048 V ref)
//   PACE HW [64|128|256|512]   PIT divider -> EVSYS channel 1 -> ADC start
//                         (32768/div Hz; no CPU between samples). Default 256.
//   PACE SW <ms>          software pace: a TimeEvent of the sampler
//   PACE OFF
//   ALARM <mv>            LED on while PD1 > mv (ALARM 0 = off)
//   CLOCK [<hz>|<n>M|<n>k]   as clock_console: the owner's duty on a clock
//                         change (hardware pace paused across the switch)
//   STAT                  sample rate, queue drops, unknown codes, serial errors
//   HELP
//
// What it shows on the bench: the result path ISR -> post -> AO at the
// hardware pace, attribution by the reported input code, publish to
// two subscribers by value, the queue as the ammortizer (STAT), the
// same millivolts test_avr_analog measures, the pace source being the
// app's choice (PIT here, any generator tomorrow), and a clock change
// under sampling.

#include <avr/interrupt.h>
#include <stdint.h>
#include <variant>

#include "avrdx/adc.hpp"
#include "avrdx/clock.hpp"
#include "avrdx/dac.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/uart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/analog.hpp"
#include "util/analog_sampler.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"

using P = brio::AvrPlatform;
using Boot = brio::Clock<brio::ClockSource::crystal, 24'000'000>;

namespace {

using Led = brio::Pin<'F', 2>;
using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;
using Adc = brio::Adc<0>;
using Dac = brio::Dac<0>;

using SysClock = brio::DynamicClock<Boot, Serial, Adc>;   // both rebased on set()
constexpr SysClock clock;
constexpr uint32_t baud = 115200;

constexpr brio::Ref ref = brio::Ref::v2048;
constexpr uint16_t ref_mv = brio::ref_mv(ref);

// The converter's configuration is the owner's (main): 2.048 V, 375 kHz
// CLK_ADC with the init delay and sample length the temperature sensor
// needs (>= 25 us, >= 28 us), no accumulation so the temperature formula
// applies to the raw result.
constexpr brio::AdcConfig adc_cfg{.reference = ref, .prescaler = brio::AdcPresc::div64,
                                  .sample_length = 12,
                                  .init_delay = brio::AdcInitDelay::cycles64};

// ---- the inputs, by position ------------------------------------------------
using Loop = brio::AnalogIn<brio::Pin<'D', 1>>;   // index 0: the DAC loop
constexpr uint8_t idx_loop = 0, idx_temp = 1, idx_vdd = 2;

struct Monitor;
struct Alarm;
using Sampler = brio::AnalogSampler<Adc, P, brio::Subscribers<Monitor, Alarm>,
                                    Loop{}, brio::AdcInput::temp, brio::AdcInput::vdd_div10>;

// ---- the hardware pace: PIT divider -> channel 1 -> ADC start ---------------
using PaceChannel = brio::EventChannel<1>;

struct Pace {
    static inline uint16_t div = 0;   // 0 = hardware pace off

    static void hw(uint16_t d) {
        switch (d) {
            case 64:  PaceChannel::source(brio::EvPitDiv<64>{}); break;
            case 128: PaceChannel::source(brio::EvPitDiv<128>{}); break;
            case 256: PaceChannel::source(brio::EvPitDiv<256>{}); break;
            default:  PaceChannel::source(brio::EvPitDiv<512>{}); d = 512; break;
        }
        div = d;
        Adc::start_on(PaceChannel{});
    }
    static void off() {
        Adc::start_on_events(false);
        PaceChannel::off();
        div = 0;
    }
    // The owner's duty around a clock change: no event may start a
    // conversion while CLK_ADC is being re-derived.
    static void suspend() { if (div) Adc::start_on_events(false); }
    static void resume() { if (div) Adc::start_on_events(true); }
};

// ---- Monitor: last value per input, one line a second -----------------------
struct Report {};

struct Monitor : brio::Fsm<Monitor, brio::AnalogSample, Report> {
    static inline brio::EventQueue<Event, 8, P> queue;
    static inline brio::TimeEvent<P, Monitor, Report> report{Report{}};
    static inline uint16_t last[3]{};
    static inline uint16_t count = 0;        // samples since the last report
    static inline uint16_t rate = 0;         // samples/s at the last report

    static void init() { start(&running); }
    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { report.arm_every(brio::ticks_from_ms<P>(1000)); return handled(); },
            [](brio::AnalogSample s) {
                last[s.index] = s.value;
                ++count;
                return handled();
            },
            [](Report) {
                rate = count;
                count = 0;
                if (verbose) line();
                return handled();
            },
            [](auto) { return unhandled(); });
    }
    static inline bool verbose = true;

    static void line() {
        const uint16_t k = Adc::temp_kelvin(last[idx_temp]);
        brio::print(serial, "PD1 ", brio::adc_mv(last[idx_loop], Adc::steps(), ref_mv), " mV | die ",
                    static_cast<int16_t>(k - 273), " C | VDD ",
                    static_cast<uint16_t>(brio::adc_mv(last[idx_vdd], Adc::steps(), ref_mv) * 10u),
                    " mV | ", rate, " samples/s", brio::crlf);
    }
};

// ---- Alarm: the LED follows a threshold on the loop input -------------------
struct SetAlarm { uint16_t mv; };

struct Alarm : brio::Fsm<Alarm, brio::AnalogSample, SetAlarm> {
    static inline brio::EventQueue<Event, 8, P> queue;
    static inline uint16_t threshold_mv = 0;

    static void init() { Led::output(); start(&running); }
    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::AnalogSample s) {
                if (s.index != idx_loop) return handled();
                const bool over = threshold_mv != 0 &&
                                  brio::adc_mv(s.value, Adc::steps(), ref_mv) > threshold_mv;
                if (over) Led::set(); else Led::clear();
                return handled();
            },
            [](SetAlarm a) { threshold_mv = a.mv; if (a.mv == 0) Led::clear(); return handled(); },
            [](auto) { return unhandled(); });
    }
};

// ---- the console ------------------------------------------------------------
struct Console : brio::Fsm<Console, brio::LineReceived> {
    static inline brio::EventQueue<Event, 2, P> queue;

    using Parser = brio::ConsoleCommandParser<4>;
    using Router = brio::CommandRouter<Serial, 4>;
    using Cmd = Router::CommandType;

    static void init() { start(&running); }
    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::LineReceived l) { handle_line(l.line.get()); return handled(); },
            [](auto) { return unhandled(); });
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

    static bool parse_u32(const char* text, uint32_t& v) {
        if (*text < '0' || *text > '9') return false;
        v = 0;
        while (*text >= '0' && *text <= '9') v = v * 10u + static_cast<uint32_t>(*text++ - '0');
        if (*text == 'M' || *text == 'm') { v *= 1'000'000u; ++text; }
        else if (*text == 'K' || *text == 'k') { v *= 1'000u; ++text; }
        return *text == '\0';
    }

    static void cmd_help(const Cmd&, Serial s) {
        brio::print(s, "DAC <mv> | PACE HW [64|128|256|512] | PACE SW <ms> | PACE OFF | "
                       "ALARM <mv> | CLOCK [<hz>|<n>M|<n>k] | STAT | QUIET | HELP", brio::crlf);
    }

    static void cmd_dac(const Cmd& cmd, Serial s) {
        uint32_t mv;
        if (cmd.argument_count != 1 || !parse_u32(cmd.arguments[0], mv) || mv >= ref_mv) {
            brio::print(s, "usage: DAC <mv>  (0..", ref_mv - 1, ")", brio::crlf);
            return;
        }
        Dac::set_mv(static_cast<uint16_t>(mv), ref_mv);
        brio::print(s, "DAC ", mv, " mV", brio::crlf);
    }

    static void cmd_pace(const Cmd& cmd, Serial s) {
        const char* mode = cmd.argument_count >= 1 ? cmd.arguments[0] : "";
        if (brio::command_equals(mode, "OFF")) {
            Pace::off();
            Sampler::stop();
            brio::print(s, "pace off", brio::crlf);
        } else if (brio::command_equals(mode, "HW")) {
            uint32_t d = 256;
            if (cmd.argument_count == 2 && !parse_u32(cmd.arguments[1], d)) d = 256;
            Sampler::stop();
            Pace::hw(static_cast<uint16_t>(d));
            brio::print(s, "hardware pace: PIT/", Pace::div, " = ", 32768u / Pace::div,
                        " samples/s", brio::crlf);
        } else if (brio::command_equals(mode, "SW")) {
            uint32_t ms;
            if (cmd.argument_count != 2 || !parse_u32(cmd.arguments[1], ms) || ms == 0) {
                brio::print(s, "usage: PACE SW <ms>", brio::crlf);
                return;
            }
            Pace::off();
            Sampler::start_every(brio::ticks_from_ms<P>(ms));
            brio::print(s, "software pace: every ", ms, " ms", brio::crlf);
        } else {
            brio::print(s, "usage: PACE HW [64|128|256|512] | PACE SW <ms> | PACE OFF", brio::crlf);
        }
    }

    static void cmd_alarm(const Cmd& cmd, Serial s) {
        uint32_t mv;
        if (cmd.argument_count != 1 || !parse_u32(cmd.arguments[0], mv)) {
            brio::print(s, "usage: ALARM <mv>  (0 = off)", brio::crlf);
            return;
        }
        brio::post<Alarm>(SetAlarm{static_cast<uint16_t>(mv)});
        if (mv) brio::print(s, "alarm above ", mv, " mV", brio::crlf);
        else brio::print(s, "alarm off", brio::crlf);
    }

    static void cmd_clock(const Cmd& cmd, Serial s) {
        if (cmd.argument_count == 0) {
            brio::print(s, "CLK_PER = ", SysClock::hz(), " Hz, CLK_ADC = ", Adc::clock_hz_adc(),
                        " Hz", brio::crlf);
            return;
        }
        uint32_t next;
        if (!parse_u32(cmd.arguments[0], next) || !SysClock::can_run_at(next) ||
            !Serial::can_baud(next, baud)) {
            brio::print(s, "refused (unreachable rate, or < ", Serial::min_hz_for(baud),
                        " Hz for the console)", brio::crlf);
            return;
        }
        brio::print(s, "switching to ", next, " Hz", brio::crlf);
        Pace::suspend();             // the owner's duty: no event-started conversion mid-switch
        SysClock::set(next);         // Serial then Adc rebased, then the prescaler
        Pace::resume();
        brio::print(s, "now at ", SysClock::hz(), " Hz, CLK_ADC ", Adc::clock_hz_adc(), " Hz",
                    brio::crlf);
    }

    static void cmd_stat(const Cmd&, Serial s);
    static void cmd_quiet(const Cmd&, Serial s) {
        Monitor::verbose = !Monitor::verbose;
        brio::print(s, "report ", Monitor::verbose ? "on" : "off", brio::crlf);
    }

    static constexpr Router::Route routes[] = {
        {"HELP", cmd_help}, {"DAC", cmd_dac}, {"PACE", cmd_pace}, {"ALARM", cmd_alarm},
        {"CLOCK", cmd_clock}, {"STAT", cmd_stat}, {"QUIET", cmd_quiet},
    };
    static constexpr uint8_t route_count = sizeof(routes) / sizeof(routes[0]);
};

using SerialLines = brio::SerialPort<Serial, P, Console, 80>;

void Console::cmd_stat(const Cmd&, Serial s) {
    brio::print(s, Monitor::rate, " samples/s | sampler q_drops=", Sampler::queue.overflows(),
                " unknown=", Sampler::unknown_inputs(),
                " | monitor q_drops=", Monitor::queue.overflows(),
                " alarm q_drops=", Alarm::queue.overflows(),
                " | serial rx_overruns=", Serial::rx_overruns(),
                " line_overflows=", SerialLines::line_overflows(), brio::crlf);
    Monitor::line();
}

} // namespace

// ---- target glue ------------------------------------------------------------
ISR(ADC0_RESRDY_vect) { brio::post<Sampler>(brio::Sampled{Adc::resrdy(), Adc::selected()}); }
ISR(USART2_RXC_vect) {
    if (Serial::rxc()) brio::post<SerialLines>(brio::RxActivity{});
}
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { brio::Ticker::pit(); }

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, baud);
    brio::Ticker::init();

    Dac::init({.reference = ref});                 // buffered out on PD6
    Dac::set_mv(1000, ref_mv);
    Adc::init<adc_cfg>(clock);                     // the owner configures; the sampler walks
    Adc::enable_resrdy_interrupt(true);
    Pace::hw(256);                                 // 128 samples/s, no CPU between them
    sei();

    brio::print(serial, brio::crlf, "AVR128DB48 brio sampler (src=", xtal ? "XTAL" : "OSCHF",
                ", ", SysClock::hz(), " Hz, CLK_ADC ", Adc::clock_hz_adc(), " Hz), DAC 1000 mV, "
                "PIT/256 pace, type HELP", brio::crlf, "> ");

    // Consumers before producers: Console before SerialLines (line loans);
    // the sampler publishes by value, its position is free.
    brio::Kernel<P, Console, SerialLines, Monitor, Alarm, Sampler>::run();
}
