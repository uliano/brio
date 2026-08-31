// energy_logger - the DUT half of the energy experiment (see
// ../README.md for the whole design): an event logger living one of
// three power strategies while the SAM board meters every joule.
//
// RUN <strategy> [live_s] [iters] walks the measurement choreography -
// ack, console silence, park-enter signature, 3 s parked in
// POWER-DOWN (the meter's zero/pedestal span), park-leave, the LIVE
// PHASE below, park again, report - with the witness pin carrying one
// data toggle per PROCESSED BURST and the signatures of
// ../energy_link.hpp around the parks.
//
// The live phase watches the stimulus (SAM DAC -> PD3, threshold
// energy_link::detect_threshold_mv against the internal 2.048 V
// reference so the volts do not slide with the supply) and on each
// burst does `iters` rounds of CRC-16 over a 64-byte buffer - the same
// work, the same code, whatever the strategy:
//
//   0 sprint      24 MHz; STANDBY with the ADC free-running +
//                 RUNSTDBY and the window comparator as the only wake.
//                 Peripherals watch, the CPU pays nothing to wait.
//   1 pace        DynamicClock: quiet at 3 MHz awake (single
//                 conversions, IDLE between ticks), set(24M) on
//                 detection, work, back to 3 MHz. The clock-adaptation
//                 strategy under test.
//   2 static_low  6 MHz fixed; the same standby watch as sprint; work
//                 at 6 MHz. The "choose the static clock well" baseline
//                 - the real SAM-side alternative.
//   3 rehearsal   no detector: one data toggle per 500 ms (the
//                 witness/choreography test load).
//
// PARKING NOTE, stated not hidden: the PIT keeps ticking at 1024 Hz
// through power-down, so parked current is the PD floor plus that
// wake duty (and the board's ADuM). The sleep characterization phase
// will park with the tick slowed.
//
// Wiring (see ../README.md): PD2 -> meter PA04 (witness), meter PA02
// -> PD3 (stimulus), console on USART2 ALT1 PF4/PF5 through the
// board's ADuM, common GND.
//
// build: boards = db48
// build: monitor_speed = 115200

#include <avr/interrupt.h>
#include <stdint.h>
#include <variant>

#include "../energy_link.hpp"
#include "avrdx/adc.hpp"
#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/sleep.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/vref.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/crc.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"

using P = brio::AvrPlatform;

// OSCHF only, never the crystal: it must never draw, timing precision
// is the METER's job, and internal oscillators keep the DUT identical
// across the 3.0..4.2 V window. DynamicClock because strategy `pace`
// IS the dynamic clock; the others just never call set().
using Boot = brio::Clock<brio::ClockSource::internal, 24'000'000>;
using Serial = brio::Uart<2, brio::Route::alt1>;  // PF4/PF5, the ADuM pair
using Adc0 = brio::Adc<0>;
using SysClock = brio::DynamicClock<Boot, Serial, Adc0>;
constexpr SysClock clock;
constexpr Serial serial;
constexpr uint32_t console_baud = 115200;

namespace {

using Witness = brio::Pin<'D', 2>;   // -> meter PA04
using StimIn = brio::Pin<'D', 3>;    // <- meter PA02 (DAC), ADC0/AIN3

// Detection thresholds in ADC counts: the contract's millivolts on the
// 2.048 V internal reference at 12 bits (1 count = 0.5 mV).
constexpr uint16_t detect_code = static_cast<uint16_t>(
    static_cast<uint32_t>(energy::detect_threshold_mv) * 4096u / 2048u);
constexpr uint16_t release_code = static_cast<uint16_t>(
    static_cast<uint32_t>(energy::stimulus_quiet_mv + 150u) * 4096u / 2048u);

volatile bool wcmp_hit = false;

/// Every input buffer off (a floating CMOS input burns shoot-through,
/// measured 0.49 mA on this board) except the console pair PF4/PF5.
void quiet_pins() {
    constexpr brio::PinConfig quiet{.sense = brio::PinSense::input_disable};
    brio::Port<'A'>::configure_mask(0xFF, quiet);
    brio::Port<'B'>::configure_mask(0x3F, quiet);
    brio::Port<'C'>::configure_mask(0xFF, quiet);
    brio::Port<'D'>::configure_mask(0xFF, quiet);
    brio::Port<'E'>::configure_mask(0x0F, quiet);
    brio::Port<'F'>::configure_mask(0x0F, quiet);  // PF4/PF5 = console
}

/// One signature: n toggle PAIRS - 2n edges, resting level preserved.
void signature(uint8_t n) {
    for (uint8_t i = 0; i < n; ++i) {
        Witness::toggle();
        brio::delay_us(clock, energy::signature_half_us);
        Witness::toggle();
        brio::delay_us(clock, energy::signature_half_us);
    }
}

/// Park in power-down until `ms` have passed (the PIT wakes per tick).
void park_ms(uint16_t ms) {
    const uint32_t t0 = brio::Ticker::millis();
    while (brio::Ticker::millis() - t0 < ms) {
        brio::Sleep::enter(brio::SleepMode::power_down);
    }
}

// ---- the burst work ---------------------------------------------------------

uint8_t work_buf[64];
volatile uint16_t work_sink;   // keeps the CRC loop un-deletable

void work(uint16_t iters) {
    uint16_t acc = 0xFFFFu;
    for (uint16_t i = 0; i < iters; ++i) {
        acc ^= brio::crc16(work_buf, sizeof(work_buf));
    }
    work_sink = acc;
}

// ---- the detector -----------------------------------------------------------

/// Configure the stimulus watcher. `standby_watch` = the sprint /
/// static_low shape (free-running + RUNSTDBY + window comparator as
/// the wake); pace polls single conversions instead.
bool detector_init(bool standby_watch) {
    brio::AdcConfig c{};
    c.reference = brio::Ref::v2048;
    c.free_running = standby_watch;
    c.run_standby = standby_watch;
    if (!Adc0::init(clock, c)) {
        return false;
    }
    Adc0::select(brio::AnalogIn<StimIn>{});
    if (standby_watch) {
        Adc0::window(Adc0::Window::above, 0, detect_code);
        Adc0::start();   // the free run begins here
    }
    return true;
}

/// One single conversion (pace's poll and everyone's release check).
uint16_t adc_single() {
    Adc0::start();
    while (Adc0::busy()) {
    }
    return Adc0::result();
}

// ---- the live phase ---------------------------------------------------------

struct LiveResult {
    uint16_t bursts = 0;
    bool detector_ok = true;
};

LiveResult live_strategy(uint8_t strat, uint32_t ms, uint16_t iters) {
    LiveResult r{};
    const bool watch = (strat == 0u || strat == 2u);

    if (strat == 1u) { (void)SysClock::set(3'000'000); }
    if (strat == 2u) { (void)SysClock::set(6'000'000); }
    if (strat == 4u) {
        // sprint_duty: the frugal watch the first map point demanded -
        // STANDBY between PIT ticks, ONE single conversion per wake
        // (software threshold), 24 MHz for the work. The ADC never
        // free-runs, so the clock chain sleeps between samples; the
        // watch costs ~1-2% duty instead of a converter at full tilt.
        if (!detector_init(false)) {
            r.detector_ok = false;
            return r;
        }
        const uint32_t t0 = brio::Ticker::millis();
        while (brio::Ticker::millis() - t0 < ms) {
            brio::Sleep::enter(brio::SleepMode::standby);
            if (adc_single() > detect_code) {
                work(iters);
                Witness::toggle();
                ++r.bursts;
                while (brio::Ticker::millis() - t0 < ms &&
                       adc_single() > release_code) {
                    brio::Sleep::enter(brio::SleepMode::standby);
                }
            }
        }
        return r;
    }
    if (strat == 3u) {
        // Rehearsal load: a data toggle per 500 ms, IDLE between.
        const uint32_t t0 = brio::Ticker::millis();
        uint32_t next = 500;
        while (brio::Ticker::millis() - t0 < ms) {
            if (brio::Ticker::millis() - t0 >= next) {
                Witness::toggle();
                ++r.bursts;
                next += 500;
            }
            brio::Sleep::enter(brio::SleepMode::idle);
        }
        return r;
    }

    if (!detector_init(watch)) {
        r.detector_ok = false;
        (void)SysClock::set(24'000'000);
        return r;
    }

    const uint32_t t0 = brio::Ticker::millis();
    while (brio::Ticker::millis() - t0 < ms) {
        // -- wait for a burst --
        bool detected = false;
        if (watch) {
            wcmp_hit = false;
            Adc0::clear_window_flag();
            Adc0::enable_wcmp_interrupt(true);
            while (!wcmp_hit && brio::Ticker::millis() - t0 < ms) {
                cli();
                if (!wcmp_hit) {
                    brio::Sleep::arm(brio::SleepMode::standby);
                    sei();
                    brio::Sleep::sleep();
                    brio::Sleep::disarm();
                } else {
                    sei();
                }
            }
            sei();
            Adc0::enable_wcmp_interrupt(false);
            detected = wcmp_hit;
        } else {
            while (brio::Ticker::millis() - t0 < ms) {
                if (adc_single() > detect_code) {
                    detected = true;
                    break;
                }
                brio::Sleep::enter(brio::SleepMode::idle);
            }
        }
        if (!detected) {
            break;   // the live span ended while waiting
        }

        // -- the burst: work at full (or the strategy's) speed --
        if (strat == 1u) { (void)SysClock::set(24'000'000); }
        work(iters);
        Witness::toggle();
        ++r.bursts;
        if (strat == 1u) { (void)SysClock::set(3'000'000); }

        // -- wait for the stimulus to drop before re-arming --
        while (brio::Ticker::millis() - t0 < ms) {
            const uint16_t level = watch ? Adc0::result() : adc_single();
            if (level < release_code) {
                break;
            }
            brio::Sleep::enter(brio::SleepMode::idle);
        }
    }

    Adc0::window_off();
    (void)SysClock::set(24'000'000);
    return r;
}

// ---- the console ------------------------------------------------------------
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

    static uint32_t arg_n(const Cmd& cmd, uint8_t idx, uint32_t fallback) {
        if (cmd.argument_count <= idx) {
            return fallback;
        }
        uint32_t v = 0;
        const char* p = cmd.arguments[idx];
        if (*p == '\0') {
            return fallback;
        }
        while (*p >= '0' && *p <= '9') {
            v = v * 10u + static_cast<uint32_t>(*p - '0');
            ++p;
        }
        return (*p == '\0') ? v : fallback;
    }

    static void cmd_help(const Cmd&, Serial s) {
        brio::print(s, "energy_logger:", brio::crlf,
            "  RUN <strat> [live_s] [iters]   the measurement choreography",
            brio::crlf,
            "    strat: 0 sprint | 1 pace | 2 static_low | 3 rehearsal",
            brio::crlf,
            "           4 sprint_duty (standby, 1 conversion per tick)",
            brio::crlf,
            "    park 3s / live / park 3s; one witness toggle per burst",
            brio::crlf);
    }

    static void cmd_run(const Cmd& cmd, Serial s) {
        const uint8_t strat = static_cast<uint8_t>(arg_n(cmd, 0, 3));
        const uint32_t live_s = arg_n(cmd, 1, 10);
        const uint16_t iters = static_cast<uint16_t>(arg_n(cmd, 2, 300));
        if (strat > 4u) {
            brio::print(s, "RUN: strat 0..4", brio::crlf);
            return;
        }
        brio::print(s, "RUN strat=", strat, " live=", live_s, " s iters=",
                    iters, " (silent until done)", brio::crlf);
        brio::delay_us(clock, 20'000);   // let the TX ring drain

        const uint32_t t0 = brio::Ticker::millis();
        signature(energy::sig_park_enter);
        park_ms(3000);
        signature(energy::sig_park_leave);
        const LiveResult res = live_strategy(strat, live_s * 1000u, iters);
        signature(energy::sig_park_enter);
        park_ms(3000);
        signature(energy::sig_park_leave);
        const uint32_t t_ms = brio::Ticker::millis() - t0;

        brio::print(s, "RUN done: t=", t_ms, " ms, bursts=", res.bursts,
                    res.detector_ok ? "" : ", DETECTOR REFUSED",
                    brio::crlf);
    }

    static constexpr Router::Route routes[] = {
        {"HELP", cmd_help},
        {"RUN", cmd_run},
    };
    static constexpr uint8_t route_count =
        sizeof(routes) / sizeof(routes[0]);
};

using SerialLines = brio::SerialPort<Serial, P, Console, 48>;

}  // namespace

ISR(RTC_PIT_vect) { brio::Ticker::pit(); }
ISR(USART2_RXC_vect) {
    if (Serial::rxc()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(ADC0_WCMP_vect) {
    (void)Adc0::wcmp();   // reading RES clears the flag, verdict captured
    wcmp_hit = true;
}

int main() {
    quiet_pins();
    (void)SysClock::init();
    Serial::init(clock, console_baud);
    brio::Ticker::init();
    Witness::output();
    for (uint8_t i = 0; i < sizeof(work_buf); ++i) {
        work_buf[i] = static_cast<uint8_t>(i * 37u + 11u);
    }
    sei();

    brio::print(serial, brio::crlf,
                "energy_logger (24 MHz OSCHF, strategies), type HELP",
                brio::crlf, "> ");

    brio::Kernel<P, Console, SerialLines>::run();
}
