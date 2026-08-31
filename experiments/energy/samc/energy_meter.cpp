// energy_meter - the instrument half of the energy experiment (see
// ../README.md for the whole design). On the bench this board is three
// things at once: the WORLD (DAC stimulus from a seeded schedule), the
// JUDGE (the witness pin on an AC comparator, counted and timestamped)
// and the METER (SDADC on the shunt in psu mode, SAR on the capacitor
// in cap mode).
//
// TODAY'S SURFACE: the bring-up letters, console-driven so run.py and a
// human speak the same grammar.
//
//   IMEAS [n]   n single conversions on SDADC pair 0 (shunt sense,
//               INN0 = PA06 = DUT side, INP0 = PA07 = supply side,
//               INTREF 1.024 V + reference buffer, chopper on, OSR 256):
//               mean/min/max/rms. Serves, in this order, the offset
//               letter (sense leads shorted together), the pedestal
//               letter (DUT parked in power-down: offset + the ADuM's
//               quiescent + 1 uA - the difference from the offset letter
//               IS the ADuM, measured), and the known-load calibration
//               (fixed resistors in the DUT's place calibrate R_shunt in
//               place).
//   WIT [s]     the witness-comparator letter: COMP0 on PA04 against the
//               internal 1.024 V bandgap, hysteresis on, counting edges
//               by interrupt for s seconds. Against energy_logger's 1 Hz
//               witness toggle it must count 2 edges per second at every
//               DUT supply in the 3.0..4.2 V window - the C21's digital
//               VIH (0.7 x VDD) is why this is a comparator and not an
//               EIC line (../README.md, "the judge").
//
// The stimulus (DAC + seeded schedule), the timestamped judge and the
// windowed energy integral land next.
//
// Wiring (see ../README.md): shunt sense -> PA06/PA07 twisted; witness
// -> PA04; common GND with the DUT board. Console = the board's own
// CH340 (PB30/PB31).
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "../energy_link.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "samc/ac.hpp"
#include "samc/adc.hpp"
#include "samc/clock.hpp"
#include "samc/dac.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/sdadc.hpp"
#include "samc/sercom.hpp"
#include "samc/supc.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"
#include "util/proto/line_parser.hpp"
#include "util/serial_port.hpp"
#include "util/timestamp.hpp"

using P = brio::SamPlatform;

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using Led = brio::Pin<'B', 23>;  // the board's LED: alive-light

// The shunt sense pair = SDADC pair 0. The pads are NOT claimed from
// PORT: an analog input reaches the pad with no mux, and the reset
// state (high-Z input) is exactly right for a sense lead.
//   PA06 = AINN0 = the DUT side (downstream), PA07 = AINP0 = the supply
//   side (upstream) -> positive readings for current INTO the DUT.
// The witness lands on PA04 = the AC's AIN0 (COMP0), same no-mux rule.

constexpr brio::UartPads console_pads{
    .tx = brio::SercomPad::pad0,
    .rx = brio::SercomPad::pad1,
    .tx_pin = {'B', 30, brio::PinFunction::d},
    .rx_pin = {'B', 31, brio::PinFunction::d},
};
static_assert(MUX_PB30D_SERCOM5_PAD0 ==
              static_cast<uint8_t>(console_pads.tx_pin.function));
static_assert(MUX_PB31D_SERCOM5_PAD1 ==
              static_cast<uint8_t>(console_pads.rx_pin.function));

using Serial = brio::Uart<5, console_pads>;
constexpr Serial serial;
constexpr uint32_t console_baud = 115200;

using Sdadc = brio::Sdadc;
using WitnessComp = brio::AcComparator<0>;
using VAdc = brio::Adc<0>;
using VSense = brio::AnalogIn<brio::Pin<'B', 9>>;  // PB09 = ADC0/AIN3

constexpr uint8_t main_gen = 0;
constexpr uint32_t main_gen_hz = SysClock::hz;

bool sdadc_ok = false;   // set at boot; IMEAS refuses without it
bool ac_ok = false;      // set at boot; WIT refuses without it
bool dac_up = false;     // set at boot; STIM refuses without it
// What the DAC is currently asked to play - node_mv() parks it at zero
// while SUPC.SEL visits 4.096 V (the DAC rides the SAME bandgap, so a
// live level would glitch 4x and fake a burst - bench-caught) and puts
// this value back after.
uint16_t dac_level = 0;

bool dac_play(uint16_t code) {
    dac_level = code;
    return brio::Dac::set(code);
}

// Edges counted by AC_Handler while a WIT letter runs. Volatile: the
// ISR writes, the command loop polls (the ticker doctrine - gcc -Os
// deletes a bare polling loop otherwise).
volatile uint32_t wit_edges = 0;

// ---- the judge: witness edge groups -----------------------------------------
//
// The decoder energy_link.hpp's timing is sized for: edges closer than
// 2 ms belong to one GROUP; a group of 2N edges is a signature of N
// pulses (park-enter = 6 edges, park-leave = 10), a group of ONE edge
// is a data toggle. AC_Handler does the grouping with millis
// timestamps; JLOG drains the ring. JARM arms the comparator
// permanently (WIT's one-shot letter still works, but disarms the
// judge when it ends - use one or the other).
struct WitGroup {
    uint32_t t_ms;
    uint16_t edges;
};
constexpr uint8_t wit_ring_size = 64;
volatile WitGroup wit_ring[wit_ring_size];
volatile uint8_t wit_ring_n = 0;      // saturates; JLOG clears
volatile uint32_t wit_last_ms = 0;
volatile uint32_t wit_group_t0 = 0;
volatile uint16_t wit_group_edges = 0;
bool judge_armed = false;

/// Close the in-flight group into the ring (ISR or guarded context).
void wit_close_group() {
    if (wit_group_edges == 0u) {
        return;
    }
    if (wit_ring_n < wit_ring_size) {
        wit_ring[wit_ring_n].t_ms = wit_group_t0;
        wit_ring[wit_ring_n].edges = wit_group_edges;
        wit_ring_n = wit_ring_n + 1u;
    }
    wit_group_edges = 0;
}

// ---- the window engine ------------------------------------------------------
//
// A measurement window is the SDADC free-running at OSR 256 (6 MHz /
// 4 / 256 = 5859.375 conversions per second, NO dead time - the SINC
// integrates every modulator sample, which is the whole granularity
// argument of ../README.md) with SDADC_Handler accumulating the raw
// 24-bit results. V is taken by VMEAS at open and close (the SUPC SEL
// switch happens only while the SDADC is idle); the offset subtracted
// is the stored zero (the ZERO letter measures it live with the DUT
// parked; the compiled default is the bring-up value at the 3.3 V
// common mode, +4.39 mV).
volatile int64_t win_sum = 0;
volatile uint32_t win_count = 0;
volatile int32_t win_max_raw = INT32_MIN;
volatile uint32_t win_overruns = 0;
bool win_open = false;
uint32_t win_open_ms = 0;
uint16_t win_v_open_mv = 0;
int32_t zero_raw = 35926;   // bring-up offset; ZERO overrides

/// Free-running conversion period: 1024 CLK_SDADC cycles at 6 MHz.
/// t_us = n x 512 / 3, exactly.
constexpr uint32_t win_rate_num = 3;     // us per sample = 512/3
constexpr uint32_t win_rate_den = 512;

constexpr int32_t r_shunt_milliohm = 10180;  // calibrated in place, +-1%

/// The shunt-sense configuration: INTREF (1.024 V via SUPC.VREF.SEL)
/// with the reference buffer 39.8.2 requires for it, chopper on (table
/// 45-27's own condition), 6 MHz CLK_SDADC, OSR 256, single conversions
/// with the silicon's own two warm-up windows.
brio::SdadcConfig sense_cfg() {
    brio::SdadcConfig c{};
    c.reference = brio::SdadcRef::intref;
    c.reference_buffer = true;
    c.prescaler = 3;                    // 48 MHz / (2 x 4) = 6 MHz
    c.osr = brio::SdadcOsr::osr256;
    c.chopper = true;
    return c;
}

/// Integer square root (for the rms line; 32-bit input is plenty here).
uint32_t isqrt(uint32_t v) {
    uint32_t r = 0;
    uint32_t bit = 1uL << 30;
    while (bit > v) { bit >>= 2; }
    while (bit != 0u) {
        if (v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

/// N single conversions on the selected pair, statistics in the RAW
/// 24-bit datapath units (1 specified count = 256 raw). The two
/// discarded conversions are this app's warm-up on top of SKPCNT.
struct SenseStats {
    int32_t raw_mean = 0;
    int32_t raw_low = 0;
    int32_t raw_high = 0;
    uint32_t raw_rms = 0;
    uint16_t taken = 0;
};

SenseStats sense_stats(uint16_t count) {
    SenseStats s{};
    int32_t buf[64];
    const uint16_t n = count > 64u ? 64u : (count == 0u ? 1u : count);
    Sdadc::discard(2);
    for (uint16_t i = 0; i < n; ++i) {
        Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
        if (!Sdadc::start()) {
            continue;
        }
        uint32_t spins = 0xFFFFFFu;
        while (spins-- != 0u && !Sdadc::ready()) {
        }
        if (!Sdadc::ready()) {
            continue;
        }
        buf[s.taken] = Sdadc::result24();
        ++s.taken;
    }
    if (s.taken == 0u) {
        return s;
    }
    int64_t sum = 0;
    s.raw_low = buf[0];
    s.raw_high = buf[0];
    for (uint16_t i = 0; i < s.taken; ++i) {
        sum += buf[i];
        if (buf[i] < s.raw_low) { s.raw_low = buf[i]; }
        if (buf[i] > s.raw_high) { s.raw_high = buf[i]; }
    }
    s.raw_mean = static_cast<int32_t>(sum / s.taken);
    uint64_t var = 0;
    for (uint16_t i = 0; i < s.taken; ++i) {
        const int64_t d = buf[i] - s.raw_mean;
        var += static_cast<uint64_t>(d * d);
    }
    var /= s.taken;
    s.raw_rms = isqrt(var > 0xFFFFFFFFuLL ? 0xFFFFFFFFu
                                          : static_cast<uint32_t>(var));
    return s;
}

/// Raw 24-bit datapath units -> nanovolts across the sense pair:
/// one SPECIFIED count (256 raw) = VREF/32768 = 31.25 uV at 1.024 V,
/// so one raw unit = 31250/256 nV and nV = raw x 15625 / 128.
int32_t raw_to_nv(int32_t raw) {
    return static_cast<int32_t>((static_cast<int64_t>(raw) * 15625) / 128);
}

/// The SAR's node-voltage configuration: INTREF as REFERENCE (needs no
/// VREFOE - the measured fact), whose LEVEL is SUPC.VREF.SEL. The
/// letter runs it at 4.096 V, where a 12-bit count is EXACTLY one
/// millivolt, and restores 1.024 V (the SDADC's and the AC's level)
/// before returning.
brio::AdcConfig vsense_cfg() {
    brio::AdcConfig c{};
    c.reference = brio::Ref::intref;
    return c;
}

/// The node voltage on PB09, mean of n SAR conversions in mV. Switches
/// SUPC.VREF.SEL to 4.096 V for its duration and restores 1.024 V -
/// legal only while the SDADC is IDLE, which is why the window letters
/// call it strictly outside the free-running span.
bool node_mv(uint16_t& mv_out, uint16_t n) {
    if (dac_up) {
        (void)brio::Dac::set(0);   // no rising edge: a dip cannot detect
    }
    (void)brio::Vref::configure(brio::VrefConfig{.level =
        brio::VrefLevel::v4_096});
    const bool up = VAdc::init(main_gen, vsense_cfg(), main_gen_hz);
    uint32_t sum = 0;
    uint16_t taken = 0;
    if (up) {
        VAdc::select(VSense{});
        for (uint16_t i = 0; i < n; ++i) {
            uint16_t v = 0;
            if (VAdc::read(v)) {
                sum += v;
                ++taken;
            }
        }
        VAdc::release();
    }
    (void)brio::Vref::configure(brio::VrefConfig{.level =
        brio::VrefLevel::v1_024});
    if (dac_up) {
        (void)brio::Dac::set(dac_level);   // resume the level in force
    }
    if (taken == 0u) {
        return false;
    }
    mv_out = static_cast<uint16_t>(sum / taken);
    return true;
}

// ---- events -----------------------------------------------------------------
struct Beat {};
struct StimTick {};

// ---- the world: the seeded stimulus -----------------------------------------
//
// The DAC on PA02 plays energy_link.hpp's two levels on its INTREF
// (SUPC SEL is parked at 1.024 V, so code ~= millivolt x 1023/1024):
// quiet, then bursts of `burst_len` ms separated by seeded gaps of
// gap_min + xorshift % gap_span. Burst START times (this board's
// millis - the same clock that stamps the witness) go into a table the
// verdict letter matches toggles against. The DUT is BLIND to all of
// this: it must detect the analog level for real.
constexpr uint16_t dac_code_of(uint16_t mv) {
    return static_cast<uint16_t>((static_cast<uint32_t>(mv) * 1023u) / 1024u);
}
constexpr uint16_t stim_quiet_code = dac_code_of(energy::stimulus_quiet_mv);
constexpr uint16_t stim_burst_code = dac_code_of(energy::stimulus_burst_mv);
constexpr uint8_t stim_max_bursts = 64;

struct Stim : brio::Fsm<Stim, StimTick> {
    static inline brio::EventQueue<Event, 2, P> queue;
    static inline brio::TimeEvent<P, Stim, StimTick> timer{StimTick{}};

    static inline uint32_t rng = 1;
    static inline uint16_t total = 0;
    static inline uint16_t played = 0;
    static inline uint16_t gap_min = 200;
    static inline uint16_t gap_span = 300;
    static inline uint16_t burst_len = 50;
    static inline bool in_burst = false;
    static inline bool active = false;
    static inline uint32_t starts[stim_max_bursts];

    static void init() { start(&running); }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](StimTick) { step(); return handled(); },
            [](auto) { return unhandled(); }
        );
    }

    static uint32_t next_gap() {
        rng = energy::xorshift32(rng);
        return gap_min + ((rng >> 4) % gap_span);
    }

    static void begin(uint32_t seed, uint16_t n, uint16_t gmin,
                      uint16_t gspan, uint16_t blen) {
        rng = (seed == 0u) ? 1u : seed;
        total = n > stim_max_bursts ? stim_max_bursts : n;
        played = 0;
        gap_min = gmin;
        gap_span = (gspan == 0u) ? 1u : gspan;
        burst_len = blen;
        in_burst = false;
        active = true;
        (void)dac_play(stim_quiet_code);
        timer.arm(next_gap());
    }

    static void step() {
        if (!active) {
            return;
        }
        if (!in_burst) {
            starts[played] = brio::Ticker::millis();
            (void)dac_play(stim_burst_code);
            in_burst = true;
            timer.arm(burst_len);
        } else {
            (void)dac_play(stim_quiet_code);
            in_burst = false;
            ++played;
            if (played >= total) {
                active = false;
                brio::print(serial, "STIM done: ", played, " bursts",
                            brio::crlf, "> ");
            } else {
                timer.arm(next_gap());
            }
        }
    }
};

// ---- the heartbeat: LED at 1 Hz = the meter is alive ------------------------
struct Heart : brio::Fsm<Heart, Beat> {
    static inline brio::EventQueue<Event, 2, P> queue;
    static inline brio::TimeEvent<P, Heart, Beat> beat{Beat{}};

    static void init() {
        Led::output();
        start(&running);
    }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                beat.arm_every(brio::ticks_from_ms<P>(500));
                return handled();
            },
            [](Beat) {
                Led::toggle();
                return handled();
            },
            [](auto) { return unhandled(); }
        );
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

    /// The idx-th argument as a number, or fallback when absent/garbled.
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
    static uint32_t arg_or(const Cmd& cmd, uint32_t fallback) {
        return arg_n(cmd, 0, fallback);
    }

    static void cmd_help(const Cmd&, Serial s) {
        brio::print(s,
            "energy_meter bring-up letters (../README.md):", brio::crlf,
            "  IMEAS [n]  n(<=64) conversions on the shunt sense pair,",
            brio::crlf,
            "             stats in raw counts and nV (R not applied)",
            brio::crlf,
            "  WIT [s]    count witness edges on PA04 vs bandgap for s s",
            brio::crlf,
            "  VMEAS [n]  node voltage on PB09, mean of n (1 count = 1 mV)",
            brio::crlf,
            "  ZERO       store current sense reading as the offset",
            brio::crlf,
            "  WOPEN / WSTAT / WCLOSE   the energy window (free-running",
            brio::crlf,
            "             integral, V at both ends, E = V*I*t)",
            brio::crlf,
            "  JARM / JLOG   the judge: witness groups decoded and",
            brio::crlf,
            "             timestamped (signatures + data toggles)",
            brio::crlf,
            "  UPTIME", brio::crlf);
    }

    static void cmd_uptime(const Cmd&, Serial s) {
        brio::TimeStamp ts;
        brio::Ticker::now(ts);
        brio::print(s, "uptime: ", ts, brio::crlf);
    }

    static void cmd_imeas(const Cmd& cmd, Serial s) {
        if (!sdadc_ok || win_open) {
            brio::print(s, "IMEAS: refused (no SDADC, or window open)",
                        brio::crlf);
            return;
        }
        const uint16_t n = static_cast<uint16_t>(arg_or(cmd, 32));
        const SenseStats st = sense_stats(n);
        if (st.taken == 0u) {
            brio::print(s, "IMEAS: no conversion completed", brio::crlf);
            return;
        }
        brio::print(s, "IMEAS n=", st.taken,
                    " raw mean=", st.raw_mean,
                    " min=", st.raw_low, " max=", st.raw_high,
                    " rms=", st.raw_rms, brio::crlf,
                    "  mean=", raw_to_nv(st.raw_mean), " nV",
                    "  rms=", raw_to_nv(static_cast<int32_t>(st.raw_rms)),
                    " nV  (divide by R_shunt for current)", brio::crlf);
    }

    static void cmd_wit(const Cmd& cmd, Serial s) {
        if (!ac_ok) {
            brio::print(s, "WIT: AC did not init at boot", brio::crlf);
            return;
        }
        brio::AcConfig c{};
        c.positive = brio::AcPositive::pin0;       // PA04 = AIN0
        c.negative = brio::AcNegative::bandgap;    // 1.024 V (SUPC SEL)
        c.interrupt_on = brio::AcInterrupt::toggle;
        c.speed = brio::AcSpeed::high;
        c.hysteresis = true;                        // continuous mode only
        if (!WitnessComp::configure(c) || !WitnessComp::enable(true)) {
            brio::print(s, "WIT: comparator refused", brio::crlf);
            return;
        }
        // Erratum 1.5.6: enabling with MUXNEG = bandgap can raise one
        // spurious COMP flag - clear before arming, as ac.hpp says.
        WitnessComp::clear_flag();
        wit_edges = 0;
        WitnessComp::arm(true);
        brio::Nvic::enable(brio::Ac::irq());

        const uint32_t seconds = arg_or(cmd, 5);
        const uint32_t t0 = brio::Ticker::millis();
        while (brio::Ticker::millis() - t0 < seconds * 1000u) {
        }

        WitnessComp::arm(false);
        const uint32_t edges = wit_edges;
        (void)WitnessComp::enable(false);
        brio::print(s, "WIT ", seconds, " s: ", edges, " edges (state now ",
                    WitnessComp::state() ? "high" : "low", ")", brio::crlf);
    }

    static void cmd_vmeas(const Cmd& cmd, Serial s) {
        if (win_open) {
            brio::print(s, "VMEAS: refused, window open (SEL switch would"
                        " move the SDADC's reference)", brio::crlf);
            return;
        }
        uint16_t mv = 0;
        const uint16_t n = static_cast<uint16_t>(arg_or(cmd, 32));
        if (!node_mv(mv, n > 256u ? 256u : (n == 0u ? 1u : n))) {
            brio::print(s, "VMEAS: no conversion completed", brio::crlf);
            return;
        }
        brio::print(s, "VMEAS mean=", mv, " mV", brio::crlf);
    }

    static void cmd_zero(const Cmd&, Serial s) {
        if (!sdadc_ok || win_open) {
            brio::print(s, "ZERO: refused", brio::crlf);
            return;
        }
        const SenseStats st = sense_stats(64);
        if (st.taken == 0u) {
            brio::print(s, "ZERO: no conversion completed", brio::crlf);
            return;
        }
        zero_raw = st.raw_mean;
        brio::print(s, "ZERO stored raw=", zero_raw, " (",
                    raw_to_nv(zero_raw) / 1000, " uV), rms=", st.raw_rms,
                    brio::crlf);
    }

    static void cmd_wopen(const Cmd&, Serial s) {
        if (!sdadc_ok || win_open) {
            brio::print(s, "WOPEN: refused", brio::crlf);
            return;
        }
        if (!node_mv(win_v_open_mv, 32)) {
            brio::print(s, "WOPEN: V measurement failed", brio::crlf);
            return;
        }
        {
            brio::InterruptGuard g;
            win_sum = 0;
            win_count = 0;
            win_max_raw = INT32_MIN;
            win_overruns = 0;
        }
        // The driver keeps 39.6.2.1's discipline: CTRLC only with the
        // converter disabled.
        if (!Sdadc::enable(false) || !Sdadc::free_running(true) ||
            !Sdadc::enable(true)) {
            brio::print(s, "WOPEN: converter refused", brio::crlf);
            return;
        }
        // START-AND-VERIFY: a START issued right on the heels of the
        // re-enable is swallowed (bench-caught: the same start() works
        // once the block has settled), so the stream is confirmed by its
        // FIRST RESULT before the ISR is armed - the standing RESRDY
        // then interrupts immediately and the accumulation begins there.
        // The first 2-3 results are the SINC settling (39.6.2.3):
        // 0.005% of a 10 s window, accepted and stated.
        Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
        bool streaming = false;
        for (uint8_t attempt = 0; attempt < 3u && !streaming; ++attempt) {
            if (!Sdadc::start()) {
                continue;
            }
            uint32_t spins = 300'000u;   // ~2 ms; a window is ~171 us
            while (spins-- != 0u && !Sdadc::ready()) {
            }
            streaming = Sdadc::ready();
        }
        if (!streaming) {
            brio::print(s, "WOPEN: stream did not start", brio::crlf);
            return;
        }
        Sdadc::arm(Sdadc::flag_resrdy);
        brio::Nvic::enable(Sdadc::irq());
        win_open = true;
        win_open_ms = brio::Ticker::millis();
        brio::print(s, "WOPEN V=", win_v_open_mv, " mV zero=", zero_raw,
                    brio::crlf);
    }

    /// Snapshot of the running accumulator, ISR-coherent.
    static void win_snapshot(int64_t& sum, uint32_t& count, int32_t& mx,
                             uint32_t& ovr) {
        brio::InterruptGuard g;
        sum = win_sum;
        count = win_count;
        mx = win_max_raw;
        ovr = win_overruns;
    }

    static void cmd_wstat(const Cmd&, Serial s) {
        if (!win_open) {
            brio::print(s, "WSTAT: no window open", brio::crlf);
            return;
        }
        int64_t sum;
        uint32_t count, ovr;
        int32_t mx;
        win_snapshot(sum, count, mx, ovr);
        if (count == 0u) {
            brio::print(s, "WSTAT: no samples yet", brio::crlf);
            return;
        }
        const int32_t mean =
            static_cast<int32_t>(sum / count) - zero_raw;
        const int32_t ua = static_cast<int32_t>(
            (static_cast<int64_t>(raw_to_nv(mean))) / r_shunt_milliohm);
        brio::print(s, "WSTAT t=", brio::Ticker::millis() - win_open_ms,
                    " ms n=", count, " I=", ua, " uA max_raw=",
                    mx - zero_raw, " ovr=", ovr, brio::crlf);
    }

    static void cmd_wclose(const Cmd&, Serial s) {
        if (!win_open) {
            brio::print(s, "WCLOSE: no window open", brio::crlf);
            return;
        }
        Sdadc::disarm(Sdadc::flag_resrdy);
        (void)Sdadc::enable(false);
        (void)Sdadc::free_running(false);
        (void)Sdadc::enable(true);
        win_open = false;
        int64_t sum;
        uint32_t count, ovr;
        int32_t mx;
        win_snapshot(sum, count, mx, ovr);
        uint16_t v_close = 0;
        (void)node_mv(v_close, 32);
        if (count == 0u) {
            brio::print(s, "WCLOSE: empty window", brio::crlf);
            return;
        }
        // t_us = n x 512/3 exactly (1024 CLK cycles at 6 MHz per sample)
        const int64_t t_us =
            (static_cast<int64_t>(count) * win_rate_den) / win_rate_num;
        const int32_t mean =
            static_cast<int32_t>(sum / count) - zero_raw;
        const int64_t i_na =
            (static_cast<int64_t>(raw_to_nv(mean)) * 1000) /
            r_shunt_milliohm;
        const int32_t v_mv = (win_v_open_mv + v_close) / 2;
        const int64_t p_uw = (i_na * v_mv) / 1'000'000;
        const int64_t e_uj = (p_uw * t_us) / 1'000'000;
        brio::print(s, "WCLOSE n=", count, " t=",
                    static_cast<uint32_t>(t_us / 1000), " ms ovr=", ovr,
                    brio::crlf,
                    "  I=", static_cast<int32_t>(i_na / 1000),
                    " uA  V=", win_v_open_mv, "/", v_close,
                    " mV  P=", static_cast<int32_t>(p_uw),
                    " uW  E=", static_cast<int32_t>(e_uj), " uJ",
                    brio::crlf);
    }

    static void cmd_diag(const Cmd&, Serial s) {
        brio::print(s, "CTRLA=", brio::hex(SDADC_REGS->SDADC_CTRLA),
                    " CTRLB=", brio::hex(SDADC_REGS->SDADC_CTRLB),
                    " CTRLC=", brio::hex(SDADC_REGS->SDADC_CTRLC),
                    " REFCTRL=", brio::hex(SDADC_REGS->SDADC_REFCTRL),
                    " INPUTCTRL=", brio::hex(SDADC_REGS->SDADC_INPUTCTRL),
                    brio::crlf,
                    "  INTFLAG=", brio::hex(SDADC_REGS->SDADC_INTFLAG),
                    " INTENSET=", brio::hex(SDADC_REGS->SDADC_INTENSET),
                    " SYNCBUSY=", brio::hex(SDADC_REGS->SDADC_SYNCBUSY),
                    " n=", win_count, brio::crlf);
        // Kick: one START, then watch INTFLAG with the NVIC line masked
        // so the ISR cannot eat the flag before we see it.
        brio::Nvic::disable(Sdadc::irq());
        Sdadc::clear_flags(Sdadc::flag_resrdy);
        const bool started = Sdadc::start();
        uint32_t spins = 3'000'000u;
        while (spins != 0u && !Sdadc::ready()) { --spins; }
        brio::print(s, "  kick: start=", started ? "ok" : "REFUSED",
                    " resrdy=", Sdadc::ready() ? "yes" : "NO",
                    " spins_left=", spins, brio::crlf);
        if (win_open) { brio::Nvic::enable(Sdadc::irq()); }
    }

    static void cmd_jarm(const Cmd&, Serial s) {
        if (!ac_ok) {
            brio::print(s, "JARM: AC did not init at boot", brio::crlf);
            return;
        }
        brio::AcConfig c{};
        c.positive = brio::AcPositive::pin0;       // PA04 = AIN0
        c.negative = brio::AcNegative::bandgap;    // 1.024 V (SUPC SEL)
        c.interrupt_on = brio::AcInterrupt::toggle;
        c.speed = brio::AcSpeed::high;
        c.hysteresis = true;
        if (!WitnessComp::configure(c) || !WitnessComp::enable(true)) {
            brio::print(s, "JARM: comparator refused", brio::crlf);
            return;
        }
        WitnessComp::clear_flag();   // erratum 1.5.6 discipline
        {
            brio::InterruptGuard g;
            wit_ring_n = 0;
            wit_group_edges = 0;
            wit_edges = 0;
        }
        WitnessComp::arm(true);
        brio::Nvic::enable(brio::Ac::irq());
        judge_armed = true;
        brio::print(s, "JARM: judge armed (state ",
                    WitnessComp::state() ? "high" : "low", ")", brio::crlf);
    }

    static void cmd_jlog(const Cmd&, Serial s) {
        if (!judge_armed) {
            brio::print(s, "JLOG: judge not armed (JARM first)", brio::crlf);
            return;
        }
        WitGroup local[wit_ring_size];
        uint8_t n;
        {
            brio::InterruptGuard g;
            // Close a stale in-flight group so a finished run drains
            // fully even with no further edge to push it out.
            if (wit_group_edges != 0u &&
                brio::Ticker::millis() - wit_last_ms > 2u) {
                wit_close_group();
            }
            n = wit_ring_n;
            for (uint8_t i = 0; i < n; ++i) {
                local[i].t_ms = wit_ring[i].t_ms;
                local[i].edges = wit_ring[i].edges;
            }
            wit_ring_n = 0;
        }
        brio::print(s, "JLOG ", n, " group(s):", brio::crlf);
        for (uint8_t i = 0; i < n; ++i) {
            const char* kind = "data";
            if (local[i].edges == 2u * energy::sig_park_enter) {
                kind = "PARK-ENTER";
            } else if (local[i].edges == 2u * energy::sig_park_leave) {
                kind = "PARK-LEAVE";
            } else if (local[i].edges != 1u) {
                kind = "?";
            }
            brio::print(s, "  t=", local[i].t_ms, " ms edges=",
                        local[i].edges, " ", kind, brio::crlf);
        }
    }

    /// STIM <seed> <n> <gap_min> <gap_span> - the burst length would be
    /// a 5th argument the 4-slot parser cannot carry, so it is FIXED at
    /// 50 ms in v1. The golden vector (the first four gaps derived from
    /// the seed) is printed for run.py to verify its own xorshift
    /// against - divergence invalidates the run.
    static void cmd_stim(const Cmd& cmd, Serial s) {
        if (!dac_up) {
            brio::print(s, "STIM: DAC did not init at boot", brio::crlf);
            return;
        }
        if (Stim::active) {
            brio::print(s, "STIM: already playing", brio::crlf);
            return;
        }
        const uint32_t seed = arg_n(cmd, 0, 0xC0FFEEu);
        const uint16_t n = static_cast<uint16_t>(arg_n(cmd, 1, 10));
        const uint16_t gmin = static_cast<uint16_t>(arg_n(cmd, 2, 200));
        const uint16_t gspan = static_cast<uint16_t>(arg_n(cmd, 3, 300));
        constexpr uint16_t blen = 50;
        // The golden vector: the first four gaps, from a scratch copy.
        uint32_t x = (seed == 0u) ? 1u : seed;
        brio::print(s, "STIM seed=", seed, " n=", n, " gaps=", gmin, "+",
                    gspan, " burst=", blen, " ms; first gaps:");
        for (uint8_t i = 0; i < 4u; ++i) {
            x = energy::xorshift32(x);
            brio::print(s, " ", gmin + ((x >> 4) % (gspan ? gspan : 1)));
        }
        brio::print(s, brio::crlf);
        Stim::begin(seed, n, gmin, gspan, blen);
    }

    static void cmd_sstat(const Cmd&, Serial s) {
        brio::print(s, "SSTAT ", Stim::active ? "playing " : "idle ",
                    Stim::played, "/", Stim::total, " bursts", brio::crlf);
    }

    /// Match the judge's data toggles against the stimulus start table:
    /// sequential assignment, latency = toggle - start. Consumes the
    /// witness ring (like JLOG) and the burst table.
    static void cmd_jverdict(const Cmd&, Serial s) {
        WitGroup local[wit_ring_size];
        uint8_t n;
        {
            brio::InterruptGuard g;
            if (wit_group_edges != 0u &&
                brio::Ticker::millis() - wit_last_ms > 2u) {
                wit_close_group();
            }
            n = wit_ring_n;
            for (uint8_t i = 0; i < n; ++i) {
                local[i].t_ms = wit_ring[i].t_ms;
                local[i].edges = wit_ring[i].edges;
            }
            wit_ring_n = 0;
        }
        const uint16_t bursts = Stim::played;
        uint16_t matched = 0;
        uint16_t k = 0;
        brio::print(s, "JVERDICT ", bursts, " burst(s):", brio::crlf);
        for (uint8_t i = 0; i < n && k < bursts; ++i) {
            if (local[i].edges != 1u) {
                continue;   // signatures and glitches are not completions
            }
            if (local[i].t_ms < Stim::starts[k]) {
                brio::print(s, "  spurious toggle at ", local[i].t_ms,
                            " ms", brio::crlf);
                continue;
            }
            // Skip bursts this toggle cannot belong to (missed ones).
            while (k + 1u < bursts && local[i].t_ms >= Stim::starts[k + 1u]) {
                brio::print(s, "  burst ", k, " at ", Stim::starts[k],
                            " ms: MISSED", brio::crlf);
                ++k;
            }
            brio::print(s, "  burst ", k, " at ", Stim::starts[k],
                        " ms: latency ", local[i].t_ms - Stim::starts[k],
                        " ms", brio::crlf);
            ++k;
            ++matched;
        }
        for (; k < bursts; ++k) {
            brio::print(s, "  burst ", k, " at ", Stim::starts[k],
                        " ms: MISSED", brio::crlf);
        }
        brio::print(s, "  matched ", matched, "/", bursts, brio::crlf);
    }

    static constexpr Router::Route routes[] = {
        {"HELP", cmd_help},
        {"DIAG", cmd_diag},
        {"JARM", cmd_jarm},
        {"JLOG", cmd_jlog},
        {"STIM", cmd_stim},
        {"SSTAT", cmd_sstat},
        {"JVERDICT", cmd_jverdict},
        {"UPTIME", cmd_uptime},
        {"IMEAS", cmd_imeas},
        {"WIT", cmd_wit},
        {"VMEAS", cmd_vmeas},
        {"ZERO", cmd_zero},
        {"WOPEN", cmd_wopen},
        {"WSTAT", cmd_wstat},
        {"WCLOSE", cmd_wclose},
    };
    static constexpr uint8_t route_count =
        sizeof(routes) / sizeof(routes[0]);
};

using SerialLines = brio::SerialPort<Serial, P, Console, 80>;

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void SERCOM5_Handler() {
    if (Serial::isr()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
// The window accumulator: one free-running result per interrupt, 64-bit
// sum (soft, ~cheap at 5.9 kHz on a 48 MHz core). Reading RESULT is the
// acknowledgement; the flag is cleared defensively too.
extern "C" void SDADC_Handler() {
    if (Sdadc::overrun()) {
        win_overruns = win_overruns + 1u;
    }
    const int32_t r = Sdadc::result24();
    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
    win_sum = win_sum + r;
    win_count = win_count + 1u;
    if (r > win_max_raw) {
        win_max_raw = r;
    }
}
// Bound even though only WIT arms it: an unbound vector is a silent
// spin at Default_Handler (the sleepwalk campaign's watchdog lesson).
extern "C" void AC_Handler() {
    const uint8_t flags = brio::Ac::take_flags();
    if ((flags & WitnessComp::flag) != 0u) {
        wit_edges = wit_edges + 1u;
        const uint32_t now = brio::Ticker::millis();
        if (wit_group_edges != 0u && now - wit_last_ms > 2u) {
            wit_close_group();
        }
        if (wit_group_edges == 0u) {
            wit_group_t0 = now;
        }
        wit_group_edges = wit_group_edges + 1u;
        wit_last_ms = now;
    }
}

int main()
{
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, console_baud);
    (void)brio::Ticker::init(clock);
    brio::enable_interrupts();

    // One reference level rules both instruments: SUPC.VREF.SEL at
    // 1.024 V is the SDADC's INTREF and the AC's bandgap threshold -
    // the witness threshold energy_link.hpp states.
    (void)brio::Vref::configure(brio::VrefConfig{.level =
        brio::VrefLevel::v1_024});
    static_assert(energy::witness_threshold_mv == 1024);

    sdadc_ok = Sdadc::init(main_gen, sense_cfg(), main_gen_hz) &&
               Sdadc::select(0);
    ac_ok = brio::Ac::init(main_gen);
    brio::Dac::claim_vout<brio::Pin<'A', 2>>();
    dac_up = brio::Dac::init(main_gen,
                             brio::DacConfig{.reference = brio::DacRef::intref,
                                             .external_output = true});
    if (dac_up) {
        (void)dac_play(stim_quiet_code);
    }

    if (serial_ok) {
        brio::print(serial, brio::crlf, "energy_meter (clk=",
                    clock_ok ? "OSC48M" : "FAILED",
                    ", sdadc=", sdadc_ok ? "up" : "FAILED",
                    ", ac=", ac_ok ? "up" : "FAILED",
                    "), type HELP", brio::crlf, "> ");
    }

    brio::Kernel<P, Console, SerialLines, Stim, Heart>::run();
}
