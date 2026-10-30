// rubato_passes.hpp - THE WAITS AND THE PASSES OF THE SUITES UNDER RUBATO:
// the target-free and stack-free half that every suite running a family's
// drivers inside Rubato, the preemptive kernel, compiles unchanged around
// its own pack - the Console's turn over the console, the base turn and the
// wait turns written out, PassSpec and the seven passes, the pass's
// statistics and its label, and the pressure's verdict. The link suites'
// body (rubato_link.hpp, the SPI and UART stacks) and the I2C suites' body
// (rubato_i2c.hpp, the I2C stack) each include it at the place their pack
// is complete; it is never included by a suite directly.
//
// APP-LEVEL BENCH TOOLING beside the protocol headers, included by relative
// path as they are; nothing in brio knows it exists, and nothing here names
// a register or a bus.
//
// WHAT THE INCLUDING BODY HAS DECLARED BEFORE THE INCLUDE, all inside the
// suite's anonymous namespace with `using namespace brio` in force:
//
//   core_hz, cycles_per_us   the core clock, in which the ruler counts
//   P                        the Rubato platform
//   K, TE                    the kernel type Rubato<P, ...> over the pack,
//                            and TimeEvents<P>
//   Ruler                    Ruler::now(): core cycles, monotone modulo
//                            2^32 (the link body's header says what a
//                            ruler must be)
//   spin(cycles)             a busy wait of at least `cycles` core cycles
//   Serial, serial, bench    the console's Uart and its test bench
//   console_banner()         the menu's print
//   Pressure                 the pack's highest object, with its dispatch
//                            count `Pressure::runs` and its `queue`
//   pressure_spin            the cycles Pressure's dispatch spins (set here
//                            per pass)
//   landing_slots, landed[]  the landing trace: one slot per object of the
//                            pack, by its index from the top, and the
//                            kernel loop's base in the last
//   landed_completion, landed_dma_completion, landed_marked, pressure_irqs
//                            what the pressure's interrupt records beside
//                            the slot (the including body's note_landing):
//                            landings inside the arbiter's completion, of
//                            those a DMA completion, landings with the
//                            PACK'S MARKED STATE set - LineSink on the
//                            stack for the link pack, the host on the wire
//                            inside start() for the I2C pack - and every
//                            interrupt
//   pressure_arm/go/halt/quiet(...)   the family's pressure timer (the
//                            including body declares them, the suite
//                            defines them)
//
// A LETTER'S WAITS ARE THE CONSOLE'S: each turn of a wait is Rubato's base
// turn written out (the matured time events posted with the ceiling held at
// 0, then the scheduler through the platform's entry and exit, base_turn()),
// because the kernel loop is below the Console and does not turn while a
// letter runs - so an arbiter's per-bus timeout is a live clock inside a
// letter.
//
// THE PASSES. Every letter runs its traffic seven times: a CONTROL with the
// pressure's timer silent, then under pressure at 97, 193 and 389 us with a
// 300-cycle spin, and again with a 3000-cycle spin. Every pass is judged on
// its own; a pass under pressure that fails where its control passes is a
// finding about the service, printed with its counters. A pressure pass
// also judges the pressure itself (judge_pressure()): the family's timer
// counts from the clock its core runs on, so over a pass of T core cycles
// it interrupts floor(T / (core MHz x period_us)) times, plus or minus one
// for the two ends of the window, and each interrupt is one dispatch, none
// dropped by Pressure's queue - which the family keeps true by keeping
// Pressure's dispatch, a spin of at most 3000 cycles and the scheduler's
// few hundred, shorter than its shortest period. A suite may add a pass fed
// by another source (a PassSpec naming its `source`, which every label of
// the pass carries), judged by the same arithmetic when that source counts
// from the same crystal. The pass's print, which names the pack's objects,
// is each body's own (print_pass()).

#pragma once

/// The console's received characters, each a letter or '?' (the console's
/// vector posts console_rx when a byte arrived).
void console_turn() {
    uint8_t c = 0;
    while (Serial::read_byte(c)) {
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            console_banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}

/// One turn of Rubato::run()'s loop, written out in the Console's dispatch:
/// the matured time events posted with the ceiling held at 0, then the
/// scheduler entered with it still there at the Console's own ceiling, as
/// the platform's entry hands it over, and then the platform's exit
/// (kernel/platform.hpp, the entry and the exit): the ceiling put back and
/// the count of quiet posts compared with the one activate() hands back,
/// both under one mask, and the scheduler run again from the ceiling at 0
/// when the count moved. Without the compare, a post landing in the run's
/// last choice - after its look at the receiver, with the ceiling at 0, so
/// counted quiet and asking for nothing - waits for the Console's next
/// turn, a whole wait turn late; and when the letter leaves its wait there
/// (pass_end()), the dispatch falls into the next pass, whose pressure
/// verdict then reads one over while this one reads one short. The
/// Console's own received byte moves the count too and costs one more
/// run, which finds nothing above the Console. The mask holds the store
/// and the compare alone (six instructions between the mask and the
/// unmask in the STM32G0B1RE's release listing), never activate().
void base_turn() {
    const uint8_t c = P::ceiling;
    P::ceiling = 0;
    TE::process();
    for (;;) {
        const Activation a = K::activate(c);
        typename P::CriticalSection cs;
        P::ceiling = c;
        if (*const_cast<const volatile uint8_t*>(&quiet_posts<P>) == a.count) {
            return;
        }
        P::ceiling = 0;
    }
}

/// A wait's turn: the base turn, then 500 to 1523 cycles of the Console's
/// own, the length from an 8-bit LFSR - so that the walk with the ceiling at
/// 0 is a few per cent of a wait and not most of it (a tick landing in the
/// walk waits for its end), and so that the wait's own period never locks
/// to the pressure's and steers where it lands.
uint8_t wait_lfsr = 0xA5;
void wait_turn() {
    base_turn();
    const uint8_t v = wait_lfsr;
    wait_lfsr = static_cast<uint8_t>((v << 1) | (((v >> 7) ^ (v >> 5) ^ (v >> 4) ^ (v >> 3)) & 1u));
    spin(500u + 4u * static_cast<uint32_t>(v));
}

template <typename Pred>
bool wait_until(Pred done, uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (!done()) {
        if (Ticker::millis() - t0 >= ms) {
            return done();
        }
        wait_turn();
    }
    return true;
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
        wait_turn();
    }
}

/// A body that never waits in microseconds leaves it uncalled, a function of
/// an unnamed namespace -Wunused-function would otherwise name.
[[maybe_unused]] void wait_us(uint32_t us) { spin(us * cycles_per_us); }

// =============================================================================
// The passes and the pressure
// =============================================================================

struct PassSpec {
    uint16_t period_us;   ///< 0: the control, the pressure's timer silent
    uint16_t spin;        ///< Pressure's dispatch, in core cycles
    /// The pressure's source, named in the pass's label: null for the
    /// family's own timer (pressure_name), a text for a pass a suite adds
    /// with another source ("core 1's doorbell").
    const char* source = nullptr;
};
constexpr PassSpec passes[] = {{0, 0},      {97, 300},   {193, 300}, {389, 300},
                               {97, 3000}, {193, 3000}, {389, 3000}};
constexpr uint8_t pass_count = sizeof passes / sizeof passes[0];

struct PassStats {
    uint32_t wall = 0;        ///< core cycles, the timer on to off (the control: the pass)
    uint32_t irqs = 0;
    uint32_t runs = 0;
    uint16_t overflows = 0;
    uint32_t land[landing_slots] = {};
    uint32_t completion = 0;
    uint32_t dma_completion = 0;
    uint32_t marked = 0;      ///< landings with the pack's marked state set (the file header)
};

uint32_t pass_t0 = 0;
uint32_t pass_runs0 = 0;
uint16_t pass_overflows0 = 0;

char label_buf[48];

/// "control: ", "97 us x 300: " or, for a pass with a source of its own,
/// "core 1's doorbell, 97 us x 3000: ", the prefix of a pass's verdicts.
const char* pass_label(const PassSpec& s) {
    uint8_t n = 0;
    auto put = [&n](char c) {
        if (n + 1u < sizeof label_buf) {
            label_buf[n++] = c;
        }
    };
    auto put_num = [&put](uint32_t v) {
        char d[6];
        uint8_t k = 0;
        do {
            d[k++] = static_cast<char>('0' + v % 10u);
            v /= 10u;
        } while (v != 0u && k < sizeof d);
        while (k != 0u) {
            put(d[--k]);
        }
    };
    if (s.period_us == 0u) {
        for (const char* p = "control: "; *p != '\0'; ++p) put(*p);
    } else {
        if (s.source != nullptr) {
            for (const char* p = s.source; *p != '\0'; ++p) put(*p);
            put(',');
            put(' ');
        }
        put_num(s.period_us);
        for (const char* p = " us x "; *p != '\0'; ++p) put(*p);
        put_num(s.spin);
        put(':');
        put(' ');
    }
    label_buf[n] = '\0';
    return label_buf;
}

void pass_begin(const PassSpec& s) {
    for (uint8_t i = 0; i < landing_slots; ++i) {
        landed[i] = 0;
    }
    landed_completion = 0;
    landed_dma_completion = 0;
    landed_marked = 0;
    pressure_irqs = 0;
    pass_runs0 = Pressure::runs;
    pass_overflows0 = Pressure::queue.overflows();
    pressure_spin = s.spin;
    if (s.period_us != 0u) {
        pressure_arm(s.period_us);
        pass_t0 = Ruler::now();
        pressure_go();
    } else {
        pass_t0 = Ruler::now();
    }
}

PassStats pass_end(const PassSpec& s) {
    PassStats st{};
    if (s.period_us != 0u) {
        pressure_halt();
        st.wall = Ruler::now() - pass_t0;
        pressure_quiet();
    } else {
        st.wall = Ruler::now() - pass_t0;
    }
    st.irqs = pressure_irqs;
    st.runs = Pressure::runs - pass_runs0;
    st.overflows = static_cast<uint16_t>(Pressure::queue.overflows() - pass_overflows0);
    for (uint8_t i = 0; i < landing_slots; ++i) {
        st.land[i] = landed[i];
    }
    st.completion = landed_completion;
    st.dma_completion = landed_dma_completion;
    st.marked = landed_marked;
    pressure_spin = 0;
    return st;
}

/// The pressure was real: one interrupt a period over the pass, plus or
/// minus one for the window's ends (the file header's arithmetic), every
/// one dispatched, none dropped.
void judge_pressure(const PassSpec& s, const PassStats& st) {
    if (s.period_us == 0u) {
        bench.verdict(pass_label(s), "the pressure's timer stayed silent", st.irqs == 0u);
        return;
    }
    const uint32_t period = static_cast<uint32_t>(s.period_us) * cycles_per_us;
    const uint32_t expect = st.wall / period;
    const bool count_ok = st.irqs + 1u >= expect && st.irqs <= expect + 1u;
    bench.verdict(pass_label(s), "Pressure ticked once a period, each tick one dispatch, none dropped",
                  count_ok && st.runs == st.irqs && st.overflows == 0u);
}
