// tc_readsync_probe - the one-behind experiment.
//
// samc/tc.hpp's read_sync() returns the PREVIOUS READSYNC's snapshot
// (measured by test_samc_sleep: 0, 196, 201, 205 on a pair that had run
// six milliseconds). The candidate fix waits for SYNCBUSY.COUNT to RISE
// before waiting for it to fall. This probe, in order:
//
//   A  reproduces the defect with the CURRENT read_sync();
//   B  OBSERVES what SYNCBUSY and the COUNT shadow actually do after a
//      raw READSYNC command, sampled tight into RAM - the fix must come
//      from the observation, not from the guess;
//   C  runs the candidate fixed read four times (first value must be
//      the CURRENT count, not zero);
//   D  prices both reads, and re-checks the fixed one on a 48 MHz clock
//      (the rise must not become a hang when the crossing is fast);
//   E  the same reproduce/observe/fix on the TCC, whose read_sync() has
//      the same shape.
//
// PROBE, not a suite: prints everything once at boot and parks. No
// wires; the pair TC2+TC3 runs on OSCULP32K through generator 3.
//
// build: boards = c21j

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/sercom.hpp"
#include "samc/tc.hpp"
#include "samc/tcc.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"

using P = brio::SamPlatform;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

constexpr brio::UartPads console_pads{
    .tx = brio::SercomPad::pad0,
    .rx = brio::SercomPad::pad1,
    .tx_pin = {'B', 30, brio::PinFunction::d},
    .rx_pin = {'B', 31, brio::PinFunction::d},
};
using Serial = brio::Uart<5, console_pads>;
constexpr Serial serial;
using brio::crlf;
using brio::print;

using Watch = brio::Tc<2>;          // TC2+TC3 pair, 32-bit
using Wave = brio::Tcc<0>;
constexpr uint8_t gen_slow = 3;     // OSCULP32K, 32768 Hz
constexpr uint8_t gen_fast = 0;     // OSC48M

void wait_ms(uint32_t ms) {
    const uint32_t until = brio::Ticker::ticks() + ms + 1u;
    while (brio::Ticker::ticks() < until) {
    }
}

// ---- the candidate fixed read ----------------------------------------------
//
// READSYNC, then wait the COUNT bit to RISE (bounded - on a fast clock
// the whole crossing may finish between two checks), then to FALL, then
// load. The rise bound is small on purpose: if the bit never shows, the
// crossing either already completed (new value in place) or the silicon
// does not pulse it - phase B says which.

// The DOUBLE-COMMAND read, designed from phase B's observation: the
// shadow lands about half a slow period AFTER SYNCBUSY.CTRLB clears,
// with no bit advertising it. A second READSYNC's own crossing covers
// the first's landing gap, so what this returns is the count AT THE
// FIRST COMMAND - the verb's entry time, which is the honest contract.
uint32_t fixed_read_tc(uint32_t spins = 400'000u) {
    for (int pass = 0; pass < 2; ++pass) {
        Watch::regs().TC_CTRLBSET =
            TC_CTRLBSET_CMD(TC_CTRLBSET_CMD_READSYNC_Val);
        uint32_t f = spins;
        while ((Watch::regs().TC_SYNCBUSY &
                (TC_SYNCBUSY_CTRLB_Msk | TC_SYNCBUSY_COUNT_Msk)) != 0u &&
               f-- != 0u) {
        }
    }
    return Watch::count32_raw();
}

uint32_t fixed_read_tcc(uint32_t spins = 400'000u) {
    for (int pass = 0; pass < 2; ++pass) {
        Wave::regs().TCC_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC;
        uint32_t f = spins;
        while ((Wave::regs().TCC_SYNCBUSY &
                (TCC_SYNCBUSY_CTRLB_Msk | TCC_SYNCBUSY_COUNT_Msk)) != 0u &&
               f-- != 0u) {
        }
    }
    return Wave::count_raw();
}

// ---- phase B: observe ------------------------------------------------------

constexpr uint32_t obs_n = 1200;
volatile uint32_t obs_sync[obs_n];
volatile uint32_t obs_count[obs_n];

template <typename Sync, typename Count>
void observe(Sync sync, Count count, const char* tag, uint32_t cmd_write()) {
    // A tight sampling loop: sample index IS the timestamp (a handful
    // of CPU cycles per iteration against a 30.5 us slow-clock period).
    (void)cmd_write();
    for (uint32_t i = 0; i < obs_n; ++i) {
        obs_sync[i] = sync();
        obs_count[i] = count();
    }
    print(serial, "  ", tag, " transitions (idx: SYNCBUSY, COUNTraw):", crlf);
    uint32_t last_s = obs_sync[0] + 1u;  // force the first line
    uint32_t last_c = obs_count[0] + 1u;
    for (uint32_t i = 0; i < obs_n; ++i) {
        if (obs_sync[i] != last_s || obs_count[i] != last_c) {
            print(serial, "    ", i, ": ", brio::hex(obs_sync[i]), ", ",
                  obs_count[i], crlf);
            last_s = obs_sync[i];
            last_c = obs_count[i];
        }
    }
}

uint32_t tc_cmd() {
    Watch::regs().TC_CTRLBSET =
        TC_CTRLBSET_CMD(TC_CTRLBSET_CMD_READSYNC_Val);
    return 0;
}
uint32_t tcc_cmd() {
    Wave::regs().TCC_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC;
    return 0;
}

bool watch_up(uint8_t generator) {
    Watch::release();
    if (!Watch::init(generator)) {
        return false;
    }
    if (!Watch::configure(brio::TcConfig{.mode = brio::TcMode::count32,
                                         .prescaler = brio::TcPrescaler::div1})) {
        return false;
    }
    return Watch::enable(true);
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { Serial::isr(); }

int main() {
    (void)SysClock::init();
    (void)Serial::init(clock, 115200);
    (void)brio::Ticker::init(clock);
    (void)brio::Gclk<gen_slow>::configure(
        {.source = brio::GclkSource::osculp32k, .div = 0});
    brio::enable_interrupts();

    print(serial, crlf, "tc_readsync_probe", crlf);

    // ---- A: reproduce with the CURRENT read_sync ----
    print(serial, "A  reproduce (TC pair at 32768 Hz, 6 ms of running):", crlf);
    (void)watch_up(gen_slow);
    wait_ms(6);
    {
        const uint32_t r1 = Watch::count32(), r2 = Watch::count32(),
                       r3 = Watch::count32(), r4 = Watch::count32();
        print(serial, "   current read_sync x4: ", r1, " ", r2, " ", r3, " ",
              r4, crlf);
    }

    // ---- B: observe SYNCBUSY + shadow after one raw READSYNC ----
    print(serial, "B  observe after one raw READSYNC:", crlf);
    (void)watch_up(gen_slow);
    wait_ms(6);
    observe([] { return static_cast<uint32_t>(Watch::regs().TC_SYNCBUSY); },
            [] { return Watch::count32_raw(); }, "TC", tc_cmd);

    // ---- C: the candidate fixed read ----
    print(serial, "C  fixed read (rise-then-fall) x4 after 6 ms:", crlf);
    (void)watch_up(gen_slow);
    wait_ms(6);
    {
        const uint32_t r1 = fixed_read_tc(), r2 = fixed_read_tc(),
                       r3 = fixed_read_tc(), r4 = fixed_read_tc();
        print(serial, "   fixed: ", r1, " ", r2, " ", r3, " ", r4, crlf);
    }

    // ---- D: cost, and the fast-clock check ----
    {
        (void)watch_up(gen_slow);
        wait_ms(6);
        const uint32_t t0 = brio::Ticker::ticks();
        for (uint32_t i = 0; i < 256; ++i) {
            (void)Watch::count32();
        }
        const uint32_t t1 = brio::Ticker::ticks();
        for (uint32_t i = 0; i < 256; ++i) {
            (void)fixed_read_tc();
        }
        const uint32_t t2 = brio::Ticker::ticks();
        print(serial, "D  cost of 256 reads at 32 kHz: current ", t1 - t0,
              " ms, fixed ", t2 - t1, " ms", crlf);

        (void)watch_up(gen_fast);
        wait_ms(2);
        {
            const uint32_t r1 = fixed_read_tc(), r2 = fixed_read_tc(),
                           r3 = fixed_read_tc(), r4 = fixed_read_tc();
            print(serial, "   fast clock (48 MHz) fixed x4: ", r1, " ", r2,
                  " ", r3, " ", r4, crlf);
        }
        const uint32_t t3 = brio::Ticker::ticks();
        for (uint32_t i = 0; i < 256; ++i) {
            (void)fixed_read_tc();
        }
        const uint32_t t4 = brio::Ticker::ticks();
        for (uint32_t i = 0; i < 256; ++i) {
            (void)Watch::count32();
        }
        const uint32_t t5 = brio::Ticker::ticks();
        print(serial, "   256 reads at 48 MHz: fixed ", t4 - t3,
              " ms, current ", t5 - t4, " ms", crlf);
    }

    // ---- E: the TCC, same shape ----
    print(serial, "E  TCC0 at 32768 Hz:", crlf);
    Wave::release();
    (void)Wave::init(gen_slow);
    (void)Wave::configure(brio::TccConfig{});
    (void)Wave::wave(brio::TccWaveConfig{});
    (void)Wave::enable(true);
    wait_ms(6);
    {
        const uint32_t r1 = Wave::count(), r2 = Wave::count(),
                       r3 = Wave::count(), r4 = Wave::count();
        print(serial, "   current count() x4: ", r1, " ", r2, " ", r3, " ",
              r4, crlf);
    }
    Wave::release();
    (void)Wave::init(gen_slow);
    (void)Wave::configure(brio::TccConfig{});
    (void)Wave::wave(brio::TccWaveConfig{});
    (void)Wave::enable(true);
    wait_ms(6);
    observe([] { return Wave::regs().TCC_SYNCBUSY; },
            [] { return Wave::count_raw(); }, "TCC", tcc_cmd);
    {
        const uint32_t r1 = fixed_read_tcc(), r2 = fixed_read_tcc(),
                       r3 = fixed_read_tcc(), r4 = fixed_read_tcc();
        print(serial, "   fixed x4: ", r1, " ", r2, " ", r3, " ", r4, crlf);
    }

    print(serial, "DONE", crlf);
    for (;;) {
        P::idle();
    }
}
