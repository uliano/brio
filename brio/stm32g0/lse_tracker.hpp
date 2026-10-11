/*
 * lse_tracker.hpp
 *
 * THE CORE MEASURED AGAINST THE LSE CRYSTAL, ALL THE TIME, BY HARDWARE -
 * the frequency measurement RM0444 5.2 draws for TIM16 (figure 13: TI1
 * taken from LSI, LSE, the RTC wake-up or MCO2 by TIM16_TISEL), turned
 * from a one-shot measurement into a running one.
 *
 * WHY A RUNNING ONE. This family's boards run the core on HSI16 through
 * the PLL (a Nucleo-64's HSE is not fitted), and the HSI16 does not just
 * sit off nominal by its trim - it WANDERS, hundreds of ppm over seconds
 * (docs/stm32g0/clock.md, "The core against the LSE, tracked"). A rate
 * measured once is no time base a minute later; a rate measured over the
 * stretch that matters - the span a receiver timed, the window a run
 * took - is.
 *
 * THE ENGINE, from the chapter's parts:
 *  - TIM16 channel 1 capturing TI1 = LSE (TISEL code 2,
 *    RtcDomain::lse_tim16_ti1_code), every EIGHTH rising edge (ICPSC
 *    /8, the largest capture prescaler: 4096 captures a second instead
 *    of 32768), through the input filter (ICF 8: six samples at
 *    fDTS / 8 - an unfiltered capture of an internal clock line
 *    over-captures, docs/stm32g0/rtc.md), the counter free-running over
 *    16 bits at TIMPCLK with no prescaler;
 *  - a DMA channel on TIM16_CH1's request (DMAMUX, RM0444 table 55)
 *    copying each capture, CIRCULAR, into a ring of `entries` half-words
 *    - no CPU and no interrupt per capture, ever;
 *  - the rate over the last k captures is then the sum of k 16-bit
 *    differences (each interval is 8 x TIMPCLK / 32768 counts - 15625 at
 *    64 MHz, under the counter's wrap at every clock this family runs)
 *    against k x 8 periods of the crystal.
 * Declined, each for its reason: TIM14's RTCCLK input (the LSE only
 * while RTCSEL says so, and RTCSEL is one-way, rtc.md); TIM17 (no LSE
 * input); an interrupt per capture (4096 entries a second for what one
 * DMA request does); a capture every edge (eight times the bus traffic
 * for a resolution the averaging already gives).
 *
 * THE WALK IS THE READ, AND IT RACES THE CHANNEL. A capture lands every
 * 244 us and overwrites the OLDEST entry, so hz() walks back from the
 * channel's write position, one subtraction and one add an entry, no
 * division inside (a modulo a step, a library call on this core, was
 * measured costing several capture periods and reading a torn ring), and
 * stops `guard` entries short of the write head: thirteen Thumb
 * instructions an entry in the release listing, some 1.3 ms for the
 * whole of a 4096 ring at 64 MHz, the one 64-bit division after the
 * loop. Its result is the timer clock's rate over the k x 244 us that
 * ENDED when it began.
 *
 * WHAT IT DOES NOT DO: start the crystal. The LSE is the RTC domain's
 * (RtcDomain::lse_enable(), behind PWR's DBP gate), so start() refuses
 * until RtcDomain::lse_ready(). It owns Tim<16> and the DMA channel it is
 * given from start() to stop().
 *
 *   using Tracker = brio::LseTracker<1, 2>;     // DMA1 channel 2, 1024 entries
 *   Tracker::start(clock);                      // after the LSE is ready
 *   ...                                         // a quarter second later
 *   const auto hz = Tracker::hz_over(counts);   // the core over that span
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "stm32g0xx.h"

#include "stm32g0/dma.hpp"
#include "stm32g0/rtc.hpp"
#include "stm32g0/tim.hpp"

namespace brio {

template <uint8_t dma_n, uint8_t dma_ch, uint16_t entries = 1024>
class LseTracker {
    static_assert(entries >= 64u && entries <= 8192u,
                  "brio LseTracker: a ring of 64 to 8192 captures (16 ms to 2 s of the crystal)");

public:
    using Meter = Tim<16>;
    using Channel = DmaChannel<dma_n, dma_ch>;

    /// LSE periods between two captures (ICPSC /8).
    static constexpr uint32_t periods = 8;
    /// The crystal's frequency, the reference every reading is against.
    static constexpr uint32_t lse_hz = 32'768;
    /// How far short of the channel's write head a walk stops.
    static constexpr uint16_t guard = 8;
    /// The longest walk, in capture intervals.
    static constexpr uint16_t most = static_cast<uint16_t>(entries - guard - 1u);
    /// One interval's length, in microseconds of the crystal.
    static constexpr uint32_t interval_us = periods * 1'000'000u / lse_hz;   // 244
    /// The whole ring's span, in milliseconds: how long after start() the
    /// first full walk is possible.
    static constexpr uint32_t entries_ms = static_cast<uint32_t>((uint64_t{entries} * periods * 1000u) / lse_hz);

    LseTracker() = delete;

    /**
     * Program TIM16 and the channel and let them run. `clock` names the
     * timer clock the reading is checked against (TIMPCLK, tim_clock_hz):
     * a reading more than five per cent from it is not believed - no ring
     * of captures yet, or not this crystal. False while the LSE is not
     * ready, or when a verb refuses. The first full walk is a quarter
     * second away at the default size (filled()).
     */
    template <typename Clock>
    static bool start(const Clock& clock) {
        static_assert(tim_clock_hz(Clock{}) >= 8'000'000u,
                      "brio LseTracker: below 8 MHz the filter's six samples at fDTS / 8 outlast half "
                      "an LSE period, and an interval is under 2000 counts");
        nominal_ = tim_clock_hz(clock);
        lo_ = nominal_ / 100u * 95u;    // the belief window, computed once
        hi_ = nominal_ / 100u * 105u;
        stop();
        if (!RtcDomain::lse_ready()) {
            return false;
        }
        Meter::init();
        if (!Meter::configure({.prescaler = 0, .period = 0xFFFF}) ||
            !Meter::capture_channel(0, {.select = TimChannelSelect::direct,
                                        .polarity = TimCapturePolarity::rising,
                                        .prescaler = TimCapturePrescaler::every8,
                                        .filter = 8}) ||
            !Meter::input_select(0, RtcDomain::lse_tim16_ti1_code)) {
            Meter::release();
            return false;
        }
        Dma<dma_n>::bus_clock(true);
        Channel::set_peripheral(Meter::ccr_address(0));
        if (!DmaMux::request(Channel::mux_channel, Meter::dma_compare_request(0))) {
            Meter::release();
            return false;
        }
        Channel::restart(ccr_word, ring_, entries);
        Meter::interrupts(Meter::compare_dma(0), true);
        Meter::enable(true);
        running_ = true;
        return true;
    }

    /// The channel stopped and its multiplexer line given back, TIM16
    /// released. The ring keeps its last captures; nothing reads them.
    static void stop() {
        if (!running_) {
            return;
        }
        Meter::interrupts(Meter::compare_dma(0), false);
        Channel::stop();
        (void)DmaMux::release(Channel::mux_channel);
        Meter::release();
        running_ = false;
    }

    static bool running() { return running_; }

    /// Capture intervals a walk can use now: the ring's whole length less
    /// the guard once the channel has gone round (its completion flag,
    /// which nothing else clears), the captures so far on the first lap.
    static uint16_t filled() {
        if (!running_) {
            return 0;
        }
        if (Channel::flag(DmaFlag::complete)) {
            return most;
        }
        const uint16_t written = static_cast<uint16_t>(entries - Channel::count());
        return written > guard + 1u ? static_cast<uint16_t>(written - guard - 1u) : 0u;
    }

    /**
     * The timer clock's rate over the last `k` capture intervals (k x
     * 244 us), in hertz; nothing when the ring does not hold k of them
     * or the reading is not believable. `k` beyond the ring is clipped.
     */
    static std::optional<uint32_t> hz(uint16_t k) {
        const uint16_t have = filled();
        if (k > have) {
            k = have;
        }
        if (k == 0u) {
            return std::nullopt;
        }
        // The newest COMPLETE entry is the one behind the channel's position.
        const uint16_t pos = static_cast<uint16_t>(entries - Channel::count());
        uint16_t i = pos == 0u ? static_cast<uint16_t>(entries - 1u) : static_cast<uint16_t>(pos - 1u);
        uint16_t newer = ring_[i];
        uint32_t sum = 0;
        for (uint16_t j = 0; j < k; ++j) {
            i = i == 0u ? static_cast<uint16_t>(entries - 1u) : static_cast<uint16_t>(i - 1u);
            const uint16_t older = ring_[i];
            sum += static_cast<uint16_t>(newer - older);
            newer = older;
        }
        // k intervals of `periods` LSE periods: k x 8 / 32768 seconds.
        const uint64_t rate = (static_cast<uint64_t>(sum) * (lse_hz / periods) + k / 2u) / k;
        if (rate < lo_ || rate > hi_) {
            return std::nullopt;
        }
        return static_cast<uint32_t>(rate);
    }

    /// The rate over a stretch of `counts` timer counts that ENDED NOW:
    /// the intervals covering it and two more, sixteen at least.
    static std::optional<uint32_t> hz_over(uint32_t counts) {
        const uint32_t per = nominal_ / (lse_hz / periods);   // counts an interval
        uint32_t k = (per != 0u ? counts / per : 0u) + 2u;
        k = k < 16u ? 16u : (k > most ? most : k);
        return hz(static_cast<uint16_t>(k));
    }

    /// The same as an error against the nominal timer clock, in ppm
    /// (positive: the clock runs fast).
    static std::optional<int32_t> ppm_over(uint32_t counts) {
        const std::optional<uint32_t> f = hz_over(counts);
        if (!f || nominal_ == 0u) {
            return std::nullopt;
        }
        return static_cast<int32_t>((static_cast<int64_t>(*f) - static_cast<int64_t>(nominal_)) *
                                    1'000'000 / static_cast<int64_t>(nominal_));
    }

    /// The nominal timer clock start() was given.
    static uint32_t nominal_hz() { return nominal_; }

private:
    /// VERY HIGH, by design/dma.md's rule: a capture register the channel
    /// did not empty in time is overwritten by the next capture (CC1OF) -
    /// a starved capture channel overruns like a receive ring. One
    /// request in 244 us asks almost nothing of the bus.
    static constexpr uint32_t ccr_word = dma_ccr_config({.direction = DmaDirection::peripheral_to_memory,
                                                         .circular = true,
                                                         .memory_increment = true,
                                                         .peripheral_width = DmaWidth::half,
                                                         .memory_width = DmaWidth::half,
                                                         .priority = DmaPriority::very_high});

    inline static volatile uint16_t ring_[entries] = {};
    inline static uint32_t nominal_ = 0;
    inline static uint32_t lo_ = 0, hi_ = 0;
    inline static bool running_ = false;
};

}  // namespace brio
