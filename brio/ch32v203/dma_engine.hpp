/*
 * dma_engine.hpp
 *
 * What a TRANSPORT needs to know about the DMA controller without
 * including it: the "no engine" tag of every optional engine slot in
 * this stratum, and THE REQUEST TABLE.
 *
 * WHY THE TWO LIVE TOGETHER. On this family the channel IS the request
 * (RM 11.2.3): there is no multiplexer, table 11-5 wires each
 * peripheral event to one channel, and a transport with an engine slot
 * must be able to REFUSE an engine that names another channel. It can
 * only do that if it can read the table - and it must not include
 * ch32v203/dma.hpp for it, because a driver with an empty slot would
 * then drag the whole controller into every program with a console.
 * So the table is here, beside the tag, and it is the ONE place in the
 * stratum where a peripheral's channel number is written: the USART's
 * two channels, the SPI's, the I2C's, the ADC's and the timers' are all
 * read out of the same rows a reader can hold the manual against.
 *
 * `present` is the only thing a task asks the tag about, with
 * `if constexpr`, so every engine branch disappears from a driver that
 * does not name one.
 *
 * THE TABLE IS PER PART, not per family. Two classes share one map -
 * table 11-5 for the CH32V20x_D6 and table 11-6 for the D8, which is
 * the same map plus TIM5's six rows - but WHICH peripherals the part in
 * front of you offers is parts/<part>.hpp's business: the smallest
 * package has one usart and no I2C at all, so its USART3 and I2C1 rows
 * name a request nothing can raise. Such a row answers 0, `channel 0`
 * meaning "no channel" everywhere in this stratum, and
 * `DmaRequestOf<r>` turns that into a compile error on the line that
 * asked.
 */

#pragma once

#include <stdint.h>

#include "ch32v203/device.hpp"

namespace brio {

/// The empty engine slot.
struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

/// The channel an engine sits on - 0 for the empty slot, which has
/// none - so a transport can check the request table against a named
/// engine without asking the tag for a member it has not got.
template <typename E>
constexpr uint8_t dma_engine_channel() {
    if constexpr (E::present) {
        return E::channel;
    } else {
        return 0;
    }
}

/// Two engines on one transport must not name the same channel: a
/// channel moves data ONE way. Generic over any engine that says
/// `present` and `channel`.
template <typename Tx, typename Rx>
constexpr bool dma_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::channel != Rx::channel;
    } else {
        return true;
    }
}

// ---- the request table (RM 11.2.3, tables 11-5 and 11-6) ------------------

/**
 * Every peripheral event this family's one controller answers, named
 * the way the manual's table names it - with WCH's USART4 spelled the
 * way the rest of this stratum spells it, `uart4`, because that is what
 * the part's pin table and RCC call it.
 *
 * A timer publishes SEVERAL events and they do not share a channel: an
 * update is not a capture, and TIM1's trigger, commutation and fourth
 * channel are three names for one row (they raise the same request).
 */
enum class DmaRequest : uint8_t {
    adc1,
    spi1_rx, spi1_tx,
    spi2_rx, spi2_tx,
    usart1_tx, usart1_rx,
    usart2_rx, usart2_tx,
    usart3_tx, usart3_rx,
    uart4_tx, uart4_rx,
    i2c1_tx, i2c1_rx,
    i2c2_tx, i2c2_rx,
    tim1_ch1, tim1_ch2, tim1_ch3, tim1_ch4, tim1_trig, tim1_com, tim1_up,
    tim2_ch1, tim2_ch2, tim2_ch3, tim2_ch4, tim2_up,
    tim3_ch1, tim3_ch3, tim3_ch4, tim3_trig, tim3_up,
    tim4_ch1, tim4_ch2, tim4_ch3, tim4_up,
    tim5_ch1, tim5_ch2, tim5_ch3, tim5_ch4, tim5_trig, tim5_up,
};

/**
 * The channel that serves a request on THIS PART, or 0 when the part
 * has not got the peripheral that raises it.
 *
 * The rows are tables 11-5 and 11-6 read column by column. The two
 * tables are the same map: the D8 class adds TIM5's six events and
 * changes nothing else, so one function serves both classes and the
 * part's own table decides which rows exist.
 */
constexpr uint8_t dma_request_channel(DmaRequest r) {
    switch (r) {
        // ADC1 - every part of the series has at least one converter.
        case DmaRequest::adc1:      return device::adc_count >= 1u ? 1 : 0;

        // SPI1 and SPI2. The parts below the CH32V203C8 offer one SPI.
        case DmaRequest::spi1_rx:   return device::spi_count >= 1u ? 2 : 0;
        case DmaRequest::spi1_tx:   return device::spi_count >= 1u ? 3 : 0;
        case DmaRequest::spi2_rx:   return device::spi_count >= 2u ? 4 : 0;
        case DmaRequest::spi2_tx:   return device::spi_count >= 2u ? 5 : 0;

        // The four serial ports. Which of them a part offers is a mask
        // and not a count (parts/<part>.hpp): the smallest package
        // offers USART2 alone.
        case DmaRequest::usart1_tx: return device::has_usart(1) ? 4 : 0;
        case DmaRequest::usart1_rx: return device::has_usart(1) ? 5 : 0;
        case DmaRequest::usart2_rx: return device::has_usart(2) ? 6 : 0;
        case DmaRequest::usart2_tx: return device::has_usart(2) ? 7 : 0;
        case DmaRequest::usart3_tx: return device::has_usart(3) ? 2 : 0;
        case DmaRequest::usart3_rx: return device::has_usart(3) ? 3 : 0;
        case DmaRequest::uart4_tx:  return device::has_usart(4) ? 1 : 0;
        case DmaRequest::uart4_rx:  return device::has_usart(4) ? 8 : 0;

        // I2C1 and I2C2. One package of the nine has no I2C at all.
        case DmaRequest::i2c1_tx:   return device::i2c_count >= 1u ? 6 : 0;
        case DmaRequest::i2c1_rx:   return device::i2c_count >= 1u ? 7 : 0;
        case DmaRequest::i2c2_tx:   return device::i2c_count >= 2u ? 4 : 0;
        case DmaRequest::i2c2_rx:   return device::i2c_count >= 2u ? 5 : 0;

        // The advanced-control timer: one row for its trigger, its
        // commutation and its fourth channel, which share channel 4.
        case DmaRequest::tim1_ch1:  return device::advanced_timer_count >= 1u ? 2 : 0;
        case DmaRequest::tim1_ch2:  return device::advanced_timer_count >= 1u ? 3 : 0;
        case DmaRequest::tim1_ch4:
        case DmaRequest::tim1_trig:
        case DmaRequest::tim1_com:  return device::advanced_timer_count >= 1u ? 4 : 0;
        case DmaRequest::tim1_up:   return device::advanced_timer_count >= 1u ? 5 : 0;
        case DmaRequest::tim1_ch3:  return device::advanced_timer_count >= 1u ? 6 : 0;

        // The three general-purpose timers every part of the series
        // has. The fold is ch32v203/tim.hpp's tim_present(): the count
        // starts at TIM2, so TIM4 is the third of them.
        case DmaRequest::tim2_ch3:  return device::general_timer_count >= 1u ? 1 : 0;
        case DmaRequest::tim2_up:   return device::general_timer_count >= 1u ? 2 : 0;
        case DmaRequest::tim2_ch1:  return device::general_timer_count >= 1u ? 5 : 0;
        case DmaRequest::tim2_ch2:
        case DmaRequest::tim2_ch4:  return device::general_timer_count >= 1u ? 7 : 0;

        case DmaRequest::tim3_ch3:  return device::general_timer_count >= 2u ? 2 : 0;
        case DmaRequest::tim3_ch4:
        case DmaRequest::tim3_up:   return device::general_timer_count >= 2u ? 3 : 0;
        case DmaRequest::tim3_ch1:
        case DmaRequest::tim3_trig: return device::general_timer_count >= 2u ? 6 : 0;

        case DmaRequest::tim4_ch1:  return device::general_timer_count >= 3u ? 1 : 0;
        case DmaRequest::tim4_ch2:  return device::general_timer_count >= 3u ? 4 : 0;
        case DmaRequest::tim4_ch3:  return device::general_timer_count >= 3u ? 5 : 0;
        case DmaRequest::tim4_up:   return device::general_timer_count >= 3u ? 7 : 0;

        // Table 11-6's own six rows: the 32-bit timer of the other
        // device class, and of no part below it.
        case DmaRequest::tim5_ch2:  return device::has_tim5 ? 1 : 0;
        case DmaRequest::tim5_ch3:  return device::has_tim5 ? 3 : 0;
        case DmaRequest::tim5_ch4:  return device::has_tim5 ? 6 : 0;
        case DmaRequest::tim5_ch1:
        case DmaRequest::tim5_trig: return device::has_tim5 ? 7 : 0;
        case DmaRequest::tim5_up:   return device::has_tim5 ? 8 : 0;
    }
    return 0;
}

/// Whether THIS PART can raise that request at all.
constexpr bool dma_request_present(DmaRequest r) { return dma_request_channel(r) != 0u; }

/**
 * The channel of a request, as a compile-time constant that REFUSES a
 * peripheral this part has not got:
 *
 *     using Tx = brio::DmaTxEngine<brio::DmaRequestOf<brio::DmaRequest::usart2_tx>::channel>;
 *
 * which is how an application names the request rather than the number
 * and still gets the number the silicon wired.
 */
template <DmaRequest r>
struct DmaRequestOf {
    static_assert(dma_request_channel(r) != 0,
                  "brio DMA: this part does not offer the peripheral that raises that request "
                  "(brio/ch32v203/parts/ has its table)");
    static constexpr uint8_t channel = dma_request_channel(r);
};

} // namespace brio
