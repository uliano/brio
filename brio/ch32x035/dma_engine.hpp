/*
 * dma_engine.hpp
 *
 * What a TRANSPORT needs to know about the DMA controller without
 * including it: the "no engine" tag of every optional engine slot in
 * this stratum, the priority the engines in those slots arm at, and THE
 * REQUEST TABLE (RM 9.2.3).
 *
 * WHY THEY LIVE TOGETHER. On this series the channel IS the request
 * (9.2.3, figure 9-1: each channel ORs the requests of a fixed handful of
 * peripherals, there is no multiplexer), and a transport with an engine
 * slot must REFUSE an engine that names another channel - a wrong channel
 * waits for a request that never comes, with no error flag. It can only
 * do that if it can read the table, and it must not include
 * ch32x035/dma.hpp for it, or every program with a console would carry
 * the controller. So the table is here, beside the tag, and it is the ONE
 * place in the stratum where a peripheral's channel number is written.
 *
 * ONE CONTROLLER, EIGHT CHANNELS: a request lands on a channel number and
 * nothing else names it. Which peripherals the part in front of you
 * offers is parts/<part>.hpp's business - the 20-pin CH32X035 parts offer
 * no USART1, the CH32X035F7P6 no USART4 - and such a row answers channel
 * 0, "no channel" everywhere in this stratum, which `DmaRequestOf<r>`
 * turns into a compile error on the line that asked. The ADC, the SPI,
 * the I2C and the two advanced timers are every part's (the datasheet's
 * model table); the I2C's row stands on every part and its block may be
 * withdrawn by the lot note (parts/<part>.hpp), which no register says.
 *
 * `present` is the only thing a task asks the tag about, with
 * `if constexpr`, so every engine branch disappears from a driver that
 * does not name one - and an engine reaches the driver only through its
 * own published names (start, service, block_flags, the flag names),
 * never the controller's.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/device.hpp"

namespace brio {

/// The empty engine slot.
struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

/// The channel an engine sits on - 0 for the empty slot, which has none -
/// so a transport can check the request table against a named engine
/// without asking the tag for a member it has not got.
template <typename E>
constexpr uint8_t dma_engine_channel() {
    if constexpr (E::present) {
        return E::channel;
    } else {
        return 0;
    }
}

/// Two engines on one transport must not name the same channel: a channel
/// moves data ONE way. Generic over any engine that says `present` and
/// `channel`.
template <typename Tx, typename Rx>
constexpr bool dma_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::channel != Rx::channel;
    } else {
        return true;
    }
}

/**
 * CFGR.PL: the software half of the arbitration, the channel number
 * deciding between equal levels - the lower number wins (RM 9.2.1). It
 * lives here, beside the tag, because a transport names the level of the
 * engines in its slots and must not include the controller to do it.
 *
 * THE LEVEL IS THE PERIPHERAL'S (docs/design/dma.md). An engine whose
 * peripheral OVERRUNS when its channel is starved - a receive ring, an SPI
 * host's receive, a converter's stream - arms at `very_high`; one whose
 * peripheral only waits or holds its last value - a transmit, a paced
 * output, an I2C host's receive under its stretched clock - at `high`; a
 * memory-to-memory copy at `low`, because no request paces it and it asks
 * for the bus without a pause, so a channel it outranks moves nothing until
 * its block is over. The engines' defaults are those three by direction
 * (ch32x035/dma.hpp), and every driver arms its engines at its own level,
 * with its chapter's reason beside the call.
 */
enum class DmaPriority : uint8_t { low = 0, medium = 1, high = 2, very_high = 3 };

// ---- the request table (RM 9.2.3) ---------------------------------------------

/**
 * Every peripheral event the controller answers, named the way figure 9-1
 * and the table under it name them. A timer publishes SEVERAL events and
 * they do not share a channel: an update is not a capture, and a trigger,
 * a commutation and a channel can be three names for one row (they raise
 * the same request). The series has one SPI and one I2C, and TIM3 raises
 * no DMA request at all (it is in no row).
 */
enum class DmaRequest : uint8_t {
    adc1,
    spi1_rx, spi1_tx,
    usart1_tx, usart1_rx,
    usart2_rx, usart2_tx,
    usart3_tx, usart3_rx,
    usart4_tx, usart4_rx,
    i2c1_tx, i2c1_rx,
    tim1_ch1, tim1_ch2, tim1_ch3, tim1_ch4, tim1_trig, tim1_com, tim1_up,
    tim2_ch1, tim2_ch2, tim2_ch3, tim2_ch4, tim2_trig, tim2_com, tim2_up,
};

/// The controller's channels, 1..8 (9.1).
inline constexpr uint8_t dma_channel_count = 8;

/**
 * The channel that serves a request on THIS PART, or 0 when the part has
 * not got the peripheral that raises it - figure 9-1 read channel by
 * channel, cross-checked cell by cell against the table printed under it:
 *
 *   1  ADC, USART4_TX, TIM2_CH3
 *   2  SPI_RX, USART3_TX, TIM1_CH1, TIM2_UP
 *   3  SPI_TX, USART3_RX, TIM1_CH2
 *   4  USART1_TX, TIM1_CH4, TIM1_TRIG, TIM1_COM
 *   5  USART1_RX, TIM1_UP, TIM2_CH1
 *   6  USART2_RX, I2C_TX, TIM1_CH3
 *   7  USART2_TX, I2C_RX, TIM2_CH2, TIM2_CH4
 *   8  USART4_RX, TIM2_TRIG, TIM2_COM
 */
constexpr uint8_t dma_request_channel(DmaRequest r) {
    const auto usart = [](uint8_t n, uint8_t ch) -> uint8_t { return device::has_usart(n) ? ch : uint8_t{0}; };
    switch (r) {
        case DmaRequest::adc1:      return 1;
        case DmaRequest::spi1_rx:   return 2;
        case DmaRequest::spi1_tx:   return 3;
        case DmaRequest::usart1_tx: return usart(1, 4);
        case DmaRequest::usart1_rx: return usart(1, 5);
        case DmaRequest::usart2_rx: return usart(2, 6);
        case DmaRequest::usart2_tx: return usart(2, 7);
        case DmaRequest::usart3_tx: return usart(3, 2);
        case DmaRequest::usart3_rx: return usart(3, 3);
        case DmaRequest::usart4_tx: return usart(4, 1);
        case DmaRequest::usart4_rx: return usart(4, 8);
        case DmaRequest::i2c1_tx:   return 6;
        case DmaRequest::i2c1_rx:   return 7;
        case DmaRequest::tim1_ch1:  return 2;
        case DmaRequest::tim1_ch2:  return 3;
        case DmaRequest::tim1_ch4:
        case DmaRequest::tim1_trig:
        case DmaRequest::tim1_com:  return 4;
        case DmaRequest::tim1_up:   return 5;
        case DmaRequest::tim1_ch3:  return 6;
        case DmaRequest::tim2_ch3:  return 1;
        case DmaRequest::tim2_up:   return 2;
        case DmaRequest::tim2_ch1:  return 5;
        case DmaRequest::tim2_ch2:
        case DmaRequest::tim2_ch4:  return 7;
        case DmaRequest::tim2_trig:
        case DmaRequest::tim2_com:  return 8;
    }
    return 0;
}

/// Whether THIS PART can raise that request at all.
constexpr bool dma_request_present(DmaRequest r) { return dma_request_channel(r) != 0u; }

/**
 * The channel of a request, as a compile-time constant that REFUSES a
 * peripheral this part has not got:
 *
 *     using Tx = brio::DmaRequestOf<brio::DmaRequest::usart4_tx>;
 *     using Engine = brio::DmaTxEngine<Tx::channel>;
 *
 * which is how an application names the request rather than the number
 * and still gets the number the silicon wired.
 */
template <DmaRequest r>
struct DmaRequestOf {
    static_assert(dma_request_present(r),
                  "brio DMA: this part does not offer the peripheral that raises that request "
                  "(brio/ch32x035/parts/ has its table)");
    static constexpr uint8_t channel = dma_request_channel(r);
};

/// The channel each USART direction requests on, read out of the table:
/// USART1 transmits on 4 and receives on 5, USART2 on 7 and 6, USART3 on 2
/// and 3, USART4 on 1 and 8. Zero for an instance this part has not got.
constexpr uint8_t usart_dma_tx_channel(uint8_t n) {
    return n == 1 ? dma_request_channel(DmaRequest::usart1_tx)
         : n == 2 ? dma_request_channel(DmaRequest::usart2_tx)
         : n == 3 ? dma_request_channel(DmaRequest::usart3_tx)
         : n == 4 ? dma_request_channel(DmaRequest::usart4_tx) : uint8_t{0};
}
constexpr uint8_t usart_dma_rx_channel(uint8_t n) {
    return n == 1 ? dma_request_channel(DmaRequest::usart1_rx)
         : n == 2 ? dma_request_channel(DmaRequest::usart2_rx)
         : n == 3 ? dma_request_channel(DmaRequest::usart3_rx)
         : n == 4 ? dma_request_channel(DmaRequest::usart4_rx) : uint8_t{0};
}

} // namespace brio
