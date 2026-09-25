/*
 * dma_engine.hpp
 *
 * What a TRANSPORT needs to know about the DMA controllers without
 * including them: the "no engine" tag of every optional engine slot in
 * this stratum, the SLOT a request lands in, and THE REQUEST TABLE.
 *
 * WHY THEY LIVE TOGETHER. On this family the channel IS the request
 * (RM 11.2.3): there is no multiplexer, the tables wire each peripheral
 * event to one channel of one controller, and a transport with an engine
 * slot must be able to REFUSE an engine that names another. It can only
 * do that if it can read the table - and it must not include
 * ch32vx03/dma.hpp for it, because a driver with an empty slot would then
 * drag the whole controller into every program with a console. So the
 * table is here, beside the tag, and it is the ONE place in the stratum
 * where a peripheral's channel number is written: the USARTs', the SPIs',
 * the I2Cs', the converters' and the timers' are all read out of the same
 * rows a reader can hold the manual against.
 *
 * A REQUEST LANDS IN A SLOT, NOT ON A CHANNEL NUMBER. The CH32V303
 * (CH32V30x_D8) carries TWO controllers - DMA1 with seven channels and
 * DMA2 with eleven - and a channel number alone names two different
 * channels there: UART4 transmits on DMA2's fifth, which is not DMA1's
 * fifth. So the table answers a `DmaSlot`, the controller AND the channel,
 * and every check a transport makes compares the whole slot. That is why
 * this is ONE function and not a controller function beside a channel
 * function: two answers invite a check that asks only the second, and a
 * DMA2 engine would then pass for a DMA1 request that happens to share its
 * number - a wedge with no error flag, which is exactly what the check is
 * there to refuse.
 *
 * THE TABLE IS PER PART, not per family. Three device classes read three
 * tables - 11-5 for the CH32V20x_D6, 11-6 for the D8 (the same map plus
 * TIM5's six rows), and for the CH32V30x_D8 table 11-2 for DMA1 (11-5's
 * rows but for UART4's two) with tables 11-3 and 11-4 for DMA2 - and
 * WHICH peripherals the part in front of you offers is parts/<part>.hpp's
 * business: the smallest package has one usart and no I2C at all, the
 * 128 KB CH32V303 has no TIM5 and no TIM8. Such a row answers the empty
 * slot, `channel 0` meaning "no channel" everywhere in this stratum, and
 * `DmaRequestOf<r>` turns that into a compile error on the line that
 * asked.
 *
 * `present` is the only thing a task asks the tag about, with
 * `if constexpr`, so every engine branch disappears from a driver that
 * does not name one.
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/device.hpp"

namespace brio {

/// The empty engine slot.
struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

/**
 * Where a request is served: a controller, 1 or 2, and one of its
 * channels. The empty slot - controller and channel both 0 - is what a
 * request the part cannot raise answers, and what an empty engine slot
 * sits on. A structural type, so a slot can be a template argument.
 */
struct DmaSlot {
    uint8_t controller = 0;
    uint8_t channel = 0;

    constexpr bool present() const { return controller != 0u && channel != 0u; }
    friend constexpr bool operator==(const DmaSlot&, const DmaSlot&) = default;
};

/// The slot an engine sits on - the empty one for the empty tag, which
/// has none - so a transport can check the request table against a named
/// engine without asking the tag for a member it has not got.
template <typename E>
constexpr DmaSlot dma_engine_slot() {
    if constexpr (E::present) {
        return DmaSlot{E::controller, E::channel};
    } else {
        return DmaSlot{};
    }
}

/// Two engines on one transport must not name the same channel of the
/// same controller: a channel moves data ONE way. Generic over any engine
/// that says `present`, `controller` and `channel`.
template <typename Tx, typename Rx>
constexpr bool dma_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return dma_engine_slot<Tx>() != dma_engine_slot<Rx>();
    } else {
        return true;
    }
}

// ---- the request table (RM 11.2.3) ------------------------------------------

/**
 * Every peripheral event the controllers of this family answer, named the
 * way the manual's tables name them - with WCH's USART4..USART8 spelled
 * the way the rest of this stratum spells them, `uart4` to `uart8`,
 * because that is what the parts' pin tables and RCC call them.
 *
 * A timer publishes SEVERAL events and they do not share a channel: an
 * update is not a capture, and a trigger, a commutation and a channel can
 * be three names for one row (they raise the same request).
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
    // The rows only the CH32V30x_D8's second controller answers (tables
    // 11-3 and 11-4): the basic timers, the three advanced timers of the
    // 256 KB parts, the upper serial ports, SPI3, the SDIO host, the two
    // DACs and the second converter.
    tim6_up,
    tim7_up,
    tim8_ch1, tim8_ch2, tim8_ch3, tim8_ch4, tim8_trig, tim8_com, tim8_up,
    tim9_ch1, tim9_ch2, tim9_ch3, tim9_ch4, tim9_trig, tim9_com, tim9_up,
    tim10_ch1, tim10_ch2, tim10_ch3, tim10_ch4, tim10_trig, tim10_com, tim10_up,
    uart5_tx, uart5_rx,
    uart6_tx, uart6_rx,
    uart7_tx, uart7_rx,
    uart8_tx, uart8_rx,
    spi3_rx, spi3_tx,
    sdio,
    dac1, dac2,
    adc2,
};

/// TIM5's requests are DMA1's on a part with one controller and TIM5 (the
/// CH32V203RB, table 11-6), and DMA2's on a part with two (table 11-3).
constexpr bool tim5_on_dma1() { return device::has_tim5 && device::dma_controller_count == 1u; }
constexpr bool tim5_on_dma2() { return device::has_tim5 && device::dma_controller_count >= 2u; }

/// Whether the part has timer `n` at all, read off the part's three masks
/// (parts/<part>.hpp). The DMA table asks this rather than the timer
/// chapter, which it must not include.
constexpr bool dma_timer_present(uint8_t n) {
    const uint32_t all = static_cast<uint32_t>(device::advanced_timer_instances) |
                         static_cast<uint32_t>(device::general_timer_instances) |
                         static_cast<uint32_t>(device::basic_timer_instances);
    return n < 16u && (all & (1UL << n)) != 0u;
}

/// DMA2's rows exist only where the part has the controller.
constexpr bool dma2_present() { return device::dma_controller_count >= 2u; }

/**
 * The slot that serves a request on THIS PART, or the empty slot when the
 * part has not got the peripheral that raises it.
 *
 * DMA1's rows are tables 11-5, 11-6 and 11-2 read column by column: the
 * three are one map, the D8 adding TIM5's six events and the CH32V30x_D8
 * dropping UART4's two (they are DMA2's there). DMA2's rows are tables
 * 11-3 and 11-4, cross-checked cell by cell against the DMA2 figure
 * printed beside them (captioned "Table 11-2 DMA2 request mapping").
 */
constexpr DmaSlot dma_request_channel(DmaRequest r) {
    constexpr DmaSlot none{};
    const auto one = [](bool present, uint8_t ch) { return present ? DmaSlot{1, ch} : DmaSlot{}; };
    const auto two = [](bool present, uint8_t ch) {
        return present && dma2_present() ? DmaSlot{2, ch} : DmaSlot{};
    };
    switch (r) {
        // ADC1 - every part of the family has at least one converter.
        case DmaRequest::adc1:      return one(device::adc_count >= 1u, 1);

        // SPI1 and SPI2. The parts below the CH32V203C8 offer one SPI.
        case DmaRequest::spi1_rx:   return one(device::spi_count >= 1u, 2);
        case DmaRequest::spi1_tx:   return one(device::spi_count >= 1u, 3);
        case DmaRequest::spi2_rx:   return one(device::spi_count >= 2u, 4);
        case DmaRequest::spi2_tx:   return one(device::spi_count >= 2u, 5);

        // The first three serial ports. Which of them a part offers is a
        // mask and not a count (parts/<part>.hpp): the smallest package
        // offers USART2 alone.
        case DmaRequest::usart1_tx: return one(device::has_usart(1), 4);
        case DmaRequest::usart1_rx: return one(device::has_usart(1), 5);
        case DmaRequest::usart2_rx: return one(device::has_usart(2), 6);
        case DmaRequest::usart2_tx: return one(device::has_usart(2), 7);
        case DmaRequest::usart3_tx: return one(device::has_usart(3), 2);
        case DmaRequest::usart3_rx: return one(device::has_usart(3), 3);
        // UART4 is DMA1's where there is no DMA2 (tables 11-5 and 11-6)
        // and DMA2's where there is (table 11-3).
        case DmaRequest::uart4_tx:
            return dma2_present() ? two(device::has_usart(4), 5) : one(device::has_usart(4), 1);
        case DmaRequest::uart4_rx:
            return dma2_present() ? two(device::has_usart(4), 3) : one(device::has_usart(4), 8);

        // I2C1 and I2C2. One package of the family has no I2C at all.
        case DmaRequest::i2c1_tx:   return one(device::i2c_count >= 1u, 6);
        case DmaRequest::i2c1_rx:   return one(device::i2c_count >= 1u, 7);
        case DmaRequest::i2c2_tx:   return one(device::i2c_count >= 2u, 4);
        case DmaRequest::i2c2_rx:   return one(device::i2c_count >= 2u, 5);

        // The first advanced-control timer: one row for its trigger, its
        // commutation and its fourth channel, which share channel 4.
        case DmaRequest::tim1_ch1:  return one(dma_timer_present(1), 2);
        case DmaRequest::tim1_ch2:  return one(dma_timer_present(1), 3);
        case DmaRequest::tim1_ch4:
        case DmaRequest::tim1_trig:
        case DmaRequest::tim1_com:  return one(dma_timer_present(1), 4);
        case DmaRequest::tim1_up:   return one(dma_timer_present(1), 5);
        case DmaRequest::tim1_ch3:  return one(dma_timer_present(1), 6);

        // The three general-purpose timers every part has.
        case DmaRequest::tim2_ch3:  return one(dma_timer_present(2), 1);
        case DmaRequest::tim2_up:   return one(dma_timer_present(2), 2);
        case DmaRequest::tim2_ch1:  return one(dma_timer_present(2), 5);
        case DmaRequest::tim2_ch2:
        case DmaRequest::tim2_ch4:  return one(dma_timer_present(2), 7);

        case DmaRequest::tim3_ch3:  return one(dma_timer_present(3), 2);
        case DmaRequest::tim3_ch4:
        case DmaRequest::tim3_up:   return one(dma_timer_present(3), 3);
        case DmaRequest::tim3_ch1:
        case DmaRequest::tim3_trig: return one(dma_timer_present(3), 6);

        case DmaRequest::tim4_ch1:  return one(dma_timer_present(4), 1);
        case DmaRequest::tim4_ch2:  return one(dma_timer_present(4), 4);
        case DmaRequest::tim4_ch3:  return one(dma_timer_present(4), 5);
        case DmaRequest::tim4_up:   return one(dma_timer_present(4), 7);

        // TIM5: table 11-6's six rows on the CH32V203RB, table 11-3's on
        // the 256 KB CH32V303 - two different maps for the same six events.
        case DmaRequest::tim5_ch2:
            return tim5_on_dma2() ? DmaSlot{2, 4} : one(tim5_on_dma1(), 1);
        case DmaRequest::tim5_ch3:
            return tim5_on_dma2() ? DmaSlot{2, 2} : one(tim5_on_dma1(), 3);
        case DmaRequest::tim5_ch4:
            return tim5_on_dma2() ? DmaSlot{2, 1} : one(tim5_on_dma1(), 6);
        case DmaRequest::tim5_ch1:
            return tim5_on_dma2() ? DmaSlot{2, 5} : one(tim5_on_dma1(), 7);
        case DmaRequest::tim5_trig:
            return tim5_on_dma2() ? DmaSlot{2, 1} : one(tim5_on_dma1(), 7);
        case DmaRequest::tim5_up:
            return tim5_on_dma2() ? DmaSlot{2, 2} : one(tim5_on_dma1(), 8);

        // ---- DMA2 alone (tables 11-3 and 11-4) --------------------------------
        case DmaRequest::tim6_up:   return two(dma_timer_present(6), 3);
        case DmaRequest::tim7_up:   return two(dma_timer_present(7), 4);

        case DmaRequest::tim8_ch3:
        case DmaRequest::tim8_up:   return two(dma_timer_present(8), 1);
        case DmaRequest::tim8_ch4:
        case DmaRequest::tim8_trig:
        case DmaRequest::tim8_com:  return two(dma_timer_present(8), 2);
        case DmaRequest::tim8_ch1:  return two(dma_timer_present(8), 3);
        case DmaRequest::tim8_ch2:  return two(dma_timer_present(8), 5);

        case DmaRequest::tim9_up:   return two(dma_timer_present(9), 6);
        case DmaRequest::tim9_ch1:  return two(dma_timer_present(9), 7);
        case DmaRequest::tim9_ch4:  return two(dma_timer_present(9), 8);
        case DmaRequest::tim9_ch2:  return two(dma_timer_present(9), 9);
        case DmaRequest::tim9_trig:
        case DmaRequest::tim9_com:  return two(dma_timer_present(9), 10);
        case DmaRequest::tim9_ch3:  return two(dma_timer_present(9), 11);

        case DmaRequest::tim10_ch4:  return two(dma_timer_present(10), 6);
        case DmaRequest::tim10_trig:
        case DmaRequest::tim10_com:  return two(dma_timer_present(10), 7);
        case DmaRequest::tim10_ch1:  return two(dma_timer_present(10), 8);
        case DmaRequest::tim10_ch3:  return two(dma_timer_present(10), 9);
        case DmaRequest::tim10_ch2:  return two(dma_timer_present(10), 10);
        case DmaRequest::tim10_up:   return two(dma_timer_present(10), 11);

        case DmaRequest::uart5_rx:  return two(device::has_usart(5), 2);
        case DmaRequest::uart5_tx:  return two(device::has_usart(5), 4);
        case DmaRequest::uart6_tx:  return two(device::has_usart(6), 6);
        case DmaRequest::uart6_rx:  return two(device::has_usart(6), 7);
        case DmaRequest::uart7_tx:  return two(device::has_usart(7), 8);
        case DmaRequest::uart7_rx:  return two(device::has_usart(7), 9);
        case DmaRequest::uart8_tx:  return two(device::has_usart(8), 10);
        case DmaRequest::uart8_rx:  return two(device::has_usart(8), 11);

        // SPI/I2S3: one row a direction, the I2S face included.
        case DmaRequest::spi3_rx:
            return two((device::spi_instances & (1U << 3)) != 0u, 1);
        case DmaRequest::spi3_tx:
            return two((device::spi_instances & (1U << 3)) != 0u, 2);

        case DmaRequest::sdio:      return two(device::has_sdio, 4);
        case DmaRequest::dac1:      return two(device::has_dac, 3);
        case DmaRequest::dac2:      return two(device::has_dac, 4);

        // ADC2 on channel 5: table 11-3's note gives the row to this class
        // "with the penultimate sixth digit of the lot number not being
        // zero", 12.2.7's note 2 says only ADC1 has a DMA request, and the
        // DMA2 figure of RM V2.5 no longer draws it. The row is stated as
        // the table gives it, and the converter asks the die whether its
        // lot raises it (ch32vx03/adc.hpp's dma(): the CH32V303VCT6 does
        // not).
        case DmaRequest::adc2:      return two(device::adc_count >= 2u, 5);
    }
    return none;
}

/// Whether THIS PART can raise that request at all.
constexpr bool dma_request_present(DmaRequest r) { return dma_request_channel(r).present(); }

/**
 * The slot of a request, as compile-time constants that REFUSE a
 * peripheral this part has not got:
 *
 *     using Tx = brio::DmaRequestOf<brio::DmaRequest::usart2_tx>;
 *     using Engine = brio::DmaTxEngine<Tx::controller, Tx::channel>;
 *
 * which is how an application names the request rather than the numbers
 * and still gets the numbers the silicon wired.
 */
template <DmaRequest r>
struct DmaRequestOf {
    static_assert(dma_request_present(r),
                  "brio DMA: this part does not offer the peripheral that raises that request "
                  "(brio/ch32vx03/parts/ has its table)");
    static constexpr DmaSlot slot = dma_request_channel(r);
    static constexpr uint8_t controller = slot.controller;
    static constexpr uint8_t channel = slot.channel;
};

} // namespace brio
