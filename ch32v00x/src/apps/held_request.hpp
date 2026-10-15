// held_request.hpp - the suites' shared staging of a HELD DMA request on the
// CH32V00x: a request raised with no channel enabled to take it and its
// requester's gate closed without a reset, and the next owner of the
// channel counted.
//
// A DMA channel's requests are an OR (RM 8.2.1, table 8-2), and on this die
// a raised request is HELD until the channel takes it or its peripheral's
// reset line is pulsed: the next owner of the channel takes one stray item
// and the channel then serves nothing more while the request stands - a
// DEAD CHANNEL (docs/ch32v00x/dma.md). The timers' compare requests reach
// every channel (table 8-2), so a dead channel is staged with no wire: TIM1's
// channel 3 on DMA channel 6, TIM2's channel 2 on DMA channel 7.
//
// Test code, not a driver: it writes a timer's registers directly, as the
// suites that include it do.

#pragma once

#include <stdint.h>

#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/tim.hpp"

namespace held {

/// A compare request RAISED AND HELD: CCxDE set and the counter run from
/// zero at HCLK past one match of the channel's compare (10 counts in, a
/// period of 1000: some fifty microseconds, two matches), stopped, and the
/// gate closed with no reset. `ch` is the channel's index, 0..3. (On the
/// CH32X035 the software compare event, SWEVGR's CCxG, left the next owner
/// nothing: the match is the request.)
template <class T, typename Clk>
void hold_compare_request(uint8_t ch, Clk clock) {
    T::bus_clock(true);
    brio::TimRegs& r = T::regs();
    r.PSC = 0;
    r.ATRLR = 999;
    r.CHCVR[ch] = 10;
    r.CNT = 0;
    r.DMAINTENR = static_cast<uint16_t>(1u << (9u + ch));
    r.CTLR1 = brio::tim_cen;
    (void)brio::delay_us(clock, 50);
    r.CTLR1 = 0;
    T::bus_clock(false);
}

/// The release contract on the timer: the reset line, then the gate.
template <class T>
void free_compare_request() {
    T::bus_clock(true);
    T::reset();
    T::bus_clock(false);
}

/// The NEXT OWNER of a channel: a transmit block of four bytes into a RAM
/// cell, request-paced (MEM2MEM clear), asked by nothing - any item it moves
/// is a request still standing on the channel's OR. Its count read 300 us
/// after the enable (a held request is taken within a few bus cycles).
inline volatile uint8_t sink_cell = 0;
template <uint8_t ch, typename Clk>
uint16_t next_owner_moves(Clk clock) {
    static const uint8_t four[4] = {1, 2, 3, 4};
    using C = brio::DmaChannel<ch>;
    C::stop();
    (void)C::prepare(brio::DmaTransfer{.peripheral = &sink_cell, .memory = const_cast<uint8_t*>(four), .count = 4,
                                       .config = {.direction = brio::DmaDirection::memory_to_peripheral,
                                                  .memory_increment = true, .priority = brio::DmaPriority::high}});
    C::enable(true);
    (void)brio::delay_us(clock, 300);
    const uint16_t moved = static_cast<uint16_t>(4u - C::count());
    C::stop();
    return moved;
}

}  // namespace held
