/*
 * dma_engine.hpp
 *
 * NoDmaEngine, the empty slot's tag - and THE USARTs' ROWS of the DMA
 * request table (RM 9.2.3), which is all a transport needs to know about
 * the controller before one is driven.
 *
 * The serial transport (usart.hpp) has the two engine slots every stratum
 * with a DMA controller gives it, so a program written against the
 * CH32V203's Uart names the same parameters here. This series has an
 * eight-channel controller whose CHANNEL IS THE REQUEST (9.2.3: each
 * peripheral request is wired to one channel, no multiplexer), and no
 * driver for it in this stratum; so the slots accept the empty tag alone,
 * and a transport refuses anything else at compile time. The table below
 * is the fact a DmaTxEngine or DmaRxEngine will be checked against the
 * day it exists.
 */

#pragma once

#include <stdint.h>

namespace brio {

/// The empty engine slot: `present` is what every engine branch of a
/// transport tests, and the rest of an engine's surface is never asked of
/// it.
struct NoDmaEngine {
    static constexpr bool present = false;
};

/// The channel each USART direction requests on (RM 9.2.3): USART1
/// transmits on 4 and receives on 5, USART2 on 7 and 6, USART3 on 2 and 3,
/// USART4 on 1 and 8. Zero for an instance this series has not got.
constexpr uint8_t usart_dma_tx_channel(uint8_t n) {
    return n == 1 ? 4 : n == 2 ? 7 : n == 3 ? 2 : n == 4 ? 1 : 0;
}
constexpr uint8_t usart_dma_rx_channel(uint8_t n) {
    return n == 1 ? 5 : n == 2 ? 6 : n == 3 ? 3 : n == 4 ? 8 : 0;
}

} // namespace brio
