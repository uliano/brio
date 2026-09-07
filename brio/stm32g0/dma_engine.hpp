/*
 * dma_engine.hpp
 *
 * The "no DMA engine" tag of every optional engine slot in this stratum
 * - stm32g0/usart.hpp's two, stm32g0/spi.hpp's two - and nothing else.
 *
 * IT IS A TAG, NOT A BASE CLASS: `present` is the only thing a task asks
 * about, and it asks with `if constexpr`, so every engine branch - the
 * pump, the completion path and the state they need - disappears from a
 * driver that does not name one. The proof is the measured kind: the
 * release images of every app that uses no engine are BYTE-IDENTICAL to
 * the ones built before the slots existed (docs/stm32g0/dma.md records
 * that gate).
 *
 * IT LIVES IN A FILE OF ITS OWN AND NOT IN stm32g0/dma.hpp, and the
 * reason is the whole point of an optional slot: a driver with such a
 * slot must not include dma.hpp, or every program with a console (or a
 * bus) would carry the DMA controller. An application that wants an
 * engine includes both headers and names the channel; one that does not
 * never sees the controller at all - which is also why a task reaches
 * its engines only through THEIR OWN published names (start(),
 * service(), flag_complete, flag_error) and never spells a DmaChannel or
 * a DmaFlag.
 *
 * It was usart.hpp's until spi.hpp needed the same tag: two headers
 * cannot each define it, and neither is the other's natural home.
 */

#pragma once

namespace brio {

struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

} // namespace brio
