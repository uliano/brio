/*
 * dma_engine.hpp
 *
 * The "no DMA engine" tag of every optional engine slot in this stratum
 * - stm32g0/usart.hpp's two, stm32g0/spi.hpp's two - and nothing else.
 *
 * IT IS A TAG, NOT A BASE CLASS: `present` is the only thing a task asks
 * about, and it asks with `if constexpr`, so every engine branch - the
 * pump, the completion path and the state they need - disappears from a
 * driver that does not name one: an image whose tasks all take the
 * default carries no engine code at all.
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
 * And it is a file rather than a member of one of the two drivers
 * because two headers cannot each define the same tag, and neither is
 * the other's natural home.
 *
 * Beside the tag, the two pieces of vocabulary an OWNER speaks to its
 * engines without including the controller: which of a channel's events
 * interrupt (DmaIrq, handed to arm()), and whether a buffer is aligned
 * for a beat of a given width (dma_beat_aligned, the test an engine's
 * start() makes and a driver makes first, to choose its pump instead).
 */

#pragma once

#include <stdint.h>

namespace brio {

struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

/**
 * Which of a channel's events an engine takes an interrupt on (RM0444
 * 10.5: TCIE and TEIE). A TRANSFER ERROR IS ALWAYS ARMED: it disables the
 * channel in hardware (10.4.7), and its interrupt is the only way the
 * owner learns that a block died. The COMPLETION is armed unless the
 * owner holds another edge that proves it - the SPI host's transmit
 * block, whose completion the receive block's proves (every frame that
 * came back was clocked out first), or an I2C tenure's, which its own
 * STOP proves - so a transaction costs one interrupt, not one a channel.
 */
enum class DmaIrq : uint8_t {
    complete_and_error = 0,
    error_only = 1,
};

/// Is `p` aligned for a beat of `bytes` (1, 2 or 4)? 10.4.3's note: "the
/// AHB master bus source/destination address must be aligned with the
/// programmed size of the transferred single data". A null pointer is
/// aligned - the test is about addresses, the null is the caller's.
inline bool dma_beat_aligned(const volatile void* p, uint8_t bytes) {
    return (reinterpret_cast<uintptr_t>(p) & static_cast<uintptr_t>(bytes - 1u)) == 0u;
}

} // namespace brio
