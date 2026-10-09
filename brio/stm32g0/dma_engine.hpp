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
 * Beside the tag, the three pieces of vocabulary an OWNER speaks to its
 * engines without including the controller: the level an engine arms at
 * (DmaPriority, which a transport names for each engine in its slots),
 * which of a channel's events interrupt (DmaIrq, handed to arm()), and
 * whether a buffer is aligned for a beat of a given width
 * (dma_beat_aligned, the test an engine's start() makes and a driver
 * makes first, to choose its pump instead).
 */

#pragma once

#include <stdint.h>

namespace brio {

struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

/**
 * CCR.PL (RM0444 10.6.3): the software half of the arbitration. The
 * hardware half is the channel INDEX and cannot be configured: between
 * equal levels the lower-numbered channel wins (10.4.4). The DMAMUX in
 * front of the controller routes one request line to each channel and
 * ranks nothing (11.4.4).
 *
 * THE LEVEL IS THE PERIPHERAL'S (docs/design/dma.md). An engine whose
 * peripheral OVERRUNS when its channel is starved - a receive ring, an
 * SPI host's receive, a converter's stream - arms at `very_high`; one
 * whose peripheral waits or holds its last value - a transmit, a paced
 * output, an I2C host's receive under its stretched clock - at `high`; a memory-to-
 * memory copy at `low`. THIS ARBITER'S OWN FACT bounds what the copy can
 * do: a memory-to-memory channel is re-arbitrated after every single
 * transfer and, whenever another channel is requesting, the arbiter
 * ALTERNATES and grants that one, "which may be of lower priority than
 * the memory-to-memory channel" (10.4.4) - so one copy never starves a
 * ring, at any level. Two copies alternate with EACH OTHER, and a ring
 * ranked below them waits for the pair - measured: two copies starved
 * USART2's receive ring past a character time on the STM32G0B1RE, and
 * the ring at `very_high` beside the same pair lost nothing
 * (docs/stm32g0/dma.md); the levels are what keeps it first. The engines' defaults are those three by direction
 * (stm32g0/dma.hpp), and every driver arms its engines at its own level,
 * with its chapter's reason beside the call. It lives here, beside the
 * tag, because a transport names its engines' levels and must not
 * include the controller to do it.
 */
enum class DmaPriority : uint8_t {
    low = 0,
    medium = 1,
    high = 2,
    very_high = 3,
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
