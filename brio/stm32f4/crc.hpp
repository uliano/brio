/*
 * crc.hpp
 *
 * The STM32F4's CRC calculation unit (RM0090 ch. 4, RM0390 ch. 4, RM0383
 * ch. 3): `Crc`, a MONOSTATE resource over the whole chapter - which is
 * three registers and no options at all.
 *
 * WHAT THIS BLOCK IS. A hardware CRC-32 with the Ethernet polynomial
 * 0x04C11DB7 WIRED IN: no polynomial register, no initial-value register,
 * no reversal and no final XOR - the four knobs the L4's and F7's later
 * version of this peripheral grew. What it computes is therefore exactly
 * one function, the one catalogued as CRC-32/MPEG-2: initial value
 * 0xFFFFFFFF, most significant bit first, no reflection of input or
 * output, no final inversion. `crc32_ethernet()` below is that same
 * function in constexpr C++, and the two agreeing is what the bench suite
 * measures.
 *
 * FIVE FACTS THAT SHAPE THE FILE.
 *
 * 1. THE DATA REGISTER IS THE OPERATION. One register, CRC_DR: a WRITE
 *    feeds a 32-bit word into the calculator, a READ returns the running
 *    result. There is no start bit and no ready flag - "the write
 *    operation is stalled until the end of the CRC computation" (4.3), so
 *    back-to-back writes and a read right after a write are correct by
 *    construction, and the four AHB cycles the computation takes are paid
 *    by the bus and not by a poll.
 *
 * 2. A WORD IS THE GRAIN, AND THE PROGRAM DECIDES ITS BYTE ORDER. 4.3:
 *    "CRC computation is done on the whole 32-bit data word, and not byte
 *    per byte", and 4.4 adds that the registers "have to be accessed by
 *    words (32 bits)" - this version of the block has no byte or half-word
 *    access, so a BYTE STREAM has no direct answer here. `word_be()` packs
 *    four bytes most-significant-first, which is the packing that makes
 *    the unit's result equal CRC-32/MPEG-2 over those bytes in that order;
 *    a stream whose length is not a multiple of four is the caller's to
 *    pad, and the padding is part of the checksum's definition.
 *
 * 3. THE RESET BIT IS THE ONLY CONTROL - AND IT IS NOT INSTANT. CRC_CR has
 *    one bit, RESET, which is write-only and self-clearing and puts CRC_DR
 *    back to 0xFFFFFFFF (4.4.3). There is nothing else to configure, so
 *    `reset()` is what starts every checksum and `init()` is the clock gate
 *    plus that. What the chapter does not say, and the bench measured, is
 *    that the reset takes effect a few cycles AFTER the store: a read of
 *    CRC_DR in the next instruction returns the old running value, and a
 *    word written in the next instruction is SWALLOWED. `reset()` therefore
 *    waits for the initial value to appear before it returns.
 *
 * 4. CRC_IDR IS NOT PART OF THE CHECKSUM. Eight bits of scratch that the
 *    RESET bit deliberately does not touch (4.3, 4.4.2) - the block's one
 *    piece of state a program can use to remember something across a
 *    checksum. It is cleared by the peripheral's RCC reset line and by
 *    nothing else, which is the difference between `reset()` and
 *    `reset_block()`.
 *
 * 5. NO INTERRUPT, NO DMA REQUEST, NO EVENT. The chapter has no IER, the
 *    part has no CRC vector and no DMA request line is mapped to this
 *    block on any part of the family - a bulk checksum is a loop of stores
 *    and nothing else. What a DMA stream CAN do is memory-to-memory into
 *    CRC_DR with the destination not incremented; that is the DMA
 *    chapter's arrangement and stm32f4/dma.hpp already has every verb it
 *    needs, so nothing is added here for it.
 *
 * ERRATA. No item of ES0206 Rev 24, ES0298 Rev 8 or ES0287 Rev 6 is filed
 * against the CRC calculation unit. (The three sheets' "wrong CRC
 * calculation when the polynomial is even" is the SPI's own CRC unit,
 * stm32f4/spi.hpp's, which is a different block with a programmable
 * polynomial.)
 *
 * PRESENT ON EVERY PART. All twenty-three device headers of the pack
 * declare CRC_BASE and RCC_AHB1ENR_CRCEN, so unlike the RNG next door
 * this file has no absence to handle and nothing in the reserve.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"

namespace brio {

// =============================================================================
// The polynomial, in software
// =============================================================================

/// The generator this block has wired in (4.2): x^32 + x^26 + x^23 + x^22
/// + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1.
inline constexpr uint32_t crc32_ethernet_poly = 0x04C11DB7u;

/// What CRC_DR holds after a reset, and the initial value every software
/// computation below starts from (4.4.1's reset value).
inline constexpr uint32_t crc32_ethernet_init = 0xFFFFFFFFu;

/**
 * One byte into a running CRC-32, most significant bit first, no
 * reflection - the primitive the hardware's word-wise operation is built
 * out of. Bitwise and table-free: eight shifts a byte, which is what the
 * kernel's crc16 does too (util/crc.hpp) and for the same reason - a
 * 1 KB table would cost more than it saves in a program that has a
 * hardware unit for the bulk of it.
 */
constexpr uint32_t crc32_ethernet_byte(uint32_t crc, uint8_t byte) {
    crc ^= static_cast<uint32_t>(byte) << 24;
    for (uint8_t i = 0; i < 8; ++i) {
        crc = (crc & 0x80000000u) != 0u ? ((crc << 1) ^ crc32_ethernet_poly) : (crc << 1);
    }
    return crc;
}

/// One 32-bit word into a running CRC, in the order the silicon takes it:
/// the word's most significant byte first. Feeding `w` here and writing
/// `w` to CRC_DR are the same operation.
constexpr uint32_t crc32_ethernet_word(uint32_t crc, uint32_t word) {
    crc = crc32_ethernet_byte(crc, static_cast<uint8_t>(word >> 24));
    crc = crc32_ethernet_byte(crc, static_cast<uint8_t>(word >> 16));
    crc = crc32_ethernet_byte(crc, static_cast<uint8_t>(word >> 8));
    return crc32_ethernet_byte(crc, static_cast<uint8_t>(word));
}

/// A run of words from the initial value - the whole checksum in software,
/// for a constant known at compile time, for a program whose block is busy
/// with something else, and for the bench suite to judge the silicon
/// against.
constexpr uint32_t crc32_ethernet(const uint32_t* words, size_t count) {
    uint32_t crc = crc32_ethernet_init;
    for (size_t i = 0; i < count; ++i) {
        crc = crc32_ethernet_word(crc, words[i]);
    }
    return crc;
}

/// A run of BYTES in software, for the length this unit cannot take
/// (anything that is not a whole number of words) and for the published
/// check value the family fixture pins the implementation to.
constexpr uint32_t crc32_ethernet_bytes(const uint8_t* bytes, size_t count) {
    uint32_t crc = crc32_ethernet_init;
    for (size_t i = 0; i < count; ++i) {
        crc = crc32_ethernet_byte(crc, bytes[i]);
    }
    return crc;
}

/// Four bytes as the word that makes the unit's answer the byte stream's:
/// `b0` is the first byte of the stream and the word's most significant.
constexpr uint32_t word_be(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    return (static_cast<uint32_t>(b0) << 24) | (static_cast<uint32_t>(b1) << 16) |
           (static_cast<uint32_t>(b2) << 8) | static_cast<uint32_t>(b3);
}

// =============================================================================
// The block
// =============================================================================

/**
 * The CRC calculation unit as a monostate resource.
 *
 *   brio::Crc::init();                       // the AHB1 gate, then a reset
 *   for (uint32_t w : words) brio::Crc::feed(w);
 *   const uint32_t sum = brio::Crc::value();
 *
 * NO OWNERSHIP AND NO GUARD. There is one unit and it holds the running
 * result in the same register a second user would write, so two pieces of
 * code that checksum from different contexts corrupt each other's answer
 * with no flag to show for it. This driver does not arbitrate that - there
 * is nothing in the silicon to arbitrate WITH - so a program that
 * checksums from an interrupt as well as from the loop owns the
 * arrangement (a critical section around the run, or the unit reserved for
 * one context).
 */
struct Crc {
    Crc() = delete;

    static CRC_TypeDef& regs() { return *CRC; }

    /// RCC_AHB1ENR.CRCEN. Every verb below needs it: with the gate shut a
    /// write to CRC_DR is dropped and a read answers zero.
    static void clock(bool on) { Rcc::ahb1_clock(RCC_AHB1ENR_CRCEN, on); }
    static bool clock() { return Rcc::ahb1_clock(RCC_AHB1ENR_CRCEN); }

    /// The gate open and the calculator at its initial value - what a
    /// program calls once before its first checksum.
    static void init() {
        clock(true);
        (void)reset();
    }

    /// How many extra reads of CRC_DR `reset()` will spend waiting for the
    /// reset to land before it gives up. A bound and not an expectation:
    /// the measured cost is under it by a wide margin.
    static constexpr uint8_t reset_spins = 8;

    /**
     * CRC_CR.RESET (4.4.3): CRC_DR back to 0xFFFFFFFF. The bit is
     * write-only and clears itself, so the store is a plain one and never a
     * read-modify-write, and CRC_IDR is deliberately NOT touched.
     *
     * THE RESET IS NOT INSTANT, AND THE CHAPTER DOES NOT SAY SO. Measured:
     * a read of CRC_DR in the instruction after the store still returns the
     * OLD running value, and a WORD FED in the instruction after the store
     * is swallowed - the calculator is busy resetting and the write goes
     * nowhere, with no flag to say it did. (4.3's "the write operation is
     * stalled until the end of the CRC computation" covers a computation
     * and not this.) So the verb waits, by reading CRC_DR until it shows
     * the initial value, and RETURNS the number of extra reads that took:
     * a measurement the bench suite prints and a caller can ignore.
     */
    static uint8_t reset() {
        regs().CR = CRC_CR_RESET;
        uint8_t spins = 0;
        while (regs().DR != crc32_ethernet_init) {
            if (++spins >= reset_spins) {
                break;
            }
        }
        return spins;
    }

    /// The peripheral's RCC reset line: the whole block to its reset state,
    /// CRC_IDR included, which `reset()` does not do.
    static void reset_block() { Rcc::ahb1_reset(RCC_AHB1RSTR_CRCRST); }

    // ---- the calculation (4.3) -----------------------------------------------

    /// One 32-bit word into the calculator. The store stalls until the
    /// computation is done, so there is nothing to wait for and nothing to
    /// poll - the next store or the next read is already correct.
    static void feed(uint32_t word) { regs().DR = word; }

    /// A run of words, in order. `count == 0` leaves the running value
    /// alone, which is what makes an empty message's checksum the initial
    /// value.
    static void feed(const uint32_t* words, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            regs().DR = words[i];
        }
    }

    /// Four bytes as one word, most significant first (see `word_be`).
    static void feed_bytes(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
        regs().DR = word_be(b0, b1, b2, b3);
    }

    /// The running result. Reading does not consume it: the next `feed()`
    /// continues from here, which is what lets a checksum be taken in
    /// pieces.
    static uint32_t value() { return regs().DR; }

    /// A whole checksum in one call: reset, feed, read.
    static uint32_t compute(const uint32_t* words, size_t count) {
        (void)reset();
        feed(words, count);
        return value();
    }

    // ---- the scratch register (4.4.2) -----------------------------------------

    /// CRC_IDR, eight bits of general-purpose storage in the peripheral.
    /// It survives `reset()` by design and dies with `reset_block()`.
    ///
    /// THE ACCESS WIDTH IS THE HEADER'S AND NOT THE MANUAL'S. 4.4 says the
    /// CRC registers "have to be accessed by words (32 bits)", but ST's own
    /// device header declares this one `__IO uint8_t` at offset 0x04 with
    /// three reserved bytes after it - so the header asks for a BYTE store
    /// where the manual asks for a word. The header wins in code (it is
    /// what every ST driver compiles to) and the bench suite is what
    /// settles it: `test_stm32f4_misc` writes a byte and reads it back.
    static void idr(uint8_t v) { regs().IDR = v; }
    static uint8_t idr() { return regs().IDR; }
};

} // namespace brio
