/*
 * crc.hpp
 *
 * The CH32V203's cyclic redundancy check unit (RM ch. 5): `Crc`, a
 * MONOSTATE resource over the whole chapter - which is three registers
 * and no options at all.
 *
 * WHAT THIS BLOCK IS. A hardware CRC-32 with the Ethernet polynomial
 * 0x04C11DB7 WIRED IN: no polynomial register, no initial-value
 * register, no reversal and no final XOR. What it computes is therefore
 * exactly one function, the one catalogued as CRC-32/MPEG-2 - initial
 * value 0xFFFFFFFF, most significant bit first, no reflection of input
 * or output, no final inversion - and that function is not this
 * family's: `crc32_ethernet()` in util/crc.hpp is the same arithmetic
 * in constexpr C++, shared with every other target whose silicon has
 * this one polynomial and no knob to change it. The two agreeing is
 * what a bench suite measures.
 *
 * FOUR FACTS THAT SHAPE THE FILE.
 *
 * 1. THE DATA REGISTER IS THE OPERATION. One register, CRC_DATAR: a
 *    WRITE feeds a 32-bit word into the calculator, a READ returns the
 *    running result. There is no start bit and no ready flag - 5.2 says
 *    "the hardware calculation interrupts the write operation of the
 *    system, so new values can be written continuously", which is the
 *    same promise the same block makes on its ST relatives, and 5.1
 *    prices it at four HCLK cycles. Back-to-back writes and a read
 *    right after a write are correct by construction, and the cost is
 *    paid by the bus rather than by a poll.
 *
 * 2. A WORD IS THE GRAIN, AND THE PROGRAM DECIDES ITS BYTE ORDER. 5.2:
 *    "the CRC unit calculates the whole 32-bit data word, rather than
 *    byte per byte". A BYTE STREAM therefore has no direct answer here.
 *    `word_be()` packs four bytes most-significant-first, which is the
 *    packing that makes the unit's result equal CRC-32/MPEG-2 over
 *    those bytes in that order; a stream whose length is not a multiple
 *    of four is the caller's to pad, and the padding is part of the
 *    checksum's definition. (The name is the STM32F4 stratum's too: the
 *    byte order into a register is a chapter's fact, so each chapter
 *    states it - and where the two chapters state the same thing, the
 *    two files spell it the same way.)
 *
 * 3. THE RESET BIT IS THE ONLY CONTROL. CRC_CTLR has one bit, RST,
 *    write-only and self-clearing, which puts CRC_DATAR back to
 *    0xFFFFFFFF (5.3.3). There is nothing else to configure, so
 *    `reset()` is what starts every checksum and `init()` is the clock
 *    gate plus that. Whether the reset lands INSTANTLY is the one open
 *    question of this chapter: the same block on an STM32F4 was
 *    measured taking a few cycles, with a read in the next instruction
 *    returning the old value and a word written in the next instruction
 *    SWALLOWED, and neither manual says so. `reset()` therefore waits
 *    for the initial value to appear and RETURNS how many extra reads
 *    that took - zero if this silicon is instant, which is a
 *    measurement rather than an assumption.
 *
 * 4. CRC_IDATAR IS NOT PART OF THE CHECKSUM. Eight bits of scratch that
 *    RST deliberately does not touch (5.2, 5.3.2) - the block's one
 *    piece of state a program can use to remember something across a
 *    checksum. NOTHING SHORT OF A SYSTEM RESET CLEARS IT: this block
 *    has no line in RCC_AHBRSTR, whose bits [11:0] are reserved
 *    (3.4.11), so there is no peripheral reset to offer and no verb
 *    here offers one. Measured: a checksum, a pattern in the scratch,
 *    and the register still holding it afterwards.
 *
 * NO INTERRUPT, NO DMA REQUEST, NO EVENT. The chapter has no interrupt
 * register, the vector table has no CRC line, and tables 11-5 and 11-6
 * map no DMA request to this block - a bulk checksum is a loop of
 * stores and nothing else. What a DMA channel CAN do is memory-to-
 * memory into CRC_DATAR with the destination not incremented; that is
 * the DMA chapter's arrangement and ch32vx03/dma.hpp already has every
 * verb it needs, so nothing is added here for it.
 *
 * PRESENT ON EVERY PART. The block hangs off the HB bus behind
 * RCC_HBPCENR's CRCEN, and neither the reference manual nor the
 * datasheet's table 2-1 makes it a per-part fact, so unlike most of
 * this stratum this file asks the part table nothing.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"
#include "util/crc.hpp"

namespace brio {

// =============================================================================
// The registers (RM 5.3)
// =============================================================================

/// Table 5-1: three registers on the HB bus at 0x4002 3000. IDATAR is
/// eight bits wide and the chapter names it R8_CRC_IDATAR, which is
/// what decides the access width of that one store.
struct CrcRegs {
    volatile uint32_t DATAR;     ///< 0x00 write a word in, read the result out
    volatile uint8_t IDATAR;     ///< 0x04 eight bits of scratch, untouched by RST
    uint8_t RESERVED0[3];
    volatile uint32_t CTLR;      ///< 0x08 one bit: RST
};

inline CrcRegs* crc_regs() { return reinterpret_cast<CrcRegs*>(hb_base + 0x3000); }

/// CRC_CTLR (5.3.3): write-one, self-clearing, and the only control the
/// block has.
inline constexpr uint32_t crc_rst = 1UL << 0;

// =============================================================================
// The packing
// =============================================================================

// THE FUNCTION IS NOT THIS FAMILY'S, so it does not live here:
// crc32_ethernet_poly, crc32_ethernet_init, crc32_ethernet_byte,
// crc32_ethernet_word, crc32_ethernet and crc32_ethernet_bytes are
// util/crc.hpp's, beside the CRC-16 the stored records carry. What
// stays here is the PACKING, which is this chapter's own: how a byte
// stream becomes the words this data register takes.

/// Four bytes as the word that makes the unit's answer the byte
/// stream's: `b0` is the first byte of the stream and the word's most
/// significant.
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
 *   brio::Crc::init();                        // the HB gate, then a reset
 *   for (uint32_t w : words) brio::Crc::feed(w);
 *   const uint32_t sum = brio::Crc::value();
 *
 * NO OWNERSHIP AND NO GUARD. There is one unit and it holds the running
 * result in the same register a second user would write, so two pieces
 * of code that checksum from different contexts corrupt each other's
 * answer with no flag to show for it. This driver does not arbitrate
 * that - there is nothing in the silicon to arbitrate WITH - so a
 * program that checksums from an interrupt as well as from the loop
 * owns the arrangement (a critical section around the run, or the unit
 * reserved for one context).
 */
struct Crc {
    Crc() = delete;

    static CrcRegs& regs() { return *crc_regs(); }

    /// RCC_HBPCENR.CRCEN, clear at reset. Every verb below needs it -
    /// but what the gate holds is the WRITE path alone: with it shut
    /// the block keeps its state and its registers still ANSWER a read,
    /// while a store into them is dropped in silence (measured, and the
    /// same shape the window watchdog's gate has on this family).
    static void clock(bool on) {
        if (on) {
            Rcc::enable(Bus::hb, rcc_hb_crc);
        } else {
            Rcc::disable(Bus::hb, rcc_hb_crc);
        }
    }
    static bool clock() { return Rcc::enabled(Bus::hb, rcc_hb_crc); }

    /// The gate open and the calculator at its initial value - what a
    /// program calls once before its first checksum. Answers what
    /// `reset()` answered, so a caller that cares about the reset's
    /// latency gets it here too.
    static uint8_t init() {
        clock(true);
        return reset();
    }

    /// How many extra reads of CRC_DATAR `reset()` spends waiting for
    /// the reset to land before it gives up. A BOUND and not an
    /// expectation.
    static constexpr uint8_t reset_spins = 8;

    /**
     * CRC_CTLR.RST (5.3.3): CRC_DATAR back to 0xFFFFFFFF. The bit is
     * write-only and clears itself, so the store is a plain one and
     * never a read-modify-write, and CRC_IDATAR is deliberately NOT
     * touched.
     *
     * The verb waits, by reading CRC_DATAR until it shows the initial
     * value, and RETURNS the number of extra reads that took (see fact
     * 3 in the file header): zero means the reset landed before the
     * next instruction could look.
     */
    static uint8_t reset() {
        regs().CTLR = crc_rst;
        uint8_t spins = 0;
        while (regs().DATAR != crc32_ethernet_init) {
            if (++spins >= reset_spins) {
                break;
            }
        }
        return spins;
    }

    // ---- the calculation (5.2) ----------------------------------------------

    /// One 32-bit word into the calculator. The store stalls until the
    /// computation is done, so there is nothing to wait for and nothing
    /// to poll - the next store or the next read is already correct.
    static void feed(uint32_t word) { regs().DATAR = word; }

    /// A run of words, in order. `count == 0` leaves the running value
    /// alone, which is what makes an empty message's checksum the
    /// initial value.
    static void feed(const uint32_t* words, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            regs().DATAR = words[i];
        }
    }

    /// Four bytes as one word, most significant first (see `word_be`).
    static void feed_bytes(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
        regs().DATAR = word_be(b0, b1, b2, b3);
    }

    /// The running result. Reading does not consume it: the next
    /// `feed()` continues from here, which is what lets a checksum be
    /// taken in pieces.
    static uint32_t value() { return regs().DATAR; }

    /// A whole checksum in one call: reset, feed, read.
    static uint32_t compute(const uint32_t* words, size_t count) {
        (void)reset();
        feed(words, count);
        return value();
    }

    /**
     * A whole checksum over a BYTE array whose length is known at
     * compile time - the byte stream packed into the words this
     * register takes, four at a time, most significant first.
     *
     * A length that is not a whole number of words is a COMPILE ERROR
     * and not a rounded answer: this unit has no byte grain (fact 2),
     * so what such a call would compute is not the checksum of what was
     * asked. util/crc.hpp's `crc32_ethernet_bytes()` is the software
     * answer for any other length.
     */
    template <size_t N>
    static uint32_t compute_bytes(const uint8_t (&bytes)[N]) {
        static_assert(N % 4u == 0u,
                      "brio Crc: this unit computes on a whole 32-bit word and has no byte "
                      "grain (RM 5.2), so a byte run must be a multiple of four - pad it, and "
                      "the padding is part of the checksum's definition, or use "
                      "util/crc.hpp's crc32_ethernet_bytes() in software");
        (void)reset();
        for (size_t i = 0; i < N; i += 4u) {
            feed_bytes(bytes[i], bytes[i + 1u], bytes[i + 2u], bytes[i + 3u]);
        }
        return value();
    }

    // ---- the scratch register (5.3.2) ---------------------------------------

    /// CRC_IDATAR, eight bits of general-purpose storage in the
    /// peripheral. It survives `reset()` by design and every word fed
    /// through the calculator, and only a system reset clears it (fact
    /// 4). The chapter names the register R8, so the access is a byte
    /// store.
    static void scratch(uint8_t v) { regs().IDATAR = v; }
    static uint8_t scratch() { return regs().IDATAR; }
};

} // namespace brio
