/*
 * sha256.hpp
 *
 * The SHA-256 accelerator (datasheet 12.13): `Sha256`, a MONOSTATE - one
 * instance, ten registers, no configuration worth the name. A block the
 * RP2040 never had.
 *
 * WHAT IT IS. The compression function of FIPS 180-4 in hardware and
 * NOTHING ELSE. There is no message length register, no padding, no
 * multi-context state to save: a write to CSR.START loads the eight
 * initial words, every sixteen words written to WDATA are digested as
 * one 512-bit block, and SUM0..SUM7 hold the state after the last block
 * that completed. So the block computes exactly one function, and what
 * that function IS lives in util/sha256.hpp - the constexpr twin this
 * file's bench suite judges the silicon against, and the half of the job
 * the hardware does not do.
 *
 * THE PADDING IS SOFTWARE'S (12.13.1). The standard's trailing one bit,
 * the zeros and the 64-bit length are bytes like any others here, and
 * `hash()` builds them with util/sha256.hpp's `sha256_tail()` - the same
 * arithmetic the software path uses, so the two cannot drift apart.
 *
 * BSWAP, AND WHY EVERY WORD THIS DRIVER WRITES IS LOADED LITTLE-ENDIAN
 * (12.13.3). The bus interface assembles byte and halfword writes into a
 * message word in LITTLE-endian order, so that the same buffer DMAed at
 * any transfer width gives the same answer on a little-endian machine;
 * but SHA-256 wants the first byte of the message in the MOST
 * significant lane. CSR.BSWAP, set at reset, swaps the assembled word on
 * its way into the core and reconciles the two. This driver leaves it
 * set and loads each word from the message little-endian, which makes a
 * word write and four byte writes the same operation - and makes a
 * driver that never has to think about the machine's own endianness.
 *
 * THE HANDSHAKE, AND THE FLAG THAT CATCHES A MISSED ONE. After sixteen
 * words the core is busy for 57 cycles: CSR.WDATA_RDY goes low and a
 * write during that window is DROPPED, with CSR.ERR_WDATA_NOT_RDY left
 * standing to say so. Every write here polls first, and the error flag
 * is a first-class verb because a suite that does not read it cannot
 * tell a lost word from a wrong answer.
 *
 * THROUGHPUT (12.13.2). Sixty-four cycles to write a block over the APB
 * (four cycles a word) plus 57 to digest it: one block per 121 system
 * clock cycles at best, 0.53 bytes a cycle, 79.3 MB/s at 150 MHz - and
 * that is the DMA's figure. A polled loop like this one pays a status
 * read per word on top, so the suite measures what the processor
 * actually gets rather than repeating the ceiling.
 *
 * DMA (12.13.4). The block raises `Dreq::sha256`, and the request is for
 * a WHOLE BLOCK at a time - there is no FIFO, the data goes straight
 * into the message schedule - so CSR.DMA_SIZE must match the channel's
 * transfer width and the channel's count must be a multiple of sixteen
 * words. `dma_size()` is here and no engine is: a channel that feeds
 * this block is the caller's, and the accounting that would make it a
 * BlockPlayer belongs to whoever needs it.
 *
 * ONE SUM AT A TIME. There is one block and no context save, so two
 * owners cannot interleave messages through it (5.4.4 says the same of
 * the bootrom's own use, and gives the boot locks as the mechanism for
 * agreeing about it). This driver states the rule and enforces nothing:
 * a lock is a policy, and brio's programs have one owner.
 *
 * AND WHY NO ATOMIC ALIAS TOUCHES CSR. One word holds four kinds of bit:
 * two ordinary read-write fields, a SELF-CLEARING one (START), a
 * WRITE-ONE-TO-CLEAR one (ERR_WDATA_NOT_RDY) and two read-only statuses.
 * The set alias of 2.1.3 ORs a bit into the register's contents, which
 * would SET a write-one-to-clear flag rather than clear it; and a plain
 * read-modify-write would carry a standing error flag back as a one and
 * clear it in silence. So every store here is the two CONFIGURATION
 * fields as they read, plus the one bit being asked for - the error flag
 * is cleared when a caller asks and never as a side effect. One owner
 * makes the read-modify-write safe; the file says so above.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <optional>
#include <span>

#include "rp2350/device.hpp"
#include "rp2350/dma_engine.hpp"
#include "rp2350/resets.hpp"
#include "util/sha256.hpp"

namespace brio {

/// CSR.DMA_SIZE: the transfer width a DMA channel feeding WDATA uses, so
/// that the block asks for the right NUMBER of transfers per 512-bit
/// block (sixteen words, thirty-two halfwords, sixty-four bytes).
enum class Sha256DmaSize : uint8_t {
    bytes = 0,
    halfwords = 1,
    words = 2,
};

/// What one attempt to take a digest found, when it found none.
enum class Sha256Error : uint8_t {
    not_ready,    ///< WDATA_RDY never rose: the core is not clocked, or is held in reset
    no_sum,       ///< SUM_VLD never rose after a whole number of blocks
    write_lost,   ///< ERR_WDATA_NOT_RDY: a word was written into a busy core and dropped
};

/**
 * The accelerator as a monostate resource.
 *
 *   brio::Sha256::init();
 *   auto d = brio::Sha256::hash(message);         // padding included
 *   if (d && *d == brio::sha256(message)) { ... } // the twin agrees
 *
 * `hash()` is the whole job. Below it sit the verbs a program that feeds
 * the block from somewhere else needs - `start()`, `write_word()`,
 * `write_block()`, `digest()` - in the order 12.13 lists them.
 */
struct Sha256 {
    Sha256() = delete;

    static SHA256_Type& regs() { return *SHA256; }

    /// The reset controller's bit for this block (7.5): it comes up HELD,
    /// like every peripheral here.
    static constexpr uint32_t reset_block = ResetBlock::sha256;

    /// What a DMA channel feeding WDATA names as its pacing request.
    static constexpr Dreq dreq = Dreq::sha256;

    /// How many spins a status poll is given before it gives up. A block
    /// takes 121 system clock cycles at worst, so anything beyond this is
    /// a block that is not running at all.
    static constexpr uint32_t default_spins = 100'000u;

    // ---- bring-up ------------------------------------------------------------

    /**
     * Release the block from reset and put the state machine at the
     * start of a message: BSWAP set (so a word loaded little-endian from
     * a byte buffer is the standard's big-endian message word),
     * DMA_SIZE at words, the write-error flag cleared.
     *
     * `Resets::cycle()` and not `release()`: on this chip a previous
     * life is the ordinary case - a debugger's reset request resets the
     * cores alone - so a block is started from its reset state and never
     * from whatever the last image left in it.
     */
    static bool init() {
        if (!Resets::cycle(reset_block)) {
            return false;
        }
        byte_swap(true);
        dma_size(Sha256DmaSize::words);
        clear_write_error();
        start();
        return wdata_ready();
    }

    /// Everything off: the block back into reset, which is the only way
    /// to make its state unreadable.
    static void release() { Resets::hold(reset_block); }

    // ---- the control and status register (12.13.5) ---------------------------

    /// The two fields of CSR a store must preserve; everything else in
    /// the word is read-only, self-clearing or write-one-to-clear.
    static constexpr uint32_t csr_config_bits =
        SHA256_CSR_BSWAP_BITS | SHA256_CSR_DMA_SIZE_BITS;

    /// CSR.START: load the eight initial words and clear the internal
    /// counters. Forces WDATA_RDY and SUM_VLD high at once, so a digest
    /// taken before any data is written is the INITIAL STATE and not a
    /// message's - which is why `hash()` always writes at least the
    /// padding block.
    static void start() { store(SHA256_CSR_START_BITS); }

    /// CSR.BSWAP. Set at reset and left set by `init()`; a program with
    /// message words already in big-endian order clears it.
    static void byte_swap(bool on) {
        regs().CSR = (regs().CSR & csr_config_bits & ~static_cast<uint32_t>(SHA256_CSR_BSWAP_BITS)) |
                     (on ? static_cast<uint32_t>(SHA256_CSR_BSWAP_BITS) : 0u);
    }
    static bool byte_swap() { return (regs().CSR & SHA256_CSR_BSWAP_BITS) != 0u; }

    /// CSR.DMA_SIZE, which must be written BEFORE a channel is triggered.
    static void dma_size(Sha256DmaSize size) {
        regs().CSR =
            (regs().CSR & csr_config_bits & ~static_cast<uint32_t>(SHA256_CSR_DMA_SIZE_BITS)) |
            (static_cast<uint32_t>(size) << SHA256_CSR_DMA_SIZE_LSB);
    }
    static Sha256DmaSize dma_size() {
        return static_cast<Sha256DmaSize>((regs().CSR & SHA256_CSR_DMA_SIZE_BITS) >>
                                          SHA256_CSR_DMA_SIZE_LSB);
    }

    /// CSR.WDATA_RDY: the core will take another word. Low for 57 cycles
    /// after every sixteenth.
    static bool wdata_ready() { return (regs().CSR & SHA256_CSR_WDATA_RDY_BITS) != 0u; }

    /// CSR.SUM_VLD: SUM0..SUM7 hold the state after a completed block.
    /// Goes low at the first write of a block and high again when that
    /// block's digest is done.
    static bool sum_valid() { return (regs().CSR & SHA256_CSR_SUM_VLD_BITS) != 0u; }

    /// CSR.ERR_WDATA_NOT_RDY: a word was written while the core was busy
    /// and the core did not take it. Write-one-to-clear.
    static bool write_error() { return (regs().CSR & SHA256_CSR_ERR_WDATA_NOT_RDY_BITS) != 0u; }
    static void clear_write_error() { store(SHA256_CSR_ERR_WDATA_NOT_RDY_BITS); }

    // ---- feeding the core ----------------------------------------------------

    /// One message word, with the handshake: poll WDATA_RDY, then store.
    /// False when the core never became ready.
    static bool write_word(uint32_t word, uint32_t spins = default_spins) {
        for (uint32_t i = 0; i < spins; ++i) {
            if (wdata_ready()) {
                regs().WDATA = word;
                return true;
            }
        }
        return false;
    }

    /// One word with no handshake, for a caller that has just written
    /// fewer than sixteen since the last wait and knows the core cannot
    /// be busy. Mixing this with `write_word()` is fine; mixing WIDTHS
    /// inside one block is not (12.13.3), and this driver only ever
    /// writes words.
    static void write_word_unchecked(uint32_t word) { regs().WDATA = word; }

    /**
     * One 512-bit block of the message, as 64 bytes. Each word is loaded
     * LITTLE-endian from the bytes, which with BSWAP set is the
     * standard's big-endian message word - see the file header. False
     * when the core stopped taking data, or when `block` is not a whole
     * block.
     */
    static bool write_block(std::span<const uint8_t> block, uint32_t spins = default_spins) {
        if (block.size() != sha256_block_bytes) {
            return false;
        }
        for (size_t i = 0; i < sha256_block_bytes; i += 4) {
            const uint32_t word = static_cast<uint32_t>(block[i]) |
                                  (static_cast<uint32_t>(block[i + 1]) << 8) |
                                  (static_cast<uint32_t>(block[i + 2]) << 16) |
                                  (static_cast<uint32_t>(block[i + 3]) << 24);
            if (!write_word(word, spins)) {
                return false;
            }
        }
        return true;
    }

    // ---- taking the result ---------------------------------------------------

    /**
     * The state after the last completed block: SUM0..SUM7 read as H0..H7
     * once SUM_VLD stands. Nothing when it never rose within the budget,
     * or when a word was dropped along the way - a digest taken over a
     * message the core did not fully receive is worse than none.
     */
    static std::optional<Sha256Digest> digest(uint32_t spins = default_spins) {
        for (uint32_t i = 0; i < spins; ++i) {
            if (sum_valid()) {
                if (write_error()) {
                    last_error_ = Sha256Error::write_lost;
                    return std::nullopt;
                }
                Sha256Digest d{};
                d.words[0] = regs().SUM0;
                d.words[1] = regs().SUM1;
                d.words[2] = regs().SUM2;
                d.words[3] = regs().SUM3;
                d.words[4] = regs().SUM4;
                d.words[5] = regs().SUM5;
                d.words[6] = regs().SUM6;
                d.words[7] = regs().SUM7;
                return d;
            }
        }
        last_error_ = Sha256Error::no_sum;
        return std::nullopt;
    }

    /**
     * THE WHOLE FUNCTION over a byte range: start, every whole block of
     * the message, the padding of 12.13.1 built in software, the digest.
     * What `brio::sha256()` computes, computed by the silicon.
     *
     * Nothing when the core stopped answering; `last_error()` says which
     * way. An empty message is legal and gives the standard's own
     * answer, because the padding block is still a block.
     */
    static std::optional<Sha256Digest> hash(std::span<const uint8_t> message,
                                            uint32_t spins = default_spins) {
        clear_write_error();
        start();
        size_t offset = 0;
        while (message.size() - offset >= sha256_block_bytes) {
            if (!write_block(message.subspan(offset, sha256_block_bytes), spins)) {
                last_error_ = Sha256Error::not_ready;
                return std::nullopt;
            }
            offset += sha256_block_bytes;
        }
        std::array<uint8_t, sha256_max_tail_bytes> tail{};
        const size_t tail_bytes = sha256_tail(message.subspan(offset), message.size(), tail);
        for (size_t i = 0; i < tail_bytes; i += sha256_block_bytes) {
            if (!write_block(std::span<const uint8_t>{tail}.subspan(i, sha256_block_bytes),
                             spins)) {
                last_error_ = Sha256Error::not_ready;
                return std::nullopt;
            }
        }
        return digest(spins);
    }

    /// Why the last `hash()` or `digest()` gave nothing. Meaningless
    /// after one that gave a digest.
    static Sha256Error last_error() { return last_error_; }

private:
    /// One store into CSR: the configuration fields as they read, plus
    /// the one action bit asked for. See the file header for why neither
    /// an atomic alias nor a plain read-modify-write will do.
    static void store(uint32_t action) { regs().CSR = (regs().CSR & csr_config_bits) | action; }

    static inline Sha256Error last_error_ = Sha256Error::not_ready;
};

} // namespace brio
