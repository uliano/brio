/*
 * sha256.hpp
 *
 * SHA-256 in constexpr C++, as FIPS 180-4 defines it: the message
 * digested 512 bits at a time, the result 256 bits, the padding the
 * standard's own.
 *
 * WHY IT IS HERE AND NOT IN A DRIVER, for util/crc.hpp's reason. A
 * hardware accelerator for this function has nothing to configure: no
 * polynomial, no initial value, no variant - it computes SHA-256 and
 * only SHA-256, so what it computes has one home, and this is it. The
 * software twin below is
 *
 *   - what a bench suite judges such a block against, a known answer at
 *     compile time beside the number the silicon produced;
 *   - what a program uses while the block is busy, or on a target that
 *     has none;
 *   - what answers for a constant with no code at all.
 *
 * AND IT IS ALSO HALF OF THE DRIVER. The accelerators of this kind
 * digest whole 512-bit blocks and PAD NOTHING: software owes them the
 * trailing one bit, the zeros and the 64-bit length. `sha256_tail()`
 * below builds exactly those bytes, so the arithmetic that decides where
 * a message ends is written once, tested on the host, and shared by the
 * software path and the hardware one.
 *
 * COST. Sixty-four rounds a block over a sixty-four word schedule: about
 * 2 kB of constants and a few hundred bytes of code, and on the order of
 * a microsecond per block on a 150 MHz 32-bit core. The message schedule
 * is a local array of 64 words - 256 bytes of stack - which is why this
 * is a function and never an object a small target keeps around.
 *
 * NOT A MESSAGE AUTHENTICATION CODE. HMAC, key derivation and signature
 * verification are built ON this and are not here; nothing in brio needs
 * them yet.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <bit>
#include <span>

namespace brio {

/// The standard's block: 512 bits.
inline constexpr size_t sha256_block_bytes = 64;
/// And its result: 256 bits, in eight 32-bit words.
inline constexpr size_t sha256_digest_bytes = 32;
inline constexpr size_t sha256_digest_words = 8;

/**
 * A digest, as the eight state words H0..H7 the standard names - which
 * is also the order a hardware block of this kind presents them in, one
 * word per result register. `bytes()` is the canonical serialization:
 * each word most significant byte first, which is how a SHA-256 is
 * written down.
 */
struct Sha256Digest {
    std::array<uint32_t, sha256_digest_words> words{};

    constexpr bool operator==(const Sha256Digest&) const = default;

    constexpr std::array<uint8_t, sha256_digest_bytes> bytes() const {
        std::array<uint8_t, sha256_digest_bytes> out{};
        for (size_t i = 0; i < sha256_digest_words; ++i) {
            out[4 * i + 0] = static_cast<uint8_t>(words[i] >> 24);
            out[4 * i + 1] = static_cast<uint8_t>(words[i] >> 16);
            out[4 * i + 2] = static_cast<uint8_t>(words[i] >> 8);
            out[4 * i + 3] = static_cast<uint8_t>(words[i]);
        }
        return out;
    }
};

/// The initial state: the fractional parts of the square roots of the
/// first eight primes. A hardware block loads these when it is started.
inline constexpr std::array<uint32_t, sha256_digest_words> sha256_initial_state = {
    0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
    0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u,
};

/// The sixty-four round constants: the fractional parts of the cube
/// roots of the first sixty-four primes.
inline constexpr std::array<uint32_t, 64> sha256_round_constants = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u,
    0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
    0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
    0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
    0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
    0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u,
    0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u,
    0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
    0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

/// One message word, most significant byte first - the order the
/// standard defines the schedule in.
constexpr uint32_t sha256_word_be(std::span<const uint8_t> block, size_t index) {
    return (static_cast<uint32_t>(block[4 * index + 0]) << 24) |
           (static_cast<uint32_t>(block[4 * index + 1]) << 16) |
           (static_cast<uint32_t>(block[4 * index + 2]) << 8) |
           static_cast<uint32_t>(block[4 * index + 3]);
}

/**
 * One 512-bit block into the running state: the schedule expanded to
 * sixty-four words, sixty-four rounds over eight working variables, the
 * result added back. `block` is exactly `sha256_block_bytes` long;
 * anything shorter is the caller's error and reads past its own buffer,
 * which is why the only public entry points below own their own bounds.
 */
constexpr void sha256_compress(std::array<uint32_t, sha256_digest_words>& state,
                               std::span<const uint8_t> block) {
    std::array<uint32_t, 64> w{};
    for (size_t i = 0; i < 16; ++i) {
        w[i] = sha256_word_be(block, i);
    }
    for (size_t i = 16; i < 64; ++i) {
        const uint32_t a = w[i - 15];
        const uint32_t b = w[i - 2];
        const uint32_t s0 = std::rotr(a, 7) ^ std::rotr(a, 18) ^ (a >> 3);
        const uint32_t s1 = std::rotr(b, 17) ^ std::rotr(b, 19) ^ (b >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    uint32_t h0 = state[0];
    uint32_t h1 = state[1];
    uint32_t h2 = state[2];
    uint32_t h3 = state[3];
    uint32_t h4 = state[4];
    uint32_t h5 = state[5];
    uint32_t h6 = state[6];
    uint32_t h7 = state[7];

    for (size_t i = 0; i < 64; ++i) {
        const uint32_t s1 = std::rotr(h4, 6) ^ std::rotr(h4, 11) ^ std::rotr(h4, 25);
        const uint32_t ch = (h4 & h5) ^ (~h4 & h6);
        const uint32_t t1 = h7 + s1 + ch + sha256_round_constants[i] + w[i];
        const uint32_t s0 = std::rotr(h0, 2) ^ std::rotr(h0, 13) ^ std::rotr(h0, 22);
        const uint32_t maj = (h0 & h1) ^ (h0 & h2) ^ (h1 & h2);
        const uint32_t t2 = s0 + maj;
        h7 = h6;
        h6 = h5;
        h5 = h4;
        h4 = h3 + t1;
        h3 = h2;
        h2 = h1;
        h1 = h0;
        h0 = t1 + t2;
    }

    state[0] += h0;
    state[1] += h1;
    state[2] += h2;
    state[3] += h3;
    state[4] += h4;
    state[5] += h5;
    state[6] += h6;
    state[7] += h7;
}

/// The longest tail `sha256_tail()` can produce: at most 63 leftover
/// bytes, the one bit, the zeros and the eight length bytes never reach
/// past two blocks.
inline constexpr size_t sha256_max_tail_bytes = 2 * sha256_block_bytes;

/**
 * THE PADDING, as a run of bytes a caller can feed to anything that
 * digests whole blocks - the software loop below, or an accelerator's
 * data port.
 *
 * `leftover` is what is left of the message after every whole block has
 * been consumed (fewer than 64 bytes), `total_bytes` the length of the
 * WHOLE message. Writes into `out` the leftover, then a byte 0x80 (the
 * standard's single one bit followed by seven zeros), then zeros, then
 * the message length in BITS as eight bytes most significant first, and
 * returns how many bytes that came to - always 64 or 128, never
 * anything else, because the whole point of the padding is to land on a
 * block boundary.
 *
 * Returns 0, writing nothing, when `leftover` is 64 bytes or longer:
 * that is a caller who has not consumed the blocks it owed.
 */
constexpr size_t sha256_tail(std::span<const uint8_t> leftover, uint64_t total_bytes,
                             std::array<uint8_t, sha256_max_tail_bytes>& out) {
    const size_t used = leftover.size();
    if (used >= sha256_block_bytes) {
        return 0;
    }
    for (size_t i = 0; i < sha256_max_tail_bytes; ++i) {
        out[i] = 0;
    }
    for (size_t i = 0; i < used; ++i) {
        out[i] = leftover[i];
    }
    out[used] = 0x80u;
    // The length occupies the last eight bytes of the last block, so the
    // tail is one block when the marked-up message still fits in 56
    // bytes and two when it does not.
    const size_t length = used + 1 <= sha256_block_bytes - 8 ? sha256_block_bytes
                                                             : 2 * sha256_block_bytes;
    const uint64_t bits = total_bytes * 8u;
    for (size_t i = 0; i < 8; ++i) {
        out[length - 1 - i] = static_cast<uint8_t>(bits >> (8 * i));
    }
    return length;
}

/**
 * The whole function over a byte range: the digest of `message`, with
 * the padding done here.
 *
 *   constexpr auto d = brio::sha256(std::span<const uint8_t>{text});
 *
 * An empty message is legal and has the standard's well-known answer.
 */
constexpr Sha256Digest sha256(std::span<const uint8_t> message) {
    std::array<uint32_t, sha256_digest_words> state = sha256_initial_state;
    size_t offset = 0;
    while (message.size() - offset >= sha256_block_bytes) {
        sha256_compress(state, message.subspan(offset, sha256_block_bytes));
        offset += sha256_block_bytes;
    }
    std::array<uint8_t, sha256_max_tail_bytes> tail{};
    const size_t tail_bytes = sha256_tail(message.subspan(offset), message.size(), tail);
    for (size_t i = 0; i < tail_bytes; i += sha256_block_bytes) {
        sha256_compress(state, std::span<const uint8_t>{tail}.subspan(i, sha256_block_bytes));
    }
    return Sha256Digest{state};
}

} // namespace brio
