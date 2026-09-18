// Host tests for util/sha256.hpp: the published FIPS 180-4 vectors, the
// padding arithmetic on every boundary it has, the block-at-a-time path
// against the one-shot one, and the proof that the whole function
// answers at COMPILE time - which is what lets a bench suite carry a
// known answer beside the number a hardware accelerator produced.
// Run with: ctest --preset host (or ctest --preset host -R test_sha256)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "util/sha256.hpp"

namespace {

using brio::Sha256Digest;
using brio::sha256;

std::span<const uint8_t> bytes_of(const std::string& s) {
    return {reinterpret_cast<const uint8_t*>(s.data()), s.size()};
}

/// The digest as it is written down: sixty-four lowercase hex digits.
std::string hex_of(const Sha256Digest& d) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    for (uint8_t b : d.bytes()) {
        out.push_back(digits[b >> 4]);
        out.push_back(digits[b & 0x0Fu]);
    }
    return out;
}

/// The message digested one block at a time with the padding built by
/// hand - the path an accelerator's driver takes, spelled here in
/// software so that the two agree by test and not by hope.
Sha256Digest blockwise(std::span<const uint8_t> message) {
    auto state = brio::sha256_initial_state;
    size_t offset = 0;
    while (message.size() - offset >= brio::sha256_block_bytes) {
        brio::sha256_compress(state, message.subspan(offset, brio::sha256_block_bytes));
        offset += brio::sha256_block_bytes;
    }
    std::array<uint8_t, brio::sha256_max_tail_bytes> tail{};
    const size_t n = brio::sha256_tail(message.subspan(offset), message.size(), tail);
    for (size_t i = 0; i < n; i += brio::sha256_block_bytes) {
        brio::sha256_compress(
            state, std::span<const uint8_t>{tail}.subspan(i, brio::sha256_block_bytes));
    }
    return Sha256Digest{state};
}

} // namespace

TEST_CASE("the three published FIPS 180-4 vectors") {
    // The empty message, which the standard's own appendix does not list
    // but every implementation is pinned to.
    CHECK(hex_of(sha256({})) ==
          "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

    // One block: "abc", 24 bits (appendix B.1).
    const std::string abc = "abc";
    CHECK(hex_of(sha256(bytes_of(abc))) ==
          "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");

    // Two blocks: 448 bits, the length that forces the padding into a
    // second block (appendix B.2).
    const std::string two = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    REQUIRE(two.size() == 56);
    CHECK(hex_of(sha256(bytes_of(two))) ==
          "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");

    // A million 'a': the vector that exercises the 64-bit length field
    // and sixteen thousand blocks of schedule (appendix B.3).
    const std::string million(1'000'000, 'a');
    CHECK(hex_of(sha256(bytes_of(million))) ==
          "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST_CASE("the digest is the eight words H0..H7, most significant byte first") {
    const std::string abc = "abc";
    const Sha256Digest d = sha256(bytes_of(abc));
    CHECK(d.words[0] == 0xba7816bfu);
    CHECK(d.words[7] == 0xf20015adu);
    const auto b = d.bytes();
    CHECK(b[0] == 0xbau);
    CHECK(b[3] == 0xbfu);
    CHECK(b[31] == 0xadu);
    CHECK(b.size() == brio::sha256_digest_bytes);
}

TEST_CASE("the whole function answers at compile time") {
    static constexpr std::array<uint8_t, 3> abc = {'a', 'b', 'c'};
    static constexpr Sha256Digest d = sha256(std::span<const uint8_t>{abc});
    static_assert(d.words[0] == 0xba7816bfu);
    static_assert(d.words[7] == 0xf20015adu);
    static_assert(sha256(std::span<const uint8_t>{}).words[0] == 0xe3b0c442u);
    CHECK(d.words[0] == 0xba7816bfu);
}

TEST_CASE("the padding lands on a block boundary at every length") {
    std::array<uint8_t, brio::sha256_max_tail_bytes> tail{};
    for (size_t used = 0; used < brio::sha256_block_bytes; ++used) {
        const std::vector<uint8_t> left(used, 0x5Au);
        const size_t n = brio::sha256_tail(std::span<const uint8_t>{left}, used, tail);
        // One block while the marked-up message still fits in 56 bytes,
        // two after that - and never anything in between.
        CHECK(n == (used + 1 <= 56 ? 64u : 128u));
        CHECK(tail[used] == 0x80u);
        // Everything between the marker and the length field is zero.
        for (size_t i = used + 1; i < n - 8; ++i) {
            CHECK(tail[i] == 0u);
        }
        // And the last eight bytes are the bit count, big-endian.
        const uint64_t bits = static_cast<uint64_t>(used) * 8u;
        for (size_t i = 0; i < 8; ++i) {
            CHECK(tail[n - 1 - i] == static_cast<uint8_t>(bits >> (8 * i)));
        }
    }
}

TEST_CASE("a leftover of a whole block or more is refused") {
    std::array<uint8_t, brio::sha256_max_tail_bytes> tail{};
    const std::vector<uint8_t> too_much(brio::sha256_block_bytes, 0u);
    CHECK(brio::sha256_tail(std::span<const uint8_t>{too_much},
                            brio::sha256_block_bytes, tail) == 0u);
}

TEST_CASE("block at a time equals one shot, at every length across two blocks") {
    std::vector<uint8_t> message;
    for (size_t len = 0; len <= 200; ++len) {
        message.assign(len, 0u);
        for (size_t i = 0; i < len; ++i) {
            message[i] = static_cast<uint8_t>(i * 7u + 3u);
        }
        const std::span<const uint8_t> m{message};
        CHECK(blockwise(m) == sha256(m));
    }
}

TEST_CASE("a one-bit change moves the whole digest") {
    std::vector<uint8_t> a(4096, 0xA5u);
    std::vector<uint8_t> b = a;
    b[2048] ^= 0x01u;
    const Sha256Digest da = sha256(std::span<const uint8_t>{a});
    const Sha256Digest db = sha256(std::span<const uint8_t>{b});
    CHECK_FALSE(da == db);
    int differing = 0;
    for (size_t i = 0; i < brio::sha256_digest_words; ++i) {
        if (da.words[i] != db.words[i]) {
            ++differing;
        }
    }
    CHECK(differing == 8);
}
