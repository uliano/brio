// The padding of a message can need two blocks, so the tail buffer is
// 128 bytes and a single block will not do.
#include <array>
#include <span>

#include "util/sha256.hpp"

void f() {
    std::array<uint8_t, brio::sha256_block_bytes> tail{};
    (void)brio::sha256_tail(std::span<const uint8_t>{}, 0, tail);
}
