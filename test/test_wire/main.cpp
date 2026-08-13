// Host tests for util/wire.hpp: big-endian word helpers, in particular
// the sign extension of 24-bit ADC words. Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>

#include "util/wire.hpp"

using namespace brio;

// The helpers are constexpr: lock the core cases at compile time too.
namespace {
constexpr uint8_t k24_min[] = {0x80, 0x00, 0x00};
constexpr uint8_t k24_neg1[] = {0xFF, 0xFF, 0xFF};
constexpr uint8_t k24_max[] = {0x7F, 0xFF, 0xFF};
static_assert(load_be24_signed(k24_min) == -8388608);
static_assert(load_be24_signed(k24_neg1) == -1);
static_assert(load_be24_signed(k24_max) == 8388607);
static_assert(load_be16((const uint8_t[]){0x12, 0x34}) == 0x1234);
static_assert(load_be32((const uint8_t[]){0xDE, 0xAD, 0xBE, 0xEF}) ==
              0xDEADBEEF);
} // namespace

TEST_CASE("16-bit round trip") {
    uint8_t buf[2];
    for (uint32_t v : {0x0000u, 0x0001u, 0x1234u, 0x8000u, 0xFFFFu}) {
        store_be16(buf, static_cast<uint16_t>(v));
        CHECK(load_be16(buf) == v);
    }
    store_be16(buf, 0xA1B2);
    CHECK(buf[0] == 0xA1);                   // MSB first on the wire
    CHECK(buf[1] == 0xB2);
}

TEST_CASE("24-bit round trip and wire order") {
    uint8_t buf[3];
    store_be24(buf, 0x123456);
    CHECK(buf[0] == 0x12);
    CHECK(buf[1] == 0x34);
    CHECK(buf[2] == 0x56);
    CHECK(load_be24(buf) == 0x123456);
    store_be24(buf, 0xFFABCDEF);             // top byte must be dropped
    CHECK(load_be24(buf) == 0xABCDEF);
}

TEST_CASE("24-bit signed: two's complement sign extension") {
    uint8_t buf[3];
    store_be24(buf, 0x000000);
    CHECK(load_be24_signed(buf) == 0);
    store_be24(buf, 0x000001);
    CHECK(load_be24_signed(buf) == 1);
    store_be24(buf, 0xFFFFFF);
    CHECK(load_be24_signed(buf) == -1);
    store_be24(buf, 0x800000);
    CHECK(load_be24_signed(buf) == -8388608);
    store_be24(buf, 0x7FFFFF);
    CHECK(load_be24_signed(buf) == 8388607);
}

TEST_CASE("32-bit round trip") {
    uint8_t buf[4];
    store_be32(buf, 0xDEADBEEF);
    CHECK(buf[0] == 0xDE);
    CHECK(buf[3] == 0xEF);
    CHECK(load_be32(buf) == 0xDEADBEEF);
}

TEST_CASE("a multi-word frame is indexed by stride") {
    // four 24-bit groups back to back, ADS131M-style
    uint8_t frame[12];
    store_be24(frame + 0, 0x000102);
    store_be24(frame + 3, 0xFFFFFE);
    store_be24(frame + 6, 0x7FFFFF);
    store_be24(frame + 9, 0x800001);
    CHECK(load_be24_signed(frame + 0) == 0x0102);
    CHECK(load_be24_signed(frame + 3) == -2);
    CHECK(load_be24_signed(frame + 6) == 8388607);
    CHECK(load_be24_signed(frame + 9) == -8388607);
}
