// Host tests for util/rgb_lamp.hpp + util/pwm_channel.hpp: the PwmChannel
// concept, per-channel scaling (8-bit identity, on/off, 16-bit ceil).
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>

#include "util/rgb_lamp.hpp"

namespace {

template <uint16_t Max>
struct FakeChannel {
    static constexpr uint16_t max = Max;
    static inline uint16_t last = 0xFFFF;
    static void duty(uint16_t v) { last = v; }
};

using C8 = FakeChannel<255>;
using C1 = FakeChannel<1>;
using C16 = FakeChannel<65535>;

static_assert(brio::PwmChannel<C8>);
static_assert(brio::PwmChannel<C1>);
static_assert(brio::PwmChannel<C16>);

struct NotAChannel { static void duty(uint16_t) {} };   // no max
static_assert(!brio::PwmChannel<NotAChannel>);

} // namespace

TEST_CASE("8-bit channels get the level unchanged") {
    brio::RgbLamp<C8, C8, C8>::show({0, 90, 255});
    CHECK(C8::last == 255);
    brio::RgbLamp<C8, C1, C16>::show({7, 0, 0});
    CHECK(C8::last == 7);
}

TEST_CASE("on/off channels: any non-zero level is on, zero is off") {
    brio::RgbLamp<C8, C1, C16>::show({0, 1, 0});
    CHECK(C1::last == 1);
    brio::RgbLamp<C8, C1, C16>::show({0, 90, 0});
    CHECK(C1::last == 1);
    brio::RgbLamp<C8, C1, C16>::show({0, 0, 0});
    CHECK(C1::last == 0);
}

TEST_CASE("wide channels scale with ceil: 255 -> max, small levels never vanish") {
    brio::RgbLamp<C8, C1, C16>::show({0, 0, 255});
    CHECK(C16::last == 65535);
    brio::RgbLamp<C8, C1, C16>::show({0, 0, 1});
    CHECK(C16::last == 257);       // ceil(65535/255) = 257
    brio::RgbLamp<C8, C1, C16>::show({0, 0, 128});
    CHECK(C16::last == 32896);     // ceil(128*65535/255)
    brio::RgbLamp<C8, C1, C16>::off();
    CHECK(C16::last == 0);
}
