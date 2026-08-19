// Host tests for util/analog.hpp: reference millivolts, counts <-> mV,
// DAC codes, temperature formula. Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include "util/analog.hpp"

using namespace brio;

TEST_CASE("reference millivolts") {
    static_assert(ref_mv(Ref::v1024) == 1024);
    static_assert(ref_mv(Ref::v4096) == 4096);
    static_assert(ref_mv(Ref::vdd) == 0);
    CHECK(ref_mv(Ref::vdd, 3300) == 3300);
    CHECK(ref_mv(Ref::vrefa, 2046) == 2046);
}

TEST_CASE("adc counts to millivolts, rounded") {
    CHECK(adc_mv(0, 4096, 2048) == 0);
    CHECK(adc_mv(4095, 4096, 2048) == 2048);      // 2047.5 rounds up
    CHECK(adc_mv(2048, 4096, 2048) == 1024);
    CHECK(adc_mv(1023, 1024, 4096) == 4092);
    CHECK(adc_mv(16 * 2048, 16 * 4096, 2048) == 1024);   // accumulated x16
    CHECK(adc_mv_signed(-2048, 2048, 2048) == -2048);
    CHECK(adc_mv_signed(2047, 2048, 2048) == 2047);
    CHECK(adc_mv_signed(-1, 2048, 2048) == -1);
}

TEST_CASE("dac code and back") {
    CHECK(dac_code(0, 1024, 2048) == 0);
    CHECK(dac_code(1024, 1024, 2048) == 512);
    CHECK(dac_code(2048, 1024, 2048) == 1023);      // saturates
    CHECK(dac_code(5000, 1024, 2048) == 1023);
    CHECK(dac_code(100, 1024, 0) == 0);             // unknown reference
    CHECK(dac_mv(512, 1024, 2048) == 1024);
    CHECK(dac_mv(1023, 1024, 2048) == 2046);
}

TEST_CASE("temperature formula from the datasheet example shape") {
    // offset - result = 1000, slope 300: 1000*300/4096 = 73.2 -> 73 K (toy numbers)
    CHECK(temp_kelvin(2000, 300, 3000) == 73);
    // a realistic device: slope ~ 3000, offset ~ 3800, reading ~ 3400 -> ~293 K
    const uint16_t k = temp_kelvin(3400, 3000, 3800);
    CHECK(k > 280);
    CHECK(k < 305);
}
