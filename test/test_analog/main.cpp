// Host tests for util/analog.hpp: counts <-> mV, DAC codes.
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include "util/analog.hpp"

using namespace brio;

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
