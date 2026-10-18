// Host tests for util/analog.hpp: counts <-> mV, DAC codes.
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

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

TEST_CASE("adc sums of any width to millivolts") {
    // the 32-bit form overflows here: 4095 x 1024 x 1024 + half > 2^32
    CHECK(adc_mv(4095u * 1024u, 4096u * 1024u, 1024) != 1024);
    CHECK(adc_sum_mv(4095u * 1024u, 4096u * 1024u, 1024) == 1024);     // 1023.75 rounds up
    CHECK(adc_sum_mv(2048u * 1024u, 4096u * 1024u, 4096) == 2048);
    CHECK(adc_sum_mv(4095u * 1024u, 4096u * 1024u, 5000) == 4999);     // 4998.8
    CHECK(adc_sum_mv(0, 4096u * 1024u, 5000) == 0);
    CHECK(adc_sum_mv(2048, 4096, 2048) == adc_mv(2048, 4096, 2048));    // agrees where both fit
    CHECK(adc_sum_mv(1, 4096u * 1024u, 65535) == 0);
    CHECK(adc_sum_mv_signed(-2048 * 1024, 2048u * 1024u, 2048) == -2048);
    CHECK(adc_sum_mv_signed(2047 * 1024, 2048u * 1024u, 4096) == 4094);
    CHECK(adc_sum_mv_signed(-1, 2048, 2048) == -1);
    static_assert(adc_sum_mv(4095u * 1024u, 4096u * 1024u, 1024) == 1024);
}
