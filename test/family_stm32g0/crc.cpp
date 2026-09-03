// Family smoke TU: stm32g0/crc.hpp on each of the three headers the
// desk's boards span. Instantiation only - no main(), no hardware.
//
// The CRC is the same peripheral on every part of the family, so what
// this fixture checks is not per-header geometry but the CONFIGURATION
// CHECKER and the two named presets: an even polynomial and one wider
// than the size it declares are the two rules 14.3.3 states and the
// silicon does not enforce, and the presets' check values are what the
// bench suite is judged against.

#include <stdint.h>

#include <span>

#include "stm32g0/crc.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "util/crc.hpp"

using namespace brio;

// ---- presence --------------------------------------------------------------
static_assert(crc_present(), "every STM32G0 carries one CRC unit");
static_assert(crc_clock_mask() != 0 && crc_reset_mask() != 0);

// ---- the width arithmetic --------------------------------------------------
static_assert(crc_width_bits(CrcWidth::w32) == 32);
static_assert(crc_width_bits(CrcWidth::w16) == 16);
static_assert(crc_width_bits(CrcWidth::w8) == 8);
static_assert(crc_width_bits(CrcWidth::w7) == 7);
static_assert(crc_width_mask(CrcWidth::w32) == 0xFFFFFFFFu);
static_assert(crc_width_mask(CrcWidth::w16) == 0xFFFFu);
static_assert(crc_width_mask(CrcWidth::w8) == 0xFFu);
static_assert(crc_width_mask(CrcWidth::w7) == 0x7Fu);
// The register's own codes, since the enumerator IS what gets written.
static_assert(static_cast<uint8_t>(CrcWidth::w32) == 0 &&
              static_cast<uint8_t>(CrcWidth::w7) == 3);
static_assert(static_cast<uint8_t>(CrcReverse::none) == 0 &&
              static_cast<uint8_t>(CrcReverse::word) == 3);

// ---- 14.3.3's two unenforced rules -----------------------------------------
static_assert(crc_config_valid(CrcConfig{}), "the reset values are a legal config");
static_assert(!crc_config_valid(CrcConfig{.polynomial = 0x1020u, .size = CrcWidth::w16}),
              "even polynomials are not supported (14.3.3)");
static_assert(!crc_config_valid(CrcConfig{.polynomial = 0x11021u, .size = CrcWidth::w16}),
              "a 17-bit polynomial does not fit a 16-bit POLYSIZE");
static_assert(!crc_config_valid(CrcConfig{.polynomial = 0x1021u, .size = CrcWidth::w8}),
              "...nor a 16-bit one an 8-bit POLYSIZE");
static_assert(crc_config_valid(CrcConfig{.polynomial = 0x07u, .size = CrcWidth::w8}));
static_assert(crc_config_valid(CrcConfig{.polynomial = 0x09u, .size = CrcWidth::w7}),
              "CRC-7/MMC fits its seven bits");
static_assert(!crc_config_valid(CrcConfig{.polynomial = 0x89u, .size = CrcWidth::w7}),
              "...and a 0x89 does not");

// ---- the two named standards -----------------------------------------------
static_assert(crc_config_valid(crc_ccitt_false_config));
static_assert(crc_ccitt_false_config.polynomial == 0x1021u &&
                  crc_ccitt_false_config.init == 0xFFFFu &&
                  crc_ccitt_false_config.size == CrcWidth::w16 &&
                  crc_ccitt_false_config.reverse_in == CrcReverse::none &&
                  !crc_ccitt_false_config.reverse_out,
              "the preset must BE util/crc.hpp's CRC-16/CCITT-FALSE");
static_assert(crc_config_valid(crc32_ieee_config));
static_assert(crc32_ieee_config.polynomial == 0x04C11DB7u &&
                  crc32_ieee_config.init == 0xFFFFFFFFu &&
                  crc32_ieee_config.reverse_in == CrcReverse::byte &&
                  crc32_ieee_config.reverse_out);
static_assert(crc32_ieee_finish(0x340BC6D9u) == 0xCBF43926u,
              "the final XOR the hardware does not do: the raw register value "
              "the check string leaves, complemented, is the standard answer");
static_assert(crc32_ieee_finish(crc32_ieee_finish(0x12345678u)) == 0x12345678u,
              "and it is its own inverse");

// The software reference the bench suite is judged against, evaluated
// HERE at compile time so the check value itself cannot drift.
constexpr uint8_t check_string[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
static_assert(crc16(check_string, 9) == 0x29B1u,
              "util/crc.hpp's own CRC-16/CCITT-FALSE check value");

// ---- every verb ------------------------------------------------------------
constexpr uint8_t payload[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
constexpr uint32_t words[2] = {0x1A2B3C4Du, 0x11223344u};

void exercise_crc() {
    (void)&Crc::regs();
    Crc::bus_clock(true);
    (void)Crc::bus_clock();
    Crc::reset();
    Crc::init();

    (void)Crc::configure(crc_ccitt_false_config);
    (void)Crc::configure(crc32_ieee_config);
    (void)Crc::configure(CrcConfig{.polynomial = 0x09u,
                                   .size = CrcWidth::w7,
                                   .init = 0});
    Crc::restart();
    (void)Crc::width();
    (void)Crc::reverse_in();
    (void)Crc::reverse_out();
    (void)Crc::polynomial();
    (void)Crc::initial();

    Crc::feed(0x1A2B3C4Du);
    Crc::feed16(0x1234);
    Crc::feed8(0x5A);
    Crc::feed(std::span<const uint8_t>{payload});
    Crc::feed(std::span<const uint32_t>{words});

    (void)Crc::value();
    (void)Crc::value_masked();
    (void)Crc::value_masked_now();
    (void)Crc::data_address();

    Crc::scratch(0xDEADBEEFu);
    (void)Crc::scratch();

    Crc::release();
}
