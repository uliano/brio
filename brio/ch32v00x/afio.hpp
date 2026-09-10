/*
 * afio.hpp
 *
 * The alternate-function remaps of the CH32V00x (RM 7.2.11, 7.3.2.2):
 * one register, AFIO_PCFR1, moves whole peripherals between pad sets -
 * a CODE per peripheral (four bits for TIM1 and USART1, three for
 * TIM2, USART2, SPI1 and I2C1, one for each ADC trigger) selecting one
 * COLUMN of the chapter's tables 7-8 to 7-15. Every column is stated
 * here as constexpr data, so a program names a code and gets its pads
 * from the same source that programs the register, and a pin table in
 * a driver can be built from a code instead of being retyped.
 *
 * WHAT THE TABLES SAY that a caller has to know:
 *  - a code is per PERIPHERAL, not per signal: TIM1's nine signals move
 *    together, and a column's pads may collide with another column's -
 *    the tables are the datasheet's and nothing here checks two
 *    peripherals against each other;
 *  - TIM1_RM = 11xx feeds TIM1_CH1 from the LSI instead of a pad (table
 *    7-8's note), which is a capture of the LSI with no wire;
 *  - PA1PA2_RM hands the crystal pads to GPIO;
 *  - SWCFG = 100 DISABLES THE DEBUG PORT until the next reset. The verb
 *    exists (a program that wants PD1 as a pad is a legitimate thing),
 *    spelled so that no caller reaches it by accident, and a program
 *    that calls it has lost the probe until it reboots;
 *  - USART2's default column puts TX on PA7 - the RESET PIN of the
 *    CH32V006K8 - so USART2 exists on this package only remapped.
 *
 * TIM2's table is the CH32V002/004/005/006 one (7-9-1); the CH32V007
 * moves one pad (CH2 at code 010 is PB3 there). I2C1's is 7-13-1; the
 * V007 differs at code 010's SDA. Stated for the part on the desk;
 * the second part will make this a tiered table.
 *
 * The drivers take their pads through this file: spi.hpp's SpiPins
 * and i2c.hpp's I2cPins carry a `remap` code and init() programs it;
 * usart.hpp's Uart takes the code as a template parameter; tim.hpp's
 * timers get `remap(code)` and a TimPad from `afio_tim1_pads(code)`.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pin.hpp"

namespace brio {

inline volatile uint32_t& afio_pcfr1() {
    return *reinterpret_cast<volatile uint32_t*>(pb2_base + 0x000C);
}

// PCFR1's fields (7.3.2.2)
inline constexpr uint32_t afio_spi1_rm_shift = 0;
inline constexpr uint32_t afio_i2c1_rm_shift = 3;
inline constexpr uint32_t afio_usart1_rm_shift = 6;
inline constexpr uint32_t afio_tim1_rm_shift = 10;
inline constexpr uint32_t afio_tim2_rm_shift = 14;
inline constexpr uint32_t afio_pa1pa2_rm = 1UL << 17;
inline constexpr uint32_t afio_adc_etrginj_rm = 1UL << 18;
inline constexpr uint32_t afio_adc_etrgreg_rm = 1UL << 19;
inline constexpr uint32_t afio_usart2_rm_shift = 20;
inline constexpr uint32_t afio_swcfg_shift = 24;

// =============================================================================
// The tables: one row per code, the pads in the chapter's column order
// =============================================================================

struct Tim1Pads {
    Pad etr, ch1, ch2, ch3, ch4, bkin, ch1n, ch2n, ch3n;
};
inline constexpr uint8_t afio_tim1_codes = 10;
inline constexpr Tim1Pads afio_tim1_table[afio_tim1_codes] = {
    {{'C', 5}, {'D', 2}, {'A', 1}, {'C', 3}, {'C', 4}, {'C', 2}, {'D', 0}, {'A', 2}, {'D', 1}},   // 0000 default
    {{'D', 4}, {'D', 2}, {'A', 1}, {'C', 3}, {'C', 4}, {'C', 2}, {'D', 0}, {'A', 2}, {'D', 1}},   // 0001
    {{'C', 5}, {'C', 6}, {'C', 7}, {'C', 0}, {'D', 3}, {'C', 1}, {'C', 3}, {'C', 4}, {'D', 1}},   // 0010
    {{'C', 2}, {'C', 4}, {'C', 7}, {'C', 5}, {'D', 4}, {'C', 1}, {'C', 3}, {'D', 2}, {'C', 6}},   // 0011
    {{'D', 4}, {'A', 3}, {'B', 0}, {'B', 1}, {'D', 1}, {'B', 3}, {'A', 0}, {'A', 2}, {'D', 0}},   // 0100
    {{'D', 4}, {'A', 3}, {'B', 0}, {'C', 3}, {'D', 1}, {'B', 3}, {'A', 0}, {'A', 2}, {'D', 0}},   // 0101
    {{'D', 4}, {'A', 3}, {'B', 0}, {'B', 1}, {'B', 2}, {'A', 7}, {'A', 0}, {'A', 2}, {'D', 0}},   // 0110
    {{'B', 4}, {'C', 4}, {'C', 5}, {'C', 6}, {'C', 7}, {'B', 2}, {'C', 0}, {'C', 1}, {'C', 2}},   // 0111
    {{'B', 4}, {'C', 4}, {'C', 5}, {'C', 6}, {'C', 7}, {'B', 2}, {'A', 3}, {'B', 0}, {'B', 1}},   // 1000
    {{'B', 4}, {'A', 0}, {'A', 1}, {'A', 2}, {'A', 3}, {'B', 2}, {'C', 0}, {'C', 1}, {'C', 2}},   // 1001
};
constexpr Tim1Pads afio_tim1_pads(uint8_t code) { return afio_tim1_table[code < afio_tim1_codes ? code : 0u]; }
/// Table 7-8's note: codes 11xx feed TIM1_CH1 from the LSI.
constexpr bool afio_tim1_ch1_is_lsi(uint8_t code) { return (code & 0xCu) == 0xCu; }

struct Tim2Pads {
    Pad ch1_etr, ch2, ch3, ch4;
};
inline constexpr uint8_t afio_tim2_codes = 8;
inline constexpr Tim2Pads afio_tim2_table[afio_tim2_codes] = {
    {{'D', 4}, {'D', 3}, {'C', 0}, {'D', 7}},   // 000 default
    {{'C', 1}, {'D', 3}, {'C', 0}, {'D', 7}},   // 001
    {{'C', 5}, {'C', 2}, {'D', 2}, {'C', 1}},   // 010
    {{'C', 1}, {'C', 7}, {'D', 6}, {'D', 5}},   // 011
    {{'C', 0}, {'C', 1}, {'C', 3}, {'B', 6}},   // 100
    {{'A', 0}, {'A', 1}, {'A', 2}, {'A', 3}},   // 101
    {{'B', 1}, {'A', 1}, {'A', 2}, {'A', 3}},   // 110
    {{'D', 3}, {'D', 4}, {'A', 2}, {'A', 3}},   // 111
};
constexpr Tim2Pads afio_tim2_pads(uint8_t code) { return afio_tim2_table[code < afio_tim2_codes ? code : 0u]; }

struct UsartPadSet {
    Pad tx, rx, cts, rts;
};
inline constexpr uint8_t afio_usart1_codes = 10;
inline constexpr UsartPadSet afio_usart1_table[afio_usart1_codes] = {
    {{'D', 5}, {'D', 6}, {'D', 3}, {'C', 2}},   // 0000 default
    {{'D', 6}, {'D', 5}, {'C', 6}, {'C', 7}},   // 0001
    {{'D', 0}, {'D', 1}, {'C', 3}, {'C', 2}},   // 0010
    {{'C', 0}, {'C', 1}, {'C', 6}, {'C', 7}},   // 0011
    {{'D', 1}, {'B', 3}, {'D', 7}, {'A', 5}},   // 0100
    {{'B', 3}, {'D', 1}, {'D', 7}, {'A', 5}},   // 0101
    {{'C', 5}, {'C', 6}, {'C', 7}, {'B', 4}},   // 0110
    {{'B', 5}, {'B', 6}, {'C', 7}, {'B', 4}},   // 0111
    {{'A', 0}, {'A', 1}, {'D', 2}, {'D', 3}},   // 1000
    {{'A', 0}, {'C', 4}, {'D', 5}, {'D', 4}},   // 1001
};
constexpr UsartPadSet afio_usart1_pads(uint8_t code) { return afio_usart1_table[code < afio_usart1_codes ? code : 0u]; }

inline constexpr uint8_t afio_usart2_codes = 7;
inline constexpr UsartPadSet afio_usart2_table[afio_usart2_codes] = {
    {{'A', 7}, {'B', 3}, {'A', 4}, {'A', 5}},   // 000 default: TX on the K8's reset pin
    {{'A', 4}, {'A', 5}, {'A', 7}, {'B', 3}},   // 001
    {{'A', 2}, {'A', 3}, {'A', 0}, {'A', 1}},   // 010
    {{'D', 2}, {'D', 3}, {'A', 0}, {'A', 1}},   // 011
    {{'B', 0}, {'B', 1}, {'B', 6}, {'A', 1}},   // 100
    {{'C', 4}, {'D', 1}, {'A', 4}, {'A', 1}},   // 101
    {{'A', 6}, {'A', 5}, {'A', 7}, {'B', 3}},   // 110
};
constexpr UsartPadSet afio_usart2_pads(uint8_t code) { return afio_usart2_table[code < afio_usart2_codes ? code : 0u]; }

struct SpiPadSet {
    Pad nss, sck, miso, mosi;
};
inline constexpr uint8_t afio_spi1_codes = 7;
inline constexpr SpiPadSet afio_spi1_table[afio_spi1_codes] = {
    {{'C', 1}, {'C', 5}, {'C', 7}, {'C', 6}},   // 000 default
    {{'C', 0}, {'C', 5}, {'C', 7}, {'C', 6}},   // 001
    {{'C', 4}, {'D', 2}, {'B', 3}, {'D', 3}},   // 010
    {{'B', 0}, {'B', 1}, {'B', 2}, {'C', 0}},   // 011
    {{'D', 3}, {'D', 4}, {'D', 5}, {'D', 6}},   // 100
    {{'C', 1}, {'A', 1}, {'B', 5}, {'A', 2}},   // 101
    {{'C', 4}, {'B', 5}, {'C', 7}, {'B', 4}},   // 110
};
constexpr SpiPadSet afio_spi1_pads(uint8_t code) { return afio_spi1_table[code < afio_spi1_codes ? code : 0u]; }

struct I2cPadSet {
    Pad scl, sda;
};
/// Codes 0..3, and 1xx (4 and above) the fifth column.
inline constexpr uint8_t afio_i2c1_codes = 5;
inline constexpr I2cPadSet afio_i2c1_table[afio_i2c1_codes] = {
    {{'C', 2}, {'C', 1}},   // 000 default
    {{'D', 1}, {'D', 0}},   // 001
    {{'C', 5}, {'C', 6}},   // 010
    {{'B', 5}, {'B', 6}},   // 011
    {{'B', 3}, {'D', 1}},   // 1xx
};
constexpr I2cPadSet afio_i2c1_pads(uint8_t code) { return afio_i2c1_table[code >= 4u ? 4u : code]; }

/// Tables 7-14 and 7-15: the ADC's two external trigger pads.
constexpr Pad afio_adc_injected_trigger_pad(bool remapped) { return remapped ? Pad{'A', 2} : Pad{'D', 1}; }
constexpr Pad afio_adc_rule_trigger_pad(bool remapped) { return remapped ? Pad{'C', 2} : Pad{'D', 3}; }

// =============================================================================
// The register
// =============================================================================

/// AFIO_PCFR1's fields as verbs. The AFIO's clock is opened first: the
/// register answers nothing without it.
struct Afio {
    Afio() = delete;

    static void clock_on() { rcc()->PB2PCENR |= rcc_pb2_afio; }

    static void remap_tim1(uint8_t code) { field(afio_tim1_rm_shift, 0xFu, code < afio_tim1_codes ? code : 0u); }
    static void remap_tim2(uint8_t code) { field(afio_tim2_rm_shift, 0x7u, code); }
    static void remap_usart1(uint8_t code) { field(afio_usart1_rm_shift, 0xFu, code < afio_usart1_codes ? code : 0u); }
    static void remap_usart2(uint8_t code) { field(afio_usart2_rm_shift, 0x7u, code < afio_usart2_codes ? code : 0u); }
    static void remap_spi1(uint8_t code) { field(afio_spi1_rm_shift, 0x7u, code < afio_spi1_codes ? code : 0u); }
    static void remap_i2c1(uint8_t code) { field(afio_i2c1_rm_shift, 0x7u, code); }
    static void remap_adc_injected_trigger(bool on) { bit(afio_adc_etrginj_rm, on); }
    static void remap_adc_rule_trigger(bool on) { bit(afio_adc_etrgreg_rm, on); }
    /// PA1 and PA2 as GPIO instead of the crystal's pads.
    static void pa1_pa2_gpio(bool on) { bit(afio_pa1pa2_rm, on); }

    static uint8_t tim1_remap() { return static_cast<uint8_t>((afio_pcfr1() >> afio_tim1_rm_shift) & 0xFu); }
    static uint8_t tim2_remap() { return static_cast<uint8_t>((afio_pcfr1() >> afio_tim2_rm_shift) & 0x7u); }
    static uint8_t usart1_remap() { return static_cast<uint8_t>((afio_pcfr1() >> afio_usart1_rm_shift) & 0xFu); }
    static uint8_t usart2_remap() { return static_cast<uint8_t>((afio_pcfr1() >> afio_usart2_rm_shift) & 0x7u); }
    static uint8_t spi1_remap() { return static_cast<uint8_t>((afio_pcfr1() >> afio_spi1_rm_shift) & 0x7u); }
    static uint8_t i2c1_remap() { return static_cast<uint8_t>((afio_pcfr1() >> afio_i2c1_rm_shift) & 0x7u); }

    /// SWCFG = 100: THE DEBUG PORT OFF, PD1 a pad, until the next reset.
    /// A program that calls this has lost its probe until it reboots -
    /// and a program that calls it early in every boot has lost it for
    /// good unless the boot reads a condition first. Spelled long on
    /// purpose.
    static void disable_debug_port_until_reset() { field(afio_swcfg_shift, 0x7u, 4u); }
    static bool debug_port_enabled() { return ((afio_pcfr1() >> afio_swcfg_shift) & 0x4u) == 0u; }

private:
    static void field(uint32_t shift, uint32_t mask, uint32_t value) {
        clock_on();
        afio_pcfr1() = (afio_pcfr1() & ~(mask << shift)) | ((value & mask) << shift);
    }
    static void bit(uint32_t b, bool on) {
        clock_on();
        afio_pcfr1() = on ? (afio_pcfr1() | b) : (afio_pcfr1() & ~b);
    }
};

// The tables' corners, pinned.
static_assert(afio_tim1_pads(0).ch1 == Pad{'D', 2} && afio_tim1_pads(3).ch1 == Pad{'C', 4} &&
              afio_tim1_pads(9).ch3n == Pad{'C', 2});
static_assert(afio_tim1_ch1_is_lsi(0xC) && afio_tim1_ch1_is_lsi(0xF) && !afio_tim1_ch1_is_lsi(0x3));
static_assert(afio_tim2_pads(0).ch1_etr == Pad{'D', 4} && afio_tim2_pads(7).ch2 == Pad{'D', 4});
static_assert(afio_usart1_pads(0).tx == Pad{'D', 5} && afio_usart1_pads(1).tx == Pad{'D', 6} &&
              afio_usart1_pads(9).rx == Pad{'C', 4});
static_assert(afio_usart2_pads(0).tx == Pad{'A', 7} && afio_usart2_pads(3).tx == Pad{'D', 2});
static_assert(afio_spi1_pads(0).sck == Pad{'C', 5} && afio_spi1_pads(4).mosi == Pad{'D', 6});
static_assert(afio_i2c1_pads(0).scl == Pad{'C', 2} && afio_i2c1_pads(7).sda == Pad{'D', 1});

} // namespace brio
