/*
 * afio.hpp
 *
 * The alternate-function block of the CH32V203 (RM 10.2.11, 10.3.2):
 * the two remap registers, the external-interrupt port multiplexer and
 * the event output - everything in chapter 10 that is not a pad.
 *
 * A REMAP IS A COLUMN, NOT A PIN. AFIO_PCFR1 and AFIO_PCFR2 hold one
 * field per peripheral, and the field's value selects a whole COLUMN of
 * the chapter's tables: SPI1's four signals move together, TIM1's nine
 * do, a USART's five do. Every column this series has is stated here as
 * constexpr data, so a driver names a code and gets its pads from the
 * same source that programs the register - the CH32V00x stratum's
 * arrangement, and the one the later chapters of this family take their
 * pads from (the SPI's and I2C's pin structs carry a code, the Uart
 * takes one as a template parameter, a timer gets remap()).
 *
 * WHICH COLUMNS ARE OURS IS TWO QUESTIONS, AND THE FILE ANSWERS BOTH.
 *
 *  1. THE DEVICE CLASS. This manual covers four families and nearly
 *     every field carries a note naming the classes it applies to. Ours
 *     are CH32V20x_D6 (every part up to the CH32V203C8), CH32V20x_D8
 *     (the CH32V203RB) and CH32V30x_D8 (the four CH32V303). So USART1
 *     has the two columns of table 10-23 here and not its four - the
 *     high bit of its code is AFIO_PCFR2 bit 26, which 10.3.2.2's note
 *     excludes for the D6, the D8 and the D8W alike and grants the
 *     CH32V30x_D8, whose two further columns this driver does not
 *     reach; USART2's remap (10-24) is not the D6's; UART4 reads a
 *     DIFFERENT TABLE per class (10-26 for every class but the D6,
 *     10-27 for the CH32V203C8, whose default pads are PB0/PB1 and not
 *     PC10/PC11); the ADC's four trigger remaps (bits 17..20), SPI3's
 *     (28), CAN2's (22), the Ethernet's (21, 23), and PCFR2's TIM8..10,
 *     UART5..8 and FSMC fields belong to classes or blocks this driver
 *     does not reach - the CH32V303's among them - so no verb here
 *     writes them.
 *     TWO OF THOSE CLASS NOTES ARE MEASURED and not taken on trust,
 *     because the field is READ-ONLY AT ZERO on the CH32V203C8 where the
 *     note is easy to read the other way: USART3's two bits (its note
 *     (4) says the default mapping alone is the D6's) and TIM2's
 *     internal-trigger bit (whose note names this whole series). A write
 *     of either reads back zero on the silicon - the reference suite
 *     stages both - so the driver refuses them here and offers them on
 *     the other class, where no board has yet answered.
 *  2. THE PACKAGE. A column whose pads no pin of this package brings out
 *     is not a remap, it is a disconnection: TIM3's full column is
 *     PC6..PC9, which only the CH32V203RB bonds; TIM1's is port E, which
 *     no part of the series has at all; USART2's and USART3's upper
 *     columns need PD3..PD12. `afio_remap_has_code()` therefore asks the
 *     part's own bonding table (pin.hpp's `pad_bonded`) as well as the
 *     class, and a column with not one pad on this package is REFUSED -
 *     by a static_assert where the code is a constant, by false where it
 *     is not.
 *     The two questions agree wherever the manual states both: the
 *     columns it restricts to "64-pin and above" are exactly the ones
 *     whose pads only the 64-pin part bonds.
 *
 * THE DEBUG PORT IS NOT A REMAP. SW_CFG (PCFR1 26:24) hands PA13 and
 * PA14 back to GPIO, and a program that does that on this board has lost
 * its probe until the next power cycle - the two-wire port is how the
 * image got there. The field is READ here by default and written only
 * through a verb spelled long enough that nobody reaches it by accident.
 *
 * WHAT THE REGISTER PAIR ALSO HOLDS. AFIO_EXTICR1..4 (10.3.2.3..6) is
 * the multiplexer that says WHICH PORT feeds each of the sixteen pin
 * lines of the EXTI - exti.hpp's verbs go through this file's, the way
 * the STM32F4's go through SYSCFG's. AFIO_ECR (10.3.2.1) is the EVENT
 * OUTPUT: a core "EVENTOUT" signal brought out on a pad of ports A..D.
 * That signal is the Cortex-M3's, of the CH32F20x half of this manual;
 * nothing in the QingKe V4 processor manual names an instruction that
 * raises it, so the register is implemented and what can drive it here
 * is a question this stratum has not answered (docs/ch32v203/pin.md).
 *
 * The AFIO's clock gate is closed out of reset and every verb opens it,
 * the way pin.hpp's opens a port's: a register that answers nothing
 * because of a gate is the most expensive kind of silence.
 */

#pragma once

#include <stdint.h>

#include "ch32v203/clock.hpp"
#include "ch32v203/device.hpp"
#include "ch32v203/pin.hpp"

namespace brio {

// =============================================================================
// The columns, as the chapter's tables have them
// =============================================================================

/// TIM1's nine signals (table 10-15).
struct Tim1Pads {
    Pad etr, ch1, ch2, ch3, ch4, bkin, ch1n, ch2n, ch3n;

    constexpr bool any_bonded() const {
        return pad_bonded(etr) || pad_bonded(ch1) || pad_bonded(ch2) || pad_bonded(ch3) ||
               pad_bonded(ch4) || pad_bonded(bkin) || pad_bonded(ch1n) || pad_bonded(ch2n) ||
               pad_bonded(ch3n);
    }
};

/// A general-purpose timer's channels (tables 10-16..10-18). TIM3's and
/// TIM4's tables name no ETR: TIM3's is PD2 whatever the remap says
/// (10-17's note 1) and TIM4's reaches no pad of this series.
struct TimPads {
    Pad etr, ch1, ch2, ch3, ch4;

    constexpr bool any_bonded() const {
        return pad_bonded(etr) || pad_bonded(ch1) || pad_bonded(ch2) || pad_bonded(ch3) ||
               pad_bonded(ch4);
    }
};

/// A USART's five (tables 10-23..10-27). A column that has no CK, CTS or
/// RTS - UART4's on the CH32V203RB - leaves them invalid.
struct UsartPadSet {
    Pad tx, rx, ck, cts, rts;

    constexpr bool any_bonded() const {
        return pad_bonded(tx) || pad_bonded(rx) || pad_bonded(ck) || pad_bonded(cts) ||
               pad_bonded(rts);
    }
};

/// SPI's four (table 10-32).
struct SpiPadSet {
    Pad nss, sck, miso, mosi;

    constexpr bool any_bonded() const {
        return pad_bonded(nss) || pad_bonded(sck) || pad_bonded(miso) || pad_bonded(mosi);
    }
};

/// I2C's two, and the SMBus alert the remap table does not move: the
/// datasheet's pin table is where SMBA is named (CH32V203DS0 3.2 -
/// I2C1_SMBA on PB5, I2C2_SMBA on PB12).
struct I2cPadSet {
    Pad scl, sda, smba;

    constexpr bool any_bonded() const {
        return pad_bonded(scl) || pad_bonded(sda) || pad_bonded(smba);
    }
};

/// CAN's pair (table 10-35).
struct CanPadSet {
    Pad rx, tx;

    constexpr bool any_bonded() const { return pad_bonded(rx) || pad_bonded(tx); }
};

// ---- TIM1 (table 10-15) ----------------------------------------------------
// Code 10 is reserved and code 11 is port E, which no part of this
// series bonds: two columns are reachable here.
inline constexpr uint8_t afio_tim1_codes = 4;
inline constexpr Tim1Pads afio_tim1_table[afio_tim1_codes] = {
    {{'A', 12}, {'A', 8}, {'A', 9}, {'A', 10}, {'A', 11}, {'B', 12}, {'B', 13}, {'B', 14}, {'B', 15}},  // 00
    {{'A', 12}, {'A', 8}, {'A', 9}, {'A', 10}, {'A', 11}, {'A', 6}, {'A', 7}, {'B', 0}, {'B', 1}},      // 01
    {},                                                                                                 // 10 reserved
    {{'E', 7}, {'E', 9}, {'E', 11}, {'E', 13}, {'E', 14}, {'E', 15}, {'E', 8}, {'E', 10}, {'E', 12}},   // 11
};

// ---- TIM2 (table 10-16) ----------------------------------------------------
inline constexpr uint8_t afio_tim2_codes = 4;
inline constexpr TimPads afio_tim2_table[afio_tim2_codes] = {
    {{'A', 0}, {'A', 0}, {'A', 1}, {'A', 2}, {'A', 3}},        // 00
    {{'A', 15}, {'A', 15}, {'B', 3}, {'A', 2}, {'A', 3}},      // 01
    {{'A', 0}, {'A', 0}, {'A', 1}, {'B', 10}, {'B', 11}},      // 10
    {{'A', 15}, {'A', 15}, {'B', 3}, {'B', 10}, {'B', 11}},    // 11
};

// ---- TIM3 (table 10-17) ----------------------------------------------------
// Its external trigger is PD2 in every column (note 1), so the ETR field
// carries that pad and not a moving one.
inline constexpr uint8_t afio_tim3_codes = 4;
inline constexpr TimPads afio_tim3_table[afio_tim3_codes] = {
    {{'D', 2}, {'A', 6}, {'A', 7}, {'B', 0}, {'B', 1}},        // 00
    {},                                                        // 01 reserved
    {{'D', 2}, {'B', 4}, {'B', 5}, {'B', 0}, {'B', 1}},        // 10
    {{'D', 2}, {'C', 6}, {'C', 7}, {'C', 8}, {'C', 9}},        // 11
};

// ---- TIM4 (table 10-18) ----------------------------------------------------
inline constexpr uint8_t afio_tim4_codes = 2;
inline constexpr TimPads afio_tim4_table[afio_tim4_codes] = {
    {{}, {'B', 6}, {'B', 7}, {'B', 8}, {'B', 9}},              // 0
    {{}, {'D', 12}, {'D', 13}, {'D', 14}, {'D', 15}},          // 1
};

// ---- USART1 (table 10-23) --------------------------------------------------
// Two columns on this series: the code's high bit is AFIO_PCFR2 bit 26,
// which the D6, the D8 and the D8W all lack.
inline constexpr uint8_t afio_usart1_codes = 2;
inline constexpr UsartPadSet afio_usart1_table[afio_usart1_codes] = {
    {{'A', 9}, {'A', 10}, {'A', 8}, {'A', 11}, {'A', 12}},     // 0 default
    {{'B', 6}, {'B', 7}, {'A', 8}, {'A', 11}, {'A', 12}},      // 1
};

// ---- USART2 (table 10-24) --------------------------------------------------
inline constexpr uint8_t afio_usart2_codes = 2;
inline constexpr UsartPadSet afio_usart2_table[afio_usart2_codes] = {
    {{'A', 2}, {'A', 3}, {'A', 4}, {'A', 0}, {'A', 1}},        // 0 default
    {{'D', 5}, {'D', 6}, {'D', 7}, {'D', 3}, {'D', 4}},        // 1 (not the D6's)
};

// ---- USART3 (table 10-25) --------------------------------------------------
// The two upper columns need PD8..PD12 and are not this series': stated
// as the manual's, refused by the bonding.
inline constexpr uint8_t afio_usart3_codes = 4;
inline constexpr UsartPadSet afio_usart3_table[afio_usart3_codes] = {
    {{'B', 10}, {'B', 11}, {'B', 12}, {'B', 13}, {'B', 14}},   // 00 default
    {{'C', 10}, {'C', 11}, {'C', 12}, {'B', 13}, {'B', 14}},   // 01
    {{'A', 13}, {'A', 14}, {'D', 10}, {'D', 11}, {'D', 12}},   // 10
    {{'D', 8}, {'D', 9}, {'D', 10}, {'D', 11}, {'D', 12}},     // 11
};

// ---- UART4: two tables, one per device class -------------------------------
// Table 10-26 is the CH32V20x_D8's (TX/RX alone, from PC10/PC11); table
// 10-27 is named for the CH32V203C8 and carries five signals from
// PB0..PB4. A part with no fourth USART at all (device::has_usart(4))
// reaches neither.
inline constexpr uint8_t afio_uart4_codes = 4;
inline constexpr UsartPadSet afio_uart4_table_d8[afio_uart4_codes] = {
    {{'C', 10}, {'C', 11}, {}, {}, {}},                        // 00 default
    {{'B', 0}, {'B', 1}, {}, {}, {}},                          // 01
    {{'E', 0}, {'E', 1}, {}, {}, {}},                          // 1x
    {{'E', 0}, {'E', 1}, {}, {}, {}},
};
inline constexpr UsartPadSet afio_uart4_table_d6[afio_uart4_codes] = {
    {{'B', 0}, {'B', 1}, {'B', 2}, {'B', 3}, {'B', 4}},        // x0 default
    {{'A', 5}, {'B', 5}, {'A', 6}, {'A', 7}, {'A', 15}},       // x1
    {{'B', 0}, {'B', 1}, {'B', 2}, {'B', 3}, {'B', 4}},
    {{'A', 5}, {'B', 5}, {'A', 6}, {'A', 7}, {'A', 15}},
};

// ---- SPI (table 10-32) -----------------------------------------------------
inline constexpr uint8_t afio_spi1_codes = 2;
inline constexpr SpiPadSet afio_spi1_table[afio_spi1_codes] = {
    {{'A', 4}, {'A', 5}, {'A', 6}, {'A', 7}},                  // 0 default
    {{'A', 15}, {'B', 3}, {'B', 4}, {'B', 5}},                 // 1
};

/// SPI2 has no remap field: one column, from the datasheet's pin table.
inline constexpr SpiPadSet afio_spi2_pad_set = {{'B', 12}, {'B', 13}, {'B', 14}, {'B', 15}};

// ---- I2C (table 10-34) -----------------------------------------------------
inline constexpr uint8_t afio_i2c1_codes = 2;
inline constexpr I2cPadSet afio_i2c1_table[afio_i2c1_codes] = {
    {{'B', 6}, {'B', 7}, {'B', 5}},                            // 0 default
    {{'B', 8}, {'B', 9}, {'B', 5}},                            // 1 (SMBA does not move)
};

/// I2C2 has no remap field either.
inline constexpr I2cPadSet afio_i2c2_pad_set = {{'B', 10}, {'B', 11}, {'B', 12}};

// ---- CAN1 (table 10-35) ----------------------------------------------------
// Code 01 is reserved; code 11 lands on the oscillator's own two pads,
// which a program reaches only with the crystal off and PD0/PD1 handed
// to GPIO (Remap::pd0_pd1).
inline constexpr uint8_t afio_can1_codes = 4;
inline constexpr CanPadSet afio_can1_table[afio_can1_codes] = {
    {{'A', 11}, {'A', 12}},                                    // 00 default
    {},                                                        // 01 reserved
    {{'B', 8}, {'B', 9}},                                      // 10
    {{'D', 0}, {'D', 1}},                                      // 11
};

// =============================================================================
// The register fields
// =============================================================================

/**
 * The remap fields this SERIES has, one enumerator per field of
 * AFIO_PCFR1 and AFIO_PCFR2 that some part of it can use. What is
 * deliberately absent, with the class the manual gives it to: the ADC
 * trigger remaps (PCFR1 17..20, the CH32F20x_D8/D8C and the CH32V30x/
 * V31x), the Ethernet's (21 and 23) and CAN2's (22), SPI3's (28),
 * FSMC_NADV and the TIM8/9/10 fields of PCFR2, and USART1's high bit
 * (PCFR2 26) - and USART5..8's, which are the bigger families' fifth to
 * eighth serial ports. A field that is not here is not reachable through
 * this driver.
 */
enum class Remap : uint8_t {
    spi1,        ///< PCFR1 0
    i2c1,        ///< PCFR1 1
    usart1,      ///< PCFR1 2 (the low bit; this series has no high one)
    usart2,      ///< PCFR1 3
    usart3,      ///< PCFR1 5:4
    tim1,        ///< PCFR1 7:6
    tim2,        ///< PCFR1 9:8
    tim3,        ///< PCFR1 11:10
    tim4,        ///< PCFR1 12
    can1,        ///< PCFR1 14:13
    pd0_pd1,     ///< PCFR1 15: the oscillator's pads as GPIO
    tim5_ch4,    ///< PCFR1 16: TIM5's channel 4 from the LSI instead of a pad
    tim2_itr1,   ///< PCFR1 29: TIM2's internal trigger 1, PTP or the USB SOF
    ptp_pps,     ///< PCFR1 30: the Ethernet's pulse-per-second on PB5
    uart4,       ///< PCFR2 17:16
};

/// Where a field sits: which of the two registers, its lowest bit and
/// how many bits it has.
struct AfioField {
    bool second;    ///< true: AFIO_PCFR2, false: AFIO_PCFR1
    uint8_t shift;
    uint8_t width;
};

constexpr AfioField afio_field_of(Remap r) {
    switch (r) {
        case Remap::spi1:      return {false, 0, 1};
        case Remap::i2c1:      return {false, 1, 1};
        case Remap::usart1:    return {false, 2, 1};
        case Remap::usart2:    return {false, 3, 1};
        case Remap::usart3:    return {false, 4, 2};
        case Remap::tim1:      return {false, 6, 2};
        case Remap::tim2:      return {false, 8, 2};
        case Remap::tim3:      return {false, 10, 2};
        case Remap::tim4:      return {false, 12, 1};
        case Remap::can1:      return {false, 13, 2};
        case Remap::pd0_pd1:   return {false, 15, 1};
        case Remap::tim5_ch4:  return {false, 16, 1};
        case Remap::tim2_itr1: return {false, 29, 1};
        case Remap::ptp_pps:   return {false, 30, 1};
        case Remap::uart4:     return {true, 16, 2};
    }
    return {false, 0, 1};
}

/// The pads of one column, where the peripheral has a table of them.
constexpr Tim1Pads afio_tim1_pads(uint8_t code) {
    return afio_tim1_table[code < afio_tim1_codes ? code : 0u];
}
constexpr TimPads afio_tim2_pads(uint8_t code) {
    return afio_tim2_table[code < afio_tim2_codes ? code : 0u];
}
constexpr TimPads afio_tim3_pads(uint8_t code) {
    return afio_tim3_table[code < afio_tim3_codes ? code : 0u];
}
constexpr TimPads afio_tim4_pads(uint8_t code) {
    return afio_tim4_table[code < afio_tim4_codes ? code : 0u];
}
constexpr UsartPadSet afio_usart1_pads(uint8_t code) {
    return afio_usart1_table[code < afio_usart1_codes ? code : 0u];
}
constexpr UsartPadSet afio_usart2_pads(uint8_t code) {
    return afio_usart2_table[code < afio_usart2_codes ? code : 0u];
}
constexpr UsartPadSet afio_usart3_pads(uint8_t code) {
    return afio_usart3_table[code < afio_usart3_codes ? code : 0u];
}
/// UART4's column, from the table its DEVICE CLASS reads: table 10-27 is
/// the CH32V20x_D6's, and table 10-26's note names every other class this
/// stratum serves.
constexpr UsartPadSet afio_uart4_pads(uint8_t code) {
    const uint8_t c = code < afio_uart4_codes ? code : 0u;
    return device::device_class != DeviceClass::v20x_d6 ? afio_uart4_table_d8[c]
                                                        : afio_uart4_table_d6[c];
}
constexpr SpiPadSet afio_spi1_pads(uint8_t code) {
    return afio_spi1_table[code < afio_spi1_codes ? code : 0u];
}
constexpr I2cPadSet afio_i2c1_pads(uint8_t code) {
    return afio_i2c1_table[code < afio_i2c1_codes ? code : 0u];
}
constexpr CanPadSet afio_can1_pads(uint8_t code) {
    return afio_can1_table[code < afio_can1_codes ? code : 0u];
}

/// The pads of a USART's column by INSTANCE number, which is what a
/// transport's pin parameter wants: 1..4, and an instance this part has
/// not got answers with nothing bonded.
constexpr UsartPadSet afio_usart_pads(uint8_t instance, uint8_t code) {
    return !device::has_usart(instance) ? UsartPadSet{} :
           instance == 1 ? afio_usart1_pads(code) :
           instance == 2 ? afio_usart2_pads(code) :
           instance == 3 ? afio_usart3_pads(code) :
           instance == 4 ? afio_uart4_pads(code) : UsartPadSet{};
}

/**
 * Is `code` a column THIS PART can use? Two conditions, both from the
 * documents (see the file header):
 *
 *  - the DEVICE CLASS has the column: a reserved code is not one, and
 *    neither is a column the manual gives to another class;
 *  - the PACKAGE brings out at least one of its pads: a column with
 *    none of them is a disconnection and not a remap.
 *
 * The parts the two documents name for the PD0/PD1 remap - the
 * CH32V203C6/C8, F6P6, G6U6, K8T6 (RM 10.2.11.2, datasheet note 4) -
 * are exactly the parts whose table bonds those two pads, so the
 * bonding answers for that field too, including for the CH32V203K6,
 * which neither document names and whose package is the K8's.
 */
constexpr bool afio_remap_has_code(Remap r, uint8_t code) {
    switch (r) {
        case Remap::spi1:
            return device::spi_count >= 1 && code < afio_spi1_codes &&
                   afio_spi1_pads(code).any_bonded();
        case Remap::i2c1:
            return device::i2c_count >= 1 && code < afio_i2c1_codes &&
                   afio_i2c1_pads(code).any_bonded();
        case Remap::usart1:
            return device::has_usart(1) && code < afio_usart1_codes &&
                   afio_usart1_pads(code).any_bonded();
        // 10.3.2.2's note: the D6 has the default mapping alone.
        case Remap::usart2:
            return device::has_usart(2) && code < afio_usart2_codes &&
                   (code == 0 || device::device_class != DeviceClass::v20x_d6) &&
                   afio_usart2_pads(code).any_bonded();
        // MEASURED on the CH32V203C8: the field is tied to zero on this
        // class - a write of 01 reads back 00 - which is what
        // 10.3.2.2's note (4) means by "default mapping (00b) only
        // exists for CH32V20x_D6, CH32F20x_D6". The D8's columns are
        // the manual's word alone, and no board of that part is here.
        case Remap::usart3:
            return device::has_usart(3) && code < afio_usart3_codes &&
                   (code == 0 || device::device_class != DeviceClass::v20x_d6) &&
                   afio_usart3_pads(code).any_bonded();
        case Remap::uart4:
            return device::has_usart(4) && code < afio_uart4_codes &&
                   afio_uart4_pads(code).any_bonded();
        case Remap::tim1:
            return device::advanced_timer_count >= 1 && code < afio_tim1_codes &&
                   afio_tim1_pads(code).any_bonded();
        case Remap::tim2:
            return device::general_timer_count >= 1 && code < afio_tim2_codes &&
                   afio_tim2_pads(code).any_bonded();
        case Remap::tim3:
            return device::general_timer_count >= 2 && code < afio_tim3_codes &&
                   afio_tim3_pads(code).any_bonded();
        case Remap::tim4:
            return device::general_timer_count >= 3 && code < afio_tim4_codes &&
                   afio_tim4_pads(code).any_bonded();
        case Remap::can1:
            return device::can_count >= 1 && code < afio_can1_codes &&
                   afio_can1_pads(code).any_bonded();
        case Remap::pd0_pd1:
            return code < 2 && pad_bonded(Pad{'D', 0}) && pad_bonded(Pad{'D', 1});
        // The channel's other end is the LSI, so code 1 needs no pad;
        // the timer itself is the CH32V203RB's.
        case Remap::tim5_ch4:
            return device::has_tim5 && code < 2;
        // No pad either: the trigger is wired inside, from the Ethernet's
        // time stamp (code 0) or the USB controller's frame marker (1).
        // MEASURED on the CH32V203C8: the bit is tied to zero here, so
        // the choice belongs to the class that has an Ethernet PTP to
        // choose against, whatever 10.3.2.2's note says about the series.
        case Remap::tim2_itr1:
            return device::general_timer_count >= 1 &&
                   device::device_class != DeviceClass::v20x_d6 && code < 2;
        // The Ethernet's pulse-per-second on PB5, and only where there
        // is an Ethernet to make one.
        case Remap::ptp_pps:
            return device::has_ethernet && code < 2;
    }
    return false;
}

/// How many pads of a column this package brings out - what tells a
/// program whether the SIGNALS it needs are there, where
/// afio_remap_has_code() only promises that some pad is.
constexpr bool afio_column_fully_bonded(Remap r, uint8_t code) {
    switch (r) {
        case Remap::spi1: {
            const SpiPadSet p = afio_spi1_pads(code);
            return pad_bonded(p.nss) && pad_bonded(p.sck) && pad_bonded(p.miso) &&
                   pad_bonded(p.mosi);
        }
        case Remap::i2c1: {
            const I2cPadSet p = afio_i2c1_pads(code);
            return pad_bonded(p.scl) && pad_bonded(p.sda);
        }
        case Remap::usart1:
        case Remap::usart2:
        case Remap::usart3:
        case Remap::uart4: {
            const UsartPadSet p = r == Remap::usart1   ? afio_usart1_pads(code)
                                  : r == Remap::usart2 ? afio_usart2_pads(code)
                                  : r == Remap::usart3 ? afio_usart3_pads(code)
                                                       : afio_uart4_pads(code);
            return pad_bonded(p.tx) && pad_bonded(p.rx);
        }
        case Remap::tim1: {
            const Tim1Pads p = afio_tim1_pads(code);
            return pad_bonded(p.ch1) && pad_bonded(p.ch2) && pad_bonded(p.ch3) &&
                   pad_bonded(p.ch4);
        }
        case Remap::tim2:
        case Remap::tim3:
        case Remap::tim4: {
            const TimPads p = r == Remap::tim2   ? afio_tim2_pads(code)
                              : r == Remap::tim3 ? afio_tim3_pads(code)
                                                 : afio_tim4_pads(code);
            return pad_bonded(p.ch1) && pad_bonded(p.ch2) && pad_bonded(p.ch3) &&
                   pad_bonded(p.ch4);
        }
        case Remap::can1: {
            const CanPadSet p = afio_can1_pads(code);
            return pad_bonded(p.rx) && pad_bonded(p.tx);
        }
        case Remap::pd0_pd1:
            return pad_bonded(Pad{'D', 0}) && pad_bonded(Pad{'D', 1});
        case Remap::tim5_ch4:
        case Remap::tim2_itr1:
            return afio_remap_has_code(r, code);
        case Remap::ptp_pps:
            return afio_remap_has_code(r, code) && (code == 0 || pad_bonded(Pad{'B', 5}));
    }
    return false;
}

// =============================================================================
// The EXTI multiplexer's encoding (10.3.2.3..10.3.2.6)
// =============================================================================

/// Four bits per line: 0000 the PA pin of that number, 0001 PB, 0010 PC,
/// 0011 PD, 0100 PE, the rest reserved. 0xFF for a letter this family
/// has no port for.
constexpr uint8_t exti_port_code(char port) {
    return port == 'A'   ? 0u
           : port == 'B' ? 1u
           : port == 'C' ? 2u
           : port == 'D' ? 3u
           : port == 'E' ? 4u
                         : 0xFFu;
}

/// The reverse, for a read-back: 0 for a code no port of this family
/// answers to.
constexpr char exti_port_letter(uint8_t code) {
    return code == 0u   ? 'A'
           : code == 1u ? 'B'
           : code == 2u ? 'C'
           : code == 3u ? 'D'
           : code == 4u ? 'E'
                        : '\0';
}

// =============================================================================
// The block
// =============================================================================

/**
 * AFIO, monostate: the remaps, the EXTI multiplexer, the event output
 * and the debug port's own field. Every verb opens the block's clock
 * gate first - PB2PCENR bit 0, closed out of reset - because these
 * registers answer nothing without it and write nothing either.
 */
struct Afio {
    Afio() = delete;

    static void clock_on() { Rcc::enable(Bus::pb2, rcc_pb2_afio); }

    static AfioRegs& regs() { return *afio(); }

    // ---- the remaps --------------------------------------------------------

    /**
     * Select a column. False - and NOTHING WRITTEN - when this part has
     * no such column: a code the device class does not have, a reserved
     * one, or one whose pads no pin of this package brings out.
     */
    static bool remap(Remap r, uint8_t code) {
        if (!afio_remap_has_code(r, code)) {
            return false;
        }
        write_field(afio_field_of(r), code);
        return true;
    }

    /// The same where the column is a constant: the refusal is a compile
    /// error, which is what a program that has chosen its pads wants.
    template <Remap R, uint8_t Code>
    static void remap() {
        static_assert(afio_remap_has_code(R, Code),
                      "brio Afio::remap: this part has no such column - the code is reserved, "
                      "or the device class does not have it, or this package brings out not one "
                      "of its pads (brio/ch32v203/afio.hpp's tables, parts/<part>.hpp's bonding)");
        write_field(afio_field_of(R), Code);
    }

    /// What the field holds now.
    static uint8_t remap_code(Remap r) {
        clock_on();
        const AfioField f = afio_field_of(r);
        const uint32_t v = f.second ? regs().PCFR2 : regs().PCFR1;
        return static_cast<uint8_t>((v >> f.shift) & mask_of(f.width));
    }

    // ---- the debug port (PCFR1 26:24) --------------------------------------

    /// SW_CFG as it stands: 0xx while the two-wire port is alive, 100
    /// once it has been given up.
    static uint8_t debug_config() {
        clock_on();
        return static_cast<uint8_t>((regs().PCFR1 >> 24) & 0x7u);
    }

    /// Is PA13/PA14 still the debug port's?
    static bool debug_port_enabled() { return (debug_config() & 0x4u) == 0u; }

    /**
     * SW_CFG = 100: THE DEBUG PORT OFF, PA13 and PA14 plain pads, until
     * the next reset. A program that calls this has lost its probe until
     * the board reboots - and one that calls it early in every boot has
     * lost it for good unless the boot reads a condition first. Spelled
     * long on purpose.
     */
    static void disable_debug_port_until_reset() { write_field({false, 24, 3}, 4); }

    // ---- the EXTI multiplexer (AFIO_EXTICR1..4) ----------------------------

    /**
     * Point EXTI line `line` (0..15) at port `port`'s pin of the same
     * number. False for a line outside the sixteen, a letter this family
     * has no port for, or a port this package bonds no pin of.
     *
     * Nothing here refuses to take a line from another port - that is a
     * policy, and it lives one level up in exti.hpp, which is where a
     * claim can be seen.
     */
    static bool exti_source(uint8_t line, char port) {
        const uint8_t code = exti_port_code(port);
        if (line >= 16u || code == 0xFFu || !device::has_port(port)) {
            return false;
        }
        clock_on();
        const uint32_t shift = static_cast<uint32_t>(line & 3u) * 4u;
        volatile uint32_t& reg = regs().EXTICR[line >> 2];
        reg = (reg & ~(0xFUL << shift)) | (static_cast<uint32_t>(code) << shift);
        return true;
    }

    /// Which port feeds that line now, as a letter; 0 for a line outside
    /// the sixteen or a code no port of this family answers to.
    static char exti_source(uint8_t line) {
        if (line >= 16u) {
            return '\0';
        }
        clock_on();
        const uint32_t shift = static_cast<uint32_t>(line & 3u) * 4u;
        return exti_port_letter(static_cast<uint8_t>((regs().EXTICR[line >> 2] >> shift) & 0xFu));
    }

    // ---- the event output (AFIO_ECR) ---------------------------------------

    /**
     * Bring the core's EVENTOUT signal out on a pad: EVOE with the port
     * and the pin. Ports A..D only - the field's other codes are
     * reserved - and the pin must be one this package bonds.
     *
     * WHAT DRIVES THE SIGNAL is not this chapter's to say, and the
     * QingKe V4 processor manual does not name an instruction that
     * raises it (the Cortex-M3 part of this family has SEV). The
     * register is here, and what it does on this core is an open
     * question, not a promise.
     */
    static bool event_output(char port, uint8_t pin) {
        const uint8_t code = exti_port_code(port);
        if (code > 3u || pin >= 16u || !pad_bonded(Pad{port, pin})) {
            return false;
        }
        clock_on();
        regs().ECR = (1UL << 7) | (static_cast<uint32_t>(code) << 4) | static_cast<uint32_t>(pin);
        return true;
    }

    /// EVOE cleared: the pad is the program's again. The port and pin
    /// fields keep what they held, as the silicon does.
    static void event_output_off() {
        clock_on();
        regs().ECR &= ~(1UL << 7);
    }

    static bool event_output_enabled() {
        clock_on();
        return (regs().ECR & (1UL << 7)) != 0u;
    }
    static char event_output_port() {
        clock_on();
        return exti_port_letter(static_cast<uint8_t>((regs().ECR >> 4) & 0x7u));
    }
    static uint8_t event_output_pin() {
        clock_on();
        return static_cast<uint8_t>(regs().ECR & 0xFu);
    }

private:
    static constexpr uint32_t mask_of(uint8_t width) { return (1UL << width) - 1UL; }

    static void write_field(AfioField f, uint32_t value) {
        clock_on();
        volatile uint32_t& reg = f.second ? regs().PCFR2 : regs().PCFR1;
        const uint32_t m = mask_of(f.width) << f.shift;
        reg = (reg & ~m) | ((value << f.shift) & m);
    }
};

// The tables' corners, pinned - the manual's own numbers, checked here
// so a typo in a column is a compile error and not a wire that does
// nothing.
static_assert(afio_tim1_pads(0).ch1 == Pad{'A', 8} && afio_tim1_pads(0).bkin == Pad{'B', 12} &&
              afio_tim1_pads(1).bkin == Pad{'A', 6} && afio_tim1_pads(1).ch3n == Pad{'B', 1} &&
              afio_tim1_pads(3).etr == Pad{'E', 7});
static_assert(afio_tim2_pads(0).ch1 == Pad{'A', 0} && afio_tim2_pads(1).ch2 == Pad{'B', 3} &&
              afio_tim2_pads(2).ch3 == Pad{'B', 10} && afio_tim2_pads(3).etr == Pad{'A', 15});
static_assert(afio_tim3_pads(0).ch1 == Pad{'A', 6} && afio_tim3_pads(2).ch1 == Pad{'B', 4} &&
              afio_tim3_pads(3).ch4 == Pad{'C', 9} && afio_tim3_pads(0).etr == Pad{'D', 2} &&
              afio_tim3_pads(2).etr == Pad{'D', 2});
static_assert(afio_tim4_pads(0).ch1 == Pad{'B', 6} && afio_tim4_pads(1).ch4 == Pad{'D', 15} &&
              !afio_tim4_pads(0).etr.valid());
static_assert(afio_usart1_pads(0).tx == Pad{'A', 9} && afio_usart1_pads(1).tx == Pad{'B', 6} &&
              afio_usart1_pads(1).rx == Pad{'B', 7} && afio_usart1_pads(1).ck == Pad{'A', 8});
static_assert(afio_usart2_pads(0).tx == Pad{'A', 2} && afio_usart2_pads(1).tx == Pad{'D', 5});
static_assert(afio_usart3_pads(0).tx == Pad{'B', 10} && afio_usart3_pads(1).tx == Pad{'C', 10} &&
              afio_usart3_pads(3).rx == Pad{'D', 9});
static_assert(afio_spi1_pads(0).sck == Pad{'A', 5} && afio_spi1_pads(1).mosi == Pad{'B', 5} &&
              afio_spi2_pad_set.sck == Pad{'B', 13});
static_assert(afio_i2c1_pads(0).scl == Pad{'B', 6} && afio_i2c1_pads(1).sda == Pad{'B', 9} &&
              afio_i2c1_pads(1).smba == Pad{'B', 5} && afio_i2c2_pad_set.scl == Pad{'B', 10});
static_assert(afio_can1_pads(0).rx == Pad{'A', 11} && afio_can1_pads(2).tx == Pad{'B', 9} &&
              afio_can1_pads(3).tx == Pad{'D', 1});
static_assert(afio_uart4_pads(0).tx == (device::device_class != DeviceClass::v20x_d6
                                            ? Pad{'C', 10} : Pad{'B', 0}) &&
              afio_uart4_pads(1).tx == (device::device_class != DeviceClass::v20x_d6
                                            ? Pad{'B', 0} : Pad{'A', 5}));

// The codes a package decides: TIM1's full column is port E, which only
// the LQFP100 bonds, and TIM1's code 10 is reserved everywhere; USART2's
// remap is not the D6's.
static_assert(!afio_remap_has_code(Remap::tim1, 2));
static_assert(afio_remap_has_code(Remap::tim1, 3) == device::has_port('E'));
static_assert(!afio_remap_has_code(Remap::usart2, 1) ||
              device::device_class != DeviceClass::v20x_d6);
static_assert(!afio_remap_has_code(Remap::tim3, 1) && !afio_remap_has_code(Remap::can1, 1));

// The multiplexer's codes are the chapter's.
static_assert(exti_port_code('A') == 0 && exti_port_code('E') == 4 && exti_port_code('F') == 0xFF);
static_assert(exti_port_letter(3) == 'D' && exti_port_letter(5) == '\0');

} // namespace brio
