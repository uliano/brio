/*
 * afio.hpp
 *
 * The alternate-function block of the CH32V203 and the CH32V303 (RM
 * 10.2.11, 10.3.2): the two remap registers, the external-interrupt port
 * multiplexer and the event output - everything in chapter 10 that is not
 * a pad.
 *
 * A REMAP IS A COLUMN, NOT A PIN. AFIO_PCFR1 and AFIO_PCFR2 hold one
 * field per peripheral, and the field's value selects a whole COLUMN of
 * the chapter's tables: SPI1's four signals move together, TIM1's nine
 * do, a USART's five do. Every column this family has is stated here as
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
 *     has the two columns of table 10-23 on the CH32V203 and all four on
 *     the CH32V303 - the high bit of its code is AFIO_PCFR2 bit 26, which
 *     10.3.2.2's note excludes for the D6, the D8 and the D8W alike -
 *     USART2's remap (10-24) is not the D6's; UART4 reads a DIFFERENT
 *     TABLE per class (10-26 for every class but the D6, 10-27 for the
 *     CH32V203C8, whose default pads are PB0/PB1 and not PC10/PC11); and
 *     the fields the CH32V303 adds - TIM8, TIM9, TIM10, UART5..8, SPI3,
 *     the four ADC trigger remaps and the FSMC's - exist where their block
 *     does, which the part table says. CAN2's field (PCFR1 22) and the
 *     Ethernet's (21, 23) belong to blocks no part of this stratum
 *     carries: they are named, so that a program asking for them is
 *     REFUSED rather than writing a bit, and nothing reaches them.
 *     TWO OF THOSE CLASS NOTES ARE MEASURED and not taken on trust,
 *     because the field is READ-ONLY AT ZERO on the CH32V203C8 where the
 *     note is easy to read the other way: USART3's two bits (its note
 *     (4) says the default mapping alone is the D6's) and TIM2's
 *     internal-trigger bit (whose note names this whole series). A write
 *     of either reads back zero on that silicon - the reference suite
 *     stages both - so the driver refuses them on the D6 and offers them
 *     on the other two classes; on the CH32V303VCT6 both take a write
 *     and read it back (measured by the same letter).
 *  2. THE PACKAGE. A column whose pads no pin of this package brings out
 *     is not a remap, it is a disconnection: TIM3's full column is
 *     PC6..PC9, which only the 64-pin and 100-pin parts bond; TIM1's is
 *     port E, which only the LQFP100 has; USART2's and USART3's upper
 *     columns need PD3..PD12. `afio_remap_has_code()` therefore asks the
 *     part's own bonding table (pin.hpp's `pad_bonded`) as well as the
 *     class, and a column with not one pad on this package is REFUSED -
 *     by a static_assert where the code is a constant, by false where it
 *     is not. The CH32V303's three extra advanced timers are judged by
 *     their FOUR CHANNEL PADS and not by one: the columns the manual
 *     restricts to the LQFP100 in words ("only LQFP100 package") are
 *     exactly the ones whose channel pads the 64-pin part does not bond -
 *     where TIM10's full column would otherwise pass on the two PD0/PD1
 *     pads that package shares with its oscillator.
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
 * is a question this stratum has not answered (docs/ch32vx03/pin.md).
 *
 * The AFIO's clock gate is closed out of reset and every verb opens it,
 * the way pin.hpp's opens a port's: a register that answers nothing
 * because of a gate is the most expensive kind of silence.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <array>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/pin.hpp"

namespace brio {

// =============================================================================
// The columns, as the chapter's tables have them
// =============================================================================

/// An advanced-control timer's nine signals (tables 10-15, 10-20..10-22):
/// TIM1's, and on the CH32V303RC and VC TIM8's, TIM9's and TIM10's.
struct AdvancedTimPads {
    Pad etr, ch1, ch2, ch3, ch4, bkin, ch1n, ch2n, ch3n;

    constexpr bool any_bonded() const {
        return pad_bonded(etr) || pad_bonded(ch1) || pad_bonded(ch2) || pad_bonded(ch3) ||
               pad_bonded(ch4) || pad_bonded(bkin) || pad_bonded(ch1n) || pad_bonded(ch2n) ||
               pad_bonded(ch3n);
    }
    /// All four channel pads on this package - how the CH32V303's three
    /// extra advanced timers are judged (the file header).
    constexpr bool channels_bonded() const {
        return pad_bonded(ch1) && pad_bonded(ch2) && pad_bonded(ch3) && pad_bonded(ch4);
    }
};

/// A general-purpose timer's channels (tables 10-16..10-18). TIM3's ETR
/// is PD2 whatever the remap says (10-17's note 1), and TIM4's is PE0 in
/// both of its columns (the CH32V303 datasheet's table 3-4) - a pad the
/// LQFP100 alone bonds.
struct TimPads {
    Pad etr, ch1, ch2, ch3, ch4;

    constexpr bool any_bonded() const {
        return pad_bonded(etr) || pad_bonded(ch1) || pad_bonded(ch2) || pad_bonded(ch3) ||
               pad_bonded(ch4);
    }
};

/// A USART's five (tables 10-23..10-31). A column that has no CK, CTS or
/// RTS - UART4's on the classes that read table 10-26, and UART5..8's -
/// leaves them invalid.
struct UsartPadSet {
    Pad tx, rx, ck, cts, rts;

    constexpr bool any_bonded() const {
        return pad_bonded(tx) || pad_bonded(rx) || pad_bonded(ck) || pad_bonded(cts) ||
               pad_bonded(rts);
    }
};

/// SPI's four (tables 10-32, 10-33).
struct SpiPadSet {
    Pad nss, sck, miso, mosi;

    constexpr bool any_bonded() const {
        return pad_bonded(nss) || pad_bonded(sck) || pad_bonded(miso) || pad_bonded(mosi);
    }
};

/// The I2S face of SPI2 and SPI3 (table 10-33's second half, the
/// CH32V303 datasheet's table 3-4): the word select, the bit clock, the
/// data and the master clock. SPI3_RM moves I2S3's four with SPI3's.
struct I2sPadSet {
    Pad ws, ck, sd, mck;

    constexpr bool any_bonded() const {
        return pad_bonded(ws) || pad_bonded(ck) || pad_bonded(sd) || pad_bonded(mck);
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

/// CAN's pair (tables 10-35, 10-36).
struct CanPadSet {
    Pad rx, tx;

    constexpr bool any_bonded() const { return pad_bonded(rx) || pad_bonded(tx); }
};

// ---- TIM1 (table 10-15) ----------------------------------------------------
// Code 10 is reserved and code 11 is port E, which only the LQFP100
// bonds.
inline constexpr uint8_t afio_tim1_codes = 4;
inline constexpr AdvancedTimPads afio_tim1_table[afio_tim1_codes] = {
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
// Table 10-18 names no ETR. The CH32V303 datasheet's table 3-4 puts
// TIM4_ETR on PE0 under both codes; the CH32V203's names none - no part
// of that series has a port E - so the entry is that class's.
inline constexpr Pad afio_tim4_etr =
    device::device_class == DeviceClass::v30x_d8 ? Pad{'E', 0} : Pad{};
inline constexpr uint8_t afio_tim4_codes = 2;
inline constexpr TimPads afio_tim4_table[afio_tim4_codes] = {
    {afio_tim4_etr, {'B', 6}, {'B', 7}, {'B', 8}, {'B', 9}},       // 0
    {afio_tim4_etr, {'D', 12}, {'D', 13}, {'D', 14}, {'D', 15}},   // 1
};

// ---- TIM5 -------------------------------------------------------------------
// No column of pads at all: CH1..CH4 are PA0..PA3 on every part that has
// the timer (the datasheets' pin tables), and the one AFIO field that
// names it (TIM5CH4_RM, table 10-19) moves channel 4 to the LSI, which is
// not a pad. No pin of either series carries a TIM5_ETR.
inline constexpr TimPads afio_tim5_pad_set = {{}, {'A', 0}, {'A', 1}, {'A', 2}, {'A', 3}};

// ---- TIM8 (table 10-20), the CH32V303RC's and VC's ---------------------------
inline constexpr uint8_t afio_tim8_codes = 2;
inline constexpr AdvancedTimPads afio_tim8_table[afio_tim8_codes] = {
    {{'A', 0}, {'C', 6}, {'C', 7}, {'C', 8}, {'C', 9}, {'A', 6}, {'A', 7}, {'B', 0}, {'B', 1}},     // 0
    {{'A', 0}, {'B', 6}, {'B', 7}, {'B', 8}, {'C', 13}, {'B', 9}, {'A', 13}, {'A', 14}, {'A', 15}}, // 1
};

// ---- TIM9 (table 10-21 - titled "TIM8" in the manual, its rows TIM9's) --------
// The 1x columns are "only LQFP100" (note 1): port D's pads.
inline constexpr uint8_t afio_tim9_codes = 4;
inline constexpr AdvancedTimPads afio_tim9_table[afio_tim9_codes] = {
    {{'A', 2}, {'A', 2}, {'A', 3}, {'A', 4}, {'C', 4}, {'C', 5}, {'C', 0}, {'C', 1}, {'C', 2}},        // 00
    {{'A', 2}, {'A', 2}, {'A', 3}, {'A', 4}, {'C', 14}, {'A', 1}, {'B', 0}, {'B', 1}, {'B', 2}},       // 01
    {{'D', 9}, {'D', 9}, {'D', 11}, {'D', 13}, {'D', 15}, {'D', 14}, {'D', 8}, {'D', 10}, {'D', 12}},  // 10
    {{'D', 9}, {'D', 9}, {'D', 11}, {'D', 13}, {'D', 15}, {'D', 14}, {'D', 8}, {'D', 10}, {'D', 12}},  // 11
};

// ---- TIM10 (table 10-22) ------------------------------------------------------
// The 1x columns are "only LQFP100" (note 1); their ETR and CH1 are PD0
// and PD1, which the smaller packages bond as their oscillator's pads -
// which is why these timers are judged by their channel pads.
inline constexpr uint8_t afio_tim10_codes = 4;
inline constexpr AdvancedTimPads afio_tim10_table[afio_tim10_codes] = {
    {{'C', 10}, {'B', 8}, {'B', 9}, {'C', 3}, {'C', 11}, {'C', 12}, {'A', 12}, {'A', 13}, {'A', 14}},  // 00
    {{'B', 11}, {'B', 3}, {'B', 4}, {'B', 5}, {'C', 15}, {'B', 10}, {'A', 5}, {'A', 6}, {'A', 7}},     // 01
    {{'D', 0}, {'D', 1}, {'D', 3}, {'D', 5}, {'D', 7}, {'E', 2}, {'E', 3}, {'E', 4}, {'E', 5}},        // 10
    {{'D', 0}, {'D', 1}, {'D', 3}, {'D', 5}, {'D', 7}, {'E', 2}, {'E', 3}, {'E', 4}, {'E', 5}},        // 11
};

// ---- USART1 (table 10-23) --------------------------------------------------
// Four columns, the code's low bit in AFIO_PCFR1 (bit 2) and its high bit
// in AFIO_PCFR2 (bit 26); the two upper columns are the CH32V30x_D8's
// alone - the note gives the D6, the D8 and the D8W "only the default
// mapping (00b) and remapping (01b)" - so a part of those classes has a
// table of two.

/// Whether this class has USART1's high code bit, AFIO_PCFR2 bit 26.
inline constexpr bool afio_usart1_high_bit = device::device_class == DeviceClass::v30x_d8;

inline constexpr UsartPadSet afio_usart1_columns[4] = {
    {{'A', 9}, {'A', 10}, {'A', 8}, {'A', 11}, {'A', 12}},     // 00 default
    {{'B', 6}, {'B', 7}, {'A', 8}, {'A', 11}, {'A', 12}},      // 01
    {{'B', 15}, {'A', 8}, {'A', 10}, {'A', 5}, {'A', 9}},      // 10 (the CH32V303's)
    {{'A', 6}, {'A', 7}, {'A', 5}, {'C', 4}, {'C', 5}},        // 11 (the CH32V303's)
};
inline constexpr uint8_t afio_usart1_codes = afio_usart1_high_bit ? 4u : 2u;

/// The first `N` columns of a table, as the array a class reads.
template <uint8_t N, typename T, size_t M>
constexpr std::array<T, N> afio_first_columns(const T (&all)[M]) {
    static_assert(N <= M);
    std::array<T, N> out{};
    for (uint8_t i = 0; i < N; ++i) {
        out[i] = all[i];
    }
    return out;
}

inline constexpr std::array<UsartPadSet, afio_usart1_codes> afio_usart1_table =
    afio_first_columns<afio_usart1_codes>(afio_usart1_columns);

// ---- USART2 (table 10-24) --------------------------------------------------
inline constexpr uint8_t afio_usart2_codes = 2;
inline constexpr UsartPadSet afio_usart2_table[afio_usart2_codes] = {
    {{'A', 2}, {'A', 3}, {'A', 4}, {'A', 0}, {'A', 1}},        // 0 default
    {{'D', 5}, {'D', 6}, {'D', 7}, {'D', 3}, {'D', 4}},        // 1 (not the D6's)
};

// ---- USART3 (table 10-25) --------------------------------------------------
// The upper columns need PD8..PD12, which only the LQFP100 bonds.
inline constexpr uint8_t afio_usart3_codes = 4;
inline constexpr UsartPadSet afio_usart3_table[afio_usart3_codes] = {
    {{'B', 10}, {'B', 11}, {'B', 12}, {'B', 13}, {'B', 14}},   // 00 default
    {{'C', 10}, {'C', 11}, {'C', 12}, {'B', 13}, {'B', 14}},   // 01
    {{'A', 13}, {'A', 14}, {'D', 10}, {'D', 11}, {'D', 12}},   // 10
    {{'D', 8}, {'D', 9}, {'D', 10}, {'D', 11}, {'D', 12}},     // 11
};

// ---- UART4: two tables, one per device class -------------------------------
// Table 10-26 is every class's but the D6's (TX/RX alone, from PC10/PC11);
// table 10-27 is named for the CH32V203C8 and carries five signals from
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

// ---- UART5..UART8 (tables 10-28..10-31), the CH32V303RC's and VC's ----------
// TX and RX alone, and each third column port E's.
inline constexpr uint8_t afio_uart5_8_codes = 4;
inline constexpr UsartPadSet afio_uart5_table[afio_uart5_8_codes] = {
    {{'C', 12}, {'D', 2}, {}, {}, {}},                         // 00 default
    {{'B', 4}, {'B', 5}, {}, {}, {}},                          // 01
    {{'E', 8}, {'E', 9}, {}, {}, {}},                          // 1x
    {{'E', 8}, {'E', 9}, {}, {}, {}},
};
inline constexpr UsartPadSet afio_uart6_table[afio_uart5_8_codes] = {
    {{'C', 0}, {'C', 1}, {}, {}, {}},
    {{'B', 8}, {'B', 9}, {}, {}, {}},
    {{'E', 10}, {'E', 11}, {}, {}, {}},
    {{'E', 10}, {'E', 11}, {}, {}, {}},
};
inline constexpr UsartPadSet afio_uart7_table[afio_uart5_8_codes] = {
    {{'C', 2}, {'C', 3}, {}, {}, {}},
    {{'A', 6}, {'A', 7}, {}, {}, {}},
    {{'E', 12}, {'E', 13}, {}, {}, {}},
    {{'E', 12}, {'E', 13}, {}, {}, {}},
};
inline constexpr UsartPadSet afio_uart8_table[afio_uart5_8_codes] = {
    {{'C', 4}, {'C', 5}, {}, {}, {}},
    {{'A', 14}, {'A', 15}, {}, {}, {}},
    {{'E', 14}, {'E', 15}, {}, {}, {}},
    {{'E', 14}, {'E', 15}, {}, {}, {}},
};

// ---- SPI (tables 10-32, 10-33) ---------------------------------------------
inline constexpr uint8_t afio_spi1_codes = 2;
inline constexpr SpiPadSet afio_spi1_table[afio_spi1_codes] = {
    {{'A', 4}, {'A', 5}, {'A', 6}, {'A', 7}},                  // 0 default
    {{'A', 15}, {'B', 3}, {'B', 4}, {'B', 5}},                 // 1
};

/// SPI2 has no remap field: one column, from the datasheet's pin table.
inline constexpr SpiPadSet afio_spi2_pad_set = {{'B', 12}, {'B', 13}, {'B', 14}, {'B', 15}};

/// SPI3, the CH32V303RC's and VC's: two columns behind PCFR1 bit 28.
inline constexpr uint8_t afio_spi3_codes = 2;
inline constexpr SpiPadSet afio_spi3_table[afio_spi3_codes] = {
    {{'A', 15}, {'B', 3}, {'B', 4}, {'B', 5}},                 // 0 default
    {{'A', 4}, {'C', 10}, {'C', 11}, {'C', 12}},               // 1
};

/// I2S2 on SPI2's pads, its master clock on PC6 (the CH32V303
/// datasheet's table 3-4); no remap field.
inline constexpr I2sPadSet afio_i2s2_pad_set = {{'B', 12}, {'B', 13}, {'B', 15}, {'C', 6}};

/// I2S3 on SPI3's columns - the same field moves both - with the master
/// clock on PC7 in either (table 10-33).
inline constexpr I2sPadSet afio_i2s3_table[afio_spi3_codes] = {
    {{'A', 15}, {'B', 3}, {'B', 5}, {'C', 7}},                 // 0 default
    {{'A', 4}, {'C', 10}, {'C', 12}, {'C', 7}},                // 1
};

// ---- I2C (table 10-34) -----------------------------------------------------
inline constexpr uint8_t afio_i2c1_codes = 2;
inline constexpr I2cPadSet afio_i2c1_table[afio_i2c1_codes] = {
    {{'B', 6}, {'B', 7}, {'B', 5}},                            // 0 default
    {{'B', 8}, {'B', 9}, {'B', 5}},                            // 1 (SMBA does not move)
};

/// I2C2 has no remap field either.
inline constexpr I2cPadSet afio_i2c2_pad_set = {{'B', 10}, {'B', 11}, {'B', 12}};

// ---- CAN (tables 10-35, 10-36) ---------------------------------------------
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

/// CAN2's two columns, the manual's data for a block no part of this
/// stratum carries (device::can_count is one everywhere): its field is
/// refused.
inline constexpr uint8_t afio_can2_codes = 2;
inline constexpr CanPadSet afio_can2_table[afio_can2_codes] = {
    {{'B', 12}, {'B', 13}},                                    // 0 default
    {{'B', 5}, {'B', 6}},                                      // 1
};

// =============================================================================
// The register fields
// =============================================================================

/**
 * The remap fields of AFIO_PCFR1 and AFIO_PCFR2 this stratum names, one
 * enumerator per field. Which of them a PART has is
 * `afio_remap_has_code()`'s question; two are named only to be refused
 * (CAN2's and the Ethernet's receive pads, blocks of other classes).
 * MII_RMII_SEL (PCFR1 23) is not a remap but the Ethernet MAC's own
 * configuration, and is not here.
 */
enum class Remap : uint8_t {
    spi1,        ///< PCFR1 0
    i2c1,        ///< PCFR1 1
    usart1,      ///< PCFR1 2, and PCFR2 26 as the high bit on the CH32V303
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
    spi3,        ///< PCFR1 28: SPI3 and I2S3 (the CH32V303RC and VC)
    tim8,        ///< PCFR2 2
    tim9,        ///< PCFR2 4:3
    tim10,       ///< PCFR2 6:5
    uart5,       ///< PCFR2 19:18
    uart6,       ///< PCFR2 21:20
    uart7,       ///< PCFR2 23:22
    uart8,       ///< PCFR2 25:24
    adc1_etrginj, ///< PCFR1 17: ADC1's injected trigger from EXTI15 to TIM8_CH4
    adc1_etrgreg, ///< PCFR1 18: ADC1's regular trigger from EXTI11 to TIM8_TRGO
    adc2_etrginj, ///< PCFR1 19: the same for ADC2
    adc2_etrgreg, ///< PCFR1 20
    fsmc_nadv,   ///< PCFR2 10: FSMC_NADV on PB7 (0) or not driven (1)
    can2,        ///< PCFR1 22 - refused: no part here has a second CAN
    eth,         ///< PCFR1 21 - refused: the Ethernet's MII pads, other classes'
};

/// Where a field sits: which of the two registers, its lowest bit and
/// how many bits it has.
struct AfioField {
    bool second;    ///< true: AFIO_PCFR2, false: AFIO_PCFR1
    uint8_t shift;
    uint8_t width;
};

/// Whether this part's DEVICE CLASS has a field for `r` at all: the
/// fifteen fields the CH32V203 knows are every class's, the rest the
/// CH32V303's - and CAN2's and the Ethernet's, which name blocks of
/// classes neither series is, are given a position on that class alone
/// so that a read of them reads a reserved-at-zero bit and not another
/// field.
constexpr bool afio_field_present(Remap r) {
    return device::device_class == DeviceClass::v30x_d8 ||
           static_cast<uint8_t>(r) <= static_cast<uint8_t>(Remap::uart4);
}

/// The field of a remap, where the class has one (afio_field_present).
/// USART1's is its LOW bit alone - the high one, PCFR2 bit 26, is the
/// CH32V303's and Afio writes it beside (`afio_usart1_high_bit`).
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
        default: break;
    }
    // The CH32V303's own fields. On the CH32V203's two classes this block
    // is not compiled at all: there the fields are refused before anyone
    // asks where they are (afio_remap_has_code), and remap_code<R>()
    // refuses a read of one at compile time.
    if constexpr (device::device_class == DeviceClass::v30x_d8) {
        switch (r) {
            case Remap::spi3:         return {false, 28, 1};
            case Remap::tim8:         return {true, 2, 1};
            case Remap::tim9:         return {true, 3, 2};
            case Remap::tim10:        return {true, 5, 2};
            case Remap::uart5:        return {true, 18, 2};
            case Remap::uart6:        return {true, 20, 2};
            case Remap::uart7:        return {true, 22, 2};
            case Remap::uart8:        return {true, 24, 2};
            case Remap::adc1_etrginj: return {false, 17, 1};
            case Remap::adc1_etrgreg: return {false, 18, 1};
            case Remap::adc2_etrginj: return {false, 19, 1};
            case Remap::adc2_etrgreg: return {false, 20, 1};
            case Remap::fsmc_nadv:    return {true, 10, 1};
            case Remap::can2:         return {false, 22, 1};
            case Remap::eth:          return {false, 21, 1};
            default: break;
        }
    }
    // A field this class has not got. No verb writes one (remap() asks
    // afio_remap_has_code() first); what a READ of one answers is the
    // precondition remap_code() states.
    return {false, 0, 1};
}

/// The pads of one column, where the peripheral has a table of them.
constexpr AdvancedTimPads afio_tim1_pads(uint8_t code) {
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
constexpr AdvancedTimPads afio_tim8_pads(uint8_t code) {
    return afio_tim8_table[code < afio_tim8_codes ? code : 0u];
}
constexpr AdvancedTimPads afio_tim9_pads(uint8_t code) {
    return afio_tim9_table[code < afio_tim9_codes ? code : 0u];
}
constexpr AdvancedTimPads afio_tim10_pads(uint8_t code) {
    return afio_tim10_table[code < afio_tim10_codes ? code : 0u];
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
constexpr UsartPadSet afio_uart5_pads(uint8_t code) {
    return afio_uart5_table[code < afio_uart5_8_codes ? code : 0u];
}
constexpr UsartPadSet afio_uart6_pads(uint8_t code) {
    return afio_uart6_table[code < afio_uart5_8_codes ? code : 0u];
}
constexpr UsartPadSet afio_uart7_pads(uint8_t code) {
    return afio_uart7_table[code < afio_uart5_8_codes ? code : 0u];
}
constexpr UsartPadSet afio_uart8_pads(uint8_t code) {
    return afio_uart8_table[code < afio_uart5_8_codes ? code : 0u];
}
constexpr SpiPadSet afio_spi1_pads(uint8_t code) {
    return afio_spi1_table[code < afio_spi1_codes ? code : 0u];
}
constexpr SpiPadSet afio_spi3_pads(uint8_t code) {
    return afio_spi3_table[code < afio_spi3_codes ? code : 0u];
}
constexpr I2sPadSet afio_i2s3_pads(uint8_t code) {
    return afio_i2s3_table[code < afio_spi3_codes ? code : 0u];
}
constexpr I2cPadSet afio_i2c1_pads(uint8_t code) {
    return afio_i2c1_table[code < afio_i2c1_codes ? code : 0u];
}
constexpr CanPadSet afio_can1_pads(uint8_t code) {
    return afio_can1_table[code < afio_can1_codes ? code : 0u];
}
constexpr CanPadSet afio_can2_pads(uint8_t code) {
    return afio_can2_table[code < afio_can2_codes ? code : 0u];
}

/// The pads of a USART's column by INSTANCE number, which is what a
/// transport's pin parameter wants: 1..8, and an instance this part has
/// not got answers with nothing bonded.
constexpr UsartPadSet afio_usart_pads(uint8_t instance, uint8_t code) {
    return !device::has_usart(instance) ? UsartPadSet{} :
           instance == 1 ? afio_usart1_pads(code) :
           instance == 2 ? afio_usart2_pads(code) :
           instance == 3 ? afio_usart3_pads(code) :
           instance == 4 ? afio_uart4_pads(code) :
           instance == 5 ? afio_uart5_pads(code) :
           instance == 6 ? afio_uart6_pads(code) :
           instance == 7 ? afio_uart7_pads(code) :
           instance == 8 ? afio_uart8_pads(code) : UsartPadSet{};
}

/// Whether this part has advanced-control timer `n` - the part table's
/// mask, asked here so that the three extra advanced timers' fields exist
/// exactly where the timer does.
constexpr bool afio_advanced_timer(uint8_t n) {
    return n < 16u && (device::advanced_timer_instances & static_cast<uint16_t>(1U << n)) != 0u;
}

/**
 * Is `code` a column THIS PART can use? Two conditions, both from the
 * documents (see the file header):
 *
 *  - the DEVICE CLASS has the column, and the part has the block the
 *    field moves: a reserved code is not one, and neither is a column
 *    the manual gives to another class;
 *  - the PACKAGE brings out at least one of its pads - all four channel
 *    pads, for the CH32V303's three extra advanced timers: a column with
 *    none of them is a disconnection and not a remap.
 *
 * The oscillator pads' field is the part table's own fact
 * (`device::osc_pads_as_pd0_pd1`): the parts the two documents name for
 * it, the CH32V203K6 whose package is the K8's, and the three smaller
 * CH32V303 - but not the LQFP100, whose PD0 and PD1 are pins of their
 * own and need no remap.
 */
constexpr bool afio_remap_has_code(Remap r, uint8_t code) {
    switch (r) {
        case Remap::spi1:
            return device::spi_count >= 1 && code < afio_spi1_codes &&
                   afio_spi1_pads(code).any_bonded();
        case Remap::i2c1:
            return device::i2c_count >= 1 && code < afio_i2c1_codes &&
                   afio_i2c1_pads(code).any_bonded();
        // Two columns on the CH32V203's classes, four on the CH32V303's
        // (table 10-23's note).
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
        // exists for CH32V20x_D6, CH32F20x_D6". The other two classes'
        // columns are the manual's word, asked of a CH32V303 by the
        // reference suite.
        case Remap::usart3:
            return device::has_usart(3) && code < afio_usart3_codes &&
                   (code == 0 || device::device_class != DeviceClass::v20x_d6) &&
                   afio_usart3_pads(code).any_bonded();
        case Remap::uart4:
            return device::has_usart(4) && code < afio_uart4_codes &&
                   afio_uart4_pads(code).any_bonded();
        case Remap::uart5:
            return device::has_usart(5) && code < afio_uart5_8_codes &&
                   afio_uart5_pads(code).any_bonded();
        case Remap::uart6:
            return device::has_usart(6) && code < afio_uart5_8_codes &&
                   afio_uart6_pads(code).any_bonded();
        case Remap::uart7:
            return device::has_usart(7) && code < afio_uart5_8_codes &&
                   afio_uart7_pads(code).any_bonded();
        case Remap::uart8:
            return device::has_usart(8) && code < afio_uart5_8_codes &&
                   afio_uart8_pads(code).any_bonded();
        case Remap::spi3:
            return (device::spi_instances & (1U << 3)) != 0u && code < afio_spi3_codes &&
                   afio_spi3_pads(code).any_bonded();
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
        case Remap::tim8:
            return afio_advanced_timer(8) && code < afio_tim8_codes &&
                   afio_tim8_pads(code).channels_bonded();
        case Remap::tim9:
            return afio_advanced_timer(9) && code < afio_tim9_codes &&
                   afio_tim9_pads(code).channels_bonded();
        case Remap::tim10:
            return afio_advanced_timer(10) && code < afio_tim10_codes &&
                   afio_tim10_pads(code).channels_bonded();
        case Remap::can1:
            return device::can_count >= 1 && code < afio_can1_codes &&
                   afio_can1_pads(code).any_bonded();
        case Remap::can2:
            return device::can_count >= 2 && code < afio_can2_codes &&
                   afio_can2_pads(code).any_bonded();
        case Remap::pd0_pd1:
            return code < 2 && device::osc_pads_as_pd0_pd1;
        // The channel's other end is the LSI, so code 1 needs no pad;
        // the timer itself is the CH32V203RB's and the 256 KB CH32V303's.
        case Remap::tim5_ch4:
            return device::has_tim5 && code < 2;
        // No pad either: the trigger is wired inside, from the Ethernet's
        // time stamp (code 0) or the USB controller's frame marker (1).
        // MEASURED on the CH32V203C8: the bit is tied to zero there, so
        // the choice belongs to the other classes, whatever 10.3.2.2's
        // note says about the whole series; the reference suite asks the
        // CH32V303.
        case Remap::tim2_itr1:
            return device::general_timer_count >= 1 &&
                   device::device_class != DeviceClass::v20x_d6 && code < 2;
        // The Ethernet's pulse-per-second on PB5, and only where there
        // is an Ethernet to make one.
        case Remap::ptp_pps:
            return device::has_ethernet && code < 2;
        // The Ethernet's MII receive pads: "only available for
        // CH32F207VCT6, CH32V307VCT6, CH32V317VCT6", none of which is
        // this stratum's.
        case Remap::eth:
            return false;
        // TIM8's CH4 and TRGO as the converters' external triggers: the
        // CH32V30x's field (10.3.2.2), and "only supported for products
        // where TIM8 is present" (the note under table 10-40).
        case Remap::adc1_etrginj:
        case Remap::adc1_etrgreg:
            return device::device_class == DeviceClass::v30x_d8 && afio_advanced_timer(8) &&
                   device::adc_count >= 1 && code < 2;
        case Remap::adc2_etrginj:
        case Remap::adc2_etrgreg:
            return device::device_class == DeviceClass::v30x_d8 && afio_advanced_timer(8) &&
                   device::adc_count >= 2 && code < 2;
        // FSMC_NADV on PB7 or silent: the part with the FSMC (the
        // LQFP100) alone.
        case Remap::fsmc_nadv:
            return device::has_fsmc && code < 2;
    }
    return false;
}

/// The field a USART instance's column is selected by.
constexpr Remap afio_usart_remap(uint8_t instance) {
    return instance == 1   ? Remap::usart1
           : instance == 2 ? Remap::usart2
           : instance == 3 ? Remap::usart3
           : instance == 4 ? Remap::uart4
           : instance == 5 ? Remap::uart5
           : instance == 6 ? Remap::uart6
           : instance == 7 ? Remap::uart7
                           : Remap::uart8;
}

/**
 * The code of the column that puts a USART's TX on `tx`, where THIS PART
 * has one - asked by the pad, which is how a board is wired - and 0xFF
 * where it has not. A code means different pads on different classes
 * (UART4's code 0 is PC10 on the CH32V303 and PB0 on the CH32V203C8), so
 * a program that names its pad instead of its code cannot be moved to the
 * wrong one by a change of part: 0xFF is no code at all, and
 * `Afio::remap<R, 0xFF>()` does not compile.
 */
constexpr uint8_t afio_usart_code_for(uint8_t instance, Pad tx) {
    if (instance < 1u || instance > 8u || !tx.valid()) {
        return 0xFFu;
    }
    for (uint8_t code = 0; code < 4u; ++code) {
        if (afio_remap_has_code(afio_usart_remap(instance), code) &&
            afio_usart_pads(instance, code).tx == tx) {
            return code;
        }
    }
    return 0xFFu;
}

/// How many pads of a column this package brings out - what tells a
/// program whether the SIGNALS it needs are there, where
/// afio_remap_has_code() only promises that some pad is.
constexpr bool afio_column_fully_bonded(Remap r, uint8_t code) {
    switch (r) {
        case Remap::spi1:
        case Remap::spi3: {
            const SpiPadSet p = r == Remap::spi1 ? afio_spi1_pads(code) : afio_spi3_pads(code);
            return afio_remap_has_code(r, code) && pad_bonded(p.nss) && pad_bonded(p.sck) &&
                   pad_bonded(p.miso) && pad_bonded(p.mosi);
        }
        case Remap::i2c1: {
            const I2cPadSet p = afio_i2c1_pads(code);
            return pad_bonded(p.scl) && pad_bonded(p.sda);
        }
        case Remap::usart1:
        case Remap::usart2:
        case Remap::usart3:
        case Remap::uart4:
        case Remap::uart5:
        case Remap::uart6:
        case Remap::uart7:
        case Remap::uart8: {
            const UsartPadSet p = r == Remap::usart1   ? afio_usart1_pads(code)
                                  : r == Remap::usart2 ? afio_usart2_pads(code)
                                  : r == Remap::usart3 ? afio_usart3_pads(code)
                                  : r == Remap::uart4  ? afio_uart4_pads(code)
                                  : r == Remap::uart5  ? afio_uart5_pads(code)
                                  : r == Remap::uart6  ? afio_uart6_pads(code)
                                  : r == Remap::uart7  ? afio_uart7_pads(code)
                                                       : afio_uart8_pads(code);
            return pad_bonded(p.tx) && pad_bonded(p.rx);
        }
        case Remap::tim1:
        case Remap::tim8:
        case Remap::tim9:
        case Remap::tim10: {
            const AdvancedTimPads p = r == Remap::tim1   ? afio_tim1_pads(code)
                                      : r == Remap::tim8 ? afio_tim8_pads(code)
                                      : r == Remap::tim9 ? afio_tim9_pads(code)
                                                         : afio_tim10_pads(code);
            return p.channels_bonded();
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
        case Remap::can1:
        case Remap::can2: {
            const CanPadSet p = r == Remap::can1 ? afio_can1_pads(code) : afio_can2_pads(code);
            return afio_remap_has_code(r, code) && pad_bonded(p.rx) && pad_bonded(p.tx);
        }
        case Remap::pd0_pd1:
            return afio_remap_has_code(r, code);
        case Remap::tim5_ch4:
        case Remap::tim2_itr1:
        case Remap::eth:
        case Remap::adc1_etrginj:
        case Remap::adc1_etrgreg:
        case Remap::adc2_etrginj:
        case Remap::adc2_etrgreg:
            return afio_remap_has_code(r, code);
        case Remap::ptp_pps:
            return afio_remap_has_code(r, code) && (code == 0 || pad_bonded(Pad{'B', 5}));
        case Remap::fsmc_nadv:
            return afio_remap_has_code(r, code) && (code == 1 || pad_bonded(Pad{'B', 7}));
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
        if (afio_usart1_high_bit && r == Remap::usart1) {
            write_field(usart1_high_field, static_cast<uint32_t>(code >> 1));
        }
        return true;
    }

    /// The same where the column is a constant: the refusal is a compile
    /// error, which is what a program that has chosen its pads wants.
    template <Remap R, uint8_t Code>
    static void remap() {
        static_assert(afio_remap_has_code(R, Code),
                      "brio Afio::remap: this part has no such column - the code is reserved, "
                      "or the device class does not have it, or the part has not got the block "
                      "the field moves, or this package brings out not one of its pads "
                      "(brio/ch32vx03/afio.hpp's tables, parts/<part>.hpp's facts)");
        write_field(afio_field_of(R), Code);
        if constexpr (afio_usart1_high_bit && R == Remap::usart1) {
            write_field(usart1_high_field, static_cast<uint32_t>(Code >> 1));
        }
    }

    /// What the field holds now - both of USART1's bits on the CH32V303.
    /// PRECONDITION: a field this device class has (afio_field_present()).
    /// One it has not got has no position on this class, and what this
    /// verb reads for it is PCFR1's bit 0 - SPI1's field - so a caller
    /// with a run-time Remap asks afio_field_present() first; the
    /// compile-time face below refuses one outright.
    static uint8_t remap_code(Remap r) {
        clock_on();
        const AfioField f = afio_field_of(r);
        const uint32_t v = f.second ? regs().PCFR2 : regs().PCFR1;
        uint8_t code = static_cast<uint8_t>((v >> f.shift) & mask_of(f.width));
        if (afio_usart1_high_bit && r == Remap::usart1) {
            code = static_cast<uint8_t>(code | (((regs().PCFR2 >> usart1_high_field.shift) & 1u) << 1));
        }
        return code;
    }

    /// The same, for a field named as a constant: one this device class
    /// has not got is a compile error.
    template <Remap R>
    static uint8_t remap_code() {
        static_assert(afio_field_present(R),
                      "brio Afio::remap_code: this device class has no such field - it is one of "
                      "the CH32V303's (brio/ch32vx03/afio.hpp's afio_field_present)");
        return remap_code(R);
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
    /// USART1's high code bit: AFIO_PCFR2 bit 26, the CH32V303's.
    static constexpr AfioField usart1_high_field{true, 26, 1};

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
              afio_tim4_pads(0).etr == afio_tim4_etr);
static_assert(afio_tim5_pad_set.ch1 == Pad{'A', 0} && afio_tim5_pad_set.ch4 == Pad{'A', 3} &&
              !afio_tim5_pad_set.etr.valid());
static_assert(afio_tim8_pads(0).ch1 == Pad{'C', 6} && afio_tim8_pads(0).bkin == Pad{'A', 6} &&
              afio_tim8_pads(0).ch1n == Pad{'A', 7} && afio_tim8_pads(1).ch4 == Pad{'C', 13} &&
              afio_tim8_pads(1).ch3n == Pad{'A', 15} && afio_tim8_pads(1).etr == Pad{'A', 0});
static_assert(afio_tim9_pads(0).ch1 == Pad{'A', 2} && afio_tim9_pads(0).ch4 == Pad{'C', 4} &&
              afio_tim9_pads(1).bkin == Pad{'A', 1} && afio_tim9_pads(1).ch3n == Pad{'B', 2} &&
              afio_tim9_pads(2).ch1n == Pad{'D', 8} && afio_tim9_pads(3).ch4 == Pad{'D', 15});
static_assert(afio_tim10_pads(0).ch1 == Pad{'B', 8} && afio_tim10_pads(0).etr == Pad{'C', 10} &&
              afio_tim10_pads(0).ch1n == Pad{'A', 12} && afio_tim10_pads(1).ch4 == Pad{'C', 15} &&
              afio_tim10_pads(2).etr == Pad{'D', 0} && afio_tim10_pads(3).ch3n == Pad{'E', 5});
static_assert(afio_usart1_pads(0).tx == Pad{'A', 9} && afio_usart1_pads(1).tx == Pad{'B', 6} &&
              afio_usart1_pads(1).rx == Pad{'B', 7} && afio_usart1_pads(1).ck == Pad{'A', 8} &&
              afio_usart1_columns[2].tx == Pad{'B', 15} && afio_usart1_columns[2].rx == Pad{'A', 8} &&
              afio_usart1_columns[3].tx == Pad{'A', 6} && afio_usart1_columns[3].rts == Pad{'C', 5});
static_assert(afio_usart2_pads(0).tx == Pad{'A', 2} && afio_usart2_pads(1).tx == Pad{'D', 5});
static_assert(afio_usart3_pads(0).tx == Pad{'B', 10} && afio_usart3_pads(1).tx == Pad{'C', 10} &&
              afio_usart3_pads(3).rx == Pad{'D', 9});
static_assert(afio_uart5_pads(0).rx == Pad{'D', 2} && afio_uart6_pads(1).tx == Pad{'B', 8} &&
              afio_uart7_pads(1).rx == Pad{'A', 7} && afio_uart8_pads(1).tx == Pad{'A', 14} &&
              afio_uart8_pads(2).rx == Pad{'E', 15});
static_assert(afio_spi1_pads(0).sck == Pad{'A', 5} && afio_spi1_pads(1).mosi == Pad{'B', 5} &&
              afio_spi2_pad_set.sck == Pad{'B', 13});
static_assert(afio_spi3_pads(0).nss == Pad{'A', 15} && afio_spi3_pads(1).sck == Pad{'C', 10} &&
              afio_i2s3_pads(0).mck == Pad{'C', 7} && afio_i2s3_pads(1).sd == Pad{'C', 12} &&
              afio_i2s2_pad_set.mck == Pad{'C', 6});
static_assert(afio_i2c1_pads(0).scl == Pad{'B', 6} && afio_i2c1_pads(1).sda == Pad{'B', 9} &&
              afio_i2c1_pads(1).smba == Pad{'B', 5} && afio_i2c2_pad_set.scl == Pad{'B', 10});
static_assert(afio_can1_pads(0).rx == Pad{'A', 11} && afio_can1_pads(2).tx == Pad{'B', 9} &&
              afio_can1_pads(3).tx == Pad{'D', 1} && afio_can2_pads(1).tx == Pad{'B', 6});
static_assert(afio_uart4_pads(0).tx == (device::device_class != DeviceClass::v20x_d6
                                            ? Pad{'C', 10} : Pad{'B', 0}) &&
              afio_uart4_pads(1).tx == (device::device_class != DeviceClass::v20x_d6
                                            ? Pad{'B', 0} : Pad{'A', 5}));

// The codes a package decides: TIM1's full column is port E, which only
// the LQFP100 bonds, and TIM1's code 10 is reserved everywhere; USART2's
// remap is not the D6's; CAN2 and the Ethernet's pads are nobody's here.
static_assert(!afio_remap_has_code(Remap::tim1, 2));
static_assert(afio_remap_has_code(Remap::tim1, 3) == device::has_port('E'));
static_assert(!afio_remap_has_code(Remap::usart2, 1) ||
              device::device_class != DeviceClass::v20x_d6);
static_assert(!afio_remap_has_code(Remap::tim3, 1) && !afio_remap_has_code(Remap::can1, 1));
static_assert(!afio_remap_has_code(Remap::can2, 0) && !afio_remap_has_code(Remap::eth, 1));

// The multiplexer's codes are the chapter's.
static_assert(exti_port_code('A') == 0 && exti_port_code('E') == 4 && exti_port_code('F') == 0xFF);
static_assert(exti_port_letter(3) == 'D' && exti_port_letter(5) == '\0');

} // namespace brio
