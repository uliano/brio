/*
 * afio.hpp
 *
 * The alternate-function block of the CH32X035 (RM 8.2.4, 8.3.2): the
 * remap register, the external-interrupt port multiplexer and the debug
 * port's own field - everything in chapter 8 that is not a pad.
 *
 * A REMAP IS A COLUMN, NOT A PIN. AFIO_PCFR1 holds one field per
 * peripheral, and the field's value selects a whole COLUMN of the
 * chapter's descriptions: a USART's five signals move together, TIM2's
 * nine do. The FIELDS of every peripheral are named here (Remap), so the
 * register is decoded whole; the COLUMNS - which pad each code puts each
 * signal on - are stated as constexpr data for the USARTs, the one block
 * with a driver in this stratum, and arrive with their drivers for SPI1,
 * I2C1, TIM1..TIM3 and the PIOC. A driver names a code and gets its pads
 * from the same source that programs the register - the sibling strata's
 * arrangement.
 *
 * A COLUMN'S CODES ARE THE FIELD'S. USART2's field is three bits and its
 * four upper codes (1xx) are one column; USART4's are two columns, 1x0
 * and 1x1 (8.3.2.1). Every code of every field is a column the manual
 * describes, so a code is refused only when it does not fit its field -
 * or, for a USART, when not one pad of the column is bonded by this
 * package, which is a disconnection and not a remap.
 *
 * THE DEBUG PORT IS NOT A REMAP. SW_CFG (PCFR1 26:24) hands PC18 and PC19
 * back to GPIO, and a program that does that has lost its probe until the
 * next reset - the two-wire port is how the image got there. The field is
 * READ here by default and written only through a verb spelled long
 * enough that nobody reaches it by accident. Two USART columns land on
 * those pads - USART3's code 1 and USART4's 1x0 for its RX - and the
 * transport refuses them while the port is the probe's (usart.hpp).
 *
 * THE EXTI MULTIPLEXER. AFIO_EXTICR1 and EXTICR2 say which PORT feeds
 * each of the twenty-four pin lines, two bits a line, and the codes are
 * 00 port A, 10 port B, 11 port C - 01 reserved (8.3.2.2). The verbs are
 * here because the register is; the EXTI itself has no driver in this
 * stratum.
 *
 * AFIO_CTLR (8.3.2.4) - the USB and USB PD pads' pull-ups, source
 * voltages and comparators, and four pads' input filters - is read here
 * and written by nothing: it belongs to the USB chapters.
 *
 * The AFIO's clock gate is closed out of reset and every verb opens it,
 * the way pin.hpp's opens a port's.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/device.hpp"
#include "ch32x035/pin.hpp"

namespace brio {

// =============================================================================
// The fields of AFIO_PCFR1 (8.3.2.1)
// =============================================================================

/// One remap field per peripheral, in the register's order.
enum class Remap : uint8_t { spi1, i2c1, usart1, usart2, usart3, usart4, tim1, tim2, tim3, pioc };

/// Where a field sits and how wide it is.
struct AfioField {
    uint8_t shift;
    uint8_t width;
};

constexpr AfioField afio_field_of(Remap r) {
    switch (r) {
        case Remap::spi1:   return {0, 2};
        case Remap::i2c1:   return {2, 3};
        case Remap::usart1: return {5, 2};
        case Remap::usart2: return {7, 3};
        case Remap::usart3: return {10, 2};
        case Remap::usart4: return {12, 3};
        case Remap::tim1:   return {15, 3};
        case Remap::tim2:   return {18, 3};
        case Remap::tim3:   return {21, 2};
        case Remap::pioc:   return {23, 1};
    }
    return {0, 0};
}

/// The field of a USART instance.
constexpr Remap afio_usart_remap(uint8_t n) {
    return n == 1 ? Remap::usart1 : n == 2 ? Remap::usart2 : n == 3 ? Remap::usart3 : Remap::usart4;
}

// =============================================================================
// The USARTs' columns (8.3.2.1, and the datasheet's table 2-1 names the
// same pads with the code after the signal: TX2_2 is USART2's TX under
// code 10)
// =============================================================================

/// A USART's five signals. A column with no CK - USART3's code 3 - leaves
/// it invalid.
struct UsartPadSet {
    Pad tx, rx, ck, cts, rts;

    constexpr bool any_bonded() const {
        return pad_bonded(tx) || pad_bonded(rx) || pad_bonded(ck) || pad_bonded(cts) ||
               pad_bonded(rts);
    }
};

/// The column a code selects, for an instance 1..4; a code outside the
/// field is an empty column.
constexpr UsartPadSet afio_usart_pads(uint8_t n, uint8_t code) {
    if (n == 1) {
        switch (code) {
            case 0: return {{'B', 10}, {'B', 11}, {'B', 9},  {'C', 16}, {'C', 17}};
            case 1: return {{'A', 10}, {'A', 11}, {'B', 9},  {'C', 16}, {'C', 17}};
            case 2: return {{'B', 10}, {'B', 11}, {'B', 5},  {'A', 9},  {'A', 8}};
            case 3: return {{'A', 7},  {'B', 2},  {'B', 12}, {'A', 13}, {'A', 14}};
            default: return {};
        }
    }
    if (n == 2) {
        switch (code) {
            case 0: return {{'A', 2},  {'A', 3},  {'A', 4},  {'A', 0},  {'A', 1}};
            case 1: return {{'A', 20}, {'A', 19}, {'A', 23}, {'A', 1},  {'A', 2}};
            case 2: return {{'A', 15}, {'A', 16}, {'A', 22}, {'A', 17}, {'A', 21}};
            case 3: return {{'C', 0},  {'C', 1},  {'B', 20}, {'C', 2},  {'C', 3}};
            case 4: case 5: case 6: case 7:
                    return {{'A', 15}, {'A', 16}, {'A', 22}, {'A', 17}, {'C', 3}};
            default: return {};
        }
    }
    if (n == 3) {
        switch (code) {
            case 0: return {{'B', 3},  {'B', 4},  {'B', 5}, {'B', 6},  {'B', 7}};
            case 1: return {{'C', 18}, {'C', 19}, {'B', 5}, {'B', 6},  {'B', 7}};
            case 2: return {{'A', 18}, {'B', 14}, {'B', 8}, {'A', 3},  {'A', 4}};
            case 3: return {{'B', 16}, {'B', 17}, {},       {'B', 18}, {'B', 19}};
            default: return {};
        }
    }
    if (n == 4) {
        switch (code) {
            case 0: return {{'B', 0},  {'B', 1},  {'B', 2}, {'B', 15}, {'A', 8}};
            case 1: return {{'A', 5},  {'A', 9},  {'A', 6}, {'A', 7},  {'B', 21}};
            case 2: return {{'C', 16}, {'C', 17}, {'B', 2}, {'B', 15}, {'A', 8}};
            case 3: return {{'B', 9},  {'A', 10}, {'B', 8}, {'A', 14}, {'A', 13}};
            case 4: case 6:
                    return {{'B', 13}, {'C', 19}, {'A', 8}, {'A', 5},  {'A', 6}};
            case 5: case 7:
                    return {{'C', 17}, {'C', 16}, {'B', 2}, {'B', 15}, {'A', 8}};
            default: return {};
        }
    }
    return {};
}

/// Whether a code is one of this field's on this part: it fits the field,
/// and - for a USART - at least one pad of its column is bonded.
constexpr bool afio_remap_has_code(Remap r, uint8_t code) {
    const AfioField f = afio_field_of(r);
    if (code >= (1u << f.width)) {
        return false;
    }
    switch (r) {
        case Remap::usart1: return afio_usart_pads(1, code).any_bonded();
        case Remap::usart2: return afio_usart_pads(2, code).any_bonded();
        case Remap::usart3: return afio_usart_pads(3, code).any_bonded();
        case Remap::usart4: return afio_usart_pads(4, code).any_bonded();
        default: return true;
    }
}

// =============================================================================
// The block
// =============================================================================

/**
 * AFIO, monostate: the remaps, the EXTI multiplexer and the debug port's
 * own field. Every verb opens the block's clock gate first -
 * RCC_APB2PCENR bit 0, closed out of reset - because a register behind a
 * shut gate is not there: a read of it answers with the last word the bus
 * carried (measured on port A's INDR, whose gate is a bit of the same
 * register) and a write into it lands nowhere.
 */
struct Afio {
    Afio() = delete;

    static void clock_on() { Rcc::enable(Bus::pb2, rcc_pb2_afio); }

    static AfioRegs& regs() { return *afio(); }

    // ---- the remaps --------------------------------------------------------

    /// Select a column. False - and NOTHING WRITTEN - when this part has no
    /// such column (afio_remap_has_code).
    static bool remap(Remap r, uint8_t code) {
        if (!afio_remap_has_code(r, code)) {
            return false;
        }
        write_field(afio_field_of(r), code);
        return true;
    }

    /// The same where the column is a constant: the refusal is a compile
    /// error.
    template <Remap R, uint8_t Code>
    static void remap() {
        static_assert(afio_remap_has_code(R, Code),
                      "brio Afio::remap: no such column on this part - the code does not fit "
                      "the field, or this package bonds not one of the column's pads "
                      "(brio/ch32x035/afio.hpp, parts/<part>.hpp)");
        write_field(afio_field_of(R), Code);
    }

    /// What the field holds now.
    static uint8_t remap_code(Remap r) {
        clock_on();
        const AfioField f = afio_field_of(r);
        return static_cast<uint8_t>((regs().PCFR1 >> f.shift) & ((1UL << f.width) - 1u));
    }

    // ---- the debug port (PCFR1 26:24) --------------------------------------

    /// SW_CFG as it stands: 0xx while the two-wire port is alive, 100 once
    /// it has been given up (every other code "invalid", 8.3.2.1).
    static uint8_t debug_config() {
        clock_on();
        return static_cast<uint8_t>((regs().PCFR1 >> 24) & 0x7u);
    }

    /// Are PC18/PC19 still the debug port's?
    static bool debug_port_enabled() { return (debug_config() & 0x4u) == 0u; }

    /**
     * SW_CFG = 100: THE DEBUG PORT OFF, PC18 and PC19 plain pads, until the
     * next reset. A program that calls this has lost its probe until the
     * board reboots - and one that calls it early in every boot has lost
     * it for good unless the boot reads a condition first. Spelled long on
     * purpose.
     */
    static void disable_debug_port_until_reset() { write_field({24, 3}, 4); }

    // ---- the EXTI multiplexer (AFIO_EXTICR1, EXTICR2) ------------------------

    /**
     * Point EXTI line `line` (0..23) at port `port`'s pin of the same
     * number. False for a line outside the twenty-four, a letter this
     * series has no port for, or a port this package bonds no pin of.
     */
    static bool exti_source(uint8_t line, char port) {
        const uint8_t code = port == 'A' ? afio_exti_port_a :
                             port == 'B' ? afio_exti_port_b :
                             port == 'C' ? afio_exti_port_c : 0xFFu;
        if (line >= exti_pad_lines || code == 0xFFu || !device::has_port(port)) {
            return false;
        }
        clock_on();
        const uint32_t shift = static_cast<uint32_t>(line & 15u) * 2u;
        volatile uint32_t& reg = regs().EXTICR[line >> 4];
        reg = (reg & ~(0x3UL << shift)) | (static_cast<uint32_t>(code) << shift);
        return true;
    }

    /// Which port feeds that line now, as a letter; 0 for a line outside
    /// the twenty-four or the reserved code.
    static char exti_source(uint8_t line) {
        if (line >= exti_pad_lines) {
            return '\0';
        }
        clock_on();
        const uint32_t shift = static_cast<uint32_t>(line & 15u) * 2u;
        const uint32_t code = (regs().EXTICR[line >> 4] >> shift) & 0x3u;
        return code == afio_exti_port_a ? 'A' :
               code == afio_exti_port_b ? 'B' :
               code == afio_exti_port_c ? 'C' : '\0';
    }

    // ---- AFIO_CTLR, read only ----------------------------------------------

    /// The USB and USB PD pads' control register as it stands (8.3.2.4).
    static uint32_t control() {
        clock_on();
        return regs().CTLR;
    }

private:
    static void write_field(AfioField f, uint32_t value) {
        clock_on();
        const uint32_t mask = ((1UL << f.width) - 1u) << f.shift;
        regs().PCFR1 = (regs().PCFR1 & ~mask) | ((value << f.shift) & mask);
    }
};

} // namespace brio
