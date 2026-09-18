/*
 * can.hpp
 *
 * The CAN vocabulary the controllers share: the classic frame, the bit
 * timing in the units a human states it in with the search that finds
 * it, the protocol's own error codes and error states, and the two
 * counters every node keeps. NO BUS AO, NO ARBITER POLICY AND NO
 * CONCEPT - docs/design/can.md says why: a transmission's completion is
 * not a reply, reception is routing, and a contract for either would
 * today be one controller's element layout with a util name on it.
 *
 * BORN AT THE THIRD CONTROLLER, NOT THE SECOND. Two bxCAN strata (the
 * STM32F4's and the CH32V203's, the same IP under two vendors' names)
 * and one M_CAN (the STM32G0's FDCAN) are what this file was written
 * against: what all three say the same way is here, what each says its
 * own way stays in its stratum. The frame is the classic one, eight
 * bytes; the FD superset stays the M_CAN stratum's until a second FD
 * controller exists to write it against. The filters are not here at
 * all: a bank of two registers with four shapes and a list of elements
 * with a range/dual/mask type are two models of the same idea, and a
 * shared type would be one of them wearing the other's name.
 *
 * THE IDENTIFIER IS NATURAL AND RIGHT-ALIGNED, 0..0x7FF standard and
 * 0..0x1FFFFFFF extended, whatever a register does with it (a bxCAN
 * mailbox stores the standard identifier in bits 31:21, an M_CAN
 * element in ID[28:18]): the driver shifts, the program never does.
 * THE LENGTH IS BYTES AND NOT A DLC: the DLC is the wire's coding of it,
 * computed on the way out, and a classic frame carries at most eight.
 *
 * THE TIMING IS STATED IN HUMAN UNITS - a prescaler that DIVIDES, three
 * segments that COUNT quanta, one less than each in the registers -
 * and the limits a controller's registers put on them travel as a
 * value, `CanTimingLimits`, which each stratum states from its own
 * chapter. The search is EXACT: a bit rate that is a fraction of a per
 * cent off is a bus that works between two nodes of one crystal and
 * fails when the third joins, so a clock that cannot divide into the
 * rate yields an empty timing, never a rounded one.
 */

#pragma once

#include <stdint.h>

namespace brio {

// =============================================================================
// The frame
// =============================================================================

/// The widest identifier each format carries: eleven bits standard,
/// twenty-nine extended.
inline constexpr uint32_t can_std_id_max = 0x7FFu;
inline constexpr uint32_t can_ext_id_max = 0x1FFFFFFFu;

/// The most data bytes a classic frame carries.
inline constexpr uint8_t can_max_length = 8;

/**
 * A classic CAN message, in or out. `filter_index` and `timestamp` are
 * filled on RECEPTION only - which filter admitted the frame, and the
 * controller's counter at its start of frame, in the unit and under the
 * conditions the stratum's document states (a bxCAN stamps only in its
 * time-triggered mode, an M_CAN whenever its counter runs) - and are
 * ignored on transmission.
 */
struct CanFrame {
    uint32_t id = 0;
    bool extended = false;
    bool remote = false;
    uint8_t length = 0;
    uint8_t data[can_max_length] = {};
    uint8_t filter_index = 0;
    uint16_t timestamp = 0;
};

constexpr bool can_frame_valid(const CanFrame& f) {
    return f.length <= can_max_length && f.id <= (f.extended ? can_ext_id_max : can_std_id_max);
}

/// How many BIT TIMES a classic data frame of `length` bytes occupies on
/// the wire before stuffing: 44 + 8N with a standard identifier, 64 + 8N
/// with an extended one, end of frame included and the three-bit
/// inter-frame space not. Bit stuffing adds up to one bit in five to the
/// stuffable part, so a measured frame is always this long or longer.
constexpr uint16_t can_frame_bits(uint8_t length, bool extended) {
    return static_cast<uint16_t>((extended ? 64u : 44u) + 8u * length);
}

// =============================================================================
// The bit timing
// =============================================================================

/**
 * The bit time, in the units a human states it in and not in the
 * registers' off-by-ones: `brp` is the prescaler as a DIVIDER, `ts1` and
 * `ts2` are the two bit segments as quanta COUNTS, `sjw` the
 * resynchronization jump width in quanta. `brp == 0` means "no timing" -
 * what the search returns when the clock cannot make the rate exactly.
 */
struct CanTiming {
    uint16_t brp = 0;
    uint8_t ts1 = 0;
    uint8_t ts2 = 0;
    uint8_t sjw = 1;
};

/**
 * What a controller's registers allow of a timing, stated by the stratum
 * from its chapter: the widest divider, the widest segments and jump,
 * and the quanta a bit may hold (the sum 1 + TS1 + TS2, which a chapter
 * may cap below what the three fields could reach).
 */
struct CanTimingLimits {
    uint16_t brp_max = 0;
    uint8_t ts1_max = 0;
    uint8_t ts2_max = 0;
    uint8_t sjw_max = 0;
    uint8_t quanta_min = 0;
    uint8_t quanta_max = 0;
};

/// A limits value that can bound a search at all: every ceiling at least
/// one, a bit of three quanta or more, the ceiling not below the floor.
constexpr bool can_timing_limits_valid(const CanTimingLimits& l) {
    return l.brp_max >= 1u && l.ts1_max >= 1u && l.ts2_max >= 1u && l.sjw_max >= 1u &&
           l.quanta_min >= 3u && l.quanta_max >= l.quanta_min;
}

/// Time quanta in one bit: the synchronization segment plus the two bit
/// segments.
constexpr uint16_t can_timing_quanta(const CanTiming& t) {
    return static_cast<uint16_t>(1u + t.ts1 + t.ts2);
}

/// Whether a timing fits a controller's registers and its chapter's
/// window, the jump width never wider than the second segment.
constexpr bool can_timing_fits(const CanTiming& t, const CanTimingLimits& l) {
    return t.brp >= 1u && t.brp <= l.brp_max && t.ts1 >= 1u && t.ts1 <= l.ts1_max &&
           t.ts2 >= 1u && t.ts2 <= l.ts2_max && t.sjw >= 1u && t.sjw <= l.sjw_max &&
           t.sjw <= t.ts2 && can_timing_quanta(t) >= l.quanta_min &&
           can_timing_quanta(t) <= l.quanta_max;
}

/// The bit rate a timing produces from a controller clock of `clock_hz`.
constexpr uint32_t can_bitrate_of(uint32_t clock_hz, const CanTiming& t) {
    const uint32_t div = static_cast<uint32_t>(t.brp) * can_timing_quanta(t);
    return div == 0u ? 0u : clock_hz / div;
}

/// Where in the bit the sample point falls, in per mille - the number a
/// CAN bus is designed around (87.5% is the CiA recommendation for the
/// rates below 800 kbit/s and what most tools default to).
constexpr uint16_t can_sample_point_of(const CanTiming& t) {
    const uint16_t n = can_timing_quanta(t);
    return n == 0u ? 0u : static_cast<uint16_t>((1000u * (1u + t.ts1)) / n);
}

/**
 * THE TIMING SEARCH, at compile time when its arguments are: the EXACT
 * `bitrate_hz` from `clock_hz` within `limits`, with the sample point as
 * close to `sample_permille` as the segments allow.
 *
 * Only exact divisions are considered - an empty timing comes back when
 * `clock_hz` is not a whole multiple of `bitrate_hz` times some legal
 * quanta count. Quanta counts are tried from the widest down (a wider
 * bit resolves the sample point more finely), and the first count whose
 * best sample point beats every wider one's wins; the jump width is the
 * second segment capped at the controller's widest jump. A stratum wraps
 * this in its own `can_timing_for(clock_hz, bitrate_hz, sample)` with
 * its limits bound, which is the verb a program calls.
 */
constexpr CanTiming can_timing_search(uint32_t clock_hz, uint32_t bitrate_hz,
                                      const CanTimingLimits& limits,
                                      uint16_t sample_permille = 875) {
    CanTiming best{};
    if (clock_hz == 0u || bitrate_hz == 0u || !can_timing_limits_valid(limits)) {
        return best;
    }
    uint32_t best_err = 0xFFFFFFFFu;
    for (uint8_t n = limits.quanta_max; n >= limits.quanta_min; --n) {
        const uint32_t denom = bitrate_hz * n;
        if (denom == 0u || clock_hz % denom != 0u) {
            continue;
        }
        const uint32_t brp = clock_hz / denom;
        if (brp < 1u || brp > limits.brp_max) {
            continue;
        }
        for (uint8_t ts1 = 1; ts1 <= limits.ts1_max; ++ts1) {
            if (ts1 + 1u > n) {
                break;
            }
            const uint32_t ts2 = n - 1u - ts1;
            if (ts2 < 1u || ts2 > limits.ts2_max) {
                continue;
            }
            const uint32_t sp = (1000u * (1u + ts1)) / n;
            const uint32_t err = sp > sample_permille ? sp - sample_permille : sample_permille - sp;
            if (err < best_err) {
                best_err = err;
                best.brp = static_cast<uint16_t>(brp);
                best.ts1 = ts1;
                best.ts2 = static_cast<uint8_t>(ts2);
                best.sjw = static_cast<uint8_t>(ts2 < limits.sjw_max ? ts2 : limits.sjw_max);
            }
        }
    }
    return best;
}

// =============================================================================
// Errors and the error state
// =============================================================================

/**
 * The last error code the protocol engine saw on the bus - the eight
 * codes ISO 11898-1's controllers all publish in one three-bit field
 * (bxCAN's ESR.LEC, M_CAN's PSR.LEC and DLEC). The seventh is the one
 * SOFTWARE writes, so that a later read can tell nothing has happened
 * since; an M_CAN writes it back on every read of its own.
 */
enum class CanError : uint8_t {
    none = 0,
    stuff = 1,
    form = 2,
    acknowledge = 3,
    bit_recessive = 4,   ///< wanted recessive, monitored dominant
    bit_dominant = 5,    ///< wanted dominant, monitored recessive
    crc = 6,
    no_change = 7,
};

/// The node's error state, the protocol's fault confinement as ONE
/// observable: the two counters below the warning limit, at or above it
/// (warning), a counter past 127 (passive: the node acknowledges but no
/// longer signals active error flags), or the transmit counter past 255
/// (bus-off: the node is off the wire until its recovery sequence).
enum class CanErrorState : uint8_t {
    active = 0,
    warning = 1,
    passive = 2,
    bus_off = 3,
};

/// The state from the three flags a controller publishes, the deepest
/// standing one winning - bus-off implies passive implies warning.
constexpr CanErrorState can_error_state(bool warning, bool passive, bool bus_off) {
    if (bus_off) {
        return CanErrorState::bus_off;
    }
    if (passive) {
        return CanErrorState::passive;
    }
    return warning ? CanErrorState::warning : CanErrorState::active;
}

/// The two counters of fault confinement, as the controller reports them.
struct CanErrorCounters {
    uint8_t transmit = 0;   ///< TEC: past 255 the node is bus-off
    uint8_t receive = 0;    ///< REC: past 127 the node is error passive
};

} // namespace brio
