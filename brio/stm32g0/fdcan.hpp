/*
 * fdcan.hpp
 *
 * The STM32G0's FD controller area network (RM0444 ch. 36) - the Bosch
 * M_CAN core ST ships on the G0B1/G0C1 class and nowhere else in this
 * family, in the resource/task split this stratum uses everywhere:
 *
 *   Fdcan<1>, Fdcan<2>   the two CAN modules - every register of ch. 36,
 *                        the message RAM they read and write, the
 *                        INIT/CCE state machine that gates the whole
 *                        configuration, the filters, the two Rx FIFOs,
 *                        the three Tx buffers, the Tx event FIFO, the
 *                        timestamp and timeout counters, the error
 *                        machine and the two interrupt lines;
 *   FdcanPad<sel>        one pad handed to FDCANn_RX or FDCANn_TX.
 *
 *   using Can = brio::Fdcan<1>;
 *   constexpr brio::FdcanConfig cfg{
 *       .nominal = *brio::fdcan_bit_timing_for(64'000'000, 500'000, 875),
 *       .mode = brio::FdcanMode::internal_loop_back,
 *       .standard_filters = 1,
 *   };
 *   Can::enter<cfg>();                       // clock, reset, RAM, timing
 *   Can::standard_filter(0, {.type = brio::FdcanFilterType::classic,
 *                            .action = brio::FdcanFilterAction::store_fifo0,
 *                            .id1 = 0x123, .id2 = 0x7FF});
 *   Can::start();                            // INIT cleared, on the bus
 *
 * NO TASK IS BUILT HERE. A CAN bus AO, a frame vocabulary shared with
 * another architecture's controller and the policy that goes with them
 * are a util design question, and this stratum must not open it with one
 * implementation in hand: `FdcanFrame` below is deliberately
 * TARGET-LOCAL and says so. The avrdx/rtc.hpp precedent - a task is born
 * with its first user.
 *
 * SEVEN FACTS THAT SHAPE EVERYTHING BELOW.
 *
 * 1. THE CHAPTER IS ABSENT FROM MOST OF THE FAMILY. Table 1 gives FDCAN1
 *    and FDCAN2 to the G0B1/G0C1 alone; the G071 and G031 headers
 *    declare no base, no struct and no interrupt enumerator. So the
 *    REGISTER-FACING half of this file is compiled only where the header
 *    declares the block, and `Fdcan<n>` simply does not exist on a part
 *    without one (the avrdx/opamp.hpp precedent). The pure ARITHMETIC -
 *    the bit-timing chooser, the DLC coding, the element codecs - is
 *    outside that gate and compiles everywhere, because it is a property
 *    of the CAN protocol and not of a peripheral.
 *
 * 2. ONE CLOCK, ONE RESET, ONE DIVIDER FOR THE WHOLE SUBSYSTEM. Figure
 *    392 draws the two modules inside one block: RCC_APBENR1.FDCANEN
 *    clocks both, RCC_APBRSTR1.FDCANRST RESETS BOTH (so `reset()` asked
 *    for through FDCAN2 takes FDCAN1's registers down with it - measured,
 *    and said here rather than discovered), and the one CKDIV register
 *    lives at the CONFIGURATION block's address, is FDCAN1's by
 *    36.4.37's own note, and divides the kernel clock for both
 *    instances. The kernel clock itself is chosen in RCC_CCIPR2 - a
 *    DIFFERENT register from the CCIPR every other multiplexer of this
 *    stratum uses, which is why clock.hpp has a second verb for it.
 *
 * 3. INIT AND CCE ARE THE WHOLE CONFIGURATION GATE (36.3.4). A
 *    protected register is writable only with CCCR.INIT and CCCR.CCE
 *    both set, so every configuring verb here is a REFUSAL outside that
 *    state - false, and nothing written - the way usart.hpp's UE-
 *    protected verbs are. Three corollaries the chapter states and this
 *    file enforces: CCE can only be set while INIT is set and is cleared
 *    by hardware when INIT is cleared; setting CCE RESETS eight status
 *    registers and holds both handlers idle; and TXBAR/TXBCR are
 *    writable only with CCE CLEAR, which is why requesting a
 *    transmission is not a configuring verb. INIT itself crosses a clock
 *    domain: 36.4.6's note says to read it back before writing it again,
 *    so `init_mode()` waits, bounded, and reports.
 *
 * 4. THE MESSAGE RAM IS A FIXED MAP AND ITS CONTENT AT RESET IS
 *    UNDEFINED. Figure 399: 212 words per instance - 28 standard filter
 *    words, 8 extended filter elements of two words, two Rx FIFOs of
 *    three 18-word elements, a three-element Tx event FIFO of two words
 *    and three 18-word Tx buffers - with FDCAN2's map starting where
 *    FDCAN1's ends (36.3.6's own arithmetic: +0x350). Nothing in the
 *    silicon clears it, so `enter()` zeroes the instance's 212 words
 *    before it configures anything. The RAM is ordinary CPU-addressable
 *    memory, which is what makes an element codec possible at all - and
 *    what makes 36.3.7's warning real: reading an element out of turn is
 *    legal, acknowledging it is not.
 *
 * 5. TEST MODE IS THE ONLY BENCH THIS BOARD HAS. With no transceiver on
 *    the desk the loop-back modes are the experiment: EXTERNAL loop-back
 *    (TEST.LBCK alone) feeds the transmitter back to the receiver
 *    INTERNALLY, ignores acknowledge errors and STILL DRIVES THE TX PAD -
 *    so a frame is countable on a pin - while INTERNAL loop-back
 *    (LBCK + CCCR.MON) disconnects the RX pin and holds TX recessive.
 *    TEST.TX = 01 puts the SAMPLE POINT on the TX pad, which turns a
 *    GPIO edge counter into a bit-rate meter. Writing FDCAN_TEST at all
 *    needs CCCR.TEST, and clearing CCCR.TEST returns the whole register
 *    to its reset value (36.4.4).
 *
 * 6. THE INTERRUPT LINE SELECT IS SEVEN GROUPS AND NOT THIRTY FLAGS.
 *    On this part FDCAN_ILS carries seven bits, each naming a GROUP of
 *    IR flags (36.4.17) - the classic M_CAN's per-flag ILS is not what
 *    this silicon has - so `interrupt_line()` speaks groups and
 *    `fdcan_group_of()` says which group an IR flag belongs to. Both
 *    lines land on a SHARED VECTOR: table 61 puts fdcan1_intr0_it and
 *    fdcan2_intr0_it on TIM16's line and both intr1_it on TIM17's, so
 *    ONE handler serves two peripherals and two instances. The ISR
 *    BODIES therefore obey the stratum's shared-vector contract - each
 *    returns the mask IT served and clears exactly that - and an app's
 *    handler calls every body that can be on the line.
 *
 * 7. A REFUSAL IS ALWAYS A `false` WITH NOTHING WRITTEN, and every
 *    configuring verb has a compile-time twin whose static_assert names
 *    the rule that was broken. `fdcan_config_valid()` is the one place
 *    the chapter's configuration rules are written down.
 *
 * CONCURRENCY. The registers are plain APB reads and writes with no
 * read-modify-write hazard between a handler and the loop EXCEPT in
 * CCCR, IE, ILS and the two Tx interrupt-enable registers, which are
 * read-modify-written here; configure in setup, and treat IR (rc_w1) and
 * the acknowledge registers as the only registers a handler touches.
 * FIVE REGISTERS DESTROY WHAT THEY REPORT WHEN READ - PSR sets LEC and
 * DLEC to 7 and clears RESI/RBRS/REDL/PXE, ECR clears CEL - so
 * `status()` and `error_counters()` read each of them EXACTLY ONCE per
 * call and hand back the whole decoded picture.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <optional>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "util/clock.hpp"

namespace brio {

// =============================================================================
// The protocol arithmetic - present on every part of the family
// =============================================================================

/// RCC_CCIPR2.FDCANSEL (5.4.22). Only `pclk` is reachable on this
/// stratum: `Clock<>` builds SYSCLK from HSI16 or the PLL's R output and
/// configures neither the PLL's Q output nor HSE, so asking for either
/// of the other two would select a clock nothing has started.
enum class FdcanClock : uint8_t {
    pclk = 0,   ///< the APB clock - what `Clock<>::pclk_hz` names
    pllq = 1,   ///< PLLQCLK - not built by clock.hpp, refused here
    hse = 2,    ///< HSE - no crystal is fitted on a Nucleo-64, refused here
};

/// FDCAN_CKDIV.PDIV (36.4.37): the SUBSYSTEM's kernel-clock divider,
/// /1 and then the even numbers to /30. The value is the code, the
/// divisor is `fdcan_divider_value()`.
enum class FdcanClockDivider : uint8_t {
    div1 = 0, div2 = 1, div4 = 2, div6 = 3, div8 = 4, div10 = 5,
    div12 = 6, div14 = 7, div16 = 8, div18 = 9, div20 = 10, div22 = 11,
    div24 = 12, div26 = 13, div28 = 14, div30 = 15,
};

/// What the code divides by. Code 0 is /1; every other code n is /2n.
constexpr uint8_t fdcan_divider_value(FdcanClockDivider d) {
    const uint8_t code = static_cast<uint8_t>(d);
    return code == 0u ? 1u : static_cast<uint8_t>(2u * code);
}

/// The operating mode a configuration asks for (36.3.4). `restricted`
/// is ASM alone; the two loop-backs are TEST.LBCK, the internal one
/// with CCCR.MON beside it.
enum class FdcanMode : uint8_t {
    normal,               ///< event-driven CAN, the reset behaviour
    bus_monitor,          ///< MON: receives and acknowledges nothing, TX held recessive
    restricted,           ///< ASM: receives and acknowledges, never sends a dominant bit
    external_loop_back,   ///< TEST.LBCK: own frames received, ack errors ignored, TX PAD DRIVEN
    internal_loop_back,   ///< TEST.LBCK + MON: RX pin disconnected, TX pad held recessive
};

/// CCCR.FDOE and CCCR.BRSE, table 213's two knobs as one choice.
enum class FdcanFd : uint8_t {
    off = 0,                    ///< classic CAN only; FDF and BRS in a Tx element are ignored
    on = 1,                     ///< FDOE: FD frames, one bit rate throughout
    on_with_bit_rate_switch = 2 ///< FDOE + BRSE: a Tx element's BRS switches to the data timing
};

/// FDCAN_TEST.TX[1:0] (36.4.4): what drives the FDCANn_TX pad.
enum class FdcanTxPin : uint8_t {
    core = 0,           ///< the CAN core's serial output (the reset value)
    sample_point = 1,   ///< THE SAMPLE POINT SIGNAL - the bit-rate meter of a board with no bus
    dominant = 2,
    recessive = 3,
};

/// FDCAN_TSCC.TSS. Codes 00 and 11 both mean "always zero" (36.4.8).
enum class FdcanTimestamp : uint8_t {
    off = 0,            ///< the counter reads 0x0000
    internal = 1,       ///< incremented every TCP CAN bit times
    external_tim3 = 2,  ///< tim3_cnt[15:0] - the one external source this part wires up
};

/// FDCAN_TOCC.TOS: what starts and reloads the timeout counter.
enum class FdcanTimeoutMode : uint8_t {
    continuous = 0,
    tx_event_fifo = 1,
    rx_fifo0 = 2,
    rx_fifo1 = 3,
};

/// RXGFC.ANFS / RXGFC.ANFE: what happens to a frame no filter matched.
/// Codes 10 and 11 both reject (36.4.19).
enum class FdcanNonMatching : uint8_t {
    fifo0 = 0,
    fifo1 = 1,
    reject = 2,
};

/// A standard filter element's SFT (table 221). `disabled` is 11, which
/// the chapter says behaves exactly like SFEC = 000.
enum class FdcanFilterType : uint8_t {
    range = 0,     ///< id1 .. id2
    dual = 1,      ///< id1 or id2
    classic = 2,   ///< id1 = value, id2 = mask
    disabled = 3,
};

/// An extended filter element's EFT (table 223). Its code 11 is not
/// "disabled" but a SECOND range filter - the one that does NOT apply
/// XIDAM to the received identifier first.
enum class FdcanExtFilterType : uint8_t {
    range = 0,          ///< id1 .. id2, XIDAM applied first
    dual = 1,
    classic = 2,
    range_no_mask = 3,  ///< id1 .. id2, XIDAM NOT applied
};

/// SFEC / EFEC (tables 221 and 223), the same seven codes for both
/// lists. Code 111 is "not used" and is not spelled.
enum class FdcanFilterAction : uint8_t {
    disabled = 0,
    store_fifo0 = 1,
    store_fifo1 = 2,
    reject = 3,
    priority = 4,           ///< raise IR.HPM, store nothing
    priority_fifo0 = 5,
    priority_fifo1 = 6,
};

/// PSR.ACT: what the module is doing on the bus right now.
enum class FdcanActivity : uint8_t {
    synchronizing = 0,
    idle = 1,
    receiver = 2,
    transmitter = 3,
};

/// PSR.LEC / PSR.DLEC (36.4.13). `none` is what a successful transfer
/// leaves; `no_change` is 7, which every READ of PSR writes back.
enum class FdcanError : uint8_t {
    none = 0,
    stuff = 1,
    form = 2,
    ack = 3,
    bit1 = 4,   ///< wanted recessive, monitored dominant
    bit0 = 5,   ///< wanted dominant, monitored recessive - and the bus-off recovery marker
    crc = 6,
    no_change = 7,
};

/// HPMS.MSI: where a high-priority match put the frame.
enum class FdcanHighPriorityStorage : uint8_t {
    none = 0,
    overrun = 1,
    fifo0 = 2,
    fifo1 = 3,
};

// ---- FDCAN_IR / FDCAN_IE, one name per bit (36.4.15) ------------------------

/// The interrupt register's flags. Edge-sensitive and rc_w1: a body
/// clears exactly what it serves.
namespace FdcanFlag {
constexpr uint32_t rx_fifo0_new = 1u << 0;
constexpr uint32_t rx_fifo0_full = 1u << 1;
constexpr uint32_t rx_fifo0_lost = 1u << 2;
constexpr uint32_t rx_fifo1_new = 1u << 3;
constexpr uint32_t rx_fifo1_full = 1u << 4;
constexpr uint32_t rx_fifo1_lost = 1u << 5;
constexpr uint32_t high_priority = 1u << 6;
constexpr uint32_t transmission_completed = 1u << 7;
constexpr uint32_t cancellation_finished = 1u << 8;
constexpr uint32_t tx_fifo_empty = 1u << 9;
constexpr uint32_t tx_event_new = 1u << 10;
constexpr uint32_t tx_event_full = 1u << 11;
constexpr uint32_t tx_event_lost = 1u << 12;
constexpr uint32_t timestamp_wrap = 1u << 13;
constexpr uint32_t ram_access_failure = 1u << 14;
constexpr uint32_t timeout = 1u << 15;
constexpr uint32_t error_logging_overflow = 1u << 16;
constexpr uint32_t error_passive = 1u << 17;
constexpr uint32_t warning = 1u << 18;
constexpr uint32_t bus_off = 1u << 19;
constexpr uint32_t watchdog = 1u << 20;
constexpr uint32_t protocol_error_arbitration = 1u << 21;
constexpr uint32_t protocol_error_data = 1u << 22;
constexpr uint32_t reserved_address = 1u << 23;
constexpr uint32_t all = 0x00FFFFFFu;
} // namespace FdcanFlag

/// FDCAN_ILS's seven GROUPS (36.4.17). This is the whole of the line
/// select on this silicon: a group's flags go to line 1 when its bit is
/// set and to line 0 when it is clear.
namespace FdcanGroup {
constexpr uint32_t rx_fifo0 = 1u << 0;
constexpr uint32_t rx_fifo1 = 1u << 1;
constexpr uint32_t status_message = 1u << 2;   ///< TCF, TC, HPM
constexpr uint32_t tx_fifo_error = 1u << 3;    ///< TEFL, TEFF, TEFN, TFE
constexpr uint32_t misc = 1u << 4;             ///< TOO, MRAF, TSW
constexpr uint32_t bit_line_error = 1u << 5;   ///< EP, ELO
constexpr uint32_t protocol_error = 1u << 6;   ///< ARA, PED, PEA, WDI, BO, EW
constexpr uint32_t all = 0x7Fu;
} // namespace FdcanGroup

/// Which ILS group an IR flag belongs to - 36.4.17's own lists, read
/// the other way round. 0 for a bit that is not a flag.
constexpr uint32_t fdcan_group_of(uint32_t flag) {
    if ((flag & (FdcanFlag::rx_fifo0_new | FdcanFlag::rx_fifo0_full |
                 FdcanFlag::rx_fifo0_lost)) != 0u) {
        return FdcanGroup::rx_fifo0;
    }
    if ((flag & (FdcanFlag::rx_fifo1_new | FdcanFlag::rx_fifo1_full |
                 FdcanFlag::rx_fifo1_lost)) != 0u) {
        return FdcanGroup::rx_fifo1;
    }
    if ((flag & (FdcanFlag::high_priority | FdcanFlag::transmission_completed |
                 FdcanFlag::cancellation_finished)) != 0u) {
        return FdcanGroup::status_message;
    }
    if ((flag & (FdcanFlag::tx_fifo_empty | FdcanFlag::tx_event_new |
                 FdcanFlag::tx_event_full | FdcanFlag::tx_event_lost)) != 0u) {
        return FdcanGroup::tx_fifo_error;
    }
    if ((flag & (FdcanFlag::timestamp_wrap | FdcanFlag::ram_access_failure |
                 FdcanFlag::timeout)) != 0u) {
        return FdcanGroup::misc;
    }
    if ((flag & (FdcanFlag::error_logging_overflow |
                 FdcanFlag::error_passive)) != 0u) {
        return FdcanGroup::bit_line_error;
    }
    if ((flag & (FdcanFlag::warning | FdcanFlag::bus_off | FdcanFlag::watchdog |
                 FdcanFlag::protocol_error_arbitration |
                 FdcanFlag::protocol_error_data |
                 FdcanFlag::reserved_address)) != 0u) {
        return FdcanGroup::protocol_error;
    }
    return 0;
}

/// Every IR flag that belongs to the groups in `groups`. The inverse of
/// fdcan_group_of(), and what an ISR body needs to split the register.
constexpr uint32_t fdcan_flags_of(uint32_t groups) {
    uint32_t mask = 0;
    for (uint8_t bit = 0; bit < 24; ++bit) {
        const uint32_t flag = 1u << bit;
        if ((fdcan_group_of(flag) & groups) != 0u) {
            mask |= flag;
        }
    }
    return mask;
}

// ---- DLC coding (table 212) -------------------------------------------------

/// How many DATA BYTES a DLC codes. Codes 0..8 are the byte count in
/// both formats; codes 9..15 are eight bytes in classic CAN and
/// 12/16/20/24/32/48/64 in FD.
constexpr uint8_t fdcan_dlc_to_length(uint8_t dlc, bool fd) {
    if (dlc <= 8u) {
        return dlc;
    }
    if (!fd) {
        return 8u;
    }
    switch (dlc) {
        case 9: return 12;
        case 10: return 16;
        case 11: return 20;
        case 12: return 24;
        case 13: return 32;
        case 14: return 48;
        default: return 64;
    }
}

/// The DLC that codes exactly `length` bytes, or 0xFF when no DLC does -
/// 13 bytes is not a CAN FD frame and never will be.
constexpr uint8_t fdcan_length_to_dlc(uint8_t length) {
    if (length <= 8u) {
        return length;
    }
    switch (length) {
        case 12: return 9;
        case 16: return 10;
        case 20: return 11;
        case 24: return 12;
        case 32: return 13;
        case 48: return 14;
        case 64: return 15;
        default: return 0xFF;
    }
}

constexpr bool fdcan_length_valid(uint8_t length) {
    return fdcan_length_to_dlc(length) != 0xFF;
}

// ---- bit timing (36.3.3, 36.4.3, 36.4.7) ------------------------------------

/// One phase's bit timing as the REGISTER holds it: every field is the
/// programmed value, one less than the thing it counts (36.4.7 -
/// "the actual interpretation by the hardware is such that one more than
/// the value programmed here is used"). `nominal` and `data` differ only
/// in their field widths, which is why one struct serves both and the
/// validators take a flag.
struct FdcanBitTiming {
    uint16_t brp = 0;    ///< NBRP 0..511 / DBRP 0..31 - tq = (brp + 1) / fdcan_tq_ck
    uint8_t tseg1 = 0;   ///< NTSEG1 0..255 / DTSEG1 0..31 - tBS1 = (tseg1 + 1) tq
    uint8_t tseg2 = 0;   ///< NTSEG2 0..127 / DTSEG2 0..15 - tBS2 = (tseg2 + 1) tq
    uint8_t sjw = 0;     ///< NSJW 0..127 / DSJW 0..15 - tSJW = (sjw + 1) tq

    /// SYNC_SEG + BS1 + BS2, in time quanta.
    constexpr uint32_t tq_per_bit() const {
        return 1u + (static_cast<uint32_t>(tseg1) + 1u) +
               (static_cast<uint32_t>(tseg2) + 1u);
    }
    /// Where the sample point sits, in parts per thousand of the bit.
    constexpr uint16_t sample_point_permille() const {
        return static_cast<uint16_t>(
            ((1u + static_cast<uint32_t>(tseg1) + 1u) * 1000u) / tq_per_bit());
    }
    /// Kernel-clock periods per bit: the divider times the quanta.
    constexpr uint32_t clocks_per_bit() const {
        return (static_cast<uint32_t>(brp) + 1u) * tq_per_bit();
    }
};

/// 36.4.7: "The CAN bit time can be programed in the range of 4 to
/// 81 x tq" - the nominal phase's window.
constexpr uint32_t fdcan_nominal_tq_min = 4;
constexpr uint32_t fdcan_nominal_tq_max = 81;

/// The data phase has no printed window of its own; 36.3.4 states its
/// floor ("the shortest configurable bit time of four time quanta") and
/// the register widths give the ceiling (1 + 32 + 16).
constexpr uint32_t fdcan_data_tq_min = 4;
constexpr uint32_t fdcan_data_tq_max = 49;

/// Whether a timing fits the NOMINAL registers and 36.4.7's window.
constexpr bool fdcan_nominal_timing_valid(const FdcanBitTiming& t) {
    // NTSEG1 is eight bits and NSJW/NTSEG2 seven, so `tseg1 <= 255` is a
    // tautology on a uint8_t and only the two narrower fields need a
    // range check at all - the widths are named in the struct.
    return t.brp <= 511u && t.tseg2 <= 127u && t.sjw <= 127u &&
           t.sjw <= t.tseg2 &&
           t.tq_per_bit() >= fdcan_nominal_tq_min &&
           t.tq_per_bit() <= fdcan_nominal_tq_max;
}

/// Whether a timing fits the DATA registers and 36.3.4's floor.
constexpr bool fdcan_data_timing_valid(const FdcanBitTiming& t) {
    return t.brp <= 31u && t.tseg1 <= 31u && t.tseg2 <= 15u && t.sjw <= 15u &&
           t.sjw <= t.tseg2 &&
           t.tq_per_bit() >= fdcan_data_tq_min &&
           t.tq_per_bit() <= fdcan_data_tq_max;
}

/// The bit rate a timing gives from a time-quantum clock of `tq_hz`.
constexpr uint32_t fdcan_bit_hz(uint32_t tq_hz, const FdcanBitTiming& t) {
    const uint32_t per_bit = t.clocks_per_bit();
    return per_bit == 0u ? 0u : tq_hz / per_bit;
}

namespace detail {

/// The shared chooser. THE RULE, and it is a choice this driver makes
/// rather than one the chapter states: the bit rate must come out EXACT
/// (tq_hz divisible by bit_hz, and the quotient divisible by the
/// prescaler), and among the prescalers that admit an exact bit the
/// SMALLEST is taken - the smallest prescaler is the finest time
/// quantum, which is the most tq in the bit and therefore the finest
/// grid the sample point can be placed on. The sample point is then put
/// at the nearest whole tq to the requested one and the answer is
/// REFUSED if that lands further than 1 % away, so a caller never gets a
/// silently different sample point. SJW is min(BS2, 4) - the four time
/// quanta 36.3.3's own text names - which is always legal because SJW
/// may not exceed BS2.
constexpr std::optional<FdcanBitTiming> timing_for(uint32_t tq_hz, uint32_t bit_hz,
                                                   uint16_t sample_permille,
                                                   uint16_t brp_max, uint16_t tseg1_max,
                                                   uint8_t tseg2_max, uint8_t sjw_max,
                                                   uint32_t tq_min, uint32_t tq_max) {
    if (tq_hz == 0u || bit_hz == 0u || sample_permille < 500u || sample_permille > 950u) {
        return std::nullopt;
    }
    if ((tq_hz % bit_hz) != 0u) {
        return std::nullopt;
    }
    const uint32_t total = tq_hz / bit_hz;
    for (uint32_t brp = 0; brp <= brp_max; ++brp) {
        const uint32_t divider = brp + 1u;
        if ((total % divider) != 0u) {
            continue;
        }
        const uint32_t n = total / divider;
        if (n < tq_min || n > tq_max) {
            continue;
        }
        // ts1 counts SYNC_SEG's neighbour: the sample point is after
        // (1 + ts1) quanta of the bit.
        uint32_t ts1 = (n * sample_permille + 500u) / 1000u;
        if (ts1 < 2u) {
            ts1 = 2u;
        }
        ts1 -= 1u;
        if (ts1 + 2u > n) {
            ts1 = n - 2u;
        }
        const uint32_t ts2 = n - 1u - ts1;
        if (ts1 < 1u || ts2 < 1u || (ts1 - 1u) > tseg1_max || (ts2 - 1u) > tseg2_max) {
            continue;
        }
        const uint32_t got = ((1u + ts1) * 1000u) / n;
        const uint32_t off = got > sample_permille ? got - sample_permille
                                                   : sample_permille - got;
        if (off > 10u) {
            return std::nullopt;
        }
        uint32_t sjw = ts2 < 4u ? ts2 : 4u;
        if (sjw - 1u > sjw_max) {
            sjw = static_cast<uint32_t>(sjw_max) + 1u;
        }
        return FdcanBitTiming{static_cast<uint16_t>(brp),
                              static_cast<uint8_t>(ts1 - 1u),
                              static_cast<uint8_t>(ts2 - 1u),
                              static_cast<uint8_t>(sjw - 1u)};
    }
    return std::nullopt;
}

} // namespace detail

/// The NOMINAL bit timing for `bit_hz` off a time-quantum clock of
/// `tq_hz`, with the sample point at `sample_permille` parts per
/// thousand. Empty when no exact answer exists (see detail::timing_for).
constexpr std::optional<FdcanBitTiming> fdcan_bit_timing_for(
    uint32_t tq_hz, uint32_t bit_hz, uint16_t sample_permille = 875) {
    return detail::timing_for(tq_hz, bit_hz, sample_permille, 511u, 255u, 127u, 127u,
                              fdcan_nominal_tq_min, fdcan_nominal_tq_max);
}

/// The same for the DATA phase, whose fields are far narrower.
constexpr std::optional<FdcanBitTiming> fdcan_data_timing_for(
    uint32_t tq_hz, uint32_t bit_hz, uint16_t sample_permille = 875) {
    return detail::timing_for(tq_hz, bit_hz, sample_permille, 31u, 31u, 15u, 15u,
                              fdcan_data_tq_min, fdcan_data_tq_max);
}

// ---- the frame, and why it is target-local ----------------------------------

/**
 * One CAN frame as this driver speaks it.
 *
 * DELIBERATELY TARGET-LOCAL. A `brio::CanFrame` every controller shares,
 * with a bus AO over it, is a util design decision, and a decision taken
 * from ONE implementation is a decision taken from the M_CAN's element
 * layout - which is not a neutral shape. When a second silicon's CAN
 * arrives the vocabulary is designed against both; until then this
 * struct is the STM32G0's and the doc says so.
 *
 * `id` IS THE NATURAL IDENTIFIER, right aligned: 0..0x7FF for a standard
 * frame and 0..0x1FFF_FFFF for an extended one. The element's own ID
 * field stores a standard identifier in ID[28:18] (table 215) and the
 * codecs below do that shift, because the filters speak the natural
 * 11-bit number (SFID1[10:0]) and a caller made to shift for one and not
 * the other would get it wrong.
 *
 * `length` IS BYTES AND NOT A DLC. The DLC is the wire's coding of it
 * and is computed on the way out; a length no DLC codes is refused.
 */
struct FdcanFrame {
    uint32_t id = 0;                 ///< natural identifier, right aligned
    uint8_t length = 0;              ///< 0..64 DATA BYTES (not the DLC)
    bool extended = false;           ///< XTD: a 29-bit identifier
    bool remote = false;             ///< RTR: a remote frame carries no data
    bool fd = false;                 ///< FDF: the FD frame format
    bool bit_rate_switch = false;    ///< BRS: switch to the data timing after it
    bool error_state_indicator = false;  ///< ESI (Tx: force recessive; Rx: what was seen)
    bool store_event = true;         ///< T1.EFC - write a Tx event FIFO element
    uint8_t marker = 0;              ///< T1.MM, copied into the Tx event element
    // Rx only, ignored on the way out.
    bool non_matching = false;       ///< R1.ANMF - no filter matched this frame
    uint8_t filter_index = 0;        ///< R1.FIDX - which element did match
    uint16_t timestamp = 0;          ///< R1.RXTS - the counter at start of frame
    uint8_t data[64] = {};
};

/// One Tx event FIFO element (tables 218 and 219): the transmit status
/// of a frame, decoupled from the buffer that carried it.
struct FdcanTxEvent {
    uint32_t id = 0;
    uint8_t length = 0;
    bool extended = false;
    bool remote = false;
    bool fd = false;
    bool bit_rate_switch = false;
    bool error_state_indicator = false;
    uint8_t marker = 0;
    uint8_t event_type = 0;   ///< E1.ET: 01 Tx event, 10 transmission in spite of cancellation
    uint16_t timestamp = 0;
};

/// The whole of FDCAN_PSR, decoded in ONE read (the register destroys
/// four of its own fields when read - 36.4.13's rc_r and rs marks).
struct FdcanStatus {
    FdcanActivity activity = FdcanActivity::synchronizing;
    FdcanError last_error = FdcanError::no_change;
    FdcanError data_last_error = FdcanError::no_change;
    bool bus_off = false;
    bool warning = false;
    bool error_passive = false;
    bool received_esi = false;    ///< RESI
    bool received_brs = false;    ///< RBRS
    bool received_fd = false;     ///< REDL
    bool protocol_exception = false;  ///< PXE
    uint8_t tdcv = 0;             ///< the secondary sample point, in mtq
};

/// FDCAN_ECR in one read - and CEL is cleared BY that read (36.4.12).
struct FdcanErrorCounters {
    uint8_t transmit = 0;      ///< TEC 0..255; > 255 is bus-off
    uint8_t receive = 0;       ///< REC 0..127
    bool receive_passive = false;  ///< RP: REC has reached 128
    uint8_t logging = 0;       ///< CEL, and reading this struct has just zeroed it
};

/// FDCAN_HPMS: what the last "set priority" filter match did.
struct FdcanHighPriority {
    bool extended_list = false;   ///< FLST
    uint8_t filter_index = 0;     ///< FIDX
    FdcanHighPriorityStorage storage = FdcanHighPriorityStorage::none;
    uint8_t buffer_index = 0;     ///< BIDX, valid only when a FIFO was named
};

/// A standard message ID filter element (36.3.11) - one RAM word.
struct FdcanStandardFilter {
    FdcanFilterType type = FdcanFilterType::disabled;
    FdcanFilterAction action = FdcanFilterAction::disabled;
    uint16_t id1 = 0;   ///< SFID1[10:0]: the ID, the range start, or the value
    uint16_t id2 = 0;   ///< SFID2[10:0]: the second ID, the range end, or the mask
};

/// An extended message ID filter element (36.3.12) - two RAM words.
struct FdcanExtendedFilter {
    FdcanExtFilterType type = FdcanExtFilterType::range;
    FdcanFilterAction action = FdcanFilterAction::disabled;
    uint32_t id1 = 0;   ///< EFID1[28:0]
    uint32_t id2 = 0;   ///< EFID2[28:0]
};

// ---- the element codecs (tables 214..219) -----------------------------------

/// A message RAM element as words. An Rx or Tx element is at most
/// eighteen; `count` is how many this frame actually occupies, which is
/// two headers plus one word per four data bytes.
struct FdcanElementWords {
    uint32_t w[18] = {};
    uint8_t count = 0;
};

/// Frame -> Tx buffer element (tables 216 and 217). Empty when the
/// length is not one a DLC codes.
constexpr std::optional<FdcanElementWords> fdcan_encode_tx(const FdcanFrame& f) {
    const uint8_t dlc = fdcan_length_to_dlc(f.length);
    if (dlc == 0xFF) {
        return std::nullopt;
    }
    if (!f.fd && f.length > 8u) {
        return std::nullopt;
    }
    FdcanElementWords e{};
    const uint32_t id_field = f.extended ? (f.id & 0x1FFFFFFFu)
                                         : ((f.id & 0x7FFu) << 18);
    e.w[0] = id_field |
             (f.remote ? (1u << 29) : 0u) |
             (f.extended ? (1u << 30) : 0u) |
             (f.error_state_indicator ? (1u << 31) : 0u);
    e.w[1] = (static_cast<uint32_t>(dlc) << 16) |
             (f.bit_rate_switch ? (1u << 20) : 0u) |
             (f.fd ? (1u << 21) : 0u) |
             (f.store_event ? (1u << 23) : 0u) |
             (static_cast<uint32_t>(f.marker) << 24);
    const uint8_t data_words = static_cast<uint8_t>((f.length + 3u) / 4u);
    for (uint8_t i = 0; i < data_words; ++i) {
        uint32_t word = 0;
        for (uint8_t b = 0; b < 4u; ++b) {
            const uint8_t k = static_cast<uint8_t>(4u * i + b);
            if (k < f.length) {
                word |= static_cast<uint32_t>(f.data[k]) << (8u * b);
            }
        }
        e.w[2 + i] = word;
    }
    e.count = static_cast<uint8_t>(2u + data_words);
    return e;
}

/// Rx FIFO element -> frame (tables 214 and 215). The caller supplies as
/// many words as the element's DLC needs; anything beyond `count` is
/// read as zero, which is what makes the fixture's hand-built words a
/// complete test of the decode.
constexpr FdcanFrame fdcan_decode_rx(const FdcanElementWords& e) {
    FdcanFrame f{};
    const uint32_t r0 = e.w[0];
    const uint32_t r1 = e.w[1];
    f.extended = (r0 & (1u << 30)) != 0u;
    f.remote = (r0 & (1u << 29)) != 0u;
    f.error_state_indicator = (r0 & (1u << 31)) != 0u;
    f.id = f.extended ? (r0 & 0x1FFFFFFFu) : ((r0 >> 18) & 0x7FFu);
    f.timestamp = static_cast<uint16_t>(r1 & 0xFFFFu);
    const uint8_t dlc = static_cast<uint8_t>((r1 >> 16) & 0xFu);
    f.bit_rate_switch = (r1 & (1u << 20)) != 0u;
    f.fd = (r1 & (1u << 21)) != 0u;
    f.filter_index = static_cast<uint8_t>((r1 >> 24) & 0x7Fu);
    f.non_matching = (r1 & (1u << 31)) != 0u;
    f.length = f.remote ? 0u : fdcan_dlc_to_length(dlc, f.fd);
    const uint8_t data_words = static_cast<uint8_t>((f.length + 3u) / 4u);
    for (uint8_t i = 0; i < data_words; ++i) {
        const uint32_t word = (2u + i) < 18u ? e.w[2 + i] : 0u;
        for (uint8_t b = 0; b < 4u; ++b) {
            const uint8_t k = static_cast<uint8_t>(4u * i + b);
            if (k < f.length) {
                f.data[k] = static_cast<uint8_t>((word >> (8u * b)) & 0xFFu);
            }
        }
    }
    return f;
}

/// Tx event FIFO element -> event (tables 218 and 219).
constexpr FdcanTxEvent fdcan_decode_event(uint32_t e0, uint32_t e1) {
    FdcanTxEvent ev{};
    ev.extended = (e0 & (1u << 30)) != 0u;
    ev.remote = (e0 & (1u << 29)) != 0u;
    ev.error_state_indicator = (e0 & (1u << 31)) != 0u;
    ev.id = ev.extended ? (e0 & 0x1FFFFFFFu) : ((e0 >> 18) & 0x7FFu);
    ev.timestamp = static_cast<uint16_t>(e1 & 0xFFFFu);
    ev.bit_rate_switch = (e1 & (1u << 20)) != 0u;
    ev.fd = (e1 & (1u << 21)) != 0u;
    ev.event_type = static_cast<uint8_t>((e1 >> 22) & 0x3u);
    ev.marker = static_cast<uint8_t>((e1 >> 24) & 0xFFu);
    ev.length = fdcan_dlc_to_length(static_cast<uint8_t>((e1 >> 16) & 0xFu), ev.fd);
    return ev;
}

/// Standard filter -> its one RAM word (table 220).
constexpr uint32_t fdcan_standard_filter_word(const FdcanStandardFilter& f) {
    return (static_cast<uint32_t>(f.type) << 30) |
           (static_cast<uint32_t>(f.action) << 27) |
           ((static_cast<uint32_t>(f.id1) & 0x7FFu) << 16) |
           (static_cast<uint32_t>(f.id2) & 0x7FFu);
}

/// Extended filter -> its two RAM words (table 222).
constexpr uint32_t fdcan_extended_filter_word0(const FdcanExtendedFilter& f) {
    return (static_cast<uint32_t>(f.action) << 29) | (f.id1 & 0x1FFFFFFFu);
}
constexpr uint32_t fdcan_extended_filter_word1(const FdcanExtendedFilter& f) {
    return (static_cast<uint32_t>(f.type) << 30) | (f.id2 & 0x1FFFFFFFu);
}

// ---- the configuration ------------------------------------------------------

/**
 * Everything `Fdcan<n>::enter()` writes between setting INIT + CCE and
 * clearing them again.
 *
 * ASM IS ITS OWN BIT, so `restricted` is its own flag rather than a
 * value of `mode`: the silicon can be told to hold back its dominant
 * bits on top of normal or bus-monitoring operation, and it ENTERS the
 * mode by itself when the Tx handler cannot read the message RAM in
 * time (36.3.4). `FdcanMode::restricted` is the ordinary way to ask for
 * it alone; the flag is the way to ask for it on top of something else,
 * and 36.3.4's note - "restricted operation mode must not be combined
 * with the loop-back mode" - is what `fdcan_config_valid()` refuses.
 *
 * THE CLOCK DIVIDER IS NOT HERE. CKDIV is the SUBSYSTEM's one register
 * (fact 2), and 36.4.37's note says it "must be modified before
 * configuring the other FDCAN instances" - a program-wide act with a
 * verb of its own, not a per-instance configuration field.
 */
struct FdcanConfig {
    FdcanBitTiming nominal{};
    FdcanBitTiming data{};                  ///< only written when `fd` is not off
    bool transmitter_delay_compensation = false;  ///< DBTP.TDC
    uint8_t tdc_offset = 0;                 ///< TDCR.TDCO 0..127 mtq
    uint8_t tdc_filter = 0;                 ///< TDCR.TDCF 0..127 mtq
    FdcanMode mode = FdcanMode::normal;
    bool restricted = false;                ///< CCCR.ASM on top of `mode`
    FdcanFd fd = FdcanFd::off;
    bool non_iso = false;                   ///< CCCR.NISO - Bosch V1.0 instead of ISO 11898-1
    bool disable_auto_retransmit = false;   ///< CCCR.DAR
    bool protocol_exception_disable = false;  ///< CCCR.PXHD
    bool edge_filtering = false;            ///< CCCR.EFBI - see ES0548 2.13.1
    bool transmit_pause = false;            ///< CCCR.TXP - two bit times between frames
    bool tx_queue = false;                  ///< TXBC.TFQM: false = FIFO, true = queue
    bool rx_fifo0_overwrite = false;        ///< RXGFC.F0OM
    bool rx_fifo1_overwrite = false;        ///< RXGFC.F1OM
    FdcanNonMatching non_matching_standard = FdcanNonMatching::fifo0;
    FdcanNonMatching non_matching_extended = FdcanNonMatching::fifo0;
    bool reject_remote_standard = false;    ///< RXGFC.RRFS
    bool reject_remote_extended = false;    ///< RXGFC.RRFE
    uint8_t standard_filters = 0;           ///< RXGFC.LSS 0..28
    uint8_t extended_filters = 0;           ///< RXGFC.LSE 0..8
    uint32_t extended_mask = 0x1FFFFFFFu;   ///< XIDAM - all ones is "not active"
    FdcanTimestamp timestamp = FdcanTimestamp::off;
    uint8_t timestamp_prescaler = 1;        ///< TSCC.TCP, 1..16 CAN bit times
    FdcanTimeoutMode timeout_mode = FdcanTimeoutMode::continuous;
    bool timeout_enable = false;            ///< TOCC.ETOC
    uint16_t timeout_period = 0xFFFF;       ///< TOCC.TOP
    uint8_t ram_watchdog = 0;               ///< RWD.WDC, 0 = disabled
};

/// Every configuration rule of chapter 36 in one place. False means
/// `enter()` writes nothing at all.
constexpr bool fdcan_config_valid(const FdcanConfig& c) {
    // 36.3.4's note: restricted operation must not be combined with
    // either loop-back. The two loop-backs are values of `mode`, so the
    // illegal pairing can only be built through the ASM flag.
    if (c.restricted && (c.mode == FdcanMode::external_loop_back ||
                         c.mode == FdcanMode::internal_loop_back)) {
        return false;
    }
    if (!fdcan_nominal_timing_valid(c.nominal)) {
        return false;
    }
    if (c.fd != FdcanFd::off) {
        if (!fdcan_data_timing_valid(c.data)) {
            return false;
        }
        // 36.4.3's note: "the data phase bit rate must be higher than or
        // equal to the nominal bit rate". Both phases count the SAME
        // time-quantum clock, so the comparison needs no frequency.
        if (c.data.clocks_per_bit() > c.nominal.clocks_per_bit()) {
            return false;
        }
    }
    if (c.tdc_offset > 127u || c.tdc_filter > 127u) {
        return false;
    }
    if (c.standard_filters > 28u || c.extended_filters > 8u) {
        return false;
    }
    if (c.extended_mask > 0x1FFFFFFFu) {
        return false;
    }
    if (c.timestamp_prescaler < 1u || c.timestamp_prescaler > 16u) {
        return false;
    }
    return true;
}

/// Whether a standard filter element is one the silicon can store: the
/// two identifiers are eleven bits and the "not used" action code does
/// not exist.
constexpr bool fdcan_standard_filter_valid(const FdcanStandardFilter& f) {
    return f.id1 <= 0x7FFu && f.id2 <= 0x7FFu &&
           static_cast<uint8_t>(f.action) <= 6u;
}

constexpr bool fdcan_extended_filter_valid(const FdcanExtendedFilter& f) {
    return f.id1 <= 0x1FFFFFFFu && f.id2 <= 0x1FFFFFFFu &&
           static_cast<uint8_t>(f.action) <= 6u;
}

#if defined(FDCAN1_BASE)

// =============================================================================
// The pads
// =============================================================================

/**
 * One pad handed to an FDCAN signal. THE AF IS THE DATASHEET'S: every
 * FDCAN pad of this part is AF3 (DS13560 tables 14, 15, 17 and 18) -
 *
 *   FDCAN1_RX  PA11  PB8   PC4  PD0   PD12
 *   FDCAN1_TX  PA12  PB9   PC5  PD1   PD13
 *   FDCAN2_RX  PB0   PB5   PB12 PC2   PD14
 *   FDCAN2_TX  PB1   PB6   PB13 PC3   PD15
 *
 * - and no symbol of the device header can check the claim, which is
 * fact 5 of pin.hpp's own header. The bench is the check.
 *
 * THE RX PAD'S PULL IS THE BENCH'S ONLY BUS. A pad under an INPUT
 * alternate function still follows its own PUPDR (measured by the LPTIM
 * and USART campaigns), so `claim_rx(PinPull::up)` gives the receiver a
 * recessive line and `PinPull::down` a stuck-dominant one - which is how
 * the error machine is reachable on a board with no transceiver.
 */
template <PinSel sel>
struct FdcanPad {
    FdcanPad() = delete;

    static_assert(sel.valid(),
                  "brio FdcanPad: this device has no such pad (port absent, or a pin "
                  "number past 15)");

    using pin = Pin<sel.port, sel.pin>;
    static constexpr PinSel selection = sel;

    /// The TX pad: an output alternate function. `speed` matters at the
    /// megabit rates the data phase reaches.
    static void claim_tx(PinSpeed speed = PinSpeed::very_high) {
        pin::function(sel.function, {.speed = speed});
    }
    /// The RX pad, with the pull that decides what an unconnected line
    /// reads: up = recessive, down = dominant.
    static void claim_rx(PinPull pull = PinPull::up) {
        pin::function(sel.function, {.pull = pull});
    }
    static void pull(PinPull p) { pin::pull(p); }
    static void release() { pin::release(); }
};

// =============================================================================
// The resource
// =============================================================================

/**
 * Fdcan<n>: one CAN module, and through it the subsystem it shares with
 * the other one.
 *
 * Every verb that writes a protected register refuses unless
 * `configurable()` - CCCR.INIT and CCCR.CCE both set - and writes
 * nothing when it refuses (fact 3). `enter()` is the whole sequence in
 * one call; `start()` and `stop()` are the two ends of it afterwards.
 */
template <uint8_t n>
struct Fdcan {
    static_assert(fdcan_present(n),
                  "brio Fdcan: this device has no such FDCAN instance (the device "
                  "header declares no FDCANn_BASE for it; the G0B1/G0C1 class has "
                  "FDCAN1 and FDCAN2, every smaller STM32G0 has neither)");

    Fdcan() = delete;

    static constexpr uint8_t index = n;

    // ---- the message RAM map (36.3.6, figure 399) ---------------------------
    // Word offsets inside THIS instance's 212-word slice. The map is the
    // manual's figure and is fixed: nothing on this part configures it.
    static constexpr uint16_t ram_words = 212;
    static constexpr uint8_t element_words = 18;     ///< one Rx or Tx element
    static constexpr uint8_t event_words = 2;        ///< one Tx event element
    static constexpr uint8_t fifo_depth = 3;
    static constexpr uint8_t standard_filter_count = 28;
    static constexpr uint8_t extended_filter_count = 8;
    static constexpr uint16_t standard_filter_start = 0x000 / 4;   ///< FLSSA
    static constexpr uint16_t extended_filter_start = 0x070 / 4;   ///< FLESA
    static constexpr uint16_t rx_fifo0_start = 0x0B0 / 4;          ///< F0SA
    static constexpr uint16_t rx_fifo1_start = 0x188 / 4;          ///< F1SA
    static constexpr uint16_t tx_event_start = 0x260 / 4;          ///< EFSA
    static constexpr uint16_t tx_buffer_start = 0x278 / 4;         ///< TBSA
    static constexpr uint8_t tx_buffers = 3;

    /// How many APB reads of CCCR a cross-domain handshake is given
    /// before a verb reports failure. 36.4.6's note asks for a readback
    /// and gives no bound, so this file supplies one - generous by three
    /// orders of magnitude at any rate this part can run.
    static constexpr uint32_t sync_spins = 200000;

    static FDCAN_GlobalTypeDef& regs() {
        return *reinterpret_cast<FDCAN_GlobalTypeDef*>(fdcan_base(n));
    }
    /// The instance's message RAM, as words.
    static volatile uint32_t* ram() {
        return reinterpret_cast<volatile uint32_t*>(fdcan_ram_base(n));
    }
    /// The SUBSYSTEM's configuration block - FDCAN1's CKDIV, shared.
    static FDCAN_Config_TypeDef& config_regs() {
        return *reinterpret_cast<FDCAN_Config_TypeDef*>(fdcan_config_base());
    }

    static constexpr IRQn_Type irq0() { return fdcan_irq(0); }
    static constexpr IRQn_Type irq1() { return fdcan_irq(1); }

    // ---- the subsystem's clock and reset ------------------------------------

    /// RCC_APBENR1.FDCANEN - ONE bit for both instances and the
    /// configuration block.
    static void bus_clock(bool on) { Rcc::apb1_clock(fdcan_clock_mask(), on); }
    static bool bus_clock() { return Rcc::apb1_clock(fdcan_clock_mask()); }

    /// RCC_APBRSTR1.FDCANRST, pulsed. THIS RESETS BOTH INSTANCES - there
    /// is one reset bit for the subsystem - so calling it through FDCAN2
    /// takes FDCAN1's registers back to their reset values too. It does
    /// NOT clear the message RAM, whose content is undefined either way.
    static void reset() { Rcc::apb1_reset(fdcan_reset_mask()); }

    /// RCC_CCIPR2.FDCANSEL. Only `pclk` is reachable: nothing in
    /// clock.hpp starts the PLL's Q output or HSE, so selecting either
    /// would leave the CAN core with no clock at all. False, and nothing
    /// written, for the other two.
    static bool kernel_clock(FdcanClock c) {
        if (c != FdcanClock::pclk) {
            return false;
        }
        return Rcc::kernel_clock2(fdcan_clock_select_pos(), static_cast<uint8_t>(c));
    }
    static uint8_t kernel_clock() { return Rcc::kernel_clock2(fdcan_clock_select_pos()); }

    /// FDCAN_CKDIV.PDIV. THE REGISTER IS FDCAN1'S AND THE DIVIDER IS THE
    /// SUBSYSTEM'S (36.4.37's note), so this moves the time quantum of
    /// BOTH modules and 36.4.37 asks for it to be set before the other
    /// instance is configured. 36.4.37 gates the write on CCCR.CCE
    /// alone; CCE is unreachable without INIT (36.3.4), so this verb
    /// asks for the same `configurable()` every other protected write
    /// does.
    static bool clock_divider(FdcanClockDivider d) {
        if (!configurable()) {
            return false;
        }
        config_regs().CKDIV = static_cast<uint32_t>(d) & 0xFu;
        return true;
    }
    static FdcanClockDivider clock_divider() {
        return static_cast<FdcanClockDivider>(config_regs().CKDIV & 0xFu);
    }

    /// The time-quantum clock a bit timing is counted in: the kernel
    /// clock after CKDIV. `pclk_hz` is the caller's, because a clock tag
    /// is the one truth about a rate on this target.
    static uint32_t tq_hz(uint32_t pclk) {
        return pclk / fdcan_divider_value(clock_divider());
    }

    // ---- the INIT / CCE state machine (36.3.4, 36.4.6) ----------------------

    static bool in_init() { return (regs().CCCR & FDCAN_CCCR_INIT) != 0u; }
    static bool configuration() { return (regs().CCCR & FDCAN_CCCR_CCE) != 0u; }
    /// The state every protected register needs: both bits set.
    static bool configurable() {
        const uint32_t c = regs().CCCR;
        return (c & (FDCAN_CCCR_INIT | FDCAN_CCCR_CCE)) ==
               (FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
    }

    /// Write CCCR.INIT and WAIT for the readback, because the bit
    /// crosses into the kernel clock domain and 36.4.6's note says the
    /// previous value must be seen before a new one is written. False on
    /// the bounded timeout, with the bit written.
    static bool init_mode(bool on) {
        FDCAN_GlobalTypeDef& r = regs();
        r.CCCR = on ? (r.CCCR | FDCAN_CCCR_INIT) : (r.CCCR & ~FDCAN_CCCR_INIT);
        for (uint32_t i = 0; i < sync_spins; ++i) {
            if (((r.CCCR & FDCAN_CCCR_INIT) != 0u) == on) {
                return true;
            }
        }
        return false;
    }

    /// CCCR.CCE. Setting it needs INIT already standing (36.3.4) and is
    /// REFUSED otherwise; clearing it is always legal. Setting it also
    /// resets FDCAN_HPMS, both RXFnS, TXFQS, TXBRP, TXBTO, TXBCF and
    /// TXEFS, presets the timeout counter to TOCC.TOP and holds both
    /// handlers idle - which is why `enter()` sets it before it writes
    /// a filter or a Tx buffer.
    static bool configuration(bool on) {
        FDCAN_GlobalTypeDef& r = regs();
        if (on && !in_init()) {
            return false;
        }
        r.CCCR = on ? (r.CCCR | FDCAN_CCCR_CCE) : (r.CCCR & ~FDCAN_CCCR_CCE);
        return true;
    }

    // ---- bit timing ---------------------------------------------------------

    /// FDCAN_NBTP. Protected, and the window of 36.4.7 is checked.
    static bool nominal_timing(const FdcanBitTiming& t) {
        if (!configurable() || !fdcan_nominal_timing_valid(t)) {
            return false;
        }
        regs().NBTP = (static_cast<uint32_t>(t.sjw & 0x7Fu) << FDCAN_NBTP_NSJW_Pos) |
                      (static_cast<uint32_t>(t.brp & 0x1FFu) << FDCAN_NBTP_NBRP_Pos) |
                      (static_cast<uint32_t>(t.tseg1) << FDCAN_NBTP_NTSEG1_Pos) |
                      (static_cast<uint32_t>(t.tseg2 & 0x7Fu) << FDCAN_NBTP_NTSEG2_Pos);
        return true;
    }
    static FdcanBitTiming nominal_timing() {
        const uint32_t v = regs().NBTP;
        return FdcanBitTiming{
            static_cast<uint16_t>((v >> FDCAN_NBTP_NBRP_Pos) & 0x1FFu),
            static_cast<uint8_t>((v >> FDCAN_NBTP_NTSEG1_Pos) & 0xFFu),
            static_cast<uint8_t>((v >> FDCAN_NBTP_NTSEG2_Pos) & 0x7Fu),
            static_cast<uint8_t>((v >> FDCAN_NBTP_NSJW_Pos) & 0x7Fu)};
    }

    /// FDCAN_DBTP, and DBTP.TDC beside it: the transceiver delay
    /// compensation lives in the DATA timing register because it only
    /// acts in the data phase (36.3.4).
    static bool data_timing(const FdcanBitTiming& t, bool delay_compensation = false) {
        if (!configurable() || !fdcan_data_timing_valid(t)) {
            return false;
        }
        regs().DBTP = (delay_compensation ? FDCAN_DBTP_TDC : 0u) |
                      (static_cast<uint32_t>(t.brp & 0x1Fu) << FDCAN_DBTP_DBRP_Pos) |
                      (static_cast<uint32_t>(t.tseg1 & 0x1Fu) << FDCAN_DBTP_DTSEG1_Pos) |
                      (static_cast<uint32_t>(t.tseg2 & 0xFu) << FDCAN_DBTP_DTSEG2_Pos) |
                      (static_cast<uint32_t>(t.sjw & 0xFu) << FDCAN_DBTP_DSJW_Pos);
        return true;
    }
    static FdcanBitTiming data_timing() {
        const uint32_t v = regs().DBTP;
        return FdcanBitTiming{
            static_cast<uint16_t>((v >> FDCAN_DBTP_DBRP_Pos) & 0x1Fu),
            static_cast<uint8_t>((v >> FDCAN_DBTP_DTSEG1_Pos) & 0x1Fu),
            static_cast<uint8_t>((v >> FDCAN_DBTP_DTSEG2_Pos) & 0xFu),
            static_cast<uint8_t>((v >> FDCAN_DBTP_DSJW_Pos) & 0xFu)};
    }
    static bool delay_compensation() { return (regs().DBTP & FDCAN_DBTP_TDC) != 0u; }

    /// FDCAN_TDCR: where the secondary sample point sits inside the
    /// received bit, and the filter window that keeps a glitch in the
    /// FDF bit from ending the measurement early. Both in mtq.
    static bool delay_compensation_offsets(uint8_t offset, uint8_t filter) {
        if (!configurable() || offset > 127u || filter > 127u) {
            return false;
        }
        regs().TDCR = (static_cast<uint32_t>(offset) << FDCAN_TDCR_TDCO_Pos) |
                      (static_cast<uint32_t>(filter) << FDCAN_TDCR_TDCF_Pos);
        return true;
    }

    // ---- the mode bits (36.3.4, 36.4.6) -------------------------------------

    /// CCCR.MON. 36.3.4: set only with INIT + CCE, cleared at any time.
    static bool bus_monitor(bool on) {
        if (on && !configurable()) {
            return false;
        }
        write_cccr(FDCAN_CCCR_MON, on);
        return true;
    }
    static bool bus_monitor() { return (regs().CCCR & FDCAN_CCCR_MON) != 0u; }

    /// CCCR.ASM, the restricted operation mode. Same gate as MON - and
    /// REFUSED while TEST.LBCK stands, because 36.3.4 forbids the
    /// combination and this is the only road to it once a mode has been
    /// entered. The hardware sets ASM by itself when the Tx handler
    /// cannot read the RAM in time, and clearing it is how a program
    /// leaves that state.
    static bool restricted(bool on) {
        if (on && (!configurable() || loop_back())) {
            return false;
        }
        write_cccr(FDCAN_CCCR_ASM, on);
        return true;
    }
    static bool restricted() { return (regs().CCCR & FDCAN_CCCR_ASM) != 0u; }

    /// CCCR.DAR - "disable automatic retransmission". Both directions
    /// need INIT + CCE (36.3.4). The verb is spelled the way an
    /// application thinks: `auto_retransmit(false)` sets DAR.
    static bool auto_retransmit(bool on) {
        if (!configurable()) {
            return false;
        }
        write_cccr(FDCAN_CCCR_DAR, !on);
        return true;
    }
    static bool auto_retransmit() { return (regs().CCCR & FDCAN_CCCR_DAR) == 0u; }

    /// CCCR.TEST: what opens FDCAN_TEST for writing. Clearing it puts
    /// that whole register back to its reset value (36.4.4), which is
    /// what takes a loop-back down.
    static bool test_mode(bool on) {
        if (on && !configurable()) {
            return false;
        }
        write_cccr(FDCAN_CCCR_TEST, on);
        return true;
    }
    static bool test_mode() { return (regs().CCCR & FDCAN_CCCR_TEST) != 0u; }

    /// CCCR.FDOE and CCCR.BRSE: table 213's two knobs. Protected.
    static bool fd_mode(FdcanFd f) {
        if (!configurable()) {
            return false;
        }
        write_cccr(FDCAN_CCCR_FDOE, f != FdcanFd::off);
        write_cccr(FDCAN_CCCR_BRSE, f == FdcanFd::on_with_bit_rate_switch);
        return true;
    }
    static FdcanFd fd_mode() {
        const uint32_t c = regs().CCCR;
        if ((c & FDCAN_CCCR_FDOE) == 0u) {
            return FdcanFd::off;
        }
        return (c & FDCAN_CCCR_BRSE) != 0u ? FdcanFd::on_with_bit_rate_switch
                                           : FdcanFd::on;
    }

    /// CCCR.NISO - the Bosch V1.0 frame format instead of ISO 11898-1's
    /// (they differ in the CRC's initialization and stuff-bit count, so
    /// two nodes that disagree cannot talk).
    static bool non_iso(bool on) {
        if (!configurable()) {
            return false;
        }
        write_cccr(FDCAN_CCCR_NISO, on);
        return true;
    }
    static bool non_iso() { return (regs().CCCR & FDCAN_CCCR_NISO) != 0u; }

    /// CCCR.TXP: two CAN bit times of silence after every successful
    /// transmission, so a burst from one node cannot starve the bus.
    static bool transmit_pause(bool on) {
        if (!configurable()) {
            return false;
        }
        write_cccr(FDCAN_CCCR_TXP, on);
        return true;
    }
    static bool transmit_pause() { return (regs().CCCR & FDCAN_CCCR_TXP) != 0u; }

    /// CCCR.EFBI: two consecutive dominant tq required before an edge is
    /// taken for hard synchronization. ES0548 2.13.1 IS AN OBLIGATION ON
    /// THIS BIT, not a reason to refuse it: with edge filtering enabled
    /// an FD frame whose integration phase ends on a falling RX edge can
    /// be received with its first bit wrong, which the CRC catches and
    /// an error frame answers. The workaround is to leave the bit clear
    /// or to accept the retransmission; nothing a driver can do.
    static bool edge_filtering(bool on) {
        if (!configurable()) {
            return false;
        }
        write_cccr(FDCAN_CCCR_EFBI, on);
        return true;
    }
    static bool edge_filtering() { return (regs().CCCR & FDCAN_CCCR_EFBI) != 0u; }

    /// CCCR.PXHD: with protocol exception handling DISABLED a recessive
    /// res bit is answered with a form error instead of a silent
    /// re-integration (36.3.4).
    static bool protocol_exception_disable(bool on) {
        if (!configurable()) {
            return false;
        }
        write_cccr(FDCAN_CCCR_PXHD, on);
        return true;
    }
    static bool protocol_exception_disable() {
        return (regs().CCCR & FDCAN_CCCR_PXHD) != 0u;
    }

    // ---- the test register (36.4.4) -----------------------------------------

    /// TEST.LBCK. Needs CCCR.TEST; refused otherwise, and refused while
    /// ASM stands (36.3.4's note, the other direction of restricted()).
    static bool loop_back(bool on) {
        if (!test_mode() || (on && restricted())) {
            return false;
        }
        FDCAN_GlobalTypeDef& r = regs();
        r.TEST = on ? (r.TEST | FDCAN_TEST_LBCK) : (r.TEST & ~FDCAN_TEST_LBCK);
        return true;
    }
    static bool loop_back() { return (regs().TEST & FDCAN_TEST_LBCK) != 0u; }

    /// TEST.TX: what drives the pad. `sample_point` is the bench's
    /// bit-rate meter (fact 5). Needs CCCR.TEST.
    static bool tx_pin(FdcanTxPin p) {
        if (!test_mode()) {
            return false;
        }
        FDCAN_GlobalTypeDef& r = regs();
        r.TEST = (r.TEST & ~FDCAN_TEST_TX_Msk) |
                 (static_cast<uint32_t>(p) << FDCAN_TEST_TX_Pos);
        return true;
    }
    static FdcanTxPin tx_pin() {
        return static_cast<FdcanTxPin>((regs().TEST & FDCAN_TEST_TX_Msk) >>
                                       FDCAN_TEST_TX_Pos);
    }

    /// TEST.RX: the level on FDCANn_RX as the core sees it, after the
    /// synchronizer - so it lags a pad change by a few APB periods
    /// (36.4.4's own note). True is recessive.
    static bool rx_pin() { return (regs().TEST & FDCAN_TEST_RX) != 0u; }

    // ---- power-down (36.3.4) ------------------------------------------------

    /// CCCR.CSR, the clock stop REQUEST: the module finishes what it is
    /// transmitting, waits for bus idle, sets INIT itself and then
    /// acknowledges. Legal at any time - it is not a configuration bit.
    static void power_down(bool on) { write_cccr(FDCAN_CCCR_CSR, on); }
    static bool power_down() { return (regs().CCCR & FDCAN_CCCR_CSR) != 0u; }
    /// CCCR.CSA, read-only: the clocks may now be stopped.
    static bool power_down_acked() { return (regs().CCCR & FDCAN_CCCR_CSA) != 0u; }

    // ---- the message RAM ----------------------------------------------------

    /// One word of this instance's slice, by WORD offset (0..211).
    static uint32_t ram_word(uint16_t offset) {
        return offset < ram_words ? ram()[offset] : 0u;
    }
    static bool set_ram_word(uint16_t offset, uint32_t value) {
        if (offset >= ram_words) {
            return false;
        }
        ram()[offset] = value;
        return true;
    }
    /// Zero all 212 words. The RAM's content after a reset is undefined
    /// (fact 4), so this is not decoration: an un-zeroed filter list is
    /// 28 elements of whatever the last program left.
    static void clear_ram() {
        volatile uint32_t* m = ram();
        for (uint16_t i = 0; i < ram_words; ++i) {
            m[i] = 0;
        }
    }

    /// FDCAN_RWD.WDC: the message RAM watchdog's start value, counted
    /// down in fdcan_pclk periods from every RAM access and reloaded
    /// when the RAM answers. 0 disables it. Protected.
    static bool ram_watchdog(uint8_t count) {
        if (!configurable()) {
            return false;
        }
        regs().RWD = count;
        return true;
    }
    static uint8_t ram_watchdog() {
        return static_cast<uint8_t>(regs().RWD & FDCAN_RWD_WDC_Msk);
    }
    /// RWD.WDV, the live counter.
    static uint8_t ram_watchdog_value() {
        return static_cast<uint8_t>((regs().RWD & FDCAN_RWD_WDV_Msk) >> FDCAN_RWD_WDV_Pos);
    }

    // ---- filters (36.3.11, 36.3.12, 36.4.19, 36.4.20) -----------------------

    /// One standard filter element, written into the message RAM.
    /// THE RAM IS NOT A PROTECTED REGISTER: 36.3.6 says the list is
    /// executed from element #0 at every frame, so editing an element
    /// while the module runs is legal and takes effect at the next
    /// frame. What IS protected is how many elements the list has
    /// (RXGFC.LSS), which is why `filter_lists()` refuses and this does
    /// not.
    static bool standard_filter(uint8_t index, const FdcanStandardFilter& f) {
        if (index >= standard_filter_count || !fdcan_standard_filter_valid(f)) {
            return false;
        }
        ram()[standard_filter_start + index] = fdcan_standard_filter_word(f);
        return true;
    }
    static uint32_t standard_filter(uint8_t index) {
        return index < standard_filter_count ? ram()[standard_filter_start + index] : 0u;
    }

    static bool extended_filter(uint8_t index, const FdcanExtendedFilter& f) {
        if (index >= extended_filter_count || !fdcan_extended_filter_valid(f)) {
            return false;
        }
        volatile uint32_t* m = ram() + extended_filter_start +
                               static_cast<uint16_t>(2u * index);
        m[0] = fdcan_extended_filter_word0(f);
        m[1] = fdcan_extended_filter_word1(f);
        return true;
    }

    /// RXGFC.LSS and RXGFC.LSE - how many elements of each list are
    /// evaluated. 36.4.19: values above 28 and 8 "are interpreted as"
    /// the maximum, so the silicon clamps rather than refuses; this verb
    /// refuses instead, because a program that asks for 31 standard
    /// filters has a bug and a clamp hides it.
    static bool filter_lists(uint8_t standard, uint8_t extended) {
        if (!configurable() || standard > standard_filter_count ||
            extended > extended_filter_count) {
            return false;
        }
        FDCAN_GlobalTypeDef& r = regs();
        r.RXGFC = (r.RXGFC & ~(FDCAN_RXGFC_LSS_Msk | FDCAN_RXGFC_LSE_Msk)) |
                  (static_cast<uint32_t>(standard) << FDCAN_RXGFC_LSS_Pos) |
                  (static_cast<uint32_t>(extended) << FDCAN_RXGFC_LSE_Pos);
        return true;
    }
    static uint8_t standard_filter_list() {
        return static_cast<uint8_t>((regs().RXGFC & FDCAN_RXGFC_LSS_Msk) >>
                                    FDCAN_RXGFC_LSS_Pos);
    }
    static uint8_t extended_filter_list() {
        return static_cast<uint8_t>((regs().RXGFC & FDCAN_RXGFC_LSE_Msk) >>
                                    FDCAN_RXGFC_LSE_Pos);
    }

    /// RXGFC.ANFS / RXGFC.ANFE: where a frame no filter matched goes.
    static bool non_matching(FdcanNonMatching standard, FdcanNonMatching extended) {
        if (!configurable()) {
            return false;
        }
        FDCAN_GlobalTypeDef& r = regs();
        r.RXGFC = (r.RXGFC & ~(FDCAN_RXGFC_ANFS_Msk | FDCAN_RXGFC_ANFE_Msk)) |
                  (static_cast<uint32_t>(standard) << FDCAN_RXGFC_ANFS_Pos) |
                  (static_cast<uint32_t>(extended) << FDCAN_RXGFC_ANFE_Pos);
        return true;
    }

    /// RXGFC.RRFS / RXGFC.RRFE: reject every remote frame before the
    /// filter list is even reached (figures 400 and 401).
    static bool reject_remote(bool standard, bool extended) {
        if (!configurable()) {
            return false;
        }
        FDCAN_GlobalTypeDef& r = regs();
        uint32_t v = r.RXGFC & ~(FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE);
        v |= standard ? FDCAN_RXGFC_RRFS : 0u;
        v |= extended ? FDCAN_RXGFC_RRFE : 0u;
        r.RXGFC = v;
        return true;
    }

    /// RXGFC.F0OM / F1OM: blocking (the reset behaviour) or overwrite.
    static bool fifo_overwrite(bool fifo0, bool fifo1) {
        if (!configurable()) {
            return false;
        }
        FDCAN_GlobalTypeDef& r = regs();
        uint32_t v = r.RXGFC & ~(FDCAN_RXGFC_F0OM | FDCAN_RXGFC_F1OM);
        v |= fifo0 ? FDCAN_RXGFC_F0OM : 0u;
        v |= fifo1 ? FDCAN_RXGFC_F1OM : 0u;
        r.RXGFC = v;
        return true;
    }
    static bool fifo_overwrite(uint8_t fifo) {
        const uint32_t bit = fifo == 0u ? FDCAN_RXGFC_F0OM : FDCAN_RXGFC_F1OM;
        return (regs().RXGFC & bit) != 0u;
    }
    static uint32_t global_filter_config() { return regs().RXGFC; }

    /// FDCAN_XIDAM, AND-ed with every received 29-bit identifier before
    /// the extended list runs - except for a range filter whose EFT is
    /// 11 (36.3.6).
    static bool extended_mask(uint32_t mask) {
        if (!configurable() || mask > 0x1FFFFFFFu) {
            return false;
        }
        regs().XIDAM = mask;
        return true;
    }
    static uint32_t extended_mask() { return regs().XIDAM & 0x1FFFFFFFu; }

    /// FDCAN_HPMS, updated by every "set priority" match.
    static FdcanHighPriority high_priority() {
        const uint32_t v = regs().HPMS;
        FdcanHighPriority h{};
        h.extended_list = (v & FDCAN_HPMS_FLST) != 0u;
        h.filter_index = static_cast<uint8_t>((v & FDCAN_HPMS_FIDX_Msk) >>
                                              FDCAN_HPMS_FIDX_Pos);
        h.storage = static_cast<FdcanHighPriorityStorage>(
            (v & FDCAN_HPMS_MSI_Msk) >> FDCAN_HPMS_MSI_Pos);
        h.buffer_index = static_cast<uint8_t>((v & FDCAN_HPMS_BIDX_Msk) >>
                                              FDCAN_HPMS_BIDX_Pos);
        return h;
    }

    // ---- receiving (36.3.7, 36.3.8, 36.4.22 .. 36.4.25) ---------------------

    static uint32_t rx_status(uint8_t fifo) {
        return fifo == 0u ? regs().RXF0S : regs().RXF1S;
    }
    /// FnFL, the fill level: how many elements are waiting.
    static uint8_t rx_available(uint8_t fifo) {
        return static_cast<uint8_t>(rx_status(fifo) & FDCAN_RXF0S_F0FL_Msk);
    }
    static bool rx_full(uint8_t fifo) {
        return (rx_status(fifo) & FDCAN_RXF0S_F0F) != 0u;
    }
    /// RFnL: at least one frame was dropped because the FIFO was full.
    /// A copy of the IR flag, and cleared with it.
    static bool rx_lost(uint8_t fifo) {
        return (rx_status(fifo) & FDCAN_RXF0S_RF0L) != 0u;
    }
    static uint8_t rx_get_index(uint8_t fifo) {
        return static_cast<uint8_t>((rx_status(fifo) & FDCAN_RXF0S_F0GI_Msk) >>
                                    FDCAN_RXF0S_F0GI_Pos);
    }
    static uint8_t rx_put_index(uint8_t fifo) {
        return static_cast<uint8_t>((rx_status(fifo) & FDCAN_RXF0S_F0PI_Msk) >>
                                    FDCAN_RXF0S_F0PI_Pos);
    }

    /// Read the element at `slot` WITHOUT acknowledging it. 36.3.7's own
    /// warning applies: reading out of turn is legal, but the
    /// acknowledge index must then NOT be written, or the get index
    /// jumps and the older elements are lost.
    static bool rx_peek(uint8_t fifo, uint8_t slot, FdcanFrame& out) {
        if (fifo > 1u || slot >= fifo_depth) {
            return false;
        }
        const uint16_t base = static_cast<uint16_t>(
            (fifo == 0u ? rx_fifo0_start : rx_fifo1_start) +
            static_cast<uint16_t>(slot) * element_words);
        volatile uint32_t* m = ram() + base;
        FdcanElementWords e{};
        e.w[0] = m[0];
        e.w[1] = m[1];
        const bool fd = (e.w[1] & (1u << 21)) != 0u;
        const bool remote = (e.w[0] & (1u << 29)) != 0u;
        const uint8_t len = remote
                                ? 0u
                                : fdcan_dlc_to_length(
                                      static_cast<uint8_t>((e.w[1] >> 16) & 0xFu), fd);
        const uint8_t words = static_cast<uint8_t>((len + 3u) / 4u);
        for (uint8_t i = 0; i < words; ++i) {
            e.w[2 + i] = m[2 + i];
        }
        e.count = static_cast<uint8_t>(2u + words);
        out = fdcan_decode_rx(e);
        return true;
    }

    /// FDCAN_RXFnA: set the get index to `slot` + 1 and recompute the
    /// fill level. 36.3.7: after a SEQUENCE of reads it is enough to
    /// write the index of the LAST element read.
    static bool rx_acknowledge(uint8_t fifo, uint8_t slot) {
        if (fifo > 1u || slot >= fifo_depth) {
            return false;
        }
        if (fifo == 0u) {
            regs().RXF0A = slot;
        } else {
            regs().RXF1A = slot;
        }
        return true;
    }

    /// The ordinary read: the element at the GET index, acknowledged.
    /// False when the FIFO is empty.
    static bool rx_read(uint8_t fifo, FdcanFrame& out) {
        if (fifo > 1u || rx_available(fifo) == 0u) {
            return false;
        }
        const uint8_t slot = rx_get_index(fifo);
        if (!rx_peek(fifo, slot, out)) {
            return false;
        }
        return rx_acknowledge(fifo, slot);
    }

    /// THE OVERWRITE MODE'S OWN READ (36.3.6, "Rx FIFO overwrite mode").
    /// When the FIFO is full and the module may overwrite, reading at
    /// the get index races the put index onto the same element - so the
    /// chapter says to start "at least at get index + 1". This verb is
    /// that rule as code; it is only different from rx_read() while the
    /// FIFO is full, and it acknowledges the element it actually read.
    static bool rx_read_overwrite(uint8_t fifo, FdcanFrame& out) {
        if (fifo > 1u || rx_available(fifo) == 0u) {
            return false;
        }
        uint8_t slot = rx_get_index(fifo);
        if (rx_full(fifo)) {
            slot = static_cast<uint8_t>((slot + 1u) % fifo_depth);
        }
        if (!rx_peek(fifo, slot, out)) {
            return false;
        }
        return rx_acknowledge(fifo, slot);
    }

    // ---- transmitting (36.3.9, 36.4.26 .. 36.4.34) --------------------------

    /// TXBC.TFQM: false is a FIFO (insertion order), true is a QUEUE
    /// (lowest identifier first, lowest buffer number on a tie).
    /// Protected.
    static bool tx_queue_mode(bool queue) {
        if (!configurable()) {
            return false;
        }
        FDCAN_GlobalTypeDef& r = regs();
        r.TXBC = queue ? (r.TXBC | FDCAN_TXBC_TFQM) : (r.TXBC & ~FDCAN_TXBC_TFQM);
        return true;
    }
    static bool tx_queue_mode() { return (regs().TXBC & FDCAN_TXBC_TFQM) != 0u; }

    /// TXFQS.TFFL: how many consecutive buffers are free from the get
    /// index. READ AS ZERO IN QUEUE MODE (36.4.27) - so a queue's room
    /// is `!tx_full()`, not this.
    static uint8_t tx_free() {
        return static_cast<uint8_t>(regs().TXFQS & FDCAN_TXFQS_TFFL_Msk);
    }
    static bool tx_full() { return (regs().TXFQS & FDCAN_TXFQS_TFQF) != 0u; }
    static uint8_t tx_put_index() {
        return static_cast<uint8_t>((regs().TXFQS & FDCAN_TXFQS_TFQPI_Msk) >>
                                    FDCAN_TXFQS_TFQPI_Pos);
    }
    /// TXFQS.TFGI, also read as zero in queue mode.
    static uint8_t tx_get_index() {
        return static_cast<uint8_t>((regs().TXFQS & FDCAN_TXFQS_TFGI_Msk) >>
                                    FDCAN_TXFQS_TFGI_Pos);
    }

    /// Write a frame into Tx buffer `index` WITHOUT requesting it - the
    /// half of `tx_put()` a caller needs when it adds several frames and
    /// requests them with one TXBAR write (36.3.6, "Tx FIFO").
    static bool tx_put_buffer(uint8_t index, const FdcanFrame& f) {
        if (index >= tx_buffers) {
            return false;
        }
        const auto e = fdcan_encode_tx(f);
        if (!e) {
            return false;
        }
        volatile uint32_t* m = ram() + tx_buffer_start +
                               static_cast<uint16_t>(index) * element_words;
        for (uint8_t i = 0; i < e->count; ++i) {
            m[i] = e->w[i];
        }
        return true;
    }

    /// FDCAN_TXBAR: request the buffers named in `mask`. WRITABLE ONLY
    /// WITH CCE CLEAR (36.3.4) - so this refuses while the module is
    /// configurable, which is the one place in this file where a verb
    /// refuses because a gate is OPEN.
    static bool tx_request(uint8_t mask) {
        if (configuration() || (mask & ~0x7u) != 0u) {
            return false;
        }
        regs().TXBAR = mask;
        return true;
    }

    /// The ordinary transmit: write at the put index and request it.
    /// Empty when the FIFO/queue is full, when the length is not one a
    /// DLC codes, or when CCE still stands.
    static std::optional<uint8_t> tx_put(const FdcanFrame& f) {
        if (tx_full() || configuration()) {
            return std::nullopt;
        }
        const uint8_t index = tx_put_index();
        if (!tx_put_buffer(index, f)) {
            return std::nullopt;
        }
        if (!tx_request(static_cast<uint8_t>(1u << index))) {
            return std::nullopt;
        }
        return index;
    }

    /// FDCAN_TXBCR, the same CCE rule as TXBAR. A cancellation of a
    /// transmission already on the wire is finished at the END of it,
    /// successful or not (36.4.28).
    static bool tx_cancel(uint8_t mask) {
        if (configuration() || (mask & ~0x7u) != 0u) {
            return false;
        }
        regs().TXBCR = mask;
        return true;
    }

    static uint8_t tx_pending() { return static_cast<uint8_t>(regs().TXBRP & 0x7u); }
    static uint8_t tx_occurred() { return static_cast<uint8_t>(regs().TXBTO & 0x7u); }
    static uint8_t tx_cancelled() { return static_cast<uint8_t>(regs().TXBCF & 0x7u); }
    static uint8_t tx_cancel_requested() {
        return static_cast<uint8_t>(regs().TXBCR & 0x7u);
    }

    /// TXBTIE / TXBCIE: which buffers raise IR.TC and IR.TCF. Not
    /// protected - a program may arm a buffer at any time.
    static bool tx_buffer_interrupts(uint8_t mask) {
        if ((mask & ~0x7u) != 0u) {
            return false;
        }
        regs().TXBTIE = mask;
        return true;
    }
    static uint8_t tx_buffer_interrupts() {
        return static_cast<uint8_t>(regs().TXBTIE & 0x7u);
    }
    static bool tx_cancel_interrupts(uint8_t mask) {
        if ((mask & ~0x7u) != 0u) {
            return false;
        }
        regs().TXBCIE = mask;
        return true;
    }
    static uint8_t tx_cancel_interrupts() {
        return static_cast<uint8_t>(regs().TXBCIE & 0x7u);
    }

    // ---- the Tx event FIFO (36.3.10, 36.4.35, 36.4.36) ----------------------

    static uint8_t events_available() {
        return static_cast<uint8_t>(regs().TXEFS & FDCAN_TXEFS_EFFL_Msk);
    }
    static bool events_full() { return (regs().TXEFS & FDCAN_TXEFS_EFF) != 0u; }
    static bool events_lost() { return (regs().TXEFS & FDCAN_TXEFS_TEFL) != 0u; }
    static uint8_t event_get_index() {
        return static_cast<uint8_t>((regs().TXEFS & FDCAN_TXEFS_EFGI_Msk) >>
                                    FDCAN_TXEFS_EFGI_Pos);
    }
    static uint8_t event_put_index() {
        return static_cast<uint8_t>((regs().TXEFS & FDCAN_TXEFS_EFPI_Msk) >>
                                    FDCAN_TXEFS_EFPI_Pos);
    }

    /// The element at the get index, acknowledged. Its two words are at
    /// EFSA + 2 x get index (36.3.6).
    static bool event_read(FdcanTxEvent& out) {
        if (events_available() == 0u) {
            return false;
        }
        const uint8_t slot = event_get_index();
        volatile uint32_t* m = ram() + tx_event_start +
                               static_cast<uint16_t>(slot) * event_words;
        out = fdcan_decode_event(m[0], m[1]);
        regs().TXEFA = slot;
        return true;
    }

    // ---- timestamp and timeout (36.3.4, 36.4.8 .. 36.4.11) ------------------

    /// FDCAN_TSCC. `prescaler` is in CAN BIT TIMES, 1..16, and only
    /// means anything for the internal source. Protected.
    static bool timestamp(FdcanTimestamp source, uint8_t prescaler = 1) {
        if (!configurable() || prescaler < 1u || prescaler > 16u) {
            return false;
        }
        regs().TSCC = (static_cast<uint32_t>(prescaler - 1u) << FDCAN_TSCC_TCP_Pos) |
                      static_cast<uint32_t>(source);
        return true;
    }
    static FdcanTimestamp timestamp() {
        return static_cast<FdcanTimestamp>(regs().TSCC & FDCAN_TSCC_TSS_Msk);
    }
    static uint8_t timestamp_prescaler() {
        return static_cast<uint8_t>(
            ((regs().TSCC & FDCAN_TSCC_TCP_Msk) >> FDCAN_TSCC_TCP_Pos) + 1u);
    }
    static uint16_t timestamp_value() {
        return static_cast<uint16_t>(regs().TSCV & 0xFFFFu);
    }
    /// A write of any value puts the internal counter back to zero; with
    /// the external source it has no effect (36.4.9).
    static void reset_timestamp() { regs().TSCV = 0; }

    /// FDCAN_TOCC: the down-counter, its reload and what starts it.
    /// Protected, and it "can only be started while INIT is cleared".
    static bool timeout(FdcanTimeoutMode mode, uint16_t period, bool enable) {
        if (!configurable()) {
            return false;
        }
        regs().TOCC = (static_cast<uint32_t>(period) << FDCAN_TOCC_TOP_Pos) |
                      (static_cast<uint32_t>(mode) << FDCAN_TOCC_TOS_Pos) |
                      (enable ? FDCAN_TOCC_ETOC : 0u);
        return true;
    }
    static bool timeout_enabled() { return (regs().TOCC & FDCAN_TOCC_ETOC) != 0u; }
    static FdcanTimeoutMode timeout_mode() {
        return static_cast<FdcanTimeoutMode>((regs().TOCC & FDCAN_TOCC_TOS_Msk) >>
                                             FDCAN_TOCC_TOS_Pos);
    }
    static uint16_t timeout_period() {
        return static_cast<uint16_t>((regs().TOCC & FDCAN_TOCC_TOP_Msk) >>
                                     FDCAN_TOCC_TOP_Pos);
    }
    static uint16_t timeout_value() { return static_cast<uint16_t>(regs().TOCV & 0xFFFFu); }
    /// In CONTINUOUS mode a write presets the counter to TOP and keeps
    /// counting; under a FIFO it has no effect (36.3.4).
    static void reset_timeout() { regs().TOCV = 0; }

    // ---- status and errors (36.3.5, 36.4.12, 36.4.13) -----------------------

    /// ONE read of FDCAN_PSR, decoded. The read is destructive by
    /// design: LEC and DLEC are `rs` (they read back as 7 afterwards)
    /// and RESI/RBRS/REDL/PXE are `rc_r`. So a caller that wants both
    /// the error code and the activity must take them from ONE call -
    /// which is why this returns a struct and there is no per-field
    /// accessor.
    static FdcanStatus status() {
        const uint32_t v = regs().PSR;
        FdcanStatus s{};
        s.activity = static_cast<FdcanActivity>((v & FDCAN_PSR_ACT_Msk) >>
                                                FDCAN_PSR_ACT_Pos);
        s.last_error = static_cast<FdcanError>(v & FDCAN_PSR_LEC_Msk);
        s.data_last_error = static_cast<FdcanError>((v & FDCAN_PSR_DLEC_Msk) >>
                                                    FDCAN_PSR_DLEC_Pos);
        s.bus_off = (v & FDCAN_PSR_BO) != 0u;
        s.warning = (v & FDCAN_PSR_EW) != 0u;
        s.error_passive = (v & FDCAN_PSR_EP) != 0u;
        s.received_esi = (v & FDCAN_PSR_RESI) != 0u;
        s.received_brs = (v & FDCAN_PSR_RBRS) != 0u;
        s.received_fd = (v & FDCAN_PSR_REDL) != 0u;
        s.protocol_exception = (v & FDCAN_PSR_PXE) != 0u;
        s.tdcv = static_cast<uint8_t>((v & FDCAN_PSR_TDCV_Msk) >> FDCAN_PSR_TDCV_Pos);
        return s;
    }

    /// ONE read of FDCAN_ECR - and the read CLEARS CEL (36.4.12's
    /// rc_r), which is why the logging counter comes back in the same
    /// struct and not from a verb of its own.
    static FdcanErrorCounters error_counters() {
        const uint32_t v = regs().ECR;
        FdcanErrorCounters e{};
        e.transmit = static_cast<uint8_t>(v & FDCAN_ECR_TEC_Msk);
        e.receive = static_cast<uint8_t>((v & FDCAN_ECR_REC_Msk) >> FDCAN_ECR_REC_Pos);
        e.receive_passive = (v & FDCAN_ECR_RP) != 0u;
        e.logging = static_cast<uint8_t>((v & FDCAN_ECR_CEL_Msk) >> FDCAN_ECR_CEL_Pos);
        return e;
    }

    /// The two read-only identity registers. CREL is the core's release
    /// and date; ENDN must read 0x87654321 or the bus is byte-swapped
    /// (36.4.2's own note).
    static uint32_t core_release() { return regs().CREL; }
    static uint32_t endianness() { return regs().ENDN; }

    // ---- interrupts (36.4.15 .. 36.4.18) ------------------------------------

    static uint32_t flags() { return regs().IR & FdcanFlag::all; }
    /// rc_w1: writing a one clears, writing a zero does nothing.
    static void clear_flags(uint32_t mask) { regs().IR = mask & FdcanFlag::all; }

    static bool interrupts(uint32_t mask, bool on) {
        if ((mask & ~FdcanFlag::all) != 0u) {
            return false;
        }
        FDCAN_GlobalTypeDef& r = regs();
        r.IE = on ? (r.IE | mask) : (r.IE & ~mask);
        return true;
    }
    static uint32_t interrupts() { return regs().IE; }

    /// FDCAN_ILS: which LINE a GROUP of flags is signalled on - seven
    /// groups and not thirty flags on this silicon (fact 6). `line` 0
    /// leaves the group's bit clear, `line` 1 sets it.
    static bool interrupt_line(uint32_t groups, uint8_t line) {
        if ((groups & ~FdcanGroup::all) != 0u || line > 1u) {
            return false;
        }
        FDCAN_GlobalTypeDef& r = regs();
        r.ILS = line == 1u ? (r.ILS | groups) : (r.ILS & ~groups);
        return true;
    }
    static uint32_t interrupt_line() { return regs().ILS & FdcanGroup::all; }

    /// FDCAN_ILE: the two lines themselves.
    static void interrupt_lines(bool line0, bool line1) {
        regs().ILE = (line0 ? FDCAN_ILE_EINT0 : 0u) | (line1 ? FDCAN_ILE_EINT1 : 0u);
    }
    static bool interrupt_line_enabled(uint8_t line) {
        const uint32_t bit = line == 0u ? FDCAN_ILE_EINT0 : FDCAN_ILE_EINT1;
        return (regs().ILE & bit) != 0u;
    }

    /// Every IR flag currently routed to `line` by ILS.
    static uint32_t flags_on_line(uint8_t line) {
        const uint32_t on_one = fdcan_flags_of(regs().ILS & FdcanGroup::all);
        return line == 1u ? on_one : (FdcanFlag::all & ~on_one);
    }

    /**
     * THE ISR BODY, one per line. The vector is SHARED three ways -
     * TIM16 (or TIM17), the other FDCAN instance, and this one - so the
     * body serves only the flags that are ENABLED, on ITS line, and
     * clears exactly what it returns. An app's handler calls every body
     * that can be on the line and each answers for its own.
     */
    [[gnu::always_inline]] static uint32_t isr(uint8_t line) {
        FDCAN_GlobalTypeDef& r = regs();
        const uint32_t served = r.IR & r.IE & flags_on_line(line) & FdcanFlag::all;
        if (served != 0u) {
            r.IR = served;
        }
        return served;
    }
    [[gnu::always_inline]] static uint32_t isr0() { return isr(0); }
    [[gnu::always_inline]] static uint32_t isr1() { return isr(1); }

    // ---- the whole sequence -------------------------------------------------

    /**
     * Clock the subsystem, reset it, zero the message RAM, take the
     * module into INIT + CCE and write every protected register of the
     * configuration. The module is left IN INIT: filters and Tx buffers
     * are written next, and `start()` puts it on the bus.
     *
     * NOT `reset()`: this verb does NOT pulse the subsystem reset,
     * because that would take the OTHER instance down with it (fact 2)
     * and a program bringing up both would then destroy the first while
     * entering the second. `Fdcan<n>::reset()` is the caller's, once,
     * before either `enter()`.
     *
     * False - with nothing written past the clock - when the
     * configuration breaks a rule, and false when a handshake times out.
     */
    static bool enter(const FdcanConfig& c) {
        if (!fdcan_config_valid(c)) {
            return false;
        }
        bus_clock(true);
        if (!init_mode(true) || !configuration(true)) {
            return false;
        }
        clear_ram();

        if (!nominal_timing(c.nominal)) {
            return false;
        }
        if (c.fd != FdcanFd::off) {
            if (!data_timing(c.data, c.transmitter_delay_compensation)) {
                return false;
            }
            if (!delay_compensation_offsets(c.tdc_offset, c.tdc_filter)) {
                return false;
            }
        }

        // The mode bits, all in CCCR, written as ONE store so a
        // half-configured module never exists between two writes.
        uint32_t cccr = regs().CCCR &
                        ~(FDCAN_CCCR_NISO | FDCAN_CCCR_TXP | FDCAN_CCCR_EFBI |
                          FDCAN_CCCR_PXHD | FDCAN_CCCR_BRSE | FDCAN_CCCR_FDOE |
                          FDCAN_CCCR_TEST | FDCAN_CCCR_DAR | FDCAN_CCCR_MON |
                          FDCAN_CCCR_ASM);
        const bool loopback = c.mode == FdcanMode::external_loop_back ||
                              c.mode == FdcanMode::internal_loop_back;
        const bool monitor = c.mode == FdcanMode::bus_monitor ||
                             c.mode == FdcanMode::internal_loop_back;
        const bool asm_bit = c.restricted || c.mode == FdcanMode::restricted;
        cccr |= c.non_iso ? FDCAN_CCCR_NISO : 0u;
        cccr |= c.transmit_pause ? FDCAN_CCCR_TXP : 0u;
        cccr |= c.edge_filtering ? FDCAN_CCCR_EFBI : 0u;
        cccr |= c.protocol_exception_disable ? FDCAN_CCCR_PXHD : 0u;
        cccr |= c.fd == FdcanFd::on_with_bit_rate_switch ? FDCAN_CCCR_BRSE : 0u;
        cccr |= c.fd != FdcanFd::off ? FDCAN_CCCR_FDOE : 0u;
        cccr |= loopback ? FDCAN_CCCR_TEST : 0u;
        cccr |= c.disable_auto_retransmit ? FDCAN_CCCR_DAR : 0u;
        cccr |= monitor ? FDCAN_CCCR_MON : 0u;
        cccr |= asm_bit ? FDCAN_CCCR_ASM : 0u;
        regs().CCCR = cccr;
        // FDCAN_TEST is only writable now that CCCR.TEST stands.
        regs().TEST = loopback ? FDCAN_TEST_LBCK : 0u;

        if (!filter_lists(c.standard_filters, c.extended_filters) ||
            !non_matching(c.non_matching_standard, c.non_matching_extended) ||
            !reject_remote(c.reject_remote_standard, c.reject_remote_extended) ||
            !fifo_overwrite(c.rx_fifo0_overwrite, c.rx_fifo1_overwrite) ||
            !extended_mask(c.extended_mask) ||
            !tx_queue_mode(c.tx_queue) ||
            !timestamp(c.timestamp, c.timestamp_prescaler) ||
            !timeout(c.timeout_mode, c.timeout_period, c.timeout_enable) ||
            !ram_watchdog(c.ram_watchdog)) {
            return false;
        }
        clear_flags(FdcanFlag::all);
        return true;
    }

    /// The compile-time twin (the lptim.hpp / reset.hpp spelling): the
    /// same sequence, with the rule a bad configuration broke named in
    /// the assertion.
    template <FdcanConfig c>
    static bool enter() {
        static_assert(fdcan_config_valid(c),
                      "brio Fdcan: illegal configuration - restricted operation (ASM) "
                      "must not be combined with either loop-back (36.3.4); the "
                      "nominal bit time must fit NBTP's fields and 36.4.7's 4..81 tq "
                      "window with SJW no longer than BS2; a data bit time must fit "
                      "DBTP's narrower fields, 36.3.4's 4 tq floor, and must not be "
                      "SLOWER than the nominal one (36.4.3); TDCO and TDCF are seven "
                      "bits; the filter lists hold at most 28 and 8 elements "
                      "(36.4.19); XIDAM is 29 bits; and the timestamp prescaler "
                      "counts 1..16 bit times (36.4.8)");
        return enter(c);
    }

    /// Clear INIT: the bit stream processor synchronizes on eleven
    /// consecutive recessive bits and the module is on the bus. CCE is
    /// cleared BY HARDWARE with it, which is what makes TXBAR writable.
    static bool start() { return init_mode(false); }

    /// Set INIT: transfers stop, TX goes recessive, the error counters
    /// are UNCHANGED and no configuration register moves (36.3.4).
    static bool stop() { return init_mode(true); }

    /// Give the module's pads back and take the subsystem's clock away.
    /// Does NOT pulse the reset, for the reason `enter()` does not.
    static void release() {
        stop();
        regs().IE = 0;
        regs().ILE = 0;
        bus_clock(false);
    }

private:
    static void write_cccr(uint32_t bit, bool on) {
        FDCAN_GlobalTypeDef& r = regs();
        r.CCCR = on ? (r.CCCR | bit) : (r.CCCR & ~bit);
    }
};

#endif // FDCAN1_BASE

} // namespace brio
