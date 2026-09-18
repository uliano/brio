/*
 * can.hpp
 *
 * The STM32F4's Basic Extended CAN controller (RM0090 ch. 32, RM0390
 * ch. 30): `Can<1|2|3>`, the RESOURCE over the whole chapter, with
 * `CanFrame`, `CanTiming` and `CanFilter` as its vocabulary.
 *
 * THE VOCABULARY IS SHARED, THE BUS AO IS NOT. `CanFrame`, `CanTiming`
 * and `CanError` are util/can.hpp's, written against this block, the
 * CH32V203's twin of it and the STM32G0's M_CAN (docs/design/can.md);
 * this file binds CAN_BTR's field widths into the shared search and
 * keeps what is the bxCAN's alone - the filter banks with their four
 * shapes, the mailbox's outcome - and offers no AO, no BusMaster policy
 * and no concept, for the reasons that page gives: a transmission's
 * completion is not a reply, and reception is routing.
 *
 * THREE INSTANCES, ONE FILTER BLOCK. CAN1 and CAN2 come as a pair on every
 * part that has CAN at all except the F413/F423, which add a third; the
 * F401, F410 and F411 have none. CAN1 is the MASTER: the filter registers
 * are its, and CAN2's filters are banks of CAN1's block above
 * CAN_FMR.CAN2SB - so filtering on CAN2 writes CAN1's registers and needs
 * CAN1'S CLOCK ON, which `filter_clock()` is for. CAN3, where it exists,
 * is a master with a block of its own; how many banks that block has is a
 * fact of RM0430, which is not on the desk, so every filter verb on
 * `Can<3>` refuses rather than write a bank that may not be there
 * (`can_filter_banks()` in the reserve).
 *
 * SEVEN FACTS THAT SHAPE THE FILE.
 *
 * 1. THREE MODES, AND EACH IS A REQUEST WITH AN ACKNOWLEDGE. INRQ and
 *    SLEEP are asked for in CAN_MCR; INAK and SLAK in CAN_MSR are the
 *    silicon saying it has arrived (30.4). Leaving initialization or sleep
 *    also costs a SYNCHRONIZATION - eleven consecutive recessive bits on
 *    CANRX - so `start()` can take a whole frame time to answer and every
 *    wait here is bounded and reports rather than hangs. THE RESET STATE
 *    IS SLEEP, not normal: a block whose clock was just turned on
 *    transmits nothing until it is woken.
 *
 * 2. THE BIT TIME IS COUNTED IN PCLK1 PERIODS. tq = (BRP + 1) x tPCLK, the
 *    bit is 1 + TS1 + TS2 quanta, and the sample point sits after the
 *    first 1 + TS1 of them (30.7.7). `can_timing_for()` searches that at
 *    COMPILE time for an EXACT bit rate - a CAN bus with a rounded bit
 *    rate is a bus that works until the third node joins - and reports the
 *    sample point it reached; a rate the APB clock cannot divide into
 *    exactly is an empty timing and a static_assert away from the
 *    application. CAN_BTR is writable ONLY in initialization mode
 *    (30.9.1), and so are the two test-mode bits that live in it.
 *
 * 3. LOOPBACK AND SILENT ARE THE WIRELESS INSTRUMENTS - AND LOOPBACK STILL
 *    WANTS ITS RECEIVE PAD. In loop back mode (LBKM) the node receives its
 *    own transmissions through an internal feedback from Tx to Rx, ignores
 *    acknowledge errors and, says 30.5.2, disregards "the actual value of
 *    the CANRX input pin" - which is true of the DATA and NOT of the
 *    SYNCHRONIZATION: measured, a node in loopback whose receive pad is
 *    held DOMINANT never leaves initialization mode, because leaving it
 *    costs eleven recessive bits and the counter watches the pad. So a
 *    loopback self-test needs no transceiver and no peer, but it does need
 *    CAN_RX claimed and held recessive - the pad's own pull-up will do,
 *    and CAN_TX can stay unclaimed so that the chip drives nothing.
 *    In silent mode (SILM) the node listens without ever driving a
 *    dominant bit and CANNOT START A TRANSMISSION AT ALL (30.5.1) - a
 *    transmit request there stays in its mailbox until it is aborted. The
 *    two together are the "hot self-test" of 30.5.3.
 *
 * 4. THREE TRANSMIT MAILBOXES AND TWO ORDERS. With more than one pending,
 *    the scheduler picks by IDENTIFIER (lowest wins, ties by mailbox
 *    number) or by REQUEST ORDER when MCR.TXFP is set (30.7.1). A mailbox
 *    is write-protected unless it is empty (TME), which is why
 *    `transmit()` takes the frame whole and refuses when there is no room,
 *    and RQCP is the completion flag whose clear also clears TXOK, ALST
 *    and TERR for that mailbox.
 *
 * 5. TWO RECEIVE FIFOS OF THREE, AND THE OVERRUN POLICY IS A CHOICE. FMP
 *    counts what is pending, FULL says three are, FOVR says a fourth
 *    arrived - and MCR.RFLM decides which message is lost: with the lock
 *    clear the newest overwrites the last stored one, with it set the
 *    newest is discarded and the three oldest survive (30.7.3). The output
 *    mailbox is released with RFOM, and until it is the next message is
 *    not reachable.
 *
 * 6. THE FILTER BANKS ARE A SCARCE SHARED RESOURCE WITH FOUR SHAPES. Each
 *    of the 28 banks is one 32-bit filter or two 16-bit ones (FS1R), in
 *    mask or in list mode (FM1R), assigned to FIFO 0 or FIFO 1 (FFA1R) and
 *    active or not (FA1R) - and the FILTER MATCH INDEX the receiver
 *    reports is a number over the ACTIVE filters of that FIFO in the
 *    manual's priority order (32-bit before 16-bit, list before mask, low
 *    bank before high), not the bank number. Configuration is legal only
 *    with FINIT set or the bank deactivated (30.9.1), and FINIT
 *    DEACTIVATES RECEPTION while it stands (30.4.1).
 *
 * 7. FOUR VECTORS PER INSTANCE. Transmit (the three mailboxes together),
 *    FIFO 0, FIFO 1, and status-change-and-error - the last one carrying
 *    the error flags, the wake-up and the sleep acknowledge, all behind
 *    ERRI/WKUI/SLAKI, which are rc_w1 (30.9.2). The three ISR bodies below
 *    read and clear at the top, as every ISR body in this stratum does.
 *
 * THE ONE ERRATUM, AND IT IS LIVE. ES0206 2.13.1 and ES0298 2.15.1, every
 * revision of the F427/F437/F429/F439 and of the F446: "bxCAN
 * time-triggered communication mode not supported ... timestamp values are
 * not available ... TTCM must be kept cleared", no workaround. So
 * `options()` REFUSES a configuration that asks for TTCM on those part
 * classes, `time_triggered_supported()` says whether it would, and the
 * TIME field a received frame carries is left in `CanFrame` but is
 * meaningless without the mode. On the part classes whose errata sheet was
 * not read the bit is writable and the document says which those are.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "util/can.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

// The frame, the timing and the error codes are the shared CAN vocabulary
// (util/can.hpp, docs/design/can.md); what follows binds this block's
// register widths into it and adds what is the bxCAN's alone - the
// filter banks and the mailbox's outcome.

/// What CAN_BTR holds of a timing (30.9.7): a ten-bit BRP, TS1 four
/// bits, TS2 three, SJW two - a bit of three to twenty-five quanta.
inline constexpr CanTimingLimits can_timing_limits{1024, 16, 8, 4, 3, 25};

/**
 * THE TIMING SEARCH for this block, at compile time: the EXACT
 * `bitrate_hz` from `pclk_hz` (CAN_BTR counts PCLK1 periods, 30.7.7),
 * with the sample point as close to `sample_permille` as the segment
 * limits allow - util/can.hpp's search under this block's limits, and an
 * empty timing when the APB clock cannot divide into the rate exactly.
 *
 *   constexpr auto t = brio::can_timing_for(brio::apb_hz(clock, false), 500'000);
 *   static_assert(brio::can_timing_valid(t), "no exact 500 kbit/s from this PCLK1");
 */
constexpr CanTiming can_timing_for(uint32_t pclk_hz, uint32_t bitrate_hz,
                                   uint16_t sample_permille = 875) {
    return can_timing_search(pclk_hz, bitrate_hz, can_timing_limits, sample_permille);
}

/// Whether a timing fits CAN_BTR.
constexpr bool can_timing_valid(const CanTiming& t) {
    return can_timing_fits(t, can_timing_limits);
}

// ---- filters -----------------------------------------------------------------

/// CAN_FS1R: one 32-bit filter per bank, or two 16-bit ones.
enum class CanFilterScale : uint8_t { dual16 = 0, single32 = 1 };

/// CAN_FM1R: the bank's two registers are an identifier and a mask, or two
/// (four, at 16 bits) identifiers to match exactly.
enum class CanFilterMode : uint8_t { mask = 0, list = 1 };

/**
 * One filter bank's whole configuration. `r1` and `r2` are the bank's two
 * 32-bit registers, laid out as figure 391 says for the scale and mode
 * chosen - `can_filter32()` and `can_filter16_pair()` build them.
 */
struct CanFilter {
    uint8_t bank = 0;
    CanFilterScale scale = CanFilterScale::single32;
    CanFilterMode mode = CanFilterMode::mask;
    uint8_t fifo = 0;
    uint32_t r1 = 0;
    uint32_t r2 = 0;
    bool active = true;
};

/**
 * A 32-bit filter word (figure 391, the "One 32-Bit Filter" row):
 * STID[10:0] or EXTID[28:18] in bits 31:21, EXID[17:0] in bits 20:3, IDE
 * in bit 2, RTR in bit 1, bit 0 zero. The same layout serves as the
 * IDENTIFIER and as the MASK - a mask word is built by passing the bits
 * that must match, so `can_filter32(0x7FF, false, false)` masks the whole
 * standard identifier and lets IDE and RTR through.
 */
constexpr uint32_t can_filter32(uint32_t id, bool extended, bool remote) {
    const uint32_t body = extended ? (id << 3) : (id << 21);
    return body | (extended ? (1u << 2) : 0u) | (remote ? (1u << 1) : 0u);
}

/// The same for the IDE and RTR bits alone, which is what a mask needs
/// when the identifier itself is a don't-care.
constexpr uint32_t can_filter32_flags_mask(bool match_ide, bool match_rtr) {
    return (match_ide ? (1u << 2) : 0u) | (match_rtr ? (1u << 1) : 0u);
}

/**
 * A 16-bit filter half (figure 391, the "16-Bit" rows): STID[10:0] in bits
 * 15:5, RTR in bit 4, IDE in bit 3, EXID[17:15] in bits 2:0. A 16-bit
 * filter can therefore only look at the top of an extended identifier -
 * which is the price of having four of them in one bank.
 */
constexpr uint16_t can_filter16(uint32_t id, bool extended, bool remote) {
    const uint32_t stid = extended ? (id >> 18) : id;
    const uint32_t ext_high = extended ? ((id >> 15) & 0x7u) : 0u;
    return static_cast<uint16_t>(((stid & 0x7FFu) << 5) | (remote ? (1u << 4) : 0u) |
                                 (extended ? (1u << 3) : 0u) | ext_high);
}

/// Two 16-bit halves into one bank register: the LOW half is the lower
/// filter number, the high half the next one - for a mask bank the low
/// half is the identifier and the high half its mask.
constexpr uint32_t can_filter16_pair(uint16_t low, uint16_t high) {
    return (static_cast<uint32_t>(high) << 16) | low;
}

/// The bank every program starts with: one 32-bit mask filter with a mask
/// of zero, which admits every frame to `fifo`.
constexpr CanFilter can_filter_accept_all(uint8_t bank, uint8_t fifo = 0) {
    return CanFilter{bank, CanFilterScale::single32, CanFilterMode::mask, fifo, 0u, 0u, true};
}

// ---- the rest of the vocabulary ------------------------------------------------

/// What a transmit mailbox's flags say once RQCP stands (30.9.2).
struct CanTxResult {
    bool completed = false;         ///< RQCP: the request (send or abort) is done
    bool ok = false;                ///< TXOK: it was sent and acknowledged
    bool arbitration_lost = false;  ///< ALST
    bool error = false;             ///< TERR
};

#if defined(CAN1_BASE)

/// The bits of CAN_IER, as one enumeration - the register is a flat set of
/// enables with no grouping of its own (30.9.2). Inside the presence guard
/// with the instance below it, because a part with no bxCAN has no CAN_IER
/// bit macros either; everything ABOVE this line - the frame, the timing
/// search, the filter words - is arithmetic and compiles on every part, so
/// that a portable program can name a bit rate on a part that cannot carry
/// the bus.
enum class CanInterrupt : uint32_t {
    tx_mailbox_empty = CAN_IER_TMEIE,
    fifo0_pending = CAN_IER_FMPIE0,
    fifo0_full = CAN_IER_FFIE0,
    fifo0_overrun = CAN_IER_FOVIE0,
    fifo1_pending = CAN_IER_FMPIE1,
    fifo1_full = CAN_IER_FFIE1,
    fifo1_overrun = CAN_IER_FOVIE1,
    error_warning = CAN_IER_EWGIE,
    error_passive = CAN_IER_EPVIE,
    bus_off = CAN_IER_BOFIE,
    last_error_code = CAN_IER_LECIE,
    error = CAN_IER_ERRIE,
    wakeup = CAN_IER_WKUIE,
    sleep_ack = CAN_IER_SLKIE,
};

/// The two pads a CAN node needs, each with the alternate function the
/// datasheet gives the signal there (AF9 for CAN1 and CAN2 on every part
/// class whose datasheet this project has read). A loopback self-test
/// needs neither pad and this stratum's suite claims none.
struct CanPins {
    PinSel rx;
    PinSel tx;
};

constexpr bool can_pins_valid(const CanPins& p) {
    return p.rx.valid() && p.tx.valid() && !(p.rx.port == p.tx.port && p.rx.pin == p.tx.pin);
}

/**
 * CAN_MCR's behaviour bits, as one configuration (30.9.2). The defaults
 * are the register's reset values, so a program states only what it wants
 * changed - and `time_triggered` is refused where the errata forbid it.
 */
struct CanOptions {
    bool time_triggered = false;  ///< TTCM: the 16-bit timer and the time stamps
    bool auto_bus_off = false;    ///< ABOM: leave bus-off without software
    bool auto_wakeup = false;     ///< AWUM: leave sleep on bus activity
    bool no_retransmit = false;   ///< NART: one attempt per request
    bool fifo_locked = false;     ///< RFLM: on overrun keep the three oldest
    bool tx_fifo_priority = false;///< TXFP: transmit in request order, not by id
    bool debug_freeze = true;     ///< DBF: stop with the core halted (the reset value)
};

// =============================================================================
// The instance
// =============================================================================

/**
 * One bxCAN instance.
 *
 *   using Bus = brio::Can<1>;
 *   constexpr auto timing = brio::can_timing_for(brio::apb_hz(clock, false), 500'000);
 *   Bus::clock(true);
 *   Bus::init_mode(true);
 *   Bus::timing(timing);                       // BTR is writable only here
 *   Bus::options({.auto_bus_off = true});
 *   Bus::filter_init(true);
 *   Bus::filter(brio::can_filter_accept_all(0));
 *   Bus::filter_init(false);
 *   Bus::start();                              // normal mode, synchronized
 *   Bus::transmit(frame);
 *
 * THE PADS ARE THE APPLICATION'S, as everywhere in this stratum:
 * `CanPins` names the two with the alternate function the datasheet gives
 * them (AF9 for CAN1 and CAN2 on every part of the family whose datasheet
 * is on the desk), and `claim_pads()` configures them. A loopback
 * self-test needs neither.
 */
template <uint8_t N>
struct Can {
    static_assert(can_present(N),
                  "brio Can: this device has no such bxCAN instance (the device header "
                  "declares no CANn_BASE for it: CAN1 and CAN2 are a pair on the F405 class "
                  "and up, CAN3 is the F413/F423's, and the F401, F410 and F411 have none)");

    Can() = delete;

    static constexpr uint8_t instance = N;

    static CAN_TypeDef& regs() { return *reinterpret_cast<CAN_TypeDef*>(can_base(N)); }

    /// The instance whose registers hold the filter banks this one
    /// filters through, and its block.
    static constexpr uint8_t filter_master = can_filter_master(N);
    static CAN_TypeDef& filter_regs() {
        return *reinterpret_cast<CAN_TypeDef*>(can_base(filter_master));
    }

    /// How many banks that block has, and 0 where this project has not
    /// read the manual that says (CAN3's).
    static constexpr uint8_t filter_banks = can_filter_banks(N);

    /// The four vectors (30.8).
    static constexpr IRQn_Type tx_irq() { return can_tx_irq(N); }
    static constexpr IRQn_Type rx_irq(uint8_t fifo) { return can_rx_irq(N, fifo); }
    static constexpr IRQn_Type sce_irq() { return can_sce_irq(N); }

    /// Whether CAN_MCR.TTCM may be set on this part class - false where an
    /// errata sheet this project read forbids it (the reserve's
    /// `can_ttcm_erratum()`).
    static constexpr bool time_triggered_supported() { return !can_ttcm_erratum(); }

    /// A bounded wait's budget. Leaving initialization or sleep costs
    /// eleven recessive bit times - 88 us at the family's slowest sensible
    /// rate - so a hundred thousand register reads is a fault and not a
    /// slow bus.
    static constexpr uint32_t ack_spins = 200'000u;

    // ---- the clock gates -----------------------------------------------------

    /// RCC_APB1ENR: this instance's own gate.
    static void clock(bool on) { Rcc::apb1_clock(can_clock_mask(N), on); }
    static bool clock() { return Rcc::apb1_clock(can_clock_mask(N)); }

    /// The gate on the block that holds this instance's FILTER banks -
    /// the same one for CAN1 and CAN3, and CAN1's for CAN2. A filter
    /// write with it shut is dropped in silence, which is the one trap
    /// the master/slave arrangement sets.
    static void filter_clock(bool on) { Rcc::apb1_clock(can_clock_mask(filter_master), on); }
    static bool filter_clock() { return Rcc::apb1_clock(can_clock_mask(filter_master)); }

    /// The peripheral's reset line: every register of this instance back
    /// to its reset value. On CAN1 that includes the filter block both
    /// instances share.
    static void reset_block() { Rcc::apb1_reset(can_reset_mask(N)); }

    // ---- the pads ------------------------------------------------------------

    /**
     * Both pads into their alternate function - TX push-pull, RX with the
     * pull the caller names (a transceiver drives RX, so `none` is right
     * with one attached and `up` is what holds the input recessive with
     * nothing attached).
     *
     * The pins are a TEMPLATE argument because a `Pin<port, number>` is a
     * type: the same arrangement as the Uart and SpiHost tasks', with the
     * AF number the application's to read off the datasheet.
     */
    template <CanPins pins>
    static void claim_pads(PinPull rx_pull = PinPull::none,
                           PinSpeed tx_speed = PinSpeed::high) {
        static_assert(can_pins_valid(pins),
                      "brio Can: the two pads must be on ports this package bonds, and must "
                      "not be the same pad");
        Pin<pins.tx.port, pins.tx.pin>::function(pins.tx.function, {.speed = tx_speed});
        Pin<pins.rx.port, pins.rx.pin>::function(pins.rx.function, {.pull = rx_pull});
    }

    /// ONE pad, the receiver's - what a node that only listens needs, and
    /// what makes a bus of the pad's own pull-up for a program measuring
    /// what happens when nobody acknowledges.
    template <PinSel rx>
    static void claim_rx_pad(PinPull pull = PinPull::up) {
        static_assert(rx.valid(),
                      "brio Can: that pad is not on a port this package bonds");
        Pin<rx.port, rx.pin>::function(rx.function, {.pull = pull});
    }

    // ---- the three modes (30.4) -----------------------------------------------

    static bool in_init() { return (regs().MSR & CAN_MSR_INAK) != 0u; }
    static bool in_sleep() { return (regs().MSR & CAN_MSR_SLAK) != 0u; }
    /// Neither acknowledged: the block is taking part in bus activity.
    static bool in_normal() { return (regs().MSR & (CAN_MSR_INAK | CAN_MSR_SLAK)) == 0u; }

    /**
     * Enter or leave initialization mode, waiting for INAK to agree.
     * Entering also clears SLEEP, because 30.4.3 asks for it: a request to
     * initialize from sleep is refused unless the sleep bit goes down with
     * it. Leaving initialization is what `start()` does with a name that
     * says so.
     */
    static bool init_mode(bool on) {
        if (on) {
            regs().MCR = (regs().MCR & ~CAN_MCR_SLEEP) | CAN_MCR_INRQ;
        } else {
            regs().MCR &= ~CAN_MCR_INRQ;
        }
        return wait(CAN_MSR_INAK, on);
    }

    /// Leave initialization and take part in the bus - the same store as
    /// `init_mode(false)`, under the name the sequence deserves. It costs
    /// the synchronization of eleven recessive bits, which on a bus with
    /// no other node and no loopback never arrives: the return is what
    /// says so.
    static bool start() { return init_mode(false); }

    /// CAN_MCR.SLEEP: the block's clock stopped, its mailboxes still
    /// readable. Waits for SLAK. Leaving sleep also synchronizes.
    static bool sleep(bool on) {
        if (on) {
            regs().MCR |= CAN_MCR_SLEEP;
        } else {
            regs().MCR &= ~CAN_MCR_SLEEP;
        }
        return wait(CAN_MSR_SLAK, on);
    }

    /// CAN_MCR.RESET: a master reset of the block - every register to its
    /// reset value and SLEEP mode entered - through the peripheral's own
    /// bit rather than through RCC. Self-clearing.
    static void master_reset() { regs().MCR |= CAN_MCR_RESET; }

    /**
     * The behaviour bits of CAN_MCR, all at once. REFUSED WHOLE (false,
     * nothing written) when `time_triggered` is asked for on a part class
     * whose errata sheet forbids it - the mode does not work there and a
     * program that thinks it has time stamps is worse off than one told no.
     */
    static bool options(const CanOptions& o) {
        if (o.time_triggered && !time_triggered_supported()) {
            return false;
        }
        uint32_t v = regs().MCR & (CAN_MCR_INRQ | CAN_MCR_SLEEP);
        v |= o.time_triggered ? CAN_MCR_TTCM : 0u;
        v |= o.auto_bus_off ? CAN_MCR_ABOM : 0u;
        v |= o.auto_wakeup ? CAN_MCR_AWUM : 0u;
        v |= o.no_retransmit ? CAN_MCR_NART : 0u;
        v |= o.fifo_locked ? CAN_MCR_RFLM : 0u;
        v |= o.tx_fifo_priority ? CAN_MCR_TXFP : 0u;
        v |= o.debug_freeze ? CAN_MCR_DBF : 0u;
        regs().MCR = v;
        return true;
    }

    /// What CAN_MCR holds now.
    static CanOptions options() {
        const uint32_t v = regs().MCR;
        return CanOptions{(v & CAN_MCR_TTCM) != 0u, (v & CAN_MCR_ABOM) != 0u,
                          (v & CAN_MCR_AWUM) != 0u, (v & CAN_MCR_NART) != 0u,
                          (v & CAN_MCR_RFLM) != 0u, (v & CAN_MCR_TXFP) != 0u,
                          (v & CAN_MCR_DBF) != 0u};
    }

    // ---- the bit timing and the test modes (30.9.2) -----------------------------

    /**
     * CAN_BTR: the bit time and the two test-mode bits, which live in the
     * same register and are therefore written together. REFUSED unless the
     * block is in initialization mode (30.9.1 makes the register writable
     * only there, and a write outside it is dropped in silence) or the
     * timing is not a legal one.
     */
    static bool timing(const CanTiming& t, bool loopback = false, bool silent = false) {
        if (!can_timing_valid(t) || !in_init()) {
            return false;
        }
        regs().BTR = (static_cast<uint32_t>(t.brp - 1u) << CAN_BTR_BRP_Pos) |
                     (static_cast<uint32_t>(t.ts1 - 1u) << CAN_BTR_TS1_Pos) |
                     (static_cast<uint32_t>(t.ts2 - 1u) << CAN_BTR_TS2_Pos) |
                     (static_cast<uint32_t>(t.sjw - 1u) << CAN_BTR_SJW_Pos) |
                     (loopback ? CAN_BTR_LBKM : 0u) | (silent ? CAN_BTR_SILM : 0u);
        return true;
    }

    /// What CAN_BTR holds now, in the same human units.
    static CanTiming timing() {
        const uint32_t v = regs().BTR;
        CanTiming t{};
        t.brp = static_cast<uint16_t>(((v & CAN_BTR_BRP_Msk) >> CAN_BTR_BRP_Pos) + 1u);
        t.ts1 = static_cast<uint8_t>(((v & CAN_BTR_TS1_Msk) >> CAN_BTR_TS1_Pos) + 1u);
        t.ts2 = static_cast<uint8_t>(((v & CAN_BTR_TS2_Msk) >> CAN_BTR_TS2_Pos) + 1u);
        t.sjw = static_cast<uint8_t>(((v & CAN_BTR_SJW_Msk) >> CAN_BTR_SJW_Pos) + 1u);
        return t;
    }

    static bool loopback() { return (regs().BTR & CAN_BTR_LBKM) != 0u; }
    static bool silent() { return (regs().BTR & CAN_BTR_SILM) != 0u; }

    // ---- transmission (30.7.1, 30.9.3) -------------------------------------------

    /// TSR.TMEx: mailbox `mb` (0..2) holds no pending request and may be
    /// written.
    static bool mailbox_empty(uint8_t mb) {
        return mb < 3u && (regs().TSR & (CAN_TSR_TME0 << mb)) != 0u;
    }

    /// TSR.CODE[1:0]: the number of the next free mailbox, or - with all
    /// three pending - the one with the lowest priority.
    static uint8_t next_mailbox() {
        return static_cast<uint8_t>((regs().TSR & CAN_TSR_CODE_Msk) >> CAN_TSR_CODE_Pos);
    }

    /// How many of the three are free.
    static uint8_t free_mailboxes() {
        const uint32_t tsr = regs().TSR;
        return static_cast<uint8_t>(((tsr & CAN_TSR_TME0) != 0u ? 1u : 0u) +
                                    ((tsr & CAN_TSR_TME1) != 0u ? 1u : 0u) +
                                    ((tsr & CAN_TSR_TME2) != 0u ? 1u : 0u));
    }

    /// TSR.LOWx: with more than one pending, this mailbox is the lowest
    /// priority of them. Zero when only one is pending (30.9.2).
    static bool lowest_priority(uint8_t mb) {
        return mb < 3u && (regs().TSR & (CAN_TSR_LOW0 << mb)) != 0u;
    }

    /**
     * Put a frame in a free mailbox and request its transmission; the
     * mailbox number, or nothing when all three are pending or the frame
     * is malformed.
     *
     * The order is the chapter's: the data and the length first, the
     * identifier register LAST with TXRQ in the same store - a mailbox is
     * write-protected the moment it stops being empty, and TXRQ is what
     * ends its emptiness.
     */
    static std::optional<uint8_t> transmit(const CanFrame& f) {
        if (!can_frame_valid(f)) {
            return std::nullopt;
        }
        const uint8_t mb = next_mailbox();
        if (mb > 2u || !mailbox_empty(mb)) {
            return std::nullopt;
        }
        CAN_TxMailBox_TypeDef& box = regs().sTxMailBox[mb];
        box.TDTR = (box.TDTR & ~static_cast<uint32_t>(CAN_TDT0R_DLC)) | f.length;
        box.TDLR = static_cast<uint32_t>(f.data[0]) | (static_cast<uint32_t>(f.data[1]) << 8) |
                   (static_cast<uint32_t>(f.data[2]) << 16) |
                   (static_cast<uint32_t>(f.data[3]) << 24);
        box.TDHR = static_cast<uint32_t>(f.data[4]) | (static_cast<uint32_t>(f.data[5]) << 8) |
                   (static_cast<uint32_t>(f.data[6]) << 16) |
                   (static_cast<uint32_t>(f.data[7]) << 24);
        box.TIR = (f.extended ? ((f.id << 3) | CAN_TI0R_IDE) : (f.id << 21)) |
                  (f.remote ? CAN_TI0R_RTR : 0u) | CAN_TI0R_TXRQ;
        return mb;
    }

    /// TSR.ABRQx: give up on a pending request. A mailbox in the middle of
    /// a transmission empties at the end of it either way (30.7.1).
    static bool abort(uint8_t mb) {
        if (mb > 2u) {
            return false;
        }
        regs().TSR = CAN_TSR_ABRQ0 << (8u * mb);
        return true;
    }

    /// RQCP and the three flags it governs, for one mailbox.
    static CanTxResult result(uint8_t mb) {
        CanTxResult r{};
        if (mb > 2u) {
            return r;
        }
        const uint32_t tsr = regs().TSR >> (8u * mb);
        r.completed = (tsr & CAN_TSR_RQCP0) != 0u;
        r.ok = (tsr & CAN_TSR_TXOK0) != 0u;
        r.arbitration_lost = (tsr & CAN_TSR_ALST0) != 0u;
        r.error = (tsr & CAN_TSR_TERR0) != 0u;
        return r;
    }

    /// Clear RQCP - which clears TXOK, ALST and TERR for that mailbox too
    /// (30.9.2). rc_w1, so a plain store of the one bit.
    static bool clear_result(uint8_t mb) {
        if (mb > 2u) {
            return false;
        }
        regs().TSR = CAN_TSR_RQCP0 << (8u * mb);
        return true;
    }

    // ---- reception (30.7.3, 30.9.3) ------------------------------------------------

    /// FMP: how many messages are waiting in FIFO `fifo` (0..3).
    static uint8_t pending(uint8_t fifo) {
        return static_cast<uint8_t>(fifo_reg(fifo) & CAN_RF0R_FMP0);
    }
    /// FULL: three are.
    static bool full(uint8_t fifo) { return (fifo_reg(fifo) & CAN_RF0R_FULL0) != 0u; }
    /// FOVR: a fourth arrived and one message was lost - which one is
    /// MCR.RFLM's business.
    static bool overrun(uint8_t fifo) { return (fifo_reg(fifo) & CAN_RF0R_FOVR0) != 0u; }

    static void clear_full(uint8_t fifo) { fifo_reg(fifo) = CAN_RF0R_FULL0; }
    static void clear_overrun(uint8_t fifo) { fifo_reg(fifo) = CAN_RF0R_FOVR0; }

    /// The output mailbox WITHOUT releasing it - for a handler that wants
    /// to look before it takes.
    static std::optional<CanFrame> peek(uint8_t fifo) {
        if (fifo > 1u || pending(fifo) == 0u) {
            return std::nullopt;
        }
        const CAN_FIFOMailBox_TypeDef& box = regs().sFIFOMailBox[fifo];
        const uint32_t rir = box.RIR;
        const uint32_t rdtr = box.RDTR;
        CanFrame f{};
        f.extended = (rir & CAN_RI0R_IDE) != 0u;
        f.id = f.extended ? (rir >> 3) : (rir >> 21);
        f.remote = (rir & CAN_RI0R_RTR) != 0u;
        f.length = static_cast<uint8_t>(rdtr & CAN_RDT0R_DLC);
        f.filter_index = static_cast<uint8_t>((rdtr & CAN_RDT0R_FMI) >> CAN_RDT0R_FMI_Pos);
        f.timestamp = static_cast<uint16_t>((rdtr & CAN_RDT0R_TIME) >> CAN_RDT0R_TIME_Pos);
        const uint32_t low = box.RDLR;
        const uint32_t high = box.RDHR;
        f.data[0] = static_cast<uint8_t>(low);
        f.data[1] = static_cast<uint8_t>(low >> 8);
        f.data[2] = static_cast<uint8_t>(low >> 16);
        f.data[3] = static_cast<uint8_t>(low >> 24);
        f.data[4] = static_cast<uint8_t>(high);
        f.data[5] = static_cast<uint8_t>(high >> 8);
        f.data[6] = static_cast<uint8_t>(high >> 16);
        f.data[7] = static_cast<uint8_t>(high >> 24);
        return f;
    }

    /// RFOM: release the output mailbox so the next message becomes
    /// reachable. Setting it on an empty FIFO has no effect (30.9.2).
    static bool release(uint8_t fifo) {
        if (fifo > 1u) {
            return false;
        }
        fifo_reg(fifo) = CAN_RF0R_RFOM0;
        return true;
    }

    /// The common case: take the output mailbox and release it.
    static std::optional<CanFrame> receive(uint8_t fifo) {
        const auto f = peek(fifo);
        if (f) {
            (void)release(fifo);
        }
        return f;
    }

    // ---- errors (30.7.6, 30.9.2) ----------------------------------------------------

    /// The transmit error counter's low byte - the counter is nine bits
    /// wide and this register carries eight of them, so a node that has
    /// just gone bus-off (TEC above 255) reads a SMALL number here.
    static uint8_t tec() {
        return static_cast<uint8_t>((regs().ESR & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    }
    static uint8_t rec() {
        return static_cast<uint8_t>((regs().ESR & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
    }

    /// EWGF: a counter has reached 96. EPVF: one has passed 127 and the
    /// node is error passive. BOFF: TEC passed 255 and the node is off the
    /// bus, transmitting and receiving nothing.
    static bool error_warning() { return (regs().ESR & CAN_ESR_EWGF) != 0u; }
    static bool error_passive() { return (regs().ESR & CAN_ESR_EPVF) != 0u; }
    static bool bus_off() { return (regs().ESR & CAN_ESR_BOFF) != 0u; }

    /// The three flags folded into the vocabulary's one observable, and
    /// the two counters as its pair - one read of CAN_ESR each.
    static CanErrorState error_state() {
        const uint32_t esr = regs().ESR;
        return can_error_state((esr & CAN_ESR_EWGF) != 0u, (esr & CAN_ESR_EPVF) != 0u,
                               (esr & CAN_ESR_BOFF) != 0u);
    }
    static CanErrorCounters error_counters() {
        const uint32_t esr = regs().ESR;
        return CanErrorCounters{static_cast<uint8_t>((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos),
                                static_cast<uint8_t>((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos)};
    }

    static CanError last_error() {
        return static_cast<CanError>((regs().ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
    }
    /// Write the "set by software" code, so that a later read tells a
    /// fresh error from a stale one. The field is the one writable part of
    /// CAN_ESR.
    static void mark_error() {
        regs().ESR = (regs().ESR & ~static_cast<uint32_t>(CAN_ESR_LEC)) |
                     (static_cast<uint32_t>(CanError::no_change) << CAN_ESR_LEC_Pos);
    }

    /**
     * Ask for the bus-off recovery a node with ABOM clear needs: set and
     * clear INRQ (30.7.6). The 128 x 11 recessive bits the standard
     * demands are counted AFTER that, in normal mode - in initialization
     * mode the block does not watch CANRX at all - so this verb starts the
     * recovery and does not finish it, and `bus_off()` is what says when
     * it has.
     */
    static bool request_bus_off_recovery() {
        if (!init_mode(true)) {
            return false;
        }
        return start();
    }

    // ---- the master status register (30.9.2) ------------------------------------

    /// The actual level of the CAN_RX pin, and the level at the last
    /// sample point - two read-only windows onto the wire that need no
    /// frame to be useful (a bus stuck dominant reads 0 here).
    static bool rx_level() { return (regs().MSR & CAN_MSR_RX) != 0u; }
    static bool last_sample() { return (regs().MSR & CAN_MSR_SAMP) != 0u; }
    static bool receiving() { return (regs().MSR & CAN_MSR_RXM) != 0u; }
    static bool transmitting() { return (regs().MSR & CAN_MSR_TXM) != 0u; }

    /// The three rc_w1 event flags of CAN_MSR: an error condition became
    /// pending, a start of frame was seen in sleep, sleep was entered.
    static bool error_flag() { return (regs().MSR & CAN_MSR_ERRI) != 0u; }
    static bool wakeup_flag() { return (regs().MSR & CAN_MSR_WKUI) != 0u; }
    static bool sleep_ack_flag() { return (regs().MSR & CAN_MSR_SLAKI) != 0u; }
    static void clear_error_flag() { regs().MSR = CAN_MSR_ERRI; }
    static void clear_wakeup_flag() { regs().MSR = CAN_MSR_WKUI; }
    static void clear_sleep_ack_flag() { regs().MSR = CAN_MSR_SLAKI; }

    // ---- interrupts (30.9.2) ----------------------------------------------------

    static void interrupt(CanInterrupt i, bool on) {
        const uint32_t m = static_cast<uint32_t>(i);
        regs().IER = on ? (regs().IER | m) : (regs().IER & ~m);
    }
    static bool interrupt(CanInterrupt i) {
        return (regs().IER & static_cast<uint32_t>(i)) != 0u;
    }
    /// Every enable off - what a teardown wants, and what keeps a shared
    /// filter block's other instance from raising a vector nobody binds.
    static void interrupts_off() { regs().IER = 0; }

    // ---- the filter block (30.9.4) -----------------------------------------------

    /// CAN_FMR.FINIT. WHILE IT STANDS, RECEPTION IS DEACTIVATED on every
    /// instance the block serves (30.4.1) - it is a configuration window
    /// and not a mode to live in.
    static bool filter_init(bool on) {
        if (filter_banks == 0u) {
            return false;
        }
        CAN_TypeDef& f = filter_regs();
        f.FMR = on ? (f.FMR | CAN_FMR_FINIT) : (f.FMR & ~CAN_FMR_FINIT);
        return true;
    }
    static bool filter_initializing() {
        return filter_banks != 0u && (filter_regs().FMR & CAN_FMR_FINIT) != 0u;
    }

    /**
     * CAN_FMR.CAN2SB, the bank where CAN1's filters end and CAN2's begin -
     * the one number the two instances have to agree on, and a
     * DUAL-INSTANCE fact: on a part with CAN1 alone, or on CAN3's own
     * block, there is nothing to split and this verb refuses.
     * The reset value is 14 (CAN_FMR's reset value 0x2A1C0E01).
     */
    static bool start_bank(uint8_t bank) {
        if (!can_dual() || filter_banks == 0u || bank > filter_banks) {
            return false;
        }
        CAN_TypeDef& f = filter_regs();
        f.FMR = (f.FMR & ~static_cast<uint32_t>(CAN_FMR_CAN2SB)) |
                (static_cast<uint32_t>(bank) << CAN_FMR_CAN2SB_Pos);
        return true;
    }
    static uint8_t start_bank() {
        if (!can_dual() || filter_banks == 0u) {
            return 0;
        }
        return static_cast<uint8_t>((filter_regs().FMR & CAN_FMR_CAN2SB) >> CAN_FMR_CAN2SB_Pos);
    }

    /// The first and last+1 bank THIS instance's frames are filtered
    /// through: below CAN2SB for CAN1 of a pair, from CAN2SB up for CAN2,
    /// and the whole block for an instance that shares with nobody.
    static uint8_t first_bank() {
        if (N == 2u) {
            return start_bank();
        }
        return 0;
    }
    static uint8_t bank_limit() {
        if (N == 1u && can_dual()) {
            return start_bank();
        }
        return filter_banks;
    }
    /// Is `bank` one this instance may configure? The split is enforced
    /// here because the silicon does not: a write to a bank on the other
    /// side of CAN2SB lands, and the frames it admits arrive at the other
    /// instance.
    static bool bank_ok(uint8_t bank) {
        return filter_banks != 0u && bank < bank_limit() && bank >= first_bank();
    }

    /**
     * One filter bank, whole: the scale, the mode, the FIFO, the two
     * registers and the activation. REFUSED when the bank is not this
     * instance's, when the block's bank count is not known (CAN3), or when
     * the FIFO is not 0 or 1.
     *
     * The order is 30.9.1's: the bank is DEACTIVATED first (its registers
     * are writable only then, or under FINIT), the three configuration
     * bits and the values are written, and the activation comes last.
     */
    static bool filter(const CanFilter& c) {
        if (!bank_ok(c.bank) || c.fifo > 1u) {
            return false;
        }
        CAN_TypeDef& f = filter_regs();
        const uint32_t b = 1UL << c.bank;
        f.FA1R &= ~b;
        f.FS1R = c.scale == CanFilterScale::single32 ? (f.FS1R | b) : (f.FS1R & ~b);
        f.FM1R = c.mode == CanFilterMode::list ? (f.FM1R | b) : (f.FM1R & ~b);
        f.FFA1R = c.fifo == 1u ? (f.FFA1R | b) : (f.FFA1R & ~b);
        f.sFilterRegister[c.bank].FR1 = c.r1;
        f.sFilterRegister[c.bank].FR2 = c.r2;
        if (c.active) {
            f.FA1R |= b;
        }
        return true;
    }

    /// What a bank holds now, whatever wrote it.
    static std::optional<CanFilter> filter(uint8_t bank) {
        if (!bank_ok(bank)) {
            return std::nullopt;
        }
        CAN_TypeDef& f = filter_regs();
        const uint32_t b = 1UL << bank;
        CanFilter c{};
        c.bank = bank;
        c.scale = (f.FS1R & b) != 0u ? CanFilterScale::single32 : CanFilterScale::dual16;
        c.mode = (f.FM1R & b) != 0u ? CanFilterMode::list : CanFilterMode::mask;
        c.fifo = (f.FFA1R & b) != 0u ? 1u : 0u;
        c.r1 = f.sFilterRegister[bank].FR1;
        c.r2 = f.sFilterRegister[bank].FR2;
        c.active = (f.FA1R & b) != 0u;
        return c;
    }

    /// CAN_FA1R for one bank, on its own - the cheap way to silence a
    /// filter without rewriting it.
    static bool filter_active(uint8_t bank, bool on) {
        if (!bank_ok(bank)) {
            return false;
        }
        CAN_TypeDef& f = filter_regs();
        const uint32_t b = 1UL << bank;
        f.FA1R = on ? (f.FA1R | b) : (f.FA1R & ~b);
        return true;
    }
    static bool filter_active(uint8_t bank) {
        return bank_ok(bank) && (filter_regs().FA1R & (1UL << bank)) != 0u;
    }

    /// Every bank of THIS instance deactivated - the state 30.4.1
    /// recommends for a filter nobody uses, and the one a suite starts
    /// each of its cases from.
    static bool filters_off() {
        if (filter_banks == 0u) {
            return false;
        }
        for (uint8_t bank = first_bank(); bank < bank_limit(); ++bank) {
            filter_regs().FA1R &= ~(1UL << bank);
        }
        return true;
    }

    // ---- the ISR bodies ----------------------------------------------------------

    /// What the transmit vector found: the three mailboxes' results, with
    /// RQCP cleared for each one that had completed (which clears its
    /// TXOK, ALST and TERR with it, so the results are read BEFORE the
    /// clear).
    struct CanTxEvent {
        CanTxResult mailbox[3];
    };

    [[gnu::always_inline]] static CanTxEvent tx_isr() {
        CanTxEvent e{};
        uint32_t clear = 0;
        for (uint8_t mb = 0; mb < 3u; ++mb) {
            e.mailbox[mb] = result(mb);
            if (e.mailbox[mb].completed) {
                clear |= CAN_TSR_RQCP0 << (8u * mb);
            }
        }
        if (clear != 0u) {
            regs().TSR = clear;
        }
        return e;
    }

    /// What a receive vector found. The MESSAGES are not taken here - a
    /// FIFO's output mailbox is the handler's to read with `receive()`, as
    /// many times as `pending` says - but FULL and FOVR are cleared,
    /// because they are level flags that would re-enter the vector
    /// immediately.
    struct CanRxEvent {
        uint8_t pending = 0;
        bool full = false;
        bool overrun = false;
    };

    [[gnu::always_inline]] static CanRxEvent rx_isr(uint8_t fifo) {
        CanRxEvent e{};
        if (fifo > 1u) {
            return e;
        }
        const uint32_t r = fifo_reg(fifo);
        e.pending = static_cast<uint8_t>(r & CAN_RF0R_FMP0);
        e.full = (r & CAN_RF0R_FULL0) != 0u;
        e.overrun = (r & CAN_RF0R_FOVR0) != 0u;
        const uint32_t clear = (e.full ? CAN_RF0R_FULL0 : 0u) | (e.overrun ? CAN_RF0R_FOVR0 : 0u);
        if (clear != 0u) {
            fifo_reg(fifo) = clear;
        }
        return e;
    }

    /// What the status-change-and-error vector found: the error picture as
    /// CAN_ESR holds it, plus the two events that share the vector. The
    /// three rc_w1 flags of CAN_MSR are cleared at the top; CAN_ESR's own
    /// bits are STATUSES and clear themselves when the condition passes.
    struct CanStatusEvent {
        bool error = false;
        bool wakeup = false;
        bool sleep_ack = false;
        bool warning = false;
        bool passive = false;
        bool bus_off = false;
        CanError last_error = CanError::none;
        uint8_t tec = 0;
        uint8_t rec = 0;
    };

    [[gnu::always_inline]] static CanStatusEvent sce_isr() {
        const uint32_t msr = regs().MSR;
        CanStatusEvent e{};
        e.error = (msr & CAN_MSR_ERRI) != 0u;
        e.wakeup = (msr & CAN_MSR_WKUI) != 0u;
        e.sleep_ack = (msr & CAN_MSR_SLAKI) != 0u;
        const uint32_t clear = (e.error ? CAN_MSR_ERRI : 0u) | (e.wakeup ? CAN_MSR_WKUI : 0u) |
                               (e.sleep_ack ? CAN_MSR_SLAKI : 0u);
        if (clear != 0u) {
            regs().MSR = clear;
        }
        const uint32_t esr = regs().ESR;
        e.warning = (esr & CAN_ESR_EWGF) != 0u;
        e.passive = (esr & CAN_ESR_EPVF) != 0u;
        e.bus_off = (esr & CAN_ESR_BOFF) != 0u;
        e.last_error = static_cast<CanError>((esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
        e.tec = static_cast<uint8_t>((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
        e.rec = static_cast<uint8_t>((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
        return e;
    }

    // ---- teardown -------------------------------------------------------------------

    /// This instance's own registers back to their reset values (CAN_MCR's
    /// RESET bit, which also puts it in sleep) and its clock off. The
    /// FILTER block is not touched: on CAN2 it is not this instance's, and
    /// on CAN1 it is shared.
    static void release_instance() {
        interrupts_off();
        master_reset();
        clock(false);
    }

private:
    static volatile uint32_t& fifo_reg(uint8_t fifo) {
        return fifo == 0u ? regs().RF0R : regs().RF1R;
    }

    static bool wait(uint32_t msr_bit, bool set) {
        for (uint32_t spins = 0; spins < ack_spins; ++spins) {
            if (((regs().MSR & msr_bit) != 0u) == set) {
                return true;
            }
        }
        return false;
    }
};

#endif // CAN1_BASE

} // namespace brio
