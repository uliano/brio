/*
 * dmac.hpp
 *
 * The SAM C21 Direct Memory Access Controller (DS60001479M ch. 25 - NOT
 * ch. 19, which is where an older revision's numbering put it), in the
 * two strata the rest of this target uses:
 *
 *  Dmac        the BLOCK resource - the one AHB clock it needs, the
 *              enable/reset discipline, the two descriptor tables it
 *              cannot work without, the arbiter's four priority levels,
 *              and the CHID-free interrupt dispatch (see below).
 *
 *  DmaDescriptor / dma_descriptor(DmaTransfer)
 *              the SRAM transfer descriptor as a constexpr VALUE, and
 *              the builder that turns a start pointer plus a beat count
 *              into the register words - END-ADDRESS ARITHMETIC INCLUDED,
 *              because that is the one place this peripheral bites.
 *
 *  DmaChannel<n>
 *              one channel: trigger source and action, priority, the
 *              descriptor slot, enable/suspend/resume, the software
 *              trigger, the interrupt arming, and harvest() - the only
 *              way to read mid-block progress, with erratum 1.10.4's
 *              write-back corruption checked rather than trusted.
 *
 *  DmaTxEngine<ch> / DmaRxEngine<ch>
 *              the two peripheral engines samc/sercom.hpp's Uart takes
 *              as OPTIONAL policies. They live here, not there, so that
 *              sercom.hpp never includes this header and a program that
 *              names no engine cannot pay for one (see "ZERO WHEN
 *              ABSENT" below).
 *
 * =========================================================================
 * THE CHANNEL REGISTERS SIT BEHIND A SELECTOR. This is the load-bearing
 * fact of the whole driver.
 *
 * CHCTRLA, CHCTRLB, CHINTENCLR, CHINTENSET, CHINTFLAG and CHSTATUS are
 * ONE register set each, not twelve: which channel they talk to is
 * whatever was last written to CHID (25.8.17). Every channel access is
 * therefore TWO accesses - select, then use - and the pair is not
 * atomic. Nothing about that would matter in a single-threaded program;
 * it matters here because the DMAC's own interrupt handler re-arms
 * channels (a software-linked chain, the TX engine's next block), so an
 * ISR taking CHID in the middle of main context's select-then-use would
 * silently redirect main's access to the handler's channel.
 *
 * So EVERY CHID access in this file goes through Dmac::with_channel(),
 * which holds SamPlatform's CriticalSection (PRIMASK) across select and
 * use. There are no exceptions, and DmaChannel is a friend of Dmac
 * precisely so that there CANNOT be: nothing outside these two types can
 * reach CHID at all. The guard is three instructions on this core
 * (mrs/cpsid/msr) and is paid by ISR paths too - a handler that skipped
 * it would corrupt main context, which is the wrong side of the trade.
 *
 * THE ONE CHANNEL-ADDRESSED ACCESS THAT NEEDS NO GUARD is INTPEND
 * (25.8.10): a single 16-bit register carrying the LOWEST channel number
 * with a pending interrupt together with that channel's SUSP/TCMPL/TERR
 * flags, where a write of {flags, id} clears those flags FOR THAT ID.
 * One read tells the handler which channel and why; one store
 * acknowledges it. Because it is one register and one access, it cannot
 * be split by an interrupt and needs no critical section - which is what
 * makes Dmac::take_pending() the natural shape for the ISR body, and why
 * the handler does not fight main context for CHID at all.
 *
 * =========================================================================
 * DESCRIPTOR MEMORY IS THE DRIVER'S TO PROVIDE. The DMAC fetches every
 * descriptor from SRAM: BASEADDR points at the array of FIRST descriptors
 * (one per channel, indexed by channel number) and WRBADDR at the
 * write-back array where the controller stores each channel's ONGOING
 * descriptor (25.6.2.3). Both are here as static tables of DMAC_CH_NUM
 * entries - 12 x 16 x 2 = 384 bytes of .bss - and the two sections are
 * kept SEPARATE (the chapter offers sharing them; separate is what lets a
 * transaction be repeated without rebuilding the first descriptor, and it
 * is what makes the write-back a second, independent copy the harvest can
 * cross-check).
 *
 * ZERO WHEN ABSENT. The tables are static members of Dmac, so they exist
 * only in a program that names Dmac - and with -ffunction-sections
 * -fdata-sections -Wl,--gc-sections (this target's flags) an image that
 * does not reach them does not carry them. That claim is not asserted
 * here, it is MEASURED: the release images of the apps that use no DMA
 * are byte-identical before and after this header existed.
 *
 * Alignment: the register descriptions of BASEADDR/WRBADDR require
 * 64-bit alignment and the device header's own descriptor type carries
 * aligned(8). The tables below ask for SIXTEEN, which is a superset: the
 * descriptor stride is 0x10, so 16-byte alignment of the array gives
 * every entry in it the same alignment as the first, and removes a class
 * of doubt for nothing.
 *
 * =========================================================================
 * THE END-ADDRESS QUIRK. For an INCREMENTING side, the descriptor's
 * SRCADDR/DSTADDR field does not hold the address the transfer starts at
 * - it holds the address one beat PAST the last one, i.e. start plus the
 * whole length in bytes (25.6.2.7). Get it wrong and the transfer runs
 * over the wrong memory, silently and at full speed. For a STATIC side
 * (a peripheral's DATA register) the field is the plain address.
 *
 * This driver never lets a caller near that arithmetic: DmaTransfer takes
 * a START pointer and a BEAT COUNT, and dma_descriptor() computes the
 * register words - constexpr, so the family fixture pins the computation
 * with static_asserts and the bench proves it again with data.
 *
 * DATASHEET DISAGREEMENT, recorded because it is real: 25.6.2.7 prints
 * the increment as `SRCADDR_START + BTCNT x BEATSIZE x 2^STEPSIZE` while
 * the SRCADDR/DSTADDR register descriptions (25.10.3, 25.10.4) print
 * `... + BTCNT x BEATSIZE + 1 x 2^STEPSIZE` - a stray "+ 1" that appears
 * in the register half and in neither functional-description formula.
 * Both sections define BEATSIZE in their own "where" list as "the
 * configured number of BYTES in a beat", under which reading the "+ 1"
 * cannot be right at all. 25.6.2.7 is the self-consistent text, this
 * driver implements it, and the bench suite decides it by moving bytes
 * and looking at where they landed.
 *
 * =========================================================================
 * ERRATA (DS80000740S, matrix re-read against this chip: E/G/J family,
 * silicon rev F, DSU DID 0x11010500).
 *
 *  - 1.10.4 "Concurrent channels triggers" (the summary table files the
 *    same item under the name "Linked Descriptors") is LIVE HERE: E/G/J
 *    at revisions E, F and H. When several channels are triggered
 *    concurrently, THE WRITE-BACK DESCRIPTORS MAY BE CORRUPTED. Microchip's
 *    workaround - "multiple transfers must only be sequenced using linked
 *    descriptors on a single channel" - amounts to not using concurrent
 *    channels, which a full-duplex serial port cannot honour: its TX and
 *    RX engines ARE two channels triggered by two independent peripheral
 *    events. So this driver takes the other road: IT NEVER TRUSTS A
 *    WRITE-BACK READ. Every field of a write-back descriptor except BTCNT
 *    and VALID is INVARIANT - the controller copies it from the fetched
 *    descriptor and never rewrites it - so harvest() compares all of them
 *    against the copy this driver loaded, bounds-checks BTCNT against the
 *    programmed length, and DISCARDS a reading that fails, counting it.
 *    The TX side never reads a write-back at all: the block length it
 *    programmed plus TCMPL is the whole truth there.
 *
 *  - 1.10.1 (CRCDATAIN written in two consecutive instructions) is rev B
 *    only. The CRC engine is not built here anyway.
 *  - 1.10.2 (fetch error with a linked descriptor on one of several
 *    channels) is E/G/J revisions B, C and D. NOT this chip.
 *  - 1.10.3 (enabling a lower-numbered channel while a linked-descriptor
 *    channel runs) is E/G/J revisions B, C and D. NOT this chip.
 *
 *    THE TRAP IN THAT TABLE, since two of the three above look alive at a
 *    glance: each item's matrix has an E/G/J row AND an N row, and for
 *    1.10.2 and 1.10.3 it is the N row - a different sub-family, no board
 *    of which exists here - that carries the marks under E and F. Read
 *    the row, not the column.
 *
 * =========================================================================
 * NOT BUILT, deliberately (docs/samc/dmac.md carries the same list):
 * the CRC engine (CRCCTRL/CRCDATAIN/CRCCHKSUM/CRCSTATUS are named nowhere
 * below - util/crc.hpp already computes the two polynomials this repo
 * needs, and a hardware CRC with a rev-B erratum on its data port earns
 * its place only when a consumer asks); linked descriptors (legal on
 * rev F, but 1.10.4's shape makes software-linked chains the honest
 * default and nothing here has needed a chain that the TCMPL interrupt
 * could not build); the event system inputs and outputs (EVACT/EVOSEL/
 * EVIE/EVOE are exposed as descriptor and channel fields because they
 * are part of the words this driver writes, but no EVSYS driver exists on
 * this target to route them, so none of them is exercised); QOSCTRL
 * (left at its reset value); RUNSTDBY and the standby sequence of
 * 25.6.7 (the power pass owns sleep on this target).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <optional>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/nvic.hpp"
#include "samc/platform_sam.hpp"

namespace brio {

// =============================================================================
// The vocabulary (pure: no register is touched above the resource)
// =============================================================================

/// BTCTRL.BEATSIZE (25.10.1): the width of one data-transfer bus access.
/// The code is not the width - `dma_beat_bytes()` is.
enum class DmaBeat : uint8_t {
    byte = DMAC_BTCTRL_BEATSIZE_BYTE_Val,     ///< 8-bit bus transfer
    hword = DMAC_BTCTRL_BEATSIZE_HWORD_Val,   ///< 16-bit bus transfer
    word = DMAC_BTCTRL_BEATSIZE_WORD_Val,     ///< 32-bit bus transfer
};

/// Bytes moved per beat: 1, 2 or 4. The end-address arithmetic needs the
/// WIDTH, the register needs the CODE, and confusing the two is exactly
/// how the "+ 1" in 25.10.3 reads as if it made sense.
constexpr uint32_t dma_beat_bytes(DmaBeat b) {
    return 1UL << static_cast<uint32_t>(b);
}

/// BTCTRL.STEPSIZE (25.10.1): the increment, in beats, of whichever side
/// STEPSEL points at. The other side always advances by exactly one beat.
enum class DmaStep : uint8_t {
    x1 = DMAC_BTCTRL_STEPSIZE_X1_Val,
    x2 = DMAC_BTCTRL_STEPSIZE_X2_Val,
    x4 = DMAC_BTCTRL_STEPSIZE_X4_Val,
    x8 = DMAC_BTCTRL_STEPSIZE_X8_Val,
    x16 = DMAC_BTCTRL_STEPSIZE_X16_Val,
    x32 = DMAC_BTCTRL_STEPSIZE_X32_Val,
    x64 = DMAC_BTCTRL_STEPSIZE_X64_Val,
    x128 = DMAC_BTCTRL_STEPSIZE_X128_Val,
};

constexpr uint32_t dma_step_factor(DmaStep s) {
    return 1UL << static_cast<uint32_t>(s);
}

/// BTCTRL.STEPSEL: which side STEPSIZE applies to. There is exactly one
/// step size per descriptor and it belongs to one side; the naming here
/// says which, rather than leaving a bare bool to be misread.
enum class DmaStepSide : uint8_t {
    destination = DMAC_BTCTRL_STEPSEL_DST_Val,
    source = DMAC_BTCTRL_STEPSEL_SRC_Val,
};

/// BTCTRL.BLOCKACT (25.10.1): what the channel does when a block ends.
/// Note the register description's own clause behind `none`: with the
/// block action set to none, TCMPL is NOT RAISED at all - so a channel
/// whose completion must be noticed needs `interrupt` (or `both`), not
/// merely an armed TCMPL.
enum class DmaBlockAction : uint8_t {
    none = DMAC_BTCTRL_BLOCKACT_NOACT_Val,
    interrupt = DMAC_BTCTRL_BLOCKACT_INT_Val,
    suspend = DMAC_BTCTRL_BLOCKACT_SUSPEND_Val,
    both = DMAC_BTCTRL_BLOCKACT_BOTH_Val,
};

/// BTCTRL.EVOSEL: what an enabled channel event output strobes on. The
/// reserved code 0x2 has no enumerator here because it has no meaning.
enum class DmaEventOut : uint8_t {
    none = DMAC_BTCTRL_EVOSEL_DISABLE_Val,
    block = DMAC_BTCTRL_EVOSEL_BLOCK_Val,
    beat = DMAC_BTCTRL_EVOSEL_BEAT_Val,
};

/// CHCTRLB.TRIGACT (25.8.19): how much one trigger buys. `beat` is what
/// a serial port needs - one trigger per RXC or DRE, one beat moved -
/// while `block` is the memory-to-memory shape: one trigger runs the
/// whole block. The reserved code 0x1 has no enumerator.
enum class DmaTriggerAction : uint8_t {
    block = DMAC_CHCTRLB_TRIGACT_BLOCK_Val,
    beat = DMAC_CHCTRLB_TRIGACT_BEAT_Val,
    transaction = DMAC_CHCTRLB_TRIGACT_TRANSACTION_Val,
};

/// CHCTRLB.LVL: the arbiter's four priority levels. A level only takes
/// part at all if CTRL.LVLENx enables it (Dmac::init enables all four by
/// default, so a level is never silently dead).
enum class DmaPriority : uint8_t {
    level0 = 0,
    level1 = 1,
    level2 = 2,
    level3 = 3,
};

/// CHCTRLB.EVACT (25.8.19): what an incoming event does to the channel.
/// Exposed because it is part of the word this driver writes; nothing
/// routes events to the DMAC on this target yet (no EVSYS driver), so
/// every value but `none` is untested silicon from here.
enum class DmaEventAction : uint8_t {
    none = DMAC_CHCTRLB_EVACT_NOACT_Val,
    trigger = DMAC_CHCTRLB_EVACT_TRIG_Val,
    conditional_trigger = DMAC_CHCTRLB_EVACT_CTRIG_Val,
    conditional_block = DMAC_CHCTRLB_EVACT_CBLOCK_Val,
    suspend = DMAC_CHCTRLB_EVACT_SUSPEND_Val,
    resume = DMAC_CHCTRLB_EVACT_RESUME_Val,
    skip_next_suspend = DMAC_CHCTRLB_EVACT_SSKIP_Val,
};

/// CHINTFLAG / CHINTENSET / CHINTENCLR share one three-bit layout
/// (25.8.20 - 25.8.22), and so do INTPEND's upper bits.
struct DmaFlag {
    DmaFlag() = delete;

    static constexpr uint8_t transfer_error = DMAC_CHINTFLAG_TERR_Msk;
    static constexpr uint8_t complete = DMAC_CHINTFLAG_TCMPL_Msk;
    static constexpr uint8_t suspended = DMAC_CHINTFLAG_SUSP_Msk;
    static constexpr uint8_t all = transfer_error | complete | suspended;
};

/// CHSTATUS (25.8.23): the three live status bits of the selected channel.
struct DmaStatus {
    DmaStatus() = delete;

    static constexpr uint8_t pending = DMAC_CHSTATUS_PEND_Msk;
    static constexpr uint8_t busy = DMAC_CHSTATUS_BUSY_Msk;
    /// Set when an invalid descriptor was fetched; cleared by a software
    /// RESUME command and by nothing else.
    static constexpr uint8_t fetch_error = DMAC_CHSTATUS_FERR_Msk;
};

// =============================================================================
// Trigger sources
// =============================================================================

/**
 * CHCTRLB.TRIGSRC. Table 25-2 is fifty-odd codes long and every one of
 * them belongs to a peripheral instance, so - exactly as with the GCLK
 * channel ids in samc/clock.hpp - the codes are READ OFF THE DEVICE
 * HEADER per instance and never computed from an index. The two the
 * serial engines need are SERCOMn_DMAC_ID_RX / _TX; the rest arrive with
 * the driver that owns them (a TC's OVF, the ADC's RESRDY, ...) rather
 * than as a fifty-line enum nothing here can verify.
 *
 * `dma_trigger_none` is the code that leaves a channel on software and
 * event triggers alone (0x00 DISABLE), which is what a memory-to-memory
 * channel wants.
 */
inline constexpr uint8_t dma_trigger_none = DMAC_CHCTRLB_TRIGSRC_DISABLE_Val;

/// SERCOM instances on THIS device, from the device header's own
/// <INSTANCE>_REGS symbols - four on the E package, six on the G and J.
///
/// This DUPLICATES samc/sercom.hpp's `sercom_count`, and the duplication
/// is deliberate: including sercom.hpp here would make every program that
/// has a serial port include the DMAC, which is exactly the coupling the
/// optional engines exist to avoid (see "ZERO WHEN ABSENT"). The two
/// ladders are held in step by a static_assert in the family fixture,
/// where both headers are legitimately in scope.
#if defined(SERCOM5_REGS)
inline constexpr uint8_t dma_sercom_count = 6;
#elif defined(SERCOM4_REGS)
inline constexpr uint8_t dma_sercom_count = 5;
#elif defined(SERCOM3_REGS)
inline constexpr uint8_t dma_sercom_count = 4;
#elif defined(SERCOM2_REGS)
inline constexpr uint8_t dma_sercom_count = 3;
#else
inline constexpr uint8_t dma_sercom_count = 2;
#endif

/// The RX trigger of one SERCOM instance ("a character has arrived").
template <uint8_t n>
constexpr uint8_t dma_trigger_sercom_rx() {
    static_assert(n < dma_sercom_count,
                  "no such SERCOM on this device, so it has no DMA trigger: the E "
                  "package bonds four (SERCOM0..3), the G and J six");
    if constexpr (n == 0) return SERCOM0_DMAC_ID_RX;
#if defined(SERCOM1_REGS)
    else if constexpr (n == 1) return SERCOM1_DMAC_ID_RX;
#endif
#if defined(SERCOM2_REGS)
    else if constexpr (n == 2) return SERCOM2_DMAC_ID_RX;
#endif
#if defined(SERCOM3_REGS)
    else if constexpr (n == 3) return SERCOM3_DMAC_ID_RX;
#endif
#if defined(SERCOM4_REGS)
    else if constexpr (n == 4) return SERCOM4_DMAC_ID_RX;
#endif
#if defined(SERCOM5_REGS)
    else if constexpr (n == 5) return SERCOM5_DMAC_ID_RX;
#endif
    else return dma_trigger_none;
}

/// The TX trigger of one SERCOM instance ("the transmit buffer is free").
template <uint8_t n>
constexpr uint8_t dma_trigger_sercom_tx() {
    static_assert(n < dma_sercom_count,
                  "no such SERCOM on this device, so it has no DMA trigger: the E "
                  "package bonds four (SERCOM0..3), the G and J six");
    if constexpr (n == 0) return SERCOM0_DMAC_ID_TX;
#if defined(SERCOM1_REGS)
    else if constexpr (n == 1) return SERCOM1_DMAC_ID_TX;
#endif
#if defined(SERCOM2_REGS)
    else if constexpr (n == 2) return SERCOM2_DMAC_ID_TX;
#endif
#if defined(SERCOM3_REGS)
    else if constexpr (n == 3) return SERCOM3_DMAC_ID_TX;
#endif
#if defined(SERCOM4_REGS)
    else if constexpr (n == 4) return SERCOM4_DMAC_ID_TX;
#endif
#if defined(SERCOM5_REGS)
    else if constexpr (n == 5) return SERCOM5_DMAC_ID_TX;
#endif
    else return dma_trigger_none;
}

// =============================================================================
// The transfer descriptor
// =============================================================================

/**
 * What a caller states about a block transfer. START pointers and a BEAT
 * COUNT - never an end address: turning those into the register words is
 * dma_descriptor()'s job, and the whole point of this type existing.
 *
 * `beats` is the number of BEATS, not bytes: at DmaBeat::word a count of
 * 4 moves sixteen bytes. Zero is refused (see dma_transfer_valid): BTCNT
 * is a down-counter and a zero start would either move nothing or wrap
 * to 65536 beats, and the chapter promises neither - 25.6.1.1 states the
 * range as 1 to 64k beats and stops there.
 */
struct DmaTransfer {
    const volatile void* source = nullptr;
    volatile void* destination = nullptr;
    uint16_t beats = 0;

    DmaBeat beat = DmaBeat::byte;
    /// Whether each side walks. A peripheral's DATA register is the
    /// static side; a buffer in RAM is the walking one.
    bool source_increment = true;
    bool destination_increment = true;

    /// The step size, and which side owns it. Left at x1/destination the
    /// two sides both advance one beat at a time, which is every ordinary
    /// transfer.
    DmaStep step = DmaStep::x1;
    DmaStepSide step_side = DmaStepSide::destination;

    /// What happens at the end of the block. `interrupt` is the default
    /// because a completion nobody can observe is rarely what was meant -
    /// and because BLOCKACT::none suppresses TCMPL outright (25.8.20).
    DmaBlockAction block_action = DmaBlockAction::interrupt;
    DmaEventOut event_output = DmaEventOut::none;

    /// The next descriptor of a linked list, or null for the last one.
    /// Linked descriptors are not used by anything in this repo (see the
    /// file header on erratum 1.10.4); the field exists because it is a
    /// word of the descriptor and lying about it by omission would be
    /// worse than leaving it available.
    const void* next = nullptr;

    bool valid = true;   ///< BTCTRL.VALID
};

/// Everything a transfer needs before the arithmetic can mean anything:
/// both ends named, at least one beat, and a length that the addressing
/// can express. Checked by the builder and static_assertable.
constexpr bool dma_transfer_valid(const DmaTransfer& t) {
    return t.source != nullptr && t.destination != nullptr && t.beats != 0;
}

/**
 * The 16-byte SRAM descriptor as a plain VALUE: constexpr-constructible,
 * comparable, copyable - which is what lets the family fixture prove the
 * end-address arithmetic without a chip, and what lets DmaChannel keep a
 * copy of what it loaded to judge the write-back against (see harvest()).
 *
 * The layout is the device header's `dmac_descriptor_registers_t` bit for
 * bit; the static_asserts under this class hold the two in step, so a
 * device-pack update that moved a field would fail the build rather than
 * the transfer.
 */
struct DmaDescriptor {
    uint16_t btctrl = 0;
    uint16_t btcnt = 0;
    uint32_t srcaddr = 0;
    uint32_t dstaddr = 0;
    uint32_t descaddr = 0;

    constexpr bool operator==(const DmaDescriptor&) const = default;

    /// True once a descriptor has been built for a real transfer.
    constexpr bool valid_bit() const { return (btctrl & DMAC_BTCTRL_VALID_Msk) != 0u; }

    /// BTCTRL with VALID masked out: the part the controller never
    /// rewrites, and therefore the part a write-back must still match.
    constexpr uint16_t invariant_control() const {
        return static_cast<uint16_t>(btctrl & ~DMAC_BTCTRL_VALID_Msk);
    }
};

/**
 * The end address of one side: `start` when it does not increment, and
 * start + beats x beat_bytes x step otherwise (25.6.2.7 - see the file
 * header for the "+ 1" the register descriptions print and this does
 * not).
 *
 * The product is named in 32 bits explicitly: 65535 beats of 4 bytes at
 * a x128 step is 2^25, which fits, but only because every operand says
 * so - `beats` alone is 16-bit and the house rule is that arithmetic
 * past 16 bits names its own width.
 */
constexpr uint32_t dma_end_address(uint32_t start, uint16_t beats, DmaBeat beat,
                                   bool increments, uint32_t step_factor) {
    if (!increments) {
        return start;
    }
    return start + static_cast<uint32_t>(beats) * dma_beat_bytes(beat) * step_factor;
}

/// BTCTRL for a transfer: everything about it that is not an address.
/// Split out from dma_descriptor() so it can be evaluated - and
/// static_asserted - without a pointer anywhere near it.
constexpr uint16_t dma_btctrl(const DmaTransfer& t) {
    return static_cast<uint16_t>(
        DMAC_BTCTRL_STEPSIZE(static_cast<uint32_t>(t.step)) |
        DMAC_BTCTRL_STEPSEL(static_cast<uint32_t>(t.step_side)) |
        (t.destination_increment ? DMAC_BTCTRL_DSTINC_Msk : 0u) |
        (t.source_increment ? DMAC_BTCTRL_SRCINC_Msk : 0u) |
        DMAC_BTCTRL_BEATSIZE(static_cast<uint32_t>(t.beat)) |
        DMAC_BTCTRL_BLOCKACT(static_cast<uint32_t>(t.block_action)) |
        DMAC_BTCTRL_EVOSEL(static_cast<uint32_t>(t.event_output)) |
        (t.valid ? DMAC_BTCTRL_VALID_Msk : 0u));
}

/**
 * Build the descriptor from two ADDRESSES and a shape - the whole
 * computation, and the one a constant expression can perform.
 *
 * This overload exists because a `reinterpret_cast` from a pointer to an
 * integer may not appear in a constant expression AT ALL, which would
 * make the pointer-taking builder below impossible to static_assert. The
 * arithmetic that actually bites - the end address - therefore lives
 * here, where the family fixture proves it with no chip and no bench.
 *
 * NOTE the asymmetry the silicon defines and this simply passes on: the
 * step size applies to the side STEPSEL names, and to the OTHER side one
 * beat always. So a descriptor with step_side = destination and step x4
 * walks the source by one beat per beat and the destination by four.
 *
 * A shape that describes nothing (no beats) yields an all-zero, invalid
 * descriptor rather than a word that would send the controller somewhere.
 */
constexpr DmaDescriptor dma_descriptor_at(uint32_t source, uint32_t destination,
                                          const DmaTransfer& shape,
                                          uint32_t next = 0) {
    if (shape.beats == 0u || source == 0u || destination == 0u) {
        return DmaDescriptor{};
    }

    const uint32_t step = dma_step_factor(shape.step);
    const uint32_t src_step = (shape.step_side == DmaStepSide::source) ? step : 1UL;
    const uint32_t dst_step = (shape.step_side == DmaStepSide::destination) ? step : 1UL;

    return DmaDescriptor{
        .btctrl = dma_btctrl(shape),
        .btcnt = shape.beats,
        .srcaddr = dma_end_address(source, shape.beats, shape.beat,
                                   shape.source_increment, src_step),
        .dstaddr = dma_end_address(destination, shape.beats, shape.beat,
                                   shape.destination_increment, dst_step),
        .descaddr = next,
    };
}

/// The same, from the pointers a caller actually holds. Two casts and
/// dma_descriptor_at(); it is `constexpr` only so that the zero-beat
/// early return can be folded, never because a real transfer could be.
constexpr DmaDescriptor dma_descriptor(const DmaTransfer& t) {
    if (!dma_transfer_valid(t)) {
        return DmaDescriptor{};
    }
    return dma_descriptor_at(reinterpret_cast<uint32_t>(t.source),
                             reinterpret_cast<uint32_t>(t.destination), t,
                             reinterpret_cast<uint32_t>(t.next));
}

static_assert(sizeof(DmaDescriptor) == 16,
              "the SRAM descriptor is exactly 128 bits (25.9)");
static_assert(sizeof(DmaDescriptor) == sizeof(dmac_descriptor_registers_t),
              "DmaDescriptor must mirror the device header's descriptor type");
static_assert(offsetof(dmac_descriptor_registers_t, DMAC_BTCNT) == 2);
static_assert(offsetof(dmac_descriptor_registers_t, DMAC_SRCADDR) == 4);
static_assert(offsetof(dmac_descriptor_registers_t, DMAC_DSTADDR) == 8);
static_assert(offsetof(dmac_descriptor_registers_t, DMAC_DESCADDR) == 12);

// =============================================================================
// Results
// =============================================================================

/// One channel's mid-transfer progress, as harvest() reports it.
struct DmaProgress {
    uint16_t remaining = 0;  ///< beats still to move (the write-back's BTCNT)
    uint16_t done = 0;       ///< beats already moved: length - remaining
    bool complete = false;   ///< the block ended (remaining == 0)
};

/// What the interrupt dispatch found: which channel, and why.
struct DmaInterrupt {
    uint8_t channel = 0;
    uint8_t flags = 0;   ///< DmaFlag bits, already cleared in the controller

    constexpr bool complete() const { return (flags & DmaFlag::complete) != 0u; }
    constexpr bool error() const { return (flags & DmaFlag::transfer_error) != 0u; }
    constexpr bool suspended() const { return (flags & DmaFlag::suspended) != 0u; }
};

// =============================================================================
// The block resource
// =============================================================================

/// What the block itself is configured with. The four arbitration levels
/// are all enabled by default: a channel whose priority level is disabled
/// is not slow, it is INVISIBLE to the arbiter (25.8.1), and a driver
/// that let that happen by omission would be lying by default.
struct DmacConfig {
    /// CTRL.LVLENx, one bit per level, bit x = level x.
    uint8_t levels = 0x0F;
    /// PRICTRL0.RRLVLENx: round-robin instead of lowest-channel-first
    /// WITHIN a level. Zero = the reset scheme, static priority.
    uint8_t round_robin = 0x00;
    /// DBGCTRL.DBGRUN: keep transferring while a debugger holds the CPU.
    /// Not reset by a software reset (25.8.6), so it is written last and
    /// unconditionally.
    bool debug_run = false;
};

template <uint8_t n>
class DmaChannel;

/**
 * The DMAC block.
 *
 * CLOCKS ARE ALMOST NOTHING HERE, which is worth saying out loud because
 * every other peripheral of this target needs two: the DMAC has NO GCLK
 * peripheral channel at all (there is no DMAC_GCLK_ID in the device
 * header and 25.5.3 names only CLK_DMAC_AHB), so the single AHB mask bit
 * is the whole clock story. That bit is already set out of reset
 * (AHBMASK reset value 0x1CFF), and init() sets it anyway: `hz` is a
 * promise on this target and so is "the block is clocked" - neither may
 * rest on what a debugger or a bootloader left behind.
 */
class Dmac {
public:
    Dmac() = delete;

    /// Channels this device has (the device header is the authority; 12
    /// on every C21 variant) and arbitration levels (4).
    static constexpr uint8_t channel_count = DMAC_CH_NUM;
    static constexpr uint8_t level_count = DMAC_LVL_NUM;

    static constexpr IRQn_Type irq() { return DMAC_IRQn; }

    static dmac_registers_t& regs() { return *DMAC_REGS; }

    // ---- clock, enable, reset ---------------------------------------------

    /// The one clock: MCLK's AHB mask bit (25.5.3).
    static void bus_clock(bool on) { Mclk::ahb(MCLK_AHBMASK_DMAC_Msk, on); }

    static bool enabled() {
        return (regs().DMAC_CTRL & DMAC_CTRL_DMAENABLE_Msk) != 0u;
    }

    /**
     * CTRL.DMAENABLE. Disabling is not instantaneous: 25.8.1 says the bit
     * stays set until the internal data buffer has drained and the
     * ongoing burst has finished, so a disable that must be COMPLETE
     * before the next step is waited out - bounded, like every wait in
     * this stratum, and reported rather than hung on.
     */
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint16_t v = regs().DMAC_CTRL;
        regs().DMAC_CTRL = static_cast<uint16_t>(
            on ? (v | DMAC_CTRL_DMAENABLE_Msk)
               : (v & static_cast<uint16_t>(~DMAC_CTRL_DMAENABLE_Msk)));
        if (on) {
            return true;
        }
        while (spins-- != 0u) {
            if (!enabled()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Everything back to reset (DBGCTRL excepted), which per 25.8.1
     * REQUIRES both the DMA and the CRC engine to be disabled first - a
     * SWRST written with either enabled is ignored and answered with an
     * access error, so this disables and waits before it asks.
     */
    static bool reset(uint32_t spins = 0xFFFFu) {
        regs().DMAC_CTRL = static_cast<uint16_t>(regs().DMAC_CTRL &
                                                 ~DMAC_CTRL_CRCENABLE_Msk);
        if (!enable(false, spins)) {
            return false;
        }
        regs().DMAC_CTRL = DMAC_CTRL_SWRST_Msk;
        // Bounded, like every wait in this stratum. CTRL is 16 bits, so
        // this cannot borrow clock.hpp's clock_wait (which reads a
        // 32-bit synchronization register) and spells the same loop out.
        while (spins-- != 0u) {
            if ((regs().DMAC_CTRL & DMAC_CTRL_SWRST_Msk) == 0u) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Clock, reset, register the two descriptor tables, set the
     * arbitration up and enable the block.
     *
     * The two BASEADDR/WRBADDR writes are why the reset comes first: both
     * registers are ENABLE-PROTECTED (25.6.2.1), so a write while the
     * block runs is discarded rather than refused. Everything below the
     * enable is therefore written into a stopped controller on purpose.
     *
     * False when the reset did not complete. A caller that gets false has
     * a block that is NOT configured - not one that is half configured:
     * nothing after the failed step is written.
     */
    static bool init(const DmacConfig& cfg = {}, uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        bus_clock(true);
        if (!reset(spins)) {
            return false;
        }

        for (uint8_t i = 0; i < channel_count; ++i) {
            descriptors_[i] = dmac_descriptor_registers_t{};
            writeback_[i] = dmac_descriptor_registers_t{};
        }
        regs().DMAC_BASEADDR = reinterpret_cast<uint32_t>(&descriptors_[0]);
        regs().DMAC_WRBADDR = reinterpret_cast<uint32_t>(&writeback_[0]);

        uint32_t prictrl = 0;
        for (uint8_t lvl = 0; lvl < level_count; ++lvl) {
            if ((cfg.round_robin & static_cast<uint8_t>(1u << lvl)) != 0u) {
                prictrl |= DMAC_PRICTRL0_RRLVLEN0_Msk << (8u * lvl);
            }
        }
        regs().DMAC_PRICTRL0 = prictrl;
        regs().DMAC_DBGCTRL = cfg.debug_run ? DMAC_DBGCTRL_DBGRUN_Msk : 0u;

        // DMAC_CTRL_LVLEN() is the GROUP macro (all four bits); the
        // per-level DMAC_CTRL_LVLEN0() masks to bit 8 alone, so feeding
        // it a four-bit mask silently enables level 0 and nothing else -
        // and a channel whose level is not enabled is INVISIBLE to the
        // arbiter, never slow. Bench-caught exactly that way: two
        // channels at level 1 sat still for four seconds while the
        // level-0 ones ran nine thousand blocks.
        regs().DMAC_CTRL = static_cast<uint16_t>(
            DMAC_CTRL_LVLEN(cfg.levels & 0x0Fu) | DMAC_CTRL_DMAENABLE_Msk);
        return true;
    }

    /// Which arbitration levels the block is currently taking requests
    /// from (CTRL.LVLENx), as a four-bit mask.
    static uint8_t levels() {
        return static_cast<uint8_t>((regs().DMAC_CTRL & DMAC_CTRL_LVLEN_Msk) >>
                                    DMAC_CTRL_LVLEN_Pos);
    }

    /// Stop everything and hand the block back: no channel runs, no line
    /// is armed, no clock is burnt. The descriptor tables stay where they
    /// are - they are this driver's storage, not the controller's.
    static void release(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        (void)enable(false, spins);
        bus_clock(false);
    }

    // ---- block-level readbacks --------------------------------------------

    /// One bit per channel: an interrupt is pending on it (25.8.11).
    static uint32_t interrupt_status() { return regs().DMAC_INTSTATUS; }
    /// One bit per channel: it has started a transfer and not finished.
    static uint32_t busy_channels() { return regs().DMAC_BUSYCH; }
    /// One bit per channel: a trigger is queued for it.
    static uint32_t pending_channels() { return regs().DMAC_PENDCH; }
    /// ACTIVE (25.8.14): the arbiter's live view - which channel is in
    /// the active registers, its remaining BTCNT (valid only while
    /// ABUSY), and which levels have work.
    static uint32_t active() { return regs().DMAC_ACTIVE; }

    /// The raw INTPEND word. Prefer take_pending() unless the flags must
    /// be inspected WITHOUT being cleared.
    static uint16_t interrupt_pending() { return regs().DMAC_INTPEND; }

    /**
     * @brief The interrupt dispatch: which channel needs service, and
     * why - flags CLEARED in the same breath.
     *
     * This is the whole ISR body of the block. INTPEND holds the LOWEST
     * channel number with a pending interrupt together with that
     * channel's three flags, and a write of {flags, id} clears those
     * flags for that id (25.8.10) - so one read and one store serve one
     * channel, and neither of them touches CHID. That is not a small
     * detail: it is what lets the handler run without contending for the
     * selector main context is using (see the file header).
     *
     * Nullopt when nothing is pending. A handler loops until it gets
     * nullopt, because INTPEND reports one channel at a time and several
     * may be pending at once:
     *
     *     extern "C" void DMAC_Handler() {
     *         while (const auto irq = brio::Dmac::take_pending()) {
     *             ...
     *         }
     *     }
     */
    [[gnu::always_inline]] static std::optional<DmaInterrupt> take_pending() {
        const uint16_t word = regs().DMAC_INTPEND;
        const uint8_t flags =
            static_cast<uint8_t>((word >> DMAC_INTPEND_TERR_Pos) & DmaFlag::all);
        if (flags == 0u) {
            return std::nullopt;
        }
        const uint8_t id = static_cast<uint8_t>(word & DMAC_INTPEND_ID_Msk);
        // One 16-bit store: the id selects, the flag bits clear. Writing
        // the flags back exactly as read acknowledges precisely what was
        // reported and nothing that arrived in between.
        regs().DMAC_INTPEND = static_cast<uint16_t>(
            DMAC_INTPEND_ID(id) | (static_cast<uint16_t>(flags) << DMAC_INTPEND_TERR_Pos));
        return DmaInterrupt{id, flags};
    }

    // ---- the descriptor tables ---------------------------------------------

    /// Where the controller looks for channel `id`'s FIRST descriptor,
    /// and where it stores that channel's ONGOING one. Public because
    /// the write-back is the harvest's evidence and a suite must be able
    /// to look at it (and, in the 1.10.4 letter, to scribble on it).
    static volatile dmac_descriptor_registers_t& descriptor(uint8_t id) {
        return descriptors_[id];
    }
    static volatile dmac_descriptor_registers_t& write_back(uint8_t id) {
        return writeback_[id];
    }

    /// Read one write-back entry as a value. Field-by-field through the
    /// volatile members, so the compiler cannot turn it into a block copy
    /// that reads a field twice or not at all.
    static DmaDescriptor read_write_back(uint8_t id) {
        volatile dmac_descriptor_registers_t& w = writeback_[id];
        DmaDescriptor d{};
        d.btctrl = w.DMAC_BTCTRL;
        d.btcnt = w.DMAC_BTCNT;
        d.srcaddr = w.DMAC_SRCADDR;
        d.dstaddr = w.DMAC_DSTADDR;
        d.descaddr = w.DMAC_DESCADDR;
        return d;
    }

    /// Store a descriptor into a table slot, field by field.
    static void write_descriptor(volatile dmac_descriptor_registers_t& slot,
                                 const DmaDescriptor& d) {
        // VALID last is not a ceremony: a controller that fetched this
        // slot between the stores must not find a valid descriptor with
        // half of its addresses still belonging to the previous transfer.
        slot.DMAC_BTCTRL = static_cast<uint16_t>(d.btctrl & ~DMAC_BTCTRL_VALID_Msk);
        slot.DMAC_BTCNT = d.btcnt;
        slot.DMAC_SRCADDR = d.srcaddr;
        slot.DMAC_DSTADDR = d.dstaddr;
        slot.DMAC_DESCADDR = d.descaddr;
        slot.DMAC_BTCTRL = d.btctrl;
    }

private:
    template <uint8_t>
    friend class DmaChannel;

    /**
     * THE ONE DOOR TO THE CHANNEL REGISTERS. Select under a critical
     * section, use under the same one, release. Private, with DmaChannel
     * the only friend, so the discipline of the file header is structural
     * rather than a request: nothing else in the program can name CHID.
     *
     * `op` is handed the register block and must do its business with no
     * further ceremony - it is already inside the guard.
     */
    template <typename Op>
    [[gnu::always_inline]] static decltype(auto) with_channel(uint8_t id, Op&& op) {
        typename SamPlatform::CriticalSection cs;
        regs().DMAC_CHID = static_cast<uint8_t>(DMAC_CHID_ID(id));
        return op(regs());
    }

    /// 12 x 16 bytes each, 384 bytes together, in .bss. Present only in a
    /// program that names Dmac at all - and, with this target's
    /// -fdata-sections/--gc-sections, only in an image that reaches them.
    alignas(16) static inline dmac_descriptor_registers_t descriptors_[channel_count];
    alignas(16) static inline dmac_descriptor_registers_t writeback_[channel_count];
};

// =============================================================================
// One channel
// =============================================================================

/// What a channel is configured with: where its triggers come from, how
/// much one trigger buys, and where it sits in the arbitration.
struct DmaChannelConfig {
    /// CHCTRLB.TRIGSRC - `dma_trigger_none` leaves the channel on
    /// software and event triggers (the memory-to-memory shape).
    uint8_t trigger = dma_trigger_none;
    DmaTriggerAction action = DmaTriggerAction::block;
    DmaPriority priority = DmaPriority::level0;

    /// The event input half. Nothing routes events to the DMAC on this
    /// target yet; see the enum's own comment.
    DmaEventAction event_action = DmaEventAction::none;
    bool event_input = false;    ///< CHCTRLB.EVIE
    bool event_output = false;   ///< CHCTRLB.EVOE
    /// CHCTRLA.RUNSTDBY. Left false: sleep belongs to the power pass, and
    /// 25.6.7's suspend-before-standby sequence has no owner here yet.
    bool run_standby = false;
};

constexpr uint32_t dma_chctrlb(const DmaChannelConfig& c) {
    return DMAC_CHCTRLB_TRIGACT(static_cast<uint32_t>(c.action)) |
           DMAC_CHCTRLB_TRIGSRC(c.trigger) |
           DMAC_CHCTRLB_LVL(static_cast<uint32_t>(c.priority)) |
           (c.event_output ? DMAC_CHCTRLB_EVOE_Msk : 0u) |
           (c.event_input ? DMAC_CHCTRLB_EVIE_Msk : 0u) |
           DMAC_CHCTRLB_EVACT(static_cast<uint32_t>(c.event_action));
}

/**
 * DmaChannel<n> - one of the twelve.
 *
 * Every verb below that touches a channel register goes through
 * Dmac::with_channel(), so every one of them costs a PRIMASK save and
 * restore around two register accesses. That is the price of a selector
 * shared with an interrupt handler, and it is paid uniformly rather than
 * being reasoned about per call site.
 *
 * The channel keeps a COPY of the descriptor it loaded (16 bytes of .bss
 * per instantiated channel). It is not bookkeeping for its own sake: it
 * is the reference harvest() judges the write-back against, and without
 * it erratum 1.10.4 would be undetectable rather than merely unavoidable.
 */
template <uint8_t n>
class DmaChannel {
    static_assert(n < DMAC_CH_NUM,
                  "no such DMA channel: this device has DMAC_CH_NUM of them "
                  "(twelve on every SAM C21 variant), numbered from zero");

public:
    DmaChannel() = delete;

    static constexpr uint8_t index = n;
    /// This channel's bit in the block-level INTSTATUS/BUSYCH/PENDCH words.
    static constexpr uint32_t mask = 1UL << n;

    // ---- configuration ----------------------------------------------------

    /**
     * Write CHCTRLB and CHCTRLA's RUNSTDBY.
     *
     * CHCTRLB is ENABLE-PROTECTED except for CMD and LVL (25.6.2.1), so
     * this disables the channel first and leaves it disabled: enable() is
     * a separate, deliberate step, exactly as with the SERCOM.
     */
    static bool configure(const DmaChannelConfig& cfg, uint32_t spins = 0xFFFFu) {
        if (!enable(false, spins)) {
            return false;   // CHCTRLB is enable-protected: do not write it blind
        }
        Dmac::with_channel(n, [&](dmac_registers_t& r) {
            r.DMAC_CHCTRLB = dma_chctrlb(cfg);
            r.DMAC_CHCTRLA = static_cast<uint8_t>(
                cfg.run_standby ? DMAC_CHCTRLA_RUNSTDBY_Msk : 0u);
            r.DMAC_CHINTENCLR = DmaFlag::all;
            r.DMAC_CHINTFLAG = DmaFlag::all;
        });
        return true;
    }

    /**
     * CHCTRLA.SWRST - the channel's registers back to reset. Ignored
     * while the channel is enabled (25.8.18), so this disables first, and
     * the descriptor slot plus the loaded copy are cleared with it: a
     * reset channel must not be re-enabled onto the previous transfer.
     *
     * WHAT A RESET DOES NOT UNDO, bench-measured and documented nowhere
     * in ch. 25: after a channel has taken a BUS ERROR (CHINTFLAG.TERR
     * from an access the AHB refused), the FIRST BLOCK it runs afterwards
     * LOSES ITS FIRST BEAT - and it does so even though this reset
     * reports success, the descriptor is correct and the write-back
     * afterwards says BTCNT = 0. Measured deterministically: the block
     * right after the error fails every time, and 32 consecutive blocks
     * after that one are byte-exact. So a channel recovering from a bus
     * error must SPEND ONE BLOCK AND DISCARD IT. This driver does not do
     * that for the caller - a bus error means an address was wrong, and
     * quietly papering over the first block afterwards would hide the
     * second half of the diagnosis.
     */
    static bool reset(uint32_t spins = 0xFFFFu) {
        if (!enable(false, spins)) {
            // SWRST is IGNORED while ENABLE is still set, and it is
            // ignored SILENTLY - the bit simply never reads back set, so
            // a wait for it to clear succeeds instantly and the channel
            // is left exactly as it was. Refusing here is the only
            // honest answer. See enable() for how this was found.
            return false;
        }
        const bool ok = Dmac::with_channel(n, [&](dmac_registers_t& r) {
            r.DMAC_CHCTRLA = DMAC_CHCTRLA_SWRST_Msk;
            // Bounded, like every wait in this stratum: SWRST clears
            // itself when the reset completes (25.8.18) and a bit that
            // never clears is reported, never spun on forever.
            uint32_t left = spins;
            while (left-- != 0u) {
                if ((r.DMAC_CHCTRLA & DMAC_CHCTRLA_SWRST_Msk) == 0u) {
                    return true;
                }
            }
            return false;
        });
        // BOTH tables, not just the descriptor. CHCTRLA.SWRST resets the
        // channel's REGISTERS; the two SRAM sections are this driver's
        // own storage and the controller has no idea they exist, so a
        // reset that left the write-back alone would leave the next
        // transfer's evidence contaminated by the previous one's.
        Dmac::write_descriptor(Dmac::descriptor(n), DmaDescriptor{});
        Dmac::write_descriptor(Dmac::write_back(n), DmaDescriptor{});
        loaded_ = DmaDescriptor{};
        return ok;
    }

    // ---- the descriptor ----------------------------------------------------

    /**
     * Put a descriptor in this channel's slot of the BASEADDR table, and
     * remember it. Main context only, and only while the channel is not
     * running - a descriptor swapped under a live fetch is exactly the
     * race 25.6.3.1.2 spends a page working around.
     */
    static void load(const DmaDescriptor& d) {
        loaded_ = d;
        Dmac::write_descriptor(Dmac::descriptor(n), d);
    }

    /// Build and load in one step.
    static bool load(const DmaTransfer& t) {
        if (!dma_transfer_valid(t)) {
            return false;
        }
        load(dma_descriptor(t));
        return true;
    }

    /// The descriptor this channel was last given - the reference the
    /// write-back is judged against.
    static const DmaDescriptor& loaded() { return loaded_; }

    // ---- running -----------------------------------------------------------

    /**
     * CHCTRLA.ENABLE.
     *
     * DISABLING IS NOT INSTANTANEOUS, and this is the single sharpest
     * edge on the channel: 25.8.18 says a '0' written during an ongoing
     * transfer does not clear the bit until the internal data buffer has
     * drained and the ongoing burst has finished (25.6.3.6 spells the
     * same abort sequence out). Until it clears, the channel is STILL
     * ENABLED - which means CHCTRLB is still enable-protected and
     * CHCTRLA.SWRST is still ignored, both of them SILENTLY.
     *
     * A disable that did not wait therefore produced a channel that
     * looked reset and was not. Bench-caught precisely: a 16-beat block
     * on a channel "reset" out of a still-draining previous transfer
     * lost its FIRST BEAT - fifteen bytes correct, byte zero untouched,
     * and a write-back cheerfully reporting BTCNT = 0.
     *
     * So the off path waits, bounded, and reports. The on path has
     * nothing to wait for.
     */
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        Dmac::with_channel(n, [&](dmac_registers_t& r) {
            const uint8_t v = r.DMAC_CHCTRLA;
            r.DMAC_CHCTRLA = static_cast<uint8_t>(
                on ? (v | DMAC_CHCTRLA_ENABLE_Msk)
                   : (v & static_cast<uint8_t>(~DMAC_CHCTRLA_ENABLE_Msk)));
        });
        if (on) {
            return true;
        }
        while (spins-- != 0u) {
            if (!enabled()) {
                return true;
            }
        }
        return false;
    }

    static bool enabled() {
        return Dmac::with_channel(n, [](dmac_registers_t& r) {
            return (r.DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk) != 0u;
        });
    }

    /**
     * SWTRIGCTRL: raise a software trigger. The register's own semantics
     * are worth knowing before reading it back - a write of one triggers
     * the channel only if it was not already pending, and the bit READS
     * BACK SET exactly when the trigger was LOST because a transfer was
     * already queued (25.8.8). So `trigger_lost()` is a real question,
     * not a status bit.
     */
    static void trigger() { Dmac::regs().DMAC_SWTRIGCTRL = mask; }

    static bool trigger_lost() {
        return (Dmac::regs().DMAC_SWTRIGCTRL & mask) != 0u;
    }
    static void clear_trigger_lost() { Dmac::regs().DMAC_SWTRIGCTRL = 0; }

    /// CHCTRLB.CMD. Not enable-protected, and the register is otherwise
    /// read-modify-written under the guard so a command does not undo the
    /// configuration next to it.
    static void command(uint32_t cmd) {
        Dmac::with_channel(n, [&](dmac_registers_t& r) {
            const uint32_t v = r.DMAC_CHCTRLB & ~DMAC_CHCTRLB_CMD_Msk;
            r.DMAC_CHCTRLB = v | DMAC_CHCTRLB_CMD(cmd);
        });
    }

    /// Suspend after the ongoing burst (25.6.3.2). Ignored on a DISABLED
    /// channel - the controller drops the command silently, which is why
    /// harvest() asks `enabled()` before it waits for anything.
    static void suspend() { command(DMAC_CHCTRLB_CMD_SUSPEND_Val); }

    /// Resume. Issued on a channel that is NOT suspended, this instead
    /// skips the next suspend action (25.6.3.3) - the same register value,
    /// two meanings, decided by the state it lands in.
    static void resume() { command(DMAC_CHCTRLB_CMD_RESUME_Val); }

    // ---- flags and status ---------------------------------------------------

    /// CHINTFLAG, as it stands (no clearing).
    static uint8_t flags() {
        return Dmac::with_channel(n, [](dmac_registers_t& r) {
            return static_cast<uint8_t>(r.DMAC_CHINTFLAG);
        });
    }

    /// Read and clear in one guarded pass - the ISR-style verb, for a
    /// handler that dispatches per channel rather than through
    /// Dmac::take_pending().
    static uint8_t take_flags() {
        return Dmac::with_channel(n, [](dmac_registers_t& r) {
            const uint8_t f = r.DMAC_CHINTFLAG;
            if (f != 0u) {
                r.DMAC_CHINTFLAG = f;
            }
            return f;
        });
    }

    static void clear_flags(uint8_t mask_) {
        Dmac::with_channel(n, [&](dmac_registers_t& r) { r.DMAC_CHINTFLAG = mask_; });
    }

    /// CHINTENSET / CHINTENCLR are set-only and clear-only registers, so
    /// each of these is a plain store: no read-modify-write to race the
    /// handler with.
    static void arm(uint8_t mask_, bool on) {
        Dmac::with_channel(n, [&](dmac_registers_t& r) {
            if (on) {
                r.DMAC_CHINTENSET = mask_;
            } else {
                r.DMAC_CHINTENCLR = mask_;
            }
        });
    }
    static uint8_t armed() {
        return Dmac::with_channel(n, [](dmac_registers_t& r) {
            return static_cast<uint8_t>(r.DMAC_CHINTENSET);
        });
    }

    /// CHSTATUS (25.8.23): PEND / BUSY / FERR of this channel.
    static uint8_t status() {
        return Dmac::with_channel(n, [](dmac_registers_t& r) {
            return static_cast<uint8_t>(r.DMAC_CHSTATUS);
        });
    }
    static bool busy() { return (status() & DmaStatus::busy) != 0u; }
    static bool pending() { return (status() & DmaStatus::pending) != 0u; }
    /// An invalid descriptor was fetched. Cleared only by a software
    /// RESUME command (25.8.23) - not by writing anything.
    static bool fetch_error() { return (status() & DmaStatus::fetch_error) != 0u; }

    // ---- progress -----------------------------------------------------------

    /**
     * @brief How far the block has got - the ONE way to ask, and the one
     * place erratum 1.10.4 is answered.
     *
     * The controller keeps BTCNT in an internal register and only spills
     * it to the write-back section when the channel loses priority, is
     * SUSPENDED, or is disabled (25.10.2). So a suspend is not a
     * side-effect of asking, it IS the asking: the channel is suspended,
     * the write-back read, and the channel resumed.
     *
     * THE READING IS THEN CHECKED, NOT BELIEVED. Everything in a
     * write-back descriptor except BTCNT and BTCTRL.VALID is invariant -
     * the controller copied it from the descriptor it fetched and never
     * touches it again - so a corrupted write-back (1.10.4) shows up as
     * any of the four invariants differing from what load() put there, or
     * as a BTCNT above the programmed length. Either way the reading is
     * DISCARDED, the violation counted, the channel resumed and the
     * caller told nothing rather than something wrong.
     *
     * @return the progress, or nullopt when the reading was refused
     * (a write-back inconsistency, counted in violations(); or the
     * suspend never took hold, counted in suspend_timeouts()).
     *
     * Callable from main context or from a handler: everything it touches
     * is guarded. It is NOT free - it stops the channel for the duration
     * - so the pacing is the caller's policy, never this driver's.
     */
    static std::optional<DmaProgress> harvest(uint32_t spins = 0xFFFFu) {
        if (!loaded_.valid_bit()) {
            return std::nullopt;   // nothing was ever loaded: nothing to report
        }

        // THE WHOLE HANDSHAKE IS ONE CRITICAL SECTION, and not for the
        // usual CHID reason (with_channel already covers that).
        // CHINTFLAG.SUSP is the handshake this function waits on, and
        // Dmac::take_pending() - the block's interrupt dispatch - reads
        // INTPEND and writes the flags it saw straight back, which
        // CLEARS them. A handler that fired mid-harvest could therefore
        // acknowledge the very SUSP this loop is waiting for, and the
        // wait would then burn its entire budget on a flag that had
        // already been and gone. Measured: roughly one such loss per
        // 70000 harvests under a five-channel load, which is exactly
        // often enough to be mistaken for silicon.
        //
        // The cost is honest and bounded: a harvest measures ~500 cycles
        // (10 us at 48 MHz), so that is how long interrupts are masked.
        // It is one more reason harvest() states that its PACING is the
        // caller's policy - a tight loop over it is not free.
        typename SamPlatform::CriticalSection cs;

        // A channel that has already finished is disabled (BLOCKACT
        // none/interrupt disable it at the end of the last block), and a
        // suspend command on a disabled channel is silently ignored
        // (25.6.3.2). Its write-back is final, so read it as it stands.
        bool running = enabled();
        if (running) {
            clear_flags(DmaFlag::suspended);
            suspend();
            uint32_t left = spins;
            for (;;) {
                if ((flags() & DmaFlag::suspended) != 0u) {
                    break;
                }
                // THE RACE THIS LOOP EXISTS TO SURVIVE. The block can end
                // between the enabled() test above and the suspend
                // command landing - and a suspend on a channel that has
                // since disabled itself is dropped SILENTLY (25.6.3.2),
                // so SUSP would never arrive and the wait would burn its
                // whole budget before giving up. A channel that is no
                // longer enabled has already written back its final
                // descriptor, which is the very answer being asked for.
                //
                // Not hypothetical: without this, a tight harvest loop
                // over five concurrent channels reported one or two
                // refusals per four-second run - all of them this race,
                // none of them erratum 1.10.4, and telling the two apart
                // is the whole point of counting them separately.
                if (!enabled()) {
                    running = false;
                    break;
                }
                if (left-- == 0u) {
                    ++timeouts_;
                    return std::nullopt;
                }
            }
        }

        const DmaDescriptor w = Dmac::read_write_back(n);

        if (running) {
            clear_flags(DmaFlag::suspended);
            resume();
        }

        if (!consistent(w)) {
            ++violations_;
            return std::nullopt;
        }

        const uint16_t remaining = w.btcnt;
        return DmaProgress{
            .remaining = remaining,
            .done = static_cast<uint16_t>(loaded_.btcnt - remaining),
            .complete = remaining == 0u,
        };
    }

    /**
     * Is this write-back self-consistent with the descriptor that was
     * loaded? The whole of erratum 1.10.4's detection, in one predicate,
     * so a suite can point it at a deliberately scribbled write-back and
     * watch it say no.
     *
     * BTCTRL is compared with VALID masked out - that bit legitimately
     * clears when the block completes - and BTCNT is bounds-checked
     * rather than equality-checked, because it is the one field that is
     * SUPPOSED to move.
     */
    static bool consistent(const DmaDescriptor& w) {
        return w.invariant_control() == loaded_.invariant_control() &&
               w.srcaddr == loaded_.srcaddr && w.dstaddr == loaded_.dstaddr &&
               w.descaddr == loaded_.descaddr && w.btcnt <= loaded_.btcnt;
    }

    /// Write-back readings refused because they failed consistent().
    /// THIS IS THE ERRATUM 1.10.4 COUNTER: on silicon where the erratum
    /// never fires it stays zero for the life of the program, and that
    /// zero is the measurement.
    static uint32_t violations() { return violations_; }
    /// Harvests abandoned because the suspend did not take hold in time.
    static uint32_t suspend_timeouts() { return timeouts_; }
    static void clear_counters() { violations_ = 0; timeouts_ = 0; }

private:
    static inline DmaDescriptor loaded_{};
    static inline uint32_t violations_ = 0;
    static inline uint32_t timeouts_ = 0;
};

// =============================================================================
// The peripheral engines samc/sercom.hpp takes as options
// =============================================================================

/**
 * DmaTxEngine<ch> - "drain a buffer into a peripheral's DATA register".
 *
 * The engine owns a channel and nothing else: the peripheral's DATA
 * address and its TX trigger code are handed in at arm() time by whoever
 * owns the peripheral, so this type knows nothing about SERCOMs and would
 * serve a DAC or an SPI unchanged.
 *
 * TRIGACT is BEAT: one trigger, one beat. The peripheral raises its
 * "transmit buffer is free" trigger once per byte and the channel moves
 * one byte - which is precisely the work the DRE interrupt used to do,
 * now done without entering the CPU at all. The DRE interrupt is
 * therefore DISARMED by whoever installs this engine; the trigger has
 * replaced it, and leaving both armed would give the byte away twice.
 *
 * NO WRITE-BACK IS EVER READ HERE. The engine knows the block length it
 * programmed and TCMPL tells it the block ended; there is no third fact
 * to want, and so erratum 1.10.4 has no surface on this side at all.
 */
template <uint8_t ch>
class DmaTxEngine {
    // Stated HERE and not left to DmaChannel<ch> alone: an engine is
    // named by an application inside a Uart's template arguments, where
    // the channel is a number somebody typed, and the number must be
    // refused at that spelling. DmaChannel is instantiated lazily, so
    // without this a Uart with an impossible engine compiled happily
    // until something touched it.
    static_assert(ch < DMAC_CH_NUM,
                  "no such DMA channel for this engine: the DMAC has DMAC_CH_NUM "
                  "of them (twelve on every SAM C21 variant), numbered from zero");

    using Channel = DmaChannel<ch>;

public:
    DmaTxEngine() = delete;

    /// The tag samc/sercom.hpp's Uart tests with `if constexpr`.
    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;

    /**
     * Claim the channel for this peripheral. `data` is the address of
     * the peripheral's transmit data register, `trigger` its TX trigger
     * code (dma_trigger_sercom_tx<n>() and its kin).
     */
    static void arm(volatile void* data, uint8_t trigger,
                    DmaPriority priority = DmaPriority::level0) {
        data_ = data;
        (void)Channel::reset();
        (void)Channel::configure({
            .trigger = trigger,
            .action = DmaTriggerAction::beat,
            .priority = priority,
        });
        Channel::arm(DmaFlag::complete | DmaFlag::transfer_error, true);
        Nvic::enable(Dmac::irq());
    }

    /**
     * Send one contiguous run. False when a block is already in flight or
     * the run is empty - the caller (the Uart) then keeps the bytes in
     * its ring and offers them again when the current block completes.
     */
    static bool start(const uint8_t* buffer, uint16_t length) {
        if (busy_ || buffer == nullptr || length == 0u) {
            return false;
        }
        if (!Channel::load(DmaTransfer{
                .source = buffer,
                .destination = data_,
                .beats = length,
                .beat = DmaBeat::byte,
                .source_increment = true,
                .destination_increment = false,
                .block_action = DmaBlockAction::interrupt,
            })) {
            return false;
        }
        in_flight_ = length;
        busy_ = true;
        Channel::enable(true);
        return true;
    }

    /// Beats of the block currently in flight (0 when idle).
    static uint16_t in_flight() { return busy_ ? in_flight_ : 0u; }
    static bool busy() { return busy_; }

    /**
     * The block ended - called from the DMAC handler when take_pending()
     * names this channel. Returns how many beats the finished block
     * carried, so the owner can consume exactly that much of its ring.
     */
    static uint16_t complete() {
        const uint16_t moved = in_flight_;
        in_flight_ = 0;
        busy_ = false;
        return moved;
    }

    /// Stop mid-block and hand the channel back. What was already moved
    /// is on the wire; what was not is the caller's to re-offer.
    static void stop() {
        (void)Channel::enable(false);
        Channel::arm(DmaFlag::all, false);
        in_flight_ = 0;
        busy_ = false;
    }

private:
    static inline volatile void* data_ = nullptr;
    static inline uint16_t in_flight_ = 0;
    static inline bool busy_ = false;
};

/**
 * DmaRxEngine<ch> - "fill a buffer from a peripheral's DATA register".
 *
 * The mirror of the TX engine, with one asymmetry that is not an
 * accident: THE ARRIVAL OF DATA IS NOT AN EVENT ANYONE IS TOLD ABOUT.
 * A receive block completes only when the buffer is full, which at an
 * idle line may be never, so the only way to know how much has arrived
 * is to ASK - and asking means suspending the channel to force the
 * write-back (DmaChannel::harvest). Hence:
 *
 *   HARVEST PACING IS THE CALLER'S. This engine never schedules itself.
 *   Whoever owns it decides how often to ask - a kernel TimeEvent every
 *   few ticks is the shape brio expects - and pays the latency it chose.
 *
 * WHAT IS TRADED AWAY, stated plainly because it cannot be given back:
 * per-byte error attribution. With the RXC interrupt armed, STATUS is
 * read for EACH character before its DATA and a corrupted byte is
 * dropped precisely. With the channel consuming RXC instead, nobody
 * reads STATUS per character: the owner reads it at harvest granularity,
 * counts what it finds and cannot say WHICH byte of the harvested run
 * was bad. A protocol with its own framing and checksum does not care; a
 * console that wants exact frame-error attribution should not take this
 * engine.
 */
template <uint8_t ch>
class DmaRxEngine {
    static_assert(ch < DMAC_CH_NUM,
                  "no such DMA channel for this engine: the DMAC has DMAC_CH_NUM "
                  "of them (twelve on every SAM C21 variant), numbered from zero");

    using Channel = DmaChannel<ch>;

public:
    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;

    /// Claim the channel. `data` is the peripheral's receive data
    /// register, `trigger` its RX trigger code.
    static void arm(volatile void* data, uint8_t trigger,
                    DmaPriority priority = DmaPriority::level0) {
        data_ = data;
        (void)Channel::reset();
        (void)Channel::configure({
            .trigger = trigger,
            .action = DmaTriggerAction::beat,
            .priority = priority,
        });
        Channel::arm(DmaFlag::complete | DmaFlag::transfer_error, true);
        Nvic::enable(Dmac::irq());
    }

    /// Point the channel at a run of free buffer and start filling it.
    static bool start(uint8_t* buffer, uint16_t length) {
        if (buffer == nullptr || length == 0u) {
            return false;
        }
        (void)Channel::enable(false);
        if (!Channel::load(DmaTransfer{
                .source = data_,
                .destination = buffer,
                .beats = length,
                .beat = DmaBeat::byte,
                .source_increment = false,
                .destination_increment = true,
                .block_action = DmaBlockAction::interrupt,
            })) {
            return false;
        }
        capacity_ = length;
        taken_ = 0;
        Channel::enable(true);
        return true;
    }

    /**
     * How many NEW beats have landed since the last ask. Nullopt when the
     * reading was refused (see DmaChannel::harvest) - the caller then
     * publishes nothing and asks again next time, which loses no data:
     * the bytes are in the buffer either way, only the count was doubted.
     */
    static std::optional<uint16_t> take() {
        const auto p = Channel::harvest();
        if (!p) {
            return std::nullopt;
        }
        const uint16_t fresh = static_cast<uint16_t>(p->done - taken_);
        taken_ = p->done;
        return fresh;
    }

    /// True once the block filled the whole run: the owner must hand over
    /// a new one (start()) or the channel stays idle with its trigger
    /// piling up losses in the peripheral.
    static bool full() { return taken_ >= capacity_ && capacity_ != 0u; }

    static uint16_t capacity() { return capacity_; }
    static uint16_t taken() { return taken_; }

    static void stop() {
        (void)Channel::enable(false);
        Channel::arm(DmaFlag::all, false);
        capacity_ = 0;
        taken_ = 0;
    }

private:
    static inline volatile void* data_ = nullptr;
    static inline uint16_t capacity_ = 0;
    static inline uint16_t taken_ = 0;
};

} // namespace brio
