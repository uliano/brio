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
 *  DmaTxEngine<ch, Elem> / DmaRxEngine<ch, Elem>
 *              the two peripheral engines samc/sercom.hpp's Uart takes
 *              as OPTIONAL policies. They live here, not there, so that
 *              sercom.hpp never includes this header and a program that
 *              names no engine cannot pay for one (see "ZERO WHEN
 *              ABSENT" below).
 *
 *  DmaLoopEngine<ch, Elem> / DmaPingPongEngine<ch, Elem>
 *              the two STREAMING engines: one table played into a
 *              peripheral for ever, and two buffers with the engine
 *              filling one while the caller drains the other. Same
 *              monostate shape, same hardening.
 *
 * ALL FOUR ENGINES ARE PERIPHERAL-AGNOSTIC by construction - a data
 * address and a trigger code are handed in at arm() - and all four
 * inherit the same four pieces of hardening, which is the reason they
 * are one family and not four files:
 *
 *   kick()      A TRIGGER IS AN EDGE, NOT A LEVEL. A peripheral asserts
 *               its request as a level and the controller latches a
 *               pending trigger when that level RISES (25.8.8), so a
 *               block armed while the request already stands waits for
 *               an edge that has gone by: enabled channel, empty
 *               CHSTATUS, standing peripheral flag, not one beat moving.
 *               One software trigger closes the hole, and doubling is
 *               impossible by construction (one pending bit, raised only
 *               if clear), so a kick racing a real trigger is LOST.
 *               BUT NOT EVERY PERIPHERAL PRESENTS ITS REQUEST THAT WAY,
 *               and the owner is the only thing that can know: a SERCOM's
 *               DRE and an ADC's RESRDY do, a TC CAPTURE CHANNEL DOES NOT
 *               - a capture stream armed with INTFLAG.MCx already
 *               standing starts anyway, and resumes from a dead stop with
 *               the flag up and TRIGSRC untouched (test_samc_timer_dma
 *               letter b, docs/samc/dmac.md). kick() is harmless where it
 *               is unnecessary and necessary where it is not; arming with
 *               the request drained is what makes the first beat a fresh
 *               one either way.
 *   abandon()   THE CALLER DECIDES A BLOCK IS DEAD, never the engine:
 *               only the peripheral's owner can read the flags that make
 *               "dead" a fact rather than a timeout. What the
 *               abandonment loses is stated, not pretended away, and
 *               every one of them is counted in faults().
 *   harvest()   mid-block progress goes through DmaChannel::harvest()
 *               and nowhere else, so every write-back reading is
 *               VALIDATED against the descriptor this driver loaded and
 *               a corrupted one (erratum 1.10.4) is refused and counted
 *               rather than believed.
 *   the zeroed write-back is answered rather than suspended on - see
 *               harvest()'s own comment on 25.6.2.8.
 *
 * `Elem` IS THE BEAT (dma_beat_of): the element type's `sizeof` feeds
 * both BEATSIZE and the end-address arithmetic, so the two cannot
 * disagree, and a width the silicon does not implement is a compile
 * error. It defaults to `uint8_t`, which is what a SERCOM moves, so
 * every existing spelling `DmaTxEngine<3>` means exactly what it always
 * did.
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
 *    and VALID is invariant WHILE THE BLOCK RUNS - the controller copies
 *    it from the fetched descriptor and, until the block ends, writes back
 *    only the beat counter (25.10.2) - so harvest() compares all of them
 *    against the copy this driver loaded, bounds-checks BTCNT against the
 *    programmed length, and DISCARDS a reading that fails, counting it.
 *
 *    THE READING IS NOT THE DAMAGE, and that is the correction this
 *    header carries after the bench went hunting for a wedged serial port
 *    (docs/samc/dmac.md, docs/samc/sercom.md). 25.6.2.6: "For an ongoing
 *    block transfer, the descriptor will be fetched from the WRITE-BACK
 *    memory section (WRBADDR)." The write-back is therefore not a report
 *    the driver may take or leave - it is the controller's LIVE COPY of
 *    the descriptor it is running. When 1.10.4 corrupts it, the transfer
 *    itself is destroyed: the channel stops moving bytes, raises no
 *    interrupt, and sits there enabled for ever. Caught in the act with
 *    two engines on one SERCOM: the transmit channel enabled, its
 *    peripheral's DRE and TXC both set (the transmitter idle and asking),
 *    CHSTATUS all zeros, no flag anywhere - and its write-back holding
 *    the OTHER channel's descriptor (BTCTRL 0x809 with SRCADDR = the
 *    SERCOM's DATA register, where its own says 0x409 and a RAM address).
 *    On the receive side the same corruption shows as CHSTATUS.FERR,
 *    which 25.6.2.8 raises when an invalid descriptor is fetched, and
 *    which is cleared only by a software RESUME.
 *
 *    So validating the reading is NECESSARY AND NOT SUFFICIENT, and the
 *    two engines below carry the other half: abandon(), which throws away
 *    a block the silicon has stopped running and hands the channel back
 *    ready for the next one. WHAT IS LOST when it fires cannot be
 *    recovered and is not pretended otherwise - an unknown tail of one
 *    transmit block, or one receive block's worth of arrival count - so
 *    every abandonment is COUNTED and the count is public. WHO decides a
 *    block is dead is the peripheral's owner, never this file: only the
 *    owner knows what its peripheral's own flags mean (see
 *    samc/sercom.hpp's dead-block predicate, which is one line of SERCOM
 *    truth: a transmit block cannot be in flight while DRE and TXC are
 *    both set).
 *
 *    The TX side still never reads a write-back for its PROGRESS: the
 *    block length it programmed plus TCMPL is the whole truth there.
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
 * its place only when a consumer asks); LINKED DESCRIPTORS, which is
 * also why there is no hardware circular mode here - a self-linked
 * descriptor is the only way chapter 25 offers to make a channel repeat
 * without the CPU (25.6.3.1), and 1.10.4 corrupting a write-back that
 * 25.6.2.6 makes the LIVE descriptor leaves a self-linked chain with no
 * second copy to judge the first against, so DmaLoopEngine closes its
 * loop from the TCMPL interrupt instead; the event system inputs and
 * outputs beyond what a proof needs (EVACT/EVOSEL/EVIE/EVOE are exposed
 * as descriptor and channel fields because they are part of the words
 * this driver writes, and a channel driven only by an event has been
 * measured, but no engine here routes one); QOSCTRL (left at its reset
 * value); RUNSTDBY and the standby sequence of 25.6.7 (the power pass
 * owns sleep on this target).
 *
 * AND ONE LEVEL UP: the target-independent contract these engines
 * satisfy is util/block_stream.hpp (design/block-stream.md) -
 * DmaPingPongEngine is a BlockSource and DmaLoopEngine a BlockPlayer,
 * checked in the family fixture, and BlockRelay is the AO that hands a
 * source's filled blocks to subscribers as Lease::dispatch loans. The
 * contract deliberately speaks BLOCKS and not DMA, so the next
 * platform's stream machinery (or an interrupt-fed implementation on a
 * machine with none) has a fixed point to be measured against.
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

/**
 * The beat that moves one `Elem` - the bridge between a C++ element type
 * and BEATSIZE, and the ONE place the four engines below decide their
 * bus width.
 *
 * THE TYPE IS THE WIDTH: a `uint16_t` table is moved with halfword
 * beats, a `uint32_t` one with word beats, and there is no way to ask
 * for a mismatch, because the same `sizeof` feeds both the register
 * field and the end-address arithmetic. Any other size is a COMPILE
 * ERROR rather than a silent truncation - 25.10.1 implements exactly
 * three widths and the reserved fourth code is not one of them.
 *
 * WHAT THIS DOES NOT CHECK is alignment, because the descriptor has no
 * field for it: the AHB will not fetch a halfword from an odd address
 * or a word from an unaligned one, and a misaligned buffer is a BUS
 * ERROR (CHINTFLAG.TERR) rather than a bad transfer. A `uint16_t` or
 * `uint32_t` array is aligned by the language, which is why the engines
 * take a typed pointer and not a `void*`.
 */
template <typename Elem>
constexpr DmaBeat dma_beat_of() {
    static_assert(sizeof(Elem) == 1u || sizeof(Elem) == 2u || sizeof(Elem) == 4u,
                  "a DMA beat is 1, 2 or 4 bytes (25.10.1 BEATSIZE), so a DMA "
                  "engine's element type must be exactly one of those widths");
    return sizeof(Elem) == 1u   ? DmaBeat::byte
           : sizeof(Elem) == 2u ? DmaBeat::hword
                                : DmaBeat::word;
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
    ///
    /// It is not an exotic state. 25.6.2.8 raises it whenever the
    /// controller fetches a descriptor with VALID = 0, and the ONGOING
    /// descriptor is fetched from the write-back section (25.6.2.6) -
    /// which is exactly what erratum 1.10.4 corrupts. A channel that has
    /// it is suspended, keeps collecting triggers in CHSTATUS.PEND, and
    /// never transfers again until somebody notices.
    static bool fetch_error() { return (status() & DmaStatus::fetch_error) != 0u; }

    /// Clear a fetch error: 25.8.23 says the bit "is cleared when a
    /// software resume command is executed", and nothing else clears it.
    static void clear_fetch_error() { resume(); }

    /**
     * Is the ONGOING descriptor still this channel's own?
     *
     * The cheap, non-invasive half of harvest(): it reads the write-back
     * and judges it, without suspending anything. It cannot say how far
     * a block has got - only whether the controller is still running the
     * transfer that was programmed. False after erratum 1.10.4 has
     * scribbled the live copy.
     *
     * MEANINGLESS BEFORE THE CONTROLLER HAS SPILLED: a freshly started
     * block whose write-back still holds the previous one's descriptor
     * (or zeros, after reset()) answers false with nothing wrong. So
     * this is a question to ask about a block that has had TIME, and the
     * caller supplies that judgment - see the engines' abandon().
     */
    static bool write_back_ok() { return consistent(Dmac::read_write_back(n)); }

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

        // NOTHING TO SUSPEND, AND SUSPENDING IT IS NOT FREE. When the
        // controller has not written this slot since reset() zeroed it,
        // the block has not started: there is no progress to force out,
        // and the suspend/resume pair is a real risk rather than a waste.
        // 25.6.2.8: "when the channel is resumed and the DMA fetches the
        // next descriptor with null address (DESCADDR = 0x00000000) ...
        // the channel operation is suspended and CHSTATUS.FERR is set" -
        // and every descriptor this driver builds for a single block has
        // DESCADDR = 0. Measured: harvesting a just-armed receive channel
        // in a tight loop killed it with a fetch error, deterministically,
        // and the recovery re-armed it straight into the same wall.
        if (Dmac::read_write_back(n) == DmaDescriptor{}) {
            return DmaProgress{
                .remaining = loaded_.btcnt,
                .done = 0,
                .complete = false,
            };
        }

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
 * DmaTxEngine<ch, Elem> - "drain a buffer into a peripheral's DATA
 * register".
 *
 * The engine owns a channel and nothing else: the peripheral's DATA
 * address and its TX trigger code are handed in at arm() time by whoever
 * owns the peripheral, so this type knows nothing about SERCOMs and
 * serves a DAC or an SPI unchanged.
 *
 * `Elem` IS THE BEAT (dma_beat_of): `DmaTxEngine<3>` is a byte engine,
 * which is what a SERCOM wants and why the parameter defaults;
 * `DmaTxEngine<3, uint16_t>` moves halfwords, which is what a converter's
 * data register wants. Nothing else in the engine changes - the width
 * lives in one constant and the typed pointer start() takes.
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
template <uint8_t ch, typename Elem = uint8_t>
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
    /// The bus width one element costs, from the element type alone.
    static constexpr DmaBeat beat = dma_beat_of<Elem>();
    using element = Elem;

    /**
     * Claim the channel for this peripheral. `data` is the address of
     * the peripheral's transmit data register, `trigger` its TX trigger
     * code (dma_trigger_sercom_tx<n>() and its kin).
     */
    static void arm(volatile void* data, uint8_t trigger,
                    DmaPriority priority = DmaPriority::level0) {
        data_ = data;
        trigger_ = trigger;
        priority_ = priority;
        claim();
        Nvic::enable(Dmac::irq());
    }

    /**
     * @brief Throw away a block the silicon has stopped running, and hand
     * the channel back ready for the next one.
     *
     * THE CALLER DECIDES THAT THE BLOCK IS DEAD, not this engine: only
     * the peripheral's owner knows what its own flags mean (samc/
     * sercom.hpp's Uart asks whether DRE and TXC are both set, which no
     * live transmit block can allow). This verb is the consequence, and
     * it is deliberately blunt - the channel is reset, reconfigured and
     * re-armed from scratch, because after erratum 1.10.4 the controller's
     * live descriptor copy is not something to reason about.
     *
     * WHAT IS LOST: an unknown tail of the abandoned block. The engine
     * told its owner `length` beats were in flight and cannot say how
     * many of them reached the wire, so the owner's ring is left as it
     * was and those bytes are simply gone. There is no honest alternative
     * - the one field that would say is the corrupted one.
     *
     * @return true when a block was abandoned (and faults() incremented);
     * false when nothing was in flight.
     */
    static bool abandon() {
        if (!busy_) {
            return false;
        }
        ++faults_;
        claim();
        in_flight_ = 0;
        busy_ = false;
        return true;
    }

    /// How many blocks have been abandoned - the erratum's running bill,
    /// zero on silicon where 1.10.4 does not apply.
    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    /**
     * Send one contiguous run. False when a block is already in flight or
     * the run is empty - the caller (the Uart) then keeps the bytes in
     * its ring and offers them again when the current block completes.
     */
    static bool start(const Elem* buffer, uint16_t length) {
        if (busy_ || buffer == nullptr || length == 0u) {
            return false;
        }
        if (!Channel::load(DmaTransfer{
                .source = buffer,
                .destination = data_,
                .beats = length,
                .beat = beat,
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

    /**
     * Send ONE element `length` times - the source address held still.
     * The SPI host's dummy-fill for a read-only transfer, where a null
     * tx means "0xFF on every character". A SIBLING VERB rather than a
     * flag on start(): a defaulted argument re-compiles every existing
     * call site, and the pre-existing images' byte-identity is a gate
     * this stratum keeps (the descriptor differs by exactly one bit).
     */
    static bool start_fixed(const Elem* one, uint16_t length) {
        if (busy_ || one == nullptr || length == 0u) {
            return false;
        }
        if (!Channel::load(DmaTransfer{
                .source = one,
                .destination = data_,
                .beats = length,
                .beat = beat,
                .source_increment = false,
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


    /**
     * @brief Raise ONE software trigger on the channel.
     *
     * THE STANDING REQUEST. A peripheral asserts its DMA request as a
     * LEVEL - "my transmit buffer is free", "I have a character" - and the
     * DMAC turns that level into a pending trigger when it RISES. A block
     * armed while the level is ALREADY HIGH therefore waits for an edge
     * that has already happened and may never happen again: the channel
     * sits enabled, CHSTATUS empty, the peripheral's own flag standing,
     * and not one beat moves. The owner, which is the only thing that can
     * read the peripheral's flag, gives the channel the missing edge with
     * this.
     *
     * Safe against doubling by construction: SWTRIGCTRL raises the
     * pending bit only if it was not already set (25.8.8), and the
     * channel has exactly one, so a kick that races a real hardware
     * trigger is simply LOST (readable through trigger_lost()) rather
     * than moving a second beat.
     */
    static void kick() { Channel::trigger(); }

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
    /// Take the channel from whatever state it is in: reset (which also
    /// clears both descriptor tables), configure, arm. arm() and
    /// abandon() are the same act with a different reason.
    static void claim() {
        (void)Channel::reset();
        (void)Channel::configure({
            .trigger = trigger_,
            .action = DmaTriggerAction::beat,
            .priority = priority_,
        });
        Channel::arm(DmaFlag::complete | DmaFlag::transfer_error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline uint8_t trigger_ = dma_trigger_none;
    static inline DmaPriority priority_ = DmaPriority::level0;
    static inline uint16_t in_flight_ = 0;
    static inline uint32_t faults_ = 0;
    static inline bool busy_ = false;
};

/**
 * DmaRxEngine<ch, Elem> - "fill a buffer from a peripheral's DATA
 * register".
 *
 * `Elem` IS THE BEAT, exactly as in the TX engine above, and it defaults
 * to a byte so every existing spelling means what it always did.
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
template <uint8_t ch, typename Elem = uint8_t>
class DmaRxEngine {
    static_assert(ch < DMAC_CH_NUM,
                  "no such DMA channel for this engine: the DMAC has DMAC_CH_NUM "
                  "of them (twelve on every SAM C21 variant), numbered from zero");

    using Channel = DmaChannel<ch>;

public:
    DmaRxEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    /// The bus width one element costs, from the element type alone.
    static constexpr DmaBeat beat = dma_beat_of<Elem>();
    using element = Elem;

    /// Claim the channel. `data` is the peripheral's receive data
    /// register, `trigger` its RX trigger code.
    static void arm(volatile void* data, uint8_t trigger,
                    DmaPriority priority = DmaPriority::level0) {
        data_ = data;
        trigger_ = trigger;
        priority_ = priority;
        claim();
        Nvic::enable(Dmac::irq());
    }

    /// True while the channel is not running a block at all - it
    /// finished, or was never started. The owner must hand it a new run,
    /// whatever the engine's own beat arithmetic says: a reading that was
    /// refused leaves `taken_` behind, and a re-arm rule that trusted
    /// only `full()` would then never fire again.
    static bool idle() { return !Channel::enabled(); }

    /// Point the channel at a run of free buffer and start filling it.
    static bool start(Elem* buffer, uint16_t length) {
        if (buffer == nullptr || length == 0u) {
            return false;
        }
        (void)Channel::enable(false);
        if (!Channel::load(DmaTransfer{
                .source = data_,
                .destination = buffer,
                .beats = length,
                .beat = beat,
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

    /// Drain `length` elements into ONE cell - the destination held
    /// still. The SPI host's discard sink for a write-only transfer,
    /// where the completion still needs every character RECEIVED (the
    /// last one is on the wire until it is) but nobody wants the bytes.
    /// A sibling verb, not a flag, for start_fixed()'s own reason.
    static bool start_discard(Elem* sink, uint16_t length) {
        if (sink == nullptr || length == 0u) {
            return false;
        }
        (void)Channel::enable(false);
        if (!Channel::load(DmaTransfer{
                .source = data_,
                .destination = sink,
                .beats = length,
                .beat = beat,
                .source_increment = false,
                .destination_increment = false,
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



    /**
     * @brief Raise ONE software trigger on the channel.
     *
     * THE STANDING REQUEST. A peripheral asserts its DMA request as a
     * LEVEL - "my transmit buffer is free", "I have a character" - and the
     * DMAC turns that level into a pending trigger when it RISES. A block
     * armed while the level is ALREADY HIGH therefore waits for an edge
     * that has already happened and may never happen again: the channel
     * sits enabled, CHSTATUS empty, the peripheral's own flag standing,
     * and not one beat moves. The owner, which is the only thing that can
     * read the peripheral's flag, gives the channel the missing edge with
     * this.
     *
     * Safe against doubling by construction: SWTRIGCTRL raises the
     * pending bit only if it was not already set (25.8.8), and the
     * channel has exactly one, so a kick that races a real hardware
     * trigger is simply LOST (readable through trigger_lost()) rather
     * than moving a second beat.
     */
    static void kick() { Channel::trigger(); }

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
    /// See DmaTxEngine::claim() - the same act, the same two reasons.
    static void claim() {
        (void)Channel::reset();
        (void)Channel::configure({
            .trigger = trigger_,
            .action = DmaTriggerAction::beat,
            .priority = priority_,
        });
        Channel::arm(DmaFlag::complete | DmaFlag::transfer_error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline uint8_t trigger_ = dma_trigger_none;
    static inline DmaPriority priority_ = DmaPriority::level0;
    static inline uint16_t capacity_ = 0;
    static inline uint16_t taken_ = 0;
};

// =============================================================================
// The two streaming engines
// =============================================================================

/**
 * DmaLoopEngine<ch, Elem> - "play one table into a peripheral, for ever".
 *
 * The shape a waveform wants: a caller-owned table of `Elem`, a
 * peripheral data register, and a trigger. Every time the block ends,
 * the SAME block starts again - so the table plays as a loop and the CPU
 * is in the path only for the few stores that re-arm it.
 *
 * THERE IS NO HARDWARE CIRCULAR MODE ON THIS CONTROLLER, and that is why
 * the loop is closed in software. Chapter 25 offers exactly one way to
 * make a channel repeat without the CPU: a LINKED DESCRIPTOR whose
 * DESCADDR points back at itself (25.6.3.1). Linked descriptors are
 * deliberately not built in this driver - erratum 1.10.4 corrupts the
 * write-back, 25.6.2.6 makes the write-back the LIVE descriptor of an
 * ongoing block, and a self-linked chain has no second copy to judge the
 * first against. So the lap boundary is a TCMPL interrupt and complete()
 * is what the owner's handler calls there. The cost is one interrupt per
 * lap, which is one per TABLE and not one per sample; the gain is that
 * every lap is re-armed from a descriptor this driver built and can
 * still validate.
 *
 * THE RE-ARM DOES NOT KICK, AND THAT IS A CORRECTNESS RULE RATHER THAN
 * A CHOICE. kick() is for a request that IS ALREADY STANDING; issued
 * when it is not, it moves a beat the peripheral never asked for - here
 * that means writing the next table entry over a value the peripheral
 * has not consumed yet, so one sample of every lap would silently
 * vanish. At the moment TCMPL fires, the last beat has just SERVED the
 * request, so the request is down and the next edge is the peripheral's
 * to raise. The engine therefore re-arms and waits, exactly as the two
 * serial engines do, and kick() stays the OWNER's verb for the one
 * moment it is right: the FIRST arm, and the arm after an abandon(),
 * where the request may have risen before the channel existed. The
 * suites do it in one line - `if (Dac::empty()) Loop::kick();`.
 *
 * WHAT A LAP BOUNDARY COSTS THE STREAM is the interrupt's own latency,
 * and it is not hidden: the peripheral is unserved from the moment the
 * block ends until the handler re-arms, so a request rising inside that
 * window is served late and one rising twice inside it is served once.
 * At a paced rate well below the interrupt's turnaround that is nothing
 * - measured on this bench, a 5 kHz stream loses no sample at any lap
 * boundary over thousands of laps - and a stream that cannot afford it
 * wants a longer table, which is the only knob there is.
 */
template <uint8_t ch, typename Elem = uint8_t>
class DmaLoopEngine {
    static_assert(ch < DMAC_CH_NUM,
                  "no such DMA channel for this engine: the DMAC has DMAC_CH_NUM "
                  "of them (twelve on every SAM C21 variant), numbered from zero");

    using Channel = DmaChannel<ch>;

public:
    DmaLoopEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr DmaBeat beat = dma_beat_of<Elem>();
    using element = Elem;

    /// Claim the channel for this peripheral. `data` is the address of
    /// the register the table is played into, `trigger` the peripheral's
    /// trigger code (Dac::dma_trigger_empty and its kin).
    static void arm(volatile void* data, uint8_t trigger,
                    DmaPriority priority = DmaPriority::level0) {
        data_ = data;
        trigger_ = trigger;
        priority_ = priority;
        claim();
        Nvic::enable(Dmac::irq());
    }

    /**
     * Begin playing `table`, and keep playing it until stop().
     *
     * The table is the CALLER'S and must outlive the stream; nothing is
     * copied.
     *
     * THE POINTER IS `const volatile` ON PURPOSE. The controller reads
     * this memory and the compiler cannot see it happen, so a table the
     * program fills and then hands over is exactly the shape gcc has
     * already been caught optimizing on this target (a zeroing store
     * sunk past a transfer - the DMAC campaign's own lesson). Declaring
     * the parameter volatile lets a caller keep its table volatile
     * without a cast, and a plain array still converts to it for free.
     */
    static bool start(const volatile Elem* table, uint16_t length) {
        if (table == nullptr || length == 0u) {
            return false;
        }
        table_ = table;
        length_ = length;
        laps_ = 0;
        return launch();
    }

    /**
     * The block ended - called from the DMAC handler when
     * Dmac::take_pending() names this channel. Counts the lap and starts
     * the same block again.
     *
     * @return the beats the finished lap carried, so an owner that wants
     * to know the stream is alive has a number rather than a promise.
     */
    static uint16_t complete() {
        if (!running_) {
            return 0;
        }
        // Written out rather than `++`: compound operations on a
        // volatile are deprecated in C++20 and this build is -Werror.
        laps_ = laps_ + 1u;
        if (!launch()) {
            running_ = false;
            return 0;
        }
        return length_;
    }

    /// Laps finished since start(). A stream that is alive is one whose
    /// lap count moves; nothing else in this engine says so.
    static uint32_t laps() { return laps_; }
    static bool running() { return running_; }
    static uint16_t length() { return length_; }

    /// How far into the CURRENT lap the controller has got, through
    /// harvest()'s validated path - nullopt when the reading was refused
    /// (erratum 1.10.4) exactly as everywhere else in this file.
    static std::optional<DmaProgress> progress() { return Channel::harvest(); }

    /// See DmaTxEngine::kick(). Public because the OWNER is the only
    /// thing that can see its peripheral's request already standing.
    static void kick() { Channel::trigger(); }

    /**
     * Throw away a lap the silicon has stopped running and start a fresh
     * one - the same caller-decides-the-block-is-dead doctrine
     * DmaTxEngine::abandon() carries, and for the same reason: only the
     * peripheral's owner can read the flags that make "dead" a fact
     * rather than a timeout.
     *
     * WHAT IS LOST is the tail of the abandoned lap - an unknown number
     * of samples the peripheral never got - and it is counted as a
     * fault, not papered over. The next lap starts at the table's
     * beginning, so the STREAM restarts in phase and the loss shows as a
     * gap and not as a permanent offset.
     */
    static bool abandon() {
        if (!running_) {
            return false;
        }
        ++faults_;
        claim();
        return launch();
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    /// Stop at the end of nothing - immediately. What the current lap
    /// had already moved is in the peripheral; the rest is not sent.
    static void stop() {
        (void)Channel::enable(false);
        Channel::arm(DmaFlag::all, false);
        running_ = false;
    }

private:
    /// Load the block and set it going. The descriptor is rebuilt every
    /// lap rather than relied on to survive: it costs six stores, and
    /// after an abandon() there is nothing in the tables worth trusting.
    static bool launch() {
        if (table_ == nullptr || length_ == 0u) {
            return false;
        }
        if (!Channel::load(DmaTransfer{
                .source = table_,
                .destination = data_,
                .beats = length_,
                .beat = beat,
                .source_increment = true,
                .destination_increment = false,
                .block_action = DmaBlockAction::interrupt,
            })) {
            return false;
        }
        running_ = true;
        // NO KICK HERE - see the class comment. A kick is right only
        // where the peripheral's request is ALREADY STANDING, which the
        // owner is the only thing that can see; issued blind it would
        // overwrite a value the peripheral has not taken.
        Channel::enable(true);
        return true;
    }

    /// See DmaTxEngine::claim() - the same act, the same two reasons.
    static void claim() {
        (void)Channel::reset();
        (void)Channel::configure({
            .trigger = trigger_,
            .action = DmaTriggerAction::beat,
            .priority = priority_,
        });
        Channel::arm(DmaFlag::complete | DmaFlag::transfer_error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline const volatile Elem* table_ = nullptr;
    static inline uint8_t trigger_ = dma_trigger_none;
    static inline DmaPriority priority_ = DmaPriority::level0;
    static inline uint16_t length_ = 0;
    // laps_ is written in the DMAC handler and READ FROM THREAD CONTEXT,
    // typically in a polling loop - the ticker's own lesson (gcc -Os
    // deleted a bare polling loop) applies, so the getter's load must be
    // a volatile one. faults_ is thread-written (abandon() is the
    // owner's verb) and stays plain.
    static inline volatile uint32_t laps_ = 0;
    static inline uint32_t faults_ = 0;
    static inline volatile bool running_ = false;
};

/**
 * DmaPingPongEngine<ch, Elem> - "fill one buffer while the caller drains
 * the other".
 *
 * The shape a sampled stream wants, and the answer to the asymmetry
 * DmaRxEngine states: a receive block completes only when the buffer is
 * full, so with ONE buffer the peripheral is unserved for the whole time
 * the caller spends reading it. With two, the block that ends hands its
 * buffer to the caller and the next block starts on the other one inside
 * the same interrupt.
 *
 * THE ACCOUNTING IS THE API, and this is the design position. A stream
 * whose drainer falls behind cannot be made correct by cleverness - the
 * samples are gone - so the only thing worth building is a stream that
 * says so exactly:
 *
 *   laps()      buffers filled and handed over, since start()
 *   overruns()  times the engine had NO free buffer to start the next
 *               block in, i.e. the caller still held both
 *   stalled()   whether it is in that state right now
 *
 * ON AN OVERRUN THE ENGINE SKIPS THE LAP - it starts no block at all -
 * rather than re-using the buffer the caller is reading. That trades
 * SAMPLES for INTEGRITY: everything the caller is handed is a complete,
 * untorn block, and the samples that arrived while the engine was
 * stalled were never written anywhere. release() restarts it.
 *
 * WHAT IS NOT COUNTED, said plainly: HOW MANY samples were lost during a
 * stall. The controller counts what it moves, and a stalled channel
 * moves nothing; the number of arrivals that went unserved is the
 * PERIPHERAL's to report (a converter's OVERRUN flag), never this
 * engine's. An owner that wants the loss and not just the stall reads
 * its peripheral.
 *
 * BUFFER OWNERSHIP is strictly alternating and there are exactly two
 * states per buffer, so the whole model is three counters: which buffer
 * the engine fills next, which the caller drains next, and how many are
 * pending. `pending_` reaches two only in a stall, which is what makes
 * "the buffer the engine needs is the one the caller holds" a fact of
 * the arithmetic rather than a comparison of pointers.
 *
 * THE SWAP DOES NOT KICK, for the reason DmaLoopEngine's comment gives
 * from the other side: the beat that ended the block was the one that
 * SERVED the peripheral's request, so the request is down and a kick
 * would move a beat out of a data register holding nothing new - a
 * DUPLICATE SAMPLE in the middle of the stream. kick() is the owner's
 * verb for the first arm and for the arm after an abandon().
 */
template <uint8_t ch, typename Elem = uint8_t>
class DmaPingPongEngine {
    static_assert(ch < DMAC_CH_NUM,
                  "no such DMA channel for this engine: the DMAC has DMAC_CH_NUM "
                  "of them (twelve on every SAM C21 variant), numbered from zero");

    using Channel = DmaChannel<ch>;

public:
    DmaPingPongEngine() = delete;

    static constexpr bool present = true;
    static constexpr uint8_t channel = ch;
    static constexpr DmaBeat beat = dma_beat_of<Elem>();
    using element = Elem;

    /// Claim the channel. `data` is the peripheral's result register,
    /// `trigger` its trigger code (Adc<n>::dma_trigger_resrdy and kin).
    static void arm(volatile void* data, uint8_t trigger,
                    DmaPriority priority = DmaPriority::level0) {
        data_ = data;
        trigger_ = trigger;
        priority_ = priority;
        claim();
        Nvic::enable(Dmac::irq());
    }

    /**
     * Begin streaming into `first`, with `second` as the buffer the next
     * block will use. Both are the CALLER'S, both must hold `length`
     * elements, and both must outlive the stream.
     *
     * `volatile` for the reason DmaLoopEngine::start() gives, and here
     * it is the more important direction: the controller WRITES these
     * buffers and the compiler sees nothing, so a caller reading a
     * drained buffer through a non-volatile pointer is reading what gcc
     * thinks is there. `ready()` hands back a volatile pointer for the
     * same reason.
     */
    static bool start(volatile Elem* first, volatile Elem* second,
                      uint16_t length) {
        if (first == nullptr || second == nullptr || first == second ||
            length == 0u) {
            return false;
        }
        buffer_[0] = first;
        buffer_[1] = second;
        length_ = length;
        fill_ = 0;
        drain_ = 0;
        pending_ = 0;
        laps_ = 0;
        overruns_ = 0;
        stalled_ = false;
        return launch();
    }

    /**
     * The block ended - called from the DMAC handler when
     * Dmac::take_pending() names this channel. Hands the filled buffer
     * to the caller and starts the next block in the other one, or
     * counts an overrun and stalls.
     *
     * @return the beats the finished block carried, or zero when nothing
     * was running.
     */
    static uint16_t complete() {
        if (!running_) {
            return 0;
        }
        // Written out rather than `++`: compound operations on a
        // volatile are deprecated in C++20 and this build is -Werror.
        laps_ = laps_ + 1u;
        pending_ = static_cast<uint8_t>(pending_ + 1u);
        fill_ = static_cast<uint8_t>(fill_ ^ 1u);
        if (pending_ >= 2u) {
            // The buffer the engine needs next is the one the caller
            // still has not released. Skip the lap rather than write
            // into it.
            overruns_ = overruns_ + 1u;
            stalled_ = true;
            running_ = false;
            return length_;
        }
        if (!launch()) {
            running_ = false;
        }
        return length_;
    }

    /// The buffer that is full and waiting for the caller, or nullptr.
    /// Valid until release() is called for it and not one beat longer.
    static volatile Elem* ready() {
        return pending_ != 0u ? buffer_[drain_] : nullptr;
    }
    /// How many elements the ready buffer holds - always the whole
    /// block, because a buffer is handed over only when it is full.
    static uint16_t ready_length() { return pending_ != 0u ? length_ : 0u; }

    /**
     * Hand the ready buffer back to the engine. Restarts a stalled
     * stream, which is the one place this verb does more than
     * bookkeeping - and why it holds a critical section: complete()
     * runs in the DMAC handler and touches the same three counters.
     */
    static bool release() {
        typename SamPlatform::CriticalSection cs;
        if (pending_ == 0u) {
            return false;
        }
        pending_ = static_cast<uint8_t>(pending_ - 1u);
        drain_ = static_cast<uint8_t>(drain_ ^ 1u);
        if (stalled_) {
            stalled_ = false;
            if (!launch()) {
                return false;
            }
        }
        return true;
    }

    static uint32_t laps() { return laps_; }
    /// Times the engine found both buffers held by the caller. See the
    /// class comment on what this does and does NOT count.
    static uint32_t overruns() { return overruns_; }
    static bool stalled() { return stalled_; }
    static bool running() { return running_; }
    static uint16_t length() { return length_; }
    /// Buffers filled and not yet released: 0, 1, or 2 (2 = stalled).
    static uint8_t pending() { return pending_; }

    /// How far into the CURRENT block the controller has got, through
    /// harvest()'s validated path - the ONLY way to read mid-block
    /// progress, and nullopt when the reading was refused.
    static std::optional<DmaProgress> progress() { return Channel::harvest(); }

    /// See DmaTxEngine::kick().
    static void kick() { Channel::trigger(); }

    /**
     * Throw away a block the silicon has stopped running and start a
     * fresh one in the same buffer - the caller-decides doctrine again.
     * The partly filled buffer is NOT handed over: a torn block is
     * exactly what this engine exists not to produce, so the whole thing
     * is discarded and counted.
     *
     * A STALLED STREAM IS NOT A DEAD ONE and this refuses it without
     * counting anything: while the engine is waiting for the caller to
     * release a buffer there is no block in flight, so there is nothing
     * to abandon and a fault counted there would be a fault that never
     * happened. release() is the verb for that state.
     */
    static bool abandon() {
        if (stalled_ || !running_) {
            return false;
        }
        ++faults_;
        claim();
        return launch();
    }

    static uint32_t faults() { return faults_; }
    static void clear_faults() { faults_ = 0; }

    static void stop() {
        (void)Channel::enable(false);
        Channel::arm(DmaFlag::all, false);
        running_ = false;
        stalled_ = false;
        pending_ = 0;
        length_ = 0;
    }

private:
    static bool launch() {
        if (buffer_[fill_] == nullptr || length_ == 0u) {
            return false;
        }
        (void)Channel::enable(false);
        if (!Channel::load(DmaTransfer{
                .source = data_,
                .destination = buffer_[fill_],
                .beats = length_,
                .beat = beat,
                .source_increment = false,
                .destination_increment = true,
                .block_action = DmaBlockAction::interrupt,
            })) {
            return false;
        }
        running_ = true;
        // NO KICK HERE - see the class comment. A kick is right only
        // where the peripheral's request is ALREADY STANDING, which the
        // owner is the only thing that can see; issued blind it would
        // overwrite a value the peripheral has not taken.
        Channel::enable(true);
        return true;
    }

    /// See DmaTxEngine::claim() - the same act, the same two reasons.
    static void claim() {
        (void)Channel::reset();
        (void)Channel::configure({
            .trigger = trigger_,
            .action = DmaTriggerAction::beat,
            .priority = priority_,
        });
        Channel::arm(DmaFlag::complete | DmaFlag::transfer_error, true);
    }

    static inline volatile void* data_ = nullptr;
    static inline volatile Elem* buffer_[2] = {nullptr, nullptr};
    static inline uint8_t trigger_ = dma_trigger_none;
    static inline DmaPriority priority_ = DmaPriority::level0;
    static inline uint16_t length_ = 0;
    // laps_ and overruns_ are written in the DMAC handler and read from
    // thread context, typically in a polling loop - the ticker's lesson
    // (gcc -Os deleted a bare polling loop), so their loads must be
    // volatile. faults_ is thread-written and stays plain.
    static inline volatile uint32_t laps_ = 0;
    static inline volatile uint32_t overruns_ = 0;
    static inline uint32_t faults_ = 0;
    static inline volatile uint8_t fill_ = 0;
    static inline volatile uint8_t drain_ = 0;
    static inline volatile uint8_t pending_ = 0;
    static inline volatile bool stalled_ = false;
    static inline volatile bool running_ = false;
};

} // namespace brio
