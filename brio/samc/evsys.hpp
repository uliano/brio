/*
 * evsys.hpp
 *
 * The SAM C21 Event System (DS60001479M ch. 29): twelve channels
 * carrying events from any of 95 generators to any of 47 users, with no
 * CPU in the path.
 *
 * THIS IS THE ONE PLACE WHERE THE AVR SHAPE DOES NOT TRANSFER, and the
 * difference is worth stating before any verb. On the AVR the event
 * system is a small fixed table: a channel is a typed thing, a generator
 * is a type, and legality is a compile-time question that
 * `avrdx/evsys.hpp` answers with per-generator types. Here it is an
 * ALLOCATOR - twelve identical channels, numeric generator and user
 * codes drawn from two tables ninety-five and forty-seven rows long,
 * and a per-channel generic clock. Reproducing the AVR's per-generator
 * types would mean 95 of them, and they would encode a table this
 * header has no business owning: which generator a peripheral offers is
 * that peripheral's driver's knowledge, and it should hand over a code.
 *
 * So this header owns the FABRIC and not the vocabulary. It moves
 * channels, users, paths and edges; a driver that generates events
 * publishes its generator codes, and a driver that consumes them
 * publishes its user index. That division is what keeps this file from
 * growing a table for every chapter in the book.
 *
 * THE FOUR RULES OF THE CHAPTER, each with code behind it.
 *
 * 1. THE USER MULTIPLEXER IS WRITTEN BEFORE THE CHANNEL. "The user
 *    multiplexer must always be configured before the channel"
 *    (29.6.2.3), which is why `connect()` exists as one verb taking
 *    both and there is no way to do it in the wrong order by accident.
 *
 * 2. USER.CHANNEL IS THE CHANNEL PLUS ONE. Zero means "no channel
 *    output selected" and channel m is selected by writing m+1
 *    (29.8.9's own note). Every verb here speaks plain channel numbers
 *    and the off-by-one lives in exactly two lines.
 *
 * 3. THE PATH DECIDES WHETHER EDGE DETECTION IS LEGAL, and both ways.
 *    On the asynchronous path "the edge detection is not required and
 *    must be disabled by software"; on the synchronous and
 *    resynchronized paths "edge detection must be enabled" (29.6.2.6,
 *    29.6.2.7). A configuration that gets this backwards is refused
 *    rather than written.
 *
 * 4. THE ASYNCHRONOUS PATH HAS NO STATUS AT ALL. No interrupts, and
 *    CHSTATUS and both interrupt flags read as zero for that channel
 *    (29.6.2.9, 29.6.2.10, 29.6.2.11). Code that polls `busy()` to pace
 *    an asynchronous channel is polling a constant.
 *
 * ERRATA: THREE OF THE FOUR ARE LIVE ON THIS SILICON, which is a lot
 * for a twenty-page chapter.
 *
 *  - 1.12.1 (all revisions): in SYNCHRONOUS mode a channel whose
 *    generic clock is always on (ONDEMAND = 0) can raise SPURIOUS
 *    OVERRUN interrupts. The workaround is ONDEMAND = 1, so that is the
 *    default here and a synchronous configuration that clears it is
 *    refused.
 *  - 1.12.3 (all revisions): a software event on a RESYNCHRONIZED path
 *    does not set CHBUSY immediately, and a second event arriving in
 *    that window is LOST WITH NO OVERRUN FLAG - the worst kind of
 *    silence. The chapter's remedy is to wait three
 *    GCLK_EVSYS_CHANNEL_n cycles before the next software event.
 *    `trigger()` cannot know that clock's rate, so the wait is the
 *    caller's; what this header does is say so and offer `busy()`.
 *  - 1.12.4 (all revisions): a freshly configured and enabled channel is
 *    busy for one GCLK tick WITHOUT CHBUSY showing it, so a trigger
 *    issued immediately can be swallowed. Visible when the EVSYS clock
 *    is slower than the CPU - which is the interesting case. Again the
 *    wait is the caller's and `configure()` says so.
 *  - 1.12.2 is NOT this silicon (E/G/J revisions B..E only).
 *
 * NOT BUILT (docs/samc/evsys.md carries the list): the generator and
 * user TABLES, deliberately - see above; and SleepWalking, which needs
 * the power pass.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/clock.hpp"

namespace brio {

/// CHANNELn.PATH (29.6.2.6).
enum class EventPath : uint8_t {
    /// Straight through, no clock, no latency, no status, no interrupts.
    asynchronous = EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val,
    /// One GCLK_EVSYS_CHANNEL_n of latency. For a generator that shares
    /// the channel's generator; a generator on a different one may not
    /// be seen at all.
    synchronous = EVSYS_CHANNEL_PATH_SYNCHRONOUS_Val,
    /// Three cycles of latency, and the path to use when the generator
    /// and the channel do not share a generic clock generator.
    resynchronized = EVSYS_CHANNEL_PATH_RESYNCHRONIZED_Val,
};

/// CHANNELn.EDGSEL (29.6.2.7). `none` is the register's NO_EVT_OUTPUT.
enum class EventEdge : uint8_t {
    none = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val,
    rising = EVSYS_CHANNEL_EDGSEL_RISING_EDGE_Val,
    falling = EVSYS_CHANNEL_EDGSEL_FALLING_EDGE_Val,
    both = EVSYS_CHANNEL_EDGSEL_BOTH_EDGES_Val,
};

/**
 * One channel's whole CHANNELn register.
 *
 * `generator` is a NUMERIC CODE and that is deliberate: the table of 95
 * of them belongs to the peripherals that offer them, not here. Zero -
 * the reset value - selects no generator at all, which is exactly what a
 * channel driven only by software events wants.
 */
struct EventChannelConfig {
    uint8_t generator = 0;
    EventPath path = EventPath::asynchronous;
    EventEdge edge = EventEdge::none;

    /// CHANNELn.ONDEMAND, and it defaults TRUE because of erratum
    /// 1.12.1: a synchronous channel whose clock never stops raises
    /// spurious overruns.
    bool on_demand = true;
    bool run_standby = false;
};

/**
 * The event system as a monostate resource: the fabric, the status and
 * the software trigger. Channels are addressed by number rather than by
 * type because that is what an allocator is - and because a driver
 * handing over a generator code has no channel type to hand it to.
 */
struct Evsys {
    Evsys() = delete;

    static constexpr uint8_t channel_count = EVSYS_CHANNELS;   // 12
    /// The device header sizes the USER array; the chapter's table runs
    /// two rows longer for the N family, which this part is not.
    static constexpr uint8_t user_count = 47;

    static constexpr IRQn_Type irq() { return EVSYS_IRQn; }

    /// Each channel has its own generic clock, and the IDs are
    /// contiguous from channel 0's - which is a fact of the header and
    /// not a formula this header invents, so it is anchored to the
    /// header's own first constant.
    static constexpr uint8_t gclk_id(uint8_t channel) {
        return static_cast<uint8_t>(EVSYS_GCLK_ID_0 + channel);
    }

    static constexpr bool valid_channel(uint8_t c) { return c < channel_count; }
    static constexpr bool valid_user(uint8_t u) { return u < user_count; }

    static constexpr bool config_valid(const EventChannelConfig& c) {
        if (c.generator > (EVSYS_CHANNEL_EVGEN_Msk >> EVSYS_CHANNEL_EVGEN_Pos)) {
            return false;
        }
        // Rule 3: the path decides whether an edge is legal, both ways.
        if (c.path == EventPath::asynchronous) {
            return c.edge == EventEdge::none;
        }
        if (c.edge == EventEdge::none) {
            return false;
        }
        // Erratum 1.12.1: a synchronous channel with a free-running
        // clock raises spurious overruns, and the workaround is the only
        // configuration this driver will write.
        return !(c.path == EventPath::synchronous && !c.on_demand);
    }

    // ---- claim and teardown ------------------------------------------------

    static void bus_clock(bool on) { Mclk::apb_c(MCLK_APBCMASK_EVSYS_Msk, on); }

    /// The EVSYS is always enabled (29.6.2.2); the only global verb is a
    /// software reset, which cancels every ongoing event.
    static void reset() { EVSYS_REGS->EVSYS_CTRLA = EVSYS_CTRLA_SWRST_Msk; }

    // ---- channels ----------------------------------------------------------

    /**
     * Write one channel's configuration.
     *
     * ERRATUM 1.12.4, ALL REVISIONS: for one GCLK_EVSYS_CHANNEL_n tick
     * after this returns the channel is busy WITHOUT CHBUSY showing it,
     * and a trigger issued inside that window can be swallowed. This
     * header cannot wait it out - it does not know the channel clock's
     * rate, and on a slow EVSYS clock the wait is long - so the caller
     * must pace the first trigger. It matters exactly when the EVSYS
     * clock is slower than the CPU, which is the interesting case.
     */
    static bool configure(uint8_t channel, const EventChannelConfig& cfg) {
        if (!valid_channel(channel) || !config_valid(cfg)) {
            return false;
        }
        EVSYS_REGS->EVSYS_CHANNEL[channel] =
            EVSYS_CHANNEL_EVGEN(cfg.generator) |
            EVSYS_CHANNEL_PATH(static_cast<uint32_t>(cfg.path)) |
            EVSYS_CHANNEL_EDGSEL(static_cast<uint32_t>(cfg.edge)) |
            (cfg.on_demand ? EVSYS_CHANNEL_ONDEMAND_Msk : 0u) |
            (cfg.run_standby ? EVSYS_CHANNEL_RUNSTDBY_Msk : 0u);
        return true;
    }

    static uint32_t channel_reg(uint8_t channel) {
        return valid_channel(channel) ? EVSYS_REGS->EVSYS_CHANNEL[channel] : 0u;
    }

    /// Point a channel at nothing: generator 0, asynchronous, no edge.
    static void release_channel(uint8_t channel) {
        if (valid_channel(channel)) {
            EVSYS_REGS->EVSYS_CHANNEL[channel] = 0u;
        }
    }

    // ---- users -------------------------------------------------------------

    /**
     * Connect a user to a channel - the one verb, taking both, because
     * 29.6.2.3 requires the user multiplexer to be written BEFORE the
     * channel and a two-verb API would let a caller get that backwards.
     * `configure()` is therefore called from inside, after the user.
     */
    static bool connect(uint8_t user, uint8_t channel,
                        const EventChannelConfig& cfg) {
        if (!valid_user(user) || !valid_channel(channel) || !config_valid(cfg)) {
            return false;
        }
        // The user multiplexer first (29.6.2.3), and the +1 the register
        // wants lives here and in user_channel() and nowhere else.
        EVSYS_REGS->EVSYS_USER[user] =
            EVSYS_USER_CHANNEL(static_cast<uint32_t>(channel) + 1u);
        return configure(channel, cfg);
    }

    /// Attach one more user to a channel someone else already
    /// configured - "several event users can share the same channel and
    /// therefore answer to the same event" (29.2).
    static bool attach(uint8_t user, uint8_t channel) {
        if (!valid_user(user) || !valid_channel(channel)) {
            return false;
        }
        EVSYS_REGS->EVSYS_USER[user] =
            EVSYS_USER_CHANNEL(static_cast<uint32_t>(channel) + 1u);
        return true;
    }

    static void disconnect(uint8_t user) {
        if (valid_user(user)) {
            EVSYS_REGS->EVSYS_USER[user] = 0u;
        }
    }

    /// Which channel a user listens to, in plain channel numbers.
    /// `channel_count` means "none" - the register's own zero.
    static uint8_t user_channel(uint8_t user) {
        if (!valid_user(user)) {
            return channel_count;
        }
        const uint32_t raw =
            (EVSYS_REGS->EVSYS_USER[user] & EVSYS_USER_CHANNEL_Msk) >>
            EVSYS_USER_CHANNEL_Pos;
        return raw == 0u ? channel_count : static_cast<uint8_t>(raw - 1u);
    }

    // ---- status ------------------------------------------------------------
    //
    // ALL OF THIS READS ZERO ON AN ASYNCHRONOUS CHANNEL (29.6.2.11), so
    // a caller polling one of these to pace an async channel is polling
    // a constant. The verbs are here for the synchronous paths.

    static uint32_t channel_status() { return EVSYS_REGS->EVSYS_CHSTATUS; }

    /// CHBUSYn: an event on this channel has not been taken by every
    /// user connected to it. The bits live in the high half.
    static bool busy(uint8_t channel) {
        return valid_channel(channel) &&
               (channel_status() & (EVSYS_CHSTATUS_CHBUSY0_Msk << channel)) != 0u;
    }
    /// USRRDYn: every user on this channel is ready for another event.
    static bool users_ready(uint8_t channel) {
        return valid_channel(channel) &&
               (channel_status() & (EVSYS_CHSTATUS_USRRDY0_Msk << channel)) != 0u;
    }

    static uint32_t flags() { return EVSYS_REGS->EVSYS_INTFLAG; }
    static uint32_t armed() { return EVSYS_REGS->EVSYS_INTENSET; }
    static void clear_flags(uint32_t mask) { EVSYS_REGS->EVSYS_INTFLAG = mask; }
    static void arm(uint32_t mask) { EVSYS_REGS->EVSYS_INTENSET = mask; }
    static void disarm(uint32_t mask) { EVSYS_REGS->EVSYS_INTENCLR = mask; }

    /// INTFLAG bit masks for one channel: OVR in the low half, EVD in
    /// the high one - the same split CHSTATUS uses.
    static constexpr uint32_t overrun_flag(uint8_t channel) {
        return EVSYS_INTFLAG_OVR0_Msk << channel;
    }
    static constexpr uint32_t detected_flag(uint8_t channel) {
        return EVSYS_INTFLAG_EVD0_Msk << channel;
    }

    static bool overrun(uint8_t channel) {
        return valid_channel(channel) && (flags() & overrun_flag(channel)) != 0u;
    }
    static bool detected(uint8_t channel) {
        return valid_channel(channel) && (flags() & detected_flag(channel)) != 0u;
    }

    /// The ISR body; the app binds EVSYS_Handler.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t p = flags() & armed();
        if (p != 0u) {
            clear_flags(p);
        }
        return p;
    }

    // ---- the software event ------------------------------------------------

    /**
     * Raise an event on a channel from software (29.6.2.12) - serviced
     * as a generator's would be, whatever EVGEN says, which is what
     * makes a channel with no generator useful and what makes this
     * whole subsystem testable without a single wire.
     *
     * WHAT AN ASYNCHRONOUS CHANNEL DOES WITH ONE IS THE USER'S
     * BUSINESS, not the path's - measured twice, from opposite ends.
     * Eight back-to-back software events on an asynchronous channel move
     * nothing through a DMA channel, where one on a clocked path moves a
     * whole block (docs/samc/evsys.md); but sixteen of sixteen single
     * ones reach a CCL LUT on that same asynchronous path, with a
     * disconnected-user control catching none and one of them moving a
     * DMA block THROUGH the LUT (docs/samc/ccl.md). A register write has
     * no width for the DMAC's trigger stage; the CCL's event input has
     * an edge detector of its own and catches every one. So a channel
     * meant for software events needs a clocked path FOR THE DMAC, and
     * nothing here can promise more than that about a user it has never
     * seen.
     *
     * SWEVT IS WRITE-ONLY (the device header declares it `__O`), so
     * there is nothing to read back and no read-modify-write to get
     * wrong: one store, one channel.
     *
     * ERRATUM 1.12.3, ALL REVISIONS: on a RESYNCHRONIZED path CHBUSY is
     * not set immediately after this, and a second software event
     * arriving inside that window is LOST WITHOUT RAISING THE OVERRUN
     * FLAG. Three GCLK_EVSYS_CHANNEL_n cycles is the chapter's remedy,
     * and it is the caller's to spend - this header does not know that
     * clock's rate.
     */
    static void trigger(uint8_t channel) {
        if (valid_channel(channel)) {
            EVSYS_REGS->EVSYS_SWEVT = static_cast<uint32_t>(1u) << channel;
        }
    }
};

} // namespace brio
