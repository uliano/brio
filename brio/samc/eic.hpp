/*
 * eic.hpp
 *
 * The SAM C21 External Interrupt Controller (DS60001479M ch. 26): the
 * peripheral where this family keeps its PIN INTERRUPTS. There are none
 * in PORT - samc/pin.hpp says so and stops there - so everything the
 * AVR spells in PINnCTRL.ISC lives here, one register field per line,
 * behind a peripheral of its own with its own clock, its own enable and
 * its own event outputs.
 *
 *  Eic              the block: APB clock, the clock CHOICE (GCLK_EIC or
 *                   CLK_ULP32K), reset and enable with their
 *                   synchronization, the per-line sense/filter/async
 *                   configuration, INTFLAG/INTENSET, the event outputs,
 *                   the NMI, and the ISR body.
 *
 *  ExtInt<Pin>      one line reached through the PAD that carries it:
 *                   the pad's EXTINT number comes from the device
 *                   header (`PIN_PxyA_EIC_EXTINT_NUM`), never from a
 *                   formula, and a pad the header does not bond refuses
 *                   to compile.
 *
 * THREE FACTS THAT SHAPE THE FILE.
 *
 * 1. THE PAD-TO-LINE MAP IS NOT A FORMULA. PA16 is line 0, PA24 is line
 *    12, PA27 is line 15, PB30 is line 14: any "pin number modulo 16"
 *    rule is wrong on this family. The device header carries one
 *    `PIN_P<pad>A_EIC_EXTINT_NUM` per bonded pad and that is the whole
 *    authority - probed, symbol by symbol, in samc/device_tables.hpp
 *    (the one file where vendor-macro `#ifdef` walls are allowed to
 *    live), so a pad that a smaller package does not bond simply has
 *    no entry and `ExtInt<>` on it fails to compile with a message
 *    that says why.
 *
 * 2. ONE NVIC VECTOR FOR ALL SIXTEEN LINES. 26.6.6 says "The EIC has
 *    one interrupt request line for each external interrupt (EXTINTx)",
 *    but the device header gives this part a single `EIC_IRQn` - the
 *    same shape the SERCOM has here, and the same discipline: the ISR
 *    body masks INTFLAG with INTENSET and dispatches on the result. The
 *    NMI is the exception and is genuinely separate: it has its own
 *    exception vector, is always enabled, and cannot be masked.
 *
 * 3. THE CLOCK IS OPTIONAL AND KNOWING WHEN IS THE WHOLE TRICK
 *    (26.5.3, 26.6.3). Level detection with no filter is done
 *    ASYNCHRONOUSLY and needs no clock at all; so does edge detection
 *    with ASYNCH set. Filtering, and synchronous edge detection, need
 *    GCLK_EIC or CLK_ULP32K - and in those modes the pin is SAMPLED, so
 *    "pulses with duration lower than two EIC clock periods may not be
 *    properly detected". `needs_clock()` answers the question from a
 *    configuration, so an application never has to re-derive it.
 *
 *    AND THE CONSEQUENCE THE CHAPTER NEVER DRAWS, measured here
 *    (docs/samc/eic.md carries it): 26.6.3 says the EIC "automatically
 *    requests GCLK_EIC or CLK_ULP32K to operate" in the sampled modes,
 *    and CTRLA.ENABLE is write-synchronized AGAINST THAT REQUESTED
 *    CLOCK. So WHETHER THE BLOCK CAN BE ENABLED AT ALL DEPENDS ON WHAT
 *    ITS LINES ASKED FOR. With GCLK_EIC disconnected, a block whose
 *    lines are all clockless (level, or asynchronous edge) enables,
 *    detects and disables perfectly; the moment ONE line asks to be
 *    sampled, `enable(true)` writes the bit - which reads back at once,
 *    as 26.8.1 promises - and SYNCBUSY.ENABLE stands forever, with
 *    nothing detected at all. The write is PENDING rather than lost:
 *    connect the clock and that same enable completes with no second
 *    write. `enable()` returning false is the only warning there is,
 *    which is why it returns one. The cheap way out is CLK_ULP32K,
 *    which needs no GCLK channel and is always running on this family.
 *
 *    Table 26-2's worst-case detection latencies, for reference:
 *      level, no filter    5 CLK_EIC_APB
 *      level + filter      4 EIC clocks + 5 CLK_EIC_APB
 *      edge, no filter     4 EIC clocks + 5 CLK_EIC_APB
 *      edge + filter       6 EIC clocks + 5 CLK_EIC_APB
 *
 * ENABLE-PROTECTION IS THE STRUCTURAL RULE HERE. CONFIGn, ASYNCH,
 * EVCTRL (and CTRLA.CKSEL) are writable only while CTRLA.ENABLE is zero
 * (26.6.2.1). Every verb that touches one of them therefore REFUSES
 * while the block is enabled rather than storing into a register the
 * silicon will not take - which is why they return bool.
 *
 * ERRATA, DS80000740S, read on the E/G/J ROW at revision F - and the
 * row matters more here than in any chapter so far, because FIVE of the
 * six EIC items look alarming and only ONE of them is this silicon:
 *
 *  - 1.11.1 NMI Exception (an NMI fires as soon as the config is
 *    written, with a pull-up and a rising-edge sense): revision B only.
 *  - 1.11.2 ASYNCH is not write-protected: revision B only.
 *  - 1.11.3 Spurious Flag (a flag appears at CTRLA.ENABLE for a
 *    low/rising/both line with the filter on): revisions B..E. Not this
 *    chip, so `enable()` does NOT quietly clear INTFLAG behind the
 *    caller's back - on an affected part the caller clears INTFLAG
 *    after enabling and before arming, which is the erratum's own
 *    workaround.
 *  - 1.11.4 False NMI Interrupt on an on-the-fly NMISENSE change:
 *    revisions B..E. `nmi_configure()` therefore does not clear
 *    NMIFLAG for you either; it says so.
 *  - 1.11.5 Edge Detection (SYNCBUSY.ENABLE released three EIC clocks
 *    before edge detection actually works): the N FAMILY ONLY. This is
 *    the item most likely to be applied by mistake on this part.
 *  - 1.11.6 Edge Detection in Standby: EVERY REVISION OF E/G/J, so LIVE
 *    HERE. With ASYNCH set and the device in Standby only the FIRST
 *    edge is detected; the rest are ignored until the device wakes.
 *    Microchip's workaround for this family is blunt - "asynchronous
 *    edge detection doesn't work, instead use the synchronous edge
 *    detection" - and the cheap clock for that is CLK_ULP32K. This
 *    driver cannot enforce it, because it does not know whether the
 *    application ever sleeps: `asynchronous` is legal and useful while
 *    the device is awake, and `EicLineConfig::asynchronous` carries the
 *    obligation in its own comment. The power pass owns the rest.
 *
 * NOT BUILT (docs/samc/eic.md carries the list): the debouncer
 * (DEBOUNCEN/DPRESCALER/PINSTATE - SAM C20/C21 N variants only, and the
 * device header for this family does not even declare the registers, so
 * there is nothing to gate); and sleep/wake behaviour beyond the
 * erratum above, which belongs to the power pass together with
 * util/power.hpp's SleepSite.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/device_tables.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/// CONFIGn.SENSEx and NMICTRL.NMISENSE - the same encoding for both
/// (26.8.10, 26.8.2). The two reserved codes 0x6/0x7 are simply not
/// spellable.
enum class EicSense : uint8_t {
    none = EIC_CONFIG_SENSE0_NONE_Val,
    rising = EIC_CONFIG_SENSE0_RISE_Val,
    falling = EIC_CONFIG_SENSE0_FALL_Val,
    both = EIC_CONFIG_SENSE0_BOTH_Val,
    high = EIC_CONFIG_SENSE0_HIGH_Val,
    low = EIC_CONFIG_SENSE0_LOW_Val,
};

constexpr bool eic_sense_is_edge(EicSense s) {
    return s == EicSense::rising || s == EicSense::falling || s == EicSense::both;
}
constexpr bool eic_sense_is_level(EicSense s) {
    return s == EicSense::high || s == EicSense::low;
}

/// CTRLA.CKSEL (26.8.1): which clock the sampled parts of the block run
/// on. GCLK_EIC buys a rate the filter can be designed around;
/// CLK_ULP32K is the OSCULP32K straight from OSC32KCTRL and costs
/// almost nothing - and needs no GCLK channel at all.
enum class EicClock : uint8_t {
    gclk = EIC_CTRLA_CKSEL_CLK_GCLK_Val,
    ulp32k = EIC_CTRLA_CKSEL_CLK_ULP32K_Val,
};

/**
 * One external interrupt line's whole configuration - the SENSEx and
 * FILTENx fields of CONFIGn, plus that line's bits of ASYNCH and
 * EVCTRL, because those three registers describe ONE line between them
 * and a caller that sets them separately can only get them out of step.
 */
struct EicLineConfig {
    EicSense sense = EicSense::none;

    /// FILTENx: the majority-of-three filter, sampled at the EIC clock
    /// (26.6.3, table 26-1). Costs the clock and two more sampling
    /// periods of latency.
    bool filter = false;

    /**
     * ASYNCH[x]: edge detection with no clock (26.6.4.2). The flag is
     * set directly by the pad rather than by comparing two samples, so
     * a pulse of any width is caught and no EIC clock is needed.
     *
     * ERRATUM 1.11.6, EVERY REVISION OF THIS FAMILY: in STANDBY only
     * the first such edge is detected and the rest are lost until the
     * device wakes. A line that must wake the device repeatedly from
     * standby has to use SYNCHRONOUS edge detection instead (leave this
     * false and give the block a clock - CLK_ULP32K is the cheap one).
     * While the device is awake this bit is the fast, clockless path
     * and there is nothing wrong with it.
     */
    bool asynchronous = false;

    /// EVCTRL.EXTINTEO[x]: this line also drives its EVSYS generator.
    /// The generator's code is Eic::event_generator(line).
    bool event_out = false;
};

/// The chapter's own rule, as one question: does this configuration
/// need GCLK_EIC or CLK_ULP32K to be running (26.5.3, 26.6.3)?
/// Level detection without a filter, and asynchronous edge detection,
/// are the two clockless modes.
constexpr bool eic_needs_clock(const EicLineConfig& c) {
    if (c.filter) {
        return true;
    }
    return eic_sense_is_edge(c.sense) && !c.asynchronous;
}

/// 26.8.10's own note: "The filter must be disabled if the asynchronous
/// detection is enabled." Everything else the register can hold is
/// legal, including a level sense with no detection at all.
constexpr bool eic_line_config_valid(const EicLineConfig& c) {
    return !(c.filter && c.asynchronous);
}

/// NMICTRL (26.8.2). The NMI is a line apart: its own register, its own
/// exception vector, always enabled, and detection turned on by
/// NMISENSE alone - "the EIC module is not required to be enabled"
/// (26.6.4.1).
struct EicNmiConfig {
    EicSense sense = EicSense::none;
    bool filter = false;         ///< NMIFILTEN
    bool asynchronous = false;   ///< NMIASYNCH
};

constexpr bool eic_nmi_config_valid(const EicNmiConfig& c) {
    return !(c.filter && c.asynchronous);
}

constexpr bool eic_nmi_needs_clock(const EicNmiConfig& c) {
    if (c.filter) {
        return true;
    }
    return eic_sense_is_edge(c.sense) && !c.asynchronous;
}

// =============================================================================
// The block
// =============================================================================

class Eic {
public:
    Eic() = delete;

    /// From the device header, not from the chapter's "up to 16".
    static constexpr uint8_t line_count = EIC_EXTINT_NUM;
    static constexpr uint8_t config_regs = EIC_NUMBER_OF_CONFIG_REGS;
    static constexpr uint8_t gclk_id = EIC_GCLK_ID;

    /// ONE vector for all sixteen lines (see the file header). The NMI
    /// has its own exception and is not an IRQn at all.
    static constexpr IRQn_Type irq() { return EIC_IRQn; }

    static eic_registers_t& regs() { return *EIC_REGS; }

    static constexpr bool valid_line(uint8_t line) { return line < line_count; }

    /**
     * The EVSYS generator code of one line (the generator table of ch.
     * 29: EXTINT0 is 0x0E and the lines are contiguous from there).
     * Published HERE and not in evsys.hpp, which owns the fabric and
     * not the vocabulary.
     *
     * All sixteen lines are generators, whatever 26.6.7 says: that
     * section's prose reads "External event from pin (EXTINT0-7)" while
     * its own EVCTRL register is sixteen bits wide and the EVSYS
     * generator table lists EXTINT0..EXTINT15. The register and the
     * table agree with each other and with the silicon
     * (docs/samc/eic.md carries the measurement); the sentence does
     * not.
     */
    static constexpr uint8_t event_generator(uint8_t line) {
        return static_cast<uint8_t>(0x0Eu + line);
    }

    // ---- claim and teardown ------------------------------------------------

    static void bus_clock(bool on) { Mclk::apb_a(MCLK_APBAMASK_EIC_Msk, on); }

    static bool sync_wait(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().EIC_SYNCBUSY, mask, false, spins);
    }

    /// CTRLA.SWRST - write-synchronized, and it disables the block.
    static bool reset(uint32_t spins = 0xFFFFu) {
        regs().EIC_CTRLA = EIC_CTRLA_SWRST_Msk;
        return sync_wait(EIC_SYNCBUSY_SWRST_Msk, spins);
    }

    /**
     * CTRLA.ENABLE, write-synchronized both ways.
     *
     * FALSE MEANS THE BLOCK IS NOT RUNNING, and the usual reason is
     * that a line asked to be SAMPLED while the block has no clock:
     * this write synchronizes against the clock the configured lines
     * request, so with GCLK_EIC disconnected a filtered or
     * synchronously detected line makes the enable stand pending
     * forever (bit written, bit readable, SYNCBUSY.ENABLE set), while a
     * block of purely clockless lines enables at once. See the file
     * header. Nothing here can wait that out, because there is no rate
     * to wait for.
     *
     * It deliberately does NOT touch INTFLAG. Erratum 1.11.3 (a
     * spurious flag at enable on a low/rising/both line with the filter
     * on) is revisions B..E, not this silicon, and clearing flags a
     * caller may be relying on would be worse than the erratum.
     */
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint8_t v = static_cast<uint8_t>(regs().EIC_CTRLA &
                                               ~EIC_CTRLA_ENABLE_Msk);
        regs().EIC_CTRLA =
            static_cast<uint8_t>(on ? (v | EIC_CTRLA_ENABLE_Msk) : v);
        return sync_wait(EIC_SYNCBUSY_ENABLE_Msk, spins);
    }
    static bool enabled() { return (regs().EIC_CTRLA & EIC_CTRLA_ENABLE_Msk) != 0u; }

    /**
     * CTRLA.CKSEL - enable-protected, and NOT write-synchronized (the
     * register description is explicit about the second half). Refused
     * while the block is enabled rather than silently dropped.
     *
     * `gclk` needs the GCLK_EIC channel connected as well; `ulp32k`
     * needs nothing but a running OSCULP32K, which on this family is
     * always running.
     */
    static bool clock_select(EicClock c) {
        if (enabled()) {
            return false;
        }
        const uint8_t v = static_cast<uint8_t>(regs().EIC_CTRLA &
                                               ~EIC_CTRLA_CKSEL_Msk);
        regs().EIC_CTRLA =
            static_cast<uint8_t>(v | EIC_CTRLA_CKSEL(static_cast<uint8_t>(c)));
        return true;
    }
    static EicClock clock_select() {
        return static_cast<EicClock>((regs().EIC_CTRLA & EIC_CTRLA_CKSEL_Msk) >>
                                     EIC_CTRLA_CKSEL_Pos);
    }

    /// Point GCLK_EIC at a generator (only needed for EicClock::gclk).
    static bool clock(uint8_t generator, uint32_t spins = 0xFFFFu) {
        return GclkChannel::connect(gclk_id, generator, spins);
    }

    /**
     * APB clock on, everything back to reset, block left DISABLED -
     * because every configuration register is enable-protected and the
     * caller has lines to write before enabling. 26.6.2.1's order is
     * exactly this: bus clock, NMI, EIC clock, CONFIGn, ASYNCH, ENABLE.
     */
    static bool init(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        bus_clock(true);
        return reset(spins);
    }

    static void release(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        (void)reset(spins);
        GclkChannel::disconnect(gclk_id);
        bus_clock(false);
    }

    // ---- lines -------------------------------------------------------------

    /**
     * Write one line's SENSE/FILTEN, its ASYNCH bit and its EVCTRL bit.
     *
     * ENABLE-PROTECTED, all three registers (26.6.2.1), so this refuses
     * while the block is enabled instead of writing into registers the
     * silicon ignores. It is also the only verb that touches those
     * registers, which is what keeps the three descriptions of one line
     * in step.
     */
    static bool configure_line(uint8_t line, const EicLineConfig& cfg) {
        if (!valid_line(line) || !eic_line_config_valid(cfg) || enabled()) {
            return false;
        }
        const uint8_t reg = static_cast<uint8_t>(line / 8u);
        const uint32_t shift = static_cast<uint32_t>(line % 8u) * 4u;
        const uint32_t field =
            (static_cast<uint32_t>(cfg.sense) << EIC_CONFIG_SENSE0_Pos) |
            (cfg.filter ? EIC_CONFIG_FILTEN0_Msk : 0u);
        regs().EIC_CONFIG[reg] =
            (regs().EIC_CONFIG[reg] & ~(0xFu << shift)) | (field << shift);

        const uint32_t bit = static_cast<uint32_t>(1u) << line;
        regs().EIC_ASYNCH =
            cfg.asynchronous ? (regs().EIC_ASYNCH | bit) : (regs().EIC_ASYNCH & ~bit);
        regs().EIC_EVCTRL =
            cfg.event_out ? (regs().EIC_EVCTRL | bit) : (regs().EIC_EVCTRL & ~bit);
        return true;
    }

    /// Read one line's configuration back out of the three registers.
    static EicLineConfig line_config(uint8_t line) {
        if (!valid_line(line)) {
            return {};
        }
        const uint8_t reg = static_cast<uint8_t>(line / 8u);
        const uint32_t shift = static_cast<uint32_t>(line % 8u) * 4u;
        const uint32_t field = (regs().EIC_CONFIG[reg] >> shift) & 0xFu;
        const uint32_t bit = static_cast<uint32_t>(1u) << line;
        return EicLineConfig{
            .sense = static_cast<EicSense>(field & EIC_CONFIG_SENSE0_Msk),
            .filter = (field & EIC_CONFIG_FILTEN0_Msk) != 0u,
            .asynchronous = (regs().EIC_ASYNCH & bit) != 0u,
            .event_out = (regs().EIC_EVCTRL & bit) != 0u,
        };
    }

    /// Sense NONE, no filter, no async, no event: the line back to
    /// reset. Enable-protected like configure_line().
    static bool release_line(uint8_t line) {
        return configure_line(line, EicLineConfig{});
    }

    // ---- flags and interrupts ----------------------------------------------

    static constexpr uint32_t line_mask(uint8_t line) {
        return static_cast<uint32_t>(1u) << line;
    }

    static uint32_t flags() { return regs().EIC_INTFLAG; }
    /// INTFLAG is W1C, so this is a plain store: no read-modify-write.
    static void clear_flags(uint32_t mask) { regs().EIC_INTFLAG = mask; }
    static uint32_t armed() { return regs().EIC_INTENSET; }
    static void arm(uint32_t mask) { regs().EIC_INTENSET = mask; }
    static void disarm(uint32_t mask) { regs().EIC_INTENCLR = mask; }

    static bool flag(uint8_t line) {
        return valid_line(line) && (flags() & line_mask(line)) != 0u;
    }
    static void clear_flag(uint8_t line) {
        if (valid_line(line)) {
            clear_flags(line_mask(line));
        }
    }

    /**
     * The ISR body; the app binds EIC_Handler and dispatches on the
     * returned mask (bit x = EXTINTx).
     *
     * INTENSET masks the read exactly as the SERCOM's does, because one
     * vector serves sixteen sources and a flag that nobody armed is not
     * this handler's business. IN LEVEL-SENSITIVE MODE THE FLAG COMES
     * STRAIGHT BACK if the pin still matches (26.6.3), so a handler
     * that only clears will spin: the level has to be dealt with, or
     * the line disarmed.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t p = flags() & armed();
        if (p != 0u) {
            clear_flags(p);
        }
        return p;
    }

    // ---- the NMI -----------------------------------------------------------

    /**
     * Write NMICTRL. NOT enable-protected and NOT dependent on
     * CTRLA.ENABLE at all: "NMI detection is enabled only by the
     * NMICTRL.NMISENSE value, and the EIC module is not required to be
     * enabled" (26.6.4.1). The bus clock IS required.
     *
     * NMIFLAG is deliberately left alone. Erratum 1.11.4 (a false NMI
     * from an on-the-fly NMISENSE change, remedied by clearing NMIFLAG
     * afterwards) is revisions B..E and not this silicon; on an
     * affected part the caller clears it, and the flag stays the
     * caller's to read either way.
     */
    static bool nmi_configure(const EicNmiConfig& cfg) {
        if (!eic_nmi_config_valid(cfg)) {
            return false;
        }
        regs().EIC_NMICTRL = static_cast<uint8_t>(
            EIC_NMICTRL_NMISENSE(static_cast<uint8_t>(cfg.sense)) |
            (cfg.filter ? EIC_NMICTRL_NMIFILTEN_Msk : 0u) |
            (cfg.asynchronous ? EIC_NMICTRL_NMIASYNCH_Msk : 0u));
        return true;
    }

    static EicNmiConfig nmi_config() {
        const uint8_t v = regs().EIC_NMICTRL;
        return EicNmiConfig{
            .sense = static_cast<EicSense>(v & EIC_NMICTRL_NMISENSE_Msk),
            .filter = (v & EIC_NMICTRL_NMIFILTEN_Msk) != 0u,
            .asynchronous = (v & EIC_NMICTRL_NMIASYNCH_Msk) != 0u,
        };
    }

    static bool nmi_flag() { return (regs().EIC_NMIFLAG & EIC_NMIFLAG_NMI_Msk) != 0u; }
    static void clear_nmi_flag() { regs().EIC_NMIFLAG = EIC_NMIFLAG_NMI_Msk; }

    /// The NMI exception body; the app binds NonMaskableInt_Handler.
    /// There is no enable to consult - an NMI is always enabled - so
    /// this is read-and-clear and nothing else.
    [[gnu::always_inline]] static bool take_nmi() {
        const bool f = nmi_flag();
        if (f) {
            clear_nmi_flag();
        }
        return f;
    }
};

// =============================================================================
// Pad to line: samc/device_tables.hpp is the authority
// =============================================================================
//
// `eic_extint_line(port, pin)`, `eic_nmi_pad(port, pin)` and the
// `extint_exists<L, N>` probe live in the reserve (device_tables.hpp),
// generated symbol by symbol from the device header's own
// `PIN_P<pad>A_EIC_EXTINT_NUM` / `PIN_PA08A_EIC_NMI` constants - see
// that file for why they are probes and not per-variant tables.

/**
 * One EIC line reached through the pad that carries it.
 *
 *   using Button = brio::Pin<'B', 22>;
 *   using ButtonInt = brio::ExtInt<Button>;      // EXTINT6, from the header
 *   ButtonInt::claim(brio::PinPull::up);
 *   Eic::configure_line(ButtonInt::line, {.sense = EicSense::falling});
 *
 * A pad the device does not bond to the EIC fails to compile here, and
 * that is the per-package gate: no `#if defined(PORTB)` ladder, no
 * hand-kept table, just the absence of the header's own symbol.
 *
 * ONE LINE, SEVERAL PADS, and the driver cannot police it: up to four
 * pads share each EXTINT number, and 26.6.6 note 2 says that if two of
 * them are muxed to the EIC at once "only one will be active (the first
 * one programmed)". Which pad won is not readable anywhere.
 */
template <class P>
struct ExtInt {
    ExtInt() = delete;

    static_assert(extint_exists<P::port_letter, P::pin_number>,
                  "this pad has no EIC external interrupt line on this device "
                  "(the device header defines no PIN_P<pad>A_EIC_EXTINT_NUM "
                  "for it)");

    using pin = P;
    static constexpr uint8_t line =
        static_cast<uint8_t>(eic_extint_line(P::port_letter, P::pin_number));
    static constexpr uint32_t mask = Eic::line_mask(line);

    /// This line's EVSYS generator code (see Eic::event_generator).
    static constexpr uint8_t event_generator = Eic::event_generator(line);

    /// Hand the pad to the EIC (peripheral function A) with its input
    /// buffer on - the mux does not turn the buffer on, and without it
    /// the line sees nothing.
    static void claim(PinPull pull = PinPull::none) {
        P::function(PinFunction::a, PinConfig{.input_enable = true, .pull = pull});
    }

    /// Give the pad back to PORT (PMUXEN cleared, PINCFG left alone).
    static void release() { P::release(); }

    static bool flag() { return Eic::flag(line); }
    static void clear_flag() { Eic::clear_flag(line); }
    static void arm(bool on) {
        if (on) {
            Eic::arm(mask);
        } else {
            Eic::disarm(mask);
        }
    }
    static bool armed() { return (Eic::armed() & mask) != 0u; }

    static bool configure(const EicLineConfig& cfg) {
        return Eic::configure_line(line, cfg);
    }
};

/**
 * The NMI, reached through its pad - the same shape as ExtInt<> for a
 * line that has no number, no INTENSET bit and no EVSYS generator.
 *
 *   using Nmi = brio::ExtNmi<brio::Pin<'A', 8>>;
 *   Nmi::claim(brio::PinPull::up);
 *   brio::Eic::nmi_configure({.sense = brio::EicSense::falling,
 *                             .asynchronous = true});
 *
 * There is no arming: an NMI is always enabled, is taken at any
 * priority, and cannot be masked by PRIMASK. A LEVEL sense on a pad
 * held at that level is therefore an unbreakable loop - which is why
 * the only safe way to explore this on a board whose pads are not all
 * known is an EDGE sense.
 */
template <class P>
struct ExtNmi {
    ExtNmi() = delete;

    static_assert(eic_nmi_pad(P::port_letter, P::pin_number),
                  "this pad is not the EIC NMI pad on this device (the device "
                  "header defines no PIN_P<pad>A_EIC_NMI for it)");

    using pin = P;

    static void claim(PinPull pull = PinPull::none) {
        P::function(PinFunction::a, PinConfig{.input_enable = true, .pull = pull});
    }
    static void release() { P::release(); }

    static bool flag() { return Eic::nmi_flag(); }
    static void clear_flag() { Eic::clear_nmi_flag(); }
    static bool configure(const EicNmiConfig& cfg) { return Eic::nmi_configure(cfg); }
};

} // namespace brio
