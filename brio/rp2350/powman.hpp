/*
 * powman.hpp
 *
 * THE ALWAYS-ON BLOCK (datasheet 6.2 the power domains and their states,
 * 6.3 the core voltage regulator, 6.4 the register list, 12.10 the
 * always-on timer, 3.5.10 what a debugger can do to the sequencers, 7
 * what a wake from a powered-down core is) - the one peripheral of this
 * chip that keeps running when the processors, the bus fabric and every
 * other peripheral have no supply at all.
 *
 * FIVE DOMAINS AND ONE SEQUENCER. 6.2.1 splits the digital core into
 * AON (this block and the timer in it), SWCORE (the processors, the
 * fabric, every peripheral), XIP (the cache and the boot RAM), SRAM0
 * (banks 0..3) and SRAM1 (banks 4..9). A power state is named Pc.m: c is
 * the switched core, m the three memory domains in the order XIP, SRAM0,
 * SRAM1. A P0.m state is a normal one - software runs; a P1.m state has
 * no processor in it at all, and LEAVING ONE IS A BOOT. `PowerState`
 * below is the datasheet's twelve states as the four-bit code STATE.REQ
 * and STATE.CURRENT carry (a ONE means powered DOWN), and
 * `power_transition_legal` is 6.2.3's table as arithmetic.
 *
 * THE PASSWORD. Every register of this block up to offset 0xac takes a
 * write only with 0x5AFE in its top sixteen bits; a write without it is
 * dropped and sets BADPASSWD (6.4). A READ of a protected register does
 * NOT return the password, so an ordinary read-modify-write would write
 * a zero password back - which is why the store in this file happens in
 * exactly ONE private verb and nothing else ever writes POWMAN. The
 * atomic aliases of 2.1.3 are unusable here for the same reason: they
 * carry bits, not a password. Only the scratch registers, the four boot
 * vector words and the three interrupt registers are unprotected.
 *
 * WHAT THIS FILE WILL NOT WRITE, AND WHY. The core voltage regulator
 * (6.3) and the brown-out detector are decoded here and NEVER written: a
 * wrong VSEL is a voltage on the digital core, and VREG_CTRL.UNLOCK
 * cannot be locked again once it is set. The chapter's own warning is
 * "avoid accidental writes to the VREG register", and a framework that
 * offers a verb for it offers the accident. What a program gets is
 * `VregStatus` and `BodStatus`, read-only, in millivolts - and the
 * sequencer's automatic switch between the regulator's normal and
 * low-power modes, which is SEQ_CFG's business and takes no voltage from
 * anyone.
 *
 * THE ALWAYS-ON TIMER (12.10) IS PART OF THIS BLOCK, not a peripheral
 * beside it: it shares the register file, the password and the domain.
 * It counts MILLISECONDS to 64 bits off whichever source TIMER names -
 * the low-power oscillator (the only one that survives a P1 state), the
 * crystal through clk_ref, or a GPIO's tick - and its one alarm is both
 * an interrupt and a POWER-UP REQUEST. `AonTimer` is that counter;
 * `AonTimer::calibrate()` is what answers the low-power oscillator's own
 * inaccuracy, because the divider is told what rate to assume and this
 * die's oscillator is not at its nominal one.
 *
 * A DEBUGGER CAN BLOCK A POWER-DOWN. The SW-DP's CSYSPWRUPREQ powers
 * every domain and inhibits software's own transitions (6.2.3.3): a
 * probe that has attached and detached is likely to leave it asserted,
 * and then a request to power the core down is answered with
 * REQ_IGNORED and nothing happens. `debug_powerup_pending()` is how a
 * program sees that condition, and `ignore_debug_powerup(true)` -
 * DBG_PWRCFG.IGNORE - is the one way past it from this side of the
 * wire. A suite that sleeps says which of the two it ran under.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "rp2350/device.hpp"

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"

namespace brio {

// ---- the power states (6.2.2) -----------------------------------------------

/// The four domains STATE's nibble names, as the bit each owns. A SET
/// bit is a domain that is powered DOWN - the register's own polarity,
/// kept rather than inverted so that a printed code reads as the
/// datasheet's.
struct PowerDomainBit {
    static constexpr uint8_t swcore = 1u << 3;
    static constexpr uint8_t xip = 1u << 2;
    static constexpr uint8_t sram0 = 1u << 1;
    static constexpr uint8_t sram1 = 1u << 0;
};

/**
 * The twelve states of table 475, spelled Pc.m as the datasheet spells
 * them: `p0_0` is everything on, `p1_7` is the switched core, the XIP
 * cache and both SRAM domains off. The value IS the four-bit field.
 */
enum class PowerState : uint8_t {
    p0_0 = 0x0,   ///< normal operation
    p0_1 = 0x1,   ///< normal, SRAM1 off
    p0_2 = 0x2,   ///< normal, SRAM0 off
    p0_3 = 0x3,   ///< normal, both SRAM domains off
    p1_0 = 0x8,   ///< low power, every memory retained
    p1_1 = 0x9,   ///< low power, SRAM1 off
    p1_2 = 0xA,   ///< low power, SRAM0 off
    p1_3 = 0xB,   ///< low power, both SRAM domains off
    p1_4 = 0xC,   ///< low power, XIP off
    p1_5 = 0xD,   ///< low power, XIP and SRAM1 off
    p1_6 = 0xE,   ///< low power, XIP and SRAM0 off
    p1_7 = 0xF,   ///< low power, nothing but the always-on domain
};

constexpr uint8_t state_code(PowerState s) { return static_cast<uint8_t>(s); }
/// Is the switched core powered off in this state - is it a P1.m?
constexpr bool is_low_power_state(PowerState s) {
    return (state_code(s) & PowerDomainBit::swcore) != 0u;
}

/**
 * 6.2.3's table as arithmetic, in the STATE register's own three rules
 * (the register description in 6.4, which is what the silicon obeys):
 *
 *  - a request may not combine a power UP with a power DOWN;
 *  - a request may not leave the switched core powered with the XIP
 *    domain unpowered;
 *  - a P1.m state may only be left for a P0.m one, never for another
 *    P1.m.
 *
 * The prose of 6.2.3 states a fourth rule - that leaving a P1.m state
 * may not power a powered-off SRAM domain back on - which its OWN table
 * contradicts (P1.1 -> P0.0 is listed as valid), and which the register
 * description does not repeat. The register's three are what this
 * function enforces, and what the bench suite measures against
 * STATE.BAD_SW_REQ.
 */
constexpr bool power_transition_legal(PowerState from, PowerState to) {
    const uint8_t f = state_code(from);
    const uint8_t t = state_code(to);
    if ((t & PowerDomainBit::swcore) == 0u && (t & PowerDomainBit::xip) != 0u) {
        return false;
    }
    if ((f & PowerDomainBit::swcore) != 0u && (t & PowerDomainBit::swcore) != 0u) {
        return false;
    }
    const uint8_t going_down = static_cast<uint8_t>(t & ~f);
    const uint8_t going_up = static_cast<uint8_t>(f & ~t);
    return going_down == 0u || going_up == 0u;
}

/// What became of a request to change the power state. Only `accepted`
/// means the sequencer took it - and for a request that powers the
/// switched core down, `accepted` is a value nothing reads, because the
/// way back from that state is a boot.
enum class PowerRequest : uint8_t {
    accepted,   ///< the sequencer took it
    illegal,    ///< not one of 6.2.3's transitions - refused before the store
    no_wake,    ///< no armed way back: refused by this driver and not by the silicon
    busy,       ///< a transition was already CHANGING, and writes are ignored then
    ignored,    ///< STATE.REQ_IGNORED: a debugger's or a hardware power-up request stood
    bad_request,///< STATE.BAD_SW_REQ: the silicon judged it invalid after all
};

/// What woke the switched core last, and what is asking for it now
/// (CURRENT_PWRUP_REQ and LAST_SWCORE_PWRUP share this numbering).
enum class PowerUpSource : uint8_t {
    chip_reset = 0,   ///< the source is in CHIP_RESET (rp2350/reset.hpp)
    pwrup0 = 1,
    pwrup1 = 2,
    pwrup2 = 3,
    pwrup3 = 4,
    coresight = 5,    ///< the debug port's CSYSPWRUPREQ
    alarm = 6,        ///< the always-on timer's alarm
};

/// The bit each source owns in CURRENT_PWRUP_REQ, which is a SET of
/// standing requests where LAST_SWCORE_PWRUP is one number.
constexpr uint32_t pwrup_bit(PowerUpSource s) { return 1UL << static_cast<uint8_t>(s); }

// ---- the regulator and the brown-out detector, READ-ONLY (6.3) ---------------

/// VSEL's thirty-two codes as millivolts (the VREG register's table).
/// The steps are 50 mV to 1.40 V, then irregular - a table and not a
/// formula, because the silicon's is not one either.
constexpr uint16_t vreg_vsel_mv(uint8_t code) {
    constexpr uint16_t mv[32] = {550,  600,  650,  700,  750,  800,  850,  900,
                                 950,  1000, 1050, 1100, 1150, 1200, 1250, 1300,
                                 1350, 1400, 1500, 1600, 1650, 1700, 1800, 1900,
                                 2000, 2350, 2500, 2650, 2800, 3000, 3150, 3300};
    return mv[code & 0x1Fu];
}

/// BOD's eighteen thresholds as millivolts (the BOD register's table).
/// The step is 43 mV exactly; codes above 17 are not in the table and
/// answer 0.
constexpr uint16_t bod_vsel_mv(uint8_t code) {
    const uint8_t c = static_cast<uint8_t>(code & 0x1Fu);
    if (c > 17u) {
        return 0u;
    }
    return static_cast<uint16_t>(473u + 43u * c);
}

/// The regulator as a value: what it is set to and what it reports.
/// Every field is a READ - see the file header.
struct VregStatus {
    uint8_t vsel = 0;             ///< the VSEL code
    uint16_t mv = 0;              ///< vreg_vsel_mv(vsel)
    bool high_impedance = false;  ///< VREG.HIZ
    bool updating = false;        ///< VREG.UPDATE_IN_PROGRESS
    bool vout_ok = false;         ///< VREG_STS.VOUT_OK - the output in regulation
    bool starting_up = false;     ///< VREG_STS.STARTUP
    bool unlocked = false;        ///< VREG_CTRL.UNLOCK, which cannot be undone
    bool voltage_limit_off = false;  ///< VREG_CTRL.DISABLE_VOLTAGE_LIMIT
    bool isolated = false;        ///< VREG_CTRL.ISOLATE
    uint8_t high_temp_code = 0;   ///< VREG_CTRL.HT_TH
    /// The junction temperature the regulator gives up at, in degrees
    /// Celsius (6.3's table under HT_TH).
    uint16_t high_temp_c = 0;
};

/// One of the two low-power settings the SEQUENCER applies for the
/// regulator, on the way into a P1.m state and on the way out.
struct VregLowPower {
    uint8_t vsel = 0;
    uint16_t mv = 0;
    bool linear = false;   ///< MODE: the low-power linear mode, 1 mA at most
    bool high_impedance = false;
};

/// The brown-out detector as a value, read-only for the regulator's
/// reason: its threshold is a supply level, not a setting.
struct BodStatus {
    bool enabled = false;
    uint8_t vsel = 0;
    uint16_t mv = 0;
    bool isolated = false;        ///< BOD_CTRL.ISOLATE
    bool lp_entry_enabled = false;
    uint16_t lp_entry_mv = 0;
    bool lp_exit_enabled = false;
    uint16_t lp_exit_mv = 0;
};

/// SEQ_CFG's live half: what the sequencer is doing, and what it will do
/// at the next transition.
struct SequencerConfig {
    bool using_fast_powck = false;  ///< the block's clock is clk_ref, not the low-power oscillator
    bool using_bod_lp = false;
    bool using_vreg_lp = false;
    bool use_fast_powck = false;
    bool run_lposc_in_lp = false;   ///< the oscillator kept running while the core is down
    bool use_bod_hp = false;
    bool use_bod_lp = false;
    bool use_vreg_hp = false;
    bool use_vreg_lp = false;
    bool hw_pwrup_sram0 = false;    ///< 1 = leave SRAM0 as it is on the way back up
    bool hw_pwrup_sram1 = false;
};

// ---- the four GPIO power-up sources (6.4, PWRUP0..3) -------------------------

/// A power-up source watches a pad for a LEVEL or for an EDGE.
enum class PwrupMode : uint8_t {
    level = POWMAN_PWRUP0_MODE_VALUE_LEVEL,
    edge = POWMAN_PWRUP0_MODE_VALUE_EDGE,
};

/// And it watches it in one direction: low / falling, or high / rising.
enum class PwrupDirection : uint8_t {
    low_falling = 0,
    high_rising = 1,
};

/// What a power-up source is set to. `source` is the pad number in the
/// register's own numbering: 0..47 the GPIO of the larger package,
/// 48..53 the six QSPI pads, 63 (the reset value) none.
struct PwrupConfig {
    uint8_t source = 0x3Fu;
    PwrupMode mode = PwrupMode::level;
    PwrupDirection direction = PwrupDirection::low_falling;
};

/// The QSPI pads as power-up sources, by the numbers 6.4 gives them.
struct PwrupQspiSource {
    static constexpr uint8_t ss = 48;
    static constexpr uint8_t sd0 = 49;
    static constexpr uint8_t sd1 = 50;
    static constexpr uint8_t sd2 = 51;
    static constexpr uint8_t sd3 = 52;
    static constexpr uint8_t sclk = 53;
};

/// Is this a source number the silicon accepts at all, and does THIS
/// package bond it? An invalid selection is ignored by the hardware in
/// silence, which is exactly the failure a sleep must not have.
constexpr bool pwrup_source_valid(uint8_t source) {
    if (source <= 53u && source >= 48u) {
        return true;
    }
    return source < gpio_count;
}

// ---- the block ---------------------------------------------------------------

/**
 * POWMAN: the power manager, the always-on domain's one peripheral.
 *
 * Everything here is a monostate - there is one of this block - and
 * every write goes through `write_protected`, which is the only place in
 * brio that spells the password.
 */
struct Powman {
    Powman() = delete;

    /// The two interrupt lines this block raises: POWMAN_IRQ_POW for the
    /// state machine's complaints and the regulator's, POWMAN_IRQ_TIMER
    /// for the always-on timer's alarm. An app binds `isr_powman_pow`
    /// and `isr_powman_timer` - the same two names on both
    /// architectures.
    static constexpr IRQn_Type irq_pow() { return POWMAN_IRQ_POW_IRQn; }
    static constexpr IRQn_Type irq_timer() { return POWMAN_IRQ_TIMER_IRQn; }

    // ---- the password itself ------------------------------------------------

    /// A write that did not carry the password - the flag that proves a
    /// password is a password. Write-to-clear, and the clear needs the
    /// password too.
    static bool bad_password() { return (POWMAN->BADPASSWD & POWMAN_BADPASSWD_BITS) != 0u; }
    static void clear_bad_password() { write_protected(POWMAN->BADPASSWD, POWMAN_BADPASSWD_BITS); }

    /// THE ONE WRITE IN THIS FILE THAT DOES NOT CARRY THE PASSWORD, and
    /// the only register it may be aimed at: a store of BADPASSWD's own
    /// bit with no password in it. If the password is a password the
    /// write is dropped and the flag RISES; if it were not, the write
    /// would CLEAR the flag instead. Either way nothing else in the
    /// block moves, which is what makes this the one place the question
    /// can be asked without staking anything on the answer - and a
    /// document that says "a write without the password is ignored"
    /// needs the measurement behind it.
    static void probe_password() { POWMAN->BADPASSWD = POWMAN_BADPASSWD_BITS; }

    // ---- the power state (6.2) ----------------------------------------------

    /// STATE as one word - the current state, the requested one and the
    /// five flags together, for a diagnostic line that wants them all.
    static uint32_t state_word() { return POWMAN->STATE & POWMAN_STATE_BITS; }

    /// What the four domains are doing now (STATE.CURRENT).
    static PowerState current_state() {
        return static_cast<PowerState>(POWMAN->STATE & POWMAN_STATE_CURRENT_BITS);
    }
    /// The state last requested (STATE.REQ), which is not what is
    /// current until the sequencer has run.
    static PowerState requested_state() {
        return static_cast<PowerState>((POWMAN->STATE & POWMAN_STATE_REQ_BITS) >>
                                       POWMAN_STATE_REQ_LSB);
    }
    /// A transition has started and POWMAN ignores writes until it ends.
    static bool changing() { return (POWMAN->STATE & POWMAN_STATE_CHANGING_BITS) != 0u; }
    /// A request is taken and the sequencer is waiting for the
    /// processors to halt (or for whatever else must finish first).
    static bool waiting() { return (POWMAN->STATE & POWMAN_STATE_WAITING_BITS) != 0u; }
    /// The silicon's own verdict on the last software request.
    static bool bad_software_request() {
        return (POWMAN->STATE & POWMAN_STATE_BAD_SW_REQ_BITS) != 0u;
    }
    static bool bad_hardware_request() {
        return (POWMAN->STATE & POWMAN_STATE_BAD_HW_REQ_BITS) != 0u;
    }
    /// A power-up arrived while a power-down request was waiting: the
    /// request was dropped and the core stayed up. Write-to-clear.
    static bool powerup_while_waiting() {
        return (POWMAN->STATE & POWMAN_STATE_PWRUP_WHILE_WAITING_BITS) != 0u;
    }
    /// A software request clashed with a standing hardware or debugger
    /// one. Write-to-clear, and the first thing to read after a
    /// power-down that did not happen.
    static bool request_ignored() {
        return (POWMAN->STATE & POWMAN_STATE_REQ_IGNORED_BITS) != 0u;
    }
    /// Both write-to-clear flags of STATE, cleared together.
    ///
    /// AND THE CLEAR IS ITSELF A REQUEST, because the two flags live in
    /// STATE beside REQ and acknowledging one means storing that
    /// register. 6.2.3.1's first rule is that a write to REQ with a
    /// power-up request pending sets REQ_IGNORED and takes no further
    /// action - so while a debugger's CSYSPWRUPREQ stands this verb
    /// leaves REQ_IGNORED exactly where it found it (measured), and
    /// `ignore_debug_powerup(true)` is what makes it clearable.
    static void clear_request_flags() {
        write_state(0u, POWMAN_STATE_PWRUP_WHILE_WAITING_BITS | POWMAN_STATE_REQ_IGNORED_BITS);
    }

    /**
     * Ask for a state that keeps the switched core POWERED - a move
     * between P0.m states, which is a memory domain going down or coming
     * back up. It starts at once and the caller waits for CHANGING to
     * fall.
     *
     * A powered-down SRAM domain LOSES ITS CONTENTS (6.5.5), so this is
     * a verb about memory a program has stopped using, never about
     * memory it means to find again.
     */
    static PowerRequest request_state(PowerState to) {
        if (is_low_power_state(to)) {
            return PowerRequest::illegal;
        }
        return submit(to);
    }

    /**
     * THE ONE VERB THAT DOES NOT RETURN WHEN IT WORKS: power the
     * switched core down into `to`, one of the eight P1.m states.
     *
     * What comes back is a BOOT - the bootrom runs, the crt runs, main()
     * runs - so nothing of this program's state crosses but what lives
     * in this block (the eight scratch words, the four boot words, the
     * timer) and in whatever SRAM domain `to` left powered. The
     * boot-side report is `last_powerup_source()` beside
     * `Reset::causes()`'s `swcore_powerdown`.
     *
     * A WAY BACK IS MANDATORY AND CHECKED HERE: with no GPIO power-up
     * enabled and no alarm armed to power up on, this returns
     * `no_wake` and writes nothing. The silicon would take the request
     * and the board would be gone until its supply is cycled.
     *
     * The sequencer holds STATE.WAITING until the processors halt, so
     * the caller must sleep: with interrupts masked and every queue
     * empty, `wait_for_interrupt()` in a loop is the shape, and the loop
     * matters because a wake that is not the one asked for leaves the
     * core running with REQ still standing.
     */
    static PowerRequest power_down(PowerState to) {
        if (!is_low_power_state(to)) {
            return PowerRequest::illegal;
        }
        if (!powerup_armed()) {
            return PowerRequest::no_wake;
        }
        return submit(to);
    }

    /// Is there an armed way back from a P1.m state: one of the four
    /// GPIO power-up sources enabled, or the timer's alarm enabled with
    /// PWRUP_ON_ALARM set?
    static bool powerup_armed();

    /// Wait for a transition that keeps the core powered to finish.
    /// False when CHANGING still stands after the bounded wait.
    static bool wait_settled() {
        for (uint32_t spins = 4'000'000u; spins != 0u; --spins) {
            if (!changing()) {
                return true;
            }
        }
        return false;
    }

    // ---- the power-up sources (6.4, PWRUP0..3) -------------------------------

    /// One of the four GPIO power-up slots, set and enabled. False when
    /// the source number is not one this package bonds - the hardware
    /// would ignore it in silence.
    static bool pwrup(uint8_t slot, const PwrupConfig& cfg) {
        if (slot > 3u || !pwrup_source_valid(cfg.source)) {
            return false;
        }
        const uint32_t word =
            (static_cast<uint32_t>(cfg.source) & POWMAN_PWRUP0_SOURCE_BITS) |
            (static_cast<uint32_t>(cfg.mode) << POWMAN_PWRUP0_MODE_LSB) |
            (static_cast<uint32_t>(cfg.direction) << POWMAN_PWRUP0_DIRECTION_LSB);
        // THREE STORES AND NOT ONE. The pad, the sense and the enable
        // are written apart: a slot enabled in the same word that names
        // its pad would latch whatever the multiplexer was passing
        // while it settled. Then the latched event is cleared, and only
        // then is the slot enabled - so the first event it reports is
        // one the pad really made.
        write_protected(pwrup_reg(slot), word);
        write_protected(pwrup_reg(slot), word | POWMAN_PWRUP0_STATUS_BITS);
        write_protected(pwrup_reg(slot), word | POWMAN_PWRUP0_ENABLE_BITS);
        return true;
    }
    /// Disable a slot, which also clears a power-up event it had
    /// latched.
    static void pwrup_disable(uint8_t slot) {
        if (slot <= 3u) {
            write_protected(pwrup_reg(slot), POWMAN_PWRUP0_SOURCE_BITS | POWMAN_PWRUP0_STATUS_BITS);
        }
    }
    static bool pwrup_enabled(uint8_t slot) {
        return slot <= 3u && (pwrup_reg(slot) & POWMAN_PWRUP0_ENABLE_BITS) != 0u;
    }
    static uint8_t pwrup_source(uint8_t slot) {
        return slot <= 3u ? static_cast<uint8_t>(pwrup_reg(slot) & POWMAN_PWRUP0_SOURCE_BITS) : 0x3Fu;
    }
    /// The level of the watched pad, which the block reports only while
    /// the slot is enabled.
    static bool pwrup_raw(uint8_t slot) {
        return slot <= 3u && (pwrup_reg(slot) & POWMAN_PWRUP0_RAW_STATUS_BITS) != 0u;
    }
    /// The slot's own event flag: a latched edge, or a level that
    /// stands. Write-to-clear (an edge; a level goes when the pad does).
    static bool pwrup_status(uint8_t slot) {
        return slot <= 3u && (pwrup_reg(slot) & POWMAN_PWRUP0_STATUS_BITS) != 0u;
    }
    static void pwrup_clear(uint8_t slot) {
        if (slot <= 3u) {
            const uint32_t keep = pwrup_reg(slot) & (POWMAN_PWRUP0_SOURCE_BITS |
                                                     POWMAN_PWRUP0_ENABLE_BITS |
                                                     POWMAN_PWRUP0_MODE_BITS |
                                                     POWMAN_PWRUP0_DIRECTION_BITS);
            write_protected(pwrup_reg(slot), keep | POWMAN_PWRUP0_STATUS_BITS);
        }
    }
    /// Every slot disabled: what a program does before it arms the one
    /// it means, and what a suite does between letters.
    static void pwrup_disable_all() {
        for (uint8_t s = 0; s < 4u; ++s) {
            pwrup_disable(s);
        }
    }

    /// The set of power-up requests standing right now, one bit per
    /// `PowerUpSource` (CURRENT_PWRUP_REQ).
    static uint32_t current_powerup_requests() {
        return POWMAN->CURRENT_PWRUP_REQ & POWMAN_CURRENT_PWRUP_REQ_BITS;
    }
    /// What powered the switched core up last (LAST_SWCORE_PWRUP): the
    /// boot-side answer to "why am I running". A zero means a chip
    /// reset, whose own source is CHIP_RESET (rp2350/reset.hpp).
    static uint32_t last_powerup_requests() {
        return POWMAN->LAST_SWCORE_PWRUP & POWMAN_LAST_SWCORE_PWRUP_BITS;
    }
    /// The same as one source, the lowest bit standing - which is what
    /// the field is in every case but a tie.
    static PowerUpSource last_powerup_source() {
        const uint32_t w = last_powerup_requests();
        for (uint8_t i = 1; i <= 6u; ++i) {
            if ((w & (1UL << i)) != 0u) {
                return static_cast<PowerUpSource>(i);
            }
        }
        return PowerUpSource::chip_reset;
    }

    // ---- the debugger's standing power-up request (6.2.3.3, 3.5.10) ----------

    /// Is the debug port asking for every domain to be powered? A probe
    /// that has attached is very likely to leave CSYSPWRUPREQ asserted
    /// when it detaches, and while it stands a power-down request is
    /// answered with REQ_IGNORED.
    static bool debug_powerup_pending() {
        return (current_powerup_requests() & pwrup_bit(PowerUpSource::coresight)) != 0u;
    }
    /// DBG_PWRCFG.IGNORE: stop paying attention to the debugger's
    /// request. The chapter's own note is that without this it would be
    /// impossible to sleep after a probe has been attached once.
    static void ignore_debug_powerup(bool on) {
        write_protected(POWMAN->DBG_PWRCFG, on ? POWMAN_DBG_PWRCFG_IGNORE_BITS : 0u);
    }
    static bool ignoring_debug_powerup() {
        return (POWMAN->DBG_PWRCFG & POWMAN_DBG_PWRCFG_IGNORE_BITS) != 0u;
    }

    // ---- the sequencer (SEQ_CFG) --------------------------------------------

    static SequencerConfig sequencer() {
        const uint32_t w = POWMAN->SEQ_CFG;
        SequencerConfig c{};
        c.using_fast_powck = (w & POWMAN_SEQ_CFG_USING_FAST_POWCK_BITS) != 0u;
        c.using_bod_lp = (w & POWMAN_SEQ_CFG_USING_BOD_LP_BITS) != 0u;
        c.using_vreg_lp = (w & POWMAN_SEQ_CFG_USING_VREG_LP_BITS) != 0u;
        c.use_fast_powck = (w & POWMAN_SEQ_CFG_USE_FAST_POWCK_BITS) != 0u;
        c.run_lposc_in_lp = (w & POWMAN_SEQ_CFG_RUN_LPOSC_IN_LP_BITS) != 0u;
        c.use_bod_hp = (w & POWMAN_SEQ_CFG_USE_BOD_HP_BITS) != 0u;
        c.use_bod_lp = (w & POWMAN_SEQ_CFG_USE_BOD_LP_BITS) != 0u;
        c.use_vreg_hp = (w & POWMAN_SEQ_CFG_USE_VREG_HP_BITS) != 0u;
        c.use_vreg_lp = (w & POWMAN_SEQ_CFG_USE_VREG_LP_BITS) != 0u;
        c.hw_pwrup_sram0 = (w & POWMAN_SEQ_CFG_HW_PWRUP_SRAM0_BITS) != 0u;
        c.hw_pwrup_sram1 = (w & POWMAN_SEQ_CFG_HW_PWRUP_SRAM1_BITS) != 0u;
        return c;
    }

    /// Which SRAM domains the sequencer powers back up when the
    /// switched core comes up from a P1.m state: false = power it up
    /// (the reset value), true = leave it as the low-power state had it.
    /// Refused while a transition is in flight, SEQ_CFG's writes being
    /// ignored then.
    static bool sram_stays_down_on_powerup(bool sram0, bool sram1) {
        if (changing()) {
            return false;
        }
        const uint32_t mask =
            POWMAN_SEQ_CFG_HW_PWRUP_SRAM0_BITS | POWMAN_SEQ_CFG_HW_PWRUP_SRAM1_BITS;
        const uint32_t value = (sram0 ? POWMAN_SEQ_CFG_HW_PWRUP_SRAM0_BITS : 0u) |
                               (sram1 ? POWMAN_SEQ_CFG_HW_PWRUP_SRAM1_BITS : 0u);
        write_protected(POWMAN->SEQ_CFG, (POWMAN->SEQ_CFG & seq_cfg_rw & ~mask) | value);
        return true;
    }

    /**
     * WHICH CLOCK THIS BLOCK ITSELF RUNS ON (SEQ_CFG.USE_FAST_POWCK, set
     * at reset): true = the reference clock while the switched core is
     * powered, false = the low-power oscillator always.
     *
     * It matters to exactly one thing, and that thing is a DORMANT: the
     * power-down sequencer moves this block onto the low-power
     * oscillator by itself when the switched core goes, but a dormant is
     * not a power-down and gets no such help - so a dormant that stops
     * the crystal stops clk_ref, and with it everything in this block
     * that clk_ref was clocking. 6.4's own note is that the setting
     * "takes effect when a power up sequence is next run", which is a
     * claim the bench measures: `sequencer().using_fast_powck` is what
     * the block reports it is actually doing.
     */
    static bool use_fast_clock(bool on) {
        if (changing()) {
            return false;
        }
        const uint32_t v = POWMAN->SEQ_CFG & seq_cfg_rw & ~POWMAN_SEQ_CFG_USE_FAST_POWCK_BITS;
        write_protected(POWMAN->SEQ_CFG, v | (on ? POWMAN_SEQ_CFG_USE_FAST_POWCK_BITS : 0u));
        return true;
    }

    /// Keep the low-power oscillator running while the switched core is
    /// down (SEQ_CFG.RUN_LPOSC_IN_LP, set at reset). Clearing it is
    /// unwise while the always-on timer counts that oscillator, which is
    /// the only configuration that can wake a P1.m state on time.
    static bool run_lposc_in_low_power(bool on) {
        if (changing()) {
            return false;
        }
        const uint32_t v = POWMAN->SEQ_CFG & seq_cfg_rw & ~POWMAN_SEQ_CFG_RUN_LPOSC_IN_LP_BITS;
        write_protected(POWMAN->SEQ_CFG, v | (on ? POWMAN_SEQ_CFG_RUN_LPOSC_IN_LP_BITS : 0u));
        return true;
    }

    // ---- the regulator and the detector, read-only (6.3) --------------------

    static VregStatus vreg() {
        const uint32_t v = POWMAN->VREG;
        const uint32_t ctrl = POWMAN->VREG_CTRL;
        const uint32_t sts = POWMAN->VREG_STS;
        VregStatus s{};
        s.vsel = static_cast<uint8_t>((v & POWMAN_VREG_VSEL_BITS) >> POWMAN_VREG_VSEL_LSB);
        s.mv = vreg_vsel_mv(s.vsel);
        s.high_impedance = (v & POWMAN_VREG_HIZ_BITS) != 0u;
        s.updating = (v & POWMAN_VREG_UPDATE_IN_PROGRESS_BITS) != 0u;
        s.vout_ok = (sts & POWMAN_VREG_STS_VOUT_OK_BITS) != 0u;
        s.starting_up = (sts & POWMAN_VREG_STS_STARTUP_BITS) != 0u;
        s.unlocked = (ctrl & POWMAN_VREG_CTRL_UNLOCK_BITS) != 0u;
        s.voltage_limit_off = (ctrl & POWMAN_VREG_CTRL_DISABLE_VOLTAGE_LIMIT_BITS) != 0u;
        s.isolated = (ctrl & POWMAN_VREG_CTRL_ISOLATE_BITS) != 0u;
        s.high_temp_code =
            static_cast<uint8_t>((ctrl & POWMAN_VREG_CTRL_HT_TH_BITS) >> POWMAN_VREG_CTRL_HT_TH_LSB);
        s.high_temp_c = high_temp_threshold_c(s.high_temp_code);
        return s;
    }

    /// HT_TH's eight codes as degrees Celsius (6.3.6).
    static constexpr uint16_t high_temp_threshold_c(uint8_t code) {
        constexpr uint16_t table[8] = {100, 105, 110, 115, 120, 125, 135, 150};
        return table[code & 7u];
    }

    static VregLowPower vreg_low_power_entry() { return decode_vreg_lp(POWMAN->VREG_LP_ENTRY); }
    static VregLowPower vreg_low_power_exit() { return decode_vreg_lp(POWMAN->VREG_LP_EXIT); }

    static BodStatus bod() {
        const uint32_t b = POWMAN->BOD;
        BodStatus s{};
        s.enabled = (b & POWMAN_BOD_EN_BITS) != 0u;
        s.vsel = static_cast<uint8_t>((b & POWMAN_BOD_VSEL_BITS) >> POWMAN_BOD_VSEL_LSB);
        s.mv = bod_vsel_mv(s.vsel);
        s.isolated = (POWMAN->BOD_CTRL & POWMAN_BOD_CTRL_ISOLATE_BITS) != 0u;
        const uint32_t e = POWMAN->BOD_LP_ENTRY;
        const uint32_t x = POWMAN->BOD_LP_EXIT;
        s.lp_entry_enabled = (e & POWMAN_BOD_LP_ENTRY_EN_BITS) != 0u;
        s.lp_entry_mv = bod_vsel_mv(
            static_cast<uint8_t>((e & POWMAN_BOD_LP_ENTRY_VSEL_BITS) >> POWMAN_BOD_LP_ENTRY_VSEL_LSB));
        s.lp_exit_enabled = (x & POWMAN_BOD_LP_EXIT_EN_BITS) != 0u;
        s.lp_exit_mv = bod_vsel_mv(
            static_cast<uint8_t>((x & POWMAN_BOD_LP_EXIT_VSEL_BITS) >> POWMAN_BOD_LP_EXIT_VSEL_LSB));
        return s;
    }

    // ---- the scratch registers (6.4) ----------------------------------------

    /// Eight words that survive a power-down of the switched core - the
    /// watchdog's four do not, living in that domain. UNPROTECTED: no
    /// password, a plain 32-bit store.
    static uint32_t scratch(uint8_t n) { return n < 8u ? (&POWMAN->SCRATCH0)[n] : 0u; }
    static void scratch(uint8_t n, uint32_t value) {
        if (n < 8u) {
            (&POWMAN->SCRATCH0)[n] = value;
        }
    }

    /// The four words the BOOTROM reads as a vector after a power-up
    /// (5.2.3): BOOT0 and BOOT1 are magic numbers it checks, BOOT2 the
    /// entry point, BOOT3 a parameter. brio writes none of them - a
    /// program that wants the bootrom to jump somewhere else asks for
    /// it deliberately - and reads them so that a suite can say they are
    /// clear.
    static uint32_t boot_word(uint8_t n) { return n < 4u ? (&POWMAN->BOOT0)[n] : 0u; }

    // ---- the interrupts (6.4) -----------------------------------------------

    /// The four sources, one bit each, shared by INTR / INTE / INTF /
    /// INTS. The timer's is the alarm; the other three are the state
    /// machine's complaints and the regulator's.
    struct Interrupt {
        static constexpr uint32_t vreg_output_low = POWMAN_INTR_VREG_OUTPUT_LOW_BITS;
        static constexpr uint32_t timer = POWMAN_INTR_TIMER_BITS;
        static constexpr uint32_t state_req_ignored = POWMAN_INTR_STATE_REQ_IGNORED_BITS;
        static constexpr uint32_t pwrup_while_waiting = POWMAN_INTR_PWRUP_WHILE_WAITING_BITS;
        static constexpr uint32_t all = POWMAN_INTR_BITS;
    };

    /// The raw sources, whether or not anybody asked for them.
    static uint32_t raw_interrupts() { return POWMAN->INTR & POWMAN_INTR_BITS; }
    /// Only `vreg_output_low` is latched and write-to-clear; the other
    /// three follow their flags in STATE and TIMER and are cleared
    /// there.
    static void clear_raw(uint32_t bits) { POWMAN->INTR = bits & Interrupt::vreg_output_low; }
    static void interrupt(uint32_t bits, bool on) {
        if (on) {
            hw_set(POWMAN->INTE, bits & POWMAN_INTE_BITS);
        } else {
            hw_clear(POWMAN->INTE, bits & POWMAN_INTE_BITS);
        }
    }
    static uint32_t interrupts() { return POWMAN->INTE & POWMAN_INTE_BITS; }
    static void force(uint32_t bits, bool on) {
        if (on) {
            hw_set(POWMAN->INTF, bits & POWMAN_INTF_BITS);
        } else {
            hw_clear(POWMAN->INTF, bits & POWMAN_INTF_BITS);
        }
    }
    static uint32_t forced() { return POWMAN->INTF & POWMAN_INTF_BITS; }
    /// What reaches the two lines: the raw sources masked by INTE, with
    /// the forced ones in.
    static uint32_t pending() { return POWMAN->INTS & POWMAN_INTS_BITS; }

private:
    static constexpr uint32_t password = 0x5AFEu << 16;

    /// The ONE store into a protected register. `value` is the whole
    /// word's worth of writable bits; the password is added here and
    /// nowhere else.
    static void write_protected(volatile uint32_t& reg, uint32_t value) {
        reg = password | value;
    }

    /// STATE's writable bits, written as one word: the four-bit request
    /// and the two write-to-clear flags. CURRENT, CHANGING, WAITING and
    /// the two BAD_* flags are read-only and drop out of the store.
    static void write_state(uint8_t req, uint32_t clear_bits) {
        write_protected(POWMAN->STATE,
                        (static_cast<uint32_t>(req) << POWMAN_STATE_REQ_LSB) |
                            (clear_bits & (POWMAN_STATE_PWRUP_WHILE_WAITING_BITS |
                                           POWMAN_STATE_REQ_IGNORED_BITS)));
    }

    /// SEQ_CFG's writable half - the nine RW bits; the three USING_*
    /// bits are the sequencer's report and never written back.
    static constexpr uint32_t seq_cfg_rw =
        POWMAN_SEQ_CFG_USE_FAST_POWCK_BITS | POWMAN_SEQ_CFG_RUN_LPOSC_IN_LP_BITS |
        POWMAN_SEQ_CFG_USE_BOD_HP_BITS | POWMAN_SEQ_CFG_USE_BOD_LP_BITS |
        POWMAN_SEQ_CFG_USE_VREG_HP_BITS | POWMAN_SEQ_CFG_USE_VREG_LP_BITS |
        POWMAN_SEQ_CFG_HW_PWRUP_SRAM0_BITS | POWMAN_SEQ_CFG_HW_PWRUP_SRAM1_BITS;

    /// The request itself, with the two judgements the silicon makes
    /// after the store read back.
    static PowerRequest submit(PowerState to) {
        if (changing()) {
            return PowerRequest::busy;
        }
        if (!power_transition_legal(current_state(), to)) {
            return PowerRequest::illegal;
        }
        clear_request_flags();
        write_state(state_code(to), 0u);
        if (request_ignored()) {
            return PowerRequest::ignored;
        }
        if (bad_software_request()) {
            return PowerRequest::bad_request;
        }
        return PowerRequest::accepted;
    }

    static volatile uint32_t& pwrup_reg(uint8_t slot) { return (&POWMAN->PWRUP0)[slot & 3u]; }

    static VregLowPower decode_vreg_lp(uint32_t w) {
        VregLowPower s{};
        s.vsel = static_cast<uint8_t>((w & POWMAN_VREG_LP_ENTRY_VSEL_BITS) >>
                                      POWMAN_VREG_LP_ENTRY_VSEL_LSB);
        s.mv = vreg_vsel_mv(s.vsel);
        s.linear = (w & POWMAN_VREG_LP_ENTRY_MODE_BITS) != 0u;
        s.high_impedance = (w & POWMAN_VREG_LP_ENTRY_HIZ_BITS) != 0u;
        return s;
    }

    friend struct AonTimer;
};

// ---- the always-on timer (12.10) ---------------------------------------------

/// What the 1 kHz tick is derived from. The read-only USING_* flags of
/// TIMER answer this; the SET bits beside them ask for a change and
/// clear themselves when it has been made.
enum class AonSource : uint8_t {
    lposc,       ///< the low-power oscillator (or an external 32 kHz clock in its place)
    xosc,        ///< the crystal through clk_ref - stops when the switched core does
    gpio_1khz,   ///< an external 1 ms tick on one of the four pads 12.10.7 names
};

/// Which pads may carry an external clock or tick (12.10.7,
/// EXT_TIME_REF.SOURCE_SEL). Only one feature may use one at a time.
enum class AonTimeRefPin : uint8_t {
    gpio12 = 0,
    gpio20 = 1,
    gpio14 = 2,
    gpio22 = 3,
};

/**
 * The always-on timer: 64 bits of MILLISECONDS, the only counter on this
 * chip that runs in every power state.
 *
 * THE DIVIDER IS TOLD WHAT TO DIVIDE. The tick is made by a fractional
 * divider whose divisor is a NUMBER SOFTWARE WRITES - 32.768 for the
 * low-power oscillator, 12000.0 for a 12 MHz crystal - and not a
 * measurement the hardware makes. So the timer is exactly as accurate as
 * the rate it has been told, and on a die whose low-power oscillator is
 * well away from its nominal 32.768 kHz the default divisor is simply
 * wrong. `calibrate()` is the answer: the oscillator counted against the
 * crystal by the frequency counter of 8.1.4, and the result written into
 * the divisor. The crystal is the ruler because it is the only accurate
 * thing on the board.
 *
 * ONE ALARM, TWO DUTIES. TIMER.ALARM_ENAB arms a match against the
 * 64-bit alarm registers; the match raises TIMER.ALARM, which can be an
 * INTERRUPT (INTE.TIMER, the line `isr_powman_timer`) and, with
 * TIMER.PWRUP_ON_ALARM set, a POWER-UP REQUEST that brings the switched
 * core back from a P1.m state. It is also what ends a DORMANT, and for
 * that the interrupt need not be enabled at all (6.5.3.1) - though a
 * program that wants a handler to run enables it anyway.
 *
 * THE SOURCE AND THE POWER STATE ARE TIED. The crystal stops when the
 * switched core is powered down, so the power-down sequencer reverts the
 * timer to the low-power oscillator before it goes (12.10.5.3) - but a
 * DORMANT is not a power-down and gets no such help: a program that
 * stops the crystal by hand must move the timer onto the low-power
 * oscillator itself, or its alarm will never come.
 */
struct AonTimer {
    AonTimer() = delete;

    /// The line the alarm raises, bound as `isr_powman_timer`.
    static constexpr IRQn_Type irq() { return POWMAN_IRQ_TIMER_IRQn; }

    // ---- running, reading, setting (12.10.3) --------------------------------

    static bool running() { return (POWMAN->TIMER & POWMAN_TIMER_RUN_BITS) != 0u; }
    static void run(bool on) { write_timer(on ? POWMAN_TIMER_RUN_BITS : 0u, POWMAN_TIMER_RUN_BITS); }

    /**
     * The 64-bit value, read by 12.10.3's own procedure: the upper half,
     * the lower half, the upper half again, repeated while the upper
     * half moved. Safe from any context, and it takes two bus reads in
     * the case that matters.
     */
    static uint64_t now() {
        for (;;) {
            const uint32_t hi = POWMAN->READ_TIME_UPPER;
            const uint32_t lo = POWMAN->READ_TIME_LOWER;
            if (POWMAN->READ_TIME_UPPER == hi) {
                return (static_cast<uint64_t>(hi) << 32) | lo;
            }
        }
    }
    /// The low half alone - a span of up to 49 days of milliseconds,
    /// which is what every wrap-safe difference in this stratum uses.
    static uint32_t now_low() { return POWMAN->READ_TIME_LOWER; }

    /// Set the time. Only with the counter STOPPED (12.10.3), which this
    /// verb enforces rather than assumes: false when it is running.
    static bool set(uint64_t ms) {
        if (running()) {
            return false;
        }
        write_protected(POWMAN->SET_TIME_15TO0, static_cast<uint32_t>(ms & 0xFFFFu));
        write_protected(POWMAN->SET_TIME_31TO16, static_cast<uint32_t>((ms >> 16) & 0xFFFFu));
        write_protected(POWMAN->SET_TIME_47TO32, static_cast<uint32_t>((ms >> 32) & 0xFFFFu));
        write_protected(POWMAN->SET_TIME_63TO48, static_cast<uint32_t>((ms >> 48) & 0xFFFFu));
        return true;
    }

    /// Back to zero WITHOUT stopping and without touching the alarm -
    /// the self-clearing CLEAR bit, which is what makes this an interval
    /// timer as well as a clock.
    static void clear() { write_timer(POWMAN_TIMER_CLEAR_BITS, POWMAN_TIMER_CLEAR_BITS); }

    // ---- the alarm (12.10.4) ------------------------------------------------

    /// Arm the alarm at an absolute time. The enable is dropped first
    /// because the four registers must not be written under it, and
    /// raised again at the end; a stale ALARM flag is cleared on the way
    /// so the first match is news.
    static void alarm_at(uint64_t ms) {
        alarm_enable(false);
        clear_alarm();
        write_protected(POWMAN->ALARM_TIME_15TO0, static_cast<uint32_t>(ms & 0xFFFFu));
        write_protected(POWMAN->ALARM_TIME_31TO16, static_cast<uint32_t>((ms >> 16) & 0xFFFFu));
        write_protected(POWMAN->ALARM_TIME_47TO32, static_cast<uint32_t>((ms >> 32) & 0xFFFFu));
        write_protected(POWMAN->ALARM_TIME_63TO48, static_cast<uint32_t>((ms >> 48) & 0xFFFFu));
        alarm_enable(true);
    }
    /// The same, `ms` from now.
    static uint64_t alarm_in(uint32_t ms) {
        const uint64_t at = now() + ms;
        alarm_at(at);
        return at;
    }
    /// What the four registers hold - they are ordinary read-write
    /// words, so the alarm reads back as written.
    static uint64_t alarm_time() {
        return (static_cast<uint64_t>(POWMAN->ALARM_TIME_63TO48 & 0xFFFFu) << 48) |
               (static_cast<uint64_t>(POWMAN->ALARM_TIME_47TO32 & 0xFFFFu) << 32) |
               (static_cast<uint64_t>(POWMAN->ALARM_TIME_31TO16 & 0xFFFFu) << 16) |
               static_cast<uint64_t>(POWMAN->ALARM_TIME_15TO0 & 0xFFFFu);
    }

    static void alarm_enable(bool on) {
        write_timer(on ? POWMAN_TIMER_ALARM_ENAB_BITS : 0u, POWMAN_TIMER_ALARM_ENAB_BITS);
    }
    static bool alarm_enabled() { return (POWMAN->TIMER & POWMAN_TIMER_ALARM_ENAB_BITS) != 0u; }
    /// The match happened. Write-to-clear, and the clear is what lets
    /// the next one be seen.
    static bool alarm_fired() { return (POWMAN->TIMER & POWMAN_TIMER_ALARM_BITS) != 0u; }
    static void clear_alarm() { write_timer_raw(POWMAN_TIMER_ALARM_BITS); }

    /// Make the match a POWER-UP REQUEST as well as a flag: the way back
    /// from a P1.m state, and the thing `Powman::power_down` insists on.
    static void powerup_on_alarm(bool on) {
        write_timer(on ? POWMAN_TIMER_PWRUP_ON_ALARM_BITS : 0u, POWMAN_TIMER_PWRUP_ON_ALARM_BITS);
    }
    static bool powerup_on_alarm() {
        return (POWMAN->TIMER & POWMAN_TIMER_PWRUP_ON_ALARM_BITS) != 0u;
    }

    /// The alarm's interrupt at this block's own line.
    static void interrupt(bool on) { Powman::interrupt(Powman::Interrupt::timer, on); }
    static bool interrupt_enabled() {
        return (Powman::interrupts() & Powman::Interrupt::timer) != 0u;
    }
    static bool pending() { return (Powman::pending() & Powman::Interrupt::timer) != 0u; }

    /**
     * THE ISR BODY the app binds to `isr_powman_timer`: true when this
     * block's timer interrupt was standing, in which case the alarm has
     * been acknowledged and DISARMED.
     *
     * Disarming is not optional. TIMER.ALARM is raised by a COMPARISON
     * that goes on being true after the match, so an armed alarm whose
     * flag is merely cleared raises it again on the next tick and the
     * handler never ends.
     */
    [[gnu::always_inline]] static bool isr() {
        if (!pending()) {
            return false;
        }
        alarm_enable(false);
        clear_alarm();
        return true;
    }

    // ---- the tick source (12.10.5) ------------------------------------------

    static AonSource source() {
        const uint32_t w = POWMAN->TIMER;
        if ((w & POWMAN_TIMER_USING_XOSC_BITS) != 0u) {
            return AonSource::xosc;
        }
        if ((w & POWMAN_TIMER_USING_GPIO_1KHZ_BITS) != 0u) {
            return AonSource::gpio_1khz;
        }
        return AonSource::lposc;
    }
    /// Is the millisecond counter being steered by an external 1 Hz
    /// tick as well (12.10.6)?
    static bool synchronised_to_gpio_1hz() {
        return (POWMAN->TIMER & POWMAN_TIMER_USING_GPIO_1HZ_BITS) != 0u;
    }

    /// Move the tick onto the crystal. The select bit clears itself when
    /// the change has been made, which can take a whole tick; false when
    /// it had not cleared within the bounded wait, or when clk_ref is
    /// not the crystal - 12.10.5.3 makes that the caller's duty and this
    /// verb makes it a refusal.
    static bool use_xosc() {
        if (Clocks::ref_source() != static_cast<uint8_t>(RefSource::xosc)) {
            return false;
        }
        write_timer(POWMAN_TIMER_USE_XOSC_BITS, POWMAN_TIMER_USE_XOSC_BITS);
        return wait_source(AonSource::xosc);
    }
    /// And back onto the low-power oscillator - the only source that
    /// survives a power-down of the switched core, and the only one a
    /// DORMANT that stops the crystal may rely on.
    static bool use_lposc() {
        write_timer(POWMAN_TIMER_USE_LPOSC_BITS, POWMAN_TIMER_USE_LPOSC_BITS);
        return wait_source(AonSource::lposc);
    }

    /// Which pad an external clock or tick is taken from, and whether it
    /// drives the low-power clock outright (12.10.5.2). The clock select
    /// may only be written with the counter stopped; the pad select is
    /// this file's only write to EXT_TIME_REF and neither is used by any
    /// brio program yet.
    static bool time_reference(AonTimeRefPin pin, bool drive_low_power_clock) {
        if (drive_low_power_clock && running()) {
            return false;
        }
        write_protected(POWMAN->EXT_TIME_REF,
                        static_cast<uint32_t>(pin) |
                            (drive_low_power_clock ? POWMAN_EXT_TIME_REF_DRIVE_LPCK_BITS : 0u));
        return true;
    }

    // ---- the divisor, and the measurement behind it (12.10.5.1) -------------

    /// What the divider is told the low-power oscillator runs at, in
    /// kilohertz and a 16-bit fraction of one (the reset pair is 32 and
    /// 0xC49C - 32.768 kHz).
    static uint8_t lposc_khz_int() {
        return static_cast<uint8_t>(POWMAN->LPOSC_FREQ_KHZ_INT & POWMAN_LPOSC_FREQ_KHZ_INT_BITS);
    }
    static uint16_t lposc_khz_frac() {
        return static_cast<uint16_t>(POWMAN->LPOSC_FREQ_KHZ_FRAC & POWMAN_LPOSC_FREQ_KHZ_FRAC_BITS);
    }
    /// And what the crystal is said to run at (12000 and 0 at reset).
    static uint16_t xosc_khz_int() {
        return static_cast<uint16_t>(POWMAN->XOSC_FREQ_KHZ_INT & POWMAN_XOSC_FREQ_KHZ_INT_BITS);
    }
    static uint16_t xosc_khz_frac() {
        return static_cast<uint16_t>(POWMAN->XOSC_FREQ_KHZ_FRAC & POWMAN_XOSC_FREQ_KHZ_FRAC_BITS);
    }

    /// The pair the counter is CURRENTLY dividing, as hertz - what a
    /// program comparing the divisor against a measurement wants.
    static uint32_t declared_hz() {
        if (source() == AonSource::xosc) {
            return static_cast<uint32_t>(xosc_khz_int()) * 1000u +
                   (static_cast<uint32_t>(xosc_khz_frac()) * 1000u) / 65536u;
        }
        return static_cast<uint32_t>(lposc_khz_int()) * 1000u +
               (static_cast<uint32_t>(lposc_khz_frac()) * 1000u) / 65536u;
    }

    /// Write the low-power oscillator's divisor. Only with the counter
    /// stopped or not running off it (12.10.5.1), which this verb
    /// enforces. The integer field is six bits: a rate at or above
    /// 64 kHz cannot be described here.
    static bool lposc_khz(uint8_t whole, uint16_t frac) {
        if (whole == 0u || whole > POWMAN_LPOSC_FREQ_KHZ_INT_BITS) {
            return false;
        }
        if (running() && source() == AonSource::lposc) {
            return false;
        }
        write_protected(POWMAN->LPOSC_FREQ_KHZ_INT, whole);
        write_protected(POWMAN->LPOSC_FREQ_KHZ_FRAC, frac);
        return true;
    }
    /// The same for the crystal: only with the counter stopped or not
    /// running off it. The chapter's floor is 2.0 kHz.
    static bool xosc_khz(uint16_t whole, uint16_t frac) {
        if (whole < 2u) {
            return false;
        }
        if (running() && source() == AonSource::xosc) {
            return false;
        }
        write_protected(POWMAN->XOSC_FREQ_KHZ_INT, whole);
        write_protected(POWMAN->XOSC_FREQ_KHZ_FRAC, frac);
        return true;
    }

    /**
     * MEASURE THE LOW-POWER OSCILLATOR AND TELL THE DIVIDER. The
     * frequency counter of 8.1.4 counts it against clk_ref - the crystal
     * after `Clock::init()` - and the result becomes LPOSC_FREQ_KHZ_INT
     * and _FRAC.
     *
     * Without this the timer is as wrong as the oscillator is, and this
     * oscillator is specified to +/- 20 per cent untrimmed. Returns the
     * rate it measured, or nothing when the counter could not see the
     * oscillator or the counter is running off it and may not be
     * retuned.
     */
    template <typename Clock>
    static std::optional<uint32_t> calibrate(Clock, uint8_t interval = 15) {
        const std::optional<uint32_t> hz =
            FreqCounter::count_hz(CountSource::lposc, Clock::ref_hz, interval);
        if (!hz.has_value() || *hz == 0u) {
            return std::nullopt;
        }
        const uint32_t whole = *hz / 1000u;
        const uint32_t frac = (static_cast<uint64_t>(*hz % 1000u) * 65536u) / 1000u;
        if (!lposc_khz(static_cast<uint8_t>(whole), static_cast<uint16_t>(frac))) {
            return std::nullopt;
        }
        return hz;
    }

    /**
     * Start the timer from zero on the low-power oscillator, with the
     * divisor set from a MEASUREMENT of that oscillator and the crystal
     * divisor set from the clock task's own crystal rate.
     *
     * The alarm is left disarmed, its interrupt off and its power-up
     * duty off: what wakes what is the program's decision and never a
     * default. False when the oscillator could not be counted - the
     * timer is then not started, because a counter told the wrong rate
     * is worse than none.
     */
    template <typename Clock>
    static bool init(Clock c) {
        run(false);
        alarm_enable(false);
        clear_alarm();
        powerup_on_alarm(false);
        interrupt(false);
        if (!xosc_khz(static_cast<uint16_t>(Clock::xtal_hz / 1000u),
                      static_cast<uint16_t>((static_cast<uint64_t>(Clock::xtal_hz % 1000u) * 65536u) /
                                            1000u))) {
            return false;
        }
        if (!calibrate(c).has_value()) {
            return false;
        }
        if (!set(0u)) {
            return false;
        }
        run(true);
        return true;
    }

private:
    static constexpr uint32_t password = 0x5AFEu << 16;

    static void write_protected(volatile uint32_t& reg, uint32_t value) {
        reg = password | value;
    }

    /// TIMER's ORDINARY read-write bits - the ones a read-modify-write
    /// may carry back. The four USING_* flags are read-only, ALARM is
    /// write-to-clear and the three USE_* selects are self-clearing, so
    /// none of them belongs in the value that is preserved.
    static constexpr uint32_t timer_rw = POWMAN_TIMER_USE_GPIO_1HZ_BITS |
                                         POWMAN_TIMER_PWRUP_ON_ALARM_BITS |
                                         POWMAN_TIMER_ALARM_ENAB_BITS | POWMAN_TIMER_RUN_BITS |
                                         POWMAN_TIMER_NONSEC_WRITE_BITS;

    /// One field of TIMER changed, the other read-write fields carried
    /// over and every write-to-clear and self-clearing bit written as a
    /// zero.
    static void write_timer(uint32_t value, uint32_t mask) {
        const uint32_t kept = (POWMAN->TIMER & timer_rw) & ~mask;
        write_protected(POWMAN->TIMER, kept | (value & mask));
    }
    /// The same, plus the bits named written as ones: for the
    /// write-to-clear and self-clearing half, which `write_timer` is
    /// careful never to raise by accident.
    static void write_timer_raw(uint32_t bits) {
        write_protected(POWMAN->TIMER, (POWMAN->TIMER & timer_rw) | bits);
    }

    /// Wait for a source change to take: the select bit clears itself
    /// and the matching USING_* flag rises, which can take a whole tick
    /// of the OLD source.
    static bool wait_source(AonSource want) {
        for (uint32_t spins = 4'000'000u; spins != 0u; --spins) {
            if (source() == want) {
                return true;
            }
        }
        return false;
    }
};

inline bool Powman::powerup_armed() {
    if (AonTimer::alarm_enabled() && AonTimer::powerup_on_alarm()) {
        return true;
    }
    for (uint8_t s = 0; s < 4u; ++s) {
        if (pwrup_enabled(s)) {
            return true;
        }
    }
    return false;
}

} // namespace brio
