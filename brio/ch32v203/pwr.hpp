/*
 * pwr.hpp
 *
 * The power controller (RM ch. 2): the three low-power modes and the
 * pair of bits that choose between them, the regulator's price in a
 * Stop, what a Standby keeps of the RAM, the supply monitor with its
 * own EXTI line, the wake-up pad, and the two flags a program reads at
 * the boot to learn how it got there.
 *
 * TWO REGISTERS, AND A THIRD PLACE. PWR_CTLR and PWR_CSR are the whole
 * chapter (table 2-2); their map is ch32v203/device.hpp's, because the
 * RTC chapter reaches the same block for DBP alone. Two bits that
 * belong to this subject are in neither register - the core voltage and
 * the low-power one - because this family puts in EXTEN_CTR (ch. 33)
 * what did not fit elsewhere; they are verbs of `Pwr` here, so a reader
 * looking for the regulator finds it in the power chapter's file. A
 * third bit of that register, HSEPLP, is the CLOCK chapter's
 * (`Rcc::hse_in_low_power()`): it is about the oscillator.
 *
 * ## The three modes, and the ONE pair of bits that names them
 *
 * Table 2-1, with the entry conditions of 2.3.2 .. 2.3.4:
 *
 *   Sleep    SLEEPDEEP = 0, PDDS = 0    the core clock stops and
 *                                       nothing else does; any
 *                                       interrupt or wake event ends it
 *   Stop     SLEEPDEEP = 1, PDDS = 0    HSE, HSI, PLL and the
 *                                       peripheral clocks stop, SRAM
 *                                       and registers are kept, the
 *                                       pins hold their state; only an
 *                                       EXTI line ends it, and the
 *                                       program resumes ON THE HSI
 *   Standby  SLEEPDEEP = 1, PDDS = 1    the same, plus the regulator
 *                                       off; the documented exits are
 *                                       the WKUP pad's rising edge, the
 *                                       RTC alarm, the NRST pad and an
 *                                       IWDG reset - and AN ORDINARY
 *                                       EXTI LINE ends it too, which
 *                                       2.3.4 does not say (measured) -
 *                                       and every one of them comes
 *                                       back THROUGH A POWER RESET
 *
 * SLEEPDEEP is the core's, not this block's (2.3's note points at
 * PFIC_SCTLR), which is why `arm()` writes the pair together: neither
 * bit means anything without the other. Arming is not sleeping - the
 * instruction is the kernel idle path's, ch32v203/platform.hpp - and
 * that split is what lets util/power.hpp's model exist with no new
 * kernel hook. `enter()` is the deliberate one-shot for a program that
 * means to stop right here.
 *
 * LPDS PRICES A STOP: with it clear the regulator stays in its normal
 * mode, with it set the regulator goes low-power, and RAMLV on top of
 * that puts the RAM itself in its low-voltage mode (2.3.3, and RAMLV's
 * own note: "valid when the LPDS bit is 1"). Nothing here measures what
 * either is worth - that is a current on a meter and this chapter's
 * document says so - so they are a `StopConfig` a caller states and
 * this file only enforces the interlock.
 *
 * WHAT A STANDBY KEEPS, PER CLASS. 2.3.4 gives four bits, and the
 * register description keys them by device class in a way that is easy
 * to misread: on the CH32V20x_D6 - every part of this series but the
 * CH32V203RB - R2KSTY and R2KVBAT govern THE WHOLE 20 KB (which is
 * that class's LARGEST array: four parts of this series carry ten),
 * and R30KSTY and R30KVBAT do not exist; on the D8 they are a 2 KB
 * bank and a 30 KB one. So the verbs here are named for the banks and not for the sizes,
 * and `ram_retention_bytes` says what the first one covers on the part
 * being compiled.
 *
 * THE PVD IS TWO TABLES AND THE PART DOES NOT SAY WHICH. PLS[2:0]
 * selects one of eight thresholds, and 2.4.1 gives two sets of
 * voltages: one for dies whose FEATURE_SIGN.VLEVEL reads 1 (a supply
 * from 2.4 V) and one for dies where it reads 0 (from 1.8 V), with
 * 33.2.3 adding that the register is only to be believed when its low
 * byte is the inverse of the next - and that a die failing that test is
 * to be read as the 2.4 V kind. That is a fact about the DIE and not
 * about the part number, so it cannot be a constant in parts/: the
 * millivolts of a level are a RUN-TIME question here, and
 * `pvd_rising_mv(level)` reads the die to answer it. The constexpr
 * forms take the supply level as an argument for a program that knows.
 *
 * THE WAKE-UP PAD IS PA0, on every package of this series (datasheet
 * 3.2's pin tables name it PA0-WKUP in all four). EWUP forces it to an
 * input with a pull-down and arms its rising edge as a Standby exit;
 * with EWUP clear it is an ordinary pin. WUF is the flag it raises -
 * and the RTC alarm raises the same one (2.4.2's own wording), which is
 * why a program that arms both learns from it only THAT it was woken.
 *
 * AND WUF IS THE WAKE'S FLAG AND NOT THE EVENT'S, measured: neither an
 * alarm served with the core running nor an edge on the pad with EWUP
 * set and the core running moves it, while a Standby ended by that same
 * alarm comes back with it standing. It is therefore read at the BOOT,
 * beside SBF, and it tells one exit from another - a Standby ended by
 * an ordinary EXTI line comes back with SBF alone.
 *
 * THE DEBUG MODULE'S LOW-POWER BITS ARE READ HERE AND NEVER WRITTEN.
 * DBGMCU_CR (34.2.1) is a core CSR at 0x7C0 on this family, and its
 * three lowest bits decide whether FCLK and HCLK keep running in Sleep,
 * in Stop and in Standby - which is to say whether a debugger's
 * presence turns a measurement into a fiction. A `csrw` to that CSR
 * from the running program RESETS THE PART on this silicon (measured,
 * docs/ch32v203/README.md), so this file offers the three READS and no
 * writer at all: a suite says what it found and refuses to trust a
 * timing it did not take with them clear.
 *
 * WHAT IS NOT HERE. There is no flash power-down bit in this chapter
 * (PWR_CTLR [15:9] is reserved, where the sister family carries
 * FLASH_LP), no low-power RUN mode (2.3 lists three modes and no
 * fourth), and no auto-wake-up unit: the alarm that ends a Stop on this
 * family is the RTC's, through EXTI line 17 (2.3.5), and
 * ch32v203/sleep.hpp is what arms it. VBAT itself is a pin and not a
 * register.
 *
 * The numbers behind the measured statements above, and the rest of
 * what the silicon answered, are in docs/ch32v203/sleep.md.
 */

#pragma once

#include <stdint.h>

#include "ch32v203/clock.hpp"
#include "ch32v203/device.hpp"
#include "ch32v203/exti.hpp"

namespace brio {

// =============================================================================
// The bits (RM 2.4.1, 2.4.2)
// =============================================================================

/// PWR_CTLR (2.4.1). CWUF and CSBF are write-one-to-act and read as
/// zero always, so a read-modify-write of this register acknowledges
/// nothing by accident.
inline constexpr uint32_t pwr_lpds      = 1UL << 0;    ///< the regulator's low-power mode in Stop
inline constexpr uint32_t pwr_pdds      = 1UL << 1;    ///< 1: Standby, 0: Stop
inline constexpr uint32_t pwr_cwuf      = 1UL << 2;    ///< write 1: clear WUF, two clocks later
inline constexpr uint32_t pwr_csbf      = 1UL << 3;    ///< write 1: clear SBF
inline constexpr uint32_t pwr_pvde      = 1UL << 4;
inline constexpr uint32_t pwr_pls_mask  = 0x7UL << 5;
inline constexpr uint32_t pwr_pls_shift = 5;
// pwr_dbp (bit 8) is device.hpp's: the RTC chapter opens the domain with it.
inline constexpr uint32_t pwr_r2ksty    = 1UL << 16;   ///< the first RAM bank kept in Standby
inline constexpr uint32_t pwr_r30ksty   = 1UL << 17;   ///< the second bank, CH32V20x_D8 alone
inline constexpr uint32_t pwr_r2kvbat   = 1UL << 18;   ///< the first bank kept on VBAT
inline constexpr uint32_t pwr_r30kvbat  = 1UL << 19;   ///< the second on VBAT, D8 alone
inline constexpr uint32_t pwr_ramlv     = 1UL << 20;   ///< the RAM's low-voltage mode, needs LPDS

/// The four retention bits as one field: 2.4.1's closing note says that
/// bits 16..20 "can only be reset by backup" while every other bit of
/// the register is reset by a Standby wake, so they are the part of
/// PWR_CTLR that OUTLIVES the mode they configure.
inline constexpr uint32_t pwr_retention_bits =
    pwr_r2ksty | pwr_r30ksty | pwr_r2kvbat | pwr_r30kvbat | pwr_ramlv;

/// PWR_CSR (2.4.2). WUF, SBF and PVDO are read-only; EWUP is the only
/// bit a program writes here, and the register is NOT reset by a
/// Standby wake ("remains unchanged after woken up from Standby mode").
inline constexpr uint32_t pwr_wuf  = 1UL << 0;
inline constexpr uint32_t pwr_sbf  = 1UL << 1;
inline constexpr uint32_t pwr_pvdo = 1UL << 2;   ///< 1: VDD is BELOW the selected threshold
inline constexpr uint32_t pwr_ewup = 1UL << 8;

// =============================================================================
// The modes
// =============================================================================

/// What a following sleep instruction would take. The names are the
/// chapter's; `armed()` and `mode()` read them back off the two bits.
enum class PwrMode : uint8_t {
    sleep = 0,     ///< SLEEPDEEP 0
    stop = 1,      ///< SLEEPDEEP 1, PDDS 0
    standby = 2,   ///< SLEEPDEEP 1, PDDS 1
};

/// Whether leaving a mode is a reset rather than a return. Standby's
/// documented exits all are (2.3.4), which is why no sleep site arms it.
constexpr bool pwr_mode_resets(PwrMode m) { return m == PwrMode::standby; }

/// Whether a mode stops the clock tree - which is to say whether the
/// program comes back on the HSI with the PLL off, and whether the
/// core's counter stood still while it slept.
constexpr bool pwr_mode_stops_clocks(PwrMode m) { return m != PwrMode::sleep; }

/// LPDS: what the regulator does in a Stop.
enum class StopRegulator : uint8_t { main = 0, low_power = 1 };

/**
 * What a Stop costs to hold. Neither bit changes what the mode DOES -
 * the clocks stop either way and the wake is an EXTI line either way -
 * so what they buy is current and what they cost is wake latency, and
 * this file measures neither.
 *
 * `ram_low_voltage` is RAMLV, whose own note makes it valid only with
 * the low-power regulator; `stop_config_valid()` is that interlock and
 * the site's static_assert is where it bites.
 */
struct StopConfig {
    StopRegulator regulator = StopRegulator::main;
    bool ram_low_voltage = false;
};

constexpr bool stop_config_valid(const StopConfig& c) {
    return !c.ram_low_voltage || c.regulator == StopRegulator::low_power;
}

// =============================================================================
// The supply monitor (RM 2.2.2), and the die fact its table is keyed on
// =============================================================================

/// FEATURE_SIGN (33.2.3) at 0x1FFF F7D0: the die's own word, whose low
/// bit says what supply the part is specified down to. Read-only, and
/// outside every peripheral's window - the same region as the ESIG.
inline uint32_t feature_sign() {
    return *reinterpret_cast<volatile const uint32_t*>(0x1FFFF7D0UL);
}

/// Which of 2.4.1's two PVD tables this die uses.
enum class PwrSupplyLevel : uint8_t {
    from_1v8 = 0,   ///< VLEVEL = 0
    from_2v4 = 1,   ///< VLEVEL = 1, and what an unbelievable FEATURE_SIGN is read as
};

/**
 * The die's supply level, with 33.2.3's own validity test applied:
 * FEATURE_SIGN is to be believed only where its bits [15:8] are the
 * inverse of its bits [7:0], and a die that fails the test is the
 * 2.4 V kind. That makes this verb total - there is no "unknown"
 * answer to carry around - and it makes the conservative reading the
 * default, since the 2.4 V table's thresholds are the higher pair.
 */
inline PwrSupplyLevel pwr_supply_level() {
    const uint32_t sign = feature_sign();
    const uint32_t low = sign & 0xFFu;
    const uint32_t high = (sign >> 8) & 0xFFu;
    if (high != (~low & 0xFFu)) {
        return PwrSupplyLevel::from_2v4;
    }
    return (low & 1u) != 0u ? PwrSupplyLevel::from_2v4 : PwrSupplyLevel::from_1v8;
}

/// PLS[2:0]: the threshold, named by its code because what it is worth
/// in millivolts is the die's and not the part's (the file header).
enum class PvdLevel : uint8_t {
    level0 = 0, level1 = 1, level2 = 2, level3 = 3,
    level4 = 4, level5 = 5, level6 = 6, level7 = 7,
};

inline constexpr uint8_t pvd_level_count = 8;
inline constexpr PvdLevel pvd_level_lowest = PvdLevel::level0;
inline constexpr PvdLevel pvd_level_highest = PvdLevel::level7;

constexpr bool pvd_level_valid(PvdLevel level) {
    return static_cast<uint8_t>(level) < pvd_level_count;
}

/// The RISING threshold of a level, in millivolts, on a die of the
/// stated supply level (2.4.1's two tables).
constexpr uint16_t pvd_rising_mv(PvdLevel level, PwrSupplyLevel supply) {
    constexpr uint16_t from_2v4[8] = {2370, 2550, 2630, 2760, 2870, 3030, 3180, 3290};
    constexpr uint16_t from_1v8[8] = {2190, 2330, 2390, 2480, 2570, 2690, 2780, 2880};
    const uint8_t i = static_cast<uint8_t>(level) & 0x7u;
    return supply == PwrSupplyLevel::from_2v4 ? from_2v4[i] : from_1v8[i];
}

/// The FALLING threshold of the same level: where PVDO clears again as
/// VDD climbs back. The gap is the roughly 200 mV of hysteresis figure
/// 2-3 draws, stated per level by the tables.
constexpr uint16_t pvd_falling_mv(PvdLevel level, PwrSupplyLevel supply) {
    constexpr uint16_t from_2v4[8] = {2290, 2460, 2550, 2670, 2780, 2930, 3060, 3190};
    constexpr uint16_t from_1v8[8] = {2130, 2250, 2320, 2420, 2510, 2610, 2690, 2790};
    const uint8_t i = static_cast<uint8_t>(level) & 0x7u;
    return supply == PwrSupplyLevel::from_2v4 ? from_2v4[i] : from_1v8[i];
}

static_assert(pvd_rising_mv(pvd_level_lowest, PwrSupplyLevel::from_2v4) <
              pvd_rising_mv(pvd_level_highest, PwrSupplyLevel::from_2v4));
static_assert(pvd_falling_mv(PvdLevel::level3, PwrSupplyLevel::from_1v8) <
              pvd_rising_mv(PvdLevel::level3, PwrSupplyLevel::from_1v8));

// =============================================================================
// The block
// =============================================================================

/**
 * The power controller, monostate.
 *
 *   brio::Pwr::bus_clock(true);
 *   if (brio::Pwr::standby_flag()) { ... we came back from a Standby ... }
 *   brio::Pwr::clear_flags();
 *
 * EVERY VERB OPENS THE GATE FIRST. PWREN in RCC_PB1PCENR is clear out
 * of reset (3.4.8) and this block's registers answer nothing through a
 * closed gate, so a read that skipped the gate would report a state the
 * silicon never held.
 */
struct Pwr {
    Pwr() = delete;

    // ---- the gate -----------------------------------------------------------

    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(Bus::pb1, rcc_pb1_pwr);
        } else {
            Rcc::disable(Bus::pb1, rcc_pb1_pwr);
        }
    }
    static bool bus_clock() { return Rcc::enabled(Bus::pb1, rcc_pb1_pwr); }

    /// The gate, opened. Every verb below starts here.
    static void open() { Rcc::enable(Bus::pb1, rcc_pb1_pwr); }

    static uint32_t ctlr() { open(); return pwr()->CTLR; }
    static uint32_t csr() { open(); return pwr()->CSR; }

    // ---- the mode pair (2.3.1 .. 2.3.4) -------------------------------------

    /**
     * Put the machine in the state a following sleep instruction will
     * take: SLEEPDEEP and PDDS together, which is the only pair that
     * means anything.
     *
     * ARMING IS NOT SLEEPING. Nothing stops here; the core stops when
     * something executes the instruction, which in a brio program is
     * the kernel's idle path (ch32v203/platform.hpp). `mode()` reads
     * the state back off the silicon - there is no shadow of it here.
     */
    static bool arm(PwrMode m) {
        open();
        // SETEVENT is masked out of both stores: it is write-one-to-latch
        // and a read-modify-write of this register would arm an event
        // nobody asked for, which would end the next sleep before it
        // began (the platform's idle() takes the same care).
        if (m == PwrMode::sleep) {
            // 2.3.2's entry conditions are SLEEPDEEP = 0 AND PDDS = 0.
            // The first alone decides the mode - PDDS means nothing
            // without a deep sleep - but taking the second down with it
            // is what makes "nothing armed here can reach Standby" a
            // fact of the REGISTERS and not only of the code.
            pwr()->CTLR &= ~pwr_pdds;
            pfic_sctlr() = pfic_sctlr() & ~(sctlr_sleepdeep | sctlr_setevent);
            return true;
        }
        if (m == PwrMode::standby) {
            pwr()->CTLR |= pwr_pdds;
        } else {
            pwr()->CTLR &= ~pwr_pdds;
        }
        (void)pwr()->CTLR;   // the store has landed before SLEEPDEEP joins it
        pfic_sctlr() = (pfic_sctlr() | sctlr_sleepdeep) & ~sctlr_setevent;
        return true;
    }

    /// Arm a Stop and price it in one call. False, and nothing armed,
    /// for a configuration the chapter's own interlock forbids.
    static bool arm(PwrMode m, const StopConfig& c) {
        if (!stop_config(c)) {
            return false;
        }
        return arm(m);
    }

    /// What a following sleep instruction would take, read off the two
    /// bits: SLEEPDEEP clear is Sleep whatever PDDS holds.
    static PwrMode mode() {
        if ((pfic_sctlr() & sctlr_sleepdeep) == 0u) {
            return PwrMode::sleep;
        }
        open();
        return (pwr()->CTLR & pwr_pdds) != 0u ? PwrMode::standby : PwrMode::stop;
    }

    /// LPDS and RAMLV, the two bits that price a Stop. False for a
    /// configuration 2.4.1 forbids (RAMLV without LPDS), with nothing
    /// written.
    static bool stop_config(const StopConfig& c) {
        if (!stop_config_valid(c)) {
            return false;
        }
        open();
        uint32_t v = pwr()->CTLR & ~(pwr_lpds | pwr_ramlv);
        if (c.regulator == StopRegulator::low_power) { v |= pwr_lpds; }
        if (c.ram_low_voltage) { v |= pwr_ramlv; }
        pwr()->CTLR = v;
        return true;
    }

    static StopConfig stop_config() {
        const uint32_t v = ctlr();
        return StopConfig{(v & pwr_lpds) != 0u ? StopRegulator::low_power : StopRegulator::main,
                          (v & pwr_ramlv) != 0u};
    }

    // ---- the instruction ----------------------------------------------------

    /**
     * The plain wait-for-interrupt. On this core it sleeps until an
     * interrupt the core can TAKE arrives, which is why the platform's
     * idle path does not use it: with interrupts masked - the state the
     * kernel's idle hook is entered in - a bare wfi here would sleep
     * past every pending interrupt for ever on the sister family, and
     * whether this core keeps the specification's promise instead has
     * not been measured.
     */
    static void wait_for_interrupt() { __asm__ volatile("wfi" ::: "memory"); }

    /**
     * The wait-for-EVENT form, which is the one that is correct under
     * both readings: PFIC_SCTLR.WFITOWFE makes the next wfi wait for an
     * event, SEVONPEND makes any interrupt turning pending one, and the
     * event is LATCHED - so an interrupt that became pending between
     * the decision to sleep and this instruction ends the wait at once
     * instead of being slept past (2.3.1's third WFE shape, and the
     * platform's own idle()).
     */
    static void wait_for_event() {
        pfic_sctlr() = (pfic_sctlr() | sctlr_wfitowfe | sctlr_sevonpend) & ~sctlr_setevent;
        __asm__ volatile("wfi" ::: "memory");
    }

    /**
     * Arm `m` and stop, here and now - the deliberate one-shot for a
     * program with no kernel under it, and the ONLY door to Standby in
     * this stratum (ch32v203/sleep.hpp's ladder does not reach it).
     *
     * NOT [[noreturn]], although every Standby measured on this silicon
     * has ended in a POWER RESET - the RTC alarm's line as an interrupt,
     * the same line as an EVENT ALONE, and a PAD'S line as an event,
     * which is a wake 2.3.4 does not even list. Table 2-1's note says
     * the opposite ("any event can also wake up the system, but the
     * system will not be reset after wake-up"), and three lines are not
     * every source, so the verb stays one that may return and a caller
     * that means to end here spins after the call.
     *
     * It does NOT refuse over an active bus master. This is the
     * mechanism, and the policy - a sleep is legal only while the core
     * owns the bus - is the sleep site's, where the model can see it.
     */
    static void enter(PwrMode m) {
        (void)arm(m);
        wait_for_event();
    }

    // ---- the two flags (2.4.2) ----------------------------------------------

    /// WUF: a rising edge on the WKUP pad or an RTC alarm was detected.
    /// ONE FLAG FOR BOTH SOURCES, by 2.4.2's own wording.
    static bool wakeup_flag() { return (csr() & pwr_wuf) != 0u; }

    /// SBF: the machine entered Standby. Read at the boot, it is how a
    /// program that came back through the reset vector knows where from.
    static bool standby_flag() { return (csr() & pwr_sbf) != 0u; }

    /// CWUF: "clear the WUF after 2 system clock cycles" (2.4.1), so a
    /// read taken in the next instruction may still see it standing.
    static void clear_wakeup_flag() {
        open();
        pwr()->CTLR |= pwr_cwuf;
    }

    static void clear_standby_flag() {
        open();
        pwr()->CTLR |= pwr_csbf;
    }

    /// Both, in one store - the boot verb, after the flags have been read.
    static void clear_flags() {
        open();
        pwr()->CTLR |= pwr_cwuf | pwr_csbf;
    }

    // ---- the wake-up pad ----------------------------------------------------

    /// The pad EWUP claims, on every package of this series (datasheet
    /// 3.2). It is the port's own pin until EWUP is set, and on the
    /// bench board it is also the user key.
    static constexpr char wakeup_port = 'A';
    static constexpr uint8_t wakeup_pin_number = 0;
    static constexpr bool wakeup_pad_bonded =
        (device::port_pins(wakeup_port) & (1u << wakeup_pin_number)) != 0u;

    /**
     * EWUP: "WKUP is forced in input pull-down configuration, used to
     * wake up the MCU from standby mode" (2.4.2). Setting it takes the
     * pad away from the port - the pull-down is the hardware's, not
     * ch32v203/pin.hpp's - and its RISING edge is the wake. Returns
     * what the bit reads back as.
     */
    static bool wakeup_pin(bool on) {
        open();
        pwr()->CSR = on ? (pwr()->CSR | pwr_ewup) : (pwr()->CSR & ~pwr_ewup);
        return wakeup_pin() == on;
    }
    static bool wakeup_pin() { return (csr() & pwr_ewup) != 0u; }

    // ---- the supply monitor (2.2.2) -----------------------------------------

    /// The EXTI line the PVD's output is wired to (2.2.2 step 2, and
    /// 9.4.2's table): the path by which a supply falling through the
    /// threshold becomes an interrupt or a wake event.
    static constexpr uint8_t pvd_exti_line = Exti::line_pvd;

    /// Arm the detector at a threshold, or disarm it. The order is
    /// 2.2.2's: the level first, the enable after.
    static bool pvd(bool on, PvdLevel level = pvd_level_lowest) {
        if (!pvd_level_valid(level)) {
            return false;
        }
        open();
        uint32_t v = pwr()->CTLR & ~(pwr_pvde | pwr_pls_mask);
        v |= (static_cast<uint32_t>(level) << pwr_pls_shift) & pwr_pls_mask;
        pwr()->CTLR = v;
        if (on) {
            pwr()->CTLR = v | pwr_pvde;
        }
        return pvd() == on;
    }

    /// The same with the level a CONSTANT: a code past the three-bit
    /// field is a compile error here, where the run-time form can only
    /// answer false.
    template <PvdLevel level>
    static bool pvd(bool on) {
        static_assert(pvd_level_valid(level),
                      "brio Pwr: PLS is THREE bits (RM 2.4.1) - the eight thresholds are "
                      "PvdLevel::level0 .. level7, and what each is worth in millivolts is "
                      "the DIE's (pvd_rising_mv)");
        return pvd(on, level);
    }

    static bool pvd() { return (ctlr() & pwr_pvde) != 0u; }
    static PvdLevel pvd_level() {
        return static_cast<PvdLevel>((ctlr() & pwr_pls_mask) >> pwr_pls_shift);
    }

    /// PVDO: true while VDD/VDDA sits BELOW the selected threshold.
    /// Valid only with the detector enabled (2.4.2).
    static bool supply_low() { return (csr() & pwr_pvdo) != 0u; }

    /// The die's supply level and what the selected threshold is worth
    /// on it - the run-time forms, because the table is the die's
    /// (the file header).
    static PwrSupplyLevel supply_level() { return pwr_supply_level(); }
    static uint16_t pvd_rising_mv(PvdLevel level) {
        return brio::pvd_rising_mv(level, supply_level());
    }
    static uint16_t pvd_falling_mv(PvdLevel level) {
        return brio::pvd_falling_mv(level, supply_level());
    }

    /// Point EXTI line 16 at the detector and arm it as an INTERRUPT
    /// (the PFIC line is the caller's), or take it down. Both edges by
    /// default: the fall is the warning and the rise is the recovery.
    static bool arm_pvd(bool on, ExtiSense sense = ExtiSense::both) {
        if (!Exti::sense(pvd_exti_line, on ? sense : ExtiSense::none)) {
            return false;
        }
        return Exti::interrupt(pvd_exti_line, on);
    }

    /// The same as an EVENT rather than an interrupt: what ends a sleep
    /// with no handler at all.
    static bool arm_pvd_event(bool on, ExtiSense sense = ExtiSense::both) {
        if (!Exti::sense(pvd_exti_line, on ? sense : ExtiSense::none)) {
            return false;
        }
        return Exti::event(pvd_exti_line, on);
    }

    /// The body of the handler on the PVD's own vector (`Irq::pvd`):
    /// the line's flag cleared - it is the only one to clear, PVDO
    /// being a level and not a latch - and the level handed back.
    [[gnu::always_inline]] static bool pvd_isr() {
        const bool low = supply_low();
        (void)Exti::clear(pvd_exti_line);
        return low;
    }

    // ---- what a Standby keeps (2.3.4) ---------------------------------------

    /// Whether this part has the SECOND retention bank at all: the
    /// 30 KB one is the CH32V20x_D8's and the CH32V30x_D8's (2.4.1's
    /// notes list both), and on the D6 the first bit governs the whole
    /// array.
    static constexpr bool has_upper_ram_retention =
        device::device_class != DeviceClass::v20x_d6;

    /// How many bytes the FIRST retention bit covers on this part.
    /// 2.4.1 states it per class: the low 2 KB on the CH32V20x_D8 and the
    /// CH32V30x_D8, and "the 20K RAM" on the D6 - WHICH IS THAT CLASS'S
    /// LARGEST ARRAY, while four parts of this series carry ten. The
    /// constant is therefore the manual's number bounded by what the part
    /// has, and whether the bit really reaches the whole array of a
    /// smaller part is a question docs/ch32v203/sleep.md's gap list
    /// carries.
    static constexpr uint32_t ram_retention_bytes =
        device::device_class != DeviceClass::v20x_d6
            ? 2UL * 1024UL
            : (device::sram_bytes < 20UL * 1024UL ? device::sram_bytes : 20UL * 1024UL);

    /// R2KSTY: keep the first bank powered through a Standby on VDD.
    static void retain_ram(bool on) {
        open();
        pwr()->CTLR = on ? (pwr()->CTLR | pwr_r2ksty) : (pwr()->CTLR & ~pwr_r2ksty);
    }
    static bool retain_ram() { return (ctlr() & pwr_r2ksty) != 0u; }

    /// R2KVBAT: the same bank charged while the supply is VBAT.
    static void retain_ram_on_vbat(bool on) {
        open();
        pwr()->CTLR = on ? (pwr()->CTLR | pwr_r2kvbat) : (pwr()->CTLR & ~pwr_r2kvbat);
    }
    static bool retain_ram_on_vbat() { return (ctlr() & pwr_r2kvbat) != 0u; }

    /// R30KSTY and R30KVBAT: the upper bank, on the class that has one.
    /// Elsewhere the verbs write nothing and read false, because the
    /// bits are reserved there and the first pair already covers the
    /// whole array.
    static void retain_upper_ram(bool on) {
        if constexpr (has_upper_ram_retention) {
            open();
            pwr()->CTLR = on ? (pwr()->CTLR | pwr_r30ksty) : (pwr()->CTLR & ~pwr_r30ksty);
        } else {
            (void)on;
        }
    }
    static bool retain_upper_ram() {
        if constexpr (has_upper_ram_retention) {
            return (ctlr() & pwr_r30ksty) != 0u;
        } else {
            return false;
        }
    }
    static void retain_upper_ram_on_vbat(bool on) {
        if constexpr (has_upper_ram_retention) {
            open();
            pwr()->CTLR = on ? (pwr()->CTLR | pwr_r30kvbat) : (pwr()->CTLR & ~pwr_r30kvbat);
        } else {
            (void)on;
        }
    }
    static bool retain_upper_ram_on_vbat() {
        if constexpr (has_upper_ram_retention) {
            return (ctlr() & pwr_r30kvbat) != 0u;
        } else {
            return false;
        }
    }

    // ---- the regulator's own bits, which live in EXTEN (RM 33.2.1) ----------

    /// LDOTRIM: the digital core's voltage, 1.3 / 1.2 / 1.1 / 1.0 V by
    /// code, 1.1 V out of reset. A code of this register and not a
    /// millivolt count, because what a lowered core voltage costs in
    /// maximum clock rate is nowhere in the documents.
    static uint8_t core_voltage() {
        return static_cast<uint8_t>((exten()->CTR & exten_ldotrim_mask) >> 10);
    }
    static void core_voltage(uint8_t code) {
        const uint32_t ctr = exten()->CTR & ~exten_lkuprst;
        exten()->CTR = (ctr & ~exten_ldotrim_mask) |
                       ((static_cast<uint32_t>(code) << 10) & exten_ldotrim_mask);
    }

    /// ULLDOTRIM: the same for the ultra-low-power regulator, which is
    /// the one a Stop with LPDS set runs on. The manual gives the field
    /// no voltage table at all, only "adjust ULLDO voltage value in
    /// low-power mode" and a reset code of 10b.
    static uint8_t low_power_voltage() {
        return static_cast<uint8_t>((exten()->CTR & exten_ulldotrim_mask) >> 8);
    }
    static void low_power_voltage(uint8_t code) {
        const uint32_t ctr = exten()->CTR & ~exten_lkuprst;
        exten()->CTR = (ctr & ~exten_ulldotrim_mask) |
                       ((static_cast<uint32_t>(code) << 8) & exten_ulldotrim_mask);
    }

    // HSEPLP - whether the crystal keeps oscillating through a
    // low-power mode, and so whether a Stop comes back without paying
    // the oscillator's start-up again - is in the same register and is
    // NOT here: the HSE is the clock chapter's and the verb is
    // `Rcc::hse_in_low_power()`, which already answers false on every
    // class but the CH32V20x_D8.

    // ---- what the debugger left behind (RM 34.2.1) --------------------------

    /// DBGMCU_CR, a core CSR at 0x7C0 on this family. READ ONLY HERE:
    /// a csrw to it from the running program resets the part on this
    /// silicon, so the three bits below are reported and never set -
    /// what they are for, from this file's side, is telling a
    /// measurement from a fiction.
    static uint32_t debug_cr() {
        uint32_t v;
        __asm__ volatile("csrr %0, 0x7C0" : "=r"(v));
        return v;
    }

    /// FCLK and HCLK kept running in Sleep, in Stop and in Standby.
    /// Any of them set means a probe is holding the clocks up and the
    /// sleep being measured is not the sleep the silicon would take.
    static bool debug_in_sleep() { return (debug_cr() & (1UL << 0)) != 0u; }
    static bool debug_in_stop() { return (debug_cr() & (1UL << 1)) != 0u; }
    static bool debug_in_standby() { return (debug_cr() & (1UL << 2)) != 0u; }
    static bool debug_holds_clocks() {
        return (debug_cr() & 0x7UL) != 0u;
    }
};

} // namespace brio
