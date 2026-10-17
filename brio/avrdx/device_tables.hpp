/*
 * device_tables.hpp
 *
 * THE RESERVE of the avrdx stratum - the one file where the preprocessor
 * is allowed to ask the device header what exists. The standing rule
 * (docs/design/overview.md, "Generalization rule"): a probe that yields
 * a VALUE lives here and is exported as constexpr data; a driver keeps
 * `#ifdef` only to select per-instance CODE (a register-struct
 * reference, a block the part has not got), never to learn a fact.
 *
 * THREE KINDS OF ENTRY, each asked of the thing that IS the answer:
 *
 *  - FEATURES AND INSTANCES. Each a probe of the header symbol that is
 *    the feature itself: `MVIO` is the multi-voltage I/O block,
 *    `CLKCTRL_XOSCHFCTRLA` the high-frequency crystal oscillator's
 *    register, `TCA1` the second TCA, `CLKCTRL_PLLS_bm` the PLL. Where
 *    the header names a feature by an ENUMERATOR rather than a macro (an
 *    AC power profile, a window selection), the probe is a `requires` on
 *    the enum type; where it names it by a struct MEMBER (an event
 *    channel, an op amp's registers), a `requires` on the struct. A
 *    feature is never borrowed to mean a FAMILY: "has MVIO" is a fact
 *    about PORTC's supply, and it stops meaning "an AVR DB" the moment
 *    any other die without MVIO joins the stratum.
 *
 *  - THE PART: its DIE and its PACKAGE. What only a die can key - the
 *    errata, whose sheets are written per die (DS80000882C for the
 *    AVR128DA28/32/48/64, DS80000915F for the AVR128DB28/32/48/64,
 *    DS80001048D for the AVR64EA28/32/48) - and the package's pin
 *    count, which no peripheral macro separates on every die. A feature
 *    combination cannot stand in for the die: an AVR64DD32's header
 *    carries MVIO, TCD0, the PLL and XOSCHF exactly as a DB's does, so
 *    "MVIO and a TCD" would hand it the DB's errata without a word. The
 *    die is therefore keyed on the compiler's device macro (`-mmcu`
 *    defines `__AVR_AVR128DB48__`, ...), grouped by errata sheet, and a
 *    part this table does not list is a compile error: the stratum's
 *    claim (avrdx/cmake/avr-mcus.cmake's packages) made at compile time.
 *    The AVR64EA rows describe the parts the stratum is widening to;
 *    the rest of the stratum does not build for them yet.
 *
 *  - THE ERRATA AND THE FAMILY FACTS NO HEADER CARRIES, as values keyed
 *    on the die, each with its section. Only the items a driver reads
 *    are here: a value no code consults would be a claim with no guard.
 *
 * The PORTMUX route fields are probed here too (mask and position per
 * instance, from the header's `_gm`/`_gp`): their WIDTH and which of a
 * peripheral's route registers holds them is the one thing that moves
 * between the layouts, and the drivers write the fields from these
 * values rather than from a width they assume.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

namespace brio {

// =============================================================================
// The part: die and package
// =============================================================================

/// The silicon an image runs on, one value per errata sheet.
enum class AvrDie : uint8_t {
    avr128da,   ///< AVR128DA28/32/48/64, errata DS80000882C
    avr128db,   ///< AVR128DB28/32/48/64, errata DS80000915F
    avr64ea,    ///< AVR64EA28/32/48, errata DS80001048D
};

#if defined(__AVR_AVR128DA28__)
inline constexpr AvrDie avr_die = AvrDie::avr128da;
inline constexpr uint8_t package_pins = 28;
#elif defined(__AVR_AVR128DA32__)
inline constexpr AvrDie avr_die = AvrDie::avr128da;
inline constexpr uint8_t package_pins = 32;
#elif defined(__AVR_AVR128DA48__)
inline constexpr AvrDie avr_die = AvrDie::avr128da;
inline constexpr uint8_t package_pins = 48;
#elif defined(__AVR_AVR128DA64__)
inline constexpr AvrDie avr_die = AvrDie::avr128da;
inline constexpr uint8_t package_pins = 64;
#elif defined(__AVR_AVR128DB28__)
inline constexpr AvrDie avr_die = AvrDie::avr128db;
inline constexpr uint8_t package_pins = 28;
#elif defined(__AVR_AVR128DB32__)
inline constexpr AvrDie avr_die = AvrDie::avr128db;
inline constexpr uint8_t package_pins = 32;
#elif defined(__AVR_AVR128DB48__)
inline constexpr AvrDie avr_die = AvrDie::avr128db;
inline constexpr uint8_t package_pins = 48;
#elif defined(__AVR_AVR128DB64__)
inline constexpr AvrDie avr_die = AvrDie::avr128db;
inline constexpr uint8_t package_pins = 64;
#elif defined(__AVR_AVR64EA28__)
inline constexpr AvrDie avr_die = AvrDie::avr64ea;
inline constexpr uint8_t package_pins = 28;
#elif defined(__AVR_AVR64EA32__)
inline constexpr AvrDie avr_die = AvrDie::avr64ea;
inline constexpr uint8_t package_pins = 32;
#elif defined(__AVR_AVR64EA48__)
inline constexpr AvrDie avr_die = AvrDie::avr64ea;
inline constexpr uint8_t package_pins = 48;
#else
// The compiler's own name for the part, said before the refusal.
#define BRIO_AVR_PART_STR2(x) #x
#define BRIO_AVR_PART_STR(x) BRIO_AVR_PART_STR2(x)
#pragma message("brio avrdx: the part is " BRIO_AVR_PART_STR(__AVR_DEVICE_NAME__))
#undef BRIO_AVR_PART_STR
#undef BRIO_AVR_PART_STR2
#error "brio avrdx: a part this stratum does not claim (AVR128DA28/32/48/64, AVR128DB28/32/48/64, AVR64EA28/32/48) - its errata are not known here"
#endif

// =============================================================================
// Features and instances
// =============================================================================

/// Whether this package bonds out the port at all: the 28/32-pin parts
/// lack PORTB and PORTE, PORTG exists on 64 pins only. Port-level only:
/// a pin missing from an EXISTING port (PF4 on a 28-pin part) is a
/// pin-level bonding fact the drivers' route tables carry. Drivers use
/// this to keep an instance usable when only one of its pin POSITIONS
/// is absent (an `if constexpr` on the missing branch), and to refuse a
/// config that asks for it.
constexpr bool port_exists(char p) {
    switch (p) {
        case 'A': case 'C': case 'D': case 'F': return true;
#ifdef PORTB
        case 'B': return true;
#endif
#ifdef PORTE
        case 'E': return true;
#endif
#ifdef PORTG
        case 'G': return true;
#endif
        default: return false;
    }
}

/// Whether PORTC is the MVIO domain, powered from VDDIO2.
#ifdef MVIO
inline constexpr bool has_mvio = true;
#else
inline constexpr bool has_mvio = false;
#endif

/// Whether the part has the high-frequency crystal oscillator (XOSCHF:
/// a crystal on PA0/PA1, or an external clock through it). A part
/// without it takes an external CLOCK on PA0 directly; a program
/// choosing its HF source asks this, never the device header.
#if defined(CLKCTRL_XOSCHFCTRLA)
inline constexpr bool has_xoschf = true;
#else
inline constexpr bool has_xoschf = false;
#endif

/// The PLL that clocks the TCD (its status bit is the block).
#if defined(CLKCTRL_PLLS_bm)
inline constexpr bool has_pll = true;
#else
inline constexpr bool has_pll = false;
#endif

/// OSCHF's run-time frequency select (FRQSEL): where it is absent the
/// oscillator's rate is chosen by a fuse.
#if defined(CLKCTRL_FRQSEL_gm)
inline constexpr bool has_oschf_frqsel = true;
#else
inline constexpr bool has_oschf_frqsel = false;
#endif

/// CLKCTRL.MCLKTIMEBASE: CLK_PER cycles per microsecond, the time base
/// other peripherals count in.
#if defined(CLKCTRL_TIMEBASE_gm)
inline constexpr bool has_clock_timebase = true;
#else
inline constexpr bool has_clock_timebase = false;
#endif

/// The voltage regulator's sleep control (SLPCTRL.VREGCTRL.PMODE).
#if defined(SLPCTRL_PMODE_gm)
inline constexpr bool has_vreg = true;
#else
inline constexpr bool has_vreg = false;
#endif

/// PINnCTRL.INLVL: the TTL input threshold.
#if defined(PORT_INLVL_bm)
inline constexpr bool has_inlvl = true;
#else
inline constexpr bool has_inlvl = false;
#endif

/// PORTx.EVGENCTRLA: pin event generators as two slots per port.
#if defined(PORT_EVGEN0SEL_gm)
inline constexpr bool has_port_evgen = true;
#else
inline constexpr bool has_port_evgen = false;
#endif

/// RTC.PITEVGENCTRLA: PIT event generators as two slots.
#if defined(RTC_EVGEN0SEL_gm)
inline constexpr bool has_pit_evgen = true;
#else
inline constexpr bool has_pit_evgen = false;
#endif

/// VREF.ADC0REF: the ADC's reference chosen in the VREF block (where it
/// is absent, the ADC chooses its own).
#if defined(VREF_ADC0REF)
inline constexpr bool has_vref_adc0ref = true;
#else
inline constexpr bool has_vref_adc0ref = false;
#endif

/// The ADC's programmable gain amplifier (the differential ADC design).
#if defined(ADC_PGAEN_bm)
inline constexpr bool has_adc_pga = true;
#else
inline constexpr bool has_adc_pga = false;
#endif

/// NVMCTRL with a page buffer (STATUS.FLBUSY is NVM version 3's own bit).
#if defined(NVMCTRL_FLBUSY_bm)
inline constexpr bool has_nvm_page_buffer = true;
#else
inline constexpr bool has_nvm_page_buffer = false;
#endif

/// RAMPZ (ELPM reaches above 64 KB) and the FLMAP flash section
/// mapping, as the compiler knows the core.
#if defined(__AVR_HAVE_RAMPZ__)
inline constexpr bool has_rampz = true;
#else
inline constexpr bool has_rampz = false;
#endif
#if defined(__AVR_HAVE_FLMAP__)
inline constexpr bool has_flmap = true;
#else
inline constexpr bool has_flmap = false;
#endif

/// RSTCTRL.SWRR's software-reset bit, whatever this header calls it
/// (same bit, same CCP-protected register, two names).
#if defined(RSTCTRL_SWRE_bm)
inline constexpr uint8_t software_reset_bm = RSTCTRL_SWRE_bm;
#else
inline constexpr uint8_t software_reset_bm = RSTCTRL_SWRST_bm;
#endif

/// The TCD (one instance where it exists).
#if defined(TCD0)
inline constexpr bool has_tcd = true;
#else
inline constexpr bool has_tcd = false;
#endif

/// Instance counts, each read off the instances' own macros.
#if defined(TCA1)
inline constexpr uint8_t tca_count = 2;
#else
inline constexpr uint8_t tca_count = 1;
#endif

#if defined(TCB4)
inline constexpr uint8_t tcb_count = 5;
#elif defined(TCB3)
inline constexpr uint8_t tcb_count = 4;
#else
inline constexpr uint8_t tcb_count = 3;
#endif

#if defined(USART5)
inline constexpr uint8_t usart_count = 6;
#elif defined(USART4)
inline constexpr uint8_t usart_count = 5;
#elif defined(USART3)
inline constexpr uint8_t usart_count = 4;
#else
inline constexpr uint8_t usart_count = 3;
#endif

#if defined(SPI1)
inline constexpr uint8_t spi_count = 2;
#else
inline constexpr uint8_t spi_count = 1;
#endif

#if defined(TWI1)
inline constexpr uint8_t twi_count = 2;
#else
inline constexpr uint8_t twi_count = 1;
#endif

#if defined(AC2)
inline constexpr uint8_t ac_count = 3;
#elif defined(AC1)
inline constexpr uint8_t ac_count = 2;
#else
inline constexpr uint8_t ac_count = 1;
#endif

#if defined(ZCD2)
inline constexpr uint8_t zcd_count = 3;
#elif defined(ZCD1)
inline constexpr uint8_t zcd_count = 2;
#elif defined(ZCD0)
inline constexpr uint8_t zcd_count = 1;
#else
inline constexpr uint8_t zcd_count = 0;
#endif

/// CCL look-up tables: the LUT4 register macros exist where six do.
#if defined(CCL_LUT4CTRLA)
inline constexpr uint8_t lut_count = 6;
#else
inline constexpr uint8_t lut_count = 4;
#endif

/// The concepts below make a member or enumerator lookup a substitution
/// rather than a hard error: a requires-expression over a NON-dependent
/// type is checked at once and would not compile where the name is
/// absent.
template <typename T>
concept HasEventChannel6 = requires(T& e) { e.CHANNEL6; };
template <typename T>
concept HasEventChannel8 = requires(T& e) { e.CHANNEL8; };

/// Event channels: CHANNEL0..n-1 as the EVSYS struct declares them.
inline constexpr uint8_t event_channels =
    HasEventChannel8<EVSYS_t> ? 10 : HasEventChannel6<EVSYS_t> ? 8 : 6;

/// Op amps: none without the OPAMP block; two or three by whether the
/// OP2 registers are in its struct.
#ifdef OPAMP
template <typename T>
concept HasOpamp2Registers = requires(T& o) { o.OP2CTRLA; };
inline constexpr uint8_t opamp_count = HasOpamp2Registers<OPAMP_t> ? 3 : 2;
#else
inline constexpr uint8_t opamp_count = 0;
#endif

/// The AC's third power profile and its window pairing with ACn+2:
/// enumerators, so the probe is a lookup in the enum type. The codes are
/// 0xFF where the header has no such enumerator.
template <typename E>
concept HasAcPowerProfile2 = requires { E::AC_POWER_PROFILE2_gc; };
template <typename E>
concept HasAcWinselUpsel2 = requires { E::AC_WINSEL_UPSEL2_gc; };

template <typename E>
constexpr uint8_t ac_power_profile2_code_of() {
    if constexpr (HasAcPowerProfile2<E>) return static_cast<uint8_t>(E::AC_POWER_PROFILE2_gc);
    else return 0xFF;
}
template <typename E>
constexpr uint8_t ac_winsel_upsel2_code_of() {
    if constexpr (HasAcWinselUpsel2<E>) return static_cast<uint8_t>(E::AC_WINSEL_UPSEL2_gc);
    else return 0xFF;
}

inline constexpr uint8_t ac_power_profile2_code = ac_power_profile2_code_of<AC_POWER_t>();
inline constexpr uint8_t ac_winsel_upsel2_code = ac_winsel_upsel2_code_of<AC_WINSEL_t>();
inline constexpr bool has_ac_profile2 = ac_power_profile2_code != 0xFF;
inline constexpr bool has_ac_upsel2 = ac_winsel_upsel2_code != 0xFF;

// =============================================================================
// PORTMUX route fields
// =============================================================================
//
// Mask and position of each instance's route field, the header's own
// `_gm`/`_gp`; 0 for an instance this part has not got. The masks are
// the layout: two bits per instance on the AVR DA/DB, three for USART0
// and SPI0 on the AVR EA.

constexpr uint8_t usart_route_gm(uint8_t n) {
    switch (n) {
        case 0: return PORTMUX_USART0_gm;
        case 1: return PORTMUX_USART1_gm;
        case 2: return PORTMUX_USART2_gm;
#ifdef PORTMUX_USART3_gm
        case 3: return PORTMUX_USART3_gm;
#endif
#ifdef PORTMUX_USART4_gm
        case 4: return PORTMUX_USART4_gm;
#endif
#ifdef PORTMUX_USART5_gm
        case 5: return PORTMUX_USART5_gm;
#endif
        default: return 0;
    }
}

constexpr uint8_t usart_route_gp(uint8_t n) {
    switch (n) {
        case 0: return PORTMUX_USART0_gp;
        case 1: return PORTMUX_USART1_gp;
        case 2: return PORTMUX_USART2_gp;
#ifdef PORTMUX_USART3_gp
        case 3: return PORTMUX_USART3_gp;
#endif
#ifdef PORTMUX_USART4_gp
        case 4: return PORTMUX_USART4_gp;
#endif
#ifdef PORTMUX_USART5_gp
        case 5: return PORTMUX_USART5_gp;
#endif
        default: return 0;
    }
}

/// Whether USART n's field lives in USARTROUTEB: the fields of one
/// register never overlap, so an instance whose field overlaps a lower
/// instance's is in the second register (USART4 and USART5 on the AVR
/// DA/DB, USART2 on the AVR EA).
constexpr bool usart_route_in_b(uint8_t n) {
    for (uint8_t i = 0; i < n; ++i) {
        if ((usart_route_gm(i) & usart_route_gm(n)) != 0) return true;
    }
    return false;
}

constexpr uint8_t spi_route_gm(uint8_t n) {
    switch (n) {
        case 0: return PORTMUX_SPI0_gm;
#ifdef PORTMUX_SPI1_gm
        case 1: return PORTMUX_SPI1_gm;
#endif
        default: return 0;
    }
}

constexpr uint8_t spi_route_gp(uint8_t n) {
    switch (n) {
        case 0: return PORTMUX_SPI0_gp;
#ifdef PORTMUX_SPI1_gp
        case 1: return PORTMUX_SPI1_gp;
#endif
        default: return 0;
    }
}

constexpr uint8_t twi_route_gm(uint8_t n) {
    switch (n) {
        case 0: return PORTMUX_TWI0_gm;
#ifdef PORTMUX_TWI1_gm
        case 1: return PORTMUX_TWI1_gm;
#endif
        default: return 0;
    }
}

constexpr uint8_t twi_route_gp(uint8_t n) {
    switch (n) {
        case 0: return PORTMUX_TWI0_gp;
#ifdef PORTMUX_TWI1_gp
        case 1: return PORTMUX_TWI1_gp;
#endif
        default: return 0;
    }
}

constexpr uint8_t tca_route_gm(uint8_t n) {
    switch (n) {
        case 0: return PORTMUX_TCA0_gm;
#ifdef PORTMUX_TCA1_gm
        case 1: return PORTMUX_TCA1_gm;
#endif
        default: return 0;
    }
}

// =============================================================================
// Errata and family facts, keyed on the die
// =============================================================================

/// DS80000882C 2.6.1 (every AVR DA revision): PB6/PB7 and PE4..PE7 are
/// not connected to the event system, neither as a pin generator nor
/// as EVOUT.
inline constexpr bool erratum_da_evsys_pins = avr_die == AvrDie::avr128da;

/// DS80000882C 2.11.1 (every AVR DA revision): TCA1's ALT2 (PORTE) and
/// ALT3 (PORTG) routes do not work.
inline constexpr bool erratum_da_tca1_alt23 = avr_die == AvrDie::avr128da;

/// DS80000882C 2.14.2 (every AVR DA revision): the TWI's SDAHOLD 50 ns
/// and 300 ns selections are swapped in the silicon.
inline constexpr bool erratum_da_twi_sdahold_swap = avr_die == AvrDie::avr128da;

/// DS80000882C 2.7.1 (every AVR DA revision): the flash's data-space
/// window ignores the section protections.
inline constexpr bool erratum_da_window = avr_die == AvrDie::avr128da;

/// USART2's ALT1 route keeps XDIR on PF3 on the AVR DB; the AVR DA's
/// route table lists no XDIR for it (the two device headers' route
/// enum comments - a fact of the die's PORTMUX, carried by no symbol).
inline constexpr bool usart2_alt1_has_xdir = avr_die == AvrDie::avr128db;

} // namespace brio
