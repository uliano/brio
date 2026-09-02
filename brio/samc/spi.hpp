/*
 * spi.hpp
 *
 * The SAM C21 SERCOM in SPI mode (DS60001479M ch. 32, over the baud
 * generator and the pad matrix of the shared ch. 30), in the two strata
 * the rest of this target uses:
 *
 *  Spi<n>      the RESOURCE - a typed view of one instance's SPI
 *              register set: both roles, every field of CTRLA and
 *              CTRLB, the synchronous baud arithmetic, the flags and
 *              the ONE combined interrupt question this core's single
 *              vector has to ask.
 *
 *  SpiHost<n, pads>
 *              the TASK util/spi_bus.hpp (= util/bus_master.hpp) drives:
 *              the transfer ENGINE. A Request is a two-phase transaction
 *              in one chip-select window, the buffers travel with it as
 *              Lease::reply loans, and the completion is the bus AO's
 *              TransferDone. The SAME descriptor shape avrdx/spi.hpp
 *              carries, so a device client written for one architecture
 *              reads unchanged on the other.
 *
 *  SpiClient<n, pads>
 *              the other end of the wire: a polled surface plus the ISR
 *              bodies, with the three things this silicon gives a client
 *              and the AVR does not - preloading (CTRLB.PLOADEN),
 *              select-low detection (CTRLB.SSDE) and address recognition
 *              (CTRLA.FORM = 0x2 with CTRLB.AMODE and ADDR).
 *
 * ONE INSTANCE, ONE ADDRESS, ONE SET OF FACTS. Everything that belongs
 * to the SERCOM INSTANCE rather than to a mode - the APB mask, the GCLK
 * channel, the NVIC line, the two DMAC trigger codes - is Sercom<n>'s
 * and is used from here; only the REGISTER VIEW differs, and that comes
 * from Sercom<n>::spi_regs(). There is no second table of which channel
 * SERCOM5 sits on, and there cannot be one.
 *
 * THE HEADER'S TWO SPI VIEWS ARE THE SAME REGISTERS. sercom.h declares
 * `sercom_spim_registers_t` and `sercom_spis_registers_t` with identical
 * offsets and identical field positions under two name prefixes, so this
 * file uses ONE (SPIM) for both roles and the role is CTRLA.MODE and
 * nothing else. That is asserted at the bottom of this file against the
 * header's own SPIS macros rather than believed.
 *
 * PADS ARE NOT PINS, AND DOPO IS A TRIPLE. CTRLA.DIPO names the pad the
 * data INPUT is read from and takes any of the four. CTRLA.DOPO does
 * NOT name a pad: its four codes each fix a whole (DO, SCK, SS) triple
 * (32.8.1's table), so the three cannot be chosen independently and
 * "swap SCK and SS" is not a configuration the silicon offers. A caller
 * states the triple it has wired and spi_dopo_for() answers with the one
 * code that produces it, or with nothing - which is a compile error at
 * the task. NOTE the device header's naming trap, the same one TXPO has
 * in sercom.hpp: it calls DOPO = 0x1 `DOPO_PAD1_Val` while that code
 * puts DO on PAD[2]; the enumerator names the CODE, not the pad.
 * Everything below is named after the pad a SIGNAL lands on.
 *
 * WHICH SIGNAL IS WHICH DEPENDS ON THE ROLE (32.8.1, table 32-2):
 *
 *              host            client
 *   DO         MOSI (out)      MISO (out)
 *   DI         MISO (in)       MOSI (in)
 *   SCK        out             in
 *   SS         out, MSSEN=1    in, always
 *
 * so on ONE fixed four-wire harness the two roles are two DIFFERENT
 * DOPO rows, not one row with the directions flipped. The bench proves
 * exactly that: the same four wires carry a host on row 0x0 and a
 * client on row 0x2.
 *
 * Facts that shape the code (DS60001479M 32.5.x, 32.6.x, 32.8.x, and
 * errata DS80000740S, silicon rev F on the bench chip):
 *  - CTRLA, CTRLB, BAUD and ADDR are ENABLE-PROTECTED (32.6.2.1): a
 *    write while the peripheral is enabled is DISCARDED, not refused.
 *    Everything that changes them here disables first. The exceptions
 *    the chapter names are CTRLA.ENABLE, CTRLA.SWRST and CTRLB.RXEN;
 *  - SYNCBUSY has exactly three bits (32.8.8): SWRST, ENABLE, CTRLB.
 *    Writing CTRLB while SYNCBUSY.CTRLB stands is an APB ERROR, so
 *    every CTRLB write here is followed by a bounded wait;
 *  - ENABLING THE PERIPHERAL CLEARS CTRLB.RXEN (32.8.2) and raises
 *    SYNCBUSY.CTRLB until the receiver is really up - the same trap
 *    sercom.hpp's enable() documents for the USART, and the reason
 *    enable() here waits out BOTH synchronizations;
 *  - a client's DATA write needs THREE SCK CYCLES to reach the shift
 *    register (32.6.2.6.2), so the FIRST character of a transaction is
 *    never the one just written unless preloading is on. That is not a
 *    driver bug to hide, it is the reason CTRLB.PLOADEN exists;
 *  - RXC is set when a character has been fully shifted IN, and reading
 *    DATA is what clears it. It is therefore the exact "one character
 *    has moved, both ways" edge, which is why the host engine arms RXC
 *    and never DRE: one interrupt per byte, with the received byte in
 *    hand at the moment the next one is written;
 *  - STATUS carries ONE error bit, BUFOVF (32.6.2.7), and CTRLA.IBON
 *    decides whether it is raised at the overflow or travels with the
 *    data through the two-level receive buffer;
 *  - THERE IS NO EVENT SURFACE AT ALL: 32.5.6 and 32.6.4.3 are both
 *    "Not applicable". This is the first peripheral in this stratum with
 *    nothing to publish under the EVSYS ruling, and the absence is
 *    stated rather than left to be noticed;
 *  - erratum 1.17.16: CTRLA.SWRST does NOTHING while CTRLA.ENABLE = 0.
 *    reset() therefore enables the instance first when it finds it
 *    disabled - in SPI HOST mode, because the enable synchronizes
 *    against GCLK_SERCOMx_CORE and a CLIENT is clocked by an external
 *    SCK that may not be running at all;
 *  - erratum 1.17.3 (LIVE on every revision): with CTRLB.PLOADEN set,
 *    the first character a client sends is a dummy unless the host
 *    holds SS low for the WHOLE transmission. The driver cannot enforce
 *    what the other end of the wire does, so the obligation is stated on
 *    SpiConfig::preload and nowhere pretended away;
 *  - erratum 1.17.19 (LIVE): DBGCTRL is reset by CTRLA.SWRST although
 *    32.6.2.2 promises it is the one register a reset spares. configure()
 *    writes DBGCTRL AFTER any reset for that reason;
 *  - erratum 1.17.20 (LIVE): a client left with a preloaded character on
 *    the way into standby consumes extra power. A power-pass fact, named
 *    on the field;
 *  - erratum 1.17.1 is REVISION B ONLY on the E/G/J row (the spurious
 *    INTFLAG.SSL when a client is enabled with SSDE and RXEN together).
 *    It is named here because it is the one item a reader would apply
 *    without checking the row, and it does NOT apply to this silicon.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// =============================================================================
// The knobs (32.8.1, 32.8.2)
// =============================================================================

/// Which end of the bus this instance is (CTRLA.MODE: 0x3 host, 0x2
/// client). There is NO runtime demotion on this peripheral - the AVR's
/// "a low SS turns a host into a client" has no counterpart here, and a
/// role is a configuration written while the SERCOM is disabled.
enum class SpiRole : uint8_t {
    host = SERCOM_SPIM_CTRLA_MODE_SPI_MASTER_Val,
    client = SERCOM_SPIM_CTRLA_MODE_SPI_SLAVE_Val,
};

/// The four clock phase/polarity combinations (32.6.2.5, table 32-3).
/// SPELLED THE SAME WAY AS avrdx/spi.hpp: bit 1 is CPOL, bit 0 is CPHA,
/// so brio's SpiMode is one vocabulary across the two architectures even
/// though the register the bits go into is not the same one. Here they
/// are two separate bits of CTRLA rather than one field of CTRLB.
enum class SpiMode : uint8_t {
    mode0 = 0,   ///< SCK idle low,  sample on the leading (rising) edge
    mode1 = 1,   ///< SCK idle low,  sample on the trailing (falling) edge
    mode2 = 2,   ///< SCK idle high, sample on the leading (falling) edge
    mode3 = 3,   ///< SCK idle high, sample on the trailing (rising) edge
};

/// The idle level of SCK for a mode (bit 1 = CPOL).
constexpr bool spi_cpol(SpiMode m) { return (static_cast<uint8_t>(m) & 0x02u) != 0; }
/// CPHA (bit 0): the sampling edge is the trailing one.
constexpr bool spi_cpha(SpiMode m) { return (static_cast<uint8_t>(m) & 0x01u) != 0; }

/// CTRLB.CHSIZE (32.8.2). Codes 0x2..0x7 are Reserved and are refused.
enum class SpiCharSize : uint8_t {
    eight = SERCOM_SPIM_CTRLB_CHSIZE_8_BIT_Val,
    nine = SERCOM_SPIM_CTRLB_CHSIZE_9_BIT_Val,
};

/// CTRLA.FORM (32.8.1). Only two of the sixteen codes exist: 0x1 and
/// 0x3..0xF are Reserved. `with_address` is a CLIENT frame format -
/// 32.6.3.1 checks the first character of a transaction against ADDR.
enum class SpiForm : uint8_t {
    spi = SERCOM_SPIM_CTRLA_FORM_SPI_FRAME_Val,
    with_address = SERCOM_SPIM_CTRLA_FORM_SPI_FRAME_WITH_ADDR_Val,
};

/// CTRLB.AMODE (32.8.2), the client's address matching. 0x3 is Reserved.
enum class SpiAddressMode : uint8_t {
    mask = SERCOM_SPIM_CTRLB_AMODE_MASK_Val,             ///< ADDRMASK masks ADDR
    two_addresses = SERCOM_SPIM_CTRLB_AMODE_2_ADDRESSES_Val,  ///< ADDR and ADDRMASK both match
    range = SERCOM_SPIM_CTRLB_AMODE_RANGE_Val,           ///< ADDRMASK..ADDR inclusive
};

// =============================================================================
// Pads, pins and the DOPO triple
// =============================================================================

/**
 * Where the four SPI signals sit. `data_out`/`sck`/`ss` must together be
 * one of DOPO's four rows; `data_in` is free (DIPO takes any pad).
 *
 * The PIN half is the board's, exactly as UartPads has it: the device
 * header's PIN_P<pad><fn>_SERCOM<n>_PAD<k> symbols say which pin reaches
 * which pad through which PMUX function, and an application states what
 * it wired. This header checks the pads exactly and the pins only as far
 * as it can (the group exists, the number is in range) - that a given
 * PIN really reaches that PAD is the open device-table question
 * sercom.hpp's header describes, and it is open here too.
 *
 * `ss_pin` is read ONLY when the SS PAD is claimed by the peripheral: a
 * client (always - it is what selects it) or a host with hardware SS
 * control. A host doing software chip select leaves that pad alone and
 * drives an ORDINARY GPIO, which is what SpiHost's Request carries as a
 * PinRef - and which is the only arrangement that can hold one select
 * window over several characters (see SpiConfig::hardware_ss).
 */
struct SpiPads {
    SercomPad data_out = SercomPad::pad0;   ///< DO: MOSI on a host, MISO on a client
    SercomPad sck = SercomPad::pad1;
    SercomPad ss = SercomPad::pad2;
    SercomPad data_in = SercomPad::pad3;    ///< DI: MISO on a host, MOSI on a client

    SercomPadPin data_out_pin{};
    SercomPadPin sck_pin{};
    SercomPadPin ss_pin{};
    SercomPadPin data_in_pin{};

    /// Whether the DI pad is muxed to the SERCOM at all. 32.5.1: "If the
    /// receiver is disabled, the data input pin can be used for other
    /// purposes" - a transmit-only host (a display, a shift register)
    /// leaves MISO to whoever else wants it.
    bool has_data_in = true;
};

/**
 * The CTRLA.DOPO code that produces this (DO, SCK, SS) triple, or
 * nothing when no code does. 32.8.1's table, transcribed:
 *
 *   code   DO       SCK      SS
 *   0x0    PAD[0]   PAD[1]   PAD[2]
 *   0x1    PAD[2]   PAD[3]   PAD[1]
 *   0x2    PAD[3]   PAD[1]   PAD[2]
 *   0x3    PAD[0]   PAD[3]   PAD[1]
 *
 * Four rows out of the 24 orderings of three distinct pads: the triple
 * is what the silicon offers, and a wiring that does not land on a row
 * is a wiring this peripheral cannot serve - which is worth knowing at
 * COMPILE time and not on a scope.
 */
constexpr std::optional<uint8_t> spi_dopo_for(SercomPad dout, SercomPad sck, SercomPad ss) {
    using P = SercomPad;
    if (dout == P::pad0 && sck == P::pad1 && ss == P::pad2) {
        return static_cast<uint8_t>(SERCOM_SPIM_CTRLA_DOPO_PAD0_Val);
    }
    if (dout == P::pad2 && sck == P::pad3 && ss == P::pad1) {
        return static_cast<uint8_t>(SERCOM_SPIM_CTRLA_DOPO_PAD1_Val);
    }
    if (dout == P::pad3 && sck == P::pad1 && ss == P::pad2) {
        return static_cast<uint8_t>(SERCOM_SPIM_CTRLA_DOPO_PAD2_Val);
    }
    if (dout == P::pad0 && sck == P::pad3 && ss == P::pad1) {
        return static_cast<uint8_t>(SERCOM_SPIM_CTRLA_DOPO_PAD3_Val);
    }
    return {};
}

/// DIPO is simply the pad number (32.8.1).
constexpr uint8_t spi_dipo(SercomPad p) { return static_cast<uint8_t>(p); }

/// Is this pad layout one the silicon can produce at all - the DOPO row
/// and the two pins the peripheral always drives?
constexpr bool spi_pads_valid(const SpiPads& p) {
    return spi_dopo_for(p.data_out, p.sck, p.ss).has_value() &&
           p.data_out_pin.valid() && p.sck_pin.valid() &&
           (!p.has_data_in || p.data_in_pin.valid());
}

// =============================================================================
// The baud arithmetic (pure: no register is touched below this line)
// =============================================================================

/**
 * What SCK the BAUD register value `reg` produces at a reference rate,
 * in the SYNCHRONOUS regime the SPI puts the generator in (table 30-2):
 *
 *     f_SCK = f_ref / (2 x (BAUD + 1))
 *
 * BAUD is EIGHT bits here (32.8.3), not the USART's sixteen, so the
 * ladder is 128 rates: f_ref/2 down to f_ref/512.
 */
constexpr uint32_t spi_sck_hz(uint32_t ref_hz, uint8_t reg) {
    return ref_hz / (2u * (static_cast<uint32_t>(reg) + 1u));
}

/// The fastest and slowest SCK this generator can produce at `ref_hz`
/// (BAUD = 0 and BAUD = 255, table 30-2's own condition f_SCK <= f_ref/2).
constexpr uint32_t spi_max_sck_hz(uint32_t ref_hz) { return ref_hz / 2u; }
constexpr uint32_t spi_min_sck_hz(uint32_t ref_hz) { return ref_hz / 512u; }

/**
 * The BAUD register value for a bit rate:
 *
 *     BAUD = f_ref / (2 x f_SCK) - 1
 *
 * ROUNDED SO THE RESULT IS NEVER FASTER THAN ASKED. A requested SCK is
 * a device's datasheet CEILING far more often than a target, so the
 * division rounds UP and the produced rate lands at or below the
 * request; spi_sck_hz() says what it really is. Nullopt when the rate is
 * outside the generator's own range - and 0 is a LEGAL BAUD value here
 * (it is the fastest rate), so it cannot double as a refusal.
 *
 * No wide arithmetic is needed: 2 x f_SCK is checked against f_ref
 * BEFORE it is formed, which also keeps the product inside 32 bits.
 */
constexpr std::optional<uint8_t> spi_baud_reg(uint32_t ref_hz, uint32_t sck_hz) {
    if (ref_hz == 0u || sck_hz == 0u) {
        return {};
    }
    if (sck_hz > ref_hz / 2u) {
        return {};   // even BAUD = 0 is slower than this
    }
    const uint32_t divisor = 2u * sck_hz;
    const uint32_t ratio = (ref_hz + divisor - 1u) / divisor;   // ceil(f_ref / 2 f_SCK)
    if (ratio == 0u || ratio > 256u) {
        return {};   // BAUD would not fit the eight bits
    }
    return static_cast<uint8_t>(ratio - 1u);
}

// =============================================================================
// Flags and status
// =============================================================================

/// INTFLAG, INTENSET and INTENCLR share one bit layout (32.8.4 - 32.8.6)
/// and, on this core, ONE interrupt vector - the same shape the USART
/// has, and for the same reason: SERCOMn_IRQn is one line.
struct SpiFlag {
    SpiFlag() = delete;

    static constexpr uint8_t dre = static_cast<uint8_t>(SERCOM_SPIM_INTFLAG_DRE_Msk);
    static constexpr uint8_t txc = static_cast<uint8_t>(SERCOM_SPIM_INTFLAG_TXC_Msk);
    static constexpr uint8_t rxc = static_cast<uint8_t>(SERCOM_SPIM_INTFLAG_RXC_Msk);
    static constexpr uint8_t ssl = static_cast<uint8_t>(SERCOM_SPIM_INTFLAG_SSL_Msk);
    static constexpr uint8_t error = static_cast<uint8_t>(SERCOM_SPIM_INTFLAG_ERROR_Msk);
    static constexpr uint8_t all = static_cast<uint8_t>(dre | txc | rxc | ssl | error);
};

/// STATUS (32.8.7) has exactly one bit on this peripheral.
struct SpiStatus {
    SpiStatus() = delete;

    static constexpr uint16_t overflow = SERCOM_SPIM_STATUS_BUFOVF_Msk;
    static constexpr uint16_t receive_errors = overflow;
};

// =============================================================================
// The configuration
// =============================================================================

/// Everything one instance is configured with. `baud` is the BAUD
/// REGISTER value - spi_baud_reg() computes it from a reference rate and
/// an SCK, exactly as on the USART side: the resource speaks the
/// register, the task speaks hertz.
struct SpiConfig {
    SpiPads pads{};
    SpiRole role = SpiRole::host;
    SpiMode mode = SpiMode::mode0;
    SpiForm form = SpiForm::spi;
    SpiCharSize bits = SpiCharSize::eight;

    /// CTRLA.DORD. SPI is MSB-first by convention AND by reset value, so
    /// unlike the USART's frame this default needs no argument.
    bool lsb_first = false;

    /// CTRLB.RXEN. A host with the receiver off gets no RXC and cannot
    /// use the byte-per-interrupt pump; it also frees the DI pad
    /// (32.5.1).
    bool receiver = true;

    /// HOST only, CTRLB.MSSEN: the peripheral drives the SS pad itself.
    /// READ 32.6.3.5 BEFORE ASKING FOR IT - hardware SS raises the line
    /// between EVERY character ("The SS pin will always be driven high
    /// for a minimum of one baud cycle between each data sent"), so a
    /// device that needs one select window around a MULTI-BYTE command
    /// cannot be driven this way at all. Software chip select on an
    /// ordinary GPIO is what SpiHost's Request carries, and it is the
    /// arrangement 32.6.3.3 calls "host with several clients".
    bool hardware_ss = false;

    /// CLIENT only, CTRLB.SSDE: a high-to-low transition on SS raises
    /// INTFLAG.SSL and can wake the device (32.6.3.6).
    bool ss_low_detect = false;

    /// CLIENT only, CTRLB.PLOADEN: load the shift register while SS is
    /// high, so the first character of a transaction is the answer and
    /// not the shifter's leftover (32.6.3.2).
    ///
    /// TWO STANDING OBLIGATIONS THIS DRIVER CANNOT ENFORCE, both from
    /// errata live on every revision. 1.17.3: the HOST must keep SS low
    /// until the end of the transmission or the first character is a
    /// dummy anyway - what the other board does is not knowable here.
    /// 1.17.20: a character left preloaded on the way into standby costs
    /// extra current. Neither has a workaround; both are stated.
    bool preload = false;

    /// CLIENT only, and only with `form` = with_address (32.6.3.1).
    SpiAddressMode address_mode = SpiAddressMode::mask;
    uint8_t address = 0;        ///< ADDR.ADDR
    uint8_t address_mask = 0;   ///< ADDR.ADDRMASK

    /// CTRLA.IBON: raise STATUS.BUFOVF at the overflow instead of
    /// letting it travel with the data through the receive buffer
    /// (32.6.2.7).
    bool immediate_overflow = false;

    /// CTRLA.RUNSTDBY. 32.6.5 gives it four different meanings across
    /// the two roles; the one that matters to a client is that with the
    /// bit CLEAR "all reception will be dropped, including the ongoing
    /// transaction".
    bool run_standby = false;

    /// DBGCTRL.DBGSTOP: halt the baud generator when a debugger halts
    /// the CPU. Erratum 1.17.19 (live) resets this register on every
    /// CTRLA.SWRST although 32.6.2.2 says a reset spares it, so
    /// configure() writes it last.
    bool debug_stop = false;

    /// HOST only: the BAUD register value. Ignored by a client, which is
    /// clocked by the host's SCK (32.6.2.3).
    uint8_t baud = 0;
};

/**
 * A bare configuration over one pad layout in one role - the value the
 * TASKS check their pads against at compile time.
 *
 * A pad layout is not legal or illegal by itself: the role decides which
 * signal each pad carries (table 32-2), so the same four wires that are
 * a perfectly good host are a broken client, and only spi_config_valid()
 * knows the difference. This makes that question askable without
 * inventing a second, weaker validity function beside it.
 */
constexpr SpiConfig spi_role_probe(const SpiPads& p, SpiRole r) {
    SpiConfig c{};
    c.pads = p;
    c.role = r;
    return c;
}

/// Which enumerators really exist - a cast can put a Reserved code in an
/// enum class, and a Reserved code written into a register is not a
/// refusal, it is undefined silicon.
constexpr bool spi_form_valid(SpiForm f) {
    return f == SpiForm::spi || f == SpiForm::with_address;
}
constexpr bool spi_char_size_valid(SpiCharSize c) {
    return c == SpiCharSize::eight || c == SpiCharSize::nine;
}
constexpr bool spi_address_mode_valid(SpiAddressMode m) {
    return m == SpiAddressMode::mask || m == SpiAddressMode::two_addresses ||
           m == SpiAddressMode::range;
}

/// Is the SS PAD the peripheral's? A client is selected by it and always
/// needs it; a host only claims it under hardware SS control.
constexpr bool spi_ss_pad_claimed(const SpiConfig& c) {
    return c.role == SpiRole::client || c.hardware_ss;
}

/**
 * Is this configuration one the silicon and the chapter allow?
 *
 * Each clause names WHOSE rule it is, because the two kinds do not
 * travel together: a SILICON refusal is a register that cannot hold the
 * value, a DRIVER refusal is this file declining an arrangement the
 * register would accept and no section of ch. 32 describes.
 */
constexpr bool spi_config_valid(const SpiConfig& c) {
    // --- silicon: the pad triple, the reserved codes ---------------------
    if (!spi_pads_valid(c.pads)) return false;
    if (!spi_form_valid(c.form)) return false;
    if (!spi_char_size_valid(c.bits)) return false;
    if (!spi_address_mode_valid(c.address_mode)) return false;

    // --- silicon: which knob belongs to which role -----------------------
    // 32.8.2 gives MSSEN to the host ("Host SPI Select Enable") and
    // PLOADEN/SSDE to the client; 32.8.1's FORM "selects the various
    // frame formats supported by the SPI in CLIENT mode", and 32.6.3.1's
    // address recognition is a client's alone.
    if (c.role == SpiRole::client && c.hardware_ss) return false;
    if (c.role == SpiRole::host && (c.preload || c.ss_low_detect)) return false;
    if (c.role == SpiRole::host && c.form == SpiForm::with_address) return false;

    // --- silicon: 32.6.3.1's own sentence --------------------------------
    // "Preload must be disabled (CTRLB.PLOADEN=0) in order to use this
    // mode." The two features share the client's first character and
    // cannot both own it.
    if (c.form == SpiForm::with_address && c.preload) return false;

    // --- silicon: the SS pin must be real when the peripheral drives or
    //     reads it ---------------------------------------------------------
    if (spi_ss_pad_claimed(c) && !c.pads.ss_pin.valid()) return false;

    // --- silicon: a client has to be clocked and selected ----------------
    // Both are INPUTS it cannot do without, and a client that cannot
    // receive can never see the character it is supposed to answer.
    if (c.role == SpiRole::client && !c.pads.has_data_in) return false;

    // --- driver: the input pad ------------------------------------------
    // DI on the SCK pad would sample the clock, which no arrangement in
    // 32.6.3 describes; DI on a CLAIMED SS pad would have the peripheral
    // read data off the line it is selected by. DI == DO is NOT refused:
    // that is loop-back, and 32.6.3.4 names it ("configure DIPO and DOPO
    // to use the same data pins ... the loop-back is through the pad").
    if (c.pads.has_data_in) {
        if (c.pads.data_in == c.pads.sck) return false;
        if (spi_ss_pad_claimed(c) && c.pads.data_in == c.pads.ss) return false;
    }
    return true;
}

// =============================================================================
// CTRLA / CTRLB, as values
// =============================================================================

/// CTRLA for an SPI configuration - ENABLE deliberately NOT included:
/// the whole register is written while the instance is disabled (it is
/// enable-protected), and enable() sets the bit afterwards in a store of
/// its own. Callers reach this through configure(); it is public because
/// a suite that wants to know what a configuration WOULD write should
/// not have to guess.
constexpr uint32_t spi_ctrla(const SpiConfig& c) {
    const uint8_t dopo = spi_dopo_for(c.pads.data_out, c.pads.sck, c.pads.ss).value_or(0);
    return SERCOM_SPIM_CTRLA_MODE(static_cast<uint32_t>(c.role)) |
           SERCOM_SPIM_CTRLA_DOPO(dopo) |
           SERCOM_SPIM_CTRLA_DIPO(spi_dipo(c.pads.data_in)) |
           SERCOM_SPIM_CTRLA_FORM(static_cast<uint32_t>(c.form)) |
           (spi_cpol(c.mode) ? SERCOM_SPIM_CTRLA_CPOL_Msk : 0u) |
           (spi_cpha(c.mode) ? SERCOM_SPIM_CTRLA_CPHA_Msk : 0u) |
           (c.lsb_first ? SERCOM_SPIM_CTRLA_DORD_Msk : 0u) |
           (c.immediate_overflow ? SERCOM_SPIM_CTRLA_IBON_Msk : 0u) |
           (c.run_standby ? SERCOM_SPIM_CTRLA_RUNSTDBY_Msk : 0u);
}

/// CTRLB, receiver included.
constexpr uint32_t spi_ctrlb(const SpiConfig& c) {
    return SERCOM_SPIM_CTRLB_CHSIZE(static_cast<uint32_t>(c.bits)) |
           SERCOM_SPIM_CTRLB_AMODE(static_cast<uint32_t>(c.address_mode)) |
           (c.receiver ? SERCOM_SPIM_CTRLB_RXEN_Msk : 0u) |
           (c.hardware_ss ? SERCOM_SPIM_CTRLB_MSSEN_Msk : 0u) |
           (c.ss_low_detect ? SERCOM_SPIM_CTRLB_SSDE_Msk : 0u) |
           (c.preload ? SERCOM_SPIM_CTRLB_PLOADEN_Msk : 0u);
}

/// ADDR: the client's address and its mask, in one word (32.8.9).
constexpr uint32_t spi_addr(const SpiConfig& c) {
    return SERCOM_SPIM_ADDR_ADDR(static_cast<uint32_t>(c.address)) |
           SERCOM_SPIM_ADDR_ADDRMASK(static_cast<uint32_t>(c.address_mask));
}

// =============================================================================
// The resource
// =============================================================================

/**
 * One SERCOM instance seen as an SPI, either role.
 *
 * The instance answers nothing until BOTH its clocks run: the APB bus
 * clock (MCLK, or the registers do not respond) and the core clock (a
 * GCLK peripheral channel, or nothing ever synchronizes - and, on a
 * host, nothing generates SCK either). The order the tasks below use is
 * bus, core, reset, configure, enable, pads - and it is that order for
 * those reasons, the pads LAST so a half-configured peripheral never
 * reaches the wire.
 *
 * Everything per-INSTANCE is Sercom<n>'s and is simply forwarded here:
 * one table of which GCLK channel and which NVIC line an instance uses,
 * shared by both personalities.
 */
template <uint8_t n>
class Spi {
    using Base = Sercom<n>;

public:
    Spi() = delete;

    static constexpr uint8_t index = n;

    // ---- where this instance lives (all Sercom<n>'s) -----------------------

    static sercom_spim_registers_t& regs() { return Base::spi_regs(); }
    static constexpr uint8_t gclk_core_id() { return Base::gclk_core_id(); }
    static constexpr uint32_t apb_mask() { return Base::apb_mask(); }
    static constexpr IRQn_Type irq() { return Base::irq(); }
    static constexpr uint8_t dma_rx_trigger() { return Base::dma_rx_trigger(); }
    static constexpr uint8_t dma_tx_trigger() { return Base::dma_tx_trigger(); }
    /// DATA is the only register a transfer ever touches (32.6.4.1).
    static volatile void* data_address() { return &regs().SERCOM_DATA; }

    // ---- clocks ------------------------------------------------------------

    static void bus_clock(bool on) { Base::bus_clock(on); }
    static bool core_clock(uint8_t generator) { return Base::core_clock(generator); }

    // ---- synchronization (32.6.6, 32.8.8) ----------------------------------

    static bool sync_busy(uint32_t mask) { return (regs().SERCOM_SYNCBUSY & mask) != 0u; }

    /// Bounded, like every wait in this stratum: a synchronization that
    /// never completes is reported, never hung on.
    static bool wait_sync(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().SERCOM_SYNCBUSY, mask, false, spins);
    }

    // ---- reset and enable (32.6.2.2) ---------------------------------------

    static bool enabled() {
        return (regs().SERCOM_CTRLA & SERCOM_SPIM_CTRLA_ENABLE_Msk) != 0u;
    }

    /**
     * Everything back to its reset value, instance disabled.
     *
     * ERRATUM 1.17.16: SWRST does nothing while ENABLE = 0 - exactly the
     * state a freshly booted instance is in, so the obvious "reset
     * first, then configure" would silently do nothing. A disabled
     * instance is therefore ENABLED first and reset from there.
     * THE BENCH COULD NOT REPRODUCE THE ITEM in SPI mode at rev F
     * (test_samc_spi a: SWRST from the disabled state resets the block,
     * synchronization completing) - the discipline is kept anyway: the
     * sheet marks every revision, other SERCOM modes are unmeasured, and
     * the cost is one enable.
     *
     * IT IS ENABLED IN SPI HOST MODE, and the mode is the point: the
     * enable synchronizes against GCLK_SERCOMx_CORE, and only a host
     * uses that clock. A client is clocked by an external SCK which the
     * other board may not be driving at all, so enabling in client mode
     * to satisfy the erratum could leave SYNCBUSY.ENABLE standing for
     * ever. No pad is muxed to the SERCOM at this point in either task's
     * init(), so the brief enable is invisible outside the chip.
     *
     * ERRATUM 1.17.19: this reset also clears DBGCTRL, which 32.6.2.2
     * says it should not. configure() writes DBGCTRL afterwards.
     */
    static bool reset(uint32_t spins = 0xFFFFu) {
        if (!enabled()) {
            regs().SERCOM_CTRLA =
                SERCOM_SPIM_CTRLA_MODE(SERCOM_SPIM_CTRLA_MODE_SPI_MASTER_Val) |
                SERCOM_SPIM_CTRLA_ENABLE_Msk;
            if (!wait_sync(SERCOM_SPIM_SYNCBUSY_ENABLE_Msk, spins)) {
                return false;
            }
        }
        regs().SERCOM_CTRLA = SERCOM_SPIM_CTRLA_SWRST_Msk;
        return wait_sync(SERCOM_SPIM_SYNCBUSY_SWRST_Msk, spins);
    }

    /**
     * ENABLE, waiting out BOTH synchronizations it triggers.
     *
     * The second one is easy to miss and is 32.8.2's own sentence:
     * enabling the peripheral CLEARS CTRLB.RXEN and raises
     * SYNCBUSY.CTRLB until the receiver is really up. A driver that
     * returned as soon as SYNCBUSY.ENABLE cleared would hand back a
     * receiver that is not enabled yet - and on this peripheral the
     * receiver is what raises RXC, i.e. the whole byte pump.
     */
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint32_t v = regs().SERCOM_CTRLA;
        regs().SERCOM_CTRLA = on ? (v | SERCOM_SPIM_CTRLA_ENABLE_Msk)
                                 : (v & ~SERCOM_SPIM_CTRLA_ENABLE_Msk);
        bool ok = wait_sync(SERCOM_SPIM_SYNCBUSY_ENABLE_Msk, spins);
        ok = wait_sync(SERCOM_SPIM_SYNCBUSY_CTRLB_Msk, spins) && ok;
        return ok;
    }

    // ---- configuration ------------------------------------------------------

    /**
     * Write the whole SPI configuration. CTRLA, CTRLB, BAUD and ADDR are
     * enable-protected (32.6.2.1) - a write while the instance runs is
     * DISCARDED, not refused - so this disables first and leaves the
     * instance disabled: enable(true) is a separate, deliberate step.
     *
     * False (and nothing programmed) for a configuration this silicon or
     * this chapter does not allow, or when a synchronization did not
     * complete. The PORT side of the pads is NOT touched here: pads are
     * the SERCOM's, pins are PORT's and the task's.
     */
    static bool configure(const SpiConfig& cfg, uint32_t spins = 0xFFFFu) {
        if (!spi_config_valid(cfg)) {
            return false;
        }
        if (!enable(false, spins)) {
            return false;
        }
        regs().SERCOM_INTENCLR = SpiFlag::all;
        regs().SERCOM_CTRLA = spi_ctrla(cfg);
        regs().SERCOM_CTRLB = spi_ctrlb(cfg);
        // Writing CTRLB again before this clears is an APB ERROR
        // (32.8.8) - the one synchronization here that is not merely
        // about knowing when a setting took effect.
        if (!wait_sync(SERCOM_SPIM_SYNCBUSY_CTRLB_Msk, spins)) {
            return false;
        }
        regs().SERCOM_BAUD = cfg.baud;
        regs().SERCOM_ADDR = spi_addr(cfg);
        // LAST, and after any reset: erratum 1.17.19 has SWRST clearing
        // this register although 32.6.2.2 exempts it.
        regs().SERCOM_DBGCTRL =
            cfg.debug_stop ? static_cast<uint8_t>(SERCOM_SPIM_DBGCTRL_DBGSTOP_Msk) : 0u;
        clear_status(SpiStatus::receive_errors);
        regs().SERCOM_INTFLAG = SpiFlag::all;
        return true;
    }

    /// The compile-time twin: a configuration this silicon cannot hold
    /// is a compile ERROR rather than a false at run time.
    template <SpiConfig cfg>
    static bool configure(uint32_t spins = 0xFFFFu) {
        static_assert(spi_pads_valid(cfg.pads),
                      "brio Spi: this pad layout is not one the silicon offers - "
                      "CTRLA.DOPO fixes the whole (DO, SCK, SS) TRIPLE and has only "
                      "four rows (32.8.1), and every pin named must be a real one");
        static_assert(spi_config_valid(cfg),
                      "brio Spi: this SPI configuration is refused - see "
                      "spi_config_valid() for which clause, each of which names "
                      "whether it is the silicon's rule or this driver's");
        return configure(cfg, spins);
    }

    static uint8_t baud_reg() { return regs().SERCOM_BAUD; }
    /// Enable-protected: the instance must be disabled or the store is
    /// discarded (32.6.2.1).
    static void baud_reg(uint8_t v) { regs().SERCOM_BAUD = v; }

    static uint32_t ctrla() { return regs().SERCOM_CTRLA; }
    static uint32_t ctrlb() { return regs().SERCOM_CTRLB; }
    static uint32_t addr_reg() { return regs().SERCOM_ADDR; }
    static uint8_t dbgctrl() { return regs().SERCOM_DBGCTRL; }

    /// The role in force, read back off CTRLA.MODE. Nullopt when the
    /// instance is not in an SPI mode at all (it may be a USART or an
    /// I2C, or freshly reset).
    static std::optional<SpiRole> role() {
        const uint32_t m = (regs().SERCOM_CTRLA & SERCOM_SPIM_CTRLA_MODE_Msk) >>
                           SERCOM_SPIM_CTRLA_MODE_Pos;
        if (m == static_cast<uint32_t>(SpiRole::host)) return SpiRole::host;
        if (m == static_cast<uint32_t>(SpiRole::client)) return SpiRole::client;
        return {};
    }

    /// The transfer mode in force (CTRLA.CPOL/CPHA).
    static SpiMode mode() {
        const uint32_t v = regs().SERCOM_CTRLA;
        return static_cast<SpiMode>(
            ((v & SERCOM_SPIM_CTRLA_CPOL_Msk) != 0u ? 0x2u : 0u) |
            ((v & SERCOM_SPIM_CTRLA_CPHA_Msk) != 0u ? 0x1u : 0u));
    }

    /**
     * CTRLB.RXEN alone, live.
     *
     * The one CTRLB bit that is NOT enable-protected (32.8.2), and the
     * one place this peripheral lets a running configuration change:
     * turning the receiver off flushes the receive buffer, drops what
     * was arriving and clears STATUS.BUFOVF. It is write-synchronized
     * "somewhat differently" (32.6.6's own words) - the bit reads back
     * as 1 only once the receiver is really up - so the wait is on
     * SYNCBUSY.CTRLB either way.
     */
    static bool receiver(bool on, uint32_t spins = 0xFFFFu) {
        if (!wait_sync(SERCOM_SPIM_SYNCBUSY_CTRLB_Msk, spins)) {
            return false;   // a CTRLB write on top of a pending one is an APB error
        }
        const uint32_t v = regs().SERCOM_CTRLB;
        regs().SERCOM_CTRLB = on ? (v | SERCOM_SPIM_CTRLB_RXEN_Msk)
                                 : (v & ~SERCOM_SPIM_CTRLB_RXEN_Msk);
        return wait_sync(SERCOM_SPIM_SYNCBUSY_CTRLB_Msk, spins);
    }
    static bool receiver() {
        return (regs().SERCOM_CTRLB & SERCOM_SPIM_CTRLB_RXEN_Msk) != 0u;
    }

    /// Hand the instance back: interrupts off, disabled, core clock
    /// released, bus clock off. The pads' PINS are the task's to release.
    static void release(uint32_t spins = 0xFFFFu) {
        regs().SERCOM_INTENCLR = SpiFlag::all;
        (void)enable(false, spins);
        GclkChannel::disconnect(gclk_core_id());
        bus_clock(false);
    }

    // ---- interrupts (32.8.4 - 32.8.6) ---------------------------------------

    /**
     * The flags that are BOTH raised and enabled - the one question a
     * shared vector has to ask, and the reason the answer cannot be
     * INTFLAG alone: DRE is a CONDITION, not an event, and reads 1
     * whenever the transmit buffer is free, which is most of the time.
     */
    [[gnu::always_inline]] static uint8_t pending() {
        return static_cast<uint8_t>(regs().SERCOM_INTFLAG & regs().SERCOM_INTENSET);
    }

    static uint8_t flags() { return regs().SERCOM_INTFLAG; }
    static uint8_t armed() { return regs().SERCOM_INTENSET; }

    /// The write-one-to-clear half of INTFLAG (TXC, SSL, ERROR). RXC and
    /// DRE are conditions and ignore a write: RXC is cleared by reading
    /// DATA, DRE by writing it (32.8.6).
    static void clear_flags(uint8_t mask) { regs().SERCOM_INTFLAG = mask; }

    /// INTENSET and INTENCLR are set-only and clear-only registers, so
    /// each of these is a PLAIN STORE of one bit: no read-modify-write
    /// to race with the handler.
    static void enable_interrupt(uint8_t mask, bool on) {
        if (on) {
            regs().SERCOM_INTENSET = mask;
        } else {
            regs().SERCOM_INTENCLR = mask;
        }
    }
    static void enable_dre_interrupt(bool on) { enable_interrupt(SpiFlag::dre, on); }
    static void enable_rxc_interrupt(bool on) { enable_interrupt(SpiFlag::rxc, on); }
    static void enable_txc_interrupt(bool on) { enable_interrupt(SpiFlag::txc, on); }
    static void enable_ssl_interrupt(bool on) { enable_interrupt(SpiFlag::ssl, on); }

    static bool dre_flag() { return (regs().SERCOM_INTFLAG & SpiFlag::dre) != 0u; }
    static bool rxc_flag() { return (regs().SERCOM_INTFLAG & SpiFlag::rxc) != 0u; }
    /// Host: the last character has been shifted out and DATA holds
    /// nothing new. Client: the host has raised SS (32.8.6).
    static bool txc_flag() { return (regs().SERCOM_INTFLAG & SpiFlag::txc) != 0u; }
    static bool ssl_flag() { return (regs().SERCOM_INTFLAG & SpiFlag::ssl) != 0u; }
    static bool error_flag() { return (regs().SERCOM_INTFLAG & SpiFlag::error) != 0u; }

    // ---- status and data -----------------------------------------------------

    [[gnu::always_inline]] static uint16_t status() { return regs().SERCOM_STATUS; }
    /// Write-one-to-clear, as a plain store of just those bits.
    static void clear_status(uint16_t mask) { regs().SERCOM_STATUS = mask; }
    static bool overflow_flag() { return (status() & SpiStatus::overflow) != 0u; }

    /// Reading DATA also clears INTFLAG.RXC; writing it clears
    /// INTFLAG.DRE and INTFLAG.TXC. Nine bits wide (32.8.10) - the
    /// character size decides how many of them mean anything.
    [[gnu::always_inline]] static uint16_t data() {
        return static_cast<uint16_t>(regs().SERCOM_DATA & SERCOM_SPIM_DATA_DATA_Msk);
    }
    [[gnu::always_inline]] static void data(uint16_t v) { regs().SERCOM_DATA = v; }

    /// Drain the two-level receive buffer (and the shifter's spill).
    static void flush_rx() {
        for (uint8_t i = 0; i < 4u && rxc_flag(); ++i) {
            (void)data();
        }
        clear_status(SpiStatus::receive_errors);
        clear_flags(SpiFlag::error);
    }
};

// =============================================================================
// The host task
// =============================================================================

/*
 * SpiHost<n, pads, generator>
 *
 * The transfer ENGINE: the target-side half of the SPI stack, driven by
 * util/spi_bus.hpp (which owns arbitration and replies). This task owns
 * the wire: chip select, the D/C line of display-style devices, and the
 * byte pump under the SERCOM interrupt.
 *
 * Transaction descriptor (Request) - two phases in ONE chip-select
 * window, the SAME shape avrdx/spi.hpp carries:
 *
 *   phase 1 (optional): cmd[cmd_len] transmitted with DC LOW
 *   phase 2 (optional): len bytes with DC HIGH, FULL-DUPLEX -
 *                       transmit tx[] (or 0xFF dummies if tx == null),
 *                       capture into rx[] (or discard if rx == null)
 *
 * CS is ACTIVE LOW and asserted/released by the engine around the whole
 * transaction; dc may be a null PinRef. Buffer ownership travels with
 * the request (Lease::reply: the client hands the spans off until its
 * SpiDone comes back). A zero-total-length request completes on the spot
 * without touching the wire.
 *
 * WHY THE CHIP SELECT IS A GPIO AND NOT THE PERIPHERAL'S. CTRLB.MSSEN
 * exists and this driver exposes it - but 32.6.3.5 says hardware SS is
 * raised "for a minimum of one baud cycle between each data sent", so it
 * frames one CHARACTER and not one TRANSACTION. Every device this engine
 * is for wants the second. So the Request carries a PinRef, exactly as
 * on the AVR, and 32.6.3.3's "host with several clients" is the
 * arrangement: MSSEN clear, one ordinary output per client.
 *
 * WHY RXC AND NOT DRE DRIVES THE PUMP. The two are one interrupt line
 * here, so the choice is which condition to arm. DRE means "the
 * transmit buffer is free" - true well before the character is on the
 * wire, and true continuously afterwards. RXC means "a character has
 * been fully shifted in", which on a full-duplex bus is exactly "one
 * character has moved, both ways" - and reading DATA to clear it is the
 * same access that captures the received byte. One interrupt per
 * character, with nothing to disambiguate. It does mean the receiver
 * must be enabled even for a write-only transfer, which costs the DI
 * pad and nothing else.
 *
 * NO PER-REQUEST CHIP-SELECT DELAY. avrdx/spi.hpp's Request carries
 * cs_setup_us, timed by avrdx/delay.hpp. This stratum HAS no delay
 * facility, and inventing one inside a driver is not where it belongs -
 * so the field is absent rather than approximated. A device that needs
 * settling time after CS falls spends it in its own AO (a time event, or
 * a zero-length request before the real one); the day a SAM device
 * really needs microseconds, samc/delay.hpp is the file that gets built.
 *
 * THE TWO OPTIONAL DMA ENGINE SLOTS (the Uart's shape, for the same
 * reason: an engineless build must stay byte-identical, so the slots
 * default to NoDmaEngine and every DMA branch folds away under
 * `if constexpr`). With DmaTxEngine/DmaRxEngine named, the DATA PHASE
 * of every request runs on the DMAC: the RX channel drains DATA on the
 * RXC trigger, the TX channel feeds it on DRE, and the transaction is
 * complete when the RECEIVE block completes - the last character is on
 * the wire until it has been shifted back in, so RX completion is the
 * only edge that means "done", exactly the reason the byte pump arms
 * RXC and never DRE. The command phase stays on the byte pump (it is a
 * few bytes with a DC flip at its end); a null tx feeds 0xFF dummies
 * from a held source address, a null rx drains into a held sink cell.
 *
 * THE DMAC BLOCK IS THE APP'S: Dmac::init() once, before any engined
 * init() - the engines arm CHANNELS of a controller somebody else owns,
 * and a driver that reset the shared block would stop every other
 * channel in the program (the Uart's own contract, restated).
 *
 * THE KICK DOCTRINE INVERTS IN SPI MODE, and it was measured, not
 * assumed. The UART campaign's finding was that a channel armed while
 * the peripheral's request LEVEL is already high sees no beat (the
 * trigger latches on the RISE) and must be kicked. THIS mode's TX
 * request behaves as the opposite: ENABLING the channel with DRE
 * already standing fires the first beat by itself, the chain then
 * sustains on the per-character rises - and a kick on top of that
 * start is one EXTRA beat whose byte lands in a full transmit buffer
 * and is DISCARDED in silence (measured three ways on the loop-back
 * bench: with the kick, exactly one early character vanishes from the
 * wire - the kick's own - at every rate; without it the stream is
 * byte-exact). So launch_dma() starts the two channels and kicks
 * NOTHING; on silicon where the SPI request ever behaved the UART way
 * the bounded fault path below is what would fire, and the kick
 * question would reopen with that measurement in hand. RXC starts
 * clear (flushed) and is a clean rise under either reading. Both
 * engines' completions arrive on the DMAC's own vector, so AN APP THAT
 * NAMES ENGINES BINDS DMAC_Handler TOO (below) - polled requests
 * included: the spin waits on a flag that handler sets.
 *
 * A DMA TRANSFER ERROR (erratum 1.10.4's class) is the one failure this
 * otherwise ACK-less bus can detect: the request completes with
 * spi_dma_fault in status() instead of pretending, the engines'
 * faults() counters carry the bill, and CS is raised so the bus is
 * released either way.
 *
 * ISR wiring (app glue, as usual - one vector for the whole SERCOM):
 *   extern "C" void SERCOM1_Handler() {
 *       if (SpiHw::isr()) { brio::post<SpiBus>(brio::TransferDone{SpiHw::status()}); }
 *   }
 *   // engine users bind the DMAC's vector as well:
 *   extern "C" void DMAC_Handler() {
 *       while (const auto irq = brio::Dmac::take_pending()) {
 *           if (SpiHw::dma_isr(irq->channel, irq->flags)) {
 *               brio::post<SpiBus>(brio::TransferDone{SpiHw::status()});
 *           }
 *       }
 *   }
 */

/// The DMA transfer error as a BusDone status - the first engine-defined
/// code the bus_master contract reserves (bus_engine_status). SPI has no
/// wire-level failure, so on an engineless host status() is spi_ok by
/// construction and this code is unreachable.
inline constexpr uint8_t spi_dma_fault = bus_engine_status;

template <uint8_t n, SpiPads pads, uint8_t generator = 0,
          typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
class SpiHost {
    using S = Spi<n>;

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from samc/dmac.hpp, or NoDmaEngine (the default)");
    // BOTH OR NEITHER: the transaction's completion is the RECEIVE
    // block's (see the class comment), so a TX engine alone has no edge
    // to complete on, and an RX engine alone would race the byte pump
    // for DATA.
    static_assert(TxEngine::present == RxEngine::present,
                  "brio SpiHost: name both DMA engines or neither - the data phase "
                  "is full-duplex and its completion is the RECEIVE block's");
    static_assert(uart_engines_distinct<TxEngine, RxEngine>(),
                  "the two engines must ride two different DMA channels");

    static_assert(spi_pads_valid(pads),
                  "these SERCOM pads cannot carry an SPI host: CTRLA.DOPO fixes the "
                  "whole (DO, SCK, SS) triple and offers only four rows (32.8.1), "
                  "and every pin named must be a real one on this device");
    // The pads are checked IN THE ROLE, not on their own: the role is
    // what decides which signal each pad carries (table 32-2), so a
    // layout is legal or not only once the side of the wire is known.
    static_assert(spi_config_valid(spi_role_probe(pads, SpiRole::host)),
                  "these SERCOM pads cannot carry an SPI HOST - see "
                  "spi_config_valid() for which clause refused them, and whether "
                  "it is the silicon's rule or this driver's");

    using DoPin = Pin<pads.data_out_pin.port, pads.data_out_pin.pin>;
    using SckPin = Pin<pads.sck_pin.port, pads.sck_pin.pin>;
    using DiPin = Pin<pads.data_in_pin.port, pads.data_in_pin.pin>;

public:
    SpiHost() = delete;

    using Resource = S;
    static constexpr SpiPads pin_pads = pads;

    /// The GCLK generator this task takes its core clock from. Generator
    /// 0 is CLK_MAIN undivided in this stratum (samc/clock.hpp says so,
    /// and Clock::hz is that rate), which is what lets init() derive the
    /// baud divisor from the app's Clock tag alone.
    static constexpr uint8_t core_generator = generator;

    /// Whether the data phase rides the DMAC (the two engine slots).
    static constexpr bool has_engines = TxEngine::present;

    struct Request {
        PinRef cs;   ///< asserted low around the transaction
        PinRef dc;   ///< display D/C line; null = no such pin
        /// Phase 1, sent with DC low; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        uint8_t cmd_len;
        /// Phase 2 out, null = 0xFF dummies; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; LENT until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint16_t len;   ///< phase 2 length
        ReplyTo<SpiDone> reply;

        /// Per-transaction bus configuration. On a SHARED bus every
        /// device names its own speed and mode in the request, and the
        /// engine reprograms the peripheral at each start() - which
        /// costs a disable/enable pair only when something CHANGED, and
        /// nothing per byte.
        ///
        /// THE RATE IS THE BAUD REGISTER VALUE, not an enum of
        /// divisions: this generator has 256 of them, and naming them
        /// would be inventing a vocabulary the silicon does not have.
        /// Use SpiHost::baud_for(hz) once, at the client's own init, and
        /// keep the byte. It is a DIVISION of the core clock, so it
        /// follows a clock change by itself exactly as the AVR's
        /// SpiClock does; max_sck_hz() is the ceiling that clamps it.
        uint8_t baud = 0;
        SpiMode mode = SpiMode::mode0;

        /// Completion style, the client's call: false = per-character
        /// ISR pump (the kernel keeps running between characters); true
        /// = POLLED inside start(), completing synchronously. At fast
        /// SCK the polled loop wins on every axis - a character at
        /// f_ref/2 is sixteen CPU cycles while an ISR entry alone costs
        /// more. The price is that THIS dispatch blocks for the whole
        /// transfer (bounded, chosen here); global interrupts stay
        /// enabled throughout - only the SERCOM's own RXC is silenced.
        bool polled = false;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle -----------------------------------------------------------

    /**
     * @brief Bring the instance up as a host: clocks, configuration,
     * pads, pins, the NVIC line.
     *
     * Call AFTER the main clock is set up and before interrupts are
     * enabled globally; `clock` is the app's brio::Clock tag, so the
     * baud arithmetic comes from Clock::hz and never from a second
     * statement of the rate.
     *
     * `max_sck_hz` is an optional CEILING for the whole bus: with it set
     * the engine slows any request that would exceed it, and re-resolves
     * the limit after a clock change, which is what makes rebase()
     * meaningful. 0 = no ceiling.
     *
     * False when the ceiling cannot be produced at this clock, or when
     * one of the peripheral's synchronizations did not complete.
     */
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        static_assert(clock_follows<Clock, SpiHost>(),
                      "this SpiHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its SCK ceiling would go stale on a "
                      "clock change");

        Nvic::disable(S::irq());
        ceiling_hz_ = max_sck_hz;
        rebase(clock_hz(clock));
        if (max_sck_hz != 0 && !ceiling_) {
            return false;   // not even the slowest BAUD honours it
        }

        S::bus_clock(true);
        if (!S::core_clock(generator)) {
            return false;
        }
        if (!S::reset()) {
            return false;
        }

        applied_ = boot_config();
        if (!S::configure(applied_)) {
            return false;
        }
        if (!S::enable(true)) {
            return false;
        }
        if constexpr (has_engines) {
            // Claim the two channels for this SERCOM's data register and
            // trigger codes. arm() also enables the DMAC's NVIC line -
            // the app's DMAC_Handler binding is part of taking engines.
            TxEngine::arm(S::data_address(), S::dma_tx_trigger());
            RxEngine::arm(S::data_address(), S::dma_rx_trigger());
        }
        status_ = spi_ok;

        // The pads go to the SERCOM only now, with the peripheral
        // already enabled and SCK sitting at its configured idle level:
        // handing PORT the pins first would park an undriven pad on the
        // bus, and a client watching SCK has no way to tell a glitch
        // from a clock edge.
        DoPin::function(pads.data_out_pin.function);
        SckPin::function(pads.sck_pin.function);
        if constexpr (pads.has_data_in) {
            DiPin::function(pads.data_in_pin.function, {.input_enable = true});
        }
        // The SS pad is claimed only under hardware SS control; the
        // software chip selects this engine really uses are ordinary
        // GPIOs, configured by the device clients that own them.
        S::flush_rx();
        Nvic::enable(S::irq());
        return true;
    }

    /**
     * @brief The core clock changed (DynamicClock fan-out).
     *
     * A Request's `baud` is a DIVISION of the core clock and scales with
     * it by itself, exactly like the AVR's SpiClock - so what is
     * recomputed here is only the ceiling. The bus must be IDLE: a
     * transfer in flight keeps its divisor for the remaining characters.
     */
    static void rebase(uint32_t hz) {
        ref_hz_ = hz;
        ceiling_ = ceiling_hz_ ? spi_baud_reg(hz, ceiling_hz_) : std::optional<uint8_t>{};
    }

    /// The BAUD value that produces at most `hz` of SCK at the core
    /// clock last seen - the chooser a device's datasheet limit is
    /// spoken to. Nullopt when this generator cannot go that slow (or
    /// when the request is faster than f_ref/2).
    static std::optional<uint8_t> baud_for(uint32_t hz) {
        return spi_baud_reg(ref_hz_, hz);
    }

    /// What SCK a request at this BAUD really runs at, ceiling included.
    static uint32_t sck_hz(uint8_t b) { return spi_sck_hz(ref_hz_, clamp(b)); }

    /// The ceiling in force (0 = none) and the BAUD value it resolves to.
    static uint32_t max_sck_hz() { return ceiling_hz_; }
    static std::optional<uint8_t> ceiling_baud() { return ceiling_; }

    /// The core clock rate this task last computed against.
    static uint32_t reference_hz() { return ref_hz_; }

    /**
     * @brief Put the peripheral at this mode and rate NOW, moving no data.
     *
     * FOR CALLERS THAT FRAME THE SELECT WINDOW THEMSELVES (Request.cs
     * null). start() applies a request's mode before it asserts the
     * request's own cs, so an engine-owned select window always opens
     * with SCK settled at the new idle level - but a caller driving CS by
     * hand inverts that order, and a mode change is a CPOL FLIP ON THE
     * WIRE: flipped inside an open select window it is one extra edge,
     * and a selected client counts it into the character (measured as a
     * one-bit slip in both directions, test_samc_spi c's first version).
     * Prime FIRST, then assert the select.
     */
    static void prime(SpiMode mode, uint8_t baud) { apply(mode, clamp(baud)); }

    // ---- the transfer --------------------------------------------------------

    /**
     * @brief Begin a transaction (called by SpiBus from main context).
     * @return true when the transaction completed SYNCHRONOUSLY (polled
     * requests, and the degenerate zero-length one); false when it runs
     * on the ISR and a TransferDone will follow. That is exactly
     * util/bus_master.hpp's engine contract.
     */
    static bool start(const Request& r) {
        req_ = r;
        pos_ = 0;
        in_cmd_ = (r.cmd_len > 0);
        status_ = spi_ok;
        if (total_len() == 0) {
            return true;   // nothing to move: complete on the spot
        }
        apply(r.mode, clamp(r.baud));
        if (in_cmd_) {
            r.dc.clear();
        } else {
            r.dc.set();
        }
        r.cs.clear();   // assert, active low
        S::flush_rx();  // a stale character would be captured as this one's

        if constexpr (has_engines) {
            if (!r.polled) {
                if (in_cmd_) {
                    // The command phase runs on the byte pump; isr()
                    // hands over to the engines at its end.
                    S::enable_rxc_interrupt(true);
                    S::data(req_.cmd.get()[0]);
                    return false;
                }
                launch_dma();
                return false;   // dma_isr() is the completion edge
            }
            // Polled with engines: the command phase spins per byte,
            // the data phase spins on the DMAC's completion - which
            // still arrives through DMAC_Handler / dma_isr(), so the
            // binding is not optional for polled requests either.
            S::enable_rxc_interrupt(false);
            for (uint8_t i = 0; i < r.cmd_len; ++i) {
                (void)xfer(r.cmd.get()[i]);
            }
            r.dc.set();
            if (r.len != 0) {
                launch_dma();
                spin_dma();
            }
            r.cs.set();
            return true;
        }

        if (!r.polled) {
            S::enable_rxc_interrupt(true);
            S::data(first_byte());   // the ISR pumps the rest
            return false;
        }
        // Polled pump: silence the SERCOM's own interrupt (the bound
        // handler would steal the characters) - global interrupts STAY
        // ENABLED, so the ticker and anything else preempt this loop
        // freely.
        S::enable_rxc_interrupt(false);
        // The loans are VIEWS: .get() hands out the raw pointer the
        // loops index (Borrowed is not a container).
        for (uint8_t i = 0; i < r.cmd_len; ++i) {
            (void)xfer(r.cmd.get()[i]);
        }
        r.dc.set();   // data phase (a no-op when len == 0)
        if (r.rx.get() == nullptr && r.tx.get() != nullptr) {          // bulk write
            const uint8_t* p = r.tx.get();
            for (uint16_t k = r.len; k != 0; --k) {
                (void)xfer(*p++);
            }
        } else if (r.rx.get() != nullptr && r.tx.get() == nullptr) {   // bulk read
            uint8_t* p = r.rx.get();
            for (uint16_t k = r.len; k != 0; --k) {
                *p++ = xfer(0xFF);
            }
        } else {                                                       // full duplex
            for (uint16_t i = 0; i < r.len; ++i) {
                const uint8_t in = xfer((r.tx.get() != nullptr) ? r.tx.get()[i] : 0xFF);
                if (r.rx.get() != nullptr) {
                    r.rx.get()[i] = in;
                }
            }
        }
        r.cs.set();   // release: transaction done
        return true;
    }

    /**
     * @brief SERCOM interrupt body - call from SERCOMn_Handler().
     *
     * ONE VECTOR, so the body starts by asking which source is both
     * raised AND enabled. Only RXC is ever armed by this engine (see the
     * class comment), and reading DATA is both the capture and the
     * acknowledgement.
     *
     * @return true when the transaction just completed (CS released):
     * the edge on which the app's glue posts TransferDone to the bus AO.
     */
    [[gnu::always_inline]] static bool isr() {
        if ((S::pending() & SpiFlag::rxc) == 0u) {
            return false;
        }
        const uint8_t in = static_cast<uint8_t>(S::data());

        if constexpr (has_engines) {
            // Only the COMMAND phase ever runs on this pump: at its end
            // the engines take the data phase and this interrupt goes
            // quiet. The received byte is the command's echo - discarded,
            // as the engineless path discards it too.
            (void)in;
            ++pos_;
            if (pos_ >= req_.cmd_len) {
                in_cmd_ = false;
                S::enable_rxc_interrupt(false);
                req_.dc.set();
                if (req_.len == 0) {
                    req_.cs.set();   // a command-only request: done here
                    return true;
                }
                launch_dma();
                return false;        // dma_isr() is the completion edge
            }
            S::data(req_.cmd.get()[pos_]);
            return false;
        }

        if (!in_cmd_ && req_.rx.get() != nullptr) {
            req_.rx.get()[pos_] = in;
        }
        ++pos_;

        if (in_cmd_ && pos_ >= req_.cmd_len) {
            in_cmd_ = false;
            pos_ = 0;
            req_.dc.set();   // command phase over
        }
        if (!in_cmd_ && pos_ >= req_.len) {
            S::enable_rxc_interrupt(false);
            req_.cs.set();   // release: transaction done
            return true;
        }
        S::data(next_byte());
        return false;
    }

    /**
     * @brief DMAC interrupt body - call from DMAC_Handler() with each
     * take_pending() result (engine builds only; on an engineless host
     * this compiles away).
     *
     * @return true when the transaction just completed (CS released):
     * the edge on which the glue posts TransferDone{status()}.
     *
     * A TRANSFER ERROR ON EITHER CHANNEL ENDS THE TRANSACTION with
     * spi_dma_fault: a TX block the silicon stopped running starves the
     * receive side for ever (the UART campaign's wedge, met here as a
     * bounded failure instead), and an RX error means the count can no
     * longer be trusted. Both channels are put away, CS is raised, and
     * the fault is REPORTED rather than retried - retry policy is the
     * bus AO's, not the engine's.
     */
    [[gnu::always_inline]] static bool dma_isr(uint8_t channel, uint8_t flags) {
        if constexpr (has_engines) {
            // take_pending() aligns the flags to bit 0 = TERR, the same
            // layout CHINTFLAG has - so the device header's own mask
            // asks the question without this file including dmac.hpp.
            const bool error = (flags & DMAC_CHINTFLAG_TERR_Msk) != 0u;
            if (channel == TxEngine::channel) {
                if (error) {
                    (void)TxEngine::abandon();
                    return finish_dma(spi_dma_fault);
                }
                (void)TxEngine::complete();
                return false;   // the transmit side never completes a transaction
            }
            if (channel == RxEngine::channel) {
                if (!dma_active_) {
                    return false;
                }
                return finish_dma(error ? spi_dma_fault : spi_ok);
            }
        }
        (void)channel;
        (void)flags;
        return false;
    }

    /// The engine's completion status, read by the app glue for the
    /// TransferDone payload. Always spi_ok on an engineless host.
    static uint8_t status() { return status_; }

    /// Hand the pins back, then the peripheral.
    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Nvic::disable(S::irq());
        S::release();
        DoPin::release();
        SckPin::release();
        if constexpr (pads.has_data_in) {
            DiPin::release();
        }
    }

private:
    // The engines carry BYTES: the Request is byte-oriented and the
    // beat is the element (dmac.hpp). Checked here so a wider engine is
    // refused at ITS spelling, not at a pointer mismatch three screens
    // down.
    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, uint8_t> &&
                   std::is_same_v<typename RxEngine::element, uint8_t>;
        } else {
            return true;
        }
    }(), "brio SpiHost: the DMA engines must carry uint8_t elements - the "
         "Request's buffers are bytes");

    /// Start the data phase on the two channels. The RECEIVE channel
    /// goes first (its trigger is a rise that has not happened yet);
    /// the transmit one starts SECOND AND IS NOT KICKED - in SPI mode
    /// the standing DRE level fires the first beat at the enable
    /// itself, and a kick would add a beat the full transmit buffer
    /// discards (the class comment carries the measurement).
    static void launch_dma() {
        dma_done_ = false;
        dma_active_ = true;
        if (req_.rx.get() != nullptr) {
            (void)RxEngine::start(req_.rx.get(), req_.len);
        } else {
            (void)RxEngine::start_discard(&rx_sink_, req_.len);
        }
        if (req_.tx.get() != nullptr) {
            (void)TxEngine::start(req_.tx.get(), req_.len);
        } else {
            (void)TxEngine::start_fixed(&tx_dummy_, req_.len);
        }
    }

    /// One exit for the data phase, from either flavour of dma_isr().
    /// @return true when the ISR-style caller should post completion.
    static bool finish_dma(uint8_t st) {
        if (st != spi_ok) {
            status_ = st;
            (void)TxEngine::abandon();   // re-claims: interrupts re-armed
            RxEngine::stop();
            // stop() disarms the channel's interrupts with it; the next
            // transaction needs the claim back.
            RxEngine::arm(S::data_address(), S::dma_rx_trigger());
        }
        dma_active_ = false;
        dma_done_ = true;
        if (!req_.polled) {
            req_.cs.set();
            return true;
        }
        return false;
    }

    /// The polled request's wait on the DMAC completion - bounded, like
    /// every wait in this stratum (the slowest character is 512 x 9
    /// core-clock cycles, and the budget scales with the length).
    static void spin_dma() {
        uint32_t spins = 200000u + 6000u * static_cast<uint32_t>(req_.len);
        while (!dma_done_ && spins-- != 0u) {
        }
        if (!dma_done_) {
            // Nothing completed inside a generous bound: the 1.10.4
            // class of death, or a clock that stopped. Put both
            // channels away and REPORT - never hang the dispatch.
            // abandon() re-claims by itself; the stopped receive
            // channel gets its claim (interrupts included) re-armed.
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address(), S::dma_rx_trigger());
            dma_active_ = false;
            status_ = spi_dma_fault;
        }
    }

    /// A slower BAUD is a LARGER register value, so the ceiling clamps
    /// from below.
    static uint8_t clamp(uint8_t b) {
        if (!ceiling_) {
            return b;
        }
        return b < *ceiling_ ? *ceiling_ : b;
    }

    static SpiConfig boot_config() {
        SpiConfig c{};
        c.pads = pads;
        c.role = SpiRole::host;
        c.mode = SpiMode::mode0;
        c.receiver = true;   // RXC is the pump: see the class comment
        c.baud = ceiling_ ? *ceiling_ : 0;
        return c;
    }

    /**
     * Put the peripheral where this request wants it.
     *
     * CTRLA (which is where CPOL and CPHA live here, unlike the AVR's
     * CTRLB) and BAUD are BOTH enable-protected, so any change costs a
     * disable/enable pair - which is why the applied state is cached and
     * the pair is paid only when something really moved. A run of
     * requests to one device costs nothing at all.
     */
    static void apply(SpiMode mode, uint8_t baud) {
        if (mode == applied_.mode && baud == applied_.baud) {
            return;
        }
        applied_.mode = mode;
        applied_.baud = baud;
        (void)S::enable(false);
        S::regs().SERCOM_CTRLA = spi_ctrla(applied_);
        S::baud_reg(baud);
        (void)S::enable(true);
    }

    /// One polled character: write, spin on RXC (which is the character
    /// having been fully shifted BOTH ways), read back. The spin is
    /// bounded - a bus whose clock never runs must not hang the kernel -
    /// and the bound is generous: the slowest character this generator
    /// can produce is 512 x 9 core-clock cycles.
    static uint8_t xfer(uint8_t out) {
        S::data(out);
        uint32_t spins = 200000u;
        while (!S::rxc_flag() && spins-- != 0u) {
        }
        return static_cast<uint8_t>(S::data());
    }

    static uint16_t total_len() {
        return static_cast<uint16_t>(req_.cmd_len) + req_.len;
    }

    static uint8_t first_byte() { return in_cmd_ ? req_.cmd.get()[0] : data_byte(0); }
    static uint8_t next_byte() { return in_cmd_ ? req_.cmd.get()[pos_] : data_byte(pos_); }
    static uint8_t data_byte(uint16_t i) {
        return (req_.tx.get() != nullptr) ? req_.tx.get()[i] : 0xFF;
    }

    static inline Request req_{};
    static inline uint16_t pos_ = 0;
    static inline bool in_cmd_ = false;
    /// The last completion's status (spi_ok / spi_dma_fault). Plain: it
    /// is written before the completion edge and read after it.
    static inline uint8_t status_ = spi_ok;
    /// ISR-written, thread-polled (the polled DMA spin): volatile, the
    /// ticker doctrine.
    static inline volatile bool dma_done_ = false;
    static inline volatile bool dma_active_ = false;
    /// The write-only transfer's discard cell and the read-only one's
    /// dummy source (see launch_dma).
    static inline uint8_t rx_sink_ = 0;
    static constexpr uint8_t tx_dummy_ = 0xFF;
    /// The configuration really in the registers - the cache `apply()`
    /// compares against, and the thing a re-init overwrites wholesale.
    static inline SpiConfig applied_{};
    static inline uint32_t ref_hz_ = 0;
    static inline uint32_t ceiling_hz_ = 0;
    static inline std::optional<uint8_t> ceiling_{};
};

// =============================================================================
// The client task
// =============================================================================

/*
 * SpiClient<n, pads>
 *
 * The other end of the wire (32.6.2.6.2): SS, MOSI and SCK are inputs,
 * the host sets the pace, and the only thing this side controls is WHAT
 * it has ready to shift out when the next clock arrives.
 *
 * THE THREE-CYCLE RULE IS THE WHOLE STORY. "After DATA is written it
 * takes up to three SCK clock cycles until the content of DATA is ready
 * to be loaded into the Shift register on the next character boundary.
 * As a consequence, the first character transferred in a SPI transaction
 * will not be the content of DATA" - and, for the characters after it,
 * "the data has to be written into DATA register with at least three SCK
 * clock cycles left in the current character transmission. If this
 * criteria is not met, the previously received character will be
 * transmitted." So a client that answers late does not drop a character:
 * it ECHOES the one it just received, which is a distinctive enough
 * signature to recognise on the host's side.
 *
 * PRELOADING (32.6.3.2, CTRLB.PLOADEN) is the cure for the FIRST
 * character: with it, a DATA write made while SS is high goes straight
 * into the shift register, so the transaction opens with the answer
 * rather than the shifter's leftover. Exactly ONE character is preloaded
 * that way; a second write waits in DATA. Erratum 1.17.3 puts an
 * obligation on the HOST for this to hold (SS low for the whole
 * transmission) and no driver on this side can enforce it.
 *
 * AND THE PUMP MUST RUN ONE AHEAD. The three SCK cycles above elapse
 * only WHILE SCK RUNS, so an answer written in the inter-character gap -
 * where a poll loop reacting to RXC lands - matures mid-character and
 * reaches the wire one character LATE (measured: the host reads the
 * preload exactly, then the whole stream slips by one). The working
 * shape is preload b0, park b1 in DATA at once, and on every received
 * character write the next-PLUS-ONE: each value is then in place a whole
 * character early, and the three cycles are already spent when its
 * boundary comes.
 *
 * THERE IS NO DEMOTION HERE. On the AVR a host that sees its SS pin
 * pulled low becomes a client on the spot, and avrdx/spi.hpp has to
 * report and undo it. This peripheral has nothing of the kind: the role
 * is CTRLA.MODE, written while the SERCOM is disabled, and a low SS on a
 * host with MSSEN = 0 is simply a pin nobody is looking at. What the
 * silicon offers instead is CTRLB.SSDE - a client that WAKES on the
 * select edge - which is a different feature answering a different
 * question.
 *
 * The surface is polled plus ISR bodies, deliberately thin: a client is
 * a protocol, and which protocol is the application's. ISR wiring is the
 * app's as always:
 *   extern "C" void SERCOM1_Handler() { Peer::isr(); }
 */
template <uint8_t n, SpiPads pads>
class SpiClient {
    using S = Spi<n>;

    static_assert(spi_pads_valid(pads),
                  "these SERCOM pads cannot carry an SPI client: CTRLA.DOPO fixes the "
                  "whole (DO, SCK, SS) triple and offers only four rows (32.8.1)");
    // The pads are checked IN THE ROLE. This is the assertion that
    // catches the mistake the two roles invite on ONE harness: keeping
    // the host's DOPO row when the directions swap, which points the
    // answer line at the host's own MOSI.
    static_assert(spi_config_valid(spi_role_probe(pads, SpiRole::client)),
                  "these SERCOM pads cannot carry an SPI CLIENT - on a client DO is "
                  "MISO and DI is MOSI (table 32-2), so a harness that is a host on "
                  "one DOPO row is a client on a DIFFERENT one");
    static_assert(pads.has_data_in,
                  "an SPI client must be able to RECEIVE: DI is its MOSI, and a client "
                  "that cannot hear the host has nothing to answer");
    static_assert(pads.ss_pin.valid(),
                  "an SPI client is SELECTED by its SS pad (table 32-2), so the pin "
                  "that pad is bonded to must be a real one");

    using DoPin = Pin<pads.data_out_pin.port, pads.data_out_pin.pin>;
    using SckPin = Pin<pads.sck_pin.port, pads.sck_pin.pin>;
    using SsPin = Pin<pads.ss_pin.port, pads.ss_pin.pin>;
    using DiPin = Pin<pads.data_in_pin.port, pads.data_in_pin.pin>;

public:
    SpiClient() = delete;

    using Resource = S;
    static constexpr SpiPads pin_pads = pads;
    static constexpr uint8_t core_generator = 0;

    /// What this end is configured as. Everything a client can be told
    /// that a host cannot; the pads and the role are the task's.
    struct Config {
        SpiMode mode = SpiMode::mode0;
        bool lsb_first = false;
        SpiCharSize bits = SpiCharSize::eight;
        /// 32.6.3.2: open a transaction with the answer instead of the
        /// shifter's leftover. Errata 1.17.3 and 1.17.20 travel with it -
        /// see SpiConfig::preload.
        bool preload = true;
        /// 32.6.3.6: raise INTFLAG.SSL on the select edge (and wake).
        bool ss_low_detect = false;
        /// Address recognition (32.6.3.1): the first character of a
        /// transaction is matched against ADDR and the whole transaction
        /// is IGNORED when it does not match. Mutually exclusive with
        /// `preload`, by the chapter's own sentence.
        bool address_match = false;
        SpiAddressMode address_mode = SpiAddressMode::mask;
        uint8_t address = 0;
        uint8_t address_mask = 0;
        bool immediate_overflow = false;
        bool run_standby = false;
        /// Whether the DO (MISO) pad is handed to the SERCOM at all. A
        /// client on a shared harness that must stay DARK - never
        /// driving the answer line until it has decided to - leaves this
        /// false and calls drive_output(true) for the one window it
        /// answers in.
        bool drive_output = true;
    };

    /**
     * @brief Bring the instance up as a client.
     *
     * The core clock is still required even though the SHIFT register
     * runs on the host's SCK: 32.5.3 says GCLK_SERCOMx_CORE "is required
     * to clock the SPI", and every synchronization in the peripheral
     * crosses into it - a client without it never finishes enabling.
     *
     * False when the configuration is refused or a synchronization did
     * not complete.
     */
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg = {}) {
        (void)clock;
        Nvic::disable(S::irq());
        S::bus_clock(true);
        if (!S::core_clock(core_generator)) {
            return false;
        }
        if (!S::reset()) {
            return false;
        }
        if (!S::configure(config_of(cfg))) {
            return false;
        }
        if (!S::enable(true)) {
            return false;
        }
        // Inputs first, output last: a pad handed over before the
        // peripheral is up would drive whatever the shifter happens to
        // hold onto a bus the host may already be using.
        SckPin::function(pads.sck_pin.function, {.input_enable = true});
        SsPin::function(pads.ss_pin.function, {.input_enable = true});
        DiPin::function(pads.data_in_pin.function, {.input_enable = true});
        drive_output(cfg.drive_output);
        S::flush_rx();
        Nvic::enable(S::irq());
        return true;
    }

    /// Hand the answer line to the SERCOM, or take it back. A DARK
    /// listener on a shared harness (one where another board also drives
    /// the line at times) keeps it false and opens it for exactly the
    /// window it answers in. Taking it back leaves the pad an ordinary
    /// input: PMUXEN off, no pull, nothing driven.
    static void drive_output(bool on) {
        if (on) {
            DoPin::function(pads.data_out_pin.function);
        } else {
            DoPin::release();
            DoPin::configure({.input_enable = true});
        }
    }

    /// Is the host holding SS low right now? A live pin read - the SERCOM
    /// publishes no such status bit, which is why the pad keeps its
    /// input buffer on.
    static bool selected() { return !SsPin::read(); }

    // ---- the byte surface -----------------------------------------------------

    /// Load the next character to shift out. While SS is HIGH and
    /// CTRLB.PLOADEN is set this reaches the shift register directly
    /// (32.6.3.2); otherwise it lands in DATA and takes up to three SCK
    /// cycles to be usable.
    static void write(uint16_t v) { S::data(v); }
    static bool writable() { return S::dre_flag(); }

    /// One received character, or nothing. Reading DATA is what clears
    /// RXC, so this is also the acknowledgement.
    static std::optional<uint16_t> poll() {
        if (!S::rxc_flag()) {
            return {};
        }
        return S::data();
    }

    static bool overflow() { return S::overflow_flag(); }
    static void clear_overflow() {
        S::clear_status(SpiStatus::overflow);
        S::clear_flags(SpiFlag::error);
    }
    /// Client-side TXC: the host has raised SS, i.e. the transaction is
    /// over (32.8.6). With address matching on it is set only for a
    /// transaction that matched.
    static bool transaction_done() { return S::txc_flag(); }
    static void clear_transaction_done() { S::clear_flags(SpiFlag::txc); }
    /// The select edge, when CTRLB.SSDE asked for it (32.6.3.6).
    static bool select_edge() { return S::ssl_flag(); }
    static void clear_select_edge() { S::clear_flags(SpiFlag::ssl); }

    static uint8_t flags() { return S::flags(); }

    // ---- the ISR bodies --------------------------------------------------------

    /**
     * @brief The instance's ONE interrupt body - call from
     * SERCOMn_Handler().
     *
     * A client's protocol is the application's, so this body does not
     * decide anything: it reports WHICH sources are both raised and
     * enabled and lets the app's glue act. The received character is not
     * consumed here - reading DATA is the app's, because only the app
     * knows where the character goes.
     *
     * @return the pending mask (SpiFlag::rxc | ::txc | ::ssl | ...), 0
     * when this SERCOM was not the one asking.
     */
    [[gnu::always_inline]] static uint8_t isr() { return S::pending(); }

    static void enable_rxc_interrupt(bool on) { S::enable_rxc_interrupt(on); }
    static void enable_txc_interrupt(bool on) { S::enable_txc_interrupt(on); }
    static void enable_ssl_interrupt(bool on) { S::enable_ssl_interrupt(on); }

    /// Hand the pins back, then the peripheral.
    static void release() {
        Nvic::disable(S::irq());
        S::release();
        DoPin::release();
        SckPin::release();
        SsPin::release();
        DiPin::release();
    }

private:
    static SpiConfig config_of(const Config& c) {
        SpiConfig s{};
        s.pads = pads;
        s.role = SpiRole::client;
        s.mode = c.mode;
        s.form = c.address_match ? SpiForm::with_address : SpiForm::spi;
        s.bits = c.bits;
        s.lsb_first = c.lsb_first;
        s.receiver = true;
        s.preload = c.preload;
        s.ss_low_detect = c.ss_low_detect;
        s.address_mode = c.address_mode;
        s.address = c.address;
        s.address_mask = c.address_mask;
        s.immediate_overflow = c.immediate_overflow;
        s.run_standby = c.run_standby;
        return s;
    }
};

// =============================================================================
// The header's two SPI views really are one register set
// =============================================================================
//
// This file works through SPIM for BOTH roles (Sercom<n>::spi_regs()),
// which is only legitimate while the header's SPIS declaration agrees
// bit for bit. Every field this driver writes is checked, so a device
// pack that ever separated them would fail the BUILD rather than the
// bench.

static_assert(sizeof(sercom_spim_registers_t) == sizeof(sercom_spis_registers_t));
static_assert(SERCOM_SPIM_CTRLA_MODE_Msk == SERCOM_SPIS_CTRLA_MODE_Msk);
static_assert(SERCOM_SPIM_CTRLA_ENABLE_Msk == SERCOM_SPIS_CTRLA_ENABLE_Msk);
static_assert(SERCOM_SPIM_CTRLA_SWRST_Msk == SERCOM_SPIS_CTRLA_SWRST_Msk);
static_assert(SERCOM_SPIM_CTRLA_DOPO_Msk == SERCOM_SPIS_CTRLA_DOPO_Msk);
static_assert(SERCOM_SPIM_CTRLA_DIPO_Msk == SERCOM_SPIS_CTRLA_DIPO_Msk);
static_assert(SERCOM_SPIM_CTRLA_CPOL_Msk == SERCOM_SPIS_CTRLA_CPOL_Msk);
static_assert(SERCOM_SPIM_CTRLA_CPHA_Msk == SERCOM_SPIS_CTRLA_CPHA_Msk);
static_assert(SERCOM_SPIM_CTRLA_DORD_Msk == SERCOM_SPIS_CTRLA_DORD_Msk);
static_assert(SERCOM_SPIM_CTRLA_FORM_Msk == SERCOM_SPIS_CTRLA_FORM_Msk);
static_assert(SERCOM_SPIM_CTRLA_IBON_Msk == SERCOM_SPIS_CTRLA_IBON_Msk);
static_assert(SERCOM_SPIM_CTRLA_RUNSTDBY_Msk == SERCOM_SPIS_CTRLA_RUNSTDBY_Msk);
static_assert(SERCOM_SPIM_CTRLB_CHSIZE_Msk == SERCOM_SPIS_CTRLB_CHSIZE_Msk);
static_assert(SERCOM_SPIM_CTRLB_AMODE_Msk == SERCOM_SPIS_CTRLB_AMODE_Msk);
static_assert(SERCOM_SPIM_CTRLB_RXEN_Msk == SERCOM_SPIS_CTRLB_RXEN_Msk);
static_assert(SERCOM_SPIM_CTRLB_MSSEN_Msk == SERCOM_SPIS_CTRLB_MSSEN_Msk);
static_assert(SERCOM_SPIM_CTRLB_SSDE_Msk == SERCOM_SPIS_CTRLB_SSDE_Msk);
static_assert(SERCOM_SPIM_CTRLB_PLOADEN_Msk == SERCOM_SPIS_CTRLB_PLOADEN_Msk);
static_assert(SERCOM_SPIM_INTFLAG_DRE_Msk == SERCOM_SPIS_INTFLAG_DRE_Msk);
static_assert(SERCOM_SPIM_INTFLAG_TXC_Msk == SERCOM_SPIS_INTFLAG_TXC_Msk);
static_assert(SERCOM_SPIM_INTFLAG_RXC_Msk == SERCOM_SPIS_INTFLAG_RXC_Msk);
static_assert(SERCOM_SPIM_INTFLAG_SSL_Msk == SERCOM_SPIS_INTFLAG_SSL_Msk);
static_assert(SERCOM_SPIM_INTFLAG_ERROR_Msk == SERCOM_SPIS_INTFLAG_ERROR_Msk);
static_assert(SERCOM_SPIM_STATUS_BUFOVF_Msk == SERCOM_SPIS_STATUS_BUFOVF_Msk);
static_assert(SERCOM_SPIM_SYNCBUSY_SWRST_Msk == SERCOM_SPIS_SYNCBUSY_SWRST_Msk);
static_assert(SERCOM_SPIM_SYNCBUSY_ENABLE_Msk == SERCOM_SPIS_SYNCBUSY_ENABLE_Msk);
static_assert(SERCOM_SPIM_SYNCBUSY_CTRLB_Msk == SERCOM_SPIS_SYNCBUSY_CTRLB_Msk);
static_assert(SERCOM_SPIM_ADDR_ADDR_Msk == SERCOM_SPIS_ADDR_ADDR_Msk);
static_assert(SERCOM_SPIM_ADDR_ADDRMASK_Msk == SERCOM_SPIS_ADDR_ADDRMASK_Msk);
static_assert(SERCOM_SPIM_DBGCTRL_DBGSTOP_Msk == SERCOM_SPIS_DBGCTRL_DBGSTOP_Msk);

// The chapter's own arithmetic, pinned at compile time (table 30-2,
// synchronous row) at the 48 MHz this stratum's Clock<> produces.
static_assert(spi_baud_reg(48'000'000UL, 24'000'000UL) == 0);
static_assert(spi_sck_hz(48'000'000UL, 0) == 24'000'000UL);
static_assert(spi_baud_reg(48'000'000UL, 1'000'000UL) == 23);
static_assert(spi_sck_hz(48'000'000UL, 23) == 1'000'000UL);
static_assert(spi_baud_reg(48'000'000UL, 93'750UL) == 255);
static_assert(spi_sck_hz(48'000'000UL, 255) == 93'750UL);
// Faster than f_ref/2, and slower than f_ref/512: both outside the
// generator, both refused rather than clamped silently.
static_assert(!spi_baud_reg(48'000'000UL, 30'000'000UL).has_value());
static_assert(!spi_baud_reg(48'000'000UL, 50'000UL).has_value());
// A rate the divisor cannot hit exactly rounds DOWN in frequency, never
// up: 700 kHz asked, 685714 Hz produced.
static_assert(spi_baud_reg(48'000'000UL, 700'000UL) == 34);
static_assert(spi_sck_hz(48'000'000UL, 34) == 685'714UL);

} // namespace brio
