/*
 * spi.hpp
 *
 * The SAM C21 SERCOM in SPI mode (DS60001479M ch. 32, over the baud
 * generator and the pad matrix of the shared ch. 30), for a program that
 * talks to SPI devices through the bus arbiter, or answers another
 * board's host as a client. The smallest use, a device client on the
 * arbitrated bus (docs/samc21/spi.md, "How to use"):
 *
 *     using SpiHw = brio::SpiHost<1, my_pads>;
 *     using SpiBus = brio::SpiBus<SpiHw, P>;     // an active object
 *     SpiHw::init(clock);                        // in main(), after the clock task
 *
 * then an active object posts a Request (the chip select, the buffers as
 * lend<Lease::reply>(...), baud = *SpiHw::baud_for(hz), the mode, and
 * the engine it names) to SpiBus and gets an SpiDone back. The program names
 * how each request runs. SpiEngine::polled is the loop, inside the arbiter's
 * dispatch, at any length, no vector bound. SpiEngine::dma is the DMA engines,
 * at any length: the board file names them in the host's template arguments
 * (samc21/dmac.hpp, its budget of two channels), the program binds the DMAC's
 * vector and the SERCOM's to dma_isr(), and start() returns with the wire
 * running. On a host with no engines, a request naming dma is refused (SpiDone
 * with spi_rejected, nothing on the wire; `SpiHw::has_dma` is false for a
 * program's static_assert).
 *
 * Who calls what, and from where. init() is a setup call in main(),
 * after the clock task's init() and before interrupts are enabled
 * globally; it waits, bounded, for the peripheral's synchronizations and
 * answers false when one did not complete or a stated SCK ceiling cannot
 * be produced. start() is the arbiter's, from its dispatch in main
 * context: a polled request runs whole inside it with global interrupts
 * enabled, bounded by one spin budget, the request's cs_setup_us spent
 * spinning there first; a refused one returns at once. A change of mode
 * or rate costs start() a disable/enable pair and its bounded
 * synchronization waits first. dma_isr() is the vectors' body and ends a
 * request on the engines. A host with no engines arms no interrupt of the
 * SERCOM's and leaves its NVIC line disabled. The Request's three buffers
 * are lent from the post until the SpiDone the arbiter replies with:
 * untouched and in scope until then, which nothing checks. The chip
 * select is a GPIO the device client configures (init() claims DO, SCK
 * and, when the pads name one, DI, and nothing else). SpiClient's polled runs belong to the context that
 * calls them and spin, bounded by the caller's budget, until the run
 * completes, the host raises the select, or the budget runs out. The
 * resource's configuring verbs are setup calls with no guard of their
 * own. The parts:
 *
 *  Spi<n>      the resource - a typed view of one instance's SPI
 *              register set: both roles, every field of CTRLA and
 *              CTRLB, the synchronous baud arithmetic, the flags and
 *              the one combined interrupt question this core's single
 *              vector has to ask.
 *
 *  SpiHost<n, pads, generator, TxEngine, RxEngine, Placement>
 *              the task util/spi_bus.hpp (= util/bus_master.hpp) drives:
 *              the transfer engine. A Request is a two-phase transaction
 *              in one chip-select window, the buffers travel with it as
 *              Lease::reply loans, and it runs in the loop or on the DMA
 *              engines as it names. It is the descriptor shape every target's
 *              SpiHost carries, so a device client reads unchanged on any of them.
 *
 *  SpiClient<n, pads>
 *              the other end of the wire: a polled surface plus the ISR
 *              bodies, with the three things this silicon gives a
 *              client - preloading (CTRLB.PLOADEN),
 *              select-low detection (CTRLB.SSDE) and address recognition
 *              (CTRLA.FORM = 0x2 with CTRLB.AMODE and ADDR).
 *
 * One instance, one address, one set of facts. Everything that belongs
 * to the SERCOM instance rather than to a mode - the APB mask, the GCLK
 * channel, the NVIC line - is Sercom<n>'s
 * and is used from here; only the register view differs, and that comes
 * from Sercom<n>::spi_regs(). There is no second table of which channel
 * SERCOM5 sits on, and there cannot be one.
 *
 * The header's two SPI views are the same registers. sercom.h declares
 * `sercom_spim_registers_t` and `sercom_spis_registers_t` with identical
 * offsets and identical field positions under two name prefixes, so this
 * file uses one (SPIM) for both roles and the role is CTRLA.MODE and
 * nothing else. That is asserted at the bottom of this file against the
 * header's own SPIS macros rather than believed.
 *
 * Pads are not pins, and DOPO is a triple. CTRLA.DIPO names the pad the
 * data input is read from and takes any of the four. CTRLA.DOPO does
 * not name a pad: its four codes each fix a whole (DO, SCK, SS) triple
 * (32.8.1's table), so the three cannot be chosen independently and
 * "swap SCK and SS" is not a configuration the silicon offers. A caller
 * states the triple it has wired and spi_dopo_for() answers with the one
 * code that produces it, or with nothing - which is a compile error at
 * the task. Note the device header's naming trap, the same one TXPO has
 * in sercom.hpp: it calls DOPO = 0x1 `DOPO_PAD1_Val` while that code
 * puts DO on PAD[2]; the enumerator names the code, not the pad.
 * Everything below is named after the pad a signal lands on.
 *
 * Which signal is which depends on the role (32.8.1, table 32-2):
 *
 *              host            client
 *   DO         MOSI (out)      MISO (out)
 *   DI         MISO (in)       MOSI (in)
 *   SCK        out             in
 *   SS         out, MSSEN=1    in, always
 *
 * so on one fixed four-wire harness the two roles are two different
 * DOPO rows, not one row with the directions flipped. The bench proves
 * exactly that: the same four wires carry a host on row 0x3 and a
 * client on row 0x1.
 *
 * Facts that shape the code (DS60001479M 32.5.x, 32.6.x, 32.8.x, and
 * errata DS80000740S, silicon rev F on the bench chip):
 *  - CTRLA, CTRLB, BAUD and ADDR are enable-protected (32.6.2.1): a
 *    write while the peripheral is enabled is discarded, not refused.
 *    Everything that changes them here disables first. The exceptions
 *    the chapter names are CTRLA.ENABLE, CTRLA.SWRST and CTRLB.RXEN;
 *  - SYNCBUSY has exactly three bits (32.8.8): SWRST, ENABLE, CTRLB.
 *    Writing CTRLB while SYNCBUSY.CTRLB stands is an APB error, so
 *    every CTRLB write here is followed by a bounded wait;
 *  - enabling the peripheral clears CTRLB.RXEN (32.8.2) and raises
 *    SYNCBUSY.CTRLB until the receiver is really up - the same trap
 *    sercom.hpp's enable() documents for the USART, and the reason
 *    enable() here waits out both synchronizations;
 *  - a client's DATA write needs three SCK cycles to reach the shift
 *    register (32.6.2.6.2), so the FIRST character of a transaction is
 *    never the one just written unless preloading is on. That is not a
 *    driver bug to hide, it is the reason CTRLB.PLOADEN exists;
 *  - RXC is set when a character has been fully shifted in, and reading
 *    DATA is what clears it. It is therefore the exact "one character
 *    has moved, both ways" edge, which the host engine's receive loop
 *    waits for, the received byte in hand when it reads. It is not the
 *    wire's end: in the modes whose last edge is not the sampling one it
 *    rises half a bit before that edge (measured, docs/samc21/spi.md), and
 *    TXC is what closes the frame;
 *  - STATUS carries one error bit, BUFOVF (32.6.2.7), and CTRLA.IBON
 *    decides whether it is raised at the overflow or travels with the
 *    data through the two-level receive buffer;
 *  - there is no event surface at all: 32.5.6 and 32.6.4.3 are both
 *    "Not applicable". Under samc21/evsys.hpp's division of labour a
 *    peripheral publishes its own generator and user codes; this one
 *    has none to publish, and the absence is
 *    stated rather than left to be noticed;
 *  - erratum 1.17.16: CTRLA.SWRST does nothing while CTRLA.ENABLE = 0,
 *    on every revision by the sheet; not reproduced in SPI mode on this
 *    silicon (measured: SWRST from the disabled state resets the block,
 *    its synchronization completing; reset()'s comment). reset() keeps
 *    the discipline and enables the instance first when it finds it
 *    disabled - in SPI host mode, because the enable synchronizes
 *    against GCLK_SERCOMx_CORE and a CLIENT is clocked by an external
 *    SCK that may not be running at all;
 *  - erratum 1.17.3 (live on every revision): with CTRLB.PLOADEN set,
 *    the first character a client sends is a dummy unless the host
 *    holds SS low for the whole transmission. The driver cannot enforce
 *    what the other end of the wire does, so the obligation is stated on
 *    SpiConfig::preload and nowhere pretended away;
 *  - erratum 1.17.19 (live on the sheet): DBGCTRL is reset by
 *    CTRLA.SWRST although 32.6.2.2 promises it is the one register a
 *    reset spares. Not reproduced on this silicon (measured: DBGCTRL
 *    reads 0x1 across SWRST from both states, docs/samc21/spi.md);
 *    configure() writes DBGCTRL after any reset, correct under either
 *    answer;
 *  - erratum 1.17.20 (live): a client left with a preloaded character on
 *    the way into standby consumes extra power. A sleep-current fact,
 *    named on the field;
 *  - erratum 1.17.1 is revision B only on the E/G/J row (the spurious
 *    INTFLAG.SSL when a client is enabled with SSDE and RXEN together).
 *    It is named here because it is the one item a reader would apply
 *    without checking the row, and it does not apply to this silicon.
 */

#pragma once

#include <stdint.h>

#include <bit>
#include <optional>
#include <span>
#include <type_traits>

#include "sam.h"

#include "samc21/clock.hpp"
#include "samc21/delay.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/platform.hpp"
#include "samc21/sercom.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/placement.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// =============================================================================
// The knobs (32.8.1, 32.8.2)
// =============================================================================

/// Which end of the bus this instance is (CTRLA.MODE: 0x3 host, 0x2
/// client). There is NO runtime demotion on this peripheral: a low SS
/// does not turn a host into a client, and a role is a configuration
/// written while the SERCOM is disabled.
enum class SpiRole : uint8_t {
    host = SERCOM_SPIM_CTRLA_MODE_SPI_MASTER_Val,
    client = SERCOM_SPIM_CTRLA_MODE_SPI_SLAVE_Val,
};

/// The four clock phase/polarity combinations (32.6.2.5, table 32-3). BIT
/// 1 IS CPOL AND BIT 0 IS CPHA, so brio's SpiMode is one vocabulary
/// across targets even though the register the bits go into is not the
/// same one. Here they are two separate bits of CTRLA rather than one
/// field of CTRLB.
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

    /// CTRLB.RXEN. A host with the receiver off gets no RXC, which
    /// SpiHost's receive loops wait on; it also frees the DI pad
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
     * state a freshly booted instance is in, so the obvious "reset first,
     * then configure" would silently do nothing. A disabled instance is
     * therefore ENABLED first and reset from there. THE BENCH COULD NOT
     * REPRODUCE THE ITEM in SPI mode at rev F (measured: SWRST from the
     * disabled state resets the block, its synchronization completing) -
     * the discipline is kept anyway: the sheet marks every revision,
     * other SERCOM modes are unmeasured, and the cost is one enable.
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
     * receiver is what raises RXC, the edge every receive loop waits on.
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
    /// released, bus clock off - and no reset. The pads' PINS are the
    /// task's to release. A disabled SERCOM withdraws a raised DMA request
    /// (measured on the same SERCOM as a Uart, docs/samc21/dmac.md, "A
    /// released requester"), and the disable's synchronization answer is
    /// not needed: one that never completed gates the instance with ENABLE
    /// set, the state that page measures holding a request, and the next
    /// host init() resets the instance before anything else
    /// (docs/design/dma.md, "The release contract").
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

    /// Inline, all three: a polled loop asks them once a character, and
    /// a call per poll turn was a tenth of the character at 12 MHz.
    [[gnu::always_inline]] static bool dre_flag() {
        return (regs().SERCOM_INTFLAG & SpiFlag::dre) != 0u;
    }
    [[gnu::always_inline]] static bool rxc_flag() {
        return (regs().SERCOM_INTFLAG & SpiFlag::rxc) != 0u;
    }
    /// Host: the last character has been shifted out and DATA holds
    /// nothing new. Client: the host has raised SS (32.8.6).
    [[gnu::always_inline]] static bool txc_flag() {
        return (regs().SERCOM_INTFLAG & SpiFlag::txc) != 0u;
    }
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

    /// The address the DMA engines write and read: DATA, whose low byte a byte beat
    /// reaches. The two triggers are Sercom<n>'s, the instance's own constants.
    static volatile void* data_address() { return &regs().SERCOM_DATA; }
    static constexpr uint8_t dma_tx_trigger() { return Base::dma_tx_trigger(); }
    static constexpr uint8_t dma_rx_trigger() { return Base::dma_rx_trigger(); }

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
 * SpiHost<n, pads, generator, TxEngine, RxEngine, Placement>
 *
 * The transfer ENGINE: the target-side half of the SPI stack, driven by
 * util/spi_bus.hpp (which owns arbitration and replies). This task owns
 * the wire: chip select, the D/C line of display-style devices, the
 * polled loops and the DMA engines that move the characters.
 *
 * TWO WAYS TO RUN A REQUEST, AND THE REQUEST NAMES ONE. Nothing here chooses
 * by cost: a wrong choice costs time, never correctness, and the wire does the
 * same under both. Under either, when a request ends the wire has ended: the
 * select rises after the last SCK edge, and whatever tells the caller so comes
 * after that edge too.
 *  - SpiEngine::polled: the loops below, inside start(), at any length. No DMA,
 *    no vector, and start() returns true.
 *  - SpiEngine::dma: the engines, at any length. start() returns false and
 *    dma_isr() ends it, a command followed by data excepted (below). In order:
 *    on a host with no engines, spi_rejected (a host with engines refuses every
 *    request, of either engine, while its own transfer is in flight, below); nothing to move, spi_ok; what the
 *    engines cannot carry - on a host with the transmit engine alone, a request
 *    that reads (rx not null) - spi_rejected, before the registers, the select or
 *    the D/C are touched, so the program names polled for its reads; a command
 *    followed by data runs the command in the loop, one fixed rule, a command being
 *    short and a block of its own a second launch and a second interrupt, and the
 *    data on the engines; a command alone rides the engines; and otherwise the
 *    engines. start() returns true for a polled request, an empty one and a refused
 *    one, and false only when the engines have it.
 *
 * THE ONE BUSY CHECK IS THE HOST'S OWN TRANSFER IN FLIGHT, made first: a request on the
 * engines between its launch and its end, whichever engine the new request names.
 * start() answers true with spi_rejected before the select, the D/C or the block moves,
 * and the transfer in flight goes on untouched. The arbiter never starts a request over
 * one in flight, so only a program that drives the host directly can meet it, and it
 * reads status() at once: the transfer's own end writes its answer there (spi_ok or
 * spi_dma_fault), so a vector landing between the two reads hands it that answer instead.
 *
 * THE ENGINE SLOTS: both, the transmit one alone, or neither (the defaults). The
 * engines are samc21/dmac.hpp's, each a channel of the two an image may hold, claimed
 * for this SERCOM's transmit and receive requests when the program is compiled
 * (samc21/sercom.hpp's sercom_dma_claimed()): a channel another transport of the image
 * names, or this SERCOM's request on both channels, does not compile. Each is armed
 * at init() and held to release() whether a request runs or not. So a board file
 * spends the budget: the pair takes it whole, and the console beside it takes no
 * channel; the transmit engine alone leaves one, a display's writes beside a
 * console's transmitter.
 *
 * ONE INTERRUPT A REQUEST, TXC, in every shape. "In Host mode, this flag is set when
 * the data have been shifted out and there are no new data in DATA" (32.8.6): it
 * closes the last frame on the wire, where a receive block's completion would come up
 * to half a bit early (RXC, the file header), so no vector here waits for the shifter.
 * Both channels are silent, armed for their transfer errors alone (DmaCompletion). A
 * buffer that ran dry BETWEEN two beats raises TXC too, the channel a frame late, and the
 * beat that clears it may wait as long as a load above the channel lasts. So that entry
 * clears the flag itself ("cleared by writing a one to it", 32.8.6): the flag is set by a
 * frame's end and by nothing else, it stays down however long the buffer stands dry, and
 * a request takes at most an entry a frame instead of re-entering the vector for the
 * length of the load. Only a transmit channel that has disabled itself has written its
 * last beat, and then TXC can only be the last frame's. The channel looked at again
 * behind that store says whether a last beat landed around it, whose frame's end the
 * store may have taken: the vector then waits out one frame, bounded, in that one case.
 * A receive channel's last beat follows RXC, which is up by then, by a few bus cycles at
 * most; a bound on it that runs out is a fault. What the contradiction proves is a fault
 * too: TXC up, DRE up, and a transmit channel enabled with nothing pending has lost its
 * trigger, and no beat would ever come.
 *
 * THE TWO VECTORS ARE BOTH THE PROGRAM'S, each calling dma_isr(): the SERCOM's for TXC
 * and the DMAC's for a channel's bus error, the second shared with every other owner of a
 * channel (samc21/dmac.hpp):
 *
 *     extern "C" void SERCOM1_Handler() {
 *         if (SpiHw::dma_isr()) { brio::post<SpiBus>(brio::TransferDone{SpiHw::status()}); }
 *     }
 *     extern "C" void DMAC_Handler() {
 *         if (SpiHw::dma_isr()) { brio::post<SpiBus>(brio::TransferDone{SpiHw::status()}); }
 *     }
 *
 * A vector left unbound is the crt's weak alias of its silent spin, where the program stops
 * at the first request's end. A polled request needs neither.
 *
 * A WRITE RIDES THE TRANSMIT CHANNEL ALONE (a null rx: a display's pixels), on a pair too, the
 * pair still holding both channels. The receiver stays on, its two levels overflow in silence,
 * ERROR is never armed, and the end drains them and clears BUFOVF (32.6.2.7). A read starts
 * its receive channel first: RXC is clear, every character of a command phase having been read
 * back, and it is the transmit channel's first beat that starts the clock. Neither is kicked:
 * DRE stands, and the enable fires the first beat by itself (samc21/dmac.hpp).
 *
 * AFTER A STALL OR A TRANSFER ERROR THE NEXT REQUEST STARTS ON A CLEAN BLOCK, whoever calls it,
 * and the program calls nothing: the host is the one type there is both in direct use and under
 * the arbiter, and the arbiter goes on to the next waiting request before the requester has
 * run. A stall resets the SERCOM inside start(), main context, the select released FIRST: SWRST
 * takes the instance out of host mode, the pads undriven until the enable drives them again, and
 * in modes 2 and 3 that return to an idle high is an edge on a line with a pull or a charge on it
 * (measured, docs/samc21/spi.md), which only a released select makes harmless. With a null select
 * the program's own select stands through it: the host cannot release what it does not hold. A
 * transfer error is met in the vector, which decides and does not work: the channels put away,
 * the select released, a reset owed. The debt is a value of the half-word apply() compares
 * (reset_owed_key), so the next start() that moves data, or prime(), pays it on the path that
 * already asks, out of line (an empty or refused request leaves it owed), and neither a polled
 * request nor a refusal carries a test for it; written in the vector
 * only while a request is in flight and read in main context only when none is, it needs no
 * critical section. reset_block() is the one sequence, called by the stall, by the owed reset and
 * by recover(). What it cannot do: on a SERCOM whose core clock is stopped SWRST never completes,
 * and nothing else may be written to it - so the debt stands, the next request stalls on its own
 * budget, and the first one after the clock returns pays it and runs. recover() stays for the
 * timed arbiter (util/bus_master.hpp) and a program's own use.
 *
 * Transaction descriptor (Request) - two phases in ONE chip-select
 * window, the shape every target's SpiHost carries:
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
 * THE REQUEST IS LENT FOR THE CALL (util/bus_master.hpp's contract): it
 * lies in a slot of the arbiter's that stays as it is until the dispatch
 * ends. A request that completes inside start() is read through the
 * reference and copied nowhere; one on the engines keeps its two pins, what
 * dma_isr() raises at the end, and nothing else of it.
 *
 * WHY THE CHIP SELECT IS A GPIO AND NOT THE PERIPHERAL'S. CTRLB.MSSEN
 * exists and this driver exposes it - but 32.6.3.5 says hardware SS is
 * raised "for a minimum of one baud cycle between each data sent", so it
 * frames one CHARACTER and not one TRANSACTION. Every device this engine
 * is for wants the second. So the Request carries a PinRef, and
 * 32.6.3.3's "host with several clients" is the
 * arrangement: MSSEN clear, one ordinary output per client.
 *
 * TWO CHARACTERS IN FLIGHT, FROM THE CHAPTER. "The SPI is single
 * buffered for transmitting and double buffered for receiving. When
 * transmitting data, the Data register can be loaded with the next
 * character to be transmitted during the current transmission" (32.6.1;
 * 32.2: "one-level transmit buffer, two-level receive buffer"); the
 * host's sequence is 32.6.2.6.1's: a character written to DATA moves to
 * the shift register, DRE rises once it has (three CLK_SERCOM_APB
 * cycles after DATA empties, 32.6.2.6.2's last sentence), and RXC rises
 * in the clock cycle the character's last bit is shifted in, its
 * received twin then sitting in the two-level receive buffer. So the
 * receive shapes of the loop keep the NEXT character in DATA while the
 * current one shifts: write k + 1 on DRE, read k on RXC, the bus never
 * idle between characters while the software is faster than a
 * character. THE IN-FLIGHT COUNT IS THE RECEIVE BUFFER'S DEPTH, which is
 * what makes it safe under every interrupt of the image with no rate
 * threshold: the loop runs with interrupts enabled, and a handler that
 * holds it for longer than a character lets k complete into the buffer
 * and then k + 1 - two, the buffer's two levels (32.2, 32.6.2.7) - and
 * then the bus STOPS for want of a k + 2 nobody wrote. A late reader
 * idles the bus; it never overflows the receiver. The transmit-only
 * shape keeps two in flight on DRE alone and lets the receiver overflow
 * in silence, drained at its tail (write_phase). Measured by bench_samc's
 * letter e: STATUS.BUFOVF read after every run, at 12 and 3 MHz, under
 * the console's and the tick's interrupts (docs/samc21/spi.md). The
 * receiver is enabled even for a write-only transfer - the command phase
 * is read back to place the D/C edge, below -, which costs the DI pad
 * and nothing else.
 *
 * THE PHASE BOUNDARY IS WHERE THE WRITE-AHEAD STOPS. The D/C line must
 * change between the last command character and the first data one ON
 * THE WIRE, and after the command's last edge: RXC rises half a bit before
 * that edge in modes 0 and 2 (measured, docs/samc21/spi.md), so a command phase
 * ends on TXC (32.8.6), D/C rises then, and the first data character is written after it -
 * one character in flight across that one boundary, two everywhere else. A
 * request with no command phase never meets it; one that is all command
 * raises D/C on the same TXC, the wire's end.
 *
 * AT ITS END, OK OR FAILED, A REQUEST LEAVES D/C AND THE SELECT HIGH: a
 * clean end, a stall in either phase on either engine, a transfer error, and
 * recover() over a request in flight. A refused or empty request moves neither.
 *
 * THE BOUNDS OF A REQUEST ON THE WIRE ARE TIMES, derived from the clocks when the rate is applied
 * (docs/design/spi-bus.md, "Every bound is a time"). A frame is 16 x (BAUD + 1)
 * core cycles on the wire (SCK = f_ref / 2 (BAUD + 1), table 30-2, on generator 0
 * f_ref the core's own rate, which is what lets the count ignore the rate). A polled transaction's budget is at least
 * twice its frames' wire time and 4096 core cycles more, held as a count of poll
 * turns at four core cycles a turn, under what a turn costs in the release listing
 * (a load, a test, a branch not taken, a decrement, a compare and a branch taken:
 * eight cycles at the least, some sixteen measured from the flash): a turn only runs while the wire is moving, a handler over the loop
 * spends none of it, and a stalled SERCOM holds the loop for the budget's time
 * or longer. release()'s wait on a block in flight is that block's wire time and
 * two frames, by the same count. What waits on no wire keeps a fixed count:
 * Spi::reset()'s 65535 turns for a synchronization, 44 ms when a stopped clock
 * never completes it (docs/samc21/spi.md), and the vector's 64 turns for a
 * receive beat, hundreds of core cycles against a beat's few.
 *
 * THE RECEIVE BUFFER IS NOT FLUSHED AT EVERY START. Every character a
 * receive shape of the loop writes is read back by it, so nothing stands
 * in the receiver when a transaction ends; the one shape that lets the
 * receiver overflow - the transmit-only data phase - drains the
 * two-level buffer and clears BUFOVF (32.6.2.7) at its own tail, the
 * request that made the spill paying for it. A loop whose bounded spin
 * ran out flushes too, since a character may still land.
 *
 * THE PER-REQUEST CHIP-SELECT DELAY is part of that shared descriptor:
 * the Request carries cs_setup_us, spent spinning in start() (main
 * context, bounded by the byte) between the CS assertion and the first
 * clock, timed by samc21/delay.hpp on a rate init()/rebase() keep
 * current - so it follows a clock change, and a device client reads
 * unchanged on any target. It is served ONLY while a Ticker runs
 * (delay_us's own contract; with SysTick stopped the wait is skipped and
 * the transaction proceeds), and on a program whose tick period is
 * SHORTER than the requested setup the cap skips it too - both stated
 * here, neither reachable on the standard 1000 Hz ticker with a uint8_t
 * of microseconds.
 *
 * status() is spi_ok, spi_stalled when a request's bounded spin ran out,
 * spi_dma_fault when a channel took a bus error or lost its trigger, or
 * spi_rejected when a request was refused.
 *
 * WHERE THE HOT PATH RUNS FROM: the host's placement, its last template
 * parameter (util/placement.hpp). It places the loops: the polled transaction
 * - its command phase, the D/C flip and its data phase in whichever of the loops
 * below the spans pick - which start() calls once a request, and the command loop
 * of a request on the engines. Under InFlash, the default, each is what it would
 * be without the placement, in the flash; under InRam it is one instance
 * in SRAM (`.fast_text.brio`), a call through the linker's long-branch
 * veneer from the flash. dma_isr() takes the place of the vector that binds it.
 * The flash runs behind two wait states at 48 MHz
 * (DS60001479M table 45-41), and the receive loop fetched from it holds
 * a character longer than the wire's at the top rate. The measured
 * figures, every placement, are docs/samc21/spi.md's.
 */

/// A DMA transfer error, or a channel found dead, as a BusDone status - the first
/// engine-defined code the bus_master contract reserves (bus_engine_status).
inline constexpr uint8_t spi_dma_fault = bus_engine_status;

template <uint8_t n, SpiPads pads, uint8_t generator = 0, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine, typename Placement = InFlash>
class SpiHost {
    using S = Spi<n>;

    static_assert(HotPlacement<Placement, SamPlatform>,
                  "brio SpiHost: the placement is util/placement.hpp's InFlash (the default) or InRam");
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: samc21/dmac.hpp's DmaTxEngine "
                  "and DmaRxEngine, or NoDmaEngine (the default)");
    static_assert(!RxEngine::present || TxEngine::present,
                  "brio SpiHost: a receive engine comes with a transmit engine - the transmit "
                  "channel's beats are what clock the wire; a host that writes alone names the "
                  "transmit engine alone");
    static_assert([] {
        if constexpr (RxEngine::present) {
            return TxEngine::channel != RxEngine::channel;
        } else {
            return true;
        }
    }(), "brio SpiHost: the two engines ride two different DMA channels, 0 and 1");
    // The channels' one owner each, this SERCOM's two requests: a second owner
    // of either channel in the image, or this SERCOM's request on both, does not
    // compile (samc21/sercom.hpp, sercom_dma_claimed()).
    static_assert(sercom_dma_claimed<TxEngine, n, DmaSignal::tx>());
    static_assert(sercom_dma_claimed<RxEngine, n, DmaSignal::rx>());

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
    /// 0 is CLK_MAIN undivided in this stratum (samc21/clock.hpp says so,
    /// and Clock::hz is that rate), which is what lets init() derive the
    /// baud divisor from the app's Clock tag alone.
    static constexpr uint8_t core_generator = generator;

    /// The engines this host was given (the class comment).
    static constexpr bool has_engines = TxEngine::present;
    static constexpr bool has_rx_engine = RxEngine::present;
    /// Can this host run a request naming SpiEngine::dma? False refuses every
    /// such request (spi_rejected), so a program that posts one asserts this -
    /// `static_assert(SpiHw::has_dma)`. True on the transmit engine alone
    /// refuses the ones that read.
    static constexpr bool has_dma = has_engines;

    /// Forty bytes and no padding - the two pin references, the three
    /// spans, then every narrow field packed, the reply capsule closing
    /// the last word. The field NAMES are the contract every target's
    /// SpiHost shares (a device client fills them by name); the ORDER is
    /// this family's.
    struct Request {
        PinRef cs;   ///< asserted low around the transaction
        PinRef dc;   ///< display D/C line; null = no such pin
        /// Phase 1, sent with DC low; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        /// Phase 2 out, null = 0xFF dummies; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; LENT until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint16_t len;      ///< phase 2 length
        uint8_t cmd_len;   ///< phase 1 length

        /// The way to run it (SpiEngine, util/spi_bus.hpp), and the host does
        /// what it says (the class comment): polled, the loop, inside start(),
        /// the arbiter's dispatch held for the whole transfer (bounded), global
        /// interrupts enabled throughout; dma, the engines, start() returning
        /// with the wire running, refused on a host that cannot carry it.
        SpiEngine engine = SpiEngine::polled;

        /// Microseconds between the CS assertion and the first clock -
        /// what a device's datasheet calls CS setup (the same field, byte
        /// for byte, in every target's Request). Spent spinning in
        /// start(), main context; 0 = none.
        uint8_t cs_setup_us = 0;

        /// Per-transaction bus configuration. On a SHARED bus every
        /// device names its own speed and mode in the request, and the
        /// engine reprograms the peripheral at each start() - which
        /// costs a disable/enable pair only when something CHANGED, and
        /// nothing per byte.
        ///
        /// THE RATE IS THE BAUD REGISTER VALUE, not an enum of divisions:
        /// this generator has 256 of them, and naming them would be
        /// inventing a vocabulary the silicon does not have. Use
        /// SpiHost::baud_for(hz) once, at the client's own init, and keep
        /// the byte. It is a DIVISION of the core clock, so it follows a
        /// clock change by itself; max_sck_hz() is the ceiling that
        /// clamps it.
        uint8_t baud = 0;
        SpiMode mode = SpiMode::mode0;

        ReplyTo<SpiDone> reply;
    };
    static_assert(std::is_trivially_copyable_v<Request>);
    // Forty bytes, no padding: a field added is a word more in every
    // arbiter slot.
    static_assert(sizeof(Request) == 40,
                  "brio SpiHost: the Request grew - check its layout for padding");

    // ---- lifecycle -----------------------------------------------------------

    /// Bring the instance up as a host: clocks, configuration, the engines
    /// armed, pads, pins; on a host with no engines the NVIC line left
    /// disabled (it arms no interrupt), on one with engines opened for TXC. Setup's
    /// call, main context; it waits, bounded, for the peripheral's synchronizations.
    ///
    /// Call after the main clock is set up and before interrupts are
    /// enabled globally; `clock` is the app's brio::Clock tag, so the
    /// baud arithmetic comes from Clock::hz and never from a second
    /// statement of the rate. The pads handed over are DO, SCK and DI
    /// (when the pads name one); the chip selects are each device
    /// client's own GPIO outputs, which it configures (high) before its
    /// first request.
    ///
    /// `max_sck_hz` is an optional CEILING for the whole bus: with it set
    /// the engine slows any request that would exceed it, and re-resolves
    /// the limit after a clock change, which is what makes rebase()
    /// meaningful. 0 = no ceiling.
    ///
    /// False when the ceiling cannot be produced at this clock, or when
    /// one of the peripheral's synchronizations, or a channel's reset, did not
    /// complete.
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        static_assert(clock_follows<Clock, SpiHost>(),
                      "this SpiHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its SCK ceiling and its cs_setup "
                      "timing would go stale on a clock change");

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
        // THE BLOCK FROM RESET, first (docs/design/dma.md, "The release
        // contract"): the clock just opened, no register of this transport's
        // written and no channel armed, SWRST withdraws whatever a previous
        // life left standing - a transmit request held behind the gate
        // included - so the engines meet none.
        if (!S::reset()) {
            return false;
        }

        applied_ = boot_config();
        applied_key_ = key_of(applied_.mode, applied_.baud);
        turns_a_frame_ = turns_a_frame(applied_.baud);
        if (!S::configure(applied_)) {
            return false;
        }
        if (!S::enable(true)) {
            return false;
        }
        status_ = spi_ok;
        if constexpr (has_engines) {
            dma_active_ = false;
            if (!TxEngine::arm(S::data_address(), S::dma_tx_trigger(),
                               TxEngine::Completion::silent)) {
                return false;
            }
            if constexpr (has_rx_engine) {
                if (!RxEngine::arm(S::data_address(), S::dma_rx_trigger(),
                                   RxEngine::Completion::silent)) {
                    TxEngine::release();
                    return false;
                }
            }
            Nvic::enable(S::irq());
        }

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
        return true;
    }

    /// The core clock changed (DynamicClock fan-out).
    ///
    /// A Request's `baud` is a DIVISION of the core clock and scales with
    /// it by itself, so what is recomputed here is only the ceiling. The
    /// bus must be IDLE: a transfer in flight keeps its divisor for the
    /// remaining characters.
    static void rebase(uint32_t hz) {
        ref_hz_ = hz;
        ceiling_ = ceiling_hz_ ? spi_baud_reg(hz, ceiling_hz_) : std::optional<uint8_t>{};
        // A slower BAUD is a LARGER register value, so the ceiling is a
        // floor on the byte; 0 (the fastest rate) is the identity.
        floor_ = ceiling_ ? *ceiling_ : 0u;
        // The cs_setup timing follows the clock too - the one division
        // of delay_rate() is paid here, never at wait time.
        cs_rate_ = delay_rate(hz);
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

    /// Put the peripheral at this mode and rate NOW, moving no data.
    ///
    /// FOR CALLERS THAT FRAME THE SELECT WINDOW THEMSELVES (Request.cs
    /// null). start() applies a request's mode before it asserts the
    /// request's own cs, so an engine-owned select window always opens
    /// with SCK settled at the new idle level - but a caller driving CS
    /// by hand inverts that order, and a mode change is a CPOL FLIP ON
    /// THE WIRE: flipped inside an open select window it is one extra
    /// edge, and a selected client counts it into the character (measured
    /// as a one-bit slip in both directions). Prime FIRST, then assert
    /// the select. A reset owed since a stall or a transfer error is paid
    /// here too, before that select.
    static void prime(SpiMode mode, uint8_t baud) { apply(mode, clamp(baud)); }

    // ---- the transfer --------------------------------------------------------

    /// Run a transaction, or begin it (called by SpiBus from main context). True
    /// when it ended inside the call - a polled request, the empty one, a refused
    /// one (spi_rejected in status(), the wire untouched, a transfer of this host's
    /// in flight among the reasons), a command loop that stalled - and false when
    /// the engines have it and dma_isr() will end it:
    /// util/bus_master.hpp's engine contract. True is "ended", not "succeeded": a
    /// request whose budget ran out returns true with the select released, the
    /// SERCOM reset and status() = spi_stalled, so a caller driving the engine
    /// without the arbiter reads status() and calls nothing else. The order of the
    /// refusals is the class comment's.
    ///
    /// The request is read through `r` and copied nowhere; on the engines its two
    /// pins are kept.
    static bool start(const Request& r) {
        const bool on_engines = r.engine == SpiEngine::dma;
        if constexpr (has_engines) {
            if (dma_active_) [[unlikely]] {
                // The one busy check, first: this host's own transfer is in
                // flight, and nothing here moves (the class comment).
                status_ = spi_rejected;
                return true;
            }
        } else {
            if (on_engines) {
                // This host has no engines: refused before the registers, the
                // select or the D/C are touched.
                status_ = spi_rejected;
                return true;
            }
        }
        status_ = spi_ok;
        // cmd_len is a byte and len sixteen bits: the sum needs seventeen, or a
        // request of 65536 frames would read as a request of none.
        const uint32_t total = static_cast<uint32_t>(r.cmd_len) + r.len;
        if (total == 0u) {
            return true;   // nothing to move: complete on the spot
        }
        if constexpr (has_engines && !has_rx_engine) {
            if (on_engines && r.rx.get() != nullptr) {
                // A transmit engine alone has nowhere to put what comes back:
                // the program names polled for its reads.
                status_ = spi_rejected;
                return true;
            }
        }
        apply(r.mode, clamp(r.baud));
        if (r.cmd_len != 0) {
            r.dc.clear();
        } else {
            r.dc.set();
        }
        r.cs.clear();   // assert, active low
        if (r.cs_setup_us != 0u) {
            // The device's CS setup, spent here in main context. Skipped
            // by delay_us's own contract when no Ticker runs - see the
            // class comment.
            (void)delay_us(cs_rate_, r.cs_setup_us);
        }
        if constexpr (has_engines) {
            if (on_engines) {
                return start_engines(r);   // false: dma_isr() is the completion edge
            }
        }
        // Global interrupts STAY ENABLED, so the ticker and anything else
        // preempt this loop freely.
        polled_transaction(r, total);
        r.cs.set();   // release: transaction done
        if (status_ == spi_stalled) [[unlikely]] {
            (void)reset_block();   // the select released first: the class comment
        }
        return true;
    }

    /// The vectors' body, on a host with engines: call it from the SERCOM's handler
    /// and from DMAC_Handler, and from nowhere else (the class comment). True when
    /// the transaction just completed, the select released: the edge the program's
    /// glue posts TransferDone{status()} on. Runs to completion and waits at most a
    /// few bus cycles, bounded, for a receive channel's last beat, and one frame in the
    /// rare case of last_frame_out(). False, and free, on a host with no engines; one load
    /// and a branch between its requests, which is what this body costs another owner's
    /// completion.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            // Nothing of this host is pending between its requests: a channel
            // interrupts for its transfer error alone, and TXC is armed from a
            // launch to its end. So another owner's completion pays this load.
            if (!dma_active_) {
                return false;
            }
            uint8_t flags = TxEngine::take_interrupt();
            if constexpr (has_rx_engine) {
                flags = static_cast<uint8_t>(flags | RxEngine::take_interrupt());
            }
            if ((flags & TxEngine::flag_error) != 0u) [[unlikely]] {
                return end_on_fault();
            }
            if (S::txc_flag()) {
                return transmit_complete();
            }
        }
        return false;
    }

    /// The last request's status: read by the arbiter after a start() that returned
    /// true, and by the glue in the vector right after a dma_isr() that returned true.
    /// spi_ok, spi_stalled when its bounded spin ran out, spi_dma_fault when a channel
    /// took a bus error or lost its trigger, or spi_rejected when it was refused.
    static uint8_t status() { return status_; }

public:
    /// Put the ENGINE back where start() is legal: the select of a transaction on the
    /// engines released first, its channels put away and re-armed, the peripheral
    /// reset and reconfigured to the applied state. Clocks, pads and the ceiling
    /// arithmetic are untouched (a SERCOM software reset reaches none of them), so no
    /// Clock is needed.
    ///
    /// The verb a timed SpiBus calls on a transaction that never answered
    /// (util/bus_master.hpp); a program may call it with the bus idle, and owes it after
    /// nothing: a stall and a transfer error need no call (the class comment). Main
    /// context. A channel running here is disabled mid-block, and beside another owner's
    /// running channel that is what docs/samc21/dmac.md's rule speaks of. The host's one
    /// other path that disables an enabled channel beside another owner's is the lost
    /// trigger on the transmit engine alone, met in the vector (end_on_fault()): no beat
    /// is in flight there, the dead-block case of that page's exceptions.
    ///
    /// Returns false when a synchronization never settled (the bounded
    /// waits' honesty: a false engine is refusing, not hanging).
    static bool recover() {
        if constexpr (has_engines) {
            bool was_active = false;
            {
                typename SamPlatform::CriticalSection cs;
                was_active = dma_active_;
                dma_active_ = false;
            }
            S::enable_txc_interrupt(false);
            if (was_active) {
                tenure_cs_.set();
                tenure_dc_.set();
                put_channels_away();
            }
        }
        return reset_block();
    }

    /// Hand the pins back, then the peripheral: the engines first. THE TRANSMIT BLOCK IN FLIGHT ENDS FIRST
    /// (docs/samc21/dmac.md, "The rule": another owner's channel may be running), the wait
    /// on the channel itself, so no vector is needed, and bounded by that block's time on
    /// the wire; a block that outlives the bound is disabled anyway.
    static void release() {
        if constexpr (has_engines) {
            // The block's wire time and two frames (the class comment), a turn here
            // a masked select and load, far over four cycles.
            uint32_t spins = (static_cast<uint32_t>(TxEngine::in_flight()) + 2u) * turns_a_frame_ / 2u;
            while (TxEngine::running() && spins-- != 0u) {
            }
            TxEngine::release();
            if constexpr (has_rx_engine) {
                RxEngine::release();
            }
            dma_active_ = false;
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
    /// A slower BAUD is a LARGER register value, so the ceiling clamps
    /// from below: one byte compared, no optional on this path.
    [[gnu::always_inline]] static uint8_t clamp(uint8_t b) {
        return b < floor_ ? floor_ : b;
    }

    static SpiConfig boot_config() {
        SpiConfig c{};
        c.pads = pads;
        c.role = SpiRole::host;
        c.mode = SpiMode::mode0;
        c.receiver = true;   // RXC paces the receive loops: see the class comment
        c.baud = ceiling_ ? *ceiling_ : 0;
        return c;
    }

    /// The mode and the rate as ONE half-word, the thing apply() compares.
    [[gnu::always_inline]] static uint16_t key_of(SpiMode mode, uint8_t baud) {
        return static_cast<uint16_t>((static_cast<uint16_t>(mode) << 8) | baud);
    }

    /// The half-word that says a reset is owed: no mode has this high byte, so the next
    /// apply() compares unequal and pays it (the class comment).
    static constexpr uint16_t reset_owed_key = 0xFF00u;

    /**
     * Put the peripheral where this request wants it: ONE compare on the
     * path a run of requests to one device takes, the change outlined.
     *
     * CTRLA (which is where CPOL and CPHA live on this peripheral) and
     * BAUD are BOTH enable-protected, so any change costs a
     * disable/enable pair - which is why the applied state is cached and
     * the pair is paid only when something really moved - or a reset is owed.
     */
    [[gnu::always_inline]] static void apply(SpiMode mode, uint8_t baud) {
        if (key_of(mode, baud) != applied_key_) {
            apply_change(mode, baud);
        }
    }

    [[gnu::noinline]] static void apply_change(SpiMode mode, uint8_t baud) {
        const bool owed = applied_key_ == reset_owed_key;
        applied_key_ = key_of(mode, baud);
        applied_.mode = mode;
        applied_.baud = baud;
        turns_a_frame_ = turns_a_frame(baud);
        if (owed) {
            (void)reset_block();
            return;
        }
        (void)S::enable(false);
        S::regs().SERCOM_CTRLA = spi_ctrla(applied_);
        S::baud_reg(baud);
        (void)S::enable(true);
    }

    /// A poll turn's floor in core cycles, and the slack a transaction's
    /// budget starts from (the class comment).
    static constexpr uint32_t turn_cycles = 4u;
    static constexpr uint32_t budget_slack_cycles = 4096u;

    /// The budget per frame at a BAUD value: twice the frame's 16 x (BAUD + 1)
    /// core cycles, in turns. Worked out where a rate is applied, so a request
    /// pays one multiply for it.
    static constexpr uint32_t turns_a_frame(uint8_t baud) {
        return 2u * 16u * (static_cast<uint32_t>(baud) + 1u) / turn_cycles;
    }

    /// The bounded spin of a polled transaction, ONE budget for the whole of
    /// it: a time, at least twice its wire time and 4096 core cycles more,
    /// counted in turns of at least turn_cycles (the class comment).
    static uint32_t spin_budget(uint32_t frames) {
        return budget_slack_cycles / turn_cycles + turns_a_frame_ * frames;
    }

    /// Spin until an INTFLAG bit stands, charging the transaction's
    /// budget one per turn. False when it ran out: the clock stopped, or
    /// the peripheral is not what it was.
    [[gnu::always_inline]] static bool wait_flag(uint8_t flag, uint32_t& budget) {
        do {
            if ((S::regs().SERCOM_INTFLAG & flag) != 0u) {
                return true;
            }
        } while (--budget != 0u);
        return false;
    }

    /**
     * One phase of a polled transaction, the chapter's host sequence
     * (32.6.2.6.1) a character AHEAD: character 0 into DATA; then, for
     * every k, DRE and the write of k + 1, RXC and the read of k. The
     * shape is a template parameter so the per-character loop carries
     * no null test: the three shapes a request can take (write-only,
     * read-only, both) and the fourth (neither: clock dummies, discard)
     * are four instantiations of this one loop. Pointers walk and the
     * budget counts down in a register: on this core's eight low
     * registers an index, a count and the two flag masks spilled the
     * budget to the stack, and a poll that found the flag still clear
     * cost a load and a store (counted in the listing, measured as a
     * fifth over the chapter's bare loop at 12 MHz).
     */
    template <bool has_tx, bool has_rx>
    [[gnu::always_inline]] static bool run_phase(const uint8_t* tx, uint8_t* rx, uint32_t count,
                                                 uint32_t& budget) {
        if (count == 0u) {
            return true;
        }
        volatile uint32_t& data = S::regs().SERCOM_DATA;
        data = has_tx ? *tx : 0xFFu;
        uint32_t left = count - 1u;   // characters still to write
        for (; left != 0u; --left) {
            if (!wait_flag(SpiFlag::dre, budget)) {
                return false;
            }
            if constexpr (has_tx) {
                ++tx;
                data = *tx;
            } else {
                data = 0xFFu;
            }
            if (!wait_flag(SpiFlag::rxc, budget)) {
                return false;
            }
            const uint8_t in = static_cast<uint8_t>(data);
            if constexpr (has_rx) {
                *rx++ = in;
            }
        }
        if (!wait_flag(SpiFlag::rxc, budget)) {
            return false;
        }
        const uint8_t in = static_cast<uint8_t>(data);
        if constexpr (has_rx) {
            *rx = in;
        }
        return true;
    }

    /**
     * The TRANSMIT-ONLY data phase (a null rx: a display's pixels), paced
     * on DRE alone: a character into DATA as soon as the one before it
     * has moved to the shift register (32.6.2.6.1), the answers never
     * read. Two characters are in flight at every instant and the
     * receive side overflows in silence - BUFOVF stands, ERROR is never
     * armed, both cleared at the tail. The end is TXC, "set when the
     * data have been shifted out and there are no new data in DATA"
     * (32.8.6; measured to close the LAST character, spi.md): cleared
     * before the first character goes out, and if a handler holds the
     * loop long enough for the shifter to run dry in between, raised
     * early and cleared again by the next write ("cleared ... by writing
     * new data to DATA", 32.8.6), so the tail's wait is the last
     * character's whatever happened in between. One poll a character
     * where the receive shape has two, and no reading side to overrun.
     */
    template <bool has_tx>
    [[gnu::always_inline]] static bool write_phase(const uint8_t* tx, uint32_t count,
                                                   uint32_t& budget) {
        if (count == 0u) {
            return true;
        }
        volatile uint32_t& data = S::regs().SERCOM_DATA;
        S::clear_flags(SpiFlag::txc);
        data = has_tx ? *tx : 0xFFu;
        for (uint32_t left = count - 1u; left != 0u; --left) {
            if (!wait_flag(SpiFlag::dre, budget)) {
                return false;
            }
            if constexpr (has_tx) {
                ++tx;
                data = *tx;
            } else {
                data = 0xFFu;
            }
        }
        const bool whole = wait_flag(SpiFlag::txc, budget);
        S::flush_rx();   // the answers nobody read, and the overflow they left (32.6.2.7)
        return whole;
    }

    /// The two-level receive buffer's depth (32.2): what a reader may
    /// leave unread, and what a transmit-only phase may leave behind
    /// without a drain.
    static constexpr uint8_t rx_depth = 2;

    /// The whole of a polled transaction: the command phase
    /// read back character by character and closed on TXC (the wire's end of
    /// it: the D/C flips after its last edge), the D/C flip, then the data phase in the
    /// shape the two spans give it - transmit-only on DRE where it is
    /// longer than the receive buffer (a display's row) and read back
    /// where it is not (a command's parameters: measured, the tail's
    /// drain costs a short request more than reading two answers), and
    /// anything with an rx a character ahead on DRE and RXC. A budget
    /// that ran out leaves the request completed as far as it went, the
    /// receiver flushed and the status spi_stalled (util/spi_bus.hpp):
    /// the SERCOM clocked nothing back in a budget many times the
    /// transaction's own length, so its clock is not running, and
    /// start() resets it, the select released. One function of the host's
    /// placement (the class comment), called once a request.
    static void polled_transaction(const Request& r, uint32_t total) {
        Hot<Placement>::run([&r, total] { polled_body(r, total); });
    }

    /// The command phase, polled on either engine: its characters read back, then
    /// TXC, which the last character's write cleared and which closes it on the wire
    /// where RXC comes half a bit early in modes 0 and 2 (the class comment). True,
    /// for one test, when there is no command.
    [[gnu::always_inline]] static bool command_phase(const Request& r, uint32_t& budget) {
        if (r.cmd_len == 0u) {
            return true;
        }
        return run_phase<true, false>(r.cmd.get(), nullptr, r.cmd_len, budget) &&
               wait_flag(SpiFlag::txc, budget);
    }

    /// The polled transaction's path, written once.
    [[gnu::always_inline]] static void polled_body(const Request& r, uint32_t total) {
        uint32_t budget = spin_budget(total);
        bool whole = command_phase(r, budget);
        r.dc.set();   // data phase, and the request's end: raised on a stall too
        if (whole) {
            const uint8_t* tx = r.tx.get();
            uint8_t* rx = r.rx.get();
            if (rx == nullptr) {
                if (r.len <= rx_depth) {
                    // No longer than the receive buffer: nothing to
                    // drain at the end, the answers read back as they
                    // land - cheaper than a tail that drains.
                    whole = tx != nullptr ? run_phase<true, false>(tx, nullptr, r.len, budget)
                                          : run_phase<false, false>(nullptr, nullptr, r.len, budget);
                } else {
                    whole = tx != nullptr ? write_phase<true>(tx, r.len, budget)
                                          : write_phase<false>(nullptr, r.len, budget);
                }
            } else {
                whole = tx != nullptr ? run_phase<true, true>(tx, rx, r.len, budget)
                                      : run_phase<false, true>(nullptr, rx, r.len, budget);
            }
        }
        // WHEN THE REQUEST ENDS THE WIRE HAS ENDED: the receive shapes stop on RXC, up to
        // half a bit before the last SCK edge (the file header), and TXC closes the frame.
        // One wait a request, one load where the shape stopped on TXC already. A
        // character's writes cleared any TXC an early rise left, so this one is the last
        // character's.
        if (whole && !wait_flag(SpiFlag::txc, budget)) {
            whole = false;
        }
        if (!whole) {
            status_ = spi_stalled;
            S::flush_rx();
        }
    }

    // ---- the engines ---------------------------------------------------------

    /// The command phase of a request on the engines, in the loop (the class comment):
    /// one more instance of the host's placement. True when its last edge passed.
    static bool command_loop(const Request& r) {
        bool whole = false;
        Hot<Placement>::run([&r, &whole] {
            uint32_t budget = spin_budget(r.cmd_len);
            whole = command_phase(r, budget);
        });
        return whole;
    }

    /// A request onto the engines, once a transaction, out of start()'s polled path: the
    /// two pins kept for the end, a command followed by data in the loop with no channel
    /// armed (so a stall there has nothing to put away), a command alone on the engines, and
    /// the block. False: dma_isr() ends it. True only when the command loop stalled.
    [[gnu::noinline]] static bool start_engines(const Request& r) {
        tenure_cs_ = r.cs;
        tenure_dc_ = r.dc;
        if (r.cmd_len != 0u) {
            if (r.len == 0u) {
                launch_engines(r.cmd.get(), nullptr, r.cmd_len);
                return false;
            }
            if (!command_loop(r)) {
                status_ = spi_stalled;
                S::flush_rx();
                r.dc.set();   // the rule of a request's end, on a stall too
                r.cs.set();
                (void)reset_block();
                return true;
            }
            r.dc.set();
        }
        launch_engines(r.tx.get(), r.rx.get(), r.len);
        return false;
    }

    /// One block: TXC cleared and armed before anything can raise it, the receive channel
    /// first where there is one to fill, the transmit channel last, its first beat the
    /// standing DRE's. Whether this request reads travels with it for the end.
    static void launch_engines(const uint8_t* tx, uint8_t* rx, uint16_t len) {
        dma_active_ = true;
        S::clear_flags(SpiFlag::txc);
        S::enable_txc_interrupt(true);
        if constexpr (has_rx_engine) {
            tenure_reads_ = rx != nullptr;
            if (rx != nullptr) {
                RxEngine::start(rx, len);
            }
        } else {
            (void)rx;
        }
        if (tx != nullptr) {
            TxEngine::start_run(tx, len);
        } else {
            TxEngine::start_fixed(&tx_dummy_, len);
        }
    }

    /// TXC stands on a request on the engines (the class comment): the last frame's when
    /// the transmit channel has written its last beat; otherwise a buffer that ran dry, or a
    /// channel that lost its trigger, a contradiction: the flags read again after the
    /// channel, a beat landing between the two looks having cleared them. The dry buffer's
    /// flag is cleared here, a channel behind a load being alive and its next beat as far
    /// off as the load's end.
    [[gnu::always_inline]] static bool transmit_complete() {
        if (TxEngine::running()) {
            if (TxEngine::waiting() && S::dre_flag() && S::txc_flag()) {
                return end_on_fault();
            }
            S::clear_flags(SpiFlag::txc);
            if (TxEngine::running()) {
                return false;
            }
            last_frame_out();
        }
        if constexpr (has_rx_engine) {
            if (tenure_reads_) {
                // RXC is up by now, and the channel's beat takes a few bus cycles: 64
                // turns are hundreds of core cycles. One that does not end has missed a
                // character the receiver overflowed on.
                uint8_t left = 64u;
                while (!RxEngine::idle()) {
                    if (--left == 0u) {
                        return end_on_fault();
                    }
                }
            } else {
                S::flush_rx();
            }
        } else {
            S::flush_rx();
        }
        (void)TxEngine::complete();
        S::enable_txc_interrupt(false);
        tenure_dc_.set();   // a command alone leaves D/C where a polled one does
        tenure_cs_.set();
        status_ = spi_ok;   // over a refusal that wrote status() while this one flew
        dma_active_ = false;
        return true;
    }

    /// The last beat landed around the store that cleared a dry buffer's TXC, and that
    /// store may have taken the last frame's end: the frame is waited out on the flag,
    /// turns_a_frame_ looks covering it, and either way it is over when this returns.
    [[gnu::noinline, gnu::cold]] static void last_frame_out() {
        for (uint32_t left = turns_a_frame_; left != 0u && !S::txc_flag(); --left) {
        }
    }

    /// A channel's bus error, or a channel found dead: the vector decides and does not work
    /// (the class comment). The channels put away, the select released, a reset owed.
    [[gnu::noinline, gnu::cold]] static bool end_on_fault() {
        S::enable_txc_interrupt(false);
        put_channels_away();
        tenure_dc_.set();
        tenure_cs_.set();
        status_ = spi_dma_fault;
        applied_key_ = reset_owed_key;
        dma_active_ = false;
        return true;
    }

    /// Both channels reset and re-armed, whichever of them took the fault or was running.
    [[gnu::noinline, gnu::cold]] static void put_channels_away() {
        (void)TxEngine::abandon();
        if constexpr (has_rx_engine) {
            RxEngine::abandon();
        }
    }

    /// THE ONE RESET SEQUENCE (the class comment), cold and out of line: SWRST (32.6.2.2),
    /// what stood in the buffers and the shifter going with the registers' values, the
    /// applied configuration written back, the instance enabled, the receiver empty. The
    /// caller has released the select. False leaves a reset owed: SWRST did not complete, the
    /// core clock stopped, and nothing else may be written to the instance until it does.
    [[gnu::noinline, gnu::cold]] static bool reset_block() {
        if (S::reset() && S::configure(applied_) && S::enable(true)) {
            S::flush_rx();
            applied_key_ = key_of(applied_.mode, applied_.baud);
            return true;
        }
        applied_key_ = reset_owed_key;
        return false;
    }

    /// The last request's status: written in start() and at the end of a request on the
    /// engines, read by the arbiter after it returns or in the vector after the end.
    static inline volatile uint8_t status_ = spi_ok;
    /// A request is on the engines: from its launch to its end in the vector.
    static inline volatile bool dma_active_ = false;
    /// It reads: the receive channel's last beat is part of its end.
    static inline bool tenure_reads_ = false;
    /// The pins it keeps for its end.
    static inline PinRef tenure_cs_{};
    static inline PinRef tenure_dc_{};
    /// The dummy a null tx sends (0xFF): const, so it costs no RAM, and the DMAC reads the
    /// flash as well as the SRAM.
    static constexpr uint8_t tx_dummy_ = 0xFFu;
    /// The configuration really in the registers - the record a re-init
    /// or recover() writes wholesale - and its mode-and-rate half-word,
    /// the one thing apply() compares.
    static inline SpiConfig applied_{};
    static inline volatile uint16_t applied_key_ = 0;
    /// The polled budget per frame at the rate applied (the class comment).
    static inline uint32_t turns_a_frame_ = 0;
    static inline uint32_t ref_hz_ = 0;
    static inline uint32_t ceiling_hz_ = 0;
    static inline std::optional<uint8_t> ceiling_{};
    static inline uint8_t floor_ = 0;
    static inline DelayRate cs_rate_{};
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
 * THERE IS NO DEMOTION HERE - no host that sees its SS pin pulled low
 * becomes a client on the spot and has to be reported and undone. The
 * role is CTRLA.MODE, written while the SERCOM is disabled, and a low SS
 * on a host with MSSEN = 0 is simply a pin nobody is looking at. What the
 * silicon offers instead is CTRLB.SSDE - a client that WAKES on the
 * select edge - which is a different feature answering a different
 * question.
 *
 * The surface is polled plus ISR bodies, deliberately thin: a client is
 * a protocol, and which protocol is the application's. Its unit is the
 * RUN - answer(), exchange(), receive() serve a transaction's characters
 * in one call, polled, one ahead on the boundary - and the per-character
 * verbs are the degenerate case. ISR wiring is the
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

    /// Bring the instance up as a client.
    ///
    /// The core clock is still required even though the SHIFT register
    /// runs on the host's SCK: 32.5.3 says GCLK_SERCOMx_CORE "is required
    /// to clock the SPI", and every synchronization in the peripheral
    /// crosses into it - a client without it never finishes enabling.
    ///
    /// False when the configuration is refused or a synchronization did
    /// not complete.
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

    /// HOW MANY ANSWERS MUST BE QUEUED AHEAD of the character the host is
    /// about to clock for them to reach the wire in order - the one
    /// integer that differs between this family's client pump and the
    /// other strata's. TWO here: the three-SCK-cycle rule makes a write
    /// made in the inter-character gap mature one character LATE, so the
    /// pump preloads b0 into the shifter, parks b1 in DATA at once, and
    /// on every received character writes the next-plus-one.
    static constexpr uint8_t frames_ahead = 2;

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

    // ---- the polled runs ---------------------------------------------------------
    //
    // A client's unit is the RUN: the answers to a transaction, or what
    // it brings, served polled in one call - the per-character verbs
    // above being its degenerate case. What bounds a run here is
    // 32.6.2.6.2: a character written to DATA reaches the shifter only
    // if it was written "with at least three SCK clock cycles left in
    // the current character", and DRE rises when the character before it
    // has moved to the shifter - three CLK_SERCOM_APB cycles after the
    // character boundary. So the NEXT answer must land within the first
    // five SCK cycles of the character in flight, less those three
    // cycles: 17 core cycles at 12 MHz on a 48 MHz core, 37 at 6, 77 at
    // 3. The loops are written to that window: the flag is polled four
    // times for each look at the select and the budget, the path from
    // the read that sees it to the store is a branch (answer() holds the
    // next answer in a register before the poll; exchange()'s turn,
    // short of registers, loads it after - two cycles), and nothing in
    // a loop is a call. Counted in the release listing and measured
    // against the wire in docs/samc21/spi.md.
    //
    // THEY ARE ALWAYS INLINE so a program may place the function that
    // calls them in SRAM (`.fast_text`, docs/samc21/platform.md), where a
    // turn costs less than from the flash behind two wait states
    // (docs/samc21/spi.md measures both against the window above).
    // Placement is the program's call, as for a handler: a run is inline
    // in its caller, and the caller is the program's.

    /// How a run ended.
    enum class RunEnd : uint8_t {
        complete,   ///< every character of the run moved (exchange, receive)
        closed,     ///< the host raised the select: INTFLAG.TXC (32.8.6)
        stalled,    ///< the spin budget ran out with neither flag raised
    };

    /// What a run did: `count` characters - answers loaded into DATA for
    /// answer(), characters received for exchange() and receive() - and
    /// how it ended.
    struct Run {
        uint16_t count;
        RunEnd end;
    };

    /**
     * ANSWER a transaction with `out`, polled on DRE alone, reading
     * nothing. Call it with the select still HIGH: with
     * Config::preload the first write goes straight into the shifter
     * (32.6.3.2) and the second parks in DATA at once, so the run is on
     * the wire from the first character and one ahead from then on (the
     * class comment); without it the shifter's leftover leads, as the
     * chapter says. Every later answer is written on the DRE that marks
     * a character boundary, inside the five-cycle window above.
     *
     * The run ENDS WHEN THE HOST RAISES THE SELECT (TXC: set at SS high,
     * 32.8.6 - cleared on entry, so a former transaction's close does not
     * end this one), however many characters it clocked: fewer than the
     * run, and `count` says how many answers were loaded; more, and the
     * extra characters carry "the previously received character"
     * (32.6.2.6.2) - a caller that wants a fill pads its run. The run
     * keeps DATA full one character ahead, so a host that stops early
     * raises the select with DRE down and the poll reads TXC alone; a
     * select raised while DATA is FREE - TXC standing beside DRE when the
     * poll reads them, or rising between that read and the store - is
     * cleared by the store (32.8.6), and the run then ends `stalled`.
     *
     * NOTHING IS READ while it runs: the two-level receive buffer fills
     * after two characters and STATUS.BUFOVF rises (32.6.2.7) - what the
     * host sent is LOST, the price of a one-flag turn. The tail drains
     * the receiver and clears BUFOVF and ERROR, so the next run or poll
     * starts clean.
     *
     * `spins` bounds the whole run, counted down once per idle poll: a
     * host that never selects, or never closes, ends it `stalled`.
     */
    template <typename T>
    [[gnu::always_inline]] static Run answer(std::span<const T> out, uint32_t spins) {
        static_assert(run_char<T>, "a client's character is eight bits or nine "
                                   "(CTRLB.CHSIZE): uint8_t or uint16_t");
        auto& regs = S::regs();
        const T* const first = out.data();
        const T* const end = first + out.size();
        const T* p = first;
        RunEnd how = RunEnd::closed;
        S::clear_flags(SpiFlag::txc);
        for (; p != end; ++p) {
            T next = *p;
            in_register(next);
            if (!await<SpiFlag::dre>(spins, how)) {
                goto tail;
            }
            regs.SERCOM_DATA = next;
        }
        // Every answer is loaded, the last two still to be clocked: the
        // host's close is the end.
        while ((regs.SERCOM_INTFLAG & SpiFlag::txc) == 0u) {
            if (--spins == 0u) {
                how = RunEnd::stalled;
                break;
            }
        }
    tail:
        const Run r{static_cast<uint16_t>(p - first), how};
        S::flush_rx();
        return r;
    }

    /**
     * EXCHANGE: answer with `out` and keep what the host sends in `in`,
     * the run's length the shorter of the two. The answers go exactly as
     * answer()'s - on DRE, inside the window, the first with the select
     * high - and each turn then reads the character that completed at
     * the same boundary (RXC stands by then: a character is transferred
     * "within the same clock cycle its last data bit is received",
     * 32.6.2.6.2), two behind the write, the receive buffer's depth. So a
     * turn has two polls and a read where answer()'s has one poll; it
     * must stay under a character (32 core cycles at 12 MHz) as well as
     * land its write inside the window.
     *
     * Ends `complete` when the last character is in; `closed` when the
     * host raised the select first, what it had sent read out of the
     * buffer into `in` (`count` says how far); `stalled` on the budget.
     * The receiver is flushed at the start, with the select high, so
     * `in` holds this transaction alone; the first write clears a former
     * transaction's TXC (32.8.6).
     */
    template <typename T>
    [[gnu::always_inline]] static Run exchange(std::span<const T> out, std::span<T> in,
                                               uint32_t spins) {
        static_assert(run_char<T>, "a client's character is eight bits or nine "
                                   "(CTRLB.CHSIZE): uint8_t or uint16_t");
        const uint32_t len = out.size() < in.size() ? out.size() : in.size();
        if (len == 0u) {
            return {0, RunEnd::complete};
        }
        S::flush_rx();
        auto& regs = S::regs();
        const T* p = out.data();
        const T* const p_end = p + len;
        T* const first = in.data();
        T* const q_end = first + len;
        T* q = first;
        RunEnd how = RunEnd::complete;
        regs.SERCOM_DATA = *p++;
        // The second answer: parked in DATA at once under preload, at the
        // first boundary without it.
        if (p != p_end) {
            T next = *p;
            in_register(next);
            if (!await<SpiFlag::dre>(spins, how)) {
                goto drain;
            }
            regs.SERCOM_DATA = next;
            ++p;
        }
        // The steady state: an answer on each boundary, then the character
        // two behind it read - two pointers and the end in the loop, the
        // rest of the run's state outside it, so the turn fits the core's
        // eight low registers.
        for (; p != p_end; ++p, ++q) {
            const T next = *p;
            if (!await<SpiFlag::dre>(spins, how)) {
                goto drain;
            }
            regs.SERCOM_DATA = next;
            if (!await<SpiFlag::rxc>(spins, how)) {
                goto drain;
            }
            *q = static_cast<T>(regs.SERCOM_DATA);
        }
        // The last two characters, nothing left to write.
        for (; q != q_end; ++q) {
            if (!await<SpiFlag::rxc>(spins, how)) {
                goto drain;
            }
            *q = static_cast<T>(regs.SERCOM_DATA);
        }
        return {static_cast<uint16_t>(q - first), RunEnd::complete};
    drain:
        // What arrived before the close or the stall is still in the
        // buffer: it belongs to the run.
        while (q != q_end && S::rxc_flag()) {
            *q++ = static_cast<T>(regs.SERCOM_DATA);
        }
        return {static_cast<uint16_t>(q - first), how};
    }

    /**
     * RECEIVE a run into `in`, polled on RXC alone, writing nothing - the
     * shifter then sends the previously received character back
     * (32.6.2.6.2), so a host reading during a receive-only run reads an
     * echo one character late. The three-SCK-cycle window does not bind
     * here (no write is timed); the receive buffer does: a turn must
     * average under a character, and a stall longer than two lets the
     * third overflow (BUFOVF, 32.6.2.7). Ends as exchange() does; the
     * receiver is NOT flushed at the start, so what stood in it when the
     * run began is the run's first character.
     */
    template <typename T>
    [[gnu::always_inline]] static Run receive(std::span<T> in, uint32_t spins) {
        static_assert(run_char<T>, "a client's character is eight bits or nine "
                                   "(CTRLB.CHSIZE): uint8_t or uint16_t");
        auto& regs = S::regs();
        T* const first = in.data();
        T* const q_end = first + in.size();
        T* q = first;
        RunEnd how = RunEnd::complete;
        for (; q != q_end; ++q) {
            if (!await<SpiFlag::rxc>(spins, how)) {
                break;
            }
            *q = static_cast<T>(regs.SERCOM_DATA);
        }
        return {static_cast<uint16_t>(q - first), how};
    }

    // ---- the ISR bodies --------------------------------------------------------

    /// The instance's ONE interrupt body - call from
    /// SERCOMn_Handler().
    ///
    /// A client's protocol is the application's, so this body does not
    /// decide anything: it reports WHICH sources are both raised and
    /// enabled and lets the app's glue act. The received character is not
    /// consumed here - reading DATA is the app's, because only the app
    /// knows where the character goes.
    ///
    /// Returns the pending mask (SpiFlag::rxc | ::txc | ::ssl | ...), 0
    /// when this SERCOM was not the one asking.
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
    /// The two character types a run may carry: the beat is the element.
    template <typename T>
    static constexpr bool run_char = std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>;

    /// Hold `v` in a register from here on: the next answer is loaded
    /// BEFORE the poll, so the path from the flag to the store carries no
    /// load of it (the compiler sinks the load past the poll otherwise -
    /// two cycles of the five-SCK-cycle window, read in the listing).
    template <typename T>
    [[gnu::always_inline]] static void in_register(T& v) {
        __asm__ volatile("" : "+r"(v));
    }

    /// Poll INTFLAG until `flag` (DRE or RXC) stands: true. False when
    /// the host raised the select first (TXC) or the run's budget ran
    /// out, `how` saying which. FOUR POLLS A COUNT: the select and the
    /// budget are asked once every four reads, so the read that sees the
    /// flag follows the one before it by a load, a shift and a branch,
    /// and the branch out of the poll is the whole path to the store.
    /// Each flag is tested by shifting it into the sign bit, not against
    /// a mask: a mask held in a register for the whole run pushed the
    /// loaded answer out to the stack on this core's eight low registers,
    /// and a reload from there between the flag and the store (the
    /// listing).
    template <uint8_t flag>
    [[gnu::always_inline]] static bool await(uint32_t& spins, RunEnd& how) {
        constexpr int shift = 31 - std::countr_zero(flag);
        constexpr int txc_shift = 31 - std::countr_zero(SpiFlag::txc);
        auto& regs = S::regs();
        const auto has = [](uint32_t f, int sh) {
            return static_cast<int32_t>(f << sh) < 0;
        };
        for (;;) {
            if (has(regs.SERCOM_INTFLAG, shift)) {
                return true;
            }
            if (has(regs.SERCOM_INTFLAG, shift)) {
                return true;
            }
            if (has(regs.SERCOM_INTFLAG, shift)) {
                return true;
            }
            const uint32_t f = regs.SERCOM_INTFLAG;
            if (has(f, shift)) {
                return true;
            }
            if (has(f, txc_shift)) {
                how = RunEnd::closed;
                return false;
            }
            if (--spins == 0u) {
                how = RunEnd::stalled;
                return false;
            }
        }
    }

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
