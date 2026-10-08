/*
 * sercom.hpp
 *
 * The SAM C21 SERCOM in USART mode (DS60001479M ch. 31, with the baud
 * generator and the pad matrix of the shared ch. 30), in the two strata
 * the rest of this target uses:
 *
 *  Sercom<n>   the RESOURCE - a typed view of one instance's USART_INT
 *              register set: the bus and core clocks it needs before it
 *              answers at all, the reset/enable discipline with its
 *              SYNCBUSY waits, the whole configuration in one struct,
 *              the flag verbs, and the ONE combined interrupt question
 *              this core's single vector has to ask.
 *
 *  Uart<n, pads, rx_size, tx_size>
 *              the TASK - the interrupt-driven full-duplex byte
 *              transport every console sits on: two SPSC rings
 *              (util/ring.hpp, lock-free here: atomic_width is 4),
 *              error counters, an init() that speaks HERTZ, and the
 *              ByteSink/ByteSource surface print.hpp writes to.
 *
 * SCOPE, honestly. USART mode only, asynchronous, internal clock, 16x
 * ARITHMETIC oversampling - what a console needs, and no more. The
 * SERCOM's other personalities are whole chapters of their own and get
 * their own headers - SPI host/client is ch. 32 in samc21/spi.hpp, which
 * reaches the registers through Sercom<n>::spi_regs() and shares this
 * class's per-instance facts (clocks, NVIC line); I2C host/client is
 * ch. 33, in samc21/i2c.hpp. Inside
 * USART mode, what is NOT BUILT rather than half-built (the fractional
 * and 3x baud regimes, the synchronous role, handshaking, RS485, LIN,
 * IrDA, auto-baud, start-of-frame detection) is listed with its reasons
 * in docs/samc21/sercom.md. The baud arithmetic below names its
 * oversampling explicitly so a second regime slots in without moving
 * anything.
 *
 * ONE INTERRUPT VECTOR, NOT TWO, whatever a two-vector transport would
 * lead one to expect. A SERCOM has exactly ONE line in the NVIC
 * (SERCOM5_IRQn = 14 here) shared by DRE, TXC, RXC, RXS, CTSIC, RXBRK and
 * ERROR - so the task offers ONE ISR body, Uart::isr(), which reads
 * INTFLAG, masks it with INTENSET (a raised flag nobody asked for must
 * not be acted on - DRE in particular reads 1 whenever the transmit
 * buffer is empty, which is most of the time) and serves whatever is
 * genuinely pending. The app binds exactly one vector:
 *
 *   extern "C" void SERCOM5_Handler() {
 *       if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
 *   }
 *
 * SYNCHRONIZATION IS REAL HERE. SWRST, ENABLE and CTRLB cross into the
 * peripheral's own clock domain, and 31.8.10 promises an APB ERROR for
 * a CTRLB write issued while the previous one is still in flight. Every
 * such write in this file is followed by a BOUNDED wait (clock.hpp's
 * clock_wait, the same helper and the same discipline the clock tree
 * uses - it waits on a synchronization bit, whichever peripheral owns
 * it). A wait that times out is REPORTED, never hung on.
 *
 * PADS ARE NOT PINS. The SERCOM knows four pads and nothing else: which
 * pad carries TxD is CTRLA.TXPO, which one carries RxD is CTRLA.RXPO,
 * and those two fields are all this resource has to say about routing.
 * Which PIN a pad is bonded to, and through which PMUX function, is a
 * fact of the package's I/O multiplexing table - the device header's
 * PIN_P<pad><fn>_SERCOM<n>_PAD<k> symbols - and it is handed to the
 * task as the UartPads value below, whose two SercomPadPin members the
 * task turns into ordinary Pin<>::function() calls. A generic per-
 * package pad table is the same device-table job pin.hpp leaves open
 * for pin bonding, and it is equally not built here: naming the pin is
 * the application's, because only the board knows.
 *
 * TxD CANNOT GO ANYWHERE. CTRLA.TXPO (31.8.1) offers TxD on SERCOM
 * PAD[0] or PAD[2] and on no other pad, while RxD may be any of the
 * four. So a two-wire link is never free to swap its two pads: with the
 * console's PAD[0]/PAD[1] pair, PAD[0] IS the transmitter and PAD[1] the
 * receiver, and "the other way round" is not a configuration the silicon
 * offers - it is a different pad pair. uart_pads_valid() refuses the
 * impossible one at compile time instead of letting it fail on the wire.
 * NOTE the device header's own naming trap: it calls TXPO = 0x1
 * `TXPO_PAD1_Val`, but that code puts TxD on SERCOM PAD[2] - the
 * enumerator names the CODE, not the pad. Everything below is named
 * after the pad the SIGNAL lands on.
 *
 * Facts that shape the code (DS60001479M 30.6.2.3, 31.5.1, 31.6.2.1,
 * 31.8.x, and errata DS80000740S, silicon rev F on the bench chip):
 *  - CTRLA, CTRLB and BAUD are ENABLE-PROTECTED (31.6.2.1): a write
 *    while the peripheral is enabled is DISCARDED. Everything that
 *    changes them here disables first;
 *  - the input pull of a SERCOM input pin can only be a pull-DOWN
 *    (31.5.1): an idle RxD line has to be held high by whatever drives
 *    it, there is no internal pull-up to lean on;
 *  - STATUS carries the error bits OF THE CHARACTER AT THE HEAD of the
 *    two-deep receive FIFO and must be read BEFORE DATA (31.8.11);
 *    reading DATA is also what clears INTFLAG.RXC;
 *  - INTFLAG.TXC is cleared by WRITING DATA and set when the shifter
 *    has emptied with nothing new queued (31.8.8) - which makes it an
 *    exact "the last byte is on the wire" answer, and is why rebase()
 *    needs no timing guesswork;
 *  - erratum 1.17.15: the ERROR interrupt does not wake the device, and
 *    the documented work-around is to take the errors on RXC and read
 *    STATUS - which is what this driver does, so the ERROR interrupt is
 *    never enabled at all;
 *  - erratum 1.17.16: CTRLA.SWRST does NOTHING while CTRLA.ENABLE = 0.
 *    reset() therefore enables the instance first when it finds it
 *    disabled - see the comment there;
 *  - erratum 1.17.4: DBGCTRL.DBGSTOP does not actually halt transmission
 *    in Debug mode. The field is exposed and the erratum named, because
 *    a fact with no code behind it must at least be visible;
 *  - erratum 1.17.14 (standby over-consumption with RUNSTDBY = 0 and the
 *    receiver disabled) is a sleep-current fact: nothing here sleeps;
 *  - erratum 1.10.4 (the DMAC's, concurrent channel triggers corrupting
 *    the write-back): the Uart has ONE optional engine slot, the
 *    TRANSMIT one, filled by samc21/dmac.hpp's DmaTxEngine - the one DMA
 *    user this stratum admits, on one channel, claimed by one owner at a
 *    time. The receiver is the interrupt receiver, always.
 */

#pragma once

#include <stdint.h>

#include <cstring>
#include <span>
#include <optional>

#include "sam.h"

#include "samc21/clock.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/platform.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"

namespace brio {

// =============================================================================
// Which instances exist
// =============================================================================

/// SERCOM instances on THIS device. The device header is the authority:
/// it declares an instance by defining its <INSTANCE>_REGS address, and
/// the count differs across the family (four on the E package, six on
/// the G and J). Every per-instance lookup below is gated on the same
/// symbols, so an instance that is not bonded never names a register.
#if defined(SERCOM5_REGS)
inline constexpr uint8_t sercom_count = 6;
#elif defined(SERCOM4_REGS)
inline constexpr uint8_t sercom_count = 5;
#elif defined(SERCOM3_REGS)
inline constexpr uint8_t sercom_count = 4;
#elif defined(SERCOM2_REGS)
inline constexpr uint8_t sercom_count = 3;
#else
inline constexpr uint8_t sercom_count = 2;
#endif

// =============================================================================
// Pads, pins and the frame
// =============================================================================

/// The four pads a SERCOM knows. RXPO takes any of them; TXPO does not
/// (see the file header).
enum class SercomPad : uint8_t { pad0 = 0, pad1 = 1, pad2 = 2, pad3 = 3 };

/// Where one pad is bonded on THIS board: a PORT pin and the PMUX
/// function that reaches the SERCOM from it. Both come from the device
/// header's I/O multiplexing symbols (PIN_PB30D_SERCOM5_PAD0 and its
/// MUX_ twin say "PB30, function D"); an application states them, and
/// may cross-check its statement against those very macros - see the
/// console app for the shape of that assertion.
struct SercomPadPin {
    char port = 'A';
    uint8_t pin = 0;
    PinFunction function = PinFunction::d;

    constexpr bool valid() const { return port_exists(port) && pin < 32u; }
};

/// A two-wire asynchronous link: which pad carries each direction, and
/// where those two pads come out.
struct UartPads {
    SercomPad tx = SercomPad::pad0;
    SercomPad rx = SercomPad::pad1;
    SercomPadPin tx_pin{};
    SercomPadPin rx_pin{};
};

/// TxD lives on PAD[0] or PAD[2] and nowhere else (CTRLA.TXPO, 31.8.1).
constexpr bool uart_tx_pad_exists(SercomPad p) {
    return p == SercomPad::pad0 || p == SercomPad::pad2;
}

/// The TXPO code that puts TxD on `p`. The two codes this driver can
/// use are the two that leave RTS/CTS/TE out of the picture.
constexpr uint8_t uart_txpo(SercomPad p) {
    return p == SercomPad::pad2
               ? static_cast<uint8_t>(SERCOM_USART_INT_CTRLA_TXPO_PAD1_Val)
               : static_cast<uint8_t>(SERCOM_USART_INT_CTRLA_TXPO_PAD0_Val);
}

/// RXPO is simply the pad number (31.8.1).
constexpr uint8_t uart_rxpo(SercomPad p) { return static_cast<uint8_t>(p); }

/// The LOOP-BACK pair: RxD on the very pad TxD drives, both directions
/// on one pin (31.6.3.8: "configure RXPO and TXPO to use the same data
/// pins for transmit and receive. The loop-back is through the pad, so
/// the signal is also available externally"). The receiver hears its own
/// transmitter with no wire - a loop at every rate the generator makes.
constexpr bool uart_pads_loop_back(const UartPads& p) {
    return p.tx == p.rx && p.tx_pin.port == p.rx_pin.port && p.tx_pin.pin == p.rx_pin.pin &&
           p.tx_pin.function == p.rx_pin.function;
}

/// Is this pad/pin pair something the silicon and the package can do?
/// The pad side is exact; the pin side is checked only as far as this
/// header can know it (the group exists, the pin number is in range) -
/// that a given PIN really reaches that PAD is the open device-table
/// question of the file header. Two directions on one pad are the
/// loop-back above, and only with one pin named twice.
constexpr bool uart_pads_valid(const UartPads& p) {
    return uart_tx_pad_exists(p.tx) && (p.tx != p.rx || uart_pads_loop_back(p)) &&
           p.tx_pin.valid() && p.rx_pin.valid();
}

/// CTRLB.CHSIZE (31.8.2). The codes are not the bit counts and are not
/// contiguous: the device header publishes no enumerators for them, so
/// the datasheet's own table is transcribed here.
enum class UartBits : uint8_t {
    eight = 0x0,
    nine = 0x1,
    five = 0x5,
    six = 0x6,
    seven = 0x7,
};

/// Parity is TWO registers: CTRLA.FORM turns the parity bit on, CTRLB
/// PMODE picks its sense. One enum here, because the two must agree and
/// only the driver can guarantee they do.
enum class UartParity : uint8_t { none, even, odd };

/// The frame, in the application's terms.
struct UartFormat {
    UartBits bits = UartBits::eight;
    UartParity parity = UartParity::none;
    bool two_stop = false;    ///< CTRLB.SBMODE (the receiver ignores it)
    /// CTRLA.DORD. A standard UART frame is LSB-FIRST on the wire, and
    /// that is what the chapter's own init sequence prescribes - but
    /// the BIT's reset value is MSB-first, so the default here must
    /// say it out loud. Bench-caught: with this defaulted false the
    /// banner arrived exactly bit-reversed (0x0D 0x0A 'S' read back as
    /// 0xB0 0x50 0xCA), every other register verified correct over SWD.
    bool lsb_first = true;
};

/// Everything one instance is configured with. `baud` is the BAUD
/// REGISTER value - sercom_baud_reg() computes it from a reference rate
/// and a bit rate: the resource speaks the register, the task speaks
/// hertz.
struct SercomUartConfig {
    UartPads pads{};
    UartFormat format{};
    uint16_t baud = 0;
    bool rx = true;                   ///< CTRLB.RXEN
    bool tx = true;                   ///< CTRLB.TXEN
    bool run_standby = false;         ///< CTRLA.RUNSTDBY (see erratum 1.17.14)
    bool immediate_overflow = false;  ///< CTRLA.IBON
    bool debug_stop = false;          ///< DBGCTRL.DBGSTOP (see erratum 1.17.4)
};

// =============================================================================
// The baud arithmetic (pure: no register is touched below this line)
// =============================================================================

/**
 * floor(n x 65536 / d) rounded to nearest, for n < d.
 *
 * THE WIDTH IS THE WHOLE PROBLEM, so it is spelled out. The baud
 * equation below needs a numerator of n x 65536 - about 2^42 at a
 * 48 MHz reference - which neither 16 nor 32 bits hold, and which
 * `uint64_t` would hold only by dragging a 64-bit division
 * (__aeabi_uldivmod) into a driver that computes this once. So the
 * product is never FORMED: sixteen restoring-division steps double a
 * REMAINDER that is always smaller than d, which makes 2 x d the widest
 * value that ever exists, and uint32_t is named explicitly for it. The
 * result is exact.
 *
 * Cost, measured rather than assumed: in a constant-expression context
 * (a static_assert, a `constexpr` BAUD an application spells itself) it
 * evaluates entirely at compile time and costs nothing; called with the
 * rate as an ordinary argument, gcc at -Os does NOT unroll it, so it
 * stays a sixteen-iteration loop of about thirty bytes, run once per
 * init(). Both are cheaper than the library division the wide product
 * would have needed.
 *
 * Preconditions (guaranteed by the only caller): n < d and 2 x d fits
 * in 32 bits.
 */
constexpr uint16_t sercom_scale_65536(uint32_t n, uint32_t d) {
    if (d == 0u || n >= d) {
        return 0;
    }
    uint32_t rem = n;
    uint32_t quot = 0;
    for (uint8_t step = 0; step < 16u; ++step) {
        quot <<= 1;
        rem <<= 1;
        if (rem >= d) {
            rem -= d;
            quot |= 1u;
        }
    }
    if (2u * rem >= d) {
        ++quot;   // round to nearest: the granularity is one 65536th
    }
    return static_cast<uint16_t>(quot > 0xFFFFu ? 0xFFFFu : quot);
}

/**
 * The BAUD register value for a bit rate, in 16x ARITHMETIC
 * oversampling (CTRLA.SAMPR = 0x0), from table 30-2:
 *
 *     BAUD = 65536 x (1 - S x f_baud / f_ref)
 *
 * with S = 16 samples per bit and f_ref the SERCOM's core clock
 * (GCLK_SERCOMx_CORE, NOT the CPU clock in general - see Uart::init()).
 * Nullopt when the rate is out of the mode's own range (f_baud must not
 * exceed f_ref / S, table 30-2's condition column): 0 is a LEGAL BAUD
 * value here - it is the fastest rate the generator produces - so it
 * cannot double as a refusal, and std::optional says so.
 */
constexpr std::optional<uint16_t> sercom_baud_reg(uint32_t ref_hz, uint32_t baud,
                                                  uint8_t samples = 16) {
    if (ref_hz == 0u || baud == 0u || samples == 0u) {
        return {};
    }
    if (baud > ref_hz / samples) {
        return {};
    }
    const uint32_t consumed = static_cast<uint32_t>(samples) * baud;
    return sercom_scale_65536(ref_hz - consumed, ref_hz);
}

/**
 * What a BAUD register value really produces at a reference rate - the
 * generator's own answer, not the one that was asked for:
 *
 *     f_baud = f_ref x (65536 - BAUD) / (65536 x S)
 *
 * f_ref x (65536 - BAUD) is again too wide, and again is not formed:
 * f_ref is split at bit 16, the high half divides exactly and the low
 * one multiplies to at most 65535 x 65536 - the largest product 32 bits
 * still hold, which is why the split is at 16 and not elsewhere.
 */
constexpr uint32_t sercom_actual_baud(uint32_t ref_hz, uint16_t reg,
                                      uint8_t samples = 16) {
    if (samples == 0u) {
        return 0;
    }
    const uint32_t k = 65536u - static_cast<uint32_t>(reg);   // 1 .. 65536
    const uint32_t hi = ref_hz >> 16;
    const uint32_t lo = ref_hz & 0xFFFFu;
    const uint32_t scaled = hi * k + ((lo * k) >> 16);        // f_ref x k / 65536
    return scaled / samples;
}

/// The slowest core clock that can still produce `baud`: the mode's own
/// condition f_baud <= f_ref / S.
constexpr uint32_t sercom_min_ref_hz(uint32_t baud, uint8_t samples = 16) {
    return baud * samples;
}

// =============================================================================
// The resource
// =============================================================================

/// INTFLAG, INTENSET and INTENCLR share one bit layout (31.8.6 - 31.8.8)
/// and, on this core, one interrupt vector. These are the seven sources
/// that vector multiplexes.
struct SercomFlag {
    SercomFlag() = delete;

    static constexpr uint8_t dre = static_cast<uint8_t>(SERCOM_USART_INT_INTFLAG_DRE_Msk);
    static constexpr uint8_t txc = static_cast<uint8_t>(SERCOM_USART_INT_INTFLAG_TXC_Msk);
    static constexpr uint8_t rxc = static_cast<uint8_t>(SERCOM_USART_INT_INTFLAG_RXC_Msk);
    static constexpr uint8_t rxs = static_cast<uint8_t>(SERCOM_USART_INT_INTFLAG_RXS_Msk);
    static constexpr uint8_t ctsic = static_cast<uint8_t>(SERCOM_USART_INT_INTFLAG_CTSIC_Msk);
    static constexpr uint8_t rxbrk = static_cast<uint8_t>(SERCOM_USART_INT_INTFLAG_RXBRK_Msk);
    static constexpr uint8_t error = static_cast<uint8_t>(SERCOM_USART_INT_INTFLAG_ERROR_Msk);
    static constexpr uint8_t all = static_cast<uint8_t>(SERCOM_USART_INT_INTFLAG_Msk);
};

/// STATUS (31.8.9). Every bit here is write-one-to-clear except CTS,
/// which is a live pin level.
struct SercomStatus {
    SercomStatus() = delete;

    static constexpr uint16_t parity_error = SERCOM_USART_INT_STATUS_PERR_Msk;
    static constexpr uint16_t frame_error = SERCOM_USART_INT_STATUS_FERR_Msk;
    static constexpr uint16_t overflow = SERCOM_USART_INT_STATUS_BUFOVF_Msk;
    static constexpr uint16_t cts = SERCOM_USART_INT_STATUS_CTS_Msk;
    static constexpr uint16_t sync_field_error = SERCOM_USART_INT_STATUS_ISF_Msk;
    static constexpr uint16_t collision = SERCOM_USART_INT_STATUS_COLL_Msk;
    /// The three a plain 8N1 receiver can ever see.
    static constexpr uint16_t receive_errors = parity_error | frame_error | overflow;
};

/// CTRLA for a USART-mode configuration - ENABLE deliberately NOT
/// included: the whole register is written while the instance is
/// disabled (it is enable-protected), and enable() sets the bit
/// afterwards in a store of its own.
constexpr uint32_t sercom_uart_ctrla(const SercomUartConfig& c) {
    return SERCOM_USART_INT_CTRLA_MODE(SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK_Val) |
           SERCOM_USART_INT_CTRLA_CMODE(SERCOM_USART_INT_CTRLA_CMODE_ASYNC_Val) |
           SERCOM_USART_INT_CTRLA_SAMPR(SERCOM_USART_INT_CTRLA_SAMPR_16X_ARITHMETIC_Val) |
           SERCOM_USART_INT_CTRLA_TXPO(uart_txpo(c.pads.tx)) |
           SERCOM_USART_INT_CTRLA_RXPO(uart_rxpo(c.pads.rx)) |
           SERCOM_USART_INT_CTRLA_FORM(
               c.format.parity == UartParity::none
                   ? SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_NO_PARITY_Val
                   : SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_WITH_PARITY_Val) |
           (c.format.lsb_first ? SERCOM_USART_INT_CTRLA_DORD_Msk : 0u) |
           (c.run_standby ? SERCOM_USART_INT_CTRLA_RUNSTDBY_Msk : 0u) |
           (c.immediate_overflow ? SERCOM_USART_INT_CTRLA_IBON_Msk : 0u);
}

/// CTRLB, including the two direction enables.
constexpr uint32_t sercom_uart_ctrlb(const SercomUartConfig& c) {
    return SERCOM_USART_INT_CTRLB_CHSIZE(static_cast<uint32_t>(c.format.bits)) |
           (c.format.two_stop ? SERCOM_USART_INT_CTRLB_SBMODE_Msk : 0u) |
           (c.format.parity == UartParity::odd ? SERCOM_USART_INT_CTRLB_PMODE_Msk : 0u) |
           (c.tx ? SERCOM_USART_INT_CTRLB_TXEN_Msk : 0u) |
           (c.rx ? SERCOM_USART_INT_CTRLB_RXEN_Msk : 0u);
}

/**
 * One SERCOM instance, seen through its USART_INT register view.
 *
 * The instance answers nothing until BOTH its clocks run: the APB bus
 * clock (MCLK, or the registers do not respond) and the core clock
 * (a GCLK peripheral channel, or nothing ever synchronizes). init()
 * order on the task below is bus, core, reset, configure, enable -
 * and it is that order for those reasons.
 */
template <uint8_t n>
class Sercom {
    static_assert(n < sercom_count,
                  "no such SERCOM on this device: the E package has four "
                  "(SERCOM0..3), the G and J packages six");

public:
    Sercom() = delete;

    static constexpr uint8_t index = n;

    // ---- where this instance lives ---------------------------------------

    /**
     * The instance itself - the UNION of the SERCOM's four personalities
     * (the device header's `sercom_registers_t`: I2CM, I2CS, SPIS, SPIM,
     * USART_EXT, USART_INT at one address).
     *
     * The address ladder lives here, once, and each personality's header
     * picks its own member off it: this file takes USART_INT below,
     * samc21/spi.hpp takes SPIM. Everything that is a fact of the INSTANCE
     * rather than of a mode - the two clocks, the NVIC line, the DMAC
     * trigger codes - stays a member of this class and is shared by all
     * of them, which is what keeps a second personality from becoming a
     * second source of truth about which channel SERCOM5 is on.
     */
    static sercom_registers_t& instance() {
        if constexpr (n == 0) return *SERCOM0_REGS;
#if defined(SERCOM1_REGS)
        else if constexpr (n == 1) return *SERCOM1_REGS;
#endif
#if defined(SERCOM2_REGS)
        else if constexpr (n == 2) return *SERCOM2_REGS;
#endif
#if defined(SERCOM3_REGS)
        else if constexpr (n == 3) return *SERCOM3_REGS;
#endif
#if defined(SERCOM4_REGS)
        else if constexpr (n == 4) return *SERCOM4_REGS;
#endif
#if defined(SERCOM5_REGS)
        else if constexpr (n == 5) return *SERCOM5_REGS;
#endif
        else return *SERCOM0_REGS;
    }

    static sercom_usart_int_registers_t& regs() { return instance().USART_INT; }

    /// The same instance seen as an SPI HOST register set - the view
    /// samc21/spi.hpp works through. SPIM and SPIS have IDENTICAL layouts
    /// in this device header (the same offsets, the same field positions
    /// under two name prefixes), so ONE view serves both roles and the
    /// role is CTRLA.MODE and nothing else; spi.hpp static_asserts that
    /// agreement rather than trusting it.
    static sercom_spim_registers_t& spi_regs() { return instance().SPIM; }

    /// The instance seen as an I2C HOST and as an I2C CLIENT - the two
    /// views samc21/i2c.hpp works through. UNLIKE the SPI pair these are
    /// genuinely DIFFERENT register sets (the client has no BAUD and no
    /// bus state, the host no AMATCH machinery; STATUS and INTFLAG carry
    /// different bits at the same offsets), so both views exist and the
    /// role decides which one speaks the truth.
    static sercom_i2cm_registers_t& i2cm_regs() { return instance().I2CM; }
    static sercom_i2cs_registers_t& i2cs_regs() { return instance().I2CS; }

    /// The GCLK peripheral channel feeding this instance's SLOW clock -
    /// the one the I2C's three SMBus time-outs count on (33.6.3.1: it
    /// must run at 32.768 kHz). IT IS SHARED: SERCOM0..4 all sit on
    /// channel 18 and only SERCOM5 has its own (24), so routing it for
    /// one instance's time-outs routes it for every colleague's - a fact
    /// of the clock tree, stated here because this accessor is where a
    /// caller finds the channel.
    static constexpr uint8_t gclk_slow_id() {
        if constexpr (n == 0) return SERCOM0_GCLK_ID_SLOW;
#if defined(SERCOM1_REGS)
        else if constexpr (n == 1) return SERCOM1_GCLK_ID_SLOW;
#endif
#if defined(SERCOM2_REGS)
        else if constexpr (n == 2) return SERCOM2_GCLK_ID_SLOW;
#endif
#if defined(SERCOM3_REGS)
        else if constexpr (n == 3) return SERCOM3_GCLK_ID_SLOW;
#endif
#if defined(SERCOM4_REGS)
        else if constexpr (n == 4) return SERCOM4_GCLK_ID_SLOW;
#endif
#if defined(SERCOM5_REGS)
        else if constexpr (n == 5) return SERCOM5_GCLK_ID_SLOW;
#endif
        else return SERCOM0_GCLK_ID_SLOW;
    }

    /// The GCLK peripheral channel that feeds this instance's core
    /// clock. NOT a formula: the device header gives SERCOM0..4 the
    /// channels 19..23 but SERCOM5 the channel 25 (24 being its own
    /// private SLOW channel, where the others share 18), so each one is
    /// read off the header rather than computed.
    static constexpr uint8_t gclk_core_id() {
        if constexpr (n == 0) return SERCOM0_GCLK_ID_CORE;
#if defined(SERCOM1_REGS)
        else if constexpr (n == 1) return SERCOM1_GCLK_ID_CORE;
#endif
#if defined(SERCOM2_REGS)
        else if constexpr (n == 2) return SERCOM2_GCLK_ID_CORE;
#endif
#if defined(SERCOM3_REGS)
        else if constexpr (n == 3) return SERCOM3_GCLK_ID_CORE;
#endif
#if defined(SERCOM4_REGS)
        else if constexpr (n == 4) return SERCOM4_GCLK_ID_CORE;
#endif
#if defined(SERCOM5_REGS)
        else if constexpr (n == 5) return SERCOM5_GCLK_ID_CORE;
#endif
        else return SERCOM0_GCLK_ID_CORE;
    }

    /// This instance's bit in MCLK.APBCMASK - every SERCOM is on the
    /// APB-C bus (17.8.x).
    static constexpr uint32_t apb_mask() {
        if constexpr (n == 0) return MCLK_APBCMASK_SERCOM0_Msk;
#if defined(SERCOM1_REGS)
        else if constexpr (n == 1) return MCLK_APBCMASK_SERCOM1_Msk;
#endif
#if defined(SERCOM2_REGS)
        else if constexpr (n == 2) return MCLK_APBCMASK_SERCOM2_Msk;
#endif
#if defined(SERCOM3_REGS)
        else if constexpr (n == 3) return MCLK_APBCMASK_SERCOM3_Msk;
#endif
#if defined(SERCOM4_REGS)
        else if constexpr (n == 4) return MCLK_APBCMASK_SERCOM4_Msk;
#endif
#if defined(SERCOM5_REGS)
        else if constexpr (n == 5) return MCLK_APBCMASK_SERCOM5_Msk;
#endif
        else return MCLK_APBCMASK_SERCOM0_Msk;
    }

    /// This instance's DMAC transmit trigger: "the transmit buffer is
    /// free" (CHCTRLB.TRIGSRC, table 25-2) - the one trigger the transmit
    /// engine needs. A per-instance constant of the SERCOM's own device
    /// header (SERCOMn_DMAC_ID_TX, in instance/sercomN.h), read beside
    /// gclk_core_id() and apb_mask() the same way and never computed from
    /// n.
    static constexpr uint8_t dma_tx_trigger() {
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
        else return SERCOM0_DMAC_ID_TX;
    }

    /// The address the transmit engine writes: DATA, whose low byte a
    /// byte beat reaches - where an 8-bit frame lives.
    static volatile void* data_address() { return &regs().SERCOM_DATA; }

    /// The ONE NVIC line this instance raises - every interrupt source
    /// of the peripheral arrives on it (see the file header).
    static constexpr IRQn_Type irq() {
        if constexpr (n == 0) return SERCOM0_IRQn;
#if defined(SERCOM1_REGS)
        else if constexpr (n == 1) return SERCOM1_IRQn;
#endif
#if defined(SERCOM2_REGS)
        else if constexpr (n == 2) return SERCOM2_IRQn;
#endif
#if defined(SERCOM3_REGS)
        else if constexpr (n == 3) return SERCOM3_IRQn;
#endif
#if defined(SERCOM4_REGS)
        else if constexpr (n == 4) return SERCOM4_IRQn;
#endif
#if defined(SERCOM5_REGS)
        else if constexpr (n == 5) return SERCOM5_IRQn;
#endif
        else return SERCOM0_IRQn;
    }

    // ---- clocks -----------------------------------------------------------

    /// The APB bus clock: without it the registers do not answer at all.
    static void bus_clock(bool on) { Mclk::apb_c(apb_mask(), on); }

    /// The core clock: which GCLK generator drives the serial engine
    /// (and therefore every synchronization in this file). f_ref of the
    /// baud arithmetic IS this generator's rate.
    static bool core_clock(uint8_t generator) {
        return GclkChannel::connect(gclk_core_id(), generator);
    }

    // ---- synchronization (31.8.10) ----------------------------------------

    static bool sync_busy(uint32_t mask) { return (regs().SERCOM_SYNCBUSY & mask) != 0u; }

    /// Bounded, like every wait in this stratum: a synchronization that
    /// never completes is reported, never hung on. (clock_wait lives in
    /// samc21/clock.hpp because the clock tree needed it first; it waits
    /// on a synchronization bit, whichever peripheral owns it.)
    static bool wait_sync(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().SERCOM_SYNCBUSY, mask, false, spins);
    }

    // ---- reset and enable (31.6.2.2) --------------------------------------

    static bool enabled() {
        return (regs().SERCOM_CTRLA & SERCOM_USART_INT_CTRLA_ENABLE_Msk) != 0u;
    }

    /**
     * Everything back to its reset value (DBGCTRL excepted), instance
     * disabled.
     *
     * ERRATUM 1.17.16: SWRST does nothing while ENABLE = 0 - which is
     * exactly the state a freshly booted instance is in, so the obvious
     * "reset first, then configure" would silently do nothing and leave
     * whatever a debugger or a bootloader had set up. So a disabled
     * instance is ENABLED first, in USART internal-clock mode (the mode
     * matters: the synchronization the enable needs runs on the core
     * clock, and only the internal-clock mode uses it), and reset from
     * there. No pad is muxed to the SERCOM at this point in init(), so
     * the brief enable is invisible outside the chip.
     */
    static bool reset(uint32_t spins = 0xFFFFu) {
        if (!enabled()) {
            regs().SERCOM_CTRLA =
                SERCOM_USART_INT_CTRLA_MODE(SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK_Val) |
                SERCOM_USART_INT_CTRLA_ENABLE_Msk;
            if (!wait_sync(SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk, spins)) {
                return false;
            }
        }
        regs().SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST_Msk;
        return wait_sync(SERCOM_USART_INT_SYNCBUSY_SWRST_Msk, spins);
    }

    /**
     * ENABLE, waiting out BOTH synchronizations it triggers.
     *
     * The second one is easy to miss: 31.8.2 says that enabling the
     * peripheral CLEARS CTRLB.TXEN/RXEN and raises SYNCBUSY.CTRLB until
     * the two directions are really up. A driver that returned as soon
     * as SYNCBUSY.ENABLE cleared would hand back a transmitter that is
     * not enabled yet.
     */
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint32_t v = regs().SERCOM_CTRLA;
        regs().SERCOM_CTRLA = on ? (v | SERCOM_USART_INT_CTRLA_ENABLE_Msk)
                                 : (v & ~SERCOM_USART_INT_CTRLA_ENABLE_Msk);
        bool ok = wait_sync(SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk, spins);
        ok = wait_sync(SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk, spins) && ok;
        return ok;
    }

    // ---- configuration ----------------------------------------------------

    /**
     * Write the whole USART configuration. CTRLA, CTRLB and BAUD are
     * enable-protected (31.6.2.1) - a write while the instance runs is
     * DISCARDED, not refused - so this disables first and leaves the
     * instance disabled: enable(true) is a separate, deliberate step.
     *
     * False (and nothing programmed) when the pad choice is not one the
     * silicon offers, or when a synchronization did not complete. The
     * PORT side of the pads is NOT touched here: pads are the SERCOM's,
     * pins are PORT's and the task's.
     */
    static bool configure(const SercomUartConfig& cfg, uint32_t spins = 0xFFFFu) {
        if (!uart_pads_valid(cfg.pads)) {
            return false;
        }
        if (!enable(false, spins)) {
            return false;
        }
        regs().SERCOM_INTENCLR = SercomFlag::all;
        regs().SERCOM_CTRLA = sercom_uart_ctrla(cfg);
        regs().SERCOM_CTRLB = sercom_uart_ctrlb(cfg);
        // Writing CTRLB again before this clears is an APB ERROR
        // (31.8.10) - the one synchronization here that is not merely
        // about knowing when a setting took effect.
        if (!wait_sync(SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk, spins)) {
            return false;
        }
        regs().SERCOM_BAUD = cfg.baud;
        regs().SERCOM_DBGCTRL =
            cfg.debug_stop ? static_cast<uint8_t>(SERCOM_USART_INT_DBGCTRL_DBGSTOP_Msk) : 0u;
        clear_status(SercomStatus::receive_errors);
        regs().SERCOM_INTFLAG = SercomFlag::all;
        return true;
    }

    static uint16_t baud_reg() { return regs().SERCOM_BAUD; }
    /// Enable-protected: the instance must be disabled or the store is
    /// discarded (31.6.2.1).
    static void baud_reg(uint16_t v) { regs().SERCOM_BAUD = v; }

    /// Hand the instance back: interrupts off, disabled, core clock
    /// released, bus clock off. The pads' PINS are the task's to release.
    static void release(uint32_t spins = 0xFFFFu) {
        regs().SERCOM_INTENCLR = SercomFlag::all;
        (void)enable(false, spins);
        GclkChannel::disconnect(gclk_core_id());
        bus_clock(false);
    }

    // ---- interrupts (31.8.6 - 31.8.8) -------------------------------------

    /**
     * The flags that are BOTH raised and enabled - the one question a
     * shared vector has to ask, and the reason the answer cannot be
     * INTFLAG alone: DRE is a CONDITION, not an event, and reads 1
     * whenever the transmit buffer is empty. A handler that acted on
     * every raised flag would run the transmit path on every receive
     * interrupt.
     */
    [[gnu::always_inline]] static uint8_t pending() {
        return static_cast<uint8_t>(regs().SERCOM_INTFLAG & regs().SERCOM_INTENSET);
    }

    static uint8_t flags() { return regs().SERCOM_INTFLAG; }
    static uint8_t armed() { return regs().SERCOM_INTENSET; }

    /// The write-one-to-clear half of INTFLAG (TXC, RXS, CTSIC, RXBRK,
    /// ERROR). RXC and DRE are conditions and ignore a write: RXC is
    /// cleared by reading DATA, DRE by writing it.
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
    static void enable_dre_interrupt(bool on) { enable_interrupt(SercomFlag::dre, on); }
    static void enable_rxc_interrupt(bool on) { enable_interrupt(SercomFlag::rxc, on); }
    static void enable_txc_interrupt(bool on) { enable_interrupt(SercomFlag::txc, on); }

    static bool dre_flag() { return (regs().SERCOM_INTFLAG & SercomFlag::dre) != 0u; }
    static bool rxc_flag() { return (regs().SERCOM_INTFLAG & SercomFlag::rxc) != 0u; }
    /// Set when the shifter has emptied and DATA holds nothing new;
    /// cleared by writing DATA. An exact "the last byte is out".
    static bool txc_flag() { return (regs().SERCOM_INTFLAG & SercomFlag::txc) != 0u; }

    // ---- status and data --------------------------------------------------

    /// STATUS describes the character at the HEAD of the receive FIFO
    /// and must be read BEFORE data() (31.8.11).
    [[gnu::always_inline]] static uint16_t status() { return regs().SERCOM_STATUS; }
    /// Write-one-to-clear, as a plain store of just those bits.
    static void clear_status(uint16_t mask) { regs().SERCOM_STATUS = mask; }

    /// Reading DATA also clears INTFLAG.RXC; writing it clears
    /// INTFLAG.DRE and INTFLAG.TXC (31.6.2.4).
    [[gnu::always_inline]] static uint16_t data() { return regs().SERCOM_DATA; }
    [[gnu::always_inline]] static void data(uint16_t v) { regs().SERCOM_DATA = v; }

    /// Drain the two-deep receive FIFO (and its shifter's spill).
    static void flush_rx() {
        for (uint8_t i = 0; i < 4u && rxc_flag(); ++i) {
            (void)data();
        }
        clear_status(SercomStatus::receive_errors);
    }
};

// =============================================================================
// The task
// =============================================================================

/*
 * Uart<n, pads, rx_size, tx_size>
 *
 * The interrupt-driven full-duplex byte transport: 8N1 by default, SPSC
 * rings on both sides, ByteSink + ByteSource. It carries the Uart surface
 * every target's console sits on, with ONE combined isr() rather than a
 * per-condition pair, because this core gives the peripheral one vector
 * (see the file header).
 *
 *  - all state is `static inline` (one set per instantiation, in .bss,
 *    no constructor before main): hardware is touched ONLY by the
 *    explicit init(), called after the clock is up;
 *  - byte transport only - text formatting lives in util/print.hpp;
 *  - RX hardware error flags (frame / parity / buffer overflow) are
 *    counted; corrupted bytes (frame or parity) are dropped.
 *
 * The empty default constructor is intentionally AVAILABLE: an instance
 * carries no state and acts as a zero-cost tag so call sites read
 * naturally, e.g.
 *
 *   constexpr brio::UartPads console_pads{
 *       .tx = brio::SercomPad::pad0,
 *       .rx = brio::SercomPad::pad1,
 *       .tx_pin = {'B', 30, brio::PinFunction::d},
 *       .rx_pin = {'B', 31, brio::PinFunction::d},
 *   };
 *   using Serial = brio::Uart<5, console_pads>;
 *   constexpr Serial serial;                    // tag object (no state)
 *
 *   extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }
 *
 *   int main() {
 *       SysClock::init();
 *       Serial::init(clock, 115200);
 *       brio::enable_interrupts();
 *       brio::print(serial, "hello", brio::crlf);
 *   }
 *
 * TX policy: write_byte() has TRY semantics (false when the TX ring is
 * full, nothing counted - the caller decides whether to retry, drop or
 * block; print.hpp blocks). RX overflow (ring full, byte lost) IS
 * counted, as are the hardware error flags.
 *
 * Ring sizes: NOTHING here pushes them towards 256. This core reads a
 * word atomically (SamPlatform::atomic_width == 4), so util/ring.hpp
 * takes its lock-free path at every size and a larger ring costs only
 * RAM. 64/256 are kept as console-class defaults, not as a ceiling.
 */
/**
 * THE COPY'S CROSSOVER: the run length from which the transport's copy
 * into or out of a ring calls the runtime's memcpy instead of its own
 * byte loop (Uart::copy_run()), when the two ends share their alignment
 * in the word. Measured with bench_samc's letter u - write_bulk() timed
 * at nine lengths and the four source alignments, the figures in
 * docs/samc21/sercom.md: the inline byte loop costs about nine and a half
 * cycles a byte at 48 MHz behind two wait states; memcpy about 170 cycles
 * of call, tests and tail, then under a cycle a byte when the ends are
 * co-aligned (its word path) - so the two meet at about twenty bytes - and
 * eleven a byte when they are not (its byte path, the same loop out of
 * line), where it never pays and is not called.
 */
inline constexpr uint32_t uart_copy_crossover = 20;

/**
 * The "no DMA engine" default of the Uart's one optional engine slot.
 *
 * It is a TAG, not a base class: `present` is the only thing the task
 * asks about, and it asks with `if constexpr`, so every engine branch -
 * the pump, the completion path, the state they need - disappears from a
 * Uart that does not name one: an engineless image carries none of it.
 *
 * The real engine is samc21/dmac.hpp's DmaTxEngine, and it lives THERE
 * rather than here on purpose: sercom.hpp must not include dmac.hpp, or
 * every program with a serial port would carry the controller. An
 * application that wants the engine includes both headers and names it;
 * one that does not, never sees the DMAC at all.
 *
 * ONE SLOT, THE TRANSMIT ONE. Erratum 1.10.4 corrupts the live write-back
 * of channels triggered concurrently, measured on this transport's own
 * pair writing the SERCOM's registers (docs/samc21/dmac.md), so this
 * stratum drives the DMAC for one user on one channel: the bulk direction,
 * the one whose bytes the program produces. The receiver keeps its
 * interrupt, which takes every level of the two-level buffer an entry and
 * holds 3 Mbaud (docs/samc21/sercom.md).
 */
struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

template <uint8_t n, UartPads pads, uint32_t rx_size = 64, uint32_t tx_size = 256,
          typename TxEngine = NoDmaEngine>
class Uart {
    using S = Sercom<n>;

    static_assert(uart_pads_valid(pads),
                  "these SERCOM pads cannot carry an asynchronous link: TxD exists "
                  "only on PAD[0] and PAD[2] (CTRLA.TXPO), the two directions share a "
                  "pad only as the loop-back (31.6.3.8: one pin named for both), and "
                  "both pins must be real ones on this device");

    // AN ENGINE IS CHECKED WHERE IT IS NAMED. `sizeof` demands a
    // COMPLETE type, which instantiates the engine here, at the template
    // argument the application actually typed - so an engine's own
    // static_asserts (its channel number, above all) fire on that line.
    // Without this the engine stays uninstantiated until something first
    // touches it, and a Uart carrying an impossible channel compiled
    // perfectly happily; the family fixture's negative TU says so.
    static_assert(sizeof(TxEngine) > 0,
                  "the engine slot must name a complete type: samc21/dmac.hpp's "
                  "DmaTxEngine, or NoDmaEngine (the default)");
    // The engine moves a ring's run as ONE block, and BTCNT counts 65535
    // beats at most (25.6.1.1): an engined transmit ring is no longer.
    static_assert(!TxEngine::present || tx_size <= 65535u,
                  "an engined transmit ring holds at most 65535 bytes: the engine "
                  "moves a run of it as one DMA block, and BTCNT is 16 bits");

    using TxPin = Pin<pads.tx_pin.port, pads.tx_pin.pin>;
    using RxPin = Pin<pads.rx_pin.port, pads.rx_pin.pin>;

    // One ring pair per instantiation (static inline -> .bss, no ctor).
    //
    // THE RECEIVE RING SKIPS A LOSS when the handler fills it: a character
    // lost on the handler's rare paths (a full ring, a frame or parity
    // error, an overflow before the character in hand) is reported to the
    // ring (util/ring.hpp's SkipRing), whose consumer's next look skips
    // everything queued, the skip epoch moving between the last run before
    // the loss and the first after it.
    //
    // BOTH RINGS START ON A WORD BOUNDARY (alignas(4): a Ring opens with
    // its slots, and a SkipRing with its Ring): the run copy takes memcpy's word path only when
    // its two ends share their alignment in the word (copy_run()), and a
    // word-aligned source then meets a run starting at any slot index that
    // is a multiple of four - instead of whatever alignment the linker's
    // placement of the statics happened to give.
    alignas(4) static inline SkipRing<uint8_t, rx_size, SamPlatform> m_rx{};
    alignas(4) static inline Ring<uint8_t, tx_size, SamPlatform> m_tx{};

    // Error counters, written in the handler, read from the main loop.
    // A byte moves in one access on this core; they wrap at 255. Written
    // as `x = x + 1` because compound ops on volatile are deprecated in
    // C++20.
    static inline volatile uint8_t m_rx_overruns = 0;   // RX ring full, byte lost
    static inline volatile uint8_t m_frame_errors = 0;  // FERR: byte dropped
    static inline volatile uint8_t m_parity_errors = 0; // PERR: byte dropped
    static inline volatile uint8_t m_hw_overruns = 0;   // BUFOVF: bytes lost in HW
    /// DMA blocks abandoned because the silicon had stopped running them
    /// (see nudge_blocked_tx()). Touched only from inside `if constexpr
    /// (has_tx_engine)` branches, so an engineless Uart never odr-uses it
    /// and does not carry the byte.
    static inline volatile uint8_t m_dma_faults = 0;
    static inline uint32_t m_baud = 0;                  // for rebase()
    /// A byte was handed to the transmitter since init(). TXC is clear out
    /// of configure() and only a frame's departure sets it, so a port that
    /// has sent nothing reads idle by this and not by the flag. Written by
    /// the producer's verbs alone, in main context.
    static inline bool m_tx_used = false;

    /// The bound on a wait for the wire to fall idle (rebase(), set_baud()).
    static constexpr uint32_t tx_drain_spins = 8'000'000u;
    /// The longest run the engine moves as one block: half the ring. A
    /// block's completion is where its slots are freed, so the producer
    /// fills one half while the channel drains the other.
    static constexpr uint32_t tx_block_most = tx_size / 2u;
    /// The engine's owner id (DmaTxEngine::claim()): this SERCOM, plus one.
    static constexpr uint8_t engine_owner = static_cast<uint8_t>(n + 1u);

public:
    /// Instances are empty tags for concept-based call sites (print(serial, ...)).
    constexpr Uart() = default;

    /// The resource underneath, for the register-level verbs a console
    /// occasionally wants (the status flags, DBGCTRL, the teardown).
    using Resource = S;

    /// Whether the transmitter was given the DMA engine. Everything below
    /// branches on this with `if constexpr`, so a false one costs nothing
    /// at all - not a test, not a byte of state.
    static constexpr bool has_tx_engine = TxEngine::present;

    /// The GCLK generator this task takes its core clock from.
    /// Generator 0 is CLK_MAIN undivided in this stratum (samc21/clock.hpp
    /// states it so, and Clock::hz is that rate), which is what lets
    /// init() derive the baud divisor from the app's Clock tag alone. A
    /// SERCOM on any OTHER generator would need its own reference rate
    /// and is not built.
    static constexpr uint8_t generator = 0;

    // ---- lifecycle --------------------------------------------------------

    /// Bring the instance up: clocks, configuration, pads, pins,
    /// the receive interrupt and its NVIC line.
    ///
    /// Call AFTER the main clock is set up and before interrupts are
    /// enabled globally; `clock` is the app's brio::Clock tag
    /// (samc21/clock.hpp), so the baud divisor comes from Clock::hz and
    /// never from a second statement of the rate. The divisor is computed
    /// here rather than folded - see sercom_scale_65536() for what that
    /// costs and why it is still the cheap option.
    ///
    /// False when the rate cannot be produced at this clock, or when one
    /// of the peripheral's synchronizations did not complete - the caller
    /// then knows the transport is NOT up, rather than printing into a
    /// ring nothing will drain.
    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, const UartFormat& format = {}) {
        static_assert(clock_follows<Clock, Uart>(),
                      "this Uart is initialized with a DynamicClock that does not "
                      "list it among its Users: it would keep the old baud after "
                      "a clock change");

        const std::optional<uint16_t> reg = sercom_baud_reg(clock_hz(clock), baud);
        if (!reg) {
            return false;
        }

        // init() STARTS the transport: nothing a previous life left in
        // the rings or the counters is this one's traffic. The handler
        // is the rings' other party, so its line goes down first -
        // Ring::clear() is the one verb that is not concurrent.
        Nvic::disable(S::irq());
        m_rx.clear();
        m_tx.clear();
        clear_errors();
        m_baud = baud;
        m_tx_used = false;

        S::bus_clock(true);
        if (!S::core_clock(generator)) {
            return false;
        }
        if (!S::reset()) {
            return false;
        }

        const SercomUartConfig cfg{.pads = pads, .format = format, .baud = *reg};
        if (!S::configure(cfg)) {
            return false;
        }
        if (!S::enable(true)) {
            return false;
        }

        // The pads go to the SERCOM only now, with the transmitter
        // already enabled and its TxD idling high: handing PORT the pin
        // first would park a not-yet-driven pad on the line, and a
        // low-going edge there is a start bit to whatever listens.
        // A SERCOM input pin can be given a pull-DOWN and nothing else
        // (31.5.1), so RxD is left bare - the idle level is the sender's
        // to hold.
        TxPin::function(pads.tx_pin.function);
        RxPin::function(pads.rx_pin.function, {.input_enable = true});

        S::flush_rx();

        S::enable_rxc_interrupt(true);
        // WITH THE ENGINE THE DRE INTERRUPT IS NEVER ARMED: the DMA trigger
        // and the interrupt are the SAME condition, so arming both would
        // have the channel and the handler both serve one byte. The
        // engine's claim is the one-user rule (samc21/dmac.hpp): a second
        // owner's arm() while this one holds it is a panic. Without the
        // engine, DRE is armed on demand by write_byte().
        if constexpr (has_tx_engine) {
            if (!TxEngine::arm(engine_owner, S::data_address(), S::dma_tx_trigger())) {
                return false;
            }
        }

        Nvic::enable(S::irq());
        return true;
    }

    /// The core clock changed (DynamicClock fan-out): keep the
    /// same bit rate at the new rate.
    ///
    /// Called BEFORE the clock actually changes, so the drain below runs
    /// at the rate the queued bytes were meant for. TXC answers "the last
    /// byte has left the shifter" exactly - it is cleared by every write
    /// to DATA and set only when the shifter empties with nothing
    /// queued - so no timing guess is needed here, as it would be on a
    /// transmit-complete flag that cannot tell "done" from "idle since a
    /// while". A byte being RECEIVED during the switch may still be
    /// garbled: the caller picks a quiet moment.
    ///
    /// BAUD is enable-protected, so the instance is stopped around the
    /// write. Main context only.
    static void rebase(uint32_t hz) {
        // Bounded, like every wait in this stratum: a full 256-byte ring
        // at 9600 baud is a quarter of a second, and the budget is
        // generous at any rate this transport reaches. tx_idle() is the
        // wire's answer - the ring empty and the last stop bit gone.
        uint32_t spins = tx_drain_spins;
        while (!tx_idle() && spins-- != 0u) {
        }
        const std::optional<uint16_t> reg = sercom_baud_reg(hz, m_baud);
        if (!reg) {
            return;   // the new rate cannot carry this baud: nothing better to do
        }
        (void)S::enable(false);
        S::baud_reg(*reg);
        (void)S::enable(true);
    }

    /**
     * Change the rate under the running port: BAUD rewritten for `baud`
     * at the reference clock `hz`, once the TX side has gone idle the
     * way rebase() waits for it (the same bounded wait). False, and
     * nothing written, when the generator cannot express the rate. Main
     * context only; the caller owns the agreement with the other end.
     */
    static bool set_baud(uint32_t hz, uint32_t baud) {
        const std::optional<uint16_t> reg = sercom_baud_reg(hz, baud);
        if (!reg) {
            return false;
        }
        uint32_t spins = tx_drain_spins;
        while (!tx_idle() && spins-- != 0u) {
        }
        (void)S::enable(false);
        S::baud_reg(*reg);
        (void)S::enable(true);
        m_baud = baud;
        return true;
    }

    /// The slowest core clock that can still produce `baud` (the mode's
    /// own f_baud <= f_ref/16). A rebase() below this leaves the USART
    /// unable to hit the rate - check before.
    static constexpr uint32_t min_hz_for(uint32_t baud) { return sercom_min_ref_hz(baud); }

    /// True if the generator can produce `baud` at `hz`.
    static constexpr bool can_baud(uint32_t hz, uint32_t baud) {
        return sercom_baud_reg(hz, baud).has_value();
    }

    /// What the generator really produces at `hz` (the divisor read back
    /// from BAUD), not what was asked for.
    static uint32_t actual_baud(uint32_t hz) {
        return sercom_actual_baud(hz, S::baud_reg());
    }

    // ---- the ISR body -----------------------------------------------------

    /// The instance's ONE interrupt body - call from SERCOMn_Handler().
    ///
    /// Every source of this peripheral shares the vector, so the body
    /// starts by asking which of them is both raised AND enabled, then
    /// serves each. Only the two a byte transport needs are ever armed:
    /// RXC and, on demand, DRE. The ERROR interrupt deliberately is not -
    /// erratum 1.17.15 says it does not wake the device and directs the
    /// error check to the RXC path, which is exactly where STATUS is read
    /// anyway (it has to be read before DATA).
    ///
    /// Returns true when the RX ring transitioned empty -> non-empty: the
    /// edge signal for kernel glue ("post RxActivity to the serial AO on
    /// true"). Every empty->non-empty transition reports true and the
    /// consumer only empties the ring by draining it, so no wakeup is
    /// ever lost. Plain (non-kernel) apps may ignore the return value.
    ///
    /// THE RECEIVER IS A TWO-LEVEL BUFFER (31.6.2.6), and an entry takes
    /// every level it finds: RXC is asked again after each character and
    /// the loop ends when it reads clear. One entry per character on a
    /// line slower than the handler, one per two where characters arrive
    /// faster than an entry and its exit - the shape that decides whether
    /// the receiver keeps up at the generator's top rate (sercom.md).
    /// A CLEAN CHARACTER'S PATH IS CALL-FREE: the body, the ring's verbs
    /// and the register accesses are all inline, so a handler placed in
    /// SRAM (.ram_text, docs/samc21/platform.md) runs it from SRAM
    /// entirely; a receive error and a full ring are calls out of line
    /// (receive_hit(), receive_full()).
    // always_inline: a single call site (the vector binding in the app),
    // so inlining costs no flash and lets the compiler save only the
    // registers it actually uses - see ticker.hpp tick().
    [[gnu::always_inline]] static bool isr() {
        const uint8_t active = S::pending();
        bool edge = false;
        if ((active & SercomFlag::rxc) != 0u) {
            edge = receive();
        }
        if ((active & SercomFlag::dre) != 0u) {
            feed();
        }
        return edge;
    }

    // ---- the DMA half ------------------------------------------------------

    /// The DMAC's interrupt body, for the transport that holds the engine -
    /// call from DMAC_Handler():
    ///
    ///     extern "C" void DMAC_Handler() { Serial::dma_isr(); }
    ///
    /// A completion means the block handed over has gone into DATA, so
    /// exactly that many bytes are released from the ring and the next
    /// contiguous run is started. A transfer error (a bus error: the
    /// silicon disabled the channel, 25.6.2.8) is a block lost - abandoned,
    /// counted in dma_faults() - and the next run started. Nothing, and
    /// free, without the engine.
    [[gnu::always_inline]] static void dma_isr() {
        if constexpr (has_tx_engine) {
            while (const uint8_t flags = TxEngine::take_interrupt()) {
                if ((flags & TxEngine::flag_error) != 0u) {
                    if (TxEngine::abandon()) {
                        m_dma_faults = m_dma_faults + 1;
                    }
                } else if ((flags & TxEngine::flag_complete) != 0u) {
                    m_tx.consume(static_cast<typename decltype(m_tx)::index_t>(
                        TxEngine::complete()));
                }
                pump_tx();
            }
        }
    }

    // ---- byte transport (satisfies ByteSink / ByteSource) -----------------

    /// Try to queue one byte for transmission; false when the TX ring is
    /// full. Without an engine, arming DRE is a plain store to INTENSET,
    /// so it cannot race the handler's INTENCLR; with one, the DRE
    /// interrupt stays off for good and the engine is nudged instead.
    /// The public behaviour is identical either way - print() still
    /// blocks on a full ring and returns when the bytes are queued.
    static bool write_byte(uint8_t b) {
        if (!m_tx.push(b)) {
            // A REFUSED BYTE STILL NUDGES. print() answers a false here
            // by trying again for ever, so if this path can leave the
            // transport unpoked the program stops - and it could: the
            // ring is full precisely when nothing is draining it. See
            // nudge_blocked_tx().
            nudge_blocked_tx();
            return false;
        }
        m_tx_used = true;
        if constexpr (has_tx_engine) {
            pump_tx();
        } else {
            S::enable_dre_interrupt(true);
        }
        return true;
    }

    /// Fetch one received byte; false when nothing is pending.
    static bool read_byte(uint8_t& b) {
        const auto v = m_rx.pop();
        if (!v) {
            return false;
        }
        b = *v;
        return true;
    }

    /**
     * Queue a run of bytes in BULK: copy straight into the ring's own
     * free run and nudge the transport once a run - twice at most, the
     * first byte's own below - instead of once per byte. Returns how
     * many were queued (short of `src.size()` when the ring filled).
     *
     * WHY THIS EXISTS, measured rather than assumed. write_byte() ends
     * in a nudge - arming DRE on the plain transport, pump_tx() on the
     * engined one - and paying that per byte is what a link faster than
     * about 1 Mbaud runs out of CPU for. The probe app `serial_speed`
     * measured a hard plateau at 98 kB/s through write_byte() at EVERY
     * rate from 1 Mbaud up, while the same wire polled bare
     * reached 299 kB/s at 3 Mbaud: the ceiling was this API, not the
     * silicon and not the cable. Worse, the DMA engine came out SLOWER
     * than the interrupt (64 kB/s), because a pump_tx() per byte starts
     * a fresh block for one byte and the engine never gets a run to move.
     *
     * The ring already had the two halves this needs (write_span() +
     * publish(), util/ring.hpp): a producer may fill the contiguous run
     * it already owns and then publish it, which is exactly one index
     * store. Nothing about the concurrency model changes - this side
     * still writes only its own index - so the engine or the handler
     * draining the other end needs no cooperation.
     *
     * THE FIRST BYTE GOES BEFORE THE COPY, on the plain transport: it is
     * pushed and DRE armed exactly as write_byte() does, so an idle
     * transmitter starts on it while the rest is copied, where a nudge
     * after the copy started the first frame only once the whole run was
     * in the ring. feed() disarms DRE when it pops the ring's last byte,
     * and that first byte WAS the ring, so the rest, once published, is
     * armed again - one more INTENSET store, idempotent: two nudges a run
     * at most. A run that finds the ring full is write_byte()'s refusal,
     * nudge included. WITH AN ENGINE the run is copied whole and the
     * engine pumped once: an engine starts on a block, and a first byte
     * of its own would be a block and a completion interrupt more every
     * run.
     *
     * THE COPY is copy_run(): the runtime's memcpy for a run of at least
     * uart_copy_crossover bytes whose ends share their word alignment, the
     * inline byte loop otherwise (the measurement is there).
     */
    static uint32_t write_bulk(std::span<const uint8_t> src) {
        uint32_t done = 0;
        if constexpr (!has_tx_engine) {
            // write_byte()'s two steps, spelled here so the run's path
            // holds no call. A full ring is its refusal, nudge included -
            // and so is an empty run, the legal nudge sercom.md documents.
            if (src.empty() || !m_tx.push(src[0])) {
                nudge_blocked_tx();
                return 0;
            }
            S::enable_dre_interrupt(true);
            done = 1;
        }
        while (done < src.size()) {
            const auto room = m_tx.write_span();
            if (room.empty()) {
                break;
            }
            const uint32_t want = static_cast<uint32_t>(src.size()) - done;
            const uint32_t take =
                want < room.size() ? want : static_cast<uint32_t>(room.size());
            copy_run(room.data(), src.data() + done, take);
            m_tx.publish(static_cast<typename decltype(m_tx)::index_t>(take));
            done += take;
        }
        if (done != 0u) {
            m_tx_used = true;
        }
        if constexpr (has_tx_engine) {
            if (done != 0u) {
                pump_tx();   // ONE nudge for the whole run - the entire point
            } else {
                nudge_blocked_tx();   // nothing fitted: see write_byte()
            }
        } else if (done > 1u) {
            S::enable_dre_interrupt(true);   // the rest, behind a first byte feed() may have disarmed on
        }
        return done;
    }

    /**
     * Take a run of received bytes in BULK, the mirror of write_bulk():
     * copy out of the ring's own ready run and consume it with one index
     * store, instead of a pop() per byte. Returns how many were taken.
     */
    static uint32_t read_bulk(std::span<uint8_t> dst) {
        uint32_t done = 0;
        while (done < dst.size()) {
            const auto run = m_rx.read_span();
            if (run.empty()) {
                break;
            }
            const uint32_t want = static_cast<uint32_t>(dst.size()) - done;
            const uint32_t take =
                want < run.size() ? want : static_cast<uint32_t>(run.size());
            copy_run(dst.data() + done, run.data(), take);
            m_rx.consume(static_cast<typename decltype(m_rx)::index_t>(take));
            done += take;
        }
        return done;
    }

    /// The received bytes IN PLACE: the contiguous run ready to be read,
    /// never wrapping - the receive ring's consumer half under the ring's
    /// own names. With consume() it is util/stream.hpp's SpanSource,
    /// which SerialPort drains a run at a time with no copy at all.
    static std::span<const uint8_t> read_span() { return m_rx.read_span(); }

    /// Release the first `count` bytes of read_span(), oldest first, clamped
    /// to what is queued.
    static void consume(uint32_t count) {
        constexpr uint32_t most = decltype(m_rx)::capacity();
        m_rx.consume(static_cast<typename decltype(m_rx)::index_t>(count < most ? count : most));
    }

    // ---- introspection -----------------------------------------------------

    static auto rx_pending() { return m_rx.count(); }

    /// THE WIRE IS IDLE: nothing queued, no block in flight, and the last
    /// stop bit off the pad. TXC is that last clause exactly - set when
    /// the shifter has emptied with nothing new in DATA, cleared by any
    /// write to DATA (31.6.2.5, 31.8.8), the transmit channel's beats
    /// included, so it never answers for a frame before the current one.
    /// The ring's emptiness covers the other two: the engine releases a
    /// block's bytes only at its completion, when its last beat has
    /// written DATA. A port that has sent nothing since init() has TXC
    /// clear and is idle (m_tx_used).
    static bool tx_idle() { return m_tx.empty() && (!m_tx_used || S::txc_flag()); }

    static uint8_t rx_overruns() { return m_rx_overruns; }

    /**
     * THE SKIP EPOCH (util/stream.hpp's SkippingSource, docs/design/
     * serial.md): a count, modulo 2^32 and NEVER cleared - neither by
     * clear_errors() nor by init() -, of the characters the line carried
     * that the receive ring does not deliver clean. A reader that
     * compares it at every run knows a run is not contiguous with the one
     * before.
     *
     * It is the SkipRing's skips(): a character dropped on a full ring or
     * for a frame or parity error, and an overflow (BUFOVF: one at least
     * lost in the hardware before the character in hand), each reported to
     * the ring, a skip made at the consumer's next read_span() - which
     * discards what the ring held, so the count moves between the run
     * before the loss and the run after it. Rare paths only: nothing on a
     * clean character's path.
     */
    static uint32_t rx_skips() { return m_rx.skips(); }

    static uint8_t frame_errors() { return m_frame_errors; }
    static uint8_t parity_errors() { return m_parity_errors; }
    static uint8_t hw_overruns() { return m_hw_overruns; }

    /// Transmit blocks this transport had to throw away - a block the
    /// silicon stopped running (nudge_blocked_tx()) or a bus error
    /// (dma_isr()). With one channel in the image neither has a known
    /// cause; every suite judges this zero. Always 0, and free, without
    /// the engine.
    static uint8_t dma_faults() {
        if constexpr (has_tx_engine) {
            return m_dma_faults;
        } else {
            return 0;
        }
    }

    static void clear_errors() {
        m_rx_overruns = 0;
        m_frame_errors = 0;
        m_parity_errors = 0;
        m_hw_overruns = 0;
        if constexpr (has_tx_engine) {
            m_dma_faults = 0;
        }
    }

    /// Stop the transport and hand everything back: the engine and its
    /// claim first (a channel still writing DATA of a SERCOM being torn
    /// down is the one teardown order that matters; the claim given back
    /// is what lets another transport take the engine), then the NVIC
    /// line, the peripheral, both clocks and the two pins.
    static void release() {
        if constexpr (has_tx_engine) {
            TxEngine::release(engine_owner);
        }
        Nvic::disable(S::irq());
        S::release();
        TxPin::release();
        RxPin::release();
    }

private:
    /**
     * The producer has been refused: the transmit ring is full, which is
     * exactly the state in which nothing is draining it. Repair whatever
     * is repairable, then nudge.
     *
     * THE DEAD-BLOCK PREDICATE: a channel waiting for a trigger that
     * stands. The peripheral says its transmit buffer is EMPTY (DRE) and
     * its shifter has finished (TXC) - DRE is this channel's trigger, high
     * - and the channel says it is ENABLED with no trigger pending and no
     * beat moving (DmaTxEngine::waiting(): CHCTRLA.ENABLE, CHSTATUS.PEND
     * and BUSY). A channel with beats left latches DRE's rise and fills
     * DATA within one beat, so it cannot read that way; this one has lost
     * its trigger or is running a descriptor that is not the one
     * programmed. So the block is not slow, it is dead, and the only thing
     * to do is throw it away and start the next one. No timer, no rate,
     * no guess: the contradiction IS the evidence.
     *
     * THE ENGINE'S OWN "IN FLIGHT" IS NOT THE CHANNEL'S, and the
     * predicate asks the channel because of it. busy() is software: it
     * stays true from a block's last beat until dma_isr() hears the
     * completion, and it is true again for a block the handler has just
     * started whose first beat has not landed. Main context preempted
     * across that handler for a character time sees DRE and TXC both up
     * in either window - measured at 2 Mbaud with ONE channel and no
     * erratum in reach: blocks abandoned in nine echoes of twenty, up to
     * four in one, when the test was busy() and the flags alone. A
     * finished single block is DISABLED (25.6.2.6) and a just-started one
     * shows PEND - measured on 3376 starts onto a standing DRE, never
     * "nothing pending" (docs/samc21/sercom.md) - so neither reads as
     * waiting. And the three readings are taken under the mask, so the
     * handler cannot complete one block and start the next between them;
     * the abandon itself runs unmasked, a dead channel raising no
     * interrupt that could change its state.
     *
     * What killed a block that way, measured, was erratum 1.10.4 - a
     * concurrently triggered second channel corrupting this one's
     * write-back, which 25.6.2.6 makes the controller's LIVE descriptor -
     * and an image now holds one channel (samc21/dmac.hpp), so the repair
     * has no known cause left; it stays because without it a dead block
     * would leave the transport simply stopped: DmaTxEngine::busy() true
     * for ever,
     * pump_tx() returns at its first line every time, the ring fills,
     * and print() spins in Ring::push with the board silent. A
     * corruption that leaves DATA full or the transmitter off (DRE
     * clear) is beyond it: the predicate sees only the dead block that
     * left the transmitter idle and asking.
     *
     * WITHOUT an engine there is nothing to repair and the nudge is the
     * ordinary one - arming DRE, which write_byte() would have done
     * anyway had the byte fitted.
     */
    static void nudge_blocked_tx() {
        if constexpr (has_tx_engine) {
            // The cheap half first, unmasked: this path runs on every
            // refused byte of a spinning print(), and almost always one of
            // the two is false.
            if (TxEngine::busy() && transmitter_idle() && tx_block_dead()) {
                if (TxEngine::abandon()) {
                    m_dma_faults = m_dma_faults + 1;
                }
            }
            pump_tx();
        } else {
            S::enable_dre_interrupt(true);
        }
    }

    /// DRE and TXC both up: nothing in DATA, nothing in the shifter.
    static bool transmitter_idle() {
        constexpr uint8_t idle = SercomFlag::dre | SercomFlag::txc;
        return (S::flags() & idle) == idle;
    }

    /// The predicate's DECISION, under the mask (see nudge_blocked_tx()):
    /// the engine claims a block, its channel waits, the trigger stands.
    static bool tx_block_dead() {
        typename SamPlatform::CriticalSection cs;
        return TxEngine::busy() && TxEngine::waiting() && transmitter_idle();
    }

    /**
     * Hand the transmit engine the next contiguous run of the TX ring,
     * if it is free to take one.
     *
     * TWO CONTEXTS, ONE CONSUMER. This runs both from write_byte() in
     * main context and from dma_isr() in the handler, and it is the
     * ring's CONSUMER side in both - which the SPSC contract allows only
     * one of. The engine's reservation is what makes the two into one
     * logical consumer: it is taken under the critical section, and the
     * context that holds it is the only one that reads the run and starts
     * the block - the other finds the engine reserved and returns. The
     * consumer index moves only in dma_isr(), on a completion, which no
     * reserved engine with no block in flight can raise.
     *
     * The run is contiguous by construction: read_span() stops at the
     * end of the buffer, so a wrapped ring goes out in two blocks and
     * the second is started by the first one's completion.
     */
    static void pump_tx() {
        if constexpr (has_tx_engine) {
            {
                // THE MASK COVERS THE CLAIM AND NOTHING ELSE: a
                // test-and-set of the engine's busy flag. Once it is held,
                // no completion of this engine can land - no block of it
                // is in flight - so the ring's run and the channel's
                // programming need no mask at all.
                typename SamPlatform::CriticalSection cs;
                if (!TxEngine::reserve()) {
                    return;
                }
            }
            auto run = m_tx.read_span();
            if (run.empty()) {
                TxEngine::cancel();
                return;
            }
            // HALF THE RING AT MOST: the block's completion is what frees
            // its slots, so a block of the whole ring leaves a producer
            // longer than the ring nothing to refill until the wire has
            // gone idle - measured, a 4096-byte print through a 2048-byte
            // ring waited a whole copy at 3 Mbaud (sercom.md).
            if (run.size() > tx_block_most) {
                run = run.first(tx_block_most);
            }
            // NO KICK. DRE is a level the DMAC latches on its rise, and a
            // rise while the channel is disabled with its trigger selected
            // is served on the next enable, as is the claim's selection of
            // the trigger onto a DRE already standing - so the enable
            // fires the first beat by itself. Measured on 3376 block
            // starts, after a DRE standing up to 1.8 ms among them: the
            // first beat had always landed by the next register read, and
            // a kick that came after it had started was a second beat
            // into a full DATA (one byte of a block lost, sercom.md).
            (void)TxEngine::launch(run);
        }
    }

    /// The copy into the transmit ring's free run and out of the receive
    /// ring's ready one: two pointers and the test at the bottom - a load,
    /// a store, two steps and one branch a byte - unless the run is at
    /// least uart_copy_crossover long AND its two ends share their
    /// alignment in the word, where the runtime's memcpy moves it on its
    /// word path; across a misalignment memcpy's byte path is this loop
    /// out of line, a call dearer (rt/rt.cpp). `len` is at least one.
    [[gnu::always_inline]] static void copy_run(uint8_t* to, const uint8_t* from, uint32_t len) {
        constexpr uintptr_t word_mask = sizeof(uint32_t) - 1u;
        if (len >= uart_copy_crossover &&
            ((reinterpret_cast<uintptr_t>(to) ^ reinterpret_cast<uintptr_t>(from)) & word_mask) ==
                0u) {
            std::memcpy(to, from, len);
            return;
        }
        uint8_t* const end = to + len;
        do {
            *to++ = *from++;
        } while (to != end);
    }

    /// Every received character the two-level buffer holds: per character
    /// STATUS first (it belongs to the character about to be read), then
    /// DATA (which is what advances the FIFO and clears RXC), then RXC
    /// asked again. A buffer overflow means the hardware ALREADY lost
    /// bytes, but the one in hand is good; a frame or parity error means
    /// this one is not, and it is dropped. The edge is the ring's empty ->
    /// non-empty across the whole entry.
    [[gnu::always_inline]] static bool receive() {
        const bool was_empty = m_rx.empty();
        bool pushed = false;
        do {
            const uint16_t st = S::status();
            const uint8_t byte = static_cast<uint8_t>(S::data());
            const uint16_t errors = static_cast<uint16_t>(st & SercomStatus::receive_errors);
            // THE CLEAN CHARACTER'S PATH STAYS IN THE BODY; the errors,
            // the full ring and the losses they report are calls out of
            // line (receive_hit(), receive_full()).
            if (errors == 0u) [[likely]] {
                if (m_rx.push(byte)) [[likely]] {
                    pushed = true;
                } else {
                    receive_full();
                }
            } else {
                pushed = receive_hit(errors, byte) || pushed;
            }
        } while (S::rxc_flag());
        return was_empty && pushed;
    }

    /// THE RARE PATHS, OUT OF LINE: a character that found the ring full,
    /// and one that carries a receive error - counted, cleared, and its
    /// loss reported. Kept out of the handler's body so the clean
    /// character's path keeps its registers: inlined, a rare path's
    /// counters spill the character to the stack (counted in the release
    /// listing); out of line the clean path costs 218 cycles a character
    /// from the flash (letter u of bench_samc; 140 from SRAM,
    /// bench_samc_ram).
    [[gnu::noinline, gnu::cold]] static void receive_full() {
        m_rx_overruns = m_rx_overruns + 1;
        rx_lost();
    }

    /// True when the character was kept and pushed. BUFOVF reports the
    /// characters lost before it, which is kept; FERR or PERR drop it.
    [[gnu::noinline, gnu::cold]] static bool receive_hit(uint16_t errors, uint8_t byte) {
        S::clear_status(errors);
        S::clear_flags(SercomFlag::error);   // the combined flag travels with them
        if ((errors & SercomStatus::overflow) != 0u) {
            m_hw_overruns = m_hw_overruns + 1;
            rx_lost();   // the characters lost BEFORE this one
        }
        bool keep = true;
        if ((errors & SercomStatus::frame_error) != 0u) {
            m_frame_errors = m_frame_errors + 1;
            keep = false;
        }
        if ((errors & SercomStatus::parity_error) != 0u) {
            m_parity_errors = m_parity_errors + 1;
            keep = false;
        }
        if (!keep) {
            rx_lost();
            return false;
        }
        if (m_rx.push(byte)) {
            return true;
        }
        receive_full();
        return false;
    }

    /// A character the interrupt receiver lost, reported to the SkipRing.
    [[gnu::always_inline]] static void rx_lost() { m_rx.lost(); }

    /// Feed the next byte and disarm when the ring drains (write_byte()
    /// re-arms). No race with write_byte(): a handler on this core runs
    /// to completion against the main context, so the ring test and the
    /// disarm are one indivisible step from main's point of view.
    [[gnu::always_inline]] static void feed() {
        if (const auto c = m_tx.pop()) {
            S::data(*c);
            if (m_tx.empty()) {
                S::enable_dre_interrupt(false);
            }
        } else {
            S::enable_dre_interrupt(false);
        }
    }
};

namespace detail {
/// A pad pair every device of the family bonds, used only to check the
/// concepts below at namespace scope.
inline constexpr UartPads sercom_probe_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};
} // namespace detail

static_assert(ByteTransport<Uart<0, detail::sercom_probe_pads>> &&
                  BulkSink<Uart<0, detail::sercom_probe_pads>> &&
                  SkippingSource<Uart<0, detail::sercom_probe_pads>>,
              "Uart must satisfy the transport concepts, the skip epoch included");
static_assert(ClockUser<Uart<0, detail::sercom_probe_pads>>,
              "Uart must be listable among a dynamic clock's users");

} // namespace brio
