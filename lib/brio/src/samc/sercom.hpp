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
 * SERCOM's other three personalities (SPI host/client, I2C host/client)
 * are whole chapters of their own and get their own headers; inside
 * USART mode the fractional and 3x baud regimes, the synchronous role
 * with XCK, hardware handshaking (RTS/CTS), RS485/TE, LIN, IrDA,
 * collision detection, auto-baud, start-of-frame detection and the DMA
 * triggers are declared here as NOT BUILT rather than half-built. The
 * baud arithmetic below names its oversampling explicitly so a second
 * regime slots in without moving anything.
 *
 * ONE INTERRUPT VECTOR, NOT TWO. This is the one place where the AVR
 * shape must NOT be copied. An AVR USART raises RXC and DRE on two
 * separate vectors, so avrdx/usart.hpp offers two ISR bodies. A SERCOM
 * has exactly ONE line in the NVIC (SERCOM5_IRQn = 14 here) shared by
 * DRE, TXC, RXC, RXS, CTSIC, RXBRK and ERROR - so the task offers ONE
 * ISR body, Uart::isr(), which reads INTFLAG, masks it with INTENSET
 * (a raised flag nobody asked for must not be acted on - DRE in
 * particular reads 1 whenever the transmit buffer is empty, which is
 * most of the time) and serves whatever is genuinely pending. The app
 * binds exactly one vector:
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
 *    receiver disabled) belongs to the power pass: nothing here sleeps.
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
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

/// Is this pad/pin pair something the silicon and the package can do?
/// The pad side is exact; the pin side is checked only as far as this
/// header can know it (the group exists, the pin number is in range) -
/// that a given PIN really reaches that PAD is the open device-table
/// question of the file header.
constexpr bool uart_pads_valid(const UartPads& p) {
    return uart_tx_pad_exists(p.tx) && p.tx != p.rx && p.tx_pin.valid() &&
           p.rx_pin.valid();
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
/// and a bit rate, exactly as on the AVR side: the resource speaks the
/// register, the task speaks hertz.
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

    static sercom_usart_int_registers_t& regs() {
        if constexpr (n == 0) return SERCOM0_REGS->USART_INT;
#if defined(SERCOM1_REGS)
        else if constexpr (n == 1) return SERCOM1_REGS->USART_INT;
#endif
#if defined(SERCOM2_REGS)
        else if constexpr (n == 2) return SERCOM2_REGS->USART_INT;
#endif
#if defined(SERCOM3_REGS)
        else if constexpr (n == 3) return SERCOM3_REGS->USART_INT;
#endif
#if defined(SERCOM4_REGS)
        else if constexpr (n == 4) return SERCOM4_REGS->USART_INT;
#endif
#if defined(SERCOM5_REGS)
        else if constexpr (n == 5) return SERCOM5_REGS->USART_INT;
#endif
        else return SERCOM0_REGS->USART_INT;
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
    /// samc/clock.hpp because the clock tree needed it first; it waits
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
 * rings on both sides, ByteSink + ByteSource. It is the AVR Uart's twin
 * in everything but the ISR surface - one combined isr() instead of the
 * AVR's rxc()/dre() pair, because this core gives the peripheral one
 * vector (see the file header).
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
 * Ring sizes: unlike AVR, NOTHING here pushes them towards 256. This
 * core reads a word atomically (SamPlatform::atomic_width == 4), so
 * util/ring.hpp takes its lock-free path at every size and a larger
 * ring costs only RAM. 64/256 are kept as console-class defaults, not
 * as a ceiling.
 */
template <uint8_t n, UartPads pads, uint32_t rx_size = 64, uint32_t tx_size = 256>
class Uart {
    using S = Sercom<n>;

    static_assert(uart_pads_valid(pads),
                  "these SERCOM pads cannot carry an asynchronous link: TxD exists "
                  "only on PAD[0] and PAD[2] (CTRLA.TXPO), the two directions cannot "
                  "share a pad, and both pins must be real ones on this device");

    using TxPin = Pin<pads.tx_pin.port, pads.tx_pin.pin>;
    using RxPin = Pin<pads.rx_pin.port, pads.rx_pin.pin>;

    // One ring pair per instantiation (static inline -> .bss, no ctor).
    static inline Ring<uint8_t, rx_size, SamPlatform> m_rx{};
    static inline Ring<uint8_t, tx_size, SamPlatform> m_tx{};

    // Error counters, written in the handler, read from the main loop.
    // A byte moves in one access on this core; they wrap at 255. Written
    // as `x = x + 1` because compound ops on volatile are deprecated in
    // C++20.
    static inline volatile uint8_t m_rx_overruns = 0;   // RX ring full, byte lost
    static inline volatile uint8_t m_frame_errors = 0;  // FERR: byte dropped
    static inline volatile uint8_t m_parity_errors = 0; // PERR: byte dropped
    static inline volatile uint8_t m_hw_overruns = 0;   // BUFOVF: bytes lost in HW
    static inline uint32_t m_baud = 0;                  // for rebase()

public:
    /// Instances are empty tags for concept-based call sites (print(serial, ...)).
    constexpr Uart() = default;

    /// The resource underneath, for the register-level verbs a console
    /// occasionally wants (the status flags, DBGCTRL, the teardown).
    using Resource = S;

    /// The GCLK generator this task takes its core clock from.
    /// Generator 0 is CLK_MAIN undivided in this stratum (samc/clock.hpp
    /// states it so, and Clock::hz is that rate), which is what lets
    /// init() derive the baud divisor from the app's Clock tag alone. A
    /// SERCOM on any OTHER generator would need its own reference rate
    /// and is not built.
    static constexpr uint8_t generator = 0;

    // ---- lifecycle --------------------------------------------------------

    /**
     * @brief Bring the instance up: clocks, configuration, pads, pins,
     * the receive interrupt and its NVIC line.
     *
     * Call AFTER the main clock is set up and before interrupts are
     * enabled globally; `clock` is the app's brio::Clock tag
     * (samc/clock.hpp), so the baud divisor comes from Clock::hz and
     * never from a second statement of the rate. The divisor is computed
     * here rather than folded - see sercom_scale_65536() for what that
     * costs and why it is still the cheap option.
     *
     * False when the rate cannot be produced at this clock, or when one
     * of the peripheral's synchronizations did not complete - the caller
     * then knows the transport is NOT up, rather than printing into a
     * ring nothing will drain.
     */
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
        S::enable_rxc_interrupt(true);   // DRE is armed on demand by write_byte()
        Nvic::enable(S::irq());
        return true;
    }

    /**
     * @brief The core clock changed (DynamicClock fan-out): keep the
     * same bit rate at the new rate.
     *
     * Called BEFORE the clock actually changes, so the drain below runs
     * at the rate the queued bytes were meant for. TXC answers "the last
     * byte has left the shifter" exactly - it is cleared by every write
     * to DATA and set only when the shifter empties with nothing queued
     * - so no timing guess is needed here (the AVR twin has to wait out
     * two frame times because its TXCIF cannot tell "done" from "idle
     * since a while"). A byte being RECEIVED during the switch may still
     * be garbled: the caller picks a quiet moment.
     *
     * BAUD is enable-protected, so the instance is stopped around the
     * write. Main context only.
     */
    static void rebase(uint32_t hz) {
        // Bounded, like every wait in this stratum. A full 256-byte ring
        // at 9600 baud is a quarter of a second and one frame is about a
        // millisecond, so both budgets are generous at any rate this
        // transport reaches - and an instance that has never transmitted
        // has TXC clear and simply falls through the second one, which
        // is the right answer there (nothing is in flight).
        constexpr uint32_t ring_drain_spins = 8'000'000u;
        constexpr uint32_t frame_spins = 200'000u;
        uint32_t spins = ring_drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
        }
        spins = frame_spins;
        while (!S::txc_flag() && spins-- != 0u) {
        }
        const std::optional<uint16_t> reg = sercom_baud_reg(hz, m_baud);
        if (!reg) {
            return;   // the new rate cannot carry this baud: nothing better to do
        }
        (void)S::enable(false);
        S::baud_reg(*reg);
        (void)S::enable(true);
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

    /**
     * @brief The instance's ONE interrupt body - call from SERCOMn_Handler().
     *
     * Every source of this peripheral shares the vector, so the body
     * starts by asking which of them is both raised AND enabled, then
     * serves each. Only the two a byte transport needs are ever armed:
     * RXC and, on demand, DRE. The ERROR interrupt deliberately is not -
     * erratum 1.17.15 says it does not wake the device and directs the
     * error check to the RXC path, which is exactly where STATUS is read
     * anyway (it has to be read before DATA).
     *
     * @return true when the RX ring transitioned empty -> non-empty: the
     * edge signal for kernel glue ("post RxActivity to the serial AO on
     * true"). Every empty->non-empty transition reports true and the
     * consumer only empties the ring by draining it, so no wakeup is
     * ever lost. Plain (non-kernel) apps may ignore the return value.
     */
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

    // ---- byte transport (satisfies ByteSink / ByteSource) -----------------

    /// Try to queue one byte for transmission; false when the TX ring is
    /// full. Arming DRE is a plain store to INTENSET, so it cannot race
    /// the handler's INTENCLR.
    static bool write_byte(uint8_t b) {
        if (!m_tx.push(b)) {
            return false;
        }
        S::enable_dre_interrupt(true);
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

    /// Queue as much of the buffer as fits; returns the number queued.
    static uint8_t write(const uint8_t* buffer, uint8_t len) {
        uint8_t written = 0;
        while (written < len && write_byte(buffer[written])) {
            ++written;
        }
        return written;
    }

    // ---- introspection -----------------------------------------------------

    static auto rx_pending() { return m_rx.count(); }
    static bool tx_idle() { return m_tx.empty(); }

    static uint8_t rx_overruns() { return m_rx_overruns; }
    static uint8_t frame_errors() { return m_frame_errors; }
    static uint8_t parity_errors() { return m_parity_errors; }
    static uint8_t hw_overruns() { return m_hw_overruns; }

    static void clear_errors() {
        m_rx_overruns = 0;
        m_frame_errors = 0;
        m_parity_errors = 0;
        m_hw_overruns = 0;
    }

    /// Stop the transport and hand everything back: NVIC line,
    /// peripheral, both clocks and the two pins.
    static void release() {
        Nvic::disable(S::irq());
        S::release();
        TxPin::release();
        RxPin::release();
    }

private:
    /// One received character: STATUS first (it belongs to the character
    /// about to be read), then DATA (which is what advances the FIFO and
    /// clears RXC). A buffer overflow means the hardware ALREADY lost
    /// bytes, but the one in hand is good; a frame or parity error means
    /// this one is not.
    static bool receive() {
        const uint16_t st = S::status();
        const uint8_t byte = static_cast<uint8_t>(S::data());
        const uint16_t errors = static_cast<uint16_t>(st & SercomStatus::receive_errors);
        if (errors != 0u) {
            S::clear_status(errors);
            S::clear_flags(SercomFlag::error);   // the combined flag travels with them
            if ((errors & SercomStatus::overflow) != 0u) {
                m_hw_overruns = m_hw_overruns + 1;
            }
            if ((errors & SercomStatus::frame_error) != 0u) {
                m_frame_errors = m_frame_errors + 1;
            }
            if ((errors & SercomStatus::parity_error) != 0u) {
                m_parity_errors = m_parity_errors + 1;
            }
            if ((errors & (SercomStatus::frame_error | SercomStatus::parity_error)) != 0u) {
                return false;   // drop the corrupted byte
            }
        }
        const bool was_empty = m_rx.empty();
        if (!m_rx.push(byte)) {
            m_rx_overruns = m_rx_overruns + 1;
            return false;   // a full ring cannot be empty: no edge
        }
        return was_empty;
    }

    /// Feed the next byte and disarm when the ring drains (write_byte()
    /// re-arms). No race with write_byte(): a handler on this core runs
    /// to completion against the main context, so the ring test and the
    /// disarm are one indivisible step from main's point of view.
    static void feed() {
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

static_assert(ByteTransport<Uart<0, detail::sercom_probe_pads>>,
              "Uart must satisfy the transport concepts");
static_assert(ClockUser<Uart<0, detail::sercom_probe_pads>>,
              "Uart must be listable among a dynamic clock's users");

} // namespace brio
