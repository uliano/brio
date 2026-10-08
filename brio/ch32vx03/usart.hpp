/*
 * usart.hpp
 *
 * The serial ports of the CH32V203 and the CH32V303 (RM ch. 18) in the
 * two strata every brio serial driver has (docs/design/serial.md):
 *
 *  Usart<n>              the RESOURCE: which instance, where its
 *                        registers are, its bus gate and its reset, its
 *                        vector, its DMA slots, and the whole register
 *                        description of chapter 18 as verbs - the
 *                        frame, the divisor, mute mode with both wakes,
 *                        LIN's break, single-wire half duplex, IrDA,
 *                        the smartcard, the synchronous clock, the
 *                        flow-control pair, the DMA requests, every flag
 *                        and every interrupt enable, and on the CH32V303
 *                        the lot's MARK and SPACE parity and short words;
 *  Uart<n, P, ...>       the TASK: the interrupt-driven byte transport
 *                        with two rings and one ISR body - every
 *                        console's personality, a ByteTransport for
 *                        print() and SerialPort, the same surface the
 *                        other five targets expose.
 *
 * UP TO EIGHT INSTANCES, TWO BUSES. USART1 sits on PB2 and every other
 * port - USART2, USART3 and UART4..UART8 - on PB1, which on this family
 * does NOT run at the same rate above 72 MHz (clock.hpp caps PB1 there),
 * so the transport asks its own bus for the clock its divisor counts.
 * WHICH instances a part offers is the part's table (device::has_usart),
 * and the list is not always the first n of them: the smallest part
 * offers one usart and it is USART2, because its package bonds neither
 * of USART1's pin pairs; the CH32V203C8 and RB offer four; the 128 KB
 * CH32V303 three; the CH32V303RC and VC all eight. UART5..UART8 live at
 * addresses that are not in instance order (UART6..8 BELOW USART2 on
 * PB1), which is device.hpp's usart_base_for() and nobody else's.
 *
 * WHICH OF THEM IS A FULL USART is a second part fact. Chapter 18's
 * opening counts three USARTs and five UARTs for the whole family and
 * then names the exception: on the CH32V203C8 the fourth serial port is
 * a USART4 - the datasheet's pin table agrees, giving it a CK, a CTS and
 * an RTS pad - while on the CH32V203RB and every CH32V303 it is a UART4
 * with TX and RX alone, and UART5..8 are UARTs everywhere. So `is_full`
 * - the synchronous clock, the smartcard and the flow-control pair - is
 * a PART fact (device::usart_full) and not a number's parity, and the
 * verbs behind it are REFUSED AT COMPILE TIME on an instance that is not
 * one; the read-backs of those modes answer false there.
 *
 * THE PADS ARE THE REMAP TABLES' (afio.hpp, tables 10-23..10-31): the
 * `remap` template parameter names a COLUMN and the TX, RX, CK, CTS and
 * RTS pads follow from the same table that programs AFIO_PCFR1/PCFR2.
 * A code of 0 writes nothing at all - the reset column is already
 * there, and USART1 carries the probe's console on this board, which is
 * a pair of pads no init() may move behind the program's back. UART4 is
 * where the table matters most: the manual has two of them and the one
 * that starts at PC10/PC11 belongs to the CH32V20x_D8 and the CH32V303,
 * the CH32V203C8 being named in the other, whose default pads are PB0
 * and PB1. TWO COLUMNS LAND ON THE DEBUG PORT'S OWN PADS - USART3's
 * code 2 (PA13/PA14) and UART8's code 1 (PA14/PA15) - and a transport
 * naming one is refused by init() while the two-wire port is still the
 * probe's: afio.hpp's long-spelled verb is how a program gives it up.
 *
 * TWO RINGS AND TWO FLAGS. Bytes leave through a TX ring drained by the
 * TXE interrupt (write_byte() arms TXEIE after a byte, write_bulk() after
 * a run's first byte and again after the rest; the ISR disarms it when
 * the ring runs dry, because TXE stands for ever while the transmitter
 * is idle and would re-enter the handler without end), and arrive into
 * an RX ring filled by RXNE, taken a byte at a time by read_byte() or a
 * run at a time, in place, by read_span() and consume(). isr() returns
 * the "RX went non-empty" EDGE that util/serial_port.hpp posts on -
 * one event per idle-to-busy transition, not one per byte.
 *
 * THE BAUD DIVISOR IS THE WHOLE REGISTER. BRR counts the peripheral
 * clock's periods per bit in sixteenths (18.3: a 12-bit mantissa and a
 * 4-bit fraction), so the value to store is simply pclk / baud - no
 * field arithmetic - and actual_baud() inverts it, which is how a
 * program reports the rate it is really running at. Below 16 the
 * generator has nothing to divide by: refused.
 *
 * ERRORS ARE READ THEN CLEARED. Parity, framing, noise, overrun and the
 * idle line stand in STATR and go away when STATR is read and then
 * DATAR, in that order (18.10.1); RXNE, TC, LBD and CTS also clear by
 * writing a zero over them. The handler therefore reads the status
 * ONCE, decides from that copy, and lets the DATAR read do the
 * clearing. A byte that arrived with an error is DROPPED rather than
 * pushed - a corrupted byte in a line assembler is worse than a gap -
 * and the counter says it happened.
 *
 * THE MODES EXCLUDE EACH OTHER THE WAY 18.4 .. 18.7 SAY, and the
 * exclusions are NOT symmetrical: the synchronous clock wants SCEN,
 * HDSEL and IREN clear; half duplex wants SCEN, CLKEN and IREN clear;
 * the smartcard wants LINEN, HDSEL and IREN clear and KEEPS CLKEN,
 * which is where its card clock comes from - so smartcard() writes that
 * bit itself, there being no other verb the exclusions would let reach
 * it under SCEN; IrDA wants LINEN, STOP, CLKEN, SCEN and HDSEL clear.
 * Each verb refuses instead of storing a combination the chapter
 * declares undefined.
 *
 * TWO OPTIONAL DMA ENGINE SLOTS, the shape every stratum with a
 * controller uses: a transmit engine drains the TX ring by contiguous
 * runs (the ring's read_span, consumed by exactly what the block
 * carried), and a receive engine runs in its CIRCULAR shape over the
 * whole receive ring's storage (ch32vx03/dma.hpp's DmaRxEngine): the
 * channel writes the storage lap after lap and is never re-armed, the
 * ring's producer index is the channel's own count, and the receive ring
 * is the consumer half of that (util/ring.hpp's HardwareRing) - so a byte
 * that lands is readable at once, a burst wraps the storage's end with no
 * CPU, and nothing is lost between runs because there are none. What the
 * consumer did not keep up with - a lap written over its unread bytes -
 * the ring counts and skips, and rx_overruns() reports it. Without an
 * engine the slot is the NoDmaEngine tag, every engine branch is
 * compiled out and init() does not so much as touch CTLR3. An engine is refused
 * on any SLOT but the instance's own - the controller and the channel -
 * which the request table of ch32vx03/dma_engine.hpp answers: USART1
 * transmits on DMA1's channel 4 and receives on 5, USART2 on 7 and 6,
 * USART3 on 2 and 3, UART4 on DMA1's 1 and 8 on the CH32V203 and on
 * DMA2's 5 and 3 on the CH32V303, whose UART5..8 are DMA2's too (4 and 2,
 * 6 and 7, 8 and 9, 10 and 11). A program with an engine running does not
 * sleep on this family, because in Sleep the bus matrix serves the core
 * alone (docs/ch32vx03/dma.md) - and a receive ring runs from init() to
 * release().
 *
 * UNDER THE RECEIVE ENGINE THE CPU NEVER READS DATAR, and what a clear
 * costs is this silicon's own answer - measured on the CH32V203C8T6, and
 * NOT the STM32F4's although the two chapters word it alike (18.10.1:
 * "reading the status register and then reading the data register").
 * Here a read of STATR ARMS the clear and the next read of DATAR - the
 * channel's - clears EVERY error flag and IDLE standing at that read,
 * including one its own frame just raised: a break whose FE rose after a
 * status read is gone by the time anybody looks, while one with no status
 * read since the last DATAR read stands until somebody reads STATR and
 * the channel reads the next frame. And clearing IDLE that way also
 * forgets the idle the clearing frame armed, as on the F4: a burst of one
 * frame after an idle the vector saw raises no IDLE of its own.
 *
 * THE BURST EDGE COMES FROM A VECTOR. With the receive engine the USART's
 * vector runs two states over that clear. WAITING FOR THE END: IDLEIE,
 * PEIE and EIE armed; an idle line or an error enters, its status read
 * counts the errors and arms the clear, the edge is reported, and the
 * vector turns to WAITING FOR A FRAME - those three disarmed, RXNEIE
 * armed (the channel still takes the byte; the interrupt only says one
 * came, and the channel's read of it finished the clear). That entry
 * READS NO STATR - a status read there would arm the clear again, and the
 * next frame's own error would be cleared by its own read unseen - but
 * CNTR alone: a moved count turns the vector back, reporting the edge
 * again, the edge of a burst of one frame. The channel's half and full
 * marks report it too, so a stream with no silence in it is told twice a
 * lap. Two interrupts a burst, none a byte, the edge gated as the
 * interrupt receiver's: once per idle-to-busy transition of the consumer,
 * re-opened when its look finds the ring empty. A CHANNEL THAT NEVER TAKES
 * THE FRAME - stopped by a transfer error, or frozen by a request another
 * peripheral left held on its OR (docs/ch32vx03/dma.md) - leaves RXNE
 * standing under RXNEIE, and the wait would re-enter for ever: its entries
 * that find the count unmoved are counted, and the 255th gives the channel
 * up - a DMA fault counted, RXNEIE down, the edge reported, nothing else
 * touched in the vector - and the consumer's next look restarts the ring
 * from its first element and arms the wait for the end, as after a
 * transfer error.
 *
 * WHAT THE CHANNEL'S READ BOUNDS - the counts, never the bytes. The frame
 * after one whose error the vector counted loses its own (the count's
 * status read armed the clear that frame's read performs): a run of
 * errored frames counts every other one, errors separated by a clean
 * frame count each. The first frame of a burst after an idle line the
 * vector saw loses its error the same way. And a status read ANYWHERE
 * ELSE - tx_idle(), the interrupt transmitter's entry, a thread polling a
 * flag - arms the clear for the next frame: an error there is cleared
 * unseen.
 *
 * THE LOT'S REGISTERS. Four things the chapter gives the CH32V30x_D8 -
 * and there only "with the penultimate sixth digit of the lot number not
 * being zero", which no register states and the part number does not
 * carry: CTLR4 (18.10.8: the MARK and SPACE parity, a parity bit held at
 * one or at zero, and MS_ERRIE, its error's interrupt), CTLR1's M_EXT
 * (18.10.4: seven, six and five-bit words), and STATR's MS_ERR and
 * RX_BUSY (18.10.1). None of them is the CH32V203's: every verb here that
 * names one is REFUSED AT COMPILE TIME on that series' two classes. On a
 * CH32V303 they are VERBS THAT ASK THE DIE, this stratum's rule for a
 * lot's register (docs/ch32vx03/README.md): M_EXT is a field of a
 * register that exists, so it is written and read back; CTLR4 is a
 * register of its own at 0x1C, and an absent register can answer as a
 * MIRROR of another, so `ctlr4_present()` reads the word first - the
 * real one holds nothing outside its three bits - and then writes it
 * only on a disabled port, every other register of the block compared
 * before and after and the block put back through its reset line and
 * the program's own words when anything else moved. The answer is kept
 * per instance, and a CTLR4 verb asks it before it writes a bit. RX_BUSY
 * and MS_ERR are status bits of a register that exists and read zero on
 * a die without them. The CH32V303VCT6 the reference suite ran on is
 * such a die: CTLR4's address read zero on USART2 and UART4 and moved
 * nothing when written, M_EXT did not keep a code, RX_BUSY never rose
 * under a frame - and the WIRE agrees, the parity bit staying computed
 * and the word eight bits long with both stored raw, so the four are
 * absent there and not write-only (docs/ch32vx03/usart.md).
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <atomic>
#include <bit>
#include <span>

#include "ch32vx03/afio.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/dma_engine.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"

namespace brio {

// =============================================================================
// The chapter's vocabulary
// =============================================================================

/// DATA bits of a frame. The register speaks in WORD length (M: 8 or 9
/// bits including the parity bit when there is one), so seven data bits
/// exist only with a parity bit and nine only without.
enum class UartBits : uint8_t { seven = 7, eight = 8, nine = 9 };

enum class UartParity : uint8_t { none, even, odd };

/// CTLR2.STOP, all four codes; the half and one-and-a-half stops are the
/// chapter's own (18.10.5).
enum class UartStop : uint8_t { one = 0, half = 1, two = 2, one_and_half = 3 };

/// The frame the two ends agree on. Defaults to 8N1.
struct UartFormat {
    UartBits bits = UartBits::eight;
    UartParity parity = UartParity::none;
    UartStop stop = UartStop::one;
};

constexpr bool uart_format_valid(const UartFormat& f) {
    if (f.bits == UartBits::seven) {
        return f.parity != UartParity::none;
    }
    if (f.bits == UartBits::nine) {
        return f.parity == UartParity::none;
    }
    return true;
}

/// CTLR1's M, PCE and PS for a format (the word length is the data bits
/// plus the parity bit).
constexpr uint16_t usart_ctlr1_format(const UartFormat& f) {
    uint16_t v = 0;
    const bool parity = f.parity != UartParity::none;
    if (f.bits == UartBits::nine || (f.bits == UartBits::eight && parity)) {
        v |= usart_m;
    }
    if (parity) {
        v |= usart_pce;
    }
    if (f.parity == UartParity::odd) {
        v |= usart_ps;
    }
    return v;
}

constexpr uint16_t usart_ctlr2_stop(UartStop s) {
    return static_cast<uint16_t>(static_cast<uint16_t>(s) << usart_stop_shift);
}

/// The mask of the data bits a frame carries in DATAR (the parity bit,
/// when there is one, is the receiver's to check and not the byte's).
constexpr uint16_t uart_data_mask(const UartFormat& f) {
    return f.bits == UartBits::nine ? 0x1FFu : f.bits == UartBits::eight ? 0xFFu : 0x7Fu;
}

/// Mute mode (18.10.4's RWU and WAKE, 18.10.5's ADD): the receiver
/// asleep until the line goes idle, or until a frame whose MSB is set
/// carries this node's 4-bit address.
enum class MuteWake : uint8_t { idle_line, address_mark };

struct MuteConfig {
    MuteWake wake = MuteWake::idle_line;
    uint8_t address = 0;   ///< 0..15, for address_mark
};

constexpr bool mute_valid(const MuteConfig& c) { return c.address <= 15u; }

/// LIN mode (18.10.5's LINEN): the break SBK sends, and its detection at
/// ten or eleven bits with its own flag and interrupt.
struct LinConfig {
    bool break_11bit = false;   ///< LBDL: 11-bit detection instead of 10
    bool break_interrupt = false;
};

/// IrDA (18.7, 18.10.7): the SIR encoder on TX and decoder on RX, normal
/// mode at the bit rate (a 3/16 pulse) or low-power mode on the
/// prescaled clock. GPR.PSC is all eight bits in low-power mode and
/// must be 1 in normal mode; a prescaler of 0 "means reservation" and is
/// refused.
struct IrdaConfig {
    bool low_power = false;
    uint8_t prescaler = 1;   ///< GPR.PSC
};

constexpr bool irda_valid(const IrdaConfig& c) {
    return c.prescaler != 0u && (c.low_power || c.prescaler == 1u);
}

/// The synchronous mode's three choices (18.4, figure 18-2): the clock's
/// idle level (CPOL), the capture edge (CPHA) and whether the last data
/// bit gets a clock pulse too (LBCL). THE SENSE OF LBCL IS THIS
/// MANUAL'S, and it is the opposite of the F1 family's: 18.10.5 reads
/// "0: the clock pulse of the last bit of data is output from CK; 1: ...
/// is not output". `last_bit_clock` names the BIT and not a promise -
/// what the pad does is docs/ch32vx03/usart.md's measurement.
struct UsartSyncConfig {
    bool clock_idle_high = false;      ///< CPOL
    bool capture_second_edge = false;  ///< CPHA
    bool last_bit_clock = false;       ///< LBCL, as the register spells it
};

/// The smartcard (18.6): ISO 7816-3 on a single wire with the card's
/// clock on CK - a NACK on a parity error, the guard time in bit times
/// (GPR.GT) and the clock prescaler (GPR.PSC, the low FIVE bits, the
/// source divided by twice the value, so 31 is the deepest division and
/// 0 is reserved).
struct SmartcardConfig {
    bool nack = true;
    uint8_t guard_time = 0;
    uint8_t clock_prescaler = 1;
};

constexpr bool smartcard_valid(const SmartcardConfig& c) {
    return c.clock_prescaler != 0u && c.clock_prescaler <= 31u;
}

/// Whether this part's DEVICE CLASS has the lot's four features at all -
/// CTLR4, M_EXT, MS_ERR and RX_BUSY (the file header): the CH32V30x_D8's
/// among this stratum's classes, whose notes name no CH32V203. Whether the
/// DIE has them is a question only the die answers.
inline constexpr bool usart_class_has_lot_registers =
    device::device_class == DeviceClass::v30x_d8;

/// CTLR4's CHECK_SEL (18.10.8): the parity bit's level held - at ONE
/// (MARK) or at ZERO (SPACE) - instead of computed; `off` is the chapter's
/// "0x", the parity PCE and PS say. What the receiver does with a parity
/// bit that did not hold that level is MS_ERR, the resource's
/// mark_space_error().
enum class UsartMarkSpace : uint8_t { off = 0, mark = 2, space = 3 };

/// CTLR1's M_EXT (18.10.4): the word shorter than M makes it - seven, six
/// or five data bits; `off` is the chapter's "00: invalid, M bit
/// determines data length".
enum class UsartShortWord : uint8_t { off = 0, seven = 1, six = 2, five = 3 };

/// CTLR4's field for a choice, and the choice a field holds - the two
/// reserved codes read back as off, which is what they mean.
constexpr uint16_t usart_ctlr4_check(UsartMarkSpace m) {
    return static_cast<uint16_t>(static_cast<uint16_t>(m) << 2);
}
constexpr UsartMarkSpace usart_mark_space_of(uint16_t ctlr4) {
    const uint16_t code = static_cast<uint16_t>((ctlr4 & usart_check_sel_mask) >> 2);
    return code == 2u ? UsartMarkSpace::mark : code == 3u ? UsartMarkSpace::space
                                                          : UsartMarkSpace::off;
}
constexpr uint16_t usart_ctlr1_short_word(UsartShortWord w) {
    return static_cast<uint16_t>(static_cast<uint16_t>(w) << usart_m_ext_shift);
}
constexpr bool usart_mark_space_valid(UsartMarkSpace m) {
    return m == UsartMarkSpace::off || m == UsartMarkSpace::mark || m == UsartMarkSpace::space;
}
constexpr bool usart_short_word_valid(UsartShortWord w) { return static_cast<uint8_t>(w) <= 3u; }
/// The data bits a short word carries in DATAR: seven, six or five.
constexpr uint16_t usart_short_word_mask(UsartShortWord w) {
    return w == UsartShortWord::seven ? 0x7Fu : w == UsartShortWord::six ? 0x3Fu
                                          : w == UsartShortWord::five ? 0x1Fu : 0xFFu;
}

/// The divisor this clock and baud ask for: pclk/baud in sixteenths,
/// rounded to nearest so the error is halved. Below 16 there is no whole
/// clock period per sixteenth of a bit and the generator has nothing to
/// divide by - init() refuses such a value.
constexpr uint32_t usart_divisor(uint32_t pclk, uint32_t baud) {
    return baud == 0u ? 0u : (pclk + baud / 2u) / baud;
}

constexpr bool usart_divisor_valid(uint32_t brr) { return brr >= 16u && brr <= 0xFFFFu; }

/// The rate a divisor really gives at this clock, and the smallest clock
/// that can produce a rate at all (sixteen periods a bit).
constexpr uint32_t usart_actual_baud(uint32_t pclk, uint32_t brr) {
    return brr == 0u ? 0u : pclk / brr;
}
constexpr uint32_t usart_min_hz(uint32_t baud) { return baud * 16u; }

/// Which bus an instance answers on, and therefore which gate opens it
/// and which clock its divisor counts: USART1 alone is PB2's.
constexpr Bus usart_bus_for(uint8_t n) { return n == 1 ? Bus::pb2 : Bus::pb1; }

/// The gate of an instance (RCC_PB2PCENR bit 14, RCC_PB1PCENR bits 17..20
/// for USART2..UART5 and 6..8 for UART6..UART8 - 3.4.7, 3.4.8). A bit of a
/// block this part has not got is a gate to nothing; no verb reaches one.
constexpr uint32_t usart_gate_for(uint8_t n) {
    return n == 1 ? rcc_pb2_usart1 :
           n == 2 ? rcc_pb1_usart2 :
           n == 3 ? rcc_pb1_usart3 :
           n == 4 ? rcc_pb1_uart4 :
           n == 5 ? rcc_pb1_uart5 :
           n == 6 ? rcc_pb1_uart6 :
           n == 7 ? rcc_pb1_uart7 :
           n == 8 ? rcc_pb1_uart8 : 0;
}

/// The instance's vector - on the CH32V303 UART4 and UART5 are entries 68
/// and 69 and UART6..UART8 87..89 of that class's own tail (device.hpp's
/// Irq table); a line the part's class has not got is irq_none.
constexpr Irq usart_irq_for(uint8_t n) {
    return n == 1 ? Irq::usart1 :
           n == 2 ? Irq::usart2 :
           n == 3 ? Irq::usart3 :
           n == 4 ? Irq::uart4 :
           n == 5 ? Irq::uart5 :
           n == 6 ? Irq::uart6 :
           n == 7 ? Irq::uart7 : Irq::uart8;
}

/// The AFIO field an instance's column is selected by.
constexpr Remap usart_remap_of(uint8_t n) {
    return afio_usart_remap(n);
}

/// Whether a column exists for this instance ON THIS PART - the device
/// class has it and the package bonds at least one of its pads, which is
/// afio.hpp's one judgement and not a second table here.
constexpr bool usart_remap_valid(uint8_t n, uint8_t code) {
    return afio_remap_has_code(usart_remap_of(n), code);
}

/// The two pads a transport claims, out of the five a column carries.
struct UsartPads {
    Pad tx;
    Pad rx;
};

/// The whole column - TX, RX, CK, CTS, RTS - under a remap code. ONE
/// table answers, afio.hpp's, which is also what writes the register.
constexpr UsartPadSet usart_column_for(uint8_t n, uint8_t code = 0) {
    return afio_usart_pads(n, code);
}

constexpr UsartPads usart_pads_for(uint8_t n, uint8_t code = 0) {
    const UsartPadSet p = usart_column_for(n, code);
    return UsartPads{p.tx, p.rx};
}

/// Whether a column puts TX or RX on one of the two pads the debug port
/// owns from reset (parts/<part>.hpp's debug_swdio_* and debug_swclk_*):
/// USART3's code 2 (PA13/PA14) and UART8's code 1 (PA14/PA15), where the
/// device class and the package have those columns. A transport naming one
/// is refused by init() while the two-wire port is still the probe's.
constexpr bool usart_column_on_debug_port(uint8_t n, uint8_t code) {
    const UsartPadSet p = usart_column_for(n, code);
    const Pad dio{device::debug_swdio_port, device::debug_swdio_pin};
    const Pad clk{device::debug_swclk_port, device::debug_swclk_pin};
    return p.tx == dio || p.tx == clk || p.rx == dio || p.rx == clk;
}

/// The DMA slot each direction of an instance answers on - the controller
/// and its channel - read out of the one request table
/// (ch32vx03/dma_engine.hpp): the empty slot where the part has not got the
/// instance at all. THE CHANNEL IS THE REQUEST here, so these two slots are
/// what an engine is checked against; on the CH32V303 UART4's are DMA2's,
/// and so are UART5..UART8's (tables 11-3 and 11-4).
constexpr DmaSlot usart_dma_tx_slot(uint8_t n) {
    return n == 1 ? dma_request_channel(DmaRequest::usart1_tx) :
           n == 2 ? dma_request_channel(DmaRequest::usart2_tx) :
           n == 3 ? dma_request_channel(DmaRequest::usart3_tx) :
           n == 4 ? dma_request_channel(DmaRequest::uart4_tx) :
           n == 5 ? dma_request_channel(DmaRequest::uart5_tx) :
           n == 6 ? dma_request_channel(DmaRequest::uart6_tx) :
           n == 7 ? dma_request_channel(DmaRequest::uart7_tx) :
           n == 8 ? dma_request_channel(DmaRequest::uart8_tx) : DmaSlot{};
}

constexpr DmaSlot usart_dma_rx_slot(uint8_t n) {
    return n == 1 ? dma_request_channel(DmaRequest::usart1_rx) :
           n == 2 ? dma_request_channel(DmaRequest::usart2_rx) :
           n == 3 ? dma_request_channel(DmaRequest::usart3_rx) :
           n == 4 ? dma_request_channel(DmaRequest::uart4_rx) :
           n == 5 ? dma_request_channel(DmaRequest::uart5_rx) :
           n == 6 ? dma_request_channel(DmaRequest::uart6_rx) :
           n == 7 ? dma_request_channel(DmaRequest::uart7_rx) :
           n == 8 ? dma_request_channel(DmaRequest::uart8_rx) : DmaSlot{};
}

/// The channel numbers of those slots, 0 for none - what a message prints.
constexpr uint8_t usart_dma_tx_channel(uint8_t n) { return usart_dma_tx_slot(n).channel; }
constexpr uint8_t usart_dma_rx_channel(uint8_t n) { return usart_dma_rx_slot(n).channel; }

// =============================================================================
// Usart<n>: the resource
// =============================================================================

/**
 * One USART instance, register by register. Every verb stores what it
 * names and nothing else; a verb whose combination the chapter declares
 * undefined refuses (false, nothing written). The task below is written
 * on these verbs; a program that wants the chapter beyond the transport
 * - a break on a LIN line, a muted receiver, a clocked frame - reaches
 * them here.
 */
template <uint8_t n>
struct Usart {
    static_assert(n >= 1u && n <= 8u && usart_base_for(n) != 0,
                  "brio Usart: this family addresses USART1..USART3 and UART4..UART8");
    static_assert(device::has_usart(n),
                  "brio Usart: this part does not offer that instance - UART5..UART8 are the "
                  "CH32V303RC's and VC's, UART4 the CH32V203C8's, RB's and those two's "
                  "(parts/<part>.hpp)");

    Usart() = delete;

    static constexpr uint8_t number = n;
    static constexpr Bus bus = usart_bus_for(n);
    static constexpr UsartPads pads = usart_pads_for(n);
    static constexpr Irq irq = usart_irq_for(n);
    static constexpr DmaSlot dma_tx_slot = usart_dma_tx_slot(n);
    static constexpr DmaSlot dma_rx_slot = usart_dma_rx_slot(n);
    static constexpr uint8_t dma_tx_channel = dma_tx_slot.channel;
    static constexpr uint8_t dma_rx_channel = dma_rx_slot.channel;

    /// Whether this instance is a full USART - the synchronous clock,
    /// the smartcard and the flow-control pair - or an asynchronous
    /// receiver alone. A PART fact (the file header): the fourth port is
    /// a USART4 on the CH32V203C8 and a UART4 everywhere else, and UART5..8
    /// are UARTs on every part that has them.
    static constexpr bool is_full = device::usart_full(n);

    /// Whether the DEVICE CLASS has the lot's four features (the file
    /// header); the verbs below that name one are refused elsewhere.
    static constexpr bool has_lot_registers = usart_class_has_lot_registers;

    static UsartRegs& regs() { return *reinterpret_cast<UsartRegs*>(usart_base_for(n)); }

    /// The peripheral's clock gate. Nothing in this file reads a
    /// register before this is on: an unclocked block answers rubbish.
    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(bus, usart_gate_for(n));
        } else {
            Rcc::disable(bus, usart_gate_for(n));
        }
    }

    /// Put the block back to its reset state, gate left as it is.
    static void reset() { Rcc::reset(bus, usart_gate_for(n)); }

    /// Select the instance's column (afio.hpp). False - and nothing
    /// written - for a column this part has not got. Code 0 is the reset
    /// column and writing it is still a write, which is why the
    /// transport skips the call entirely at that code.
    static bool remap(uint8_t code) {
        Afio::clock_on();
        return Afio::remap(usart_remap_of(n), code);
    }

    // ---- configuration ----------------------------------------------------

    /// The frame and the divisor, with the port DISABLED: CTLR1's frame
    /// bits and CTLR2's stop bits may only be trusted while UE is clear
    /// or the transmitter idle, and a divisor written under traffic
    /// lands mid-frame. The two registers are written WHOLE - this is
    /// the from-scratch verb, and the modes below are what a program
    /// adds to it afterwards.
    static bool configure(const UartFormat& f, uint32_t brr) {
        if (!uart_format_valid(f) || !usart_divisor_valid(brr)) {
            return false;
        }
        regs().CTLR1 = usart_ctlr1_format(f);
        regs().CTLR2 = usart_ctlr2_stop(f.stop);
        regs().BRR = static_cast<uint16_t>(brr);
        return true;
    }

    /// The stop bits alone. Refused under IrDA, which 18.7 wants at one.
    static bool stop_bits(UartStop s) {
        if ((regs().CTLR3 & usart_iren) != 0u && s != UartStop::one) {
            return false;
        }
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~usart_stop_mask) | usart_ctlr2_stop(s));
        return true;
    }

    static bool set_brr(uint32_t v) {
        if (!usart_divisor_valid(v)) {
            return false;
        }
        regs().BRR = static_cast<uint16_t>(v);
        return true;
    }
    static uint16_t brr() { return regs().BRR; }

    static bool enabled() { return (regs().CTLR1 & usart_ue) != 0u; }

    static void enable(bool on) {
        if (on) {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_ue);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_ue);
        }
    }

    static void transmitter(bool on) {
        regs().CTLR1 = static_cast<uint16_t>(on ? (regs().CTLR1 | usart_te)
                                               : (regs().CTLR1 & ~usart_te));
    }

    static void receiver(bool on) {
        regs().CTLR1 = static_cast<uint16_t>(on ? (regs().CTLR1 | usart_re)
                                               : (regs().CTLR1 & ~usart_re));
    }

    // ---- the receiver's modes ---------------------------------------------

    /// Mute mode: WAKE and ADD written, RWU left to mute()/unmute()
    /// because 18.10.4's note 1 says the receiver must have taken a byte
    /// before an idle-line wake can work.
    static bool mute_mode(const MuteConfig& c) {
        if (!mute_valid(c)) {
            return false;
        }
        bit(regs().CTLR1, usart_wake, c.wake == MuteWake::address_mark);
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~usart_add_mask) | c.address);
        return true;
    }
    /// Put the receiver to sleep. 18.10.4's note 2: under an address-mark
    /// wake RWU cannot be written while RXNE stands - refused then.
    static bool mute() {
        if ((regs().CTLR1 & usart_wake) != 0u && (regs().STATR & usart_rxne) != 0u) {
            return false;
        }
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_rwu);
        return true;
    }
    static void unmute() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_rwu); }
    static bool muted() { return (regs().CTLR1 & usart_rwu) != 0u; }

    // ---- the line's modes -------------------------------------------------

    /// LIN mode. Refused while half duplex, IrDA, the smartcard or the
    /// clock is on - a break is a line's whole frame time and none of
    /// those shapes has room for it.
    static bool lin(const LinConfig& c) {
        if ((regs().CTLR2 & usart_clken) != 0u ||
            (regs().CTLR3 & (usart_hdsel | usart_iren | usart_scen)) != 0u) {
            return false;
        }
        uint16_t v = static_cast<uint16_t>(regs().CTLR2 & ~(usart_lbdl | usart_lbdie));
        v |= usart_linen;
        if (c.break_11bit) { v |= usart_lbdl; }
        if (c.break_interrupt) { v |= usart_lbdie; }
        regs().CTLR2 = v;
        return true;
    }
    static void lin_off() {
        regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 & ~(usart_linen | usart_lbdl | usart_lbdie));
    }
    static bool lin_enabled() { return (regs().CTLR2 & usart_linen) != 0u; }

    /// SBK: one break frame after the current one; the bit clears itself
    /// when the break's stop bit is out (18.10.4). Ten or eleven bits of
    /// low outside LIN mode, thirteen inside it.
    static void send_break() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_sbk); }
    static bool break_pending() { return (regs().CTLR1 & usart_sbk) != 0u; }

    /// Single-wire half duplex (18.5): the TX pad alone carries both
    /// directions and the chapter asks for it as an OPEN-DRAIN output,
    /// because the bus has more than one talker on it. Refused while
    /// LIN, IrDA, the smartcard or the clock is on.
    static bool half_duplex(bool on) {
        if (on && ((regs().CTLR2 & (usart_linen | usart_clken)) != 0u ||
                   (regs().CTLR3 & (usart_iren | usart_scen)) != 0u)) {
            return false;
        }
        bit(regs().CTLR3, usart_hdsel, on);
        return true;
    }
    static bool half_duplex() { return (regs().CTLR3 & usart_hdsel) != 0u; }

    /// IrDA (18.7): refused while LIN, half duplex, the smartcard or the
    /// clock is on, with stop bits other than one, and with a prescaler
    /// the register description calls reserved.
    static bool irda(const IrdaConfig& c) {
        if (!irda_valid(c)) {
            return false;
        }
        if ((regs().CTLR2 & (usart_linen | usart_clken)) != 0u ||
            (regs().CTLR3 & (usart_hdsel | usart_scen)) != 0u) {
            return false;
        }
        if ((regs().CTLR2 & usart_stop_mask) != 0u) {
            return false;
        }
        regs().GPR = static_cast<uint16_t>((regs().GPR & ~usart_psc_mask) | c.prescaler);
        uint16_t v = static_cast<uint16_t>(regs().CTLR3 & ~usart_irlp);
        if (c.low_power) { v |= usart_irlp; }
        v |= usart_iren;
        regs().CTLR3 = v;
        return true;
    }
    static void irda_off() { regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 & ~(usart_iren | usart_irlp)); }
    static bool irda_enabled() { return (regs().CTLR3 & usart_iren) != 0u; }

    /// GPR's two fields as they stand: the prescaler both IrDA and the
    /// smartcard use, and the guard time the smartcard alone does.
    static uint8_t prescaler() { return static_cast<uint8_t>(regs().GPR & usart_psc_mask); }
    static uint8_t guard_time() { return static_cast<uint8_t>((regs().GPR & usart_gt_mask) >> 8); }

    /// The smartcard (18.6), a FULL instance's - refused at compile time
    /// on a UART. Sets the guard time and the card-clock prescaler, 1.5
    /// stop bits, CLKEN and SCEN: the clock is the CARD'S and belongs to
    /// the mode, which is why this verb writes it rather than leaving it
    /// to a synchronous() the chapter's own exclusions would refuse under
    /// SCEN. What is not written is the CK pad - the caller hands that
    /// over (usart_column_for(n, code).ck) - and CPOL, CPHA and LBCL,
    /// which keep whatever a program chose. Refused while LIN, half duplex
    /// or IrDA is on.
    static bool smartcard(const SmartcardConfig& c) {
        static_assert(is_full, "brio Usart: the smartcard is a full USART's - this instance is "
                               "a UART, TX and RX alone (18.6; parts/<part>.hpp's usart_full)");
        if (!smartcard_valid(c)) {
            return false;
        }
        if ((regs().CTLR2 & usart_linen) != 0u ||
            (regs().CTLR3 & (usart_hdsel | usart_iren)) != 0u) {
            return false;
        }
        regs().GPR = static_cast<uint16_t>((static_cast<uint16_t>(c.guard_time) << 8) |
                                           c.clock_prescaler);
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~usart_stop_mask) |
                                             usart_ctlr2_stop(UartStop::one_and_half) |
                                             usart_clken);
        uint16_t v = static_cast<uint16_t>(regs().CTLR3 & ~usart_nack);
        if (c.nack) { v |= usart_nack; }
        v |= usart_scen;
        regs().CTLR3 = v;
        return true;
    }
    /// SCEN and its NACK dropped. The CARD CLOCK IS LEFT RUNNING: CLKEN
    /// is the synchronous half's bit and synchronous_off() is what stops
    /// it, under its own TE/RE rule.
    static void smartcard_off() {
        static_assert(is_full, "brio Usart: the smartcard is a full USART's - this instance is "
                               "a UART, TX and RX alone (18.6)");
        regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 & ~(usart_scen | usart_nack));
    }
    /// A question every instance may be asked: false on a UART, which has
    /// no smartcard to be in.
    static bool smartcard_enabled() {
        if constexpr (!is_full) {
            return false;
        } else {
            return (regs().CTLR3 & usart_scen) != 0u;
        }
    }

    /// THE SYNCHRONOUS MODE (18.4), a FULL instance's - refused at compile
    /// time on a UART: the column's CK pad carries a clock while the
    /// TRANSMITTER shifts and at no other time, and the receiver samples
    /// on it - this side is the master and CK is an output only. Refused
    /// while LIN, half duplex, IrDA or the smartcard is on, and while the
    /// transmitter or the receiver is enabled, because CPOL, CPHA and LBCL
    /// "need to be set when TE and RE are not enabled". The CK pad itself
    /// is the caller's to hand to the peripheral (usart_column_for(n,
    /// code).ck).
    static bool synchronous(const UsartSyncConfig& c) {
        static_assert(is_full, "brio Usart: the synchronous clock is a full USART's - this "
                               "instance is a UART and has no CK pad (18.4)");
        if ((regs().CTLR2 & usart_linen) != 0u ||
            (regs().CTLR3 & (usart_hdsel | usart_iren | usart_scen)) != 0u) {
            return false;
        }
        if ((regs().CTLR1 & (usart_te | usart_re)) != 0u) {
            return false;
        }
        uint16_t v = static_cast<uint16_t>(regs().CTLR2 & ~(usart_cpol | usart_cpha | usart_lbcl));
        if (c.clock_idle_high) { v |= usart_cpol; }
        if (c.capture_second_edge) { v |= usart_cpha; }
        if (c.last_bit_clock) { v |= usart_lbcl; }
        v |= usart_clken;
        regs().CTLR2 = v;
        return true;
    }
    /// CLKEN and its three companions dropped; the same TE/RE rule.
    static bool synchronous_off() {
        static_assert(is_full, "brio Usart: the synchronous clock is a full USART's - this "
                               "instance is a UART and has no CK pad (18.4)");
        if ((regs().CTLR1 & (usart_te | usart_re)) != 0u) {
            return false;
        }
        regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 &
                                             ~(usart_clken | usart_cpol | usart_cpha | usart_lbcl));
        return true;
    }
    /// False on a UART, which has no clock to run.
    static bool synchronous_enabled() {
        if constexpr (!is_full) {
            return false;
        } else {
            return (regs().CTLR2 & usart_clken) != 0u;
        }
    }

    /// The hardware flow-control pair (18.10.6), a FULL instance's -
    /// refused at compile time on a UART: RTS driven low while the
    /// receiver can take a frame, CTS sampled before each frame goes out.
    static bool flow_control(bool rts, bool cts) {
        static_assert(is_full, "brio Usart: the flow-control pair is a full USART's - this "
                               "instance is a UART, TX and RX alone (18.10.6)");
        bit(regs().CTLR3, usart_rtse, rts);
        bit(regs().CTLR3, usart_ctse, cts);
        return true;
    }
    static bool rts_enabled() { return (regs().CTLR3 & usart_rtse) != 0u; }
    static bool cts_enabled() { return (regs().CTLR3 & usart_ctse) != 0u; }

    /// CTLR3's two DMA request enables (18.10.6). With DMAT set, TXE
    /// raises a request instead of feeding the interrupt; with DMAR
    /// set, so does RXNE - which is why a receiver with an engine
    /// leaves RXNEIE alone.
    static void dma_transmit(bool on) {
        regs().CTLR3 = static_cast<uint16_t>(on ? (regs().CTLR3 | usart_dmat)
                                               : (regs().CTLR3 & ~usart_dmat));
    }
    static void dma_receive(bool on) {
        regs().CTLR3 = static_cast<uint16_t>(on ? (regs().CTLR3 | usart_dmar)
                                               : (regs().CTLR3 & ~usart_dmar));
    }

    /// Where an engine points: the one register both directions share.
    static volatile void* data_address() { return static_cast<volatile void*>(&regs().DATAR); }

    // ---- interrupts and flags ---------------------------------------------

    /// CTLR1's five enables by mask: usart_peie, usart_txeie, usart_tcie,
    /// usart_rxneie (ORE rides it), usart_idleie.
    static void interrupts(uint16_t ctlr1_mask, bool on) { bit(regs().CTLR1, ctlr1_mask, on); }
    static void rxne_interrupt(bool on) { bit(regs().CTLR1, usart_rxneie, on); }
    static void txe_interrupt(bool on) { bit(regs().CTLR1, usart_txeie, on); }
    static bool txe_interrupt() { return (regs().CTLR1 & usart_txeie) != 0u; }
    static void tc_interrupt(bool on) { bit(regs().CTLR1, usart_tcie, on); }
    static void idle_interrupt(bool on) { bit(regs().CTLR1, usart_idleie, on); }
    static void parity_interrupt(bool on) { bit(regs().CTLR1, usart_peie, on); }
    static void break_interrupt(bool on) { bit(regs().CTLR2, usart_lbdie, on); }
    /// CTSIE, the flow-control pair's own interrupt - a full USART's.
    static void cts_interrupt(bool on) {
        static_assert(is_full, "brio Usart: CTS and its interrupt are a full USART's - this "
                               "instance is a UART, TX and RX alone (18.10.6)");
        bit(regs().CTLR3, usart_ctsie, on);
    }
    /// EIE: FE, ORE and NE raise the vector - under DMAR only (18.10.6).
    static void error_interrupt(bool on) { bit(regs().CTLR3, usart_eie, on); }

    static uint16_t status() { return regs().STATR; }
    static bool flag(uint16_t mask) { return (regs().STATR & mask) != 0u; }
    /// The write-zero-to-clear flags (RXNE, TC, LBD, CTS); any other bit
    /// in the mask is ignored, because writing it does nothing.
    static void clear_flags(uint16_t mask) {
        regs().STATR = static_cast<uint16_t>(~(mask & usart_statr_rw0));
    }
    /// The read-sequence clear of IDLE, ORE, NE, FE and PE (and of RXNE,
    /// whose byte this discards).
    static void clear_by_read() {
        (void)regs().STATR;
        (void)regs().DATAR;
    }

    // ---- the line ---------------------------------------------------------

    static bool tx_empty() { return (regs().STATR & usart_txe) != 0u; }
    static bool tx_complete() { return (regs().STATR & usart_tc) != 0u; }
    static bool rx_ready() { return (regs().STATR & usart_rxne) != 0u; }

    /// The word as the receiver has it - nine bits when the frame is
    /// nine bits wide. Reading DATAR is what clears RXNE, and the
    /// STATR-then-DATAR pair is what clears the error flags.
    static uint16_t read_word() { return static_cast<uint16_t>(regs().DATAR); }
    static void write_word(uint16_t w) { regs().DATAR = w; }
    static uint8_t read_data() { return static_cast<uint8_t>(regs().DATAR & 0xFFu); }
    static void write_data(uint8_t b) { regs().DATAR = b; }

    /// Read the errors and clear them, the sequence the chapter
    /// prescribes (18.10.1): the status register, then the data one.
    static uint16_t take_errors() {
        const uint16_t status = regs().STATR;
        const uint16_t errors = static_cast<uint16_t>(status & (usart_pe | usart_fe | usart_ne | usart_ore));
        if (errors != 0u) {
            (void)regs().DATAR;
        }
        return errors;
    }

    /// The baud the port is ACTUALLY running at, from the divisor in the
    /// register - what a program reports instead of what it asked for.
    static uint32_t actual_baud(uint32_t pclk) {
        const uint32_t brr = regs().BRR;
        return brr == 0u ? 0u : pclk / brr;
    }

    // ---- the lot's registers (18.10.1, 18.10.4, 18.10.8) -------------------
    //
    // The CH32V30x_D8's, and there only on the lots the notes name (the
    // file header). Every verb below is refused at compile time on the
    // CH32V203's two classes; on a CH32V303 it asks the die before it
    // trusts a bit.

    /**
     * Does THIS DIE have CTLR4? The answer is the silicon's, kept once
     * found. Asked with the port DISABLED (UE clear) and quiet, because
     * the probe writes the address and an absent register can be a MIRROR
     * of another one of this block: false - nothing written, nothing kept -
     * while UE is set.
     *
     *  1. The word is read first. The real CTLR4 holds nothing outside
     *     MS_ERRIE and CHECK_SEL (18.10.8), so a word with any other bit
     *     set is another register's, and the answer is no with no write.
     *  2. Every other register of the block is read, CTLR4's three bits
     *     are written INVERTED, and everything is read again: the register
     *     is there when the three bits came back and nothing else moved,
     *     and its own word is then put back.
     *  3. When anything else moved, the address was a mirror: the block is
     *     put back through its reset line and the program's BRR, CTLR2,
     *     CTLR3, GPR and CTLR1 are written again - the status flags going
     *     to their reset values, which on a disabled port is what they
     *     were. A die whose address holds nothing at all moves nothing and
     *     keeps nothing, and the answer is no.
     */
    static bool ctlr4_present() {
        static_assert(has_lot_registers,
                      "brio Usart: CTLR4 is the CH32V30x_D8's register (18.10.8's note) - no "
                      "CH32V203 has it");
        if (ctlr4_ != 0u) {
            return ctlr4_ == 1u;
        }
        UsartRegs& r = regs();
        if ((r.CTLR1 & usart_ue) != 0u) {
            return false;
        }
        const uint16_t word = r.CTLR4;
        if ((word & static_cast<uint16_t>(~usart_ctlr4_bits)) != 0u) {
            ctlr4_ = 2;
            return false;
        }
        const uint16_t statr = r.STATR;
        const uint16_t brr = r.BRR;
        const uint16_t c1 = r.CTLR1;
        const uint16_t c2 = r.CTLR2;
        const uint16_t c3 = r.CTLR3;
        const uint16_t gpr = r.GPR;
        const uint16_t probe = static_cast<uint16_t>(word ^ usart_ctlr4_bits);
        r.CTLR4 = probe;
        const bool took = (r.CTLR4 & usart_ctlr4_bits) == probe;
        const bool still = r.STATR == statr && r.BRR == brr && r.CTLR1 == c1 && r.CTLR2 == c2 &&
                           r.CTLR3 == c3 && r.GPR == gpr;
        if (took && still) {
            r.CTLR4 = word;
            ctlr4_ = 1;
            return true;
        }
        if (!still) {
            reset();
            r.BRR = brr;
            r.CTLR2 = c2;
            r.CTLR3 = c3;
            r.GPR = gpr;
            r.CTLR1 = c1;
        }
        ctlr4_ = 2;
        return false;
    }

    /// What the probe has said so far: true only where ctlr4_present()
    /// found the register. A question that writes nothing, for a program
    /// that wants to know before it asks.
    static bool ctlr4_known() {
        static_assert(has_lot_registers,
                      "brio Usart: CTLR4 is the CH32V30x_D8's register (18.10.8's note)");
        return ctlr4_ == 1u;
    }

    /**
     * CTLR4's CHECK_SEL: the parity bit held at ONE (mark) or at ZERO
     * (space) instead of computed, or back to the computed one (off). The
     * FRAME that carries the bit - M and PCE - is configure()'s: a
     * parity bit has to be there for its level to be held. False, with
     * nothing written, on a die whose CTLR4 is not there (the probe runs
     * on the first call, which wants the port disabled) and for a code the
     * field does not have; true when the field reads back as asked.
     */
    static bool mark_space(UsartMarkSpace m) {
        static_assert(has_lot_registers,
                      "brio Usart: the MARK and SPACE parity is CTLR4's, the CH32V30x_D8's "
                      "register (18.10.8's note) - no CH32V203 has it");
        if (!usart_mark_space_valid(m) || !ctlr4_present()) {
            return false;
        }
        UsartRegs& r = regs();
        r.CTLR4 = static_cast<uint16_t>((r.CTLR4 & usart_ms_errie) | usart_ctlr4_check(m));
        return usart_mark_space_of(r.CTLR4) == m;
    }
    /// The level the parity bit is held at - `off` on a die the probe has
    /// not found the register on, whose address is not read at all.
    static UsartMarkSpace mark_space() {
        static_assert(has_lot_registers,
                      "brio Usart: the MARK and SPACE parity is CTLR4's (18.10.8's note)");
        return ctlr4_known() ? usart_mark_space_of(regs().CTLR4) : UsartMarkSpace::off;
    }
    /// MS_ERRIE: a parity bit that did not hold its level raises the
    /// vector. The same probe and the same read-back as mark_space().
    static bool mark_space_interrupt(bool on) {
        static_assert(has_lot_registers,
                      "brio Usart: MS_ERRIE is CTLR4's, the CH32V30x_D8's register (18.10.8)");
        if (!ctlr4_present()) {
            return false;
        }
        bit(regs().CTLR4, usart_ms_errie, on);
        return ((regs().CTLR4 & usart_ms_errie) != 0u) == on;
    }
    /// MS_ERR (18.10.1): the received parity bit did not hold the level
    /// CHECK_SEL asks. Cleared as PE is, by clear_by_read(). Reads zero on
    /// a die without the lot's registers - it is a bit of a register that
    /// is there.
    static bool mark_space_error() {
        static_assert(has_lot_registers,
                      "brio Usart: MS_ERR is the CH32V30x_D8's status bit (18.10.1's note)");
        return (regs().STATR & usart_ms_err) != 0u;
    }
    /// RX_BUSY (18.10.1): the receiver is inside a frame. Zero on a die
    /// without the lot's registers, which no single read can tell from a
    /// receiver at rest - the question is a frame's, and the reference
    /// suite asks it while one arrives.
    static bool receiving() {
        static_assert(has_lot_registers,
                      "brio Usart: RX_BUSY is the CH32V30x_D8's status bit (18.10.1's note)");
        return (regs().STATR & usart_rx_busy) != 0u;
    }

    /**
     * CTLR1's M_EXT: a word of seven, six or five data bits, or back to
     * the word M says (off). A field of a register every die has, so it
     * is written and read back: false, with the field put back to off,
     * where the die did not keep it. Refused while the port is enabled -
     * a frame's shape changes with UE clear, as configure()'s does - and
     * configure() writes CTLR1 whole, so this comes after it. Whether the
     * parity bit PCE adds is counted in the short word or beside it the
     * chapter does not say.
     */
    static bool short_word(UsartShortWord w) {
        static_assert(has_lot_registers,
                      "brio Usart: M_EXT is the CH32V30x_D8's field (18.10.4's note) - no CH32V203 "
                      "has five, six or seven-bit words");
        UsartRegs& r = regs();
        if (!usart_short_word_valid(w) || (r.CTLR1 & usart_ue) != 0u) {
            return false;
        }
        const uint16_t field = usart_ctlr1_short_word(w);
        r.CTLR1 = static_cast<uint16_t>((r.CTLR1 & ~usart_m_ext_mask) | field);
        if ((r.CTLR1 & usart_m_ext_mask) != field) {
            r.CTLR1 = static_cast<uint16_t>(r.CTLR1 & ~usart_m_ext_mask);
            return false;
        }
        return true;
    }
    static UsartShortWord short_word() {
        static_assert(has_lot_registers,
                      "brio Usart: M_EXT is the CH32V30x_D8's field (18.10.4's note)");
        return static_cast<UsartShortWord>((regs().CTLR1 & usart_m_ext_mask) >> usart_m_ext_shift);
    }

private:
    static void bit(volatile uint16_t& r, uint16_t mask, bool on) {
        r = static_cast<uint16_t>(on ? (r | mask) : (r & ~mask));
    }

    /// What ctlr4_present() found: 0 not asked yet, 1 there, 2 not there.
    static inline uint8_t ctlr4_ = 0;
};

// =============================================================================
// Uart: the transport task
// =============================================================================

/// The peripheral clock an instance counts its divisor in: USART1 is the
/// PB2 one and every other port is PB1's, and the two buses do NOT run
/// at the same rate above the cap ch32vx03/clock.hpp states.
template <uint8_t instance, typename C>
constexpr uint32_t usart_bus_hz(C clock) {
    if constexpr (C::is_static) {
        (void)clock;
        return instance == 1u ? C::pclk2_hz : C::pclk1_hz;
    } else {
        (void)clock;
        return instance == 1u ? C::pclk2_hz() : C::pclk1_hz();
    }
}

/// The same answer from an HCLK, which is what a dynamic clock hands a
/// user it is about to rebase: the RCC still holds the prescalers of the
/// rate being left, so the new bus rate is arithmetic and not a read.
template <uint8_t instance>
constexpr uint32_t usart_bus_hz_at(uint32_t hclk) {
    return instance == 1u ? pclk2_hz_at(hclk) : pclk1_hz_at(hclk);
}

/**
 * What a Uart may be told beyond its frame, its rings and its engines,
 * as one trailing template parameter. The FRAME IS NOT IN HERE on this
 * family - it has a parameter of its own, ahead of the engine slots -
 * which is this stratum's spelling of the CH32V00x's UartOptions
 * (docs/design/serial.md's realizations table records it). The defaults
 * are the console's personality and compile to exactly the code they
 * did before this struct existed.
 */
struct UartOptions {
    /// Single-wire half duplex (18.5): one pad, the TX pad, as an
    /// alternate-function OPEN DRAIN against an external pull-up; the RX
    /// pad is left alone. What the instance's own receiver hears of what
    /// it sends is docs/ch32vx03/usart.md's measurement.
    bool half_duplex = false;
    /// The flow-control pair on the column's CTS and RTS pads, on a full
    /// instance (Usart<n>::is_full).
    bool rts = false;
    bool cts = false;
};

/**
 * The receive ring a transport keeps, chosen by whether its receive has an
 * engine: a SkipRing, which the handler pushes into and tells when it lost
 * a byte, where it has none; and where
 * it has one, the CONSUMER HALF of a ring the channel itself writes -
 * util/ring.hpp's HardwareRing over a storage array the transport owns
 * (keyed by the transport's type), the receive engine its RingCounter. A
 * partial specialization and not std::conditional_t, because naming
 * HardwareRing over NoDmaEngine would be checked even in the arm not taken.
 * Where the engine's channel is held to the 64 KB rule (the CH32V303's
 * DMA1) the storage is aligned to its own size, a power of two, so no
 * placement the linker chooses can straddle a 64 KB page.
 */
template <bool engine, typename Owner, uint16_t size, typename Engine, typename P>
struct UartRxRing {
    using type = SkipRing<uint8_t, size, P>;
};
template <typename Owner, uint16_t size, typename Engine, typename P>
struct UartRxRing<true, Owner, size, Engine, P> {
    static_assert(size >= 2u && size <= 0x8000u && std::has_single_bit(size),
                  "brio Uart: a receive engine runs the whole receive ring as one circular "
                  "block - a power of two, 2..32768 bytes (the ring's arithmetic is a mask, and "
                  "CNTR counts 65535 at most)");
    alignas(Engine::bounded_to_64k ? size : 1u) static inline uint8_t storage[size]{};
    using type = HardwareRing<storage, Engine>;
};

/**
 * The interrupt-driven serial port.
 *
 *   using Serial = brio::Uart<1, Platform>;
 *   constexpr Serial serial;                 // tag for print(serial, ...)
 *   Serial::init(clock, 115200);
 *   BRIO_CH32_VECTOR(usart1_handler) { Serial::isr(); }
 *
 * `P` is the platform, which the rings need to know whether an index can
 * be shared with a handler bare (atomic_width) or wants a guard.
 */
template <uint8_t instance, typename P, uint16_t rx_size = 64, uint16_t tx_size = 64,
          UartFormat format = {}, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine, uint8_t remap = 0, UartOptions opts = {}>
struct Uart {
    static_assert(instance >= 1u && instance <= 8u && usart_base_for(instance) != 0,
                  "brio Uart: this family addresses USART1..USART3 and UART4..UART8");
    static_assert(device::has_usart(instance),
                  "brio Uart: this part does not offer that instance - UART5..UART8 are the "
                  "CH32V303RC's and VC's (parts/<part>.hpp)");
    static_assert(uart_format_valid(format) && format.bits != UartBits::nine,
                  "brio Uart: the rings carry bytes - seven data bits with a parity bit, or eight, "
                  "with or without one; nine-bit words are the resource's read_word()/write_word()");
    static_assert(usart_remap_valid(instance, remap),
                  "brio Uart: no such column for this instance on this part (afio.hpp's tables "
                  "10-23 to 10-27 - the device class has it and the package bonds its pads)");
    static_assert(!opts.rts || device::usart_full(instance),
                  "brio Uart: the flow-control pair is a full USART's - the fourth port has it on "
                  "the CH32V203C8 alone, and UART5..UART8 nowhere (18.10.6)");
    static_assert(!opts.cts || device::usart_full(instance),
                  "brio Uart: the flow-control pair is a full USART's - the fourth port has it on "
                  "the CH32V203C8 alone, and UART5..UART8 nowhere (18.10.6)");
    static_assert(!opts.rts || pad_bonded(usart_column_for(instance, remap).rts),
                  "brio Uart: this package does not bond this column's RTS pad - the signal exists "
                  "and the pin does not (parts/<part>.hpp)");
    static_assert(!opts.cts || pad_bonded(usart_column_for(instance, remap).cts),
                  "brio Uart: this package does not bond this column's CTS pad - the signal exists "
                  "and the pin does not (parts/<part>.hpp)");
    // An engine is checked where it is NAMED: sizeof demands a complete
    // type, so an engine's own static_asserts fire on the line the
    // application wrote.
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / DmaRxEngine from "
                  "ch32vx03/dma.hpp, or NoDmaEngine (the default)");
    static_assert(!TxEngine::present ||
                      dma_engine_slot<TxEngine>() == usart_dma_tx_slot(instance),
                  "brio Uart: RM 11.2.3 - this instance transmits on its own DMA slot, controller "
                  "AND channel (USART1 on DMA1's 4, USART2 on 7, USART3 on 2; UART4 on DMA1's 1 "
                  "on the CH32V203 and on DMA2's 5 on the CH32V303; UART5..UART8 on DMA2's 4, 6, "
                  "8 and 10)");
    static_assert(!RxEngine::present ||
                      dma_engine_slot<RxEngine>() == usart_dma_rx_slot(instance),
                  "brio Uart: RM 11.2.3 - this instance receives on its own DMA slot, controller "
                  "AND channel (USART1 on DMA1's 5, USART2 on 6, USART3 on 3; UART4 on DMA1's 8 "
                  "on the CH32V203 and on DMA2's 3 on the CH32V303; UART5..UART8 on DMA2's 2, 7, "
                  "9 and 11)");
    static_assert(dma_engines_distinct<TxEngine, RxEngine>(),
                  "the two engines of a Uart must not share a DMA channel");

    Uart() = default;   // a tag instance: constexpr Uart<1, P> serial;

    using Resource = Usart<instance>;

    static constexpr uint8_t number = instance;
    static constexpr uint8_t remap_code = remap;
    static constexpr UsartPads pads = usart_pads_for(instance, remap);
    static constexpr UsartPadSet column = usart_column_for(instance, remap);
    static constexpr UartOptions options = opts;
    static constexpr bool has_tx_engine = TxEngine::present;
    static constexpr bool has_rx_engine = RxEngine::present;

    using Tx = Pin<pads.tx.port, pads.tx.pin>;
    using Rx = Pin<pads.rx.port, pads.rx.pin>;

    static UsartRegs& regs() { return Resource::regs(); }

    /**
     * Open the port at `baud` in the given frame (8N1 by default), with
     * the receive interrupt armed.
     *
     * Returns false when the divisor comes out below the smallest the
     * chapter allows (16, i.e. one whole peripheral-clock period per bit
     * sixteenth): a baud rate this clock cannot serve is a fact the
     * caller must see rather than a port that mangles every frame.
     */
    template <typename C>
    static bool init(C clock, uint32_t baud) {
        static_assert(clock_follows<C, Uart>(),
                      "brio Uart: the baud divisor is derived from the peripheral clock, so a "
                      "dynamic clock must list this port among the users it rebases");

        const uint32_t pclk = usart_bus_hz<instance>(clock);
        const uint32_t brr = usart_divisor(pclk, baud);
        if (!usart_divisor_valid(brr)) {
            return false;
        }

        // A column on the debug port's own pads is refused while the
        // two-wire port is still the probe's (the file header): its pads
        // configured here would fight the link that put the image there.
        if constexpr (usart_column_on_debug_port(instance, remap)) {
            if (Afio::debug_port_enabled()) {
                return false;
            }
        }

        Resource::bus_clock(true);

        // The column, only where it is not the reset one: AFIO's reset
        // value already selects code 0, and USART1 carries this board's
        // console - a write there would move the console's pads.
        if constexpr (remap != 0u) {
            (void)Resource::remap(remap);
        }

        // Pads before the enable: TE's idle frame must land on a pad the
        // peripheral already owns.
        if constexpr (opts.half_duplex) {
            Tx::function(PinDrive::open_drain);   // the one wire; RX is left alone
        } else {
            Tx::function();                  // alternate function, push-pull
            Rx::input();                     // floating: the peer drives it
        }
        if constexpr (opts.rts) {
            Rts::function();
        }
        if constexpr (opts.cts) {
            Cts::input(PinPull::up);         // unconnected reads "not clear to send"
        }

        if (!Resource::configure(format, brr)) {
            return false;
        }
        m_baud = baud;

        m_tx.clear();
        m_rx.clear();
        if constexpr (has_rx_engine) {
            m_rx.clear_overruns();
        }
        m_rx_overruns = 0;
        m_hw_overruns = 0;
        m_frame_errors = 0;
        m_noise_errors = 0;
        m_parity_errors = 0;

        // CTLR3 IS TOUCHED ONLY BY A PORT THAT HAS SOMETHING TO PUT IN
        // IT: with no engine and no option there is nothing of this
        // transport's there, and whatever a program stored itself stays.
        if constexpr (has_tx_engine || has_rx_engine || opts.half_duplex || opts.rts || opts.cts) {
            m_dma_faults = 0;
            uint16_t ctlr3 = regs().CTLR3;
            if constexpr (has_tx_engine) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_dmat); }
            if constexpr (has_rx_engine) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_dmar); }
            if constexpr (opts.half_duplex) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_hdsel); }
            if constexpr (opts.rts) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_rtse); }
            if constexpr (opts.cts) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_ctse); }
            regs().CTLR3 = ctlr3;
        }

        // RXNE belongs to the receive channel when an engine has it, so
        // the per-byte interrupt is armed only where no engine is; with
        // one, the vector waits for the end of a burst - the idle line and
        // the errors (EIE under DMAR, PE on its own enable).
        regs().CTLR1 = static_cast<uint16_t>(
            usart_ctlr1_format(format) | usart_ue | usart_te | usart_re |
            (has_rx_engine ? static_cast<uint16_t>(usart_idleie | usart_peie) : usart_rxneie));

        if constexpr (has_rx_engine) {
            regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 | usart_eie);
            m_rx_waiting = false;
            // The channel takes RXNE - into the whole receive ring, lap
            // after lap, from here on, its half and full marks the edge of
            // a stream with no silence. A ring the engine refuses (its
            // storage misplaced for the channel) is a port that cannot
            // receive, which the caller is told.
            if (!RxEngine::arm(Resource::data_address(), RxRing::storage, true)) {
                return false;
            }
            m_rx_drained = true;
            (void)RxEngine::start();
        }
        if constexpr (has_tx_engine) {
            TxEngine::arm(Resource::data_address());
        }

        Pfic::enable(usart_irq_for(instance));
        return true;
    }

    /**
     * The ISR body of WHICHEVER DMA channel this transport owns - bind
     * it to the vector(s) the engines' channels report on:
     *
     *     BRIO_CH32_VECTOR(dma1_channel7_handler) { (void)Serial::dma_isr(); }
     *
     * Each engine reads only its own channel's flags, so this is safe
     * on a vector another channel of the program shares nothing with.
     * On the transmit channel a completion releases exactly the block's
     * bytes from the ring and starts the next run. On the receive channel
     * a completion is a LAP of the ring, counted - the ring's producer
     * index is the channel's count plus these laps - and the lap's half
     * and full marks are the edge of a stream that does not fall silent;
     * a transfer error stops the channel, and the consumer's next look
     * starts it again, the vector calling it.
     *
     * Returns true when the receive ring holds bytes its consumer has not
     * been told of - the edge, as isr()'s: post RxActivity on true.
     */
    [[gnu::always_inline]] static bool dma_isr() {
        bool edge = false;
        if constexpr (has_tx_engine) {
            const uint8_t f = TxEngine::service();
            if ((f & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                bump(m_dma_faults);
            } else if ((f & TxEngine::flag_complete) != 0u) {
                m_tx.consume(static_cast<typename decltype(m_tx)::index_t>(TxEngine::complete()));
                pump_tx();
            }
        }
        if constexpr (has_rx_engine) {
            const uint8_t f = RxEngine::service();
            if ((f & RxEngine::flag_error) != 0u) {
                (void)RxEngine::abandon();
                bump(m_dma_faults);
                m_rx_drained = false;
                edge = true;   // the consumer must come: its look starts the ring again
            } else if (f != 0u) {
                if ((f & RxEngine::flag_complete) != 0u) {
                    RxEngine::lap();   // a lap: counted, nothing re-armed
                }
                edge = told();   // the half or the full mark of a lap
            }
        }
        return edge;
    }

    /**
     * The receive edge asked from the consumer's side, and the ring's
     * housekeeping.
     *
     * NOBODY NEEDS TO ASK: isr() and dma_isr() report the edge - the idle
     * line, a burst's first frame, the lap's marks (the file header). This
     * is the same gate from the main context, for an owner that still
     * asks: true when the ring holds bytes and the consumer has found it
     * empty since the last true, from a vector or from here, so asking
     * twice is answering once. THE SILICON IS ASKED, NOT THE ARITHMETIC: a
     * ring never stops on its own, so a channel that is not running was
     * stopped by a transfer error (11.2.1) and is started again from the
     * storage's first element, the view's positions with it - the consumer
     * is this context, so the view's clear() has no reader to race; every
     * read verb does the same. It reads neither STATR nor DATAR: the
     * errors are the vector's to count. False, and free, without an
     * engine.
     */
    static bool harvest() {
        if constexpr (!has_rx_engine) {
            return false;
        } else {
            restart_if_stopped();
            // waiting(): a look that writes nothing - empty() may skip,
            // moving the tail under a run the consumer holds.
            if (!m_rx_drained || m_rx.waiting() == 0u) {
                return false;
            }
            m_rx_drained = false;
            return true;
        }
    }

    /// Blocks the engines threw away - a transfer error's, and a receive
    /// channel given up on a frame it never took. Always 0, and free,
    /// without one.
    static uint16_t dma_faults() { return m_dma_faults; }

    /**
     * The port's whole interrupt body - bind this instance's vector to
     * it.
     *
     * Returns true when a byte arrived into an EMPTY ring: the edge
     * util/serial_port.hpp posts on.
     */
    // always_inline: one call site (the app's vector binding), so the
    // compiler saves only the registers this body uses.
    //
    // WITH A RECEIVE ENGINE THE HANDLER NEVER READS DATAR: RXNE is the
    // channel's request, and a read would take a byte from under the
    // channel. Its receive half is the two states of the file header -
    // the wait for a frame first, which reads no STATR, and with a
    // transmit engine nothing else of this port raises the vector.
    [[gnu::always_inline]] static bool isr() {
        bool rx_edge = false;
        bool waited = false;
        if constexpr (has_rx_engine) {
            if (m_rx_waiting) {
                waited = true;
                rx_edge = rx_frame_entry();
                if constexpr (has_tx_engine) {
                    return rx_edge;
                }
            }
        }

        const uint16_t status = regs().STATR;

        if constexpr (!has_rx_engine) {
            if ((status & usart_rxne) != 0u) {
                const uint8_t byte = static_cast<uint8_t>(regs().DATAR & 0xFFu);
                // Every byte lost is reported to the receive ring
                // (lost()): the flagged byte dropped here - with an
                // overrun's, lost in the shift register - and one a full
                // ring refuses. The consumer's next look skips everything
                // queued.
                if ((status & (usart_fe | usart_ne | usart_pe | usart_ore)) != 0u) {
                    if ((status & usart_fe) != 0u) { bump(m_frame_errors); }
                    if ((status & usart_ne) != 0u) { bump(m_noise_errors); }
                    if ((status & usart_pe) != 0u) { bump(m_parity_errors); }
                    if ((status & usart_ore) != 0u) { bump(m_hw_overruns); }
                    m_rx.lost();
                } else {
                    const bool was_empty = m_rx.empty();
                    if (m_rx.push(byte)) {
                        rx_edge = was_empty;
                    } else {
                        bump(m_rx_overruns);
                        m_rx.lost();
                    }
                }
            }
        } else if (!waited) {
            rx_edge = rx_end_entry(status);
        }

        if ((status & usart_txe) != 0u && (regs().CTLR1 & usart_txeie) != 0u) {
            if (const auto next = m_tx.pop()) {
                regs().DATAR = *next;
            } else {
                // TXE stands while the transmitter is idle: disarm, or
                // the handler is re-entered for ever.
                regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_txeie);
            }
        }

        return rx_edge;
    }

    // ---- byte transport (ByteSink / ByteSource) ---------------------------

    /// Queue one byte; false when the TX ring is full (print() spins).
    /// Without an engine TXEIE is armed; with one the engine is pumped -
    /// and a REFUSED byte pumps it too. print() answers false by trying
    /// again for ever, and the ring is full exactly when nothing is
    /// draining it: an engine left idle by a block dma_isr() abandoned
    /// would make that spin a deadlock, where the TX policy promises a
    /// stall (docs/design/serial.md). The plain transport needs no such
    /// pump: the push that filled the ring armed TXEIE, a level that
    /// cannot be missed.
    static bool write_byte(uint8_t b) {
        if (!m_tx.push(b)) {
            if constexpr (has_tx_engine) {
                pump_tx();
            }
            return false;
        }
        if constexpr (has_tx_engine) {
            pump_tx();
        } else {
            arm_txe();
        }
        return true;
    }

    /// Take one received byte; false when none is pending - or, with a
    /// receive engine, when the byte was written over while it was being
    /// read (the ring counts it in rx_overruns()).
    static bool read_byte(uint8_t& b) {
        if constexpr (has_rx_engine) {
            const std::span<const uint8_t> run = look();
            if (run.empty()) {
                return false;
            }
            b = run[0];
            return m_rx.consume(1u);
        } else {
            const auto v = m_rx.pop();
            if (!v) {
                return false;
            }
            b = *v;
            return true;
        }
    }

    /// Queue a RUN (util/stream.hpp's BulkSink): as much of `src` as the
    /// ring has room for, copied into the ring's own free run and handed
    /// over with one index store per contiguous part. Never blocks;
    /// returns how many were queued. A run that finds no room is
    /// write_byte()'s refusal: nothing armed on the plain transport (the
    /// push that filled the ring armed TXEIE, a level), the engine pumped
    /// on the engined one.
    ///
    /// THE FIRST BYTE GOES BEFORE THE COPY, on the plain transport: it is
    /// pushed and TXEIE armed exactly as write_byte() does, so an idle
    /// transmitter starts on it while the rest is copied. The handler
    /// disarms on the TXE entry that finds the ring empty - the one after
    /// it hands DATAR that byte, TXE rising again as the byte moves to the
    /// shift register - and the first byte was the whole ring, so the
    /// rest, once published, is armed again: one more CTLR1
    /// read-modify-write, idempotent, two a run at most. WITH AN ENGINE
    /// the run is copied whole and the engine pumped once, a block being
    /// what it starts on.
    static uint32_t write_bulk(std::span<const uint8_t> src) {
        uint32_t queued = 0;
        if constexpr (!has_tx_engine) {
            // write_byte()'s two steps, spelled here so the run's path
            // holds no call: a full ring is its refusal, nothing to arm.
            if (src.empty() || !m_tx.push(src[0])) {
                return 0;
            }
            arm_txe();
            queued = 1;
        }
        while (queued < src.size()) {
            const auto room = m_tx.write_span();
            if (room.empty()) {
                break;
            }
            const uint32_t want = static_cast<uint32_t>(src.size()) - queued;
            const uint32_t take = want < room.size() ? want : static_cast<uint32_t>(room.size());
            // Two pointers and no index, and the test at the bottom: a
            // load, a store, two steps and one branch a byte, where an index
            // re-adds both bases every byte. `take` is at least one here -
            // the room is not empty and the run is not done.
            const uint8_t* from = src.data() + queued;
            uint8_t* to = room.data();
            if (take >= copy_threshold &&
                ((reinterpret_cast<uintptr_t>(to) ^ reinterpret_cast<uintptr_t>(from)) & 3u) == 0u) {
                (void)memcpy(to, from, take);
            } else {
                uint8_t* const end = to + take;
                do {
                    *to++ = *from++;
                } while (to != end);
            }
            m_tx.publish(static_cast<typename decltype(m_tx)::index_t>(take));
            queued += take;
        }
        if constexpr (has_tx_engine) {
            pump_tx();
        } else if (queued > 1u) {
            // The rest, behind a first byte the handler may have disarmed on.
            arm_txe();
        }
        return queued;
    }

    /// The received bytes IN PLACE: the contiguous run ready to be read,
    /// never wrapping - the receive ring's consumer half under the ring's
    /// own names. With consume() it is util/stream.hpp's SpanSource,
    /// which SerialPort drains a run at a time.
    ///
    /// With a receive engine the run is the CHANNEL'S ring, read where it
    /// writes: its bytes are the stream as long as the consumer is less than
    /// a lap behind, and consume() says whether they were.
    static std::span<const uint8_t> read_span() {
        if constexpr (has_rx_engine) {
            return look();
        } else {
            return m_rx.read_span();
        }
    }

    /// Release the first `count` bytes of read_span(), oldest first, clamped
    /// to what is queued. With a receive engine it answers whether the run
    /// was intact when it was read - false when the channel wrote over it
    /// while it was held, which the ring counts in rx_overruns() and skips.
    static auto consume(uint32_t count) {
        if constexpr (has_rx_engine) {
            return m_rx.consume(count);
        } else {
            constexpr uint32_t most = decltype(m_rx)::capacity();
            m_rx.consume(static_cast<typename decltype(m_rx)::index_t>(count < most ? count : most));
        }
    }

    /// THE WIRE IS IDLE: nothing queued, no block in flight and the last
    /// frame's stop bit out (STATR.TC, 18.10.1). A transmit block clears TC
    /// as it starts, because the channel's writes of DATAR run no part of
    /// TC's software clear and the flag would answer from the frame before
    /// the block. What a program waits on before a reset or a clock change.
    static bool tx_idle() {
        if constexpr (has_tx_engine) {
            if (TxEngine::busy()) {
                return false;
            }
        }
        return m_tx.empty() && (regs().STATR & usart_tc) != 0u;
    }

    // ---- introspection ----------------------------------------------------

    static auto rx_pending() { return m_rx.count(); }

    /// The gaps in the stream the receive ring hands out, since the
    /// program started, NEVER CLEARED - util/stream.hpp's SkippingSource,
    /// the epoch util/serial_port.hpp compares at every run. Without an
    /// engine: the SkipRing's skips (modulo 2^8), one at the consumer's
    /// look after a byte the handler dropped for a flag or a full ring, or
    /// an overrun - the look that discards what the ring held. With one: the view's skips (a lap missed, a held
    /// run refused) plus m_rx_lost - an overrun, a restart after a
    /// transfer error (a framed frame is stored: the channel moves it) -
    /// counted when the vector or the look sees them, so the line torn is
    /// the one begun when the consumer next looks.
    static uint32_t rx_skips() {
        if constexpr (has_rx_engine) {
            return m_rx.skips() + m_rx_lost;
        } else {
            return m_rx.skips();
        }
    }

    static uint32_t baud() { return m_baud; }
    static uint32_t actual_baud(uint32_t pclk) { return Resource::actual_baud(pclk); }

    /// Bytes the receive ring lost to a consumer that did not keep up:
    /// without an engine, a byte refused by a full ring; with one, a lap of
    /// the channel's ring written over unread bytes, or a run written over
    /// while it was held - each one a skip (util/ring.hpp's
    /// HardwareRing::overruns()), saturating here.
    static uint16_t rx_overruns() {
        if constexpr (has_rx_engine) {
            const uint32_t n = m_rx.overruns();
            return n > 0xFFFFu ? uint16_t{0xFFFF} : static_cast<uint16_t>(n);
        } else {
            return m_rx_overruns;
        }
    }
    static uint16_t hw_overruns() { return m_hw_overruns; }
    static uint16_t frame_errors() { return m_frame_errors; }
    static uint16_t noise_errors() { return m_noise_errors; }
    static uint16_t parity_errors() { return m_parity_errors; }

    static void clear_errors() {
        if constexpr (has_rx_engine) {
            m_rx.clear_overruns();
        }
        m_rx_overruns = 0;
        m_hw_overruns = 0;
        m_frame_errors = 0;
        m_noise_errors = 0;
        m_parity_errors = 0;
        m_dma_faults = 0;
    }

    /// The divisor this bus clock and baud ask for, and the two
    /// questions a program asks before it moves a rate.
    static constexpr uint32_t divisor_for(uint32_t pclk, uint32_t baud) {
        return usart_divisor(pclk, baud);
    }
    static constexpr uint32_t min_hz_for(uint32_t baud) { return usart_min_hz(baud); }
    static constexpr bool can_baud(uint32_t pclk, uint32_t baud) {
        return usart_divisor_valid(usart_divisor(pclk, baud));
    }

    /// Follow a clock that changed rate. The argument is HCLK - what a
    /// dynamic clock hands every user - and this port derives its own
    /// bus rate from it, because the two peripheral buses do not divide
    /// the same number.
    ///
    /// It is called BEFORE the rate moves, with the old clock still
    /// running, so the queue is drained here: a byte still in the shift
    /// register when SYSCLK changes goes out at neither baud rate. The
    /// drain is bounded - a port whose line is held off must not hang
    /// the switch - and the divisor is written whatever the drain found.
    static void rebase(uint32_t hz) {
        constexpr uint32_t drain_spins = 2'000'000UL;
        uint32_t spins = drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
            if constexpr (has_tx_engine) {
                pump_tx();
            }
        }
        spins = drain_spins;
        while ((regs().STATR & usart_tc) == 0u && spins-- != 0u) {
        }
        const uint32_t brr = usart_divisor(usart_bus_hz_at<instance>(hz), m_baud);
        if (usart_divisor_valid(brr)) {
            regs().BRR = static_cast<uint16_t>(brr);
        }
    }

    /// Move the LINK to a different bit rate, the clock staying put -
    /// the mirror of rebase(), and `hz` is HCLK exactly as rebase()
    /// takes it, this port deriving its own bus rate from it. False,
    /// and nothing written, when the rate is unreachable.
    static bool set_baud(uint32_t hz, uint32_t baud) {
        const uint32_t brr = usart_divisor(usart_bus_hz_at<instance>(hz), baud);
        if (!usart_divisor_valid(brr)) {
            return false;
        }
        constexpr uint32_t drain_spins = 2'000'000UL;
        uint32_t spins = drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
            if constexpr (has_tx_engine) {
                pump_tx();
            }
        }
        spins = drain_spins;
        while ((regs().STATR & usart_tc) == 0u && spins-- != 0u) {
        }
        regs().BRR = static_cast<uint16_t>(brr);
        m_baud = baud;
        return true;
    }

    /// Stop the port and park its pads: the vector off, the engines
    /// stopped, UE clear, the block's reset pulsed, the bus clock closed,
    /// the pins released.
    ///
    /// THE RESET LINE BEFORE THE GATE. A DMA request this block has raised
    /// is HELD until the channel acknowledges it or the block is reset -
    /// clearing DMAT, DMAR or UE does not withdraw it, and a gated clock
    /// freezes it on the channel's OR, where the next owner's first enable
    /// moves one item on it and the channel then stalls (measured on both
    /// series, docs/ch32vx03/dma.md, "A released requester"). A transmit
    /// request stands from init() on with an engine, DMAT meeting an idle
    /// transmitter, so every engined port leaves one; the pulse takes it
    /// with CTLR3, two read-modify-writes of the bus's reset register.
    static void release() {
        Pfic::disable(usart_irq_for(instance));
        if constexpr (has_tx_engine) {
            TxEngine::stop();
        }
        if constexpr (has_rx_engine) {
            RxEngine::stop();
        }
        Resource::enable(false);
        Resource::reset();
        Resource::bus_clock(false);
        Tx::release();
        if constexpr (!opts.half_duplex) {
            Rx::release();
        }
        if constexpr (opts.rts) {
            Rts::release();
        }
        if constexpr (opts.cts) {
            Cts::release();
        }
    }

private:
    /// The flow-control pads, formed only where the options ask for them
    /// - a column that has no such pad would not make a Pin at all.
    using Cts = Pin<opts.cts ? column.cts.port : pads.tx.port,
                    opts.cts ? column.cts.pin : pads.tx.pin>;
    using Rts = Pin<opts.rts ? column.rts.port : pads.tx.port,
                    opts.rts ? column.rts.pin : pads.tx.pin>;

    /// Start the next contiguous run of the TX ring on the engine, if it
    /// is idle and there is one.
    ///
    /// THE MASK COVERS THE CLAIM AND NOTHING ELSE. The completion handler
    /// pumps too, so the engine's busy flag is tested and set under the
    /// guard - a handful of cycles. The run is read and the channel
    /// programmed unmasked: a claimed engine has no block in flight, so no
    /// completion can come to consume the ring under the reader or start a
    /// block of its own. A run that turns out empty gives the claim back.
    static void pump_tx() {
        if constexpr (has_tx_engine) {
            {
                typename P::CriticalSection cs;
                if (!TxEngine::claim()) {
                    return;
                }
            }
            const auto run = m_tx.read_span();
            if (run.empty()) {
                TxEngine::unclaim();
                return;
            }
            Resource::clear_flags(usart_tc);   // TC from the frame before this block, written 0
            (void)TxEngine::launch(run);   // a refusal gives the claim back itself
        }
    }

    /// TXEIE armed from the thread. With a receive engine and no transmit
    /// one the vector rewrites CTLR1's receive enables, so the thread's
    /// read-modify-write takes the guard - the decision, three
    /// instructions - or it could store back an enable the vector had just
    /// turned; every other shape stores bare.
    [[gnu::always_inline]] static void arm_txe() {
        if constexpr (has_rx_engine) {
            typename P::CriticalSection cs;
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_txeie);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_txeie);
        }
    }

    /// The consumer's look at the receive ring under an engine: a channel
    /// a transfer error stopped started again first, and a look that finds
    /// the ring empty RE-OPENS THE EDGE and looks again, so a byte landing
    /// between the two is in this run or raises the edge.
    static std::span<const uint8_t> look() {
        restart_if_stopped();
        std::span<const uint8_t> run = m_rx.read_span();
        if (run.empty()) {
            m_rx_drained = true;
            std::atomic_signal_fence(std::memory_order_seq_cst);
            run = m_rx.read_span();
        }
        return run;
    }

    /// A ring never stops on its own: a channel that is not running was
    /// stopped by a transfer error, and is started again from the
    /// storage's first element with the view - the consumer's context.
    ///
    /// A ring the vector GAVE UP on (rx_stalled()) is restarted the same
    /// way: its channel still enabled over a frozen handshake, the re-arm's
    /// first store takes EN down.
    ///
    /// A RESTARTED RING WAITS FOR THE END: the vector's state is put back
    /// with the channel - the wait for a frame it was in counted the old
    /// lap's position, and a channel the vector gave up on left every
    /// receive enable down. One read-modify-write of each control register
    /// under the guard, the transmitter's TXEIE sharing CTLR1 with them;
    /// the rare path only.
    static void restart_if_stopped() {
        if constexpr (has_rx_engine) {
            if (RxEngine::idle() || m_rx_given_up) {
                m_rx_given_up = false;
                m_rx.clear();
                m_rx_lost = m_rx_lost + 1u;   // the unread bytes went with the stopped lap
                (void)RxEngine::start();
                typename P::CriticalSection cs;
                m_rx_waiting = false;
                regs().CTLR1 = static_cast<uint16_t>((regs().CTLR1 & ~usart_rxneie) |
                                                     usart_idleie | usart_peie);
                regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 | usart_eie);
            }
        }
    }

    /// THE EDGE'S GATE, for the vectors: true once per idle-to-busy
    /// transition of the consumer. Each caller has seen the channel write
    /// since the consumer was last told, so no look at the view is needed.
    [[gnu::always_inline]] static bool told() {
        if (m_rx_drained) {
            m_rx_drained = false;
            return true;
        }
        return false;
    }

    /// WAITING FOR THE END (the file header): an idle line or an error,
    /// from the entry's one status read. An entry showing neither and no
    /// transmit condition to explain it is an idle line a status read
    /// elsewhere armed and a frame cleared before this vector ran: served
    /// as the idle it was. CNTR is read before RXNEIE is armed and after:
    /// a frame the channel took between raises no entry, so it is served
    /// here.
    [[gnu::always_inline]] static bool rx_end_entry(uint16_t status) {
        if ((status & (usart_idle | usart_fe | usart_ne | usart_pe | usart_ore)) == 0u) {
            if constexpr (!has_tx_engine) {
                if ((status & usart_txe) != 0u && (regs().CTLR1 & usart_txeie) != 0u) {
                    return false;   // the transmitter's entry
                }
            }
        }
        if ((status & usart_fe) != 0u) { bump(m_frame_errors); }
        if ((status & usart_ne) != 0u) { bump(m_noise_errors); }
        if ((status & usart_pe) != 0u) { bump(m_parity_errors); }
        if ((status & usart_ore) != 0u) {
            bump(m_hw_overruns);
            m_rx_lost = m_rx_lost + 1u;
        }
        m_rx_waiting = true;
        const uint16_t at = static_cast<uint16_t>(RxEngine::remaining());
        m_rx_at = at;
        m_rx_unmoved = 0;
        regs().CTLR1 = static_cast<uint16_t>((regs().CTLR1 & ~(usart_idleie | usart_peie)) | usart_rxneie);
        regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 & ~usart_eie);
        const bool edge = told();
        if (static_cast<uint16_t>(RxEngine::remaining()) == at) {
            return edge;
        }
        const bool first = rx_frame_entry();
        return edge || first;
    }

    /// WAITING FOR A FRAME: the channel has moved since the wait began, so
    /// its read finished the clear - back to waiting for the end, and the
    /// edge, which is the only one a burst of one frame gets. NO STATR
    /// READ: it would arm the clear the next frame's read performs.
    ///
    /// An entry with the count unmoved is the transmitter's, or a frame
    /// the channel has not taken yet - or one it NEVER takes: RXNE is a
    /// level, and over a channel that does not serve its request (a
    /// transfer error stopped it, or another peripheral's held request
    /// froze its handshake - docs/ch32vx03/dma.md, "A released requester")
    /// this vector would be re-entered for ever, the thread starved. So
    /// the entries that find the count unmoved with nothing else armed to
    /// explain them are COUNTED, and the rx_stall_entries-th gives the
    /// channel up (rx_stalled()); the count then stays there, the state
    /// parked until the consumer's look restarts the ring. Two compares
    /// and an add an entry, on this path alone.
    [[gnu::always_inline]] static bool rx_frame_entry() {
        if (static_cast<uint16_t>(RxEngine::remaining()) == m_rx_at) {
            if constexpr (!has_tx_engine) {
                if ((regs().CTLR1 & usart_txeie) != 0u) {
                    return false;   // the transmitter's entry, perhaps
                }
            }
            if (m_rx_unmoved == rx_stall_entries) {
                return false;   // given up already: the consumer's look restarts it
            }
            m_rx_unmoved = static_cast<uint8_t>(m_rx_unmoved + 1u);
            if (m_rx_unmoved != rx_stall_entries) {
                return false;   // the frame not yet taken
            }
            return rx_stalled();
        }
        m_rx_waiting = false;
        regs().CTLR1 = static_cast<uint16_t>((regs().CTLR1 & ~usart_rxneie) | usart_idleie | usart_peie);
        regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 | usart_eie);
        return told();
    }

    /// How many entries of the wait for a frame may find the channel's
    /// count unmoved before the channel is given up. A live channel takes
    /// RXNE's byte within its arbitration - a few bus cycles behind any
    /// other channel's item - where one entry of this vector is tens of
    /// cycles, so a frame not yet taken is one or two entries; 255 is
    /// thousands of cycles of a channel that moved nothing. A channel a
    /// higher-priority memory-to-memory block starves for that long is
    /// given up too, and restarted by the consumer's next look.
    static constexpr uint8_t rx_stall_entries = 255;

    /// THE CHANNEL GIVEN UP: the fault counted, the ring marked for the
    /// consumer's look to restart (restart_if_stopped(), which stops the
    /// channel and starts it again from the storage's first element, as
    /// after a transfer error) and the edge reported so the consumer comes.
    /// RXNEIE goes down, the one enable of this state (IDLEIE, PEIE and EIE
    /// are already down), so the standing RXNE re-enters nothing in
    /// between; the state stays the wait for a frame with its count at the
    /// bound, parked. The channel itself is left to the thread: stopping it
    /// is the ledger of working bus masters' business too, a guarded call
    /// a leaf vector does not make.
    [[gnu::always_inline]] static bool rx_stalled() {
        bump(m_dma_faults);
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_rxneie);
        m_rx_given_up = true;
        m_rx_drained = false;
        return true;
    }

    /// Saturating: a counter that wraps would report a healthy port.
    /// ALWAYS INLINE because it is the one call isr() and dma_isr() would
    /// otherwise make, on their error paths: a call anywhere in a handler
    /// makes it a non-leaf function, and a non-leaf handler built for the
    /// V4F's ilp32f saves all twenty caller-saved f-registers in its own
    /// prologue, every interrupt, the hardware prologue saving integer
    /// registers only (pfic.hpp). Inline, the vector is a leaf and saves
    /// none of them.
    [[gnu::always_inline]] static void bump(uint16_t& counter) {
        if (counter != 0xFFFFu) {
            ++counter;
        }
    }

    // The receive ring: Ring without a receive engine, the view over the
    // channel's ring with one (UartRxRing above).
    using RxRing = UartRxRing<RxEngine::present, Uart, rx_size, RxEngine, P>;
    static inline typename RxRing::type m_rx{};
    static inline Ring<uint8_t, tx_size, P> m_tx{};
    static inline uint32_t m_baud = 0;
    /// Whether the consumer has found the receive ring empty since the edge
    /// was last reported - by a vector or by harvest() - what makes the
    /// answer an EDGE over a ring the channel fills with no step of ours in
    /// between. Set by the consumer's look, cleared by whoever reports;
    /// touched only with a receive engine.
    static inline volatile bool m_rx_drained = true;
    /// The receive engine's vector state (the file header's two states):
    /// waiting for a frame, and CNTR when that wait began.
    static inline volatile bool m_rx_waiting = false;
    static inline volatile uint16_t m_rx_at = 0;
    /// The entries of that wait that found the count unmoved
    /// (rx_frame_entry()); the vector's alone.
    static inline uint8_t m_rx_unmoved = 0;
    /// The vector gave the channel up: set there, cleared by the
    /// consumer's restart.
    static inline volatile bool m_rx_given_up = false;

    /// write_bulk()'s copy into the ring: the runtime's memcpy for a run of
    /// at least this many bytes whose source and ring slot share their
    /// alignment modulo the word (rt/rt.cpp's word path), the byte loop
    /// otherwise, where memcpy would run the same loop behind a call.
    static constexpr uint32_t copy_threshold = 16;
    static inline uint16_t m_rx_overruns = 0;
    static inline uint16_t m_hw_overruns = 0;
    static inline uint16_t m_frame_errors = 0;
    static inline uint16_t m_noise_errors = 0;
    static inline uint16_t m_parity_errors = 0;
    static inline uint16_t m_dma_faults = 0;
    /// Under a receive engine, the bytes lost beside the view's skips (an
    /// overrun, a restart), never cleared: rx_skips()'s second term.
    static inline volatile uint32_t m_rx_lost = 0;
};

} // namespace brio
