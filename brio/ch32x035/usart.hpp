/*
 * usart.hpp
 *
 * The CH32X035's USART (RM ch. 14) as a serial port for a program: the
 * console a program prints to, a line-oriented link, a byte transport to
 * another chip, or one of the modes the peripheral offers (LIN, IrDA,
 * the smartcard, a synchronous clock, half duplex on one wire). The
 * header has the two layers every brio serial driver has
 * (docs/design/serial.md is the stack above them):
 *
 *  Usart<n>              the resource: which instance, where its
 *                        registers are, its bus gate and its reset, its
 *                        vector, its DMA channels, and the whole register
 *                        description of chapter 14 as verbs - the frame,
 *                        the divisor, mute mode with both wakes, LIN's
 *                        break, single-wire half duplex, IrDA, the
 *                        smartcard, the synchronous clock, the
 *                        flow-control pair, the DMA requests, every flag
 *                        and every interrupt enable;
 *  Uart<n, P, ...>       the task: the interrupt-driven byte transport with
 *                        two rings and one interrupt body - every
 *                        console's personality, a ByteTransport for
 *                        print() and SerialPort, the same surface - and
 *                        the same template parameters - as the
 *                        CH32V203's (brio/ch32vx03/usart.hpp).
 *
 * The smallest use, the console on USART2's default column, is written
 * out at Uart below: `using Serial = brio::Uart<2, P>`, the instance's
 * vector bound to Serial::isr() by the program under BRIO_CH32_INTERRUPT,
 * `Serial::init(clock, 115200)` after the clock task, then print().
 *
 * Who calls what, and from where. init() and release() are setup verbs:
 * main(), after the clock task and before interrupts are enabled
 * globally; init() waits for nothing. isr() is the interrupt body: the
 * program's handler for the instance's vector calls it and nothing else
 * does; it runs to completion like every interrupt handler in brio, and
 * masks nothing. dma_isr() is the engines' channels' body the same way.
 * The transmit verbs (write_byte(), write_bulk(), and print() over them)
 * and the receive verbs (read_byte(), read_bulk(), read_span() with
 * consume()) belong to the main context - setup, or the handler of an
 * active object, a brio object with an event queue the kernel serves one
 * event at a time - and never to an interrupt handler: each ring has one
 * producer and one consumer, and the interrupt body is the other end of
 * both. Arming the transmit interrupt from the thread is a
 * read-modify-write of CTLR1, bare on every shape but the one in which
 * the vector rewrites that register too (a receive engine without a
 * transmit one), where it takes the platform's guard for three
 * instructions; claiming the transmit engine takes the guard for the
 * claim alone. Nothing in the task waits, except rebase() and set_baud(),
 * main-context verbs that spin until the queued bytes have left the wire,
 * each wait bounded so a wedged line cannot hang them. Storage: the rings
 * are the task's own static arrays; write_bulk() copies the caller's
 * span, which is free again when it returns; read_span() hands out the
 * ring's own bytes, valid until consume() releases them. The resource's
 * configuring verbs are setup verbs with no guard of their own; its flag
 * and data verbs are single loads and stores, legal from any context.
 *
 * Four instances, one clock. USART1 answers on PB2 and USART2..USART4 on
 * PB1, but this series has no peripheral-bus prescaler: every one of them
 * counts its divisor in HCLK (14.3), so the transport's rate is the
 * clock's and nothing else. Which instances a part offers is the part's
 * table (device::has_usart), and on the 20-pin packages that is not the
 * first n of them: the CH32X035F8U6 offers USART2, USART3 and USART4 and
 * no USART1, because no USART1 column has both its TX and its RX pads on
 * that package.
 *
 * Every instance is a full USART. Chapter 14 counts four universal
 * synchronous asynchronous transceivers and the block diagram gives each
 * one RX, TX, CTS, RTS and CK - there is no UART here, so the smartcard,
 * the synchronous clock and the flow-control pair are verbs of every
 * instance. What a column may lack is a pad: USART3's code 3 has no CK.
 *
 * The pads are the remap columns' (afio.hpp): the `remap` template
 * parameter names a column and the TX, RX, CK, CTS and RTS pads follow
 * from the table that programs AFIO_PCFR1. A code of 0 writes nothing at
 * all - the reset column is already there. Two rules come from the
 * package: a column is a transport's only where its TX pad may be driven
 * and its RX pad is bonded - a TX on a pad that shares its package pin
 * with another (pin.hpp) is refused, which is what closes USART4's
 * PC16/PC17 columns on every part but the CH32X035F8U6 - and a column
 * whose TX or RX lands on the debug port's own pads (USART3's code 1, and
 * USART4's codes 4 and 6 for RX) is refused by init() while the two-wire
 * port is still the probe's.
 *
 * Two rings and two flags. Bytes leave through a TX ring drained by the
 * TXE interrupt (write_byte() arms TXEIE after a byte, write_bulk() after
 * a run's first byte and again after the rest; the ISR disarms it when
 * the ring runs dry, because TXE stands for ever while the transmitter
 * is idle and would re-enter the handler without end), and arrive into
 * an RX ring filled by RXNE, taken a byte at a time by read_byte() or a
 * run at a time, in place, by read_span() and consume(). isr() returns
  * the "RX went non-empty" edge that util/serial_port.hpp posts on - one
 * event per idle-to-busy transition, not one per byte.
 *
 * The baud divisor is the whole register. BRR counts HCLK periods per bit
 * in sixteenths (14.3, 14.10.3: a 12-bit mantissa and a 4-bit fraction),
 * so the value to store is simply hclk / baud - no field arithmetic - and
 * actual_baud() inverts it. Below 16 the generator has nothing to divide
 * by: refused.
 *
 * Errors are read then cleared. Parity, framing, noise, overrun and the
 * idle line stand in STATR and go away when STATR is read and then DATAR,
 * in that order (14.10.1); RXNE, TC, LBD and CTS also clear by writing a
 * zero over them. The handler therefore reads the status once, decides
 * from that copy, and lets the DATAR read do the clearing. A byte that
 * arrived with an error is dropped rather than pushed, and the counter
 * says it happened.
 *
 * The modes exclude each other the way 14.4 .. 14.7 say, and the
 * exclusions are not symmetrical: the synchronous clock wants SCEN, HDSEL
 * and IREN clear; half duplex wants SCEN, CLKEN and IREN clear; the
 * smartcard wants LINEN, HDSEL and IREN clear and keeps CLKEN, which is
 * where its card clock comes from; IrDA wants LINEN, STOP, CLKEN, SCEN and
 * HDSEL clear. Each verb refuses instead of storing a combination the
 * chapter declares undefined.
 *
 * Half duplex drives push-pull. 14.5 asks for the TX pad "in output mode
 * plus pull" and table 8-3 for an alternate push-pull output with an
 * external pull - the only alternate output this series has (pin.hpp).
 * Two talkers on one wire can therefore fight; the manual leaves avoiding
 * that to the program.
 *
 * Two optional DMA engine slots, one for each of the instance's two
 * requests (14.8, DMAT and DMAR): a transmit engine drains the TX ring by
 * contiguous runs (the ring's read_span, consumed by exactly what the
 * block carried when it completes - the run started outside the mask, the
 * engine claimed under it), and a receive engine runs in its circular
 * shape over the whole receive storage (ch32x035/dma.hpp): the channel
 * writes it lap after lap and is never re-armed, its count is the ring's
 * producer index, and the receive ring is the consumer half of that,
 * util/ring.hpp's HardwareRing - so nothing is lost between runs, there
 * being none, and a lap the consumer did not keep up with is counted and
 * skipped (rx_overruns()). Without an engine the slot is the NoDmaEngine
 * tag and every engine branch is compiled out. An engine is refused on any
 * channel but the instance's own (dma_engine.hpp's table: USART2 transmits
 * on 7 and receives on 6, USART4 on 1 and 8). The ring outranks the
 * copies: DATAR holds one frame, so the receive engine arms at very_high
 * (UartOptions::rx_priority, the one knob) and the transmit engine at
 * high, the rule of docs/design/dma.md.
 *
 * Where the hot paths run from: the port's placement, its last template
 * parameter (util/placement.hpp) - inline in the vector that binds them by
 * default, one instance each in SRAM under InRam. Two paths take it:
 *  - The transmit engine's block boundary. A block's completion releases
 *    its bytes and starts the next run while the shifter still holds about
 *    a frame and a half. The DMA vector's whole body is the placement's -
 *    the flags' read, the channels' service and the restart, the ring's
 *    consume and read_span and the engine's launch inlined, no call left
 *    on the path - and from the flash, behind the two wait states, it
 *    outlasts that window above 1.5 Mbaud and the line idles at every
 *    boundary; under InRam, behind a vector in the flash that keeps its
 *    entry and one call, it is gapless to 3 Mbaud with some 15 to 20 cycles
 *    to spare, where the restart alone in SRAM behind the flags' read and
 *    service in the flash came out a few cycles past it
 *    (docs/ch32x035/usart.md, letter s).
 *  - The per-byte interrupt transport's isr(), which calls nothing: under
 *    InRam, behind a vector in the flash, it takes a gapless run in to
 *    2.4 Mbaud where the flash takes 2; a program that places the VECTOR
 *    itself (`.fast_text`) runs the inline body from SRAM with it, no call
 *    at all, and takes 3 Mbaud (docs/ch32x035/platform.md, "A handler in
 *    SRAM").
 *
 * Why the ring and not the vendor's shape. WCH's USART_Idle_Recv example
 * receives into two halves it swaps from the IDLE vector and the channel's
 * completion - the channel disabled, its count and address rewritten,
 * re-enabled, a window in which a frame has no channel - and copies each
 * half into a ring of its own, a second pass over every byte. The circular
 * channel has no window and no copy: the consumer reads where the channel
 * wrote.
 *
 * Under the receive engine the CPU never reads DATAR, and the clear is
 * this silicon's - measured on the CH32X035F8U6 with frames from a second
 * chip into USART4's receiver and the channel reading DATAR
 * (docs/ch32x035/usart.md): a read of STATR arms the clear (14.10.1:
 * "reading the status register and then the data register"), and the next
 * read of DATAR - the channel's - performs it (measured on PE and on IDLE);
 * a flag with no status read since the last DATAR read stands. The clear of
 * IDLE also forgets the idle its own frame armed: one frame after an idle
 * that was read raises no IDLE of its own, two frames do - and a status
 * read that found IDLE down arms no clear of it, so one frame after THAT
 * read raises IDLE (measured, docs/ch32x035/usart.md). The CH32V006's and
 * the CH32V203's USARTs answer the same on PE, and the CH32V006 measured
 * the rest of the rule - every error flag standing at that read cleared -
 * which this vector is written to.
 *
 * The burst edge comes from a vector. With the receive engine the USART's
 * vector runs two states over that clear. Waiting for the end: IDLEIE,
 * PEIE and EIE armed; an idle line or an error enters, its status read
 * counts the errors and arms the clear, the edge is reported, and the
  * vector turns to waiting for a frame - those three disarmed, RXNEIE
 * armed (the channel still takes the byte; the interrupt only says one
 * came). That entry reads no STATR, which after the channel's read would
 * arm the clear the next frame's own read performs, but CNTR alone: a
 * moved count turns the vector back, reporting the edge again - the only
 * edge a burst of one frame gets. A port with no transmit engine, whose
 * interrupt transmitter shares the vector, asks CTLR1 whether TXEIE stands
 * and reads STATR for TXE only then. The channel's half and full marks report
 * it from dma_isr(), so a stream with no silence is told twice a lap. An
 * overrun with nothing to take - ORE left up with RXNE down by a channel
 * that was starved, and no frame after it - would re-enter that wait back
 * to back (RXNEIE rides ORE, 14.10.1): its second entry with the count
 * unmoved reads STATR once and, finding ORE without RXNE, reads DATAR - the
 * clear the wait's own status read armed - and turns back to the wait for
 * the end (rx_overrun_probe). Two interrupts a burst, none a byte; the
 * edge gated once per idle-to-busy transition of the consumer, re-opened
 * when its look finds the ring empty. A channel that never takes the frame
 * - stopped by a transfer error, or frozen by a request another peripheral
 * left on its OR (docs/design/dma.md's release contract) - leaves RXNE
 * standing under RXNEIE, and the wait would re-enter for ever: its entries
 * that find the count unmoved are counted, and the 255th gives the channel
 * up - a DMA fault counted, RXNEIE down, the edge reported - and the
 * consumer's next look restarts the ring from its first element and arms
 * the wait for the end, as after a transfer error. Both guards are measured
 * here, each staged by a block above the ring (docs/ch32x035/usart.md).
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <atomic>
#include <span>

#include "ch32x035/afio.hpp"
#include "ch32x035/clock.hpp"
#include "ch32x035/device.hpp"
#include "ch32x035/dma_engine.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "util/clock.hpp"
#include "util/placement.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"

namespace brio {

// =============================================================================
// The chapter's vocabulary
// =============================================================================

/// Data bits of a frame. The register speaks in word length (M: 8 or 9
/// bits including the parity bit when there is one), so seven data bits
/// exist only with a parity bit and nine only without.
enum class UartBits : uint8_t { seven = 7, eight = 8, nine = 9 };

enum class UartParity : uint8_t { none, even, odd };

/// CTLR2.STOP, all four codes; the half and one-and-a-half stops are the
/// chapter's own (14.10.5).
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

/// Mute mode (14.10.4's RWU and WAKE, 14.10.5's ADD): the receiver asleep
/// until the line goes idle, or until a frame whose MSB is set carries
/// this node's 4-bit address.
enum class MuteWake : uint8_t { idle_line, address_mark };

struct MuteConfig {
    MuteWake wake = MuteWake::idle_line;
    uint8_t address = 0;   ///< 0..15, for address_mark
};

constexpr bool mute_valid(const MuteConfig& c) { return c.address <= 15u; }

/// LIN mode (14.10.5's LINEN): the break SBK sends, and its detection at
/// ten or eleven bits with its own flag and interrupt.
struct LinConfig {
    bool break_11bit = false;   ///< LBDL: 11-bit detection instead of 10
    bool break_interrupt = false;
};

/// IrDA (14.7, 14.10.7): the SIR encoder on TX and decoder on RX, normal
/// mode at the bit rate or low-power mode on the prescaled clock. GPR.PSC
/// is all eight bits in low-power mode and must be 1 in normal mode; a
/// prescaler of 0 "indicates a hold" and is refused.
struct IrdaConfig {
    bool low_power = false;
    uint8_t prescaler = 1;   ///< GPR.PSC
};

constexpr bool irda_valid(const IrdaConfig& c) {
    return c.prescaler != 0u && (c.low_power || c.prescaler == 1u);
}

/// The synchronous mode's three choices (14.4): the clock's idle level
/// (CPOL), the capture edge (CPHA) and whether the last data bit gets a
/// clock pulse too (LBCL). The sense of LBCL is this manual's, and it is
/// the opposite of the F1 family's: 14.10.5 reads "1: The clock pulse for
/// the last bit of data is not output from CK; 0: ... will be output".
/// `last_bit_clock` names the bit and not a promise.
struct UsartSyncConfig {
    bool clock_idle_high = false;      ///< CPOL
    bool capture_second_edge = false;  ///< CPHA
    bool last_bit_clock = false;       ///< LBCL, as the register spells it
};

/// The smartcard (14.6): ISO 7816-3 on a single wire with the card's clock
/// on CK - a NACK on a parity error, the guard time in bit times (GPR.GT)
/// and the clock prescaler (GPR.PSC, the low FIVE bits, the source divided
/// by twice the value, so 31 is the deepest division and 0 is reserved).
struct SmartcardConfig {
    bool nack = true;
    uint8_t guard_time = 0;
    uint8_t clock_prescaler = 1;
};

constexpr bool smartcard_valid(const SmartcardConfig& c) {
    return c.clock_prescaler != 0u && c.clock_prescaler <= 31u;
}

/// The divisor this clock and baud ask for: hclk/baud in sixteenths,
/// rounded to nearest so the error is halved. Below 16 there is no whole
/// clock period per sixteenth of a bit and the generator has nothing to
/// divide by - init() refuses such a value.
constexpr uint32_t usart_divisor(uint32_t hclk, uint32_t baud) {
    return baud == 0u ? 0u : (hclk + baud / 2u) / baud;
}

constexpr bool usart_divisor_valid(uint32_t brr) { return brr >= 16u && brr <= 0xFFFFu; }

/// The rate a divisor really gives at this clock, and the smallest clock
/// that can produce a rate at all (sixteen periods a bit).
constexpr uint32_t usart_actual_baud(uint32_t hclk, uint32_t brr) {
    return brr == 0u ? 0u : hclk / brr;
}
constexpr uint32_t usart_min_hz(uint32_t baud) { return baud * 16u; }

/// Which bus an instance answers on, and therefore which gate opens it:
/// USART1 alone is PB2's. The divisor counts HCLK on both.
constexpr Bus usart_bus_for(uint8_t n) { return n == 1 ? Bus::pb2 : Bus::pb1; }

/// The gate of an instance (RCC_APB2PCENR bit 14, RCC_APB1PCENR bits
/// 17..19 - 3.4.6, 3.4.7), the same bit in the matching reset register.
constexpr uint32_t usart_gate_for(uint8_t n) {
    return n == 1 ? rcc_pb2_usart1 :
           n == 2 ? rcc_pb1_usart2 :
           n == 3 ? rcc_pb1_usart3 :
           n == 4 ? rcc_pb1_usart4 : 0;
}

/// The instance's vector (RM table 7-1).
constexpr Irq usart_irq_for(uint8_t n) {
    return n == 1 ? Irq::usart1 : n == 2 ? Irq::usart2 : n == 3 ? Irq::usart3 : Irq::usart4;
}

/// The AFIO field an instance's column is selected by.
constexpr Remap usart_remap_of(uint8_t n) { return afio_usart_remap(n); }

/// The whole column - TX, RX, CK, CTS, RTS - under a remap code. ONE
/// table answers, afio.hpp's, which is also what writes the register.
constexpr UsartPadSet usart_column_for(uint8_t n, uint8_t code = 0) {
    return afio_usart_pads(n, code);
}

/// The two pads a transport claims, out of the five a column carries.
struct UsartPads {
    Pad tx;
    Pad rx;
};

constexpr UsartPads usart_pads_for(uint8_t n, uint8_t code = 0) {
    const UsartPadSet p = usart_column_for(n, code);
    return UsartPads{p.tx, p.rx};
}

/// Whether a column is a transport's on this part: the code is one of the
/// field's, its TX pad may be driven (bonded and alone on its pin) and
/// its RX pad is bonded.
constexpr bool usart_remap_valid(uint8_t n, uint8_t code) {
    const UsartPadSet p = usart_column_for(n, code);
    return afio_remap_has_code(usart_remap_of(n), code) && pad_can_drive(p.tx) && pad_bonded(p.rx);
}

/// Whether a column puts TX or RX on one of the two pads the debug port
/// owns from reset (device.hpp's debug_swdio_* and debug_swclk_*).
constexpr bool usart_column_on_debug_port(uint8_t n, uint8_t code) {
    const UsartPadSet p = usart_column_for(n, code);
    const Pad dio{debug_swdio_port, debug_swdio_pin};
    const Pad clk{debug_swclk_port, debug_swclk_pin};
    return p.tx == dio || p.tx == clk || p.rx == dio || p.rx == clk;
}

// =============================================================================
// Usart<n>: the resource
// =============================================================================

/**
 * One USART instance, register by register. Every verb stores what it
 * names and nothing else; a verb whose combination the chapter declares
 * undefined refuses (false, nothing written). The task below is written
 * on these verbs; a program that wants the chapter beyond the transport -
 * a break on a LIN line, a muted receiver, a clocked frame - reaches them
 * here.
 */
template <uint8_t n>
struct Usart {
    static_assert(n >= 1u && n <= 4u,
                  "brio Usart: this series has USART1..USART4");
    static_assert(device::has_usart(n),
                  "brio Usart: this part does not offer that instance - its package bonds no "
                  "column of it a transport could use (parts/<part>.hpp)");

    Usart() = delete;

    static constexpr uint8_t number = n;
    static constexpr Bus bus = usart_bus_for(n);
    static constexpr UsartPads pads = usart_pads_for(n);
    static constexpr Irq irq = usart_irq_for(n);
    static constexpr uint8_t dma_tx_channel = usart_dma_tx_channel(n);
    static constexpr uint8_t dma_rx_channel = usart_dma_rx_channel(n);

    /// Every instance of this series is a full USART (the file header).
    static constexpr bool is_full = true;

    static UsartRegs& regs() { return *reinterpret_cast<UsartRegs*>(usart_base_for(n)); }

    /// The peripheral's clock gate. Nothing in this file reads a register
    /// before this is on: an unclocked block answers rubbish.
    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(bus, usart_gate_for(n));
        } else {
            Rcc::disable(bus, usart_gate_for(n));
        }
    }

    /// Put the block back to its reset state, gate left as it is.
    static void reset() { (void)Rcc::reset(bus, usart_gate_for(n)); }

    /// Select the instance's column (afio.hpp). False - and nothing
    /// written - for a column this part has not got. Code 0 is the reset
    /// column and writing it is still a write, which is why the transport
    /// skips the call entirely at that code.
    static bool remap(uint8_t code) { return Afio::remap(usart_remap_of(n), code); }

    // ---- configuration ----------------------------------------------------

    /// The frame and the divisor, with the port disabled. The two registers
    /// are written whole - this is the from-scratch verb, and the modes
    /// below are what a program adds to it afterwards.
    static bool configure(const UartFormat& f, uint32_t brr) {
        if (!uart_format_valid(f) || !usart_divisor_valid(brr)) {
            return false;
        }
        regs().CTLR1 = usart_ctlr1_format(f);
        regs().CTLR2 = usart_ctlr2_stop(f.stop);
        regs().BRR = static_cast<uint16_t>(brr);
        return true;
    }

    /// The stop bits alone. Refused under IrDA, which 14.7 wants at one.
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

    static void enable(bool on) { bit(regs().CTLR1, usart_ue, on); }
    static void transmitter(bool on) { bit(regs().CTLR1, usart_te, on); }
    static void receiver(bool on) { bit(regs().CTLR1, usart_re, on); }

    // ---- the receiver's modes ---------------------------------------------

    /// Mute mode: WAKE and ADD written, RWU left to mute()/unmute() because
    /// 14.10.4's note 1 says the receiver must have taken a byte before an
    /// idle-line wake can work.
    static bool mute_mode(const MuteConfig& c) {
        if (!mute_valid(c)) {
            return false;
        }
        bit(regs().CTLR1, usart_wake, c.wake == MuteWake::address_mark);
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~usart_add_mask) | c.address);
        return true;
    }
    /// Put the receiver to sleep. 14.10.4's note 2: under an address-mark
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
    /// clock is on (14.4 .. 14.7).
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

    /// SBK: one break frame after the current one; the bit clears itself on
    /// the break frame's stop bit (14.10.4).
    static void send_break() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_sbk); }
    static bool break_pending() { return (regs().CTLR1 & usart_sbk) != 0u; }

    /// Single-wire half duplex (14.5): the TX pad alone carries both
    /// directions - an alternate push-pull output with an external pull
    /// (the file header). Refused while LIN, IrDA, the smartcard or the
    /// clock is on.
    static bool half_duplex(bool on) {
        if (on && ((regs().CTLR2 & (usart_linen | usart_clken)) != 0u ||
                   (regs().CTLR3 & (usart_iren | usart_scen)) != 0u)) {
            return false;
        }
        bit(regs().CTLR3, usart_hdsel, on);
        return true;
    }
    static bool half_duplex() { return (regs().CTLR3 & usart_hdsel) != 0u; }

    /// IrDA (14.7): refused while LIN, half duplex, the smartcard or the
    /// clock is on, with stop bits other than one, and with a prescaler
    /// the register description calls a hold.
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

    /// The smartcard (14.6). Sets the guard time and the card-clock
    /// prescaler, 1.5 stop bits, CLKEN and SCEN: the clock is the card's
    /// and belongs to the mode. What is not written is the CK pad - the
    /// caller hands that over (usart_column_for(n, code).ck) - and CPOL,
    /// CPHA and LBCL. Refused while LIN, half duplex or IrDA is on.
    static bool smartcard(const SmartcardConfig& c) {
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
    /// SCEN and its NACK dropped. The card clock is left running: CLKEN is
    /// the synchronous half's bit and synchronous_off() is what stops it.
    static void smartcard_off() {
        regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 & ~(usart_scen | usart_nack));
    }
    static bool smartcard_enabled() { return (regs().CTLR3 & usart_scen) != 0u; }

    /// The synchronous mode (14.4): the column's CK pad carries a clock
    /// while the transmitter shifts and at no other time, and this side is
    /// the master. Refused while LIN, half duplex, IrDA or the smartcard is
    /// on, and while the transmitter or the receiver is enabled, because
    /// CPOL, CPHA and LBCL "need to be set when TE and RE are not enabled".
    /// The CK pad itself is the caller's to hand to the peripheral.
    static bool synchronous(const UsartSyncConfig& c) {
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
        if ((regs().CTLR1 & (usart_te | usart_re)) != 0u) {
            return false;
        }
        regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 &
                                             ~(usart_clken | usart_cpol | usart_cpha | usart_lbcl));
        return true;
    }
    static bool synchronous_enabled() { return (regs().CTLR2 & usart_clken) != 0u; }

    /// The hardware flow-control pair (14.10.6): RTS driven low while the
    /// receiver can take a frame, CTS sampled before each frame goes out.
    static bool flow_control(bool rts, bool cts) {
        bit(regs().CTLR3, usart_rtse, rts);
        bit(regs().CTLR3, usart_ctse, cts);
        return true;
    }
    static bool rts_enabled() { return (regs().CTLR3 & usart_rtse) != 0u; }
    static bool cts_enabled() { return (regs().CTLR3 & usart_ctse) != 0u; }

    /// CTLR3's two DMA request enables (14.10.6, 14.8). With DMAT set, TXE
    /// raises a request instead of feeding the interrupt; with DMAR set,
    /// so does RXNE.
    static void dma_transmit(bool on) { bit(regs().CTLR3, usart_dmat, on); }
    static void dma_receive(bool on) { bit(regs().CTLR3, usart_dmar, on); }

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
    static void cts_interrupt(bool on) { bit(regs().CTLR3, usart_ctsie, on); }
    /// EIE: FE, ORE and NE raise the vector - under DMAR only (14.10.6).
    static void error_interrupt(bool on) { bit(regs().CTLR3, usart_eie, on); }

    static uint16_t status() { return regs().STATR; }
    static bool flag(uint16_t mask) { return (regs().STATR & mask) != 0u; }
    /// The write-zero-to-clear flags (RXNE, TC, LBD, CTS); any other bit in
    /// the mask is ignored, because writing it does nothing.
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

    /// The word as the receiver has it - nine bits when the frame is nine
    /// bits wide. Reading DATAR is what clears RXNE, and the STATR-then-
    /// DATAR pair is what clears the error flags.
    static uint16_t read_word() { return static_cast<uint16_t>(regs().DATAR); }
    static void write_word(uint16_t w) { regs().DATAR = w; }
    static uint8_t read_data() { return static_cast<uint8_t>(regs().DATAR & 0xFFu); }
    static void write_data(uint8_t b) { regs().DATAR = b; }

    /// Read the errors and clear them, the sequence the chapter prescribes
    /// (14.10.1): the status register, then the data one.
    static uint16_t take_errors() {
        const uint16_t status = regs().STATR;
        const uint16_t errors = static_cast<uint16_t>(status & (usart_pe | usart_fe | usart_ne | usart_ore));
        if (errors != 0u) {
            (void)regs().DATAR;
        }
        return errors;
    }

    /// The baud the port is actually running at, from the divisor in the
    /// register - what a program reports instead of what it asked for.
    static uint32_t actual_baud(uint32_t hclk) {
        const uint32_t brr = regs().BRR;
        return brr == 0u ? 0u : hclk / brr;
    }

private:
    static void bit(volatile uint16_t& r, uint16_t mask, bool on) {
        r = static_cast<uint16_t>(on ? (r | mask) : (r & ~mask));
    }
};

// =============================================================================
// Uart: the transport task
// =============================================================================

/**
 * What a Uart may be told beyond its frame, its rings and its engines, as
 * one trailing template parameter - the CH32V203 stratum's struct, whose
 * defaults are the console's personality.
 */
struct UartOptions {
    /// Single-wire half duplex (14.5): one pad, the TX pad, as an alternate
    /// push-pull output against an external pull; the RX pad is left alone.
    bool half_duplex = false;
    /// The flow-control pair on the column's CTS and RTS pads.
    bool rts = false;
    bool cts = false;
    /// The receive engine's level in the controller's arbitration (9.3.3's
    /// PL), `very_high` by default: DATAR holds one frame, so a channel
    /// starved for a frame time loses the next one to an overrun - which a
    /// memory-to-memory copy a level above the ring, or at its level on a
    /// lower channel, does (docs/design/dma.md). A program that ranks
    /// another channel above its ring says so here; refused on a port with
    /// no receive engine, which has no channel to rank.
    DmaPriority rx_priority = DmaPriority::very_high;
};

/**
 * The receive ring a transport keeps: a SkipRing, which the interrupt
 * receiver pushes into and tells when it lost a byte, without a receive
  * engine; with one, the consumer half of the ring the channel writes -
 * util/ring.hpp's HardwareRing over a storage array the transport owns
 * (keyed by the transport's type), the engine its RingCounter. A partial
 * specialization, so that HardwareRing over NoDmaEngine is never named.
 */
template <bool engine, typename Owner, uint16_t size, typename Engine, typename P>
struct UartRxRing {
    using type = SkipRing<uint8_t, size, P>;
};
template <typename Owner, uint16_t size, typename Engine, typename P>
struct UartRxRing<true, Owner, size, Engine, P> {
    static_assert(size >= 2u && size <= 0x8000u && (size & (size - 1u)) == 0u,
                  "brio Uart: a receive engine runs the whole receive ring as one circular "
                  "block - a power of two, 2..32768 bytes");
    static inline uint8_t storage[size]{};
    using type = HardwareRing<storage, Engine>;
};

/**
 * The serial port, interrupt-driven, with a DMA engine in either slot:
 * what a program names for its console, with the instance's vector
 * bound to isr() under the handler attribute and init() called after
 * the clock task.
 *
 *   using Serial = brio::Uart<2, Platform>;
 *   constexpr Serial serial;                 // tag for print(serial, ...)
 *   Serial::init(clock, 115200);
 *   extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { Serial::isr(); }
 *
 * `P` is the platform, which the rings need to know whether an index can
 * be shared with a handler bare (atomic_width) or wants a guard.
 * `Placement` is where the transmit engine's block restart and the
 * per-byte interrupt body run from (util/placement.hpp; the file
 * header): InFlash, the default, or InRam. Every template argument but
 * the instance and the platform has a default: the console is
 * `Uart<2, P>`.
 */
template <uint8_t instance, typename P, uint16_t rx_size = 64, uint16_t tx_size = 64,
          UartFormat format = {}, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine, uint8_t remap = 0, UartOptions opts = {}, typename Placement = InFlash>
struct Uart {
    static_assert(HotPlacement<Placement, P>,
                  "brio Uart: the placement is util/placement.hpp's InFlash (the default) or InRam, and "
                  "InRam only on a platform whose linker scripts place .fast_text");
    static_assert(instance >= 1u && instance <= 4u,
                  "brio Uart: this series has USART1..USART4");
    static_assert(device::has_usart(instance),
                  "brio Uart: this part does not offer that instance (parts/<part>.hpp)");
    static_assert(uart_format_valid(format) && format.bits != UartBits::nine,
                  "brio Uart: the rings carry bytes - seven data bits with a parity bit, or eight, "
                  "with or without one; nine-bit words are the resource's read_word()/write_word()");
    static_assert(usart_remap_valid(instance, remap),
                  "brio Uart: no such column for this instance on this part - the code does not "
                  "fit the field, the package does not bond its RX pad, or its TX pad is not one "
                  "a program may drive (afio.hpp's columns, pin.hpp's shorts, parts/<part>.hpp)");
    static_assert(!opts.rts || pad_can_drive(usart_column_for(instance, remap).rts),
                  "brio Uart: this column's RTS pad is not bonded on this package, or shares "
                  "its pin with another pad (parts/<part>.hpp)");
    static_assert(!opts.cts || pad_bonded(usart_column_for(instance, remap).cts),
                  "brio Uart: this package does not bond this column's CTS pad (parts/<part>.hpp)");
    // An engine is checked where it is NAMED: sizeof demands a complete
    // type, so an engine's own static_asserts fire on the line the
    // application wrote.
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / DmaRxEngine from "
                  "ch32x035/dma.hpp, or NoDmaEngine (the default)");
    static_assert(!TxEngine::present || dma_engine_channel<TxEngine>() == usart_dma_tx_channel(instance),
                  "brio Uart: RM 9.2.3 - this instance transmits on its own DMA channel (USART1 on 4, "
                  "USART2 on 7, USART3 on 2, USART4 on 1: ch32x035/dma_engine.hpp's table)");
    static_assert(!RxEngine::present || dma_engine_channel<RxEngine>() == usart_dma_rx_channel(instance),
                  "brio Uart: RM 9.2.3 - this instance receives on its own DMA channel (USART1 on 5, "
                  "USART2 on 6, USART3 on 3, USART4 on 8: ch32x035/dma_engine.hpp's table)");
    static_assert(dma_engines_distinct<TxEngine, RxEngine>(),
                  "the two engines of a Uart must not share a DMA channel");
    static_assert(RxEngine::present || opts.rx_priority == DmaPriority::very_high,
                  "brio Uart: rx_priority ranks the receive engine's channel - this port has no "
                  "receive engine");

    Uart() = default;   // a tag instance: constexpr Uart<2, P> serial;

    using Resource = Usart<instance>;

    static constexpr uint8_t number = instance;
    static constexpr uint8_t remap_code = remap;
    static constexpr UsartPads pads = usart_pads_for(instance, remap);
    static constexpr UsartPadSet column = usart_column_for(instance, remap);
    static constexpr UartOptions options = opts;
    static constexpr bool has_tx_engine = TxEngine::present;
    static constexpr bool has_rx_engine = RxEngine::present;
    /// write_bulk()'s copy into the ring: the runtime's memcpy for a run of
    /// at least this many bytes whose source and ring slot share their
    /// alignment modulo the word (rt/rt.cpp's word path), the byte loop
    /// otherwise, where memcpy would run the same loop behind a call.
    static constexpr uint32_t copy_threshold = 16;

    using Tx = Pin<pads.tx.port, pads.tx.pin>;
    using Rx = Pin<pads.rx.port, pads.rx.pin>;

    static UsartRegs& regs() { return Resource::regs(); }

    /**
     * Open the port at `baud` in the given frame (8N1 by default), with the
     * receive interrupt armed - or, with a receive engine, the ring running
     * and the vector waiting for the end of a burst. A setup verb: main(),
     * after the clock task (`clock` is its tag; the divisor is computed
     * from the rate its type claims, so a port opened on a chip still at
     * its reset rate speaks at a sixth of the rate asked and nothing
     * refuses it) and before interrupts are enabled globally. Waits for
     * nothing; opens the gate, writes the column, hands the pads over,
     * configures the frame and the divisor, binds the engines and enables
     * the PFIC line.
     *
     * Returns false when the divisor comes out below the smallest the
     * chapter allows (16), and when the column lands on the debug port's
     * pads while the two-wire port is still the probe's - a baud rate this
     * clock cannot serve, or a column that would fight the link that put
     * the image there, is a fact the caller must see.
     */
    template <typename C>
    static bool init(C clock, uint32_t baud) {
        static_assert(clock_follows<C, Uart>(),
                      "brio Uart: the baud divisor is derived from HCLK, so a dynamic clock must "
                      "list this port among the users it rebases");

        const uint32_t brr = usart_divisor(clock_hz(clock), baud);
        if (!usart_divisor_valid(brr)) {
            return false;
        }

        if constexpr (usart_column_on_debug_port(instance, remap)) {
            if (Afio::debug_port_enabled()) {
                return false;
            }
        }

        Resource::bus_clock(true);

        // The column, only where it is not the reset one: AFIO's reset
        // value already selects code 0.
        if constexpr (remap != 0u) {
            (void)Resource::remap(remap);
        }

        // Pads before the enable: TE's idle frame must land on a pad the
        // peripheral already owns.
        Tx::function();
        if constexpr (!opts.half_duplex) {
            (void)Rx::input();                   // floating: the peer drives it
        }
        if constexpr (opts.rts) {
            Rts::function();
        }
        if constexpr (opts.cts) {
            (void)Cts::input(PinPull::up);       // unconnected reads "not clear to send"
        }

        if (!Resource::configure(format, brr)) {
            return false;
        }
        m_baud = baud;

        m_tx.clear();
        m_rx.clear();
        clear_errors();
        m_dma_faults = 0;

        // CTLR3 is touched only by a port that has something to put in it:
        // with no option and no engine there is nothing of this transport's
        // there, and whatever a program stored itself stays. An engine's
        // request enable (DMAT, DMAR) and, under the receive engine, EIE -
        // FE, NE and ORE raise the vector under DMAR alone (14.10.6).
        if constexpr (opts.half_duplex || opts.rts || opts.cts || has_tx_engine || has_rx_engine) {
            uint16_t ctlr3 = regs().CTLR3;
            if constexpr (opts.half_duplex) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_hdsel); }
            if constexpr (opts.rts) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_rtse); }
            if constexpr (opts.cts) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_ctse); }
            if constexpr (has_tx_engine) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_dmat); }
            if constexpr (has_rx_engine) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_dmar | usart_eie); }
            regs().CTLR3 = ctlr3;
        }

        if constexpr (has_rx_engine) {
            m_rx_waiting = false;
            m_rx_drained = true;
            m_rx_given_up = false;
            // The whole receive storage, circular, its half and full marks
            // the edge of a stream with no silence - at the options' level,
            // very_high unless the program says otherwise
            // (UartOptions::rx_priority). Bound before UE: the channel is
            // waiting for the first RXNE when the receiver opens.
            if (!RxEngine::arm_ring(&regs().DATAR, RxRing::storage, true, opts.rx_priority)) {
                return false;
            }
        }
        if constexpr (has_tx_engine) {
            // High, a level below the receive rings: a transmit channel that
            // waits leaves the line idle a while (TXE stands, 14.10.1) and
            // loses nothing.
            TxEngine::arm(&regs().DATAR, DmaPriority::high);
        }

        // RXNE is the receive channel's when an engine has it, and the
        // vector then waits for the end of a burst: the idle line and the
        // errors (EIE above, PE on its own enable).
        regs().CTLR1 = static_cast<uint16_t>(usart_ctlr1_format(format) | usart_ue | usart_te | usart_re |
                                             (has_rx_engine ? static_cast<uint16_t>(usart_idleie | usart_peie)
                                                            : usart_rxneie));

        Pfic::enable(usart_irq_for(instance));
        return true;
    }

    /**
     * The interrupt body of whichever DMA channel this transport owns -
     * call it from the vector(s) the engines' channels report on, and
     * from nowhere else:
     *
     *     extern "C" BRIO_CH32_INTERRUPT void dma1_channel1_handler() { (void)Link::dma_isr(); }
     *
     * The flags of every channel are one register (INTFR), read ONCE here
     * for both engines; each engine takes and clears only its own channel's
     * armed flags, so this is safe on a vector another channel of the
     * program shares nothing with. On the transmit channel a completion
     * releases exactly the block's bytes from the ring and starts the next
     * run; on the receive channel a completion is a LAP of the ring,
     * counted, and the lap's half and full marks are the receive edge; a
     * transfer error stops the channel, and the consumer's next look starts
     * it again.
     *
     * Returns true when the receive ring holds bytes its consumer has not
     * been told of - the edge, as isr()'s: post RxActivity on true. False,
     * and free, with the slots empty.
     */
    [[gnu::always_inline]] static bool dma_isr() {
        return Hot<Placement>::run([] { return dma_entry(); });
    }

    /**
     * The receive edge asked from the consumer's side, and the ring's
     * housekeeping. Nobody needs to ask: isr() and dma_isr() report the
     * edge (the file header). This is the same gate from the main context,
     * for an owner that still asks - true when the ring holds bytes and the
     * consumer has found it empty since the last true, from a vector or
     * from here - and a channel a transfer error stopped is started again
     * (the look is the consumer's, so is the restart; every read verb does
     * the same). It reads neither STATR nor DATAR: the errors are the
     * vector's to count. False, and free, without an engine.
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

    /// Transfer errors the engines reported (a fault is a block thrown
    /// away), and receive channels given up on a frame they never took.
    /// Always 0, and free, without an engine.
    static uint16_t dma_faults() { return m_dma_faults; }

    /**
     * The port's whole interrupt body - bind this instance's vector to it
     * (the class comment), and call it from nowhere else. It serves the
     * transmit ring on TXE and the receive ring on RXNE, counts the
     * errors, and masks nothing.
     *
     * Returns true when a byte arrived into an empty ring: the edge
     * util/serial_port.hpp posts on (RxActivity to the active object that
     * assembles lines); a program with no such object discards it.
     */
    // always_inline: one call site (the app's vector binding), so the
    // compiler saves only the registers this body uses. Through the port's
    // placement (the file header): inline in the vector by default, one
    // instance in SRAM under InRam.
    [[gnu::always_inline]] static bool isr() {
        return Hot<Placement>::run([] { return isr_entry(); });
    }

private:
    /// The DMA vector's body, written once: dma_isr() runs it through the
    /// port's placement (the file header) - the flags' read and the
    /// channels' service with the restart, all of it, so under InRam the
    /// vector's own path in the flash is its entry and one call.
    [[gnu::always_inline]] static bool dma_entry() {
        bool edge = false;
        uint32_t intfr = 0;
        if constexpr (has_tx_engine) {
            intfr = TxEngine::block_flags();
        } else if constexpr (has_rx_engine) {
            intfr = RxEngine::block_flags();
        }
        if constexpr (has_tx_engine) {
            const uint8_t f = TxEngine::service(intfr);
            if ((f & TxEngine::flag_error) != 0u) {
                tx_fault();
            } else if ((f & TxEngine::flag_complete) != 0u) {
                tx_block_done();
            }
        }
        if constexpr (has_rx_engine) {
            const uint8_t f = RxEngine::service(intfr);
            if ((f & RxEngine::flag_error) != 0u) {
                rx_fault();
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

    /// A channel's transfer error, out of line: the body above is flattened
    /// whole, and the rare path stays a call rather than bytes of the
    /// frequent one - SRAM bytes under InRam. The fault counted, saturating.
    [[gnu::noinline]] static void tx_fault() {
        (void)TxEngine::abandon();
        bump(m_dma_faults);
    }
    [[gnu::noinline]] static void rx_fault() {
        (void)RxEngine::abandon();
        bump(m_dma_faults);
    }

    /// The interrupt body's path, written once: isr() runs it through the
    /// port's placement.
    [[gnu::always_inline]] static bool isr_entry() {
        bool rx_edge = false;
        bool waited = false;
        // With a receive engine the handler never reads DATAR, and the wait
        // for a frame comes first: it reads no STATR, and with a transmit
        // engine nothing else of this port raises the vector.
        if constexpr (has_rx_engine) {
            if (m_rx_waiting) {
                waited = true;
                rx_edge = rx_frame_entry();
                if constexpr (has_tx_engine) {
                    return rx_edge;
                } else {
                    // The wait for a frame reads no STATR, with the
                    // interrupt transmitter idle too: the status read below
                    // is TXE's, and made with TXEIE down it armed the clear
                    // of the NEXT frame's error for nothing - after one
                    // counted error a run of errored frames counted no
                    // more (measured, docs/ch32x035/usart.md). CTLR1 is read
                    // instead, which arms nothing.
                    if ((regs().CTLR1 & usart_txeie) == 0u) {
                        return rx_edge;
                    }
                }
            }
        }

        const uint16_t status = regs().STATR;

        if constexpr (has_rx_engine) {
            if (!waited) {
                rx_edge = rx_end_entry(status);
            }
        } else if ((status & usart_rxne) != 0u) {
            const uint8_t byte = static_cast<uint8_t>(regs().DATAR & 0xFFu);
            // Every byte lost is reported to the receive ring (lost()):
            // the flagged byte dropped here - with an overrun's, lost in
            // the shift register - and one a full ring refuses. The
            // consumer's next look skips everything queued.
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

        if constexpr (!has_tx_engine) {
            if ((status & usart_txe) != 0u && (regs().CTLR1 & usart_txeie) != 0u) {
                if (const auto next = m_tx.pop()) {
                    regs().DATAR = *next;
                } else {
                    // TXE stands while the transmitter is idle: disarm, or
                    // the handler is re-entered for ever.
                    regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_txeie);
                }
            }
        }

        return rx_edge;
    }

public:
    // ---- byte transport (ByteSink / ByteSource) ---------------------------

    /// Queue one byte; false when the TX ring is full (print() spins). Main
    /// context only: the ring's one producer. The byte is copied; nothing
    /// waits. Without an engine TXE is armed; with one the engine is
    /// nudged - and a
    /// refused byte nudges too, since the ring is full exactly when nothing
    /// is draining it and print() would otherwise spin on a transport nobody
    /// poked.
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
    /// receive engine, when it was written over while it was read
    /// (rx_overruns() counts it). Main context only: the ring's one
    /// consumer.
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

    /// Queue a run (util/stream.hpp's BulkSink): as much of `src` as the
    /// ring has room for, copied into the ring's own free run and handed
    /// over with one index store per contiguous part. Never blocks; returns
    /// how many were queued, and the caller's span is free when it
    /// returns. Main context only. A run that finds no room is
    /// write_byte()'s refusal: nothing armed on the plain transport (the
    /// push that filled the ring armed TXEIE, a level), the engine nudged
    /// on the engined one.
    ///
    /// The first byte goes before the copy, on the plain transport: it is
    /// pushed and TXEIE armed exactly as write_byte() does, so an idle
    /// transmitter starts on it while the rest is copied. The handler
    /// disarms on the TXE entry that finds the ring empty - the one after
    /// it hands DATAR that byte, TXE rising again as the byte moves to the
    /// shift register - and the first byte was the whole ring, so the rest,
    /// once published, is armed again: one more CTLR1 read-modify-write,
    /// idempotent, two a run at most. With an engine the run is copied whole
    /// and the engine nudged once, a block being what it starts on.
    static uint32_t write_bulk(std::span<const uint8_t> src) {
        uint32_t queued = 0;
        if constexpr (!has_tx_engine) {
            // write_byte()'s two steps, spelled here so the run's path holds
            // no call: a full ring is its refusal, nothing to arm.
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
            // Two pointers and no index, and the test at the bottom: a load,
            // a store, two steps and one branch a byte, where an index
            // re-adds both bases every byte. `take` is at least one here -
            // the room is not empty and the run is not done.
            const uint8_t* from = src.data() + queued;
            uint8_t* to = room.data();
            if constexpr (has_tx_engine) {
                if (take >= copy_threshold &&
                    ((reinterpret_cast<uintptr_t>(to) ^ reinterpret_cast<uintptr_t>(from)) & 3u) == 0u) {
                    (void)memcpy(to, from, take);
                    m_tx.publish(static_cast<typename decltype(m_tx)::index_t>(take));
                    queued += take;
                    continue;
                }
            }
            uint8_t* const end = to + take;
            do {
                *to++ = *from++;
            } while (to != end);
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

    /// The received bytes in place: the contiguous run ready to be read,
    /// never wrapping - the receive ring's consumer half under the ring's
    /// own names, valid until consume() releases it. With consume() it is
    /// util/stream.hpp's SpanSource, which SerialPort drains a run at a
    /// time. With a receive engine the run is the channel's ring, read
    /// where it writes, and consume() says whether it was intact. Main
    /// context only.
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

    /// The wire is idle: nothing queued, no block in flight, and the last
    /// frame's stop bit out (STATR.TC, 14.10.1) - what a program waits on
    /// before a reset or a clock change. A transmit block clears TC as it
    /// starts (pump_tx()), so the flag cannot answer from the frame before
    /// the block.
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

    /// The gaps in the stream the receive ring hands out, since the program
    /// started, never cleared - util/stream.hpp's SkippingSource,
    /// util/serial_port.hpp's epoch. Without an engine: the SkipRing's
    /// skips (modulo 2^8), one at the consumer's look after a byte the
    /// handler dropped for a flag or a full ring, or an overrun - the look
    /// that discards what the ring held. With one: the view's skips (a lap
    /// missed, a held run refused) plus m_rx_lost - an overrun, a restart
    /// after a transfer error (a framed frame is stored: the channel moves
    /// it) - counted when the vector or the look sees them, so the line
    /// torn is the one begun when the consumer next looks.
    static uint32_t rx_skips() {
        if constexpr (has_rx_engine) {
            return m_rx.skips() + m_rx_lost;
        } else {
            return m_rx.skips();
        }
    }

    static uint32_t baud() { return m_baud; }
    static uint32_t actual_baud(uint32_t hclk) { return Resource::actual_baud(hclk); }

    /// Bytes the receive side lost to a consumer that did not keep up:
    /// without an engine, a byte a full ring refused; with one, a lap of the
    /// channel's ring written over unread bytes, or a run written over while
    /// held - each one a skip, saturating here.
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
    }

    /// The divisor this clock and baud ask for, and the two questions a
    /// program asks before it moves a rate.
    static constexpr uint32_t divisor_for(uint32_t hclk, uint32_t baud) {
        return usart_divisor(hclk, baud);
    }
    static constexpr uint32_t min_hz_for(uint32_t baud) { return usart_min_hz(baud); }
    static constexpr bool can_baud(uint32_t hclk, uint32_t baud) {
        return usart_divisor_valid(usart_divisor(hclk, baud));
    }

    /// Follow a clock that changed rate. The argument is HCLK, which is what
    /// the divisor counts on this series.
    ///
    /// It is called before the rate moves, with the old clock still running,
    /// so the queue is drained here: a byte still in the shift register when
    /// HCLK changes goes out at neither baud rate. The drain is bounded - a
    /// port whose line is held off must not hang the switch - and the
    /// divisor is written whatever the drain found.
    static void rebase(uint32_t hz) {
        drain();
        const uint32_t brr = usart_divisor(hz, m_baud);
        if (usart_divisor_valid(brr)) {
            regs().BRR = static_cast<uint16_t>(brr);
        }
    }

    /// Move the link to a different bit rate, the clock staying put - the
    /// mirror of rebase(), `hz` being HCLK as rebase() takes it. False, and
    /// nothing written, when the rate is unreachable.
    static bool set_baud(uint32_t hz, uint32_t baud) {
        const uint32_t brr = usart_divisor(hz, baud);
        if (!usart_divisor_valid(brr)) {
            return false;
        }
        drain();
        regs().BRR = static_cast<uint16_t>(brr);
        m_baud = baud;
        return true;
    }

    /// Stop the port and park its pads: the vector off, the engines'
    /// channels stopped, UE clear, the block's reset pulsed, the bus clock
    /// closed, the pins released. A setup verb, main context; what the
    /// rings held no longer goes out, and is cleared by the next init().
    ///
    /// The reset line before the gate (docs/design/dma.md, the release
    /// contract). A channel of this controller is an OR of the requests
    /// wired to it (dma_engine.hpp's table: USART2's two share 6 and 7 with
    /// the I2C, USART4's transmit shares 1 with the ADC), and on this die a
    /// raised request outlives its peripheral's release unless the block
    /// is reset (measured on the CH32X035F8U6, docs/ch32x035/dma.md, "A
    /// released requester"): the next owner's first enable would move one
    /// stray item. The pulse - two read-modify-writes of the bus's reset
    /// register - leaves none, and takes CTLR3 with it.
    static void release() {
        Pfic::disable(usart_irq_for(instance));
        if constexpr (has_tx_engine) {
            TxEngine::stop();
        }
        if constexpr (has_rx_engine) {
            RxEngine::stop();
        }
        regs().CTLR1 = 0;
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
    /// The flow-control pads, formed only where the options ask for them -
    /// a column that has no such pad would not make a Pin at all.
    using Cts = Pin<opts.cts ? column.cts.port : pads.tx.port,
                    opts.cts ? column.cts.pin : pads.tx.pin>;
    using Rts = Pin<opts.rts ? column.rts.port : pads.tx.port,
                    opts.rts ? column.rts.pin : pads.tx.pin>;

    /// Start the next contiguous run of the TX ring on the engine, if it is
    /// idle and there is one. The mask covers the claim and nothing else:
    /// the engine's busy flag tested and set under the guard - the
    /// completion path, which clears it, runs in the channel's handler -
    /// and the run read and the block programmed outside it, because a
    /// claimed engine has no block in flight and so no completion can land
    /// under the programming, nor any consume() move the ring's tail. The
    /// producer is this context (the ring is SPSC), so a run found empty
    /// after the claim stays empty until the claim is given back.
    static void pump_tx() {
        if constexpr (has_tx_engine) {
            if (m_tx.empty()) {
                return;
            }
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
            (void)TxEngine::launch(run);
        }
    }

    /// The transmit block's completion: its bytes released from the ring,
    /// the next run started - flattened into dma_entry(), the port's
    /// placement's (the file header).
    [[gnu::always_inline]] static void tx_block_done() {
        m_tx.consume(static_cast<typename decltype(m_tx)::index_t>(TxEngine::complete()));
        pump_tx();
    }

    /// The transmit ring out to the wire, bounded: what rebase() and
    /// set_baud() wait for before the divisor moves.
    static void drain() {
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

    /// The consumer's look at the receive ring under an engine: a channel a
    /// transfer error stopped started again first, and a look that finds
    /// the ring empty re-opens the edge and looks again, so a byte landing
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
    /// stopped by a transfer error, and is bound again from the storage's
    /// first element with the view - the consumer's context. A ring the
    /// vector GAVE UP on (rx_stalled()) is bound again the same way: its
    /// channel still enabled over a frozen handshake, the binding's first
    /// store takes EN down.
    ///
    /// A restarted ring waits for the end: the vector's state is put back
    /// with the channel - the wait for a frame it was in counted the old
    /// lap's position, and a channel the vector gave up on left every
    /// receive enable down. One read-modify-write of each control register
    /// under the guard, the transmitter's TXEIE sharing CTLR1 with them; the
    /// rare path only.
    static void restart_if_stopped() {
        if constexpr (has_rx_engine) {
            if (RxEngine::idle() || m_rx_given_up) {
                m_rx_given_up = false;
                (void)RxEngine::arm_ring(&regs().DATAR, RxRing::storage, true, opts.rx_priority);
                m_rx.clear();
                m_rx_lost = m_rx_lost + 1u;   // the unread bytes went with the stopped lap
                typename P::CriticalSection cs;
                m_rx_waiting = false;
                regs().CTLR1 = static_cast<uint16_t>((regs().CTLR1 & ~usart_rxneie) |
                                                     usart_idleie | usart_peie);
                regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 | usart_eie);
            }
        }
    }

    /// The edge's gate, for the vectors: true once per idle-to-busy
    /// transition of the consumer. Each caller has seen the channel write
    /// since the consumer was last told, so no look at the view is needed.
    [[gnu::always_inline]] static bool told() {
        if (m_rx_drained) {
            m_rx_drained = false;
            return true;
        }
        return false;
    }

    /// Waiting for the end (the file header): an idle line or an error, from
    /// the entry's one status read. An entry showing neither and no transmit
    /// condition to explain it is an idle line a status read elsewhere armed
    /// and a frame cleared before this vector ran: served as the idle it
    /// was. CNTR is read before RXNEIE is armed and after: a frame the
    /// channel took between raises no entry, so it is served here.
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

    /// Waiting for a frame: the channel has moved since the wait began, so
    /// its read finished the clear - back to waiting for the end, and the
    /// edge, which is the only one a burst of one frame gets. No STATR read:
    /// it would arm the clear the next frame's read performs.
    ///
    /// An entry with the count unmoved is the transmitter's, or a frame the
    /// channel has not taken yet - or one it NEVER takes: RXNE is a level,
    /// and over a channel that does not serve its request this vector would
    /// be re-entered for ever, the thread starved. So the entries that find
    /// the count unmoved with nothing else armed to explain them are
    /// counted, and the rx_stall_entries-th gives the channel up
    /// (rx_stalled()); the count then stays there, the state parked until
    /// the consumer's look restarts the ring. Or one with nothing for the
    /// channel to take at all - an overrun that left ORE up with RXNE down,
    /// which RXNEIE re-enters on: the rx_overrun_probe-th unmoved entry asks
    /// STATR, the one status read of this state, and clears it.
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
            if (m_rx_unmoved == rx_overrun_probe) {
                // The overrun with nothing to take (rx_overrun_probe): one
                // status read, and DATAR read where it shows ORE with RXNE
                // down - the clear the wait's own status read armed.
                const uint16_t status = regs().STATR;
                if ((status & (usart_ore | usart_rxne)) == usart_ore) {
                    (void)regs().DATAR;
                    return rx_wait_for_the_end();
                }
            }
            if (m_rx_unmoved != rx_stall_entries) {
                return false;   // the frame not yet taken
            }
            return rx_stalled();
        }
        return rx_wait_for_the_end();
    }

    /// Back to waiting for the end, and the edge: RXNEIE down, IDLEIE, PEIE
    /// and EIE up. The channel moved, or the wait's overrun was cleared
    /// (rx_frame_entry()).
    [[gnu::always_inline]] static bool rx_wait_for_the_end() {
        m_rx_waiting = false;
        regs().CTLR1 = static_cast<uint16_t>((regs().CTLR1 & ~usart_rxneie) | usart_idleie | usart_peie);
        regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 | usart_eie);
        return told();
    }

    /**
     * The entry of the wait for a frame at which an unmoved count is asked
     * why: an overrun with nothing to take. A starved channel can leave ORE
     * up with RXNE down - the frame it took at last is out of DATAR, the one
     * the overrun lost never reached it - and when no frame follows (a
     * message's tail), RXNEIE, which ORE raises on its own (14.10.1),
     * re-enters the vector back to back with nothing for the channel to
     * move. So the second unmoved entry - the first one is the frame not yet
     * taken that a contended channel shows - reads STATR once, and where ORE
     * stands with RXNE down reads DATAR: the clear 14.10.1 prescribes, and
     * the vector goes back to the wait for the end.
     *
     * What the two reads cost the counts: nothing, and the reason is the
     * count unmoved. No DATAR read has been made since the status read that
     * began this wait, so the clear that read armed is armed still and a
     * second status read arms nothing new; the DATAR read performs it, over
     * flags that read already saw and counted. A frame waiting for a
     * starved channel shows RXNE, and nothing is read but the status.
     */
    static constexpr uint8_t rx_overrun_probe = 2;

    /// How many entries of the wait for a frame may find the channel's count
    /// unmoved before the channel is given up. A live channel takes RXNE's
    /// byte within its arbitration - a few bus cycles behind any other
    /// channel's item - where one entry of this vector is tens of cycles, so
    /// a frame not yet taken is one or two entries; 255 is thousands of
    /// cycles of a channel that moved nothing. A channel a memory-to-memory
    /// block outranks and starves for that long - a ring a program armed
    /// below its copy (UartOptions::rx_priority) - is given up too, and
    /// restarted by the consumer's next look.
    static constexpr uint8_t rx_stall_entries = 255;

    /// The channel given up: the fault counted, the ring marked for the
    /// consumer's look to bind again (restart_if_stopped(), which stops the
    /// channel and starts it from the storage's first element, as after a
    /// transfer error) and the edge reported so the consumer comes. RXNEIE
    /// goes down, the one enable of this state, so the standing RXNE
    /// re-enters nothing in between.
    [[gnu::always_inline]] static bool rx_stalled() {
        bump(m_dma_faults);
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_rxneie);
        m_rx_given_up = true;
        m_rx_drained = false;
        return true;
    }

    /// Saturating: a counter that wraps would report a healthy port.
    [[gnu::always_inline]] static void bump(volatile uint16_t& counter) {
        const uint16_t n = counter;
        if (n != 0xFFFFu) {
            counter = static_cast<uint16_t>(n + 1u);
        }
    }

    using RxRing = UartRxRing<RxEngine::present, Uart, rx_size, RxEngine, P>;
    static inline typename RxRing::type m_rx{};
    static inline Ring<uint8_t, tx_size, P> m_tx{};
    static inline uint32_t m_baud = 0;
    /// Whether the consumer has found the receive ring empty since the edge
    /// was last reported - set by its look, cleared by whoever reports.
    static inline volatile bool m_rx_drained = true;
    /// The receive engine's vector state: waiting for a frame, and CNTR
    /// when that wait began.
    static inline volatile bool m_rx_waiting = false;
    static inline volatile uint16_t m_rx_at = 0;
    /// The entries of that wait that found the count unmoved
    /// (rx_frame_entry()); the vector's alone.
    static inline uint8_t m_rx_unmoved = 0;
    /// The vector gave the channel up: set there, cleared by the consumer's
    /// restart.
    static inline volatile bool m_rx_given_up = false;
    /// The counters: bumped by the vector, read and cleared by the thread -
    /// volatile, so a thread that reads one after a wait loads it again
    /// rather than reusing a value from before the wait (a read of
    /// dma_faults() hoisted over a settle answered 0 for a channel the
    /// vector had given up; test_ch32_serial letter m).
    static inline volatile uint16_t m_rx_overruns = 0;
    static inline volatile uint16_t m_hw_overruns = 0;
    static inline volatile uint16_t m_frame_errors = 0;
    static inline volatile uint16_t m_noise_errors = 0;
    static inline volatile uint16_t m_parity_errors = 0;
    static inline volatile uint16_t m_dma_faults = 0;
    /// Under a receive engine, the bytes lost beside the view's skips (an
    /// overrun, a restart), never cleared: rx_skips()'s second term.
    static inline volatile uint32_t m_rx_lost = 0;
};

} // namespace brio
