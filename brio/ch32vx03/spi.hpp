/*
 * spi.hpp
 *
 * The SPI of the CH32V203 and the CH32V303 (RM ch. 20) as a bus for a
 * program: a display, a flash chip or a converter driven from an active
 * object (a component of a brio program with its own event queue, served
 * one event at a time by the kernel) through the arbiter of
 * util/spi_bus.hpp, this chip as a client another host clocks, or, on
 * the CH32V303RC and VC, an I2S audio link. Up to three instances, both
 * roles, 8- or 16-bit frames, the four modes, the three chip-select
 * arrangements, the simplex and bidirectional line modes, hardware CRC
 * and two DMA requests per instance - the STM32F1's SPI under WCH's
 * register names, with one register of its own (HSCR, a high-speed read
 * mode) - and on the CH32V303RC and VC the I2S face of SPI2 and SPI3.
 * Four layers: the resource, the host engine and the client of the stack
 * docs/design/spi-bus.md draws, and the audio face.
 *
 *  - `Spi<n>` is the resource, a monostate (static members only, never
 *    instantiated): the register block, its gate and reset, the
 *    configuration under the rules the chapter states, the data and
 *    status verbs, the sequences that clear each flag, and the ISR body
 *    that reads the raised-and-enabled sources. It decides nothing.
 *  - `SpiHost<n, pins, TxEngine, RxEngine>` is the engine
 *    util/spi_bus.hpp's SpiBus drives, the layer that moves one
 *    transaction's frames, in a polled loop or on two DMA channels, the
 *    way each request names. Its Request carries the fields
 *    docs/design/spi-bus.md lists as every family's ("Realizations": a
 *    chip select, D/C, a command phase, a data phase with optional out
 *    and in, the per-request mode, the engine that runs it), with the
 *    rate and the frame size as this family's SpiClock and SpiDataSize,
 *    so an 8-bit request written over SpiBus
 *    on another family carries the same fields here, the rate and the
 *    frame size apart: their types are each family's, and so is the
 *    rate field's name (`baud` on the SAM C21).
 *  - `SpiClient<n, pins>` is the other end of the wire, a thin polled
 *    surface with an ISR body: a client is a protocol, and which one is
 *    the application's.
 *  - `I2s<n>` is the same register block in its audio face, a resource
 *    and nothing above it: the four standards, the data and channel
 *    widths, both roles and both directions, the clock generator and its
 *    master clock, the flags and the requests the two faces share.
 *
 * The smallest use, docs/ch32vx03/spi.md's first block of "How to use
 * it" - one device, polled, on SPI2's own column, in main() after the
 * clock task's init():
 *
 *     using Bus = brio::SpiHost<2>;
 *     Bus::init(clock);                        // no ceiling: every rate legal
 *
 *     const uint8_t cmd[1] = {0x9F};
 *     uint8_t id[3] = {};
 *     Bus::Request r{};
 *     r.cs      = brio::Pin<'B', 12>::ref();
 *     r.cmd     = brio::lend<brio::Lease::reply>(cmd);
 *     r.cmd_len = 1;
 *     r.rx      = brio::lend<brio::Lease::reply>(id);
 *     r.len     = 3;
 *     r.mode    = brio::SpiMode::mode0;
 *     r.clock   = brio::SpiClock::div16;
 *     r.engine  = brio::SpiEngine::polled;
 *     (void)Bus::start(r);                     // true: it is done, `id` is filled
 *
 * and its second block, the same request on a bus several active
 * objects share: the arbiter (util/spi_bus.hpp, the active object that
 * runs the posted requests one after another),
 * `using Arb = brio::SpiBus<Bus, Platform, 4>;`, listed in the program's
 * Tenuto, over a host with SPI2's two DMA engines (`DmaTxEngine<1, 5>`,
 * `DmaRxEngine<1, 4>`), the two channels' vectors bound with
 * `BRIO_CH32_VECTOR` to bodies that post `TransferDone{Bus::status()}`
 * to the arbiter when `Bus::dma_rx_isr()` or `Bus::dma_tx_isr()` answers
 * true, then `r.engine = SpiEngine::dma`, a `reply_to<MyAo, SpiDone>()` and
 * `brio::post<Arb>(r)` from an active object's handler (SpiHost below
 * writes it out). The chip select is a plain pin the program has made
 * an output and driven high first (pin.hpp's `output(true)`): the engine
 * only stores to it, one BSHR or BCR store an edge.
 *
 * Who calls what, and from where. init() runs in main(), after the
 * clock task's init(): every rate it works out - the instance's bus, a
 * ceiling's code, the chip-select setup's busy-wait - is the clock
 * type's and not the hardware's, so a host
 * initialized at the 8 MHz reset clock under a 144 MHz type clocks
 * SPI1's requests eighteen times and SPI2's and SPI3's nine times
 * slower than sck_hz() reports (derived from the arithmetic, not
 * measured), and nothing checks the order; init() waits for nothing.
 * release(), bit_order(), pad_speed(s), prime() and claim_nss_pad() are
 * setup verbs as well, main context or one active object, none guarded;
 * release(), and bit_order() and prime() when they move CTLR1, drain
 * the block first with Spi::disable()'s bounded waits (Spi::wait_polls
 * each, derived from the slowest frame the chapter allows) and discard
 * their answer. start() is the arbiter's, from its
 * dispatch in main context, or the program's for a polled request on a
 * bus no arbiter owns. A request names the engine that runs it
 * (`Request::engine`, docs/design/spi-bus.md, "Two ways to run a
 * request"), and the host does what it says, at any length. SpiEngine::polled
 * runs an explicit loop, whatever engines the host names, and completes
 * inside start(), spinning under one spin budget for the transaction
 * (take_budget(): twice the request's wire time at the rate applied, in
 * polls of at least a core cycle each; a budget that runs out ends
 * spi_stalled, the select and the D/C released and the block reset before
 * start() returns); it depends on no
 * interrupt vector and needs no engines, so it runs with global interrupts
 * masked as it does with them enabled. SpiEngine::dma runs on the DMA
 * engines: start() launches them and returns false, and the receive
 * channel's vector ends the request, so it needs engines bound and their
 * vectors bound. What it cannot run is refused, start() answering true
 * with spi_rejected before the block, the select or the D/C is touched:
 * any request on a host with no engines; on a host with them, 16-bit frames
 * from an odd address or over engines of uint8_t, a run across 64 KB on the
 * CH32V303's DMA1 (SpiHost below). And a host with engines refuses any
 * request while a block of its own is still on them - the one busy check,
 * made first, the same way whichever engine the request names.
 * One part of a dma request runs in a loop inside start(): a command phase
 * with a data phase behind it. Inside start() the
 * chip-select setup time is a busy wait, and a request that changes the
 * mode, the rate or the width drains the block first with the same
 * bounded waits, their answer discarded (apply()). dma_rx_isr(),
 * dma_tx_isr() and dma_isr() are vector bodies, called from the program's
 * bound vector and nowhere else; they wait for nothing, and on a transfer
 * error they only drop the requests, put the channels away, release the
 * select and note that a reset is owed. WHEN A REQUEST ENDS THE WIRE HAS
 * ENDED, on either engine and with or without a select: start() returning
 * true, or a vector body returning true, comes after the last SCK edge, and
 * the select rises after it too. The polled write waits for BSY to fall;
 * the polled read and the receive channel's completion rest on RXNE, which
 * this silicon raises at the frame's end and not before it in all four
 * modes (test_vx03_spi letter s, docs/ch32vx03/spi.md), so nothing waits for
 * more. On the CH32V303's images (the F
 * extension, the hardware prologue on) the vector BRIO_CH32_VECTOR binds
 * is a naked trampoline whose body must do no floating-point work
 * (pfic.hpp); these bodies do none. AFTER spi_stalled OR spi_dma_fault THE
 * PROGRAM CALLS NOTHING: the next start() finds a clean block, and a
 * request queued behind the failed one runs on it (SpiHost below).
 * recover() is the timed arbiter's, on a tenure (a transaction's time on
 * the bus) that never answered, or a program's, and waits, bounded, for
 * the drain. status() and busy() are reads of volatile flags, legal from
 * any context. Without engines
 * nothing here masks interrupts; with them, wherever a DMA block starts
 * or ends (in start(), in the vectors, in init(), recover() and
 * release()) the channel's entry into or exit from the count of active
 * bus masters is made under an InterruptGuard of a few instructions
 * (ch32vx03/bus_activity.hpp). The host arms no interrupt of the SPI
 * block: init(), recover() and release() leave the instance's own line
 * disabled at the PFIC. The Request is lent for the start() call, and
 * what a tenure on the engines needs after it is copied; its three
 * buffers are lent from the post to the SpiDone the arbiter replies with
 * - untouched and in scope until then, which nothing checks - and, for a
 * polled start() called directly, for the call. The client's runs are polled verbs that spin for a whole
 * transaction under the caller's spin budget, and its isr() is its
 * vector's body. The I2S face's smallest use is written out at I2s
 * below.
 *
 * Instances on two buses, and that is the rate table. SPI1 sits on PB2,
 * whose rate is HCLK on this family, and SPI2 and SPI3 on PB1, which the
 * stratum caps at 72 MHz - so the same BR code means two different
 * frequencies on SPI1 and on the other two, and a program must ask the
 * instance (`Spi<n>::bus_hz(clock)`), never the system clock. Which
 * instances a part has is `device::spi_instances` (the datasheets' tables
 * 2-1 and 2-1-1: the parts below the CH32V203C8 have SPI1 alone, the
 * CH32V203C8 and RB and the 128 KB CH32V303 SPI1 and SPI2, the CH32V303RC
 * and VC all three), and an instance a part has not got does not compile.
 *
 * No FIFO (figure 20-1): one transmit buffer, one receive buffer, one
 * shift register. TXE means "the buffer moved into the shifter" - a
 * frame early - and RXNE "a frame has been shifted both ways", which is
 * the one moment the bus is provably idle; so the host's receive loop
 * reads on RXNE and a client answers one ahead. DFF and CRCEN "can only be
 * written when SPE is 0" (20.4.1), which is what makes apply() pay a
 * disable/enable pair when a request changes the frame size, and BR,
 * LSBFIRST, MSTR, CPOL and CPHA "cannot be modified during
 * communication".
 *
 * One of the chapter's sequences does not work here. 20.2.7's recipe
 * for clearing MODF - a STATR access then a CTLR1 write - leaves the
 * flag standing on this silicon under every variant the bench tried
 * (clear_mode_fault()'s own comment lists them), and the block's RCC
 * reset pulse is the way back. The verb reports rather than promises.
 *
 * The pads are a column (afio.hpp, tables 10-32 and 10-33). SPI1 has two:
 * the default PA4/PA5/PA6/PA7 and code 1's PA15/PB3/PB4/PB5. SPI2 has no
 * remap field - one column, PB12..PB15, from the datasheet's pin table.
 * SPI3 has two: the default PA15/PB3/PB4/PB5 - the very pads of SPI1's
 * second column - and code 1's PA4/PC10/PC11/PC12, behind PCFR1's bit 28,
 * which moves I2S3's pads with them. `SpiPins` carries the code and
 * init() writes it, refused where the part bonds no pad of it.
 *
 * The DMA requests are slots, a controller and one of its channels
 * (dma_engine.hpp, tables 11-2 to 11-5): SPI1 receives on DMA1's channel
 * 2 and transmits on 3, SPI2 on DMA1's 4 and 5, and SPI3 - and the I2S
 * face of SPI3, whose requests are the same two - on DMA2's 1 and 2. On
 * this family the channel is the request, so an engine slot naming any
 * other one is refused at compile time - it would move nothing. In sleep
 * the bus matrix serves the core alone (this
 * stratum's finding, docs/ch32vx03/README.md): a transport with engines
 * holds the program awake, exactly as usart.hpp's does.
 *
 * The high-speed read mode is a lot's story, and one code means the same
 * on every lot. HSCR.HSRXEN delays a host's sampling of MISO so that a
 * late answer is caught at the top rate. 20.4.10 tells it four ways by
 * class and lot: on the CH32V20x_D6 it "is only valid at clock division
 * 2" and the bit is write-only; on the CH32V20x_D8 the bit is write-only
 * and the mode valid at /2 alone on lots whose fifth digit from the end
 * is below 2, anywhere on the others; on the CH32V30x_D8 lots whose
 * penultimate sixth digit is not zero the bit reads back, BR takes a
 * second ladder under it (20.4.1: FPCLK/2, /3 ... /9) and a second mode,
 * HSRXEN2, exists; on that class's other lots the bit is write-only again
 * and the mode valid at /2 alone where the fifth digit is below 2. BR =
 * 000 is FPCLK/2 in both ladders, so the one code at
 * which HSRXEN means the same thing on every die of every class is /2 -
 * and `high_speed_read()` refuses at any other, at run time against the
 * code in the register and at compile time where the configuration is a
 * constant (`configure_high_speed_read<cfg>()`). HSRXEN2 is a verb that
 * asks the die (the CH32V303's, and only with SPI's bus at 120 MHz or
 * above, which only SPI1's PB2 reaches). What the mode buys is measured
 * (docs/ch32vx03/spi.md): on the CH32V303VCT6, SPI2 hosting SPI3 over
 * the evaluation board's wires at /2 - 36 MHz - read 254 bytes of 256
 * wrong without it, 253 of them the client's byte one bit late, and every
 * byte right with it; that die kept no HSRXEN2.
 *
 * The I2S face, the CH32V303RC's and VC's. The register file carries
 * SPIx_I2S_CFGR on every instance and SPIx_I2SPR on SPI2 and SPI3, and
 * chapter 20 is written for four families at once; what decides whether
 * a part has the face is its datasheet - table 2-1-1 gives two I2S to the
 * 256 KB CH32V303 and none to the 128 KB ones, and the CH32V203's table
 * 2-1 has no I2S row at all - so `I2s<n>` compiles where
 * `device::has_i2s` says and for SPI2 and SPI3 alone, and the SPI face
 * keeps `i2s_config_writable()`, the one question it asks of that
 * register elsewhere. The clock a master divides is SYSCLK: figure 3-3,
 * the simple tree this class has, draws SYSCLK to both I2S interfaces,
 * and RCC_CFGR2's I2S2SRC and I2S3SRC only choose between SYSCLK and a
 * PLL3 the CH32V30x_D8C has and this class has not - so `i2s_clock_hz()`
 * answers a static Clock's SYSCLK, and the divider pair is 20.3.3's
 * arithmetic against it (measured: 16015 frames a second counted against
 * the core for the 16014 that arithmetic gives at 144 MHz). The face
 * shares the vector, the gate, the reset, CTLR2's enables and the two DMA
 * requests with the SPI face; CTLR1, the CRC and MODF are not used in
 * it. One sentence of 20.3.6.2 does not hold: TXE reads 1 on a configured
 * transmitting face with I2SE clear, and a datum written there is the
 * first word on the wire once the face is enabled (measured on the
 * CH32V303VCT6) - so nothing here waits for TXE to rise at the enable,
 * and a program may load its first word before it.
 */

#pragma once

#include <stdint.h>

#include <bit>
#include <optional>
#include <span>
#include <type_traits>

#include "ch32vx03/afio.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/dma_engine.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// =============================================================================
// The registers (tables 20-1 and 20-2): sixteen bits at a four-byte stride
// =============================================================================

struct SpiRegs {
    volatile uint16_t CTLR1;    ///< 0x00
    uint16_t RESERVED0;
    volatile uint16_t CTLR2;    ///< 0x04
    uint16_t RESERVED1;
    volatile uint16_t STATR;    ///< 0x08
    uint16_t RESERVED2;
    volatile uint16_t DATAR;    ///< 0x0c
    uint16_t RESERVED3;
    volatile uint16_t CRCR;     ///< 0x10 the polynomial
    uint16_t RESERVED4;
    volatile uint16_t RCRCR;    ///< 0x14 receive CRC, read-only
    uint16_t RESERVED5;
    volatile uint16_t TCRCR;    ///< 0x18 transmit CRC, read-only
    uint16_t RESERVED6;
    volatile uint16_t I2SCFGR;  ///< 0x1c the I2S face (see the file header)
    uint16_t RESERVED7;
    volatile uint16_t I2SPR;    ///< 0x20 the I2S prescaler, SPI2's alone
    uint16_t RESERVED8;
    volatile uint16_t HSCR;     ///< 0x24 high-speed control
    uint16_t RESERVED9;
};

/// The three instances this family addresses (tables 20-1 to 20-3):
/// SPI1 on PB2, SPI2 and SPI3 on PB1 - device.hpp's spi3_base.
constexpr uint32_t spi_base_for(uint8_t n) {
    return n == 1 ? pb2_base + 0x3000 : n == 2 ? pb1_base + 0x3800 : n == 3 ? spi3_base : 0;
}

/// Which bus an instance hangs on, which is both its gate register and
/// the clock its BR field divides.
constexpr Bus spi_bus_for(uint8_t n) { return n == 1 ? Bus::pb2 : Bus::pb1; }

constexpr uint32_t spi_gate_for(uint8_t n) {
    return n == 1 ? rcc_pb2_spi1 : n == 2 ? rcc_pb1_spi2 : n == 3 ? rcc_pb1_spi3 : 0;
}

/// The instance's vector - SPI3's is entry 67 of the CH32V303's own tail,
/// and irq_none on a class that has no such line.
constexpr Irq spi_irq_for(uint8_t n) {
    return n == 1 ? Irq::spi1 : n == 2 ? Irq::spi2 : Irq::spi3;
}

/// Does THIS PART have that instance? The part table's mask
/// (device::spi_instances, from the datasheets' tables 2-1 and 2-1-1),
/// never a count read as "the first n".
constexpr bool spi_present(uint8_t n) {
    return n >= 1u && n <= 3u && (device::spi_instances & static_cast<uint16_t>(1U << n)) != 0u;
}

/// The two rows per instance (tables 11-2 to 11-5), read through
/// dma_engine.hpp so that the slots live in exactly one place - SPI3's
/// are DMA2's - and the channel numbers of those slots, which is what a
/// message prints.
constexpr DmaSlot spi_dma_rx_slot(uint8_t n) {
    return dma_request_channel(n == 1 ? DmaRequest::spi1_rx
                               : n == 2 ? DmaRequest::spi2_rx : DmaRequest::spi3_rx);
}
constexpr DmaSlot spi_dma_tx_slot(uint8_t n) {
    return dma_request_channel(n == 1 ? DmaRequest::spi1_tx
                               : n == 2 ? DmaRequest::spi2_tx : DmaRequest::spi3_tx);
}
constexpr uint8_t spi_dma_rx_channel(uint8_t n) { return spi_dma_rx_slot(n).channel; }
constexpr uint8_t spi_dma_tx_channel(uint8_t n) { return spi_dma_tx_slot(n).channel; }

// CTLR1 (20.4.1)
inline constexpr uint16_t spi_cpha     = 1u << 0;
inline constexpr uint16_t spi_cpol     = 1u << 1;
inline constexpr uint16_t spi_mstr     = 1u << 2;
inline constexpr uint16_t spi_br_mask  = 7u << 3;
inline constexpr uint16_t spi_br_shift = 3;
inline constexpr uint16_t spi_spe      = 1u << 6;
inline constexpr uint16_t spi_lsbfirst = 1u << 7;
inline constexpr uint16_t spi_ssi      = 1u << 8;
inline constexpr uint16_t spi_ssm      = 1u << 9;
inline constexpr uint16_t spi_rxonly   = 1u << 10;
inline constexpr uint16_t spi_dff      = 1u << 11;
inline constexpr uint16_t spi_crcnext  = 1u << 12;
inline constexpr uint16_t spi_crcen    = 1u << 13;
inline constexpr uint16_t spi_bidioe   = 1u << 14;
inline constexpr uint16_t spi_bidimode = 1u << 15;
// CTLR2 (20.4.2)
inline constexpr uint16_t spi_rxdmaen  = 1u << 0;
inline constexpr uint16_t spi_txdmaen  = 1u << 1;
inline constexpr uint16_t spi_ssoe     = 1u << 2;
inline constexpr uint16_t spi_errie    = 1u << 5;
inline constexpr uint16_t spi_rxneie   = 1u << 6;
inline constexpr uint16_t spi_txeie    = 1u << 7;
/// The three enables as one field, so a verb that rewrites CTLR2 keeps
/// whatever the caller had armed.
inline constexpr uint16_t spi_ctlr2_enables = spi_errie | spi_rxneie | spi_txeie;
// STATR (20.4.3). UDR and CHSID belong to the I2S face and are named
// here because the register has them, not because this driver uses them.
inline constexpr uint16_t spi_rxne     = 1u << 0;
inline constexpr uint16_t spi_txe      = 1u << 1;
inline constexpr uint16_t spi_chside   = 1u << 2;
inline constexpr uint16_t spi_udr      = 1u << 3;
inline constexpr uint16_t spi_crcerr   = 1u << 4;   ///< the one rc_w0 flag of the register
inline constexpr uint16_t spi_modf     = 1u << 5;
inline constexpr uint16_t spi_ovr      = 1u << 6;
inline constexpr uint16_t spi_bsy      = 1u << 7;
// HSCR (20.4.10). HSRXEN is every class's; HSRXEN2 is the CH32V30x_D8's,
// and there a lot's (the file header).
inline constexpr uint16_t spi_hsrxen   = 1u << 0;
inline constexpr uint16_t spi_hsrxen2  = 1u << 2;
/// HSRXEN2 "is used only when ... SPI PCLK is greater than or equal to
/// 120M" (20.4.10): the bus rate below which its verb refuses.
inline constexpr uint32_t spi_hsrxen2_min_bus_hz = 120'000'000UL;
// SPIx_I2S_CFGR (20.4.8): the face's whole configuration, and I2SMOD the
// one bit the SPI face writes, only to ask the silicon whether the
// register answers.
inline constexpr uint16_t spi_i2s_chlen        = 1u << 0;
inline constexpr uint16_t spi_i2s_datlen_shift = 1;
inline constexpr uint16_t spi_i2s_datlen_mask  = 3u << 1;
inline constexpr uint16_t spi_i2s_ckpol        = 1u << 3;
inline constexpr uint16_t spi_i2s_std_shift    = 4;
inline constexpr uint16_t spi_i2s_std_mask     = 3u << 4;
inline constexpr uint16_t spi_i2s_pcmsync      = 1u << 7;
inline constexpr uint16_t spi_i2s_cfg_shift    = 8;
inline constexpr uint16_t spi_i2s_cfg_mask     = 3u << 8;
inline constexpr uint16_t spi_i2se             = 1u << 10;
inline constexpr uint16_t spi_i2smod           = 1u << 11;
// SPIx_I2SPR (20.4.9), SPI2's and SPI3's: the linear divider, ODD and the
// master clock's output enable.
inline constexpr uint16_t spi_i2s_div_mask = 0x00FFu;
inline constexpr uint16_t spi_i2s_odd      = 1u << 8;
inline constexpr uint16_t spi_i2s_mckoe    = 1u << 9;

// =============================================================================
// The vocabulary
// =============================================================================

/// The flags STATR raises, as the ISR body hands them back.
struct SpiFlag {
    static constexpr uint32_t rxne = spi_rxne;
    static constexpr uint32_t txe = spi_txe;
    static constexpr uint32_t crc_error = spi_crcerr;
    static constexpr uint32_t mode_fault = spi_modf;
    static constexpr uint32_t overrun = spi_ovr;
    static constexpr uint32_t busy = spi_bsy;
    /// The three sources ERRIE gates in the SPI face (20.2.8).
    static constexpr uint32_t errors = spi_crcerr | spi_modf | spi_ovr;
    /// The I2S face's own two: a client transmitter clocked before its
    /// data register was written, and the side the datum belongs to.
    static constexpr uint32_t underrun = spi_udr;
    static constexpr uint32_t channel_side = spi_chside;
    /// The two sources ERRIE gates in the I2S face (20.3.8).
    static constexpr uint32_t i2s_errors = spi_ovr | spi_udr;
};

/// CPOL and CPHA as the four Motorola modes (20.2.1).
enum class SpiMode : uint8_t { mode0 = 0, mode1 = 1, mode2 = 2, mode3 = 3 };

constexpr bool spi_mode_cpol(SpiMode m) { return (static_cast<uint8_t>(m) & 2u) != 0u; }
constexpr bool spi_mode_cpha(SpiMode m) { return (static_cast<uint8_t>(m) & 1u) != 0u; }

/// BR[2:0]: the INSTANCE'S OWN bus clock over two to the code plus one.
enum class SpiClock : uint8_t {
    div2 = 0, div4 = 1, div8 = 2, div16 = 3, div32 = 4, div64 = 5, div128 = 6, div256 = 7,
};

constexpr uint16_t spi_division(SpiClock c) {
    return static_cast<uint16_t>(2u << static_cast<uint8_t>(c));
}
constexpr uint32_t spi_sck_hz(uint32_t pclk, SpiClock c) {
    return pclk >> (static_cast<uint8_t>(c) + 1u);
}

/// The coarsest code whose SCK does not exceed `max_hz` on a bus at
/// `pclk`; nullopt when even /256 does - refused, never rounded up.
constexpr std::optional<SpiClock> spi_rate_for(uint32_t pclk, uint32_t max_hz) {
    if (pclk == 0u || max_hz == 0u) {
        return {};
    }
    for (uint8_t code = 0; code < 8u; ++code) {
        if ((pclk >> (code + 1u)) <= max_hz) {
            return static_cast<SpiClock>(code);
        }
    }
    return {};
}

/**
 * The BR code for a ceiling as a COMPILE-TIME CONSTANT, refused where
 * the bus cannot make it:
 *
 *     constexpr auto rate = brio::SpiRateOf<SysClock::pclk1_hz, 4'000'000>::clock;
 *
 * which is how a program states a device's datasheet limit and gets the
 * register's own vocabulary back. The runtime chooser spi_rate_for()
 * only REPORTS the same refusal, as an empty optional.
 */
template <uint32_t pclk, uint32_t max_hz>
struct SpiRateOf {
    static_assert(spi_rate_for(pclk, max_hz).has_value(),
                  "brio SPI: this bus cannot produce an SCK at or below that ceiling - even "
                  "its slowest code (the bus clock over 256) is faster, and a rate is never "
                  "rounded up");
    static constexpr SpiClock clock = *spi_rate_for(pclk, max_hz);
};

/// DFF: the two frame sizes this chapter has.
enum class SpiDataSize : uint8_t { bits8 = 0, bits16 = 1 };

constexpr bool spi_frame_is_halfword(SpiDataSize s) { return s == SpiDataSize::bits16; }
constexpr uint8_t spi_data_bits(SpiDataSize s) { return s == SpiDataSize::bits16 ? 16u : 8u; }
constexpr uint16_t spi_frame_mask(SpiDataSize s) {
    return s == SpiDataSize::bits16 ? 0xFFFFu : 0x00FFu;
}

enum class SpiRole : uint8_t { client = 0, host = 1 };

/// The chip-select arrangements of 20.2.1: `software` is SSM/SSI (the
/// pad free, the engine's select an ordinary GPIO); `hardware_output` is
/// SSOE, the pad driven low by the host while SPE is set;
/// `hardware_input` is the multi-host input whose low level raises MODF
/// and demotes the host - and is what frames a CLIENT's transaction.
enum class SpiNss : uint8_t { software, hardware_output, hardware_input };

/// 20.2.4's line arrangements.
enum class SpiDirection : uint8_t {
    full_duplex,       ///< two data lines, both ways
    receive_only,      ///< RXONLY: two lines, the output one released
    half_duplex_out,   ///< BIDIMODE + BIDIOE: one line, transmitting
    half_duplex_in,    ///< BIDIMODE, BIDIOE clear: one line, receiving
};

/// Which pads carry the four signals, and the REMAP CODE that puts the
/// peripheral on them (afio.hpp, table 10-32): init() writes the code.
/// No alternate-function number - a pad has the one function its column
/// gives it. A signal the program does not wire is an invalid Pad.
struct SpiPins {
    Pad nss{};
    Pad sck{};
    Pad miso{};
    Pad mosi{};
    uint8_t remap = 0;
};

/// How many columns an instance has: SPI1's two, SPI2's one (no remap
/// field exists for it) and SPI3's two (table 10-33).
constexpr uint8_t spi_column_count(uint8_t n) {
    return n == 1 ? afio_spi1_codes : n == 3 ? afio_spi3_codes : 1u;
}

/// The pads of column `code` on that instance, every signal named.
constexpr SpiPins spi_pins_for(uint8_t n, uint8_t code) {
    const SpiPadSet p = n == 1   ? afio_spi1_pads(code)
                        : n == 3 ? afio_spi3_pads(code)
                                 : afio_spi2_pad_set;
    return SpiPins{.nss = p.nss, .sck = p.sck, .miso = p.miso, .mosi = p.mosi,
                   .remap = static_cast<uint8_t>(n == 2 ? 0u : code)};
}

/// The reset column of each instance, which is what a program that has
/// not chosen its pads gets.
inline constexpr SpiPins spi1_default_pins = spi_pins_for(1, 0);
inline constexpr SpiPins spi2_default_pins = spi_pins_for(2, 0);
inline constexpr SpiPins spi3_default_pins = spi_pins_for(3, 0);

/// The default for `SpiHost<n>` and `SpiClient<n>`: each instance's own
/// first column, chosen by the instance rather than by SPI1's.
template <uint8_t n>
inline constexpr SpiPins spi_default_pins = spi_pins_for(n, 0);

/**
 * Is that column usable on this part, for that instance? A link needs
 * SCK, no two signals may name one pad, the code must be a column the
 * instance has, and every pad the program NAMES must be one the package
 * brings out (afio.hpp's tables are the series'; the bonding is the
 * part's).
 */
constexpr bool spi_pins_valid(uint8_t n, const SpiPins& p) {
    if (!p.sck.valid() || p.remap >= spi_column_count(n)) {
        return false;
    }
    const Pad pads[] = {p.nss, p.sck, p.miso, p.mosi};
    for (uint8_t i = 0; i < 4u; ++i) {
        if (pads[i].valid() && !pad_bonded(pads[i])) {
            return false;
        }
        for (uint8_t j = static_cast<uint8_t>(i + 1u); j < 4u; ++j) {
            if (pads[i].valid() && pads[j].valid() && pads[i] == pads[j]) {
                return false;
            }
        }
    }
    return true;
}

struct SpiConfig {
    SpiRole role = SpiRole::host;
    SpiMode mode = SpiMode::mode0;
    SpiClock clock = SpiClock::div16;
    SpiDataSize bits = SpiDataSize::bits8;
    bool lsb_first = false;
    SpiNss nss = SpiNss::software;
    SpiDirection direction = SpiDirection::full_duplex;
    bool crc = false;
    uint16_t crc_polynomial = 0x0007u;
    bool dma_transmit = false;
    bool dma_receive = false;
};

/// The refusals the chapter owes: every field must hold a value its
/// register FIELD encodes (a cast-in frame width or BR code would spill
/// into its neighbours); CRC "can only be used in full duplex mode"
/// (20.4.1); a polynomial of zero computes nothing; and SSOE is a host
/// verb ("Disable SS output in master mode") that a client has no use
/// for.
constexpr bool spi_config_valid(const SpiConfig& c) {
    if (static_cast<uint8_t>(c.role) > 1u || static_cast<uint8_t>(c.mode) > 3u ||
        static_cast<uint8_t>(c.clock) > 7u || static_cast<uint8_t>(c.bits) > 1u ||
        static_cast<uint8_t>(c.nss) > 2u || static_cast<uint8_t>(c.direction) > 3u) {
        return false;
    }
    if (c.crc && (c.direction != SpiDirection::full_duplex || c.crc_polynomial == 0u)) {
        return false;
    }
    if (c.role == SpiRole::client && c.nss == SpiNss::hardware_output) {
        return false;
    }
    return true;
}

/// What CTLR1 holds for a configuration, SPE clear.
constexpr uint16_t spi_ctlr1_of(const SpiConfig& c) {
    uint16_t v = 0;
    if (spi_mode_cpha(c.mode)) { v |= spi_cpha; }
    if (spi_mode_cpol(c.mode)) { v |= spi_cpol; }
    if (c.role == SpiRole::host) { v |= spi_mstr; }
    v |= static_cast<uint16_t>(static_cast<uint16_t>(c.clock) << spi_br_shift);
    if (c.lsb_first) { v |= spi_lsbfirst; }
    if (c.nss == SpiNss::software) {
        v |= spi_ssm;
        if (c.role == SpiRole::host) { v |= spi_ssi; }   // a client under SSM keeps SSI low
    }
    if (c.bits == SpiDataSize::bits16) { v |= spi_dff; }
    if (c.crc) { v |= spi_crcen; }
    switch (c.direction) {
        case SpiDirection::full_duplex:     break;
        case SpiDirection::receive_only:    v |= spi_rxonly; break;
        case SpiDirection::half_duplex_out: v |= spi_bidimode | spi_bidioe; break;
        case SpiDirection::half_duplex_in:  v |= spi_bidimode; break;
    }
    return v;
}

/// What CTLR2 holds for a configuration, the interrupt enables clear.
constexpr uint16_t spi_ctlr2_of(const SpiConfig& c) {
    uint16_t v = 0;
    if (c.nss == SpiNss::hardware_output) { v |= spi_ssoe; }
    if (c.dma_transmit) { v |= spi_txdmaen; }
    if (c.dma_receive) { v |= spi_rxdmaen; }
    return v;
}

/// The rate of the bus an INSTANCE hangs on, out of a static or a
/// dynamic clock - the one number its BR field divides.
template <uint8_t instance, typename C>
constexpr uint32_t spi_bus_hz(C clock) {
    if constexpr (C::is_static) {
        (void)clock;
        return instance == 1u ? C::pclk2_hz : C::pclk1_hz;
    } else {
        (void)clock;
        return instance == 1u ? C::pclk2_hz() : C::pclk1_hz();
    }
}

/// The same answer from an HCLK, which is what a dynamic clock hands a
/// user it is about to rebase (the prescalers still hold the rate being
/// left, so this is arithmetic and not a read).
template <uint8_t instance>
constexpr uint32_t spi_bus_hz_at(uint32_t hclk) {
    return instance == 1u ? pclk2_hz_at(hclk) : pclk1_hz_at(hclk);
}

// =============================================================================
// The resource
// =============================================================================

/**
 * Spi<n>: the register block and its verbs.
 *
 *   using Bus = brio::Spi<2>;
 *   Bus::bus_clock(true);
 *   Bus::reset();
 *   Bus::configure({.role = brio::SpiRole::host, .clock = brio::SpiClock::div16});
 *   Bus::enable();
 *
 * `configure()` REFUSES what spi_config_valid() refuses and writes the
 * two control words with SPE clear (DFF and CRCEN are enable-protected,
 * 20.4.1); the interrupt enables are kept as they stand. The ISR body
 * returns the raised-and-enabled sources for the caller to act on; it
 * consumes nothing.
 */
template <uint8_t n>
struct Spi {
    static_assert(spi_base_for(n) != 0, "brio Spi: this family has SPI1, SPI2 and SPI3");
    static_assert(spi_present(n),
                  "brio Spi: this part does not offer that instance - the parts below the "
                  "CH32V203C8 have SPI1 alone, and SPI3 is the CH32V303RC's and VC's (the "
                  "datasheets' tables 2-1 and 2-1-1, parts/<part>.hpp)");

    Spi() = delete;

    static constexpr uint8_t number = n;
    static constexpr Bus bus = spi_bus_for(n);
    static constexpr Irq irq = spi_irq_for(n);
    /// Tables 11-2 to 11-5: the two slots this instance's requests reach.
    static constexpr DmaSlot dma_rx_slot = spi_dma_rx_slot(n);
    static constexpr DmaSlot dma_tx_slot = spi_dma_tx_slot(n);
    static constexpr uint8_t dma_rx_channel = dma_rx_slot.channel;
    static constexpr uint8_t dma_tx_channel = dma_tx_slot.channel;
    /// SPI2 and SPI3 carry the I2S prescaler (tables 20-2 and 20-3 have
    /// it, 20-1 has not), which is what a master I2S needs; whether the
    /// PART has the I2S face at all is device::has_i2s.
    static constexpr bool has_i2s_prescaler = (n != 1);

    static SpiRegs& regs() { return *reinterpret_cast<SpiRegs*>(spi_base_for(n)); }
    static volatile void* data_address() { return &regs().DATAR; }

    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(bus, spi_gate_for(n));
        } else {
            Rcc::disable(bus, spi_gate_for(n));
        }
    }
    /// The RCC reset pulse: every register back to its reset value.
    static void reset() { Rcc::reset(bus, spi_gate_for(n)); }

    /// The instance's own bus rate out of the application's clock tag.
    template <typename C>
    static constexpr uint32_t bus_hz(C clock) {
        return spi_bus_hz<n, C>(clock);
    }

    // ---- configuration ------------------------------------------------------

    /// The whole configuration, with SPE clear on the way in. Refused by
    /// the chapter's rules; the interrupt enables are kept as they are.
    static bool configure(const SpiConfig& c) {
        if (!spi_config_valid(c)) {
            return false;
        }
        write_config(c);
        return true;
    }

    /// The same where the configuration is a CONSTANT: what configure()
    /// reports as false is a compile error here, on the line that wrote
    /// the value.
    template <SpiConfig c>
    static void configure() {
        static_assert(spi_config_valid(c),
                      "brio Spi: this configuration is not one chapter 20 has - a field holds "
                      "a value its register field does not encode, or the CRC is asked for "
                      "outside full duplex or with a zero polynomial, or a CLIENT is asked "
                      "for the NSS OUTPUT, which is a host's verb (20.4.2)");
        write_config(c);
    }

    /// Put the column this instance's pads come from into AFIO. SPI2 has
    /// no remap field, so only code 0 is accepted for it; SPI3's field
    /// (PCFR1 bit 28) moves I2S3's pads with its own.
    static bool remap(uint8_t code) {
        if constexpr (n == 1) {
            return Afio::remap(Remap::spi1, code);
        } else if constexpr (n == 3) {
            return Afio::remap(Remap::spi3, code);
        } else {
            return code == 0u;
        }
    }

    static void enable() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_spe); }
    static bool enabled() { return (regs().CTLR1 & spi_spe) != 0u; }

    /**
     * The drain-then-disable procedure. Clearing SPE while a frame is in
     * flight truncates it on the wire, so this waits for the buffer
     * empty (TXE) and the shifter idle (not BSY) before dropping the
     * bit. Each wait is bounded; false says one ran out and the bit was
     * dropped regardless.
     *
     * A HOST IN RECEIVE-ONLY OR HALF-DUPLEX INPUT NEVER STOPS BY ITSELF:
     * its clock runs as long as SPE stands, so BSY is the wrong thing to
     * wait for there and `disable()` drops the bit at once (the caller
     * stops such a host between frames - see `stop_receive_only()`).
     */
    static bool disable() {
        bool ok = true;
        if (enabled() && !receive_only_host()) {
            ok = wait_until([] { return txe(); }) && wait_until([] { return !busy(); });
        }
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_spe);
        return ok;
    }

    /// Is this a host whose clock free-runs (RXONLY or BIDIMODE with
    /// BIDIOE clear)? The one configuration whose disable rule differs.
    static bool receive_only_host() {
        const uint16_t c = regs().CTLR1;
        if ((c & spi_mstr) == 0u) {
            return false;
        }
        if ((c & spi_bidimode) != 0u) {
            return (c & spi_bidioe) == 0u;
        }
        return (c & spi_rxonly) != 0u;
    }

    /**
     * Stop a receive-only host cleanly: SPE is dropped in the window
     * between the second-to-last frame's last clock edge and the last
     * frame's first, and the last frame is then read out. The sequence
     * is the F1 lineage's and this chapter does not spell it, so the
     * bench's own finding is what docs/ch32vx03/spi.md reports; the verb
     * returns the last frame when one was there.
     */
    static std::optional<uint16_t> stop_receive_only(SpiDataSize bits) {
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_spe);
        std::optional<uint16_t> last{};
        if (wait_until([] { return rxne(); })) {
            last = data(bits);
        }
        (void)wait_until([] { return !busy(); });
        return last;
    }

    // ---- data ---------------------------------------------------------------

    /// Write one frame; a 16-bit store lands the low byte in 8-bit mode.
    static void data(SpiDataSize, uint16_t v) { regs().DATAR = v; }
    /// Read one frame, which is also what clears RXNE.
    static uint16_t data(SpiDataSize bits) {
        return static_cast<uint16_t>(regs().DATAR & spi_frame_mask(bits));
    }
    static void data8(uint8_t v) { regs().DATAR = v; }
    static uint8_t data8() { return static_cast<uint8_t>(regs().DATAR); }

    static bool rxne() { return (regs().STATR & spi_rxne) != 0u; }
    static bool txe() { return (regs().STATR & spi_txe) != 0u; }
    static bool busy() { return (regs().STATR & spi_bsy) != 0u; }
    static uint16_t status() { return regs().STATR; }

    /// What the block is configured as right now - the two questions a
    /// console asks of a live instance.
    static SpiRole role() {
        return (regs().CTLR1 & spi_mstr) != 0u ? SpiRole::host : SpiRole::client;
    }
    static SpiMode mode() {
        const uint16_t c = regs().CTLR1;
        return static_cast<SpiMode>(((c & spi_cpol) != 0u ? 2u : 0u) |
                                    ((c & spi_cpha) != 0u ? 1u : 0u));
    }
    static SpiClock clock() {
        return static_cast<SpiClock>((regs().CTLR1 & spi_br_mask) >> spi_br_shift);
    }
    static SpiDataSize bits() {
        return (regs().CTLR1 & spi_dff) != 0u ? SpiDataSize::bits16 : SpiDataSize::bits8;
    }

    /// Throw away whatever the receive buffer holds and the OVR it may
    /// have raised (the DATAR-then-STATR sequence, 20.2.7).
    static void flush_rx() {
        (void)regs().DATAR;
        (void)regs().STATR;
    }

    // ---- errors (20.2.7) ----------------------------------------------------

    static bool overrun() { return (regs().STATR & spi_ovr) != 0u; }
    /// "Reading the data register followed by the status register will
    /// clear this bit" - DATAR then STATR, in that order. Measured so on both
    /// parts: that STATR load still reads the flag up, the load after it down.
    static void clear_overrun() {
        (void)regs().DATAR;
        (void)regs().STATR;
    }
    static bool mode_fault() { return (regs().STATR & spi_modf) != 0u; }
    /**
     * 20.2.7's sequence: "first perform a read or write operation to
     * STATR, and then write to CTLR1".
     *
     * MEASURED ON THIS SILICON: IT DOES NOT PUT THE FLAG DOWN. Five
     * variants of those two steps were tried and all leave MODF
     * standing - the CTLR1 write rewriting the register as it stands,
     * the write that RAISES SPE (which is the shape the vendor's own
     * library documents, and the one the chapter's words leave open),
     * the write that restores SSI, the sequence with SSI raised BEFORE
     * it, and a STATR WRITE in place of the read. The only way back
     * measured is the block's RCC reset pulse.
     *
     * So the verb runs the chapter's sequence and REPORTS whether the
     * flag went down, rather than promising what the words say. A host
     * that has been demoted has lost SPE and MSTR with the flag and must
     * be reconfigured anyway: reset() then configure() is that path, and
     * SpiHost's reset_block() is it written out.
     */
    static bool clear_mode_fault() {
        (void)regs().STATR;
        regs().CTLR1 = regs().CTLR1;
        return !mode_fault();
    }
    static bool crc_error() { return (regs().STATR & spi_crcerr) != 0u; }
    /// The one rc_w0 flag of the register: a zero into its bit, ones
    /// everywhere else.
    static void clear_crc_error() { regs().STATR = static_cast<uint16_t>(~spi_crcerr); }

    // ---- CRC (20.2.5) -------------------------------------------------------

    /// Send the transmit CRC after the frame just written. "This bit
    /// should be set immediately after writing the last data."
    static void crc_next() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_crcnext); }
    /// The polynomial, writable with SPE clear like CRCEN itself.
    static void crc_polynomial(uint16_t poly) { regs().CRCR = poly; }
    static uint16_t crc_polynomial() { return regs().CRCR; }
    /// "This register needs to be read when BSY is 0."
    static uint16_t rx_crc() { return regs().RCRCR; }
    static uint16_t tx_crc() { return regs().TCRCR; }

    // ---- the chip select and the rest --------------------------------------

    /// SSI, for a configuration under SSM: high is "not selected" on a
    /// host (and what keeps it a host), low selects a client.
    static void software_select(bool selected) {
        if (selected) {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_ssi);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_ssi);
        }
    }

    /// BIDIOE, for a half-duplex link that turns around between phases:
    /// true drives the one data line, false listens on it.
    static void half_duplex_output(bool on) {
        if (on) {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_bidioe);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_bidioe);
        }
    }

    /**
     * HSCR.HSRXEN, the high-speed read mode: a HOST's sampling of MISO
     * moved late enough to catch an answer at the top rate. The verb
     * REFUSES at any BR code but /2 - the one code whose meaning no lot
     * of any class changes (the file header) - rather than arming a mode
     * a die may ignore or read on another ladder. The bit is write-only
     * on every die but the CH32V303 lots that also have HSRXEN2, so
     * nothing reads it back, and HSCR is written WHOLE: HSRXEN2 goes with
     * the mode it adds to. The mode outlives a reconfiguration - HSCR is
     * not CTLR1 - so a host that moves off /2 turns it off first.
     */
    static bool high_speed_read(bool on) {
        if (on && clock() != SpiClock::div2) {
            return false;
        }
        regs().HSCR = on ? spi_hsrxen : uint16_t{0};
        return true;
    }

    /// The whole configuration AND the high-speed read, where the
    /// configuration is a CONSTANT: a BR code other than /2, or a client
    /// (the mode is a host's reading), is a compile error on the line
    /// that wrote the value - configure<cfg>()'s refusals besides.
    template <SpiConfig c>
    static void configure_high_speed_read() {
        static_assert(spi_config_valid(c),
                      "brio Spi: this configuration is not one chapter 20 has (see configure<cfg>())");
        static_assert(c.clock == SpiClock::div2,
                      "brio Spi: the high-speed read mode means the same thing on every lot at BR "
                      "= /2 alone (20.4.10: the CH32V20x_D6's only valid code, and the one the "
                      "CH32V30x_D8's second ladder leaves where it was)");
        static_assert(c.role == SpiRole::host,
                      "brio Spi: the high-speed read mode moves a HOST's sampling of MISO "
                      "(20.4.10) - a client samples on its host's clock");
        write_config(c);
        regs().HSCR = spi_hsrxen;
    }

    /**
     * HSCR.HSRXEN2, the second high-speed read mode: the CH32V30x_D8's,
     * on the lots 20.4.10's note names, "used only when HSRXEN is turned
     * on and SPI PCLK is greater than or equal to 120M". So the verb takes
     * the instance's bus rate, refuses below 120 MHz - which leaves it to
     * SPI1, PB1 being capped at 72 - and at a BR code other than /2, and
     * writes the two modes TOGETHER, mode 2 being an addition to mode 1.
     * It then ASKS THE DIE, the way this stratum asks for a lot's bit: the
     * word is read FIRST - HSCR holds nothing outside its two bits, so a
     * word with any other bit set is another register's answer (an
     * address that mirrors another has been met on this die) and the
     * answer is no with nothing written - then the two bits are written
     * and the word must come back as exactly those two, which is what the
     * lots that have HSRXEN2 give (HSRXEN reads back there too). A die
     * that did not keep them is left at HSRXEN alone and the answer is
     * false. Refused at compile time on the CH32V203's classes.
     */
    static bool high_speed_read2(bool on, uint32_t bus_hz) {
        static_assert(device::device_class == DeviceClass::v30x_d8,
                      "brio Spi: HSRXEN2 is the CH32V30x_D8's (20.4.10's note) - no CH32V203 has "
                      "it");
        if (!on) {
            regs().HSCR = spi_hsrxen;
            return true;
        }
        if (bus_hz < spi_hsrxen2_min_bus_hz || clock() != SpiClock::div2) {
            return false;
        }
        constexpr uint16_t both = static_cast<uint16_t>(spi_hsrxen | spi_hsrxen2);
        if ((regs().HSCR & static_cast<uint16_t>(~both)) != 0u) {
            return false;   // the read path is not HSCR's own
        }
        regs().HSCR = both;
        if (regs().HSCR != both) {
            regs().HSCR = spi_hsrxen;
            return false;   // this die's lot has no HSRXEN2
        }
        return true;
    }

    /**
     * Does SPI_I2S_CFGR's I2SMOD take a write on this silicon? On a part
     * the datasheet gives no I2S (the file header) this is the one
     * question the driver asks of that register; where the face is the
     * part's, `I2s<n>` is its driver. The bit is set, read back and put
     * away, with SPE clear as 20.4.8 requires. It moves no pad and starts
     * nothing.
     */
    static bool i2s_config_writable() {
        const uint16_t saved = regs().I2SCFGR;
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_spe);
        regs().I2SCFGR = static_cast<uint16_t>(saved | spi_i2smod);
        const bool took = (regs().I2SCFGR & spi_i2smod) != 0u;
        regs().I2SCFGR = saved;
        return took;
    }

    static void dma_requests(bool tx, bool rx) {
        uint16_t v = static_cast<uint16_t>(regs().CTLR2 & ~(spi_txdmaen | spi_rxdmaen));
        if (tx) { v |= spi_txdmaen; }
        if (rx) { v |= spi_rxdmaen; }
        regs().CTLR2 = v;
    }

    // ---- interrupts (20.2.8) ------------------------------------------------

    static void rxne_interrupt(bool on) { interrupt(spi_rxneie, on); }
    static void txe_interrupt(bool on) { interrupt(spi_txeie, on); }
    static void error_interrupt(bool on) { interrupt(spi_errie, on); }

    /// The raised sources that are ENABLED, as SpiFlag bits for the
    /// program's handler to act on: RXNE and TXE under their own enables,
    /// the three errors under ERRIE. Nothing is cleared here; two loads,
    /// STATR and CTLR2, from the vector the program binds.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint16_t st = regs().STATR;
        const uint16_t en = regs().CTLR2;
        uint32_t up = 0;
        if ((en & spi_rxneie) != 0u && (st & spi_rxne) != 0u) { up |= SpiFlag::rxne; }
        if ((en & spi_txeie) != 0u && (st & spi_txe) != 0u) { up |= SpiFlag::txe; }
        if ((en & spi_errie) != 0u) {
            up |= st & SpiFlag::errors;
        }
        return up;
    }

private:
    /// The two control words, SPE clear on the way in (DFF and CRCEN are
    /// enable-protected, 20.4.1).
    static void write_config(const SpiConfig& c) {
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_spe);
        if (c.crc) {
            regs().CRCR = c.crc_polynomial;
        }
        regs().CTLR1 = spi_ctlr1_of(c);
        regs().CTLR2 =
            static_cast<uint16_t>((regs().CTLR2 & spi_ctlr2_enables) | spi_ctlr2_of(c));
    }

    static void interrupt(uint16_t bit, bool on) {
        if (on) {
            regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 | bit);
        } else {
            regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 & ~bit);
        }
    }

    /// The bound of wait_until(), derived from the slowest frame the chapter
    /// allows: sixteen bits at BR's /256 are 4096 cycles of the block's
    /// bus, and the slowest bus is HCLK over sixteen (RCC's PPRE fields),
    /// so 65536 core cycles; twice that, counted in polls of at least one
    /// core cycle each, so a wait the wire can end never ends before it -
    /// about 0.9 ms at 144 MHz for the polls alone, a few times that where
    /// a poll takes a few cycles.
    static constexpr uint32_t wait_polls = 2u * 16u * 256u * 16u;

    /// A bounded wait: true when the condition came, false when the
    /// budget ran out (a bus whose clock never runs must not hang the
    /// kernel): wait_polls polls.
    template <typename Pred>
    static bool wait_until(Pred pred) {
        for (uint32_t spins = wait_polls; spins != 0u; --spins) {
            if (pred()) {
                return true;
            }
        }
        return false;
    }
};

// =============================================================================
// The host engine
// =============================================================================

/// A block the engines could not finish, a transfer error on either DMA
/// channel: the engines' own status code, in the range util/bus_master.hpp
/// leaves to engines.
inline constexpr uint8_t spi_dma_fault = bus_engine_status;
// The two other codes this host answers are the vocabulary's
// (util/spi_bus.hpp). spi_stalled: a polled frame's flag never came within
// the transaction's one spin budget - a clock that stopped, a block that is
// dead; the select and the D/C are released and the block reset and
// reconfigured before start() returns. spi_rejected: a request naming
// SpiEngine::dma that the engines cannot run or on a host with none, or any
// request while this host's own block is still on the engines, refused
// before anything reaches the wire.

/**
 * SpiHost<n, pins, TxEngine, RxEngine>
 *
 * The engine SpiBus (util/spi_bus.hpp = BusMaster) drives. Its Request
 * carries the fields every family's host shares (docs/design/spi-bus.md,
 * "Realizations"): the engine asserts the request's own chip select in
 * start(), which the bus AO calls, and releases it when the transaction
 * ends; a COMMAND phase goes out with D/C low and a DATA phase with it
 * high, the data phase has an optional out buffer (null = 0xFF dummies)
 * and an optional in buffer (null = the frames read back are discarded),
 * and every buffer is LENT until the reply lands. The mode, rate and frame
 * size travel per request, so a shared bus serves devices that disagree on
 * them; the bit order is a property of the WIRE and a bus-level verb.
 *
 *   using Bus = brio::SpiHost<2, brio::spi_default_pins<2>,
 *                             brio::DmaTxEngine<1, 5>, brio::DmaRxEngine<1, 4>>;
 *   using Arb = brio::SpiBus<Bus, P, 4>;          // the arbiter, in the Tenuto
 *   Bus::init(clock);                             // after the clock task
 *   brio::Pin<'B', 12>::output(true);             // the select, high
 *
 *   Bus::Request r{};
 *   r.cs = brio::Pin<'B', 12>::ref();
 *   r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
 *   r.clock = brio::SpiClock::div8;               // 9 MHz on PB1 at 72
 *   r.reply = brio::reply_to<Display, brio::SpiDone>();
 *   r.engine = brio::SpiEngine::dma;
 *   brio::post<Arb>(r);                           // on the engines
 *
 *   BRIO_CH32_VECTOR(dma1_channel4_handler) {
 *       if (Bus::dma_rx_isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
 *   }
 *   BRIO_CH32_VECTOR(dma1_channel5_handler) {
 *       if (Bus::dma_tx_isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
 *   }
 *
 * THE REQUEST IS LENT FOR THE CALL (util/bus_master.hpp): a request that
 * completes inside start() is read where it lies, nothing copied; one that
 * goes to the engines has what its tenure needs after the return - the
 * select and the D/C its completion raises - copied into the engine's own
 * Tenure in four word stores, and the reply stays the arbiter's.
 * The Request is laid out so that nothing in it is padding: the two
 * PinRefs, the three buffers and the reply first (eight words), then the
 * lengths and the four settings in two words.
 *
 * TWO ENGINES, AND THE REQUEST NAMES ONE (docs/design/spi-bus.md, "Two ways
 * to run a request"). Nothing here chooses and no cost is read at run time:
 * what a request costs at each rate is measured and stated in
 * docs/ch32vx03/spi.md, as advice to the program that names the engine.
 *
 *  - POLLED (`SpiEngine::polled`): the loops below inside start(), at every
 *    length and on every host, the caller spinning for the whole
 *    transaction and the engines never touched, so a polled request
 *    depends on no interrupt vector.
 *  - DMA (`SpiEngine::dma`): the two channels below at every length, one
 *    frame included: start() returns false and the receive channel's vector
 *    ends the transaction. In this order:
 *      1. a host with no engines bound REFUSES it: start() answers true with
 *         spi_rejected, before it touches the block, the select or the D/C,
 *         and `has_dma` says so at compile time;
 *      2. a request with nothing to move completes on the spot, spi_ok;
 *      3. what the engines cannot carry is REFUSED in the same way:
 *         engines_carry() below, every answer the channels' own launch could
 *         refuse with, asked before anything moves;
 *      4. a command phase with a data phase behind it runs in the loop inside
 *         start(), D/C rising after its last frame, and the data phase on the
 *         engines: a command is short by nature, and a block of its own would
 *         cost a second launch and a second interrupt. A stall in that loop
 *         ends the request as a polled one: start() true, spi_stalled, the
 *         select released, the block reset. A command phase that is the whole
 *         request rides the engines, D/C rising at its completion;
 *      5. everything else is one block on the engines.
 *
 * start() answers true for a polled request, an empty one and a refused one,
 * and false only when the engines have it.
 *
 * The host arms no interrupt of the SPI block: the instance's vector is not
 * its own, and init() leaves the instance's PFIC line disabled.
 *
 * ONE TRANSMIT BUFFER, ONE SHIFTER, ONE RECEIVE BUFFER (20.2.2, no FIFO),
 * and the polled loops are two. THE WRITE (no in buffer, and every command
 * phase) USES THE WRITE-AHEAD: a frame written whenever TXE says the buffer
 * is free - a frame time ahead of the shifter's need, so the clock never
 * pauses - and the answers left to overrun the receive buffer, which 20.2.7
 * makes harmless for the transmit side; the tail waits TXE (the last frame
 * into the shifter) and then not BSY (out of it), and the DATAR-then-STATR
 * read clears RXNE and OVR. Wire-bound at every rate, no hazard. THE
 * RECEIVE DECLINES IT and keeps ONE frame in flight at every rate: a second
 * frame queued behind the one shifting overruns the receive buffer if
 * anything holds the loop from reading frame k longer than one frame time
 * after its RXNE (20.2.7), and every handler and every preemption can. With
 * one in flight the clock waits for the read, so this host, which owns the
 * clock, cannot lose a frame: a handler or a preemption is a gap on the
 * bus. The next frame's load is pinned inside the wire time and the DATAR
 * read and the next write sit in adjacent instructions. What it costs is
 * the turn between two frames, at every rate from /2 to /256, in both
 * widths and both CPHA modes (bench_vx03 letter e's spi.frame lines, HCLK
 * cycles a frame over the wire at 144 MHz): 10 to 22 on SPI1 and 20 to 30 on SPI2 on the CH32V203C8T6, 11 to 21 and
 * 20 to 32 on the CH32V303VCT6.
 * A read that wants the wire's own rate names SpiEngine::dma. The width and
 * the buffers are locals of the loop, a null out buffer a zero-stride pointer
 * at 0xFF; one spin budget bounds the whole transaction.
 *
 * THE ENGINE SLOTS: both or neither (a block is full duplex and its
 * completion is the RECEIVE block's), on the two channels table 11-5 wires
 * to THIS instance and no others. ONE INTERRUPT A BLOCK: every frame that
 * comes back was clocked out first, so the receive block's completion
 * proves the transmit block's, and the transmit engine is armed for its
 * ERROR alone. THE BEAT IS THE FRAME: a request in 16-bit frames runs on the
 * engines as half-words - one 16-bit access to DATAR a frame, DFF and the
 * access agreeing - when its buffers are half-word aligned and the engines'
 * element is at least 16 bits. One the engines cannot carry so - 16-bit
 * frames from an odd address (the controller would round the address down in
 * silence, 11.3.6), or over engines of uint8_t - is refused, since running it
 * in the loop would hold the core for the whole wire behind a request that
 * said not to; a program whose buffers may sit anywhere names
 * SpiEngine::polled for them, or aligns them.
 *
 * FRAMES IN A BYTE BUFFER: one byte per 8-bit frame, two bytes low-first
 * per 16-bit frame - the Request's rule, its buffers being bytes whatever
 * the frame size (the polled loops' load_frame() and store_into()), one
 * half-word in memory, which is what an engine's beat moves.
 *
 * AFTER A FAILURE THE NEXT REQUEST STARTS ON A CLEAN BLOCK, whoever calls
 * it, and the program calls nothing. This type does it because it exists in
 * direct use as it does under the arbiter, and the arbiter goes on to the
 * next waiting request before the requester has run. After spi_stalled, in
 * start(): the select released FIRST, because the reset takes CPOL down with
 * CTLR1 and so moves the SCK pad in modes 2 and 3, and the device must have
 * seen the transaction end; then reset_block(), the block's RCC reset and the
 * applied configuration written back. After spi_dma_fault, in the vector: the
 * requests down, the channels put away, the select released and reset_owed_
 * set, and nothing else - no disable, no reset, no configuration and no wait;
 * the next start() tests that flag first and runs reset_block() before
 * anything else. The flag needs no critical section: the vector writes it
 * only while a transaction is in flight, and main context reads and clears it
 * only when none is. A caller framing the select by hand primes before it
 * falls, and prime() pays what a fault left owing, so the reset and the
 * frames still leaving the block stay outside that window. What a reset cannot
 * do: it does not open a closed clock gate, so on a block whose gate is closed
 * the next request stalls too, on its own budget. What it takes with it:
 * whatever CTLR1, CTLR2 and HSCR hold outside the applied configuration (a
 * hardware NSS arrangement, the high-speed read a program set on the
 * resource), the program's to set again.
 *
 * NO FLUSH AT START: the chapter gives no way for a frame to stand in the
 * receive buffer at the start of a transaction but a tenure that ended
 * without reading what it clocked, which here is a fault path - the
 * engines abandoned, a polled frame that never came - and every fault path
 * has reset the block before the next request's first frame. The normal
 * paths read every frame by count.
 *
 * IN SLEEP THE BUS MATRIX SERVES THE CORE ALONE on this family, so a
 * host with engines holds the program awake while a block is in flight;
 * `busy()` is what a sleep site asks.
 */
template <uint8_t n, SpiPins pins = spi_default_pins<n>, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine>
class SpiHost {
    using S = Spi<n>;

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from ch32vx03/dma.hpp, or NoDmaEngine (the default)");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio SpiHost: name both DMA engines or neither - the data phase is "
                  "full-duplex and its completion is the RECEIVE block's");
    static_assert(dma_engines_distinct<TxEngine, RxEngine>(),
                  "brio SpiHost: the two engines must ride two different DMA channels");
    static_assert(!TxEngine::present || dma_engine_slot<TxEngine>() == S::dma_tx_slot,
                  "brio SpiHost: on this family the channel IS the request (RM tables 11-2 to "
                  "11-5) - SPI1 transmits on DMA1's channel 3, SPI2 on DMA1's 5 and SPI3 on "
                  "DMA2's 2, and an engine on any other slot would move nothing");
    static_assert(!RxEngine::present || dma_engine_slot<RxEngine>() == S::dma_rx_slot,
                  "brio SpiHost: on this family the channel IS the request (RM tables 11-2 to "
                  "11-5) - SPI1 receives on DMA1's channel 2, SPI2 on DMA1's 4 and SPI3 on "
                  "DMA2's 1, and an engine on any other slot would move nothing");
    // Each engine's slot is this instance's request's for the whole image
    // (docs/design/dma.md, "The ownership rule"): another owner on either
    // channel does not compile. The I2S face raises the same two requests.
    static_assert(dma_engine_claim<TxEngine, DmaOwner<DmaPeriph::spi, n, DmaSignal::tx>>());
    static_assert(dma_engine_claim<RxEngine, DmaOwner<DmaPeriph::spi, n, DmaSignal::rx>>());
    static_assert(spi_pins_valid(n, pins),
                  "brio SpiHost: these SPI pads are not a link on this part - SCK is required, "
                  "no two signals may name the same pad, the remap code must be a column the "
                  "instance has, and every named pad must be one the package bonds");
    static_assert(pins.mosi.valid(),
                  "brio SpiHost: a host needs SCK and MOSI; MISO is optional only for a "
                  "write-only bus");

    using SckPin = Pin<pins.sck.port, pins.sck.pin>;
    using MosiPin = Pin<pins.mosi.port, pins.mosi.pin>;
    using MisoPin = Pin<pins.miso.valid() ? pins.miso.port : pins.sck.port,
                        pins.miso.valid() ? pins.miso.pin : pins.sck.pin>;
    using NssPin = Pin<pins.nss.valid() ? pins.nss.port : pins.sck.port,
                       pins.nss.valid() ? pins.nss.pin : pins.sck.pin>;

public:
    SpiHost() = delete;

    using Resource = S;
    static constexpr uint8_t number = n;
    static constexpr SpiPins pin_pads = pins;
    static constexpr bool has_engines = TxEngine::present;
    /// Can this host run a request naming SpiEngine::dma? The contract's name for
    /// has_engines (docs/design/spi-bus.md): false refuses every request
    /// naming SpiEngine::dma, so a program that posts one asserts it -
    /// `static_assert(Bus::has_dma)`.
    static constexpr bool has_dma = has_engines;

    struct Request {
        PinRef cs;   ///< asserted low around the transaction
        PinRef dc;   ///< display D/C line; null = no such pin
        /// Phase 1, sent with DC low; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        /// Phase 2 out, null = 0xFF dummies; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; LENT until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        ReplyTo<SpiDone> reply;
        uint16_t len;      ///< phase 2 length, in FRAMES
        uint8_t cmd_len;   ///< phase 1 length, in FRAMES
        /// Microseconds between the CS assertion and the first clock -
        /// what a device's datasheet calls CS setup. Spent spinning in
        /// start(), main context; 0 = none.
        uint8_t cs_setup_us = 0;

        /// Per-transaction bus configuration: a shared bus's devices each
        /// name their own, and the engine reprograms the peripheral only
        /// when the register word they make CHANGED.
        SpiClock clock = SpiClock::div16;
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;

        /// The engine that runs it (SpiEngine, util/spi_bus.hpp): polled, a
        /// loop inside start(), needing no engines and no vector; dma, the DMA
        /// engines at any length, ended by the receive channel's vector,
        /// refused (spi_rejected) on a host with none (has_dma) or when they
        /// cannot carry it (the class comment).
        SpiEngine engine = SpiEngine::polled;
    };
    static_assert(std::is_trivially_copyable_v<Request>);
    static_assert(sizeof(Request) == 2u * sizeof(PinRef) + 4u * sizeof(void*) + 8u,
                  "brio SpiHost: the Request is laid out with no padding - eight words of "
                  "pins, buffers and reply, then two of lengths and settings");

    // ---- lifecycle ----------------------------------------------------------

    /**
     * Bring the instance up as a host: gate, reset, configuration, pads,
     * the engines' binding. `clock` is the app's Clock tag; the rate this
     * instance's BR field divides is ITS OWN BUS'S, which the tag
     * answers. `max_sck_hz` is an optional CEILING for the whole bus,
     * re-resolved on rebase(); 0 = none. False when the ceiling cannot
     * be produced at this clock, the column's remap is refused, or the
     * boot configuration is refused.
     *
     * Once, from main(), after the clock task's init() and before the
     * first request: the rates it works out are the clock type's, not
     * the hardware's, and nothing checks the order (the file header).
     * It waits for nothing. The chip select is not one of its pads: the
     * program makes the Request's `cs` an output, driven high, itself.
     * The instance's PFIC line is left disabled - the host arms no
     * interrupt of the block - and the two channels' vectors are the
     * program's to bind.
     */
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        static_assert(clock_follows<Clock, SpiHost>(),
                      "this SpiHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its SCK ceiling and its cs_setup timing "
                      "would go stale on a clock change");
        Pfic::disable(S::irq);
        ceiling_hz_ = max_sck_hz;
        rebase(clock_hz(clock));
        if (max_sck_hz != 0u && !ceiling_) {
            return false;   // not even the bus over 256 honours it
        }
        S::bus_clock(true);
        S::reset();
        // THE COLUMN IS WRITTEN EVERY TIME, code 0 included: AFIO's field
        // is this instance's alone (no console or crystal shares it), and
        // a program that moves a host back to the reset column would
        // otherwise leave the remap standing.
        if (!S::remap(pins.remap)) {
            return false;
        }
        applied_ = boot_config();
        if (!S::configure(applied_)) {
            return false;
        }
        take_words();
        status_ = spi_ok;
        dma_active_ = false;
        if constexpr (has_engines) {
            reset_owed_ = false;
            arm_engines();
        }
        // The pads go to the peripheral only now, with the registers
        // already holding the idle polarity: a pad handed over first
        // would park undriven on the bus, and a client watching SCK
        // cannot tell a glitch from an edge. SPE is raised last.
        SckPin::function(PinDrive::push_pull, pad_speed_);
        MosiPin::function(PinDrive::push_pull, pad_speed_);
        if constexpr (pins.miso.valid()) {
            MisoPin::input();
        }
        // THE NSS PAD IS NOT CLAIMED: the boot configuration is
        // SpiNss::software and 20.2.1 leaves the pad free under it -
        // which is what this engine's chip select is, an ordinary GPIO
        // the Request carries. claim_nss_pad() is for a program that
        // moves the resource to a hardware arrangement.
        S::enable();
        S::flush_rx();
        return true;
    }

    /// The bus clock changed (DynamicClock fan-out). A Request's `clock`
    /// is a DIVISION of it and scales by itself; what is recomputed is
    /// the ceiling and the cs_setup timing. THE BUS MUST BE IDLE.
    static void rebase(uint32_t hz) {
        hclk_hz_ = hz;
        pclk_hz_ = spi_bus_hz_at<n>(hz);
        ceiling_ = ceiling_hz_ != 0u ? spi_rate_for(pclk_hz_, ceiling_hz_)
                                     : std::optional<SpiClock>{};
        floor_code_ = ceiling_ ? static_cast<uint8_t>(*ceiling_) : uint8_t{0};
        cs_rate_ = delay_rate(hz);
        // HCLK over this instance's bus, a power of two (RCC's PPRE): the
        // shift the polled budget turns bus cycles into core cycles with.
        pclk_shift_ = static_cast<uint8_t>(std::countr_zero(hz / pclk_hz_));
        take_budget();
    }

    /// The BR code that produces at most `hz` of SCK on this instance's
    /// bus - the chooser a device's datasheet limit is spoken to.
    static std::optional<SpiClock> clock_for(uint32_t hz) { return spi_rate_for(pclk_hz_, hz); }
    /// What a request at this code really runs at, ceiling included.
    static uint32_t sck_hz(SpiClock c) {
        return spi_sck_hz(pclk_hz_, static_cast<SpiClock>(clamp_code(c)));
    }
    static uint32_t max_sck_hz() { return ceiling_hz_; }
    static std::optional<SpiClock> ceiling_clock() { return ceiling_; }
    /// The rate this instance's BR field divides - PCLK2 for SPI1,
    /// PCLK1 for SPI2 - and the HCLK it was derived from.
    static uint32_t reference_hz() { return pclk_hz_; }
    static uint32_t hclk_hz() { return hclk_hz_; }

    /**
     * THE SLEW CLASS OF THE PADS THIS HOST DRIVES (SCK and MOSI; an
     * input pad has none). The fastest edge is also the loudest, and on
     * a bus of jumper wires a data line's edge coupled into the clock is
     * one sampling edge too many - which is a correctness parameter and
     * not a taste, in exactly the modes that sample on the falling edge
     * while the data changes on the rising one (docs/ch32vx03/spi.md
     * carries the measurement). The default is `PinSpeed::fast`, and a
     * choice made here SURVIVES the next init().
     */
    static void pad_speed(PinSpeed s) {
        pad_speed_ = s;
        SckPin::function(PinDrive::push_pull, s);
        MosiPin::function(PinDrive::push_pull, s);
    }
    static PinSpeed pad_speed() { return pad_speed_; }

    /// Put the peripheral at this mode, rate and frame size NOW, moving
    /// no data - for callers that frame the select window themselves
    /// (Request.cs null). A mode change is a CPOL FLIP ON THE WIRE and a
    /// selected client counts it as an edge: prime FIRST, then select. It
    /// pays the reset a spi_dma_fault left owing, so for a caller framing the
    /// select by hand that reset and the frames still leaving the block stay
    /// outside the window.
    static void prime(SpiMode m, SpiClock c, SpiDataSize bits = SpiDataSize::bits8) {
        if constexpr (has_engines) {
            if (reset_owed_) {
                (void)reset_block();
            }
        }
        apply(ctlr1_for(m, clamp_code(c), bits));
    }

    /// The bus's bit order (LSBFIRST): a property of the WIRE, applied
    /// once, not a per-request field. The bus must be idle.
    static bool bit_order(bool lsb_first) {
        if (applied_.lsb_first == lsb_first) {
            return true;
        }
        applied_.lsb_first = lsb_first;
        (void)S::disable();
        const bool ok = S::configure(applied_);
        S::enable();
        take_words();
        return ok;
    }
    static bool lsb_first() { return applied_.lsb_first; }

    // ---- the transfer -------------------------------------------------------

    /// Begin a transaction (SpiBus calls it from main context), on the engine
    /// it names (the class comment). True when it completed inside the call -
    /// a polled request, the empty one and a refused one (spi_rejected, the
    /// wire untouched); false when the engines have it and the receive
    /// channel's vector will end it, a TransferDone following - exactly
    /// util/bus_master.hpp's engine contract. The request is read where it
    /// lies; what a tenure on the engines needs after the return is copied
    /// here, in word stores.
    ///
    /// The arbiter's call, from its dispatch, or the program's for a
    /// polled request on a bus no arbiter owns. A request that finds a block
    /// of this host's own still on the engines is refused first, spi_rejected,
    /// before anything moves. A polled request spins here
    /// for the whole transaction under one spin budget (spi_stalled when it
    /// runs out, the select released and the block reset before the return),
    /// depending on no vector and masking nothing, and so does a command
    /// phase with a data phase behind it on the engines; the chip-select setup
    /// time is a busy wait here too. After a spi_dma_fault the reset that fault
    /// left owing runs first, here, in main context. The buffers stay the
    /// engine's until the reply, or until the return for a polled request
    /// called directly. The answer is the completion style, never a verdict:
    /// status() carries how the transaction ended. When it answers true after
    /// frames moved, the last SCK edge is behind it.
    static bool start(const Request& r) {
        if constexpr (!has_dma) {
            if (r.engine == SpiEngine::dma) {
                // This host has no engines: refused before the block, the
                // select or the D/C is touched, so nothing reaches the wire.
                status_ = spi_rejected;
                return true;
            }
        } else {
            // THE ONE BUSY CHECK (docs/design/spi-bus.md, "One busy check,
            // before anything moves"): this host's own block still on the
            // engines refuses the request, whichever engine it names, before
            // the select, the D/C or the block moves. An engineless host has
            // nothing in flight here: a polled request completes inside
            // start(). The channels are this host's alone for the whole image
            // (docs/design/dma.md, "The ownership rule"), so no other owner's
            // block is a case.
            if (dma_active_) [[unlikely]] {
                status_ = spi_rejected;
                return true;
            }
            if (reset_owed_) [[unlikely]] {
                (void)reset_block();
            }
        }
        status_ = spi_ok;
        if ((static_cast<uint32_t>(r.cmd_len) + r.len) == 0u) {
            return true;
        }
        if constexpr (has_dma) {
            if (r.engine == SpiEngine::dma) {
                return start_engines(r);
            }
        }

        open_window(r);
        const bool wide = spi_frame_is_halfword(r.bits);
        uint32_t budget = polled_budget(static_cast<uint32_t>(r.cmd_len) + r.len);
        uint8_t st = spi_ok;
        if (r.cmd_len != 0u) {
            st = poll_write_phase(r.cmd.get(), r.cmd_len, wide, budget);
            r.dc.set();
        }
        if (st == spi_ok && r.len != 0u) {
            st = r.rx.get() == nullptr ? poll_write_phase(r.tx.get(), r.len, wide, budget)
                                       : poll_duplex_phase(r.tx.get(), r.rx.get(), r.len, wide, budget);
        }
        r.cs.set();
        if (st != spi_ok) [[unlikely]] {
            status_ = st;
            (void)reset_block();
        }
        return true;
    }

    /// The RECEIVE channel's interrupt body - call from its vector, the
    /// program's binding, and from nowhere else; it masks nothing beyond the
    /// engines' own bus-master count. The one interrupt of a request: the
    /// block's completion ends it, D/C up and the select released; a transfer
    /// error ends it with spi_dma_fault and leaves the block's reset to the
    /// next start(). Compiles away on an engineless host. True when the
    /// transaction just completed, the last SCK edge behind it: the edge the
    /// program's glue posts TransferDone{status()} on.
    [[gnu::always_inline]] static bool dma_rx_isr() {
        if constexpr (has_engines) {
            const uint8_t rx = RxEngine::service();
            if (rx != 0u && dma_active_) {
                if ((rx & RxEngine::flag_error) != 0u) {
                    finish_dma(spi_dma_fault);
                    return true;
                }
                finish_dma(spi_ok);
                return true;
            }
        }
        return false;
    }

    /// The TRANSMIT channel's interrupt body - call from its vector, which
    /// rings on a transfer error alone: the transmit block's completion is
    /// proved by the receive block's and raises nothing.
    [[gnu::always_inline]] static bool dma_tx_isr() {
        if constexpr (has_engines) {
            const uint8_t tx = TxEngine::service();
            if (tx != 0u && dma_active_) {
                finish_dma(spi_dma_fault);
                return true;
            }
        }
        return false;
    }

    /// Both bodies in one, for a program that binds the two vectors to the
    /// same function: each reads only its own channel's flags.
    [[gnu::always_inline]] static bool dma_isr() { return dma_tx_isr() || dma_rx_isr(); }

    /// The engine's completion status, for the TransferDone payload: read
    /// in the vector right after a body answered true, or after a start()
    /// that answered true.
    static uint8_t status() { return status_; }

    /// Is a block on the engines? What a sleep site asks on a family whose
    /// bus matrix serves the core alone while it sleeps. A request that
    /// runs polled never answers true from outside: it completes inside
    /// start(), on the caller's own stack.
    static bool busy() { return dma_active_; }

    /// Put the ENGINE back where start() is legal, a tenure on the engines
    /// included: the verb a timed SpiBus calls on a transaction that never
    /// answered (spi_timeout), and a program's with the bus idle. Nothing
    /// needs it after spi_stalled or spi_dma_fault, which leave the next
    /// start() a clean block by themselves. The select released FIRST, so a
    /// frame still leaving is never read under a D/C it was not sent with, then
    /// the D/C a command alone holds low, the engines put away, then
    /// reset_block(). Main context; it waits, bounded, for the drain. False
    /// when a bounded wait ran out or the block's gate is closed.
    static bool recover() {
        t_.cs.set();
        t_.dc.set();   // the end of a request, however it ended: both high
        if constexpr (has_engines) {
            S::dma_requests(false, false);
            arm_engines();
        }
        dma_active_ = false;
        return reset_block();
    }

    /// SPE cleared and the gate closed, with no reset pulse, as every
    /// transport's release() on this family: init() starts from the block's
    /// reset (docs/design/dma.md, "The release contract"). This block's
    /// requests do not outlive SPE anyway (measured on all three instances,
    /// docs/ch32vx03/dma.md, "A released requester").
    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
            reset_owed_ = false;
        }
        Pfic::disable(S::irq);
        (void)S::disable();
        S::bus_clock(false);
        SckPin::release();
        MosiPin::release();
        if constexpr (pins.miso.valid()) {
            MisoPin::release();
        }
    }

    /// Hand the NSS pad to the peripheral - only for a program that has
    /// moved the resource to a hardware arrangement (SSOE's output, or
    /// the multi-host input). The task's own transactions never use it.
    static void claim_nss_pad(bool on) {
        if constexpr (pins.nss.valid()) {
            if (on) { NssPin::function(); } else { NssPin::release(); }
        } else {
            (void)on;
        }
    }

private:
    static_assert([] {
        if constexpr (TxEngine::present) {
            return sizeof(typename TxEngine::element) <= 2u &&
                   sizeof(typename RxEngine::element) <= 2u;
        } else {
            return true;
        }
    }(), "brio SpiHost: the DMA engines' element is the widest beat DATAR takes - "
         "uint8_t, or uint16_t for the 16-bit frames (DATAR is a 16-bit register)");

    /// Whether the engines can carry 16-bit frames at all.
    static constexpr bool engines_take_halfwords = [] {
        if constexpr (TxEngine::present) {
            return sizeof(typename TxEngine::element) >= 2u &&
                   sizeof(typename RxEngine::element) >= 2u;
        } else {
            return false;
        }
    }();

    /// What a tenure on the engines reads after start() returned: the two
    /// pads its completion raises, copied from the Request in word stores.
    struct Tenure {
        PinRef cs;
        PinRef dc;
    };

    /// The engines bind DATAR once: the transmit side armed for its ERROR
    /// alone (the receive block's completion is the transaction's), the
    /// receive side for its completion and its error.
    ///
    /// THE RECEIVE SIDE AT VERY_HIGH, THE TRANSMIT AT HIGH (DmaPriority's
    /// rule). The host clocks a frame out whenever the transmit channel
    /// fills DATAR, and DATAR's receive half holds one frame: a receive
    /// channel the transmit one outran would find the next frame on top of
    /// its last - OVR, the frame lost (20.2.7's overrun). A transmit
    /// channel that waits only stops the clock between two frames, which
    /// the host owns.
    static void arm_engines() {
        if constexpr (has_engines) {
            (void)TxEngine::arm(S::data_address(), TxEngine::flag_error, DmaPriority::high);
            (void)RxEngine::arm(S::data_address(), DmaPriority::very_high);
        }
    }

    /// The request's settings on the block, then D/C at the level of its first
    /// phase and the select down, with its setup time. What both engines run
    /// first, once a request.
    [[gnu::always_inline]] static void open_window(const Request& r) {
        apply(ctlr1_for(r.mode, clamp_code(r.clock), r.bits));
        if (r.cmd_len != 0u) {
            r.dc.clear();
        } else {
            r.dc.set();
        }
        r.cs.clear();   // assert, active low
        if (r.cs_setup_us != 0u) {
            (void)delay_us(cs_rate_, r.cs_setup_us);
        }
    }

    /// Whether a span of `count` frames at `a` crosses a 64 KB boundary, on the
    /// channels 11.2.3's note (1) holds to that rule: the CH32V303's DMA1. Null
    /// is the engine's own cell, which does not increment.
    [[gnu::always_inline]] static bool crosses_64k(const uint8_t* a, uint16_t count, bool wide) {
        if constexpr (RxEngine::bounded_to_64k) {
            if (a == nullptr) {
                return false;
            }
            const uint32_t first = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(a));
            const uint32_t bytes = wide ? static_cast<uint32_t>(count) * 2u : count;
            return ((first ^ (first + bytes - 1u)) >> 16) != 0u;
        } else {
            (void)a;
            (void)count;
            (void)wide;
            return false;
        }
    }

    /**
     * Can the engines carry this run? Every refusal launch_beats()'s calls
     * could answer, asked here, before anything moves, so that none of them
     * is left to answer after the select fell:
     *
     *  - 16-bit frames over engines of uint8_t, or from an odd address in
     *    either buffer: the frame is ONE 16-bit access, and an odd address
     *    would be rounded down by the controller in silence (11.3.6);
     *  - a span across 64 KB in either buffer on the CH32V303's DMA1 (11.2.3's
     *    note (1), measured as a wrap: dma.hpp's header).
     *
     * A block of this host's own still in flight is start()'s one busy check,
     * made before this; no other owner holds these channels (the class's
     * claims). What launch_beats() cannot be refused: an empty run or one
     * past CNTR (the count is a uint16_t and start() has left no empty
     * request), a null run (a null buffer is the engine's own cell). So its
     * answers are discarded with nothing left to say.
     */
    [[gnu::always_inline]] static bool engines_carry(bool wide, const uint8_t* out, const uint8_t* in,
                                                     uint16_t count) {
        if constexpr (has_engines) {
            if (wide) {
                if constexpr (engines_take_halfwords) {
                    const uint32_t odd = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(out)) |
                                         static_cast<uint32_t>(reinterpret_cast<uintptr_t>(in));
                    if ((odd & 1u) != 0u) {
                        return false;
                    }
                } else {
                    return false;
                }
            }
            return !crosses_64k(out, count, wide) && !crosses_64k(in, count, wide);
        } else {
            (void)wide;
            (void)out;
            (void)in;
            (void)count;
            return false;
        }
    }

    /// A request naming SpiEngine::dma, onto the engines: the class comment's
    /// steps 3, 4 and 5. Once a request, out of the vectors' way. False: the
    /// receive channel's vector ends it. True: refused, or the command loop
    /// stalled.
    static bool start_engines(const Request& r) {
        const bool wide = spi_frame_is_halfword(r.bits);
        const bool command_alone = r.len == 0u;
        const uint8_t* const out = command_alone ? r.cmd.get() : r.tx.get();
        uint8_t* const in = command_alone ? nullptr : r.rx.get();
        const uint16_t count = command_alone ? r.cmd_len : r.len;
        if (!engines_carry(wide, out, in, count)) {
            status_ = spi_rejected;
            return true;
        }
        open_window(r);
        t_.cs = r.cs;
        t_.dc = r.dc;
        if (r.cmd_len != 0u && !command_alone) {
            uint32_t budget = polled_budget(r.cmd_len);
            const uint8_t st = poll_write_phase(r.cmd.get(), r.cmd_len, wide, budget);
            r.dc.set();
            if (st != spi_ok) [[unlikely]] {
                r.cs.set();
                status_ = st;
                (void)reset_block();
                return true;
            }
        }
        dma_active_ = true;
        if constexpr (engines_take_halfwords) {
            if (wide) {
                launch_beats<uint16_t>(out, in, count);
                return false;
            }
        }
        launch_beats<uint8_t>(out, in, count);
        return false;
    }

    /**
     * A block onto the two engines. BOTH CHANNELS ARE PROGRAMMED FIRST AND THE
     * REQUESTS RAISED IN ONE STORE AFTER THEM: a request that rises while its
     * channel is disabled is latched by the controller and served at the
     * channel's next enable, so no request is up while a channel is programmed
     * - and since nothing can complete before that last store, the
     * completion's own store (finish_dma() drops both requests) is always the
     * later one. With the requests raised around each channel instead, a
     * one-frame transaction completed INSIDE the launch and the launch's last
     * store raised both again after it - the next request then stalled, its
     * transmit block moving into nothing (measured). The channels' answers are
     * discarded because engines_carry() asked each refusal first.
     */
    template <typename T>
    static void launch_beats(const uint8_t* out, uint8_t* in, uint16_t count) {
        if constexpr (has_engines) {
            T* const rx = reinterpret_cast<T*>(in);
            const T* const tx = reinterpret_cast<const T*>(out);
            if (rx != nullptr) {
                (void)RxEngine::start(std::span<T>(rx, count));
            } else {
                (void)RxEngine::start_discard(reinterpret_cast<T*>(&rx_sink_), count);
            }
            if (tx != nullptr) {
                (void)TxEngine::start(std::span<const T>(tx, count));
            } else {
                (void)TxEngine::start_fixed(reinterpret_cast<const T*>(&tx_dummy_), count);
            }
            S::dma_requests(true, true);
        }
    }

    /// One exit for a tenure on the engines. The requests go down FIRST, so
    /// that neither channel sees one rise while it is being stopped; then
    /// both channels are stopped - the receive one too, whose EN a
    /// completed block leaves set and whose bus-master count would
    /// otherwise keep the core awake (ch32vx03/bus_activity.hpp). A fault
    /// notes the reset it owes and leaves it to the next start(), which clears
    /// the frames still in the block and the one the receive buffer may hold.
    /// D/C rises, which a command phase that was the whole request still held
    /// low, and the select is released; the vector's caller posts completion.
    static void finish_dma(uint8_t st) {
        if constexpr (has_engines) {
            S::dma_requests(false, false);
            // The tenure's own answer, always: a request the busy check
            // refused behind it answered spi_rejected in the same word.
            status_ = st;
            if (st != spi_ok) {
                reset_owed_ = true;
                (void)TxEngine::abandon();
                RxEngine::stop();
            } else {
                (void)TxEngine::complete();
                RxEngine::complete();
            }
        }
        dma_active_ = false;
        t_.dc.set();
        t_.cs.set();
    }

    /**
     * THE ONE RESET: the block back to its reset state and the applied
     * configuration written over it, whatever failed. The stall paths call
     * it, the reset a fault left owing, and recover(); each has released the
     * select first. Cold and out of line, main context only: disable() can
     * wait, so no vector calls it. False when a bounded wait of disable() ran
     * out, which it asks only of a block whose SPE reads back set: a closed
     * gate reads CTLR1 as zero, and a demoted host has lost the bit. The
     * column is written again because the field is the instance's and another
     * host type on it may have moved it; the pads are not this reset's.
     *
     * A CLOSED GATE IS LEFT ALONE, and the answer is false. Measured: with the
     * pulse and its writes run on a closed gate, a request on the gate opened
     * stalled once more; with neither, a gate closed and opened keeps the
     * registers and the same request runs spi_ok. So the next request on a
     * closed gate stalls on its own budget, and one on the gate opened runs on
     * the block as it was; what a fault left owing stays owed.
     */
    [[gnu::cold, gnu::noinline]] static bool reset_block() {
        if (!Rcc::enabled(S::bus, spi_gate_for(n))) {
            return false;
        }
        if constexpr (has_engines) {
            reset_owed_ = false;
        }
        Pfic::disable(S::irq);
        const bool ok = S::disable();
        S::reset();
        (void)S::remap(pins.remap);
        const bool cfg = S::configure(applied_);
        S::enable();
        S::flush_rx();
        take_words();
        return ok && cfg;
    }

    /// A slower rate is a LARGER code, so the ceiling is a floor on it.
    static uint8_t clamp_code(SpiClock c) {
        const uint8_t code = static_cast<uint8_t>(c);
        return code < floor_code_ ? floor_code_ : code;
    }

    static SpiConfig boot_config() {
        SpiConfig c{};
        c.role = SpiRole::host;
        c.mode = SpiMode::mode0;
        c.bits = SpiDataSize::bits8;
        c.clock = ceiling_ ? *ceiling_ : SpiClock::div16;
        c.nss = SpiNss::software;
        // The DMA requests are NOT part of the applied configuration:
        // launch_beats() raises them around the engines (see there).
        return c;
    }

    /// The bits of CTLR1 a request decides (20.4.1): CPHA and CPOL are
    /// SpiMode's own two bits in the register's own places, BR the code,
    /// DFF the width.
    static constexpr uint16_t ctlr1_request_mask =
        static_cast<uint16_t>(spi_cpha | spi_cpol | spi_br_mask | spi_dff);

    /// The two words apply() compares against: what CTLR1 holds for the
    /// applied configuration with SPE clear, and the part of it no request
    /// decides.
    static void take_words() {
        ctlr1_ = spi_ctlr1_of(applied_);
        base_ = static_cast<uint16_t>(ctlr1_ & ~ctlr1_request_mask);
        take_budget();
    }

    /// CTLR1 for this request's mode, rate code and width over the
    /// applied base: no table, no branch.
    [[gnu::always_inline]] static uint16_t ctlr1_for(SpiMode m, uint8_t code, SpiDataSize bits) {
        return static_cast<uint16_t>(
            base_ | (static_cast<uint16_t>(m) & 3u) |
            static_cast<uint16_t>((code & 7u) << spi_br_shift) |
            static_cast<uint16_t>((static_cast<uint16_t>(bits) & 1u) << 11));
    }

    /// Put the peripheral where this request wants it: ONE WORD compared,
    /// and the disable/enable pair (DFF is enable-protected, BR/CPOL/CPHA
    /// "cannot be modified during communication") paid only when it moved.
    [[gnu::always_inline]] static void apply(uint16_t want) {
        if (want != ctlr1_) {
            reapply(want);
        }
    }
    static void reapply(uint16_t want) {
        ctlr1_ = want;
        take_budget();
        applied_.mode = static_cast<SpiMode>(want & 3u);
        applied_.clock = static_cast<SpiClock>((want & spi_br_mask) >> spi_br_shift);
        applied_.bits = (want & spi_dff) != 0u ? SpiDataSize::bits16 : SpiDataSize::bits8;
        (void)S::disable();
        S::regs().CTLR1 = want;
        S::enable();
    }

    // ---- the polled loops ---------------------------------------------------

    /// THE POLLED BUDGET, derived from the clocks whenever a rate is applied
    /// and kept as a shift, so a request pays one shift of its frames: a frame
    /// is 8 or 16 bits (DFF) of 2^(BR + 1) bus cycles each, and a bus cycle
    /// 2^pclk_shift_ core cycles (rebase()); the budget is twice the request's
    /// frames in core cycles, plus 256, counted in polls of STATR, each at
    /// least one core cycle. So it never runs out before the wire has had
    /// twice its own time, a handler that lands in the loop costing time and
    /// no polls; and on a block that never moves it ends after that many polls
    /// - 16 frames of 8 bits at /64 on SPI1 (PCLK2 = HCLK) are 16640 polls, a
    /// few cycles each. The shift stops at 15, where the longest request still
    /// fits a word (65790 frames). That takes something off only at /256 with
    /// 16-bit frames on a bus at an eighth of HCLK, and on one at a sixteenth at
    /// /256 or at /128 with 16-bit frames: a frame there is at most 65536 core
    /// cycles against 32768 polls of five instructions each, still past the
    /// wire's time.
    static void take_budget() {
        const uint32_t code = (static_cast<uint32_t>(ctlr1_) & spi_br_mask) >> spi_br_shift;
        const uint32_t bits_shift = (ctlr1_ & spi_dff) != 0u ? 4u : 3u;
        const uint32_t shift = bits_shift + code + 1u + pclk_shift_ + 1u;
        budget_shift_ = static_cast<uint8_t>(shift < 15u ? shift : 15u);
    }
    [[gnu::always_inline]] static uint32_t polled_budget(uint32_t frames) {
        return (frames << budget_shift_) + 256u;
    }

    static uint8_t poll_write_phase(const uint8_t* tx, uint16_t frames, bool wide, uint32_t& budget) {
        return wide ? poll_write<true>(tx, frames, budget) : poll_write<false>(tx, frames, budget);
    }
    static uint8_t poll_duplex_phase(const uint8_t* tx, uint8_t* rx, uint16_t frames, bool wide, uint32_t& budget) {
        return wide ? poll_duplex<true>(tx, rx, frames, budget) : poll_duplex<false>(tx, rx, frames, budget);
    }

    /// The out buffer walked at the frame's width, or a zero-stride stand
    /// on 0xFF when there is none.
    template <bool wide>
    [[gnu::always_inline]] static uint32_t load_frame(const uint8_t* p) {
        uint32_t v = p[0];
        if constexpr (wide) {
            v |= static_cast<uint32_t>(p[1]) << 8;
        }
        return v;
    }
    template <bool wide>
    [[gnu::always_inline]] static void store_into(uint8_t* p, uint16_t v) {
        p[0] = static_cast<uint8_t>(v);
        if constexpr (wide) {
            p[1] = static_cast<uint8_t>(v >> 8);
        }
    }

    /**
     * THE TRANSMIT-ONLY LOOP (20.2.2's "maintain a complete data flow"): a
     * frame written whenever TXE says the buffer is free - a frame time
     * ahead of the shifter's need, so the clock never pauses - and the
     * answers left to overrun the receive buffer, which 20.2.7 makes
     * harmless for the transmit side. The tail waits TXE (the last frame
     * into the shifter) and then not BSY (out of it, 20.4.3); the
     * DATAR-then-STATR read clears RXNE and OVR. `frames` >= 1.
     */
    template <bool wide>
    static uint8_t poll_write(const uint8_t* tx, uint16_t frames, uint32_t& budget) {
        SpiRegs& regs = S::regs();
        const uint8_t* src = tx != nullptr ? tx : ones_;
        const uint32_t step = tx != nullptr ? (wide ? 2u : 1u) : 0u;
        uint32_t spins = budget;
        uint32_t left = frames;   // a word: a half-word counter costs a zero-extension a turn
        do {
            while ((regs.STATR & spi_txe) == 0u) {
                if (--spins == 0u) { budget = 0; return spi_stalled; }
            }
            regs.DATAR = static_cast<uint16_t>(load_frame<wide>(src));
            src += step;
        } while (--left != 0u);
        while ((regs.STATR & spi_txe) == 0u) {
            if (--spins == 0u) { budget = 0; return spi_stalled; }
        }
        while ((regs.STATR & spi_bsy) != 0u) {
            if (--spins == 0u) { budget = 0; return spi_stalled; }
        }
        (void)regs.DATAR;
        (void)regs.STATR;
        budget = spins;
        return spi_ok;
    }

    /**
     * THE RECEIVE LOOP, one frame in flight at every rate: frame 0 written;
     * then for each frame the next one's load, the wait on RXNE, the DATAR
     * read and the DATAR write back to back, the store of the frame read;
     * the last frame read. Nothing is queued behind the frame shifting, so
     * the next frame moves only after this one was read, a handler or a
     * preemption is a gap on the bus and OVR cannot rise (20.2.7: it needs a
     * frame completed over an unread one). The load is pinned inside the
     * wire time by an empty asm, because the compiler would sink it past the
     * poll, onto the bus's dead time (a WORD, so the barrier zero-extends
     * nothing). `frames` >= 1.
     */
    template <bool wide>
    static uint8_t poll_duplex(const uint8_t* tx, uint8_t* rx, uint16_t frames, uint32_t& budget) {
        SpiRegs& regs = S::regs();
        constexpr uint32_t width = wide ? 2u : 1u;
        const uint8_t* src = tx != nullptr ? tx : ones_;
        const uint32_t step = tx != nullptr ? width : 0u;
        uint32_t spins = budget;
        uint32_t left = frames;
        regs.DATAR = static_cast<uint16_t>(load_frame<wide>(src));   // the shifter is idle: it moves at once
        while (--left != 0u) {
            src += step;
            const uint32_t next = load_frame<wide>(src);
            __asm__ volatile("" : : "r"(next));
            while ((regs.STATR & spi_rxne) == 0u) {
                if (--spins == 0u) { budget = 0; return spi_stalled; }
            }
            const uint16_t got = regs.DATAR;
            regs.DATAR = static_cast<uint16_t>(next);
            store_into<wide>(rx, got);
            rx += width;
        }
        while ((regs.STATR & spi_rxne) == 0u) {
            if (--spins == 0u) { budget = 0; return spi_stalled; }
        }
        store_into<wide>(rx, regs.DATAR);
        budget = spins;
        return spi_ok;
    }

    static inline Tenure t_{};
    /// Written by the vector and read by the thread: volatile, as the two
    /// below.
    static inline volatile uint8_t status_ = spi_ok;
    static inline volatile bool dma_active_ = false;
    /// A fault ended a tenure on the engines and the next start() owes the
    /// block its reset: the vector sets it, while a transaction is in flight;
    /// main context tests and clears it when none is. Named on a host with
    /// engines alone.
    static inline volatile bool reset_owed_ = false;
    /// The dummies of a one-sided block, wide enough for either frame: a
    /// byte beat reads the low byte of either (little-endian).
    static inline uint16_t rx_sink_ = 0;
    static constexpr uint16_t tx_dummy_ = 0xFFFF;
    /// The polled loops' zero-stride stand: what a null out buffer clocks.
    static constexpr uint8_t ones_[2] = {0xFF, 0xFF};
    static inline SpiConfig applied_{};
    static inline uint16_t ctlr1_ = 0;   ///< CTLR1 as applied, SPE clear
    static inline uint16_t base_ = 0;    ///< ctlr1_ outside the request's bits
    static inline uint8_t floor_code_ = 0;
    static inline uint32_t hclk_hz_ = 0;
    static inline uint32_t pclk_hz_ = 0;
    static inline uint32_t ceiling_hz_ = 0;
    static inline std::optional<SpiClock> ceiling_{};
    static inline DelayRate cs_rate_{};
    static inline uint8_t pclk_shift_ = 0;   ///< log2(HCLK / this bus), rebase()
    static inline uint8_t budget_shift_ = 0; ///< take_budget(): the polled budget's shift
    static inline PinSpeed pad_speed_ = PinSpeed::fast;
};

// =============================================================================
// The client task
// =============================================================================

/**
 * SpiClient<n, pins>
 *
 * The other end of the wire: SCK, MOSI and NSS are inputs, the host sets
 * the pace, and the only thing this side controls is WHAT it has ready
 * to shift out when the next clock arrives.
 *
 * ONE FRAME AHEAD. With no FIFO the transmit buffer holds exactly the
 * next answer: 20.2.3 says the slave starts to transmit on the first
 * sampling edge, moving the buffer into the shifter and raising TXE -
 * the moment to write the NEXT one. So: load the first answer before the
 * host selects (`enable(first)`), then write frame k + 1 on TXE while
 * frame k shifts; a write made on RXNE instead reaches the buffer one
 * frame late under a host that clocks back to back. A client that falls
 * behind does not stall the bus: what the host reads is whatever the
 * shifter held, a wrong answer and never a missing one.
 *
 * THE NSS PAD IS THE TRANSACTION: read directly (selected()), since the
 * peripheral publishes no select status. `drive_output` makes this client
 * a DARK LISTENER - MISO is handed to the peripheral only for the window
 * it answers in, so a shared harness stays uncontested. On this family
 * the pad can be taken back to a floating INPUT with the block still
 * enabled (`drive_output(false)` at any time), which is what the bench
 * measured; nothing of the peripheral has to move with it.
 */
template <uint8_t n, SpiPins pins = spi_default_pins<n>>
class SpiClient {
    using S = Spi<n>;

    static_assert(spi_pins_valid(n, pins),
                  "brio SpiClient: these SPI pads are not a link on this part - SCK is "
                  "required, no two signals may name the same pad, the remap code must be a "
                  "column the instance has, and every named pad must be one the package bonds");
    static_assert(pins.mosi.valid(), "brio SpiClient: a client listens on MOSI");

    using SckPin = Pin<pins.sck.port, pins.sck.pin>;
    using MosiPin = Pin<pins.mosi.port, pins.mosi.pin>;
    using MisoPin = Pin<pins.miso.valid() ? pins.miso.port : pins.sck.port,
                        pins.miso.valid() ? pins.miso.pin : pins.sck.pin>;
    using NssPin = Pin<pins.nss.valid() ? pins.nss.port : pins.sck.port,
                       pins.nss.valid() ? pins.nss.pin : pins.sck.pin>;

public:
    SpiClient() = delete;

    using Resource = S;
    static constexpr uint8_t number = n;
    static constexpr SpiPins pin_pads = pins;
    static constexpr bool has_nss_pad = pins.nss.valid();

    struct Config {
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;
        bool lsb_first = false;
        /// software: the pad is free and select() drives SSI.
        /// hardware_input: the NSS pad IS the chip select (the default).
        SpiNss nss = SpiNss::hardware_input;
        SpiDirection direction = SpiDirection::full_duplex;
        bool crc = false;
        uint16_t crc_polynomial = 0x0007u;
        /// Whether the MISO pad is handed to the peripheral at all.
        bool drive_output = true;
    };

    /// Bring the instance up as a client. The bus clock is required even
    /// though the shifter runs on the host's SCK. False when the
    /// configuration is refused - a client with SSOE, above all, which is
    /// a host's verb.
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg = {}) {
        (void)clock;
        Pfic::disable(S::irq);
        S::bus_clock(true);
        S::reset();
        if (!S::remap(pins.remap)) {
            return false;
        }
        if (!S::configure(config_of(cfg))) {
            return false;
        }
        // Inputs first, the output last.
        SckPin::input();
        MosiPin::input();
        if constexpr (has_nss_pad) {
            NssPin::input(PinPull::up);
        }
        drive_output(cfg.drive_output);
        S::flush_rx();
        Pfic::enable(S::irq);
        return true;
    }

    /// Hand the answer line to the peripheral, or take it back (a
    /// floating input, driving nothing) - the dark listener's one verb.
    static void drive_output(bool on) {
        if constexpr (pins.miso.valid()) {
            if (on) {
                MisoPin::function(PinDrive::push_pull, pad_speed_);
            } else {
                MisoPin::release();
            }
        } else {
            (void)on;
        }
    }
    static bool output_driven() {
        if constexpr (pins.miso.valid()) {
            return MisoPin::nibble() ==
                   pin_nibble(PinMode::alternate, PinDrive::push_pull, pad_speed_);
        } else {
            return false;
        }
    }

    /// The slew class of the ONE pad a client drives - SpiHost's verb
    /// with the same reason behind it, and a choice that survives the
    /// next init().
    static void pad_speed(PinSpeed s) {
        pad_speed_ = s;
        if (output_driven_now()) {
            drive_output(true);
        }
    }
    static PinSpeed pad_speed() { return pad_speed_; }

    /// SPE up and the FIRST ANSWER in the buffer before the host's clock
    /// can arrive. Call it with NSS still high.
    static void enable(uint16_t first) {
        S::enable();
        write(first);
    }
    /// SPE up with the transmit buffer EMPTY: the opening of a run, which
    /// writes its own first answer.
    static void enable() { S::enable(); }
    static bool disable() { return S::disable(); }

    /// How many answers must be queued ahead of the frame the host is
    /// about to clock: ONE here - no FIFO, one buffer.
    static constexpr uint8_t frames_ahead = 1;

    /// Load the next frame to shift out; on TXE, for the one-ahead rule.
    static void write(uint16_t v) { S::data(bits_, v); }
    static bool writable() { return S::txe(); }

    /// One received frame, or nothing. Reading DATAR clears RXNE.
    static std::optional<uint16_t> poll() {
        if (!S::rxne()) {
            return {};
        }
        return S::data(bits_);
    }

    /// Is the host holding NSS low right now? A live pad read. Always
    /// false without an NSS pad.
    static bool selected() {
        if constexpr (has_nss_pad) {
            return !NssPin::read();
        } else {
            return false;
        }
    }

    /// The software select, for a client under SSM.
    static void select(bool on) { S::software_select(on); }

    static bool overrun() { return S::overrun(); }
    static void clear_overrun() { S::clear_overrun(); }
    static bool crc_error() { return S::crc_error(); }
    static void clear_crc_error() { S::clear_crc_error(); }
    static uint16_t status() { return S::status(); }

    // ---- the polled runs ---------------------------------------------------
    //
    // A client's unit is the RUN: the answers to a transaction, or what it
    // brings, served polled in one call - the per-frame verbs above being
    // its degenerate case. What bounds a run is figure 20-1's one transmit
    // buffer in front of the shifter: 20.2.3's client moves the buffer into
    // the shifter on the frame's first sampling edge and raises TXE, so the
    // answer to frame k + 1 may be written from then until frame k + 1's
    // first edge - most of a frame of window; the receive buffer is ONE
    // frame deep, so a run that reads must read each frame inside the
    // next one. THE SELECT PAD IS THE WINDOW (the peripheral publishes no
    // select status): a run waits for the pad to FALL before it takes a
    // high pad for the close, and reads it once for every four polls of the
    // flag - a STATR load is an APB load and the pad's a GPIO one, and the
    // flag's own load, test and branch is then the whole path from the
    // flag to the store; a run without an NSS pad ends on its count or its
    // budget alone. They are ALWAYS INLINE, so the caller's frame width and
    // buffers are constants of the loop.

    /// How a run ended.
    enum class RunEnd : uint8_t {
        complete,   ///< every frame of the run moved
        closed,     ///< the host raised the select (the NSS pad read high)
        stalled,    ///< the spin budget ran out first
    };

    /// What a run did: `count` frames - answers written for answer(),
    /// frames received for exchange() and receive() - and how it ended.
    struct Run {
        uint16_t count;
        RunEnd end;
    };

    /**
     * ANSWER a transaction with `out`, polled on TXE alone, reading
     * nothing. Call it with SPE up (`enable()`), the buffer EMPTY and the
     * select still HIGH: the first answer goes into the buffer at once,
     * every later one on the TXE of the frame before it. Ends when the host
     * raises the select; the frames past the run's end carry what the
     * shifter is left with. The receive buffer overruns from the second
     * frame (nothing is read) and the tail's DATAR-then-STATR read clears
     * it (20.2.7).
     */
    template <typename T>
    [[gnu::always_inline]] static Run answer(std::span<const T> out, uint32_t spins) {
        static_assert(run_frame<T>, "a frame of this block is eight bits or sixteen (DFF): uint8_t or uint16_t");
        SpiRegs& regs = S::regs();
        const T* const first = out.data();
        const T* const end = first + out.size();
        const T* p = first;
        RunEnd how = RunEnd::closed;
        if (p != end) {
            regs.DATAR = *p++;
        }
        if (!await_select(spins)) {
            how = RunEnd::stalled;
            S::flush_rx();
            return {static_cast<uint16_t>(p - first), how};
        }
        for (; p != end; ++p) {
            const T next = *p;
            if (!await<spi_txe>(spins, how)) {
                break;
            }
            regs.DATAR = next;
        }
        if (p == end) {
            if constexpr (has_nss_pad) {
                while (!pad_closed()) {
                    if (--spins == 0u) {
                        how = RunEnd::stalled;
                        break;
                    }
                }
            } else {
                how = RunEnd::complete;
            }
        }
        S::flush_rx();
        return {static_cast<uint16_t>(p - first), how};
    }

    /**
     * EXCHANGE: answer with `out` and keep what the host sends in `in`, the
     * run's length the shorter of the two. The answers go as answer()'s;
     * each turn then reads, on its RXNE, the frame whose TXE it wrote on -
     * frame k is read while frame k + 1 shifts, inside the receive buffer's
     * one frame of grace. Ends `complete` when the last frame is in,
     * `closed` when the host raised the select first (what arrived before
     * is in `in`), `stalled` on the budget. The receive side is cleared at
     * the start, the select high.
     */
    template <typename T>
    [[gnu::always_inline]] static Run exchange(std::span<const T> out, std::span<T> in, uint32_t spins) {
        static_assert(run_frame<T>, "a frame of this block is eight bits or sixteen (DFF): uint8_t or uint16_t");
        const uint32_t len = out.size() < in.size() ? out.size() : in.size();
        if (len == 0u) {
            return {0, RunEnd::complete};
        }
        SpiRegs& regs = S::regs();
        S::flush_rx();
        const T* p = out.data();
        const T* const p_end = p + len;
        T* const first = in.data();
        T* const q_end = first + len;
        T* q = first;
        RunEnd how = RunEnd::complete;
        regs.DATAR = *p++;
        if (!await_select(spins)) {
            return {0, RunEnd::stalled};
        }
        for (; p != p_end; ++p, ++q) {
            const T next = *p;
            if (!await<spi_txe>(spins, how)) {
                return drained(q, first, q_end, how);
            }
            regs.DATAR = next;
            if (!await<spi_rxne>(spins, how)) {
                return drained(q, first, q_end, how);
            }
            *q = static_cast<T>(regs.DATAR);
        }
        if (!await<spi_rxne>(spins, how)) {
            return drained(q, first, q_end, how);
        }
        *q++ = static_cast<T>(regs.DATAR);
        return {static_cast<uint16_t>(q - first), RunEnd::complete};
    }

    /**
     * RECEIVE a run into `in`, polled on RXNE alone, writing nothing. The
     * receive buffer is NOT flushed at the start, so a frame standing in it
     * when the run began is the run's first. Ends as exchange() does.
     */
    template <typename T>
    [[gnu::always_inline]] static Run receive(std::span<T> in, uint32_t spins) {
        static_assert(run_frame<T>, "a frame of this block is eight bits or sixteen (DFF): uint8_t or uint16_t");
        SpiRegs& regs = S::regs();
        T* const first = in.data();
        T* const q_end = first + in.size();
        T* q = first;
        RunEnd how = RunEnd::complete;
        if ((regs.STATR & spi_rxne) == 0u && !await_select(spins)) {
            return {0, RunEnd::stalled};
        }
        for (; q != q_end; ++q) {
            if (!await<spi_rxne>(spins, how)) {
                if ((regs.STATR & spi_rxne) != 0u) {
                    *q++ = static_cast<T>(regs.DATAR);
                }
                break;
            }
            *q = static_cast<T>(regs.DATAR);
        }
        return {static_cast<uint16_t>(q - first), how};
    }

    /// The ISR body: the raised-and-enabled sources, for the app's glue.
    [[gnu::always_inline]] static uint32_t isr() { return S::isr(); }

    static void rxne_interrupt(bool on) { S::rxne_interrupt(on); }
    static void txe_interrupt(bool on) { S::txe_interrupt(on); }
    static void error_interrupt(bool on) { S::error_interrupt(on); }

    static SpiDataSize bits() { return bits_; }

    static void release() {
        Pfic::disable(S::irq);
        (void)S::disable();
        S::bus_clock(false);
        SckPin::release();
        MosiPin::release();
        if constexpr (pins.miso.valid()) {
            MisoPin::release();
        }
        if constexpr (has_nss_pad) {
            NssPin::release();
        }
    }

private:
    static SpiConfig config_of(const Config& c) {
        bits_ = c.bits;
        SpiConfig s{};
        s.role = SpiRole::client;
        s.mode = c.mode;
        s.bits = c.bits;
        s.lsb_first = c.lsb_first;
        s.nss = c.nss;
        s.direction = c.direction;
        s.crc = c.crc;
        s.crc_polynomial = c.crc_polynomial;
        return s;
    }

    static bool output_driven_now() {
        if constexpr (pins.miso.valid()) {
            return (MisoPin::nibble() & 0xCu) == 0x8u;   // CNF 10: alternate push-pull
        } else {
            return false;
        }
    }

    template <typename T>
    static constexpr bool run_frame = std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>;

    /// The NSS pad read low, the window open (yes at once without a pad).
    [[gnu::always_inline]] static bool pad_low() {
        if constexpr (has_nss_pad) {
            return (NssPin::P::in() & NssPin::mask) == 0u;
        } else {
            return true;
        }
    }

    /// THE WINDOW CLOSES ON TWO READS. A high read is taken for the host's
    /// close only when the pad still reads high at the next look: on a
    /// board-to-board bus the select wire carries the transients of the
    /// other three, and a run that took one high read for the close ended
    /// mid-window - measured on the CH32V203C8T6 against a CH32V303VCT6
    /// host, 3 to 17 frames of 32 taken in runs the block itself served
    /// whole (its own NSS input never saw the glitch: the DMA-fed serves on
    /// the same wire were exact). The second read is the next look's own,
    /// so a flag wait pays one register clear for it, and a real close is
    /// seen one look late; a host's real close stands for the rest of the
    /// gap between windows.
    /// pad_closed() is the same rule for a loop that does nothing else:
    /// eight reads in a row, all high.
    [[gnu::always_inline]] static bool pad_closed() {
        for (uint8_t k = 0; k < 8u; ++k) {
            if (pad_low()) {
                return false;
            }
        }
        return true;
    }

    [[gnu::always_inline]] static bool await_select(uint32_t& spins) {
        while (!pad_low()) {
            if (--spins == 0u) {
                return false;
            }
        }
        return true;
    }

    /// Poll STATR until `flag` stands: true. False when the host raised the
    /// select - two looks in a row reading it high (pad_closed()'s reason) -
    /// or the budget ran out, `how` saying which. FOUR POLLS A LOOK (the
    /// runs' comment).
    template <uint16_t flag>
    [[gnu::always_inline]] static bool await(uint32_t& spins, RunEnd& how) {
        SpiRegs& regs = S::regs();
        bool high_once = false;
        for (;;) {
            if ((regs.STATR & flag) != 0u) { return true; }
            if ((regs.STATR & flag) != 0u) { return true; }
            if ((regs.STATR & flag) != 0u) { return true; }
            if ((regs.STATR & flag) != 0u) { return true; }
            if constexpr (has_nss_pad) {
                if (!pad_low()) {
                    if (high_once) {
                        how = RunEnd::closed;
                        return false;
                    }
                    high_once = true;
                } else {
                    high_once = false;
                }
            }
            if (--spins == 0u) {
                how = RunEnd::stalled;
                return false;
            }
        }
    }

    /// An exchange that ended early: the frame standing in the receive
    /// buffer, if any, is the run's last.
    template <typename T>
    [[gnu::always_inline]] static Run drained(T* q, T* first, T* q_end, RunEnd how) {
        SpiRegs& regs = S::regs();
        if (q != q_end && (regs.STATR & spi_rxne) != 0u) {
            *q++ = static_cast<T>(regs.DATAR);
        }
        return {static_cast<uint16_t>(q - first), how};
    }

    static inline SpiDataSize bits_ = SpiDataSize::bits8;
    static inline PinSpeed pad_speed_ = PinSpeed::fast;
};

// =============================================================================
// The I2S face (20.3, 20.4.8, 20.4.9) - the CH32V303RC's and VC's
// =============================================================================

/// I2SCFG[1:0] (20.4.8): what the block is on the audio bus - a host
/// drives CK and WS, a client takes them - and which way SD goes.
enum class I2sMode : uint8_t {
    client_transmit = 0,
    client_receive = 1,
    host_transmit = 2,
    host_receive = 3,
};

constexpr bool i2s_mode_is_host(I2sMode m) {
    return m == I2sMode::host_transmit || m == I2sMode::host_receive;
}
constexpr bool i2s_mode_is_transmit(I2sMode m) {
    return m == I2sMode::host_transmit || m == I2sMode::client_transmit;
}

/// I2SSTD[1:0] (20.3.2): the four audio standards - Philips (WS one
/// clock ahead of the MSB), MSB-justified (WS with the MSB), LSB-justified
/// (the datum's last bit against the channel's end) and PCM, whose frame
/// PCMSYNC makes short (a one-bit WS) or long (thirteen bits).
enum class I2sStandard : uint8_t {
    philips = 0,
    msb_justified = 1,
    lsb_justified = 2,
    pcm = 3,
};

/// DATLEN[1:0]: the datum a channel carries. The fourth code is "not
/// allowed" (20.4.8) and has no name here.
enum class I2sDataLength : uint8_t { bits16 = 0, bits24 = 1, bits32 = 2 };

/// CHLEN: the channel's width. "Only when DATLEN = 00, the write operation
/// of this bit is meaningful, otherwise the channel length is fixed to 32
/// bits by hardware" - which is why a wider datum in a 16-bit channel is
/// REFUSED here rather than quietly given the channel it would get.
enum class I2sChannelLength : uint8_t { bits16 = 0, bits32 = 1 };

struct I2sConfig {
    I2sMode mode = I2sMode::host_transmit;
    I2sStandard standard = I2sStandard::philips;
    bool pcm_long_frame = false;       ///< PCMSYNC, and only under `pcm`
    I2sDataLength data = I2sDataLength::bits16;
    I2sChannelLength channel = I2sChannelLength::bits16;
    bool clock_idle_high = false;      ///< CKPOL: the level CK rests at
    bool master_clock_out = false;     ///< MCKOE: MCK = 256 x FS on its own pad, a host's
    uint8_t div = 2;                   ///< I2SDIV, 2..255 (0 and 1 forbidden), a host's
    bool odd = false;                  ///< ODD: the divider is 2 x div + 1
    bool dma_transmit = false;
    bool dma_receive = false;
};

/// The channel width the silicon really uses: 32 bits whenever the datum
/// is wider than 16 (20.4.8) - which a VALID configuration states itself.
constexpr bool i2s_channel_is_32(const I2sConfig& c) {
    return c.data != I2sDataLength::bits16 || c.channel == I2sChannelLength::bits32;
}

/// Data-register accesses per channel (20.3.2): ONE for a 16-bit datum,
/// in a 16- or a 32-bit channel, and TWO for a 24- or 32-bit one - the
/// high half first - which is also two DMA items.
constexpr uint8_t i2s_accesses_per_channel(const I2sConfig& c) {
    return c.data == I2sDataLength::bits16 ? 1u : 2u;
}

/// The whole divisor 20.3.3 puts between I2SxCLK and one audio frame: two
/// channels of 16 or 32 bits times the linear divider 2 x I2SDIV + ODD -
/// or, with MCK out, a fixed 256 (16 x 2 x 8 and 32 x 2 x 4 alike).
constexpr uint32_t i2s_frame_factor(bool channel_32, bool master_clock_out) {
    if (master_clock_out) {
        return 256u;
    }
    return channel_32 ? 64u : 32u;
}

/// The sampling frequency a divider pair produces from an I2S clock; 0 for
/// a forbidden divider (I2SDIV of 0 or 1).
constexpr uint32_t i2s_fs_hz(uint32_t i2s_clk_hz, uint8_t div, bool odd, bool channel_32,
                             bool master_clock_out) {
    if (div < 2u) {
        return 0u;
    }
    const uint32_t whole = 2u * static_cast<uint32_t>(div) + (odd ? 1u : 0u);
    const uint32_t den = i2s_frame_factor(channel_32, master_clock_out) * whole;
    return den == 0u ? 0u : i2s_clk_hz / den;
}

/// The same for a configuration, and the bit clock it puts on CK: two
/// channels of its width per frame.
constexpr uint32_t i2s_fs_hz(uint32_t i2s_clk_hz, const I2sConfig& c) {
    return i2s_fs_hz(i2s_clk_hz, c.div, c.odd, i2s_channel_is_32(c), c.master_clock_out);
}
constexpr uint32_t i2s_bit_clock_hz(uint32_t i2s_clk_hz, const I2sConfig& c) {
    return i2s_fs_hz(i2s_clk_hz, c) * (i2s_channel_is_32(c) ? 64u : 32u);
}

struct I2sPrescaler {
    uint8_t div = 0;
    bool odd = false;
};

/// The divider pair whose FS is nearest `fs_hz`, or nullopt when the
/// whole divisor would fall outside 4..511 (I2SDIV 2..255 with ODD). An
/// audio rate is hardly ever exact against a system clock, so the rounding
/// is to nearest and i2s_fs_hz() says what really comes out.
constexpr std::optional<I2sPrescaler> i2s_prescaler_for(uint32_t i2s_clk_hz, uint32_t fs_hz,
                                                        bool channel_32, bool master_clock_out) {
    if (i2s_clk_hz == 0u || fs_hz == 0u) {
        return {};
    }
    const uint32_t unit = i2s_frame_factor(channel_32, master_clock_out) * fs_hz;
    if (unit == 0u) {
        return {};
    }
    const uint32_t whole = (i2s_clk_hz + unit / 2u) / unit;
    if (whole < 4u || whole > 511u) {
        return {};
    }
    return I2sPrescaler{static_cast<uint8_t>(whole / 2u), (whole & 1u) != 0u};
}

/// The refusals the chapter owes: a field holding a value its register
/// field does not encode (DATLEN's fourth code among them); a host's
/// forbidden divider (I2SDIV 0 and 1, 20.4.9); PCM's long frame named
/// outside PCM; a datum wider than 16 bits asked of a 16-bit channel,
/// which is not a frame this block makes (20.4.8); and the master clock
/// asked of a client, whose MCKOE "is only used in I2S master mode".
constexpr bool i2s_config_valid(const I2sConfig& c) {
    if (static_cast<uint8_t>(c.mode) > 3u || static_cast<uint8_t>(c.standard) > 3u ||
        static_cast<uint8_t>(c.data) > 2u || static_cast<uint8_t>(c.channel) > 1u) {
        return false;
    }
    if (i2s_mode_is_host(c.mode) && c.div < 2u) {
        return false;
    }
    if (c.pcm_long_frame && c.standard != I2sStandard::pcm) {
        return false;
    }
    if (c.data != I2sDataLength::bits16 && c.channel == I2sChannelLength::bits16) {
        return false;
    }
    if (c.master_clock_out && !i2s_mode_is_host(c.mode)) {
        return false;
    }
    return true;
}

/// SPIx_I2S_CFGR for a configuration, I2SE clear.
constexpr uint16_t i2s_i2scfgr_of(const I2sConfig& c) {
    uint16_t v = spi_i2smod;
    v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.mode) << spi_i2s_cfg_shift));
    v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.standard) << spi_i2s_std_shift));
    if (c.pcm_long_frame) { v = static_cast<uint16_t>(v | spi_i2s_pcmsync); }
    v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.data) << spi_i2s_datlen_shift));
    if (c.channel == I2sChannelLength::bits32) { v = static_cast<uint16_t>(v | spi_i2s_chlen); }
    if (c.clock_idle_high) { v = static_cast<uint16_t>(v | spi_i2s_ckpol); }
    return v;
}

/// SPIx_I2SPR for a configuration.
constexpr uint16_t i2s_i2spr_of(const I2sConfig& c) {
    uint16_t v = c.div;
    if (c.odd) { v = static_cast<uint16_t>(v | spi_i2s_odd); }
    if (c.master_clock_out) { v = static_cast<uint16_t>(v | spi_i2s_mckoe); }
    return v;
}

/**
 * 20.3.4.3's and 20.3.5.2's way to stop a RECEIVER without starting
 * another transfer: which RXNE to wait for - the LAST one, or the one
 * before it - and how many I2S clock periods to spend after it before
 * I2SE is cleared. A 16-bit datum in a 32-bit channel waits 17 periods
 * after the second-to-last RXNE when LSB-justified and one after the last
 * otherwise; every other host combination waits one after the
 * second-to-last; a client clears I2SE at the last RXNE.
 */
struct I2sReceiveStop {
    bool after_last = false;
    uint8_t clock_periods = 0;
};

constexpr I2sReceiveStop i2s_receive_stop(const I2sConfig& c) {
    if (!i2s_mode_is_host(c.mode)) {
        return {true, 0};
    }
    if (c.data == I2sDataLength::bits16 && c.channel == I2sChannelLength::bits32) {
        return c.standard == I2sStandard::lsb_justified ? I2sReceiveStop{false, 17}
                                                         : I2sReceiveStop{true, 1};
    }
    return {false, 1};
}

/// Whether THIS PART has the I2S face of instance `n`: the datasheet's
/// I2S row (device::has_i2s) and one of the two instances that carry it.
constexpr bool i2s_present(uint8_t n) {
    return device::has_i2s && (n == 2u || n == 3u) && spi_present(n);
}

/**
 * The I2S clock this class feeds a master: SYSCLK itself (figure 3-3 - the
 * simple tree - and RCC_CFGR2's I2S2SRC and I2S3SRC, whose other choice is
 * a PLL3 the CH32V30x_D8C has and this class has not). A static Clock's
 * sysclk_hz; under a DynamicClock the program hands the rate in force to
 * i2s_prescaler_for() itself.
 */
template <typename C>
constexpr uint32_t i2s_clock_hz(C clock) {
    static_assert(C::is_static,
                  "brio I2s: the I2S clock is SYSCLK, which a static Clock states - under a "
                  "DynamicClock the rate in force is the program's to hand to i2s_prescaler_for()");
    (void)clock;
    return C::sysclk_hz;
}

/// The four pads an I2S link can claim, as a COLUMN of the instance (the
/// datasheet's table 3-4, afio.hpp): the word select, the bit clock, the
/// data line and the optional master clock. They are the SPI's pads under
/// other names - WS is NSS's, CK is SCK's, SD is MOSI's - and I2S3's move
/// with SPI3's remap code; MCK sits on a pad of its own (PC6 for I2S2, PC7
/// for I2S3) that no remap moves.
struct I2sPins {
    Pad ws{};
    Pad ck{};
    Pad sd{};
    Pad mck{};
    uint8_t remap = 0;
};

/// The column of instance `n` under `code`, MCK named or not.
constexpr I2sPins i2s_pins_for(uint8_t n, uint8_t code = 0, bool with_mck = false) {
    const I2sPadSet p = n == 3 ? afio_i2s3_pads(code) : afio_i2s2_pad_set;
    return I2sPins{.ws = p.ws, .ck = p.ck, .sd = p.sd, .mck = with_mck ? p.mck : Pad{},
                   .remap = static_cast<uint8_t>(n == 3 ? code : 0u)};
}

template <uint8_t n>
inline constexpr I2sPins i2s_default_pins = i2s_pins_for(n, 0);

/// Is that set a column of instance `n` on this part? WS, CK and SD are
/// required and must be the column's own pads, the code must be a column
/// the instance has, MCK is optional and must be the instance's OWN master
/// clock pad, and every pad named must be one the package bonds.
constexpr bool i2s_pins_valid(uint8_t n, const I2sPins& p) {
    if ((n != 2u && n != 3u) || p.remap >= (n == 3u ? afio_spi3_codes : 1u)) {
        return false;
    }
    const I2sPins column = i2s_pins_for(n, p.remap, true);
    if (!(p.ws == column.ws && p.ck == column.ck && p.sd == column.sd)) {
        return false;
    }
    if (p.mck.valid() && !(p.mck == column.mck)) {
        return false;
    }
    return pad_bonded(p.ws) && pad_bonded(p.ck) && pad_bonded(p.sd) &&
           (!p.mck.valid() || pad_bonded(p.mck));
}

/**
 * I2s<n> - SPI2's or SPI3's register block in its audio face, a RESOURCE
 * and nothing above it (the file header).
 *
 *   using Out = brio::I2s<2>;
 *   Out::bus_clock(true);
 *   Out::reset();
 *   Out::claim_pads<brio::i2s_default_pins<2>>(brio::I2sMode::host_transmit);
 *   const auto pre = *brio::i2s_prescaler_for(brio::i2s_clock_hz(clock), 48'000, false, false);
 *   (void)Out::configure({.mode = brio::I2sMode::host_transmit, .div = pre.div, .odd = pre.odd});
 *   Out::enable();
 *
 * WHAT IS SHARED WITH THE SPI FACE AND WHAT IS NOT. The gate, the reset,
 * the vector, DATAR, STATR and CTLR2's interrupt and DMA enables are one
 * block's and are reached here by the same names; CTLR1, the CRC
 * registers, SSOE, MODF and CRCERR are not used in I2S mode (20.3.1), and
 * I2SMOD is what decides which face answers. `configure()` writes I2SPR,
 * then I2S_CFGR whole, then CTLR2's two DMA bits - 20.3.4.1's order -
 * with I2SE and SPE clear on the way in, because I2SMOD "can only be set
 * when SPI or I2S is disabled" and every other field of both registers
 * "should be set when I2S is turned off".
 */
template <uint8_t n>
struct I2s {
    static_assert(device::has_i2s,
                  "brio I2s: this part has no I2S - the datasheets give two to the CH32V303RC and "
                  "VC alone (table 2-1-1's I2S row; the CH32V203's table 2-1 has none)");
    static_assert(n == 2u || n == 3u,
                  "brio I2s: the I2S face is SPI2's and SPI3's (20.3.1, tables 20-2 and 20-3)");

    I2s() = delete;

    using Block = Spi<n>;

    static constexpr uint8_t number = n;
    static constexpr Irq irq = Block::irq;
    /// The SPI face's two requests, the same two slots (20.3.9).
    static constexpr DmaSlot dma_rx_slot = Block::dma_rx_slot;
    static constexpr DmaSlot dma_tx_slot = Block::dma_tx_slot;

    static SpiRegs& regs() { return Block::regs(); }
    static volatile void* data_address() { return Block::data_address(); }

    // The gate, the reset and the column are the instance's.
    static void bus_clock(bool on) { Block::bus_clock(on); }
    static void reset() { Block::reset(); }
    static bool remap(uint8_t code) { return Block::remap(code); }

    /// The whole configuration (the class comment's order), refused by
    /// i2s_config_valid()'s rules with nothing written.
    static bool configure(const I2sConfig& c) {
        if (!i2s_config_valid(c)) {
            return false;
        }
        write_config(c);
        return true;
    }

    /// The same where the configuration is a CONSTANT: the refusal is a
    /// compile error on the line that wrote it.
    template <I2sConfig c>
    static void configure() {
        static_assert(i2s_config_valid(c),
                      "brio I2s: this configuration is not one chapter 20 has - a field holds a "
                      "value its register field does not encode, a host's divider is 0 or 1, PCM's "
                      "long frame is named outside PCM, a datum wider than 16 bits is asked of a "
                      "16-bit channel (20.4.8: that channel is 32 bits whatever CHLEN says), or "
                      "the master clock is asked of a client");
        write_config(c);
    }

    /// Back to the SPI face: I2SE and I2SMOD cleared together.
    static void select_spi_mode() {
        regs().I2SCFGR = static_cast<uint16_t>(regs().I2SCFGR & ~(spi_i2se | spi_i2smod));
    }
    static bool i2s_mode_selected() { return (regs().I2SCFGR & spi_i2smod) != 0u; }

    static void enable() { regs().I2SCFGR = static_cast<uint16_t>(regs().I2SCFGR | spi_i2se); }
    static bool enabled() { return (regs().I2SCFGR & spi_i2se) != 0u; }

    /// A TRANSMITTER stops after its last frame is out - TXE, then BSY
    /// down, 20.3.4.2's advice - each wait bounded, false when one ran
    /// out and I2SE was cleared regardless. A RECEIVER stops where it
    /// stands: the way to end a reception on a frame's edge is
    /// i2s_receive_stop()'s, which is timing the caller owns.
    static bool disable() {
        bool ok = true;
        if (enabled() && i2s_mode_is_transmit(mode())) {
            ok = wait_until([] { return tx_empty(); }) && wait_until([] { return !busy(); });
        }
        regs().I2SCFGR = static_cast<uint16_t>(regs().I2SCFGR & ~spi_i2se);
        return ok;
    }

    static I2sMode mode() {
        return static_cast<I2sMode>((regs().I2SCFGR & spi_i2s_cfg_mask) >> spi_i2s_cfg_shift);
    }
    static I2sStandard standard() {
        return static_cast<I2sStandard>((regs().I2SCFGR & spi_i2s_std_mask) >> spi_i2s_std_shift);
    }

    // ---- the generator -------------------------------------------------------

    /// I2SDIV and ODD alone, with the block disabled: what a program that
    /// changes the sampling rate and nothing else writes. MCKOE is kept.
    static bool prescaler(uint8_t div, bool odd) {
        if (div < 2u || enabled()) {
            return false;
        }
        regs().I2SPR = static_cast<uint16_t>((regs().I2SPR & spi_i2s_mckoe) | div |
                                             (odd ? spi_i2s_odd : uint16_t{0}));
        return true;
    }
    static uint8_t prescaler_div() { return static_cast<uint8_t>(regs().I2SPR & spi_i2s_div_mask); }
    static bool prescaler_odd() { return (regs().I2SPR & spi_i2s_odd) != 0u; }
    static bool master_clock_out() { return (regs().I2SPR & spi_i2s_mckoe) != 0u; }

    // ---- data and flags ------------------------------------------------------

    static void data(uint16_t v) { regs().DATAR = v; }
    static uint16_t data() { return regs().DATAR; }

    static bool tx_empty() { return (regs().STATR & spi_txe) != 0u; }
    static bool rx_ready() { return (regs().STATR & spi_rxne) != 0u; }
    /// BSY; 20.3.6.1: always LOW in master receive, whatever is moving.
    static bool busy() { return (regs().STATR & spi_bsy) != 0u; }
    static uint16_t status() { return regs().STATR; }
    /// CHSIDE, refreshed when TXE rises on a transmitter and when a datum
    /// lands on a receiver: false the left channel, true the right. No
    /// meaning under PCM, nor after an underrun or an overrun (20.3.6.4).
    static bool right_channel() { return (regs().STATR & spi_chside) != 0u; }

    /// UDR: a client transmitter clocked before software loaded DATAR
    /// (20.3.7.1), cleared by a read of STATR.
    static bool underrun() { return (regs().STATR & spi_udr) != 0u; }
    static void clear_underrun() { (void)regs().STATR; }
    /// OVR: DATAR then STATR, as on the SPI face (20.3.7.2).
    static bool overrun() { return (regs().STATR & spi_ovr) != 0u; }
    static void clear_overrun() {
        (void)regs().DATAR;
        (void)regs().STATR;
    }

    static void dma_requests(bool tx, bool rx) { Block::dma_requests(tx, rx); }
    static void rxne_interrupt(bool on) { Block::rxne_interrupt(on); }
    static void txe_interrupt(bool on) { Block::txe_interrupt(on); }
    static void error_interrupt(bool on) { Block::error_interrupt(on); }

    /// The raised-and-enabled sources: RXNE and TXE under their own
    /// enables, OVR and UDR under ERRIE (20.3.8). Nothing is cleared.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint16_t st = regs().STATR;
        const uint16_t en = regs().CTLR2;
        uint32_t up = 0;
        if ((en & spi_rxneie) != 0u && (st & spi_rxne) != 0u) { up |= SpiFlag::rxne; }
        if ((en & spi_txeie) != 0u && (st & spi_txe) != 0u) { up |= SpiFlag::txe; }
        if ((en & spi_errie) != 0u) {
            up |= st & SpiFlag::i2s_errors;
        }
        return up;
    }

    // ---- the pads (table 10-5) -----------------------------------------------

    /// Hand a column's pads to the face for a mode: WS and CK as push-pull
    /// alternate outputs on a host and floating inputs on a client, SD an
    /// output on a transmitter and a pulled-up input on a receiver, MCK an
    /// output where the column names it and the mode is a host's. The
    /// column's remap code is written here, SPI3's field moving the SPI
    /// face's pads with it.
    template <I2sPins pins>
    static bool claim_pads(I2sMode m) {
        static_assert(i2s_pins_valid(n, pins),
                      "brio I2s: these pads are not a column of this instance on this part - WS, "
                      "CK and SD must be the column's own (I2S2: PB12/PB13/PB15; I2S3: PA15/PB3/PB5 "
                      "or PA4/PC10/PC12), the master clock the instance's own pad (PC6 for I2S2, "
                      "PC7 for I2S3), and every pad one the package bonds");
        if (!remap(pins.remap)) {
            return false;
        }
        using Ws = Pin<pins.ws.port, pins.ws.pin>;
        using Ck = Pin<pins.ck.port, pins.ck.pin>;
        using Sd = Pin<pins.sd.port, pins.sd.pin>;
        if (i2s_mode_is_host(m)) {
            Ws::function();
            Ck::function();
        } else {
            Ws::input();
            Ck::input();
        }
        if (i2s_mode_is_transmit(m)) {
            Sd::function();
        } else {
            Sd::input(PinPull::up);
        }
        if constexpr (pins.mck.valid()) {
            using Mck = Pin<pins.mck.port, pins.mck.pin>;
            if (i2s_mode_is_host(m)) {
                Mck::function();
            } else {
                Mck::release();
            }
        }
        return true;
    }

    /// The column's pads back to floating inputs, driving nothing.
    template <I2sPins pins>
    static void release_pads() {
        static_assert(i2s_pins_valid(n, pins), "brio I2s: not a column of this instance");
        Pin<pins.ws.port, pins.ws.pin>::release();
        Pin<pins.ck.port, pins.ck.pin>::release();
        Pin<pins.sd.port, pins.sd.pin>::release();
        if constexpr (pins.mck.valid()) {
            Pin<pins.mck.port, pins.mck.pin>::release();
        }
    }

private:
    static void write_config(const I2sConfig& c) {
        SpiRegs& r = regs();
        r.I2SCFGR = static_cast<uint16_t>(r.I2SCFGR & ~spi_i2se);
        r.CTLR1 = static_cast<uint16_t>(r.CTLR1 & ~spi_spe);
        r.I2SPR = i2s_i2spr_of(c);
        r.I2SCFGR = i2s_i2scfgr_of(c);
        uint16_t v = static_cast<uint16_t>(r.CTLR2 & ~(spi_txdmaen | spi_rxdmaen | spi_ssoe));
        if (c.dma_transmit) { v = static_cast<uint16_t>(v | spi_txdmaen); }
        if (c.dma_receive) { v = static_cast<uint16_t>(v | spi_rxdmaen); }
        r.CTLR2 = v;
    }

    /// A bounded wait, the SPI face's.
    template <typename Pred>
    static bool wait_until(Pred pred) {
        for (uint32_t spins = 400'000u; spins != 0u; --spins) {
            if (pred()) {
                return true;
            }
        }
        return false;
    }
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

// 20.4.1's BR table on the two buses of the 144 MHz tree: SPI1 divides
// PCLK2 (= HCLK) and SPI2 divides PCLK1 (72 MHz), so the same code is
// two frequencies.
static_assert(spi_sck_hz(144'000'000UL, SpiClock::div2) == 72'000'000UL);
static_assert(spi_sck_hz(72'000'000UL, SpiClock::div2) == 36'000'000UL);
static_assert(spi_sck_hz(144'000'000UL, SpiClock::div256) == 562'500UL);
static_assert(spi_sck_hz(72'000'000UL, SpiClock::div256) == 281'250UL);
static_assert(spi_division(SpiClock::div2) == 2 && spi_division(SpiClock::div256) == 256);
static_assert(spi_rate_for(144'000'000UL, 72'000'000UL) == SpiClock::div2);
static_assert(spi_rate_for(144'000'000UL, 71'999'999UL) == SpiClock::div4);
static_assert(spi_rate_for(72'000'000UL, 1'000'000UL) == SpiClock::div128);
static_assert(spi_rate_for(144'000'000UL, 562'500UL) == SpiClock::div256);
// A ceiling below the bus over 256 is REFUSED, never rounded up.
static_assert(!spi_rate_for(144'000'000UL, 562'499UL).has_value());
static_assert(!spi_rate_for(0, 1'000'000UL).has_value());

static_assert(!spi_frame_is_halfword(SpiDataSize::bits8) &&
              spi_frame_is_halfword(SpiDataSize::bits16));
static_assert(spi_frame_mask(SpiDataSize::bits8) == 0xFFu && spi_data_bits(SpiDataSize::bits16) == 16);

// The four modes.
static_assert(!spi_mode_cpol(SpiMode::mode1) && spi_mode_cpha(SpiMode::mode1));
static_assert(spi_mode_cpol(SpiMode::mode2) && !spi_mode_cpha(SpiMode::mode2));

// The two buses, their gates and their vectors.
static_assert(spi_base_for(1) == 0x40013000UL && spi_base_for(2) == 0x40003800UL &&
              spi_base_for(3) == 0x40003C00UL && spi_base_for(4) == 0);
static_assert(spi_bus_for(1) == Bus::pb2 && spi_bus_for(2) == Bus::pb1 && spi_bus_for(3) == Bus::pb1);
static_assert(spi_gate_for(1) == rcc_pb2_spi1 && spi_gate_for(2) == rcc_pb1_spi2 &&
              spi_gate_for(3) == rcc_pb1_spi3);
static_assert(spi_irq_for(1) == Irq::spi1 && spi_irq_for(2) == Irq::spi2 &&
              spi_irq_for(3) == Irq::spi3);
static_assert(spi_bus_hz_at<1>(144'000'000UL) == 144'000'000UL);
static_assert(spi_bus_hz_at<2>(144'000'000UL) == 72'000'000UL);
static_assert(spi_bus_hz_at<3>(144'000'000UL) == 72'000'000UL);

// Table 11-5's four rows, through dma_engine.hpp - and SPI3's two on DMA2
// where the part has the instance (table 11-3).
static_assert(spi_dma_rx_channel(1) == 2 && spi_dma_tx_channel(1) == 3);
static_assert(spi_present(2) == (device::spi_count >= 2u));
static_assert(spi_present(3) == (device::spi_count >= 3u));
static_assert(!spi_present(3) ||
              (spi_dma_rx_slot(3) == DmaSlot{2, 1} && spi_dma_tx_slot(3) == DmaSlot{2, 2}));

// Tables 10-32 and 10-33: SPI1's two columns, SPI2's single one, SPI3's
// two - the first of which is SPI1's second.
static_assert(spi_column_count(1) == 2 && spi_column_count(2) == 1 && spi_column_count(3) == 2);
static_assert(spi_pins_for(3, 0).nss == Pad{'A', 15} && spi_pins_for(3, 0).sck == Pad{'B', 3} &&
              spi_pins_for(3, 0).miso == Pad{'B', 4} && spi_pins_for(3, 0).mosi == Pad{'B', 5});
static_assert(spi_pins_for(3, 1).nss == Pad{'A', 4} && spi_pins_for(3, 1).sck == Pad{'C', 10} &&
              spi_pins_for(3, 1).remap == 1u);
static_assert(spi_pins_for(1, 0).sck == Pad{'A', 5} && spi_pins_for(1, 0).nss == Pad{'A', 4});
static_assert(spi_pins_for(1, 1).sck == Pad{'B', 3} && spi_pins_for(1, 1).nss == Pad{'A', 15} &&
              spi_pins_for(1, 1).remap == 1u);
static_assert(spi_pins_for(2, 0).sck == Pad{'B', 13} && spi_pins_for(2, 0).mosi == Pad{'B', 15} &&
              spi_pins_for(2, 0).remap == 0u);

// The refusals spi_pins_valid() owes: a column the instance has not, two
// signals on one pad, and no SCK.
static_assert(!spi_pins_valid(2, SpiPins{.sck = {'B', 13}, .mosi = {'B', 15}, .remap = 1}));
static_assert(!spi_pins_valid(1, SpiPins{.sck = {'A', 5}, .mosi = {'A', 5}}));
static_assert(!spi_pins_valid(1, SpiPins{.mosi = {'A', 7}}));

// A value no register field encodes is refused too.
static_assert(!spi_config_valid(SpiConfig{.bits = static_cast<SpiDataSize>(2)}));
static_assert(!spi_config_valid(SpiConfig{.clock = static_cast<SpiClock>(8)}));
static_assert(!spi_config_valid(SpiConfig{.nss = static_cast<SpiNss>(3)}));

// The compile-time chooser and its refusal.
static_assert(SpiRateOf<72'000'000UL, 4'000'000UL>::clock == SpiClock::div32);
static_assert(SpiRateOf<144'000'000UL, 562'500UL>::clock == SpiClock::div256);

// The refusals spi_config_valid() owes the chapter.
static_assert(spi_config_valid(SpiConfig{}));
static_assert(!spi_config_valid(SpiConfig{.direction = SpiDirection::half_duplex_out, .crc = true}));
static_assert(!spi_config_valid(SpiConfig{.crc = true, .crc_polynomial = 0}));
static_assert(!spi_config_valid(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_output}));

// The control words: a mode 3 host at /16 under software select, and a
// hardware-output select with the receive request armed.
static_assert(spi_ctlr1_of(SpiConfig{.mode = SpiMode::mode3, .clock = SpiClock::div16}) ==
              (spi_cpha | spi_cpol | spi_mstr | (3u << spi_br_shift) | spi_ssm | spi_ssi));
static_assert(spi_ctlr2_of(SpiConfig{.nss = SpiNss::hardware_output, .dma_receive = true}) ==
              (spi_ssoe | spi_rxdmaen));
// A client under SSM keeps SSI low; the line modes are two bits.
static_assert(spi_ctlr1_of(SpiConfig{.role = SpiRole::client, .clock = SpiClock::div2}) == spi_ssm);
static_assert((spi_ctlr1_of(SpiConfig{.direction = SpiDirection::half_duplex_in}) & spi_bidimode) != 0u);
static_assert((spi_ctlr1_of(SpiConfig{.direction = SpiDirection::receive_only}) & spi_rxonly) != 0u);

// 20.3.3's arithmetic: FS is I2SxCLK over 32 or 64 bits a frame times the
// linear divider, or over 256 of it with MCK out - at SYSCLK = 144 MHz a
// 48 kHz frame of two 16-bit channels is a divider of 94 and comes out at
// 47872 Hz, and the ladder's two ends are refused rather than rounded.
static_assert(i2s_frame_factor(false, false) == 32u && i2s_frame_factor(true, false) == 64u);
static_assert(i2s_frame_factor(false, true) == 256u && i2s_frame_factor(true, true) == 256u);
static_assert(i2s_prescaler_for(144'000'000UL, 48'000UL, false, false)->div == 47u &&
              !i2s_prescaler_for(144'000'000UL, 48'000UL, false, false)->odd);
static_assert(i2s_fs_hz(144'000'000UL, 47, false, false, false) == 47'872UL);
static_assert(i2s_prescaler_for(144'000'000UL, 48'000UL, true, false)->div == 23u &&
              i2s_prescaler_for(144'000'000UL, 48'000UL, true, false)->odd);
static_assert(i2s_fs_hz(144'000'000UL, 23, true, true, false) == 47'872UL);
static_assert(i2s_fs_hz(144'000'000UL, 1, false, false, false) == 0u);
static_assert(i2s_prescaler_for(144'000'000UL, 8'000UL, false, true)->div == 35u);
static_assert(!i2s_prescaler_for(144'000'000UL, 1'000UL, false, false).has_value());
static_assert(!i2s_prescaler_for(8'000'000UL, 96'000UL, true, true).has_value());
static_assert(i2s_bit_clock_hz(144'000'000UL, I2sConfig{.div = 47}) == 47'872UL * 32u);

// The refusals i2s_config_valid() owes, and the two register words.
static_assert(i2s_config_valid(I2sConfig{}));
static_assert(!i2s_config_valid(I2sConfig{.div = 1}));
static_assert(i2s_config_valid(I2sConfig{.mode = I2sMode::client_receive, .div = 0}));
static_assert(!i2s_config_valid(I2sConfig{.pcm_long_frame = true}));
static_assert(i2s_config_valid(I2sConfig{.standard = I2sStandard::pcm, .pcm_long_frame = true}));
static_assert(!i2s_config_valid(I2sConfig{.data = I2sDataLength::bits24}));
static_assert(i2s_config_valid(I2sConfig{.data = I2sDataLength::bits24,
                                         .channel = I2sChannelLength::bits32}));
static_assert(!i2s_config_valid(I2sConfig{.data = static_cast<I2sDataLength>(3),
                                          .channel = I2sChannelLength::bits32}));
static_assert(!i2s_config_valid(I2sConfig{.mode = I2sMode::client_transmit,
                                          .master_clock_out = true}));
static_assert(i2s_i2scfgr_of(I2sConfig{.mode = I2sMode::host_receive,
                                       .standard = I2sStandard::pcm, .pcm_long_frame = true,
                                       .data = I2sDataLength::bits32,
                                       .channel = I2sChannelLength::bits32,
                                       .clock_idle_high = true}) ==
              (spi_i2smod | (3u << spi_i2s_cfg_shift) | (3u << spi_i2s_std_shift) |
               spi_i2s_pcmsync | (2u << spi_i2s_datlen_shift) | spi_i2s_chlen | spi_i2s_ckpol));
static_assert(i2s_i2spr_of(I2sConfig{.master_clock_out = true, .div = 3, .odd = true}) ==
              (3u | spi_i2s_odd | spi_i2s_mckoe));
static_assert(i2s_accesses_per_channel(I2sConfig{}) == 1u &&
              i2s_accesses_per_channel(I2sConfig{.data = I2sDataLength::bits24,
                                                 .channel = I2sChannelLength::bits32}) == 2u);
// 20.3.4.3's stops.
static_assert(i2s_receive_stop(I2sConfig{.mode = I2sMode::host_receive,
                                         .standard = I2sStandard::lsb_justified,
                                         .channel = I2sChannelLength::bits32}).clock_periods == 17u);
static_assert(i2s_receive_stop(I2sConfig{.mode = I2sMode::host_receive,
                                         .channel = I2sChannelLength::bits32}).after_last);
static_assert(!i2s_receive_stop(I2sConfig{.mode = I2sMode::host_receive}).after_last);
static_assert(i2s_receive_stop(I2sConfig{.mode = I2sMode::client_receive}).after_last);

// The columns: I2S2 on SPI2's pads with MCK on PC6, I2S3 on SPI3's with MCK
// on PC7 in either column; an MCK that is not the instance's is refused.
static_assert(i2s_pins_for(2).ws == Pad{'B', 12} && i2s_pins_for(2).ck == Pad{'B', 13} &&
              i2s_pins_for(2).sd == Pad{'B', 15} && !i2s_pins_for(2).mck.valid() &&
              i2s_pins_for(2, 0, true).mck == Pad{'C', 6});
static_assert(i2s_pins_for(3).ws == Pad{'A', 15} && i2s_pins_for(3, 1).ck == Pad{'C', 10} &&
              i2s_pins_for(3, 1, true).mck == Pad{'C', 7} && i2s_pins_for(3, 1).remap == 1u);
static_assert(!i2s_pins_valid(2, I2sPins{.ws = {'B', 12}, .ck = {'B', 13}, .sd = {'B', 15},
                                         .mck = {'C', 7}}));
static_assert(!i2s_pins_valid(1, i2s_pins_for(2)));
static_assert(!i2s_pins_valid(2, I2sPins{.ws = {'B', 12}, .ck = {'B', 13}, .sd = {'B', 14}}));
static_assert(i2s_present(2) == device::has_i2s && i2s_present(3) == device::has_i2s);

}  // namespace brio
