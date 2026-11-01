/*
 * spi.hpp
 *
 * The ARM PrimeCell Synchronous Serial Port (PL022), written once for
 * every family that carries it: the SPI of the RP2040 and the RP2350, as
 * a bus host an application's active objects share through the bus
 * arbiter, or as a client at the other end of the wire. This is an IP
 * stratum - a directory of brio/ named for a peripheral design that
 * several vendors license into their chips (an "IP block") rather than
 * for a silicon - and it sits where a core stratum sits, above util/ and
 * below the families (docs/pl022/README.md, this file's document, and
 * the rule in docs/design/overview.md: "A core stratum sits between
 * util/ and the families that share a core").
 *
 * The three layers every brio bus driver has are all here:
 *
 *  Pl022Ssp<Chip, n>     the resource, named for the IP: which instance,
 *                        where its registers are, its reset and its
 *                        interrupt line (both asked of the family), the
 *                        two control registers under the block's rule
 *                        (written with SSE clear), the prescaler pair,
 *                        the data and status verbs, the interrupt trio
 *                        and the ISR body that reads the raised-and-
 *                        enabled sources. It decides nothing.
 *  Pl022Host<Chip, n, pins, TxEngine, RxEngine, Placement>
 *                        the engine util/spi_bus.hpp's SpiBus drives (the
 *                        arbiter: util/bus_master.hpp's active object
 *                        that serializes every client's requests to one
 *                        bus). Its Request is the transaction descriptor
 *                        every stratum's SPI host takes
 *                        (docs/design/spi-bus.md: a chip select, a D/C
 *                        line, a command phase, a data phase with
 *                        optional out and in, the per-request mode / rate
 *                        / frame size, the polled or asynchronous way to
 *                        run it), so an application written over SpiBus
 *                        on another target runs here unchanged.
 *  Pl022Client<Chip, n, pins>
 *                        the other end of the wire, a thin surface with
 *                        an ISR body: a client is a protocol and which
 *                        one is the application's.
 *
 * An application never writes these names. A family supplies the chip's
 * facts as a traits type - a struct of types, constants and static
 * functions this file reads, checked against the `Pl022Chip` concept
 * below - and keeps, in its own header, the public names apps use as
 * aliases of the three templates:
 *
 *   RP2040   Rp2040Pl022 in brio/rp2040/spi.hpp: Pl022<n>,
 *            SpiHost<n, pins, TxEngine, RxEngine, Placement> and
 *            SpiClient<n, pins>
 *   RP2350   Rp2350Pl022 in brio/rp2350/spi.hpp: the same three names
 *   host     SimPl022 in brio/host/sim_pl022.hpp: a block made of RAM,
 *            for the host suite test_pl022, which names the templates
 *
 * The smallest use, a host on two DMA channels under the arbiter on the
 * RP2040 as brio/rp2040/spi.hpp and docs/rp2040/spi.md spell it:
 *
 *   constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
 *   using Bus = brio::SpiHost<0, pins, brio::DmaTxEngine<4, uint16_t>,
 *                             brio::DmaRxEngine<5, uint16_t>>;
 *   using Spi = brio::SpiBus<Bus, P, 4>;              // the arbiter
 *   using Cs = brio::Pin<17>;
 *
 *   brio::Dma::init();                                // before the host binds its engines
 *   Bus::init(clock, 8'000'000);                      // a ceiling for the whole bus
 *   Cs::output(true);                                 // the select idles high
 *
 *   Bus::Request r{};
 *   r.cs = Cs::ref();
 *   r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
 *   r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
 *   r.clock = brio::SpiClocks::div4;
 *   r.mode = brio::SpiMode::mode0;
 *   r.reply = brio::reply_to<Display, brio::SpiDone>();
 *   brio::post<Spi>(r);                               // asynchronous: polled stays false
 *
 *   extern "C" void isr_dma_0() {
 *       if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
 *   }
 *
 * Who calls what, and from where.
 *  - init(), rebase() and release() are setup verbs, in main: init()
 *    after the clock and before interrupts are enabled globally,
 *    rebase() with the bus idle. init() turns the ceiling it is given
 *    into a divisor of the rate the family's traits call SSPCLK, so the
 *    clock comes first.
 *  - A requester - an active object that uses the bus, not the SPI
 *    client below - never calls the host: it posts a Request to the
 *    arbiter from its own dispatch (an active object's handler - an
 *    active object is a component with its own event queue, served one
 *    event at a time by the kernel) or from main, and gets an SpiDone
 *    event back. The arbiter calls start() from its dispatch, in main
 *    context (code that is not an interrupt handler: main and every
 *    dispatch), one request at a time. start() follows the engine
 *    contract of util/bus_master.hpp: true means the request completed
 *    inside the call.
 *  - start() of a polled request runs the whole transaction before it
 *    returns: it waits for every frame, bounded by one budget for the
 *    whole transaction, with interrupts left on (no handler that holds
 *    the loop off can overrun it, below), and spends the request's
 *    cs_setup_us in the family's busy-wait; a polled read long enough to
 *    gain from the engines rides them, start() waiting on their
 *    completion (which the DMA line's body records). An asynchronous request is
 *    the DMA engines' (docs/design/spi-bus.md, "Two ways to run a
 *    request"): start() arms them and returns, and completes it on the
 *    DMA line - or, too short to repay them, runs it polled and returns
 *    done; on a host with no engines bound it is refused at once
 *    (spi_rejected, nothing on the wire). Pl022Host below has the rule.
 *  - dma_isr() is the body of the DMA line the engines report on, the
 *    host's one interrupt body: the program's handler for that line calls
 *    it and nothing else does. It returns true when the transaction has
 *    just completed (the select released), the edge on which the handler
 *    posts TransferDone to the arbiter, and runs to completion, as every
 *    interrupt handler in brio does. The host arms no source of the
 *    block's own interrupt line: that vector is a client's.
 *  - What masks: nothing. init() leaves the instance's own line disabled
 *    and release() disables it, and nothing of the transfer path
 *    (start(), dma_isr()) takes a critical section. The family's
 *    set_bits() and clear_bits() are one bus write where the chip has
 *    atomic register aliases (both RP chips use theirs), a guarded
 *    read-modify-write otherwise.
 *  - Storage: a Request's buffers are lent (`Lease::reply`,
 *    kernel/borrowed.hpp) from the post until its SpiDone arrives, and
 *    the requester leaves them untouched for that whole time; the arbiter
 *    lends the request itself to start() for the call, a request that
 *    completes inside it keeping nothing and an engined one copying what
 *    its tenure (the time one transaction holds the bus) needs into
 *    `tenure_`.
 *  - The client is not driven by the arbiter: init() and enable() (with
 *    the first answer queued, the select still high) in main, then its
 *    verbs, which the program calls from its handler for the instance's
 *    vector (docs/rp2040/spi.md shows one) or from main, and none of
 *    which waits: writable() and write() are one register access each,
 *    poll() a status read and, when a frame waits, the data read, and
 *    isr() a read of the masked status and a write to the clear
 *    register when a clearable source is up.
 *
 * This file knows no chip, and no core either. It includes kernel/ and
 * util/ and nothing else: no vendor header, no family header, no core
 * stratum, and it never names a reset controller, a clock, a pad
 * function, an interrupt controller or a DMA request. Even the
 * microsecond busy-wait a chip select's setup time is spent on arrives
 * as a type of the family's, whose verb this file calls without naming
 * it - a PL022 may sit on a core brio/cortexm/ knows nothing about.
 * What a family owes it is the `Pl022Chip` concept below,
 * satisfied by one traits type the family's own header defines; that
 * header also keeps the public names apps use (`Pl022<n>`, `SpiHost<n,
 * pins, ...>`, `SpiClient<n, pins>`) as aliases of these three
 * templates, so nothing above ever spells a Chip.
 *
 * What the IP does, and how it shapes the engine:
 *  - a host or a client, Motorola SPI, TI or Microwire framing, frames
 *    of 4 to 16 bits (DSS), two FIFOs whose depth is a synthesis
 *    parameter and therefore the family's (`Chip::fifo_depth`: 8 on
 *    both RP chips);
 *  - the bit rate is SSPCLK / (CPSDVSR x (1 + SCR)), the prescaler even
 *    in 2..254 and the serial clock rate 0..255, so the reachable
 *    divisors run from 2 to 65024 and a rate is refused rather than
 *    rounded up past the slowest;
 *  - a client needs SSPCLK at least twelve times its own clock: the
 *    input is double-synchronized, which is the block's ratio and not a
 *    chip's;
 *  - control registers are written with the port disabled (SSE clear),
 *    so a change of mode, rate or width costs a disable/enable pair -
 *    paid only when something moved. LBM and SOD are the two live bits,
 *    changeable under SSE;
 *  - four maskable sources on one interrupt line per instance: the
 *    receive FIFO at or above half (RXIM), the receive timeout (RTIM: a
 *    frame waiting and no clock for 32 bit periods), the transmit FIFO
 *    at or below half (TXIM), the receive overrun (RORIM). Only the
 *    timeout and the overrun clear through SSPICR; the two FIFO levels
 *    clear by moving data. The client answers on them; the HOST arms
 *    none (declined: a handler a FIFO level would cost the core more
 *    than the wire's own frames at every rate a program uses, where the
 *    engines cost a fixed sum - docs/design/spi-bus.md, "Two ways to run
 *    a request");
 *  - one DMA request per FIFO (SSPDMACR): the host's asynchronous way;
 *  - LBM loops the transmitter into the receiver with nothing on the
 *    wire - a self-test with no peer, which is why it is a host verb
 *    here: the host re-applies its whole configuration at a request that
 *    changes mode, rate or width, so the loop-back has to be part of the
 *    applied state or it would be lost at the first such request.
 *
 * The FIFOs are the polled loop's write-ahead. A FIFO's
 * worth of frames is in flight at once - written with no flag read, since
 * a transmit FIFO that holds fewer than its depth cannot be full and a
 * receive FIFO that is handed back no more than its depth cannot overrun
 * (12.3.3.4 and 12.3.3.5 of the RP2350 data sheet, the depth the
 * family's `fifo_depth`) - and every frame that comes back is the one
 * witness the bus has room for one more: the receive side counts what
 * the transmit side only promised.
 *
 * The polled loop has two shapes. The receive shape (`run_duplex`)
 * primes the transmit FIFO with the first `fifo_depth` frames and then,
 * per frame, waits for RNE, reads the frame - stored, or thrown away
 * when the request has no in buffer - and writes the one `fifo_depth`
 * ahead: three accesses of the block a frame (SSPSR, SSPDR in, SSPDR
 * out), the frames in flight - written and not yet read - never more
 * than the receive FIFO holds, so no handler that holds the loop off can
 * overrun it. The transmit-only shape (`run_write`) paces on TNF and
 * reads nothing per frame - two accesses a frame - letting the receive
 * FIFO overrun by design with RORIM masked, and at its tail waits for
 * BSY to clear, drains the receive FIFO and clears the overrun; that
 * tail is up to a FIFO's worth of reads after the wire, so the shape is
 * taken only for a write longer than the FIFO at the one rate where the
 * receive shape falls behind the wire (clk_peri / 2 on 8-bit frames,
 * measured on both of the RP2350's cores). No accessor call on either,
 * the width a template parameter, the buffers advancing pointers, the
 * waits bounded by one budget for the whole transaction. Both read every
 * field through the Request they were lent and copy nothing: a polled
 * request completes inside start(), so nothing of it is needed after the
 * return (util/bus_master.hpp's contract).
 *
 * The asynchronous way is the DMA engines (Pl022Host below has the rule,
 * the short request's arithmetic above it). What an engined transaction
 * needs after start() returned - the request's words - is copied into
 * `tenure_` as inline stores, never a call. The phase boundary drains on
 * every way: the first data frame goes out only when the last command
 * frame has come back (the polled loop's command turn ends on its last
 * echo, a command block on the receive channel's completion), so D/C
 * flips between them on the wire. The receive FIFO is not flushed at
 * start(): every transaction takes back every frame it wrote, so a frame
 * can stand in it only after a fault, and the fault paths drain it.
 *
 * The Request is laid out without padding: the four words first (the two
 * out buffers, the in buffer, the reply), then the two half-words (the
 * length, the prescaler pair), then the bytes - 28 bytes where the
 * family's PinRef is one byte, and every copy the kernel and the arbiter
 * make of it is that much shorter. The field names are the ones every
 * stratum's Request carries (docs/design/spi-bus.md, "Realizations")
 * and do not move; the order is this file's.
 *
 * `apply()` is one compare: the prescaler pair, the mode and the width
 * are folded into the word the block takes (SSPCPSR over SSPCR0,
 * `spi_control_word`) and held against the word last applied; only a
 * change pays the disable/enable pair the chapter demands. A request
 * above the bus's ceiling is clamped on the slow path alone.
 *
 * The selects. The host's chip select is a pin the Request carries
 * (docs/design/spi-bus.md's arrangement on every target): a device
 * reading a multi-frame transaction inside one select window wants a
 * select that stays low, which the block's own frame signal SSPFSSOUT is
 * not in every mode - with SPH = 0 it pulses high between the frames of a
 * continuous transfer, with SPH = 1 it stays low through one and rises
 * only after the last frame (RP2040 4.4.3.10 to 4.4.3.13). A host whose
 * pin set names `cs` hands that pad to the block all the same, as the
 * frame signal and nothing else: a select per frame in modes 0 and 2,
 * per run of back-to-back frames in modes 1 and 3, the request's own
 * `cs` then left null - exactly what a client of this block takes in
 * modes 0 and 2, and what a device whose every frame is a transaction
 * wants. The client's select is the pad: the PL022 in slave
 * mode frames on SSPFSSIN, so a client's `cs` goes to the peripheral and
 * `selected()` reads the same pad. And with SPH = 0 (modes 0 and 2) the
 * client frames on the select: it takes one frame per select window and
 * ignores the rest while the select stays low, so a host holding a
 * select for a whole transaction reaches it for more than one frame only
 * in modes 1 and 3.
 *
 * The dark listener is the pad and not SOD. `Pl022Client::drive_output`
 * hands the transmit pad to the peripheral or takes it away to no
 * function with the family's dark config - an input with no pull - because
 * what SOD does to the pad is a fact of the chip around the block and not
 * of the block: the block's pad-enable output nSSPOE reaches the pad on
 * one silicon this file has met and not on the other (the RP2350 tristates
 * a deselected client and one under SOD, the RP2040 keeps both driven -
 * each family's document). Taking the pad away is right either way. It is
 * not the pad's reset state: that carries a pull-down on both chips, and a
 * far host's receive pad carries a pull-up, so a dark line released to
 * reset sat between the two (measured: a foreign host read neither level).
 * `sod()` stays a bit of the resource, for a program that wants to see
 * what its own silicon does with it.
 *
 * The engine slots: a transmit and a receive engine, both or neither
 * (the data phase is full duplex and the transaction's completion is the
 * receive block's), told the instance's two requests at arm(). A phase
 * moves as one block each way - the data phase, and the command phase
 * when it repays a block of its own, chained to the data block from the
 * interrupt; a shorter command phase runs polled inside start(). One
 * interrupt a block: every frame that came back was sent, so the receive
 * block's completion proves the transmit block's, and the transmit engine
 * is armed to report a bus error and nothing else (the family's
 * DmaReport::errors). The beat is the frame: engines whose element is 16
 * bits wide - SSPDR's width - carry both frame sizes, each block at its
 * frame's width; byte engines carry 8-bit frames, and a 16-bit request
 * then runs polled inside start(), as does one whose buffers are not
 * aligned to a half-word. Frames in a byte buffer: one byte per
 * 8-bit frame, two bytes low-first per 16-bit frame - the rule of the
 * Request every stratum's SPI host takes (docs/design/spi-bus.md), which
 * on a little-endian core is a half-word in memory.
 *
 * Where the hot path runs is the program's choice, by the host's last
 * template parameter, util/placement.hpp's tag: start() and the DMA
 * line's body (dma_isr()) each run through `Hot<Placement>` - where they
 * would be without it under `InFlash`, the default (the DMA body inline
 * in the app's vector, start() a function in the flash), one flattened
 * instance each in `.fast_text.brio` under
 * `InRam`, on a family whose chip traits say `places_fast_text`. On a
 * core executing in place behind a cache what that buys is the cold
 * transaction, the one whose code the cache has not got (the family's
 * platform document, "A handler in SRAM"). The engines' launch is one
 * body under either tag (shared()), and a change of mode, rate or width
 * and a polled wait that ran out stay out of line in the flash.
 *
 * Not built, each with its reason in docs/pl022/README.md: the National
 * Semiconductor Microwire transaction (half duplex, a control frame and then the
 * slave's answer, where this engine's Request is full duplex and one
 * frame wide), the client on the DMA engines.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <concepts>
#include <optional>
#include <type_traits>
#include <utility>

#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/placement.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// ---- what a family owes this file ---------------------------------------------

/**
 * The absent engine, as the concept measures a family's
 * `engines_distinct` against it: `present` is all this file ever asks of
 * an engine slot, and a family's own empty-slot tag says the same.
 */
struct Pl022AbsentEngine {
    Pl022AbsentEngine() = delete;
    static constexpr bool present = false;
};

/**
 * What the host's two ways cost the core of one family, in that core's
 * cycles: the figures the short asynchronous request is decided from
 * (Pl022Host's "THE SHORT ASYNCHRONOUS REQUEST", below). They are the
 * core's and the flash's around the block, never the block's, so each
 * family states its own, measured on its own cores, with the source and
 * the arithmetic in its document.
 */
struct Pl022Costs {
    /// E - F for a request that reads (an in buffer): what the engines
    /// cost a transaction beyond a polled transaction's fixed part - the
    /// launch, the one interrupt, the completion against the polled loop's
    /// entry and exit.
    uint16_t engines_over_read;
    /// E - F for a request that only writes (no in buffer).
    uint16_t engines_over_written;
    /// The receive loop's shortest frame: its three accesses of the block
    /// and the stored frame, at the fastest rate, where the loop and not
    /// the wire sets the pace.
    uint16_t read_turn;
    /// The transmit-only loop's shortest frame, the same way.
    uint16_t write_turn;
};

/**
 * THE CONTRACT: what a family's chip-traits type states so that this
 * file can drive the block without knowing which silicon carries it.
 * Every member is here because some line below needs it.
 *
 * Two things this concept names and cannot check here: `sspclk_hz(clock)`,
 * a template over the family's OWN clock types, of which this file knows
 * none; and the VERB over `DelayRate`, which is found by argument-
 * dependent lookup on a type this file only has as a name. Each has a
 * concept of its own - `Pl022ChipClock` and `Pl022ChipDelay` - checked
 * where the thing is in hand, which for both is the host.
 */
template <typename C>
concept Pl022Chip =
    requires {
        /// The register block of one instance, as the family's device
        /// description spells it: the member names are the PL022's own
        /// (SSPCR0, SSPCR1, SSPDR, SSPSR, ...).
        typename C::Regs;
        /// What the family's interrupt controller calls a line.
        typename C::Irq;
        /// That controller: enable / disable one line. A family may carry
        /// an NVIC under one architecture and something else under
        /// another, so this file never names one.
        typename C::Interrupts;
        /// The family's pin-set type, one pad per signal: which pads
        /// exist and which function routes an SPI to them is the
        /// family's table, never this file's.
        typename C::Pins;
        /// A pin named at RUN TIME - what a Request carries as its chip
        /// select and its D/C line, since those are the application's
        /// pins and not the peripheral's.
        typename C::PinRef;
        /// What the family's DMA calls a peripheral request.
        typename C::DmaRequest;
        /// The family's microsecond busy-wait, as the precomputed factor
        /// a caller stores once: `cs_setup_us` is spent on it. The VERB
        /// over this type is `Pl022ChipDelay` below.
        typename C::DelayRate;
        /// How many instances this family carries, how deep the two
        /// FIFOs were synthesized, and what the pin set puts in a signal
        /// that is not routed.
        { C::instances } -> std::convertible_to<uint8_t>;
        { C::fifo_depth } -> std::convertible_to<uint8_t>;
        { C::no_pad } -> std::convertible_to<uint8_t>;
        /// What the host's two ways cost this family's core (Pl022Costs):
        /// from them a host with engines decides which asynchronous
        /// request is too short to repay them and runs it polled, and
        /// which polled read is long enough to ride them.
        { C::costs } -> std::convertible_to<Pl022Costs>;
    } &&
    requires(volatile uint32_t& reg, uint32_t bits, typename C::Pins pins, uint8_t instance,
             typename C::Irq line, typename C::PinRef line_out, uint32_t hz) {
        /// Instance 0 answers every per-instance question; so does every
        /// other instance the family has. Each is a template on the
        /// instance number so the answer is a compile-time constant -
        /// a register address, a line, a request.
        { C::template regs<0>() } -> std::same_as<typename C::Regs&>;
        { C::template irq<0>() } -> std::same_as<typename C::Irq>;
        /// The block from its reset state, held, and whether it is out:
        /// on one family a reset controller, on another a clock gate.
        { C::template reset<0>() } -> std::same_as<bool>;
        { C::template released<0>() } -> std::same_as<bool>;
        C::template hold<0>();
        /// The two request numbers a host hands its engines.
        { C::template tx_request<0>() } -> std::same_as<typename C::DmaRequest>;
        { C::template rx_request<0>() } -> std::same_as<typename C::DmaRequest>;
        /// One bus write that sets or clears bits of a register, with no
        /// read-modify-write: the enable and the two live bits share
        /// SSPCR1, and the interrupt mask is touched from two contexts.
        C::set_bits(reg, bits);
        C::clear_bits(reg, bits);
        /// The pads: is this pin set legal for this instance, and which
        /// pad carries each signal (`no_pad` for one the set leaves out).
        { C::pins_valid(instance, pins) } -> std::same_as<bool>;
        { C::sck_pad(pins) } -> std::same_as<uint8_t>;
        { C::tx_pad(pins) } -> std::same_as<uint8_t>;
        { C::rx_pad(pins) } -> std::same_as<uint8_t>;
        { C::cs_pad(pins) } -> std::same_as<uint8_t>;
        /// Then the PAD ITSELF as a type, with the function code that
        /// routes an SPI to it and the electrical setup each signal
        /// wants; the three verbs this file calls on a pad are handing it
        /// over, taking it back and reading its level (a client's select).
        /// The two receive pads are asked SEPARATELY because they are two
        /// electrical situations: a host's answer line may have nothing
        /// driving it (no client, or one listening dark), while a
        /// client's command line is driven by the host whenever it
        /// matters.
        C::template Pad<0>::function(C::pad_function, C::sck_pad_config());
        C::template Pad<0>::function(C::pad_function, C::tx_pad_config());
        C::template Pad<0>::function(C::pad_function, C::host_rx_pad_config());
        C::template Pad<0>::function(C::pad_function, C::client_rx_pad_config());
        C::template Pad<0>::function(C::pad_function, C::cs_pad_config());
        /// A client's answer line while it listens DARK: handed to NO
        /// function - nothing drives it - and set up as the dark config
        /// says, which carries no pull: the far host's own pull on its
        /// receive pad holds the wire, and a pull here would fight it.
        C::template Pad<0>::function(C::no_function, C::dark_pad_config());
        { C::template Pad<0>::read() } -> std::same_as<bool>;
        C::template Pad<0>::release();
        /// The run-time pin, driven high and low around a transaction.
        line_out.set();
        line_out.clear();
        /// Two engines of one host must not name one DMA channel; what
        /// "the same channel" means is the family's DMA's business.
        { C::template engines_distinct<Pl022AbsentEngine, Pl022AbsentEngine>() }
            -> std::same_as<bool>;
        /// The busy-wait's factor for a rate, computed once per clock
        /// change so no division runs at wait time.
        { C::delay_rate(hz) } -> std::same_as<typename C::DelayRate>;
        /// The line's two verbs, on the family's own controller.
        C::Interrupts::enable(line);
        C::Interrupts::disable(line);
    };

/**
 * WHICH CLOCK OF THIS FAMILY'S TREE IS SSPCLK - the half of the contract
 * that is a template over the family's clock types and so can only be
 * checked where a clock is in hand (Pl022Host::init).
 */
template <typename C, typename Clock>
concept Pl022ChipClock = requires(Clock clock) {
    { C::sspclk_hz(clock) } -> std::same_as<uint32_t>;
};

/**
 * THE VERB THAT BELONGS TO `DelayRate`, and so cannot be named here: a
 * microsecond busy-wait is made of a core's own counter, and the PL022
 * sits on cores this file must know nothing about. The family hands over
 * its own type and this file calls that type's verb, found where the
 * type is - which is why `delay_us` is written unqualified and why the
 * check is a concept of its own, where the type is in hand.
 */
template <typename C>
concept Pl022ChipDelay = requires(typename C::DelayRate rate, uint32_t us) {
    { delay_us(rate, us) } -> std::convertible_to<bool>;
};

// =============================================================================
// The vocabulary
// =============================================================================

/// SPO and SPH as the four Motorola modes (the mode's bit 1 is the
/// polarity, bit 0 the phase).
enum class SpiMode : uint8_t { mode0 = 0, mode1 = 1, mode2 = 2, mode3 = 3 };
constexpr bool spi_mode_cpol(SpiMode m) { return (static_cast<uint8_t>(m) & 2u) != 0u; }
constexpr bool spi_mode_cpha(SpiMode m) { return (static_cast<uint8_t>(m) & 1u) != 0u; }

/// SSPCR0.FRF: the three framings.
enum class SpiFormat : uint8_t { motorola = 0, ti = 1, microwire = 2 };

/// The bit rate: sspclk / (cpsdvsr x (1 + scr)), cpsdvsr even in
/// 2..254, scr 0..255.
struct SpiClock {
    uint8_t cpsdvsr = 2;
    uint8_t scr = 7;
    constexpr bool valid() const { return cpsdvsr >= 2u && (cpsdvsr & 1u) == 0u; }
    constexpr uint32_t divisor() const { return static_cast<uint32_t>(cpsdvsr) * (static_cast<uint32_t>(scr) + 1u); }
    constexpr bool operator==(const SpiClock& o) const { return cpsdvsr == o.cpsdvsr && scr == o.scr; }
};

/// The named divisors of the rate field every stratum's SPI Request
/// carries (docs/design/spi-bus.md: the reference rate over a power of
/// two), and any even divisor up to 65024.
struct SpiClocks {
    static constexpr SpiClock div2{2, 0};
    static constexpr SpiClock div4{2, 1};
    static constexpr SpiClock div8{2, 3};
    static constexpr SpiClock div16{2, 7};
    static constexpr SpiClock div32{2, 15};
    static constexpr SpiClock div64{2, 31};
    static constexpr SpiClock div128{2, 63};
    static constexpr SpiClock div256{2, 127};
};

constexpr uint32_t spi_sck_hz(uint32_t pclk, SpiClock c) { return pclk / c.divisor(); }

/// The fastest setting whose SCK does not exceed `max_hz` at `pclk`;
/// nullopt when even 254 x 256 does not slow it enough - refused,
/// never rounded up.
constexpr std::optional<SpiClock> spi_clock_for(uint32_t pclk, uint32_t max_hz) {
    if (pclk == 0u || max_hz == 0u) {
        return {};
    }
    std::optional<SpiClock> best{};
    uint32_t best_hz = 0;
    for (uint32_t cpsdvsr = 2; cpsdvsr <= 254u; cpsdvsr += 2u) {
        // The smallest 1 + scr that keeps pclk / (cpsdvsr x (1 + scr)) <= max.
        const uint64_t need = (static_cast<uint64_t>(pclk) + static_cast<uint64_t>(cpsdvsr) * max_hz - 1u) /
                              (static_cast<uint64_t>(cpsdvsr) * max_hz);
        if (need > 256u) {
            continue;
        }
        const uint32_t scr1 = need == 0u ? 1u : static_cast<uint32_t>(need);
        const uint32_t hz = pclk / (cpsdvsr * scr1);
        if (hz > best_hz) {
            best_hz = hz;
            best = SpiClock{static_cast<uint8_t>(cpsdvsr), static_cast<uint8_t>(scr1 - 1u)};
        }
    }
    return best;
}

/// The two frame sizes the Request speaks (the resource takes 4..16).
enum class SpiDataSize : uint8_t { bits8 = 0, bits16 = 1 };
constexpr bool spi_frame_is_halfword(SpiDataSize s) { return s == SpiDataSize::bits16; }
constexpr uint8_t spi_data_bits(SpiDataSize s) { return s == SpiDataSize::bits16 ? 16u : 8u; }
constexpr uint16_t spi_frame_mask(SpiDataSize s) { return s == SpiDataSize::bits16 ? 0xFFFFu : 0x00FFu; }

enum class SpiRole : uint8_t { client = 0, host = 1 };

// ---- the register vocabulary --------------------------------------------------------

/// SSPCR0: the frame, the framing, the phase pair and the serial clock
/// rate.
struct SpiControl0 {
    static constexpr uint32_t data_size_lsb = 0;      ///< DSS: the frame's bits, less one
    static constexpr uint32_t data_size_bits = 0xFu << data_size_lsb;
    static constexpr uint32_t format_lsb = 4;         ///< FRF
    static constexpr uint32_t cpol = 1u << 6;         ///< SPO
    static constexpr uint32_t cpha = 1u << 7;         ///< SPH
    static constexpr uint32_t rate_lsb = 8;           ///< SCR
    static constexpr uint32_t rate_bits = 0xFFu << rate_lsb;
};

/// SSPCR1: the enable, the role, and the two bits that are LIVE under it.
struct SpiControl1 {
    static constexpr uint32_t loopback = 1u << 0;        ///< LBM
    static constexpr uint32_t enable = 1u << 1;          ///< SSE
    static constexpr uint32_t client = 1u << 2;          ///< MS
    static constexpr uint32_t output_disable = 1u << 3;  ///< SOD
};

/// SSPCPSR: the even prescaler, and SSPDR's frame field.
struct SpiDataField {
    static constexpr uint32_t prescaler = 0xFFu;
    static constexpr uint32_t frame = 0xFFFFu;
};

/// SSPSR's flags.
struct SpiFlag {
    static constexpr uint32_t tx_empty = 1u << 0;
    static constexpr uint32_t tx_not_full = 1u << 1;
    static constexpr uint32_t rx_not_empty = 1u << 2;
    static constexpr uint32_t rx_full = 1u << 3;
    static constexpr uint32_t busy = 1u << 4;
};

/// The four interrupt sources: one layout for IMSC, RIS, MIS and ICR
/// (only the timeout and the overrun clear through ICR; the two FIFO
/// levels clear by moving data).
struct SpiInterrupt {
    static constexpr uint32_t overrun = 1u << 0;     ///< RORIM: the receive FIFO overflowed
    static constexpr uint32_t rx_timeout = 1u << 1;  ///< RTIM: data waiting, 32 bit periods idle
    static constexpr uint32_t rx = 1u << 2;          ///< RXIM: the receive FIFO at or above half
    static constexpr uint32_t tx = 1u << 3;          ///< TXIM: the transmit FIFO at or below half
    static constexpr uint32_t all = tx | rx | rx_timeout | overrun;
};

/// SSPDMACR: the two request enables.
struct SpiDmaControl {
    static constexpr uint32_t rx = 1u << 0;
    static constexpr uint32_t tx = 1u << 1;
};

/// The whole configuration of the block.
struct SpiConfig {
    SpiRole role = SpiRole::host;
    SpiMode mode = SpiMode::mode0;
    SpiFormat format = SpiFormat::motorola;
    uint8_t bits = 8;                 ///< DSS + 1: 4..16
    SpiClock clock = SpiClocks::div16;
    bool loopback = false;            ///< LBM: the transmitter into the receiver
    bool output_disabled = false;     ///< SOD: a client that answers nothing on the wire
};

constexpr bool spi_config_valid(const SpiConfig& c) {
    return c.bits >= 4u && c.bits <= 16u && c.clock.valid() &&
           static_cast<uint8_t>(c.format) <= 2u &&
           !(c.role == SpiRole::host && c.output_disabled);
}

/// SSPCR0 for a configuration.
constexpr uint32_t spi_cr0_of(const SpiConfig& c) {
    uint32_t v = static_cast<uint32_t>(c.clock.scr) << SpiControl0::rate_lsb;
    v |= static_cast<uint32_t>(c.format) << SpiControl0::format_lsb;
    v |= static_cast<uint32_t>(c.bits - 1u) << SpiControl0::data_size_lsb;
    if (c.format == SpiFormat::motorola) {
        if (spi_mode_cpol(c.mode)) { v |= SpiControl0::cpol; }
        if (spi_mode_cpha(c.mode)) { v |= SpiControl0::cpha; }
    }
    return v;
}

/// SSPCR1 for a configuration, SSE clear.
constexpr uint32_t spi_cr1_of(const SpiConfig& c) {
    uint32_t v = 0;
    if (c.role == SpiRole::client) { v |= SpiControl1::client; }
    if (c.loopback) { v |= SpiControl1::loopback; }
    if (c.output_disabled) { v |= SpiControl1::output_disable; }
    return v;
}

// =============================================================================
// The resource
// =============================================================================

template <Pl022Chip Chip, uint8_t n>
struct Pl022Ssp {
    static_assert(n < Chip::instances, "this family has no PL022 with that number");
    Pl022Ssp() = delete;

    using Regs = typename Chip::Regs;

    static constexpr uint8_t index = n;
    static constexpr uint8_t fifo_depth = Chip::fifo_depth;
    static constexpr typename Chip::DmaRequest dreq_tx = Chip::template tx_request<n>();
    static constexpr typename Chip::DmaRequest dreq_rx = Chip::template rx_request<n>();

    static Regs& regs() { return Chip::template regs<n>(); }
    static constexpr typename Chip::Irq irq() { return Chip::template irq<n>(); }
    static volatile void* data_address() { return &regs().SSPDR; }

    /// The block from its reset state: held, released, ready.
    static bool reset() { return Chip::template reset<n>(); }
    static bool released() { return Chip::template released<n>(); }
    static void hold() { Chip::template hold<n>(); }

    static bool enabled() { return (regs().SSPCR1 & SpiControl1::enable) != 0u; }
    static void enable(bool on) {
        if (on) { Chip::set_bits(regs().SSPCR1, SpiControl1::enable); }
        else { Chip::clear_bits(regs().SSPCR1, SpiControl1::enable); }
    }

    /// The whole configuration, refused while enabled (the block's rule:
    /// control registers are programmed with SSE clear) and for what
    /// spi_config_valid refuses. The interrupt mask is kept.
    static bool configure(const SpiConfig& c) {
        if (enabled() || !spi_config_valid(c)) {
            return false;
        }
        regs().SSPCPSR = c.clock.cpsdvsr;
        regs().SSPCR0 = spi_cr0_of(c);
        regs().SSPCR1 = spi_cr1_of(c);
        return true;
    }

    /// LBM and SOD are the two live bits of SSPCR1: allowed under SSE.
    static void loopback(bool on) {
        if (on) { Chip::set_bits(regs().SSPCR1, SpiControl1::loopback); }
        else { Chip::clear_bits(regs().SSPCR1, SpiControl1::loopback); }
    }
    static bool loopback() { return (regs().SSPCR1 & SpiControl1::loopback) != 0u; }
    static void output_disabled(bool on) {
        if (on) { Chip::set_bits(regs().SSPCR1, SpiControl1::output_disable); }
        else { Chip::clear_bits(regs().SSPCR1, SpiControl1::output_disable); }
    }

    /// The rate as the registers hold it.
    static SpiClock clock() {
        return SpiClock{static_cast<uint8_t>(regs().SSPCPSR & SpiDataField::prescaler),
                        static_cast<uint8_t>((regs().SSPCR0 & SpiControl0::rate_bits) >> SpiControl0::rate_lsb)};
    }
    static uint8_t bits() { return static_cast<uint8_t>((regs().SSPCR0 & SpiControl0::data_size_bits) + 1u); }

    // ---- data and status ------------------------------------------------------
    static uint32_t flags() { return regs().SSPSR; }
    static bool tx_not_full() { return (flags() & SpiFlag::tx_not_full) != 0u; }
    static bool tx_empty() { return (flags() & SpiFlag::tx_empty) != 0u; }
    static bool rx_not_empty() { return (flags() & SpiFlag::rx_not_empty) != 0u; }
    static bool rx_full() { return (flags() & SpiFlag::rx_full) != 0u; }
    static bool busy() { return (flags() & SpiFlag::busy) != 0u; }

    static void write_data(uint16_t v) { regs().SSPDR = v; }
    static uint16_t read_data() { return static_cast<uint16_t>(regs().SSPDR & SpiDataField::frame); }
    /// Throw away whatever the receive FIFO holds.
    static void flush_rx() {
        while (rx_not_empty()) {
            (void)regs().SSPDR;
        }
    }

    // ---- interrupts -----------------------------------------------------------
    static void interrupts(uint32_t mask, bool on) {
        if (on) { Chip::set_bits(regs().SSPIMSC, mask); } else { Chip::clear_bits(regs().SSPIMSC, mask); }
    }
    static uint32_t interrupts() { return regs().SSPIMSC; }
    static uint32_t raw_pending() { return regs().SSPRIS; }
    static uint32_t pending() { return regs().SSPMIS; }
    /// The two clearable sources (the timeout, the overrun).
    static void clear_pending(uint32_t mask) { regs().SSPICR = mask & (SpiInterrupt::rx_timeout | SpiInterrupt::overrun); }

    /// The raised-and-enabled sources, the answer every brio resource's
    /// ISR body gives; the two clearable ones are cleared here.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t up = pending();
        if ((up & (SpiInterrupt::rx_timeout | SpiInterrupt::overrun)) != 0u) {
            clear_pending(up);
        }
        return up;
    }

    /// The DMA request enables (SSPDMACR).
    static void dma_requests(bool tx, bool rx) {
        regs().SSPDMACR = (tx ? SpiDmaControl::tx : 0u) | (rx ? SpiDmaControl::rx : 0u);
    }
};

// =============================================================================
// The host engine
// =============================================================================

/// A DMA block the engines could not finish, or a polled block that
/// never completed: the engine's own status code, in the range
/// util/bus_master.hpp leaves to engines.
inline constexpr uint8_t spi_dma_fault = bus_engine_status;

/// SSPCPSR over SSPCR0 as ONE word - the prescaler in bits 23:16, the
/// control register's own sixteen below - for a Motorola host at one
/// prescaler pair, mode and width: the word `apply()` compares, computed
/// from a request's three fields in a handful of shifts. The polarity
/// is the mode's bit 1 into SPO (bit 6), the phase its bit 0 into SPH
/// (bit 7), DSS is 7 for a byte and 15 for a half-word; FRF stays zero.
[[gnu::always_inline]] inline constexpr uint32_t spi_control_word(SpiClock c, SpiMode m, SpiDataSize bits) {
    return (static_cast<uint32_t>(c.cpsdvsr) << 16) |
           (static_cast<uint32_t>(c.scr) << SpiControl0::rate_lsb) |
           ((static_cast<uint32_t>(m) & 2u) << 5) | ((static_cast<uint32_t>(m) & 1u) << 7) |
           (7u | (static_cast<uint32_t>(bits) << 3));
}
// The folded word is what configure() writes, held against the long way.
static_assert(spi_control_word(SpiClocks::div16, SpiMode::mode3, SpiDataSize::bits16) ==
              ((2u << 16) | spi_cr0_of({.mode = SpiMode::mode3, .bits = 16, .clock = SpiClocks::div16})));
static_assert(spi_control_word(SpiClock{254, 255}, SpiMode::mode2, SpiDataSize::bits8) ==
              ((254u << 16) | spi_cr0_of({.mode = SpiMode::mode2, .bits = 8, .clock = SpiClock{254, 255}})));
static_assert(spi_control_word(SpiClocks::div2, SpiMode::mode1, SpiDataSize::bits8) ==
              ((2u << 16) | spi_cr0_of({.mode = SpiMode::mode1, .bits = 8, .clock = SpiClocks::div2})));

/*
 * THE SHORT ASYNCHRONOUS REQUEST, and the arithmetic behind it.
 *
 * On the engines a transaction costs the core a fixed sum, E - start(),
 * the two channels' launch, the one interrupt, the completion - whatever
 * its length: the wire's time is the core's own. Polled, it costs the core
 * the whole transaction: a fixed part F and each frame, which is the
 * wire's frame or the loop's own turn, the longer. A frame on the wire is
 * `bits` SCK periods and, with SPH = 0 (modes 0 and 2), the 1.5 periods of
 * the frame gap the block keeps between the frames of a continuous
 * transfer (12.3.4.10 and 12.3.4.12 of the RP2350 data sheet; the polled
 * loop and the engines pay it alike, measured). An SCK period is the
 * divisor's SSPCLK cycles, and SSPCLK need not be the core's clock, so the
 * frame is scaled by the ratio of the two (pl022_frame_cycles()). So:
 *
 *     a frame read (an in buffer)   max(the wire's frame, the receive loop's turn)
 *     a frame written (none)        max(the wire's frame, the transmit-only loop's turn)
 *
 * and a request is cheaper polled while F and its frames stay under E: it
 * rides the engines from the smallest count N with F + N x frame >= E,
 * ceil((E - F) / frame) (pl022_async_min_frames()). Only the difference E -
 * F enters, one for a read and one for a write, and a family measures it as
 * one difference through one call (Pl022Costs, the family's document). The
 * request's frames are counted whole: its command phase as frames written,
 * its data phase as frames read or written.
 *
 * The same figures say when a POLLED read is better off on the engines.
 * The FIFOs keep eight frames in flight on the polled loop, so it has no
 * dead bus below its own turn; where that turn is longer than the wire's
 * frame - the fastest rates - each frame read costs (turn - frame) more
 * than the engines' wire would, and a polled read rides the engines inside
 * start() from ceil((E - F) / (turn - frame)) frames
 * (pl022_polled_dma_min_frames()); where the wire is the longer, never.
 *
 * The host keeps the two frame costs and that count for the configuration
 * in force, computed where the control word is written (reconfigure(),
 * init(), rebase()) and never per request: an asynchronous request pays two
 * multiplies and a compare, a polled one a compare.
 */

/// SSPCLK's cycle in the core's, in 256ths: what turns a frame on the wire
/// into the core cycles it costs. 256 when either rate is unknown.
constexpr uint32_t pl022_core_per_sspclk_q8(uint32_t core_hz, uint32_t sspclk_hz) {
    if (core_hz == 0u || sspclk_hz == 0u) {
        return 256u;
    }
    const uint64_t q = (static_cast<uint64_t>(core_hz) << 8) / sspclk_hz;
    return q > 0xFFFF'FFFFull ? 0xFFFF'FFFFu : static_cast<uint32_t>(q);
}

/// A frame on the wire in core cycles at a rate, mode and width: `bits` SCK
/// periods and the 1.5 of the frame gap with SPH = 0, each period the
/// divisor's SSPCLK cycles, scaled by `core_per_sspclk_q8`. Saturated at
/// 0xFFFF: a frame that long rides the engines from one frame whatever the
/// figures.
constexpr uint32_t pl022_frame_cycles(uint32_t core_per_sspclk_q8, SpiClock c, SpiMode m, SpiDataSize bits) {
    const uint64_t halves = 2u * static_cast<uint64_t>(spi_data_bits(bits)) + (spi_mode_cpha(m) ? 0u : 3u);
    const uint64_t v = (static_cast<uint64_t>(c.divisor()) * halves * core_per_sspclk_q8) >> 9;
    return v > 0xFFFFu ? 0xFFFFu : static_cast<uint32_t>(v);
}

/// What a polled frame costs the core: the wire's frame or the loop's turn,
/// the longer - the receive loop's for a frame read, the transmit-only
/// loop's for one written.
constexpr uint32_t pl022_polled_frame_cost(const Pl022Costs& k, uint32_t frame, bool read) {
    const uint32_t turn = read ? k.read_turn : k.write_turn;
    return frame > turn ? frame : turn;
}

/// E - F, a read's (`read`) or a write's.
constexpr uint32_t pl022_async_budget(const Pl022Costs& k, bool read) {
    return read ? k.engines_over_read : k.engines_over_written;
}

/// The frames from which an asynchronous request of one phase rides the
/// engines, a frame on the wire being `frame` core cycles (`read`: with an
/// in buffer); a shorter one runs polled and completes inside start().
constexpr uint16_t pl022_async_min_frames(const Pl022Costs& k, uint32_t frame, bool read) {
    const uint32_t f = pl022_polled_frame_cost(k, frame, read);
    const uint32_t n = f == 0u ? 1u : (pl022_async_budget(k, read) + f - 1u) / f;
    return static_cast<uint16_t>(n > 1u ? (n < 0xFFFFu ? n : 0xFFFFu) : 1u);
}

/// The frames from which a POLLED read rides the engines inside start(): the
/// receive loop's turn beyond the wire's frame, a frame, against E - F;
/// 0xFFFF (never) where the wire is the longer.
constexpr uint16_t pl022_polled_dma_min_frames(const Pl022Costs& k, uint32_t frame) {
    if (k.read_turn <= frame) {
        return 0xFFFFu;
    }
    const uint32_t dead = k.read_turn - frame;
    const uint32_t n = (pl022_async_budget(k, true) + dead - 1u) / dead;
    return static_cast<uint16_t>(n < 0xFFFFu ? (n > 1u ? n : 1u) : 0xFFFFu);
}

// The arithmetic on a core that runs at SSPCLK: a byte frame at /4 in mode 3
// is 32 cycles, in mode 0 38 (the gap); a 16-bit frame at /2 in mode 0 35.
static_assert(pl022_frame_cycles(256u, SpiClocks::div4, SpiMode::mode3, SpiDataSize::bits8) == 32u);
static_assert(pl022_frame_cycles(256u, SpiClocks::div4, SpiMode::mode0, SpiDataSize::bits8) == 38u);
static_assert(pl022_frame_cycles(256u, SpiClocks::div2, SpiMode::mode0, SpiDataSize::bits16) == 35u);
static_assert(pl022_frame_cycles(pl022_core_per_sspclk_q8(150'000'000u, 12'000'000u), SpiClocks::div2, SpiMode::mode3,
                                 SpiDataSize::bits8) == 200u);
static_assert(pl022_async_min_frames({400u, 400u, 30u, 16u}, 38u, true) == 11u);
static_assert(pl022_async_min_frames({400u, 400u, 30u, 16u}, 16u, true) == 14u);
static_assert(pl022_polled_dma_min_frames({400u, 400u, 30u, 16u}, 16u) == 29u);
static_assert(pl022_polled_dma_min_frames({400u, 400u, 30u, 16u}, 38u) == 0xFFFFu);

/**
 * Pl022Host<Chip, n, pins, TxEngine, RxEngine, Placement>
 *
 * The engine SpiBus (util/spi_bus.hpp = BusMaster) drives; the family's
 * own header aliases it to `SpiHost<n, pins, ...>` and carries the
 * example in that family's spelling. Its Request is the descriptor every
 * stratum's SPI host takes (docs/design/spi-bus.md; the file header):
 * this host asserts the request's own chip select around the
 * transaction (low at start(), high when it completes), a command phase
 * goes out with D/C low and a data phase with it high, the data phase
 * has an optional out buffer (null = 0xFF dummies) and an optional in
 * buffer (null = the frames read back are discarded), and every buffer
 * is lent until the reply lands. The mode, rate and frame size travel per
 * request; the bit order is the PL022's (most significant first, no
 * other) and no verb offers another.
 *
 * The two ways (docs/design/spi-bus.md, "Two ways to run a request"):
 *
 *  - POLLED (`polled` true): the loops of the file header inside start(),
 *    and on a host with engines a read long enough to repay them on the
 *    engines inside start() too, start() waiting on their completion
 *    (polled_dma_min_frames()).
 *  - ASYNCHRONOUS (`polled` false) IS DMA. With no engines bound the
 *    request is refused: start() answers true at once with spi_rejected in
 *    status(), before it touches the block, the select or the D/C, so
 *    nothing reaches the wire; `has_dma` says so at compile time. With
 *    engines bound, a request whose polled transfer would cost less than
 *    the engines' fixed cost runs polled and completes inside start()
 *    (async_min_frames(), the arithmetic above), and so does one the
 *    engines cannot carry (16-bit frames on byte engines or off a
 *    half-word). A longer one goes to the engines: a command phase on
 *    them too when it would repay a block of its own (or is the whole
 *    request), polled inside start() otherwise, and the data phase on
 *    them, start() returning false and dma_isr() ending the transaction -
 *    one interrupt a block, a command block's completion launching the
 *    data block from the vector.
 */
template <Pl022Chip Chip, uint8_t n, typename Chip::Pins pins, typename TxEngine, typename RxEngine,
          typename Placement = InFlash>
class Pl022Host {
    using S = Pl022Ssp<Chip, n>;
    using Regs = typename Chip::Regs;
    static_assert(Chip::pins_valid(n, pins),
                  "brio SpiHost: these pads cannot carry this instance's signals - which pad "
                  "carries which signal of which SPI is fixed per instance, each under the one "
                  "function that routes the block to it, so see the family's own pin table");
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0, "the engine slots must name a complete type");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio SpiHost: name both DMA engines or neither - the data phase is full-duplex and "
                  "its completion is the RECEIVE block's");
    static_assert(Chip::template engines_distinct<TxEngine, RxEngine>(),
                  "brio SpiHost: the two engines must ride two different DMA channels");
    static_assert(HotPlacement<Placement, Chip>,
                  "brio SpiHost: the placement is InFlash, or InRam on a family whose chip traits say "
                  "places_fast_text (util/placement.hpp)");
    static_assert(Pl022ChipDelay<Chip>,
                  "the family's chip traits must hand over a DelayRate its own delay_us takes: "
                  "a Request's cs_setup_us is spent on it (Pl022ChipDelay)");
    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, typename RxEngine::element> &&
                   (std::is_same_v<typename TxEngine::element, uint8_t> ||
                    std::is_same_v<typename TxEngine::element, uint16_t>);
        } else {
            return true;
        }
    }(), "brio SpiHost: both DMA engines carry the same element, uint8_t or uint16_t - the widest "
         "beat SSPDR takes is sixteen bits; uint16_t engines carry 16-bit frames too, uint8_t "
         "ones leave them to the polled loop");

    static constexpr uint8_t sck_pin = Chip::sck_pad(pins);
    static constexpr uint8_t tx_pin = Chip::tx_pad(pins);
    static constexpr uint8_t rx_pin = Chip::rx_pad(pins);
    static constexpr uint8_t cs_pin = Chip::cs_pad(pins);
    static_assert(tx_pin != Chip::no_pad, "brio SpiHost: a host needs SCK and TX; RX is optional for a write-only bus");

    using SckPad = typename Chip::template Pad<sck_pin>;
    using TxPad = typename Chip::template Pad<tx_pin>;
    using RxPad = typename Chip::template Pad<rx_pin != Chip::no_pad ? rx_pin : sck_pin>;
    using CsPad = typename Chip::template Pad<cs_pin != Chip::no_pad ? cs_pin : sck_pin>;

public:
    Pl022Host() = delete;
    using Resource = S;
    static constexpr typename Chip::Pins pin_set = pins;
    static constexpr bool has_engines = TxEngine::present;
    /// Can this host run an asynchronous request? The contract's name for
    /// has_engines (docs/design/spi-bus.md): false refuses every request
    /// not marked polled, so a program that posts one asserts it -
    /// `static_assert(Bus::has_dma)`.
    static constexpr bool has_dma = has_engines;
    /// Engines whose element is a half-word: 16-bit frames ride them.
    static constexpr bool wide_engines = [] {
        if constexpr (TxEngine::present) {
            return sizeof(typename TxEngine::element) == 2u;
        } else {
            return false;
        }
    }();
    /// The family's figures the short asynchronous request is decided from
    /// (Pl022Costs, the arithmetic above).
    static constexpr Pl022Costs costs = Chip::costs;
    /// E - F for a read and for a write.
    static constexpr uint32_t async_read_budget_cycles = pl022_async_budget(costs, true);
    static constexpr uint32_t async_write_budget_cycles = pl022_async_budget(costs, false);
    /// The pin set names `cs`: the block's own frame signal SSPFSSOUT
    /// drives it (the file header's THE SELECTS) - one select a frame in
    /// modes 0 and 2, one a run of back-to-back frames in modes 1 and 3 -
    /// and a request's own `cs` is left null.
    static constexpr bool frame_select = cs_pin != Chip::no_pad;

    /// The loop-back (LBM) as part of the applied configuration, so a
    /// request that re-applies mode, rate or width keeps it: the
    /// wireless instrument. The bus must be idle.
    static void loopback(bool on) {
        if (applied_.loopback == on) {
            return;
        }
        applied_.loopback = on;
        S::enable(false);
        (void)S::configure(applied_);
        S::enable(true);
        S::flush_rx();
    }
    static bool loopback() { return applied_.loopback; }

    /// The transaction descriptor - the fields docs/design/spi-bus.md
    /// ("Realizations") gives every stratum's SPI Request, under those
    /// names, laid out words first so that nothing is padded but the
    /// tail (the file header).
    struct Request {
        /// Phase 1, sent with DC low; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        /// Phase 2 out, null = 0xFF dummies; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; LENT until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        ReplyTo<SpiDone> reply;
        uint16_t len;      ///< phase 2 length, in FRAMES
        /// Per-transaction bus configuration: a shared bus's devices
        /// each name their own, and the engine reprograms the peripheral
        /// only when something CHANGED.
        SpiClock clock = SpiClocks::div16;
        typename Chip::PinRef cs;   ///< asserted low around the transaction
        typename Chip::PinRef dc;   ///< display D/C line; null = no such pin
        /// Microseconds between the CS assertion and the first clock -
        /// what a device's datasheet calls CS setup. Spent spinning in
        /// start(), main context; 0 = none.
        uint8_t cs_setup_us = 0;
        uint8_t cmd_len;   ///< in FRAMES
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;
        /// The way to run it: true = polled inside start(); false =
        /// asynchronous, on the DMA engines - polled inside start() when
        /// too short to repay them (async_min_frames()), refused
        /// (spi_rejected) on a host with none (has_dma).
        bool polled = false;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle ----------------------------------------------------------

    /// Bring the instance up as a host: the block out of reset, the boot
    /// configuration, the engines' binding, the pads. `clock` is the app's
    /// Clock tag - the divisor is of whichever rate the family's traits
    /// call SSPCLK. `max_sck_hz` is an optional ceiling for the whole bus;
    /// 0 = none. False when the ceiling cannot be produced or the block
    /// did not come up. Main context, after the clock and before
    /// interrupts are enabled globally. The instance's interrupt line is
    /// left disabled and its four sources masked: the host arms none of
    /// them (the DMA line's body is its one interrupt body).
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        static_assert(Pl022ChipClock<Chip, Clock>,
                      "the family's chip traits must say which rate of this Clock type is "
                      "SSPCLK (Pl022ChipClock's sspclk_hz)");
        static_assert(clock_follows<Clock, Pl022Host>(),
                      "this SpiHost is initialized with a DynamicClock that does not list it "
                      "among its Users: its SCK ceiling and its cs_setup timing would go stale");
        Chip::Interrupts::disable(S::irq());
        ceiling_hz_ = max_sck_hz;
        rebase(Chip::sspclk_hz(clock), clock_hz(clock));
        if (max_sck_hz != 0u && !ceiling_) {
            return false;
        }
        if (!S::reset()) {
            return false;
        }
        applied_ = boot_config();
        applied_word_ = spi_control_word(applied_.clock, applied_.mode, SpiDataSize::bits8);
        costs_for(applied_.clock, applied_.mode, SpiDataSize::bits8);
        if (!S::configure(applied_)) {
            return false;
        }
        status_ = spi_ok;
        in_cmd_ = false;
        S::interrupts(SpiInterrupt::all, false);
        if constexpr (has_engines) {
            // The receive block's completion is the transaction's, and the
            // proof of the transmit block's: that one reports errors alone.
            // The transmit engine at the family's NORMAL level: a starved
            // transmit holds SCK between two frames and loses nothing.
            TxEngine::arm(S::data_address(), S::dreq_tx, false, TxEngine::Report::errors);
            arm_rx_engine();
        }
        // The pads go to the peripheral with the registers already
        // holding the idle polarity, then SSE: a pad handed over first
        // would show whatever an unconfigured block drives.
        SckPad::function(Chip::pad_function, Chip::sck_pad_config());
        TxPad::function(Chip::pad_function, Chip::tx_pad_config());
        if constexpr (rx_pin != Chip::no_pad) {
            RxPad::function(Chip::pad_function, Chip::host_rx_pad_config());
        }
        if constexpr (frame_select) {
            // The frame signal idles high with the block configured, so
            // the pad goes over deasserted.
            CsPad::function(Chip::pad_function, Chip::cs_pad_config());
        }
        S::enable(true);
        S::flush_rx();
        return true;
    }

    /// The reference rate and the core rate changed. A Request's `clock`
    /// is a DIVISION of SSPCLK and scales by itself; what is recomputed
    /// is the ceiling, the cs_setup timing and what a frame costs the
    /// core. The bus must be idle.
    static void rebase(uint32_t pclk_hz, uint32_t sys_hz) {
        pclk_hz_ = pclk_hz;
        ceiling_ = ceiling_hz_ != 0u ? spi_clock_for(pclk_hz, ceiling_hz_) : std::optional<SpiClock>{};
        cs_rate_ = Chip::delay_rate(sys_hz);
        core_per_sspclk_q8_ = pl022_core_per_sspclk_q8(sys_hz, pclk_hz);
        costs_for(applied_.clock, applied_.mode, spi_frame_size(applied_.bits));
    }
    /// The setting that produces at most `hz` of SCK at the SSPCLK last
    /// seen - the chooser a device's datasheet limit is spoken to.
    static std::optional<SpiClock> clock_for(uint32_t hz) { return spi_clock_for(pclk_hz_, hz); }
    /// What a request at this setting really runs at, ceiling included.
    static uint32_t sck_hz(SpiClock c) { return spi_sck_hz(pclk_hz_, clamp(c)); }
    static uint32_t max_sck_hz() { return ceiling_hz_; }
    static std::optional<SpiClock> ceiling_clock() { return ceiling_; }
    static uint32_t reference_hz() { return pclk_hz_; }

    /// The frames from which an asynchronous request of one phase rides the
    /// engines at this rate (ceiling included), mode and width, at the
    /// clocks last seen (`read`: with an in buffer); a shorter one runs
    /// polled and completes inside start(). The arithmetic above.
    static uint16_t async_min_frames(SpiClock c, SpiMode m, SpiDataSize bits, bool read) {
        return pl022_async_min_frames(costs, pl022_frame_cycles(core_per_sspclk_q8_, clamp(c), m, bits), read);
    }
    /// The frames from which a POLLED read rides the engines inside start()
    /// at this rate, mode and width; 0xFFFF where it never does.
    static uint16_t polled_dma_min_frames(SpiClock c, SpiMode m, SpiDataSize bits) {
        return pl022_polled_dma_min_frames(costs, pl022_frame_cycles(core_per_sspclk_q8_, clamp(c), m, bits));
    }

    /// Put the peripheral at this mode, rate and frame size NOW, moving
    /// no data - for callers that frame the select window themselves
    /// (Request.cs null). A mode change is a CPOL FLIP ON THE WIRE and a
    /// selected client counts it as an edge: prime FIRST, then select.
    static void prime(SpiMode m, SpiClock c, SpiDataSize bits = SpiDataSize::bits8) {
        reconfigure(clamp(c), m, bits);
    }

    // ---- the transfer -------------------------------------------------------

    /// Begin a transaction (SpiBus calls it from main context). True when
    /// it completed inside the call - a polled request, an asynchronous one
    /// too short for the engines or one they cannot carry, the empty one,
    /// and on a host with no engines a refused one (spi_rejected, the wire
    /// untouched); false when it went to the engines and dma_isr() will end
    /// it, a TransferDone following - exactly util/bus_master.hpp's engine
    /// contract. The request is LENT for the call: what completes inside
    /// it is read through the reference and nothing is kept, an engined
    /// one copies what its tenure needs (the file header). A polled
    /// request waits here for its whole transaction, bounded by one
    /// budget; a budget that runs out ends it with the engine's own status
    /// and the block left for recover().
    static bool start(const Request& r) {
        return Hot<Placement>::run([&r] { return start_body(r); });
    }

    /// The DMA line's interrupt body - call it from the line the engines
    /// report on, and from nowhere else. The receive channel's completion
    /// ends a block: a command block's end raises D/C and puts the data
    /// block on the engines, a data block's end (or a command block with no
    /// data phase after it) ends the transaction, the select released. A
    /// bus error on either channel ends it with spi_dma_fault; the transmit
    /// channel reports a bus error and nothing else (init()), so a block
    /// of its own on the line IS an error. Compiles away on an engineless
    /// host. True when the transaction just completed: the edge the
    /// program's glue posts TransferDone{status()} on. Runs to completion
    /// and masks nothing.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            return Hot<Placement>::run([] { return dma_isr_body(); });
        } else {
            return false;
        }
    }

    /// The engine's completion status, for the TransferDone payload: read
    /// in the vector right after dma_isr() answered true, or after a
    /// start() that answered true.
    static uint8_t status() { return status_; }

    /// Put the engine back where start() is legal (the verb a timed
    /// SpiBus calls on a transaction that never answered): the select
    /// released first, the engines put away and re-claimed, the block
    /// reset and reconfigured to the applied state.
    static bool recover() {
        tenure_.cs.set();
        if constexpr (has_engines) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();
            arm_rx_engine();
        }
        in_cmd_ = false;
        dma_active_ = false;
        dma_done_ = false;
        S::enable(false);
        const bool ok = S::reset();
        const bool cfg = S::configure(applied_);
        S::interrupts(SpiInterrupt::all, false);
        S::enable(true);
        S::flush_rx();
        return ok && cfg;
    }

    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Chip::Interrupts::disable(S::irq());
        S::interrupts(SpiInterrupt::all, false);
        S::enable(false);
        SckPad::release();
        TxPad::release();
        if constexpr (rx_pin != Chip::no_pad) {
            RxPad::release();
        }
        if constexpr (frame_select) {
            CsPad::release();
        }
        S::hold();
    }

private:
    /// start()'s body, written once (util/placement.hpp: inline where
    /// the placement puts it). The two phases run through ONE call of the
    /// polled loop, the command phase's turn and then the data phase's, so
    /// the flattened body carries one copy of each loop shape and not two;
    /// a data phase on the engines leaves that loop for them.
    [[gnu::always_inline]] static bool start_body(const Request& r) {
        if constexpr (!has_dma) {
            if (!r.polled) {
                // Asynchronous is DMA, and this host has no engines:
                // refused before the block, the select or the D/C is
                // touched, so nothing reaches the wire.
                status_ = spi_rejected;
                return true;
            }
        }
        status_ = spi_ok;
        const uint16_t total = static_cast<uint16_t>(static_cast<uint16_t>(r.cmd_len) + r.len);
        if (total == 0u) {
            return true;
        }
        apply(r);
        const bool has_cmd = r.cmd_len != 0u;
        if (has_cmd) {
            r.dc.clear();
        } else {
            r.dc.set();
        }
        r.cs.clear();   // assert, active low
        if (r.cs_setup_us != 0u) {
            (void)delay_us(cs_rate_, r.cs_setup_us);
        }
        // Does the data phase ride the engines? Asynchronous: when the
        // request repays them and they can carry it. Polled: a read long
        // enough to repay them at this rate (the arithmetic above). No
        // flush of the receive FIFO here: every path leaves it empty, and
        // the fault paths drain it.
        bool engines = false;
        if constexpr (has_dma) {
            engines = r.polled ? polled_rides_engines(r) : repays_engines(r);
            if (engines && !r.polled && has_cmd && command_block(r)) {
                // The command phase as a block of its own; its completion
                // in dma_isr() raises D/C and launches the data block.
                take_tenure(r);
                in_cmd_ = r.len != 0u;
                launch_dma(r.cmd.get(), nullptr, r.cmd_len);
                return false;
            }
        }
        // THE POLLED LOOP: the whole request, or the command phase before a
        // data phase on the engines.
        uint32_t budget = spin_budget(engines ? static_cast<uint16_t>(r.cmd_len) : total);
        bool data = !has_cmd;
        const uint8_t* out = data ? r.tx.get() : r.cmd.get();
        uint8_t* in = data ? r.rx.get() : nullptr;
        uint16_t count = data ? r.len : static_cast<uint16_t>(r.cmd_len);
        bool ok = true;
        for (;;) {
            if constexpr (has_dma) {
                if (data && engines) {
                    take_tenure(r);
                    in_cmd_ = false;
                    launch_dma(r.tx.get(), r.rx.get(), r.len);
                    if (!r.polled) {
                        return false;   // dma_isr() is the completion edge
                    }
                    spin_dma();
                    r.cs.set();
                    return true;
                }
            }
            ok = run_polled(out, in, count, r.bits, budget);
            if (!ok || data) {
                break;
            }
            r.dc.set();
            if (r.len == 0u) {
                break;
            }
            data = true;
            out = r.tx.get();
            in = r.rx.get();
            count = r.len;
        }
        if (!ok) {
            settle_fault();
        }
        r.cs.set();
        return true;
    }

    /// dma_isr()'s body.
    [[gnu::always_inline]] static bool dma_isr_body() {
        if constexpr (has_engines) {
            if ((TxEngine::service() & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                return finish_dma(spi_dma_fault);
            }
            const uint8_t rx = RxEngine::service();
            if (rx != 0u && dma_active_) {
                if ((rx & RxEngine::flag_error) != 0u) {
                    return finish_dma(spi_dma_fault);
                }
                if ((rx & RxEngine::flag_complete) != 0u) {
                    if (in_cmd_) {
                        return next_block();
                    }
                    return finish_dma(spi_ok);
                }
            }
        }
        return false;
    }

    /// What an engined transaction keeps of a request after start()
    /// returned: the lent request copied AS WORDS (one load and one store a
    /// word, inline on every core - a struct assignment of this size is a
    /// memcpy call on a RISC-V build at -Os). The reply capsule is the
    /// arbiter's and is copied with the rest, never read.
    static_assert(sizeof(Request) % 4u == 0u, "the Request is copied as words");
    static constexpr size_t request_words = sizeof(Request) / 4u;

    /// The request's words, one load and one store each: the alignment is
    /// the Request's own (it holds pointers), stated to the compiler so
    /// that a strict-alignment core moves words and not bytes, and the
    /// four-byte copies are what every compiler expands in place.
    template <size_t... I>
    [[gnu::always_inline]] static inline void copy_words(Request& to, const Request& from,
                                                         std::index_sequence<I...>) {
        uint8_t* d = static_cast<uint8_t*>(__builtin_assume_aligned(&to, 4));
        const uint8_t* s = static_cast<const uint8_t*>(__builtin_assume_aligned(&from, 4));
        (__builtin_memcpy(d + 4u * I, s + 4u * I, 4u), ...);
    }

    static void take_tenure(const Request& r) {
        copy_words(tenure_, r, std::make_index_sequence<request_words>{});
    }

    /// Can the engines carry a run of these frames between these two
    /// buffers? 8-bit frames always; 16-bit frames on half-word engines,
    /// both buffers aligned to a half-word (a null one is) - a misaligned
    /// run is one the DMA would move wrongly and refuses, so such a request
    /// runs polled whichever way it asked (the family exception
    /// docs/design/spi-bus.md allows: a refusal would make the answer
    /// depend on where a buffer happens to sit).
    [[gnu::always_inline]] static bool carriable(SpiDataSize bits, const void* a, const void* b) {
        if (!spi_frame_is_halfword(bits)) {
            return true;
        }
        if constexpr (wide_engines) {
            const uintptr_t joint = reinterpret_cast<uintptr_t>(a) | reinterpret_cast<uintptr_t>(b);
            return (joint & 1u) == 0u;
        } else {
            return false;
        }
    }

    /// Does an asynchronous request repay the engines (the arithmetic
    /// above, folded from the two frame costs kept for the configuration in
    /// force), and can they carry it? Its command phase counts as frames
    /// written, its data phase as frames read or written.
    [[gnu::always_inline]] static bool repays_engines(const Request& r) {
        const bool read = r.rx.get() != nullptr;
        const uint32_t cost = static_cast<uint32_t>(r.cmd_len) * write_frame_cost_ +
                              static_cast<uint32_t>(r.len) * (read ? read_frame_cost_ : write_frame_cost_);
        if (cost < (read ? async_read_budget_cycles : async_write_budget_cycles)) {
            return false;
        }
        return r.len != 0u ? carriable(r.bits, r.tx.get(), r.rx.get()) : carriable(r.bits, r.cmd.get(), nullptr);
    }

    /// Does an engined request's command phase ride the engines as a block
    /// of its own? When it is the whole request, or would repay a block
    /// alone and the engines can carry it; otherwise it runs polled inside
    /// start() ahead of the data block. A command block chained from the
    /// interrupt costs less than E (a launch and an interrupt), so the rule
    /// is the conservative side.
    [[gnu::always_inline]] static bool command_block(const Request& r) {
        return r.len == 0u || (static_cast<uint32_t>(r.cmd_len) * write_frame_cost_ >= async_write_budget_cycles &&
                               carriable(r.bits, r.cmd.get(), nullptr));
    }

    /// Does a polled request's data phase ride the engines inside start()?
    /// A read of polled_dma_min_frames() or more at the configuration in
    /// force, that the engines can carry; a write never does, the
    /// transmit-only loop keeping the wire's pace (the file header).
    [[gnu::always_inline]] static bool polled_rides_engines(const Request& r) {
        return r.rx.get() != nullptr && r.len >= polled_dma_min_ && carriable(r.bits, r.tx.get(), r.rx.get());
    }

    /// A block onto the engines - ONE body under either placement, with
    /// three call sites (a command block, a data block from start(), a data
    /// block chained from the vector): flattened, a launch is some 1 KB of
    /// the engines' programming, which every site inlining it would carry.
    static void launch_dma(const uint8_t* out, uint8_t* in, uint16_t count) {
        shared([out, in, count] { launch_dma_body(out, in, count); });
    }
    [[gnu::always_inline]] static void launch_dma_body(const uint8_t* out, uint8_t* in, uint16_t count) {
        dma_done_ = false;
        dma_active_ = true;
        if constexpr (wide_engines) {
            if (spi_frame_is_halfword(tenure_.bits)) {
                launch_frames<uint16_t>(out, in, count);
                return;
            }
        }
        launch_frames<uint8_t>(out, in, count);
    }

    /// A hot path's helper with more than one call site, kept as one body
    /// under either placement: out of line in the flash under InFlash,
    /// its own instance in `.fast_text.brio` under InRam - which the
    /// placed instances reach SRAM to SRAM, with no veneer.
    template <class F>
    [[gnu::always_inline]] static auto shared(F&& f) {
        if constexpr (std::is_same_v<Placement, InRam>) {
            return Hot<InRam>::run(f);
        } else {
            return out_of_line(f);
        }
    }
    template <class F>
    [[gnu::noinline, gnu::flatten]] static auto out_of_line(F&& f) {
        return f();
    }

    /// One block each way, F the frame's beat. THE REQUESTS ARE RAISED
    /// AROUND THE ENGINES AND NOWHERE ELSE (the file header). The receive
    /// channel first: its request rises only when a frame has come back,
    /// and the transmit side is what starts the clock. The byte buffers are
    /// handed over as runs of F: a 16-bit frame low-first IS a half-word in
    /// memory, and carriable() has checked the alignment that makes the
    /// address one. A null out buffer clocks the fixed dummy cell, a null
    /// in buffer (the command block's echo, a write) lands in the sink.
    template <typename F>
    static void launch_frames(const uint8_t* out, uint8_t* in, uint16_t count) {
        if (in != nullptr) {
            (void)RxEngine::start(reinterpret_cast<F*>(in), count);
        } else {
            (void)RxEngine::start_discard(reinterpret_cast<F*>(&rx_sink_), count);
        }
        S::dma_requests(false, true);
        if (out != nullptr) {
            (void)TxEngine::start(reinterpret_cast<const F*>(out), count);
        } else {
            (void)TxEngine::start_fixed(reinterpret_cast<const F*>(&tx_dummy_), count);
        }
        S::dma_requests(true, true);
    }

    /// The receive engine bound, at the family's HIGH level - the rule's
    /// for an overrunning receive (docs/design/dma.md): the host's clock
    /// does not wait for a starved receive channel, and a frame that lands
    /// on a full receive FIFO is lost (SSPRIS.RORRIS). Every bind of it is
    /// this one, so the level is stated once.
    static void arm_rx_engine() {
        if constexpr (has_engines) {
            RxEngine::arm(S::data_address(), S::dreq_rx, true);
        }
    }

    /// A command block ended: every command frame came back, so the bus is
    /// idle and D/C flips between the phases on the wire; then the data
    /// block onto the engines. Interrupt context; one launch.
    static bool next_block() {
        in_cmd_ = false;
        S::dma_requests(false, false);
        (void)TxEngine::complete();   // the receive block's end proves the transmit one's
        tenure_.dc.set();
        launch_dma(tenure_.tx.get(), tenure_.rx.get(), tenure_.len);
        return false;
    }

    /// One exit for a tenure on the engines. True when the vector's caller
    /// should post completion: an asynchronous tenure, its select released.
    static bool finish_dma(uint8_t st) {
        S::dma_requests(false, false);
        if (st != spi_ok) {
            status_ = st;
            (void)TxEngine::abandon();
            RxEngine::stop();
            arm_rx_engine();
            S::flush_rx();   // a frame the stopped channel never took
        } else if (TxEngine::busy()) {
            // The receive block completing is proof the transmit one did.
            (void)TxEngine::complete();
        }
        in_cmd_ = false;
        dma_active_ = false;
        dma_done_ = true;
        if (!tenure_.polled) {
            tenure_.cs.set();
            return true;
        }
        return false;
    }

    /// The polled request's wait on the DMA completion - bounded (the
    /// slowest frame is 65024 x 16 reference cycles; the budget scales).
    static void spin_dma() {
        uint32_t spins = spin_budget(tenure_.len);
        while (!dma_done_ && spins-- != 0u) {
        }
        if (!dma_done_) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();
            arm_rx_engine();
            dma_active_ = false;
            status_ = spi_dma_fault;
            S::flush_rx();
        }
    }

    /// A slower rate is a LARGER divisor, so the ceiling clamps from below.
    static SpiClock clamp(SpiClock c) {
        if (!ceiling_ || c.divisor() >= ceiling_->divisor()) {
            return c;
        }
        return *ceiling_;
    }

    static SpiConfig boot_config() {
        SpiConfig c{};
        c.role = SpiRole::host;
        c.mode = SpiMode::mode0;
        c.bits = 8;
        c.clock = ceiling_ ? *ceiling_ : SpiClocks::div16;
        return c;
    }

    /// The Request's frame size for the resource's bit count.
    static constexpr SpiDataSize spi_frame_size(uint8_t bits) {
        return bits == 16u ? SpiDataSize::bits16 : SpiDataSize::bits8;
    }

    /// Put the peripheral where this request wants it: ONE compare of
    /// the folded word against the one applied (the file header). A
    /// request whose word differs goes the long way - clamped to the
    /// ceiling, compared again, written only if it still differs.
    [[gnu::always_inline]] static void apply(const Request& r) {
        if (spi_control_word(r.clock, r.mode, r.bits) == applied_word_) {
            return;
        }
        reconfigure(clamp(r.clock), r.mode, r.bits);
    }

    /// The control registers are written with SSE clear, so any change
    /// costs a disable/enable pair - paid only when something moved - and
    /// the frame costs follow the new word. OUT OF LINE under either
    /// placement: a change of mode, rate or width is not a transaction's
    /// own path, and the hot body's flatten would otherwise carry it (with
    /// the resource's configure()) into the placed instance.
    [[gnu::noinline]] static void reconfigure(SpiClock c, SpiMode m, SpiDataSize bits) {
        const uint32_t word = spi_control_word(c, m, bits);
        if (word == applied_word_) {
            return;
        }
        applied_word_ = word;
        applied_.mode = m;
        applied_.clock = c;
        applied_.bits = spi_data_bits(bits);
        S::enable(false);
        (void)S::configure(applied_);
        S::enable(true);
        costs_for(c, m, bits);
    }

    /// The figures the short-request arithmetic folds a request from, kept
    /// for the configuration in force: computed where the control word is
    /// written and when the clocks move, never per request.
    static void costs_for(SpiClock c, SpiMode m, SpiDataSize bits) {
        if constexpr (has_dma) {
            const uint32_t frame = pl022_frame_cycles(core_per_sspclk_q8_, c, m, bits);
            read_frame_cost_ = static_cast<uint16_t>(pl022_polled_frame_cost(costs, frame, true));
            write_frame_cost_ = static_cast<uint16_t>(pl022_polled_frame_cost(costs, frame, false));
            polled_dma_min_ = pl022_polled_dma_min_frames(costs, frame);
        } else {
            (void)c;
            (void)m;
            (void)bits;
        }
    }

    // ---- the frames ---------------------------------------------------------

    /// The next frame out of an advancing pointer: one byte, or two
    /// low-first for a half-word; 0xFFFF from a null one.
    template <bool wide>
    [[gnu::always_inline]] static uint32_t next_frame(const uint8_t*& p) {
        if (p == nullptr) {
            return 0xFFFFu;
        }
        uint32_t v = p[0];
        if constexpr (wide) {
            v |= static_cast<uint32_t>(p[1]) << 8;
            p += 2;
        } else {
            p += 1;
        }
        return v;
    }
    /// A frame into an advancing pointer, the same packing.
    template <bool wide>
    [[gnu::always_inline]] static void store_frame(uint8_t*& p, uint32_t v) {
        p[0] = static_cast<uint8_t>(v);
        if constexpr (wide) {
            p[1] = static_cast<uint8_t>(v >> 8);
            p += 2;
        } else {
            p += 1;
        }
    }

    /// The spin budget of a polled transaction: one slowest frame (65024
    /// x 16 reference cycles, about 200 000 turns of the wait) per frame
    /// and one more, saturated.
    static uint32_t spin_budget(uint32_t frames) {
        return frames < 21'000u ? 200'000u * (frames + 1u) : 0xFFFF'FFFFu;
    }

    /// THE POLLED LOOP, THE RECEIVE SHAPE (the file header): `count`
    /// frames out of `out` (null = 0xFFFF) and into `in` (null = thrown
    /// away as they land), a FIFO's worth IN FLIGHT - written and not yet
    /// read back. The transmit FIFO is
    /// empty on entry - every earlier transaction took back or drained
    /// every frame it wrote - so the first `fifo_depth` frames go in with
    /// no flag read (12.3.4.3's priming, with the port enabled); then per
    /// frame: wait for RNE, read, and write the frame a FIFO's depth
    /// ahead, which cannot find the transmit FIFO full because fewer
    /// than its depth are in flight. NO HANDLER CAN OVERRUN IT: whatever
    /// runs while this loop is held off, the frames that can land in the
    /// receive FIFO are the ones in flight, and those are never more than
    /// its depth - measured under a tick handler stretched past eight
    /// frame times, every frame exact and the overrun flag clear
    /// (docs/rp2350/spi.md). False when `budget` ran out waiting - the
    /// block stopped.
    template <bool wide>
    static bool run_duplex(const uint8_t* out, uint8_t* in, uint16_t count, uint32_t& budget) {
        Regs& regs = S::regs();
        const uint16_t ahead = count < S::fifo_depth ? count : S::fifo_depth;
        const uint8_t* wp = out;
        for (uint16_t k = 0; k < ahead; ++k) {
            regs.SSPDR = next_frame<wide>(wp);
        }
        uint8_t* rp = in;
        for (uint16_t k = count; k != 0u; --k) {
            while ((regs.SSPSR & SpiFlag::rx_not_empty) == 0u) {
                if (budget-- == 0u) {
                    return false;
                }
            }
            const uint32_t v = regs.SSPDR;
            if (in != nullptr) {
                store_frame<wide>(rp, v);
            }
            if (k > ahead) {
                regs.SSPDR = next_frame<wide>(wp);
            }
        }
        return true;
    }

    /// THE POLLED LOOP, THE TRANSMIT-ONLY SHAPE: `count` frames out of
    /// `out` (null = 0xFFFF) with the answers thrown away. It paces on
    /// TNF alone and reads nothing per frame (two accesses of the block a
    /// frame: SSPSR, SSPDR out), so the receive FIFO fills and OVERRUNS
    /// BY DESIGN: a frame pushed on a full receive FIFO is dropped and
    /// RORRIS raised while the block keeps shifting, which costs nothing
    /// while RORIM is masked - and this host never arms it. The tail
    /// waits for BSY to clear (the last frame shifted, the transmit FIFO
    /// empty), drains the receive FIFO and clears the overrun, so the
    /// block is left as every other path leaves it. THAT TAIL IS ITS
    /// PRICE: up to a FIFO's worth of reads after the wire has finished,
    /// which the receive shape spends under the wire instead - so this
    /// shape is taken only where the receive shape would fall behind the
    /// wire, which on both of the RP2350's cores is a divisor below four
    /// on 8-bit frames (letter e: 27 and 38 cycles a frame of loop against
    /// 16 of wire at clk_peri / 2; at / 4 and below both shapes read the
    /// wire's own figure), and only on a run longer than the FIFO, where
    /// the tail is shorter than what it saves.
    template <bool wide>
    static bool run_write(const uint8_t* out, uint16_t count, uint32_t& budget) {
        Regs& regs = S::regs();
        const uint8_t* wp = out;
        for (uint16_t k = count; k != 0u; --k) {
            while ((regs.SSPSR & SpiFlag::tx_not_full) == 0u) {
                if (budget-- == 0u) {
                    return false;
                }
            }
            regs.SSPDR = next_frame<wide>(wp);
        }
        while ((regs.SSPSR & SpiFlag::busy) != 0u) {
            if (budget-- == 0u) {
                return false;
            }
        }
        while ((regs.SSPSR & SpiFlag::rx_not_empty) != 0u) {
            (void)regs.SSPDR;
        }
        regs.SSPICR = SpiInterrupt::overrun;
        return true;
    }

    /// One phase of a polled request, by its shape and its width: the
    /// receive shape - discarding when there is no in buffer - unless the
    /// run is a write longer than the FIFO at a rate the receive shape
    /// cannot keep (run_write's comment: a divisor below four on 8-bit
    /// frames; a 16-bit frame at that rate is twice as long and the
    /// receive shape keeps it).
    static bool run_polled(const uint8_t* out, uint8_t* in, uint16_t count, SpiDataSize bits,
                           uint32_t& budget) {
        if (spi_frame_is_halfword(bits)) {
            return run_duplex<true>(out, in, count, budget);
        }
        if (in == nullptr && count > S::fifo_depth && applied_.clock.divisor() < 4u) {
            return run_write<false>(out, count, budget);
        }
        return run_duplex<false>(out, in, count, budget);
    }

    /// After a polled wait ran out: the block is left as it is (the
    /// arbiter's recover() is the way back), the frames still in the
    /// FIFOs let out and thrown away, the status says what happened.
    [[gnu::noinline, gnu::cold]] static void settle_fault() {
        status_ = spi_dma_fault;
        uint32_t spins = 200'000u;
        while (S::busy() && spins-- != 0u) {
        }
        S::flush_rx();
    }

    // ---- the state ----------------------------------------------------------

    /// The request an engined transaction is running (take_tenure()).
    static inline Request tenure_{};
    /// A command block is on the engines; its end launches the data block.
    static inline bool in_cmd_ = false;
    static inline uint8_t status_ = spi_ok;
    static inline volatile bool dma_done_ = false;
    static inline volatile bool dma_active_ = false;
    /// What a polled frame costs the core at the configuration in force,
    /// read and written, and from how many frames a polled read rides the
    /// engines (costs_for()): what start() decides a request's way from.
    static inline uint16_t read_frame_cost_ = 0;
    static inline uint16_t write_frame_cost_ = 0;
    static inline uint16_t polled_dma_min_ = 0xFFFFu;
    /// SSPCLK's cycle in the core's, in 256ths (rebase()).
    static inline uint32_t core_per_sspclk_q8_ = 256u;
    // One cell each serves both beats: a byte access at a half-word's
    // address is its low byte on these little-endian cores. THE DUMMY
    // LIVES IN RAM, never as a constant: the transmit engine reads it once
    // a frame, and a cell in a flash executed in place is a read through
    // the XIP cache for every frame, which competes there with the core's
    // own instruction fetches - measured on the RP2040, polled engined
    // blocks at clk_peri / 4 ran up to five per cent under the wire with
    // the host's code in the flash beside them, the wire's own rate with
    // the cell in RAM (docs/pl022/README.md).
    static inline uint16_t rx_sink_ = 0;
    static inline uint16_t tx_dummy_ = 0xFFFFu;
    static inline SpiConfig applied_{};
    static inline uint32_t applied_word_ = 0;
    static inline uint32_t pclk_hz_ = 0;
    static inline uint32_t ceiling_hz_ = 0;
    static inline std::optional<SpiClock> ceiling_{};
    static inline typename Chip::DelayRate cs_rate_{};
};

// =============================================================================
// The client task
// =============================================================================

/**
 * Pl022Client<Chip, n, pins>
 *
 * The other end of the wire: SCK, TX-in and the select are inputs, the
 * host sets the pace, and the only thing this side controls is what it
 * has ready to shift out when the next clock arrives - up to
 * `frames_ahead` frames in the transmit FIFO, written on TNF. A client
 * that falls behind does not stall the bus: what the host reads is
 * whatever the shifter held, a wrong answer and never a missing one. The
 * select is the pad: the PL022 in slave mode frames on SSPFSSIN, so
 * `pins.cs` goes to the peripheral and `selected()` reads the same pad.
 * `drive_output(false)` leaves the transmit line undriven, a dark
 * listener on a shared harness - the pad taken away to no function with no
 * pull, since what SOD does to a pad is the chip's fact and not the
 * block's (the file header).
 * The family's own header aliases this to `SpiClient<n, pins>`.
 */
template <Pl022Chip Chip, uint8_t n, typename Chip::Pins pins>
class Pl022Client {
    using S = Pl022Ssp<Chip, n>;
    static_assert(Chip::pins_valid(n, pins),
                  "brio SpiClient: these pads cannot carry this instance's signals - see the "
                  "family's own pin table");
    static constexpr uint8_t sck_pin = Chip::sck_pad(pins);
    static constexpr uint8_t tx_pin = Chip::tx_pad(pins);
    static constexpr uint8_t rx_pin = Chip::rx_pad(pins);
    static constexpr uint8_t cs_pin = Chip::cs_pad(pins);
    static_assert(rx_pin != Chip::no_pad && cs_pin != Chip::no_pad,
                  "brio SpiClient: a client listens on RX and frames on its CS pad");
    using SckPad = typename Chip::template Pad<sck_pin>;
    using RxPad = typename Chip::template Pad<rx_pin>;
    using CsPad = typename Chip::template Pad<cs_pin>;
    using TxPad = typename Chip::template Pad<tx_pin != Chip::no_pad ? tx_pin : sck_pin>;

public:
    Pl022Client() = delete;
    using Resource = S;
    static constexpr typename Chip::Pins pin_set = pins;

    struct Config {
        SpiMode mode = SpiMode::mode0;
        SpiFormat format = SpiFormat::motorola;
        uint8_t bits = 8;             ///< 4..16
        /// Whether the transmit line is driven at all (SOD clear).
        bool drive_output = true;
    };

    /// Bring the instance up as a client. `clock` is the app's Clock tag:
    /// SSPCLK must run at least twelve times the host's clock, which is
    /// the caller's to know. False when the block did not come up or the
    /// configuration is refused.
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg = {}) {
        (void)clock;
        Chip::Interrupts::disable(S::irq());
        if (!S::reset()) {
            return false;
        }
        SpiConfig c{};
        c.role = SpiRole::client;
        c.mode = cfg.mode;
        c.format = cfg.format;
        c.bits = cfg.bits;
        if (!S::configure(c)) {
            return false;
        }
        bits_ = cfg.bits;
        S::interrupts(SpiInterrupt::all, false);
        // Inputs first, the output last.
        SckPad::function(Chip::pad_function, Chip::sck_pad_config());
        RxPad::function(Chip::pad_function, Chip::client_rx_pad_config());
        CsPad::function(Chip::pad_function, Chip::cs_pad_config());
        drive_output(cfg.drive_output);
        S::flush_rx();
        Chip::Interrupts::enable(S::irq());
        return true;
    }

    /// The answer line handed to the peripheral, or taken away to no
    /// function with no pull - the dark listener (the file header: the PAD
    /// is the lever, because SOD's effect on one is the chip's fact).
    static void drive_output(bool on) {
        if constexpr (tx_pin != Chip::no_pad) {
            if (on) {
                TxPad::function(Chip::pad_function, Chip::tx_pad_config());
            } else {
                TxPad::function(Chip::no_function, Chip::dark_pad_config());
            }
        } else {
            (void)on;
        }
    }
    /// SOD, the block's own slave-output-disable bit, for a program that
    /// wants to see what it does on its board.
    static void sod(bool on) { S::output_disabled(on); }

    /// SSE up with the first answer in the FIFO before the host's clock
    /// can arrive. Call it with the select still high.
    static void enable(uint16_t first = 0xFFFFu) {
        S::enable(true);
        write(first);
    }
    static void disable() { S::enable(false); }

    /// How many answers may be queued ahead of the frame the host is
    /// about to clock: the transmit FIFO's depth.
    static constexpr uint8_t frames_ahead = S::fifo_depth;

    /// Load the next frame to shift out (on TNF).
    static void write(uint16_t v) { S::write_data(v); }
    static bool writable() { return S::tx_not_full(); }
    /// One received frame, or nothing.
    static std::optional<uint16_t> poll() {
        if (!S::rx_not_empty()) {
            return {};
        }
        return static_cast<uint16_t>(S::read_data() & ((1u << bits_) - 1u));
    }
    /// Is the host holding the select low right now? A live pad read.
    static bool selected() { return !CsPad::read(); }

    static bool overrun() { return (S::raw_pending() & SpiInterrupt::overrun) != 0u; }
    static void clear_overrun() { S::clear_pending(SpiInterrupt::overrun); }
    static uint32_t flags() { return S::flags(); }

    /// The ISR body: the raised-and-enabled sources, for the app's glue.
    [[gnu::always_inline]] static uint32_t isr() { return S::isr(); }
    static void interrupts(uint32_t mask, bool on) { S::interrupts(mask, on); }

    static void release() {
        Chip::Interrupts::disable(S::irq());
        S::interrupts(SpiInterrupt::all, false);
        S::enable(false);
        SckPad::release();
        RxPad::release();
        CsPad::release();
        if constexpr (tx_pin != Chip::no_pad) {
            TxPad::release();
        }
        S::hold();
    }

private:
    static inline uint8_t bits_ = 8;
};

} // namespace brio
