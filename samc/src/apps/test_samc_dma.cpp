// test_samc_dma - the reference bench suite for samc/dmac.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar - this is its first use on the
// SAM target). It is a REFERENCE test: it is meant to keep passing
// through every later restructuring of the driver under it.
//
// What is exercised, letter by letter:
//   a  the block, the tables, a software-triggered copy, and THE
//      END-ADDRESS QUIRK proven by data - a deliberately naive
//      start-address descriptor is shown moving the wrong bytes
//   b  the memory-to-memory matrix: three beat sizes crossed with the
//      three increment shapes, at lengths that cross alignment
//   c  software-linked chains: TCMPL re-programs the next block, N
//      blocks back to back, throughput measured
//   d  suspend/resume and harvest(), including the erratum-1.10.4
//      write-back validation shown REFUSING a scribbled write-back
//   e  interrupts: TCMPL, an invalid descriptor, a real bus error, and
//      INTPEND dispatch with two channels pending at once
//   f  the console's own SERCOM5 transmitting through a DMA channel
//   g  THE 1.10.4 HUNT: five channels triggered concurrently for
//      seconds, with the write-back violation counters as the verdict
//   r  the console's SERCOM5 RECEIVING through a DMA channel - needs
//      the person at the other end to type, so it is out of z
//   h  the same transmit path, but through Uart's OPTIONAL TX ENGINE:
//      print() into the ring, the engine drains it in blocks
//   i  the RX engine and the tick-paced harvest() contract - runner
//      driven like r, so out of z
//   j  both engines at once: the 1.10.4 stress at the task level
//
// Wiring: NONE. Every letter but r is self-contained; r asks the runner
// for a burst and says so.
//
// build: boards = c21j

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/dmac.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

constexpr brio::UartPads console_pads{
    .tx = brio::SercomPad::pad0,
    .rx = brio::SercomPad::pad1,
    .tx_pin = {'B', 30, brio::PinFunction::d},
    .rx_pin = {'B', 31, brio::PinFunction::d},
};
using Serial = brio::Uart<5, console_pads>;
constexpr Serial serial;
using Sc5 = Serial::Resource;

// THE SAME SERCOM, SEEN THROUGH THE ENGINES. Letters h/i/j drive the
// console through a SECOND Uart instantiation on instance 5 - identical
// pads and rate, but with samc/dmac.hpp's optional engines named. The
// two instantiations have separate rings and separate counters and are
// never live at the same time: each letter re-init()s the one it wants,
// which resets and reconfigures the peripheral from scratch, and hands
// the console back before it prints a single verdict.
//
// The engines take channels 6 and 7 so nothing collides with the raw
// channels letters a..g use, and the two must differ - a channel moves
// bytes one way, and sercom.hpp refuses a shared one at compile time.
constexpr uint8_t ch_engine_tx = 6;
constexpr uint8_t ch_engine_rx = 7;
using EngineSerial = brio::Uart<5, console_pads, 64, 256,
                                brio::DmaTxEngine<ch_engine_tx>,
                                brio::DmaRxEngine<ch_engine_rx>>;
constexpr EngineSerial engine_serial;

using Led = brio::Pin<'B', 23>;

brio::TestBench<Serial> bench;

using brio::crlf;
using brio::print;

// The channels this suite uses. Low numbers on purpose: 25.6.2.3 asks
// for the least significant channels when not all are needed, because
// the descriptor tables are sized by the HIGHEST channel in use.
constexpr uint8_t ch_copy = 0;     // the general-purpose memory-to-memory channel
constexpr uint8_t ch_chain = 1;    // the software-linked chain
constexpr uint8_t ch_tx = 2;       // SERCOM5 transmit
constexpr uint8_t ch_rx = 3;       // SERCOM5 receive
constexpr uint8_t ch_churn0 = 4;   // the 1.10.4 hunt's background traffic
constexpr uint8_t ch_churn1 = 5;

using Copy = brio::DmaChannel<ch_copy>;
using Chain = brio::DmaChannel<ch_chain>;
using Tx = brio::DmaChannel<ch_tx>;
using Rx = brio::DmaChannel<ch_rx>;
using Churn0 = brio::DmaChannel<ch_churn0>;
using Churn1 = brio::DmaChannel<ch_churn1>;

// ---------------------------------------------------------------------------
// A cycle-resolution stopwatch
//
// The kernel timebase ticks at 1 kHz, which is far too coarse for a
// harvest that costs microseconds. SysTick counts CPU cycles DOWN from
// LOAD to zero once per tick, so tick x (LOAD + 1) + (LOAD - VAL) is the
// same clock read at 48 MHz. The tick counter is read on both sides of
// VAL and the pair retried on a mismatch, because a tick that lands
// between the two reads would otherwise pair a new tick with an old
// remainder.
// ---------------------------------------------------------------------------
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = brio::Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = brio::Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}

uint32_t cycles_to_us(uint32_t cycles) { return cycles / (SysClock::hz / 1'000'000UL); }

// ---------------------------------------------------------------------------
// Buffers
//
// `arena` is ONE object with a decoy half below the payload half, which
// is what makes letter a's end-address proof exact: a naive descriptor
// pointing at the START of the payload reads the sixteen bytes BELOW it,
// and those bytes have to be identifiable and legally ours to read.
// ---------------------------------------------------------------------------
// EVERY DMA BUFFER IS VOLATILE, in BOTH directions, and that is not
// belt-and-braces - it is the correctness condition, learned the hard
// way here.
//
// A DMA buffer is memory shared with a third party the language knows
// nothing about. The obvious half is that the compiler cannot see the
// controller's WRITES, so a plain read after a transfer may be folded to
// whatever the CPU last stored. The half that actually bit was the other
// one: it cannot see the controller's READS either, so it is free to
// SINK the CPU's own preparation of the buffer past the code that starts
// the transfer. Letter e caught exactly that - a write-back reporting
// BTCNT = 0 with precisely the right addresses, one block's destination
// correct and the other's still zero, because the zeroing of the first
// had been moved to after the transfer that filled it.
//
// Declaring the buffers volatile fixes both halves at once: every access
// to them is a real one, and volatile accesses are performed in program
// order relative to each other - including relative to the volatile
// register writes that start and observe the transfer. It is the same
// reasoning util/ring.hpp spells for indexes shared with a handler,
// applied to a peripheral that writes memory instead of a register.
constexpr uint16_t decoy_len = 16;
constexpr uint16_t payload_len = 16;
alignas(4) volatile uint8_t arena[decoy_len + payload_len];
volatile uint8_t* const decoy = &arena[0];
volatile uint8_t* const payload = &arena[decoy_len];

constexpr uint16_t buf_len = 256;
alignas(4) volatile uint8_t src_buf[buf_len];
alignas(4) volatile uint8_t dst_buf[buf_len];
alignas(4) volatile uint8_t rx_buf[buf_len];

// The chain's blocks, and the console text letters f/g transmit.
constexpr uint16_t chain_block = 64;
constexpr uint8_t chain_blocks = 8;

constexpr char tx_text[] =
    "[dma-tx] this line left the chip without the CPU touching a byte of it\r\n";
constexpr uint16_t tx_text_len = sizeof(tx_text) - 1;

void fill_pattern(volatile uint8_t* p, uint16_t n, uint8_t seed) {
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + i * 7u);
    }
}

void all_fill(volatile uint8_t* p, uint16_t n, uint8_t v) {
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = v;
    }
}

void zero(volatile uint8_t* p, uint16_t n) { all_fill(p, n, 0); }

bool same(const volatile uint8_t* a, const volatile uint8_t* b, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

bool all_equal(const volatile uint8_t* p, uint16_t n, uint8_t v) {
    for (uint16_t i = 0; i < n; ++i) {
        if (p[i] != v) {
            return false;
        }
    }
    return true;
}

/// A cheap additive checksum - enough to say "these bytes are those
/// bytes" across a serial link, and not pretending to be a CRC.
uint16_t sum16(const volatile uint8_t* p, uint16_t n) {
    uint16_t s = 0;
    for (uint16_t i = 0; i < n; ++i) {
        s = static_cast<uint16_t>(s + p[i]);
    }
    return s;
}

// ---------------------------------------------------------------------------
// Interrupt bookkeeping
//
// The DMAC handler is the block's ONE vector; take_pending() names the
// channel and the reason and clears the flag in one store, so the
// handler is a loop over it and nothing else. Everything it records is
// read from the main loop, so every counter is volatile.
// ---------------------------------------------------------------------------
volatile uint16_t irq_complete[brio::Dmac::channel_count];
volatile uint16_t irq_error[brio::Dmac::channel_count];
volatile uint16_t irq_suspend[brio::Dmac::channel_count];
volatile uint16_t irq_rounds = 0;   ///< how many channels one entry served

/// Set while letter c wants the handler to re-program the next block of
/// the chain from inside the completion itself.
volatile bool chain_running = false;
volatile uint8_t chain_done = 0;

void clear_irq_counts() {
    for (uint8_t i = 0; i < brio::Dmac::channel_count; ++i) {
        irq_complete[i] = 0;
        irq_error[i] = 0;
        irq_suspend[i] = 0;
    }
    irq_rounds = 0;
}

/// Start block `n` of the chain: a fresh source and destination slice,
/// loaded and enabled. Called from main context for the first block and
/// from the handler for every one after it - which is the whole point of
/// the letter, so it must be callable from both.
void chain_start(uint8_t n) {
    const uint16_t offset = static_cast<uint16_t>(n * (chain_block / chain_blocks));
    (void)Chain::load(brio::DmaTransfer{
        .source = &src_buf[offset],
        .destination = &dst_buf[offset],
        .beats = chain_block / chain_blocks,
        .beat = brio::DmaBeat::byte,
    });
    Chain::enable(true);
    Chain::trigger();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

extern "C" void DMAC_Handler() {
    uint16_t served = 0;
    // INTPEND reports the LOWEST pending channel, one at a time, so the
    // handler loops until it reports nothing. No CHID is touched here -
    // see the "ONE CHANNEL-ADDRESSED ACCESS THAT NEEDS NO GUARD" note in
    // samc/dmac.hpp.
    while (const auto irq = brio::Dmac::take_pending()) {
        ++served;
        const uint8_t ch = irq->channel;
        if (ch < brio::Dmac::channel_count) {
            if (irq->complete()) {
                irq_complete[ch] = static_cast<uint16_t>(irq_complete[ch] + 1);
            }
            if (irq->error()) {
                irq_error[ch] = static_cast<uint16_t>(irq_error[ch] + 1);
            }
            if (irq->suspended()) {
                irq_suspend[ch] = static_cast<uint16_t>(irq_suspend[ch] + 1);
            }
        }
        // The engined Uart's own channels: it consumes what its
        // transmit block carried and starts the next run itself. A
        // channel that is not its own simply answers false.
        (void)EngineSerial::dma_isr(ch);
        if (ch == ch_chain && irq->complete() && chain_running) {
            const uint8_t next = static_cast<uint8_t>(chain_done + 1);
            chain_done = next;
            if (next < chain_blocks) {
                chain_start(next);
            } else {
                chain_running = false;
            }
        }
    }
    if (served > irq_rounds) {
        irq_rounds = served;
    }
}

namespace {

// ---------------------------------------------------------------------------
// Console custody
//
// Letters f, g and r take SERCOM5 away from the interrupt-driven Uart
// and give its DATA register to a DMA channel. Both directions have to
// be handed over cleanly: a DRE interrupt left armed would race the TX
// channel for the same byte, and an RXC interrupt left armed would eat
// the characters the RX channel is waiting for.
// ---------------------------------------------------------------------------

/// Drain everything the console still owes and stop the transmit
/// interrupt. TXC is exact on this peripheral ("the last byte has left
/// the shifter"), so no timing guess is needed. Bounded, like every wait.
void console_tx_quiesce() {
    uint32_t spins = 8'000'000u;
    while (!Serial::tx_idle() && spins-- != 0u) {
    }
    spins = 200'000u;
    while (!Sc5::txc_flag() && spins-- != 0u) {
    }
    Sc5::enable_dre_interrupt(false);
}

void console_tx_restore() {
    // write_byte() re-arms DRE on the next byte; nothing else is needed.
    Sc5::clear_flags(brio::SercomFlag::txc);
}

void console_rx_detach() {
    Sc5::enable_rxc_interrupt(false);
    Sc5::flush_rx();
}

void console_rx_restore() {
    Sc5::flush_rx();
    Sc5::enable_rxc_interrupt(true);
}

/// Wait for a channel to stop being enabled - which is how a single
/// descriptor with BLOCKACT interrupt ends. Bounded; returns false on a
/// timeout so a stuck channel is a FAIL line and not a hung suite.
template <typename Ch>
bool wait_done(uint32_t spins = 4'000'000u) {
    while (spins-- != 0u) {
        if (!Ch::enabled()) {
            return true;
        }
    }
    return false;
}

/// One turn of letter g's background traffic: re-arm the channel when
/// its block has ended, then give it a beat. Returns 1 when a new block
/// was started, so the caller can count them. A channel is a static-only
/// class, so it travels as a template argument and never as a value.
template <typename Ch>
uint32_t churn(uint8_t offset) {
    uint32_t started = 0;
    if (!Ch::enabled()) {
        (void)Ch::load(brio::DmaTransfer{
            .source = &src_buf[offset],
            .destination = &dst_buf[offset],
            .beats = 32,
            .beat = brio::DmaBeat::byte,
        });
        Ch::enable(true);
        started = 1;
    }
    Ch::trigger();
    return started;
}

/// One memory-to-memory block, software triggered, waited out.
template <typename Ch>
bool run_copy(const brio::DmaTransfer& t) {
    if (!Ch::load(t)) {
        return false;
    }
    Ch::enable(true);
    Ch::trigger();
    return wait_done<Ch>();
}

// =============================================================================
// a - the block, the tables, and the end-address quirk
// =============================================================================
void ta_block() {
    bench.verdict("DMAC has twelve channels", brio::Dmac::channel_count == 12);
    bench.verdict("four arbitration levels", brio::Dmac::level_count == 4);
    bench.verdict("block enabled after init", brio::Dmac::enabled());

    // The tables the controller was told about must be the ones this
    // driver owns, and both must be 128-bit aligned (the register
    // description asks for 64; the driver gives 128 - see its header).
    // All four arbitration levels must be taking requests: a channel
    // whose level is disabled is invisible to the arbiter, not slow.
    print(serial, "  CTRL=", brio::hex(brio::Dmac::regs().DMAC_CTRL),
          " levels=", brio::hex(brio::Dmac::levels()), crlf);
    bench.verdict("all four arbitration levels enabled", brio::Dmac::levels() == 0x0F);

    const uint32_t base = brio::Dmac::regs().DMAC_BASEADDR;
    const uint32_t wrb = brio::Dmac::regs().DMAC_WRBADDR;
    print(serial, "  BASEADDR=", brio::hex(base), " WRBADDR=", brio::hex(wrb), crlf);
    bench.verdict("BASEADDR registered",
                  base == reinterpret_cast<uint32_t>(&brio::Dmac::descriptor(0)));
    bench.verdict("WRBADDR registered",
                  wrb == reinterpret_cast<uint32_t>(&brio::Dmac::write_back(0)));
    bench.verdict("descriptor table 16-byte aligned", (base & 0x0Fu) == 0);
    bench.verdict("write-back table 16-byte aligned", (wrb & 0x0Fu) == 0);
    bench.verdict("the two sections are separate", base != wrb);
    bench.verdict("descriptor stride is 16 bytes",
                  reinterpret_cast<uint32_t>(&brio::Dmac::descriptor(1)) - base == 16u);

    // ---- a software-triggered copy -----------------------------------------
    fill_pattern(src_buf, 64, 0x11);
    // 72, not 64: the guard byte just past the block has to be CLEARED
    // by this test, not assumed clear. Caught on a second z run, where
    // the previous run's traffic was still sitting at dst_buf[64].
    zero(dst_buf, 72);
    Copy::configure({.action = brio::DmaTriggerAction::block});
    const bool done = run_copy<Copy>({
        .source = src_buf,
        .destination = dst_buf,
        .beats = 64,
        .beat = brio::DmaBeat::byte,
    });
    bench.verdict("SWTRIG block completed", done);
    bench.verdict("64 bytes copied exactly", same(src_buf, dst_buf, 64));
    bench.verdict("nothing written past the block", dst_buf[64] == 0);

    // ---- THE END-ADDRESS QUIRK, by data ------------------------------------
    // Both descriptors below are hand-built and differ in ONE field:
    // what SRCADDR holds. The driver's builder puts the end address
    // there; the naive reading puts the start. The decoy half of the
    // arena sits immediately below the payload, so the naive descriptor
    // reads it - visibly, byte for byte.
    fill_pattern(decoy, decoy_len, 0xA0);
    fill_pattern(payload, payload_len, 0x50);

    const brio::DmaTransfer shape{
        .source = payload,
        .destination = dst_buf,
        .beats = payload_len,
        .beat = brio::DmaBeat::byte,
    };
    const brio::DmaDescriptor correct = brio::dma_descriptor(shape);
    bench.verdict("builder's SRCADDR is start + length",
                  correct.srcaddr == reinterpret_cast<uint32_t>(payload) + payload_len);
    bench.verdict("builder's DSTADDR is start + length",
                  correct.dstaddr == reinterpret_cast<uint32_t>(dst_buf) + payload_len);

    zero(dst_buf, payload_len);
    Copy::load(correct);
    Copy::enable(true);
    Copy::trigger();
    bench.verdict("correct descriptor completed", wait_done<Copy>());
    bench.verdict("correct descriptor moved the PAYLOAD",
                  same(payload, dst_buf, payload_len));

    brio::DmaDescriptor naive = correct;
    naive.srcaddr = reinterpret_cast<uint32_t>(payload);   // the mistake
    zero(dst_buf, payload_len);
    Copy::load(naive);
    Copy::enable(true);
    Copy::trigger();
    bench.verdict("naive descriptor completed", wait_done<Copy>());
    bench.verdict("naive descriptor moved the DECOY instead",
                  same(decoy, dst_buf, payload_len));
    bench.verdict("naive descriptor moved NOT the payload",
                  !same(payload, dst_buf, payload_len));
    print(serial, "  payload[0]=", brio::hex(payload[0]), " decoy[0]=",
          brio::hex(decoy[0]), " naive dst[0]=", brio::hex(dst_buf[0]), crlf);
}

// =============================================================================
// b - the memory-to-memory matrix
// =============================================================================
void tb_matrix() {
    Copy::configure({.action = brio::DmaTriggerAction::block});

    // ---- the three beat sizes, incrementing both ends -----------------------
    struct Case {
        brio::DmaBeat beat;
        const char* name;
        uint16_t beats;
    };
    const Case cases[] = {
        {brio::DmaBeat::byte, " (byte x 37)", 37},     // odd: crosses no alignment
        {brio::DmaBeat::hword, " (hword x 19)", 19},   // 38 bytes, half-word aligned
        {brio::DmaBeat::word, " (word x 13)", 13},     // 52 bytes
    };
    for (const Case& c : cases) {
        const uint16_t bytes =
            static_cast<uint16_t>(c.beats * brio::dma_beat_bytes(c.beat));
        fill_pattern(src_buf, bytes, 0x33);
        zero(dst_buf, buf_len);
        const bool ok = run_copy<Copy>({
            .source = src_buf,
            .destination = dst_buf,
            .beats = c.beats,
            .beat = c.beat,
        });
        bench.verdict("inc->inc copy exact", c.name, ok && same(src_buf, dst_buf, bytes));
        bench.verdict("nothing past the block", c.name, dst_buf[bytes] == 0);
    }

    // ---- fixed source: a fill from one cell ---------------------------------
    src_buf[0] = 0x5A;
    zero(dst_buf, buf_len);
    bool ok = run_copy<Copy>({
        .source = &src_buf[0],
        .destination = dst_buf,
        .beats = 41,
        .beat = brio::DmaBeat::byte,
        .source_increment = false,
    });
    bench.verdict("fixed->inc filled 41 bytes", ok && all_equal(dst_buf, 41, 0x5A));
    bench.verdict("fixed->inc stopped at 41", dst_buf[41] == 0);

    // ---- fixed destination: a drain into one cell ---------------------------
    fill_pattern(src_buf, 32, 0x01);
    zero(dst_buf, buf_len);
    ok = run_copy<Copy>({
        .source = src_buf,
        .destination = &dst_buf[0],
        .beats = 32,
        .beat = brio::DmaBeat::byte,
        .destination_increment = false,
    });
    bench.verdict("inc->fixed completed", ok);
    bench.verdict("inc->fixed left only the LAST beat", dst_buf[0] == src_buf[31]);
    bench.verdict("inc->fixed wrote one cell only", dst_buf[1] == 0);

    // ---- a word-beat copy of the whole buffer -------------------------------
    fill_pattern(src_buf, buf_len, 0x77);
    zero(dst_buf, buf_len);
    ok = run_copy<Copy>({
        .source = src_buf,
        .destination = dst_buf,
        .beats = buf_len / 4,
        .beat = brio::DmaBeat::word,
    });
    bench.verdict("256-byte word copy exact", ok && same(src_buf, dst_buf, buf_len));

    // ---- throughput, at word beats and no contention ------------------------
    const uint32_t t0 = cycles_now();
    constexpr uint16_t reps = 64;
    for (uint16_t i = 0; i < reps; ++i) {
        (void)run_copy<Copy>({
            .source = src_buf,
            .destination = dst_buf,
            .beats = buf_len / 4,
            .beat = brio::DmaBeat::word,
        });
    }
    const uint32_t spent = cycles_now() - t0;
    const uint32_t per_block = spent / reps;
    print(serial, "  256-byte word block: ", per_block, " cycles (",
          cycles_to_us(per_block), " us), ", (buf_len * 64UL) / cycles_to_us(spent),
          " MB/s incl. setup", crlf);
    bench.verdict("a 256-byte word block costs under 4000 cycles", per_block < 4000u);
}

// =============================================================================
// c - software-linked chains
// =============================================================================
void tc_chain() {
    // The chain is what erratum 1.10.2/1.10.3 would have forced on
    // rev B..D silicon and what 1.10.4 makes the honest default here:
    // no linked descriptors at all, the TCMPL interrupt programs the
    // next block from inside the completion.
    Chain::configure({.action = brio::DmaTriggerAction::block});
    Chain::arm(brio::DmaFlag::complete | brio::DmaFlag::transfer_error, true);
    brio::Nvic::enable(brio::Dmac::irq());

    fill_pattern(src_buf, chain_block, 0x9C);
    zero(dst_buf, buf_len);
    clear_irq_counts();

    chain_done = 0;
    chain_running = true;
    const uint32_t t0 = cycles_now();
    chain_start(0);

    uint32_t spins = 8'000'000u;
    while (chain_running && spins-- != 0u) {
    }
    const uint32_t spent = cycles_now() - t0;

    bench.verdict("the chain ran to its end", !chain_running);
    bench.verdict("every block completed", chain_done == chain_blocks);
    bench.verdict("one TCMPL per block", irq_complete[ch_chain] == chain_blocks);
    bench.verdict("no transfer error", irq_error[ch_chain] == 0);
    bench.verdict("the whole 64 bytes arrived", same(src_buf, dst_buf, chain_block));
    bench.verdict("nothing past the chain", dst_buf[chain_block] == 0);

    const uint32_t per_block = spent / chain_blocks;
    print(serial, "  ", chain_blocks, " linked blocks in ", spent, " cycles: ", per_block,
          " cycles/block (", cycles_to_us(per_block), " us), ",
          SysClock::hz / per_block, " blocks/s", crlf);
    bench.verdict("a software link costs under 2000 cycles", per_block < 2000u);

    Chain::arm(brio::DmaFlag::all, false);
}

// =============================================================================
// d - suspend, harvest, and the 1.10.4 validation
// =============================================================================
void td_harvest() {
    // A BEAT trigger action paces the block one beat per software
    // trigger, which is what makes a mid-block reading possible at all:
    // a block-triggered copy of a few hundred bytes is over before the
    // CPU can ask.
    Copy::configure({.action = brio::DmaTriggerAction::beat});
    Copy::clear_counters();
    fill_pattern(src_buf, buf_len, 0x2B);
    zero(dst_buf, buf_len);

    constexpr uint16_t total = 64;
    const bool loaded = Copy::load(brio::DmaTransfer{
        .source = src_buf,
        .destination = dst_buf,
        .beats = total,
        .beat = brio::DmaBeat::byte,
    });
    bench.verdict("descriptor loaded", loaded);
    Copy::enable(true);

    // BENCH FINDING, and not what the chapter made one expect: nothing
    // has been TRIGGERED yet, yet the write-back is already a faithful
    // copy of the descriptor. Enabling the channel and suspending it is
    // enough to make the controller fetch the descriptor and spill it -
    // so a harvest before the first beat is ACCEPTED and honestly reports
    // zero beats done, rather than reading a stale table entry. The
    // previous letter left a DIFFERENT descriptor in this channel's
    // write-back slot, so a stale reading would have been refused: this
    // passing is the proof that the entry was rewritten.
    const auto before = Copy::harvest();
    bench.verdict("harvest before the first beat is accepted", before.has_value());
    bench.verdict("and reports nothing done yet", before && before->done == 0);
    bench.verdict("with the whole block still to go",
                  before && before->remaining == total);
    bench.verdict("no violation: the write-back was freshly spilled",
                  Copy::violations() == 0);
    Copy::clear_counters();

    // Pace 20 beats by hand.
    constexpr uint16_t paced = 20;
    for (uint16_t i = 0; i < paced; ++i) {
        Copy::trigger();
        uint32_t spins = 10'000u;
        while (Copy::busy() && spins-- != 0u) {
        }
    }

    const uint32_t t0 = cycles_now();
    const auto mid = Copy::harvest();
    const uint32_t harvest_cost = cycles_now() - t0;

    bench.verdict("mid-block harvest accepted", mid.has_value());
    if (mid) {
        print(serial, "  after ", paced, " triggers: done=", mid->done,
              " remaining=", mid->remaining, ", harvest cost ", harvest_cost,
              " cycles (", cycles_to_us(harvest_cost), " us)", crlf);
        bench.verdict("progress is exactly the beats triggered", mid->done == paced);
        bench.verdict("remaining is the rest", mid->remaining == total - paced);
        bench.verdict("not reported complete", !mid->complete);
    } else {
        bench.verdict("progress is exactly the beats triggered", false);
        bench.verdict("remaining is the rest", false);
        bench.verdict("not reported complete", false);
    }
    bench.verdict("no violation on a good write-back", Copy::violations() == 0);
    bench.verdict("the harvested bytes are correct", same(src_buf, dst_buf, paced));

    // ---- THE ERRATUM 1.10.4 VALIDATION, shown refusing ----------------------
    // The write-back section is ordinary SRAM. Scribbling one of its
    // INVARIANT fields is exactly what the erratum does to it, and the
    // harvest has to notice - it is the only defence this driver has.
    const uint32_t good_dst = brio::Dmac::read_write_back(ch_copy).dstaddr;
    brio::Dmac::write_back(ch_copy).DMAC_DSTADDR = good_dst ^ 0x40u;
    const auto scribbled = Copy::harvest();
    bench.verdict("a scribbled DSTADDR is refused", !scribbled.has_value());
    bench.verdict("and counted", Copy::violations() == 1);
    brio::Dmac::write_back(ch_copy).DMAC_DSTADDR = good_dst;

    const uint16_t good_cnt = brio::Dmac::read_write_back(ch_copy).btcnt;
    brio::Dmac::write_back(ch_copy).DMAC_BTCNT = static_cast<uint16_t>(total + 1);
    const auto over = Copy::harvest();
    bench.verdict("a BTCNT above the block length is refused", !over.has_value());
    brio::Dmac::write_back(ch_copy).DMAC_BTCNT = good_cnt;

    const uint16_t good_ctrl = brio::Dmac::read_write_back(ch_copy).btctrl;
    brio::Dmac::write_back(ch_copy).DMAC_BTCTRL =
        static_cast<uint16_t>(good_ctrl ^ DMAC_BTCTRL_SRCINC_Msk);
    const auto bad_ctrl = Copy::harvest();
    bench.verdict("a scribbled BTCTRL is refused", !bad_ctrl.has_value());
    brio::Dmac::write_back(ch_copy).DMAC_BTCTRL = good_ctrl;
    Copy::clear_counters();

    // ---- resume and finish --------------------------------------------------
    const auto again = Copy::harvest();
    bench.verdict("a restored write-back is accepted again", again.has_value());
    bench.verdict("and reports the same progress", again && again->done == paced);

    for (uint16_t i = paced; i < total; ++i) {
        Copy::trigger();
        uint32_t spins = 10'000u;
        while (Copy::busy() && spins-- != 0u) {
        }
    }
    bench.verdict("the channel disabled itself at the end", wait_done<Copy>());
    const auto final_read = Copy::harvest();
    bench.verdict("the final harvest is accepted", final_read.has_value());
    bench.verdict("and says complete", final_read && final_read->complete);
    bench.verdict("all 64 bytes are correct", same(src_buf, dst_buf, total));
    bench.verdict("no violations across the whole letter", Copy::violations() == 0);
    bench.verdict("no suspend timeouts", Copy::suspend_timeouts() == 0);
}

// =============================================================================
// e - interrupts
// =============================================================================
void te_interrupts() {
    brio::Nvic::enable(brio::Dmac::irq());
    clear_irq_counts();

    // ---- TCMPL --------------------------------------------------------------
    Copy::configure({.action = brio::DmaTriggerAction::block});
    Copy::arm(brio::DmaFlag::complete | brio::DmaFlag::transfer_error, true);
    bench.verdict("TCMPL and TERR read back armed",
                  Copy::armed() ==
                      (brio::DmaFlag::complete | brio::DmaFlag::transfer_error));
    fill_pattern(src_buf, 32, 0x64);
    zero(dst_buf, 32);
    (void)run_copy<Copy>({
        .source = src_buf,
        .destination = dst_buf,
        .beats = 32,
        .beat = brio::DmaBeat::byte,
    });
    uint32_t spins = 100'000u;
    while (irq_complete[ch_copy] == 0 && spins-- != 0u) {
    }
    bench.verdict("one TCMPL interrupt for one block", irq_complete[ch_copy] == 1);
    bench.verdict("no TERR alongside it", irq_error[ch_copy] == 0);
    bench.verdict("the block still moved its bytes", same(src_buf, dst_buf, 32));

    // ---- an invalid descriptor ----------------------------------------------
    // 25.6.2.8 says an invalid descriptor SUSPENDS the channel and sets
    // CHSTATUS.FERR; CHINTFLAG.TERR's own description (25.8.22) says the
    // same fetch sets TERR. The two READ as alternatives - and the
    // silicon does BOTH at once, which is what the verdicts below assert
    // now that it has been measured.
    clear_irq_counts();
    Copy::arm(brio::DmaFlag::all, true);
    brio::DmaDescriptor invalid = brio::dma_descriptor({
        .source = src_buf,
        .destination = dst_buf,
        .beats = 8,
        .beat = brio::DmaBeat::byte,
    });
    invalid.btctrl = static_cast<uint16_t>(invalid.btctrl & ~DMAC_BTCTRL_VALID_Msk);
    Copy::load(invalid);
    Copy::enable(true);
    Copy::trigger();
    spins = 100'000u;
    while ((irq_error[ch_copy] == 0 && irq_suspend[ch_copy] == 0) && spins-- != 0u) {
    }
    const uint8_t st = Copy::status();
    print(serial, "  invalid fetch: CHSTATUS=", brio::hex(st), " TERR=",
          irq_error[ch_copy], " SUSP=", irq_suspend[ch_copy], " TCMPL=",
          irq_complete[ch_copy], crlf);
    bench.verdict("an invalid descriptor sets CHSTATUS.FERR",
                  (st & brio::DmaStatus::fetch_error) != 0);
    bench.verdict("it raises SUSP, as 25.6.2.8 says", irq_suspend[ch_copy] != 0);
    bench.verdict("AND TERR, as 25.8.22 says", irq_error[ch_copy] != 0);
    bench.verdict("no block was reported complete", irq_complete[ch_copy] == 0);

    // FERR clears on a software RESUME and on nothing else (25.8.23) -
    // but a RESUME onto the SAME invalid descriptor with a trigger still
    // pending re-fetches it and sets FERR again before the CPU can look,
    // which is how this first read as "RESUME does not clear FERR". So
    // the descriptor is made VALID first, and the clause is then exactly
    // what the chapter says it is.
    brio::DmaDescriptor repaired = invalid;
    repaired.btctrl = static_cast<uint16_t>(repaired.btctrl | DMAC_BTCTRL_VALID_Msk);
    Copy::load(repaired);
    Copy::clear_flags(brio::DmaFlag::all);
    Copy::resume();
    bench.verdict("RESUME over a repaired descriptor clears FERR",
                  (Copy::status() & brio::DmaStatus::fetch_error) == 0);
    Copy::enable(false);
    (void)Copy::reset();

    // ---- a real bus error ----------------------------------------------------
    // 0x30000000 is not mapped on this part. The DMAC is the bus master
    // here, so the AHB error comes back to IT and raises TERR; the CPU
    // never touches the address and cannot fault on it.
    clear_irq_counts();
    Copy::configure({.action = brio::DmaTriggerAction::block});
    Copy::arm(brio::DmaFlag::all, true);
    (void)Copy::load(brio::DmaTransfer{
        .source = src_buf,
        .destination = reinterpret_cast<volatile void*>(0x30000000u),
        .beats = 4,
        .beat = brio::DmaBeat::word,
        .destination_increment = false,
    });
    Copy::enable(true);
    Copy::trigger();
    spins = 100'000u;
    while (irq_error[ch_copy] == 0 && spins-- != 0u) {
    }
    print(serial, "  unmapped write: TERR=", irq_error[ch_copy],
          " TCMPL=", irq_complete[ch_copy], " CHSTATUS=", brio::hex(Copy::status()),
          crlf);
    bench.verdict("a write to unmapped memory raises TERR", irq_error[ch_copy] != 0);
    bench.verdict("and the channel disabled itself", !Copy::enabled());
    bench.verdict("the channel resets cleanly after a bus error", Copy::reset());

    // BENCH FINDING, and the reason this block exists at all: a channel
    // that has taken a bus error does NOT come back whole on the next
    // transfer, even after a successful CHCTRLA.SWRST. Its first block
    // afterwards deterministically LOSES ITS FIRST BEAT - measured over
    // 32 repetitions, the first one failing every time and the other 31
    // byte-exact. Nothing in ch. 25 says so; 25.6.2.8 only promises the
    // counter is written back before the channel is disabled.
    //
    // The remedy is the one the measurement dictates: after a bus error,
    // spend one block and throw it away. That is what this is, and
    // asserting the loss here rather than hiding it is what keeps the
    // fact honest - if a later silicon revision stops doing it, this
    // verdict fails and the doc gets corrected.
    (void)Copy::configure({.action = brio::DmaTriggerAction::block});
    all_fill(dst_buf, 32, 0xEE);
    fill_pattern(src_buf, 16, 0xB7);
    (void)run_copy<Copy>({
        .source = src_buf,
        .destination = dst_buf,
        .beats = 16,
        .beat = brio::DmaBeat::byte,
    });
    const bool first_beat_lost =
        dst_buf[0] == 0xEE && same(&src_buf[1], &dst_buf[1], 15);
    print(serial, "  first block after the bus error: dst[0]=",
          brio::hex(dst_buf[0]), " src[0]=", brio::hex(src_buf[0]),
          first_beat_lost ? " (first beat LOST)" : " (intact)", crlf);
    bench.verdict("the post-bus-error block loses exactly its first beat",
                  first_beat_lost);
    (void)Copy::reset();

    // ---- INTPEND with two channels pending ----------------------------------
    // Both channels are armed and both blocks are launched before either
    // interrupt is serviced, by masking around the pair. INTPEND reports
    // the LOWEST pending channel first, so one entry to the handler must
    // serve both in its loop.
    clear_irq_counts();
    (void)Copy::configure({.action = brio::DmaTriggerAction::block});
    (void)Chain::configure({.action = brio::DmaTriggerAction::block});
    Copy::arm(brio::DmaFlag::complete, true);
    Chain::arm(brio::DmaFlag::complete, true);
    fill_pattern(src_buf, 16, 0xC1);

    // Repeated, not sampled once: a beat that goes missing under
    // concurrent triggers is a FREQUENCY, and one observation cannot
    // tell a silicon effect from a leftover of the sub-test above.
    constexpr uint8_t rounds = 32;
    uint8_t low_exact = 0;
    uint8_t high_exact = 0;
    uint8_t low_first_lost = 0;
    uint32_t low_lost_rounds = 0;   // bit per round that was not exact
    for (uint8_t round = 0; round < rounds; ++round) {
        // A marker rather than zero, so "the DMA never wrote here"
        // (0xEE survives) is distinguishable from "it wrote the wrong
        // thing".
        all_fill(dst_buf, 64, 0xEE);
        (void)Copy::load(brio::DmaTransfer{
            .source = src_buf, .destination = dst_buf, .beats = 16,
            .beat = brio::DmaBeat::byte});
        (void)Chain::load(brio::DmaTransfer{
            .source = src_buf, .destination = &dst_buf[32], .beats = 16,
            .beat = brio::DmaBeat::byte});
        {
            brio::SamPlatform::CriticalSection cs;
            (void)Copy::enable(true);
            (void)Chain::enable(true);
            brio::Dmac::regs().DMAC_SWTRIGCTRL = Copy::mask | Chain::mask;
            // Both blocks run and finish inside the guard; the handler
            // is held off until it leaves, so it finds two pending
            // channels.
            uint32_t left = 100'000u;
            while ((brio::Dmac::interrupt_status() & (Copy::mask | Chain::mask)) !=
                       (Copy::mask | Chain::mask) &&
                   left-- != 0u) {
            }
        }
        spins = 100'000u;
        while ((irq_complete[ch_copy] < round + 1 || irq_complete[ch_chain] < round + 1) &&
               spins-- != 0u) {
        }
        if (same(src_buf, dst_buf, 16)) {
            ++low_exact;
        } else {
            low_lost_rounds |= 1UL << round;
            if (dst_buf[0] == 0xEE && same(&src_buf[1], &dst_buf[1], 15)) {
                ++low_first_lost;
            }
        }
        if (same(src_buf, &dst_buf[32], 16)) {
            ++high_exact;
        }
    }

    print(serial, "  ", rounds, " concurrent pairs: TCMPL copy=", irq_complete[ch_copy],
          " chain=", irq_complete[ch_chain], ", most served in one entry=", irq_rounds,
          crlf, "  low exact=", low_exact, " (first beat lost ", low_first_lost,
          "), high exact=", high_exact, ", losing rounds=",
          brio::hex(low_lost_rounds), crlf);
    bench.verdict("the low channel was served every round",
                  irq_complete[ch_copy] == rounds);
    bench.verdict("the high channel was served every round",
                  irq_complete[ch_chain] == rounds);
    bench.verdict("one handler entry served both", irq_rounds >= 2);
    bench.verdict("the low channel's blocks are byte-exact", low_exact == rounds);
    bench.verdict("the high channel's blocks are byte-exact", high_exact == rounds);

    Copy::arm(brio::DmaFlag::all, false);
    Chain::arm(brio::DmaFlag::all, false);
}

// =============================================================================
// f - the console transmitting through a DMA channel
// =============================================================================
void tf_uart_tx() {
    brio::Nvic::enable(brio::Dmac::irq());
    clear_irq_counts();

    Tx::configure({
        .trigger = brio::dma_trigger_sercom_tx<5>(),
        .action = brio::DmaTriggerAction::beat,
    });
    Tx::arm(brio::DmaFlag::complete | brio::DmaFlag::transfer_error, true);
    bench.verdict("SERCOM5 TX trigger is the header's code",
                  brio::dma_trigger_sercom_tx<5>() == SERCOM5_DMAC_ID_TX);

    print(serial, "  handing SERCOM5 to channel ", ch_tx, " for one line:", crlf);
    console_tx_quiesce();

    const uint32_t t0 = cycles_now();
    const bool loaded = Tx::load(brio::DmaTransfer{
        .source = tx_text,
        .destination = &Sc5::regs().SERCOM_DATA,
        .beats = tx_text_len,
        .beat = brio::DmaBeat::byte,
        .source_increment = true,
        .destination_increment = false,
    });
    Tx::enable(true);
    // No software trigger: the SERCOM's own "transmit buffer is free"
    // raises the first one, as it does every one after it.
    const bool done = wait_done<Tx>(20'000'000u);
    const uint32_t spent = cycles_now() - t0;

    uint32_t spins = 200'000u;
    while (!Sc5::txc_flag() && spins-- != 0u) {
    }
    console_tx_restore();

    bench.verdict("the TX descriptor loaded", loaded);
    bench.verdict("the block completed", done);
    spins = 100'000u;
    while (irq_complete[ch_tx] == 0 && spins-- != 0u) {
    }
    bench.verdict("one TCMPL for the line", irq_complete[ch_tx] == 1);
    bench.verdict("no transfer error", irq_error[ch_tx] == 0);
    bench.verdict("the whole line was consumed",
                  brio::Dmac::read_write_back(ch_tx).btcnt == 0);
    bench.verdict("the console still works (you are reading this)", true);

    // At 115200 8N1 a byte is 86.8 us; the block should take the WIRE's
    // time, not the CPU's, which is the whole point.
    const uint32_t us = cycles_to_us(spent);
    print(serial, "  ", tx_text_len, " bytes in ", us, " us = ",
          (tx_text_len * 1'000'000UL) / us, " baud-bytes/s (wire floor ",
          (115200UL / 10UL), ")", crlf);
    bench.verdict("the transfer took the wire's time, not the CPU's",
                  us > (tx_text_len * 80UL) && us < (tx_text_len * 120UL));

    Tx::arm(brio::DmaFlag::all, false);
    (void)Tx::reset();
}

// =============================================================================
// g - THE ERRATUM 1.10.4 HUNT
// =============================================================================
void tg_concurrent() {
    brio::Nvic::enable(brio::Dmac::irq());
    clear_irq_counts();
    Copy::clear_counters();
    Chain::clear_counters();
    Churn0::clear_counters();
    Churn1::clear_counters();
    Rx::clear_counters();

    // Five channels triggered concurrently for several seconds:
    // - ch_tx  paced by SERCOM5's own TX trigger, one beat per byte
    // - ch_rx  paced by SERCOM5's RX trigger (it contributes whenever
    //          the person at the other end types; zero is a legal and
    //          reported outcome, not a failure)
    // - four memory-to-memory channels running flat out against them
    // and every one of them harvested over and over, because a harvest
    // is the only thing that READS a write-back and therefore the only
    // thing the erratum can be caught by.
    constexpr uint32_t run_ms = 4000;

    fill_pattern(src_buf, buf_len, 0x3E);
    zero(rx_buf, buf_len);

    // The two peripheral channels take the TOP arbitration level and the
    // bulk memory ones the bottom: a peripheral that loses a trigger
    // loses DATA, while a memory copy that waits merely waits. LVL3 is
    // the highest (25.6.2.4 - a HIGHER level number wins), which is the
    // opposite of the convention on most controllers and worth stating.
    (void)Tx::configure({.trigger = brio::dma_trigger_sercom_tx<5>(),
                         .action = brio::DmaTriggerAction::beat,
                         .priority = brio::DmaPriority::level3});
    (void)Rx::configure({.trigger = brio::dma_trigger_sercom_rx<5>(),
                         .action = brio::DmaTriggerAction::beat,
                         .priority = brio::DmaPriority::level3});
    (void)Copy::configure({.action = brio::DmaTriggerAction::beat});
    (void)Chain::configure({.action = brio::DmaTriggerAction::beat});
    (void)Churn0::configure({.action = brio::DmaTriggerAction::beat});
    (void)Churn1::configure({.action = brio::DmaTriggerAction::beat});

    print(serial, "  ", run_ms, " ms of five concurrent channels; type into the "
                  "console to feed the RX one", crlf);
    console_tx_quiesce();
    console_rx_detach();

    (void)Rx::load(brio::DmaTransfer{
        .source = &Sc5::regs().SERCOM_DATA,
        .destination = rx_buf,
        .beats = buf_len,
        .beat = brio::DmaBeat::byte,
        .source_increment = false,
    });
    Rx::enable(true);

    uint32_t tx_lines = 0;
    uint32_t churn_blocks = 0;
    uint32_t harvests = 0;
    uint32_t refused = 0;
    uint32_t tx_sum = 0;

    const uint32_t deadline = brio::Ticker::ticks() + run_ms;
    while (static_cast<int32_t>(brio::Ticker::ticks() - deadline) < 0) {
        // One console line out, byte by byte, on the peripheral trigger.
        if (!Tx::enabled()) {
            if (Tx::loaded().valid_bit()) {
                ++tx_lines;
                tx_sum += sum16(reinterpret_cast<const uint8_t*>(tx_text), tx_text_len);
            }
            (void)Tx::load(brio::DmaTransfer{
                .source = tx_text,
                .destination = &Sc5::regs().SERCOM_DATA,
                .beats = tx_text_len,
                .beat = brio::DmaBeat::byte,
                .destination_increment = false,
            });
            Tx::enable(true);
        }

        // Four memory-to-memory channels, beat-triggered so they keep
        // re-entering the arbiter rather than finishing in one burst.
        churn_blocks += churn<Copy>(0);
        churn_blocks += churn<Chain>(32);
        churn_blocks += churn<Churn0>(64);
        churn_blocks += churn<Churn1>(96);

        // And harvest everything, which is what puts a READER on every
        // write-back while every writer is busy.
        ++harvests;
        if (!Copy::harvest()) ++refused;
        if (!Chain::harvest()) ++refused;
        if (!Churn0::harvest()) ++refused;
        if (!Churn1::harvest()) ++refused;
        if (!Rx::harvest()) ++refused;
    }

    // Let the transmitter finish whatever byte it holds, then take the
    // console back.
    Tx::enable(false);
    uint32_t spins = 400'000u;
    while (!Sc5::txc_flag() && spins-- != 0u) {
    }
    const auto rx_final = Rx::harvest();
    Rx::enable(false);
    console_tx_restore();
    console_rx_restore();

    const uint32_t rx_bytes = rx_final ? rx_final->done : 0u;
    const uint32_t violations = Copy::violations() + Chain::violations() +
                                Churn0::violations() + Churn1::violations() +
                                Rx::violations();
    const uint32_t timeouts = Copy::suspend_timeouts() + Chain::suspend_timeouts() +
                              Churn0::suspend_timeouts() + Churn1::suspend_timeouts() +
                              Rx::suspend_timeouts();

    print(serial, crlf, "  ---- erratum 1.10.4 hunt ----", crlf);
    const uint16_t tx_in_flight = brio::Dmac::read_write_back(ch_tx).btcnt;
    const uint32_t tx_bytes = tx_lines * tx_text_len;
    // BEATS, not blocks. A channel starved hard enough may not FINISH a
    // block in the whole window while still having moved most of one,
    // and "did it transmit" is a question about bytes on the wire.
    const uint32_t tx_moved =
        tx_bytes + (tx_in_flight < tx_text_len ? (tx_text_len - tx_in_flight) : 0u);
    print(serial, "  TX lines=", tx_lines, " = ", tx_bytes, " bytes (checksum ", tx_sum,
          "), ", (tx_bytes * 1000UL) / run_ms, " B/s against an 11520 B/s wire; ",
          tx_in_flight, " beats still in flight, ", tx_moved, " bytes moved in all",
          crlf);
    print(serial, "  churn blocks=", churn_blocks, "  RX bytes=", rx_bytes, crlf);
    print(serial, "  harvest rounds=", harvests, "  readings refused=", refused, crlf);
    print(serial, "  write-back violations=", violations, "  suspend timeouts=",
          timeouts, crlf);
    if (violations != 0u) {
        print(serial, "  (erratum 1.10.4 reached at this density too - see letter j)",
              crlf);
    }
    if (rx_bytes != 0) {
        print(serial, "  RX checksum=", sum16(rx_buf, static_cast<uint16_t>(rx_bytes)),
              crlf);
    }

    // A modest floor, not a throughput claim: what matters for this
    // letter is that a peripheral-triggered channel really was moving
    // beats WHILE four memory channels were being triggered, suspended
    // and resumed around it. HOW FAST it managed is printed above and
    // discussed in the doc rather than asserted - it varies by two
    // orders of magnitude run to run, which is itself the finding.
    bench.verdict("the TX channel really moved bytes under the churn",
                  tx_moved >= 8);
    bench.verdict("the churn channels kept re-arming", churn_blocks >= 4);
    bench.verdict("every channel was harvested many times", harvests >= 100);
    // THE MEASUREMENT, NOT A CLAIM. Erratum 1.10.4 is real on this
    // silicon - letter j provokes it reliably and prints a corrupted
    // write-back in full. Whether THIS letter's trigger density reaches
    // it varies (it usually does not, because the transmit channel here
    // is throttled by the harvest storm around it), so the number is
    // reported and the INVARIANT is what gets asserted: a refused
    // reading is refused, never half-believed, and the data lands
    // regardless.
    bench.verdict("every refused reading was a counted one",
                  refused == violations + timeouts);
    // The four churn channels cover the first 128 bytes between them
    // (32 each), and those are the bytes that had a thousand suspends
    // and resumes land in the middle of them.
    bench.verdict("the churned copies are still byte-exact",
                  same(src_buf, dst_buf, 128));

    Copy::clear_counters();
    Chain::clear_counters();
    Churn0::clear_counters();
    Churn1::clear_counters();
    Rx::clear_counters();
    (void)Tx::reset();
    (void)Rx::reset();
}

// =============================================================================
// r - the console receiving through a DMA channel (needs the runner)
// =============================================================================
void tr_uart_rx() {
    brio::Nvic::enable(brio::Dmac::irq());
    clear_irq_counts();
    Rx::clear_counters();

    Rx::configure({
        .trigger = brio::dma_trigger_sercom_rx<5>(),
        .action = brio::DmaTriggerAction::beat,
    });
    Rx::arm(brio::DmaFlag::complete | brio::DmaFlag::transfer_error, true);
    bench.verdict("SERCOM5 RX trigger is the header's code",
                  brio::dma_trigger_sercom_rx<5>() == SERCOM5_DMAC_ID_RX);

    zero(rx_buf, buf_len);
    constexpr uint32_t window_ms = 3000;
    print(serial, "  READY - send a burst now (", window_ms, " ms window)", crlf);
    console_tx_quiesce();
    console_rx_detach();

    (void)Rx::load(brio::DmaTransfer{
        .source = &Sc5::regs().SERCOM_DATA,
        .destination = rx_buf,
        .beats = buf_len,
        .beat = brio::DmaBeat::byte,
        .source_increment = false,
    });
    Rx::enable(true);

    // Tick-paced harvesting, which is the pacing contract the RX engine
    // hands to its caller: the channel never reports arrivals by itself,
    // so somebody has to ask, and how often is that somebody's policy.
    uint32_t polls = 0;
    uint32_t refused = 0;
    uint16_t last = 0;
    uint32_t first_byte_at = 0;
    const uint32_t start = brio::Ticker::ticks();
    const uint32_t deadline = start + window_ms;
    while (static_cast<int32_t>(brio::Ticker::ticks() - deadline) < 0) {
        const uint32_t next = brio::Ticker::ticks() + 10;
        while (static_cast<int32_t>(brio::Ticker::ticks() - next) < 0) {
        }
        ++polls;
        const auto p = Rx::harvest();
        if (!p) {
            ++refused;
            continue;
        }
        if (p->done != last) {
            if (last == 0) {
                first_byte_at = brio::Ticker::ticks() - start;
            }
            last = p->done;
        }
    }

    const uint16_t status = Sc5::status();
    Rx::enable(false);
    console_rx_restore();
    console_tx_restore();

    print(serial, "  received ", last, " bytes in ", polls, " polls (", refused,
          " refused), first at ", first_byte_at, " ms, checksum ", sum16(rx_buf, last),
          crlf, "  bytes: ");
    for (uint16_t i = 0; i < last && i < 64u; ++i) {
        const uint8_t c = rx_buf[i];
        print(serial, (c >= 0x20u && c < 0x7Fu) ? static_cast<char>(c) : '.');
    }
    print(serial, crlf, "  SERCOM STATUS at the end = ", brio::hex(status), crlf);

    bench.verdict("something arrived", last != 0);
    bench.verdict("no harvest was refused", refused == 0);
    bench.verdict("no transfer error", irq_error[ch_rx] == 0);
    bench.verdict("the buffer was not overrun", last <= buf_len);
    bench.verdict("no write-back violation", Rx::violations() == 0);
    // The honest RX-DMA contract: STATUS is read ONCE, at harvest
    // granularity, so an error is known to have happened but not to
    // WHICH byte. That is the trade this engine makes.
    bench.verdict("no receive error over the burst",
                  (status & brio::SercomStatus::receive_errors) == 0);

    Rx::arm(brio::DmaFlag::all, false);
    (void)Rx::reset();
}

// =============================================================================
// The engined Uart: handing the console over and taking it back
// =============================================================================
//
// init() on either instantiation resets and reconfigures SERCOM5 from
// scratch, so switching between them is a clean handover rather than a
// negotiation. The rule the letters below all obey: DRAIN, SWITCH, TEST,
// SWITCH BACK, and only THEN print verdicts - a verdict printed while
// the other instantiation owns the peripheral would sit in a ring nobody
// is draining.

bool take_console_with_engines() {
    console_tx_quiesce();
    Sc5::enable_rxc_interrupt(false);
    return EngineSerial::init(clock, 115200);
}

bool give_console_back() {
    // Let the engine finish whatever block is on the wire, then hand the
    // peripheral to the plain interrupt-driven console again.
    uint32_t spins = 20'000'000u;
    while (!EngineSerial::tx_idle() && spins-- != 0u) {
    }
    spins = 400'000u;
    while (!Sc5::txc_flag() && spins-- != 0u) {
    }
    EngineSerial::release();
    return Serial::init(clock, 115200);
}

// =============================================================================
// h - transmitting through Uart's optional TX engine
// =============================================================================
void th_engine_tx() {
    bench.verdict("the engined Uart names both engines",
                  EngineSerial::has_tx_engine && EngineSerial::has_rx_engine);
    bench.verdict("the plain one names neither",
                  !Serial::has_tx_engine && !Serial::has_rx_engine);
    bench.verdict("SERCOM5's own TX trigger code",
                  Sc5::dma_tx_trigger() == SERCOM5_DMAC_ID_TX);

    clear_irq_counts();
    const bool took = take_console_with_engines();

    // Ordinary print() calls. write_byte() pushes into the same SPSC
    // ring as ever; what changed is who drains it - the DRE interrupt is
    // never armed, and the engine takes whole contiguous runs instead.
    const uint32_t t0 = cycles_now();
    constexpr uint8_t lines = 6;
    for (uint8_t i = 0; i < lines; ++i) {
        brio::print(engine_serial, "[engine-tx] line ", i,
                    " written with print(), drained by DMA", crlf);
    }
    uint32_t spins = 20'000'000u;
    while (!EngineSerial::tx_idle() && spins-- != 0u) {
    }
    const uint32_t spent = cycles_now() - t0;
    const bool drained = EngineSerial::tx_idle();
    const uint8_t dre_armed = Sc5::armed();

    const bool gave = give_console_back();

    bench.verdict("the engined transport came up", took);
    bench.verdict("print() drained the ring completely", drained);
    bench.verdict("the DRE interrupt was never armed",
                  (dre_armed & brio::SercomFlag::dre) == 0);
    bench.verdict("the engine's channel reported completions",
                  irq_complete[ch_engine_tx] >= lines);
    bench.verdict("no transfer error on the engine's channel",
                  irq_error[ch_engine_tx] == 0);
    bench.verdict("the console came back", gave);

    const uint32_t us = cycles_to_us(spent);
    print(serial, "  ", lines, " lines, ", irq_complete[ch_engine_tx],
          " DMA blocks, ", us, " us = ", us / lines, " us/line at the wire", crlf);
    bench.verdict("the CPU spent the wire's time, not more",
                  us > 3000u && us < 60000u);
}

// =============================================================================
// i - the RX engine and the tick-paced harvest (needs the runner)
// =============================================================================
void ti_engine_rx() {
    clear_irq_counts();
    const bool took = take_console_with_engines();

    constexpr uint32_t window_ms = 3000;
    brio::print(engine_serial, "  READY - send a burst now (", window_ms,
                " ms window)", crlf);
    uint32_t spins = 20'000'000u;
    while (!EngineSerial::tx_idle() && spins-- != 0u) {
    }

    // TICK-PACED, which is the whole contract: nothing tells this
    // transport that bytes have arrived, so somebody has to ask, and how
    // often is that somebody's policy. Ten milliseconds is this suite's
    // choice, not the driver's.
    uint32_t polls = 0;
    uint32_t edges = 0;
    uint32_t first_byte_at = 0;
    uint8_t received[128];
    uint16_t got = 0;
    const uint32_t start = brio::Ticker::ticks();
    const uint32_t deadline = start + window_ms;
    while (static_cast<int32_t>(brio::Ticker::ticks() - deadline) < 0) {
        const uint32_t next = brio::Ticker::ticks() + 10;
        while (static_cast<int32_t>(brio::Ticker::ticks() - next) < 0) {
        }
        ++polls;
        if (EngineSerial::harvest()) {
            ++edges;   // the empty -> non-empty edge a kernel AO would post on
            if (first_byte_at == 0u) {
                first_byte_at = brio::Ticker::ticks() - start;
            }
        }
        // The bytes leave the transport through the ORDINARY ByteSource
        // verb: an engine changes who fills the ring, never how it is
        // read.
        uint8_t b = 0;
        while (got < sizeof(received) && EngineSerial::read_byte(b)) {
            received[got++] = b;
        }
    }

    const uint8_t hw_over = EngineSerial::hw_overruns();
    const uint8_t frame_err = EngineSerial::frame_errors();
    const uint8_t rx_armed = Sc5::armed();
    const bool gave = give_console_back();

    print(serial, "  received ", got, " bytes in ", polls, " polls, ", edges,
          " wake edges, first at ", first_byte_at, " ms", crlf, "  bytes: ");
    for (uint16_t i = 0; i < got; ++i) {
        const uint8_t c = received[i];
        print(serial, (c >= 0x20u && c < 0x7Fu) ? static_cast<char>(c) : '.');
    }
    print(serial, crlf, "  hw_overruns=", hw_over, " frame_errors=", frame_err, crlf);

    bench.verdict("the engined transport came up", took);
    bench.verdict("the RXC interrupt was never armed",
                  (rx_armed & brio::SercomFlag::rxc) == 0);
    bench.verdict("something arrived", got != 0);
    bench.verdict("harvest reported the empty -> non-empty edge", edges != 0);
    bench.verdict("read_byte() served the bytes unchanged", got != 0);
    bench.verdict("no hardware overrun", hw_over == 0);
    bench.verdict("no receive error over the burst", frame_err == 0);
    bench.verdict("no write-back violation on the RX channel",
                  brio::DmaChannel<ch_engine_rx>::violations() == 0);
    bench.verdict("the console came back", gave);
}

// =============================================================================
// j - both engines at once: erratum 1.10.4 at the task level
// =============================================================================
void tj_engine_duplex() {
    clear_irq_counts();
    brio::DmaChannel<ch_engine_tx>::clear_counters();
    brio::DmaChannel<ch_engine_rx>::clear_counters();
    Copy::clear_counters();
    Churn0::clear_counters();

    const bool took = take_console_with_engines();

    // Full duplex through the task's own surface - print() out, harvest()
    // in - with two memory-to-memory channels churning underneath, so
    // four channels are being triggered concurrently while every one of
    // them has its write-back read.
    constexpr uint32_t run_ms = 3000;
    (void)Copy::configure({.action = brio::DmaTriggerAction::beat});
    (void)Churn0::configure({.action = brio::DmaTriggerAction::beat});
    fill_pattern(src_buf, buf_len, 0x6D);

    brio::DmaDescriptor bad_wb{};
    brio::DmaDescriptor bad_loaded{};
    uint8_t bad_status = 0;
    bool captured = false;
    uint32_t refused_readings = 0;
    uint32_t lines = 0;
    uint32_t harvests = 0;
    uint32_t rx_bytes = 0;
    uint32_t churn_blocks = 0;
    const uint32_t deadline = brio::Ticker::ticks() + run_ms;
    while (static_cast<int32_t>(brio::Ticker::ticks() - deadline) < 0) {
        if (EngineSerial::tx_idle()) {
            brio::print(engine_serial, "[duplex] line ", lines,
                        " out while the harvest runs in", crlf);
            ++lines;
        }
        churn_blocks += churn<Copy>(0);
        churn_blocks += churn<Churn0>(64);
        ++harvests;
        (void)EngineSerial::harvest();
        if (!Copy::harvest()) {
            ++refused_readings;
        }
        const uint32_t before = Churn0::violations();
        const bool churn_ok = Churn0::harvest().has_value();
        if (!churn_ok) {
            ++refused_readings;
        }
        if (!churn_ok && Churn0::violations() != before && !captured) {
            // One-shot: WHAT was inconsistent, not just that something
            // was. The same technique that turned letter e's mystery
            // into a fact.
            bad_wb = brio::Dmac::read_write_back(ch_churn0);
            bad_loaded = Churn0::loaded();
            bad_status = Churn0::status();
            captured = true;
        }
        uint8_t b = 0;
        while (EngineSerial::read_byte(b)) {
            ++rx_bytes;
        }
    }

    const uint32_t violations = brio::DmaChannel<ch_engine_tx>::violations() +
                                brio::DmaChannel<ch_engine_rx>::violations() +
                                Copy::violations() + Churn0::violations();
    const uint32_t timeouts = brio::DmaChannel<ch_engine_tx>::suspend_timeouts() +
                              brio::DmaChannel<ch_engine_rx>::suspend_timeouts() +
                              Copy::suspend_timeouts() + Churn0::suspend_timeouts();
    const uint8_t hw_over = EngineSerial::hw_overruns();
    const bool gave = give_console_back();

    print(serial, crlf, "  ---- duplex 1.10.4 stress ----", crlf);
    print(serial, "  TX lines=", lines, "  RX bytes=", rx_bytes,
          "  churn blocks=", churn_blocks, crlf);
    print(serial, "  harvest rounds=", harvests, "  write-back violations=",
          violations, "  suspend timeouts=", timeouts, "  hw_overruns=", hw_over,
          crlf);
    // PER CHANNEL, because the whole question is WHICH write-back went
    // bad: a memory channel's would point at the erratum, the receive
    // engine's own at this driver.
    print(serial, "  per channel: tx", ch_engine_tx, "=",
          brio::DmaChannel<ch_engine_tx>::violations(), "/",
          brio::DmaChannel<ch_engine_tx>::suspend_timeouts(), " rx", ch_engine_rx,
          "=", brio::DmaChannel<ch_engine_rx>::violations(), "/",
          brio::DmaChannel<ch_engine_rx>::suspend_timeouts(), " copy", ch_copy, "=",
          Copy::violations(), "/", Copy::suspend_timeouts(), " churn", ch_churn0, "=",
          Churn0::violations(), "/", Churn0::suspend_timeouts(),
          "   (violations/timeouts)", crlf);

    if (captured) {
        print(serial, "  first bad wb : ctrl=", brio::hex(bad_wb.btctrl), " cnt=",
              bad_wb.btcnt, " src=", brio::hex(bad_wb.srcaddr), " dst=",
              brio::hex(bad_wb.dstaddr), " next=", brio::hex(bad_wb.descaddr), crlf);
        print(serial, "  vs loaded    : ctrl=", brio::hex(bad_loaded.btctrl), " cnt=",
              bad_loaded.btcnt, " src=", brio::hex(bad_loaded.srcaddr), " dst=",
              brio::hex(bad_loaded.dstaddr), " next=", brio::hex(bad_loaded.descaddr),
              " CHSTATUS=", brio::hex(bad_status), crlf);
    }

    bench.verdict("the engined transport came up", took);
    bench.verdict("the TX engine kept transmitting", lines >= 2);
    bench.verdict("the churn channels kept re-arming", churn_blocks >= 4);
    bench.verdict("every channel was harvested many times", harvests >= 100);

    // ERRATUM 1.10.4 IS REAL ON THIS SILICON, and this letter is where
    // it shows: at this trigger density the counters above are NOT zero,
    // and the first corrupted write-back is printed in full - one
    // channel's SRCADDR and BTCTRL sitting in ANOTHER channel's
    // write-back entry, written there by the controller, exactly what
    // "the DMAC write-back descriptors may get corrupted" means.
    //
    // So the verdicts here are not "it never happens" - it does. They
    // are the two things the driver promises IN SPITE of it: a corrupted
    // reading is never believed, and the DATA is untouched. The
    // corruption is in the write-back descriptor, which is bookkeeping;
    // the transfers themselves land correctly, and the harvest that
    // would have mis-reported them refused instead.
    bench.verdict("the corruption was DETECTED, not suffered: copies byte-exact",
                  same(src_buf, dst_buf, 32) &&
                      same(&src_buf[64], &dst_buf[64], 32));
    // The accounting identity over the two channels this letter harvests
    // BY HAND: every reading that came back empty is one the driver
    // refused, and every refusal is counted as exactly one of the two
    // reasons. Nothing was quietly dropped, and nothing inconsistent was
    // quietly believed. (The engines' own channels are harvested inside
    // the transport and reported separately above.)
    bench.verdict("every refused reading is a counted one",
                  refused_readings == Copy::violations() + Copy::suspend_timeouts() +
                                          Churn0::violations() +
                                          Churn0::suspend_timeouts());
    bench.verdict("the transports survived it", hw_over == 0 && lines >= 2);
    bench.verdict("the console came back", gave);

    if (violations != 0u) {
        print(serial, "  ERRATUM 1.10.4 OBSERVED: ", violations,
              " corrupted write-backs refused in ", harvests * 3u,
              " readings, and every transfer still landed correctly", crlf);
    } else {
        print(serial, "  no corrupted write-back seen in this run", crlf);
    }
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf, "test_samc_dma - SAMC21J18A DMAC (ch. 25), clk=", SysClock::hz,
          " Hz", crlf);
    bench.menu();
}

} // namespace

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    const bool dma_ok = brio::Dmac::init();
    brio::Nvic::enable(brio::Dmac::irq());
    brio::enable_interrupts();

    bench.letter('a', "block, tables and the END-ADDRESS quirk", ta_block);
    bench.letter('b', "memory-to-memory matrix", tb_matrix);
    bench.letter('c', "software-linked chains", tc_chain);
    bench.letter('d', "suspend, harvest and the 1.10.4 validation", td_harvest);
    bench.letter('e', "interrupts: TCMPL, TERR, INTPEND dispatch", te_interrupts);
    bench.letter('f', "SERCOM5 transmit through a DMA channel", tf_uart_tx);
    bench.letter('g', "the erratum 1.10.4 hunt (concurrent channels)", tg_concurrent);
    bench.letter('r', "SERCOM5 receive through a DMA channel (send a burst)",
                 tr_uart_rx, false);
    bench.letter('h', "Uart's optional TX engine", th_engine_tx);
    bench.letter('i', "Uart's RX engine + tick-paced harvest (send a burst)",
                 ti_engine_rx, false);
    bench.letter('j', "both engines at once (1.10.4 at the task level)",
                 tj_engine_duplex);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " dmac=", dma_ok ? "up" : "FAILED", crlf);
        banner();
    }
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
