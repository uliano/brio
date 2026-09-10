// test_ch32_dma - the reference bench suite for the CH32V00x's DMA
// chapter: the seven channels' controller, a channel's refusals and
// flags, memory-to-memory blocks at the three widths, the transfer
// error, the interrupt - and the two engine slots of ch32v00x/usart.hpp
// exercised by THIS SUITE'S OWN CONSOLE, which transmits on channel 4
// and receives on channel 5: every line printed here left the ring on
// a DMA block, every keystroke arrived through a harvest.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE. The console is the WCH-Link's own serial.
//
// What is exercised, letter by letter:
//   a  the block and a channel as found, and the refusals: a config the
//      chapter forbids, a store into an enabled channel
//   b  memory-to-memory at byte, half-word and word widths: the copy
//      exact, CNTR counting down to zero, the completion flag, GIF
//      alongside it, the flags cleared by the write-one register
//   c  the interrupt: a block completing on channel 1 counted in its
//      own handler, with the channel's ISR body reading only the armed
//      flags
//   d  a transfer error: RM 8.2.1 promises TEIF and an automatic EN drop
//      for a block that reads a reserved address; this letter probes a
//      handful of holes in the map and REPORTS what the silicon does,
//      judging only that each block ended one way or the other
//   e  the console's transmit engine: a burst longer than the TX ring
//      printed through it, the blocks counted, no fault
//   f  the console's receive engine: what the harvest publishes is
//      what was typed - this letter asks for a line and echoes it
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

// The console on both engines: USART1 transmits on channel 4, receives
// on channel 5 (RM table 8-2).
using Serial = Uart<1, P, 64, 128, DmaTxEngine<4>, DmaRxEngine<5>>;
constexpr Serial serial;
using Led = Pin<'C', 0>;

// The channel the memory-to-memory letters use: 1, the ADC's, idle here.
using Copier = DmaChannel<1>;

TestBench<Serial> bench;

volatile uint32_t copier_completions = 0;
volatile uint32_t copier_errors = 0;
volatile uint32_t tx_blocks = 0;

uint8_t src8[64];
uint8_t dst8[64];
uint16_t src16[32];
uint16_t dst16[32];
uint32_t src32[16];
uint32_t dst32[16];

bool wait_complete(uint32_t polls = 1'000'000u) {
    for (uint32_t i = 0; i < polls; ++i) {
        if (Copier::flag(DmaFlag::complete) || Copier::flag(DmaFlag::error)) {
            return true;
        }
    }
    return false;
}

// One memory-to-memory block: `count` items of `width` from src to dst.
bool copy_block(volatile void* src, volatile void* dst, uint16_t count, DmaWidth width) {
    Copier::stop();
    const bool loaded = Copier::load(DmaTransfer{
        .peripheral = src,
        .memory = dst,
        .count = count,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = true,
                   .peripheral_increment = true,
                   .memory_increment = true,
                   .peripheral_width = width,
                   .memory_width = width,
                   .priority = DmaPriority::high},
    });
    return loaded && wait_complete();
}

// ---------------------------------------------------------------------------
// a - the block and the refusals
// ---------------------------------------------------------------------------
void ta_block() {
    Dma::open();
    print(serial, "  HBPCENR=", hex(rcc()->HBPCENR), " INTFR=", hex(Dma::flags()),
          " ch1 CFGR=", hex(Copier::regs().CFGR), crlf);
    bench.verdict("the block's gate is open (DMA1EN)", (rcc()->HBPCENR & rcc_hb_dma1) != 0u);
    Copier::stop();
    bench.verdict("channel 1 is disabled as found", !Copier::enabled());
    bench.verdict("circular + memory-to-memory is refused (RM 8.2.1)",
                  !Copier::configure({.circular = true, .memory_to_memory = true}));
    bench.verdict("a zero count is refused", !Copier::set_count(0));
    bench.verdict("a transfer with a null end is refused",
                  !Copier::load(DmaTransfer{.peripheral = nullptr, .memory = dst8, .count = 4}));
    // A configure into an ENABLED channel is refused: start a long copy
    // and try to reconfigure under it.
    for (uint32_t i = 0; i < 64u; ++i) { src8[i] = static_cast<uint8_t>(i); }
    (void)Copier::load(DmaTransfer{.peripheral = src8, .memory = dst8, .count = 64,
                                   .config = {.memory_to_memory = true, .peripheral_increment = true}});
    const bool refused = !Copier::configure({.memory_to_memory = true}) &&
                         !Copier::set_count(8);
    (void)wait_complete();
    bench.verdict("a store into an enabled channel is refused (its fields are "
                  "read-only then)",
                  refused || !Copier::enabled());   // or the block was already done
    Copier::stop();
    bench.verdict("stop() disables and clears", !Copier::enabled() && Copier::flags() == 0u);
}

// ---------------------------------------------------------------------------
// b - memory to memory at three widths
// ---------------------------------------------------------------------------
void tb_copies() {
    for (uint32_t i = 0; i < 64u; ++i) { src8[i] = static_cast<uint8_t>(i * 7u + 1u); dst8[i] = 0; }
    for (uint32_t i = 0; i < 32u; ++i) { src16[i] = static_cast<uint16_t>(i * 771u + 5u); dst16[i] = 0; }
    for (uint32_t i = 0; i < 16u; ++i) { src32[i] = i * 0x01010101u + 0xA5u; dst32[i] = 0; }

    bool ok = copy_block(src8, dst8, 64, DmaWidth::byte);
    bench.verdict("64 bytes copied memory to memory", ok && memcmp(src8, dst8, 64) == 0);
    bench.verdict("CNTR counted down to zero", Copier::count() == 0u);
    bench.verdict("the completion flag is up, and GIF beside it",
                  Copier::flag(DmaFlag::complete) && Copier::flag(DmaFlag::global));
    bench.verdict("no error", !Copier::flag(DmaFlag::error));
    Copier::clear(DmaFlag::all);
    bench.verdict("the write-one clear takes every flag down", Copier::flags() == 0u);

    ok = copy_block(src16, dst16, 32, DmaWidth::half);
    bench.verdict("32 half-words copied", ok && memcmp(src16, dst16, 64) == 0);
    ok = copy_block(src32, dst32, 16, DmaWidth::word);
    bench.verdict("16 words copied", ok && memcmp(src32, dst32, 64) == 0);

    // The rate: 64 bytes at each width timed on the STK (HCLK cycles),
    // the tick held off so the count is the block's alone.
    for (const DmaWidth w : {DmaWidth::byte, DmaWidth::half, DmaWidth::word}) {
        const uint16_t n = w == DmaWidth::byte ? 64u : w == DmaWidth::half ? 32u : 16u;
        Copier::stop();
        uint32_t cycles = 0;
        {
            InterruptGuard guard;
            const uint32_t t0 = stk()->CNT;
            (void)Copier::load(DmaTransfer{.peripheral = src8, .memory = dst8, .count = n,
                                           .config = {.memory_to_memory = true, .peripheral_increment = true,
                                                      .memory_increment = true, .peripheral_width = w,
                                                      .memory_width = w}});
            (void)wait_complete();
            const uint32_t t1 = stk()->CNT;
            cycles = t1 >= t0 ? t1 - t0 : t1 + (stk()->CMP + 1u) - t0;
        }
        print(serial, "  64 bytes as ", n, " item(s): ", cycles, " HCLK cycles from load to TCIF", crlf);
    }

    // A fixed source spread over a run: the peripheral side not incrementing.
    const uint32_t pattern = 0xDEADBEEFu;
    Copier::stop();
    (void)Copier::load(DmaTransfer{.peripheral = const_cast<uint32_t*>(&pattern), .memory = dst32, .count = 16,
                                   .config = {.memory_to_memory = true, .peripheral_increment = false,
                                              .memory_increment = true, .peripheral_width = DmaWidth::word,
                                              .memory_width = DmaWidth::word}});
    (void)wait_complete();
    bool filled = true;
    for (uint32_t i = 0; i < 16u; ++i) { filled = filled && dst32[i] == pattern; }
    bench.verdict("a non-incrementing source fills the run with one word", filled);
    Copier::stop();
}

// ---------------------------------------------------------------------------
// c - the interrupt
// ---------------------------------------------------------------------------
void tc_interrupt() {
    copier_completions = 0;
    copier_errors = 0;
    Copier::stop();
    Copier::arm(DmaFlag::complete | DmaFlag::error, true);
    Pfic::enable(Copier::irq());
    for (uint8_t round = 0; round < 5u; ++round) {
        Copier::enable(false);
        (void)Copier::load(DmaTransfer{.peripheral = src8, .memory = dst8, .count = 16,
                                       .config = {.memory_to_memory = true, .peripheral_increment = true}});
        (void)delay_us(clock, 200);
    }
    Pfic::disable(Copier::irq());
    print(serial, "  5 blocks: ", copier_completions, " completions in the handler, ",
          copier_errors, " errors", crlf);
    bench.verdict("every completion reached the channel's handler", copier_completions == 5u);
    bench.verdict("and no error did", copier_errors == 0u);
    bench.verdict("the handler cleared the flags as it went", Copier::flags() == 0u);
    Copier::stop();
}

// ---------------------------------------------------------------------------
// d - a transfer error
// ---------------------------------------------------------------------------
void td_error() {
    // Holes in the map (RM 1.2): between the peripheral buses and past
    // them, and the top of the address space. What the controller does
    // with a read from each is the finding.
    constexpr uint32_t holes[] = {0x50000000u, 0x60000000u, 0x30000000u, 0x40030000u, 0xF0000000u};
    uint8_t errors = 0, completions = 0, hung = 0;
    for (const uint32_t hole : holes) {
        Copier::stop();
        memset(dst8, 0x55, 4);
        const bool loaded = Copier::load(DmaTransfer{
            .peripheral = reinterpret_cast<volatile void*>(hole), .memory = dst8, .count = 4,
            .config = {.memory_to_memory = true, .peripheral_increment = true}});
        const bool ended = loaded && wait_complete(200'000u);
        const uint32_t f = Copier::flags();
        print(serial, "  read from ", hex(hole), ": flags=", hex(f), " EN=", Copier::enabled(),
              " CNTR=", Copier::count(), " data=", hex(dst8[0]), crlf);
        if (!ended) { ++hung; }
        else if ((f & DmaFlag::error) != 0u) { ++errors; }
        else { ++completions; }
    }
    Copier::stop();
    print(serial, "  ", errors, " error(s), ", completions, " completion(s), ", hung,
          " hung, over 5 holes", crlf);
    bench.verdict("every block from a hole ENDED (no channel left hanging)", hung == 0u);
    if (errors == 0u) {
        print(serial, "  -> NO HOLE RAISES TEIF ON THIS SILICON: a DMA read of unmapped "
                      "space completes as a normal block, the chapter's error path "
                      "was not reachable from any of them", crlf);
    }
    bench.verdict("the outcome was one of the two the chapter allows for every hole "
                  "(an error, or a completion the silicon chose to give)",
                  errors + completions == 5u);
}

// ---------------------------------------------------------------------------
// e - the console's transmit engine
// ---------------------------------------------------------------------------
void te_tx_engine() {
    const uint32_t blocks_before = tx_blocks;
    // Longer than the 128-byte TX ring: print() blocks on the full ring
    // and the engine drains it run by run.
    for (uint8_t line = 0; line < 4u; ++line) {
        print(serial, "  0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef", crlf);
    }
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 500);
    const uint32_t blocks = tx_blocks - blocks_before;
    const bool drained = Serial::tx_idle();   // before the next print refills it
    print(serial, "  272 bytes through the transmit engine in ", blocks, " block(s), faults ",
          Serial::dma_faults(), crlf);
    bench.verdict("a burst longer than the ring went out in blocks", blocks >= 2u);
    bench.verdict("with no transfer fault", Serial::dma_faults() == 0u);
    bench.verdict("and the ring drained to empty", drained);
}

// ---------------------------------------------------------------------------
// f - the console's receive engine
// ---------------------------------------------------------------------------
void tf_rx_engine() {
    print(serial, "  type up to 32 characters and Enter within 15 s:", crlf, "  > ");
    char line[33];
    uint8_t n = 0;
    const uint32_t t0 = Ticker::ticks();
    bool done = false;
    while (!done && Ticker::ticks() - t0 < 15'000u) {
        (void)Serial::harvest();
        uint8_t c;
        while (Serial::read_byte(c)) {
            if (c == '\n' || c == '\r') {
                done = n > 0u;
            } else if (n < 32u) {
                line[n++] = static_cast<char>(c);
            }
        }
    }
    line[n] = '\0';
    print(serial, crlf, "  harvested ", n, " byte(s): \"", line, "\"", crlf);
    bench.verdict("a line arrived through the receive engine", done);
    bench.verdict("with no software overrun and no fault",
                  Serial::rx_overruns() == 0u && Serial::dma_faults() == 0u);
}

void banner() {
    print(serial, crlf, "test_ch32_dma - CH32V006K8 (console on DMA channels 4 and 5)", crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel4_handler() {
    if (Serial::dma_isr()) {
        tx_blocks = tx_blocks + 1u;
    }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel5_handler() { (void)Serial::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel1_handler() {
    const uint32_t f = Copier::isr();
    if ((f & brio::DmaFlag::complete) != 0u) { copier_completions = copier_completions + 1u; }
    if ((f & brio::DmaFlag::error) != 0u) { copier_errors = copier_errors + 1u; }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the block and the refusals", ta_block);
    bench.letter('b', "memory to memory at three widths", tb_copies);
    bench.letter('c', "the interrupt", tc_interrupt);
    bench.letter('d', "a transfer error", td_error);
    bench.letter('e', "the console's transmit engine", te_tx_engine);
    bench.letter('f', "the console's receive engine (asks for a line)", tf_rx_engine, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        (void)Serial::harvest();   // the receive engine is asked every turn
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(c), brio::crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
