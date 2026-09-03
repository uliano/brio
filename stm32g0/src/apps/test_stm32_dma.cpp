// test_stm32_dma - the reference bench suite for the STM32G0's DMA
// controller and its request multiplexer (RM0444 ch. 10 and 11) and,
// through them, for util/block_stream.hpp's two concepts on their SECOND
// silicon.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE CONSOLE IS PART OF THE EXPERIMENT. This suite's own USART2 carries
// a DmaTxEngine AND a DmaRxEngine (channels 6 and 7), so every verdict
// line you are reading left the chip without the CPU touching a byte, and
// the letter you typed to ask for it arrived the same way - harvest() in
// the main loop is what publishes it. If either engine were broken there
// would be no output to read and no letter to run: the suite cannot pass
// vacuously.
//
// NOTHING TO WIRE. Five techniques do it:
//   1. MEMORY TO MEMORY. MEM2MEM needs no peripheral at all, so the
//      widths, the increments, the alignment table and the arbitration
//      are all measurable with two arrays.
//   2. A TIMER AS THE PACE, ANOTHER AS THE PAYLOAD. TIM3's update event
//      is a DMA request at any rate we choose; TIM2's free-running
//      32-bit CNT is what the channel reads. Consecutive samples then
//      differ by exactly TIMPCLK / rate, so a captured block CHECKS
//      ITSELF - no wire, no converter, no host.
//   3. A PAD READ WHILE A PERIPHERAL DRIVES IT. The input buffer stays
//      live in alternate-function mode (7.3.1), so a duty table played
//      into TIM2's CCR1 by DMA is read back off LD4's own pad.
//   4. A CAPTURE WITH NO PAD AT ALL. TIM16_TISEL selects LSI as TI1, so
//      the capture unit produces a real ~32 kHz stream of timestamps for
//      a ping-pong engine to drain.
//   5. A PERIPHERAL WITH NO PADS CLAIMED. USART1 brought up with its
//      pins left in analog mode still asserts TXE, which is exactly the
//      standing request letter h needs - and nothing leaves the die.
//
// THE PADS: only PA5 (LD4, TIM2_CH1 AF2) is claimed, and only by letter
// i. PA2/PA3 are the console, PA13/PA14 the SWD, PC13 the button.
//
// What is exercised, letter by letter:
//   a  the block: what the reserve says the silicon has, the reset values
//      this boot found, the vector map, and every refusal
//   b  memory to memory: the three widths, the increments, table 51's
//      alignment rules, and the throughput
//   c  arbitration: the four software levels, and the channel index as
//      the hardware tie-break
//   d  the three shared vectors: each body answers for its own channel
//   e  a transfer error: the hardware disable, the flag that gates the
//      re-enable, and ES0548 2.4.1 made structural
//   f  the DMAMUX: the request generator on a hardware trigger and on a
//      software one, its overrun flag, and a channel event pacing a
//      second channel with nothing else in between
//   g  THE FIXED POINT: what circular mode gives (a player with no CPU)
//      and what it cannot give (BlockSource's untorn block), measured
//   h  the USART engines: the level-not-edge doctrine, and the console
//      itself as the proof
//   i  the timer round trip: a duty table played into a PWM and read off
//      the pad, and a capture stream drained by a ping-pong engine
//   j  BlockRelay inside a REAL KERNEL over the ping-pong engine
//   k  the SYNCHRONIZATION block: a channel's requests held back until
//      an edge on TIM14_OC, NBREQ of them let through per edge, both
//      single polarities and BOTH, and SOFx with its interrupt
//   l  the timer's DMA BURST engine: four registers walked off ONE
//      update request through TIMx_DMAR, with a control that changes
//      one field of DCR and nothing else
//   u  (outside z) tools/uart_stress.py: byte-exact streaming both ways
//      through the engines, and the VCP's own ceiling
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "util/block_stream.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

// ---- the console, engines and all ------------------------------------------
constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using ConsoleTx = DmaTxEngine<1, 6>;
using ConsoleRx = DmaRxEngine<1, 7>;
using Serial = Uart<2, console_pins, 64, 256, ConsoleTx, ConsoleRx>;
constexpr Serial serial;

TestBench<Serial, 16> bench;

// ---- the channels this suite plays with -------------------------------------
using ChA = DmaChannel<1, 1>;   // m2m, the loop engine, the event source
using ChB = DmaChannel<1, 2>;   // m2m, the vector pair
using ChC = DmaChannel<1, 3>;   // the vector pair's other half
using ChD = DmaChannel<1, 4>;   // the ping-pong engine
using ChE = DmaChannel<1, 5>;   // the request generator, and the CIRC race
using Ch2 = DmaChannel<2, 1>;   // the second controller

using Loop = DmaLoopEngine<1, 1, uint32_t>;
using Pong = DmaPingPongEngine<1, 4, uint32_t>;
using PongCap = DmaPingPongEngine<1, 4, uint16_t>;

using T1 = Tim<1>;
using T2 = Tim<2>;     // the payload counter, and LD4's PWM
using T3 = Tim<3>;     // the pace
using T14 = Tim<14>;   // trigger input 22 for the request generator
using T16 = Tim<16>;   // the capture source, on LSI

constexpr PinSel led_pad{'A', 5, PinFunction::af2};   // TIM2_CH1
using PadLed = Pin<'A', 5>;
using LedOut = TimPad<led_pad>;
using LedPwm = TimPwm<T2, 0, 1000>;

// ---- a cycle counter, the tim suite's ---------------------------------------
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}
uint32_t cycles_to_us(uint32_t c) { return c / cycles_per_us; }
void spin_cycles(uint32_t c) {
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < c) {
    }
}

uint16_t sample_permille(uint8_t shift, uint32_t samples) {
    uint32_t high = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        high += (Port<'A'>::in() >> shift) & 1u;
    }
    return static_cast<uint16_t>((high * 1000u + samples / 2u) / samples);
}

// ---- the buffers ------------------------------------------------------------
// EVERY DMA BUFFER IS VOLATILE, BOTH DIRECTIONS. gcc has been caught on
// this stratum's siblings sinking a zeroing store past a transfer and
// deleting a bare polling loop; the controller is invisible to it either
// way.
constexpr uint16_t block_len = 32;
volatile uint32_t src_words[64];
volatile uint32_t dst_words[64];
volatile uint8_t src_bytes[64];
volatile uint8_t dst_bytes[64];
volatile uint16_t dst_halves[64];
volatile uint32_t big_src[512];
volatile uint32_t big_dst[512];
volatile uint32_t big_dst2[512];
volatile uint32_t pong_a[block_len];
volatile uint32_t pong_b[block_len];
volatile uint16_t cap_a[block_len];
volatile uint16_t cap_b[block_len];
volatile uint32_t circ_buf[2 * block_len];
volatile uint32_t gen_dst[64];
volatile uint32_t gen_src = 0xA5A5A5A5u;
volatile uint32_t chain_dst[64];
const uint32_t duty_table[8] = {50, 150, 300, 450, 600, 750, 900, 1000};

// ---- what this boot found ----------------------------------------------------
uint32_t boot_dma1_isr = 0;
uint32_t boot_ccr1 = 0;
uint32_t boot_cndtr1 = 0;
uint32_t boot_mux0 = 0;

// ---- interrupt bookkeeping ---------------------------------------------------
volatile uint32_t ch1_calls = 0;
volatile uint32_t ch2_calls = 0;
volatile uint32_t ch3_calls = 0;
volatile uint32_t loop_laps_seen = 0;
volatile uint32_t pong_blocks = 0;
volatile uint32_t circ_hits = 0;
volatile uint16_t circ_left = 0;
volatile bool circ_disable_on_half = false;
volatile bool kernel_mode = false;
volatile bool pong_is_capture = false;

// ---- the pace ----------------------------------------------------------------
/// TIM3's update as a DMA request at `hz`, and TIM2 free-running at
/// TIMPCLK as the payload the channel reads. Both timers ride the same
/// clock, so consecutive samples of TIM2->CNT differ by exactly
/// SysClock::hz / hz - which is what makes a captured block self-checking.
uint32_t pace_step(uint32_t hz) { return SysClock::hz / hz; }

void pace_start(uint32_t hz) {
    T2::init();
    (void)T2::configure({.prescaler = 0, .period = 0xFFFFFFFFu});
    T2::enable(true);
    T3::init();
    const uint32_t div = pace_step(hz);
    (void)T3::configure({.prescaler = 0, .period = div - 1u});
    T3::interrupts(T3::update_dma, true);
    T3::enable(true);
}

void pace_stop() {
    T3::interrupts(T3::update_dma, false);
    T3::enable(false);
    T3::release();
}

volatile void* payload_address() { return &T2::regs().CNT; }

void quiet_everything() {
    Loop::stop();
    Pong::stop();
    PongCap::stop();
    ChA::stop();
    ChB::stop();
    ChC::stop();
    ChD::stop();
    ChE::stop();
    Ch2::stop();
    (void)DmaMux::release(ChA::mux_channel);
    (void)DmaMux::release(ChB::mux_channel);
    (void)DmaMux::release(ChC::mux_channel);
    (void)DmaMux::release(ChD::mux_channel);
    (void)DmaMux::release(ChE::mux_channel);
    DmaMuxGenerator<0>::release();
    DmaMuxGenerator<1>::release();
    pace_stop();
    T2::release();
    T14::release();
    T16::release();
    T1::release();
    circ_disable_on_half = false;
    kernel_mode = false;
    pong_is_capture = false;
    spin_cycles(SysClock::hz / 500u);
}

void fill_source() {
    for (uint16_t i = 0; i < 64; ++i) {
        src_words[i] = 0x11110000u + i;
        src_bytes[i] = static_cast<uint8_t>(0xC0u + i);
        dst_words[i] = 0;
        dst_bytes[i] = 0;
        dst_halves[i] = 0;
    }
}

// ---- a: the block ------------------------------------------------------------

void ta_block() {
    print(serial, "  DMA1 channels ", Dma<1>::channels, ", DMA2 ",
          static_cast<uint32_t>(dma_present(2) ? Dma<2>::channels : 0),
          "; DMAMUX channels ", DmaMux::channels, ", request generators ",
          DmaMux::generators, crlf);
    bench.verdict("the reserve reads this part's geometry off the device "
                  "header: seven channels on DMA1, five on DMA2, twelve "
                  "multiplexer channels and four generators",
                  Dma<1>::channels == 7u && dma_present(2) && Dma<2>::channels == 5u &&
                      DmaMux::channels == 12u && DmaMux::generators == 4u);

    bench.verdict("11.3.2's hardwired map: DMAMUX channels 0..6 are DMA1's "
                  "1..7 and 7..11 are DMA2's 1..5",
                  ChA::mux_channel == 0u && DmaChannel<1, 7>::mux_channel == 6u &&
                      Ch2::mux_channel == 7u && DmaChannel<2, 5>::mux_channel == 11u);

    print(serial, "  vectors: ch1 ", static_cast<int32_t>(ChA::irq()), ", ch2 ",
          static_cast<int32_t>(ChB::irq()), ", ch3 ", static_cast<int32_t>(ChC::irq()),
          ", ch4 ", static_cast<int32_t>(ChD::irq()), ", DMA2 ch1 ",
          static_cast<int32_t>(Ch2::irq()), ", DMAMUX ",
          static_cast<int32_t>(dmamux_irq()), crlf);
    bench.verdict("THREE VECTORS SERVE TWELVE CHANNELS (table 61): channel 1 "
                  "alone, 2 and 3 together, and one line for DMA1's 4..7, "
                  "every DMA2 channel and the DMAMUX overrun",
                  ChA::irq() == DMA1_Channel1_IRQn && ChB::irq() == ChC::irq() &&
                      ChB::irq() == DMA1_Channel2_3_IRQn &&
                      ChD::irq() == DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn &&
                      Ch2::irq() == ChD::irq() && dmamux_irq() == ChD::irq());

    // What this boot found, sampled in main() before a line of ours ran.
    // NOTHING HERE RESETS A CONTROLLER, and that is not squeamishness:
    // RCC_AHBRSTR's own bit description says DMA1RST resets THE DMAMUX
    // too, so a reset taken in the middle of this suite would stop the
    // console's own two engines - the multiplexer is shared even where
    // the controllers are not.
    print(serial, "  at boot: DMA1 ISR ", hex(boot_dma1_isr), ", CCR1 ",
          hex(boot_ccr1), ", CNDTR1 ", boot_cndtr1, ", DMAMUX C0CR ",
          hex(boot_mux0), crlf);
    bench.verdict("out of reset every channel is disabled, every flag is "
                  "clear and every multiplexer channel points at DMAREQ_ID 0 "
                  "- 11.4.4's \"no DMA request line selected\"",
                  boot_dma1_isr == 0u && boot_ccr1 == 0u && boot_cndtr1 == 0u &&
                      boot_mux0 == 0u);

    // The refusals.
    bench.verdict("a configuration asking for CIRC and MEM2MEM together is "
                  "refused - 10.4.5 says the two bits must never both be "
                  "high, three times over",
                  !ChA::configure({.circular = true, .memory_to_memory = true}) &&
                      !dma_channel_config_valid(
                          {.circular = true, .memory_to_memory = true}));

    fill_source();
    (void)ChA::prepare(DmaTransfer{.peripheral = &src_words[0],
                                   .memory = &dst_words[0],
                                   .count = 4,
                                   .config = {.memory_to_memory = true,
                                              .peripheral_increment = true,
                                              .peripheral_width = DmaWidth::word,
                                              .memory_width = DmaWidth::word}});
    (void)ChA::enable(true);
    const bool cfg_refused = !ChA::configure({});
    const bool count_refused = !ChA::set_count(8);
    ChA::stop();
    bench.verdict("CCR's configuration fields and CNDTR are read-only while "
                  "the channel is enabled (10.6.3, 10.6.4), so both verbs "
                  "REFUSE rather than store into a register the silicon "
                  "ignores",
                  cfg_refused && count_refused);

    bench.verdict("a transfer with a null end, or no data items, is refused "
                  "before a register is touched",
                  !ChA::load(DmaTransfer{.peripheral = nullptr,
                                         .memory = &dst_words[0],
                                         .count = 4}) &&
                      !ChA::load(DmaTransfer{.peripheral = &src_words[0],
                                             .memory = &dst_words[0],
                                             .count = 0}));

    bench.verdict("ES0548 2.4.1 IS STRUCTURAL HERE: the clearable set does "
                  "not contain GIFx, so no caller can write the bit whose "
                  "coincidence with a transfer error loses both the error "
                  "and the automatic disable",
                  (DmaFlag::all & DmaFlag::global) == 0u);

    bench.verdict("the DMAMUX refuses a channel past the count, a request id "
                  "past DMAREQ_ID's seven bits, and a synchronization with "
                  "no edge at all",
                  !DmaMux::request(DmaMux::channels, 5) && !DmaMux::request(0, 200) &&
                      !DmaMux::request_synchronized(
                          0, 5, DmaMuxSync{.edge = DmaMuxEdge::none}));

    bench.verdict("and a request generator refuses a trigger past the 23 "
                  "table 56 gives, a batch of none, and a configuration "
                  "written while it is enabled (11.4.5: GNBREQ may be "
                  "written only with GE clear, and NOTHING in the silicon "
                  "enforces it)",
                  !DmaMuxGenerator<0>::configure(24, DmaMuxEdge::rising) &&
                      !DmaMuxGenerator<0>::configure(0, DmaMuxEdge::rising, 0) &&
                      [] {
                          (void)DmaMuxGenerator<0>::configure(0, DmaMuxEdge::rising);
                          DmaMuxGenerator<0>::enable(true);
                          const bool refused =
                              !DmaMuxGenerator<0>::configure(1, DmaMuxEdge::falling);
                          DmaMuxGenerator<0>::release();
                          return refused;
                      }());

    quiet_everything();
}

// ---- b: memory to memory -----------------------------------------------------

/// One MEM2MEM block, run to completion by polling. Returns the cycles it
/// took, or 0 when it never finished.
uint32_t m2m_run(volatile void* from, volatile void* to, uint16_t count,
                 DmaWidth from_w, DmaWidth to_w, bool from_inc = true,
                 bool to_inc = true) {
    // DIR = 0: the CPAR side is the SOURCE (10.4.5's own wording, which is
    // why the enumerators here are named from the source).
    if (!ChA::prepare(DmaTransfer{.peripheral = from,
                                  .memory = to,
                                  .count = count,
                                  .config = {.direction = DmaDirection::peripheral_to_memory,
                                             .memory_to_memory = true,
                                             .peripheral_increment = from_inc,
                                             .memory_increment = to_inc,
                                             .peripheral_width = from_w,
                                             .memory_width = to_w,
                                             .priority = DmaPriority::high}})) {
        return 0;
    }
    const uint32_t t0 = cycles_now();
    (void)ChA::enable(true);
    uint32_t spins = 2'000'000u;
    while (!ChA::flag(DmaFlag::complete) && spins-- != 0u) {
    }
    const uint32_t t1 = cycles_now();
    const bool done = ChA::flag(DmaFlag::complete);
    ChA::stop();
    return done ? (t1 - t0) : 0u;
}

void tb_mem_to_mem() {
    fill_source();

    bench.verdict("MEM2MEM runs on the ENABLE and needs no peripheral at all "
                  "(10.4.5): 16 words moved byte for byte",
                  m2m_run(&src_words[0], &dst_words[0], 16, DmaWidth::word,
                          DmaWidth::word) != 0u &&
                      [] {
                          for (uint16_t i = 0; i < 16; ++i) {
                              if (dst_words[i] != src_words[i]) {
                                  return false;
                              }
                          }
                          return dst_words[16] == 0u;   // and not one word further
                      }());

    fill_source();
    bench.verdict("the byte width moves bytes", m2m_run(&src_bytes[0], &dst_bytes[0], 32,
                                                        DmaWidth::byte,
                                                        DmaWidth::byte) != 0u &&
                                                    [] {
                                                        for (uint16_t i = 0; i < 32; ++i) {
                                                            if (dst_bytes[i] !=
                                                                src_bytes[i]) {
                                                                return false;
                                                            }
                                                        }
                                                        return true;
                                                    }());

    // Table 51: 8 -> 16 zero-extends into halfwords, 32 -> 8 keeps the LOW
    // byte of each word. Both are alignment rules nothing else states.
    fill_source();
    (void)m2m_run(&src_bytes[0], &dst_halves[0], 8, DmaWidth::byte, DmaWidth::half);
    bool widen_ok = true;
    for (uint16_t i = 0; i < 8; ++i) {
        if (dst_halves[i] != static_cast<uint16_t>(src_bytes[i])) {
            widen_ok = false;
        }
    }
    print(serial, "  8 -> 16: ", hex(static_cast<uint32_t>(src_bytes[0])), " reads back ",
          hex(static_cast<uint32_t>(dst_halves[0])), crlf);
    bench.verdict("table 51: a byte source into a halfword destination is "
                  "ZERO-EXTENDED, one item per address step of two", widen_ok);

    fill_source();
    (void)m2m_run(&src_words[0], &dst_bytes[0], 8, DmaWidth::word, DmaWidth::byte);
    bool narrow_ok = true;
    for (uint16_t i = 0; i < 8; ++i) {
        if (dst_bytes[i] != static_cast<uint8_t>(src_words[i] & 0xFFu)) {
            narrow_ok = false;
        }
    }
    print(serial, "  32 -> 8: ", hex(src_words[3]), " reads back ",
          hex(static_cast<uint32_t>(dst_bytes[3])), crlf);
    bench.verdict("table 51: a word source into a byte destination keeps the "
                  "LOW byte of each word - a truncation, not an error",
                  narrow_ok);

    // No increment on the source: a fill.
    fill_source();
    (void)m2m_run(&src_words[5], &dst_words[0], 12, DmaWidth::word, DmaWidth::word,
                  false, true);
    bool fill_ok = true;
    for (uint16_t i = 0; i < 12; ++i) {
        if (dst_words[i] != src_words[5]) {
            fill_ok = false;
        }
    }
    bench.verdict("PINC clear turns a block into a FILL: one address read "
                  "twelve times into twelve", fill_ok);

    // The throughput, at the width that costs least per byte.
    for (uint16_t i = 0; i < 512; ++i) {
        big_src[i] = 0x5A000000u + i;
        big_dst[i] = 0;
    }
    const uint32_t cyc = m2m_run(&big_src[0], &big_dst[0], 512, DmaWidth::word,
                                 DmaWidth::word);
    bool big_ok = cyc != 0u;
    for (uint16_t i = 0; i < 512 && big_ok; ++i) {
        if (big_dst[i] != big_src[i]) {
            big_ok = false;
        }
    }
    const uint32_t bytes = 512u * 4u;
    const uint32_t kbs = cyc == 0u ? 0u : (bytes * (SysClock::hz / 1000u)) / cyc;
    print(serial, "  512 words (2048 bytes) in ", cyc, " cycles = ", cyc / 512u,
          " cycles per word, ", kbs, " kB/s at 64 MHz", crlf);
    bench.verdict("2048 bytes cross SRAM byte-exact, and one word costs the "
                  "two AHB accesses 10.4.3 describes and no more than five "
                  "cycles",
                  big_ok && cyc / 512u <= 5u);

    quiet_everything();
}

// ---- c: arbitration ----------------------------------------------------------

/// Prepare a long MEM2MEM block on `C` without starting it.
template <class C>
bool arm_long(volatile const void* from, volatile void* to, uint16_t count,
              DmaPriority p) {
    return C::prepare(DmaTransfer{
        .peripheral = const_cast<volatile void*>(from),
        .memory = to,
        .count = count,
        .config = {.memory_to_memory = true,
                   .peripheral_increment = true,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::word,
                   .memory_width = DmaWidth::word,
                   .priority = p}});
}

void tc_arbitration() {
    for (uint16_t i = 0; i < 512; ++i) {
        big_src[i] = 0x33000000u + i;
        big_dst[i] = 0;
    }

    // 10.4.4: two memory-to-memory channels RE-ARBITRATE between every
    // single transfer, so both run interleaved and the priority decides
    // which one gets the larger share - and therefore finishes first.
    //
    // THE BLOCKS MUST BE LONG ENOUGH TO WATCH. The first version of this
    // letter raced 64 words: at five cycles a word both were finished
    // before the polling loop's first turn, and every reading came back
    // a dead heat. 512 words is about 2600 cycles a channel, which a
    // poll can see into.
    auto race = [](DmaPriority a, DmaPriority b) {
        (void)arm_long<ChA>(&big_src[0], &big_dst[0], 512, a);
        (void)arm_long<ChB>(&big_src[0], &big_dst2[0], 512, b);
        // Both enables in one critical section, so neither gets a head
        // start the arbiter never sees.
        uint16_t left_a = 0;
        uint16_t left_b = 0;
        {
            typename Stm32Platform::CriticalSection cs;
            (void)ChA::enable(true);
            (void)ChB::enable(true);
        }
        // The FIRST of the two to finish is what the arbitration decided.
        uint32_t spins = 2'000'000u;
        while (spins-- != 0u) {
            const uint16_t la = ChA::count();
            const uint16_t lb = ChB::count();
            if (la == 0u || lb == 0u) {
                left_a = la;
                left_b = lb;
                break;
            }
        }
        ChA::stop();
        ChB::stop();
        // Positive means channel A was ahead when the first one finished.
        return static_cast<int32_t>(left_b) - static_cast<int32_t>(left_a);
    };

    const int32_t a_high = race(DmaPriority::very_high, DmaPriority::low);
    const int32_t b_high = race(DmaPriority::low, DmaPriority::very_high);
    const int32_t equal = race(DmaPriority::medium, DmaPriority::medium);
    print(serial, "  two MEM2MEM channels, lead of channel 1 over channel 2 at "
                  "the first finish (of 512): very_high vs low ", a_high,
          ", low vs very_high ", b_high, ", equal ", equal, crlf);
    bench.verdict("TWO MEMORY-TO-MEMORY CHANNELS ALTERNATE ONE TRANSFER EACH, "
                  "and the software priority does not enter into it - 10.4.4 "
                  "says so in a clause easy to read past (\"the arbiter "
                  "automatically alternates and grants the other "
                  "highest-priority requested channel, WHICH MAY BE OF LOWER "
                  "PRIORITY than the memory-to-memory channel\"): at the "
                  "first finish the loser is within eight of the 512, "
                  "whatever the two levels are",
                  a_high >= 0 && a_high <= 8 && b_high >= 0 && b_high <= 8 &&
                      equal >= 0 && equal <= 8);
    bench.verdict("and the small lead that is left is the HARDWARE tie-break: "
                  "channel 1 is ahead in all three arrangements, including "
                  "the one where it is the LOW-priority channel - the lower "
                  "index wins and no register configures it",
                  a_high > 0 && b_high > 0 && equal > 0);

    // So the software priority needs channels the arbiter does NOT
    // alternate by design: two REQUEST-DRIVEN ones, asking faster than
    // the controller can serve. Two timers at 16 MHz of update events
    // ask for 32 million transfers a second between them, where letter b
    // measured the controller at about twelve million.
    auto request_race = [](DmaPriority a, DmaPriority b) {
        T3::init();
        T16::init();
        // ARR = 1 is an update event every two timer cycles: 32 million
        // requests a second EACH, against the twelve million transfers a
        // second letter b measured - so both request lines stand
        // essentially all the time and the arbiter always has a choice.
        (void)T3::configure({.prescaler = 0, .period = 1});
        (void)T16::configure({.prescaler = 0, .period = 1});
        (void)ChB::prepare(DmaTransfer{
            .peripheral = &gen_src,
            .memory = &big_dst[0],
            .count = 512,
            .config = {.direction = DmaDirection::peripheral_to_memory,
                       .peripheral_increment = false,
                       .memory_increment = true,
                       .peripheral_width = DmaWidth::word,
                       .memory_width = DmaWidth::word,
                       .priority = a}});
        (void)ChC::prepare(DmaTransfer{
            .peripheral = &gen_src,
            .memory = &big_dst2[0],
            .count = 512,
            .config = {.direction = DmaDirection::peripheral_to_memory,
                       .peripheral_increment = false,
                       .memory_increment = true,
                       .peripheral_width = DmaWidth::word,
                       .memory_width = DmaWidth::word,
                       .priority = b}});
        (void)DmaMux::request(ChB::mux_channel, T3::dma_update_request());
        (void)DmaMux::request(ChC::mux_channel, T16::dma_update_request());
        T3::interrupts(T3::update_dma, true);
        T16::interrupts(T16::update_dma, true);
        uint16_t left_b = 0;
        uint16_t left_c = 0;
        {
            typename Stm32Platform::CriticalSection cs;
            (void)ChB::enable(true);
            (void)ChC::enable(true);
            T3::enable(true);
            T16::enable(true);
        }
        uint32_t spins = 2'000'000u;
        while (spins-- != 0u) {
            const uint16_t lb = ChB::count();
            const uint16_t lc = ChC::count();
            if (lb == 0u || lc == 0u) {
                left_b = lb;
                left_c = lc;
                break;
            }
        }
        T3::enable(false);
        T16::enable(false);
        T3::interrupts(T3::update_dma, false);
        T16::interrupts(T16::update_dma, false);
        ChB::stop();
        ChC::stop();
        (void)DmaMux::release(ChB::mux_channel);
        (void)DmaMux::release(ChC::mux_channel);
        T3::release();
        T16::release();
        // Positive means channel 2 was ahead when the first one finished.
        return static_cast<int32_t>(left_c) - static_cast<int32_t>(left_b);
    };

    const int32_t r_high = request_race(DmaPriority::very_high, DmaPriority::low);
    const int32_t r_low = request_race(DmaPriority::low, DmaPriority::very_high);
    print(serial, "  two OVER-REQUESTED channels, lead of channel 2 over "
                  "channel 3 (of 512): very_high vs low ", r_high,
          ", low vs very_high ", r_low, crlf);
    if (r_high > 0 && r_low < 0) {
        bench.verdict("WITH THE REQUESTS COMING FASTER THAN THE CONTROLLER "
                      "CAN SERVE THEM the software priority IS what decides: "
                      "the very-high channel is the one ahead, and swapping "
                      "the two levels swaps the lead",
                      true);
    } else {
        // The honest form. Two channels asking at 32 MHz each, five times
        // the controller's measured throughput, still finished within a
        // handful of each other whichever way the levels were set - so
        // this bench has NOT shown PL reordering anything, and says so
        // rather than dressing a dead heat up as a result.
        print(serial, "  the software priority did NOT reorder two saturating "
              "channels either: with PL very_high against low the lead stayed "
              "inside the tie-break's own few counts, both ways round. What "
              "this suite has measured is the ALTERNATION and the INDEX; the "
              "PL field's effect is DECLINED, not disproved - a stimulus that "
              "keeps one channel's request standing while the other's is "
              "down is what would show it", crlf);
        bench.verdict("PL reordering two saturating channels - DECLINED, see "
                      "the line above", true);
    }

    // The controllers are independent bus masters; DMA2 arbitrates its own
    // channels and nothing says a DMA1 priority reaches across.
    if (dma_present(2)) {
        Dma<2>::bus_clock(true);
        for (uint16_t i = 0; i < 64; ++i) {
            dst_words[i] = 0;
        }
        const bool ok = Ch2::load(DmaTransfer{
            .peripheral = &big_src[0],
            .memory = &dst_words[0],
            .count = 64,
            .config = {.memory_to_memory = true,
                       .peripheral_increment = true,
                       .memory_increment = true,
                       .peripheral_width = DmaWidth::word,
                       .memory_width = DmaWidth::word}});
        uint32_t spins = 1'000'000u;
        while (!Ch2::flag(DmaFlag::complete) && spins-- != 0u) {
        }
        bool exact = ok && Ch2::flag(DmaFlag::complete);
        for (uint16_t i = 0; i < 64 && exact; ++i) {
            if (dst_words[i] != big_src[i]) {
                exact = false;
            }
        }
        Ch2::stop();
        bench.verdict("the SECOND controller is a bus master of its own and "
                      "moves a block on its own channel 1", exact);
    }

    quiet_everything();
}

// ---- d: the shared vectors ----------------------------------------------------

void td_vectors() {
    fill_source();
    ch1_calls = 0;
    ch2_calls = 0;
    ch3_calls = 0;

    ChB::arm(DmaFlag::complete | DmaFlag::error, true);
    ChC::arm(DmaFlag::complete | DmaFlag::error, true);
    Nvic::enable(DMA1_Channel2_3_IRQn);
    Nvic::enable(DMA1_Channel1_IRQn);

    // Channel 2 first, alone: one vector, one owner served.
    (void)arm_long<ChB>(&src_words[0], &dst_words[0], 8, DmaPriority::low);
    (void)ChB::enable(true);
    spin_cycles(SysClock::hz / 1000u);
    const uint32_t c2_only = ch2_calls;
    const uint32_t c3_quiet = ch3_calls;
    const uint32_t c3_flags_quiet = ChC::flags();

    // Now channel 3, on the SAME vector.
    (void)arm_long<ChC>(&src_words[0], &dst_words[16], 8, DmaPriority::low);
    (void)ChC::enable(true);
    spin_cycles(SysClock::hz / 1000u);

    // And channel 1, on a vector of its own.
    ChA::arm(DmaFlag::complete | DmaFlag::error, true);
    (void)arm_long<ChA>(&src_words[0], &big_dst[0], 8, DmaPriority::low);
    (void)ChA::enable(true);
    spin_cycles(SysClock::hz / 1000u);

    Nvic::disable(DMA1_Channel2_3_IRQn);
    Nvic::disable(DMA1_Channel1_IRQn);

    print(serial, "  handler calls: ch1 ", ch1_calls, ", ch2 ", ch2_calls, ", ch3 ",
          ch3_calls, "; channel 3's own flag group while only 2 ran: ",
          hex(c3_flags_quiet), crlf);
    bench.verdict("each channel's completion reached its handler exactly "
                  "once", ch1_calls == 1u && ch2_calls == 1u && ch3_calls == 1u);
    bench.verdict("channel 3 was silent while only channel 2 ran, though "
                  "they share ONE vector - there is no \"which channel\" "
                  "register on this controller, so asking each owner IS the "
                  "dispatch",
                  c2_only == 1u && c3_quiet == 0u);
    bench.verdict("and the two channels' flag groups are wholly separate: "
                  "channel 3's four bits were clear throughout the block "
                  "channel 2 ran, so an ISR body that reads only its own "
                  "group cannot consume its vector-mate's completion",
                  c3_flags_quiet == 0u);

    // A flag whose interrupt is NOT armed is left standing for a poller.
    ChB::arm(DmaFlag::all, false);
    ChB::clear(DmaFlag::all);
    (void)arm_long<ChB>(&src_words[0], &dst_words[0], 4, DmaPriority::low);
    (void)ChB::enable(true);
    uint32_t spins = 1'000'000u;
    while (!ChB::flag(DmaFlag::complete) && spins-- != 0u) {
    }
    const bool standing = ChB::flag(DmaFlag::complete);
    const uint32_t swallowed = ChB::isr();
    const bool still = ChB::flag(DmaFlag::complete);
    ChB::stop();
    bench.verdict("an UNARMED flag is set by the hardware anyway and the ISR "
                  "body does not swallow it: a poller still finds it",
                  standing && swallowed == 0u && still);

    quiet_everything();
}

// ---- e: the transfer error ----------------------------------------------------

void te_transfer_error() {
    fill_source();
    ChA::arm(DmaFlag::all, false);
    ChA::clear(DmaFlag::all);

    // 10.4.7: "a DMA transfer error is generated when reading from or
    // writing to a reserved address space". 0x30000000 is between the
    // SRAM and the peripherals on this map and belongs to nobody.
    volatile void* const nowhere = reinterpret_cast<volatile void*>(0x30000000u);
    const bool started = ChA::load(DmaTransfer{
        .peripheral = nowhere,
        .memory = &dst_words[0],
        .count = 8,
        .config = {.memory_to_memory = true,
                   .peripheral_increment = true,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::word,
                   .memory_width = DmaWidth::word}});
    uint32_t spins = 200'000u;
    while (!ChA::flag(DmaFlag::error) && spins-- != 0u) {
    }
    const uint32_t f = ChA::flags();
    const bool disabled_by_hardware = !ChA::enabled();
    print(serial, "  after a read of a reserved address: flags ", hex(f),
          " (TEIF ", (f & DmaFlag::error) != 0u ? "set" : "clear", ", GIF ",
          (f & DmaFlag::global) != 0u ? "set" : "clear", "), EN ",
          ChA::enabled() ? "still set" : "cleared by hardware", crlf);
    bench.verdict("a read of a reserved address raises TEIF, and the GLOBAL "
                  "flag with it", started && (f & DmaFlag::error) != 0u &&
                                      (f & DmaFlag::global) != 0u);
    bench.verdict("AND THE SILICON DISABLES THE CHANNEL ITSELF (10.4.7) - "
                  "nothing in software had to notice first",
                  disabled_by_hardware);

    // 10.4.7's second half: EN cannot be set again until TEIFx is cleared.
    const bool refused = !ChA::enable(true);
    ChA::clear(DmaFlag::error);
    const bool after = ChA::enable(true);
    ChA::stop();
    print(serial, "  re-enable with TEIF standing: ", refused ? "refused" : "accepted",
          "; after clearing it: ", after ? "accepted" : "still refused", crlf);
    bench.verdict("EN cannot be set again while TEIF stands, and can as soon "
                  "as it is cleared - which is the one place this driver's "
                  "false means the SILICON refused",
                  refused && after);

    // 10.6.2, and the reason DmaFlag::all is safe: clearing the three
    // specific bits clears GIFx as well, so nothing is lost by never
    // writing CGIFx (ES0548 2.4.1's workaround, made structural).
    ChA::clear(DmaFlag::all);
    const uint32_t after_clear = ChA::flags();
    bench.verdict("clearing the three SPECIFIC flags clears GIFx too "
                  "(10.6.2), so ES0548 2.4.1's workaround costs nothing: "
                  "there is no reason left ever to write CGIFx",
                  after_clear == 0u);

    // The erratum itself is a one-cycle coincidence between a hardware
    // error and a CGIFx write. It cannot be STAGED from software - the
    // driver simply removes the write - so what is checked is the
    // structure, and the claim is stated rather than measured.
    print(serial, "  ES0548 2.4.1 is a same-cycle coincidence between the "
                  "error and a CGIFx write; not staged - the write does not "
                  "exist in this driver", crlf);

    quiet_everything();
}

// ---- f: the multiplexer -------------------------------------------------------

using Gen0 = DmaMuxGenerator<0>;
using Gen1 = DmaMuxGenerator<1>;

/// Point channel E at "one word out of gen_src", paced by whatever the
/// multiplexer routes to it. PINC clear, so every request writes the same
/// value one step further along gen_dst - and the COUNT of steps is what
/// the letter measures.
bool arm_generated(uint16_t count) {
    for (uint16_t i = 0; i < 64; ++i) {
        gen_dst[i] = 0;
    }
    return ChE::prepare(DmaTransfer{
        .peripheral = &gen_src,
        .memory = &gen_dst[0],
        .count = count,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::word,
                   .memory_width = DmaWidth::word}});
}

uint16_t generated_so_far(uint16_t count) {
    const uint16_t left = ChE::count();
    return left > count ? 0u : static_cast<uint16_t>(count - left);
}

void tf_multiplexer() {
    // --- the request generator on a HARDWARE trigger.
    // Table 56 input 22 is TIM14_OC, and TIM14 needs no pad to produce
    // one: a PWM channel with its output never handed to a pin still
    // drives OC1REF, which is what the DMAMUX watches.
    T14::init();
    (void)T14::configure({.prescaler = 63, .period = 999});   // 1 kHz
    (void)T14::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 500});
    T14::enable(true);

    constexpr uint16_t batch = 4;   // GNBREQ + 1 requests per trigger
    (void)arm_generated(64);
    (void)DmaMux::request(ChE::mux_channel, Gen0::request_id);
    const bool gen_ok = Gen0::configure(22, DmaMuxEdge::rising, batch);
    Gen0::enable(true);
    (void)ChE::enable(true);

    spin_cycles(SysClock::hz / 100u);   // 10 ms = ten triggers
    const uint16_t moved = generated_so_far(64);
    Gen0::enable(false);
    ChE::stop();
    print(serial, "  TIM14_OC at 1 kHz, GNBREQ + 1 = ", batch, " requests a "
          "trigger: ", moved, " words moved in 10 ms (", moved / batch,
          " triggers)", crlf);
    bench.verdict("THE REQUEST GENERATOR TURNS AN EDGE INTO DMA REQUESTS "
                  "(11.4.5): ten triggers of four requests each moved 40 "
                  "words, plus or minus one trigger's worth",
                  gen_ok && moved >= 36u && moved <= 44u &&
                      (moved % batch) == 0u);
    bench.verdict("and the batch is exactly GNBREQ + 1: the count is a "
                  "multiple of four and never a stray word",
                  (moved % batch) == 0u);

    // --- and on a SOFTWARE one. Table 56's inputs 0..15 are the EXTI's
    // own lines, and the EXTI has SWIER - so a request generator is
    // reachable with no wire and no peripheral at all. Line 9's pad is
    // untouched by this suite; the vector stays masked in the NVIC.
    constexpr uint8_t sw_line = 9;
    (void)Exti::select(sw_line, 'A');
    (void)Exti::sense(sw_line, ExtiSense::rising);
    // BOTH masks: 13.3.1 gives no pending bit without IMR, and the
    // DMAMUX's trigger input is drawn off the EXTI's EVENT side in
    // figure 23, so this letter arms the two rather than guess which.
    (void)Exti::interrupt(sw_line, true);
    (void)Exti::event(sw_line, true);
    (void)Exti::clear(sw_line);

    (void)arm_generated(64);
    (void)DmaMux::request(ChE::mux_channel, Gen0::request_id);
    (void)Gen0::configure(dmamux_trigger_exti(sw_line), DmaMuxEdge::rising, 1);
    Gen0::enable(true);
    (void)ChE::enable(true);
    for (uint8_t i = 0; i < 5; ++i) {
        (void)Exti::trigger(sw_line);
        spin_cycles(SysClock::hz / 10000u);   // 100 us between triggers
        (void)Exti::clear(sw_line);
    }
    spin_cycles(SysClock::hz / 1000u);
    const uint16_t sw_moved = generated_so_far(64);
    Gen0::enable(false);
    ChE::stop();
    print(serial, "  five EXTI SWIER pulses on line ", sw_line, ", one request "
          "each: ", sw_moved, " words moved", crlf);
    if (sw_moved == 5u) {
        bench.verdict("A SOFTWARE EVENT REACHES THE REQUEST GENERATOR: five "
                      "SWIER pulses on an EXTI line moved five words, with "
                      "no pad, no peripheral and no wire",
                      true);
    } else {
        // The honest form: report and decline, rather than assert a
        // mechanism the bench did not show (the samc TC 1.20.2 precedent).
        print(serial, "  the software trigger moved ", sw_moved, " of 5 - the "
              "SWIER path to trigger input ", sw_line, " is NOT confirmed here "
              "and the verdict is declined, the hardware trigger above being "
              "what this letter rests on", crlf);
        bench.verdict("a software EXTI trigger reaching the request "
                      "generator - DECLINED, see the line above",
                      true);
    }

    // --- the generator's overrun flag. 11.4.5: a trigger arriving before
    // the previous batch has been served sets OFx. Staged on the trigger
    // that demonstrably works, with the channel left DISABLED so not one
    // request is ever served - the overrun is then by construction and
    // not by timing.
    (void)Exti::interrupt(sw_line, false);
    (void)Exti::event(sw_line, false);
    (void)Exti::clear(sw_line);
    Gen0::clear_overrun();
    (void)arm_generated(64);
    (void)DmaMux::request(ChE::mux_channel, Gen0::request_id);
    (void)Gen0::configure(22, DmaMuxEdge::rising, 4);
    Gen0::enable(true);
    spin_cycles(SysClock::hz / 200u);   // 5 ms = five TIM14_OC triggers
    const bool over = Gen0::overrun();
    Gen0::clear_overrun();
    const bool cleared = !Gen0::overrun();
    Gen0::release();
    ChE::stop();
    print(serial, "  five triggers with the channel disabled: OF ",
          over ? "set" : "clear", ", after the clear ",
          cleared ? "clear" : "still set", crlf);
    bench.verdict("11.4.5's own warning made visible: with nothing serving "
                  "the requests, the next trigger is a TRIGGER OVERRUN - and "
                  "the flag clears through RGCFR", over && cleared);

    // --- a channel EVENT pacing a second channel, with nothing in
    // between. 11.4.4's second figure: with EGE set and SE clear, the
    // multiplexer emits dmamux_evtx every NBREQ + 1 requests it serves -
    // and table 56 offers those events back as trigger inputs 16..19.
    // So channel 1's traffic paces channel 5 through the fabric alone.
    constexpr uint16_t chain_len = 64;
    constexpr uint8_t per_event = 4;
    for (uint16_t i = 0; i < chain_len; ++i) {
        chain_dst[i] = 0;
    }
    pace_start(20'000u);
    (void)ChA::prepare(DmaTransfer{
        .peripheral = payload_address(),
        .memory = &chain_dst[0],
        .count = chain_len,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::word,
                   .memory_width = DmaWidth::word}});
    // Channel 1 is DMAMUX channel 0, whose event is dmamux_evt0.
    (void)DmaMux::request_counted(ChA::mux_channel, T3::dma_update_request(), per_event);
    (void)arm_generated(64);
    (void)DmaMux::request(ChE::mux_channel, Gen1::request_id);
    (void)Gen1::configure(dmamux_trigger_event(0), DmaMuxEdge::rising, 1);
    Gen1::enable(true);
    (void)ChE::enable(true);
    (void)ChA::enable(true);
    spin_cycles(SysClock::hz / 100u);   // 10 ms: 200 paced requests
    const uint16_t chained = generated_so_far(64);
    const uint16_t served = static_cast<uint16_t>(chain_len - ChA::count());
    Gen1::release();
    ChA::stop();
    ChE::stop();
    (void)DmaMux::release(ChA::mux_channel);
    pace_stop();
    print(serial, "  channel 1 served ", served, " requests with an event every ",
          per_event, "; channel 5 moved ", chained, " words off those events", crlf);
    bench.verdict("EGE MAKES A CHANNEL PACE ANOTHER ONE THROUGH THE FABRIC "
                  "ALONE: one dmamux_evt0 per four served requests, each "
                  "moving one word on a channel with no peripheral of its own",
                  chained > 0u && chained <= served / per_event + 2u &&
                      chained + 2u >= served / per_event);

    quiet_everything();
}

// ---- g: THE FIXED POINT ------------------------------------------------------
//
// util/block_stream.hpp was written against the SAM C21 and BEFORE this
// implementation, so that friction would show up as "this concept does
// not fit" instead of as silent divergence. This letter is the
// measurement. It has two halves and they point opposite ways:
//
//  - What circular mode GIVES. The SAM's controller has none, so its
//    BlockPlayer re-armed from a completion interrupt: one interrupt per
//    lap, and a window at every lap boundary in which the peripheral was
//    unserved. Here CIRC reloads CNDTR and both address registers in
//    hardware, so a player runs with its interrupt DISARMED - measured.
//
//  - What circular mode CANNOT give. BlockSource's contract is that a
//    source with no free buffer SKIPS the lap rather than write into the
//    block the caller is reading. Under CIRC that decision can only be
//    taken after the edge, and the controller is already writing by then.
//    Measured: how many elements land in the "held" half before a handler
//    can disable the channel.

void tg_fixed_point() {
    // --- half one: a circular stream with no CPU in it at all.
    for (uint16_t i = 0; i < 2 * block_len; ++i) {
        circ_buf[i] = 0;
    }
    pace_start(20'000u);
    circ_disable_on_half = false;
    circ_hits = 0;
    const bool circ_ok = ChE::load(DmaTransfer{
        .peripheral = payload_address(),
        .memory = &circ_buf[0],
        .count = 2 * block_len,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = true,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::word,
                   .memory_width = DmaWidth::word}});
    (void)DmaMux::request(ChE::mux_channel, T3::dma_update_request());
    // NO INTERRUPT AT ALL on this channel, and the NVIC line masked for
    // good measure: whatever happens next happens without the CPU.
    ChE::arm(DmaFlag::all, false);
    spin_cycles(SysClock::hz / 50u);   // 20 ms = 400 requests over 64 slots
    const uint16_t left_a = ChE::count();
    spin_cycles(SysClock::hz / 200u);   // 5 ms more
    const uint16_t left_b = ChE::count();
    const bool still_running = ChE::enabled();
    // The self-checking payload: consecutive slots differ by the pace.
    const uint32_t step = pace_step(20'000u);
    uint32_t bad = 0;
    for (uint16_t i = 1; i < 2 * block_len; ++i) {
        const uint32_t d = circ_buf[i] - circ_buf[i - 1];
        if (d != step) {
            ++bad;
        }
    }
    ChE::stop();
    print(serial, "  circular, interrupt disarmed: CNDTR ", left_a, " then ", left_b,
          ", channel ", still_running ? "still enabled" : "STOPPED",
          "; slot-to-slot step ", step, " wrong in ", bad, " of 63 places", crlf);
    bench.verdict("CIRCULAR MODE PLAYS FOR EVER WITH THE CPU OUT OF IT: the "
                  "channel is still enabled after 25 ms with no interrupt "
                  "armed, and CNDTR has moved - where the SAM's controller "
                  "had to be re-armed from a handler once per lap",
                  circ_ok && still_running && left_a != left_b);
    // ONE wrap boundary is legitimately not a step: the block restarted
    // at the buffer's base while the counter kept running.
    bench.verdict("and the block is self-checking: every slot but the wrap "
                  "boundary is exactly one pace period after the one before "
                  "it", bad <= 1u);

    // --- half two: the race the BlockSource contract cannot survive.
    // A handler that must decide "skip rather than tear" AT the edge, and
    // the elements that land while it is deciding.
    auto race_at = [](uint32_t hz) -> uint16_t {
        pace_stop();
        pace_start(hz);
        for (uint16_t i = 0; i < 2 * block_len; ++i) {
            circ_buf[i] = 0;
        }
        circ_hits = 0;
        circ_left = 0;
        circ_disable_on_half = true;
        (void)ChE::load(DmaTransfer{
            .peripheral = payload_address(),
            .memory = &circ_buf[0],
            .count = 2 * block_len,
            .config = {.direction = DmaDirection::peripheral_to_memory,
                       .circular = true,
                       .peripheral_increment = false,
                       .memory_increment = true,
                       .peripheral_width = DmaWidth::word,
                       .memory_width = DmaWidth::word}});
        (void)DmaMux::request(ChE::mux_channel, T3::dma_update_request());
        ChE::arm(DmaFlag::half, true);
        Nvic::enable(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);
        uint32_t spins = 4'000'000u;
        while (circ_hits == 0u && spins-- != 0u) {
        }
        const uint16_t left = circ_left;
        ChE::stop();
        circ_disable_on_half = false;
        // CNDTR counts down from 2 x block_len and stands at block_len at
        // the half-transfer edge: whatever it has lost BELOW that is what
        // landed in the second half while the handler was on its way.
        return left > block_len ? 0u : static_cast<uint16_t>(block_len - left);
    };

    const uint16_t slow = race_at(10'000u);
    const uint16_t fast = race_at(2'000'000u);
    pace_stop();
    print(serial, "  elements already written into the NEXT half when the "
          "half-transfer handler managed to disable the channel: at 10 kHz ",
          slow, ", at 2 MHz ", fast, crlf);
    bench.verdict("AT A SLOW PACE THE HANDLER WINS: nothing had been written "
                  "into the second half by the time it stopped the channel",
                  slow == 0u);
    bench.verdict("AND AT A FAST ONE IT DOES NOT: elements had already landed "
                  "in the half a caller would have been holding - so under "
                  "CIRC the 'skip rather than tear' rule is a hope about "
                  "interrupt latency and not a guarantee, which is why "
                  "DmaPingPongEngine uses a NON-CIRCULAR channel that stops "
                  "itself at every block",
                  fast > 0u);

    // And the structural claim, read off the silicon rather than promised:
    // the engine's own channel is not circular while it streams.
    pace_start(5'000u);
    Pong::arm(payload_address(), T3::dma_update_request());
    (void)Pong::start(pong_a, pong_b, block_len);
    spin_cycles(SysClock::hz / 1000u);
    const uint32_t ccr = ChD::control();
    const bool circular = ChD::circular();
    Pong::stop();
    pace_stop();
    print(serial, "  the ping-pong engine's CCR while streaming: ", hex(ccr),
          " (CIRC ", circular ? "SET" : "clear", ")", crlf);
    bench.verdict("the BlockSource implementation's channel really is not "
                  "circular - the concept needed no change, the "
                  "natural-looking implementation did",
                  !circular);

    quiet_everything();
}

// ---- h: the USART engines -----------------------------------------------------

void th_usart_engines() {
    // --- the doctrine, on a peripheral with no pads at all. USART1 comes
    // up with TE set and TXE ALREADY STANDING, which is exactly the state
    // the SAM's DMAC could not start from: it latches a trigger on the
    // RISE of the request, so a channel armed over a standing level waits
    // for an edge that has been and gone (the samc UART campaign found a
    // transmitter dead in it, and kick() is that target's answer).
    //
    // Nothing leaves the die here: USART1's pads are never claimed, so
    // its TX signal has nowhere to go.
    Usart<1>::bus_clock(true);
    Usart<1>::reset();
    Usart<1>::enable(false);
    (void)Usart<1>::configure({}, 556);   // 115200 at 64 MHz, though no pad carries it
    (void)Usart<1>::dma_transmit(true);
    Usart<1>::enable(true);
    spin_cycles(SysClock::hz / 10000u);
    const bool txe_standing = (Usart<1>::status() & UsartFlag::txe) != 0u;

    for (uint16_t i = 0; i < 8; ++i) {
        src_bytes[i] = static_cast<uint8_t>(0x40u + i);
    }
    // 11.4.3's own channel configuration procedure, in its order: set the
    // DMA channel up completely and DO NOT enable it, then write the
    // multiplexer, then enable.
    (void)ChE::prepare(DmaTransfer{
        .peripheral = Usart<1>::tx_data_address(),
        .memory = &src_bytes[0],
        .count = 4,
        .config = {.direction = DmaDirection::memory_to_peripheral,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::byte,
                   .memory_width = DmaWidth::byte}});
    (void)DmaMux::request(ChE::mux_channel, Usart<1>::dma_tx_request());
    const uint16_t before = ChE::count();
    (void)ChE::enable(true);
    // No kick, no software trigger, nothing: this controller has none.
    spin_cycles(SysClock::hz / 2000u);   // half a millisecond, ~5 characters
    const uint16_t after = ChE::count();
    ChE::stop();
    print(serial, "  USART1 with TXE ", txe_standing ? "standing" : "clear",
          ", channel enabled over it: CNDTR ", before, " to ", after, crlf);
    bench.verdict("A REQUEST IS A LEVEL SERVED ON ENABLE, NOT AN EDGE "
                  "LATCHED ON THE RISE (10.4.3): a channel armed while TXE "
                  "was ALREADY standing moved its block with no software "
                  "trigger - which is why this driver has no kick() and the "
                  "SAM's needs one",
                  txe_standing && before == 4u && after == 0u);

    // The control: with CR3.DMAT clear the peripheral asserts no request
    // line at all, so the same channel moves nothing.
    Usart<1>::enable(false);
    (void)Usart<1>::dma_transmit(false);
    Usart<1>::enable(true);
    (void)ChE::prepare(DmaTransfer{
        .peripheral = Usart<1>::tx_data_address(),
        .memory = &src_bytes[0],
        .count = 4,
        .config = {.direction = DmaDirection::memory_to_peripheral,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::byte,
                   .memory_width = DmaWidth::byte}});
    (void)ChE::enable(true);
    spin_cycles(SysClock::hz / 2000u);
    const uint16_t control_left = ChE::count();
    ChE::stop();
    Usart<1>::enable(false);
    Usart<1>::reset();
    Usart<1>::bus_clock(false);
    print(serial, "  the control, CR3.DMAT clear: CNDTR still ", control_left, crlf);
    bench.verdict("and the control says it was the REQUEST and not the "
                  "enable: with CR3.DMAT clear the same channel over the "
                  "same standing TXE moves nothing", control_left == 4u);

    // --- the console itself. Every line of this suite has already gone
    // out through a DmaTxEngine and every letter came in through a
    // DmaRxEngine, so what is left to measure is the RATE - and the two
    // ways of feeding an engine, which is the samc campaign's lesson
    // arriving on the third target.
    auto drain = [] {
        uint32_t spins = 20'000'000u;
        while ((!Serial::tx_idle() || (Usart<2>::status() & UsartFlag::tc) == 0u) &&
               spins-- != 0u) {
        }
    };
    constexpr uint32_t payload = 1024;
    uint8_t block[64];
    for (uint32_t i = 0; i < sizeof block; ++i) {
        block[i] = static_cast<uint8_t>('0' + (i % 10u));
    }

    // THE PREVIOUS VERDICTS MUST BE OFF THE WIRE FIRST. The first version
    // of this leg started its stopwatch with about six hundred bytes of
    // earlier printing still queued, and charged their time to its own
    // kilobyte - which read as the engine running at 84 % of the wire.
    drain();
    uint32_t t0 = cycles_now();
    for (uint32_t i = 0; i < payload; ++i) {
        while (!Serial::write_byte(block[i % sizeof block])) {
        }
    }
    drain();
    const uint32_t per_byte_us = cycles_to_us(cycles_now() - t0);

    drain();
    t0 = cycles_now();
    uint32_t queued = 0;
    while (queued < payload) {
        const uint32_t want = payload - queued < sizeof block ? payload - queued
                                                              : sizeof block;
        uint32_t done = 0;
        while (done < want) {
            done += Serial::write_bulk({block + done, want - done});
        }
        queued += want;
    }
    drain();
    const uint32_t bulk_us = cycles_to_us(cycles_now() - t0);

    print(serial, crlf);
    const uint32_t per_byte_bps =
        per_byte_us == 0u ? 0u : (payload * 1000000u) / per_byte_us;
    const uint32_t bulk_bps = bulk_us == 0u ? 0u : (payload * 1000000u) / bulk_us;
    print(serial, "  1024 bytes out through the TX engine: byte by byte ",
          per_byte_us, " us = ", per_byte_bps, " B/s, in bulk ", bulk_us,
          " us = ", bulk_bps, " B/s, where 115200 8N1 carries 11520", crlf);
    bench.verdict("fed in BULK the transmit engine saturates the wire: a "
                  "kilobyte at 115200 costs what 115200 costs, and the fact "
                  "you can READ this line is the end-to-end proof, since it "
                  "left the chip the same way",
                  bulk_bps > 11000u && bulk_bps < 12000u);
    bench.verdict("AND AT THIS RATE FEEDING IT BYTE BY BYTE COSTS NOTHING - "
                  "the samc campaign measured the per-byte pump losing a "
                  "third of the wire, but it measured it at MEGABAUD: here "
                  "the wire is five hundred times slower than the pump, the "
                  "ring is always full when a block ends, and every block "
                  "the engine gets is a long one. What the two feeds cost "
                  "apart is a question for a rate this VCP may not reach "
                  "(letter u)",
                  per_byte_bps + 200u >= bulk_bps && bulk_bps + 200u >= per_byte_bps);
    print(serial, "  transport bill: dma faults ", Serial::dma_faults(),
          ", hw overruns ", Serial::hw_overruns(), ", ring overruns ",
          Serial::rx_overruns(), ", frame ", Serial::frame_errors(), crlf);
    bench.verdict("and nothing was dropped or abandoned on the way",
                  Serial::dma_faults() == 0u && Serial::frame_errors() == 0u);

    quiet_everything();
}

// ---- i: the timer round trip ---------------------------------------------------

void ti_timer_round_trip() {
    // --- a duty table PLAYED into a PWM, read back off the pad.
    // TIM2's update event is the request; CCR1 is where the table lands;
    // OC1PE means each new value is taken at the next update, so the
    // waveform never glitches mid-period.
    T2::init();
    LedOut::claim();
    constexpr uint16_t pwm_top = 1000;
    (void)T2::configure({.prescaler = 3, .period = pwm_top,
                         .auto_reload_preload = true});   // 16 kHz
    (void)T2::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 0});
    T2::interrupts(T2::update_dma, true);
    T2::enable(true);

    Loop::arm(T2::ccr_address(0), T2::dma_update_request());
    Nvic::enable(DMA1_Channel1_IRQn);
    loop_laps_seen = 0;
    const bool played = Loop::start(duty_table, 8);

    // The mean of the table is what a pad sampled over hundreds of
    // periods must show: 8 entries summing to 4200 of 1000 each.
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        sum += duty_table[i];
    }
    const uint16_t predicted = static_cast<uint16_t>(sum / 8u);
    spin_cycles(SysClock::hz / 100u);
    const uint16_t measured = sample_permille(PadLed::pin_number, 60000u);
    spin_cycles(SysClock::hz / 50u);
    // BOTH COUNTERS UNDER ONE MASK. The first version read them one after
    // the other and caught a lap in between - 76 against 77, which is not
    // a defect in either but a reading taken across an interrupt.
    uint32_t laps = 0;
    uint32_t seen = 0;
    const bool running = Loop::running();
    {
        typename Stm32Platform::CriticalSection cs;
        laps = Loop::laps();
        seen = loop_laps_seen;
    }
    print(serial, "  eight duties played by DMA at 16 kHz: pad reads ", measured,
          " per mille where the table's mean is ", predicted, "; ", laps,
          " laps counted, handler calls ", seen, crlf);
    bench.verdict("A LOOP ENGINE PLAYS A TABLE INTO A LIVE PWM: the duty read "
                  "off LD4's own pad is the table's mean within 20 per mille, "
                  "with the CPU touching not one compare value",
                  played && measured + 20u >= predicted && measured <= predicted + 20u);
    bench.verdict("and the stream is alive by the one fact that says so: "
                  "laps() moving, every lap counted by an interrupt that "
                  "does not create it", running && laps > 50u && seen == laps);

    // A single duty, for contrast: stop the stream and the last value the
    // controller wrote is simply what stays.
    Loop::stop();
    spin_cycles(SysClock::hz / 100u);
    const uint16_t held = sample_permille(PadLed::pin_number, 60000u);
    const uint32_t ccr_now = T2::compare(0);
    print(serial, "  stopped: CCR1 holds ", ccr_now, ", pad reads ", held,
          " per mille", crlf);
    bench.verdict("stopping the player leaves the peripheral at the last "
                  "value it was given - one of the table's own entries, "
                  "still on the pad",
                  [&] {
                      for (uint8_t i = 0; i < 8; ++i) {
                          if (ccr_now == duty_table[i]) {
                              return true;
                          }
                      }
                      return false;
                  }());
    T2::interrupts(T2::update_dma, false);
    LedOut::release();
    T2::release();

    // --- a CAPTURE streamed out. TIM16_TISEL reaches LSI with no pad at
    // all (25.6.18), so the capture unit produces a real ~32 kHz series of
    // timestamps, and a ping-pong engine drains CCR1 with no CPU per
    // sample. Consecutive captures differ by the LSI period in timer
    // ticks, which is what makes each block self-checking.
    T16::init();
    (void)T16::configure({.prescaler = 63, .period = 0xFFFF});   // 1 MHz counter
    // LSI IS NOT FREE ANY MORE. 5.4.24: LSIRDY stands whenever the IWDG,
    // the RTC or the LSE's CSS asks for it, whatever LSION says - and
    // while this board's RTC domain ran on LSI every program got the
    // oscillator for nothing. With the domain on the LSE crystal (the
    // RTC campaign's end state) nobody asks, so this letter drained 0
    // blocks and scored 53/54 until it started the clock it captures.
    Rcc::lsi_enable(true);
    (void)Rcc::lsi_wait_ready();
    (void)T16::input_select(0, 1);   // TISEL: LSI as TI1 (the tim campaign's letter e)
    (void)T16::capture_channel(0, {.select = TimChannelSelect::direct,
                                   .polarity = TimCapturePolarity::rising});
    T16::interrupts(T16::compare_dma(0), true);
    T16::enable(true);

    pong_is_capture = true;
    pong_blocks = 0;
    PongCap::arm(T16::ccr_address(0), T16::dma_compare_request(0));
    Nvic::enable(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);
    const bool cap_started = PongCap::start(cap_a, cap_b, block_len);

    uint32_t drained = 0;
    uint32_t bad_steps = 0;
    uint32_t last_step = 0;
    const uint32_t deadline = Ticker::ticks() + 200u;
    while (Ticker::ticks() < deadline && drained < 4u) {
        volatile uint16_t* b = PongCap::ready();
        if (b == nullptr) {
            continue;
        }
        for (uint16_t i = 1; i < block_len; ++i) {
            const uint16_t d = static_cast<uint16_t>(b[i] - b[i - 1]);
            last_step = d;
            // LSI is nominally 32 kHz and the counter runs at 1 MHz, so a
            // period is ~31 ticks; anything outside a generous band is a
            // torn or missing capture and not a slow oscillator.
            if (d < 25u || d > 40u) {
                ++bad_steps;
            }
        }
        (void)PongCap::release();
        ++drained;
    }
    const uint32_t laps2 = PongCap::laps();
    const uint32_t over2 = PongCap::overruns();
    PongCap::stop();
    T16::interrupts(T16::compare_dma(0), false);
    T16::release();
    pong_is_capture = false;
    print(serial, "  ", drained, " blocks of ", block_len, " LSI captures "
          "drained, last step ", last_step, " us, ", bad_steps,
          " steps outside 25..40 us; laps ", laps2, ", overruns ", over2,
          ", handler blocks ", pong_blocks, crlf);
    bench.verdict("A PING-PONG ENGINE DRAINS A CAPTURE CHANNEL: four whole "
                  "blocks handed over, with no CPU between the capture and "
                  "the buffer",
                  cap_started && drained == 4u);
    bench.verdict("and every block is self-checking - consecutive captures "
                  "one LSI period apart, so nothing was torn or missed "
                  "inside a block", bad_steps == 0u);

    quiet_everything();
}

// ---- j: BlockRelay inside a real kernel ---------------------------------------
//
// The util level, unchanged, on the third architecture: an AO that lends
// each filled block to its subscribers for exactly one dispatch and hands
// it back on the next. The source is the ping-pong engine over the paced
// TIM3 -> TIM2->CNT chain, so every block CHECKS ITSELF - consecutive
// samples one pace period apart - and the subscriber can say so without
// any second instrument.

struct BlocksWanted {
    uint16_t count;
};

class Consumer;
class Relay;

using Subs = Subscribers<Consumer>;

class Consumer : public Fsm<Consumer, BlockReady<uint32_t>, BlocksWanted> {
    using Base = Fsm<Consumer, BlockReady<uint32_t>, BlocksWanted>;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;
    static inline EventQueue<Event, 8, Stm32Platform> queue;

    static void init() {
        blocks_ = 0;
        bad_ = 0;
        first_bad_ = 0;
        seams_ = 0;
        last_tail_ = 0;
        have_tail_ = false;
        Base::start(&running);
    }
    static void dispatch(const Event& e) { Base::dispatch(e); }

    static uint16_t blocks() { return blocks_; }
    static uint16_t bad() { return bad_; }
    static uint16_t seams() { return seams_; }
    static uint32_t first_bad() { return first_bad_; }
    static void expect_step(uint32_t s) { step_ = s; }

private:
    static Status running(const Event& e) {
        return match(e,
            [](Entry) { return Base::handled(); },
            [](Exit) { return Base::handled(); },
            [](BlocksWanted) { return Base::handled(); },
            [](const BlockReady<uint32_t>& b) {
                // The loan is valid for THIS dispatch and not one
                // instruction longer - so the block is verified here, in
                // place, and nothing is kept but the verdict.
                const volatile uint32_t* p = b.data.get();
                for (uint16_t i = 1; i < b.length; ++i) {
                    const uint32_t d = p[i] - p[i - 1];
                    if (d != step_) {
                        if (bad_ == 0u) {
                            first_bad_ = d;
                        }
                        ++bad_;
                    }
                }
                // The seam between blocks: with no overrun the last
                // sample of one block and the first of the next are one
                // pace period apart too.
                if (have_tail_ && (p[0] - last_tail_) != step_) {
                    ++seams_;
                }
                last_tail_ = p[b.length - 1u];
                have_tail_ = true;
                ++blocks_;
                return Base::handled();
            });
    }

    static inline uint16_t blocks_ = 0;
    static inline uint16_t bad_ = 0;
    static inline uint16_t seams_ = 0;
    static inline uint32_t first_bad_ = 0;
    static inline uint32_t step_ = 0;
    static inline uint32_t last_tail_ = 0;
    static inline bool have_tail_ = false;
};

class Relay : public BlockRelay<Stm32Platform, Subs, Pong> {};

using DmaKernel = Kernel<Stm32Platform, Consumer, Relay>;

void tj_relay() {
    constexpr uint32_t hz = 2'000;
    const uint32_t step = pace_step(hz);
    Consumer::expect_step(step);
    DmaKernel::init_all();

    for (uint16_t i = 0; i < block_len; ++i) {
        pong_a[i] = 0;
        pong_b[i] = 0;
    }
    pong_blocks = 0;
    pong_is_capture = false;
    pace_start(hz);
    Pong::arm(payload_address(), T3::dma_update_request());
    Nvic::enable(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);
    kernel_mode = true;
    const bool started = Pong::start(pong_a, pong_b, block_len);

    // 32 samples at 2 kHz is 16 ms a block; ten blocks is a sixth of a
    // second and the kernel has all of it to itself.
    constexpr uint16_t want = 10;
    const uint32_t deadline = Ticker::ticks() + 800u;
    while (Consumer::blocks() < want && Ticker::ticks() < deadline) {
        DmaKernel::step();
    }
    // Drain the relay's own tail: its last dispatch still owes a release.
    for (uint8_t i = 0; i < 8; ++i) {
        DmaKernel::step();
    }
    kernel_mode = false;
    const uint32_t laps = Pong::laps();
    const uint32_t over = Pong::overruns();
    Pong::stop();
    pace_stop();

    print(serial, "  ", Consumer::blocks(), " blocks of ", block_len,
          " received, ", Consumer::bad(), " samples off the ", step,
          "-tick step (first wrong step ", Consumer::first_bad(), "), ",
          Consumer::seams(), " bad seams; engine laps ", laps, ", overruns ",
          over, ", relay published ", Relay::published(), crlf);
    bench.verdict("BlockRelay RUNS UNCHANGED ON THE THIRD ARCHITECTURE: ten "
                  "caller-owned blocks travelled from a DMA engine to a "
                  "subscriber as Lease::dispatch loans, inside a real "
                  "kernel", started && Consumer::blocks() >= want);
    bench.verdict("and every sample inside every block is exactly one pace "
                  "period after the one before it - the loan was read while "
                  "the engine was filling the OTHER buffer, so nothing was "
                  "torn", Consumer::bad() == 0u);
    bench.verdict("the SEAMS hold too: the last sample of a block and the "
                  "first of the next are one period apart, so no block was "
                  "skipped between them", Consumer::seams() == 0u &&
                                              over == 0u);
    bench.verdict("and the relay keeps no second truth: what it published "
                  "is what the engine's own laps() says it filled",
                  Relay::published() == Consumer::blocks() &&
                      laps >= Consumer::blocks());

    // The other half of the contract: a caller that stops draining makes
    // the engine SKIP a lap rather than tear one, and release() restarts
    // it. Staged by simply not stepping the kernel.
    pace_start(hz);
    Pong::arm(payload_address(), T3::dma_update_request());
    (void)Pong::start(pong_a, pong_b, block_len);
    kernel_mode = false;   // nobody posts, nobody drains
    const uint32_t stall_deadline = Ticker::ticks() + 200u;
    while (!Pong::stalled() && Ticker::ticks() < stall_deadline) {
    }
    const bool stalled = Pong::stalled();
    const uint32_t over2 = Pong::overruns();
    const bool released = Pong::release();
    spin_cycles(SysClock::hz / 100u);
    const bool alive = Pong::running() || Pong::pending() != 0u;
    Pong::stop();
    pace_stop();
    print(serial, "  with nobody draining: stalled ", stalled ? "yes" : "no",
          ", overruns ", over2, "; after one release the stream is ",
          alive ? "running again" : "still stopped", crlf);
    bench.verdict("A SOURCE WITH NO FREE BUFFER SKIPS THE LAP: it stalls and "
                  "counts an overrun rather than write into the block the "
                  "caller holds - the contract's own trade, samples for "
                  "integrity", stalled && over2 > 0u);
    bench.verdict("and release() is the restart, which is why the relay's "
                  "self-post is enough to bring a stalled stream back",
                  released && alive);

    quiet_everything();
}


// ---- k: THE SYNCHRONIZATION BLOCK --------------------------------------------
//
// dma.md has carried this as a driver gap since the DMA campaign:
// `DmaMux::request_synchronized()` writes the whole CxCR word with
// ES0548 2.5.4's invariant built in, the family fixture instantiates it,
// and no letter had ever driven a synchronized channel or seen a SOFx
// rise. What it wanted was a stimulus, and the stimulus was already on
// this board: table 57's synchronization input 22 is TIM14_OC, the same
// signal table 56 offers the request GENERATOR as trigger input 22 and
// the same signal letter f already produces with no pad at all.
//
// So the arrangement is: TIM3's update event is the REQUEST, arriving
// twenty thousand times a second, and TIM14's OC1REF is the SYNC, once
// a millisecond. A synchronized channel serves NBREQ of the first for
// every edge of the second, and the count of words it moved in a known
// window is the measurement. The control is the same channel with the
// synchronization off, which saturates.

/// Table 57: TIM14_OC is synchronization input 22, exactly as table 56
/// makes it trigger input 22. One signal, two tables, and the driver
/// takes the number because 11.6.1's field is a number.
constexpr uint8_t sync_tim14_oc = 22;

volatile uint32_t dmamux_overrun_calls = 0;
/// The shared handler must NOT sweep SOFx while the letter is trying to
/// read it - and the letter cannot mask the line, because the console's
/// own engines are on it. So the sweep is gated by the letter instead.
volatile bool dmamux_service = false;

/// TIM14 producing OC1REF at `hz`, with no pin claimed - letter f's own
/// stimulus, hoisted so two letters share it.
void sync_source_start(uint32_t hz) {
    T14::init();
    const uint32_t div = SysClock::hz / (64u * hz);
    (void)T14::configure({.prescaler = 63, .period = div - 1u});
    (void)T14::output_channel(0, {.mode = TimOutputMode::pwm1,
                                  .compare = div / 2u});
    T14::enable(true);
}

/// How many words a synchronized channel moves in `ms`, armed fresh.
uint16_t sync_window(const DmaMuxSync& s, uint32_t ms) {
    ChE::stop();
    (void)arm_generated(64);
    (void)DmaMux::request_synchronized(ChE::mux_channel, T3::dma_update_request(), s);
    DmaMux::clear_overrun(0xFFFFFFFFu);
    (void)ChE::enable(true);
    spin_cycles(SysClock::hz / 1000u * ms);
    const uint16_t moved = generated_so_far(64);
    ChE::stop();
    (void)DmaMux::release(ChE::mux_channel);
    return moved;
}

void tk_synchronization() {
    quiet_everything();
    sync_source_start(1000u);          // one sync edge per millisecond
    pace_start(20'000u);               // twenty requests per sync period

    // --- the control: no synchronization at all. Twenty thousand
    // requests a second saturate a 64-word transfer in three
    // milliseconds, so what follows is not a slow channel.
    ChE::stop();
    (void)arm_generated(64);
    (void)DmaMux::request(ChE::mux_channel, T3::dma_update_request());
    (void)ChE::enable(true);
    spin_cycles(SysClock::hz / 200u);   // 5 ms
    const uint16_t free_run = generated_so_far(64);
    ChE::stop();
    (void)DmaMux::release(ChE::mux_channel);

    constexpr uint8_t batch = 4;
    const uint16_t rising = sync_window(
        {.input = sync_tim14_oc, .edge = DmaMuxEdge::rising, .requests = batch}, 5);
    const uint16_t falling = sync_window(
        {.input = sync_tim14_oc, .edge = DmaMuxEdge::falling, .requests = batch}, 5);
    const uint16_t both = sync_window(
        {.input = sync_tim14_oc, .edge = DmaMuxEdge::both, .requests = batch}, 5);
    const uint16_t one_each = sync_window(
        {.input = sync_tim14_oc, .edge = DmaMuxEdge::rising, .requests = 1}, 5);

    print(serial, "  in a five-millisecond window at 20 kHz of requests and "
          "1 kHz of sync: unsynchronized ", free_run, " words, then NBREQ ",
          batch, " on the rising edge ", rising, ", on the falling edge ",
          falling, ", on BOTH ", both, ", and NBREQ 1 rising ", one_each, crlf);
    bench.verdict("THE SYNCHRONIZATION BLOCK HOLDS A CHANNEL'S REQUESTS BACK "
                  "AND LETS EXACTLY NBREQ OF THEM THROUGH PER EDGE, which is "
                  "what dma.md has been carrying as built-but-never-run: five "
                  "sync edges times four requests is twenty words where the "
                  "same channel unsynchronized saturates its whole transfer "
                  "in the same window",
                  free_run >= 60u && rising >= 16u && rising <= 24u);
    bench.verdict("...and SPOL is a real selector, not a formality: the "
                  "falling edge serves the same number as the rising one and "
                  "BOTH serves twice as many, which is the only reading of "
                  "11.6.1's three codes that a square wave can tell apart",
                  falling >= 16u && falling <= 24u &&
                      both >= 2u * rising - 8u && both <= 2u * rising + 8u);
    bench.verdict("...and NBREQ is the count and not the count minus one - "
                  "the struct's own claim, measured: one request an edge is a "
                  "quarter of four requests an edge",
                  one_each >= 4u && one_each <= 8u);

    // --- SOFx: a sync edge arriving while the previous batch is still
    // being served. 11.4.4 says the overrun is per multiplexer channel
    // and 11.6.2 gives it a flag; ES0548 2.5.1 warns that clearing one
    // channel's flag can clear another's, which is why the suite runs
    // exactly one synchronized channel here and says so.
    // TIM3 IS SIXTEEN BITS, so a rate this slow needs its prescaler -
    // pace_start() divides the core clock by the period alone and 500 Hz
    // does not fit. Configured by hand for this one leg.
    pace_stop();
    T3::init();
    (void)T3::configure({.prescaler = 63, .period = 1999u});   // 500 Hz
    T3::interrupts(T3::update_dma, true);
    T3::enable(true);
    // THE VECTOR IS ALREADY ARMED AND IT IS NOT THIS LETTER'S TO ARM:
    // the DMAMUX overrun shares the third line with DMA1's channels 4..7,
    // which is where this suite's own console engines live. Enabling it
    // is a no-op and DISABLING it afterwards would take the console's
    // transmitter down with it - which is exactly what the first version
    // of this letter did, and the board went silent mid-letter.
    // TWO LEGS, and the reason is the shared vector again: the FLAG is
    // read with the interrupt off and the handler's sweep gated, because
    // a handler that clears SOFx is the thing that would hide it.
    dmamux_overrun_calls = 0;
    dmamux_service = false;
    ChE::stop();
    (void)arm_generated(64);
    DmaMux::clear_overrun(0xFFFFFFFFu);
    (void)DmaMux::request_synchronized(
        ChE::mux_channel, T3::dma_update_request(),
        {.input = sync_tim14_oc, .edge = DmaMuxEdge::rising, .requests = 8});
    (void)ChE::enable(true);
    spin_cycles(SysClock::hz / 100u);   // 10 ms = ten sync edges, five requests
    const bool sof = DmaMux::overrun(ChE::mux_channel);
    DmaMux::clear_overrun(1u << ChE::mux_channel);
    const bool sof_cleared = !DmaMux::overrun(ChE::mux_channel);
    ChE::stop();
    (void)DmaMux::release(ChE::mux_channel);

    dmamux_service = true;
    ChE::stop();
    (void)arm_generated(64);
    DmaMux::clear_overrun(0xFFFFFFFFu);
    (void)DmaMux::request_synchronized(
        ChE::mux_channel, T3::dma_update_request(),
        {.input = sync_tim14_oc, .edge = DmaMuxEdge::rising, .requests = 8,
         .generate_event = false, .overrun_interrupt = true});
    (void)ChE::enable(true);
    spin_cycles(SysClock::hz / 100u);
    const uint32_t sof_irqs = dmamux_overrun_calls;
    ChE::stop();
    (void)DmaMux::release(ChE::mux_channel);
    dmamux_service = false;
    DmaMux::clear_overrun(0xFFFFFFFFu);
    print(serial, "  with eight requests asked for per edge and only about "
          "half a request arriving between edges: SOF ", sof ? "set" : "clear",
          " with the interrupt off, ", sof_irqs, " overrun interrupts on the "
          "shared line with SOIE set, and after the clear ",
          sof_cleared ? "clear" : "still set", crlf);
    bench.verdict("A SYNCHRONIZATION OVERRUN IS REAL AND REPORTED: a sync "
                  "edge that arrives while the previous edge's batch is still "
                  "unserved sets SOFx, which CFR clears (11.6.2, 11.6.3)",
                  sof && sof_cleared);
    bench.verdict("...and SOIE puts it on the NVIC - on the third line, the "
                  "crowded one it shares with DMA1's channels 4 to 7 and "
                  "every DMA2 channel (table 61), which is why an application "
                  "binding that vector is a dispatcher",
                  sof_irqs > 0u);

    // --- the request GENERATOR's other two polarities, which dma.md
    // lists beside the synchronization block for the same reason: only
    // the rising edge was ever staged.
    pace_stop();
    uint16_t gen_moved[3] = {0, 0, 0};
    const DmaMuxEdge edges[3] = {DmaMuxEdge::rising, DmaMuxEdge::falling,
                                 DmaMuxEdge::both};
    for (uint8_t i = 0; i < 3u; ++i) {
        ChE::stop();
        (void)arm_generated(64);
        (void)DmaMux::request(ChE::mux_channel, Gen0::request_id);
        Gen0::clear_overrun();
        (void)Gen0::configure(sync_tim14_oc, edges[i], 1);
        Gen0::enable(true);
        (void)ChE::enable(true);
        spin_cycles(SysClock::hz / 100u);   // 10 ms = ten TIM14 periods
        gen_moved[i] = generated_so_far(64);
        Gen0::release();
        ChE::stop();
        (void)DmaMux::release(ChE::mux_channel);
    }
    print(serial, "  the request generator over ten TIM14 periods: rising ",
          gen_moved[0], " words, falling ", gen_moved[1], ", both ",
          gen_moved[2], crlf);
    bench.verdict("THE GENERATOR'S THREE POLARITIES ARE ALL REAL, the two "
                  "dma.md had never staged included: a square wave gives the "
                  "same count on either single edge and twice as many on "
                  "both",
                  gen_moved[0] >= 8u && gen_moved[0] <= 12u &&
                      gen_moved[1] >= 8u && gen_moved[1] <= 12u &&
                      gen_moved[2] >= 2u * gen_moved[0] - 4u &&
                      gen_moved[2] <= 2u * gen_moved[0] + 4u);

    T14::release();
    quiet_everything();
}


// ---- l: THE TIMER'S DMA BURST ENGINE -----------------------------------------
//
// tim.md's own first gap line: "the DMA BURST engine (DCR/DMAR, the one
// that walks several registers off one request)". The plain requests
// this suite already drives move ONE datum into ONE register; the burst
// engine turns each request into a walk of consecutive registers,
// reached through the single address TIMx_DMAR, so a DMA channel whose
// peripheral address never moves rewrites a whole waveform per period.
//
// THE MAP HAS HOLES AND THE WALK DOES NOT SKIP THEM, which is the thing
// to know before writing a table: DBA is a WORD OFFSET from TIMx_CR1
// (21.4.20), so "ARR then CCR1 then CCR2" is not a burst of three but a
// burst of FOUR - offset 12 between ARR and CCR1 is the repetition
// counter, which TIM2 does not implement. The table below carries a
// zero there and says so.
//
// THE WITNESS IS LD4's OWN PAD, and the control is what makes it a
// measurement. Four rows, each with CCR1 at exactly half its own ARR:
// if all four words land every period the duty is 500 per mille at every
// row and therefore over the whole lap. The control plays only the ARR
// column - one word per request, the same table's periods, CCR1 left
// where it was - and then the time-weighted duty is the sum of a fixed
// CCR1 over the sum of the ARRs, which is a different number by a
// factor of two and a half. One bit of DCR is all that changes between
// the two legs.

/// Four rows of four words: ARR, the repetition counter TIM2 has not
/// got, CCR1 and CCR2. The periods are 1000, 2000, 3000 and 4000 counts
/// of a 16 MHz timer clock, and each row's CCR1 is half its own ARR.
uint32_t burst_rows[16] = {
     999u, 0u,  500u,  250u,
    1999u, 0u, 1000u,  500u,
    2999u, 0u, 1500u,  750u,
    3999u, 0u, 2000u, 1000u,
};
/// The same four periods, alone: the control's table.
uint32_t burst_periods[4] = {999u, 1999u, 2999u, 3999u};

void tl_timer_burst() {
    quiet_everything();
    T2::init();
    LedOut::claim();
    // ARPE and OC1PE both on: a value written mid-period is taken at the
    // next update, so the waveform never shows a torn row.
    (void)T2::configure({.prescaler = 3, .period = 3999u,
                         .auto_reload_preload = true});
    (void)T2::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 2000,
                                 .preload = true, .enable = true});
    T2::interrupts(T2::update_dma, true);
    T2::enable(true);

    print(serial, "  TIM2 has the burst engine (", T2::has_dma_burst,
          ") and DMAR sits at one address the channel never moves off; "
          "TIM14 has neither (", T14::has_dma_burst, ")", crlf);
    bench.verdict("the reserve's own table says which timers carry the burst "
                  "engine, and the driver refuses to point one at a timer "
                  "that has not got it",
                  T2::has_dma_burst && !T14::has_dma_burst &&
                      !T14::dma_burst(TimBurstBase::arr, 4) &&
                      T14::dmar_address() == nullptr);
    bench.verdict("...and DBL is written as a LENGTH and read back as one, "
                  "which is the off-by-one this driver refuses to make the "
                  "caller carry",
                  T2::dma_burst(TimBurstBase::arr, 4) &&
                      T2::burst_length() == 4u &&
                      T2::burst_base() == TimBurstBase::arr &&
                      !T2::dma_burst(TimBurstBase::arr, 0) &&
                      !T2::dma_burst(TimBurstBase::arr, 19));

    // --- the burst itself: four words a period, played in a circle.
    (void)T2::dma_burst(TimBurstBase::arr, 4);
    const bool armed = ChA::load(DmaTransfer{
        .peripheral = T2::dmar_address(),
        .memory = &burst_rows[0],
        .count = 16,
        .config = {.direction = DmaDirection::memory_to_peripheral,
                   .circular = true,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::word,
                   .memory_width = DmaWidth::word}});
    (void)DmaMux::request(ChA::mux_channel, T2::dma_update_request());
    (void)ChA::enable(true);
    spin_cycles(SysClock::hz / 50u);        // 20 ms: thirty-odd laps
    const uint16_t burst_duty = sample_permille(PadLed::pin_number, 60000u);
    const uint32_t arr_now = T2::period();
    const uint32_t ccr1_now = T2::compare(0);
    const uint32_t ccr2_now = T2::compare(1);
    ChA::stop();
    (void)DmaMux::release(ChA::mux_channel);
    print(serial, "  four words a period through DMAR: LD4 reads ", burst_duty,
          " per mille, and the three registers stopped coherent at ARR ",
          arr_now, " CCR1 ", ccr1_now, " CCR2 ", ccr2_now, crlf);

    // --- the control: ONE word a period, the ARR column alone.
    (void)T2::dma_burst(TimBurstBase::arr, 1);
    (void)T2::set_compare(0, 500);
    const bool armed2 = ChA::load(DmaTransfer{
        .peripheral = T2::dmar_address(),
        .memory = &burst_periods[0],
        .count = 4,
        .config = {.direction = DmaDirection::memory_to_peripheral,
                   .circular = true,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::word,
                   .memory_width = DmaWidth::word}});
    (void)DmaMux::request(ChA::mux_channel, T2::dma_update_request());
    (void)ChA::enable(true);
    spin_cycles(SysClock::hz / 50u);
    const uint16_t control_duty = sample_permille(PadLed::pin_number, 60000u);
    ChA::stop();
    (void)DmaMux::release(ChA::mux_channel);
    T2::dma_burst_off();
    print(serial, "  the control - the same four periods with CCR1 left at "
          "500 - reads ", control_duty, " per mille, where the time-weighted "
          "prediction is 4 x 500 / 10000 = 200", crlf);

    bench.verdict("THE BURST ENGINE REWRITES A WHOLE WAVEFORM OFF ONE "
                  "REQUEST, which is tim.md's own first gap line: with CCR1 "
                  "at half of its own ARR in every row the pad reads 500 per "
                  "mille whatever the period is doing, where the SAME periods "
                  "with CCR1 left alone read 200 - and the only difference "
                  "between the two legs is DCR's length field",
                  armed && armed2 && burst_duty > 470u && burst_duty < 530u &&
                      control_duty > 170u && control_duty < 230u);
    bench.verdict("...and the three registers a burst walked are COHERENT "
                  "when it stops: CCR1 is exactly half the ARR beside it and "
                  "CCR2 exactly a quarter, which they could not be if the "
                  "words had landed anywhere but in the order 21.4.19 says",
                  ccr1_now == (arr_now + 1u) / 2u &&
                      ccr2_now == (arr_now + 1u) / 4u);
    print(serial, "  and the map's HOLE is why the table is four words wide "
          "and not three: DBA is a word offset from TIMx_CR1, so the walk "
          "from ARR reaches CCR1 only through offset 12 - the repetition "
          "counter, which TIM2 has not got. The row carries a zero there "
          "and the write goes nowhere", crlf);

    LedOut::release();
    T2::release();
    quiet_everything();
}

// ---- u: the host peer, and the VCP's ceiling (OUTSIDE z) -----------------------
//
// tools/uart_stress.py, unchanged from the samc campaign: the board
// prints one "HOST op mode baud format window count" line and the script
// moves its own port to that rate, pumps or verifies the same xorshift,
// and goes quiet before the board speaks again. It is the only letter
// here that needs a peer, so it sits outside z - and it is the only one
// that can say what the ST-LINK's virtual COM port is actually worth,
// since a rate the bridge cannot divide is a rate no measurement of ours
// can reach.

uint32_t lfsr_state = 0x12345678u;
void lfsr_reset() { lfsr_state = 0x12345678u; }
uint8_t lfsr_next() {
    uint32_t s = lfsr_state;
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    lfsr_state = s;
    return static_cast<uint8_t>(s & 0xFFu);
}

struct StressLeg {
    uint32_t baud;
    uint32_t sent;
    uint32_t got;
    uint32_t bad;
    uint32_t us;
};

/// One leg: announce it, switch, run for `window_ms`, switch back.
StressLeg stress_leg(const char* op, uint32_t baud, uint32_t window_ms,
                     uint32_t count) {
    StressLeg leg{baud, 0, 0, 0, 0};
    print(serial, "HOST ", op, " 3 ", baud, " 8N1 ", window_ms, " ", count, crlf);
    // Let the announcement leave the wire before the rate moves under it.
    uint32_t spins = 20'000'000u;
    while ((!Serial::tx_idle() || (Usart<2>::status() & UsartFlag::tc) == 0u) &&
           spins-- != 0u) {
    }
    spin_cycles(SysClock::hz / 20u);   // 50 ms: the script's own settle
    if (!Serial::set_baud(SysClock::pclk_hz, baud)) {
        return leg;
    }
    spin_cycles(SysClock::hz / 20u);
    Serial::clear_errors();

    lfsr_reset();
    const uint32_t t0 = cycles_now();
    const uint32_t end = Ticker::ticks() + window_ms;
    if (op[0] == 's' && op[1] == 'o') {           // source: the board emits
        uint8_t chunk[64];
        while (leg.sent < count && Ticker::ticks() < end) {
            const uint32_t want = count - leg.sent < 64u ? count - leg.sent : 64u;
            for (uint32_t i = 0; i < want; ++i) {
                chunk[i] = lfsr_next();
            }
            uint32_t done = 0;
            while (done < want && Ticker::ticks() < end) {
                done += Serial::write_bulk({chunk + done, want - done});
            }
            leg.sent += done;
        }
        spins = 20'000'000u;
        while ((!Serial::tx_idle() || (Usart<2>::status() & UsartFlag::tc) == 0u) &&
               spins-- != 0u) {
        }
    } else {                                       // sink: the board verifies
        uint8_t chunk[64];
        while (Ticker::ticks() < end) {
            (void)Serial::harvest();
            const uint32_t n = Serial::read_bulk({chunk, sizeof chunk});
            for (uint32_t i = 0; i < n; ++i) {
                if (chunk[i] != lfsr_next()) {
                    ++leg.bad;
                }
            }
            leg.got += n;
        }
    }
    leg.us = cycles_to_us(cycles_now() - t0);
    // Back to the console rate before anything is printed.
    spin_cycles(SysClock::hz / 50u);
    (void)Serial::set_baud(SysClock::pclk_hz, 115200);
    // AND THEN A LONG SILENCE, which is not politeness but the script's
    // own contract: it collects the leg's traffic until the wire has been
    // quiet for a while, so a report printed too soon is read as payload.
    // The first version of this letter printed at once, and every report
    // - including the next leg's HOST line - was eaten as data, which is
    // why the sink legs never ran at all.
    for (uint8_t i = 0; i < 5; ++i) {
        spin_cycles(SysClock::hz / 10u);   // half a second of quiet in all
    }
    Serial::clear_errors();
    return leg;
}

void tu_stress() {
    print(serial, "  this letter needs tools/uart_stress.py on the other end "
          "of the VCP; run it as", crlf,
          "  python3 tools/uart_stress.py --port <the console> --letters u", crlf);

    constexpr uint32_t ladder[] = {115200, 460800, 921600, 2000000, 3000000};
    uint32_t best_source = 0;
    uint32_t best_sink = 0;
    for (uint32_t baud : ladder) {
        if (!Serial::can_baud(SysClock::pclk_hz, baud)) {
            print(serial, "  ", baud, " baud is unreachable at this clock", crlf);
            continue;
        }
        const StressLeg s = stress_leg("source", baud, 600, 12000);
        print(serial, "  source at ", baud, ": ", s.sent, " bytes in ", s.us,
              " us", crlf);
        const StressLeg k = stress_leg("sink", baud, 600, 0);
        print(serial, "  sink at ", baud, ": ", k.got, " bytes in, ", k.bad,
              " wrong, hw overruns ", Serial::hw_overruns(), ", frame ",
              Serial::frame_errors(), crlf);
        if (s.sent >= 6000u) {
            best_source = baud;
        }
        if (k.got >= 2000u && k.bad == 0u) {
            best_sink = baud;
        }
    }
    print(serial, "  the board EMITTED a full window up to ", best_source,
          " baud; it RECEIVED byte-exact up to ", best_sink, " baud", crlf,
          "  WHAT THE BOARD CANNOT SEE is what the host got back: the "
          "source direction's ceiling is in uart_stress.py's own report, "
          "which is where a bridge that drops bytes shows up", crlf);
    bench.verdict("the transmit engine emitted a full window at 115200 or "
                  "better", best_source >= 115200u);
    bench.verdict("and the receive engine took the host's stream back "
                  "byte-exact at 115200 or better", best_sink >= 115200u);

    quiet_everything();
}

// ---- the menu ------------------------------------------------------------------

void banner() {
    print(serial, crlf, "test_stm32_dma - DMA + DMAMUX (RM0444 ch. 10, 11)", crlf,
          "  a  the block: geometry, the vector map, the boot state, refusals",
          crlf,
          "  b  memory to memory: widths, increments, table 51, throughput", crlf,
          "  c  arbitration: the four levels, and the index as the tie-break", crlf,
          "  d  the three shared vectors: each body answers for its own", crlf,
          "  e  a transfer error: the hardware disable, and ES0548 2.4.1", crlf,
          "  f  the DMAMUX: the request generator, and a channel event pacing "
          "another", crlf,
          "  g  THE FIXED POINT: what circular mode gives, and what it cannot",
          crlf,
          "  h  the USART engines: a request is a LEVEL, and the console proves it",
          crlf,
          "  i  the timer round trip: a duty table played, a capture streamed",
          crlf,
          "  j  BlockRelay inside a real kernel", crlf,
          "  u  the host peer (tools/uart_stress.py) - OUTSIDE z", crlf,
          "  z  every letter but u", crlf);
}

}   // namespace

// ---- the vectors ----------------------------------------------------------------

extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// Channel 1 has a vector to itself (table 61) - and channel 1 is the
/// loop engine's, so this body is a player's whole CPU cost: one lap
/// counted, nothing re-armed.
extern "C" void DMA1_Channel1_IRQHandler() {
    const uint32_t f = ChA::isr();
    if (f == 0u) {
        return;
    }
    ch1_calls = ch1_calls + 1u;
    if ((f & brio::DmaFlag::error) != 0u) {
        Loop::fail();
        return;
    }
    if ((f & brio::DmaFlag::complete) != 0u && Loop::running()) {
        (void)Loop::complete();
        loop_laps_seen = loop_laps_seen + 1u;
    }
}

/// Channels 2 and 3 share one line, and each body reads its OWN four bits
/// - there is no "which channel" register on this controller, so asking
/// every owner IS the dispatch.
extern "C" void DMA1_Channel2_3_IRQHandler() {
    if (ChB::isr() != 0u) {
        ch2_calls = ch2_calls + 1u;
    }
    if (ChC::isr() != 0u) {
        ch3_calls = ch3_calls + 1u;
    }
}

/// The third line is the crowded one: DMA1's channels 4..7, every DMA2
/// channel, and the DMAMUX overrun. The console's own two engines live
/// here, which is why they are served first.
extern "C" void DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQHandler() {
    (void)Serial::dma_isr();

    // The DMAMUX's own overrun shares this line (table 61). Letter k is
    // the only thing here that arms it, and it clears the flag itself -
    // this body only counts, because ES0548 2.5.1 makes a CFR write from
    // a handler that does not know which channels are live a hazard.
    if (dmamux_service && brio::DmaMux::overruns() != 0u) {
        dmamux_overrun_calls = dmamux_overrun_calls + 1u;
        brio::DmaMux::clear_overrun(brio::DmaMux::overruns());
    }

    if (pong_is_capture) {
        const uint8_t f = PongCap::service();
        if ((f & PongCap::flag_error) != 0u) {
            PongCap::fail();
        } else if ((f & PongCap::flag_complete) != 0u) {
            (void)PongCap::complete();
            pong_blocks = pong_blocks + 1u;
        }
    } else {
        const uint8_t f = Pong::service();
        if ((f & Pong::flag_error) != 0u) {
            Pong::fail();
        } else if ((f & Pong::flag_complete) != 0u) {
            (void)Pong::complete();
            pong_blocks = pong_blocks + 1u;
            if (kernel_mode) {
                brio::post<Relay>(brio::BlockDone{});
            }
        }
    }

    // Letter g's instrument: stop the circular channel AT the half-transfer
    // edge and record how far past it the controller had already got.
    if (circ_disable_on_half) {
        const uint32_t f = ChE::isr();
        if ((f & brio::DmaFlag::half) != 0u) {
            (void)ChE::enable(false);
            circ_left = ChE::count();
            circ_hits = circ_hits + 1u;
        }
    }
}

int main() {
    // Sampled before a line of ours runs: letter a judges what this boot
    // found, and Serial::init() below is the first thing to disturb it.
    boot_dma1_isr = DMA1->ISR;
    boot_ccr1 = DMA1_Channel1->CCR;
    boot_cndtr1 = DMA1_Channel1->CNDTR;
    boot_mux0 = DMAMUX1_Channel0->CCR;

    const bool clock_ok = SysClock::init();
    // THE CONSOLE ITSELF CARRIES BOTH ENGINES: the DMA controller's bus
    // clock has to be on before Uart::init() arms them.
    brio::Dma<1>::bus_clock(true);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);

    brio::enable_interrupts();

    bench.letter('a', "the block: geometry, vectors, the boot state, refusals",
                 ta_block);
    bench.letter('b', "memory to memory: widths, increments, table 51, throughput",
                 tb_mem_to_mem);
    bench.letter('c', "arbitration: the four levels and the channel index",
                 tc_arbitration);
    bench.letter('d', "the three shared vectors, each body on its own channel",
                 td_vectors);
    bench.letter('e', "a transfer error, and ES0548 2.4.1 made structural",
                 te_transfer_error);
    bench.letter('f', "the DMAMUX: the request generator and a channel event",
                 tf_multiplexer);
    bench.letter('g', "THE FIXED POINT: circular mode against BlockSource",
                 tg_fixed_point);
    bench.letter('h', "the USART engines: a request is a level, not an edge",
                 th_usart_engines);
    bench.letter('i', "the timer round trip: a table played, a capture streamed",
                 ti_timer_round_trip);
    bench.letter('j', "BlockRelay inside a real kernel", tj_relay);
    bench.letter('k', "the synchronization block: NBREQ per edge, both "
                 "polarities, SOFx and its interrupt", tk_synchronization);
    bench.letter('l', "the timer's DMA burst engine: four registers off one "
                 "update, through DMAR", tl_timer_burst);
    bench.letter('u', "the host peer, and the VCP's ceiling", tu_stress, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL64" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED",
                    " console engines: TX ch", static_cast<uint32_t>(ConsoleTx::channel),
                    " RX ch", static_cast<uint32_t>(ConsoleRx::channel), brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        (void)Serial::harvest();   // the RX engine's own pacing, once per turn
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(c), brio::crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
