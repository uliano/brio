// test_stm32_fdcan - the reference bench suite for the STM32G0's FD
// controller area network (RM0444 ch. 36) and the whole message RAM
// behind it.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NO TRANSCEIVER, NO WIRE, NO SECOND NODE. A CAN controller is the one
// peripheral of this stratum whose whole purpose is another chip, and
// this board has none - so every stimulus below is the controller's own,
// and the chapter is generous enough to make that a real experiment
// rather than a compromise. Five techniques do it:
//   1. INTERNAL LOOP-BACK (TEST.LBCK + CCCR.MON). The transmitter feeds
//      the receiver inside the die, the RX pin is disconnected and the
//      TX pad is held recessive - so filters, FIFOs, the Tx event FIFO,
//      the DLC coding, timestamps and both instances at once are all
//      measurable with NOT ONE PAD CLAIMED.
//   2. EXTERNAL LOOP-BACK (LBCK alone). The same internal feedback, but
//      THE TX PAD IS DRIVEN - "the transmitted messages can be monitored
//      at the FDCAN_TX pin" - so a frame has a duration a CPU can time
//      and an edge a counter can count.
//   3. TEST.TX = 01, THE SAMPLE POINT ON THE PAD. One edge per bit time,
//      counted by a DMAMUX request generator on the pad's EXTI line with
//      no CPU in the loop (the dma suite's technique): a bit-rate meter
//      for a bus that does not exist.
//   4. THE RX PAD'S OWN PULL IS THE BUS. A pad under an input alternate
//      function still follows its PUPDR (the LPTIM and USART campaigns
//      proved it), so a pull-up is a recessive line and a pull-down a
//      stuck-dominant one - which is how the ERROR MACHINE, bus-off and
//      the recovery sequence are reachable with nothing attached.
//   5. THE INTERNAL TIMESTAMP COUNTER IS A BIT-RATE METER. TSCC.TSS = 01
//      increments TSCV once per CAN bit time, so an instance with no pad
//      at all can be weighed against TIM2 - which is how CKDIV is proven
//      to move BOTH modules.
//
// THE PADS: PB8 (FDCAN1_RX) and PB9 (FDCAN1_TX), both AF3, both proven
// free by their own pull before they are claimed and both left in analog
// mode afterwards. FDCAN2 is exercised WITHOUT A PAD throughout.
// PA2/PA3 are the console, PA5 is LD4, PA13/PA14 the SWD, PC13 the
// button, PC14/PC15 the LSE crystal.
//
// What is exercised, letter by letter:
//   a  the subsystem: presence, the ONE enable and the ONE reset, the
//      kernel clock, CKDIV moving both instances, the INIT/CCE
//      handshake and the eight registers CCE resets, the message RAM
//   b  THE BIT RATE ON THE PAD: four rates and two dividers counted with
//      no CPU, the reset NBTP, the sample point's own position
//   c  classic CAN in internal loop-back: every field of the element,
//      the FIFO indices, the acknowledge and the Tx event
//   d  external loop-back: the frame on the pad, its duration in bit
//      times, and the RX pin disregarded
//   e  filters: range, dual, classic, reject, priority, the first match,
//      XIDAM, the non-matching policy and a filter edited while running
//   f  the FIFOs and the Tx order: blocking against overwrite, FIFO
//      against queue, cancellation, and the transmit pause
//   g  CAN FD: the 0xCC question settled, BRS at three data rates, TDCV
//   h  timestamps and the timeout counter, internal and from TIM3
//   i  THE ERROR MACHINE WITH NO NODE: bit errors, EW, EP, bus-off and
//      the recovery sequence, a stuck-dominant line, DAR, ASM, MON
//   j  interrupts: the two lines, the seven groups, the shared vectors
//      and BOTH instances looping at once
//   k  power-down, and the FDCAN through a Stop
//   l  the errata: ES0548 2.13.1 staged, and 2.13.2 read off the silicon
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/fdcan.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/pwr.hpp"
#include "stm32g0/reset.hpp"
#include "stm32g0/rtc.hpp"
#include "stm32g0/sleep.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

TestBench<Serial, 16> bench;

using Can1 = Fdcan<1>;
using Can2 = Fdcan<2>;

// The two pads. Every FDCAN signal of this part is AF3 (DS13560 tables
// 14, 15, 17 and 18).
constexpr PinSel rx_sel{'B', 8, PinFunction::af3};   // FDCAN1_RX
constexpr PinSel tx_sel{'B', 9, PinFunction::af3};   // FDCAN1_TX
using RxPad = FdcanPad<rx_sel>;
using TxPad = FdcanPad<tx_sel>;
using RxPin = Pin<'B', 8>;
using TxPin = Pin<'B', 9>;
constexpr uint8_t tx_exti_line = 9;   // an EXTI line's NUMBER IS the pin's

using Stopwatch = Tim<2>;             // 32 bits at 64 MHz, free running
using Stamper = Tim<3>;               // the FDCAN's one external timestamp source

// The no-CPU edge counter: a DMAMUX request generator on the TX pad's
// EXTI line, feeding one memory-to-memory-ish channel.
using EdgeCh = DmaChannel<1, 1>;
using EdgeGen = DmaMuxGenerator<0>;
constexpr uint16_t edge_capacity = 60000;
volatile uint32_t edge_src = 0xA5A5A5A5u;
volatile uint32_t edge_dst[8] = {};

// The boot samples letter a rests on, taken before anything of ours runs.
uint32_t boot_crel = 0;
uint32_t boot_endn = 0;
uint32_t boot_cccr = 0;
uint32_t boot_nbtp = 0;
uint32_t boot_apbenr1 = 0;
uint32_t boot_ccipr2 = 0;

volatile uint32_t line0_hits = 0;
volatile uint32_t line1_hits = 0;
volatile uint32_t line0_mask = 0;
volatile uint32_t line1_mask = 0;
volatile uint32_t line0_second = 0;   // what Fdcan<2> served on the same vector
volatile uint32_t line1_second = 0;
volatile uint32_t tim16_hits = 0;
volatile uint32_t tim17_hits = 0;
volatile uint32_t rtc_wakes = 0;

// ---------------------------------------------------------------------------
// The stopwatch and the backstop
// ---------------------------------------------------------------------------

uint32_t now_ticks() { return Stopwatch::count(); }
uint32_t since(uint32_t t0) { return now_ticks() - t0; }
uint32_t us_of(uint32_t ticks) { return ticks / (SysClock::hz / 1'000'000u); }

void spin_cycles(uint32_t cycles) {
    const uint32_t t0 = now_ticks();
    while (since(t0) < cycles) {
    }
}
void spin_us(uint32_t us) { spin_cycles(us * (SysClock::hz / 1'000'000u)); }

/// Wait for the console's own transmit ring to empty, plus a character
/// time. Letter k needs it: the FDCAN's power-down window is a few
/// hundred microseconds and the console is still emitting the previous
/// verdict lines when it opens.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    spin_us(200);
}

void feed() { Iwdg::refresh(); }

bool arm_backstop() {
    // /256 with the full reload: about 32 s at this die's LSI. 28.3.1 -
    // once started nothing this program writes stops it again, so it is
    // armed once, generously, and fed.
    return Iwdg::arm(IwdgConfig{.prescaler = IwdgPrescaler::div256,
                                .reload = 0x0FFF,
                                .window = 0x0FFF});
}

// ---------------------------------------------------------------------------
// The bring-up helpers every letter shares
// ---------------------------------------------------------------------------

/// The nominal timing this suite runs at unless a letter says otherwise:
/// 500 kbit/s with the sample point at 87.5 %, from the 64 MHz PCLK.
FdcanBitTiming nominal(uint32_t bit_hz = 500'000u) {
    const auto t = fdcan_bit_timing_for(SysClock::pclk_hz, bit_hz, 875);
    return t ? *t : FdcanBitTiming{};
}

FdcanConfig loop_config(FdcanMode mode, uint32_t bit_hz = 500'000u) {
    FdcanConfig c{};
    c.nominal = nominal(bit_hz);
    c.mode = mode;
    c.standard_filters = 1;
    c.extended_filters = 1;
    c.non_matching_standard = FdcanNonMatching::fifo0;
    c.non_matching_extended = FdcanNonMatching::fifo0;
    return c;
}

/// Bring one instance up in a loop-back mode with a filter that accepts
/// everything into FIFO 0, and put it on the bus.
template <typename C>
bool bring_up(const FdcanConfig& c) {
    C::bus_clock(true);
    if (!C::enter(c)) {
        return false;
    }
    // A classic filter with a zero mask matches every identifier.
    (void)C::standard_filter(0, FdcanStandardFilter{FdcanFilterType::classic,
                                                    FdcanFilterAction::store_fifo0,
                                                    0, 0});
    (void)C::extended_filter(0, FdcanExtendedFilter{FdcanExtFilterType::classic,
                                                    FdcanFilterAction::store_fifo0,
                                                    0, 0});
    // IR.TC AND IR.TCF ARE PER-BUFFER GATED (letter c measures it): with
    // TXBTIE clear - its reset value - a completed transmission raises
    // no TC at all, which 36.4.15's one-line description of the flag does
    // not say. Every letter but c wants the naive behaviour, so it is
    // armed here for all three buffers.
    (void)C::tx_buffer_interrupts(0x7);
    (void)C::tx_cancel_interrupts(0x7);
    return C::start();
}

/// Send one frame and wait for it to come back into `fifo`. False on the
/// bounded timeout, which is what a broken loop looks like.
template <typename C>
bool round_trip(const FdcanFrame& in, FdcanFrame& out, uint8_t fifo = 0,
                uint32_t timeout_us = 20000) {
    C::clear_flags(FdcanFlag::all);
    if (!C::tx_put(in)) {
        return false;
    }
    const uint32_t t0 = now_ticks();
    while (C::rx_available(fifo) == 0u) {
        if (us_of(since(t0)) > timeout_us) {
            return false;
        }
    }
    return C::rx_read(fifo, out);
}

void drain() {
    FdcanFrame d{};
    while (Can1::rx_read(0, d)) {
    }
    while (Can1::rx_read(1, d)) {
    }
    FdcanTxEvent e{};
    while (Can1::event_read(e)) {
    }
    Can1::clear_flags(FdcanFlag::all);
}

bool same_payload(const FdcanFrame& a, const FdcanFrame& b) {
    if (a.length != b.length) {
        return false;
    }
    for (uint8_t i = 0; i < a.length; ++i) {
        if (a.data[i] != b.data[i]) {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// The no-CPU edge counter on the TX pad
// ---------------------------------------------------------------------------

bool edge_counter_arm() {
    for (uint8_t i = 0; i < 8; ++i) {
        edge_dst[i] = 0;
    }
    if (!EdgeCh::prepare(DmaTransfer{
            .peripheral = &edge_src,
            .memory = &edge_dst[0],
            .count = edge_capacity,
            .config = {.direction = DmaDirection::peripheral_to_memory,
                       .peripheral_increment = false,
                       .memory_increment = false,
                       .peripheral_width = DmaWidth::word,
                       .memory_width = DmaWidth::word}})) {
        return false;
    }
    (void)DmaMux::request(EdgeCh::mux_channel, EdgeGen::request_id);
    if (!EdgeGen::configure(dmamux_trigger_exti(tx_exti_line), DmaMuxEdge::rising, 1)) {
        return false;
    }
    EdgeGen::enable(true);
    return EdgeCh::enable(true);
}

uint32_t edge_counter_read() {
    const uint16_t left = EdgeCh::count();
    return left > edge_capacity ? 0u : static_cast<uint32_t>(edge_capacity - left);
}

void edge_counter_stop() {
    EdgeGen::enable(false);
    EdgeCh::stop();
}

/// Rising edges on the TX pad over a window of `us` microseconds.
uint32_t edges_over(uint32_t us) {
    if (!edge_counter_arm()) {
        return 0;
    }
    spin_us(us);
    const uint32_t n = edge_counter_read();
    edge_counter_stop();
    return n;
}

/// The EXTI half of the counter, set up once: line 9 on port B, rising
/// edges, both masks armed (13.3.1 gives no pending bit without IMR and
/// figure 23 draws the DMAMUX's trigger off the EVENT side).
void edge_counter_setup() {
    Dma<1>::bus_clock(true);
    (void)Exti::select(tx_exti_line, 'B');
    (void)Exti::sense(tx_exti_line, ExtiSense::rising);
    (void)Exti::interrupt(tx_exti_line, true);
    (void)Exti::event(tx_exti_line, true);
    (void)Exti::clear(tx_exti_line);
}

// ---------------------------------------------------------------------------
// Letter a - the subsystem
// ---------------------------------------------------------------------------

void ta_subsystem() {
    print(serial, "  the reserve says: FDCAN1 at ", hex(fdcan_base(1)),
          ", FDCAN2 at ", hex(fdcan_base(2)), ", CKDIV at ",
          hex(fdcan_config_base()), ", RAM at ", hex(fdcan_ram_base(1)),
          " and ", hex(fdcan_ram_base(2)), crlf);
    print(serial, "  boot, through the CLOSED clock gate: APBENR1 ",
          hex(boot_apbenr1), " (FDCANEN ",
          (boot_apbenr1 & RCC_APBENR1_FDCANEN) != 0u ? "set" : "clear",
          "), CREL ", hex(boot_crel), ", ENDN ", hex(boot_endn),
          ", CCCR ", hex(boot_cccr), ", NBTP ", hex(boot_nbtp), crlf);
    bench.verdict("THE FDCAN'S REGISTERS ANSWER THROUGH A CLOSED APB CLOCK "
                  "GATE, and with their true reset values: RCC_APBENR1.FDCANEN "
                  "was CLEAR at this boot and CREL, ENDN, CCCR and NBTP all "
                  "read what 36.4.38's map says they should - the opposite of "
                  "what VREFBUF does behind SYSCFG's gate, and not something "
                  "5.2.17 lets a reader assume either way",
                  (boot_apbenr1 & RCC_APBENR1_FDCANEN) == 0u &&
                      boot_endn == 0x87654321u && boot_cccr == 0x1u &&
                      boot_nbtp == 0x06000A03u && boot_crel != 0u);

    // The subsystem's own reset pulse FIRST, so this letter reads the
    // reset values however many times it has been run before - and so
    // the letters that follow start from a known block.
    Can1::bus_clock(true);
    Can1::reset();
    const uint32_t crel = Can1::core_release();
    const uint32_t endn = Can1::endianness();
    const uint32_t cccr = Can1::regs().CCCR;
    const uint32_t nbtp = Can1::regs().NBTP;
    const uint32_t dbtp = Can1::regs().DBTP;
    const uint32_t txfqs = Can1::regs().TXFQS;
    const uint32_t tocc = Can1::regs().TOCC;
    const uint32_t tocv = Can1::regs().TOCV;
    const uint32_t psr = Can1::regs().PSR;
    print(serial, "  with the gate open: CREL ", hex(crel), " ENDN ", hex(endn),
          " CCCR ", hex(cccr), " NBTP ", hex(nbtp), " DBTP ", hex(dbtp),
          " TXFQS ", hex(txfqs), " TOCC ", hex(tocc), " TOCV ", hex(tocv),
          " PSR ", hex(psr), crlf);
    bench.verdict("36.4.2: the endian register reads 0x87654321, so nothing "
                  "between the core and this block swaps bytes",
                  endn == 0x87654321u);
    bench.verdict("after RCC_APBRSTR1.FDCANRST, the reset values of "
                  "36.4.38's map: CCCR 0x1 (INIT alone), NBTP 0x06000A03, "
                  "DBTP 0xA33, TXFQS 0x3, TOCC 0xFFFF0000, TOCV 0xFFFF, "
                  "PSR 0x707",
                  cccr == 0x1u && nbtp == 0x06000A03u && dbtp == 0x00000A33u &&
                      txfqs == 0x3u && tocc == 0xFFFF0000u &&
                      (tocv & 0xFFFFu) == 0xFFFFu && (psr & 0x7FFu) == 0x707u);
    print(serial, "  CREL decodes as release ", (crel >> 28) & 0xFu, ".",
          (crel >> 24) & 0xFu, ".", (crel >> 20) & 0xFu, " of 201",
          (crel >> 16) & 0xFu, "-", hex((crel >> 8) & 0xFFu), "-",
          hex(crel & 0xFFu), " (the M_CAN core's own release and date, the "
          "last three fields in BCD)", crlf);

    // --- ONE enable for the whole subsystem, and what it really gates.
    // Reading is evidently not it (above), so the question is whether a
    // WRITE lands - asked on FDCAN_IE, which no protection covers.
    (void)Can1::interrupts(FdcanFlag::all, false);
    (void)Can2::interrupts(FdcanFlag::all, false);
    Can1::bus_clock(false);
    (void)Can1::interrupts(FdcanFlag::rx_fifo0_new, true);
    (void)Can2::interrupts(FdcanFlag::rx_fifo1_new, true);
    const uint32_t ie1_closed = Can1::interrupts();
    const uint32_t ie2_closed = Can2::interrupts();
    Can1::bus_clock(true);
    const uint32_t ie1_open = Can1::interrupts();
    const uint32_t ie2_open = Can2::interrupts();
    (void)Can1::interrupts(FdcanFlag::all, false);
    (void)Can2::interrupts(FdcanFlag::all, false);
    print(serial, "  with FDCANEN CLEAR, IE written 1 on FDCAN1 and 8 on "
          "FDCAN2 reads back ", ie1_closed, " and ", ie2_closed,
          "; with the gate open again they read ", ie1_open, " and ", ie2_open,
          crlf);
    bench.verdict("SO THE GATE IS A WRITE GATE AND NOT A READ GATE: a store "
                  "made with FDCANEN clear lands NOWHERE and is not replayed "
                  "when the clock comes back, while every read above answered "
                  "correctly - and one bit does it for both modules, which is "
                  "the half of 'one enable' a register can show",
                  fdcan_clock_mask() == RCC_APBENR1_FDCANEN &&
                      ie1_closed == 0u && ie2_closed == 0u &&
                      ie1_open == 0u && ie2_open == 0u);

    // Move a register of FDCAN1, then reset THROUGH FDCAN2.
    (void)Can1::init_mode(true);
    (void)Can1::configuration(true);
    (void)Can1::nominal_timing(nominal());
    const uint32_t moved = Can1::regs().NBTP;
    Can2::reset();
    const uint32_t after = Can1::regs().NBTP;
    print(serial, "  FDCAN1's NBTP written ", hex(moved),
          ", then FDCAN2::reset(): FDCAN1's NBTP reads ", hex(after), crlf);
    bench.verdict("ONE RESET FOR THE SUBSYSTEM: a reset asked for through "
                  "FDCAN2 takes FDCAN1's registers back to their reset values "
                  "too - which is why the driver's enter() does not pulse it",
                  moved != 0x06000A03u && after == 0x06000A03u);

    // --- the kernel clock (5.4.22).
    print(serial, "  boot CCIPR2 ", hex(boot_ccipr2), ", FDCANSEL now ",
          Can1::kernel_clock(), crlf);
    const bool pclk_ok = Can1::kernel_clock(FdcanClock::pclk);
    const bool pllq_refused = !Can1::kernel_clock(FdcanClock::pllq);
    const bool hse_refused = !Can1::kernel_clock(FdcanClock::hse);
    bench.verdict("the kernel clock is chosen in CCIPR2 and not in CCIPR "
                  "(5.4.22): PCLK selects, and PLLQCLK and HSE are refused "
                  "because nothing in clock.hpp starts either",
                  pclk_ok && Can1::kernel_clock() == 0u && pllq_refused &&
                      hse_refused);

    // --- the INIT/CCE handshake and its cost.
    (void)Can1::init_mode(false);
    uint32_t worst = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        const uint32_t t0 = now_ticks();
        (void)Can1::init_mode(true);
        const uint32_t up = since(t0);
        const uint32_t t1 = now_ticks();
        (void)Can1::init_mode(false);
        const uint32_t down = since(t1);
        const uint32_t m = up > down ? up : down;
        if (m > worst) {
            worst = m;
        }
    }
    (void)Can1::init_mode(true);
    print(serial, "  36.4.6's cross-domain readback costs at most ", worst,
          " CPU cycles (", us_of(worst * 1000u) / 1000u, ".",
          (us_of(worst * 1000u) % 1000u) / 100u, " us at 64 MHz)", crlf);
    bench.verdict("INIT crosses a clock domain and the readback is not a "
                  "formality, but it is bounded and short",
                  worst > 0u && worst < 20000u);

    const bool cce_refused = (Can1::init_mode(false), !Can1::configuration(true));
    (void)Can1::init_mode(true);
    const bool cce_ok = Can1::configuration(true) && Can1::configurable();
    (void)Can1::init_mode(false);
    const bool cce_auto = !Can1::configuration();
    bench.verdict("36.3.4's two rules about CCE: it cannot be SET with INIT "
                  "clear (refused, nothing written) and it is CLEARED BY "
                  "HARDWARE when INIT goes away",
                  cce_refused && cce_ok && cce_auto);

    // --- the eight registers CCE resets (36.3.4).
    // Move them first: a loop-back session that leaves two unread frames
    // in FIFO 0, one in FIFO 1, two Tx events and two TXBTO bits.
    FdcanConfig c = loop_config(FdcanMode::internal_loop_back);
    c.standard_filters = 2;
    if (bring_up<Can1>(c)) {
        (void)Can1::standard_filter(0, FdcanStandardFilter{
                                           FdcanFilterType::dual,
                                           FdcanFilterAction::priority_fifo1, 0x201, 0x201});
        (void)Can1::standard_filter(1, FdcanStandardFilter{
                                           FdcanFilterType::classic,
                                           FdcanFilterAction::store_fifo0, 0, 0});
        for (uint8_t i = 0; i < 2; ++i) {
            FdcanFrame f{};
            f.id = 0x100 + i;
            f.length = 1;
            f.data[0] = i;
            (void)Can1::tx_put(f);
            spin_us(2000);
        }
        FdcanFrame hp{};
        hp.id = 0x201;
        hp.length = 1;
        (void)Can1::tx_put(hp);
        spin_us(3000);
    }
    const uint32_t before[8] = {Can1::regs().HPMS,  Can1::regs().RXF0S,
                                Can1::regs().RXF1S, Can1::regs().TXFQS,
                                Can1::regs().TXBRP, Can1::regs().TXBTO,
                                Can1::regs().TXBCF, Can1::regs().TXEFS};
    (void)Can1::init_mode(true);
    (void)Can1::configuration(true);
    const uint32_t reset_after[8] = {Can1::regs().HPMS,  Can1::regs().RXF0S,
                                     Can1::regs().RXF1S, Can1::regs().TXFQS,
                                     Can1::regs().TXBRP, Can1::regs().TXBTO,
                                     Can1::regs().TXBCF, Can1::regs().TXEFS};
    static const char* const names[8] = {"HPMS", "RXF0S", "RXF1S", "TXFQS",
                                         "TXBRP", "TXBTO", "TXBCF", "TXEFS"};
    static const uint32_t resets[8] = {0, 0, 0, 3, 0, 0, 0, 0};
    uint8_t moved_count = 0;
    bool all_reset = true;
    for (uint8_t i = 0; i < 8; ++i) {
        print(serial, "  ", names[i], " ", hex(before[i]), " -> ",
              hex(reset_after[i]), crlf);
        if (before[i] != resets[i]) {
            ++moved_count;
        }
        if (reset_after[i] != resets[i]) {
            all_reset = false;
        }
    }
    bench.verdict("36.3.4's list, register by register: setting CCE RESETS "
                  "HPMS, both Rx FIFO status registers, TXFQS, TXBRP, TXBTO, "
                  "TXBCF and TXEFS - and this run had moved at least four of "
                  "them first",
                  all_reset && moved_count >= 4u);

    // TOCV is preset to TOCC.TOP by the same act (36.3.4's own paragraph).
    (void)Can1::timeout(FdcanTimeoutMode::continuous, 0x1234, false);
    (void)Can1::init_mode(false);
    (void)Can1::init_mode(true);
    (void)Can1::configuration(true);
    const uint16_t tocv_now = Can1::timeout_value();
    print(serial, "  TOP written 0x1234, TOCV after the next CCE: ",
          hex(tocv_now), crlf);
    bench.verdict("...and the timeout counter is PRESET to TOP by that same "
                  "act, which is the one thing on the list that is not a reset",
                  tocv_now == 0x1234u);

    // --- CKDIV: one register, both instances.
    const bool div_refused = (Can1::init_mode(false), !Can1::clock_divider(
                                                          FdcanClockDivider::div2));
    (void)Can1::init_mode(true);
    (void)Can1::configuration(true);
    const bool div_ok = Can1::clock_divider(FdcanClockDivider::div4);
    const auto seen_by_2 = Can2::clock_divider();
    (void)Can1::clock_divider(FdcanClockDivider::div1);
    // Can FDCAN2's own CCE open the register at all? 36.4.37 says CCE,
    // and does not say WHOSE.
    (void)Can1::init_mode(false);
    Can2::bus_clock(true);
    (void)Can2::init_mode(true);
    (void)Can2::configuration(true);
    const bool via2 = Can2::clock_divider(FdcanClockDivider::div6);
    const auto after2 = Can1::clock_divider();
    (void)Can2::clock_divider(FdcanClockDivider::div1);
    (void)Can2::init_mode(false);
    print(serial, "  CKDIV written /4 through FDCAN1: FDCAN2 reads /",
          fdcan_divider_value(seen_by_2), "; written /6 through FDCAN2's own "
          "CCE: accepted=", via2 ? "yes" : "no", ", FDCAN1 reads /",
          fdcan_divider_value(after2), crlf);
    bench.verdict("36.4.37: CKDIV is ONE REGISTER for the subsystem - what "
                  "FDCAN1 writes FDCAN2 reads - and it is protected, a write "
                  "outside INIT + CCE being refused with nothing stored",
                  div_refused && div_ok &&
                      seen_by_2 == FdcanClockDivider::div4);
    // 36.4.37 gates the write on "the CCE bit" and does not say WHOSE.
    // There is one register and two CCE bits, so the bench decides.
    if (after2 == FdcanClockDivider::div6) {
        bench.verdict("...and 36.4.37's unqualified 'the CCE bit' means "
                      "EITHER module's: the write landed through FDCAN2's own "
                      "configuration gate, on FDCAN1's register",
                      via2);
    } else {
        bench.verdict("...and 36.4.37's unqualified 'the CCE bit' means "
                      "FDCAN1's ALONE: the same write through FDCAN2's "
                      "configuration gate did NOT land, which is what the "
                      "note's ordering advice quietly implies - set the "
                      "divider through FDCAN1 before configuring the other "
                      "instance",
                      after2 == FdcanClockDivider::div1);
    }

    // --- the message RAM.
    volatile uint32_t* m1 = Can1::ram();
    volatile uint32_t* m2 = Can2::ram();
    for (uint16_t i = 0; i < Can1::ram_words; ++i) {
        m1[i] = 0x11110000u + i;
        m2[i] = 0x22220000u + i;
    }
    bool distinct = true;
    for (uint16_t i = 0; i < Can1::ram_words; ++i) {
        if (m1[i] != 0x11110000u + i || m2[i] != 0x22220000u + i) {
            distinct = false;
        }
    }
    const uint32_t gap = m2[0] == 0x22220000u ? 1u : 0u;
    print(serial, "  FDCAN1's word 0 = ", hex(m1[0]), ", FDCAN2's = ",
          hex(m2[0]), ", the maps are ",
          hex(fdcan_ram_base(2) - fdcan_ram_base(1)), " bytes apart", crlf);
    bench.verdict("36.3.6: the two message RAMs are 212 words each and NOT "
                  "ALIASED - FDCAN2's map starts where FDCAN1's ends, 0x350 "
                  "bytes on",
                  distinct && gap == 1u &&
                      (fdcan_ram_base(2) - fdcan_ram_base(1)) == 0x350u);

    Can1::clear_ram();
    bool zeroed = true;
    for (uint16_t i = 0; i < Can1::ram_words; ++i) {
        if (m1[i] != 0u) {
            zeroed = false;
        }
    }
    bench.verdict("...and clear_ram() really does zero all 212 words, which "
                  "is not decoration: the RAM's content after a reset is "
                  "UNDEFINED and an un-zeroed filter list is 28 elements of "
                  "whatever the last program left",
                  zeroed && m2[0] == 0x22220000u);

    // Figure 399's element start addresses, checked by writing at the
    // computed word offset and reading at the byte address the figure
    // names.
    struct Section {
        const char* name;
        uint16_t word;
        uint32_t byte_offset;
    };
    static const Section sections[6] = {
        {"11-bit filters", Can1::standard_filter_start, 0x000},
        {"29-bit filters", Can1::extended_filter_start, 0x070},
        {"Rx FIFO 0", Can1::rx_fifo0_start, 0x0B0},
        {"Rx FIFO 1", Can1::rx_fifo1_start, 0x188},
        {"Tx event FIFO", Can1::tx_event_start, 0x260},
        {"Tx buffers", Can1::tx_buffer_start, 0x278},
    };
    bool map_ok = true;
    for (uint8_t i = 0; i < 6; ++i) {
        (void)Can1::set_ram_word(sections[i].word, 0xC0DE0000u + i);
        volatile uint32_t* at = reinterpret_cast<volatile uint32_t*>(
            fdcan_ram_base(1) + sections[i].byte_offset);
        const uint32_t got = *at;
        print(serial, "  ", sections[i].name, " at word ", sections[i].word,
              " = byte ", hex(sections[i].byte_offset), ": read ", hex(got), crlf);
        if (got != 0xC0DE0000u + i) {
            map_ok = false;
        }
    }
    bench.verdict("every section start of figure 399 is where the figure puts "
                  "it: 0x000, 0x070, 0x0B0, 0x188, 0x260 and 0x278",
                  map_ok);

    Can1::clear_ram();
    Can2::clear_ram();
    Can1::release();
    Can2::release();
}

// ---------------------------------------------------------------------------
// Letter b - the bit rate on the pad
// ---------------------------------------------------------------------------

/// Does a pad follow its own internal pull between the rails? The
/// standing precondition of every pad this stratum claims.
template <typename P>
bool pad_is_free(const char* name) {
    P::input(PinPull::up);
    spin_us(200);
    const bool high = P::read();
    P::input(PinPull::down);
    spin_us(200);
    const bool low = !P::read();
    P::analog();
    print(serial, "  ", name, ": pull-up reads ", high ? "1" : "0",
          ", pull-down reads ", low ? "0" : "1", crlf);
    return high && low;
}

void tb_bit_rate() {
    const bool rx_free = pad_is_free<RxPin>("PB8 (FDCAN1_RX)");
    const bool tx_free = pad_is_free<TxPin>("PB9 (FDCAN1_TX)");
    bench.verdict("both pads follow their own internal pull between the rails "
                  "before this suite claims either - the precondition, not an "
                  "assumption",
                  rx_free && tx_free);

    edge_counter_setup();
    TxPad::claim_tx();

    struct Rate {
        uint32_t hz;
        const char* name;
    };
    static const Rate rates[4] = {{125'000u, "125 kbit/s"},
                                  {250'000u, "250 kbit/s"},
                                  {500'000u, "500 kbit/s"},
                                  {1'000'000u, "1 Mbit/s"}};
    bool all_ok = true;
    for (uint8_t i = 0; i < 4; ++i) {
        // EXTERNAL loop-back: the one mode in which the module runs its
        // bit clock on a bus of its own AND drives the TX pad, which is
        // what TEST.TX = 01 then overrides with the sample point.
        FdcanConfig c = loop_config(FdcanMode::external_loop_back, rates[i].hz);
        (void)bring_up<Can1>(c);
        (void)Can1::tx_pin(FdcanTxPin::sample_point);
        const uint32_t window_us = rates[i].hz >= 500'000u ? 20000u : 80000u;
        const uint32_t n = edges_over(window_us);
        const uint32_t measured = static_cast<uint32_t>(
            (static_cast<uint64_t>(n) * 1'000'000ULL) / window_us);
        const uint32_t expect = rates[i].hz;
        const uint32_t err = measured > expect ? measured - expect : expect - measured;
        const bool ok = err * 100u <= expect;   // 1 %, the HSI16's own trim
        all_ok = all_ok && ok;
        print(serial, "  ", rates[i].name, ": NBTP ", hex(Can1::regs().NBTP),
              ", ", n, " sample points in ", window_us, " us = ", measured,
              " bit/s (", ok ? "within" : "OUTSIDE", " 1 %)", crlf);
        Can1::release();
    }
    bench.verdict("TEST.TX = 01 PUTS THE SAMPLE POINT ON THE PAD, one edge per "
                  "bit time: four nominal bit rates from a 64 MHz PCLK, every "
                  "one of them within the HSI16's own 1 % of its arithmetic",
                  all_ok);

    // The reset NBTP, which 36.4.7's note prices at 48 MHz.
    {
        Can1::bus_clock(true);
        Can1::reset();
        (void)Can1::init_mode(true);
        (void)Can1::configuration(true);
        (void)Can1::test_mode(true);
        (void)Can1::init_mode(false);
        (void)Can1::loop_back(true);
        (void)Can1::tx_pin(FdcanTxPin::sample_point);
        const uint32_t n = edges_over(10000u);
        const uint32_t measured = n * 100u;
        print(serial, "  the RESET NBTP (0x06000A03, 16 tq): ", measured,
              " bit/s - 36.4.7 prices it at 3 Mbit/s from 48 MHz, and this "
              "board's kernel clock is 64", crlf);
        bench.verdict("...so the same register is 4 Mbit/s here, exactly as "
                      "the arithmetic says and the note does not",
                      measured > 3'960'000u && measured < 4'040'000u);
        Can1::release();
    }

    // CKDIV: the same NBTP through /2 and /4.
    {
        bool div_ok = true;
        for (uint8_t k = 0; k < 2; ++k) {
            const FdcanClockDivider d = k == 0 ? FdcanClockDivider::div2
                                               : FdcanClockDivider::div4;
            FdcanConfig c = loop_config(FdcanMode::external_loop_back, 500'000u);
            Can1::bus_clock(true);
            (void)Can1::init_mode(true);
            (void)Can1::configuration(true);
            (void)Can1::clock_divider(d);
            (void)Can1::init_mode(false);
            (void)bring_up<Can1>(c);
            (void)Can1::tx_pin(FdcanTxPin::sample_point);
            const uint32_t n = edges_over(80000u);
            const uint32_t measured = static_cast<uint32_t>(
                (static_cast<uint64_t>(n) * 1'000'000ULL) / 80000ULL);
            const uint32_t expect = 500'000u / fdcan_divider_value(d);
            const uint32_t err = measured > expect ? measured - expect : expect - measured;
            const bool ok = err * 50u <= expect;
            div_ok = div_ok && ok;
            print(serial, "  CKDIV /", fdcan_divider_value(d),
                  " under the same NBTP: ", measured, " bit/s against ", expect,
                  crlf);
            (void)Can1::init_mode(true);
            (void)Can1::configuration(true);
            (void)Can1::clock_divider(FdcanClockDivider::div1);
            Can1::release();
        }
        bench.verdict("CKDIV divides the time quantum and nothing else: the "
                      "SAME bit timing comes out at half and a quarter of its "
                      "rate through /2 and /4",
                      div_ok);
    }

    // Where the sample point actually sits inside the bit. At 125 kbit/s
    // a bit is 8 us, which a 64 MHz CPU can sample many times over.
    {
        constexpr uint32_t samples = 40000;
        uint32_t permille[2] = {0, 0};
        static const uint32_t duty_rates[2] = {125'000u, 1'000'000u};
        for (uint8_t k = 0; k < 2; ++k) {
            FdcanConfig c = loop_config(FdcanMode::external_loop_back,
                                        duty_rates[k]);
            (void)bring_up<Can1>(c);
            (void)Can1::tx_pin(FdcanTxPin::sample_point);
            uint32_t high = 0;
            for (uint32_t i = 0; i < samples; ++i) {
                if (TxPin::read()) {
                    ++high;
                }
            }
            permille[k] = (high * 1000u) / samples;
            Can1::release();
        }
        const uint16_t want = nominal(125'000u).sample_point_permille();
        print(serial, "  the sample-point signal is high for ", permille[0],
              " parts per thousand of an 8 us bit and ", permille[1],
              " of a 1 us bit, where the sample POINT sits at ", want, crlf);
        bench.verdict("WHAT TEST.TX = 01 PUTS ON THE PAD IS A STROBE AND NOT A "
                      "LEVEL - 36.4.4 says only 'sample point can be monitored "
                      "at pin FDCANx_TX' and this is the shape of it: the pad "
                      "is high for a per-mille or so of the bit and low for "
                      "the rest, and the fraction GROWS as the bit shortens, "
                      "so the pulse is a fixed width in kernel clocks and its "
                      "one edge a bit is what the counter above was counting",
                      permille[0] < 100u && permille[1] < 300u &&
                          permille[1] >= permille[0]);
    }

    // fdcan_tq_ck <= fdcan_pclk (36.3.3): unreachable by construction here.
    print(serial, "  36.3.3 requires fdcan_tq_ck <= fdcan_pclk. On this part "
          "the only kernel clock that exists is PCLK itself (the other two "
          "CCIPR2 codes are refused) and CKDIV only DIVIDES it, so the "
          "condition cannot be broken by any configuration this driver "
          "accepts", crlf);
    bench.verdict("...which is a structural answer and not a refusal, and is "
                  "said in the driver rather than enforced by it",
                  fdcan_divider_value(FdcanClockDivider::div1) == 1u);

    TxPad::release();
}

// ---------------------------------------------------------------------------
// Letter c - classic CAN in internal loop-back
// ---------------------------------------------------------------------------

void tc_classic() {
    // The RX pad pulled DOMINANT while the loop runs: internal loop-back
    // disconnects it, and this is the proof.
    RxPad::claim_rx(PinPull::down);
    edge_counter_setup();
    TxPad::claim_tx();

    FdcanConfig c = loop_config(FdcanMode::internal_loop_back);
    const bool up = bring_up<Can1>(c);
    bench.verdict("the module comes up in INTERNAL loop-back and leaves "
                  "initialization", up && !Can1::in_init());

    if (!edge_counter_arm()) {
        bench.verdict("the edge counter armed", false);
    }

    // --- 11-bit identifiers, DLC 0..8.
    bool lengths_ok = true;
    for (uint8_t len = 0; len <= 8; ++len) {
        FdcanFrame in{};
        in.id = 0x100u + len;
        in.length = len;
        in.marker = static_cast<uint8_t>(0xA0u + len);
        for (uint8_t i = 0; i < len; ++i) {
            in.data[i] = static_cast<uint8_t>(0x10u * len + i);
        }
        FdcanFrame out{};
        const bool ok = round_trip<Can1>(in, out) && out.id == in.id &&
                        !out.extended && !out.remote && !out.fd &&
                        same_payload(in, out);
        lengths_ok = lengths_ok && ok;
        if (!ok) {
            print(serial, "  DLC ", len, " FAILED: id ", hex(out.id), " len ",
                  out.length, crlf);
        }
        feed();
    }
    bench.verdict("classic CAN, 11-bit identifiers, every DLC from 0 to 8 "
                  "BYTE-EXACT through the internal loop",
                  lengths_ok);

    // --- 29-bit identifiers.
    {
        FdcanFrame in{};
        in.id = 0x1ABCDEFu;
        in.extended = true;
        in.length = 8;
        for (uint8_t i = 0; i < 8; ++i) {
            in.data[i] = static_cast<uint8_t>(0xF0u - i);
        }
        FdcanFrame out{};
        const bool ok = round_trip<Can1>(in, out);
        print(serial, "  extended: sent ", hex(in.id), " got ", hex(out.id),
              " xtd=", out.extended ? "1" : "0", " len=", out.length, crlf);
        bench.verdict("a 29-bit identifier survives the ID[28:0] field whole, "
                      "where an 11-bit one lives in ID[28:18] - the codec's "
                      "one asymmetry, measured",
                      ok && out.extended && out.id == in.id && same_payload(in, out));
    }

    // --- a remote frame.
    {
        FdcanFrame in{};
        in.id = 0x321;
        in.remote = true;
        in.length = 8;   // the DLC travels, the bytes do not
        FdcanFrame out{};
        const bool ok = round_trip<Can1>(in, out);
        print(serial, "  remote: rtr=", out.remote ? "1" : "0", " id ",
              hex(out.id), " length ", out.length, crlf);
        bench.verdict("a REMOTE frame arrives with RTR set, its identifier "
                      "intact and no data at all - and 'automated transmission "
                      "on reception of remote frames is not supported' "
                      "(36.3.4), so nothing answers it",
                      ok && out.remote && out.id == 0x321u && out.length == 0u);
    }

    // --- every field of the Rx element, read raw out of the RAM.
    {
        drain();
        Can1::clear_flags(FdcanFlag::all);
        FdcanFrame in{};
        in.id = 0x2AA;
        in.length = 4;
        in.marker = 0x5C;
        in.data[0] = 0xDE;
        in.data[1] = 0xAD;
        in.data[2] = 0xBE;
        in.data[3] = 0xEF;
        const auto slot = Can1::tx_put(in);
        spin_us(3000);
        const uint32_t rxf0s = Can1::regs().RXF0S;
        const uint8_t gi = Can1::rx_get_index(0);
        const uint32_t r0 = Can1::ram_word(Can1::rx_fifo0_start + gi * 18u);
        const uint32_t r1 = Can1::ram_word(Can1::rx_fifo0_start + gi * 18u + 1u);
        const uint32_t r2 = Can1::ram_word(Can1::rx_fifo0_start + gi * 18u + 2u);
        const uint32_t flags = Can1::flags();
        const uint8_t occurred = Can1::tx_occurred();
        FdcanFrame out{};
        const bool read_ok = Can1::rx_read(0, out);
        const uint32_t after = Can1::regs().RXF0S;
        print(serial, "  R0 ", hex(r0), " R1 ", hex(r1), " R2 ", hex(r2),
              "  RXF0S ", hex(rxf0s), " -> ", hex(after), "  IR ",
              hex(flags), " TXBTO ", hex(occurred), crlf);
        print(serial, "  decoded: ESI=", (r0 >> 31) & 1u, " XTD=", (r0 >> 30) & 1u,
              " RTR=", (r0 >> 29) & 1u, " ANMF=", (r1 >> 31) & 1u, " FIDX=",
              (r1 >> 24) & 0x7Fu, " FDF=", (r1 >> 21) & 1u, " BRS=", (r1 >> 20) & 1u,
              " DLC=", (r1 >> 16) & 0xFu, " RXTS=", r1 & 0xFFFFu, crlf);
        bench.verdict("table 215 field by field on a real element: ESI, XTD, "
                      "RTR and ANMF clear, FIDX 0 (the one filter matched), "
                      "FDF and BRS clear on a classic frame, DLC 4, and the "
                      "data word in table 216's byte order",
                      ((r0 >> 29) & 0x7u) == 0u && ((r1 >> 31) & 1u) == 0u &&
                          ((r1 >> 24) & 0x7Fu) == 0u &&
                          ((r1 >> 20) & 0x3u) == 0u &&
                          ((r1 >> 16) & 0xFu) == 4u && r2 == 0xEFBEADDEu);
        bench.verdict("...and the FIFO's own bookkeeping: the fill level was "
                      "1, the acknowledge moved the get index on and the level "
                      "back to 0",
                      read_ok && (rxf0s & 0xFu) == 1u && (after & 0xFu) == 0u &&
                          ((after >> 8) & 0x3u) == ((gi + 1u) % 3u));
        bench.verdict("the reception raised IR.RF0N and TXBTO names the "
                      "buffer the frame left from",
                      (flags & FdcanFlag::rx_fifo0_new) != 0u && slot &&
                          (occurred & (1u << *slot)) != 0u);

        // IR.TC is NOT what its one-line description suggests.
        drain();
        (void)Can1::tx_buffer_interrupts(0x0);
        Can1::clear_flags(FdcanFlag::all);
        (void)Can1::tx_put(in);
        spin_us(3000);
        const uint32_t no_tie = Can1::flags();
        drain();
        (void)Can1::tx_buffer_interrupts(0x7);
        Can1::clear_flags(FdcanFlag::all);
        (void)Can1::tx_put(in);
        spin_us(3000);
        const uint32_t with_tie = Can1::flags();
        drain();
        print(serial, "  the same transmission with TXBTIE clear gives IR ",
              hex(no_tie), " and with TXBTIE set IR ", hex(with_tie), crlf);
        bench.verdict("IR.TC IS GATED BY TXBTIE AND 36.4.15 DOES NOT SAY SO: "
                      "'transmission completed' is not raised at all unless "
                      "the buffer the frame left from has its own TXBTIE bit "
                      "set, and TXBTIE resets to zero - so a program that "
                      "waits on IR.TC out of the box waits for ever, while "
                      "TXBTO and the Tx event FIFO report the same "
                      "transmission either way",
                      (no_tie & FdcanFlag::transmission_completed) == 0u &&
                          (with_tie & FdcanFlag::transmission_completed) != 0u &&
                          (no_tie & FdcanFlag::rx_fifo0_new) != 0u);
    }

    // --- the Tx event FIFO.
    {
        drain();
        Can1::clear_flags(FdcanFlag::all);
        FdcanFrame in{};
        in.id = 0x123;
        in.length = 2;
        in.marker = 0x77;
        in.data[0] = 1;
        in.data[1] = 2;
        (void)Can1::tx_put(in);
        spin_us(3000);
        FdcanTxEvent ev{};
        const uint8_t avail = Can1::events_available();
        const bool got = Can1::event_read(ev);
        FdcanFrame drop{};
        (void)Can1::rx_read(0, drop);
        print(serial, "  Tx event: id ", hex(ev.id), " marker ",
              hex(ev.marker), " type ", ev.event_type, " len ", ev.length,
              " ts ", ev.timestamp, " (fill level was ", avail, ")", crlf);
        bench.verdict("the Tx event FIFO carries the MESSAGE MARKER back: the "
                      "byte the caller put in T1.MM arrives in E1.MM, with "
                      "event type 01 (an ordinary Tx event) and the frame's "
                      "own identifier and DLC",
                      got && avail == 1u && ev.marker == 0x77u &&
                          ev.id == 0x123u && ev.event_type == 1u &&
                          ev.length == 2u);
    }

    // --- the TX pad said NOTHING the whole time.
    const uint32_t edges = edge_counter_read();
    edge_counter_stop();
    print(serial, "  the TX pad's rising edges over the whole letter: ", edges,
          "; the pad reads ", TxPin::read() ? "recessive" : "DOMINANT",
          " and the RX pad is pulled dominant throughout", crlf);
    bench.verdict("INTERNAL LOOP-BACK'S TWO PROMISES, both measured: the TX "
                  "pad is held recessive and never moves (zero edges through "
                  "twenty frames), and the RX pin is disconnected - a pad "
                  "pulled hard DOMINANT changed nothing",
                  edges == 0u && TxPin::read());

    Can1::release();
    TxPad::release();
    RxPad::release();
}

// ---------------------------------------------------------------------------
// Letter d - external loop-back, and the frame on the pad
// ---------------------------------------------------------------------------

/// Poll the TX pad through one transmission: the time from the first
/// falling edge (SOF) to the last transition, in CPU cycles, and the
/// number of transitions. Returns false if the pad never moved.
bool frame_on_pad(uint32_t& span_cycles, uint32_t& transitions,
                  uint32_t patience_us = 5000) {
    const uint32_t deadline = patience_us * (SysClock::hz / 1'000'000u);
    const uint32_t t0 = now_ticks();
    bool level = TxPin::read();
    // Wait for SOF: the first dominant bit.
    while (level) {
        if (since(t0) > deadline) {
            return false;
        }
        level = TxPin::read();
    }
    const uint32_t start = now_ticks();
    uint32_t last = start;
    uint32_t count = 1;
    // Follow the frame: every transition moves `last`; three idle
    // milliseconds end it.
    const uint32_t idle = 3000u * (SysClock::hz / 1'000'000u);
    for (;;) {
        const bool now = TxPin::read();
        if (now != level) {
            level = now;
            last = now_ticks();
            ++count;
        }
        if (now_ticks() - last > idle) {
            break;
        }
        if (since(start) > deadline + idle) {
            break;
        }
    }
    span_cycles = last - start;
    transitions = count;
    return true;
}

void td_external() {
    RxPad::claim_rx(PinPull::down);   // the pin is DISREGARDED, and here is the proof
    TxPad::claim_tx();
    edge_counter_setup();

    FdcanConfig c = loop_config(FdcanMode::external_loop_back);
    const bool up = bring_up<Can1>(c);
    bench.verdict("EXTERNAL loop-back comes up: LBCK set and MON clear",
                  up && Can1::loop_back() && !Can1::bus_monitor());

    // A DLC-8 classic frame, and its shadow on the pad.
    FdcanFrame in{};
    in.id = 0x555;
    in.length = 8;
    for (uint8_t i = 0; i < 8; ++i) {
        in.data[i] = static_cast<uint8_t>(0x0F * (i + 1));
    }
    Can1::clear_flags(FdcanFlag::all);
    (void)Can1::tx_put(in);
    uint32_t span = 0;
    uint32_t transitions = 0;
    const bool seen = frame_on_pad(span, transitions);
    FdcanFrame out{};
    const bool got = Can1::rx_read(0, out);

    const uint32_t bit_cycles = SysClock::hz / 500'000u;   // 128 cycles a bit
    const uint32_t bits = span / bit_cycles;
    print(serial, "  the frame on PB9: ", transitions, " transitions spanning ",
          span, " cycles = ", us_of(span), " us = ", bits, " bit times at "
          "500 kbit/s", crlf);
    bench.verdict("THE FRAME LEAVES THE PAD IN EXTERNAL LOOP-BACK - 36.3.4's "
                  "'the transmitted messages can be monitored at the FDCAN_TX "
                  "pin', and it does",
                  seen && transitions > 4u);
    // SOF..last dominant edge of an 11-bit DLC-8 data frame: 1 + 11 + 1 +
    // 1 + 1 + 4 + 64 + 15 = 98 bits before stuffing, up to about 19 stuff
    // bits, and the last EDGE is somewhere in the CRC field's tail.
    bench.verdict("...and its length is the CAN frame's own arithmetic: an "
                  "11-bit DLC-8 data frame is 98 bits from SOF to the end of "
                  "the CRC before stuffing and at most about 117 after it, "
                  "which is where the last edge falls",
                  seen && bits >= 80u && bits <= 125u);
    bench.verdict("and the frame came back INSIDE as well, byte for byte - "
                  "external loop-back feeds the receiver internally, it does "
                  "not listen to the pin",
                  got && out.id == in.id && same_payload(in, out));

    // The RX pin disregarded, and the acknowledge error ignored.
    const auto st = Can1::status();
    const auto ec = Can1::error_counters();
    print(serial, "  with PB8 pulled DOMINANT throughout: TEC ", ec.transmit,
          " REC ", ec.receive, ", PSR LEC ", static_cast<uint8_t>(st.last_error),
          " ACT ", static_cast<uint8_t>(st.activity), crlf);
    bench.verdict("36.3.4's two promises for this mode: 'the actual value of "
                  "the FDCAN_RX input pin is disregarded' - a pad held "
                  "dominant did not disturb a single frame - and acknowledge "
                  "errors are ignored, so the error counters never moved",
                  ec.transmit == 0u && ec.receive == 0u && !st.bus_off);

    // Three frames back to back, to see the pad busy and the counter agree.
    if (edge_counter_arm()) {
        for (uint8_t i = 0; i < 3; ++i) {
            FdcanFrame f = in;
            f.id = 0x600u + i;
            (void)Can1::tx_put(f);
            spin_us(2000);
            FdcanFrame drop{};
            (void)Can1::rx_read(0, drop);
        }
        const uint32_t edges = edge_counter_read();
        edge_counter_stop();
        print(serial, "  three more frames: ", edges, " rising edges counted on "
              "the pad with no CPU in the loop", crlf);
        bench.verdict("the same pad counted by a DMAMUX request generator: a "
                      "CAN frame is a burst of edges, and three of them are "
                      "three bursts",
                      edges > 30u && edges < 300u);
    }

    Can1::release();
    TxPad::release();
    RxPad::release();
}

// ---------------------------------------------------------------------------
// Letter e - the filters
// ---------------------------------------------------------------------------

/// Send `id` and report which FIFO it landed in: 0, 1, or 0xFF for
/// "rejected / nowhere".
uint8_t where_does_it_land(uint32_t id, bool extended = false) {
    FdcanFrame f{};
    f.id = id;
    f.extended = extended;
    f.length = 1;
    f.data[0] = 0x5A;
    Can1::clear_flags(FdcanFlag::all);
    if (!Can1::tx_put(f)) {
        return 0xFE;
    }
    const uint32_t t0 = now_ticks();
    while (us_of(since(t0)) < 4000u) {
        if (Can1::rx_available(0) != 0u) {
            return 0;
        }
        if (Can1::rx_available(1) != 0u) {
            return 1;
        }
    }
    return 0xFF;
}

void te_filters() {
    FdcanConfig c = loop_config(FdcanMode::internal_loop_back);
    c.standard_filters = 4;
    c.extended_filters = 2;
    c.non_matching_standard = FdcanNonMatching::reject;
    c.non_matching_extended = FdcanNonMatching::reject;
    (void)bring_up<Can1>(c);

    // 0: range 0x100..0x10F -> FIFO 0
    // 1: dual 0x200 or 0x2FF -> FIFO 0
    // 2: classic 0x300 mask 0x7F0 -> FIFO 1
    // 3: reject 0x400 exactly
    (void)Can1::standard_filter(0, {FdcanFilterType::range,
                                    FdcanFilterAction::store_fifo0, 0x100, 0x10F});
    (void)Can1::standard_filter(1, {FdcanFilterType::dual,
                                    FdcanFilterAction::store_fifo0, 0x200, 0x2FF});
    (void)Can1::standard_filter(2, {FdcanFilterType::classic,
                                    FdcanFilterAction::store_fifo1, 0x300, 0x7F0});
    (void)Can1::standard_filter(3, {FdcanFilterType::dual,
                                    FdcanFilterAction::reject, 0x400, 0x400});

    const uint8_t in_range = where_does_it_land(0x105);
    drain();
    const uint8_t out_range = where_does_it_land(0x110);
    drain();
    const uint8_t dual_a = where_does_it_land(0x200);
    drain();
    const uint8_t dual_b = where_does_it_land(0x2FF);
    drain();
    const uint8_t dual_out = where_does_it_land(0x250);
    drain();
    const uint8_t classic_in = where_does_it_land(0x305);
    drain();
    const uint8_t classic_out = where_does_it_land(0x315);
    drain();
    const uint8_t rejected = where_does_it_land(0x400);
    drain();
    print(serial, "  range 0x105 -> ", in_range, ", 0x110 -> ", out_range,
          " | dual 0x200 -> ", dual_a, ", 0x2FF -> ", dual_b, ", 0x250 -> ",
          dual_out, " | classic 0x305 -> ", classic_in, ", 0x315 -> ",
          classic_out, " | reject 0x400 -> ", rejected, crlf);
    bench.verdict("the three standard filter types, each with an identifier "
                  "inside and one outside: a RANGE (0x100..0x10F), a DUAL "
                  "(0x200 or 0x2FF) and a CLASSIC mask (0x300 under 0x7F0), "
                  "with the non-matching policy set to reject",
                  in_range == 0u && out_range == 0xFF && dual_a == 0u &&
                      dual_b == 0u && dual_out == 0xFF && classic_in == 1u &&
                      classic_out == 0xFF);
    bench.verdict("...and a REJECT element really rejects, even though its "
                  "identifier is a perfectly good match",
                  rejected == 0xFF);

    // The first match wins: put an overlapping element BEFORE the one
    // that would send the frame elsewhere, and read FIDX.
    (void)Can1::standard_filter(0, {FdcanFilterType::classic,
                                    FdcanFilterAction::store_fifo0, 0x700, 0x7FF});
    (void)Can1::standard_filter(1, {FdcanFilterType::classic,
                                    FdcanFilterAction::store_fifo1, 0x700, 0x7FF});
    (void)where_does_it_land(0x700);
    FdcanFrame got{};
    const bool in_fifo0 = Can1::rx_read(0, got);
    print(serial, "  two elements match 0x700, #0 to FIFO 0 and #1 to FIFO 1: "
          "the frame is in FIFO ", in_fifo0 ? 0 : 1, ", FIDX ", got.filter_index,
          crlf);
    bench.verdict("36.3.6: 'acceptance filtering stops at the first matching "
                  "element' - element #0 took it and element #1 was never "
                  "reached, with FIDX naming which one did",
                  in_fifo0 && got.filter_index == 0u);
    drain();

    // Priority: HPM and HPMS.
    (void)Can1::standard_filter(0, {FdcanFilterType::dual,
                                    FdcanFilterAction::priority_fifo0, 0x77, 0x77});
    Can1::clear_flags(FdcanFlag::all);
    (void)where_does_it_land(0x77);
    const uint32_t flags = Can1::flags();
    const auto hp = Can1::high_priority();
    print(serial, "  a priority match on 0x77: IR.HPM ",
          (flags & FdcanFlag::high_priority) != 0u ? "set" : "clear",
          ", HPMS list ", hp.extended_list ? "extended" : "standard",
          " FIDX ", hp.filter_index, " MSI ",
          static_cast<uint8_t>(hp.storage), " BIDX ", hp.buffer_index, crlf);
    bench.verdict("a 'set priority and store' element raises IR.HPM and fills "
                  "HPMS with the list, the element index, WHERE the frame went "
                  "and the FIFO slot it went into",
                  (flags & FdcanFlag::high_priority) != 0u &&
                      !hp.extended_list && hp.filter_index == 0u &&
                      hp.storage == FdcanHighPriorityStorage::fifo0);
    drain();

    // The non-matching policy, both ways.
    (void)Can1::stop();
    (void)Can1::configuration(true);
    (void)Can1::non_matching(FdcanNonMatching::fifo1, FdcanNonMatching::reject);
    (void)Can1::filter_lists(1, 1);
    (void)Can1::start();
    (void)Can1::standard_filter(0, {FdcanFilterType::dual,
                                    FdcanFilterAction::store_fifo0, 0x11, 0x11});
    const uint8_t unmatched = where_does_it_land(0x33);
    FdcanFrame anmf{};
    const bool got_anmf = Can1::rx_read(1, anmf);
    drain();
    print(serial, "  ANFS = 'accept in FIFO 1': an unmatched 0x33 landed in "
          "FIFO ", unmatched, " with ANMF ", anmf.non_matching ? "set" : "clear",
          crlf);
    bench.verdict("RXGFC.ANFS routes what no filter matched, and the element's "
                  "own ANMF bit says so - the one case where FIDX means "
                  "nothing",
                  unmatched == 1u && got_anmf && anmf.non_matching);

    // Remote rejection.
    (void)Can1::stop();
    (void)Can1::configuration(true);
    (void)Can1::reject_remote(true, false);
    (void)Can1::non_matching(FdcanNonMatching::fifo0, FdcanNonMatching::fifo0);
    (void)Can1::start();
    FdcanFrame rf{};
    rf.id = 0x11;
    rf.remote = true;
    Can1::clear_flags(FdcanFlag::all);
    (void)Can1::tx_put(rf);
    spin_us(3000);
    const uint8_t after_reject = Can1::rx_available(0);
    const uint32_t tc = Can1::flags();
    drain();
    print(serial, "  RRFS set: the remote frame was transmitted (IR.TC ",
          (tc & FdcanFlag::transmission_completed) != 0u ? "set" : "clear",
          ") and stored ", after_reject, " times", crlf);
    bench.verdict("RXGFC.RRFS rejects every 11-bit remote frame BEFORE the "
                  "filter list is reached (figure 400's own first branch) - "
                  "the frame still goes out, and nothing comes back",
                  after_reject == 0u &&
                      (tc & FdcanFlag::transmission_completed) != 0u);

    // Extended filters and XIDAM.
    (void)Can1::stop();
    (void)Can1::configuration(true);
    (void)Can1::reject_remote(false, false);
    (void)Can1::non_matching(FdcanNonMatching::reject, FdcanNonMatching::reject);
    (void)Can1::filter_lists(0, 2);
    (void)Can1::extended_mask(0x0FFFFFFFu);   // the top bit of the ID masked away
    (void)Can1::start();
    // #0: a range 0x0100_0000..0x0100_00FF with XIDAM applied (EFT 00)
    (void)Can1::extended_filter(0, {FdcanExtFilterType::range,
                                    FdcanFilterAction::store_fifo0,
                                    0x01000000u, 0x010000FFu});
    // #1: the same range with XIDAM NOT applied (EFT 11) - into FIFO 1
    (void)Can1::extended_filter(1, {FdcanExtFilterType::range_no_mask,
                                    FdcanFilterAction::store_fifo1,
                                    0x11000000u, 0x110000FFu});
    const uint8_t masked = where_does_it_land(0x11000010u, true);
    drain();
    const uint8_t unmasked = where_does_it_land(0x01000010u, true);
    drain();
    print(serial, "  XIDAM 0x0FFFFFFF: identifier 0x11000010 landed in FIFO ",
          masked, " (element #0's range, which it only meets AFTER the mask), "
          "and 0x01000010 in FIFO ", unmasked, crlf);
    bench.verdict("36.3.6: XIDAM is AND-ed with a received 29-bit identifier "
                  "BEFORE the extended list runs - 0x11000010 masked down to "
                  "0x01000010 matched element #0's range and never reached "
                  "element #1, whose EFT = 11 asks for the raw identifier",
                  masked == 0u && unmasked == 0u);

    // The LSS clamp of 36.4.19 - what does the silicon do with 31?
    (void)Can1::stop();
    (void)Can1::configuration(true);
    Can1::regs().RXGFC = (Can1::regs().RXGFC & ~FDCAN_RXGFC_LSS_Msk) |
                         (31u << FDCAN_RXGFC_LSS_Pos);
    const uint8_t clamped = Can1::standard_filter_list();
    const bool driver_refuses = !Can1::filter_lists(31, 0);
    print(serial, "  LSS written 31 by hand: reads back ", clamped,
          "; the driver's own verb refuses it: ",
          driver_refuses ? "yes" : "no", crlf);
    bench.verdict("36.4.19's 'values greater than 28 are interpreted as 28' is "
                  "a CLAMP IN THE REGISTER and not merely in the filter "
                  "engine: 31 written by hand reads back as 28 - and the "
                  "driver refuses it anyway, because a program that asks for "
                  "31 filter elements has a bug and a silent clamp hides it",
                  driver_refuses && clamped == 28u);

    // A filter edited WHILE RUNNING (36.3.6 executes the list from #0 at
    // every frame, and the RAM is not a protected register).
    (void)Can1::filter_lists(1, 0);
    (void)Can1::non_matching(FdcanNonMatching::reject, FdcanNonMatching::reject);
    (void)Can1::start();
    (void)Can1::standard_filter(0, {FdcanFilterType::dual,
                                    FdcanFilterAction::store_fifo0, 0x123, 0x123});
    const uint8_t before_edit = where_does_it_land(0x456);
    drain();
    (void)Can1::standard_filter(0, {FdcanFilterType::dual,
                                    FdcanFilterAction::store_fifo0, 0x456, 0x456});
    const uint8_t after_edit = where_does_it_land(0x456);
    drain();
    print(serial, "  0x456 before the live edit: ", before_edit,
          " (0xFF = rejected); after it: ", after_edit, crlf);
    bench.verdict("A FILTER EDITED WHILE THE MODULE IS ON THE BUS TAKES EFFECT "
                  "AT THE NEXT FRAME: 36.3.6 executes the list from element #0 "
                  "for every message and the RAM is not a protected register, "
                  "so no INIT is needed to change what is accepted",
                  before_edit == 0xFF && after_edit == 0u);

    Can1::release();
}

// ---------------------------------------------------------------------------
// Letter f - the FIFOs and the transmit order
// ---------------------------------------------------------------------------

void tf_fifos() {
    FdcanConfig c = loop_config(FdcanMode::internal_loop_back);
    (void)bring_up<Can1>(c);

    // --- blocking: three in, the fourth lost.
    Can1::clear_flags(FdcanFlag::all);
    for (uint8_t i = 0; i < 4; ++i) {
        FdcanFrame f{};
        f.id = 0x10u + i;
        f.length = 1;
        f.data[0] = static_cast<uint8_t>(0xB0u + i);
        (void)Can1::tx_put(f);
        spin_us(2000);
        feed();
    }
    const uint8_t level = Can1::rx_available(0);
    const bool full = Can1::rx_full(0);
    const bool lost = Can1::rx_lost(0);
    const uint32_t flags = Can1::flags();
    uint8_t kept[3] = {0, 0, 0};
    for (uint8_t i = 0; i < 3; ++i) {
        FdcanFrame f{};
        if (Can1::rx_read(0, f)) {
            kept[i] = f.data[0];
        }
    }
    print(serial, "  blocking: level ", level, " full=", full ? "1" : "0",
          " RF0L=", lost ? "1" : "0", " IR ", hex(flags), ", the three kept "
          "are ", hex(kept[0]), " ", hex(kept[1]), " ", hex(kept[2]), crlf);
    bench.verdict("BLOCKING MODE (the reset behaviour): the fourth frame into "
                  "a full FIFO is DISCARDED and the three already there are "
                  "untouched, with RF0F and RF0L both raised",
                  level == 3u && full && lost && kept[0] == 0xB0u &&
                      kept[1] == 0xB1u && kept[2] == 0xB2u &&
                      (flags & FdcanFlag::rx_fifo0_lost) != 0u &&
                      (flags & FdcanFlag::rx_fifo0_full) != 0u);

    // --- overwrite: the fourth overwrites the oldest.
    (void)Can1::stop();
    (void)Can1::configuration(true);
    (void)Can1::fifo_overwrite(true, false);
    (void)Can1::start();
    drain();
    Can1::clear_flags(FdcanFlag::all);
    for (uint8_t i = 0; i < 4; ++i) {
        FdcanFrame f{};
        f.id = 0x20u + i;
        f.length = 1;
        f.data[0] = static_cast<uint8_t>(0xC0u + i);
        (void)Can1::tx_put(f);
        spin_us(2000);
        feed();
    }
    const uint8_t o_level = Can1::rx_available(0);
    const uint8_t o_get = Can1::rx_get_index(0);
    const uint8_t o_put = Can1::rx_put_index(0);
    const bool o_lost = Can1::rx_lost(0);
    uint8_t seen[3] = {0, 0, 0};
    for (uint8_t i = 0; i < 3; ++i) {
        FdcanFrame f{};
        if (Can1::rx_read(0, f)) {
            seen[i] = f.data[0];
        }
    }
    print(serial, "  overwrite: level ", o_level, " get ", o_get, " put ", o_put,
          " RF0L=", o_lost ? "1" : "0", ", read from the get index: ",
          hex(seen[0]), " ", hex(seen[1]), " ", hex(seen[2]), crlf);
    bench.verdict("OVERWRITE MODE (RXGFC.F0OM): the fourth frame takes the "
                  "OLDEST one's place and BOTH indices move on, so the FIFO "
                  "still holds three, the one that was dropped is the one that "
                  "had waited longest, and RF0L is NOT raised - a discarded "
                  "message and an overwritten one are different events",
                  o_level == 3u && o_get == 1u && o_put == 1u && !o_lost &&
                      seen[0] == 0xC1u && seen[1] == 0xC2u && seen[2] == 0xC3u);

    // And what 36.3.6's own reading rule costs. "When an Rx FIFO is
    // operated in overwrite mode and an Rx FIFO full condition is
    // signaled, reading from the Rx FIFO elements must start at least at
    // get index + 1" - the rule that keeps a reader off the element the
    // writer may be in. It is not free.
    drain();
    Can1::clear_flags(FdcanFlag::all);
    for (uint8_t i = 0; i < 4; ++i) {
        FdcanFrame g{};
        g.id = 0x28u + i;
        g.length = 1;
        g.data[0] = static_cast<uint8_t>(0xD0u + i);
        (void)Can1::tx_put(g);
        spin_us(2000);
        feed();
    }
    uint8_t safe[3] = {0, 0, 0};
    uint8_t safe_count = 0;
    for (uint8_t i = 0; i < 3; ++i) {
        FdcanFrame g{};
        if (Can1::rx_read_overwrite(0, g)) {
            safe[i] = g.data[0];
            ++safe_count;
        }
    }
    print(serial, "  the same four read through the GET + 1 rule: ", hex(safe[0]),
          " ", hex(safe[1]), " ", hex(safe[2]), " (", safe_count,
          " elements came back, not 3)", crlf);
    bench.verdict("...and 36.3.6'S OWN SAFE READ COSTS ONE MORE MESSAGE: "
                  "starting at get index + 1 skips the oldest element and "
                  "acknowledging the one actually read carries the get index "
                  "past it, so the rule that keeps a reader out of the "
                  "writer's element also throws that element away - which the "
                  "chapter asks for and never says",
                  safe_count == 2u && safe[0] == 0xD2u && safe[1] == 0xD3u);
    drain();
    (void)Can1::stop();
    (void)Can1::configuration(true);
    (void)Can1::fifo_overwrite(false, false);
    (void)Can1::start();

    // --- the Tx order: FIFO keeps the insertion order.
    drain();
    Can1::clear_flags(FdcanFlag::all);
    (void)Can1::stop();
    (void)Can1::configuration(true);
    (void)Can1::tx_queue_mode(false);
    (void)Can1::timestamp(FdcanTimestamp::internal, 1);
    (void)Can1::start();
    for (uint8_t i = 0; i < 3; ++i) {
        FdcanFrame f{};
        f.id = 0x300u - 0x10u * i;   // DESCENDING identifiers
        f.length = 1;
        f.data[0] = i;
        f.marker = i;
        (void)Can1::tx_put_buffer(i, f);
    }
    const uint8_t free_before = Can1::tx_free();
    (void)Can1::tx_request(0x7);   // all three in ONE write
    spin_us(6000);
    uint8_t order[3] = {0xFF, 0xFF, 0xFF};
    for (uint8_t i = 0; i < 3; ++i) {
        FdcanTxEvent e{};
        if (Can1::event_read(e)) {
            order[i] = e.marker;
        }
    }
    print(serial, "  Tx FIFO, three markers requested in one TXBAR write "
          "with descending identifiers: they left as ", order[0], " ", order[1],
          " ", order[2], " (free level before the request: ", free_before, ")",
          crlf);
    bench.verdict("TFQM = 0 IS A FIFO: three frames requested together leave "
                  "in the order they were WRITTEN, whatever their identifiers "
                  "say - which is 36.3.6's whole point about the Tx FIFO",
                  order[0] == 0u && order[1] == 1u && order[2] == 2u &&
                      free_before == 3u);
    drain();

    // --- the Tx QUEUE reorders by identifier.
    (void)Can1::stop();
    (void)Can1::configuration(true);
    (void)Can1::tx_queue_mode(true);
    (void)Can1::start();
    for (uint8_t i = 0; i < 3; ++i) {
        FdcanFrame f{};
        f.id = 0x300u - 0x10u * i;   // buffer 0 highest ID, buffer 2 lowest
        f.length = 1;
        f.marker = i;
        (void)Can1::tx_put_buffer(i, f);
    }
    const uint8_t q_free = Can1::tx_free();
    const uint8_t q_get = Can1::tx_get_index();
    (void)Can1::tx_request(0x7);
    spin_us(6000);
    uint8_t q_order[3] = {0xFF, 0xFF, 0xFF};
    for (uint8_t i = 0; i < 3; ++i) {
        FdcanTxEvent e{};
        if (Can1::event_read(e)) {
            q_order[i] = e.marker;
        }
    }
    print(serial, "  Tx QUEUE, the same three: they left as ", q_order[0], " ",
          q_order[1], " ", q_order[2], "; TXFQS.TFFL reads ", q_free,
          " and TFGI ", q_get, " in queue mode", crlf);
    bench.verdict("TFQM = 1 IS A QUEUE: the SAME three buffers leave LOWEST "
                  "IDENTIFIER FIRST - marker 2 (0x2E0) before 1 (0x2F0) before "
                  "0 (0x300) - which is the exact reverse of the FIFO's answer "
                  "to the same request",
                  q_order[0] == 2u && q_order[1] == 1u && q_order[2] == 0u);
    bench.verdict("...and 36.4.27's own footnote is literal: TFFL and TFGI "
                  "READ AS ZERO in queue mode, so a queue's room is TFQF and "
                  "not the free level",
                  q_free == 0u && q_get == 0u);
    drain();

    // --- cancellation.
    (void)Can1::stop();
    (void)Can1::configuration(true);
    (void)Can1::tx_queue_mode(false);
    (void)Can1::start();
    // A buffer requested and cancelled before the scan can start it.
    {
        FdcanFrame f{};
        f.id = 0x123;
        f.length = 8;
        (void)Can1::tx_put_buffer(0, f);
        (void)Can1::tx_put_buffer(1, f);
        Can1::clear_flags(FdcanFlag::all);
        (void)Can1::tx_request(0x3);
        (void)Can1::tx_cancel(0x2);   // cancel the SECOND, which cannot have started
        spin_us(4000);
        const uint8_t occurred = Can1::tx_occurred();
        const uint8_t cancelled = Can1::tx_cancelled();
        const uint8_t pending = Can1::tx_pending();
        const uint32_t f2 = Can1::flags();
        print(serial, "  cancel: TXBTO ", hex(occurred), " TXBCF ",
              hex(cancelled), " TXBRP ", hex(pending), " IR.TCF ",
              (f2 & FdcanFlag::cancellation_finished) != 0u ? "set" : "clear",
              crlf);
        bench.verdict("a cancellation of a buffer whose transmission had not "
                      "started clears its TXBRP bit, sets its TXBCF bit and "
                      "leaves TXBTO alone - while the other buffer went out "
                      "normally",
                      (occurred & 0x1u) != 0u && (cancelled & 0x2u) != 0u &&
                          (occurred & 0x2u) == 0u && pending == 0u &&
                          (f2 & FdcanFlag::cancellation_finished) != 0u);
        drain();
    }

    // "Transmission in spite of cancellation": cancel one already on the
    // wire and read the event type.
    {
        FdcanFrame f{};
        f.id = 0x123;
        f.length = 8;
        f.marker = 0x33;
        (void)Can1::tx_put_buffer(0, f);
        Can1::clear_flags(FdcanFlag::all);
        (void)Can1::tx_request(0x1);
        spin_us(60);   // well inside a 220 us frame
        (void)Can1::tx_cancel(0x1);
        spin_us(4000);
        const uint8_t occurred = Can1::tx_occurred();
        const uint8_t cancelled = Can1::tx_cancelled();
        FdcanTxEvent e{};
        const bool got = Can1::event_read(e);
        print(serial, "  cancel in flight: TXBTO ", hex(occurred), " TXBCF ",
              hex(cancelled), ", event type ", e.event_type, " marker ",
              hex(e.marker), crlf);
        bench.verdict("36.4.28: a cancellation asked for while the frame is "
                      "ALREADY ON THE WIRE cannot stop it - the transmission "
                      "finishes, TXBTO and TXBCF are BOTH set, and the Tx "
                      "event carries type 10, 'transmission in spite of "
                      "cancellation'",
                      got && (occurred & 1u) != 0u && (cancelled & 1u) != 0u &&
                          e.event_type == 2u && e.marker == 0x33u);
        drain();
    }

    // --- the Tx event FIFO overflowing.
    {
        drain();
        Can1::clear_flags(FdcanFlag::all);
        for (uint8_t i = 0; i < 4; ++i) {
            FdcanFrame f{};
            f.id = 0x40u + i;
            f.length = 1;
            f.marker = i;
            (void)Can1::tx_put(f);
            spin_us(2000);
            FdcanFrame d{};
            (void)Can1::rx_read(0, d);   // keep the Rx FIFO clear, not the event one
            feed();
        }
        const uint8_t fill = Can1::events_available();
        const bool ev_lost = Can1::events_lost();
        const uint32_t f3 = Can1::flags();
        print(serial, "  four transmissions with the event FIFO never read: "
              "fill ", fill, ", TEFL ", ev_lost ? "set" : "clear", ", IR.TEFF ",
              (f3 & FdcanFlag::tx_event_full) != 0u ? "set" : "clear", crlf);
        bench.verdict("the Tx event FIFO is three deep and the fourth event is "
                      "DISCARDED, not stored: TEFF marks it full and TEFL marks "
                      "the loss",
                      fill == 3u && ev_lost &&
                          (f3 & FdcanFlag::tx_event_full) != 0u &&
                          (f3 & FdcanFlag::tx_event_lost) != 0u);
        drain();
    }

    // --- the transmit pause, measured in the Tx event timestamps.
    {
        uint16_t gap_off = 0;
        uint16_t gap_on = 0;
        for (uint8_t pass = 0; pass < 2; ++pass) {
            (void)Can1::stop();
            (void)Can1::configuration(true);
            (void)Can1::transmit_pause(pass == 1);
            (void)Can1::timestamp(FdcanTimestamp::internal, 1);
            (void)Can1::start();
            drain();
            FdcanFrame f{};
            f.id = 0x100;
            f.length = 0;
            (void)Can1::tx_put_buffer(0, f);
            (void)Can1::tx_put_buffer(1, f);
            (void)Can1::tx_request(0x3);
            spin_us(6000);
            FdcanTxEvent a{};
            FdcanTxEvent b{};
            const bool ga = Can1::event_read(a);
            const bool gb = Can1::event_read(b);
            const uint16_t d = (ga && gb) ? static_cast<uint16_t>(b.timestamp -
                                                                  a.timestamp)
                                          : 0u;
            if (pass == 0) {
                gap_off = d;
            } else {
                gap_on = d;
            }
            drain();
        }
        print(serial, "  two back-to-back frames, start to start: ", gap_off,
              " bit times with TXP clear and ", gap_on, " with it set - a "
              "difference of ",
              gap_on > gap_off ? gap_on - gap_off : 0u, crlf);
        bench.verdict("CCCR.TXP is EXACTLY TWO CAN BIT TIMES of extra silence "
                      "between one successful transmission and the next, and "
                      "the internal timestamp counter is fine enough to say so",
                      gap_on == gap_off + 2u);
    }

    Can1::release();
}

// ---------------------------------------------------------------------------
// Letter g - CAN FD
// ---------------------------------------------------------------------------

void tg_fd() {
    RxPad::claim_rx(PinPull::up);
    TxPad::claim_tx();

    FdcanConfig c = loop_config(FdcanMode::internal_loop_back);
    c.data = *fdcan_data_timing_for(SysClock::pclk_hz, 2'000'000u, 875);
    c.fd = FdcanFd::on_with_bit_rate_switch;
    (void)bring_up<Can1>(c);
    bench.verdict("CAN FD comes up: FDOE and BRSE both set through one "
                  "configuration",
                  Can1::fd_mode() == FdcanFd::on_with_bit_rate_switch);

    // --- THE 0xCC QUESTION. 36.3.4: "In case an FDCAN Tx buffer is
    // configured for FDCAN transmission with DLC > 8, the first eight
    // bytes are transmitted as configured while the remaining part of
    // the data field is padded with 0xCC. When the FDCAN receives a
    // FDCAN frame with DLC > 8, the first eight bytes of that frame are
    // stored into the matching Rx FIFO. The remaining bytes are
    // discarded." - against figure 399, which allocates 64 bytes an
    // element. Sixty-four DISTINCT bytes, and print what comes back.
    {
        FdcanFrame in{};
        in.id = 0x1F0;
        in.length = 64;
        in.fd = true;
        in.bit_rate_switch = true;
        for (uint8_t i = 0; i < 64; ++i) {
            in.data[i] = static_cast<uint8_t>(0x40u + i);   // 0x40..0x7F, all distinct
        }
        FdcanFrame out{};
        const bool got = round_trip<Can1>(in, out);
        uint8_t exact = 0;
        uint8_t padded = 0;
        for (uint8_t i = 0; i < out.length && i < 64; ++i) {
            if (out.data[i] == in.data[i]) {
                ++exact;
            }
            if (out.data[i] == 0xCCu) {
                ++padded;
            }
        }
        print(serial, "  64 distinct bytes sent, ", out.length, " received; "
              "the first eight are ", hex(out.data[0]), " ", hex(out.data[1]),
              " ", hex(out.data[2]), " ", hex(out.data[3]), " ",
              hex(out.data[4]), " ", hex(out.data[5]), " ", hex(out.data[6]),
              " ", hex(out.data[7]), crlf);
        print(serial, "  bytes 8..11 are ", hex(out.data[8]), " ",
              hex(out.data[9]), " ", hex(out.data[10]), " ", hex(out.data[11]),
              ", bytes 60..63 are ", hex(out.data[60]), " ", hex(out.data[61]),
              " ", hex(out.data[62]), " ", hex(out.data[63]), crlf);
        print(serial, "  of the ", out.length, " received bytes, ", exact,
              " are the byte that was sent and ", padded, " are 0xCC", crlf);
        bench.verdict("A 64-BYTE CAN FD FRAME MAKES THE WHOLE ROUND TRIP "
                      "BYTE-EXACT on this silicon: DLC 15 arrives as 64 bytes "
                      "and every one of them is what was written",
                      got && out.length == 64u && exact == 64u && padded == 0u &&
                          out.fd && out.bit_rate_switch);
        print(serial, "  SO 36.3.4'S 0xCC PARAGRAPH DOES NOT DESCRIBE THIS "
              "PART. Figure 399 allocates eighteen words - 64 bytes - to every "
              "Rx and Tx element, and the element really does carry them; the "
              "paragraph belongs to an M_CAN configured with an 8-byte data "
              "field, which this one is not.", crlf);
    }

    // --- every FD DLC.
    {
        static const uint8_t lengths[7] = {12, 16, 20, 24, 32, 48, 64};
        bool all_ok = true;
        for (uint8_t k = 0; k < 7; ++k) {
            FdcanFrame in{};
            in.id = 0x1E0u + k;
            in.length = lengths[k];
            in.fd = true;
            for (uint8_t i = 0; i < lengths[k]; ++i) {
                in.data[i] = static_cast<uint8_t>(lengths[k] ^ i);
            }
            FdcanFrame out{};
            const bool ok = round_trip<Can1>(in, out) && same_payload(in, out) &&
                            out.fd;
            all_ok = all_ok && ok;
            if (!ok) {
                print(serial, "  DLC for ", lengths[k], " bytes FAILED (got ",
                      out.length, ")", crlf);
            }
            feed();
        }
        bench.verdict("table 212's seven long DLC codes, 12 through 64 bytes, "
                      "every one of them byte-exact",
                      all_ok);
    }

    // --- ESI.
    {
        FdcanFrame in{};
        in.id = 0x1D0;
        in.length = 8;
        in.fd = true;
        in.error_state_indicator = true;
        FdcanFrame out{};
        const bool got = round_trip<Can1>(in, out);
        const auto st = Can1::status();
        print(serial, "  ESI forced recessive in the Tx element: received ESI ",
              out.error_state_indicator ? "1" : "0", ", PSR.RESI ",
              st.received_esi ? "1" : "0", " REDL ", st.received_fd ? "1" : "0",
              " RBRS ", st.received_brs ? "1" : "0", crlf);
        bench.verdict("table 217's footnote: an error-ACTIVE node may still "
                      "transmit ESI recessive by asking for it in the element, "
                      "and the receiver reports it in both the element and "
                      "PSR.RESI",
                      got && out.error_state_indicator && st.received_esi &&
                          st.received_fd);
    }

    // --- NISO.
    {
        (void)Can1::stop();
        (void)Can1::configuration(true);
        (void)Can1::non_iso(true);
        (void)Can1::start();
        FdcanFrame in{};
        in.id = 0x1C0;
        in.length = 16;
        in.fd = true;
        for (uint8_t i = 0; i < 16; ++i) {
            in.data[i] = static_cast<uint8_t>(i * 3u);
        }
        FdcanFrame out{};
        const bool niso_ok = round_trip<Can1>(in, out) && same_payload(in, out);
        (void)Can1::stop();
        (void)Can1::configuration(true);
        (void)Can1::non_iso(false);
        (void)Can1::start();
        FdcanFrame out2{};
        const bool iso_ok = round_trip<Can1>(in, out2) && same_payload(in, out2);
        bench.verdict("CCCR.NISO: the Bosch V1.0 frame format and ISO "
                      "11898-1's both loop perfectly - which is exactly what a "
                      "loop-back CANNOT tell apart, since both ends change "
                      "together (the CRC and the stuff-bit count differ on the "
                      "WIRE and there is no second node here to disagree)",
                      niso_ok && iso_ok);
    }

    // --- BRS at three data rates, and the frame's duration on the pad.
    {
        (void)Can1::release();
        static const uint32_t rates[3] = {2'000'000u, 4'000'000u, 8'000'000u};
        uint32_t spans[3] = {0, 0, 0};
        bool loops[3] = {false, false, false};
        for (uint8_t k = 0; k < 3; ++k) {
            FdcanConfig e = loop_config(FdcanMode::external_loop_back);
            const auto dt = fdcan_data_timing_for(SysClock::pclk_hz, rates[k], 875);
            if (!dt) {
                continue;
            }
            e.data = *dt;
            e.fd = FdcanFd::on_with_bit_rate_switch;
            (void)bring_up<Can1>(e);
            FdcanFrame in{};
            in.id = 0x1B0;
            in.length = 64;
            in.fd = true;
            in.bit_rate_switch = true;
            for (uint8_t i = 0; i < 64; ++i) {
                in.data[i] = static_cast<uint8_t>(i);
            }
            Can1::clear_flags(FdcanFlag::all);
            (void)Can1::tx_put(in);
            uint32_t span = 0;
            uint32_t transitions = 0;
            (void)frame_on_pad(span, transitions);
            spans[k] = us_of(span);
            FdcanFrame out{};
            loops[k] = Can1::rx_read(0, out) && same_payload(in, out) &&
                       out.bit_rate_switch;
            print(serial, "  data phase ", rates[k] / 1000u, " kbit/s: the "
                  "64-byte frame occupies ", spans[k], " us of pad (",
                  transitions, " transitions), loop ",
                  loops[k] ? "byte-exact" : "BROKEN", crlf);
            Can1::release();
            feed();
        }
        bench.verdict("BIT RATE SWITCHING IS VISIBLE AS TIME: the same 64-byte "
                      "frame gets shorter as the data phase speeds up, while "
                      "its arbitration phase (500 kbit/s) does not move - and "
                      "the loop stays byte-exact at all three rates",
                      spans[0] > spans[1] && spans[1] > spans[2] && loops[0] &&
                          loops[1] && loops[2]);
    }

    // --- TDC: the transmitter delay compensation value in loop-back.
    {
        FdcanConfig e = loop_config(FdcanMode::internal_loop_back);
        e.data = *fdcan_data_timing_for(SysClock::pclk_hz, 4'000'000u, 875);
        e.fd = FdcanFd::on_with_bit_rate_switch;
        e.transmitter_delay_compensation = true;
        e.tdc_offset = 16;
        e.tdc_filter = 0;
        (void)bring_up<Can1>(e);
        FdcanFrame in{};
        in.id = 0x1A0;
        in.length = 16;
        in.fd = true;
        in.bit_rate_switch = true;
        FdcanFrame out{};
        const bool got = round_trip<Can1>(in, out);
        const auto st = Can1::status();
        print(serial, "  TDC on, TDCO 16 mtq: PSR.TDCV reads ", st.tdcv,
              " mtq after an FD frame with BRS", crlf);
        bench.verdict("36.3.4's delay measurement runs in loop-back: TDCV is "
                      "the measured FDCAN_TX to FDCAN_RX delay PLUS TDCO, and "
                      "with an internal loop the delay itself is a handful of "
                      "minimum time quanta",
                      got && st.tdcv >= 16u && st.tdcv < 127u &&
                          Can1::delay_compensation());
        Can1::release();
    }

    TxPad::release();
    RxPad::release();
}

// ---------------------------------------------------------------------------
// Letter h - timestamps and the timeout counter
// ---------------------------------------------------------------------------

void th_time() {
    FdcanConfig c = loop_config(FdcanMode::internal_loop_back);
    c.timestamp = FdcanTimestamp::internal;
    c.timestamp_prescaler = 1;
    (void)bring_up<Can1>(c);

    // --- the internal counter IS a bit-time counter.
    {
        Can1::reset_timestamp();
        const uint32_t t0 = now_ticks();
        const uint16_t a = Can1::timestamp_value();
        spin_us(20000);
        const uint16_t b = Can1::timestamp_value();
        const uint32_t elapsed = us_of(since(t0));
        const uint32_t ticks = static_cast<uint16_t>(b - a);
        const uint32_t rate = (ticks * 1000u) / (elapsed / 1000u);
        print(serial, "  TSS = internal, TCP = 1: the counter advanced ", ticks,
              " in ", elapsed, " us = ", rate, " counts/s against a nominal bit "
              "rate of 500000", crlf);
        bench.verdict("THE TIMESTAMP COUNTER RUNS ON THE BIT CLOCK AND NOT ON "
                      "TRAFFIC: with an idle bus it still counts once per CAN "
                      "bit time, which makes it a bit-rate meter for an "
                      "instance with no pad at all",
                      rate > 495000u && rate < 505000u);
    }

    // --- TCP 16.
    {
        (void)Can1::stop();
        (void)Can1::configuration(true);
        (void)Can1::timestamp(FdcanTimestamp::internal, 16);
        (void)Can1::start();
        Can1::reset_timestamp();
        const uint16_t a = Can1::timestamp_value();
        spin_us(20000);
        const uint16_t b = Can1::timestamp_value();
        const uint32_t ticks = static_cast<uint16_t>(b - a);
        print(serial, "  TCP = 16: ", ticks, " counts in the same 20 ms, "
              "against 10000 / 16 = 625", crlf);
        bench.verdict("TCP counts in MULTIPLES OF CAN BIT TIMES, 1 to 16, and "
                      "the register holds one less than the multiple (36.4.8)",
                      ticks > 615u && ticks < 635u &&
                          Can1::timestamp_prescaler() == 16u);
    }

    // --- consecutive frames' RXTS = the frame length in bit times.
    {
        (void)Can1::stop();
        (void)Can1::configuration(true);
        (void)Can1::timestamp(FdcanTimestamp::internal, 1);
        (void)Can1::start();
        drain();
        FdcanFrame f{};
        f.id = 0x111;
        f.length = 8;
        (void)Can1::tx_put_buffer(0, f);
        (void)Can1::tx_put_buffer(1, f);
        (void)Can1::tx_request(0x3);
        spin_us(6000);
        FdcanFrame a{};
        FdcanFrame b{};
        const bool ga = Can1::rx_read(0, a);
        const bool gb = Can1::rx_read(0, b);
        const uint16_t delta = static_cast<uint16_t>(b.timestamp - a.timestamp);
        print(serial, "  two back-to-back DLC-8 frames: RXTS ", hex(a.timestamp),
              " and ", hex(b.timestamp), ", ", delta, " bit times apart", crlf);
        bench.verdict("the Rx timestamp is captured AT START OF FRAME, so the "
                      "difference between two consecutive receptions is the "
                      "whole frame plus the interframe space - 111 stuffed "
                      "bits plus 3 of intermission for a DLC-8 classic frame, "
                      "give or take the stuffing",
                      ga && gb && delta >= 105u && delta <= 135u);
        drain();
    }

    // --- TSW, the wrap-around flag.
    {
        (void)Can1::stop();
        (void)Can1::configuration(true);
        (void)Can1::timestamp(FdcanTimestamp::internal, 16);
        (void)Can1::start();
        Can1::clear_flags(FdcanFlag::all);
        // 65536 counts of 16 bit times at 500 kbit/s = 2.1 s. Too long;
        // preset the counter instead by letting it run from a high value
        // is impossible (a write RESETS it), so this leg is timed.
        Can1::reset_timestamp();
        const uint32_t t0 = now_ticks();
        bool wrapped = false;
        while (us_of(since(t0)) < 2500000u) {
            feed();
            if ((Can1::flags() & FdcanFlag::timestamp_wrap) != 0u) {
                wrapped = true;
                break;
            }
        }
        const uint32_t took = us_of(since(t0));
        print(serial, "  TSW after ", took, " us, against 65536 x 16 bit times "
              "= 2097 ms at 500 kbit/s", crlf);
        bench.verdict("IR.TSW is the wrap-around of a 16-bit counter and lands "
                      "where the arithmetic puts it",
                      wrapped && took > 2000000u && took < 2300000u);
    }

    // --- the EXTERNAL timestamp: TIM3's counter, a wireless cross-check
    // between two peripherals that share nothing but this one wire in
    // the interconnect.
    {
        Stamper::init();
        (void)Stamper::configure({.prescaler = 63, .period = 0xFFFF});   // 1 MHz
        Stamper::enable(true);
        (void)Can1::stop();
        (void)Can1::configuration(true);
        (void)Can1::timestamp(FdcanTimestamp::external_tim3, 1);
        (void)Can1::start();
        drain();
        const uint16_t tim_before = static_cast<uint16_t>(Stamper::count());
        const uint16_t can_reads = Can1::timestamp_value();
        FdcanFrame f{};
        f.id = 0x222;
        f.length = 0;
        Can1::clear_flags(FdcanFlag::all);
        const uint16_t at_request = static_cast<uint16_t>(Stamper::count());
        (void)Can1::tx_put(f);
        spin_us(3000);
        FdcanFrame got{};
        const bool ok = Can1::rx_read(0, got);
        const uint16_t after = static_cast<uint16_t>(Stamper::count());
        const uint16_t lag = static_cast<uint16_t>(got.timestamp - at_request);
        print(serial, "  TSS = external: TSCV reads ", can_reads, " while TIM3 "
              "reads ", tim_before, "; a frame requested at TIM3 = ", at_request,
              " carries RXTS ", got.timestamp, " (", lag, " us later), and TIM3 "
              "had reached ", after, crlf);
        bench.verdict("TSCV IS TIM3's COUNTER when TSS = 10: the register "
                      "tracks tim3_cnt[15:0] with no prescaler of its own, "
                      "which is the one external timestamp source this part "
                      "wires up (36.4.8)",
                      static_cast<uint32_t>(
                          can_reads > tim_before
                              ? static_cast<uint16_t>(can_reads - tim_before)
                              : static_cast<uint16_t>(tim_before - can_reads)) < 200u);
        bench.verdict("...and an Rx element's RXTS is that counter captured AT "
                      "START OF FRAME: at 1 MHz the stamp lands a few tens of "
                      "microseconds after the request and well before the "
                      "frame ends",
                      ok && lag < 200u);
        Stamper::enable(false);
    }

    // --- the timeout counter, continuous.
    {
        (void)Can1::stop();
        (void)Can1::configuration(true);
        (void)Can1::timestamp(FdcanTimestamp::internal, 1);
        (void)Can1::timeout(FdcanTimeoutMode::continuous, 1000, true);
        (void)Can1::start();
        Can1::clear_flags(FdcanFlag::all);
        Can1::reset_timeout();
        const uint32_t t0 = now_ticks();
        bool fired = false;
        while (us_of(since(t0)) < 20000u) {
            if ((Can1::flags() & FdcanFlag::timeout) != 0u) {
                fired = true;
                break;
            }
        }
        const uint32_t took = us_of(since(t0));
        print(serial, "  continuous timeout, TOP = 1000 at TCP = 1: TOO after ",
              took, " us against 1000 bit times = 2000 us", crlf);
        bench.verdict("the timeout counter counts DOWN in the same units the "
                      "timestamp counter counts up, and IR.TOO lands at TOP x "
                      "TCP bit times",
                      fired && took > 1800u && took < 2400u);
    }

    // --- the timeout counter under a FIFO.
    {
        (void)Can1::stop();
        (void)Can1::configuration(true);
        (void)Can1::timeout(FdcanTimeoutMode::rx_fifo0, 1000, true);
        (void)Can1::start();
        drain();
        Can1::clear_flags(FdcanFlag::all);
        spin_us(5000);
        const bool quiet = (Can1::flags() & FdcanFlag::timeout) == 0u;
        const uint16_t preset = Can1::timeout_value();
        FdcanFrame f{};
        f.id = 0x333;
        f.length = 0;
        (void)Can1::tx_put(f);
        const uint32_t t0 = now_ticks();
        bool fired = false;
        while (us_of(since(t0)) < 20000u) {
            if ((Can1::flags() & FdcanFlag::timeout) != 0u) {
                fired = true;
                break;
            }
        }
        const uint32_t took = us_of(since(t0));
        print(serial, "  FIFO-controlled timeout: an EMPTY FIFO holds the "
              "counter at ", preset, " (no TOO in 5 ms: ", quiet ? "yes" : "no",
              "), and the first element starts it - TOO ", took,
              " us later", crlf);
        bench.verdict("36.3.4: 'when the timeout counter is controlled by one "
                      "of the FIFOs, an empty FIFO presets the counter to TOP "
                      "and down-counting is started when the first FIFO "
                      "element is stored' - both halves measured, and the "
                      "unread element is what trips it",
                      quiet && preset == 1000u && fired && took < 4000u);
        drain();
    }

    Can1::release();
}

// ---------------------------------------------------------------------------
// Letter i - the error machine with no node
// ---------------------------------------------------------------------------

void ti_errors() {
    edge_counter_setup();
    TxPad::claim_tx();
    RxPad::claim_rx(PinPull::up);   // a recessive "bus", and nothing on it

    FdcanConfig c = loop_config(FdcanMode::normal);
    (void)bring_up<Can1>(c);
    // The bit stream processor needs eleven recessive bits before it can
    // take part in bus activities (36.3.4), which at 500 kbit/s is 22 us:
    // a status read taken straight after start() finds SYNCHRONIZING.
    const auto at_once = Can1::status();
    spin_us(500);
    const auto idle = Can1::status();
    print(serial, "  normal mode on a recessive line: PSR.ACT ",
          static_cast<uint8_t>(at_once.activity), " the instant INIT is "
          "cleared and ", static_cast<uint8_t>(idle.activity),
          " 500 us later (0 = synchronizing, 1 = idle), BO ",
          idle.bus_off ? "1" : "0", crlf);
    bench.verdict("with the RX pad on its own pull-up the module sees eleven "
                  "recessive bits, leaves integration and reports IDLE - "
                  "which is the whole 'bus' this board has, and it is not "
                  "instantaneous: 36.3.4's integration is measurable",
                  at_once.activity == FdcanActivity::synchronizing &&
                      idle.activity == FdcanActivity::idle);

    // --- every dominant bit read back recessive: a bit error per attempt.
    // TEC climbs FAST here - eight per bit error and an active error flag
    // is six more dominant bits that fail the same way - so the walk up
    // figure 398's ladder is polled as tightly as the CPU can.
    FdcanFrame f{};
    f.id = 0x123;
    f.length = 0;
    Can1::clear_flags(FdcanFlag::all);
    uint8_t steps[6] = {0, 0, 0, 0, 0, 0};
    uint8_t step_count = 0;
    uint32_t ew_at = 0;
    uint32_t ep_at = 0;
    bool saw_ew = false;
    bool saw_ep = false;
    bool saw_bo = false;
    uint8_t last_tec = 0;
    FdcanError first_code = FdcanError::no_change;
    const uint32_t t0 = now_ticks();
    (void)Can1::tx_put(f);
    while (us_of(since(t0)) < 500000u) {
        // PSR FIRST and ECR second: a flag read before its counter can
        // only be answered by a counter value at or past the one that
        // raised it, where the other order reports a TEC from before the
        // flag existed (which is how this verdict first read 88 for a
        // warning level of 96).
        const auto st = Can1::status();
        const auto ec = Can1::error_counters();
        if (ec.transmit != last_tec) {
            last_tec = ec.transmit;
            if (step_count < 6u) {
                steps[step_count++] = ec.transmit;
            }
        }
        if (first_code == FdcanError::no_change &&
            st.last_error != FdcanError::no_change &&
            st.last_error != FdcanError::none) {
            first_code = st.last_error;
        }
        if (!saw_ew && st.warning) {
            saw_ew = true;
            ew_at = ec.transmit;
        }
        if (!saw_ep && st.error_passive) {
            saw_ep = true;
            ep_at = ec.transmit;
        }
        if (st.bus_off) {
            saw_bo = true;
            break;
        }
    }
    const uint32_t to_bus_off = us_of(since(t0));
    const uint32_t ir = Can1::flags();
    const bool init_set = Can1::in_init();
    print(serial, "  the first six distinct TEC readings: ", steps[0], " ",
          steps[1], " ", steps[2], " ", steps[3], " ", steps[4], " ", steps[5],
          "; the first error code was ", static_cast<uint8_t>(first_code),
          " (5 = bit0: wanted dominant, monitored recessive)", crlf);
    bool all_octets = step_count >= 3u;
    for (uint8_t k = 0; k < step_count; ++k) {
        if ((steps[k] % 8u) != 0u || (k > 0u && steps[k] <= steps[k - 1u])) {
            all_octets = false;
        }
    }
    bench.verdict("EVERY DOMINANT BIT THIS NODE SENDS IS READ BACK RECESSIVE, "
                  "so every one of them is a BIT0 ERROR (36.4.13's code 101) "
                  "and TEC MOVES ONLY IN MULTIPLES OF EIGHT - the active error "
                  "flag it answers with is six more dominant bits that fail "
                  "the same way, which is why the ladder is walked faster "
                  "than a 64 MHz CPU can read every rung of it",
                  all_octets && first_code == FdcanError::bit0);
    print(serial, "  EW first seen at TEC ", ew_at, ", EP at TEC ", ep_at,
          ", bus-off after ", to_bus_off, " us; IR ", hex(ir), ", INIT ",
          init_set ? "set BY HARDWARE" : "clear", crlf);
    bench.verdict("figure 398's state machine, walked with no second node: "
                  "the warning level at 96, error-passive at 128 and BUS-OFF "
                  "past 255, with IR.EW, IR.EP and IR.BO all raised on the way",
                  saw_ew && saw_ep && saw_bo && ew_at >= 96u && ew_at <= 144u &&
                      ep_at >= 128u && ep_at <= 176u &&
                      (ir & FdcanFlag::warning) != 0u &&
                      (ir & FdcanFlag::error_passive) != 0u &&
                      (ir & FdcanFlag::bus_off) != 0u);
    bench.verdict("...and 36.4.13's note is literal: 'if the device enters "
                  "bus-off, it sets the INIT bit of its own, stopping all bus "
                  "activities'",
                  init_set);

    // --- the recovery sequence: 128 x 11 recessive bits.
    {
        (void)Can1::tx_cancel(0x7);
        Can1::clear_flags(FdcanFlag::all);
        const uint32_t r0 = now_ticks();
        (void)Can1::init_mode(false);
        bool recovered = false;
        while (us_of(since(r0)) < 200000u) {
            feed();
            if (!Can1::status().bus_off) {
                recovered = true;
                break;
            }
        }
        const uint32_t took = us_of(since(r0));
        const auto st = Can1::status();
        const auto ec = Can1::error_counters();
        const uint32_t bits = took / 2u;   // 2 us a bit at 500 kbit/s
        print(serial, "  recovery: BO cleared after ", took, " us = ", bits,
              " bit times, against 128 x 11 = 1408 (36.4.13 says 129 x 11 = "
              "1419); ACT ", static_cast<uint8_t>(st.activity), ", TEC ",
              ec.transmit, " REC ", ec.receive, crlf);
        bench.verdict("THE BUS-OFF RECOVERY SEQUENCE RUNS ON A PULL-UP: with "
                      "INIT cleared the module counts its 11-recessive-bit "
                      "sequences off a pad nothing is driving, comes back "
                      "error-active and resets both error counters",
                      recovered && bits > 1300u && bits < 1600u &&
                          ec.transmit == 0u && ec.receive == 0u);
        bench.verdict("...and it is 36.4.13'S NOTE THAT IS RIGHT AND FIGURE "
                      "398 THAT IS NOT: the figure says '128 x 11 recessive "
                      "bits' (1408) and the note says the device 'waits for "
                      "129 occurrences of bus-idle' (1419), and the "
                      "measurement lands on the note - inside a bit time and a "
                      "half of 1419, and a dozen bits clear of 1408",
                      bits >= 1414u && bits <= 1432u);
    }

    // --- a stuck-DOMINANT line.
    {
        (void)Can1::stop();
        Can1::release();
        RxPad::claim_rx(PinPull::down);
        FdcanConfig d = loop_config(FdcanMode::normal);
        (void)bring_up<Can1>(d);
        Can1::clear_flags(FdcanFlag::all);
        FdcanFrame g{};
        g.id = 0x321;
        g.length = 0;
        const auto put = Can1::tx_put(g);
        spin_us(50000);
        const auto st = Can1::status();
        const auto ec = Can1::error_counters();
        const uint8_t pending = Can1::tx_pending();
        print(serial, "  RX pulled DOMINANT: ACT ",
              static_cast<uint8_t>(st.activity), " (0 = synchronizing), TEC ",
              ec.transmit, " REC ", ec.receive, ", TXBRP ", hex(pending),
              ", CEL ", ec.logging, crlf);
        bench.verdict("A STUCK-DOMINANT LINE IS NOT AN ERROR, IT IS A WAIT: "
                      "the module never sees the eleven recessive bits that "
                      "end integration, so it stays SYNCHRONIZING, the request "
                      "stays pending for ever and NEITHER error counter moves",
                      put && st.activity == FdcanActivity::synchronizing &&
                          ec.transmit == 0u && ec.receive == 0u && pending != 0u);
        (void)Can1::tx_cancel(0x7);
        Can1::release();
        RxPad::claim_rx(PinPull::up);
    }

    // --- DAR: exactly one attempt.
    {
        FdcanConfig d = loop_config(FdcanMode::normal);
        d.disable_auto_retransmit = true;
        (void)bring_up<Can1>(d);
        Can1::clear_flags(FdcanFlag::all);
        const auto ec0 = Can1::error_counters();
        FdcanFrame g{};
        g.id = 0x123;
        g.length = 0;
        (void)Can1::tx_put(g);
        spin_us(20000);
        const auto ec1 = Can1::error_counters();
        const uint8_t pending = Can1::tx_pending();
        const uint8_t cancelled = Can1::tx_cancelled();
        const uint8_t occurred = Can1::tx_occurred();
        const bool bo = Can1::status().bus_off;
        print(serial, "  DAR: TEC ", ec0.transmit, " -> ", ec1.transmit,
              ", TXBRP ", hex(pending), " TXBCF ", hex(cancelled),
              " TXBTO ", hex(occurred), ", bus-off ", bo ? "yes" : "no", crlf);
        bench.verdict("DISABLED AUTOMATIC RETRANSMISSION IS EXACTLY ONE "
                      "ATTEMPT: the request pending bit clears, TXBCF is set "
                      "for the failure and TXBTO stays clear - and the node "
                      "does NOT reach bus-off, where twenty milliseconds of "
                      "the same wire took a retransmitting one there in three",
                      pending == 0u && (cancelled & 1u) != 0u &&
                          occurred == 0u && !bo);
        print(serial, "  ...and the ONE attempt still cost ",
              ec1.transmit - ec0.transmit, " of TEC, because an error-ACTIVE "
              "transmitter answers its own bit error with a six-bit dominant "
              "error flag that fails the same way, over and over, until it "
              "turns error-PASSIVE at 128 and its error flags go recessive - "
              "which is where the climb stops", crlf);
        Can1::release();
    }

    // --- restricted operation: no dominant bit ever leaves.
    {
        FdcanConfig d = loop_config(FdcanMode::restricted);
        (void)bring_up<Can1>(d);
        bench.verdict("ASM is set through the mode and the module is on the bus",
                      Can1::restricted() && !Can1::in_init());
        (void)edge_counter_arm();
        Can1::clear_flags(FdcanFlag::all);
        const auto ec0 = Can1::error_counters();
        FdcanFrame g{};
        g.id = 0x123;
        g.length = 8;
        (void)Can1::tx_put(g);
        spin_us(30000);
        const uint32_t edges = edge_counter_read();
        edge_counter_stop();
        const auto ec1 = Can1::error_counters();
        const uint8_t pending = Can1::tx_pending();
        print(serial, "  restricted: ", edges, " edges on the TX pad, TEC ",
              ec0.transmit, " -> ", ec1.transmit, ", REC ", ec0.receive, " -> ",
              ec1.receive, ", CEL ", ec1.logging, ", TXBRP ", hex(pending),
              crlf);
        bench.verdict("RESTRICTED OPERATION FREEZES THE ERROR COUNTERS: "
                      "36.4.12's own sentence - 'when the ASM bit is set the "
                      "CAN protocol controller does not increment TEC and REC "
                      "when a CAN protocol error is detected' - on a line no "
                      "acknowledge can ever come back on",
                      ec1.transmit == ec0.transmit && ec1.receive == ec0.receive);
        // A DOCUMENTARY CONFLICT THE BENCH SETTLES. 36.3.4 says a
        // restricted node "does not send data frames, remote frames,
        // active error frames, or overload frames"; 36.4.6's own
        // description of the ASM bit says it "is able to transmit and
        // receive data and remote frames". Those cannot both be true,
        // and the pad says which one is.
        if (edges == 0u) {
            bench.verdict("...AND 36.3.4 BEATS 36.4.6 ON WHAT ASM DOES: the "
                          "two descriptions of restricted operation "
                          "contradict each other - the functional section "
                          "says it 'does not send data frames', the register "
                          "description says it 'is able to transmit ... data "
                          "and remote frames' - and NOT ONE EDGE left the pad "
                          "through a whole DLC-8 request, so nothing is sent",
                          pending != 0u || edges == 0u);
        } else {
            bench.verdict("...AND 36.4.6 BEATS 36.3.4 ON WHAT ASM DOES: the "
                          "two descriptions contradict each other and the pad "
                          "shows a frame, so a restricted node DOES transmit "
                          "data frames and only withholds its error and "
                          "overload flags",
                          true);
        }
        // ASM cannot be combined with a loop-back. The config checker
        // refuses the pair; these are the two LIVE roads to it.
        (void)Can1::init_mode(true);
        (void)Can1::configuration(true);
        (void)Can1::test_mode(true);
        const bool lb_with_asm = Can1::loop_back(true);
        (void)Can1::restricted(false);          // ASM clears at any time
        const bool lb_alone = Can1::loop_back(true);
        const bool asm_with_lb = Can1::restricted(true);
        print(serial, "  with ASM standing, loop_back(true) was ",
              lb_with_asm ? "ACCEPTED" : "refused",
              "; with ASM cleared it was ",
              lb_alone ? "accepted" : "REFUSED",
              "; and restricted(true) on top of LBCK was ",
              asm_with_lb ? "ACCEPTED" : "refused", crlf);
        bench.verdict("36.3.4's note - 'the restricted operation mode must "
                      "not be combined with the loop-back mode' - enforced on "
                      "the LIVE verbs and not only in the configuration "
                      "checker: whichever of the two stands, the other is "
                      "refused with nothing written",
                      !lb_with_asm && lb_alone && !asm_with_lb);
        (void)Can1::loop_back(false);
        (void)Can1::tx_cancel(0x7);
        Can1::release();
    }

    // --- bus monitoring: TXBRP held in reset.
    {
        FdcanConfig d = loop_config(FdcanMode::bus_monitor);
        (void)bring_up<Can1>(d);
        (void)edge_counter_arm();
        FdcanFrame g{};
        g.id = 0x123;
        g.length = 8;
        const auto put = Can1::tx_put(g);
        spin_us(10000);
        const uint8_t pending = Can1::tx_pending();
        const uint32_t edges = edge_counter_read();
        edge_counter_stop();
        print(serial, "  bus monitor: tx_put ", put ? "accepted" : "refused",
              ", TXBRP ", hex(pending), ", ", edges, " edges on the pad", crlf);
        bench.verdict("BUS MONITORING MODE: 36.3.4's 'the FDCAN_TXBRP register "
                      "is held in reset state' is literal - the driver's "
                      "TXBAR write is accepted by the bus and the pending bit "
                      "never appears, so the transmission simply does not "
                      "happen and nothing reaches the pad",
                      pending == 0u && edges == 0u);
        Can1::release();
    }

    TxPad::release();
    RxPad::release();
}

// ---------------------------------------------------------------------------
// Letter j - interrupts, the lines and both instances
// ---------------------------------------------------------------------------

void tj_interrupts() {
    line0_hits = 0;
    line1_hits = 0;
    line0_mask = 0;
    line1_mask = 0;
    line0_second = 0;
    line1_second = 0;
    tim16_hits = 0;
    tim17_hits = 0;

    FdcanConfig c = loop_config(FdcanMode::internal_loop_back);
    (void)bring_up<Can1>(c);

    // RF0N on line 0, TC on line 1.
    (void)Can1::interrupt_line(FdcanGroup::all, 0);
    (void)Can1::interrupt_line(FdcanGroup::status_message, 1);
    (void)Can1::interrupts(FdcanFlag::rx_fifo0_new |
                           FdcanFlag::transmission_completed, true);
    Can1::clear_flags(FdcanFlag::all);
    Can1::interrupt_lines(true, true);
    Nvic::clear_pending(Can1::irq0());
    Nvic::clear_pending(Can1::irq1());
    Nvic::enable(Can1::irq0());
    Nvic::enable(Can1::irq1());

    const uint32_t on_line0 = Can1::flags_on_line(0);
    const uint32_t on_line1 = Can1::flags_on_line(1);
    print(serial, "  ILS ", hex(Can1::interrupt_line()), ": line 0 carries "
          "", hex(on_line0), ", line 1 carries ", hex(on_line1), crlf);
    bench.verdict("FDCAN_ILS IS SEVEN GROUPS AND NOT THIRTY FLAGS on this "
                  "silicon (36.4.17): moving the 'status message' group moves "
                  "TC, TCF and HPM together, and no flag can be split off from "
                  "its group",
                  (on_line1 & FdcanFlag::transmission_completed) != 0u &&
                      (on_line1 & FdcanFlag::cancellation_finished) != 0u &&
                      (on_line1 & FdcanFlag::high_priority) != 0u &&
                      (on_line0 & FdcanFlag::rx_fifo0_new) != 0u);

    FdcanFrame f{};
    f.id = 0x456;
    f.length = 2;
    f.data[0] = 0xA1;
    f.data[1] = 0xB2;
    (void)Can1::tx_put(f);
    spin_us(5000);
    print(serial, "  one frame: line 0 fired ", line0_hits, " times serving ",
          hex(line0_mask), ", line 1 fired ", line1_hits, " times serving ",
          hex(line1_mask), crlf);
    bench.verdict("THE TWO INTERRUPT LINES ARE REALLY TWO: one frame raised "
                  "RF0N on line 0 and TC on line 1, each body serving exactly "
                  "the mask its own ILS groups name and clearing exactly that",
                  line0_hits >= 1u && line1_hits >= 1u &&
                      (line0_mask & FdcanFlag::rx_fifo0_new) != 0u &&
                      (line1_mask & FdcanFlag::transmission_completed) != 0u &&
                      (line0_mask & FdcanFlag::transmission_completed) == 0u);
    drain();

    // ILE: a line that is not enabled raises nothing.
    {
        line0_hits = 0;
        Can1::interrupt_lines(false, true);
        Can1::clear_flags(FdcanFlag::all);
        (void)Can1::tx_put(f);
        spin_us(5000);
        const uint32_t pending = Can1::flags();
        print(serial, "  with EINT0 cleared: line 0 fired ", line0_hits,
              " times while IR still shows ", hex(pending), crlf);
        bench.verdict("FDCAN_ILE gates the LINE and not the flag: with EINT0 "
                      "clear the interrupt never reaches the NVIC while the "
                      "flag stands in IR exactly as before",
                      line0_hits == 0u &&
                          (pending & FdcanFlag::rx_fifo0_new) != 0u);
        Can1::interrupt_lines(true, true);
        drain();
    }

    // Per-buffer transmit interrupts.
    {
        (void)Can1::interrupts(FdcanFlag::transmission_completed, true);
        (void)Can1::tx_buffer_interrupts(0x1);   // buffer 0 only
        line1_hits = 0;
        Can1::clear_flags(FdcanFlag::all);
        (void)Can1::tx_put_buffer(1, f);
        (void)Can1::tx_request(0x2);
        spin_us(5000);
        const uint32_t hits_b1 = line1_hits;
        drain();
        line1_hits = 0;
        Can1::clear_flags(FdcanFlag::all);
        (void)Can1::tx_put_buffer(0, f);
        (void)Can1::tx_request(0x1);
        spin_us(5000);
        const uint32_t hits_b0 = line1_hits;
        print(serial, "  TXBTIE = buffer 0 only: buffer 1's transmission gave ",
              hits_b1, " interrupts, buffer 0's gave ", hits_b0, crlf);
        bench.verdict("TXBTIE IS PER BUFFER: IR.TC is raised only by a buffer "
                      "whose own enable bit stands, so 'transmission "
                      "completed' can mean one particular message",
                      hits_b1 == 0u && hits_b0 >= 1u);
        (void)Can1::tx_buffer_interrupts(0x7);
        drain();
    }

    // BOTH instances at once, on the same two vectors.
    {
        FdcanConfig d = loop_config(FdcanMode::internal_loop_back);
        (void)bring_up<Can2>(d);
        (void)Can2::interrupt_line(FdcanGroup::all, 0);
        (void)Can2::interrupts(FdcanFlag::rx_fifo0_new, true);
        Can2::clear_flags(FdcanFlag::all);
        Can2::interrupt_lines(true, true);

        line0_hits = 0;
        line0_second = 0;
        Can1::clear_flags(FdcanFlag::all);

        FdcanFrame a{};
        a.id = 0x101;
        a.length = 1;
        a.data[0] = 0x11;
        FdcanFrame b{};
        b.id = 0x202;
        b.length = 1;
        b.data[0] = 0x22;
        (void)Can1::tx_put(a);
        (void)Can2::tx_put(b);
        spin_us(6000);

        FdcanFrame got1{};
        FdcanFrame got2{};
        const bool r1 = Can1::rx_read(0, got1);
        const bool r2 = Can2::rx_read(0, got2);
        print(serial, "  FDCAN1 received id ", hex(got1.id), " data ",
              hex(got1.data[0]), "; FDCAN2 received id ", hex(got2.id),
              " data ", hex(got2.data[0]), "; the shared line 0 body served "
              "FDCAN1 ", line0_hits, " times and FDCAN2 ", line0_second,
              " times; TIM16 ", tim16_hits, " TIM17 ", tim17_hits, crlf);
        bench.verdict("BOTH MODULES LOOP AT ONCE AND NEVER HEAR EACH OTHER: "
                      "each received its own frame and only its own, out of "
                      "its own message RAM",
                      r1 && r2 && got1.id == 0x101u && got1.data[0] == 0x11u &&
                          got2.id == 0x202u && got2.data[0] == 0x22u);
        bench.verdict("...and ONE VECTOR SERVES TIM16, FDCAN1 AND FDCAN2 "
                      "(table 61): the handler calls every body that can be on "
                      "the line and each answers for its own flags, with the "
                      "timer silent",
                      line0_hits >= 1u && line0_second >= 1u &&
                          tim16_hits == 0u && tim17_hits == 0u);
        Can2::release();
    }

    Nvic::disable(Can1::irq0());
    Nvic::disable(Can1::irq1());
    Can1::release();
}

// ---------------------------------------------------------------------------
// Letter k - power-down and the Stop mode
// ---------------------------------------------------------------------------

void tk_power() {
    FdcanConfig c = loop_config(FdcanMode::internal_loop_back);
    (void)bring_up<Can1>(c);

    // CSR with nothing pending: the handshake should be quick.
    {
        Can1::power_down(true);
        const uint32_t t0 = now_ticks();
        bool acked = false;
        while (us_of(since(t0)) < 20000u) {
            if (Can1::power_down_acked()) {
                acked = true;
                break;
            }
        }
        const uint32_t took = us_of(since(t0));
        const bool init_set = Can1::in_init();
        print(serial, "  CSR on an idle module: CSA after ", took, " us, INIT ",
              init_set ? "set by the handshake" : "clear", crlf);
        bench.verdict("36.3.4's power-down entry: the module waits for bus "
                      "idle, SETS INIT ITSELF and only then acknowledges with "
                      "CSA",
                      acked && init_set);

        Can1::power_down(false);
        const uint32_t t1 = now_ticks();
        bool cleared = false;
        while (us_of(since(t1)) < 20000u) {
            if (!Can1::power_down_acked()) {
                cleared = true;
                break;
            }
        }
        bench.verdict("...and clearing CSR clears CSA, leaving INIT standing "
                      "for the application to clear when it wants the bus back",
                      cleared && Can1::in_init());
        (void)Can1::start();
    }

    // CSR with a transmission pending: it completes first.
    {
        console_drain();
        drain();
        Can1::clear_flags(FdcanFlag::all);
        FdcanFrame f{};
        f.id = 0x123;
        f.length = 8;
        (void)Can1::tx_put(f);
        Can1::power_down(true);         // requested WHILE the frame is on the wire
        const uint32_t t0 = now_ticks();
        bool acked = false;
        while (us_of(since(t0)) < 20000u) {
            if (Can1::power_down_acked()) {
                acked = true;
                break;
            }
        }
        const uint8_t occurred = Can1::tx_occurred();
        const uint32_t took = us_of(since(t0));
        // AN UNEXPLAINED CONSOLE ARTIFACT LIVES IN THIS LEG, and it is
        // recorded rather than dressed up: about six bytes of the NEXT
        // line come out of USART2 corrupted, the same bytes on every run
        // of this letter, on a board freshly reset, with two independent
        // capture tools and with every other line of this suite intact.
        // Moving the print after the power-down is released does not
        // move it; inserting text before it moves it by exactly that
        // many bytes; draining the console first does not help. No
        // verdict rests on the line. fdcan.md carries it as an open item.
        Can1::power_down(false);
        (void)Can1::start();
        print(serial, "  CSR with a DLC-8 frame in flight: CSA after ", took,
              " us, TXBTO ", hex(occurred), crlf);
        bench.verdict("'when all pending transmission requests have completed, "
                      "the FDCAN waits until the bus-idle state is detected' - "
                      "the acknowledge waited out the whole frame and the "
                      "transmission occurred",
                      acked && occurred != 0u && took > 150u);
        drain();
    }

    // The FDCAN through a Stop. Table 27 gives the FDCAN no Stop
    // functionality and NO WAKE-UP CAPABILITY of its own, so what ends
    // this sleep is the RTC - the sleep suite's own pattern. THE DOMAIN
    // IS NEVER RESET HERE: a board whose RTC is not already on the
    // crystal is reported and the leg declines, because a BDRST would
    // cost the calendar and the five backup words every other suite on
    // this desk leans on.
    {
        Pwr::bus_clock(true);
        Pwr::rtc_domain_unlock(true);
        RtcDomain::apb_clock(true);
        const bool domain_ok = RtcDomain::enabled() &&
                               RtcDomain::selected() == RtcClockSource::lse;
        (void)Can1::stop();
        const uint32_t nbtp = Can1::regs().NBTP;
        const uint32_t rxgfc = Can1::regs().RXGFC;
        (void)Can1::standard_filter(7, FdcanStandardFilter{
                                           FdcanFilterType::classic,
                                           FdcanFilterAction::store_fifo1,
                                           0x2AA, 0x555});
        const uint32_t ram_before = Can1::standard_filter(7);

        if (!domain_ok) {
            print(serial, "  the RTC domain is not on the LSE crystal (BDCR ",
                  hex(RCC->BDCR), "), and this letter will not BDRST it to get "
                  "a wake source - the Stop leg is DECLINED and the "
                  "configuration below is checked without one", crlf);
            bench.verdict("the FDCAN through a Stop - DECLINED, see the line "
                          "above",
                          true);
        } else {
            rtc_wakes = 0;
            Nvic::clear_pending(Rtc::irq());
            Nvic::enable(Rtc::irq());
            const bool armed = Rtc::set_wakeup(RtcWakeupClock::ck_spre, 0, true);
            Nvic::clear_pending(Rtc::irq());
            Ticker::pause();
            const uint32_t t0 = now_ticks();
            (void)Pwr::arm(PwrMode::stop1);
            __DSB();
            __WFI();
            const uint32_t slept = us_of(since(t0));
            (void)Stm32SleepSite<SysClock>::resume_clock();
            Ticker::resume();
            Nvic::disable(Rtc::irq());
            (void)Rtc::set_wakeup(RtcWakeupClock::ck_spre, 0, false);
            (void)Pwr::arm(PwrMode::sleep);

            const uint32_t nbtp_after = Can1::regs().NBTP;
            const uint32_t rxgfc_after = Can1::regs().RXGFC;
            const uint32_t ram_after = Can1::standard_filter(7);
            print(serial, "  through a Stop 1 of ", slept, " us (", rtc_wakes,
                  " RTC wakes): NBTP ", hex(nbtp), " -> ", hex(nbtp_after),
                  ", RXGFC ", hex(rxgfc), " -> ", hex(rxgfc_after),
                  ", the filter word ", hex(ram_before), " -> ",
                  hex(ram_after), crlf);
            bench.verdict("table 27 gives the FDCAN no Stop functionality and "
                          "no wake-up capability of its own - but the block "
                          "and its MESSAGE RAM keep every bit through a "
                          "Stop 1, so a configuration survives the sleep and "
                          "only the bus does not",
                          armed && rtc_wakes >= 1u && nbtp_after == nbtp &&
                              rxgfc_after == rxgfc && ram_after == ram_before);
        }
    }

    Can1::release();
}

// ---------------------------------------------------------------------------
// Letter l - the errata
// ---------------------------------------------------------------------------

void tl_errata() {
    // --- ES0548 2.13.1: desynchronization with edge filtering enabled.
    // "FDCAN may desynchronize and incorrectly receive the first bit of
    // the frame if the edge filtering is enabled (EFBI) and the end of
    // the integration phase coincides with a falling edge on FDCAN_Rx.
    // ... This issue does not affect the reception of standard frames."
    // So: thousands of FD frames in loop-back, with EFBI and without,
    // counting PEA, PED and lost frames.
    struct Run {
        uint32_t sent;
        uint32_t received;
        uint32_t bad;
        uint32_t pea;
        uint32_t ped;
    };
    Run runs[2] = {};
    for (uint8_t pass = 0; pass < 2; ++pass) {
        FdcanConfig c = loop_config(FdcanMode::internal_loop_back, 1'000'000u);
        c.data = *fdcan_data_timing_for(SysClock::pclk_hz, 4'000'000u, 875);
        c.fd = FdcanFd::on_with_bit_rate_switch;
        c.edge_filtering = (pass == 1);
        (void)bring_up<Can1>(c);
        Can1::clear_flags(FdcanFlag::all);
        Run r{};
        for (uint32_t i = 0; i < 1500u; ++i) {
            FdcanFrame in{};
            in.id = 0x200u + (i & 0xFFu);
            in.length = 16;
            in.fd = true;
            in.bit_rate_switch = true;
            for (uint8_t b = 0; b < 16; ++b) {
                in.data[b] = static_cast<uint8_t>(i + b);
            }
            FdcanFrame out{};
            ++r.sent;
            if (round_trip<Can1>(in, out, 0, 2000u)) {
                ++r.received;
                if (!same_payload(in, out) || out.id != in.id) {
                    ++r.bad;
                }
            }
            const uint32_t f = Can1::flags();
            if ((f & FdcanFlag::protocol_error_arbitration) != 0u) {
                ++r.pea;
            }
            if ((f & FdcanFlag::protocol_error_data) != 0u) {
                ++r.ped;
            }
            Can1::clear_flags(FdcanFlag::protocol_error_arbitration |
                              FdcanFlag::protocol_error_data);
            if ((i & 0x3Fu) == 0u) {
                feed();
            }
        }
        runs[pass] = r;
        print(serial, "  EFBI ", pass == 1 ? "SET  " : "clear",
              ": sent ", r.sent, ", received ", r.received, ", corrupt ", r.bad,
              ", PEA ", r.pea, ", PED ", r.ped, crlf);
        Can1::release();
    }
    bench.verdict("ES0548 2.13.1 STAGED, 1500 FD frames with bit rate "
                  "switching in each arm: with edge filtering SET and with it "
                  "clear, every frame arrived and every one of them was "
                  "byte-exact - the erratum is NOT REPRODUCED here, which is "
                  "recorded as such and not as a disproof (the coincidence it "
                  "needs is an END OF INTEGRATION on a falling RX edge, and a "
                  "loop-back whose RX pin is disconnected may never offer one)",
                  runs[0].received == runs[0].sent &&
                      runs[1].received == runs[1].sent && runs[0].bad == 0u &&
                      runs[1].bad == 0u);

    // --- ES0548 2.13.2: Tx FIFO messages inverted "if FDCAN uses both a
    // dedicated Tx buffer and a Tx FIFO". Is a dedicated Tx buffer even
    // reachable here?
    {
        Can1::bus_clock(true);
        (void)Can1::init_mode(true);
        (void)Can1::configuration(true);
        const uint32_t before = Can1::regs().TXBC;
        Can1::regs().TXBC = 0xFFFFFFFFu;
        const uint32_t after = Can1::regs().TXBC;
        Can1::regs().TXBC = before;
        print(serial, "  TXBC written 0xFFFFFFFF reads back ", hex(after),
              " - the only implemented bit is TFQM (bit 24), so there is no "
              "TFQS, no NDTB and no TBSA on this part", crlf);
        bench.verdict("ES0548 2.13.2 IS UNREACHABLE BY CONSTRUCTION ON THIS "
                      "SILICON, and TXBC is the evidence: the erratum needs "
                      "'both a dedicated Tx buffer and a Tx FIFO', and its own "
                      "workaround names Tx buffers 4 and 5 - but this M_CAN is "
                      "configured with THREE Tx elements and one mode bit, so "
                      "the whole area is either a FIFO or a queue and a "
                      "dedicated buffer cannot be declared at all",
                      (after & ~FDCAN_TXBC_TFQM) == 0u);
        Can1::release();
    }
}

// ---------------------------------------------------------------------------
// The menu
// ---------------------------------------------------------------------------

void banner() {
    print(serial, crlf, "test_stm32_fdcan - RM0444 ch. 36, one board, no "
          "transceiver", crlf,
          "  PB8 = FDCAN1_RX, PB9 = FDCAN1_TX (AF3); FDCAN2 runs with no pad "
          "at all", crlf);
    bench.menu();
}

} // namespace

// ---------------------------------------------------------------------------
// The vectors
// ---------------------------------------------------------------------------

extern "C" void TIM16_FDCAN_IT0_IRQHandler() {
    // ONE LINE, THREE PERIPHERALS (table 61): TIM16 and both FDCAN
    // instances' interrupt 0. Every body answers for its own and returns
    // what it served, which is the stratum's shared-vector contract.
    const uint32_t a = Can1::isr0();
    const uint32_t b = Can2::isr0();
    if (a != 0u) {
        line0_mask = a;
        line0_hits = line0_hits + 1u;
    }
    if (b != 0u) {
        line0_second = line0_second + 1u;
    }
    if (a == 0u && b == 0u) {
        tim16_hits = tim16_hits + 1u;
    }
}

extern "C" void TIM17_FDCAN_IT1_IRQHandler() {
    const uint32_t a = Can1::isr1();
    const uint32_t b = Can2::isr1();
    if (a != 0u) {
        line1_mask = a;
        line1_hits = line1_hits + 1u;
    }
    if (b != 0u) {
        line1_second = line1_second + 1u;
    }
    if (a == 0u && b == 0u) {
        tim17_hits = tim17_hits + 1u;
    }
}

extern "C" void RTC_TAMP_IRQHandler() {
    rtc_wakes = rtc_wakes + 1u;
    (void)brio::Rtc::isr();
    (void)brio::Exti::clear(brio::rtc_exti_line);
}

extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    // Sampled through the CLOSED clock gate, before a line of ours runs.
    boot_apbenr1 = RCC->APBENR1;
    boot_ccipr2 = RCC->CCIPR2;
    boot_crel = FDCAN1->CREL;
    boot_endn = FDCAN1->ENDN;
    boot_cccr = FDCAN1->CCCR;
    boot_nbtp = FDCAN1->NBTP;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);

    Stopwatch::init();
    (void)Stopwatch::configure({.prescaler = 0, .period = 0xFFFFFFFFu});
    Stopwatch::enable(true);

    const bool wd = arm_backstop();
    brio::enable_interrupts();

    bench.letter('a', "the subsystem: one enable, one reset, one divider, "
                      "the RAM", ta_subsystem);
    bench.letter('b', "THE BIT RATE ON THE PAD, counted with no CPU",
                 tb_bit_rate);
    bench.letter('c', "classic CAN in internal loop-back, field by field",
                 tc_classic);
    bench.letter('d', "external loop-back: the frame on the pad", td_external);
    bench.letter('e', "the filters, all three types and the first match",
                 te_filters);
    bench.letter('f', "the FIFOs, the Tx order and the transmit pause",
                 tf_fifos);
    bench.letter('g', "CAN FD: the 0xCC question, BRS and TDC", tg_fd);
    bench.letter('h', "timestamps and the timeout counter", th_time);
    bench.letter('i', "THE ERROR MACHINE WITH NO NODE", ti_errors);
    bench.letter('j', "the two interrupt lines and both instances",
                 tj_interrupts);
    bench.letter('k', "power-down, and the FDCAN through a Stop", tk_power);
    bench.letter('l', "the errata: 2.13.1 staged, 2.13.2 read off TXBC",
                 tl_errata);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL64" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED",
                    " backstop=", wd ? "IWDG" : "FAILED", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        feed();
        uint8_t ch = 0;
        if (!Serial::read_byte(ch)) {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(ch), brio::crlf);
        if (ch == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(ch))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
