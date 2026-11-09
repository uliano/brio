// test_stm32_i2c - the reference bench suite for the STM32G0's I2C
// block: stm32g0/i2c.hpp over RM0444 ch. 32, both roles, in ONE image on
// ONE board.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// TWO INSTRUMENTS, ONE PAIR OF PADS, AND THE SUITE ASKS THE WIRE WHICH
// ONE IS ON THE DESK.
//
// (1) THE BOARD'S OWN SELF-LINK: I2C1 the host and I2C2 the client,
// two wires between
// them, each with a 2.2 kOhm pull-up to 3V3 -
//
//   SCL   PB8  AF6  <->  PA11 AF6
//   SDA   PB9  AF6  <->  PA12 AF6
//
// - so every wire verdict is a real bus tenure between two peripherals
// and not a loop-back. There is no third pad on either net, which is the
// one thing that shapes the instruments below. Letters b..m are this
// instrument's.
//
// (2) THE PEER BUS: the SAME two pads and the SAME pull-ups reach a
// SECOND BOARD running `twi_peer`, and EITHER instrument is valid - the
// peer names itself in its `ident` (fw 0x01xx = the AVR, 0x02xx = the
// SAM C21, 0x03xx = another STM32G0), and the desk says which one
// is fitted -
//
//   SCL   PB8  AF6  <->  a SAM C21's PA23 SERCOM3 PAD[1], or PB8 AF6
//   SDA   PB9  AF6  <->  a SAM C21's PA22 SERCOM3 PAD[0], or PB9 AF6
//
// - an stm32g0 peer answers on the SAME PIN NAMES this board hosts on,
// plus a dedicated GND, both boards at 3.3 V. The
// instrument is commanded IN BAND over the bus under test, over
// avrxt/src/apps/twi_link.hpp included by relative path - one source of
// truth for the wire format, whatever the peer's architecture - and its
// command-mode client answers ONE address (0x6B) with no general call
// and no mask, which is why nothing the wireless letters do can wake it.
// Letters n..t are this instrument's (s and t want the self-link too: its
// I2C2 is their third node).
//
// THE TWO MAY BE ON THE DESK AT ONCE: I2C2's pads and the peer's tap the
// same two nodes, so a desk carries the self-link, the peer, or both.
// main() PROBES the self-link before any letter runs - PB8 and PB9
// driven both ways against the far pad's own internal pull - and
// letters b..l, u, v, B, C, D, x and y skip themselves (TestBench::skip(),
// the reason printed and counted apart from the verdicts) when the probe
// says no. THE PEER LETTERS ARE A
// SECOND SET under their own all-key, `w`: `z` stays the single-board
// set, runnable with the far board on its dark console, and `w` needs
// `twi_peer` there and FAILS without it. With both on the wire neither
// instrument disturbs the other - the peer's command client answers
// 0x6B alone, which no self-link letter sends, and I2C2 is released
// while the peer letters run.
//
// HOW A BUS RATE IS MEASURED WITH NO PAD TO SPARE. A timer capture or an
// EXTI count would need a pad on SCL, and both ends of SCL are already
// alternate functions. So the stopwatch is the CPU's own cycle counter
// around a whole tenure, and the arithmetic is the wire's: a tenure of N
// bytes is 9 x (N + 1) SCL periods (the address byte and its ACK, then
// each data byte and its ACK) plus a START and a STOP. With the CLIENT
// PUT IN NOSTRETCH MODE it holds the clock for nothing, so the measured
// period is the CONTROLLER'S OWN - which is what the chooser predicts -
// and with the client stretching normally the same measurement prices
// the stretch. Letter d does both and says which is which.
//
// THE CLIENT RUNS ON ITS OWN INTERRUPT under BRIO_STM32G0_I2C2_HANDLER
// (I2C2 and I2C3 share one line on this part, and the reserve derives
// the name), so a host driven from main context and a client answering
// under its own vector are two independent halves of one bus on one
// core.
//
// THE CLOCK IS DYNAMIC and every letter but d runs at its first rung
// (PLL 64 MHz). The console sits on HSI16 so its divisor never moves.
// THE CLIENT'S KERNEL IS HSI16 THROUGHOUT: it is the only kernel clock
// the wake from Stop accepts (32.4.16), and it keeps the client legal at
// core rates ES0548 2.10.1 refuses.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the reserve against the header, the reset
//      values, what a PE clear resets and what it keeps, the enable
//      protection MEASURED field by field, the SMBus column asked of the
//      SILICON, the timing arithmetic and the refusals
//   b  THE LINK: write, read, write-then-read with the repeated START
//      counted on the client, and 0x7A and the general call refused
//      inside start() with the client, answering the general call,
//      matching nothing
//   c  THE VOCABULARY ON THE WIRE: nack_addr, nack_data, and a fault
//      staged from the client's own pad
//   d  THE THREE SPEEDS MEASURED, the Fm+ drive, and the whole ladder
//      through the dynamic clock with the refusals it produces - each
//      controller rate judged against the RATE ASKED: SCL never above it
//      and within a quarter below it (a self-link's
//      target follows the controller's clock and cannot tell a wrong
//      TIMINGR) - but for the 2 MHz core's rung, where the stopwatch
//      times the two vectors and the rate rests on the 16 MHz rung's
//      timed TIMINGR, the same register on the same kernel
//   e  CLOCK STRETCHING: commanded, priced, and NOSTRETCH's underrun
//   f  10-BIT ADDRESSING both ways, the second address under every mask,
//      ADDCODE and DIR
//   g  THE FILTERS: the analog one off and on, the digital one at four
//      depths, measured as the timing shift they really are
//   h  RELOAD PAST 255 BYTES through TCR, AUTOEND against a software
//      STOP, and the host's DMA engines - a write, a read, a register
//      read, a write of one (its preload alone) and a read of one - and the
//      half with no buffer they leave to the pump right after them;
//      the 300-byte write's and the engined write's SCL against 400 kHz
//   i  SMBus: the PEC end to end, PECERR staged, and THE TIME-OUTS -
//      whose hold does each of the three really police
//   j  THE WAKE FROM STOP, armed: everything around it on the self-link
//      (the Stop itself is letter t's, woken from the other chip)
//   k  THE STUCK BUS: a client holding SDA, and unstick() counting the
//      clocks until it lets go
//   l  THE KERNEL: I2cBus (= BusMaster) over I2cHost with the client
//      answering - and the per-bus timeout with recover()
//   m  the errata: 2.10.1 as the refusals it is, 2.10.2 as what the run
//      has counted
//   u  THE ENGINE NAMED (Request::engine, util/i2c_bus.hpp): a Request
//      naming nothing on the pump, `interrupt` on a long half touching no
//      channel, `dma` on halves of one byte and on a register read, a half
//      with no buffer left to the pump under `dma`, and `dma` on the
//      engineless host refused with nothing on the wire
//   v  A REFUSED WRITE, position by position, on either engine: the address,
//      a write of one, the first byte, the second, a later one, a write of
//      two; the client's count and bytes judged, and the next request whole
//      byte for byte with no init() and no PE cycle between them; a channel
//      starved under a refused byte; and a write that does not flush, the
//      control that shows the byte a refusal leaves in TXDR
//   B  ES0418 2.11.6's band by name: HSI16 the kernel, PCLK at 44 MHz, a
//      ratio of 2.75. The host's writes judged byte for byte, and a row with
//      TXDR left empty by the resource's verbs, its rate printed: what says
//      whether this silicon has the erratum
//   C  THE RECEIVE CHANNEL AT THE STOPF ENTRY, staged: what says a beat is in
//      memory, a request taken down and a channel stopped against a beat, the
//      stray beat in each order; the last byte's request taken away and let up in
//      the entry, under beats that end inside it: reads and write-then-reads of 1,
//      2 and 8 at 100 kHz, 400 kHz and 1 MHz, a read, a write and a refused write
//      whole behind them, and a byte nobody has answered i2c_dma_fault
//   D  THE SAME UNDER LOAD: channels above the host's, paced and copies, at 400 kHz
//      and 1 MHz and on the slower rungs: every read exact, how often the channel
//      was running at the STOPF entry, and its lag against the bus's window
//   n  THE PEER: the twi_link command channel to the peer board, its
//      ident and ten command round trips
//   o  the tenure shapes against the peer - write, read, write-then-read
//      with the repeated START counted from the far end, and 0x7A and the
//      general call refused inside start() with nothing on the wire
//   p  the vocabulary against the peer (nack_addr from a deaf client,
//      nack_data at a commanded byte) and commanded stretching priced
//   q  the three speeds against a second chip, Sm's and Fm's SCL
//      against the rate asked
//   r  THE KERNEL against the peer: I2cBus (= BusMaster) over I2cHost,
//      the NACK in its place, the rejection, both votes, and the wedge
//      the PEER holds answered by the per-bus timeout
//   s  TWO CONTROLLERS, ONE STOP: this board's held START and the peer's,
//      released together by I2C2's STOP - they serialize, both tenures
//      whole, and no START goes out into the third node's tenure
//   t  THE WAKE FROM STOP by the peer's address: I2C2 the client in Sleep
//      and then in Stop 1, woken by a write from the other chip, the
//      sleep timed by the RTC against the peer's lead-in
//
// NOTHING WRITES FLASH, no option byte is touched, and the RTC domain is
// never reset (letter t claims the LSE only when the domain runs on it
// already or has no clock at all). The pads this suite moves are the four
// of the I2C self-link plus the console's two.
//
// build: boards = g0b1re,g071rb,g031k8
// build: groups = abcdefghijklmuxy,nopqrst,vBCD
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "kernel/tenuto.hpp"
#include "kernel/post.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/i2c.hpp"
#include "stm32g0/lpuart.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/pwr.hpp"
#include "stm32g0/rtc.hpp"
#include "stm32g0/sleep.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/test_wait.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED. twi_link.hpp
// is pure encoding - it names no register and includes nothing of brio -
// so every architecture on this link compiles the same file.
#include "../../../avrxt/src/apps/twi_link.hpp"

namespace {

using namespace brio;

using P = Stm32g0Platform<>;

// ---------------------------------------------------------------------------
// The rates, the users, the clock
// ---------------------------------------------------------------------------

using Fast = Clock<ClockSource::pll, 64'000'000>;
using Mid = Clock<ClockSource::internal, 16'000'000, PowerRegime::range2>;
using Slow = Clock<ClockSource::internal, 2'000'000, PowerRegime::low_power_run>;
/// Letter B's rate alone: PCLK against HSI16 is 2.75, inside ES0418 2.11.6's band.
using Band = Clock<ClockSource::pll, 44'000'000>;

constexpr UartOptions console_opts{.kernel_clock = UsartClock::hsi16};

// AND ON A PART WHOSE USART2 HAS NO KERNEL-CLOCK MULTIPLEXER, THAT COSTS
// A DIFFERENT PERIPHERAL. Table 183's FULL/BASIC split moves with the
// part: where USART2 is BASIC it runs on PCLK, full stop - its divisor
// would follow every switch this suite makes and the report would be a
// function of its own subject. The instance that always has a
// multiplexer is the LPUART (34.4.6), and LPUART1_TX/RX reach THE SAME
// TWO PADS at AF6. So the console moves to LPUART1 exactly where USART2's
// multiplexer is missing. The HANDLER has to be chosen by the
// preprocessor, which cannot call a constexpr function, so the same
// header symbol the reserve probes for usart_has_clock_select(2) is
// probed here and the static_assert keeps the two answers one answer.
#if defined(RCC_CCIPR_USART2SEL_Pos)
#define BRIO_SUITE_CONSOLE_HANDLER BRIO_STM32G0_USART2_HANDLER
constexpr bool console_on_lpuart = false;
constexpr PinFunction console_af = PinFunction::af1;
#else
#define BRIO_SUITE_CONSOLE_HANDLER BRIO_STM32G0_LPUART1_HANDLER
constexpr bool console_on_lpuart = true;
constexpr PinFunction console_af = PinFunction::af6;
#endif
static_assert(console_on_lpuart == !usart_has_clock_select(2),
              "the console's instance and the reserve's own column must be "
              "one answer");

constexpr UartPins console_pins{
    .tx = {'A', 2, console_af},
    .rx = {'A', 3, console_af},
};
using Serial = std::conditional_t<
    console_on_lpuart,
    LpUart<1, console_pins, 64, 512, NoDmaEngine, NoDmaEngine, console_opts>,
    Uart<2, console_pins, 64, 512, NoDmaEngine, NoDmaEngine, console_opts>>;

// The self-link's pads (DS13560 tables 13 and 15). Both pairs are AF6;
// PB8/PB9 are plain FT_f pads and PA11/PA12 are FT_fus - "supplied from
// VDDIO2 only", which on this board is tied to VDD.
constexpr I2cPins host_pins{
    .scl = {'B', 8, PinFunction::af6},
    .sda = {'B', 9, PinFunction::af6},
};
constexpr I2cPins client_pins{
    .scl = {'A', 11, PinFunction::af6},
    .sda = {'A', 12, PinFunction::af6},
};

using H = I2c<1>;
using C = I2c<2>;
using Host = I2cHost<1, host_pins>;
using Peer = I2cClient<2, client_pins>;

// The engined host of letter h, over the SAME instance and pads: a
// second set of statics, one live at a time.
using HostTx = DmaTxEngine<1, 1>;
using HostRx = DmaRxEngine<1, 2>;
using DmaHost = I2cHost<1, host_pins, HostTx, HostRx>;

// THE LIVE ENGINE LAST IN THE FAN-OUT. A rate switch rebases both sets
// of statics, and each re-applies ITS cached speed to the one TIMINGR they
// share - so the last one rebased decides what the register holds, while
// the other's cache still names its own. With DmaHost last, letter h's
// 400 kHz stood in TIMINGR after every later switch and Host, caching
// 100 kHz, never rewrote it: letter d's 16 MHz rung then read its 100 kHz
// tenure at 3216 thousandths of the rate (measured, whenever h had run
// earlier in the same boot). Host is the engine every letter but h drives,
// and h switches no rate.
using SysClock = DynamicClock<Rates<Fast, Mid, Slow, Band>, Ticker, Serial, DmaHost, Host>;
constexpr SysClock clock;
static_assert(SysClock::rate_count == 4);

/// The kernel ticker's cycles() as a CycleRuler (util/bench.hpp): the tick
/// count and SysTick's position composed across the reload, so interrupts
/// must be enabled while it times (a tick handler that cannot run stops
/// it). hz() names the boot rung's 64 MHz, a constant as the shared waits
/// need it: on the lower rungs (letters d, B and D, 44, 16 and 2 MHz) a
/// budget on it lasts longer in wall time, which a time-out may, and no
/// fixed span (spin_us) runs there.
struct Ruler {
    static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return Fast::hz; }
};
static_assert(CycleRuler<Ruler>);

constexpr uint8_t r_fast = 0;
constexpr uint8_t r_mid = 1;
constexpr uint8_t r_slow = 2;
constexpr uint8_t r_band = 3;

Serial serial;
// TWO SETS: `z` the single-board letters, runnable with the far board on
// its dark console; `w` the peer letters n..t, which need `twi_peer` on
// the far board and FAIL without it.
TwoKeyTestBench<Serial, 'w', 28> bench;

/// The client's own address, and one it must be deaf to.
constexpr uint8_t peer_addr = 0x42;
constexpr uint8_t nobody_addr = 0x23;
constexpr uint16_t peer_addr10 = 0x155;

using SclPin = Pin<'B', 8>;
using SdaPin = Pin<'B', 9>;
using PeerSclPin = Pin<'A', 11>;
using PeerSdaPin = Pin<'A', 12>;

// ---------------------------------------------------------------------------
// Instruments
// ---------------------------------------------------------------------------

uint32_t cycles_us(uint32_t cycles) {
    const uint32_t per_us = SysClock::hz() / 1'000'000UL;
    return per_us == 0u ? 0u : cycles / per_us;
}

/// Nanoseconds, for the sub-microsecond periods a fast bus has.
uint32_t cycles_ns(uint32_t cycles) {
    const uint32_t per_mhz = SysClock::hz() / 1'000'000UL;
    return per_mhz == 0u ? 0u
                         : static_cast<uint32_t>((static_cast<uint64_t>(cycles) * 1000u) / per_mhz);
}

/// The console's last byte out before a stretch with nothing timed in it;
/// the answer dropped: a wedged console only lengthens what follows, and
/// the letters that time a window ask console_quiet() first.
void console_drain() { (void)brio::console_drain<Ruler, Serial>(); }

/// Before a timed window a derived bound judges: the console's last byte
/// out, or the letter skips - a transmit interrupt inside the window would
/// lengthen the span the bound describes.
bool console_quiet() {
    if (brio::console_drain<Ruler, Serial>()) {
        return true;
    }
    bench.skip("the console did not drain: the timed window would carry its interrupt");
    return false;
}

/// A HOLD INSIDE AN INTERRUPT: the client's commanded stretch, spent in its
/// own vector. Not on the ruler: a tick handler cannot preempt a vector of
/// its own priority, so across a SysTick period VAL wraps while the tick
/// count does not, the composed count runs backwards and a wait on it never
/// ends (measured: a 1000 us stretch on it hung the board at exactly that
/// length). delay_us() reads VAL alone with the wrap folded in, at the
/// factor of the rung in force; it refuses a millisecond or more, so the
/// hold goes in chunks it admits, and its answer is dropped because SysTick
/// runs and no chunk reaches the cap.
void spin_us_isr(uint32_t us) {
    while (us >= 500u) {
        (void)delay_us(clock, 500u);
        us -= 500u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

// ---------------------------------------------------------------------------
// The client's pump
// ---------------------------------------------------------------------------

constexpr uint16_t peer_cap = 320;

volatile bool peer_live = false;
volatile uint8_t peer_rx[peer_cap];
volatile uint16_t peer_rx_n = 0;
/// The count the previous address match closed: what a write half took
/// before a repeated START's match started the count again.
volatile uint16_t peer_rx_prev = 0;
uint8_t peer_answers[peer_cap];
volatile uint16_t peer_tx_n = 0;
uint8_t peer_filler = 0xA5;
volatile uint16_t peer_addr_hits = 0;
volatile uint16_t peer_stops = 0;
volatile uint16_t peer_nacks = 0;
volatile uint8_t peer_last_code = 0;
volatile bool peer_last_dir_read = false;
volatile uint32_t peer_errors_seen = 0;
volatile uint16_t peer_overruns = 0;
/// Commanded clock stretching: microseconds held on every data event.
volatile uint16_t peer_stretch_us = 0;
/// Commanded refusal: NACK the k-th received byte (1-based; 0 = never).
/// Needs byte control, which letter c arms.
volatile uint16_t peer_nack_at = 0;
/// The bytes byte control has decided this tenure. TCR comes once a byte and
/// RXNE, the other count, may be served on either side of it.
volatile uint16_t peer_decided = 0;

uint8_t peer_next_answer() {
    const uint16_t i = peer_tx_n;
    peer_tx_n = static_cast<uint16_t>(i + 1u);
    return i < peer_cap ? peer_answers[i] : peer_filler;
}

/// The two ISRs' own accounting - a diagnostic letter y reads, and the
/// thing that names a storm instead of leaving a wedged board.
volatile uint32_t peer_isr_entries = 0;
volatile uint32_t host_isr_entries = 0;
volatile uint32_t peer_trace[12];
volatile uint8_t peer_trace_n = 0;
volatile uint32_t host_trace[12];
volatile uint8_t host_trace_n = 0;
volatile bool trace_isrs = false;

/// A STORM BUDGET, and the reason this suite could be debugged at all: a
/// client pump that cannot clear what it is being asked about starves
/// main so completely that nothing is ever printed. Past this many
/// entries without a fresh arming the pump SILENCES ITSELF and records
/// what it was looking at, so the letter can say which flag did it.
constexpr uint32_t peer_isr_budget = 30'000;
volatile uint32_t peer_storm_flags = 0;
volatile uint32_t peer_storm_cr1 = 0;
volatile bool peer_stormed = false;

void peer_service() {
    peer_isr_entries = peer_isr_entries + 1u;
    if (!peer_live) {
        C::clear(I2cClear::all);
        return;
    }
    if (peer_isr_entries > peer_isr_budget) {
        peer_storm_flags = C::flags();
        peer_storm_cr1 = C::regs().CR1;
        peer_stormed = true;
        peer_live = false;
        C::interrupt(I2cInterrupt::all, false);
        C::clear(I2cClear::all);
        return;
    }
    const uint32_t f = Peer::isr();
    if (trace_isrs && peer_trace_n < 12u) {
        peer_trace[peer_trace_n] = f;
        peer_trace_n = static_cast<uint8_t>(peer_trace_n + 1u);
    }
    if (f == 0u) {
        return;
    }
    if ((f & I2cFlag::addr) != 0u) {
        peer_last_dir_read = Peer::host_reads();
        peer_last_code = Peer::matched_address();
        peer_addr_hits = static_cast<uint16_t>(peer_addr_hits + 1u);
        peer_rx_prev = peer_rx_n;
        peer_rx_n = 0;
        peer_tx_n = 0;
        peer_decided = 0;
        if (peer_last_dir_read) {
            // A stale byte in TXDR would be this tenure's first: 32.4.8's
            // own flush, so TXIS asks and the answer is the current
            // tenure's.
            Peer::flush();
        } else if (peer_nack_at != 0u) {
            // Byte control: one byte at a time, so each can be refused.
            (void)C::reload(1, true, false);
        }
        if (peer_stretch_us != 0u) {
            spin_us_isr(peer_stretch_us);
        }
        Peer::answer_address();
        return;
    }
    if ((f & I2cFlag::transfer_reload) != 0u) {
        // Byte control's own event: the byte is in, SCL is held between
        // the eighth and the ninth pulse, and CR2.NACK decides.
        peer_decided = static_cast<uint16_t>(peer_decided + 1u);
        (void)Peer::answer_byte(peer_decided != peer_nack_at);
        return;
    }
    if ((f & I2cFlag::rxne) != 0u) {
        const uint8_t v = Peer::take();
        if (peer_rx_n < peer_cap) {
            peer_rx[peer_rx_n] = v;
        }
        peer_rx_n = static_cast<uint16_t>(peer_rx_n + 1u);
        if (peer_stretch_us != 0u) {
            spin_us_isr(peer_stretch_us);
        }
        return;
    }
    if ((f & I2cFlag::txis) != 0u) {
        if (peer_stretch_us != 0u) {
            spin_us_isr(peer_stretch_us);
        }
        Peer::give(peer_next_answer());
        return;
    }
    if ((f & I2cFlag::nack) != 0u) {
        Peer::clear_nack();
        peer_nacks = static_cast<uint16_t>(peer_nacks + 1u);
        return;
    }
    if ((f & I2cFlag::stop) != 0u) {
        Peer::clear_stop();
        peer_stops = static_cast<uint16_t>(peer_stops + 1u);
        return;
    }
    // Every error flag is a LEVEL: record it and clear it, or the
    // handler re-enters for ever.
    peer_errors_seen = peer_errors_seen | (f & I2cFlag::errors);
    if ((f & I2cFlag::overrun) != 0u) {
        peer_overruns = static_cast<uint16_t>(peer_overruns + 1u);
    }
    C::clear(I2cClear::errors);
    // WHAT COULD NOT BE CLEARED MUST BE DISARMED, and what is cleared by
    // an ACCESS must be accessed: TC and TCR have no ICR bit at all and
    // RXNE is cleared only by reading RXDR, so a pump that merely writes
    // the ICR re-enters on them for ever. The driver's own idle sweep
    // follows the same rule and for the same reason.
    if ((C::flags() & I2C_ISR_RXNE) != 0u) {
        (void)C::data();
    }
    if ((C::flags() & (I2C_ISR_TC | I2C_ISR_TCR)) != 0u) {
        Peer::interrupt(I2cInterrupt::transfer_complete, false);
    }
}

/// Bring the client up with `n` answers loaded.
bool peer_arm(const uint8_t* answers, uint16_t n, I2cSpeed speed = I2cSpeed::fast_400k,
              bool no_stretch = false, uint16_t own = peer_addr,
              I2cAddressMode mode = I2cAddressMode::seven_bit,
              uint8_t second = 0, I2cOa2Mask mask = I2cOa2Mask::none,
              bool second_on = false, bool general = false,
              const I2cFilters& f = {}, I2cClock kernel = I2cClock::hsi16) {
    peer_live = false;
    Nvic::disable(C::irq());
    peer_rx_n = 0;
    peer_tx_n = 0;
    peer_addr_hits = 0;
    peer_stops = 0;
    peer_nacks = 0;
    peer_errors_seen = 0;
    peer_overruns = 0;
    peer_stretch_us = 0;
    peer_nack_at = 0;
    peer_isr_entries = 0;
    peer_stormed = false;
    peer_storm_flags = 0;
    for (uint16_t i = 0; i < peer_cap; ++i) {
        peer_answers[i] = i < n ? answers[i] : peer_filler;
    }
    I2cAddressConfig a{};
    a.own = own;
    a.mode = mode;
    a.enable = true;
    a.second = second;
    a.second_mask = mask;
    a.second_enable = second_on;
    const bool ok = Peer::init(clock, a, speed, kernel, f, no_stretch);
    if (general) {
        Peer::general_call(true);
    }
    // TCIE stays OFF: on a target TCR belongs to byte control alone, and
    // an armed transfer-complete interrupt with no owner is a level the
    // ICR cannot clear (the driver's own finding). Letter c arms it with
    // byte control and takes it away again.
    Peer::interrupt(I2cInterrupt::addr | I2cInterrupt::rx | I2cInterrupt::tx |
                        I2cInterrupt::nack | I2cInterrupt::stop |
                        I2cInterrupt::error,
                    true);
    Peer::interrupt(I2cInterrupt::transfer_complete, false);
    peer_live = true;
    Nvic::enable(C::irq());
    return ok;
}

void peer_stop() {
    peer_live = false;
    Nvic::disable(C::irq());
    Peer::release();
}

// ---------------------------------------------------------------------------
// The host's own glue
// ---------------------------------------------------------------------------

volatile bool host_done = false;
volatile uint16_t host_completions = 0;
/// Letter l stages a LOST INTERRUPT: the body still runs, but the
/// completion is never posted - the wedge util/bus_master.hpp's timeout
/// is for.
volatile bool host_swallow_completion = false;
volatile bool dma_host_live = false;
volatile bool bus_ao_live = false;
volatile bool raw_host_live = false;

uint8_t tx_buf[320];
uint8_t rx_buf[320];

/// A tenure that never answered at all. NOT a wire code and not one of
/// util/i2c_bus.hpp's: the suite's own word for "the engine is still
/// holding the bus", so that a stalled leg cannot read as the stale
/// i2c_ok start() left behind.
constexpr uint8_t no_answer = 200;
volatile uint32_t dma_isr_entries = 0;
volatile uint32_t host_stall_isr = 0;
volatile uint32_t stall_client_isr = 0;
volatile uint32_t stall_host_entries = 0;
volatile uint32_t stall_peer_entries = 0;
/// The host's own storm budget, the twin of the client pump's.
constexpr uint32_t host_isr_budget = 60'000;
volatile uint32_t host_storm_flags = 0;
volatile uint32_t host_storm_cr1 = 0;
volatile bool host_stormed = false;

/// One tenure through the TASK, waited out. Returns the status.
uint8_t host_tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx,
                    uint8_t rx_len, I2cSpeed speed) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    host_done = false;
    host_isr_entries = 0;
    host_stormed = false;
    if (Host::start(r)) {
        return Host::status();
    }
    // Bounded at 100 ms: the longest tenure sent here is letter i's read of
    // two the client stretches 6 ms at each of its events, about 24 ms.
    if (!brio::wait_until<Ruler>([] { return host_done; }, 100'000u)) {
        // A STALL IS A REPORT, and the two ISR counters are what tells a
        // dead wire from a storming vector.
        host_stall_isr = H::flags();
        stall_client_isr = C::flags();
        stall_host_entries = host_isr_entries;
        stall_peer_entries = peer_isr_entries;
        // The HOST is recovered and the CLIENT is put back on its feet:
        // a stalled leg must not cost the rest of the letter its
        // verdicts (a client left disarmed makes every later leg of
        // letter f read as a stall it never had).
        (void)Host::recover();
        C::clear(I2cClear::all);
        if ((C::flags() & I2C_ISR_RXNE) != 0u) {
            (void)C::data();
        }
        peer_isr_entries = 0;
        peer_stormed = false;
        return no_answer;
    }
    return Host::status();
}

/// EVERY WIRE LETTER'S OWN SAFETY NET, and the reason this suite could be
/// debugged at all: a marker whose bytes are OUT of the console before
/// the next tenure starts, so the last line on the wire names the step
/// that wedged the board. An I2C interrupt storm starves main, and a
/// suite that prints only at the end of a letter reports nothing at all
/// when one happens (measured: a letter that wedges mid-title says
/// nothing at all about where).
void mark(const char* what) {
    print(serial, "  . ", what, " isr h", host_isr_entries, " c", peer_isr_entries,
          crlf);
    console_drain();
}

/// The same, timed: the cycle count the whole tenure took.
uint32_t host_tenure_cycles(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx,
                            uint8_t rx_len, I2cSpeed speed, uint8_t& status) {
    console_drain();
    const uint32_t t0 = Ruler::now();
    status = host_tenure(addr, tx, tx_len, rx, rx_len, speed);
    return Ruler::now() - t0;
}

/// SMBus's own CRC-8, C(x) = x8 + x2 + x + 1 (32.4.11), bitwise - the
/// reference the hardware PEC is judged against, exactly as the SPI
/// suite judges the hardware CRC.
uint8_t pec_reference(const uint8_t* p, uint16_t n) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < n; ++i) {
        crc = static_cast<uint8_t>(crc ^ p[i]);
        for (uint8_t b = 0; b < 8u; ++b) {
            crc = static_cast<uint8_t>((crc & 0x80u) ? ((crc << 1) ^ 0x07u)
                                                     : (crc << 1));
        }
    }
    return crc;
}

void fill_pattern(uint8_t* p, uint16_t n, uint8_t seed) {
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + i * 7u + (i >> 3));
    }
}

bool same(const volatile uint8_t* a, const uint8_t* b, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

bool same(const uint8_t* a, const uint8_t* b, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

/// The link is alive: a one-byte write the client acknowledges. Every
/// letter that touches the wire asks first and declines with the reason
/// if the answer is no.
/// Put the host back where a letter can use it, whatever the previous
/// one left behind - a letter that fails must not cost the next one its
/// verdicts.
volatile uint16_t bus_found_busy = 0;

void host_ready() {
    (void)Host::init(clock, I2cClock::pclk);
    // A FRESH PERIPHERAL IS NOT A FRESH BUS. 32.4.6's reset clears this
    // instance's own state machine, but ISR.BUSY reflects what its
    // monitor SEES on the wire - and a line left low by whatever ran
    // before is still low. A START issued into that parks (letter c
    // measures the park), so the wire is asked about and cleared here,
    // which is exactly the recovery ladder an application owns.
    if (H::busy()) {
        bus_found_busy = static_cast<uint16_t>(bus_found_busy + 1u);
        (void)Host::unstick();
        (void)Host::recover();
    }
}

bool link_alive() {
    uint8_t one = 0x5A;
    const uint8_t st = host_tenure(peer_addr, &one, 1, nullptr, 0, I2cSpeed::fast_400k);
    if (st != i2c_ok) {
        print(serial, "  link check: status ", st, ", host ISR ", hex(host_stall_isr),
              ", client ISR ", hex(C::flags()), ", client CR1 ", hex(C::regs().CR1),
              ", matches ", peer_addr_hits, crlf);
    }
    return st == i2c_ok;
}

/// A fixed span of `ms` on the kernel tick: the shared wait with a condition
/// that never holds. The tick and not the ruler, so the span holds at every
/// rung of the DynamicClock (the ticker is rebased with it).
static_assert(P::ticks_per_second == 1000u, "settle_ms() counts a tick as a millisecond");
void settle_ms(uint32_t ms) { (void)brio::wait_ticks<P>([] { return false; }, ms); }

// ===========================================================================
// Which desk is this? The self-link, asked of the wire
// ===========================================================================
//
// THE SELF-LINK IS NOT A PROPERTY OF THIS BOARD, it is two jumpers
// from I2C1's pads to I2C2's, and a desk may carry them or not. So the
// suite ASKS THE WIRE, once, before any letter runs, and the letters
// whose second node is I2C2 SKIP THEMSELVES - by name, with the reason
// printed - when the answer is no. A skipped letter claims no verdict either
// way: TestBench::skip() counts it apart from the passes and the fails.

bool self_link = false;

/// Does driving `Drv` really move `Rd`? Both levels, each against the
/// far pad's OPPOSITE internal pull, so a pad stuck at a rail cannot
/// answer yes to both. The 2.2 kOhm pull-ups on the host pair are why
/// the LOW leg is the informative one; the HIGH leg rejects a far pad
/// that is simply floating up.
template <typename Drv, typename Rd>
bool wire_follows() {
    Rd::input(PinPull::up);
    Drv::output(false);
    spin_us<Ruler>(200);
    const bool low = !Rd::read();
    Rd::input(PinPull::down);
    Drv::set();
    spin_us<Ruler>(200);
    const bool high = Rd::read();
    Drv::release();
    Rd::release();
    return low && high;
}

// THE SELF-LINK IS A QUESTION FOR THE WIRE HERE, ON EVERY PART SO FAR.
// It is two jumpers between I2C1's pads and I2C2's, and the peer link is
// two more to another board on the same two nodes, so a desk carries
// either or both and the probe below says whether the first is there.
// Unlike the SPI suite, whose SPI2 reaches no pin of
// the LQFP32, THIS package bonds every pad of both links - I2C1 on
// PB8/PB9 and I2C2 on PA11/PA12, the Nucleo-32's own A5 and A4 (DS12992
// table 12) - so nothing here is compiled out for a package, and the
// constant stays as the one line a part that does not bond them would
// need. An absent peer with the wires in place still fails LOUDLY: that
// is firmware to flash, not a desk this suite was not built for.
constexpr bool self_link_possible = true;

bool probe_self_link() {
    if constexpr (!self_link_possible) {
        print(serial, "  self-link probe: not asked - this package bonds no "
              "pad of I2C2, so there is nothing at the far end to drive or "
              "read", crlf);
        return false;
    }
    const bool scl = wire_follows<SclPin, PeerSclPin>();
    const bool sda = wire_follows<SdaPin, PeerSdaPin>();
    print(serial, "  self-link probe: SCL ", scl, " SDA ", sda, crlf);
    return scl && sda;
}

/// The opening line of every letter whose second node is I2C2.
bool need_self_link() {
    if constexpr (!self_link_possible) {
        bench.skip("this letter's second node is the board's OWN I2C2 client on two jumpers, "
                   "and this package bonds no pad of I2C2: the letter is compiled out here");
        return false;
    }
    if (self_link) {
        return true;
    }
    bench.skip("this letter's second node is the board's OWN I2C2 client (PA11/PA12) and the "
               "probe says the two self-link wires are not on the desk; PB8/PB9 carry the bus "
               "to the PEER BOARD - letters n..t are the instrument this desk has");
    return false;
}

// ===========================================================================
// The peer: twi_link.hpp over the bus under test
// ===========================================================================
//
// The wire format is avrxt/src/apps/twi_link.hpp, included by relative
// path and NOT copied - it names no register, includes nothing of brio
// and every architecture that speaks the link compiles it. The
// instrument at the other end is `twi_peer`, in whichever of its ports
// the wires reach - the samc21 one on SERCOM3 fn C, the stm32g0 one on
// I2C1 AF6 at the same pin names this board hosts on - answering at the ONE
// command address 0x6B, with no general call and no mask, which is the
// whole coexistence argument: nothing the wireless letters do can wake
// it. The peer's own `ident` says which port it is (fw 0x01xx, 0x02xx,
// 0x03xx).
//
// ONE COMMAND IS TWO TENURES of the engine under test - a write carrying
// the frame, then a read collecting the answer - so the command channel
// is itself a measurement of this driver.

using twilink::Op;

constexpr I2cSpeed link_speed = I2cSpeed::standard_100k;
/// The command channel's own rate: link_speed, but for letter s, which
/// carries its commands at the rate it races at (see there).
I2cSpeed command_speed = link_speed;

uint8_t frame_buf[twilink::max_payload + 4];
uint8_t resp_buf[twilink::response_bytes];
twilink::Decoder dec;
bool link_quiet = false;

/// Put the host where a command can be sent: the on-board client off the
/// vector, the arbiter's claim on it dropped, the engine re-inited.
void link_ready() {
    peer_stop();
    bus_ao_live = false;
    dma_host_live = false;
    raw_host_live = false;
    host_ready();
}

bool send_frame(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    twilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof frame_buf) {
                frame_buf[n++] = b;
            }
        },
        op, p, len);
    return host_tenure(twilink::command_addr, frame_buf, n, nullptr, 0, command_speed) ==
           i2c_ok;
}

bool recv_frame(twilink::Frame& out) {
    dec.reset();
    for (uint8_t i = 0; i < twilink::response_bytes; ++i) {
        resp_buf[i] = 0;
    }
    if (host_tenure(twilink::command_addr, nullptr, 0, resp_buf,
                    twilink::response_bytes, command_speed) != i2c_ok) {
        return false;
    }
    for (uint8_t i = 0; i < twilink::response_bytes; ++i) {
        if (dec.feed(resp_buf[i]) == twilink::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

bool command_once(Op op, const uint8_t* p, uint8_t len) {
    if (!send_frame(op, p, len)) {
        return false;
    }
    settle_ms(2);
    twilink::Frame f;
    if (!recv_frame(f)) {
        return false;
    }
    return f.op == Op::ack && f.len == 2 && f.data[0] == twilink::byte_of(op);
}

const uint8_t no_payload[1] = {0};

/// Three attempts with the peer's own recovery bound between them.
bool command(Op op, const uint8_t* p = no_payload, uint8_t len = 0) {
    for (uint8_t k = 0; k < 3; ++k) {
        if (command_once(op, p, len)) {
            settle_ms(twilink::arm_ms);
            return true;
        }
        link_ready();
        settle_ms(400);
    }
    if (!link_quiet) {
        print(serial, "    LINK FAILURE op ", hex(twilink::byte_of(op)),
              ": the peer board must be running `twi_peer` (brio "
              "flash F twi_peer); check the two SCL/SDA wires, the 2.2k pull-ups "
              "and the GND.",
              crlf);
    }
    return false;
}

bool query(Op op, twilink::Frame& data) {
    if (!command(op)) {
        return false;
    }
    settle_ms(2);
    return recv_frame(data);
}

bool peer_report(twilink::Report& r) {
    for (uint8_t k = 0; k < 4; ++k) {
        twilink::Frame f;
        if (query(Op::report, f) && f.op == Op::report_data &&
            f.len == twilink::report_size) {
            r = twilink::get_report(f.data);
            return true;
        }
        settle_ms(60);
    }
    return false;
}

bool peer_act(Op op, const twilink::Params& a) {
    uint8_t p[twilink::params_size];
    twilink::put_params(p, a);
    return command(op, p, twilink::params_size);
}

bool ensure_link() {
    link_quiet = true;
    link_ready();
    for (uint8_t k = 0; k < 3; ++k) {
        if (command(Op::ping)) {
            link_quiet = false;
            return true;
        }
    }
    link_quiet = false;
    print(serial,
          "  THE PEER DID NOT ANSWER. The peer board must be running `twi_peer` "
          "(brio flash F twi_peer); its console '0' forces the "
          "command-mode client back. Check the two wires in this file's header.",
          crlf);
    return false;
}

/// The opening line of every letter whose instrument is the PEER BOARD.
/// The self-link may share the wire with it (the probe's answer changes
/// nothing here): I2C2 is released by link_ready() before every command.
/// An absent peer fails loudly - firmware to flash, or a far board left on
/// its console.
bool need_peer() {
    if (ensure_link()) {
        return true;
    }
    bench.verdict("the peer answers on the command address (the peer board running "
                  "twi_peer)",
                  false);
    return false;
}

// ===========================================================================
// a - the block, wireless
// ===========================================================================

// ---- the instances a part may not have -------------------------------------
// A part whose header declares no I2C3 must not SPELL `I2c<3>` (the
// driver's static_assert is the refusal), and `if constexpr` inside a
// plain function still instantiates the branch it discards - while a
// NON-DEPENDENT `I2c<3>` inside a template body is looked up when the
// template is DEFINED. So the instance NUMBER is made to depend on the
// reserve fact that gates the branch.
template <bool present>
using I2c3 = I2c<present ? uint8_t{3} : uint8_t{2}>;

/// 32.9.6's TIMEOUTR probe, put to a third instance where there is one.
/// Where there is not, the answer is the reserve's own (no such
/// instance, therefore no SMBus) and nothing is written.
template <bool present = i2c_present(3)>
bool third_instance_smbus_probe() {
    if constexpr (present) {
        I2c3<present>::bus_clock(true);
        I2c3<present>::reset();
        return I2c3<present>::smbus_probe();
    } else {
        return false;
    }
}

/// The three verbs that must refuse on an instance table 165 leaves
/// without the independent clock. WHICH instance that is moves with the
/// part - I2C3 where there is one, I2C2 on every part below the G0B1 -
/// so the leg names the instance it asked.
template <bool third = i2c_present(3)>
void no_independent_clock_refusals() {
    if constexpr (third) {
        using I3 = I2c3<third>;
        bench.verdict("I2C3 has no kernel-clock multiplexer: the verb says so",
                      !I3::kernel_clock(I2cClock::hsi16));
        bench.verdict("I2C3 has no SMBus time-outs: the verb says so",
                      !I3::timeouts(1, false, true, 1, true));
        bench.verdict("I2C3 has no wake from Stop: the verb says so",
                      !I3::wake_from_stop(true));
    } else {
        print(serial, "  this part has no I2C3, and table 165 leaves its I2C2 "
              "without the independent clock - so I2C2 is the instance the "
              "three verbs must refuse", crlf);
        bench.verdict("the instance without the independent clock has no "
                      "kernel-clock multiplexer: the verb says so",
                      !C::kernel_clock(I2cClock::hsi16));
        bench.verdict("... no SMBus time-outs: the verb says so",
                      !C::timeouts(1, false, true, 1, true));
        bench.verdict("... and no wake from Stop: the verb says so",
                      !C::wake_from_stop(true));
    }
}

void ta_block() {
    // ---- the reserve against the header ----
    print(serial, "  instances: I2C1 ", i2c_present(1), " I2C2 ", i2c_present(2),
          " I2C3 ", i2c_present(3), "; independent clock: I2C1 ",
          i2c_has_independent_clock(1), " I2C2 ", i2c_has_independent_clock(2),
          crlf);
    // Constants of the reserve: the compiler checks them on every part it
    // builds for, and the run prints that they hold.
    static_assert(i2c_present(1) && i2c_present(2),
                  "reserve: I2C1 and I2C2 on every G0, a third only where the header "
                  "declares one");
    static_assert(i2c_irq(1) == I2C1_IRQn && i2c_irq(1) != i2c_irq(2) &&
                      (!i2c_present(3) || i2c_irq(2) == i2c_irq(3)),
                  "reserve: I2C1's line is its own, and I2C2's is shared with an I2C3 "
                  "exactly where the part has one");
    static_assert(H::dma_rx_request() == 10 && H::dma_tx_request() == 11 &&
                      C::dma_rx_request() == 12 && C::dma_tx_request() == 13 &&
                      i2c_dma_rx_request(3) == 62 && i2c_dma_tx_request(3) == 63,
                  "reserve: DMAMUX 10/11 and 12/13 for the first two, 62/63 for a third "
                  "(table 56)");
    static_assert(i2c_clock_select_pos(1) == 12u &&
                      i2c_clock_select_pos(2) == (i2c_has_independent_clock(2) ? 14u : 0xFFu) &&
                      i2c_clock_select_pos(3) == 0xFF,
                  "reserve: I2C1 always has a kernel-clock selector, I2C2 has one only on "
                  "the parts table 165 gives it to, and a third instance never does");
    static_assert(H::exti_line == 23 &&
                      C::exti_line == (i2c_has_independent_clock(2) ? 22u : 0xFFu) &&
                      i2c_exti_line(3) == 0xFF,
                  "reserve: EXTI 23 wakes I2C1, 22 wakes an I2C2 that has the independent "
                  "clock, and nothing else wakes at all (table 65)");
    static_assert(i2c_pad_fmp_bit('B', 8) != 0 && i2c_pad_fmp_bit('B', 9) != 0 &&
                      i2c_pad_fmp_bit('A', 11) == 0 && i2c_pad_fmp_bit('A', 12) == 0,
                  "reserve: PB8/PB9 have their own FMP bits, PA11/PA12 have none");
    print(serial, "  the reserve's rows hold at compile time (static_assert): the instances, the "
                  "lines, the DMAMUX requests, the kernel-clock selectors, the wake lines, the "
                  "FMP bits",
          crlf);

    // ---- table 165's SMBus column, asked of the SILICON ----
    // 32.9.6: on an instance without SMBus, TIMEOUTR "is reserved, and
    // its bits are forced by hardware to 0". So a write that reads back
    // is the peripheral naming its own column.
    H::bus_clock(true);
    C::bus_clock(true);
    H::reset();
    C::reset();
    const bool s1 = H::smbus_probe();
    const bool s2 = C::smbus_probe();
    const bool s3 = third_instance_smbus_probe();
    print(serial, "  TIMEOUTR answers: I2C1 ", s1 ? "yes" : "no", " I2C2 ",
          s2 ? "yes" : "no", " I2C3 ",
          i2c_present(3) ? (s3 ? "yes" : "no") : "absent", crlf);
    bench.verdict("SMBus, asked of the SILICON: every instance the reserve's "
                  "table 165 column grants it answers, and every instance it "
                  "does not is forced to zero",
                  s1 == i2c_has_smbus(1) && s2 == i2c_has_smbus(2) &&
                      s3 == i2c_has_smbus(3));
    bench.verdict("... and that is exactly what the reserve's table says",
                  s1 == H::has_smbus && s2 == C::has_smbus);

    // ---- the reset values (table 214) ----
    const uint32_t cr1 = H::regs().CR1;
    const uint32_t cr2 = H::regs().CR2;
    const uint32_t isr = H::regs().ISR;
    print(serial, "  reset: CR1 ", hex(cr1), " CR2 ", hex(cr2), " OAR1 ", hex(H::oar1()),
          " TIMINGR ", hex(H::timing_reg()), " ISR ", hex(isr), crlf);
    bench.verdict("reset values: CR1, CR2, OAR1, OAR2, TIMINGR, TIMEOUTR all zero",
                  cr1 == 0 && cr2 == 0 && H::oar1() == 0 && H::oar2() == 0 &&
                      H::timing_reg() == 0 && H::timeouts() == 0);
    // The one register whose reset value is NOT zero, and the reason a
    // transmitter that waits for TXE before its first byte waits for
    // nothing.
    bench.verdict("ISR resets to 0x1: TXE STANDS out of reset", isr == 0x1u);

    // ---- what a PE clear resets and what it KEEPS (32.4.6) ----
    const auto t = *i2c_timing_for(SysClock::hz(), I2cSpeed::fast_400k);
    I2cConfig cfg{};
    cfg.timing = t;
    cfg.filters = I2cFilters{true, 4};
    cfg.general_call = true;
    bench.verdict("configure() lands with PE clear", H::configure(cfg));
    I2cAddressConfig a{};
    a.own = 0x11;
    a.second = 0x30;
    a.second_mask = I2cOa2Mask::low_2;
    a.second_enable = true;
    (void)H::addresses(a);
    H::enable();
    (void)H::transfer(0x44, false, 7, false);
    H::regs().CR2 = H::regs().CR2 | I2C_CR2_NACK;
    const uint32_t cr1_before = H::regs().CR1;
    const uint32_t timing_before = H::timing_reg();
    const uint32_t oar1_before = H::oar1();
    const uint32_t oar2_before = H::oar2();
    const uint32_t cr2_before = H::regs().CR2;
    (void)H::disable();
    const uint32_t cr2_after = H::regs().CR2;
    const uint32_t isr_after = H::regs().ISR;
    print(serial, "  PE clear: CR2 ", hex(cr2_before), " -> ", hex(cr2_after),
          " ISR -> ", hex(isr_after), crlf);
    bench.verdict("PE clear KEEPS CR1 (but PE), TIMINGR, OAR1 and OAR2",
                  (H::regs().CR1 | I2C_CR1_PE) == (cr1_before | I2C_CR1_PE) &&
                      H::timing_reg() == timing_before && H::oar1() == oar1_before &&
                      H::oar2() == oar2_before);
    bench.verdict("PE clear RESETS CR2's NACK (and START, STOP, PECBYTE)",
                  (cr2_before & I2C_CR2_NACK) != 0u &&
                      (cr2_after & (I2C_CR2_NACK | I2C_CR2_START | I2C_CR2_STOP |
                                    I2C_CR2_PECBYTE)) == 0u);
    bench.verdict("PE clear KEEPS CR2's address and NBYTES",
                  (cr2_after & (I2C_CR2_SADD | I2C_CR2_NBYTES)) ==
                      (cr2_before & (I2C_CR2_SADD | I2C_CR2_NBYTES)));
    bench.verdict("PE clear puts ISR back at 0x1", isr_after == 0x1u);

    // ---- the enable protection, MEASURED field by field ----
    // The SPI's question, asked of this chapter's four PE-gated fields:
    // does the SILICON drop a forbidden write, or does only the driver?
    // A PRINT, NOT A VERDICT: 32.9.1 says NOSTRETCH, ANFOFF and DNF "can
    // be programmed only when the I2C peripheral is disabled (PE = 0)" and
    // 32.9.5 that TIMINGR "must be configured when the I2C peripheral is
    // disabled" - duties of the programmer's, with nothing said of what a
    // write with PE set does - so what this die does with one is recorded,
    // and the verdict is the driver's refusal, which brio's contract
    // states.
    H::enable();
    const uint32_t timing_live = H::timing_reg();
    H::regs().TIMINGR = 0x00706090u;
    const bool timing_stuck = H::timing_reg() == 0x00706090u;
    H::regs().TIMINGR = timing_live;
    const uint32_t cr1_live = H::regs().CR1;
    H::regs().CR1 = cr1_live | I2C_CR1_NOSTRETCH;
    const bool nostretch_stuck = (H::regs().CR1 & I2C_CR1_NOSTRETCH) != 0u;
    H::regs().CR1 = cr1_live | I2C_CR1_ANFOFF;
    const bool anfoff_stuck = (H::regs().CR1 & I2C_CR1_ANFOFF) != 0u;
    H::regs().CR1 = (cr1_live & ~I2C_CR1_DNF) | (9u << I2C_CR1_DNF_Pos);
    const bool dnf_stuck = ((H::regs().CR1 & I2C_CR1_DNF) >> I2C_CR1_DNF_Pos) == 9u;
    H::regs().CR1 = cr1_live;
    print(serial, "  with PE set, a raw write lands on: TIMINGR ",
          timing_stuck ? "yes" : "no", " NOSTRETCH ", nostretch_stuck ? "yes" : "no",
          " ANFOFF ", anfoff_stuck ? "yes" : "no", " DNF ", dnf_stuck ? "yes" : "no",
          " - ", !timing_stuck && !nostretch_stuck && !anfoff_stuck && !dnf_stuck
                     ? "the silicon drops all four (undefined by the manual, not judged)"
                     : "the silicon takes one at least (undefined by the manual, not judged)",
          crlf);
    bench.verdict("the driver refuses this chapter's four PE-gated fields with PE set",
                  !H::timing(t) && !H::filters(I2cFilters{}) && !H::no_stretch(true) &&
                      !H::configure(cfg));
    (void)H::disable();

    // ---- the timing arithmetic at this clock ----
    for (uint8_t i = 0; i < 3u; ++i) {
        const auto s = static_cast<I2cSpeed>(i);
        const auto v = i2c_timing_for(SysClock::hz(), s);
        print(serial, "  ", i2c_speed_hz(s) / 1000u, " kHz at ", SysClock::hz() / 1000u,
              " kHz kernel: ");
        if (!v) {
            print(serial, "REFUSED (ES0548 2.10.1's floor is ",
                  i2c_min_kernel_hz(s) / 1000u, " kHz)", crlf);
            continue;
        }
        print(serial, "PRESC ", v->presc, " SCLL ", v->scll, " SCLH ", v->sclh,
              " SDADEL ", v->sdadel, " SCLDEL ", v->scldel, " -> ",
              i2c_scl_hz(SysClock::hz(), *v, I2cFilters{}, i2c_bus_timing(s)),
              " Hz at most, tSCLL ",
              i2c_scll_ns(SysClock::hz(), *v), " ns tSCLH ",
              i2c_sclh_ns(SysClock::hz(), *v), " ns", crlf);
    }
    static_assert(i2c_timing_for(64'000'000UL, I2cSpeed::standard_100k).has_value() &&
                      i2c_scl_hz(64'000'000UL,
                                 *i2c_timing_for(64'000'000UL, I2cSpeed::fast_400k),
                                 I2cFilters{}, i2c_bus_timing(I2cSpeed::fast_400k)) ==
                          400'000 &&
                      i2c_scl_hz(64'000'000UL,
                                 *i2c_timing_for(64'000'000UL, I2cSpeed::fast_plus_1m),
                                 I2cFilters{}, i2c_bus_timing(I2cSpeed::fast_plus_1m)) ==
                          1'000'000,
                  "all three speeds reachable at 64 MHz, none above its band on the fastest "
                  "wire there can be");
    print(serial, "  all three speeds reachable at 64 MHz, none above its band: a constant of "
                  "the arithmetic (static_assert)",
          crlf);

    // ---- the refusals a register can be asked for ----
    // THE INSTANCE WITHOUT THE INDEPENDENT CLOCK IS THE ONE THAT REFUSES,
    // and WHICH instance that is moves with the part: I2C3 where there is
    // one, I2C2 on every part below the G0B1 (table 165's column).
    no_independent_clock_refusals();
    (void)H::kernel_clock(I2cClock::pclk);
    bench.verdict("the wake is refused off HSI16 (32.4.16)", !H::wake_from_stop(true));
    (void)H::kernel_clock(I2cClock::hsi16);
    (void)H::filters(I2cFilters{true, 3});
    bench.verdict("the wake is refused with a digital filter (32.9.1's interlock)",
                  !H::wake_from_stop(true));
    (void)H::filters(I2cFilters{});
    bench.verdict("... and granted once both conditions hold", H::wake_from_stop(true));
    (void)H::wake_from_stop(false);
    (void)H::kernel_clock(I2cClock::pclk);
    H::reset();
}

// ===========================================================================
// b - the link
// ===========================================================================

void tb_link() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    uint8_t answers[16];
    fill_pattern(answers, 16, 0x31);
    (void)peer_arm(answers, 16, I2cSpeed::fast_400k, false, peer_addr,
                   I2cAddressMode::seven_bit, 0, I2cOa2Mask::none, false, true);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }

    mark("armed");
    // ---- a write ----
    // The link check above was a tenure of its own: its address match
    // would otherwise be counted into this one's.
    peer_addr_hits = 0;
    peer_stops = 0;
    fill_pattern(tx_buf, 12, 0x80);
    uint8_t st = host_tenure(peer_addr, tx_buf, 12, nullptr, 0, I2cSpeed::fast_400k);
    bench.verdict("write of 12 bytes: i2c_ok", st == i2c_ok);
    bench.verdict("the client received all 12", peer_rx_n == 12u);
    bench.verdict("... byte-exact", same(peer_rx, tx_buf, 12));
    bench.verdict("one address match, one STOP", peer_addr_hits == 1u && peer_stops == 1u);
    bench.verdict("the client saw a WRITE (DIR clear)", !peer_last_dir_read);

    mark("write done");
    // ---- a read ----
    peer_addr_hits = 0;
    peer_stops = 0;
    for (uint16_t i = 0; i < 16u; ++i) {
        rx_buf[i] = 0;
    }
    st = host_tenure(peer_addr, nullptr, 0, rx_buf, 10, I2cSpeed::fast_400k);
    bench.verdict("read of 10 bytes: i2c_ok", st == i2c_ok);
    bench.verdict("... byte-exact against what the client served",
                  same(rx_buf, answers, 10));
    bench.verdict("the client saw a READ (DIR set)", peer_last_dir_read);

    mark("read done");
    // ---- write-then-read: ONE tenure, TWO address matches ----
    peer_addr_hits = 0;
    peer_stops = 0;
    fill_pattern(tx_buf, 3, 0x11);
    for (uint16_t i = 0; i < 16u; ++i) {
        rx_buf[i] = 0;
    }
    st = host_tenure(peer_addr, tx_buf, 3, rx_buf, 6, I2cSpeed::fast_400k);
    bench.verdict("write-then-read: i2c_ok", st == i2c_ok);
    // The client's count restarts at the repeated START's match, so the write
    // half's is the one that match closed; a read leaves the buffer alone, so
    // the written bytes still stand at its head.
    bench.verdict("... the written bytes arrived",
                  peer_rx_prev == 3u && same(peer_rx, tx_buf, 3));
    bench.verdict("... the read bytes are the client's", same(rx_buf, answers, 6));
    // The repeated START counted on the WIRE, from the far end: two
    // matches and ONE stop is what a repeated start looks like from a
    // target.
    print(serial, "  address matches ", peer_addr_hits, ", stops ", peer_stops, crlf);
    bench.verdict("THE REPEATED START: two address matches, one STOP",
                  peer_addr_hits == 2u && peer_stops == 1u);

    mark("wr-then-rd done");
    // ---- the addresses no Request may name ----
    // 0x7A, a 10-bit prefix, and 0x00, the general call (util/i2c_bus.hpp),
    // each refused by start() itself: true, i2c_rejected, nothing armed.
    // The client was armed ANSWERING THE GENERAL CALL, so either on the
    // wire would be an address match on its count.
    peer_addr_hits = 0;
    peer_last_code = 0xFF;
    uint8_t g = 0x77;
    Host::Request bad{};
    bad.addr = 0x7A;
    bad.tx = lend<Lease::reply>(static_cast<const uint8_t*>(&g));
    bad.tx_len = 1;
    bad.speed = I2cSpeed::fast_400k;
    const bool sync_prefix = Host::start(bad);
    const uint8_t st_prefix = Host::status();
    bad.addr = 0x00;
    const bool sync_call = Host::start(bad);
    const uint8_t st_call = Host::status();
    spin_us<Ruler>(100);   // a 400 kHz address byte and more, had one gone out
    print(serial, "  0x7A: ", sync_prefix ? "refused at start()" : "STARTED", " ", st_prefix, "; 0x00: ",
          sync_call ? "refused at start()" : "STARTED", " ", st_call, "; client matches ", peer_addr_hits, crlf);
    bench.verdict("0x7A and the general call answered i2c_rejected inside start()",
                  sync_prefix && st_prefix == i2c_rejected && sync_call && st_call == i2c_rejected);
    mark("refusals done");
    bench.verdict("... nothing on the wire: the client, answering the general call, matched nothing",
                  peer_addr_hits == 0u && peer_last_code == 0xFFu);

    peer_stop();
}

// ===========================================================================
// c - the vocabulary on the wire
// ===========================================================================

void tc_vocabulary() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    uint8_t answers[8];
    fill_pattern(answers, 8, 0x60);
    (void)peer_arm(answers, 8);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }

    // ---- nobody home ----
    const uint16_t berr0 = Host::spurious_bus_errors();
    uint8_t one = 0x01;
    uint8_t st = host_tenure(nobody_addr, &one, 1, nullptr, 0, I2cSpeed::fast_400k);
    print(serial, "  a write to an empty address: status ", st, " (host ISR ",
          hex(H::flags()), ", client ISR ", hex(C::flags()), ", CR1 ",
          hex(H::regs().CR1), ", matches ", peer_addr_hits, ")", crlf);
    bench.verdict("an address nobody answers: i2c_nack_addr", st == i2c_nack_addr);
    // The empty request is the probe a scanner sends, and its address
    // phase IS the transaction.
    st = host_tenure(nobody_addr, nullptr, 0, nullptr, 0, I2cSpeed::fast_400k);
    bench.verdict("the empty PROBE at an empty address: i2c_nack_addr",
                  st == i2c_nack_addr);
    st = host_tenure(peer_addr, nullptr, 0, nullptr, 0, I2cSpeed::fast_400k);
    bench.verdict("the empty PROBE at the client's address: i2c_ok", st == i2c_ok);

    // ---- a data NACK, commanded ----
    peer_nack_at = 3;
    (void)Peer::byte_control(true);
    Peer::interrupt(I2cInterrupt::transfer_complete, true);
    fill_pattern(tx_buf, 6, 0x90);
    st = host_tenure(peer_addr, tx_buf, 6, nullptr, 0, I2cSpeed::fast_400k);
    print(serial, "  the client refused byte 3: status ", st, ", bytes in ", peer_rx_n,
          crlf);
    bench.verdict("a client that refuses the third byte: i2c_nack_data",
                  st == i2c_nack_data);
    bench.verdict("... and the first two got through", peer_rx_n >= 2u);
    peer_nack_at = 0;
    Peer::interrupt(I2cInterrupt::transfer_complete, false);
    (void)Peer::byte_control(false);

    // ---- a fault staged from the client's own pad ----
    // The CLIENT owns
    // PA12, so it can take the pad from the peripheral and hold SDA low
    // while the host is driving it high. That is arbitration lost, not a
    // bus error, and the difference is the point (32.4.17: ARLO is "a
    // high level sent on SDA but a low level sampled").
    peer_live = false;
    PeerSdaPin::clear();
    PeerSdaPin::output(false, {.open_drain = true});
    st = host_tenure(peer_addr, tx_buf, 4, nullptr, 0, I2cSpeed::fast_400k);
    const uint32_t held_isr = host_stall_isr;
    PeerSdaPin::function(client_pins.sda.function,
                         {.open_drain = true, .speed = PinSpeed::very_high});
    peer_live = true;
    print(serial, "  SDA held low by the client's pad: status ", st, ", host ISR ",
          hex(held_isr), crlf);
    bench.verdict("a tenure into a held SDA never reports i2c_ok", st != i2c_ok);
    // THE FINDING: a controller whose SDA is held low by another device
    // raises NOTHING. 32.4.9 makes the START wait for a free bus ("either
    // immediately if the BUSY flag is low, or tBUF time after the BUSY
    // flag transits from high to low"), and a low SDA under a high SCL is
    // a START the bus monitor has already seen - so BUSY stands, the
    // request is PARKED, and there is no ARLO, no BERR and no completion
    // to report.
    // WHICH IS WHY THE ARBITER OWNS THE TIMEOUT (util/i2c_bus.hpp, and
    // letter l measures it): no engine here can notice a wedged wire.
    bench.verdict("A HELD SDA IS A PARK AND NOT AN ERROR: no ARLO, no BERR - the "
                  "tenure simply never answers",
                  st == no_answer && (held_isr & (I2cFlag::arb_lost |
                                                  I2cFlag::bus_error)) == 0u);
    bench.verdict("... and BUSY is what stands in its place", (held_isr & I2cFlag::busy) != 0u);
    (void)Host::recover();

    // ---- a START still standing ----
    // Parked behind a held SDA by the resource's own verb, it stands for as long
    // as the line is held: 32.9.2 sends it "once the bus is free". 32.9.8 says
    // writing ADDRCF "also clears the START bit", and start() opens with that
    // write - yet the bench's STM32G0B1RE leaves a START parked this way
    // standing, ADDR being clear, where the same sequence on the bench's
    // STM32F446RE's FMPI2C takes it. Which a die does is printed, not judged
    // (one die of each, nothing written says which is right); the verdict is
    // the driver's answer under either: where the write leaves the START, the
    // CR2 store at start()'s end refuses and the host answers i2c_bus_error
    // with nothing moved; where it takes it, start() goes ahead with its own
    // request. The PE cycle takes it either way (32.4.6).
    peer_live = false;
    PeerSdaPin::clear();
    PeerSdaPin::output(false, {.open_drain = true});
    spin_us<Ruler>(50);
    (void)H::transfer_now(peer_addr, false, 1, true);
    spin_us<Ruler>(200);
    const bool start_stood = H::starting() && H::busy();
    H::clear(I2cClear::all);
    const bool start_left = H::starting();
    if (!start_left) {
        (void)H::transfer_now(peer_addr, false, 1, true);
        spin_us<Ruler>(200);
    }
    Host::Request sr{};
    sr.addr = peer_addr;
    sr.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    sr.tx_len = 4;
    sr.speed = I2cSpeed::fast_400k;
    const bool sr_sync = Host::start(sr);
    const uint8_t sr_status = Host::status();
    const uint32_t sr_cr2 = H::regs().CR2;
    const bool sr_nbytes = ((sr_cr2 & I2C_CR2_NBYTES_Msk) >> I2C_CR2_NBYTES_Pos) ==
                           (start_left ? 1u : 4u);
    (void)Host::recover();
    const bool start_gone = !H::starting();
    PeerSdaPin::function(client_pins.sda.function,
                         {.open_drain = true, .speed = PinSpeed::very_high});
    peer_live = true;
    spin_us<Ruler>(200);
    print(serial, "  a START parked behind a held SDA: stood ", start_stood, ", left standing by the "
          "ICR write ", start_left, "; start() over one answered ", sr_sync, ", status ", sr_status,
          ", CR2 ", hex(sr_cr2), "; the PE cycle took it ", start_gone, crlf);
    bench.verdict("A START STILL STANDING: parked behind a held SDA it stood, and start() over one "
                  "answers i2c_bus_error with CR2 unmoved if the ICR write left it, false with its "
                  "own request if the write took it (which this die does: the print above)",
                  start_stood && sr_nbytes &&
                      (start_left ? sr_sync && sr_status == i2c_bus_error : !sr_sync));
    bench.verdict("... and the PE cycle takes it", start_gone);
    (void)Host::recover();

    // ---- ES0548 2.10.2 ----
    print(serial, "  the bus monitor was found BUSY at a letter's start ",
          bus_found_busy, " times so far this run", crlf);
    const uint16_t berr1 = Host::spurious_bus_errors();
    print(serial, "  ES0548 2.10.2: BERR swept ", static_cast<uint16_t>(berr1 - berr0),
          " times in this letter (never reported as a status)", crlf);
    // The erratum's own instruction is "clear it and go on", so the only
    // thing to judge is that the tenures kept working - which every
    // verdict above already says.
    (void)peer_arm(answers, 8);
    bench.verdict("the bus still works after the staged fault", link_alive());
    peer_stop();
}

// ===========================================================================
// d - the three speeds, measured
// ===========================================================================

/// The average SCL period of a tenure of `n` written bytes, in
/// nanoseconds. A tenure is 9 x (n + 1) clock periods plus a START and a
/// STOP, which at n = 32 is under 1 % of the total.
uint32_t measure_scl_ns(uint8_t n, I2cSpeed s, uint8_t& status) {
    fill_pattern(tx_buf, n, 0x40);
    const uint32_t c = host_tenure_cycles(peer_addr, tx_buf, n, nullptr, 0, s, status);
    const uint32_t clocks = 9u * (static_cast<uint32_t>(n) + 1u);
    // The tenure's nanoseconds first, then the division by its clocks: a
    // period of 64 cycles truncated to whole cycles would be 1.5 % coarse.
    return cycles_ns(c) / clocks;
}

/// THE RATE ASKED, against the wire: an SCL period of `ns` in thousandths
/// of the rate `asked_hz` - the speed the request named, never the one
/// TIMINGR produced. A self-link shares the controller's clock with a
/// target that only follows it, so nothing but a stopwatch can see a
/// wrong TIMINGR; the tenure's START, STOP and the request's own cost sit
/// inside the reading, a few thousandths of 297 periods.
uint32_t scl_permille(uint32_t ns, uint32_t asked_hz) {
    return ns == 0u ? 0u
                    : static_cast<uint32_t>(1'000'000'000'000ull /
                                            (static_cast<uint64_t>(ns) * asked_hz));
}

/// The verdict on it: NEVER ABOVE THE RATE ASKED, derived. The chooser
/// (i2c_timing_for()) rounds both halves of the period UP into prescaler
/// units and charges the least detection delay any wire can have (two
/// kernel periods and tAF(min) an edge, 32.4.9's tSYNC floor), so the
/// period it produces is at least the one asked; a slow edge, a third
/// synchronization period, a stretch, the START, the STOP and the
/// request's own cost only add. The stopwatch shares the oscillator: the
/// host's kernel is PCLK and the ruler SysTick on HCLK, one PLL over the
/// HSI16, so DS13560 table 44's tolerance cancels in the ratio. What is
/// left is arithmetic: the tenure's nanoseconds and the division by its
/// clocks each truncate, under 1 ns of a period of 1000 ns or more - one
/// thousandth at Fm+, nothing at the slower two. So 1001 thousandths.
/// How far BELOW the rate a wire lands is the pull-ups' rise and a
/// stretching target's, which no document of this board states; the floor
/// is the framework's window for an I2C clock, a quarter below the rate
/// asked (docs/design/overview.md, "A loop proves the bytes, never the
/// rate"): a reading under it is a finding to explain, not a window to
/// widen. The reading is printed beside each verdict.
constexpr uint32_t scl_ceiling_permille = 1001;
constexpr uint32_t scl_floor_permille = 750;
bool scl_ok(uint32_t permille) {
    return permille >= scl_floor_permille && permille <= scl_ceiling_permille;
}

/// The same arithmetic in nanoseconds: a measured period reads at most this
/// much under the true one (the two truncating divisions of
/// measure_scl_ns()), and a period the register cannot shorten is judged
/// with it and nothing more.
constexpr uint32_t stopwatch_slack_ns = 1;

void td_speeds() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    // The letter's stopwatches time whole tenures against the rate asked: a
    // console that will not drain is skipped on, not timed through (the
    // windows below drain it again, its answer dropped once it has drained).
    if (!console_quiet()) {
        return;
    }
    uint8_t answers[8];
    fill_pattern(answers, 8, 0x70);

    // A TARGET NEEDS TIMINGR TOO, and here that costs something the
    // chapter never mentions: SDADEL and SCLDEL are the delays a TARGET
    // applies, so a client is configured against the FASTEST bus it
    // expects to sit on - and ES0548 2.10.1's floor applies to a target's
    // kernel clock exactly as it does to a controller's. HSI16 IS 16 MHz
    // AND Fm+ ASKS 20, so a client on the independent clock cannot be
    // configured for a 1 MHz bus at all. And the wake from Stop accepts
    // HSI16 AND NOTHING ELSE (32.4.16). The two are therefore MUTUALLY
    // EXCLUSIVE on this part, which no table says.
    bench.verdict("a client on HSI16 CANNOT be set up for an Fm+ bus (2.10.1's floor "
                  "is 20 MHz and HSI16 is 16)",
                  !peer_arm(answers, 8, I2cSpeed::fast_plus_1m, true, peer_addr,
                            I2cAddressMode::seven_bit, 0, I2cOa2Mask::none, false,
                            false, I2cFilters{}, I2cClock::hsi16));
    bench.verdict("... so the WAKE FROM STOP and Fm+ cannot be had at once: the wake "
                  "accepts HSI16 alone (32.4.16)",
                  !i2c_timing_for(16'000'000UL, I2cSpeed::fast_plus_1m).has_value());
    // On PCLK at 64 MHz all three are expressible, so that is where the
    // rates are measured. NOSTRETCH on the client besides: it holds the
    // clock for nothing, so what the stopwatch measures is the
    // CONTROLLER'S OWN rate and not the pair's turnaround.
    (void)peer_arm(answers, 8, I2cSpeed::fast_plus_1m, true, peer_addr,
                   I2cAddressMode::seven_bit, 0, I2cOa2Mask::none, false, false,
                   I2cFilters{}, I2cClock::pclk);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }

    // Fm+ needs the 20 mA pad drive before it is measured at all.
    Host::fast_plus_drive(true);
    bench.verdict("the Fm+ drive is on (SYSCFG's instance bit)", Host::fast_plus_drive());

    // THE FLOOR IS A VERDICT, THE BUDGET A PRINT:
    //   * the FLOOR is the chooser's own prediction, tSCLL + tSCLH plus
    //     the least detection delay any wire can have (the filters and
    //     two kernel periods an edge): nothing makes the clock shorter -
    //     a stretch, a slow edge and a third synchronization period all
    //     ADD - so THE SPEED ASKED IS A CEILING. The stopwatch and the
    //     kernel share the oscillator (scl_ok()), so the period reads
    //     under the floor only by the two truncating divisions, under
    //     1 ns: stopwatch_slack_ns;
    //   * the BUDGET is tSCLL + tSCLH plus the tSYNC the manual's example
    //     tables 172..174 charge for the mode (1000 / 750 / 500 ns) as
    //     their example. An example is no maximum, and this wire's rise
    //     is no document's, so where the period lands against it is
    //     printed (on the bench's 2.2 kOhm wire well inside, nearer the
    //     floor).
    for (uint8_t i = 0; i < 3u; ++i) {
        const auto s = static_cast<I2cSpeed>(i);
        if (!Host::speed_ok(s)) {
            print(serial, "  ", i2c_speed_hz(s) / 1000u, " kHz: REFUSED at this clock",
                  crlf);
            continue;
        }
        const I2cTiming t = Host::timing_of(s);
        const uint32_t halves_ns = i2c_scll_ns(Host::kernel_hz(), t) +
                                   i2c_sclh_ns(Host::kernel_hz(), t);
        const uint32_t floor_ns = 1'000'000'000UL / Host::scl_hz(s);
        constexpr uint32_t manual_sync_ns[3] = {1000, 750, 500};
        const uint32_t worst_ns = halves_ns + manual_sync_ns[i];
        peer_overruns = 0;
        uint8_t st = 0;
        const uint32_t ns = measure_scl_ns(32, s, st);
        print(serial, "  ", i2c_speed_hz(s) / 1000u, " kHz: measured ", ns,
              " ns/clock (", ns != 0u ? 1'000'000'000UL / ns : 0u, " Hz), tSCLL + tSCLH ",
              halves_ns, " ns, so this wire's tSYNC is ", ns > halves_ns ? ns - halves_ns : 0u,
              " ns; the floor ", floor_ns, " ns (", Host::scl_hz(s), " Hz), the manual's "
              "tables' budget ", worst_ns, " ns; status ", st, ", client overruns ", peer_overruns,
              crlf);
        if (st == i2c_ok) {
            const uint32_t pm = scl_permille(ns, i2c_speed_hz(s));
            print(serial, "    SCL in ", pm, " thousandths of the ", i2c_speed_hz(s) / 1000u,
                  " kHz asked", crlf);
            bench.verdict("THE RATE ASKED, ON THE WIRE: SCL never above it and within a "
                          "quarter below it", scl_ok(pm));
            bench.verdict("the measured period is at least the chooser's floor - the rate "
                          "is never above the speed asked",
                          ns + stopwatch_slack_ns >= floor_ns &&
                              ns + stopwatch_slack_ns >= 1'000'000'000UL / i2c_speed_hz(s));
            print(serial, "    against the manual's tables' budget: ",
                  ns <= worst_ns ? "inside it by " : "OVER it by ",
                  ns <= worst_ns ? worst_ns - ns : ns - worst_ns,
                  " ns (an example, not a maximum: not judged)", crlf);
            bench.verdict("... with the tenure byte-exact",
                          peer_rx_n == 32u && same(peer_rx, tx_buf, 32));
        } else {
            // AND THIS IS THE FINDING AT 1 MHz: a NOSTRETCH target must
            // read RXDR before the ninth pulse of the NEXT byte, which at
            // Fm+ is nine microseconds, and an interrupt-driven target on
            // this core does not always make it. 32.4.17: an overrun in
            // target receiver NOSTRETCH mode NACKs automatically - so the
            // controller sees i2c_nack_data and the rate cannot be
            // measured this way at all.
            print(serial, "    -> the NOSTRETCH target could not keep up; the rate "
                          "verdict moves to the stretching leg below", crlf);
            bench.verdict("a NOSTRETCH target that misses its window NACKs by itself "
                          "(32.4.17), and the controller is told",
                          st == i2c_nack_data && peer_overruns != 0u);
        }
    }

    // ---- and the same three WITH the target stretching ----
    // Stretching is flow control, so the data is exact at every rate and
    // the measured period can only be at or above the register's floor.
    // This is the leg that says a 1 MHz bus really carries bytes.
    (void)peer_arm(answers, 8, I2cSpeed::fast_plus_1m, false, peer_addr,
                   I2cAddressMode::seven_bit, 0, I2cOa2Mask::none, false, false,
                   I2cFilters{}, I2cClock::pclk);
    for (uint8_t i = 0; i < 3u; ++i) {
        const auto s = static_cast<I2cSpeed>(i);
        if (!Host::speed_ok(s)) {
            continue;
        }
        const I2cTiming t = Host::timing_of(s);
        const uint32_t floor_ns = i2c_scll_ns(Host::kernel_hz(), t) +
                                  i2c_sclh_ns(Host::kernel_hz(), t);
        uint8_t st = 0;
        const uint32_t ns = measure_scl_ns(32, s, st);
        print(serial, "  stretching at ", i2c_speed_hz(s) / 1000u, " kHz: ", ns,
              " ns/clock against a floor of ", floor_ns, " ns, status ", st, crlf);
        bench.verdict("with the target stretching the tenure is byte-exact",
                      st == i2c_ok && peer_rx_n == 32u && same(peer_rx, tx_buf, 32));
        bench.verdict("... and a stretch can only LENGTHEN the clock",
                      ns + stopwatch_slack_ns >= floor_ns);
    }

    // ---- AND THE BUDGET IS A KNOB, WHICH IS THE POINT OF STATING IT ----
    // The legs above ran on no measured tSYNC: the chooser charged the
    // floor, so the wire's own slopes made the clock SLOWER than nominal -
    // a ceiling kept, at the price of a bus below the rate asked. The
    // cure is to state what the bench measures: the difference between
    // the measured period and the register's own tSCLL + tSCLH IS this
    // wire's tSYNC, and handing it back closes the loop.
    {
        const I2cTiming t0 = Host::timing_of(I2cSpeed::standard_100k);
        const uint32_t floor_ns = i2c_scll_ns(Host::kernel_hz(), t0) +
                                  i2c_sclh_ns(Host::kernel_hz(), t0);
        uint8_t st = 0;
        const uint32_t before = measure_scl_ns(32, I2cSpeed::standard_100k, st);
        const uint32_t wire_ns = before > floor_ns ? before - floor_ns : 0u;
        I2cBusTiming measured = i2c_bus_timing(I2cSpeed::standard_100k);
        measured.sync_ns = static_cast<uint16_t>(wire_ns);
        (void)Host::init(clock, I2cClock::pclk, I2cFilters{}, measured);
        const I2cTiming t1 = Host::timing_of(I2cSpeed::standard_100k);
        // What the re-chosen register makes on this wire: its halves plus
        // the delay just measured.
        const uint32_t predicted = i2c_scll_ns(Host::kernel_hz(), t1) +
                                   i2c_sclh_ns(Host::kernel_hz(), t1) + wire_ns;
        uint8_t st2 = 0;
        const uint32_t after = measure_scl_ns(32, I2cSpeed::standard_100k, st2);
        const uint32_t before_hz = before != 0u ? 1'000'000'000UL / before : 0u;
        const uint32_t after_hz = after != 0u ? 1'000'000'000UL / after : 0u;
        // The same wire a moment later differs from its measurement by up
        // to the synchronizer's third kernel period on each edge (32.4.9's
        // "2 to 3", two a period); the prediction differs from the rate
        // asked by up to the half cycle the chooser rounds the stated delay
        // to, and is otherwise never under it (the halves rounded up into
        // prescaler units). With the four truncated divisions (both
        // readings and both halves), 2.5 kernel periods and 4 ns hold both
        // the band around the prediction and the floor at 10000 ns.
        const uint32_t jitter_ns =
            static_cast<uint32_t>(5'000'000'000ULL / (2ULL * Host::kernel_hz())) + 4u;
        print(serial, "  this wire's own tSYNC is ", wire_ns, " ns; at 100 kHz the "
              "unmeasured default gives ", before_hz, " Hz and the measured budget ",
              after_hz, " Hz (", after, " ns against the re-chosen register's ", predicted,
              " ns, plus or minus ", jitter_ns, ")", crlf);
        bench.verdict("the unmeasured default keeps the bus at or under its mode on a real "
                      "wire",
                      before + stopwatch_slack_ns >= 1'000'000'000UL / 100'000UL);
        bench.verdict("... and a MEASURED budget brings it to the rate asked: the re-chosen "
                      "register's period on this wire, to the synchronizer's jitter, never "
                      "above the rate and never slower than before",
                      st2 == i2c_ok && after + jitter_ns >= predicted &&
                          after <= predicted + jitter_ns &&
                          after + jitter_ns >= 1'000'000'000UL / 100'000UL &&
                          after <= before + stopwatch_slack_ns);
        (void)Host::init(clock, I2cClock::pclk);
    }

    // ---- what the Fm+ drive is worth ----
    uint8_t st_on = 0;
    const uint32_t ns_on = measure_scl_ns(32, I2cSpeed::fast_plus_1m, st_on);
    Host::fast_plus_drive(false);
    uint8_t st_off = 0;
    const uint32_t ns_off = measure_scl_ns(32, I2cSpeed::fast_plus_1m, st_off);
    Host::fast_plus_drive(true);
    (void)st_off;
    print(serial, "  Fm+ at 1 MHz: drive on ", ns_on, " ns/clock (status ", st_on,
          "), drive off ", ns_off, " ns/clock (status ", st_off, ")", crlf);
    // 8.1.1 makes the drive a PAD property and the SCL period is the
    // TIMING register's: the drive moves the falling EDGE, which enters
    // the period only through tSYNC1's detection of it - by how much on
    // this wire no document states, so the difference is printed above
    // and not judged.
    bench.verdict("the bus carries bytes at 1 MHz with the Fm+ drive on",
                  st_on == i2c_ok);

    // ---- the ladder ----
    // The client goes back to HSI16 for it: its kernel then does not move
    // with the core, so the same target serves every rung, and it
    // stretches again so a 2 MHz core can still answer in time.
    (void)peer_arm(answers, 8, I2cSpeed::fast_400k);
    uint32_t mid_timingr = 0;   // 100 kHz's TIMINGR on the 16 MHz rung's kernel
    for (uint8_t rung = 0; rung < 3u; ++rung) {
        SysClock::set_index(rung);
        console_drain();
        const uint32_t hz = SysClock::hz();
        print(serial, "  --- core ", hz / 1000u, " kHz, host kernel on PCLK ---", crlf);
        uint8_t reachable = 0;
        for (uint8_t i = 0; i < 3u; ++i) {
            if (Host::speed_ok(static_cast<I2cSpeed>(i))) {
                ++reachable;
            }
        }
        print(serial, "    speeds reachable on PCLK: ", reachable, " of 3", crlf);
        if (rung == r_fast) {
            bench.verdict("at 64 MHz all three speeds are reachable on PCLK",
                          reachable == 3);
        } else if (rung == r_mid) {
            // 16 MHz is above Fm's 10 MHz floor and below Fm+'s 20.
            bench.verdict("at 16 MHz Fm+ is refused and the other two are not",
                          reachable == 2 && !Host::speed_ok(I2cSpeed::fast_plus_1m));
        } else {
            // THE ERRATUM'S SHARPEST CONSEQUENCE: at 2 MHz the Standard
            // mode floor of 4 MHz refuses even a 100 kHz bus.
            bench.verdict("AT 2 MHz NO SPEED IS LEGAL ON PCLK (2.10.1's Sm floor is "
                          "4 MHz)",
                          reachable == 0);
        }
        if (reachable != 0) {
            uint8_t st = 0;
            fill_pattern(tx_buf, 16, static_cast<uint8_t>(0x50 + rung));
            st = host_tenure(peer_addr, tx_buf, 16, nullptr, 0, I2cSpeed::standard_100k);
            bench.verdict("... and the link is byte-exact at 100 kHz on this rung",
                          st == i2c_ok && same(peer_rx, tx_buf, 16));
            uint8_t st_t = 0;
            const uint32_t pm = scl_permille(measure_scl_ns(32, I2cSpeed::standard_100k, st_t),
                                             100'000u);
            print(serial, "    100 kHz on this rung: SCL in ", pm, " thousandths of the rate "
                  "asked", crlf);
            if (rung == r_mid) {
                mid_timingr = i2c_timingr(Host::timing_of(I2cSpeed::standard_100k));
            }
            bench.verdict("... at the rate asked: SCL never above 100 kHz and within a quarter "
                          "below it, TIMINGR re-chosen for this kernel",
                          st_t == i2c_ok && scl_ok(pm));
        } else {
            // AND THE RESCUE: the same core rate with the instance's own
            // kernel on HSI16. This is what the independent clock is for
            // and the letter proves it on the wire.
            (void)Host::init(clock, I2cClock::hsi16);
            uint8_t reachable_hsi = 0;
            for (uint8_t i = 0; i < 3u; ++i) {
                if (Host::speed_ok(static_cast<I2cSpeed>(i))) {
                    ++reachable_hsi;
                }
            }
            print(serial, "    ... on HSI16 instead: ", reachable_hsi, " of 3", crlf);
            bench.verdict("THE INDEPENDENT CLOCK RESCUES IT: on HSI16, Sm and Fm are "
                          "legal at a 2 MHz core",
                          reachable_hsi == 2);
            uint8_t st = 0;
            fill_pattern(tx_buf, 16, 0x5F);
            st = host_tenure(peer_addr, tx_buf, 16, nullptr, 0, I2cSpeed::standard_100k);
            bench.verdict("... and the bus runs byte-exact with the core at 2 MHz",
                          st == i2c_ok && same(peer_rx, tx_buf, 16));
            // THE STOPWATCH CANNOT TIME THIS RUNG'S SCL, and says so. With
            // the core at 2 MHz a tenure is paced by the two vectors this
            // one core serves - the controller holds SCL low while TXDR
            // waits for its TXIS entry, the target stretches until its own
            // entry reads RXDR (32.4.9, 32.4.17) - so what the tenure takes
            // is the software's, printed. No pad is spare to see SCL's
            // edges. What sets the rate is TIMINGR on
            // a 16 MHz kernel, and that is the 16 MHz rung's own register,
            // which the leg above timed at the rate asked.
            uint8_t st_t = 0;
            const uint32_t pm = scl_permille(measure_scl_ns(32, I2cSpeed::standard_100k, st_t),
                                             100'000u);
            const uint32_t hsi_timingr = i2c_timingr(Host::timing_of(I2cSpeed::standard_100k));
            print(serial, "    100 kHz on HSI16 with the core at 2 MHz: the tenure in ", pm,
                  " thousandths of the rate asked - the two vectors' service at a 2 MHz "
                  "core, not SCL; TIMINGR ", hex(hsi_timingr), ", the 16 MHz rung's ",
                  hex(mid_timingr), crlf);
            bench.verdict("... and its rate is the one timed at 16 MHz: the same TIMINGR on "
                          "the same 16 MHz kernel, the stopwatch at a 2 MHz core timing the "
                          "vectors and not the clock",
                          st_t == i2c_ok && mid_timingr != 0u && hsi_timingr == mid_timingr);
            (void)Host::init(clock, I2cClock::pclk);
        }
    }
    // THE ORDER MATTERS HERE: init() REFUSES at a 2 MHz kernel (no speed
    // is legal there) and a refused init leaves the NVIC line disarmed,
    // which is right - so the rate goes back first and the host is
    // brought up after: a host left down here is a host letter e finds
    // silent.
    SysClock::set_index(r_fast);
    console_drain();
    (void)Host::init(clock, I2cClock::pclk);
    peer_stop();
}

// ===========================================================================
// e - clock stretching
// ===========================================================================

void te_stretch() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    // The letter's stopwatches time whole tenures against the rate asked: a
    // console that will not drain is skipped on, not timed through (the
    // windows below drain it again, its answer dropped once it has drained).
    if (!console_quiet()) {
        return;
    }
    uint8_t answers[16];
    fill_pattern(answers, 16, 0x20);
    (void)peer_arm(answers, 16, I2cSpeed::fast_400k);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }

    uint8_t st = 0;
    for (uint16_t i = 0; i < 16u; ++i) {
        rx_buf[i] = 0;
    }
    const uint32_t base = host_tenure_cycles(peer_addr, nullptr, 0, rx_buf, 8,
                                             I2cSpeed::fast_400k, st);
    const uint32_t base_us = cycles_us(base);
    bench.verdict("an unstretched 8-byte read is byte-exact",
                  st == i2c_ok && same(rx_buf, answers, 8));

    console_drain();
    uint32_t last_us = base_us;
    uint16_t last_hold = 0;
    bool monotone = true;
    bool linear = true;
    bool exact = true;
    // A hold no byte in the shifter can hide (a byte and the STOP at
    // 400 kHz, 25 us), and the slope's band (derived beside the verdict).
    constexpr uint16_t stretch_unhidden_us = 50;
    constexpr uint32_t stretch_slope_band_us = 8;
    for (uint16_t hold : {20u, 50u, 100u}) {
        peer_stretch_us = static_cast<uint16_t>(hold);
        mark("stretch leg");
        const uint32_t c = host_tenure_cycles(peer_addr, nullptr, 0, rx_buf, 8,
                                              I2cSpeed::fast_400k, st);
        const uint32_t us = cycles_us(c);
        if (st == no_answer || peer_stormed || host_stormed) {
            print(serial, "  STALL: host ISR ", hex(host_stall_isr), " client ISR ",
                  hex(stall_client_isr), " entries h", stall_host_entries, " c",
                  stall_peer_entries, " | client storm ", peer_stormed ? 1 : 0, " on ",
                  hex(peer_storm_flags), " CR1 ", hex(peer_storm_cr1),
                  " | host storm ", host_stormed ? 1 : 0, " on ", hex(host_storm_flags),
                  " CR1 ", hex(host_storm_cr1), crlf);
            console_drain();
        }
        mark("leg done");
        // The address event, eight bytes and the tenth TXIS (below): ten
        // stretches of `hold` on top of the baseline, the turnaround they
        // overlap not subtracted.
        const uint32_t predicted = base_us + 10u * hold;
        print(serial, "  stretch ", hold, " us/byte: tenure ", us,
              " us, predicted ", predicted, " us, status ", st, crlf);
        if (us <= last_us) {
            monotone = false;
        }
        // THE SLOPE, NOT THE INTERCEPT. Each commanded hold lands on top
        // of the client's own turnaround, which already stretches SCL in
        // the unstretched baseline, so a tenure runs short of base plus
        // its holds; between two legs that turnaround cancels. TEN holds
        // a tenure, not nine: the address event, the eight TXIS that load
        // the bytes, and the TXIS the eighth byte's handoff to the shifter
        // raises (32.4.8: TXIS whenever TXDR is empty), whose byte is never
        // clocked - and the tenure's end is the HOST's completion, served
        // by a vector of the same priority as the client's (the reset one:
        // nothing here sets a priority), so it waits behind that tenth
        // hold (no preemption at one priority). A hold
        // the byte in the shifter can hide is one shorter than a byte and
        // the STOP, 25 us at 400 kHz: so the step is judged from 50 to
        // 100 us, where nothing hides, and printed from 20 to 50. The
        // band: the two readings truncated to whole microseconds (one),
        // delay_us()'s overshoot of up to one poll turn a hold (nine
        // instructions in the release listing, under 25 cycles, ten holds),
        // and the SysTick handlers a tenure of a millisecond meets (two
        // at most, about 70 cycles each with the exception's entry and
        // return): 1 + (250 + 140) / 64, under 8 us.
        if (last_hold != 0u) {
            const uint32_t step = 10u * static_cast<uint32_t>(hold - last_hold);
            const uint32_t grew = us > last_us ? us - last_us : 0u;
            const uint32_t err = grew > step ? grew - step : step - grew;
            print(serial, "    from ", last_hold, " to ", hold, " us/byte the tenure grew ", grew,
                  " us against ten holds' ", step, crlf);
            if (last_hold >= stretch_unhidden_us && err >= stretch_slope_band_us) {
                linear = false;
            }
        }
        last_us = us;
        last_hold = hold;
        if (st != i2c_ok || !same(rx_buf, answers, 8)) {
            exact = false;
        }
    }
    peer_stretch_us = 0;
    bench.verdict("a commanded stretch lengthens the tenure monotonically", monotone);
    bench.verdict("... by ten holds a step from 50 to 100 us a hold, to 8 us (the slope: the "
                  "client's own turnaround cancels; the tenth is the TXIS the last byte's "
                  "handoff raises, which the host's completion waits behind)",
                  linear);
    bench.verdict("... and the data is untouched by it at every hold (stretching is flow "
                  "control)",
                  exact && st == i2c_ok && same(rx_buf, answers, 8));

    // ---- NOSTRETCH, and 32.4.8's own sentence ----
    // "In transmission, the data must be written in TXDR before the
    // first SCL pulse corresponding to its transfer occurs. If not, an
    // underrun occurs, the OVR flag is set" - and 0xFF goes out in its
    // place.
    (void)peer_arm(answers, 16, I2cSpeed::fast_400k, true);
    peer_stretch_us = 300;   // deliberately late: 300 us against a 22 us byte
    for (uint16_t i = 0; i < 16u; ++i) {
        rx_buf[i] = 0;
    }
    st = host_tenure(peer_addr, nullptr, 0, rx_buf, 8, I2cSpeed::fast_400k);
    peer_stretch_us = 0;
    print(serial, "  NOSTRETCH with a late client: status ", st, ", overruns ",
          peer_overruns, ", first bytes ", hex(rx_buf[0]), " ", hex(rx_buf[1]), crlf);
    bench.verdict("a NOSTRETCH client that is late raises OVR", peer_overruns != 0u);
    bench.verdict("... and 0xFF goes out in the missing byte's place (32.4.8)",
                  rx_buf[1] == 0xFFu || rx_buf[2] == 0xFFu || rx_buf[3] == 0xFFu);
    bench.verdict("... while the tenure itself completes (an underrun is not a fault "
                  "the controller sees)",
                  st == i2c_ok);
    peer_stop();
}

// ===========================================================================
// f - 10-bit addressing, the second address, ADDCODE and DIR
// ===========================================================================

void tf_addressing() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    uint8_t answers[8];
    fill_pattern(answers, 8, 0xC0);
    // A 10-bit OA1 and a masked 7-bit OA2 at once.
    (void)peer_arm(answers, 8, I2cSpeed::fast_400k, false, peer_addr10,
                   I2cAddressMode::ten_bit, 0x50, I2cOa2Mask::low_2, true);

    // The Request is 7-bit by construction (the shape all three targets
    // share), so the 10-bit half is driven through the RESOURCE - which
    // is where 10-bit addressing lives on every target of this project.
    raw_host_live = true;
    (void)H::disable();
    const auto t = *i2c_timing_for(SysClock::hz(), I2cSpeed::fast_400k);
    (void)H::timing(t);
    H::enable();
    H::clear(I2cClear::all);

    fill_pattern(tx_buf, 4, 0xE0);
    peer_addr_hits = 0;
    (void)H::transfer(peer_addr10, false, 4, true, false, I2cAddressMode::ten_bit);
    H::start();
    bool ok = true;
    for (uint8_t i = 0; i < 4u; ++i) {
        // A byte at 400 kHz is 22.5 us; 10 ms only tells an event from none,
        // and the TXIS test below judges which one came.
        (void)brio::wait_until<Ruler>([] { return H::flag(I2cFlag::txis | I2cFlag::nack); },
                                      10'000u);
        if (!H::flag(I2cFlag::txis)) {
            ok = false;
            break;
        }
        H::data(tx_buf[i]);
    }
    // The STOP follows the last byte by one byte time; one that never comes
    // leaves the count and the bytes the verdicts below judge.
    (void)brio::wait_until<Ruler>([] { return H::flag(I2cFlag::stop); }, 10'000u);
    const bool nacked = H::flag(I2cFlag::nack);
    H::clear(I2cClear::all);
    print(serial, "  10-bit write: matches ", peer_addr_hits, ", bytes ", peer_rx_n,
          ", ADDCODE ", hex(peer_last_code), crlf);
    bench.verdict("a 10-bit address is acknowledged", ok && !nacked);
    bench.verdict("... and the bytes arrive byte-exact",
                  peer_rx_n == 4u && same(peer_rx, tx_buf, 4));
    // 32.9.7: in 10-bit mode ADDCODE is the HEADER plus the address's two
    // MSBs, not the address - which is why a client with several
    // addresses cannot simply compare it to its own.
    bench.verdict("ADDCODE is the 10-bit HEADER, not the address (32.9.7)",
                  peer_last_code != (peer_addr10 & 0x7Fu) &&
                      (peer_last_code & 0x78u) == 0x78u);

    // ---- the second address, under its mask ----
    // OA2 = 0x50 with low_2 masked compares OA2[7:3], so 0x50..0x53 all
    // match and 0x54 does not.
    raw_host_live = false;
    H::reset();
    (void)Host::init(clock, I2cClock::pclk);
    uint8_t one = 0x99;
    bool matched[5] = {};
    for (uint8_t k = 0; k < 5u; ++k) {
        const uint8_t a = static_cast<uint8_t>(0x50 + k);
        peer_addr_hits = 0;
        const uint8_t st = host_tenure(a, &one, 1, nullptr, 0, I2cSpeed::fast_400k);
        matched[k] = (st == i2c_ok);
        print(serial, "    ", hex(a), " -> ", st, crlf);
    }
    print(serial, "  OA2 0x50 masked low_2: 0x50 ", matched[0] ? "y" : "n", " 0x51 ",
          matched[1] ? "y" : "n", " 0x52 ", matched[2] ? "y" : "n", " 0x53 ",
          matched[3] ? "y" : "n", " 0x54 ", matched[4] ? "y" : "n", crlf);
    bench.verdict("OA2MSK = low_2 answers four consecutive addresses",
                  matched[0] && matched[1] && matched[2] && matched[3]);
    bench.verdict("... and stops at the fifth", !matched[4]);
    bench.verdict("i2c_oa2_compared_bits() names how many bits that is",
                  i2c_oa2_compared_bits(I2cOa2Mask::low_2) == 5);

    // ---- DIR, both ways ----
    peer_addr_hits = 0;
    const uint8_t stw = host_tenure(0x50, &one, 1, nullptr, 0, I2cSpeed::fast_400k);
    const bool dir_w = peer_last_dir_read;
    const uint8_t codew = peer_last_code;
    const uint8_t str = host_tenure(0x50, nullptr, 0, rx_buf, 1, I2cSpeed::fast_400k);
    const bool dir_r = peer_last_dir_read;
    print(serial, "  write to 0x50 status ", stw, " DIR ", dir_w ? 1 : 0, " code ",
          hex(codew), " | read status ", str, " DIR ", dir_r ? 1 : 0, " code ",
          hex(peer_last_code), crlf);
    bench.verdict("DIR reads write on a write and read on a read", !dir_w && dir_r);
    bench.verdict("ADDCODE names the SECOND address when that is what matched",
                  peer_last_code == 0x50u);
    peer_stop();
}

// ===========================================================================
// g - the filters
// ===========================================================================

void tg_filters() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    uint8_t answers[8];
    fill_pattern(answers, 8, 0x88);
    (void)peer_arm(answers, 8);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }

    // A GLITCH IS DECLINED WITH ITS REASON: both ends of both wires are
    // alternate functions and there is no third pad on either net, so
    // nothing on this board can put a 60 ns spike on SCL or SDA without
    // taking a pad away from the peripheral that must see the spike. The
    // filters' EFFECT ON THE TIMING is arithmetic and IS measured.
    bench.skip("a glitch needs a third pad on the net and this bench has none: the "
               "suppression itself is not judged");

    // The digital filter's delay is DNF x tI2CCLK and enters 32.4.5's
    // hold inequality, so a deeper filter changes the SDADEL the chooser
    // picks - and, past a point, refuses the mode outright through
    // 32.4.3's own condition.
    bool shifts = true;
    uint8_t last_sdadel = 0xFF;
    for (uint8_t dnf : {0u, 1u, 4u, 8u}) {
        const I2cFilters f{true, static_cast<uint8_t>(dnf)};
        const auto v = i2c_timing_for(SysClock::hz(), I2cSpeed::fast_400k, f);
        print(serial, "  DNF ", dnf, ": ");
        if (!v) {
            print(serial, "REFUSED by 32.4.3's own condition", crlf);
            continue;
        }
        print(serial, "SDADEL ", v->sdadel, " (", i2c_sdadel_ns(SysClock::hz(), *v),
              " ns), SCL ",
              i2c_scl_hz(SysClock::hz(), *v, f, i2c_bus_timing(I2cSpeed::fast_400k)),
              " Hz at most",
              crlf);
        if (last_sdadel != 0xFF && v->sdadel > last_sdadel) {
            shifts = false;
        }
        last_sdadel = v->sdadel;
    }
    bench.verdict("a deeper digital filter never asks for MORE hold delay "
                  "(it IS hold delay: 32.4.5 subtracts DNF from the bound)",
                  shifts);
    bench.verdict("a filter deep enough to eat the low period is refused by 32.4.3",
                  !i2c_timing_for(20'000'000UL, I2cSpeed::fast_plus_1m,
                                  I2cFilters{true, 15})
                       .has_value());

    // ---- both filters on the wire ----
    // What CAN be judged with no glitch source: that a bus configured
    // each way still carries bytes, and that ANFOFF and DNF really land.
    for (uint8_t k = 0; k < 3u; ++k) {
        const I2cFilters f = k == 0 ? I2cFilters{true, 0}
                                    : (k == 1 ? I2cFilters{false, 0}
                                              : I2cFilters{false, 6});
        (void)peer_arm(answers, 8, I2cSpeed::fast_400k, false, peer_addr,
                       I2cAddressMode::seven_bit, 0, I2cOa2Mask::none, false, false, f);
        (void)Host::init(clock, I2cClock::pclk, f);
        const I2cFilters back = H::filters();
        fill_pattern(tx_buf, 8, static_cast<uint8_t>(0xA0 + k));
        const uint8_t st = host_tenure(peer_addr, tx_buf, 8, nullptr, 0,
                                       I2cSpeed::fast_400k);
        print(serial, "  analog ", f.analog ? "on " : "off", " DNF ", f.digital,
              ": readback analog ", back.analog ? "on " : "off", " DNF ", back.digital,
              ", status ", st, crlf);
        bench.verdict("the filter configuration reads back as written",
                      back.analog == f.analog && back.digital == f.digital);
        bench.verdict("... and the bus is byte-exact under it",
                      st == i2c_ok && same(peer_rx, tx_buf, 8));
    }
    (void)Host::init(clock, I2cClock::pclk);
    peer_stop();
}

// ===========================================================================
// h - RELOAD past 255, AUTOEND against a software STOP, and the engines
// ===========================================================================

void th_long() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    // The letter's stopwatches time whole tenures against the rate asked: a
    // console that will not drain is skipped on, not timed through (the
    // windows below drain it again, its answer dropped once it has drained).
    if (!console_quiet()) {
        return;
    }
    uint8_t answers[8];
    fill_pattern(answers, 8, 0x11);
    (void)peer_arm(answers, 8);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }

    // ---- 300 bytes through TCR ----
    // The Request's lengths are BYTES IN A uint8_t on all three targets,
    // so a tenure longer than 255 is not a Request at all: RELOAD is a
    // RESOURCE feature and this is where it is driven.
    constexpr uint16_t big = 300;
    fill_pattern(tx_buf, big, 0x30);
    raw_host_live = true;
    (void)H::disable();
    const auto t = *i2c_timing_for(SysClock::hz(), I2cSpeed::fast_400k);
    (void)H::timing(t);
    H::enable();
    H::clear(I2cClear::all);
    peer_rx_n = 0;
    peer_addr_hits = 0;
    uint16_t reloads = 0;
    console_drain();
    const uint32_t big_t0 = Ruler::now();
    (void)H::transfer(peer_addr, false, 255, false, true);
    H::start();
    bool ok = true;
    uint16_t sent = 0;
    uint8_t left = 255;
    while (sent < big && ok) {
        // A byte at 400 kHz is 22.5 us; 10 ms only tells an event from none,
        // and the flags read below judge which one came (none: ok = false).
        (void)brio::wait_until<Ruler>(
            [] { return H::flag(I2cFlag::txis | I2cFlag::transfer_reload | I2cFlag::nack); },
            10'000u);
        if (H::flag(I2cFlag::nack)) {
            ok = false;
            break;
        }
        if (H::flag(I2cFlag::transfer_reload)) {
            const uint16_t rest = static_cast<uint16_t>(big - sent);
            const uint8_t chunk = rest > 255u ? 255u : static_cast<uint8_t>(rest);
            (void)H::reload(chunk, rest > 255u, rest <= 255u);
            left = chunk;
            ++reloads;
            continue;
        }
        if (H::flag(I2cFlag::txis)) {
            H::data(tx_buf[sent]);
            ++sent;
            --left;
            continue;
        }
        ok = false;
    }
    (void)left;
    // The STOP follows the last byte by one byte time; one that never comes
    // leaves the count and the bytes the verdicts below judge.
    (void)brio::wait_until<Ruler>([] { return H::flag(I2cFlag::stop); }, 10'000u);
    const uint32_t big_cycles = Ruler::now() - big_t0;
    H::clear(I2cClear::all);
    // 9 x 301 SCL periods: the address and 300 bytes, each with its ACK.
    const uint32_t big_pm = scl_permille(cycles_ns(big_cycles) / (9u * 301u), 400'000u);
    print(serial, "  300-byte write: sent ", sent, ", TCR reloads ", reloads,
          ", client received ", peer_rx_n, "; SCL in ", big_pm,
          " thousandths of the 400 kHz asked", crlf);
    bench.verdict("a 300-byte write goes through RELOAD and TCR", ok && sent == big);
    bench.verdict("... in exactly one reload past the first 255", reloads == 1u);
    bench.verdict("... and every byte arrives byte-exact",
                  peer_rx_n == big && same(peer_rx, tx_buf, big));
    bench.verdict("... in ONE tenure (one address match)", peer_addr_hits == 1u);
    bench.verdict("... at the rate asked: SCL never above 400 kHz and within a quarter "
                  "below it, across the reload",
                  ok && scl_ok(big_pm));

    // ---- AUTOEND against a software STOP ----
    // With AUTOEND clear the tenure ends at TC with SCL held, and the
    // STOP is software's - the same state a repeated START uses.
    fill_pattern(tx_buf, 4, 0x66);
    peer_addr_hits = 0;
    peer_stops = 0;
    (void)H::transfer(peer_addr, false, 4, false);
    H::start();
    for (uint8_t i = 0; i < 4u; ++i) {
        // A byte at 400 kHz is 22.5 us; a TXIS that never comes leaves TC
        // down, which the verdict below judges.
        (void)brio::wait_until<Ruler>([] { return H::flag(I2cFlag::txis); }, 10'000u);
        H::data(tx_buf[i]);
    }
    const bool tc_stands =
        brio::wait_until<Ruler>([] { return H::flag(I2cFlag::transfer_complete); }, 10'000u);
    const bool no_stop_yet = !H::flag(I2cFlag::stop);
    H::stop();
    const bool stopped = brio::wait_until<Ruler>([] { return H::flag(I2cFlag::stop); }, 10'000u);
    H::clear(I2cClear::all);
    bench.verdict("AUTOEND clear: TC stands and no STOP has gone out",
                  tc_stands && no_stop_yet);
    bench.verdict("... and a software STOP ends it", stopped);
    raw_host_live = false;
    H::reset();

    // ---- the DMA engines ----
    Dma<1>::bus_clock(true);
    dma_host_live = true;
    (void)DmaHost::init(clock, I2cClock::pclk);
    fill_pattern(tx_buf, 64, 0x77);
    peer_rx_n = 0;
    DmaHost::Request r{};
    r.addr = peer_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.tx_len = 64;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    r.rx_len = 0;
    r.speed = I2cSpeed::fast_400k;
    r.engine = I2cEngine::dma;   // every tenure of this part: the engines named
    // Each tenure below is 64 bytes at most at 400 kHz, 1.5 ms of wire: 20 ms
    // only tells a completion from a wedge, and every verdict takes the answer.
    const auto completed = [] { return brio::wait_until<Ruler>([] { return host_done; }, 20'000u); };
    host_done = false;
    console_drain();
    const uint32_t dma_t0 = Ruler::now();
    (void)DmaHost::start(r);
    const bool tx_done = completed();
    const uint32_t dma_cycles = Ruler::now() - dma_t0;
    // 9 x 65 SCL periods, the stopwatch around the whole tenure.
    const uint32_t dma_pm = scl_permille(cycles_ns(dma_cycles) / (9u * 65u), 400'000u);
    print(serial, "  DMA write of 64: status ", DmaHost::status(), ", client got ",
          peer_rx_n, "; SCL in ", dma_pm, " thousandths of the 400 kHz asked", crlf);
    bench.verdict("the host's TX engine moves 64 bytes",
                  tx_done && DmaHost::status() == i2c_ok && peer_rx_n == 64u &&
                      same(peer_rx, tx_buf, 64));
    bench.verdict("... at the rate asked: SCL under the engine never above 400 kHz and "
                  "within a quarter below it",
                  scl_ok(dma_pm));

    for (uint16_t i = 0; i < 64u; ++i) {
        rx_buf[i] = 0;
    }
    peer_tx_n = 0;
    uint8_t serve[64];
    fill_pattern(serve, 64, 0x13);
    for (uint16_t i = 0; i < 64u; ++i) {
        peer_answers[i] = serve[i];
    }
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.tx_len = 0;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.rx_len = 64;
    host_done = false;
    (void)DmaHost::start(r);
    const bool rx_done = completed();
    bench.verdict("the host's RX engine drains 64 bytes",
                  rx_done && DmaHost::status() == i2c_ok && same(rx_buf, serve, 64));

    // The register read on both engines: two bytes written, the repeated
    // START, eight read.
    for (uint16_t i = 0; i < 8u; ++i) {
        rx_buf[i] = 0;
    }
    peer_rx[0] = 0;
    peer_rx[1] = 0;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.tx_len = 2;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.rx_len = 8;
    host_done = false;
    (void)DmaHost::start(r);
    const bool wr_done = completed();
    // (The client's count restarts at every address match, the repeated
    // START's included: the two written bytes are judged by content.)
    const bool wr_ok = wr_done && DmaHost::status() == i2c_ok && same(peer_rx, tx_buf, 2) &&
                       same(rx_buf, serve, 8);
    // AND THE SHORTEST HALVES, right after them on the same host and the
    // same `dma`: a one-byte write, its preload alone, a one-byte read on its
    // channel, and a write of three with NO BUFFER, which the engines do not carry and
    // the pump fills with 0xFF - the tenure an engined host once left with
    // its transmit interrupt disarmed by the engine before it, its clock
    // stretched to the arbiter's timeout.
    peer_rx_n = 0;
    r.tx_len = 1;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    r.rx_len = 0;
    host_done = false;
    (void)DmaHost::start(r);
    const bool one_w_done = completed();
    const bool one_w = one_w_done && DmaHost::status() == i2c_ok && peer_rx_n == 1u && peer_rx[0] == tx_buf[0];
    rx_buf[0] = 0;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.tx_len = 0;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.rx_len = 1;
    host_done = false;
    (void)DmaHost::start(r);
    const bool one_r_done = completed();
    const bool one_r = one_r_done && DmaHost::status() == i2c_ok && rx_buf[0] == serve[0];
    peer_rx_n = 0;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.tx_len = 3;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    r.rx_len = 0;
    host_done = false;
    (void)DmaHost::start(r);
    const bool filler_done = completed();
    const bool filler = filler_done && DmaHost::status() == i2c_ok && peer_rx_n == 3u &&
                        peer_rx[0] == 0xFFu && peer_rx[1] == 0xFFu && peer_rx[2] == 0xFFu;
    print(serial, "  DMA write-then-read 2+8 ", wr_ok ? "exact" : "WRONG", "; one byte written ",
          one_w ? "exact" : "WRONG", ", one read ", one_r ? "exact" : "WRONG",
          "; a write of three with no buffer: done ", host_done, " status ", DmaHost::status(),
          ", client got ", peer_rx_n, crlf);
    bench.verdict("the write-then-read on both engines: two written, the repeated START, "
                  "eight read",
                  wr_ok);
    bench.verdict("under dma a one-byte write, its preload alone, and a one-byte read on its "
                  "channel",
                  one_w && one_r);
    bench.verdict("and a write with no buffer after them completes, the pump's filler on the "
                  "wire",
                  filler);
    dma_host_live = false;
    DmaHost::release();
    (void)Host::init(clock, I2cClock::pclk);
    peer_stop();
}

// ===========================================================================
// u - the engine named: Request::engine on the engined host, the refusal on
//     the engineless one
// ===========================================================================
//
// WHICH MECHANISM CARRIED A HALF is judged from two things the tenure leaves
// behind. The HOST VECTOR'S ENTRIES, an exact count: a half of m bytes on the
// pump takes m entries with its STOPF (m - 1 TXIS after the preload, or m - 1
// RXNE and the last byte at the STOPF: the preload and the STOPF read, both
// in stm32g0/i2c.hpp's start() and isr()), and on the engines the STOPF alone, no DMA vector entered (error_only). And THE
// CHANNEL'S MEMORY ADDRESS, because one byte costs the pump one entry too:
// CMAR is zeroed with the channel stopped before every tenure, and
// DmaChannel::restart() is the only thing that writes it (dma.hpp), so CMAR
// naming this tenure's run afterwards means the engine was started for it - a
// read's buffer, a write's second byte, the first being the preload's - and
// zero means it was not. Both bounds are identities, no tolerance: an entry
// more or a channel started is the engine named not followed.

/// What one engined tenure left behind.
struct EnginePath {
    uint8_t status;
    uint32_t host_entries;
    uint32_t dma_entries;
    bool tx_carried;
    bool rx_carried;
};

EnginePath engined_tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx,
                          uint8_t rx_len, I2cEngine engine, I2cSpeed speed) {
    using TxChannel = DmaChannel<1, 1>;
    using RxChannel = DmaChannel<1, 2>;
    HostTx::stop();
    HostRx::stop();
    TxChannel::regs().CMAR = 0;
    RxChannel::regs().CMAR = 0;
    DmaHost::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    r.engine = engine;
    host_done = false;
    host_isr_entries = 0;
    dma_isr_entries = 0;
    peer_rx_n = 0;
    peer_addr_hits = 0;
    uint8_t status = no_answer;
    if (DmaHost::start(r)) {
        status = DmaHost::status();
    } else {
        // The longest half here is 64 bytes at 400 kHz, 1.5 ms of wire: 20 ms
        // only tells a completion from a wedge.
        if (brio::wait_until<Ruler>([] { return host_done; }, 20'000u)) {
            status = DmaHost::status();
        } else {
            (void)DmaHost::recover();
        }
    }
    return {status, host_isr_entries, dma_isr_entries,
            tx != nullptr && TxChannel::regs().CMAR == reinterpret_cast<uint32_t>(tx + 1),
            rx != nullptr && RxChannel::regs().CMAR == reinterpret_cast<uint32_t>(rx)};
}

/// The bytes a tenure wrote, judged at the client.
bool client_got(const uint8_t* tx, uint8_t len) {
    return peer_rx_n == len && same(peer_rx, tx, len);
}

/// The two values at one speed, on the engined host.
void engine_rule(I2cSpeed speed, const char* rate, const uint8_t* serve) {
    print(serial, "  at ", rate, ":", crlf);
    fill_pattern(tx_buf, 64, 0x41);

    // ---- a Request naming nothing: the pump, the default ----
    // Two bytes each way, the shortest half whose pump takes a data entry
    // (the first byte of a half takes none: the preload, the read at the
    // STOPF), so the entries tell the pump from the engines on their own.
    EnginePath p = engined_tenure(peer_addr, tx_buf, 2, nullptr, 0, DmaHost::Request{}.engine,
                                  speed);
    print(serial, "    named nothing, write 2: status ", p.status, ", host entries ",
          p.host_entries, ", tx channel ", p.tx_carried, crlf);
    bench.verdict("a Request naming nothing rides the pump: a write of 2, two entries, no "
                  "channel started",
                  p.status == i2c_ok && p.host_entries == 2u && p.dma_entries == 0u &&
                      !p.tx_carried && client_got(tx_buf, 2));
    p = engined_tenure(peer_addr, nullptr, 0, rx_buf, 2, DmaHost::Request{}.engine, speed);
    print(serial, "    named nothing, read 2: status ", p.status, ", host entries ",
          p.host_entries, ", rx channel ", p.rx_carried, crlf);
    bench.verdict("... and a read of 2 the same way",
                  p.status == i2c_ok && p.host_entries == 2u && p.dma_entries == 0u &&
                      !p.rx_carried && same(rx_buf, serve, 2));

    // ---- interrupt: the pump at any length ----
    p = engined_tenure(peer_addr, tx_buf, 64, nullptr, 0, I2cEngine::interrupt, speed);
    print(serial, "    interrupt write 64: status ", p.status, ", host entries ", p.host_entries,
          ", tx channel ", p.tx_carried, crlf);
    bench.verdict("interrupt: a write of 64 on the pump, one entry a byte, no channel started",
                  p.status == i2c_ok && p.host_entries == 64u && p.dma_entries == 0u &&
                      !p.tx_carried && client_got(tx_buf, 64));
    p = engined_tenure(peer_addr, nullptr, 0, rx_buf, 64, I2cEngine::interrupt, speed);
    print(serial, "    interrupt read 64: status ", p.status, ", host entries ", p.host_entries,
          ", rx channel ", p.rx_carried, crlf);
    bench.verdict("... a read of 64 on the pump, one entry a byte, no channel started",
                  p.status == i2c_ok && p.host_entries == 64u && p.dma_entries == 0u &&
                      !p.rx_carried && same(rx_buf, serve, 64));

    // ---- dma: the engines at any length, one byte included ----
    p = engined_tenure(peer_addr, tx_buf, 2, nullptr, 0, I2cEngine::dma, speed);
    print(serial, "    dma write 2: status ", p.status, ", host entries ", p.host_entries,
          ", tx channel ", p.tx_carried, crlf);
    bench.verdict("dma: a write of TWO, its first byte preloaded, the channel started on the "
                  "second, the STOPF the one entry",
                  p.status == i2c_ok && p.host_entries == 1u && p.dma_entries == 0u &&
                      p.tx_carried && client_got(tx_buf, 2));
    p = engined_tenure(peer_addr, tx_buf, 1, nullptr, 0, I2cEngine::dma, speed);
    print(serial, "    dma write 1: status ", p.status, ", host entries ", p.host_entries,
          ", tx channel ", p.tx_carried, crlf);
    bench.verdict("... a write of ONE is its preload alone: no channel, the STOPF the one entry",
                  p.status == i2c_ok && p.host_entries == 1u && p.dma_entries == 0u &&
                      !p.tx_carried && client_got(tx_buf, 1));
    p = engined_tenure(peer_addr, nullptr, 0, rx_buf, 1, I2cEngine::dma, speed);
    print(serial, "    dma read 1: status ", p.status, ", host entries ", p.host_entries,
          ", rx channel ", p.rx_carried, crlf);
    bench.verdict("... a read of ONE on the engines, the channel started for it",
                  p.status == i2c_ok && p.host_entries == 1u && p.dma_entries == 0u &&
                      p.rx_carried && same(rx_buf, serve, 1));
    // The register read, one written and one read: the preload and the
    // receive channel, TC's repeated START and the STOPF the two entries.
    p = engined_tenure(peer_addr, tx_buf, 1, rx_buf, 1, I2cEngine::dma, speed);
    print(serial, "    dma 1 + 1: status ", p.status, ", host entries ", p.host_entries,
          ", tx channel ", p.tx_carried, ", rx channel ", p.rx_carried, crlf);
    bench.verdict("... a register read of one and one: the preload and the receive channel, TC "
                  "and the STOPF its two entries",
                  p.status == i2c_ok && p.host_entries == 2u && p.dma_entries == 0u &&
                      !p.tx_carried && p.rx_carried && same(rx_buf, serve, 1));

    // ---- what the engines cannot carry stays on the pump, under dma too ----
    p = engined_tenure(peer_addr, nullptr, 3, nullptr, 0, I2cEngine::dma, speed);
    const bool filled = peer_rx_n == 3u && peer_rx[0] == 0xFFu && peer_rx[1] == 0xFFu &&
                        peer_rx[2] == 0xFFu;
    print(serial, "    dma write of 3 with no buffer: status ", p.status, ", host entries ",
          p.host_entries, ", client got ", peer_rx_n, crlf);
    bench.verdict("... a half with no buffer stays on the pump, not refused: the filler, one "
                  "entry a byte",
                  p.status == i2c_ok && p.host_entries == 3u && filled);

    p = engined_tenure(peer_addr, nullptr, 0, nullptr, 3, I2cEngine::dma, speed);
    print(serial, "    dma read of 3 with no buffer: status ", p.status, ", host entries ",
          p.host_entries, crlf);
    bench.verdict("... and a read with no buffer too: the pump drains it, one entry a byte",
                  p.status == i2c_ok && p.host_entries == 3u);

    // ---- a byte an unanswered address left in TXDR, then the engines ----
    // A pumped write to nobody leaves its preloaded byte in TXDR (32.9.11);
    // TXDR takes no write over it, so it would go out first, the run's last
    // byte never (the NBYTES count), but for the flush ahead of the preload.
    p = engined_tenure(nobody_addr, tx_buf, 1, nullptr, 0, I2cEngine::interrupt, speed);
    const uint8_t nacked = p.status;
    fill_pattern(tx_buf, 8, 0x5C);
    p = engined_tenure(peer_addr, tx_buf, 4, nullptr, 0, I2cEngine::dma, speed);
    print(serial, "    a write to nobody on the pump: status ", nacked,
          "; then dma write 4: status ", p.status, ", client got ", peer_rx_n, ", first ",
          hex(peer_rx[0]), " against ", hex(tx_buf[0]), crlf);
    bench.verdict("a stale byte the pump left in TXDR never reaches the engines' tenure",
                  nacked == i2c_nack_addr && p.status == i2c_ok && client_got(tx_buf, 4));

}

void tu_engine_choice() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    static_assert(!Host::has_dma && DmaHost::has_dma,
                  "the letter's two hosts: the refusal's and the engines'");
    uint8_t serve[64];
    fill_pattern(serve, 64, 0x29);
    (void)peer_arm(serve, 64);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }

    // ---- the refusal, on the engineless host ----
    // The request names the client's own address, so anything that reached
    // the wire would raise ADDR at the client within one START and nine
    // periods of the address (28 us at this rung's 359 kHz, the slowest
    // letter d reads) and enter its vector: no entry on either vector across
    // 400 us, over fourteen of those, and BUSY low at the end, is nothing on
    // the wire.
    fill_pattern(tx_buf, 8, 0x37);
    Host::Request q{};
    q.addr = peer_addr;
    q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    q.tx_len = 4;
    q.speed = I2cSpeed::fast_400k;
    q.engine = I2cEngine::dma;
    host_done = false;
    host_isr_entries = 0;
    peer_isr_entries = 0;
    peer_addr_hits = 0;
    const bool answered = Host::start(q);
    const uint8_t refused = Host::status();
    spin_us<Ruler>(400);
    print(serial, "  dma on the engineless host: start() ", answered, ", status ", refused,
          ", client matches ", peer_addr_hits, ", vector entries host ", host_isr_entries,
          " client ", peer_isr_entries, ", BUSY ", H::busy(), crlf);
    bench.verdict("dma on a host with no engines bound: start() answers at once, i2c_rejected",
                  answered && refused == i2c_rejected && Host::idle());
    bench.verdict("... nothing on the wire: no address match, no entry on either vector, no BUSY",
                  peer_addr_hits == 0u && host_isr_entries == 0u && peer_isr_entries == 0u &&
                      !H::busy() && !host_done);
    q.engine = static_cast<I2cEngine>(3);
    bench.verdict("... a value past the enumeration is refused the same way",
                  Host::start(q) && Host::status() == i2c_rejected);
    // The same host goes on working, `interrupt` being its own.
    q.engine = I2cEngine::interrupt;
    peer_rx_n = 0;
    host_done = false;
    const bool wire_moved = !Host::start(q);
    // Four bytes at 400 kHz are 0.1 ms of wire: 20 ms only tells a completion
    // from a wedge.
    const bool q_done = brio::wait_until<Ruler>([] { return host_done; }, 20'000u);
    bench.verdict("... and interrupt on that host runs the tenure, byte-exact",
                  wire_moved && q_done && Host::status() == i2c_ok &&
                      client_got(tx_buf, 4));

    // ---- the two values, on the engined host ----
    dma_host_live = true;
    (void)DmaHost::init(clock, I2cClock::pclk);
    {
        DmaHost::Request bad{};
        bad.addr = peer_addr;
        bad.speed = I2cSpeed::fast_400k;
        bad.engine = static_cast<I2cEngine>(3);
        bench.verdict("the engined host refuses a value past the enumeration too",
                      DmaHost::start(bad) && DmaHost::status() == i2c_rejected);
    }
    engine_rule(I2cSpeed::fast_400k, "400 kHz", serve);

    // THE SAME AT THE TOP RUNG: the engine a Request names is the one
    // that carries it at every speed, so the same tenures must take the
    // same paths at 1 MHz. The client's kernel moves to
    // PCLK (ES0548 2.10.1's 20 MHz floor rules HSI16 out at Fm+), and the
    // 20 mA drive comes on for the rung and goes off after it.
    dma_host_live = false;
    DmaHost::release();
    (void)peer_arm(serve, 64, I2cSpeed::fast_plus_1m, false, peer_addr,
                   I2cAddressMode::seven_bit, 0, I2cOa2Mask::none, false, false, I2cFilters{},
                   I2cClock::pclk);
    C::instance_fast_plus(true);
    dma_host_live = true;
    (void)DmaHost::init(clock, I2cClock::pclk);
    DmaHost::fast_plus_drive(true);
    engine_rule(I2cSpeed::fast_plus_1m, "1 MHz", serve);
    DmaHost::fast_plus_drive(false);
    C::instance_fast_plus(false);
    dma_host_live = false;
    DmaHost::release();
    (void)Host::init(clock, I2cClock::pclk);
    peer_stop();
}

// ===========================================================================
// v - A REFUSED WRITE, position by position, on either engine
// ===========================================================================
//
// The client refuses by its own byte control: peer_nack_at names the received
// byte, counted from one, and it is in the client's count and buffer, its
// ninth pulse being what is refused. An address nobody answers refuses the
// address. THE PASS HAS NO init() AND NO PE CYCLE BETWEEN A FAILED TENURE AND
// THE NEXT REQUEST, at one speed: either would flush TXDR (32.9.7, TXE set when
// PE = 0) and hide a byte the failed tenure left there, and the flush start()
// opens a write with is what is being judged.

/// A write of `tx` then a read of `rx`, refused at byte `at` of the write, or on
/// the address at zero.
struct RefusedRow {
    uint8_t tx;
    uint8_t rx;
    uint8_t at;
};

constexpr RefusedRow refused_rows[] = {
    {1, 0, 0},     {2, 0, 0},  {16, 0, 0}, {1, 4, 0},  {2, 4, 0},   // the address
    {1, 0, 1},     {1, 4, 1},                                       // a write of one
    {2, 0, 1},     {2, 4, 1},  {16, 0, 1},                          // the first byte
    {2, 0, 2},     {2, 4, 2},  {16, 0, 2},                          // the second
    {3, 0, 3},     {16, 0, 3}, {16, 0, 7}, {16, 0, 16},             // a later byte
    {255, 0, 200}, {255, 0, 255},
};

/// One position's rows: how many there are and how many were right.
struct RefusedTally {
    uint16_t seen = 0;
    uint16_t right = 0;
    void add(bool ok) {
        seen = static_cast<uint16_t>(seen + 1u);
        right = static_cast<uint16_t>(right + (ok ? 1u : 0u));
    }
    bool whole() const { return seen != 0u && right == seen; }
};

/// What the rows came to, by position, for the engine of the refused write.
struct RefusedPositions {
    RefusedTally address;
    RefusedTally one;      ///< a write of one
    RefusedTally first;    ///< the first byte, of two or more
    RefusedTally second;
    RefusedTally later;
    RefusedTally two;      ///< a write of two, either byte (also counted above)
    RefusedTally next;     ///< the request after each, byte for byte
    bool whole() const {
        return address.whole() && one.whole() && first.whole() && second.whole() &&
               later.whole() && two.whole() && next.whole();
    }
};

/// THE BOUND ON ONE TENURE, in ticks of 1 ms: nine clocks for each address and
/// each byte plus the START and the STOP at the speed's period, which letter d
/// prints at 807 to 973 thousandths of the rate asked on the bench's wire (a
/// period at most a quarter longer); under byte control
/// the client adds at most its two entries a byte, which at 64 MHz and the
/// flash's two wait states are under 10 us against a byte of 22.5 us at 400 kHz.
/// Twice the wire holds all that, doubled again as this bound only tells a
/// completion from a wedge, and 3 ticks cover the tick's own reading on either
/// side of a tenure far shorter than a tick.
uint32_t refused_limit_ms(uint16_t bytes, I2cSpeed s) {
    const uint32_t clocks = 9u * (static_cast<uint32_t>(bytes) + 2u) + 2u;
    return 4u * clocks * 1000u / i2c_speed_hz(s) + 3u;
}

/// A byte of the refused write and of the one after it, each position different.
constexpr uint8_t refused_value(uint16_t k, uint16_t i) {
    return static_cast<uint8_t>(0x35u + 0x4Bu * k + 0x1Du * i + ((i * i) >> 3));
}
constexpr uint8_t refused_after(uint16_t i) { return static_cast<uint8_t>(0xC7u + 0x35u * i); }

/// One request, waited for: its status, or no_answer after recover().
template <typename Bus>
uint8_t refused_request(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx,
                        uint8_t rx_len, I2cEngine e, I2cSpeed s, bool starve) {
    typename Bus::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = s;
    r.engine = e;
    host_done = false;
    host_isr_entries = 0;
    peer_isr_entries = 0;   // the client's storm budget is a tenure's here, not a letter's
    if (Bus::start(r)) {
        return no_answer;
    }
    if (starve) {
        // Taken away ahead of the address's nine clocks, so no beat is served
        // and the channel's count stands under a first byte already gone.
        H::dma_transmit(false);
    }
    const uint32_t limit = refused_limit_ms(static_cast<uint16_t>(tx_len + rx_len), s);
    const uint32_t t0 = Ticker::millis();
    while (!host_done && Ticker::millis() - t0 < limit) {
    }
    if (!host_done) {
        (void)Bus::recover();
        return no_answer;
    }
    return Bus::status();
}

/// One row and the request after it.
template <typename Bus>
void refused_row(const RefusedRow& row, I2cEngine e, I2cEngine next, I2cSpeed s, bool print_row,
                 RefusedPositions& pos) {
    for (uint16_t i = 0; i < row.tx; ++i) {
        tx_buf[i] = refused_value(static_cast<uint16_t>(row.tx + row.at), i);
    }
    uint8_t in[4] = {0xEE, 0xEE, 0xEE, 0xEE};
    peer_nack_at = row.at;
    peer_addr_hits = 0;
    peer_rx_n = 0;
    const uint8_t st = refused_request<Bus>(row.at == 0u ? nobody_addr : peer_addr, tx_buf, row.tx,
                                            row.rx != 0u ? in : nullptr, row.rx, e, s, false);
    const uint16_t took = peer_rx_n;
    const uint16_t hits = peer_addr_hits;
    bool untouched = true;
    for (const uint8_t v : in) {
        untouched = untouched && v == 0xEEu;
    }
    const uint8_t want = row.at == 0u ? i2c_nack_addr : i2c_nack_data;
    const bool right = st == want && untouched && took == row.at &&
                       hits == (row.at == 0u ? 0u : 1u) && same(peer_rx, tx_buf, took);

    // The next request, with the refusal out of the way and nothing else done:
    // the client stays in byte control, past any byte this write can reach.
    uint8_t after[16];
    for (uint8_t i = 0; i < 16u; ++i) {
        after[i] = refused_after(i);
    }
    peer_nack_at = 0xFFFFu;
    peer_addr_hits = 0;
    peer_rx_n = 0;
    const uint8_t st2 = refused_request<Bus>(peer_addr, after, 16, nullptr, 0, next, s, false);
    const uint32_t entries = next == I2cEngine::dma ? 1u : 16u;
    const bool whole = st2 == i2c_ok && peer_rx_n == 16u && peer_addr_hits == 1u &&
                       same(peer_rx, after, 16) && host_isr_entries == entries;
    if (print_row || !right || !whole) {
        print(serial, "    ", row.tx, "+", row.rx, " refused at ", row.at, ": status ", st, " of ",
              want, ", the client took ", took, ", the next request status ", st2, ", took ",
              peer_rx_n, ", entries ", host_isr_entries, right && whole ? "" : " <- WRONG", crlf);
    }
    if (row.at == 0u) {
        pos.address.add(right);
    } else if (row.tx == 1u) {
        pos.one.add(right);
    } else if (row.at == 1u) {
        pos.first.add(right);
    } else if (row.at == 2u) {
        pos.second.add(right);
    } else {
        pos.later.add(right);
    }
    if (row.tx == 2u) {
        pos.two.add(right);
    }
    pos.next.add(whole);
}

/// A pass of every row on one host at one speed, nothing but the requests
/// between the rows.
template <typename Bus>
void refused_pass(I2cClock kernel, I2cSpeed s, I2cEngine e, I2cEngine next,
                  RefusedPositions& pos) {
    if constexpr (Bus::has_dma) {
        dma_host_live = true;
    } else {
        dma_host_live = false;
    }
    if (!Bus::init(clock, kernel)) {
        pos.address.add(false);
        return;
    }
    for (const RefusedRow& row : refused_rows) {
        refused_row<Bus>(row, e, next, s, false, pos);
    }
}

void refused_print(const char* name, const RefusedPositions& p) {
    print(serial, "  ", name, ": right of rows - the address ", p.address.right, "/", p.address.seen,
          ", a write of one ", p.one.right, "/", p.one.seen, ", the first byte ", p.first.right, "/",
          p.first.seen, ", the second ", p.second.right, "/", p.second.seen, ", a later one ",
          p.later.right, "/", p.later.seen, ", a write of two ", p.two.right, "/", p.two.seen,
          "; the next request whole ", p.next.right, "/", p.next.seen, crlf);
}

void tv_refusals() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    uint8_t serve[8];
    fill_pattern(serve, 8, 0x60);
    (void)peer_arm(serve, 8);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }
    (void)Peer::byte_control(true);
    Peer::interrupt(I2cInterrupt::transfer_complete, true);

    const I2cSpeed speeds[2] = {I2cSpeed::standard_100k, I2cSpeed::fast_400k};
    const I2cClock kernels[2] = {I2cClock::pclk, I2cClock::hsi16};
    RefusedPositions on_dma{};
    RefusedPositions on_pump{};
    for (const I2cClock k : kernels) {
        for (const I2cSpeed s : speeds) {
            refused_pass<DmaHost>(k, s, I2cEngine::dma, I2cEngine::dma, on_dma);
            refused_pass<DmaHost>(k, s, I2cEngine::dma, I2cEngine::interrupt, on_dma);
            refused_pass<DmaHost>(k, s, I2cEngine::interrupt, I2cEngine::dma, on_pump);
            refused_pass<DmaHost>(k, s, I2cEngine::interrupt, I2cEngine::interrupt, on_pump);
        }
    }
    refused_print("under dma", on_dma);
    refused_print("under interrupt", on_pump);
    bench.verdict("under dma THE ADDRESS REFUSED, writes of one, two and sixteen with and without "
                  "a read half: i2c_nack_addr, nothing at the client",
                  on_dma.address.whole());
    bench.verdict("... a write of one refused: i2c_nack_data, its byte counted", on_dma.one.whole());
    bench.verdict("... the first byte of two or more: i2c_nack_data, though the channel has filled "
                  "TXDR behind it and TXE reads clear",
                  on_dma.first.whole());
    bench.verdict("... the second byte, and a later one up to the last of 255: i2c_nack_data, the "
                  "client's count exactly the byte refused",
                  on_dma.second.whole() && on_dma.later.whole());
    bench.verdict("... a write of two, either byte", on_dma.two.whole());
    bench.verdict("... and the next request on either engine, no init() and no PE cycle between: "
                  "whole byte for byte, one entry on the engines and one a byte on the pump",
                  on_dma.next.whole());
    bench.verdict("the same rows on the interrupt, and the same next request on either engine",
                  on_pump.whole());

    // ---- the engineless host, its one engine ----
    RefusedPositions pump_only{};
    for (const I2cSpeed s : speeds) {
        refused_pass<Host>(I2cClock::pclk, s, I2cEngine::interrupt, I2cEngine::interrupt, pump_only);
    }
    refused_print("the engineless host", pump_only);
    bench.verdict("... and on the engineless host", pump_only.whole());

    // ---- A CHANNEL STARVED under a refused first byte ----
    // TXDMAEN goes away just after start(), so the first byte goes to the wire
    // with its TXIS unserved and the channel's count standing: only TXE says
    // the byte left, and the row reads i2c_nack_addr if that test goes. TXIS
    // stays up through the NACK and the STOP: 32.9.7 clears it on a store into
    // TXDR and offers no other way while NOSTRETCH is clear, and the next write
    // would storm its vector or pour its channel ahead of its START if start()
    // armed either before its preload.
    dma_host_live = true;
    (void)DmaHost::init(clock, I2cClock::pclk);
    uint8_t starved = 0;
    uint8_t txis_left = 0;
    uint8_t starved_next = 0;
    uint8_t after[16];
    for (uint8_t i = 0; i < 16u; ++i) {
        after[i] = refused_after(i);
    }
    for (uint8_t k = 0; k < 8u; ++k) {
        for (uint8_t i = 0; i < 16u; ++i) {
            tx_buf[i] = refused_value(k, i);
        }
        const uint8_t len = (k & 1u) != 0u ? 2u : 16u;
        const I2cEngine next = (k & 2u) != 0u ? I2cEngine::dma : I2cEngine::interrupt;
        peer_nack_at = 1;
        peer_rx_n = 0;
        const uint8_t st = refused_request<DmaHost>(peer_addr, tx_buf, len, nullptr, 0,
                                                    I2cEngine::dma, I2cSpeed::fast_400k, true);
        const uint16_t took = peer_rx_n;
        starved = static_cast<uint8_t>(
            starved + (st == i2c_nack_data && took == 1u && peer_rx[0] == tx_buf[0] ? 1u : 0u));
        txis_left = static_cast<uint8_t>(txis_left + ((H::flags() & I2C_ISR_TXIS) != 0u ? 1u : 0u));
        peer_nack_at = 0xFFFFu;
        peer_rx_n = 0;
        const uint8_t st2 = refused_request<DmaHost>(peer_addr, after, 16, nullptr, 0, next,
                                                     I2cSpeed::fast_400k, false);
        const uint32_t entries = next == I2cEngine::dma ? 1u : 16u;
        starved_next = static_cast<uint8_t>(
            starved_next + (st2 == i2c_ok && peer_rx_n == 16u && same(peer_rx, after, 16) &&
                                    host_isr_entries == entries
                                ? 1u
                                : 0u));
    }
    print(serial, "  a channel starved under a refused first byte, 8 rows: i2c_nack_data ", starved,
          ", TXIS left standing ", txis_left, ", the next request whole ", starved_next,
          "; the client's vector silenced by its storm budget ", peer_stormed, crlf);
    bench.verdict("A STARVED CHANNEL: its count stands under a byte already gone, and TXE says "
                  "i2c_nack_data",
                  starved == 8u);
    bench.verdict("... TXIS is left standing, the condition staged, and the next request's preload "
                  "takes it down: whole, its entries exact on either engine",
                  txis_left == 8u && starved_next == 8u);

    // ---- THE CONTROL: a write that does not flush ----
    // A refused first byte under dma leaves the channel's next byte in TXDR,
    // TXE clear. A write by the resource's verbs with no flush sends that byte
    // first and the NBYTES count leaves its own last byte behind: what start()'s
    // flush is there for, and what the judgement above would see without it.
    for (uint8_t i = 0; i < 16u; ++i) {
        tx_buf[i] = refused_value(0x55u, i);
    }
    peer_nack_at = 1;
    const uint8_t ctl = refused_request<DmaHost>(peer_addr, tx_buf, 16, nullptr, 0, I2cEngine::dma,
                                                 I2cSpeed::fast_400k, false);
    const bool stale = (H::flags() & I2C_ISR_TXE) == 0u;
    peer_nack_at = 0xFFFFu;
    peer_rx_n = 0;
    raw_host_live = true;
    H::clear(I2cClear::all);
    (void)H::transfer_now(peer_addr, false, 2, true);
    const uint32_t t0 = Ticker::millis();
    while ((H::flags() & I2C_ISR_STOPF) == 0u && Ticker::millis() - t0 < 5u) {
        if ((H::flags() & I2C_ISR_TXIS) != 0u) {
            H::data(after[0]);
        }
    }
    H::clear(I2cClear::all);
    raw_host_live = false;
    print(serial, "  the control: after a refused first byte status ", ctl, ", TXDR holding ", stale,
          "; a write of two with no flush, the client got ", peer_rx_n, ": ", hex(peer_rx[0]), " ",
          hex(peer_rx[1]), " against the channel's ", hex(tx_buf[1]), " and the write's ",
          hex(after[0]), crlf);
    bench.verdict("THE LETTER CAN FAIL: with no flush a refused write's byte goes out ahead of the "
                  "next write, which loses its last",
                  ctl == i2c_nack_data && stale && peer_rx_n == 2u && peer_rx[0] == tx_buf[1] &&
                      peer_rx[1] == after[0]);

    peer_nack_at = 0;
    dma_host_live = false;
    DmaHost::release();
    Peer::interrupt(I2cInterrupt::transfer_complete, false);
    (void)Peer::byte_control(false);
    (void)Host::init(clock, I2cClock::pclk);
    peer_stop();
}

// ===========================================================================
// B - ES0418 2.11.6's band by name
// ===========================================================================
//
// The erratum, on the STM32G071's sheet and not on this family's other two:
// with the first byte not in TXDR, two bytes are requested in a row, and if the
// first is written "in less than two I2C kernel clock cycles after the TXIS/DMA
// request" while the APB-to-kernel ratio is between 1.5 and 3, the second "is
// not internally detected" and the block stalls until a PE cycle. HSI16 against
// PCLK at 44 MHz is 2.75, and two kernel cycles are 125 ns, 5.5 cycles of the
// clock the channel runs on (this family's HPRE and PPRE are pinned at 1,
// stm32g0/clock.hpp). The host's writes judge its own path, whose first byte is
// in TXDR before the START on either engine. THE EMPTY ROW is the sequence the
// erratum describes, staged through the resource's verbs: its rate is printed and
// judged only for its accounting, since a sheet that does not list the item says
// nothing either way. That rate says whether this die stalls on that sequence.

/// A write of `len`, `tenures` times, at one speed.
struct BandShape {
    uint8_t len;
    uint16_t tenures;
};
constexpr BandShape band_shapes[3] = {{2, 200}, {16, 100}, {255, 10}};

struct BandRow {
    uint32_t tenures = 0;
    uint32_t whole = 0;      ///< answered i2c_ok, the client's count and bytes right
    uint32_t stalled = 0;    ///< no completion inside the bound, recovered by a PE cycle
    uint32_t accounted = 0;  ///< stalled, with the first byte at the client and nothing after it
    uint32_t wrong = 0;      ///< anything else
    uint32_t stall_isr = 0;  ///< at the first stall: ISR
    uint32_t stall_cndtr = 0;
    bool stall_scl = true;
    bool stall_sda = true;
    void add(const BandRow& o) {
        tenures += o.tenures;
        whole += o.whole;
        stalled += o.stalled;
        accounted += o.accounted;
        wrong += o.wrong;
        if (stalled == o.stalled && o.stalled != 0u) {
            stall_isr = o.stall_isr;
            stall_cndtr = o.stall_cndtr;
            stall_scl = o.stall_scl;
            stall_sda = o.stall_sda;
        }
    }
};

void band_fill(uint16_t k, uint8_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        tx_buf[i] = refused_value(k, i);
    }
}

/// THE EMPTY ROW'S TENURE: TXDR flushed, the transmit channel started on the
/// WHOLE run and its request enabled before the START, the host's vector
/// silenced and STOPF polled. Returns true when the STOP came inside the bound.
bool band_empty_tenure(uint8_t len, uint32_t limit_ms) {
    raw_host_live = true;
    H::enables(0u);
    H::clear(I2cClear::all);
    H::flush_tx();
    (void)HostTx::start(std::span<const uint8_t>(tx_buf, len));
    H::dma_transmit(true);
    (void)H::transfer_now(peer_addr, false, len, true);
    bool stopped = false;
    const uint32_t t0 = Ticker::millis();
    while (!stopped && Ticker::millis() - t0 < limit_ms) {
        stopped = (H::flags() & I2C_ISR_STOPF) != 0u;
    }
    return stopped;
}

/// What the empty tenure leaves is taken away, stalled or not: a stall by the
/// PE cycle the erratum names.
void band_empty_end(bool stopped) {
    H::dma_transmit(false);
    HostTx::stop();
    if (!stopped) {
        (void)DmaHost::recover();
    }
    H::clear(I2cClear::all);
    raw_host_live = false;
}

/// A batch at one speed and length on the path asked for: the first tenure of
/// every batch preceded by a probe that makes the host apply the speed.
BandRow band_batch(I2cSpeed s, const BandShape& sh, I2cEngine e, bool empty) {
    BandRow row{};
    const uint32_t limit = refused_limit_ms(sh.len, s);
    (void)refused_request<DmaHost>(peer_addr, nullptr, 0, nullptr, 0, I2cEngine::interrupt, s, false);
    for (uint16_t k = 0; k < sh.tenures; ++k) {
        band_fill(k, sh.len);
        row.tenures += 1u;
        peer_isr_entries = 0;   // the client's storm budget is a tenure's here
        peer_rx_n = 0;
        peer_addr_hits = 0;
        bool stopped = false;
        uint8_t st = no_answer;
        if (empty) {
            stopped = band_empty_tenure(sh.len, limit);
            st = stopped ? i2c_ok : no_answer;
        } else {
            typename DmaHost::Request r{};
            r.addr = peer_addr;
            r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
            r.tx_len = sh.len;
            r.speed = s;
            r.engine = e;
            host_done = false;
            if (!DmaHost::start(r)) {
                const uint32_t t0 = Ticker::millis();
                while (!host_done && Ticker::millis() - t0 < limit) {
                }
                st = host_done ? DmaHost::status() : no_answer;
            }
        }
        if (st == no_answer && row.stalled == 0u) {
            row.stall_isr = H::flags();
            row.stall_cndtr = DmaChannel<1, 1>::regs().CNDTR;
            row.stall_scl = SclPin::read();
            row.stall_sda = SdaPin::read();
        }
        if (empty) {
            band_empty_end(stopped);
        } else if (st == no_answer) {
            (void)DmaHost::recover();
        }
        if (st == i2c_ok && peer_rx_n == sh.len && peer_addr_hits == 1u &&
            same(peer_rx, tx_buf, sh.len)) {
            row.whole += 1u;
        } else if (st == no_answer) {
            row.stalled += 1u;
            row.accounted += peer_rx_n <= 1u && (peer_rx_n == 0u || peer_rx[0] == tx_buf[0]) ? 1u : 0u;
        } else {
            row.wrong += 1u;
        }
    }
    return row;
}

/// A row, its rate in parts per ten thousand.
void band_print(const char* name, const BandRow& r) {
    print(serial, "  ", name, ": ", r.tenures, " tenures, ", r.whole, " whole, ", r.stalled,
          " STALLED (", r.tenures != 0u ? r.stalled * 10000u / r.tenures : 0u,
          " per ten thousand), ", r.accounted, " of those with the first byte alone at the client, ",
          r.wrong, " wrong", crlf);
    if (r.stalled != 0u) {
        print(serial, "    the first stall: ISR ", hex(r.stall_isr), ", the channel's count ",
              r.stall_cndtr, ", SCL ", r.stall_scl ? "high" : "LOW", ", SDA ",
              r.stall_sda ? "high" : "LOW", crlf);
    }
}

void tB_band() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    uint8_t serve[8];
    fill_pattern(serve, 8, 0x60);
    (void)peer_arm(serve, 8);
    SysClock::set_index(r_band);
    console_drain();
    dma_host_live = true;
    const bool up = DmaHost::init(clock, I2cClock::hsi16);
    const uint32_t pclk = SysClock::hz();
    const uint32_t ker = DmaHost::kernel_hz();
    print(serial, "  PCLK / kernel: ", pclk, " / ", ker, " Hz", crlf);
    bench.verdict("HSI16 against PCLK at 44 MHz, a ratio of 2.75: inside the band of 1.5 to 3",
                  up && ker == 16'000'000UL && 2u * pclk >= 3u * ker && pclk <= 3u * ker);

    // ---- what a stall looks like to this letter ----
    // TXDMAEN taken away after start(): the first byte goes, the channel never
    // answers its TXIS, and SCL stays low with the tenure parked, inside the
    // bound. The next write after the PE cycle is whole.
    band_fill(1u, 16);
    peer_rx_n = 0;
    const uint8_t staged = refused_request<DmaHost>(peer_addr, tx_buf, 16, nullptr, 0,
                                                    I2cEngine::dma, I2cSpeed::fast_400k, true);
    const uint16_t staged_took = peer_rx_n;
    peer_rx_n = 0;
    const uint8_t after_staged = refused_request<DmaHost>(peer_addr, tx_buf, 16, nullptr, 0,
                                                          I2cEngine::dma, I2cSpeed::fast_400k, false);
    print(serial, "  a stall by hand: status ", staged, ", the client took ", staged_took,
          "; after the PE cycle status ", after_staged, ", took ", peer_rx_n, crlf);
    bench.verdict("A STALL IS SEEN: a channel taken away parks a tenure inside the bound, the "
                  "first byte alone at the client, and the PE cycle recovers it",
                  staged == no_answer && staged_took == 1u && after_staged == i2c_ok &&
                      peer_rx_n == 16u && same(peer_rx, tx_buf, 16));

    const I2cSpeed speeds[2] = {I2cSpeed::standard_100k, I2cSpeed::fast_400k};
    BandRow host_dma{};
    BandRow host_pump{};
    BandRow host_one{};
    BandRow empty{};
    for (const I2cSpeed s : speeds) {
        for (const BandShape& sh : band_shapes) {
            host_dma.add(band_batch(s, sh, I2cEngine::dma, false));
            empty.add(band_batch(s, sh, I2cEngine::dma, true));
        }
        host_one.add(band_batch(s, BandShape{1, 200}, I2cEngine::dma, false));
        host_pump.add(band_batch(s, BandShape{16, 100}, I2cEngine::interrupt, false));
    }
    band_print("the host under dma, 2, 16 and 255 bytes", host_dma);
    band_print("the host under dma, a write of one", host_one);
    band_print("the host on the interrupt, 16 bytes", host_pump);
    band_print("TXDR LEFT EMPTY, the channel on the whole run, 2, 16 and 255 bytes", empty);
    bench.verdict("inside the band the host's own path under dma: no stall and every byte of every "
                  "write, a write of one included",
                  host_dma.tenures != 0u && host_dma.whole == host_dma.tenures &&
                      host_one.tenures != 0u && host_one.whole == host_one.tenures);
    bench.verdict("... and on the interrupt", host_pump.tenures != 0u &&
                                                  host_pump.whole == host_pump.tenures);
    bench.verdict("the empty row accounted, tenure by tenure: whole, or stalled with the first byte "
                  "alone at the client and recovered by the PE cycle. Its rate is printed, not "
                  "judged: it says whether this die stalls on that sequence",
                  empty.tenures != 0u && empty.wrong == 0u && empty.accounted == empty.stalled);

    dma_host_live = false;
    DmaHost::release();
    SysClock::set_index(r_fast);
    console_drain();
    (void)Host::init(clock, I2cClock::pclk);
    peer_stop();
}

// ===========================================================================
// i - SMBus: the PEC and the three time-outs
// ===========================================================================

void ti_smbus() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    uint8_t answers[8];
    fill_pattern(answers, 8, 0x44);
    (void)peer_arm(answers, 8);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }

    // ---- the PEC, end to end ----
    // Both instances have the SMBus half on this part (letter a asked
    // the silicon), so PECEN can be raised and the checksum becomes the
    // last byte of the tenure.
    (void)H::disable();
    I2cConfig hc{};
    hc.timing = *i2c_timing_for(SysClock::hz(), I2cSpeed::fast_400k);
    hc.pec = true;
    const bool pec_cfg = H::configure(hc);
    H::enable();
    bench.verdict("PECEN is accepted on an instance that has SMBus", pec_cfg);

    raw_host_live = true;
    fill_pattern(tx_buf, 4, 0x21);
    peer_rx_n = 0;
    H::clear(I2cClear::all);
    // 32.4.12: with PECBYTE set and RELOAD clear, the PEC is sent after
    // NBYTES - 1 data bytes. So NBYTES counts the checksum too.
    (void)H::transfer(peer_addr, false, 5, true);
    (void)H::pec_byte(true);
    H::start();
    bool ok = true;
    for (uint8_t i = 0; i < 4u && ok; ++i) {
        // A byte at 400 kHz is 22.5 us; 10 ms only tells an event from none,
        // and the TXIS test below judges which one came.
        (void)brio::wait_until<Ruler>([] { return H::flag(I2cFlag::txis | I2cFlag::nack); },
                                      10'000u);
        if (!H::flag(I2cFlag::txis)) {
            ok = false;
            break;
        }
        H::data(tx_buf[i]);
    }
    // The checksum and the STOP follow within two byte times; a STOP that
    // never comes leaves the count the checksum verdict judges.
    (void)brio::wait_until<Ruler>([] { return H::flag(I2cFlag::stop); }, 10'000u);
    const uint8_t host_pec = H::pec();
    const uint16_t got = peer_rx_n;
    H::clear(I2cClear::all);
    // THE REFERENCE IS THE WHOLE MESSAGE INCLUDING THE ADDRESS BYTE, as
    // 32.4.11 says ("all the message bytes including addresses and
    // R/W bits").
    uint8_t msg[8];
    msg[0] = static_cast<uint8_t>(peer_addr << 1);
    for (uint8_t i = 0; i < 4u; ++i) {
        msg[i + 1u] = tx_buf[i];
    }
    const uint8_t want = pec_reference(msg, 5);
    print(serial, "  PEC: hardware ", hex(host_pec), " against a bitwise reference ",
          hex(want), "; the client received ", got, " bytes (4 data + the checksum)",
          crlf);
    bench.verdict("a PEC-protected write completes", ok);
    // THE HARDWARE PEC IS THE STANDARD'S CRC-8 OVER ADDRESS AND DATA -
    // pinned against a bitwise reference, which is the only way to know
    // it and not merely to trust it.
    bench.verdict("the hardware PEC equals the bitwise CRC-8 of the whole message",
                  host_pec == want);
    // AND THE CHECKSUM IS A BYTE ON THE WIRE: the target received one
    // more byte than the four written.
    bench.verdict("the checksum travels as a fifth byte the target receives",
                  got == 5u);
    // THE TARGET'S OWN CHECK IS DECLINED, WITH ITS REASON. Table 176
    // makes a checking target "SBC = 1, RELOAD = 0, PECBYTE = 1": the
    // check rides TARGET BYTE CONTROL, whose NBYTES has to be re-armed
    // by software after every byte. This suite's client pump is a plain
    // ADDR/RXNE/STOPF machine and byte control is letter c's alone, so
    // the target half of the PEC is not staged - said, not faked. The
    // client's PECR reads 0 exactly because nothing asked it to check.
    bench.skip("the target's own PEC check needs byte control (table 176's SBC = 1) and "
               "this pump has none");
    raw_host_live = false;
    H::reset();
    (void)Host::init(clock, I2cClock::pclk);

    // ---- THE TIME-OUTS: whose hold does each one police? ----
    // 32.4.12 reads as though TIMEOUTA policed the WIRE - "if SCL is
    // tied low for longer than
    // (TIMEOUTA + 1) x 2048 x tI2CCLK, the TIMEOUT flag is set" - so the
    // bench asks, with a control on each side.
    (void)H::disable();
    const uint32_t ker = H::kernel_hz(SysClock::hz());
    const auto code_a = i2c_timeout_code_for(ker, 4'000u, false);
    const auto code_b = i2c_timeout_code_for(ker, 4'000u, false);
    bench.verdict("32.4.13's arithmetic produces a code for a 4 ms limit",
                  code_a.has_value() && code_b.has_value());
    print(serial, "  TIMEOUTA code ", *code_a, " = ", i2c_timeout_us(ker, *code_a, false),
          " us at a ", ker / 1000u, " kHz kernel", crlf);
    (void)H::timeouts(*code_a, false, true, *code_b, false);
    H::enable();

    // (1) THE CONTROL: the HOST'S OWN hold. A tenure started and then
    // left unserved - its own interrupts cut - holds SCL low itself.
    (void)peer_arm(answers, 8);
    raw_host_live = true;
    H::clear(I2cClear::all);
    (void)H::transfer_now(peer_addr, false, 4, true);
    spin_us<Ruler>(20'000);
    const bool own_tripped = H::flag(I2cFlag::timeout);
    const uint32_t own_isr = H::flags();
    H::clear(I2cClear::all);
    (void)H::disable();
    H::enable();
    raw_host_live = false;
    print(serial, "  the HOST's own unserved hold, 20 ms against a 4 ms limit: ",
          own_tripped ? "TIMEOUT set" : "quiet", " (ISR ", hex(own_isr), ")", crlf);
    bench.verdict("TIMEOUTA sees THIS controller's own clock hold",
                  own_tripped);

    // (2) THE QUESTION: a PEER'S hold.
    (void)Host::init(clock, I2cClock::pclk);
    (void)H::disable();
    (void)H::timeouts(*code_a, false, true, *code_b, false);
    H::enable();
    (void)peer_arm(answers, 8);
    peer_stretch_us = 6'000;   // half again the limit, on every byte
    uint8_t st = 0;
    for (uint16_t i = 0; i < 8u; ++i) {
        rx_buf[i] = 0;
    }
    const uint32_t c = host_tenure_cycles(peer_addr, nullptr, 0, rx_buf, 2,
                                          I2cSpeed::fast_400k, st);
    peer_stretch_us = 0;
    const bool peer_tripped = H::flag(I2cFlag::timeout);
    print(serial, "  a CLIENT holding SCL 6 ms with the same 4 ms limit: ",
          peer_tripped ? "TIMEOUT set" : "quiet", ", tenure ", cycles_us(c),
          " us, status ", st, crlf);
    // THE ANSWER, AND IT AGREES WITH THE SAM AGAINST THE CHAPTER'S OWN
    // WORDING: a peer's stretch does not trip TIMEOUTA on this silicon
    // either. Which is why util/i2c_bus.hpp's per-bus timeout is the
    // arbiter's and not the engine's on all three targets.
    bench.verdict("A PEER'S HOLD DOES NOT TRIP IT - 32.4.12's 'SCL tied low' is this "
                  "controller's own hold",
                  !peer_tripped);
    print(serial, "    (the tenure's own length is not this letter's subject: a "
                  "6 ms hold on every event of a 2-byte read is tens of "
                  "milliseconds of wire)", crlf);
    H::clear(I2cClear::timeout);

    // (3) TIDLE = 1 is the OTHER thing TIMEOUTA can be: bus idle
    // detection, both lines high for the period.
    (void)H::disable();
    const auto code_idle = i2c_timeout_code_for(ker, 50u, true);
    (void)H::timeouts(*code_idle, true, true, *code_b, false);
    H::enable();
    H::clear(I2cClear::all);
    spin_us<Ruler>(2000);
    const bool idle_seen = H::flag(I2cFlag::timeout);
    print(serial, "  TIDLE at 50 us on a bus idle for 2 ms: ",
          idle_seen ? "TIMEOUT set" : "quiet", crlf);
    // MEASURED AND REPORTED EITHER WAY: the flag is what it is, and a
    // verdict that only passed on one outcome would be a wish.
    bench.verdict("TIDLE = 1 is accepted and the register holds it",
                  (H::timeouts() & I2C_TIMEOUTR_TIDLE) != 0u);
    print(serial, "    -> bus idle detection did ",
          idle_seen ? "" : "NOT ", "raise TIMEOUT here; recorded, not judged", crlf);

    // (4) TIMEOUTB watches THIS peripheral's own cumulative stretch, in
    // whichever role it plays - 32.9.6's own field description. A host
    // that never stretches cannot trip it, and that is the verdict.
    (void)H::disable();
    (void)H::timeouts(0, false, false, *code_b, true);
    H::enable();
    H::clear(I2cClear::all);
    (void)peer_arm(answers, 8);
    peer_stretch_us = 6'000;
    (void)host_tenure(peer_addr, nullptr, 0, rx_buf, 2, I2cSpeed::fast_400k);
    peer_stretch_us = 0;
    const bool b_tripped = H::flag(I2cFlag::timeout);
    print(serial, "  TIMEOUTB at 4 ms against the same 6 ms client hold: ",
          b_tripped ? "TRIPPED" : "quiet", crlf);
    bench.verdict("TIMEOUTB does NOT see a peer's hold either: it is tLOW:MEXT, this "
                  "controller's OWN cumulative stretch (32.9.6)",
                  !b_tripped);
    (void)H::disable();
    (void)H::timeouts(0, false, false, 0, false);
    H::enable();
    H::clear(I2cClear::all);

    // The ALERT: SMBA is a pad, and this bench has no wire for one.
    bench.skip("the SMBus ALERT needs a wire to an SMBA pad and this bench has none");
    (void)Host::init(clock, I2cClock::pclk);
    peer_stop();
}

// ===========================================================================
// j - the wake from Stop
// ===========================================================================


void tj_wake() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    if (!C::wakes_from_stop) {
        bench.skip("this part's I2C2 has no wake from Stop (table 165)");
        return;
    }
    uint8_t answers[8];
    fill_pattern(answers, 8, 0x5C);
    (void)peer_arm(answers, 8);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }
    // The client's kernel is HSI16 already (peer_arm's own choice), which
    // is 32.4.16's first condition; DNF is 0 and NOSTRETCH clear, which
    // are the other two.
    bench.verdict("WUPEN is granted: HSI16 kernel, DNF 0, stretching on",
                  Peer::wake_from_stop(true));
    bench.verdict("... and its EXTI line is unmasked (line 22, DIRECT)",
                  Exti::interrupt(C::exti_line));

    // ES0548 2.2.4: a peripheral with clock-request capability does not
    // wake the device while HSIDIV is anything but 000. At the first rung
    // the core is on the PLL and HSIDIV is 0, so the hazard does not
    // apply - said, because it is exactly the trap that catches a wake
    // arranged at any other rung.
    print(serial, "  ES0548 2.2.4 does not apply here: HSIDIV is 0 at this rung", crlf);

    // ---- AND THE STOP ITSELF IS DECLINED, WITH ITS REASON ----
    // The wake needs an ADDRESS ON THE WIRE while the core is stopped,
    // and on this bench BOTH ENDS OF THE BUS ARE ON THE SAME DIE: a Stop
    // that silences the client silences the controller that would wake
    // it. Nothing else here can put a START on SCL either - the DMA and
    // every timer stop with the core, and a bit-banged edge needs the
    // CPU. So the address-match wake wants a SECOND NODE, and this
    // letter measures everything around it instead of faking the middle.
    // (Measured: a Stop 1 entered with a tenure in flight hangs the
    // board - the host freezes mid-byte with the client stretching, and
    // neither can wake the other.)
    bench.skip("the wake itself needs a SECOND NODE - both ends of this bus stop together - "
               "so the Stop is letter t's, not faked here");

    // What CAN be measured: that the arming survives, that the line is
    // the DIRECT one table 65 names, and that the three conditions are
    // really conditions and not decoration.
    bench.verdict("WUPEN reads back set", Peer::wake_from_stop());
    bench.verdict("the wake line is EXTI 22 and it is a DIRECT line (no trigger of "
                  "its own to configure)",
                  C::exti_line == 22 && !exti_line_configurable(C::exti_line) &&
                      exti_line_implemented(C::exti_line));
    // The bus still works with the wake armed - an armed WUPEN is not a
    // mode that costs anything while the core runs.
    fill_pattern(tx_buf, 6, 0x9A);
    const uint8_t st = host_tenure(peer_addr, tx_buf, 6, nullptr, 0,
                                   I2cSpeed::standard_100k);
    bench.verdict("a bus with the wake armed carries bytes exactly as before",
                  st == i2c_ok && peer_rx_n == 6u && same(peer_rx, tx_buf, 6));
    // And the interlocks, from the other side: the wake goes away when
    // any of its three conditions does.
    (void)Peer::wake_from_stop(false);
    bench.verdict("disarming it clears WUPEN and masks the line",
                  !Peer::wake_from_stop() && !Exti::interrupt(C::exti_line));
    // ES0548 2.2.4 belongs here even though it is not this chapter's: a
    // peripheral with clock-request capability does not wake the device
    // at all while HSIDIV is anything but 000, which is exactly what an
    // I2C wake is. At this rung the core is on the PLL and HSIDIV is 0.
    print(serial, "  ES0548 2.2.4 stated: an I2C wake needs HSIDIV = 0, which this "
                  "rung has; not reproducible without the Stop above", crlf);

    peer_stop();
}

// ===========================================================================
// k - the stuck bus and unstick()
// ===========================================================================

void tk_unstick() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    uint8_t answers[8];
    fill_pattern(answers, 8, 0x37);
    (void)peer_arm(answers, 8);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }

    // A HEALTHY WIRE IS LEFT ALONE - zero pulses is both the answer and
    // the action.
    const uint8_t clean = Host::unstick();
    print(serial, "  a healthy bus: unstick() returns ", clean, crlf);
    bench.verdict("unstick() on a clean bus clocks nothing and says so", clean == 0u);
    bench.verdict("... and the bus still works", link_alive());

    // Now the client holds SDA low by GPIO, which is exactly what a
    // wedged device does, and the host's next tenure meets it.
    peer_live = false;
    PeerSdaPin::clear();
    PeerSdaPin::output(false, {.open_drain = true});
    const bool sda_low = !SdaPin::read();
    bench.verdict("the client's pad really holds SDA low (read from the host's end)",
                  sda_low);
    uint8_t one = 0x01;
    const uint8_t st = host_tenure(peer_addr, &one, 1, nullptr, 0, I2cSpeed::fast_400k);
    print(serial, "  a tenure into a held SDA: status ", st, crlf);
    bench.verdict("a tenure into a held wire does not silently succeed", st != i2c_ok);

    // unstick() clocks until the client lets go. The release is
    // commanded: the pad goes back to the peripheral after four clocks'
    // worth of time, so the count is a real measurement and not a
    // constant.
    const uint8_t pulses = Host::unstick();
    print(serial, "  unstick() with SDA still held: ", pulses, crlf);
    bench.verdict("nine clocks and a STOP cannot free a pad nothing releases: 0xFF",
                  pulses == 0xFFu);
    PeerSdaPin::function(client_pins.sda.function,
                         {.open_drain = true, .speed = PinSpeed::very_high});
    peer_live = true;
    const uint8_t after = Host::unstick();
    print(serial, "  ... and once the pad is given back: ", after, crlf);
    bench.verdict("with the wire free again unstick() clocks nothing", after == 0u);
    (void)Host::recover();
    (void)peer_arm(answers, 8);
    bench.verdict("the next tenure is i2c_ok", link_alive());
    peer_stop();
}

// ===========================================================================
// l - the kernel: I2cBus over I2cHost
// ===========================================================================

namespace kl {

/// The pending depth is 4 and letters l and r queue six, so the rejection
/// is reachable. The per-bus timeout is 20 ms: far above every tenure the
/// two letters send through the arbiter (four bytes at 400 kHz, about
/// 0.1 ms of wire, no stretch commanded), and far below the wedges they
/// stage - SDA held by the client's own pad until the reply comes (letter
/// l) and by the peer for 300 ms (letter r).
constexpr uint32_t bus_timeout_ticks = ticks_from_ms<P>(20);
using I2cArb = I2cBus<Host, P, 4, BusPassThrough, bus_timeout_ticks>;

/// THE LATEST A WEDGED TENURE CAN BE ANSWERED, in ticks counted from the
/// post: the arbiter arms its time event when the tenure goes asynchronous,
/// in the first pump after the post - one tick boundary at most between the
/// two - and kernel/time_event.hpp matures a delay of N ticks on the first
/// turn at which now() reaches the deadline, never later than N tick
/// boundaries after the arming. The pump serves it on that turn: recover()'s
/// PE cycle and the reply's dispatch, microseconds, at most one more
/// boundary before the letter reads the tick count. Sooner is legal (an
/// engine that reads back a level it did not drive answers at once).
constexpr uint32_t wedge_answer_ticks = bus_timeout_ticks + 2u;

uint8_t out_a[4];
uint8_t out_b[4];

/// The requester: it does nothing but remember what came back, in the
/// order it came back.
class Probe {
public:
    using Event = std::variant<I2cDone, SleepVote>;
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies[8];
    static inline uint8_t n = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;

    static void init() { clear_tally(); }
    static void clear_tally() {
        n = 0;
        rejected = 0;
        votes = 0;
        last_vote = false;
    }

    static void dispatch(const Event& e) {
        brio::match(
            e,
            [](const I2cDone& d) {
                if (n < 8u) {
                    replies[n] = d.status;
                }
                ++n;
                if (d.status == i2c_rejected) {
                    ++rejected;
                }
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
            });
    }
};

using BusKernel = Tenuto<P, Probe, I2cArb>;

/// The kernel's own loop, minus the sleep. Tenuto::run() matures the
/// TIME EVENTS before every step, and the per-bus timeout IS a time
/// event - a pump that only called step() would wait for a deadline
/// nothing was advancing.
void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

/// step() returns false the moment every queue is momentarily empty -
/// which on this bus is WHILE THE TENURE IS STILL ON THE WIRE, since the
/// completion arrives from an interrupt long after the AO has run. So a
/// letter that wants N replies has to keep pumping until they arrive or
/// a deadline passes; a bare pump() reads a bus in flight as a bus done.
void pump_until(uint8_t want, uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        pump();
        if (Probe::n >= want) {
            break;
        }
    }
    pump();
}

/// Everything queued, run out - between the phases of letter l, so one
/// phase's leftovers cannot be counted into the next.
void drain(uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        pump();
    }
}

Host::Request request(uint8_t addr, const uint8_t* tx) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = 4;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(nullptr));
    r.rx_len = 0;
    r.speed = I2cSpeed::fast_400k;
    r.reply = reply_to<Probe, I2cDone>();
    return r;
}

}  // namespace kl

void tl_kernel() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    uint8_t answers[8];
    fill_pattern(answers, 8, 0x4D);
    (void)peer_arm(answers, 8);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return;
    }
    kl::BusKernel::init_all();
    bus_ao_live = true;

    // Four tenures queued at once: the arbiter serializes them and the
    // replies come back in order.
    fill_pattern(kl::out_a, 4, 0x01);
    fill_pattern(kl::out_b, 4, 0x02);
    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::I2cArb>(kl::request((i == 2u) ? nobody_addr : peer_addr, kl::out_a));
    }
    kl::pump_until(4, 300);
    print(serial, "  four queued tenures: replies ", kl::Probe::n, " [",
          kl::Probe::replies[0], " ", kl::Probe::replies[1], " ",
          kl::Probe::replies[2], " ", kl::Probe::replies[3], "]", crlf);
    bench.verdict("four tenures through I2cBus, four replies", kl::Probe::n == 4u);
    bench.verdict("... in order, with the NACK delivered IN ITS PLACE as a reply",
                  kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_ok &&
                      kl::Probe::replies[2] == i2c_nack_addr &&
                      kl::Probe::replies[3] == i2c_ok);

    // The FIFO is four deep: what will not fit is rejected on the spot.
    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::I2cArb>(kl::request(peer_addr, kl::out_b));
    }
    kl::pump_until(6, 300);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n,
          ", rejected ", kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately",
                  kl::Probe::rejected != 0u);
    bench.verdict("... and every request is still answered exactly once",
                  kl::Probe::n == 6u);

    // The sleep vote, both ways: an idle bus says yes.
    kl::drain(100);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep",
                  kl::Probe::votes == 1u && kl::Probe::last_vote);
    // ... and a busy one says no. The tenure is queued first and the
    // vote asked in the same pump, so the arbiter is mid-transfer.
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(peer_addr, kl::out_a));
    post<kl::I2cArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    kl::drain(100);
    print(serial, "  the vote from a BUSY bus: ", kl::Probe::votes, " vote(s), last ",
          kl::Probe::last_vote ? "yes" : "no", crlf);
    bench.verdict("a BUSY bus votes against it", kl::Probe::votes == 1u &&
                                                     !kl::Probe::last_vote);

    // ---- THE TIMED BUS ----
    // A tenure into a client that holds SDA down: the wire is dead for
    // longer than the arbiter's limit, and the ARBITER is what notices -
    // there is no silicon time-out on any engine of this project that
    // watches a wire a client wedged (util/i2c_bus.hpp says so, and
    // letter i measures which of this block's three really do).
    kl::Probe::clear_tally();
    peer_live = false;
    PeerSdaPin::clear();
    PeerSdaPin::output(false, {.open_drain = true});
    post<kl::I2cArb>(kl::request(peer_addr, kl::out_a));
    const uint32_t t0 = Ticker::ticks();
    kl::pump_until(1, 400);
    const uint32_t took = Ticker::ticks() - t0;
    const bool still_low = !SdaPin::read();
    PeerSdaPin::function(client_pins.sda.function,
                         {.open_drain = true, .speed = PinSpeed::very_high});
    peer_live = true;
    print(serial, "  the wedged tenure answered ", kl::Probe::n, " with status ",
          kl::Probe::replies[0], " after ", took, " ms; SDA still low at the reply: ",
          still_low ? "yes" : "no", crlf);
    bench.verdict("a wedged tenure is answered - by the ARBITER, not the engine",
                  kl::Probe::n == 1u);
    // The status is the arbiter's own code where the engine never spoke,
    // and the engine's where the silicon reported first: both are a
    // report and neither is silence.
    bench.verdict("... with i2c_timeout or a wire code, never i2c_ok",
                  kl::Probe::replies[0] != i2c_ok);
    bench.verdict("... at the arbiter's own limit or sooner (two ticks over it at most), and "
                  "not the wedge's length",
                  took <= kl::wedge_answer_ticks);

    (void)peer_arm(answers, 8);
    kl::drain(50);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(peer_addr, kl::out_a));
    kl::pump_until(1, 300);
    bench.verdict("THE SAME BUS AO carries the next tenure to i2c_ok after recover()",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_ok);
    bus_ao_live = false;
    peer_stop();
}

// ===========================================================================
// m - the errata
// ===========================================================================

void tm_errata() {
    // ---- 2.10.1, as the refusals it is ----
    // The erratum has no register workaround, so the only honest form it
    // can take in a driver is a refusal - and its floors beat the
    // datasheet's own table 74 in every mode.
    print(serial, "  2.10.1 floors: Sm ", i2c_min_kernel_hz(I2cSpeed::standard_100k) / 1000u,
          " kHz, Fm ", i2c_min_kernel_hz(I2cSpeed::fast_400k) / 1000u, " kHz, Fm+ ",
          i2c_min_kernel_hz(I2cSpeed::fast_plus_1m) / 1000u, " kHz", crlf);
    print(serial, "  DS13560 table 74: Sm ",
          i2c_datasheet_min_kernel_hz(I2cSpeed::standard_100k, I2cFilters{}) / 1000u,
          " kHz, Fm ",
          i2c_datasheet_min_kernel_hz(I2cSpeed::fast_400k, I2cFilters{}) / 1000u,
          " kHz, Fm+ ",
          i2c_datasheet_min_kernel_hz(I2cSpeed::fast_plus_1m, I2cFilters{}) / 1000u,
          " kHz", crlf);
    // Constants of the arithmetic: the compiler checks them, the run prints them.
    static_assert(i2c_min_kernel_hz(I2cSpeed::standard_100k) >
                          i2c_datasheet_min_kernel_hz(I2cSpeed::standard_100k, I2cFilters{}) &&
                      i2c_min_kernel_hz(I2cSpeed::fast_400k) >
                          i2c_datasheet_min_kernel_hz(I2cSpeed::fast_400k, I2cFilters{}) &&
                      i2c_min_kernel_hz(I2cSpeed::fast_plus_1m) >
                          i2c_datasheet_min_kernel_hz(I2cSpeed::fast_plus_1m, I2cFilters{}),
                  "the erratum's floor is STRICTER than the datasheet's in all three modes");
    static_assert(!i2c_timing_for(3'900'000UL, I2cSpeed::standard_100k).has_value() &&
                      i2c_timing_for(4'000'000UL, I2cSpeed::standard_100k).has_value(),
                  "3.9 MHz cannot make a 100 kHz bus and 4 MHz can");
    static_assert(!i2c_timing_for(19'900'000UL, I2cSpeed::fast_plus_1m).has_value() &&
                      i2c_timing_for(20'000'000UL, I2cSpeed::fast_plus_1m).has_value(),
                  "19.9 MHz cannot make a 1 MHz bus and 20 MHz can");
    print(serial, "  held at compile time (static_assert): the erratum's floor stricter than the "
                  "datasheet's in all three modes, 3.9 MHz refused for 100 kHz and 4 MHz taken, "
                  "19.9 MHz refused for 1 MHz and 20 MHz taken",
          crlf);
    // WHAT IS NOT STAGED, and why: reproducing the erratum means driving
    // a transmitter whose tSU;DAT is under one kernel period, which on
    // this bench means a bit-banged sender on a pad the peripheral must
    // also own. There is no third pad on either net.
    bench.skip("the wrong sampling itself needs a bit-banged sender on a pad the peripheral "
               "owns");

    // ---- 2.10.2, as what the run has counted ----
    const uint16_t berr = Host::spurious_bus_errors();
    // Printed and not judged: the count is whatever this board's wire raised,
    // one specimen, and the erratum's workaround ("clear it and go on") is
    // what the tenure verdicts across the run already exercise.
    print(serial, "  2.10.2: BERR swept ", berr,
          " times since this host's init (never a status: the erratum's own workaround)",
          crlf);
    if (self_link) {
        uint8_t answers[4];
        fill_pattern(answers, 4, 0x01);
        (void)peer_arm(answers, 4);
        bench.verdict("... and the bus works, which is what 'the transfer continues "
                      "normally' means",
                      link_alive());
        peer_stop();
    } else {
        bench.skip("the closing 'and the bus still works' leg wants the I2C2 client and the "
                   "self-link is not on the desk (letter n is the same statement against the "
                   "far board)");
    }
}

// ===========================================================================
// x - the polled trace, OUTSIDE z
// ===========================================================================
//
// A diagnostic and not a test: both instances driven with their NVIC
// lines DISABLED and every step polled, so a standing flag cannot storm
// a vector and the console cannot be starved by one. It is what a
// suite's first version needs when a wire letter wedges the board, and
// it is kept because the next person will need it too.

void tx_trace() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    uint8_t answers[8];
    fill_pattern(answers, 8, 0x61);
    (void)peer_arm(answers, 8);
    peer_live = false;
    Nvic::disable(C::irq());
    Nvic::disable(H::irq());

    // The host, by hand, at 100 kHz.
    (void)H::disable();
    const auto t = *i2c_timing_for(SysClock::hz(), I2cSpeed::standard_100k);
    (void)H::timing(t);
    H::enable();
    H::clear(I2cClear::all);

    uint32_t trace[24];
    uint8_t n = 0;
    trace[n++] = H::flags();
    trace[n++] = C::flags();
    (void)H::transfer(peer_addr, false, 2, true);
    trace[n++] = H::cr2();
    H::start();
    for (uint8_t step = 0; step < 8u && n < 20u; ++step) {
        // The next event, bounded at 2 ms - a byte at 100 kHz is 90 us; a step
        // where nothing changed is traced as it stands.
        const uint32_t before = H::flags();
        (void)brio::wait_until<Ruler>([before] { return H::flags() != before; }, 2'000u);
        trace[n++] = H::flags();
        trace[n++] = C::flags();
        // Serve both ends by hand, exactly as the two pumps would.
        if (C::flag(I2cFlag::addr)) {
            C::clear(I2cClear::addr);
        }
        if (C::flag(I2cFlag::rxne)) {
            (void)C::data();
        }
        if (H::flag(I2cFlag::txis)) {
            H::data(static_cast<uint8_t>(0xB0 + step));
        }
        if (H::flag(I2cFlag::stop)) {
            break;
        }
    }
    console_drain();
    print(serial, "  host CR1 ", hex(H::regs().CR1), " CR2 ", hex(H::cr2()),
          " TIMINGR ", hex(H::timing_reg()), crlf);
    print(serial, "  client CR1 ", hex(C::regs().CR1), " OAR1 ", hex(C::oar1()),
          " TIMINGR ", hex(C::timing_reg()), crlf);
    print(serial, "  SCL pad ", SclPin::read() ? 1 : 0, " SDA pad ",
          SdaPin::read() ? 1 : 0, crlf);
    for (uint8_t i = 0; i < n; ++i) {
        print(serial, "  [", i, "] ", hex(trace[i]), crlf);
    }
    H::clear(I2cClear::all);
    C::clear(I2cClear::all);
    (void)Host::init(clock, I2cClock::pclk);
    peer_stop();
    print(serial, "  the trace above is a diagnostic: no verdict", crlf);
}

/// The same tenure with the vectors LIVE, bounded, with both lines cut
/// before a word is printed - so a storm is counted and named instead of
/// starving the console that would report it.
void ty_isr_trace() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
        return;
    }
    uint8_t answers[8];
    fill_pattern(answers, 8, 0x61);
    (void)peer_arm(answers, 8);
    (void)Host::init(clock, I2cClock::pclk);
    peer_isr_entries = 0;
    host_isr_entries = 0;
    peer_trace_n = 0;
    host_trace_n = 0;
    trace_isrs = true;

    Host::Request r{};
    static uint8_t two[2] = {0x5A, 0x5B};
    r.addr = peer_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(two));
    r.tx_len = 2;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.rx_len = 2;
    r.speed = I2cSpeed::fast_400k;
    host_done = false;
    const bool sync = Host::start(r);
    // Two bytes each way at 400 kHz are 0.2 ms of wire; bounded at 20 ms, and
    // whether it completed is read below and printed.
    (void)brio::wait_until<Ruler>([] { return host_done; }, 20'000u);
    // BOTH LINES CUT before anything is said.
    Nvic::disable(H::irq());
    Nvic::disable(C::irq());
    trace_isrs = false;
    const uint32_t he = host_isr_entries;
    const uint32_t pe = peer_isr_entries;
    const bool done = host_done;
    const uint8_t st = Host::status();
    const uint32_t hf = H::flags();
    const uint32_t cf = C::flags();
    console_drain();
    print(serial, "  sync ", sync ? 1 : 0, " done ", done ? 1 : 0, " status ", st,
          "  host ISR entries ", he, ", client ISR entries ", pe, crlf);
    print(serial, "  host ISR now ", hex(hf), " CR1 ", hex(H::regs().CR1),
          " CR2 ", hex(H::cr2()), " | client ISR now ", hex(cf), " CR1 ",
          hex(C::regs().CR1), crlf);
    for (uint8_t i = 0; i < host_trace_n; ++i) {
        print(serial, "  host[", i, "] ", hex(host_trace[i]), crlf);
    }
    for (uint8_t i = 0; i < peer_trace_n; ++i) {
        print(serial, "  peer[", i, "] ", hex(peer_trace[i]), crlf);
    }
    H::clear(I2cClear::all);
    C::clear(I2cClear::all);
    (void)Host::init(clock, I2cClock::pclk);
    peer_stop();
    print(serial, "  the ISR trace above is a diagnostic: no verdict", crlf);
}

// ===========================================================================
// n - the peer's command channel
// ===========================================================================

void tn_peer_link() {
    // The opening verdict is need_peer()'s own question, judged here by its
    // answer: ensure_link() pings the peer, and an absent one fails it.
    const bool alive = ensure_link();
    bench.verdict("the peer answers a ping over the two wires - and every command "
                  "is TWO TENURES of the engine under test, a write carrying the "
                  "frame and a read collecting the answer",
                  alive);
    if (!alive) {
        return;
    }

    twilink::Frame f;
    const bool got = query(Op::ident, f) && f.op == Op::ident_data &&
                     f.len == twilink::ident_size;
    if (got) {
        const auto id = twilink::get_ident(f.data);
        char label[9] = {};
        for (uint8_t i = 0; i < 8; ++i) {
            label[i] = id.label[i];
        }
        print(serial, "  peer: label '", label, "' xtal=", id.xtal, " sanity=",
              hex(id.sanity), " fw=", hex(id.version), crlf);
        bench.verdict("ident comes back and it IS twi_peer (the sanity byte), from a "
                      "SECOND BOARD speaking the same wire format",
                      id.sanity == twilink::ident_sanity);
    } else {
        bench.verdict("ident comes back", false);
    }

    uint8_t good = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (command(Op::ping)) {
            ++good;
        }
    }
    print(serial, "  ", good, " of 10 command round trips answered at ",
          Host::scl_hz(link_speed) / 1000u, " kHz", crlf);
    bench.verdict("the channel is steady over ten command round trips", good == 10u);
}

// ===========================================================================
// o - the tenure shapes against the peer
// ===========================================================================

void to_peer_shapes() {
    if (!need_peer()) {
        return;
    }

    // THE SERVE ENDS ON ITS DEADLINE, never by count: a count reached in
    // the middle of the combined tenure would cut it in half and leave
    // the repeated START talking to a command-mode client.
    twilink::Params a{};
    a.count = 64;
    a.ms = 250;
    a.addr = twilink::dut_addr;
    a.seed = 0x30;
    a.pattern = twilink::pattern_counting;
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve command", false);
        return;
    }

    uint8_t w[8];
    for (uint8_t i = 0; i < 8; ++i) {
        w[i] = static_cast<uint8_t>(0x11u * (i + 1u));
    }
    const uint8_t ws = host_tenure(twilink::dut_addr, w, 8, nullptr, 0, link_speed);
    for (uint8_t i = 0; i < 8; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t rs = host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 8, link_speed);
    uint8_t rmism = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        if (rx_buf[i] != twilink::pattern_value(a.pattern, a.seed, i)) {
            ++rmism;
        }
    }
    for (uint8_t i = 0; i < 4; ++i) {
        tx_buf[i] = static_cast<uint8_t>(0xA0u + i);
        rx_buf[i] = 0xEE;
    }
    const uint8_t cs = host_tenure(twilink::dut_addr, tx_buf, 4, rx_buf, 4, link_speed);
    settle_ms(300);   // past the serve's own deadline: the peer is back
    twilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  write=", ws, " read=", rs, " mism=", rmism, " combined=", cs,
          "; peer: count=", r.count, " (received ", r.aux0, ", served ", r.aux1,
          ") addr_hits=", r.addr_hits, " first=", hex(r.first), " sum=", hex(r.sum),
          crlf);
    bench.verdict("a write tenure, a read tenure and the combined write-then-read "
                  "all complete i2c_ok against a SECOND CHIP",
                  ws == i2c_ok && rs == i2c_ok && cs == i2c_ok);
    bench.verdict("the bytes read back are the peer's own pattern, byte-exact",
                  rmism == 0u);
    bench.verdict("the peer accounts every byte of the three tenures (8 written, 8 "
                  "served, then 4 and 4)",
                  rep && r.count == 24u);
    bench.verdict("and the COMBINED tenure hit the client's address machinery TWICE "
                  "- the repeated START, counted from the far end",
                  rep && r.addr_hits == 4u);

    // The addresses no Request may name (util/i2c_bus.hpp): 0x7A and the
    // general call refused by start() itself, against a peer at dut_addr
    // ANSWERING THE GENERAL CALL - so a byte of either on the wire would
    // be an address match on its count; the write to dut_addr after them
    // is its one match and its two bytes.
    twilink::Params g{};
    g.count = 64;
    g.ms = 250;
    g.addr = twilink::dut_addr;
    g.flags = twilink::flag_general_call;
    if (peer_act(Op::serve, g)) {
        uint8_t gw[2] = {0x5A, 0xA5};
        Host::Request bad{};
        bad.addr = 0x7A;
        bad.tx = lend<Lease::reply>(static_cast<const uint8_t*>(gw));
        bad.tx_len = 2;
        bad.speed = link_speed;
        const bool sync_prefix = Host::start(bad);
        const uint8_t st_prefix = Host::status();
        bad.addr = 0x00;
        const bool sync_call = Host::start(bad);
        const uint8_t st_call = Host::status();
        const uint8_t own = host_tenure(twilink::dut_addr, gw, 2, nullptr, 0, link_speed);
        settle_ms(300);
        twilink::Report gr{};
        const bool grep = peer_report(gr);
        print(serial, "  0x7A: ", sync_prefix ? "refused at start()" : "STARTED", " ", st_prefix, "; 0x00: ",
              sync_call ? "refused at start()" : "STARTED", " ", st_call, "; then dut_addr ", own,
              "; peer: addr_hits=", gr.addr_hits, " count=", gr.count, " last_addr=", hex(gr.last_addr), crlf);
        bench.verdict("0x7A and the general call answered i2c_rejected inside start(), nothing on the wire: "
                      "the peer, answering the general call, took only the write to its own address",
                      sync_prefix && st_prefix == i2c_rejected && sync_call && st_call == i2c_rejected &&
                          own == i2c_ok && grep && gr.addr_hits == 1u && gr.count == 2u &&
                          gr.last_addr == twilink::dut_addr);
    } else {
        bench.verdict("the peer accepted the general-call serve", false);
    }
}

// ===========================================================================
// p - the vocabulary on the wire, and commanded stretching
// ===========================================================================

void tp_peer_vocabulary() {
    if (!need_peer()) {
        return;
    }

    // A DEAF peer: its client parked where nobody calls. Both a write and
    // the EMPTY request an address scanner sends meet nobody-home.
    twilink::Params a{};
    a.count = 8;
    a.ms = 600;
    a.addr = twilink::dut_addr;
    a.flags = twilink::flag_deaf;
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the deaf serve", false);
        return;
    }
    uint8_t w[4] = {1, 2, 3, 4};
    const uint8_t deaf = host_tenure(twilink::dut_addr, w, 4, nullptr, 0, link_speed);
    const uint8_t probe = host_tenure(twilink::dut_addr, nullptr, 0, nullptr, 0,
                                      link_speed);
    settle_ms(700);

    // A data NACK at a commanded byte.
    twilink::Params n{};
    n.count = 16;
    n.ms = 600;
    n.addr = twilink::dut_addr;
    n.nack_at = 3;
    if (!peer_act(Op::serve, n)) {
        bench.verdict("the peer accepted the nack-at serve", false);
        return;
    }
    uint8_t w6[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
    const uint8_t nack = host_tenure(twilink::dut_addr, w6, 6, nullptr, 0, link_speed);
    settle_ms(700);
    twilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  deaf=", deaf, " probe=", probe, " data-nack=", nack,
          " peer flags=", hex(r.flags), " count=", r.count, crlf);
    bench.verdict("an address nobody answers reports i2c_nack_addr - the scanner's "
                  "probe result, on a board that is otherwise alive",
                  deaf == i2c_nack_addr && probe == i2c_nack_addr);
    bench.verdict("a commanded NACK on the 3rd data byte reports i2c_nack_data - the "
                  "wire-level vocabulary is REAL statuses across two architectures",
                  nack == i2c_nack_data && rep &&
                      (r.flags & twilink::report_nacked) != 0u);

    // Commanded clock stretching, priced against an unstretched baseline.
    twilink::Params base{};
    base.count = 128;   // never reached; the deadline ends the serve
    base.ms = 700;
    base.addr = twilink::dut_addr;
    if (!peer_act(Op::serve, base)) {
        bench.verdict("the peer accepted the baseline serve", false);
        return;
    }
    uint8_t w8[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint32_t t0 = Ruler::now();
    bool base_ok = true;
    for (uint8_t k = 0; k < 8; ++k) {
        if (host_tenure(twilink::dut_addr, w8, 8, nullptr, 0, link_speed) != i2c_ok) {
            base_ok = false;
        }
    }
    const uint32_t base_us = cycles_us(Ruler::now() - t0);
    settle_ms(750);

    // THE MODEL, from the F446 peer's run_serve: a hold at the address match
    // and one before each byte it takes, and its block is one byte deep -
    // each later byte shifts in during the hold before the previous one's
    // take - so the tenure of a write of 8 holds the address's hold and
    // seven takes' (the eighth falls after the STOP), eight in all, plus
    // the two byte times no hold overlaps (the address and the first byte,
    // 9 SCL periods each at 100 kHz). The holds are the peer's spin on its
    // cycle counter, at least hold_us on the crystal behind its PLL (the
    // ST-LINK's MCO, UM1724 7.9.1; even 1000 ppm moves 16 ms by 16 us).
    // THIS end's stopwatch runs on the HSI16: DS13560 table 44's 15.88 MHz
    // at 30 C and its drift row's -1 % over 0..85 C, -1.75 % in all, read
    // 16180 us as 15896 at the lowest (the microseconds truncated), and the
    // bound's own division is rounded one lower.
    constexpr uint32_t stretch_hold_us = 2000;
    constexpr uint32_t stretch_holds = 8;
    constexpr uint32_t stretch_wire_us = 2u * 9u * 10u;   // two bytes at 100 kHz
    constexpr uint32_t stretch_floor_us = static_cast<uint32_t>(
        (static_cast<uint64_t>(stretch_holds * stretch_hold_us + stretch_wire_us) * 9825u) /
            10000u -
        1u);
    static_assert(link_speed == I2cSpeed::standard_100k,
                  "the stretch floor counts 10 us SCL periods");
    twilink::Params st{};
    st.count = 64;
    st.ms = 150;
    st.addr = twilink::dut_addr;
    st.hold_us = stretch_hold_us;
    if (!peer_act(Op::serve, st)) {
        bench.verdict("the peer accepted the stretched serve", false);
        return;
    }
    t0 = Ruler::now();
    const uint8_t s2 = host_tenure(twilink::dut_addr, w8, 8, nullptr, 0, link_speed);
    const uint32_t stretched_us = cycles_us(Ruler::now() - t0);
    settle_ms(300);
    twilink::Report sr{};
    const bool srep = peer_report(sr);
    print(serial, "  8 x 8 bytes unstretched: ", base_us,
          " us; ONE 8-byte tenure at 2 ms per byte: ", stretched_us,
          " us, against at least ", stretch_floor_us, " (eight holds and two bytes, "
          "the HSI16 at its slowest); peer count=", sr.count, crlf);
    bench.verdict("the unstretched baseline tenures all complete i2c_ok", base_ok);
    bench.verdict("a client stretching every data byte by 2 ms stretches the WALL "
                  "TIME the model predicts (eight holds and two byte times, the "
                  "stopwatch's HSI16 at its slowest) and the tenure still completes "
                  "i2c_ok - stretching is flow control and the controller simply waits",
                  s2 == i2c_ok && stretched_us >= stretch_floor_us && srep && sr.count >= 8u);
}

// ===========================================================================
// q - the three speeds against a second chip
// ===========================================================================

void tq_peer_speeds() {
    if (!need_peer()) {
        return;
    }

    struct Rung {
        I2cSpeed speed;
        const char* name;
    };
    const Rung rungs[] = {{I2cSpeed::standard_100k, "100k"},
                          {I2cSpeed::fast_400k, "400k"},
                          {I2cSpeed::fast_plus_1m, "1M  "}};
    uint8_t exact = 0;
    bool sm_fm_ok = true;
    uint32_t pm[3] = {};
    for (uint8_t i = 0; i < 3u; ++i) {
        twilink::Params a{};
        a.count = 64;
        a.ms = 250;
        a.addr = twilink::dut_addr;
        a.seed = static_cast<uint8_t>(0x50u + i * 0x10u);
        a.pattern = twilink::pattern_counting;
        if (!peer_act(Op::serve, a)) {
            bench.verdict("the peer accepted the serve for this rung", false);
            return;
        }
        for (uint8_t k = 0; k < 8; ++k) {
            tx_buf[k] = static_cast<uint8_t>(0xC0u + k);
            rx_buf[k] = 0xEE;
        }
        const bool legal = Host::speed_ok(rungs[i].speed);
        // The previous rung's line is still going out: the write's stopwatch
        // starts with the console quiet, or its transmit interrupt is inside
        // the reading the rate verdict judges.
        if (!console_quiet()) {
            return;
        }
        const uint32_t w0 = Ruler::now();
        const uint8_t ws = legal ? host_tenure(twilink::dut_addr, tx_buf, 8, nullptr, 0,
                                               rungs[i].speed)
                                 : 0xEEu;
        // 9 x 9 SCL periods, the stopwatch around the whole write.
        pm[i] = legal ? scl_permille(cycles_ns(Ruler::now() - w0) / 81u,
                                     i2c_speed_hz(rungs[i].speed))
                      : 0u;
        const uint8_t rs = legal ? host_tenure(twilink::dut_addr, nullptr, 0, rx_buf, 8,
                                               rungs[i].speed)
                                 : 0xEEu;
        uint8_t mism = 0;
        for (uint8_t k = 0; k < 8; ++k) {
            if (rx_buf[k] != twilink::pattern_value(a.pattern, a.seed, k)) {
                ++mism;
            }
        }
        settle_ms(300);
        const bool ok = legal && ws == i2c_ok && rs == i2c_ok && mism == 0u;
        print(serial, "  ", rungs[i].name, ": SCL ", Host::scl_hz(rungs[i].speed) / 1000u,
              " kHz, write=", ws, " read=", rs, " mism=", mism, " -> ",
              ok ? "byte-exact both ways" : "NOT exact", "; the write's SCL in ", pm[i],
              " thousandths of the rate asked", crlf);
        if (ok) {
            ++exact;
        } else if (i < 2u) {
            sm_fm_ok = false;
        }
    }
    print(serial, "  ", exact, " of 3 speeds byte-exact against the peer", crlf);
    bench.verdict("Standard mode and Fast mode both carry a write and a read "
                  "byte-exact between TWO SEPARATE CHIPS",
                  sm_fm_ok);
    // Never above: scl_ok()'s arithmetic. How far below is the F446 target's
    // stretch (its polled serve, one byte late) and the wire's rise on top of
    // the request's own cost - nothing a document bounds, so the floor is the
    // design's quarter (scl_ok()); the thousandths are printed above.
    bench.verdict("... at the rates asked: the write's SCL never above 100 or 400 kHz "
                  "and within a quarter below them",
                  sm_fm_ok && scl_ok(pm[0]) && scl_ok(pm[1]));
    // FAST-MODE-PLUS IS THE RUNG THAT IS ABOUT THE WIRE AND THE FAR
    // END, not this controller: what a megahertz bus does here is a
    // property of these two jumpers, their 2.2 kOhm pull-ups and the
    // peer's own input path (a SAM C21 target, for one, has no input
    // filter at all). The number is printed and
    // the verdict claims only that the register accepted the rate.
    bench.verdict("...and Fm+ is REACHABLE at this kernel clock (whether the wire "
                  "carries it is the print above, not this verdict)",
                  Host::speed_ok(I2cSpeed::fast_plus_1m));
    bench.verdict("the command channel survives the ladder", command(Op::ping));
}

// ===========================================================================
// r - THE KERNEL against the peer
// ===========================================================================

void tr_peer_kernel() {
    if (!need_peer()) {
        return;
    }

    // The peer serves for the whole letter's first phase; its deadline is
    // long enough for the queued tenures and the vote round.
    twilink::Params a{};
    a.count = 512;   // never reached: the deadline is the exit
    a.ms = 900;
    a.addr = twilink::dut_addr;
    a.seed = 0x70;
    a.pattern = twilink::pattern_counting;
    const bool served = peer_act(Op::serve, a);
    bench.verdict("the peer accepted the serve the kernel letter drives", served);
    if (!served) {
        return;
    }

    kl::BusKernel::init_all();
    bus_ao_live = true;
    host_swallow_completion = false;

    fill_pattern(kl::out_a, 4, 0x01);
    fill_pattern(kl::out_b, 4, 0x02);
    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::I2cArb>(
            kl::request((i == 2u) ? nobody_addr : twilink::dut_addr, kl::out_a));
    }
    kl::pump_until(4, 400);
    print(serial, "  four queued tenures: replies ", kl::Probe::n, " [",
          kl::Probe::replies[0], " ", kl::Probe::replies[1], " ",
          kl::Probe::replies[2], " ", kl::Probe::replies[3], "]", crlf);
    bench.verdict("four tenures through I2cBus against a SECOND CHIP, four replies "
                  "- util/i2c_bus.hpp and util/bus_master.hpp with not one line "
                  "changed for the pairing",
                  kl::Probe::n == 4u);
    bench.verdict("... in order, with the NACK from an address nobody answers "
                  "delivered IN ITS PLACE as a reply",
                  kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_ok &&
                      kl::Probe::replies[2] == i2c_nack_addr &&
                      kl::Probe::replies[3] == i2c_ok);

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_b));
    }
    kl::pump_until(6, 400);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n,
          ", rejected ", kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately",
                  kl::Probe::rejected != 0u);
    bench.verdict("... and every request is still answered exactly once",
                  kl::Probe::n == 6u);

    kl::drain(100);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep",
                  kl::Probe::votes == 1u && kl::Probe::last_vote);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    post<kl::I2cArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    kl::drain(150);
    print(serial, "  the vote from a BUSY bus: ", kl::Probe::votes, " vote(s), last ",
          kl::Probe::last_vote ? "yes" : "no", crlf);
    bench.verdict("a BUSY bus votes against it",
                  kl::Probe::votes == 1u && !kl::Probe::last_vote);

    // ---- THE TIMED BUS, with the wedge held by the OTHER BOARD ----
    // The peer holds SDA down from its own PORT for longer than the
    // arbiter's limit. No silicon time-out on this controller watches a
    // wire a client wedged (letter i measures whose hold each of the
    // three really polices), so the ARBITER is what notices.
    bus_ao_live = false;
    settle_ms(400);   // the serve's deadline has to expire first
    twilink::Params h{};
    h.ms = 300;
    h.aux8 = 0;        // only the deadline releases the line
    h.aux16 = 0;
    const bool armed = peer_act(Op::hold_sda, h);
    bus_ao_live = true;
    bench.verdict("the peer accepted the hold_sda command", armed);
    if (!armed) {
        peer_stop();
        return;
    }
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    const uint32_t t0 = Ticker::ticks();
    kl::pump_until(1, 400);
    const uint32_t took = Ticker::ticks() - t0;
    const bool still_low = !SdaPin::read();
    print(serial, "  the wedged tenure answered ", kl::Probe::n, " with status ",
          kl::Probe::replies[0], " after ", took, " ms; SDA still low at the reply: ",
          still_low ? "yes" : "no", crlf);
    // WHICH OF THE TWO ANSWERS COMES IS A PROPERTY OF THE WEDGE'S
    // TIMING, and both are a report: a foreign chip that takes SDA while
    // this controller is already driving a START makes the silicon read
    // back a level it did not drive, which is i2c_arb_lost AT ONCE; a
    // line already low when the START is issued parks instead, and then
    // nothing but the arbiter's own limit answers (the self-link's
    // letter l measures that half).
    print(serial, "  the answer is ",
          kl::Probe::replies[0] == i2c_arb_lost
              ? "i2c_arb_lost - the ENGINE's own wire code, read back from a level "
                "it did not drive"
              : (kl::Probe::replies[0] == i2c_timeout
                     ? "i2c_timeout - the ARBITER's, the silicon having seen a park "
                       "and no error at all"
                     : "an engine code"),
          crlf);
    bench.verdict("a tenure into a wire a FOREIGN CHIP holds down is answered IN ITS "
                  "PLACE - never silence and never i2c_ok",
                  kl::Probe::n == 1u && kl::Probe::replies[0] != i2c_ok);
    bench.verdict("... at the arbiter's own limit or sooner (two ticks over it at most), "
                  "never at the wedge's length",
                  took <= kl::wedge_answer_ticks);

    // And the same bus AO carries the next tenure once the peer lets go. The
    // serve goes as ONE command attempt and no init(): link_ready() and
    // command()'s retries both bring the host up again, which would judge
    // init() and not the recover() the arbiter's timeout ran.
    bus_ao_live = false;
    settle_ms(400);
    twilink::Params again{};
    again.count = 64;
    again.ms = 250;
    again.addr = twilink::dut_addr;
    again.seed = 0x90;
    uint8_t again_p[twilink::params_size];
    twilink::put_params(again_p, again);
    const bool re = command_once(Op::serve, again_p, twilink::params_size);
    if (re) {
        settle_ms(twilink::arm_ms);   // the peer's arming, as command() leaves it
    }
    bus_ao_live = true;
    kl::drain(20);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a));
    kl::pump_until(1, 300);
    print(serial, "  after the release: replies ", kl::Probe::n, " status ",
          kl::Probe::replies[0], crlf);
    bench.verdict("THE SAME BUS AO carries the next tenure to i2c_ok after "
                  "recover()",
                  re && kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_ok);
    bus_ao_live = false;
    settle_ms(300);
    peer_stop();
}

// ===========================================================================
// s - TWO CONTROLLERS, ONE STOP: the held STARTs against a second chip
// ===========================================================================
//
// TWO CONTROLLERS' STARTS, RELEASED BY ONE EDGE. Neither controller can be
// told when the other's START leaves, so a THIRD node frees the bus for
// both at once. On this desk that node is the board's own I2C2, whose pads
// tap the same two nodes (the self-link): it opens a tenure nobody answers
// at about 2 kHz, so the bus is BUSY for some 4.7 ms; inside that window
// this board's I2C1 sets START, which 32.4.9 holds behind the busy bus,
// and the peer - told by `arb` to wait for BUSY and a lead-in - sets its
// own, which RM0390 24.3.3 holds the same way once its engine's bounded
// wait for a free bus (100 us) has run out. I2C2's STOP releases both.
//
// WHAT THE TWO SILICONS DO WITH IT IS SERIALIZE, measured: neither START
// is committed early enough to meet the other's. A sampled trace of both
// lines puts this controller's START 5.75, 2.0 and 0.75 us after the STOP
// at 100 kHz, 400 kHz and 1 MHz, and the peer's within about 2 us; in
// twelve rendezvous over the three rates and both rendezvous rates, the
// second controller saw the first's START every time, waited behind it,
// and went second - no address was ever arbitrated. So this letter judges
// what holds: two held STARTs on one bus, released together, both run
// their tenure whole and neither is lost or corrupted. A live arbitration
// would want one START committed inside the other's few hundred
// nanoseconds of detection, which nothing here can aim.

/// The rendezvous node's slow timing: PRESC 15 on the 16 MHz kernel is a
/// 1 us tick, SCLL and SCLH 256 ticks each - a 512 us SCL period, so a
/// START, nine clocks and a STOP keep the bus busy for about 4.7 ms.
constexpr I2cTiming rendezvous_timing{.presc = 15, .scll = 0xFF, .sclh = 0xFF,
                                      .sdadel = 2, .scldel = 4};

/// Put I2C2 on its pads with the slow timing, enabled and silent - BEFORE
/// the peer is armed, so the pad claim's edges are seen by a peer still in
/// command mode. False when I2C2 refuses the timing.
bool rendezvous_prepare() {
    uint8_t none[1] = {0};
    (void)peer_arm(none, 0, I2cSpeed::standard_100k);
    peer_live = false;
    Nvic::disable(C::irq());
    Peer::interrupt(I2cInterrupt::all, false);
    if (!C::disable() || !C::timing(rendezvous_timing)) {
        return false;
    }
    C::enable();
    C::clear(I2cClear::all);
    return true;
}

/// Open the tenure nobody answers on I2C2, as a CONTROLLER: the bus is
/// busy from its START (some 256 us after this call - a SCLL's worth)
/// until its own STOP, AUTOEND after the address NACK.
bool rendezvous_fire() {
    return C::transfer_now(twilink::deaf_addr, false, 0, true);
}

/// Wait for the rendezvous tenure's STOP (bounded: 20 ms, four times the
/// 4.7 ms it takes) and give I2C2 back. True when the STOP was seen.
bool rendezvous_close() {
    const uint32_t t0 = Ticker::millis();
    bool stopped = false;
    while (Ticker::millis() - t0 < 20u) {
        if (C::flag(I2cFlag::stop)) {
            stopped = true;
            break;
        }
    }
    C::clear(I2cClear::all);
    peer_stop();
    return stopped;
}

void ts_two_controllers() {
    if (!need_peer()) {
        return;
    }
    if (!self_link) {
        bench.skip("the rendezvous node is this board's I2C2 on the self-link's pads, and the "
                   "probe says they are not on the desk");
        return;
    }
    // THE COMMANDS GO AT THE RATE THIS CONTROLLER CONTENDS AT. TIMINGR is
    // PE-gated, so a request at a new rate costs this block a PE cycle -
    // and a PE cycle CLEARS BUSY (32.9.7), so the START that follows goes
    // out INTO the rendezvous tenure instead of waiting behind it
    // (measured: I2C2 left mid-address with both lines low, the peer
    // reporting a lost arbitration against it). docs/stm32g0/i2c.md
    // carries it as a finding.
    constexpr I2cSpeed contend_speed = I2cSpeed::fast_400k;
    struct CommandRate {
        CommandRate() { command_speed = contend_speed; }
        ~CommandRate() { command_speed = link_speed; }
    } const rate_guard;
    // 0x6B and 0x11 against the peer's 0x2C: an arbitration, had there
    // been one, would have gone against this board in the first round and
    // for it in the second.
    const uint8_t aims[2] = {twilink::command_addr, twilink::low_addr};
    constexpr uint8_t reps = 5;
    uint8_t whole = 0;
    uint8_t lost = 0;
    uint8_t rendezvous_ok = 0;
    for (uint8_t i = 0; i < 2u; ++i) {
        for (uint8_t k = 0; k < reps; ++k) {
            link_ready();
            if (!rendezvous_prepare()) {
                bench.verdict("I2C2 took the rendezvous timing", false);
                peer_stop();
                return;
            }
            twilink::Params a{};
            a.ms = 600;
            a.addr = twilink::dut_addr;
            a.target = twilink::dut_addr;
            a.count = 4;
            a.seed = 0x40;
            a.aux16 = 300;   // lead-in after BUSY, microseconds
            if (!peer_act(Op::arb, a)) {
                bench.verdict("the peer accepted the arbitration command", false);
                peer_stop();
                return;
            }
            // The peer re-inits its instance as a controller after the
            // ack, with a reset of the block among it: a START it does
            // not see is a bus it never knows to be busy.
            settle_ms(20);
            if (!rendezvous_fire()) {
                bench.verdict("I2C2 opened the rendezvous tenure", false);
                peer_stop();
                return;
            }
            // Past the peer's lead-in and its engine's 100 us wait, so
            // both STARTs are held in hardware when the STOP comes.
            spin_us<Ruler>(1500);
            for (uint8_t b = 0; b < 5u; ++b) {
                tx_buf[b] = static_cast<uint8_t>(0xD0u + b);
            }
            const uint8_t st = host_tenure(aims[i], tx_buf, 5, nullptr, 0, contend_speed);
            if (rendezvous_close()) {
                ++rendezvous_ok;
            }
            settle_ms(650);   // past the peer's deadline: back in command mode
            twilink::Report r{};
            const bool rep = peer_report(r);
            const bool peer_lost = rep && (r.flags & twilink::report_arblost) != 0u;
            if (st == i2c_arb_lost || peer_lost) {
                ++lost;
            }
            const bool ok = st == i2c_nack_addr && rep &&
                            (r.flags & twilink::report_host_ran) != 0u &&
                            r.mstatus == i2c_nack_addr;
            if (ok) {
                ++whole;
            }
            if (k == 0u || !ok) {
                print(serial, "  this board -> ", hex(aims[i]), ", the peer -> 0x2C: this "
                      "board status=", st, "; peer flags=", hex(r.flags), " mstatus=",
                      r.mstatus, crlf);
            }
        }
    }
    print(serial, "  ", whole, " of ", 2u * reps, " rendezvous ran BOTH tenures whole; ",
          lost, " arbitration(s) lost; ", rendezvous_ok, " rendezvous tenures closed by "
          "their STOP", crlf);
    bench.verdict("the third node's tenure ends on its own STOP every time - neither "
                  "held START went out into it",
                  rendezvous_ok == 2u * reps);
    bench.verdict("two held STARTs released by one STOP SERIALIZE: both controllers run "
                  "their tenure whole to the address nobody answers (i2c_nack_addr at "
                  "both ends), every time, at either address order",
                  whole == 2u * reps && lost == 0u);
    bench.verdict("the command channel is back after the rendezvous", command(Op::ping));
}

// ===========================================================================
// t - THE WAKE FROM STOP, the address coming from another chip
// ===========================================================================
//
// Letter j arms everything around the wake and declines the Stop itself:
// both ends of the self-link are on this die, and a Stop that silences the
// client silences the controller that would wake it. The PEER removes the
// obstacle - `arb` makes it a controller that writes to an address after a
// lead-in of its choosing - and I2C2, whose pads tap the same bus, is the
// client it wakes: HSI16 kernel, DNF 0, stretching on (32.4.16's three
// conditions), its EXTI line 22 direct. The rendezvous is this board's
// own: I2C1 opens a tenure nobody answers, the peer sees BUSY and waits
// its lead-in (5 ms), and this board is in Stop by then.
//
// The wake source is proven in the shallower mode first - the same
// arrangement in Sleep - and the Stop carries a BACKSTOP that does not
// depend on the bus: the RTC's wake-up timer on the LSE at one second, so
// a wake that never comes costs a second and a failed verdict, not a
// board left asleep. The RTC is also the wall the Stop is timed with.

volatile uint32_t rtc_backstops = 0;
bool rtc_wall_ready = false;

/// Claim the RTC domain on the LSE the way every sleeping suite here does:
/// never reset (another suite's backup registers live there), claimed only
/// when it is already on LSE or has no clock, the calendar set only when it
/// never was. True when the RTC runs on the crystal.
bool rtc_up() {
    if (rtc_wall_ready) {
        return true;
    }
    Pwr::bus_clock(true);
    Pwr::rtc_domain_unlock(true);
    RtcDomain::apb_clock(true);
    const RtcClockSource now = RtcDomain::selected();
    if (now != RtcClockSource::lse && now != RtcClockSource::none) {
        return false;
    }
    RtcDomain::lse_enable(true);
    if (!RtcDomain::lse_wait_ready(4'000'000UL)) {
        return false;
    }
    bool ok = RtcDomain::open(RtcClockSource::lse);
    Rtc::bypass_shadow(true);
    if (ok && !Rtc::calendar_set()) {
        ok = Rtc::init(RtcPrescalers{},
                       RtcDateTime{.hour = 0, .minute = 0, .second = 0, .day = 1,
                                   .month = 1, .year = 24, .weekday = 1});
    }
    if (ok) {
        Nvic::enable(Rtc::irq());
    }
    rtc_wall_ready = ok;
    return ok;
}

/// The RTC as a wall clock: subsecond ticks within the minute.
uint32_t rtc_wall() {
    RtcReading r{};
    if (!Rtc::read(r)) {
        return 0xFFFFFFFFu;
    }
    const uint32_t per_second = static_cast<uint32_t>(Rtc::prescalers().sync) + 1u;
    return static_cast<uint32_t>(r.time.second) * per_second +
           (per_second - 1u - r.subsecond);
}
uint32_t rtc_delta(uint32_t from, uint32_t to) {
    const uint32_t mod = 60u * (static_cast<uint32_t>(Rtc::prescalers().sync) + 1u);
    return (to >= from) ? (to - from) : (mod - from + to);
}
uint32_t rtc_ms(uint32_t ticks) {
    const uint32_t hz = 32768u / (static_cast<uint32_t>(Rtc::prescalers().async) + 1u);
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1000ULL) / hz);
}

struct WakeLeg {
    bool armed = false;      ///< the peer took the command and the client the wake
    uint32_t slept_ms = 0;   ///< the RTC's reading of the sleep
    uint32_t pre_ms = 0;     ///< from the rendezvous to the sleep, on the tick
    bool by_backstop = false;
    uint16_t got = 0;        ///< bytes the client took
    bool exact = false;      ///< and they are the peer's pattern
    uint8_t peer_status = 0xEE;
};

/// One wake: the peer told to write `n` bytes to the client `lead_ms`
/// after BUSY, this board's own rendezvous tenure, then `mode`.
WakeLeg wake_leg(PwrMode mode, uint8_t n, uint16_t lead_ms, uint8_t seed) {
    WakeLeg w{};
    link_ready();
    uint8_t none[1] = {0};
    if (!peer_arm(none, 0, I2cSpeed::standard_100k) || !Peer::wake_from_stop(true)) {
        peer_stop();
        return w;
    }
    twilink::Params a{};
    a.ms = 900;
    a.addr = twilink::dut_addr;
    a.target = peer_addr;
    a.count = n;
    a.seed = seed;
    a.aux16 = static_cast<uint16_t>(lead_ms * 1000u);
    if (!peer_act(Op::arb, a)) {
        (void)Peer::wake_from_stop(false);
        peer_stop();
        return w;
    }
    w.armed = true;
    settle_ms(20);   // the peer's role switch, as in letter s
    peer_rx_n = 0;
    peer_addr_hits = 0;
    // The rendezvous: a tenure nobody answers, done and STOPped before the
    // sleep (a Stop entered with a tenure in flight hangs - letter j).
    console_drain();
    const uint32_t r0 = Ticker::millis();
    (void)host_tenure(twilink::deaf_addr, nullptr, 0, nullptr, 0, link_speed);
    (void)Rtc::set_wakeup(RtcWakeupClock::ck_spre, 1, true);
    Rtc::clear_flags(RtcFlag::wakeup);
    rtc_backstops = 0;
    const uint32_t w0 = rtc_wall();
    w.pre_ms = Ticker::millis() - r0;
    Ticker::pause();
    Pwr::enter(mode);
    Ticker::resume();
    (void)SysClock::restore();   // a Stop leaves SYSCLK on HSISYS
    w.slept_ms = rtc_ms(rtc_delta(w0, rtc_wall()));
    w.by_backstop = rtc_backstops != 0u;
    Rtc::clear_wakeup();
    // The tenure the address opened runs on the client's vector; wait it
    // out, bounded (four bytes at 100 kHz are under half a millisecond).
    const uint32_t t0 = Ticker::millis();
    while (peer_rx_n < n && Ticker::millis() - t0 < 50u) {
    }
    w.got = peer_rx_n;
    bool exact = w.got == n;
    for (uint8_t i = 0; exact && i < n; ++i) {
        exact = peer_rx[i] == twilink::pattern_value(twilink::pattern_counting, seed, i);
    }
    w.exact = exact;
    (void)Peer::wake_from_stop(false);
    settle_ms(950);   // past the peer's deadline
    peer_stop();
    twilink::Report r{};
    if (peer_report(r)) {
        w.peer_status = r.mstatus;
    }
    return w;
}

void tt_peer_wake() {
    if (!need_peer()) {
        return;
    }
    if (!self_link) {
        bench.skip("the client the peer wakes is this board's I2C2 on the self-link's pads, and "
                   "the probe says they are not on the desk");
        return;
    }
    if (!C::wakes_from_stop) {
        bench.skip("this part's I2C2 has no wake from Stop (table 165)");
        return;
    }
    if (!rtc_up()) {
        print(serial, "  the RTC domain is not on the LSE crystal and cannot be claimed: "
                      "no backstop and no wall, so the Stop is not attempted", crlf);
        bench.verdict("the RTC runs on the LSE (the Stop's backstop)", false);
        return;
    }
    // Fifty milliseconds of lead-in: long enough that a sleep which ended
    // at once (a pending interrupt, a Stop never entered) cannot be
    // mistaken for one the address ended, at the RTC's 3.9 ms unit.
    constexpr uint16_t lead_ms = 50;
    const uint32_t unit_us =
        (static_cast<uint32_t>(Rtc::prescalers().async) + 1u) * 1'000'000UL / 32768u;
    // THE BAND: the peer writes lead_ms after the rendezvous's BUSY and this
    // board went to sleep pre_ms after it, so the sleep is due
    // lead_ms - pre_ms. Two RTC readings each quantized by a unit put the
    // reading within one unit of that either way; a unit more on each side
    // for the tick's own millisecond on pre_ms and the peer's latency.
    const auto in_band = [&](const WakeLeg& w) {
        const uint32_t due = lead_ms > w.pre_ms ? lead_ms - w.pre_ms : 0u;
        const uint32_t slack = (2u * unit_us + 999u) / 1000u + 1u;
        return w.slept_ms + slack >= due && w.slept_ms <= due + slack;
    };

    // ---- Sleep first: the wake source proven in the shallower mode ----
    const WakeLeg s1 = wake_leg(PwrMode::sleep, 4, lead_ms, 0x60);
    print(serial, "  Sleep: armed=", s1.armed, " slept ", s1.slept_ms, " ms (due ",
          lead_ms - s1.pre_ms, "), backstop ", s1.by_backstop ? "FIRED" : "quiet", ", ",
          s1.got, " byte(s) taken, peer status ", s1.peer_status, crlf);
    const bool sleep_ok = s1.armed && !s1.by_backstop && s1.exact && in_band(s1) &&
                          s1.peer_status == i2c_ok;
    bench.verdict("in SLEEP, the peer's write to this board's client ends the sleep and "
                  "is taken whole (the wake source proven before the Stop)",
                  sleep_ok);
    if (!sleep_ok) {
        print(serial, "  the Stop is not attempted without the Sleep leg", crlf);
        return;
    }

    // ---- Stop 1 ----
    const WakeLeg s2 = wake_leg(PwrMode::stop1, 4, lead_ms, 0x70);
    print(serial, "  Stop 1: armed=", s2.armed, " slept ", s2.slept_ms, " ms (due ",
          lead_ms - s2.pre_ms, "; the RTC's unit is ", unit_us, " us), backstop ",
          s2.by_backstop ? "FIRED" : "quiet", ", ", s2.got, " byte(s) taken, peer status ",
          s2.peer_status, crlf);
    bench.verdict("in STOP 1 the client's own address, sent by ANOTHER CHIP, wakes the "
                  "core - not the one-second backstop",
                  s2.armed && !s2.by_backstop);
    bench.verdict("... and it slept until the peer's address came: the RTC's reading "
                  "of the Stop is the lead-in less the time to fall asleep, within the "
                  "band derived above",
                  in_band(s2));
    bench.verdict("and the four bytes that address opened are taken whole, the peer's "
                  "write completing i2c_ok - the client stretched the wake out of SCL",
                  s2.exact && s2.peer_status == i2c_ok);
    bench.verdict("the command channel is back after the Stop", command(Op::ping));
}

// ---------------------------------------------------------------------------
// the banner
// ---------------------------------------------------------------------------

// ===========================================================================
// C, D - THE RECEIVE CHANNEL AT THE STOPF ENTRY
// ===========================================================================
//
// Under `dma` a read's one edge is STOPF, and STOPF says the wire is over: RXNE rose
// some nine tenths of a period before the NACK's ninth clock and more than a period
// and a half before the STOP, and the receive channel still had the arbiter (10.4.4)
// and the bus matrix's round robin (10.4.1) to win. Letter C stages a channel that has
// not moved the last byte at that entry and judges what the host does with it; letter
// D asks whether a load makes one late by itself, and how near.

using LateChannel = DmaChannel<HostRx::controller, HostRx::channel>;

/// A delay of no bus access but the fetch: about five cycles a turn from the flash.
[[gnu::always_inline]] inline void late_burn(uint32_t turns) {
    while (turns != 0u) {
        __NOP();
        --turns;
    }
}

/// THE VECTOR'S WATCH, ahead of the host's body and behind it. Only a STOPF entry
/// counts: on a write-then-read the TC entry comes first.
struct LateWatch {
    volatile bool on = false;
    volatile int16_t raise_turns = -1;   ///< let the request up, this many burns ahead of the body
    volatile bool steal = false;         ///< read RXDR ahead of the body: a byte nobody will have
    volatile uint16_t race_words = 0;    ///< let the request up under a load block of this many beats
    volatile uint32_t stops = 0;         ///< STOPF entries watched
    volatile uint32_t running = 0;       ///< the channel enabled with a count left, at the first load
    volatile uint32_t rxne = 0;          ///< RXNE up at that first load
    const volatile uint8_t* cell = nullptr;
    volatile uint8_t seen = 0;           ///< the cell at the first load behind the body
    volatile uint32_t t0 = 0;            ///< SysTick's VAL just ahead of the body
    volatile uint32_t body_on_time = 0;  ///< the longest body, the channel done at the first load
    volatile uint32_t body_late = 0;     ///< the longest body, the channel running at it
    volatile bool was_late = false;
};
LateWatch late_watch;

template <uint8_t ch>
void late_block_set(uint16_t words);

[[gnu::always_inline]] inline void late_watch_before() {
    const uint32_t f = H::flags();
    if ((f & I2cFlag::stop) == 0u) {
        return;
    }
    late_watch.stops = late_watch.stops + 1u;
    late_watch.was_late = LateChannel::enabled() && LateChannel::count() != 0u;
    if (late_watch.was_late) {
        late_watch.running = late_watch.running + 1u;
    }
    if ((f & I2cFlag::rxne) != 0u) {
        late_watch.rxne = late_watch.rxne + 1u;
    }
    if (late_watch.race_words != 0u) {
        const uint16_t beats = late_watch.race_words;
        late_block_set<3>(static_cast<uint16_t>((beats + 2u) / 3u));
        late_block_set<4>(static_cast<uint16_t>((beats + 1u) / 3u));
        late_block_set<5>(static_cast<uint16_t>(beats / 3u));
        H::dma_receive(true);
    }
    if (late_watch.raise_turns >= 0) {
        H::dma_receive(true);
        late_burn(static_cast<uint32_t>(late_watch.raise_turns));
    }
    if (late_watch.steal) {
        (void)H::data();
    }
    late_watch.t0 = SysTick->VAL;
}

[[gnu::always_inline]] inline void late_watch_after() {
    const uint32_t now = SysTick->VAL;
    const uint32_t t0 = late_watch.t0;
    const uint32_t body = t0 >= now ? t0 - now : t0 + SysTick->LOAD + 1u - now;
    if (late_watch.was_late) {
        late_watch.body_late = body > late_watch.body_late ? body : late_watch.body_late;
    } else {
        late_watch.body_on_time = body > late_watch.body_on_time ? body : late_watch.body_on_time;
    }
    if (late_watch.cell != nullptr) {
        late_watch.seen = *late_watch.cell;
    }
}

/// The load's channels, DMA1's three the host leaves on every part: a word from one
/// cell into another, at very_high, above the host's two at high. A paced channel
/// rides a timer's update; a copy (MEM2MEM) runs flat out and 10.4.4 alternates it
/// with whoever else is asking. A block is re-armed by the thread's turn when its
/// count runs out, so a load has gaps a program's blocks would have too.
constexpr uint16_t late_block_words = 1000;
volatile uint32_t late_load_from = 0x5A5A5A5Au;
volatile uint32_t late_load_to[3];

struct LateLoad {
    const char* name;
    uint8_t paced;      ///< how many paced channels, from channel 3 up
    uint16_t period;    ///< the timers' period, in TIMPCLK cycles
    uint8_t copies;     ///< how many copies, on the channels above those
    uint8_t skew = 0;   ///< burns between one timer's start and the next: their phase
    DmaPriority copy_level = DmaPriority::very_high;
};
uint8_t late_load_paced = 0;
uint8_t late_load_copies = 0;

template <uint8_t ch>
void late_load_program(uint8_t request, bool copy, DmaPriority copy_level = DmaPriority::very_high) {
    (void)DmaMux::request(DmaChannel<1, ch>::mux_channel, request);
    DmaTransfer t{};
    t.peripheral = &late_load_from;
    t.memory = &late_load_to[ch - 3u];
    t.count = late_block_words;
    t.config.memory_to_memory = copy;
    t.config.memory_increment = false;
    t.config.peripheral_width = DmaWidth::word;
    t.config.memory_width = DmaWidth::word;
    t.config.priority = copy ? copy_level : DmaPriority::very_high;
    (void)DmaChannel<1, ch>::load(t);
}

template <typename T>
void late_timer_on(uint16_t period) {
    T::init();
    (void)T::configure({.prescaler = 0, .period = static_cast<uint32_t>(period - 1u)});
    T::interrupts(T::update_dma, true);
    T::enable(true);
}

template <typename T>
void late_timer_off() {
    T::interrupts(T::update_dma, false);
    T::enable(false);
    T::release();
}

void late_load_on(const LateLoad& l) {
    late_load_paced = l.paced;
    late_load_copies = l.copies;
    if (l.paced > 0u) {
        late_load_program<3>(Tim<3>::dma_update_request(), false);
        late_timer_on<Tim<3>>(l.period);
    }
    if (l.paced > 1u) {
        late_load_program<4>(Tim<1>::dma_update_request(), false);
        late_burn(l.skew);
        late_timer_on<Tim<1>>(l.period);
    }
    if (l.paced > 2u) {
        late_load_program<5>(Tim<2>::dma_update_request(), false);
        late_burn(l.skew);
        late_timer_on<Tim<2>>(l.period);
    }
    const uint8_t first = l.paced;
    if (l.copies > 0u && first == 0u) {
        late_load_program<3>(0, true, l.copy_level);
    }
    if ((l.copies > 0u && first == 1u) || (l.copies > 1u && first == 0u)) {
        late_load_program<4>(0, true, l.copy_level);
    }
    if ((l.copies > 0u && first == 2u) || (l.copies > 1u && first == 1u) ||
        (l.copies > 2u && first == 0u)) {
        late_load_program<5>(0, true, l.copy_level);
    }
}

template <uint8_t ch>
[[gnu::always_inline]] inline bool late_block_out() {
    return DmaChannel<1, ch>::count() == 0u;
}

template <uint8_t ch>
void late_block_set(uint16_t words) {
    (void)DmaChannel<1, ch>::enable(false);
    DmaChannel<1, ch>::clear(DmaFlag::all);
    (void)DmaChannel<1, ch>::set_count(words);
    (void)DmaChannel<1, ch>::enable(true);
}

/// The thread's turn: once every block of the load has run out, all of them again. The
/// gap that leaves is where a channel below them gets its turn, as under a program's
/// blocks; with no gap a byte stretched behind them would wait for ever.
void late_load_turn() {
    const uint8_t used = static_cast<uint8_t>(late_load_paced + late_load_copies);
    if (used == 0u || (used > 0u && !late_block_out<3>()) || (used > 1u && !late_block_out<4>()) ||
        (used > 2u && !late_block_out<5>())) {
        return;
    }
    late_block_set<3>(late_block_words);
    if (used > 1u) {
        late_block_set<4>(late_block_words);
    }
    if (used > 2u) {
        late_block_set<5>(late_block_words);
    }
}

void late_load_off() {
    if (late_load_paced > 0u) {
        late_timer_off<Tim<3>>();
    }
    if (late_load_paced > 1u) {
        late_timer_off<Tim<1>>();
    }
    if (late_load_paced > 2u) {
        late_timer_off<Tim<2>>();
    }
    DmaChannel<1, 3>::stop();
    DmaChannel<1, 4>::stop();
    DmaChannel<1, 5>::stop();
    (void)DmaMux::request(DmaChannel<1, 3>::mux_channel, 0);
    (void)DmaMux::request(DmaChannel<1, 4>::mux_channel, 0);
    (void)DmaMux::request(DmaChannel<1, 5>::mux_channel, 0);
    late_load_paced = 0;
    late_load_copies = 0;
}

/// THE BOUND ON ONE TENURE of this letter, in ticks of 1 ms: letter v's, its
/// arithmetic there, plus what a load adds. A byte the channel has not taken stretches
/// SCL (32.4.7) until the channel gets a turn, at worst the three channels' blocks of
/// 1000 beats at the 5 cycles a beat a load's own beat takes (docs/stm32g0/dma.md) -
/// 15000 cycles, 234 us at 64 MHz and 938 us at 16 MHz, 1 ms to be a wedge and not a slow
/// row, at every byte and the address. At 2 MHz, where the letter also runs, they are 7.5 ms
/// against 6 ms on a read of one, so the derivation stops at 16 MHz: that row's copies
/// alternate with lower channels (10.4.4) and do not reach this worst, and a late one would
/// be a false red, never a false green.
uint32_t late_limit_ms(uint16_t bytes, I2cSpeed s) {
    return refused_limit_ms(bytes, s) + bytes + 2u;
}

/// A completion waited for, the load turning; false after recover().
bool late_wait_done(uint32_t limit_ms) {
    const uint32_t t0 = Ticker::millis();
    while (!host_done && Ticker::millis() - t0 < limit_ms) {
        late_load_turn();
    }
    if (!host_done) {
        (void)DmaHost::recover();
        return false;
    }
    return true;
}

/// A byte of read `k`, each position different, so a stale or a moved byte shows.
constexpr uint8_t late_value(uint16_t k, uint16_t i) {
    return static_cast<uint8_t>(0x6Bu + 0x3Du * k + 0x1Bu * i);
}

constexpr uint8_t late_buf = 16;
uint8_t late_in[late_buf + 1u];   // the last, a sentinel past every read here
uint8_t late_out[8];
uint16_t late_serial = 0;

void late_serve(uint16_t k, uint8_t n) {
    for (uint8_t i = 0; i < n; ++i) {
        peer_answers[i] = late_value(k, i);
    }
}

/// How the last byte's request is staged.
enum class LateStage : uint8_t {
    none,       ///< nothing touched: the channel as the load leaves it
    starve,     ///< the request taken away for the last byte, up to the end
    raise,      ///< the same, let up in the STOPF entry `turns` of burns ahead of the host
    steal,      ///< the same, RXDR read in the entry ahead of the host: the byte is nobody's
    race,       ///< the same, let up in the entry under a load block `turns` beats long
};

struct LateRow {
    uint32_t reads = 0;
    uint32_t exact = 0;      ///< i2c_ok, every cell its byte, the sentinel whole, in the vector
    uint32_t first_wrong = 0;   ///< the first cell not its byte (the suspicion's mark)
    uint32_t last_wrong = 0;    ///< the last cell not its byte
    uint32_t status_wrong = 0;  ///< not the status the stage owes
    uint32_t lost = 0;       ///< no completion within the bound
    uint32_t running = 0;    ///< the channel enabled with a count left at the vector's first load
    uint32_t rxne = 0;       ///< RXNE up at it
    uint32_t left_on = 0;    ///< the channel still enabled with a count left after the completion
    uint32_t stray = 0;      ///< a sentinel written, or a cell moved after the completion
    uint32_t next_read = 0;     ///< the read of eight that follows, whole
    uint32_t next_write = 0;    ///< the write of eight that follows, whole
    uint32_t next_refused = 0;  ///< the refused write behind them: the status, then a write whole
    uint32_t nexts = 0;
};

void late_row_print(const char* name, const LateRow& r) {
    print(serial, "    ", name, ": ", r.reads, " reads, exact ", r.exact, ", first cell wrong ",
          r.first_wrong, ", last wrong ", r.last_wrong, ", status wrong ", r.status_wrong,
          ", unanswered ", r.lost, "; running at the entry ", r.running, ", RXNE up ", r.rxne,
          "; left running ", r.left_on, ", moved after ", r.stray, crlf);
    if (r.nexts != 0u) {
        print(serial, "      behind them, of ", r.nexts, ": read of eight whole ", r.next_read,
              ", write of eight whole ", r.next_write, ", refused write and the next whole ",
              r.next_refused, crlf);
    }
}

/// One request on the engined host, waited for: true when it answered.
bool late_request(const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len, I2cSpeed s,
                  uint8_t addr = peer_addr) {
    DmaHost::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = s;
    r.engine = I2cEngine::dma;
    host_done = false;
    host_isr_entries = 0;
    peer_isr_entries = 0;
    (void)DmaHost::start(r);
    return late_wait_done(late_limit_ms(static_cast<uint16_t>(tx_len + rx_len), s));
}

/// The requests behind a staged read: a read of eight, a write of eight, a write the
/// client refuses at its second byte (in byte control for that write alone, letter v's
/// verbs: a client in byte control gives nothing to a read) and a write of eight again -
/// each on `dma`, no init() and no PE cycle of the host's between them.
void late_nexts(I2cSpeed s, LateRow& row) {
    row.nexts = row.nexts + 1u;
    const uint16_t k = late_serial = static_cast<uint16_t>(late_serial + 1u);
    late_serve(k, 8);
    uint8_t in[8] = {};
    bool same_in = late_request(nullptr, 0, in, 8, s) && DmaHost::status() == i2c_ok;
    for (uint8_t i = 0; i < 8u && same_in; ++i) {
        same_in = in[i] == late_value(k, i);
    }
    row.next_read = row.next_read + (same_in ? 1u : 0u);
    for (uint8_t i = 0; i < 8u; ++i) {
        late_out[i] = late_value(static_cast<uint16_t>(k + 1u), i);
    }
    peer_rx_n = 0;
    bool whole = late_request(late_out, 8, nullptr, 0, s) && DmaHost::status() == i2c_ok &&
                 peer_rx_n == 8u && same(peer_rx, late_out, 8);
    row.next_write = row.next_write + (whole ? 1u : 0u);
    (void)Peer::byte_control(true);
    Peer::interrupt(I2cInterrupt::transfer_complete, true);
    peer_nack_at = 2;
    peer_rx_n = 0;
    const bool refused = late_request(late_out, 8, nullptr, 0, s) &&
                         DmaHost::status() == i2c_nack_data && peer_rx_n == 2u;
    peer_nack_at = 0;
    Peer::interrupt(I2cInterrupt::transfer_complete, false);
    (void)Peer::byte_control(false);
    peer_rx_n = 0;
    whole = late_request(late_out, 8, nullptr, 0, s) && DmaHost::status() == i2c_ok &&
            peer_rx_n == 8u && same(peer_rx, late_out, 8);
    row.next_refused = row.next_refused + (refused && whole ? 1u : 0u);
}

/// One read on the channel, staged, judged in the vector and behind it.
void late_read(uint8_t tx_len, uint8_t n, I2cSpeed s, LateStage stage, int16_t turns,
               LateRow& row, uint8_t want = i2c_ok) {
    const uint16_t k = late_serial = static_cast<uint16_t>(late_serial + 1u);
    late_serve(k, n);
    for (uint8_t i = 0; i < late_buf + 1u; ++i) {
        late_in[i] = static_cast<uint8_t>(~late_value(k, i));
    }
    const uint8_t sentinel = late_in[n];
    for (uint8_t i = 0; i < 8u; ++i) {
        late_out[i] = static_cast<uint8_t>(0x11u + i);
    }
    DmaHost::Request r{};
    r.addr = peer_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(late_out));
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(late_in));
    r.rx_len = n;
    r.speed = s;
    r.engine = I2cEngine::dma;
    late_watch.cell = &late_in[n - 1u];
    late_watch.seen = static_cast<uint8_t>(~late_value(k, static_cast<uint16_t>(n - 1u)));
    late_watch.raise_turns = stage == LateStage::raise ? turns : int16_t{-1};
    late_watch.steal = stage == LateStage::steal;
    late_watch.race_words = stage == LateStage::race ? static_cast<uint16_t>(turns) : uint16_t{0};
    late_watch.on = true;
    const uint32_t running0 = late_watch.running;
    const uint32_t rxne0 = late_watch.rxne;
    const uint32_t limit = late_limit_ms(static_cast<uint16_t>(tx_len + n), s);
    host_done = false;
    host_isr_entries = 0;
    peer_isr_entries = 0;
    (void)DmaHost::start(r);
    if (stage != LateStage::none) {
        if (n > 1u) {
            const uint32_t t0 = Ticker::millis();
            while (LateChannel::count() > 1u && !host_done && Ticker::millis() - t0 < limit) {
            }
        }
        H::dma_receive(false);
    }
    const bool answered = late_wait_done(limit);
    late_watch.on = false;
    late_watch.raise_turns = -1;
    late_watch.steal = false;
    late_watch.race_words = 0;
    const uint8_t status = answered ? DmaHost::status() : no_answer;
    row.reads = row.reads + 1u;
    row.lost = row.lost + (answered ? 0u : 1u);
    row.status_wrong = row.status_wrong + (status != want ? 1u : 0u);
    row.running = row.running + (late_watch.running - running0);
    row.rxne = row.rxne + (late_watch.rxne - rxne0);
    // What the requester would read, the first load behind the body, then all of it.
    const uint8_t last = late_watch.seen;
    bool exact = answered && status == want && last == late_value(k, static_cast<uint16_t>(n - 1u));
    for (uint8_t i = 0; i < n && exact; ++i) {
        exact = late_in[i] == late_value(k, i);
    }
    exact = exact && late_in[n] == sentinel;
    row.exact = row.exact + (exact ? 1u : 0u);
    row.first_wrong = row.first_wrong + (late_in[0] != late_value(k, 0) ? 1u : 0u);
    row.last_wrong = row.last_wrong +
                     (late_in[n - 1u] != late_value(k, static_cast<uint16_t>(n - 1u)) ? 1u : 0u);
    row.left_on = row.left_on + (LateChannel::enabled() && LateChannel::count() != 0u ? 1u : 0u);
    uint8_t held[late_buf + 1u];
    for (uint8_t i = 0; i < late_buf + 1u; ++i) {
        held[i] = late_in[i];
    }
    late_burn(40);
    bool moved = late_in[n] != sentinel;
    for (uint8_t i = 0; i < late_buf + 1u; ++i) {
        moved = moved || held[i] != late_in[i];
    }
    row.stray = row.stray + (moved ? 1u : 0u);
}

/// Every shape once, at one speed, under one stage.
void late_shapes(I2cSpeed s, LateStage stage, int16_t turns, LateRow& row, bool nexts) {
    for (const uint8_t n : {uint8_t{1}, uint8_t{2}, uint8_t{8}}) {
        late_read(0, n, s, stage, turns, row);
        if (nexts) {
            late_nexts(s, row);
        }
        late_read(1, n, s, stage, turns, row);
        if (nexts) {
            late_nexts(s, row);
        }
    }
}

/// A read of one with the vector's line off and the request held down until RXNE
/// stands: the common ground of the polled stages. False if RXNE never rose.
bool late_hold(uint16_t k, I2cSpeed s, uint32_t& limit) {
    late_serve(k, 1);
    late_in[0] = static_cast<uint8_t>(~late_value(k, 0));
    DmaHost::Request r{};
    r.addr = peer_addr;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(late_in));
    r.rx_len = 1;
    r.speed = s;
    r.engine = I2cEngine::dma;
    host_done = false;
    host_isr_entries = 0;
    peer_isr_entries = 0;
    limit = late_limit_ms(1, s);
    Nvic::disable(H::irq());
    (void)DmaHost::start(r);
    H::dma_receive(false);
    const uint32_t t0 = Ticker::millis();
    while ((H::flags() & I2cFlag::rxne) == 0u && Ticker::millis() - t0 < limit) {
        late_load_turn();
    }
    return (H::flags() & I2cFlag::rxne) != 0u;
}

void late_release(uint32_t limit) {
    Nvic::enable(H::irq());
    (void)late_wait_done(limit);
}

// ---- what says the last beat is in memory ----

struct CountRow {
    uint32_t rows = 0;
    uint32_t stale = 0;
    uint32_t turns_max = 0;
};

/// The request let up, the count polled down to zero, and the very next load the
/// memory cell: 10.6.4 decrements "after each single DMA 'read followed by write'
/// transfer". A turn is one load of CNDTR and a branch.
void late_count_row(uint16_t k, I2cSpeed s, uint32_t lead, CountRow& row) {
    uint32_t limit = 0;
    if (!late_hold(k, s, limit)) {
        late_release(limit);
        return;
    }
    const uint32_t request = H::regs().CR1 | I2C_CR1_RXDMAEN;
    uint8_t first = 0;
    uint32_t turns = 0;
    {
        InterruptGuard guard;
        late_burn(lead);
        H::regs().CR1 = request;
        while (LateChannel::count() != 0u && turns < 20'000u) {
            ++turns;
        }
        first = *static_cast<volatile uint8_t*>(late_in);
    }
    row.rows = row.rows + 1u;
    row.stale = row.stale + (first != late_value(k, 0) ? 1u : 0u);
    row.turns_max = turns > row.turns_max ? turns : row.turns_max;
    late_release(limit);
}

// ---- the request taken down against the request rising ----

struct DownRow {
    uint32_t rows = 0;
    uint32_t taken = 0;      ///< count down at the first load
    uint32_t in_flight = 0;  ///< RXNE down, count up: the beat under way
    uint32_t left = 0;       ///< RXNE up, count up: no beat seen
    uint32_t left_taken = 0; ///< ... and the count fell afterwards: a grant outlived the enable
    uint32_t in_flight_turns = 0;   ///< the most loads the count took to fall
    uint32_t in_flight_stuck = 0;   ///< the count never fell
    uint32_t stale = 0;      ///< count down, the cell at the next load not the byte
};

/// RXDMAEN up, then down `turns` of burn() later, with interrupts masked: where the
/// first loads find the beat. 10.4.3: the request is acknowledged at the grant, and the
/// read and the write come after, so a grant can outlive the enable.
void late_down_row(uint16_t k, I2cSpeed s, uint32_t turns, DownRow& row) {
    uint32_t limit = 0;
    if (!late_hold(k, s, limit)) {
        late_release(limit);
        return;
    }
    const uint32_t down = H::regs().CR1;
    const uint32_t up = down | I2C_CR1_RXDMAEN;
    uint32_t isr = 0;
    uint16_t count = 0;
    uint32_t wait = 0;
    uint16_t later = 1;
    uint8_t first = 0;
    {
        InterruptGuard guard;
        H::regs().CR1 = up;
        late_burn(turns);
        H::regs().CR1 = down;
        isr = H::flags();
        count = LateChannel::count();
        later = count;
        while (later != 0u && wait < 256u) {
            later = LateChannel::count();
            ++wait;
        }
        first = *static_cast<volatile uint8_t*>(late_in);
    }
    row.rows = row.rows + 1u;
    const bool rxne_up = (isr & I2cFlag::rxne) != 0u;
    if (count == 0u) {
        row.taken = row.taken + 1u;
        row.stale = row.stale + (first != late_value(k, 0) ? 1u : 0u);
    } else if (!rxne_up) {
        row.in_flight = row.in_flight + 1u;
        if (later != 0u) {
            row.in_flight_stuck = row.in_flight_stuck + 1u;
        } else {
            row.in_flight_turns = wait > row.in_flight_turns ? wait : row.in_flight_turns;
            row.stale = row.stale + (first != late_value(k, 0) ? 1u : 0u);
        }
    } else {
        row.left = row.left + 1u;
        if (later == 0u) {
            row.left_taken = row.left_taken + 1u;
            row.in_flight_turns = wait > row.in_flight_turns ? wait : row.in_flight_turns;
        }
    }
    late_release(limit);
}

// ---- the enable cleared against the beat ----

struct ClearRow {
    uint32_t rows = 0;
    uint32_t left = 0;       ///< RXNE up: the clear came first
    uint32_t whole = 0;      ///< RXNE down, the cell the byte, the count down
    uint32_t count_up = 0;   ///< RXNE down, the cell the byte, the count up
    uint32_t aborted = 0;    ///< RXNE down and the cell never the byte: the beat cut between its accesses
};

/// EN cleared `turns` of burn() after the request is let up, the request still up:
/// 10.4.5's "stopped and aborted", against a beat in progress. The first loads after
/// the store are the cell, the count, ISR; the cell again after a pause.
void late_clear_row(uint16_t k, I2cSpeed s, uint32_t turns, ClearRow& row) {
    uint32_t limit = 0;
    if (!late_hold(k, s, limit)) {
        late_release(limit);
        return;
    }
    const uint32_t up = H::regs().CR1 | I2C_CR1_RXDMAEN;
    const uint32_t cleared = LateChannel::regs().CCR & ~DMA_CCR_EN;
    uint32_t isr = 0;
    uint16_t count = 0;
    uint8_t later = 0;
    {
        InterruptGuard guard;
        H::regs().CR1 = up;
        late_burn(turns);
        LateChannel::regs().CCR = cleared;
        count = LateChannel::count();
        isr = H::flags();
        H::dma_receive(false);
        late_burn(40);
        later = *static_cast<volatile uint8_t*>(late_in);
    }
    row.rows = row.rows + 1u;
    if ((isr & I2cFlag::rxne) != 0u) {
        row.left = row.left + 1u;
    } else if (later != late_value(k, 0)) {
        row.aborted = row.aborted + 1u;
    } else if (count != 0u) {
        row.count_up = row.count_up + 1u;
    } else {
        row.whole = row.whole + 1u;
    }
    late_release(limit);
}

constexpr LateLoad late_load_saturating{"three paced channels, every 8 cycles", 3, 8, 0};

// ---- a stray beat ----

/// The next block's count, a hundred burns after its start with nothing on the wire:
/// a beat there is the request a stop left behind. The tenure before it is waited out
/// first; the enable is down throughout.
bool late_next_block_moved() {
    static uint8_t scratch[2];
    HostRx::stop();
    (void)HostRx::start(std::span<uint8_t>(scratch, 2u));
    late_burn(100);
    const bool moved = LateChannel::count() != 2u;
    HostRx::stop();
    return moved;
}

/// A clear ahead of the request, as F4's page has it: EN down, the next store RXDMAEN
/// up, then RXDR read, with the enable up or taken down ahead of the read.
void late_stray_clear_first(uint16_t k, I2cSpeed s, bool enable_first, uint32_t& left,
                            uint32_t& strays) {
    uint32_t limit = 0;
    if (!late_hold(k, s, limit)) {
        late_release(limit);
        return;
    }
    const uint32_t up = H::regs().CR1 | I2C_CR1_RXDMAEN;
    const uint32_t cleared = LateChannel::regs().CCR & ~DMA_CCR_EN;
    bool stood = false;
    {
        InterruptGuard guard;
        LateChannel::regs().CCR = cleared;
        H::regs().CR1 = up;
        stood = (H::flags() & I2cFlag::rxne) != 0u;
        if (enable_first) {
            H::dma_receive(false);
        }
        (void)H::data();
    }
    late_release(limit);
    const bool moved = late_next_block_moved();
    left = left + (stood ? 1u : 0u);
    strays = strays + (stood && moved ? 1u : 0u);
}

/// A channel starved under a request, the shape of the entry on a load that outranks
/// the channel. The load holds the DMA controller's arbitration (a block longer than the row) and the request
/// goes up under it. Then, by `order`, the steps of a stop, and only then the load off:
/// `after` is the count behind the load, the next block's start is judged as above.
enum class LateOrder : uint8_t {
    read_up,          ///< RXDR read with the enable up, the channel left running
    down_read,        ///< the enable down, RXDR read, the channel left running
    down_read_clear,  ///< the enable down, RXDR read, EN cleared
    clear_down_read,  ///< EN cleared, the enable down, RXDR read
    down_clear_read,  ///< the enable down, EN cleared, RXDR read
};

struct StarvedRow {
    uint32_t rows = 0;
    uint32_t served = 0;    ///< the count down ahead of the steps: the load let it through
    uint32_t behind = 0;    ///< a beat behind the load
    uint32_t stray = 0;     ///< a beat at the next block's start
};

void late_stray_starved(uint16_t k, I2cSpeed s, LateOrder order, StarvedRow& row) {
    late_load_on(late_load_saturating);
    uint32_t limit = 0;
    if (!late_hold(k, s, limit)) {
        late_load_off();
        late_release(limit);
        return;
    }
    const uint32_t up = H::regs().CR1 | I2C_CR1_RXDMAEN;
    uint16_t count = 0;
    uint16_t after = 0;
    {
        InterruptGuard guard;
        late_load_turn();
        H::regs().CR1 = up;
        late_burn(2);
        count = LateChannel::count();
        switch (order) {
            case LateOrder::read_up:
                (void)H::data();
                break;
            case LateOrder::down_read:
                H::dma_receive(false);
                (void)H::data();
                break;
            case LateOrder::down_read_clear:
                H::dma_receive(false);
                (void)H::data();
                (void)LateChannel::enable(false);
                break;
            case LateOrder::clear_down_read:
                (void)LateChannel::enable(false);
                H::dma_receive(false);
                (void)H::data();
                break;
            case LateOrder::down_clear_read:
                H::dma_receive(false);
                (void)LateChannel::enable(false);
                (void)H::data();
                break;
        }
        late_load_off();
        late_burn(100);
        after = LateChannel::count();
    }
    row.rows = row.rows + 1u;
    row.served = row.served + (count == 0u ? 1u : 0u);
    row.behind = row.behind + (count != 0u && after != count ? 1u : 0u);
    late_release(limit);
    row.stray = row.stray + (late_next_block_moved() ? 1u : 0u);
}

/// A tenure dropped by recover() over a starved channel, the request standing: the
/// per-bus timeout's path behind a load. The next block's start is judged as above.
void late_recover_row(uint16_t k, I2cSpeed s, StarvedRow& row) {
    late_load_on(late_load_saturating);
    uint32_t limit = 0;
    if (!late_hold(k, s, limit)) {
        late_load_off();
        late_release(limit);
        return;
    }
    const uint32_t up = H::regs().CR1 | I2C_CR1_RXDMAEN;
    uint16_t count = 0;
    uint16_t after = 0;
    {
        InterruptGuard guard;
        late_load_turn();
        H::regs().CR1 = up;
        late_burn(2);
        count = LateChannel::count();
        (void)DmaHost::recover();
        late_load_off();
        late_burn(100);
        after = LateChannel::count();
    }
    Nvic::enable(H::irq());
    row.rows = row.rows + 1u;
    row.served = row.served + (count == 0u ? 1u : 0u);
    row.behind = row.behind + (count != 0u && after != count ? 1u : 0u);
    row.stray = row.stray + (late_next_block_moved() ? 1u : 0u);
}

// ---- the letters ----

/// The client, its timing solved for Fm+ on PCLK where the rung asks
/// for 1 MHz (ES0548 2.10.1's 20 MHz floor rules HSI16 out there), on HSI16 where the
/// core is under that floor.
void late_client(I2cClock kernel) {
    uint8_t serve[8];
    fill_pattern(serve, 8, 0x60);
    const I2cSpeed solved = kernel == I2cClock::pclk ? I2cSpeed::fast_plus_1m : I2cSpeed::fast_400k;
    (void)peer_arm(serve, 8, solved, false, peer_addr, I2cAddressMode::seven_bit, 0,
                   I2cOa2Mask::none, false, false, I2cFilters{}, kernel);
    peer_nack_at = 0;
}

/// The 20 mA drive on both ends for the 1 MHz rung, off for the others.
void late_rung(I2cSpeed s) {
    const bool plus = s == I2cSpeed::fast_plus_1m;
    C::instance_fast_plus(plus);
    DmaHost::fast_plus_drive(plus);
}

bool late_begin() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return false;
    }
    if (!need_self_link()) {
        return false;
    }
    uint8_t serve[8];
    fill_pattern(serve, 8, 0x60);
    (void)peer_arm(serve, 8);
    host_ready();
    if (!link_alive()) {
        bench.verdict("the link is alive (declined: no answer from the client)", false);
        peer_stop();
        return false;
    }
    Host::release();
    late_client(I2cClock::pclk);
    dma_host_live = true;
    return DmaHost::init(clock, I2cClock::pclk);
}

void late_end() {
    late_watch = LateWatch{};
    late_rung(I2cSpeed::fast_400k);
    dma_host_live = false;
    DmaHost::release();
    SysClock::set_index(r_fast);
    console_drain();
    (void)Host::init(clock, I2cClock::pclk);
    peer_stop();
}

/// C - the staging. Rows of a run: `reads` of 12 for the shapes, three speeds.
void tC_late_stage() {
    if (!late_begin()) {
        return;
    }
    const I2cSpeed speeds[3] = {I2cSpeed::standard_100k, I2cSpeed::fast_400k,
                                I2cSpeed::fast_plus_1m};
    const char* const names[3] = {"100 kHz", "400 kHz", "1 MHz"};

    // ---- what says the beat is in memory ----
    CountRow count{};
    for (uint32_t lead = 0; lead < 8u; ++lead) {
        for (uint8_t i = 0; i < 25u; ++i) {
            late_serial = static_cast<uint16_t>(late_serial + 1u);
            late_count_row(late_serial, I2cSpeed::fast_400k, lead, count);
        }
    }
    late_load_on({"", 0, 0, 1});
    for (uint32_t lead = 0; lead < 8u; ++lead) {
        for (uint8_t i = 0; i < 25u; ++i) {
            late_serial = static_cast<uint16_t>(late_serial + 1u);
            late_count_row(late_serial, I2cSpeed::fast_400k, lead, count);
        }
    }
    late_load_off();
    print(serial, "  the count polled to zero, the next load the cell: ", count.rows,
          " rows (half under a copy at very_high), stale ", count.stale,
          ", at most ", count.turns_max, " loads of the count", crlf);
    // No bound to derive: 10.6.4's order, each row judged at zero. A stale cell would
    // say the count falls ahead of the memory's write.
    bench.verdict("THE COUNT AT ZERO SAYS THE LAST BEAT IS IN MEMORY: the first load after it reads "
                  "the byte", count.rows == 400u && count.stale == 0u);

    // ---- the request taken down against the request rising ----
    DownRow down{};
    for (uint32_t turns = 0; turns < 24u; ++turns) {
        for (uint8_t i = 0; i < 20u; ++i) {
            late_serial = static_cast<uint16_t>(late_serial + 1u);
            late_down_row(late_serial, I2cSpeed::fast_400k, turns, down);
        }
    }
    print(serial, "  RXDMAEN up, then down 0 to 23 burns later: ", down.rows, " rows; the count down "
          "at the first load ", down.taken, ", the beat under way (RXNE down, count up) ",
          down.in_flight, ", no beat (RXNE up) ", down.left, "; of those a beat came after all ",
          down.left_taken, "; the count fell within ", down.in_flight_turns,
          " loads, never in ", down.in_flight_stuck, "; a stale cell ", down.stale, crlf);
    // 480 = 24 turns x 20 rows. Zero judged on the stuck and the stale: a beat under
    // way that never ended, or ended with the wrong byte, contradicts the entry's wait.
    // Both pass with no beat caught, which is the printed count's to say.
    bench.verdict("ANY BEAT THAT OUTLIVES THE REQUEST FINISHES, IF ONE IS CAUGHT: its count falls and the cell "
                  "is the byte", down.rows == 480u && down.in_flight_stuck == 0u && down.stale == 0u);
    bench.verdict("... inside the entry's wait, if any", down.in_flight_turns < DmaHost::late_beat_turns);

    // ---- the enable cleared against the beat ----
    ClearRow clear{};
    for (uint32_t turns = 0; turns < 24u; ++turns) {
        for (uint8_t i = 0; i < 20u; ++i) {
            late_serial = static_cast<uint16_t>(late_serial + 1u);
            late_clear_row(late_serial, I2cSpeed::fast_400k, turns, clear);
        }
    }
    print(serial, "  EN cleared 0 to 23 burns after RXDMAEN up, the request up: ", clear.rows,
          " rows; RXNE up, the clear first ", clear.left, "; the byte in memory, count down ",
          clear.whole, "; the byte in memory, count up ", clear.count_up,
          "; RXNE down and the byte nowhere ", clear.aborted, crlf);

    // ---- a stray beat ----
    uint32_t left_up = 0;
    uint32_t stray_up = 0;
    uint32_t left_down = 0;
    uint32_t stray_down = 0;
    for (uint8_t i = 0; i < 100u; ++i) {
        late_serial = static_cast<uint16_t>(late_serial + 1u);
        late_stray_clear_first(late_serial, I2cSpeed::fast_400k, false, left_up, stray_up);
        late_serial = static_cast<uint16_t>(late_serial + 1u);
        late_stray_clear_first(late_serial, I2cSpeed::fast_400k, true, left_down, stray_down);
    }
    print(serial, "  EN down, then RXDMAEN up, then RXDR read: with the enable up, the byte stood in ",
          left_up, " of 100, a beat at the next block's start in ", stray_up,
          "; the enable down first, stood in ", left_down, ", a beat in ", stray_down, crlf);
    StarvedRow host_order{};
    const LateOrder orders[5] = {LateOrder::read_up, LateOrder::down_read, LateOrder::down_read_clear,
                                 LateOrder::clear_down_read, LateOrder::down_clear_read};
    const char* const order_names[5] = {"RXDR read, enable up", "enable down, read",
                                        "enable down, read, EN clear", "EN clear, enable down, read",
                                        "enable down, EN clear, read"};
    for (uint8_t o = 0; o < 5u; ++o) {
        StarvedRow starved{};
        for (uint8_t i = 0; i < 100u; ++i) {
            late_serial = static_cast<uint16_t>(late_serial + 1u);
            late_stray_starved(late_serial, I2cSpeed::fast_400k, orders[o], starved);
        }
        print(serial, "  a starved channel, ", order_names[o], ": ", starved.rows, " rows, served ahead ",
              starved.served, ", a beat behind the load ", starved.behind,
              ", a beat at the next block's start ", starved.stray, crlf);
        if (orders[o] == LateOrder::down_read_clear) {
            host_order = starved;
        }
    }
    StarvedRow recovered{};
    for (uint8_t i = 0; i < 100u; ++i) {
        late_serial = static_cast<uint16_t>(late_serial + 1u);
        late_recover_row(late_serial, I2cSpeed::fast_400k, recovered);
    }
    print(serial, "  a starved channel, recover() over the standing request: ", recovered.rows,
          " rows, served ahead ", recovered.served, ", a beat behind the load ", recovered.behind,
          ", a beat at the next block's start ", recovered.stray, crlf);
    // No bound to derive: 100 rows a run, each stray judged at zero under the two orders the
    // host keeps. The other orders' counts are printed: the silicon's, not the host's. A
    // stray of the clear-first rows is counted only where the byte stood, so the stage must
    // have reached that case on every row for their zero to say anything.
    bench.verdict("THE HOST'S ORDER LEAVES NOTHING: the enable down ahead of the read, the byte "
                  "standing on every row, a beat at the next block's start in no row, the channel "
                  "starved or not",
                  left_down == 100u && stray_down == 0u && host_order.rows == 100u &&
                      host_order.stray == 0u);
    bench.verdict("... and neither does recover() over a standing request",
                  recovered.rows == 100u && recovered.stray == 0u);

    // ---- the request taken away for the last byte ----
    bool all_exact = true;
    bool all_next = true;
    for (uint8_t v = 0; v < 3u; ++v) {
        late_rung(speeds[v]);
        LateRow stage{};
        for (uint8_t pass = 0; pass < 5u; ++pass) {
            late_shapes(speeds[v], LateStage::starve, -1, stage, true);
        }
        print(serial, "  the request taken away for the last byte, ", names[v], crlf);
        late_row_print("reads of 1, 2, 8 and write-then-reads 1+1, 1+2, 1+8", stage);
        // 30 = 5 passes x 6 shapes. The stage holds the byte back, so every entry
        // finds the channel running and RXNE up: anything else says the stage missed.
        all_exact = all_exact && stage.reads == 30u && stage.exact == 30u && stage.running == 30u &&
                    stage.rxne == 30u && stage.left_on == 0u && stage.stray == 0u;
        all_next = all_next && stage.next_read == 30u && stage.next_write == 30u &&
                   stage.next_refused == 30u;
    }
    bench.verdict("THE LAST BYTE THE CHANNEL WAS NEVER ASKED FOR: every byte at its place in the vector, "
                  "the channel stopped, nothing moving afterwards, at the three speeds", all_exact);
    bench.verdict("... and behind it a read of eight, a write of eight and a refused write whole",
                  all_next);

    // ---- the same, the request let up at the entry ----
    bool raised_exact = true;
    bool raised_next = true;
    for (uint8_t v = 1; v < 3u; ++v) {
        late_rung(speeds[v]);
        LateRow raised{};
        for (int16_t turns = 0; turns < 24; ++turns) {
            late_read(0, 1, speeds[v], LateStage::raise, turns, raised);
            late_read(0, 2, speeds[v], LateStage::raise, turns, raised);
            late_read(0, 8, speeds[v], LateStage::raise, turns, raised);
            late_read(1, 8, speeds[v], LateStage::raise, turns, raised);
            late_nexts(speeds[v], raised);
        }
        print(serial, "  the request let up in the STOPF entry 0 to 23 burns ahead of the body, ",
              names[v], crlf);
        late_row_print("reads of 1, 2, 8, a write-then-read 1+8", raised);
        // 96 = 24 turns x 4 shapes, every entry staged as above; a beat may win by
        // the host's first load, so `running` is printed and not judged here.
        raised_exact = raised_exact && raised.reads == 96u && raised.exact == 96u &&
                       raised.left_on == 0u && raised.stray == 0u;
        raised_next = raised_next && raised.next_read == 24u && raised.next_write == 24u &&
                      raised.next_refused == 24u;
    }
    bench.verdict("THE HOST STOPS A CHANNEL OVER A STANDING REQUEST: every byte in the vector, swept "
                  "across the entry, at 400 kHz and 1 MHz", raised_exact);
    bench.verdict("... and behind it a read of eight, a write of eight and a refused write whole",
                  raised_next);

    // ---- the request under a load that ends inside the entry ----
    bool raced_exact = true;
    bool raced_next = true;
    for (uint8_t v = 1; v < 3u; ++v) {
        late_rung(speeds[v]);
        late_load_on(late_load_saturating);
        (void)DmaChannel<1, 3>::enable(false);
        (void)DmaChannel<1, 4>::enable(false);
        (void)DmaChannel<1, 5>::enable(false);
        late_load_paced = 0;   // the entry's to start, the thread's turn out of it
        LateRow raced{};
        for (int16_t words = 1; words <= 48; ++words) {
            late_read(0, 1, speeds[v], LateStage::race, words, raced);
            late_read(0, 2, speeds[v], LateStage::race, words, raced);
            late_read(0, 8, speeds[v], LateStage::race, words, raced);
            late_read(1, 8, speeds[v], LateStage::race, words, raced);
            late_nexts(speeds[v], raced);
        }
        late_load_paced = 3;
        late_load_off();
        print(serial, "  the request up in the entry under 1 to 48 beats of three channels above it, ",
              names[v], crlf);
        late_row_print("reads of 1, 2, 8, a write-then-read 1+8", raced);
        // 192 = 48 beats x 4 shapes; `running` is every row's by the stage's making, and the
        // beat may land on either side of the body's load, so nothing else is judged of it.
        raced_exact = raced_exact && raced.reads == 192u && raced.exact == 192u &&
                      raced.left_on == 0u && raced.stray == 0u;
        raced_next = raced_next && raced.next_read == 48u && raced.next_write == 48u &&
                     raced.next_refused == 48u;
    }
    print(serial, "  the host's body so far, between SysTick reads: at most ", late_watch.body_on_time,
          " cycles with the channel done at the first load, ", late_watch.body_late,
          " with it running", crlf);
    bench.verdict("A BEAT LANDING ANYWHERE IN THE ENTRY: every byte in the vector, nothing moving after",
                  raced_exact);
    bench.verdict("... and behind it a read of eight, a write of eight and a refused write whole",
                  raced_next);

    // ---- a byte nobody has ----
    late_rung(I2cSpeed::fast_400k);
    late_watch.body_late = 0;
    LateRow stolen{};
    for (uint8_t pass = 0; pass < 10u; ++pass) {
        late_read(0, 1, I2cSpeed::fast_400k, LateStage::steal, -1, stolen, i2c_dma_fault);
        late_read(0, 8, I2cSpeed::fast_400k, LateStage::steal, -1, stolen, i2c_dma_fault);
        late_nexts(I2cSpeed::fast_400k, stolen);
    }
    print(serial, "  the last byte read away ahead of the host's body: ", stolen.reads,
          " reads, the body at most ", late_watch.body_late, " cycles, the status owed wrong in ",
          stolen.status_wrong, ", left running ", stolen.left_on,
          "; behind them read whole ", stolen.next_read, ", write whole ", stolen.next_write,
          ", refused whole ", stolen.next_refused, crlf);
    // 20 = 10 passes x 2 lengths, 10 of the requests behind them.
    bench.verdict("A BYTE NOBODY HAS IS A STATUS, NEVER i2c_ok: i2c_dma_fault, and the channel stopped",
                  stolen.reads == 20u && stolen.status_wrong == 0u && stolen.left_on == 0u &&
                      stolen.lost == 0u);
    bench.verdict("... and the requests behind it whole",
                  stolen.next_read == 10u && stolen.next_write == 10u && stolen.next_refused == 10u);

    late_end();
}

// ---- D: the load, the lag, the window ----

struct LagRow {
    uint32_t rows = 0;
    uint32_t lag_max = 0;
    uint32_t window_min = 0xFFFF'FFFFu;
    int32_t margin_min = 0x7FFF'FFFF;
    uint32_t late = 0;     ///< STOPF up with the count not yet at zero
    uint32_t turn_max = 0;
    uint32_t wrong = 0;
};

/// The request held down until RXNE rises, then let up: a masked poll dates the count's
/// zero (the channel's lag, from the request) and STOPF (the window it has), one tenure.
/// The client has given its byte before the mask comes down. A turn of the wait for RXNE
/// is one load and a branch, a turn of the poll three loads (CNDTR, ISR, SysTick's VAL):
/// the figures' resolution, the poll's printed. Each row is good to that turn, and the
/// let-up itself costs a store and a load on the far side of RXNE.
void late_lag_row(uint16_t k, I2cSpeed s, LagRow& row) {
    late_serve(k, 1);
    late_in[0] = static_cast<uint8_t>(~late_value(k, 0));
    DmaHost::Request r{};
    r.addr = peer_addr;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(late_in));
    r.rx_len = 1;
    r.speed = s;
    r.engine = I2cEngine::dma;
    host_done = false;
    host_isr_entries = 0;
    peer_isr_entries = 0;
    peer_tx_n = 0;
    const uint32_t limit = late_limit_ms(1, s);
    Nvic::disable(H::irq());
    (void)DmaHost::start(r);
    H::dma_receive(false);
    const uint32_t up = H::regs().CR1 | I2C_CR1_RXDMAEN;
    const uint32_t t_wait = Ticker::millis();
    while (peer_tx_n == 0u && Ticker::millis() - t_wait < limit) {
        late_load_turn();
    }
    const uint32_t period = SysTick->LOAD + 1u;
    uint32_t zero_at = 0;
    uint32_t stop_at = 0;
    uint32_t turn = 0;
    bool zero = false;
    bool stopped = false;
    {
        InterruptGuard guard;
        late_load_turn();
        for (uint32_t turns = 200'000u; turns != 0u && (H::flags() & I2cFlag::rxne) == 0u; --turns) {
        }
        const uint32_t t0 = SysTick->VAL;
        H::regs().CR1 = up;
        uint32_t last = t0;
        bool first = true;
        for (uint32_t turns = 4000u; turns != 0u && !(zero && stopped); --turns) {
            const uint16_t c = LateChannel::count();
            const uint32_t f = H::flags();
            const uint32_t now = SysTick->VAL;
            // SysTick counts down and wraps at most once in a row (no division: a
            // modulo on this core would be most of the turn).
            const uint32_t step = last >= now ? last - now : last + period - now;
            turn = !first && step > turn ? step : turn;
            first = false;
            last = now;
            const uint32_t at = t0 >= now ? t0 - now : t0 + period - now;
            if (!zero && c == 0u) {
                zero = true;
                zero_at = at;
            }
            if (!stopped && (f & I2cFlag::stop) != 0u) {
                stopped = true;
                stop_at = at;
            }
        }
    }
    late_release(limit);
    row.wrong = row.wrong + (host_done && DmaHost::status() == i2c_ok && late_in[0] == late_value(k, 0)
                                 ? 0u
                                 : 1u);
    if (!stopped) {
        return;
    }
    row.rows = row.rows + 1u;
    row.turn_max = turn > row.turn_max ? turn : row.turn_max;
    row.window_min = stop_at < row.window_min ? stop_at : row.window_min;
    if (zero) {
        row.lag_max = zero_at > row.lag_max ? zero_at : row.lag_max;
        const int32_t m = static_cast<int32_t>(stop_at) - static_cast<int32_t>(zero_at);
        row.margin_min = m < row.margin_min ? m : row.margin_min;
    }
    if (!zero || zero_at > stop_at) {
        row.late = row.late + 1u;
    }
}

void late_lag_print(const LagRow& r) {
    print(serial, "      polled, ", r.rows, " tenures: the count's zero at most ", r.lag_max,
          " core cycles from the request, STOPF at least ", r.window_min, ", the least margin ",
          r.margin_min, ", STOPF first in ", r.late, ", a poll's turn at most ", r.turn_max,
          ", wrong ", r.wrong, crlf);
}

constexpr LateLoad late_loads[] = {
    {"nothing else", 0, 0, 0},
    {"a paced channel, every 64 cycles", 1, 64, 0},
    {"a paced channel, every 16 cycles", 1, 16, 0},
    {"a paced channel, every 4 cycles", 1, 4, 0},
    {"two paced channels, every 16 cycles", 2, 16, 0},
    {"two paced channels, every 4 cycles", 2, 4, 0},
    {"three paced channels, every 16 cycles", 3, 16, 0},
    {"three paced channels, every 8 cycles", 3, 8, 0},
    {"... their timers 1 burn apart", 3, 8, 0, 1},
    {"... 2 burns apart", 3, 8, 0, 2},
    {"... 3 burns apart", 3, 8, 0, 3},
    {"three paced channels, every 6 cycles", 3, 6, 0},
    {"... 1 burn apart", 3, 6, 0, 1},
    {"three paced channels, every 4 cycles", 3, 4, 0},
    {"... 1 burn apart", 3, 4, 0, 1},
    {"a copy", 0, 0, 1},
    {"two copies", 0, 0, 2},
    {"three copies", 0, 0, 3},
    {"three copies at low, the default", 0, 0, 3, 0, DmaPriority::low},
    {"a paced channel every 8 cycles and two copies", 1, 8, 2},
};

constexpr LateLoad late_loads_copies{"three copies", 0, 0, 3};

struct LateCensus {
    uint32_t reads = 0;
    uint32_t exact = 0;
    uint32_t lost = 0;
};

/// One row: reads of 1, 2 and 8 on nothing but the load, each length's count of a
/// channel still running at the vector's first load, then the poll's margin on reads
/// of one.
void late_census_row(const char* name, I2cSpeed s, LateCensus& all) {
    LateRow by[3] = {};
    const uint8_t lengths[3] = {1, 2, 8};
    for (uint8_t pass = 0; pass < 50u; ++pass) {
        for (uint8_t i = 0; i < 3u; ++i) {
            late_read(0, lengths[i], s, LateStage::none, -1, by[i]);
        }
    }
    LagRow lag{};
    for (uint8_t i = 0; i < 40u; ++i) {
        late_serial = static_cast<uint16_t>(late_serial + 1u);
        late_lag_row(late_serial, s, lag);
    }
    print(serial, "    ", name, ": reads of 1, 2, 8, 50 each: exact ", by[0].exact, ", ", by[1].exact, ", ",
          by[2].exact, "; running at the entry ", by[0].running, ", ", by[1].running, ", ",
          by[2].running, "; unanswered ", by[0].lost + by[1].lost + by[2].lost, "; left running ",
          by[0].left_on + by[1].left_on + by[2].left_on, crlf);
    late_lag_print(lag);
    console_drain();
    for (const LateRow& r : by) {
        all.reads = all.reads + r.reads;
        all.exact = all.exact + r.exact;
        all.lost = all.lost + r.lost;
    }
}

/// D - the census: every load at 400 kHz and 1 MHz, then the slower rungs.
void tD_late_load() {
    if (!late_begin()) {
        return;
    }
    const I2cSpeed speeds[2] = {I2cSpeed::fast_400k, I2cSpeed::fast_plus_1m};
    const char* const names[2] = {"400 kHz", "1 MHz"};
    LateCensus all{};
    for (const LateLoad& l : late_loads) {
        print(serial, "  ", l.name, crlf);
        for (uint8_t v = 0; v < 2u; ++v) {
            late_rung(speeds[v]);
            late_load_on(l);
            late_census_row(names[v], speeds[v], all);
            late_load_off();
        }
    }
    // The slower rungs, the kernel on HSI16: the bus's window shrinks in core cycles
    // and the channel's own does not. Bare, and under the load the rows above name.
    late_rung(I2cSpeed::fast_400k);
    late_client(I2cClock::hsi16);
    const uint8_t rungs[2] = {r_mid, r_slow};
    const char* const rung_names[2] = {"core 16 MHz, kernel HSI16", "core 2 MHz, kernel HSI16"};
    for (uint8_t i = 0; i < 2u; ++i) {
        SysClock::set_index(rungs[i]);
        console_drain();
        if (!DmaHost::init(clock, I2cClock::hsi16)) {
            all.reads = all.reads + 1u;   // no verdict can pass over a refused rung
        }
        print(serial, "  ", rung_names[i], crlf);
        late_census_row("400 kHz, nothing else", speeds[0], all);
        late_load_on(late_loads_copies);
        late_census_row("400 kHz, three copies", speeds[0], all);
        late_load_off();
        SysClock::set_index(r_fast);
        console_drain();
    }
    const uint32_t all_reads = all.reads;
    const uint32_t all_exact = all.exact;
    const uint32_t all_lost = all.lost;
    print(serial, "  in all ", all_reads, " reads, exact ", all_exact, ", unanswered ", all_lost, crlf);
    // No bound to derive: each read is judged byte for byte, and unanswered is
    // late_limit_ms(), its arithmetic above. How often a channel was late is the load's
    // and is printed, not judged: a rate of zero would say nothing of the next load.
    bench.verdict("EVERY READ EXACT UNDER EVERY LOAD, late or not, its last byte in memory in the vector",
                  all_reads != 0u && all_exact == all_reads && all_lost == 0u);
    late_end();
}

void banner() {
    print(serial, crlf, "test_stm32_i2c - STM32G0 I2C (RM0444 ch. 32)", crlf);
    if (self_link) {
        print(serial, "  the SELF-LINK is on the desk: I2C1 host PB8/PB9 AF6  <->  "
                      "I2C2 client PA11/PA12 AF6; letters b..m, u, v, B, C and D are live, client "
                      "address ",
              hex(peer_addr), ", client kernel HSI16", crlf);
    } else {
        print(serial, "  NO SELF-LINK on the desk (probed): letters b..l, u, v, B, x and y "
                      "skip themselves and claim nothing", crlf);
    }
    print(serial, "  PB8/PB9 AF6 with their 2.2k pull-ups reach a PEER BOARD "
                  "running `twi_peer` (its ident says which port; command address ",
          hex(twilink::command_addr), ") - letters n..t under 'w' (s and t on the "
          "self-link too); they FAIL when it does not answer", crlf);
    bench.menu();
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// The Stop's backstop (letter t): counted, because Rtc::isr() clears WUTF
/// at entry and a flag read after the sleep would say it never fired.
extern "C" void RTC_TAMP_IRQHandler() {
    const uint32_t f = brio::Rtc::isr();
    if ((f & brio::RtcFlag::wakeup) != 0u) {
        rtc_backstops = rtc_backstops + 1u;
    }
}
extern "C" void BRIO_SUITE_CONSOLE_HANDLER() { (void)Serial::isr(); }

/// I2C1's line is its own. Which host owns it is a flag, because two
/// I2cHost instantiations over one instance share the peripheral and not
/// the statics.
extern "C" void I2C1_IRQHandler() {
    host_isr_entries = host_isr_entries + 1u;
    if (host_isr_entries > host_isr_budget) {
        host_storm_flags = H::flags();
        host_storm_cr1 = H::regs().CR1;
        host_stormed = true;
        H::interrupt(brio::I2cInterrupt::all, false);
        H::clear(brio::I2cClear::all);
        return;
    }
    if (trace_isrs && host_trace_n < 12u) {
        host_trace[host_trace_n] = H::flags();
        host_trace_n = static_cast<uint8_t>(host_trace_n + 1u);
    }
    if (raw_host_live) {
        // A letter driving the RESOURCE by hand: nothing here owns the
        // flags, so the vector is silenced by disabling the interrupts
        // rather than by serving them.
        H::interrupt(brio::I2cInterrupt::all, false);
        return;
    }
    if (dma_host_live) {
        if (late_watch.on) {
            late_watch_before();
        }
        if (DmaHost::isr()) {
            if (late_watch.on) {
                late_watch_after();
            }
            host_done = true;
            host_completions = static_cast<uint16_t>(host_completions + 1u);
        }
        return;
    }
    if (bus_ao_live) {
        if (Host::isr() && !host_swallow_completion) {
            brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
        }
        return;
    }
    if (Host::isr()) {
        host_done = true;
        host_completions = static_cast<uint16_t>(host_completions + 1u);
    }
}

/// I2C2 and I2C3 share one line where the part has an I2C3 (the reserve
/// derives the name), so the app binds the derived spelling.
extern "C" void BRIO_STM32G0_I2C2_HANDLER() { peer_service(); }

extern "C" void DMA1_Channel1_IRQHandler() {
    dma_isr_entries = dma_isr_entries + 1u;
    if (dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
    }
}

extern "C" void DMA1_Channel2_3_IRQHandler() {
    dma_isr_entries = dma_isr_entries + 1u;
    if (dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    const bool host_ok = Host::init(clock, brio::I2cClock::pclk);

    bench.letter('a', "the block: the reserve, the reset values, PE's semantics, the "
                      "enable protection measured", ta_block);
    bench.letter('b', "THE LINK: write, read, the repeated START, the reserved addresses refused",
                 tb_link);
    bench.letter('c', "the vocabulary on the wire: nack_addr, nack_data, a staged "
                      "fault", tc_vocabulary);
    bench.letter('d', "the three speeds MEASURED, the Fm+ drive, and the ladder",
                 td_speeds);
    bench.letter('e', "clock stretching, priced - and NOSTRETCH's underrun", te_stretch);
    bench.letter('f', "10-bit addressing, the second address under its mask, ADDCODE",
                 tf_addressing);
    bench.letter('g', "the filters: the timing they really are", tg_filters);
    bench.letter('h', "RELOAD past 255, AUTOEND against a software STOP, the engines",
                 th_long);
    bench.letter('i', "SMBus: the PEC, and WHOSE hold each time-out polices",
                 ti_smbus);
    bench.letter('j', "the wake from Stop: the client woken by its own address",
                 tj_wake);
    bench.letter('k', "the stuck bus, and unstick() counting the clocks", tk_unstick);
    bench.letter('l', "THE KERNEL: I2cBus over I2cHost, and the per-bus timeout",
                 tl_kernel);
    bench.letter('m', "the errata: 2.10.1 as refusals, 2.10.2 as a count", tm_errata);
    bench.letter('u', "THE ENGINE NAMED: the default, interrupt, dma at one byte, the refusal",
                 tu_engine_choice);
    bench.letter('v', "A REFUSED WRITE at every position on either engine, and the next request "
                      "byte for byte", tv_refusals);
    bench.letter('B', "ES0418 2.11.6's band by name: the host's writes, and TXDR left empty, its "
                      "rate printed", tB_band);
    bench.letter('C', "THE RECEIVE CHANNEL AT THE STOPF ENTRY, staged: the last byte's request taken "
                      "away, and let up in the entry", tC_late_stage);
    bench.letter('D', "... and under load: every read, the channel's lag against the bus's window",
                 tD_late_load);
    bench.second_letter('n', "THE PEER: the twi_link command channel to the peer board",
                 tn_peer_link);
    bench.second_letter('o', "the tenure shapes against the peer, the repeated START "
                      "counted from the far end", to_peer_shapes);
    bench.second_letter('p', "the vocabulary against the peer, and commanded stretching",
                 tp_peer_vocabulary);
    bench.second_letter('q', "the three speeds against a second chip", tq_peer_speeds);
    bench.second_letter('r', "THE KERNEL against the peer: I2cBus over I2cHost, and the "
                      "wedge the peer holds", tr_peer_kernel);
    bench.second_letter('s', "TWO CONTROLLERS, ONE STOP: the held STARTs against the peer's",
                 ts_two_controllers);
    bench.second_letter('t', "the wake from Stop by ANOTHER CHIP's address, Sleep first",
                 tt_peer_wake);
    bench.letter('x', "a POLLED trace of one tenure, both ends by hand", tx_trace,
                 false);
    bench.letter('y', "the same tenure with the vectors LIVE, bounded and counted",
                 ty_isr_trace, false);

    if (serial_ok) {
        const auto idcode = brio::DeviceIdcode::read();
        print(serial, crlf, "part DEV_ID ", hex(idcode.dev_id),
              " REV_ID ", hex(idcode.rev_id), crlf);
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL 64 MHz" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " host=", host_ok ? "I2C1" : "FAILED", crlf);
        // THE TOPOLOGY IS ASKED OF THE WIRE, ONCE, before any letter can
        // run: which of the two instruments this desk is carrying is not
        // something the image may assume.
        self_link = probe_self_link();
        (void)Host::init(clock, brio::I2cClock::pclk);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
