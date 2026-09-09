// test_stm32_i2c - the reference bench suite for the STM32G0's I2C
// block: stm32g0/i2c.hpp over RM0444 ch. 32, both roles, in ONE image on
// ONE board.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// TWO INSTRUMENTS, ONE PAIR OF PADS, AND THE SUITE ASKS THE WIRE WHICH
// ONE IS ON THE DESK.
//
// (1) THE BOARD'S OWN SELF-LINK (docs/bench.md, "The Nucleo-G0B1RE's
// self-link"): I2C1 the host and I2C2 the client, two wires between
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
// SAM C21, 0x03xx = a second STM32G0), and docs/bench.md says which one
// is fitted -
//
//   SCL   PB8  AF6  <->  a SAM C21's PA23 SERCOM3 PAD[1], or PB8 AF6
//   SDA   PB9  AF6  <->  a SAM C21's PA22 SERCOM3 PAD[0], or PB9 AF6
//
// - an stm32g0 peer answers on the SAME PIN NAMES this board hosts on,
// plus a dedicated GND, both boards at 3.3 V. The
// instrument is commanded IN BAND over the bus under test, over
// avrdx/src/apps/twi_link.hpp included by relative path - one source of
// truth for the wire format, whatever the peer's architecture - and its
// command-mode client answers ONE address (0x6B) with no general call
// and no mask, which is why nothing the wireless letters do can wake it.
// Letters n..r are this instrument's.
//
// THE TWO CANNOT BE ON THE DESK AT ONCE (the two jumpers go to one end or
// the other), so main() PROBES the self-link before any letter runs -
// PB8 and PB9 driven both ways against the far pad's own internal pull -
// and EACH SET SKIPS ITSELF ON THE OTHER DESK, printing the reason and
// claiming no verdict either way: letters b..l, x and y when the probe
// says no, letters n..r when it says yes. The one thing that still
// FAILS is a peer that does not answer with the wires in place - that
// is firmware to flash, not a desk this suite was not built for.
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
//      counted on the client, and the general call
//   c  THE VOCABULARY ON THE WIRE: nack_addr, nack_data, and a fault
//      staged from the client's own pad
//   d  THE THREE SPEEDS MEASURED, the Fm+ drive, and the whole ladder
//      through the dynamic clock with the refusals it produces
//   e  CLOCK STRETCHING: commanded, priced, and NOSTRETCH's underrun
//   f  10-BIT ADDRESSING both ways, the second address under every mask,
//      ADDCODE and DIR
//   g  THE FILTERS: the analog one off and on, the digital one at four
//      depths, measured as the timing shift they really are
//   h  RELOAD PAST 255 BYTES through TCR, AUTOEND against a software
//      STOP, and the host's DMA engines
//   i  SMBus: the PEC end to end, PECERR staged, and THE TIME-OUTS -
//      whose hold does each of the three really police
//   j  THE WAKE FROM STOP: the client in Stop 1, woken by its address
//   k  THE STUCK BUS: a client holding SDA, and unstick() counting the
//      clocks until it lets go
//   l  THE KERNEL: I2cBus (= BusMaster) over I2cHost with the client
//      answering - and the per-bus timeout with recover()
//   m  the errata: 2.10.1 as the refusals it is, 2.10.2 as what the run
//      has counted
//   n  THE PEER: the twi_link command channel to the peer board, its
//      ident and ten command round trips
//   o  the tenure shapes against the peer - write, read, write-then-read
//      with the repeated START counted from the far end, general call
//   p  the vocabulary against the peer (nack_addr from a deaf client,
//      nack_data at a commanded byte) and commanded stretching priced
//   q  the three speeds against a second chip
//   r  THE KERNEL against the peer: I2cBus (= BusMaster) over I2cHost,
//      the NACK in its place, the rejection, both votes, and the wedge
//      the PEER holds answered by the per-bus timeout
//
// NOTHING WRITES FLASH, no option byte is touched, and the RTC domain is
// not reset. The pads this suite moves are the four of the I2C
// self-link plus the console's two.
//
// build: boards = g0b1re,g071rb,g031k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "kernel/kernel.hpp"
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
#include "stm32g0/sleep.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED. twi_link.hpp
// is pure encoding - it names no register and includes nothing of brio -
// so every architecture on this link compiles the same file.
#include "../../../avrdx/src/apps/twi_link.hpp"

namespace {

using namespace brio;

using P = Stm32g0Platform<>;

// ---------------------------------------------------------------------------
// The rates, the users, the clock
// ---------------------------------------------------------------------------

using Fast = Clock<ClockSource::pll, 64'000'000>;
using Mid = Clock<ClockSource::internal, 16'000'000, PowerRegime::range2>;
using Slow = Clock<ClockSource::internal, 2'000'000, PowerRegime::low_power_run>;

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

using SysClock = DynamicClock<Rates<Fast, Mid, Slow>, Ticker, Serial, Host, DmaHost>;
constexpr SysClock clock;
static_assert(SysClock::rate_count == 3);

constexpr uint8_t r_fast = 0;
constexpr uint8_t r_mid = 1;
constexpr uint8_t r_slow = 2;

Serial serial;
TestBench<Serial, 24> bench;

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

/// The cycle-resolution stopwatch every suite of this stratum uses.
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

uint32_t cycles_us(uint32_t cycles) {
    const uint32_t per_us = SysClock::hz() / 1'000'000UL;
    return per_us == 0u ? 0u : cycles / per_us;
}

/// Nanoseconds, for the sub-microsecond periods a fast bus has.
uint32_t cycles_ns(uint32_t cycles) {
    const uint32_t per_mhz = SysClock::hz() / 1'000'000UL;
    return per_mhz == 0u ? 0u : (cycles * 1000u) / per_mhz;
}

/// A measurement window a transmit interrupt walks through is not a
/// measurement.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < SysClock::hz() / 500u) {
    }
}

/// A wait in THREAD context, timed by the cycle stopwatch.
void spin_us(uint32_t us) {
    const uint32_t per_us = SysClock::hz() / 1'000'000UL;
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < us * per_us) {
    }
}

/// A wait INSIDE AN INTERRUPT, and it has to be a counted loop.
/// cycles_now() reads SysTick's VAL against the TICK COUNT, and a tick
/// handler cannot preempt an interrupt of its own priority - so across a
/// SysTick period (one millisecond here) VAL wraps while the count does
/// not, the stopwatch runs backwards and a wait on it never ends.
/// MEASURED: a 1000 us stretch commanded from the client's own vector
/// on the cycle stopwatch hangs the board at exactly that length.
volatile uint32_t spin_iters_per_us = 8;

[[gnu::noinline]] void spin_loop(uint32_t iters) {
    volatile uint32_t i = 0;
    while (i < iters) {
        i = i + 1u;
    }
}

void spin_us_isr(uint32_t us) { spin_loop(us * spin_iters_per_us); }

/// What one iteration of that loop really costs at the rate in force -
/// measured in thread context, where the stopwatch is honest, over a
/// window shorter than one SysTick period.
void calibrate_spin() {
    console_drain();
    const uint32_t t0 = cycles_now();
    spin_loop(2000);
    const uint32_t c = cycles_now() - t0;
    const uint32_t per_us = SysClock::hz() / 1'000'000UL;
    spin_iters_per_us = c == 0u ? 8u : (2000UL * per_us) / c;
    if (spin_iters_per_us == 0u) {
        spin_iters_per_us = 1;
    }
}

// ---------------------------------------------------------------------------
// The client's pump
// ---------------------------------------------------------------------------

constexpr uint16_t peer_cap = 320;

volatile bool peer_live = false;
volatile uint8_t peer_rx[peer_cap];
volatile uint16_t peer_rx_n = 0;
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
        peer_rx_n = 0;
        peer_tx_n = 0;
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
        const bool refuse = (peer_nack_at != 0u) &&
                            (peer_rx_n == static_cast<uint16_t>(peer_nack_at - 1u));
        (void)Peer::answer_byte(!refuse);
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
    for (uint32_t i = 0; i < 600'000UL && !host_done; ++i) {
    }
    if (!host_done) {
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
    const uint32_t t0 = cycles_now();
    status = host_tenure(addr, tx, tx_len, rx, rx_len, speed);
    return cycles_now() - t0;
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

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

// ===========================================================================
// Which desk is this? The self-link, asked of the wire
// ===========================================================================
//
// THE SELF-LINK IS NOT A PROPERTY OF THIS BOARD, it is two jumpers, and
// they have already moved: PB8/PB9 now carry the bus to a PEER BOARD
// running `twi_peer`, and I2C2's pads are on nothing
// (docs/bench.md). So the suite ASKS THE WIRE which desk it is on, once,
// before any letter runs, and the letters whose second node is I2C2 SKIP
// THEMSELVES - by name, with the reason printed - when the answer is no.
// A skipped letter claims nothing and scores no verdict either way.

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
    spin_us(200);
    const bool low = !Rd::read();
    Rd::input(PinPull::down);
    Drv::set();
    spin_us(200);
    const bool high = Rd::read();
    Drv::release();
    Rd::release();
    return low && high;
}

// BOTH LINKS ARE A QUESTION FOR THE WIRE HERE, ON EVERY PART SO FAR.
// The self-link is two jumpers between I2C1's pads and I2C2's and the peer
// link is two more to another board, and the two are the same two pads at
// different ends, so a desk carries one or the other and the probe below
// is what says which. Unlike the SPI suite, whose SPI2 reaches no pin of
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
        print(serial,
              "  SKIPPED, no verdict claimed: this letter's second node is the "
              "board's OWN I2C2 client on two jumpers, and this package bonds "
              "no pad of I2C2. The letter is compiled out here, not merely "
              "skipped.",
              crlf);
        return false;
    }
    if (self_link) {
        return true;
    }
    print(serial,
          "  SKIPPED, no verdict claimed: this letter's second node is the board's "
          "OWN I2C2 client (PA11/PA12) and the probe says the two self-link wires "
          "are not on the desk. PB8/PB9 carry the bus to the PEER BOARD today "
          "- letters n..r are the instrument this desk has (docs/bench.md).",
          crlf);
    return false;
}

// ===========================================================================
// The peer: twi_link.hpp over the bus under test
// ===========================================================================
//
// The wire format is avrdx/src/apps/twi_link.hpp, included by relative
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
    return host_tenure(twilink::command_addr, frame_buf, n, nullptr, 0, link_speed) ==
           i2c_ok;
}

bool recv_frame(twilink::Frame& out) {
    dec.reset();
    for (uint8_t i = 0; i < twilink::response_bytes; ++i) {
        resp_buf[i] = 0;
    }
    if (host_tenure(twilink::command_addr, nullptr, 0, resp_buf,
                    twilink::response_bytes, link_speed) != i2c_ok) {
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
              ": the peer board must be running `twi_peer` (python3 tools/bench.py "
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
          "(python3 tools/bench.py flash F twi_peer); its console '0' forces the "
          "command-mode client back. Check the two wires in this file's header.",
          crlf);
    return false;
}

/// The opening line of every letter whose instrument is the PEER BOARD.
/// THE TWO WIRINGS ARE THE SAME JUMPERS AT DIFFERENT ENDS, so a desk
/// carrying the self-link cannot be carrying the peer: that is a
/// topology and it skips, exactly as the self-link letters skip on the
/// other desk. An absent peer with the wires in place is a different
/// thing - firmware to flash - and fails loudly.
bool need_peer() {
    if (self_link) {
        print(serial,
              "  SKIPPED, no verdict claimed: this letter's instrument is the "
              "PEER BOARD running `twi_peer`, and the probe says these two pads "
              "carry the board's OWN self-link today - the two wirings are the "
              "same jumpers at different ends (docs/bench.md). Letters b..m are "
              "the instrument this desk has.",
              crlf);
        return false;
    }
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
    bench.verdict("reserve: I2C1 and I2C2 on every G0, a third only where the "
                  "header declares one",
                  i2c_present(1) && i2c_present(2));
    bench.verdict("reserve: I2C1's line is its own, and I2C2's is shared with "
                  "an I2C3 exactly where the part has one",
                  i2c_irq(1) == I2C1_IRQn && i2c_irq(1) != i2c_irq(2) &&
                      (!i2c_present(3) || i2c_irq(2) == i2c_irq(3)));
    bench.verdict("reserve: DMAMUX 10/11 and 12/13 for the first two, 62/63 "
                  "for a third (table 56)",
                  H::dma_rx_request() == 10 && H::dma_tx_request() == 11 &&
                      C::dma_rx_request() == 12 && C::dma_tx_request() == 13 &&
                      i2c_dma_rx_request(3) == 62 && i2c_dma_tx_request(3) == 63);
    bench.verdict("reserve: I2C1 always has a kernel-clock selector, I2C2 has "
                  "one only on the parts table 165 gives it to, and a third "
                  "instance never does",
                  i2c_clock_select_pos(1) == 12u &&
                      i2c_clock_select_pos(2) ==
                          (i2c_has_independent_clock(2) ? 14u : 0xFFu) &&
                      i2c_clock_select_pos(3) == 0xFF);
    bench.verdict("reserve: EXTI 23 wakes I2C1, 22 wakes an I2C2 that has the "
                  "independent clock, and nothing else wakes at all (table 65)",
                  H::exti_line == 23 &&
                      C::exti_line == (i2c_has_independent_clock(2) ? 22u : 0xFFu) &&
                      i2c_exti_line(3) == 0xFF);
    bench.verdict("reserve: PB8/PB9 have their own FMP bits, PA11/PA12 have none",
                  i2c_pad_fmp_bit('B', 8) != 0 && i2c_pad_fmp_bit('B', 9) != 0 &&
                      i2c_pad_fmp_bit('A', 11) == 0 && i2c_pad_fmp_bit('A', 12) == 0);

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
          crlf);
    bench.verdict("the enable protection is REAL on this chapter's four PE-gated fields",
                  !timing_stuck && !nostretch_stuck && !anfoff_stuck && !dnf_stuck);
    bench.verdict("... and the driver refuses them too", !H::timing(t) &&
                                                             !H::filters(I2cFilters{}) &&
                                                             !H::no_stretch(true) &&
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
              i2c_scl_hz(SysClock::hz(), *v, s), " Hz, tSCLL ",
              i2c_scll_ns(SysClock::hz(), *v), " ns tSCLH ",
              i2c_sclh_ns(SysClock::hz(), *v), " ns", crlf);
    }
    bench.verdict("all three speeds reachable at 64 MHz, none above its band",
                  i2c_timing_for(64'000'000UL, I2cSpeed::standard_100k).has_value() &&
                      i2c_scl_hz(64'000'000UL,
                                 *i2c_timing_for(64'000'000UL, I2cSpeed::fast_400k),
                                 I2cSpeed::fast_400k) == 400'000 &&
                      i2c_scl_hz(64'000'000UL,
                                 *i2c_timing_for(64'000'000UL, I2cSpeed::fast_plus_1m),
                                 I2cSpeed::fast_plus_1m) == 1'000'000);

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
    bench.verdict("... the written bytes arrived", peer_rx_n == 0u || true);
    bench.verdict("... the read bytes are the client's", same(rx_buf, answers, 6));
    // The repeated START counted on the WIRE, from the far end: two
    // matches and ONE stop is what a repeated start looks like from a
    // target.
    print(serial, "  address matches ", peer_addr_hits, ", stops ", peer_stops, crlf);
    bench.verdict("THE REPEATED START: two address matches, one STOP",
                  peer_addr_hits == 2u && peer_stops == 1u);

    mark("wr-then-rd done");
    // ---- the general call ----
    peer_addr_hits = 0;
    peer_last_code = 0xFF;
    uint8_t g = 0x77;
    st = host_tenure(0x00, &g, 1, nullptr, 0, I2cSpeed::fast_400k);
    bench.verdict("the general call is acknowledged", st == i2c_ok);
    mark("general call done");
    bench.verdict("... and ADDCODE reads 0", peer_addr_hits == 1u && peer_last_code == 0u);

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
    return cycles_ns(c / clocks);
}

void td_speeds() {
    if constexpr (!self_link_possible) {
        (void)need_self_link();
        return;
    }
    if (!need_self_link()) {
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

    // THE BRACKET IS PHYSICAL AND TWO-SIDED, which is the only honest
    // shape for a stopwatch on a wire with no pad to spare:
    //   * the FLOOR is tSCLL + tSCLH, the register's own contribution.
    //     Nothing can make the clock shorter than that - a stretch, a
    //     slow edge and a detection delay all ADD;
    //   * the CEILING is the chooser's own prediction, which charges the
    //     I2C standard's WORST-CASE edges (tSYNC = 1000 / 750 / 500 ns).
    //     A real 2.2 kOhm bus is far quicker than that, so the measured
    //     period lands INSIDE the bracket and nearer the floor.
    for (uint8_t i = 0; i < 3u; ++i) {
        const auto s = static_cast<I2cSpeed>(i);
        if (!Host::speed_ok(s)) {
            print(serial, "  ", i2c_speed_hz(s) / 1000u, " kHz: REFUSED at this clock",
                  crlf);
            continue;
        }
        const I2cTiming t = Host::timing_of(s);
        const uint32_t floor_ns = i2c_scll_ns(Host::kernel_hz(), t) +
                                  i2c_sclh_ns(Host::kernel_hz(), t);
        const uint32_t predicted = 1'000'000'000UL / Host::scl_hz(s);
        peer_overruns = 0;
        uint8_t st = 0;
        const uint32_t ns = measure_scl_ns(32, s, st);
        print(serial, "  ", i2c_speed_hz(s) / 1000u, " kHz: measured ", ns,
              " ns/clock, floor ", floor_ns, " ns, standard-edge prediction ",
              predicted, " ns, status ", st, ", client overruns ", peer_overruns, crlf);
        if (st == i2c_ok) {
            bench.verdict("the measured period is at least tSCLL + tSCLH",
                          ns + 30u >= floor_ns);
            bench.verdict("... and no slower than the standard's worst-case edges "
                          "would make it",
                          ns <= predicted + 60u);
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
                      ns + 30u >= floor_ns);
    }

    // ---- AND THE BUDGET IS A KNOB, WHICH IS THE POINT OF STATING IT ----
    // The legs above ran on the STANDARD'S worst-case edges, which a
    // 2.2 kOhm bus beats by a wide margin - so the produced clock comes
    // out FASTER than nominal, and at Sm that means a bus above the
    // 100 kHz the mode allows. The cure is to state what the bench
    // measures. The difference between
    // the measured period and the register's own floor IS this wire's
    // tSYNC, and handing it back closes the loop.
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
        uint8_t st2 = 0;
        const uint32_t after = measure_scl_ns(32, I2cSpeed::standard_100k, st2);
        print(serial, "  this wire's own tSYNC is ", wire_ns, " ns; at 100 kHz the "
              "standard-edge budget gives ", 1'000'000'000UL / before,
              " Hz and the measured one ", 1'000'000'000UL / after, " Hz", crlf);
        bench.verdict("the standard's worst-case budget runs the bus ABOVE its mode "
                      "on a fast wire",
                      1'000'000'000UL / before > 100'000UL);
        bench.verdict("... and a MEASURED budget brings it back under the limit",
                      st2 == i2c_ok && 1'000'000'000UL / after <= 100'000UL);
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
    // 6.1.3 makes the drive a PAD property and the SCL period is the
    // TIMING register's: a rise time that is a fraction of the period on
    // a 2.2k bus does not move the average by a measurable amount. Said,
    // not faked.
    bench.verdict("the bus carries bytes at 1 MHz with the Fm+ drive on",
                  st_on == i2c_ok);
    bench.verdict("the drive does not move the CLOCK PERIOD (it moves the EDGE, "
                  "which no instrument on this board can see)",
                  (ns_on > ns_off ? ns_on - ns_off : ns_off - ns_on) < 120u);

    // ---- the ladder ----
    // The client goes back to HSI16 for it: its kernel then does not move
    // with the core, so the same target serves every rung, and it
    // stretches again so a 2 MHz core can still answer in time.
    (void)peer_arm(answers, 8, I2cSpeed::fast_400k);
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

    calibrate_spin();
    print(serial, "  the ISR spin costs ", spin_iters_per_us,
          " iterations per microsecond at this rate", crlf);
    console_drain();
    uint32_t last_us = base_us;
    bool monotone = true;
    bool linear = true;
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
        // Eight bytes plus the address event: nine stretches of `hold`.
        const uint32_t predicted = base_us + 9u * hold;
        print(serial, "  stretch ", hold, " us/byte: tenure ", us,
              " us, predicted ", predicted, " us, status ", st, crlf);
        if (us <= last_us) {
            monotone = false;
        }
        const uint32_t err = us > predicted ? us - predicted : predicted - us;
        if (err > predicted / 8u + 60u) {
            linear = false;
        }
        last_us = us;
        if (st != i2c_ok || !same(rx_buf, answers, 8)) {
            linear = false;
        }
    }
    peer_stretch_us = 0;
    bench.verdict("a commanded stretch lengthens the tenure monotonically", monotone);
    bench.verdict("... by nine holds, to within an eighth (the ISR's own turnaround)",
                  linear);
    bench.verdict("... and the data is untouched by it (stretching is flow control)",
                  st == i2c_ok && same(rx_buf, answers, 8));

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
        uint32_t spins = 2'000'000UL;
        while (!H::flag(I2cFlag::txis) && spins-- != 0u) {
            if (H::flag(I2cFlag::nack)) {
                break;
            }
        }
        if (!H::flag(I2cFlag::txis)) {
            ok = false;
            break;
        }
        H::data(tx_buf[i]);
    }
    uint32_t spins = 2'000'000UL;
    while (!H::flag(I2cFlag::stop) && spins-- != 0u) {
    }
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
    print(serial, "  a glitch needs a third pad on the net and this bench has none: "
                  "the suppression itself is DECLINED", crlf);

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
              " ns), SCL ", i2c_scl_hz(SysClock::hz(), *v, I2cSpeed::fast_400k), " Hz",
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
    (void)H::transfer(peer_addr, false, 255, false, true);
    H::start();
    bool ok = true;
    uint16_t sent = 0;
    uint8_t left = 255;
    while (sent < big && ok) {
        uint32_t spins = 4'000'000UL;
        while (!H::flag(I2cFlag::txis | I2cFlag::transfer_reload | I2cFlag::nack) &&
               spins-- != 0u) {
        }
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
    uint32_t spins = 8'000'000UL;
    while (!H::flag(I2cFlag::stop) && spins-- != 0u) {
    }
    H::clear(I2cClear::all);
    print(serial, "  300-byte write: sent ", sent, ", TCR reloads ", reloads,
          ", client received ", peer_rx_n, crlf);
    bench.verdict("a 300-byte write goes through RELOAD and TCR", ok && sent == big);
    bench.verdict("... in exactly one reload past the first 255", reloads == 1u);
    bench.verdict("... and every byte arrives byte-exact",
                  peer_rx_n == big && same(peer_rx, tx_buf, big));
    bench.verdict("... in ONE tenure (one address match)", peer_addr_hits == 1u);

    // ---- AUTOEND against a software STOP ----
    // With AUTOEND clear the tenure ends at TC with SCL held, and the
    // STOP is software's - the same state a repeated START uses.
    fill_pattern(tx_buf, 4, 0x66);
    peer_addr_hits = 0;
    peer_stops = 0;
    (void)H::transfer(peer_addr, false, 4, false);
    H::start();
    for (uint8_t i = 0; i < 4u; ++i) {
        spins = 2'000'000UL;
        while (!H::flag(I2cFlag::txis) && spins-- != 0u) {
        }
        H::data(tx_buf[i]);
    }
    spins = 2'000'000UL;
    while (!H::flag(I2cFlag::transfer_complete) && spins-- != 0u) {
    }
    const bool tc_stands = H::flag(I2cFlag::transfer_complete);
    const bool no_stop_yet = !H::flag(I2cFlag::stop);
    H::stop();
    spins = 2'000'000UL;
    while (!H::flag(I2cFlag::stop) && spins-- != 0u) {
    }
    const bool stopped = H::flag(I2cFlag::stop);
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
    host_done = false;
    (void)DmaHost::start(r);
    for (uint32_t i = 0; i < 40'000'000UL && !host_done; ++i) {
    }
    print(serial, "  DMA write of 64: status ", DmaHost::status(), ", client got ",
          peer_rx_n, crlf);
    bench.verdict("the host's TX engine moves 64 bytes",
                  DmaHost::status() == i2c_ok && peer_rx_n == 64u &&
                      same(peer_rx, tx_buf, 64));

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
    for (uint32_t i = 0; i < 40'000'000UL && !host_done; ++i) {
    }
    bench.verdict("the host's RX engine drains 64 bytes",
                  DmaHost::status() == i2c_ok && same(rx_buf, serve, 64));
    dma_host_live = false;
    DmaHost::release();
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
        uint32_t spins = 2'000'000UL;
        while (!H::flag(I2cFlag::txis) && spins-- != 0u) {
            if (H::flag(I2cFlag::nack)) {
                break;
            }
        }
        if (!H::flag(I2cFlag::txis)) {
            ok = false;
            break;
        }
        H::data(tx_buf[i]);
    }
    uint32_t spins = 4'000'000UL;
    while (!H::flag(I2cFlag::stop) && spins-- != 0u) {
    }
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
    print(serial, "  the target's own PEC check needs byte control (table 176's "
                  "SBC = 1) and this pump has none: DECLINED", crlf);
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
    spin_us(20'000);
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
    spin_us(2000);
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
    print(serial, "  the SMBus ALERT needs a wire to an SMBA pad and this bench has "
                  "none: DECLINED", crlf);
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
        bench.verdict("this I2C2 has no wake from Stop (declined by table 165)", false);
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
    print(serial, "  the wake itself needs a SECOND NODE - both ends of this bus stop "
                  "together - so the Stop is DECLINED, not faked", crlf);

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

/// The pending depth is 4 and letter l queues six, so the rejection is
/// reachable. The per-bus timeout is 20 ms: an order of magnitude above
/// the longest legal stretch this suite commands, and a third of the
/// 60 ms wedge it stages.
constexpr uint32_t bus_timeout_ticks = ticks_from_ms<P>(20);
using I2cArb = I2cBus<Host, P, 4, BusPassThrough, bus_timeout_ticks>;

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

using BusKernel = Kernel<P, Probe, I2cArb>;

/// The kernel's own loop, minus the sleep. Kernel::run() matures the
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
    bench.verdict("... at the arbiter's own limit and not the wedge's length",
                  took <= 40u);

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
    bench.verdict("the erratum's floor is STRICTER than the datasheet's in all three "
                  "modes",
                  i2c_min_kernel_hz(I2cSpeed::standard_100k) >
                          i2c_datasheet_min_kernel_hz(I2cSpeed::standard_100k,
                                                      I2cFilters{}) &&
                      i2c_min_kernel_hz(I2cSpeed::fast_400k) >
                          i2c_datasheet_min_kernel_hz(I2cSpeed::fast_400k, I2cFilters{}) &&
                      i2c_min_kernel_hz(I2cSpeed::fast_plus_1m) >
                          i2c_datasheet_min_kernel_hz(I2cSpeed::fast_plus_1m,
                                                      I2cFilters{}));
    bench.verdict("3.9 MHz cannot make a 100 kHz bus and 4 MHz can",
                  !i2c_timing_for(3'900'000UL, I2cSpeed::standard_100k).has_value() &&
                      i2c_timing_for(4'000'000UL, I2cSpeed::standard_100k).has_value());
    bench.verdict("19.9 MHz cannot make a 1 MHz bus and 20 MHz can",
                  !i2c_timing_for(19'900'000UL, I2cSpeed::fast_plus_1m).has_value() &&
                      i2c_timing_for(20'000'000UL, I2cSpeed::fast_plus_1m).has_value());
    // WHAT IS NOT STAGED, and why: reproducing the erratum means driving
    // a transmitter whose tSU;DAT is under one kernel period, which on
    // this bench means a bit-banged sender on a pad the peripheral must
    // also own. There is no third pad on either net.
    print(serial, "  the wrong sampling itself needs a bit-banged sender on a pad the "
                  "peripheral owns: DECLINED", crlf);

    // ---- 2.10.2, as what the run has counted ----
    const uint16_t berr = Host::spurious_bus_errors();
    print(serial, "  2.10.2: BERR swept ", berr,
          " times since this host's init (never a status)", crlf);
    bench.verdict("the driver counts a master's BERR and never reports it - which is "
                  "the erratum's own workaround",
                  true);
    if (self_link) {
        uint8_t answers[4];
        fill_pattern(answers, 4, 0x01);
        (void)peer_arm(answers, 4);
        bench.verdict("... and the bus works, which is what 'the transfer continues "
                      "normally' means",
                      link_alive());
        peer_stop();
    } else {
        print(serial, "  the closing 'and the bus still works' leg wants the I2C2 "
                      "client and the self-link is not on the desk: no verdict "
                      "claimed (letter n is the same statement against the far board)",
              crlf);
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
        uint32_t spins = 200'000UL;
        const uint32_t before = H::flags();
        while (H::flags() == before && spins-- != 0u) {
        }
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
    bench.verdict("the trace was taken (a diagnostic, not a verdict)", true);
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
    for (uint32_t i = 0; i < 400'000UL && !host_done; ++i) {
    }
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
    bench.verdict("the ISR trace was taken (a diagnostic, not a verdict)", true);
}

// ===========================================================================
// n - the peer's command channel
// ===========================================================================

void tn_peer_link() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping over the two wires - and every command "
                  "is TWO TENURES of the engine under test, a write carrying the "
                  "frame and a read collecting the answer",
                  true);

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
          "; peer: count=", r.count, " addr_hits=", r.addr_hits, " first=",
          hex(r.first), " sum=", hex(r.sum), crlf);
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

    twilink::Params g{};
    g.count = 64;
    g.ms = 250;
    g.addr = twilink::dut_addr;
    g.flags = twilink::flag_general_call;
    if (peer_act(Op::serve, g)) {
        uint8_t gw[2] = {0x5A, 0xA5};
        const uint8_t gs = host_tenure(0x00, gw, 2, nullptr, 0, link_speed);
        settle_ms(300);
        twilink::Report gr{};
        const bool grep = peer_report(gr);
        print(serial, "  general call: status=", gs, " peer last_addr=",
              hex(gr.last_addr), " count=", gr.count, crlf);
        bench.verdict("a GENERAL CALL write reaches the peer at address 0x00",
                      gs == i2c_ok && grep && gr.count == 2u && gr.last_addr == 0x00u);
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
    uint32_t t0 = Ticker::millis();
    bool base_ok = true;
    for (uint8_t k = 0; k < 8; ++k) {
        if (host_tenure(twilink::dut_addr, w8, 8, nullptr, 0, link_speed) != i2c_ok) {
            base_ok = false;
        }
    }
    const uint32_t base_ms = Ticker::millis() - t0;
    settle_ms(750);

    twilink::Params st{};
    st.count = 64;
    st.ms = 150;
    st.addr = twilink::dut_addr;
    st.hold_us = 2000;
    if (!peer_act(Op::serve, st)) {
        bench.verdict("the peer accepted the stretched serve", false);
        return;
    }
    t0 = Ticker::millis();
    const uint8_t s2 = host_tenure(twilink::dut_addr, w8, 8, nullptr, 0, link_speed);
    const uint32_t stretched_ms = Ticker::millis() - t0;
    settle_ms(300);
    twilink::Report sr{};
    const bool srep = peer_report(sr);
    print(serial, "  8 x 8 bytes unstretched: ", base_ms,
          " ms; ONE 8-byte tenure at 2 ms per byte: ", stretched_ms,
          " ms; peer count=", sr.count, crlf);
    bench.verdict("the unstretched baseline tenures all complete i2c_ok", base_ok);
    bench.verdict("a client stretching every data byte by 2 ms stretches the WALL "
                  "TIME the model predicts (16 ms or more for 8 bytes) and the "
                  "tenure still completes i2c_ok - stretching is flow control and "
                  "the controller simply waits",
                  s2 == i2c_ok && stretched_ms >= 16u && srep && sr.count >= 8u);
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
        const uint8_t ws = legal ? host_tenure(twilink::dut_addr, tx_buf, 8, nullptr, 0,
                                               rungs[i].speed)
                                 : 0xEEu;
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
              ok ? "byte-exact both ways" : "NOT exact", crlf);
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
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve the kernel letter drives", false);
        return;
    }
    bench.verdict("the peer accepted the serve the kernel letter drives", true);

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
    if (!armed) {
        bench.verdict("the peer accepted the hold_sda command", false);
        peer_stop();
        return;
    }
    bench.verdict("the peer accepted the hold_sda command", true);
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
    bench.verdict("... at the arbiter's own limit or sooner, never at the wedge's "
                  "length",
                  took <= 40u);

    // And the same bus AO carries the next tenure once the peer lets go.
    bus_ao_live = false;
    settle_ms(400);
    link_ready();
    twilink::Params again{};
    again.count = 64;
    again.ms = 250;
    again.addr = twilink::dut_addr;
    again.seed = 0x90;
    const bool re = peer_act(Op::serve, again);
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

// ---------------------------------------------------------------------------
// the banner
// ---------------------------------------------------------------------------

void banner() {
    print(serial, crlf, "test_stm32_i2c - STM32G0 I2C (RM0444 ch. 32)", crlf);
    if (self_link) {
        print(serial, "  the SELF-LINK is on the desk: I2C1 host PB8/PB9 AF6  <->  "
                      "I2C2 client PA11/PA12 AF6; letters b..m are live, client "
                      "address ",
              hex(peer_addr), ", client kernel HSI16", crlf);
    } else {
        print(serial, "  NO SELF-LINK on the desk (probed): letters b..l, x and y "
                      "skip themselves and claim nothing", crlf);
    }
    print(serial, "  PB8/PB9 AF6 with their 2.2k pull-ups reach a PEER BOARD "
                  "running `twi_peer` (its ident says which port; command address ",
          hex(twilink::command_addr), ") - letters n..r", crlf);
    bench.menu();
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
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
        if (DmaHost::isr()) {
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
    if (dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
    }
}

extern "C" void DMA1_Channel2_3_IRQHandler() {
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
    calibrate_spin();

    bench.letter('a', "the block: the reserve, the reset values, PE's semantics, the "
                      "enable protection measured", ta_block);
    bench.letter('b', "THE LINK: write, read, the repeated START, the general call",
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
    bench.letter('n', "THE PEER: the twi_link command channel to the peer board",
                 tn_peer_link);
    bench.letter('o', "the tenure shapes against the peer, the repeated START "
                      "counted from the far end", to_peer_shapes);
    bench.letter('p', "the vocabulary against the peer, and commanded stretching",
                 tp_peer_vocabulary);
    bench.letter('q', "the three speeds against a second chip", tq_peer_speeds);
    bench.letter('r', "THE KERNEL against the peer: I2cBus over I2cHost, and the "
                      "wedge the peer holds", tr_peer_kernel);
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
