// test_avr_sleep - the SLEEP test SUITE for the AVR DA/DB target: the
// three sleep modes of SLPCTRL (avrxt/sleep.hpp) and the voltage
// regulator's sleep profile, measured on the silicon instead of read
// off tables 13-2..13-5 (which errata DS80000915F 3.4.1 rewrites).
//
// What it proves. The register surface and its two enforced rules (the
// erratum 2.2.4 NOP before every CTRLA store, and HTLLEN refused while
// a TWI client or the CCL could still try to wake the device). That
// IDLE through Sleep::enter() behaves like the platform's own idle().
// That STANDBY really stops CLK_PER and that the PIT still wakes from
// it. WHICH RUNSTDBY DECIDES: two different bits carry that name, a
// peripheral's and each oscillator's, and the matrix of test d sweeps
// both against two clock sources - the PERIPHERAL's is the one that
// keeps a counter alive (its request revives the whole clock chain),
// the OSCILLATOR's only buys the wake-up time test g then measures.
// That
// POWER-DOWN stops even the RTC counter while its PIT half keeps
// running, and that a PORT pin wakes from both deep modes with the
// pad driven by the device's own event system (no wires: EVSYS is
// alive in every sleep mode, so a PIT divider routed to EVOUT on PD2
// is a wake-up source the board builds for itself).
//
// The TWO-BOARD half (set y) measures what one board cannot: HOW LONG a
// wake-up takes. A sleeping chip cannot time its own return - the only
// clock power-down leaves running is the PIT's, and the counter that
// would measure the restart is exactly the one the mode stops (test f).
// So the ruler moves off-chip: the desk partner, a SAM C21 running
// samc21/src/apps/sleep_peer.cpp (src/apps/sleep_peer.cpp is an AVR
// peer speaking the same sleep_link.hpp), drives a stimulus edge at a
// known count of a 32-bit stopwatch of its 24 MHz crystal and CAPTURES
// that stopwatch in hardware when this board's wake-up ISR answers with
// an edge of its own. Tests h..m then time idle,
// standby and power-down against every clock configuration that
// matters, wake this board on a start bit and on a TWI address match,
// and read the numbers back over the same link.
//
// Reference test of avrxt/sleep.hpp (docs/avrxt/platform.md): keep it
// passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800. This suite OWNS the RTC block: no
// Ticker runs here, because the PIT period is the instrument (125 ms)
// and the counter half is one of the things measured - which is also
// why every bound in the two-board half is a spin count and not a
// millisecond.
//
// WIRING. The single-board half (z) needs NOTHING beyond the desk's
// standing state - but PD1/PD2 MUST BE FREE of the bus jumpers, because
// test e drives PD2 from the event system and senses its own edge on
// the pad. The two-board half (y) needs the peer board running
// sleep_peer and this wiring - the desk's link to the SAM C21: PE0 the
// shared one-wire command channel (this board's USART4 TXD pad under
// LBME, the SAM's PA16), PE2 the peer's stimulus into this board (PA19),
// PE3 this board's echo into the peer's capture (PA17), and the I2C bus
// on PA2/PA3 (the SAM's PA22/PA23) for test m. PE1 is spare. Event channels 0 (the PIT divider
// to the pad), 1 (the PIT divider into the CCL, test n), 4 and 5 (the
// 32-bit stopwatch's carry and snapshot).
//
// THE TWO PARTS. The DB's main clock is a crystal on XOSCHF, with a
// start-up of its own and a RUNSTDBY that keeps it - and the regulator -
// up in standby. The DA's is an external clock on PA0: an input with no
// start-up and no RUNSTDBY, so a wake on it pays the regulator alone, and
// the one oscillator a DA can hold up in standby is OSCHF. Every "crystal
// with its RUNSTDBY" leg is therefore, on a DA, the external clock with
// OSCHF's RUNSTDBY set or clear (use_crystal()), and the verdicts that
// judge a crystal's start-up judge the regulator's there instead; each
// leg's label says which part's meaning it carries.
//
// Tests e, f, k, m and n enter POWER-DOWN. Every such sleep is bracketed
// by a watchdog and a .noinit token, so a mode that fails to wake resets
// the board and SAYS SO at the next boot instead of hanging the bench.
//
// THE AVR EA (DS40002443A ch. 13). Its SLPCTRL is CTRLA alone: the same
// three modes and no regulator control, so every leg that sets PMODE or
// HTLLEN is the DA/DB's alone, and what the EA's legs ask instead is
// whether its wake pays a regulator bill the chapter does not list
// (13.3.2.2 names one, table 13-4 does not). Every EA pin senses fully
// asynchronously (18.3.3.1): letter p wakes the device from power-down on
// ONE edge of each pin a self-made edge reaches (EVOUT and the CCL's
// outputs, driven by the PIT through the event system), and letter q on
// the pins the partner board's wires reach. The EA has no USART4 and no
// USART reaches PE0, so the link is USART1 on ALT1: its TXD PC4 is
// jumpered onto PE0 (and PC5..PC7 onto PE1..PE3, avr_board.hpp's
// board_pc_pe_jumpers), the stimulus and the echo stay on PE2/PE3 and
// their PC partners stay inputs. No ADC (its EA engine is not written),
// no TCD and no op amp: letter o keeps its TCA and AC legs.
//
// Commands: ? | a the register surface | b idle through enter() |
// c standby is real | d the RUNSTDBY chain | e a pin wakes from the
// deep modes | f power-down stops the rest | g the voltage regulator
// (the AVR EA: the wake-up in 32 kHz ticks) | n the CCL as a wake-up
// source | o RUNSTDBY unit by unit | p (the AVR EA) every self-driven pin
// wakes from power-down on one edge | z = all of those
//   two-board (the peer runs sleep_peer): h the link and the two wires |
//   i the awake baseline and idle | j standby, timed from outside |
//   k power-down, timed from outside | l start-of-frame | m a TWI
//   address match | q (the AVR EA) the peer's edges on more pins |
//   y = all of h..m (and q)

// THE AVR64EA48 builds the suite as two images, held to the ATmega4809's
// 48 KB (the groups line below; docs/avrxt/README.md, "The group axis"):
// the single-board set z in one, the two-board set y in the other; the
// 128 KB DA and DB build one.
//
// build: boards = db48,da48,ea48
// build: groups = abcdefgnop,hijklmq
// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrxt/ac.hpp"
#if !defined(BRIO_BOARD_ea48)
#include "avrxt/adc.hpp"
#endif
#include "avrxt/ccl.hpp"
#include "avrxt/clock.hpp"
#include "avrxt/delay.hpp"
#include "avrxt/evsys.hpp"
#if !defined(BRIO_BOARD_ea48)
#include "avrxt/opamp.hpp"
#endif
#include "avrxt/pin.hpp"
#include "avrxt/platform.hpp"
#include "avrxt/reset.hpp"
#include "avrxt/rtc.hpp"
#include "avrxt/sleep.hpp"
#include "avrxt/tca.hpp"
#include "avrxt/tcb.hpp"
#if !defined(BRIO_BOARD_ea48)
#include "avrxt/tcd.hpp"
#endif
#include "avrxt/twi.hpp"
#include "avrxt/usart.hpp"
#include "avrxt/userrow.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#include "sleep_link.hpp"

#include "avr_board.hpp"

using SysClock = BoardClock;   // the board's HF source (avr_board.hpp)
constexpr SysClock clock;

/// The escape hatch of the two power-down tests. A mode that does not
/// wake is caught by the watchdog, and this .noinit word is what tells
/// the next boot which sleep swallowed the program.
///
/// `inline` on purpose: gcc gives an inline variable with a section
/// attribute a COMDAT group and a plain one none, and the two section
/// types cannot merge - AvrPlatform's own panic_record_ is a static
/// inline member in .noinit, so this must be inline too or the link
/// fails.
struct Token {
    uint16_t magic;
    uint8_t stage;      ///< which sleep was in flight (0 = none)
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using P = AvrPlatform;
using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

constexpr uint16_t token_magic = 0x51EE;

// ---- the instruments -----------------------------------------------------------
// A 32-bit CLK_PER counter (a TCB cascade) is the "did the main clock
// run?" probe; the RTC counter is the "did the 32 kHz domain run?" one;
// the PIT is the waker and, at 4096 cycles of CLK_RTC, the wall clock:
// one period = 125 ms = 3'000'000 CLK_PER ticks at 24 MHz.
using Waker = Tcb<0>;                ///< a wake source that is NOT the PIT
using WatchLo = Tcb<1>;
using WatchHi = Tcb<2>;
using Watch = CascadedCounter<WatchLo, WatchHi>;
using ChCarry = EventChannel<4>;
using ChSnap = EventChannel<5>;
using ChPad = EventChannel<0>;       ///< a PIT divider onto the pad (test e)
using ChCcl = EventChannel<1>;       ///< a PIT divider into a LUT (test n)
using PadPin = Pin<'D', 2>;          ///< EVOUTD, and a fully asynchronous sense pin

// ---- the two-board half's wires and peripherals -----------------------------------
// PE0 is the shared command wire (both boards' USART4 TXD pads, LBME);
// PE2 is the peer's stimulus into this chip - and it is a Px2 pin, one
// of the two FULLY ASYNCHRONOUS positions of every port (port.md),
// which is what lets an edge on it wake this device from power-down at
// all; PE3 is this chip's echo back into the peer's hardware capture.
#if defined(BRIO_BOARD_ea48)
// The AVR EA: no USART4, and no USART route reaches PE0 (DS40002443A
// 17.3.3, 17.3.4): USART1's ALT1 TXD is PC4, jumpered onto PE0 on this board, so
// the command wire is the same net through another pad. PE0 itself stays
// an input with no pull - one driver per net.
using LinkUart = Usart<1>;
constexpr UsartRoute link_route = UsartRoute::alt1;
using LinePin = Pin<'C', 4>;
#else
using LinkUart = Usart<4>;
constexpr UsartRoute link_route = UsartRoute::def;
using LinePin = Pin<'E', 0>;
#endif
constexpr char line_port = LinePin::port_letter;
constexpr uint8_t line_pin = LinePin::pin_number;
using WakePin = Pin<'E', 2>;
using EchoPin = Pin<'E', 3>;
using Client = I2cClient<0, TwiRoute::def>;
using WakeLut = Lut<0>;
constexpr uint8_t client_addr = 0x42;

constexpr uint32_t crystal_hz = SysClock::hz;
constexpr PitPeriod pit_period = PitPeriod::cyc4096;     // 125 ms at 32.768 kHz
constexpr uint16_t pit_cycles_used = 4096;
/// CLK_PER ticks in one PIT period, nominal (OSC32K's own error moves it).
constexpr uint32_t ticks_per_pit = crystal_hz / (32768u / pit_cycles_used);
/// A 2 ms TCB period at CLK_PER/1.
constexpr uint16_t waker_2ms = static_cast<uint16_t>(crystal_hz / 500u);

// ---- shared with the ISRs ---------------------------------------------------------
volatile uint16_t pit_irqs = 0;
volatile uint16_t pad_irqs = 0;
volatile uint16_t waker_irqs = 0;
volatile uint16_t rtc_irqs = 0;
volatile uint16_t wake_irqs = 0;      ///< PORTE: the peer's stimulus on PE2
volatile uint16_t ccl_irqs = 0;       ///< the CCL LUT interrupt (test n)
volatile uint16_t unit_irqs = 0;      ///< the unit under test in test o (AC, ADC, TCD)
volatile uint16_t adc_last = 0;       ///< the ADC's last RESRDY result (test o)
volatile uint16_t sfd_irqs = 0;       ///< USART4 RXSIF: a start bit in standby
volatile uint16_t twis_addr = 0;      ///< TWI client: address matches
volatile uint16_t twis_stops = 0;     ///< TWI client: STOP conditions
volatile uint8_t twis_rx[8];
volatile uint8_t twis_n = 0;

// ---- the bench ---------------------------------------------------------------------
// z runs the single-board letters, y the two-board ones.
TwoKeyTestBench<Serial, 'y'> bench;

void verdict(const char* name, bool ok) { bench.verdict(name, ok); }
bool near_u32(uint32_t a, uint32_t b, uint32_t tol) {
    return (a > b ? a - b : b - a) <= tol;
}

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    delay_us(clock, 2000);            // the shift register, generously
}

// ---- the stopwatch ----------------------------------------------------------------

/// (Re)build the 32-bit CLK_PER counter. `standby` sets RUNSTDBY on
/// BOTH halves - the cascade task does not expose the flag, so the two
/// configurations are written here through the resource's own config
/// struct (the same two the task writes, plus run_standby).
void watch_init(bool standby) {
    Watch::init(TcbClock::div1, ChCarry{}, ChSnap{});
    if (standby) {
        WatchHi::init({.mode = TcbMode::capture, .clock = TcbClock::event, .compare = 0,
                       .event_input = true, .cascade = true, .run_standby = true});
        WatchLo::init({.mode = TcbMode::capture, .clock = TcbClock::div1, .compare = 0,
                       .event_input = true, .run_standby = true});
    }
    Watch::reset();
}

// ---- the main clock ---------------------------------------------------------------
// Nothing may be printed between these two: the console's baud is
// programmed for 24 MHz and OSCHF is only within a per cent of the
// crystal - good enough to keep a byte, not good enough to trust a
// whole test's output to.

#if defined(BRIO_BOARD_ea48)
/// CLK_PER from OSCHF at the rate its fuse chose (OSCCFG, 20 MHz on this
/// board's chip: the EA has no FRQSEL), through the prescaler the boot's
/// Clock left at /1, with the oscillator's own RUNSTDBY as asked.
bool use_oschf(bool osc_standby) {
    Oschf::run_standby(osc_standby);
    return MainClock::select(MainSource::oschf);
}
#else
/// CLK_PER from OSCHF at 24 MHz, with the oscillator's own RUNSTDBY as
/// asked.
bool use_oschf(bool osc_standby) {
    (void)Oschf::set_hz(24'000'000);
    Oschf::run_standby(osc_standby);
    return MainClock::select(MainSource::oschf);
}
#endif

/// CLK_PER from the crystal again, restarted with the RUNSTDBY flag
/// asked. XOSCHFCTRLA's fields are read-only while the oscillator is
/// enabled, so the crystal is reconfigured from OSCHF and only then
/// selected back.
///
/// MEASURED: an oscillator whose RUNSTDBY is CLEAR does not run merely
/// because it is enabled - it runs when something
/// REQUESTS it. Enabling the crystal with RUNSTDBY = 0 while CLK_PER
/// comes from OSCHF therefore leaves MCLKSTATUS.EXTS clear forever, and
/// waiting for it is a dead end. Selecting the crystal IS the request:
/// the switch is what starts it, and MCLKSTATUS.SOSC stays set until it
/// is stable - which is why the wait moves into select() here, with a
/// bound generous enough for the 4096-cycle start-up.
#if defined(CLKCTRL_XOSCHFCTRLA)
bool use_crystal(bool osc_standby) {
    if (MainClock::source() == MainSource::extclk) {
        if (!use_oschf(true)) return false;
    }
    Xoschf::start_crystal(crystal_hz, XoschfStartup::cycles4k, osc_standby);
    if (osc_standby && !Xoschf::wait_stable()) return false;
    return MainClock::select(MainSource::extclk, 0x00FFFFFFu) && Xoschf::stable();
}
#else
/// The DA: CLK_PER from the external clock on PA0. There is no oscillator
/// behind it on the chip, so it has no RUNSTDBY and no start-up of its
/// own; what a standby wake on it pays is the voltage regulator, which
/// VREGCTRL's AUTO profile drops whenever OSC32K is the last clock
/// running (13.3.5). The one oscillator the DA can hold up in standby is
/// OSCHF, so `osc_standby` is OSCHF's RUNSTDBY here - the DA's stand-in
/// for the DB crystal's own bit: set, the regulator stays at full drive
/// and the external clock wakes at once; clear, the wake pays the
/// regulator's start-up (measured: test j). The board's oscillator runs
/// whatever the chip does; the boot's Clock init proved it is there (it
/// asked through a TCD request before selecting it, clock.hpp).
bool use_crystal(bool osc_standby) {
    Oschf::run_standby(osc_standby);
    if (MainClock::source() == MainSource::extclk) return true;
    return MainClock::select(MainSource::extclk, 0x00FFFFFFu);
}
#endif

// ---- one sleep, measured ----------------------------------------------------------

struct SleepRun {
    uint32_t ticks;      ///< CLK_PER ticks the stopwatch counted across the sleep
    uint16_t rtc;        ///< RTC counter ticks (32 kHz) across it
    uint16_t pits;       ///< PIT interrupts taken
    uint16_t pads;       ///< PORTD interrupts taken
    uint16_t wakers;     ///< TCB0 interrupts taken
    uint16_t ccls;       ///< CCL interrupts taken (test n)
    uint16_t units;      ///< the unit's interrupts taken (test o)
    uint16_t rtc0;       ///< the RTC counter before the sleep
    uint16_t rtc1;       ///< and immediately after the wake (see test f: stale)
};

/// Wait for the next PIT interrupt: a sleep entered right after one
/// spans a whole period, with no boundary to race.
void pit_sync() {
    const uint16_t p0 = pit_irqs;
    sei();
    for (uint32_t i = 0; i < 4'000'000u && pit_irqs == p0; ++i) {
    }
}

/// Sleep once in `m` and report what moved. `sync` waits for a PIT edge
/// first (test e turns it off: there the PIT interrupt is disabled on
/// purpose). `spin` replaces the sleep with a busy wait for the same
/// wake-up - the baseline every latency figure is measured against.
SleepRun sleep_once(SleepMode m, bool sync = true, bool spin = false) {
    console_drain();
    if (sync) pit_sync();
    SleepRun r{};
    cli();
    const uint32_t t0 = Watch::read();
    const uint16_t c0 = Rtc::count();
    const uint16_t p0 = pit_irqs, d0 = pad_irqs, w0 = waker_irqs, l0 = ccl_irqs;
    const uint16_t u0 = unit_irqs;
    sei();
    if (spin) {
        while (pit_irqs == p0 && pad_irqs == d0 && waker_irqs == w0) {
        }
    } else {
        Sleep::enter(m);
    }
    cli();
    r.ticks = Watch::read() - t0;
    r.rtc1 = Rtc::count();
    r.rtc0 = c0;
    r.rtc = static_cast<uint16_t>(r.rtc1 - c0);
    r.pits = static_cast<uint16_t>(pit_irqs - p0);
    r.pads = static_cast<uint16_t>(pad_irqs - d0);
    r.wakers = static_cast<uint16_t>(waker_irqs - w0);
    r.ccls = static_cast<uint16_t>(ccl_irqs - l0);
    r.units = static_cast<uint16_t>(unit_irqs - u0);
    sei();
    return r;
}

/// The same, in POWER-DOWN, with the watchdog as the escape hatch: a
/// mode that never wakes resets the board, and `stage` is what the next
/// boot prints instead of the bench hanging.
SleepRun power_down_once(uint8_t stage, bool sync = true) {
    console_drain();
    if (sync) pit_sync();
    token.magic = token_magic;
    token.stage = stage;
    (void)Watchdog::arm(WdtTime::s4);
    const SleepRun r = sleep_once(SleepMode::power_down, false);
    (void)Watchdog::off();
    token.magic = 0;
    token.stage = 0;
    return r;
}

#if defined(BRIO_BOARD_ea48)
// ---- the AVR EA's self-driven pads (letter p) ------------------------------------
// The pins a PIT divider can drive while the device is in POWER-DOWN,
// with no wire: an EVOUT position, or a LUT's output on an asynchronous
// path (FILTSEL = 0, EDGEDET = 0: table 13-3 note 3). Each pad senses its
// own drive. On this board PA2/PA3 and PC2/PC3 are the I2C bus, PC6 is the
// peer's stimulus net, PF2/PF3 the LED and the button and PF6 RESET -
// none of them is driven here.
using PinA6 = Pin<'A', 6>;           ///< LUT0's ALT1 output
using PinA7 = Pin<'A', 7>;           ///< EVOUTA ALT1
using PinC7 = Pin<'C', 7>;           ///< EVOUTC ALT1: the echo's net, PE3 its partner
using PinD3 = Pin<'D', 3>;           ///< LUT2's output
using PinD6 = Pin<'D', 6>;           ///< LUT2's ALT1 output
using PinD7 = Pin<'D', 7>;           ///< EVOUTD ALT1
using PortaLut = Lut<0>;
using PortdLut = Lut<2>;
using ChPin = EventChannel<2>;       ///< the PIT divider into a LUT (letter p)

volatile uint16_t porta_irqs = 0;    ///< PORTA: letter p's PA6/PA7
volatile uint16_t portc_irqs = 0;    ///< PORTC: letter p's PC7, letter q's PC4/PC6

/// Every pad letters p and q drive or sense back to an input with no
/// sense, its driver released, its flag cleared.
void quiesce_pins() {
    EvOut<PinA7>::unlisten();
    EvOut<PinC7>::unlisten();
    EvOut<PinD7>::unlisten();
    (void)PortaLut::init({});        // the output pin released, the route back to default
    PortaLut::disable();
    (void)PortdLut::init({});
    PortdLut::disable();
    PortaLut::EventA::unlisten();
    PortdLut::EventA::unlisten();
    ChPin::off();
    PinA6::configure({});
    PinA7::configure({});
    PinC7::configure({});
    PinD3::configure({});
    PinD6::configure({});
    PinD7::configure({});
    Pin<'C', 6>::configure({});
    LinePin::configure({.pull = PinPull::up});   // the command wire's idle level, as command mode leaves it
    Pin<'E', 0>::configure({});
    (void)Port<'A'>::take_flags();
    (void)Port<'C'>::take_flags();
    (void)Port<'D'>::take_flags();
}
#endif

// ---- the state every test starts from ----------------------------------------------

void quiesce() {
    console_drain();                  // it moves the main clock: no byte in flight
    Sleep::disarm();
    (void)Watchdog::off();
#if !defined(BRIO_BOARD_ea48)
    Vreg::power(VregPower::normal);
    (void)Vreg::high_temp_low_leakage(false);
#endif
    Ccl::disable();
    WakeLut::disable();
    WakeLut::sense(LutSense::none);
    WakeLut::EventA::unlisten();
    ChCcl::off();
#if !defined(BRIO_BOARD_ea48)
    Twi<1>::client_enable(false);
#endif
    Twi<0>::client_enable(false);     // test m's address-match wake source

    EvOut<PadPin>::unlisten();
    ChPad::off();
    PadPin::configure({});
    PadPin::clear_flag();
#if defined(BRIO_BOARD_ea48)
    quiesce_pins();
#endif

    // The two-board half's pins: the stimulus stops being a wake-up
    // source, the echo parks low. Nothing here touches USART4 - the
    // command channel is the two-board tests' own business.
    WakePin::configure({});
    EchoPin::clear();
    EchoPin::output();
    (void)Port<'E'>::take_flags();

#if defined(BRIO_BOARD_ea48)
    Oschf::run_standby(true);         // as Clock::init() left it
#endif
    (void)use_crystal(true);          // the board's boot configuration

    Waker::enable_capt_interrupt(false);
    Waker::disable();
    Rtc::disable();
    Rtc::enable_ovf_interrupt(false);
    Rtc::enable_cmp_interrupt(false);
    RtcClock::select(RtcSource::osc32k);
    Pit::init(pit_period);            // enabled, interrupt on: the waker
    watch_init(false);
    rtc_irqs = 0;
    waker_irqs = rtc_irqs;
    pad_irqs = waker_irqs;
    pit_irqs = pad_irqs;
    sfd_irqs = 0;
    ccl_irqs = sfd_irqs;
    wake_irqs = ccl_irqs;
    twis_stops = 0;
    twis_addr = twis_stops;
    twis_n = 0;
#if defined(BRIO_BOARD_ea48)
    porta_irqs = 0;
    portc_irqs = 0;
#endif
    sei();
}

// ---- a the register surface ----------------------------------------------------------
// No sleeping here: what a program can ask SLPCTRL for, and the two
// rules the driver enforces rather than merely documents.
#if defined(BRIO_BOARD_ea48)
// The AVR EA's SLPCTRL is CTRLA alone (DS40002443A 13.4): the three modes
// and the erratum's NOP (DS80001048D 2.2.3, the DB's 2.2.4 word for
// word); there is no VREGCTRL, so no regulator verb exists to test - its
// absence is a compile-time refusal (test/family/neg/sleep_no_vreg_on_ea).
void ta_surface() {
    print(serial, "a SLPCTRL: the surface and the NOP (the AVR EA: CTRLA alone, no "
                  "regulator control)", crlf);
    quiesce();

    verdict("nothing is armed at rest", !Sleep::armed());
    Sleep::arm(SleepMode::idle);
    verdict("arm(idle): SEN set, SMODE reads back idle",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::idle);
    Sleep::arm(SleepMode::standby);
    verdict("arm(standby) reads back",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::standby);
    Sleep::arm(SleepMode::power_down);
    verdict("arm(power_down) reads back",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::power_down);
    Sleep::disarm();
    verdict("disarm() clears SEN", !Sleep::armed());
    verdict("and the platform's own readback agrees", !P::sleep_armed());

    // DS80001048D 2.2.3 (revisions B1 and B2): a store to an address
    // >= 0x40 immediately followed by a write to SLPCTRL.CTRLA loses that
    // write. A TCB counter store (0x0B0A) and an arm() with nothing in
    // between must still arm, the NOP being in sleep.hpp; that it is
    // really adjacent to the store is read in the listing.
    Waker::count(0);
    Sleep::arm(SleepMode::standby);
    verdict("a store above 0x40 immediately before arm() does not eat it "
            "(DS80001048D 2.2.3's NOP)",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::standby);
    Waker::count(0);
    Sleep::disarm();
    verdict("and the same holds for disarm()", !Sleep::armed());
    print(serial, "  no VREGCTRL on this die: PMODE and HTLLEN do not exist, and Vreg is "
                  "refused by name at compile time.", crlf);
    quiesce();
}
#else
void ta_surface() {
    print(serial, "a SLPCTRL and VREGCTRL: the surface, the NOP and the HTLLEN interlock", crlf);
    quiesce();

    verdict("nothing is armed at rest", !Sleep::armed());
    Sleep::arm(SleepMode::idle);
    verdict("arm(idle): SEN set, SMODE reads back idle",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::idle);
    Sleep::arm(SleepMode::standby);
    verdict("arm(standby) reads back",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::standby);
    Sleep::arm(SleepMode::power_down);
    verdict("arm(power_down) reads back",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::power_down);
    Sleep::disarm();
    verdict("disarm() clears SEN", !Sleep::armed());
    verdict("and the platform's own readback agrees", !P::sleep_armed());

    // Errata 2.2.4: a store to an address >= 0x40 immediately followed
    // by a write to SLPCTRL.CTRLA loses that write. Every store in
    // sleep.hpp is preceded by the documented NOP, so the sequence
    // below - a TCB counter write (0x0A1A) and an arm() with nothing in
    // between - must still arm. (That the NOP is really adjacent to the
    // store is a property of the generated code, checked in
    // firmware.lst, not from here.)
    Waker::count(0);
    Sleep::arm(SleepMode::standby);
    verdict("a store above 0x40 immediately before arm() does not eat it "
            "(erratum 2.2.4's NOP)",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::standby);
    Waker::count(0);
    Sleep::disarm();
    verdict("and the same holds for disarm()", !Sleep::armed());

    // VREGCTRL is the block's one CCP-protected register.
    verdict("the regulator starts in its Normal profile", Vreg::power() == VregPower::normal);
    Vreg::power(VregPower::performance);
    verdict("PMODE = performance reads back (a CCP write really landed)",
            Vreg::power() == VregPower::performance);
    Vreg::power(VregPower::normal);
    verdict("and back to normal", Vreg::power() == VregPower::normal);

    verdict("HTLLEN starts clear", !Vreg::high_temp_low_leakage());
    verdict("with no TWI client and no CCL, enabling HTLLEN is accepted",
            Vreg::high_temp_low_leakage(true) && Vreg::high_temp_low_leakage());
    verdict("PMODE survived the HTLLEN write", Vreg::power() == VregPower::normal);
    verdict("and disabling it is accepted",
            Vreg::high_temp_low_leakage(false) && !Vreg::high_temp_low_leakage());

    // The interlock, both halves. The CCL is enabled with no LUT
    // configured (it touches no pin at all); the TWI half is exercised
    // on TWI1, whose default pins PF2/PF3 go nowhere on this board -
    // the desk's I2C bus is TWI0's and is never touched here.
    Ccl::enable();
    verdict("the CCL is enabled", Ccl::enabled());
    verdict("HTLLEN is REFUSED while the CCL could wake the device",
            !Vreg::high_temp_low_leakage(true));
    verdict("and nothing was written", !Vreg::high_temp_low_leakage());
    Ccl::disable();
    verdict("with the CCL off it is accepted again", Vreg::high_temp_low_leakage(true));
    (void)Vreg::high_temp_low_leakage(false);

    Twi<1>::client_enable(true);
    verdict("a TWI client is enabled", Twi<1>::client_enabled());
    verdict("HTLLEN is REFUSED while a TWI address match could wake the device",
            !Vreg::high_temp_low_leakage(true));
    verdict("and nothing was written", !Vreg::high_temp_low_leakage());
    Twi<1>::client_enable(false);
    verdict("with the client off it is accepted again", Vreg::high_temp_low_leakage(true));
    verdict("disabling HTLLEN is never refused", Vreg::high_temp_low_leakage(false));
    print(serial, "  the interlock is chapter 13's own warning made enforceable: with "
                  "HTLLEN set, the TWI address match and the CCL are not wake-up "
                  "sources any more.", crlf);
    quiesce();
}
#endif

// ---- b IDLE through Sleep::enter() ----------------------------------------------------
// The platform's idle() is the kernel's hook, and the six-cycle cost of
// its wake is test_avr_platform's measurement. What is proven here is that
// Sleep::enter(idle) is the same thing under an application's control:
// it stops the CPU, comes back on an interrupt, and disarms itself.
void tb_idle() {
    print(serial, "b IDLE through Sleep::enter(): the CPU stops, an interrupt returns it", crlf);
    quiesce();

    // A TCB alarm every 2 ms is the wake source: the PIT's 125 ms would
    // make the loop below take a quarter of a minute.
    verdict("the 2 ms alarm started", PeriodicTick<Waker>::init(clock, 500u));
    Pit::enable_interrupt(false);            // one wake source at a time

    console_drain();
    uint32_t asleep_turns = 0, awake_turns = 0;
    bool woke = true, disarmed = true, ints_on = true;
    constexpr uint8_t n = 16;
    for (uint8_t i = 0; i < n; ++i) {
        cli();
        const uint16_t w0 = waker_irqs;
        sei();
        Sleep::enter(SleepMode::idle);
        if (Sleep::armed()) disarmed = false;
        if (!P::interrupts_enabled()) ints_on = false;
        cli();
        if (waker_irqs == w0) woke = false;
        sei();
        ++asleep_turns;
    }
    verdict("enter(idle) returned only after an interrupt fired", woke);
    verdict("SLPCTRL is disarmed on return", disarmed);
    verdict("interrupts are still enabled on return", ints_on);
    verdict("that is 16 sleeps", asleep_turns == n);

    // The CPU really stopped: over the same span (16 alarms) a counter
    // that only advances while the CPU runs turns once per wake asleep
    // and thousands of times awake.
    volatile uint32_t work = 0;
    cli();
    uint16_t w0 = waker_irqs;
    sei();
    while (static_cast<uint16_t>(waker_irqs - w0) < n) {
        work = work + 1;
    }
    awake_turns = work;
    print(serial, "  over 16 alarm periods: ", asleep_turns, " loop turns asleep (one per "
                  "wake), ", awake_turns, " awake", crlf);
    verdict("the loop is frozen while the CPU sleeps", asleep_turns <= n);
    verdict("the same span awake turns it thousands of times",
            awake_turns > 100u * asleep_turns);

    // Parity with the kernel's own hook, on the same alarm.
    const uint32_t before = Watch::read();
    P::idle();
    const uint32_t after = Watch::read();
    verdict("the platform's idle() still works next to the new verb", after != before);
    verdict("and leaves SLPCTRL disarmed too", !P::sleep_armed());
    quiesce();
}

// ---- c standby is real -----------------------------------------------------------------
void tc_standby() {
    print(serial, "c STANDBY: CLK_PER stops, the PIT still wakes", crlf);
    quiesce();
    // The stopwatch has no RUNSTDBY and the crystal has no requester
    // asking for it: nothing should count.
    const SleepRun spun = sleep_once(SleepMode::standby, true, true);
    const SleepRun slept = sleep_once(SleepMode::standby);

    print(serial, "  one PIT period (", pit_cycles_used, " CLK_RTC cycles = 125 ms): "
                  "awake the stopwatch counted ", spun.ticks, " CLK_PER ticks, asleep ",
          slept.ticks, " (nominal ", ticks_per_pit, " per period)", crlf);
    verdict("the PIT interrupt is what came back", slept.pits == 1);
    verdict("awake, the stopwatch counts the whole period",
            near_u32(spun.ticks, ticks_per_pit, ticks_per_pit / 8));
    verdict("asleep, it counts almost nothing: CLK_PER was gone",
            slept.ticks < ticks_per_pit / 100);
    print(serial, "  what it did count is the tail after the wake-up: ", slept.ticks,
          " ticks = ", (slept.ticks * 1000u) / (crystal_hz / 1000u), " us of code "
          "between the wake and the read", crlf);
    verdict("standby left SLPCTRL disarmed", !Sleep::armed());
    quiesce();
}

// ---- d the RUNSTDBY chain ---------------------------------------------------------------
// RUNSTDBY on a peripheral asks for its clock to be kept alive in
// standby; each oscillator has a RUNSTDBY of its own. Which of the two
// decides? Both ends are swept here - two sources x the oscillator flag
// x the peripheral flag - and every leg is one 125 ms sleep with the
// PIT as the waker. The source column is read back from CLKCTRL at the
// moment of the measurement, so a clock move that did not happen shows
// up in the table instead of being assumed.
struct Leg {
    const char* what;
    uint32_t ticks;
    uint8_t source;
};

const char* source_name(uint8_t s) {
    return s == static_cast<uint8_t>(MainSource::oschf) ? "OSCHF"
         : s == static_cast<uint8_t>(MainSource::extclk) ? (has_xoschf ? "XOSCHF" : "EXTCLK")
         : "other";
}

void td_chain() {
    print(serial, "d the RUNSTDBY chain: peripheral flag x oscillator flag x source", crlf);
    quiesce();
    Leg legs[5];
    auto leg = [](const char* what) -> Leg {
        const uint32_t t = sleep_once(SleepMode::standby).ticks;
        return {what, t, static_cast<uint8_t>(MainClock::source())};
    };

    // 1. the baseline of test c, on the crystal: no peripheral request.
    watch_init(false);
    legs[0] = leg(has_xoschf ? "crystal (RUNSTDBY on) + TCB RUNSTDBY off"
                             : "EXTCLK (OSCHF held up) + TCB RUNSTDBY off");

    // 2. the peripheral asks; the crystal was started with its own
    //    RUNSTDBY set (that is what Clock<crystal>::init does).
    watch_init(true);
    legs[1] = leg(has_xoschf ? "crystal (RUNSTDBY on) + TCB RUNSTDBY on"
                             : "EXTCLK (OSCHF held up) + TCB RUNSTDBY on");

    // 3. the same, with the crystal's own RUNSTDBY cleared.
    console_drain();
    const bool re_xtal = use_crystal(false);
    watch_init(true);
    legs[2] = leg(has_xoschf ? "crystal (RUNSTDBY off) + TCB RUNSTDBY on"
                             : "EXTCLK (OSCHF not held) + TCB RUNSTDBY on");

    // 4./5. the same two on OSCHF. The console is silent across both
    //       switches: its baud belongs to the crystal.
    console_drain();
    const bool on_oschf = use_oschf(false);
    watch_init(true);
    legs[3] = leg("OSCHF (RUNSTDBY off) + TCB RUNSTDBY on");
    const bool oschf_std = use_oschf(true);
    watch_init(true);
    legs[4] = leg("OSCHF (RUNSTDBY on) + TCB RUNSTDBY on");

    // A wake source that is NOT the PIT: a TCB interrupt, from a TCB
    // whose chain is complete. Table 13-4 lists TCB among standby's
    // wake-up sources - for the instances left running.
    (void)Waker::init({.mode = TcbMode::periodic, .clock = TcbClock::div1,
                       .compare = waker_2ms, .run_standby = true});     // 2 ms
    Waker::enable_capt_interrupt(true);
    Pit::enable_interrupt(false);
    const SleepRun by_tcb = sleep_once(SleepMode::standby, false);
    Pit::enable_interrupt(true);
    Waker::enable_capt_interrupt(false);
    Waker::disable();

    // And the same TCB with its RUNSTDBY cleared: it cannot wake what
    // it cannot count. The PIT is the backstop that ends the sleep.
    (void)Waker::init({.mode = TcbMode::periodic, .clock = TcbClock::div1,
                       .compare = waker_2ms});
    Waker::enable_capt_interrupt(true);
    const SleepRun by_pit = sleep_once(SleepMode::standby);
    Waker::enable_capt_interrupt(false);
    Waker::disable();

    console_drain();
    const bool back = use_crystal(true);

    verdict("every clock move of the sweep landed",
            re_xtal && on_oschf && oschf_std && back);
    for (const Leg& l : legs) {
        print(serial, "  ", l.what, ": ", l.ticks, " CLK_PER ticks in 125 ms (nominal ",
              ticks_per_pit, ", source in force ", source_name(l.source), ")", crlf);
    }
    bool counted[5];
    for (uint8_t i = 0; i < 5; ++i) {
        counted[i] = near_u32(legs[i].ticks, ticks_per_pit, ticks_per_pit / 8);
    }
    verdict("without the peripheral's RUNSTDBY nothing counts, whatever the "
            "oscillator does", !counted[0]);
    verdict("with BOTH ends armed the counter runs through the sleep",
            counted[1] && counted[4]);
    // The measured rule, and it is NOT symmetric: the PERIPHERAL's flag
    // decides. Its request is what keeps CLK_PER - and the oscillator
    // behind it - alive in standby; the oscillator's own RUNSTDBY adds
    // nothing while something is requesting it (what it does buy shows
    // up in test g, as wake-up latency).
    verdict(has_xoschf ? "the peripheral's RUNSTDBY ALONE is enough on the crystal: its "
                         "request keeps the oscillator running"
                       : "the peripheral's RUNSTDBY ALONE is enough on the external clock: its "
                         "request keeps CLK_PER running", counted[2]);
    verdict("and the same on OSCHF", counted[3]);
    // The two sources' counts differ by OSCHF's own error against the
    // crystal - a few hundred ppm - and the ruler is OSC32K, whose period
    // wanders by as much between two legs (measured: the crystal's count
    // moves 1800 ticks, 600 ppm, from run to run, and OSCHF read FASTER
    // than the crystal in one run of twelve). No verdict rests on it: the
    // source in force is read back from CLKCTRL instead (above).
    const int32_t apart = static_cast<int32_t>(legs[4].ticks) -
                          static_cast<int32_t>(legs[1].ticks);
    if constexpr (has_xoschf) {
        print(serial, "  OSCHF minus crystal over the same PIT period: ", apart,
              " ticks (OSCHF's error, inside the OSC32K ruler's wander)", crlf);
    } else {
        print(serial, "  OSCHF minus the external clock over the same PIT period: ", apart,
              " ticks (OSCHF's error, inside the OSC32K ruler's wander)", crlf);
    }
    verdict("a TCB whose chain is complete wakes the CPU from standby by itself",
            by_tcb.wakers >= 1 && by_tcb.pits == 0);
    verdict("a TCB whose RUNSTDBY is clear cannot: the PIT had to end that sleep",
            by_pit.wakers == 0 && by_pit.pits == 1);
#if defined(BRIO_BOARD_ea48)
    print(serial, "  the TCB wake ended after ", by_tcb.ticks, " CLK_PER ticks (its 2 ms "
                  "period is ", waker_2ms, ")", crlf);
#else
    print(serial, "  the TCB wake ended after ", by_tcb.ticks, " CLK_PER ticks (its 2 ms "
                  "period is 48000)", crlf);
#endif
    quiesce();
}

// ---- e a pin wakes from the deep modes ---------------------------------------------------
// Zero wires: EVSYS is alive in every sleep mode (table 13-2), so a PIT
// divider routed to EVOUT on PD2 drives that pad while the CPU sleeps,
// and PD2's own sense (Px2 is one of the two fully asynchronous pins of
// every port) sees the edge the device made for itself.
void te_pin_wake() {
    print(serial, "e a PORT pin wakes from standby and from power-down (no wires: the "
                  "PIT drives PD2 through EVOUT)", crlf);
    quiesce();

    ChPad::source(EvPitDiv<1024>{});          // 32 Hz square: an edge every 15.6 ms
    PadPin::configure({.sense = PinSense::both});
    EvOut<PadPin>::listen(ChPad{});
    Pit::enable_interrupt(false);             // the PIT must not be the one waking us
    PadPin::clear_flag();

    console_drain();
    delay_us(clock, 40'000u);                 // an edge has certainly passed
    const uint16_t d0 = pad_irqs;
    verdict("the pad really toggles: its interrupt fires with the CPU awake", d0 > 0);

    const SleepRun standby = sleep_once(SleepMode::standby, false);
    verdict("standby was ended by the pin, not by anything else",
            standby.pads >= 1 && standby.pits == 0 && standby.wakers == 0);
    print(serial, "  standby: ", standby.pads, " pad interrupt(s), stopwatch ",
          standby.ticks, " ticks (frozen)", crlf);

    // Power-down. The PIT interrupt goes back on as a BACKSTOP with a
    // long period (1 s): if the pad were dead in this mode the sleep
    // would still end, and the two counters say which source did it.
    Pit::period(PitPeriod::cyc32768);
    Pit::enable_interrupt(true);
    PadPin::clear_flag();
    const SleepRun pd = power_down_once(1, false);
    print(serial, "  power-down: ", pd.pads, " pad interrupt(s), ", pd.pits,
          " PIT interrupt(s) of the 1 s backstop, stopwatch ", pd.ticks, " ticks", crlf);
    verdict("power-down was ended by the pin, before the 1 s backstop could fire",
            pd.pads >= 1 && pd.pits == 0);
    print(serial, "  so the event system really drives a pad while every clock but the "
                  "32 kHz one is stopped, and a fully asynchronous pin sense picks it "
                  "up from power-down.", crlf);
    quiesce();
}

// ---- f power-down stops the rest -----------------------------------------------------------
// Both chains complete: OSCHF with RUNSTDBY under a TCB with RUNSTDBY,
// and the RTC counter with RUNSTDBY on the 32 kHz domain. In standby
// both count; in power-down neither the counter nor CLK_PER does, and
// the PIT alone comes back.
//
// One measured fact shapes the measurement: RTC.CNT read IMMEDIATELY after
// a wake still carries the value it had when the CPU went to sleep. The
// read is synchronized into CLK_PER (26.10) and that path needs the
// clock back plus a CLK_RTC edge, so the counter is read here after a
// settling delay - the raw read is kept and printed as its own finding.
void tf_power_down() {
    print(serial, "f POWER-DOWN: only the PIT survives - the RTC COUNTER does not", crlf);
    quiesce();

    (void)Rtc::init({.prescaler = RtcPrescaler::div1, .period = 0xFFFF,
                     .run_standby = true});
    console_drain();
    (void)use_oschf(true);
    watch_init(true);

    const SleepRun standby = sleep_once(SleepMode::standby);
    delay_us(clock, 1000);
    const uint16_t standby_settled = static_cast<uint16_t>(Rtc::count() - standby.rtc0);
    const SleepRun pd = power_down_once(2);
    delay_us(clock, 1000);
    const uint16_t pd_settled = static_cast<uint16_t>(Rtc::count() - pd.rtc0);

    console_drain();
    (void)use_crystal(true);

    print(serial, "  standby:    RTC counter +", standby.rtc, " ticks read at once, +",
          standby_settled, " read 1 ms later; stopwatch +", standby.ticks,
          " CLK_PER ticks", crlf);
    print(serial, "  power-down: RTC counter +", pd.rtc, " / +", pd_settled,
          " the same two ways; stopwatch +", pd.ticks, " ticks; PIT interrupts ",
          pd.pits, crlf);
    verdict("in standby the RTC counter runs (RUNSTDBY): one PIT period of "
            "CLK_RTC ticks", near_u32(standby_settled, pit_cycles_used, 256));
    verdict("in standby a fully armed chain keeps CLK_PER alive",
            near_u32(standby.ticks, ticks_per_pit, ticks_per_pit / 8));
    verdict("in power-down the RTC COUNTER stops even with RUNSTDBY set",
            pd_settled < 256);
    verdict("in power-down CLK_PER stops whatever any RUNSTDBY says",
            pd.ticks < ticks_per_pit / 100);
    verdict("and the PIT half of the same block still fires", pd.pits == 1);
    verdict("CNT read at the instant of the wake is STALE: it still reads the "
            "value it had at the SLEEP instruction", standby.rtc < 8);
    // The two counters measure the same interval in two clock domains,
    // which is what makes OSC32K's own error a number here.
    const uint32_t rtc_hz = standby.ticks
        ? static_cast<uint32_t>((static_cast<uint64_t>(pit_cycles_used) * crystal_hz) /
                                standby.ticks)
        : 0;
    print(serial, "  the same 4096 CLK_RTC cycles measured ", standby.ticks,
          " CLK_PER ticks, so CLK_RTC ran at ", rtc_hz, " Hz (nominal 32768)", crlf);
    print(serial, "  the wake-up latency out of power-down is NOT measurable from inside "
                  "this chip: the only clock that survives the mode is the PIT's, and the "
                  "counter that could time the restart is exactly the one it stops.", crlf);
    quiesce();
}

#if defined(BRIO_BOARD_ea48)
// ---- g the wake-up in 32 kHz ticks (the AVR EA) -------------------------------------------------
// The DB's letter g times VREGCTRL's profile on this ruler; the EA has no
// profile to set, and the question it leaves is whether a wake pays a
// regulator start-up at all - 13.3.2.2 names one, table 13-4 lists only
// "one OSC start-up". One board answers it coarsely: the RTC counter is
// the one clock a standby sleep keeps (its RUNSTDBY), CNT zeroed after a
// PIT interrupt reads 4096 plus the wake-up at the next one, read after a
// settle (letter f: CNT is stale at the instant of the wake), and the
// same sequence awake is the baseline that removes the settle and the
// zeroing. One tick is 1/32768 s nominal, 30.9 us on this die (OSC32K
// 1.3 % slow, the RTC suite's figure). The partner board's 41.7 ns ruler
// (letter j) measures the same configurations.
void tg_wake_ticks() {
    print(serial, "g the wake-up out of standby in 32 kHz ticks: what an oscillator's "
                  "RUNSTDBY buys, and whether a regulator start-up is paid (no regulator "
                  "control on this die)", crlf);
    quiesce();
    (void)Rtc::init({.prescaler = RtcPrescaler::div1, .period = 0xFFFF,
                     .run_standby = true});

    auto wake_ticks = [](bool spin) -> int16_t {
        console_drain();
        pit_sync();
        cli();
        Rtc::count(0);
        sei();
        (void)sleep_once(SleepMode::standby, false, spin);
        delay_us(clock, 1000);
        return static_cast<int16_t>(static_cast<int16_t>(Rtc::count()) -
                                    static_cast<int16_t>(pit_cycles_used));
    };

    // quiesce() leaves the crystal the main clock with its RUNSTDBY set,
    // and OSCHF with its own (Clock::init()'s state).
    const int16_t awake = wake_ticks(true);
    const int16_t xtal_kept = wake_ticks(false);
    console_drain();
    const bool m1 = use_oschf(false);            // the crystal still runs: its RUNSTDBY
    const int16_t oschf_beside = wake_ticks(false);
    const bool m2 = use_oschf(true);
    Xoschf::stop();
    const int16_t oschf_kept = wake_ticks(false);
    const bool m3 = use_oschf(false);            // nothing left but the 32 kHz domain
    const int16_t oschf_alone = wake_ticks(false);
    console_drain();
    const bool m4 = use_crystal(false);
    Oschf::run_standby(false);
    const int16_t xtal_restart = wake_ticks(false);
    console_drain();
    Oschf::run_standby(true);
    const bool back = use_crystal(true);

    verdict("every clock move of the sweep landed", m1 && m2 && m3 && m4 && back);
    print(serial, "  CNT - 4096 after the wake, in CLK_RTC ticks of 30.9 us (awake ", awake,
          " is the baseline: the settle and the CNT zeroing)", crlf);
    print(serial, "    crystal, its RUNSTDBY on:              ", xtal_kept, crlf);
    print(serial, "    OSCHF restarting, the crystal running:  ", oschf_beside, crlf);
    print(serial, "    OSCHF, its RUNSTDBY on:                ", oschf_kept, crlf);
    print(serial, "    OSCHF restarting, nothing else running: ", oschf_alone, crlf);
    print(serial, "    crystal restarting (its RUNSTDBY off):  ", xtal_restart, crlf);
    // A kept-alive clock wakes in six CLK_PER cycles (0.3 us): on this
    // ruler only the two reads' quantization is left, one tick either way.
    verdict("an oscillator kept alive by its RUNSTDBY costs nothing on this ruler: the "
            "crystal's and OSCHF's within one tick of awake",
            xtal_kept >= awake - 1 && xtal_kept <= awake + 1 && oschf_kept >= awake - 1 &&
                oschf_kept <= awake + 1);
    // The crystal's restart is 4096 of its cycles (CSUTHF 4k, the boot's
    // and use_crystal()'s) = 204.8 us at 20 MHz = 6.6 ticks of 30.9 us:
    // at least 6 ticks, less the quantization's one.
    verdict("a crystal that has to restart costs its counted start-up at least "
            "(4096 cycles = 204.8 us = 6.6 ticks: 5 or more)", xtal_restart >= awake + 5);
    // What letting go of every oscillator adds to OSCHF's restart: the
    // DA/DB's regulator start-up (125 us on the DA, 290 us on the DB) would
    // be four and nine ticks here; one is the quantization.
    verdict("OSCHF alone costs what OSCHF beside a running crystal costs, to a tick: no "
            "regulator start-up is paid", oschf_alone >= oschf_beside - 1 &&
                                           oschf_alone <= oschf_beside + 1);
    print(serial, "  the crystal's restart: ", xtal_restart - awake, " ticks over awake (the "
          "counted 204.8 us is 6.6; letter j has it in 41.7 ns ticks)", crlf);
    quiesce();
}
#else
// ---- g the voltage regulator ----------------------------------------------------------------
// What one board can say about VREGCTRL: that PMODE is a configuration
// and not a pulse (it survives a sleep), that HTLLEN does not cost the
// PIT its wake-up, and how long a wake-up out of standby takes. The
// 32 kHz domain is the only clock that keeps running across a standby
// sleep, so the RTC counter is the only ruler available: one tick is
// 30.5 us.
//
// The ruler: CNT is zeroed just after a PIT interrupt, so the NEXT one
// falls at CNT = 4096 plus whatever the wake-up cost. CNT is read after
// a fixed settling delay (test f: the read path is stale at the instant
// of the wake), and the same sequence measured AWAKE is the baseline
// that removes the settle and the cost of zeroing CNT. What the
// difference then shows is what the sleep itself cost - which is where
// an oscillator's own RUNSTDBY earns its keep: it is not what
// keeps a peripheral counting (test d), it is what saves the restart.
void tg_vreg() {
    print(serial, "g VREGCTRL: the profile across a sleep, HTLLEN with the PIT, and the "
                  "wake-up delay in 32 kHz ticks", crlf);
    quiesce();
    (void)Rtc::init({.prescaler = RtcPrescaler::div1, .period = 0xFFFF,
                     .run_standby = true});

    auto wake_ticks = [](bool spin) -> int16_t {
        console_drain();
        pit_sync();
        cli();
        Rtc::count(0);
        sei();
        (void)sleep_once(SleepMode::standby, false, spin);
        delay_us(clock, 1000);
        return static_cast<int16_t>(static_cast<int16_t>(Rtc::count()) -
                                    static_cast<int16_t>(pit_cycles_used));
    };

    // The sweep: the baseline, then the two sources x the oscillator's
    // own RUNSTDBY x the regulator's profile. The pairs that matter are
    // the ones where the oscillator really has to start again - that is
    // where both flags can show what they cost.
    const int16_t awake = wake_ticks(true);
    Vreg::power(VregPower::normal);
    const int16_t xtal_on = wake_ticks(false);
    Vreg::power(VregPower::performance);
    const int16_t xtal_on_perf = wake_ticks(false);
    verdict("PMODE survived a standby sleep", Vreg::power() == VregPower::performance);

    // The crystal with its own RUNSTDBY cleared: nothing requests it
    // while the CPU sleeps, so it stops and has to start again.
    console_drain();
    const bool xtal_free = use_crystal(false);
    Vreg::power(VregPower::normal);
    const int16_t xtal_off = wake_ticks(false);
    Vreg::power(VregPower::performance);
    const int16_t xtal_off_perf = wake_ticks(false);

    console_drain();
    const bool on_oschf = use_oschf(true);
    Vreg::power(VregPower::normal);
    const int16_t oschf_on = wake_ticks(false);
    const bool oschf_free = use_oschf(false);
    const int16_t oschf_off = wake_ticks(false);
    Vreg::power(VregPower::performance);
    const int16_t oschf_off_perf = wake_ticks(false);
    console_drain();
    Vreg::power(VregPower::normal);
    const bool back = use_crystal(true);

    verdict("every clock move of the sweep landed",
            xtal_free && on_oschf && oschf_free && back);
    print(serial, "  CNT - 4096 after the wake, in CLK_RTC ticks of 30.5 us (awake ",
          awake, " is the baseline: the settle and the CNT zeroing)", crlf);
    if constexpr (has_xoschf) {
        print(serial, "    crystal, its RUNSTDBY on:  ", xtal_on, " normal, ", xtal_on_perf,
              " performance", crlf);
        print(serial, "    crystal, its RUNSTDBY off: ", xtal_off, " normal, ", xtal_off_perf,
              " performance", crlf);
    } else {
        print(serial, "    EXTCLK, OSCHF held up:     ", xtal_on, " normal, ", xtal_on_perf,
              " performance", crlf);
        print(serial, "    EXTCLK, nothing held up:   ", xtal_off, " normal, ", xtal_off_perf,
              " performance", crlf);
    }
    print(serial, "    OSCHF,   its RUNSTDBY on:  ", oschf_on, " normal", crlf);
    print(serial, "    OSCHF,   its RUNSTDBY off: ", oschf_off, " normal, ", oschf_off_perf,
          " performance", crlf);
    verdict("waking from standby never costs less than staying awake",
            xtal_on >= awake && xtal_on_perf >= awake && xtal_off >= awake &&
                xtal_off_perf >= awake && oschf_on >= awake && oschf_off >= awake &&
                oschf_off_perf >= awake);
    verdict(has_xoschf ? "an oscillator kept alive by its OWN RUNSTDBY costs nothing to wake "
                         "from"
                       : "an oscillator kept alive by its RUNSTDBY costs nothing to wake from: "
                         "OSCHF, and the external clock while OSCHF is held up",
            xtal_on <= awake + 2 && oschf_on <= awake + 2);
    if constexpr (has_xoschf) {
        verdict("one that has to start again costs real time, and the crystal costs "
                "much more than OSCHF", xtal_off > oschf_off && oschf_off > oschf_on);
    } else {
        // The DA: the external clock has no start-up, but with nothing
        // held up the regulator drops and its start-up - test j: about
        // 130 us, four ticks of 30.5 us - is what the wake pays.
        verdict("the external clock with nothing held up pays the regulator's start-up "
                "(3 ticks or more), and so does OSCHF restarting",
                xtal_off >= awake + 3 && oschf_off >= awake + 3);
    }
    print(serial, "  that is the whole point of an oscillator's RUNSTDBY: test d showed "
                  "it is not what keeps a peripheral counting - this is what it buys.",
          crlf);
    if constexpr (has_xoschf) {
        print(serial, "  the regulator's profile moves the restarting cases by ",
              xtal_off - xtal_off_perf, " (crystal) and ", oschf_off - oschf_off_perf,
              " (OSCHF) ticks; a difference of zero or one tick is below this ruler's "
              "resolution and is reported, not claimed.", crlf);
    } else {
        print(serial, "  the regulator's profile moves the restarting cases by ",
              xtal_off - xtal_off_perf, " (EXTCLK) and ", oschf_off - oschf_off_perf,
              " (OSCHF) ticks; a difference of zero or one tick is below this ruler's "
              "resolution and is reported, not claimed.", crlf);
    }
    // The one place a single board can SEE the regulator: an OSCHF
    // restart out of standby is two costs in series, the oscillator's
    // and the regulator's, and PMODE = FULL pays the second one in
    // advance. On the crystal the oscillator's own start-up buries it.
    verdict("PMODE = performance really shortens a wake-up that has to restart "
            "OSCHF", oschf_off - oschf_off_perf >= 4);
    if constexpr (has_xoschf) {
        verdict("and it cannot shorten the crystal's own start-up",
                xtal_off_perf > oschf_off_perf);
    } else {
        verdict("and on the external clock it removes the whole cost: no oscillator is "
                "left to start", xtal_off_perf <= awake + 2);
    }

    // HTLLEN shortens power-down's wake-up list to the PORT pin, BOD
    // VLM, MVIO and the PIT. The PIT half of that is testable here.
    verdict("HTLLEN can be set (no TWI client, no CCL)", Vreg::high_temp_low_leakage(true));
    const SleepRun pd = power_down_once(3);
    verdict("with HTLLEN set, the PIT still wakes the device from power-down",
            pd.pits == 1);
    verdict("and HTLLEN is still set on the way out", Vreg::high_temp_low_leakage());
    verdict("clearing it is accepted", Vreg::high_temp_low_leakage(false));
    print(serial, "  the other half of HTLLEN - that the TWI address match and the CCL "
                  "stop waking the device - is not testable from one board: it is an "
                  "ABSENCE, and the driver refuses the combination that would need it.",
          crlf);
    quiesce();
}
#endif

// ==============================================================================
//  THE TWO-BOARD HALF (h..m, set y)
//
//  The far board runs sleep_peer and is driven IN BAND over the shared PE0
//  wire (src/apps/sleep_link.hpp: magic, opcode, checksum, ack before
//  act, every action bounded and self-restoring). The command channel is
//  8N1 at slink::command_baud with LBME at both ends: this board takes
//  the wire only to talk and hands it back to the pull-up to listen.
//
//  THE MEASUREMENT, once, so no test has to re-explain it:
//
//    The peer raises PE2 at a known count of a free-running 32-bit
//    stopwatch (the SAM C21 by a compare match, through its event
//    system to the pad). That edge is what has to wake this chip. This
//    chip's PORTE ISR raises PE3 as its first statement; at the peer
//    that edge is an event that latches the same stopwatch into a
//    capture register. The number that comes back is therefore
//
//      the peer's stimulus -> this chip's wake-up -> ISR prologue -> the store
//
//    in the peer's 24 MHz ticks (41.7 ns; from its crystal, which ident
//    reports). The fixed part - the peer's compare-to-edge path, the
//    wire, this chip's vector latency and ISR prologue, the peer's
//    capture path - is measured AWAKE in test i and is the baseline
//    every sleep figure is read against.
// ==============================================================================

const uint8_t no_payload[1] = {0};
slink::Decoder dec;

/// This suite owns the RTC block, so there is no Ticker to ask the time
/// of: every bound in the link client is a spin count. One iteration of
/// the polling loops below is a handful of cycles at 24 MHz, so this is
/// a generous "at least a millisecond" - a bound only has to outlast the
/// peer's own deadline, never to be accurate.
constexpr uint32_t spins_per_ms = 2000;

void put4(uint8_t b) { (void)LinkUart::send(b, 200'000u); }

/// Half-duplex turnaround guard. RXCIF is raised at the MIDDLE of the
/// stop bit - half a bit BEFORE the sender's TXCIF - so an end that
/// answers at once starts its start bit while the other end is still
/// transmitting with its receiver off. Four bit times, from the BAUD
/// register in force.
void turnaround_guard() {
    const uint32_t bit_cycles =
        (static_cast<uint32_t>(LinkUart::baud_reg()) * LinkUart::samples()) / 64u;
    delay_cycles(4u * (bit_cycles ? bit_cycles : 1u));
}

/// Take the shared wire: receiver off (LBME would otherwise decode our
/// own echo), pin driven, transmitter on.
void line_talk() {
    turnaround_guard();
    LinkUart::enable_rx(false);
    LinePin::set();
    LinePin::output();
    LinkUart::enable_tx(true);
    LinkUart::clear_txc();            // so line_listen waits for THIS burst
}

/// Give it back to the pull-up and listen again. It OWNS the wait for
/// the line to go idle: TXCIF is write-one-to-clear, so a caller that
/// waits first and then calls this would spin out its budget deaf.
void line_listen(bool wait = true) {
    if (wait) (void)LinkUart::wait_line_idle(200'000u);
    LinkUart::enable_tx(false);
    LinePin::input();
    pinctrl_of(line_port, line_pin) |= PORT_PULLUPEN_bm;
    LinkUart::flush_rx();
    LinkUart::enable_rx(true);
}

/// Command mode, the state everything returns to: 8N1 at the command
/// baud, receiver listening to the TXD pad through LBME, the pad itself
/// released to the pull-up.
bool link_command_mode() {
    const bool ok = LinkUart::init({.route = link_route,
                                    .baud = usart_baud_reg(SysClock::hz, slink::command_baud),
                                    .tx = false,
                                    .loop_back = true});
    LinePin::input();
    pinctrl_of(line_port, line_pin) |= PORT_PULLUPEN_bm;
    LinkUart::flush_rx();
    LinkUart::clear_txc();
    dec.reset();
    return ok;
}

bool link_quiet = false;      ///< suppress the failure dump while probing

bool recv_frame(slink::Frame& out, uint16_t ms) {
    dec.reset();
    const uint32_t budget = static_cast<uint32_t>(ms) * spins_per_ms;
    for (uint32_t i = 0; i < budget; ++i) {
        if (!LinkUart::rxc_flag()) continue;
        const UsartFrame f = LinkUart::receive();
        if (!f.clean()) { dec.reset(); continue; }
        if (dec.feed(static_cast<uint8_t>(f.data)) == slink::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

bool command_once(slink::Op op, const uint8_t* p, uint8_t len, uint16_t ms) {
    LinkUart::flush_rx();
    line_talk();
    slink::write_frame(put4, op, p, len);
    line_listen();
    slink::Frame f;
    if (!recv_frame(f, ms)) return false;
    return f.op == slink::Op::ack && f.len == 2 && f.data[0] == slink::byte_of(op);
}

/// The recovery guarantee in action: three attempts, each separated by a
/// quiet interval longer than the peer's own reassembly timeout.
bool command(slink::Op op, const uint8_t* p = no_payload, uint8_t len = 0,
             uint16_t ms = 80) {
    for (uint8_t k = 0; k < 3; ++k) {
        if (command_once(op, p, len, ms)) return true;
        (void)link_command_mode();
        delay_us(clock, 70'000);
    }
    if (!link_quiet) {
        print(serial, "    LINK FAILURE op ", hex(slink::byte_of(op)),
              ": the peer board did not acknowledge. It is sleep_peer on the shared PE0 wire; "
              "'0' on its console forces command mode.", crlf);
    }
    (void)link_command_mode();
    delay_us(clock, 70'000);
    return false;
}

bool query(slink::Op op, slink::Frame& data, const uint8_t* p = no_payload,
           uint8_t len = 0, uint16_t ms = 80) {
    if (!command(op, p, len, ms)) return false;
    return recv_frame(data, ms);
}

bool ensure_link() {
    link_quiet = true;
    (void)link_command_mode();
    const bool up = command(slink::Op::ping);
    link_quiet = false;
    if (!up) {
        print(serial, "  the peer board did not answer on the shared PE0 wire. It must be "
                      "running sleep_peer; '0' on its console forces command mode.", crlf);
    }
    return up;
}

bool peer_ident(slink::Ident& d) {
    slink::Frame f;
    if (!query(slink::Op::ident, f) || f.op != slink::Op::ident_data ||
        f.len != slink::ident_size) {
        return false;
    }
    d = slink::get_ident(f.data);
    return true;
}

/// Ask for the report of the last action, after giving the peer's bound
/// time to expire.
bool peer_report(slink::Report& r, uint16_t wait_ms) {
    (void)link_command_mode();
    if (wait_ms) delay_us(clock, static_cast<uint32_t>(wait_ms) * 1000u);
    for (uint8_t k = 0; k < 4; ++k) {
        slink::Frame f;
        if (query(slink::Op::report, f) && f.op == slink::Op::report_data &&
            f.len == slink::report_size) {
            r = slink::get_report(f.data);
            return true;
        }
        delay_us(clock, 40'000);
    }
    return false;
}

/// Command an action and wait out the rendezvous: the peer reconfigures
/// as soon as its acknowledgement has left the line.
bool peer_act(slink::Op op, const slink::Params& a) {
    uint8_t p[slink::params_size] = {};
    slink::put_params(p, a);
    if (!command(op, p, slink::params_size)) return false;
    delay_us(clock, static_cast<uint32_t>(slink::settle_ms) * 1000u);
    return true;
}

// ---- one measured train ---------------------------------------------------------

struct Train {
    uint32_t ticks[slink::max_shots] = {};
    uint8_t n = 0;          ///< values fetched
    uint16_t hits = 0;      ///< echoes the peer captured
    uint16_t woke = 0;      ///< stimuli this board answered
};

uint16_t train_woke = 0;

/// Arm the wires and tell the peer to fire `shots` stimuli. It waits
/// `lead_ms` before the first one, which is the window this board has to
/// move its main clock and get into the answering loop.
bool train_arm(uint8_t shots, uint16_t lead_ms = 300, uint16_t period_ms = 100,
               uint16_t deadline_ms = 300) {
    EchoPin::clear();
    EchoPin::output();
    WakePin::configure({.sense = PinSense::rising});
    WakePin::clear_flag();
    (void)Port<'E'>::take_flags();
    cli();
    wake_irqs = 0;
    sei();
    slink::Params a{};
    a.count = shots;
    a.delay_ms = lead_ms;
    a.period_ms = period_ms;
    a.deadline_ms = deadline_ms;
    a.hold_us = 300;
    return peer_act(slink::Op::pulse, a);
}

/// Answer every stimulus of the train, either awake or from `m`. The PIT
/// is the backstop: a stimulus that never arrives costs at most a few of
/// its periods, and its own wake-ups are simply slept through again.
void train_answer(bool sleeping, SleepMode m, uint8_t shots) {
    train_woke = 0;
    for (uint8_t i = 0; i < shots; ++i) {
        cli();
        const uint16_t d0 = wake_irqs;
        sei();
        bool got = false;
        for (uint8_t k = 0; k < 8 && !got; ++k) {
            if (sleeping) {
                Sleep::enter(m);
            } else {
                for (uint32_t s = 0; s < 1'500'000u && wake_irqs == d0; ++s) {
                }
            }
            cli();
            got = wake_irqs != d0;
            sei();
        }
        if (got) ++train_woke;
        // The echo has to stay up long enough for the peer's event path
        // to latch it, and then go back down: the next shot needs a
        // RISING edge on the same wire.
        delay_us(clock, 50);
        EchoPin::clear();
    }
}

/// Fetch the peer's numbers. Must run with the main clock back on the
/// crystal - the command channel's BAUD belongs to it.
bool train_collect(Train& t) {
    t.n = 0;
    t.woke = train_woke;
    slink::Report r{};
    if (!peer_report(r, 20)) return false;
    t.hits = r.hits;
    const uint8_t want = r.count > slink::max_shots ? slink::max_shots
                                                    : static_cast<uint8_t>(r.count);
    while (t.n < want) {
        const uint8_t p[1] = {t.n};
        slink::Frame f;
        if (!query(slink::Op::gaps, f, p, 1) || f.op != slink::Op::gaps_data ||
            f.len < slink::gaps_header) {
            return false;
        }
        const uint8_t start = f.data[0], k = f.data[1];
        if (k == 0 || start != t.n) return false;
        for (uint8_t i = 0; i < k && t.n < slink::max_shots; ++i) {
            t.ticks[t.n++] = slink::get32(f.data + slink::gaps_header + 4u * i);
        }
    }
    return true;
}

/// The middle value of the shots that were captured, or slink::no_capture
/// when none was. A median, not a mean: one PIT wake-up landing on top of
/// a stimulus must not move the figure the verdict reads.
uint32_t median_of(const Train& t) {
    uint32_t s[slink::max_shots];
    uint8_t k = 0;
    for (uint8_t i = 0; i < t.n; ++i) {
        if (t.ticks[i] != slink::no_capture) s[k++] = t.ticks[i];
    }
    if (k == 0) return slink::no_capture;
    for (uint8_t i = 1; i < k; ++i) {
        const uint32_t x = s[i];
        uint8_t j = i;
        while (j > 0 && s[j - 1] > x) { s[j] = s[j - 1]; --j; }
        s[j] = x;
    }
    return s[k / 2];
}

/// The peer's 24 MHz ticks, in the unit that reads best: one tick is
/// 41.7 ns.
void print_ticks(uint32_t ticks) {
    if (ticks == slink::no_capture) { print(serial, "MISS"); return; }
    print(serial, ticks, " ticks = ");
    if (ticks < 24'000u) print(serial, (ticks * 125u) / 3u, " ns");
    else print(serial, ticks / 24u, " us");
}

void print_train(const char* what, const Train& t) {
    print(serial, "    ", what, ": median ");
    print_ticks(median_of(t));
    print(serial, "  (", t.woke, " woke, ", t.hits, " captured of ", t.n, ":");
    for (uint8_t i = 0; i < t.n; ++i) {
        print(serial, " ");
        if (t.ticks[i] == slink::no_capture) print(serial, "-");
        else print(serial, t.ticks[i]);
    }
    print(serial, ")", crlf);
}

constexpr uint8_t train_shots = 8;

#if defined(BRIO_BOARD_ea48)
// ---- the clock configurations the deep modes are measured against (the AVR EA) ----
//
// Five of them. The EA has no regulator profile to set, so what the legs
// separate is the oscillator's restart from whatever else a wake pays:
// OSCHF restarting beside a crystal its own RUNSTDBY keeps running,
// against OSCHF restarting with nothing but the 32 kHz domain left - the
// configuration in which the DA/DB regulator drops (13.3.5 of theirs) and
// which DS40002443A 13.3.2.2 still describes ("the time it takes to start
// the regulator if it has been switched off") while table 13-4 lists one
// oscillator start-up only.
enum class LegClock : uint8_t {
    xtal_restart,       ///< crystal main clock, its own RUNSTDBY clear, OSCHF's clear
    xtal_kept,          ///< crystal main clock, kept alive by its RUNSTDBY
    oschf_xtal_alive,   ///< OSCHF main clock restarting, the crystal still oscillating
    oschf_kept,         ///< OSCHF main clock kept alive by its RUNSTDBY, the crystal stopped
    oschf_alone,        ///< OSCHF main clock restarting, every other oscillator stopped
};

/// Entered with the crystal the main clock and its RUNSTDBY set (quiesce()
/// and run_leg() leave it so).
bool apply_leg_clock(LegClock c) {
    switch (c) {
        case LegClock::xtal_restart: {
            const bool ok = use_crystal(false);
            Oschf::run_standby(false);      // after: use_crystal() hops through OSCHF
            return ok;
        }
        case LegClock::xtal_kept:
            return use_crystal(true);
        case LegClock::oschf_xtal_alive:
            return use_oschf(false);
        case LegClock::oschf_kept: {
            const bool ok = use_oschf(true);
            Xoschf::stop();
            return ok;
        }
        default: {
            const bool ok = use_oschf(false);
            Xoschf::stop();
            return ok;
        }
    }
}

/// One whole leg: arm the peer, move the clock, answer the train, put
/// everything back and fetch the numbers. Nothing is printed between the
/// two console_drain() calls - the console's BAUD belongs to the crystal.
///
/// NO OTHER WAKE THAN THE STIMULUS: the PIT's interrupt is off for the
/// train and the watchdog, with `stage` in the .noinit token, is the
/// escape hatch of every leg, standby included. A crystal that ran a
/// moment before restarts faster than a cold one - measured with the
/// PIT's 250 ms backstop left on: one or two shots of eight, where the
/// two periods' beat puts a PIT wake just before them, restarted in
/// 0.16 .. 0.39 ms against the cold 0.61 .. 0.67; with it off, none in
/// eighty-eight - so a backstop wake inside the train would time the
/// PIT's history, not the stimulus.
bool run_leg(LegClock c, SleepMode m, uint8_t stage, Train& t) {
    if (!train_arm(train_shots)) return false;
    console_drain();
    const bool moved = apply_leg_clock(c);
    Pit::enable_interrupt(false);
    token.magic = token_magic;
    token.stage = stage;
    (void)Watchdog::arm(WdtTime::s8);
    train_answer(true, m, train_shots);
    (void)Watchdog::off();
    token.magic = 0;
    token.stage = 0;
    Pit::enable_interrupt(true);
    console_drain();
    Oschf::run_standby(true);
    const bool back = use_crystal(true);
    return train_collect(t) && moved && back;
}
#else
// ---- the clock configurations the deep modes are measured against ---------------
//
// Four of them, and the fourth is the one the bench found and the data
// sheet does not spell out. VREGCTRL's AUTO profile drops the regulator
// to low power "in Standby and Power-Down, and whenever OSC32K is the
// only clock running" (13.3.5) - so an oscillator left RUNNING by its
// own RUNSTDBY keeps the regulator at full drive even when it is not the
// main clock source, and the wake-up never pays the regulator's
// start-up. `oschf_alone` stops the crystal outright, which is the only
// way to see that cost in standby.
enum class LegClock : uint8_t {
    xtal_restart,       ///< crystal main clock, its own RUNSTDBY clear
    xtal_kept,          ///< crystal main clock, kept alive by its RUNSTDBY
    oschf_xtal_alive,   ///< OSCHF main clock, the crystal still oscillating
    oschf_alone,        ///< OSCHF main clock, every other oscillator stopped
};

bool apply_leg_clock(LegClock c) {
    switch (c) {
        case LegClock::xtal_restart: {
            const bool ok = use_crystal(false);
            Oschf::run_standby(false);      // after: use_crystal() hops through OSCHF
            return ok;
        }
        case LegClock::xtal_kept:
            return use_crystal(true);
        case LegClock::oschf_xtal_alive:
            // The DA has no second oscillator to leave running: its leg
            // holds OSCHF itself up (test j's DA legs).
            if constexpr (has_xoschf) {
                return use_oschf(false);
            } else {
                return use_oschf(true);
            }
        default: {
            const bool ok = use_oschf(false);
#if defined(CLKCTRL_XOSCHFCTRLA)
            Xoschf::stop();
#endif
            return ok;
        }
    }
}

/// One whole leg: arm the peer, move the clock, answer the train, put
/// everything back and fetch the numbers. Nothing is printed between the
/// two console_drain() calls - the console's BAUD belongs to the crystal.
/// `stage` non-zero brackets the sleeps with the watchdog and the
/// .noinit token, which is what every power-down leg asks for.
bool run_leg(LegClock c, bool perf, SleepMode m, uint8_t stage, Train& t) {
    if (!train_arm(train_shots)) return false;
    console_drain();
    const bool moved = apply_leg_clock(c);
    if (perf) Vreg::power(VregPower::performance);
    if (stage) {
        token.magic = token_magic;
        token.stage = stage;
        (void)Watchdog::arm(WdtTime::s8);
    }
    train_answer(true, m, train_shots);
    if (stage) {
        (void)Watchdog::off();
        token.magic = 0;
        token.stage = 0;
    }
    Vreg::power(VregPower::normal);
    console_drain();
    const bool back = use_crystal(true);
    return train_collect(t) && moved && back;
}
#endif

// ---- h the link and the two measurement wires ------------------------------------
// Wire sanity BEFORE any sleep is involved: the command channel both
// ways, a corrupted frame refused, and each of the two measurement wires
// exercised on its own with this board wide awake.
void th_link() {
    print(serial, "h the link and the two measurement wires (nothing sleeps here)", crlf);
    quiesce();
    const bool up = ensure_link();
    verdict("the peer board answers a ping on the shared PE0 wire", up);
    if (!up) return;

    slink::Ident d{};
    const bool got = peer_ident(d);
    verdict("it identifies itself", got);
    if (got) {
        char label[9] = {};
        for (uint8_t i = 0; i < 8; ++i) label[i] = d.label[i];
        print(serial, "  peer: board ", label, ", CLK_PER from ",
              d.clock == slink::clock_crystal ? "XTAL" : "OSCHF", ", fw ",
              hex(d.version), crlf);
        verdict("the peer board runs sleep_peer", d.sanity == slink::ident_sanity);
        verdict("and it says which clock its stopwatch really counts",
                d.clock == slink::clock_crystal || d.clock == slink::clock_oschf);
    }

    // A frame whose checksum does not add up is refused, and the link
    // survives the desync.
    LinkUart::flush_rx();
    line_talk();
    put4(slink::magic);
    put4(slink::byte_of(slink::Op::ping));
    put4(0);
    put4(0x00);                                  // not the checksum
    line_listen();
    slink::Frame bad;
    verdict("a corrupted frame is refused with a nak, not obeyed",
            recv_frame(bad, 120) && bad.op == slink::Op::nak);
    (void)link_command_mode();
    verdict("and the link works again straight after", command(slink::Op::ping));

    // PE3: this board drives, B captures. The stopwatch pipeline is
    // proven end to end with no sleep anywhere near it.
    slink::Params a{};
    a.delay_ms = 5;
    a.deadline_ms = 300;
    EchoPin::clear();
    EchoPin::output();
    const bool armed = peer_act(slink::Op::capture, a);
    if (armed) {
        delay_us(clock, 40'000);
        EchoPin::set();
        delay_us(clock, 500);
        EchoPin::clear();
    }
    slink::Report r{};
    const bool have = armed && peer_report(r, 40);
    verdict("an edge this board puts on PE3 is CAPTURED by the peer's stopwatch",
            have && r.hits == 1);
    if (have) {
        print(serial, "  the peer measured ");
        print_ticks(r.ticks);
        print(serial, " from arming to that edge (a software delay here, not a "
                      "latency: it proves the path)", crlf);
    }

    // PE2: the peer drives, this board senses - awake, through the very ISR
    // every later test measures.
    Train t{};
    bool ok = train_arm(3, 250);
    if (ok) train_answer(false, SleepMode::idle, 3);
    ok = ok && train_collect(t);
    verdict("the link carried the stimulus train", ok);
    verdict("every stimulus on PE2 reached this board's PORTE interrupt",
            ok && t.woke == 3);
    verdict("and every echo came back to the peer's capture", ok && t.hits == 3);
    if (ok) print_train("awake, three shots", t);
    print(serial, "  both wires and the stopwatch are proven: everything below is a "
                  "sleep mode measured through this same path.", crlf);
    quiesce();
}

// ---- i the awake baseline and IDLE ----------------------------------------------
// The baseline is the whole fixed cost of the measurement: the peer's
// zero-to-edge gap, the wire, this chip's interrupt latency and the ISR
// prologue up to the store that raises PE3. IDLE then adds the data
// sheet's six CLK_PER cycles (13.3.3.2) and nothing else - which at
// 24 MHz is 250 ns, well under this ruler's noise. So the verdict is a
// CLASS, not an equality: idle must not cost a microsecond more than
// staying awake, and the exact ticks are printed for both.
void ti_idle_latency() {
    print(serial, "i the awake baseline and the cost of IDLE, timed from the peer board", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer board is reachable", false); return; }
    Pit::period(PitPeriod::cyc8192);          // a 250 ms backstop, out of the way

    Train awake{}, idle{};
    bool ok = train_arm(train_shots);
    if (ok) train_answer(false, SleepMode::idle, train_shots);
    ok = ok && train_collect(awake);
    bool ok2 = ok && train_arm(train_shots);
    if (ok2) train_answer(true, SleepMode::idle, train_shots);
    ok2 = ok2 && train_collect(idle);

    verdict("both trains got through the link", ok && ok2);
    if (!(ok && ok2)) { quiesce(); return; }
    print_train("awake  ", awake);
    print_train("idle   ", idle);
    const uint32_t ba = median_of(awake), bi = median_of(idle);
    verdict("awake, every stimulus was answered and captured",
            awake.woke == train_shots && awake.hits == train_shots);
    verdict("from IDLE, every stimulus was answered and captured",
            idle.woke == train_shots && idle.hits == train_shots);
    verdict("the awake baseline is a couple of microseconds: interrupt latency "
            "plus the ISR prologue", ba != slink::no_capture && ba < 240u);
#if defined(BRIO_BOARD_ea48)
    verdict("IDLE adds less than a microsecond to it (the six-cycle wake is "
            "300 ns at 20 MHz)",
            bi != slink::no_capture && ba != slink::no_capture && bi < ba + 24u);
#else
    verdict("IDLE adds less than a microsecond to it (the six-cycle wake is "
            "250 ns at 24 MHz)",
            bi != slink::no_capture && ba != slink::no_capture && bi < ba + 24u);
#endif
    print(serial, "  IDLE - awake = ", static_cast<int32_t>(bi) - static_cast<int32_t>(ba),
          " of the peer's ticks (41.7 ns each); the six-cycle figure itself is measured "
          "on-chip by test_avr_platform f.", crlf);
    quiesce();
}

#if defined(BRIO_BOARD_ea48)
// ---- j STANDBY, timed from outside (the AVR EA) -----------------------------------
// The five clock configurations of the EA (apply_leg_clock), in the
// peer's 41.7 ns ticks. The two OSCHF restarts are the experiment: beside
// a running crystal, and with every other oscillator stopped - their
// difference is what the device pays besides the oscillator when it lets
// go of everything, the regulator start-up 13.3.2.2 names. Nothing is
// printed while the main clock is not the crystal.
void tj_standby_latency() {
    print(serial, "j STANDBY wake-up latency against five clock configurations (no "
                  "regulator control on this die)", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer board is reachable", false); return; }
    Pit::period(PitPeriod::cyc8192);

    struct Leg { LegClock clk; const char* what; };
    constexpr Leg legs[] = {
        {LegClock::xtal_restart, "crystal, its RUNSTDBY off (it must start again)"},
        {LegClock::xtal_kept, "crystal, its RUNSTDBY on  (kept alive)         "},
        {LegClock::oschf_xtal_alive, "OSCHF restarting, the crystal still running   "},
        {LegClock::oschf_kept, "OSCHF kept by its RUNSTDBY, the crystal off    "},
        {LegClock::oschf_alone, "OSCHF restarting, every other oscillator off   "},
    };
    constexpr uint8_t n = sizeof(legs) / sizeof(legs[0]);
    Train t[n];
    uint32_t med[n];
    bool ok = true, all_woke = true;
    for (uint8_t i = 0; i < n; ++i) {
        ok = ok && run_leg(legs[i].clk, SleepMode::standby,
                           static_cast<uint8_t>(13 + i), t[i]);
        med[i] = median_of(t[i]);
        if (t[i].woke != train_shots) all_woke = false;
    }
    verdict("all five trains got through the link and every clock move landed", ok);
    if (!ok) { quiesce(); return; }
    for (uint8_t i = 0; i < n; ++i) print_train(legs[i].what, t[i]);
    bool captured = true;
    for (uint8_t i = 0; i < n; ++i) captured = captured && med[i] != slink::no_capture;
    verdict("every configuration woke on every stimulus", all_woke && captured);
    if (!captured) { quiesce(); return; }
    const int32_t bill = static_cast<int32_t>(med[4]) - static_cast<int32_t>(med[2]);
    print(serial, "  OSCHF alone minus OSCHF beside the crystal: ", bill, " ticks = ",
          bill / 24, " us - what letting go of every oscillator adds", crlf);
    verdict("a clock kept alive by its own RUNSTDBY costs almost nothing (under 50 us), "
            "the crystal's and OSCHF's", med[1] < 1'200u && med[3] < 1'200u);
    // tOSCHF_ST (DS40002443A 35.10.1, table 35-10) is 10 us typical "from
    // the wake-up event to code execution", with no limits: the restart
    // over the kept-alive OSCHF of the same path is held to that figure
    // halved and doubled, 5..20 us = 120..480 ticks - a band no regulator
    // start-up of this family (125 us on the DA, 290 us on the DB) fits in.
    const uint32_t restart = med[2] > med[3] ? med[2] - med[3] : 0;
    print(serial, "  OSCHF's restart over the kept OSCHF: ", restart, " ticks = ",
          (restart * 125u) / 3u, " ns (tOSCHF_ST: 10 us typical)", crlf);
    verdict("OSCHF restarting beside a running crystal costs its own start-up: 5..20 us "
            "over the kept OSCHF (tOSCHF_ST, 10 us typical)",
            restart > 120u && restart < 480u);
    // THE REGULATOR QUESTION. Each median is one of eight shots quantized
    // to a tick (41.7 ns), the shots of one leg spread by two ticks: a
    // microsecond (24 ticks) is twelve times that, and a hundredth of the
    // smallest regulator start-up this family has shown (the DA's 125 us).
    verdict("OSCHF restarting with EVERY other oscillator stopped costs the same to a "
            "microsecond: this die pays no regulator start-up (13.3.2.2's clause is "
            "never met)", (bill < 0 ? -bill : bill) < 24);
    // A crystal's restart is CSUTHF's 4096 counted cycles (204.8 us at
    // 20 MHz, the boot's and use_crystal()'s setting) after the oscillation
    // has built up; the chapter gives no figure for that build-up (table
    // 35-12's start-up row is empty), so the upper bound is the class's - three
    // milliseconds - not a derivation.
    verdict("a crystal that has to restart costs its counted start-up at least (4096 "
            "cycles at 20 MHz = 204.8 us) and less than 3 ms",
            med[0] > 4'915u && med[0] < 72'000u);
    quiesce();
}

// ---- k POWER-DOWN, timed from outside (the AVR EA) --------------------------------
// Power-down stops every oscillator whatever its RUNSTDBY says (12.3.8:
// "run constantly, except for Power-Down"), so OSCHF held by its flag
// restarts like OSCHF left alone, and the crystal like a crystal. The
// stimulus lands on PE2, a pin whose sense is asynchronous on every
// family; letter p and letter q wake the EA on its other pins.
void tk_power_down_latency() {
    print(serial, "k POWER-DOWN wake-up latency on an asynchronous pin", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer board is reachable", false); return; }
    Pit::period(PitPeriod::cyc8192);

    struct Leg { LegClock clk; uint8_t stage; const char* what; };
    constexpr Leg legs[] = {
        {LegClock::xtal_kept, 4, "crystal main clock                       "},
        {LegClock::oschf_kept, 5, "OSCHF main clock, its RUNSTDBY set       "},
        {LegClock::oschf_alone, 6, "OSCHF main clock, its RUNSTDBY clear     "},
    };
    constexpr uint8_t n = sizeof(legs) / sizeof(legs[0]);
    Train t[n];
    uint32_t med[n];
    bool ok = true, all_woke = true;
    for (uint8_t i = 0; i < n; ++i) {
        ok = ok && run_leg(legs[i].clk, SleepMode::power_down, legs[i].stage, t[i]);
        med[i] = median_of(t[i]);
        if (t[i].woke != train_shots) all_woke = false;
    }
    verdict("all three trains got through the link and every clock move landed", ok);
    if (!ok) { quiesce(); return; }
    for (uint8_t i = 0; i < n; ++i) print_train(legs[i].what, t[i]);
    bool captured = true;
    for (uint8_t i = 0; i < n; ++i) captured = captured && med[i] != slink::no_capture;
    verdict("a pin edge wakes this device from POWER-DOWN, every time, on all three "
            "configurations", all_woke && captured);
    if (!captured) { quiesce(); return; }
    verdict("a crystal main clock costs its counted start-up at least (204.8 us) and "
            "less than 3 ms (letter j's bounds)", med[0] > 4'915u && med[0] < 72'000u);
    verdict("OSCHF's RUNSTDBY buys nothing in POWER-DOWN: the two OSCHF legs within "
            "a microsecond (letter j's resolution argument)",
            (med[1] > med[2] ? med[1] - med[2] : med[2] - med[1]) < 24u);
    // tOSCHF_ST doubled (20 us) plus a kept clock's whole path (letter j:
    // 2.2 us, the peer's capture and this chip's vector included): 22 us.
    verdict("and OSCHF out of POWER-DOWN costs its own start-up and nothing more: under "
            "22 us, no regulator start-up here either", med[2] < 528u);
    quiesce();
}
#else
// ---- j STANDBY, timed from outside ----------------------------------------------
// Phase one proved WHICH RUNSTDBY keeps a peripheral counting and got at
// the wake-up cost only through the 30.5 us RTC ruler. Here the peer
// measures six configurations in 41.7 ns ticks, and the sixth one is the
// finding: the regulator costs nothing until the device is allowed to
// let go of every oscillator. Nothing is printed while the main clock is
// not the crystal - the console's BAUD belongs to it.
void tj_standby_latency() {
    print(serial, "j STANDBY wake-up latency against six clock configurations", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer board is reachable", false); return; }
    Pit::period(PitPeriod::cyc8192);

    struct Leg { LegClock clk; bool perf; const char* what; };
    constexpr Leg legs[] = {
        {LegClock::xtal_restart, false, has_xoschf ? "crystal, its RUNSTDBY off (it must start again)"
                                                   : "EXTCLK, nothing held up in standby          "},
        {LegClock::xtal_kept, false, has_xoschf ? "crystal, its RUNSTDBY on  (kept alive)         "
                                                : "EXTCLK, OSCHF held up by its RUNSTDBY        "},
        {LegClock::oschf_xtal_alive, false, has_xoschf ? "OSCHF, the crystal still running, normal      "
                                                       : "OSCHF held up by its RUNSTDBY, normal        "},
        {LegClock::oschf_xtal_alive, true, has_xoschf ? "OSCHF, the crystal still running, performance "
                                                      : "OSCHF held up by its RUNSTDBY, performance   "},
        {LegClock::oschf_alone, false, "OSCHF ALONE, every other oscillator off, normal"},
        {LegClock::oschf_alone, true, "OSCHF ALONE, every other oscillator off, perf. "},
    };
    Train t[6];
    uint32_t med[6];
    bool ok = true, all_woke = true;
    for (uint8_t i = 0; i < 6; ++i) {
        ok = ok && run_leg(legs[i].clk, legs[i].perf, SleepMode::standby, 0, t[i]);
        med[i] = median_of(t[i]);
        if (t[i].woke != train_shots) all_woke = false;
    }
    verdict("all six trains got through the link and every clock move landed", ok);
    if (!ok) { quiesce(); return; }
    for (uint8_t i = 0; i < 6; ++i) print_train(legs[i].what, t[i]);
    verdict("every configuration woke on every stimulus", all_woke);
    if constexpr (has_xoschf) {
        verdict("a crystal that has to restart costs the milliseconds phase one "
                "inferred from 48 RTC ticks (0.8 .. 3 ms)",
                med[0] != slink::no_capture && med[0] > 19'200u && med[0] < 72'000u);
        verdict("a crystal kept alive by its own RUNSTDBY costs almost nothing "
                "(under 50 us)", med[1] != slink::no_capture && med[1] < 1'200u);
        verdict("OSCHF restarting with the crystal still oscillating is the data "
                "sheet's own 24-30 us and NOTHING MORE (10 .. 60 us here)",
                med[2] != slink::no_capture && med[2] > 240u && med[2] < 1'440u);
        verdict("and the regulator's profile does not move it by a microsecond: "
                "there is no regulator start-up to pay for",
                med[3] != slink::no_capture &&
                    (med[3] > med[2] ? med[3] - med[2] : med[2] - med[3]) < 24u);
    } else {
        // The DA: no crystal to restart, so legs 0 and 1 are the external
        // clock with and without OSCHF held up, and legs 2 and 3 OSCHF held up
        // by its own bit. With nothing held up the wake pays the regulator's
        // start-up - the band DB's OSCHF-alone leg was given, 100 .. 800 us,
        // the data sheet's 100 us (table 37-10, power-down, AUTO) its floor;
        // with something held up it pays nothing the 50 us of a kept-alive
        // clock does not cover.
        verdict("the external clock with nothing held up pays the regulator's start-up "
                "and nothing else (100 .. 800 us)",
                med[0] != slink::no_capture && med[0] > 2'400u && med[0] < 19'200u);
        verdict("with OSCHF held up by its RUNSTDBY it costs almost nothing (under 50 us)",
                med[1] != slink::no_capture && med[1] < 1'200u);
        verdict("OSCHF held up by its own RUNSTDBY costs almost nothing too (under 50 us)",
                med[2] != slink::no_capture && med[2] < 1'200u);
        verdict("and the regulator's profile does not move it by a microsecond: the "
                "regulator never dropped",
                med[3] != slink::no_capture &&
                    (med[3] > med[2] ? med[3] - med[2] : med[2] - med[3]) < 24u);
    }
    verdict("stop the crystal too and the SAME OSCHF restart suddenly costs "
            "hundreds of microseconds (100 .. 800 us)",
            med[4] != slink::no_capture && med[4] > 2'400u && med[4] < 19'200u);
    verdict("because that cost is the REGULATOR, and PMODE = performance pays it "
            "in advance: back to the bare oscillator restart",
            med[5] != slink::no_capture && med[5] < 1'440u && med[5] * 3u < med[4]);
    if constexpr (has_xoschf) {
        print(serial, "  that is the standby rule this bench found: VREGCTRL's AUTO profile "
                      "drops the regulator only when OSC32K is the last clock left, so an "
                      "oscillator held up by its own RUNSTDBY buys a fast wake-up for the "
                      "OTHER source as well.", crlf);
        print(serial, "  the ordering: kept-alive < OSCHF beside a running crystal = OSCHF "
                      "alone with PMODE = performance < OSCHF alone < a crystal restart.",
              crlf);
    } else {
        print(serial, "  the same standby rule on the DA: its external clock is an input with "
                      "no RUNSTDBY, so OSCHF's is the one bit that keeps the regulator up - "
                      "for the external clock as well.", crlf);
    }
    quiesce();
}

// ---- k POWER-DOWN, timed from outside --------------------------------------------
// The gap phase one could not close. PE2 is a Px2 pin - one of the two
// FULLY ASYNCHRONOUS positions of every port - which is what makes an
// edge on it a wake-up source with every clock in the device stopped.
// The watchdog and the .noinit token bracket every train: a mode that
// never wakes resets the board and says so at the next boot.
//
// Power-down has no oscillator to hold the regulator up - RUNSTDBY buys
// nothing here, table 13-3 stops every domain but the PIT's - so this is
// where PMODE earns its keep unconditionally.
void tk_power_down_latency() {
    print(serial, "k POWER-DOWN wake-up latency on a fully asynchronous pin", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer board is reachable", false); return; }
    Pit::period(PitPeriod::cyc8192);

    struct Leg { LegClock clk; bool perf; uint8_t stage; const char* what; };
    constexpr Leg legs[] = {
        {LegClock::xtal_kept, false, 4, has_xoschf ? "crystal main clock                       "
                                                   : "EXTCLK main clock                        "},
        {LegClock::oschf_alone, false, 5, "OSCHF main clock, normal regulator       "},
        {LegClock::oschf_alone, true, 6, "OSCHF main clock, PMODE = performance    "},
    };
    Train t[3];
    uint32_t med[3];
    bool ok = true, all_woke = true;
    for (uint8_t i = 0; i < 3; ++i) {
        ok = ok && run_leg(legs[i].clk, legs[i].perf, SleepMode::power_down,
                           legs[i].stage, t[i]);
        med[i] = median_of(t[i]);
        if (t[i].woke != train_shots) all_woke = false;
    }
    verdict("all three trains got through the link and every clock move landed", ok);
    if (!ok) { quiesce(); return; }
    for (uint8_t i = 0; i < 3; ++i) print_train(legs[i].what, t[i]);
    verdict("a pin edge wakes this device from POWER-DOWN, every time, on all "
            "three configurations", all_woke);
    if constexpr (has_xoschf) {
        verdict("out of power-down a crystal main clock costs milliseconds (0.5 .. 6 ms)",
                med[0] != slink::no_capture && med[0] > 12'000u && med[0] < 144'000u);
    } else {
        // Power-down stops every oscillator whatever its RUNSTDBY: the
        // external clock has none to restart, so it pays the regulator
        // alone - the 100 .. 800 us band - and never more than OSCHF,
        // which pays that and its own start-up.
        verdict("out of power-down the external clock pays the regulator's start-up and no "
                "oscillator's (100 .. 800 us), no more than OSCHF's restart",
                med[0] != slink::no_capture && med[0] > 2'400u && med[0] < 19'200u &&
                    med[0] <= med[1]);
    }
    verdict("OSCHF out of power-down costs hundreds of microseconds (100 .. 800 us): "
            "the oscillator's 24 us plus the regulator's own start-up",
            med[1] != slink::no_capture && med[1] > 2'400u && med[1] < 19'200u);
    verdict("PMODE = performance removes the regulator's share and leaves the bare "
            "oscillator restart (under 60 us)",
            med[2] != slink::no_capture && med[2] < 1'440u);
    verdict("which is a factor of several, not a rounding: this is the one place "
            "the regulator's profile is worth its quiescent current",
            med[2] * 3u < med[1]);
    if constexpr (has_xoschf) {
        print(serial, "  the crystal's start-up buries the regulator entirely: its restart "
                      "is the same milliseconds in standby and in power-down.", crlf);
    }
    quiesce();
}
#endif

// ---- l start-of-frame detection --------------------------------------------------
// usart.md's one bench gap: SFDEN needs a device that is really in
// standby, and until there was a second board there was nothing to send
// the frame. Errata 2.16.2 makes it a pair of verbs rather than a
// configuration field - armed on the way into the sleep, disarmed on the
// way out - and that is exactly the shape used here. That the detector
// works at all through LBME, listening to the TXD pad of a shared wire,
// is itself a fact of this desk and not of the data sheet.
//
// 27.3.4.2 says a high-to-low transition powers the oscillator up and
// "the rest of the data frame can be received, provided that the baud
// rate is slow enough concerning the oscillator start-up time". The legs
// below take that sentence apart into TWO independent questions, each
// with its own designed experiment, against the three main-clock restart
// times test j has already measured: a crystal (about 1.5 ms), OSCHF
// with the regulator asleep (about 300 us) and OSCHF with PMODE =
// performance (about 24 us).
//
//   DID IT WAKE? The detector needs the line to still be LOW when the
//   peripheral clock is back. So each restart time is probed with two
//   bytes at one rate: 0xFF holds the line low for the START BIT ALONE,
//   0x00 for nine bit times. If the low level is what matters, 0x00
//   wakes where 0xFF does not, at every restart time - and the rate that
//   separates them is the restart time itself.
//
//   DID THE FRAME SURVIVE? Only if the clock is back before the middle
//   of the start bit, where the receiver takes its samples: half a bit.
#if defined(BRIO_BOARD_ea48)
// The AVR EA's four: no regulator profile, so OSCHF restarts either
// beside the running crystal or with every other oscillator stopped (the
// two restarts letter j separates).
enum class SfdClock : uint8_t { xtal_kept, xtal_restart, oschf_beside, oschf_alone };
#else
enum class SfdClock : uint8_t { xtal_kept, xtal_restart, oschf_slow, oschf_fast };
#endif

struct SfdLeg {
    uint32_t baud;
    uint8_t value;
    SfdClock clk;
    SleepMode mode;
    const char* what;
};

#if defined(BRIO_BOARD_ea48)
bool apply_sfd_clock(SfdClock c) {
    switch (c) {
        case SfdClock::xtal_kept: return use_crystal(true);
        case SfdClock::xtal_restart: {
            const bool ok = use_crystal(false);
            Oschf::run_standby(false);
            return ok;
        }
        case SfdClock::oschf_beside: return use_oschf(false);
        default: {
            const bool ok = use_oschf(false);
            Xoschf::stop();
            return ok;
        }
    }
}
#else
bool apply_sfd_clock(SfdClock c) {
    switch (c) {
        case SfdClock::xtal_kept: return use_crystal(true);
        case SfdClock::xtal_restart: return use_crystal(false);
        default: {
            const bool ok = use_oschf(false);
#if defined(CLKCTRL_XOSCHFCTRLA)
            Xoschf::stop();          // test j: a running crystal holds the regulator up
#endif
            if (c == SfdClock::oschf_fast) Vreg::power(VregPower::performance);
            return ok;
        }
    }
}
#endif

/// One SFD leg. `woke` and `intact` come back by reference; false means
/// the link or a clock move failed.
bool sfd_once(const SfdLeg& leg, bool& woke, bool& intact, bool& arrived,
              uint8_t& got) {
    const uint8_t value = leg.value;
    (void)link_command_mode();
    slink::Params a{};
    a.value = value;
    a.rate = leg.baud;
    a.delay_ms = 60;
    a.deadline_ms = 200;
    if (!peer_act(slink::Op::sfd_byte, a)) return false;

    // This board's end of the shared wire at the peer's rate: receiver
    // only, listening to the TXD pad through LBME.
    const uint16_t br = usart_baud_reg(SysClock::hz, leg.baud);
    if (!LinkUart::init({.route = link_route, .baud = br, .tx = false, .loop_back = true})) {
        return false;
    }
    LinePin::input();
    pinctrl_of(line_port, line_pin) |= PORT_PULLUPEN_bm;
    LinkUart::flush_rx();
    LinkUart::clear_rxs();
    cli();
    sfd_irqs = 0;
    sei();
    LinkUart::enable_rxs_interrupt(true);

    console_drain();
    const bool moved = apply_sfd_clock(leg.clk);
    // Armed only around the transition, and only here: with SFDEN set in
    // Active mode a read of RXDATA restarts the frame (errata 2.16.2).
    LinkUart::arm_start_of_frame();
    cli();
    const uint16_t s0 = sfd_irqs;
    sei();
    for (uint8_t k = 0; k < 4; ++k) {
        Sleep::enter(leg.mode);
        cli();
        const bool done = sfd_irqs != s0;
        sei();
        if (done) break;
    }
    LinkUart::disarm_start_of_frame();
    LinkUart::enable_rxs_interrupt(false);
    // The wake happens at the START BIT: the rest of the frame is still
    // on the wire. Give it three character times before asking.
    delay_us(clock, (30u * 1'000'000u) / leg.baud + 300u);
    const bool have = LinkUart::rxc_flag();
    const UsartFrame f = have ? LinkUart::receive() : UsartFrame{};
#if defined(BRIO_BOARD_ea48)
    Oschf::run_standby(true);
#else
    Vreg::power(VregPower::normal);
#endif
    const bool back = use_crystal(true);
    cli();
    woke = sfd_irqs != s0;
    sei();
    arrived = have;
    intact = have && f.clean() && static_cast<uint8_t>(f.data) == value;
    got = have ? static_cast<uint8_t>(f.data) : 0;
    (void)link_command_mode();
    return moved && back;
}

#if defined(BRIO_BOARD_ea48)
// The AVR EA's legs, at the console's rate (460800) and the link's
// (115200), against the restart times letter j measures: a kept clock
// back 0.5 us after the edge (j's kept leg less i's awake baseline, 52 -
// 41 ticks), OSCHF's restart about 9.7 us (its 275 ticks less the 41),
// the crystal's 0.61..0.67 ms from cold (0.16 ms the fastest seen when a
// PIT wake had just restarted it: run_leg()).
// The rule the DA/DB legs found, held to those times: the device wakes
// if the line is still LOW when the clock is back (a start bit alone is
// one bit time, 0x00 nine), and the frame survives if the clock is back
// before the middle of the start bit, where the receiver samples.
constexpr uint8_t sfd_legs = 13;

void tl_sfd() {
    print(serial, "l USART start-of-frame detection: a start bit on the shared wire "
                  "wakes this board (USART1 under LBME on PC4, the PE0 net)", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer board is reachable", false); return; }
    Pit::period(PitPeriod::cyc32768);         // a 1 s backstop (the DB's reason, above)

    constexpr SfdLeg legs[sfd_legs] = {
        {9'600, 0x5A, SfdClock::xtal_kept, SleepMode::standby,
         "clock kept alive      9600   0x5A (0.5 us vs a 52 us half-bit)  "},
        {115'200, 0x5A, SfdClock::xtal_kept, SleepMode::standby,
         "clock kept alive      115200 0x5A (0.5 us vs a 4.3 us half-bit) "},
        {460'800, 0x5A, SfdClock::xtal_kept, SleepMode::standby,
         "clock kept alive      460800 0x5A (0.5 us vs a 1.09 us half-bit)"},
        {9'600, 0x5A, SfdClock::oschf_beside, SleepMode::standby,
         "OSCHF beside crystal  9600   0x5A (9.7 us vs a 52 us half-bit) "},
        {115'200, 0x5A, SfdClock::oschf_beside, SleepMode::standby,
         "OSCHF beside crystal  115200 0x5A (9.7 us vs 17.4 us of low)   "},
        {115'200, 0xFF, SfdClock::oschf_alone, SleepMode::standby,
         "OSCHF alone           115200 0xFF (9.7 us vs 8.7 us of low)    "},
        {115'200, 0x00, SfdClock::oschf_alone, SleepMode::standby,
         "OSCHF alone           115200 0x00 (9.7 us vs 78 us of low)     "},
        {9'600, 0xFF, SfdClock::oschf_alone, SleepMode::standby,
         "OSCHF alone           9600   0xFF (9.7 us vs 104 us of low)    "},
        {9'600, 0x00, SfdClock::oschf_alone, SleepMode::standby,
         "OSCHF alone           9600   0x00 (9.7 us vs 938 us of low)    "},
        {9'600, 0xFF, SfdClock::xtal_restart, SleepMode::standby,
         "crystal restarts      9600   0xFF (0.61..0.67 ms vs 104 us)    "},
        {9'600, 0x00, SfdClock::xtal_restart, SleepMode::standby,
         "crystal restarts      9600   0x00 (0.61..0.67 ms vs 938 us)    "},
        {115'200, 0x5A, SfdClock::xtal_kept, SleepMode::idle,
         "IDLE, clock running   115200 0x5A                              "},
        {460'800, 0x5A, SfdClock::xtal_kept, SleepMode::idle,
         "IDLE, clock running   460800 0x5A                              "},
    };
    bool woke[sfd_legs] = {}, intact[sfd_legs] = {}, arrived[sfd_legs] = {};
    uint8_t got[sfd_legs] = {};
    bool ok = true;
    for (uint8_t i = 0; i < sfd_legs; ++i) {
        ok = ok && sfd_once(legs[i], woke[i], intact[i], arrived[i], got[i]);
    }
    verdict("the peer sent every commanded byte and the link survived", ok);
    for (uint8_t i = 0; i < sfd_legs; ++i) {
        print(serial, "    ", legs[i].what, ": ", woke[i] ? "WOKE   " : "no wake",
              "  frame ", intact[i] ? "INTACT" : (arrived[i] ? "GARBLED" : "lost   "),
              " (got ", hex(got[i]), ", sent ", hex(legs[i].value), ")", crlf);
    }
    verdict("with the main clock kept alive a start bit wakes this board from STANDBY at "
            "9600, at the link's 115200 and at the console's 460800",
            woke[0] && woke[1] && woke[2]);
    verdict("and the WAKING FRAME survives at all three: the clock is back 0.5 us after "
            "the edge, inside even 460800's 1.09 us half-bit", intact[0] && intact[1] &&
                                                                 intact[2]);
    verdict("an OSCHF restart (9.7 us) keeps the frame at 9600 (52 us half-bit) and "
            "loses it at 115200 (4.3 us half-bit), though 17.4 us of low wake it",
            woke[3] && intact[3] && woke[4] && !intact[4]);
    verdict("THE WAKE-UP RULE at OSCHF's 9.7 us: 0xFF at 115200 (low for 8.7 us) does not "
            "wake, 0x00 (78 us) does - and its frame is lost or garbled", !woke[5] && woke[6] &&
                                                                         !intact[6]);
    verdict("at 9600 OSCHF is back inside the start bit's first half: 0xFF and 0x00 both "
            "wake and both frames survive", woke[7] && intact[7] && woke[8] && intact[8]);
    verdict("the same rule at the crystal's restart: 104 us of low is not enough even "
            "just after a run (0.16 ms), 938 us is enough even cold (0.67 ms) - and that "
            "frame is lost",
            !woke[9] && woke[10] && !intact[10]);
    verdict("from IDLE the start-of-frame flag never fires - RXSIF is set only in "
            "Standby (25.3.4.2) - and the frame is simply received",
            !woke[11] && intact[11] && !woke[12] && intact[12]);
    verdict("SFDEN is disarmed again on the way out", !LinkUart::start_of_frame_armed());
    quiesce();
}
#else
constexpr uint8_t sfd_legs = 11;

void tl_sfd() {
    print(serial, "l USART start-of-frame detection: a start bit on the shared wire "
                  "wakes this board (SFD through LBME is itself a fact of this desk)",
          crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer board is reachable", false); return; }
    // The PIT is only a backstop here, and it must stay out of the way:
    // a device that is AWAKE in the PIT's own vector when the start bit
    // arrives is not in standby, and would look like a failed wake-up.
    // One second between its interrupts against a byte due in sixty
    // milliseconds makes that coincidence vanish.
    Pit::period(PitPeriod::cyc32768);

    constexpr SfdLeg legs[sfd_legs] = {
        {9'600, 0x5A, SfdClock::xtal_kept, SleepMode::standby,
         has_xoschf ? "clock kept alive      9600   0x5A (no restart at all)         "
                    : "EXTCLK, OSCHF held up 9600   0x5A (no restart at all)         "},
        {115'200, 0x5A, SfdClock::xtal_kept, SleepMode::standby,
         has_xoschf ? "clock kept alive      115200 0x5A (no restart at all)         "
                    : "EXTCLK, OSCHF held up 115200 0x5A (no restart at all)         "},
        {460'800, 0x5A, SfdClock::xtal_kept, SleepMode::standby,
         has_xoschf ? "clock kept alive      460800 0x5A (no restart at all)         "
                    : "EXTCLK, OSCHF held up 460800 0x5A (no restart at all)         "},
        {9'600, 0x5A, SfdClock::oschf_fast, SleepMode::standby,
         "OSCHF alone, perf.    9600   0x5A (24 us vs a 52 us half-bit) "},
        {115'200, 0xFF, SfdClock::oschf_fast, SleepMode::standby,
         "OSCHF alone, perf.    115200 0xFF (24 us vs 8.7 us of low)    "},
        {115'200, 0x00, SfdClock::oschf_fast, SleepMode::standby,
         "OSCHF alone, perf.    115200 0x00 (24 us vs 78 us of low)     "},
        {9'600, 0xFF, SfdClock::oschf_slow, SleepMode::standby,
         "OSCHF alone, normal   9600   0xFF (300 us vs 104 us of low)   "},
        {9'600, 0x00, SfdClock::oschf_slow, SleepMode::standby,
         "OSCHF alone, normal   9600   0x00 (300 us vs 938 us of low)   "},
        {2'400, 0xFF, SfdClock::xtal_restart, SleepMode::standby,
         has_xoschf ? "crystal restarts      2400   0xFF (1.5 ms vs 417 us of low)   "
                    : "EXTCLK alone          2400   0xFF (130 us vs 417 us of low)   "},
        {2'400, 0x00, SfdClock::xtal_restart, SleepMode::standby,
         has_xoschf ? "crystal restarts      2400   0x00 (1.5 ms vs 3.75 ms of low)  "
                    : "EXTCLK alone          2400   0x00 (130 us vs 3.75 ms of low)  "},
        {115'200, 0x5A, SfdClock::xtal_kept, SleepMode::idle,
         "IDLE, clock running   115200 0x5A                             "},
    };
    bool woke[sfd_legs] = {}, intact[sfd_legs] = {}, arrived[sfd_legs] = {};
    uint8_t got[sfd_legs] = {};
    bool ok = true;
    for (uint8_t i = 0; i < sfd_legs; ++i) {
        ok = ok && sfd_once(legs[i], woke[i], intact[i], arrived[i], got[i]);
    }
    verdict("the peer sent every commanded byte and the link survived", ok);
    for (uint8_t i = 0; i < sfd_legs; ++i) {
        print(serial, "    ", legs[i].what, ": ", woke[i] ? "WOKE   " : "no wake",
              "  frame ", intact[i] ? "INTACT" : (arrived[i] ? "GARBLED" : "lost   "),
              " (got ", hex(got[i]), ", sent ", hex(legs[i].value), ")", crlf);
    }
    verdict("with the main clock kept alive a start bit wakes this board from "
            "STANDBY at every rate", woke[0] && woke[1] && woke[2]);
    if constexpr (has_xoschf) {
        verdict("and the WAKING FRAME itself survives, all three rates",
                intact[0] && intact[1] && intact[2]);
    } else {
        // The DA's external clock, even with OSCHF held up, wakes about a
        // microsecond later than awake (test j: 44 ticks of 41.7 ns against
        // 20) - inside 9600's and 115200's half-bits (52 and 4.3 us), level
        // with 460800's (1.09 us), which is printed above and not judged.
        verdict("and the WAKING FRAME itself survives at 9600 and 115200 (half-bits of 52 "
                "and 4.3 us against a wake of about 1 us)", intact[0] && intact[1]);
    }
    verdict("a restart well inside the start bit's first half keeps the frame too: "
            "24 us against a 52 us half-bit at 9600", woke[3] && intact[3]);
    verdict("THE WAKE-UP RULE, at a 24 us restart: 0xFF (low for 8.7 us) does not "
            "wake, 0x00 (low for 78 us) does", !woke[4] && woke[5]);
    verdict("the same rule at a 300 us restart: 104 us of low is not enough, "
            "938 us is", !woke[6] && woke[7]);
    if constexpr (has_xoschf) {
        verdict("and at a 1.5 ms crystal restart: 417 us of low is not enough, 3.75 ms "
                "is", !woke[8] && woke[9]);
    } else {
        // The external clock alone pays the regulator, about 130 us (test
        // j): inside 2400 baud's 208 us half-bit, so the start bit of 0xFF
        // is low long enough AND the frame is back in time.
        verdict("and at the external clock's 130 us (the regulator alone): 417 us of low is "
                "enough, and both 2400-baud frames survive (130 us is inside a 208 us "
                "half-bit)", woke[8] && woke[9] && intact[8] && intact[9]);
    }
    print(serial, "  so the detector does not latch an EDGE: it needs the line still "
                  "LOW when the peripheral clock comes back, which is why the byte's "
                  "own bit pattern decides whether a slow-restarting device wakes at "
                  "all.", crlf);
    verdict("every wake that arrived after the middle of the start bit cost the "
            "frame", has_xoschf ? !intact[5] && !intact[7] && !intact[9]
                                : !intact[5] && !intact[7]);
    if constexpr (!has_xoschf) {
        // The external clock alone at 9600: the same wake-up rule at the
        // regulator's 130 us - a start bit alone (104 us) is too short, nine
        // bit times (938 us) are not, and arrive after the half-bit (52 us).
        constexpr SfdLeg extra[3] = {
            {9'600, 0xFF, SfdClock::xtal_restart, SleepMode::standby,
             "EXTCLK alone          9600   0xFF (130 us vs 104 us of low)   "},
            {9'600, 0x00, SfdClock::xtal_restart, SleepMode::standby,
             "EXTCLK alone          9600   0x00 (130 us vs 938 us of low)   "},
            {9'600, 0x5A, SfdClock::xtal_restart, SleepMode::standby,
             "EXTCLK alone          9600   0x5A (130 us vs 208 us of low)   "},
        };
        bool xw[3] = {}, xi[3] = {}, xa[3] = {};
        uint8_t xg[3] = {};
        bool xok = true;
        for (uint8_t i = 0; i < 3; ++i) {
            xok = xok && sfd_once(extra[i], xw[i], xi[i], xa[i], xg[i]);
        }
        for (uint8_t i = 0; i < 3; ++i) {
            print(serial, "    ", extra[i].what, ": ", xw[i] ? "WOKE   " : "no wake",
                  "  frame ", xi[i] ? "INTACT" : (xa[i] ? "GARBLED" : "lost   "),
                  " (got ", hex(xg[i]), ", sent ", hex(extra[i].value), ")", crlf);
        }
        verdict("the three 9600-baud legs on the external clock alone got through the link",
                xok);
        verdict("the rule at the external clock's 130 us: a start bit alone (104 us) does "
                "not wake, nine bit times do - and that frame is lost or garbled",
                !xw[0] && xw[1] && !xi[1]);
    }
    verdict("from IDLE the start-of-frame flag never fires - RXSIF is set only in "
            "Standby (27.5.5) - and the frame is simply received",
            !woke[10] && intact[10]);
    verdict("SFDEN is disarmed again on the way out, as errata 2.16.2 demands",
            !LinkUart::start_of_frame_armed());
    quiesce();
}
#endif

// ---- m a TWI address match wakes the device --------------------------------------
// This board becomes a client at 0x42 on the shared I2C bus (TWI0
// default, PA2/PA3) and sleeps with it enabled; the peer addresses it.
// The SCL stretch from the match to the first serviced byte IS the
// wake-up latency, and it is visible in the tenure's wall time, which
// the peer measures.
//
// The HTLLEN half of chapter 13's warning is asserted in situ here: with
// a client enabled the driver REFUSES to arm high-temperature low
// leakage, so the subset of power-down that has no TWI wake-up is
// enforced by construction and never entered by accident.
bool twi_tenure(SleepMode m, bool sleeping, uint8_t first, slink::Report& r) {
    cli();
    twis_stops = 0;
    twis_addr = twis_stops;
    twis_n = 0;
    sei();
    slink::Params a{};
    a.count = 3;
    a.addr = client_addr;
    a.value = first;
    a.rate = 100'000u;
    a.delay_ms = 120;
    a.deadline_ms = 300;
    if (!peer_act(slink::Op::twi_write, a)) return false;

    // THE ADDRESS MATCH IS THE ONLY TWI WAKE-UP SOURCE: table 13-4 lists
    // it and nothing else of the peripheral, so the DATA interrupts that
    // carry the rest of the frame cannot wake this device. Going back to
    // sleep after the match would therefore stall the tenure with the
    // host holding SCL - measured, and it is why this loop sleeps only
    // until the match and then STAYS AWAKE while the vector serves the
    // frame to its closing STOP.
    if (sleeping) {
#if defined(BRIO_BOARD_ea48)
        // The AVR EA: no wake but the address match (run_leg()'s reason -
        // a PIT wake just before would time a warm crystal), the watchdog
        // the escape hatch in both modes.
        Pit::enable_interrupt(false);
        token.magic = token_magic;
        token.stage = 7;
        (void)Watchdog::arm(WdtTime::s4);
#else
        if (m == SleepMode::power_down) {
            token.magic = token_magic;
            token.stage = 7;
            (void)Watchdog::arm(WdtTime::s4);
        }
#endif
        cli();
        const uint16_t a0 = twis_addr;
        const uint16_t s0 = twis_stops;
        sei();
        for (uint8_t k = 0; k < 6; ++k) {
            Sleep::enter(m);
            cli();
            const bool matched = twis_addr != a0;
            sei();
            if (matched) break;
        }
        for (uint32_t i = 0; i < 2'000'000u && twis_stops == s0; ++i) {
        }
#if defined(BRIO_BOARD_ea48)
        (void)Watchdog::off();
        token.magic = 0;
        token.stage = 0;
        Pit::enable_interrupt(true);
#else
        if (m == SleepMode::power_down) {
            (void)Watchdog::off();
            token.magic = 0;
            token.stage = 0;
        }
#endif
    }
    // Awake or not, let the closing STOP land before the numbers are read.
    delay_us(clock, 20'000);
    return peer_report(r, 60);
}

#if defined(BRIO_BOARD_ea48)
// The AVR EA: the same three tenures. No HTLLEN to refuse; the client
// stretches SCL "during the wake-up period until the main clock is
// available" (DS40002443A 27.3.5), so the power-down tenure carries the
// crystal's restart.
void tm_twi_wake() {
    print(serial, "m a TWI address match wakes this board from standby and from "
                  "power-down (the peer board is the host on the I2C bus)", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer board is reachable", false); return; }
    Pit::period(PitPeriod::cyc8192);

    verdict("the client comes up at 0x42 on TWI0's default pins",
            Client::init(clock, {.address = client_addr,
                                 .stop_interrupt = true,
                                 .data_interrupt = true,
                                 .address_interrupt = true}));

    slink::Report awake{}, standby{}, pd{};
    bool ok = twi_tenure(SleepMode::idle, false, 0x10, awake);
    const uint8_t awake_n = twis_n;
    const bool awake_ok = twis_rx[0] == 0x10 && twis_rx[1] == 0x11 && twis_rx[2] == 0x12;
    ok = ok && twi_tenure(SleepMode::standby, true, 0x20, standby);
    const uint8_t standby_n = twis_n;
    const bool standby_ok = twis_rx[0] == 0x20 && twis_rx[1] == 0x21 && twis_rx[2] == 0x22;
    ok = ok && twi_tenure(SleepMode::power_down, true, 0x30, pd);
    const uint8_t pd_n = twis_n;
    const bool pd_ok = twis_rx[0] == 0x30 && twis_rx[1] == 0x31 && twis_rx[2] == 0x32;

    verdict("the link carried all three tenures", ok);
    if (!ok) { Client::release(); quiesce(); return; }
    print(serial, "    awake      : ", awake_n, " bytes here, peer status ",
          hex(awake.status), ", tenure ");
    print_ticks(awake.ticks);
    print(serial, crlf, "    standby    : ", standby_n, " bytes here, peer status ",
          hex(standby.status), ", tenure ");
    print_ticks(standby.ticks);
    print(serial, crlf, "    power-down : ", pd_n, " bytes here, peer status ",
          hex(pd.status), ", tenure ");
    print_ticks(pd.ticks);
    print(serial, crlf);

    verdict("awake, the three bytes arrive intact and every one is acknowledged",
            awake_n == 3 && awake_ok && (awake.flags & slink::report_acked));
    verdict("STANDBY: the address match wakes this board and the frame is served "
            "in full", standby_n == 3 && standby_ok &&
                       (standby.flags & slink::report_acked));
    verdict("POWER-DOWN: it wakes there too, frame intact",
            pd_n == 3 && pd_ok && (pd.flags & slink::report_acked));
    verdict("waking from STANDBY with the main clock kept alive costs the bus "
            "nothing measurable: the tenure is the awake one to within 10 us",
            standby.ticks > awake.ticks ? standby.ticks - awake.ticks < 240u
                                        : awake.ticks - standby.ticks < 240u);
    verdict("waking from POWER-DOWN costs the bus the crystal's restart - more than its "
            "counted 204.8 us and less than 3 ms (letter k's bounds), held on SCL by the "
            "client", pd.ticks > awake.ticks && pd.ticks - awake.ticks > 4'915u &&
                          pd.ticks - awake.ticks < 72'000u);
    print(serial, "  the stretch over the awake baseline: standby +",
          static_cast<int32_t>(standby.ticks / 24u) - static_cast<int32_t>(awake.ticks / 24u),
          " us, power-down +", (pd.ticks - awake.ticks) / 24u, " us", crlf);
    Client::release();
    quiesce();
}
#else
void tm_twi_wake() {
    print(serial, "m a TWI address match wakes this board from standby and from "
                  "power-down (the peer board is the host on the I2C bus)", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer board is reachable", false); return; }
    Pit::period(PitPeriod::cyc8192);

    verdict("the client comes up at 0x42 on TWI0's default pins",
            Client::init(clock, {.address = client_addr,
                                 .stop_interrupt = true,
                                 .data_interrupt = true,
                                 .address_interrupt = true}));
    verdict("HTLLEN is REFUSED in situ, with the wake-up source it would silence "
            "actually enabled", !Vreg::high_temp_low_leakage(true));
    verdict("and nothing was written", !Vreg::high_temp_low_leakage());

    slink::Report awake{}, standby{}, pd{};
    bool ok = twi_tenure(SleepMode::idle, false, 0x10, awake);
    const uint8_t awake_n = twis_n;
    const bool awake_ok = twis_rx[0] == 0x10 && twis_rx[1] == 0x11 && twis_rx[2] == 0x12;
    ok = ok && twi_tenure(SleepMode::standby, true, 0x20, standby);
    const uint8_t standby_n = twis_n;
    const bool standby_ok = twis_rx[0] == 0x20 && twis_rx[1] == 0x21 && twis_rx[2] == 0x22;
    ok = ok && twi_tenure(SleepMode::power_down, true, 0x30, pd);
    const uint8_t pd_n = twis_n;
    const bool pd_ok = twis_rx[0] == 0x30 && twis_rx[1] == 0x31 && twis_rx[2] == 0x32;

    verdict("the link carried all three tenures", ok);
    if (!ok) { Client::release(); quiesce(); return; }
    print(serial, "    awake      : ", awake_n, " bytes here, peer status ",
          hex(awake.status), ", tenure ");
    print_ticks(awake.ticks);
    print(serial, crlf, "    standby    : ", standby_n, " bytes here, peer status ",
          hex(standby.status), ", tenure ");
    print_ticks(standby.ticks);
    print(serial, crlf, "    power-down : ", pd_n, " bytes here, peer status ",
          hex(pd.status), ", tenure ");
    print_ticks(pd.ticks);
    print(serial, crlf);

    verdict("awake, the three bytes arrive intact and every one is acknowledged",
            awake_n == 3 && awake_ok && (awake.flags & slink::report_acked));
    verdict("STANDBY: the address match wakes this board and the frame is served "
            "in full", standby_n == 3 && standby_ok &&
                       (standby.flags & slink::report_acked));
    verdict("POWER-DOWN: it wakes there too (HTLLEN clear), frame intact",
            pd_n == 3 && pd_ok && (pd.flags & slink::report_acked));
    // The client holds SCL from the address match until the first
    // serviced byte, so the whole wake-up is paid by the HOST on the
    // wire - and it is the SAME wake-up test j and test k measured on a
    // pin, seen from the other side of the bus.
    verdict("waking from STANDBY with the main clock kept alive costs the bus "
            "nothing measurable: the tenure is the awake one to within 10 us",
            standby.ticks > awake.ticks ? standby.ticks - awake.ticks < 240u
                                        : awake.ticks - standby.ticks < 240u);
    if constexpr (has_xoschf) {
        verdict("waking from POWER-DOWN costs the bus the whole crystal restart - the "
                "0.8 .. 3 ms test k measured on a pin",
                pd.ticks > awake.ticks && pd.ticks - awake.ticks > 19'200u &&
                    pd.ticks - awake.ticks < 72'000u);
    } else {
        verdict("waking from POWER-DOWN costs the bus the regulator's start-up - the "
                "100 .. 800 us test k measured on a pin",
                pd.ticks > awake.ticks && pd.ticks - awake.ticks > 2'400u &&
                    pd.ticks - awake.ticks < 19'200u);
    }
    if constexpr (has_xoschf) {
        print(serial, "  the stretch over the awake baseline: standby +",
              static_cast<int32_t>(standby.ticks / 24u) -
                  static_cast<int32_t>(awake.ticks / 24u),
              " us, power-down +", (pd.ticks - awake.ticks) / 24u,
              " us. At 100 kHz an SCL bit is 10 us, which is why a standby wake of "
              "under two microseconds is invisible on the wire and a crystal restart "
              "is not.", crlf);
    } else {
        print(serial, "  the stretch over the awake baseline: standby +",
              static_cast<int32_t>(standby.ticks / 24u) -
                  static_cast<int32_t>(awake.ticks / 24u),
              " us, power-down +", (pd.ticks - awake.ticks) / 24u,
              " us. At 100 kHz an SCL bit is 10 us, which is why a standby wake of "
              "under two microseconds is invisible on the wire and the regulator's "
              "start-up is not.", crlf);
    }

    Client::release();
    quiesce();
    verdict("the client is released and HTLLEN is settable again",
            Vreg::high_temp_low_leakage(true));
    (void)Vreg::high_temp_low_leakage(false);
}
#endif

// ---- n the CCL as a wake-up source (single board) ---------------------------------
// ccl.md's parked item. Table 13-4 lists the CCL among standby's wake-up
// sources when it has RUNSTDBY, and among power-down's only when the
// path through it is FULLY ASYNCHRONOUS - FILTSEL = 0 and EDGEDET = 0.
// All three shapes are built here on the one signal that survives every
// sleep mode: a PIT divider through the event system, which needs no
// wires. What the bench then shows is that the restriction is really
// about WHICH CLOCK the LUT was given: a filter running on OSC32K keeps
// working in power-down because that domain never stops, while the same
// filter on CLK_PER does not.
struct CclShape {
    LutFilter filter;
    LutClock clock;
    const char* what;
};

bool ccl_shape(const CclShape& s) {
    Ccl::disable();
    const bool ok = WakeLut::init({.in0 = LutInput::event_a,
                                   .truth = lut_truth([](bool a, bool, bool) { return a; }),
                                   .filter = s.filter,
                                   .edge_detect = false,
                                   .clock = s.clock,
                                   .interrupt = LutSense::rising});
    WakeLut::event_a_on(ChCcl{});
    Ccl::enable(true);                        // RUNSTDBY
    delay_us(clock, 20'000);
    return ok;
}

void tn_ccl_wake() {
    print(serial, "n the CCL as a wake-up source: which shape wakes from standby, and "
                  "which from power-down", crlf);
    quiesce();
    Pit::period(PitPeriod::cyc32768);         // a 1 s backstop, far from the CCL's rate
    Pit::enable_interrupt(true);
    ChCcl::source(EvPitDiv<64>{});            // 512 Hz square: an edge every ~1 ms

    constexpr CclShape filtered_32k = {LutFilter::filter, LutClock::osc32k,
                                       "clocked by OSC32K and FILTERED"};
    constexpr CclShape filtered_per = {LutFilter::filter, LutClock::clk_per,
                                       "clocked by CLK_PER and FILTERED"};
    constexpr CclShape async_path = {LutFilter::none, LutClock::osc32k,
                                     "FILTSEL = 0, EDGEDET = 0 (fully asynchronous)"};

    // 1. STANDBY, the RUNSTDBY-relevant shape: the LUT is clocked and
    //    filtered, so it needs its clock domain kept alive.
    verdict("a clocked, filtered LUT fed by the PIT divider comes up",
            ccl_shape(filtered_32k));
    cli();
    const uint16_t c0 = ccl_irqs;
    sei();
    verdict("it really toggles: its interrupt fires with the CPU awake", c0 > 0);
    const SleepRun standby = sleep_once(SleepMode::standby, false);
    verdict("the CCL wakes this board from STANDBY, before the 1 s backstop could",
            standby.pits == 0 && standby.ccls > 0);

    // 2. POWER-DOWN, three shapes. Every count below is taken INSIDE the
    //    measured sleep (SleepRun), so "which source ended it" is read off
    //    the same window and not off whatever ran after the wake.
    const SleepRun pd_32k = power_down_once(8, false);
    const bool cfg_per = ccl_shape(filtered_per);
    const SleepRun pd_per = power_down_once(9, false);
    const bool cfg_async = ccl_shape(async_path);
    const SleepRun pd_async = power_down_once(10, false);

    verdict("both other shapes come up too", cfg_per && cfg_async);
    print(serial, "  standby,    ", filtered_32k.what, ": ", standby.pits,
          " PIT / ", standby.ccls, " CCL interrupt(s)", crlf);
    print(serial, "  power-down, ", filtered_32k.what, ": ", pd_32k.pits,
          " PIT / ", pd_32k.ccls, " CCL interrupt(s)", crlf);
    print(serial, "  power-down, ", filtered_per.what, ": ", pd_per.pits,
          " PIT / ", pd_per.ccls, " CCL interrupt(s)", crlf);
    print(serial, "  power-down, ", async_path.what, ": ", pd_async.pits,
          " PIT / ", pd_async.ccls, " CCL interrupt(s)", crlf);

    verdict("a FULLY ASYNCHRONOUS CCL path wakes this board from POWER-DOWN, "
            "before the backstop - table 13-4's note 4",
            pd_async.pits == 0 && pd_async.ccls > 0);
    // The single CCL interrupt that follows this one is not a wake-up:
    // the frozen filter catches up the instant CLK_PER is back, which is
    // after the backstop has already ended the sleep. Which source ENDED
    // it is what the PIT count says.
    verdict("a FILTERED LUT clocked from CLK_PER does NOT wake from POWER-DOWN: "
            "that clock is gone and the 1 s backstop is what ends the sleep",
            pd_per.pits > 0);
    verdict("but a filtered LUT clocked from OSC32K DOES, because the 32 kHz "
            "domain is exactly what power-down keeps for the PIT",
            pd_32k.pits == 0 && pd_32k.ccls > 0);
    print(serial, "  so note 4's 'fully asynchronous' is really a statement about the "
                  "LUT's CLOCK: any path whose clock survives the mode survives with "
                  "it, and on this silicon OSC32K does.", crlf);
    quiesce();
}

// ---- o RUNSTDBY of the analog and the timers, in a real STANDBY ------------------------------
// Each unit's own RUNSTDBY decides whether it works while the CPU is in
// STANDBY (test d proved it for a TCB: the PERIPHERAL's request keeps
// its clock alive). Every leg is one 125 ms standby with the PIT as the
// backstop, the unit armed with and then without its flag:
//  - TCA0 counting CLK_PER/64: across the sleep it counts the whole PIT
//    period, or nothing;
//  - AC0 on PD2 (AINP0) against its DACREF, the pad driven by EVOUTD from
//    a PIT divider at 32 Hz (test e's square, an edge every 15.6 ms): its
//    CMP interrupt ends the sleep, or the PIT does. That also decides the
//    data sheet's contradiction (32.3.5: the interrupt works without
//    CLK_PER; 32.5.1: interrupts and status are not updated with the
//    clock stopped);
//  - ADC0 started by the same PIT divider through the event system (an
//    event-started conversion, 33.3.3.2), converting OP0's output (PD2 =
//    AIN2) - OP0 a follower of VDD/2, its pad with the pull-up on: its
//    RESRDY interrupt ends the sleep, or the PIT does; and the result
//    says whether the OP AMP stayed on (VDD/2) or went off with the
//    pull-up taking its pad to VDD;
//  - TCD0 counting CLK_PER or OSCHF over a 4096-count ramp (43.7 ms at
//    24 MHz / 256): its OVF interrupt ends an IDLE sleep; 25.3.6 says
//    the TCD stops in standby (and power-down), whatever its clock
//    source does.
using TcaU = Tca<0>;
using AcU = Ac<0>;
#if !defined(BRIO_BOARD_ea48)
using AdcU = Adc<0>;
#ifdef OPAMP
using OpU = Opamp<0>;
#endif
using TcdU = Tcd<0>;
#endif

constexpr uint32_t tca_per_pit = ticks_per_pit / 64u;      ///< 46875 at CLK_PER/64

/// One 125 ms standby with TCA0 counting, CNT read at the wake. `spin`
/// is the awake baseline over the same PIT period.
uint16_t tca_leg(bool run_standby, bool spin) {
    console_drain();
    pit_sync();
    (void)TcaU::init({.mode = TcaMode::normal, .clock = TcaClock::div64, .period = 0xFFFF,
                      .run_standby = run_standby});
    (void)sleep_once(SleepMode::standby, false, spin);
    const uint16_t c = TcaU::count();
    TcaU::disable();
    return c;
}

/// One standby with AC0 armed on PD2's 32 Hz square.
SleepRun ac_leg(bool run_standby) {
    (void)AcU::init({.positive = AcPos::ainp0, .negative = AcNeg::dacref,
                     .reference = Ref::v1024, .dacref = 128, .run_standby = run_standby,
                     .sense = AcSense::both});
    ChPad::source(EvPitDiv<1024>{});
    EvOut<PadPin>::listen(ChPad{});
    console_drain();
    delay_us(clock, 40'000u);                  // the comparator settled on a toggling pad
    AcU::clear_flag();
    AcU::enable_interrupt(true);
    const SleepRun r = sleep_once(SleepMode::standby);
    AcU::enable_interrupt(false);
    AcU::disable();
    EvOut<PadPin>::unlisten();
    ChPad::off();
    PadPin::configure({});
    return r;
}

#if !defined(BRIO_BOARD_ea48)
struct AdcLeg {
    uint16_t awake;      ///< a software-started conversion before the sleep
    SleepRun run;
    uint16_t slept;      ///< the RESRDY result of the event-started one
    uint16_t freed;      ///< awake again, the op amp released, the pull-up alone
};

/// One standby with ADC0 started by the event system, converting OP0 -
/// or, on a part with no op amp (the DA), the pad with its pull-up alone.
AdcLeg adc_leg(bool adc_standby, bool op_standby) {
    AdcLeg l{};
#ifdef OPAMP
    OpampSystem::init(clock);
    (void)OpU::init({.positive = OpampPos::vdd_div2, .negative = OpampNeg::out,
                     .output = OpampOutput::normal, .mode = OpampMode::software,
                     .run_standby = op_standby});
#else
    (void)op_standby;
#endif
    // The pull-up on the output pad: the op amp overrides it while on;
    // off, it takes the pad to VDD - the two states a conversion tells
    // apart. The digital input buffer stays off (the op amp's analog pad).
    PadPin::configure({.pull = PinPull::up, .sense = PinSense::input_disable});
#ifdef OPAMP
    OpU::wait_settled();
#else
    // No op amp: the pad is DRIVEN high from PORT instead - a low-impedance
    // VDD the converter reads at full scale asleep as awake (a pull-up
    // alone is tens of kilohms, which the sampling capacitor loads down
    // differently from one conversion to the next: measured 3962 awake,
    // 3908 asleep).
    PadPin::set();
    PadPin::output();
    delay_us(clock, 1000u);
#endif
    (void)AdcU::init(clock, {.reference = Ref::vdd, .run_standby = adc_standby});
    AdcU::select(AnalogIn<PadPin>{});
    l.awake = AdcU::read();
    adc_last = 0;
    AdcU::enable_resrdy_interrupt(true);
    ChPad::source(EvPitDiv<1024>{});
    AdcU::start_on(ChPad{});
    l.run = sleep_once(SleepMode::standby);
    l.slept = adc_last;
    AdcU::start_on_events(false);
    AdcU::enable_resrdy_interrupt(false);
    ChPad::off();
    // The control: the same pad with the op amp released (disabled,
    // OUTMODE off), the pull-up alone on it.
#ifdef OPAMP
    OpU::release();
#endif
    PadPin::configure({.pull = PinPull::up, .sense = PinSense::input_disable});
    delay_us(clock, 1000u);
    l.freed = AdcU::read();
    AdcU::disable();
#ifdef OPAMP
    OpampSystem::disable();
#else
    PadPin::clear();
    PadPin::input();
#endif
    PadPin::configure({});
    return l;
}

/// One sleep with TCD0's OVF interrupt armed.
SleepRun tcd_leg(SleepMode m, TcdClock src) {
    (void)TcdU::init({.clock = src, .sync_prescaler = TcdSyncPrescaler::div8,
                      .count_prescaler = TcdCountPrescaler::div32,
                      .waveform = TcdWaveform::one_ramp, .compare_b_clear = 4095});
    TcdU::enable_ovf_interrupt(true);
    const SleepRun r = m == SleepMode::power_down ? power_down_once(11) : sleep_once(m);
    TcdU::enable_ovf_interrupt(false);
    (void)TcdU::disable();
    return r;
}
#endif

#if defined(BRIO_BOARD_ea48)
// The AVR EA: TCA0 and AC0 as on the DA/DB. It has no op amp and no TCD;
// its ADC is another engine (adc.hpp's second half) whose standby leg
// is not written here yet (docs/avrxt/adc.md) - those legs are the DA/DB's.
void to_runstdby() {
    print(serial, "o RUNSTDBY of the timers and the analog, in a real STANDBY: TCA, AC "
                  "(the AVR EA: no op amp, no TCD, the ADC's leg not written)", crlf);
    quiesce();

    // TCA0: the asleep count differs from the awake one only by the wake
    // path - CLK_PER kept alive by the TCA's own request, no oscillator
    // restart - and 64 counts (4096 CLK_PER, 205 us at 20 MHz) bound that
    // tenfold. Without the flag only the 2 ms console drain before the
    // sleep counts (625 counts at 20 MHz / 64), plus 1/100 of the period.
    const uint16_t tca_awake = tca_leg(false, true);
    const uint16_t tca_on = tca_leg(true, false);
    const uint16_t tca_off = tca_leg(false, false);
    print(serial, "  TCA0 at CLK_PER/64 over one PIT period: awake ", tca_awake,
          ", standby with RUNSTDBY ", tca_on, ", without ", tca_off, " (nominal ",
          tca_per_pit, ")", crlf);
    verdict("TCA0 with RUNSTDBY counts through STANDBY as it does awake",
            near_u32(tca_on, tca_awake, 64));
    verdict("without it TCA0 stops with CLK_PER: only the code outside the sleep counts",
            tca_off < 750u + tca_per_pit / 100u);

    // AC0 on PD2's 32 Hz square (EVOUTD from the PIT), its CMP interrupt
    // against the PIT's 125 ms: the standby table (13-1) does not list the
    // AC among the units RUNSTDBY keeps, the wake-up table (13-3) and the
    // AC's own chapter (30.3.5) do.
    const SleepRun ac_on = ac_leg(true);
    const SleepRun ac_off = ac_leg(false);
    print(serial, "  AC0 on PD2's 32 Hz square: with RUNSTDBY ", ac_on.units,
          " CMP interrupt(s), ", ac_on.pits, " PIT; without ", ac_off.units, " / ",
          ac_off.pits, crlf);
    verdict("AC0 with RUNSTDBY: its CMP interrupt ends STANDBY, the PIT never fires "
            "(30.3.5 and table 13-3, against table 13-1's omission)",
            ac_on.units >= 1 && ac_on.pits == 0);
    verdict("without it the comparator sleeps and the PIT ends the sleep",
            ac_off.units == 0 && ac_off.pits == 1);
    print(serial, "  the ADC's RUNSTDBY: SKIP - the EA's ADC engine is not written; the op "
                  "amp and the TCD: this die has neither", crlf);
    quiesce();
}
#else
void to_runstdby() {
    print(serial, "o RUNSTDBY of the analog and the timers, in a real STANDBY: TCA, AC, the "
                  "event-started ADC, the op amp; the TCD stops", crlf);
    quiesce();

    // TCA0. Both counts measure the same PIT period (OSC32K's error
    // cancels): the asleep one differs from the awake one only by the
    // wake-up path - CLK_PER kept alive by the TCA's own request, so no
    // oscillator restart, a few microseconds - and 64 counts (4096 CLK_PER
    // cycles, 171 us) bounds that tenfold. Without the flag the count is
    // what runs outside the sleep: sleep_once's 2 ms console drain (750
    // counts) plus test c's 1/100 of the period for the rest.
    const uint16_t tca_awake = tca_leg(false, true);
    const uint16_t tca_on = tca_leg(true, false);
    const uint16_t tca_off = tca_leg(false, false);
    print(serial, "  TCA0 at CLK_PER/64 over one PIT period: awake ", tca_awake,
          ", standby with RUNSTDBY ", tca_on, ", without ", tca_off, " (nominal ",
          tca_per_pit, ")", crlf);
    verdict("TCA0 with RUNSTDBY counts through STANDBY as it does awake",
            near_u32(tca_on, tca_awake, 64));
    verdict("without it TCA0 stops with CLK_PER: only the code outside the sleep counts",
            tca_off < 750u + tca_per_pit / 100u);

    // AC0. The square toggles every 15.6 ms, the PIT ends the sleep at
    // 125 ms: a comparator alive in standby wakes the CPU on the first
    // edge - eight edges inside the window - and the PIT count is 0.
    const SleepRun ac_on = ac_leg(true);
    const SleepRun ac_off = ac_leg(false);
    print(serial, "  AC0 on PD2's 32 Hz square: with RUNSTDBY ", ac_on.units,
          " CMP interrupt(s), ", ac_on.pits, " PIT; without ", ac_off.units, " / ",
          ac_off.pits, crlf);
    verdict("AC0 with RUNSTDBY: its CMP interrupt ends STANDBY, the PIT never fires "
            "(32.3.5 holds, not 32.5.1's 'not updated')", ac_on.units >= 1 && ac_on.pits == 0);
    verdict("without it the comparator sleeps and the PIT ends the sleep",
            ac_off.units == 0 && ac_off.pits == 1);

    // ADC0 + OP0. The awake conversion of the follower reads VDD/2 against
    // VDD: 2048 of 4096. An event-started conversion in standby must read
    // the same pad within 32 counts (39 mV at 5 V - far above the
    // converter's few counts of noise, far below the 2048 counts to the
    // pull-up's rail). With the op amp off in standby nothing holds VDD/2:
    // the pull-up moves the pad more than 1024 counts (a quarter of the
    // scale, thirty times the band a driven pad stays in) above it; the
    // control - the op amp released, the pull-up alone - says where the
    // pull-up takes the pad.
#ifdef OPAMP
    const AdcLeg both = adc_leg(true, true);
    const AdcLeg op_off = adc_leg(true, false);
    const AdcLeg adc_off = adc_leg(false, true);
    print(serial, "  ADC0 on OP0 (follower of VDD/2, pull-up on its pad): awake ", both.awake,
          "; standby, both RUNSTDBY: ", both.run.units, " RESRDY, ", both.run.pits, " PIT, result ",
          both.slept, crlf);
    print(serial, "  op amp without RUNSTBY: awake ", op_off.awake, ", in standby ",
          op_off.run.units, " RESRDY, ", op_off.run.pits, " PIT, result ", op_off.slept,
          "; ADC without RUNSTDBY: ", adc_off.run.units, " RESRDY, ", adc_off.run.pits,
          " PIT", crlf);
    print(serial, "  the pad with the op amp released and the pull-up alone: ", both.freed, ", ",
          op_off.freed, ", ", adc_off.freed, crlf);
    verdict("the follower reads VDD/2 awake (2048 of 4096, within 64)",
            near_u32(both.awake, 2048, 64));
    verdict("an event-started conversion runs in STANDBY with the ADC's RUNSTDBY: its "
            "RESRDY ends the sleep, the PIT never fires",
            both.run.units >= 1 && both.run.pits == 0);
    verdict("and it converts the op amp kept on by RUNSTBY: the awake value within 32",
            near_u32(both.slept, both.awake, 32));
    verdict("the op amp WITHOUT RUNSTBY is off in standby: nothing holds VDD/2, the "
            "pull-up moved the pad", op_off.run.units >= 1 &&
            op_off.slept > static_cast<uint16_t>(op_off.awake + 1024u));
    // A finding, not a contract: an op amp switched off BY THE SLEEP
    // (OUTMODE still normal) keeps loading its pad - the pull-up reaches
    // some 3550 counts - where the same op amp RELEASED (disabled,
    // OUTMODE off) leaves the pull-up the rail. Within 95 counts of full scale: the
    // converter's few counts of noise and no load.
    verdict("the control: released (disabled, OUTMODE off), the pull-up alone takes "
            "the pad to the rail", op_off.freed > 4000u && both.freed > 4000u);
    verdict("the ADC without RUNSTDBY converts nothing in standby: the PIT ends the "
            "sleep", adc_off.run.units == 0 && adc_off.run.pits == 1);
#else
    // No op amp on this part (the DA): the event-started conversion
    // reads the pad driven high from PORT, VDD against VDD - full scale,
    // within 95 counts of 4095 (the converter's few counts of noise, no
    // load). What stays measurable is the ADC's own RUNSTDBY.
    const AdcLeg on = adc_leg(true, false);
    const AdcLeg off = adc_leg(false, false);
    print(serial, "  ADC0 on PD2 driven high (no op amp on this part): awake ", on.awake,
          "; standby with RUNSTDBY: ", on.run.units, " RESRDY, ", on.run.pits,
          " PIT, result ", on.slept, "; without: ", off.run.units, " RESRDY, ",
          off.run.pits, " PIT", crlf);
    verdict("the driven pad reads the rail awake (above 4000 of 4096)", on.awake > 4000u);
    verdict("an event-started conversion runs in STANDBY with the ADC's RUNSTDBY: its "
            "RESRDY ends the sleep, the PIT never fires",
            on.run.units >= 1 && on.run.pits == 0);
    verdict("and converts the pad as it reads awake (within 32)",
            near_u32(on.slept, on.awake, 32));
    verdict("the ADC without RUNSTDBY converts nothing in standby: the PIT ends the "
            "sleep", off.run.units == 0 && off.run.pits == 1);
    print(serial, "  the op amp's RUNSTBY: SKIP - this part has no OPAMP", crlf);
#endif

    // TCD0. Its ramp overflows every 43.7 ms (24 MHz / 8 / 32, 4096
    // counts), inside the 125 ms PIT window: in IDLE its OVF ends the
    // sleep; in STANDBY the PIT does, from CLK_PER and from OSCHF with
    // the oscillator's own RUNSTDBY set (25.3.6: the TCD stops in
    // standby - it has no RUNSTDBY of its own).
    const SleepRun tcd_idle = tcd_leg(SleepMode::idle, TcdClock::clkper);
    const SleepRun tcd_std = tcd_leg(SleepMode::standby, TcdClock::clkper);
    (void)Oschf::set_hz(24'000'000);
    Oschf::run_standby(true);
    const SleepRun tcd_osc = tcd_leg(SleepMode::standby, TcdClock::oschf);
    const SleepRun tcd_pd = tcd_leg(SleepMode::power_down, TcdClock::oschf);
    print(serial, "  TCD0 OVF every 43.7 ms: IDLE ", tcd_idle.units, " OVF / ", tcd_idle.pits,
          " PIT; STANDBY on CLK_PER ", tcd_std.units, " / ", tcd_std.pits,
          "; STANDBY on OSCHF (RUNSTDBY) ", tcd_osc.units, " / ", tcd_osc.pits,
          "; POWER-DOWN on OSCHF ", tcd_pd.units, " / ", tcd_pd.pits, crlf);
    verdict("in IDLE the TCD's OVF ends the sleep", tcd_idle.units >= 1 && tcd_idle.pits == 0);
    verdict("in STANDBY the TCD stops on CLK_PER: the PIT ends the sleep",
            tcd_std.units == 0 && tcd_std.pits == 1);
    verdict("and on OSCHF kept running by its own RUNSTDBY too",
            tcd_osc.units == 0 && tcd_osc.pits == 1);
    verdict("and in POWER-DOWN", tcd_pd.units == 0 && tcd_pd.pits == 1);
    quiesce();
}
#endif

#if defined(BRIO_BOARD_ea48)
// ---- p every self-driven pin wakes from power-down on ONE edge (the AVR EA) -----------------
// DS40002443A 18.3.3.1: every pin of the EA senses fully asynchronously,
// so one edge of ANY pin wakes it from power-down - on the DA/DB only Px2
// and Px6 do, the rest needing both edges or a level (their 18.3.3.1).
// Each pad below is driven by the PIT's 32 Hz divider through the event
// system while every clock but the 32 kHz one is stopped - an EVOUT
// position, or a LUT's output on an asynchronous path (no filter, no edge
// detector: table 13-3 note 3) - and senses its own drive: FALLING alone,
// then RISING alone, one power-down sleep each, a 1 s PIT backstop behind
// it. PE3 shares PC7's net and senses it too. Five of the eight pads are
// neither Px2 nor Px6; every power-down is bracketed by the watchdog.
struct PadWake {
    uint16_t awake;            ///< the pad's interrupts in 40 ms awake, both edges sensed
    SleepRun fall, rise;       ///< the two power-down sleeps
    uint16_t fall_n, rise_n;   ///< the pad's interrupts across each
};

template <typename Sensed>
PadWake pad_wake(volatile uint16_t& counter, uint8_t stage) {
    PadWake w{};
    Sensed::configure({.sense = PinSense::both});
    Sensed::clear_flag();
    console_drain();
    cli();
    uint16_t c0 = counter;
    sei();
    delay_us(clock, 40'000u);                 // two edges or three of the 32 Hz square
    cli();
    w.awake = static_cast<uint16_t>(counter - c0);
    sei();
    Sensed::configure({.sense = PinSense::falling});
    Sensed::clear_flag();
    cli();
    c0 = counter;
    sei();
    w.fall = power_down_once(stage, false);
    cli();
    w.fall_n = static_cast<uint16_t>(counter - c0);
    sei();
    Sensed::configure({.sense = PinSense::rising});
    Sensed::clear_flag();
    cli();
    c0 = counter;
    sei();
    w.rise = power_down_once(static_cast<uint8_t>(stage + 1), false);
    cli();
    w.rise_n = static_cast<uint16_t>(counter - c0);
    sei();
    Sensed::configure({});
    Sensed::clear_flag();
    return w;
}

/// A LUT that copies event input A (the PIT divider on ChPin) to its
/// output pin, on an asynchronous path, with the block's RUNSTDBY.
template <typename L>
bool lut_drive(bool alt) {
    Ccl::disable();
    const bool ok = L::init({.in0 = LutInput::event_a,
                             .truth = lut_truth([](bool a, bool, bool) { return a; }),
                             .output_pin = true,
                             .alt_pin = alt});
    L::event_a_on(ChPin{});
    Ccl::enable(true);
    return ok;
}

bool pad_woke(const PadWake& w) {
    return w.awake >= 1 && w.fall_n >= 1 && w.fall.pits == 0 && w.rise_n >= 1 &&
           w.rise.pits == 0;
}

void print_pad(const char* what, const PadWake& w) {
    print(serial, "    ", what, ": awake ", w.awake, " edge(s) in 40 ms; power-down FALLING ",
          w.fall_n, " pad / ", w.fall.pits, " PIT, RISING ", w.rise_n, " pad / ", w.rise.pits,
          " PIT", crlf);
}

void tp_pin_wake() {
    print(serial, "p every pin a self-made edge reaches wakes this board from POWER-DOWN on "
                  "ONE edge (every pin senses asynchronously on this die)", crlf);
    quiesce();
    Pit::period(PitPeriod::cyc32768);         // the 1 s backstop
    Pit::enable_interrupt(true);
    ChPad::source(EvPitDiv<1024>{});          // 32 Hz: an edge every 15.6 ms
    ChPin::source(EvPitDiv<1024>{});          // the same divider, the same slot

    constexpr uint8_t n = 8;
    const char* const what[n] = {
        "PD2 (EVOUTD, a Px2)         ", "PD7 (EVOUTD ALT1)           ",
        "PA7 (EVOUTA ALT1)           ", "PC7 (EVOUTC ALT1)           ",
        "PE3 (PC7's net, PC7 driving)", "PA6 (LUT0 ALT1, a Px6)      ",
        "PD3 (LUT2)                  ", "PD6 (LUT2 ALT1, a Px6)      ",
    };
    PadWake w[n];
    EvOut<PadPin>::listen(ChPad{});
    w[0] = pad_wake<PadPin>(pad_irqs, 20);
    EvOut<PadPin>::unlisten();
    EvOut<PinD7>::listen(ChPad{});
    w[1] = pad_wake<PinD7>(pad_irqs, 22);
    EvOut<PinD7>::unlisten();
    EvOut<PinA7>::listen(ChPad{});
    w[2] = pad_wake<PinA7>(porta_irqs, 24);
    EvOut<PinA7>::unlisten();
    EchoPin::input();                         // one driver per net: PC7 drives, PE3 listens
    EvOut<PinC7>::listen(ChPad{});
    w[3] = pad_wake<PinC7>(portc_irqs, 26);
    w[4] = pad_wake<EchoPin>(wake_irqs, 28);
    EvOut<PinC7>::unlisten();
    EchoPin::clear();
    EchoPin::output();
    ChPad::off();
    const bool l0 = lut_drive<PortaLut>(true);
    w[5] = pad_wake<PinA6>(porta_irqs, 30);
    Ccl::disable();
    (void)PortaLut::init({});                 // PA6 released before the block runs again
    PortaLut::disable();
    const bool l1 = lut_drive<PortdLut>(false);
    w[6] = pad_wake<PinD3>(pad_irqs, 32);
    const bool l2 = lut_drive<PortdLut>(true);
    w[7] = pad_wake<PinD6>(pad_irqs, 34);
    Ccl::disable();

    verdict("the three LUT drivers came up", l0 && l1 && l2);
    for (uint8_t i = 0; i < n; ++i) print_pad(what[i], w[i]);
    bool all = true;
    for (uint8_t i = 0; i < n; ++i) all = all && pad_woke(w[i]);
    verdict("every pad toggles awake, and ONE falling edge and ONE rising edge each "
            "woke the device from POWER-DOWN before the 1 s backstop", all);
    verdict("among them five pins that are neither Px2 nor Px6 (PD7, PA7, PC7, PE3, "
            "PD3): no DA/DB pin of those positions wakes on a single edge there",
            pad_woke(w[1]) && pad_woke(w[2]) && pad_woke(w[3]) && pad_woke(w[4]) &&
                pad_woke(w[6]));
    verdict("and a LUT's asynchronous output drives its pad through POWER-DOWN (PA6, "
            "PD3, PD6)", pad_woke(w[5]) && pad_woke(w[6]) && pad_woke(w[7]));
    quiesce();
}

// ---- q the partner's edges on more pins (the AVR EA) ----------------------------------------
// The partner reaches four nets of this board, two pads each (the PC4..PC7
// jumpers). A start bit on the PE0/PC4 net - the peer's sfd_byte action,
// 0x00 at 9600, the line low for nine bit times - is ONE falling edge on
// two pins that are neither Px2 nor Px6, each sensed alone in turn; the
// stimulus train letter k lands on PE2 is sensed on PC6 instead, its echo
// raised by the PORTC vector, so the peer times that wake too.
template <typename Sensed>
bool start_bit_wake(PinPull pull, volatile uint16_t& counter, uint8_t stage, SleepRun& r,
                    uint16_t& got) {
    (void)link_command_mode();
    slink::Params a{};
    a.value = 0x00;
    a.rate = 9'600;
    a.delay_ms = 60;
    a.deadline_ms = 200;
    if (!peer_act(slink::Op::sfd_byte, a)) return false;
    Sensed::configure({.pull = pull, .sense = PinSense::falling});
    Sensed::clear_flag();
    cli();
    const uint16_t c0 = counter;
    sei();
    r = power_down_once(stage, false);
    cli();
    got = static_cast<uint16_t>(counter - c0);
    sei();
    Sensed::configure({.pull = pull});
    Sensed::clear_flag();
    delay_us(clock, 20'000);                  // the byte's tail and the peer's restore
    (void)link_command_mode();
    return true;
}

void tq_peer_pins() {
    print(serial, "q the partner's edges wake this board from POWER-DOWN on more pins: a "
                  "start bit on PE0 and on PC4, the stimulus on PC6", crlf);
    quiesce();
    if (!ensure_link()) { verdict("the peer board is reachable", false); return; }
    Pit::period(PitPeriod::cyc32768);         // the 1 s backstop

    SleepRun r_e0{}, r_c4{};
    uint16_t n_e0 = 0, n_c4 = 0;
    const bool ok_e0 = start_bit_wake<Pin<'E', 0>>(PinPull::none, wake_irqs, 40, r_e0, n_e0);
    const bool ok_c4 = start_bit_wake<LinePin>(PinPull::up, portc_irqs, 41, r_c4, n_c4);
    print(serial, "    PE0 FALLING: ", n_e0, " pad / ", r_e0.pits, " PIT;  PC4 FALLING: ",
          n_c4, " pad / ", r_c4.pits, " PIT", crlf);
    verdict("the peer sent both start bits", ok_e0 && ok_c4);
    verdict("ONE falling edge on PE0 woke the device from POWER-DOWN, before the 1 s "
            "backstop", ok_e0 && n_e0 >= 1 && r_e0.pits == 0);
    verdict("and one on PC4", ok_c4 && n_c4 >= 1 && r_c4.pits == 0);

    // PC6: the stimulus net, PE2's sense off - one sensing pad per net.
    Train t{};
    bool ok = train_arm(3, 250);
    WakePin::configure({});
    Pin<'C', 6>::configure({.sense = PinSense::rising});
    Pin<'C', 6>::clear_flag();
    (void)Port<'E'>::take_flags();
    uint16_t woke = 0;
    if (ok) {
        Pit::enable_interrupt(false);         // no wake but the stimulus (run_leg())
        token.magic = token_magic;
        token.stage = 42;
        (void)Watchdog::arm(WdtTime::s8);
        for (uint8_t i = 0; i < 3; ++i) {
            cli();
            const uint16_t c0 = portc_irqs;
            sei();
            bool got = false;
            for (uint8_t k = 0; k < 8 && !got; ++k) {
                Sleep::enter(SleepMode::power_down);
                cli();
                got = portc_irqs != c0;
                sei();
            }
            if (got) ++woke;
            delay_us(clock, 50);
            EchoPin::clear();
        }
        (void)Watchdog::off();
        token.magic = 0;
        token.stage = 0;
        Pit::enable_interrupt(true);
    }
    Pin<'C', 6>::configure({});
    train_woke = woke;
    ok = ok && train_collect(t);
    verdict("the link carried the stimulus train", ok);
    if (ok) print_train("PC6 from power-down (crystal main clock)", t);
    verdict("every stimulus on PC6 woke the device from POWER-DOWN and its echo came back",
            ok && t.woke == 3 && t.hits == 3);
    quiesce();
}
#endif

// ---- the menu -------------------------------------------------------------------------------

void register_tests() {
    bench.letter('a', "the register surface", ta_surface);
    bench.letter('b', "idle through enter()", tb_idle);
    bench.letter('c', "standby is real", tc_standby);
    bench.letter('d', "the RUNSTDBY chain", td_chain);
    bench.letter('e', "a pin wakes from the deep modes", te_pin_wake);
    bench.letter('f', "power-down stops the rest", tf_power_down);
#if defined(BRIO_BOARD_ea48)
    bench.letter('g', "the wake-up in 32 kHz ticks", tg_wake_ticks);
#else
    bench.letter('g', "the voltage regulator", tg_vreg);
#endif
    bench.letter('n', "the CCL as a wake-up source", tn_ccl_wake);
#if defined(BRIO_BOARD_ea48)
    bench.letter('o', "RUNSTDBY of TCA and AC", to_runstdby);
    bench.letter('p', "every self-driven pin wakes from power-down on one edge", tp_pin_wake);
#else
    bench.letter('o', "RUNSTDBY of TCA, AC, ADC, op amp; the TCD", to_runstdby);
#endif
    bench.second_letter('h', "the link and the two wires", th_link);
    bench.second_letter('i', "the awake baseline and idle", ti_idle_latency);
    bench.second_letter('j', "standby timed from outside", tj_standby_latency);
    bench.second_letter('k', "power-down timed from outside", tk_power_down_latency);
    bench.second_letter('l', "start-of-frame", tl_sfd);
    bench.second_letter('m', "a TWI address match", tm_twi_wake);
#if defined(BRIO_BOARD_ea48)
    bench.second_letter('q', "the peer's edges on PE0, PC4, PC6", tq_peer_pins);
#endif
}

void help() {
    print(serial, "test_avr_sleep: z = the single-board letters, y = the two-board ones (the "
                  "peer board = sleep_peer)", crlf);
    bench.menu();
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

ISR(RTC_PIT_vect) {
    (void)Pit::take_flag();
    pit_irqs = pit_irqs + 1;
}
ISR(RTC_CNT_vect) {
    (void)Rtc::take_flags();
    rtc_irqs = rtc_irqs + 1;
}
ISR(PORTD_PORT_vect) {
    (void)Port<'D'>::take_flags();
    pad_irqs = pad_irqs + 1;
}

/// THE WAKE-UP ISR OF THE TWO-BOARD HALF, and the one place in this
/// suite where the ORDER of two statements is a measurement. The echo
/// goes up FIRST - a single-cycle SBI on a low-I/O VPORT register - so
/// everything the peer reports is the silicon's wake-up plus this
/// vector's own latency and prologue, and nothing else. The flag clear
/// and the counter follow.
ISR(PORTE_PORT_vect) {
    EchoPin::set();
    (void)Port<'E'>::take_flags();
    wake_irqs = wake_irqs + 1;
}

#if defined(BRIO_BOARD_ea48)
/// The AVR EA's link is USART1 (ALT1, PC4): its RXC vector carries RXSIF
/// (DS40002443A 25.3.4.2), and the body is the DA/DB's below.
ISR(USART1_RXC_vect) {
    if (LinkUart::rxs_flag()) {
        LinkUart::clear_rxs();
        sfd_irqs = sfd_irqs + 1;
    }
}

/// Letter p's PA6/PA7.
ISR(PORTA_PORT_vect) {
    (void)Port<'A'>::take_flags();
    porta_irqs = porta_irqs + 1;
}

/// Letter p's PC7 and letter q's PC4 and PC6. The echo first, as in the
/// PORTE vector: letter q times PC6's wake through it (PE3 is an input
/// whenever PC7 is driven, so the store is inert there).
ISR(PORTC_PORT_vect) {
    EchoPin::set();
    (void)Port<'C'>::take_flags();
    portc_irqs = portc_irqs + 1;
}
#else
/// USART4's RXC vector carries RXSIF too (27.5.6): with SFDEN armed and
/// RXSIE on, THIS is what a start bit on the shared wire wakes into. It
/// must not read RXDATA - errata 2.16.2 says a read during a reception
/// restarts the frame - so it only clears the start flag and counts.
ISR(USART4_RXC_vect) {
    if (LinkUart::rxs_flag()) {
        LinkUart::clear_rxs();
        sfd_irqs = sfd_irqs + 1;
    }
}
#endif

/// The TWI client of test m, serviced entirely from its vector: the
/// address match that wakes the device and every byte of the frame are
/// one uninterrupted piece of work, so the stretch the peer measures
/// is the wake-up and nothing else.
ISR(TWI0_TWIS_vect) {
    const auto s = Client::isr();
    if (s.address_or_stop()) {
        if (s.is_address()) {
            twis_addr = twis_addr + 1;
            twis_n = 0;
            Client::respond(TwiAck::ack);
        } else {
            twis_stops = twis_stops + 1;
            Client::complete();
        }
    } else if (s.data()) {
        if (s.host_reading()) {
            Client::transmit(0x5A);
        } else {
            const uint8_t v = Client::receive(TwiAck::ack);
            if (twis_n < 8) {
                twis_rx[twis_n] = v;
                twis_n = twis_n + 1;
            }
        }
    }
}

// Test o's units: each body clears its own flag and counts.
ISR(AC0_AC_vect) {
    (void)AcU::cmp();
    unit_irqs = unit_irqs + 1;
}
#if !defined(BRIO_BOARD_ea48)
ISR(ADC0_RESRDY_vect) {
    adc_last = AdcU::resrdy();
    unit_irqs = unit_irqs + 1;
}
ISR(TCD0_OVF_vect) {
    TcdU::ovf();
    unit_irqs = unit_irqs + 1;
}
#endif

ISR(CCL_CCL_vect) {
    (void)Ccl::take_flags();
    ccl_irqs = ccl_irqs + 1;
}

// All four TCB vectors are bound as a net: an unbound vector on this
// core is a jump to 0, i.e. a silent reset loop, and the deep sleeps
// here are exactly the place where a stray flag would find one.
ISR(TCB0_INT_vect) {
    (void)Waker::take_flags();
    waker_irqs = waker_irqs + 1;
}
ISR(TCB1_INT_vect) { (void)WatchLo::take_flags(); }
ISR(TCB2_INT_vect) { (void)WatchHi::take_flags(); }
ISR(TCB3_INT_vect) { (void)Tcb<3>::take_flags(); }

int main() {
    const ResetFlags why = Reset::take_flags();
    const bool hung = token.magic == token_magic && token.stage != 0;
    const uint8_t hung_stage = token.stage;
    token.magic = 0;
    token.stage = 0;

    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    RtcClock::select(RtcSource::osc32k);
    Pit::init(pit_period);
    sei();

    auto board = board_id();
    if (board.empty()) board = "?";
#if defined(BRIO_BOARD_ea48)
    print(serial, crlf, "test_avr_sleep - sleep test suite (board ", board,
          ", clk=", xtal ? "XTAL" : "OSCHF", " ", crystal_hz / 1'000'000u,
          " MHz, silicon rev ", hex(SYSCFG.REVID), ", RSTFR=", hex(why.raw), ")", crlf);
    print(serial, "  BOD.CTRLA ", hex(BOD.CTRLA), " (from FUSE.BODCFG; SLEEP, bits 1:0, = 1: "
                  "the BOD runs continuously in standby and power-down, its current in every "
                  "sleep here)", crlf);
#else
    print(serial, crlf, "test_avr_sleep - sleep test suite (board ", board,
          ", clk=", xtal ? (has_xoschf ? "XTAL" : "EXTCLK") : "OSCHF", " 24 MHz, silicon rev ", hex(SYSCFG.REVID),
          ", RSTFR=", hex(why.raw), ")", crlf);
#endif
    print(serial, "  PD1/PD2 must be free of the bus jumpers: test e drives PD2 from the "
                  "event system and senses its own edge.", crlf);
    print(serial, "  the two-board set y needs the peer board running sleep_peer: PE0 the shared "
                  "command wire, PE2 its stimulus in, PE3 the echo out, PA2/PA3 the bus.",
          crlf);
    if (hung) {
        print(serial, "  PREVIOUS RUN DID NOT WAKE: a power-down sleep (stage ", hung_stage,
              ") was ended by the watchdog", why.watchdog ? "" : " - though RSTFR does not "
              "name the watchdog", ". That is a FINDING, not a pass.", crlf);
    }
    register_tests();
    help();

    bench.prompt();
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (!bench.handle(static_cast<char>(c))) { print(serial, "? for help", crlf); }
        bench.prompt();
    }
}
