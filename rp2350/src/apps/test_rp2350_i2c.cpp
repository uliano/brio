// test_rp2350_i2c - the reference bench suite for this chip's I2C
// (rp2350/i2c.hpp over datasheet 12.2, the Synopsys DW_apb_i2c v2.03a):
// the block WIRELESS, THIS CHIP'S PADS - the isolation latch, the pad
// register the driver writes, the unstick's hand-over - on the high half
// of the bank, above GP31, and then the host and the client on a wire to a
// SECOND CHIP: every tenure shape, the vocabulary produced on the wire, the
// three speeds measured, the DMA engines, util/i2c_bus.hpp's arbiter with
// its timed bus, the stuck bus, arbitration between two hosts, and this
// board as the far host's client. ONE SOURCE,
// BOTH ARCHITECTURES: nothing in chapter 12.2 is a processor's, so every
// verdict below is expected to read the same on the Cortex-M33 half and on
// the Hazard3 half.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// THE WIRE IS THE DESK PARTNER'S. I2C0 on GP36 (SDA) / GP37 (SCL) is
// wired to the WeAct RP2040's I2C1, each line with 1.5 kOhm to THAT
// board's 3V3 (docs/boards/weact-rp2350b.md). That board runs `twi_peer`
// (rp2040/src/apps/twi_peer.cpp), commanded IN BAND over the bus under
// test through avrxt/src/apps/twi_link.hpp; the two-board letters are the
// ones both Raspberry Pi suites share, rp2040/src/apps/rp_i2c_pair.hpp,
// whose header states what they ask of a suite. Every letter that drives
// the link reads the partner's rail first through
// rp2350/src/apps/pair_power.hpp (erratum RP2350-E9's own workaround: the
// pad's pull-down, its input buffer pulsed) and drives nothing when it
// reads low; letter j reads it as one of its verdicts.
//
// THE RULER IS SYSTEM TIMER 0 (rp2350/timer.hpp). THE UNSTICK HAS A
// RULER OF ITS OWN - the platform timer, which rp2350/delay.hpp counts
// because it is the one counter both architectures have - and main
// starts it, on both halves, because on the Cortex-M33 half nothing else
// does.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: 9.4 as the driver states it over the whole
//      bank, the timing arithmetic at this clk_sys (table 1106's floors
//      reproduced), the reset state, the FIFO depths the block reports,
//      the refusals, the enable and the disable through IC_ENABLE_STATUS
//   b  THE SCAN: every address 0x08..0x77 probed, the peer's 0x6B alone
//      answering, nobody-home as i2c_nack_addr at the address's NACK
//   c  THE PEER: ident (kind 0x08, on its crystal), thirty command round
//      trips, ten at each speed
//   d  the tenure shapes against the peer: write, read, write-then-read
//      with the repeated START counted from the far end, the probe, reads
//      across the peer's refills, a 200-byte write through the pump
//   e  the vocabulary: nack_addr from a deaf client, 0x7A, the general
//      call and `dma` on the engineless host refused inside start() with
//      nothing on the wire, nack_data at the first and at the third data
//      byte, the client's stretch priced on a read and absent on a write
//   f  THE THREE SPEEDS byte-exact both ways and MEASURED on the ruler
//   g  THE DMA ENGINES (channels 4 and 5), every request naming its
//      engine: a 64-byte read timed, a write-then-read, reads of one and
//      two bytes on the receive engine, writes under `dma` on the pump,
//      `interrupt` at 64 bytes touching no channel, the fault path, the
//      bursts, and a data NACK with the read half's engines launched
//   r  A RELEASED REQUESTER, THE SAME OWNER AGAIN: the host released with its DMA
//      enables set, initialized again, no request standing and one engined read
//      exact - and the control with no reset, the channel moving at once
//   h  THE KERNEL: I2cBus over I2cHost, the NACK in its place, the
//      rejection, both votes, and THE TIMED BUS - the peer holding SCL -
//      answered by the arbiter's per-bus timeout
//   i  THE STUCK BUS: the peer holding SDA, unstick() counting the clocks
//      until it lets go, then a line nine clocks cannot free
//   j  THE PADS, which is where this chip differs from its predecessor,
//      on GP36/GP37: the link's pull-ups read the way E9 forces, the
//      isolation latch dropped by the driver's own hand-over, the whole
//      pad register read back against 12.2.1.3, an address phase nobody
//      answers carried by them, the unstick's hand-over through the HIGH
//      half's SIO registers and back, and both pads left isolated again
//      by release()
//   k  ARBITRATION on the wire, both directions, and an engined read
//      losing it
//   l  THE ROLES INVERT: this board's client written to and read from by
//      the peer's host, and a general call commanded of that host refused
//      there, nothing reaching this client
//   y  A PURE WRITE ON THE TRANSMIT ENGINE, its bytes in byte beats through
//      IC_DATA_CMD's zero-filling alias (2.1.5): 3 to 255 bytes at the three
//      speeds, a read's halfword commands between them on the same channel,
//      the partner judging every byte at its position; and the control, the
//      channel bound at the register's own address
//   n  a NACK in mid-write, at the first byte and on the address, the channel
//      running; the control, the same write with nothing refused
//   o  a lost arbitration with the channel launched; the control, the other
//      direction
//   v  a bus error on the channel, a run that leaves the map; the control, the
//      next write from RAM
//   x  (outside z) the hand-made window traced: what the block reports of
//      a START and a STOP it did not make, and a tenure written inside it
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/dma.hpp"
#include "rp2350/flash.hpp"
#include "rp2350/i2c.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: one file for every board
// that speaks it.
#include "../../../avrxt/src/apps/twi_link.hpp"

#include "pair_power.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;
constexpr SysClock clock;
using P = Rp2350Platform<>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Ruler = Timer<0>;

TestBench<Serial> bench;

// I2C0 on the pair's wires: GP37 (SCL), GP36 (SDA).
constexpr I2cPins link_pins{.scl = 37, .sda = 36};
using Host = I2cHost<0, link_pins>;
using DmaTx = DmaTxEngine<4, uint16_t>;
using DmaRx = DmaRxEngine<5>;
using DmaHost = I2cHost<0, link_pins, DmaTx, DmaRx>;
using Client = I2cClient<0, link_pins>;
using Raw = DwApbI2c<0>;
using SdaPin = Pin<36>;
using SclPin = Pin<37>;
constexpr uint8_t partner_kind = twilink::peer_rp2040;
constexpr const char* partner_text = "the WeAct RP2040";

uint32_t us_now() { return Ruler::now_low(); }
/// Every line of the XIP cache dropped: letter g's cold read.
void xip_empty() { Xip::invalidate_all(); }
const char* arch_name() { return core_kind == CoreKind::hazard3 ? "Hazard3" : "Cortex-M33"; }

// THE TWO-BOARD LETTERS, over the names above.
#include "../../../rp2040/src/apps/rp_i2c_pair.hpp"

/// One address-only tenure (the probe) at 400 kHz, waited out.
uint8_t probe(uint8_t addr) { return tenure<Host>(addr, nullptr, 0, nullptr, 0, I2cSpeed::fast_400k); }


// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    bench.verdict("9.4 as the driver states it over the WHOLE bank: I2C0 SDA on 0/4/../44 and SCL on "
                  "1/5/../45, I2C1 SDA on 2/6/../46 and SCL on 3/7/../47, one function column and no "
                  "second one",
                  i2c_sda_pin(0, 0) && i2c_sda_pin(0, 44) && i2c_scl_pin(0, 1) && i2c_scl_pin(0, 45) &&
                      i2c_sda_pin(1, 2) && i2c_sda_pin(1, 46) && i2c_scl_pin(1, 3) && i2c_scl_pin(1, 47) &&
                      !i2c_sda_pin(0, 2) && !i2c_scl_pin(1, 13) && !i2c_sda_pin(1, 48) &&
                      !i2c_pins_valid(0, {.scl = 12, .sda = 13}) &&
                      Rp2350DwApbI2c::pad_function == PinFunction::i2c);
    for (uint8_t s = 0; s < i2c_speed_count; ++s) {
        const auto sp = static_cast<I2cSpeed>(s);
        const auto t = i2c_timing_for(SysClock::hz, sp);
        print(serial, "  ", i2c_speed_hz(sp) / 1000u, " kHz at ", SysClock::hz / 1'000'000u, " MHz: HCNT ",
              t ? t->hcnt : 0u, " LCNT ", t ? t->lcnt : 0u, " SPKLEN ", t ? t->spklen : 0u, " hold ",
              t ? t->sda_tx_hold : 0u, " setup ", t ? t->sda_setup : 0u, " -> ",
              t ? i2c_scl_hz(SysClock::hz, *t) : 0u, " Hz on an ideal wire", crlf);
    }
    bench.verdict("the timing arithmetic: the three speeds solved at this clk_sys produce their asked rate "
                  "on an ideal wire (the SPKLEN + 8 cycles the block adds subtracted from the counts)",
                  i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::standard_100k)) == 100'000u &&
                      i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::fast_400k)) >= 399'000u &&
                      i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::fast_plus_1m)) == 1'000'000u);
    bench.verdict("table 1106's floors reproduced: fast mode at 12 MHz is (LCNT 15, HCNT 6), standard at "
                  "2.7 MHz is (12, 6), fast-mode-plus needs 32 MHz, a hertz below each is refused",
                  i2c_timing_for(12'000'000u, I2cSpeed::fast_400k)->lcnt == 15u &&
                      i2c_timing_for(12'000'000u, I2cSpeed::fast_400k)->hcnt == 6u &&
                      i2c_timing_for(2'700'000u, I2cSpeed::standard_100k)->lcnt == 12u &&
                      !i2c_timing_for(11'999'999u, I2cSpeed::fast_400k).has_value() &&
                      !i2c_timing_for(31'999'999u, I2cSpeed::fast_plus_1m).has_value() &&
                      i2c_timing_for(32'000'000u, I2cSpeed::fast_plus_1m).has_value());
    using S = DwApbI2c<0>;
    const bool reset_ok = S::reset();
    const uint32_t con = S::con();
    print(serial, "  I2C0 after reset: IC_CON=", hex(con), " SPKLEN=", S::timing(I2cSpeed::fast_400k).spklen,
          " SDA_HOLD=", S::timing(I2cSpeed::fast_400k).sda_tx_hold, " enabled=", S::enabled(),
          " status=", hex(S::flags()), " IC_COMP_PARAM_1=", hex(S::regs().IC_COMP_PARAM_1),
          " line=", static_cast<uint32_t>(S::irq()), crlf);
    bench.verdict("the block's reset state: a host with its client half disabled, fast speed code, restarts "
                  "enabled (IC_CON 0x65), the spike filter at 7, the hold at 1, disabled, both FIFOs empty - "
                  "and the component parameter register reading zero, as 12.2 leaves it",
                  reset_ok && con == 0x65u && S::timing(I2cSpeed::fast_400k).spklen == 7u &&
                      S::timing(I2cSpeed::fast_400k).sda_tx_hold == 1u && !S::enabled() && S::tx_empty() &&
                      !S::rx_not_empty() && S::regs().IC_COMP_PARAM_1 == 0u);
    bench.verdict("the two interrupt lines are 3.2's, which are NOT the RP2040's 23 and 24: I2C0_IRQ is 36 "
                  "and I2C1_IRQ is 37, the same numbers under both architectures (3.8.4.2)",
                  static_cast<uint32_t>(DwApbI2c<0>::irq()) == 36u &&
                      static_cast<uint32_t>(DwApbI2c<1>::irq()) == 37u);
    // The transmit FIFO's depth, measured: entries queued under
    // TX_CMD_BLOCK until the FIFO says full, then flushed by a disable.
    S::enable();
    S::command_block(true);
    uint8_t depth = 0;
    while (S::tx_not_full() && depth < 64u) {
        S::push(i2c_write_entry(depth));
        ++depth;
    }
    const uint8_t counted = S::tx_count();
    const bool over = (S::raw_pending() & I2cInterrupt::tx_over) == 0u;
    S::push(i2c_write_entry(0xFF));   // one too many
    const bool over_now = (S::raw_pending() & I2cInterrupt::tx_over) != 0u;
    (void)S::disable();
    S::command_block(false);
    S::clear_all();
    print(serial, "  the transmit FIFO under TX_CMD_BLOCK: full after ", depth, " entries (TXFLR ", counted,
          "), TX_OVER ", over_now ? "raised" : "NOT raised", " on the next; after the disable TXFLR ",
          S::tx_count(), crlf);
    bench.verdict("the transmit FIFO is sixteen deep (12.2.1), measured under TX_CMD_BLOCK: full at sixteen, "
                  "TX_OVER on the seventeenth, flushed by the disable",
                  depth == 16u && counted == 16u && over && over_now && S::tx_count() == 0u);
    const bool cfg = S::configure({.speed = I2cSpeed::fast_plus_1m}) &&
                     S::timing(*i2c_timing_for(SysClock::hz, I2cSpeed::fast_plus_1m)) && S::target(0x42);
    const I2cTiming back = S::timing(I2cSpeed::fast_plus_1m);
    const I2cTiming asked = *i2c_timing_for(SysClock::hz, I2cSpeed::fast_plus_1m);
    print(serial, "  a configuration read back: HCNT ", back.hcnt, " LCNT ", back.lcnt, " (asked for ",
          asked.hcnt, " and ", asked.lcnt, "), TAR ", hex(S::target()), crlf);
    bench.verdict("a configuration, a timing and a target are taken with the block disabled and read back "
                  "as they were written",
                  cfg && back.hcnt == asked.hcnt && back.lcnt == asked.lcnt &&
                      back.spklen == asked.spklen && S::target() == 0x42u &&
                      (S::con() & I2C_IC_CON_SPEED_BITS) == (2u << I2C_IC_CON_SPEED_LSB));
    S::enable();
    const bool running = S::running();
    const bool refused = !S::configure({}) && !S::timing(back) && !S::target(0x10) && !S::own_address(0x11);
    (void)S::target(0x10);   // ignored by the hardware too
    bench.verdict("enabled: IC_ENABLE_STATUS reports it, and the configuration, the timing, the target and "
                  "the own address are refused",
                  running && refused && S::target() == 0x42u);
    const uint32_t t0 = us_now();
    const bool off = S::disable();
    const uint32_t took = us_now() - t0;
    print(serial, "  disable on an idle bus: ", off ? "reported" : "NOT reported", " in ", took, " us", crlf);
    bench.verdict("the disable of an idle block is reported by IC_ENABLE_STATUS within a few microseconds",
                  off && took < 20u);
    S::hold();
}

// =============================================================================
// j - the pads, and the isolation latch this chip added
// =============================================================================
void tj_pads() {
    const pair::PartnerRail rail = pair::partner_rail(clock);
    print(serial, "  GP36 and GP37 on their own pull-down, the buffer pulsed (E9's workaround): SDA ",
          rail.sda ? "HIGH" : "low", ", SCL ", rail.scl ? "HIGH" : "low", crlf);
    bench.verdict("THE LINK'S PULL-UPS READ THE WAY E9 FORCES: both lines high through 1.5 kOhm to the "
                  "RP2040's 3V3 against the pad's own pull-down - the partner powered, the pads free to drive",
                  rail.sda && rail.scl);
    if (!rail.up()) {
        print(serial, "  THE PARTNER IS UNPOWERED: no link pad is driven (an RP2040 pad is not failsafe)", crlf);
        return;
    }
    host_ready();
    const uint32_t pad36 = Gpio::pad(36);
    const uint32_t pad37 = Gpio::pad(37);
    const bool fn_ok = SdaPin::function() == PinFunction::i2c && SclPin::function() == PinFunction::i2c;
    const bool iso_ok = !SdaPin::isolated() && !SclPin::isolated();
    const bool ie_ok = SdaPin::input_enabled() && SclPin::input_enabled();
    const uint32_t wanted = pad_value(i2c_pad_config);
    print(serial, "  after init: GP36 pad=", hex(pad36), " GP37 pad=", hex(pad37), " (the driver asks for ",
          hex(wanted), "), FUNCSEL ", static_cast<uint32_t>(SdaPin::function()), "/",
          static_cast<uint32_t>(SclPin::function()), ", isolated ", SdaPin::isolated(), "/", SclPin::isolated(),
          crlf);
    bench.verdict("THE HAND-OVER DROPS THE ISOLATION LATCH: both pads read back FUNCSEL 3 with ISO clear, "
                  "the input buffer on, and the whole pad register exactly what 12.2.1.3 asks for - the "
                  "pull-up, the slew limit, the Schmitt trigger",
                  fn_ok && iso_ok && ie_ok && pad36 == wanted && pad37 == wanted);
    // The pads carry an address phase: the host clocks nine SCL periods
    // and reads the ACK slot high, which needs both pads working - SCL
    // driven and released, SDA released and read back.
    const uint8_t st = probe(nobody);
    print(serial, "  the probe of ", hex(nobody), ", which nothing on the bus answers: status ", st, crlf);
    bench.verdict("... and the pads so configured carry an address phase: nobody home is i2c_nack_addr",
                  st == i2c_nack_addr);
    // The unstick takes the pads by hand - through the HIGH half's SIO
    // registers, these being GP36 and GP37 - and gives them back.
    const uint8_t cleared = Host::unstick();
    const uint32_t pad36_after = Gpio::pad(36);
    const uint32_t pad37_after = Gpio::pad(37);
    const bool fn_back = SdaPin::function() == PinFunction::i2c && SclPin::function() == PinFunction::i2c;
    const uint8_t st2 = probe(nobody);
    print(serial, "  after unstick() (", cleared, "): GP36 pad=", hex(pad36_after), " GP37 pad=", hex(pad37_after),
          ", the next probe ", st2, crlf);
    bench.verdict("the unstick reads a healthy bus above GP31 (0), takes both pads to SIO and hands them back "
                  "to the block with the same pad register: the probe after it lands unchanged",
                  cleared == 0u && pad36_after == wanted && pad37_after == wanted && fn_back && st2 == i2c_nack_addr);
    owner = Owner::none;
    Host::release();
    const uint32_t pad36_released = Gpio::pad(36);
    print(serial, "  after release(): GP36 pad=", hex(pad36_released), " FUNCSEL ",
          static_cast<uint32_t>(SdaPin::function()), crlf);
    bench.verdict("release() puts the pads back where a reset leaves them (9.11's 0x116: isolated, the "
                  "input buffer off, a pull-down) with no owner",
                  pad36_released == PADS_BANK0_GPIO0_RESET && SdaPin::isolated() && !SdaPin::input_enabled() &&
                      SdaPin::function() == PinFunction::none && SclPin::isolated() &&
                      SclPin::function() == PinFunction::none);
}

// =============================================================================
// y, n, o, v - a pure write on the transmit engine, through the alias
// =============================================================================

/// The engined host up with no other owner on the vector, a peer's serve
/// already running.
void write_host_ready() {
    owner = Owner::none;
    Host::release();
    (void)DmaHost::init(clock);
    owner = Owner::dma_host;
}

/// The peer's report over the pump, the engined host given back first.
bool write_host_done(const twilink::Params& a, uint32_t since_us, twilink::Report& r) {
    owner = Owner::none;
    DmaHost::release();
    host_ready();
    outlast(a, since_us);
    return peer_report(r);
}

/// `n` bytes of the serve's pattern from position `from` into tx_buf, and
/// their sum.
uint32_t pattern_run(uint8_t seed, uint16_t from, uint16_t n) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < n; ++i) {
        tx_buf[i] = twilink::pattern_value(twilink::pattern_counting, seed, static_cast<uint16_t>(from + i));
        sum += tx_buf[i];
    }
    return sum;
}

/// One write under `dma`, its bytes the serve's pattern from `from`.
uint8_t pattern_write(uint8_t seed, uint16_t from, uint16_t n, I2cSpeed speed, uint32_t& sum) {
    sum += pattern_run(seed, from, n);
    // The wire's time twice over at nine periods a byte, and 5 ms for the
    // handlers: a 255-byte write at 100 kHz is 46 ms + 5.
    const uint32_t wait = 18'000'000u / i2c_speed_hz(speed) * (n + 1u) + 5000u;
    return tenure<DmaHost>(twilink::dut_addr, tx_buf, static_cast<uint8_t>(n), nullptr, 0, speed, wait, I2cEngine::dma);
}

// y - the bytes, judged by the partner at each position.
void ty_write_bytes() {
    if (!need_peer()) {
        return;
    }
    const twilink::Params a = serve_params(800, 0x00);
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve command", false);
        return;
    }
    uint32_t s0 = us_now();
    write_host_ready();
    struct Run {
        uint16_t n;
        I2cSpeed speed;
    };
    const Run runs[] = {{3, I2cSpeed::fast_400k},       {4, I2cSpeed::fast_400k},  {17, I2cSpeed::fast_400k},
                        {18, I2cSpeed::fast_400k},      {64, I2cSpeed::fast_400k}, {255, I2cSpeed::fast_400k},
                        {255, I2cSpeed::standard_100k}, {255, I2cSpeed::fast_plus_1m}};
    uint16_t sent = 0;
    uint16_t served = 0;
    uint32_t sum = 0;
    uint8_t clean = 0;
    bool over = false;
    uint8_t read_ok = 0;
    uint8_t reads = 0;
    for (const Run& run : runs) {
        const uint32_t t0 = us_now();
        const uint8_t st = pattern_write(a.seed, sent, run.n, run.speed, sum);
        const uint32_t took = us_now() - t0;
        const uint32_t lines = dma_entries;
        const uint32_t entries = isr_entries;
        over = over || (Raw::raw_pending() & I2cInterrupt::tx_over) != 0u;
        sent = static_cast<uint16_t>(sent + run.n);
        print(serial, "  ", run.n, " bytes under dma at ", i2c_speed_hz(run.speed) / 1000u, " kHz: status ", st, " in ", took,
              " us, ", lines, " DMA line entry, ", entries, " host entries", crlf);
        // One block, so one DMA line entry; the STOP, and TX_EMPTY when the
        // block's end found no room for the last entry.
        if (st == i2c_ok && lines == 1u && entries >= 1u && entries <= 2u) {
            ++clean;
        }
        // THE SAME CHANNEL NEXT, ITS OTHER WIDTH: a read's halfword commands
        // through the alias, the beat and the step moved and moved back.
        if (run.n >= 18u) {
            for (uint8_t i = 0; i < 16u; ++i) {
                rx_buf[i] = 0xEE;
            }
            const uint8_t sr = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 16, I2cSpeed::fast_400k, 20'000u,
                                               I2cEngine::dma);
            ++reads;
            if (sr == i2c_ok && pattern_mismatches(rx_buf, 16, a.seed, served) == 0u) {
                ++read_ok;
            }
            served = static_cast<uint16_t>(served + 16u);
        }
    }
    bench.verdict("A PURE WRITE UNDER dma RIDES THE CHANNEL: 3 to 255 bytes at the three speeds i2c_ok, one DMA line "
                  "entry, the STOP and at most one TX_EMPTY on the host's vector, TX_OVER never raised",
                  clean == sizeof runs / sizeof runs[0] && !over);
    bench.verdict("... and the same channel between them carries a read's halfword commands through the same address, "
                  "each read of 16 exact",
                  reads != 0u && read_ok == reads);
    // WHAT STAYS ON THE PUMP UNDER dma: no DMA line entry.
    const uint8_t s1 = pattern_write(a.seed, sent, 1, I2cSpeed::fast_400k, sum);
    const uint32_t l1 = dma_entries;
    sent = static_cast<uint16_t>(sent + 1u);
    const uint8_t s2 = pattern_write(a.seed, sent, 2, I2cSpeed::fast_400k, sum);
    const uint32_t l2 = dma_entries;
    sent = static_cast<uint16_t>(sent + 2u);
    bench.verdict("writes of one and two bytes under dma stay on the pump: no byte between the first entry and the "
                  "last, no DMA line entry, i2c_ok",
                  s1 == i2c_ok && s2 == i2c_ok && l1 == 0u && l2 == 0u);
    twilink::Report r{};
    bool rep = write_host_done(a, s0, r);
    print(serial, "  the peer received ", static_cast<uint16_t>(r.count - served), " of ", sent, ", served ", r.aux1, " (",
          served, "), sum ", hex(r.sum), " (", hex(static_cast<uint16_t>(sum)), "), first ", hex(r.first), ", ", r.sstatus,
          " off the pattern at their position", crlf);
    bench.verdict("THE PARTNER JUDGES THE BYTES: every one received once, the sum exact, none off the counting pattern "
                  "at its position",
                  rep && static_cast<uint16_t>(r.count - served) == sent && r.aux1 == static_cast<uint8_t>(served) &&
                      r.sum == static_cast<uint16_t>(sum) && r.sstatus == 0u);
    // THE CONTROL: the channel bound at the register's own address. The
    // pattern's byte 0x01 is a read command there and 0x02 a STOP, so the same
    // write does not move exact bytes - the partner's count says so.
    if (!need_peer()) {
        return;
    }
    const twilink::Params c = serve_params(300, 0x00);
    if (!peer_act(Op::serve, c)) {
        bench.verdict("the peer accepted the serve command", false);
        return;
    }
    s0 = us_now();
    write_host_ready();
    DmaTx::arm(Raw::data_address(), Raw::dreq_tx, false);
    uint32_t csum = 0;
    const uint8_t cst = pattern_write(c.seed, 0, 32, I2cSpeed::fast_400k, csum);
    (void)DmaHost::recover();
    twilink::Report cr{};
    rep = write_host_done(c, s0, cr);
    print(serial, "  the control, 32 bytes with the channel at the register's own address: status ", cst, ", the peer "
          "received ", cr.aux0, ", served ", cr.aux1, ", ", cr.sstatus, " off the pattern", crlf);
    bench.verdict("THE CONTROL: at the register's own address the same write is not those bytes - the replicated lanes "
                  "make read commands of some, the partner's count short",
                  rep && cr.aux0 != 32u && cr.aux1 != 0u);
}

// n - a NACK in mid-write, the channel running.
void tn_write_nack() {
    if (!need_peer()) {
        return;
    }
    // The peer stores nack_at - 1 and refuses the next, whatever the FIFO
    // holds ahead of it. The longest write, so that the channel is still
    // pouring into the flushed FIFO when the host's vector comes: refused at the
    // twentieth, some 217 of its 253 entries remain, where a write of 64 leaves
    // 44, which were gone before the vector on the Cortex-M33 - and the abort's
    // stop is judged only on a block that outlasts the vector (the rest falls
    // into the flushed FIFO whether the channel is stopped or not).
    twilink::Params a = serve_params(300, 0x10);
    a.nack_at = 20;
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the nack-at serve", false);
        return;
    }
    uint32_t s0 = us_now();
    write_host_ready();
    uint32_t sum = 0;
    const uint8_t st = pattern_write(a.seed, 0, 255, link_speed, sum);
    const uint32_t lines = dma_entries;
    const uint32_t entries = isr_entries;
    const bool stopped = !DmaTx::busy();
    // A read on the engines right after: the abort left them bound and free.
    for (uint8_t i = 0; i < 16u; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t sr = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 16, link_speed, 200'000u, I2cEngine::dma);
    const uint8_t mr = pattern_mismatches(rx_buf, 16, a.seed, 0);
    twilink::Report r{};
    bool rep = write_host_done(a, s0, r);
    const uint32_t stored_sum = pattern_run(a.seed, 0, 19);
    print(serial, "  255 under dma, the twentieth refused: status ", st, ", ", lines, " DMA line entries, ", entries,
          " host entries, the channel ", stopped ? "stopped" : "RUNNING", "; the peer stored ", r.aux0, " (sum ", hex(r.sum),
          " of ", hex(static_cast<uint16_t>(stored_sum)), "), ", r.sstatus, " off the pattern; the next engined read ", sr,
          ", ", mr, " wrong", crlf);
    // The abort flushes the transmit FIFO and holds it so until it is read
    // away, which leaves the request standing: what remains of the block pours
    // in microseconds into nothing, so its end may reach the DMA line before
    // the host's vector stops the channel, one entry or none.
    bench.verdict("A NACK IN MID-WRITE: i2c_nack_data on the host's one entry, at most one on the DMA line, the channel "
                  "stopped, nineteen bytes stored and exact",
                  st == i2c_nack_data && lines <= 1u && entries == 1u && stopped && rep && r.aux0 == 19u &&
                      r.sum == static_cast<uint16_t>(stored_sum) && r.sstatus == 0u);
    bench.verdict("... and the next engined read is exact: the abort stopped the channel before the FIFOs came out of "
                  "their flushed state, no byte of the block starting a tenure of its own",
                  sr == i2c_ok && mr == 0u && rep && r.aux1 == 16u);
    // THE FIRST BYTE REFUSED AND THE ADDRESS REFUSED, the block barely started.
    if (!need_peer()) {
        return;
    }
    twilink::Params f = serve_params(300, 0x10);
    f.nack_at = 1;
    if (!peer_act(Op::serve, f)) {
        bench.verdict("the peer accepted the nack-at serve", false);
        return;
    }
    s0 = us_now();
    write_host_ready();
    uint32_t sum1 = 0;
    const uint8_t s1 = pattern_write(f.seed, 0, 32, link_speed, sum1);
    const uint8_t s2 = tenure<DmaHost>(nobody, tx_buf, 32, nullptr, 0, link_speed, 200'000u, I2cEngine::dma);
    const bool free = !DmaTx::busy();
    twilink::Report fr{};
    rep = write_host_done(f, s0, fr);
    print(serial, "  the first byte refused: status ", s1, ", the peer stored ", fr.aux0, "; nobody home on a write of 32 under "
          "dma: status ", s2, crlf);
    bench.verdict("the first byte refused is i2c_nack_data with nothing stored, a deaf address on a write under dma "
                  "i2c_nack_addr, each with the channel stopped",
                  s1 == i2c_nack_data && s2 == i2c_nack_addr && free && rep && fr.aux0 == 0u);
    // THE CONTROL: no refusal, the same write after the aborts: all of it.
    if (!need_peer()) {
        return;
    }
    const twilink::Params c = serve_params(300, 0x10);
    if (!peer_act(Op::serve, c)) {
        bench.verdict("the peer accepted the serve command", false);
        return;
    }
    s0 = us_now();
    write_host_ready();
    uint32_t csum = 0;
    const uint8_t cs = pattern_write(c.seed, 0, 255, link_speed, csum);
    const uint32_t cl = dma_entries;
    twilink::Report cr{};
    rep = write_host_done(c, s0, cr);
    print(serial, "  the control, no refusal: status ", cs, ", ", cl, " DMA line entry, the peer stored ", cr.aux0, crlf);
    bench.verdict("THE CONTROL: with nothing refused the same write is i2c_ok, its block ended, all 255 stored exact",
                  cs == i2c_ok && cl == 1u && rep && cr.aux0 == 255u && cr.sstatus == 0u &&
                      cr.sum == static_cast<uint16_t>(csum));
}

// o - arbitration with the channel launched.
void to_write_arbitration() {
    if (!need_peer()) {
        return;
    }
    struct Round {
        uint8_t target;
        bool lose;
    };
    // Letter k's two addresses: 0x6B against the peer's 0x2C loses at the first
    // bit, 0x11 wins at the second; letter k's window of 350 us.
    const Round rounds[] = {{twilink::command_addr, true}, {twilink::low_addr, false}};
    for (const Round& rd : rounds) {
        twilink::Params a{};
        a.ms = 600;
        a.addr = twilink::dut_addr;
        a.target = twilink::dut_addr;
        a.count = 4;
        a.seed = 0x40;
        a.aux16 = 120;
        if (!peer_act(Op::arb, a)) {
            bench.verdict("the peer accepted the arbitration command", false);
            return;
        }
        const uint32_t s0 = us_now();
        write_host_ready();
        fill_pattern(tx_buf, 255, 0xD0);
        const uint8_t st = arb_tenure<DmaHost>(rd.target, tx_buf, 255, nullptr, 0, 350, I2cEngine::dma);
        const bool stopped = !DmaTx::busy();
        twilink::Report r{};
        const bool rep = write_host_done(a, s0, r);
        const bool peer_lost = rep && (r.flags & twilink::report_arblost) != 0u;
        const bool peer_ran = rep && (r.flags & twilink::report_host_ran) != 0u;
        print(serial, "  a write of 255 under dma -> ", hex(rd.target), " against the peer -> 0x2C: this board ", st,
              ", the channel ", stopped ? "stopped" : "RUNNING", "; the peer: flags ", hex(r.flags), " status ", r.mstatus, crlf);
        if (rd.lose) {
            bench.verdict("A LOST ARBITRATION ON THE CHANNEL: aiming at the higher address, i2c_arb_lost, the channel "
                          "stopped, the peer's host keeping the bus",
                          st == i2c_arb_lost && stopped && peer_ran && !peer_lost && r.mstatus == i2c_nack_addr);
        } else {
            bench.verdict("THE CONTROL, the other direction: aiming at the lower address this host keeps the bus and "
                          "nobody answers (i2c_nack_addr), the peer's host the one that loses",
                          st == i2c_nack_addr && stopped && peer_lost);
        }
    }
    bench.verdict("the command channel is back after the races", command(Op::ping));
}

// v - a bus error on the channel.
void tv_write_fault() {
    if (!need_peer()) {
        return;
    }
    // THE LAST BYTE OF SRAM (2.2.3: SRAM_END is 0x20082000): start() reads it
    // for the first entry, and the channel's first beat reads the address past
    // it, which nothing answers: a read bus error on the channel's first beat.
    const auto* const edge = reinterpret_cast<const uint8_t*>(0x20081FFFu);
    const twilink::Params a = serve_params(300, 0x30);
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve command", false);
        return;
    }
    const uint32_t s0 = us_now();
    write_host_ready();
    const uint8_t sf = tenure<DmaHost>(twilink::dut_addr, edge, 16, nullptr, 0, I2cSpeed::fast_400k, 20'000u, I2cEngine::dma);
    const uint32_t lines = dma_entries;
    const uint32_t entries = isr_entries;
    const bool stopped = !DmaTx::busy();
    twilink::Report r{};
    bool rep = write_host_done(a, s0, r);
    print(serial, "  a write of 16 from the last byte of SRAM: status ", sf, " (i2c_dma_fault ", i2c_dma_fault, "), ", lines,
          " DMA line entry, ", entries, " host entries, the channel ", stopped ? "stopped" : "RUNNING", "; the peer stored ",
          r.aux0, crlf);
    // The bound: the one entry start() wrote is all the block ever held, and the
    // abort may cut it: nothing stored, or its one byte.
    bench.verdict("A BUS ERROR ON THE CHANNEL: a write whose run leaves the map is i2c_dma_fault, on the DMA line alone, "
                  "the channel stopped, at most its one first byte reaching the partner",
                  sf == i2c_dma_fault && lines == 1u && entries == 0u && stopped && rep && r.aux0 <= 1u);
    // THE CONTROL: the same request from a buffer the map holds.
    if (!need_peer()) {
        return;
    }
    const twilink::Params c = serve_params(300, 0x30);
    if (!peer_act(Op::serve, c)) {
        bench.verdict("the peer accepted the serve command", false);
        return;
    }
    const uint32_t c0 = us_now();
    write_host_ready();
    uint32_t sum = 0;
    const uint8_t cs = pattern_write(c.seed, 0, 16, I2cSpeed::fast_400k, sum);
    twilink::Report cr{};
    rep = write_host_done(c, c0, cr);
    print(serial, "  the control, 16 bytes from RAM: status ", cs, ", the peer stored ", cr.aux0, ", ", cr.sstatus,
          " off the pattern", crlf);
    bench.verdict("THE CONTROL: the next write from a buffer in the map is i2c_ok and exact, the channel back in service",
                  cs == i2c_ok && rep && cr.aux0 == 16u && cr.sstatus == 0u && cr.sum == static_cast<uint16_t>(sum));
}

void banner() {
    print(serial, crlf, "test_rp2350_i2c - the RP2350 I2C (datasheet 12.2, DW_apb_i2c v2.03a) on the ",
          arch_name(), " half: the block wireless, then I2C0 on GP37/GP36 against the RP2040's twi_peer; clk_sys=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_i2c0() { pair_i2c_isr(); }
extern "C" void isr_dma_0() { pair_dma_isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer<0>::init(clock);
    // The unstick's half-bits are paced by the platform timer
    // (rp2350/delay.hpp), which the Hazard3 half's ticker starts and the
    // Cortex-M33 half's does not. Starting it here is idempotent and
    // makes the two halves the same program.
    const bool ruler_ok = brio::Mtime::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    const bool dma_ok = brio::Dma::init();
    brio::enable_interrupts();

    // Every letter that drives the link reads the partner's rail first
    // (pair_power.hpp); j reads it itself, as one of its verdicts.
    using pair::guarded;
    bench.letter('a', "the block, wireless", ta_block);
    bench.letter('b', "the scan", guarded<bench, clock, tb_scan>);
    bench.letter('c', "the peer: ident and the command channel", guarded<bench, clock, tc_peer>);
    bench.letter('d', "the tenure shapes", guarded<bench, clock, td_shapes>);
    bench.letter('e', "the vocabulary on the wire, the stretch priced", guarded<bench, clock, te_vocabulary>);
    bench.letter('f', "the three speeds, measured", guarded<bench, clock, tf_speeds>);
    bench.letter('g', "the DMA engines", guarded<bench, clock, tg_dma>);
    bench.letter('r', "a released requester: the same owner initialized again", guarded<bench, clock, tr_released_requester>);
    bench.letter('h', "the kernel: I2cBus over I2cHost, the timed bus", guarded<bench, clock, th_kernel>);
    bench.letter('i', "the stuck bus", guarded<bench, clock, ti_unstick>);
    bench.letter('j', "the pads and the isolation latch, above GP31", tj_pads);
    bench.letter('k', "arbitration on the wire", guarded<bench, clock, tk_arbitration>);
    bench.letter('l', "the roles invert: this board as the peer's client", guarded<bench, clock, tl_roles>);
    bench.letter('m', "the client's edges: the overrun, the disable inside a tenure", guarded<bench, clock, tm_client_edges>);
    bench.letter('y', "a pure write on the channel: the bytes, judged by the partner", guarded<bench, clock, ty_write_bytes>);
    bench.letter('n', "a pure write on the channel: a NACK in mid-write", guarded<bench, clock, tn_write_nack>);
    bench.letter('o', "a pure write on the channel: a lost arbitration", guarded<bench, clock, to_write_arbitration>);
    bench.letter('v', "a pure write on the channel: a bus error", guarded<bench, clock, tv_write_fault>);
    bench.letter('x', "the hand-made window, traced (outside z)", guarded<bench, clock, tx_window>, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: arch=", arch_name(), " clk=", clock_ok ? "PLL150" : "FAILED",
                    " timer0=", timer_ok ? "1us" : "FAILED", " ruler=", ruler_ok ? "1us" : "FAILED",
                    " tick=", tick_ok ? "on" : "FAILED", " dma=", dma_ok ? "released" : "FAILED", brio::crlf);
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
        brio::print(serial, static_cast<char>(c), brio::crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
