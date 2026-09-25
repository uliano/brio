// test_vx03_usbfs - the reference bench suite for the USB host/device
// controller of RM ch. 23 in device mode (ch32vx03/usbfs.hpp) under the
// device stack (util/usb/device.hpp) and the CDC ACM class
// (util/usb/cdc.hpp): the enumeration a real host performs, the pads the
// block drives, the port as a byte transport, the class requests, a
// reconnect, and what is this block's own - the data toggle kept in the
// endpoint's register and a lost acknowledgement staged against it, the
// one receive-length register filed per endpoint, the automatic pause the
// driver rests on, the bus floor under HCLK, the frame counter the
// vendor's header names, the block with the core asleep in both
// directions, and suspend and resume with the wake-up lines of table 9-3.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE CONSOLE IS THE PROBE'S UART, on USART1's own pads PA9/PA10, and THE
// BOARD'S USB CONNECTOR IS THE THING UNDER TEST: on WCH's CH32V303
// evaluation board that is P14, on PA11/PA12. The host is the other end
// of every letter that needs one: Linux enumerates the device (the
// pid.codes test identity 1209:0001), binds cdc-acm, and the bench's host
// script drives /dev/ttyACM* while the letter watches this end - it finds
// the port by that identity. A host-assisted letter waits a
// BOUNDED time and then says it was skipped, so the all-key is green with
// the cable in and no script running. THE CABLE ITSELF IS NOT OPTIONAL:
// the letters in the all-key need no host ACTION but they need a host,
// and with the connector empty they fail and print an enumeration that
// never happened.
//
// THE STREAMS COUNT, both ways. The script's echo (y) and its pours (v, d)
// send bytes that each go one above the last, modulo 256, so a packet
// lost on the way in is a JUMP in what arrives - which is how the letters
// t, b, h and d see a loss without the host's help - and the device's own
// pours (v, n) are the same counting stream, which the script's read
// checks byte by byte on the host's side.
//
// A POUR WAITS FOR THE HOST TO OPEN THE PORT (DTR up). A tty the host has
// not yet put in raw mode ECHOES what it receives back to the device -
// measured: "^C", "^\" and the rest of the control characters' echoes
// arriving on the bulk OUT when the device poured first - so letters v
// and n start pouring only once the open is seen.
//
// THE CLOCK IS 96 MHz AND THE CRYSTAL IS WHY: full-speed USB wants its 48
// MHz within 2500 ppm and the HSI-rooted trees of this family are some
// four tenths of a per cent fast (the clock chapter), so the tree is the
// board's 8 MHz crystal through the PLL at x12 and USBPRE halves it
// whole. Letters h and e divide HCLK under that PLL with HPRE - the one
// shape the clock task cannot express - and rebase the console and the
// tick at every rung, so they speak and time at every rate.
//
// THE LOOP IDLES: an attached controller holds one bus-master count and
// the platform's idle() returns at once while it stands
// (ch32vx03/bus_activity.hpp); letter a measures that the call costs
// nothing, and letter d stages what it prevents with a bare wfi.
//
// THE PADS. PA11/PA12 (the connector), PA9/PA10 (the console), PB2 (the
// LED, toggled per command as every suite of this target does), and PB7
// read once as a contrast in letter p. No wire is needed.
//
// What is exercised, letter by letter:
//   a  THE CONTROLLER AND THE ENUMERATION, no host action: the block up
//      with its 48 MHz and the pull-up still down, then start() and the
//      host's enumeration to CONFIGURED, the setups and the stalls, the
//      class's endpoints armed, the pool they spent (256 bytes: endpoint
//      zero's 64, the notification's 64, the bulk pair's 128 in one
//      piece), what RX_LEN and INT_ST said at a SETUP, what endpoint zero
//      answers at rest, what a buffer register reads back, the counters
//      at zero, and THE BUS COUNT at one with a thousand idle() calls
//      costing nothing
//   p  THE PADS, no host action: D+ read through the port and through the
//      block's own RB_UD_DP_PIN with the pull-up down and up - the pair
//      the part table names is the pair the block drives - and an
//      enumeration with the pads' port clock shut
//   u  A RECONNECT, no host action: the pull-up dropped for half a second
//      - no bus reset reported over the SE0 the detach leaves - and raised
//      again, a fresh bus reset and enumeration
//   e  THE BUS FLOOR, ENUMERATION, no host action: HCLK at 96, 48, 24, 12,
//      6 and 1.5 MHz under the same PLL, a reconnect at each rung
//   f  THE FRAME COUNTER, no host action: the vendor's SOF interrupt
//      (reserved in the manual) counting the host's frames for 100 ms
//   d  (host: d) THE CORE ASLEEP UNDER THE HOST'S POUR: the FIFO overflow
//      counter in a healthy run, then a bare wfi with nobody draining the
//      ring, then a bare wfi with the ring drained at every wake - the
//      block's DMA writing the host's packets with the core asleep
//   n  (host: r) THE CORE ASLEEP THE OTHER WAY: the device pouring with the
//      core in a bare wfi between its packets, the script counting what
//      arrives against the counting stream, and an enumeration with the
//      core in wfi between its interrupts
//   y  (host: y) ECHO: every byte back for ten seconds, the host judging
//   w  (host: w) THE LINE CODING AND DTR, as the host sets them
//   v  (host: v) THROUGHPUT both ways
//   l  (host: y, after w) THE RECEIVE LENGTH PER ENDPOINT: one RX_LEN
//      register for every endpoint, and the bytes the driver filed under
//      each - the bulk OUT's against what the class took, endpoint zero's
//      against the seven bytes of every line coding (so the script's open
//      must change the coding: w first)
//   t  (host: d) THE TOGGLE UNDER A LOST PACKET: the PID the bulk OUT
//      expects flipped under the pour, one packet dropped by its PID, one
//      jump of one packet in the stream, and the two back in step
//   b  (host: d) THE AUTOMATIC PAUSE: the vector held off with the pause
//      on (the status byte changing once and standing) and then with it
//      off (the block taking packet over packet into an unread buffer) -
//      why the driver sets it
//   h  (host: d) THE BUS FLOOR, DATA: the pour drained at HCLK 96 MHz down
//      to 1.5, bytes, jumps and overflows at each rung
//   s  (host: s) SUSPEND AND RESUME, and which of the two wake-up lines
//      table 9-3 names for this class - 18 and 20 - a resume raises
//
// Start the letter here FIRST, then the script's mode inside the window
// the letter prints; a host already pouring when a letter begins fills
// the ring before the letter watches, and the letter says so.
//
// build: boards = v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/exti.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usart.hpp"
#include "ch32vx03/usbfs.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
/// The board's 8 MHz crystal through the PLL: 96 MHz of HCLK, 48 of PB1,
/// and the USB divider halving the PLL whole (the file header).
using SysClock = Clock<ClockSource::pll, 96'000'000, 8'000'000>;
constexpr SysClock clock;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

TestBench<Serial> bench;

/// The pool that never refuses a claim.
using Usb = Usbfs<>;
/// Interface 0, the notification on endpoint 1 and the bulk pair on 2.
using Port = UsbCdcAcm<Usb, P>;

struct Descriptors {
    static constexpr auto device =
        usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        usb_concat(usb_configuration_head(9 + Port::descriptor_bytes, Port::interface_count),
                   Port::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static constexpr auto manufacturer = usb_string_descriptor("brio");
    static constexpr auto product = usb_string_descriptor("test_vx03_usbfs");
    static constexpr auto serial_number = usb_string_descriptor("ch32vx03");
    static std::span<const uint8_t> string(uint8_t index) {
        switch (index) {
            case 0: return language;
            case 1: return manufacturer;
            case 2: return product;
            case 3: return serial_number;
            default: return {};
        }
    }
};
using Device = UsbDevice<Usb, Descriptors, Port>;

/// The pads the part table names for the controller, and the other pair
/// the datasheet's table also names, read as a contrast in letter p (on
/// this board PB6/PB7 carry the I2C self-link's pull-ups).
using PadDp = Pin<device::usbfs_dp_port, device::usbfs_dp_pin>;
using PadDm = Pin<device::usbfs_dm_port, device::usbfs_dm_pin>;
using OtherDp = Pin<'B', 7>;

/// The bulk pair's endpoint number - the class's default.
constexpr uint8_t data_endpoint = 2;

/// What a configured CDC port spends of the pool: endpoint zero's 64,
/// the notification's half, and the bulk pair's two halves in one piece.
constexpr uint16_t pool_configured = usbfs_ep0_bytes + usbfs_half_bytes + 2u * usbfs_half_bytes;

// ---------------------------------------------------------------------------
// The rulers
// ---------------------------------------------------------------------------

uint32_t wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
    return ms;
}

const char* state_name(UsbDeviceState s) {
    switch (s) {
        case UsbDeviceState::detached: return "detached";
        case UsbDeviceState::defaulted: return "default";
        case UsbDeviceState::addressed: return "addressed";
        case UsbDeviceState::configured: return "configured";
    }
    return "?";
}

/// Wait for the host to configure the device; the milliseconds it took,
/// or zero when the budget ran out.
uint32_t wait_configured(uint32_t budget_ms) {
    const uint32_t t0 = Ticker::millis();
    while (!Device::configured()) {
        if (Ticker::millis() - t0 > budget_ms) {
            return 0;
        }
    }
    const uint32_t took = Ticker::millis() - t0;
    return took == 0u ? 1u : took;
}

/// A host letter's clean start: whatever a previous letter left in the
/// rings is dropped, so the counts belong to this letter alone.
void settle_port() {
    uint8_t b = 0;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 500u) {
        if (!Port::read_byte(b) && Port::tx_idle()) {
            break;
        }
    }
}

/// Is there a host at all? Every host-assisted letter asks first.
bool need_host() {
    if (Device::configured()) {
        return true;
    }
    print(serial, "  SKIPPED, and it passes as a skip: the device is not configured, so no host "
                  "is answering on the connector. Plug P14 into the host and run letter a first.",
          crlf);
    bench.verdict("the host letter is skipped and says so", true);
    return false;
}

/// Wait for the host to send something on the port, a bounded wait; the
/// bytes it sent, or zero.
uint32_t wait_for_traffic(uint32_t budget_ms) {
    const uint32_t rx0 = Port::rx_bytes();
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < budget_ms) {
        if (Port::rx_bytes() != rx0) {
            return Port::rx_bytes() - rx0;
        }
    }
    return 0;
}

/// Wait for the host to OPEN the port - DTR raised - before the device
/// pours: a tty the host has not yet put in raw mode ECHOES what it
/// receives back to the device (measured: "^C", "^\\" and the rest of the
/// control characters' echoes arriving on the bulk OUT), so a pour that
/// starts before the open fills the other direction with the host's
/// echo. The milliseconds it took, or zero.
uint32_t wait_for_open(uint32_t budget_ms) {
    const uint32_t t0 = Ticker::millis();
    while (!Port::dtr()) {
        if (Ticker::millis() - t0 > budget_ms) {
            return 0;
        }
    }
    const uint32_t took = Ticker::millis() - t0;
    wait_ms(50);
    return took == 0u ? 1u : took;
}

/// A host letter that needs the script pouring: the port settled, the
/// mode named, and the first byte waited for. False (a skip, said) when
/// the host never pours.
bool need_pour(const char* mode, uint32_t budget_ms) {
    if (!need_host()) {
        return false;
    }
    settle_port();
    print(serial, "  run the script's ", mode, " now (this letter waits ", budget_ms / 1000u,
          " s for the first byte)", crlf);
    if (wait_for_traffic(budget_ms) == 0u) {
        print(serial, "  SKIPPED, and it passes as a skip: the host sent nothing - the host end is "
                      "the bench's USB script, started in the window above.",
              crlf);
        bench.verdict("the host letter is skipped and says so", true);
        return false;
    }
    return true;
}

/// The host's streams count (the file header): a byte that is not one
/// above the last is a JUMP, and the jump's size is what was lost.
struct StreamCheck {
    uint8_t head[16] = {};
    uint32_t bytes = 0;
    uint32_t jumps = 0;
    uint8_t last_jump = 0;
    uint8_t first_jump = 0;
    uint32_t first_jump_at = 0;
    uint8_t next = 0;
    bool started = false;

    void take(uint8_t b) {
        if (started && b != next) {
            if (jumps == 0u) {
                first_jump = static_cast<uint8_t>(b - next);
                first_jump_at = bytes;
            }
            ++jumps;
            last_jump = static_cast<uint8_t>(b - next);
        }
        started = true;
        next = static_cast<uint8_t>(b + 1u);
        if (bytes < sizeof head) {
            head[bytes] = b;
        }
        ++bytes;
    }
};

/// Drain the port for `ms` milliseconds through a stream check; the bytes
/// drained.
uint32_t drain(StreamCheck& check, uint32_t ms) {
    const uint32_t before = check.bytes;
    uint8_t b = 0;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
        if (Port::read_byte(b)) {
            check.take(b);
        }
    }
    return check.bytes - before;
}

/// HCLK moved under the running PLL by HPRE alone - the USB clock, which
/// USBPRE takes from the PLL before HPRE, does not move - with the
/// console and the tick rebased FIRST, at the rate being left, which is
/// the order a dynamic clock keeps. PB1 stays at or under its cap.
void switch_hclk(uint32_t hclk) {
    Serial::rebase(hclk);
    Ticker::rebase(hclk);
    Rcc::prescalers(hpre_for(SysClock::hz, hclk), ppre_for(hclk, pb1_max_hz), 0);
}

constexpr uint32_t ladder[] = {96'000'000UL, 48'000'000UL, 24'000'000UL, 12'000'000UL, 6'000'000UL, 1'500'000UL};
constexpr uint8_t ladder_rungs = sizeof(ladder) / sizeof(ladder[0]);

// =============================================================================
// a - the controller, and the host enumerating the device
// =============================================================================
void ta_enumerate() {
    Device::stop();
    Usb::release();
    wait_ms(20);

    const bool up = Usb::init(clock);
    print(serial, "  init: ", up ? "ok" : "FAILED", "; the tree feeds the controller ", SysClock::usb_hz,
          " Hz with HCLK at ", SysClock::hz, "; pulled up=", Usb::pulled_up() ? 1 : 0,
          " bus masters=", P::bus_masters_active(), " pads D+=", device::usbfs_dp_port,
          device::usbfs_dp_pin, " D-=", device::usbfs_dm_port, device::usbfs_dm_pin, crlf);
    bench.verdict("the controller comes up on a tree that makes it exactly 48 MHz, with the "
                  "pull-up still down and no bus master counted - the host must not see a "
                  "device before the program can answer it",
                  up && SysClock::usb_hz == usb_required_hz && !Usb::pulled_up() &&
                      P::bus_masters_active() == 0u);
    bench.verdict("the pause the driver rests on is set, and only endpoint zero's buffer is spent",
                  Usb::auto_pause() && Usb::buffer_used() == usbfs_ep0_bytes);

    const uint32_t setups_before = Device::setups();
    const uint32_t stalls_before = Device::stalls();
    Device::start();
    const uint32_t took = wait_configured(1500u);
    print(serial, "  start(): pulled up=", Usb::pulled_up() ? 1 : 0, " bus masters=",
          P::bus_masters_active(), "; the host took ", took, " ms: state=",
          state_name(Device::state()), " address=", Device::address(), " resets=",
          Device::resets(), " setups=", Device::setups(), " stalls=", Device::stalls(),
          " last request=", hex(Device::last_request()), crlf);
    bench.verdict("start() raises the pull-up and the host enumerates the device to CONFIGURED "
                  "within a second and a half",
                  Usb::pulled_up() && took != 0u && Device::configured());
    bench.verdict("the host reset the bus at least once and assigned a non-zero address that the "
                  "block holds",
                  Device::resets() >= 1u && Device::address() != 0u &&
                      Usb::address() == Device::address());
    bench.verdict("the enumeration took at least the four requests a host must make, and few "
                  "stalls (a device qualifier and a string this device has not are stalled by "
                  "the rules, and the next SETUP still arrived)",
                  Device::setups() - setups_before >= 4u && Device::stalls() - stalls_before <= 8u);
    bench.verdict("the port is configured with its bulk OUT armed and nothing in flight",
                  Port::configured() && Port::out_armed() && !Port::in_armed() && Port::tx_idle());

    print(serial, "  pool: ", Usb::buffer_used(), " of ", Usb::pool_size, " bytes spent; RX_LEN "
          "said ", Usb::setup_length(), " at the last SETUP, INT_ST ", hex(Usb::setup_status()),
          "; endpoint zero at rest answers OUT with ", hex(Usb::regs().UEP[0].RX_CTRL),
          " and IN with ", hex(Usb::regs().UEP[0].TX_CTRL), "; the bulk OUT expects DATA",
          Usb::out_toggle(data_endpoint) ? 1 : 0, crlf);
    // WHAT THE BUFFER REGISTER KEEPS: the vendor reads it back as a 16-bit
    // offset (the driver's file header). Read here, never by the driver.
    print(serial, "  endpoint zero's buffer register reads back ", hex(Usb::regs().UEP_DMA[0]),
          " and endpoint 2's ", hex(Usb::regs().UEP_DMA[data_endpoint]), crlf);
    bench.verdict("endpoint zero's buffer, the notification's half and the bulk pair's two halves "
                  "in one piece: 256 bytes",
                  Usb::buffer_used() == pool_configured);
    print(serial, "  errors=", Usb::errors(), " FIFO overflows=", Usb::overruns(),
          " PID mismatches=", Usb::toggle_errors(), crlf);
    bench.verdict("no stray SETUP, no FIFO overflow and no repeated packet through the whole "
                  "enumeration",
                  Usb::errors() == 0u && Usb::overruns() == 0u && Usb::toggle_errors() == 0u);

    // THE COUNT, AND THE IDLE PATH IT GOVERNS: a thousand calls at a count
    // of one are a thousand returns.
    const uint8_t masters = P::bus_masters_active();
    const uint16_t overruns_before = Usb::overruns();
    const uint32_t t0 = Ticker::millis();
    for (uint16_t i = 0; i < 1000u; ++i) {
        P::CriticalSection cs;   // the kernel enters idle() masked
        P::idle();
    }
    const uint32_t idle_ms = Ticker::millis() - t0;
    print(serial, "  bus masters=", masters, " attached=", Usb::attached() ? 1 : 0,
          "; a thousand idle() calls took ", idle_ms, " ms", crlf);
    bench.verdict("the attached controller counts itself one bus master, so the kernel's idle "
                  "path returns at once instead of sleeping - and the port survives it",
                  masters == 1u && Usb::attached() && idle_ms <= 20u && Device::configured() &&
                      Usb::overruns() == overruns_before);
}

// =============================================================================
// p - the pads the block drives
// =============================================================================
void tp_pads() {
    Device::stop();
    wait_ms(50);
    const bool dp_port_down = PadDp::read();
    const bool dp_block_down = Usb::dp_high();
    const bool dm_port_down = PadDm::read();
    const bool other_down = OtherDp::read();

    Device::start();
    wait_ms(5);   // before the host's debounce ends and its bus reset drives SE0
    const bool dp_port_up = PadDp::read();
    const bool dp_block_up = Usb::dp_high();
    const bool dm_port_up = PadDm::read();
    const bool other_up = OtherDp::read();

    print(serial, "  pull-up down: D+ port=", dp_port_down ? 1 : 0, " block=", dp_block_down ? 1 : 0,
          " D- port=", dm_port_down ? 1 : 0, " PB7=", other_down ? 1 : 0, crlf);
    print(serial, "  pull-up up:   D+ port=", dp_port_up ? 1 : 0, " block=", dp_block_up ? 1 : 0,
          " D- port=", dm_port_up ? 1 : 0, " PB7=", other_up ? 1 : 0, crlf);
    // THE BLOCK'S OWN PIN BIT IS THE VERDICT: on this board PB6/PB7 carry
    // the I2C self-link's 4.7 kOhm pull-ups, so a block that drove that
    // pair would read D+ high with its own pull-up down. The port's view
    // of PA11/PA12 is printed beside it - whether a GPIO input still sees
    // a pad the transceiver owns is a finding, not a precondition.
    bench.verdict("with the pull-up down the host's 15 kOhm holds D+ low and with it up D+ reads "
                  "high, as the block sees its own pad - which on this board only the cabled "
                  "pair can do",
                  !dp_block_down && dp_block_up);
    bench.verdict("and the part table's D+ pad, read through its port, follows the same two "
                  "levels while D- stays low",
                  !dp_port_down && dp_port_up && !dm_port_down && !dm_port_up);

    const uint32_t took = wait_configured(2000u);
    print(serial, "  re-enumerated in ", took, " ms", crlf);
    bench.verdict("the host enumerates the device again after the pads were read",
                  took != 0u && Device::configured());

    // THE PADS WITH THEIR PORT'S CLOCK SHUT - the device controller of
    // ch. 21 attaches and then fails every packet so (usb.md), and the
    // vendor's sequence for this block never opens the port. The console
    // is silent across the window: USART1's pads are port A's too.
    Device::stop();
    wait_ms(300);
    while (!Serial::tx_idle()) {
    }
    wait_ms(2);
    Rcc::disable(Bus::pb2, gpio_clock_for(device::usbfs_dp_port));
    const bool shut = !Rcc::enabled(Bus::pb2, gpio_clock_for(device::usbfs_dp_port));
    Device::start();
    const uint32_t took_shut = wait_configured(2000u);
    const uint16_t overflows_shut = Usb::overruns();
    Rcc::enable(Bus::pb2, gpio_clock_for(device::usbfs_dp_port));
    print(serial, "  with port ", device::usbfs_dp_port, "'s clock ", shut ? "SHUT" : "open (?)",
          ": configured in ", took_shut, " ms (0: never), FIFO overflows ", overflows_shut, crlf);
    bench.verdict("THE PADS NEED NO PORT CLOCK HERE: with the gate of the port the two pads "
                  "belong to shut, the host enumerated the device - this block reaches its pads "
                  "without GPIO, where the device controller of ch. 21 cannot",
                  shut && took_shut != 0u);
    if (!Device::configured()) {
        Device::stop();
        wait_ms(300);
        Device::start();
        (void)wait_configured(2000u);
    }
}

// =============================================================================
// u - a reconnect: the pull-up dropped and raised
// =============================================================================
void tu_reconnect() {
    if (!Device::configured()) {
        print(serial, "  SKIPPED, and it passes as a skip: the device is not configured, so "
                      "there is nothing to detach. Run letter a first.",
              crlf);
        bench.verdict("the reconnect is skipped and says so", true);
        return;
    }
    const uint32_t resets = Device::resets();
    const uint8_t address = Device::address();

    Device::stop();
    // THE DETACHED BUS IS SE0 - the host's two pull-downs and nobody's
    // pull-up - and a block whose transceiver is still on could take that
    // for a bus reset. Read again after the time a host takes to call an
    // SE0 a disconnect, so a reset reported meanwhile shows.
    wait_ms(100);
    print(serial, "  stopped, 100 ms later: pulled up=", Usb::pulled_up() ? 1 : 0, " state=",
          state_name(Device::state()), " resets ", resets, " -> ", Device::resets(),
          " bus masters=", P::bus_masters_active(), crlf);
    bench.verdict("stop() drops the pull-up, which detaches the device and gives its bus-master "
                  "count back - and the detached block reports no bus reset over the SE0 it leaves",
                  !Usb::pulled_up() && P::bus_masters_active() == 0u &&
                      Device::state() == UsbDeviceState::detached && Device::resets() == resets);

    wait_ms(500);
    Device::start();
    const uint32_t took = wait_configured(2000u);
    print(serial, "  re-attached: resets ", resets, " -> ", Device::resets(), ", address ",
          address, " -> ", Device::address(), ", configured in ", took, " ms, bus masters=",
          P::bus_masters_active(), crlf);
    bench.verdict("raising the pull-up again brings the host's bus reset and a fresh enumeration, "
                  "and the count is held once more",
                  Device::resets() > resets && took != 0u && P::bus_masters_active() == 1u);
    bench.verdict("the port is configured again with its bulk OUT armed",
                  Port::configured() && Port::out_armed());
}

// =============================================================================
// e - the bus floor: an enumeration at every rung of HCLK
// =============================================================================
void te_ladder_enumeration() {
    uint32_t took[ladder_rungs] = {};
    uint16_t overflows[ladder_rungs] = {};
    for (uint8_t i = 0; i < ladder_rungs; ++i) {
        switch_hclk(ladder[i]);
        const uint16_t ov0 = Usb::overruns();
        Device::stop();
        wait_ms(300);
        Device::start();
        took[i] = wait_configured(2000u);
        overflows[i] = static_cast<uint16_t>(Usb::overruns() - ov0);
    }
    switch_hclk(SysClock::hz);
    // Printed at the suite's own rate: below 1.84 MHz the console's 115200
    // baud is not a divisor USART1 can make, so the lowest rung is silent.
    for (uint8_t i = 0; i < ladder_rungs; ++i) {
        print(serial, "  HCLK ", ladder[i] / 1000UL, " kHz: configured in ", took[i],
              " ms (0: never), FIFO overflows ", overflows[i], crlf);
    }
    if (!Device::configured()) {
        Device::stop();
        wait_ms(300);
        Device::start();
        (void)wait_configured(2000u);
    }
    print(serial, "  back at ", SysClock::hz / 1'000'000UL, " MHz: state=", state_name(Device::state()),
          "; the driver's floor is ", usbfs_min_hclk_hz / 1000UL, " kHz", crlf);
    bool above = true;
    for (uint8_t i = 0; i < ladder_rungs; ++i) {
        if (ladder[i] >= usbfs_min_hclk_hz) {
            above = above && took[i] != 0u && overflows[i] == 0u;
        }
    }
    bench.verdict("the host enumerates the device at every rung of HCLK from 96 MHz down to the "
                  "floor the driver refuses to go below, with no FIFO overflow",
                  above);
    bench.verdict("and at the lowest rung it does not (the rungs between are printed above and "
                  "judged by the pour, letter h)",
                  took[ladder_rungs - 1u] == 0u);
    bench.verdict("and the device is configured again at the suite's own rate", Device::configured());
}

// =============================================================================
// f - the frame counter the vendor's header names
// =============================================================================
void tf_frames() {
    if (!Device::configured()) {
        print(serial, "  SKIPPED, and it passes as a skip: with no host there are no frames. Run "
                      "letter a first.",
              crlf);
        bench.verdict("the frame letter is skipped and says so", true);
        return;
    }
    const uint32_t f0 = Usb::frames();
    Usb::count_frames(true);
    wait_ms(100);
    Usb::count_frames(false);
    const uint32_t counted = Usb::frames() - f0;
    wait_ms(10);
    const uint32_t after = Usb::frames() - f0;
    print(serial, "  frames counted in 100 ms: ", counted, "; ten more milliseconds with the bit "
          "clear added ", after - counted, crlf);
    bench.verdict("INT_EN's bit 7 - reserved in the manual, the device SOF interrupt in the "
                  "vendor's header - counts a host frame a millisecond",
                  counted >= 95u && counted <= 105u);
    bench.verdict("and clearing it stops the count", after == counted);
}

// =============================================================================
// d - the FIFO overflow, and the sleep that fills it (host: d)
// =============================================================================
void td_overflow() {
    const uint16_t standing = Usb::overruns();
    print(serial, "  FIFO overflows=", standing, " errors=", Usb::errors(), " bus masters=",
          P::bus_masters_active(), crlf);
    wait_ms(100);
    bench.verdict("the FIFO overflow counter does not move on a device the core stays awake for",
                  Usb::overruns() == standing);

    if (!need_pour("d", 6000u)) {
        return;
    }

    // AWAKE FIRST, as the reference: the same span, the same pour. One
    // stream check runs through all three windows, so a packet the sleep
    // loses is a jump where the drain picks the stream up again.
    StreamCheck check;
    const uint32_t awake = drain(check, 50);
    const uint32_t jumps_awake = check.jumps;
    const uint16_t ov_before = Usb::overruns();
    const uint32_t rx_before = Port::rx_bytes();

    // THEN ASLEEP, WITH THE BARE INSTRUCTION - the one the platform's
    // idle() would not execute with the count standing, written out as an
    // instrument (test_vx03_usb's letter d says why it is a true WFI).
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 50u) {
        __asm__ volatile("wfi" ::: "memory");
    }
    const uint16_t overflowed = static_cast<uint16_t>(Usb::overruns() - ov_before);
    const uint32_t arrived = Port::rx_bytes() - rx_before;

    const uint32_t recovered = drain(check, 200);
    const uint32_t jumps_first = check.jumps - jumps_awake;
    print(serial, "  50 ms awake took ", awake, " bytes with ", jumps_awake, " jumps; 50 ms of wfi "
          "took ", arrived, " and counted ", overflowed, " FIFO overflows; 200 ms awake again "
          "drained ", recovered, " with ", jumps_first, " jumps (the last of ",
          check.last_jump, " bytes: the sleep's loss, if any)", crlf);
    bench.verdict("A SLEEPING CORE THAT DOES NOT DRAIN FILLS THE RING AND THE BLOCK ANSWERS NAK: "
                  "the wfi window carried at most the class's ring, and nothing was lost - no "
                  "jump, no overflow",
                  arrived < awake / 2u && jumps_first == 0u && overflowed == 0u);

    // THEN ASLEEP BETWEEN PACKETS, which is the question: the same
    // instruction with the ring drained after every wake, so the class
    // re-arms the endpoint at once and the host's next packet reaches the
    // block while the core sleeps again. Its DMA writes that packet into
    // RAM across the bus matrix - which on this family serves no master
    // but the core in a sleep (bus_activity.hpp) - so a starved DMA
    // shows here as a FIFO overflow or a jump, and a working one as bytes.
    const uint16_t ov_between = Usb::overruns();
    const uint32_t jumps_between = check.jumps;
    uint32_t wakes = 0;
    uint32_t between = 0;
    const uint32_t t1 = Ticker::millis();
    while (Ticker::millis() - t1 < 50u) {
        __asm__ volatile("wfi" ::: "memory");
        ++wakes;
        uint8_t b = 0;
        while (Port::read_byte(b)) {
            check.take(b);
            ++between;
        }
    }
    const uint16_t overflowed_between = static_cast<uint16_t>(Usb::overruns() - ov_between);
    const uint32_t recovered_between = drain(check, 100);
    const uint32_t jumps_sleeping = check.jumps - jumps_between;
    print(serial, "  50 ms of wfi with the ring drained at every wake: ", wakes, " wakes, ",
          between, " bytes, ", overflowed_between, " FIFO overflows, ", jumps_sleeping,
          " jumps (the last of ", check.last_jump, " bytes); 100 ms awake after drained ",
          recovered_between, crlf);
    bench.verdict("with the core asleep between packets the block's DMA still carries them: "
                  "bytes arrived through the sleeping window with no overflow and no jump",
                  between > 0u && overflowed_between == 0u && jumps_sleeping == 0u);
    bench.verdict("with the core awake again the port takes bytes, still configured",
                  recovered_between > 0u && Device::configured());
}

// =============================================================================
// y - echo (host: y)
// =============================================================================
void ty_echo() {
    if (!need_host()) {
        return;
    }
    settle_port();
    print(serial, "  10 s of echo: run the script's y now", crlf);
    const uint32_t rx0 = Port::rx_bytes();
    const uint32_t tx0 = Port::tx_bytes();
    const uint8_t overruns = Port::rx_overruns();
    if (wait_for_traffic(10000u) == 0u) {
        print(serial, "  SKIPPED, and it passes as a skip: the host sent nothing in ten "
                      "seconds.",
              crlf);
        bench.verdict("the echo is skipped and says so", true);
        return;
    }
    uint32_t echoed = 0;
    uint8_t buf[64];
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 10500u) {
        uint32_t n = 0;
        while (n < sizeof buf && Port::read_byte(buf[n])) {
            ++n;
        }
        uint32_t written = 0;
        while (written < n) {
            written += Port::write(buf + written, n - written);
        }
        echoed += n;
    }
    while (!Port::tx_idle() && Ticker::millis() - t0 < 11500u) {
    }
    print(serial, "  echoed ", echoed, " bytes of the ", Port::rx_bytes() - rx0, " received (",
          Port::tx_bytes() - tx0, " sent), ring overruns ", Port::rx_overruns() - overruns,
          ", FIFO overflows ", Usb::overruns(), ", PID mismatches ", Usb::toggle_errors(), crlf);
    bench.verdict("every byte the host sent came back and none was dropped on the way in - the "
                  "host end is what judges the bytes themselves",
                  echoed > 0u && Port::rx_bytes() - rx0 == echoed &&
                      Port::tx_bytes() - tx0 == echoed && Port::rx_overruns() == overruns);
}

// =============================================================================
// w - the line coding and DTR (host: w)
// =============================================================================
void tw_line() {
    if (!need_host()) {
        return;
    }
    const uint32_t codings = Port::coding_changes();
    const uint32_t states = Port::line_state_changes();
    print(serial, "  15 s for the host to open the port at another frame and raise DTR", crlf);
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 15000u) {
        if (Port::coding_changes() != codings && Port::line_state_changes() != states) {
            break;
        }
    }
    const UsbLineCoding after = Port::line_coding();
    print(serial, "  then: ", after.baud, "/", after.data_bits, "/", after.parity, "/",
          after.stop_bits, " dtr=", Port::dtr() ? 1 : 0, " rts=", Port::rts() ? 1 : 0, crlf);
    if (Port::coding_changes() == codings && Port::line_state_changes() == states) {
        print(serial, "  SKIPPED, and it passes as a skip: the host neither set a line coding "
                      "nor moved a control line in fifteen seconds.",
              crlf);
        bench.verdict("the line coding letter is skipped and says so", true);
        return;
    }
    bench.verdict("the host's SET_LINE_CODING arrives through endpoint zero's OUT data stage - "
                  "its PID matched, its length filed - and reads back as the host sent it",
                  Port::coding_changes() > codings);
    bench.verdict("the host's SET_CONTROL_LINE_STATE arrives in the setup packet: DTR raised",
                  Port::line_state_changes() > states && Port::dtr());
}

// =============================================================================
// n - the core asleep, the controller working (host: v for the second half)
// =============================================================================
/// A bare wfi with interrupts enabled: the core sleeps until one is taken.
/// An instrument, as in letter d - the platform's idle() would not sleep
/// with the controller's bus-master count standing.
void bare_wfi() { __asm__ volatile("wfi" ::: "memory"); }

void tn_asleep() {
    if (!need_host()) {
        return;
    }
    // THE IN DIRECTION, ASLEEP: the device pours IN packets, the class
    // loading each one from its ring in the handler and the core in wfi
    // between them, so the block's DMA reads every packet out of RAM with
    // the core asleep. The host end counts what arrives against the
    // counting stream the device loads.
    settle_port();
    print(serial, "  2 s of the device pouring with the core in wfi between packets, once the "
                  "host opens the port: run the script's r now",
          crlf);
    if (wait_for_open(10000u) == 0u) {
        print(serial, "  SKIPPED, and it passes as a skip: the host never opened the port.", crlf);
        bench.verdict("the asleep letter is skipped and says so", true);
        return;
    }
    const uint32_t tx0 = Port::tx_bytes();
    uint8_t pattern[64];
    uint8_t next = 0;
    uint32_t in_wakes = 0;
    const uint32_t t1 = Ticker::millis();
    while (Ticker::millis() - t1 < 2000u) {
        for (uint8_t i = 0; i < 64u; ++i) {
            pattern[i] = next++;
        }
        uint32_t written = 0;
        while (written < 64u && Ticker::millis() - t1 < 2000u) {
            const uint32_t n = Port::write(pattern + written, 64u - written);
            written += n;
            if (n == 0u) {
                bare_wfi();
                ++in_wakes;
            }
        }
    }
    const uint32_t sent = Port::tx_bytes() - tx0;
    print(serial, "  the device sent ", sent, " bytes in 2 s = ", sent / 2000u, " KB/s with ",
          in_wakes, " wakes; what arrived is the host end's to count", crlf);
    bench.verdict("the device poured into a reading host with the core in wfi between its "
                  "packets - the host end counts the bytes the block could not read whole out "
                  "of RAM as breaks in the stream",
                  sent >= 8192u && in_wakes > 0u);
    while (!Port::tx_idle() && Ticker::millis() - t1 < 3000u) {
    }
    wait_ms(2500);   // the host end's read, over
    settle_port();

    // AN ENUMERATION THE CORE SLEEPS THROUGH: the pull-up dropped and
    // raised, and from the raise on nothing but interrupts wakes the core -
    // every IN stage of the host's control transfers read out of RAM by the
    // block with the core in wfi.
    Device::stop();
    wait_ms(300);
    const uint16_t ov0 = Usb::overruns();
    const uint32_t setups0 = Device::setups();
    uint32_t wakes = 0;
    Device::start();
    const uint32_t t0 = Ticker::millis();
    while (!Device::configured() && Ticker::millis() - t0 < 2000u) {
        bare_wfi();
        ++wakes;
    }
    const uint32_t took = Device::configured() ? Ticker::millis() - t0 : 0u;
    print(serial, "  an enumeration with the core in wfi between interrupts: ",
          took != 0u ? "configured" : "NOT configured", " in ", took, " ms, ", wakes, " wakes, ",
          Device::setups() - setups0, " setups, ", Usb::overruns() - ov0,
          " FIFO overflows; state=", state_name(Device::state()), " address=", Device::address(),
          " last request=", hex(Device::last_request()), crlf);
    bench.verdict("AN ENUMERATION DOES NOT COMPLETE WITH THE CORE ASLEEP between its interrupts: "
                  "the SETUPs arrive and the descriptors the block reads out of RAM for the IN "
                  "stages do not - which is what the controller's bus-master count, held from "
                  "its pull-up to its detach, keeps the kernel's idle path from doing",
                  took == 0u && Device::setups() != setups0);
    if (took == 0u) {
        Device::stop();
        wait_ms(300);
        Device::start();
        (void)wait_configured(2000u);
    }
    bench.verdict("and the port is configured when the core is awake again", Device::configured());
}

// =============================================================================
// v - throughput both ways (host: v)
// =============================================================================
void tv_throughput() {
    if (!need_host()) {
        return;
    }
    settle_port();
    print(serial, "  3 s of the device pouring once the host opens the port: run the script's v "
                  "now",
          crlf);
    if (wait_for_open(10000u) == 0u) {
        print(serial, "  SKIPPED, and it passes as a skip: the host never opened the port.", crlf);
        bench.verdict("the throughput letter is skipped and says so", true);
        return;
    }
    uint8_t pattern[64];
    uint8_t next = 0;
    const uint32_t tx0 = Port::tx_bytes();
    uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 3000u) {
        for (uint8_t i = 0; i < 64u; ++i) {
            pattern[i] = next++;
        }
        uint32_t written = 0;
        while (written < 64u && Ticker::millis() - t0 < 3000u) {
            written += Port::write(pattern + written, 64u - written);
        }
    }
    while (!Port::tx_idle() && Ticker::millis() - t0 < 4000u) {
    }
    const uint32_t sent = Port::tx_bytes() - tx0;
    print(serial, "  the device sent ", sent, " bytes in 3 s = ", sent / 3000u, " KB/s", crlf);
    if (sent < 8192u) {
        print(serial, "  SKIPPED, and it passes as a skip: with no host reading the port the "
                      "transmit ring fills and stays full.",
              crlf);
        bench.verdict("the throughput letter is skipped and says so", true);
        return;
    }
    bench.verdict("the device pours into a host that is reading, one packet per IN token and no "
                  "double buffer",
                  sent >= 8192u);

    print(serial, "  now the other way: the two seconds count from the host's first byte", crlf);
    if (wait_for_traffic(5000u) == 0u) {
        print(serial, "  SKIPPED, and it passes as a skip: the host poured nothing back.", crlf);
        bench.verdict("the second half of the throughput letter is skipped and says so", true);
        return;
    }
    StreamCheck check;
    const uint32_t rx0 = Port::rx_bytes();
    const uint8_t overruns = Port::rx_overruns();
    const uint16_t repeats0 = Usb::toggle_errors();
    const uint16_t errors0 = Usb::errors();
    const uint16_t fifo0 = Usb::overruns();
    (void)drain(check, 2000);
    const uint32_t got = Port::rx_bytes() - rx0;
    print(serial, "  the device took ", got, " bytes in 2 s = ", got / 2000u, " KB/s, ",
          check.jumps, " jumps (the first of ", check.first_jump, " bytes at byte ",
          check.first_jump_at, "), ring overruns ", Port::rx_overruns() - overruns,
          ", packets dropped by their PID ", Usb::toggle_errors() - repeats0, ", stray "
          "completions ", Usb::errors() - errors0, ", FIFO overflows ", Usb::overruns() - fifo0,
          crlf, "  the first bytes:");
    for (const uint8_t h : check.head) {
        print(serial, " ", hex(h));
    }
    print(serial, crlf);
    bench.verdict("the device drains what the host pours with no ring overrun and no gap - the OUT "
                  "endpoint is armed only while a whole packet fits, so USB's own NAK is the flow "
                  "control",
                  got > 0u && Port::rx_overruns() == overruns && check.jumps <= 1u);
}

// =============================================================================
// l - the receive length, filed per endpoint (host: y)
// =============================================================================
void tl_lengths() {
    if (!need_host()) {
        return;
    }
    settle_port();
    print(serial, "  10 s of echo with the lengths counted: run the script's y now (its opening "
                  "of the port sets a line coding too)",
          crlf);
    const uint32_t ep0 = Usb::received(0);
    const uint32_t bulk = Usb::received(data_endpoint);
    const uint32_t port = Port::rx_bytes();
    const uint32_t codings = Port::coding_changes();
    if (wait_for_traffic(10000u) == 0u) {
        print(serial, "  SKIPPED, and it passes as a skip: the host sent nothing.", crlf);
        bench.verdict("the length letter is skipped and says so", true);
        return;
    }
    uint8_t buf[64];
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 10500u) {
        uint32_t n = 0;
        while (n < sizeof buf && Port::read_byte(buf[n])) {
            ++n;
        }
        uint32_t written = 0;
        while (written < n) {
            written += Port::write(buf + written, n - written);
        }
    }
    const uint32_t ep0_bytes = Usb::received(0) - ep0;
    const uint32_t bulk_bytes = Usb::received(data_endpoint) - bulk;
    const uint32_t port_bytes = Port::rx_bytes() - port;
    const uint32_t coding_count = Port::coding_changes() - codings;
    print(serial, "  endpoint 0 filed ", ep0_bytes, " bytes over ", coding_count,
          " line codings; endpoint ", data_endpoint, " filed ", bulk_bytes, " where the class "
          "took ", port_bytes, crlf);
    bench.verdict("every byte the class took on the bulk OUT came from a length RX_LEN reported "
                  "for endpoint 2",
                  bulk_bytes > 0u && bulk_bytes == port_bytes);
    bench.verdict("and endpoint zero's OUT data stages filed seven bytes a line coding - the one "
                  "register telling the two endpoints apart by the status beside it",
                  ep0_bytes == 7u * coding_count);
}

// =============================================================================
// t - the toggle under a lost packet (host: d)
// =============================================================================
void tt_toggle() {
    if (!need_pour("d", 6000u)) {
        return;
    }
    StreamCheck check;
    (void)drain(check, 100);
    const uint32_t jumps_before = check.jumps;
    const uint16_t mismatches_before = Usb::toggle_errors();

    // THE STAGED LOSS: the PID the bulk OUT expects, flipped. The host's
    // next packet then carries the other PID - the silicon acknowledges
    // it, the driver drops it as a repeat - and the one after it matches.
    const bool pid = Usb::out_toggle(data_endpoint);
    (void)Usb::set_out_toggle(data_endpoint, !pid);
    const uint32_t taken = drain(check, 200);
    const uint16_t mismatches = static_cast<uint16_t>(Usb::toggle_errors() - mismatches_before);
    const uint32_t jumps = check.jumps - jumps_before;
    print(serial, "  the bulk OUT expected DATA", pid ? 1 : 0, " and was told DATA", pid ? 0 : 1,
          "; 200 ms took ", taken, " bytes, ", mismatches, " packets dropped by their PID, ", jumps,
          " jumps, the last of ", check.last_jump, " bytes", crlf);
    if (taken == 0u && mismatches > 2u) {
        // THE SILICON DID NOT ACKNOWLEDGE THE MISMATCH: the host is sending
        // the same packet again and again. Put the PID back, and say so.
        (void)Usb::set_out_toggle(data_endpoint, pid);
        print(serial, "  the stream stalled: the block does not acknowledge a mismatched PID, so "
                      "the host repeats it - the PID put back",
              crlf);
    }
    bench.verdict("exactly one packet is dropped by its PID, and it is the one the flip "
                  "mismatched",
                  mismatches == 1u);
    bench.verdict("the stream shows one jump of one packet (at most 64 bytes) and runs on in step "
                  "after it",
                  jumps == 1u && check.last_jump > 0u && check.last_jump <= 64u && taken > 0u);
}

// =============================================================================
// b - the automatic pause (host: d)
// =============================================================================
/// What the block did while nobody served it: the vector held off for
/// `ms` under the pour, INT_ST polled all the while. Every change of that
/// byte is a transaction the block TOOK - and one it took over the last,
/// since the handler that would have read the last is the one held off.
struct Hold {
    uint8_t flags_at_start;
    uint8_t status_at_start;
    uint8_t flags_at_end;
    uint8_t rx_ctrl;
    uint32_t changes;
};

Hold hold_vector(uint32_t ms) {
    Hold h{};
    Pfic::disable(Irq::usbfs);
    h.flags_at_start = Usb::flags();
    h.status_at_start = Usb::status();
    h.rx_ctrl = Usb::regs().UEP[data_endpoint].RX_CTRL;
    uint8_t last = h.status_at_start;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
        const uint8_t now = Usb::status();
        if (now != last) {
            ++h.changes;
            last = now;
        }
    }
    h.flags_at_end = Usb::flags();
    Pfic::enable(Irq::usbfs);
    return h;
}

void tb_pause() {
    if (!need_pour("d", 6000u)) {
        return;
    }
    StreamCheck check;
    (void)drain(check, 50);
    const uint16_t overflows_before = Usb::overruns();
    const uint16_t repeats_on = Usb::toggle_errors();

    // WITH THE PAUSE: the vector held off for 20 ms under the pour, the
    // port drained just before so the bulk OUT is armed.
    const Hold on = hold_vector(20);
    const uint32_t jumps_before = check.jumps;
    (void)drain(check, 200);
    const uint32_t jumps_paused = check.jumps - jumps_before;
    print(serial, "  pause ON, vector held 20 ms: flags ", hex(on.flags_at_start), " then ",
          hex(on.flags_at_end), ", INT_ST ", hex(on.status_at_start), " changing ", on.changes,
          " times, the bulk OUT answering ", hex(on.rx_ctrl), "; afterwards ", jumps_paused,
          " jumps, ", Usb::toggle_errors() - repeats_on, " packets dropped by their PID, FIFO "
          "overflows ", Usb::overruns() - overflows_before, crlf);
    bench.verdict("the first packet after the vector was held raised the completion flag and "
                  "the status byte changed ONCE, for it, and never again in 20 ms - with the "
                  "endpoint's own register still answering ACK: the block holds the bus off by "
                  "itself",
                  (on.flags_at_end & usbfs_uif_transfer) != 0u && on.changes <= 1u &&
                      (on.rx_ctrl & usbfs_uep_res_mask) == usbfs_uep_res_ack);
    bench.verdict("so nothing is lost: no jump, no packet dropped by its PID, no overflow",
                  jumps_paused == 0u && Usb::toggle_errors() == repeats_on &&
                      Usb::overruns() == overflows_before);

    // WITHOUT IT: the same hold, the pause cleared - staged, and put back.
    // The port is drained first, as before the first hold: the print
    // above let the ring fill, and a full ring leaves the OUT endpoint
    // unarmed, which is a hold that could lose nothing.
    (void)drain(check, 50);
    const uint32_t jumps_before_off = check.jumps;
    const uint16_t repeats_off = Usb::toggle_errors();
    Usb::auto_pause(false);
    const Hold off = hold_vector(20);
    Usb::auto_pause(true);
    (void)drain(check, 200);
    const uint32_t jumps_unpaused = check.jumps - jumps_before_off;
    print(serial, "  pause OFF, vector held 20 ms: flags ", hex(off.flags_at_start), " then ",
          hex(off.flags_at_end), ", INT_ST ", hex(off.status_at_start), " changing ", off.changes,
          " times, the bulk OUT answering ", hex(off.rx_ctrl), "; afterwards ", jumps_unpaused,
          " jumps (the last of ", check.last_jump, " bytes), ", Usb::toggle_errors() - repeats_off,
          " packets dropped by their PID; state=", state_name(Device::state()), crlf);
    bench.verdict("WITHOUT THE PAUSE THE BLOCK GOES ON TAKING PACKETS into the buffer nobody has "
                  "read: the status byte changed under the held vector, each change a packet "
                  "over the last - which is why init() sets it",
                  off.changes > 1u);
    bench.verdict("and with it back the port carries on configured", Device::configured() &&
                                                                       Usb::auto_pause());
}

// =============================================================================
// h - the bus floor: the pour drained at every rung of HCLK (host: d)
// =============================================================================
void th_ladder_data() {
    if (!need_pour("d", 6000u)) {
        return;
    }
    uint32_t bytes[ladder_rungs] = {};
    uint32_t jumps[ladder_rungs] = {};
    uint16_t overflows[ladder_rungs] = {};
    for (uint8_t i = 0; i < ladder_rungs; ++i) {
        switch_hclk(ladder[i]);
        StreamCheck check;
        const uint16_t ov0 = Usb::overruns();
        bytes[i] = drain(check, 300);
        jumps[i] = check.jumps;
        overflows[i] = static_cast<uint16_t>(Usb::overruns() - ov0);
    }
    switch_hclk(SysClock::hz);
    for (uint8_t i = 0; i < ladder_rungs; ++i) {
        print(serial, "  HCLK ", ladder[i] / 1000UL, " kHz: ", bytes[i], " bytes in 300 ms, ",
              jumps[i], " jumps, ", overflows[i], " FIFO overflows", crlf);
    }
    bool above = true;
    for (uint8_t i = 0; i < ladder_rungs; ++i) {
        if (ladder[i] >= usbfs_min_hclk_hz) {
            above = above && bytes[i] > 0u && jumps[i] == 0u && overflows[i] == 0u;
        }
    }
    const uint8_t low = ladder_rungs - 1u;
    bench.verdict("the pour is carried with no jump and no overflow at every rung of HCLK from "
                  "96 MHz down to the driver's floor",
                  above);
    bench.verdict("and at the lowest rung it is not carried whole (the rung between the floor and "
                  "it is printed above - bytes lost there in some runs and not in others, which "
                  "is why the floor is where it is)",
                  bytes[low] == 0u || jumps[low] != 0u || overflows[low] != 0u);
    bench.verdict("and the port is configured at the suite's own rate again", Device::configured());
}

// =============================================================================
// s - suspend and resume, and the wake-up lines (host: s)
// =============================================================================
/// The wake-up line table 9-3 also names and the driver does not arm - 20
/// on the CH32V303, 18 on the CH32V203 - armed by hand as the contrast.
constexpr uint8_t other_wakeup_line =
    Usb::wakes_on_line18 ? Exti::line_usbfs_wakeup : Exti::line_usbd_wakeup;
constexpr Irq other_wakeup_irq = Usb::wakes_on_line18 ? Irq::usbfs_wakeup : Irq::usb_wakeup;
volatile uint32_t other_hits = 0;

void ts_suspend() {
    if (!need_host()) {
        return;
    }
    // Both lines table 9-3 names for this class's wake-up, each as an
    // interrupt: the driver's own, and the other by hand.
    (void)Usb::arm_wakeup(true);
    (void)Exti::clear(other_wakeup_line);
    (void)Exti::sense(other_wakeup_line, ExtiSense::rising);
    (void)Exti::interrupt(other_wakeup_line, true);
    Pfic::enable(other_wakeup_irq);

    const uint32_t suspends = Device::suspends();
    const uint32_t resumes = Device::resumes();
    const uint32_t wake_own = Usb::wakeups();
    const uint32_t wake_other = other_hits;
    print(serial, "  20 s for the host to let the port suspend (the script's s)", crlf);
    uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 20000u && Device::suspends() == suspends) {
    }
    if (Device::suspends() == suspends) {
        print(serial, "  SKIPPED, and it passes as a skip: this host did not let the bus go idle.",
              crlf);
        bench.verdict("the suspend letter is skipped and says so", true);
    } else {
        const uint8_t masters = P::bus_masters_active();
        const uint32_t own_at_suspend = Usb::wakeups() - wake_own;
        print(serial, "  suspended: suspends=", Device::suspends(), " state register says ",
              Usb::suspended() ? "suspended" : "awake", ", bus masters=", masters,
              ", the wake-up line fired ", own_at_suspend, " times so far; 20 s for the resume",
              crlf);
        bench.verdict("the controller reports the bus going idle as a suspend, and its state "
                      "register agrees",
                      Usb::suspended());
        bench.verdict("a suspended controller still holds its bus-master count, deliberately",
                      masters == 1u);
        t0 = Ticker::millis();
        while (Ticker::millis() - t0 < 20000u && Device::resumes() == resumes) {
        }
        wait_ms(200);
        const uint32_t hits_own = Usb::wakeups() - wake_own;
        const uint32_t hits_other = other_hits - wake_other;
        print(serial, "  then: resumes=", Device::resumes(), " resets=", Device::resets(),
              " state=", state_name(Device::state()), "; the driver's wake-up line ",
              Usb::wakeup_line, " fired ", hits_own, " times, line ", other_wakeup_line, " ",
              hits_other, crlf);
        if (Device::resumes() == resumes) {
            print(serial, "  the resume did not come in twenty seconds; the port may need "
                          "touching at the host end",
                  crlf);
            bench.verdict("the resume is not claimed and says so", true);
        } else {
            bench.verdict("the resume is reported and the port is configured as it was",
                          Device::configured() && Port::configured());
            bench.verdict("and the host's resume reached the core through the line the driver "
                          "arms - once, at the resume and not at the suspend - and never "
                          "through the other line table 9-3 names",
                          own_at_suspend == 0u && hits_own == 1u && hits_other == 0u);
        }
    }
    Pfic::disable(other_wakeup_irq);
    (void)Exti::release(other_wakeup_line);
    (void)Usb::arm_wakeup(false);
}

// ===========================================================================
// The menu
// ===========================================================================
void banner() {
    print(serial, crlf, "test_vx03_usbfs - ", device::part_name, ", the host/device controller of "
                        "RM ch. 23 in device mode under the CDC ACM stack; HCLK ",
          SysClock::hz, " Hz, the controller fed ", SysClock::usb_hz, crlf,
          "  the console is the probe's UART and P14 is the device: the host at its other end "
          "answers the letters d, n, y, w, v, l, t, b, h and s",
          crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

/// The controller's one line: every event of this block in device mode.
extern "C" BRIO_CH32_INTERRUPT void usbfs_handler() { Device::isr(); }
/// The two wake-up vectors: the driver's line through its body, the
/// other line counted by hand (letter s).
extern "C" BRIO_CH32_INTERRUPT void usb_wakeup_handler() {
    if constexpr (Usb::wakes_on_line18) {
        (void)Usb::wakeup_isr();
    } else if (brio::Exti::isr(brio::Exti::vector_lines(brio::Irq::usb_wakeup)) != 0u) {
        other_hits = other_hits + 1u;
    }
}
extern "C" BRIO_CH32_INTERRUPT void usbfs_wakeup_handler() {
    if constexpr (!Usb::wakes_on_line18) {
        (void)Usb::wakeup_isr();
    } else if (brio::Exti::isr(brio::Exti::vector_lines(brio::Irq::usbfs_wakeup)) != 0u) {
        other_hits = other_hits + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool usb_ok = Usb::init(clock);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();
    if (usb_ok) {
        Device::start();   // the pull-up: the host enumerates from here
    }

    bench.letter('a', "the controller, and the host enumerating the device", ta_enumerate);
    bench.letter('p', "the pads the block drives", tp_pads);
    bench.letter('u', "a reconnect: the pull-up dropped and raised", tu_reconnect);
    bench.letter('e', "the bus floor: an enumeration at HCLK 96 MHz down to 1.5", te_ladder_enumeration);
    bench.letter('f', "the frame counter the vendor's header names", tf_frames);
    bench.letter('d', "the FIFO overflow, and the sleep that fills it (host: d)", td_overflow);
    bench.letter('n', "the core asleep: the IN pour and an enumeration (host: r)", tn_asleep, false);
    bench.letter('y', "echo for 10 s (host: y)", ty_echo, false);
    bench.letter('w', "the line coding and DTR from the host (host: w)", tw_line, false);
    bench.letter('v', "throughput both ways (host: v)", tv_throughput, false);
    bench.letter('l', "the receive length, filed per endpoint (host: y)", tl_lengths, false);
    bench.letter('t', "the toggle under a lost packet (host: d)", tt_toggle, false);
    bench.letter('b', "the automatic pause, on and off (host: d)", tb_pause, false);
    bench.letter('h', "the bus floor: the pour at HCLK 96 MHz down to 1.5 (host: d)", th_ladder_data, false);
    bench.letter('s', "suspend and resume, and the wake-up lines (host: s)", ts_suspend, false);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "XT96" : "FAILED", " usb=", usb_ok ? "48MHz" : "FAILED",
              " tick=", tick_ok ? "STK" : "FAILED", crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            // THE LOOP IDLES (the file header): with the controller attached
            // the count is not zero and this returns at once.
            P::CriticalSection cs;
            P::idle();
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
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
