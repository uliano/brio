# USB (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.7 (the
controller: 12.7.1 what it is and the 4 kB of dual-port RAM that is its
whole interface, 12.7.2 CHANGES FROM RP2040 - 12.7.2.1 the errata fixed,
12.7.2.2 the new features, 12.7.2.2.3 the device ones and 12.7.2.2.4 the
error handling -, 12.7.3 the architecture with 12.7.3.7.1 the concurrent
access rule, 12.7.3.7.2 the memory map, 12.7.3.7.3 and 12.7.3.7.4 the
endpoint and buffer control words, 12.7.3.8 the device controller packet
by packet, 12.7.3.10 the VBUS signals, 12.7.5 the registers), 9.4 (the
function tables, and table 647's BANK 1, which is where the DP and DM
pads live), 8.6 (the PLLs), 7.5 (the subsystem resets), 3.2 (the
interrupt line), and Appendix E's **RP2350-E12**, the one item filed
against this block. docs/design/usb.md for the stack and the contracts.
The driver: `brio/rp2350/usb.hpp` (`Usb`, the endpoint controller) over
`clock.hpp` (`Clocks::usb_select`, `PllUsb`) and `resets.hpp`; the stack
`util/usb/device.hpp` and the class `util/usb/cdc.hpp` above it. The
reference suite: `test_rp2350_usb`, the host as the other end, from one
source on both of this chip's architectures.

## What the silicon does

One controller, one full-speed PHY on the chip's own two pins with its
pull-up, and 4 kB of dual-port RAM that is the whole interface between
the core and the controller: the eight-byte setup packet at offset 0, an
endpoint control word per endpoint and direction (the type, the buffer's
offset, single or double buffered, an interrupt per buffer), a buffer
control word per endpoint and direction (the length, the data PID, FULL,
AVAILABLE, STALL; a second half for the second buffer), endpoint zero's
64-byte buffer at 0x100 and 3840 bytes of buffer space from there - sixty
64-byte buffers, two of them endpoint zero's own pair, so fifty-eight are
left to hand out from 0x180. The registers at 0x50110000 hold the device
address, the muxing onto the PHY, the SIE's control and status (the bus
reset, the setup packet, suspend, resume, the error flags), the interrupt
quartet, and BUFF_STATUS - one bit per endpoint and direction - that says
which buffers completed. The register block answers the atomic aliases of
2.1.3 natively, and the CLEAR alias is the one way every status bit of
this block is cleared - the SIE's flags, BUFF_STATUS, EP_ABORT_DONE,
EP_STATUS_STALL_NAK, the error counters, the watchdog's fired bit - which
is the idiom 12.7.4.2's own listing uses and which leaves whatever else
lives in the register alone. THE DUAL-PORT RAM ANSWERS NO ALIAS
(12.7.3.7), so every write into it is a plain store.

A buffer is handed over by writing its control word in two steps, the
length and the PID first and AVAILABLE after a few cycles, because the
controller reads the word at 48 MHz while the core writes it at 150
(12.7.3.7.1 and the warning under table 1194).

**Everything of the RP2040's controller is here and works the same way**
- 12.7.2 says so in as many words - and what is added is the following.

**The one new duty: MAIN_CTRL.PHY_ISO.** It resets to 1, it isolates the
PHY from the switched core power domain, and while it stands the PHY does
nothing at all. Software must clear it at start-up and after a
power-down: it is the one line of RP2040 software that does not carry
over. The driver clears it LAST OF ALL - after the muxing, the power
overrides, the controller's own enable, the SIE and the interrupt mask -
which is 12.7.2.2.1's "remove isolation once software has configured the
controller", and which leaves the chapter's register order otherwise
exactly as it stands.

**Three reset values moved**, and all three are answered by writing the
register WHOLE rather than setting bits into it:

| Register | RP2040 reset | RP2350 reset | Why it matters to a device |
|---|---|---|---|
| `MAIN_CTRL` | 0x00000000 | 0x00000004 | PHY_ISO set: the PHY is isolated until told otherwise |
| `USB_MUXING` | 0x00000000 | 0x00000001 | TO_PHY already set, to match the PHY's isolation latches |
| `SIE_CTRL` | 0x00000000 | 0x00008000 | PULLDOWN_EN set - the HOST's termination, so the pins do not float while the block is unused; a device must drop it |

**DP and DM can be ordinary pads.** They are GPIO 56 and 57 of **bank
1** (table 647), whose function-select registers are the IO_QSPI block's
`USBPHY_DP_CTRL` and `USBPHY_DM_CTRL` and whose SIO face is the high
bank's bits 24 and 25; `USB_MUXING.USBPHY_AS_GPIO` is the other half of
the route. Neither is ever taken here: bank 1's FUNCSEL resets to NULL
and nothing in this stratum writes it (`pin.hpp` is bank 0 alone), and
`init()` stores `USB_MUXING` whole, so USBPHY_AS_GPIO, TO_EXTPHY,
TO_DIGITAL_PAD and SWAP_DPDM land clear whatever a previous image left.

**Erratum RP2350-E12 replaces the RP2040's E16, and is stricter.** Eight
SIE_STATUS flags (TRANS_COMPLETE, SETUP_REC, STALL_REC, NAK_REC,
RX_SHORT_PACKET, ACK_REQ, DATA_SEQ_ERROR, RX_OVERFLOW) and six INTR
flags cross from clk_usb to clk_sys without adequate synchronisation and
can be LOST when clk_sys is at or below clk_usb. The workaround is a
frequency rule: **clk_sys at least ten per cent above clk_usb** - 52.8
MHz, not 48 - while the peripheral is in use. The quasi-static bus states
(reset, suspend, resume) are exempt. It affects stepping A2, which is the
one on the bench.

**LINESTATE_TUNING is a register to leave alone.** 12.7.2 says so
plainly, and its reset value 0x0f8 already has every device-side fix on:
wake from suspend on any bus activity, the receive-error quiesce, the SE0
chatter fix, the bit-stuff fix, and the buffer-control DOUBLE READ that
makes the two-step AVAILABLE write unnecessary. The driver READS it and
has no verb that writes it - a negative of the family check asserts the
absence rather than a comment promising it. The two-step write is kept
all the same, because 12.7.3.7.1 still prescribes it.

**The diagnostics the RP2040 had not** (12.7.2.2.4), none of them on the
path a packet takes: a two-bit transmit error count per endpoint and a
receive transaction/sequence pair per endpoint (EP_TX_ERROR,
EP_RX_ERROR) that replicate the host's own error count, so a program can
tell that the host has probably given the endpoint up; a short packet
flagged in its own right (SIE_STATUS.RX_SHORT_PACKET) and, for endpoint
zero, SIE_CTRL.EP0_STOP_ON_SHORT_PACKET; two free-running 21-bit counters
of the 48 MHz domain, one raw and one latched at the last start of frame;
SM_STATE, which exposes the main, bus-control and receive-deserialiser
state machines; and DEV_SM_WATCHDOG, a watchdog on an 18-bit limit of
clk_usb cycles that raises a flag when the device state machine stays out
of idle too long and, if its own RESET bit is set, forces the machine
back to idle as well.

**Every RP2040 USB erratum is fixed** (12.7.2.1): E2 (the device endpoint
abort that would not clear) and E5 among them, so the abort path is
usable here, and E3, E4 and E15, which were the host side's and the
device receive state machine's. Nothing in this driver is a workaround.

The host half of the block (12.7.3.9, with SOF_WR, INT_EP_CTRL,
NAK_POLL, the fifteen polled interrupt endpoints and the new
stop-on-NAK feature of 12.7.2.2.2) is not reachable from this driver, by
design: brio has no host side.

## Types and verbs

- `usb_clk_sys_ok(hz)`: erratum RP2350-E12's rule as a predicate.
  `Usb::init` asserts it at compile time against the clock it is handed.
- `Usb`, a monostate: `init(clock)` (the USB PLL at 48 MHz if it is not
  there already, clk_usb from it, the block out of reset, its RAM
  cleared, then the chapter's own register order - USB_MUXING, USB_PWR
  with VBUS detection forced because the board wires no VBUS pin,
  MAIN_CTRL enabling the controller in device mode, SIE_CTRL, the
  interrupt mask - each of the three written WHOLE so that the moved
  reset values cannot survive, and THE PHY'S ISOLATION LIFTED LAST OF
  ALL, which is what 12.7.2.2.1 asks for; the pull-up stays down),
  `release` (the line dropped, the PHY isolated again, the block back
  into reset - for a block `init()` has taken out of reset, its first
  acts being register writes).
- The `UsbController` contract: `connect(on)` (the pull-up),
  `set_address`, `configure_endpoint(address, type, max)` (64-byte
  buffers handed out in claim order; a bulk OUT endpoint gets two
  halves), `deconfigure_endpoints`, `stall` / `stalled` (endpoint zero's
  armed in EP_STALL_ARM too), `submit_in(number, data)` (the packet
  copied into the RAM, the PID kept per endpoint and direction, endpoint
  zero's restarted at DATA1 by a setup packet), `submit_out(number,
  max)` (the next half in turn on a double-buffered endpoint, its own
  16-bit control word), `out_data(number)`, `setup()`, `take_events()`
  (the SIE flags and BUFF_STATUS read and acknowledged, the completed
  half and its length read first); `out_slots` (two: what a class arms
  at once).
- The readbacks: `connected`, `suspended`, `vbus_detected`, `pulled_up`,
  `address`, `frame` (the last start-of-frame number), `sie_status`,
  `line_state`, `take_errors` (CRC, bit stuff, overflow, timeout, data
  sequence), `buffers_used`, `buff_status`, `buff_cpu_should_handle`.
- The PHY's isolation and the muxing: `phy_isolated`, `phy_isolate(on)`,
  `controller_enabled`, `phy_as_gpio`, `muxing`.
- LINESTATE_TUNING, read and never written: `linestate_tuning`,
  `buffer_control_double_read_fix`, `wake_on_any_bus_activity`.
- The diagnostics: `endpoint_errors(number)` (a `UsbEndpointErrors`
  record: the transmit count and the two receive flags),
  `clear_endpoint_errors`, `take_endpoint_error`, `take_short_packet`,
  `ep0_stop_on_short_packet(on)`, `sof_timestamp_raw`,
  `sof_timestamp_last`, `since_sof` (their difference, the 21-bit wrap
  taken into account), `sm_state` with `sm_main` / `sm_bus_control` /
  `sm_rx_deserialiser`, and the device state-machine watchdog:
  `arm_device_watchdog(UsbDeviceWatchdogConfig)` (a limit past eighteen
  bits is REFUSED, not truncated), `disarm_device_watchdog`,
  `device_watchdog_armed`, `device_watchdog_limit`,
  `take_device_watchdog_fired`.
- The abort path: `abort(address, on)`, `aborted`, `abort_done`.
- The stall and NAK reporting: `stall_nak_status`, `clear_stall_nak`,
  `ep0_report(on_stall, on_nak)` - endpoint zero's two bits live in
  SIE_CTRL, because endpoint zero has no endpoint control word.
- `remote_wakeup()`: SIE_CTRL.RESUME, the device asking a suspended host
  to wake it.
- The interrupt surface: `interrupt_enable` both ways,
  `interrupt_raw`, `interrupt_status`, `force_interrupt`,
  `forced_interrupts`, `irq()`. The app binds `isr_usbctrl`, which is
  the same name on both architectures.
- The stack and the class are docs/design/usb.md's: `UsbDevice<Usb,
  Descriptors, Classes...>` with `start` / `stop` / `isr` and its
  counters, `UsbCdcAcm<Usb, P, interface, ep_notify, ep_data, rx, tx>`
  as a byte transport.

## How to use it

```cpp
using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;   // E12 wants > 52.8 MHz
using Port = brio::UsbCdcAcm<brio::Usb, P>;                            // interface 0, endpoints 1 and 2
struct Descriptors {
    static constexpr auto device = brio::usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration = brio::usb_concat(
        brio::usb_configuration_head(9 + Port::descriptor_bytes, Port::interface_count), Port::descriptors);
    static std::span<const uint8_t> string(uint8_t index);   // 0 the languages, then the strings
};
using Device = brio::UsbDevice<brio::Usb, Descriptors, Port>;

brio::Usb::init(clock);        // clears MAIN_CTRL.PHY_ISO last of all
Device::start();               // the pull-up: the host enumerates from here
extern "C" void isr_usbctrl() {
    Device::isr();
    if (Port::take_rx_edge()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
brio::print(Port{}, "hello", brio::crlf);   // once the host has configured the port
```

The port is a `ByteTransport`: a console's `SerialPort` runs over it as
over a UART, and the source is the same on both architectures. The
identity 1209:0001 is pid.codes' test pair, meant for a device that is
not a product; a product states its own.

After a power-down of the switched core domain the PHY comes back
isolated, so the way back is the whole of `init()` again - not a poke at
one bit:

```cpp
brio::Usb::phy_isolate(true);     // what a power-down leaves behind
Device::stop();
brio::Usb::init(clock);           // the isolation lifted last, once everything is configured again
Device::start();
```

The diagnostics are a report and not a path. A console command that says
where the bus is:

```cpp
brio::print(s, "phy_isolated=", brio::Usb::phy_isolated(),
               " since_sof=", brio::Usb::since_sof(),          // clk_usb cycles; < 48 000 on a live bus
               " ep2_tx_errors=", brio::Usb::endpoint_errors(2).tx_count,
               " sm=", brio::hex(brio::Usb::sm_state()), brio::crlf);
```

## Not covered yet

Driver gaps, each with its reason:

- **Host mode** (12.7.3.9, and the stop-on-NAK feature 12.7.2.2.2 added
  to it): brio has no host side, by decision (docs/design/usb.md). The
  registers that are host-only - SOF_WR, INT_EP_CTRL, NAK_POLL and the
  fifteen ADDR_ENDP of the polled interrupt endpoints - have no verb
  here, and a negative of the family check asserts that MAIN_CTRL's mode
  bit has none either.
- **Writing LINESTATE_TUNING**: declined. 12.7.2 says to leave it at its
  reset value, every device fix in it is already on, and a driver that
  offered the verb would be offering a way to turn a fix off. The
  absence is asserted by the family check.
- **USBPHY_DIRECT, USBPHY_DIRECT_OVERRIDE and USBPHY_TRIM**: the PHY
  driven by hand, pin by pin, and the pull-down trim. Nothing here needs
  the bus bit-banged, and the overrides are a way to break a working
  PHY; born with the first program that has to drive the lines with the
  controller off.
- **Isochronous endpoints and a double-buffered IN**: no class here
  streams, and the buffer control's isochronous offset field
  (table 1194, bits 28:27) goes with them. A second half on the IN would
  be the first thing a streaming class asks for.
- **VBUS detection and overcurrent from a pad** (12.7.3.10, and 9.4's
  VBUS functions on bank 0): the board wires neither, so detection is
  forced in USB_PWR; `SIE_STATUS.VBUS_OVER_CURR` has no verb.
- **`buff_cpu_should_handle` as the way to tell the halves apart**: a
  readback and not the path. The completed half of a double-buffered OUT
  endpoint is told from the strict alternation of the halves and each
  half's own FULL bit, which is what the other Raspberry Pi controller
  was measured to do; one mechanism proven on silicon is worth more than
  two, and the register is exposed so a letter can hold them against
  each other.
- **Suspend as a power state**: the suspend and resume events are
  counted and `remote_wakeup()` exists, but the current a suspended
  device draws, and the sleep site that would let the chip sleep with
  the controller armed, belong to the power chapter.
Implemented but not bench-verified, each with the letter of
`test_rp2350_usb` that will measure it:

- The controller coming up: clk_usb counted at 48 MHz off the USB PLL,
  clk_sys counted above E12's margin, VBUS forced and seen, the pull-up
  down until the stack raises it, MAIN_CTRL.PHY_ISO lifted by `init()`,
  USB_MUXING reading exactly TO_PHY plus SOFTCON with USBPHY_AS_GPIO
  clear, and SIE_CTRL with the host's pull-downs dropped (letter a).
- A real host enumerating the device: the bus reset, the descriptors,
  the address, the configuration reached inside a second and a half, the
  four buffers the CDC port claims, the start-of-frame counter advancing
  a frame a millisecond, and no SIE error (letter a).
- The two free-running 48 MHz timestamps and `since_sof` under a
  millisecond's worth of cycles on a live bus, SM_STATE inside its three
  fields, the per-endpoint error counters clear after a clean
  enumeration and clear again after the write-to-clear pair,
  LINESTATE_TUNING standing at 0x0f8, the device state-machine watchdog
  taking its 18-bit limit and reading it back (with a longer limit
  refused), and an idle endpoint's abort completing and LIFTING - which
  is the RP2040's erratum E2 fixed (letter b).
- **Whether the device state-machine watchdog's counter is held reset
  while the machine is idle.** The register description says only that
  the counter is reset on every state transition, which leaves open
  whether a quiet bus makes it fire. Letter b arms it WITHOUT the forced
  reset - so that a spurious fire cannot disturb the bus the suite is
  talking over - and prints whether it fired at the widest limit and at
  a millisecond's worth of cycles; those two numbers are the answer, and
  no verdict assumes one.
- A reconnect through the pull-up: dropped for half a second, raised,
  the host's bus reset counted and the device enumerated afresh
  (letter c).
- A reconnect through the PHY's isolation, which is this chip's own way
  and the shape a power-down leaves behind: PHY_ISO set for half a
  second, then the whole bring-up again (letter d).
- The port as a byte transport against a Linux host: the echo byte-exact
  over ten seconds, the line coding and DTR arriving through the control
  data stage, and the throughput both ways - the RP2040's numbers with
  the same class over a double-buffered bulk OUT are the thing to
  compare against (letters y, w and v, host-assisted and outside the
  all-key).
- The kernel console over the chip's own connector, on both
  architectures, with the serial string naming the half that is running.
- `take_errors` reporting a real error: a cable pulled mid-packet, which
  12.7.3.5 says raises them. No letter provokes it.
- `ep0_stop_on_short_packet`, `take_short_packet`, `stall_nak_status` /
  `ep0_report` and `remote_wakeup`: written, and no letter drives them -
  the stack's control machine ends its own data stage and reads the
  length rather than the flag, nothing here wants an interrupt per NAK,
  and a remote wakeup needs a host that has armed it.
- Two ports in one device (a second `UsbCdcAcm` on interfaces 2 and 3):
  the descriptors glue and the controller has buffers to spare, and
  nothing has asked for it.
