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

**AND IT IS NOT A DISCONNECT.** Isolating the PHY cuts it from the
switched core domain; it does not take the pull-up off the wire, and the
host is told nothing. Measured: with PHY_ISO set for half a second and
the whole bring-up run afterwards, **no bus reset arrives in six
seconds** - the host goes on addressing the address it assigned, while
the device's own stack has been reset to DEFAULT and answers at none.
A program coming back from a power-down must therefore FORCE the
reconnect: drop the pull-up long enough for the host to run its
disconnect debounce, then raise it. `Device::stop()`, half a second,
`Device::start()` is that, and it is the only way back.

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

## Bench findings

On an RP2350 in the QFN-80 package, **stepping A2**, clk_sys at 150 MHz
and clk_usb at 48 MHz off the USB PLL, the board's own USB-C into a Linux
host, on BOTH architectures: `test_rp2350_usb` reports **29 pass, 0 fail**
on the Cortex-M33 pair and on the Hazard3 pair, from one source, each on
three flash-and-run cycles, with the three host-assisted letters run
beside them. Every verdict reads the same on the two halves.

- **THE CONTROLLER COMES UP IN ABOUT 130 MICROSECONDS**: 133 to 135 us on
  the Cortex-M33 half and 108 to 110 us on the Hazard3 one, with clk_usb
  counted at **48 000 000 Hz** (48 000 062 once) and clk_sys at
  150 000 000 - well over erratum RP2350-E12's edge, which is 52.8 MHz.
  After `init()` MAIN_CTRL reads **0x1** (the isolation lifted, the
  controller enabled in device mode), USB_MUXING **0x9** (TO_PHY and
  SOFTCON, USBPHY_AS_GPIO clear) and SIE_CTRL **0x20000000** - the host's
  pull-downs, which reset SET on this chip, gone.
- **A LINUX HOST ENUMERATES THE DEVICE IN HALF A SECOND TO THREE
  QUARTERS**: 508 to 739 ms from `start()` to CONFIGURED, in **2 bus
  resets and 13 setup packets**, with **3 stalls** - the device qualifier
  and the strings a full-speed device has not, refused by the rules. The
  descriptors are 18 bytes for the device and 67 for the configuration
  (two interfaces, three endpoints) with four strings; the class's four
  buffers are handed out past endpoint zero's; the start-of-frame counter
  advances exactly **10 frames in 10 ms**; and the SIE error word is
  **0x0** through the whole of it.
- **THE TWO FREE-RUNNING 48 MHz TIMESTAMPS.** The raw counter advances
  **14 429 to 14 477 cycles over 300 us** - 48 MHz within half a per cent
  - and `since_sof` reads **18 024 to 40 002 cycles** on a live bus,
  always under the 48 000 that would be a millisecond. SM_STATE reads
  **0x100** and nothing above its three fields: main 0, bus control 0,
  receive deserialiser 1.
- **THE PER-ENDPOINT ERROR COUNTERS ARE CLEAR** after a clean
  enumeration, on endpoint zero and on the bulk pair alike, and
  ENDPOINT_ERROR reads 0; the write-to-clear pair reads back clear.
  LINESTATE_TUNING stands at its reset value **0xF8** - the
  buffer-control double read and wake-on-any-bus-activity fixes both on -
  and no verb here writes it.
- **THE DEVICE STATE-MACHINE WATCHDOG DOES NOT FIRE ON A QUIET MACHINE,
  which the register description leaves open.** Armed at its widest
  18-bit limit it reads back **262 143** and stands; over 20 ms it fires
  **0 times**, and **0 times** again at a millisecond's worth of cycles
  (48 000). So the counter is held while the machine is idle, and a limit
  past eighteen bits is refused instead of truncated.
- **THE ENDPOINT ABORT WORKS, WHICH IS THE RP2040'S ERRATUM E2 FIXED.**
  An idle endpoint's abort completes after **0 or 1 status reads** and
  LIFTS - on the other chip it stood for ever - and the port is still
  configured with its bulk OUT armed afterwards.
- **A RECONNECT THROUGH THE PULL-UP** takes **528 to 553 ms** from the
  raise to CONFIGURED, with the host's bus reset counted (2 -> 5: three
  more) and a fresh address.
- **A RECONNECT THROUGH THE PHY'S ISOLATION DOES NOT HAPPEN.** With
  PHY_ISO set for half a second and the whole bring-up run afterwards,
  the pull-up bit still reading 1 while the isolation stood: **no bus
  reset in six seconds**, resets 5 -> 5, the device left in DEFAULT while
  the host goes on addressing what it assigned. The way back is the
  device's own pull-up, dropped for half a second: **652 to 681 ms** to
  CONFIGURED, three more bus resets, a new address. The isolation raises
  **no SIE error** either way.
- **THE PORT AS A BYTE TRANSPORT, AGAINST A LINUX HOST.** Echo, in chunks
  of 1 to 300 bytes for six seconds: **2 463 024 bytes byte-exact on the
  Cortex-M33 half and 2 470 356 on the Hazard3 one**, no overrun, no SIE
  error, no endpoint error - **410.5 and 411.7 KB/s round trip**.
  Throughput: **675 KB/s device to host on the Cortex-M33 half and
  673 KB/s on the Hazard3 one** (2.03 and 2.02 MB in three seconds), and
  **906 and 905 KB/s host to device** (1.81 MB in two seconds) with no
  overrun. The other Raspberry Pi controller with the same class
  measured 559 KB/s out and 907 KB/s in: **the double-buffered bulk OUT
  is the same, and the single-buffered IN is some twenty per cent
  faster here.**
- **THE CLASS REQUESTS ARRIVE THROUGH THE CONTROL DATA STAGE.** A host
  opening the port at 19200 7E2 with DTR and RTS up is read back as
  **19200/7/2/2 dtr=1 rts=1**, the coding-change and line-state counters
  each advancing by one.
- **THE KERNEL CONSOLE RUNS ON THE CHIP'S OWN CONNECTOR, on both
  architectures**, and the serial string names the half: a host that has
  seen both images tells them apart by the device's name alone. Its own
  USB report on a live port reads state 3, an address the host assigned,
  2 bus resets, 16 setups, 3 stalls, the isolation lifted and the last
  start of frame a few thousand 48 MHz cycles old.

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
Implemented but not bench-verified, each with what would measure it:

- **`take_errors` reporting a real error**: a cable pulled mid-packet,
  which 12.7.3.5 says raises them. Every letter above leaves the SIE
  error word at zero, which is the right answer on a clean bus and no
  proof that the wrong one would be reported. It wants a hand on the
  cable.
- **`ep0_stop_on_short_packet`, `take_short_packet`, `stall_nak_status` /
  `ep0_report` and `remote_wakeup`**: written, and no letter drives them
  - the stack's control machine ends its own data stage and reads the
  length rather than the flag, nothing here wants an interrupt per NAK,
  and a remote wakeup needs a host that has armed it, which this one
  does not.
- **Two ports in one device** (a second `UsbCdcAcm` on interfaces 2 and
  3): the descriptors glue and the controller has buffers to spare, and
  nothing has asked for it.
- **The suspend and resume counters against a host that really
  suspends.** They advance here - the console's own report shows one
  suspend on a live port - but nothing arms an autosuspend on purpose or
  measures what the device draws in it; that pair belongs to the power
  chapter.
