# USB OTG, device mode (STM32F4)

Documents of record: **RM0383** ch. 22 (the STM32F411's OTG_FS: 22.3 the
core and its PHY, 22.5 the peripheral's states and endpoints, 22.8 the
low-power modes, 22.10..22.14 the FIFOs and their RAM, 22.15 the
interrupt hierarchy, 22.16 the registers, 22.17 the programming model),
**RM0390** ch. 31 and **RM0090** ch. 34 and 35 for the siblings and for
the high-speed core's own full-speed PHY; **ES0287** 2.12, **ES0298**
2.16 and **ES0206** 2.14 and 2.15 (the six OTG errata, two of them live
in device mode); the datasheets' alternate-function tables for the pads
(DS10314 table 9, DS10693 table 11, the F429's table 12);
[../design/usb.md](../design/usb.md) for the stack and the contracts.
The driver: [`brio/stm32f4/usb.hpp`](../../brio/stm32f4/usb.hpp)
(`UsbOtg<core>`, with `UsbFs` and `UsbHs` its two spellings) over
`clock.hpp`, `pin.hpp` and the reserve; the stack `util/usb/device.hpp`
and the class `util/usb/cdc.hpp` above it. The reference suite:
`test_stm32f4_usb`, the host as the other end; `console_usb` is the
kernel console over the same port.

## What the silicon does

A Synopsys DWC2 core with ST's glue, up to two of them: OTG_FS, whose
full-speed PHY is on chip and bonded to PA11 and PA12 at AF10, and
OTG_HS, which drives a ULPI transceiver at high speed and ALSO has a
full-speed PHY of its own on PB14 and PB15 at AF12. Seen from device
mode at full speed the two are one programmer's model at two base
addresses, which is why the driver is one template over the core and
not two drivers; the F410 has neither, the F401/F411/F412/F413 have the
full-speed core alone, the rest have both.

THE INTERFACE IS FIFOS AND NOTHING ELSE. Where the RP2040's controller
offers a dual-port RAM the program addresses, this core offers a block
of dedicated RAM reached only through PUSH AND POP REGISTERS, one 32-bit
window per FIFO, and the packet's LENGTH travels beside the data in a
transfer-size register:

- ONE SHARED RECEIVE FIFO for every OUT endpoint. Each packet is stacked
  behind the last with a STATUS ENTRY on top of it, and the application
  pops the status (GRXSTSP says which endpoint, how many bytes and what
  kind of entry) and then the bytes. A SETUP packet arrives as two
  entries - the eight bytes, then a "setup stage done" - and the
  endpoint's own SETUP interrupt follows the second.
- ONE TRANSMIT FIFO PER IN ENDPOINT, each a slice of the same RAM the
  application maps by hand, as a start address and a depth in 32-bit
  words. Nothing checks the map: two FIFOs at one address is a
  configuration the silicon accepts in silence.
- A TRANSFER, not a packet, is what an endpoint register describes: a
  packet count and a byte count the core walks down, with one interrupt
  at the end of the whole thing.

The RAM is 1.25 Kbyte - 320 words - on the full-speed core and 4 Kbyte
on the high-speed one. Endpoint zero's transfer-size register is the
narrow one: one packet-count bit and a seven-bit byte count, so a
control transfer is one packet at a time whatever the others do.

The core needs 48 MHz on its own domain (the main PLL's Q output on this
family) and an AHB above 14.2 MHz, whose rate chooses the turnaround
GUSBCFG.TRDT adds so a slow bus still answers an IN token in time
(RM0383 table 132). Device mode is forced in GUSBCFG, which frees the ID
pad; the VBUS pad is freed by GCCFG, whose bits are the one thing about
this peripheral that was respun twice inside the family - NOVBUSSENS on
the older parts, VBDEN with a session-valid override in GOTGCTL on the
newer ones, and neither where there is no core.

Two errata are live in device mode, and both are answered in code: a
transmit-FIFO write sequence INTERRUPTED by an access to an endpoint
register corrupts the next data written (ES0287 2.12.1), and DCFG's
address field does not read back correctly just after it is written
(2.12.6). The other four are host mode's. Separately, ES0206 2.2.14
asks for PA12 to be left unconnected on the impacted F42x/F43x parts and
names the high-speed core in full-speed mode as the way out - which is
the reason `UsbHs` exists here.

## Types and verbs

- `OtgCore` (`fs`, `hs`) and the reserve's `otg_facts(core)`: presence,
  base, the bus its gate sits on, the FIFO words, the endpoint numbers,
  the pads and their AF, and whether GUSBCFG's PHY select is writable.
  Beside it `otg_present`, `otg_irq`, `otg_wakeup_irq`,
  `otg_wakeup_exti_line`, `otg_vbus_style` / `otg_gccfg_device` (the
  three GCCFG generations reduced to the value a device wants),
  `otg_has_session_override` / `otg_session_override_bits` /
  `otg_session_valid_bit`, and `otg_turnaround_for` (table 132).
- `UsbOtg<core>`, with `UsbFs` and `UsbHs` where the part has them:
  `init(clock, sense_vbus = false)` (the gate, the pads, the core soft
  reset, the transceiver, device mode forced with the chapter's 25 ms
  settle, the FIFO map, the interrupts at the block and at the NVIC; the
  pull-up left down), `release`. The `UsbController` contract:
  `connect(on)` (the soft disconnect), `set_address`,
  `configure_endpoint(address, type, max)` (a slice of the FIFO RAM per
  IN endpoint, REFUSED when the budget is spent), `deconfigure_endpoints`,
  `stall` / `stalled`, `submit_in(number, data)`, `submit_out(number,
  max)`, `out_data(number)`, `setup()`, `take_events()` - the ISR body -
  and `out_slots` (eight, what a class may hold armed).
- The budget as constants: `fifo_words`, `rx_words`, `ep0_tx_words`,
  `tx_words_for(packet)`, `endpoint_count`, `max_packet`, `irq_line`.
- The readbacks: `address` (the value the driver wrote, never the
  field), `pulled_up`, `suspended`, `erratic_error`, `enumerated_speed`,
  `frame`, `session_valid`, `in_device_mode`, `core_id`,
  `fifo_words_used`, `out_armed(number)`, and the counters
  `rx_entries`, `fifo_waits`, `stalls`, `timeouts`, `setup_overruns`,
  `early_suspends`, `mode_mismatches`, `sessions`, `otg_events`,
  `enumerations`.
- The low-power verbs of 22.8: `gate_clocks(on)`, `phy_suspended`,
  `remote_wakeup(on)`.
- The stack and the class are [../design/usb.md](../design/usb.md)'s:
  `UsbDevice<UsbFs, Descriptors, Classes...>` with `start` / `stop` /
  `isr` and its counters, `UsbCdcAcm<UsbFs, P, interface, ep_notify,
  ep_data, rx, tx>` as a byte transport.

### Two things the driver does that the contract does not ask for

**The OUT side counts packets.** A device that arms one OUT packet at a
time NAKs the host between packets, and behind a hub a NAK is expensive.
This core has no double buffer to offer but it has a transfer of several
packets, which it takes back to back into the shared FIFO with no NAK in
between - so `submit_out` counts CREDITS, one per call as the contract
says, and the driver programs a transfer of as many packets as stand
armed the moment the endpoint is idle. The receive FIFO is sized for
exactly `out_slots` packets plus the ten words the chapter reserves for
setup packets, one for the global OUT NAK pattern and one per OUT
endpoint for its completion entry.

**One data packet is reported per pass.** The receive FIFO is one queue
for every endpoint, and the contract carries one packet per endpoint per
call, so the drain pops entries up to and including the FIRST data
packet and leaves the rest where they are. RXFLVL is a level - the FIFO
is not empty - so the interrupt stands and the next pass takes the next
packet. Nothing is lost and nothing is copied twice.

## How to use it

```cpp
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 96'000'000, 25'000'000>;
static_assert(SysClock::usb_hz == brio::otg_clock_hz);      // the driver asserts it too

using Usb = brio::UsbFs;
using Port = brio::UsbCdcAcm<Usb, P>;                       // interface 0, endpoints 1 and 2
struct Descriptors {
    static constexpr auto device = brio::usb_device_descriptor(
        {.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration = brio::usb_concat(
        brio::usb_configuration_head(9 + Port::descriptor_bytes, Port::interface_count),
        Port::descriptors);
    static std::span<const uint8_t> string(uint8_t index);  // 0 the languages, then the strings
};
using Device = brio::UsbDevice<Usb, Descriptors, Port>;

Usb::init(clock);
Device::start();                                            // the pull-up: the host enumerates
extern "C" void OTG_FS_IRQHandler() { Device::isr(); }
brio::print(Port{}, "hello", brio::crlf);                   // once the host has configured it
```

The port is a `ByteTransport`: a console's SerialPort runs over it as
over a UART. THE CLOCK IS THE ONE THING TO GET RIGHT FIRST - the rate a
program picks must give the PLL's Q output 48 MHz exactly, and
`Clock::usb_hz` says what a rate gives: from a 25 MHz root 96 MHz does
and the 100 MHz the same board's other programs run gives 40, from an
8 MHz root 96 and 168 MHz do and 180 gives 45. The identity 1209:0001 is
pid.codes' test pair, meant for a device that is not a product.

`sense_vbus` is false by default, which frees the VBUS pad and makes the
core take a session as valid at all times: a bus-powered board with no
VBUS wire wants that, and it is what leaves PA9 and PA10 to the console
USART. A board that wires VBUS passes true and gets the comparator.

## Bench findings

The reference suite is `test_stm32f4_usb`, green on the STM32F411CE
black pill whose USB-C connector is the part's OTG FS port: **ALL: 33
pass, 0 fail** in the all-key and eight more verdicts in the five
host-assisted letters, against a Linux host (cdc-acm bound to the port,
pyserial as the host end).

- THE CONTROLLER: `init` in 34.4 ms - all but 200 us of it the 25 ms the
  chapter asks for after device mode is forced, spent in a counted loop
  because a driver cannot assume a timebase - with the core identifier
  reading 0x1200, the mode device, the session valid (forced) and the
  pull-up still down. The FIFO map: 151 words of receive FIFO, 16 for
  endpoint zero's transmit FIFO, 167 of 320 spoken for before a class
  claims anything and 199 after the CDC's three endpoints take 16 words
  each.
- THE ENUMERATION: the pull-up up, the host enumerates the device to
  CONFIGURED in 440 to 700 ms - four bus resets, some thirty setup
  packets and six stalls over the whole all-key, no mode mismatch and no
  control-IN timeout - and `lsusb` reads the eighteen device bytes, the
  0x43 configuration bytes with both interfaces, the CDC functional
  descriptors and all four strings back.
- THE ADDRESS MUST BE IN DCFG BEFORE THE STATUS STAGE. This is the one
  place the silicon and chapter 9 disagree, and it is the whole
  difference between a device that enumerates and one that does not:
  with the address written after the status stage of SET_ADDRESS - which
  is what the specification says and what the stack above does, because
  it is what a controller whose address register bites at once needs -
  the host takes the status, asks its next question at the new address
  and is answered by nobody: seven bus resets and "device descriptor
  read/8, error -32" from the kernel, every attempt. RM0383 22.17.5 puts
  the two steps the other way round, and with DCFG written the moment
  the SETUP packet is popped the same host enumerates in six hundred
  milliseconds. So the driver takes the address out of the SETUP packet
  as it passes - the one place it reads one - and the stack's own
  `set_address`, a status stage later, writes the same value again.
- STUPCNT IS NOT RESTORED BY THE CORE. It counts down one per SETUP
  packet, and a control transfer whose whole answer is a status IN
  (SET_ADDRESS, SET_CONFIGURATION) never touches DOEPTSIZ0 - so after
  three of those the endpoint would stop taking SETUP packets. It is
  topped up at every setup.
- A DETACHED CORE IS NOT A SUSPENDED ONE: with the pull-up down, 200 ms
  of a silent bus raise no early suspend and no SUSPSTS. The suspend
  condition is an idle bus the device is still ATTACHED to - and that
  state is free to measure, in the window between the pull-up going up
  and the host noticing it: SUSPSTS comes up **6007 us** after the
  pull-up rises, which is the chapter's three milliseconds to the early
  suspend and three more to the suspend, and it goes down again 130 to
  180 ms later when the host resets the bus.
- THE CLOCK GATING of 22.8: with the bus suspended, STPPCLK and GATEHCLK
  set, PCGCCTL.PHYSUSP comes up within 1 us; ungated, the same window
  ends with the host enumerating the device as usual. Gated while merely
  DETACHED, PHYSUSP never comes up - the PHY suspends for a suspended
  bus, not for an absent one.
- THE START-OF-FRAME counter: 100 frames in 100001 us.
- A RECONNECT: the pull-up dropped for 500 ms detaches the device;
  raised, the host resets the bus twice more and enumerates afresh in
  520 to 700 ms with the port configured again.
- STALLS: a stall stands on both directions of a bulk endpoint and reads
  back, clears with the data toggle put back, and endpoint zero takes
  one on both directions - the silicon clearing it at the next setup
  token, as chapter 9 wants.
- ECHO: every byte of 201469 sent by the host in chunks of 1 to 300
  bytes over nine seconds came back byte-exact, no overrun, no packet
  held for want of FIFO room.
- THROUGHPUT: the device pours **658 KB/s** into the port (548 counted
  by the host over its own window) and takes **440 KB/s** from it with
  no overrun. The transmit path never once had to wait for the
  empty-level interrupt: a transmit FIFO of one packet and one packet in
  flight means the room is there by construction.
- THE FIRST TRANSFER OF A FRESHLY CONFIGURED OUT ENDPOINT CARRIES ONE
  PACKET, and every one after it as many as stand armed: the class arms
  its eight credits one call at a time, the first of them finds the
  endpoint idle and programs a transfer of one, and the seven that
  follow cannot grow a packet count the core is already walking. The
  endpoint reads back one armed packet after the configuration and eight
  from the next transfer on. One NAK window at the start of a
  configuration, and none after it.
- NAK FLOW CONTROL: with nobody reading, the receive ring takes 1984
  bytes of its 2047 and the endpoint runs out of armed packets; the host
  is NAKed from there on and its writes eventually time out. When the
  device starts draining, all 1097728 bytes the host wrote arrive with
  **zero breaks** in the counting pattern and no overrun - a NAK costs
  no byte.
- THE ZERO-LENGTH PACKET: 1024 bytes - sixteen whole packets and not a
  byte more - go out in 1.3 to 1.5 ms and the host's read returns at
  once with all of them, which is what the zero-length packet after the
  last full one is for.
- THE LINE CODING AND DTR: a port set to 19200 7E2 reads back
  19200/7/2/2 through the control data stage, and DTR and RTS lowered
  and raised move the line-state counter.
- THE CONSOLE over the port (`console_usb`): HELP, UPTIME, LED, USB and
  ERR answered on /dev/ttyACM with no driver installed, the banner held
  until the host raises DTR. A HOST-SIDE TRAP, not the device's: on a
  Linux desktop ModemManager probes a new CDC ACM port that declares the
  AT-commands protocol, so the first line a terminal sends may land
  behind a probe's leftovers - send an empty line first.

## Not covered yet

Driver gaps, each with its reason:

- Host mode and the OTG role switch (22.4, 22.6, 22.17.8): brio has no
  host side, by decision ([../design/usb.md](../design/usb.md)).
- Isochronous endpoints: refused by `configure_endpoint`. The even/odd
  frame this core wants programmed per transfer has no place in a
  contract drawn at the packet, and no class here streams; a streaming
  class would be the reason to widen both.
- The internal DMA (GAHBCFG.DMAEN, the high-speed core's): the
  full-speed core has none, and the endpoint FIFOs at 96 MHz are not the
  bottleneck of anything measured here.
- VBUS sensing from the pad, the SOF pulse on OTG_FS_SOF, and the
  session request protocol: the board wires no VBUS and no ID, so the
  pad is given away and `sense_vbus` false is the only path with a
  witness.
- Babble on an OUT endpoint: RM0383's DOEPINTx table names BERR at bit
  12 and the CMSIS device headers declare no such bit, so there is no
  header symbol to name it by and this stratum does not invent register
  facts. The mask bit (DOEPMSK.BERRM) is left masked.
- More than one packet of OUT data in a control transfer's data stage:
  the stack's limit, not this driver's
  ([../design/usb.md](../design/usb.md)).
- A second USB clock source (the F412/F413's and F446's CK48MSEL, which
  can take the 48 MHz from PLLSAI or PLLI2S instead of the main PLL's
  Q): the parts on the bench have no such selector, and the two whose
  manuals are on the desk do not need one.

Implemented but not bench-verified, each with what would measure it:

- `UsbHs`, the high-speed core through its own full-speed PHY: it
  compiles for every part whose header declares the second base address
  and it is written from the same chapters, but no board here cables
  PB14 and PB15 to a host. A board with a second USB connector, or the
  STM32F429I-DISC1's own OTG HS receptacle, would measure it - and it is
  also the way out of ES0206 2.2.14 on an impacted F42x/F43x part.
- `sense_vbus = true`: the code path is there and the GCCFG value
  differs, but a board that wires VBUS to the pad is what would show the
  session coming and going with the cable.
- `remote_wakeup`: a host that suspends the port and then lets the
  device wake it. The port here is never suspended by the host, only by
  the window before it notices the pull-up.
- The turnaround values below 0x6 (table 132's slower AHB bands): every
  rate that gives the core its 48 MHz on this family is above 32 MHz, so
  only the fastest band is ever chosen; a part clocked from a slower
  root would take the others.
- The empty-level interrupt finishing a packet the FIFO had no room for:
  with one packet in flight and a transmit FIFO of one packet the room
  is always there, so `fifo_waits` stayed zero through every measure. A
  transmit FIFO deliberately made smaller than a packet would exercise
  it.
- Endpoints 3 and above, and a second class in one device: the CDC port
  claims one interrupt and one bulk pair, which is three of the four the
  full-speed core has. A second `UsbCdcAcm` on interfaces 2 and 3 would
  need endpoint 3 for both its notification and its bulk pair, which
  this core cannot give - the high-speed core's six endpoints can.
