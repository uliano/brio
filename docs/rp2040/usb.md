# USB (RP2040)

This document is for a programmer who wants an RP2040 program to talk
to a computer over the chip's own USB connector, with no probe and no
adapter - most often as a serial port the computer sees as
`/dev/ttyACM*`, a console that needs no driver installed. Three layers
do it: `Usb`, this stratum's driver of the chip's device controller;
`util/usb/device.hpp`'s `UsbDevice`, the device stack that answers the
host's enumeration; and `util/usb/cdc.hpp`'s `UsbCdcAcm`, the serial
port class, a byte transport with the same surface as a UART's, so a
console's `SerialPort` and `print` run over it unchanged
([design/usb.md](../design/usb.md) is the stack and its contracts).

The smallest use, a serial port on the connector, from "How to use it"
below:

```cpp
using Port = brio::UsbCdcAcm<brio::Usb, P>;                 // interface 0, endpoints 1 and 2
...
using Device = brio::UsbDevice<brio::Usb, Descriptors, Port>;

brio::Usb::init(clock);
Device::start();                                             // the pull-up: the host enumerates from here
extern "C" void isr_usbctrl() {
    Device::isr();
    if (Port::take_rx_edge()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
brio::print(Port{}, "hello", brio::crlf);                     // once the host has configured the port
```

Line by line: `Port` names the serial class over this chip's controller
and the program's platform `P`; `Descriptors` (written out under "How to
use it") is the device's identity - the device descriptor, the
configuration with the port's own descriptors glued in, the strings -
and `Device` the stack over the controller with the port as its one
class; `Usb::init()` takes the clock's tag ([clock.md](clock.md)),
starts the USB PLL at 48 MHz unless it runs already, releases the
controller and enables its line, with the pull-up still down;
`Device::start()` raises the pull-up, and the host sees a device
attach and starts enumerating; `isr_usbctrl` is the controller's
handler under the pico-sdk's name: `Device::isr()` answers the host,
and `take_rx_edge()` reports that the port received bytes, which the
console posts to the active object that assembles lines; `print` writes
to the port as to a UART.

What decides correctness even there. The vector is bound and calls
`Device::isr()`: the whole enumeration runs in that handler, and an
unbound vector parks the core in the crt's `Default_Handler` spin at the
first bus event. The port takes bytes only once the host has configured
it: before that `write_byte` and `write_bulk` refuse everything, so a
`print` - which waits until every byte is taken - spins until the host
has enumerated the device and configured the port, and a program whose
first line must be seen waits for `Port::configured()` and for the
host's terminal to raise DTR (`Port::dtr()`) before it prints. clk_sys
runs above 48 MHz, erratum E16's condition, which `init()` refuses at
compile time otherwise. And the identity 1209:0001 the descriptors under
"How to use it" name is a test pair, not a product's: a product names
its own.

## Documents of record

The RP2040 datasheet (build 3184e62), 4.1 (the
USB controller: 4.1.2 the architecture - the clock, the PHY, the dual-
port RAM and its layout, the endpoint and buffer control words, the
device controller's SETUP, IN and OUT handling, suspend and resume -,
4.1.3 the programmer's model, 4.1.4 the registers), the errata E2 and
E5 (the B0 and B1 steppings); docs/design/usb.md for the stack and
the contracts. The driver: `brio/rp2040/usb.hpp` (`Usb`, the
endpoint controller) over `clock.hpp` (`Clocks::usb_select`, the USB
PLL) and `resets.hpp`; the stack `util/usb/device.hpp` and the class
`util/usb/cdc.hpp` above it. The reference suite: `test_rp2040_usb`,
the host as the other end (a suite's letter is one command key of its
console and the group of verdicts it runs; the all-key runs the set,
and a host-assisted letter outside it runs only when asked by name).

## What the silicon does

One controller, one full-speed PHY on the chip's own two pins with
its pull-up, and 4 KB of dual-port RAM that is the whole interface
between the core and the controller: the eight-byte setup packet at
offset 0, an endpoint control word per endpoint and direction (the
type, the buffer's offset, single or double buffered, an interrupt
per buffer), a buffer control word per endpoint and direction (the
length, the data PID, FULL, AVAILABLE, STALL; a second half for the
second buffer), endpoint zero's 64-byte buffer at 0x100 and 3840
bytes of buffers from 0x180. The registers hold the device address,
the muxing onto the PHY, the SIE's control and status (the bus reset,
the setup packet, suspend, resume, the error flags), the interrupt
trio, and BUFF_STATUS, one bit per endpoint and direction, that says
which buffers completed - with BUFF_CPU_SHOULD_HANDLE naming the half
on a double-buffered one. The controller wants clk_usb at 48 MHz and
clk_sys above it (erratum E16). A buffer is handed over by writing
its control word in two steps, the length and the PID first and
AVAILABLE after a few clk_usb periods, because the controller reads
the word at 48 MHz while the core writes it at 125.

Three facts measured that the chapter does not state as they stand:
the controller's buffer select of a double-buffered endpoint keeps
its value across a reconfigure, so the first offer after one carries
RESET_BUFSEL or the halves and the host's data toggles disagree; the
halves of a double-buffered OUT endpoint complete in strict
alternation and each half's FULL bit says whether it did, which is
the way to tell the two apart when both completed before the
interrupt ran; and a NAK costs a whole frame behind a hub's
transaction translator, so a single-buffered bulk OUT endpoint takes
some 60 KB/s and a double-buffered one over 900.

## Types and verbs

- `Usb`: `init(clock)` (the USB PLL at 48 MHz if it is not there,
  clk_usb from it, the block out of reset, its RAM cleared, the PHY
  muxed in, VBUS detection forced because the boards wire no VBUS
  pin, device mode, the interrupts at the block and at this core's
  NVIC; the pull-up down), `release`; the `UsbController` contract:
  `connect(on)` (the pull-up), `set_address`, `configure_endpoint(
  address, type, max)` (64-byte buffers handed out in claim order; a
  bulk OUT endpoint gets two halves), `deconfigure_endpoints`,
  `stall` / `stalled` (endpoint zero's armed in EP_STALL_ARM too),
  `submit_in(number, data)` (the packet copied into the RAM, the PID
  kept per endpoint and direction, endpoint zero's restarted at DATA1
  by a setup packet), `submit_out(number, max)` (the next half in
  turn on a double-buffered endpoint, its own 16-bit control word),
  `out_data(number)`, `setup()`, `take_events()` (the SIE flags and
  BUFF_STATUS read and acknowledged, the completed half and its
  length read first); `out_slots` (two: what a class arms at once).
  The readbacks: `connected`, `suspended`, `vbus_detected`,
  `pulled_up`, `address`, `frame` (the last start-of-frame number),
  `sie_status`, `take_errors` (CRC, bit stuff, overflow, timeout,
  data sequence), `buffers_used`.
- `clock.hpp` grew `Clocks::usb_select(aux, div)`, `usb_stop`,
  `usb_enabled`, `usb_source` with `UsbAux`.
- The stack and the class are docs/design/usb.md's: `UsbDevice<Usb,
  Descriptors, Classes...>` with `start` / `stop` / `isr` and its
  counters, `UsbCdcAcm<Usb, P, interface, ep_notify, ep_data, rx,
  tx>` as a byte transport.

## How to use it

```cpp
using Port = brio::UsbCdcAcm<brio::Usb, P>;                 // interface 0, endpoints 1 and 2
struct Descriptors {
    static constexpr auto device = brio::usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration = brio::usb_concat(
        brio::usb_configuration_head(9 + Port::descriptor_bytes, Port::interface_count), Port::descriptors);
    static std::span<const uint8_t> string(uint8_t index);   // 0 the languages, then the strings
};
using Device = brio::UsbDevice<brio::Usb, Descriptors, Port>;

brio::Usb::init(clock);
Device::start();                                             // the pull-up: the host enumerates from here
extern "C" void isr_usbctrl() {
    Device::isr();
    if (Port::take_rx_edge()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
brio::print(Port{}, "hello", brio::crlf);                     // once the host has configured the port
```

The port is a `ByteTransport`: a console's SerialPort runs over it as
over a UART. The identity 1209:0001 is pid.codes' test pair, meant
for a device that is not a product; a product states its own.
`Port::write_bulk(run)` queues a run before the first packet goes,
where `write_byte` sends its first byte alone and batches the rest
behind it - and `print` hands it every `string_view` whole and a C
string, every number among them, as its first byte and then the rest,
so a print that finds the port idle sends that byte in a packet of its
own.

## Bench findings

The reference suite is `test_rp2040_usb`, green on the WeAct board:
the enumeration and the reconnect in the all-key, the four host
letters against a Linux host (cdc-acm bound to the port, pyserial as
the host end) through a hub.

- THE CONTROLLER: init in 125 us with clk_usb counted at 48 MHz, VBUS
  seen, the pull-up down; raised, the host enumerates the device to
  CONFIGURED in 550 to 720 ms - two bus resets (Linux resets once
  more after the first descriptor), 13 setup packets, three stalls
  (a device qualifier and the strings it has not), the address in
  the twenties -, the class's three endpoints claimed with the bulk
  OUT's two halves armed, the start-of-frame counter advancing a
  frame a millisecond, no SIE error.
- A RECONNECT: the pull-up dropped for half a second detaches the
  device; raised, the host resets the bus three more times and
  enumerates afresh in 710 ms, the port configured again.
- ECHO: every byte of 2.5 MB sent by the host in chunks of 1 to 300
  bytes over ten seconds came back byte-exact, no overrun, no SIE
  error; 300 KB as a stream echoed in 540 ms.
- THE LINE CODING AND DTR: a port opened at 19200 7E2 reads back
  19200/7/2/2 through the control data stage, DTR and RTS raised
  through SET_CONTROL_LINE_STATE.
- THROUGHPUT: the device pours 559 KB/s into the port (479 counted by
  the host over its window) with the single-buffered bulk IN; the
  host pours 907 KB/s into the device (648 counted by its own writes)
  with the double-buffered bulk OUT and no overrun - 85 KB/s before
  the second half, the frame per NAK behind the hub.
- THE CONSOLE over the port (the USB console app): HELP, UPTIME, LED,
  ERR and USB answered on /dev/ttyACM with no driver installed, the
  banner held until the host raises DTR - on the WeAct board and on
  the Pico at once, two ports on one host, each enumerated in two
  resets and sixteen setup packets.
- TWO PORTS IN ONE DEVICE (letter x, host-assisted): the device restarted
  with a second `UsbCdcAcm` on interfaces 2 and 3 (notification on
  endpoint 3, the bulk pair on endpoint 4) - one configuration of four
  interfaces, 125 bytes, the descriptors glued in order - enumerates to
  CONFIGURED in 0.35 to 0.73 s, Linux's cdc-acm binding both as two
  ttyACM (by-id `-if00` and `-if02`, with no interface association
  descriptor); 8192 bytes written to each, port A echoing them and port
  B each plus one, both exact at the host with no overrun and no SIE
  error; then the one-port device back. Ten runs.

## Not covered yet

Driver gaps, each with its reason:

- Host mode (4.1.2.9): brio has no host side, by decision
  (docs/design/usb.md).
- Isochronous endpoints and a double-buffered IN: no class streams,
  and the bulk IN at 559 KB/s single buffered is not the bottleneck
  of any program here; a second half on the IN would be the first
  thing a streaming class asks.
- VBUS detection from a pin (2.19's VBUS function on a GPIO) and the
  overcurrent input: the boards wire neither; detection is forced.
- Remote wakeup (SIE_CTRL.RESUME) and the suspend current: the sleep
  chapter's meter; the suspend and resume events are counted.
- The errata E2 and E5 of the B0 and B1 steppings: every board on the
  bench is B2, where the silicon fixes both.

Implemented but not bench-verified, each with what would measure it:

- `Usb::take_errors` reporting a real error: a cable pulled mid-packet,
  which the chapter says raises them.
- The reference suite on a Pico: the console app enumerates and
  answers there; the suite's letters want the probe's console, which
  the Pico lends to the WeAct board's second core meanwhile.
