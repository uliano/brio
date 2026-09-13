# USB - the device stack over an endpoint controller

The target-independent design of brio's USB device side:
[`util/usb/device.hpp`](../../brio/util/usb/device.hpp) (the setup
packet and chapter 9's vocabulary, the descriptors as constexpr bytes,
the `UsbController` and `UsbClass` contracts, `UsbDevice` - the
control-endpoint machine) and
[`util/usb/cdc.hpp`](../../brio/util/usb/cdc.hpp) (CDC ACM: the
serial port class that is also a byte transport). The host side of
USB is not brio's and will not be.

## Why the cut is at the packet

Every USB device controller brio meets hands packets over
differently - a dual-port RAM with a control word per endpoint, a
FIFO, a table of buffers in main memory - and every one agrees on
what the host can see: endpoint zero takes eight-byte setup packets
and answers them in stages; the other endpoints move packets of at
most their maximum size; a bus reset puts the device at address zero.
So the controller contract is drawn at the packet and nowhere else:

- `submit_in(number, bytes)`: ONE packet to send, copied into the
  controller's own storage before the call returns, reported in
  `in_done` when the host took it;
- `submit_out(number, max)`: ONE packet's reception armed, reported in
  `out_done` with its bytes in `out_data(number)` until the endpoint
  is armed again;
- `setup()`, `set_address`, `configure_endpoint` / `deconfigure_endpoints`,
  `stall` / `stalled`, `connect` (the pull-up), and `take_events()` -
  the reset, the setup packet, suspend and resume, the completions,
  acknowledged.

Everything above the packet is written once: the control transfer's
stages, the descriptors, the address that takes effect only after
its own status stage (the status travels at the old address), the
configuration that claims the classes' endpoints, the routing of a
class request to the interface it names, of a packet to the endpoint
that owns it. The stack is PROVEN ON THE HOST first against a
scripted controller (`host/sim_usb.hpp`: the test plays the host one
packet at a time) and meets silicon afterwards; a new controller is
a family chapter that realizes the contract and nothing more.

Data PIDs, buffer ownership, the double-buffering a controller may
offer: the controller's, invisible above the contract. The stack
never sends more than one packet per endpoint at a time, which is
what keeps the contract small and every controller honest.

## Where it runs

The whole stack is an ISR body: `UsbDevice::isr()` is what the
application's controller vector calls. Enumeration is a dialogue the
host paces in milliseconds, and every answer is a copy of constexpr
bytes or a one-line state change, so nothing is deferred to an active
object. A class with more to do than moving bytes (the serial port's
rings) does it in the same body and hands the rest to the program
through its own surface. The program's side of a class runs in the
main loop under the platform's guard, the ISR's side in this body:
brio's two contexts and one boundary, as everywhere.

## The control transfer

A setup packet opens a transfer and closes any pending one. A request
answered with SEND goes out in packets of the control endpoint's size
and, when the data is shorter than the host asked and ends on a packet
boundary, a zero-length packet after it; the host's empty OUT is the
status stage. A request answered with RECEIVE arms the control
endpoint for the host's data (one packet at most: the class requests
brio carries fit in one), hands it to the class, and answers an empty
IN as the status. ACK answers the empty IN at once; STALL stalls both
directions of endpoint zero until the next setup packet. The standard
requests are chapter 9's: the three descriptors, the address, the
configuration (one, by decision: a second configuration is a program
nobody has written), the status and the endpoint halt, the interface
(alternate zero only), the device qualifier stalled because every
controller brio runs is full speed.

## A class

A type with static verbs: the interfaces it owns, the descriptor
bytes the application glues after `usb_configuration_head`,
`configure()` when the host chooses the configuration (claim the
endpoints, arm them), `reset()`, `control(setup)` for the class
requests aimed at its interfaces (an answer of SEND, RECEIVE, ACK or
STALL), `control_data` for the bytes a RECEIVE brought, `in_done` /
`out_done` for its endpoints' packets. A device lists its classes as
template arguments; the stack asks each whether an interface or an
endpoint is its.

CDC ACM is the first class and the reason for the stack: a serial
port the host needs no driver for, with `write_byte`, `read_byte`
and `tx_idle` - the surface of a UART - so a console or a SerialPort
runs over the chip's own connector unchanged. Its flow control is
USB's own: the OUT endpoint is re-armed only while a whole packet
fits in the receive ring, and the host NAKs meanwhile, losing no byte.
The line coding and the control line state are received and reported,
not obeyed: a virtual port has no baud rate.

## Realizations

| stratum | controller | beyond the contract |
|---|---|---|
| rp2040 | `Usb` (`rp2040/usb.hpp`), the chip's own controller over its dual-port RAM | 64-byte buffers handed out in the order the classes claim endpoints; the two-step buffer control write the clock ratio demands; endpoint zero's stall armed in a second register; the data PIDs kept per endpoint and direction, endpoint zero's restarted at DATA1 by a setup packet; clk_usb at 48 MHz from the USB PLL the converter shares |
| host | `SimUsb` (`host/sim_usb.hpp`) | the scripted controller the stack's tests play the host against: one packet per endpoint and direction, NAKs and stalls counted, a controller that refuses to configure |

Every other family with a controller (STM32 G0B1 and G4, the F4's
OTG, the CH32V203's) is a chapter of its own; the ones without
(AVR DA/DB, SAM C21, CH32V006) keep their console on a probe's bridge.

## Not covered yet

- Isochronous endpoints and double buffering: no class here streams
  audio; the contract admits one packet in flight per endpoint by
  decision, and a streaming class would be the reason to widen it.
- A second configuration, alternate interface settings other than
  zero, remote wakeup acted on: stalled or ignored by the rules; the
  first program that needs one brings it.
- Control transfers with an OUT data stage of more than one packet:
  every class request brio carries fits in one; a vendor class with a
  longer one would extend the machine by a stage.
- HID and a vendor bulk class: the roadmap's next two classes over
  the same stack, the second the bench's fastest link.
