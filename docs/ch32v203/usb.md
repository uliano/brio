# USB device controller (CH32V203)

A full-speed USB DEVICE controller that is, register for register, the
one on the STM32F1 and F0 lines under WCH's names. Documents of record:
the CH32FV2x_V3xRM reference manual V2.3, chapter 21 whole - one manual
covers four families and what this page states is the CH32V20x_D6
class's, every part up to the CH32V203C8 - with 21.2.1 for the packet
memory it shares with the CAN, 21.2.2 for the wake from power-down,
21.2.4 for the suspend and wake process, 21.3 for the registers and
33.2.1 for the pull-up, which lives outside the chapter; the datasheet CH32V203DS0 V2.8 table 2-1 for which parts
carry the block and its pin tables for the pads; and
[../design/usb.md](../design/usb.md) for the contract this driver
realizes. Driver:
[brio/ch32v203/usb.hpp](../../brio/ch32v203/usb.hpp) (`Usbd`), with the
stack [util/usb/device.hpp](../../brio/util/usb/device.hpp) and the
class [util/usb/cdc.hpp](../../brio/util/usb/cdc.hpp) above it, the
clock tree of [clock.md](clock.md) feeding it and
[sleep.md](sleep.md)'s count of bus masters keeping the core awake for
it. Reference suite: `test_v203_usb`, the console on the probe's UART
and the board's own connector under test; the enumeration's own
apparatus is `usb_probe`, the instrument that asks the controller what
it is doing while a host tries to enumerate it.

## What the silicon does

### ST's controller under other names

Eight endpoint registers whose status fields are toggles, a control
register and a status register, a device address, a frame counter, the
table offset, and a packet memory the CPU and the engine share and the
program lays out itself. Nothing here is the Synopsys core of the
STM32F4 and nothing is the RP2040's: it is a third shape, and the
reason it lives in this stratum rather than in one of its own is that
an IP stratum is born at the SECOND family that carries the IP.

The part table says which packages have it
([parts/](../../brio/ch32v203/parts/), `device::has_usbd`): every one
of the nine but the CH32V203F8, whose TSSOP20 bonds neither PA11 nor
PA12. A part without it refuses the type at compile time.

### The packet memory is 512 bytes seen through a 32-bit window

Each 16-bit word of the shared memory occupies four bytes of the CPU's
address space, so the PMA byte offset `o` is at 0x40006000 + 2*o and
nothing wider than a halfword may be stored there. The first 64 bytes
hold the buffer description table - eight entries of four halfwords,
the transmit address and count and the receive address and count of
each endpoint - and this driver puts it at offset zero and hands
buffers out above it, in the order the classes claim their endpoints.

COUNTn_RX is two fields in one halfword, which is the sharpest of the
small edges here: the program writes the buffer's SIZE into it as a
BLOCK COUNT (blocks of two up to 62 bytes, of thirty-two from 64 up)
and the hardware writes the RECEIVED count into the same halfword. An
endpoint left as the hardware leaves it therefore has no size any more,
which is why the driver rewrites that field every time it arms a
reception - measured: an endpoint armed without it acknowledges the
next packet and stores none of it, the count arriving in an otherwise
untouched memory.

### And the memory is shared with the CAN controller

Per 21.2.1, when the CAN controller is used its filter table takes the
TOP 128 bytes of the 512 and the USB keeps the low 384. Whether CAN is
in use is not a fact this file can see, so it is a PARAMETER and not a
guess: `Usbd<>` budgets 384 bytes and refuses the endpoint that would
not fit, and a program with no CAN says `Usbd<512>` and gets the rest.
A CDC ACM port spends 328 of them - the table's 64, endpoint zero's two
buffers of 64, the notification's 8 and the bulk pair's two of 64.

### The status fields are written by XOR

STAT_RX, STAT_TX, DTOG_RX and DTOG_TX of an endpoint register are
"write 1 to INVERT, write 0 to leave" (21.3.6), while CTR_RX and CTR_TX
in the same register are "write 0 to clear, write 1 to leave". No field
of that register can therefore be stored directly: every write computes
the XOR that lands on the value it wants and puts ones in the two
completion flags it must not disturb. The status register ISTR is the
third convention again - read-and-write-zero, so a flag is cleared by
storing a word with a zero in that one bit and ones everywhere else -
and its CTR bit is not clearable there at all: it stands until the
endpoint's own flag falls.

The endpoint TYPE codes are the chapter's and not the descriptor's
(bulk is 00 here and 10 on the wire), which is a translation the driver
owns so that nothing above it has to know.

### The pads are GPIO pads, and that is nowhere in the chapter

D- and D+ are PA11 and PA12, and the block reaches them THROUGH PORT A.
With that port's clock closed - which is how it comes out of reset -
the pull-up still works, because the pull-up is not in this block
either: it is EXTEN_CTR's bit 1 (33.2.1). So a program that opened only
the USB's own gate presents a device to the host and then fails every
packet. MEASURED that way: the SETUP counted but its first halfword
never stored, sixteen packet-memory overflows in one attempt, the host
giving up with a protocol error. WCH's own `USB_Port_Set()` opens port
A's clock and leaves the two pads floating inputs before it touches the
pull-up, and `init()` does the same.

### One clock rate, and it is the tree's business

The controller wants 48 MHz exactly, and RCC's USBPRE divides the PLL
by 1, 2 or 3 to make it - so only a PLL at 48, 96 or 144 MHz can feed
it, the divider is programmed before the USB gate is ever opened
(3.4.2's order), and `init()` refuses at COMPILE time a tree that
cannot ([clock.md](clock.md)). Full speed wants that 48 MHz within 2500
ppm, and every HSI-rooted tree of this family measures four tenths of a
per cent fast (the clock chapter) - nearly twice the allowance - so a
program with a USB takes its PLL from a crystal.

### The core must not sleep while the controller works

MEASURED, and the manual says the opposite: 2.4's table gives Sleep as
"core clock off, no effect on other clocks" and its prose as "the core
stops running and all peripherals are still running". With the core in
WFI or in the platform's WFE idle this controller cannot reach its
packet memory. An armed bulk endpoint receives NOTHING - the host's
bytes are lost and the overflow counter climbs one per attempt - and an
enumeration never gets past its first control transfer: the SETUP is
taken and answered, the host acknowledges the data, and the status
stage dies with an overflow. The same image with a loop that only
steps enumerates, configures and carries bytes with not one overflow.

It is not the USB's alone and it is not this driver's doing. In a sleep
of any depth here NO BUS MASTER BUT THE CORE GETS A CYCLE: a
memory-to-memory DMA started just before a sleep moves the handful of
items already in its pipeline and then nothing until the core wakes,
while the clocks keep running (the timers and the core's own counter
count the whole sleep). The controller's reach into its packet memory
is such an access, which is why a transfer with a payload fails and one
without - SET_ADDRESS - passes.

No mitigation short of staying awake works, and they were measured one
by one: unmasking the overflow interrupt wakes the core but the packet
is already gone and the host does not retry a failed bulk burst;
staying awake for ten or a hundred milliseconds after an overflow
recovers nothing of the burst in flight; holding the endpoint at NAK
across the sleep still overflows; re-validating the endpoint finds it
valid. What DOES work is not sleeping and dividing HCLK instead: the
device enumerates and carries its four kilobytes with HCLK at 96, at 48
and at 24 MHz with the USB clock at 48, and fails at 12 MHz with the
sleeping case's own signature. So 24 MHz is the floor, `init()` refuses
a slower tree at compile time (`usbd_min_hclk_hz`), and what the
program gives up on this family is not throughput but sleep.

THE VENDOR'S OWN EXAMPLE FAILS THE SAME WAY, which is what settles
whose finding this is: the EVT's SimulateCDC, built with WCH's compiler
and flags and unchanged but for one `wfi` at the end of its loop, never
reaches CONFIGURED - SET_ADDRESS gets through, the descriptor reads do
not, the overflow counted 18 to 26 times - and its own `__WFE()` fails
the same while a busy-wait of the same length works. A variant that
sleeps only once configured loses every one of the 4096 bytes the host
writes, and the loss happens with the core woken at least once per USB
frame, so it is neither the depth nor the length of the sleep. There is
no errata sheet for this family, and the absence of an erratum is
evidence of nothing.

AND SO IT IS A MECHANISM AND NOT AN INSTRUCTION TO THE PROGRAMMER. An
attached controller counts itself one bus master
([bus_activity.hpp](../../brio/ch32v203/bus_activity.hpp)), the
platform's `idle()` does not sleep while that count stands and a sleep
site refuses to arm over it ([sleep.md](sleep.md)). `connect()` is
where the count is taken and given back - from the pull-up to the
detach, deliberately, and not from the first packet to the suspend -
so a program with a USB may drive the kernel any way it likes, its
idle path included, and what decides is the silicon's own state.

### An overflow costs the endpoint its buffer, and the driver gives it back

COUNTn_RX once more, and this is its sharpest face. A reception that
completes writes its length into the low ten bits and leaves the SIZE
above them standing - measured 0x8440 where a 64-byte buffer's encoding
is 0x8400 and the packet was 64 bytes. A reception that OVERFLOWS does
not: the size field comes back ZERO and the halfword reads as the bare
count, 0x0040 (measured over the debug port on an endpoint that had
just lost a packet). And because the packet never completed, no CTR_RX
is raised and no layer above ever arms that endpoint again: it stays
VALID over a buffer that now says zero blocks, so EVERY packet after it
overflows too. Measured, before the driver answered it: an endpoint
left alone after one overflow took 64 bytes in 200 ms where a healthy
one takes ninety thousand, and the overflow counter climbed for as long
as the host kept sending.

So the packet-memory overflow is not a flag this driver only counts:
`take_events()` writes the size encoding back for every endpoint that
is still armed and has no reception standing. The lost packet is still
lost - a control transfer survives because the HOST retries it, a bulk
burst does not - but the port is a port again the moment the cause
stops.

### The suspend is a handshake, and the wake-up is its answer

21.2.4 asks the program to answer the bus going idle by putting the
module in suspend itself: FSUSP shields the detector, which would
otherwise raise the flag again for as long as the bus stays idle, and
LPMODE is the low-power half beside it. THE WAKE-UP IS WHAT MAKES THAT
MANDATORY AND NOT A COURTESY: the condition for WKUP is a wake-up
signal reaching a SUSPENDED module, so a program that reports the
suspend and leaves the block running never hears the resume. Measured
that way before the driver kept the handshake: the suspend counted, the
host's resume never, over twenty seconds of a bus carrying frames
again. With FSUSP set on the event and cleared on the wake-up, both are
counted and the port is configured and carrying frames on the other
side. A bus reset ends any suspend too, so the reset path clears FSUSP
before it lays the endpoints out again.

### Two vectors, and this driver uses one

The low-priority line (interrupt 36, shared with the CAN's receive
FIFO 0) carries every event of a single-buffered device: the bus
reset, the completions, suspend and resume, the errors and the
overflow. The high-priority one (35, shared with the CAN's transmit)
belongs to double-buffered and isochronous endpoints, which this driver
does not offer, so it is never enabled. A third line, the wake-up
(58), is EXTI line 18's - the path that would bring the core out of a
low-power mode on bus activity, which this driver does not use either.

A bus reset is taken literally by this block: the endpoint registers,
the address and the table offset are all back at their reset values,
so the control endpoint must be laid out again before the host's first
SETUP, which follows within microseconds. The driver does it in the
event handler and guards the one case that would ruin it - a reception
already standing in the buffer, whose length would be destroyed by
rewriting the size field over it (measured with the handler held at a
breakpoint: the reset and the first SETUP can both be pending when it
finally runs, and a SETUP of length zero is a request the stack can
only stall).

## Types and verbs

`Usbd<pma_bytes>` is the controller, everything static; `pma_bytes` is
what the program gives the USB of the shared memory, 384 by default
(the CAN-safe budget) and 512 for a program with no CAN. `init(clock)`
is the block's clock, 21.2.2's wake from power-down, the two pads
through port A, the control endpoint laid out and the interrupts armed
- with the pull-up left DOWN, because the host must not see a device
before the program can answer it; `release()` gives all of it back.
Both of the compile-time refusals live in `init()`: the 48 MHz the tree
must make, and `usbd_min_hclk_hz`, the measured floor of the bus.

The [design/usb.md](../design/usb.md) contract is realized verbatim:
`connect(on)` (the pull-up in EXTEN, and the bus-master count),
`set_address`, `configure_endpoint(address, type, max)` (a buffer out
of the budget, the endpoint left at NAK), `deconfigure_endpoints`,
`stall` / `stalled`, `submit_in(number, data)` (one packet copied into
the shared memory and the endpoint armed to answer the next IN token;
an empty span is the zero-length packet), `submit_out(number, max)`,
`out_data(number)`, `setup()` and `take_events()` - which drains the
status register in a bounded loop, because several events stand at once
and CTR falls only with the endpoint's own flag, and which is also
where the two repairs live: FSUSP set and cleared around the host's
suspend, and the receive buffers' size written back on an overflow.

The readbacks are what a program and a suite ask: `frame()` (the
host's start-of-frame number), `address()`, `pulled_up()`,
`buffer_used()` / `buffer_free()` (the shared memory spent and left),
`errors()` (the block's error flag, counted) and `overruns()` (the
packet-memory overflow, counted). Both counters SATURATE - a counter
that wrapped would report a healthy bus - and both count from `init()`,
so what they carry is what this bring-up of the block has seen and not
what the last one did.

`usbd()` is the register view - `Usbd::regs()` the same block reached
through the type - and `usbd_pma(offset)` the window into the shared
memory, for an instrument that has to look at what a driver has no
reason to read.

Above them nothing is this stratum's: `UsbDevice<Usbd<>, Descriptors,
Classes...>` is the control-endpoint machine and `UsbCdcAcm<Usbd<>, P,
...>` the serial-port class, both of [design/usb.md](../design/usb.md).

## How to use it

A console on the chip's own connector - the whole of it, because the
stack is where everything above the packet lives:

```cpp
using Usb = brio::Usbd<>;                                  // 384 bytes, CAN-safe
using Port = brio::UsbCdcAcm<Usb, P>;                      // interface 0, endpoints 1 and 2
struct Descriptors {
    static constexpr auto device = brio::usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration = brio::usb_concat(
        brio::usb_configuration_head(9 + Port::descriptor_bytes, Port::interface_count), Port::descriptors);
    static std::span<const uint8_t> string(uint8_t index);   // 0 the languages, then the strings
};
using Device = brio::UsbDevice<Usb, Descriptors, Port>;

using SysClock = brio::Clock<brio::ClockSource::pll, 96'000'000, 8'000'000>;   // the crystal: 2500 ppm
Usb::init(clock);
Device::start();                                             // the pull-up: the host enumerates from here

extern "C" BRIO_CH32_INTERRUPT void usb_lp_can1_rx0_handler() { Device::isr(); }
brio::print(Port{}, "hello", brio::crlf);                    // once the host has configured the port
```

The port is a `ByteTransport`: a `SerialPort` and a console run over it
exactly as over a UART, and the line coding and DTR the host sets are
reported and not obeyed - a virtual port has no baud rate. The identity
1209:0001 is pid.codes' test pair, meant for a device that is not a
product; a product states its own.

A program that also uses the CAN leaves the top 128 bytes alone by
saying nothing, and one that does not may take them:

```cpp
using Usb = brio::Usbd<512>;      // no CAN in this program: the whole memory
```

And a program with a USB does not have to remember not to sleep. It
may call the kernel's idle path like any other program: the attached
controller holds one bus-master count and the platform's `idle()`
returns at once while it does ([sleep.md](sleep.md)). What such a
program gives up is the current a sleep would have saved, not the loop
it is written in.

## Bench findings

Measured on a CH32V203C8T6 with the board's USB-C in a Linux host, the
core on the board's 8 MHz crystal through the PLL at 96 MHz with the
USB fed 48. The reference suite is `test_v203_usb`, whose console is
the probe's UART because the connector is the thing under test and
whose host-assisted letters have a pyserial script at the other end;
beside it two apparatus programs, the console over a CDC ACM port (the
stack and the class above this driver, unchanged from the other
targets' use of them) and `usb_probe`, which carries the same stack
under a UART console and can switch its loop between spinning, WFI and
the platform's WFE idle on command.

- **THE PORT ENUMERATES AND CARRIES BYTES.** The host binds cdc-acm to
  the device, the port answers on /dev/ttyACM with no driver installed,
  the banner is held until the host raises DTR, and the console's
  commands are answered with no packet-memory overflow and no bus
  error.
- **THE PADS ARE PORT A'S.** With port A's clock shut the host still
  sees the device attach - the pull-up is EXTEN's - and then every
  packet fails: the SETUP counted, its first halfword never stored,
  sixteen overflows in one attempt, the host abandoning the enumeration
  with a protocol error.
- **THE SLEEP.** With the core in WFI or in the platform's WFE idle the
  controller loses the host's bytes and an enumeration dies in its
  first control transfer; the same image spinning enumerates,
  configures and carries data with not one overflow. Both idle forms
  fail and a busy-wait works, so it is the core being asleep and not
  the idiom.
- **THE SAME UNDER THE VENDOR'S OWN CODE.** The EVT's SimulateCDC
  example, built with WCH's compiler and unchanged but for one `wfi`
  in its loop, never reaches CONFIGURED: SET_ADDRESS gets through, the
  descriptor reads do not, the overflow counted 18 to 26 times. Its own
  `__WFE()` fails the same; a busy-wait of the same length works; a
  variant that sleeps only once configured loses every one of the 4096
  bytes the host writes; and the loss happens with the core woken at
  least once per USB frame, so it is neither the depth nor the length
  of the sleep that matters.
- **THE MITIGATIONS THAT DO NOT WORK**, each measured on that same
  example: the overflow interrupt unmasked (the core wakes, the packet
  is gone, the host does not retry the burst), ten and a hundred
  milliseconds awake after the overflow, the endpoint held at NAK
  across the sleep, the endpoint re-validated afterwards.
- **THE ONE THAT DOES: A SLOWER BUS.** The device enumerates and
  carries 4096 bytes with HCLK at 96, 48 and 24 MHz and the USB clock
  at 48, and fails at 12 MHz with the sleeping case's own signature.
  That is what makes 24 MHz the compile-time floor.
- **THE DEBUG MODULE'S KEEP-THE-CLOCKS-ALIVE BIT COULD NOT BE TRIED**:
  a `csrw` to CSR 0x7C0 from the running program resets the part, so
  the three bits of 34.2.1 are read here and never written.

- **THE ENUMERATION.** `init()` takes 2 us and leaves the pull-up
  down; from `start()` the host has the device CONFIGURED in 363 to 628
  ms, spending twenty-odd setup packets and stalling the half-dozen
  requests this device has not got (a device qualifier, a string past
  the four). A CDC port with its three endpoints and the control pair
  spends 328 of the 384 bytes the CAN-safe budget leaves, 56 free. The
  host's frame counter advances 9 or 10 in 10 ms, and the whole
  enumeration passes with no bus error and no packet-memory overflow.
- **THE COUNT, AND THE LOOP IT FREES.** With the device attached the
  platform reports one bus master, and a thousand `idle()` calls out of
  the kernel's own idle path cost 0 to 1 ms - a thousand tick periods
  is what they would cost if the count were zero - with the port still
  configured and not one overflow behind them.
- **A RECONNECT.** `stop()` drops the pull-up, the count with it, and
  the device is detached; `start()` half a second later brings three
  bus resets and a fresh enumeration in 363 to 586 ms, with the host
  handing back THE SAME address it had given before.
- **THE PORT AS A BYTE TRANSPORT.** Ten seconds of echo carry 2.87
  million bytes, byte-exact at the host end (chunks of 1 to 300 bytes,
  319 KB/s round trip), with no ring overrun, no bus error and no
  overflow. Pouring one way the device sends 2.30 MB in 3 s (767 KB/s,
  one packet per IN token and no double buffer on this block); draining
  the other it takes 923 kB in 2 s (461 KB/s) with no ring overrun,
  because the OUT endpoint is armed only while a whole packet fits and
  USB's own NAK is the flow control.
- **THE LINE CODING IS REPORTED AND NOT OBEYED.** A host opening the
  port at 19200 7E2 with DTR and RTS up is read back as 19200/7/even/2
  with both lines raised, while this console - a real UART at its own
  115200 - goes on unchanged.
- **THE SUSPEND AND ITS RESUME.** With the host's runtime power
  management armed on the port, the controller reports the bus going
  idle within a second and a half and the wake-up when the host touches
  the port again; the device is configured, its frame counter running
  and its counters clean on the other side. A suspended controller
  still holds its bus-master count, deliberately: the count is the
  attachment's and not the traffic's.
- **THE OVERFLOW, STAGED ON PURPOSE.** Under a host pouring at full
  speed, 50 ms with the core awake carry 22 kB into the port; 50 ms of
  a bare `wfi` - the instruction the platform's idle path would not
  have taken, with the count standing - carry NOTHING and count some
  620 packet-memory overflows. The receive buffer's size halfword reads
  0x8440 after it, which is the driver's repair standing (0x8400 for a
  whole 64-byte buffer, and the 64 of the last count under it), and
  with the core awake again the port takes 95 kB in 200 ms with the
  overflow counter standing still.

## Not covered yet

Driver gaps, each with its reason:

- **The double buffer (EP_KIND on a bulk endpoint) and the isochronous
  endpoints**, and with them the high-priority vector they arrive on. A
  CDC console needs neither, and the double buffer changes what DTOG
  means on every access; it is written when a program needs the
  bandwidth and can measure the difference.
- **Remote wake-up (CNTR.RESUME) and the suspend's low-power half**
  (LPMODE, the regulator, the wake-up line's EXTI 18). The handshake is
  kept as far as FSUSP, which is what the suspend and the wake-up
  events need; what a suspended controller would let the program sleep
  through is a question for a meter, and no program here asks the host
  to be woken.
- **The USBFS host/device controller of chapter 23**, on PB6/PB7. It is
  a different peripheral with its own registers, not a second instance
  of this one, and it is declined until a program names it as its first
  user - a board that wires that connector, or a host side, which brio
  does not have and will not ([design/usb.md](../design/usb.md)).
- **The 1-wire (single-ended) mode of CNTR**, which the manual gives to
  another family's lot numbers, and the low-speed bit beside the
  pull-up in EXTEN: this stratum is a full-speed device.
- **A second class, or a second port in one device.** The stack routes
  by interface and endpoint and the descriptors are the application's
  to glue, so nothing here is missing; it is born with the first
  program that wants two ttyACM or a class that is not a serial port.

Implemented but not bench-verified, each with what would measure it:

- **The endpoint halt.** `stall()` and `stalled()` are exercised on
  endpoint zero by every enumeration that answers a request the device
  has not got; a host that halts and clears a BULK endpoint is what
  would measure them there.
- **A budget other than the default.** `Usbd<512>` and `Usbd<128>`
  compile and the refusal above 512 is a compile error; what would
  measure the larger one is an endpoint set that asks for more than the
  384 bytes a CDC port leaves room in.
- **The eight parts other than the CH32V203C8.** The block is a part
  fact only in its presence (`device::has_usbd`), the whole stratum
  compiles for all nine both ways the hardware prologue can be built
  (`brio check ch32v203`), and the suite links whole for the 32 KB tier
  (21572 bytes of the 28672 such a part leaves the linker). What would
  measure them is a board.
- **WHICH endpoint overflowed.** The status register does not say, so
  the repair writes the size back for every armed receiver - three of
  them on a CDC device, the control pair and the bulk OUT. What would
  measure that walk where it costs something is a device with several
  classes armed at once.
