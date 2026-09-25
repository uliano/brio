# USB host/device controller, device mode (CH32V303, CH32V203)

WCH's own full-speed USB controller - host and device in one block,
USBFS or OTG_FS in the vendor's documents - driven in DEVICE mode as the
endpoint controller of brio's USB stack. It is the CH32V303's one
full-speed controller and the CH32V203's second one, beside the device
controller of [usb.md](usb.md). Documents of record: the
CH32FV2x_V3xRM reference manual V2.3, chapter 23 - 23.1, the global
registers of 23.2.1 and the device registers of 23.2.2; the host
registers of 23.2.3 are read only far enough to confirm they are the
host's - with its class notes (23.2.1.2 gives the 1-wire mode to some
lots of the CH32V30x_D8 among others, 23.2.1.8 and 23.2.1.9 give the OTG
registers to the CH32V305 and CH32V307 alone), 9.4.4's table 9-3 for
the wake-up lines, 3.4.2 for USBPRE and 3.4.6 for the HB gate; the
CH32V303/305/307/317 datasheet V3.5, table 2-1-1 (one "USB (FS) HD" on
each of the four CH32V303) and the pin and alternate-function tables of
chapter 3 for the pads; the CH32V203 datasheet V2.8, table 2-1 (a USBHD
on the F8, the G8, the C6, the C8 and the RB) and its pin tables; and
[../design/usb.md](../design/usb.md) for the contract this driver
realizes. Driver: [brio/ch32vx03/usbfs.hpp](../../brio/ch32vx03/usbfs.hpp)
(`Usbfs`), with the stack
[util/usb/device.hpp](../../brio/util/usb/device.hpp) and the class
[util/usb/cdc.hpp](../../brio/util/usb/cdc.hpp) above it, the clock tree
of [clock.md](clock.md) feeding it and [sleep.md](sleep.md)'s count of
bus masters keeping the core awake for it. Reference suite:
`test_vx03_usbfs`, the console on the probe's UART and the board's own
connector under test. The vendor's voice on this block is the EVT's
USBFS device examples (SimulateCDC among them), read against the manual
and never copied.

## What the silicon does

### WCH's own design, and a different peripheral from the device controller

From the manual (23.2.1, 23.2.2): a register file of BYTES at
0x50000000 on the HB bus - a control register, an interrupt enable, a
device address, a state register, a flag register with a status
register beside it, one receive-length register - then eight endpoint
numbers with a MODE NIBBLE each, packed two to a byte in the order the
chapter gives register by register (1 above 4, 3 above 2, 6 above 5, 7
alone), a buffer address, a transmit length, and TWO RESPONSE BYTES per
number, one answering IN and one answering OUT. The numbers 8 to 15 have
no registers of their own: 23.2.2 maps them onto those of 1 to 7 - 9 onto
1, 8 and 12 onto 4, 10 onto 2, 11 onto 3, 13 onto 5, 14 onto 6, 15 onto 7,
which is what the "(9)" and "(8/12)" in the registers' names say. The
endpoint buffers are not in the block: they are the program's own SRAM,
which the block reaches by DMA. Nothing here is the device controller of
ch. 21 - its registers, its packet memory, its vectors and its pads are
all different - and a part can carry both: the CH32V203C8 has the device
controller on PA11/PA12 and this block on PB6/PB7.

Which parts carry it is the part table's (`device::has_usbfs`), from the
two datasheets: every CH32V303, and the CH32V203F8, G8, C6, C8 and RB;
the F6, the G6, the K6 and the K8 have none, and the type is refused
there at compile time. Its pads are the part table's too
(`device::usbfs_dm_port` and its three siblings): PB6/PB7 on the
CH32V203 - I2C1's default pair as well, and on the TSSOP20 the pins of
the debug port - and PA11/PA12 on the CH32V303, the OTG_FS_DM/DP of the
datasheet's table 3-1, which also names USBFS_DM/DP on PB6/PB7 in a row
shared with the CH32V305 and CH32V307. Measured on the CH32V303VCT6:
the block drives PA11/PA12 - the device enumerates on WCH's evaluation
board's connector P14, wired there, and the block's own D+ bit follows
the pull-up on PA12 while PB7 sits at its board's pull-up.

### The buffers are the program's memory

From the manual (23.2.2, table 23-4, 23.2.2.6): endpoint zero sends and
receives through ONE 64-byte buffer; endpoints 1..7 take a buffer per
direction, and a number with both directions enabled uses one region
with the OUT half first and the IN half 64 bytes above it (the vendor's
sequence loads an IN packet at +64 exactly when the number receives
too). Every buffer address is four-byte aligned, and a receive buffer is
at least min(the largest packet + 2, 64) bytes long, which a 64-byte
half always is.

So the driver owns a POOL, a static array aligned to four, and hands it
out in the order the stack claims endpoints: endpoint zero's 64 bytes
at init(), then 64 bytes a direction. The stack claims one direction at
a time, so the second claim of a number lands beside the first when the
first was the last thing handed out - the IN half moving up by 64 when
it was claimed first - and otherwise both halves move to a fresh 128
bytes. That happens inside a configuration, before anything is armed,
and a claim that would move an ARMED half is refused. The pool's size
is a template parameter because the memory is the program's: 960 bytes
by default (endpoint zero and all seven numbers both ways, so no claim
ever fails for want of memory), and a CDC ACM port spends 256 of them -
endpoint zero's 64, the notification's half, and the bulk pair's two
halves in one piece.

### The buffer register holds an offset

The manual gives R32_UEPn_DMA thirty-two bits. MEASURED, it reads back
the address's LOW BITS alone: 0x5E0 for a buffer at 0x2000_05E0 and 0x660
for one at 0x2000_0660, while the registers nothing wrote read reset
values of up to seventeen bits. The vendor's copy path reads it the same
way, back and plus 0x20000000. So a buffer must lie in the low part of
SRAM: every part's linked SRAM lies in its first 64 KB, the driver
refuses at compile time a part that would link more, and it never reads
the register back - it keeps its own record of where every buffer is.

### One receive length and one status for every endpoint, and the pause that makes that safe

From the manual (23.2.1.5 to 23.2.1.7): the token and the endpoint of
the transfer that completed are one status byte, and the length of the
last reception is one register, for every endpoint - both overwritten by
the next transfer. RB_UC_INT_BUSY (23.2.1.1) is what keeps them
standing: with it set the block answers NAK to every transaction while
the transfer flag is up, so the status and the length describe exactly
one transfer until the handler clears the flag. MEASURED under a host's
pour with the vector held off for 20 ms: with the bit set the status
byte changed ONCE, for the first packet, and then stood; with it clear
it changed 248 to 297 times, each change a packet taken into the buffer
nobody had read, and the stream the host sent came out with jumps. The
driver sets the bit at init(), and its event loop reads the status and
the length first, answers the endpoint, and clears the flag LAST. The
length is filed per endpoint on the spot, which is what `out_data(n)`
returns.

AND A SETUP IS NOT JUDGED BY THAT BYTE'S ENDPOINT FIELD. At a SETUP the
field holds what the last IN or OUT left, or its reset garbage: measured
on the CH32V303VCT6, it read 6 at the first SETUP after a bus reset, and
a driver that took it for the setup's endpoint dropped every SETUP of an
enumeration (the host's descriptor reads timing out one after another).
WCH's own sequence never looks at it for a SETUP. Every SETUP is
endpoint zero's in this driver - and the length register beside it is
not the setup's either: it read 1022 at the last SETUP of an
enumeration, so the eight bytes are copied whatever it says.

### The response is not changed by the silicon

The manual describes no change the block makes to a response field
(23.2.2.8, 23.2.2.9) - only the automatic toggle - and MEASURED: after a
completed OUT the endpoint's register still answers ACK, the pause above
being the only thing that holds the next packet off. WCH's sequence sets
NAK itself after every IN it completes on a data endpoint, while after a
bulk OUT it leaves the response at ACK and points the buffer register at
the next slot of a ring of its own. So the driver sets NAK on every
endpoint it reports a completion for, before the flag is cleared, and a
packet goes out or comes in once per submit, which is the stack's own
contract.

### The data toggle is the driver's, and its only copy is the register

From the manual: T_TOG and R_TOG are the PID an endpoint sends and the
PID it expects, and an automatic toggle would flip them in hardware on
endpoints 1..7 - endpoint zero has none. The driver flips them itself,
on every endpoint the same way and in the register, with no second copy
to fall out of step: DATA0 at a claim and after a cleared halt, DATA1
both ways on endpoint zero at every SETUP (chapter 9's rule), and flipped
at every IN the host took and every OUT whose PID matched. An OUT whose
PID did NOT match (RB_UIS_TOG_OK clear, 23.2.1.6) is a packet the host
is sending again because it never saw the device's acknowledgement.
MEASURED with the mismatch staged under a pour: the block acknowledges
such a packet - the host moved on and did not send it again - the
driver dropped it, the stream showed one jump of one packet, and the
two ends ran on in step. The stack never sees a PID.

### A SETUP is taken whatever endpoint zero's response says

Chapter 9 demands it, the vendor's sequence depends on it, and 23.2.2.9
is silent about it. MEASURED: this driver leaves endpoint zero at NAK in
both directions between control transfers (the two response bytes read
0x02 at rest), and every SETUP of every enumeration landed. At a SETUP
the driver sets both directions of endpoint zero to NAK with DATA1,
which also clears a halt the last request left, and copies the eight
bytes out of the shared buffer at once, because the answer the stack
writes next lands in that same buffer.

### The pull-up is this block's, and it is also the device enable

From the manual (23.2.1.1, table 23-2): the system-control field of
R8_USB_CTRL is one field for both - 00 the device function off and no
pull-up, 1x the device function on with the internal 1.5 kOhm on D+. The
transceiver (RB_UD_PORT_EN) and the release of the host's pull-downs
(RB_UD_PD_DIS) are a second register's. The driver brings the
transceiver up at init() and leaves the field at 00, so the host sees no
device until `connect()` - the stack's `start()` - raises it; measured,
the host enumerated the device every time the field went up, after an
init() that left it down.

AND A DETACHED BLOCK REPORTS NOTHING, which the silicon would otherwise
contradict: with the pull-up down the bus is SE0 - the host's two
pull-downs and nobody's pull-up - and the transceiver, still on, calls
that a bus reset (measured: one within 100 ms of every detach, which put
the stack back in its default state as if a host had attached it). The
driver acknowledges and drops every flag while the field is down.

### The pads need no port clock

From the manual, nothing: the chapter never mentions GPIO, and the
vendor's sequence for this block touches none. MEASURED: with the gate
of the pads' port shut the host enumerated the device - where the
device controller of ch. 21 attaches and then fails every packet so
([usb.md](usb.md)) - and a GPIO input still reads the level on D+ while
the transceiver owns the pad. `init()` opens the port anyway and leaves
the two pads floating inputs, their reset state, so that no GPIO
configuration a program left behind stands between the transceiver and
the wire.

### One clock rate, and a floor under the bus

The controller wants 48 MHz, and the tree's USBPRE divides the PLL by
1, 2 or 3 to make it (3.4.2): only a PLL at 48, 96 or 144 MHz feeds it,
and `init()` refuses at compile time a static tree that cannot, and
answers false under a dynamic clock whose rate in force cannot. The
datasheet's note under its clock tree, and the manual's under figure
3-3, ask the CPU clock to be 48, 96 or 144 MHz whenever USB is used.
MEASURED with HCLK divided under a 96 MHz PLL whose 48 MHz USBPRE takes
whole: the host enumerated the device, and a pour arrived with no jump
and no overflow, at 96, 48, 24 and 12 MHz; at 6 MHz the device still
enumerated every time, but the pour arrived with bytes missing in three
runs of seven - with no overflow flagged; at 1.5 MHz it enumerated never,
with the FIFO overflow counting, and the pour arrived broken. The
driver's floor, `usbfs_min_hclk_hz`, is therefore 12 MHz, refused below
at compile time - half the device controller's, whose floor on the
CH32V203 is 24. The 48 MHz source select of RCC_CFGR2 - bit 31,
USBFSSRC in 3.3.5.6's text, USBHSSRC in 3.4.12's table and OTGFSSRC in
figure 3-2, the clock tree of the CH32V305 and CH32V307, while the
CH32V303's own tree (figure 3-3) draws no select at all - chooses
between the PLL and the high-speed PHY's own PLL, which belongs to the
D8C classes: its reset value is the PLL, which is what every measurement
here ran on, and nothing here writes it.

### The core must not sleep while the controller works

In a sleep of any depth on this family no bus master but the core gets
a cycle ([sleep.md](sleep.md)), and this block's DMA reaches its buffers
over the same matrix. MEASURED on the CH32V303VCT6 with a bare `wfi` -
the instruction the platform's idle path would not execute over this
controller - between packets: the host's OUT packets arrived whole (25
to 32 kB in 50 ms, the awake rate, no jump, no overflow), but THE IN
PACKETS THE BLOCK READ OUT OF RAM CARRIED ZEROS where the program's bytes
should have been - 360 000 to 500 000 breaks in 1.3 to 1.5 MB of a
counting stream, the first of them always a byte followed by 0x00, with
no error at either end - and an enumeration with the core in wfi
between its interrupts got through SET_ADDRESS and stopped at the
descriptors the block had to read. So this controller counts itself a
bus master from the pull-up to the detach
([bus_activity.hpp](../../brio/ch32vx03/bus_activity.hpp)): the
platform's `idle()` does not sleep while it is attached and a sleep site
refuses to arm over it. `connect()` is where the count is taken and
given back.

### Suspend and resume, and the wake-up line

From the manual (23.2.1.5, 23.2.1.4): ONE flag for both events - the bus
went idle, or it woke - with the state register saying which; the
driver reads it after the ten microseconds the vendor's sequence waits,
and measured, that reading told a host's suspend from its resume, the
state register agreeing. The block has no handshake of its own around a
suspend, where the device controller of ch. 21 wants FSUSP set.

Table 9-3 gives this controller's wake-up event to EXTI line 20 on every
class of this stratum, and on the CH32V30x_D8 it names line 18 for
"USBD/USBFSOTG" as well. MEASURED on the CH32V303VCT6, both lines armed:
a host's resume raised line 18 once and line 20 never, and neither fired
at the suspend. So the driver's wake-up line is 18 (vector 58) on that
class and table 9-3's 20 (vector 60) on the CH32V203's.

### One vector, and the host's side left alone

Every event of this block in device mode arrives on one line
(`Irq::usbfs`: 59 on the CH32V203, 83 on the CH32V303). Host mode
(23.2.3) is the same block with other meanings at some of the same
addresses; the driver never selects it. The handler's body is the
stack's, which calls out of line, so on the CH32V303 it opens with the
twenty f-register saves [platform.md](platform.md) prices - twenty `fsw`
in the compiled handler of both images that bind it.

## Types and verbs

`Usbfs<pool_bytes>` is the controller, everything static; `pool_bytes`
is the RAM the program gives the endpoint buffers, a whole number of
64-byte halves with endpoint zero's first, 960 by default
(`usbfs_pool_all`, `pool_size` on the type); `endpoint_count` is 8 -
endpoint zero and the numbers 1..7 - and `max_packet` 64, the
full-speed bulk packet, which is also the half a direction is given;
`wakeup_line` and `wakeup_irq` are the wake-up line and its vector for
the part's class (`wakes_on_line18` says which). `init(clock)` is the
pads, the HB gate, the protocol engine reset and released, every
endpoint parked at NAK, endpoint zero laid out, the interrupts of a
device (a bus reset, a completed transfer, suspend and resume, a FIFO
overflow), the automatic pause, the DMA and the transceiver - with the
pull-up left DOWN, because the host must not see a device before the
program can answer it. `release()` gives all of it back. Both clock
refusals live in `init()`.

The [design/usb.md](../design/usb.md) contract is realized verbatim:
`connect(on)` (the device function and the pull-up in one field, and the
bus-master count), `set_address`, `configure_endpoint(address, type,
max)` (a half of the pool, the mode nibble, the buffer register, the
endpoint at NAK with DATA0), `deconfigure_endpoints`, `stall` /
`stalled`, `submit_in(number, data)` (one packet copied into the
endpoint's buffer and the endpoint armed with the PID its register
holds; an empty span is the zero-length packet), `submit_out(number,
max)`, `out_data(number)` (the buffer the block wrote, with the length
it reported for that transfer), `setup()` and `take_events()`, which
drains the flags in a bounded loop - a bus reset, then a completed
transfer, then suspend or resume, then a FIFO overflow - and stops at
the first transfer on endpoint zero, so the control machine above, which
takes the SETUP before the completions, sees them in the order the bus
made them; with the pull-up down it acknowledges every flag and reports
nothing. `configure_endpoint<address, type, max>()` is the same claim
with the three known at compile time: a number past 7, a packet above 64
bytes, a type other than bulk and interrupt, and a pool that could not
hold the one claim beside endpoint zero's buffer are compile errors
there.

The ISR body is the stack's: `UsbDevice<Usbfs<>, ...>::isr()`, bound to
`usbfs_handler`. The wake-up line is `arm_wakeup(on)` (a rising edge,
the interrupt and its vector) and `wakeup_isr()`, the body for the
line's vector - `usb_wakeup_handler` on the CH32V303,
`usbfs_wakeup_handler` on the CH32V203 - with `wakeups()` counting.

The readbacks and the instruments: `address()`, `pulled_up()`,
`attached()` (the count this controller holds), `suspended()`,
`bus_in_reset()`, `sie_free()` (the state register), `dp_high()` /
`dm_high()` (the levels the block sees on its pads), `flags()` /
`status()` (the flag and status bytes as they stand), `setup_length()`
and `setup_status()` (what the length register and the status byte said
at the last SETUP), `buffer_used()` / `buffer_free()`, `received(n)`
(the bytes filed under endpoint n), and three saturating counters from
`init()`: `errors()` (an IN or OUT completion reported on a number
nothing has claimed), `overruns()` (the FIFO overflow flag) and
`toggle_errors()` (OUT packets dropped for their PID). `in_toggle(n)` /
`out_toggle(n)` read the PID each direction holds, and two verbs exist
for a suite to stage what the driver prevents: `set_out_toggle(n, pid)`,
the lost acknowledgement, and `auto_pause(on)`, the pause. And
`count_frames(on)` / `frames()` / `frame()`: INT_EN's bit 7, which the
manual reserves and the vendor's own header names the device SOF
interrupt, counting the host's frames - off at init(), because it is a
thousand interrupts a second, and the one way this block has to count
them: it keeps no frame number of its own.

`usbfs()` is the register view (`Usbfs::regs()` the same block reached
through the type), `usbfs_mod_slot(n)` where endpoint n's mode nibble
lives, and `usbfs_min_hclk_hz` the bus floor.

## How to use it

A console on the chip's own connector - the stack and the class are
the ones the device controller runs, unchanged:

```cpp
using Usb = brio::Usbfs<>;                                  // 960 bytes: no claim ever refused
using Port = brio::UsbCdcAcm<Usb, P>;                       // interface 0, endpoints 1 and 2
struct Descriptors { /* the device, the configuration, the strings: design/usb.md */ };
using Device = brio::UsbDevice<Usb, Descriptors, Port>;

using SysClock = brio::Clock<brio::ClockSource::pll, 96'000'000, 8'000'000>;   // the crystal: 2500 ppm
Usb::init(clock);
Device::start();                                             // the pull-up: the host enumerates from here

extern "C" BRIO_CH32_INTERRUPT void usbfs_handler() { Device::isr(); }
```

A program that knows its endpoints gives the pool exactly what they
take:

```cpp
using Usb = brio::Usbfs<256>;      // a CDC ACM port's endpoints, and not a byte more
```

The host's resume as a wake-up, on the line the part's class uses:

```cpp
(void)Usb::arm_wakeup(true);
extern "C" BRIO_CH32_INTERRUPT void usb_wakeup_handler() { (void)Usb::wakeup_isr(); }   // CH32V303: line 18
```

And a program that runs on either series takes whichever controller the
part carries. The controller is a type, chosen by a constant; the
VECTOR is a symbol, which only the preprocessor can choose, so the part
table states the one fact it needs for that as a macro too:

```cpp
using Usb = std::conditional_t<brio::device::has_usbd, brio::Usbd<>, brio::Usbfs<>>;
#if BRIO_CH32VX03_HAS_USBD
extern "C" BRIO_CH32_INTERRUPT void usb_lp_can1_rx0_handler() { Device::isr(); }
#else
extern "C" BRIO_CH32_INTERRUPT void usbfs_handler() { Device::isr(); }
#endif
```

A program with a USB does not have to remember not to sleep: the
attached controller holds one bus-master count and the platform's
`idle()` returns at once while it does ([sleep.md](sleep.md)).

## Bench findings

`test_vx03_usbfs` on the CH32V303VCT6 at 96 MHz from the board's
crystal, the board's connector P14 on a Linux host through a hub:
**23 pass, 0 fail** in `z`, and every host-assisted letter green with
the bench's host script at the other end - the host end judging the
bytes in the echo and in the device's pours.

**The first enumeration, and the one change it forced.** As written
against the manual the driver took a SETUP's endpoint from the status
byte, read 6 there at the first SETUP after a bus reset, and dropped
every SETUP of the enumeration: the host's descriptor reads timed out,
the setup packet itself sitting whole in the pool where the DMA had put
it. With every SETUP taken as endpoint zero's the host enumerated the
device - 1209:0001, a CDC ACM port - and the kernel console answered
over it.

**An enumeration** to CONFIGURED in 533 to 565 ms from the pull-up, with
four bus resets, twenty-seven setups and six stalls (a device qualifier
and strings the device has not, stalled by the rules, the next SETUP
arriving each time); 256 of the 960 bytes spent; endpoint zero
answering NAK both ways at rest; no stray completion, no FIFO overflow,
no dropped PID; the bus-master count at one and a thousand `idle()`
calls costing under a millisecond.

**The pads**: D+ low with the pull-up down and high with it up, read
both through the block's own bit and through port A's input register,
D- low throughout, PB7 at its I2C pull-up; and an enumeration in 402 to
434 ms with port A's clock SHUT.

**A reconnect**: no bus reset reported in the 100 ms after the pull-up
went down - after the change that drops what a detached block reports;
before it, one in each - and a fresh enumeration in 362 to 570 ms,
the address the host gave the same.

**The bus floor**: configured in 360 to 500 ms and a pour of 22 to
187 kB in 300 ms with no jump and no overflow at 96, 48, 24 and 12 MHz
of HCLK, over seven runs of the pour; at 6 MHz configured in 419 to 973
ms every time and 11 kB of the pour carried, but with 2, 16 and 34 jumps
in three runs of the seven and no overflow counted; at 1.5 MHz never
configured, 8 to 12 FIFO overflows, and 2450 to 2725 jumps in 2919 to
2975 bytes of the pour.

**The frame counter**: 99 and 100 frames in 100 ms with INT_EN's bit 7
set, and none in the ten milliseconds after it was cleared.

**As a byte transport**: 3 039 909 to 3 055 489 bytes echoed byte-exact
in nine seconds of chunks of 1 to 300 bytes (338 to 340 kB/s round trip)
with no ring overrun; 768 kB/s out of the device, every byte of its
counting stream in order at the host, and 564 to 609 kB/s into it with
no jump;
the line coding read back as 19200 7E2 with DTR and RTS raised; seven
bytes filed under endpoint zero for the one SET_LINE_CODING an open made,
and the bytes filed under the bulk OUT equal to what the class took.

**The toggle**: with the bulk OUT told DATA1 where DATA0 was due, one
packet dropped by its PID, one jump of 64 bytes in the stream, and
120 to 124 kB carried in step in the 200 ms after - the mismatch staged
both ways round, DATA0 told DATA1 and DATA1 told DATA0.

**The pause**: with the vector held off for 20 ms under a pour and the
pause set, the status byte changed once and stood, the endpoint's own
register still answering ACK, and nothing was lost; with it clear, 248
to 297 changes in the same hold, and the stream came out with jumps of
64 and 128 bytes or with packets dropped by their PID.

**The core asleep** (a bare `wfi`, the platform's idle path left
aside): with nobody draining the class's ring the block answered NAK
once it was full - 128 to 192 bytes in 50 ms, nothing lost; with the
ring drained at every wake, 491 to 536 wakes carried 25 792 to 32 186
bytes in 50 ms, the awake rate, with no jump and no overflow; the device's
own pour with the core in wfi between its packets reached the host at
636 to 768 kB/s with 359 902 to 497 931 BREAKS in 1.27 to 1.54 MB, the
first a byte followed by 0x00; and an enumeration with the core in wfi
between its interrupts never configured in two seconds - sixteen to
nineteen setups, stuck at the descriptor reads after SET_ADDRESS.

**Suspend and resume**: the host's autosuspend reported as a suspend,
the state register saying suspended and the bus-master count still
one; the resume reported and the port configured as before; EXTI line
18 fired once, at the resume, and line 20 never.

## Not covered yet

Driver gaps, each with its reason:

- **The host half of the chapter (23.2.3) and OTG's SRP and HNP**: brio
  has no host side and will not ([design/usb.md](../design/usb.md)); the
  OTG registers are, besides, the CH32V305's and CH32V307's alone.
- **The double buffer (RB_UEPn_BUF_MOD, table 23-4) and isochronous
  endpoints** (the "no response" codes, endpoint 3's 1023 bytes): the
  contract admits one packet in flight per endpoint and direction, and no
  class here streams; a streaming class would be the reason to widen it.
- **Endpoints 8..15**, which 23.2.2 maps onto the registers of 1..7: a
  number above 7 would share the resources of the one it maps onto, and
  no class here needs more than seven.
- **Low speed** (RB_UC_LOW_SPEED, RB_UD_LOW_SPEED) **and the 1-wire
  mode** (RB_U_1WIRE_MODE, which 23.2.1.2 gives the CH32V303 on some lots
  only): this stratum is a full-speed device on two wires.
- **The NAK interrupt** (RB_UIE_DEV_NAK): it would fire on every NAK this
  device answers - every poll of an idle IN endpoint - and nothing above
  the packet wants that; it is left disabled.
- **The suspend's low-power half and remote wake-up**: what a suspended
  controller would let the core sleep through is a question for a meter,
  the count is deliberately the attachment's and not the traffic's, and
  no program here asks the host to be woken.
- **The block's line in RCC_AHBRSTR** (bit 12): not pulsed. `init()`
  writes every register it relies on and the chapter's own engine reset
  is the vendor's way back.
- **The two general-purpose bits** (RB_UDA_GP_BIT, RB_UD_GP_BIT): scratch
  a program has RAM for; `set_address()` carries the first through.
- **The state register's RB_UMS_R_FIFO_RDY** (the receive FIFO not
  empty): read by no verb - the transfer flag is what the driver acts on.
- **A dynamic clock that switches while the controller is attached**:
  every switch of this family parks SYSCLK on the HSI with the PLL
  stopped ([clock.md](clock.md)), which stops the USB clock with it; a
  program detaches first. `init()` checks only the rate in force.

Implemented but not bench-verified, each with what would measure it:

- **The CH32V203's five parts with this block**: they build (`brio
  check ch32vx03`), and the CH32V203 board brio is tested on carries its
  one USB connector on the device controller's pads, PA11/PA12. What
  would measure them - the wake-up line table 9-3 gives that class
  among the rest - is a board with a connector on PB6/PB7.
- **A pool that is not the default**: `Usbfs<256>` compiles and a CDC
  port fits it exactly; what would measure the relocation of a number's
  halves is a class that claims an IN half before its OUT half and
  something else between them.
- **The endpoint halt on a bulk endpoint**: a host that halts and clears
  one is what would measure `stall()` and `stalled()` there.
- **What the pour loses at 6 MHz of HCLK, and why no flag says so**: the
  bytes missing there came with no FIFO overflow, and whether they are
  packets the block acknowledged and never wrote or bytes it wrote wrong
  is not told apart by a counting stream of period 256; a stream whose
  every packet carries its own sequence number would tell them apart.
