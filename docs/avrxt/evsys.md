# EVSYS - the event system (AVR DA, DB and EA)

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (EVSYS
chapter 16, PORTMUX 17.5.1), errata DS80000915F (no EVSYS items); for
the AVR64EA28/32/48 DS40002443A (EVSYS chapter 16, PORT.EVGENCTRLA
18.5.17, RTC.PITEVGENCTRLA 24.13.17, EVSYSROUTEA 17.5) and DS80001048D
(no EVSYS, PORT or RTC item); and,
for the DA parts, DS80000882C (2.6.1: PB6/PB7 and PE4..PE7 are not
connected to the event system, as generators or as EVOUT - pins a DA
bonds on 64 pins only; `EvPin` and `EvOut` refuse them there, the
negative tests `evsys_no_pe4_event_on_da` and `evsys_no_evoutb_alt_on_da`).
Driver: `avrxt/evsys.hpp`. Reference test: `test_avr_timer` (TCB capture
and count by event, CCL LUT routing, AC OUT as generator).

"Event" on this page is the HARDWARE event of the data sheet - a
signal on an internal wire - not the kernel's queued value; the bridge
between the two is always an ISR body that posts.

## What the silicon does

Hardware routing between peripherals: a change in one peripheral (the
generator) travels on a channel to any number of users, without the
CPU, in every sleep mode. Each channel is driven by ONE generator
(`CHANNELn` = a code from 16.5.2, `0x00` = off) and listened to by any
number of users (`USERxxx` = n + 1, `0` = disconnected, 16.5.3). Both
are ordinary registers, rewritable at any time: a channel can be
re-sourced and a user connected or disconnected while running.

**Channels.** 10 (`CHANNEL0..9`) on 48/64-pin parts; 8 on 28/32-pin
parts, which also have no `SWEVENTB` register. Every channel has two
subchannels: asynchronous (the generator's signal as is) and
synchronous (the same, synchronized to CLK_PER, two to three cycles
later); which one a user sees is a fixed property of the user and the
EVSYS picks it by itself (16.3.2.5).

**Generators** (16.5.2). Grouped; pulse vs level and sync vs async are
per generator:

- RTC: `OVF`, `CMP` (pulses, CLK_RTC); `PIT_DIV8192/4096/2048/1024` on
  EVEN channels only, `PIT_DIV512/256/128/64` on ODD channels only.
  The two families share codes 0x08-0x0B: the parity of the channel
  picks the family. These are levels - the prescaled RTC clock
  divided, a free square wave with no CPU;
- PORT pins: the pin LEVEL (not an edge; the user's edge detection
  makes the edge), zero if the pin's input driver is disabled.
  PORTA/PORTB only on channels 0-1, PORTC/PORTD on 2-3, PORTE/PORTF
  on 4-5, PORTG on 6-7 (64-pin parts). The two ports of a pair share
  the code space: 0x40+n for the first, 0x48+n for the second;
- peripherals, all on all channels: `CCL LUTn` out (level, async),
  `ACn OUT` (level, async), `ADC0 RESRDY` (pulse, sync), `TCAn
  OVF_LUNF/HUNF/CMPn` (sync), `TCBn CAPT/OVF` (sync), `TCD0
  CMPBCLR/CMPASET/CMPBSET/PROGEV` (async), `ZCDn OUT` (level, async),
  `USARTn XCK` and `SPIn SCK` (levels, sync), `UPDI SYNCH` (sync),
  and on the DB family only `MVIO VDDIO2OK` (async) and `OPAMPn
  READY` (sync). The codes shared by DA and DB are identical;
- software: writing bit n of `SWEVENTA` (`SWEVENTB` for channels 8-9)
  INVERTS the channel's signal for one CLK_PER cycle (16.3.2.6) - a
  high pulse on an idle channel, a low pulse on a channel a level
  generator holds high. Needs CLK_PER: not in standby.

**Users** (16.5.3, table 16-4). What a user DOES with the event is
configured in the user's own peripheral (TCB CAPT means capture,
restart, single-shot... per its mode; TCA CNTA counts on edge or
level or takes direction from the level): the EVSYS only connects.
Detection (edge/level/none) and sync/async are per user: async users
(ADC start, EVOUT, CCL, TCD, TCB CAPT in some modes) respond in
standby without a clock; sync users (TCA, TCB COUNT, USART IRDA) need
CLK_PER. The full user map is 54 registers (0x00-0x35); smaller
packages lack the instances they do not bond out.

**EVOUT.** One user per port: the channel's signal appears on a pin,
`Px2` in the default position on every port present, `Px7` (ALT1 via
`PORTMUX.EVSYSROUTEA`, 17.5.1) on PORTA/B/C/D/E/G where that pin
exists; EVOUTF has no ALT1 on any package. Any generator becomes a
signal on a pin: the test instrument of this peripheral.

**The AVR EA's model** (DS40002443A 16.4, 16.5.2). Six channels
(`CHANNEL0..5`, one `SWEVENTA`) on every package, and EVERY generator
legal on every channel: no parity, no port pair. Pins and PIT dividers
reach the channels through SLOTS - two per port, `PORTx.EVGENCTRLA`
picking which pin each carries (18.5.17, reset PIN0), and two in the RTC,
`RTC.PITEVGENCTRLA` picking which divider each carries, DIV4 .. DIV32768
(code log2(div) - 1, 24.13.17; the divider chain runs only while RTCEN or
PITEN is set, 24.5.2.2). The channel code names the slot: 0x40 + 2 x
port + slot for a pin, 0x08 + slot for the PIT. So at most two pins of
one port and two PIT dividers are routed at once - a contended resource
the DA/DB do not have. ADC0 adds SAMPLE and WINDOW generators (0x25,
0x26). EVOUT: PA2/PA7, PB2, PC2/PC7, PD2/PD7, PE2, PF2/PF7 (17.5) - and
PF7 is the UPDI pin.

## Types and verbs

The model: a GENERATOR is a type carrying its `CHANNELn` code and its
channel legality as constexpr facts; a USER is a type that knows its
`USERxxx` register; a CHANNEL is a resource handle. The compiler
checks LEGALITY (this generator on this channel, this pin as an
EVOUT); it does not check EXCLUSIVITY of a channel several states
rewire - that is ownership, the business of one AO's FSM. A generator
or user missing from the vocabulary is added here, three lines, never
worked around with a raw register write.

| Type | Meaning |
|------|---------|
| `EventChannel<n>` | channel handle (existence checked): `source(G{})` routes generator G (legality static_asserted), `off()` idles the channel, `pulse()` fires a software event |
| `EvPitDiv<div, slot = 0>` | prescaled RTC clock / div. AVR DA/DB: 1024..8192 legal on even channels, 64..512 on odd, slot 0 only. AVR EA: a PIT event SLOT (0 or 1) carrying any divider 4..32768, on any channel; `source()` first calls its `prepare()`, a read-modify-write of the slot's nibble in RTC.PITEVGENCTRLA; `release()` turns the slot off |
| `EvRtcOvf`, `EvRtcCmp` | RTC overflow / compare match, all channels |
| `EvPin<Pin, slot = 0>` | the pin's level. AVR DA/DB: legal on the port pair's two channels, slot 0 only. AVR EA: through the port's event SLOT (0 or 1), on any channel; `source()` first calls its `prepare()`, a read-modify-write of the slot's field in PORTx.EVGENCTRLA (`Port<L>::event_selection(slot)` reads it back) |
| `EvAdc0Sample`, `EvAdc0Window` (EA) | ADC0 sample ready and window compare, all channels |
| `EvAdc0Ready` | ADC0 result ready, all channels |
| `EvTcaOvf<n>`, `EvTcaHunf<n>`, `EvTcaCmp<n, ch>` | TCA overflow / high-byte underflow / compare match, all channels; n gated by the package's TCA count (TCA1 on 48/64-pin) |
| `EvTcbCapt<n>`, `EvTcbOvf<n>` | TCB CAPT flag / counter overflow (the 32-bit cascade carry), all channels; n gated by the package's TCB count (TCB4 on 64-pin) |
| `EvLut<n>` | CCL LUT output level, all channels |
| `EvUpdiSynch`, `EvMvioOk` (DB), `EvZcdOut<n>`, `EvOpampReady<n>` (DB), `EvUsartXck<n>`, `EvSpiSck<n>`, `EvTcdCmpBClr/ASet/BSet/ProgEv` (TCD parts) | the rest of the generator table (16.5.2), codes verified against the device header's enums; instance counts gated per package (ZCD0 alone on 28/32 pins, ZCD0..1 on the AVR DA48, ZCD0..2 on the DB48 and the 64-pin parts) |
| `EvAcOut<n>` | comparator output level, all channels |
| `EvOut<Pin>` | the channel's signal on a pin: Px2 default on every port present, Px7 ALT1 on every port but PORTF (AVR DA/DB) or on PORTA, PORTC and PORTD (AVR EA - its PF7 ALT1 is the UPDI pin, refused by name: a GPIO only with UPDIPINCFG = 0, which loses the debug link without a high-voltage programmer); `listen` selects the PORTMUX position, drives the pin as output, connects; `unlisten` tears it all down (pin back to input, PORTMUX back to default) |
| `EvAdc0Start` | ADC0 start on event (the ADC's `start_on()` also sets STARTEI; listening alone arms nothing) |
| `EvTcaCntA<n>`, `EvTcaCntB<n>` | TCA event inputs A and B; the action is EVACTA/EVACTB in the TCA; n gated by the package's TCA count |
| `EvTcbCaptIn<n>`, `EvTcbCountIn<n>` | TCB capture input (the TCB must also set CAPTEI) and count input |
| `EvLutIn<n, 'A'/'B'>` | CCL LUT n event input A or B |
| `EvUsartIrda<n>`, `EvTcdInputA/B` (TCD parts), `EvOpampCtl<n, OpampAction::enable/disable/dump/drive>` (DB) | the rest of the user table (16.5.3); `usart_count` gates the instances per package |
| `EventUserBase` | `listen(brio::EventChannel<n>{})` / `unlisten()` for every user |
| concepts | `EventGenerator` (code, per-channel legality), `SlottedEventGenerator` (a generator with a `prepare()`, called by `source()` BEFORE CHANNELn is written, so a listened channel never carries the slot's previous selection after the route is made), `EventUser` (the USER register, unlisten) |
| helpers | `event_channels` (6, 8 or 10, the device tables' probe of the EVSYS struct), `evsys_pulse(ch)` (software event on a run-time channel - what a driver holding its channel as a value uses) |

The peripheral drivers wrap the users (`Tcb<n>::capture_on`,
`Tca<n>::event_a_on`, `Lut<n>::event_a_on`, `Adc<0>::start_on`): an
application usually names only the channel and the generator.

## How to use it

**A fixed route owned by a state** - Entry routes, Exit disconnects,
exactly like arming a time event:

```cpp
// Entry: ADC paced by the PIT at 512 Hz, mirrored on a pin.
brio::EventChannel<1>::source(brio::EvPitDiv<64>{});     // odd channel: 512 Hz
brio::EvAdc0Start::listen(brio::EventChannel<1>{});
brio::EvOut<brio::Pin<'D', 2>>::listen(brio::EventChannel<1>{});
// Exit:
brio::EvOut<brio::Pin<'D', 2>>::unlisten();
brio::EvAdc0Start::unlisten();
brio::EventChannel<1>::off();
```

**A pin into a timer** (the capture path of the TCB meters):

```cpp
brio::EventChannel<0>::source(brio::EvPin<brio::Pin<'A', 2>>{});  // A/B pair: channels 0-1
brio::EvTcbCaptIn<0>::listen(brio::EventChannel<0>{});            // Tcb<0> mode + CAPTEI arm it
```

**On the AVR EA, the slots named in the type** - the second pin of
PORTA rides slot 1, so it does not move what slot 0 carries:

```cpp
brio::EventChannel<5>::source(brio::EvPin<brio::Pin<'A', 4>, 1>{});   // PA4 via PORTA slot 1
brio::EventChannel<0>::source(brio::EvPitDiv<4, 0>{});                 // CLK_RTC / 4: 8.192 kHz
```

The cost of a slotted route: the slot's read-modify-write (a load, an
AND, an OR, a store: about five cycles) before the channel's store. Two
contexts re-selecting the two slots of one port race on the shared
register: one owner per port's slots, as per channel.

**A software event** (test stimulus, one CLK_PER inversion):

```cpp
brio::EventChannel<3>::pulse();
```

## Bench findings

- **The AVR EA's PIT slots** (`test_avr_rtc` p, AVR64EA48 rev B1, ten
  runs of ten): every divider of 24.13.17 exists - DIV4 through
  DIV32768 through each of the two slots, each period within 0.1 % of
  div / 32768 of the OSC32K second measured just before (the chapter's
  overview, 24.2, still says "64..8192"); both slots run at once, each
  at its own divider, and selecting one leaves the other's nibble. A
  slot needs **PITEN**: with the PIT disabled and the counter enabled -
  CNT counting some 650 in 20 ms - DIV4 carries no edge at all, where
  24.5.2.2 says the shared prescaler runs while either function is
  enabled; with PITEN it runs (165 edges in 20 ms), with both off and
  with the slot OFF it is still. An app that routes a PIT slot keeps
  the PIT enabled (its interrupt may stay off: the Ticker's `pause()`
  keeps PITEN). The pin slot is port.md's (`test_avr_pin` e).

- On the AVR64EA48 (rev B1) every route `test_avr_timer` makes runs on
  the EA's six channels with each generator on the channel the letter
  names (no parity, no port pairs), the second pin of a port through its
  slot 1 (letter t's PD2 beside PD0's slot 0): SPI0's SCK, USART1's XCK
  (200 ticks at 20 MHz), the software event's inversion, EVOUT at PA7
  (its 100 nF read 5 us after the change, twenty runs of twenty), PC7
  and PD7.
- On the AVR128DA48 (rev A7) every event route `test_avr_timer`,
  `test_avr_tcd` and `test_avr_sleep` build runs as on the DB; its 48
  pins bond none of the pins DS80000882C 2.6.1 cuts off. EVOUT's ALT1
  pin PA7, loaded there with the board's 100 nF, reads the held level 5
  us after the change in twelve runs of twelve.
- `EvPitDiv<64>` on an odd channel measures 512 Hz on an EVOUT pin,
  `EvPitDiv<8192>` on an even one 4 Hz: the dividers count CLK_RTC
  cycles (32768 / div), and the RTC counter's own prescaler does not
  move them ([rtc.md](rtc.md)).
- `EvOut::listen` driving the pin as output works; the chapter does
  not spell out a port override for EVOUT, so the driver sets DIR.
- Connect and disconnect at run time behave as ordinary register
  writes: the signal appears and vanishes on the pin.
- Illegal pairings (a PIT divider on the wrong parity, a pin on the
  wrong channel pair, a non-EVOUT pin) are refused at compile time.
- `test_avr_timer` closes its loops through this peripheral
  with no wires: `EvPin` generators feed TCB capture and count users,
  LUT outputs and event inputs route through channels, AC OUT drives
  a channel as a generator.
- `test_avr_timer` e, the generators and positions no other letter
  routes: SPI0's SCK (a pinless host, mode 0) gives exactly 32 rising
  edges for four bytes; USART1's XCK as a synchronous host at 100 kHz
  measures 240 CLK_PER ticks a period through a TCB frequency meter -
  and it runs with NO frame in flight, the host's clock being free
  running; the software event on a channel a pin holds HIGH gives a
  LOW pulse each time (three pulses, three rising edges at their ends,
  the same count as on an idle channel), the 16.3.2.6 inversion; EVOUT
  at the ALT1 positions PA7, PC7 and PD7 follows the channel both ways.
- The IRDA user carries USART2's IRCOM receiver off its pin
  ([usart.md](usart.md), the IREI finding).
- **Asynchronous delivery in standby and power-down**: EVOUT, an
  asynchronous user, drives its pad from a PIT divider with every clock
  but the 32 kHz one stopped, and that pad's edge wakes the device
  ([platform.md](platform.md), the sleep modes); the ADC's event start,
  another, converts in standby. On the AVR64EA48 (rev B1) the PIT's
  event slot drives EVOUT through power-down on PD2, PD7, PA7 and PC7
  and a LUT's input on channel 2 the same way (`test_avr_sleep` e and p,
  ten runs of ten): the EA's event system routes with every clock but
  the 32 kHz one stopped, as 16.3.3 says.

## Not covered yet

Driver gaps:

- on the AVR EA, an allocator of the pin and PIT slots: the slot is in
  the type and the app's to keep unique per port (as a channel is):
  declined, no driver allocates a resource the app did not name;
- EVOUTF's ALT1 on the AVR EA (PF7, the UPDI pin) - declined above;

- pin-level bonding of an EVOUT ALT1 pin within an existing port
  (whether THIS package bonds Px7) - deferred to the device tables;
- the static allocator (`EventSystem<Route...>`: routes declared once,
  channels assigned at compile time) - sugar over the primitives,
  which are the peripheral.

Implemented but not bench-verified:

- the latency from a PITEVGENCTRLA or EVGENCTRLA write to the event
  (no synchronization is documented for either register): a write
  stamped by the TCB cascade against the EVOUT edge it causes would
  measure it;

- the vocabulary no driver exercises yet - the UPDI, MVIO and ZCD
  generators: codes verified against the device header's own enums,
  never routed (the OPAMP's READY generator and its four users, and the
  TCD's generators and both inputs, run in their own suites -
  [opamp.md](opamp.md), [tcd.md](tcd.md));
- the two-to-three cycle latency of the synchronous subchannel
  (documented, never measured).
