# EVSYS - the event system (AVR DA/DB)

> **PROVISIONAL.** The typed vocabulary now covers the chapter's
> full generator and user tables (the instance counts and DB-only
> rows gated by the device header); what remains unverified is the
> vocabulary no driver exercises yet, and the static allocator is
> still sugar-to-be - see "Not covered yet". Documents of record: AVR128DB28/32/48/64 data
> sheet DS40002247B (EVSYS chapter 16, PORTMUX 17.5.1), errata
> DS80000915F (no EVSYS items). Driver: `avrdx/evsys.hpp`. Reference
> test: `test_avr_timer` (TCB capture and count by event, CCL LUT
> routing, AC OUT as generator).

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
| `EvPitDiv<div>` | prescaled RTC clock / div; 1024..8192 legal on even channels, 64..512 on odd |
| `EvRtcOvf`, `EvRtcCmp` | RTC overflow / compare match, all channels |
| `EvPin<Pin>` | the pin's level; legal on the port pair's two channels |
| `EvAdc0Ready` | ADC0 result ready, all channels |
| `EvTcaOvf<n>`, `EvTcaHunf<n>`, `EvTcaCmp<n, ch>` | TCA overflow / high-byte underflow / compare match, all channels |
| `EvTcbCapt<n>`, `EvTcbOvf<n>` | TCB CAPT flag / counter overflow (the 32-bit cascade carry), all channels; n gated by the package's TCB count (TCB4 on 64-pin) |
| `EvLut<n>` | CCL LUT output level, all channels |
| `EvUpdiSynch`, `EvMvioOk` (DB), `EvZcdOut<n>`, `EvOpampReady<n>` (DB), `EvUsartXck<n>`, `EvSpiSck<n>`, `EvTcdCmpBClr/ASet/BSet/ProgEv` (TCD parts) | the rest of the generator table (16.5.2), codes verified against the device header's enums; instance counts gated per package |
| `EvAcOut<n>` | comparator output level, all channels |
| `EvOut<Pin>` | the channel's signal on a pin: Px2 default on every port present, Px7 ALT1 on every port but PORTF; `listen` selects the PORTMUX position, drives the pin as output, connects; `unlisten` tears it all down (pin back to input, PORTMUX back to default) |
| `EvAdc0Start` | ADC0 start on event (the ADC's `start_on()` also sets STARTEI; listening alone arms nothing) |
| `EvTcaCntA<n>`, `EvTcaCntB<n>` | TCA event inputs A and B; the action is EVACTA/EVACTB in the TCA |
| `EvTcbCaptIn<n>`, `EvTcbCountIn<n>` | TCB capture input (the TCB must also set CAPTEI) and count input |
| `EvLutIn<n, 'A'/'B'>` | CCL LUT n event input A or B |
| `EvUsartIrda<n>`, `EvTcdInputA/B` (TCD parts), `EvOpampCtl<n, OpampAction::enable/disable/dump/drive>` (DB) | the rest of the user table (16.5.3); `usart_count` gates the instances per package |
| `EventUserBase` | `listen(brio::EventChannel<n>{})` / `unlisten()` for every user |
| concepts | `EventGenerator` (code, per-channel legality), `EventUser` (the USER register, unlisten) |
| helpers | `event_channels` (8 or 10, from the device header), `evsys_pulse(ch)` (software event on a run-time channel - what a driver holding its channel as a value uses) |

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

**A software event** (test stimulus, one CLK_PER inversion):

```cpp
brio::EventChannel<3>::pulse();
```

## Bench findings

- `EvPitDiv<64>` on an odd channel measures 512 Hz on an EVOUT pin,
  `EvPitDiv<8192>` on an even one 4 Hz (scope; RTC prescaler at 1, as
  `Ticker` leaves it: 32768 / div).
- `EvOut::listen` driving the pin as output works; the chapter does
  not spell out a port override for EVOUT, so the driver sets DIR.
- Connect and disconnect at run time behave as ordinary register
  writes: the signal appears and vanishes on the pin.
- Illegal pairings (a PIT divider on the wrong parity, a pin on the
  wrong channel pair, a non-EVOUT pin) are refused at compile time.
- `test_avr_timer` (82/82) closes its loops through this peripheral
  with no wires: `EvPin` generators feed TCB capture and count users,
  LUT outputs and event inputs route through channels, AC OUT drives
  a channel as a generator.

## Not covered yet

Driver gaps:

- pin-level bonding of an EVOUT ALT1 pin within an existing port
  (whether THIS package bonds Px7) - deferred to the device tables;
- the static allocator (`EventSystem<Route...>`: routes declared once,
  channels assigned at compile time) - sugar over the primitives,
  which are the peripheral.

Implemented but not bench-verified:

- the freshly filled vocabulary (UPDI/MVIO/ZCD/OPAMP/USART/SPI/TCD
  generators, IRDA/TCD/OPAMP users): codes verified against the
  device header's own enums, no driver exercises them yet;
- the software event's inversion-on-level behavior (a low pulse on a
  channel held high);
- asynchronous delivery in standby (an async user with no clock);
- the EVOUT ALT1 positions;
- the two-to-three cycle latency of the synchronous subchannel
  (documented, never measured).
