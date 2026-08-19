# The event system

Hardware routing between peripherals: a generator's state change
travels on a channel to any number of users, without the CPU, in every
sleep mode. On AVR DA/DB it is the EVSYS peripheral (DS40002247B ch.
16); a sibling exists on SAMD (EVSYS), a partial analogue on STM32
(TIM TRGO, EXTI, DMAMUX). This page has two parts: what the silicon
offers (the analysis that decides the shape), and the brio
representation. Nothing here is built yet; the shape is fixed first,
the tables grow with the first users (ADC start, TCB capture).

## What the hardware offers (AVR DA/DB, 48-pin)

**Channels.** 10 (`CHANNEL0..9`), each driven by ONE generator
(`CHANNELn = code`, `0x00` = off) and listened to by ANY number of
users (`USERxxx = n + 1`, `0` = disconnected). Every channel has two
subchannels: asynchronous (the generator's signal as is) and
synchronous (the same, synchronised to CLK_PER, 2-3 cycles later);
which one a user sees is a fixed property of the user, and the EVSYS
picks it - nothing to configure.

**Generators** (16.5.2, ~50 codes). Grouped:
- RTC: `OVF`, `CMP` (pulses, CLK_RTC); `PIT_DIV8192/4096/2048/1024`
  on EVEN channels only, `PIT_DIV512/256/128/64` on ODD channels only
  (levels: the prescaled RTC clock divided - a free square wave with
  no CPU, the natural pacer for ADC sampling);
- PORT pins: `PORTx.PINn` = the pin LEVEL (not an edge; the user's
  edge detection makes the edge). PORTA/PORTB only on channels 0-1,
  PORTC/PORTD on 2-3, PORTE/PORTF on 4-5 (PORTG 6-7, absent on 48
  pins). Zero if the pin's input driver is disabled;
- peripherals: `CCL LUTn` out (level), `ACn OUT` (level), `ZCDn OUT`,
  `ADC0 RESRDY` (pulse), `OPAMPn READY`, `USARTn XCK`, `SPIn SCK`
  (levels), `TCAn OVF/HUNF/CMPn` (pulses), `TCBn CAPT/OVF` (pulses),
  `TCD0 CMPBCLR/CMPASET/CMPBSET/PROGEV`, `MVIO VDDIO2OK`, `UPDI SYNCH`
  - all on all channels;
- software: `SWEVENTA/B` bit n = one-CLK_PER pulse on channel n
  (needs CLK_PER: not in standby).
Pulse vs level and sync vs async are per generator (table 16-2); a
level generator into an edge-detecting user gives one event per
transition.

**Users** (16.5.3, 54 registers on the 48-pin: `USERCCLLUT0A..5B`,
`USERADC0START`, `USEREVSYSEVOUTA..F`, `USERUSARTnIRDA`,
`USERTCAnCNTA/CNTB`, `USERTCBnCAPT/COUNT`, `USERTCD0INPUTA/B`,
`USEROPAMPn ENABLE/DISABLE/DUMP/DRIVE`). What a user DOES with the
event is configured in the user's own peripheral (TCB CAPT means
capture, restart, single-shot... per its mode; TCA CNTA count on edge
or level, direction control...): the EVSYS only connects. Detection
(edge/level) and sync/async are per user (table 16-4): async users
(ADC start, EVOUT, CCL, TCD, TCB CAPT in some modes) work in standby
with no clock; sync users (TCA, TCB COUNT, USART IRDA) need CLK_PER.

**EVOUT.** Six pin outputs (`EVOUTA..F`), each a user like any other:
the channel's signal appears on the pin (PA2/PA7, PB2, PC2/PC7,
PD2/PD7, PE2, PF2 - default/alt via `PORTMUX.EVSYSROUTEA`). Any
generator becomes a signal on a pin: THE test instrument of this
peripheral - a logic analyzer on an EVOUT verifies a generator, a
channel constraint, a connect/disconnect, with no other peripheral
involved. (The pin must be driven as output; to be confirmed on the
bench - the chapter does not spell out the override.)

**Facts that shape the design.**
- Routing is register writes; the response is "short and
  predictable" (a few CLK_PER for sync users, none for async ones);
  it works in idle and standby.
- Configuration is not only initial: `CHANNELn` and `USERxxx` are
  ordinary registers, rewritable at any time - a channel can be
  re-sourced, a user connected and disconnected while running. Some
  designs use this deliberately (route the ADC start to the PIT in
  one state, to a pin edge in another; silence a user by
  disconnecting it).
- The two legality constraints (PIT dividers by channel parity, pins
  by channel pair) are compile-time knowable facts of the device.
- Errata DS80000915F (2025, silicon A4/A5/B0): no EVSYS items.

## The brio representation

Following the position taken in [overview.md](overview.md) ("hardware
routing is behaviour, not only config"): typed vocabulary, run-time
primitives, static allocation as sugar. Names below are proposals.

**Vocabulary as types** (`avrdx/evsys.hpp`, growing on demand):
- a **generator** is a type carrying its code and its channel legality
  as constexpr facts - `EvPitDiv<64>` (`code = 0x0B`, legal on odd
  channels), `EvPin<Pin<'A', 2>>` (`0x42`, channels 0-1),
  `EvRtcOvf`, `EvAdc0Ready`, `EvTca0Ovf`, `EvTcb0Capt`... - satisfying
  an `EventGenerator` concept (`code`, `legal_on(ch)`);
- a **user** is a type carrying the index of its USER register -
  `EvOut<Pin<'D', 2>>` (also knows its PORTMUX bit and pin), `EvAdc0Start`,
  `EvTcb0Capt`, `EvTca0CountA`... - satisfying an `EventUser` concept;
- a **channel** is a resource handle, `EventChannel<n>` (0..9,
  existence checked).
The tables are transcribed only for what a step needs (the PIT
dividers, the pins, EVOUT, ADC start, then TCB/TCA as their tasks
arrive); a generator or user missing from the table is added, three
lines, when a driver or an app needs it - never worked around with a
raw register write.

**Run-time primitives** (the layer that IS the peripheral):
- `EventChannel<n>::source(EvPitDiv<64>{})` - `static_assert` legality
  of that generator on that channel, write `CHANNELn`; `off()`; 
  `pulse()` (software event, needs CLK_PER);
- `EvAdc0Start::listen(EventChannel<3>{})` - write `USERADC0START =
  4`; `EvAdc0Start::unlisten()`; for `EvOut<Pin>` `listen` also sets
  the PORTMUX position and drives the pin as output.
Each is one register write, callable from any handler: a rewire is an
action of a state (Entry routes, Exit disconnects), exactly like
arming a time event; the fact that it is a run-time operation is what
lets an FSM own a piece of hardware routing.

**Static allocation as sugar** (`EventSystem<Route...>`): a route is a
generator plus its users, `EventRoute<EvPitDiv<64>, EvAdc0Start,
EvOut<Pin<'D', 2>>>`; `EventSystem<R1, R2, ...>` assigns channels at
compile time (lowest legal free channel per route, `static_assert`
when the constraints cannot be met - two odd-only routes plus a pin
route on channels 0-1 is a real puzzle the compiler solves or refuses),
`init()` calls the primitives for every route, `R1::channel` is a
constexpr the app can name (to `pulse()` it, to hand it to a task).
Sugar: it does nothing the primitives cannot; it exists so the routes
that never change are declared once, in one place, checked as a whole.

**What stays the app's (or the owning AO's) responsibility.**
Contention for a channel rewired at run time: the compiler checks
legality, not exclusivity - a channel that changes with state belongs
to one AO's FSM, and that FSM is where "who drives it when" is
decided (ownership, as with borrowed buffers; no locks). Sync/async is
a fact to know, not a constraint to check: the EVSYS synchronises for
sync users by itself; the 2-3 cycle latency is documented.

**What the users' peripherals will do with it.** The EVSYS driver
connects; the meaning of the event on the user side is configured by
that peripheral's task (`Adc` start-on-event, a `PeriodMeter<Tcb<0>>`
capturing on event, `TcaPwm` counting on event...). Task types take
an `EventChannel<n>` (or a route) as a parameter when they listen -
the same "named resource" idiom.

## Verification path

`events0` (bench app, next step): route generators to EVOUT pins and
watch with the logic analyzer - `EvPitDiv<64>` on an odd channel
(the "prescaled RTC clock" is CLK_RTC through RTC.CTRLA's prescaler,
which Ticker leaves at 1: 32768 / 64 = 512 Hz square wave on the pin,
`EvPitDiv<8192>` = 4 Hz - both trivially measured), `EvPitDiv<8192>`
on an even one, a button pin on channel 0, a
software `pulse()` from a console command; connect/disconnect from
an FSM's Entry/Exit and see the signal appear and vanish; the compile
errors for an illegal channel. First real user: ADC start from the PIT
(the analog step). Then TCB capture (the timer step).

## Cross-target note

SAMD's EVSYS maps onto this shape one to one (channels, generators
with per-channel constraints, users; pin output through the CCL or a
timer, to check). STM32 has no channel matrix: the
"route" idea survives as typed connect/disconnect on the specific
trigger muxes (TIM TRGO -> ADC EXTSEL, DMAMUX request generator);
the primitives are the portable part, `EventSystem` allocation is
AVR/SAMD-specific sugar. Which is why the primitives are the layer
that IS the peripheral, and the static allocation is only sugar.
