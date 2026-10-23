# PORT - the I/O pins (AVR DA, DB and EA)

This document is for a programmer who needs a pin: an LED, a button, a
row of keys, a pad an analog input reads. `Pin<'A', 2>` names pin PA2 as
a type (the letter and the number are template arguments, fixed when
the program is compiled), and its functions set the direction, drive
and read the level and configure the pin's input stage: the pull-up,
the inversion, the input threshold and the interrupt sense.
`Port<'A'>` works on a whole port at once, among other things the one
interrupt vector every pin of the port shares; `PinSet` handles up to
eight pins on any ports as one bit mask; and `PinRef` is a small
run-time handle for a pin that must travel inside an event.

The smallest use, a button with an interrupt, the first example under
"How to use it":

```cpp
using Button = brio::Pin<'A', 2>;                 // PA2: fully async - wakes from standby
Button::input();
Button::configure({.pull = brio::PinPull::up, .sense = brio::PinSense::falling});
ISR(PORTA_PORT_vect) {
    const uint8_t who = brio::Port<'A'>::take_flags();
    if (who & Button::mask) post<Ui>(Pressed{});
}
```

Line by line: `Button` names the pin (this line configures nothing);
`input()` makes it an input, with no pull; `configure()` writes the
pin's whole control byte, `PINnCTRL`, in one store - the pull-up on and
an interrupt on each falling edge; `ISR(PORTA_PORT_vect)` is avr-libc's
way of writing the interrupt handler of port A's one vector;
`take_flags()` reads which pins of port A raised it and clears exactly
those; and the handler posts an event, `Pressed`, to an active object
`Ui` (`brio::post`; an active object is a component of a brio program
with its own event queue, served one event at a time by the kernel,
[the learning route's introduction](../learn/introduction.md)). The
excerpt is a fragment: `Ui` and `Pressed` are the program's own.

What decides correctness even there:

- **The handler clears the flags it serves.** A pin's flag stays set
  until software writes a one to it, and the interrupt request stays
  active as long as it is set (DS40002247B 18.3.3, 18.5.10): a handler
  that leaves it standing is entered again at once, for ever.
  `take_flags()` clears what it read with a plain store; a
  read-modify-write of `INTFLAGS` (or an `SBI` on it) would read every
  pending flag back as 1 and clear flags the handler never looked at.
- **Every sensed pin of a port raises the same vector.** Several pins
  can fire together, so the handler tests the mask; the program binds
  the vector itself, and the driver binds none.
- **Not every pin wakes the chip from every sleep on every edge.** On
  the AVR DA/DB only Px2 and Px6 are fully asynchronous: a falling edge
  on another pin does not wake STANDBY or POWER-DOWN, where both edges
  or a level held through the wake-up do ("Bench findings" below); on
  the AVR EA every pin is. `Pin::fully_async` says which.
- **The configuring functions are read-modify-write sequences**, except
  `configure()`: `input()`, `pull()`, `invert()`, `sense()` and
  `input_enable()` read the pin's `PINnCTRL`, change one field and write
  it back, so two contexts configuring the same pin at once can lose a
  field. Call them from setup or from an active object's handler.
  `set()`, `clear()`, `toggle()`, `output()` and `read()` are single
  instructions (an `SBI` or a `CBI` on the VPORT alias, read in the
  release listings, or one store or load), safe from any context.

## Documents of record

AVR128DB28/32/48/64 data sheet DS40002247B
(PORT chapter 18, I/O multiplexing chapter 3), errata DS80000915F
(2.9.1 PD0 floating input on 28/32-pin; 2.2.4 VPORT stores after a
store to >= 64 - Pin uses SBI/CBI and PinRef stores into the PORT's
OUTSET/OUTCLR in the extended I/O space, so neither is exposed) and,
for the DA parts, DS80000882C (2.8.1: a pin selected as an analog input
loses its digital input - the ADC and AC drivers disable it themselves,
so nothing reads it digitally meanwhile); for the AVR64EA28/32/48
DS40002443A (PORT chapter 18, I/O multiplexing chapter 3) and
DS80001048D (2.2.3, the twin of the DB's 2.2.4; 3.5, the INLVL
levels). Driver: `avrxt/pin.hpp`. Reference test: `test_avr_pin` (a
letter, below, is one command key of a suite's console and the group of
verdicts it runs; this suite's all-key is `a`).

## What the silicon does

Up to seven ports of up to eight pins, each pin with: direction and
output value (atomic SET/CLR/TGL registers, plus the VPORT aliases in
bit-accessible I/O space - SBI/CBI, single cycle), a synchronized
digital input (`IN`), and a `PINnCTRL` byte holding the pin's whole
personality: `INVEN` (inverts input AND output), `PULLUPEN` (only
while the pin is an input), `INLVL` (the DB and the EA, not the DA:
Schmitt-from-supply or TTL input threshold - VIL < 0.8 V, VIH > 2.0 V,
on the EA for VDD above 2.7 V), and `ISC` - the input/sense
configuration that is BOTH the interrupt sense (both edges / rising /
falling / low level) AND the input buffer switch (`INPUT_DISABLE`:
`IN` freezes, interrupts and events die, the pad keeps feeding the
analog mux - what an ADC/AC input wants: no mid-rail current in the
Schmitt stage, no digital noise).

Facts that matter to code:

- **One interrupt vector per port** (`PORTx_PORT_vect`): any sensed
  pin raises its bit in `PORTx.INTFLAGS`; several pins can fire
  together, so the ISR reads the MASK. The flags are
  write-1-to-clear: a plain store - an RMW (or SBI, which is one)
  reads every pending flag back as 1 and clears the lot. Measured:
  the clear lands one cycle after the store - a back-to-back read
  still sees the old flags.
- **The sense fires on the SYNCHRONIZED input**: an OUTPUT pin whose
  buffer stays on senses its own edges (how the test suite closes its
  loops); `level_low` re-fires for as long as the pin reads low
  (measured: continuously - it is a level, not an event); edges less
  than ~3 CLK_PER apart are lost on partially-async pins (the
  interrupt dead-time).
- **Px2 and Px6 are FULLY asynchronous** on the AVR DA/DB (I/O mux
  note, DS40002247B 18.3.3.1): they wake from every sleep mode on every
  sense, catch sub-CLK_PER pulses and have no dead-time; every other pin
  needs CLK_PER running (or BOTHEDGES/LEVEL held until wake-up
  completes). **On the AVR EA every pin is** (DS40002443A 18.3.3.1): the
  partial/full table does not exist there, and `Pin::fully_async` reads
  the device tables' `all_pins_fully_async`.
- **The AVR EA's pin event slots**: `PORTx.EVGENCTRLA` (18.5.17) holds
  the two pins of the port the event system can carry at once; the
  routes select them (`EvPin<P, slot>`, [evsys.md](evsys.md)) and
  `Port<L>::event_selection(slot)` reads one back.
- **INVEN inverts the sense too** (measured: rising + INVEN counts
  the physical falling edges), and toggling INVEN makes an edge every
  peripheral on the pin sees. The datasheet's three write-hazards
  (18.3.3): INVEN toggled in the same cycle as an ISC change loses
  that edge's interrupt; INLVL must change with the pin's consumers
  quiet; an ISC change while an interrupt is synchronizing can fire a
  spurious one or lose one. The driver's answer is the one-store
  `configure()`.
- **The multi-pin engine** (18.3.2.4): `PINCONFIG` holds a PINnCTRL
  value and is MIRRORED across ports; `PINCTRLUPD/SET/CLR` (per
  port) copy/set/clear it into every pin of a mask in one operation.
- `PORTCTRL.SRL` limits the slew rate of the whole port. The PORT
  keeps running while the CPU is halted in debug.
- Errata 2.9.1: on 28/32-pin DB parts PD0 is not bonded but its input
  buffer exists and floats - constant current draw; the datasheet's
  remedy is INPUT_DISABLE on PD0. A board/device-init fact (one
  store), noted here on purpose and NOT wrapped by the framework: a
  kludge for broken silicon stays visible.

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `PinConfig` | `invert`, `pull` (`PinPull`: none, up - the family has no pull-down, so the enum has no `down`), `input_level` (`PinLevel::schmitt/ttl`, DB only - the field exists only where the silicon has it), `sense` (`PinSense`: none, both, rising, falling, input_disable, level_low) |
| `Pin<'A', 5>` | `output`/`input(pull)`, `set`/`clear`/`toggle`/`read`/`is_output` (VPORT, single cycle), `configure(cfg)` (the whole PINnCTRL in ONE store), the single-field RMW verbs `invert`/`pull`/`sense`, `input_enable(bool)` (the digital input buffer - a code of the ISC field, so it leaves an armed sense at INTDISABLE), `flag`/`clear_flag` (this pin's W1C bit), `pinctrl()`, `ref()` (runtime PinRef), `fully_async` (Px2/Px6 on the DA/DB, every pin on the EA), PwmChannel (max 1) |
| `Port<'C'>` | the port's own registers: `in`/`dir_set`/`dir_clear`/`out_set`/`out_clear`/`out_toggle` (masks), `flags`/`clear_flags(mask)`, `take_flags` (ISR body of PORTx_PORT_vect: the fired mask, cleared), `slew_limit(bool)`/`slew_limit()`, `configure_mask(pins, cfg)` (the multi-pin engine), `event_selection(slot)` (the AVR EA: the pin a PORTx.EVGENCTRLA slot carries), `regs()`/`vregs()` |
| `PinSet<Pins...>` | up to 8 pins on ANY ports as one bit mask: `input(pull)`, `output`, `read`, `write`, `configure(cfg)` (grouped BY PORT at compile time: one mirrored PINCONFIG store plus one PINCTRLUPD per involved port - the set behaves like a single register), `port_mask<'C'>()` |
| `port_by_letter(c)`, `pinctrl_of(c, n)`, `port_exists(c)` | a port named by a letter that is a configuration VALUE (a route's port, a comparator's input): the PORT (or one PINnCTRL), folded to one address when the letter is a constant; a constant letter naming a port this package has not got is a compile error wherever the call is optimized (every release image and the family fixtures; in a build that inlines nothing, the debug one, the letter is not seen as a constant and an absent one falls back to PORTA, the run-time path of a letter the caller has validated); `port_exists` is the device tables' question the drivers ask first |
| `PinRef` | the runtime descriptor (3 bytes): a pin inside a request event; `set`/`clear` are ONE store into the PORT's `OUTSET`/`OUTCLR` through the pointer (1 cycle; atomic against an ISR that moves another pin of the same port, where a read-modify-write of `OUT` would write that pin back), `valid()`; null = no-op |

## How to use it

A button with an interrupt (the classic):

```cpp
using Button = brio::Pin<'A', 2>;                 // PA2: fully async - wakes from standby
Button::input();
Button::configure({.pull = brio::PinPull::up, .sense = brio::PinSense::falling});
ISR(PORTA_PORT_vect) {
    const uint8_t who = brio::Port<'A'>::take_flags();
    if (who & Button::mask) post<Ui>(Pressed{});
}
```

A row of keys on two ports, configured as one register:

```cpp
using Keys = brio::PinSet<brio::Pin<'A', 2>, brio::Pin<'A', 3>,
                          brio::Pin<'C', 6>, brio::Pin<'C', 7>>;
Keys::input();
Keys::configure({.pull = brio::PinPull::up, .sense = brio::PinSense::both});
const uint8_t raw = ~Keys::read() & Keys::mask;   // active-low
```

An analog pin done right (what the ADC/AC drivers do for you):

```cpp
brio::Pin<'D', 1>::configure({.sense = brio::PinSense::input_disable});
```

## Bench findings

`test_avr_pin` (A5, 27 verdicts; the same 27 green on the AVR128DA48,
rev A7, 5 V): every sense counts exactly (10
rising, 10 falling, 20 both, 0 with none, on self-driven edges 2 us
apart); `level_low` is quiet while high, re-fires continuously while
low (245 ISRs in a ~20 us window) and stops on the rising edge;
rising + INVEN counts the physical falling edges; the W1C discipline
holds (clearing one pin's flag leaves the other's) and the clear
lands one cycle after the store; a pulled-up input reads 1;
`input_disable` freezes IN at its last value and the buffer comes
back live; the multi-pin engine writes one setting into three pins on
two ports (PINnCTRL readbacks exact) and both ports interrupt; the
Port mask verbs and the slew-limit bit read back as written. A
`PinRef` drives its pin both ways and a null one touches nothing; and
an ISR's edge on PC7, placed by a `level_low` storm on PD3 at each of
64 points across eight pairs of `PinRef` edges on PC6, is never undone
- the same sweep over a read-modify-write of `OUT` through the pointer
(the shape a VPORT descriptor compiles to) undoes it at 18 of the 64.

**A fully asynchronous pin wakes STANDBY on any edge; a partially
asynchronous one on both edges only** (`test_avr_spi` w, table 18-4).
The edges come from the other board - its SPI host's select on PE3 and
its clock on PE2 - with this chip in STANDBY, CLK_PER stopped and the RTC
PIT the only other wake-up (counted, a backstop 500 ms away): PE2
RISING and PE2 FALLING each end the sleep with PE2's own interrupt, the
backstop silent; PE3 FALLING does NOT - the select really fell, the peer
clocked its whole burst, and the backstop ended the sleep with no pin
interrupt at all; PE3 on BOTH edges wakes it with PE3's interrupt. The
both-edges wake needs the level to outlast the wake-up (the table's
note): the select is held 4.9 ms against the 1.5 ms a standby wake
takes on the crystal ([platform.md](platform.md)). Fourteen runs, no
exception.

**On the AVR64EA48** (rev B1, 20 MHz, `test_avr_pin` from the console,
30 of 30): letters 1 to 9 green as on the DA/DB, 27 verdicts, and letter e
- PD3 through PORTD's event slot 1 to a TCB counting its rises, the slot
read back as pin 3 with slot 0 untouched at PIN0, ten rises counted ten,
the slot moved to a quiet PD5 counting none - three of three in ten runs
of ten. PC6/PC7 drive the jumpered nets to PE2/PE3 with no change to any
verdict.

**On the AVR EA ONE edge of any pin wakes POWER-DOWN** (`test_avr_sleep`
p and q on the AVR64EA48, ten runs of ten; DS40002443A 18.3.3.1). Six
pads driven by the PIT's 32 Hz divider through the event system while
every clock but the 32 kHz one is stopped - EVOUT on PD2, PA7 and PC7, a
LUT's asynchronous output on PA6 and PD3 - each sense their own drive,
and PE3 senses PC7's on the jumpered net: a FALLING sense alone and a
RISING sense alone each end one power-down sleep with the pad's own
interrupt, a 1 s PIT backstop never firing. Four of them (PA7, PC7, PE3,
PD3) sit where the DA/DB's pins are only partially asynchronous. PD6 and
PD7 (EVOUTD's and LUT2's ALT1 pads) are the board's analog net and are
not driven. The partner's edges add three: a start bit's single
falling edge on PE0 and on PC4, and the stimulus on PC6, whose wake out
of power-down costs the crystal's restart as PE2's does
([platform.md](platform.md)).

## Not covered yet

Driver gaps:

- Pin-level bonding within an existing port (which Pxn THIS package
  bonds) - the reserve (`avrxt/device_tables.hpp`) knows the package's
  pin count but carries no per-pin mask yet; born with the first
  package where only the pin count tells two pin sets apart.

Implemented but not bench-verified:

- The INLVL thresholds (needs analog levels) and the slew rate's
  electrical effect (needs a scope) - both are configuration
  read-back only today.
- The fully-async wake of a Px6 position on the DA/DB: Px2's is
  witnessed above (PE2); no wire of the DB's desk reaches a Px6 pad - a
  peer driving one would measure it the same way.
- On the AVR EA: the other half of 18.3.3.1, a pulse shorter than one
  CLK_PER caught - a pulse shorter than 50 ns at 20 MHz, which the
  SAM C21 partner can put on the stimulus net (PE2) as two IOBUS stores
  one 48 MHz cycle apart, its edge counted by the pin's flag - and the
  single-edge wake of the pins no self-made or partner edge reaches on
  the bench board - PA4, PA5, PB0..PB5, PC0, PC1, PC5 and PE1, PD0, PD1,
  PD4, PD5, and the pins its I2C bus, LED and button hold - which a wire
  from an EVOUT pad would measure the same way.
