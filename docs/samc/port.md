# PORT (SAM C21)

> **PROVISIONAL.** Direction, value, pulls, drive strength, the
> peripheral-function handoff and the multi-pin engine are
> implemented; what a pin CANNOT do here by design - sense edges and
> raise interrupts - belongs to the EIC, a separate peripheral with its
> own driver ([eic.md](eic.md)). The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M, PORT ch. 28
(the EIC, for contrast, is ch. 26). Driver: `samc/pin.hpp`. The
family fixture is `test/family_samc/pin.cpp` plus the two port-C
negatives under `tools/check_samc.sh`.

## What the silicon does

**Two groups, everywhere.** The PORT peripheral has exactly two
32-pin groups (A and B) on EVERY variant of this family - the device
header's `PORT_GROUPS` says 2 with no per-variant override - so a
group-C request is refused on all of them, not on small packages.
Which PINS of an existing group are bonded on a given package is a
finer, per-package question this driver leaves open, exactly as the
AVR side does.

**The input buffer is OFF by default.** `PINCFG.INEN` gates it, and
`IN` reads 0 for a pin whose buffer is disabled - including a pin
this program is driving. The driver therefore raises INEN in both
`input()` and `output()`, so `read()` means the same thing here as on
every other brio target; a pin whose microamps matter is parked with
`configure({})`, which clears the whole PINCFG.

**The pull has a direction, and it lives in OUT.** `PINCFG.PULLEN`
turns the resistor on; whether it pulls up or down is the pin's OUT
bit while the pin is an input (28.6.3.2). The driver speaks one
`PinPull` enum and keeps the two registers agreeing - a bool could
not.

**The single-bit registers make pin operations race-free.**
DIRSET/DIRCLR/DIRTGL and OUTSET/OUTCLR/OUTTGL are write-only masks:
every Pin verb is one plain store, nothing to guard against a handler
touching another pin of the same group. The per-pin PMUX nibble is
the exception (two pins share a byte, so `function()` is a
read-modify-write - configuration-time work by nature).

**WRCONFIG is the multi-pin engine** (28.8.11): one store writes the
same PINCFG (and optionally the same PMUX function) into every pin of
a 16-bit half-group mask; a mask spanning both halves costs the two
stores it must. The driver's static_asserts pin the bit
correspondence that makes the engine's upper half a shifted copy of
PINCFG.

**No senses, no flags.** Edge/level detection and pin interrupts are
the EIC's (ch. 26), reached through PMUX function A - a different
peripheral with its own clocking and its own driver, `samc/eic.hpp`
([eic.md](eic.md)).

**But the PORT is an event USER** (EVCTRL, 28.6.4): four event inputs
per group, each naming one pin of that group and one of four actions -
SET, CLR and TGL act on the pin's OUT REGISTER, while OUT makes "the
output pin follow the event input signal, INDEPENDENTLY OF THE OUT
REGISTER VALUE" (28.6.5). Those are two different places and the
difference is everything: under PMUXEN the OUT register is still the
internal pull's direction while the output driver belongs to the
peripheral. 28.6.4 also says only the OUT action survives a standby -
SET, CLR and TGL want up to three clock cycles the PORT does not have
there. All three sentences are measured, in "Bench findings".

**And PMUXEN takes the pad away from the output driver**, measured while
building that driver: with the mux selecting an input-only function,
writing DIR and OUT moves nothing at the pad, while PINCFG.PULLEN and the
OUT bit that gives the pull its direction keep working. 28.6.1's
"override the connection between the PORT and that I/O pin" reaches the
driver and not the pull - which is what makes a pad-driven bench test
possible with no wire at all (see [eic.md](eic.md)).

## Types and verbs

- **`port_exists(letter)`** - the group-count fact, read from the
  device header, no letter list to go stale.
- **`Pin<letter, n>`** - output/input(pull)/set/clear/toggle/read/
  is_output; `configure(PinConfig)` as one PINCFG store with the pad
  under PORT (PMUXEN clear); `function(PinFunction, PinConfig)` hands
  the pad to a peripheral (the PMUX nibble plus PMUXEN, INEN still
  the caller's to ask for - the mux does not turn the buffer on);
  `release()` takes it back; single-field verbs for INEN, the pull
  and DRVSTR. A Pin is also the degenerate `PwmChannel` (max = 1) and
  a `PinRef` factory - the runtime descriptor whose set/clear are
  OUTSET/OUTCLR stores, safe from any context.
- **`PinConfig`** - input_enable, `PinPull` {none, up, down},
  strong_drive. Deliberately NOT here: PMUXEN, which is decided by
  WHICH verb runs, never by a flag that could contradict it.
- **`PinFunction`** - the PMUX codes A..I. Which function a given pad
  offers is the package's I/O multiplexing table - the device
  header's `MUX_P<pin><fn>_<peripheral>` constants let an application
  static_assert its own claim.
- **`Port<letter>`** - the 32-bit mask verbs,
  `configure_mask`/`function_mask` over WRCONFIG, and the group's four
  EVENT INPUTS: `event_input_count`, `event_user(m)` (the EVSYS user
  index this peripheral publishes - the four PORT users are the
  peripheral's and not the group's, and which group an input acts on is
  decided by which group's EVCTRL enables it), `configure_event(m,
  PortEventConfig)` with its compile-time twin, `event_config(m)`,
  `evctrl()` and `release_events()`. Table 29-3 marks all four users
  ASYNCHRONOUS PATH ONLY; that is an obligation on the caller's
  `EventChannelConfig`, which this file does not see.
- **`PortEventConfig`** - the pin the input addresses (PIDm), the
  `PortEventAction` it takes (EVACTm: `out`, `set`, `clear`, `toggle`)
  and whether it is listening (PORTEIm). `port_event_config_valid()` is
  the five-bit pin field, and nothing else in the register can be
  wrong.

## How to use it

```cpp
using Led = brio::Pin<'B', 23>;
Led::output();
Led::toggle();

using Button = brio::Pin<'B', 22>;
Button::input(brio::PinPull::up);

// hand a pad to a peripheral (a SERCOM input still needs INEN):
brio::Pin<'B', 31>::function(brio::PinFunction::d, {.input_enable = true});
```

## Bench findings

- Output drive and toggle: the board's LED on PB23, from the raw
  bring-up probe through the kernel heartbeat, at every firmware
  since.
- **The four event actions, separated.** A square wave carried to
  PORT event input 0 over an asynchronous channel, with the same pad
  read back: EVACT = OUT moves the pad whether it is PORT's own output
  or handed to the EIC under PMUXEN, and EVACT = TGL moves it under
  PMUXEN too - at HALF the rate, because one toggle of the OUT bit is a
  whole pad period. So OUT reaches the pad and TGL reaches the OUT
  register that is the pull's direction, exactly as 28.6.5 separates
  them.
- **28.6.4's standby note, measured.** Across one standby of a hundred
  wave periods the OUT action kept driving the pad (a hundred EXTINT
  detections of it) and the TGL action produced NONE. "In Standby mode,
  only the Out action is possible" holds.
- **This is what makes a pad move while the CPU is stopped**, which is
  the one thing the pull-walking of [eic.md](eic.md) cannot do (its
  every step is a CPU store). The chain is in
  [platform.md](platform.md), "Sleep, peripheral by peripheral".
- The peripheral handoff: PB30/PB31 given to SERCOM5 through function
  D carry a byte-exact 115200 console both ways (see `sercom.md`).

## Not covered yet

Driver gaps (not built):
- A `PinSet` analog (the AVR's cross-port mask type has no user here
  yet) and the per-package pin-bonding tables.
- **CTRL's continuous input sampling** (28.8.1), which 28.6.5 requires
  for a pin read through the IOBUS. Nothing here reads a pin that way.
- **The IOBUS window itself.** Reached in [dsu.md](dsu.md)'s campaign
  for erratum 1.13.3 and found NOT to be a plain mirror (DIR and OUT
  read through it, IN and CTRL read zero); this driver uses the APB
  addresses only.

Implemented but not bench-verified:
- Pulls (up and down), `strong_drive`, `configure_mask`/
  `function_mask` over WRCONFIG, `release()` - all family-compiled,
  none observed electrically yet.
- **Event inputs 1..3 and the SET / CLEAR actions**: input 0 carries the
  OUT and TGL measurements and input 1 the AC's stimulus, both with
  EVACT = OUT; SET and CLR are written, read back and never watched.
  Nor has one pin been addressed by several inputs at once, which is
  where table 28-3's priority rule would show.
