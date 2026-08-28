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
([eic.md](eic.md)). PORT's event outputs (EVCTRL) are still out of scope
here.

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
- **`Port<letter>`** - the 32-bit mask verbs, and
  `configure_mask`/`function_mask` over WRCONFIG.

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
- The peripheral handoff: PB30/PB31 given to SERCOM5 through function
  D carry a byte-exact 115200 console both ways (see `sercom.md`).

## Not covered yet

Driver gaps (not built):
- PORT event outputs (EVCTRL). (Pin senses and interrupts are not a gap
  here at all: they are the EIC's, ch. 26, and they are built -
  [eic.md](eic.md).)
- A `PinSet` analog (the AVR's cross-port mask type has no user here
  yet) and the per-package pin-bonding tables.

Implemented but not bench-verified:
- Pulls (up and down), `strong_drive`, `configure_mask`/
  `function_mask` over WRCONFIG, `release()` - all family-compiled,
  none observed electrically yet.
