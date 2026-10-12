# Pins (CH32X035)

GPIO and the alternate-function block of the CH32X035 series: pins as
types over ports of up to twenty-four pins, the pad's four bits in three
configuration registers, the pulls, the bonding and the pads a package
shorts together, the one-way lock - and AFIO, where a peripheral's
signals move as a COLUMN, the debug port's two pads are given up, and
each external-interrupt line is told its port.

Documents of record: the CH32X035 reference manual V1.8 (the opening of
ch. 8 for the pulls, 8.2 for the functions and the lock, 8.3.1 for the
GPIO registers, 8.3.2 for AFIO) and the CH32X035/X033 datasheet V1.7
(the pinout figures of 2.1 and table 2-1 per package, with its notes 4
to 7 on the shorted pins; the alternate-function tables for the remap
columns). WCH's EVT peripheral library is the source of the CFGHR copy
below, read and not copied. The headers are
[brio/ch32x035/pin.hpp](../../brio/ch32x035/pin.hpp) and
[brio/ch32x035/afio.hpp](../../brio/ch32x035/afio.hpp); the reference
suite is `test_x035_pin`.

## What the silicon does

### The pad

- **Four bits a pin, and MODE carries no speed** (8.3.1.1). MODE 00 is
  an input, and 01, 10 and 11 are all "output mode"; CNF under an input
  is 00 analog, 01 floating, 10 pulled (11 reserved), and under an
  output 00 push-pull and 10 alternate push-pull - "I2C automatic open
  drain". There is NO open-drain output code: a pin drives push-pull or
  hands itself to a peripheral, and an I2C pad turns open-drain by
  itself. Every configuration register resets to 0x44444444, eight
  floating inputs.
- **Twenty-four pins a port, three configuration registers**: CFGLR
  for pins 0..7, CFGHR for 8..15 and CFGXR for 16..23 (8.3.1.1,
  8.3.1.2, 8.3.1.8). INDR, OUTDR and BCR are twenty-four bits wide; BSHR
  sets pins 0..15 in its low half and resets them in its high half, and
  BSXR does the same for 16..23 in its two low bytes, a set winning over
  a reset in the same store (8.3.1.5, 8.3.1.9). Every register is
  accessed as a word. On the die the ports are PA0..PA23, PB0..PB21 and
  PC0..PC7, PC10, PC11, PC14..PC19.
- **WCH's library does not read CFGHR on some dies.** On a die whose
  chip-identifier word at 0x1FFFF704 has zero in bits 7:4, its GPIO
  initialization keeps a copy of each port's CFGHR in RAM, starting from
  the reset value, and writes the register WHOLE from the copy - never a
  read-modify-write. The reference manual says nothing of it. The
  CH32X035F8U6 has 1 there, and its CFGHR reads back what is written
  (measured).
- **The pull is the output register's.** An input with CNF 10 is
  pulled, and that pin's OUTDR bit says which way: 1 up, 0 down. Every
  pad has the pull-up; ONLY PA0..PA15, PC16 and PC17 have the pull-down
  (the opening of ch. 8). PC14..PC17 have more pull-up strengths, set in
  the USB PD and USB blocks' own registers - AFIO_CTLR for PC16 and PC17
  (8.3.2.4).
- **An analog input reads zero** in INDR (8.2.9), its Schmitt trigger
  off. PA0..PA7, PB0, PB1 and PC0..PC3 carry the ADC's channels 0 to 13.
- **A package bonds some pads, and on six parts SHORTS some together.**
  Table 2-1's notes 4 to 7: on every part but the CH32X035F8U6, PC16
  and PC11 share a package pin and so do PC17 and PC10; on the two
  28-pin parts PB1 and PB5; on the QSOP28 PA12 and PC14, PA13 and PC15;
  on the CH32X033F8P6 PA7 and PB0 - and "both IOs are prohibited from
  being configured as output functions".
- **The lock is a one-way door** (8.2.5, 8.3.1.7): LCKR's key sequence -
  LCKK at bit 24 written 1, 0, 1 with the pins' mask below it - freezes
  those pins' configuration until the next reset, and two reads of LCKK
  confirm it took.

### AFIO

- **A remap is a column, not a pin** (8.3.2.1). AFIO_PCFR1 holds one
  field per peripheral - SPI1, I2C1, USART1 to USART4, TIM1 to TIM3 and
  the PIOC - and a field's value moves all of that peripheral's signals
  together; the four USARTs' columns are:

  | instance | code | TX | RX | CK | CTS | RTS |
  |---|---|---|---|---|---|---|
  | USART1 | 0 | PB10 | PB11 | PB9 | PC16 | PC17 |
  | USART1 | 1 | PA10 | PA11 | PB9 | PC16 | PC17 |
  | USART1 | 2 | PB10 | PB11 | PB5 | PA9 | PA8 |
  | USART1 | 3 | PA7 | PB2 | PB12 | PA13 | PA14 |
  | USART2 | 0 | PA2 | PA3 | PA4 | PA0 | PA1 |
  | USART2 | 1 | PA20 | PA19 | PA23 | PA1 | PA2 |
  | USART2 | 2 | PA15 | PA16 | PA22 | PA17 | PA21 |
  | USART2 | 3 | PC0 | PC1 | PB20 | PC2 | PC3 |
  | USART2 | 4..7 | PA15 | PA16 | PA22 | PA17 | PC3 |
  | USART3 | 0 | PB3 | PB4 | PB5 | PB6 | PB7 |
  | USART3 | 1 | PC18 | PC19 | PB5 | PB6 | PB7 |
  | USART3 | 2 | PA18 | PB14 | PB8 | PA3 | PA4 |
  | USART3 | 3 | PB16 | PB17 | - | PB18 | PB19 |
  | USART4 | 0 | PB0 | PB1 | PB2 | PB15 | PA8 |
  | USART4 | 1 | PA5 | PA9 | PA6 | PA7 | PB21 |
  | USART4 | 2 | PC16 | PC17 | PB2 | PB15 | PA8 |
  | USART4 | 3 | PB9 | PA10 | PB8 | PA14 | PA13 |
  | USART4 | 4, 6 | PB13 | PC19 | PA8 | PA5 | PA6 |
  | USART4 | 5, 7 | PC17 | PC16 | PB2 | PB15 | PA8 |

  and I2C1's six, in its field's eight codes (SCL first): 000 PA10/PA11,
  001 PA13/PA14, 010 PC16/PC17, 011 PC19/PC18, 1x0 PC17/PC16 and 1x1
  PC18/PC19 ([i2c.md](i2c.md)).

- **The debug port's pads are PC18 (SWDIO) and PC19 (SWCLK)**, the
  debug port's from reset; SW_CFG (PCFR1 bits 26:24) gives them back to
  GPIO at 100, every code 0xx leaves them the probe's, and the rest are
  "invalid". USART3's column 1, USART4's columns 4 and 6 and I2C1's 011
  and 1x1 put a signal on them, and so does the PIOC's default mapping.
- **The EXTI multiplexer** (8.3.2.2, 8.3.2.3): two bits a line in
  AFIO_EXTICR1 (lines 0..15) and AFIO_EXTICR2 (16..23) say which port's
  pin of the same number feeds the line - 00 port A, 10 port B, 11 port
  C, 01 reserved, which is NOT the sibling families' order; the three
  codes are measured, read back on lines 5 and 19.
- **AFIO_CTLR** (8.3.2.4) is the USB and USB PD pads' register - their
  pull-up modes, the PHYs' 3.3 V selection, the BC protocol's sources
  and comparators - with four pads' 10 us input filters (PA3, PA4, PB5,
  PB6). The two USB pads' pull-up MODES are a strength and not an
  enable - "only pull-up mode is set here", the pad's configuration
  deciding - and the measured answer is that a pulled input takes them
  and the I2C's alternate function does not ([i2c.md](i2c.md)).
- **AFIO's clock gate is closed out of reset** (RCC_APB2PCENR bit 0,
  3.4.6), and every `Afio` verb opens it before it touches a register. A
  read through a shut gate is not the register's: through a port's, it
  returns the last word the bus carried (measured, below).

## Types and verbs

`Pin<'A', 5>` (pin.hpp) is one pin as a type. It refuses at COMPILE time
a pin number past 23, and a pad the part's package does not bond (the
part table's `port_pins`). Levels: `set`, `clear`, `toggle` (one store
per register half, so a handler on another pin of the same port is never
caught between a read and a write), `read` (INDR), `read_out` (OUTDR).
Configuration - each verb opens the port's clock first:

| verb | the nibble | refused |
|---|---|---|
| `output()`, `output(level)` | push-pull output (0x1); the second drives the level BEFORE the mode | at compile time on a pad that shares its package pin |
| `function()` | alternate push-pull (0x9), the pad handed to whatever peripheral the remap register gives it | the same |
| `input(PinPull)` | floating (0x4) or pulled (0x8), the direction written into OUTDR first | answers FALSE, writing nothing, for `down` on a pad with no pull-down |
| `input<PinPull>()` | the same where the pull is a constant | a pull-down on a pad without one is a compile error |
| `analog()` | analog (0x0) | - |
| `release()` | floating input, the reset state | - |

`nibble()` reads the four bits back, `lock()` and `locked()` are the
one-pin face of the port's lock, `ref()` gives a `PinRef` - a pin named
at run time, what a bus request carries for its chip select - and `max`
= 1 with `duty(v)` makes a pin a `PwmChannel` of one step. `can_drive`
and `has_pull_down` state the part's facts about the pad as constants.

`Port<'A'>` is the port: `bonded` and `twinned` as masks, `in()`,
`out()`, `out_write(v)` (the whole port in one store), `out_set(mask)`
(BSHR for 0..15, BSXR for 16..23), `out_clear(mask)` (BCR, one store),
`out_toggle(mask)`; `configure(pin, nibble)` for one pin and
`configure_pins(mask, nibble)` for many - ONE store per configuration
register, false and nothing written when the mask names a pad the
package does not bond or asks a driving nibble of a shorted pad;
`lock(mask)` and `lock<Mask>()` (a pad the part has not got is a compile
error there), `locked()`, `locked_pins()`. A pin the lock holds is left
alone by every configuring verb. **CFGHR is written only from its RAM
copy, and never read to configure**, on every die, as WCH's library does
on some: `cfghr_copy()` is what the stratum last stored and
`cfghr_register()` what a read of the register answers - the pair a
suite compares - which makes every store to CFGHR this file's.

`Pad` names a pad at compile time (`{'B', 11}`), with `pad_bonded`,
`pad_twinned`, `pad_can_drive` and `pad_has_pull_down` over the part
table - the vocabulary a driver's pin tables are written in.

`Afio` (afio.hpp), monostate, every verb opening the block's gate:
`remap(Remap, code)` - false and NOTHING WRITTEN for a code the field
has not got or, for a USART, I2C1, SPI1, TIM1 or TIM2, a column not one
of whose pads this package bonds (a mask a field, `afio_codes`, computed
at compile time) - and `remap<R, Code>()` refusing the same at compile
time; `remap_code(Remap)`; `debug_config()` and `debug_port_enabled()`
for SW_CFG as it stands, and `disable_debug_port_until_reset()`, the one
verb that writes it, spelled long on purpose; `exti_source(line, port)`
(false for a line past 23, a letter with no port or a port this package
bonds nothing of) and `exti_source(line)` reading the letter back;
`control()`, AFIO_CTLR read, `usb_pads_taken()` its USB_IOEN, and
`usb_pad_pull_ups(udp, udm)` its one written field pair (`UsbPadPull`:
none, weak, 10 kOhm, 1.5 kOhm). `Remap` names the ten fields,
`afio_field_of` places them, and the columns of the USARTs, I2C1, SPI1
and the two advanced timers are data - `afio_usart_pads(n, code)`
returns a `UsartPadSet`, `afio_i2c1_pads(code)` an `I2cPadSet`,
`afio_spi1_pads(code)` a `SpiPadSet`, `afio_tim1_pads(code)` and
`afio_tim2_pads(code)` a `TimPadSet` - read by the same code that
programs the register ([usart.md](usart.md), [i2c.md](i2c.md),
[spi.md](spi.md), [tim.md](tim.md)).

## How to use it

An LED, a button with the pull-up, and an analog input:

```cpp
using Led = brio::Pin<'A', 0>;
using Button = brio::Pin<'B', 3>;       // no pull-down here, a pull-up is fine
using Level = brio::Pin<'A', 7>;

Led::output(true);                      // high before it drives: an LED from 3.3 V stays dark
Button::input<brio::PinPull::up>();
Level::analog();
```

A parallel group, configured and written in one store each:

```cpp
using PortA = brio::Port<'A'>;
PortA::configure_pins(0x00F0u, brio::pin_nibble_output);   // PA4..PA7 together
PortA::out_write((PortA::out() & ~0x00F0u) | (value << 4));   // value: four bits
```

A peripheral's column, where the program wants the pads another code
gives:

```cpp
brio::Afio::remap<brio::Remap::tim1, 1>();   // TIM1's C2N and C3N on PB0/PB1
```

A configuration frozen for good (until the next reset):

```cpp
brio::Pin<'B', 12>::lock();
```

## Bench findings

The reference suite is `test_x035_pin` (19 verdicts in `z` and 2 more
in letter `w` with its jumper PB11-PB12 in place) on a CH32X035F8U6 -
WCH's evaluation board in its QFN20 edition, over a WCH-LinkE - beside
one read over the debug port. PA5, PA6 and PA7 are wires to the desk's
CH32V006K8U6, whose dark console leaves its ends floating inputs: the
letters that drive and pull them ran green across those wires. What
they measured:

- **CFGHR reads back what its copy wrote** (letter `a`): PB11 taken
  through the five nibbles of the series, the copy and the register
  agreeing at every one - 0x44440444, 0x44444444, 0x44448444, 0x44441444
  and 0x44449444, the other seven nibbles at 4, the floating input of
  reset - on a die whose chip-identifier word reads 0x035E0611, with 1
  in bits 7:4: a die WCH's library reads CFGHR on, and not the one its
  caution is for. The five nibbles land in CFGLR too (PA6), the three inputs in
  CFGXR (PC14), and every pad released reads the floating input again.
- **The levels** (letter `b`): PA6 driven high and low reads each back
  through INDR and OUTDR, and one BSHR store toggles it; the run-time
  `PinRef` clears, toggles and reads the same pad; PA5, PA6 and PA7 are
  set in one BSHR store and toggled in another, and the whole-port write
  drives what it names.
- **The pulls** (letter `c`): PA7, free, reads 1 pulled up and 0 pulled
  down; PB12 reads high on its pull-up, and the pull-down it has not got
  is refused with nothing written.
- **The high byte** (letter `d`): PC14's output data bit is set through
  BSXR and cleared through BCR; pulled up, the pad reads 0 - the board
  ties that line, CC1, to ground through 5.1 kOhm at its connector.
- **AFIO** (letter `e`): TIM3's remap field takes code 1, reads it back
  and takes its reset code again; SW_CFG reads 0 with the whole of
  PCFR1 at zero, the debug port the probe's; EXTI line 5 starts on port
  A, takes port B with code 10 and line 19 port C with code 11 in
  EXTICR2, each read back, and both go back to port A with the two
  registers at zero.
- **Across a wire** (letter `w`, the jumper found at both levels before
  the letter judges): PA5 follows PA4 over eight edges, and PA4 driven
  low holds PA5 low against its pull-up, measured on a jumper PA4-PA5;
  the letter's jumper is PB11-PB12, PA4 and PA5 being the desk's SPI
  wires.
- **A port read through its shut gate answers with the last word the
  bus carried**, not with the pads: port A's gate is closed out of
  reset, and its INDR, read over the debug port while the gate was shut,
  returned the last word the bus had moved. A pad is read after its port
  is configured - every configuring verb of `Pin` and `Port` opens the
  gate, and `read()` and `in()` do not.
- **And a store into its output register lands nowhere**: PB3 set high
  and then made an output, its port B never configured before, drove the
  bus behind its 1.5 kOhm resistors LOW - the bit set through the shut
  gate was not there when the configuration opened it - and high once the
  verb opened the gate first (measured, the I2C suite's pull-up rail).
  `output(level)` and `input(pull)`, whose output-data store precedes
  the configuration on purpose, open the gate first.

## Not covered yet

Driver gaps, each with its reason:

- **The remap columns of TIM3 and the PIOC**: their fields are named and
  written, their columns come with their drivers (born with their first
  user) - the other blocks' are data because those blocks have drivers.
- **The external interrupts themselves** (RM 7.4): the multiplexer is
  here, the EXTI's six registers are mapped in device.hpp, and an
  `ExtInt<Pin>` is born with its first user.
- **AFIO_CTLR's other writes** - USB_IOEN, the PHYs' supply selection,
  the BC protocol, the four input filters: the USB and USB PD chapters',
  and the filters' first user.
- **The pull-up strengths PC14 and PC15 have** beyond the plain one: set
  in the USB PD block's registers, its chapter's (PC16's and PC17's are
  `usb_pad_pull_ups()`).
- **Giving the debug pads back and taking them again at run time**:
  `disable_debug_port_until_reset()` is the whole of it - there is no
  way back but a reset, which is why no suite calls it.

Implemented but not bench-verified, each with what would measure it:

- **Letter `w` on PB11-PB12**: the jumper is not on the board; strapped,
  one run of the letter would measure it.
- **What the CFGHR copy guards against**: the CH32X035F8U6's register
  reads back right (letter `a`); a die with zero in the chip-identifier
  word's bits 7:4 - another part or another lot - is what would show the
  case WCH's library keeps the copy for.
- **The lock**: PB12's configuration frozen, a reconfiguration left
  without effect, the copy unchanged: letter `l`, by name only, because
  nothing but a reset undoes it.
- **The shorted pads' refusal** is the compiler's, proven by the family
  check's negative tests on every part that has such a pair; what the
  silicon does with an output on one is what the datasheet forbids and
  no letter tries.
