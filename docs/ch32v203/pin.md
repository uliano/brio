# GPIO, the remaps and the EXTI (CH32V203)

The pads of RM chapter 10 and the external interrupt lines of 9.4:
sixteen pins a port over five ports, four configuration bits a pin,
pulls that live in the output register, two remap registers that move
whole peripherals between pad sets, and twenty-two lines of which
sixteen are pins. Documents of record: the CH32F/V20x_V30x_V31x
reference manual V2.3 (10.1 and 10.2 for the modes, 10.2.11 and 10.3.2
for the remaps, 10.3.1 for the registers and the lock, 9.4 and 9.5.1
for the lines), the CH32V203 datasheet V2.8 (3.2's pin tables for the
bonding and for which remapped function reaches which pad of each
package, note 4 for the oscillator's pads, note 7 for the shorted pins
of the small packages), and the device class notes both documents are
full of - ours is CH32V20x_D6 for every part up to the CH32V203C8 and
CH32V20x_D8 for the CH32V203RB. Drivers:
[brio/ch32v203/pin.hpp](../../brio/ch32v203/pin.hpp),
[afio.hpp](../../brio/ch32v203/afio.hpp),
[exti.hpp](../../brio/ch32v203/exti.hpp). Reference suite:
`test_v203_pin`.

## What the silicon does

### The pad

- **One nibble a pin, CNF above a TWO-BIT MODE** (10.3.1.1), sixteen
  pins a port split over CFGLR (0..7) and CFGHR (8..15). MODE carries a
  SPEED: 00 input, 01 output at 10 MHz, 10 at 2 MHz, 11 at 50 MHz. CNF
  under an input means analog (00), floating (01) or pulled (10), with
  11 reserved; under an output it means push-pull (00), open-drain
  (01), alternate push-pull (10) or alternate open-drain (11). Fifteen
  configurations in all, and the reset one is 0x4, a floating input.
- **Pulls have no register**: an input with CNF 10 is pulled, and the
  DIRECTION is that pin's bit in OUTDR - 1 up, 0 down (10.3.1.4).
  Setting a pull therefore writes the output data register of a pin
  that is an input. Measured both ways on four pads.
- **The pulls are the INPUT driver's** (10.2.7: an output mode disables
  them), so a released open-drain output is not held anywhere by the
  chip. Measured: from a low level, an open-drain output with a one in
  OUTDR leaves its pad low - there is no P-MOS and no pull to take it
  up - and the same pad reads high the moment it becomes a pulled-up
  input.
- **An analog pad's input driver is off**: INDR reads zero on it
  whatever the pad is at (10.2.9). Measured on a pad that read high an
  instant earlier.
- **BSHR sets from its low half and clears from its high half**, BCR
  clears, and both are write-one: a zero touches nothing, so a masked
  store leaves every other pin of the port alone. There is no toggle
  register - `toggle()` reads OUTDR and writes both halves of BSHR in
  ONE store, which is atomic against a handler on another pin. Writing
  OUTDR whole is the other kind of verb and takes every pin with it.
- **A port's clock gate is closed out of reset** and every configuring
  verb opens it (measured: the gate closed by hand, then one pad
  configured, then the gate reads open).
- **The unbonded pins of a port do not read the manual's reset value.**
  Measured on the LQFP48, whose port C brings out three pins: after a
  peripheral reset pulse GPIOC reads CFGLR = 0x00000000 and CFGHR =
  0x44400000 - three nibbles of 0x4 for PC13..PC15 and zero everywhere
  else, where 10.3.1.1 gives 0x44444444 for both registers. A program
  that checks a whole configuration register against the manual is
  checking the package.
- **The configuration lock is a one-way door** (10.2.5, 10.3.1.7). The
  key sequence - write LCKK with the mask, write the mask, write LCKK
  with the mask, then read twice - freezes the nibbles of the pins in
  the mask, and only a reset undoes it: there is no unlock. Measured on
  PC13: LCKR reads 0x00012000 afterwards and a configuration write
  leaves the nibble where it was.

### The remaps

- **A remap is a COLUMN, not a pin**: AFIO_PCFR1 and AFIO_PCFR2 hold one
  field per peripheral and its value selects a whole column of the
  chapter's tables (10.2.11), so SPI1's four signals move together and
  TIM1's nine do. The datasheet's own pin tables name the remapped
  function on each pad of each package, which is the per-package half of
  the same statement.
- **Which columns exist is the DEVICE CLASS's question.** On the D6:
  USART1 has two columns and not four (its high bit is AFIO_PCFR2 bit
  26, which 10.3.2.2 excludes for the D6, the D8 and the D8W alike);
  UART4 reads table 10-27 and not 10-26, so its default pads are
  PB0/PB1 and its remap is PA5/PB5 - and only the CH32V203C8 of the
  small parts has a fourth serial port at all; the ADC's four trigger
  remaps, the Ethernet's two, CAN2's, SPI3's, the FSMC's and the
  TIM8/TIM9/TIM10 fields of PCFR2 belong to other classes or to blocks
  no part of this series carries.
- **Two of those class notes are the silicon's, not the manual's.**
  Measured on the CH32V203C8: USART3's two bits and TIM2's
  internal-trigger bit are READ-ONLY AT ZERO - a write of 01 into
  USART3_RM reads back 00, and a write of 1 into TIM2ITR1_RM reads back
  0. The first settles 10.3.2.2's ambiguous note (4), "default mapping
  (00b) only exists for CH32V20x_D6, CH32F20x_D6", in favour of "on this
  class the default mapping is all there is"; the second contradicts its
  note outright, which gives the bit to "CH32F2x, CH32V2x ... whole
  series chips". The driver refuses both here and offers them on the
  other class, where no board has answered yet.
- **A column whose pads this package does not bond is a
  disconnection**, not a remap: TIM1's complete mapping is port E,
  which no part of the series has; TIM3's is PC6..PC9, which only the
  64-pin part bonds; USART2's and USART3's upper columns need
  PD3..PD12. Wherever the manual states the limit in words - "only
  LQFP100", "64-pin and above" - the bonding table says the same thing,
  so the driver asks the bonding.
- **The oscillator's two pads become GPIO through a remap** (10.2.11.2,
  datasheet note 4): PD0/PD1 are OSC_IN/OSC_OUT after every reset and
  PCFR1 bit 15 hands them to port D. The parts the two documents name
  are exactly the parts whose package bonds them.
- **SW_CFG (PCFR1 26:24) gives the debug port away.** 100 turns the
  two-wire port off and makes PA13/PA14 plain pads until the next
  reset; the field is read here and written only through a verb spelled
  long enough that nobody reaches it by accident.
- **AFIO_ECR is an event output** (10.3.2.1): EVOE brings a core
  "EVENTOUT" signal out on a pad of ports A..D (the field's other codes
  are reserved). The register takes a port and a pin and reads them
  back, measured; WHAT raises that signal on the QingKe V4 is named in
  no document of this family - the signal is the Cortex-M3's, of the
  CH32F20x half of this manual.

### The lines

- **Twenty-two lines** (table 9-3, 9.5.1's registers carry bits
  [21:0]): sixteen pin lines, then the PVD (16), the RTC alarm (17),
  the USB device controller's wake-up (18), the Ethernet's (19), the
  USB host/device controller's (20) and, on the CH32V20x_D8 alone, the
  internal 32 kHz calibration's (21).
- **The line number IS the pin number and the port is a choice**
  (10.2.3): PA1, PB1, PC1, PD1 and PE1 all reach EXTI1 and only one of
  them at a time, chosen four bits at a time in AFIO_EXTICR1..4. The
  silicon does not arbitrate - the last write takes the line - so the
  driver refuses a claim on a line another port is using and offers a
  `steal()` that says what it does. Measured: with the line stolen, the
  old pad's toggles arrive nowhere.
- **The vectors**: lines 0..4 have one each, 9..5 share one and 15..10
  share another; the PVD's, the RTC alarm's and the two USB wake-ups
  take the vector of the peripheral they belong to. A shared handler
  reads the flags of its own lines, clears them and dispatches on what
  fired - measured, ten edges over two lines of one vector, each
  counted on its own line.
- **There is no level sense and no clock gate.** RTENR and FTENR are
  the whole of the trigger selection, and there is no EXTIEN bit in
  RCC_PB2PCENR: the block is simply there, and its edge detection is
  asynchronous.
- **A line enabled nowhere raises no flag** - measured, for a pad's edge
  and for the software trigger alike - although 9.5.1.5 says a software
  trigger sets the flag whatever the enables hold. A line enabled in
  INTENR with its vector masked at the controller does raise the flag,
  which is how a program polls one; the flag is write-one-clear.
- **The software trigger's bit STANDS until the flag is cleared.**
  Measured: after `SWIEVR` is written, the bit reads back 1, and
  clearing the line's flag clears it - the STM32F1's rule for this
  register, and not the sister family's self-clearing store. The
  driver's `trigger()` is a read-modify-write of one bit for that
  reason.
- **A line in EVENR ends a WFE with no handler and no flag** (9.4.2).
  Measured: 15 core cycles from a latched line event to the platform's
  `idle()` returning, with the flag down and the controller's pending
  bit down.

## Types and verbs

[pin.hpp](../../brio/ch32v203/pin.hpp): `Pin<'A', 1>` - `output(level,
drive, speed)`, `input(pull)`, `analog()`, `function(drive, speed)`,
`release()`, `set()`/`clear()`/`toggle()`, `read()`/`read_out()`,
`nibble()`, `lock()`/`locked()`, `ref()` for the runtime `PinRef` a bus
request carries (`set`/`clear`/`write`/`toggle`/`read`, all of which do
nothing and read low on a null one), and the constexpr `Pad` the remap
tables are made of; a `PwmChannel` with max 1, so a bare pin drives an
`RgbLamp`. `Port<'B'>`
- the registers, `clock_on()` (called by every configuring verb),
`in()`/`out()`, the masked stores `out_set`/`out_clear`/`out_toggle`,
the whole-port `out_write`, `configure(pin, nibble)` and
`configure_pins(mask, nibble)`, `nibble(pin)`, and the lock in both
faces (`lock<Mask>()` checked at compile time, `lock(mask)` at run
time) with `locked()`/`locked_pins()`. `pin_nibble(mode, drive, speed)`
is the encoding, `pad_bonded(pad)` the bonding question.

[afio.hpp](../../brio/ch32v203/afio.hpp): the columns as constexpr data
- `afio_tim1_pads(code)`, `afio_tim2_pads`, `afio_tim3_pads`,
`afio_tim4_pads`, `afio_usart1_pads`, `afio_usart2_pads`,
`afio_usart3_pads`, `afio_uart4_pads` (whose table is the class's),
`afio_usart_pads(instance, code)`, `afio_spi1_pads`, `afio_i2c1_pads`,
`afio_can1_pads`, and the fixed columns `afio_spi2_pad_set` /
`afio_i2c2_pad_set` of the two instances with no remap field - plus
`Remap`, the enumeration of the fields this series has,
`afio_remap_has_code(remap, code)` (the class AND the bonding),
`afio_column_fully_bonded()`, and `Afio`: `remap(r, code)` refusing what
the part has not, `remap<R, Code>()` refusing it at compile time,
`remap_code(r)`, `exti_source(line, port)` and its read-back,
`event_output(port, pin)`, `debug_config()`/`debug_port_enabled()` and
`disable_debug_port_until_reset()`.

[exti.hpp](../../brio/ch32v203/exti.hpp): `Exti` - `implemented(line)`,
`irq(line)` (an optional: the two lines the other class adds have no
vector named in this stratum), `vector_lines(irq)`, `sense(line,
ExtiSense)` and its read-back, `interrupt(line, on)`, `event(line,
on)`, `trigger(line)`/`triggered(line)`,
`pending()`/`clear()`/`clear_lines()`, `select(line, port)` /
`steal()` / `selected()` / `in_use()`, `isr(lines)` for a vector's body
and `served(fired, line)` for its dispatch, `release(line)`.
`ExtiLine<n>` names a line as a constant (a line this part has not got
is a compile error) and `ExtInt<Pin>` names one through its pad -
`claim(pull)`, `select()`, `steal()`, `configure(sense)`, `arm(on)`,
`event(on)`, `trigger()`, `pending()`, `clear()`, `served(fired)`,
`release()`. `exti_lines_distinct<...>()` is the compile-time check an
application puts on its own set of lines.

## How to use it

A pad, and a pad handed to a peripheral:

```cpp
using Led = brio::Pin<'B', 2>;
Led::output();                                   // push-pull, 50 MHz
Led::toggle();

using Button = brio::Pin<'A', 1>;
Button::input(brio::PinPull::up);                // the pull is OUTDR's bit
if (Button::read()) { /* ... */ }

using Tx = brio::Pin<'A', 9>;
Tx::function();                                  // alternate push-pull
```

A whole port at once, and a configuration frozen for good:

```cpp
brio::Port<'B'>::configure_pins(0x00F0u,
    brio::pin_nibble(brio::PinMode::output, brio::PinDrive::push_pull,
                     brio::PinSpeed::fast));
brio::Port<'B'>::out_set(0x00F0u);               // the other pins untouched

(void)brio::Pin<'C', 13>::lock();                // until the next reset
```

A peripheral on a remapped column - the code and the pads from the same
table:

```cpp
constexpr brio::SpiPadSet pads = brio::afio_spi1_pads(1);   // PA15/PB3/PB4/PB5
static_assert(brio::afio_remap_has_code(brio::Remap::spi1, 1));
brio::Afio::remap<brio::Remap::spi1, 1>();
```

A pad as an interrupt, and the same line as a wake event:

```cpp
using Sense = brio::Pin<'A', 1>;
using Line = brio::ExtInt<Sense>;

Line::claim(brio::PinPull::up);                  // input + pull + EXTICR
(void)Line::configure(brio::ExtiSense::falling);
(void)Line::arm(true);
brio::Pfic::enable(*Line::irq());                // exti1

extern "C" BRIO_CH32_INTERRUPT void exti1_handler() {
    const uint32_t fired = brio::Exti::isr(brio::Exti::vector_lines(brio::Irq::exti1));
    if (brio::Exti::served(fired, Line::line)) { /* ... */ }
}

(void)Line::arm(false);
(void)Line::event(true);                         // ends idle() with no handler
```

util/input_scanner.hpp's `InputScanner` takes any type with a `read()` -
`struct Contact { static bool read() { return Sense::read(); } };` - and
publishes an `InputEdge` per debounced flip, which is the polled way to
the same question.

## Bench findings

The reference suite is `test_v203_pin` (30 verdicts in `z` on the
CH32V203C8T6, plus the letter `k` by name, which needs a finger). It
runs with NO WIRE: a pad reads its own level through the input register
in every mode but analog, and that pad's own output feeds the EXTI line
of its number, so senses, edges, flags and the event mode are all
measurable on a bare board. The tree runs on the HSI's PLL at 144 MHz
because one of the remaps hands the crystal's pads to GPIO. `z` is
repeatable: running it twice in a row gives the same thirty verdicts,
the second time with PC13 already locked from the first.

- **The nibble**: a configuring verb opens the port's clock gate; a
  reset port holds 0x4 in the nibble of every pin it bonds and ZERO in
  the others (GPIOC on this package: CFGLR 0x00000000, CFGHR
  0x44400000); all fifteen configurations of 10.3.1.1 land as
  `pin_nibble()` spells them, 15 of 15 in the low register (PA1) and 15
  of 15 in the high one (PB12); BSHR's two halves, BCR and `toggle()`
  do what the chapter says.
- **The pulls**: four floating pads read high pulled up and low pulled
  down a microsecond after each switch, and the direction is OUTDR's bit
  under nibble 0x8, both ways.
- **The open drain**: a push-pull output drives its own pad both ways;
  an open-drain output with a one in OUTDR does not drive it high (the
  pad stayed low at the store and 100 us after it) and does pull it low;
  the same pad reads high once it is a pulled-up input; an analog pad's
  input register reads zero on a pad that read high an instant before.
- **The whole-port verbs**: `configure_pins()` writes one nibble into
  the six pins of a mask; the mask set, cleared, toggled and written
  whole reads back on INDR; the three masked stores leave the pin beside
  the mask alone and the whole-port store takes it with them.
- **The remaps**: nine columns written, read back and restored on this
  part (SPI1, I2C1, UART4, TIM1, TIM2, TIM3's partial, both of CAN1's
  and the oscillator's pads as GPIO); seven refused as absent (USART2's
  and USART3's remapped columns, TIM3's full one, TIM4's, TIM5's
  channel 4, TIM2's internal trigger and the Ethernet's
  pulse-per-second); USART3_RM and TIM2ITR1_RM written RAW read back
  zero, which is why the driver refuses them; USART1's remapped column
  exists on this package and is never selected, because its TX pad is
  the console's; the debug port's field reads 0 through all of it; and
  AFIO_ECR takes a port and a pin, reads them back and turns off again,
  refusing port E.
- **The lines**: ten toggles of PA1 on its own line 1 are five rising,
  five falling and ten of both edges, one interrupt each; a line with
  neither edge enabled is silent under a toggling pad; the flag of a
  line armed in INTENR with its vector shut stands for a poller and is
  cleared by writing one; an edge on a line enabled nowhere raises no
  flag; the software trigger raises the flag of an armed line, its own
  bit stands until the flag is cleared, and one trigger is one
  interrupt; the 9..5 vector counted three edges of PB5 and two of PB6
  with neither line stealing the other's, in five entries, and the
  15..10 vector six of PB12 and four of PB13 in ten; and a line taken
  from PA1 by `steal()` counted none of its four toggles afterwards.
- **The event mode**: `idle()` with nothing armed waits one kernel tick;
  with a line's event already latched it returns after 15 core cycles
  and no tick at all, with no flag raised and no pending interrupt.
- **The KEY's pad** (letter `k`, by name): a thousand samples at 10 ms
  with nobody pressing the button read low every time.

## Not covered yet

Driver gaps, each with its reason:

- **What the KEY on PA0 does when pressed.** The letter exists and
  wants a finger on the board; with nobody pressing, the pad read low a
  thousand times in a row, which is consistent with the vendor's
  "active high, no external resistor" and proves none of it.
- **PC14/PC15 as GPIO** (10.2.11.1: the LSE's pads are port C's pins
  with LSEON clear): this board's 32 kHz crystal sits on them and the
  backup domain's own chapter runs on it ([rtc.md](rtc.md)), so
  stopping that oscillator to take the two pads would cost that
  chapter its clock.
- **The remap columns no chapter has followed a signal onto**: CAN's
  two, which have no driver here, and I2C1's second, whose pads carry
  no bus on this board ([i2c.md](i2c.md)). The tables are data here
  and the fields are verbs; where a chapter uses a column it measures
  the signal on the pad - the fourth serial port's second column
  ([usart.md](usart.md)), SPI1's second and TIM2's partial remap under
  it ([spi.md](spi.md)).
- **The fields of other classes and absent blocks**: the ADC's four
  trigger remaps, the Ethernet's two, CAN2's, SPI3's, the FSMC's and
  PCFR2's TIM8/TIM9/TIM10 fields. Each belongs to a device class this
  series is not or to a block no part of it carries, so no verb reaches
  them.
- **What drives the event output.** AFIO_ECR is implemented and
  measured as a register; no document of this family names an
  instruction or a signal that raises EVENTOUT on the QingKe V4 core, so
  a program that enables it has connected a pad to something this
  project cannot describe.
- **The shorted pins of the three smallest packages.** The datasheet's
  note 7 says the 20- and 28-pin packages tie at least two I/O
  functions to one physical pin and that driving both is a hazard; the
  part tables state the bonding pin by pin, and nothing here refuses to
  configure both halves of such a pin.

Implemented but not bench-verified, each with what would measure it:

- **The eight parts other than the CH32V203C8.** The stratum compiles
  for all nine both ways the hardware prologue can be built, and the
  refusals are checked per part by the family fixture; what would
  measure the rest is a board. Three answers are the CH32V203RB's
  alone: UART4's other table, USART3's two columns, and EXTI lines 19
  and 21.
- **The two USB wake-up lines.** `ExtiLine<n>` reaches each of the four
  peripheral lines, and two of them are measured by the chapter that
  owns the source - the PVD's through its software trigger
  ([sleep.md](sleep.md)) and the RTC alarm's as an interrupt, as an
  event and as a Stop's end ([rtc.md](rtc.md), [sleep.md](sleep.md)).
  What would measure the other two is the USB chapter, whose controller
  raises them.
- **The configuration lock on a whole mask**, and what a peripheral
  reset pulse does to a standing lock: the suite locks one pin of one
  port, which is all a board with one free port-C pad allows.
