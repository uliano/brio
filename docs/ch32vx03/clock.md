# RCC (CH32V203 and CH32V303)

This document is for a programmer bringing a CH32V203 or a CH32V303 to
the rate the program wants, and for one who needs the clock block
itself: the roots and the PLL, the peripheral gates, the clock output,
the clock security system. Its first subject is the clock task, the
driver every driver that takes a clock depends on (a task, in brio,
is the driver layer over a block's registers that a program or an
active object calls; an active object is a component of a brio program
with its own event queue, served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)). A static
`Clock<source, hz, xtal_hz>` names a rate as a type and is the one rate
truth of the program: its `init()` takes the tree from the reset clock
(the 8 MHz HSI, undivided) to that rate, and each such driver's
`init()` takes the type's tag and computes its divisor from the rates the type
states - HCLK, the two peripheral buses, what a timer counts, what the
ADC and the USB blocks get - never from the hardware
([design/clock.md](../design/clock.md) is the model). A
`DynamicClock<Rates<...>, Users...>` is a pack of such types the program
switches between at run time, every driver that follows the tree listed
as a user and rebased before the tree moves. `Rcc` is the resource under
both, the block itself, and `Mco` the small task over the clock output
pad.

The smallest use, the first block of "How to use it" below, which is
most programs:

```cpp
using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

SysClock::init();                  // the HSI whole, x18
Serial::init(clock, 115200);       // the divisor folds at compile time
brio::Ticker::init(clock);
```

Line by line: `SysClock` is a type alias that names the rate - the PLL
fed by the HSI, 8 MHz arriving whole and multiplied by eighteen to
144 MHz, this family's ceiling; the PLL's input divider and multiplier,
HPRE and the two bus prescalers are found at compile time, and every
rate the tree derives from it (`pclk1_hz`, `pclk2_hz`, `usb_hz`,
`adc_hz` ...) is a constant of the type. `clock` is an empty tag object
of that type: it holds nothing, its type carries the rates, and it is
the argument every driver's `init` takes to learn them. `SysClock::init()`
is the clock task's verb that takes the tree to the rate, called first
in `main()` (its second verb, `restore()`, puts the rate back after a
sleep, [sleep.md](sleep.md)): it parks SYSCLK on the HSI, writes the
prescalers while the machine is slow, starts the roots the rate needs
and the PLL, switches, and answers whether all of it took (the excerpt
discards the answer; a program reads it). The
next two lines are other drivers' `init`, each handed the tag: `Serial`
is the program's alias for a serial port ([usart.md](usart.md), USART1
for instance, `brio::Uart<1, P>`), whose baud divisor is computed from
the tag's rate for its bus (PCLK2 for USART1) and folds at compile time
because the rate is a constant; `brio::Ticker` is the kernel's timebase
([platform.md](platform.md)), whose compare is computed from HCLK. All
three run in `main()` before `brio::enable_interrupts()` and the
kernel's `run()`.

What decides correctness even there:

- **The clock task first.** A driver initialized before
  `SysClock::init()`, on a chip still at the reset clock (what
  `Clock<ClockSource::internal, 8'000'000>` names), computes its divisor
  for the rate the type claims and runs at the ratio of the two: under
  the 144 MHz type above, eighteen times too slowly for a block clocked
  from HCLK or PB2 (the ticker, USART1, SPI1, TIM1: a port asked for
  115200 baud speaks at 6400) and for a timer on PB1 (which counts twice
  its divided bus, 144 MHz by the type), nine times too slowly for a
  port or a bus host on PB1, whose rate the type states as 72 MHz, the
  stratum's cap - derived from the divisor arithmetic, not measured. A
  port or the ticker writes its divisor once, so it is right again once
  the clock task has run. Nothing checks the order: no driver reads the
  tree.
- **Its answer.** `init()` answers false when the HSI or the HSE
  never comes ready, the PLL never locks or a switch never takes, each
  wait a bounded poll (`clock_timeout_turns`, 100 000 turns), and it
  stops at that step. Past its first two steps (the HSI started, SYSCLK
  parked on it) the prescalers of the rate asked are already written, so a
  false can leave the chip neither on the reset clock nor at the rate
  its type claims (on the HSI with PB1 halved, for a 144 MHz type whose
  PLL never locked), and every divisor computed from the type is wrong.
- **What the type refuses.** A rate the tree cannot make exactly, a
  crystal rate outside the part's oscillator range, a crystal on a
  package that brings out no oscillator pad, a PLL input or output
  outside the part's ranges, an HCLK above the part's ceiling: each is a
  compile error in `Clock`, never a rounded rate.
- **A dynamic clock's users.** `DynamicClock::init()` enters the pack's
  first rate with no fan-out and runs first in `main()` the same way. A
  driver that follows the tree and takes a dynamic clock (the serial
  port, the ticker, the SPI and I2C hosts) does not compile unless it is
  listed among the users; the timers, the ADC and the I2S take a static
  clock only. `set<hz>()`, `set(hz)` and `set_index()` are called from
  `main()` or an active object's handler, never from an interrupt
  handler, one caller at a time, and only while nothing that depends on
  the rate is mid-transfer; `restore()` is the sleep site's
  ([sleep.md](sleep.md)), called on its wake path, an interrupt body
  among them, and does nothing while a switch is in progress.

The rest of this page, in order: the documents of record; what the
silicon does (the roots and the PLL with its per-class input divider,
the prescalers, the ADC's and the USB blocks' dividers, the flash, the
clock security system and the ready interrupts, the clock output, the
LSI, the gates); the types and verbs (`Clock`, `Rates` and
`DynamicClock`, `Rcc` and `Mco`, and what is not in this file); how to
use it (one rate, the same from the crystal, a pack of rates, a driver
that follows the rate, the clock output, the clock security system);
the bench findings, measured on the CH32V203C8T6 and the CH32V303VCT6;
and what is not covered.

The clock tree of RM ch. 3 (the reference manual named below) as this
stratum drives it: the static `Clock` every driver derives its rate
from, the `DynamicClock` that moves between a pack of rates at run
time, and the `Rcc` resource under both - the roots, the PLL, the
switch, the four prescalers, the output, the security system and the
peripheral gates. The pieces of the tree that live in another chapter
are here too, because they are the tree: the PLL's input divider for
the HSI (EXTEN_CTR.HSIPRE, RM 33.2.1) and the rate above which the
flash chapter asks for HCLK halved around an erase (RM 32.1).

## Documents of record

The CH32F/V20x_V30x_V31x reference manual V2.3 (3.3 for the tree, 3.4
for the registers, 33.2 for EXTEN) read under the **CH32V20x_D6** class
note for every part up to the CH32V203C8, **CH32V20x_D8** for the
CH32V203RB and **CH32V30x_D8** for the four CH32V303 parts (the device
classes are the manual's names for the groups of parts whose registers
differ; the part table, `brio/ch32vx03/parts/`, gives each part its
class) - the chapter's simple tree, which it gives the D6 and the
V30x_D8 alike, the PLL2, PLL3 and PREDIV of the D8C classes being
another family's - the CH32V203 datasheet V2.8 (tables 4-11 and 4-13 to
4-15 for the oscillators and the PLL) and the CH32V303/305/307/317
datasheet V3.5 (tables 4-12 and 4-14 to 4-16). Driver:
[brio/ch32vx03/clock.hpp](../../brio/ch32vx03/clock.hpp). Reference
suite: `test_vx03_clock` (a suite's letter is one command key of its
console and the group of verdicts it runs; `z` the key that runs the
suite's single-board set, here every letter, `a` to `i`).

## What the silicon does

- **Two high-speed roots and one PLL.** The HSI is an 8 MHz RC, on out
  of reset, factory-calibrated (HSICAL, read-only) and nudged by a
  five-bit HSITRIM whose reset value is the centre, 16, worth about
  20 kHz a step. The HSE takes a crystal or, with HSEBYP written while
  HSEON is clear, an external clock. The PLL multiplies its input by
  x2..x16 or x18 - the last code is x18 and not x17, which is the one
  trap in PLLMUL - and **its input divider is per device class**:
  PLLXTPRE divides the HSE by one or two on the CH32V20x_D6 and the
  CH32V30x_D8 and by four or eight on the CH32V203RB, whose oscillator
  is 32 MHz and nothing else. The HSI's own divider is not an RCC bit at all: it is
  EXTEN_CTR.HSIPRE, which is 0 out of reset - the PLL sees 4 MHz, not
  8 - and whose sense is the opposite of PLLXTPRE's, the bit set being
  the whole clock.
- **The PLL's own range is not the multiplier's.** The datasheets rate
  the input at 3..25 MHz (4..25 on the CH32V203RB) and the output at
  18..144 MHz (40..240 there), the CH32V303's the same as the
  CH32V203C8's, so a rate the ladder can make is not
  always a rate the PLL may run at; both edges are part facts and both
  are checked at compile time.
- **PLLMUL, PLLSRC and PLLXTPRE take a write only while the PLL is
  off, and the PLL does not stop while it is the system clock.** A
  program that reaches for a new rate without parking SYSCLK on the HSI
  first has every one of those writes swallowed in silence, and keeps
  the rate it had while its drivers compute divisors for the rate they
  asked for.
- **One HPRE and two peripheral prescalers.** HPRE's ladder is 1, then
  2, 4, 8, 16, 64, 128, 256, 512: **it skips 32**. PPRE1 and PPRE2
  share one encoding (0xx undivided, then /2 /4 /8 /16). A timer on a
  bus whose prescaler is not 1 counts at twice the bus rate (figure
  3-3) - which is the timers' chapter, stated here because it is a
  property of the tree.
- **The ADC has a prescaler of its own** off PCLK2 (/2 /4 /6 /8) and a
  rating of 14 MHz, so a PCLK2 above eight times that has no code that
  keeps the converter in range. The duty-cycle bit beside the field
  (ADCDUTY) belongs to every class; the second one (ADC_DUTY_SEL, a
  75 % duty) is the CH32V30x_D8's alone and even there only by lot
  number, so its verb writes the bit and reads it back, answering what
  the die kept - nothing, on the CH32V303VCT6 measured, where ADCDUTY
  takes its write and moves neither a reading nor a conversion's length
  ([adc.md](adc.md)).
- **The USB controllers want exactly 48 MHz** from the PLL through
  USBPRE, and 3.4.2 asks for the divider to be written before the USB
  clock gates are opened. The codes are /1, /2 and /3 on the
  CH32V20x_D6 and the CH32V30x_D8; the fourth code, /5 from a 240 MHz
  PLL, is the CH32V20x_D8's and even there it depends on the lot
  number. On the CH32V303 the 48 MHz feeds the host/device controller
  alone, the full-speed device controller not being on that part
  (below).
- **There is no flash wait-state field.** The flash chapter's register
  table (32.4) has no latency at all: the array is split into a
  zero-wait region and a non-zero-wait one by the part, and
  FLASH_CTLR.SCKMOD chooses whether the flash is read at the system
  clock or at half of it, defaulting to half. What the chapter does ask
  for is at the other end - above 120 MHz an erase or a program wants
  HCLK halved around it (32.1), which the flash driver does not do
  behind the program's back: it refuses an erase or a program at such a
  rate instead ([nvm.md](nvm.md)).
- **The clock security system is the non-maskable interrupt's.** With
  CSSON set and the HSE ready, a failure of the oscillator makes the
  silicon switch SYSCLK to the HSI, stop the HSE and the PLL, brake the
  advanced-control timer and raise the NMI with CSSF set; CSSC clears
  the flag. The detector is armed by hardware when HSERDY rises and
  disarmed when the oscillator stops. The ready flags of the five roots
  have ordinary interrupt enables in RCC_INTR and share the RCC vector,
  and each flag is cleared by a write-only bit sixteen places above it -
  but the HSI's is never raised (measured on both parts, below), so four
  roots of the five can interrupt.
- **The clock output is a pad on PA8** carrying SYSCLK, the HSI, the
  HSE or the PLL halved - the four codes of MCO[3:0] this class has,
  the rest belonging to the D8C families - and 3.3.5.5 warns that the
  output may truncate cycles when the source changes. **Two packages of
  the series do not bring PA8 out** (the CH32V203F6 and the
  CH32V203G6), so what the multiplexer carries and what leaves the chip
  are two questions.
- **The LSI is a wide RC**, the watchdog's and the RTC's root, with its
  two bits (LSION, LSIRDY) in RCC_RSTSCKR beside the reset flags. The
  datasheets give 25..60 kHz for it, typically 39, on the CH32V203C8
  and the CH32V303 alike (25..45, typically 32, on the CH32V203RB): a
  nominal rate, never a computed one.
- **The peripheral gates are the whole of 3.4.6 to 3.4.8**, one bit per
  block on each of the three buses, with a reset line beside each -
  and a bit for a block the part has not got reads back zero (below).

## Types and verbs

[brio/ch32vx03/clock.hpp](../../brio/ch32vx03/clock.hpp).

`Clock<source, hz, xtal_hz>` is the static main clock: `internal` (the
HSI), `crystal` or `external` (the HSE at the rate named third), `pll`
(the HSI multiplied, or the crystal named third multiplied). It is a
tuple of constants, not just a rate - `hz` is HCLK, `pclk1_hz` and
`pclk2_hz` the two bus rates, `timclk1_hz`/`timclk2_hz` what a timer on
each counts, `usb_divider`/`usb_hz` what the USB blocks get,
`adc_code`/`adc_hz`/`adc_in_spec` what the converter gets and whether
that is within its rating, `sysclk_source` the root SWS will report,
`pll_in_div`/`pll_input_divided`/`pll_mul` the ratio the search folded,
and `flash_needs_halving` the flash chapter's rule at this rate. Every
one is constexpr, so a divisor built from them folds. `init()` parks
SYSCLK on the HSI, programs the prescalers while the machine is slow,
starts the roots and switches - returning false for a root that never
comes ready, a PLL that never locks or a switch that never takes;
`restore()` is that same order, for a wake that dropped the tree back
to the HSI.

`Rates<R0, R1, ...>` + `DynamicClock<Rates<...>, Users...>` is the
runtime regime, the STM32F4's shape: R0 is the boot rate, the Users are
the drivers a switch rebases in list order before anything moves, and
every switch is a rate's own `init()` - which parks on the HSI, so one
order serves both directions. The rate in force answers `hz()`,
`pclk1_hz()`, `pclk2_hz()`, `usb_hz()`, `adc_hz()`, `adc_in_spec()`;
the pack answers `rate_count`, `rate_hz(i)`, `rate_index()`,
`rate_pclk1_hz(i)`, `rate_pclk2_hz(i)`, `rate_usb_hz(i)`,
`rate_source(i)` - the discrete-rate surface
[ch32vx03/delay.hpp](../../brio/ch32vx03/delay.hpp) indexes its
per-rate table by. `set<hz>()` and `set(hz)` take the first rate at
that rate, `set_index<i>()` and `set_index(i)` name one exactly (two
rates may share an hz: 48 MHz from the HSI's PLL and from a crystal's
are different tuples), `restore()` puts the current rate back with no
fan-out, `switching()` says a switch is between its first and its last
store, and `rebases<U>` is what a clocked driver asserts about itself.
**A user is handed HCLK**, never a bus rate: `pclk1_hz_at(hclk)` and
`pclk2_hz_at(hclk)` are the arithmetic it derives its own from, because
the RCC still holds the old prescalers when the fan-out runs.

`Rcc` is the resource under both. The roots: `hsi_enable`/`hsi_start`/
`hsi_stop`/`hsi_ready`/`hsi_trim`/`hsi_calibration`, `hse_enable`/
`hse_start`/`hse_stop`/`hse_ready`/`hse_in_low_power`, `lsi_enable`/
`lsi_start`/`lsi_stop`/`lsi_ready` - the `enable` verb returning at once
and the `start` verb waiting a bounded time, which is what lets a
program time a ramp. The PLL: `pll_start(from_hse, divided, mul)`,
`pll_stop`, `pll_ready`, `pll_from_hse`, `pll_input_divided`,
`pll_multiplier`. The switch and the dividers: `sysclk_select`,
`sysclk_status`, `prescalers`, `hpre_code`, `ppre1_code`, `ppre2_code`,
`adc_prescaler`, `adc_duty_extended`, `adc_duty_75` (the lot's bit: on
every other class it writes nothing and answers false), `eth_prescaler`,
`usb_prescaler`. The security system: `clock_monitor`, `clock_failed`
and `css_isr()`, the non-maskable interrupt's body - which clears the
flag and says whether the CSS was the reason, leaving what to do about
the rate to the program, since by then SYSCLK is already back on the
HSI. The ready interrupts: `ready_interrupts`, `interrupt_flags`,
`clear_interrupt_flags` and `ready_isr()`, the RCC vector's body. The
gates: `enable`, `disable`, `enabled` and `reset` over `Bus::hb`,
`Bus::pb2` and `Bus::pb1`, which is what every configuring driver opens
its own gate with. `Mco` is the small task over the output pad:
`init(source, speed)` claims PA8 and selects, `off()` releases both,
`source()` reads back, `has_pad` is the part fact that makes `init()`
answer false instead of refusing to compile.

What is not in this file: RCC_BDCTLR - the LSE, the RTC's clock select
and RTCEN - because the whole register is write-protected by PWR's DBP
bit and belongs with the backup domain, where `RtcDomain` owns it
([rtc.md](rtc.md)), and RCC_RSTSCKR's reset flags, which are
[reset.hpp](../../brio/ch32vx03/reset.hpp)'s; this file owns only the
two LSI bits that share that register.

## How to use it

One rate for the life of the program, which is what most programs want:

```cpp
using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

SysClock::init();                  // the HSI whole, x18
Serial::init(clock, 115200);       // the divisor folds at compile time
brio::Ticker::init(clock);
```

The same rate from the board's crystal, which is the third parameter:

```cpp
using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000, 8'000'000>;
```

A program that changes rate at run time names the pack and the drivers
that must follow it:

```cpp
using Top  = brio::Clock<brio::ClockSource::pll, 144'000'000>;
using Mid  = brio::Clock<brio::ClockSource::pll,  48'000'000>;
using Boot = brio::Clock<brio::ClockSource::internal, 8'000'000>;
using SysClock = brio::DynamicClock<brio::Rates<Top, Mid, Boot>,
                                    brio::Ticker, Serial>;
constexpr SysClock clock;

SysClock::init();                  // Top, the pack's first
SysClock::set<48'000'000>();       // the users rebased, then the switch
SysClock::restore();               // after a wake that dropped the tree
```

A driver that follows the rate offers `static void rebase(uint32_t
hclk)` and derives its own bus rate from what it is handed:

```cpp
static void rebase(uint32_t hclk) {
    const uint32_t pclk = brio::pclk1_hz_at(hclk);   // this block is on PB1
    // ... drain what is in flight, then program the divisor
}
```

The clock output, for a counter or a scope at the pad:

```cpp
brio::Mco::init(brio::McoSource::hsi);
brio::Mco::off();
```

The clock security system, once the crystal is running:

```cpp
BRIO_CH32_VECTOR(nmi_handler) {
    if (brio::Rcc::css_isr()) {
        // SYSCLK is already back on the HSI and the PLL is off
    }
}

brio::Rcc::clock_monitor(true);
```

## Bench findings

The reference suite is `test_vx03_clock` (55 verdicts in `z` on the
CH32V203C8T6 and 54 on the CH32V303VCT6 - both boards carry an 8 MHz
crystal and a 32.768 kHz one; the CH32V203C8T6's board straps PA8 to
PA6, which counts the clock output, and nothing else is wired). Its rate pack is the ten trees those boards can
make: the bare HSI, the PLL on the HSI at 48, 52, 96 and 144 MHz, the
bare 8 MHz crystal, and the PLL on that crystal at 48, 72, 96 and
144 MHz - 52 MHz being the one rate the PLL can only reach through the
HALVED input, which is what exercises the EXTEN bit.

- **Every rate of the tree is reached, and the console survives all of
  them.** The suite's clock is dynamic and the console's own port is
  one of its users, so the verdict lines are printed through a divisor
  computed for the rate they name. Twenty switches up and down the pack
  leave the tick counting exactly the span asked at every step and
  `delay_us` serving its wait at every one.
- **The rates measured against the host's clock**, by bracketing 2000
  kernel ticks between two console lines: every HSI-rooted rate reads
  **+0.42 to +0.46 %** (8, 48, 52, 96 and 144 MHz) and every
  crystal-rooted one **-0.04 to -0.05 %**, over two passes of the whole
  pack. The method carries a systematic bias of about 0.07 % - the
  transmission time of the closing line at 115200 - which puts the RC of
  this die around half a per cent fast and the crystal's rates within a
  few hundredths of exact. That the five rates on each root agree with
  each other to within 0.03 % is the PLL's and the prescalers'
  arithmetic being exact: the whole error is the root's. On the
  CH32V303VCT6, with the closing line alone on the wire for five ticks
  as the opening one is: every HSI-rooted rate **+0.26 to +0.38 %** and
  every crystal-rooted one **-0.00 to -0.11 %**, over two passes. The
  spread is that board's probe: its serial bridge forwards in 128-byte
  blocks and flushes a partial one some two milliseconds after the line
  goes idle, and not always the same two, so the bracket resolves about
  a tenth of a per cent there - the crystal exact within it, the HSI of
  that die a third of a per cent fast, and the halved input again
  indistinguishable from the whole one.
- **The LSI takes about 3.1 ms to LSIRDY** from a cleared LSION (3100
  to 3200 us over several runs, measured in 50 us steps), with the
  backup domain untouched - between the datasheet's two figures, 230 us
  with the LSE running and 5 ms without - and 4.35 to 4.40 ms on the
  CH32V303VCT6. LSIRDY falls in under one 50 us step on both. Its RATE
  is not measured here: nothing in this chapter can count it.
- **The board's 8 MHz crystal reaches HSERDY in 1.7 ms** (repeatable to
  the 50 us step), against the datasheet's 2.5 ms typical for an 8 MHz
  crystal; the CH32V303 board's reaches it in 1.05 ms, against its
  datasheet's 1.5 typical and 4 at most. HSERDY falls in under one step
  on both.
- **The ready interrupt reaches the RCC vector.** With HSERDYIE armed
  over that same ramp, the vector runs exactly once, its body finds
  HSERDYF - and only that flag - standing, and leaves none behind, so
  the line does not re-enter. Letter `i` does the same for the other
  roots, ten runs on each part: the LSI restarted with LSIRDYIE armed
  (the vector once, LSIRDYF alone), the PLL - off under the bare
  crystal - restarted by the switch back to the boot rate with
  PLLRDYIE armed (PLLRDYF alone), and the LSE, behind the backup
  domain's door, stopped and restarted with LSERDYIE armed (LSERDYF
  alone, the crystal ready **754 to 755 ms** after LSEON on the
  CH32V203C8T6's board and **785 to 802 ms** on the CH32V303's).
- **THE HSI'S RESTART RAISES NO READY FLAG**, against 3.3.2's and
  3.4.3's text, on both parts: stopped under the crystal's PLL (HSIRDY
  falls) and started again with HSIRDYIE armed, the vector never runs
  in 50 ms; unarmed, HSIRDYF does not stand once HSIRDY has risen,
  whether the oscillator was off for 100 us or for 20 ms; and RCC_INTR
  read at the top of main() after a reset holds no flag either. Letter
  `i` judges exactly that, ten runs on each part. A program that wants
  to know the HSI is back polls HSIRDY - which is what `hsi_start()`
  does.
- **The HSI can be stopped.** With SYSCLK on the PLL fed by the
  crystal, clearing HSION drops HSIRDY and the program runs on -
  measured because the chapter recommends against stopping it and says
  nothing about refusing - on both parts; the CH32V303VCT6's factory
  calibration reads 0x70.
- **The clock security system arms over a healthy crystal**: CSSON
  reads back, CSSF stays clear and the non-maskable interrupt does not
  fire in 100 ms of monitoring; CSSON clears again on demand.
- **The clock output takes all four sources of this class** - SYSCLK,
  the HSI, the HSE and the PLL halved - each read back through the
  multiplexer, with the pad claimed and released. **What leaves the pad
  is counted** where PA8 is strapped to PA6 (the strap probed first,
  both levels against the opposite pull): TIM3 in external clock mode
  1 on PA6, over a 5 ms gate of a core clock made from the crystal,
  reads the HSE at **7 999 911 to 8 000 111 Hz** - one edge either side
  of 8 MHz exactly, the gate's resolution - and the HSI at **+0.59 to
  +0.67 %** (8 046 909 to 8 053 309 Hz) over ten runs, inside the
  datasheets' calibrated accuracy. SYSCLK and the PLL halved are 48 to
  144 MHz at the rates that make them, past the pad's fastest class (50
  MHz in the datasheets' I/O tables) and past what a timer clocked at
  HCLK can count of an outside edge, so the letter reads them back and
  does not count them.
- **The USB divider's three codes each feed an enumeration**: /3 under
  the USB console's 144 MHz, /2 under both USB suites' 96 MHz, and /1
  under a tree rebuilt at 48 MHz - code 00 written before the gate opens
  and the host enumerating the device on it, on the CH32V203C8T6's
  device controller and the CH32V303VCT6's host/device one alike
  ([usb.md](usb.md), [usbfs.md](usbfs.md), letter `k` of each suite).
- **Every peripheral gate this part has opens and closes**, each read
  back one at a time - twenty-six on the three buses of the
  CH32V203C8T6, forty-two on the CH32V303VCT6 (DMA2, the FSMC, the RNG,
  SDIO, port E, TIM5 to TIM10, SPI3, UART5 to UART8 and the DAC beside
  the CH32V203C8's gates, and not its USB device controller's) -
  USART1's and GPIOA's being the two the suite leaves alone, the console
  running on them, and a reset pulse leaves a gate where it was. **A
  gate bit for a block this part has not got reads back zero**: DMA2's,
  port E's and TIM5's stay clear when written on the CH32V203C8T6, so
  the enable registers implement the bits of the die and not of the
  family - and on the CH32V303VCT6 THE USB DEVICE CONTROLLER'S bit
  (RCC_APB1PCENR bit 23) is such a bit, against RM ch. 21's own opening,
  which says that controller applies to the whole family: the one
  full-speed controller that part has is the host/device one of ch. 23,
  on the HB gate.

## Not covered yet

Driver gaps, each with its reason:

- **RCC_CFGR2.** Its PLL2, PLL3, PREDIV and I2S/RNG selectors belong to
  the D8C classes of other families; the one field of it a part of this
  stratum could use is the USBFS clock source, whose other choice is the
  high-speed PHY's PLL of those classes - its reset value is the PLL
  through USBPRE, which is what the host/device controller was measured
  on ([usbfs.md](usbfs.md)).
- **The oscillator calibration registers of table 3-2** (HSE_CAL_CTRL,
  the five LSI32K ones): the table's own note applies them to the
  CH32V20x_D8W, which is another family.
- **PB1 above 72 MHz.** The bus is capped there - `pclk1_hz` halves
  HCLK above it - and the datasheet says 144; where the real ceiling
  lies is open, and the one measurement that bears on it is the USB
  controller's, which lives on PB1 and lost every packet of an
  enumeration with the bus undivided. The instruments to answer it are
  now on the chip (a timer on PB1 counted against the crystal, a
  USART's divisor against a known sender); what is missing is a build
  with the cap lifted, which this driver does not offer.
- **The Ethernet prescaler's effect.** `eth_prescaler()` writes the
  field on the one part of the family with a MAC, and the block itself
  is another chapter's.

Implemented but not bench-verified:

- **The HSE in bypass** (`ClockSource::external`): the board's pads
  carry a crystal, so an external clock into OSC_IN wants a signal
  generator or a second board's clock output on a wire.
- **The clock security system's FAILURE.** Arming it is measured;
  firing it wants the crystal killed on a running board, which is a
  wire cut or a shorted pad and not something a suite can stage.
- **The clock output's frequency on the CH32V303VCT6's board**, which
  has no strap from PA8 to a timer's pad: the multiplexer is read back
  there and the letter says it does not count. What would measure it is
  that strap (PA8 to PA6, as on the CH32V203C8T6's board).
- **`hse_in_low_power()`**, the CH32V203RB's bit: written and read
  back nowhere but a family compile; what would measure it is that
  part on a board.
- **`adc_duty_extended()` on the CH32V203.** The converter chapter's
  suite times a conversion with the bit both ways on the CH32V303VCT6
  and finds nothing moved ([adc.md](adc.md)); the same timing on a
  CH32V203 would measure it there.
- **ADC_DUTY_SEL on a die that has it.** `adc_duty_75()` asks the die,
  and the CH32V303VCT6 answered no; what would measure the 75 % duty is
  a CH32V303 of a lot whose penultimate sixth digit is not zero, under
  the converter suite's letter `o`.
- **Every part but the CH32V203C8 and the CH32V303VC.** The whole
  chapter compiles for all thirteen both ways the hardware prologue can
  be built (`brio check ch32vx03`), including the CH32V203RB's own PLL
  arithmetic - its 32 MHz oscillator divided by four or eight - and the
  two packages with no oscillator pad, which the family fixture refuses
  a crystal on. What would measure them is a board.
