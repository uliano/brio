# Clock (CH32X035)

The main clock of the CH32X035 series, where brio's clock model
([../design/clock.md](../design/clock.md)) meets its simplest silicon:
ONE root, the 48 MHz internal RC, and ONE divider between it and every
block on the chip. A static `Clock<internal, hz>` names a rate the root
divides into exactly and is the one truth every driver derives from; a
`DynamicClock` moves the divider at run time with its users rebased
first. `Rcc` beside them is the block itself - the root and its trim,
the divider as it stands, the clock output, the peripheral gates and
resets, and the reset flags.

Documents of record: the CH32X035 reference manual V1.8 (3.2 for the
reset sources, 3.3 and figure 3-2 for the tree, 3.3.2 for the HSI,
3.3.3.4 for the clock output, 3.4.1 to 3.4.9 for the registers, 20.3.1
for the flash's wait states) and the CH32X035/X033 datasheet V1.7
(table 3-9 for the HSI's accuracy and start-up, table 2-1 for the pad
the clock output needs). The header is
[brio/ch32x035/clock.hpp](../../brio/ch32x035/clock.hpp); the reference
suite is `test_x035_clock`.

## What the silicon does

- **ONE ROOT.** The HSI, a 48 MHz RC oscillator, is on out of reset and
  ready (HSION and HSIRDY both 1), and the manual recommends not turning
  it off (3.3.2); SYSCLK IS the HSI - there is no PLL, no crystal
  oscillator, no low-speed RC and no clock switch (3.3.1, 3.3.3.1). Its
  accuracy after the factory calibration is -1.7 to +1.6 per cent over 0
  to 70 C and -2.6 to +2.2 over -40 to 85 C, and it starts in 1.5 to 3.5
  us (datasheet table 3-9).
- **THE CALIBRATION AND THE TRIM.** HSICAL[7:0] is the factory's value,
  loaded at reset and read-only; HSITRIM[4:0] is the user's, SUPERIMPOSED
  on it, 16 at reset - "48MHz +-1%" - at "about 110KHz per step"
  (3.4.1).
- **ONE DIVIDER.** HCLK is SYSCLK through HPRE[3:0]: codes 0 to 7 divide
  by 1 to 8, codes 8 to 15 by 2, 4, 8, 16, 32, 64, 128 and 256 - the two
  halves overlapping on 2, 4 and 8 - and the reset value is 0101, /6, so
  the chip wakes at **8 MHz** (3.4.2). Every rate the chip can run at is
  48 MHz divided by one of those.
- **NO BUS PRESCALER.** Everything below HB runs at HCLK - the USARTs'
  divisors count HCLK (14.3), the timers and the rest the same - so a
  peripheral's rate is the clock's and nothing else, and `pclk_hz` is
  `hz`. What does not run at HCLK is the tree's other branches (figure
  3-2): SYSCLK/1024, some 47 kHz, for the auto-wakeup and the
  independent watchdog - whose start forces the HSI on for good
  (3.3.3.3) - the ADC's own divider, the USB clock, and SYSCLK/6 for the
  flash's programming interface.
- **THE FLASH'S WAIT STATES FOLLOW HCLK** (20.3.1): 0 up to 12 MHz, 1
  up to 24 MHz, 2 up to 48 MHz, the fourth code invalid - FLASH_ACTLR
  holds that field and nothing else.
- **A NOTE WITH NO BIT.** 3.4.2 closes HPRE's description with "when the
  prescaler factor of the HB clock source is greater than 1, the
  prefetch buffer must be turned on", and no register of the manual has
  a prefetch-buffer bit; WCH's own clock setup divides HCLK and touches
  nothing else. The driver does the same.
- **THE CLOCK OUTPUT** carries SYSCLK or the HSI (CFGR0.MCO = 100 or
  101; every other code, nothing) on **PB9** as an alternate push-pull
  output (3.3.3.4; the datasheet's pin table). PB9 is bonded on the
  LQFP64M, the LQFP48 and the two 28-pin packages, and on neither
  20-pin one.
- **THE GATES.** Three enable registers, one bit a block (3.4.5 to
  3.4.7): RCC_AHBPCENR for the DMA, SRAMEN (the SRAM's clock IN SLEEP,
  not the SRAM's clock), the USB host/device controller and the USB PD
  controller - its reset value 0x00021004, those last three ON; the
  PB2 register for AFIO, the three ports, the ADC, TIM1, SPI1 and USART1;
  the PB1 register for TIM2, TIM3, the window watchdog, USART2 to USART4,
  I2C1 and PWR. The reset registers mirror them (3.4.3, 3.4.4), and the
  HB one holds three lines alone - the USB host/device controller's, the
  PIOC's and the USB PD's (3.4.9): the DMA and the SRAM have none.
- **THE RESET FLAGS ACCUMULATE** in RCC_RSTSCKR until RMVF is written
  (3.4.8): LPWRRSTF, WWDGRSTF, IWDGRSTF, SFTRSTF, PORRSTF, PINRSTF and
  OPARSTF. The system reset has more sources than flags (3.2.2): a core
  deadlock, an OPA output going high, a USB PD hard reset (whose flag,
  the manual says, is the software reset's) and an ADC watchdog reset
  among them. THE MANUAL GIVES TWO RESET VALUES for the register: its
  register table (table 3-1) says 0x0C000000, PORRSTF and PINRSTF, and
  3.4.8's own bit table gives PINRSTF a reset value of 0. On the QFN20
  there is no reset pin at all.

## Types and verbs

Vocabulary: `ClockSource` (`internal`, the one root), `sysclk_max_hz`
(48 MHz), `hpre_divider(code)` and `hpre_for(src_hz, hz)` (the HPRE
code that divides exactly, the low code where two do, 0xFF where none
does), `Bus` (`hb`, `pb2`, `pb1` - where a gate and a reset line sit),
`McoSource` (`none`, `sysclk`, `hsi`), `clock_timeout_turns` (every
wait for HSIRDY is bounded).

`Rcc`, the block, monostate:

| verbs | what they do |
|---|---|
| `enable(bus, mask)`, `disable(bus, mask)`, `enabled(bus, mask)` | a peripheral's gate, by the `rcc_*` masks of device.hpp |
| `reset(bus, mask)` | a peripheral's reset line pulsed, its gate left alone; on `hb` the mask is RCC_AHBRSTR's |
| `hsi_on()`, `hsi_ready()`, `hsi_start()` | the root read, and asked for with a bounded wait (it is on out of reset; a program that stopped it gets it back) |
| `hsi_trim(t)`, `hsi_trim()`, `hsi_calibration()` | the user trim written and read, the factory value read |
| `hpre_code()`, `hpre(code)`, `hclk_hz()` | the divider as it stands, a raw store, and the rate the registers say |
| `mco(src)`, `mco()` | the output multiplexer alone - the pad is `Mco`'s |
| `reset_flags()`, `clear_reset_flags()` | RCC_RSTSCKR's flags read without disturbing them, and RMVF |

`Mco`, the clock output as a task: `has_pad` (PB9 bonded or not, from
the part table), `init(src)` - the pad claimed and the multiplexer set
in one verb, FALSE and nothing done on a package without the pad -
`off()` and `source()`.

`Clock<ClockSource::internal, hz>`, the static clock: `hz` (HCLK),
`pclk_hz` (the same), `sysclk_hz` (48 MHz), `hpre_code`, `is_static`.
A rate that is not 48 MHz divided exactly by an HPRE divider is a
COMPILE error, never a rounded one. `init()` asks for the HSI, raises
the wait states to what the faster of the current and the new rate
needs, moves HPRE, then sets the new rate's own wait states - false,
the tree untouched, if the HSI never reports ready; `restore()` is
`init()` again, for a program coming back from a sleep that stopped
the tree.

`DynamicClock<Boot, Users...>`, the runtime regime - the CH32V00x's
shape: `Boot` names the root undivided (`Clock<internal, 48'000'000>`,
enforced), `set<hz>()` and `set(hz)` name the NEW RATE - a rate no
divider reaches is a compile error or a false - and fan it out to the
`Users` (each a `ClockUser`, checked where the list is written) in list
order, synchronously, BEFORE the divider moves, so a user can drain
what it has in flight at the old rate; the wait states are raised
before a rise and lowered after a fall. `hz()`, `pclk_hz()`,
`can_run_at(hz)`, `rebases<U>`, and the discrete-rate surface a delay
table is indexed by: `rate_count` (16), `rate_hz(i)`, `rate_index()`.
`restore()` puts the CURRENT rate back after a sleep that stopped the
tree, telling the users nothing - their rate never changed.

## How to use it

The static clock, which is most programs:

```cpp
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

int main() {
    SysClock::init();                     // first: 48 MHz, two wait states
    Serial::init(clock, 115200);          // drivers ask the tag
    brio::Ticker::init(clock);
}
```

`Clock<ClockSource::internal, 8'000'000>` is the reset clock named out
loud; 16 MHz, 9.6 MHz or 187.5 kHz are rates too, and 20 MHz is a
compile error.

A clock that moves, with the ticker and a console as its users:

```cpp
using Boot = brio::Clock<brio::ClockSource::internal, 48'000'000>;
using SysClock = brio::DynamicClock<Boot, brio::Ticker, Serial>;
constexpr SysClock clock;

SysClock::init();                 // 48 MHz
SysClock::set<6'000'000>();       // the ticker and the port rebased, then HPRE /8
```

The clock output, where the package has PB9:

```cpp
if (!brio::Mco::init(brio::McoSource::hsi)) {
    // a 20-pin package: no pad for it
}
```

The reset causes at boot, before anything clears them:

```cpp
const uint32_t flags = brio::Rcc::reset_flags();
brio::Rcc::clear_reset_flags();
const bool watchdog = (flags & (brio::rcc_iwdgrstf | brio::rcc_wwdgrstf)) != 0u;
```

## Not covered yet

Driver gaps, each with its reason:

- **Stopping the HSI**: `hsi_on()` reads it and no verb clears HSION -
  it is the one root, the manual advises against it, and a program that
  wants the core stopped wants a sleep, whose chapter is not written
  here ([README.md](README.md)).
- **The branches this file does not set**: the ADC's divider (CLK_DIV,
  the ADC chapter's), the auto-wakeup's and the watchdog's SYSCLK/1024
  (their chapters'), the USB clock (the USB chapters').
- **Closing the USB blocks' gates at boot**: RCC_AHBPCENR has them open
  out of reset and nothing here closes them - born with the USB chapters,
  or with a current measurement that says what closing them saves.
- **A trim loop** that steers HSITRIM against an outside reference (a
  host-timed bracket, the USB's start of frame): born with its first
  user.

Implemented but not bench-verified, each with what would measure it:

- **The tree after `init()`**: the HSI on and ready, the trim at the
  centre, the factory calibration, HPRE at /1, two wait states, the
  clock output off, the gates the console opened, and RCC_AHBPCENR's
  reset value as the manual gives it: `test_x035_clock` letter a.
- **The ladder**, 48 -> 24 -> 16 -> 12 -> 9.6 -> 8 -> 6 -> 3 MHz and
  back up through a `DynamicClock` whose users are the ticker and the
  console itself: at each rung the code, the wait states, two hundred
  ticks exact, a 500 us wait served and a console line read clean -
  which is also the answer to the prefetch note, a divided HCLK with no
  prefetch bit to set: letter b.
- **The trim**, one step each way with the console still reading and
  the centre restored: letter c. How many hertz a step is wants a
  counter on the clock output, which the QFN20 does not bond.
- **The rate against the host's clock**, a bracket of ticks at 48 MHz
  and at the reset clock's 8 MHz for whoever times the two lines - the
  HSI's accuracy against table 3-9: letter d.
- **The clock output**: its source field written and read back for
  every source, and `init()` answering false on the QFN20: letter e.
  The waveform on PB9 wants a package that bonds it and a counter.
- **The gates and the reset lines**: every enable of the chapter opened,
  read back and closed as it was, a reset pulse leaving its gate alone:
  letter f.
- **The reset flags and the manual's two reset values of RCC_RSTSCKR**:
  what a boot finds, and a clear leaving nothing: `test_x035_platform`
  letter a - where a boot after the probe's reset reads the probe's
  reset and not a power-on's.
