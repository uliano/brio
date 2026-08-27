# Clock - OSCCTRL, GCLK, MCLK (SAM C21)

> **PROVISIONAL.** One path through the clock tree is implemented and
> bench-verified: OSC48M undivided into generator 0 into an undivided
> CPU clock, plus the peripheral-channel verb the SERCOM console
> already rides. Every other root of the tree - the crystal, the
> 32 kHz oscillators, the FDPLL - is vocabulary that refuses to
> compile, not code. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M - GCLK ch. 16,
MCLK ch. 17, OSCCTRL ch. 20, the flash wait-state table 45-41 - and
errata DS80000740S items 1.2.2 and 1.2.3, both encoded in code.
Driver: `samc/clock.hpp`. The family fixture is
`test/family_samc/clock.cpp` under `tools/check_samc.sh`.

## What the silicon does

**The tree, and where this driver stands in it.** Clock sources
(OSCCTRL and friends) feed nine **generic clock generators** (GCLK);
generator 0 is the main clock, divided by MCLK's CPUDIV into
CLK_CPU/CLK_AHB/CLK_APBx; the other generators feed **peripheral
channels** - each clocked peripheral names one generator through its
own channel (PCHCTRL), indexed by a per-instance constant from the
device header. Out of reset the device already runs: OSC48M divided
by 12 (4 MHz), generator 0 sourced from it and enabled, CPUDIV = 1.
That reset state is why reaching 48 MHz is one divider write - and
why this driver's init is mostly confirmation: it RE-STATES generator
0 and CPUDIV rather than trusting whatever a debugger or bootloader
left behind.

**Flash wait states bound the frequency** (table 45-41, the
conservative VDD > 2.7 V column): 19 MHz at 0 WS, 38 at 1, 64 at 2 -
and NVMCTRL 27.5.2 orders them adapted BEFORE a rise and after a
fall. They are NVMCTRL's register, but nothing may raise the
frequency without them, so the verb lives here until a samc nvm
driver takes it over.

**Synchronization is per-register and real.** The OSC48M divider
write crosses clock domains (OSC48MSYNCBUSY), each generator's
GENCTRL write has its SYNCBUSY bit, and a peripheral channel must be
DISABLED before its generator changes (PCHCTRL has no
synchronization of its own). Every wait in the driver is BOUNDED and
reported - a clock that never becomes ready is a false return, not a
hang.

**The two errata, as code.** 1.2.2: writing OSC48MDIV while the
oscillator runs UNREQUESTED wedges its sync bit - so init clears
ONDEMAND first. 1.2.3: a rare no-start on parts produced before
2025-01 - mitigated by exactly the same setting, ENABLE = 1 with
ONDEMAND = 0, which init writes and leaves.

**A generator source change is glitch-free on the fly** (16.6.2.6):
the old source is released only once the new one is ready - which is
what lets init re-state generator 0 while executing from it. The
same section's note for a REAL source change (ONDEMAND on the
outgoing source) is recorded in the driver comment for the day a
second source exists.

## Types and verbs

Resources (thin typed register views, `samc/clock.hpp`):

- **`FlashWaitStates`** - get/set, and `for_hz()` folding table 45-41
  at compile time.
- **`Osc48m`** - enable/on_demand/run_standby, ready and its bounded
  wait, the 1..16 output divider (ratio in, ratio out) with its sync
  wait, startup time.
- **`Gclk<n>`** (n < 9, refused otherwise) - `configure(GclkConfig)`
  writes the whole GENCTRL in one store and waits the sync:
  source, the divider in both DIVSEL regimes (the chapter's own two
  meanings of the field, kept explicit), improved duty, output to the
  GCLK_IO pad, run-standby.
- **`GclkChannel`** - `connect(channel, generator)` with the
  disable-first discipline, disconnect, the one-way WRTLOCK. The
  channel index is the device header's per-instance constant
  (`SERCOM5_GCLK_ID_CORE` and kin) - a fact of the peripheral, passed
  by its driver, never computed (SERCOM5 breaks any formula).
- **`Mclk`** - CPUDIV (one-hot, refused otherwise) and the AHB/APBx
  bus-clock masks, the bit being each driver's own header constant.

Task:

- **`Clock<ClockSource::internal, hz>`** - the static main clock:
  `hz` is the ONE compile-time truth every driver derives from (there
  is no F_CPU in this build). Any exact OSC48M ratio is accepted -
  48, 24, 16, 12, 9.6, 8, 6, 4.8, 4, 3.2, 3 MHz - and the fractional
  ratios are refused at compile time, because `Clock::hz` must not
  lie. `init()` orders wait states correctly around the change,
  applies the errata discipline, re-states generator 0 and CPUDIV,
  and returns whether the rate is really the one `hz` claims. The
  other `ClockSource` enumerators exist so the vocabulary is stable:
  choosing one is a compile error with an explanation, never a
  silently wrong clock.

## How to use it

```cpp
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

int main() {
    const bool ok = SysClock::init();   // false = the rate is NOT hz
    // a peripheral's driver wires its own core clock:
    //   Sercom<5>::core_clock(0);      // channel from ITS header constant
}
```

## Bench findings

- `init()` returns true on the bench chip, and the whole register
  state was verified over SWD with the CPU halted: OSC48MDIV = 0,
  OSC48MCTRL = ENABLE with ONDEMAND clear, GENCTRL0 = OSC48M + GENEN,
  CPUDIV = 1, RWS = 2 - every write landed as coded.
- The rate is right by consequence: the SERCOM baud generator
  programmed from `Clock::hz` produced a byte-exact 115200 console,
  and the SysTick reload from the same truth holds wall-clock time
  (see `platform.md` and `sercom.md`).

## Not covered yet

Driver gaps (vocabulary without code, each refused at compile time):
- XOSC (the board's 24 MHz crystal on PA14/PA15, deliberately
  unused), its clock-failure detection, and external-clock mode.
- OSC32K / XOSC32K / OSCULP32K as selectable sources; the RTC's clock.
- FDPLL96M.
- A `DynamicClock` for this target - and with it the ticker's
  ClockUser question (`samc/ticker.hpp` documents the caveat and
  refuses the combination mechanically).

Implemented but not bench-verified:
- Generators 1..8 and their dividers/IDC/output pads (the verbs are
  family-compiled; only generator 0 has run on silicon).
- `GclkChannel::lock`, `Osc48m::run_standby`, `Osc48m::startup`,
  rates other than 48 MHz (the arithmetic is compile-checked; only
  the undivided rate has run).
