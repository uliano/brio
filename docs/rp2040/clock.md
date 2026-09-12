# Clock (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 2.15
(clocks: the generators, the multiplexers, the switching sequences of
2.15.3.2, the enables, resus), 2.16 (XOSC), 2.17 (ROSC), 2.18 (PLL:
the constraints of 2.18.2, the configuration of 2.18.3); Appendix B
(E7, E10). The driver: `brio/rp2040/clock.hpp`. The reference suite:
`test_rp2040_clock` (every rate counted by the chip's own frequency
counter, the switches timed by the system timer, the drivers on
clk_sys through every switch, and two letters over a clock wire
between two boards).

## What the silicon does

The FOURTH clock model brio meets. There is no bus prescaler and no
enable bit per peripheral: one clk_sys feeds the cores, the fabric,
the memories and every peripheral's bus interface; a separate
clk_peri feeds the UARTs and SPIs, so that their bit rates survive a
change of clk_sys (the datasheet's own reason for it); clk_usb,
clk_adc and clk_rtc have generators of their own; clk_ref is the
always-running reference the watchdog's tick and the frequency
counter use. Each generator is an auxiliary multiplexer over the
chip's sources - the crystal oscillator, the ring oscillator, the two
PLLs, two GPIO clock inputs - and a divider (integer plus a fraction
that dithers); clk_ref and clk_sys have a GLITCHLESS multiplexer in
front, because they must never stop, and its SELECTED register says
which source is in force. The sequences of 2.15.3.2 follow: a
glitchless generator's aux select is changed only with the glitchless
mux parked elsewhere; a generator without one is stopped, changed and
restarted.

At power-up the chip runs on the RING OSCILLATOR - clk_ref and
clk_sys at about 6.5 MHz, a rate that is neither exact nor stable -
with the crystal off, both PLLs in reset and clk_peri disabled. The
crystal oscillator takes 1..15 MHz and needs a startup delay in units
of 256 crystal cycles before STATUS.STABLE rises (47 for 12 MHz and
1 ms). The system PLL multiplies the crystal into a VCO of 750..1600
MHz through FBDIV 16..320 and divides it by two post dividers of
1..7; clk_sys tops out at 133 MHz. The datasheet's advice is the
highest VCO for the least jitter and the larger post divider first
for the least power. Erratum E7: the COUNT registers of both
oscillators are unreliable; E10: ROSC's BADWRITE is.

## Types and verbs

- `ClockSource`: `crystal` and `pll` built; `internal` (the ROSC),
  `external` (a clock into XIN) and `gpin` declared and refused - the
  ring oscillator's rate is not a truth `hz` could state.
- `Clock<source, hz, crystal_hz = 12'000'000, peri = PeriSource::sys>`
  - the static main clock, the ONE truth: `hz` = clk_sys, `pclk_hz` =
  clk_peri (clk_sys undivided, or the crystal under
  `PeriSource::crystal`), `pll` (the setting found), `startup_delay`;
  `init()` runs the whole sequence and answers false when the crystal
  did not start, the PLL did not lock or a switch did not take,
  leaving the tree on clk_ref. Called on a running tree it is THE RATE
  SWITCH: the stable crystal is kept, clk_sys parks on clk_ref while
  the PLL re-locks, and the drivers on clk_sys (the ticker, a UART on
  `PeriSource::sys`) are owed a `rebase(hz)` - a UART on a crystal-fed
  clk_peri is owed nothing. `count_hz(what)` counts a source against
  the crystal.
- `pll_config_for(ref_hz, out_hz)` - the exact ratio at compile time
  under 2.18.2's constraints, REFDIV 1, the highest VCO, the larger
  post divider first; a rate with no exact ratio is a compile error.
  `xosc_startup_delay(crystal_hz, settle_us)`.
- `Xosc` (init with the delay - a stable crystal is kept -, `stable`,
  `enabled`, `startup_delay`, `stop`), `PllSys` and `PllUsb` (one
  `PllBlock` at two addresses: init with a `PllConfig`, `locked`,
  `config` read back, `stop`), `Rosc` (`running`, `stable`, `start`,
  `stop`, `dormant` - the keyword, each waiting for STABLE after the
  wake), `Clocks` (`ref_select`, `sys_from_ref`, `sys_from_aux`,
  `sys_source`, the two dividers and `sys_divider()` read back,
  `peri_select`, `peri_enabled`, `peri_source`, clk_adc's
  `adc_select(aux, div)`, `adc_stop`, `adc_enabled`, `adc_source`, and
  clk_rtc's `rtc_select(aux, div_int, div_frac)`, `rtc_stop`,
  `rtc_enabled`, `rtc_source`, `rtc_divider256` - the same
  stop-select-start as clk_peri, generators with an aux mux alone;
  the top-level gates `sleep_enables` / `wake_enables` / `enabled` on
  a `SleepClocks` and the named sets, [sleep.md](sleep.md)) with
  `RefSource`, `SysAux`, `PeriAux`, `AdcAux`, `RtcAux`.
- `FreqCounter::count_hz(source, ref_hz, interval = 15)` - a
  `CountSource` (every root and generator) counted against clk_ref
  over 2^interval microseconds, in hertz; nullopt when the source
  died mid-count or the count never finished.
- `ClockOut<n>` (n 0..3 on GP21, GP23, GP24, GP25): `init(GpoutSource,
  div_int, div_frac)`, `stop`, `enabled` - a source through the
  generator's divider onto the pin; `ClockIn<n>` (n 0..1 on GP20,
  GP22): `init`, `release`, `count_source` - the pin as a source the
  generators and the counter name.

## How to use it

```cpp
using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;
const bool clock_ok = SysClock::init();    // first in main()
Serial::init(clock, 115200);               // divides clk_peri = 125 MHz
```

The crystal alone, for a program that wants 12 MHz and no PLL:

```cpp
using SysClock = brio::Clock<brio::ClockSource::crystal, 12'000'000>;
```

A rate switch under a running program, the UARTs untouched because
clk_peri is on the crystal:

```cpp
using Fast = brio::Clock<brio::ClockSource::pll, 125'000'000, 12'000'000, brio::PeriSource::crystal>;
using Slow = brio::Clock<brio::ClockSource::pll, 48'000'000, 12'000'000, brio::PeriSource::crystal>;
Serial::init(Fast{}, 115200);      // divides clk_peri = 12 MHz, once
...
const bool ok = Slow::init();      // clk_sys to 48 MHz, ~200 us
brio::Ticker::rebase(Slow::hz);    // the ticker is on clk_sys
```

A clock on a wire, and the wire counted by another board:

```cpp
brio::ClockOut<0>::init(brio::GpoutSource::xosc, 12);      // 1 MHz on GP21
brio::ClockIn<0>::init();                                   // GP20 on the other board
const auto hz = SysClock::count_hz(brio::ClockIn<0>::count_source);
```

## Bench findings

- After `Clock<pll, 125 MHz>::init()` on the WeAct board: XOSC.STATUS
  reads STABLE and ENABLED with the delay at 47; PLL_SYS.CS reads
  LOCK with REFDIV 1, FBDIV 125 and the post dividers 6 and 2 (the
  datasheet's 125 MHz recipe); CLK_SYS_SELECTED reads the aux source
  and CLK_PERI_CTRL reads enabled on clk_sys - all read back over SWD
  with the program running.
- The UART's divisor computed from `pclk_hz` lands on the datasheet's
  own example (67 + 52/64 at 125 MHz for 115200), and the console
  passes both ways at that divisor with zero framing errors: clk_peri
  runs at the rate the PLL claims, to the resolution a UART frame
  gives (a few per cent).
- The PLL's ratio against the crystal is exact to 5 ppm: 200 SysTick
  periods of clk_sys at 125 MHz span 199999 us on the system timer,
  which counts the crystal's own microseconds
  (`test_rp2040_platform`).
- The clock suite, green on both boards, with the frequency counter
  as the measure (every rate within its 30 Hz grain of the claim):
  - the tree after init(): the crystal stable at delay 47, the PLL
    locked at 125 / 6 x 2, clk_sys on its aux undivided, clk_peri on
    clk_sys; xosc, clk_ref, pll_sys, clk_sys and clk_peri counted at
    their claims; the ring oscillator at 5.8 to 5.9 MHz on the two
    boards (the datasheet's "about 6.5");
  - THE SWITCHES: the PLL to the crystal alone 134 us, back 94 us,
    the PLL re-locked at another rate (48, 100, 133 MHz: 1440, 1500,
    1596 MHz VCOs, each at pll_config_for's ratio) about 200 us,
    the crystal kept throughout;
  - clk_peri on the crystal: the console re-divided once for 12 MHz
    and then untouched while clk_sys went 125 to 48 and back, and
    while clk_sys was divided by two - every line printed across the
    switches is the proof; clk_peri counted 12 MHz throughout;
  - `delay_us` at 12, 48, 100, 125 and 133 MHz: 900 us served as
    902..913 us and 100 us as 103..129 us - the excess is the
    bracket's own ~350 CPU cycles (two timer reads and the calls),
    dearest at 12 MHz; a whole tick refused at every rate;
  - the SysTick ticker rebased: 100 ticks span 100000 us (99992 at
    12 MHz) at every rate;
  - the ring oscillator stopped counts 0 Hz (not DIED: that flag is
    for a source that dies MID-count), restarted within 0.1 % of its
    rate before;
  - the PLL asked for a 192 MHz VCO, below the documented 750 MHz
    floor, LOCKS in about 75 us: the range is a promise of the
    datasheet, not a refusal of the silicon - `pll_config_for` is the
    guard, at compile time.
- THE TWO BOARDS' CRYSTALS: the crystal of one divided by 12 on GP21
  (`ClockOut<0>`), counted by the other on GP20 against its own
  crystal, reads 999968 Hz - the two crystals 32 ppm apart, the one
  external reference this bench has.

## Not covered yet

Driver gaps, each with its reason:

- The USB PLL and the clk_usb, clk_adc and clk_rtc generators: born
  with their consumers (the ADC, the RTC, a USB driver).
- A GPIO clock input as clk_sys or clk_ref (`SysAux::gpin0` is
  named, no task takes it): born with a board that brings its clock
  on a pin; the counter and the generators can already name it.
- Resus (2.15.5): a debugging aid the datasheet itself warns off in
  normal operation.
- A dynamic clock: this chip has no voltage side to a rate (one
  regulator setting serves the whole range) and clk_peri's
  independence makes a change cheap - built when a program wants to
  scale, in the STM32G0's `Rates<>` shape.
- The ring oscillator's configuration (its frequency range, drive
  stages and divider): never a clock truth, so never a `Clock`; start
  and stop are the verbs, the rest waits for a use.
- A runtime range check in `PllSys::init` (a VCO or a divider outside
  2.18.2): the task's ratio is found and checked at compile time, and
  the resource writes what it is given - a program composing its own
  `PllConfig` owns the constraints.

Implemented but not bench-verified, each with what would measure it:

- The crystal's absolute rate: the suites prove the PLL's ratio and
  the two crystals' ratio, never a crystal against a standard; a
  reference counter on GP21's output.
- The failure paths (a crystal that does not start, a PLL that does
  not lock): a board with the crystal removed, and a PLL setting the
  silicon refuses - the one tried below its VCO floor locked.
- `ClockOut`'s fractional divider and the outputs on GP23..GP25: a
  counter on those pins (GP25 is the boards' LED).
