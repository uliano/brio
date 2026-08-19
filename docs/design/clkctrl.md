# CLKCTRL - the clock controller (AVR DA/DB)

> **PROVISIONAL.** This driver was not written from a systematic,
> exhaustive review of the data sheet chapter and the errata; it
> covers what the bench needed. The exhaustive pass (see "Not covered
> yet") is pending. Documents consulted: AVR128DB28/32/48/64 data sheet
> DS40002247B (CLKCTRL, electricals 39.10), errata DS80000915F (2.5.x:
> EXTS status bit and RUNSTDBY with external sources on rev A4, PLL
> items). Drivers: `avrdx/clock.hpp` (`Clock`, `DynamicClock`),
> `avrdx/delay.hpp`. Model: [clock.md](clock.md). Reference test:
> `clock_console` (dynamic clock under a running console, 24 -> 12 ->
> 2 MHz), every app for the static configuration.

## What the driver does today

The main clock (CLK_MAIN -> CLK_PER = CLK_CPU) from one of three
sources, through the main prescaler:

- `ClockSource::internal`: OSCHF at 1/2/3/4/8/12/16/20/24 MHz;
- `ClockSource::crystal`: the high-frequency crystal on PA0/PA1
  (XOSCHF, DB only; 4k-cycle start-up; frequency range selected from
  the rate);
- `ClockSource::external`: an external clock on PA0.
Prescaler `ClockDiv::div1..div64` (the twelve values of MCLKCTRLB).
An external source that fails to start leaves the device on OSCHF at
the SAME rate (hence the rule: the rate must be one OSCHF can produce)
and `init()` reports it. `DynamicClock` changes the prescaler at run
time and rebases the listed users first (see clock.md). The RTC's
clock (OSC32K / XOSC32K) is not touched here: `Ticker::init()` picks
it.

## Types and verbs

| Entity | Role |
|--------|------|
| `Clock<source, source_hz, div>` | static clock: `hz` constexpr, `init()` -> bool (running from the requested source), `is_static = true` |
| `DynamicClock<Boot, Users...>` | runtime clock over a static Boot: `init()`, `hz()`, `set<hz>()` / `set(hz)` -> bool, `can_run_at(hz)`, `rebases<U>` |
| `ClockSource`, `ClockDiv`, `clock_divisor()`, `oschf_frqsel()`, `div_for()` | the vocabulary and the silicon's tables |
| `delay_us(clock, us)`, `delay_us_runtime(cycles_per_us, us)`, `cycles_per_us(hz)` | the short-wait role ("at least") |
| `clock_hz(clock)`, `clock_follows<C, D>()`, `ClockUser` | `util/clock.hpp`, target-independent |

## How to use it

```cpp
using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;
int main() {
    const bool xtal = SysClock::init();        // first line
    Serial::init(clock, 460800);
    ...
}
```
A low-power fixed rate: `Clock<ClockSource::internal, 4'000'000>`.
Runtime: `DynamicClock<Boot, Serial, Adc<0>>` and `SysClock::set<4'000'000>()`
(clock.md). The board's `F_CPU` is not defined in this build.

## Bench findings

- `clock_console`: 24 -> 12 -> 2 MHz under the running console at
  115200, 1 MHz refused (USART needs >= 16 x baud).
- Errata 2.2.4 (write lost after a store to >= 64 followed by a
  SLPCTRL.CTRLA write) handled in `AvrPlatform::idle()` with the NOP.

## Not covered yet (the chapter offers it, the driver does not)

PLL (2x/3x, up to 48 MHz, TCD's clock), OSCHF auto-tuning from a 32k
crystal, clock failure detection with automatic switching, XOSC32K
configuration (the 32k crystal - Ticker only detects it), OSC32K
control, the CLKOUT pin, RUNSTDBY choices per oscillator, the
electricals' start-up times in code, errata 2.5.x (rev A4) handling.
