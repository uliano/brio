# Clock - RCC, PWR (STM32G0)

> **PROVISIONAL.** Two roots of the tree are implemented and
> bench-verified - HSI16 through its divider, and HSI16 through the PLL
> to the part's 64 MHz ceiling - as static tasks in all three voltage
> regimes (Range 1, Range 2, low-power run) and as the members of a
> DYNAMIC clock that moves SYSCLK between them under the running
> program, with the bus prescalers pinned at 1, the flash latency
> sequenced, and the peripheral clock enables, the kernel-clock
> multiplexers and the clock output exposed as resource verbs.
> Everything else the chapter offers is declared and refused. The list
> is in "Not covered yet".

Documents of record: RM0444 Rev 6 - RCC ch. 5 (the tree 5.2, the
registers 5.4, the clock output 5.2.16), FLASH 3.3.4 (the latency table
13 and the ordering rule), PWR 4.1.4 (voltage scaling) and 4.3.2
(low-power run) - and errata ES0548 Rev 3 item 2.2.4, a stated caveat.
Drivers: `stm32g0/clock.hpp` (`Rcc`, the `Clock<source, hz, regime>`
task and `DynamicClock<Rates<...>, Users...>`), `stm32g0/flash.hpp`
(`FlashWaitStates`, `FlashAccel`), and `stm32g0/pwr.hpp` for what this
chapter borrows from chapter 4 - `Pwr::range()`, because the latency
table is indexed by the voltage range, and the range and low-power-run
setters a rate's own `init()` sequences. ONE CHAPTER, ONE OWNER: chapter
4 lives in [pwr.md](pwr.md) and RCC_BDCR - LSE, RTCSEL, RTCEN, BDRST -
lives in [rtc.md](rtc.md), because that register is unreachable without
the RTC domain's own write gate and its choices are one-way. The family
fixture is `test/family_stm32g0/clock.cpp` plus eight negatives under
`tools/check_stm32g0.sh`; the bench suite is `test_stm32_clock`.

## What the silicon does

**The third clock model brio meets.** The AVR has one prescaler on one
main clock; the SAM has a generic-clock generator per peripheral; this
family has ONE SYSCLK selected from five roots (HSISYS, HSE, PLLRCLK,
LSI, LSE), two SHARED PRESCALERS below it (HPRE for HCLK = the AHB
and the core, PPRE for PCLK = the APB), an ENABLE BIT per peripheral
that gates its bus clock (RCC_IOPENR/AHBENR/APBENR1/APBENR2), and a
KERNEL-CLOCK multiplexer for the few peripherals that may run off
something other than their bus (USART1..3, LPUARTs, I2C1, ADC,
LPTIMs, RTC - RCC_CCIPR; the I2S, USB and FDCAN - RCC_CCIPR2, a SECOND
register the smaller headers do not even declare as a struct member).
Three of those multiplexers are DRIVEN: the USART's, by
`stm32g0/usart.hpp`, BOTH LPTIMs', by `stm32g0/lptim.hpp`, and the
FDCAN's, by `stm32g0/fdcan.hpp` - which is where a peripheral's own
multiplexer belongs, since the block that counts the clock is the one
that knows what it is worth. The LPTIM's four codes (PCLK, LSI, HSI16, LSE) are
the same for both instances on every part of this pack, and which of
them survives a Stop is the low-power timer's own chapter
([lptim.md](lptim.md)). What crosses the util clock contract is
unchanged: `clock_hz(clock)` is SYSCLK = HCLK, and this first task
pins HPRE and PPRE at 1 so PCLK == HCLK == `hz`; `Clock::pclk_hz` is
stated beside `hz` so a driver on APB asks for the rate that is its
own, and the day the prescalers move nothing above the driver changes.

**Out of reset the part runs HSI16 undivided** - 16 MHz on HSISYS,
Range 1, zero wait states - and the PLL, HSE, LSI, LSE and HSI48 are
off. A debugger session or a bootloader can leave anything behind, so
`Clock::init()` re-states every step it depends on.

**A clockless peripheral does not answer** (5.2.17): with its enable
bit clear, reads of a peripheral's registers are "not effective" and
writes are dropped, silently. GPIO ports are such peripherals (the AVR
and the SAM have no such gate on PORT), and so is PWR - which is why
every configuring pin verb opens its port's clock first, and why
`Clock::init()` opens PWR's before reading the voltage range (the
bench read the correct reset value through the closed gate once; that
is luck, not a contract). The enable takes two clock cycles to act;
every enable verb reads its register back, which is the stall that
covers it.

**The flash latency must lead a rise and follow a fall** (3.3.4, table
13): at Range 1 HCLK <= 24 MHz wants 0 wait states, <= 48 wants 1,
<= 64 wants 2; the new LATENCY is in force only when it READS BACK,
and the sequence is latency, then SW, then (optionally) HPRE, with
SWS as the witness that the switch took. Range 2 caps SYSCLK at 16 MHz
and wants ONE wait state above 8 MHz where Range 1 wants none - the
table's second column, which a Range 2 rate takes inside its own
`init()`. A Range 1 rate REFUSES a core it finds in Range 2 rather than
raising it (a range change is the orderer's sequence, `DynamicClock`'s
or the program's); a Range 2 rate lowers the range LAST, after the root
and the wait states (4.1.4's falling order), and a low-power-run rate
sets LPR last of all, at the 2 MHz 4.3.2 wants - having first LEFT
low-power run if a previous life was in it, because every step of that
sequence is priced for the main regulator.

**A rate is a tuple and the dynamic clock is a pack.** On the AVR a
dynamic clock's rate is the boot rate over one prescaler; here it is
(the root and its rate, the VCORE range, the regulator) - 64 MHz on the
PLL in Range 1, 16 MHz on HSISYS in Range 2, 2 MHz on HSISYS/8 on the
low-power regulator - and the reachable rates come from two disjoint
families with no single prescaler to index. So `DynamicClock<Rates<R0,
R1, ...>, Users...>` names its discrete set as an explicit pack of static
`Clock` tasks (R0 the boot rate), and a switch is DIRECTION-AWARE around
the regulator: rising, low-power run is left (REGLPF clear) and the
range raised (VOSF clear) BEFORE the fan-out and the rate's `init()`
(wait states up, then the root); falling, the fan-out and the rate's
`init()` go first (the root, then wait states down) and the range and
LPR follow inside it. A Stop drops SYSCLK to HSISYS with the PLL off and
HSIDIV and LPR kept (pwr.md fact 4); `restore()` re-runs the CURRENT
rate's `init()` with no fan-out, or finds it in force and does nothing.
The design and its reasons: [../design/clock.md](../design/clock.md).

**HSISYS is not left divided behind a PLL rate.** A PLL rate takes
HSI16 undivided and has no reason to touch HSIDIV, so a program climbing
from HSISYS/8 to the PLL would keep the divider - harmless to the rate
(measured: the CPU at 64 MHz on the crystal's scale with HSIDIV still
3) and NOT harmless to what a Stop lands on: HSISYS at 2 MHz instead of
16, and ES0548 2.2.4's wake hazard with it. The dynamic clock writes
HSIDIV back to 0 after every switch onto the PLL.

**The PLL is configured only while stopped** (5.2.4): PLLON clear,
PLLRDY clear, then PLLCFGR, then PLLON, then PLLRDY - and PLLRCLK
cannot be selected as SYSCLK until it is ready (5.2.7). The ratio
limits (5.4.4): input after /M in 2.66..16 MHz, VCO in 96..344 MHz, N
in 8..86, R in 2..8, PLLRCLK <= 64 MHz. The exact ratio for a rate is
searched at COMPILE TIME (`pll_config_for`), deterministically: the
smallest M first, then R, then N - 64 MHz is M 1 / N 8 / R 2, VCO
128 MHz; a rate no exact ratio reaches is a compile error naming the
rule, never an approximation.

**HSIDIV is not free of consequences** (ES0548 2.2.4, no workaround):
with the divider at anything but 1 the part cannot enter Stop when
SYSCLK is HSE, and clock-request-capable peripherals cannot wake it
from Stop. A divided `internal` rate is legal here and stated as a
caveat for the sleep site. THIS HALF OF THE ERRATUM IS NOW MEASURED and
it is real: a USART on HSI16 with its wake armed does NOT come out of
Stop 0 at HSIDIV = /4, where the RTC does (usart.md, pwr.md).

**The KERNEL clocks are the other half of RCC_CCIPR and they are what a
serial port really divides.** USARTnSEL picks PCLK / SYSCLK / HSI16 /
LSE for USART1..3 and for both LPUARTs (5.4.21), and a port on HSI16 or
LSE does not move when SYSCLK does - which is what makes `rebase()` a
no-op for it, on purpose. `kernel_clock(pos, code)` is the one verb, and
each peripheral publishes its own field position from the reserve.

**WHO FOLLOWS A SYSCLK CHANGE, and who refuses one** - the inventory
the build enforces through `clock_follows` (a driver initialized with a
dynamic clock must be among its users, or it does not compile): the
`Uart` FOLLOWS (a `ClockUser` whose `rebase` drains and reloads BRR,
folded to nothing for a port on HSI16 or LSE - which is how a console
survives every switch untouched); the SysTick `BasicTicker` and
`SysTickCounter` FOLLOW (a new reload; the ticker restarts its period
and loses the phase of the tick in progress - under a tick, late, never
early - one more reason a rescaling program is a tickless one, whose
timebase on the crystal never asks); the `Adc` FOLLOWS in a PCLK mode
(the next larger division when the config's own would put fADC over
35 MHz, a pure function of config and rate; a no-op in the asynchronous
mode); the timers REFUSE (`tim_clock_hz` asserts a static clock: a
timer's periods, prescalers and captures are all in PCLK cycles and no
`rebase` can keep them - a rescaling program keeps its timers on what
does not move, the LPTIM on LSE and the RTC); `SyncHost`, `IrdaLink`
and `Smartcard` REFUSE (no `rebase`); the FDCAN and the window watchdog
take the bus rate as a number and keep it; the LPTIM, the RTC and the
IWDG never see SYSCLK at all. `delay_us` dispatches on the rate index
into a per-rate table built at compile time - no division at wait time,
as on the AVR ([../armv6m/README.md](../armv6m/README.md)).

**The clock output reaches the timers with no pad.** RCC_CFGR.MCOSEL /
MCOPRE put one of the tree's clocks, prescaled by a power of two, on a
SIGNAL that TIM2's ETRSEL (TIM2's alone: TIM3/TIM4's name the
comparators only, 22.4.26..27) and TIM14/16/17's TISEL name among their
sources - so a timer can count an internal clock it is not on: HSI16/64
into TIM2's ETR is a 4 us wall that does not move with SYSCLK, and it
is what the dynamic clock's own suite measures every switch on; HSI16/8
into the same ETR is a clock that runs ONLY WHILE THE PART IS AWAKE,
which is how a Stop is told from a Sleep and how the tickless timebase
priced its lap wakes ([tim.md](tim.md) for the timer half,
[platform.md](platform.md) for the lap).

**RCC_CCIPR2 IS A SECOND REGISTER AND IT NEEDED A SECOND VERB.** The
I2S, USB and FDCAN selects live there (5.4.22) and not in the CCIPR, and
the register is a STRUCT MEMBER only the G0B1/G0C1 header declares - so
the reserve hands back a pointer to it (`rcc_ccipr2()`, null elsewhere,
the `flash_ecc2r()` precedent) and `kernel_clock2(pos, code)` returns
false with nothing written on a part that has none. That is what lets
this file compile unchanged on every header of the pack while the
FDCAN's own select is driven from `fdcan.hpp` ([fdcan.md](fdcan.md)).
Its other two codes - PLLQCLK and HSE - are refused by that driver,
because nothing here builds either.

**RCC_CR.HSIKERON is NOT the same mechanism as a peripheral's clock
REQUEST**, and the difference matters wherever Stop does. 33.5.21: a
USART whose kernel clock is gated in Stop asks for it back on the
falling edge of its RX line and releases it again if the wake-up event
is not verified - the oscillator starts ON DEMAND and only for as long
as the frame lasts. HSIKERON instead keeps HSI16 running
unconditionally, which costs current and buys the start-up time back. A
wake from Stop works with the request alone; the bit is the escape for a
consumer that cannot afford the start-up, and the way to MEASURE the
difference. (It is also NOT a way round 2.2.4 - measured, usart.md.)

## Types and verbs

- `ClockSource` - `internal` (HSISYS), `pll` (HSI16 x PLL, R output);
  `crystal`, `external`, `lsi`, `lse` declared and refused.
- `PowerRegime` - `range1` (the reset state, every rate to 64 MHz),
  `range2` (SYSCLK <= 16 MHz, one more wait state at the top of its
  column), `low_power_run` (Range 2 and PWR_CR1.LPR, SYSCLK <= 2 MHz);
  `flash_wait_states_for(regime, hz)` is table 13 by column.
- `Clock<source, hz, regime = range1>` - `hz` (SYSCLK = HCLK),
  `pclk_hz` (= hz), `is_static`, `power_regime`, `sysclk_source` (the
  SWS value the root reports), `wait_states`, the `hsidiv` / `pll`
  setting the rate needs, `init()` (false when a root did not report
  ready, the switch did not take, the latency did not land, a regulator
  wait ran out, or - for a Range 1 rate - the part is not in Range 1).
  A Range 2 rate above 16 MHz and a low-power-run rate above 2 MHz are
  compile errors.
- `Rates<R0, R1, ...>` - the pack, `R0` the boot rate.
- `DynamicClock<Rates<...>, Users...>` - `is_static` false, `hz()`,
  `pclk_hz()`, the discrete-rate surface `rate_count` / `rate_hz(i)` /
  `rate_index()` / `rate_regime(i)` / `rate_source(i)` /
  `power_regime()`, `rebases<U>`, `index_of(hz)` / `can_run_at(hz)`,
  `init()` (the boot rate, no fan-out), `set<hz>()` (a rate outside the
  pack does not compile) / `set(hz)` (false, nothing changed) - the
  FIRST rate at that hz - and `set_index<i>()` / `set_index(i)` to name
  one exactly; each fans the new rate out to the users in list order and
  then runs the rate's `init()` inside the direction-aware ladder;
  `restore()` (the current rate after a Stop, no fan-out; nothing done
  when SWS already reports its root, or while a `set()` is in progress
  - legal from an ISR); `switching()`.
- `Rcc` - `hsi_enable`/`hsi_ready`/`hsi_wait_ready`, `hsi_div` (code
  0..7 = divide by 2^code), `pll_enable`/`pll_ready`/`pll_wait`,
  `pll_configure(PllConfig)` (refused while the PLL runs or outside
  the limits), `sysclk_select`/`sysclk_status`/`sysclk_wait`
  (`SysclkSource`), `bus_prescalers_unity`, the enables `io_clock(port,
  on)`, `ahb_clock`/`apb1_clock`/`apb2_clock(mask, on)` with an
  `apb1_clock(mask)` readback, the multiplexer `kernel_clock(pos,
  code)`, `hsi_kernel_request(on)` (RCC_CR.HSIKERON), the clock output
  `mco(source_code, log2_div)` (the `mco_*_code` constants for the
  codes every header shares; a code past the header's field is refused)
  / `mco_off()`, and the LSI root - `lsi_enable(on)`, `lsi_enabled()`,
  `lsi_ready()`, `lsi_wait_ready()`.
- **LSI lives in RCC_CSR, a register with two owners.** Bits 1..0 are
  the oscillator's and belong here; bits 31..23 are the reset flags and
  RMVF, and belong to [reset.md](reset.md). Each side
  read-modify-writes and neither touches the other's bits. LSI is not
  offered as a SYSCLK root (`ClockSource::lsi` is refused); it exists
  because the IWDG and the RTC are clocked from it. Two facts worth
  carrying: 5.2.14 says STARTING THE IWDG forces LSI on whatever LSION
  says, and 5.4.24 says LSIRDY may stand with LSION clear whenever the
  IWDG, the RTC or the CSS on LSE asks - on the bench board the RTC
  does exactly that (RCC_BDCR reads 0x8200: RTCEN set, RTCSEL = LSI,
  and the RTC domain is not reset by a system reset), so LSIRDY is no
  witness for a running watchdog. LSI itself measures **32586 Hz** on
  this die, weighed by a TIM16 capture against the core ([rtc.md](rtc.md))
  and agreeing with the 32536 Hz that [reset.md](reset.md) derived from
  an IWDG time-out by a wholly different route.
- `PllConfig {m, n, r}`, `pll_config_valid`, `pll_output_hz`,
  `pll_config_for(hz)`, `hsidiv_for(hz)` - constexpr, fixture-pinned.
- `FlashWaitStates` - `get`, `set(ws)` (waits for the readback, refuses
  > 2), `for_hz(hz)` (Range 1), `for_hz_range2(hz)` (declared);
  `FlashAccel` - `prefetch`/`instruction_cache` readback and setters;
  `flash_size_kb()`.

## How to use it

The 64 MHz road, the default of every kernel app:

```cpp
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;
const bool ok = SysClock::init();   // first thing in main()
```

The boot rate re-stated (nothing moves, everything is checked):

```cpp
using SysClock = brio::Clock<brio::ClockSource::internal, 16'000'000>;
```

A slow internal rate (2 MHz, HSIDIV = 3 - mind ES0548 2.2.4):

```cpp
using SysClock = brio::Clock<brio::ClockSource::internal, 2'000'000>;
```

The same rate on the low-power regulator, as a static boot clock (the
whole of 4.3.2's entry, and its exit first if a previous life was there):

```cpp
using SysClock = brio::Clock<brio::ClockSource::internal, 2'000'000,
                             brio::PowerRegime::low_power_run>;
```

A program that moves between the three - the console on HSI16 so its
divisor never moves, the kernel timebase on the crystal so kernel time
never moves, everything on PCLK listed as a user:

```cpp
using Fast = brio::Clock<brio::ClockSource::pll, 64'000'000>;
using Mid  = brio::Clock<brio::ClockSource::internal, 16'000'000, brio::PowerRegime::range2>;
using Slow = brio::Clock<brio::ClockSource::internal, 2'000'000, brio::PowerRegime::low_power_run>;
using SysClock = brio::DynamicClock<brio::Rates<Fast, Mid, Slow>,
                                    brio::SysTickCounter, Serial, Link, brio::Adc>;
constexpr SysClock clock;
using P = brio::Stm32g0Platform<brio::LptimTicker<>>;
using Site = brio::Stm32g0SleepSite<SysClock, brio::LptimTicker<>>;

SysClock::init();                    // Fast, the pack's first
Serial::init(clock, 115200);         // every user asks the same tag
...
SysClock::set<2'000'000>();          // the users rebased, then Slow: LPR last
SysClock::set_index<0>();            // LPR left, Range 1, then the PLL
```

After a Stop the plain site's `disarm()` calls `SysClock::restore()` -
at the wake convention, which is AFTER the AO the deadline was for has
run, on HSISYS at 16 MHz with the PLL off. A program that wants its rate
back before any AO runs calls `restore()` from the wake's own ISR (the
suite binds it beside the timebase's `isr()`); with no Stop in the way
that is one register read.

## Bench findings

On the Nucleo-G0B1RE (silicon revision Z), read over SWD with the
core halted after `Clock<pll, 64 MHz>::init()`: RCC_CR 0x03000500
(PLLON + PLLRDY, HSION + HSIRDY), RCC_CFGR 0x12 (SW and SWS both
PLLRCLK), RCC_PLLCFGR 0x30000802 (R 2, REN, N 8, M 1, source HSI16),
FLASH_ACR latency 2, PWR_CR1 0x208 (Range 1); SysTick's reload
63999 and the console's baud 115107 = 64e6/556 are the same 64 MHz
seen from two other registers, and the kernel tick is +0.24 % against
the PC's clock over ten seconds (HSI16's 1 %). The raw-register probe
app, with no brio code in the loop, reaches the same state with the
same sequence (its blink at 64 MHz is a 4x faster blink than at 16).

**The dynamic clock on silicon** (`test_stm32_clock`, ten letters, 42
verdicts, 42/42 three times, on the tickless platform with the console
on HSI16 and USART1 as a single-wire loop on PCLK as the rebased user;
TIM2 on MCO = HSI16/64 as the wall, weighed at 250 kHz within HSI16's
1 % of the crystal):

- Every rung reads back as its type claims - SWS, HSIDIV, PLLON, VOS,
  LPR, REGLPF, LATENCY - and the CPU is at the rate the type claims on
  the crystal's scale: 500 waits of `delay_us(999)` take 498..501 ms at
  64 and 16 MHz and 515 at 2 MHz, the excess being the calls' own
  cycles, 32 times dearer there. The loop-back is byte-exact at every
  rung (64 of 64; BRR 0x8B at 16 MHz, 0x11 at 2), and 72 switches round the
  ladder with 16 bytes exchanged at each landed with no refusal, no
  wrong state and no bad byte.
- **The four steps the design named as unbenched are closed**: HSIDIV
  written under a running HSISYS core (0 -> 3 at 16 MHz, the program
  going on at 2), a wait-state DECREASE (2 -> 1 after the fall to 16,
  1 -> 0 after the fall to 2, read back), Range 2 with its own column
  (LATENCY 1 at 16 MHz where Range 1 wants 0), and the PLL's refusal
  while running (`pll_configure` false with PLLON, PLLCFGR untouched).
- **What a switch costs**, on the wall, the console drained first (the
  wall's 4 us quantum on top): 64 -> 16 MHz Range 2 in 48..52 us;
  16 -> 2 MHz low-power run 88..92 us; 2 MHz LPR -> 64 MHz 380..384 us
  (REGLPF, VOSF, the PLL lock, in that order); 64 -> 2 MHz 132 us;
  2 -> 16 MHz 260 us (leaving LPR at 2 MHz, where every polled wait is
  slow); 16 -> 64 MHz 76..80 us.
- **The ADC follows**: a converter on PCLK/1 brought up at 16 MHz reads
  VREFINT at 16, 64 (its division moved to /2 by the rebase, disabled
  and re-enabled - fADC 32 MHz where /1 would be 64), 2 MHz in low-power
  run (/1 again, fADC 2 MHz) and 16 again, VDDA 3310..3316 mV across the
  four; the asynchronous mode at 2 MHz LPR (HSI16/2) reads 3316 with its
  CKMODE untouched by a rise to 64 - the calibration factor holds across
  the clock changes.
- **Kernel time does not move**: a 50-tick periodic through ten switches
  in three seconds fired 61 times at exactly 50 ticks, none early, at
  all three rates.
- **A Stop at each end of the ladder**, with THE WALL AS THE WITNESS
  THAT THE STOP HAPPENED (TIM2 counts HSI16 only while HSI16 runs and
  its bus is clocked, i.e. only awake: a WFI that fell through as a
  Sleep - table 31's own escape - would count 75000 in 300 ms): a
  300-tick deadline through a Stop 1 at 64 MHz matures at 292 ms of RTC
  wall with 436 us awake in all, the PLL re-locked by `restore()` from
  the wake's ISR before the deadline's AO ran (one interrupt, one
  restore); at 2 MHz in low-power run THE PART WAKES IN LOW-POWER RUN
  WITH HSIDIV KEPT (4.3.6's own sentence) - LPR and REGLPF standing
  after the wake, SYSCLK on HSISYS/8, 7.8 ms awake (the round's own
  turns at 2 MHz) - and `restore()` finds nothing to do. And A STOP 0
  ARMED FROM LOW-POWER RUN STOPS THE CLOCKS LIKE ANY STOP (7.8 ms awake
  of 294, the deadline met, the part back in low-power run with HSIDIV
  kept) - 4.3.6 speaks of the main regulator and only warns about
  HSIDIV, 4.3.7 admits Stop 1 from LPR in so many words; whether the
  main regulator was on during it (a Stop 0) or the low-power one
  stayed (a Stop 1 in all but name) is not observable from any
  register, and only a meter would tell.
- **The SysTick ticker as a user**: SysTick handed from the
  interrupt-less counter to `BasicTicker` for one letter, its
  `rebase()` reloading it at every switch - 500..501 ticks per 500 ms
  of crystal at 16 MHz Range 2, 2 MHz low-power run, 16 MHz on the PLL
  and 64 MHz, never fast - then handed back with the handler silent
  again. (The suite lists BOTH SysTick writers as users for that
  letter, which no program would do; the two `rebase()` write the same
  reload and neither touches CTRL - the counter's first version
  reprogrammed CTRL and turned the ticker's interrupt off, which is why
  it no longer does.)
- **A PLL rate in Range 2**: 16 MHz as PLLRCLK from a VCO at 128 MHz
  (M 1 / N 8 / R 8, exactly table 47's Range 2 ceiling, the driver's
  own static_assert), VOS 2, one wait state; reached from 64 MHz on the
  PLL by a reconfiguration through HSISYS in 72..76 us, from 2 MHz in
  low-power run in 304..312 us, left for it in 92..96 us and for 64 MHz
  in 88..92, the loop exact at every landing, the CPU at 16 MHz on the
  crystal's scale.
- **HSIDIV left behind a PLL rate**, the first version's finding: with
  no HSIDIV write in the PLL rate's `init()`, every rise from 2 MHz
  reached 64 MHz on the crystal's scale with HSIDIV still 3 - which is
  what the dynamic clock's write after every switch onto the PLL now
  prevents (above).
- **`delay_us` at every rung**, on the wall: 20/100/500/900 us served
  at 20..900 (+0..4) at 64 MHz, +4..8 at 16 MHz, +32..40 at 2 MHz - the
  poll's own cost, 32 times the 64 MHz one - never early, the
  millisecond cap refused at every rate.

## Not covered yet

Driver gaps:
- The other roots as SYSCLK: HSE (crystal, or the ST-LINK's MCO in
  bypass through the Nucleo's solder bridges), LSI, LSE, and HSI48
  (the G0B1/G0C1's USB clock, with its CRS); the PLL's P and Q
  outputs and its HSE input. LSE now RUNS on this board and is
  measured (32703 Hz against the core - [rtc.md](rtc.md)); what is
  missing is only the path that would make it SYSCLK, and the day it is
  built the task must ASK `RtcDomain` for a running crystal rather than
  start one behind the RTC's back.
- HPRE and PPRE other than 1 (a bus-dividing task, and `pclk_hz`
  becoming a real second rate); MCO2 and MCO on a pad; the CSS and
  LSECSS; the RCC interrupts; the peripheral RESET registers beyond
  the USART's own verb; the sleep-mode clock enables (IOPSMENR and
  friends).
- The dynamic clock's fan-out for the timers (a `rebase` that keeps
  periods across a PSC change is not promisable at the prescaler's
  granularity - refused, stated) and for the USART personalities
  without one (`SyncHost`, `IrdaLink`, `Smartcard` - refused); a
  power-manager AO that asks the bus AOs before a switch
  ([../design/clock.md](../design/clock.md), the caller picks the
  moment); what a rate COSTS in current (the meter question, the energy
  experiment's).
- HSI16 trimming (RCC_ICSCR) and its measurement against LSE through
  TIM14/16/17 (5.2.16) - the FREQM-style scale this board does not
  have yet.
- Flash: everything but the latency and the two accelerators (the
  FLASH campaign).

Implemented, not bench-verified: `FlashAccel`'s setters; a `BasicTicker`
program (the SysTick platform) rescaling under a kernel - the ticker's
`rebase` is measured as a user, the tick's lateness across a switch
(under one tick, by construction) is not.

The kernel-clock multiplexer is bench-driven for all four codes on
USART2 and on both LPUARTs ([usart.md](usart.md), [lpuart.md](lpuart.md)),
and CCIPR2's FDCAN field for its one reachable code
- including a console that kept talking at 115200 while its own clock
moved under it. HSIKERON is written, read back and slept on; what it
COSTS in current is the meter question this stratum keeps deferring.
