# Clock - RCC, PWR (STM32G0)

> **PROVISIONAL.** Two roots of the tree are implemented and
> bench-verified as the static task - HSI16 through its divider, and
> HSI16 through the PLL to the part's 64 MHz ceiling - with the bus
> prescalers pinned at 1, the flash latency sequenced, and the
> peripheral clock enables and kernel-clock multiplexers exposed as
> resource verbs. Everything else the chapter offers is declared and
> refused. The list is in "Not covered yet".

Documents of record: RM0444 Rev 6 - RCC ch. 5 (the tree 5.2, the
registers 5.4), FLASH 3.3.4 (the latency table 13 and the ordering
rule), PWR 4.1.4 (voltage scaling) - and errata ES0548 Rev 3 item
2.2.4, a stated caveat. Drivers: `stm32g0/clock.hpp` (`Rcc` and the
`Clock<source, hz>` task), `stm32g0/flash.hpp` (`FlashWaitStates`,
`FlashAccel`), and `stm32g0/pwr.hpp` for the one thing this chapter
borrows from chapter 4 - `Pwr::range()`, because the latency table is
indexed by the voltage range. ONE CHAPTER, ONE OWNER: chapter 4 lives
in [pwr.md](pwr.md) and RCC_BDCR - LSE, RTCSEL, RTCEN, BDRST - lives in
[rtc.md](rtc.md), because that register is unreachable without the RTC
domain's own write gate and its choices are one-way. The family fixture is
`test/family_stm32g0/clock.cpp` plus three negatives under
`tools/check_stm32g0.sh`.

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
SWS as the witness that the switch took. Range 2 halves the ceilings
and caps SYSCLK at 16 MHz; this stratum runs in Range 1 only and
refuses to init in the other.

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
- `Clock<source, hz>` - `hz` (SYSCLK = HCLK), `pclk_hz` (= hz),
  `is_static`, the `hsidiv` / `pll` setting the rate needs, `init()`
  (false when a root did not report ready, the switch did not take,
  the latency did not land, or the part is not in Range 1).
- `Rcc` - `hsi_enable`/`hsi_ready`/`hsi_wait_ready`, `hsi_div` (code
  0..7 = divide by 2^code), `pll_enable`/`pll_ready`/`pll_wait`,
  `pll_configure(PllConfig)` (refused while the PLL runs or outside
  the limits), `sysclk_select`/`sysclk_status`/`sysclk_wait`
  (`SysclkSource`), `bus_prescalers_unity`, the enables `io_clock(port,
  on)`, `ahb_clock`/`apb1_clock`/`apb2_clock(mask, on)` with an
  `apb1_clock(mask)` readback, the multiplexer `kernel_clock(pos,
  code)`, `hsi_kernel_request(on)` (RCC_CR.HSIKERON), and the LSI root - `lsi_enable(on)`, `lsi_enabled()`,
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
  becoming a real second rate); MCO/MCO2; the CSS and LSECSS; the
  RCC interrupts; the peripheral RESET registers beyond the USART's
  own verb; the sleep-mode clock enables (IOPSMENR and friends).
- Range 2 (the low-power regulator range) as a TASK - the setter and
  its two ordering sequences now exist in [pwr.md](pwr.md), and what is
  missing here is a `Clock<>` that knows the Range 2 latency column.
- `DynamicClock` - the same deferral as on the SAM, for the same
  reason: which root SYSCLK takes at run time and who is told is a
  design decision, opened by a real consumer.
- HSI16 trimming (RCC_ICSCR) and its measurement against LSE through
  TIM14/16/17 (5.2.16) - the FREQM-style scale this board does not
  have yet.
- Flash: everything but the latency and the two accelerators (the
  FLASH campaign).

Implemented, not bench-verified: `ClockSource::internal` at any rate
but 16 MHz (the divider write under a running core), `pll_configure`'s
refusal while running, `FlashWaitStates::set`'s bounded wait on a
DECREASE (every init so far raised), `FlashAccel`'s setters.

The kernel-clock multiplexer is bench-driven for all four codes on
USART2 and on both LPUARTs ([usart.md](usart.md), [lpuart.md](lpuart.md)),
and CCIPR2's FDCAN field for its one reachable code
- including a console that kept talking at 115200 while its own clock
moved under it. HSIKERON is written, read back and slept on; what it
COSTS in current is the meter question this stratum keeps deferring.
