# Clock - RCC, the regulator scale and the flash latency (STM32F4)

Documents of record: RM0090 Rev 22 ch. 7 (RCC for the F42x/F43x; ch. 6
is the F405 class's twin), 5.1.4 (the regulator's scales and
over-drive) and 3.5.1 (the wait states, table 12); RM0390 Rev 6 ch. 6,
5.1.4 and 3.4.1 (table 5) for the F446; RM0383 Rev 4 ch. 6, 5.1.4 and
3.4.1 (table 5) for the F411; the errata sheets' 2.2.7 ("Delay after
an RCC peripheral clock enabling", every enable verb here reads its
register back) and ES0206 2.2.12 (over-drive unavailable on silicon
revisions A and Y - the DISC1's part is revision 4/5/B, REV_ID 0x2003,
and over-drive is measured on it). Drivers: `stm32f4/clock.hpp` (`Rcc`,
`Clock<source, hz, hse_hz, hse_mode>`, `apb_hz`), `stm32f4/pwr.hpp`
(`Pwr`: the bus gate, `VoltageScale`, the over-drive pair),
`stm32f4/flash.hpp` (`FlashWaitStates`, `FlashAccel`, and the device
signature: `DeviceUid`, `flash_size_kbytes`, `DeviceIdcode`). The
family fixture is `test/family_stm32f4/clock.cpp` with the negatives
that refuse an unreachable PLL ratio, a rate above the part's ceiling,
a rate on a part whose ladder was not read, a divided HSI and an HSE
root without its rate. The reference suite is `test_stm32f4_platform`,
letter e.

## What the silicon does

**The STM32G0's model, with the prescalers no longer pinned.** One
SYSCLK from HSI (16 MHz), HSE (a crystal of 4..26 MHz or a bypassed
clock of 1..50 MHz) or the main PLL; shared prescalers below it - HPRE
for AHB, PPRE1 and PPRE2 for the two APBs - and an ENABLE BIT per
peripheral in AHB1ENR/AHB2ENR/AHB3ENR/APB1ENR/APB2ENR that gates its
bus clock (a peripheral whose bit is clear does not answer). What the
G0 stratum could do and this one cannot is run every bus at SYSCLK: the
APBs have CEILINGS - 45 and 90 MHz on the F42x/F43x and F446, 42 and 84
on the F405 class, 50 and 100 on the F411 (RM0090 7.3.3's cautions and
their twins) - and a core at 180 MHz cannot feed them undivided. So
`Clock::hz` is SYSCLK = HCLK (HPRE stays 1), and beside it the task
states `pclk1_hz` and `pclk2_hz`, each the largest power-of-two
division of HCLK under its bus's ceiling: 45 and 90 MHz at 180, 50 and
100 at 100, both 16 at the reset rate. A driver on an APB asks
`apb_hz(clock, on_apb2)` for the rate that is really its own - the
USART's divisor divides its bus clock, not SYSCLK - and what crosses
the util contract is unchanged: `clock_hz(clock)` is HCLK.

**The way up is a sequence the manuals spell out**, and the task
follows it literally (RM0090 5.1.4, 3.5.1, 7.3.3): the REGULATOR SCALE
(PWR_CR.VOS) for the target rate is written while the PLL is off and
SYSCLK is HSI - the reset state - and takes effect when the PLL turns
on; the PLL is configured and started; where the rate needs OVER-DRIVE
(above 168 MHz on the parts that have it) the pair is sequenced
between the PLL's start and the switch to it - ODEN then ODRDY, ODSWEN
then ODSWRDY, the system stalled during the switch, no peripheral clock
yet enabled; the FLASH LATENCY for the target HCLK is written and READ
BACK (3.5.1: "check that the new number of wait states is taken into
account"), the ART accelerator turned on; the APB prescalers written;
the PLL waited for; the switch, and SWS read back. Entering Stop
disables both the PLL and the over-drive - the power chapter's to
restore.

**The ladders are the part class's.** Scale 3 up to 120 MHz, scale 2 up
to 144 (168 in over-drive), scale 1 up to 168 (180 in over-drive) on
the F42x/F43x and the F446; scale 2 up to 144, scale 1 up to 168 with
ONE VOS bit and no over-drive on the F405 class; scale 3 up to 64,
scale 2 up to 84, scale 1 up to 100 and no over-drive on the F411. The
wait states at 2.7..3.6 V: one per 30 MHz band up to 5 at 150..180 on
the 180 MHz parts (RM0090 table 12), the F411's bands 30, 64, 90 and
100 MHz (RM0383 table 5). None of it is in the device header, so the
reserve keys it on the device-select define and knows it for these four
classes alone; five other classes are refused above the reset rate
([README.md](README.md), "Family coverage"). The lower voltage ranges'
columns are declared and not carried: the boards run at 3.3 V.

**The PLL's constraints** (7.3.2): the input divided by M in 1..2 MHz
(2 MHz "to limit PLL jitter"), the VCO in 100..432 MHz, N in 50..432, P
in {2, 4, 6, 8}, Q in 2..15 for the 48 MHz domain (USB OTG FS, SDIO,
the RNG). The task searches an EXACT ratio at compile time - smallest
M first, the largest legal input - and Q as the smallest divider
keeping VCO / Q at or below 48 MHz: 8 MHz x 180 / 4 / 2 = 180 MHz with
Q 8 (45 MHz - a USB program takes 168 MHz, whose VCO of 336 gives
48 MHz exactly at Q 7); 25 MHz / 16 = 1.5625 MHz x 128 / 2 = 100 MHz
with Q 5 (40 MHz). PLLCFGR is written only while the PLL is off, with
the F446's PLLR field at its reset value 2 and the reserved top bits
kept elsewhere.

**Every peripheral enable reads its register back**: the errata's 2.2.7
says a register written right after its clock enable may not take the
store, and recommends a dummy read of the enable register - the
readback in `io_clock`, `apb1_clock` and their siblings is that read.

**Out of reset** the part runs HSI at 16 MHz as SYSCLK = HCLK = PCLK1 =
PCLK2, FLASH_ACR.LATENCY 0 and the accelerator off, VOS at its reset
scale (scale 1 on the F42x/F43x, scale 2 on the F411 - the register's
reset values differ by class). A rate at or below 16 MHz needs none of
the ladder: `Clock<hsi, 16 MHz>` compiles on every header and writes
nothing but the accelerator's enables.

## Types and verbs

- `Rcc` (monostate) - `hsi_enable`/`hsi_ready`/`hsi_wait_ready`;
  `hse_enable(on, bypass)` (HSEBYP written before HSEON)/`hse_ready`/
  `hse_wait_ready`/`hse_bypassed`; `css(on)` (the clock security system
  on the HSE); `pll_enable`/`pll_ready`/`pll_wait(ready)`,
  `pll_configure(PllConfig)` (refused while the PLL is on);
  `sysclk_select(SysclkSource)`/`sysclk_status`/`sysclk_wait`;
  `bus_prescalers(apb1_div, apb2_div)` with HPRE at 1, `apb1_divider`/
  `apb2_divider`/`ahb_undivided` readbacks; `mco1(source, div)` on PA8
  and, where the header has it, `mco2(source, div)` on PC9 with the
  published source codes; the enables `io_clock(port, on)`,
  `ahb1_clock`/`ahb2_clock`/`ahb3_clock` (the latter two where the bus
  exists), `apb1_clock`, `apb2_clock`, each with its readback, and the
  reset pulses `ahb1_reset`, `ahb2_reset`, `ahb3_reset`, `apb1_reset`,
  `apb2_reset`. `ready_spins` bounds every wait.
- `PllConfig{m, n, p, q, from_hse}`; `pll_config_for(src_hz, out_hz,
  from_hse)` (m == 0 for none); `apb_divider_for(hclk, ceiling)`,
  `ppre_code(div)`; `RateRegime{known, scale, over_drive}` and
  `regime_for(hclk)` from the reserve's ladder.
- `ClockSource::hsi | hse | pll_hsi | pll_hse`, `HseMode::crystal |
  bypass`, `SysclkSource::hsi | hse | pll`, the constants `hsi_hz`,
  `hse_crystal_min_hz`/`max_hz`, `hse_bypass_min_hz`/`max_hz`, the PLL
  windows.
- `Clock<source, hz, hse_hz = 0, hse_mode = crystal>` - `hz`,
  `pclk1_hz`, `pclk2_hz`, `usb_hz` (the Q output, 0 without the PLL),
  `is_static`, `source`, `uses_hse`, `uses_pll`, `root_hz`, `pll`,
  `regime`, `wait_states`, `apb1_div`, `apb2_div`, `needs_ladder`,
  `init()` (false when a root does not come up or a readback does not
  agree). Refused at compile time: an HSE root without its rate or
  outside the datasheets' windows, a divided `hsi`, an `hse` rate other
  than the root's, a PLL ratio with no exact solution, a rate above the
  ladder or on a part whose ladder is unknown.
- `apb_hz(clock, on_apb2)` - the bus rate a peripheral's divisor
  divides, folding for a static clock.
- `Pwr` (monostate) - `bus_clock(on)` (APB1ENR.PWREN with the readback;
  every other verb opens it first), `scale(VoltageScale)`/`scale()`/
  `scale_exists`/`scale_ready`, `has_over_drive`, `over_drive_enter`
  (ODEN, ODRDY, ODSWEN, ODSWRDY, each waited for), `over_drive_exit`
  (both bits at once), `over_drive_active`. `VoltageScale::scale3 |
  scale2 | scale1`.
- `FlashWaitStates` - `get`, `set(ws)` with the readback, `needed_for
  (hclk)` from the ladder, `max_latency` from the field's width.
  `FlashAccel` - `prefetch`, `icache`, `dcache` set/get, `icache_reset`/
  `dcache_reset` (refused while the cache is on), `enable_all`.
- `DeviceUid::read()` (three words at UID_BASE), `flash_size_kbytes()`,
  `DeviceIdcode::read()` (DEV_ID, REV_ID from DBGMCU_IDCODE).

## How to use it

The board file's one line, then `init()` first thing in `main()`:

```cpp
// The Nucleo-F446RE: the ST-LINK's 8 MHz MCO into HSE in bypass.
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
// The STM32F429I-DISC1: its 8 MHz crystal X3.
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
// The black pill: a 25 MHz crystal, the F411's ceiling.
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
// A USB program on a 180 MHz part: 168 MHz, whose Q output is 48 MHz exactly.
using UsbClock = brio::Clock<brio::ClockSource::pll_hse, 168'000'000, 8'000'000>;
static_assert(UsbClock::usb_hz == 48'000'000);

constexpr SysClock clock;
const bool clock_ok = SysClock::init();   // false: a root did not come up - see the CLK verb
```

A peripheral's rate: `brio::apb_hz(clock, brio::Usart<2>::on_apb2)`
gives 45 MHz at 180; `clock_hz(clock)` stays 180 MHz for the
timebase and `delay_us`.

## Bench findings

`test_stm32f4_platform` letter e and the console's `CLK` verb, on the
three boards:

- **180 MHz in over-drive on the Nucleo-F446RE** from the ST-LINK's
  MCO in bypass: CR reads HSEON | HSERDY | HSEBYP | PLLON | PLLRDY,
  PLLCFGR 0x28402D04 (M 4, N 180, P 2, Q 8, HSE, R 2), CFGR 0x940A
  (SW and SWS PLL, PPRE1 /4, PPRE2 /2), FLASH_ACR 0x705 (latency 5,
  PRFTEN | ICEN | DCEN), PWR_CR 0x3C000 (VOS 11, ODEN, ODSWEN), PWR_CSR
  0x34000 (VOSRDY, ODRDY, ODSWRDY) - read over SWD after a halt; the
  kernel tick counted 2000 in two seconds of wall time, so the PLL's
  ratio holds to the reload's arithmetic.
- **180 MHz in over-drive on the STM32F429I-DISC1** from its 8 MHz
  crystal: the same registers but HSEBYP clear. THE BYPASS
  CONFIGURATION DOES NOT WORK ON THIS BOARD: with HSEBYP set and the
  MCO route's SB18 open, HSERDY never rises (CR read 0x57E83: HSEON and
  HSEBYP set, HSERDY clear, the PLL never started), `init()` returns
  false at the first wait, and the program that ignores the false runs
  on HSI at 16 MHz with a console divisor meant for 90 MHz - the noise
  on the host that led to UM1670 7.12.1.
- **100 MHz on the black pill** from its 25 MHz crystal: PLLCFGR
  0x25402010 (M 16, N 128, P 2, Q 5, HSE), CFGR 0x100A (PPRE1 /2,
  PPRE2 /1), FLASH_ACR 0x703 (latency 3), VOS 11 with no over-drive
  bits (the register has none).
- The console's baud on each board comes out of `apb_hz`: 115089 on the
  45 MHz PCLK1 (USART2, BRR 391), 115236 on the 90 MHz PCLK2 (USART1,
  BRR 781), 115207 on the 100 MHz PCLK2 (BRR 868) - the three within
  0.2 % of nominal and byte-exact on the wire.
- The 16 MHz reset rate on every one of the twenty-three headers, and
  the ladder-dependent rates on the four known classes, compile
  (`brio check stm32f4`); the F401, F410, F412, F413 and F469 headers
  refuse 84 MHz by name.

## Not covered yet

Driver gaps:
- A DYNAMIC clock (the STM32G0's `Rates<>` pack and direction-aware
  switch), and the way DOWN in general: `init()` runs once from the
  reset state, and a fall's order (frequency first, latency after, then
  the scale) is written nowhere; born with the power chapter, whose Stop
  mode undoes both the PLL and the over-drive and needs a `restore()`.
- `over_drive_exit`'s sequence under a running program (the verb exists
  and is unexercised - nothing leaves over-drive yet), the under-drive
  mode of Stop, the regulator's low-power modes: the power chapter's.
- The AHB prescaler (HPRE other than 1), LSI and LSE as roots or as
  anything (the RTC chapter's), the I2S and SAI PLLs (PLLI2S, PLLSAI -
  the audio and LTDC chapters'), the F446's PLLR output, DCKCFGR's
  timer prescaler rule (the timer chapter states `TIMPRE`), the RTC
  prescaler in CFGR, MCO on a pad (the verbs write CFGR; the pads are
  the caller's and nothing measures the output), the CSS interrupt,
  the RCC interrupt register, the sleep-mode enables (AHBxLPENR,
  APBxLPENR - all set at reset, nothing here clears one).
- The other voltage ranges' wait-state columns (the boards are at
  3.3 V; a 1.8 V design would read table 12's last column into the
  reserve) and the five part classes whose manuals are not on the desk
  ([README.md](README.md)).
- HSI trimming (RCC_CR.HSITRIM): born with a ruler to trim against.
- The Q output on a wire: `usb_hz` is arithmetic until the USB
  chapter's controller enumerates on it.

Implemented, not bench-verified: `Rcc::css`, `mco1`/`mco2` (no pad
driven, no counter on the other end), `ahb2_clock`/`ahb3_clock` and the
reset pulses (no driver on those buses yet), `FlashAccel`'s individual
setters and the cache resets (`enable_all` is what runs), `Pwr::scale`
at scale 2 and 3 (every board runs at scale 1: 180 MHz needs it, and
the F411's 100 MHz does too), `Pwr::over_drive_exit`, the one-bit VOS
path of the F405 class (compiled on its headers, no board),
`Clock<hsi>` and `Clock<hse>` undivided at run time (compiled
everywhere, never flashed - the boards run the PLL), and a PLL from HSI
(`pll_hsi`; the boards' roots are their HSEs).
