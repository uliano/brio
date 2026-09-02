# Reset and the two watchdogs - why the program is running, and how to end it (STM32G0)

> **PROVISIONAL.** The reset flags, both watchdog chapters and the
> fault-to-breadcrumb path are built and bench-verified. What is left
> out is the OPTION-BYTE side of the story - the bits that decide
> whether either watchdog is a hardware one and what they do in Stop and
> Standby are read here and written nowhere - and the PWR half of
> PWRRSTF. The list is in "Not covered yet".

Documents of record: RM0444 Rev 6 - 5.1 (the three kinds of reset),
5.4.24 (RCC_CSR), ch. 28 (IWDG), ch. 29 (WWDG), 5.2.14 (the watchdog
clock), 40.10.3 (the debug freeze register) - with DS13560 Rev 5 table
46 (the LSI's 29.5..34 kHz) and table 73 (the IWDG's time-out grid)
behind them, and errata **ES0548 Rev 3, where neither watchdog and no
reset flag has an item on either silicon revision**. The errata's only
LSI item, 2.2.1, is conditioned on "LSI clocks the RTC, or it clocks the
clock security system on LSE" - nothing in this chapter brings that
about, though the bench board's RTC does (see below), and the item's own
workaround is keyed on PWRRSTF, a flag this driver reads.

Driver: `stm32g0/reset.hpp` (`ResetFlag`, `Reset`, `Iwdg`, `Wwdg`,
`ResetReporter`, `hard_fault_reset`). Family fixture
`test/family_stm32g0/reset.cpp` plus four negatives under
`tools/check_stm32g0.sh`; the bench suite is `test_stm32_platform`.

## What the silicon does

**The flags accumulate, and there is no "the cause."** RCC_CSR's seven
reset flags are set by hardware and "cleared by setting the RMVF bit" -
nothing else takes them down, and 5.4.24 adds that the register is
"reset upon system reset, except for reset flags that are only reset
upon power reset". This register is a HISTORY: the AVR's RSTFR habit,
not the SAM's exclusive RCAUSE, so the boot verb is read-and-clear
(`take_flags()`) and what a boot sees is the delta since the last clear.
And **PINRSTF is a catch-all**: 5.4.24 sets it "when a reset from the
PF2-NRST pin occurs OR when a system reset is triggered by any other
source", so it stands beside every other flag and names the pin only
when it stands ALONE. Between the two facts, the cause is not a function
of the register's value, which is why this driver offers masks and no
`cause()` enum.

**RCC_CSR has two owners.** Bits 31..23 are this chapter's; bits 1..0
are LSION/LSIRDY and belong to [clock.md](clock.md)'s `Rcc`. Every write
on either side is a read-modify-write that preserves the other's bits.

**The IWDG is on another clock, software cannot stop it, and a reset
can.** It counts LSI, its registers sit in the VDD domain, and 28.3.1
says "Once running, the IWDG cannot be stopped" - a sentence the Stop
and Standby summaries (5.3) finish with the four words that matter,
"except upon a reset". MEASURED, and worth stating because the family
lore and both other brio targets teach the opposite: the boot after an
IWDG reset outlives its own time-out several times over with nothing
refreshing anything. A program does NOT inherit a watchdog it must feed
for ever. There is still no bit that says whether it runs, and LSIRDY is
not one (below).

**The IWDG's keyed registers do not update until it is started**, and no
part of chapter 28 says so. PR, RLR and WINR are writable only after the
key 0x5555 (28.3.4) and any other key value - the 0xAAAA refresh
included - locks them again. Each has a bit in IWDG_SR that hardware
raises AT THE STORE and drops when the value has crossed into the VDD
domain, and a register read while its bit stands returns the OLD value
(28.4.2/3/5 say so three times). With the watchdog STOPPED the bit never
drops, LSION or no LSION: 5.2.14 is where the reason hides - "if the
IWDG is started ... after the LSI oscillator temporization, the clock is
provided to the IWDG" - so the logic that performs the update has no
clock until the start key. That is why every sequence in 28.3.2 begins
with 0xCCCC, and why `arm()` starts before it configures.

**The WWDG is the opposite peripheral in every way.** It counts PCLK
through a fixed /4096 and a programmable /2^WDGTB, its bus clock enable
is CLEAR AT RESET (and a peripheral with no clock does not answer a read
or take a store, 5.2.17), and WDGA is "set by software and only cleared
by hardware after a reset" (29.5.1). Its down-counter is FREE-RUNNING
"even if the watchdog is disabled" (29.3.3) and EWIF "is also set if the
interrupt is not enabled" (29.5.3) - so the whole TIMING path is
measurable with WDGA never written, which is what keeps a one-way switch
out of the bench suite's `z`. What is not reachable that way is the
interrupt: with WDGA clear the flag rises and no request is made
(measured), so 29.2's "triggered (if enabled and the watchdog
activated)" is the exact sentence and 29.5.2's bit description is the
loose one.

**A window that cannot be served is refused, on both.** The IWDG resets
if a refresh happens while the counter is above WIN, so WIN = 0 is a
setting in which no refresh is ever legal; a WWDG refresh is legal only
while the counter is at or below W and above 0x3F (29.3.3), so any W
below 0x40 is the same trap. Both are refused by the `*_config_valid()`
predicates. Provoking a reset ON PURPOSE has its own spelling on each:
`Iwdg::force_reset()` refreshes into a closed window,
`Wwdg::force_reset()` clears T6 with WDGA set (29.3.3's own note).

**The debug freeze bits need two things.** DBG_APB_FZ1's DBG_IWDG_STOP
and DBG_WWDG_STOP answer a store only with RCC_APBENR1.DBGEN set (clear
at reset), and 40.10.3 says the register is "not reset by system reset" -
so it holds whatever touched it last, across every warm boot.

## Types and verbs

- `ResetFlag` - `low_power`, `window_watchdog`, `independent_watchdog`,
  `software`, `power`, `pin`, `option_loader`, plus `all` and
  `watchdog` (the two watchdogs together).
- `Reset` - `flags()` (non-destructive), `take_flags()` (read then
  RMVF), `clear_flags()`, `pin_only(bits)` (the one reading PINRSTF can
  be trusted for), `software()` (SYSRESETREQ, `[[noreturn]]`).
- `IwdgPrescaler` (`div4`..`div256`; code 7 is a second spelling of
  /256 and is never written), `iwdg_divider(p)`,
  `iwdg_nominal_ms(p, reload, lsi_hz = 32000)` - the rate is the
  CALLER's argument, because LSI is an uncalibrated RC.
- `IwdgConfig {prescaler, reload, window}` + `iwdg_config_valid` -
  twelve-bit fields, and a zero window refused.
- `Iwdg` - the keys `refresh()` / `unlock()` / `start()`, the update
  bits `status()` / `busy(mask)` / `sync(mask)`, the readbacks
  `prescaler()` / `prescaler_bits()` / `reload()` / `window()`,
  `configure(cfg)` and its compile-time twin `configure<cfg>()`,
  `arm(cfg)` (start, configure, and a closing refresh only when the
  window is disabled - with a live window the counter has just been
  reloaded ABOVE it and a refresh there would be the reset),
  `force_reset()`, `debug_freeze()`.
- `WwdgPrescaler` (`div1`..`div128`), `wwdg_step_cycles(p)`,
  `wwdg_timeout_us(pclk_hz, p, t)` - RM0444 29.3.4's formula in 32
  bits, exact at any whole-megahertz PCLK.
- `WwdgConfig {prescaler, window, early_wakeup}` +
  `wwdg_config_valid` - seven-bit window, and anything below 0x40
  refused.
- `Wwdg` - `bus_clock(on)` / `bus_clock()`, `irq()` (position 0, not
  shared on this family), `cr()` / `cfr()` / `enabled()` /
  `counter()` / `prescaler()` / `window()` /
  `early_wakeup_enabled()`, `configure(cfg)` and `configure<cfg>()`,
  `refresh(t = 0x7F)` (T6 forced set - 29.3.3's Warning made
  structural), `start(t = 0x7F)`, `force_reset()`, the flag
  `flag()` / `clear_flag()` (rc_w0: cleared by writing ZERO) and the
  ISR body `isr()`, `debug_freeze()`.
- `ResetReporter` - a panic Reporter that resets, so the breadcrumb is
  read at the next boot.
- `hard_fault_reset<P>(context)` - the HardFault BODY an app binds. It
  does NOT go through `panic()`: a BKPT taken from inside HardFault is
  a lockup on this core. It never overwrites a record that already
  stands, which is what makes it compose with `panic()`.

## How to use it

```cpp
#include "stm32g0/reset.hpp"

extern "C" void HardFault_Handler() {
    brio::hard_fault_reset<brio::Stm32Platform>();
}
extern "C" void WWDG_IRQHandler() { (void)brio::Wwdg::isr(); }

int main() {
    const uint32_t why = brio::Reset::take_flags();   // once, at boot
    if (why & brio::ResetFlag::watchdog) { /* a recovery boot */ }

    // A watchdog with about a second of rope. arm() starts it first,
    // because the registers do not update before that.
    (void)brio::Iwdg::arm(brio::IwdgConfig{
        .prescaler = brio::IwdgPrescaler::div8,
        .reload = 0x0EEE,
    });
    for (;;) {
        brio::Iwdg::refresh();
        // ... the loop that must keep running
    }
}
```

## Bench findings

`test_stm32_platform` on the Nucleo-G0B1RE at 64 MHz, nothing wired.
Six letters in `z` (53 verdicts, 53/53 - one cold from a fresh flash and
five warm) and letter `i` outside it (26 verdicts, 26/26 twice), which
reboots the board six times and resumes from a `.noinit` token.

- **The flags accumulate, seen on real resets.** Leg 1 clears them and
  does a software reset: the next boot reads exactly `SFT | PIN`
  (0x14000000). Leg 2 does NOT clear them and lets the IWDG bite: the
  next boot reads `IWDG | SFT | PIN` (0x34000000) - the previous
  reset's bit still standing. Leg 3 clears them again and violates the
  WWDG's window: `WWDG | PIN` alone (0x44000000), which is RMVF proven
  on the same run.
- **PINRSTF is raised by a software reset**, with nothing else beside
  it and the pin untouched - the catch-all caught.
- **THE IWDG IS STOPPED BY THE RESET IT CAUSES.** Leg 2 arms it at /8
  with a reload of 0x0EEE and is reset 940 ms later; the boot that
  follows then sits for 1500 ms with nothing refreshing anything and
  prints its next line. Had 28.3.1's first sentence been the whole
  truth, that boot would have died at 940 ms and every one after it.
- **The IWDG's time-out, measured: 940 ms** against 955 ms nominal, so
  **LSI = 32536 Hz** (a second run: 940 ms, 32536 Hz; an earlier
  arrangement at /4 gave 504 ms and 32507 Hz) - inside DS13560 table
  46's 29.5..34 kHz, about 1.7% above the 32 kHz nominal. The setting
  is deliberately not the reset one, so the time-out itself proves the
  configuration landed: an unconfigured watchdog would have taken
  512 ms.
- **A keyed IWDG write raises its update bit AT THE STORE** (the read
  right after it already sees the bit) - only the CLEARING waits for
  the peripheral's clock. **With the watchdog stopped it never
  clears**: 626 ms of bounded wait and RVU still standing, with
  IWDG_RLR still reporting 0x0FFF although 0x0ABC was written.
  `configure()` answers false rather than hanging. `arm()`, which
  starts first, answers true and its timing is what the wall clock
  above confirms.
- **A refresh really does re-lock the protected registers**: after
  0xAAAA a bare store into IWDG_PR raises no update bit at all.
- **LSIRDY IS NOT A WATCHDOG WITNESS ON THIS BOARD.** It stands with
  LSION clear at every boot, and the reason is not the IWDG:
  `RCC_BDCR` reads 0x8200 - RTCEN set, RTCSEL = LSI - and the RTC
  domain survives every system reset. (Read over SWD and printed by
  letter `a`. It also means ES0548 2.2.1's precondition - LSI clocking
  the RTC - is live on this board, though its trigger, a VDD reset
  without a backup-domain reset, is not something this suite provokes.)
- **The WWDG's down-counter free-runs with WDGA clear** (0x7F to 0x77
  in 4 ms at /8) and **EWIF rises at 0x40 with the interrupt
  disabled**: 32128 us at /8 and 258048 us at /64, against 63 steps of
  512 and 4096 us. The measurement lands between 62 and 63 steps, and
  the missing step is 29.3.3's own - "the timing varies between a
  minimum and a maximum value, due to the unknown status of the
  prescaler when writing to the WWDG_CR register".
- **EWIF is rc_w0 both ways**: a write of one leaves it standing, a
  write of zero clears it - the opposite discipline to every
  write-one-to-clear register in the other two strata.
- **The early-wakeup INTERRUPT needs WDGA.** With the watchdog
  deactivated, EWI enabled and the NVIC line armed, the flag rises and
  the handler does not run (0 calls in 250 ms). With the watchdog
  ACTIVATED it does: letter `i`'s sixth leg arms the warning on a
  watchdog that is about to reset the board anyway, and the handler
  fires one step before the reset, reading 0x40 out of the counter.
- **The breadcrumb crosses a real reset, twice over.** A `panic()`
  through `ResetReporter` comes back with its code and context byte
  intact; a UDF three instructions later comes back as `kernel_fault`
  with the context the fault body was given. With C_DEBUGEN cleared by
  `bench.py`, `panic()`'s closing BKPT escalates into the fault handler
  - so the reset may come from either path, and the record survives
  only because `hard_fault_reset()` refuses to overwrite a valid one.
- **WDGA is cleared by the reset** (29.5.1 confirmed): the boot after a
  WWDG reset finds the watchdog disabled, unlike the IWDG's registers,
  which come back at their own reset values.
- **The debug freeze bits are writable only with RCC_APBENR1.DBGEN on**
  - the first version of the suite measured a store that went nowhere -
  and, being in a register 40.10.3 does not reset on a system reset,
  they were found set on warm boots and clear on cold ones.

## Not covered yet

Driver gaps (this chapter's option space the stratum does not touch):
- The OPTION BYTES behind both watchdogs: `IWDG_SW` and `WWDG_SW`
  (hardware or software watchdog), `IWDG_STOP` and `IWDG_STDBY` (the
  counter's fate in the low-power modes), and `nRST_STOP` /
  `nRST_STDBY` / `nRST_SHDW`, which are what LPWRRSTF reports on. They
  are read-only facts here; writing FLASH_OPTR belongs to the flash
  campaign, and a wrong option byte is a bricked board.
- The NRST pin's three modes (reset input/output, reset input, PF2
  GPIO) - option bytes again.
- The PWR side of PWRRSTF: the BOR levels and the brown-out detector
  arrive with the sleep pass.
- The RTC domain reset (`RCC_BDCR.BDRST`), which is the one thing that
  would put this board's RTCEN/RTCSEL back - and ES0548 2.2.1's
  workaround. It belongs to an RTC or clock pass, not here.

Implemented, not bench-verified:
- `Iwdg::force_reset()` (the window-violation path; the WWDG's
  equivalent IS exercised, and the IWDG's window is never armed by any
  letter), `Iwdg::configure<cfg>()`'s run-time half on a RUNNING
  watchdog other than through `arm()`, and the IWDG's window option
  itself - no letter arms a live window.
- `Wwdg::force_reset()` (letter `i` reaches the same reset by letting
  the counter run down instead), and the WWDG's early-wakeup interrupt
  as a REFRESHING recovery handler (29.4's second use), which no letter
  stages because activating the watchdog is one-way inside a power
  cycle.
- LPWRRSTF and OBLRSTF: nothing here enters a low-power mode or
  launches the option byte loader, so those two flags have never been
  seen set.
