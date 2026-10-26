# Reset, the two watchdogs and the fault vectors - why the program is running, and how to end it (STM32F4)

This document is for a programmer who needs to know why the program is
running - a power-on, the reset pin, a software reset, a watchdog - or
wants to end it on purpose and find out at the next boot what ended it.
`stm32f4/reset.hpp` reads and clears the RCC's reset flags (`Reset`),
drives the two watchdogs this family carries (`Iwdg`, the independent
one on its own low-speed RC oscillator, the LSI; `Wwdg`, the window
watchdog on the APB1 clock), switches the Cortex-M4's three configurable
fault vectors on and reads what they recorded (`Faults`), and gives a
program the body to bind to a fault vector so that a crash leaves a note
the next boot can read instead of a silent spin (`hard_fault_reset<P>()`).

The smallest use, from the "How to use it" block below:

```cpp
extern "C" void HardFault_Handler() {
    brio::hard_fault_reset<brio::Stm32f4Platform<>>();
}

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

Line by line: the `extern "C"` function is the HardFault interrupt
handler, as in C, under the name the crt gives the vector, and its body
writes the kernel's panic record (code `kernel_fault`) unless one already
stands, then resets the chip; `take_flags()` reads the flags and clears
them, so what it returns is everything that happened since the last
clear, as a mask (`ResetFlag::watchdog` is the two watchdogs' bits
together); `arm()` starts the independent watchdog and then gives it its
timing, here the LSI divided by 8 and a reload of 0x0EEE, 955 ms at the
32 kHz the tables assume; `refresh()` reloads the counter, and a program
that stops calling it is reset at the end of the time-out.

What decides correctness even there. `take_flags()` is the boot's one
read: the register accumulates every source until it is cleared, so a
second call reports only what came after the first, and no single bit is
"the cause" (PINRSTF in particular is raised by the internal sources
too). Once started, the independent watchdog cannot be stopped by
software: only a reset stops it, so every path of the program after
`arm()` must refresh inside the time-out, and a debugger halt freezes the
counter only because the debugger set the freeze bits. `arm()` answers
false in two cases that leave the part in two states: before anything
is written, when the setting itself is invalid, and the watchdog is
then not running; or after the start, when an update never crossed to
the watchdog's own clock domain, and the watchdog is running then all
the same, with whatever timing it had - the example discards that
answer, and a program that cares reads it. The LSI is an uncalibrated RC
oscillator (17..47 kHz by the datasheet; 33.5 and 31.4 kHz on two bench
boards, below), so a time-out is the nominal one only to that accuracy.
The fault body runs only where the program binds it: an unbound
HardFault vector is the crt's own spin, and the three configurable
faults reach their own vectors only after `Faults::enable()`, escalating
to HardFault until then. The record it leaves is read at the next boot
with the kernel's `take_panic_record<P>()` (`kernel/panic.hpp`).

The rest of this page, in order: the documents of record; what the
silicon does (the flags and their two owners, the IWDG and the update
bits its keyed registers wait on, the WWDG and its arithmetic, the fault
vectors, the debug freeze bits); the types and verbs; how to use it,
with the window watchdog and a fault banked by the program; the bench
findings, every one taken across real resets; and what is not covered.

## Documents of record

RM0090 Rev 22 - 7.1 (the three kinds of reset),
7.3.21 (RCC_CSR), 7.2.9 (the watchdog clock), ch. 21 (IWDG), ch. 22
(WWDG), 38.16 (the debug freeze bits) - with its twins RM0390 Rev 6
(6.3.21, ch. 20, ch. 21) and RM0383 Rev 4 (6.3.21, ch. 15, ch. 16),
PM0214 Rev 10 4.3 (SHCSR, CCR, CFSR, HFSR, MMFAR, BFAR and AIRCR's
SYSRESETREQ) for the core half, and the LSI's 17..47 kHz from each
part's datasheet (the F429's is DocID024030 Rev 10 table 42). The
errata carry one item of this chapter, and all four carry it on every
silicon revision: **ES0206 Rev 24 2.8.1..2.8.4** (ES0298 Rev 8
2.9.1..2.9.4, ES0287 Rev 6 2.7.1..2.7.4, ES0321 Rev 14 2.10.1..2.10.4) -
the IWDG's RVU and PVU flags
are never cleared if the device enters Stop while one stands, or if the
APB clock is below twice the IWDG clock. Neither can bite here: the
driver's waits are bounded and report instead of hanging, and PCLK1 is
megahertz against the LSI's kilohertz. No sheet has an item for the
WWDG or for the reset flags.

Driver: `stm32f4/reset.hpp` (`ResetFlag`, `Reset`, `Iwdg`, `Wwdg`,
`FaultRecord`, `Faults`, `ResetReporter`, `hard_fault_reset`), with the
LSI verbs in `stm32f4/clock.hpp`'s `Rcc` (the oscillator is the clock
tree's, and RCC_CSR is a register with two owners). Family fixture
`test/family_stm32f4/reset.cpp` plus three negatives under `brio check
stm32f4`; the bench suite is `test_stm32f4_reset`.

## What the silicon does

**The flags accumulate, and there is no "the cause."** RCC_CSR's seven
reset flags are set by hardware and every one of them is "cleared by
writing to the RMVF bit" - nothing else takes them down, and 7.3.21 adds
that the register is "reset by system reset, except reset flags by power
reset only". This register is a history: the boot verb is read-and-clear
(`take_flags()`) and what a boot sees is the delta since the last clear.
**PINRSTF is raised by a software reset too** (measured): 7.3.21
describes it as the NRST pin's alone, but the pad is driven low by the
internal reset sources, so the bit names the pin only when it stands
alone - which is what `pin_only()` asks. Between the two facts the cause
is not a function of the register's value, which is why this driver
offers masks and no `cause()` enum. The family's own two supply bits,
PORRSTF and BORRSTF, are not exclusive either: 7.3.21 sets BORRSTF "when
a POR/PDR or BOR reset occurs", so they stand together.

**RCC_CSR has two owners.** Bits 31..24 are this chapter's (the flags
and RMVF); bits 1..0 are LSION/LSIRDY and belong to
[clock.md](clock.md)'s `Rcc`. Every write on either side is a
read-modify-write that preserves the other's bits, which is safe in both
directions because RMVF reads as zero and writing zero to it has no
effect.

**This family's IWDG has no window.** Four registers - KR, PR, RLR, SR -
and that is the whole of chapter 21: no WINR, no WVU, and so no
early-refresh reset. The reserve asks the device header
(`iwdg_has_window()`) and answers false on all twenty-three, which is
what makes the absence a checked fact; a deliberate reset is therefore
spelled as the SHORTEST time-out (`force_reset()`: /4 with a reload of
zero) and not as a refresh into a closed window.

**The IWDG's keyed registers do not update until it is started**, and no
part of chapter 21 says so. PR and RLR are writable only after the key
0x5555 (21.3.2) and any other key value - the 0xAAAA refresh included -
locks them again. Each has a bit in IWDG_SR that hardware raises at the
store (measured: the read right after it already sees the bit) and drops
when the value has crossed into the VDD domain, and a register read
while its bit stands returns the old value (21.4.2 and 21.4.3 say so
twice). With the watchdog stopped the bit never drops, LSION or no
LSION: 7.2.9 is where the reason hides - "if the independent watchdog is
started ... after the LSI oscillator temporization, the clock is
provided to the IWDG" - so the logic that performs the update has no
clock until the start key. That is why `arm()` starts before it
configures, and why `configure()` on a stopped watchdog answers false in
bounded time instead of hanging.

**Software cannot stop the IWDG, and the reset it causes can** - the
last four words measured. 21.3 starts it with 0xCCCC and 5.3.4 finishes
the sentence with "Once started it cannot be stopped except by a Reset";
the bench sits still for three time-outs after an IWDG reset with
nothing refreshing anything and lives, so a program does not inherit a
watchdog it must feed for ever. There is no bit that says whether it
runs. The one witness the silicon offers is the LSI the start forces on:
`Iwdg::running()` reads LSIRDY standing with LSION clear, measured in
both directions (true with the watchdog live, false at every boot). It
is a witness and not a proof - a program that set LSION itself hides the
evidence - but the other candidate requestor does not spoil it: on this
family a backup domain holding RTCEN with RTCSEL naming the LSI leaves
LSIRDY clear (measured).

**The WWDG is the opposite peripheral in every way.** It counts PCLK1
through a fixed /4096 and a programmable /2^WDGTB - two bits here, four
codes, /1 to /8, the width read off the header
(`wwdg_prescaler_codes()`) - its bus clock enable is clear at reset, and
WDGA is "set by software and only cleared by hardware after a reset"
(22.6.1). Its down-counter is free-running "even if the watchdog is
disabled" (22.3) and EWIF "is also set if the interrupt is not enabled"
(22.6.3) - so the whole timing path is measurable with WDGA never
written, which is what keeps a one-way switch out of the bench suite's
`z`. What is not reachable that way is the interrupt: with WDGA clear
the flag rises and no request is made (measured), so 22.2's "triggered
(if enabled and the watchdog activated)" is the exact sentence and
22.6.2's bit description is the loose one. **A window below 0x40 can
never be served** - a refresh is legal only while the counter is at or
below W and above 0x3F - so `wwdg_config_valid()` refuses it, and
`force_reset()` (WDGA with T6 clear, 22.3's own note) is how a caller
asks for the reset deliberately.

**Its arithmetic is PCLK1's, not the core's.** At the top of this
family's ladder APB1 runs at a quarter of HCLK, so a step is 4096 x
2^WDGTB cycles of 45 MHz and not of 180: `wwdg_timeout_us()` takes the
bus rate as an argument for exactly that reason. It reproduces table
109 (30 MHz: 136 us at /1 from T[5:0] = 0, 69905 us at /8 from 0x3F);
**22.4's worked example is inconsistent with its own formula** - it
prints 21.85 ms where 4096 x 2^3 x 64 cycles of 24 MHz are 87.38 ms -
and the table, the formula and the bench agree against it.

**A Cortex-M4 has four fault vectors and three of them are disabled at
reset.** MemManage, BusFault and UsageFault are configurable exceptions
(PM0214 4.3.10: SHCSR's MEMFAULTENA, BUSFAULTENA, USGFAULTENA, all clear
out of reset, measured), and while they are off every fault escalates to
HardFault with HFSR.FORCED set - which is what a program that never calls
`Faults::enable()` sees. Two conditions are not faults at all unless CCR asks for
them: a divide by zero (DIV_0_TRP) and an unaligned access
(UNALIGN_TRP). The second trap catches the runtime's own copy: `memcpy`
reads its source with word and halfword loads at the source's own
offset on this core ([design/runtime.md](../design/runtime.md), "The
copy across alignments"), so with UNALIGN_TRP on every copy of two
bytes or more whose source is off the boundary its loads want - a
transport's run of 12 bytes or more into a ring whose head is off a
word, three offsets in four - is a UsageFault. `Faults` is the switch and the status; the record it
reads is twelve bytes and the driver owns no storage for it, because
where a wreck goes across a reset is the application's business, exactly
as the kernel's PanicRecord is the platform's.

**The debug freeze bits are the debugger's.** DBGMCU_APB1_FZ's
DBG_IWDG_STOP and DBG_WWDG_STOP freeze the two counters while the core
is halted, and OpenOCD's `stm32f4x.cfg` sets both at every attach; the
register is reset by a power-on reset only (38.16), so they stand across
every warm boot (measured: APB1FZ reads 0x1800 on a board that has been
flashed). The driver exposes them read only - a program that fought them
would hide the behaviour a measurement is after - and they cost nothing
to read: DBGMCU sits at 0xE0042000, in the core's private peripheral
space, with no bus-clock gate in front of it (measured: its IDCODE
answers with no enable opened). Frozen under halt is not frozen while
running, which is why a suite that measures a time-out measures the real
one.

## Types and verbs

- `ResetFlag` - `low_power`, `window_watchdog`, `independent_watchdog`,
  `software`, `power_on`, `pin`, `brown_out`, plus `all`, `watchdog`
  (the two watchdogs together) and `supply` (the two the supply raises
  together).
- `Reset` - `flags()` (non-destructive), `take_flags()` (read then
  RMVF), `clear_flags()`, `pin_only(bits)` (the one reading PINRSTF can
  be trusted for), `software()` (SYSRESETREQ, `[[noreturn]]`).
- `IwdgPrescaler` (`div4`..`div256`; code 7 is a second spelling of /256
  and is never written), `iwdg_divider(p)`,
  `iwdg_nominal_ms(p, reload, lsi_hz = 32000)` - the rate is the
  CALLER's argument, because LSI is an uncalibrated RC.
- `IwdgConfig {prescaler, reload}` + `iwdg_config_valid` - a twelve-bit
  reload and no window; a reload of ZERO is legal (table 107's min
  column).
- `Iwdg` - the keys `refresh()` / `unlock()` / `start()`, the witness
  `running()`, the update bits `status()` / `busy(mask)` / `sync(mask)`,
  the readbacks `prescaler()` / `prescaler_bits()` / `reload()`,
  `configure(cfg)` and its compile-time twin `configure<cfg>()`,
  `arm(cfg)` (start, configure, refresh - the only order that works),
  `force_reset()` (the shortest time-out, refreshed into), and
  `debug_frozen()`.
- `WwdgPrescaler` (`div1`..`div8`), `wwdg_step_cycles(p)`,
  `wwdg_timeout_us(pclk1_hz, p, t)` - 22.4's formula in 32 bits, exact
  at any whole-megahertz PCLK1, giving the time to the RESET (the
  warning is one step earlier).
- `WwdgConfig {prescaler, window, early_wakeup}` + `wwdg_config_valid` -
  a seven-bit window, anything below 0x40 refused, and a prescaler code
  the field cannot hold refused with it.
- `Wwdg` - `bus_clock(on)` / `bus_clock()`, `irq()` (position 0, not
  shared on this family), `cr()` / `cfr()` / `enabled()` / `counter()` /
  `prescaler()` / `window()` / `early_wakeup_enabled()` /
  `in_window()` (whether a refresh would be legal right now),
  `configure(cfg)` and `configure<cfg>()`, `refresh(t = 0x7F)` (T6
  forced set - 22.4's Warning made structural), `start(t = 0x7F)`,
  `force_reset()`, the flag `flag()` / `clear_flag()` (rc_w0: cleared by
  writing ZERO) and the ISR body `isr()`, `debug_frozen()`.
- `FaultRecord {cfsr, hfsr, address}` - twelve bytes with the
  predicates `empty()`, `mem_fault()`, `bus_fault()`, `usage_fault()`,
  `escalated()` (HFSR.FORCED) and `address_valid()`.
- `Faults` - `enable(mem, bus, usage)` and the three readbacks,
  `divide_by_zero_trap(on)` / `unaligned_trap(on)` and their readbacks,
  `read()` (non-destructive), `clear()` (both status registers are
  write-one-to-clear), `take()` (the pair a handler wants).
- `Rcc::lsi_enable(on)` / `lsi_enabled()` / `lsi_ready()` /
  `lsi_wait_ready()` in [clock.md](clock.md) - the oscillator the IWDG
  counts.
- `ResetReporter` - a panic Reporter that resets, so the breadcrumb is
  read at the next boot.
- `hard_fault_reset<P>(context)` - the fault BODY an app binds, and the
  same body serves the other three vectors where a program enables them.
  It does NOT go through `panic()`: a BKPT taken from inside a fault
  handler is a lockup. It never overwrites a record that already stands,
  which is what makes it compose with `panic()`.

## How to use it

```cpp
#include "stm32f4/reset.hpp"

extern "C" void HardFault_Handler() {
    brio::hard_fault_reset<brio::Stm32f4Platform<>>();
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

A window watchdog with its early warning, and the wreck of a fault
banked before the reboot:

```cpp
brio::Wwdg::bus_clock(true);
(void)brio::Wwdg::configure(brio::WwdgConfig{
    .prescaler = brio::WwdgPrescaler::div8,
    .window = 0x60,
    .early_wakeup = true,
});
brio::Nvic::enable(brio::Wwdg::irq());
brio::Wwdg::start(0x7F);          // one way: only a reset disables it
// ... and in the loop, when the counter has entered the window:
if (brio::Wwdg::in_window()) { brio::Wwdg::refresh(); }

// Faults at their own vectors, with the record kept by the program:
brio::Faults::enable(false, true, true);
brio::Faults::divide_by_zero_trap(true);
extern "C" void UsageFault_Handler() {
    my_noinit_record = brio::Faults::take();
    brio::hard_fault_reset<brio::Stm32f4Platform<>>();
}
```

## Bench findings

`test_stm32f4_reset` at 180 MHz (PCLK1 45 MHz), nothing wired. Four
letters in `z` (**44 verdicts**); letter `i` outside it (**34
verdicts**), which reboots the board seven times and resumes from a
`.noinit` token; and letter `j` (**17 verdicts**), four more reboots
through the same token. `z` and `i` are green on the STM32F429I-DISC1,
the 32F469IDISCOVERY and the Nucleo-F446RE; `j` on the Nucleo-F446RE,
where `z` ran ten times in one boot, and `i` and `j` ten times each in
alternation (plus twelve more of `j` alone), all green. The IWDG leg
weighs each board's LSI: 33.5 kHz on the DISC1 (below), 35275 Hz on the
32F469IDISCOVERY, 31.4 kHz on the Nucleo-F446RE (971..973 ms over the
ten runs); every other figure below is the same on the three boards
unless it names one.

- **The flags accumulate, seen on real resets.** Leg 1 clears them and
  does a software reset: the next boot reads exactly `SFT | PIN`
  (0x14000000). Leg 2 does NOT clear them and lets the IWDG bite: the
  next boot reads `IWDG | SFT | PIN` (0x34000000) - the previous reset's
  bit still standing. Leg 3 clears them again and violates the WWDG's
  window: `WWDG | PIN` alone (0x44000000), which is RMVF proven on the
  same run. A board just flashed reads `SFT | POR | PIN | BOR`
  (0x1E000000): the supply's two bits from the power-up, and the
  programmer's own `reset run` beside them.
- **PINRSTF is raised by a software reset**, with nothing else beside it
  and the pin untouched - the flag 7.3.21 gives to the pin alone, caught
  standing for an internal source.
- **THE IWDG IS STOPPED BY THE RESET IT CAUSES.** Leg 2 arms it at /8
  with a reload of 0x0EEE and is reset 912 ms later; the boot that
  follows then sits for 3000 ms - three time-outs - with nothing
  refreshing anything, and prints its next line. A program does not
  inherit the watchdog its predecessor started.
- **The IWDG's time-out, measured: 912 and 914 ms** on two runs against
  955 ms nominal, so **LSI = 33535 and 33461 Hz** - the two agree to two
  parts in a thousand, and both sit inside the datasheet's 17..47 kHz,
  about 4.7% above the 32 kHz the tables assume. The setting is
  deliberately not the reset one, so the time-out itself proves the
  configuration landed: an unconfigured watchdog would have taken
  512 ms.
- **The witness reads both ways.** With the watchdog live and LSION
  never written, `Iwdg::running()` is true; at every boot of every
  letter it is false, and the board's backup domain reads RCC_BDCR =
  0x8200 (RTCEN set, RTCSEL = LSI) with LSIRDY clear all the same - an
  RTC's standing select does not force this oscillator, so the witness
  is not spoiled by it.
- **The LSI starts in 116 us** from LSION to LSIRDY (122 us on the
  Nucleo-F446RE), against the datasheet's 15 us typical and 40 us
  maximum - the same order, measured through a polling loop that costs
  some of it.
- **A keyed IWDG write raises its update bit AT THE STORE** (the read
  right after it already sees the bit) - only the CLEARING waits for the
  peripheral's clock. **With the watchdog stopped it never clears**:
  355 ms of bounded wait with RVU still standing, IWDG_RLR still
  reporting 0x0FFF although 0x0ABC was written, and `configure()`
  answering false after the same 355 ms rather than hanging. `arm()`,
  which starts first, answers true and its timing is what the wall clock
  above confirms.
- **A refresh really does re-lock the protected registers**: after
  0xAAAA a bare store into IWDG_PR raises no update bit at all.
- **A reload written back to back with the prescaler, right after the
  start key, can be in force before the start's own count begins.**
  The LSI takes about 120 us to start, so the start key, PR and RLR all
  wait for its first edges; with the two registers written one after
  the other (21.3's list, and the vendor's initialization sequence) and
  a reload of zero, the IWDG reset the Nucleo-F446RE 297 us after the
  start key with PVU and RVU both still standing - before `configure()`
  could return - in eight boots of eight that followed an earlier reset of
  the run (two boots straight after a flash did not show it). With
  the write of RLR held until PVU has cleared (24 runs of letter `j`'s
  first leg), or until LSIRDY and 200 us more (3 runs), the block is
  clocked by then and the same reload lands after the start: every run
  returned from the call before the reset. `configure()` therefore waits for each update before the
  next register; a reload long enough to outlast the crossing never
  shows the difference.
- **`Iwdg::force_reset()` resets the device 679 us after the call**
  (Nucleo-F446RE, LSI 31.4 kHz; the same figure in 22 runs): the LSI's
  start, the two crossings one after the other, the refresh's crossing
  and one count of /4. The bound letter `j` judges, 2 ms, is the same
  path at the datasheet's slowest LSI (17 kHz): 40 us of start, 2 x 5
  periods of crossing, 5 more for the refresh, 4 for the count - 1.23
  ms - with the polling's slack; the reset setting would take 512 ms.
  The flags read `IWDG | PIN` and nothing else.
- **`Wwdg::force_reset()` resets at once**: WDGA with T6 clear (22.3's
  note) reset the board before the loop after the write could store a
  single time - against one counter step at /1, 16384 core cycles - and
  the flags read `WWDG | PIN` alone. The write needs the WWDG's bus
  clock, which the caller opens (`bus_clock(true)`): through a closed
  gate it lands nowhere.
- **The WWDG's down-counter free-runs with WDGA clear** (0x7F to 0x5A in
  26851 us at /8, against 26943 us due for those 37 steps), and
  `in_window()` turns true exactly at the counter value the window
  names.
- **EWIF rises at 0x40 with the interrupt disabled**: 5825 us at /1 and
  45875 us at /8, against 5734 and 45875 us due for 63 steps (a step is
  91 and 728 us at PCLK1 = 45 MHz). One lands exactly on the 63 steps,
  the other one step late - 22.3's own "unknown status of the prescaler
  when writing to the WWDG_CR register", in both directions.
- **EWIF is rc_w0 both ways**: a write of one leaves it standing, a
  write of zero clears it.
- **The early-wakeup INTERRUPT needs WDGA.** With the watchdog
  deactivated, EWI enabled and the NVIC line armed, the flag rises and
  the handler does not run (0 calls in 250 ms). With the watchdog
  ACTIVATED it does: letter `i`'s last leg arms the warning on a
  watchdog that is about to reset the board anyway, and the handler
  fires one step before the reset, reading 0x40 out of the counter.
- **WDGA is cleared by the reset** (22.6.1 confirmed) and so is its bus
  clock, unlike the IWDG's registers, which come back at their own reset
  values.
- **The breadcrumb crosses a real reset, three times over.** A `panic()`
  through `ResetReporter` comes back with its code and context byte
  intact; a UDF and a divide by zero come back as `kernel_fault` with
  the context each fault body was given. With C_DEBUGEN cleared by
  `bin/brio`, `panic()`'s closing BKPT escalates into the fault handler
  - so the reset may come from either path, and the record survives only
  because `hard_fault_reset()` refuses to overwrite a valid one.
- **The two fault legs read exactly what the core writes.** The UDF with
  the three configurable faults disabled: CFSR 0x00010000
  (UFSR.UNDEFINSTR) with HFSR 0x40000000 (FORCED) - the escalation
  caught. The divide by zero with USGFAULTENA and DIV_0_TRP on: CFSR
  0x02000000 (UFSR.DIVBYZERO) with **HFSR zero** - an enabled
  configurable fault is taken at its own vector and escalates nothing.
  Neither leg had an address (MMARVALID and BFARVALID clear, as they
  must be for a UsageFault). SHCSR comes back at its reset value after
  the reboot, so the enables are the program's to make again.
- **A BusFault is taken at its own vector, precise, with the address.**
  With BUSFAULTENA on, a word load from the first address past the
  SRAM (the linker script's end of RAM: 0x20020000 on the
  Nucleo-F446RE, where 0x20020000..0x3FFFFFFF is reserved) comes back
  as CFSR 0x00008200 (BFSR.PRECISERR with BFARVALID), BFAR holding
  that very address and HFSR zero, through the same
  `hard_fault_reset()` body.
- **UNALIGN_TRP turns an unaligned word load into a UsageFault**: an
  LDR from an odd address, written by hand so the compiler cannot split
  it into byte loads, with the trap and USGFAULTENA on, comes back as
  CFSR 0x01000000 (UFSR.UNALIGNED), no address, HFSR zero; CCR reads
  0x200 at the next boot, the trap gone with the reset.
- **The three configurable faults are disabled at reset** (SHCSR reads
  zero at every boot) and the two CCR traps are off (CCR reads 0x200,
  STKALIGN alone).
- **The debug freeze bits stand on a flashed board**: DBGMCU_APB1_FZ
  reads 0x1800 (both DBG_IWDG_STOP and DBG_WWDG_STOP) after `bin/brio`'s
  upload, whose `reset run` does not clear them. They freeze the
  counters under HALT only: every time-out above was measured with them
  set and matches its formula, so a running suite is unaffected by them.

## Not covered yet

Driver gaps (this chapter's option space the stratum does not touch):

- The OPTION BYTES behind this chapter, all of them FLASH_OPTCR's:
  `WDG_SW` (a hardware IWDG started at power-on before any software
  runs), `nRST_STOP` and `nRST_STDBY` (what LPWRRSTF reports on) and the
  BOR level behind BORRSTF. Writing that register belongs to the flash
  chapter and a wrong option byte bricks a board; they are read-only
  facts here, and the driver reads none of them.
- The debug freeze bits as WRITERS. Declined: they are the debugger's,
  they survive every reset but a power-on, and a program that set them
  would hide the very behaviour a measurement is after.
- The MPU, and with it any MemManage fault a program could provoke
  deliberately. It is a chapter of its own and nothing here needs it.
- The FPU's own UsageFault sources (the lazy-stacking faults, CPACR
  access errors): born with the first brio program that executes a float
  instruction.

Implemented, not bench-verified:

- The MemManage VECTOR. Letter `d` enables and disables it, and no
  letter takes one: a MemManage needs the MPU (above), and what would
  measure it is a leg that programs one region without access and
  touches it.
- LPWRRSTF, never seen set: it needs the `nRST_STOP` / `nRST_STDBY`
  option bytes (above), and with them at their shipped value a Standby
  wake looks like a power-on instead.
- The suite on the STM32F411CE black pill (it builds there, `boards =
  f429zi,f446re,f411ce,f469ni`), and letter `j` on the STM32F429ZI and
  the STM32F469NI: what would measure them is `z`, `i` and `j` on each.
  The WWDG's figures above are PCLK1's, so they scale with each board's
  bus rate.
