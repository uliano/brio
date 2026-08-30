# RSTC and WDT - why the program is running, and how to end it (SAM C21)

> **PROVISIONAL.** Both chapters are built and bench-verified, and the
> fault-to-breadcrumb path with them. What is left out is the SUPC side
> of the reset story - BODVDD and BODCORE are sources named here and
> configured in a chapter that has no driver - and the two always-on
> behaviours no test may provoke. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 18 (RSTC) and
ch. 23 (WDT), with the fuse mapping in 9.3 and the reset-effect matrix in
table 18-1 - and errata DS80000740S, where **neither module has an item**:
there is no section for either. The watchdog does appear inside someone
else's workaround: 1.22.1 (XOSC/XOSC32K clock-failure detection cannot
switch to the safe clock when the input is stuck high) tells the
application to run the WDT and switch clocks in firmware after the reset,
which will matter to the clock pass when XOSC exists. Driver:
`samc/reset.hpp` (`Reset`, `Watchdog`, `ResetReporter`,
`hard_fault_reset`). Family fixture `test/family_samc/reset.cpp` plus one
negative under `tools/check_samc.sh`; the bench suite is
`test_samc_platform`.

## What the silicon does

**RCAUSE is exclusive, not cumulative**, and this is the fact most worth
stating because the AVR family teaches the opposite habit. "When a Reset
occurs, the bit corresponding to the Reset source is set to '1' and all
other bits are written to '0'" (18.8.1). The register is read-only, there
is nothing to clear, and it always describes exactly one reset - the
last. The AVR's RSTFR accumulates history and needs a read-and-clear verb
at boot; porting that habit here would mean writing to a read-only
register and reading a history that does not exist.

**Not every reset resets everything** (table 18-1). Only a power-supply
reset - POR, BODVDD, BODCORE - clears the whole device. An external reset
spares the RTC, OSC32KCTRL, RSTC itself and a WRTLOCK'd GCLK; a watchdog
reset or a system reset request spares those AND THE DEBUG LOGIC. Two
consequences, and the second one bites:

- a watchdog reset leaves a running RTC running, which is a feature when
  something must keep time across a recovery and a trap for code that
  assumes a reset is a clean slate;
- once a debug probe has enabled halting debug, no software reset turns
  it off again. See "the BKPT hazard" below.

**SRAM appears in no row of that table**, for any source including
power-on. Nothing promises a `.noinit` breadcrumb survives - which is why
`take_panic_record()` checks a magic word, and why the linker script
marks `.noinit` NOLOAD so the crt neither loads nor zeroes it. That the
breadcrumb does in fact survive a system reset on this silicon is a
measurement, and the bench suite makes it.

**The watchdog runs on its own oscillator and its reset values are
fuses.** The counter is clocked at 1.024 kHz from OSCULP32K, deliberately
inaccurate - "the exact time-out period may vary from device-to-device"
(23.5.3) - so a watchdog margin here is a real margin and not a rounding.
And CTRLA.ENABLE, CTRLA.ALWAYSON, CTRLA.WEN, CONFIG.PER, CONFIG.WINDOW
and EWCTRL.EWOFFSET are all loaded from the NVM User Row at power-on
(23.6.2.2): what a program finds in those registers at boot is what the
fuse row says. `samc/nvm.hpp`'s `NvmUserRow` reads that row, so the two
drivers describe the same thing from two directions and must agree.

**Three periods, one encoding.** CONFIG.PER (the time-out, or the OPEN
window), CONFIG.WINDOW (the CLOSED window) and EWCTRL.EWOFFSET all count
CLK_WDT_OSC cycles as 8 << n, twelve values from 8 to 16384 - about 8 ms
to about 16 s. One enum covers all three.

**The CLEAR register is a loaded gun.** Writing 0xA5 restarts the period;
"writing any other value than 0xA5 to CLEAR will issue an immediate
system reset" (23.6.2.4). Measured, that sentence is both wider and
softer than it reads: it bites whether the watchdog is enabled or not,
and it is not immediate - CLEAR is write-synchronized, so the reset lands
a few CLK_WDT_OSC cycles later and the CPU runs on in the meantime.

**Enable-protection and synchronization split the registers in two.**
CTRLA (except ENABLE), CONFIG and EWCTRL are writable only while the
watchdog is disabled; CTRLA.ENABLE, CTRLA.WEN, CTRLA.ALWAYSON and CLEAR
cross into the 1.024 kHz domain and each has a SYNCBUSY bit. Nothing is
both.

**Always-on is one-way.** Setting CTRLA.ALWAYSON makes the watchdog run
regardless of ENABLE, makes CONFIG and EWCTRL read-only, and can be
cleared only by a power-on reset. A program that sets it by accident
makes a board that resets itself until it is unplugged.

## Types and verbs

**`ResetCause`** - `unknown`, `power_on`, `brown_out_core`,
`brown_out_vdd`, `external`, `watchdog`, `system_request`. An enum and
not a flag set, because RCAUSE names one source.

**`Reset`** - `cause_bits`, `cause`, `power_supply_reset`, `user_reset`,
`warm` (true when the peripherals that survive a user reset are still
running), and `software()`, which resets through the CPU's SYSRESETREQ
and shows up as `system_request`. There is no configuration: RSTC has one
read-only register, no interrupts, no events, and is active in every
sleep mode.

**`WdtCycles`** - the shared period encoding, `cyc8` to `cyc16384`.
`wdt_nominal_ms()` converts one to milliseconds at the nominal rate,
rounded, which is arithmetic about the nominal and not a promise about a
particular die.

**`WdtConfig`** - `period`, `window_mode` + `window`, `early_warning` +
`ew_offset`, `always_on`. Defaults: a 16384-cycle period, normal mode, no
warning, not always-on.

| Field | Register | Note |
|---|---|---|
| `period` | CONFIG.PER | the time-out; the OPEN window in window mode |
| `window_mode` | CTRLA.WEN | a clear before the window opens is itself a reset |
| `window` | CONFIG.WINDOW | the CLOSED window |
| `early_warning` | INTENSET.EW | an interrupt before the bite |
| `ew_offset` | EWCTRL.EWOFFSET | from the period's start; ignored in window mode |
| `always_on` | CTRLA.ALWAYSON | ONE-WAY - only a power-on reset undoes it |

**`Watchdog`** - `arm<cfg>()` refuses an impossible configuration at
compile time, `arm(cfg)` returns false and writes nothing. Impossible
means one thing: in NORMAL mode an early-warning offset at or past the
period, where 23.6.8.2 says the reset arrives first and the interrupt
never does, so a caller who asked for a warning would silently get none.
Window mode is exempt - there the offset is unused.

Beside arming: `disable` (refused while always-on, because 23.6.2.3 says
it cannot work and pretending otherwise would leave a caller believing a
live watchdog is off), `clear` (the 0xA5 key, and only that), and
`force_reset` (a deliberately wrong key). Read-back: `ctrla`, `enabled`,
`window_mode`, `always_on`, `config`, `period`, `window`, `ew_offset`.
Synchronization: `busy`, `sync`, both bounded. The interrupt:
`arm_interrupt`, `flags`, `armed`, `clear_flags`, `early_warning_flag`,
`isr` (the handler body; the app binds the vector). And `bus_clock`, for
a power pass that wants the APB clock off - it is on at reset.

**`ResetReporter`** - a `kernel/panic.hpp` Reporter that ends the program
with a system reset instead of a spin, so the breadcrumb is read at the
next boot rather than needing a debugger.

**`hard_fault_reset<P>(context)`** - the HardFault handler BODY, bound by
the app:

```cpp
extern "C" void HardFault_Handler() {
    brio::hard_fault_reset<brio::SamPlatform>();
}
```

It does not go through `panic()`, and the reason is specific to this
core: `panic()` calls `break_here()`, which is BKPT, and a BKPT taken
from inside HardFault is a LOCKUP - a locked-up core never reaches the
reset. And it does not overwrite an existing valid record: a fault that
follows a panic is a consequence of something already diagnosed, so the
original code and context stand.

Both the reporter and the body COMPOSE with the trace half of the
post-mortem, which lives in `samc/postmortem.hpp` and changes nothing
here: `TracingReporter<Store>` captures and then chains to
`ResetReporter`, and `hard_fault_trace_reset<P, Store>()` captures and
then calls this body. What the trace is worth, and why on a board with
DHCSR.C_DEBUGEN clear the fault body is the path that runs even for an
orderly panic, are in [mtb.md](mtb.md).

## How to use it

**Ask why you are here**, once, at boot:

```cpp
switch (Reset::cause()) {
case ResetCause::watchdog:       /* recover, and count it */ break;
case ResetCause::power_on:       /* cold start */ break;
default: break;
}
if (auto record = take_panic_record<SamPlatform>()) {
    // report record->code and record->context over the console
}
```

**Run a watchdog over a loop that must not stall:**

```cpp
Watchdog::arm(WdtConfig{.period = WdtCycles::cyc2048});   // ~2 s
for (;;) {
    kernel.step();
    Watchdog::clear();
}
```

**Ask to be warned before the bite**, which is how a sleeping system gets
a chance to feed the dog and go back to sleep:

```cpp
constexpr WdtConfig cfg{
    .period = WdtCycles::cyc2048,
    .early_warning = true,
    .ew_offset = WdtCycles::cyc1024,   // must be SHORTER than the period
};
Watchdog::arm<cfg>();
// extern "C" void WDT_Handler() { if (Watchdog::isr()) { ... } }
```

**Make a panic reportable** rather than a silent stop, by choosing the
reporter that resets:

```cpp
panic<SamPlatform, ResetReporter>(PanicCode::assert_failed, my_context);
// ... and at the next boot, take_panic_record() has the story
```

## Bench findings

From `test_samc_platform` (34 verdicts in `z`, plus letter `i` outside it
- `i` reboots the board six times and must be asked for by name, with
`bench.py run C i --expect="->"`). Nothing to wire, and nothing costs
endurance.

- **The fuse row and the watchdog registers agree**, field by field:
  ENABLE, ALWAYSON, PER, WINDOW and EWOFFSET all read back what
  `NvmUserRow` reports. This board carries the production defaults - the
  watchdog off, all three period fields at 0xB.
- **RCAUSE really is exclusive**: exactly one bit set at every one of the
  seven boots the suite observes, and reading it changes nothing.
- **OSCULP32K measured, by difference: 1030.4 Hz** against a nominal
  1024, about +0.6%. The two-point method is what makes that number
  mean anything - a single early-warning measurement carries a constant
  **3 ms arming cost** (the CTRLA and CLEAR writes crossing into the
  1.024 kHz domain), so 512 cycles time as 499 ms and 1024 as 996 ms,
  and only their difference - 496 ms for exactly 512 cycles - is free of
  it. CONFIG.WINDOW counts the same clock as EWCTRL.EWOFFSET.
- **A wrong CLEAR key resets whether the watchdog runs or not.**
  23.6.2.4's sentence sits inside the Normal-mode section, which leaves
  the stopped case open; measured, the key bites with CTRLA.ENABLE clear
  exactly as it does with it set, and RCAUSE calls it a WATCHDOG reset
  both times. It is also **not immediate**: CLEAR is write-synchronized,
  and the first version of this test ran on past its own trigger into
  the next leg before the reset landed.
- **The breadcrumb survives a system reset**, code and context byte
  intact - which table 18-1 promises nowhere, so it is a measurement and
  not a quotation.
- **A software reset and a watchdog reset are distinguishable** at the
  next boot (SYST against WDT), so a program can signal two intentions
  across a reset with no surviving RAM at all.
- **A HardFault reaches `hard_fault_reset()` and resets**, leaving a
  `kernel_fault` record with its context byte. Getting there needed the
  right instruction: **UDF, because an unaligned volatile word load does
  not fault** - gcc knows the constant's misalignment and emits four byte
  loads with shifts, so the intended fault quietly reads valid SRAM and
  returns. Only the permanently-undefined instruction is beyond the
  compiler's reach.
- **A watchdog time-out and a window violation both reset**, both report
  WDT, and after either the watchdog comes back at its FUSE setting - not
  the one that bit.
- **THE BKPT HAZARD, and it is an operational fact about this desk.** A
  core with DHCSR.C_DEBUGEN set HALTS on a BKPT instead of faulting on
  it, so `break_here()` - which every `panic()` ends in - stops the board
  dead with no output. Attaching a probe sets that bit, and table 18-1
  makes it sticky: the debug logic is reset by a power-on or an external
  reset and NOT by a watchdog reset or a system reset request, so every
  later software reset inherits it. Diagnosed by halting the silent board
  and finding it parked on the BKPT instruction. `tools/bench.py` now
  clears C_DEBUGEN as the last step of every SAM flash, which is what a
  board with no probe attached looks like.
- **An unbound vector is a silent death** on this target - the crt's
  default handler is a spin - and `HardFault_Handler` needed the `weak`
  attribute the crt's own comment already claimed it had: without it an
  app that binds the vector fails to link.

## Not covered yet

Driver gaps (not built):

- **SUPC** (ch. 22) entirely, and with it the configuration behind two of
  the six reset causes: BODVDD and BODCORE are named by `ResetCause` and
  set up by a chapter that has no driver. The user row's BODVDD level,
  disable and action fields are read by `samc/nvm.hpp` and acted on by
  nobody.
- **Vector table relocation** (VTOR): a bootloader would want it, nothing
  else does.
- **The external reset pin** as a source this code can provoke - only the
  operator can pull it, so `ResetCause::external` is decoded and never
  produced here.

Implemented but not bench-verified:

- **Always-on mode.** `WdtConfig::always_on` is written and read back in
  the family fixture and never set on hardware: it can be undone only by
  a power-on reset, so a test that armed it would leave a board resetting
  itself until someone unplugged it. The same reason keeps
  `disable()`'s refusal-while-always-on path unexercised.
- `bus_clock()`, which only matters once something turns the APB clock
  off - the power pass.
- `Reset::warm()` as a decision: the RTC surviving a user reset is
  table 18-1's claim and there is no RTC driver yet to observe it with.
- Operation on the E and G variants: compile-checked only. Neither module
  varies by package.
