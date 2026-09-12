# Reset (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 2.12
(chip-level reset), 2.13 (the power-on state machine), 2.4.2 (what
SYSRESETREQ resets), 4.7 (the watchdog's REASON and trigger), 2.8 (the
bootrom); the Cortex-M0+ AIRCR. The driver: `brio/rp2040/reset.hpp`
over `kernel/panic.hpp` and `rp2040/watchdog.hpp`. The reference
suites: `test_rp2040_platform`, letters a, g and i;
`test_rp2040_multicore` for the two-core facts.

## What the silicon does

Three kinds of reset, three signatures. A CHIP-LEVEL reset (power-on
or brown-out, the RUN pin, the rescue debug port) takes the power-on
state machine down and back up and sets its flag in
VREG_AND_CHIP_RESET.CHIP_RESET - HAD_POR, HAD_RUN, HAD_PSM_RESTART -
which then STANDS for the life of the supply. A WATCHDOG reset (the
countdown or CTRL.TRIGGER) resets what the PSM's WDSEL selects and
names itself in WATCHDOG.REASON, the last event replacing the
previous. A CORE reset, SCB.AIRCR.SYSRESETREQ, "only resets the
Cortex-M0+ processor core" (2.4.2): the core that asked restarts
through the bootrom and the second stage, the other core keeps
running, the debug logic keeps what a probe left in it, and NO MARK
is left - no flag set, none cleared. So this chip has no chip-wide
software reset of its own, and a program's "reboot" is the
watchdog's trigger under the power-on state machine's selection:
both cores, every peripheral but the oscillators, REASON.FORCE as its
mark. The SRAM keeps its content through all of them (a fact the
datasheet does not promise; measured), the watchdog's scratch
registers through all but the chip-level one.

So the causes word is a history, not a cause, and "what caused THIS
boot" is a comparison with the word the previous life saw - which is
how the bench suite reads it, through a scratch register.

## Types and verbs

- `ResetCause`: `power_on`, `run_pin`, `rescue`, `watchdog_timer`,
  `watchdog_force`, `all`.
- `Reset::causes()` (the history as one word, read-only, unchanged by
  reading), `Reset::software()` (the chip's reboot: the watchdog's
  trigger, never returns), `Reset::core()` (SYSRESETREQ: this core
  alone, never returns - the verb for a program that means exactly
  that).
- `ResetReporter`: the panic Reporter that reboots the chip instead of
  spinning, so the breadcrumb is read at the next boot - from either
  core.
- `fault_reset<P>(context)`: the HardFault body an app binds to
  `isr_hardfault` - writes a kernel_fault breadcrumb unless one stands,
  then resets; not through panic(), whose BKPT would lock the core up
  inside HardFault.

## How to use it

```cpp
const uint32_t causes = brio::Reset::causes();
const auto record = brio::take_panic_record<P>();     // once, at boot

extern "C" void isr_hardfault() { brio::fault_reset<P>(); }
brio::panic<P, brio::ResetReporter>(brio::PanicCode::assert_failed, 7);
```

## Bench findings

All from `test_rp2040_platform` letter i, five legs on one board,
green on both:

- `Reset::core()` restarts the core through the bootrom with the
  causes word exactly as it was; the .noinit word survives.
- `Reset::software()` (the trigger) and a 100 ms time-out each set
  their own REASON bit and clear the other's; HAD_POR stands
  throughout; the .noinit word survives both.
- A panic through `ResetReporter` is read at the next boot with its
  code and context, and its reset reads REASON.FORCE; a HardFault
  (UDF) through `fault_reset()` leaves a kernel_fault breadcrumb with
  the context the app gave, the same mark.
- SYSRESETREQ IS ONE CORE'S (`test_rp2040_multicore`): a two-kernel
  program reset through it on core 0 came back to a core 1 still
  running its old life and deaf to a new launch - which is why the
  chip's reboot is the watchdog's, why `Core1::launch` resets core 1
  first, and why a probe's `reset run` on core 0 alone is not a
  reboot ([multicore.md](multicore.md)).
- A board reports HAD_POR alone from its power-on until its first
  watchdog event, through every reflash over the debug port: the
  probe's reset is not a chip-level one, and the flags stand.

## Not covered yet

Driver gaps, each with its reason:

- The RUN pin and the rescue debug port as causes: the pin needs a
  wire to ground (a letter with a hand on the desk), the rescue DP
  the git OpenOCD's RESCUE mode; both are one-line verdicts to add
  with the first suite that runs them.
- Brown-out detection (VREG_AND_CHIP_RESET.BOD): a supply to lower;
  the chapter of the regulator, when written.
