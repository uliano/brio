# Reset (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), chapter 7
whole (7.1 the three tiers, 7.2 what changed from the RP2040, 7.3
chip-level resets with their table of causes, 7.4 the power-on state
machine, 7.5 the subsystem reset controller), 6.4 (POWMAN's CHIP_RESET
and its password), 12.9 (the watchdog's REASON and trigger), 3.5.8 (the
rescue reset), 3.8.5.3 (what a RISC-V hart's reset controls are), 13.9
(ARCHSEL and the warm reset that samples it), 5.2 (the bootrom's boot
path); Appendix E, RP2350-E19. The drivers: `brio/rp2350/reset.hpp` (the
causes and the two reboots) and the power-on state machine beside the
subsystem reset controller in `brio/rp2350/resets.hpp` - the controller
itself is [platform.md](platform.md)'s, which every chapter reaches its
own block through. The reference suite: `test_rp2350_timer`, letters a,
n and i.

## What the silicon does

THREE TIERS (7.1), and a program meets all three:

- **Chip-level** resets take the whole device down: the power-on reset,
  brown-out, the RUN pin, a debugger's request, the rescue port, a supply
  glitch, a switched-core power-down, the RISC-V debug module's
  non-debug-module reset, and a watchdog event configured to reach that
  far. Each one runs the power-on state machine afterwards.
- **System** resets are the stages of that machine (7.4): the
  oscillators, the clock generators, the bus fabric, the memories, SIO,
  the access controller, the processors - released in a fixed sequence,
  each stage waiting for the one before. A watchdog event restarts the
  sequence from whatever PSM_WDSEL names. This is the tier an ordinary
  reboot takes.
- **Subsystem** resets are the peripheral blocks (7.5), which come up
  HELD and are released by software - which is why "open the clock" on
  other families is "release the reset" here. That tier is
  [platform.md](platform.md)'s: every chapter of this stratum starts by
  releasing its own block.

THE RECORD MOVED AND WIDENED. CHIP_RESET lived in the RP2040's LDO block;
here it is POWMAN's, in the ALWAYS-ON power domain, so it outlives a
power-down of the core - and it carries twelve latches where the RP2040
had three. Beside it WATCHDOG.REASON names the LAST watchdog event, TIMER
or FORCE, each replacing the other. `Reset::causes()` reads both as one
word.

AND THE WORD IS NOT A CAUSE. WHAT THE CHIP-LEVEL HALF IS EXACTLY, THE
DATASHEET SAYS TWO WAYS: 7.3.3 calls CHIP_RESET "the source of the last
chip-level reset", which would make it one cause at a time, while the bit
names are latches (HAD_*), 7.3.1's table describes each source in terms
of what it "resets" rather than what it clears, and the RP2040's
equivalent word was measured to stand for the life of the supply. Either
way, "what caused THIS boot" is the comparison of the word with the one
the previous life saw - which is how the bench suite reads it, through a
token it banks before each leg; the two readings differ only in what the
comparison should find, and the suite reports which this silicon is.

ONE MORE THING CLEARS REASON HERE, and not on the RP2040 (12.9's own
note): a DEBUGGER WARM RESET of either core - the Arm SYSRESETREQ or the
RISC-V hartreset - clears WATCHDOG.REASON, so that code loaded under a
probe after a time-out does not go on reading the time-out.
`Reset::core()` is that same signal from software.

THE TWO WAYS A PROGRAM ENDS ITSELF ARE NOT THE SAME ON THE TWO HALVES.

- `software()` is the watchdog's trigger with PSM_WDSEL selecting every
  stage but the two oscillators: the whole system through the bootrom,
  both cores, the crystal kept, REASON.FORCE as its mark. It is the
  reboot BOTH architectures have, and it is what ResetReporter and
  `fault_reset()` call.
- `core()` is the processor's own reset request, and it exists on the
  Cortex-M33 half alone. On Hazard3 the hart reset controls -
  `dmcontrol.hartreset` and `dmcontrol.ndmreset` - are IN THE DEBUG
  MODULE (3.8.5.3), reachable by a debugger and by no instruction, and
  the RISC-V cores have no cold reset domain at all (13.9). So the verb
  is refused at compile time there, with the guard on its argument, and
  the suite's reset letter has five legs on one half and four on the
  other.

A WARM RESET IS WHERE THE ARCHITECTURE IS SAMPLED (13.9): a core reads
ARCHSEL when its warm reset is released, whether that reset came from the
power-on state machine, the watchdog, SYSRESETREQ or hartreset. That is
the mechanism behind this target's whole build shape - the bootrom writes
ARCHSEL from the image it found and resets the cores into it - and it is
why `reset run` from a probe restarts a program in its own architecture
without touching the clock tree, the pads or the peripherals.

POWMAN IS READ AND NEVER WRITTEN BY THIS STRATUM. Every register of that
block up to offset 0xac, CHIP_RESET among them, takes a write only with
the password 0x5AFE in its top half (6.4), and what those registers do
belongs to the power chapter. The two flags a program might think to
clear are the bootrom's anyway: RESCUE_FLAG is read and cleared by the
boot path before any user code runs (5.2, 3.5.8), so `rescue_flag()`
ordinarily reads false even after a rescue and `ResetCause::rescue` is
what answers the question; DOUBLE_TAP is the bootrom's alternate-boot
request.

## Types and verbs

- `ResetCause`: `power_on`, `brown_out`, `run_pin`, `debug_port`,
  `rescue`, `watchdog_powman_async`, `watchdog_powman`,
  `watchdog_swcore`, `swcore_powerdown`, `glitch_detect`,
  `hazard3_sys_reset`, `watchdog_psm`, `watchdog_timer`,
  `watchdog_force`; `all`, and the two halves `chip_level` (CHIP_RESET's
  twelve bits, in the always-on domain) and `watchdog` (REASON's two,
  which a processor's own reset clears).
- `Reset::causes()` (both registers as one word, read-only and unchanged
  by reading), `rescue_flag()`, `double_tap()`.
- `Reset::software()` - the reboot both halves have, never returns.
- `Reset::core()` - the processor's own reset request, the Cortex-M33
  half's alone; `Reset::core<CoreKind::hazard3>()` is refused by both
  compilers and a plain `core()` by exactly the build that has not got
  it.
- `ResetReporter` - the panic Reporter that reboots instead of spinning,
  so the breadcrumb is read at the next boot.
- `fault_reset<P>(context)` - the fault body an app binds to the crt's
  entry, which is `isr_hardfault` on the Arm half and
  `isr_riscv_exception` on the RISC-V one: two names because the two crts
  have two tables, one function because the wreck is the same. Not
  through `panic()`, whose breakpoint instruction would escalate into the
  very fault this body serves.
- In `brio/rp2350/resets.hpp`, beside the subsystem controller
  ([platform.md](platform.md)): `PsmStage` (every stage of the sequence
  as a mask, with `all`, `oscillators` and `reboot`) and `Psm`
  (`watchdog_resets`, `done`, `hold`, `release`, `held`) for the middle
  tier. FRCE_ON is deliberately absent - 7.4.2 calls it a development
  feature that does nothing on a production device.
  [watchdog.md](watchdog.md) has the table of the three WDSEL registers.

## How to use it

```cpp
const uint32_t causes = brio::Reset::causes();       // once, at boot
const auto record = brio::take_panic_record<P>();

extern "C" void isr_hardfault()       { brio::fault_reset<P>(); }   // Arm
extern "C" void isr_riscv_exception() { brio::fault_reset<P>(); }   // RISC-V
brio::panic<P, brio::ResetReporter>(brio::PanicCode::assert_failed, 7);

brio::Reset::software();                             // the reboot, both halves
```

An app may bind both fault names unconditionally and ask the
preprocessor nothing: each image links the one its own crt declares,
and the other is a function nobody calls.

## Not covered yet

Driver gaps, each with its reason:

- Writing anything in POWMAN, the password included: the whole block is
  the power chapter's, and the two flags a reset story might want to
  clear are the bootrom's own. Until that chapter, the always-on tier of
  this word is read and reported.
- Brown-out detection, the glitch detector and the switched-core
  power-down as causes a program can PROVOKE: the first wants a supply to
  lower, the second is read-only here (a detector that fires is a reset
  the bench cannot argue with), the third belongs to the power chapter.
  Their latches are decoded and reported; nothing here raises one.
- The RUN pin and the rescue port as causes: the pin needs a wire to
  ground and a hand on the desk, the rescue port is the flash verb's own
  path and marks the boot it recovers - both are one verdict to add to
  the suite's first letter when a run happens to carry them.
- A second core's reset, and what SYSRESETREQ does to the core that did
  not ask: that is the multicore chapter, which this stratum has not
  written.

Implemented but not bench-verified, each with the letter of
`test_rp2350_timer` that will measure it:

- The causes word read twice unchanged, some chip-level source named for
  this boot's lineage, the rescue flag down under user code, and the
  power-on state machine holding no stage (letter a).
- WHICH OF THE TWO READINGS OF CHIP_RESET IS TRUE - a word that
  accumulates, or one that holds the last chip-level source alone. Leg 1
  prints the word before and after a reboot and asserts that it is one of
  the two; the printed pair says which (letter i, leg 1).
- `Reset::software()` marking REASON.FORCE and the power manager's
  HAD_WATCHDOG_RESET_PSM (letter i, leg 1).
- A watchdog time-out marking REASON.TIMER and clearing FORCE (leg 2).
- A panic through `ResetReporter` and a fault through `fault_reset()`
  read at the next boot with their code and context (legs 3 and 4). The
  fault leg reaches the crt's entry through a breakpoint instruction with
  no debugger attached; on the Hazard3 half that depends on the probe
  having left `dcsr.ebreakm` clear, which is the ordinary state after a
  `reset run` and is not guaranteed.
- `Reset::core()` leaving the chip-level latches untouched AND CLEARING
  the watchdog's two bits - the difference from the RP2040 (leg 5, the
  Arm half only; the RISC-V half closes the letter at leg 4 with the
  verdict that says why).
- The panic breadcrumb in `.noinit` written, taken once and gone, with no
  reset in between (letter n).
- Whether the `.noinit` SRAM survives each of these resets, which the
  datasheet promises nowhere (letter i, every leg, beside the scratch
  registers).
