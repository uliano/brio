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

AND THE WORD IS NOT A CAUSE - NOR DOES A REBOOT APPEAR IN IT. 7.3.3 calls
CHIP_RESET "the source of the last chip-level reset", and CHIP-LEVEL is
the operative word: a watchdog event that runs the power-on state machine
is a SYSTEM reset, and the chip-level half comes through it bit for bit
unchanged (measured across a software reboot, a time-out, a panic and a
fault). What would make a watchdog event reach that record is POWMAN's
own WDSEL - RESET_PSM and the three bits under it (6.4) - whose reset
value is zero and which nothing in this stratum writes, POWMAN being the
power chapter's whole. So HAD_WATCHDOG_RESET_PSM and its three
neighbours stand for resets this framework never asks for, and a program
answers "what caused THIS boot" from WATCHDOG.REASON and from its own
survivors, never from the chip-level half. The bench suite reads it that
way, through a token it banks before each leg.

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

## Bench findings

On an RP2350 in the QFN-80 package, **stepping A2**, and on BOTH
architectures. The boot letter is part of `test_rp2350_timer`'s **93
pass, 0 fail** on each half; the reboot letter runs by name and reports
**28 pass, 0 fail** on the Cortex-M33 pair over five legs and **24 pass,
0 fail** on the Hazard3 pair over four.

- **A REBOOT LEAVES THE CHIP-LEVEL RECORD ALONE.** Across a
  `software()` reboot the word reads **0x10 before and 0x10 after** - the
  rescue latch of the flash path that loaded the image, and nothing
  added. The same holds for a watchdog time-out. HAD_WATCHDOG_RESET_PSM
  is never raised by anything this stratum does, because POWMAN_WDSEL
  stays at its reset value: the tier is SYSTEM, and the always-on record
  is for the chip-level tier alone. Reading the causes twice does not
  change them.
- **The rescue flag is down under user code**, as 5.2 says it must be -
  the bootrom reads it first and clears it to acknowledge - while
  `ResetCause::rescue` beside it still names the boot's lineage.
- **WATCHDOG.REASON is the cause that answers.** `software()` marks
  FORCE and clears TIMER; a time-out marks TIMER and clears FORCE; a
  panic through `ResetReporter` and a fault through `fault_reset()` both
  mark FORCE, being that same reboot.
- **BOTH SURVIVORS CROSS EVERY ONE OF THESE RESETS** - the watchdog's
  scratch registers and the `.noinit` SRAM - on every leg and on both
  architectures, carrying the same token. That settles 7.2 against
  7.3.1's table for this tier, and it makes the `.noinit` breadcrumb a
  thing a program may rely on across a reboot, which the datasheet
  promises nowhere.
- **The breadcrumb arrives with its code and its context**: a panic
  written through `ResetReporter` is read at the next boot as
  `queue_overflow` with the context the app gave, and a breakpoint
  instruction with no debugger attached reaches the crt's fault entry on
  BOTH halves - `isr_hardfault` on one and `isr_riscv_exception` on the
  other - leaving a `kernel_fault` breadcrumb with its own context. The
  RISC-V leg depended on the probe having left `dcsr.ebreakm` clear, and
  after an ordinary `reset run` it was.
- **`Reset::core()` is not a chip-level reset and it wipes REASON.** On
  the Arm half the chip-level latches come through it exactly as they
  were, while the watchdog's two bits are gone - which the RP2040's did
  not do, and which is why code loaded under a probe here does not go on
  reading an old time-out. The RISC-V half has no such verb (it is
  refused at compile time) and its letter closes a leg earlier.
- **Every reboot completed through the bootrom**, in both architectures,
  with the power-on state machine's hold register clear beforehand
  (erratum RP2350-E19's condition).

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

Implemented but not bench-verified, each with what would measure it:

- WHETHER THE CHIP-LEVEL LATCHES ACCUMULATE OR REPLACE. Every boot of
  this bench arrives through the same chip-level source, the flash path's
  rescue, so the word has never had to hold two: it would take a boot of
  one chip-level kind followed by one of another - a RUN-pin reset after
  a rescue, say - and both of those want a hand or a wire on the desk.
- The decoded latches for brown-out, the glitch detector, the
  switched-core power-down, the RUN pin and the debug port: each is read
  and reported and none has been raised, for the reasons in the driver
  gaps above.
