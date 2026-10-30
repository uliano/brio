# Platform (CH32V203 and CH32V303)

This document is for a programmer putting a brio program on a CH32V203
or a CH32V303. It describes what the kernel stands on here, the
platform, and the timebase the program's time events run on. A brio
kernel asks the chip for a handful of things, stated by the kernel's
`Platform` concept (a concept is a compiler-checked set of requirements
on a type;
[design/kernel.md, "Platform: what the machine provides"](../design/kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp)):
a way to mask interrupts for a critical section, a way to sleep until
the next interrupt without losing one that arrives at the wrong moment,
a source of time, a breakpoint for a panic, and a record a panic leaves
for the next boot. `Ch32vx03Platform<TB>` is this family's answer, and
`TB` is its timebase: the `Ticker` on the core's own 64-bit system
counter (the STK), a 1000 Hz interrupt. The page also covers the one
spelling of an interrupt handler on this target, the same on both
parts, whose expansion the F extension and the `CH32VX03_HPE` option
choose, and the failing half: which reset happened, how to cause
one, and the record a crash leaves for the next boot. Over the running
half sits the platform of the preemptive kernel,
`Ch32vx03RubatoPlatform<TB>`: its scheduling request is the software
interrupt, with nesting turned on at run time for that one line, and
where its scheduler runs is stated under "Types and verbs".

The smallest use, the platform's lines of the block under "How to use
it" below:

```cpp
using P = brio::Ch32vx03Platform<>;

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
...

int main() {
    const uint32_t flags = brio::Reset::take_flags();    // first, and once
    const auto record = brio::take_panic_record<P>();    // fetch-and-clear
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Tenuto<P, ...>::run();
}
```

Line by line: `P` names the platform with its default timebase, the
empty angle brackets taking the `Ticker`. The second line binds the
STK's interrupt vector to `Ticker::tick()`: `systick_handler` is the
name the crt (the startup code that runs before `main()`,
`ch32vx03/src/glue/startup_ch32vx03.S`) gives that vector, and
`BRIO_CH32_LEAF_VECTOR` is the binding's form for a leaf, a handler that
compiles to no call and touches no floating-point register, which the
tick is (`Ticker::tick()` is always inlined into it). The `...` line
after it stands for the two fault bindings "How to use it" adds, for a
program that wants a crash recorded. `flags` is the history of reset
causes since the last clear, read and cleared once at boot
([reset.hpp](../../brio/ch32vx03/reset.hpp)); `record` is the record of
a panic that ended the previous run, if any, for the program to report
(kernel/panic.hpp); a program that reports neither may discard both.
`SysClock::init()` is the clock task and `clock` its tag object (an
empty object whose type carries the rates, the argument each driver's
`init()` takes), declared as in [clock.md](clock.md)'s smallest use
(`using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;`
and `constexpr SysClock clock;`). `Tenuto<P, ...>` is the kernel itself,
the cooperative scheduler ([design/kernel.md](../design/kernel.md)), and
the `...` inside it stands for the program's active objects in priority
order (an active object is a component of a brio program with its own
event queue, served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)); `run()` never
returns.

What decides whether that works:

- **The clock task first.** The ticker computes its compare from the
  rate the clock type claims (`Clock::hz / 1000 - 1`), not from the
  hardware, so it is right only once the tree runs at that rate: a
  ticker set up for 144 MHz on a chip still at its 8 MHz HSI reset rate
  ticks eighteen times too slowly (derived from the compare arithmetic,
  not measured), and nothing refuses it - `Ticker::init()` answers false
  only when the period comes out zero. `SysClock::init()` answers
  false when a root never starts, the PLL never locks or the switch
  never takes, and the chip is then not at the rate the type claims (on
  the HSI, possibly with the type's prescalers already written,
  [clock.md](clock.md)); a program looks at both answers.
- **The vector bound by the program.** The kernel binds no vector.
  Every handler the crt's vector table names is a weak alias of
  `default_handler`, a spin loop, until the program defines it: a
  program that enables the tick and forgets its binding stops in that
  spin at the first tick, with no fault and no message
  (the probe names the line: [README.md](README.md), "Debugging").
- **The one spelling of a binding.** A vector is bound with
  `BRIO_CH32_VECTOR(name) { body }` or one of its two forms, never by
  writing an interrupt attribute by hand: the macro follows the
  project's `CH32VX03_HPE` option (a CMake option of the `ch32vx03/`
  project, chosen at configure, on by default: with it the crt sets
  INTSYSCR.HWSTKEN and the core's hardware prologue saves the
  caller-saved integer registers, so a handler must carry WCH's fast
  attribute, and one switch serves the whole image because a fast
  handler under a prologue that is off corrupts the program it
  interrupted). On the CH32V303, whose images are built with the F
  extension, and with the hardware prologue on, `BRIO_CH32_VECTOR` makes
  a naked trampoline (a vector with no prologue of its own that calls
  the body as an ordinary function) whose body must do no floating-point
  work, because
  nothing saves the interrupted program's f-registers; a body that does
  float work binds with `BRIO_CH32_VECTOR_FLOAT`. `brio check ch32vx03`
  holds every F image to those promises ("Types and verbs" below); an
  ordinary build does not run that check, so a body that breaks the
  promise compiles and links, and the interrupted program's f-registers
  are then not saved.
- **Interrupts enabled before `run()`** (`brio::enable_interrupts()`),
  because that is `run()`'s own requirement: it is called once, with
  interrupts enabled and the clock and the timebase set up
  ([kernel/tenuto.hpp](../../brio/kernel/tenuto.hpp)).
- **No sleep while a bus master works.** In a sleep of any depth on
  this family no bus master but the core gets a cycle, so `idle()`
  returns at once, interrupts enabled, while a DMA channel is enabled
  or a USB controller is attached (the count of active bus masters,
  [sleep.md](sleep.md)): such a program spins instead of sleeping.
  There is no tickless timebase here: the STK stops with HCLK in a
  Stop, and the kernel's optional `idle_until()` hook is not provided.

The rest of this page, in order: the documents of record; what the
silicon does (the interrupt entry, how the core sleeps and wakes, the
hardware prologue, the STK, the reset flags and the reset request, the
trap vectors, corecfgr, and the V4F's floating-point unit with what an
interrupt pays for it), with "Code in SRAM buys nothing here"; the
types and verbs (the platform, the three vector bindings and the check
that holds them to their promises, the timebase and the busy-wait, the
failing half); how to use it; the bench findings, on the CH32V203C8T6
and the CH32V303VCT6, not every one on both; and what is not covered.

The platform is realized on the QingKe V4B of the CH32V203 and the
QingKe V4F of the CH32V303 - the same core with a single-precision
floating-point unit, RV32IMAFC where the other is RV32IMAC - in two
halves: the running half - the critical section, the idle hook, the
STK timebase, the microsecond busy-wait, the interrupt round trip, and
on the V4F the floating-point unit's state and what an interrupt pays
for it - and the failing half - which reset happened, how to cause
one, and the record a crash leaves for the next boot.

## Documents of record

The CH32F/V20x_V30x_V31x reference manual V2.3
(3.2 for the reset sources and 3.4.10 for the flags, 9.5.2 for the
PFIC's registers, read for the CH32V20x_D6 class of the CH32V203C8 and
the CH32V30x_D8 class of the CH32V303) and the QingKe V4 microprocessor
manual V1.1 (table 1-1 for each model's nesting and hardware-prologue
depths, 2.1 for the exception codes, 2.2 for what a trap does to
mcause, mepc, mtval and mstatus, 3.1 for the PFIC's pending, active,
priority and global-status registers, 3.2 and 3.4 for the interrupt
CSRs and the hardware prologue - its note 3 for what that prologue does
not save - 3.3 for nesting, 5 for the system timer, 6 for the sleep
modes, 8 for the CSR list and 8.2 for mstatus.FS and the floating-point
CSRs). No errata
sheet exists for this family; the bench findings below are the entries
this document keeps in its place.

The headers are [platform.hpp](../../brio/ch32vx03/platform.hpp),
[pfic.hpp](../../brio/ch32vx03/pfic.hpp) (the mask, the per-line
enables and the three vector bindings),
[ticker.hpp](../../brio/ch32vx03/ticker.hpp),
[delay.hpp](../../brio/ch32vx03/delay.hpp) and
[reset.hpp](../../brio/ch32vx03/reset.hpp); the crt is
[startup_ch32vx03.S](../../ch32vx03/src/glue/startup_ch32vx03.S), and
the check that holds the F images' vectors to their promises is
`cli/vector_guard.py`. The reference suite is `test_vx03_platform` (a
suite's letter is one command key of its console and the group of
verdicts it runs; `z` is the key that runs every letter but `i`, which
reboots the board three times).

## What the silicon does

- **Interrupt entry does not clear MIE** (V4 manual 2.2), so a handler
  runs with the global enable still set; the PFIC decides whether
  another line may preempt it, and with INTSYSCR.INESTEN off - which is
  what the crt leaves - it holds the next line until MRET.
- **Nesting is INTSYSCR's INESTEN with the priority bytes' preemption
  bits** (3.2, 3.3): out of reset, and as the crt leaves it, nothing
  nests. With INESTEN set and one preemption bit (PMTCFG = 01), a line
  whose priority byte has bit 7 clear preempts the handler of one whose
  byte has it set, and two lines of one preemption level never preempt
  each other. Measured on both cores: a line at 0x00 raised inside the
  handler of a line at 0x80 is taken there, PFIC_GISR's NESTSTA reading
  0x03 (the second level) against 0x01 at the first, and the same line
  with both at 0x00 waits for the MRET. The outer handler's mepc, mcause
  and mstatus read the same after the nested level as before it: the
  controller keeps them per level. Table 1-1's figures are the model's:
  the V4B nests two levels and its hardware prologue is two deep, and it
  ignores PMTCFG (INTSYSCR reads 0x3 after a write of 0x7, the one
  preemption bit fixed); the V4F nests eight with a prologue three deep
  and takes the field (0x8003, PMTSTA 0x80). The priority byte keeps
  bits 7:5 on both (0xF0 reads back 0xE0).
- **A pend through PFIC_IPSR is three instructions late**, and a
  `fence.i` after the store takes it at once: measured on both cores,
  three instructions retire after the store before the trap, two behind
  a `fence` or a load of the PFIC, none behind a `fence.i`; the pending
  bit reads clear in the handler, and a line pended inside its own
  handler is taken after the MRET. With a line pending under the mask,
  the `csrsi` that lifts it lets two more instructions retire on the V4B
  before the trap and none on the V4F.
- **An MRET into a rewritten mepc lands with the interrupted code's
  registers**, the hardware prologue popping the sixteen it pushed at the
  trap's entry: sixteen patterns set before a pend were all found at the
  address the handler wrote into mepc, on both cores. The machine-mode
  `ecall` arrives at the table's entry 5 with mcause 11, the prologue
  pushing and popping around it as around any trap, and no line
  preempts its handler (a line pended inside it is taken after its
  MRET).
- **A WFI sleep ends on an interrupt the controller responds to**
  (6.2), and with mstatus.MIE clear it responds to none: a `wfi`
  executed masked on the CH32V203C8T6 slept past the STK and TIM2, both
  pending and enabled, for as long as it was left - twenty seconds, and
  five in a second run - until a debugger's halt woke it (measured; 6.2
  lists a debug request among the wakes). The core can sleep as a WFE
  instead: PFIC_SCTLR.WFITOWFE turns `wfi` into a wait-for-event and
  SEVONPEND makes every interrupt entering the pending state an event,
  latched, so a WFE after the event returns at once. 6.2 gives a WFE
  three wakes that matter here - an event; an enabled interrupt "when
  an interrupt is generated", whose handler then runs first; and,
  through SEVONPEND, "a new interrupt pending signal (the previously
  generated pending signal does not take effect)" - and only the last,
  an edge, reaches a core sleeping with MIE clear. The platform
  therefore sleeps with MIE set.
- **The waking edge is latched too.** An interrupt that wakes a WFE
  enters the pending state, so SEVONPEND latches it as well, and the
  next WFE returns at once on it. PFIC_SCTLR.SETEVENT (V4 manual 3.1,
  "set the event to wake up the WFE case") sets the latch by hand, and
  a WFE executed over a set latch returns at once and clears it - the
  pair that consumes it. WFITOWFE and SEVONPEND stay set after
  a `wfi` (measured: a WFE with no store before it still waits for an
  event), so "the subsequent WFI" is not a one-shot on this core.
- **A core in debug mode cannot enter any sleep mode at all** (6.1), so
  a sleep measured with a probe halted on the core is not a
  measurement.
- **The hardware prologue/epilogue (HPE)** saves sixteen caller-saved
  registers (x1, x5..x7, x10..x17, x28..x31) in one cycle to an
  internal stack the program cannot see, two levels deep on the V4B and
  three on the V4F (table 1-1), and pops them on MRET (3.4). This is where the two QingKe cores differ: the V2C
  of the CH32V00x pushes its ten registers to the user stack, so there
  the hardware buys only the fetch of a prologue, while here it buys
  the pushes themselves (measured below: 64 bytes of stack and 10
  cycles). It is enabled by INTSYSCR.HWSTKEN and used by a handler
  declared with the vendor compiler's `interrupt("WCH-Interrupt-fast")`
  attribute, which then emits no prologue of its own.
- **The STK is sixty-four bits wide** (5): CNTL/CNTH count up (or down,
  MODE) at HCLK or HCLK/8 towards CMPLR/CMPHR, with an auto-reload
  (STRE) and a flag the handler must clear by writing zero to it. It is
  interrupt 12 of the vector table, and the interrupt number is the
  table index on this core. Its control register also carries SWIE, the
  software interrupt's other way in.
- **The reset flags accumulate** in RCC_RSTSCKR until RMVF is written
  (3.4.10), and the register's own reset value is 0x0C000000: a
  power-on raises PORRSTF and PINRSTF together, so the pin flag does
  not name the pin at the boot that follows one. This family carries
  six flags where the CH32V00x carries seven of its own: the two
  watchdogs, software, power, pin, and LPWRRSTF - the low-power reset
  a Standby or Stop entry becomes when the option bytes say so (3.2.2).
- **A reset request is PFIC_CFGR.RSTSYS** written with KEY3 (0xBEEF in
  the high half, 9.5.2.10); PFIC_SCTLR bit 31 is the same reset with no
  key. It reads back as SFTRSTF, alone (measured).
- **`ebreak` with no debugger attached is taken to the vector table's
  breakpoint entry, index 9** (measured), not to the exception entry at
  index 3 - while mcause reports exception code 3, which is that
  manual's number for a breakpoint. The vector index and the exception
  code are two different numbers here, and a program that wants a crash
  recorded binds both entries.
- **The cause of a trap is in mcause, mepc and mtval** (2.2): the cause
  register's top bit says interrupt or exception and the rest is the
  code of table 2-1 (or the interrupt number), mepc holds the
  instruction that trapped and mtval the address or opcode behind it.
  mstatus.MPIE keeps the MIE the trap interrupted.
- **corecfgr (CSR 0xBC0) configures the pipeline and the instruction
  prediction** and the manual gives no bit table (8.3); WCH's own
  startup file writes 0x1F into it, and so does this crt. It is
  writable at run time and reads back what is written (measured), and
  what those five bits buy is measured below.
- **The V4F's floating-point unit is off out of reset.** mstatus.FS
  (bits 14:13) is its state - 00 Off, 01 Initial, 10 Clean, 11 Dirty -
  and with FS Off every FP instruction, fcsr's own accesses among them,
  raises an exception (8.2). The crt of an image built with the F
  extension writes FS = 01 and then clears fcsr, and that write is
  itself a change of FP state: main() finds FS = 11 (measured, and a
  write to fcsr alone measured to move Initial to Dirty). The state is
  only on or off to a program with no context switch. The unit records
  its exceptions in fcsr's five sticky flags and never traps on them,
  and the rounding mode in frm is honoured (measured, below).
- **The hardware prologue saves integer registers and nothing else**
  (3.4, note 3): the sixteen above, never an f-register. WCH's compiler,
  under the fast attribute and the ilp32f ABI, stores in a handler's own
  prologue every caller-saved f-register the handler clobbers, and all
  twenty of them (ft0..ft11, fa0..fa7) once it calls a function -
  without looking at the callee, a visible float-free one included - 80
  bytes of user stack and forty memory accesses, placed at the entry
  ahead of the branch that calls or shrink-wrapped onto it as the
  function's shape decides. So in an image built with F a vector is not
  an attributed handler but a naked trampoline, a call of an ordinary
  function and MRET (Types and verbs, below): the body saves what the
  ABI asks of a callee and no caller-saved register, and since nothing
  then saves the f-registers of the program it interrupts, the
  stratum's check walks every trampoline's body in every linked image
  and fails on a floating-point instruction it reaches. A leaf - a
  handler that calls nothing and touches no f-register - saves nothing
  under the fast attribute, F or not, so it is bound as the attributed
  handler still (the leaf form), the check holding it to being one.
  Measured on the CH32V303VCT6, a trampoline against the attributed
  handler in images built from one source: a receive vector that posts,
  65 cycles a byte against 105 (the twenty saves gone, the kernel's
  `post` reached by a tail jump), about 35 cycles less a transmit
  interrupt; a handler that calls out 65
  cycles of round trip against 100 (letter `l`); a leaf 53 to 66 cycles
  bound leaf against 57 to 70 as a trampoline, the call and its return
  (letter `e`); and the I2C suite's refused last byte at 8 MHz and 400
  kHz, lost with the saves and kept without them ([i2c.md](i2c.md)).
  Over the 24 images of the part, one handler saves f-registers - the
  float trampoline of letters `k` and `l` - where every vector bound by
  the attribute would make 68 (66 of them saving all twenty); 89
  vectors are leaves bound as such, and the images' flash is 4864 bytes
  less than with every vector attributed (4552 for the trampolines, 312
  for the leaves bound without one).

### Code in SRAM buys nothing here

Other families let a program place a driver's hot path in SRAM, opt-in
per driver ([../../brio/util/placement.hpp](../../brio/util/placement.hpp)),
because their code flash answers behind wait states. This one's does
not where the image runs: both datasheets count a part's flash as its
zero-wait run area (the CH32V203's note 1 to table 2-1, the CH32V303's
to table 2-1-1), which the hardware loads at boot - the CH32V303
datasheet's table 4-18 counts that "code load time" in a wake from
Standby ([nvm.md](nvm.md), "The window and the tail") - and every
linker script of this project places the image inside it. WCH's own
startup puts its `.highcode` section in the code flash, ahead of
`.text`, and not in SRAM (the EVT packages' `Link.ld`). Measured on both
dies, the same code timed on the STK from the window and from SRAM
(the function copied there with `.data`), the best of sixteen runs:

| the code | window | SRAM |
|---|---|---|
| a four-instruction loop with its taken branch, 32-bit or compressed, per iteration | 5.00 cycles | 5.01 |
| 64 straight 32-bit `addi` and a `ret`, a call | 72 (73 at 96 and 8 MHz) | 77 (78) |
| a loop of calls to an empty function, from the window, per call | 9 at 144 MHz on the CH32V203C8T6, 11 on the CH32V303VCT6; 8 at 96 and 8 MHz on both | 13 at 144 MHz, 14 at 96 and 8 |

Identical on the CH32V203C8T6 and the CH32V303VCT6 but for the call's
rate at 144 MHz: SRAM never fetches faster than the window, it is a
cycle slower on straight code and four to six slower on a call into it
from the image. So the platform does not say `places_fast_text`, no
driver of this stratum takes a placement tag, and `InRam` on this
platform does not compile (util/placement.hpp's `HotPlacement`). The
tail above the window is the one code flash here behind a wait - 20
cycles a word read (nvm.md) - and no script places code there.

The two cores part at a taken branch: in a loop where any predictor is
warm, a turn with a forward branch taken costs one to three cycles more
than the same turn with it not taken on the CH32V203C8T6's V4B and none
to one more on the CH32V303VCT6's V4F, and a taken branch to a 32-bit
instruction straddling a word two more than to an aligned one on the
V4B, one more on the V4F - where every load, store, multiply and
straight-line fetch `bench_vx03`'s letter i prices costs the same on
both. It is what makes the same handler cheaper on the V4F
([i2c.md](i2c.md), the pump's handler).

## Types and verbs

The code below is the QingKe core stratum's, written once for the three
QingKe families ([../qingke/README.md](../qingke/README.md)): this
family's [pfic.hpp](../../brio/ch32vx03/pfic.hpp),
[ticker.hpp](../../brio/ch32vx03/ticker.hpp) and
[delay.hpp](../../brio/ch32vx03/delay.hpp) are its device header over
[brio/qingke/pfic.hpp](../../brio/qingke/pfic.hpp),
[brio/qingke/ticker.hpp](../../brio/qingke/ticker.hpp) and
[brio/qingke/delay.hpp](../../brio/qingke/delay.hpp), and its platform
type a struct over
[brio/qingke/platform.hpp](../../brio/qingke/platform.hpp)'s
`QingkePlatform`, given this family's facts by a chip traits type
(`Ch32vx03Qingke`: the bus question, the count of working masters) and,
for Rubato,
[brio/ch32vx03/schedule.hpp](../../brio/ch32vx03/schedule.hpp) over
[brio/qingke/schedule.hpp](../../brio/qingke/schedule.hpp), the family's
one fact there the request's `fence.i`. The core files' comments carry
the measurements of all three families' cores, each with its chip named.

`Ch32vx03Platform<TB>`
([brio/ch32vx03/platform.hpp](../../brio/ch32vx03/platform.hpp)) is the
concept member for member: `CriticalSection` is pfic.hpp's
`InterruptGuard` (one `csrrci` reads and clears mstatus.MIE, the
destructor restores only what it found set), `idle()` is the
WFITOWFE/SEVONPEND store (SLEEPONEXIT cleared in it, so the waking
handler returns to the caller's loop), the unmask, then the `wfi` as a
WFE, and after the wake SETEVENT and one more `wfi`, which returns at
once and consumes the latch the wake left, so the caller's loop turns
once per interrupt (with MIE set by then, an interrupt pending around
the consume is taken by its level, not found by the latch) - guarded by
the count of active bus masters and, with a deep mode armed, out of
line with a pause of the timebase and no consume ([sleep.md](sleep.md))
- `break_here()` is `ebreak`,
`atomic_width` 4, `now()` the timebase's tick count, and the breadcrumb
a `PanicRecord` in `.noinit`. The interrupt verbs and the per-line
enables are [brio/ch32vx03/pfic.hpp](../../brio/ch32vx03/pfic.hpp):
`enable/disable/enabled/pending/set_pending/clear_pending/active` (the
manual's ISR bank is the enable status and its IPR the pending one;
IENR/IRER/IPSR/IPRR are write-one), and `Pfic::in_handler()`, which the
platform's `in_handler()` forwards and the Rubato platform inherits
(kernel/platform.hpp's optional member, `HandlerAware`): PFIC_GISR's
GACTSTA, bit 8 at offset 0x4C, an interrupt being executed at any
nesting level (the reference manual 9.5.2.11, the QingKe V4 manual
3.1) - not mstatus.MIE, which a critical section clears as a trap's
entry does. `BRIO_CH32_VECTOR(name) { body }`,
with its two forms, is the one spelling of a vector binding, and it
follows two facts of the build. Without the F extension (every CH32V203
image) it is an attributed handler under the name - WCH's fast attribute
when the image is built with `CH32VX03_HPE` (the default), gcc's plain
`interrupt` otherwise, one option for the whole image because a fast
handler under an HPE that is off corrupts the program it interrupted -
and the attribute carries `no_icf`: gcc's identical code folding turns
the second of two handlers of one body into a handler that calls the
first, which ends in MRET, so the call never returns and the caller's
frame stays on the stack; on the V4F that frame is the twenty
f-registers such a caller saves, eighty bytes (measured on the
CH32V303VCT6: the interrupted program's next return jumped to a saved
float). With F and the hardware prologue (the CH32V303's images) it is
the naked trampoline: `name` is two instructions - a `jal` to an ordinary
`extern "C"` function `name_body` holding the braces (two bytes
compressed where the body lies within its reach, four otherwise) and
MRET - and the body ends in `ret`; two bodies that fold are ordinary
functions, for which a fold returns where it should. With F
and the hardware prologue off it stays the attributed handler of the
plain `interrupt`, whose software prologue saves whatever a call may
clobber, f-registers included. `BRIO_CH32_VECTOR_FLOAT(name)` is the
binding of a body that does float work: the same trampoline with the
twenty caller-saved f-registers stored on the user stack around the
call - the set gcc saves for a handler that calls out - and fcsr in a
word of the same frame (96 bytes, the 16-byte alignment kept), which
gcc does not save: one `csrrw` reads the interrupted program's fcsr
into `t0`, a register the hardware has saved, and writes zero, so the
body starts from fcsr's reset state (round to nearest, no flag) and the
program finds its rounding mode and its sticky flags as it left them
after MRET. The reset state rather than the program's own is the
choice that makes a handler's arithmetic the same whatever it
interrupts - a program that sets another rounding mode for its own work
does not change its handlers' results with it, and the flags a body
finds are the ones it raised - and it costs nothing over inheriting,
the swap being the instruction that reads fcsr. Where there is no F or
no hardware prologue it is the plain binding. `BRIO_CH32_LEAF_VECTOR(name) { body }` is the binding of a
leaf: with F and the hardware prologue it is the attributed handler of
the fast attribute, which saves nothing for a body that calls nothing
and touches no f-register, so it needs no trampoline; the image carries
the absolute symbol `__brio_leaf_<name>` as the binding's mark, which
costs no byte of it; and wherever else it is `BRIO_CH32_VECTOR`
exactly. The promises are a check: `brio check ch32vx03`, after linking
every image, runs `cli/vector_guard.py` over the F images - from each
vector table entry that is a plain trampoline it walks the body's calls,
tail jumps and branches through the listing and fails on a
floating-point instruction, an access to fcsr, frm or fflags, an
indirect call or jump it cannot resolve (a switch's jump table it reads
from the image, every entry required inside the function) or an
undecoded four-byte instruction, naming the image, the vector and the
chain of functions; a float trampoline must store and load the twenty
and fcsr, each from its own slot, and its body is left alone; a vector
bound leaf fails on a call, a jump or a branch out of it, an indirect
transfer other than a jump table into itself, or a floating-point
instruction, naming the vector; and a plain trampoline whose body came
out a leaf is reported as a candidate for the leaf form, a line of the
output and not a failure; and any other entry of the table but the
crt's `default_handler` - a handler attributed by hand among them -
fails, the three macros being the only bindings the check can hold to
a promise. Its fixtures under `test/family_ch32vx03/guard/` are linked
and must be let through, reported or refused by name: the refusals a
float operation reached through a call, a call through a pointer, a
leaf that calls out, a leaf that does float work, a float trampoline
that leaves fcsr unsaved and a handler attributed by hand.
Interrupt nesting is never enabled by the crt or by a driver: the
kernel's rule
([design/kernel.md, "The model in one page"](../design/kernel.md#1-the-model-in-one-page)).
The preemptive kernel's platform turns it on for its scheduling request
alone, one level below every peripheral line ("Types and verbs").
`stack_untouched()` is the RAM ledger: the crt paints the free RAM
between the last section the linker placed and the stack top with one
pattern before the first call, and the verb walks up from that floor
until the pattern breaks - how many bytes of stack no call has reached
since reset.

`Ch32vx03RubatoPlatform<TB>` (the same header) is
`Ch32vx03Platform<TB>` whole plus the three members of a preemptive
platform, the one the preemptive kernel `Rubato` runs on
([../design/kernel.md](../design/kernel.md), "Rubato"): `preemptive`,
`ceiling` (the index of the object being dispatched, 255 at the loop's
base) and `pend_schedule()`, which is
[brio/qingke/schedule.hpp](../../brio/qingke/schedule.hpp)'s
`request_scheduler()`: the software interrupt pended through PFIC_IPSR
and a `fence.i` (this family's `scheduling_request_posted`), five instructions out of line. `scheduler_init()` puts
the software interrupt's priority byte at 0x80 and sets INTSYSCR.INESTEN
with one preemption bit, so every peripheral line, at the reset
priority 0, sits at one level above it: the peripheral handlers still
never nest among themselves, and every one of them interrupts the
software interrupt's. The scheduler runs in THREAD MODE: the software
interrupt's body, `scheduler_trap<K>()`, pushes a 16-byte block with the
trap's mepc and mstatus, writes the entry's address into mepc and
returns (nine instructions); the entry, `scheduler_thread<K>()`, stores
the sixteen caller-saved integer registers the hardware prologue just
popped back into the register file - the interrupted code's -, sets MIE,
calls `K::activate()`, loads them back and executes `ecall`; and the
machine-mode environment call's body, `scheduler_return()`, puts mepc
and mstatus back from the block, pops it and returns into the
interrupted code (six instructions). Without the hardware prologue the
two bodies save their one scratch register in the block themselves.
In an image built with F the entry also saves, when mstatus.FS is
Dirty, the twenty caller-saved f-registers and fcsr (96 bytes) and
writes FS = Clean, so a nested entry over a dispatch that has done no
float work saves nothing; the exit's mstatus puts the interrupted
code's FS back. fcsr is the program's: a preempting dispatch starts from
the rounding mode it preempts.
The two vectors are bound with
`BRIO_CH32_SCHEDULER_VECTOR(name)`: naked, `no_icf`, and marked a leaf
(`__brio_leaf_<name>`), which they are - no call, no f-register - so the
vector guard holds them to it in every F image and does not walk into
the scheduler, which no vector calls. A program executes no `ecall` of
its own.

The timebase is [brio/ch32vx03/ticker.hpp](../../brio/ch32vx03/ticker.hpp)'s
`BasicTicker<tps>` over the STK (`Ticker` at 1000 Hz; `init`, `tick`,
`ticks/millis/secs/now`, `pause/resume`, `advance` for a site that
froze time, `rebase` for a clock that changed rate), and the busy-wait
[brio/ch32vx03/delay.hpp](../../brio/ch32vx03/delay.hpp)'s `delay_us`:
at least, never early, no division at wait time, and refused - with no
time spent and a `[[nodiscard]]` answer - for a request of one tick
period or more, above the 65536 us gate its 32-bit arithmetic rests on,
or with the counter stopped.

The failing half is [brio/ch32vx03/reset.hpp](../../brio/ch32vx03/reset.hpp):
`ResetFlag` (the six bits by name, plus `all` and `watchdog`),
`Reset::flags/clear_flags/take_flags/software`, and for a crash the
trap readers `machine_cause/machine_epc/machine_tval`, the `FaultCause`
codes, `fault_context()` (the cause packed into the breadcrumb's one
detail byte - the interrupt bit above, the code below - with
`fault_was_interrupt`, `fault_code` and `fault_cause_name` to read it
back), `ResetReporter` (the panic reporter that resets instead of
halting) and `fault_reset<P>()`, the fault vector's body that writes a
kernel_fault record carrying that byte - never over one that already
stands - and resets; its second form takes a byte of the handler's own
instead, for a vector that has something better to say than mcause.

## How to use it

An app names the platform, binds the vectors it owns with the one
attribute, and hands both trap entries the fault body:

```cpp
using P = brio::Ch32vx03Platform<>;

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_VECTOR(fault_handler) { brio::fault_reset<P>(); }
BRIO_CH32_VECTOR(breakpoint_handler) { brio::fault_reset<P>(); }

int main() {
    const uint32_t flags = brio::Reset::take_flags();    // first, and once
    const auto record = brio::take_panic_record<P>();    // fetch-and-clear
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Tenuto<P, ...>::run();
}
```

A wait shorter than a tick is `delay_us(clock, us)`; a wait of a tick or
more is a TimeEvent and `delay_us` refuses it. A reboot on purpose is
`Reset::software()`; a panic that must be seen at the next boot goes
through `panic<P, ResetReporter>(code, context)` - and with no debugger
attached that call's own `ebreak` arrives at the breakpoint vector,
whose `fault_reset<P>()` leaves the record panic() already wrote and
performs the reset.

The same program under the preemptive kernel: the platform type, the
two naked vectors and one call more.

```cpp
#include "kernel/rubato.hpp"

using P = brio::Ch32vx03RubatoPlatform<>;
using K = brio::Rubato<P, Sensor, Logger>;   // Sensor preempts Logger

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return(); }

int main() {
    SysClock::init();
    brio::Ticker::init(clock);
    brio::scheduler_init();         // nesting on, the software interrupt below every line
    brio::enable_interrupts();
    K::run();
}
```

## Bench findings

The reference suite is `test_vx03_platform`, at 144 MHz from the HSI:
on the CH32V203C8T6 fifty-six verdicts in `z` - letters `a` to `g`
(forty), the bus in sleep, the mask's shadow, the idle hook against the
tick's edge, the kernel's turn and the runtime's functions - on the
CH32V303VCT6 seventy-one, the same with the floating-point unit's three
letters beside them - and on both nine more in letter `i`,
which reboots the board three times. On the 32 KB parts the suite is
two images (`-1` with `a` to `g`, `i`, `t` and `w`, `-2` with `m`, `n`
and `o`), run on the CH32V203C8T6 before letter `o` joined the second:
48 and 7 verdicts. What it measured:

- **The idle hook wakes.** With the console silent, two `idle()` calls
  covered the 626 us to the next tick (on the CH32V303VCT6 two as well,
  over 484 and 968 us in two runs) and returned with interrupts enabled;
  a 5 ms window with interrupts masked advanced the tick by one on both
  (the STK interrupt is a pending bit: one tick delivered, the rest
  coalesced). Two calls and not one because the console's last
  interrupt, taken while the letter printed and not inside an
  `idle()`, left its latch standing: the first call returns at once on
  it and consumes it.
- **No edge position loses the wake** (letter `w`, both parts).
  The STK's counter is written so its compare comes D cycles after a
  masked check, D swept from 0 to 599 one cycle at a time, then the
  kernel's loop - check, `idle()` - until one tick is served (and, a
  second pass, two), with the event latch cleared and set by hand
  before each try; TIM2 at 1 MHz clocks each try and rescues a lost one
  at 50 ms, and a try that ends a whole tick period late (past 250 us
  waiting for one tick, 1250 us for two) is a wake lost until the next
  tick. The handler's entry time steps one cycle per D, so the edge
  is placed to the cycle across the whole sequence, the sleep entry
  and the consume after the wake included. With the OLD order - the store, the `wfi` with MIE clear,
  the unmask after - no position of 600 lost the wake in either latch
  state, waiting for one tick or two; nor in twenty-four code layouts of
  a probe that shifted the old body and the platform's by two and four
  bytes at a time: the one cycle at which the CH32V00x's QingKe V2
  loses an edge to its sleep entry
  ([../ch32v00x/platform.md](../ch32v00x/platform.md)) is not on this
  die. WCH's `__WFE()` - SETEVENT, then two WFEs - under
  the same masked check loses fourteen positions in every layout (its
  first `wfi` consumes the latch, an edge's included): the instrument
  catches a lost wake where there is one. With the UNMASKED order the
  platform takes - the store, the unmask, the `wfi` - none lost in the
  suite's image, built with and without the hardware prologue, in the
  32 KB tier's image, in twelve layouts of the probe, and with eight
  `nop`s between the unmask and the `wfi`; and with the consume after
  the wake, none lost and none a tick late in either latch state,
  waiting for one tick or two, the slowest try 5 us for one tick and
  1005 us for two. What the order and the consume cost, HCLK cycles
  with the hardware prologue:

  | the idle order | edge to the handler | edge to the caller's loop | an `idle()` the latch returns at once |
  |---|---|---|---|
  | masked `wfi`, unmask after | 28 | 65 | 33 |
  | unmask, then `wfi` | 22 | 61 | 28 |
  | unmask, `wfi`, then the consume (the platform's) | **23** | **68** | **33** |

  (25 and 69 for the unmasked order without the hardware prologue; the
  last column carries two counter reads.) On the CH32V303VCT6's V4F the
  platform's order and its consume lose nothing either - 2400 tries,
  none lost and none a tick late, the slowest 5 us for one tick and
  1005 us for two, three runs alike - at 22 cycles from the edge to the
  handler, 74 to the caller's loop and 32 for an `idle()` the latch
  returns at once, the tick bound leaf (22 and 73 against 24 and 76
  with the tick bound as a trampoline, in two images that differ in
  that binding alone: its call and return the difference). The loop and
  `idle()` are the CH32V203C8's instructions one for one in the two
  images, and no f-register is touched on the way: neither the sleep
  path nor the loop uses one, and the tick uses none (the stratum's
  check says so of every image). The one cycle less to the handler and
  five more to the loop than the V4B's are the core's or the image's
  layout, not chased. The handler now runs straight out of the sleep, by
  6.2's second WFE item, instead of after an unmask. In the console's
  kernel loop the Sleep path is thirteen instructions executed where it
  was fifteen - one SCTLR load where there were two, the deep path out
  of line - and a 180 s soak of 2986 commands at random intervals,
  asynchronous to the tick, went without one unanswered. The consume
  after the wake adds three: an `ori`, the store and the second `wfi`.
- **One kernel turn per interrupt** (letter `o`): a Tenuto pack of
  three quiet AOs, two of them with a periodic time event, turned as
  `run()` turns it, the tick the only interrupt: 101 turns over 100
  ticks with the latch consumed, 200 without it - the second turn of
  each pair finding nothing, its `idle()` returning at once on the
  latch the wake left. That turn costs 141 cycles (two counter reads
  included); the consume costs 5 to 7 cycles a wake: from an edge that
  finds the core asleep to the caller's loop 68 against 61, an `idle()`
  the latch returns at once 33 against 28, a quiet turn 146 against
  141. The CH32V303VCT6 turns 101 times over the same 100 ticks, a
  quiet turn 142 cycles.
- **The STK arithmetic holds.** CMPLR = 143999 as programmed, CMPHR and
  CNTH both zero - the reload puts the low half back at the compare, so
  the high half of this 64-bit counter never moves. Over 200 reloads,
  bracketed between two tick edges, the CNT-delta accumulation
  `delay_us` is built on tracked the interrupt count to 39 cycles of
  error out of 28 800 000 (1 ppm; 51 to 55 cycles on the
  CH32V303VCT6).
- **`delay_us` is late by a constant, never early.** 100 us spent 14434
  HCLK cycles for 14400 asked and 900 us - the longest wait that still
  fits a tick period - spent 129643 for 129600 (14438 to 14442 and
  129638 to 129644 on the CH32V303VCT6): some 40 cycles of polling
  granularity either way, not a proportional error. Twenty waits of 500
  us took exactly 10 ticks; 1000 us and above are refused, and so is the
  65536 us gate.
- **The interrupt round trip, in HCLK cycles**: a software interrupt
  raised by hand through the PFIC's pending register, with the cycle
  counter read at the raise, at the handler's first and last statements
  and when the raiser sees it done, 64 rounds each, every round
  identical to the cycle:

  | image | entry | body | exit | whole trip | .text | user stack |
  |---|---|---|---|---|---|---|
  | HPE on (`CH32VX03_HPE=ON`, the default) | 15 | 12 | 25 | **52** | 31652 B | - |
  | HPE off (gcc's own prologue) | 19 | 12 | 29 | **60** | 31796 B | 32 B more |

  Eight cycles on a minimal handler - 56 ns at 144 MHz - 144 bytes of
  flash over this image's handlers, and USER STACK that the hardware
  stack carries instead: the registers a handler saves, 32 bytes in
  this suite's ledger (`stack_untouched()` after the same letter, 18896
  against 18864). That last number is what separates this core from the
  CH32V00x's, where the same feature moves the pushes from the handler's
  code to the hardware but leaves them on the user stack; and 52 cycles
  against that family's 78 is what a bigger core does with the same
  measurement. Every vector is aligned to four (pfic.hpp): in two images
  that differ in the alignment alone, ten runs each, the trip with the
  hardware prologue took 52 cycles against 56. Free in semantics, so it
  is the default. The CH32V303VCT6's V4F, with the hardware prologue
  and the handler bound leaf: entry 15, body 12, exit 25, a whole trip
  of 52 to 62 cycles over 64 rounds (ten runs; 53 to 63 in an image
  whose vectors sat where gcc's two-byte alignment put them). In two images that
  differ in that binding alone the leaf took entry 16, exit 25 and 53
  to 66 against entry 18, exit 27 to 30 and 57 to 70 as a trampoline:
  two cycles of the call on the way in and two to five of the body's
  return on the way out are the trampoline's price where nothing is
  saved, the rest the image's layout.
- **corecfgr's 0x1F buys nothing measurable here.** Three thousand
  iterations of a loop with a load and a data-dependent branch over a
  pseudo-random byte table took 36811 HCLK cycles with the register as
  the crt leaves it and 36813 with it cleared to zero - two cycles,
  0.005 per cent, with the register read back as zero to prove the
  second run really ran without the bits. On the CH32V303VCT6's V4F the
  same loop is SLOWER with the bits: 37005 to 37101 cycles at 0x1F
  over three runs against 36811 at zero every time, half to eight
  tenths of a per cent. The value stays written because it is what the vendor's silicon
  is shipped running, not because this bench can show what it buys.
- **The tick is the HSI's, and the HSI of this die is half a per cent
  fast.** Letter `g` brackets 5000 of its own ticks between two console
  lines; the host's clock timed that bracket at 4.9744 s over three
  runs (-5.1e-3). The same bracket with the PLL fed from the board's
  8 MHz crystal instead of the HSI measures 4.9993 s (-1.4e-4), which
  is the console path's own bias - so the 5.0e-3 between them is the
  oscillator and not the method. A program whose timebase must be
  accurate takes the crystal. The CH32V303VCT6's HSI is a third of a
  per cent fast by the clock suite's bracket ([clock.md](clock.md)),
  whose closing line is kept alone on the wire - which this letter's is
  not, and that board's probe forwards its serial in blocks, so its
  host-timed number is the clock suite's to give.
- **The reset flags.** The boot after the probe's post-programming
  reset reads 0x10000000: SFTRSTF and nothing else, the flags having
  been cleared at the previous boot - so that reset is a system reset
  and reads as software; after an image that never cleared them the
  CH32V303VCT6 read 0x18000000, the power-on's PORRSTF still standing
  beside it. `take_flags()` leaves the register at zero with
  RMVF back at zero and the LSI bits it shares with the clock tree
  untouched.
- **Three real resets** (letter `i`): `Reset::software()` boots to
  SFTRSTF alone, with no watchdog or low-power flag beside it and no
  breadcrumb; `panic<P, ResetReporter>(queue_overflow, 7)` boots to
  SFTRSTF with the record carrying its code and context; and `ebreak`
  with no debugger attached lands in `breakpoint_handler` (vector 9,
  the token the suite banks says which of the two bodies ran), whose
  `fault_reset<P>()` boots to SFTRSTF with a kernel_fault record whose
  context byte is an exception, code 3 - the core's own word for a
  breakpoint. The `.noinit` section does what the linker script and the
  crt promise across all three. The CH32V303VCT6 answers the three legs
  the same, its `ebreak` taken to vector 9 as well.
- **The floating-point unit's state** (letter `j`, CH32V303VCT6):
  mstatus reads 0x7880 at main(), FS = 11 - the crt's fcsr write
  having dirtied the Initial it wrote, which the letter then shows
  directly (FS set back to 01 by hand, fcsr written alone, FS = 11) -
  and one `fmadd.s` moves an Initial unit to Dirty with an exact 6.375
  for 1.5 x 2.25 + 3.0. A division by zero raises DZ alone (fflags
  0x08) and an inexact quotient NX alone (0x01), neither of them
  trapping; frm takes round-toward-zero, reads it back, and 1/3 comes
  out 0x3EAAAAAA under it against 0x3EAAAAAB to nearest.
- **Floating point under an interrupt storm** (letter `k`): 200000
  iterations of twenty float additions held in twenty f-registers, with
  TIM6 at the core's clock interrupting every 400 cycles - the kernel's
  tick going on beside it - and its handler, bound by the float
  trampoline, squaring and summing twenty float locals of its own -
  23262 interrupts over 64 ms - leave every one of the twenty
  accumulators exact to the bit and every handler's own sum right. The
  float trampoline's twenty stores and loads and fcsr's word, with what
  the body saves of the callee-saved ones, are all there is, and they
  are enough.
- **What an interrupt pays for floating point** (letter `l`): letter
  `e`'s method on three lines raised by hand, best of 32 rounds each - a
  handler bound leaf that touches no f-register takes 48 cycles of
  round trip (entry 13, body 12); the float trampoline with a float
  multiply and a multiply-add in its body 125 (entry 46, body 29: the
  twenty stores and fcsr's swap with zero ahead of the body, fcsr's
  restore and the twenty loads after it; in two images that differ in
  fcsr's four instructions alone, 123 and entry 47 against 111 and
  entry 41, twelve cycles); and a plain trampoline whose body calls a
  function the compiler cannot see into 66 (entry 17, body 22), the
  call costing no f-register (ten runs; 49, 126 and 66 in an image whose
  vectors sat where gcc's two-byte alignment put them). Bound as attributed handlers the three
  take 54, 67 (the compiler saving the f-registers the body used, and
  fcsr not) and 100 (all twenty, for the call): the float trampoline's
  set is the price of a body that does float work, and the plain
  trampoline makes calling out of a handler cost the call alone. Then
  fcsr across the float trampoline: with the program's fcsr set to
  0x20 (round toward zero) and to 0x81 (round to nearest, ties to
  max magnitude, with NX standing), the body finds 0 both times and
  raises NX in it, and the program finds its 0x20 and its 0x81 after
  the return.
- **The bus in sleep** (letter `m`): a memory-to-memory DMA1 block of
  65535 words between two fixed addresses moves 48010 words in 2 ms
  with the core spinning - five cycles a word - on both parts, and
  across 1.9 ms of the platform's idle() with the bus-master count set
  aside 37 on the CH32V303VCT6 (six of them already moved when the core
  went to sleep) and 26 to 28 on the CH32V203C8T6 (four), one tick
  served by the one-shot wake: IN SLEEP THE DMA STALLS, the bus matrix
  serving the core alone. With the count standing, idle() over the
  working channel returned in 16 cycles on the CH32V303VCT6 and 13 on
  the CH32V203C8T6.
- **The mask's shadow** (letter `n`, both parts): whether an
  interrupt already on its way is taken AFTER the instruction that
  masks it - the question behind the manual's V2.5 note asking for a
  `fence.i` after a mask. A line (EXTI2's vector, pended by a store into
  PFIC_IPSR with no EXTI activity) is raised, a swept number of `nop`s
  later - zero to nineteen, 250 trials each - it is masked, eight
  `nop`s run masked, and the mask is lifted; the handler records mepc,
  so every trial says whether the interrupt came before the mask
  instruction, inside the masked region, or after the unmask. A pend
  takes three instructions to arrive: from three `nop`s of lead on,
  every trial was taken before the mask. The other 750 trials of each
  variant are the race, and they split cleanly by the kind of mask -
  and, for the global one, by the core:

  | the mask | taken inside the masked region, CH32V303VCT6 | CH32V203C8T6 | the rest |
  |---|---|---|---|
  | `csrrci` on mstatus.MIE, the platform guard's own | **0** of 5000 | **250** of 5000 | held until the `csrsi`, taken on the instruction after it |
  | `csrrci` then `fence.i` | **0** of 5000 | **250** of 5000 | the same |
  | a store into PFIC_IRER, the line's own disable | **750** of 5000 (150 per thousand: every trial of the race) | **750** of 5000 | - |
  | a store into PFIC_IRER then `fence.i` | **750** of 5000 | **750** of 5000 | - |

  The same numbers, trial for trial, from an image built with the
  hardware prologue and one built without it, on each part. So on the
  V4F the global mask has NO shadow - an interrupt pended but not yet
  taken when the `csrrci` retires is never taken after it, with or
  without a `fence.i`. The V4B's 250 are one lead alone, two `nop`s,
  every trial of it: the interrupt is taken with mepc FOUR bytes past
  the `csrrci` - the instruction after it, before any masked
  instruction has run - and with MPIE clear, so the mask had already
  taken effect when the trap came and the `mret` returns into the
  region masked. No instruction of a critical section runs before such
  a handler, so the section is whole on both cores; what differs is
  only where mepc says the interrupt was taken. A line's own disable
  has, on both, a shadow of up to three
  instructions (mepc 2 to 6 bytes past the store, a compressed `sw`
  and two compressed `nop`s, or the `sw` and the four-byte `fence.i`),
  and the `fence.i` does not close it. The note names PFIC_IENRx, the
  register that ENABLES a line; the store that disables one is
  PFIC_IRER's, and that is the one measured.


**The preemptive kernel** - `test_vx03_rubato`, 40 verdicts in `z` on the
CH32V203C8T6 and 46 on the CH32V303VCT6, at 144 MHz, ten runs of `z`
alike (twenty on the CH32V303VCT6): a Rubato pack of five objects, the
Console the lowest and the letters' runner, TIM2's compare the
interrupt the road-1 letters land, the STK's CNTL read bare the ruler
(two reads one cycle apart, against 35 or 36 cycles for the ticker's
composed `cycles()`, the two agreeing over half a millisecond and more
to the cycle, and 28 and within 8 cycles on the CH32V303VCT6),
nothing wired and nothing lit. Road 1 preempts at any depth: an
interrupt's post to High inside Low's dispatch runs High to its end
before Low resumes, ten runs alike, every object reading its own index
as the ceiling and the UART's interrupt the loop's - 255 at its base,
or 0 while the base turn walks the time events (kernel/rubato.hpp holds
it there so that a timer's fire only queues; no dispatch of Top, whose
index is also 0, can be open then, nothing posting to Top between
letters; every key of these runs arrived at the base, so the 0 rests
on the code); road 2 nests
High inside Mid inside Low, each inside the post that asked; at depth 2
an interrupt over High's dispatch, started by either road, runs Top
before High resumes; three timers matured in one turn are posted with
the ceiling at 0 and served highest first, and High preempting Low's
loop of `arm()` and `disarm()` (in the loop's eighteenth turn of 200)
leaves the list whole; a post under the mask pends the software
interrupt, the receiver runs when the section ends, before the
statement after it. Every timing below is its letter's least and most
over those runs where no tick landed in it, each within the bound
its letter derives over the release listing - the path's instructions
at one cycle, a load or a taken branch two, a call or a return three,
and the trap's entry and MRET at letter `e`'s 16 and 25 above - and
beside the Cortex-M realizations':

| HCLK cycles | CH32V203C8T6 | CH32V303VCT6 | STM32F446RE | STM32G0B1RE |
|---|---|---|---|---|
| road 1: an interrupt's last statement to the preempting dispatch's first | **119**..121 | **126**..137 | 143..148 | 180..181 |
| road 1 at depth 2, over a preempting dispatch | 107..109 | 115..125 | 134..138 | 155..156 |
| a post that asks nothing | **55** | 55 | 78 | 95 |
| a road-2 post, the receiver's empty dispatch inside | **310** | 311..313 | 398 | 517 |
| `activate()` over four empty queues | 90 | 81 | 80 | 111 |
| one `arm()`, its masked window inside | 55 | 59 | 79 | 93 |
| a post under the mask: the section's end to the receiver's first | 132..134 | 136..156 | - | 192..193 |
| stack high-water of the suite's deepest chain (bytes) | 1000 | 1156 | 712 | 500 |
| the entry's f-register save, FS Dirty, in / out | - | 27..30 / 29 | (lazy, in the frame) | - |

Road 1 is the TIM2 handler's tail, its MRET straight into the pending
software interrupt, that body's nine instructions and its MRET, the
entry's nineteen and `activate()`'s visits to Top and High, each
queue's two cursors loaded and compared in line (the queue's header
forces `empty()` inline, so a visit makes no call): 87
instructions outside the traps, 172 cycles by the letters' model with
letter `e`'s two returns and one entry (66), against 119 measured: the
model, its trap figures above all (each carries a few instructions of
the raiser's own), is an upper estimate, and the bound (261, 273 on the
V4F) is loose by that much. Two events no tick touches differ by the
predictor's state at the path's taken transfers, five on the V4B and
seven on the V4F, two cycles each at most: the quiet events spread over
2 cycles on the CH32V203C8T6 and up to 11 on the CH32V303VCT6, against
10 and 14. A tick landed in one event of 1000 on the CH32V203C8T6 (145
cycles) and in six of 2000 on the CH32V303VCT6 (152 to 172), read
inside each. The CH32V303VCT6's seven cycles more are the vector's
trampoline (pfic.hpp: its body's return before the MRET), the entry's
test of mstatus.FS, and where the path's branch targets fall in their
words, which costs the two cores differently (above). The road-2
post's scheduling share, 255 cycles, is two traps from thread mode - the pend and the `ecall` - and
their two returns around the entry's 37 instructions and the rescan;
the stack's 1000 bytes (1156) are the deepest chain's frames plus the
block and the entry's 64 bytes at each level of preemption (on the V4F
the f-registers' 96 too, where the entry saves them), where a Cortex-M
level costs its exception frame.

On the CH32V303VCT6 (`z`, 46 verdicts, twenty runs alike) the same letters
and the floating-point letter `j`: Low's float series under
round-toward-zero, preempted at a mark inside its loop by High's own
series, and in ten of twenty runs by a float handler too, bound by the
float trampoline: every sum bit-exact against the same code run without
preemption, Low's fflags cleared before the preemption still clear
after it though High's series raised NX in its own (0x01), Low's fcsr at
the end as without the preemption (0x21), High finding FS Clean. The
entry's save, timed by road 1 over Low, the least of sixteen each:
from the edge to High's first statement 156 to 158 cycles with Low's
unit Dirty against 128 to 130 with it Clean, from High's last statement
to Low's first 158 against 129 - the twenty stores and
fcsr 27 to 30 cycles on the way in, the twenty loads and fcsr 29 on the
way out, as the letter prints them, beside the
float trampoline's own 33 and 26 (letter `l` above).

The scheduler's frame measured alone, with an image built for it
alone whose scheduler stand-in clobbers every caller-saved register:
sixteen patterns set before the request, all sixteen back after the
return and the stack pointer where it was, sixteen trips alike, in an
image built with the hardware prologue and in one built without it
(where the two bodies save their scratch register themselves) on the
CH32V203C8T6.

The suite in images built without the hardware prologue
(CH32VX03_HPE=OFF), its bounds drawn for that build (letter c), ten runs
of `z` alike on each part: 40 verdicts on the CH32V203C8T6 and 45 on the
CH32V303VCT6. Road 1 there takes 140 cycles on the CH32V203C8T6 and 198
on the CH32V303VCT6, a road-2 post 317 and 312; the CH32V303VCT6's
handler that calls out saves and reloads the twenty f-registers in
software around its body, and that reload leaves FS Dirty, so there the
entry saves on both of letter `j`'s paths and the two cost the same.

**The drivers under the preemptive kernel** - the suite is
`test_vx03_rubato_link`, one source and two images, each board the
other's peer with its unchanged `spi_peer` (`z`, 120 verdicts on the
CH32V203C8T6, 121 on the CH32V303VCT6) and `usart_peer` (`y`, 111), ten
runs of each key on each image, every verdict green. It runs the real
SPI and UART stacks inside a Rubato pack of seven objects:
`util/spi_bus.hpp`'s arbiter over the engined `SpiHost` (SPI2 on the
CH32V203C8, SPI3 on its second column on the CH32V303VC, half-word
engines on the instance's two DMA slots, [spi.md](spi.md)),
`util/line_reader.hpp`'s `LineReader` over the link port with both its
engines (USART3, and UART4 on its second column, [usart.md](usart.md)),
a borrower of the reader's lines above it, and on top an object that
TIM2's update interrupt posts to every 97, 193 or 389 us, whose dispatch
spins 300 or 3000 cycles - at 97 us x 3000, 21 % of the core. Every
letter runs once with TIM2 silent as its control and six times under
that pressure, and every pass judges the pressure itself: one dispatch
per period over the pass, none dropped. Both boards run their crystals
at 144 MHz. The target-free half of the suite (the pack, the passes,
the numbered lines, the window client, the UART legs) is the link
suites' shared body, `avrxt/src/apps/rubato_link.hpp`. What it
established:

- **The polled SPI loop's limit is this host's too, and it is always
  detected.** The loop runs inside the arbiter's dispatch, so a dispatch
  above the arbiter holds it off for its whole length, which the host's
  `hold_off_cycles` does not count; with one buffer each way, a receive
  keeps two frames in flight from a frame of 256 cycles (/16 for byte
  frames, /8 for 16-bit ones on these PB1 instances), and a hold-off
  longer than a frame there loses one to OVR. The suite's polled windows
  are the two shapes the engines leave to the loop: 8 16-bit frames
  from an odd address and 2 byte frames. With TIM2 silent every rung
  from /256 to /8 is byte-exact on both, and on the RXNE pump (16
  16-bit frames from an odd address) and the engines (32 frames), in
  all twenty control passes. Under pressure the pump and the engines
  stay exact at every rung of every pass, and the polled loop is cut
  short with `spi_overrun` - 78 rungs over 120 pressure passes - only
  where two frames are in flight and a frame is shorter than the
  preemption, never handing back a run with a hole: under a
  3000-cycle spin and the scheduler's trip around it, at frames of
  2048 cycles and under (16-bit /64 22 times, /32 24, /16 3, /8 9; byte
  frames /128 3, /64 8, /32 6), never at 4096 or more (16-bit /128 and
  /256, byte frames /256); under a 300-cycle spin three times in sixty
  passes, at frames of 256 and 512 cycles (byte /16 and /32, 16-bit /8),
  never at 1024.
- **/4 = 18 MHz exact, /2 not, both printed.** Letter `p` judges to /8,
  where the two datasheets' timing still holds the client's answer
  ([spi.md](spi.md), the pair's section), and prints the two codes
  above: /4 byte-exact on every path in every pass on both images (280
  of 280 windows each), /2 wrong on every one (the host reading the
  answers one bit late, as the pair's ladder finds).
- **The matrix and the arbiter.** The four modes in both widths, LSb
  first and a bit-order mismatch on the engines at /32, byte-exact in
  every pass, the data windows at 954 to 1000 per mille of the control's
  rate under pressure; three requesters of three priorities on one
  select window - the one above the arbiter on the engines (6 frames),
  the one below it on the polled loop and the preempting object on the
  pump, their requests 2 frames each, under `dma_min_frames` - answered
  every one `spi_ok`, to its requester, in its order, with no timeout
  and no rejection and the peer counting every frame exact, in 9264 to
  10103 us a window (three runs on each image); the polled loop keeps
  both of its frames in flight there, 4096 cycles each at /256, longer
  than the preemption that holds it. The command channel needed
  no retry in the twenty runs of `z`.
- **The UART receive side under preemption.** Numbered lines echoed by
  the far chip at 460800 and 921600, assembled by `LineReader` from the
  ring and lent to the borrower: 200 a leg, none lost, out of order or
  corrupt, every port counter zero in 210 readings on each image, 909 to
  923 per mille of the wire at 460800 and 864 to 890 at 921600 (the
  writer paces each line to its own last stop bit and two frame times);
  300 lines blasted through the transmit engine into the peer's sink,
  counted and summed exact, 1001 per mille of the wire at both rates -
  the divisor's +1602 ppm (72 MHz over 156 and 78) at both ends.
- **The loan under preemption.** At 921600 with the borrower spinning
  6000 cycles a line and the lines streamed twenty ahead, every line
  exact and lent alone, the two line buffers strictly alternating, the
  receive engine writing during the borrower's dispatch 19 to 24 times
  a leg and the preempting object landing inside the borrower 25 to 121
  times a pressure pass, 873 to 894 per mille of the wire.
- **The crystal is that letter's premise.** The far echo is a polled
  loop with no FIFO that holds two characters of a writer's lead. On the
  crystals the two ends divide one PCLK1 by one divisor and agree within
  nine ppm ([usart.md](usart.md)), 0.13 of a character over the leg; a
  clock half a per cent fast of the far crystal would lead it by some 70
  characters over the same leg. In a diagnostic build (not kept) clocked
  so - the HSI's PLL, the clock type without the crystal's rate - the
  paced and the blasted legs passed and the streamed one failed in every
  pass, the control included: the far receiver counted 10 to 15 framing
  errors a leg and 25 to 27 lines a leg came back corrupt.
- **The DMA release contract holds at every hand-over.** Each leg moves
  the link port from the polled command channel to the Uart task and
  back through the block's reset line ([dma.md](dma.md), "A released
  requester"): 35 hand-overs a run of `y` with both engines bound, 700
  over the twenty runs, every leg byte-exact from its first block on;
  and letter `i` read, in every run, both link channels and both SPI
  channels disabled, the SPI's request enables clear and TIM2's DMA
  enables clear (TIM2's update is a row of DMA1's channel 2, the
  CH32V203's link transmit channel).
- **The V4F's float tax stays out of the link's vectors.** On the
  CH32V303VCT6 image the vector guard walked the 8 trampolines over 26
  functions with no floating-point instruction reached and checked the
  9 leaf vectors, and letter `i` read mstatus.FS Clean in all ten runs:
  the scheduler's entry saved the crt's Dirty unit once, and nothing of
  the pack wrote a float after.

The images: on the CH32V203C8 40952 bytes of text and 724 of data,
41676 of the 60 KB the linker script gives it (67 %), and 5436 bytes
of RAM of 20 KB (27 %); on the CH32V303VC 44720 of 252 KB (17 %) and
5440 of 64 KB.

## Not covered yet

Driver gaps, each with its reason:

- The PFIC's priority threshold, nesting among the peripheral lines and
  its four free vectored entries (VTFADDRR): an interrupt handler is not
  interrupted by another on every target
  ([../design/kernel.md](../design/kernel.md)), so the one nesting with
  a user is the preemptive kernel's - every peripheral line at one
  level, the software interrupt alone below it - and the vectored
  entries arrive with a user.
- The second route to the same reset, PFIC_SCTLR bit 31 with no key,
  and that register's SLEEPONEXIT: one way in is enough for a reboot,
  and SLEEPONEXIT has no user - a brio program sleeps in the kernel
  loop's idle path and not on a handler's exit, and that path clears
  it - while the rest of that register's sleep bits are the power
  chapter's ([sleep.md](sleep.md)).
- mepc and mtval are readable but do not cross a reset: the breadcrumb
  has one byte for the detail, and it carries the cause.

Implemented but not bench-verified, each with what would measure it:

- **`in_handler()` on this silicon**: the read of PFIC_GISR's GACTSTA
  is the manual's on both series,
  compiled for all thirteen parts; under Rubato a dispatch started at a
  handler's exit runs in thread mode after the trap's MRET, where the
  bit is expected clear, and the same letter in a preempting dispatch
  would measure that too; a letter that asks it from a
  vector, from the main loop and under a critical section, as the
  STM32F4's serial suite does
  ([../stm32f4/usart.md](../stm32f4/usart.md), letter h), would
  measure it.
- **Rubato on the other parts**: the platform, its request and the two
  vector bodies compile for all thirteen, with and without the hardware
  prologue (the family fixture), and the suite is built and run for
  the CH32V203C8 and the CH32V303VC alone; on the CH32V20x_D8 class
  (the CH32V203RB) a run is what would measure it.
- **PINRSTF as the flag of an event**, rather than as the register's
  documented reset value: the NRST pin pulled low by hand, and a supply
  cycled, with the flags read at the boot that follows. PORRSTF is the
  flag a Standby wake leaves on both parts ([sleep.md](sleep.md)), so
  that half is measured.
- **LPWRRSTF**: nothing raises it. The two watchdog flags are measured
  in [watchdog.md](watchdog.md), and a Standby wake - the one event
  that might have been a "low-power reset" - leaves PORRSTF instead on
  both parts, beside IWDGRSTF when the independent watchdog ended it
  ([sleep.md](sleep.md)), so what sets this flag is still unknown - and
  the reference manual's V2.5 revision marks the bit Reserved
  ([vendor/README.md](vendor/README.md)); the option bytes that would
  arm one are decoded and never written, by the flash chapter's own
  decision ([nvm.md](nvm.md)).
- **The idle hook's POWER.** `idle()` is proven to sleep and wake and
  to turn the loop once per interrupt, not to sleep cheaply: what a
  sleep saves is a current measurement with the probe detached, since a
  core in debug mode never sleeps at all.
- **Letter `o` in the 32 KB tier's second image**: the same source as
  the CH32V203C8's, linked by the CH32V203C6 preset; `-2` run on the
  CH32V203C8T6 would count it.
- **What closes a line's own disable.** Letter `n` shows that a store
  into PFIC_IRER lets an interrupt already on its way through up to
  three instructions later and that a `fence.i` does not stop it; a
  read-back of the register, or of the line's enable status, after the
  store is the next variant to time.
- **Letter `g` timed on the CH32V303VCT6.** The suite's host-timed
  bracket through that board's probe carries the probe's block
  forwarding; the rate of that die's HSI is the clock suite's
  measurement ([clock.md](clock.md)), and this letter's own number
  there would want a probe that forwards as bytes come.
