# Platform (CH32X035)

This document is for a programmer putting a brio program on a CH32X035.
It describes what the kernel stands on here, the platform, and the
timebase the program's time events run on. A brio kernel asks the chip
for a handful of things, stated by the kernel's `Platform` concept (a
concept is a compiler-checked set of requirements on a type;
[design/kernel.md, "Platform: what the machine provides"](../design/kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp)):
a way to mask interrupts for a critical section, a way to sleep until
the next interrupt without losing one that arrives at the wrong moment,
a source of time, a breakpoint for a panic, and a record a panic leaves
for the next boot. `Ch32x035Platform<TB>` is this family's answer on the
QingKe V4C core, and `TB` is its timebase: the `Ticker` on the core's
own 64-bit system counter (the STK), a 1000 Hz interrupt. The page also
covers the one spelling of an interrupt handler, what the startup code
(the crt, the assembly file that runs before `main()`) writes into the
core, and where code runs: the flash, or SRAM for a path a program
places there, a driver's or its own.

The smallest use is the glue every program of this project writes, as
the console and the test suites spell it:

```cpp
using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

int main() {
    const auto record = brio::take_panic_record<P>();   // first, and once
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Tenuto<P, ...>::run();
}
```

Line by line: `P` names the platform with its default timebase, the
empty angle brackets taking the `Ticker`; `record` is the record of a
panic that ended the previous run, if any, for the program to report
before anything else (kernel/panic.hpp), and a program that reports
nothing may discard it; the `...` of the last line stands for the
program's active objects in priority order. Four things decide whether
that works. The clock task must have run
first (`SysClock::init()`, [clock.md](clock.md)): the ticker computes
its compare value from the rate the clock type claims, and a ticker
initialized on a chip still at its 8 MHz reset rate ticks six times too
slowly. The STK vector must be bound to `Ticker::tick()` by the program,
as above, under the crt's name for it (`systick_handler`) and with
`BRIO_CH32_INTERRUPT` in front: the kernel binds no vector, and a
program that forgets it compiles, runs and never advances time.
`BRIO_CH32_INTERRUPT` is the one spelling of a handler attribute on
this target, and it follows the project's `CH32X035_HPE` option, a
CMake option of the `ch32x035/` project chosen at configure, on by
default: with it the crt sets INTSYSCR.HWSTKEN and the attribute is
WCH's fast attribute, the core's hardware prologue saving the registers
a handler clobbers; without it the attribute is gcc's plain `interrupt`
and the handler saves them itself. One switch for the whole image,
because a fast handler in an image whose crt did not set the bit
corrupts the program it interrupted. Interrupts must be enabled before
`run()` (`brio::enable_interrupts()`), since the kernel sleeps between
events and the tick is what brings it back. And a program with a DMA
channel at work never sleeps: the DMA controller moves nothing in the
Sleep on this die, so `idle()` returns at once while any channel is
enabled with items left, and a receive ring, a channel that is always
working, holds the program awake ([dma.md](dma.md)). There is no
tickless timebase here: the STK stops with the core, and the kernel's
optional `idle_until()` hook is not provided.

The rest of this page, in order: the documents of record; what the
silicon does (the core, the two ways it sleeps, the hardware prologue,
the STK, the traps, corecfgr, the resets); the types and verbs; how to
use it; the bench findings, measured to the cycle; "A handler in SRAM",
the opt-in placement of a driver's or a program's hot path; and what is
not covered. The failing half the sibling strata carry beside a
platform, which reset happened as a type of its own, the record a crash
leaves, the watchdogs, is not written here (the list at the end); the
reset flags themselves are RCC's and live in [clock.md](clock.md).

## Documents of record

The CH32X035 reference manual V1.8 (3.2 for the
reset sources, 7.1 and table 7-1 for the PFIC and the vector table,
7.5.2 for the PFIC's registers with 7.5.2.38 for PFIC_SCTLR, 7.5.3 for
INTSYSCR and mtvec, 7.5.5 for the STK, 13.4 for TIM3, the suite's rescue
timer, ch. 19 for the electronic signature, 20.3.1 for the flash's wait
states) and the QingKe V4 microprocessor manual V1.1 (table 1-1 for
the V4C, 2.2 for what a trap does to mstatus and 8.2 for what an MRET
leaves in it, 3.4 for the hardware prologue, 6 for the sleep modes and
what ends them, 8.3 for corecfgr).
No errata sheet exists for this series. The headers are
[brio/ch32x035/platform.hpp](../../brio/ch32x035/platform.hpp),
[pfic.hpp](../../brio/ch32x035/pfic.hpp),
[ticker.hpp](../../brio/ch32x035/ticker.hpp) and
[delay.hpp](../../brio/ch32x035/delay.hpp); the reference suite is
`test_x035_platform` (a suite's letter, below, is one command key of
its console and the group of verdicts it runs, `z` the key that runs
every letter of this suite).

## What the silicon does

What the documents say, with what the CH32X035F8U6 has answered marked
as measured - the numbers are the bench findings'; the sibling QingKe
cores' measurements are cited beside them where they bear on the same
question.

- **The QingKe V4C is the V4B with a faster divider and a memory
  protection unit**: RV32IMAC, thirty-two registers, WCH's `xw`
  compressed extension, two levels of hardware stack and two of nesting,
  four vector-table-free channels, a five-cycle integer divide where the
  V4B takes nine (the reference manual's core table; the QingKe V4
  manual's table 1-1). misa reads 0x40901105: A, C, I and M, with U and
  X beside them (measured).
- **Interrupt entry does not clear MIE** (V4 manual 2.2): a handler runs
  with the global enable still set, and the PFIC decides whether another
  line may preempt it - with INTSYSCR.INESTEN clear, which is what the
  crt leaves, none may until MRET.
- **A WFI sleep ends on "the interrupt source responded by the interrupt
  controller"** (V4 manual 6.2), a wording that does not say whether the
  global mask must be set for the wake - which is the question the
  CH32V00x's smaller core answers with a deadlock (measured there,
  [../ch32v00x/platform.md](../ch32v00x/platform.md)). The core can
  sleep as a WFE instead: PFIC_SCTLR.WFITOWFE turns the next `wfi` into
  a wait-for-event, and SEVONPEND makes every interrupt that turns
  pending an event, masked or not, latched - "if the WFE instruction is
  not executed, the system will be woken up immediately after the next
  execution of the instruction" (RM 7.5.2.38). 6.2 gives a WFE three
  wakes that matter here: an event; an ENABLED interrupt, "and after
  waking up, the microprocessor executes the interrupt function first" -
  a level; and, with SEVONPEND, "a new interrupt pending signal" of any
  line - an edge. Entered with MIE set, the WFE needs no edge: that is
  the platform's form (the bench findings measure it).
- **The waking edge is latched too** (measured): an interrupt that wakes
  a WFE has turned pending, so SEVONPEND latches it, and the NEXT WFE
  returns at once on it - a quiet kernel loop turned twice per tick with
  nothing to consume the latch. PFIC_SCTLR.SETEVENT ("set the event to
  wake up the WFE case", RM 7.5.2.38) sets the latch by hand, and a WFE
  executed over a set latch returns at once and leaves it clear - the
  pair that consumes it.
- **In Sleep the STK keeps its clock** and in a deep sleep every clock
  may stop (V4 manual 6.1); a core in debug mode enters no sleep at all,
  so a sleep observed with a probe halted on the core is not one.
- **The hardware prologue/epilogue (HPE)** saves the sixteen integer
  caller-saved registers to an internal stack the program cannot see,
  in one cycle, and restores them on MRET (V4 manual 3.4; two levels
  deep on this core, RM 7.1.1). INTSYSCR.HWSTKEN enables it, and a
  handler declared with the vendor compiler's
  `interrupt("WCH-Interrupt-fast")` attribute relies on it and emits no
  prologue of its own. On the CH32V203's V4B the hardware saved 10
  cycles and 64 bytes of user stack a round trip
  ([../ch32vx03/platform.md](../ch32vx03/platform.md)); on this core the
  round trip is 96 to 100 cycles at 48 MHz with it and 120 to 128
  without (letter `e`, below).
- **The STK is sixty-four bits wide** (RM 7.2, 7.5.5): CNTL/CNTH count
  up (or down, MODE) at HCLK or HCLK/8 towards CMPLR/CMPHR, reload from
  zero with STRE, and raise CNTIF, which the handler clears by writing
  zero. It is interrupt 12 of the vector table, and the interrupt number
  is the table index on this core. STK_CTLR also carries SWIE, the
  software interrupt's other way in. This manual gives the register no
  INIT bit (its bits 30:5 are reserved), where the QingKe V4 manual's
  chapter on the counter has one at bit 5; nothing here writes it. The
  compare, the reload and the high halves that never move under it are
  measured.
- **Traps land in the vector table by kind**: the exception entry at
  index 3 and a breakpoint's own at index 9 - on the CH32V203 an
  `ebreak` with no debugger reached index 9 while mcause said exception
  code 3 (measured there).
- **corecfgr (CSR 0xBC0) configures the pipeline and the instruction
  prediction** and the V4 manual gives it no bit table (8.3); this
  series' reference manual does not name it. WCH's own startup file for
  the series writes 0x1F into it, and so does this crt. On the
  CH32V203's V4B the five bits bought two cycles in 36811 of a timed
  loop; on the CH32V303's V4F that loop ran slightly slower with them
  ([../ch32vx03/platform.md](../ch32vx03/platform.md)); on this core a
  loop of the platform suite's own runs 7.3 per cent slower with them
  than with the register cleared, while a taken branch or a jump
  through a register that shares its fetch word with a load or a store
  before it costs 4 cycles less with them, the predictor they turn on
  sending the fetch ahead (both measured).
- **A reset request is PFIC_CFGR.SYSRST written with KEY3 (0xBEEF in
  the high half), or PFIC_SCTLR.SYSRST with no key** (RM 3.2.2,
  7.5.2.10, 7.5.2.38). And with PFIC_SCTLR.RSTEN at its reset value of
  zero the chip has a core deadlock reset, generated - in 3.2.2's words -
  "when the core addresses an exception or enters an NMI interrupt", and
  never in debug mode; what exactly that sentence covers is not read
  further here, and no verb arms or disarms it.

## Types and verbs

`Ch32x035Platform<TB>`
([brio/ch32x035/platform.hpp](../../brio/ch32x035/platform.hpp)) is the
concept member for member: `CriticalSection` is pfic.hpp's
`InterruptGuard` (one `csrrci` reads and clears mstatus.MIE, the
destructor restores only what it found set, so guards nest), `idle()`
is one store of PFIC_SCTLR (WFITOWFE and SEVONPEND set, SETEVENT and
SLEEPONEXIT clear - with SLEEPONEXIT set the waking handler's mret would
sleep again), the unmask, the `wfi` that sleeps as a WFE, and after the
wake SETEVENT and one more `wfi`, which returns at once and consumes the
latch the wake left, so the caller's loop turns once per interrupt;
`break_here()` is `ebreak`, `now()` the timebase's tick count,
`ticks_per_second` the timebase's rate, `atomic_width` 4, and the
breadcrumb a `PanicRecord` in `.noinit`, which the crt neither loads nor
zeroes. In the release listing the Sleep path is a leaf of fourteen
instructions behind the DMA guard below, the consume three of them (an
`ori`, the store and the second `wfi`). With SLEEPDEEP found set - which no verb of this stratum
does - the path is `idle_deep()`, out of line: the timebase paused and
its pending bit cleared first, as the sibling strata's hooks do,
because a tick that is merely pending would end the sleep before it
began, then the same store, unmask and `wfi`, and no consume, a second
`wfi` with SLEEPDEEP armed being a second entry into that mode. There
is no
`idle_until()` (the timebase stops with the core). And no sleep over a
working DMA channel: in the Sleep the controller moves nothing (measured,
[dma.md](dma.md)), so `idle()` first asks `Dma::any_working()` - a
channel enabled and circular or with items left - and while one is, it
unmasks and returns at once. The registers are the answer, read in the
idle path alone: with the controller's gate closed four instructions (the
gate's load and a branch), with it open six a channel that is not
working; the CH32V203 and the CH32V303 keep a count their drivers update
instead, because their USB controllers are bus masters too, which here
have no driver. A receive ring is always working: a program with one
holds itself awake.

The interrupt verbs are [pfic.hpp](../../brio/ch32x035/pfic.hpp)'s:
`enable_interrupts`, `disable_interrupts`, `interrupts_enabled`, and
`Pfic`'s per-line `enable/disable/enabled/pending/set_pending/
clear_pending/active` over the write-one banks (the manual's ISR bank is
the ENABLE status and its IPR the pending one), and
`Pfic::in_handler()`, which the platform's `in_handler()` forwards
(kernel/platform.hpp's optional member, `HandlerAware`): PFIC_GISR's
GACTSTA, bit 8 at offset 0x4C, an interrupt being executed (the
reference manual 7.5.2.11, the QingKe V4 manual 3.1) - not mstatus.MIE,
which a critical section clears as a trap's entry does. `BRIO_CH32_INTERRUPT` is
the one spelling of the handler attribute: WCH's fast attribute when the
image is built with `CH32X035_HPE` (the default), gcc's plain
`interrupt` otherwise - one option for the whole image, because a fast
handler under an HPE that is off corrupts the program it interrupted -
and both carry `no_icf`: gcc's identical code folding turns the second
of two handlers of one body into a handler that CALLS the first, which
ends in MRET, so the call never returns and the caller's frame stays on
the stack (measured on the CH32V303VCT6,
[../ch32vx03/platform.md](../ch32vx03/platform.md)). Under Tenuto
interrupt nesting is never enabled: the kernel's rule
([design/kernel.md, "The model in one page"](../design/kernel.md#1-the-model-in-one-page));
the preemptive platform below turns it on for its scheduling request
alone.

`Ch32x035RubatoPlatform<TB>` (the same header) is
`Ch32x035Platform<TB>` whole plus the three members of a preemptive
platform, the one the preemptive kernel `Rubato` runs on
([../design/kernel.md](../design/kernel.md), "Rubato"): `preemptive`,
`ceiling` (the index of the object being dispatched, 255 at the loop's
base) and `pend_schedule()`, which is
[brio/ch32x035/schedule.hpp](../../brio/ch32x035/schedule.hpp)'s
`request_scheduler()`: the software interrupt (line 14 of the vector
table, RM 7.3) pended through PFIC_IPSR1 (RM 7.5.2.25), four
instructions out of line. `scheduler_init()` puts the software
interrupt's priority byte at 0x80 and sets INTSYSCR.INESTEN with one
preemption bit (the QingKe V4 manual 3.2), so every peripheral line, at
the reset priority 0, sits at one level above it: the peripheral
handlers still never nest among themselves, and every one of them
interrupts the software interrupt's; the V4C's two nesting levels and
two-deep hardware prologue (table 1-1) are exactly that depth. The
scheduler runs in THREAD MODE: the software interrupt's body,
`scheduler_trap<K>()`, pushes a 16-byte block with the trap's mepc and
mstatus, writes the entry's address into mepc and returns (nine
instructions); the entry, `scheduler_thread<K>()`, stores the sixteen
caller-saved integer registers the hardware prologue just popped back
into the register file - the interrupted code's -, sets MIE, calls
`K::activate()`, loads them back and executes `ecall`; and the
machine-mode environment call's body, `scheduler_return()`, puts mepc
and mstatus back from the block, pops it and returns into the
interrupted code (six instructions). Without the hardware prologue the
two bodies save their one scratch register in the block themselves.
This core has no FPU, so the entry has nothing else to save. The two
vectors are bound with `BRIO_CH32_SCHEDULER_VECTOR(name)`: naked,
`no_icf`, aligned to four like every vector of this stratum, and marked
a leaf (`__brio_leaf_<name>`), which they are: no call, nothing saved in
software. A program executes no `ecall` of its own. The file is the
CH32V203's and the CH32V303's
([brio/ch32vx03/schedule.hpp](../../brio/ch32vx03/schedule.hpp)) with
the f-register shape left out and the request's `fence.i` dropped: on
the V4B three instructions retire behind the pending store unless the
fence closes them, while this core takes the trap at the instruction
after the store (the platform suite's letter `i` reads mepc in the
handler), so here the request is the store alone.

`stack_untouched()` is the RAM ledger: the crt paints the free RAM
between the last section the linker placed and the stack top with one
pattern before the first call, and the verb walks up from that floor
until the pattern breaks - how many bytes of stack no call has reached
since reset.

The timebase is [ticker.hpp](../../brio/ch32x035/ticker.hpp)'s
`BasicTicker<tps>` over the STK (`Ticker` at 1000 Hz; `init`, `tick`,
`ticks/millis/secs/now`, `pause/resume`, `advance` for a site that froze
time, `rebase` for a clock that changed rate), counting HCLK - STCLK set
- up to a compare of hz/tps - 1 with the auto-reload, the high halves
never moving. A rate that does not divide 1000 is refused, so `millis()`
is exact. The busy-wait is [delay.hpp](../../brio/ch32x035/delay.hpp)'s
`delay_us`: at least, never early, no division at wait time, and
refused - no time spent, a `[[nodiscard]]` answer - for a request of one
tick period or more, above the 65536 us gate its 32-bit arithmetic rests
on, or with the counter stopped; under a `DynamicClock` it dispatches by
rate index into a table built at compile time, and below 1 MHz of HCLK
the factor rounds up to one cycle a microsecond, late and never early.

What the crt writes, in [startup_ch32x035.S](../../ch32x035/src/glue/startup_ch32x035.S):
the global pointer and the stack, `.data` and `.bss`, the paint, then
corecfgr = 0x1F, mstatus = 0x1880 (machine mode, MPIE set, MIE clear),
INTSYSCR = 0x1 when the image is built with the HPE and nothing
otherwise - never INESTEN - and mtvec = the table with both mode bits;
then the static constructors and a call to main(). Every handler is
weak: an alias of `default_handler`, or for the exception and the
breakpoint entries a spin of its own.

## How to use it

An app names the platform, binds the vectors it owns with the one
attribute, and starts the clock before the timebase:

```cpp
using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

int main() {
    const auto record = brio::take_panic_record<P>();   // first, and once
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Tenuto<P, ...>::run();
}
```

A wait shorter than a tick is `delay_us(clock, us)`, whose answer says
whether it was served; a wait of a tick or more is a TimeEvent. A panic
is `panic<P>(code, context)` (kernel/panic.hpp): the breadcrumb written,
then `break_here()` - with a debugger attached the program stops there,
and without one the `ebreak` lands in the crt's breakpoint spin, where a
probe finds it and where the breadcrumb waits for the next boot's
`take_panic_record<P>()` if a reset comes before the power goes - a
reset something else causes: no verb of this stratum performs one (the
gap list below).

The same program under the preemptive kernel: the platform type, the
two naked vectors and one call more.

```cpp
#include "kernel/rubato.hpp"

using P = brio::Ch32x035RubatoPlatform<>;
using K = brio::Rubato<P, Sensor, Logger>;   // Sensor preempts Logger

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
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

The reference suite is `test_x035_platform` (105 verdicts in `z`, nothing
wired) on a CH32X035F8U6 - WCH's evaluation board in its QFN20 edition,
over a WCH-LinkE - at 48 MHz from the HSI with the flash at two wait
states, in the image built with the hardware prologue, the board's
rail supplied (with none, fed through the pads' protection diodes, it
restarts under load at 48 MHz:
[../boards/ch32x035-evt-f8u6.md](../boards/ch32x035-evt-f8u6.md));
where two numbers stand for one quantity they are two runs of `z`.
Letters `w` and `k` also ran at 8 MHz - HPRE dividing the HSI by six,
the flash at no wait state, the one line that image differs by - where
the consume was measured before and after. What it measured:

- **The boot story** (letter `a`). The flags at boot read 0x18000000,
  SFTRSTF and PORRSTF, and `reset_flags()` reads them without disturbing
  them - two reads agree; after `clear_reset_flags()` the whole register
  reads zero, RMVF included, which on this silicon takes the verb's
  second store ([clock.md](clock.md)). The electronic signature gives 62
  KB of code flash, and the unique identifier reads 0x2917ABCD,
  0x9129BC45 and 0xFFFFFFFF - its third word, bits 95..64 of the 96 RM
  19.1 gives it, all ones on this die. The word at 0x1FFFF704 reads
  0x035E0611 - the library's 0x035E06x1 for this part, 1 in bits 7:4
  ([pin.md](pin.md) for what that decides).
- **The critical section and the idle hook** (letter `b`). The guard
  masks, a nested one still masks, leaving the inner scope does not
  unmask and leaving the outer one does. A 5 ms window with interrupts
  masked advanced the tick by one: the STK interrupt is a pending bit,
  one tick delivered and the rest coalesced. With the console silent,
  two `idle()` calls - each made with interrupts masked, as the kernel
  makes it - covered the 940 us to the next tick and returned with
  interrupts enabled: the WFE form wakes on this core.
- **No edge position loses the wake** (letter `w`). The STK's compare
  placed 0 to 599 cycles ahead, one cycle a step, before the kernel's
  own shape - a masked check, `idle()`, until one tick or two - with the
  event latch set and cleared by hand before each try, so the edge walks
  across every instruction of the idle path and of the loop around it,
  the sleep entry and the consume included; TIM3 counting at 1 MHz is
  each try's clock and its rescue (a try that reaches its 50 ms period
  is a wake lost for good, one a tick period late a wake lost until the
  next tick). 600 positions in each of the four cases: none lost and
  none a tick late, the slowest try 85 us waiting for one tick and 1086
  us for two, in today's order. In the masked order - the store, the
  `wfi` with MIE clear, the unmask after, no consume - the same walk lost
  none either, in this one code layout: the order this hook does not
  use is the one the QingKe V2 loses an edge in, and the unmasked order
  needs none. An edge that finds the core asleep reaches the handler's
  first statement in 23 cycles and the caller's loop in 68 at 8 MHz
  (masked order: 28 and 66 - the handler waited for the unmask); an
  `idle()` that the latch returns at once costs 23 cycles, two counter
  reads included (24). At 48 MHz, two wait states in the fetch, the same
  walk loses none and none is late in the four cases; the edge reaches
  the handler in 43 cycles and the loop in 167, and an `idle()` the
  latch returns at once costs 92, two counter reads included, its idle
  path asking the DMA controller's gate first.
- **One turn per wake** (letter `k`). A Tenuto pack of three quiet AOs,
  two holding a periodic time event that does not fire in the window,
  turned as `Tenuto::run()` turns it over 100 ticks with the tick the
  only interrupt: 101 kernel turns, in two runs at 8 MHz and two at 48
  (a quiet turn 416 cycles at 48 MHz). Without the consume
  the same pack turned 200 times, in two runs: the waking edge left
  latched ended every second `idle()` at once. That wasted turn costs
  126 cycles, two counter reads included; the consume costs three
  instructions and, from the handler's first statement to the caller's
  loop, 7 cycles a wake (45 against 38).
- **The STK timebase** (letter `c`). CTLR reads 0xF - counting up on
  HCLK, reloading, interrupting - with CMPLR = 47999 as programmed and
  CMPHR and CNTH both zero: the reload puts the low half back at the
  compare, so the high half of the 64-bit counter never moves.
  `ticks()`, `millis()`, `secs()` and `now()` agree (3488, 3488, 3 and
  3.488 s). Over 200 reloads bracketed between two tick edges, the
  CNT-delta accumulation `delay_us` rests on came to 9599716 and 9599852
  cycles against 9600000: 284 and 148 cycles short, 30 and 15 ppm.
- **`delay_us` is late by a constant, never early** (letter `d`). 100 us
  spent 4912 to 4944 HCLK cycles for 4800 asked over twelve runs, and
  900 us - the longest wait that still fits a tick period - 43320 to
  43348 for 43200: 112 to 148 cycles late at both lengths, not a
  proportional error.
  Twenty waits of 500 us took exactly 10 ticks; 1000 us, 50 ms and the
  65536 us gate are refused, and a zero wait is served.
- **The interrupt round trip, in HCLK cycles** (letter `e`): the
  software interrupt - vector 14 - raised by hand through the PFIC's
  pending register, with the cycle counter read at the raise, at the
  handler's first and last statements and when the raiser sees it done,
  64 rounds each and every raise taken: entry 36 in every round, body 26,
  exit 34 with one iteration of the raiser's spin in it, the whole trip
  96 to 100 - the image built with the hardware prologue
  (`CH32X035_HPE=ON`, the default). In an image built with
  `CH32X035_HPE=OFF` (its `z` green, 105 verdicts; ten runs of the letter):
  entry 42, body 28, exit 50, the trip 120 to 128 - the prologue buys 24
  to 28 cycles on the minimal handler, the sixteen registers saved and
  restored by gcc's own code instead. The user stack the image never
  touched reads 19004 bytes with it and 18972 without: 32 bytes, the
  deepest handler frame's software saves, which the hardware stack takes
  instead.
- **The InRam bodies without the hardware prologue** (`test_x035_spi`
  letter `b` in an image built with `CH32X035_HPE=OFF`, three runs): the
  SPI pump's frame - its body placed in SRAM, called from a vector in the
  flash - 289 cycles against 224 with the prologue, and from the flash
  332.5 against 268: some 65 cycles a frame more either way, the vector
  saving the caller-saved registers in software around the call. Its
  handler bodies: the console's 148 cycles, the tick's 69, the pump's 270,
  all still under the host's default hold-off of 380. The SRAM
  transmit-only loop reads 17.0 cycles a frame at /2 against the wire's
  16 (16.0 with the prologue), and the costs no longer match the
  constants the host states, which are the default image's: the letter's
  verdicts on both fail in that image, as they must.
- **The tick inherits the HSI's error** (letter `g`, six runs): 5000
  ticks bracketed by two console lines, the lines time-stamped as they
  arrive by a host on an NTP-disciplined clock: 5.0120 to 5.0134 s - the
  tick 2410 to 2690 ppm slow, the HSI's -0.24 to -0.27 per cent at the
  factory calibration and the trim's centre, which the desk's crystal
  measured too ([clock.md](clock.md)); the arrival jitter of the probe's
  bridge is a millisecond or so, some 200 ppm of a 5 s bracket.
- **corecfgr's 0x1F makes a data-dependent loop slower** (letter
  `f`). The crt left 0x1F in the register; 2000 iterations of a loop
  with a load and a data-dependent branch over a 64-byte pseudo-random
  table took 24192 to 24200 HCLK cycles with it over thirteen runs, and
  22544 in every run with the register cleared - 7.3 per cent more with
  the vendor's bits - the register reading back zero for the cleared
  loop and 0x1F after it. The figure moves with the loop's address
  modulo four: at another placement of the same loop the gap read 25
  per cent (23716 against 18924).
  The crt writes the value WCH's own startup file writes.
- **What the crt left in the core** (letter `h`): INTSYSCR = 0x1, the
  hardware stack on and nesting off; mtvec = 0x3, the table at the
  image's base 0x0000 0000 with both mode bits; misa = 0x40901105, a
  32-bit core with A, C, I and M, and U and X beside them. mstatus reads
  0x1888 in a letter - machine mode, MIE and MPIE set - with MPP = 3,
  the crt's value, though the tick's handler has returned through MRET
  thousands of times before the letter reads it: an MRET leaves MPP
  where it found it, as the QingKe V4 manual's 8.2 has it (an MPP of 3
  written at startup keeps the core in machine mode after every return)
  and not as the privileged specification does, whose MRET drops MPP to
  the least-privileged mode the core implements - user mode here, by
  misa.
- **The RAM ledger answers.** `stack_untouched()` read 18460 bytes never
  touched after this suite's whole `z`, 19616 after the clock suite's
  and the pin suite's, and 19416 after the USART suite's, of the part's
  20 KB.

**The fetch and trap cost model** - letter `i`, 54 verdicts, ten runs of
the letter alike: the figures `test_x035_rubato`'s bounds are drawn
with (below), measured on hand-placed assembly timed on the STK's CNTL
read bare, at 48 MHz from the flash. A fetch figure is a pattern of N
units timed against the same pattern of 2N, each the least of 32 calls
made with interrupts masked, so the difference is N units and nothing
else; a trap figure is the difference of two stamps, one 32-bit load of
the counter each, the first one word ahead of the instruction that
raises the trap and the second the handler's first instruction, less
the stamp's own word - two loads back to back read 4 cycles apart -
judged in every one of 256 trips, the few a tick landed in dropped. The
software interrupt is letter `e`'s vector, so for its rows mtvec points
at a copy of the crt's table, elsewhere in the flash, with that one
entry replaced; TIM2's capture-compare line, pended by a store and
timed through both tables, says the table's place costs nothing. The
figures, in HCLK cycles, with corecfgr as the crt leaves it and again
with it cleared - each fetch and trap figure judged to the cycle, the
predictor's rows (the extra a call costs right after the other way's
training, against its trained cost) against the allowance below:

| figure | 0x1F | cleared |
|---|---|---|
| a 32-bit instruction, on a word or half a word off | 4 | 4 |
| a compressed instruction | 2 | 2 |
| a taken `c.j`, `j` or conditional branch, its target in one word | 8 | 8 |
| a `j` to a 32-bit instruction across two words | 12 | 12 |
| a conditional branch not taken | 4 | 4 |
| a call and its return, both targets on a word | 16 | 16 |
| a store and a taken branch in one fetch word | 8 | 12 |
| a load and a call through a register in one fetch word, the return included | 16 | 20 |
| a store pending the software interrupt, to its handler's first instruction | 20 | 20 |
| a store pending TIM2's line, through the crt's table and through the copy | 20, 20 | 20, 20 |
| an `ecall`, to its handler's first instruction | 20 | 20 |
| the `csrsi` that finds the software interrupt pending, to its handler | 20 | 20 |
| an MRET into code, to its target | 8 | 8 |
| an MRET into a pending interrupt, to that handler's first instruction | 24 | 24 |
| one compressed branch on a word, falling through right after a taken training | 8 more | 0 |
| one 32-bit branch half a word off, the same | 0 or 8 more | 0 |
| one 32-bit branch on a word, the same | 0 to 4 more | 0 |
| sixteen 32-bit branches, the same | 12 to 20 more | 0 |
| sixteen compressed branches, the same | 56 to 64 more | 0 |
| any of them taken right after a fall-through training; sixteen calls through a register to the other of two leaves | 0 | 0 |

mepc reads the instruction after the store and after the `csrsi`: the
trap is taken there, and nothing retires behind either. The predictor
corecfgr's 0x1F turns on costs nothing when it guesses right. A branch
it guessed taken that falls through costs up to 8 cycles more, one it
guessed to fall through that is taken nothing, and sixteen in a row cost
at most 64 more, 4 a branch; and over the 32 timed calls of a row of
not-taken branches the slowest exceeded the least by up to 44 cycles
with sixteen branches, 80 with thirty-two. The Rubato bounds allow
4 cycles at each predicted site of a path, its calls and returns
counted, and every row here is judged so and within it; the allowance
holds over a path, not at each site: one wrong guess alone can cost
8, twice a site's share. With the register cleared no call of a pattern differs from
another, and every figure is the same but one pairing: a transfer the
core resolves when it executes - a taken branch, a jump through a
register - that shares its fetch word with a load or a store before it
waits 4 cycles more; a direct jump, resolved at its decode, does not,
and with 0x1F the predictor sends the fetch to the target before the
transfer executes. In the image built without the hardware prologue
(`CH32X035_HPE=OFF`, its `z` green with 105 verdicts, five runs of the
letter) every fetch and trap figure is the same to the cycle: the core's
push and pop cost a trap nothing measurable.

**The preemptive kernel** - `test_x035_rubato`, 40 verdicts in `z` on
the CH32X035F8U6 at 48 MHz, ten runs of `z` alike in the image built
with the hardware prologue and ten in the one built without it: a
Rubato pack of five objects, the Console the lowest and the letters'
runner, TIM2's capture-compare line the interrupt the road-1 letters
land, the STK's CNTL read bare the ruler (two reads 3 cycles apart,
against 76 cycles for the ticker's composed `cycles()`, the two
agreeing over half a millisecond and more within 12), nothing wired and
nothing lit. Road 1 preempts at any depth: an interrupt's post to High
inside Low's dispatch runs High to its end before Low resumes, ten runs
alike, every object reading its own index as the ceiling and the UART's
interrupt the base's 255; road 2 nests High inside Mid inside Low, each
inside the post that asked; at depth 2 an interrupt over High's
dispatch, started by either road, runs Top before High resumes; three
timers matured in one turn are posted with the ceiling at 0 and served
highest first, and High preempting Low's loop of `arm()` and `disarm()`
(in the loop's sixth turn of 200) leaves the list whole; a post under
the mask pends the software interrupt, the receiver runs when the
section ends, before the statement after it. INTSYSCR reads 0x3 once
the nesting is on (0x2 without the prologue): PMTCFG's bit reads back
zero, as on the CH32V203. Every timing below is within the bound its
letter derives over the release listing with this core's fetch, which
the suite's letter c states and `test_x035_platform`'s letter `i`
measures (above) -
straight-line code a word every 4 cycles, a taken transfer 8 to its
target (12 to a 32-bit target across two words), a trap's entry 20
cycles from the instruction that raises it, an MRET into code 8 and
into a pending interrupt 24, the same with the hardware prologue and
without it - and the branch predictor corecfgr's 0x1F turns on, whose
wrong guess costs up to 4 cycles a branch, call, jump or return: the
least of each figure is that model's count to the cycle, the road-2
post's aside (12 over), and the spread above it is the predictor's:

| HCLK cycles | CH32X035F8U6 | without the prologue | CH32V203C8T6 (144 MHz) |
|---|---|---|---|
| road 1: an interrupt's last statement to the preempting dispatch's first | **356**..368 | 392..400 | 131..133 |
| road 1 at depth 2, over a preempting dispatch | 292..376 | 328..416 | 110..112 |
| a post that asks nothing | **141**..149 | 155 | 55 |
| a road-2 post, the receiver's empty dispatch inside | **876**..877 | 919..927 | 337 |
| `activate()` over four empty queues | 309 | 315..323 | 120 |
| one `arm()`, its masked window inside | 145..165 | 147 | 51..61 |
| a post under the mask: the section's end to the receiver's first | 405..485 | 407..435 | 150..152 |
| stack high-water of the suite's deepest chain (bytes) | 1000 | 1076 | 1000 |

Road 1 is the TIM2 handler's tail, its MRET straight into the pending
software interrupt, that body's nine instructions and its MRET, the
entry's nineteen and `activate()`'s visits to Top and High: 101
instructions, the CH32V203's code, and 356 cycles by the model,
measured 356 at the fastest event of every run: 7.4 us at 48 MHz
against the CH32V203's 0.9 us at 144, the fetch's 4 cycles a word
making the difference, not the instructions. A tick landed in one or
two events of 100 in each run (up to 452 cycles, read inside each). The
road-2 post's scheduling share, 735 cycles, is two traps from thread
mode - the pend and the `ecall` - and their two returns around the
entry's 37 instructions and the rescan; the request is the store alone,
because this core takes the software interrupt at the instruction after
the store that pends it, and after the `csrsi` that unmasks it (the
types and verbs above), where the CH32V203's request pays a `fence.i`
for the three instructions its core lets retire. The stack's
1000 bytes are the deepest chain's frames plus the block and the
entry's 64 bytes at each level of preemption, as on the CH32V203. In
the image built without the hardware prologue (CH32X035_HPE=OFF) the
traps cost the same and the handlers that save in software pay it: road
1 392 cycles (TIM2's sixteen reloads, the software interrupt's scratch
save), a road-2 post 919 to 927. With corecfgr cleared in a probe image
every path of the suite ran the same cycle count at every event, the
model's; the register's 0x1F buys nothing on these paths and the
platform suite's letter f already has it making a data-dependent loop
7 per cent slower.

## A handler in SRAM

**What the silicon does.** At 48 MHz the code flash answers behind two
wait states (RM 20.3.1), and a taken branch is a fetch the core waits
for: from it the SPI's bare transmit loop takes 20.8 cycles a frame at
/2, and the same loop from SRAM 16.7 - the wire's 16 (measured,
[spi.md](spi.md)). The vector table holds absolute addresses (mtvec's
MODE1, RM 7.5.3.2), so a handler placed in SRAM is entered from its
vector directly, and a call between SRAM and the flash is an `auipc` and
a `jalr`, which reach 2 GB either side of the caller: no veneer either
way.

**The option and its spelling: opt-in, per driver.** The seven linker
scripts place the input sections `.fast_text` and `.fast_text.*` first
in `.data` ([../../ch32x035/ld/ch32x035f8.ld](../../ch32x035/ld/ch32x035f8.ld)),
so the crt's copy of `.data` moves them into SRAM before `main()`, and
the platform says so (`Ch32x035Platform::places_fast_text`). A driver of
this stratum takes a placement, util/placement.hpp's `InFlash` (the
default) or `InRam`, as its last template parameter - `SpiHost`,
`I2cHost`, and `Uart` after its options:

```cpp
using Link = Uart<4, P, 512, 256, UartFormat{}, DmaTxEngine<1>, DmaRxEngine<8>, 0, UartOptions{}, InRam>;
```

Each driver writes its hot body once and runs it through
`Hot<Placement>::run` ([../../brio/util/placement.hpp](../../brio/util/placement.hpp)):
under `InFlash` the body is inline where it is called, the vector that
binds it, as if there were no trampoline; under `InRam` each path is one
out-of-line instance in `.fast_text.brio`, flattened, a call from the
vector. Only the instances an image names exist: an image that opts into
nothing has no byte of code in SRAM. Compared with the same images built
with every hot body written inline and no trampoline at all: the
console, `usart_peer` and the clock, pin, platform and timer suites are
byte-identical; `spi_peer` differs in one immediate, the default
hold-off; `twi_peer` and `test_x035_dma` by what `InFlash`'s flattening
inlines into the vector - the I2C's STOPF clear, the USART's consume and
block start, whose completion takes 271 cycles from the flash flattened
against 371 with three calls. A third tag, or `InRam` on a platform that
does not place `.fast_text`, does not compile
(`test/family_ch32x035/neg/`).

A program places a function of its own - the vector it binds, above all
- with one attribute, `.fast_text`:

```cpp
extern "C" BRIO_CH32_INTERRUPT [[gnu::section(".fast_text")]] void usart4_handler() { (void)Link::isr(); }
```

The name is two names. A function defined in a header is emitted in a
COMDAT group, and GCC refuses one section name for a grouped function
and a plain one in the same translation unit ("section type conflict",
WCH's gcc 15.2); a local clone of a grouped function - IPA-SRA's - leaves
the group and meets the same refusal among the grouped ones (measured:
the refusal goes away with `-fno-ipa-sra`, not with `-fno-ipa-cp`). So
the trampoline's instances name `.fast_text.brio` and are never cloned,
and a program's plain function names `.fast_text`; a program's own
inline or template function goes through `Hot<InRam>::run` rather than
an attribute. The family fixtures compile a placed vector beside the
`InRam` instances in one unit (`test/family_ch32x035/spi.cpp`,
`dma.cpp`, `i2c.cpp`). `.ram_text`, which the scripts keep beside it,
is the name of code that must run from SRAM for correctness - nothing of
this stratum's.

**Which placement buys what.** A driver's `InRam` moves what only the
driver can: its private paths (the SPI host's polled loops) and the
bodies it calls out of the vector, with the vector left in the flash. A
program that places the vector moves everything inline in it - every
`InFlash` body it binds - with no call at all. On a path whose body is
inline in the vector whole (the I2C event pump, a USART's per-byte
transport), placing the vector is the faster of the two, and placing
both buys nothing more: an `InRam` body called from a vector in SRAM
measured the same as the vector alone (79 and 95 cycles against 79 and
94, below), its bytes spent twice. On a path the driver calls out of
line or keeps private, `InRam` is the only way.

**What it buys, measured** at 48 MHz on the CH32X035F8U6, each pair one
suite with the path in the flash and in SRAM, nothing else moved:

| path | placed by | from the flash | from SRAM | SRAM bytes |
|---|---|---|---|---|
| the SPI pump, a frame at /2, behind the suite's vector (stamps, owner switch) | the driver (`InRam`) | 268.6 cycles | 225.1 | 302, 332 with engines |
| the SPI transmit-only loop, a frame at /2 (the wire 16) | the driver | 24.0, its function aligned to a word | 16.0 | 798 a host type, both loops |
| the SPI receive loop, a frame at /2 and /4 (the wire 16, 32) | the driver | 51.9, 52.0, aligned likewise | 28.9, 46.9 | (above) |
| the USART transmit engine's completion, the DMA vector's body | the driver | 285 to 288 (324 at most) with the vector on a word, 276 to 283 half a word off; the line idle at a block boundary above 1.5 Mbaud | 129 to 132 (154 at most), gapless to 3 Mbaud | 326 |
| a USART's interrupt transport, a gapless run of 256 in | the driver, or the program's vector | whole to 2 Mbaud | to 2.4 Mbaud behind a vector in the flash; to 3 Mbaud with the vector placed | 488 an engined port's `isr()`; the suite's vector 1324 |
| the I2C event pump's entry, a 255-byte write and a 16-byte read | the driver, or the program's vector | 205, 253 (333 at most) | 140, 157 behind a vector in the flash (195 at most); 79, 94 with the vector placed | 968 for one plain host |
| the console's handler body (115200, `test_x035_spi`) | the program | 145 | 66 | 404 |
| the tick's handler body | the program | 65 | 29 | 100 |

The handler bodies are stamped on the STK between their first and last
statement; the hardware prologue's entry and exit, 96 to 100 cycles a
round trip with the vector in the flash (above), are outside them. From
the flash, code with taken branches costs what its address modulo four
makes it, the fetch's word: the same instructions at every offset through
32 bytes took one of two costs, by the offset's second bit and by
nothing else - 52.0 or 72.0 cycles a frame for the SPI receive loop at
/4, 68 or 72 for a body of four taken forward branches - where
straight-line code and three-instruction loops did not move and SRAM
moved by a cycle at most (a scratch sweep, [spi.md](spi.md), "The
fetch"). gcc aligns a function to two bytes under -Os, so a function
that keeps a stated flash cost is `aligned(4)` - the SPI host's polled
loops, and every vector, through `BRIO_CH32_INTERRUPT` (pfic.hpp): a
vector's own code costs the same wherever the image puts it, two bytes
of padding at most, while a function it calls in the flash keeps a
stated cost only if it is aligned itself.

**What it costs.** The bytes in the last column, of the part's 20 KB -
the load image stays in the flash beside them. A placed function exists
only from the crt's copy on, and the crt enters `main()` with interrupts
masked, so no vector reaches SRAM the copy has not written. A driver's
`InRam` body is a call from a vector in the flash: under the hardware
prologue (the default) the call saves no register, and the vector passes
the closure's address, one instruction; in an image built without it
the vector saves the caller-saved registers in software around the
call. The hold-off the SPI host draws its write-ahead from is a fact of
the image: its default is the one safe with every driver in the flash,
and a program that places its handlers states its own
([spi.md](spi.md)).

## Not covered yet

Driver gaps, each with its reason:

- **Reset as a type, the fault record and the two watchdogs** - the
  sibling strata's `reset.hpp` (`Reset`, `ResetReporter`,
  `fault_reset<P>()`, `Iwdg`, `Wwdg`): born with the watchdog chapter
  (RM ch. 5, 6), their first user. A reboot on purpose, a panic that
  survives into the next boot and a fault body that records its cause
  wait for it; the reset flags are `Rcc`'s verbs meanwhile
  ([clock.md](clock.md)).
- **The PFIC's priority threshold, its four vector-table-free channels
  and any priority byte but the scheduler's** (RM 7.5.2.9, 7.5.2.12 to
  7.5.2.16, 7.5.2.37): no user; the nesting is the preemptive
  platform's alone, two levels, every peripheral line at the reset
  priority.
- **The physical memory protection unit** (RM 7.5.4): no user.
- **The core deadlock reset's switch, PFIC_SCTLR.RSTEN**: left at its
  reset value, the reset on. Which traps raise it is a measurement - an
  exception taken inside the fault handler, the reset flags read at the
  next boot - that belongs with the fault body above, its first user.
- **The debug module's freeze bits** (DBGMCU_CR, CSR 0x7C0, RM 23.2.1):
  no verb reads or writes them - on the CH32V203 a `csrw` to that CSR
  from the running program resets the part
  ([../ch32vx03/sleep.md](../ch32vx03/sleep.md)).
- **The scheduler's path in SRAM**: the two naked vectors, the entry,
  `activate()`, the queues' out-of-line verbs and the request run from
  the flash, 4 cycles a word; by the figures of "A handler in SRAM"
  below a `.fast_text` placement would cut road 1 by about half. Not
  tried: a placement the preemptive platform chooses for a program is a
  decision of its own.

Implemented but not bench-verified, each with what would measure it:

- **Rubato on the other six parts**: the platform, its request and the
  two vector bodies compile for all seven, with and without the hardware
  prologue (the family fixture); the suite is built and run for the
  CH32X035F8U6 alone.

- **`in_handler()` on this silicon**: the read of PFIC_GISR's GACTSTA
  is the manual's, compiled for
  the seven parts; a letter that asks it from a
  vector, from the main loop and under a critical section, as the
  STM32F4's serial suite does
  ([../stm32f4/usart.md](../stm32f4/usart.md), letter h), would
  measure it.
- **The deep path, `idle_deep()`**: no verb of this stratum sets
  SLEEPDEEP, so the paused timebase, the unmask before the `wfi` and the
  absent consume are unmeasured; the sleep site born with the power
  chapter is its first user and its suite the measurement.
- **The idle hook's power**: `idle()` proven to sleep once per wake is
  not `idle()` proven to sleep cheaply - the current in Sleep is a
  measurement with an ammeter and the probe detached.
