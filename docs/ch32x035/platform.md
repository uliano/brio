# Platform (CH32X035)

This document is for a programmer putting a brio program on a CH32X035.
It describes what the kernel stands on here, the platform, and the
timebase the program's time events run on. A brio kernel asks the chip
for a handful of things, stated by the kernel's `Platform` concept (a
concept is a compiler-checked set of requirements on a type;
[design/kernel.md, "Platform: what the machine
provides"](../design/kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp)):
a way to mask interrupts for a critical section, a way to sleep until
the next interrupt without losing one that arrives at the wrong moment,
a source of time, a breakpoint for a panic, and storage the startup code
does not clear, where a panic leaves a record that the next boot can
read if something outside the program resets the chip - no verb of this
family does (How to use it). `Ch32x035Platform<TB>` is this family's
answer on the QingKe V4C core, and `TB` is its timebase: the `Ticker` on
the core's own 64-bit system counter (the STK), a 1000 Hz interrupt.
`Ch32x035RubatoPlatform<TB>` is the same platform for the preemptive
kernel, Rubato. A handler is bound with one macro, the startup code (the
crt, the assembly file that runs before `main()`) leaves the core
configured as the next sections state, and code runs from the flash or,
for a path a program places there, from SRAM: a driver's, the program's
own, or the preemptive kernel's scheduler path, which goes there unless
the program says otherwise.

The smallest complete program, one active object that counts seconds (an
active object is a component of a brio program with its own event queue,
served one event at a time by the kernel):

```cpp
#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/ticker.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/panic.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"

using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

struct Second {};

// One active object: a time event every second, counted.
struct Uptime : brio::Fsm<Uptime, Second> {
    static inline brio::EventQueue<Event, 2, P> queue;
    static inline brio::TimeEvent<P, Uptime, Second> every{Second{}};
    static inline uint32_t seconds = 0;

    static void init() { start(&running); }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                every.arm_every(brio::ticks_from_secs<P>(1));
                return handled();
            },
            [](Second) {
                ++seconds;
                return handled();
            },
            [](auto) { return unhandled(); });
    }
};

} // namespace

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }

int main() {
    // A previous run's panic, if any, read before anything can panic
    // again; this program has no way to report it, so it only clears it.
    (void)brio::take_panic_record<P>();
    if (!SysClock::init() || !brio::Ticker::init(clock)) {
        brio::panic<P>(brio::PanicCode::kernel_fault);   // no clock or no tick: nothing can run
    }
    brio::enable_interrupts();
    brio::Tenuto<P, Uptime>::run();   // the active objects, highest priority first
}
```

`P` names the platform with its default timebase, the empty angle
brackets taking the `Ticker`. `take_panic_record<P>()` returns the
record of a panic that ended the previous run, if one is there, and
clears it, so a second call finds nothing (kernel/panic.hpp); it comes
first so that the record is read before anything in this run can panic
and overwrite it. `Tenuto<P, ...>` is the cooperative kernel, and the
active objects it lists are in priority order, the first the highest.
Four things decide whether the program works:

1. **The clock task runs first** (`SysClock::init()`,
   [clock.md](clock.md)): the ticker computes its compare value from the
   rate the clock type claims, so a ticker initialized on a chip still
   at its 8 MHz reset rate ticks six times too slowly.
2. **The program binds the STK's vector to `Ticker::tick()`**, with
   `BRIO_CH32_VECTOR` under the crt's name for it, `systick_handler`.
   The kernel binds no vector, and the crt points every unbound one at
   its default spin: a program that forgets the binding compiles, and
   the first tick after `enable_interrupts()` enters the spin and the
   program stops there.
3. **Interrupts are enabled before `run()`**
   (`brio::enable_interrupts()`): the kernel sleeps between events, and
   the tick is what brings it back.
4. **Every handler is bound with `BRIO_CH32_VECTOR`** (or, for the
   preemptive kernel's two vectors, `BRIO_CH32_SCHEDULER_VECTOR`), never
   given an interrupt attribute by hand, because the binding follows the
   image's hardware-prologue option and a handler written for the other
   setting corrupts the program it interrupts.

The handler attribute `BRIO_CH32_VECTOR` writes follows the project's
CMake option `CH32X035_HPE`, chosen at configure and on by default. With
it, the crt sets INTSYSCR.HWSTKEN, the core's hardware prologue saves
the caller-saved registers on a trap's entry, and the attribute is WCH's
fast one, under which the handler saves none of them itself. Without it,
the attribute is gcc's plain `interrupt` and the handler saves them
itself. It is one switch for the whole image because a fast handler in
an image whose crt did not set the bit returns into a program whose
registers it clobbered.

A program with a DMA channel at work never sleeps: the DMA controller
moves nothing in this die's Sleep, so `idle()` returns at once while a
channel works, and a receive ring keeps the program awake for as long as
it runs (Types and verbs). There is no tickless timebase here - a
timebase that keeps counting while the core sleeps and wakes it at the
next deadline, which the kernel's optional `idle_until()` hook serves:
the STK interrupts every millisecond, and in a deep sleep, where such a
timebase would earn its keep, the QingKe V4 manual lets every clock stop
(6.1). A type for the cause of the last reset, a fault record and the
two watchdogs are not written for this family (Not covered yet); the
reset flags are RCC's verbs, in [clock.md](clock.md).

## Documents of record

The CH32X035 reference manual V1.8 ("RM"): the core comparison table at
its head (the V4C against the V4B and V4F), 2.3.2 for the Sleep and
2.3.3 and 2.3.4 for the Stop and the Standby, 3.2.2 for the reset
sources, 7.1.1 for the PFIC's nesting and hardware-stack depth, 7.3 and
table 7-1 for the vector table, 7.5.2 for the PFIC's registers - 7.5.2.9
for the priority threshold, 7.5.2.10 for PFIC_CFGR, 7.5.2.11 for
PFIC_GISR, 7.5.2.12 to 7.5.2.16 for the vector-table-free channels,
7.5.2.25 for PFIC_IPSR1, 7.5.2.37 for the priority bytes, 7.5.2.38 for
PFIC_SCTLR - 7.5.3 for INTSYSCR (7.5.3.1) and mtvec (7.5.3.2), 7.5.4 for
the memory protection unit, 7.5.5 for the STK, ch. 5 and 6 for the two
watchdogs, 13.4 for TIM3, which the platform suite runs as its rescue
timer (its update interrupt ends a try whose wake was lost), ch. 19 and
19.1 for the electronic signature, 20.3.1 for the flash's wait states,
23.2.1 for the debug module's DBGMCU_CR. The QingKe V4 microprocessor
manual V1.1: table 1-1 for the V4C, 2.2 for what a trap does to mstatus,
3.1 for the PFIC's register set, 3.2 for INTSYSCR and mtvec, 3.4 for the
hardware prologue, 3.5 for the vector-table-free channels, ch. 5 for the
system counter, 6.1 and 6.2 for the sleep modes and what ends them, 8.2
for what an MRET leaves in mstatus, 8.3 for corecfgr. WCH publishes no
errata sheet for this series that we could find
([vendor/README.md](vendor/README.md)).

The headers are
[brio/ch32x035/platform.hpp](../../brio/ch32x035/platform.hpp),
[pfic.hpp](../../brio/ch32x035/pfic.hpp),
[ticker.hpp](../../brio/ch32x035/ticker.hpp),
[delay.hpp](../../brio/ch32x035/delay.hpp) and
[schedule.hpp](../../brio/ch32x035/schedule.hpp). The reference suite is
`test_x035_platform`; the preemptive kernel's are `test_x035_rubato`
(the scheduler path in SRAM) and `test_x035_rubato_flash` (the same
letters with the path in the flash); the table of "A handler in SRAM"
reads `test_x035_spi`, `test_x035_i2c`, `test_x035_usart` and
`test_x035_dma`, whose documents name the peers they run against. A
suite's letter is one command key of its console and the group of
verdicts it runs, `z` the key that runs every letter of the suite.

## What the silicon does

What the manuals say, and what the CH32X035F8U6 has answered, marked
measured, with the numbers in the bench findings; the other QingKe
cores' measurements are cited beside them where they bear on the same
question.

- **The QingKe V4C is the V4B with a faster divider and a memory
  protection unit** (RM's core comparison table): RV32IMAC, thirty-two
  registers, WCH's `xw` compressed extension, two levels of hardware
  stack and two of nesting (RM 7.1.1, the V4 manual's table 1-1), four
  vector-table-free channels - interrupt lines whose handler's address
  the PFIC holds in a register and enters without reading the vector
  table (V4 manual 3.5) - and an integer divide of five cycles where the
  V4B takes nine (RM's table). misa reads 0x40901105: A, C, I and M,
  with U and X beside them (measured).
- **Interrupt entry does not clear MIE** (V4 manual 2.2): a handler runs
  with the global enable still set, and the PFIC decides whether another
  line may preempt it - with INTSYSCR.INESTEN clear, which is what the
  crt leaves, none may until MRET.
- **A WFI sleep ends on "the interrupt source responded by the interrupt
  controller"** (V4 manual 6.2), a wording that does not say whether the
  global mask must be set for the wake. On the CH32V006 and the
  CH32V203C8 a `wfi` with MIE clear sleeps past every pending interrupt
  (measured there, [../ch32v00x/platform.md](../ch32v00x/platform.md),
  [../ch32vx03/platform.md](../ch32vx03/platform.md)); on this core the
  question is unasked (Not covered yet). The core can sleep as a WFE
  instead: PFIC_SCTLR.WFITOWFE turns the next `wfi` into a wait for an
  event, and SEVONPEND makes every interrupt that turns pending an
  event, masked or not, latched - "if the WFE instruction is not
  executed, the system will be woken up immediately after the next
  execution of the instruction" (RM 7.5.2.38). The V4 manual's 6.2 gives
  a WFE three wakes that matter here: an event; an enabled interrupt,
  "and after waking up, the microprocessor executes the interrupt
  function first" - a level; and, with SEVONPEND, "a new interrupt
  pending signal" of any line - an edge. The platform enters the WFE
  with MIE set, which depends on the level and not on the edge; Types
  and verbs gives the reasoning.
- **The waking edge is latched too**: an interrupt that wakes a WFE has
  turned pending, so SEVONPEND latches it, and the next WFE returns at
  once on it. PFIC_SCTLR.SETEVENT ("set the event to wake up the WFE
  case", RM 7.5.2.38) sets the latch by hand, and a WFE executed over a
  set latch returns at once and leaves it clear - the pair that consumes
  it. Without that consume a quiet kernel loop turned twice per tick,
  measured once in a probe image (the bench findings, letter `k`).
- **In Sleep the STK keeps its clock** and in a deep sleep every clock
  may stop (V4 manual 6.1); a core in debug mode enters no sleep at all,
  so a sleep observed with a probe halted on the core is not one.
- **The hardware prologue/epilogue (HPE)** saves the sixteen integer
  caller-saved registers to an internal stack the program cannot see, in
  one cycle, and restores them on MRET (V4 manual 3.4; two levels deep
  on this core, RM 7.1.1 and table 1-1). INTSYSCR.HWSTKEN enables it (RM
  7.5.3.1), and a handler declared with the vendor compiler's
  `interrupt("WCH-Interrupt-fast")` attribute relies on it and emits no
  prologue of its own. On the CH32V203's V4B the hardware saved 10
  cycles and 64 bytes of user stack a round trip
  ([../ch32vx03/platform.md](../ch32vx03/platform.md)); on this core the
  round trip is 96 to 100 cycles at 48 MHz with it and 120 to 128
  without (letter `e`, in the bench findings).
- **The STK is sixty-four bits wide** (RM 7.5.5): CNTL/CNTH count up (or
  down, MODE) at HCLK or HCLK/8 to CMPLR/CMPHR, restart from zero there
  when STRE is set, and raise CNTIF, which the handler clears by writing
  zero. It is interrupt 12 of the vector table, and the interrupt number
  is the table index on this core. STK_CTLR also carries SWIE, the
  software interrupt's other way in. This manual gives the register no
  INIT bit (its bits 30:5 are reserved), where the QingKe V4 manual's
  chapter on the counter has one at bit 5; nothing here writes it. The
  compare, the restart and the high halves that never move under it are
  measured.
- **mtvec has two mode bits** (RM 7.5.3.2, V4 manual 3.2): MODE0 set
  enters each interrupt at the table's base plus four times its number,
  and MODE1 set makes each entry the handler's absolute address rather
  than an instruction. The crt sets both, so a handler placed in SRAM is
  entered from its entry directly.
- **Traps land in the vector table by kind**: the exception entry at
  index 3 and a breakpoint's own at index 9. On the CH32V203 an `ebreak`
  with no debugger reached index 9 while mcause said exception code 3
  (measured there); no letter executes an `ebreak` without a debugger on
  this core, so here it rests on the V4's vector table alone.
- **corecfgr (CSR 0xBC0) configures the pipeline and the instruction
  prediction** and the V4 manual gives it no bit table (8.3); this
  series' reference manual does not name it. The crt writes 0x1F into
  it, the value WCH's own startup file for the series writes. On the
  CH32V203's V4B the five bits bought two cycles in 36811 of a timed
  loop; on the CH32V303's V4F that loop ran slightly slower with them
  ([../ch32vx03/platform.md](../ch32vx03/platform.md)). On this core a
  loop of the platform suite's own runs 7.3 per cent slower with them
  than with the register cleared at one placement of the loop (25 per
  cent at another placement, read in an earlier image), while from the
  flash a taken branch or a jump through a register that shares its
  fetch word with a load or a store before it costs 4 cycles less with
  them, the predictor they turn on sending the fetch ahead (both
  measured). Which value a program should want is open (Not covered
  yet).
- **A reset request is PFIC_CFGR.SYSRST written with KEY3 (0xBEEF in the
  high half), or PFIC_SCTLR.SYSRST with no key** (RM 3.2.2, 7.5.2.10,
  7.5.2.38). And with PFIC_SCTLR.RSTEN at its reset value of zero the
  chip has a core deadlock reset, generated - in 3.2.2's words - "when
  the core addresses an exception or enters an NMI interrupt", and never
  in debug mode; which traps raise it is not established (Not covered
  yet), and no verb arms or disarms it.

## Types and verbs

The code is the QingKe core stratum's, written once for the three QingKe
families ([../qingke/README.md](../qingke/README.md)). This family's
[pfic.hpp](../../brio/ch32x035/pfic.hpp),
[ticker.hpp](../../brio/ch32x035/ticker.hpp) and
[delay.hpp](../../brio/ch32x035/delay.hpp) each include the family's
device header and then the core file of the same name
([brio/qingke/pfic.hpp](../../brio/qingke/pfic.hpp),
[brio/qingke/ticker.hpp](../../brio/qingke/ticker.hpp),
[brio/qingke/delay.hpp](../../brio/qingke/delay.hpp)).

The platform type is a struct over
[brio/qingke/platform.hpp](../../brio/qingke/platform.hpp)'s
`QingkePlatform`, given this family's one fact by a chip traits type,
`Ch32x035Qingke`: whether a bus master the Sleep would stall is working,
which here the DMA controller's own registers answer.

For Rubato,
[brio/ch32x035/schedule.hpp](../../brio/ch32x035/schedule.hpp) gives
[brio/qingke/schedule.hpp](../../brio/qingke/schedule.hpp) two facts:
whether the request's store is posted, `scheduling_request_posted`,
which follows where the scheduler path runs (true from SRAM, false from
the flash); and the family's step of `scheduler_init()`,
`scheduler_vector_table_init()`, which copies the vector table into SRAM
when the path runs there. The core files' comments carry the
measurements of all three families' cores, each with its chip named.

`Ch32x035Platform<TB>`
([brio/ch32x035/platform.hpp](../../brio/ch32x035/platform.hpp)) is the
concept member for member. `CriticalSection` is pfic.hpp's
`InterruptGuard`: one `csrrci` reads and clears mstatus.MIE, and the
destructor restores only what it found set, so guards nest. `idle()` is
a store of PFIC_SCTLR (WFITOWFE and SEVONPEND set, SETEVENT and
SLEEPONEXIT clear - with SLEEPONEXIT set the waking handler's MRET would
sleep again), the unmask, the `wfi` that sleeps as a WFE, and after the
wake a second store setting SETEVENT and one more `wfi`, which returns
at once and consumes the latch the wake left, so the caller's loop turns
once per interrupt.

**Why the WFE entered with MIE set loses no wake.** The kernel calls
`idle()` with interrupts masked, after it found every queue empty, so an
interrupt that posts after that check must end the sleep. The hook
unmasks before the `wfi`. An interrupt already pending at the unmask
runs its handler there, and SEVONPEND has latched its pending edge, so
the WFE returns at once; one that turns pending between the unmask and
the `wfi` is taken there and latched the same way; one that turns
pending while the core sleeps, or on the cycle the WFE goes to sleep,
wakes it as an enabled interrupt with MIE set - a level, the V4 manual
6.2's second wake, whatever the edge's timing - and its handler runs
before the instruction after the `wfi`. In every case the caller's loop
turns and finds what the handler posted. The consume after the wake is
safe for the same reason: MIE is set by then, so an interrupt that turns
pending around the consume has its handler run by its level before the
caller's next masked check, and the latch that the consume may eat
belongs to an interrupt already taken or to a line the PFIC does not
enable, which wakes nothing the kernel waits for.

`break_here()` is `ebreak`, `now()` the timebase's tick count,
`ticks_per_second` the timebase's rate, and `atomic_width` 4: an aligned
32-bit word moves in one uninterruptible access, which is what lets
util/ring.hpp take its lock-free path. The panic record is a
`PanicRecord` in `.noinit`, which the crt neither loads nor zeroes.
`take_panic_record<P>()` tells a record from what the SRAM held at
power-up by one 16-bit magic word, 0xB210 (kernel/panic.hpp), and clears
that word when it returns the record; power-up contents that happen to
hold the word read as a record.

**A receive ring holds the program awake.** In this die's Sleep (RM
2.3.2) the DMA controller moves nothing (measured, [dma.md](dma.md)), so
`idle()` first asks `Dma::any_working()` - a channel enabled and
circular or with items left - and while one is, it unmasks and returns
at once, and the kernel loop spins until no channel works. A receive
ring is a circular channel, working for as long as it runs. The guard
reads the DMA controller's registers in the idle path alone. With the
controller's clock gate (RCC_AHBPCENR's DMA bit) closed it is one load:
four instructions, the address, the load, the mask and the branch. With
the gate open it is, after four instructions of loop setup, five
instructions for a channel that is not enabled and nine for one that is
enabled with its transfer done (the release listing). The CH32V203 and
the CH32V303 answer the same question with a count their drivers keep,
because their USB controllers are bus masters too. This family's USB
controllers have no driver, and **the DMA controller's registers are the
whole answer only while that holds**: a USB driver here would have to
join the question (Not covered yet).

In the release listing the Sleep path after the guard is a leaf - a
function that calls nothing - of fourteen instructions, the consume
three of them (an `ori`, the store and the second `wfi`). With SLEEPDEEP
found set - which no verb of this stratum sets - the path is
`idle_deep()`, out of line: the timebase paused and its pending bit
cleared first, because a tick that is merely pending would end the sleep
before it began, then the same store, unmask and `wfi`, and no consume,
since a second `wfi` with SLEEPDEEP armed would be a second entry into
that mode. The next `idle()` after a deep wake may then return at once
on the latch the wake left, one turn of the kernel loop spent on
nothing; the deep path is unmeasured here (Not covered yet).

The interrupt verbs are [pfic.hpp](../../brio/ch32x035/pfic.hpp)'s:
`enable_interrupts`, `disable_interrupts`, `interrupts_enabled`, and
`Pfic`'s per-line `enable/disable/enabled/pending/set_pending/
clear_pending/active` over the PFIC's set and clear registers, where a 1
written to a bit acts on that line and a 0 leaves the others alone (the
manual's PFIC_ISRx registers are the enable status, its PFIC_IPRx the
pending one). `Pfic::in_handler()` reads PFIC_GISR's GACTSTA, bit 8 at
offset 0x4C, an interrupt being executed (RM 7.5.2.11, V4 manual 3.1) -
not mstatus.MIE, which a trap's entry leaves set on this core (V4 manual
2.2) and a critical section clears. The platform's `in_handler()`
forwards it: kernel/platform.hpp's optional member, which its
`HandlerAware` concept checks and which a service that may be called
from an interrupt handler requires, to tell a handler's call from the
dispatch it interrupted.

`BRIO_CH32_VECTOR` binds every ordinary handler; the preemptive kernel's
two vectors take `BRIO_CH32_SCHEDULER_VECTOR`, described with
`Ch32x035RubatoPlatform`. On this core, which has no floating-point
unit, the stratum's `BRIO_CH32_LEAF_VECTOR` and `BRIO_CH32_VECTOR_FLOAT`
are `BRIO_CH32_VECTOR` exactly. `BRIO_CH32_VECTOR`'s attribute is WCH's
fast one when the image is built with `CH32X035_HPE` (the default),
gcc's plain `interrupt` otherwise, and both carry `no_icf`. **Without
`no_icf`, gcc's identical code folding turns the second of two handlers
of one body into a handler that calls the first, which ends in MRET, so
the call never returns and the caller's frame stays on the stack**
(measured on the CH32V303VCT6,
[../ch32vx03/platform.md](../ch32vx03/platform.md)).

Under Tenuto interrupt nesting is never enabled: no interrupt handler
interrupts another, the kernel's rule ([design/kernel.md, "The model in
one page"](../design/kernel.md#1-the-model-in-one-page)). The preemptive
platform keeps that rule for every peripheral handler and turns nesting
on for one line alone, its scheduling request, which sits below every
peripheral line: a peripheral handler may interrupt the request's short
body (nine instructions under the hardware prologue) and nothing else,
and that body is no third context for a driver, as the same page says of
Rubato.

`Ch32x035RubatoPlatform<TB>` (the same header) is `Ch32x035Platform<TB>`
whole plus the three members of a preemptive platform, the one the
preemptive kernel `Rubato` runs on
([../design/kernel.md](../design/kernel.md), "Rubato"): `preemptive`,
`ceiling` (the index of the object being dispatched, 255 at the loop's
base) and `pend_schedule()`, which is
[brio/qingke/schedule.hpp](../../brio/qingke/schedule.hpp)'s
`request_scheduler()`: the software interrupt (line 14 of the vector
table, RM 7.3) pended through PFIC_IPSR1 (RM 7.5.2.25), four
instructions out of line, five where it ends with a `fence.i`. The
kernel reaches a higher-priority object by two roads: road 1 is a post
from an interrupt handler, which preempts the running object at the
handler's exit; road 2 is a post from one object's dispatch to a
higher-priority object, which runs inside the post.

`scheduler_init()` puts the software interrupt's priority byte at 0x80
and sets INTSYSCR.INESTEN with one preemption bit (V4 manual 3.2), so
every peripheral line, at the reset priority 0, sits one level above it:
the peripheral handlers still never nest among themselves, and every one
of them interrupts the software interrupt's. The V4C's two nesting
levels and two-deep hardware prologue (table 1-1) are exactly that
depth.

The scheduler runs in thread mode - machine mode outside any trap, the
interrupted code's registers saved on the stack. The software
interrupt's body, `scheduler_trap<K>()` (K the program's kernel type),
pushes a 16-byte block with the trap's mepc and mstatus, writes the
entry's address into mepc and returns (nine instructions). The entry,
`scheduler_thread<K>()`, finds in the register file the interrupted
code's sixteen caller-saved registers, which the hardware prologue
restored at that MRET; it stores them, sets MIE, calls `K::activate()`,
loads them back and executes `ecall`. The machine-mode environment
call's body, `scheduler_return()`, puts mepc and mstatus back from the
block, pops it and returns into the interrupted code (six instructions).
Without the hardware prologue the two bodies save their one scratch
register in the block themselves. This core has no FPU, so the entry has
nothing else to save; the request and the two bodies are the code the
CH32V203 and the CH32V303 run, the f-register save compiling to nothing
here.

The two vectors are bound with `BRIO_CH32_SCHEDULER_VECTOR(name)` under
the crt's names, `software_handler` and `ecall_m_handler`: naked (no
compiler-made prologue), `no_icf`, aligned to four like every vector of
the stratum, and marked a leaf with an absolute symbol,
`__brio_leaf_<name>`, which costs no byte. The CH32V303's vector guard
reads that mark in its F images; on this core nothing reads it, and the
bodies are leaves under the hardware prologue - no call, nothing saved
in software - and save their scratch register without it. **A program
executes no `ecall` of its own**: the environment call's vector is the
scheduler's way back, and an `ecall` from anywhere else would run
`scheduler_return()` over a stack that holds no block and return to
whatever two words lie there. **A program calls `scheduler_init()`
before it enables interrupts.** Nothing checks either rule.

Whether the request ends with a `fence.i` depends on where it runs,
which here the program chooses ("The scheduler's place"). A request made
from a dispatch must start the higher-priority object before the
instruction after the store: if instructions retire behind the store,
the code after a post runs before the object it posted to. This core
takes the trap at the instruction after a store run from the flash (the
platform suite's letter `i` reads mepc in the handler), so from the
flash the request is the store alone. The same store run from SRAM
retires three compressed instructions before the trap (two with the
branch predictor off), as the V4B's does (the platform suite's letter
`j`, the same reading with the sequence placed in SRAM); there the
request ends with the `fence.i`, after which the trap is taken at the
next instruction, 12 cycles from the store to the handler against 10
without the fence. `scheduling_request_posted`, the fact
[brio/ch32x035/schedule.hpp](../../brio/ch32x035/schedule.hpp) gives the
core file, follows the placement.

**The scheduler's place.** The preemptive kernel's scheduler path - the
two scheduler vectors, the thread entry, the request, `activate()` and
the out-of-line verbs of the objects' queues - runs from SRAM or from
the flash as a `// build:` line in the program's source says, one of the
lines of its header comment that the project's build reads
([../boards/README.md](../boards/README.md)): `// build: scheduler =
ram` or `// build: scheduler = flash`. A file without the line is `ram`;
any other value stops the configure with the app's name. The line sets
two things for the image
([ch32x035/CMakeLists.txt](../../ch32x035/CMakeLists.txt)). One is the
definition `BRIO_CH32_SCHEDULER_IN_RAM`, 1 or 0, from which this
family's schedule.hpp takes the request's `fence.i` and
`scheduler_init()`'s step. The other is which placement fragment the
series' section layout includes into `.data`
([ch32x035/ld/sections.ld](../../ch32x035/ld/sections.ld)):
`ld/scheduler_ram/scheduler_text.ld` names the path's sections,
`ld/scheduler_flash/scheduler_text.ld` none, and the crt's copy of
`.data` moves what is named into SRAM before `main()`.

The fragment names the path's functions by input-section patterns - the
linker-script wildcards that match a function's section, one section a
function under `-ffunction-sections`. Each pattern names a function only
a preemptive image holds: the two vectors, whose binding gives them a
section of their own; the thread entry and the request, which are the
scheduler's own; `activate()` and the queues' verbs, whose mangled names
carry the platform's type, `Ch32x035RubatoPlatform` (an object's queue
is `EventQueue<Event, N, P>`). So the kernel knows nothing of the
choice, and the line changes nothing in an image built with Tenuto,
which holds none of these functions: the byte-identity gate found every
Tenuto image of the project the same bytes with either default, measured
once; no check repeats it.

Under `ram` the vector table moves too: `scheduler_init()` copies the
crt's 55 words into `scheduler_vector_table`, 220 bytes of `.bss`, and
points mtvec at the copy with both mode bits. The entries name the same
handlers, so a program binds nothing new, and the crt's table stays
where the core boots from, serving every trap until then. From SRAM a
trap through the table in the flash costs 18 cycles from the store that
raises it to the handler and 10 through the copy, the entry fetched over
the flash's wait states (measured by the platform suite's letter `j`).
The copy is `scheduler_init()`'s and not the crt's because the crt is
assembled for every image with the image's value of the definition, a
Tenuto image's included, and only a preemptive program calls
`scheduler_init()`.

What stays in the flash under either value: the post verb itself
(kernel/post.hpp's `post<Ao, Ev>`, whose name carries no platform, so a
pattern for it would move a Tenuto image's posts too; its callee
`push()` and the request are placed), the time events' service, every
interrupt handler, and every dispatch the compiler keeps out of
`activate()`, which a program places with `.fast_text` ("A handler in
SRAM").

The default is `ram` because the path's functions exist only in a
preemptive image, so a Tenuto image pays nothing whatever the default,
and a program uses the preemptive kernel for its latency: from SRAM road
1 takes about half the cycles it takes from the flash (the bench
findings). `scheduler = flash` is the opt-out for a preemptive program
short of RAM, back to the flash's figures and to the request as the
store alone.

`stack_untouched()` is the RAM ledger: the crt paints the free RAM
between the last section the linker placed and the stack top with one
pattern before the first call, and the verb walks up from that floor
until the pattern breaks - how many bytes of stack no call has reached
since reset.

The timebase is [ticker.hpp](../../brio/ch32x035/ticker.hpp)'s
`BasicTicker<tps>` over the STK (`Ticker` at 1000 Hz; `init`, `tick`,
`ticks/millis/secs/now`, `pause/resume`, `advance` for a sleep site that
stopped the counter, `rebase` for a clock that changes rate at run
time), counting HCLK - STCLK set - up to a compare of hz/tps - 1, where
hz is HCLK's rate as the clock type states it and tps the ticks per
second, with the hardware restart, the high halves never moving. A rate
that does not divide 1000 is refused at compile time, so `millis()` is
exact.

The busy-wait is [delay.hpp](../../brio/ch32x035/delay.hpp)'s
`delay_us(clock, us)`: it waits at least `us` microseconds and never
less, and divides nothing at wait time. It refuses, spending no time, a
request of one tick period or more (1000 us at the 1000 Hz `Ticker`), a
request of 65536 us or more (the limit that keeps its 32-bit product
from wrapping), any request while the counter is stopped, and any
request when the clock gives zero cycles a microsecond. Its answer is a
`[[nodiscard]]` bool, true when the wait was served. Under a
`DynamicClock` (a clock whose rate the program changes at run time,
[clock.md](clock.md)) it reads its cycles-per-microsecond factor from a
table built at compile time, indexed by the current rate; below 1 MHz of
HCLK that factor rounds up to one cycle a microsecond, so the wait is
longer than asked, never shorter.

What the crt writes, in
[startup_ch32x035.S](../../ch32x035/src/glue/startup_ch32x035.S): the
global pointer and the stack, `.data` and `.bss`, the paint, then
corecfgr = 0x1F, mstatus = 0x1880 (machine mode, MPIE set, MIE clear),
INTSYSCR = 0x1 when the image is built with the HPE and nothing
otherwise - never INESTEN - and mtvec = the table with both mode bits
(which a preemptive image with its scheduler path in SRAM points at the
copy in SRAM later, in `scheduler_init()`); then the static constructors
and a call to `main()`. Every handler is weak: an alias of
`default_handler`, or for the exception and the breakpoint entries a
spin of its own.

## How to use it

The opening's program is the whole of a Tenuto program's glue: the
platform named, the vectors it owns bound with `BRIO_CH32_VECTOR`, the
clock started before the timebase. What it did not show follows.

A wait shorter than a tick is `delay_us`, and its answer is acted on. An
excerpt, with the opening program's definitions and `#include
"ch32x035/delay.hpp"`, for a dispatch or for `main()` after
`Ticker::init()`:

```cpp
if (!brio::delay_us(clock, 200)) {
    brio::panic<P>(brio::PanicCode::assert_failed);   // refused, no time spent: the ticker is not running
}
```

A wait of a tick or more is a time event. A panic is `panic<P>(code,
context)` (kernel/panic.hpp): the record written, then `break_here()`.
With a debugger attached the program stops there. **Without one, the
`ebreak` lands in the crt's breakpoint spin and stays there: nothing in
this family resets the chip, so the record reaches the next boot's
`take_panic_record<P>()` only through a reset from outside the program,
such as the probe's.** That the SRAM keeps its contents across a reset
is promised by no document
([brio/qingke/platform.hpp](../../brio/qingke/platform.hpp)), and no
letter measures what a power cycle leaves in it.

The same program under the preemptive kernel: the platform type, the two
naked vectors and one call more. An excerpt of the opening program,
whose `Second` and `Uptime` stay as they are; the excerpt's lines
replace its `kernel/tenuto.hpp` include, its `P` and its glue:

```cpp
#include "kernel/rubato.hpp"   // in place of kernel/tenuto.hpp

using P = brio::Ch32x035RubatoPlatform<>;
// ... SysClock, clock, Second and Uptime as in the opening ...
using K = brio::Rubato<P, Uptime>;   // the active objects, highest priority first

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return(); }

int main() {
    (void)brio::take_panic_record<P>();   // as in the opening
    if (!SysClock::init() || !brio::Ticker::init(clock)) {
        brio::panic<P>(brio::PanicCode::kernel_fault);
    }
    brio::scheduler_init();      // before the first unmask: nesting on, the request below every line
    brio::enable_interrupts();
    K::run();
}
```

`software_handler` and `ecall_m_handler` are the crt's names for vectors
14 and 5. That program runs its scheduler path from SRAM, the default,
and `scheduler_init()` moves the vector table there too, before the
interrupts are enabled. The path costs SRAM: 1724 bytes for the
preemptive suite's five-object pack, a program's `activate()` being as
large as the dispatches the compiler inlines into it (the bench
findings). A program that cannot spare the RAM opts out with one line
among the `// build:` lines of its header comment:

```cpp
// build: scheduler = flash
```

That line is a comment the build reads, not code. The path then runs
from the flash, the request is the store alone and the vector table
stays the crt's.

## Bench findings

The setup: a CH32X035F8U6 - WCH's evaluation board in its QFN20 edition,
over a WCH-LinkE - at 48 MHz from the HSI with the flash at two wait
states, the board's rail supplied (with none, fed through the pads'
protection diodes, it restarts under load at 48 MHz:
[../boards/ch32x035-evt-f8u6.md](../boards/ch32x035-evt-f8u6.md)).
`test_x035_platform` (209 verdicts in `z`, nothing wired) ran ten times
in the image built with the hardware prologue and three times in an
image built without it; each figure of this section says which, and over
how many runs.

Letters `w` and `k` were also run at 8 MHz, three runs each, in an image
built from the same source with the suite's clock line set to 8'000'000 -
HPRE dividing the HSI by six, and the clock task setting the flash to no
wait state (RM 20.3.1). That image is not kept in the tree. What the
suite measured:

- **The boot story** (letter `a`, ten runs). The flags at boot read
  0x10000000, SFTRSTF alone, after the probe's programming and reset,
  and `reset_flags()` reads them without disturbing them - two reads
  agree; after `clear_reset_flags()` the whole register reads zero, RMVF
  included, which on this silicon takes the verb's second store
  ([clock.md](clock.md)). The electronic signature gives 62 KB of code
  flash, and the unique identifier reads 0x2917ABCD, 0x9129BC45 and
  0xFFFFFFFF - its third word, bits 95..64 of the 96 RM 19.1 gives it,
  all ones on this die. The word at 0x1FFFF704 reads 0x035E0611 - the
  value WCH's EVT library expects for this part, 0x035E06x1, with 1 in
  bits 7:4 ([pin.md](pin.md) for what that decides).
- **The critical section and the idle hook** (letter `b`, ten runs). The
  guard masks, a nested one still masks, leaving the inner scope does
  not unmask and leaving the outer one does. A 5 ms window with
  interrupts masked advanced the tick by one: the STK interrupt is a
  pending bit, one tick delivered and the rest coalesced. With the
  console silent, two `idle()` calls - each made with interrupts masked,
  as the kernel makes it - covered the 234 to 792 us to the next tick
  and returned with interrupts enabled: the WFE form wakes on this core.
- **No edge position loses the wake** (letter `w`). The STK's compare is
  placed 0 to 599 cycles ahead, one cycle a step, before the kernel's
  own shape - a masked check, `idle()`, until one tick or two - with the
  event latch set and cleared by hand before each try, so the edge walks
  across every instruction of the idle path and of the loop around it,
  the sleep entry and the consume included. TIM3 counting at 1 MHz is
  each try's clock and its rescue timer: a try that reaches its 50 ms
  period is a wake lost for good, and one a tick period late a wake lost
  until the next tick. In the four cases (one tick or two, the latch set
  or clear), 600 positions each: none lost and none a tick late, at 48
  MHz in ten runs and at 8 MHz in three.

  | figure, the hook's order (unmasked) | 48 MHz, ten runs | 8 MHz, three runs |
  |---|---|---|
  | the slowest try, waiting for one tick; for two | 17 us; 1017 us | 85 to 86 us; 1085 us |
  | an edge that finds the core asleep, to the handler's first statement | 43 cycles | 21 cycles |
  | the same edge to the caller's loop | 167 cycles | 66 cycles |
  | an `idle()` the latch returns at once, two counter reads included | 92 cycles | 30 cycles |

  The two images run the same code, the DMA guard included; the cycles
  differ by the fetch, the flash answering behind two wait states at 48
  MHz and none at 8 MHz, and letter `i`'s fetch model is measured at 48
  MHz alone.
- **One turn per wake** (letter `k`). A Tenuto pack of three quiet
  active objects, two holding a periodic time event that does not fire
  in the window, turned as `Tenuto::run()` turns it over 100 ticks with
  the tick the only interrupt: 101 kernel turns (the window opens and
  closes on a tick, so a turn straddling one end counts), at 48 MHz in
  ten runs and at 8 MHz in three. A quiet turn whose `idle()` the latch
  returns at once - the turn an interrupt would cost twice over with the
  latch left - takes 416 cycles at 48 MHz and 141 at 8 MHz, two counter
  reads included. Without the consume the waking edge, left latched,
  ends every second `idle()` at once: measured once in a probe image
  with the consume removed, 200 turns over 100 ticks; no image of the
  tree reproduces it ([../ch32v00x/platform.md](../ch32v00x/platform.md)
  and [../ch32vx03/platform.md](../ch32vx03/platform.md) carry the same
  measurement for those chips). The consume costs three instructions of
  the listing.
- **The STK timebase** (letter `c`, ten runs). CTLR reads 0xF - counting
  up on HCLK, restarting, interrupting - with CMPLR = 47999 as
  programmed and CMPHR and CNTH both zero: the restart sets the low half
  back to zero at the compare, so the high half of the 64-bit counter
  never moves. `ticks()`, `millis()`, `secs()` and `now()` agree (in one
  run 39875, 39875, 39 and 39.875 s). Over 200 restarts bracketed
  between two tick edges, the counter-delta accumulation `delay_us`
  rests on came to 9599872 to 9599992 cycles against 9600000: 8 to 128
  cycles short, 13 ppm at most, the polling loop's own granularity at
  the span's two ends (the suite's verdict holds it under one period).
- **`delay_us` is late by a constant, never early** (letter `d`, ten
  runs). 100 us spent 4908 to 4940 HCLK cycles for 4800 asked, and 900
  us - the longest wait the letter asks under the 1000 us tick period -
  43316 to 43340 for 43200: 108 to 140 cycles late at both lengths, not
  a proportional error. Twenty waits of 500 us took 10 ticks in nine
  runs and 11 in one, the twenty waits' own lateness and the tick's
  phase at the two ends deciding which (the verdict accepts 10 to 12);
  1000 us, 50 ms and 65536 us are refused, and a zero wait is served.
- **The interrupt round trip, in HCLK cycles** (letter `e`): the
  software interrupt - vector 14 - raised by hand through the PFIC's
  pending register, with the cycle counter read at the raise, at the
  handler's first and last statements and when the raiser sees it done,
  64 rounds each and every raise taken. With the hardware prologue (ten
  runs): entry 36 in every round, body 26, exit 34 with one iteration of
  the raiser's spin in it, the whole trip 96 to 100. Without it (three
  runs): entry 42, body 28, exit 50, the trip 120 to 128 - the prologue
  buys 24 to 28 cycles on the minimal handler, the sixteen registers
  saved and restored by gcc's own code instead. The user stack the image
  never touched reads 8796 bytes with it and 8764 without: 32 bytes, the
  deepest handler frame's software saves, which the hardware stack takes
  instead.
- **The InRam bodies without the hardware prologue** (`test_x035_spi`
  letter `b`, three runs of an image built with `CH32X035_HPE=OFF` whose
  vectors were not aligned to four; no image of the current alignment
  has been built so and run). In that image the SPI pump's frame - its
  body placed in SRAM, called from a vector in the flash - took 289
  cycles, and 332.5 with the body inline in the vector in the flash,
  some 65 cycles a frame more either way than the image with the
  prologue gave at that layout (224 and 268): the vector saves the
  caller-saved registers in software around the call. Its handler bodies
  took 148 cycles (the console), 69 (the tick) and 270 (the pump), all
  under the SPI host's default hold-off of 380 cycles: the longest wait
  behind another handler or a masked section the host allows for when it
  writes ahead, sending the next frame before it has read the last, two
  frames in flight ([spi.md](spi.md), "The hold-off"). The SRAM
  transmit-only loop read 17.0 cycles a frame at /2 against the wire's 16.
  The SPI host's cost constants are those of the default image, so the
  letter's two cost verdicts fail in an image built without the
  prologue, which is expected.
- **The tick inherits the HSI's error.** The STK counts HCLK, the HSI
  divided, so the tick is as accurate as the HSI, which a crystal
  measured at -0.25 per cent ([clock.md](clock.md), "The HSI against a
  crystal"). Letter `g` brackets 5000 ticks between two console lines
  for a host to time; no host-timed figure of it stands here.
- **corecfgr's 0x1F makes a data-dependent loop slower** (letter `f`).
  The crt left 0x1F in the register; 2000 iterations of a loop with a
  load and a data-dependent branch over a 64-byte pseudo-random table
  took 24200 HCLK cycles with it in all ten runs, and 22544 in every run
  with the register cleared - 7.3 per cent more with the vendor's bits
  at that placement of the loop - the register reading back zero for the
  cleared loop and 0x1F after it. The figure moves with the loop's
  address modulo four: at another placement of the same loop the gap
  read 25 per cent, in an earlier image. In the image without the
  prologue the loop took 23960 to 24208 cycles with the bits (three
  runs), 22544 without.
- **What the crt left in the core** (letter `h`, ten runs): INTSYSCR =
  0x1, the hardware stack on and nesting off (0x0 in the image built
  without the prologue); mtvec = 0x3, the table at the image's base
  0x0000 0000 with both mode bits; misa = 0x40901105, a 32-bit core with
  A, C, I and M, and U and X beside them. mstatus reads 0x1888 in a
  letter - machine mode, MIE and MPIE set - with MPP = 3, the crt's
  value, though the tick's handler has returned through MRET thousands
  of times before the letter reads it: an MRET leaves MPP where it found
  it, as the QingKe V4 manual's 8.2 has it (an MPP of 3 written at
  startup keeps the core in machine mode after every return) and not as
  the privileged specification does, whose MRET drops MPP to the
  least-privileged mode the core implements - user mode here, by misa.
- **The RAM ledger answers.** `stack_untouched()` read 8796 bytes never
  touched after this suite's whole `z` (ten runs) of the part's 20 KB.
  Letter `j` holds 9360 bytes of SRAM for its code and its vector table,
  and its deepest call chain takes 1124 bytes of stack, where in an
  image of the suite without letter `j` the deepest chain took 820. The
  project's other suites print their own figure after each command the
  same way, the two preemptive suites their stack's high-water mark
  instead.

**The fetch and trap cost model** - letter `i`, 54 verdicts, every
figure but the predictor rows the same in each of the ten runs: the
figures `test_x035_rubato`'s bounds are drawn with ("The preemptive
kernel", in this section), measured on hand-placed assembly at 48 MHz
from the flash and timed on the STK's CNTL read bare - one 32-bit load
of the counter's low half, nothing composed. A fetch figure is a pattern
of N units timed against the same pattern of 2N, each the least of 32
calls made with interrupts masked, so the difference is N units and
nothing else. A trap figure is the difference of two stamps, one 32-bit
load of the counter each, the first one word ahead of the instruction
that raises the trap and the second the handler's first instruction,
less the stamp's own word - two loads a word apart read 4 cycles apart -
judged in every one of 256 trips, the few a tick landed in dropped. The
software interrupt is letter `e`'s vector, so for its rows mtvec points
at a copy of the crt's table, elsewhere in the flash, with that one
entry replaced; TIM2's capture-compare line, pended by a store from the
flash and timed through both tables, says the table's place costs
nothing from the flash. The figures, in HCLK cycles, with corecfgr as
the crt leaves it and again with it cleared, each fetch and trap figure
judged to the cycle. The predictor rows give the extra cycles a pattern
costs when the predictor was trained on the opposite outcome, compared
with the same pattern after training on its own outcome; they are judged
against the allowance described after the table:

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
| sixteen 32-bit branches, the same | 16 to 20 more | 0 |
| sixteen compressed branches, the same | 56 to 64 more | 0 |
| any of them taken right after a fall-through training; sixteen calls through a register to the other of two leaves | 0 | 0 |

mepc reads the instruction after the store and after the `csrsi`: from
the flash, where the letter runs, the trap is taken there and nothing
retires behind either; the same sequences run from SRAM retire three
compressed instructions behind the store and two behind the `csrsi`
(letter `j`, "The SRAM cost model"). The predictor corecfgr's 0x1F turns
on costs nothing when it guesses right. A branch it guessed taken that
falls through costs up to 8 cycles more, one it guessed to fall through
that is taken nothing, and sixteen in a row cost at most 64 more, 4 a
branch. Over the 32 timed calls of a row of not-taken branches the
slowest exceeded the least by up to 48 cycles with sixteen branches and
80 with thirty-two.

The Rubato bounds take an allowance of 4 cycles at each predicted site
of a path in the flash (3 at a site in SRAM, letter `j`'s figure), the
path's calls and returns counted, and every predictor row here is judged
so and is within it. The allowance holds over a path, not at each site:
one wrong guess alone can cost 8, twice a site's share. With the
register cleared no call of a pattern differs from another, and every
figure is the same but one pairing: a transfer the core resolves when it
executes - a taken branch, a jump through a register - that shares its
fetch word with a load or a store before it waits 4 cycles more; a
direct jump, resolved at its decode, does not, and with 0x1F the
predictor sends the fetch to the target before the transfer executes. In
the image built without the hardware prologue (three runs of `z`) every
fetch and trap figure is the same to the cycle: the core's push and pop
cost a trap nothing measurable.

**The SRAM cost model** - letter `j`, 104 verdicts, ten runs of the
letter alone and ten of `z`, every figure but the predictor rows the
same in all twenty: the figures `test_x035_rubato`'s bounds are drawn
with for the image that runs the scheduler path from SRAM, measured as
letter `i` measures the flash's. Its patterns, three twins of letter
`i`'s handlers and the harness that calls and times them are placed in
SRAM with `.fast_text` (the only code this suite places there). Three
rulers stand in the two letters and the preemptive suite, each a pair of
bare CNTL loads: letter `i`'s, two loads a word apart in the flash, 4
cycles; letter `j`'s twin in SRAM, two loads back to back, 1 cycle apart
with corecfgr as the crt leaves it and 2 with it cleared; and the
preemptive suite's, two loads in the flash as the compiler lays them, 3
cycles in its images built with the hardware prologue and 1 in those
built without. A ruler is the distance between the two loads' samples,
not the cost of a load, which the table gives; why corecfgr moves letter
`j`'s by a cycle is not established. A trap figure is the stamps'
difference less the ruler of the memory its first stamp is in. The rows
through a vector table in SRAM point mtvec at a copy of the crt's table
the letter fills at its start, in SRAM beside its code; the rows through
a table in the flash at a copy of the crt's table in the flash whose
software-interrupt entry points into SRAM; TIM2's line, pended by a
store, reaches letter `i`'s handler in the flash or a twin in SRAM.

The figures, in HCLK cycles, are judged by two rules. A pattern row's
least, the least of its 32 calls, is judged to the cycle, or with
corecfgr's 0x1F, where the pattern has predicted sites, within one wrong
guess, 3 cycles, because the least can carry the predictor's history
when one of the pattern's sites shares a predictor entry with a branch
of the loop around the call; in the twenty runs no row's least needed
it. A predictor row is judged against 3 cycles at each predicted site of
its window - the pattern's sites and the harness's call and return. A
trap figure is judged to the cycle in every one of 256 trips:

| figure | 0x1F | cleared |
|---|---|---|
| an instruction, 32-bit on a word or half a word off, or compressed | 1 | 1 |
| a taken `c.j` or `j`, its target in one word | 2 | 2 |
| a `j` to a 32-bit instruction across two words | 3 | 3 |
| a taken conditional branch, its target on a word | 2 | 4 |
| a conditional branch not taken | 1 | 1 |
| a call and its return, both targets on a word: by `jal`, by `jalr` | 6, 6 | 6, 8 |
| a store and a taken branch in one fetch word | 5 | 6 |
| a load and a call through a register in one fetch word, the return included | 7 | 10 |
| a load from SRAM or from a peripheral, alone or in a run | 2 | 2 |
| a store to a peripheral alone, 32-bit or compressed | 1 | 2 |
| a 32-bit store to SRAM alone among 32-bit instructions; in a run | 3; 2.25 | 3; 3 |
| a compressed store to SRAM alone among compressed instructions; in a run of two a word | 1; 1.6875 (1.625 the run half a word off) | 2; 2.5 |
| from the flash, a call into SRAM through a register and its return, the call's word included | 16 | 18 |
| a store pending the software interrupt, to its handler in SRAM: through the table in SRAM, through one in the flash | 10, 18 | 10, 18 |
| the same with a `fence.i` after the store, the table in SRAM | 12 | 12 |
| the `csrsi` that finds it pending, the table in SRAM | 10 | 9 |
| an `ecall`, to its handler, the table in SRAM | 9 | 8 |
| an MRET into SRAM code; into flash code | 5; 9 | 4; 8 |
| an MRET into a pending interrupt, to that handler, all in SRAM | 11 | 10 |
| from the flash, a store or the `csrsi` pending the software interrupt, its handler in SRAM: the table in SRAM, in the flash | 12, 18 | 12, 18 |
| from the flash, a handler's MRET into the pending software interrupt, its handler in SRAM: the table in SRAM, in the flash | 12, 20 | 12, 20 |
| a store in SRAM pending a line whose handler is in the flash: the table in SRAM, in the flash | 14, 20 | 14, 20 |
| one branch, falling through right after a taken training | 0 to 4 more | 0 |
| sixteen branches, the same: 32-bit, compressed | up to 24, up to 24 more | 0 |
| one branch taken right after a fall-through training; sixteen, 32-bit and compressed | 0 to 2; up to 18 and 16 more | 0 |
| sixteen calls through a register to the other of two leaves | 0 | 0 |

A load costs its 2 cycles wherever it reads. A store alone to a
peripheral costs 1, 32-bit or compressed; to SRAM a compressed store
among compressed instructions costs the same, and a 32-bit one among
32-bit instructions 2 more. The rows say where the difference lies but
not why; the reading they suggest - the store's write and the fetch of
the code around it share the SRAM, and 32-bit code, a word a cycle,
leaves the write no free cycle where compressed code, a word every two,
does - is an inference, the memory's arbitration being in no document
read here.

mepc read in the handler: from SRAM three compressed instructions retire
behind the store that pends the software interrupt (two with corecfgr
cleared) and two behind the `csrsi` that finds it pending; after a
`fence.i` the trap is taken at the instruction after the fence; from the
flash nothing retires behind the store or the `csrsi`, their handler in
SRAM. The vector table's place: a table in the flash costs 8 cycles more
for a trap from SRAM code to a handler in SRAM and for a flash handler's
MRET into one, 6 for a trap from SRAM code to a handler in the flash and
for a store or a `csrsi` in the flash to a handler in SRAM.

The branch predictor's wrong guess costs 3 cycles in SRAM (4 at most at
one site, a 32-bit branch half a word off, where the flash's costs up to
8), and in SRAM a branch it guessed to fall through that is taken costs
too, where in the flash it costs nothing. The spreads move in steps of
3: over the 32 timed calls of the row of sixteen not-taken branches the
slowest exceeded the least by up to 39 cycles, 13 wrong guesses over
that window's 18 predicted sites, in every image. So the letter judges
every predictor row with 3 cycles at each predicted site in SRAM, the
allowance the Rubato bounds take there too, and letter `i`'s 4 at each
site in the flash. In the image built without the hardware prologue
(three runs of the letter alone and three of `z`) every one of these
figures is the same to the cycle, the traps from and into SRAM included:
there, as from the flash, the core's push and pop cost a trap nothing
measurable.

**The preemptive kernel** - `test_x035_rubato`, the scheduler path in
SRAM (the default), and `test_x035_rubato_flash`, the same letters with
the path in the flash; 40 verdicts in `z` each, on the CH32X035F8U6 at
48 MHz. Each was run ten times in its image built with the hardware
prologue and ten times in its image built without it, every image built
from the tree as it stands and judged by the bounds its letters derive
from its own release listing (letter `c` states the arithmetic). Forty
of forty runs passed every verdict. Letter `a`'s first verdict reads the
ceiling the UART's receive interrupt saw when the letter's key arrived
and accepts the loop's two values, 255 at its base and 0 while its base
turn walks the time events (kernel/rubato.hpp holds it there so that a
timer's fire only queues); a key landing in the walk is rare - one run
in these forty - and no dispatch of Top, whose
index is also 0, can be open then, the suite arming nothing on Top.

The pack is five objects, the Console the lowest and the letters'
runner; TIM2's capture-compare line is the interrupt the road-1 letters
land, and nothing is wired or lit. The ruler is the STK's CNTL read
bare, two reads 3 cycles apart in the images with the hardware prologue
and 1 in the others, against 76 cycles for the ticker's composed
`cycles()` (72 in the flash image without the prologue). Over spans of
half a millisecond and more the two rulers agreed within 12 cycles in
39 runs of the forty and by 92 in one, which this page does not
attribute, inside the verdict's twice the composed read; the console is
drained before the reads: its transmit interrupt landing between a
pair's two reads counts as the rulers' disagreement (up to 72 cycles
measured so, 200 to 208 with the predictor off).

The traces, the same in every image: road 1 preempts at any depth - an
interrupt's post to High inside Low's dispatch runs High to its end
before Low resumes, ten runs alike, each object reading its own index as
the ceiling, and the UART's handler reading 255, the value at the loop's
base, in all but the one run whose key arrived during the base turn's
walk, which read 0; road 2 nests High inside Mid inside Low, each inside the post that
asked; at depth 2 an interrupt over High's dispatch, started by either
road, runs Top before High resumes. Three timers matured in one turn are
posted while the base turn walks the time events, with the ceiling held
at 0 so that every fire only queues, and the scheduler then serves them
highest first; High preempting Low's loop of `arm()` and `disarm()` (in
the loop's sixth turn of 200) leaves the list whole; a post under the
mask pends the software interrupt, and the receiver runs when the
section ends, before the statement after it. INTSYSCR reads 0x3 once the
nesting is on (0x2 without the prologue): PMTCFG, the nesting-depth
field `scheduler_init()` writes, reads back zero, as on the CH32V203,
whose V4B ignores the write with its nesting fixed at the two levels and
one preemption bit asked for (qingke/schedule.hpp); the preemption
measured here runs with it so.

The bounds rest on the two cost models. From the flash, as letter `i`
measures it: straight-line code a word every 4 cycles, a taken transfer
8 to its target (12 to a 32-bit target across two words), a trap's entry
20 cycles from the instruction that raises it, an MRET into code 8 and
into a pending interrupt 24, the same with the hardware prologue and
without it, and an allowance of 4 cycles at each predicted site of a
path for the branch predictor. Against that model the flash image's
leasts lie within a cycle of their counts: road 1 a cycle under its
count (307 against 308); `activate()`'s scan, the quiet post and the road-2
post 1 over theirs (225 against 224, 141 against 140, 785 against 784),
the ruler's 3 cycles being the model's 4; road 1 at depth 2 and the
post under the mask at theirs (268, 332); the spread above each least
is the predictor's. From SRAM, as letter `j` measures it: an instruction a
cycle, a taken transfer 2 or 3, a load 2 and a store 1 to 3 by its
neighbours, counted as 2, a trap's entry 10 through the table in SRAM
(12 behind the request's `fence.i`, 9 for an `ecall`), an MRET 5 into
SRAM code and 9 into the flash, 12 for a trap or an MRET in the flash
into a handler in SRAM through that table, a call from the flash into
SRAM and back 16, and an allowance of 3 cycles at each predicted site in
SRAM, the letter's wrong guess. A path that crosses between the two
memories counts above what it measures, because a crossing's figure is a
pair measured whole and split by the model: there the count is the
bound's starting point and not its least. Road 1 counts 154 by the SRAM
model and measured 154 to 160 at each run's fastest; the road-2 post
counts 418 against 406 to 408 measured, the scan 118 against 112.

In the table a figure past the double dots is the most of the ten runs.
For road 1 that most is the slowest event no tick landed in: the letter
reads the kernel tick around each event, and an event a tick landed in
took 63 to 79 cycles more than its run's fastest in SRAM and 68 to 80
from the flash (up to 239 and 387 cycles in all), and 74 to 98 and 84 to
92 in the two images without the prologue (up to 290 and 429). The
letter prints its three slowest events, so in a run where three ticks
landed the slowest event no tick landed in is not printed: the SRAM
image without the prologue had four such runs of ten, and its road-1
figure is the six others'. `activate()`'s figure is the least of 16
calls in each run:

| HCLK cycles | path in SRAM (the default) | in SRAM, without the prologue | path in the flash | in the flash, without the prologue | CH32V203C8T6 (144 MHz) |
|---|---|---|---|---|---|
| road 1: an interrupt's last statement to the preempting dispatch's first | 154..170 | 192..209 | 307..323 | 337..357 | 119..121 |
| road 1 at depth 2, over a preempting dispatch | 145..208 | 181..264 | 268..344 | 312..404 | 107..109 |
| a quiet post (it asks for no scheduling) | 108..112 | 110 | 141 | 155 | 55 |
| a road-2 post, the receiver's empty dispatch inside | 406..408 | 419..421 | 785 | 795..803 | 310 |
| `activate()` over four empty queues | 112 | 114 | 225 | 215 | 90 |
| one `arm()`, its masked window inside | 137 | 139 | 141..157 | 155 | 55 |
| a post under the mask: the section's end to the receiver's first | 163..183 | 169..180 | 332..364 | 348..364 | 132..134 |
| stack high-water of the suite's deepest chain (bytes) | 1000 | 1000 | 1000..1012 | 948..1028 | 1000 |

Road 1 is the TIM2 handler's tail, its MRET straight into the pending
software interrupt, that body's nine instructions and its MRET, the
entry's first nineteen - its frame, the sixteen stores, the `csrsi` and
the call - and `activate()`'s visits to Top and High, each queue's two
index bytes compared in line: 89 instructions, the 57 of `activate()`
the same the CH32V203 runs (87 instructions there outside the traps), so
the difference in time is the fetch's. From the flash they take 308
cycles by the model, measured 307 at the fastest event of nine runs and
311 in one: 6.4 us at 48 MHz against the CH32V203's 0.8 us (119 cycles)
at 144. From SRAM the same instructions take 154 by the model, 3.2 us:
the straight line issues an instruction a cycle where
the flash gave a word every 4, a taken transfer costs 2 or 3 where it
cost 8 to 12, and the traps and MRETs into SRAM code cost 5 to 12 where
they cost 8 to 24, while a load and a store pay the 2 cycles the flash's
fetch hid. What stays in the flash is TIM2's tail, the program's own
vector: 12 cycles.

The vector table's copy in SRAM saves 8 cycles at each of the two traps
letter `j` measures on these paths: from SRAM code a trap through the
table in the flash takes 18 cycles to its handler where the copy takes
10, and a flash handler's MRET into the pending software interrupt 20
where the copy takes 12. The `fence.i` the request ends with in SRAM
costs 2 cycles, inside the road-2 post. The post's scheduling share, 294
to 300 cycles in SRAM and 644 from the flash, is two traps from
thread mode - the pend and the `ecall` - and their two returns, around
the entry's 37 instructions (the nineteen before the call to
`activate()`, the sixteen loads, the frame's release and the `ecall`
after it) and the rescan. The post verb itself stays in the flash under
either value ("The scheduler's place"): a quiet post is 108 to 112
cycles from SRAM's `push()` called out of the flash, where a measurement
image with the suite's posts placed by name took 73 to 74. The stack's
1000 bytes are the deepest chain's frames plus the block and the entry's
64 bytes at each level of preemption, as on the CH32V203; the flash
image reads 1012 in nine runs of ten and the flash image without the
prologue 1028 in nine (948 in the run whose key arrived during the walk
of the time events), which this page does not explain (Not covered
yet).

In the images built without the hardware prologue (CH32X035_HPE=OFF) the
traps cost the same and the handlers that save in software pay it. Road
1 takes 337 cycles from the flash, where the model counts 336 - TIM2's
handler restoring sixteen registers in software, the software interrupt
saving its scratch register, and the path's words falling otherwise -
and 192 to 199 from SRAM, where the model counts 190: 30 and 38 to 39
cycles over the prologue's image, the handlers that pay them being in the flash
in both. A road-2 post takes 795 to 803 from the flash, 10 to 18 over
the prologue's image, and 419 to 421 from SRAM, 11 to 15 over. What
corecfgr's 0x1F buys on these paths, measured in the four images against
the same four with the crt writing 0 (one byte of difference each, the
crt's `li` patched in a copy of each image, ten runs of `z` for each of
the eight): from SRAM the bits buy 4 to 67 cycles on every path of the
scheduler - road 1 154 to 160 at each run's fastest against a flat 186,
the quiet post 108 to 112 against 116, the road-2 post 406 to 408
against 473, the scan 112 against 141, the post under the mask 163 to
183 against 197 to 257, 3 to 21 per cent - and their wrong guesses never
give that back, the slowest road 1 without a tick, 170, staying 16
cycles under the cleared register's fastest; from the flash the cleared
register reads road 1, depth 2, the quiet and the road-2 post within a
cycle of the model's count (road 1 307 at its fastest event and 311 at
its slowest without a tick), the post under the mask 4 over it (336
against 332) and the scan 13 over (237 against 224), gaps this page does
not attribute, and 0x1F
the same at its best on road 1 and on depth 2 and 4 under on the post
under the mask (332 against 336), up to 16 over at its worst on road 1
and 28 on the post under the mask, the same as cleared on the quiet and
the road-2 post, 12 under on the scan (225 against 237); on depth 2 its
worst event read 344, 76 over the cleared register's fastest and 12
over its worst (332), and since letter `e` reads no tick around its
events those are not attributed; the images without the prologue move
the same way. The crt keeps 0x1F, the value the CH32V203's and the
CH32V303's crts write too: from the flash it leaves the scheduler's
paths at or under the cleared register's figures at its best and at
most 28 cycles over them at its worst, depth 2's unattributed events
aside, and from SRAM it buys
them up to a fifth of their time, and what it costs a data-dependent
loop is letter `f`'s bullet in this section.

What the path in SRAM costs, read in the two images' maps (`size -A`):
1724 bytes of the 20 KB for the suite's five-object pack. `.data` holds
2104 bytes against the flash image's 600: 1504 more, the path's code -
`activate()` 1210 bytes, the dispatches the compiler inlined into it
among them (a program's `activate()` is as large as its inlined
dispatches) and every queue's `empty()`, which the kernel's header
forces inline, the thread entry 80, `push()` 72, `take()` 44, the two
vectors 30 and 18, `release()` 26, the request 16 - and 8 bytes of
alignment; `.bss` holds the vector table's copy, 220 bytes more. The
stack ledger says the same: after `z` the crt's painted region is 17184
bytes in the SRAM image and 18908 in the flash image. The flash grows by
124 bytes, `.text` and the load image together (18308 against 18184): a
call between the two memories is an `auipc` and a `jalr` where the
flash image reaches its callee with one `jal`.

**The drivers under the preemptive kernel** - the suite is
`test_x035_rubato_link`, the scheduler path in SRAM, and its twin
`test_x035_rubato_link_flash`, the same source with the path in the
flash, against the CH32V006K8U6 with its unchanged `spi_peer` (key `z`)
and `usart_peer` (key `y`). It runs the real stacks inside a Rubato
pack of seven objects: `util/spi_bus.hpp`'s arbiter over the engined
`SpiHost` on SPI1 ([spi.md](spi.md)), `util/line_reader.hpp`'s
`LineReader` over USART4 with both its engines ([usart.md](usart.md)),
a borrower of the reader's lines above it, and on top an object that
TIM2's update interrupt posts to every 97, 193 or 389 us, whose dispatch
spins 300 or 3000 cycles - at 97 us x 3000, 64 % of the core. Every
letter runs once with TIM2 silent as its control and six times under
that pressure, and every pass judges the pressure itself: one dispatch
per period over the pass, none dropped. The target-free half of the
suite is the link suites' shared body,
`avrxt/src/apps/rubato_link.hpp`; the UART key's command channel and
the HSI's calibration are `x035_link_dut.hpp`, the helpers
`test_x035_usart` shares. Key `y` (115 verdicts) ran three times on each
image, every verdict green; key `z` is built and not judged ([spi.md](spi.md),
"Not covered yet"). What it established:

- **The UART receive side under preemption.** Numbered lines echoed by
  the far chip at 460800 and 921600, assembled by `LineReader` from the
  ring and lent to the borrower: 200 a leg, none lost, out of order or
  corrupt, and every port counter zero in 63 readings on each image.
  Each line is its own burst on the wire: USART4's vector is entered
  twice a line (401 entries a leg of 201 lines, a burst's first
  frame and its idle line), so the port is told at every line's end and
  the ring's half and full marks never find a stream still arriving -
  the receive engine wrote inside the borrower's dispatch once in each of
  2 legs of 42 (the flash image's paced legs at 460800) and never in a
  streamed one. The writer, the pack's lowest object, cannot run ahead of a
  receive side above it that takes most of a line's wire time on this
  core: in the SRAM image the paced legs ran at 503 to 774 per mille of
  the wire at 460800 and 358 to 687 at 921600, the least under 97 us x
  3000; in the flash image 379 to 734 and 222 to 628.
- **The transmit engine under preemption.** 300 lines blasted into the
  peer's sink, counted and summed exact at both rates in every pass: 999
  to 1001 per mille of the wire in the SRAM image; in the flash image
  998 to 1001 except at 921600 under 97 us x 3000, 764 to 766, where the
  writer, below an object that takes its spin and the scheduler's path
  from the flash at every tick, no longer keeps the engine's ring
  filled.
- **The loan under preemption.** At 921600 with the borrower spinning
  6000 cycles a line and the lines streamed twenty ahead, every line
  exact and lent alone, the two line buffers strictly alternating, the
  preempting object landing inside the borrower 76 to 669 times a
  pressure pass in the SRAM image and 97 to 1328 in the flash image, 346
  to 618 and 218 to 570 per mille of the wire.
- **The HSI is the UART letters' premise.** The far echo is a polled loop
  with no FIFO that holds two characters of a writer's lead, and both
  ends divide 48 MHz by one divisor, so the lead is the HSI's error
  against the far crystal: measured before every echo letter by the far
  chip timing 3999 gapless frames of this end at 1 Mbaud, -3048 to -2507
  ppm over the six runs - this end slow, the echo never backing up.
  A lead under 38 ppm is what the letters accept (two characters over a
  leg's 14400, less the HSI's 100 ppm of wander between runs a second
  apart).
- **The DMA release contract holds at every hand-over.** Each leg moves
  USART4 from the polled command channel to the Uart task and back
  through the block's reset line ([dma.md](dma.md), "A released
  requester"), every leg byte-exact from its first block; letter `i`
  read both link channels and both SPI channels disabled, the SPI's
  request enables and every DMA enable of TIM2 clear (TIM2's update
  request is a row of channel 2, the SPI's receive channel, and its
  trigger's of channel 8, the ring's), once on each image.
- **Where the code runs.** The release maps: in the SRAM image
  `activate()` is 3014 bytes in SRAM, with the dispatches of Pressure,
  the borrower, the SPI client, the requester below the arbiter and the
  console inlined into it, `.data` holding 3936 bytes against the flash
  image's 744; the arbiter's and the reader's dispatches are reached
  through their state machines' handler pointers, in the flash in both
  images - so is `SpiHost::start()` with its polled loops. The
  placement moves the objects that preempt and borrow, not the polled
  SPI loop.

The images: with the path in SRAM 38500 bytes of text and 3936 of data,
42436 of the 62 KB (67 %), and 8916 bytes of RAM of 20 KB (44 %); with
the path in the flash 42196 (66 %) and 5504 (27 %).

**The I2C stack under the preemptive kernel** - the suite is
`test_x035_rubato_i2c`, the scheduler path in SRAM, and its twin
`test_x035_rubato_i2c_flash`, against the CH32V006K8U6's unchanged
`twi_peer`: key `z` (195 verdicts) green on every run of both
([i2c.md](i2c.md) says how many of each letter). A Rubato pack of five objects runs `util/i2c_bus.hpp`'s
arbiter over the engined `I2cHost<1>` ([i2c.md](i2c.md)) with three
requesters of three priorities, under the same TIM2 pressure and the same
seven passes as the link suite; the target-free half is the I2C suites'
shared body, `avrxt/src/apps/rubato_i2c.hpp`, and the waits and passes
both bodies share are `avrxt/src/apps/rubato_passes.hpp`. Every tenure
shape on both of the host's paths, both speeds, the vocabulary, the
commanded stretch and the arbiter's ordering held under every pressure as
in the control, and a preemption inside the host's `start()` is a later
START, never a stretch (the findings are [i2c.md](i2c.md)'s, "The stack
under the preemptive kernel"). Two of them are this kernel's:

- **The tenure cycle locks to the pressure's period, at a phase the
  placement moves.** Under 97 us x 3000, 400 writes landed TIM2's
  interrupt inside `start()` 140 to 213 times with the scheduler path in
  SRAM - five to eight times what independent ticks would give - and never
  with the path in the flash, `start()` itself in the flash in both images.
- **Past this core's capacity the stack grows a level a tick.** A
  held-wire flood with a probe at every tick of 97 us x 3000 asks more
  than the 4656 cycles of a period hold (a rejected probe's chain costs
  some 1600 cycles over Pressure's 3000). The arbiter's backlog never drains, the time events -
  walked at the base of the pack - are starved, and every tick that lands
  between two of the scheduler's dispatches, the ceiling back at the
  level's base, opens a new level that serves the same objects as the one
  it interrupts and returns only when all their queues are empty: one
  level of 192 bytes a tick, never unwound, until the stack meets the
  statics (26 levels read in a dump, each saved return address inside the
  scheduler's loop). Under the cooperative kernel no dispatch nests
  over another, so the same load would starve the lower objects at the
  stack's one level (not run: the suite is the preemptive kernel's). The
  suite's flood is paced by the pack's lowest object instead (a probe at
  a tick only when the Console has turned since the last), which keeps it
  under what the core serves: the letter runs inside `z`, the stack at
  its usual depth ([i2c.md](i2c.md)).

The images: with the path in SRAM 34736 bytes of the 62 KB (55 %) and
4380 of RAM (21 %); with the path in the flash 34524 and 2080.

## A handler in SRAM

**What the silicon does.** At 48 MHz the code flash answers behind two
wait states (RM 20.3.1), and a taken branch is a fetch the core waits
for. The SPI suite's bare transmit loop - a loop of the suite's own, not
the host's - takes 20.8 cycles a frame at /2 from the flash and 16.7
from SRAM, the wire's 16 (measured, [spi.md](spi.md), "The fetch"); the
host's own transmit-only loop, a function of seven instructions a frame,
takes 24.0 from the flash and 15.9 from SRAM ("What it buys, measured").
The vector table holds absolute addresses (mtvec's MODE1, RM 7.5.3.2),
so a handler placed in SRAM is entered from its vector directly, and a
call between SRAM and the flash is an `auipc` and a `jalr`, which reach
2 GB either side of the caller: no veneer either way - no stub the
linker inserts to reach a target a direct call cannot.

**The option and its spelling: opt-in, per driver.** The series' section
layout places the input sections `.fast_text` and `.fast_text.*` in
`.data`, after the scheduler path where an image places it
([../../ch32x035/ld/sections.ld](../../ch32x035/ld/sections.ld)), so the
crt's copy of `.data` moves them into SRAM before `main()`, and the
platform says so (`Ch32x035Platform::places_fast_text`). A driver of
this stratum takes a placement, util/placement.hpp's `InFlash` (the
default) or `InRam`, as the last template parameter of `SpiHost` and
`I2cHost`, and of `Uart` after its `UartOptions`. An excerpt from
`test_x035_usart`, with `P` the platform: USART4 with a 512-byte receive
ring and a 256-byte transmit ring, the default frame, a DMA engine on
channel 1 for transmit and on channel 8 for receive, the default pins
(remap 0) and options, and its hot paths in SRAM:

```cpp
using Link = brio::Uart<4, P, 512, 256, brio::UartFormat{}, brio::DmaTxEngine<1>,
                        brio::DmaRxEngine<8>, 0, brio::UartOptions{}, brio::InRam>;
```

Each driver writes its hot body once and runs it through
`Hot<Placement>::run`
([../../brio/util/placement.hpp](../../brio/util/placement.hpp)), a
trampoline - a function whose only work is to run the body it is given
where the placement says. Under `InFlash` the body is inline where it is
called, in the vector that binds it, as if there were no trampoline;
under `InRam` each path is one out-of-line instance in
`.fast_text.brio`, flattened (every call inside it inlined, so the path
calls nothing), and a call from the vector. Only the instances an image
names exist: an image that opts into nothing has no driver code in SRAM
(a preemptive image's scheduler path is placed by its own line, "The
scheduler's place"). `InFlash` costs nothing over a body written inline
in its vector: a build of the project's images with every hot body
written so linked byte-identical images for the clock, pin, platform and
timer suites, and changed the DMA suite only by what `InFlash`'s
flattening inlines into the vector - the I2C's STOPF clear, the USART's
consume and block start, three calls out of line in that build and 100
cycles of the USART completion's cost with them; the byte-identity gate
measured it once, against a tree that had no placement, and no image of
the tree reproduces the comparison. A third tag, or `InRam` on a
platform that does not place `.fast_text`, does not compile
(`test/family_ch32x035/neg/`).

A program places a function of its own - the vector it binds, above all -
with one attribute, `.fast_text`, which a vector carries on a
declaration before its binding (the binding's expansion begins with
`extern "C"`, where no attribute can stand in front of it, and gcc
merges the declaration's section into the definition). An excerpt, with
`Link` the USART4 alias of the previous excerpt:

```cpp
extern "C" [[gnu::section(".fast_text")]] void usart4_handler();
BRIO_CH32_VECTOR(usart4_handler) { (void)Link::isr(); }   // no line reader waits on the receive edge
```

`isr()` answers true when a byte arrives into an empty ring, the edge a
line reader (util/line_reader.hpp) posts on; a program with no such
object discards it, as here.

The section has two names, `.fast_text` for a program's plain function
and `.fast_text.brio` for the drivers' instances, because GCC refuses
one section name for a function in a COMDAT group and a plain one in the
same translation unit ("section type conflict", WCH's gcc 15.2). A
function defined in a header is emitted in such a group, and a local
clone of a grouped function - IPA-SRA's - leaves the group and meets the
same refusal among the grouped ones (measured: the refusal goes away
with `-fno-ipa-sra`, not with `-fno-ipa-cp`); so the trampoline's
instances name `.fast_text.brio` and are never cloned. A program's own
inline or template function, emitted in a group too, goes through
`Hot<InRam>::run` rather than the attribute. An illustration, a template
function of the program that sums a block in SRAM:

```cpp
template <class Sample>
uint32_t sum_of(const Sample* data, uint32_t n) {
    return brio::Hot<brio::InRam>::run([&] {
        uint32_t sum = 0;
        for (uint32_t i = 0; i < n; ++i) {
            sum += data[i];
        }
        return sum;
    });
}
```

Each instantiation of `sum_of` places one instance in `.fast_text.brio`.
The family fixtures compile a placed vector beside the `InRam` instances
in one unit (`test/family_ch32x035/spi.cpp`, `dma.cpp`, `i2c.cpp`).
`.ram_text`, which the scripts keep beside it, is the name of code that
must run from SRAM for correctness - nothing of this stratum's.

**Which placement buys what.** A driver's `InRam` moves what only the
driver can: its private paths (the SPI host's polled loops) and the
bodies it calls out of the vector, with the vector left in the flash. A
program that places the vector moves everything inline in it - every
`InFlash` body it binds - with no call at all. On a path whose body is
inline in the vector whole (the I2C event pump, a USART's per-byte
transport), placing the vector is the faster of the two, and placing
both buys nothing more: an `InRam` body called from a vector in SRAM
measured the same as the vector alone (79 and 95 cycles against 79 and
94, the table's I2C row), its bytes spent twice. On a path the driver
calls out of line or keeps private, `InRam` is the only way.

**What it buys, measured** at 48 MHz on the CH32X035F8U6, each pair one
suite with the path in the flash and in SRAM, nothing else moved:

| path | placed by | from the flash | from SRAM | SRAM bytes |
|---|---|---|---|---|
| the SPI pump, a frame at /2, behind the suite's vector (stamps, owner switch) | the driver (`InRam`) | 256.5 cycles | 224.5 | 302, 332 with engines |
| the SPI host's transmit-only loop, a frame at /2 (the wire 16) | the driver | 24.0, its function aligned to a word | 15.9 | 998 a host type, both loops |
| the SPI receive loop, a frame at /2 and /4 (the wire 16, 32) | the driver | 51.9, 52.0, aligned likewise | 28.9, 46.9 | (with the transmit-only loop's) |
| the USART transmit engine's completion, the DMA vector's body | the driver | 285 to 289 (324 at most) with the vector on a word; the line idle at a block boundary above 1.5 Mbaud | 129 to 132 (154 at most), gapless to 3 Mbaud | 356 |
| a USART's interrupt transport, a gapless run of 256 in | the driver, or the program's vector | whole to 2 Mbaud | to 2.4 Mbaud behind a vector in the flash; to 3 Mbaud with the vector placed | 518 an engined port's `isr()`; the suite's vector 2060 |
| the I2C event pump's entry, a 255-byte write and a 16-byte read | the driver, or the program's vector | 205, 253 (333 at most) | 140, 157 behind a vector in the flash (207 at most); 79, 94 with the vector placed | 968 for one plain host |
| the console's handler body (115200, `test_x035_spi`) | the program | 148 | 66 | 404 |
| the tick's handler body | the program | 65 | 29 | 100 |

The SRAM column of the last two rows was measured in an image that
placed the console's and the tick's vectors with `.fast_text`, which no
suite of the tree builds; its bytes were read in that image's map. The
handler bodies are stamped on the STK between their first and last
statement; the hardware prologue's entry and exit around them, 70 cycles
of the round trip's 96 to 100 with the vector in the flash (letter `e`:
entry 36, exit 34, a body of 26 between them), are outside them.

From the flash, code with taken branches costs what its address modulo
four makes it, the fetch's word: the same instructions at every offset
through 32 bytes took one of two costs, by the offset's second bit and
by nothing else - 52.0 or 72.0 cycles a frame for the SPI receive loop
at /4, 68 or 72 for a body of four taken forward branches - where
straight-line code and three-instruction loops did not move and SRAM
moved by a cycle at most (a scratch sweep, [spi.md](spi.md), "The
fetch"). gcc aligns a function to two bytes under -Os, so a function
that keeps a stated flash cost is `aligned(4)` - the SPI host's polled
loops, and every vector, through `BRIO_CH32_VECTOR` (pfic.hpp): a
vector's own code costs the same wherever the image puts it, two bytes
of padding at most, while a function it calls in the flash keeps a
stated cost only if it is aligned itself. The USART's completion took
276 to 283 cycles in a measurement image that put its vector half a word
off ([usart.md](usart.md)): the alignment buys a figure that stays, not
always a lower one.

**What it costs.** The bytes in the last column, of the part's 20 KB -
the load image stays in the flash beside them. A placed function exists
only from the crt's copy on, and the crt enters `main()` with interrupts
masked, so no vector reaches SRAM the copy has not written. A driver's
`InRam` body is a call from a vector in the flash: under the hardware
prologue (the default) the vector passes the address of the closure -
the object holding the body's captured variables - in one instruction
and calls; in an image built without it the vector saves the
caller-saved registers in software around the call. The SPI host's
hold-off, from which it decides how far to write ahead
([spi.md](spi.md), "The hold-off"), is a fact of the image: its default
is the one safe with every driver in the flash, and a program that
places its handlers states its own.

## Not covered yet

Driver gaps, each with its reason:

- **Reset as a type, the fault record and the two watchdogs** - the
  `reset.hpp` of the CH32V00x and the CH32V203/CH32V303 strata (`Reset`,
  `ResetReporter`, `fault_reset<P>()`, `Iwdg`, `Wwdg`): born with the
  watchdog chapter (RM ch. 5, 6), their first user. A reboot on purpose,
  a panic that survives into the next boot and a fault body that records
  its cause wait for it; the reset flags are `Rcc`'s verbs meanwhile
  ([clock.md](clock.md)).
- **A USB controller in the idle guard**: the series' USB controllers
  are bus masters with no driver here, so `idle()` asks the DMA
  controller alone; born with the first USB driver, which must join the
  question.
- **The PFIC's priority threshold, its four vector-table-free channels
  and any priority byte but the scheduler's** (RM 7.5.2.9, 7.5.2.12 to
  7.5.2.16, 7.5.2.37): no user; the nesting is the preemptive platform's
  alone, two levels, every peripheral line at the reset priority.
- **The physical memory protection unit** (RM 7.5.4): no user.
- **The core deadlock reset's switch, PFIC_SCTLR.RSTEN**: left at its
  reset value, the reset on. Which traps raise it is a measurement - an
  exception taken inside the fault handler, the reset flags read at the
  next boot - that belongs with the fault body of the reset item, its
  first user.
- **The debug module's freeze bits** (DBGMCU_CR, CSR 0x7C0, RM 23.2.1):
  no verb reads or writes them; no user, and on the CH32V203 a `csrw` to
  that CSR from the running program resets the part
  ([../ch32vx03/sleep.md](../ch32vx03/sleep.md)), untested here.
- **The post verb in SRAM**: under `scheduler = ram` `post<Ao, Ev>`
  stays in the flash, its `push()` and the request placed. Declined: the
  instance's name carries the receiver and the event and no platform, so
  no pattern can tell a preemptive image's posts from a Tenuto image's,
  and placing them would move every Tenuto image that keeps a post out
  of line. Placed by name in a measurement image, the suite's posts took
  a quiet post to 73 to 74 cycles where it takes 108 to 112.
- **A plain `wfi` with MIE clear on this core**: whether it wakes on a
  pending interrupt, which the manual's wording leaves open and which on
  the CH32V006 and the CH32V203C8 it does not; no user, since the hook
  never executes one. Letter `w`'s edge walk with that sequence would
  answer it.
- **The masked-order WFE on this core**: whether a WFE entered with MIE
  clear loses an edge on its entry cycle, as the CH32V006's does, and in
  which code layouts; no user, since the hook enters unmasked. Letter
  `w`'s walk in that order, over several layouts, would answer it.
- **The flash images' stack**: its high-water reads 1012 bytes in the
  flash image and 1028 in the flash image without the prologue, where the
  two SRAM images read 1000; the 12 and 28 bytes are not explained, and
  the deepest chain read in those images' listings would explain them.

Implemented but not bench-verified, each with what would measure it:

- **Rubato on the other six parts**: the platform, its request and the
  two vector bodies compile for all seven, with and without the hardware
  prologue and with the scheduler path in either place (the family
  fixture); the suites are built and run for the CH32X035F8U6 alone.
- **`in_handler()` on this silicon**: the read of PFIC_GISR's GACTSTA is
  the manual's, compiled for the seven parts; a letter that asks it from
  a vector, from the main loop and under a critical section, as the
  STM32F4's serial suite does
  ([../stm32f4/usart.md](../stm32f4/usart.md), letter h), would measure
  it.
- **The deep path, `idle_deep()`**: no verb of this stratum sets
  SLEEPDEEP, so the paused timebase, the unmask before the `wfi`, the
  absent consume and the one turn it may spend after a deep wake are
  unmeasured; the sleep site born with the chapter of the low-power
  modes (RM 2.3.3, 2.3.4) is its first user and its suite the
  measurement.
- **The InRam bodies without the hardware prologue at the current
  alignment**: measured only in an image whose vectors were not aligned
  to four; `test_x035_spi` letter `b` in an image built with
  `CH32X035_HPE=OFF` would measure it.
- **The idle hook's power**: `idle()` proven to sleep once per wake is
  not `idle()` proven to sleep cheaply - the current in Sleep is a
  measurement with an ammeter and the probe detached.
