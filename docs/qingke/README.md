# Core stratum: QingKe (`qingke/`)

`brio/qingke/` holds the code that every family built on WCH's QingKe
RISC-V cores shares: what the cores themselves define, written once. A
stratum is one directory of `brio/` that includes only what lies below
it; this one sits between the target-independent code and the three CH32
families, `ch32v00x/`, `ch32vx03/` and `ch32x035/`. A program never
includes its six files. It includes its family's header of the same name
(`ch32x035/pfic.hpp`, `ch32x035/platform.hpp` and the like), which
includes the family's device header, defines the values the core file
needs from that family - its facts, listed under "What stays per family" -
and then includes the core file. Five of the six have a family header of
that name (`schedule.hpp` on the two V4 families only); `stk.hpp` has
none and is reached through `ticker.hpp` and `delay.hpp`. That a program
includes the family's headers and never these is a convention of the
tree, not a check: nothing refuses `qingke/ticker.hpp` included after a
device header, and no program, test or driver outside the families does
it.

The cores are the QingKe V2C of the CH32V006 (RV32EC with the
multiply-only Zmmul) and the V2A of the CH32V003 (RV32EC), the
`ch32v00x/` stratum; the V4B of the CH32V203 and the V4F of the CH32V303
(RV32IMAC and RV32IMAFC, `ch32vx03/`); and the V4C of the CH32X035
(RV32IMAC, `ch32x035/`). They share one machine-mode programmer's model,
one interrupt controller (the PFIC) and one system counter (the STK),
whose width differs: 32 bits on the V2, 64 on the V4. The stratum runs
under each family's platform suite, `test_ch32_platform` on the
CH32V00x, `test_vx03_platform` and `test_vx03_rubato` on the CH32V203
and CH32V303, `test_x035_platform`, `test_x035_rubato` and
`test_x035_rubato_flash` on the CH32X035; each family's platform page
states what they measured
([../ch32v00x/platform.md](../ch32v00x/platform.md),
[../ch32vx03/platform.md](../ch32vx03/platform.md),
[../ch32x035/platform.md](../ch32x035/platform.md)).

## The six files

### `qingke/pfic.hpp`: interrupt control

`InterruptGuard` is the critical section every QingKe platform rides on:
one `csrrci` reads and clears mstatus.MIE, the global interrupt enable,
and the guard's end sets it again only if it was set, so guards nest.
Beside it are the global enable, disable and readback verbs; `Pfic`'s
per-line verbs over the PFIC's set and clear registers, where writing a
1 to a bit acts on that line and a 0 leaves the others alone, so no
read-modify-write and no guard is needed; and `Pfic::in_handler()`, the
PFIC_GISR register's GACTSTA bit, which says whether an interrupt
handler is executing.

The file binds vectors. `BRIO_CH32_VECTOR(name) { body }` is what a
program writes for an ordinary handler: a function under the crt's name
for the vector (the crt is the startup code each project's
`src/glue/startup_*.S` holds). The binding writes the handler's
attribute from the build. With the core's hardware prologue - the core
itself saving the caller-saved registers on a trap's entry and restoring
them on MRET, each project's `*_HPE` option, passed to the compiler as
`BRIO_CH32_HPE` - it is WCH's fast attribute, and gcc emits no save of
its own; without it, gcc's plain `interrupt`, and the handler saves what
it clobbers. On an image built with the F extension and the hardware
prologue both (the CH32V303's images as its project builds them), the
vector is instead a naked trampoline: a function with no compiler-made
prologue, two instructions that call an ordinary function holding the
body and then execute MRET. The reason is what the hardware prologue
does not save, an f-register: for an attributed handler that calls out,
gcc saves all twenty caller-saved f-registers, eighty bytes of stack and
forty memory operations on every entry, while an ordinary function saves
only what the ABI asks of a callee. `brio check ch32vx03` fails if a
trampoline's body or anything it reaches uses an f-register
(`cli/vector_guard.py`). Two siblings cover the cases the trampoline
does not: `BRIO_CH32_LEAF_VECTOR` binds a body that calls nothing and
touches no f-register as the attributed handler even under F, and
`BRIO_CH32_VECTOR_FLOAT` stores the f-registers and fcsr around a body
that does float work. Without F, or with the hardware prologue off, the
three macros are one. The preemptive kernel's two vectors take a fourth
binding of their own, `BRIO_CH32_SCHEDULER_VECTOR`, which
`qingke/schedule.hpp` defines.

**Every binding carries `no_icf`, because gcc's identical code folding
breaks interrupt handlers here**: it turns the second of two handlers
with one body into a handler that calls the first, which ends in MRET,
so the call never returns and the caller's frame stays on the stack - on
the CH32V303VCT6 the interrupted program resumed eighty bytes below its
frame and its next return jumped to a saved float (measured). Every
binding is also aligned to four bytes. gcc aligns a function to two
under -Os, and from the CH32X035's flash, which is fetched a word of
four bytes at a time, code with taken branches costs what its address
modulo four makes it
([../ch32x035/platform.md](../ch32x035/platform.md), "A handler in
SRAM"); aligned, a vector's own code costs the same wherever the image
puts it.

### `qingke/stk.hpp`: the counter's two widths

The STK, read from the family's register struct: the V2's 32-bit CNT and
CMP, the V4's 64-bit pairs CNTL/CNTH and CMPLR/CMPHR. `Stk64` is a
concept - a compile-time test on a type - that is true of the V4's
register struct; `ticker.hpp` and `delay.hpp` use it to pick their code.

### `qingke/ticker.hpp`: the kernel timebase

`BasicTicker<tps>` counts HCLK up to a compare of hz/tps - 1, where hz
is HCLK's rate as the program's clock type states it and tps the ticks
per second, and restarts from zero there in hardware. A rate that does
not divide 1000 is refused at compile time, so `millis()` is exact.
`cycles()` gives HCLK cycles since `init()` from any context: it reads
the tick count, the counter's pending flag, the counter, the flag and
the tick count again, in `util/cycle_count.hpp`'s order, and repeats
until the five belong to one instant, so a read that falls between a
restart and the handler's count is not torn. `pause()` and `resume()`
hold the tick off across a deep sleep, `advance()` serves a sleep site
that stopped the counter (it hands the slept span back to kernel time),
and `rebase()` a clock that changes rate at run time (a `DynamicClock`).
Each family's `ticker.hpp` names the project-wide `Ticker`, 1000 Hz on
all three.

### `qingke/delay.hpp`: the microsecond busy-wait

`delay_us` waits on the STK's counter at least the time asked and never
less. It answers with a `[[nodiscard]]` bool and spends no time when it
refuses: a request of one tick period or more (1000 us at the 1000 Hz
`Ticker`), a request of 65536 us or more (the limit that keeps its
32-bit product from wrapping), any request while the counter is stopped,
and any request when the clock gives zero cycles a microsecond. It
divides nothing at wait time: the QingKe V2 has no divide instruction,
and the V4 runs the same multiply-only code. Under a clock that changes
rate it reads its factor from a per-rate table built at compile time.

### `qingke/schedule.hpp`: the preemptive kernel's request

The preemptive kernel, Rubato ([design/kernel.md](../design/kernel.md),
"Rubato"), runs on the three V4 families. To run a higher-priority object,
it pends the scheduling request, the software interrupt, which
`scheduler_init()` puts one preemption level below every peripheral
line, turning the cores' interrupt nesting on for that purpose alone.
The request's body, `scheduler_trap<K>()` (K is the program's kernel
type), does not run the scheduler: it makes the trap return into
`scheduler_thread<K>()`, which runs in thread mode - machine mode
outside any trap, the interrupted code's registers saved on the stack -
and leaves through `ecall`, the machine-mode environment call, whose
body `scheduler_return<K>()` resumes the interrupted code. On the V4F the
entry also saves the twenty caller-saved f-registers and fcsr when
mstatus.FS says the interrupted code has written one. The two vectors,
the software interrupt's (`software_handler`) and the environment call's
(`ecall_m_handler`), are bound with `BRIO_CH32_SCHEDULER_VECTOR`.

The two bodies keep the kernel's entry and exit (`kernel/platform.hpp`).
These cores do not clear MIE when they take an interrupt or an
exception (the V4 manual 2.2), and the peripheral lines preempt the
software interrupt's handler, so each body clears MIE as its first
instruction. The trap's body then reads the kernel's ceiling into a
byte of the block it leaves on the stack and stores 0 in its place; the
entry hands that byte, `base`, to `activate(base)` and writes the count
of quiet posts it returns into the next byte; the environment call's
body puts `base` back, compares the count and, when it moved, pends the
software interrupt again, which is taken over the interrupted code once
its MRET has popped the block. One moment stays open, the trap's first
instruction, at which a peripheral handler can still preempt it: a
request made there is taken at the entry, as a run that finds the
ceiling at 0 and serves nothing (kernel/rubato.hpp, "The stack"). By the
release listings, with the hardware prologue and the linker's
relaxation turning each address's `lui` and access into one access
relative to gp: the trap's body thirteen instructions where it was
nine, the environment call's twelve on the path that finds the count
still and sixteen on the path that pends, where it was six, the entry
forty where it was thirty-eight (ninety-five and ninety-three with the
V4F's f-registers). The body is a template on the kernel,
`scheduler_return<K>()`, because no register can carry the addresses
into it: the hardware prologue's pop at its MRET gives the interrupted
code back every register the environment call found.

Rubato is not available on the CH32V00x, which does not include this
file. Its V2 core's hardware prologue pushes the interrupted code's
registers to the user stack instead of an internal one, so the MRET into
the scheduler's entry would pop that frame from below the block the
request leaves on the stack; the shape could carry over with that
frame's layout, and nothing of it has been measured there.

### `qingke/platform.hpp`: the platform type

`QingkePlatform<Chip, TB>` realizes the kernel's Platform concept once
for all three families: the critical section, the idle hook, the
timebase, `ebreak` for a panic, the panic record in `.noinit` (a section
the crt neither loads nor zeroes), and `stack_untouched()`.

The idle hook sleeps as a WFE entered with MIE set, on every core but
the CH32V003. PFIC_SCTLR.WFITOWFE turns the `wfi` into a wait for an
event, and SEVONPEND makes every interrupt that turns pending an event,
latched - the event latch: a WFE executed after it returns at once. A
plain `wfi` would not do: on these cores a `wfi` with MIE clear sleeps
past every pending interrupt (measured on the CH32V006 and the
CH32V203C8; unmeasured on the CH32X035, whose manual does not promise
otherwise). Sleeping masked would not do either: a masked WFE wakes only
on a new pending edge, and the CH32V006's misses one that arrives on its
entry cycle (measured; no such cycle found on the CH32V203C8 in
twenty-four code layouts; unmeasured on the CH32X035). Entered with MIE
set it loses no wake, since the kernel calls the hook masked and the
hook unmasks before the `wfi`: the CH32X035's platform page sets out the
reasoning ([../ch32x035/platform.md](../ch32x035/platform.md), "Types
and verbs").

After the wake the hook consumes the event latch the wake left (a
SETEVENT store and a second `wfi`, which returns at once), so the kernel
loop turns once per interrupt: 101 turns over 100 quiet ticks (the
window opens and closes on a tick, so a turn straddling one end counts)
on the CH32V006, the CH32V203C8 and the CH32X035F8U6. Without the
consume the waking edge, latched, ends every second `idle()` at once:
200 turns over 100 ticks on the CH32V006 and the CH32V203C8, measured by
their platform suites; on the CH32X035F8U6 the same, measured once in a
probe image with the consume removed, which no image of the tree
reproduces.

With SLEEPDEEP armed the hook takes the deep path, which pauses the tick
across the sleep and clears its pending bit first, since a tick merely
pending would end the WFE before the sleep began. That path is exercised
by the sleep suites of the two families that have a sleep site
([../ch32v00x/sleep.md](../ch32v00x/sleep.md),
[../ch32vx03/sleep.md](../ch32vx03/sleep.md)); the CH32X035 has none,
and there it is unmeasured.

On the CH32V003 the hook sleeps masked and from SRAM instead, because an
edge arriving in a window four cycles wide at the V2A core's `wfi` entry
leaves the core in a state only a reset ends, a wedge. In the masked
shape fetched from SRAM, with a spin of nineteen turns after the `wfi`
(the vendor's own CH32V003 sequence, read as the oracle), none of 1600
tries wedged. That shape has the masked WFE's own hole, an edge on the
entry cycle not seen, which the hook closes by emptying the latch first,
asking the PFIC whether an enabled line is pending, and keeping the
tick's edge off the entry
([../ch32v00x/platform.md](../ch32v00x/platform.md), "On the
CH32V003F4P6").

## What stays per family

Each family's header of the same name keeps what is its own beside the
include:

- the device header: `pfic()`, `PficRegs`, `pfic_sctlr()` and the
  `sctlr_*` constants, `stk()`, `StkRegs`, the `stk_*` bits and `Irq`
  with its `software` and `systick` lines - the same names in the three
  families, read by the core files as they stand;
- the chip traits type, the first template argument of `QingkePlatform`,
  whose members the `QingkeChip` concept checks; the table that closes
  this section gives each family's answers;
- `scheduling_request_posted`, on the two V4 families: whether the store
  that pends the software interrupt takes effect late, so that the
  instructions after it run before the trap. If they did, the code after
  a post would run before the higher-priority object it posted to; where
  the fact is true the request ends with a `fence.i`, which closes that.
  It is true on the V4B and the V4F, where three instructions retire
  behind the store unless a `fence.i` follows (measured on the
  CH32V203C8T6 and the CH32V303VCT6). On the V4C it follows where the
  image runs the request, which the CH32X035 project's `// build:
  scheduler` line chooses: false from the flash, where the trap is taken
  at the instruction after the store, true from SRAM, where three
  instructions retire as on the V4B (both measured on the CH32X035F8U6);
- `scheduler_vector_table_init()`, the family's step of
  `scheduler_init()`: on the CH32X035, with the scheduler path in SRAM,
  the vector table copied into SRAM and mtvec pointed at the copy; on
  the CH32V203 and the CH32V303 nothing;
- the platform type's public name, a struct over `QingkePlatform`
  (`Ch32v00xPlatform<TB>`, `Ch32vx03Platform<TB>`,
  `Ch32x035Platform<TB>`), which adds what is the family's alone:
  `places_fast_text` on the CH32V00x and the CH32X035, whose linker
  scripts place `.fast_text` (code the crt copies into SRAM before
  `main()`), and on the CH32V203 and the CH32V303
  `bus_masters_active()`, the count of working bus masters published for
  a program or a suite to read, the number behind the traits' yes or no;
  and on the two V4 families the preemptive kernel's platforms over it,
  `Ch32vx03RubatoPlatform<TB>` and `Ch32x035RubatoPlatform<TB>`;
- the project's options (the hardware prologue, the ISA and ABI), the
  crt, the clock, the pins and every peripheral.

The chip traits answer `bus_masters_working()`, the question the idle
hook asks before it sleeps: whether a bus master other than the core is
working that the sleep would stall. The CH32V00x's traits also carry
`sleep_entry_from_sram`, the CH32V003's masked entry, with the two verbs
`sram_deep_events_on()` and `sram_deep_events_off()`, which make the
EXTI lines wake events while the core is in Standby and give them back
after:

| family | how `bus_masters_working()` is answered |
|---|---|
| CH32V203, CH32V303 | the count of working DMA channels and USB controllers the drivers keep (`ch32vx03/bus_activity.hpp`) |
| CH32X035 | the DMA controller's own registers (`Dma::any_working()`) |
| CH32V003 | the DMA controller's own registers: its DMA stops in Sleep |
| CH32V006 | always false: its DMA keeps working in Sleep |

## Why the stratum exists

A core stratum holds what the families on one core share, under the rule
of [design/overview.md](../design/overview.md), "A core stratum sits
between util/ and the families that share a core". The three QingKe
families run the same programmer's model, the same PFIC and the same
STK, so the code that touches only those - the guard and the bindings,
the ticker and the busy-wait, the request and its two bodies - is one
file each here. Unlike `cortexm/`, this stratum also holds the platform
type's body: the three families' platforms are one type,
`QingkePlatform`, and differ only in the facts listed under "What stays
per family". Its one idle store serves every core: it clears SLEEPONEXIT
on each, since with that bit set the waking handler's MRET would put the
core back to sleep (the V4 manual 6.2, the V2 manual 5.2), and the
CH32V003's sleep-entry measurements are with that clear.

The vectors' alignment to four was measured on each family in two images
of its platform suite that differ in the alignment alone, ten runs each:
the software interrupt's round trip read 78 cycles aligned against 81
unaligned on the CH32V006K8U6, 52 against 56 on the CH32V203C8T6, and 52
to 62 against 53 to 63 on the CH32V303VCT6. There the preemptive
kernel's suite moved the other way, in two diagnostic builds (not kept)
that differ in the alignment alone and whose scheduler calls each
queue's `empty()` out of line where the tree's forces it inline: road 1
(an interrupt handler's post preempting the running object) read 144
cycles aligned against 140 to 141, and a road-2 post (one object's post
to a higher-priority object, which runs inside it) 344 to 347 against
338 to 342. Those figures are measured and not explained beyond the
bodies behind the vectors, whose branch targets moved with them
(`qingke/pfic.hpp`); the pair is not measured on the scheduler the tree
builds.

## What lives here, and what does not

Here: code that reads and writes the machine-mode CSRs (mstatus, mepc,
fcsr, and INTSYSCR at 0x804, WCH's interrupt system control register,
whose bits enable the hardware prologue and interrupt nesting), the PFIC
and the STK through the family's device header, and nothing of a
peripheral. The measured facts of the three families' cores stand in
these files' comments with the chip each was measured on, and in each
family's platform page with the suite.

Not here, by design: the device header and the part tables, the crt
(vector names are the families' own crts'), the clock, the pins, every
peripheral, the sleep sites (`ch32v00x/sleep.hpp`,
`ch32vx03/sleep.hpp`), the reset and watchdog verbs, and the bus-master
count the CH32V203's and CH32V303's drivers keep. Two features of the
cores have no user in brio and no code: the priority threshold, and the
vector-table-free channels (VTF, the QingKe V4 manual 3.5: an interrupt
line whose handler's address the PFIC holds in a register and enters
without reading the vector table).

## The include-order contract

A qingke header does not include a device header, since it cannot know
which. **A qingke header compiled before the family's device header does
not compile**: `qingke/pfic.hpp` and `qingke/stk.hpp` open with a
`static_assert` on the device header's `PficRegs` or `StkRegs` whose
message says so, and the other four include one of the two first.
`qingke/schedule.hpp` also needs the family's facts defined before it is
included: `scheduling_request_posted` is checked by a `static_assert`
with a message of its own, and a missing `scheduler_vector_table_init()`
is an ordinary undeclared-name error where `scheduler_init()` calls it.
`qingke/platform.hpp` needs no fact before its include: the chip traits
are the type's first template argument, and the `QingkeChip` concept
checks them, by a `static_assert` naming the members, where the family's
platform type instantiates it. Each family's header is exactly that: the
device include, the facts, then the core file, then what is the family's
own.

## Editor

`brio/qingke/.clangd` parses these files against the CH32X035 project's
database, a project that compiles all six, with `-march` narrowed to the
standard subset clangd's clang knows; every project that includes them
compiles them with its own flags at build time.
