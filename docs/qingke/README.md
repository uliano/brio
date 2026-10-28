# Core stratum: QingKe (`qingke/`)

`brio/qingke/` is one of the directories under `brio/` that is neither
target-independent nor a target: what WCH designed into its QingKe
RISC-V cores and every family built on them carries unchanged, written
once and included by each QingKe family after its own device header.
This page is for a reader of a CH32 family's headers who meets an
include from `qingke/` and wants to know what it holds, what stays per
family, which cores it has been proven on and the one rule for including
it. The stratum has six files; a program never includes them directly:
it includes its family's file of the same name (`ch32x035/pfic.hpp` and
the like), which is the family's device header, the family's facts and
then the core file.

The cores it covers are the QingKe V2C of the CH32V006 and the V2A of
the CH32V003 (RV32EC, the `ch32v00x/` stratum), the V4B of the CH32V203
and the V4F of the CH32V303 (RV32IMAC and RV32IMAFC, `ch32vx03/`) and
the V4C of the CH32X035 (RV32IMAC, `ch32x035/`): one machine-mode
programmer's model, one interrupt controller (the PFIC) and one system
counter (the STK) in two widths.

## The six files

- **`qingke/pfic.hpp`**: interrupt control. `InterruptGuard` (the
  critical section every QingKe platform rides on, one `csrrci` on
  mstatus.MIE), the global enable, disable and readback verbs, `Pfic`'s
  per-line verbs over the write-one banks and `Pfic::in_handler()`
  (PFIC_GISR's GACTSTA), and the vector bindings `BRIO_CH32_VECTOR`,
  `BRIO_CH32_LEAF_VECTOR` and `BRIO_CH32_VECTOR_FLOAT`. The binding
  chooses the handler's shape from the build: WCH's fast attribute under
  the core's hardware prologue (each project's `*_HPE` option, passed as
  `BRIO_CH32_HPE`) or gcc's plain one without it, and on an image built
  with the F extension (the CH32V303's) a naked trampoline to an
  ordinary function, whose body `brio check ch32vx03` holds to touching
  no f-register (`cli/vector_guard.py`). Every binding carries `no_icf`
  (gcc folds two handlers of one body into a call of the first, which
  ends in MRET - measured on the CH32V303VCT6) and is aligned to four
  (below).
- **`qingke/stk.hpp`**: the STK's two widths, read from the family's
  register struct: the V2's 32-bit CNT and CMP, the V4's 64-bit pairs
  CNTL/CNTH and CMPLR/CMPHR. One concept, `Stk64`, which the next two
  files ask.
- **`qingke/ticker.hpp`**: the kernel timebase on the STK. `BasicTicker`
  counts HCLK up to a compare of hz/tps - 1 with the hardware reload, at
  a rate that divides 1000 so `millis()` is exact, its `cycles()`
  composed from five reads in `util/cycle_count.hpp`'s order, `pause()`
  and `resume()` for a deep sleep, `advance()` for a timed sleep site,
  `rebase()` for a dynamic clock. Each family's `ticker.hpp` names the
  project-wide `Ticker`, 1000 Hz on all three.
- **`qingke/delay.hpp`**: `delay_us`, the microsecond busy-wait on the
  STK's counter: "at least" and never early, capped below one tick
  period (a request of a tick period or more refused, no time spent),
  no division at wait time (the QingKe V2 has no divide instruction; the V4 could divide,
  and does not need to), a per-rate table under a dynamic clock.
- **`qingke/schedule.hpp`**: the scheduling request of the preemptive
  kernel, Rubato ([design/kernel.md](../design/kernel.md), "Rubato"),
  on the QingKe V4, and the place its scheduler runs. The request is the
  software interrupt, put one preemption level below every peripheral
  line by `scheduler_init()`, which turns the cores' nesting on for that
  purpose alone; its body returns into `scheduler_thread<K>()` in thread
  mode, which leaves through the machine-mode environment call. On the
  V4F the entry saves the twenty caller-saved f-registers and fcsr when
  mstatus.FS is Dirty. The two vectors are bound with
  `BRIO_CH32_SCHEDULER_VECTOR`. The CH32V00x does not include this file:
  its V2's hardware prologue pushes to the user stack, and nothing of
  the shape has been measured there.
- **`qingke/platform.hpp`**: `QingkePlatform<Chip, TB>`, the kernel's
  Platform concept realized once: the critical section, the idle hook,
  the timebase, `ebreak` for a panic, the breadcrumb in `.noinit`, and
  `stack_untouched()`. The idle hook is a WFE entered with MIE set -
  these cores' WFI does not wake on a pending interrupt with MIE clear,
  measured on the CH32V006 and the CH32V203C8, and a masked WFE loses an
  edge on its entry cycle, measured on the CH32V006 - and it consumes
  the event latch its wake leaves, so the kernel loop turns once per
  interrupt (200 turns over 100 ticks without the consume, 101 with it,
  on the CH32V006 and the CH32V203C8; one a tick on the CH32X035F8U6).
  With SLEEPDEEP armed it pauses the tick across the deep sleep. On the
  CH32V003 it sleeps masked from SRAM instead, the shape that ends the
  V2A's sleep-entry wedge (an edge in a four-cycle window at the wfi's
  entry leaving the core in a state only a reset ends, measured, 0 of
  1600 tries in that shape).

## What stays per family

Each family's file of the same name keeps what is its own, beside the
include:

- the device header: `pfic()`, `PficRegs`, `pfic_sctlr()` and the
  `sctlr_*` constants, `stk()`, `StkRegs`, the `stk_*` bits and `Irq`
  with its `software` and `systick` lines - the same names in the three
  families, read by the core files as they stand;
- the chip traits type, the first argument of `QingkePlatform`, whose
  members the `QingkeChip` concept checks: `bus_masters_working()`, the
  question the idle hook asks before it sleeps (the CH32V203's count of
  working DMA channels and USB controllers, `ch32vx03/bus_activity.hpp`;
  the CH32X035's DMA controller's own registers; the CH32V003's DMA
  controller, whose V2A Sleep stalls it; on the CH32V006, whose Sleep
  the DMA works through, a constant false), and on the CH32V00x
  `sleep_entry_from_sram` (the CH32V003's part table), the two verbs
  that make the EXTI lines events for a Standby's span, and
  `idle_leaves_sleeponexit`, the one difference of interpretation left
  between the families: the V4 families' idle store clears SLEEPONEXIT,
  the CH32V00x's leaves it as found, because clearing it there moves the
  CH32V003's sleep entry by an instruction and that entry's timing is
  what its wedge was measured against;
- `scheduling_request_posted`, the one fact of a V4 core the request
  asks: true on the V4B and the V4F, where three instructions retire
  behind the pending store unless a `fence.i` closes them (measured on
  the CH32V203C8T6 and the CH32V303VCT6), false on the V4C, which takes
  the trap at the instruction after the store (measured on the
  CH32X035F8U6);
- the platform type's public name, a struct over `QingkePlatform`
  (`Ch32v00xPlatform<TB>`, `Ch32vx03Platform<TB>`,
  `Ch32x035Platform<TB>`), which adds what is the family's alone:
  `places_fast_text` on the CH32V00x and the CH32X035, whose linker
  scripts place `.fast_text`, and the CH32V203's `bus_masters_active()`;
  and the two V4 families' Rubato platforms over their platform type;
- the project's options (the hardware prologue, the ISA and ABI), the
  crt, the clock, the pins and every peripheral.

## Why the stratum exists

brio's naming rule factors a core stratum at the second family that
carries the core, never earlier. The QingKe families came one after the
other, each written from its own chapters, and were folded at the
third: with comments stripped, `ticker.hpp` and `delay.hpp`
were the same code on the CH32V203 and the CH32X035, the CH32V00x's
differing by the STK's register names alone; `pfic.hpp` held the same
verbs everywhere, the vector binding spelled two ways; the platforms
held one shape whose differences were hardware (the CH32V003's sleep
entry, the bus masters a sleep stalls, the vectors' alignment) or
interpretation (the binding's name, how the bus question is asked); and
`schedule.hpp` was the same on the two V4 families but for the alignment
and the request's fence. The interpretation was harmonized first - one
binding, one bus question - and the vectors' alignment, measured on the
CH32V006K8U6 and the CH32V203C8T6 to cost no cycle in the platform
suites (a round trip of 78 cycles against 81 and 52 against 56), given
to all three - where it moved the CH32V303VCT6's preemptive-kernel
figures by three to five cycles, measured and not explained beyond the
bodies behind the vectors moving with them (qingke/pfic.hpp has the
figures). Every release
image of the three families was then byte-identical before and after
the fold. Unlike `cortexm/`, this stratum holds the platform type's
body: the three platforms differed in the hooks above and nowhere else.

## What lives here, and what does not

Here: code that reads the machine-mode CSRs (mstatus, mepc, INTSYSCR at
0x804, fcsr), the PFIC and the STK through the family's device header,
and nothing of a peripheral. The measured facts of the three families'
cores stand in these files' comments with the chip each was measured on.

Not here, by design: the device header and the part tables, the crt
(vector names are the families' own crts'), the clock, the pins, every
peripheral, the sleep sites (`ch32v00x/sleep.hpp`, `ch32vx03/sleep.hpp`),
the reset and watchdog verbs, the bus-master count the CH32V203's
drivers keep, and the free vectored entries and priority threshold the
cores offer and no program here uses.

## The include-order contract

A qingke header does not include a device header, since it cannot know
which, and it refuses to compile before one: its first declaration is a
`static_assert` on the device header's `PficRegs` or `StkRegs`, whose
message says so. `qingke/schedule.hpp` and `qingke/platform.hpp` also
need the family's facts above defined before the include. Each family's
file is exactly that: the device include, the facts, then the core file,
then what is the family's own. Apps and family drivers keep including
the family's headers, never these directly.

## Editor

`brio/qingke/.clangd` parses these files against the CH32X035 project's
database, a project that compiles all six, with the march narrowed to
the standard subset clangd's clang knows; every project that includes
them compiles them with its own flags at build time.
