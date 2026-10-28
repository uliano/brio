# Platform (CH32V00x)

This document is for a programmer putting a brio program on a CH32V006
or a CH32V003. It describes what the kernel stands on here, the
platform. A brio kernel asks the chip for a handful of things, stated
by the kernel's `Platform` concept (a concept is a compiler-checked set
of requirements on a type;
[design/kernel.md, "Platform: what the machine provides"](../design/kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp)):
a way to mask interrupts for a critical section, a way to sleep until
the next interrupt without losing one that arrives at the wrong moment,
a source of time, a breakpoint for a panic, and a record a panic leaves
for the next boot. `Ch32v00xPlatform<TB>` is this family's answer, on
the QingKe V2C core of the CH32V006 and the QingKe V2A of the CH32V003,
and `TB` is its timebase: the `Ticker` on the core's system counter
(the STK), a 1000 Hz interrupt. The page has two halves: the running
half (the critical section, the idle hook, the STK timebase, the
microsecond busy-wait, the interrupt round trip) and the failing half
(which reset happened, how to cause one, the two watchdogs, and the
record a crash leaves for the next boot).

The smallest use is the glue every program of this project writes, as
"How to use it" below has it (`SysClock` and its tag `clock` - an empty
object of that type, which a driver's `init` takes to learn the rate -
are the clock task's, the program's clock type, whose `init()` sets the
chip's rate, declared as in [clock.md](clock.md)'s smallest use; the C++
notation, `using`, the angle brackets, `brio::`, is explained in
[the C++ used by blink](../learn/cpp-notation.md)):

```cpp
using P = brio::Ch32v00xPlatform<>;

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_VECTOR(fault_handler) { brio::fault_reset<P>(); }

int main() {
    const auto record = brio::take_panic_record<P>();   // first, and once
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Tenuto<P, ...>::run();
}
```

Line by line: `P` names the platform with its default timebase, the
empty angle brackets meaning that default, the `Ticker`. The two
`extern "C"` functions are interrupt handlers, as in C, under the
startup code's names for the STK's vector and for the fault entry (the
startup code, the crt, is the assembly file
`ch32v00x/src/glue/startup_ch32v00x.S` that runs before `main()`; it
names every vector and points each one the program does not bind at a
spin, a loop on itself). `fault_reset<P>()` is the fault entry's body:
it writes a `kernel_fault` record (never over one that already stands)
and resets the chip, its optional argument a context byte the record
carries (0 when omitted). An `ebreak` with no debugger attached lands
there; a program that binds no fault handler, as `blink`, leaves that
entry on the crt's spin.

In `main()`, `record` is the record of a panic that ended the previous
run, if any, for the program to report before anything else
(kernel/panic.hpp): the call returns a valid record once and clears it,
so it is taken first, before anything can panic again, and once; a
program that reports nothing may discard it. `Ticker::init()` starts
the STK at the tick rate and answers false only for a clock too slow to
divide. The `...` of the last line stands for the program's active
objects in priority order (an active object is a component of a brio
program with its own event queue, served one event at a time by the
kernel, [docs/learn/introduction.md](../learn/introduction.md)), and
`Tenuto` is that kernel, the loop that serves their queues and calls
the platform's `idle()` when they are empty
([design/kernel.md, "Scheduler"](../design/kernel.md#8-scheduler-kerneltenutohpp)).
`blink`, the maintained example, is this glue with no fault binding and
no record taken.

Three things decide whether that works. The clock task must have run
first (`SysClock::init()`, [clock.md](clock.md)): the ticker computes
its compare value from the rate the clock type claims, and a ticker
initialized on a chip still at its 8 MHz reset rate with a 48 MHz
clock type ticks six times too slowly until the clock task runs (the
compare is written once, from the claimed rate, so it is right once the
chip runs at that rate); nothing refuses it. The STK vector must be
bound to `Ticker::tick()` by the program, as above,
with `BRIO_CH32_VECTOR` under the crt's name for it
(`systick_handler`): the kernel binds no vector, and a
program that forgets it compiles, but the startup code points that
vector at its spin for unbound vectors and `Ticker::init()` enables the
line, so the first tick after `enable_interrupts()` enters the spin and
the program stops there. `BRIO_CH32_VECTOR` is the one spelling of a
vector binding on this target, and the handler attribute it writes
follows the project's
`CH32V00X_HPE` option, a CMake option of the `ch32v00x/` project chosen
at configure (`-DCH32V00X_HPE=OFF` turns it off), on by default: with
it the crt sets INTSYSCR.HWSTKEN and the attribute is
WCH's fast attribute, the core's hardware prologue saving the registers
a handler clobbers; without it the attribute is gcc's plain `interrupt`
and the handler saves them itself. One switch for the whole image,
because a fast handler in an image whose crt did not set the bit
corrupts the program it interrupted. And on the CH32V003 the idle hook
refuses to sleep while a DMA channel works, since that core's Sleep
stops the DMA controller ([dma.md](dma.md)): a program with a channel
always at work there never sleeps.

The rest of this page, in order: the documents of record; what the
silicon does; the types and verbs; how to use it; the bench findings,
measured to the cycle on the CH32V006K8U6; what the CH32V003F4P6
changes, its sleep entry above all; "A handler in SRAM", the opt-in
placement of a driver's or a program's hot path; and what is not
covered.

## Documents of record

The CH32V00X reference manual V1.5 (ch. 3 RCC for
the reset flags, ch. 6 for the PFIC and the STK), the QingKe V2
microprocessor manual V1.3 (2.2 for what interrupt entry does to
mstatus, 3.4 for the hardware prologue/epilogue, 5.2 for what wakes a
WFI). No errata sheet exists for this family that the desk could find;
the findings below are the entries this document keeps in its place.
The bench suites below are named with their letters: a suite's letter
is one command key of its console and the group of verdicts it runs,
`z` the key that runs the suite's set of letters (a letter outside it,
such as the platform suite's resetting `i`, runs by its own key).

## What the silicon does

- **Interrupt entry does not clear MIE** (QingKe V2 manual 2.2), so a
  handler runs with the global enable still set; the PFIC decides
  whether another line may preempt it, and with INTSYSCR.INESTEN off
  it holds the next line until MRET.
- **A WFI sleep ends only when an interrupt is responded** (5.2) -
  which needs MIE set. Sleeping with MIE clear over a pending interrupt
  is a sleep nobody ends.
- **The core can sleep as a WFE instead**: PFIC_SCTLR.WFITOWFE turns
  `wfi` into a wait-for-event, and SEVONPEND makes every interrupt
  entering the pending state an event, latched, so a WFE after the
  event returns at once (3.1's SCTLR). The manual's "the subsequent
  WFI" reads as one-shot; on the CH32V006 WFITOWFE reads back set
  after the `wfi` and the next `wfi` is a WFE again (measured).
- **A WFE wakes on an enabled interrupt or on a pending edge** (5.2,
  the WFE list): item (2), an enabled interrupt "generated", with the
  handler run first - which takes MIE set - and item (3), SEVONPEND's
  "new interrupt pending signal (the previously generated pending
  signal does not take effect)" for any line, enabled or not. A
  sleep entered with MIE clear has only item (3), and **an edge that
  arrives on the cycle the WFE goes to sleep is lost**: not latched
  for that WFE, not seen by the sleeping core (the finding below).
  The pending bit then stands with no new edge to come.
- **The waking edge is latched too.** An interrupt that wakes a WFE
  enters the pending state, so SEVONPEND latches it as well, and the
  next WFE returns at once on it. PFIC_SCTLR.SETEVENT (3.1, "set the
  event to wake up the WFE case") sets the latch by hand, and a WFE
  executed over a set latch returns at once and clears it - the pair
  that consumes it.
- **The hardware prologue/epilogue (HPE)** pushes the ten caller-saved
  registers (x1, x5..x7, x10..x15) to the user stack on entry - SP
  moves by 48 bytes - and pops them on MRET, two levels deep at most
  (3.4). It is enabled by INTSYSCR.HWSTKEN and used by a handler
  declared with the vendor compiler's `interrupt("WCH-Interrupt-fast")`
  attribute, which then emits no prologue of its own.
- **The STK** is a 32-bit up-counter at HCLK or HCLK/8 with a compare,
  an auto-reload (STRE) and a flag the handler must clear (RM 6.5.4);
  it is interrupt 12 of the vector table, and the interrupt number is
  the table index on this core.
- **The reset flags accumulate** in RCC_RSTSCKR until RMVF is written,
  and **PINRSTF names the pin alone**: unlike the STM32 register this
  one descends from, a software reset does not raise it.
- **A reset request** is PFIC_CFGR.SYSRESET written with KEY3 (0xBEEF
  in the high half); it reads back as SFTRSTF.
- **`ebreak` with no debugger attached** is taken to the vector table's
  fault entry (index 3).
- **The IWDG** (RM ch. 4) is a 12-bit down-counter on the LSI behind a
  prescaler, started by a key and stopped by nothing but a reset; its
  two setting registers take a write only after the unlock key and
  read back only once their update flag has dropped (five LSI cycles).
- **The WWDG** (RM ch. 5) is a 7-bit down-counter on HCLK/4096 behind
  a second divider that resets when T6 falls and when it is refreshed
  above its window, with an early warning one step before; WDGA is
  cleared only by a reset - the RCC pulse on PB1PRSTR is one. **Its
  counter does not run unarmed** (measured): 5.2.1's "free operation
  regardless of whether the watchdog function is turned on" does not
  describe this silicon.
- **The two debug freezes** (IWDG_STOP, WWDG_STOP) are bits of a core
  CSR, DBGMCU_CR at 0x7C0 (RM ch. 21), not a peripheral register.

## Types and verbs

The code below is the QingKe core stratum's, written once for the three
QingKe families ([../qingke/README.md](../qingke/README.md)): this
family's [pfic.hpp](../../brio/ch32v00x/pfic.hpp),
[ticker.hpp](../../brio/ch32v00x/ticker.hpp) and
[delay.hpp](../../brio/ch32v00x/delay.hpp) are its device header over
[brio/qingke/pfic.hpp](../../brio/qingke/pfic.hpp),
[brio/qingke/ticker.hpp](../../brio/qingke/ticker.hpp) and
[brio/qingke/delay.hpp](../../brio/qingke/delay.hpp), and its platform
type a struct over
[brio/qingke/platform.hpp](../../brio/qingke/platform.hpp)'s
`QingkePlatform`, given this family's facts by a chip traits type
(`Ch32v00xQingke`: the bus question, the CH32V003's sleep entry from
SRAM with the EXTI events of its Standby, the idle store leaving
SLEEPONEXIT as found). The core files' comments carry the measurements
of all three families' cores, each with its chip named.

`Ch32v00xPlatform<TB>` ([brio/ch32v00x/platform.hpp](../../brio/ch32v00x/platform.hpp))
is the concept member for member: `CriticalSection` is pfic.hpp's
`InterruptGuard` (one `csrrci` reads and clears mstatus.MIE, the
destructor restores only what it found set), `idle()` is the SCTLR
store, the unmask, then the WFE - an interrupt taken between the
unmask and the `wfi` latched its event, so the WFE returns, and one
pending while the core sleeps is an enabled interrupt with MIE set, a
wake by level that no edge timing can lose - and after the wake
SETEVENT and one more `wfi`, which returns at once and consumes the
latch the wake left, so the caller's loop turns once per interrupt
(with MIE set by then, an interrupt pending around the consume is
taken by its level, not found by the latch) - on the CH32V003 a
masked entry whose `wfi` runs from SRAM instead (below, "On the
CH32V003F4P6"); `break_here()` is `ebreak`,
`atomic_width` 4, the breadcrumb in `.noinit` - and with a Standby
armed (SLEEPDEEP set by [sleep.md](sleep.md)'s site) `idle()` holds
the ticker off across the WFE, since a tick turning pending would end
it before the Standby began. The interrupt verbs and
the per-line enables are [brio/ch32v00x/pfic.hpp](../../brio/ch32v00x/pfic.hpp):
`enable/disable/enabled/pending/set_pending/clear_pending/active`
(the manual's ISR bank is the enable status and its IPR the pending
one; IENR/IRER/IPSR/IPRR are write-one), and `Pfic::in_handler()`,
which the platform's `in_handler()` forwards (kernel/platform.hpp's
optional member, `HandlerAware`): PFIC_GISR's GACTSTA, bit 8 at offset
0x4C, an interrupt being executed at any nesting level (the CH32V00X
reference manual 6.5.2.7, the CH32V003's 6.5.2.7, the QingKe V2 manual
3.1) - not mstatus.MIE, which interrupt entry leaves set on this core
("What the silicon does") and a critical section clears.
`BRIO_CH32_VECTOR` is the
one spelling of a vector binding, its handler attribute WCH's fast
attribute when the image is built with `CH32V00X_HPE` (the default)
and gcc's plain `interrupt` otherwise - one option for the whole
image, because a fast handler under an HPE that is off corrupts the
program it interrupted. Interrupt nesting is never enabled: the
kernel's rule ([design/kernel.md, "The model in one page"](../design/kernel.md#1-the-model-in-one-page)).
`stack_untouched()` is the RAM ledger: the crt paints the free RAM
between the last section the linker placed and the stack top with one
pattern before the first call, and the verb walks up from that floor
until the pattern breaks - how many bytes of stack no call has reached
since reset, the margin a program has on a part with 2 KB, zero
meaning the stack has run into the data below it. Every suite prints
it after each command.

The timebase is [brio/ch32v00x/ticker.hpp](../../brio/ch32v00x/ticker.hpp)'s
`BasicTicker<tps>` over the STK (`Ticker` at 1000 Hz), the busy-wait
[brio/ch32v00x/delay.hpp](../../brio/ch32v00x/delay.hpp)'s `delay_us`
(at least, never early, refused at one tick period and above, no
division at wait time), and the failing half
[brio/ch32v00x/reset.hpp](../../brio/ch32v00x/reset.hpp): `ResetFlag`,
`Reset::flags/clear_flags/take_flags/software`, `ResetReporter` (the
panic reporter that resets) and `fault_reset<P>()`, the fault vector's
body that writes a kernel_fault record - never over one that already
stands - and resets. The two watchdogs live there too: `Iwdg`
(`start()`, `configure(IwdgConfig)` - unlock, both fields, the
bounded wait for the update flags -, `arm()` in the chapter's order,
`refresh()`, `force_reset()`, `iwdg_timeout_ms(prescaler, reload,
lsi_hz)` at a stated LSI rate) and `Wwdg` (`configure(WwdgConfig)`
refusing a window below 0x40, `start(t)`, `refresh(t)` with T6 always
written set, `force_reset()`, `counter()`, the EWIF verbs and `isr()`,
`wwdg_timeout_us(hclk, prescaler, t)`), each with its `debug_freeze()`
over the CSR.

## How to use it

An app names the platform, binds the vectors it owns with the one
attribute, and hands the fault vector the body:

```cpp
using P = brio::Ch32v00xPlatform<>;

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_VECTOR(fault_handler) { brio::fault_reset<P>(); }

int main() {
    const auto record = brio::take_panic_record<P>();   // first, and once
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Tenuto<P, ...>::run();
}
```

A wait shorter than a tick is `delay_us(clock, us)`; a wait of a tick
or more is a TimeEvent and `delay_us` refuses it. A reboot on purpose
is `Reset::software()`; a panic that must be seen at the next boot
goes through `panic<P, ResetReporter>(code, context)`.

## Bench findings

The reference suite is `test_ch32_platform` (forty-one verdicts in
`z`, nine more in letter `i`, which reboots the board three times), on
the CH32V006K8U6 at 48 MHz, and letters `e` and `u` again in an image
built with `CH32V00X_HPE=OFF` (its `z` green too). What it measured:

- **The idle hook wakes.** With the console silent, two `idle()` calls
  covered the rest of the tick (632 to 888 us across runs; the first
  returned at once on the latch the console's last interrupt left) and
  returned with interrupts enabled; a 5 ms window with interrupts masked advanced the tick by
  one (the STK interrupt is a pending bit: one tick delivered, the
  rest coalesced).
- **No edge position loses the wake** (letter `w`): the STK's compare
  placed by a CNT write at each of 400 cycles after the write, the
  kernel's shape run against it - a masked check, `idle()`, until one
  tick and until two - with the event latch set and clear by hand,
  TIM2 timing each try and rescuing a lost one: 0 of 1600 tries lost
  for good and none a tick late - the slowest try 12 us after the
  edge it waited for, against the 1000 a wake deferred to the next
  tick would take (twelve code layouts, the idle path shifted by 0 to
  22 bytes, the same).
  The same letter built over the masked sleep this hook had before
  (`wfi` with MIE clear, the unmask after) loses 2 of 800 in each
  pass, one per latch state, and probing the edge to the cycle shows
  where: the handler is entered a fixed time after the write while the
  edge comes before the WFE decides (the latch ends it at once), 38 to
  40 cycles after the edge once the core is asleep - and on the ONE
  cycle between, the edge is lost and the core sleeps on with the STK
  pending and enabled, for good (a debugger's halt was the only thing
  that woke it). Twelve code layouts (the idle path shifted by 0 to 22
  bytes): the masked sleep lost that one position in every one, the
  unmasked none; WCH's sequence (SETEVENT, then two `wfi`) under the
  same masked check loses 45 positions: the 44 from the check to the
  second `wfi`, whose first `wfi` consumes the edge's latch, and the
  one cycle of its own sleep entry. An edge on a line the PFIC
  does not enable (TIM2's update, a SEVONPEND-only wake) keeps the
  hole with the unmasked order too: one position per sleep entry.
- **What the order costs**, in HCLK cycles at 48 MHz: from an edge
  that finds the core asleep to the handler's first statement 25
  against the masked order's 38 to 40 (the interrupt is taken by the
  wake itself instead of after an unmask). The Sleep path compiles to
  a leaf - one SCTLR load, its store, `csrsi`, `wfi`, then the
  consume's `ori`, store and `wfi` - and the Standby path is out of
  line.
- **One kernel turn per interrupt** (letter `k`): a Tenuto pack of
  three quiet AOs, two of them with a periodic time event, turned as
  `run()` turns it, the tick the only interrupt: 101 turns over 100
  ticks with the latch consumed, 200 without it - the second turn of
  each pair finding nothing, its `idle()` returning at once on the
  latch the wake left. That turn costs 287 cycles (two CNT reads
  included); the consume costs 6 to 15 cycles a wake by code layout:
  from an edge that finds the core asleep to the caller's loop 118 to
  126 against 112, an `idle()` the latch returns at once 63 to 69
  against 54, a quiet turn 290 to 295 against 287.
- **The STK arithmetic holds.** CMP = 47999 as programmed; over 200
  reloads the CNT-delta accumulation `delay_us` is built on tracked
  the interrupt count to 622 cycles of error out of 9.6 million
  (65 ppm, and under one period). `delay_us(100)` spent 4871 cycles
  for 4800 asked; twenty `delay_us(500)` took exactly 10 ticks; 1000
  us and above are refused.
- **The interrupt round trip, in HCLK cycles** (letters `e` and `u`, ten
  runs of each on each build): a line pended by hand through the PFIC
  with the cycle counter read at the raise, at the handler's first and
  last statements and when the raiser sees it done, 64 rounds each,
  every round identical - the software interrupt's minimal handler, and
  the console's own USART vector around the transport's `isr()` with
  nothing for it to do:

  | handler, image | entry | body | exit | whole trip |
  |---|---|---|---|---|
  | minimal, HPE on (`CH32V00X_HPE=ON`, the default) | 26 | 20 | 32 | **78** |
  | minimal, HPE off (gcc's own prologue) | 31 | 21 | 32 | **84** |
  | the console's USART handler, HPE on | 26 | 52 | 45 | **123** |
  | the console's USART handler, HPE off | 31 | 61 | 52 | **144** |

  Six cycles on the minimal handler and 21 - 0.44 us at 48 MHz, a
  seventh of the trip - on the console's: the ten registers go to the
  stack either way, and what the hardware saves is the fetch of the
  prologue and epilogue gcc writes, which grows with the registers a
  handler uses (in the HPE-off build gcc's stores interleave with the
  body's first loads, so its entry and body columns split the prologue
  differently - the whole trip is the comparable figure). The code
  saved in the same image: 28 bytes on the console's handler, 16 on the
  software and the tick handlers. Free in semantics, so it is the
  default; what it costs is the vendor attribute, available from WCH's
  gcc or from an upstream gcc with the fast-interrupt patch.
- **The reset flags.** The boot after the probe's post-programming
  reset reads 0x18000000: SFTRSTF and the power-on flag never cleared,
  and NO PINRSTF - the finding that rewrote reset.hpp's account of the
  pin flag. `take_flags()` leaves the register at zero with RMVF back
  at zero.
- **Three real resets** (letter `i`): `Reset::software()` boots to
  SFTRSTF alone and no breadcrumb; `panic<P, ResetReporter>(
  queue_overflow, 7)` boots to SFTRSTF with the record carrying code
  and context; `ebreak` with no debugger attached lands in
  `fault_handler`, whose `fault_reset<P>(0x51)` boots to SFTRSTF with
  a kernel_fault record carrying 0x51. The `.noinit` section does what
  the linker script and the crt promise.

The watchdogs' suite is `test_ch32_watchdog` (15 verdicts in `z` on
either part, two
letters ending in real resets):

- **The IWDG bites when the arithmetic says**: /32 and a reload of 999
  is 258 ms at the 124 kHz LSI the sleep chapter measured, and the
  feeding stopped, the board rebooted with IWDGRSTF alone after 255 ms
  (a .noinit token counting the loop's last millisecond). Fed every
  60 ms it never bites; a fresh boot finds it off.
- **The update flags cross in 41 us**: PVU seen, then down 2010 cycles
  after the prescaler write - five LSI cycles, as 4.3.4 says.
- **The WWDG's step is exact**: armed at /8, the first early warning
  2064546 cycles after the start for 63 x 32768 = 2064384; fed from
  its own early-warning handler it runs six warnings in 300 ms and
  never bites; unfed it reboots with WWDGRSTF alone after 44 ms for
  43.7 computed; a refresh made ABOVE the window (0x7F written under a
  window of 0x50) reboots the board at once.
- **The counter does not run unarmed** (the finding above), EWIF cannot
  be set by software, and the RCC pulse on PB1PRSTR clears WDGA and
  EWI - the one way to put an armed WWDG back without a reboot.

## On the CH32V003F4P6

The same suite on the family's smallest part
([../boards/ch32v003f4.md](../boards/ch32v003f4.md)) built as
`rv32ec_xw`, as two images there (letters `k` and `t` apart: whole the
suite is over the part's 15 KB): the reset flags and their history,
the critical section, the STK timebase (CMP 47999 at 48 MHz),
`delay_us` (100 us in 4858 to 4865 cycles), and the round trip - the
QingKe V2A takes an interrupt FASTER than the V2C: with the hardware
prologue, 24 cycles to the handler's first statement, 16 to 17
through a minimal body, 26 to 27 back to the raiser - 66 to 68 for
the whole trip against the V2C's 78.

**The V2A's sleep entry wedges, and `idle()` sleeps another way
there.** Letter `w`'s edge walk, behind its IWDG net, and a scratch
walk over seven entry shapes (each 1600 tries, the IWDG rebooting
each hang and the position logged in .noinit):

| entry shape | hangs | lost |
|---|---|---|
| the V2C's (unmasked, from flash, consume after) | 4 a pass, consecutive | 0 |
| the same without the consume | 4 a pass | 0 |
| the V2C's from SRAM, with or without the vendor's spin | 4 a pass | 0 |
| the V2C's with the hardware prologue off | 4 a pass | 0 |
| masked, from flash, with or without a spin, with or without the consume | 4 a pass | 0 |
| masked, from SRAM, a spin of 1 turn | 4 a pass | 0 |
| masked, from SRAM, a spin of 19 turns (the vendor's shape) | 0 | 1 a pass |
| `idle()` as shipped (below) | 0 | 0 |

A hang is a core nothing reaches: not its own interrupt, not a later
line's edge (TIM2's rescue), not the debugger's halt - with
DBGMCU_CR.SLEEP keeping HCLK on in Sleep as well; a reset is the only
way out. The window is four cycles wide at the wfi's entry, the
same in every shape that hangs. The cure is the shape of the vendor's
own CH32V003 WFE (the EVT's core_riscv.h, which the CH32V006's EVT
has not): the wfi run masked from SRAM with a spin there after it,
nothing fetched from flash in the first cycles after the wake. Its
remaining hole - one position a pass, an edge on the entry cycle not
seen - is the masked WFE's own, the one the CH32V006 measured of its
masked order.

`idle()` on the CH32V003 is that shape (platform.hpp, THE CH32V003):
masked throughout, each `wfi` executed from SRAM with the 19-turn spin
after it (`wfi_from_sram()`, twelve bytes of RAM in `.ram_text.brio`), with
two more parts the masked order needs - the latch the last wake left
is emptied FIRST (SETEVENT and a `wfi` it ends at once, 5 cycles) and
the PFIC asked whether an enabled line is pending, because after the
wake the V2C's consume found the latch back (200 turns over 100
ticks: the masked order takes the interrupt only after it) - and two
reasons not to sleep at all: the tick's match closer than 96 cycles
(the guard that keeps its edge off the entry) and a DMA channel at
work ([dma.md](dma.md): the V2A's Sleep stops the controller).
Measured: `w` 0 lost and 0 late in 1600 tries, the slowest 74 us for
one tick, in the suite's layout and twice in a scratch walk's (once
with DBGMCU_CR.SLEEP set); `k` 100 turns over 100 ticks, a quiet turn
229 cycles; `z` 32/0 and 7/0 on the two images.

**A Sleep on the V2A lasts at least about 3150 cycles at 48 MHz (66
us)**: an edge 100 to 2400 cycles after the entry runs the core again
3150 cycles after the entry, to five cycles, stamped in SRAM right
after the `wfi`. So an edge that finds the core asleep reaches the
caller's loop about 3100 cycles later (126 on the V2C), the tick floor
of an idle loop is 746 000 cycles a second (1.6 %), and an interrupt
that finds the core freshly asleep is served up to 66 us late - a
handler hold-off of some 3150 cycles that a transport timed against
the image's handlers alone does not foresee ([spi.md](spi.md)).

The Standby path takes the masked SRAM `wfi` too, its EXTI lines that
interrupt made EVENTS for the Standby's span (5.2's item (1), the
vendor's deep WFE doing the same): with the unmasked entry the sleep
suite's Standby walk ended in the net's reboot, with this one 12800
Standbys all came back ([sleep.md](sleep.md)).

## A handler in SRAM

**What the silicon does.** At 48 MHz the code flash answers behind two
wait states (RM 18.3.1), and a taken branch is a fetch the core waits
for: the SPI host's transmit-only loop takes 24.9 cycles a frame at
HCLK/2 from it, and 16.0 from SRAM - the wire's 16 ([spi.md](spi.md)).
The vector table's words after the first are ABSOLUTE addresses (the
QingKe V2's vectored mode, RM 6.3), so a handler placed in SRAM is
entered from its vector directly, and a call between SRAM and the flash
is an `auipc` and a `jalr`, which reach 2 GB either side: no veneer
either way. WHERE IN THE FLASH THE CODE SITS COSTS ALMOST NOTHING on the
CH32V006K8U6: the same body of four taken forward branches, looped a
hundred times at 48 MHz with interrupts masked, took 3363 cycles at every
offset 0 mod 4 and 3366 at every offset 2 mod 4 through 32 bytes, in
three passes of the same sweep (a scratch program) - three cycles once,
at the entry, and none a branch, where the CH32X035's QingKe V4C pays
four cycles a turn of that body by its address modulo four
([../ch32x035/platform.md](../ch32x035/platform.md)). The CH32V003's
V2A has not been swept. Every vector is aligned to four all the same
(`BRIO_CH32_VECTOR`, pfic.hpp), because what a vector costs should not
move with what the image puts before it: in two images of the platform
suite that differ in the alignment alone, ten runs each, the minimal
round trip took 78 cycles against 81 and the console's USART handler
123 against 129, for 4 to 28 bytes of padding an image across the
project's images.

**The option and its spelling: opt-in, per driver, and per vector.**
Both linker scripts place the input sections `.fast_text` and
`.fast_text.*`, then `.ram_text` and `.ram_text.*`, first in `.data`
([../../ch32v00x/ld/ch32v006k8.ld](../../ch32v00x/ld/ch32v006k8.ld)),
so the crt's copy of `.data` moves them into SRAM before `main()`, and
the platform says so (`Ch32v00xPlatform::places_fast_text`) - on both
parts, the CH32V003's 2 KB included: the placement is a program's
choice and its cost the program's to weigh (below), and a link that
does not fit is refused by the script's `ram` region. A driver of this
stratum takes util/placement.hpp's `InFlash` (the default) or `InRam`
where a path of its own pays from SRAM, measured: `SpiHost`'s polled
loops and the `Uart`'s DMA vector body, for the transmit engine's block
restart (a port without a transmit engine refuses `InRam`: it has
nothing to place). Under
`InRam` each such path is ONE out-of-line instance in `.fast_text.brio`,
flattened; under `InFlash` it is inline where it is called, as if there
were no trampoline:

```cpp
using Bus = brio::SpiHost<1, brio::spi1_default_pins, brio::NoDmaEngine, brio::NoDmaEngine,
                          brio::spi_default_hold_off_cycles, brio::InRam>;
using Link = brio::Uart<2, P, 256, 256, brio::DmaTxEngine<6>, brio::DmaRxEngine<7>, 3, brio::UartOptions{},
                        brio::InRam>;
```

A program places a function of its own - the vector it binds, above
all - with one attribute, `.fast_text`, which a vector carries on a
declaration before its binding (the binding's expansion begins with
`extern "C"`, where no attribute can stand in front of it, and gcc
merges the declaration's section into the definition):

```cpp
extern "C" [[gnu::section(".fast_text")]] void i2c1_ev_handler();
BRIO_CH32_VECTOR(i2c1_ev_handler) {
    if (Bus::isr()) { brio::post<I2cBus>(brio::TransferDone{Bus::status()}); }
}
```

THE NAME IS TWO NAMES. A function defined in a header is emitted in a
COMDAT group, and GCC refuses one section name for a grouped function
and a plain one in the same translation unit ("section type conflict",
WCH's gcc 15.2), so the trampoline's instances name `.fast_text.brio`
and are never cloned, and a program's plain function names
`.fast_text`; a program's own inline or template function goes through
`Hot<InRam>::run` rather than an attribute. `.ram_text` is the name of
code that MUST run from SRAM for correctness - on this family the
CH32V003's sleep entry (above), as `.ram_text.brio` for the same
reason, so that a program may place a plain function of its own in
`.ram_text` in a unit that emits `idle()`; the family fixtures compile
both kinds in one unit (`test/family_ch32v00x/platform.cpp`, `spi.cpp`,
`dma.cpp`, `i2c.cpp`).

**Which placement buys what.** Behind a vector in the flash, a driver's
body as an instance in SRAM is a CALL from the vector and a return into
the flash, and on this core the call and the return's refetch behind the
wait states cost about twenty to thirty cycles: a short body loses them
whole, a long or looping one repays them. So the driver places what
repays it - the polled loops, a call a phase, and the transmit engine's
block restart - and leaves inline in the vector what does not: the SPI
pump, the USART's `isr()` and the I2C's event pump, which a program
moves by placing the vector, the fastest of the three ways (no call at
all). Measured on the CH32V006K8U6 at 48 MHz, a handler's body between
the meter's stamps (about 46 cycles of them inside) and the hardware
prologue's entry and exit outside:

| path | placed by | from the flash | from SRAM | SRAM bytes |
|---|---|---|---|---|
| the SPI transmit-only loop, a frame at HCLK/2, 8 and 16 bits (the wire 16, 32) | the driver (`InRam`) | 24.9, 37.9 | 16.0, 33.7 | 446 a host type, both loops at both widths |
| the SPI receive loop, an 8-bit frame at HCLK/2, /4, /16 (the wire 16, 32, 128) | the driver | 34.9, 56.8, 166.7 | 32.9, 42.8, 148.4 | (above) |
| the USART DMA vector's body, the transmit engine's block restart | the driver | 238 to 248 | 196 to 197 (207 to 210 with the restart alone placed); 141 with the vector placed instead | 268 (212) |
| the USART interrupt transmitter's `isr()` | the program's vector | 135 to 136 | 76 with the vector placed (145 as an instance behind the flash's vector) | the vector |
| the SPI pump, an 8-bit frame at HCLK/4 with an in buffer | the program's vector | 259.4 a frame, 167 an entry | 174.7 a frame with the vector placed (274.4 as an instance behind the flash's vector) | 1114, the benchmark's vector with four hosts inline |
| the I2C event pump, an entry of a 255-byte write and read | the program's vector | 154, 182 | 92, 106 with the vector placed (161, 177 as an instance behind the flash's vector) | 1818, the benchmark's vector with three hosts inline |

At the wire's level: the transmit-only loop holds HCLK/2 from SRAM, the
receive loop's dead bus a frame falls from 25 cycles to 11 at HCLK/4,
the interrupt transmitter goes from 1.42 times the wire at 3 Mbaud to
1.08 with its vector placed, and the engine's block restart, inside its
window at 3 Mbaud from both placements, keeps more of it from SRAM: 40
cycles spent ahead of the body idle the line 52 to 78 cycles a boundary
from the flash and 0 to 25 from SRAM (`bench_ch32` letters e and u,
[spi.md](spi.md), [usart.md](usart.md), [i2c.md](i2c.md)).

**What it costs.** The bytes in the last column, of the part's SRAM -
the load image stays in the flash beside them. On the CH32V003's 2 KB
the loops are 448 bytes a host type and the restart 208: with every
eligible path placed, that part's SPI suite images would reach 2044 to
2048 bytes of the 2048, and the benchmark's letter-e image would
overflow by 316, which is why nothing is placed by default. A placed
function exists only from the crt's copy on, and the crt enters
`main()` with interrupts masked, so no vector reaches SRAM the copy has
not written. THE HOLD-OFF the SPI host draws its write-ahead from is a
fact of the image: its default counts handlers in the flash, and a
program that places its handlers may state a shorter one
([spi.md](spi.md)).

## Not covered yet

Driver gaps, each with its reason:

- The PFIC's priorities and its two free vectored entries: no user.
- The hardware-enabled IWDG (the IWDG_SW option byte): an option byte
  write is the flash chapter's, and the bench part keeps the factory
  option bytes.
- The watchdogs' debug freeze as a measurement: what a halted core
  does to a running dog is a debugger session, not a suite letter.

Implemented but not bench-verified, each with what would measure it:

- **`in_handler()` on this silicon**: the read of PFIC_GISR's GACTSTA
  is the manual's on both parts,
  compiled for both; a letter that asks it from a
  vector, from the main loop and under a critical section, as the
  STM32F4's serial suite does
  ([../stm32f4/usart.md](../stm32f4/usart.md), letter h), would
  measure it.
- The idle hook's POWER: `idle()` is proven to sleep and wake, and the
  loop to turn once per interrupt, not to sleep cheaply -
  a current measurement with the probe detached (a core in debug mode
  never sleeps, QingKe V2 manual 5.1). The Standby it enters when
  [sleep.md](sleep.md)'s site has armed one is measured the same way.
- The CH32V003's late edge: an enabled line other than the tick whose
  edge lands on the V2A's sleep-entry cycle itself waits, pending and
  masked, for the next wake - inferred from the masked order's hole,
  measured as one position per pass with the STK's edge and no guard;
  a walk of a non-tick line's edge across the shipped entry would
  measure it (the sleep suite's AWU walk does it for a line that is
  also an EXTI event, which loses none).
- The CH32V003's round trip with every vector aligned to four: the
  V2A figures above were read off an image whose vectors sat wherever
  gcc's two-byte alignment put them; the platform suite's letter `e` on
  the CH32V003F4P6 would measure the aligned one.
- The V2A's minimum Sleep at other clocks: about 3150 cycles was
  measured at 48 MHz only; whether it is a time (66 us, about eight
  LSI periods) or a cycle count is a run of the latency stamp at a
  lower rung of the DynamicClock.
