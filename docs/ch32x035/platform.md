# Platform (CH32X035)

The kernel's Platform concept realized on the QingKe V4C of the
CH32X035 series: the critical section, the idle hook, the STK timebase,
the microsecond busy-wait, the one spelling of an interrupt handler,
and what the crt writes into the core before main(). The failing half
the sibling strata carry beside it - which reset happened as a type of
its own, the record a crash leaves, the watchdogs - is not written here
(the list at the end); the reset flags themselves are RCC's and live in
[clock.md](clock.md).

Documents of record: the CH32X035 reference manual V1.8 (3.2 for the
reset sources, 7.1 and table 7-1 for the PFIC and the vector table,
7.5.2 for the PFIC's registers with 7.5.2.38 for PFIC_SCTLR, 7.5.3 for
INTSYSCR and mtvec, 7.5.5 for the STK, ch. 19 for the electronic
signature) and the QingKe V4 microprocessor manual V1.1 (table 1-1 for
the V4C, 2.2 for what a trap does to mstatus, 3.4 for the hardware
prologue, 6 for the sleep modes and what ends them, 8.3 for corecfgr).
No errata sheet exists for this series. The headers are
[brio/ch32x035/platform.hpp](../../brio/ch32x035/platform.hpp),
[pfic.hpp](../../brio/ch32x035/pfic.hpp),
[ticker.hpp](../../brio/ch32x035/ticker.hpp) and
[delay.hpp](../../brio/ch32x035/delay.hpp); the reference suite is
`test_x035_platform`.

## What the silicon does

What the documents say; the CH32V203's QingKe V4B, the closest core
brio has measured, is cited where its behaviour is the question a
letter here answers.

- **The QingKe V4C is the V4B with a faster divider and a memory
  protection unit**: RV32IMAC, thirty-two registers, WCH's `xw`
  compressed extension, two levels of hardware stack and two of nesting,
  four vector-table-free channels, a five-cycle integer divide where the
  V4B takes nine (the reference manual's core table; the QingKe V4
  manual's table 1-1).
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
  pending an event, masked or not, LATCHED - "if the WFE instruction is
  not executed, the system will be woken up immediately after the next
  execution of the instruction" (RM 7.5.2.38). That form is correct
  under either reading, and it is the platform's.
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
  ([../ch32vx03/platform.md](../ch32vx03/platform.md)).
- **The STK is sixty-four bits wide** (RM 7.2, 7.5.5): CNTL/CNTH count
  up (or down, MODE) at HCLK or HCLK/8 towards CMPLR/CMPHR, reload from
  zero with STRE, and raise CNTIF, which the handler clears by writing
  zero. It is interrupt 12 of the vector table, and the interrupt number
  IS the table index on this core. STK_CTLR also carries SWIE, the
  software interrupt's other way in. This manual gives the register NO
  INIT bit (its bits 30:5 are reserved), where the QingKe V4 manual's
  chapter on the counter has one at bit 5; nothing here writes it.
- **Traps land in the vector table by kind**: the exception entry at
  index 3 and a breakpoint's own at index 9 - on the CH32V203 an
  `ebreak` with no debugger reached index 9 while mcause said exception
  code 3 (measured there).
- **corecfgr (CSR 0xBC0) configures the pipeline and the instruction
  prediction** and the V4 manual gives it no bit table (8.3); this
  series' reference manual does not name it. WCH's own startup file for
  the series writes 0x1F into it, and so does this crt. On the
  CH32V203's V4B the five bits bought two cycles in 36811 of a timed
  loop; on the CH32V303's V4F that loop ran slightly SLOWER with them
  ([../ch32vx03/platform.md](../ch32vx03/platform.md)).
- **A reset request is PFIC_CFGR.SYSRST written with KEY3 (0xBEEF in
  the high half), or PFIC_SCTLR.SYSRST with no key** (RM 3.2.2,
  7.5.2.10, 7.5.2.38). And with PFIC_SCTLR.RSTEN at its reset value of
  zero the chip has a CORE DEADLOCK RESET, generated - in 3.2.2's words -
  "when the core addresses an exception or enters an NMI interrupt", and
  never in debug mode; what exactly that sentence covers is not read
  further here, and no verb arms or disarms it.

## Types and verbs

`Ch32x035Platform<TB>`
([brio/ch32x035/platform.hpp](../../brio/ch32x035/platform.hpp)) is the
concept member for member: `CriticalSection` is pfic.hpp's
`InterruptGuard` (one `csrrci` reads and clears mstatus.MIE, the
destructor restores only what it found set, so guards nest), `idle()`
is the WFITOWFE/SEVONPEND sequence above followed by the unmask,
`break_here()` is `ebreak`, `now()` the timebase's tick count,
`ticks_per_second` the timebase's rate, `atomic_width` 4, and the
breadcrumb a `PanicRecord` in `.noinit`, which the crt neither loads nor
zeroes. With SLEEPDEEP found set - which no verb of this stratum does -
`idle()` pauses the timebase across the sleep and clears its pending
bit first, as the sibling strata's hooks do, because a tick that is
merely pending would end the sleep before it began. There is no
`idle_until()` (the timebase stops with the core) and no count of bus
masters: on the CH32V203 and the CH32V303 no master but the core gets a
bus cycle in a sleep, which is why that stratum's idle path counts them,
and here no driver starts one.

The interrupt verbs are [pfic.hpp](../../brio/ch32x035/pfic.hpp)'s:
`enable_interrupts`, `disable_interrupts`, `interrupts_enabled`, and
`Pfic`'s per-line `enable/disable/enabled/pending/set_pending/
clear_pending/active` over the write-one banks (the manual's ISR bank is
the ENABLE status and its IPR the pending one). `BRIO_CH32_INTERRUPT` is
the one spelling of the handler attribute: WCH's fast attribute when the
image is built with `CH32X035_HPE` (the default), gcc's plain
`interrupt` otherwise - one option for the whole image, because a fast
handler under an HPE that is off corrupts the program it interrupted -
and both carry `no_icf`: gcc's identical code folding turns the second
of two handlers of one body into a handler that CALLS the first, which
ends in MRET, so the call never returns and the caller's frame stays on
the stack (measured on the CH32V303VCT6,
[../ch32vx03/platform.md](../ch32vx03/platform.md)). Interrupt nesting
is never enabled: the kernel's rule
([../design/kernel.md](../design/kernel.md), section 1).

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
REFUSED - no time spent, a `[[nodiscard]]` answer - for a request of one
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
`take_panic_record<P>()` if a reset comes before the power goes.

## Not covered yet

Driver gaps, each with its reason:

- **Reset as a type, the fault record and the two watchdogs** - the
  sibling strata's `reset.hpp` (`Reset`, `ResetReporter`,
  `fault_reset<P>()`, `Iwdg`, `Wwdg`): born with the watchdog chapter
  (RM ch. 5, 6), their first user. A reboot on purpose, a panic that
  survives into the next boot and a fault body that records its cause
  wait for it; the reset flags are `Rcc`'s verbs meanwhile
  ([clock.md](clock.md)).
- **The PFIC's priorities, its nesting and its four vector-table-free
  channels** (RM 7.5.2.9, 7.5.2.12 to 7.5.2.16, 7.5.2.37): no user, and
  nesting declined by the kernel's rule.
- **The physical memory protection unit** (RM 7.5.4): no user.
- **The core deadlock reset's switch, PFIC_SCTLR.RSTEN**: left at its
  reset value, the reset on. Which traps raise it is a measurement - an
  exception taken inside the fault handler, the reset flags read at the
  next boot - that belongs with the fault body above, its first user.
- **The debug module's freeze bits** (DBGMCU_CR, CSR 0x7C0, RM 23.2.1):
  no verb reads or writes them - on the CH32V203 a `csrw` to that CSR
  from the running program resets the part
  ([../ch32vx03/sleep.md](../ch32vx03/sleep.md)).

Implemented but not bench-verified, each with what would measure it:

- **The reset flags as a history** and the electronic signature - the
  flash capacity and the unique identifier - with the word the vendor's
  library reads as the chip identifier printed: letter a.
- **The critical section and the idle hook**: masked, nested, restored
  only as found, the tick coalescing through a masked window, and
  `idle()` returning on the tick with interrupts enabled - which is the
  WFE form working on this core: letter b.
- **The STK timebase**: the compare the ticker programs, the high halves
  that never move under the reload, `ticks/millis/secs/now` agreeing,
  and the counter's delta accumulation `delay_us` rests on checked
  against the interrupt count over 200 reloads: letter c.
- **`delay_us`**: at least, never early, the cap and the refusal, a 100
  us and a 900 us wait counted in HCLK cycles: letter d.
- **The interrupt round trip** in HCLK cycles - raise, handler entry,
  handler exit, the raiser seeing it done - with the image saying which
  way it was built, so the hardware prologue's worth on this core is the
  difference between two images (`CH32X035_HPE` on and off): letter e.
- **corecfgr's 0x1F**: one known loop timed with the register as the crt
  left it and with it cleared: letter f.
- **The HSI's own accuracy**, which the tick inherits: a span of ticks
  bracketed by two console lines, for the host to time: letter g.
- **What the crt left in the core**: INTSYSCR with the hardware stack as
  the build asked and nesting OFF, mtvec on the table with both mode
  bits, mstatus readable at all - machine mode, a user-mode read trapping
  - with its MPP printed and not judged (an MRET may leave it at user
  mode, as the privileged specification has it), and misa as the core
  reports it: letter h.
- **The idle hook's POWER**: `idle()` proven to return is not `idle()`
  proven to sleep cheaply - whether the latched event is consumed by the
  `wfi` or leaves the loop spinning is a current measurement with the
  probe detached.
