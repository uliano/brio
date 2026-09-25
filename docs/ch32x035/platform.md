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
the V4C, 2.2 for what a trap does to mstatus and 8.2 for what an MRET
leaves in it, 3.4 for the hardware prologue, 6 for the sleep modes and
what ends them, 8.3 for corecfgr).
No errata sheet exists for this series. The headers are
[brio/ch32x035/platform.hpp](../../brio/ch32x035/platform.hpp),
[pfic.hpp](../../brio/ch32x035/pfic.hpp),
[ticker.hpp](../../brio/ch32x035/ticker.hpp) and
[delay.hpp](../../brio/ch32x035/delay.hpp); the reference suite is
`test_x035_platform`.

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
  pending an event, masked or not, LATCHED - "if the WFE instruction is
  not executed, the system will be woken up immediately after the next
  execution of the instruction" (RM 7.5.2.38). That form is correct
  under either reading, and it is the platform's - and it wakes on this
  core: `idle()`, called with interrupts masked as the kernel calls it,
  returns on the next tick with them enabled (measured).
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
  round trip WITH it is 100 to 104 cycles at 48 MHz (measured), and the
  image without it has not run.
- **The STK is sixty-four bits wide** (RM 7.2, 7.5.5): CNTL/CNTH count
  up (or down, MODE) at HCLK or HCLK/8 towards CMPLR/CMPHR, reload from
  zero with STRE, and raise CNTIF, which the handler clears by writing
  zero. It is interrupt 12 of the vector table, and the interrupt number
  IS the table index on this core. STK_CTLR also carries SWIE, the
  software interrupt's other way in. This manual gives the register NO
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
  loop; on the CH32V303's V4F that loop ran slightly SLOWER with them
  ([../ch32vx03/platform.md](../ch32vx03/platform.md)); on this core a
  loop of the platform suite's own runs 7.3 to 7.4 per cent SLOWER with
  them than with the register cleared (measured).
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

## Bench findings

The reference suite is `test_x035_platform` (42 verdicts in `z`, nothing
wired) on a CH32X035F8U6 - WCH's evaluation board in its QFN20 edition,
over a WCH-LinkE - at 48 MHz from the HSI with the flash at two wait
states, in the image built with the hardware prologue; where two numbers
stand for one quantity they are two runs of `z`. What it measured:

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
  masked advanced the tick by one in both runs: the STK interrupt is a
  pending bit, one tick delivered and the rest coalesced. With the
  console silent, two `idle()` calls - each made with interrupts masked,
  as the kernel makes it - covered the 507 us to the next tick (898 us in
  the other run) and returned with interrupts enabled: the WFE form
  wakes on this core.
- **The STK timebase** (letter `c`). CTLR reads 0xF - counting up on
  HCLK, reloading, interrupting - with CMPLR = 47999 as programmed and
  CMPHR and CNTH both zero: the reload puts the low half back at the
  compare, so the high half of the 64-bit counter never moves.
  `ticks()`, `millis()`, `secs()` and `now()` agree (3488, 3488, 3 and
  3.488 s). Over 200 reloads bracketed between two tick edges, the
  CNT-delta accumulation `delay_us` rests on came to 9599716 and 9599852
  cycles against 9600000: 284 and 148 cycles short, 30 and 15 ppm.
- **`delay_us` is late by a constant, never early** (letter `d`). 100 us
  spent 4924 and 4912 HCLK cycles for 4800 asked, and 900 us - the
  longest wait that still fits a tick period - 43312 and 43316 for 43200:
  112 to 124 cycles late at both lengths, not a proportional error.
  Twenty waits of 500 us took exactly 10 ticks; 1000 us, 50 ms and the
  65536 us gate are refused, and a zero wait is served.
- **The interrupt round trip, in HCLK cycles** (letter `e`): the
  software interrupt - vector 14 - raised by hand through the PFIC's
  pending register, with the cycle counter read at the raise, at the
  handler's first and last statements and when the raiser sees it done,
  64 rounds each and every raise taken: entry 37 in every round, body 27,
  exit 36 with one iteration of the raiser's spin in it, the whole trip
  100 to 104 - the same in both runs. That is the image built with the
  hardware prologue (`CH32X035_HPE=ON`, the default); the image built
  without it has not run, so what the prologue buys on this core is not
  measured (the list below).
- **corecfgr's 0x1F makes this core SLOWER** (letter `f`). The crt left
  0x1F in the register; 2000 iterations of a loop with a load and a
  data-dependent branch over a 64-byte pseudo-random table took 24204
  and 24224 HCLK cycles with it, and 22548 in both runs with the
  register cleared - 7.3 and 7.4 per cent more with the vendor's bits -
  the register reading back zero for the cleared loop and 0x1F after it.
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
- **The RAM ledger answers.** `stack_untouched()` read 19556 bytes never
  touched after this suite's whole `z`, 19616 after the clock suite's
  and the pin suite's, and 19416 after the USART suite's, of the part's
  20 KB.

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

- **What the hardware prologue buys on this core**: letter `e` has run
  in the image built with it (above); the same letter in an image built
  with `CH32X035_HPE=OFF` is the other half of the difference, and
  `stack_untouched()` after it the user stack the internal hardware stack
  spares.
- **The HSI's own accuracy**, which the tick inherits: letter `g`
  brackets 5000 ticks between two console lines and runs to its count;
  what is missing is a host that time-stamps the two lines as they
  arrive - the clock suite's letter `d` asks the same at two rates
  ([clock.md](clock.md)).
- **The idle hook's POWER**: `idle()` proven to return is not `idle()`
  proven to sleep cheaply - whether the latched event is consumed by the
  `wfi` or leaves the loop spinning is a current measurement with the
  probe detached.
