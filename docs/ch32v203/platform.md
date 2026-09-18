# Platform (CH32V203)

The kernel's Platform concept realized on the QingKe V4B, in two
halves: the RUNNING half - the critical section, the idle hook, the STK
timebase, the microsecond busy-wait, the interrupt round trip - and the
FAILING half - which reset happened, how to cause one, and the record a
crash leaves for the next boot.

Documents of record: the CH32F/V20x_V30x_V31x reference manual V2.3
(3.2 for the reset sources and 3.4.10 for the flags, 9.5.2 for the
PFIC's registers, all of it read for the CH32V20x_D6 class this part
belongs to) and the QingKe V4 microprocessor manual V1.1 (2.1 for the
exception codes, 2.2 for what a trap does to mcause, mepc, mtval and
mstatus, 3.2 and 3.4 for the interrupt CSRs and the hardware prologue,
5 for the system timer, 6 for the sleep modes, 8 for the CSR list). No
errata sheet exists for this family; the bench findings below are the
entries this document keeps in its place.

## What the silicon does

- **Interrupt entry does not clear MIE** (V4 manual 2.2), so a handler
  runs with the global enable still set; the PFIC decides whether
  another line may preempt it, and with INTSYSCR.INESTEN off - which is
  what the crt leaves - it holds the next line until MRET.
- **A WFI sleep ends on an interrupt the controller RESPONDS to**
  (6.2) - a wording that does not say whether the global mask must be
  set for the wake, which is the question the smaller QingKe core
  answers with a deadlock. The core can sleep as a WFE instead:
  PFIC_SCTLR.WFITOWFE turns the next `wfi` into a wait-for-event and
  SEVONPEND makes every interrupt entering the pending state an event,
  LATCHED, so a WFE after the event returns at once. That form is
  correct under either reading, and it is what the platform uses.
- **A core in debug mode cannot enter any sleep mode at all** (6.1), so
  a sleep measured with a probe halted on the core is not a
  measurement.
- **The hardware prologue/epilogue (HPE)** saves SIXTEEN caller-saved
  registers (x1, x5..x7, x10..x17, x28..x31) in one cycle to an
  INTERNAL stack the program cannot see, three levels deep, and pops
  them on MRET (3.4). This is where the two QingKe cores differ: the V2C
  of the CH32V00x pushes its ten registers to the USER stack, so there
  the hardware buys only the fetch of a prologue, while here it buys
  the pushes themselves (measured below: 64 bytes of stack and 10
  cycles). It is enabled by INTSYSCR.HWSTKEN and used by a handler
  declared with the vendor compiler's `interrupt("WCH-Interrupt-fast")`
  attribute, which then emits no prologue of its own.
- **The STK is SIXTY-FOUR bits wide** (5): CNTL/CNTH count up (or down,
  MODE) at HCLK or HCLK/8 towards CMPLR/CMPHR, with an auto-reload
  (STRE) and a flag the handler must clear by writing zero to it. It is
  interrupt 12 of the vector table, and the interrupt number IS the
  table index on this core. Its control register also carries SWIE, the
  software interrupt's other way in.
- **The reset flags accumulate** in RCC_RSTSCKR until RMVF is written
  (3.4.10), and the register's own reset value is 0x0C000000: a
  power-on raises PORRSTF and PINRSTF TOGETHER, so the pin flag does
  not name the pin at the boot that follows one. This family carries
  six flags where the CH32V00x carries seven of its own: the two
  watchdogs, software, power, pin, and LPWRRSTF - the low-power reset
  a Standby or Stop entry becomes when the option bytes say so (3.2.2).
- **A reset request is PFIC_CFGR.RSTSYS** written with KEY3 (0xBEEF in
  the high half, 9.5.2.10); PFIC_SCTLR bit 31 is the same reset with no
  key. It reads back as SFTRSTF, alone (measured).
- **`ebreak` with no debugger attached is taken to the vector table's
  BREAKPOINT entry, index 9** (measured), not to the exception entry at
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

## Types and verbs

`Ch32v203Platform<TB>` ([brio/ch32v203/platform.hpp](../../brio/ch32v203/platform.hpp))
is the concept member for member: `CriticalSection` is pfic.hpp's
`InterruptGuard` (one `csrrci` reads and clears mstatus.MIE, the
destructor restores only what it found set), `idle()` is the
WFITOWFE/SEVONPEND sequence above followed by the unmask - guarded by
the count of active bus masters and, with a deep mode armed, by a pause
of the timebase ([sleep.md](sleep.md)) - `break_here()`
is `ebreak`, `atomic_width` 4, `now()` the timebase's tick count, and
the breadcrumb a `PanicRecord` in `.noinit`. The interrupt verbs and the
per-line enables are [brio/ch32v203/pfic.hpp](../../brio/ch32v203/pfic.hpp):
`enable/disable/enabled/pending/set_pending/clear_pending/active` (the
manual's ISR bank is the ENABLE status and its IPR the pending one;
IENR/IRER/IPSR/IPRR are write-one). `BRIO_CH32_INTERRUPT` is the one
spelling of the handler attribute, expanding to WCH's fast attribute
when the image is built with `CH32V203_HPE` (the default) and to gcc's
plain `interrupt` otherwise - one option for the whole image, because a
fast handler under an HPE that is off corrupts the program it
interrupted. Interrupt nesting is never enabled: the kernel's rule
([../design/kernel.md](../design/kernel.md), section 1).
`stack_untouched()` is the RAM ledger: the crt paints the free RAM
between the last section the linker placed and the stack top with one
pattern before the first call, and the verb walks up from that floor
until the pattern breaks - how many bytes of stack no call has reached
since reset.

The timebase is [brio/ch32v203/ticker.hpp](../../brio/ch32v203/ticker.hpp)'s
`BasicTicker<tps>` over the STK (`Ticker` at 1000 Hz; `init`, `tick`,
`ticks/millis/secs/now`, `pause/resume`, `advance` for a site that
froze time, `rebase` for a clock that changed rate), and the busy-wait
[brio/ch32v203/delay.hpp](../../brio/ch32v203/delay.hpp)'s `delay_us`:
at least, never early, no division at wait time, and REFUSED - with no
time spent and a `[[nodiscard]]` answer - for a request of one tick
period or more, above the 65536 us gate its 32-bit arithmetic rests on,
or with the counter stopped.

The failing half is [brio/ch32v203/reset.hpp](../../brio/ch32v203/reset.hpp):
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
attribute, and hands BOTH trap entries the fault body:

```cpp
using P = brio::Ch32v203Platform<>;

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void fault_handler() { brio::fault_reset<P>(); }
extern "C" BRIO_CH32_INTERRUPT void breakpoint_handler() { brio::fault_reset<P>(); }

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

## Bench findings

The reference suite is `test_v203_platform` (forty verdicts in `z`,
nine more in letter `i`, which reboots the board three times), on the
CH32V203C8T6 at 144 MHz from the HSI. What it measured:

- **The idle hook wakes.** With the console silent, two `idle()` calls
  covered the 626 us to the next tick and returned with interrupts
  enabled; a 5 ms window with interrupts masked advanced the tick by
  one (the STK interrupt is a pending bit: one tick delivered, the rest
  coalesced).
- **The STK arithmetic holds.** CMPLR = 143999 as programmed, CMPHR and
  CNTH both zero - the reload puts the low half back at the compare, so
  the high half of this 64-bit counter never moves. Over 200 reloads,
  bracketed between two tick edges, the CNT-delta accumulation
  `delay_us` is built on tracked the interrupt count to 39 cycles of
  error out of 28 800 000 (1 ppm).
- **`delay_us` is late by a constant, never early.** 100 us spent 14434
  HCLK cycles for 14400 asked and 900 us - the longest wait that still
  fits a tick period - spent 129643 for 129600: some 40 cycles of
  polling granularity either way, not a proportional error. Twenty
  waits of 500 us took exactly 10 ticks; 1000 us and above are refused,
  and so is the 65536 us gate.
- **The interrupt round trip, in HCLK cycles**: a software interrupt
  raised by hand through the PFIC's pending register, with the cycle
  counter read at the raise, at the handler's first and last statements
  and when the raiser sees it done, 64 rounds each, every round
  identical to the cycle:

  | image | entry | body | exit | whole trip | .text | user stack |
  |---|---|---|---|---|---|---|
  | HPE on (`CH32V203_HPE=ON`, the default) | 16 | 12 | 25 | **53** | 12500 B | - |
  | HPE off (gcc's own prologue) | 19 | 12 | 32 | **63** | 12652 B | 64 B more |

  Ten cycles on a minimal handler - 69 ns at 144 MHz - 152 bytes of
  flash over this image's five handlers, and SIXTY-FOUR BYTES OF USER
  STACK that the hardware stack carries instead (`stack_untouched()`
  after the same letter, 19540 against 19476). That last number is what
  separates this core from the CH32V00x's, where the same feature moves
  the pushes from the handler's code to the hardware but leaves them on
  the user stack; and 53 cycles against that family's 83 is what a
  bigger core does with the same measurement. Free in semantics, so it
  is the default.
- **corecfgr's 0x1F buys nothing measurable here.** Three thousand
  iterations of a loop with a load and a data-dependent branch over a
  pseudo-random byte table took 36811 HCLK cycles with the register as
  the crt leaves it and 36813 with it cleared to zero - two cycles,
  0.005 per cent, with the register read back as zero to prove the
  second run really ran without the bits. The value stays written
  because it is what the vendor's silicon is shipped running, not
  because this bench can show what it does.
- **The tick is the HSI's, and the HSI of this die is half a per cent
  fast.** Letter `g` brackets 5000 of its own ticks between two console
  lines; the host's clock timed that bracket at 4.9744 s over three
  runs (-5.1e-3). The same bracket with the PLL fed from the board's
  8 MHz crystal instead of the HSI measures 4.9993 s (-1.4e-4), which
  is the console path's own bias - so the 5.0e-3 between them is the
  oscillator and not the method. A program whose timebase must be
  accurate takes the crystal.
- **The reset flags.** The boot after the probe's post-programming
  reset reads 0x10000000: SFTRSTF and nothing else, the flags having
  been cleared at the previous boot - so that reset is a system reset
  and reads as software. `take_flags()` leaves the register at zero with
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
  crt promise across all three.

## Not covered yet

Driver gaps, each with its reason:

- The PFIC's priority threshold, its nesting depth and its four free
  vectored entries (VTFADDRR): an ISR body runs to completion on every
  target ([../design/kernel.md](../design/kernel.md)), so nesting has
  no user here, and the vectored entries arrive with one.
- The second route to the same reset, PFIC_SCTLR bit 31 with no key,
  and that register's SLEEPONEXIT: one way in is enough for a reboot,
  and SLEEPONEXIT has no user - a brio program sleeps in the kernel
  loop's idle path and not on a handler's exit, and the rest of that
  register's sleep bits are the power chapter's ([sleep.md](sleep.md)).
- mepc and mtval are readable but do not cross a reset: the breadcrumb
  has one byte for the detail, and it carries the cause.

Implemented but not bench-verified, each with what would measure it:

- **PINRSTF as the flag of an event**, rather than as the register's
  documented reset value: the NRST pin pulled low by hand, and a supply
  cycled, with the flags read at the boot that follows. PORRSTF is the
  flag a Standby wake leaves ([sleep.md](sleep.md)), so that half is
  measured.
- **LPWRRSTF**: nothing raises it. The two watchdog flags are measured
  in [watchdog.md](watchdog.md), and a Standby wake - the one event
  that might have been a "low-power reset" - leaves PORRSTF instead
  ([sleep.md](sleep.md)), so what sets this flag is still unknown; the
  option bytes that would arm one are decoded and never written, by the
  flash chapter's own decision ([nvm.md](nvm.md)).
- **The idle hook's POWER.** `idle()` is proven to sleep and wake, not
  to sleep cheaply: whether the latched event is consumed by the `wfi`
  or leaves the loop spinning is a current measurement with the probe
  detached, since a core in debug mode never sleeps at all.
- **Whether a bare `wfi` wakes with MIE clear on this core.** The
  platform never does it - the WFE form is correct under both readings
  of the specification - so the question stands open; a letter that
  sleeps with the global mask clear over a pending tick, with an
  independent watchdog armed as the way back, would answer it.
- Nothing else: the timebase's `advance()`, `pause()` and `resume()`
  now have their user and their measurement in [sleep.md](sleep.md) -
  the idle path pauses the tick across a deep sleep and the timed site
  hands the frozen span back.
