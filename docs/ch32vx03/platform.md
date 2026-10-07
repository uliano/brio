# Platform (CH32V203 and CH32V303)

The kernel's Platform concept realized on the QingKe V4B of the
CH32V203 and the QingKe V4F of the CH32V303 - the same core with a
single-precision floating-point unit, RV32IMAFC where the other is
RV32IMAC - in two halves: the RUNNING half - the critical section, the
idle hook, the STK timebase, the microsecond busy-wait, the interrupt
round trip, and on the V4F the floating-point unit's state and what an
interrupt pays for it - and the FAILING half - which reset happened,
how to cause one, and the record a crash leaves for the next boot.

Documents of record: the CH32F/V20x_V30x_V31x reference manual V2.3
(3.2 for the reset sources and 3.4.10 for the flags, 9.5.2 for the
PFIC's registers, read for the CH32V20x_D6 class of the CH32V203C8 and
the CH32V30x_D8 class of the CH32V303) and the QingKe V4 microprocessor
manual V1.1 (2.1 for the exception codes, 2.2 for what a trap does to
mcause, mepc, mtval and mstatus, 3.2 and 3.4 for the interrupt CSRs and
the hardware prologue - its note 3 for what that prologue does NOT
save - 5 for the system timer, 6 for the sleep modes, 8 for the CSR
list and 8.2 for mstatus.FS and the floating-point CSRs). No errata
sheet exists for this family; the bench findings below are the entries
this document keeps in its place.

## What the silicon does

- **Interrupt entry does not clear MIE** (V4 manual 2.2), so a handler
  runs with the global enable still set; the PFIC decides whether
  another line may preempt it, and with INTSYSCR.INESTEN off - which is
  what the crt leaves - it holds the next line until MRET.
- **A WFI sleep ends on an interrupt the controller RESPONDS to**
  (6.2), and with mstatus.MIE clear it responds to none: a `wfi`
  executed masked on the CH32V203C8T6 slept past the STK and TIM2, both
  pending and enabled, for as long as it was left - twenty seconds, and
  five in a second run - until a debugger's halt woke it (measured; 6.2
  lists a debug request among the wakes). The core can sleep as a WFE
  instead: PFIC_SCTLR.WFITOWFE turns `wfi` into a wait-for-event and
  SEVONPEND makes every interrupt entering the pending state an event,
  LATCHED, so a WFE after the event returns at once. 6.2 gives a WFE
  three wakes that matter here - an event; an ENABLED interrupt "when
  an interrupt is generated", whose handler then runs first; and,
  through SEVONPEND, "a new interrupt pending signal (the previously
  generated pending signal does not take effect)" - and only the last,
  an EDGE, reaches a core sleeping with MIE clear. The platform
  therefore sleeps with MIE set.
- **The waking edge is latched too.** An interrupt that wakes a WFE
  enters the pending state, so SEVONPEND latches it as well, and the
  NEXT WFE returns at once on it. PFIC_SCTLR.SETEVENT (V4 manual 3.1,
  "set the event to wake up the WFE case") sets the latch by hand, and
  a WFE executed over a set latch returns at once and clears it - the
  pair that consumes it. WFITOWFE and SEVONPEND stay set after
  a `wfi` (measured: a WFE with no store before it still waits for an
  event), so "the subsequent WFI" is not a one-shot on this core.
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
- **The V4F's floating-point unit is OFF out of reset.** mstatus.FS
  (bits 14:13) is its state - 00 Off, 01 Initial, 10 Clean, 11 Dirty -
  and with FS Off every FP instruction, fcsr's own accesses among them,
  raises an exception (8.2). The crt of an image built with the F
  extension writes FS = 01 and then clears fcsr, and that write is
  itself a change of FP state: main() finds FS = 11 (measured, and a
  write to fcsr alone measured to move Initial to Dirty). The state is
  only on or off to a program with no context switch. The unit RECORDS
  its exceptions in fcsr's five sticky flags and never traps on them,
  and the rounding mode in frm is honoured (measured, below).
- **The hardware prologue saves integer registers and nothing else**
  (3.4, note 3): the sixteen above, never an f-register. WCH's compiler,
  under the fast attribute and the ilp32f ABI, stores in a handler's own
  prologue every caller-saved f-register the handler clobbers, and ALL
  TWENTY of them (ft0..ft11, fa0..fa7) once it calls a function -
  without looking at the callee, a visible float-free one included - 80
  bytes of user stack and forty memory accesses, placed at the entry
  ahead of the branch that calls or shrink-wrapped onto it as the
  function's shape decides. So in an image built with F a vector is not
  an attributed handler but a NAKED TRAMPOLINE, a call of an ordinary
  function and MRET (Types and verbs, below): the body saves what the
  ABI asks of a callee and no caller-saved register, and since nothing
  then saves the f-registers of the program it interrupts, the
  stratum's check walks every trampoline's body in every linked image
  and fails on a floating-point instruction it reaches. Measured on the
  CH32V303VCT6, a trampoline against the attributed handler in images
  built from one source: a receive vector that posts, bound as the
  console binds its own, 65 cycles a byte against 105 (the twenty saves
  gone, the kernel's `post` reached by a tail jump), about 35 cycles
  less a transmit interrupt; a handler that calls out 65 cycles of
  round trip against 100 (letter `l`); a leaf, which saved nothing
  either way, 8 cycles more for the call and its return (letter `e`);
  and the I2C suite's refused last byte at 8 MHz and 400 kHz, lost with
  the saves and kept without them ([i2c.md](i2c.md)). Over the 24
  images of the part, 68 handlers that saved f-registers in software -
  66 of them all twenty - became one, the float trampoline of letters
  `k` and `l`, and the images' flash 4552 bytes less.

## Types and verbs

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
manual's ISR bank is the ENABLE status and its IPR the pending one;
IENR/IRER/IPSR/IPRR are write-one). `BRIO_CH32_VECTOR(name) { body }`
is the one spelling of a vector binding, and it follows two facts of
the build. Without the F extension (every CH32V203 image) it is an
attributed handler under the name - WCH's fast attribute when the image
is built with `CH32VX03_HPE` (the default), gcc's plain `interrupt`
otherwise, one option for the whole image because a fast handler under
an HPE that is off corrupts the program it interrupted - and the
attribute carries `no_icf`: gcc's identical code folding turns the
second of two handlers of one body into a handler that CALLS the first,
which ends in MRET, so the call never returns and the caller's frame
stays on the stack; on the V4F that frame is the twenty f-registers
such a caller saves, eighty bytes (measured on the CH32V303VCT6: the
interrupted program's next return jumped to a saved float). With F and
the hardware prologue (the CH32V303's images) it is the naked
trampoline: `name` is two instructions - a `jal` to an ordinary
`extern "C"` function `name_body` holding the braces (two bytes
compressed where the body lies within its reach, four otherwise) and
MRET - and the body ends in `ret`; two bodies that fold are ordinary
functions, for which a fold returns where it should. With F
and the hardware prologue off it stays the attributed handler of the
plain `interrupt`, whose software prologue saves whatever a call may
clobber, f-registers included. `BRIO_CH32_VECTOR_FLOAT(name)` is the
binding of a body that does float work: the same trampoline with the
twenty caller-saved f-registers stored on the user stack around the
call - the set gcc saves for a handler that calls out, so fcsr is not
among them, and a body that changes the rounding mode or raises a flag
leaves it so for the program it interrupted - and where there is no F
or no hardware prologue it is the plain binding. THE TRAMPOLINE'S
PROMISE IS A CHECK: `brio check ch32vx03`, after linking every image,
runs `cli/vector_guard.py` over the F images - from each vector table
entry that is a plain trampoline it walks the body's calls, tail jumps
and branches through the listing and fails on a floating-point
instruction, an access to fcsr, frm or fflags, an indirect call or jump
it cannot resolve (a switch's jump table it reads from the image, every
entry required inside the function) or an undecoded four-byte
instruction, naming the image, the vector and the chain of functions;
its fixtures under `test/family_ch32vx03/guard/` are linked and must be
let through or refused by name, the refusals a float operation reached
through a call and a call through a pointer.
Interrupt nesting is never enabled: the kernel's rule
([../design/kernel.md](../design/kernel.md), section 1).
`stack_untouched()` is the RAM ledger: the crt paints the free RAM
between the last section the linker placed and the stack top with one
pattern before the first call, and the verb walks up from that floor
until the pattern breaks - how many bytes of stack no call has reached
since reset.

The timebase is [brio/ch32vx03/ticker.hpp](../../brio/ch32vx03/ticker.hpp)'s
`BasicTicker<tps>` over the STK (`Ticker` at 1000 Hz; `init`, `tick`,
`ticks/millis/secs/now`, `pause/resume`, `advance` for a site that
froze time, `rebase` for a clock that changed rate), and the busy-wait
[brio/ch32vx03/delay.hpp](../../brio/ch32vx03/delay.hpp)'s `delay_us`:
at least, never early, no division at wait time, and REFUSED - with no
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
attribute, and hands BOTH trap entries the fault body:

```cpp
using P = brio::Ch32vx03Platform<>;

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
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
  1005 us for two, three runs alike - at 24 cycles from the edge to the
  handler, 76 to the caller's loop and 29 for an `idle()` the latch
  returns at once (21 and 75 with the tick bound as an attributed
  handler, the trampoline's call the difference). The loop and `idle()`
  are the CH32V203C8's instructions one for one in the two images, and
  no f-register is touched on the way: neither the sleep path nor the
  loop uses one, and the tick's body uses none (the stratum's check
  says so of every image). The one cycle more to the handler and eight
  more to the loop than the V4B's are the trampoline's call and the
  core's or the image's layout, the rest not chased. The handler now runs straight out of the sleep, by
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
  quiet turn 140 cycles.
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
  | HPE on (`CH32VX03_HPE=ON`, the default) | 16 | 12 | 25 | **53** | 12500 B | - |
  | HPE off (gcc's own prologue) | 19 | 12 | 32 | **63** | 12652 B | 64 B more |

  Ten cycles on a minimal handler - 69 ns at 144 MHz - 152 bytes of
  flash over this image's five handlers, and SIXTY-FOUR BYTES OF USER
  STACK that the hardware stack carries instead (`stack_untouched()`
  after the same letter, 19540 against 19476). That last number is what
  separates this core from the CH32V00x's, where the same feature moves
  the pushes from the handler's code to the hardware but leaves them on
  the user stack; and 53 cycles against that family's 83 is what a
  bigger core does with the same measurement. Free in semantics, so it
  is the default. The CH32V303VCT6's V4F, with the hardware prologue
  and the handler a trampoline: entry 18, body 12, exit 30, a whole
  trip of 60 to 70 cycles over 64 rounds - against entry 15, exit 25
  and 52 to 59 for the same handler bound as an attributed leaf, the
  three cycles of the call on the way in and five of the body's return
  on the way out being the trampoline's price where nothing is saved.
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
  20378 interrupts over 56 ms - leave every one of the twenty
  accumulators exact to the bit and every handler's own sum right. The
  float trampoline's twenty stores and loads, with what the body saves
  of the callee-saved ones, are all there is, and they are enough.
- **What an interrupt pays for floating point** (letter `l`): letter
  `e`'s method on three lines raised by hand, best of 32 rounds each - a
  plain trampoline whose body touches no f-register takes 60 cycles of
  round trip (entry 16, body 12); the float trampoline with a float
  multiply and a multiply-add in its body 111 (entry 41, body 26: the
  twenty stores ahead of the body, the twenty loads after it); and a
  plain trampoline whose body calls a function the compiler cannot see
  into 65 (entry 17, body 21), the call costing no f-register. Bound as
  attributed handlers in the same session the three took 54, 67 (the
  compiler saving the f-registers the body used) and 100 (all
  twenty, for the call): the float trampoline's set is the price of a
  body that does float work, and the plain trampoline makes calling out
  of a handler cost the call alone.
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

## Not covered yet

Driver gaps, each with its reason:

- The PFIC's priority threshold, its nesting depth and its four free
  vectored entries (VTFADDRR): an ISR body runs to completion on every
  target ([../design/kernel.md](../design/kernel.md)), so nesting has
  no user here, and the vectored entries arrive with one.
- The second route to the same reset, PFIC_SCTLR bit 31 with no key,
  and that register's SLEEPONEXIT: one way in is enough for a reboot,
  and SLEEPONEXIT has no user - a brio program sleeps in the kernel
  loop's idle path and not on a handler's exit, and that path clears
  it - while the rest of that register's sleep bits are the power
  chapter's ([sleep.md](sleep.md)).
- mepc and mtval are readable but do not cross a reset: the breadcrumb
  has one byte for the detail, and it carries the cause.

Implemented but not bench-verified, each with what would measure it:

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
- **Letter `g` timed on the CH32V303VCT6.** The console's host-timed
  bracket through that board's probe carries the probe's block
  forwarding; the rate of that die's HSI is the clock suite's
  measurement ([clock.md](clock.md)), and this letter's own number
  there would want a probe that forwards as bytes come.
