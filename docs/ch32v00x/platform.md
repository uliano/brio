# Platform (CH32V00x)

The kernel's Platform concept realized on the QingKe V2C, in two
halves: the RUNNING half - the critical section, the idle hook, the
STK timebase, the microsecond busy-wait, the interrupt round trip -
and the FAILING half - which reset happened, how to cause one, and the
record a crash leaves for the next boot.

Documents of record: the CH32V00X reference manual V1.5 (ch. 3 RCC for
the reset flags, ch. 6 for the PFIC and the STK), the QingKe V2
microprocessor manual V1.3 (2.2 for what interrupt entry does to
mstatus, 3.4 for the hardware prologue/epilogue, 5.2 for what wakes a
WFI). No errata sheet exists for this family that the desk could find;
the findings below are the entries this document keeps in its place.

## What the silicon does

- **Interrupt entry does not clear MIE** (QingKe V2 manual 2.2), so a
  handler runs with the global enable still set; the PFIC decides
  whether another line may preempt it, and with INTSYSCR.INESTEN off
  it holds the next line until MRET.
- **A WFI sleep ends only when an interrupt is RESPONDED** (5.2) -
  which needs MIE set. Sleeping with MIE clear over a pending interrupt
  is a sleep nobody ends.
- **The core can sleep as a WFE instead**: PFIC_SCTLR.WFITOWFE turns
  the next `wfi` into a wait-for-event, and SEVONPEND makes every
  interrupt entering the pending state an event, LATCHED, so a WFE
  after the event returns at once (3.1's SCTLR).
- **The hardware prologue/epilogue (HPE)** pushes the ten caller-saved
  registers (x1, x5..x7, x10..x15) to the USER STACK on entry - SP
  moves by 48 bytes - and pops them on MRET, two levels deep at most
  (3.4). It is enabled by INTSYSCR.HWSTKEN and used by a handler
  declared with the vendor compiler's `interrupt("WCH-Interrupt-fast")`
  attribute, which then emits no prologue of its own.
- **The STK** is a 32-bit up-counter at HCLK or HCLK/8 with a compare,
  an auto-reload (STRE) and a flag the handler must clear (RM 6.5.4);
  it is interrupt 12 of the vector table, and the interrupt number IS
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
  counter does NOT run unarmed** (measured): 5.2.1's "free operation
  regardless of whether the watchdog function is turned on" does not
  describe this silicon.
- **The two debug freezes** (IWDG_STOP, WWDG_STOP) are bits of a CORE
  CSR, DBGMCU_CR at 0x7C0 (RM ch. 21), not a peripheral register.

## Types and verbs

`Ch32v00xPlatform<TB>` ([brio/ch32v00x/platform.hpp](../../brio/ch32v00x/platform.hpp))
is the concept member for member: `CriticalSection` is pfic.hpp's
`InterruptGuard` (one `csrrci` reads and clears mstatus.MIE, the
destructor restores only what it found set), `idle()` is the WFE
sequence above followed by the unmask, `break_here()` is `ebreak`,
`atomic_width` 4, the breadcrumb in `.noinit` - and with a Standby
armed (SLEEPDEEP set by [sleep.md](sleep.md)'s site) `idle()` holds
the ticker off across the WFE, since a tick turning pending would end
it before the Standby began. The interrupt verbs and
the per-line enables are [brio/ch32v00x/pfic.hpp](../../brio/ch32v00x/pfic.hpp):
`enable/disable/enabled/pending/set_pending/clear_pending/active`
(the manual's ISR bank is the ENABLE status and its IPR the pending
one; IENR/IRER/IPSR/IPRR are write-one). `BRIO_CH32_INTERRUPT` is the
one spelling of the handler attribute, expanding to WCH's fast
attribute when the image is built with `CH32V00X_HPE` (the default)
and to gcc's plain `interrupt` otherwise - one option for the whole
image, because a fast handler under an HPE that is off corrupts the
program it interrupted. Interrupt nesting is never enabled: the
kernel's rule ([../design/kernel.md](../design/kernel.md), section 1).

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
lsi_hz)` at a STATED LSI rate) and `Wwdg` (`configure(WwdgConfig)`
refusing a window below 0x40, `start(t)`, `refresh(t)` with T6 always
written set, `force_reset()`, `counter()`, the EWIF verbs and `isr()`,
`wwdg_timeout_us(hclk, prescaler, t)`), each with its `debug_freeze()`
over the CSR.

## How to use it

An app names the platform, binds the vectors it owns with the one
attribute, and hands the fault vector the body:

```cpp
using P = brio::Ch32v00xPlatform<>;

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void fault_handler() { brio::fault_reset<P>(); }

int main() {
    const auto record = brio::take_panic_record<P>();   // first, and once
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Kernel<P, ...>::run();
}
```

A wait shorter than a tick is `delay_us(clock, us)`; a wait of a tick
or more is a TimeEvent and `delay_us` refuses it. A reboot on purpose
is `Reset::software()`; a panic that must be seen at the next boot
goes through `panic<P, ResetReporter>(code, context)`.

## Bench findings

The reference suite is `test_ch32_platform` (thirty verdicts in `z`,
nine more in letter `i`, which reboots the board three times), on the
CH32V006K8U6 at 48 MHz. What it measured:

- **The idle hook wakes.** With the console silent, two `idle()` calls
  covered the 635 us to the next tick and returned with interrupts
  enabled; a 5 ms window with interrupts masked advanced the tick by
  one (the STK interrupt is a pending bit: one tick delivered, the
  rest coalesced).
- **The STK arithmetic holds.** CMP = 47999 as programmed; over 200
  reloads the CNT-delta accumulation `delay_us` is built on tracked
  the interrupt count to 622 cycles of error out of 9.6 million
  (65 ppm, and under one period). `delay_us(100)` spent 4871 cycles
  for 4800 asked; twenty `delay_us(500)` took exactly 10 ticks; 1000
  us and above are refused.
- **The interrupt round trip, in HCLK cycles**, a software interrupt
  raised by hand with the cycle counter read at the raise, at the
  handler's first and last statements and when the raiser sees it
  done, 64 rounds each, every round identical:

  | image | entry | body | exit | whole trip |
  |---|---|---|---|---|
  | HPE on (`CH32V00X_HPE=ON`, the default) | 29 | 23 | 31 | **83** |
  | HPE off (gcc's own prologue) | 35 | 24 | 33 | **92** |

  Nine cycles on a minimal handler - 0.19 us at 48 MHz - is what the
  hardware prologue is worth in latency: the ten registers go to the
  stack either way, and what the hardware saves is the fetch of the
  prologue and epilogue, which for a leaf handler gcc keeps small at
  -Os. On a handler that calls into the kernel (the console's USART
  handler saves twelve registers) the code saved is 48 bytes per
  handler and the cycles proportionally more. Free in semantics, so it
  is the default; what it costs is the vendor attribute, available
  from WCH's gcc or from an upstream gcc with the fast-interrupt patch.
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

The watchdogs' suite is `test_ch32_watchdog` (15 verdicts in `z`, two
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

## Not covered yet

Driver gaps, each with its reason:

- The PFIC's priorities and its two free vectored entries: no user.
- The hardware-enabled IWDG (the IWDG_SW option byte): an option byte
  write is the flash chapter's, and the bench part keeps the factory
  option bytes.
- The watchdogs' debug freeze as a measurement: what a halted core
  does to a running dog is a debugger session, not a suite letter.

Implemented but not bench-verified, each with what would measure it:

- The idle hook's POWER: `idle()` is proven to sleep and wake, not to
  sleep cheaply - whether the latched event is consumed by the `wfi`
  or leaves the loop spinning is a current measurement with the probe
  detached (a core in debug mode never sleeps, QingKe V2 manual 5.1).
  The Standby it enters when [sleep.md](sleep.md)'s site has armed one
  is measured the same way.
- The HPE's saving on a handler that calls into the kernel: the
  minimal handler was measured; a letter timing the USART handler's
  round trip both ways would put a number on the larger case.
