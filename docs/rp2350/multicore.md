# Multicore (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 3.1.2 (SIO's
CPUID, and MHARTID answering the same), 3.1.5 (the inter-processor FIFOs
and their interrupt), 3.1.6 (the doorbells: eight flags each way behind
one core-local line), 3.1.8 (the platform timer, one counter and a
comparator per core), 3.3 and 3.4 (the two architectures' event signals:
`sev`/`wfe` and `h3.unblock`/`h3.block`), 3.8.6.3 (where those two
Hazard3 instructions are encoded), 3.9.2 (mixed architecture
combinations), 5.2 and its table 451 (core 1's wait state in the ROM),
5.3 (the launch protocol), 7.4 (the power-on state machine, whose
FRCE_OFF.PROC1 holds core 1 in reset). Appendix E: RP2350-E2 (writes to
SIO registers at 0x180 and above release the spinlock 128 bytes below)
and RP2350-E19 (a reboot hangs with any FRCE_OFF bit but PROC1 set). The
drivers: `brio/rp2350/multicore.hpp` (the doorbell, the mailbox, the
launch) and `brio/rp2350/platform.hpp` (one platform type per core) over
`brio/util/inbox.hpp`, the bridge of design/kernel.md section 12; the
tickers are `brio/rp2350/ticker.hpp`'s, one per core. The reference
suite: `test_rp2350_multicore`.

## What the silicon does

FOUR PROCESSOR SOCKETS AND TWO CORES: a pair of Cortex-M33 and a pair of
Hazard3, of which one pair runs (3.9). Everything below is the same on
both pairs, and that is a fact of this chip rather than a convenience:
the system interrupt numbers are shared (3.8.4.2), the SIO is the SIO
whichever core reads it, and the ROM's launch protocol is one protocol.
Mixed pairs are legal - Arm core 0 with a RISC-V core 1 or the other way
round - but need two program images, which is why nothing here offers
them.

What is doubled is what belongs to a core: its interrupt controller
(every system line reaches both, and each core enables what it wants),
its timebase (two SysTicks on the Arm pair; on the RISC-V pair one shared
64-bit microsecond counter with a comparator per core, 3.1.8), its global
mask, its stack, its debug logic - and, on the Arm pair, its VTOR, its
MSPLIM and its CPACR. SIO's CPUID reads 0 or 1 on the core that reads it.

TWO CROSS-CORE CHANNELS, and the datasheet says plainly what each is for:

- **The mailbox FIFOs** (3.1.5), one per direction, "for cross-core
  events whose count and order is important". Each core writes FIFO_WR
  and reads FIFO_RD; FIFO_ST carries a room bit, a data bit and two
  sticky misuse flags (a read of an empty FIFO, a write to a full one).
  One interrupt line per core, SIO_IRQ_FIFO, number 25. THE DEPTH IS
  STATED TWICE AND DIFFERENTLY: 3.1.5 says four entries, the FIFO_ST
  register description says eight - which is why the driver states no
  depth and the suite measures one.
- **The doorbells** (3.1.6), eight flags in each direction, "for events
  which are accumulative (i.e. may post multiple times, but only answered
  once) and which can be responded to in any order". DOORBELL_OUT_SET
  raises a flag on the OPPOSITE core, DOORBELL_IN_SET on this one,
  DOORBELL_IN_CLR acknowledges and reads back what stands; the interrupt
  stands while any flag does. The line is SIO_IRQ_BELL, number 26, and it
  is CORE-LOCAL: the same number on each core, notifying the core that
  reads it.

Out of reset core 1 goes to the ROM's wait state (5.2, table 451): it
waits for its RCP salt, drains its FIFO, pushes a 0, and waits for the
six-word sequence of 5.3 - 0, 0, 1, a vector base, a stack pointer, an
entry point, each word echoed back and the sequence restarted from the
top on any wrong word - then "set Secure main sp and VTOR, then jump into
the entry point provided". The fourth word means VTOR on the Arm pair and
mtvec on the RISC-V one; the sixth carries the Thumb bit on the Arm pair.
A 0 must be preceded by a drain of the sender's own inbound FIFO and by
an EVENT, because core 1 may be asleep waiting for FIFO room - `sev`
there, `h3.unblock` here, and both are sticky, so an event posted before
the sleep makes the sleep fall through.

What the ROM does NOT hand over is the rest of a core's private state:
the global pointer on the RISC-V pair (a register the crt sets once, on
core 0), MSPLIM and CPACR on the Arm one. Nothing else of the image is
per core: one vector table, one flash, one SRAM, one clock tree.

FRCE_OFF.PROC1 (7.4) is the one stage of the power-on sequence a program
may hold without stopping itself, and holding then releasing it is what
puts core 1 back in the ROM's wait loop. RP2350-E19 - a reboot hangs with
any other FRCE_OFF bit set - therefore costs this chapter nothing: PROC1
is the only bit it touches, and the watchdog's reboot clears the rest
before it triggers ([watchdog.md](watchdog.md)).

RP2350-E2 IS LIVE AND HARMLESS HERE. A write to an SIO register at offset
0x180 or above is also decoded as a write to the spinlock 128 bytes
below, releasing that lock: the four doorbell registers alias SPINLOCK0
to SPINLOCK3, and the platform timer's registers alias others. brio takes
no SIO spinlock on this chip - the kernel's exclusion is per core and the
bridge needs none - so nothing of this stratum can be the victim; a
program that does take one reads the erratum's own list of the locks that
stay safe.

### What differs from the RP2040

- The RP2040 had no doorbell registers, so its bell was the mailbox
  FIFO. Here the bell is the doorbell and the mailbox is the launch's
  alone, which is a better division in three ways: a ring cannot fail for
  want of room (a flag that stands is already a bell, where a write to a
  full FIFO was a lost bell and a sticky error), a ring is correct from
  either core (OUT_SET or IN_SET by CPUID), and a launch needs nothing
  masked - on the RP2040 the doorbell interrupt had to be off while the
  protocol ran, because its handler would have eaten the echoes.
- The doorbell line has ONE number for both cores, where the RP2040's two
  FIFO lines had two (SIO_IRQ_PROC0 and SIO_IRQ_PROC1). Enabling it
  enables the bell of the core that runs the verb, so the verb refuses on
  any other core rather than arm the wrong one.
- The RP2040's memory-mapped divider is gone (3.1.7) and the doorbells,
  the RISC-V soft interrupt and the platform timer stand in that address
  space instead - which is what RP2350-E2 is about.
- The mailbox FIFO is FOUR words deep here against the RP2040's eight -
  measured, which also settles the two places this datasheet states it
  differently: 3.1.5 is right and the FIFO_ST register description's
  "8 words deep" is not.

## Types and verbs

- `Rp2350Platform<core, TB = CoreTicker<core>>`: one platform type per
  core, the kernel's statics keyed by it - `core`, `Timebase`,
  `Doorbell` (`SioDoorbell<core>`), `on_own_core()` (CPUID against
  `core`), and one panic breadcrumb per core; `Rp2350Platform<>` is core
  0's.
- `CoreTicker<core>` ([platform.md](platform.md), `rp2350/ticker.hpp`):
  that core's timebase at 1000 Hz - its own SysTick on the Arm half, its
  own comparator against the shared microsecond counter on the RISC-V
  one; `Ticker` is core 0's. The app's one `isr_systick` binding ticks
  the ticker of the core that took it.
- `SioDoorbell<core>`: `flags` (all eight) and `bell` (the one a ring
  raises); `ring()` from EITHER core - the opposite core's flags or this
  core's own, chosen by CPUID, followed by the architecture's event;
  `pop_all()` on this core, which acknowledges what stands and returns
  it; `pending()`, answerable from either core; `enable()` / `disable()`,
  which refuse unless the calling core IS `core`; `irq()`.
- `SioMailbox`: the FIFO as a channel of a program's own, since the
  bridge does not use it - `writable()`, `readable()`, `push()` (which
  REFUSES on a full FIFO rather than leave the sticky flag a bare store
  would), `pop()`, `drain()`, `read_on_empty()`, `write_on_full()`,
  `clear_errors()`, `irq()`.
- `core_send_event()`: the architecture's cross-core event instruction,
  `sev` or `h3.unblock` - the launch protocol's only caller.
- `Core1`: `launch(entry, stack_top = default_stack_top())` - core 1
  reset into the ROM, then the protocol; `entry` runs on core 1 on the
  stack the linker script reserves in the unstriped bank scratch_x, with
  this core's vector base. `protocol(entry, stack_top)` alone, for a core
  1 known to be waiting - it refuses while this core's SIO_IRQ_FIFO is
  enabled, whose handler would eat the echoes, and does NOT care whether
  the bell is enabled. `reset()` - core 1 into the ROM through
  FRCE_OFF.PROC1, its "drained" word popped. `entered()` - the entry
  shim's own word, true once core 1 has arrived. `trace` / `trace_count`
  - what core 1 answered, for a suite's eyes. Every wait bounded, false
  when unanswered.
- `core1_trampoline`, `Core1Frame`, `Core1::start`, `core_global_pointer()`,
  `core_vector_base()`: the entry shim in the open - the naked, relocation-
  free first instructions core 1 runs, the four words core 0 leaves on its
  stack for them, and the ordinary function they jump to, which adopts the
  vector base, sets the per-core state the ROM did not, and calls the
  program's entry. A program names none of them; a document and a
  debugger do.
- From `util/inbox.hpp`: `send<Ao>(ev)`, `Inbox<Ao>` (`overflows`,
  `pending`, `capacity`, `clear`), `Inboxes<Aos...>` (`isr()`,
  `enable()`, `idle()`), `send_reply_to<Ao, Payload>()`; from the kernel,
  the queue's `misposts()`.

## How to use it

Core 1's program: its ticker, its bell, its kernel. The source is the
same for both architectures, the vector names included.

```cpp
using P0 = brio::Rp2350Platform<0>;
using P1 = brio::Rp2350Platform<1>;

struct Echo : brio::Fsm<Echo, Ping> {
    static inline brio::EventQueue<Event, 16, P1> queue;   // core 1's
    static constexpr uint8_t inbox_depth = 16;
    ...
        [](Ping p) { brio::send<Origin>(Pong{p.n}); return handled(); }
};

extern "C" void isr_systick() {
    if (brio::core_id() == 0) brio::CoreTicker<0>::tick();
    else                      brio::CoreTicker<1>::tick();
}
// ONE bell vector for both cores: the line is core-local and carries one
// number, so the core that took it decides which drain runs.
extern "C" void isr_sio_bell() {
    if (brio::core_id() == 0) brio::Inboxes<Origin>::isr();
    else                      brio::Inboxes<Echo>::isr();
}

[[noreturn]] void core1_main() {
    brio::CoreTicker<1>::init(clock);
    brio::Inboxes<Echo>::enable();
    brio::enable_interrupts();
    brio::Tenuto<P1, Echo>::run();
}

int main() {
    SysClock::init();                          // core 0 owns the clock tree
    brio::Ticker::init(clock);
    const bool up = brio::Core1::launch(&core1_main);
    brio::Inboxes<Origin>::enable();
    brio::enable_interrupts();
    brio::send<Echo>(Ping{1});                 // crosses; post<Echo> would be refused
    brio::Tenuto<P0, Origin>::run();
}
```

A panic on either core can reboot the chip through `ResetReporter`
([reset.md](reset.md)), and core 0 reads both breadcrumbs at the next
boot: `take_panic_record<P0>()` and `take_panic_record<P1>()`. A program
that leaves the fault vectors to the crt's weak spin instead gets the
other outcome - core 1 stops and core 0 goes on - which is what the
reference suite wants.

The mailbox, if a program wants an ordered channel of its own:

```cpp
brio::SioMailbox::push(word);                  // false if there is no room
uint32_t in = 0;
while (brio::SioMailbox::pop(in)) { /* ... */ }
```

Nothing in brio enables SIO_IRQ_FIFO, and a launch refuses while it is
on. A program that enables it owns the launch's channel and must disable
it around `Core1::launch()`.

## Bench findings

All of them on an RP2350 in the QFN-80 package, **stepping A2**, at
3.3 V, clk_sys on the PLL at 150 MHz, the ruler the SIO's platform timer,
and all of them on BOTH architectures: `test_rp2350_multicore` reports
**39 pass, 0 fail** on the Cortex-M33 pair and **39 pass, 0 fail** on the
Hazard3 pair, from one source. Core 1 has no console: every fact below
about it was carried by an event that crossed.

- **The launch is microseconds.** From a core 1 left in the ROM by the
  probe's `reset run`, `Core1::launch()` - the power-on state machine's
  hold and release, then the six-word protocol - completes in **34 to
  35 us** on the Cortex-M33 pair and **25 us** on the Hazard3 one; a
  RELAUNCH of a core 1 already known to the program takes 5 to 16 us.
  Core 1's entry writes CPUID 1, and the vector base it adopted is core
  0's: 0x1000_0000 on the Arm half (VTOR) and 0x1000_0041 on the RISC-V
  one (mtvec, mode bit and all).
- **AFTER THE FLASH VERB, CORE 1 IS IN THE BOOTROM.** Read through the
  debug port with no program's help, core 1 sits at 0x0000_00DA on the
  Arm pair and 0x0000_76DE on the RISC-V one, both inside the ROM, with
  its stack pointer at its reset value - the wait state of 5.2's table
  451, reached because the chip-level rescue reset precedes the
  programming. That is the state a launch has to work from, and it does,
  flash after flash.
- **A RUNNING core 1 does not answer the protocol.** With no reset
  first, `protocol()` gets no echo at all and gives up after its bounded
  wait - **53396 us** on the Arm half, 53419 us on the RISC-V one - and
  core 1's entry stamp and run count are unchanged, so nothing of its
  life was disturbed. This is why `launch()` resets first.
- **Two tickers, and they are two.** Over 500 ms of the ruler each
  advances 500 ticks on both halves. With core 0's PAUSED for 100 ms it
  advances 0 and core 1's advances 100; resumed, core 0's takes up again
  at 20 ticks in 20 ms. On the Arm pair those are two SysTicks counting
  each core's own clk_sys; on the RISC-V pair two comparators against
  the ONE shared microsecond counter - and the pause is still per core,
  because what it masks is that core's own enable.
- **A crossing and its return cost single-digit microseconds**: 64 Pings
  one at a time come back as 64 Pongs in order, round trip **min 3, mean
  4, max 17 us** on the Cortex-M33 half and **min 4, mean 4, max 14 us**
  on the Hazard3 one - the maximum being the first, with both caches
  cold.
- **A burst finds the QUEUE before it finds the inbox.** 200 Pings
  written back to back into an inbox of 16 are ALL accepted on both
  halves: the bell's handler on core 1 drains the ring faster than core 0
  can fill it. What overflows is Echo's own event queue, 32 deep -
  168 Pings dropped and counted there, 32 dispatched and 32 Pongs back
  in order. The account balances exactly, which is the point: every loss
  is counted where it happened.
- **Both directions at once, paced, lose nothing.** One second with a
  Ping every 20 us while core 1's metronome runs on its own time events:
  **50000 Pings, 50000 Pongs in order**, and 999 to 1000 Ticks across the
  bridge on either half - no inbox overflow, no queue overflow, no
  mispost, in either direction.
- **A `post` to the other core's queue is refused and counted**, and the
  AO never sees it: the next Ping's Pong is the first.
- **Core 1 dies alone.** A panic on core 1 writes CORE 1'S breadcrumb -
  code 2, context 0x11 - which core 0 reads through
  `take_panic_record<P1>()`, and core 0's own record stays untouched. A
  halted core 1 then answers no Ping. `Core1::reset()` puts it back in
  the ROM in **3 to 6 us** (Arm) and **2 us** (RISC-V), leaves FRCE_OFF
  holding nothing at all - which is what erratum RP2350-E19 wants of a
  reboot - and the relaunch answers Pings again.
- **Under saturation the two accounts balance.** 40000 Pings offered
  flat out take 53370 us on the Arm half and 48977 us on the RISC-V one;
  all 40000 are accepted by the inbox, Echo dispatches **14920** (Arm)
  and **9172** (RISC-V), the rest - 25080 and 30828 - are dropped at
  Echo's queue and counted there, and every Pong comes back. The run is
  bounded by a COUNT and not by a time because an overflow counter is
  sixteen bits and saturates: at 100 ms flat out the Arm half offers some
  122000 Pings and loses 118807, which 65535 cannot say, and the letter
  now checks that no counter reached that ceiling.
- **The lower-priority AO of core 1 is starved by pack order, visibly.**
  Under that load Beat's metronome gets 78 of its Ticks dispatched and
  39 or 40 dropped at its own queue, on either half - the kernel serving
  Echo first, exactly as `Tenuto<P1, Echo, Beat>` promises.
- **The bell and the mailbox are two channels.** A ring raises exactly
  one of the eight flags and `pop_all()` acknowledges exactly it;
  `SioDoorbell<1>::enable()` refuses from core 0, one line number serving
  both cores; core 1 acknowledges its own bell, so nothing stands in the
  outbound register's read-back after a round trip.
- **THE MAILBOX IS FOUR WORDS DEEP.** It takes four pushes and refuses
  the fifth, with the sticky write-on-full flag still clear - so 3.1.5's
  "four entries deep" is the truth and the FIFO_ST register
  description's "8 words deep" is not.
- **A launch runs with the bell interrupt ENABLED** - 16 us (Arm) and
  12 us (RISC-V) with it on throughout, and core 1 answering afterwards.
  That is the whole gain of the doorbell over the RP2040's FIFO bell.
  The launch still refuses while the MAILBOX interrupt is enabled, whose
  handler would eat the protocol's echoes, and the refusal costs nothing:
  the check is the first thing `protocol()` does.

## Not covered yet

Driver gaps, each with its reason:

- **A mixed architecture pair** (3.9.2: Arm core 0 with a RISC-V core 1,
  or the reverse). It is legal hardware and the cores interoperate, but
  it needs TWO images - two toolchains, two linker scripts, two halves of
  one program that cannot share a translation unit - and this project's
  build has one image per configure. Declined until a program needs it.
- **The RISC-V machine software interrupt** (SIO's RISCV_SOFTIRQ, one bit
  per core, which the RISC-V crt binds as `isr_riscv_softirq`). It would
  be a second doorbell on one half of the chip only; the bell this
  chapter uses is one line, one number and one vector name on both, which
  is worth more than a spare channel. It stays bound and unraised.
- **The SIO spinlocks, the interpolators and the TMDS encoder**: nothing
  in the model wants a lock (the bridge has no shared read-modify-write
  at all), and the rest waits for a program. RP2350-E2 is the reason a
  lock, if one were ever wanted, would have to be chosen by number.
- **Per-core ownership of a peripheral's interrupt beyond rule 2**
  (design/kernel.md section 12): each driver's `init` enabling its line
  on the calling core is the whole mechanism today; a check that the same
  line is not enabled on both cores would need a register the chip does
  not offer.
- **A power model across the cores** - the chip-level sleep needs both
  kernels' consent, a vote through the bridge with the round's deadline
  as the veto. Born with the power chapter (POWMAN), which is not
  written.
- **The bus fabric's contention counters** (BUSCTRL): the measure of two
  cores executing from one XIP cache, born with the first program whose
  timing asks.

Implemented but not bench-verified, each with what would measure it:

- **A `Lease::reply` buffer crossing inside a request and back inside
  the reply.** The fences are the inbox's and the host test covers the
  capsule (`test_inbox`); what is missing is a letter with a
  buffer-carrying request, which wants an AO on core 1 that owns a
  buffer worth lending.
- **A second console for core 1.** The board has one probe and one UART
  bridge, so core 1 speaks only through the bridge here; the chip's own
  USB CDC port is the second console, and a letter that runs one would
  have to give core 1 the USB controller's interrupt line.
- **`SioMailbox` as a program's own channel.** Its depth, its refusal
  and its two sticky flags are measured; what is not is a program using
  it under load beside the bell - two channels carrying traffic at once,
  which wants a program that wants an ordered channel at all.
- **`SioDoorbell::ring()` called FROM CORE 1 towards core 1.** Both
  directions of the CPUID choice are exercised by the bridge - core 0
  ringing core 1 and core 1 ringing core 0 - and core 0 ringing its own
  bell is letter j's; the fourth case, core 1 ringing its own, has no
  caller and would want an AO on core 1 that posts to itself.
