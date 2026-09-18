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
- The mailbox FIFO's documented depth differs from the RP2040's eight
  in one of the two places this datasheet states it.

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

Implemented but not bench-verified, each with the letter of
`test_rp2350_multicore` that will measure it:

- The launch itself: core 1 reset into the ROM and the six-word protocol
  answered, its entry shim reached (`entered()`), its CPUID and vector
  base written from core 1, and the time all of it took - on both
  architectures, where the Arm half's shim also has MSPLIM and the FPU to
  set and the RISC-V half's has the global pointer (letter a).
- The protocol alone at a RUNNING core 1, refused within its bounded wait
  (letter a).
- Two tickers side by side at 1000 Hz against the shared microsecond
  ruler - two SysTicks on one half, two comparators against one counter
  on the other (letter b).
- A crossing and its return, sixty-four times one at a time, with the
  round trip measured; and the same in both directions at once, core 1's
  time events feeding the bridge while core 0 pumps (letters c and e).
- The inbox's accounting: a burst past its depth counted by the sender,
  and under saturation every loss counted where it happened, with the two
  accounts balancing (letters d and h).
- A `post` to the other core's queue refused and counted as a mispost
  (letter f).
- A panic on core 1 writing core 1's breadcrumb, a halted core 1
  answering nothing, `Core1::reset()` putting it back into the ROM with
  FRCE_OFF left clear, and a relaunch answering as before (letter g).
- THE BELL AND THE MAILBOX AS TWO CHANNELS (letter j): a ring raising
  exactly one flag and `pop_all()` acknowledging exactly it, the other
  core's bell read through the outbound register's read-back, the
  mailbox's DEPTH MEASURED against the datasheet's two different
  statements of it, `push()` refusing instead of leaving the sticky
  write-on-full flag, the launch refused while the mailbox interrupt is
  enabled, and a whole relaunch with the bell interrupt left ON.
- A `Lease::reply` buffer crossing inside a request and back inside the
  reply: no letter yet - the fences are the inbox's and the host test
  covers the capsule (`test_inbox`), so what is missing is a letter with
  a buffer-carrying request.
- A second console for core 1. The board has one probe and one UART
  bridge, so core 1 speaks only through the bridge here; the chip's own
  USB CDC port is the second console, and that chapter is not written.
