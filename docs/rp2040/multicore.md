# Multicore (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 2.3.1 (the
SIO: CPUID, the inter-core FIFOs and their interrupts, the spinlocks),
2.3.2 (the two NVICs, every line reaching both), 2.4.2 (what
SYSRESETREQ resets), 2.8.2 (the bootrom's launch protocol for core
1), 2.13 (the power-on state machine, whose FRCE_OFF resets one
core). The drivers: `brio/rp2040/multicore.hpp` (the doorbell, the
launch), `brio/rp2040/platform.hpp` (one platform type per core),
`brio/rp2040/ticker.hpp` (one ticker per core) over
`brio/util/inbox.hpp`, the bridge of design/kernel.md section 12. The
reference suite: `test_rp2040_multicore`.

## What the silicon does

Two Cortex-M0+ cores with one bus fabric, one flash and one SRAM
between them, and everything else doubled: each core has its own
NVIC, its own SysTick, its own PRIMASK, its own debug logic. Every
one of the 32 interrupt lines reaches BOTH NVICs and each core enables
what it wants. SIO's CPUID reads 0 or 1 on the core that reads it; a
pair of eight-word FIFOs, one per direction, links the cores, with
one interrupt line per core (SIO_IRQ_PROC0, SIO_IRQ_PROC1) raised
while that core's inbound FIFO holds a word; 32 hardware spinlocks
exist for a program that wants them, and this model wants none.

Out of reset core 1 runs the bootrom's wait loop: it drains its FIFO,
pushes a 0, and waits for the launch sequence - 0, 0, 1, a vector
table, a stack pointer, an entry point, each word echoed back, the
sequence restarted on any wrong word - then loads VTOR and SP and
calls the entry. SYSRESETREQ, the Cortex-M's own request, "only
resets the Cortex-M0+ processor core" (2.4.2) - the requesting core,
not the other one and not the debug logic; only the power-on state
machine resets a core whole (FRCE_OFF, per core), and only the
watchdog's path reboots the chip. The system timer's DBGPAUSE bits,
set at reset, stop the timer while EITHER core is halted by a
debugger.

## Types and verbs

- `Rp2040Platform<core, TB = CoreTicker<core>>`: one platform type per
  core, the kernel's statics keyed by it - `core`, `Timebase`,
  `Doorbell` (SioDoorbell<core>), `on_own_core()` (CPUID against
  `core`), and one panic breadcrumb per core; `Rp2040Platform<>` is
  core 0's.
- `CoreTicker<core>` (rp2040/ticker.hpp): the core's SysTick ticker at
  1000 Hz; `Ticker` is core 0's. The app's one SysTick vector ticks
  the ticker of the core that took it.
- `SioDoorbell<core>`: `ring()` from the other core (a word into the
  FIFO if it has room, else a bell is pending already), `pop_all()`,
  `pending()`, `enable()` / `disable()` (the line in the calling core's
  NVIC), `irq()`.
- `Core1`: `launch(entry, stack_top = default_stack_top())` - core 1
  reset into the bootrom, then the protocol: `entry` runs on core 1 on
  the stack the linker script reserves in the unstriped bank SRAM4,
  with core 0's vector table; `protocol(entry, stack_top)` alone, for
  a core 1 known to be waiting; `reset()` - core 1 into the bootrom
  through FRCE_OFF, its "drained" word popped; every wait bounded,
  false when unanswered. Core 0's doorbell interrupt is disabled
  around either.
- From `util/inbox.hpp`: `send<Ao>(ev)`, `Inbox<Ao>` (`overflows`,
  `pending`, `capacity`, `clear`), `Inboxes<Aos...>` (`isr()`,
  `enable()`, `idle()`), `send_reply_to<Ao, Payload>()`; from the
  kernel, the queue's `misposts()`.

## How to use it

Core 1's program: its ticker, its doorbell, its kernel.

```cpp
using P0 = brio::Rp2040Platform<0>;
using P1 = brio::Rp2040Platform<1>;

struct Echo : brio::Fsm<Echo, Ping> {
    static inline brio::EventQueue<Event, 16, P1> queue;   // core 1's
    static constexpr uint8_t inbox_depth = 16;
    ...
        [](Ping p) { brio::send<Origin>(Pong{p.n}); return handled(); }
};

extern "C" void isr_systick() {
    if (P0::core_id() == 0) brio::CoreTicker<0>::tick(); else brio::CoreTicker<1>::tick();
}
extern "C" void isr_sio_proc0() { brio::Inboxes<Origin>::isr(); }
extern "C" void isr_sio_proc1() { brio::Inboxes<Echo>::isr(); }

[[noreturn]] void core1_main() {
    brio::CoreTicker<1>::init(clock);
    brio::Inboxes<Echo>::enable();
    brio::enable_interrupts();
    brio::Kernel<P1, Echo>::run();
}

int main() {
    SysClock::init();                         // core 0 owns the clock tree
    brio::Ticker::init(clock);
    const bool up = brio::Core1::launch(&core1_main);
    brio::Inboxes<Origin>::enable();
    brio::enable_interrupts();
    brio::send<Echo>(Ping{1});                // crosses; post<Echo> would be refused
    brio::Kernel<P0, Origin>::run();
}
```

A panic on either core reboots the chip through `ResetReporter`
(rp2040/reset.hpp), and core 0 reads both breadcrumbs at the next
boot: `take_panic_record<P0>()` and `take_panic_record<P1>()`.

## Bench findings

All from `test_rp2040_multicore`, green on the Pico and the WeAct
board, core 0 running the console and a kernel stepped by hand, core
1 a kernel of its own (an echo and a metronome on its own time
events), every fact about core 1 carried by an event that crossed:

- The launch - core 1 reset into the bootrom and the six-word
  protocol - takes 64 to 71 us; the reset alone 3 to 11 us; core 1's
  entry runs 15 us after the protocol's last word. The protocol
  alone at a RUNNING core 1 is refused after its bounded wait (72
  ms): only the bootrom's loop answers it, and only a reset puts core
  1 there.
- Core 1's ticker runs at 1000 Hz on its own SysTick beside core 0's,
  each on its own counter (500 and 500 ticks in 500 ms of the timer).
- A crossing and its return: 10 us, 13 at most (64 Pings one at a
  time, each Pong back in order); the first of a cold program costs
  the cache fill on both cores (163 us measured once).
- Fifty thousand Pings a second for a second, every Pong back in
  order, while core 1's metronome delivered its thousand Ticks across
  the bridge on its own time events: no overflow anywhere, no mispost.
- A burst of 200 into an inbox of 16: what the inbox refused the
  sender counted; what core 1's queue refused, core 1's queue counted
  - and the account balanced to the event: accepted = Pongs back +
  losses named. Flat out for 100 ms (155 thousand crossings a second
  each way) the same two accounts balance, and core 1's
  lower-priority AO starves by pack order as the model says it must -
  its queue's overflow counter names it.
- A `post<Echo>()` from core 0 to core 1's queue is refused and
  counted as a mispost, and the AO never sees it.
- A panic on core 1 writes core 1's breadcrumb (read by core 0 from
  the other platform's record), a halted core 1 answers nothing,
  `Core1::reset()` puts it back into the bootrom and a new launch
  brings it back answering.
- THE PROBE TRAP. OpenOCD's target script takes both cores as an SMP
  pair by default: programming halts both, and core 1 is left with
  C_DEBUGEN set and, once, halted (DHCSR 0x00030003 read back) - a
  BKPT on core 1 then HALTS it instead of faulting, and with
  DBGPAUSE at its reset value the halted core FROZE the system timer
  under core 0, whose 10 ms wait never ended; and a `reset run` that
  resets core 0 alone leaves core 1 in its old life, deaf to a new
  launch. So `brio flash` keeps OpenOCD off core 1 (`USE_CORE 0`),
  `Timer::init` clears DBGPAUSE, `Core1::launch` resets core 1 first
  and `Reset::software()` is the chip's reboot, not one core's
  ([reset.md](reset.md)). A core 1 a debug session left halted is
  released once with the probe: `reset run` on both cores, then a
  DHCSR write of 0xA05F0000 on each
  ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)).

## Not covered yet

Driver gaps, each with its reason:

- A console on core 1 (UART1 on the second probe's bridge): the pins
  are the serial suite's cross link today; a wire change when a
  program wants core 1 to speak for itself, the bridge carries its
  facts meanwhile.
- The SIO spinlocks, dividers and interpolators: nothing in the model
  wants a lock, and the rest waits for a program.
- Per-core GPIO interrupts (IO_BANK0's PROC0/PROC1 enables) and the
  DMA's two lines: with their chapters, each driver's `init` enabling
  its line on the calling core is rule 2 already.
- The bus fabric's contention counters (BUSCTRL): the measure of two
  cores executing from one flash cache, born with the first program
  whose timing asks.
- A power model across the cores (the chip-level sleep needs both
  kernels' consent: a vote through the bridge with the round's
  deadline as the veto): born with the first power-aware two-kernel
  program.

Implemented but not bench-verified, each with what would measure it:

- Two kernels each with a console: the wire change above and a
  `brio duo` of the two consoles.
- A `Lease::reply` buffer crossing inside a request and back inside
  the reply: a letter with a buffer-carrying request; the fences are
  the inbox's and the host test covers the capsule.
