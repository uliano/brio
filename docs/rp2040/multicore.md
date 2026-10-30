# Multicore (RP2040)

This document is for a programmer who wants the RP2040's second core
to run a program of its own. A single-core brio program runs on core 0
and leaves core 1 asleep in the bootrom; a two-core program runs two
kernels, one per core, each with its own list of active objects (an
active object is a component of a brio program with its own event
queue, served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)), its own
SysTick timebase and its own panic record. Nothing is shared between
the two kernels but events, and an event crosses from one core to the
other through a bridge, `util/inbox.hpp`'s `send<Ao>()`, never through
the kernel's `post<Ao>()`, which is one core's primitive
([design/kernel.md, "Two cores: two kernels and a bridge"](../design/kernel.md#12-two-cores-two-kernels-and-a-bridge-utilinboxhpp)).
This stratum supplies the platform type per core
(`Rp2040Platform<core>`), the ticker per core (`CoreTicker<core>`),
the doorbell a crossing rings (the SIO's inter-core FIFO) and `Core1`,
which starts core 1 on a function.

The smallest use, from "How to use it" below - the two `main`s and the
glue that binds them:

```cpp
using P0 = brio::Rp2040Platform<0>;
using P1 = brio::Rp2040Platform<1>;

extern "C" void isr_systick() {
    if (P0::core_id() == 0) brio::CoreTicker<0>::tick(); else brio::CoreTicker<1>::tick();
}
extern "C" void isr_sio_proc0() { brio::Inboxes<Origin>::isr(); }
extern "C" void isr_sio_proc1() { brio::Inboxes<Echo>::isr(); }

[[noreturn]] void core1_main() {
    brio::CoreTicker<1>::init(clock);
    brio::Inboxes<Echo>::enable();
    brio::enable_interrupts();
    brio::Tenuto<P1, Echo>::run();
}

int main() {
    SysClock::init();                         // core 0 owns the clock tree
    brio::Ticker::init(clock);
    const bool up = brio::Core1::launch(&core1_main);
    brio::Inboxes<Origin>::enable();
    brio::enable_interrupts();
    brio::send<Echo>(Ping{1});                // crosses; post<Echo> would be refused
    brio::Tenuto<P0, Origin>::run();
}
```

Line by line: `P0` and `P1` are the two cores' platforms, and the
kernel's state is kept per platform type, so `Tenuto<P0, ...>` and
`Tenuto<P1, ...>` are two kernels; the one SysTick vector serves both
cores, each core's SysTick raising it on that core, and ticks the
ticker of the core that took it; `isr_sio_proc0` and `isr_sio_proc1`
are the two doorbell lines, one per core, each draining the inboxes of
the active objects that live on its core into ordinary local posts;
`core1_main()` is core 1's program - its ticker, its doorbell, its
interrupts, its kernel; `main()` is core 0's - the clock tree, its
ticker, the launch of core 1 (false when core 1 did not answer the
bootrom's protocol), its doorbell, its kernel. `Origin` lives on core 0
and `Echo` on core 1 (its queue is declared over `P1`, and its
`inbox_depth` sizes the ring that receives from core 0); `SysClock` and
`clock` are the clock type and its tag, as in
[platform.md](platform.md).

What decides correctness even there. An event for an active object on
the other core is sent, not posted: `post<Echo>()` from core 0 is
refused and counted as a mispost, and the active object never sees it
("Bench findings"). `send()` may be called from an active object's
handler or from an interrupt handler of the sending core: the sender
writes the inbox under its own core's critical section, so the loop and
the handlers of one core take turns (`util/inbox.hpp`). A full inbox
drops the event and counts it, as a full queue does; nobody waits.
Within each core the kernel's rule is the single-core one: a handler
runs to its end before the kernel serves another event there, whatever
its priority, and an event that crosses joins its active object's queue
on the other core like a local post. Each core enables its own lines: a
line's enable acts on the NVIC of the calling core, so
`Inboxes<...>::enable()` runs on the core whose inbox it is. The
`init()` of the UART, the SPI and I2C drivers and the USB device
enables the block's line on the core that calls it, and
`DmaLine<n>::enable()` or a DMA engine's `arm()` the DMA line's; for
every other block (the timer, the ADC, the RTC, the PIO, the PWM) the
program calls `Nvic::enable` itself, on the core that will serve it. A
critical section masks one core's interrupts and nothing on the other
core: two kernels share nothing but the bridge. `Core1::launch()` runs
on core 0 before core 0's doorbell interrupt is enabled, since the
launch protocol speaks over the same FIFO - with the doorbell enabled,
its handler would take the bootrom's answers out of the FIFO the
protocol reads them from - and it resets core 1 first, so a core 1 left
in an old life starts over. The clock tree is one for the chip and core
0 sets it, before the launch, so core 1's ticker starts from the rate
core 0 has already reached. And a debugger halting one core can stop
the system timer under the other: `brio flash` keeps OpenOCD off core 1
and `Timer::init()` clears the timer's debug pause ("The probe trap",
below).

## Documents of record

The RP2040 datasheet (build 3184e62), 2.3.1 (the
SIO: CPUID, the inter-core FIFOs and their interrupts, the spinlocks),
2.3.2 (the two NVICs, every line reaching both), 2.4.2 (what
SYSRESETREQ resets), 2.8.2 (the bootrom's launch protocol for core
1), 2.13 (the power-on state machine, whose FRCE_OFF resets one
core). The drivers: `brio/rp2040/multicore.hpp` (the doorbell, the
launch), `brio/rp2040/platform.hpp` (one platform type per core),
`brio/rp2040/ticker.hpp` (one ticker per core) over
`brio/util/inbox.hpp`, the bridge of
[design/kernel.md, "Two cores: two kernels and a bridge"](../design/kernel.md#12-two-cores-two-kernels-and-a-bridge-utilinboxhpp).
The reference suites: `test_rp2040_multicore` (a suite's letter is one
command key of its console and the group of verdicts it runs, and its
all-key, `z`, runs the set), and `test_rp2040_rubato`'s letters j and k
for the crossing under the preemptive kernel.

## What the silicon does

Two Cortex-M0+ cores with one bus fabric, one flash and one SRAM
between them, and everything else doubled: each core has its own
NVIC, its own SysTick, its own PRIMASK, its own debug logic. Every
one of the 32 interrupt lines reaches both NVICs and each core enables
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
  false when unanswered. Core 0's doorbell interrupt is to be disabled
  around either, which the caller keeps: nothing in `Core1` disables
  it, so a program launches before it enables the doorbell, or
  disables it around a relaunch.
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
    brio::Tenuto<P1, Echo>::run();
}

int main() {
    SysClock::init();                         // core 0 owns the clock tree
    brio::Ticker::init(clock);
    const bool up = brio::Core1::launch(&core1_main);
    brio::Inboxes<Origin>::enable();
    brio::enable_interrupts();
    brio::send<Echo>(Ping{1});                // crosses; post<Echo> would be refused
    brio::Tenuto<P0, Origin>::run();
}
```

A panic on either core reboots the chip through `ResetReporter`
(rp2040/reset.hpp), and core 0 reads both breadcrumbs at the next
boot: `take_panic_record<P0>()` and `take_panic_record<P1>()`. With no
debugger attached a panic's breakpoint raises a HardFault before the
reporter runs, so the reboot is the HardFault handler's, and happens
only when the program binds `isr_hardfault` to `fault_reset<P>()`
([reset.md](reset.md)): the crt's own `isr_hardfault` is a spin, and
the core stays in it. The suite's own panic on core 1 names no
reporter, and that core halts (below).

Under the preemptive kernel, Rubato, on either core or both
(`Rp2040RubatoPlatform<core>`, [platform.md](platform.md)), the bridge
is unchanged and a crossing is an interrupt like any other on the core
it reaches: the doorbell's handler drains the inbox into local posts,
a post to an object above that core's ceiling pends that core's PendSV,
and the receiver's dispatch starts at the handler's exit over whatever
dispatch the doorbell interrupted there - road 1 on the receiving core,
the sending core's ceiling never consulted. With a Rubato on each core
the one PendSV entry of the shared vector table takes
`rp2040/schedule.hpp`'s `scheduler_exception_pair<K0, K1>()`, which
reads SIO's CPUID and returns into the scheduler of the core that took
it; a vector table per core, the other way, is declined
([platform.md](platform.md), "How to use it"). The doorbell's road 1
and the pair body are measured by `test_rp2040_rubato` on the WeAct
RP2040 at 125 MHz ("Bench findings" below, and
[platform.md](platform.md)'s "Bench findings" for the rest of the
suite).

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
- TWO KERNELS, TWO CONSOLES: a program with the console's three
  active objects on core 0 over UART0 and the same two on core 1 over
  UART1, each on a Debug Probe's bridge, answers HELP, UPTIME (each
  core's own ticker), ERR and CORE on both with every counter at
  zero; the LED command typed on core 1 is a send across the bridge
  to core 0's Lamp, and the pin follows it (read through the
  debugger after ON, OFF and TOG).
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
- A DMA ENGINE ON THE SECOND CORE (letter i): the copy engine armed by
  core 1's kernel takes its line into core 1's NVIC, and its completion
  is served there alone, the report crossing back through the inbox
  ([dma.md](dma.md)).
- A `Lease::reply` LOAN ACROSS AND BACK (letter j, ten runs of ten): 64
  requests each lend core 0's 256-byte buffer to core 1's Echo, which
  adds one to every byte, sums them and replies through the crossing
  `ReplyTo` with the same loan; core 0, reading only once the reply is
  dispatched, finds every byte incremented, the pointer and length its
  own, the sum right - the inbox's fences carry the loan's contents as
  they carry the event.

Under Rubato, from `test_rp2040_rubato` on the WeAct RP2040 at 125 MHz,
ten runs of its `z` alike, clk_sys cycles on core 0's composed
`cycles()` with its own two-read cost of 55 taken off:

- ROAD 1 ACROSS THE CORES (letter j, core 1 under Tenuto running one
  object that answers each of core 0's sends with a send to core 0's
  High): a `post()` from core 1 to core 0's object is refused and
  counted as a mispost and pends no PendSV on core 1. Low's send crosses
  to core 1 and the answer crosses back; core 0's doorbell handler posts
  to High, and High's dispatch starts after that handler's last
  statement and before Low resumes, 7 to 13 us from Low's send to High's
  first statement across core 1 and back. From the doorbell handler's
  last statement to High's first, 212 cycles min and 213 median over
  100 crossings, none over 290 - the figure of road 1 from an alarm's
  handler on the same core (213 median), so a crossing preempts
  as any interrupt does. A crossing a kernel tick or an XIP cache refill
  lengthened is told apart by the counters read around it and reported,
  and the bound judges the ones nothing lengthened.
- A RUBATO ON EACH CORE (letter k, core 1 a pack of three on
  `Rp2040RubatoPlatform<1>`, the one PendSV entry bound to the pair
  body): each core's trace in 10 of 10 runs shows its own ceiling at
  every entry, road 1 from its own alarm and road 2 inside it; the two
  cores preempt at the same time, each preemption landing inside the
  other core's preempted dispatch on the system timer, in 10 of 10; core
  1's ceiling is 255 at its base after; its stack's high-water mark is
  260 bytes in ten runs of ten, the deepest sampled stack pointer 240 in
  a dispatch and 112 in its interrupt.
- THE CROSSING UNDER DRIVER TRAFFIC (`test_rp2040_rubato_link`, its
  eighth pass of letters o and t, [platform.md](platform.md) "Bench
  findings"): core 1 under Tenuto, one object that alarm 1 posts to
  every 97 us and that sends to core 0's preempting object through its
  inbox, so each preemption of core 0's SPI and UART stacks is road 1
  from the doorbell's handler - every send one dispatch, none dropped,
  every window and every line exact. Core 1's program shares the XIP
  cache with core 0's: its idle loop adds 280 to 440 misses to the
  sixteen SPI exchanges of core 0's matrix and its ticking 1370 to 1550
  (that suite's letter `x`, three runs; the cache's counters, the bus
  fabric's not read).

## Not covered yet

Driver gaps, each with its reason:

- The SIO spinlocks, dividers and interpolators: nothing in the model
  wants a lock, and the rest waits for a program.
- Per-core GPIO interrupts (IO_BANK0's PROC0/PROC1 enables) and the
  DMA's two lines served one per core: with their chapters. Until
  then the two-kernel model's rule 2 (each line has one owner core,
  which enables it) is the programmer's duty: a line is enabled on
  whichever core calls the verb that enables it (a driver's `init()`,
  `DmaLine<n>::enable()`, an engine's `arm()`, or `Nvic::enable`), and
  nothing refuses a line enabled on both cores.
- The bus fabric's contention counters (BUSCTRL): the measure of two
  cores executing from one flash cache, born with the first program
  whose timing asks.
- A power model across the cores (the chip-level sleep needs both
  kernels' consent: a vote through the bridge with the round's
  deadline as the veto): born with the first power-aware two-kernel
  program.
