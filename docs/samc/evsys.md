# EVSYS - the Event System (SAM C21)

> **PROVISIONAL.** The fabric is built and bench-verified end to end.
> What is deliberately NOT here is the vocabulary - the tables of 95
> generators and 47 users - and SleepWalking. The list is in "Not covered
> yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 29 - and
errata DS80000740S items 1.12.1, 1.12.3 and 1.12.4, **all three live on
every silicon revision including this one** (1.12.2 is revisions B..E and
not this chip). Driver: `samc/evsys.hpp`. Family fixture
`test/family_samc/evsys.cpp` plus one negative under
`tools/check_samc.sh`; the bench suite is `test_samc_evsys`, which uses
`samc/dmac.hpp` as its event user.

## What the silicon does

**This is where the AVR shape does not transfer**, and it is worth
saying before any verb. On the AVR the event system is a small fixed
table: a channel is a typed thing, a generator is a type, and legality is
answered at compile time. Here it is an **allocator** - twelve identical
channels, numeric generator and user codes drawn from tables ninety-five
and forty-seven rows long, and a generic clock per channel. Reproducing
the AVR's per-generator types would mean ninety-five of them, encoding a
table this driver has no business owning.

So the driver owns the **fabric** and not the vocabulary: it moves
channels, users, paths and edges, while a peripheral that generates
events publishes its generator codes and one that consumes them publishes
its user index. That division is what keeps this header from growing a
table for every chapter in the book.

**The user multiplexer is written before the channel** - "the user
multiplexer must always be configured before the channel" (29.6.2.3).

**USER.CHANNEL is the channel plus one**: zero means "no channel output
selected", and channel *m* is chosen by writing *m+1* (29.8.9's own
note).

**Three paths, and the path decides everything else.** Asynchronous is
straight through - no clock, no latency, and no status at all: both
interrupt flags and the whole channel status read zero (29.6.2.9, .10,
.11), and the edge detector "must be disabled by software". Synchronous
costs one GCLK_EVSYS_CHANNEL_n cycle and requires the generator to share
the channel's clock generator. Resynchronized costs three and is what to
use when they do not. Both clocked paths **require** edge detection
(29.6.2.7).

**A software event can be raised on any channel** (29.6.2.12), serviced
as a generator's would be - which is what makes the whole subsystem
testable without a wire. See "Bench findings" for the qualification the
chapter does not give.

## The errata, which are unusually heavy for twenty pages

- **1.12.1** (all revisions): a **synchronous** channel whose generic
  clock never stops (ONDEMAND = 0) can raise **spurious overrun
  interrupts**. ONDEMAND = 1 is the documented workaround, so it is the
  default here and a synchronous configuration that clears it is
  refused. The resynchronized path is not affected.
- **1.12.3** (all revisions): a software event on a **resynchronized**
  path does not set CHBUSY immediately, and a second event inside that
  window is **lost with no overrun flag** - the worst kind of silence.
  Three channel-clock cycles is the remedy, and it is the caller's to
  spend: the driver does not know that clock's rate.
- **1.12.4** (all revisions): a freshly configured and enabled channel is
  busy for one channel-clock tick **without CHBUSY showing it**, so a
  trigger issued immediately can be swallowed. Visible when the EVSYS
  clock is slower than the CPU - which is the interesting case.
- **1.12.2 is not this silicon** (E/G/J revisions B..E only).

## Types and verbs

**`EventPath`** - `asynchronous`, `synchronous`, `resynchronized`.
**`EventEdge`** - `none`, `rising`, `falling`, `both`.

**`EventChannelConfig`** - `generator` (a numeric code, 0 = none, which
is what a software-only channel wants), `path`, `edge`, `on_demand`
(default **true**, for erratum 1.12.1) and `run_standby`.
`Evsys::config_valid()` refuses a generator past the seven-bit field, an
asynchronous channel with an edge, a clocked channel without one, and a
synchronous channel with a free-running clock.

**`Evsys`** - the fabric, monostate. `channel_count` (12), `user_count`
(47), `gclk_id(channel)` (contiguous from the header's own first
constant), `bus_clock`, `reset`.

- *Channels*: `configure`, `channel_reg`, `release_channel`.
- *Users*: **`connect(user, channel, cfg)`** is the one verb that takes
  both, precisely so 29.6.2.3's ordering cannot be got wrong;
  `attach(user, channel)` adds a second user to a channel someone else
  configured (several users may share one); `disconnect`;
  `user_channel(user)` decodes the off-by-one back to a plain number,
  answering `channel_count` for "none".
- *Status* (clocked paths only): `channel_status`, `busy`,
  `users_ready`, `flags`, `armed`, `arm`, `disarm`, `clear_flags`,
  `overrun_flag`, `detected_flag`, `overrun`, `detected`, `isr`.
- *Software event*: `trigger(channel)`, one store into the write-only
  SWEVT.

## How to use it

**Route an event to a peripheral** - the user first, which `connect()`
enforces:

```cpp
Evsys::bus_clock(true);
GclkChannel::connect(Evsys::gclk_id(0), /* generator */ 5);   // clocked paths only
Evsys::connect(/* user */ 5, /* channel */ 0, EventChannelConfig{
    .generator = some_peripheral_generator_code,
    .path = EventPath::synchronous,
    .edge = EventEdge::rising,
});
```

**Raise one from software**, remembering the two waits the errata ask
for - one channel-clock tick after configuring, three after a software
event on a resynchronized path:

```cpp
Evsys::trigger(0);
```

## Bench findings

From `test_samc_evsys` (4 letters, 37 verdicts, 37/37). Nothing to wire:
the software event supplies the stimulus and the DMAC supplies the user.

- **An event moves bytes with no CPU in the path.** A DMA channel armed
  with *no hardware trigger* (`dma_trigger_none`, EVACT trigger, EVIE
  set) copies its block when - and only when - a software event reaches
  it through EVSYS. That also retires `dmac.md`'s own caveat that every
  EVACT value but `none` was untested silicon.
- **A SOFTWARE EVENT ON AN ASYNCHRONOUS CHANNEL DOES NOT REACH THE
  DMAC** - and what that means took two suites to pin down. Measured
  here, **eight** back-to-back software events on an asynchronous channel
  move nothing through a DMA channel, while **one** on a synchronous or
  resynchronized channel moves a whole block; 29.6.2.12 says a software
  event "can be serviced as any event generator" and never qualifies by
  path. **But the limit belongs to the USER and not to the path.**
  `test_samc_ccl` puts a *different* user on the same asynchronous
  channel - a CCL LUT, whose event input has an edge detector of its own
  - and **sixteen of sixteen single software events arrive**, with a
  disconnected-user control catching none, and with one of them moving a
  DMA block through the LUT as a second witness. So the asynchronous path
  really does carry a software event; what a register write has no width
  for is the DMAC's own trigger stage. A hardware generator crosses that
  path for every user tried (`test_samc_eic`, [eic.md](eic.md)).
- **Both clocked paths work and both raise EVD**, the event-detected flag
  that only they have.
- **The asynchronous path really is silent.** After eight events its
  overrun and event-detected flags are both zero, and CHSTATUS reports
  the channel neither busy nor ready - while the *unused* channels
  report ready (CHSTATUS reads 0xFFE, every channel but this one). Code
  that polls any of these to pace an asynchronous channel is polling a
  constant.
- **The off-by-one is real**: a user connected to channel 3 through the
  driver's plain-number verb leaves 4 in USER[m].

**CHANNELn.RUNSTDBY REACHES THE ASYNCHRONOUS PATH TOO**, which table
29-1's own layout hides: three of its four rows are SYNC/RESYNC, which
invites the reading that the bit is a synchronous-path concern - the
asynchronous path having no clock to keep alive. It is not. 29.6.4's
sentence says a channel needs the bit "to be able to run in Standby
mode", the table's single ASYNC row reads "Disabled in Standby Sleep
mode", and measured with nothing else in the chain moving, a HARDWARE
event over an ASYNCHRONOUS channel crossed 32 times in a 30 ms standby
with the bit set and NOT ONCE without it (32 awake). Every sleepwalking
chain in this stratum depends on that bit -
[platform.md](platform.md), "Sleep, peripheral by peripheral".

## Not covered yet

Driver gaps (deliberate):

- **The generator and user tables.** Ninety-five generator codes and
  forty-seven user indices belong to the peripherals that offer and
  consume them, not to this header - a driver publishes its own codes and
  hands them over. This is the position the whole design rests on, and
  the reason the file is short.
- **The channel INTERRUPT as a wake source.** CHANNELn.RUNSTDBY is
  measured on the propagation path (see "Bench findings"); an EVSYS
  overrun or detection interrupt leaving a standby is not.
- **The channel interrupt as a program's event hook.** `isr()` and the
  flags exist and are exercised, but nothing in the framework yet turns
  an EVSYS interrupt into a kernel event.

Implemented but not bench-verified:

- **Most real generators.** Every event in *this* suite is a software
  one, so `CHANNELn.EVGEN` is only ever written as zero here. The first
  driver to publish generator codes was the EIC, and `test_samc_eic`
  exercises EVGEN, the resynchronized and asynchronous paths and rising
  edge detection with a real hardware generator - the other ninety-odd
  codes wait for their own drivers.
- Falling and both-edge detection; `overrun` actually being raised
  (provoking one needs a generator faster than its user).
- Operation on the E and G variants: compile-checked only. Nothing in
  this chapter varies by package.
