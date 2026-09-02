# DMAC (SAM C21)

> **PROVISIONAL.** The block, the twelve channels, memory-to-memory
> transfers, mid-block harvesting with the erratum-1.10.4 validation,
> and the four engines - two serial, two streaming - are implemented
> and bench-verified. The CRC engine, linked descriptor lists, the
> event system hooks and the standby sequence are declared, not built.
> The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 25 - NOT
ch. 19, where an older revision's numbering put it - and errata
DS80000740S items 1.10.1..1.10.4, the whole matrix re-read against
this chip (E/G/J family, silicon rev F): 1.10.4 is LIVE and measured
here, the other three are not this silicon (see "What the silicon
does"). Driver: `samc/dmac.hpp` (`Dmac` block + `DmaDescriptor` /
`dma_descriptor()` + `DmaChannel<n>` + four engines: `DmaTxEngine` /
`DmaRxEngine`, which `samc/sercom.hpp`'s Uart takes as options, and
`DmaLoopEngine` / `DmaPingPongEngine`, the two streaming shapes).
Family fixture `test/family_samc/dmac.cpp` plus negatives under
`tools/check_samc.sh`; the bench suites are `test_samc_dma` (the block,
the channel, the serial engines), `test_samc_analog_dma` (the streaming
engines and the element-type generalization) and `test_samc_timer_dma`
(the same two engines on the timers, and the peripheral that does NOT
present its request the way 25.8.8 leads one to expect).

## What the silicon does

**The channel registers sit behind a selector.** CHCTRLA, CHCTRLB,
CHINTENCLR/SET, CHINTFLAG and CHSTATUS are ONE register set, not
twelve: they talk to whatever channel was last written to CHID
(25.8.17), so every channel access is a non-atomic select-then-use
pair. An interrupt handler that re-arms channels (the TX engine does)
would silently redirect main context's pair, so every CHID access in
the driver goes through one private door that holds the PRIMASK
critical section across both halves - structurally: `DmaChannel` is
`Dmac`'s only friend and nothing else can name CHID at all.

**INTPEND is the dispatch, and it needs no selector.** One 16-bit
register carries the LOWEST pending channel number together with that
channel's TERR/TCMPL/SUSP flags, and a write of {flags, id} clears
those flags for that id (25.8.10). One read answers "which channel and
why", one store acknowledges exactly what was read - no CHID, no
critical section, which is what lets the handler run without fighting
main context for the selector. Several channels pending means several
reads: the handler loops until nothing is reported.

**Descriptor memory is the driver's to provide.** BASEADDR points at
the array of first descriptors, WRBADDR at the write-back array where
the controller keeps each channel's ongoing state (25.6.2.3): two
static tables of 12 x 16 bytes, 384 bytes of .bss, 16-byte aligned (a
superset of the required 8: the stride is 0x10, so the whole array
inherits the first entry's alignment). Both registers are
enable-protected - a write while the block runs is DISCARDED, not
refused - so `Dmac::init()` writes them into a stopped controller.
The tables exist only in an image that reaches them (`gc-sections`),
which is half of the zero-when-absent proof below.

**The end-address quirk.** For an INCREMENTING side, the descriptor's
SRCADDR/DSTADDR holds the address one beat PAST the last one - start
plus the whole length in bytes - while a static side (a peripheral's
DATA register) holds the plain address (25.6.2.7). Get it wrong and
the transfer runs over the wrong memory silently. The driver never
lets a caller near that arithmetic: `DmaTransfer` takes a start
pointer and a BEAT count, `dma_descriptor()` computes the words, and
the computation is constexpr so the family fixture pins it without a
chip. The data sheet disagrees with itself here: 25.6.2.7 prints
`+ BTCNT x BEATSIZE x 2^STEPSIZE`, the register descriptions 25.10.3
and 25.10.4 print a stray "+ 1" that their own definition of BEATSIZE
(bytes per beat) makes meaningless. The driver implements 25.6.2.7
and the bench decided it by moving bytes and looking where they
landed: a deliberately naive start-address descriptor is shown moving
the decoy buffer that sits below the payload.

**Disabling is not instantaneous, and everything behind it is
silent.** A '0' written to CHCTRLA.ENABLE during a transfer does not
clear until the internal buffer drains (25.8.18, 25.6.3.6) - and
until it clears, CHCTRLB is still enable-protected and CHCTRLA.SWRST
is still IGNORED, both silently: the SWRST bit never reads back set,
so a wait for it to clear succeeds instantly having reset nothing.
The driver's `enable(false)` therefore waits, bounded, and `reset()`
refuses when the disable did not complete - see the bench findings
for how this was caught.

**Suspend is the only window into progress.** The controller keeps
BTCNT internally and spills it to the write-back only when the
channel loses arbitration, suspends, or disables (25.10.2). So
mid-block progress cannot be polled - `harvest()` suspends the
channel, reads the write-back, resumes. Two silent edges live inside
that handshake: a SUSPEND command on a disabled channel is dropped
(25.6.3.2), and the block can END between the enabled-test and the
command landing - so the wait accepts "channel no longer enabled" as
the answer it is. A RESUME on a channel that is not suspended is not
a no-op either: it skips the next suspend action (25.6.3.3) - and on a
channel that has not started, it is worse than that: 25.6.2.8 says a
resume that makes the controller fetch a next descriptor with a null
DESCADDR sets CHSTATUS.FERR and suspends the channel, and every
single-block descriptor this driver builds has DESCADDR = 0. So a
harvest of a channel whose write-back is still the zeros `reset()` put
there does not suspend anything at all: it answers "no beats done",
which is both true and the only safe reply.

**A TRIGGER IS AN EDGE, NOT A LEVEL - PER REQUEST SHAPE, AND THE SHAPE
IS THE PERIPHERAL MODE'S.** A peripheral asserts its DMA request while
its condition holds; the DMAC latches a pending trigger when the
request RISES. A block armed while the condition is already true is
therefore waiting for an edge that has already gone by, and the
channel sits enabled with nothing pending and nothing moving - the
USART transmit measurement, and the reason the engines expose `kick()`:
one software trigger, which the owner issues when it can see the
peripheral's flag already standing. SWTRIGCTRL raises the single
pending bit only if it was clear (25.8.8), so a kick that races a real
trigger is lost, never doubled. BUT THE SAME SERCOM IN SPI HOST MODE
MEASURES THE OTHER WAY: enabling a channel with DRE already standing
fires the first beat by itself, and a kick on top of that start is one
extra beat whose byte a full transmit buffer discards in silence
(exactly one early character vanishes from the wire, at every rate;
spi.md). So the kick is the OWNER's judgement about ITS peripheral's
request shape - USART TX kicks, SPI's launch does not, a TC capture
needs neither (below) - and the engine stays the mechanism.

**Erratum 1.10.4 is live on this silicon, and the write-back is
therefore checked, never believed.** "Concurrent channels triggers"
(the summary table files it under "Linked Descriptors"): when several
channels are triggered concurrently, write-back descriptors may be
corrupted - E/G/J at revisions E, F and H. Microchip's workaround
(sequence everything through linked descriptors on a single channel)
amounts to not using concurrent channels, which a full-duplex serial
port cannot honour. The driver takes the other road: every field of a
write-back except BTCNT and VALID is invariant WHILE THE BLOCK RUNS
(copied from the fetched descriptor, and until the block ends only the
beat counter is written back - 25.10.2), so `harvest()` compares them
all against the copy it loaded, bounds-checks BTCNT, and DISCARDS a
reading that fails, counting it in `violations()`. The corruption is
real and was caught in the act - see the bench findings.

**THE READING IS NOT THE DAMAGE.** Validating what is read is
necessary and NOT sufficient, and that correction cost a wedged serial
port to learn. 25.6.2.6: "For an ongoing block transfer, the
descriptor will be fetched from the WRITE-BACK memory section
(WRBADDR)." The write-back is not a report a driver may take or leave
- it is the controller's LIVE COPY of the descriptor it is running -
so when 1.10.4 scribbles it, the transfer itself is destroyed. The
channel stops moving bytes, raises no interrupt and sits there
enabled. On the receive side the same corruption shows as
CHSTATUS.FERR, which 25.6.2.8 raises when an invalid descriptor is
fetched and which only a software RESUME clears. Two consequences the
driver now carries: an ENGINE'S OWNER can declare a block dead and
`abandon()` it (the channel is reset, reconfigured and re-armed, and
the count is public in `faults()`), and a harvest of a channel whose
write-back the driver itself zeroed reports "nothing started" instead
of suspending it - because 25.6.2.8's OTHER clause raises the same
fetch error when a RESUME makes the controller fetch a next descriptor
whose address is null, which every single-block descriptor here has.
WHO decides a block is dead is never this driver: only the
peripheral's owner can read the peripheral's own flags (samc/
sercom.md's dead-block predicate is one line of SERCOM truth - a
transmit block cannot be in flight while DRE and TXC are both set).

The TX engine still never reads a write-back for its PROGRESS:
programmed length plus TCMPL is the whole truth on that side. The rest
of the 1.10.x matrix:
1.10.1 (CRC data port) is rev B only; 1.10.2 and 1.10.3 (linked
descriptor fetch items) mark revisions B..D for E/G/J - the trap is
that each item's matrix also has an N-family row, and for those two
it is the N row that carries the marks under E and F. Read the row,
not the column.

**The device header's LVLEN trap.** `DMAC_CTRL_LVLEN(v)` is the group
macro (all four level-enable bits); `DMAC_CTRL_LVLEN0(v)` masks to a
single bit, so feeding the per-level macro a four-bit mask silently
enables level 0 alone - and a channel whose priority level is not
enabled is not slow, it is INVISIBLE to the arbiter (25.8.1).

**Clocks are almost nothing.** The DMAC has no GCLK channel at all -
CLK_DMAC_AHB is the whole story (25.5.3), and its AHBMASK bit is set
out of reset. `init()` sets it anyway: "the block is clocked" is a
promise, not an inheritance from whatever a debugger left behind.

## Types and verbs

- **The vocabulary** - `DmaBeat` (with `dma_beat_bytes()`: the code is
  not the width), `DmaStep`/`DmaStepSide` (one step size per
  descriptor, owned by the side STEPSEL names; the other side always
  advances one beat), `DmaBlockAction` (note the clause behind
  `none`: it suppresses TCMPL outright), `DmaEventOut`,
  `DmaTriggerAction` (`beat` is the serial shape - one trigger, one
  beat; `block` the memory-to-memory one), `DmaPriority`,
  `DmaEventAction`, `DmaFlag`/`DmaStatus` masks.
- **Triggers** - read off the device header per instance, never
  computed: `dma_trigger_sercom_rx/tx<n>()` here (statically checked
  against `Sercom<n>::dma_rx/tx_trigger()` in the family fixture, so
  the two spellings cannot drift), `dma_trigger_none` for
  software-triggered channels. Other peripherals' codes arrive with
  the drivers that own them.
- **`DmaTransfer` / `dma_descriptor()`** - start pointers plus beat
  count in, register words out; zero beats or a null end yields an
  invalid all-zero descriptor, `dma_transfer_valid()` is the
  static_assertable predicate. `DmaDescriptor` mirrors the device
  header's layout bit for bit (static_asserted) and is a plain
  comparable value - which is what lets a channel keep the loaded
  copy that judges the write-back.
- **`Dmac`** - the block resource: `init(DmacConfig)` (reset, tables,
  arbitration - all four levels enabled by default, round-robin and
  DBGRUN opt-in), `take_pending()` (the INTPEND dispatch: one
  `DmaInterrupt` per call, flags already cleared, nullopt when done),
  block-level readbacks (INTSTATUS/BUSYCH/PENDCH/ACTIVE),
  `descriptor(id)`/`write_back(id)`/`read_write_back(id)` (the
  write-back is evidence, and a suite must be able to look at it),
  `release()`.
- **`DmaChannel<n>`** - one channel: `configure(DmaChannelConfig)`
  (disables first: CHCTRLB is enable-protected), `load()` (descriptor
  into the slot, copy remembered), `enable()` with the bounded
  disable wait, `reset()` (refuses when the disable failed; clears
  both table slots and the copy), `trigger()`/`trigger_lost()` (the
  SWTRIGCTRL readback semantics: the bit reads set exactly when the
  trigger was LOST to an already-pending one), `suspend()`/`resume()`,
  flags/arming/status verbs, `harvest()` -> `DmaProgress` with the
  1.10.4 validation, `violations()`/`suspend_timeouts()` counters.
  Everything channel-addressed pays the CHID guard uniformly.
- **The four engines** - `DmaTxEngine<ch, Elem>`,
  `DmaRxEngine<ch, Elem>`, `DmaLoopEngine<ch, Elem>` and
  `DmaPingPongEngine<ch, Elem>`. All four are peripheral-agnostic by
  construction: `arm(data_address, trigger)` takes any peripheral's
  DATA address and trigger code, so the same engine serves a SERCOM, a
  DAC, either ADC, the SDADC or TSENS unchanged. The channel number is
  refused at the spelling site (a static_assert in the engine,
  instantiated where it is named).
- **`Elem` IS THE BEAT**, through `dma_beat_of<Elem>()`: one `sizeof`
  feeds both BEATSIZE and the end-address arithmetic, so the two
  cannot disagree, and a width the silicon does not implement is a
  compile error (25.10.1 has three, and the fourth code is Reserved).
  It defaults to `uint8_t` - what a SERCOM moves - so every existing
  spelling means what it always did. Alignment is NOT checked, because
  the descriptor has no field for it: a misaligned buffer is a bus
  error (CHINTFLAG.TERR), which is why the engines take a typed
  pointer and not a `void*`.
- **The hardening all four inherit**, and the reason they are one
  family: `kick()` (one software trigger for a request that is already
  standing - see "a trigger is an edge" below; un-doublable by
  construction, since a channel has one pending bit and SWTRIGCTRL
  raises it only if clear), `abandon()`/`faults()` (the caller decides
  a block is dead, never the engine - only the peripheral's owner can
  read the flags that make "dead" a fact rather than a timeout; what
  the abandonment loses is stated, not pretended away), and mid-block
  progress through `DmaChannel::harvest()` and nowhere else, so every
  write-back reading is validated against the loaded descriptor.
- **`DmaTxEngine` / `DmaRxEngine`** - the optional Uart engines (see
  sercom.md for the task-side contract): drain a buffer into a
  peripheral, and fill a buffer from one. The receive side's asymmetry
  is its own: arrival is not an event anyone is told about, so
  `take()` asks by harvesting and the PACING is the caller's.
- **`DmaLoopEngine`** - one caller-owned table played into a
  peripheral for ever. `start(table, length)`, `complete()` from the
  handler (counts the lap, re-arms the same block), `laps()`,
  `progress()`, `stop()`. **There is no hardware circular mode on this
  controller**: 25.6.3.1 offers only a self-linked descriptor, and
  linked descriptors are deliberately not built here because 1.10.4
  corrupts the write-back that 25.6.2.6 makes the LIVE descriptor - a
  self-linked chain has no second copy to judge the first against. So
  the lap boundary is a TCMPL interrupt, which costs one interrupt per
  TABLE and not per sample.
- **`DmaPingPongEngine`** - two caller-owned buffers, the engine
  filling one while the caller drains the other.
  `start(first, second, length)`, `complete()` from the handler,
  `ready()`/`ready_length()`/`release()`, and the accounting that IS
  the API: `laps()`, `overruns()`, `stalled()`, `pending()`. On an
  overrun - the caller still holding both buffers - the engine SKIPS
  the lap rather than write into the buffer being read: that trades
  samples for integrity, so everything handed over is a complete,
  untorn block. What it does NOT count is how many samples were lost
  during a stall; a stalled channel moves nothing, so the loss is the
  peripheral's to report (a converter's OVERRUN flag) and the engine
  says so rather than inventing a number.
- **Neither streaming engine kicks on its re-arm, and that is a
  correctness rule.** The beat that ended the block is the one that
  SERVED the peripheral's request, so the request is down; a kick
  there would move a beat the peripheral never asked for - the next
  table entry over a value not yet consumed, or a duplicate sample out
  of a data register holding nothing new. `kick()` is the owner's verb
  for the first arm and for the arm after an `abandon()`.
- **Both streaming engines take `volatile` buffer pointers.** The
  controller reads and writes this memory where the compiler cannot
  see it, and gcc has already been caught on this target sinking a
  store past a transfer. A plain array still converts for free.

## How to use it

Memory to memory, software triggered:

```cpp
brio::Dmac::init();
using Copy = brio::DmaChannel<0>;
Copy::configure({});                       // no trigger: software, TRIGACT block
Copy::load(brio::DmaTransfer{
    .source = src, .destination = dst, .beats = 256,
    .beat = brio::DmaBeat::byte,
});
Copy::arm(brio::DmaFlag::complete | brio::DmaFlag::transfer_error, true);
Copy::enable(true);
Copy::trigger();                           // one trigger runs the block
```

The block's one vector, dispatched by INTPEND:

```cpp
extern "C" void DMAC_Handler() {
    while (const auto irq = brio::Dmac::take_pending()) {
        (void)Serial::dma_isr(irq->channel);   // the engined Uart's channels
        // ... the app's own channels dispatch on irq->channel here
    }
}
```

Mid-block progress, checked against 1.10.4:

```cpp
if (const auto p = Copy::harvest()) {
    // p->done beats have landed; p->complete when the block ended
} else {
    // reading refused: a write-back inconsistency (counted in
    // violations()) or the suspend timed out - ask again next time
}
```

`Dmac::init()` comes before any engined Uart's `init()`: the engines
configure their channels at arm time, into a block that must already
own its tables.

A waveform played out of RAM and a sampled stream read back into it,
with the CPU in neither sample path:

```cpp
using Play = brio::DmaLoopEngine<0, uint16_t>;      // halfword beats
using Grab = brio::DmaPingPongEngine<1, uint16_t>;

Play::arm(&brio::Dac::regs().DAC_DATABUF, brio::Dac::dma_trigger_empty);
Grab::arm(&Adc0::regs().ADC_RESULT, Adc0::dma_trigger_resrdy);

Play::start(wave, 32);            // the table, played for ever
Grab::start(buf_a, buf_b, 24);    // two buffers, alternating

// THE OWNER'S ONE JOB AT THE FIRST ARM: a request that is already
// standing needs an edge, and only the owner can read its peripheral's
// flag. The DAC's "DATABUF is empty" is a request the stream WANTS
// served, so it is kicked; the ADC's "a result is waiting" is a
// conversion from before the stream existed, so it is DRAINED.
Play::kick();
if (Adc0::ready()) { (void)Adc0::result(); }
```

```cpp
extern "C" void DMAC_Handler() {
    while (const auto irq = brio::Dmac::take_pending()) {
        if (!irq->complete()) { continue; }   // an error is not a completion
        switch (irq->channel) {
        case 0: (void)Play::complete(); break;
        case 1: (void)Grab::complete(); break;
        default: break;
        }
    }
}
```

Draining, with the accounting the engine gives:

```cpp
if (const volatile uint16_t* block = Grab::ready()) {
    // Grab::ready_length() elements, complete and untorn
    consume(block, Grab::ready_length());
    Grab::release();          // restarts the stream if it had stalled
}
// Grab::overruns() counts the times both buffers were held; how many
// SAMPLES that lost is the converter's OVERRUN flag, not this counter.
```

An engine is a static-only class, so its state is shared across every
use of the same instantiation: a handler that tells the WRONG engine a
block finished reprograms a running channel. One channel, one engine,
and the switch says so.

## Bench findings

- The end-address arithmetic decided by data: the naive start-address
  descriptor moved the DECOY buffer (payload[0]=0x50, decoy[0]=0xA0,
  naive dst[0]=0xA0) - the "+ 1" of 25.10.3/25.10.4 is wrong and
  25.6.2.7 is the truth.
- Throughput at 48 MHz: a 256-byte word-beat block in 971 cycles (20
  us, ~12 MB/s including setup); software-linked chains re-armed from
  the TCMPL handler run 804 cycles/block (~59700 blocks/s back to
  back). A harvest measures ~525 cycles (~10 us), which is why its
  pacing is the caller's policy.
- **Erratum 1.10.4 does not merely give a bad READING - it kills the
  TRANSFER**, and that is what a wedged console eventually proved. A
  transmit channel was caught enabled with its peripheral's DRE and TXC
  both set (the transmitter idle and asking), CHSTATUS all zeros, no
  flag anywhere - and its write-back holding the OTHER channel's
  descriptor: BTCTRL 0x809 with SRCADDR = the SERCOM's DATA register,
  where its own says 0x409 and a RAM address. Since 25.6.2.6 makes the
  write-back the ongoing descriptor, that channel was running someone
  else's transfer and never finished it; `DmaTxEngine::busy()` stayed
  true, the owner's pump did nothing every time it was called, the
  transmit ring filled and `print()` spun in `Ring::push` with the
  board silent. The fingerprint was identical across three independent
  reproductions.
- **Erratum 1.10.4 caught in the act**, twice over: under five
  concurrent channels with the engined Uart running, 340 corrupted
  write-backs were refused out of 210852 readings in one four-second
  window - every one on the heavily-churned channel, and the captured
  bad write-back is a MIXTURE: another channel's SRCADDR and BTCTRL
  with the victim's own DSTADDR. Every transfer still landed
  byte-exact: the corruption is in the REPORTING, and validation
  turns it from wrong answers into refused readings. Under trigger
  densities an order of magnitude lower (the raw-channel stress:
  ~69000 harvest rounds, ~8600 churn blocks), violations stay ZERO -
  the counter is the measurement, and it scales with concurrency.
- **A channel that took a bus error loses the first beat of its next
  block** - deterministically (the block right after the error fails
  every time, 32 consecutive blocks after it are byte-exact), even
  though the channel was reset, the descriptor is correct and the
  write-back reports BTCNT=0. Documented nowhere in ch. 25. A channel
  recovering from TERR must spend one block and discard it; the
  driver deliberately does not hide that.
- The LVLEN trap was caught exactly as the header comment records it:
  two channels at level 1 sat still for four seconds while the
  level-0 ones ran nine thousand blocks - the per-level macro had
  masked the group write down to level 0.
- The silent-SWRST edge was caught by data too: a 16-beat block on a
  channel "reset" out of a still-draining disable lost its first
  beat, fifteen bytes correct, write-back cheerfully reporting zero
  remaining. Hence the bounded disable wait and the refusing reset.
- The harvest handshake had a measured race worth its critical
  section: `take_pending()` acknowledging INTPEND mid-harvest could
  steal the SUSP flag the wait was watching - roughly one loss per
  70000 harvests under load, exactly rare enough to be mistaken for
  silicon. The whole suspend-read-resume now sits in one critical
  section (~10 us of masked interrupts per harvest).
- A DMA buffer must be volatile in BOTH directions: gcc sank a plain
  zeroing store past the transfer that was supposed to overwrite it,
  so the check read pre-transfer values. The compiler cannot see a
  DMA store; both sides of shared buffers are volatile in the suite.
- INTPEND dispatch with two channels pending serves them lowest
  first, one loop turn each; an invalid-descriptor fetch raises
  CHSTATUS.FERR with TERR AND SUSP together, as 25.8.22's clause
  says (that FERR clears only on the RESUME command is 25.8.23's
  text, encoded but not separately measured).
- Zero when absent, measured not asserted: the release images of the
  apps that name no DMA (blink, console, probe) are byte-identical
  before and after this header and the Uart's engine parameters
  existed - and the 40 AVR hexes are byte-identical after ring.hpp
  gained the span API the TX engine drains through. The element-type
  generalization and the two streaming engines were held to the same
  gate: all 27 pre-existing SAM release images are byte-identical
  after them, and all 29 after the timers' pass, which added a suite and
  changed no line of code in this header.

From `test_samc_spi` letters d and h (the serial engines' second
SERCOM personality, and the block-request engines' own speed record):
`start_fixed()`/`start_discard()` are the two sibling verbs the SPI
host's null buffers wanted (one element sent `length` times, `length`
elements drained into one cell - each differs from start() by one
descriptor bit, and they are SIBLINGS rather than flags so every
pre-existing call site stays byte-identical); the full-duplex pair
moves a data phase byte-exact through 12 MHz in loop-back and carries
a two-board link exact to 6 MHz with both ends on engines - and the
SPI-mode kick inversion above is this campaign's finding.

From `test_samc_timer_dma` (10 letters, 101 verdicts, wireless), which
is the streaming engines' THIRD peripheral family and the one that
qualifies the trigger doctrine:

- **NOT EVERY PERIPHERAL PRESENTS ITS DMA REQUEST AS A LEVEL, and a TC
  CAPTURE CHANNEL DOES NOT.** 25.8.8 makes a trigger the RISE of a
  request the peripheral holds up, which is why `DmaTxEngine` needs
  `kick()` when a SERCOM's DRE already stands and why the ADC stream
  drains RESULT before arming. A TC capture's flag is INTFLAG.MCx and
  35.6.2.8 makes reading CCx the only thing that clears it, so an unread
  capture looks identical to a standing request - and it does not
  behave like one. Measured two ways: a `DmaPingPongEngine` **armed with
  MC0 already set filled two whole blocks in fifty waveform periods**,
  and - the half that rules out "selecting TRIGSRC over a high level
  looked like a rise" - a stream STALLED to a dead stop and then
  re-enabled by `release()`, **with CHCTRLB untouched and the flag still
  up**, picked straight up again. Every capture asks again, read or not.
  So `kick()` is harmless where it is unnecessary and necessary where it
  is not, and the rule that covers both is the one the engines already
  state: arm with the request drained, and the first beat is a fresh one
  either way.
- **A DMA beat is the acknowledgement a CPU read would have been.** Two
  ping-pong streams on one capture timer's CC0 and CC1 drained eight
  blocks each with **INTFLAG.ERR never rising**, where thirty periods
  with nothing read raised it in the same letter.
- **A `DmaLoopEngine` feeding a double-buffered peripheral register is
  one write per update window**, which is the whole reason it works: 192
  streamed captures matched the played duty table with a **worst error
  of zero ticks** and a phase that held across every lap and block
  boundary. Flooded with software triggers the same engine ran **1229
  laps in 20 ms against 25** paced by the peripheral - and reported
  nothing wrong, because every beat it moved, it moved: what was lost
  was a STORE the peripheral discarded, which is the owner's to know
  and not the controller's.
- **The element type must be as wide as the REGISTER, not as wide as the
  value.** A TCC compare register is 32 bits on a 24-bit counter, and a
  halfword write lands in the low half alone (0x00ABCDEF then a halfword
  0x1234 reads back **0x00AB1234**), so a duty stream's beat is a WORD.

From `test_samc_analog_dma` (10 letters in `z`, 72 verdicts, wireless -
PA02 is the DAC's VOUT pad and ADC0's AIN0 at once), on a chain where a
timer's overflow starts the DAC and its CC0 match starts the ADC, one
sample of each per 200 us period at 5 kHz:

- **A SOFTWARE-CLOSED LOOP LOSES NOTHING AT ITS SEAMS.** With a
  32-entry table and 24-sample blocks - chosen not to divide each
  other, so a lost sample cannot hide in a coincidence - the table
  entry each captured block starts on stepped by exactly 24 (mod 32)
  through every block of every run: 31, 23, 15, 7, 31, ... over 12
  blocks and 9 laps, and 166 blocks under churn. The worst residual
  against a static calibration of the same 32 codes was **3 to 6
  counts** where the ADC's own noise over eight readings was 4 to 6 and
  one table step is 120. So the TCMPL re-arm costs no sample at a lap
  boundary, at a block boundary, or between them - at this rate.
- **The rate is the pacer's**: 1992 samples in 400 ms, 4980/s against
  5000 nominal, inside the 2.5 parts per thousand a 1 kHz tick
  quantizes a 400 ms window by at each end. Both rulers are OSC48M
  here (the TC counts GCLK0 and SysTick the CPU clock), so this checks
  the divider arithmetic and NOT the oscillator - which the clock
  campaign put 5100 ppm slow against the board's crystal.
- **The two engines stay in step by construction**: 12 ADC blocks x 24
  and 9 DAC laps x 32 are 288 samples each, exactly, because one timer
  paces both.
- **The overrun contract, measured**: with the drainer asleep for
  60 ms the stream filled both buffers, counted one overrun, set
  `stalled()`, and STOPPED - and both held blocks still fitted the
  table within 2 and 4 counts, i.e. neither was torn. The ADC's own
  OVERRUN flag was set, which is where the count of LOST SAMPLES
  lives; the engine's counter is the count of STALLS. `release()`
  restarted it and the lap count moved again.
- **INTFLAG.EMPTY IS AN EVENT, NOT A STATE.** On a DAC that has just
  been enabled and whose DATABUF has never been written, EMPTY reads
  ZERO - the flag marks the buffer BECOMING empty, not being empty. A
  DMA-fed DAC that waited for the flag before its first kick therefore
  never starts, and pays an UNDERRUN and a lost period to find out.
  What makes the first kick right is the owner's own knowledge that it
  has just reset the converter, not the flag.
- **A TRIGGER IS AN EDGE, AND SELECTING TRIGSRC IS ONE.** With a
  conversion left unread so the ADC's DMA request stood as a level, a
  bare channel CONFIGURED onto it took the trigger immediately, with
  no software trigger at all: writing CHCTRLB.TRIGSRC moves the
  multiplexer's output from the DISABLE code's constant zero to the
  standing request, and that transition is the rise. Nothing in ch. 25
  says so. And a rise arriving while the channel was DISABLED with its
  trigger already selected was ALSO latched and served on the next
  enable. So neither arrangement wedged here - `kick()` is insurance
  at a first arm on this peripheral rather than a demonstrated
  necessity, and the sercom.md wedge is not contradicted (it was
  caught with a corrupted write-back in hand, which is a different
  fact).
- **Erratum 1.10.4 reached again, and the density that reaches it is
  the concurrency and not the traffic.** Two memory-to-memory channels
  re-armed with a BOUNDED WAIT for each block ran 43000 blocks
  alongside the live analog chain with **not one refused reading**;
  the same two channels SPRAYED - triggered without waiting, so they
  are genuinely concurrent with the analog pair - reached **2733 and
  2744 refused readings out of ~36000 harvests** in a 300 ms window on
  two runs of four. Every one was refused, never believed, and every
  analog block handed over still fitted the table. Whether it fires is
  the arbiter's weather, so the suite verdicts the invariant (refused
  == violations + timeouts) and never that a corruption happened.
- **`abandon()` IS NOT ALWAYS ENOUGH.** On the runs where the erratum
  struck hardest the analog stream did not come back from an
  abandonment: `abandon()` reclaims a channel by resetting and
  reconfiguring it, and **CHCTRLA.SWRST is ignored silently while
  ENABLE is still set** (25.8.18) while ENABLE itself does not clear
  until the internal buffer drains - so a channel the corruption has
  left unable to go down has nothing at the CHANNEL level left to try.
  A reset of the BLOCK (`Dmac::init()`) brought it back. The recovery
  ladder is therefore two rungs, and a program that streams under
  concurrency needs the second one.
- **The tight harvest loop is itself a stressor.** `harvest()`
  suspends the channel inside a critical section, so spinning on it is
  a concurrency contributor and not a neutral observation - one paced
  to about one per millisecond and one spun on differ by an order of
  magnitude in what they provoke. Pacing is the caller's policy for
  this reason as much as for the 10 us it costs.
- **A 24-bit datum needs a WORD beat, and the bench can show why.**
  SDADC.RESULT is a 32-bit register with 24 bits of data whose TOP
  sixteen are the specified conversion, so a halfword beat would carry
  RESULT[15:0] - not the reading at all. Driven to a rail differential
  the raw value is 8388607, which does not fit a halfword; streamed
  through `DmaPingPongEngine<ch, uint32_t>` every word matched the
  CPU's reading exactly. With the pair shorted at ground - where the
  datum is live and its low bits move - the streamed sixteen readings
  spanned 102 to 305 raw units around a CPU mean of about -34600, i.e.
  the same quantity inside the spread the CPU itself showed. TSENS,
  whose VALUE is the same shape, streamed through the same engine at
  2490..2527 centi-C against a CPU reading of 2507.

## Not covered yet

Driver gaps (not built):
- The CRC engine (CRCCTRL/CRCDATAIN/CRCCHKSUM/CRCSTATUS): util/crc.hpp
  already serves this repo's polynomials; the hardware engine (with
  its rev-B-only data-port erratum) waits for a consumer.
- Linked descriptor lists: legal on rev F, but 1.10.4 makes
  software-linked chains (TCMPL re-arms the next block) the honest
  default, and nothing has needed a chain they could not build. This
  is also why there is NO HARDWARE CIRCULAR MODE here - a self-linked
  descriptor is the only one chapter 25 offers - and why
  `DmaLoopEngine` closes its loop from the interrupt.
- (The util-level contract these engines satisfy is
  `util/block_stream.hpp` - `BlockSource`/`BlockPlayer` plus the
  `BlockRelay` AO, design/block-stream.md. Both engines are
  concept-checked in the family fixture and the relay runs over this
  chain in `test_samc_analog_dma`'s kernel letter; the contract speaks
  blocks, not DMA, and exists as the fixed point the next platform's
  stream machinery is measured against.)
- An automatic recovery ladder. The bench established that a channel
  the erratum has left unable to clear ENABLE cannot be reclaimed at
  the channel level and needs `Dmac::init()`, and the suite spends
  that rung by hand - but no verb here escalates on its own, because
  resetting the block stops every OTHER channel too and that is a
  program-wide decision, not a driver's.
- The event system hooks, PARTLY RETIRED: an EVSYS driver now exists
  ([evsys.md](evsys.md)) and `test_samc_evsys` drives a DMA channel
  with EVACT `trigger` and EVIE set from a software event, so that
  path is silicon-tested. EVOE and EVOSEL - the DMAC as an event
  GENERATOR - and the other EVACT values remain untested from here.
- QOSCTRL (left at reset), RUNSTDBY and the 25.6.7 standby sequence
  (the power pass owns sleep on this target).
- Trigger codes beyond the two SERCOM pairs - each arrives with the
  driver that owns its peripheral.

Implemented but not bench-verified:
- Step sizes beyond x1 and STEPSEL=source (the arithmetic is
  fixture-pinned, no bench letter walks a strided buffer).
- Round-robin arbitration and DBGRUN (`DmacConfig` writes them; no
  letter measures them).
- The E and G variants (compile-checked by the family fixture only).
