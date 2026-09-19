# True random number generator (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.12 (the
generator: 12.12.1 what it is and what it yields, 12.12.2 the two
settings and the chapter's own working values, 12.12.3 the operating
sequence, 12.12.4 the caveats and the two reference drivers - 12.12.4.1
the bootrom's, which bypasses every check on purpose - 12.12.5 the
registers), 7.5 (the reset controller), 3.2 (the interrupt line). No item
of Appendix E is filed against this block. The driver:
`brio/rp2350/trng.hpp`. The reference suite: `test_rp2350_blocks`,
letters e, f and g, which runs on both of this chip's architectures from
one source.

## What the silicon does

ARM IP, NOT RASPBERRY PI'S - which is why its register names read unlike
anything else in this stratum and why its offsets start at 0x100 rather
than at zero.

A free-running ring oscillator with **no connection to the system clock
tree**, sampled every SAMPLE_CNT1 system clock cycles. The samples pass
three entropy checks and a von Neumann decorrelator, and 192 accepted
bits at a time appear in the six EHR_DATA registers. 12.12.1 gives the
yield as about 7.5 kb/s with the core at 150 MHz: a source to SEED with,
not a stream to read from.

**The three checks are all on at reset, and they are not errors in the
ordinary sense.** They are the block refusing to hand out entropy it does
not trust, and a driver that treated them as failures would give up where
it should go round again:

| Flag | What it means | What a program does |
|------|---------------|---------------------|
| `AUTOCORR_ERR` | the autocorrelation test failed FOUR TIMES IN A ROW | nothing: "RNG ceases functioning until next reset", and RNG_ICR's own description says this bit "cannot be cleared by SW". The internal soft reset is the only way back |
| `CRNGT_ERR` | two consecutive blocks of sixteen collected bits were equal | clear it; the run was discarded and the next has started |
| `VN_ERR` | thirty-two consecutive collected bits were identical | the same |

So one failure is fatal to the block until it is reset and two are
ordinary weather. The driver reports each kind separately, goes round
again on the two recoverable ones and counts them, which makes "how often
does this silicon discard a run" a number rather than an impression.

**Generation time is not deterministic** (12.12.4). The mean and the mode
are close, but a run can take a hundred times the average, and a failed
check discards the run and starts another. The chapter's own advice is to
keep a small pool and refill it in the background rather than to block.
Every wait in this driver is therefore bounded and says so when it runs
out, and no verb of it belongs in an interrupt handler with a deadline
behind it.

**THE SAMPLING INTERVAL IS A TIME, AND THE CHAPTER'S ADVICE IS A CYCLE
COUNT - which is why a program that takes 12.12.2 literally at 150 MHz
stops the block.** TRNG_CONFIG picks one of four inverter-chain lengths
and SAMPLE_CNT1 the clk_sys cycles between two samples. The chapter's
worked values are "ROSC chain length settings of 0 or 1 and sample count
settings of 20-25" for an average generation time of about 2 ms, with no
clock rate stated beside them. Measured here (below), an interval under
about 800 ns makes the autocorrelation test fail four times in a row
within microseconds and the block ceases functioning. The chapter's two
figures together name their own clock: a collection takes some 680
samples (below), and 680 samples of 25 cycles in 2 ms is a clk_sys near
8.5 MHz, the order of the ring oscillator this chip boots on. **25 cycles
is 2.9 us there and 167 ns at 150 MHz**, and only the first of those
works. So this driver states the
setting as a TIME - `trng_default_sample_ns`, 2 us - and turns it into a
count for the family's highest clk_sys, which is at or over the floor at
every rate below it; `trng_sample_cycles_for(hz)` is the same arithmetic
for a program that knows its own rate and wants the fastest legal
setting. Longer is slower and never wrong.

**The one combination that is wrong rather than slow.** With the von
Neumann decorrelator BYPASSED, SAMPLE_CNT1 "must not be less than
seventeen" - a rule and not a preference, and independent of the
interval above. This driver refuses that combination at compile time
when the settings are constant and at run time when they are not.

**The internal soft reset needs a delay.** 12.12.4.1's listing writes
TRNG_SW_RESET and then reads the register twice, with the comment that a
fixed delay is required after it. The reset also returns every setting to
its reset value (SAMPLE_CNT1 to 0xffff), which is why the recovery verb
is the reset AND the settings again.

**Reading EHR_DATA5 is what clears the result registers** (12.12.3), so
the six words are read in order 0..5 and never out of it. Until a
collection completes they all read zero, so the CPU cannot take a
half-collected number.

**What the bootrom does instead, and why this driver does not.** The ROM
streams RAW ring-oscillator samples past every check straight into the
SHA-256 accelerator (12.12.4.1), because it must boot in bounded time and
because a hash is a better conditioner than the decorrelator. That is a
legitimate use of the same silicon under a different contract; this
driver is the block as 12.12.3 specifies it, checks and all. The 128-bit
per-boot random number the ROM produced that way is available through
[bootrom.md](bootrom.md)'s `get_sys_info`, and is the cheap seed a
program usually wants.

One interrupt line, masked at reset (every RNG_IMR bit resets to one, and
a ONE masks).

## Types and verbs

- `TrngConfig` - the chain length, the sample count and the three checks.
  The sample count defaults to the measured-safe INTERVAL at the family's
  highest clk_sys, not to the chapter's cycle count.
  `trng_config_legal(cfg)` is the rule both the compile-time and the
  run-time entry points judge by, written once.
- `trng_min_sample_ns`, `trng_default_sample_ns` and
  `trng_sample_cycles_for(sys_hz, ns)` - the floor, the default and the
  arithmetic between a sampling interval and SAMPLE_CNT1, rounded up.
- `Trng::init(cfg)` / `Trng::init<cfg>()` - the block cycled through its
  subsystem reset, the IP's soft reset with its delay, the settings, every
  interrupt masked. The source is left STOPPED. The templated form
  refuses an illegal setting at compile time, naming the rule.
- `Trng::start()` / `stop()` / `running()` - the ring oscillator.
  12.12.3 asks that it be stopped when the block is not in use.
- `Trng::read()` - one attempt at 192 bits, with the protocol of 12.12.3
  in the order it is written. Nothing, plus `last_error()`, when there
  is none: `TrngError::not_ready`, `autocorrelation`, `crngt`,
  `von_neumann`, `misconfigured`.
- `Trng::read_blocking(spins)` - the same, going round again on the two
  recoverable checks within a bounded budget. `recoverable_failures()`
  is how many runs the block has discarded since `init()`.
- `Trng::sw_reset()` - the IP's internal reset and its required delay.
  `Trng::recover(cfg)` is that plus the settings again, and is what a
  program calls when a read answers `autocorrelation`.
- `Trng::busy()`, `valid()`, `status()`, `clear(flags)`,
  `interrupt_mask()`, and `Trng::isr()` - the ISR body, which reports the
  four sources and clears the two that can be cleared.
- What the block says about itself: `version()` with
  `ehr_is_192_bits()`, `has_autocorrelation()` and `has_crngt()` beside
  it; `autocorr_stats()` (the tests started and the ones that failed, in
  one register that ANY write resets); `bist()`, the three ring
  oscillator counters; `chain()`, `sample_cycles()`, `debug_control()`
  and `debug_mode()` read back.
- `reset_bit_counter()` - RST_BITS_COUNTER, which refuses while the
  source is enabled, because 12.12.5 says the enable must be clear for
  the reset to take.
- `TrngFlag` - the four bits the three interrupt registers share, with
  `all` and `recoverable`.

## How to use it

```cpp
brio::Trng::init();                       // the block and the settings
brio::Trng::start();                      // the ring oscillator

if (auto e = brio::Trng::read_blocking()) {
    seed(e->words);                       // six words, 192 bits
} else if (brio::Trng::last_error() == brio::TrngError::autocorrelation) {
    brio::Trng::recover();                // the only cure for that one
}

brio::Trng::stop();                       // it is a waste of power idle
```

A program that knows its own clock and wants the shortest legal
interval asks for it in nanoseconds rather than in cycles:

```cpp
brio::Trng::init({.sample_cycles = brio::trng_sample_cycles_for(brio::clock_hz(clock))});
```

With the settings a constant, so a mistake is a compile error:

```cpp
brio::Trng::init<brio::TrngConfig{.chain = 0,
                                  .sample_cycles = brio::trng_sample_cycles_for(150'000'000)}>();
```

## Bench findings

On an RP2350 in the QFN-80 package, **stepping A2**, clk_sys at 150 MHz,
on BOTH architectures: `test_rp2350_blocks` reports **71 pass, 0 fail**
on the Cortex-M33 pair and on the Hazard3 pair, from one source, each on
three flash-and-run cycles. Every verdict of this chapter reads the same
on the two halves; where a number differs, both are given.

- **THE SAMPLING INTERVAL, AND WHERE THE DATASHEET'S ADVICE BREAKS.**
  With clk_sys at 150 MHz, SAMPLE_CNT1 swept from 25 to 65535 at each of
  the four chain lengths, ten runs a point: at **25, 50 and 100 cycles
  every run ends in AUTOCORR_ERR** - AUTOCORR_STATISTIC reads four tries
  and four failures, inside a few microseconds, and the block then hands
  out nothing until it is reset. At **110 cycles seven runs in ten** do.
  From **120 cycles up, none of ten** does, at chain 0, 1 and 3 alike.
  120 cycles at 150 MHz is **800 ns**, and the chain length moves the
  boundary not at all: it is the INTERVAL that matters. 12.12.2's "sample
  count settings of 20-25" is therefore right only at the clock its own
  "about 2 milliseconds" implies - near 8.5 MHz, 2.9 us a sample - and
  wrong by an order of magnitude at this one. The default here is 2 us
  (300 cycles at 150 MHz), two and a half times the floor.
- **WHAT 192 BITS COST at that default.** A collection takes **1360 to
  1427 us on the Cortex-M33 half and 1351 to 1379 us on the Hazard3
  one** - some 680 samples of 300 cycles each, three and a half for every
  bit the decorrelator keeps -, sixteen of them in 21.6 to 22.8 ms -
  **133 to 140 kbit/s**,
  which is some eighteen times the 7.5 kb/s 12.12.1 states and slightly
  under 12.12.2's "about 2 milliseconds". Not one run in the six
  three-cycle passes was discarded on a CRNGT or von Neumann check
  (`recoverable_failures()` is 0 every time), and the autocorrelation
  counters read **16 tries 0 failures** in five of the six passes and
  17/1 in one: the test does still fire occasionally at 2 us, four in a
  row is what would be fatal, and nothing came near it.
- **A CRUDE QUALITY MEASUREMENT on 3072 bits** - a stuck-source check and
  not a certification. The monobit fraction lands between **48.9 and 51.1
  per cent** across the six passes, the sixteen nibble values all occur
  (counts 31..64 against an even 48), no word is zero and no word repeats
  the one before it.
- **THE FATAL CHECK, RAISED ON PURPOSE AND CURED.** The suite sets
  SAMPLE_CNT1 to 25 for one collection: AUTOCORR_STATISTIC reads **four
  tries and four failures**, `read_blocking()` answers
  `TrngError::autocorrelation` rather than running out its budget, and a
  write of every bit to RNG_ICR **leaves the flag standing** - which is
  the register description's own sentence, measured. `recover()` is the
  way back: entropy again, the flag gone.
- **THE SOFT RESET AND THE WAY BACK.** TRNG_SW_RESET returns SAMPLE_CNT1
  to 0xffff and TRNG_CONFIG to 0, as the register descriptions say;
  `recover()` puts both back, and the first collection after it arrives
  in **1473 to 1860 us** - the ordinary time, so nothing of the source is
  left disturbed. RST_BITS_COUNTER refuses while the source is enabled
  and takes the write when it is stopped, which is 12.12.5's own
  condition.
- **RNG_VERSION reads 0xF**: the 192-bit entropy holding register, the
  autocorrelation unit and the CRNGT both present. TRNG_DEBUG_CONTROL and
  RNG_DEBUG_EN_INPUT read zero after `init()`, so no check is bypassed
  and the IP's debug mode is off. The three BIST counters read **0x2, 0x0
  and 0x0** and do not move.
- **TRNG_BUSY IS TWO BITS WIDE HERE**, where 12.12.5 declares 31:1
  reserved: with a collection under way at SAMPLE_CNT1 = 65535 the
  register reads **0x3**, not 0x1. The driver masks the documented bit,
  so nothing depends on the other one.

## Not covered yet

Driver gaps, each with its reason:

- **The interrupt as a working path.** `isr()` and the mask are here and
  the vector is `isr_trng`, but every letter of the suite polls: a
  collection takes milliseconds and the program has nothing else to do
  while it waits. Born with the first program that keeps a pool topped
  up in the background, which is the shape 12.12.4 recommends.
- **The raw-sample path of 12.12.4.1** (the checks bypassed, the samples
  hashed): declined. It is the bootrom's contract and not this
  chapter's, the ROM has already done it once per boot, and the result
  is readable through `get_sys_info`. A program that wants more entropy
  faster than the checked path gives it would build this, with the SHA-256
  driver beside it.
- **The IP's debug mode** (RNG_DEBUG_EN_INPUT) and the BIST counters as
  anything but a printed number: no documentation of what they mean
  beyond the register description is on the desk, and a debug mode that
  changes what the source does is not a thing to turn on without one.
- **A `util` contract for an entropy source.** One family does not make
  a contract; the STM32F4's `Rng` and this block answer different
  questions (one word against 192 bits, a continuous test against three
  checks). It would be born at a third.

Implemented but not bench-verified, each with what would measure it:

- **The run-time refusal of an illegal setting.** The family check
  refuses every one of them at compile time; `init(cfg)` returning
  `false` with `TrngError::misconfigured` has no letter, because the only
  way to reach it is a setting a suite would have to build on purpose out
  of a run-time value. A letter that fed it one would close this.
