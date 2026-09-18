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

**The settings, and the one combination that is wrong rather than slow**
(12.12.2). TRNG_CONFIG picks one of four inverter-chain lengths and
SAMPLE_CNT1 the cycles between samples. The chapter's worked values are
"ROSC chain length settings of 0 or 1 and sample count settings of 20-25"
for an average generation time of about 2 ms; larger counts trade time
for quality. But with the von Neumann decorrelator BYPASSED, SAMPLE_CNT1
"must not be less than seventeen" - which is a rule and not a preference,
and this driver refuses that combination at compile time when the
settings are constant and at run time when they are not.

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

- `TrngConfig` - the chain length, the sample count and the three checks,
  with the chapter's own working values as defaults.
  `trng_config_legal(cfg)` is the rule both the compile-time and the
  run-time entry points judge by, written once.
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

With the settings a constant, so a mistake is a compile error:

```cpp
brio::Trng::init<brio::TrngConfig{.chain = 0, .sample_cycles = 20}>();
```

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

Implemented but not bench-verified, each with the letter of
`test_rp2350_blocks` that will measure it:

- What the block says about itself (RNG_VERSION's width and check bits,
  the settings read back, the debug controls clear), the time a 192-bit
  collection takes, the rate in bits per second, how many runs the
  silicon discards on its own checks, and the autocorrelation statistics
  (letter e).
- A crude quality measurement on 3072 bits - the monobit fraction, the
  spread over the sixteen nibble values, no zero word, no word repeating
  the one before it. A MEASUREMENT AND NOT A CERTIFICATION: it would pass
  for a counter, and it is there to catch a source that is not running
  (letter f).
- The soft reset returning SAMPLE_CNT1 to 0xffff, `recover()` putting the
  settings back, entropy again afterwards, and RST_BITS_COUNTER refusing
  while the source is enabled (letter g).
- The compile-time and run-time refusals of an illegal setting: the
  family check refuses them, no board has answered `false` from
  `init(cfg)`.
- `AUTOCORR_ERR` itself. Nothing here can provoke the fatal check - it
  wants four consecutive autocorrelation failures from a physical source
  - so the branch that reports it and the recovery it demands are written
  and untested against the real flag. The autocorrelation statistics
  letter e prints are the nearest thing: a board that fails the test
  often would show it there first.
