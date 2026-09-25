# Random number generator (CH32V303)

A random number generator on an analog noise source, three registers and
one vector, on two parts of this stratum. Documents of record: the
CH32F/V20x_V30x_V31x reference manual V2.3 - chapter 29 whole, which
says it applies to the whole family; figure 3-3, the clock tree of the
CH32V30x_D8 class; 3.3.5.8 and 3.4.12 for the clock selector that
belongs to another class; 3.4.6 and 3.4.11 for the gate and the reset
line this block has not got - and the CH32V303 datasheet V3.5, table
2-1-1, whose RNG row says which parts carry one, and figure 2-4, its
own clock tree. Driver:
[brio/ch32vx03/rng.hpp](../../brio/ch32vx03/rng.hpp) (`Rng`,
`RngError`, `RngEvent`). Family fixture: `test/family_ch32vx03/rng.cpp`,
with a negative that refuses the type on every part without the block.
Reference suite: `test_vx03_rng`.

## What the silicon does

**Two parts have it.** The CH32V303 datasheet's table 2-1-1 gives the
RNG to the CH32V303RC and VC and a dash to the CB and the RB - the same
device class - and the CH32V203's table 2-1 has no RNG row at all. So
`device::has_rng` is a part fact, and on the eleven parts without the
block the type does not exist rather than answering false.

**It is a generator with an analog heart.** "Based on continuous analog
noise" (29.1): an analog circuit produces a seed, the seed is shifted
into a linear feedback register, and when "a large number of seeds are
introduced" the register's content is handed over as a 32-bit word
(29.2). Beside the datapath sit two monitors, one on the seed and one on
the clock, whose whole purpose is to say when a word is not to be
trusted. Three registers at 0x4002 3C00 (table 29-1): RNG_CR with two
bits, RNG_SR with five, RNG_DR with the word. No chapter-11 table names
a DMA request for it, and it raises no event.

**SYSCLK runs it.** Chapter 29 says the LFSR is clocked "by a dedicated
clock (PLL48CLK)" and names that clock again in the two clock-error
bits - which on this family could only be the 48 MHz the PLL makes
through USBPRE. The class's own clock tree, figure 3-3 - "applied for
CH32F20x_D6, CH32F20x_D8, CH32V20x_D6 and CH32V30x_D8" - and the
datasheet's figure 2-4 draw SYSCLK to the TRNG instead, and the one
register that would choose, RCC_CFGR2's RNG_SRC (3.3.5.8), belongs to a
register 3.4.12 gives the D8C classes and not this one. The two
readings predict different numbers, and the silicon gave the figures'
(measured on the CH32V303VC): from a read of RNG_DR to the next DRDY,
the same number of CORE CYCLES at 144, 96 and 48 MHz off the PLL, where
the 48 MHz reading would have run three to two to one; the same again
with USBPRE dividing the PLL by 2 or 1 instead of 3, which would have
sped the generator up under the other reading; and with no PLL at all,
SYSCLK on the bare HSI, words kept coming with the clock monitor quiet.
So the driver asks nothing of the clock tree: `init()` takes no clock.

**The gate, and no reset line.** RCC_HBPCENR.RNGEN (3.4.6, bit 9) opens
the block's bus clock. RCC_AHBRSTR has no bit for it - its [11:0] are
reserved (3.4.11) - so there is no peripheral reset, and the way back is
RNGEN cleared and set.

**The first value is zero.** FIPS PUB 140-2 asks that the first word
after an enable be kept for comparison and not used, and that every word
after it be compared with the one before, a repeat failing the test.
Chapter 29 does not ask it; this silicon does. Measured: the first word
after RNGEN is set is 0x00000000 on every enable that follows a disable,
some sixty core cycles after it, at every rate - while the FIRST enable
after a reset gave that zero at every boot of one day and a nonzero
word at every boot after a power cycle, a difference the bench has not
explained; and RNGEN cleared freezes the block - the word that stood
across the disable stays readable and no other lands behind it. So a RESTART - `init()` of a generator that is off, and `recover()` -
takes the standing word between the disable and the enable and
discards TWO words after it, the zero and the one that follows: a
restart that discarded one handed the zero out once in some thirteen
thousand on that silicon, with the cause never caught again, and one
word is the price of it never happening. The last word discarded is
what the next is compared with, and `read()` refuses a repeat.

**The two errors are not the same kind**, and 29.2.2 says so. A CLOCK
error (CECS) stops the generator but "has no effect on the last random
number and can be used normally": fix the tree, clear CEIS, carry on. A
SEED error (SECS - "more than 64 identical consecutive bits; more than
32 consecutive alternating 0's and 1's") makes the word in RNG_DR
unusable, and the way back is a sequence: "the SEIS bit needs to be
cleared first, then the RNGEN bit is cleared and set to 1". `recover()`
is that sequence. Neither monitor has been seen to fire: SYSCLK never
stops, not even across a switch of the clock task, which parks on the
HSI - CEIS stayed clear across every switch and every move of the USB
prescaler - and the seed test watches for runs, which the words below
do not have.

**The latched flags are cleared with a zero, and a one does nothing.**
29.3.2 calls SEIS and CEIS "RW" and names no clearing rule; WCH's own
library clears one by storing the register with that bit zero and every
other bit one. Measured: a one written into either flag sets neither,
and the two reserved "RW" bits [4:3] keep nothing written into them. So
the vendor's store clears exactly the one flag, and the other - should
it latch between a read and the store - is written a one and kept: the
clearing verbs and the ISR body use it. The same measurement means no
program can raise either flag, and the error paths are reached by the
silicon alone.

**One vector for three events.** RNG_CR.IE raises line 63 of this
class's table on a word ready or on either error, with no per-source
enable. A ready word keeps DRDY up until RNG_DR is read, so a handler
that does not take the word is entered again at once.

**THE WORDS ARE NOT TO BE TAKEN AS RANDOM BITS.** Measured on the
CH32V303VC, and the finding of this chapter:

- DRDY rises again over an UNCHANGED word, the more often the faster
  the words are taken: 6 to 15 in a hundred of the words read raw as
  fast as the core polls DRDY equal the one before, one to sixteen in
  a thousand through `read()` at its own pace - the refusals its
  comparison makes are real ones, not a formality.
- The words come from a SMALL SET. Four thousand words hold only 900 to
  3350 distinct values - fewest at 144 and 96 MHz, most on 8 MHz - read
  at DRDY's pace or 100 us apart alike, with a single value seen as
  often as twenty-five times; where a source of thirty-two random bits
  a word repeats one among four thousand with odds of about one in half
  a million. Pacing the reads does not help: 256 words ten milliseconds
  apart held 207 to 235 distinct values, at 144 MHz from the HSI, at 48
  MHz and at 8 MHz from the crystal. Nor does the clock's source, nor
  the USB controller's gate, nor the converter powered beside it.
- So the statistics a random source passes fail: a chi-square over the
  256 values of 65536 bytes near 49000 on 255 degrees of freedom; FIPS
  PUB 140-2's poker test at 100 to 117 against its 46.17 and its runs
  test outside its intervals; the 32 bit lanes seven and more standard
  deviations off balance at 144 MHz. The monobit and long-run tests
  pass.

The driver does not condition the words: what it hands out is what the
block produced, a word that repeats its predecessor excepted, and a
program that needs randomness from it decides how many words make one
it can use.

## Types and verbs

| Name | Meaning |
|------|---------|
| `RngRegs`, `rng_regs()` | table 29-1's three registers |
| `rng_rngen`, `rng_ie` | RNG_CR's two bits |
| `rng_drdy`, `rng_cecs`, `rng_secs`, `rng_ceis`, `rng_seis`, `rng_latched` | RNG_SR's five |
| `RngError` | why an attempt gave nothing: not ready, seed error, clock error, repeated |
| `RngEvent` | what one interrupt found: ready, seed error, clock error |

`Rng`, the monostate, exists only where the part has the block:

| Purpose | Verbs |
|---------|-------|
| the gate | `clock(bool)`, `clock()` |
| the vector | `irq` |
| the generator | `enable(bool)`, `enabled()` - raw, with no discard |
| the interrupt | `interrupt(bool)`, `interrupt()` |
| the statuses | `status()`, `ready()`, `seed_error()`, `clock_error()` |
| the latched flags | `seed_error_flag()`, `clock_error_flag()`, `clear_seed_error()`, `clear_clock_error()` |
| taking a word | `read()`, `read_blocking(limit)`, `value()` (raw), `last_error()` |
| bring-up | `init()`, `discard_first(limit)` |
| after a seed error | `recover()` |
| the ISR body | `isr()` |
| teardown | `release()` |

The type is a template with one instance, `RngUnit<1>`, so that its
refusal waits for a use: the header compiles on every part, and naming
`Rng` on a part without the block is the compile error.

## How to use it

Bring the generator up and take words:

```cpp
if (!brio::Rng::init()) {                    // gate, enable, the discards
    // brio::Rng::last_error() says why
}
if (const auto v = brio::Rng::read()) {
    use(*v);
} else if (brio::Rng::last_error() == brio::RngError::seed_error) {
    (void)brio::Rng::recover();
}
```

Under the interrupt, the handler binding the vector the crt names
`rng_handler`:

```cpp
brio::Rng::interrupt(true);
brio::Pfic::enable(brio::Rng::irq);
// in the handler:
const brio::RngEvent e = brio::Rng::isr();   // the latched flags reported and cleared
if (e.ready) { queue(brio::Rng::value()); }  // taking the word puts DRDY down
```

What the words are worth is the last paragraph of the section above:
a program that wants random bits out of them folds several into one.

## Bench findings

`test_vx03_rng` measured the block on a CH32V303VCT6 with nothing wired;
every letter costs the board nothing, so `z` carries all ten, and letter
c moves the clock tree and puts it back.

- **The block** (letter `a`): the gate shut after `release()` and open
  after `clock(true)`; with RNGEN set the first word stood 62 to 70 core
  cycles later - 0x00000000 at every boot of one day, 0x8FC7E5B0,
  0xE31CEF9D and 0xE2AA5943 at three boots after a power cycle (the
  zero of a re-enable is letter `j`'s, 256 of 256); `init()` left no
  error standing, current or latched; the vector is line 63. At the
  first boot after that power cycle RNG_SR read 0x40 BEFORE anything was
  enabled: a seed error latched by the supply's excursion and kept
  across the reflash's system reset, which no reset line exists to
  clear - the recovery letter's `recover()` took it down, and a boot
  path that finds SEIS standing does the same.
- **The word** (letter `b`, at 144 MHz): DRDY down right after a read
  of RNG_DR and up again 9 to 55 core cycles later, 20 on average;
  2.5 to 2.6 million words a second through `read()`, some seven
  thousand a second refused as repeats and none for an error; of 20000
  words read raw at DRDY's pace, 1179 and 3017 equal to the word before
  in two runs.
- **The clock** (letter `c`): the word time 9 to 11 core cycles at
  least, 19 to 25 on average and 52 to 55 at most at 144, 96 and 48 MHz
  alike; 19 to 21 on average with USBPRE at /2 and /1 under the running
  PLL, each write read back; on the bare HSI at 8 MHz `init()` started
  the generator from off and 882 to 885 words came in 5 ms with CECS
  seen in no status read and CEIS clear; across three switches of the
  clock task, two moves of the prescaler and the way back from the HSI,
  CEIS latched nothing and the generator went on by itself.
- **The small set** (letter `d`): 2281 and 2324 distinct values among
  4096 words through `read()` in two runs, the most frequent seen 13
  times; the 32 lanes over 20000 words held 9640 to 10692 ones across
  three runs - the worst lane 7.2 to 9.7 standard deviations off.
- **FIPS PUB 140-2** (letter `e`, 20000 bits): monobit 9899 to 10149
  ones, passing; poker 99.75 to 117.03 against 46.17, failing; the runs
  outside their intervals, failing; the longest run 17 or 18, passing.
- **A chi-square** (letter `f`): over 65536 bytes each value seen 22 to
  1294 times against 256 expected, the statistic 48491.8 to 49200.2
  across three runs.
- **The continuous test** (letter `g`): of 100000 words `read()` handed
  out none equal to the one before it and refused 84 to 1559 as
  repeats, with no error.
- **The latched flags** (letter `h`): RNG_SR read 0x1 before, after a
  one written into SEIS, after a one into CEIS and after ones into
  [4:3]; zeros cleared both; words kept coming.
- **The vector** (letter `i`): IE over a running generator gave a
  thousand entries and a word taken at each within 1 ms, no seed or
  clock event; a one written
  into SEIS latched nothing, so the seed-error path is not reachable
  from the program.
- **The restarts** (letter `j`): RNGEN cleared and RNG_DR read, 256
  times over a standing word and 256 over one being computed - no word
  landed after the read; `recover()` 256 times and `init()` over a word
  left standing by a disable 64 times, each followed by a first word
  that was never zero; with RNGEN cleared over a ready word, the word
  stayed readable and no new one came in 10000 polls; `release()` shut
  the gate and `init()` brought the block back.
- **What does not change the words**, measured beside the suite: reads
  paced 0 to 100 us apart and 10 ms apart, SYSCLK from the HSI and from
  the crystal, the USB controller's clock gate open, the converter
  powered with its internal sources on.

## Not covered yet

Driver gaps, each with its reason:

- **Conditioning the words into random bits**, born with its first
  user: the block's words fail the statistics (above), and how many of
  them a program folds into one, and with what, depends on what the
  program needs the bits for. The driver hands out the block's words
  and says what they are worth.

Implemented but not bench-verified, each with what would measure it:

- **The seed error and its recovery under a real fault.** SECS is
  raised by the analog part alone - a one written into SEIS does
  nothing - and the recovery sequence has run only on a healthy
  generator; what would measure the real path is a fault this project
  has no way to cause.
- **The clock error.** CECS never rose at any rate from 8 to 144 MHz or
  across any switch, SYSCLK being what runs the block and never
  stopping while the core runs; what would measure it is a condition
  that stops the generator's clock with the core alive, which this
  class may not have.
- **The words of another die.** Every statistic above is one
  CH32V303VCT6's; what would measure another is `test_vx03_rng`'s
  letters d to g on another CH32V303RC or VC - a die whose words pass
  turns those letters red, which is what they are for.
- **The CH32V303RC.** The same part table as the VC's but for the
  package; what would measure it is that part on a board.
