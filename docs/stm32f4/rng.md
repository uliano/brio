# Random number generator (STM32F4)

Documents of record: RM0090 Rev 22 ch. 24, whose first line says the
chapter "applies to the whole STM32F4xx family" - as far as the family
has the block at all: RM0390 Rev 6 never mentions it, because the F446
has none. No item of ES0206 Rev 24, ES0298 Rev 8 or ES0287 Rev 6 is
filed against it. Driver: `stm32f4/rng.hpp` (`Rng`, `RngError`,
`rng_clock_ratio_ok`), over the reserve's `rng_present`,
`rng_clock_on_ahb1`, `rng_clock_mask`, `rng_reset_mask` and `rng_irq`
(`stm32f4/device_tables.hpp`). The family fixture is
`test/family_stm32f4/rng.cpp` with the negatives that refuse the type on
a part with no generator and refuse a clock with no PLL. Bench:
`test_stm32f4_misc`, letter `b`, run on the STM32F429 (the one part of
the three on the desk that carries the block).

## What the silicon does

**It is a TRUE generator, not a sequence.** Several ring oscillators
whose outputs are XORed produce a seed; the seed is shifted into a
linear feedback register; the register is handed over 32 bits at a time
when "a significant number of seeds have been introduced" (24.3). Beside
the datapath sit two monitors - one on the seed, one on the clock -
whose whole purpose is to say when the entropy is not to be trusted.
ST states the block passed the FIPS PUB 140-2 tests with a 99% success
ratio.

**Three of the twenty-three parts have not got it.** The F401, the F411
and the F446 declare no RNG_BASE, and on those parts the resource does
not exist rather than answering false. Everywhere else it does - the
F405 class, the F410, the F412, the F413/F423, the F42x/F43x and the
F469/F479.

**It has a clock of its own, and the 48 MHz domain is where that comes
from.** The LFSR is clocked by RNG_CLK "at a constant frequency, so that
the quality of the random number is independent of the HCLK frequency"
(24.3), and on this family that clock is the main PLL's Q output - the
domain the USB OTG FS and the SDIO share, which `Clock::usb_hz` reports.
THE MANUAL'S ONLY STATED CONSTRAINT ON IT IS A RATIO, NOT A RATE: the
clock monitor raises CECS when f(RNG_CLK) < f(HCLK)/16 (24.4.2). 48 MHz
is what the clock tree is designed to put there and what every ST
example uses; a SYSCLK of 180 MHz gives 45 MHz there, which the ratio
still admits by a factor of four. So `Rng::init()` refuses a rate with
no PLL at all and refuses one that would trip CECS, and does NOT insist
on 48 MHz - a number the chapter does not make a condition. (Every rate
this family's PLL can produce clears the bar comfortably: the Q search
never leaves the output below 24 MHz, and 24 x 16 is well above the
180 MHz ceiling.)

**The clock gate is on a different bus on one part.** RNGEN is bit 6 of
RCC_AHB2ENR on every part with the generator except the F410, which has
no AHB2 at all and puts the enable at the TOP of RCC_AHB1ENR, bit 31.
Neither the register nor the bit is the same, so the driver selects the
store with the header's own symbol and the reserve states the fact.

**The vector wears two names.** The large-line parts (the F405 class,
the F42x/F43x, the F469/F479) spell it HASH_RNG_IRQn - the slot is
shared with the hash processor whether or not the part has one, and on
the F405 class the header adds that name as a macro alias for its own
RNG_IRQn; the small parts that have a generator (the F410, the F412, the
F413/F423) spell it RNG_IRQn and declare no other name. The two sets are
exactly the parts with and without a backup SRAM, which is what
`rng_irq()` keys on - a reading of the twenty-three headers that `brio
check stm32f4` keeps true, since a header on the wrong side of it does
not compile.

**The first value is not a random number.** FIPS PUB 140-2 asks that
"the first random number generated after setting the RNGEN bit should
not be used, but saved for comparison with the next", and that each
subsequent number be compared with the one before, the test failing on a
repeat (24.3.1). `init()` performs the discard; `read()` performs the
comparison and refuses a repeated word rather than handing it out.

**The two errors are not the same kind.** A CLOCK error (CECS) means the
generator has stopped producing, but "has no impact on the previously
generated random numbers, and the RNG_DR register contents can be used":
fix the clock tree, clear CEIS, carry on. A SEED error (SECS) means the
analog part produced a faulty sequence - more than 64 consecutive bits
at one value, or more than 32 alternations - and then "if a number is
available in the RNG_DR register, it must not be used because it may not
have enough entropy": clear SEIS, then clear AND SET RNGEN to
reinitialize (24.3.2). `recover()` is that sequence.

**The two latched flags are rc_w0, not rc_w1.** SEIS and CEIS are
"cleared by writing it to 0" (24.4.2) - the opposite convention from
every other flag this stratum clears, and a read-modify-write with a one
would leave them standing. The clearing verbs store the register with
that one bit zeroed.

**One interrupt for three events.** RNG_CR.IE raises the vector on DRDY,
SEIS or CEIS alike and there is no per-source enable, so the ISR body
reports all three and the handler decides. Forty RNG_CLK periods pass
between two consecutive words (24.2) - 833 ns at 48 MHz.

## Types and verbs

| Name | Meaning |
|------|---------|
| `rng_clock_ratio_ok` | does this pair of rates clear 24.4.2's f(HCLK)/16 bar? |
| `rng_nominal_clock_hz` | 48 MHz - the design point, stated and not enforced |
| `rng_clocks_per_word` | 40, the periods between two words (24.2) |
| `RngError` | why a read gave nothing: not ready, seed error, clock error, repeated |
| `Rng::RngEvent` | what one interrupt found: ready, seed error, clock error |

The resource `Rng`, a monostate that exists only where the part has the
block:

| Purpose | Verbs |
|---------|-------|
| the clock gate | `clock(bool)`, `clock()`, `reset_block()`, `clock_on_ahb1` |
| the vector | `irq()` |
| the generator | `enable(bool)`, `enabled()` |
| the interrupt | `interrupt(bool)`, `interrupt()` |
| the statuses | `ready()`, `seed_error()`, `clock_error()` |
| the latched flags | `seed_error_flag()`, `clock_error_flag()`, `clear_seed_error()`, `clear_clock_error()` |
| taking a word | `read()`, `read_blocking(spins)`, `value()` (raw), `last_error()` |
| bring-up | `init(clock)`, `discard_first(spins)` |
| after a seed error | `recover()` |
| the ISR body | `isr()` |
| teardown | `release()` |

`init(clock)` is where the clock rule is checked, at COMPILE time for a
static clock: a rate whose PLL Q output is absent, or under
f(HCLK)/16, does not compile and the message says which.

## How to use it

Bring the generator up and take words:

```cpp
if (!brio::Rng::init(clock)) {                 // gate, enable, discard the first
    // the domain is not running
}
if (const auto v = brio::Rng::read()) {
    use(*v);
} else if (brio::Rng::last_error() == brio::RngError::seed_error) {
    (void)brio::Rng::recover();
}
```

A bounded wait instead of a poll of your own:

```cpp
const auto v = brio::Rng::read_blocking();     // nothing, on an error or a timeout
```

Under the interrupt, with the handler binding the vector the reserve
names:

```cpp
brio::Rng::interrupt(true);
brio::Nvic::enable(brio::Rng::irq());
// in the app's handler:
const auto e = brio::Rng::isr();               // flags reported and cleared
if (e.seed_error) { (void)brio::Rng::recover(); }
else if (e.ready)  { queue(brio::Rng::value()); }
```

## Bench findings

`test_stm32f4_misc`, letter `b`, on an STM32F429 at 180 MHz with the
PLL's Q output at 45 MHz (the 48 MHz domain is not 48 here: HCLK/16 is
11.25 MHz, and 24.4.2's ratio is what the chapter asks, so no CECS); the
same letter green on the STM32F469 at the same rates, whose generator
sits on the same domain (RM0386 19.3.6) and whose DCKCFGR.CK48MSEL is
left at its reset choice, the PLL's Q output.

**Ten thousand words in 10 ms** - 1000 words a millisecond, against the
1125 the chapter's 40 RNG_CLK periods a word would allow at this rate;
the ones 499 per mille of 320000 bits, the sixteen low-nibble buckets
577..676 for 625 expected, no two consecutive words equal; the last word
different every run. **DRDY goes down with the read and is back within
the next read** (0 us); the two latched flags clear on a write of zero;
the recovery sequence run on a healthy generator hands out words again.
**A word already computed survives the disable**: with RNGEN cleared the
first read still answers (DRDY was up), and no new word comes after it -
the disable is judged on what follows that word.

What is established without a board: the driver and its letter compile
for every one of the twenty-three device headers (`brio check stm32f4`),
the type does not exist on the three parts without the block
(`neg/rng_absent_block.cpp`), a clock with no PLL is refused where the
program names it (`neg/rng_clock_without_a_pll.cpp`), and the clock
ratio, the gate's bus and bit, and the vector's two names are asserted
against the headers in `test/family_stm32f4/rng.cpp`.

## Not covered yet

Driver gaps:
- Nothing of the chapter. Three registers, seven fields, and every one
  of them has a verb.

Implemented, not bench-verified:
- The interrupt path: one vector for DRDY, SEIS and CEIS, its body
  compiled and never entered - the letter polls; what would measure it
  is a letter arming IE over the generator and counting entries.
- The seed error itself and its recovery under a REAL fault: SECS is
  raised by the analog part and there is no way to provoke it from
  software, so even on a board with the generator the recovery path is
  exercised only in its harmless form (clear, disable, enable, discard).
- The clock error: reachable only by running the generator with the PLL
  Q output stopped or below f(HCLK)/16, which on this family means
  taking the 48 MHz domain down under a running RNG - a `DynamicClock`
  rate away, and worth one letter on a board that has the block.
