# CRC (CH32V203 and CH32V303)

A hardware CRC-32 with one polynomial wired in, three registers and no
options at all. Documents of record: the CH32F/V20x_V30x_V31x reference
manual V2.3, chapter 5 whole - which applies to the entire family and
carries no per-class note, and which neither datasheet's resource table
makes a per-part fact either. Driver:
[brio/ch32vx03/crc.hpp](../../brio/ch32vx03/crc.hpp), over the software
twin in [brio/util/crc.hpp](../../brio/util/crc.hpp). Reference suite:
`test_vx03_misc`.

## What the silicon does

### One function, and it is not this family's

Per the manual (5.1), the polynomial is 0x04C11DB7 - the CRC-32 of
Ethernet - and there is nothing beside it: no polynomial register, no
initial value register, no reversal and no final XOR. What the block
computes is therefore exactly one function, the one catalogued as
CRC-32/MPEG-2: initial value 0xFFFFFFFF, most significant bit first, no
reflection of input or output, no final inversion.

That function is not this family's property, which is why it does not
live in this stratum: `crc32_ethernet()` in
[util/crc.hpp](../../brio/util/crc.hpp) is the same arithmetic in
constexpr C++, shared with every other target whose silicon carries
this one polynomial and no knob to change it. The two agreeing is what
the suite measures.

### The data register is the operation

Per the manual (5.2), one register, CRC_DATAR: a WRITE feeds a 32-bit
word into the calculator, a READ returns the running result. There is
no start bit and no ready flag - "the hardware calculation interrupts
the write operation of the system, so new values can be written
continuously" - and 5.1 prices the computation at four HCLK cycles. So
back-to-back writes, and a read right after a write, are correct by
construction and the cost is paid by the bus rather than by a poll.
Reading does not consume the result either, which is what lets a
checksum be taken in pieces.

### A word is the grain, and the program decides its byte order

Per 5.2: the unit "calculates the whole 32-bit data word, rather than
byte per byte". A BYTE STREAM therefore has no direct answer here.
`word_be()` packs four bytes most-significant-first, which is the
packing that makes the unit's result equal CRC-32/MPEG-2 over those
bytes in that order; a stream whose length is not a multiple of four is
the caller's to pad, and the padding is part of the checksum's
definition. `Crc::compute_bytes()` refuses any other length at compile
time.

### The reset bit is the only control, and it lands at once

Per 5.3.3, CRC_CTLR has one bit, RST, write-only and self-clearing,
which puts CRC_DATAR back to 0xFFFFFFFF. There is nothing else to
configure, so a reset is what starts every checksum.

WHETHER THE RESET LANDS INSTANTLY IS WHERE THIS BLOCK AND ITS RELATIVE
DIFFER. The same design on an STM32F4 was measured taking a few cycles -
a read of the data register in the next instruction returning the OLD
running value, and a word written in the next instruction swallowed -
and neither manual says so. Measured on the CH32V203C8T6: it lands
before the next instruction can look. `Crc::reset()` waits for the
initial value to appear and RETURNS how many extra reads that took,
which is zero every time on that silicon, and a word stored in the very
next instruction is TAKEN and not swallowed - so a checksum may be
started and fed with nothing in between.

### The scratch register is not part of the checksum

Per 5.2 and 5.3.2, CRC_IDATAR is eight bits of general-purpose storage
"not affected by RST in CRC_CTLR" - the block's one piece of state a
program can use to remember something across a checksum. The chapter
names the register R8_CRC_IDATAR, which is what decides the width of
that store: measured, it takes a byte store, gives back all two hundred
and fifty-six values, and survives both a reset and every word fed
through the calculator.

AND NOTHING SHORT OF A SYSTEM RESET CLEARS IT. This block has no line
in RCC_AHBRSTR, whose bits [11:0] are reserved (3.4.11), so there is no
peripheral reset to offer and the driver offers none: what RST spares,
only a reboot takes away. Measured on both halves - the pattern
surviving a reset of the calculator, and reading zero at the next boot
after a software reset.

### The clock gate is the block's whole presence

Per 3.4.6, the block hangs off RCC_AHBPCENR's CRCEN, which is clear at
reset. What that gate holds is not a detail a program can work around,
and it is worth stating precisely because a register read is the usual
way of asking whether a peripheral is there: MEASURED, with the gate
shut a read of CRC_IDATAR - which held 0x3C - came back as the LOW BYTE
of the last word the data register had answered, and a read of the data
register came back as that same word again. What a read returns from a
block behind a closed gate is the last word the BUS carried, byte lane
and all, never the register that was asked for; and a store into either
register is dropped in silence. Re-opening the gate finds the running
value and the scratch exactly where they were left, so the state is the
block's and the gate is what stands between the bus and it.

### No interrupt, no DMA request, no event

Per the manual, the chapter has no interrupt register; the vector table
has no CRC line; and no table of 11.2.3 maps a DMA request to this
block, on either series. A bulk checksum is a loop of stores and nothing
else. What a DMA channel CAN do is memory-to-memory into CRC_DATAR with
the destination not incremented - that is the DMA chapter's arrangement
and [ch32vx03/dma.hpp](../../brio/ch32vx03/dma.hpp) already has every
verb it needs, so nothing is added here for it.

### No ownership

There is one unit and it holds the running result in the same register
a second user would write, so two pieces of code that checksum from
different contexts corrupt each other's answer with no flag to show for
it. There is nothing in the silicon to arbitrate with, so the driver
does not pretend to: a program that checksums from an interrupt as well
as from the loop owns the arrangement.

## Types and verbs

`Crc` is the block, a monostate. `clock(on)` is the AHB gate and
`init()` is that gate plus a reset - what a program calls once before
its first checksum. `reset()` returns the number of extra reads it
spent waiting for the initial value to appear. There is no verb for a
peripheral reset, because the block has no line in RCC_AHBRSTR.

`feed(word)`, `feed(words, count)` and `feed_bytes(b0, b1, b2, b3)`
push data in, `value()` reads the running result without consuming it,
and `compute(words, count)` is reset-feed-read in one call.
`compute_bytes(array)` is the same over a byte array whose length is
known at compile time and is a multiple of four; any other length is a
compile error, with `crc32_ethernet_bytes()` in util/crc.hpp as the
software answer.

`scratch(v)` and `scratch()` are CRC_IDATAR. `word_be(b0, b1, b2, b3)`
is the packing, spelled as the STM32F4 stratum spells the same
arithmetic over the same block.

## How to use it

A block of words, checksummed and judged against the software twin:

```cpp
(void)brio::Crc::init();
const uint32_t sum = brio::Crc::compute(words, count);
const bool agrees = sum == brio::crc32_ethernet(words, count);
```

A checksum taken in pieces, which is what a running total over a stream
looks like:

```cpp
(void)brio::Crc::reset();
for (const auto& chunk : chunks) {
    brio::Crc::feed(chunk.words, chunk.count);
}
const uint32_t sum = brio::Crc::value();
```

A byte stream, packed the way this register takes it:

```cpp
static constexpr uint8_t message[8] = {'b','r','i','o',' ','c','r','c'};
const uint32_t sum = brio::Crc::compute_bytes(message);
// the same number as brio::crc32_ethernet_bytes(message, 8)
```

Something remembered across a checksum:

```cpp
brio::Crc::scratch(0x5A);
const uint32_t sum = brio::Crc::compute(words, count);
const uint8_t tag = brio::Crc::scratch();     // the reset did not touch it
```

`regs()` is the register view, for a program that has to stage a
sequence these verbs do not offer - what the suite uses to ask the
silicon questions a driver has no reason to ask.

## Bench findings

`test_vx03_misc` measured the chapter on a CH32V203C8T6, at 96 MHz from
the internal oscillator, with nothing wired: fourteen verdicts in `z`
and two in the letter that reboots the board.

- **The silicon computes CRC-32/MPEG-2 and nothing else.** Eight
  lengths - one, two, three, four, eight, sixteen, sixty-three and
  sixty-four words - agree to the bit with `crc32_ethernet()` in
  util/crc.hpp, and so does the empty message, whose checksum is the
  initial value because a read starts nothing. One word of 0x00000001
  and one of 0x80000000 each move the answer and each matches the twin,
  which is what says all thirty-two bits reach the polynomial.
- **`word_be`'s packing is the right one.** Twelve bytes fed four at a
  time, most significant first, give 0x29191B51 - the same number
  `crc32_ethernet_bytes()` computes over those bytes in that order.
- **The reset lands before the next instruction.** Sixty-four resets
  over a dirty register, each preceded by a fed word: the worst spin
  count was ZERO. A word stored in the instruction right after RST is
  taken, not swallowed - the running value afterwards is that one
  word's checksum and not the initial value. This is where the block
  differs from its STM32F4 relative, which needed a few cycles for
  both.
- **The running result is not consumed by a read.** Sixteen words
  checksummed whole give 0x8E3F8DCF; five words, a read, then eleven
  more give the same number, and so does the twin stepped one word at a
  time.
- **CRC_IDATAR is eight bits of byte-wide scratch.** All two hundred
  and fifty-six values read back; the pattern survives `reset()` and
  every fed word; and at the next boot after a software reset it reads
  zero with the gate shut and the calculator at 0xFFFFFFFF - the state
  a program finds at every boot.
- **What the unit costs.** A thousand and twenty-four words through the
  block took 12578 core cycles at 96 MHz - twelve a word, 131 us for
  the run - against 81636 cycles (seventy-nine a word, 850 us) for the
  same words through the constexpr twin, which walks the polynomial bit
  by bit. Six and a half times faster, and the difference is what the
  bus stall costs rather than what the calculation does.
- **The clock gate.** With CRCEN clear the block does not answer at
  all: the scratch register read back as the data register's low byte
  and the data register as the word before it, both stores were lost,
  and re-opening the gate found the block exactly as it had been left.

## Not covered yet

Driver gaps, each with its reason:

- **A checksum fed by the DMA.** A memory-to-memory channel with the
  destination not incremented would feed this register at the
  controller's own pace, which is the only way to checksum a large
  block without a loop; the verbs for it are the DMA chapter's and
  nothing here is missing, but the arrangement has not been put
  together - it will be born with its first user.

Implemented but not bench-verified, each with what would measure it:

- **The twelve parts other than the CH32V203C8**, the CH32V303VC among
  them. The block is not a per-part fact and this file asks the part
  table nothing, and the whole stratum compiles for all thirteen both
  ways the hardware prologue can be built (`brio check ch32vx03`);
  `test_vx03_misc` builds for the CH32V303VC too. What would measure
  them is a board - for the CH32V303VC, that suite on WCH's evaluation
  board.
