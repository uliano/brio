# CRC calculation unit (STM32F4)

Documents of record: RM0390 Rev 6 ch. 4 (RM0090 Rev 22 ch. 4 and RM0383
Rev 4 ch. 3 are the same three registers under the same names); no item
of ES0206 Rev 24, ES0298 Rev 8 or ES0287 Rev 6 is filed against this
block. Driver: `stm32f4/crc.hpp` (`Crc`, and `word_be`, the packing
that turns a byte stream into the words this register takes). The same
polynomial in software - `crc32_ethernet_byte`, `crc32_ethernet_word`,
`crc32_ethernet`, `crc32_ethernet_bytes` - is `util/crc.hpp`'s, beside
the CRC-16 the stored records carry, because more than one target has a
block computing exactly this function. The family fixture is
`test/family_stm32f4/crc.cpp`, which pins the software model to the
published check value of CRC-32/MPEG-2. Bench: `test_stm32f4_misc`,
letter `a`.

## What the silicon does

**One function, wired in.** The generator polynomial is 0x04C11DB7, the
CRC-32 of Ethernet, and there is no register to change it, no initial
value to choose, no bit or byte reversal and no final XOR - the four
knobs the same peripheral grew on the STM32L4 and F7 do not exist here.
What this unit computes is therefore exactly the function catalogued as
**CRC-32/MPEG-2**: initial value 0xFFFFFFFF, most significant bit first,
no reflection, no final inversion. `crc32_ethernet()` (`util/crc.hpp`)
is that function in constexpr C++ and the bench letter is the two
agreeing.

**The data register IS the operation.** A write to CRC_DR feeds a 32-bit
word into the calculator; a read returns the running result. There is no
start bit and no ready flag, because "the write operation is stalled
until the end of the CRC computation" (4.3) - back-to-back writes, and a
read straight after a write, are correct by construction. Reading does
not consume the value: a checksum can be taken in pieces, read in the
middle, and continued.

**A word is the grain, and the byte order is the program's.** 4.3 is
explicit that the computation "is done on the whole 32-bit data word, and
not byte per byte", and 4.4 that the registers "have to be accessed by
words (32 bits)" - this version of the block has no byte or half-word
access to CRC_DR. So a BYTE STREAM has no direct answer here: the caller
packs four bytes into a word, and packing them most-significant-first
(`word_be`) is what makes the unit's result equal CRC-32/MPEG-2 over
those bytes in that order. A stream whose length is not a multiple of
four is the caller's to pad, and the padding is part of what the
checksum means.

**THE RESET IS NOT INSTANT, and the chapter does not say so.** CRC_CR
has one bit, RESET, write-only and self-clearing, which puts CRC_DR back
to 0xFFFFFFFF (4.4.3). Measured (letter `a`): a read of CRC_DR in the
instruction after the store still returns the OLD running value, and a
word FED in the instruction after the store is SWALLOWED - the write
goes nowhere and no flag says so. 4.3's stall covers a computation and
not this. `Crc::reset()` therefore waits for the initial value to appear
before it returns, and returns how many extra reads that took.

**CRC_IDR is not part of the checksum.** Eight bits of scratch that the
RESET bit deliberately does not touch (4.3, 4.4.2) and that the
peripheral's RCC reset line does clear - the difference between
`reset()` and `reset_block()`. ST's device header declares the register
`__IO uint8_t` at offset 0x04 where 4.4 asks for word accesses; the
header wins in code, and the bench measures that the byte store sticks.

**No interrupt, no DMA request, no event.** The chapter has no enable
register, no part of the family has a CRC vector and no DMA request line
is mapped to this block. A bulk checksum is a loop of stores. What a DMA
stream CAN do is memory-to-memory into CRC_DR with the destination
address not incremented - that is the DMA chapter's arrangement and
needs nothing from this file.

**Present on every part.** All twenty-three device headers declare
CRC_BASE and RCC_AHB1ENR.CRCEN, so this is the one block of the three in
`test_stm32f4_misc` with no absence to handle and nothing in the
reserve.

## Types and verbs

The polynomial, in software (constexpr, usable at compile time). It is
`util/crc.hpp`'s and not this chapter's, because the function belongs to
every block that has it wired in; `word_be` is this driver's own:

| Name | Meaning |
|------|---------|
| `crc32_ethernet_poly` | 0x04C11DB7, the generator 4.2 names |
| `crc32_ethernet_init` | 0xFFFFFFFF, CRC_DR's reset value and every computation's start |
| `crc32_ethernet_byte` | one byte into a running CRC, MSB first, no reflection |
| `crc32_ethernet_word` | one 32-bit word, in the order the silicon takes it |
| `crc32_ethernet` | a run of words from the initial value |
| `crc32_ethernet_bytes` | a run of BYTES, for the lengths this unit cannot take |
| `word_be` | four bytes as the word that makes the unit's answer the stream's |

The resource `Crc`, a monostate:

| Purpose | Verbs |
|---------|-------|
| the clock gate | `clock(bool)`, `clock()` |
| bring-up | `init()` (the gate, then a reset) |
| starting a checksum | `reset()` - returns the extra CRC_DR reads it waited |
| the whole block back | `reset_block()` (the RCC line; clears CRC_IDR too) |
| feeding | `feed(word)`, `feed(words, count)`, `feed_bytes(b0, b1, b2, b3)` |
| reading | `value()` |
| all three at once | `compute(words, count)` |
| the scratch register | `idr(uint8_t)`, `idr()` |
| the register block | `regs()` |

`reset_spins` is the bound `reset()` waits under - a bound and not an
expectation.

**No ownership and no guard.** There is one unit, and it holds the
running result in the register a second user would write, so two pieces
of code checksumming from different contexts corrupt each other with no
flag to show for it. This driver does not arbitrate that - there is
nothing in the silicon to arbitrate with - so a program that checksums
from an interrupt as well as from the loop owns the arrangement.

## How to use it

A checksum over a run of words:

```cpp
brio::Crc::init();                                  // the gate, then a reset
const uint32_t sum = brio::Crc::compute(words, count);
```

A checksum built in pieces, with other work in between:

```cpp
(void)brio::Crc::reset();
brio::Crc::feed(header, 4);
// ... other work, and even a read of the running value ...
brio::Crc::feed(body, body_words);
const uint32_t sum = brio::Crc::value();
```

A byte stream whose length is a multiple of four:

```cpp
for (size_t i = 0; i < n; i += 4) {
    brio::Crc::feed_bytes(b[i], b[i + 1], b[i + 2], b[i + 3]);
}
```

The same checksum with no hardware - for a constant known at compile
time, for a length the unit cannot take, or to judge the silicon:

```cpp
constexpr uint32_t expected = brio::crc32_ethernet(words, count);
static_assert(brio::crc32_ethernet_bytes(message, 9) == 0x0376E6E7u);
```

The scratch register, which survives a checksum:

```cpp
brio::Crc::idr(state);                              // eight bits, kept across reset()
const uint8_t back = brio::Crc::idr();
```

## Bench findings

From `test_stm32f4_misc` letter `a` on an STM32F446 at 180 MHz (the same
letter green on the STM32F429 and the STM32F469):

- **The silicon is CRC-32/MPEG-2, to the bit.** One word 0x12345678 from
  the initial value gives 0xDF8A8A2B; sixteen assorted words give
  0x7674B41F; both equal the software model, which is itself pinned to
  the published check value 0x0376E6E7 over the ASCII bytes
  "123456789" (`test/family_stm32f4/crc.cpp`).
- **A checksum taken in two halves, with a read of the running value
  between them, equals the checksum of the whole** - reading CRC_DR does
  not disturb the state.
- **An empty message is 0xFFFFFFFF**: reset, read, nothing fed.
- **THE RESET LANDS ONE READ LATE.** With a value in CRC_DR, the store
  of CRC_CR.RESET is not visible to the read that follows it; the SECOND
  read shows 0xFFFFFFFF (measured: 1 extra read at 180 MHz). A word
  written in the instruction after the store is swallowed entirely - the
  first version of this letter measured exactly that, twice: a one-word
  checksum came out as the initial value, and an "empty message" read
  came out as the previous result.
- **CRC_IDR takes a byte store** (0x5A written and read back through the
  header's `uint8_t` field), **survives CRC_CR.RESET** and is cleared
  with CRC_DR by the peripheral's RCC reset line.
- **A word costs 9.0 core cycles** at 180 MHz in a load-and-store loop
  (1024 words in 9250 cycles), of which four are the computation the
  chapter promises and the rest the loop's own.

## Not covered yet

Driver gaps:
- A byte-stream entry point that packs and pads: the packing rule is
  part of what a checksum MEANS (little-endian words, big-endian words,
  a padded tail), so it belongs to the format above this driver and not
  to the block; `word_be` and `util/crc.hpp`'s `crc32_ethernet_bytes`
  are what a format is built out of.
- Feeding CRC_DR from a DMA stream (memory-to-memory with the
  destination address fixed): `stm32f4/dma.hpp` has every verb it needs
  and no program here checksums enough to want it - born with its first
  user.

Implemented, not bench-verified: nothing. Every verb of this file runs
in letter `a`, and the block has no options to leave untried.
