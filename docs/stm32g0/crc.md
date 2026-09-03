# CRC calculation unit (STM32G0)

> **PROVISIONAL.** Chapter 14 is implemented whole - every register,
> every field, all three access widths, both reversals, the programmable
> polynomial at all four sizes and the scratch register - and every one
> of them is bench-verified. What keeps the banner is in "Not covered
> yet": the unit is complete, its RELATIONSHIP to `util/crc.hpp` is a
> util design question this stratum must not open on its own.

Documents of record: RM0444 Rev 6 - CRC ch. 14, the AHB enable and reset
5.4.8/5.4.4, the DMA request table 55 (which has NO CRC row). Errata
ES0548 Rev 3: NO ITEM TOUCHES THE CRC - a statement about the document,
not a claim about the silicon. Driver: `stm32g0/crc.hpp`; the presence
and mask facts come from `stm32g0/device_tables.hpp`. Bench suite:
`test_stm32_crc` (5 letters, 27 verdicts, wireless). Family fixture
`test/family_stm32g0/crc.cpp` plus two negatives under
`tools/check_stm32g0.sh`.

## What the silicon does

A hardware polynomial divider on the AHB with five registers, no
interrupt, no request line and no errata - the smallest complete chapter
in this stratum. Every part of the family has exactly one, so `Crc` is a
MONOSTATE and not a template.

**One data register does everything.** A write to `CRC_DR` combines the
new datum with what is already there; a read of the same address returns
the running result (14.3.3). It is also the only register of the block
with narrow accesses: 14.4 allows `CRC_DR` to be reached "by words,
right-aligned half-words and right-aligned bytes" and every other
register by 32-bit accesses only. The three widths cost 4, 2 and 1 HCLK
cycles, and an input buffer lets a second write start without stalling
(14.2).

**The polynomial is programmable and the rules around it are software's.**
`CRC_POL` takes the coefficients and `CR.POLYSIZE` the width (32, 16, 8
or 7 bits). 14.3.3 says "even polynomials are not supported" and nothing
in the silicon notices one; 14.4.5 says a narrow polynomial must sit in
the least significant bits and nothing notices when it does not. And the
polynomial "cannot be performed" mid-calculation: the application must
reset the unit or read `CRC_DR` before changing it.

**The reversals are where endianness lives.** `REV_IN` reverses the bit
order of the input by byte, half-word or whole word before the
computation sees it; `REV_OUT` reverses the result at bit level. A
32-bit write is processed most significant BYTE first, so a byte STREAM
fed as words is only the same computation as the same stream fed byte by
byte when the assembly matches the reversal.

**`CRC_INIT` is the seed and `CR.RESET` is what loads it.** Writing
`CRC_INIT` also initializes `CRC_DR` on its own (14.3.3). `CRC_IDR` is
four bytes of general storage the unit never looks at, and 14.4.2 makes
it the one register a `CR.RESET` does not touch.

**There is no final XOR.** A standard reflected CRC-32 ends with
`^ 0xFFFFFFFF` and the hardware does not do it - which is why
`crc32_ieee_finish()` exists as one constexpr line beside the preset.

## Types and verbs

`Crc`, the monostate resource:

- `init()` / `release()` / `reset()` - the AHB clock gate and the
  `RCC_AHBRSTR` pulse; 5.2.17 means a peripheral whose enable bit is
  clear does not answer register reads at all, so `init()` opens the
  gate first;
- `configure(const CrcConfig&)` - polynomial, size, init value and both
  reversals in one verb, refused for a configuration
  `crc_config_valid()` rejects, and ENDING IN A RESET so a
  configuration is a fresh start rather than a modifier of whatever was
  in DR (14.3.3's rule read forwards);
- `restart()` - `CR.RESET` alone, the verb between two independent
  checksums under one configuration;
- `feed(uint32_t)`, `feed16(uint16_t)`, `feed8(uint8_t)` - the three
  access widths through the one address, spelled through differently
  typed volatile pointers so the compiler really emits STR, STRH and
  STRB;
- `feed(std::span<const uint8_t>)` - a byte STREAM in stream order,
  words while four remain and the tail as bytes (14.3.3's own advice),
  assembled BIG-ENDIAN because a word write is processed most
  significant byte first, and falling back to byte-by-byte under the two
  reversals that cross a byte boundary so the verb means one thing
  whatever is configured;
- `feed(std::span<const uint32_t>)` - words as words, the caller owning
  what their byte order means;
- `value()`, `value_masked()`, `value_masked_now()` - the whole DR and
  14.3.3's "least significant bits" for a narrow polynomial;
- `width()`, `reverse_in()`, `reverse_out()`, `polynomial()`,
  `initial()` - readbacks off the silicon;
- `scratch(v)` / `scratch()` - `CRC_IDR`;
- `data_address()` - what a DMA channel writes to feed the unit.

`crc_config_valid()` refuses an EVEN polynomial and one that does not
fit the width it declares - the two rules 14.3.3 and 14.4.5 state and no
register enforces.

Two named presets: `crc_ccitt_false_config` (0x1021, 16 bits, init
0xFFFF, no reversal) which IS `util/crc.hpp`'s CRC-16/CCITT-FALSE, and
`crc32_ieee_config` (0x04C11DB7, 32 bits, init 0xFFFFFFFF, input
reflected per byte, output reflected) plus `crc32_ieee_finish()` for the
final XOR the hardware has no register for.

## How to use it

The CCITT-16 of a buffer, the checksum `util/crc.hpp` computes in
software:

```cpp
#include "stm32g0/crc.hpp"

brio::Crc::init();
brio::Crc::configure(brio::crc_ccitt_false_config);
brio::Crc::feed(std::span<const uint8_t>{payload, len});
const uint16_t sum = static_cast<uint16_t>(brio::Crc::value_masked());
```

A reflected CRC-32, the one zip and the SAM's DSU speak:

```cpp
brio::Crc::configure(brio::crc32_ieee_config);
brio::Crc::feed(std::span<const uint8_t>{image, bytes});
const uint32_t sig = brio::crc32_ieee_finish(brio::Crc::value());
```

A whole flash region checksummed with NO CPU in the loop - the CRC has
no request line (table 55 has no CRC row), so the only mode that can
feed it is MEMORY-TO-MEMORY, with the source incrementing and this
destination FIXED:

```cpp
brio::Crc::configure(brio::crc32_ieee_config);
using Ch = brio::DmaChannel<1, 1>;
Ch::prepare(brio::DmaTransfer{
    .peripheral = const_cast<uint32_t*>(region),   // the SOURCE, incrementing
    .memory = brio::Crc::data_address(),           // the FIXED destination
    .count = words,
    .config = {.direction = brio::DmaDirection::peripheral_to_memory,
               .memory_to_memory = true,
               .peripheral_increment = true,
               .memory_increment = false,
               .peripheral_width = brio::DmaWidth::word,
               .memory_width = brio::DmaWidth::word}});
Ch::enable(true);
while (!Ch::flag(brio::DmaFlag::complete)) {
}
const uint32_t sig = brio::crc32_ieee_finish(brio::Crc::value());
```

## Bench findings

**The reset values are table 70's own**, once the AHB gate is open: DR
and INIT all ones, POL the CRC-32 Ethernet polynomial, CR and IDR zero.
Read through the CLOSED gate, DR and POL both answer 0 - 5.2.17 again,
and here its answer is merely absent rather than misleading.

**All three of 14.3.3's REV_IN examples are exactly what the silicon
does.** 0x1A2B3C4D reversed by byte, half-word and word gives
0x58D43CB2, 0xD458B23C and 0xB23CD458, and feeding the PRE-REVERSED word
with REV_IN = none produces the same checksum as feeding the original
with the reversal on - 0xD35E9975, 0xADC6814B and 0x271D6588
respectively. The chapter's three numbers are right and they are
verified against a checksum the unit really computed, not against
themselves.

**REV_OUT reverses the output and nothing else** (0x0376E6E7 becomes
0xE7676EC0 over the check string), and **CRC_IDR survives both a
`CR.RESET` and a whole reconfiguration** - 14.4.2's sentence, measured.

**14.3.3's "least significant bits" is literal AND THE REST IS NOT
ZERO**: a 7-bit polynomial leaves DR = 0x75 of which 0x75 is the result,
so `value_masked()` is a real verb and not a formality.

**A polynomial changed mid-calculation gives the checksum of nothing** -
neither polynomial's answer - which is 14.3.3's "cannot be performed"
measured rather than assumed. BUT UNRELIABLE HERE DOES NOT MEAN RANDOM:
the same illegal sequence run twice gives the same number both times
(0xF3D and 0xF3D), so what is lost is the meaning and not the
determinism. `configure()`'s own trailing `CR.RESET` is what makes the
next calculation clean again, and that is measured too.

**The four catalogue check values over "123456789" are the hardware's**:
CRC-16/CCITT-FALSE 0x29B1, CRC-32/ISO-HDLC 0xCBF43926, CRC-8/SMBUS 0xF4
and CRC-7/MMC 0x75 - one per POLYSIZE code, so all four widths are
exercised.

**`crc_ccitt_false_config` IS `util/crc.hpp`'s CRC-16, bit for bit over
4096 pseudo-random bytes** (both 0x31A5), by two mechanisms that share
only their definition. That is the proof the preset is what it claims.

**`feed(span)` means exactly what byte-by-byte feeding means, in all
four REV_IN settings**, over a length the word path cannot cover on its
own. And for a BYTE access the three reversals are ONE thing - there is
nothing above a byte to reorder - so REV_IN only tells itself apart on
the wider accesses (0xA965C7D4 with none, 0x331B7174 with all three
others).

**What it costs, per 1000 bytes at 64 MHz**: 2056 cycles through the
word path, 4549 through the half-word path, 8055 through the byte path,
against `util/crc.hpp`'s bitwise loop at 109216. **The hardware's word
path is 53x the software loop** - and the input buffer of 14.2 is real,
a back-to-back word write costing about 8 cycles where a stalled AHB
transaction plus 14.3.3's four computation cycles would cost more than
ten.

**16 KB of this program's own flash, three routes, one number.** The
same region checksummed CPU-fed (54458 cycles), DMA-fed (28736 cycles)
and by a bitwise software reference agree exactly. The DMA path moves
36.5 MB/s with no CPU in the loop and is nearly twice as fast as the CPU
one, having no instruction fetch between beats.

**Both refusals fire before a register is touched**: an even polynomial
and one too wide for its POLYSIZE are rejected with the polynomial in
force left exactly as it was.

## util/crc.hpp is not replaced and not hooked

`util/crc.hpp`'s bitwise CRC-16/CCITT-FALSE is portable, constexpr and
the nonvolatile stores' judge on three architectures. This block
computes the same checksum 53 times faster, which the suite proves bit
for bit - but nothing in `util/` is pointed at it. A hardware hook for a
util service is a design question about the util level (a concept, a
fallback, what a constexpr context does with a peripheral), and it is
not this stratum's to open.

## Not covered yet

Driver gaps - things chapter 14 has and this file does not:

- nothing.

Implemented but not bench-verified:

- **`release()`** - the gate is closed and the block reset, but no
  letter checks what a closed gate does to a running calculation
  (5.2.17's answer is measured for a READ, at the top of letter a).

Declined, with the reason:

- **The util hook**, above: a design question for `docs/design/`, not a
  driver gap.
- **A DMA channel narrower than a word.** `data_address()` is width
  agnostic and the suite feeds words; a byte-wide MEM2MEM channel into
  `CRC_DR` would exercise the narrow access from the DMA's side rather
  than the CPU's, and nothing here needs it yet.
