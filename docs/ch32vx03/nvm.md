# Flash and the option bytes (CH32V203 and CH32V303)

The flash memory, the user option bytes and the electronic signature.
Documents of record: the CH32F/V20x_V30x_V31x reference manual V2.3
(chapter 32 for the array, the engine, the two programming methods, the
four erase grains and the option bytes; chapter 31 for the signature -
both chapters apply to the whole family, and the one per-part note in
either is table 32-4's, which gives the memory split of the user option
byte to named parts), the CH32V203 datasheet V2.8 (table 2-1 and its
notes 1 and 2 for each part's window and the tail above it, table 4-17
for the programming frequency and the operation times, table 4-18 for
the endurance) and the CH32V303/305/307/317 datasheet V3.5 (table 2-1-1
and its note 1 for the same, tables 4-19 and 4-20 for the rest, table
4-18 and its note 2 for the "code load time" of the zero-wait area).
Drivers:
[brio/ch32vx03/nvm.hpp](../../brio/ch32vx03/nvm.hpp) (the engine) and
[brio/ch32vx03/nvm_flash.hpp](../../brio/ch32vx03/nvm_flash.hpp) (the
storage medium over it). Reference suite: `test_vx03_nvm`.

## What the silicon does

### The array, and the four grains

Per the manual, the main memory is a run of 256-byte PAGES starting at
0x0800 0000, which the core also reads through the alias at zero it
boots from; above it the information block holds the 28 KB system boot
loader and the 128-byte option byte area at 0x1FFF F800. WRITE
PROTECTION is granted in 4 KB units - sixteen pages - and that unit is
also the standard erase grain.

Chapter 32 offers two ways to write and four to erase:

| verb | grain | how |
|---|---|---|
| standard program | 2 bytes | PG set, then a HALF-WORD store at an even address; the store IS the operation |
| fast program | 256 bytes | FTPG set, 64 words stored with WRBSY watched between them, then PGSTRT |
| fast page erase | 256 bytes | FTER, the address, STRT |
| standard erase | 4096 bytes | PER, the address, STRT |
| fast block erase | 32768 bytes | BER32, the address, STRT |
| chip erase | the user array | MER, STRT, no address |

Per the manual, a store of any width other than a half-word behind PG
"will generate a bus error" (32.5.3), and "fast programming related
functions can only be placed in the zero-wait area FLASH" (32.2.1) -
which, on an array that is a window AND a tail, is the section below.

### The window and the tail

Per both datasheets, a part's "flash bytes" count its ZERO-WAIT run
area and the die carries more code flash above it. The CH32V203's note
1 to table 2-1: "flash bytes represent zero-wait run area R0WAIT. For
the V203 series, non-zero-wait area is (224K-R0WAIT)" - every one of its
nine parts. The CH32V303's note 1 to table 2-1-1: "the product of 256K
FLASH+64K SRAM supports the non-zero waiting area of (480K-R0WAIT)
bytes" - the CH32V303RC and VC, and not the CB and RB, whose window is
the whole of what a program may write. So the part table states three
numbers and the driver reads them: `window_bytes` (32, 64, 128 or 256
KB), `tail_bytes` above it (192, 160, 96 or 224 KB, or none) and
`array_bytes`, the die's code flash (224 KB on the CH32V203, 480 on the
CH32V303). Every address is counted from the start of the array, so the
tail runs from `tail_base` - where the window ends - to `tail_end`.

The WINDOW is where every linker script of this project places the
image, and what the core executes from at full speed; the TAIL is
nobody's by default - no script places a byte there. The window is read
through the alias at zero the image runs at; the tail through the
array's own address, 0x0800 0000 above the offset, because the memory
map gives that address the whole code flash - "includes 0 wait and non-0
waiting areas" - and says of the alias only that it follows the BOOT
pins. The CH32V303 datasheet says what the window is in a note: its
table 4-18 counts, in the 8.9 ms a wake from Standby takes with 256 KB
as its example, a "code load time" that "is calculated based on the
current zero-wait area capacity". So the zero-wait area is a LOAD,
which the measurements below fit.

WHAT THE TAIL COSTS is the non-zero wait. Measured on the CH32V303VC, a
kilobyte of words read with interrupts masked: **5 core cycles a word
out of the window, at the alias and at the array's own address alike,
against 20 out of the tail at 96 MHz** - and at 48 MHz, 20 with SCKMOD
at half and 12 with it whole, while the window's 5 did not move with
either setting. SCKMOD is a knob on the tail's price and not the
window's. The same page of the tail read at 144 MHz cost 20 core cycles
a word too.

THE TAIL TAKES BOTH METHODS. 32.2.1's note - "fast programming related
functions can only be placed in the zero-wait area FLASH", which V2.5 of
the manual words as "FLASH erase related functions" - could be read as
a statement about the TARGET those operations write. Measured on the
CH32V303VC, it is not: a fast page program into the tail's last page
landed there byte for byte, a fast page erase erased it, and a 32 KB
erase took the tail's last block whole, each ending with BSY down, EOP
up and no error. So the note is about where the CODE running them sits,
which an image linked into the window satisfies by construction, and
every write verb takes a tail address as it takes a window one.

A TAIL READ HAS THE RATE CONTRACT TOO. 32.1's note 2 lists "the
non-zero-wait area FLASH" among the flash operations that want HCLK
halved above 120 MHz, and the tail is read out of the array at the
access clock SCKMOD sets - 72 MHz at 144, over the chapter's 60. The
window is not: every image here executes out of it at 144 MHz with
SCKMOD at its reset value, which is also WCH's own configuration. So the
tail's read, `read_tail()`, takes the clock as the write verbs do and
refuses the same rates the same two ways. The refusal is the RATING's:
on the CH32V303VC a page of the tail read by hand at 144 MHz came back
exact three times over, which is what one die did outside the rating
and not a promise about any other.

### The split, and the signature's number

Per the manual (32.4.6, table 32-4), USER[7:5] moves the line between
the window and the SRAM - on the parts its two tables name, and on no
other:

| USER[7:5] | the CH32V303RC and VC (note 1) | the CH32V203RB (CH32V20x_D8) |
|---|---|---|
| 00x | 192 KB + 128 KB | 128 KB + 64 KB |
| 01x | 224 KB + 96 KB | 144 KB + 48 KB |
| 10x | 256 KB + 64 KB | 160 KB + 32 KB |
| 110 | 128 KB + 192 KB, on lots whose sixth digit from the end is not zero | 160 KB + 32 KB |
| 111 | 288 KB + 32 KB | 160 KB + 32 KB |

The part table states the factory's combination - 256 + 64 on the
CH32V303RC and VC, 128 + 64 on the CH32V203RB - and so does each linker
script; `device::flash_split_table` says which table a part reads, and
`none` on the other ten. THE DRIVER DECODES THE FIELD AND WRITES NONE,
like every option byte, and `FlashOptions::split_matches_part()` is the
check a program can make at boot: a split that is not the part table's
comes with a linker script that is not the part's either.

WHERE FLASH_OBR CARRIES IT: 32.4.6's bit column prints the field at
[9:8] beside codes three bits wide. The register itself says otherwise -
it carries the loaded USER byte WHOLE at [9:2], so the split is [9:7]:
measured, FLASH_OBR 0x000000FC against a USER byte of 0x3F on a
CH32V203C8, and 0x0000027C against 0x9F on a CH32V303VC, whose code
100 decodes - loaded and stored alike - to the 256 KB of window and 64
KB of SRAM its part table and linker script state.

ESIG_FLACAP IS NOT THE WINDOW IN FORCE. On that CH32V303VC, whose
window is 256 KB, the register answers 288: the LARGEST window table
32-4 offers the part. On a part with no split it is the window itself
(64 on the CH32V203C8). Both measured; `flash_window_max_bytes` is that
number, which is what the signature is compared with.

### The erased pattern is not 0xFF

Per the manual, said four times (32.5.4, 32.5.7 twice, 32.6.3): "after
erasing is successful, word read - 0xe339e339, half word read - 0xe339,
even address byte read - 0x39, odd address read 0xe3". An erased cell
of this array reads back **0xE339E339**, where every other flash brio
writes leaves all ones - and the sister family's chapter promises 0xFF
for the same verbs, so this is this silicon's and not a habit of the
vendor's. `Flash::erased_word` is that pattern and `Flash::erased()`
the run test over it; what it costs a structure built on top is under
the medium, below. Measured on an erased page of both parts, and again
where nobody looked for it: the THIRD word of each die's own unique
identifier reads 0xE339E339 too.

AND THE CELLS DO NOT BEHAVE LIKE BITS THAT ONLY FALL, which is the
other half of the same fact and the part a structure above the medium
would get wrong. Measured on both parts: a HALF-WORD CELL OF THE WINDOW
TAKES PASS AFTER PASS between erases - five in a row, 0xF0F0 then
0xF000, 0x0F0F, 0xFFFF, 0x0000 and 0xA5A5, each reading back EXACTLY
what was written, so bits come back as well as go away - and on the
CH32V303VC a cell given 0x9659 and then 0x0F0F still read 0x0F0F after
a system reset. A half-word cell of the TAIL does not: given the same
two values, it read 0x0E0F, neither of them, on the CH32V303VC. And a
PAGE PROGRAMMED TWICE with no erase between reports no error and ends
up holding neither pattern and not their bitwise AND either - the same
0x15E60CED in its first word on both parts, from the same two
patterns. So what the window's cells do is the window's and not the
array's, which fits a window the datasheet says is loaded; what a cell
programmed twice reads after the window is loaded again - at a power-up
or a wake from Standby - is not measured. A program that wants a cell
to hold what it wrote erases first.

### The rate is part of the contract

Per the manual and the datasheet together, this is the one fact that
shapes the driver's shape. The datasheet's table 4-17 gives a
programming frequency of 60 MHz maximum and its note says the figure
covers "read operation, program operation and erase operation" and that
"the clock is from HCLK". FLASH_CTLR.SCKMOD chooses whether the flash
access clock is HCLK or HALF of it, and its reset value is half. So:

- at or below 120 MHz of HCLK the array is inside its rating with the
  reset value of SCKMOD, and nothing has to be done;
- SCKMOD may be set - the array read at the whole system clock - only
  at or below 60 MHz;
- above 120 MHz, 32.1 asks for HCLK to be DIVIDED BY TWO around every
  erase and program "and then restored after the FLASH operation is
  completed".

Halving HCLK moves every peripheral's divisor with it, which is the
program's business and not a storage driver's, so the driver REFUSES
the rate instead of moving the tree: at compile time under a static
`Clock` (through `Clock::flash_needs_halving`), and with a code at run
time under a `DynamicClock`, whose rate is a fact of the moment. A
program that wants to write at 144 MHz steps its own clock down first.

### Three locks, and a wrong key that lasts until a reset

Per the manual: the FPEC lock (KEYR, CTLR.LOCK) guards CTLR and every
standard operation; the FAST lock (MODEKEYR, CTLR.FLOCK) guards the
fast ones on top of it; the OPTION lock (OBKEYR, CTLR.OBWRE) guards the
option bytes. Each takes KEY1 = 0x45670123 then KEY2 = 0xCDEF89AB "in
sequence and continuously", and a wrong sequence "will lock the FPEC
module and FLASH_CTLR register and generate a bus error until the next
system reset" (32.5.2). Nothing in the driver writes a wrong key.

Measured, half of that sentence holds and half does not: the lock DOES
hold until the next system reset - a correct pair written afterwards is
ignored and every write verb answers `refused` - and NO BUS ERROR IS
RAISED. The store of the wrong key is taken in silence, the core takes
no trap, and the only sign is that the engine will not open. So a
mistyped key is not a fault a program can catch; it is a door that
stays shut until the next boot, which is exactly why this driver is the
only place that writes one.

### The status register, and the error the chapter names but does not carry

Per the manual, FLASH_STATR has five defined bits: BSY, WRBSY (the fast
program's own "the buffer has taken the word"), WRPRTERR, EOP and
EHMODS. The last two of the writable ones are cleared by writing a ONE.
There is NO PGERR bit on this family, although CTLR.ERRIE's own
description names "PGERR/WRPRTERR" - so the only error this engine can
report is a write into a protected address.

### The enhanced read mode, which fails an erase in silence

Per the manual (32.3), setting CTLR.EHMOD puts the array in a faster
read mode reported by STATR.EHMODS; leaving it is EHMOD cleared and
then RSENACT written one, in that order. Note 1 of that section says
any erase or program attempted while the mode stands WILL FAIL, and
note 2 says the mode must be left before a Stop. The driver does not
enter the mode on its own, and every erase and program verb refuses
while EHMODS reads back set rather than starting an operation the
chapter promises will not work.

MEASURED ON BOTH PARTS, THE MODE DOES NOT ENGAGE: EHMOD takes the
write and reads back set, and STATR.EHMODS never follows it, however
long the verb waits - on the CH32V203C8, whose option bytes have no
split, and on the CH32V303VC, whose option bytes carry the
flash-and-RAM split the section offers the mode to programs that
outgrew. `enhanced_read(true)` therefore answers false there after a
bounded wait, and the refusal it guards is a path no program on either
part can reach.

### The option bytes

Per the manual (32.6), eight bytes at 0x1FFF F800, each stored beside
its own inverse, reloaded into FLASH_OBR and FLASH_WPR at every system
reset: RDPR (0xA5 = the array is readable), USER (IWDGSW, STOPRST,
STANDYRST, and the memory split above on the parts table 32-4 names),
Data0 and Data1 (two bytes of the user's own), and WRPR0..3 (one bit
per 4 KB sector, a ZERO meaning protected; the last bit covers sectors
31 to 127, which on a part whose window is 128 KB or more is the whole
of its tail).

Two of the USER bits are what the watchdog and power chapters cite:
IWDGSW says whether the independent watchdog waits for software,
STOPRST and STANDYRST whether entering Stop or Standby resets the chip.
Their sense in the register is inverted against the way a reader thinks
about them - 32.4.6 gives 0 for "system will be reset when entering
Stop mode" - so `FlashOptions` states `resets_on_stop` and
`resets_on_standby`, which are the complements.

THE DRIVER READS THEM AND WRITES NONE. Releasing read protection "will
first cause the system to automatically perform a whole chip erasure
operation on the flash memory" (32.6.4); everything else in the area is
provisioning, a statement about the next boot rather than about the
running program. RDP is never written by any verb, the only view of the
area the driver hands out is const, and a store through it does not
compile.

### The signature

Per the manual (31.2), three read-only words at 0x1FFF F7E8 hold a
96-bit unique identifier and a half-word at 0x1FFF F7E0 "the capacity
of the flash memory area" in kilobytes. What that counts is the
section above's: the largest window the part can select, which is the
window itself on the CH32V203C8 - measured, 64 KB against the part
table's 64 KB - and 288 on a CH32V303VC whose window is 256, measured
too.

THE IDENTIFIER IS SIXTY-FOUR BITS AND NOT NINETY-SIX. Measured on a
CH32V203C8T6 and a CH32V303VCT6: the first two words carry the
factory's number and the THIRD reads 0xE339E339, the erased pattern of
this array - a word nobody programmed. A board is known by the pair.

### What an operation costs

Per the CH32V203 datasheet's table 4-17, and the CH32V303's table 4-19
with the same numbers: a 256-byte page program 2 ms typical, a
256-byte page erase 16 ms, a 4 KB sector erase 16 ms. Tables 4-18 and
4-20 rate the endurance at 10 000 cycles minimum with 80 000 measured
typical, and data retention at 20 years. Measured at 96 MHz against a
timer counting microseconds on the peripheral bus, on the CH32V203C8:
**a 256-byte page erase 9.8 ms, a fast page program 1.5 ms, and a
standard half-word program 2.6 ms**. On the CH32V303VC EVERY ERASE
TAKES THE SAME 16.85 ms WHATEVER ITS GRAIN - a 256-byte page of the
window, a 4 KB sector of the tail and a 32 KB block of the tail alike,
the V303 datasheet's 16 ms typical - with a fast page program 1.5 ms in
the window and in the tail and a standard half-word 2.1 ms in both. So
on both parts the two-byte write of the standard method costs MORE than
the fast method spends on a whole page, which is what makes the page
the grain a medium wants.

A WRITE IS A WAIT AND NOT A STALL, which is where this array differs
from what the other families here would predict. The core executes from
the window, and yet it goes on executing while the engine works: across
the erase the kernel's millisecond tick - which counts HANDLER RUNS, so
it can only advance if the core could fetch the handler - advanced by
what the timer measured, nine or ten milliseconds on the CH32V203C8 and
seventeen on the CH32V303VC, for a page of the window and a sector of
the tail alike, and by one or two across a page program. That is what a
window the core reads out of a load and not out of the cells would do.
So an erase is a blocking call for its caller, because the driver polls
BSY, and not a hole in the program: interrupts keep being served
throughout.

## Types and verbs

`Flash` is the engine, a monostate. Its geometry is `page_size`,
`sector_size`, `block_size`, `half_word_size` and the array's three
numbers, `window_bytes`, `tail_bytes` and `array_bytes`, with
`has_tail`, `tail_base` and `tail_end`; `in_window(addr, bytes)`,
`in_tail(addr, bytes)` and `in_array(addr, bytes)` - the first two
together, where every write verb works - are constexpr, so a program
can pin an address it knows when it is built. The locks are `unlock()`
(both of the ones the main array needs), `unlock_standard()` (the
FPEC's alone, for a program that only uses the half-word and 4 KB
verbs), `lock()`, and the three readers `locked()` / `fast_locked()` /
`options_locked()`.
WHICH lock a verb needs is 32.4.4's own bit-by-bit statement and the
driver enforces it: PG, PER, MER and STRT are "a normal unlock", while
FTPG, FTER, BER32, SCKMOD and EHMOD are "a normal unlock + quick
unlock". The unlock WINDOW is the caller's: no verb opens a lock behind
its caller's back, so a medium writing several pages pays for one
window.

The status is `status()`, `busy()`, `write_busy()`, `operation_ended()`,
`errors()` and `clear_flags()`; the interrupt is `interrupts(on_end,
on_error)` with its two readers and `isr()`, the body of the handler on
the flash vector.

`access_clock_whole()` reads SCKMOD and `access_clock_whole(clock,
whole)` writes it, refusing the whole system clock above 60 MHz;
`access_hz(clock)` is what the array is being read at.
`enhanced_read()` and `enhanced_read(on)` are 32.3's mode with its
exit sequence, the setter waiting a bounded number of reads for the
status to follow and answering what it then says. `regs()` is the
register view, for a program that has to stage a sequence these verbs
do not offer.

Reading is `read(addr, span)` and `erased(addr, bytes)` for the window
and `read_tail(clock, addr, span)` for the tail. Erasing is
`erase_page`, `erase_sector`, `erase_block32` and `erase_chip`;
programming is `program_half_word` and `program_page`. Every one of
them takes the CLOCK as its first argument, for the reason above, and
reaches the window and the tail alike. Every verb that takes an address
has a second spelling with the address as a template argument -
`erase_sector<a>(clock)` beside `erase_sector(clock, a)`, and
`read_tail<a>` - where a tail address on a part that has none and a run
past the array are COMPILE errors instead of `refused`.
`erase_chip` additionally takes a `ChipErasePolicy`, whose default
value is `refuse`: the verb exists, has no address, spares nothing, and
does nothing at all unless the caller spells out
`erase_the_running_image`.

Every write verb returns a MASK, not a bool. Zero is success; the low
bits are FLASH_STATR's own (`flash_wrprterr`, and `flash_bsy` for an
operation that never finished); `Flash::refused` is a contract the
CALLER broke (a misaligned address, a span that is not a page, a run
past the array, a policy not passed, an operation attempted locked or
in the enhanced read mode); `Flash::refused_rate` is a rate the SILICON
will not write at - or read the tail at.
The two refusals are bits no status flag uses, so a refusal is never
mistaken for a flash error.

`FlashOptions` decodes what the hardware LOADED, with `sector_protected`
and `address_protected` over the write protection, `split_code` and
`split()` over USER[7:5] and `split_matches_part()`; `FlashOptionArea`
reads the eight bytes as they stand in flash with `consistent()` for
the check the loader makes, `split_code()` and `split()` for the byte
the next reset will load, and `raw()` for a const view of the four
words. `FlashSplit` is one combination, `flash_split_of(table, code)`
the decode, `flash_split_max_kbytes(table)` a table's largest window and
`flash_window_max_bytes` this part's. `DeviceUid` and
`flash_size_kbytes()` are the signature.

`MainFlashPartition` is where the storage line is drawn and
`MainFlash<Clock>` is the medium behind
[util/nv_heap.hpp](../../brio/util/nv_heap.hpp)'s `FlashMedia`
contract - see below.

## How to use it

One page of the storage zone rewritten, at a rate that needs nothing:

```cpp
using Clk = brio::Clock<brio::ClockSource::pll, 96'000'000>;
constexpr uint32_t at = brio::MainFlashPartition::storage_base;

uint8_t page[brio::Flash::page_size] = { /* ... */ };
if (brio::Flash::unlock()) {
    uint32_t err = brio::Flash::erase_page(Clk{}, at);
    if (err == 0u) {
        err = brio::Flash::program_page(Clk{}, at, page);
    }
    brio::Flash::lock();
}
```

The same through the medium, which carries its own bounds:

```cpp
using Store = brio::MainFlash<Clk>;
static_assert(brio::FlashMedia<Store>);

const brio::FlashZone z = Store::zones()[0];
(void)Store::erase(z.floor);
(void)Store::program(z.floor, page);
```

Two bytes changed in place, the standard way:

```cpp
(void)brio::Flash::unlock_standard();
(void)brio::Flash::program_half_word(Clk{}, at + 4u, 0xBEEF);
brio::Flash::lock();
```

Two bytes of the tail, the standard way, and read back - at a rate
whose access clock is inside the rating; `erase_page`, `program_page`
and `erase_block32` take a tail address the same way:

```cpp
constexpr uint32_t cell = brio::Flash::tail_end - brio::Flash::sector_size;
(void)brio::Flash::unlock_standard();
(void)brio::Flash::erase_sector<cell>(Clk{});
(void)brio::Flash::program_half_word<cell>(Clk{}, 0xBEEF);
brio::Flash::lock();

uint16_t back = 0;
(void)brio::Flash::read_tail(Clk{}, cell, {reinterpret_cast<uint8_t*>(&back), 2u});
```

What the option bytes say about this part, and who it is:

```cpp
const brio::FlashOptions opt = brio::Flash::options();
const bool watchdog_is_mine = opt.iwdg_software;
const bool stop_is_safe = !opt.resets_on_stop;
const bool split_as_linked = opt.split_matches_part();
const brio::DeviceUid id = brio::Flash::uid();
```

## The storage medium

`MainFlash<Clock>` puts a `FlashMedia` over the LAST 4 KB of the
zero-wait WINDOW on every part of both series: sixteen pages of 256
bytes, which is exactly one write-protection unit, so the storage can be
protected or left open without touching a sector the image occupies -
and the window and not the tail, although both of the engine's methods
write the tail: a word of the tail costs four times a word of the
window to read and carries the rate contract, so a medium its program
reads is better off where the image is. Every linker script of
this project stops its flash region 4 KB short of the window
([ch32vx03/ld/ch32v203c8.ld](../../ch32vx03/ld/ch32v203c8.ld) and its
twelve siblings), so nothing the compiler emits can land in the zone and
the floor is a CONSTANT rather than a symbol that moves with every
build. `__brio_rom_end` is read back as the proof: a script edited past
the line makes `zones()` answer a band whose floor is its ceiling,
which every user of the contract refuses on.

THE CELL IS THE PAGE, 256 bytes, and `erase_size` and `write_cell`
coincide.

NOTHING STANDS ABOVE THE MEDIUM ON THIS FAMILY - no `NvHeap`, no
`NvJournal` - and that is a decision rather than an omission. What a
program gets here is a band of flash with bounds it cannot write
outside of; whatever structure it keeps there is its own. Two facts
stand behind the decision, and the second is this silicon's: the erased
pattern is not 0xFF, which
[util/nv_journal.hpp](../../brio/util/nv_journal.hpp) uses to find the
end of a half (the heap judges a map version by its magic and its CRC
and would not care); and a 256-byte cell is a coarse grain for small
values, one page an entry. A two-byte cell has a mechanism here and a
question over it: a half-word cell of the WINDOW takes pass after pass
between erases and keeps the last one across a system reset, which is
what such a store needs - but the tail's cells do not, and what a
window cell programmed twice reads after the window is loaded again is
not measured (the section on the erased pattern). So the page is the
cell by the decision above, and a two-byte cell would be born with that
measurement.

THE CLOCK IS PART OF THE MEDIUM'S TYPE, which is the one place this
stratum's `FlashMedia` differs in shape from the others'. The
contract's `program()` and `erase()` take an address and nothing else,
and an erase here is legal only below 120 MHz, so the rate has to
arrive with the type: `MainFlash<Clk>` is written with the program's
own clock, which makes the refusal a compile error under a static one
and a `false` under a dynamic one.

## Bench findings

`test_vx03_nvm` measured the chapter on a CH32V203C8T6 and on a
CH32V303VCT6 with nothing wired. The suite runs on a `DynamicClock` of
three rates - 96 MHz to work at, 48 MHz where SCKMOD may be set at all,
144 MHz to prove the refusal - which is what a program that writes flash
on this family really looks like. Thirty-two verdicts in `z`, which
spends not one cycle of the array, and nine letters by name: `w`, `v`,
`q`, `u` and `x` each spend one or two erases of the medium's zone, `y`
locks the engine with a wrong key and reboots, `t` and `s` spend one
erase each at the end of the tail, and `r` reads what `s` left there at
144 MHz. What follows holds for both parts unless a part is named.

- **The locks.** Both are shut at every boot and the option lock with
  them; KEYR's pair opens the FPEC lock alone and leaves the fast one
  shut; MODEKEYR's opens that on top; a one written into each bit
  closes both. A fast verb asked for with only the FPEC lock open is
  refused with nothing started, and `unlock()` on an open engine writes
  no key and answers true, so a medium may open the window at every
  call.
- **A wrong key locks the engine until the next system reset, and
  raises no bus error**, on both parts. After KEY1 followed by a zero,
  the correct pair is ignored and an erase answers `refused`; the core
  took no trap (the next boot's reset flags are the software reset's
  alone), and the reset opens the engine again.
- **The option bytes.** On the CH32V203C8, FLASH_OBR 0x000000FC and
  USER 0x3F; on the CH32V303VC, FLASH_OBR 0x0000027C and USER 0x9F -
  the split's code 100, 256 KB of window and 64 of SRAM, loaded and
  stored alike and the part table's. On both, FLASH_WPR 0xFFFFFFFF,
  RDPR 0xA5, Data0 and Data1 zero, every byte agreeing with its own
  inverse and OBERR clear: not read-protected, the independent watchdog
  left to software, no reset on Stop or Standby, and no sector
  write-protected - the medium's own among them. FLASH_OBR's bits [9:2]
  are the USER byte whole on both.
- **The signature.** FLACAP 64 KB on the CH32V203C8 against the part
  table's 64, and 288 on the CH32V303VC, the largest window its option
  byte can select, against a window of 256; the identifiers 0x865DABCD
  0xEF0CBCE2 and 0x0C9BABCD 0x74B0BC48, each with a third word that
  reads the erased pattern.
- **The engine idle, and five refusals that cost nothing.** BSY, WRBSY
  and WRPRTERR clear; a page address that is not a page, an address
  past the array, an odd half-word, a span that is not a whole page and
  a chip erase with no policy all come back `refused` (0x80000000) with
  nothing started. EOPIE and ERRIE arm and disarm, and an armed vector
  with no operation running is silent.
- **The enhanced read mode does not engage** on either part: EHMOD
  reads back set, EHMODS never follows.
- **The medium's geometry.** On the CH32V203C8 the window is 64 KB, the
  linker's region ends exactly at 0xF000 and the zone is
  0xF000..0x10000; on the CH32V303VC the window is 256 KB, the region
  ends exactly at 0x3F000 and the zone is 0x3F000..0x40000 - sixteen
  cells of 256 bytes each, a fresh zone reading 0xE339E339 word after
  word.
- **SCKMOD, and what it is worth to the window.** At 96 MHz the whole
  system clock is REFUSED as the access clock, which leaves the array
  at 48 MHz under the reset value's half. Stepped down to 48 MHz, where
  both settings are legal, the bit was written and read back in both
  positions - and neither path to the window got measurably faster:
  51200 instructions in a straight run took 57243 core cycles with the
  access clock halved and 57442 with it whole on the CH32V203C8, 57460
  and 57262 on the CH32V303VC, and 10240 words read out of a constant
  array 52603 against 52563, and 52596 against 52614. At this rate what
  the core waits for is not the flash.
- **The rate refusal, on the silicon.** Switched to 144 MHz, an erase
  and a program come back `refused_rate` (0x40000000) with nothing
  written and the medium's `erase()` answers false; back at 96 MHz the
  access clock is 48 MHz against the chapter's 60 MHz ceiling.
- **The page cycle** (letter `w`, one erase): a 256-byte page erased
  with no error in 9770 us on the CH32V203C8 and 16862 us on the
  CH32V303VC, reading 0xE339E339 afterwards; a whole page
  fast-programmed in 1540 and 1532 us and read back BYTE FOR BYTE; the
  kernel's tick advancing across the erase by what the timer measured
  and by one or two across the program. Then a second program into the
  same page with no erase: **no error, and a page holding neither
  pattern nor their bitwise AND** - word 0 held 0x71BECCE5,
  0x17610429 was programmed over it, and it read 0x15E60CED where the
  AND would have been 0x11200421, the same word on both parts.
- **FLASH_ADDR is not part of the fast page program** (letter `u`, two
  erases), on both parts: the sequence staged WITHOUT the store into
  that register, and with the register left pointing at another erased
  page, wrote the page its sixty-four stores named and left the other
  untouched. The driver makes the store anyway - it costs one
  instruction and it is what the sister family's chapter lists for the
  same operation - but the address the silicon uses is the one the data
  stores carry.
- **The half-word program** (letter `v`, one erase): 0xF0F0 written
  into an erased cell of the window in 2.6 ms on the CH32V203C8 and 2.1
  on the CH32V303VC and read back exactly, the half-words beside it
  untouched at 0xE339E339. Then five more passes into the same cell -
  0xF000, 0x0F0F, 0xFFFF, 0x0000, 0xA5A5 - every one reporting no error
  and reading back EXACTLY what was written, and the page left erased
  behind the letter.
- **The second pass across a reset** (letter `q`, one erase, reboots):
  on the CH32V303VC a window cell given 0x9659 and then 0x0F0F read
  0x0F0F before a software reset and 0x0F0F after it.
- **The FlashMedia contract on the silicon** (letter `x`, one erase,
  reboots), on both parts: a cell erased and programmed through
  `erase()` and `program()`, read back byte for byte through `read()`,
  with the engine locked again afterwards; an address below the zone,
  one past its top, a span that is not a whole cell and an erase
  outside the zone all refused; and the cell still holding its first
  word after a software reset.
- **The tail's price** (letter `i`, in `z`), on the CH32V303VC: a
  kilobyte of words read with interrupts masked cost 5.01 core cycles a
  word out of the window through the alias and 5.00 through the array's
  own address, and 20.04 out of the tail, at 96 MHz; at 48 MHz the
  window read 5.02 with SCKMOD at half and 5.01 with it whole, the tail
  20.05 and 12.03. Every run of the tail read the same words, through
  the raw loop and through `read_tail()`; the tail's read refused a run
  across the line, one past the end and 144 MHz, and every write verb a
  run past the array.
- **The tail written the standard way** (letter `t`, one sector erase),
  on the CH32V303VC: the tail's last sector erased in 16848 to 16856 us
  with the FPEC lock alone, reading the erased pattern throughout, the
  tick advancing 17 ms across it; thirty-two half-words programmed - the
  first in 2077 us, the rest in 64 ms - and read back exactly through
  the tail's read, the rest of the sector still erased; and a second
  pass, 0x0F0F over 0x9659, reporting no error and reading 0x0E0F.
- **The tail written the fast way** (letter `s`, one 32 KB erase and
  one page erase), on the CH32V303VC: the tail's last 32 KB erased by
  `erase_block32()` in 16852 us over a page that was not erased, every
  word of the block reading the erased pattern afterwards; a page
  programmed by `program_page()` in 1524 us and read back byte for
  byte; the same page erased by `erase_page()` in 16848 us.
- **The tail read at 144 MHz** (letter `r`, no wear, outside the
  rating), on the CH32V303VC: the page letter `s` leaves, read by hand
  three times with the access clock at 72 MHz, differed from its
  pattern in no byte of 768, at 20.05 core cycles a word; the driver's
  read refused the rate with its code, and back at 96 MHz the tail read
  what it read before.

## Not covered yet

Driver gaps, each with its reason:

- **Programming or erasing the option bytes** (OBPG, OBER, the OBKEYR
  unlock), DECLINED: releasing read protection erases the whole chip
  (32.6.4) and every other byte in that area is provisioning - a
  statement about the next boot, which on the other targets of this
  project is the business of a separate tool and not of a running
  program. The bytes are decoded read-only and RDP is never written.
- **`NvHeap` and `NvJournal` over the medium**, by decision: what a
  program on this family gets is the band and its bounds. The journal
  would in any case need to be taught an erased pattern that is not
  0xFF before it could stand here: a fresh zone reads 0xE339E339 word
  after word (measured below), where the format finds the end of a half
  by looking for a cell of 0xFF bytes
  ([../design/nv-journal.md](../design/nv-journal.md)).
- **A two-byte cell for the medium.** The chapter's standard
  programming writes a half-word, and a cell of the window takes pass
  after pass between erases and keeps the last across a system reset,
  so the grain a small-value store wants has a mechanism here - it is
  not offered because nothing stands above the medium on this family
  (the decision above), because it would cost 2.1 to 2.6 ms a write
  against 1.5 for a whole page, and because the window's second pass
  across a load of the window is not measured (below).
- **The system boot loader** at 0x1FFF 8000 and the vendor
  configuration word 32.1's note names: 28 KB of WCH's own, locked
  before delivery, and nothing in this project calls it.
- **Code or data placed in the tail by the build.** The CH32V303RC's and
  VC's linker scripts name the tail as a region nothing is placed in,
  and the CH32V203's do not name it at all, so what lives there is what
  a program writes with the engine's verbs. A word of the tail costs
  four times a word of the window to read (measured), so a section
  placed there is a decision that trades speed for room, born with the
  first program that needs the room.

Implemented but not bench-verified, each with what would measure it:

- **The eleven parts other than the CH32V203C8 and the CH32V303VC.** The
  window's size, and therefore where the medium's zone begins, folds
  through each part's own table, and so do the tail and the split; the
  whole stratum compiles for all thirteen both ways the hardware
  prologue can be built (`brio check ch32vx03`), and every image of the
  32 KB tier links against the shortened region - `test_vx03_nvm` as two
  images there. What would measure them is a board.
- **The CH32V203RB's split**, the second of table 32-4's tables: decoded
  (128, 144 or 160 KB of window) and written by no verb, with the part
  table stating the factory's 128 + 64 and the signature expected to
  read 160, the table's largest; what would measure both is that part
  on a board.
- **The tail of the CH32V203.** Its three numbers, both methods'
  erases and programs there, the tail's read and its rate refusal are
  compiled for all nine CH32V203 parts and measured on the CH32V303VC;
  what would measure them on the other series is `test_vx03_nvm`'s
  letters i, t, s and r on the CH32V203C8, whose 160 KB of tail no
  program there has read yet.
- **The chip erase.** Written, refused unless a policy is spelled out,
  and destroying the running image, so it does not belong in a suite
  that must leave the board usable. What would measure it is a board
  that can be reflashed unattended.
- **The window's second pass across a load of the window.** A cell of
  the window programmed twice keeps the second value across a system
  reset (letter `q`); what it reads after a power-up or a wake from
  Standby - which load the zero-wait area again, per the CH32V303
  datasheet's table 4-18 - is not measured. What would measure it is
  letter `q` with a Standby exit in place of the software reset, its
  wake armed on the RTC's alarm or the WKUP pad, or a power cycle. One
  reading exists: after a boot of the CH32V303VCT6 that came with SBF
  clear - a power-on-class reset, the one kind that reloads the window -
  the cell still read its second value; an observation with the reset's
  kind unwitnessed, not the measurement.
- **The flash interrupt raised by a real operation.** EOPIE and ERRIE
  arm, disarm and stay silent with nothing running, and `isr()` is
  written; what would measure it is an operation started with the
  vector enabled, which asks the suite to erase once more than it
  needs to.
