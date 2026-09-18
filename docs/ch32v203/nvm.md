# Flash and the option bytes (CH32V203)

The flash memory, the user option bytes and the electronic signature.
Documents of record: the CH32F/V20x_V30x_V31x reference manual V2.3
(chapter 32 for the array, the engine, the two programming methods, the
four erase grains and the option bytes; chapter 31 for the signature -
both chapters apply to the whole family, with no per-class note in
either) and the CH32V203 datasheet V2.8 (table 2-1 for each part's
array, table 4-17 for the programming frequency and the operation
times, table 4-18 for the endurance). Drivers:
[brio/ch32v203/nvm.hpp](../../brio/ch32v203/nvm.hpp) (the engine) and
[brio/ch32v203/nvm_flash.hpp](../../brio/ch32v203/nvm_flash.hpp) (the
storage medium over it). Reference suite: `test_v203_nvm`.

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
"will generate a bus error" (32.5.3), and the fast verbs "can only be
placed in the zero-wait area" (32.2.1) - which on this series is not a
restriction at all, because the datasheet's own note to table 2-1 says
what a part's flash size COUNTS: "flash bytes represent zero-wait run
area R0WAIT. For the V203 series, non-zero-wait area is (224K -
R0WAIT)". Every byte a user program may occupy on these nine parts is
therefore in the zero-wait area, and code running from the image
qualifies by construction.

### The erased pattern is not 0xFF

Per the manual, said four times (32.5.4, 32.5.7 twice, 32.6.3): "after
erasing is successful, word read - 0xe339e339, half word read - 0xe339,
even address byte read - 0x39, odd address read 0xe3". An erased cell
of this array reads back **0xE339E339**, where every other flash brio
writes leaves all ones - and the sister family's chapter promises 0xFF
for the same verbs, so this is this silicon's and not a habit of the
vendor's. `Flash::erased_word` is that pattern and `Flash::erased()`
the run test over it; what it costs a structure built on top is under
the medium, below. Measured on an erased page, and again where nobody
looked for it: the THIRD word of this die's own unique identifier reads
0xE339E339 too.

AND THE CELLS DO NOT BEHAVE LIKE BITS THAT ONLY FALL, which is the
other half of the same fact and the part a structure above the medium
would get wrong. Measured: a HALF-WORD CELL TAKES PASS AFTER PASS
between erases - five in a row, 0xF0F0 then 0xF000, 0x0F0F, 0xFFFF,
0x0000 and 0xA5A5, each reading back EXACTLY what was written, so bits
come back as well as go away. And a PAGE PROGRAMMED TWICE with no erase
between reports no error and ends up holding neither pattern and not
their bitwise AND either. A program that wants a cell to hold what it
wrote erases first; a program that wants to change two bytes in place
may simply write them again.

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

MEASURED ON THE CH32V203C8, THE MODE DOES NOT ENGAGE: EHMOD takes the
write and reads back set, and STATR.EHMODS never follows it, however
long the verb waits. That fits the section's own premise, which offers
the mode to a program whose code has outgrown the flash-and-RAM split
RAM_CODE_MOD chooses - a USER field table 32-4 gives to the OTHER
device classes and not to this one. `enhanced_read(true)` therefore
answers false here after a bounded wait, and the refusal it guards is a
path no program of this part can reach.

### The option bytes

Per the manual (32.6), eight bytes at 0x1FFF F800, each stored beside
its own inverse, reloaded into FLASH_OBR and FLASH_WPR at every system
reset: RDPR (0xA5 = the array is readable), USER (IWDGSW, STOPRST,
STANDYRST, and bits that belong to other device classes), Data0 and
Data1 (two bytes of the user's own), and WRPR0..3 (one bit per 4 KB
sector, a ZERO meaning protected; the last bit covers sectors 31 to
127).

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
96-bit unique identifier and a half-word at 0x1FFF F7E0 the array's
size in kilobytes. The size is the DIE's answer to the question the
part table answers from the build, so a program that reads both has a
check on its own build - measured, 64 KB against the part table's 64 KB.

THE IDENTIFIER IS SIXTY-FOUR BITS AND NOT NINETY-SIX. Measured on a
CH32V203C8T6: the first two words carry the factory's number and the
THIRD reads 0xE339E339, the erased pattern of this array - a word
nobody programmed. A board is known by the pair.

### What an operation costs

Per the datasheet's table 4-17: a 256-byte page program 2 ms typical, a
256-byte page erase 16 ms, a 4 KB sector erase 16 ms. Table 4-18 rates
the endurance at 10 000 cycles minimum with 80 000 measured typical,
and data retention at 20 years. Measured at 96 MHz against a timer
counting microseconds on the peripheral bus: **a 256-byte page erase
9.8 ms, a fast page program 1.5 ms, and a standard half-word program
2.6 ms** - so the two-byte write of the standard method costs MORE than
the fast method spends on a whole page, which is what makes the page
the grain a medium wants.

A WRITE IS A WAIT AND NOT A STALL, which is where this array differs
from what the other families here would predict. The core executes from
it, and yet it goes on executing while the engine works: across the 9.8
ms erase the kernel's millisecond tick - which counts HANDLER RUNS, so
it can only advance if the core could fetch the handler out of the same
array - advanced by the same nine or ten, and by one across the page
program. So an
erase is a blocking call for its caller, because the driver polls BSY,
and not a hole in the program: interrupts keep being served throughout.

## Types and verbs

`Flash` is the engine, a monostate. The locks are `unlock()` (both of
the ones the main array needs), `unlock_standard()` (the FPEC's alone,
for a program that only uses the half-word and 4 KB verbs), `lock()`,
and the three readers `locked()` / `fast_locked()` / `options_locked()`.
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

Reading is `read(addr, span)` and `erased(addr, bytes)`. Erasing is
`erase_page`, `erase_sector`, `erase_block32` and `erase_chip`;
programming is `program_half_word` and `program_page`. Every one of
them takes the CLOCK as its first argument, for the reason above.
`erase_chip` additionally takes a `ChipErasePolicy`, whose default
value is `refuse`: the verb exists, has no address, spares nothing, and
does nothing at all unless the caller spells out
`erase_the_running_image`.

Every write verb returns a MASK, not a bool. Zero is success; the low
bits are FLASH_STATR's own (`flash_wrprterr`, and `flash_bsy` for an
operation that never finished); `Flash::refused` is a contract the
CALLER broke (a misaligned address, a span that is not a page, a policy
not passed, an operation attempted locked or in the enhanced read
mode); `Flash::refused_rate` is a rate the SILICON will not write at.
The two refusals are bits no status flag uses, so a refusal is never
mistaken for a flash error.

`FlashOptions` decodes what the hardware LOADED, with `sector_protected`
and `address_protected` over the write protection; `FlashOptionArea`
reads the eight bytes as they stand in flash with `consistent()` for
the check the loader makes and `raw()` for a const view of the four
words. `DeviceUid` and `flash_size_kbytes()` are the signature.

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

What the option bytes say about this part, and who it is:

```cpp
const brio::FlashOptions opt = brio::Flash::options();
const bool watchdog_is_mine = opt.iwdg_software;
const bool stop_is_safe = !opt.resets_on_stop;
const brio::DeviceUid id = brio::Flash::uid();
```

## The storage medium

`MainFlash<Clock>` puts a `FlashMedia` over the LAST 4 KB of the array
on every part of the series: sixteen pages of 256 bytes, which is
exactly one write-protection unit, so the storage can be protected or
left open without touching a sector the image occupies. Every linker
script of this project stops its flash region 4 KB short of the array
([ch32v203/ld/ch32v203c8.ld](../../ch32v203/ld/ch32v203c8.ld) and its
eight siblings), so nothing the compiler emits can land in the zone and
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
values, one page an entry. A two-byte cell would be possible here - the
bench measured a half-word cell taking pass after pass between erases,
which is the mechanism such a store needs - so the page is the cell by
the decision above and not for want of one.

THE CLOCK IS PART OF THE MEDIUM'S TYPE, which is the one place this
stratum's `FlashMedia` differs in shape from the others'. The
contract's `program()` and `erase()` take an address and nothing else,
and an erase here is legal only below 120 MHz, so the rate has to
arrive with the type: `MainFlash<Clk>` is written with the program's
own clock, which makes the refusal a compile error under a static one
and a `false` under a dynamic one.

## Bench findings

`test_v203_nvm` measured the chapter on a CH32V203C8T6 with nothing
wired. The suite runs on a `DynamicClock` of three rates - 96 MHz to
work at, 48 MHz where SCKMOD may be set at all, 144 MHz to prove the
refusal - which is what a program that writes flash on this family
really looks like. Twenty-five verdicts in `z`, which spends not one
cycle of the array, and five letters by name that spend one erase each
- two for the one that asks about FLASH_ADDR.

- **The locks.** Both are shut at every boot and the option lock with
  them; KEYR's pair opens the FPEC lock alone and leaves the fast one
  shut; MODEKEYR's opens that on top; a one written into each bit
  closes both. A fast verb asked for with only the FPEC lock open is
  refused with nothing started, and `unlock()` on an open engine writes
  no key and answers true, so a medium may open the window at every
  call.
- **A wrong key locks the engine until the next system reset, and
  raises no bus error.** After KEY1 followed by a zero, the correct
  pair is ignored and an erase answers `refused`; the core took no trap
  (the next boot's reset flags are the software reset's alone), and the
  reset opens the engine again.
- **The option bytes on this part.** FLASH_OBR 0x000000FC and
  FLASH_WPR 0xFFFFFFFF; the area at 0x1FFF F800 reads RDPR 0xA5, USER
  0x3F, Data0 and Data1 zero, every byte agreeing with its own inverse
  and OBERR clear. Decoded: not read-protected, the independent
  watchdog left to software, no reset on Stop or Standby, and no sector
  write-protected - the medium's own among them.
- **The signature.** FLACAP 64 KB against the part table's 64 KB; the
  identifier 0x865DABCD 0xEF0CBCE2 and a third word that reads the
  erased pattern.
- **The engine idle, and five refusals that cost nothing.** BSY, WRBSY
  and WRPRTERR clear; a page address that is not a page, an address
  past the array, an odd half-word, a span that is not a whole page and
  a chip erase with no policy all come back `refused` (0x80000000) with
  nothing started. EOPIE and ERRIE arm and disarm, and an armed vector
  with no operation running is silent.
- **The enhanced read mode does not engage on this device class.**
  EHMOD reads back set, EHMODS never follows.
- **The medium's geometry.** The array 64 KB, the linker's region
  ending exactly at 0xF000, the zone 0xF000..0x10000 as sixteen cells
  of 256 bytes, and a fresh zone reading 0xE339E339 word after word.
- **SCKMOD, and what it is worth.** At 96 MHz the whole system clock is
  REFUSED as the access clock, which leaves the array at 48 MHz under
  the reset value's half. Stepped down to 48 MHz, where both settings
  are legal, the bit was written and read back in both positions - and
  neither path to the array got measurably faster: 51200 instructions
  in a straight run took 57243 core cycles with the access clock halved
  and 57442 with it whole, and 10240 words read out of a constant array
  52603 cycles against 52563. At this rate what the core waits for is
  not the flash, whichever of the two settings it is read at.
- **The rate refusal, on the silicon.** Switched to 144 MHz, an erase
  and a program come back `refused_rate` (0x40000000) with nothing
  written and the medium's `erase()` answers false; back at 96 MHz the
  access clock is 48 MHz against the chapter's 60 MHz ceiling.
- **The page cycle** (letter `w`, one erase): a 256-byte page erased in
  9770 us with no error, reading 0xE339E339 afterwards; a whole page
  fast-programmed in 1540 us and read back BYTE FOR BYTE; the kernel's
  tick advancing nine to ten milliseconds across the erase and one
  across the program. Then a second program into the same page with no erase:
  **no error, and a page holding neither pattern nor their bitwise
  AND** - word 0 held 0x71BECCE5, 0x17610429 was programmed over it,
  and it read 0x15E60CED where the AND would have been 0x11200421.
- **FLASH_ADDR is not part of the fast page program** (letter `u`, two
  erases): the sequence staged WITHOUT the store into that register,
  and with the register left pointing at another erased page, wrote the
  page its sixty-four stores named and left the other untouched. The
  driver makes the store anyway - it costs one instruction and it is
  what the sister family's chapter lists for the same operation - but
  the address the silicon uses is the one the data stores carry.
- **The half-word program** (letter `v`, one erase): 0xF0F0 written
  into an erased cell in 2.6 ms and read back exactly, the half-words
  beside it untouched at 0xE339E339. Then five more passes into the
  same cell - 0xF000, 0x0F0F, 0xFFFF, 0x0000, 0xA5A5 - every one
  reporting no error and reading back EXACTLY what was written, and the
  page left erased behind the letter.
- **The FlashMedia contract on the silicon** (letter `x`, one erase,
  reboots): a cell erased and programmed through `erase()` and
  `program()`, read back byte for byte through `read()`, with the
  engine locked again afterwards; an address below the zone, one past
  its top, a span that is not a whole cell and an erase outside the
  zone all refused; and the cell still holding its first word after a
  software reset.

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
  0xFF before it could stand here.
- **A two-byte cell for the medium.** The chapter's standard
  programming writes a half-word and the bench measured that such a
  cell takes pass after pass between erases, so the grain a small-value
  store wants EXISTS here - it is not offered because nothing stands
  above the medium on this family (the decision above), and it would cost
  2.6 ms a write against 1.5 for a whole page.
- **The system boot loader** at 0x1FFF 8000 and the vendor
  configuration word 32.1's note names: 28 KB of WCH's own, locked
  before delivery, and nothing in this project calls it.

Implemented but not bench-verified, each with what would measure it:

- **The eight parts other than the CH32V203C8.** The array's size, and
  therefore where the medium's zone begins, folds through each part's
  own table, and the whole stratum compiles for all nine both ways the
  hardware prologue can be built (`brio check ch32v203`); every image of
  the 32 KB tier links against the shortened region. What would measure
  them is a board.
- **The CH32V203RB's 128 KB array**, whose option bytes carry a
  SRAM_CODE_MODE field that moves the line between flash and RAM
  (32.4.6): the field is read as part of the option word and no verb
  acts on it, and the part table states the factory split. What would
  measure it is that part on a board.
- **The 32 KB block erase and the chip erase.** Both are written and
  both destroy the running image on at least one part of the series -
  the whole array is one block on the 32 KB parts - so neither belongs
  in a suite that must leave the board usable. What would measure them
  is a board that can be reflashed unattended.
- **The 4 KB sector erase.** Written and refused nowhere, but the only
  sector a suite could spend it on is the medium's own, and the page
  erase is what the medium uses; what would measure it is a program
  that keeps more than one page of storage.
- **The flash interrupt raised by a real operation.** EOPIE and ERRIE
  arm, disarm and stay silent with nothing running, and `isr()` is
  written; what would measure it is an operation started with the
  vector enabled, which asks the suite to erase once more than it
  needs to.
