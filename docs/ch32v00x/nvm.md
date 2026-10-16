# FLASH (CH32V00x)

The flash program/erase engine of RM ch. 18 and the two storage media
over it - the `FlashMedia` backends that carry util/nv_heap.hpp's block
allocator and util/nv_journal.hpp's value journal on this silicon,
their fourth. Documents of record: the CH32V00X reference manual V1.5
(18.2 for the security model, 18.3 for the registers, 18.4 for the
procedures), the CH32V006 datasheet V2.0 (the array's size per part).

## What the silicon does

- **One array, a page its unit**: 62 KB in 248 pages of 256 bytes on
  the CH32V006, 16 KB in 256 pages of 64 bytes on the CH32V003
  (`device::flash_page_bytes`), at address 0 (where the core fetches
  from) and aliased at 0x0800 0000 (the address the chapter's
  procedures write through). No second bank, no read-while-write: a
  program or erase stalls the core for its duration.
- **Programming is by whole page.** The main memory is written by FAST
  PAGE PROGRAMMING (18.4.5): the page's words loaded one by one into
  an internal buffer (64 on the CH32V006, 16 on the CH32V003), each
  followed by BUFLOAD, then one STRT. The CH32V006's FLASH_CTLR has no
  half-word PG mode; the CH32V003's has the standard one (its bit 0),
  which this driver does not use - the page is the write cell on both
  parts (the gap list says what would decide otherwise).
- **Three erase grains**: the fast page (FTER), the standard sector of
  1 KB (PER), the whole array (MER), plus on the CH32V006 a 32 KB block
  erase (BER32) for the lower half of the array.
- **Two locks.** KEYR's key pair opens the FPEC (LOCK), MODEKEYR's the
  fast operations (FLOCK); each pair must be written in order and
  consecutively, and a wrong sequence locks the block until the next
  system reset (18.4.2, 18.4.4).
- **HSI must be on** while programming or erasing (18.2.2's note).
- **The first 2 KB (pages 0-7) are write-protected regardless of WPR**
  when read protection is on; otherwise FLASH_WPR protects 2 KB units,
  a clear bit meaning protected, loaded from the option bytes at reset.

## Types and verbs

`Flash` ([brio/ch32v00x/nvm.hpp](../../brio/ch32v00x/nvm.hpp)) is the
engine, monostate: `unlock()` opens both locks and `lock()` shuts
both, `erase_page()`, `erase_sector()` and `program_page()` each wait
for BSY, run the chapter's sequence and return the error bits STATR
left (WRPRTERR) or zero - and `Flash::refused` (a bit no STATR error
uses) when the caller broke the contract with a misaligned address or
a span that is not a page, so a refusal is never mistaken for a
silicon error. Every address the engine takes is the array's own,
from 0, and `alias_of()` adds 0x0800 0000 where the chapter wants it.

[brio/ch32v00x/nvm_flash.hpp](../../brio/ch32v00x/nvm_flash.hpp) draws
a CONSTANT PARTITION, each part's (`device::flash_program_bytes`,
`device::flash_journal_pages`): the linker script gives the linker the
first 40 KB of the CH32V006's array and the first 15 KB of the
CH32V003's, and states the boundary as `__brio_rom_end`; the media
read it back and refuse to open (a zone with its floor at its ceiling)
if the script is ever edited past their floor. Above it, on the
CH32V006, `MainFlash` is the heap's 16 KB (0xA000..0xE000, 64 pages,
the map pair in the top two) and `MainFlashJournalZone` the journal's
6 KB attic (0xE000..0xF800, 24 pages, two halves of twelve); on the
CH32V003 the partition is the journal's 1 KB attic alone
(0x3C00..0x4000, 16 pages of 64 bytes, two halves of eight) - there
is no heap share on a 16 KB array and no `MainFlash` there
(`BRIO_CH32_HAS_FLASH_HEAP`). Both media declare `erase_size =
write_cell =` the page: THE PAGE IS THE CELL, and every journal entry
and every heap append costs a page (a half of twelve pages holds
twelve entries, of eight eight; the geometry assertion allows up to
ten ids on the CH32V006 and six on the CH32V003). Both open the locks
around the one operation and shut them after; the bounds check on
every program and erase is what keeps a miscounting heap from
programming the running image.

## How to use it

```cpp
using Heap = brio::NvHeap<brio::MainFlash, 8, 2>;
using Journal = brio::NvJournal<brio::MainFlashJournalZone, 6, 32, 12>;
Heap heap;
Journal journal;

heap.mount();                           // survival-aware, reports losses
auto w = heap.alloc(record_id, bytes);  // a block, appended then sealed
journal.mount();
journal.save<Calibration>(id, cal);     // a page program, ~2 ms, the core stalled
journal.save_reserved(id, value);       // the panic path: one program, no erase
```

An ordinary save is a pause of the whole program of about two
milliseconds, and a save that collects one of some 78 ms on the
CH32V006, not a background operation: this is the array the core
executes from, and every interrupt waits too - the kernel's tick among
them, which counts ONE tick for each stall of two or three periods (the
bench findings). Nothing that must run every tick should sit in the
same turn as a save, and kernel time runs slow across one.

## Bench findings

The reference suite is `test_ch32_nvm` (46 verdicts in `z`; letter `w`
wipes the partition) on the CH32V006K8U6 at 48 MHz, and 37 on the
CH32V003F4P6 - one source: no heap letter there, the page letters on
the attic's pages, the sector letter erasing the attic itself, the
journal at eight cells a half. Every time is read on TIM2 counting
microseconds: an operation stalls the core longer than a tick period,
so the STK - whose reloads inside one stall raise its pending bit once
- cannot time it.

- **The engine, measured with the core stalled** (letter `b` fifteen
  runs, `d` three): on the CH32V006 a fast page erase 2925 to 2927 us,
  a fast page program 1886 to 1890 us and a 1 KB sector erase 3124 to
  3126 us - each inside the datasheet's maximum (CH32V006 datasheet table 3-14:
  3.0, 2.0 and 3.3 ms), the sector's four pages erased in about the time
  of one. The media erase by page because the heap and the journal count
  in erase units and a smaller unit wastes less; a media that erased by
  sector would be some four times faster per byte, and is a choice the
  suite's numbers make available.
- **The stall holds the kernel's tick off**: across a page erase of
  three tick periods the tick advanced ONE (fifteen runs of fifteen) -
  the STK's reloads inside a stall raise its one pending bit once, and
  kernel time loses the periods between.
- **The journal's saves, timed** (letter `f`, sixteen saves through a
  collection or two, eighteen runs): an ordinary save 1950 to 1970 us -
  one page program and some 80 us of the journal's own work; a save
  that collects 76.0 ms with two live ids and 78.0 ms with three - both
  halves' 24 page erases, the copies and the save's own program, which
  the engine's figures above add up to (24 x 2.93 + 3 x 1.89 = 76.0 ms)
  - with the kernel's tick advancing 28 to 32 across it: some 45 to 50
  ticks of kernel time lost to the stall.
- **A programming session through the WCH-LinkE leaves the partition
  virgin**: the journal holding three live ids mounted them again after
  the probe's reset alone, and mounted EMPTY after each programming of
  the image (four times) - the probe erases beyond the image's pages, so
  nothing the media keep survives a flash on this bench.
- **A page accepts a second program between erases when only more
  bits are cleared.** Letter c programs a page with a pattern in its
  left half and 0xFF in its right, then programs it again with the
  left half repeated and the right half now a pattern: no error, the
  whole page reads back exactly as asked. The chapter promises nothing
  of the kind, so the media keep the page as the cell - a smaller
  cell would rest on a behaviour the vendor does not state, and what
  repeated programming does to a page's retention is an endurance
  question no bench here has asked. The finding is what makes a
  smaller cell POSSIBLE; the decision to use one is the application's,
  with that caveat in front of it.
- **The heap on a page-celled media**: a 300-byte block allocated,
  appended byte by byte, sealed, found again after a re-mount at its
  address and length, byte-exact; the block landed at 0xDC00 (the
  placement rule from the top, below the map pair).
- **The journal in the attic**: a typed value saved and loaded, a
  load of the wrong width refused, sixteen 32-byte saves through a
  collection with the last value holding and the first surviving,
  `save_reserved()` landing in the cell every save left behind, a
  re-mount holding three live ids with nothing torn.
- **The refusals**: both locks read shut at boot and shut again after
  `lock()`; a misaligned page or sector address and a span that is not
  a page come back `refused` before any store; the media refuse an
  address below the storage floor and one in the other media's share.

## Not covered yet

Driver gaps, each with its reason:

- The option bytes (18.5: RDPR, the USER byte with the PD7/RST choice,
  IWDG_SW, the standby reset) and the write-protection units as verbs:
  the `brio fuses` work for this target, where a wrong write costs a
  chip erase and deserves its own session.
- The 32 KB block erase (the CH32V006's) and the whole-array erase: no
  user; the probe erases the array when it programs (the bench
  findings).
- The CH32V003's half-word program mode: the page is the write cell on
  both parts, and a two-byte cell for the journal - 256 saves a half in
  place of 8 - is a decision the journal's cost per save on the bench
  would make, with the part's endurance under partial-page writes
  measured first.
- The BOOT area and the boot-mode bits of STATR: the serial ISP path
  is the probe's business until a program needs to hand itself over.
- The flash interrupts (EOPIE, ERRIE): every operation here is polled
  and stalls the core anyway, so an interrupt would announce nothing
  the caller is not already waiting for.

Implemented but not bench-verified, each with what would measure it:

- Endurance and retention under the second-program behaviour of
  letter c: a long-running letter outside `z` that re-programs one
  page many times between erases and reads it back after a wait.
- The write-protection error path (WRPRTERR): every unit is
  unprotected on the bench part, so the bit was never seen set; the
  option-byte work is where a unit gets protected on purpose and the
  engine's refusal reads it.
- **The CH32V003's engine times**: its page figures were read on a
  counter that wrapped every millisecond, which no longer times them;
  the stopwatch letters `b`, `d` and `f` now use, judged against that
  part's datasheet (V1.6 table 3-14: 3.1 ms for a 64-byte page's
  program and its erase), would - on a CH32V003 board, which the desk
  has not got.
