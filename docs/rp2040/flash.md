# Flash (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 2.6.3 (the
XIP cache, its four aliases and its registers), 2.8.1 (the flash
second stage and its checksum), 2.8.3 (the bootrom's table and the
flash access functions, 2.8.3.1.3, with the sequence a caller owes
them), 4.10 (the SSI the flash hangs off, the direct-register access
of 4.10.5), 2.19.6.3 (the QSPI pads' overrides); the two chips on the
bench (a Winbond W25Q16JV, a Zetta ZD25Q16) for the commands and their
timings; util/nv_heap.hpp and util/nv_journal.hpp for the FlashMedia
contract. The drivers: `brio/rp2040/flash.hpp` (`Flash` the engine and
the raw command, `Xip` the cache) and `brio/rp2040/nvm_flash.hpp`
(`QspiFlashPartition` and the two FlashMedia `QspiFlash` and
`QspiFlashJournalZone`), with the linker script `rp2040/ld/rp2040_2m.ld`
(the storage floor and the RAM code section). The reference suite:
`test_rp2040_flash`, wireless.

## What the silicon does

The chip has no flash of its own. It executes in place out of an
external quad-SPI chip through a 16 KB two-way cache at 0x1000_0000
and three more aliases of the same window (a no-allocate one, a
no-cache one, and one that neither hits nor fills), the serial
interface (a Synopsys SSI) set up for the chip by the 256-byte second
stage the bootrom loads, checks (a CRC-32 in its last word) and runs
first. Erasing and programming are the bootrom's business: mask ROM
carries a table of functions found by two-letter codes - connect the
pads, leave execute-in-place, erase a range, program a range, flush
the cache, and a slow command-mode re-entry - and the sequence a
caller owes them (connect, exit, the operation, flush, re-enter).
WHILE THE OPERATION RUNS THERE IS NO FLASH: the SSI is out of its
execute-in-place mode, so the caller, everything it calls and every
handler that could run must be somewhere else, in SRAM or in ROM. The
chip erases by 4 KB sectors (20h) and larger blocks (D8h, 64 KB),
programs by 256-byte pages, and a program only clears bits.

Three facts measured that the chapter does not state: the second
stage RETURNS to a caller whose link register is not zero (it vectors
into flash only when it is), so a copy of it in SRAM is the way back
into the fast quad-I/O mode the board booted in, whatever chip it
carries; a page programmed a second time between erases holds the AND
of the two programs, so the page is the cell by the bootrom's program
and not by the chip; and a masked window of five milliseconds costs
the ticker all its ticks but one - SysTick pends once, however long
the mask stays.

## Types and verbs

- `Flash`: the geometry the build states (`size_bytes` from
  `BRIO_RP2040_FLASH_KB`, `sector_size` 4096, `block_size` 65536,
  `page_size` 256, `window`), `init()` (the table looked up, the six
  functions found, the stage copied out and its CRC checked; every
  operation calls it itself), `ready`, `rom_version`, `stage_crc` /
  `stage_crc_stored`; `address(offset)` (cached) and
  `uncached_address(offset)` (the flash alone), `in_window(p)`;
  `read(addr, dst)` through the cache; `erase(addr, bytes)` by whole
  sectors, the 64 KB command where the run is aligned to it,
  `erase_sector(addr)`; `program(addr, src)` by whole pages, the
  source in SRAM; `command(tx, rx)` (a raw exchange with the chip
  select forced low, at most `max_command` bytes each way) and over
  it `jedec_id()` -> `FlashJedecId` (manufacturer, type, capacity;
  `bytes()`), `unique_id()` (4Bh, eight bytes), `status_register()`
  (05h). Every operation is one window: interrupts masked, the code
  in `.ram_text`, the stage re-entered on the way back.
- `Xip`: `enabled` / `enable`, `power_down` / `powered_down`,
  `fault_on_bad_write` / `faults_on_bad_write`, `flush` (waits),
  `flush_ready`, `hits` / `accesses` / `reset_counters`.
- `QspiFlashPartition`: the constant lines - `storage_base` (the top
  64 KB of the chip, where the linker's `flash` region stops),
  `journal_base` (the top 16 KB), `heap_end`, `storage_end`, the
  sector counts - and `geometry_matches_silicon()` against the
  linker's `__brio_flash_end`.
- `QspiFlash` (the heap's 48 KB, twelve sectors) and
  `QspiFlashJournalZone` (the attic, four sectors): the FlashMedia
  contract with `erase_size` 4096 and `write_cell` 256, the bounds
  checked before the bootrom is asked, `build_id()` from the link's
  defsym, a zone that refuses when the linker script has grown past
  the floor.

## How to use it

```cpp
brio::NvJournal<brio::QspiFlashJournalZone, 6, 32, 2> journal;   // two sectors a half, 32 cells
journal.mount();
journal.save<Calibration>(1, cal);                                // one page program, the window under 2 ms
const auto back = journal.load<Calibration>(1);

brio::NvHeap<brio::QspiFlash, 8, 2> heap;
heap.mount();
if (auto w = heap.alloc(0x0301, 300)) { w->append(bytes); w->seal(); }

if (const auto id = brio::Flash::unique_id()) { show(*id); }      // the board's identity
```

The window is the application's to keep clear: no handler runs while
it is open (the ticker misses its ticks, a byte transport holds its
bytes in the FIFO), the other core must not fetch from flash, no DMA
channel may read the window. A sector erase is the longest window an
ordinary save opens, when the journal collects; `save_reserved()`
never erases.

## Bench findings

The reference suite is `test_rp2040_flash`, green on the Pico (a
Winbond W25Q16JV) and on the WeAct board (a Zetta ZD25Q16), every
letter wireless, the system timer around every window. The two chips
answer the same commands with the same result and differ in nothing
but their timings, which is the point of the bootrom's engine.

- THE ENGINE: the bootrom's table is found (version 1) with all six
  functions; the stage's CRC computed as 2.8.1 says is the word stored
  (0xD58F0B07 for the Winbond-class stage); 9Fh answers EF 40 15 on
  the Pico and BA 60 15 on the WeAct board - Winbond and Zetta, 2 MB,
  the build's size -, 4Bh a unique id the same on every read, 05h a
  status register at zero; XIP serves after every
  exchange; the linker stops at the storage floor, the zones are the
  48 KB and the 16 KB of the partition.
- ONE SECTOR, ONE PAGE: a sector erase takes 29 ms on the Winbond and
  5.2 ms on the Zetta, a page program 0.6 ms and 1.3 ms (the window
  included; the status register reads idle when the window closes); the page reads back exact through the
  cache and past it, the rest of the sector untouched; the refusals - a
  misaligned address, a short span, a source in the window, a page
  past the chip, a run of no sectors or past the end, an exchange of
  unequal lengths or of more than 64 bytes, and the media's bounds
  (below the floor, in the attic, below the attic).
- A SECOND PROGRAM between erases lands - the right half of a page
  written after the left, the page exact; a third asking bits back to
  one leaves the AND.
- THE WINDOW AND THE CACHE: across a 5.2 ms sector erase the ticker
  counts one tick; one sector read word by word costs 5121 accesses,
  4092 hits cold and 5114 warm, and the first word misses again after
  a flush; a page read through the cache and then programmed reads
  its new bytes through the cache (the operation's own flush).
- THE HEAP mounts empty on virgin flash, allocates a 300-byte block
  in its share (two cells), re-mounts with it surviving and reads it
  back exact.
- THE JOURNAL mounts in the attic (32 cells a half), saves and loads a
  typed value, refuses a load of the wrong width, takes forty saves
  through a collection with the calibration surviving,
  `save_reserved()` in 1.4 ms with no erase, and re-mounts with three
  live values and nothing torn.
- THE WIPE (outside the all-key): the 64 KB partition erased in one
  run through the 64 KB block command - 226 ms on the Winbond, 5.2 to
  5.5 ms on the Zetta, whose block costs what its sector costs - the
  chip idle when the window closes.

## Not covered yet

Driver gaps, each with its reason:

- A lockout of the other core and of the DMA for the window: nothing
  here parks core 1 or checks the channels; the application keeps
  them off the flash (the SDK's `flash_safe_execute` is the shape a
  portable one would take, born with the first program that writes
  flash while core 1 runs).
- The SFDP table (5Ah) and the chip's other registers (the second and
  third status registers, the quad-enable bit): `command()` carries
  any of them, the stage already sets what execute-in-place needs;
  the first user names them.
- A cell smaller than the page: the chip programs a byte at a time
  (measured), the bootrom's function does not; a program of one's own
  over the SSI would give the journal a smaller cell, a design
  question of util/nv_journal.hpp's geometry before a port.
- The XIP streaming FIFO (2.6.3.3) and the cache as SRAM (the window
  with the cache disabled): no program here wants either.
- Flash sizes other than 2 MB: the linker script and the partition
  are drawn for the chips on the bench; a 16 MB board wants its own
  script and preset.

Implemented but not bench-verified, each with what would measure it:

- `Xip::power_down` and `fault_on_bad_write`: a write to a non-cached
  alias with the fault armed, caught by the HardFault handler.
- An erase that spans the 64 KB block boundary inside the partition:
  the wipe erases one aligned block; a run from the middle of one
  block into the next would take the sector command on its ends.
