# NvHeap: blocks of flash that outlive the program

This page decides how a program keeps a table of kilobytes (a
linearization, a font, a captured waveform) in the flash its image does
not occupy, so that the table survives a power cycle and a reflash and
the program learns at boot which tables came through. It is written
for an embedded programmer who has stored data in flash by hand and
wants the bookkeeping (which pages are free, where a block starts, how
long it is, whether it is still intact) done once and proven. The
allocator is `util/nv_heap.hpp`, with the `FlashMedia` contract a
target satisfies; `util/crc.hpp` is underneath it, and
`host/sim_flash.hpp` is the media the host tests run it on. Reference
test suites: `test_nv_heap` on the host, `test_avr_nvheap` on the
bench.

The smallest use: `mount()` once at boot, `find(id)` for a table that
came through, and `alloc(id, len)`, `append(bytes)` and `seal()` to
write a new one. The prerequisite that decides correctness: **nothing
is published until `seal()`**, and a `rewrite(id)` erases the old
copy at once, so a power loss between a rewrite and its seal is a
loss the next mount reports.

## What it is for

A microcontroller program that has to remember something across a
power cycle has two very different needs, and one storage class each.

A handful of small values, rewritten often (a calibration offset, a
mode, a counter), belong in the EEPROM where the part has one:
byte-granular, 100k cycles, and a record layout that writes only the
bytes that changed (`util/nv_record.hpp`), a run per operation where
the store's silicon writes one ([nv-journal.md](nv-journal.md), "The
EEPROM spelling's store"). Where the part has no EEPROM, the same need
is served by the journal of that page.

A table belongs somewhere else. A thermocouple linearization, a font,
a captured waveform, a lookup grid: kilobytes, written once in a
while, read constantly. The only memory of that size is the flash the
image does not occupy, and the only thing standing between an
application and that flash is bookkeeping: which pages are free, where
a block starts, how long it is, and whether what is there is still
what was written. NvHeap is exactly that bookkeeping, and nothing
else.

## The `FlashMedia` contract

Everything target-specific is behind one concept, whose full member
list lives in the header. Four things about it are design decisions
rather than plumbing:

- **Two granularities, not one.** `erase_size` is the erase unit,
  `write_cell` the program unit, and they are separate constants
  because the word "page" means different things to different vendors:
  512 and 2 on an AVR Dx, 2048 and 8 on an STM32G0, rows and pages
  again on a SAM C21. Code that assumes one number is code that is
  wrong on the next target.
- **A program unit is written once between erases.** The contract says
  so, and the allocator obeys it everywhere, which is what keeps
  ECC-guarded flash (the G0's double word) happy without a special
  case.
- **`zones()` is a run-time call.** The free flash of a linked image is
  not a compile-time fact: its bounds are linker symbols and they move
  with every build. The media reports bands of `{ceiling, floor}` and
  the heap rounds them inwards to whole erase units.
- **`build_id()` is diagnostic.** It is recorded in every map version
  and nothing decides on it. Validity is the checksum's business.

### Realizations

Common to all: the concept, and `NvHeap<Media, max_blocks, map_pages>`
over it unchanged; the same header mounted, allocated, sealed and
re-found a block on every media below. What differs is the array each
family lends, which is where the two granularities come from.

| stratum | realization | beyond the contract |
|---|---|---|
| avrxt, the AVR DA/DB | `NvmFlash` over the main array (`avrxt/nvm_flash.hpp`) | 512 / 2; the zones come from the linker's own symbols (the free flash of this image, with the BOOT section as floor), so a reflash can land on a block and the bench tool warns before it does; a write stalls the CPU for the page's whole erase |
| avrxt, the AVR EA | none: `NvmFlash` refuses the part at compile time | declined: a 64 KB part with a 512-byte EEPROM keeps its small values there (`NvRecord` over `EepromStore`), and its flash self-programming is not written ([the AVR NVM page](../avrxt/nvm.md)) |
| samc21 | `RwweeFlash` over the RWWEE array (`samc21/nvm_flash.hpp`), rows 0..27 of the one partition | 256 / 64; the zone is a constant (no linker section reaches that array); writing it stalls nothing (read-while-write), and it is four times as durable as the main array |
| stm32g0 | `MainFlash` over bank 2 (`stm32g0/nvm_flash.hpp`) | 2048 / 8, the ECC-guarded double word programmed once; the storage attic is the whole second bank, the program in bank 1 (erratum 2.2.10's own reason), so a write stalls nothing, which makes the single-bank parts a design question and not a port |
| stm32f4 | none: the engine is written (`stm32f4/flash.hpp`) and no `FlashMedia` over it | the sectors are uneven and the small ones sit at the bottom of the bank under the vector table, so a fixed zone means a partitioned linker script in every image for a 128 Kbyte grain sixty-four times the STM32G0's page; and on a single-bank part every erase freezes the core for its whole duration (189 ms for 16 Kbytes, 978 for 128, measured), interrupts included. What that costs against what a program would keep there is the NV stack's own review, not a port |
| ch32v00x | `MainFlash` over the top of the one array (`ch32v00x/nvm_flash.hpp`), 16 KB above the image's 40 KB | 256 / 256: the family programs by whole pages and nothing smaller, so the cell is the page and an append of any length costs one; the zone is a constant of the partition (the linker's `rom` stops at 40 KB, so no reflash can reach it); a page program stalls the core for its whole duration, about a millisecond |
| ch32vx03 | `MainFlash<Clock>` over the top of the zero-wait window (`ch32vx03/nvm_flash.hpp`), its last 4 KB, and nothing mounted on it | 256 / 256: the cell is the page, as on the sister family, and the zone is a constant of the partition (all thirteen linker scripts stop 4 KB short of the window, which is one write-protection unit; the non-zero-wait tail above it is nobody's, though both of the engine's methods write it: a word of it costs four times a window word to read and its half-word cells do not take a second pass, measured on the CH32V303VC). Three things are this silicon's own. The erased pattern is 0xE339E339 and not all ones (RM 32.5.4, said four times, measured), so `NvJournal`, which finds the end of a half by looking for a cell of 0xFF bytes, could not stand here unchanged; the heap, which judges a map version by its magic and its CRC, would not care. The clock is part of the medium's type, because an erase or a program above 120 MHz wants HCLK halved around it (32.1) while `program()` and `erase()` take an address and nothing else, so the rate arrives with the type and the refusal is a compile error under a static clock and a false under a dynamic one. And a write is a wait and not a stall: the core goes on executing out of the array while the engine works (measured: the kernel's tick, which counts handler runs, advanced by the erase's own nine to ten milliseconds on the CH32V203C8 and seventeen on the CH32V303VC), a page erase costing 9.8 ms and a page program 1.5 against a half-word's 2.6 on the CH32V203C8, and every erase 16.85 ms whatever its grain on the CH32V303VC. Neither heap nor journal is instantiated on this family, by decision: what a program gets is the band and its bounds |
| ch32x035 | none | no `FlashMedia`: the engine is not written (the option bytes are decoded read-only) and all seven linker scripts give the image the whole 62 KB, so a zone would first be a `rom` that stops short, the NV review's question ([the CH32X035 page](../ch32x035/README.md)) |
| rp2040 | `QspiFlash` over the top of the external quad-SPI chip (`rp2040/nvm_flash.hpp`), 48 KB below the journal's attic | 4096 / 256: the bootrom's program takes whole pages, so the cell is the page though the chip programs a byte (a second program between erases lands); the zone is a constant of the partition (the linker's `flash` stops 64 KB short of the chip); every erase or program is a window with the flash disconnected (code in SRAM, interrupts masked, the ticker's ticks lost), a sector erase about 5 ms and a page about 1.3 ms on the WeAct board's chip |
| rp2350 | `QspiFlash` over the top of the external quad-SPI chip (`rp2350/nvm_flash.hpp`), the top 64 kB, and nothing mounted on it | 4096 / 256: the cell is the page as on the RP2040, the bootrom's program function taking whole pages though the chip programs a byte; the zone is a constant of the partition (the linker's `flash` stops 64 kB short of the chip and `__brio_flash_end` is read back as the proof, a script edited past the line closing the medium rather than letting the two overlap); every erase or program is a window with the memory interface disconnected (code in SRAM, this core's interrupts masked, the way back into execute-in-place the bootrom's own function out of boot RAM where the RP2040 had a second stage), and the other core must not fetch from flash meanwhile, which nothing here can enforce. Neither heap nor journal is instantiated, by decision and not by omission: what a program gets is the band and its bounds, until the NV stack's own review says which programs need what |
| host | `SimFlash<Erase, Cell>` (`host/sim_flash.hpp`) | any geometry, a power switch that opens between program units, a simulated reflash, wear counters: what the power-cut sweeps run on |

## The map pair

All the bookkeeping is one structure, held in the last `map_pages`
erase units of the part, an address anchored to the silicon, not to
the linker, so no image can be built over it by accident.

A map version is one whole-unit write: a header (magic, format
version, entry count, sequence number, build id), `max_blocks`
fixed-size entries (record id, first page, size in pages, payload
length, payload CRC-16, flags) and a CRC-16 over all of it. The layout
is little-endian and `static_assert`ed to fit one erase unit; that
assertion is what makes `max_blocks` a declared limit rather than a
hope.

Versions alternate between pages. Every mutation erases the oldest
page of the rotation and writes the next version there with `seq + 1`;
the current map is the highest-`seq` version whose magic and CRC
verify. Three properties fall out of that, and none of them needs
defensive code:

- **atomicity**: the old version rules the heap until the new one is
  entirely down; a version torn by a power loss simply fails its CRC;
- **no rewriting**: each version is written once into a freshly erased
  page, which is the ECC discipline above and also why no "tombstone"
  or bit-clearing operation exists anywhere in the design;
- **wear sharing**: the rotation spreads the map's own erase cycles
  over `map_pages` pages, so the map endures `map_pages` times the
  part's per-page budget in mutations (not in boots: mounting writes
  nothing).

Two pages is the enforced minimum. With one, the erase that precedes a
version would take the only copy of the map down: not a smaller heap,
but a heap without the property the design exists for.

## Blocks are pure payload

A block is `size_pages` erase units of application bytes: no header,
no length field, no chain, no in-band markers. `find()` returns an
address and a length, and the reader is free to walk it however it
likes, including with the target's own reader, which is the point on a
part where flash reads are a special instruction.

Everything that describes a block lives in the map entry. Allocation
granularity is the erase unit, because a neighbour's erase would
otherwise take a block down with it.

## Survival-aware mount

`mount()` is read-only: a boot costs no erase cycle. It picks the
current version, then verifies every listed block by reading its
payload and checking the recorded CRC:

- what verifies is served by `find()`;
- what does not is reported as lost and disappears from the next
  published version, which frees its pages the moment the map knows.

The application therefore learns at boot exactly which of its tables
came through and which it must recreate, and if recreating one then
fails for space, it learns that too. Truths reported, never hidden.

Validity is judged by the checksum, never by which build wrote the
block. A table whose meaning changes between firmware versions is the
application's problem and it has the tools for it: a new record id, or
its own version byte inside the payload.

**The map-home guard.** The linker knows nothing about the map pages,
so nothing stops an image from growing its read-only data up into
them. `mount()` refuses, with a distinct status and no write of any
kind, if a zone floor has climbed into the map home, because
publishing a version would then erase a page of the running image's
own constants. A refused heap serves nothing and mutates nothing.

## Placement: top-down, most clearance wins

A new block goes as high as it fits: in each zone the topmost gap
among the live blocks that accommodates it, and then the zone where
the placed block ends up farthest from its own floor.

No growth model is assumed. Whether the code or the read-only data
grows next is a property of the application, not of the allocator, so
the rule measures the free space that is actually there and lets the
geometry decide. A block whose pages were freed (a loss, a supersede)
leaves a gap that the next allocation reuses immediately, and with at
most `max_blocks` live obstacles the search is a handful of
comparisons, not fragmentation management.

## The operations

`mount`, `find`, `alloc`, `append`, `seal`, `rewrite`: the header is the
reference for what each takes and returns. What matters at this
level:

- **`alloc(id, len)` reserves and erases**, then hands back a writer
  handle. Nothing about the heap changes yet: a block of the same id
  stays live and findable, and a power loss before the seal leaves the
  heap exactly as it was.
- **`append(bytes)` streams.** The handle buffers whatever does not
  fill a program unit, so the caller chunks the payload however it
  likes without knowing the media's geometry. The running checksum
  covers exactly the bytes appended; the padding of the last unit is
  not part of the payload.
- **`seal()` is the commit point.** It flushes the partial unit and
  publishes a new map version listing the survivors, minus any older
  block of the same id, plus this one.
- **`rewrite(id)` refills a live block in place**, same address, same
  reservation. Its pages are erased at once, so from that moment until
  the seal the map's recorded checksum no longer describes what is on
  the flash: a power loss there is a loss, and the next mount reports
  it as one. That is the honest cost of rewriting without a second
  copy; an application that cannot afford it allocates two ids and
  alternates.
- **There is no `free()`.** A block is superseded by allocating its id
  again, and a block whose payload no longer verifies frees itself.
  Reclaiming space is therefore always someone's deliberate act or a
  detected loss, never a stale pointer's.

## What the persistence rests on

A heap survives a reboot because flash is flash. It survives a reflash
only because of a programming convention: the tool writes the image's
own pages and leaves the rest of the part alone. That is a property of
how the bench flashes ([the Atmel-ICE page](../probes/atmel-ice.md)),
not of the silicon: flash has no EESAVE twin, and a chip erase takes
everything, map included. Nothing in the design hides this: a wiped
heap mounts as an empty one, and the application is told.

## Static limits

Everything is declared and nothing grows:

| Parameter | What it costs | Checked by |
|-----------|---------------|------------|
| `max_blocks` | the RAM index and the map's entry table | `static_assert` against one erase unit |
| `map_pages` | flash at the top of the part, and the map's wear budget | `static_assert` (at least 2) |
| the media's geometry | - | `static_assert` (flash ends on an erase unit, erase unit is whole program units) |

The RAM cost is two entry tables (the live index and the reservations)
plus the mount report; the flash cost is `map_pages` erase units plus
what the application stores.

## Not covered yet

Driver gaps, what the design does not do:

- **No ring or log structure.** A growing accumulator (a measurement
  log) is a different structure with different invariants; it waits
  for a real user.
- **No wear levelling of payload pages.** A block is written where it
  was placed. Data rewritten often does not belong in flash at all;
  that is what the EEPROM, `nv_record` and the journal are for.
- **No per-image lifetime flag.** The entry's `flags` byte is reserved
  for it: a table that must not outlive its build is a policy nobody
  has asked for yet.
- **No zone hint at `alloc`.** The placement rule is the only policy;
  letting a caller pin a block to a zone is a one-line extension when
  something needs it.

Implemented but not bench-verified:

- **A power loss in the middle of a real flash write.** The sweep that
  proves atomicity at every program unit runs on the simulated media,
  where the power switch is exact. On silicon the heap has mounted,
  allocated, sealed and re-found a block on the AVR DA/DB main array
  (512/2), the SAM C21 RWWEE array (256/64) and the STM32G0's second
  flash bank (2048/8) with no change to the header, so the
  two-granularity contract has been exercised by silicon and not only
  by the host's simulated geometries; but on all of them the only
  crash boundary exercised in hardware is a software reset, which is a
  clean one. What would measure it: a power cut staged at every
  program unit of a seal on one of those boards, with the supply
  switched by the bench.
