# NvHeap - blocks of flash that outlive the program

> **PROVISIONAL.** The allocator, its map format and its verbs are
> implemented and verified on the host and on the bench; what is missing
> is listed in "Not covered yet" at the end.

The target-independent design of the flash block allocator:
[`util/nv_heap.hpp`](../../lib/brio/src/util/nv_heap.hpp) (the allocator
and the `FlashMedia` contract), with
[`util/crc.hpp`](../../lib/brio/src/util/crc.hpp) underneath it and
[`host/sim_flash.hpp`](../../lib/brio/src/host/sim_flash.hpp) as the
media the host tests run it on. The AVR DA/DB backend is
[`avrdx/nvm_flash.hpp`](../../lib/brio/src/avrdx/nvm_flash.hpp),
described in [avrdx/nvm.md](../avrdx/nvm.md). Reference test suites:
`test_nv_heap` on the host, `test_avr_nvheap` on the bench.

## What it is for

An MCU program that has to remember something across a power cycle has
two very different needs, and one storage class each.

A handful of small values, rewritten often - a calibration offset, a
mode, a counter - belong in the EEPROM: byte-granular, 100k cycles, and
a record layout that writes only the bytes that changed
([nv_record](../../lib/brio/src/util/nv_record.hpp)).

A table belongs somewhere else. A thermocouple linearization, a font, a
captured waveform, a lookup grid: kilobytes, written once in a while,
read constantly. The only memory of that size is the flash the image
does not occupy, and the only thing standing between an application and
that flash is bookkeeping - which pages are free, where a block starts,
how long it is, and whether what is there is still what was written.
NvHeap is exactly that bookkeeping, and nothing else.

## The `FlashMedia` contract

Everything target-specific is behind one concept, whose full member list
lives in the header. Four things about it are design decisions rather
than plumbing:

- **Two granularities, not one.** `erase_size` is the erase unit,
  `write_cell` the program unit, and they are separate constants because
  the word "page" is vendor-contested: 512 and 2 on an AVR Dx, 2048 and
  8 on an STM32G0, rows and pages again on a SAM C21. Code that assumes
  one number is code that is wrong on the second target.
- **A program unit is written ONCE between erases.** The contract says
  so, and the allocator obeys it everywhere - which is what keeps
  ECC-guarded flash (the G0's double-word) happy without a special case.
- **`zones()` is a RUNTIME call.** The free flash of a linked image is
  not a compile-time fact: its bounds are linker symbols and they move
  with every build. The media reports bands of `{ceiling, floor}` and
  the heap rounds them inwards to whole erase units.
- **`build_id()` is diagnostic.** It is recorded in every map version
  and nothing decides on it. Validity is the checksum's business.

## The map pair

All the bookkeeping is one structure, held in the last `map_pages` erase
units of the part - an address anchored to the SILICON, not to the
linker, so no image can be built over it by accident.

A map version is one whole-unit write: a header (magic, format version,
entry count, sequence number, build id), `max_blocks` fixed-size entries
(record id, first page, size in pages, payload length, payload CRC-16,
flags) and a CRC-16 over all of it. The layout is little-endian and
`static_assert`ed to fit one erase unit - that assertion is what makes
`max_blocks` a declared limit rather than a hope.

Versions ping-pong. Every mutation erases the OLDEST page of the
rotation and writes the next version there with `seq + 1`; the current
map is the highest-`seq` version whose magic and CRC verify. Three
properties fall out of that, and none of them needs defensive code:

- **atomicity** - the old version rules the heap until the new one is
  entirely down; a version torn by a power loss simply fails its CRC;
- **no rewriting** - each version is written once into a freshly erased
  page, which is the ECC discipline above and also why no "tombstone"
  or bit-clearing verb exists anywhere in the design;
- **wear sharing** - the rotation spreads the map's own erase cycles
  over `map_pages` pages, so the map endures `map_pages` times the
  part's per-page budget in MUTATIONS (not in boots: mounting writes
  nothing).

Two pages is the enforced minimum. With one, the erase that precedes a
version would take the only copy of the map down - not a smaller heap,
but a heap without the property the design exists for.

## Blocks are pure payload

A block is `size_pages` erase units of application bytes: no header, no
length field, no chain, no in-band markers. `find()` returns an address
and a length, and the reader is free to walk it however it likes -
including with the target's own reader, which is the point on a part
where flash reads are a special instruction.

Everything that describes a block lives in the map entry. Allocation
granularity is the erase unit, because a neighbour's erase would
otherwise take a block down with it.

## Survival-aware mount

`mount()` is READ-ONLY - a boot costs no erase cycle. It picks the
current version, then verifies EVERY listed block by reading its payload
and checking the recorded CRC:

- what verifies is served by `find()`;
- what does not is REPORTED as lost and disappears from the next
  published version, which frees its pages the moment the map knows.

The application therefore learns at boot exactly which of its tables
came through and which it must recreate - and if recreating one then
fails for space, it learns that too. Truths reported, never hidden.

Validity is judged by the checksum, never by which build wrote the
block. A table whose MEANING changes between firmware versions is the
application's problem and it has the tools for it: a new record id, or
its own version byte inside the payload.

**The map-home guard.** The linker knows nothing about the map pages, so
nothing stops an image from growing its read-only data up into them.
`mount()` refuses - with a distinct status and no write of any kind - if
a zone floor has climbed into the map home, because publishing a version
would then erase a page of the running image's own constants. A refused
heap serves nothing and mutates nothing.

## Placement: top-down, most clearance wins

A new block goes as HIGH as it fits: in each zone the TOPMOST gap among
the live blocks that accommodates it, and then the zone where the placed
block ends up FARTHEST from its own floor.

No growth model is assumed. Whether the code or the read-only data grows
next is a property of the application, not of the allocator, so the rule
measures the free space that is actually there and lets the geometry
decide. A block whose pages were freed (a loss, a supersede) leaves a
gap that the next allocation reuses immediately, and with at most
`max_blocks` live obstacles the search is a handful of comparisons, not
fragmentation management.

## The verbs

`mount`, `find`, `alloc`, `append`, `seal`, `rewrite` - the header is
the reference for what each takes and returns. What matters at this
level:

- **`alloc(id, len)` reserves and ERASES**, then hands back a writer
  handle. Nothing about the heap changes yet: a block of the same id
  stays live and findable, and a power loss before the seal leaves the
  heap exactly as it was.
- **`append(bytes)` streams.** The handle buffers whatever does not fill
  a program unit, so the caller chunks the payload however it likes
  without knowing the media's geometry. The running checksum covers
  exactly the bytes appended - the padding of the last unit is not part
  of the payload.
- **`seal()` is the commit point**: it flushes the partial unit and
  publishes a new map version listing the survivors, minus any older
  block of the same id, plus this one.
- **`rewrite(id)` refills a live block IN PLACE** - same address, same
  reservation. Its pages are erased at once, so from that moment until
  the seal the map's recorded checksum no longer describes what is on
  the flash: a power loss there is a LOSS, and the next mount reports it
  as one. That is the honest cost of rewriting without a second copy; an
  application that cannot afford it allocates two ids and alternates.
- **There is NO free().** A block is superseded by allocating its id
  again, and a block whose payload no longer verifies frees itself.
  Reclaiming space is therefore always someone's deliberate act or a
  detected loss - never a stale pointer's.

## What the persistence rests on, honestly

A heap survives a reboot because flash is flash. It survives a REFLASH
only because of a programming CONVENTION: the tool writes the image's
own pages and leaves the rest of the part alone. That is a property of
how the bench flashes (see [bench.md](../bench.md)), not of the silicon
- flash has no EESAVE twin, and a chip erase takes everything, map
included. Nothing in the design hides this: a wiped heap mounts as an
empty one, and the application is told.

## Static limits

Everything is declared and nothing grows:

| Parameter | What it costs | Checked by |
|-----------|---------------|------------|
| `max_blocks` | the RAM index and the map's entry table | `static_assert` against one erase unit |
| `map_pages` | flash at the top of the part, and the map's wear budget | `static_assert` (>= 2) |
| the media's geometry | - | `static_assert` (flash ends on an erase unit, erase unit is whole program units) |

The RAM cost is two entry tables (the live index and the reservations)
plus the mount report; the flash cost is `map_pages` erase units plus
what the application stores.

## Not covered yet

Driver gaps - what the design does not do:

- **No ring or log structure.** A growing accumulator (a measurement
  log) is a different structure with different invariants; it waits for
  a real user.
- **No wear levelling of payload pages.** A block is written where it
  was placed. Data rewritten often does not belong in flash at all -
  that is what the EEPROM and `nv_record` are for.
- **No per-image lifetime flag.** The entry's `flags` byte is reserved
  for it: a table that must NOT outlive its build is a policy nobody has
  asked for yet.
- **No zone hint at `alloc`.** The placement rule is the only policy;
  letting a caller pin a block to a zone is a one-line extension when
  something needs it.

Implemented but not bench verified:

- **A power loss in the middle of a real flash write.** The sweep that
  proves atomicity at every program unit runs on the simulated media,
  where the power switch is exact. On silicon the same code has been
  crashed only by a software reset, which is a clean boundary.
- **Power loss on a second silicon.** The heap now runs on two real
  targets - the AVR DA/DB main array (512/2) and the SAM C21 RWWEE array
  (256/64, `samc/nvm_flash.hpp`), where it mounted, allocated, sealed and
  re-found a block with no change to this header - so the
  two-granularity contract has been exercised by silicon and not only by
  the host's two simulated geometries. What the second target has not
  added is a power-cut sweep: on both, the only crash boundary exercised
  in hardware is a software reset.
