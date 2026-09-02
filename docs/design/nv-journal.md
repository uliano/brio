# NvJournal - small values kept in flash

> **PROVISIONAL.** The journal, its entry format and its verbs are
> implemented and verified on the host and on the bench; what is missing
> is listed in "Not covered yet" at the end.

The target-independent design of the small-value store:
[`util/nv_journal.hpp`](../../brio/util/nv_journal.hpp) (the journal and
the `JournalPanic` reporter), over the same `FlashMedia` contract
[nv-heap.md](nv-heap.md) describes, with
[`util/crc.hpp`](../../brio/util/crc.hpp) underneath it and
[`host/sim_flash.hpp`](../../brio/host/sim_flash.hpp) as the media the
host tests run it on. The SAM C21 backend is `RwweeJournalZone` in
[`samc/nvm_flash.hpp`](../../brio/samc/nvm_flash.hpp), described in
[samc/nvm.md](../samc/nvm.md). Reference test suites: `test_nv_journal`
on the host, `test_samc_journal` on the bench.

## What it is for

[nv-heap.md](nv-heap.md) opens by naming two needs and one storage class
each: "a handful of small values, rewritten often - a calibration offset,
a mode, a counter - belong in the EEPROM", and "a table belongs somewhere
else". NvHeap is the second one.

This is the first one, on a part where the sentence's EEPROM does not
exist. An AVR Dx has a real byte-granular EEPROM and
[`util/nv_record.hpp`](../../brio/util/nv_record.hpp) speaks it. A SAM
C21 has none - what its data sheet calls "EEPROM emulation" is rows of
ordinary flash - and an STM32G0 has none either. Everything those parts
want to remember has to live in flash, which erases in kilobytes and
programs each cell once, and the whole difficulty is that a value
rewritten a thousand times must not cost a thousand erases of the same
row nor lose the previous value while the new one is going down.

NvJournal is exactly that, and nothing else. It is not a heap: there is
no address handed out and no block. It is not a log: there is no history,
only a current value per id.

## The two halves

The journal's region is the top `2 x half_pages` erase units of the
media, anchored to `Media::flash_end` - to the SILICON, not to the
linker, for the same reason the heap's map home is. It is split into two
HALVES that ping-pong wholesale.

Entries are APPENDED into the active half, one write cell at a time,
never rewritten. When an append plus the reserve no longer fits, a
COLLECTION copies the latest value of every live id into the freshly
erased other half and erases the old one.

That is the map pair's atomicity argument one size up, and the same three
properties fall out of it with no defensive code:

- **atomicity** - the old half rules until the new data is entirely down;
- **no rewriting** - every cell is programmed once between erases, which
  is the ECC discipline an STM32G0's double word requires;
- **wear sharing** - the two halves alternate by construction, and a save
  costs one program cell rather than an erase.

## Sequence numbers decide, CRC judges

Every entry carries a monotonically increasing 32-bit sequence number and
a CRC-16 over its own header and payload. Two rules follow from that and
they are the whole recovery logic:

- the current value of an id is the **highest-seq entry with a valid
  CRC**, wherever it lies;
- the **active half is the one holding the highest-seq valid entry** of
  all, and when both halves hold entries a collection was interrupted:
  the newer half is its destination, ids whose latest still lives in the
  other one are copied over, and then the other one is erased.

Those two resolve every state a power loss can leave, and none of them is
a special case in the code:

| Cut where | What is on the flash | What the rules do |
|-----------|----------------------|-------------------|
| mid-append | a torn entry that fails its CRC | it is stepped over; the previous value of that id still rules |
| mid-collection, before the source is erased | both halves, destination newer | the copy is resumed, then the source is erased |
| while erasing the source | destination has everything | nothing to copy; the erase is finished |
| while erasing the DESTINATION (a half is one or more erase units, so that erase is not one hardware operation either) | stale survivors there, older than the source | the source is the newer half, so it is the destination; every live id was copied into it at the previous collection, so again nothing to copy and the stale half is erased |

## The entry format

One entry is a header plus a payload, rounded up to whole write cells and
never spanning a half boundary. The header is little-endian throughout
and its offsets are `static_assert`ed constants that the host suite pins
byte for byte:

| Offset | Field | |
|--------|-------|--|
| 0 | magic | `'N','J'` |
| 2 | id | the application's name for the value |
| 3 | length | payload bytes, at most `max_payload` |
| 4 | seq | the sequence number, little-endian |
| 8 | crc | CRC-16/CCITT-FALSE over bytes 0..7 and then the payload |
| 10 | reserved | |
| 12 | payload | then 0xFF padding to a whole write cell |

The checksum covers everything that is not itself and not padding, which
is what makes a half-written entry recognizable rather than plausible.

## Mount is read-only

`mount()` walks both halves, builds the id index in RAM and reports what
it found - live ids, the active half, the highest sequence number, and
how many torn entries it stepped over. It costs no erase and no program:
booting is free, exactly as it is for the heap.

A torn tail is **reported, not treated as an error**: it is the atomicity
working. A collection left unfinished by a power loss is likewise NOTED
at mount and completed by the next ordinary `save()`, never by the boot
path - a boot must not be the thing that spends an erase.

The geometry guard is the heap's: a zone floor above the journal home
means the running image's own bytes are sitting where an entry would be
programmed, and the mount is REFUSED with a distinct status and no write
of any kind.

## The panic reserve

This is the design's centerpiece and the reason the journal is not just
"append until full".

An ordinary `save()` collects EARLY. It leaves the active half with room
for at least one more maximum-size entry in cells that are already
erased, and that room is a GUARANTEE checked by arithmetic rather than
hoped for: the geometry `static_assert` requires

    (max_ids + 2) x max_entry_cells <= half_cells

- `max_ids` maximum-size entries (the worst case a collection can leave),
plus the one being written, plus the reserve.

`save_reserved()` spends exactly that room: no collection, no erase, one
bounded polled program. That is what makes it legal from a panic handler,
where interrupts are masked for good, the kernel will never run again,
and an erase would be an unbounded wait on a supply that may be dying.

`JournalPanic` is the reporter built on it -
[`util/persistent_panic.hpp`](../../brio/util/persistent_panic.hpp)'s
shape over flash instead of over an EEPROM record. `report()` writes
through the reserve; `take()` runs at the next boot, where an erase is
affordable, and consumes the record by saving a cleared one through the
ORDINARY save - which also restores the reserve for the next failure. The
journal is reached through a REFERENCE template parameter, because a
journal is an object with a RAM index and the application owns it, where
an `NvStore` is static.

## The two spellings, and why they stay apart

`NvRecord` speaks a real EEPROM: byte-granular, rewrite in place, write
only the bytes that changed. `NvJournal` speaks flash: cells written once
between erases, a new entry per save, a collection when the half fills.
They solve the same PROBLEM and share nothing of the mechanism.

Unifying them under one concept is deliberately NOT done. A common
`save(id, span)` / `load(id, span)` facade would have to hide exactly the
things a caller has to know - what a write costs, whether it can block,
whether there is a bounded path safe from a panic handler - and brio does
not yet have a consumer that needs the same source to run over both. The
question is OPEN now that the STM32G0 runs the journal too, and it
stays a question until a real cross-target application asks it; until
then two spellings, each honest about its silicon, is the smaller lie.

## What a second target inherits

Nothing here is target-specific, so a new part gets the journal by
writing its own `FlashMedia` and nothing else - a claim that became a
measurement when the STM32G0 arrived: its geometry (2 KB erase unit, an
ECC-guarded 8-byte double word programmed once) was one of the three
the host suite swept BEFORE that silicon existed, next to the SAM
C21's RWWEE array (256/64) and an AVR Dx-shaped one (512/2, where a
single entry header spans six program units), and
`stm32g0/nvm_flash.hpp`'s journal zone in the part's second flash bank
mounted, saved, collected and kept its panic reserve with no change to
this header (docs/stm32g0/nvm.md). Its bank-2 attic is the third
place the read-while-write property was measured: a save from the
loop stalls nothing.

## Wear

A save costs one program cell. A collection costs one erase of every unit
of both halves plus one program cell per live id. So the erase budget is
spent once per (usable half capacity / average entry size) saves, and the
two halves wear evenly.

The number to know when choosing `max_ids` is that the usable room
between collections is `half_cells` minus the live entries minus the
reserve. At the maximum `max_ids` the geometry allows, that is one cell -
a collection at every second save. Sizing the region for twice the ids
actually stored is what buys the erase budget back.

## Not covered yet

Driver gaps - what the design does not do:

- **No delete verb.** An id that should mean "absent" says so in its own
  payload; a tombstone entry would cost the same cell and buy nothing.
  The consequence is that an id never stops being live and never stops
  being copied by a collection.
- **More distinct ids on the flash than `max_ids`.** The surplus is not
  indexed at mount and is dropped at the next collection. That is the
  right behaviour for a shrunken configuration and the wrong one for a
  typo; nothing warns.
- **No iteration in write order.** This is a set of values, not a log. A
  growing accumulator is a different structure with different invariants,
  the same ruling [nv-heap.md](nv-heap.md) makes.
- **No sequence-number wrap handling.** 32 bits at one save per
  millisecond is fifty days of continuous writing and far beyond any
  flash endurance budget, so the case is unreachable rather than handled.

Implemented but not bench-verified:

- **A power loss in the middle of a real flash write.** The sweep that
  proves atomicity at every program unit runs on the simulated media,
  where the power switch is exact. On silicon the only crash boundary
  exercised is a software reset - which is a clean one.
- **A torn ERASE on silicon.** The simulated power switch only opens
  between program units, so the states a half-erased half leaves are
  built by hand in the host suite and reasoned about here; no real part
  has been cut mid-erase.
