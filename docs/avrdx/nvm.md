# NVMCTRL - the nonvolatile memories

> **PROVISIONAL.** The chapter's register description, both errata
> documents and the whole command set are implemented and bench
> verified; what is missing is listed in "Not covered yet" at the end.

Documents of record: data sheet **DS40002247B** chapter 11 (NVMCTRL),
chapter 15 (CPUINT, for the vector-table bit) and section 39.8 (the
memory programming times); errata **DS80000915F** 2.7.1 / 2.7.2 (AVR
DB) and **DS80000882C** 2.7.1 / 2.7.2 / 2.17.1 (AVR DA). Driver:
[`lib/brio/src/avrdx/nvm.hpp`](../../lib/brio/src/avrdx/nvm.hpp), with
the target-independent services over it in
[`util/nv_record.hpp`](../../lib/brio/src/util/nv_record.hpp),
[`util/nv_writer.hpp`](../../lib/brio/src/util/nv_writer.hpp),
[`util/persistent_panic.hpp`](../../lib/brio/src/util/persistent_panic.hpp)
and [`util/crc.hpp`](../../lib/brio/src/util/crc.hpp). Reference test
suites: `test_avr_nvm` on the bench, `test_nv_record` on the host.

## What the silicon does

### Four memories, four jobs

| Memory | Size | Erase / write unit | Endurance | Survives a chip erase |
|--------|------|--------------------|-----------|-----------------------|
| Flash | 128 KB | page (512 B) / word | 1000 cycles | no |
| EEPROM | 512 B | byte / byte | 100k cycles | only with the EESAVE fuse |
| User Row | 32 B | the whole row / byte | Flash class | always |
| Signature Row | 64 B | read only | - | always |

The Flash figure deserves a second look: 1000 cycles is NOT the 10k
of the older AVR families - the DB datasheet lowered it to 1k "based
on validation data" (table 39-7; no typical value is published). A
chip erase spends one of those cycles on every page at once, which is
why the bench reflashes page-selectively (avrdude's default on these
parts) and reserves the chip erase for `bench.py flash --erase` - the
three measured erase regimes are in docs/bench.md.

That table is the whole division of labour. Flash is for big,
re-provisionable payloads - a font, a table, anything the programmer
can put back. EEPROM with EESAVE set is for settings, which must
survive a reflash. The User Row is identity and provisioning: it
survives everything, and it can be written over UPDI even on a locked
device. The fuses are read-only to the CPU - only UPDI programs them
(11.3.1.5), so fuse geometry is a provisioning act and not something
firmware decides.

### Flash sections, and why the fuses matter to code

Two fuses cut the Flash into up to three sections (11.3.1.1, table
11-2): `BOOTSIZE` ends BOOT, `CODESIZE` ends APPCODE, and what is left
is APPDATA. Both count 512-byte blocks. The rule that makes them
matter is inter-section write protection: **code can never write the
section it is executing from**, and it can only write sections above
it - BOOT writes APPCODE and APPDATA, APPCODE writes APPDATA, APPDATA
writes nothing (table 11-1).

A brio image is linked at address 0, so all of its code is in BOOT.
Two consequences follow, and neither is optional:

- With the shipping default `BOOTSIZE = 0` the whole Flash is BOOT and
  **no software can write any Flash at all**. Writing Flash at run time
  starts with a fuse.
- The default vector-table location is the start of the APPLICATION
  section, not address 0 (15.5.1 IVSEL, and 11.3.1.1 note 1). The
  moment `BOOTSIZE` stops being 0 the hardware looks for vectors in
  empty Flash and the first interrupt of any kind jumps into 0xFFFF -
  which is a reset loop, not a crash. `CPUINT.CTRLA.IVSEL` must be set,
  and it is correct under every geometry because BOOT always starts at
  0.

Two further bits in `NVMCTRL.CTRLB` are one-way until a reset:
`APPCODEWP` and `APPDATAWP` refuse all further writes to their section,
and `BOOTRP` makes BOOT unreadable and unexecutable from outside it -
that bit can only be WRITTEN from BOOT code and takes effect only when
execution leaves the BOOT section.

### The two ways to reach the Flash, and the one this driver uses

The Flash is visible twice: as code space through LPM/SPM with a 24-bit
`RAMPZ:Z` address, and as a 32 KB WINDOW in data space whose position
`NVMCTRL.CTRLB.FLMAP` selects in blocks of 32 KB (11.3.2). This driver
uses **code space only** - ELPM to read, SPM to write. That is a
deliberate, load-bearing choice:

- DS80000882C 2.7.1 (AVR DA, every revision) says the inter-section
  protection check ignores FLMAP, mirroring BOOT into every mapped
  section, so a write through the window with `FLMAP != 0` can land
  somewhere else entirely. The errata's own work-around is "use only
  SPM to write and LPM to read". Following it by construction removes
  the item rather than leaving it to be remembered.
- Every verb takes a `uint32_t`, so the classic way of losing the top
  half of a 128 KB part - a 16-bit `&symbol` - cannot happen.

FLMAP is still exposed (`flmap`, `set_flmap`, `flmap_locked`,
`lock_flmap`), because an app may want the window for its own READS,
which is the half of the erratum that is explicitly fine, and because
the C runtime already uses it: gcc places `.rodata` in a Flash section
and writes FLMAP in `.init`.

### The command model

Every write and erase is two steps (11.3.2.3): a command is SELECTED in
`NVMCTRL.CTRLA` under CCP with the SPM key, and then CARRIED OUT by an
ordinary store into the memory array - SPM for the Flash array, an ST
to the mapped address for EEPROM and the User Row. Changing from one
command to another must pass through NOCMD or NOOP, or the controller
raises a collision error instead of obeying. `NVMCTRL.DATA` and
`NVMCTRL.ADDR` take no part in this: they are observers reporting the
last value and the last address the controller saw (11.5.6, 11.5.7).

`NVMCTRL.STATUS` carries the two busy flags and a sticky three-bit
ERROR field cleared by writing it to zero.

### Who stalls whom

A **Flash** erase or write HALTS THE CPU for its whole duration - about
10 ms for an erase, tens of microseconds for a word. Nothing runs,
interrupts included. An **EEPROM** write does not halt the CPU; the CPU
is halted only if it starts a second NVM access while one is running.
That difference is why an interrupt-paced EEPROM writer is possible and
an interrupt-paced Flash writer is not.

## Types and verbs

### Geometry and layout

`flash_size`, `flash_page_size`, `flash_section_block`, `eeprom_size`,
`eeprom_base`, `userrow_size`, `userrow_base` are constants taken from
the device header, never copied. `FlashSection` names the three
sections; `FlashRange` is a half-open byte range with `size`, `empty`
and `contains`.

`FlashLayout<boot_blocks, code_blocks>` is the fuse geometry as a
COMPILE-TIME claim: it computes `boot_end`, `appcode_end`,
`section_of()` and `writable()` as constant expressions, so the
compile-time verbs can refuse an illegal address, and `matches_fuses()`
cross-checks the claim against the silicon at boot. Nothing about the
real geometry is knowable at compile time, which is exactly why an
image has to state the one it is built for.

### `Nvm` - the controller

Static verbs, one instance: the same shape as `Reset` and `Watchdog`
(see [platform.md](platform.md)).

- **State**: `flash_busy`, `eeprom_busy`, `busy`, `error`,
  `clear_error`, and the bounded waits `wait_idle`, `wait_flash`,
  `wait_eeprom`. `NvmError` names the five codes the register can show.
- **Commands**: `select(NvmCommand)`, `selected()`, `clear_command()`.
  `NvmCommand` lists every command of 11.5.1 except the two that erase
  a whole memory - see "What is deliberately not here".
- **Mapping and protections**: `flmap`, `set_flmap`, `flmap_locked`,
  `lock_flmap`; `protect_appcode`, `protect_appdata`,
  `protect_boot_read` and their readbacks, all one-way until a reset.
- **Vectors**: `vectors_in_boot()` and `vectors_in_boot_armed()`.
- **Geometry from the fuses**: `boot_end`, `appcode_end`,
  `section_of`, and `writable(begin, end)` - the one that answers "may
  code in BOOT write every byte of this range, right now".
- **Flash**: `flash_read` (byte, word and block, all through ELPM),
  `flash_blank`, `erase(addr, FlashErase)`, `write_word`,
  `write_block`, and `erase_ignoring_protection` (below).
- **EEPROM**: `eeprom_read` (byte and block), `eeprom_write` (EEWR, no
  erase), `eeprom_erase_write` (EEERWR), `eeprom_erase(offset,
  EepromErase)`, `eeprom_write_block`, `eeprom_poke` (the store half of
  the command model), and the EEREADY surface `eeprom_ready_flag`,
  `clear_eeprom_ready_flag`, `enable_eeprom_ready_interrupt` plus the
  ISR body `eeready()`.
- **User Row**: `userrow_read`, `userrow_write`, `userrow_write_block`,
  `userrow_erase`.
- **Compile-time verbs**: `erase_at<Layout, addr, span>`,
  `write_word_at<Layout, addr>`, `eeprom_erase_at<offset, span>`,
  `userrow_write_at<offset>` - the same operations with the address and
  span as template parameters, refusing at compile time what the
  geometry, the alignment rules or the device size forbid.
- **The scratch region**: `scratch_region()` and the three linker
  facts it is built from, `image_low_end`, `rodata_load_start`,
  `rodata_load_end`.

`Sigrow` is the read-only view of the Signature Row: `device_id` (byte
by byte or as one number), `serial`, `tempsense0`, `tempsense1`.

`EepromStore` is the EEPROM as a `util/nv_record.hpp` backend, which is
what lets the record layout, the writer AO and the persistent panic
record be target-independent and host-tested.

### The errata as code

`erase()` validates the WHOLE range - every page of it - against the
section geometry and the protection bits before issuing any command,
because the silicon checks only the FIRST page of a multi-page erase
and erases the rest regardless (DS80000915F 2.7.1, DS80000882C 2.7.2;
observed on this die, see the findings). There is no work-around in the
errata sheet: the guard IS the work-around.

`erase_ignoring_protection()` is the same command with only the device
bounds and the alignment checked. It exists so the erratum can be
MEASURED rather than believed, and so the guard can be PROVEN - the
same call that `erase()` refuses, this one performs. Applications use
`erase()`.

### What is deliberately not here

`CHER` (chip erase) and `EECHER` (EEPROM erase) are not in
`NvmCommand`. CHER is UPDI-only in the silicon anyway (11.3.2.3.8);
EECHER is NOT, and one mistaken call would erase every setting a
product has ever stored. Erasing a whole memory is a provisioning act
and goes through the programmer. Fuse writing is not offered because
the silicon does not offer it (11.3.1.5): use `bench.py fuses`.

## How to use it

### Reading and writing Flash at run time

The fuses must give the image a section above BOOT to write into; the
image states the geometry it was built for, and the driver hands back
the free Flash between the image and its read-only data.

```c++
using Layout = brio::FlashLayout<128, 0>;      // BOOTSIZE 128, CODESIZE 0
if (!Layout::matches_fuses()) { /* this image is on the wrong chip */ }

const brio::FlashRange scratch = brio::Nvm::scratch_region();
brio::Nvm::erase(scratch.begin);                       // 512 bytes, ~10 ms
brio::Nvm::write_word(scratch.begin, 0x1234);          // ~83 us
const uint16_t back = brio::Nvm::flash_read_word(scratch.begin);
```

Both calls halt the CPU for their whole duration. An erase costs the
system ten milliseconds of everything: choose when, not just where.

To have an illegal address refused before the bench:

```c++
brio::Nvm::write_word_at<Layout, 0x10000>(0x1234);     // fine
brio::Nvm::write_word_at<Layout, 0x0800>(0x1234);      // does not compile
```

### Storing a setting that survives a power cycle

```c++
struct Settings { uint16_t rate; uint8_t mode; uint8_t flags; };
using Record = brio::NvRecord<Settings, brio::EepromStore, 16>;

const std::optional<Settings> s = Record::load();       // nothing = never stored
Record::store(Settings{9600, 4, 0x0F});                 // writes only what changed
```

`store()` reads before it writes and touches only the bytes that
differ: storing an unchanged value costs zero endurance, changing one
field costs that field plus the two checksum bytes. A record whose
checksum or version does not match reads back as nothing, so an erased
EEPROM, an older layout and a write torn by a power loss all look like
"no value" instead of like garbage.

### Writing the EEPROM without stalling

Ten milliseconds per byte is not something to busy-wait on. The writer
AO does one byte per interrupt, so the kernel keeps running between
them.

```c++
using Writer = brio::NvWriter<brio::EepromStore, AvrPlatform>;

ISR(NVMCTRL_EE_vect) {                    // the app binds the vector
    brio::Nvm::eeready();                 // level flag: MUST be disabled here
    brio::post<Writer>(brio::NvReady{});
}
...
brio::post<Writer>(brio::NvWrite{addr, brio::lend<brio::Lease::reply>(bytes), len,
                                 brio::reply_to<MyAo, brio::NvDone>()});
```

The bytes are lent, not given: the field's `Lease::reply` says they
must stay alive and unchanged until `NvDone` arrives. A
second request while one is in flight waits in a small FIFO; a full
FIFO is answered at once with `nv_rejected`.

### A panic that survives the plug being pulled

```c++
using Panic = brio::PersistentPanic<brio::EepromStore, 0>;

struct Reporter {                          // save, then report at next boot
    [[noreturn]] static void report(brio::PanicCode c, uint8_t x) {
        Panic::report(c, x);
        brio::Reset::software();
    }
};
brio::panic<AvrPlatform, Reporter>(brio::PanicCode::assert_failed, 7);
...
const std::optional<brio::PanicRecord> old = Panic::take();   // at boot
```

The reporter writes with bounded polled waits, which is correct in that
one context: `panic()` masks interrupts for good and the kernel will
never run again, so there is nothing left to be responsive to.

### Writing the User Row

```c++
brio::Nvm::userrow_write(24, 0x5A);   // only clears bits: the byte must be 0xFF
brio::Nvm::userrow_erase();           // takes all 32 bytes: there is one page
```

The row holds the board's identity label
([userrow.md](userrow.md)), so an erase wipes that too - read the row
first and write it back.

### `NvmFlash` - this flash as a block store

[`avrdx/nvm_flash.hpp`](../../lib/brio/src/avrdx/nvm_flash.hpp) presents
the Flash as the `FlashMedia` the target-independent block allocator
runs on ([design/nv-heap.md](../design/nv-heap.md)). It is four verbs
over the driver above - read is `flash_read`, program is `write_block`
(FLWR selected once), erase is `erase` with its whole-range protection
check - plus the two things the allocator cannot work out for itself:

**The granularities are different numbers.** `erase_size` is 512 and
`write_cell` is 2: a page is erased, a WORD is programmed, and a word
may be programmed once between erases.

**The zones, and where their bounds come from.** gcc puts code and the
`.data` initializers low and `.rodata` in a flash section reached
through the FLMAP window, so a linked image leaves TWO bands of free
flash rather than one remainder at the top:

| Zone | Ceiling | Floor |
|------|---------|-------|
| middle | `__rodata_load_start`, rounded down to a page | `__data_load_end` rounded up to a page, RAISED to `Nvm::boot_end()` |
| tail | the end of the Flash (the heap carves its own map home out of the top) | `__rodata_load_end`, rounded up to a page |

Two things about that table are load-bearing. The symbols are the LOAD
addresses, which on this toolchain are real flash addresses; their
run-time twins (`__rodata_start` and friends) are data-space aliases
based at 0x800000 and would be nonsense as flash addresses. And the
middle floor is raised to the BOOT boundary because **SPM cannot write
inside BOOT whatever the fuses say** - under the bench geometry
(BOOTSIZE = 128) all the code is in BOOT anyway, so the boundary is
what the zone starts at.

**The build id** is a link-time constant: `CMakeLists.txt`'s
`avr_add_app()` passes `-Wl,--defsym,__nvheap_build_id=<epoch>` to
every image, the way it locks FLMAP. It is read as the symbol's four
relocation bytes and never dereferenced - the value is a number, and a
pointer is 16 bits wide on
this part. The heap records it in every map version as a diagnostic; a
block's validity is its checksum's business, never its build's.

Storing a table that survives the next reflash:

```c++
brio::NvHeap<brio::NvmFlash> heap;          // RAM only until it is mounted

const auto& r = heap.mount();               // read-only: no cycle spent
if (!r.mounted()) { /* the image has grown over the map home */ }

if (const auto block = heap.find(cal_id)) { // came through
    block->read(0, std::span<uint8_t>(table, block->length));
} else if (auto w = heap.alloc(cal_id, sizeof table)) {
    w->append(std::span<const uint8_t>(table, sizeof table));
    w->seal();                              // the commit point
}
```

## The build invariants every image carries

Three things about NVMCTRL are settled once, for every image, and not
left to each app:

- **`src/glue/ivsel_boot.cpp`** is compiled into every env (through
  `[common] base_src_filter`). It is a four-instruction `.init3`
  fragment that sets `CPUINT.CTRLA.IVSEL` under CCP, before anything
  can enable an interrupt. Without it any image on a chip with
  `BOOTSIZE != 0` reset-loops on its first interrupt.
  `Nvm::vectors_in_boot()` is the same store as a run-time verb and
  `vectors_in_boot_armed()` is the readback the suite asserts.
- **FLMAPLOCK is set at startup**: `avr_add_app()` (`CMakeLists.txt`)
  appends `-Wl,--defsym,__flmap_lock=1` to the link, which makes the linker
  script's `__flmap_value_with_lock` carry the lock bit into the
  FLMAP write the C runtime already emits. The window is a mode that
  no brio verb uses, and a mode nothing uses can only change by
  accident. An app that must exercise the field says
  `// build: flmap_lock = 0` in its header.
- **A build id is defsym'd into the link**: the same function appends
  `-Wl,--defsym,__nvheap_build_id=<epoch>`, read back by `NvmFlash`
  above. It is the NEWEST SOURCE TIMESTAMP rather than the time of the
  link, and that is a bench requirement, not a preference: an unchanged
  tree must relink to the same bytes or reflashing it stops being safe
  (see [bench.md](../bench.md) on the three erase regimes). An image
  that never names the heap never references the symbol and pays
  nothing for it.

## Bench findings (`test_avr_nvm`, AVR128DB48 rev A5, 24 MHz, 5 V)

Times are counted in CLK_PER cycles by a TCB pair cascaded into one
32-bit counter, which keeps counting while the CPU is halted.

### Programming times, against table 39-7

| Operation | Measured | Table 39-7 |
|-----------|----------|------------|
| EEPROM byte write (EEWR) | 65 us | 70 typ, 75 max |
| EEPROM byte erase-and-write (EEERWR) | 10087 us | 10070 typ |
| EEPROM byte erase (EEBER) | 10041 us | 10000 typ, 11700 max |
| EEPROM 32-byte erase (EEMBER32) | 10040 us | same row |
| Flash word write (FLWR) | 83 us | 70 typ, **75 max** |
| Flash page erase (FLPER) | 10093 us | 10000 typ, 11700 max |

Two things to take from it. **Erasing 32 EEPROM bytes costs exactly
what erasing one costs** - 10040 vs 10041 us - and the same holds for
the Flash's five multi-page spans, as 11.3.2.2 promises. And a **Flash
word write measures 83 us, above the table's 75 us maximum**; the
measurement has the ruler's own cost (70 cycles) subtracted and the
driver's stores around the SPM are a few dozen cycles, so the excess is
the silicon's, not the driver's.

### What an operation costs the rest of the system

- A Flash **page erase halts the CPU for its whole 10 ms**: a loop that
  turns every 23 cycles executes 0 of the 10528 turns it would have.
- An **interrupt is delayed by almost the whole erase**: worst case 36
  CLK_PER cycles on a quiet CPU, 217892 cycles (9078 us) with a page
  erase in the way.
- A **software timebase loses the erase entirely**. 10 ms is ten ticks
  of the 1024 Hz PIT timebase, and the counter advances by **one**: the
  PIT keeps counting in its own clock domain, but its interrupt can
  only be serviced once the CPU is given back, and one pending
  interrupt is one interrupt however long it waited. Anything that
  measures time by counting interrupts is wrong by the duration of
  every Flash erase it sits through.
- An **EEPROM write does not halt the CPU**: a polling loop turned
  10476 times during one erase-and-write, and with the writer AO the
  1024 Hz timebase advanced 84 ticks across an 80 ms eight-byte
  transfer while the main loop turned 14064 times.

### The multi-page erase erratum, observed

Under a geometry that puts an APPDATA section boundary in the middle of
a two-page erase block (BOOTSIZE 128, CODESIZE 223, so APPEND lands at
114176 between the two pages of the block at 113664), with APPDATAWP
set:

- a SINGLE-page erase of the protected page raises `write_protect` and
  the page is intact - the protection works;
- a TWO-page erase over the same page raises **no error at all** and
  **erases the protected page**, because only the first page of the
  range was checked.

That is DS80000915F 2.7.1 in the flesh on rev A5, and it is why
`Nvm::erase()` validates every page of the range itself.

### The error field

- Storing into the array with **no command selected** raises
  `invalid_command` and nothing is written - the documented way to ask
  whether a command is still armed, and the only way to see that code.
- Aiming an unguarded erase at BOOT raises `write_protect`, from BOOT
  code, exactly as table 11-1 says.
- `NVMCTRL.CTRLA.CMD` is drawn as seven bits in the register summary
  but **only six are implemented**: writing the reserved code 0x7F
  reads back 0x3F. Selecting a reserved code raises **no error by
  itself** - the error appears when a store is attempted under it.
- The field clears by writing it to zero, not by writing a one.

### Geometry, mapping and the sections

- gcc 16 puts `.rodata` in Flash **section 3** (load address 98304)
  and leaves code and the `.data` initializers low, so a 29 KB image on
  a 128 KB part has a large contiguous hole in the MIDDLE and not a
  remainder at the top. With BOOTSIZE 128 the scratch region is
  therefore **65536 .. 98304, 64 pages, 32 KB**, aligned at both ends
  and big enough for the largest erase command.
- FLMAP reads 3 at boot, matching the section `.rodata` was linked
  into; all four sections can be selected while unlocked; FLMAPLOCK
  reads back and the field then refuses to move; a reset clears it.
- `APPCODEWP` and `BOOTRP` behave as documented and are cleared by a
  reset. `BOOTRP` set from BOOT code does not disturb code running in
  BOOT - the protection acts on accesses from other sections, and this
  suite keeps running with it set.
- IVSEL stays armed across every reset, because `.init3` sets it again
  on each boot.

### EESAVE and the chip erase (checked over UPDI, not from firmware)

With a marker written into the EEPROM: a chip erase with **EESAVE set
leaves it untouched**, and a chip erase with **EESAVE clear wipes it to
0xFF**. Both chip erases left the fuses and the 32-byte User Row label
exactly as they were, as 11.3.3 and 11.3.1.4 promise.

### The services

- An unchanged `NvRecord::store()` writes **zero** bytes; changing one
  byte of the payload writes **three** (the byte plus the two checksum
  bytes); a single flipped payload bit makes the record load back as
  nothing, and restoring the byte makes it valid again.
- The writer AO takes **one interrupt per byte** and none at all for an
  unchanged run.
- A panic record written by the EEPROM reporter survives a software
  reset alongside the SRAM breadcrumb, carries the same code and
  context, and is consumed by the first `take()`.
- The User Row: a byte write into an erased byte works with no erase;
  one erase takes all 32 bytes down; the row writes back byte by byte
  and `board_id()` reads the label again.

### The declared wear budget

`test_avr_nvm z` uses a fixed set of pages so the arithmetic means
something. Per run of `z`, counted in erase cycles:

| Storage | Erase cycles per run | Endurance | Runs before the spec |
|---------|----------------------|-----------|----------------------|
| scratch page 0 (65536) | 2 | 1000 | ~500 |
| scratch page 1 (66048) | 3 | 1000 | ~330 |
| scratch pages 40..43 | 2 each | 1000 | ~500 |
| the other scratch pages 2..63 | 1 each | 1000 | ~1000 |
| EEPROM bytes 0..7, 16..23, 32..63, 96..103 | at most 10 | 100k | ~10000 |
| User Row | 0 | Flash class | - |

Letter `u` costs the User Row **one erase cycle per run** and is
therefore not part of `z`. Letter `g` needs a fuse change and works on
two pages of its own, high in the Flash and outside the scratch region
(113664 and 114176 under the geometry it asks for), at about four erase
cycles each; it is not part of `z` either.

## Not covered yet

Driver gaps - the chapter's features this driver does not implement:

- **No bootloader.** Nothing here writes BOOT (nothing can), and no
  boot loader is built. The door is left open at zero cost: BOOTSIZE 1
  is 512 bytes, which is what an Optiboot-DX fits in.
- **No flash JOURNAL.** Wear levelling of payload pages, a log or ring
  structure, a second copy - none of it exists, and a policy of that
  kind needs a user with real numbers behind it. What does exist is a
  block store: `NvmFlash` above carries the allocator described in
  [design/nv-heap.md](../design/nv-heap.md), which covers "a table that
  survives" and deliberately not "a log that grows".
- **No fuse writing**, because the silicon has none from software; and
  no chip-erase commands, deliberately (above).
- **`NVMCTRL.DATA` and `NVMCTRL.ADDR` are not exposed.** They report
  the last value and address the controller saw and program nothing;
  no verb here needed them. A debugger view of them would be the only
  use.

Implemented but not bench verified:

- **AVR DA silicon.** There is none on this desk. The DA-only erratum
  DS80000882C 2.7.1 (FLMAP ignored by the protection check) is honoured
  by construction - no verb reads or writes Flash through the data
  space window - but nothing has been measured on a DA part. Likewise
  the DA's lower PFM endurance (DS80000882C 2.17.1, 1k instead of 10k)
  is what the wear budget above already assumes.
- **The `command_collision` and `ongoing_program` error codes.** The
  driver's own sequencing waits for the busy flags and passes through
  NOCMD, so it cannot produce either; they have never been seen.
- **A power loss during a write.** The checksum is designed to catch a
  torn record and does so against a corrupted byte, but no test has cut
  the supply mid-write. The data sheet's advice (11.3.3) - enable the
  BOD, use the VLM to refuse a write near the brown-out level - is not
  implemented as policy anywhere.
