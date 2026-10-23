# NVMCTRL - the nonvolatile memories (AVR DA, DB and EA)

This document is for a programmer who needs data to outlive a reset, a
power cut or a reflash on an AVR DA, DB or EA: a setting kept in the
EEPROM, a panic record the next boot can read, a table in flash that
a page-selective reflash leaves in place, or the board's label in the
User Row.
`Nvm` is the driver over NVMCTRL, the controller that erases and writes
those memories: a monostate (a class whose state is static, with no
instances), one function per operation, each erase or write checking
its address before it touches the controller (the reads, plain loads,
check none). Most programs reach it through the
target-independent services above it. `EepromStore` presents the
EEPROM to `util/nv_record.hpp`, where `NvRecord` keeps one typed value
with a checksum at a fixed offset; `NvWriter` is an active object (an
AO: a component of a brio program with its own event queue, served one
event at a time by the kernel,
[learn/introduction.md](../learn/introduction.md)) that writes the EEPROM one operation per interrupt, so the rest of the
program keeps running while the memory works; `PersistentPanic` saves a
panic record in the EEPROM. On the DA/DB, `NvmFlash`
([`avrxt/nvm_flash.hpp`](../../brio/avrxt/nvm_flash.hpp)) presents the
flash to the block allocator `NvHeap`.

The page covers two generations of the controller, and each fact
belongs to its die. The AVR DA and DB carry NVM version 2: the CPU
writes the flash a word at a time, and the EEPROM and the User Row a
byte at a time. The AVR EA carries NVM version 3, and brio uses its EEPROM
alone, written by page in runs (a run is the unit a transfer moves, a
buffer and a length with a single byte its degenerate case; here 1 to 8
consecutive bytes inside one page, what one operation writes); the
EA's flash and User Row self-programming are declined. The part's
device header decides which half of `avrxt/nvm.hpp` compiles
(STATUS.FLBUSY, `NVMCTRL_FLBUSY_bm`, is version 3's own bit), and the
EEPROM services' surface is the same on both.

The smallest use, a setting that survives a power cycle, quoted from
"Storing a setting that survives a power cycle" below; the same lines
build and run on the DA, the DB and the EA:

```c++
struct Settings { uint16_t rate; uint8_t mode; uint8_t flags; };
using Record = brio::NvRecord<Settings, brio::EepromStore, 16>;

const std::optional<Settings> s = Record::load();       // nothing = never stored
Record::store(Settings{9600, 4, 0x0F});                 // writes only what changed
```

Line by line: `Settings` is the value to keep, a plain struct stored
byte for byte (`NvRecord` refuses at compile time a type that is not
trivially copyable); `Record` names the record - the payload type, the
store it lives in (`EepromStore`, the EEPROM) and the byte offset where
it starts, 16 - and the record takes four header bytes (a magic byte,
the version, a CRC-16) plus the payload, eight bytes here, 16 to 23;
`load()` returns the stored value, or nothing when the EEPROM holds no
valid record of this version at that offset (erased, an older layout, a
write torn by a power loss); `store()` writes only the bytes that differ
from what is stored and returns how many it wrote, zero when the value
is already there.

What decides correctness even there:

- **Where the record sits is the programmer's choice and nothing
  checks it against the other users of the EEPROM** (another record,
  the panic record, a writer's request). A record that runs past the
  end of the EEPROM is caught at run time only: `load()` returns nothing
  and `store()` writes nothing.
- **`store()` blocks.** It polls the EEPROM through each operation, with
  a bounded wait, and returns when the last one is over. On the DA/DB
  every differing byte is an erase-and-write of about 10 ms (10087 us
  measured on an AVR128DB48, below); on the EA the differing bytes of
  one 8-byte page are one operation of about 2.6 ms, or 5.2 ms when it
  must erase and write. Called from an active object's handler, that
  time is taken from every other active object; the writer that does
  not block is `NvWriter` ("Writing the EEPROM without stalling"
  below), and it needs the NVMCTRL vector bound by the program with
  `ISR(NVMCTRL_EE_vect)` on the DA/DB and `ISR(NVMCTRL_EEREADY_vect)` on
  the EA.
- **On the AVR EA's rev B1 the supply decides whether a write holds**
  (DS80001048D 2.2.1, "The errata as code" in the EA's section): no
  write checks it; `Nvm::supply_guarded()` answers at boot, and is true
  on rev B1 once the BODCFG fuse holds 0x4D (`brio fuses <board>
  bodcfg=0x4D`). A program that keeps settings on that part asserts it
  at boot.
- **A setting survives a chip erase only with the EESAVE fuse set**:
  a chip erase with EESAVE clear wipes the EEPROM to 0xFF (the findings
  below, "EESAVE and the chip erase").

Writing the flash, on the DA/DB alone, adds two more: the fuses must
give the image a section above BOOT to write into (under the shipping
`BOOTSIZE = 0` no software can write any flash), and a flash erase or
write halts the CPU for its whole duration, interrupts included ("Who
stalls whom" below).

The rest of this page, in order: the documents of record; the AVR
DA/DB's NVM version 2 - what the silicon does, the types and verbs, how
to use it (the flash, a setting, the writer, the persistent panic, the
User Row, `NvmFlash` as a block store), the build invariants every image
carries, and the bench findings on the AVR128DB48 and the AVR128DA48
with the declared wear budget; then the AVR EA's section - what the
silicon does, the silicon's offer used or declined, the types and verbs,
the errata as code, how to use it, the bench findings on the AVR64EA48
and its wear budget; and what is not covered.

## Documents of record

Data sheet **DS40002247B** chapter 11 (NVMCTRL),
chapter 15 (CPUINT, for the vector-table bit) and section 39.8 (the
memory programming times); errata **DS80000915F** 2.7.1 / 2.7.2 (AVR
DB) and **DS80000882C** 2.7.1 / 2.7.2 / 2.17.1 (AVR DA). For the AVR EA
- NVM version 3, the EEPROM alone, its own section below - data sheet
**DS40002443A** chapter 11, sections 8.5 and 8.6 (the EEPROM and the
User Row), 8.8.2.2 (BODCFG), chapter 19 (the BOD) and table 35-8 (the
programming times); errata **DS80001048D** 2.2.1 and 2.2.3. Driver:
[`brio/avrxt/nvm.hpp`](../../brio/avrxt/nvm.hpp), with
the target-independent services over it in
[`util/nv_record.hpp`](../../brio/util/nv_record.hpp),
[`util/nv_writer.hpp`](../../brio/util/nv_writer.hpp),
[`util/persistent_panic.hpp`](../../brio/util/persistent_panic.hpp)
and [`util/crc.hpp`](../../brio/util/crc.hpp). Reference test
suites: `test_avr_nvm` on the bench, `test_nv_record` on the host. A
letter, below, is one command key of a suite's console and the group of
verdicts it runs: `test_avr_nvm`'s `z` runs its single-board set,
letters a to f, and the suite has no set that needs a partner board;
letters u, g and m stay outside `z` for the wear or the fuse change
each costs ("The declared wear budget" below).

Everything from here to the AVR EA's own section is the AVR DA/DB's
NVM version 2.

## What the silicon does

### Four memories, four jobs

| Memory | Size | Erase / write unit | Endurance | Survives a chip erase |
|--------|------|--------------------|-----------|-----------------------|
| Flash | 128 KB | page (512 B) / word | 1000 cycles | no |
| EEPROM | 512 B | byte / byte | 100k cycles | only with the EESAVE fuse |
| User Row | 32 B | the whole row / byte | Flash class | always |
| Signature Row | 64 B | read only | - | always |

The Flash figure deserves a second look: 1000 cycles is not the 10k
of the older AVR families - the DB datasheet lowered it to 1k "based
on validation data" (table 39-7; no typical value is published). A
chip erase spends one of those cycles on every page at once, which is
why the bench reflashes page-selectively (avrdude's default on these
parts) and reserves the chip erase for `brio flash --erase` - the
three measured erase regimes are in [../probes/atmel-ice.md](../probes/atmel-ice.md).

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
- The default vector-table location is the start of the application
  section, not address 0 (15.5.1 IVSEL, and 11.3.1.1 note 1). The
  moment `BOOTSIZE` stops being 0 the hardware looks for vectors in
  empty Flash and the first interrupt of any kind jumps into 0xFFFF -
  which is a reset loop, not a crash. `CPUINT.CTRLA.IVSEL` must be set,
  and it is correct under every geometry because BOOT always starts at
  0.

Two further bits in `NVMCTRL.CTRLB` are one-way until a reset:
`APPCODEWP` and `APPDATAWP` refuse all further writes to their section,
and `BOOTRP` makes BOOT unreadable and unexecutable from outside it -
that bit can only be written from BOOT code and takes effect only when
execution leaves the BOOT section.

### The two ways to reach the Flash, and the one this driver uses

The Flash is visible twice: as code space through LPM/SPM with a 24-bit
`RAMPZ:Z` address, and as a 32 KB window in data space whose position
`NVMCTRL.CTRLB.FLMAP` selects in blocks of 32 KB (11.3.2). This driver
uses **code space only** - ELPM to read, SPM to write. That is a
deliberate, load-bearing choice:

- DS80000882C 2.7.1 (AVR DA, every revision) says the inter-section
  protection check ignores FLMAP, mirroring BOOT into every mapped
  section, so a write through the window with `FLMAP != 0` can land
  somewhere else entirely. The errata's own work-around is "use only
  SPM to write and LPM to read". With no verb writing through the
  window, the item is removed rather than left to be remembered.
- Every verb takes a `uint32_t`, so the classic way of losing the top
  half of a 128 KB part - a 16-bit `&symbol` - cannot happen.

FLMAP is still exposed (`flmap`, `set_flmap`, `flmap_locked`,
`lock_flmap`), because an app may want the window for its own reads,
which is the half of the erratum that is explicitly fine, and because
the C runtime already uses it: gcc places `.rodata` in a Flash section
and writes FLMAP in `.init`.

### The command model

Every write and erase is two steps (11.3.2.3): a command is selected in
`NVMCTRL.CTRLA` under CCP with the SPM key, and then carried out by an
ordinary store into the memory array - SPM for the Flash array, an ST
to the mapped address for EEPROM and the User Row. Changing from one
command to another must pass through NOCMD or NOOP, or the controller
raises a collision error instead of obeying. `NVMCTRL.DATA` and
`NVMCTRL.ADDR` take no part in this: they are observers reporting the
last value and the last address the controller saw (11.5.6, 11.5.7).

`NVMCTRL.STATUS` carries the two busy flags and a sticky three-bit
ERROR field cleared by writing it to zero.

### Who stalls whom

A **Flash** erase or write halts the CPU for its whole duration - about
10 ms for an erase, tens of microseconds for a word. Nothing runs,
interrupts included. An **EEPROM** write does not halt the CPU; the CPU
is halted only if it starts a second NVM access while one is running.
That difference is why an interrupt-paced EEPROM writer is possible and
an interrupt-paced Flash writer is not.

## Types and verbs

### Geometry and layout

`flash_size`, `flash_page_size`, `eeprom_size`, `eeprom_base`,
`userrow_size`, `userrow_base` are constants taken from the device
header, never copied; `flash_section_block`, the 512-byte block the
BOOTSIZE and CODESIZE fuses count in (11.3.1.1), is the one the driver
states itself. `FlashSection` names the three
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
- **Flash**: `flash_read` (byte and block) and `flash_read_word`, all
  through ELPM,
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
the silicon does not offer it (11.3.1.5): use `brio fuses`.

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

[`avrxt/nvm_flash.hpp`](../../brio/avrxt/nvm_flash.hpp) presents
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

- **`src/glue/ivsel_boot.cpp`** is compiled into every image:
  `avr_add_app()` adds it to every executable. It is a four-instruction
  `.init3` fragment that sets `CPUINT.CTRLA.IVSEL` under CCP, before
  anything can enable an interrupt. Without it any image on a chip
  with `BOOTSIZE != 0` reset-loops on its first interrupt.
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
  (see [../probes/atmel-ice.md](../probes/atmel-ice.md) on the three erase regimes). An image
  that never names the heap never references the symbol and pays
  nothing for it.

## Bench findings (`test_avr_nvm`: the AVR128DB48, rev A5, 24 MHz, 5 V, unless a section names the AVR128DA48)

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

### The AVR128DA48 (REVID 0x17, rev A7, 24 MHz from its external clock, 5 V)

- `z` runs green (112 verdicts) on the DA as on the DB. The times sit
  where the DB's do and where the DA's own table (DS40002183A table
  37-8) puts them: EEWR 65 us, EEERWR 10077 us, EEBER 10029 us,
  EEMBER32 10030 us, Flash word write 85 us (70 typ.), page erase 10068
  us (10 ms typ.); a page erase halts the CPU for all of it and delays
  an interrupt by 9067 us, exactly the DB's picture.
- **DS80000882C 2.7.1, the window's protection check, reproduced**
  (letter `m`, the DA only, ten runs of ten): FLMAP moved to section 2,
  the window READS the scratch page at 88064 correctly (a marker written
  through SPM comes through it), but a page erase through the window
  into that page - APPCODE, which BOOT may write, and which SPM erases
  in letter `d` - raises `write_protect` and leaves the marker intact.
  With BOOTSIZE 128 every offset of the 32 KB window is BOOT's, which is
  exactly what the erratum says the check looks at. The positive control
  - a window offset OUTSIDE BOOT, where the same erase must be accepted -
  needs BOOTSIZE below 64, a fuse this bench does not move.
- `test_avr_nvheap` `z` runs green on the DA too (51 verdicts, one run:
  the mount, the round trip, the map rotation, the rewrite in place and
  the persistence across a real reset, at its dozen page erases).

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
therefore not part of `z`; letter `m` (a DA only) costs scratch page 44
(88064) two erase cycles per run - the marker's erase and the cleanup
erase - and is outside `z` too. Letter `g` needs a fuse change and works on
two pages of its own, high in the Flash and outside the scratch region
(113664 and 114176 under the geometry it asks for), at about four erase
cycles each; it is not part of `z` either.

## The AVR EA (DS40002443A ch. 11): NVM version 3, the EEPROM engine

The AVR EA's NVMCTRL is another generation of the controller, and the
second half of `nvm.hpp` is written from its own chapter; the part's
header decides which half compiles. What the EA keeps is the EEPROM:
its flash and User Row self-programming are declined (a 64 KB part
with a 512-byte EEPROM keeps its small values in the EEPROM), so the
DA/DB's flash and User Row write verbs exist on the EA only as names
that refuse at compile time with that reason, `NvmFlash` and the heap
and journal over it refuse the part, and BOOTSIZE stays at the
factory 0 - the whole flash BOOT. The User Row stays readable
([userrow.md](userrow.md)).

### What the silicon does

The EEPROM is 512 bytes in **64 pages of 8** (table 8-3) at 0x1400,
read with a plain load, endurance 100k erase/write cycles per byte
(table 35-8), kept across a chip erase by EESAVE (8.8.2.4). It has a
page buffer of its own (the flash and the User Row share another,
11.3.2.2), and an operation erases or writes **any subset of one page**:
only the bytes loaded into the buffer are touched (11.3.1.2, table
11-4). Three page operations - EEPW (write), EEPER (erase), EEPERW
(erase, then write) - plus EEPBCLR (clear the buffer) and EECHER (the
whole EEPROM). STATUS's busy bits are swapped against the DA/DB (EEBUSY
is bit 0 here), its ERROR field has its own codes (2 WRITEPROTECT, 3
CMDCOLLISION, 4 WRONGSECTION, no 1), CTRLB grows EEWP (bit 3, one-way
until a reset), and the one NVMREADY vector (#33) carries EEREADY and
the flash's FLREADY.

**The command model, as the silicon runs it** (rev B1, `test_avr_nvm`'s
letter b). The chapter states it two ways - "fill the page buffer, then write
the command" (11.3.2.3, 11.3.2.4.2) and "commands are enabled by CTRLA
and must be followed by writes using ST" (11.3.2.4, 11.5.1) - and the
silicon runs the first, the way Microchip's own avr-libc
(`eeprom_write_byte` for this part) and pymcuprog's NVM version 3
driver do:

- a store into the EEPROM's range **loads** the buffer (ANDed with what
  it holds) whatever CTRLA holds, and starts nothing;
- **writing** a page command to CTRLA starts it at once, on the page of
  the last load, over the loaded bytes; the command stays in CTRLA
  after the operation. The DA/DB's order - command, then store - writes
  nothing here (the command finds an empty buffer and does not even
  run);
- the buffer is indexed by the **low three address bits alone**: loads
  into two pages land in one page, the last load's, at the first
  load's offset;
- CTRLA written **while an operation runs** is refused with
  CMDCOLLISION and the operation completes - so a command can be
  cleared only after its operation; a command written over a command
  with the controller idle starts with no error;
- the buffer is cleared by every operation, by EEPBCLR on its write
  (no store needed), by a reset and by a wake from sleep (11.3.2.2,
  11.3.5; measured for Idle). A load nobody committed rides along into
  the next operation: measured, which is why every operation here opens
  with EEPBCLR.

**Who stalls whom.** An EEPROM operation leaves the CPU running
(11.3.2.4.1) and interrupts are served on time meanwhile - but any
EEPROM read or buffer load during it is a bus wait that suspends the
instruction, and every interrupt with it, until the operation ends
(11.3.2.1; 11.3.2.2's note). The operations are timed by an oscillator
of the controller's own: halving CLK_PER halves the count in CLK_PER
cycles, and tuning OSCHF (not running under the crystal) moves nothing.

### The silicon's offer, used or declined

| Item | Here | Why |
|------|------|-----|
| The EEPROM page buffer and the page run (1..8 bytes, one operation) | USED: `eeprom_run`, `eeprom_store_run`, and through the store contract's run verb (`EepromStore::write_run`) by the record, the writer AO and the panic record | eight bytes cost what one does (measured to 0.1 %): the unit of cost is the page |
| EEPW / EEPER apart | USED: the store chooser picks EEPW onto 0xFF bytes and EEPER to 0xFF | each is half an EEPERW (2.6 against 5.2 ms); EEPW onto a byte that is not 0xFF is outside the chapter's precondition (11.3.2.4.1) and never chosen |
| EEPERW | USED where neither half alone does | one operation for a byte that is neither erased nor becoming 0xFF |
| EEPBCLR | USED before every operation | a stray load would AND itself into the next operation (measured); its cost is a few cycles of halted CPU per operation |
| Byte-granular marking (only loaded bytes are touched) | USED: the store chooser loads only the bytes that differ | endurance: an unchanged byte costs no cycle |
| The CPU running during an operation | USED by `NvWriter` (one interrupt per operation) | the milliseconds are the program's while it touches no EEPROM |
| EEREADY interrupt | USED (`enable_eeprom_ready_interrupt`, `eeready()`) | the writer's pace; a level interrupt, disabled in its handler |
| EEWP | USED: `protect_eeprom()`, checked by every verb before it starts | one-way protection of the settings; the silicon refuses under it too, an erase included (measured) |
| EECHER | DECLINED | one mistaken call erases every setting: a provisioning act for the programmer |
| FLREADY, the flash commands, the User Row's CPU write | DECLINED with flash self-programming | the part keeps its small values in the EEPROM (above); the verbs refuse by name |
| The VLM (BOD chapter) to refuse a write near the brown-out level (11.3.3) | DECLINED | the supply guard below covers the erratum; a battery-powered program is its first user |
| NVMCTRL.DATA and ADDR | not exposed | observers of the last access, they program nothing |

### Types and verbs

Geometry from the header (`eeprom_size`, `eeprom_base`,
`eeprom_page_size`, `userrow_size`, `userrow_base`, `flash_size`,
`flash_page_size`); `NvmError` with this generation's four codes;
`NvmCommand` with the EEPROM's commands only; `EepromOp` - `write`,
`erase`, `erase_write`, each the command itself; `EepromErase` with
spans of 1, 2, 4 and 8 bytes - the DA/DB's 16 and 32 do not exist.
`Nvm`:

- **The run**: `eeprom_run(offset, src, len, op)` - one operation over
  1..`eeprom_run_room(offset)` bytes inside one page; and its three
  steps, public for a caller that writes only some bytes of a page:
  `eeprom_open()` (wait for idle, EEPBCLR), `eeprom_load(offset,
  value)`, `eeprom_commit(op)`. `eeprom_store_run(offset, src, len)`
  leaves src's bytes in place with the cheapest operation, loading only
  the bytes that differ, nothing at all when none does.
- **The byte** (the run's degenerate case, a short path of its own):
  `eeprom_byte(offset, value, op)`, `eeprom_write` (EEPW),
  `eeprom_erase_write` (EEPERW), `eeprom_store` (the chooser's), and
  `eeprom_erase(offset, span)` (EEPER of an aligned span).
  `eeprom_write_block` stores any run, a store run per page, blocking.
  `eeprom_erase_at<offset, span>` refuses at compile time what the
  alignment forbids.
- **State and the controller**: `eeprom_busy`, `flash_busy`, `busy`,
  `error`, `clear_error`, `wait_idle`, `wait_eeprom`, `selected`,
  `clear_command` (only with the controller idle), `issue(NvmCommand)`
  (CTRLA as is - on this generation a page command written there
  starts).
- **CTRLB**: `protect_eeprom`, `eeprom_protected`; `flmap`,
  `flmap_locked`, `set_flmap` (keyed: the whole of CTRLB is under CCP
  here, table 11-7 - measured: a plain store does not move FLMAP),
  `lock_flmap`.
- **EEREADY**: `eeprom_ready_flag`, `clear_eeprom_ready_flag`,
  `enable_eeprom_ready_interrupt`, the handler body `eeready()`.
- **Reads**: `flash_read` (LPM: the 16-bit Z reaches all 64 KB, no
  RAMPZ), `userrow_read`, `Sigrow` (TEMPSENSE0 a signed gain on this
  generation, 8.9.2.2); `vectors_in_boot`.
- **The supply**: `supply_guarded()`, below.

Every verb that starts an operation returns as soon as it has STARTED,
and refuses - touching nothing - a run that leaves its page or the
array, a span not aligned to itself, EEWP set, or a controller that
never went idle. `EepromStore` is the EEPROM as the `util/nv_record.hpp`
store, and one that TAKES A RUN (`NvRunStore`): `run_room()` is
`eeprom_run_room()`, `write_run()` is `eeprom_store_run()` (a length past
the room refused before it is narrowed to the engine's byte), `write()`
is `eeprom_store()`.

**The cost of an operation**, CPU side, by the release listing of
`test_avr_nvm`: **58 cycles for one byte** (`eeprom_erase_write`: the
range and EEWP checks, the idle test, EEPBCLR with its key and the
chapter's seven halted cycles, one store, the command with its key)
and **143 for a page of eight** (`eeprom_run`: eight cycles a byte in
its load loop). Letter b measures 70 and 173: the NVM accesses'
own waits on top of the count (it measures an EEPBCLR at 14 cycles
with its key and a buffer load at 12).
Against the operation's 2.6 to 5.2 ms that is 0.1 %, with interrupts
never masked. avr-libc's `eeprom_write_byte` for this part spends about
30: it has no EEPBCLR, no EEWP check and no bounded wait, and writes
whatever stray load the buffer holds along with its byte.

### The errata as code

- **DS80001048D 2.2.1** (rev B1): an NVM erase or write below 2.7 V may
  fail; the sheet's alternative is the BOD at BODLEVEL2 or above.
  `Nvm::supply_guarded()` is true when the silicon is not rev B1 (the
  device tables' `erratum_ea_nvm_supply_revid`, SYSCFG.REVID 0x21), or
  when the BOD holds BODLEVEL2 or above in CONTINUOUS mode both awake
  (ENABLE or ENABLEWAIT) and asleep - an operation runs on through a
  sleep (11.3.5), and a sampled BOD (128 Hz at best) can miss a dip
  shorter than its period while a 5.2 ms operation runs. A boot-time
  fact the suites assert, not a refusal inside every write (a cost every
  image would pay for a lab condition); `brio fuses` decodes BODCFG on
  an EA and WARNS when it leaves the writes unguarded. The provisioning
  is **BODCFG 0x4D**: LVL 2 (BODLEVEL2, 2.60 V typical rising, 2.47 V
  minimum falling - table 35-15; the sheet's own threshold for the
  workaround), SAMPFREQ 0, ACTIVE 3 (ENABLEWAIT), SLEEP 1 (continuous)
  - DS40002443A 8.8.2.2. The BOD then draws its continuous current
  (14 uA typical, DS80001048D 3.4) in every sleep.
- **2.2.3** (B1, B2): a store to an address >= 64 followed by an ST/STD
  below 64 or a write to SLPCTRL.CTRLA loses the second. A run's
  stores go to 0x1400.. and the next store below 64 is the CCP key,
  which avr/xmega.h's `_PROTECTED_WRITE_SPM` writes with OUT - the
  sheet's own workaround; read in the release listing of every engine
  path (`eeprom_byte`, `eeprom_store`, `eeprom_run`, `eeprom_erase`,
  `eeprom_store_run`).
- **2.4.1, 2.4.2, 2.2.2** are the flash's: with no flash write they
  have no user. 2.2.2 (flash endurance 1k when the part ever runs below
  BODLEVEL3) still bounds the reflashing a board on a lower supply can
  take.

### How to use it

A setting, the panic record and the writer are the DA/DB's code above,
unchanged - the store is `EepromStore` on both - with the EA's vector
name. Here the store takes a run, so each of them spends an operation
per page its change touches, not per byte: the panic record at offset 0
is one operation, a writer request of a page one interrupt.

```c++
ISR(NVMCTRL_EEREADY_vect) {               // #33, NVMREADY: EEREADY and FLREADY
    brio::Nvm::eeready();
    brio::post<Writer>(brio::NvReady{});
}
```

The page run, where a caller holds a page's worth of bytes:

```c++
uint8_t page[8] = {...};
brio::Nvm::eeprom_store_run(64, page, sizeof page);   // one operation, the cheapest
brio::Nvm::wait_eeprom();                              // or let EEREADY say it
brio::Nvm::clear_command();                            // only once it is idle
```

At boot, on a rev B1 part:

```c++
if (!brio::Nvm::supply_guarded()) { /* provision: brio fuses <board> bodcfg=0x4D */ }
```

### Bench findings (`test_avr_nvm` on the AVR64EA48, rev B1, 20 MHz quartz, 5 V, BODCFG 0x4D)

`test_avr_nvm z`: **110 of 110 in ten runs of ten.**

**The times**, the page buffer's operations on letter b's page (bytes
40..47) against table 35-8 (which publishes typical figures only, 2 ms
for an erase or a write), the range over the runs:

| Operation | 1 byte | 8 bytes | Table 35-8 |
|-----------|--------|---------|------------|
| EEPW (write) | 2622 us (52443..52446 cycles) | 2622 us (52447..52450) | 2 ms typ |
| EEPER (erase) | 2622 us (52447..52450) | 2622 us (52447..52450) | 2 ms typ |
| EEPERW (erase, then write) | 5246 us (104914..104917) | 5246 us (104914..104917) | not listed |

Eight bytes cost what one does and an erase-and-write is an erase plus
a write, to a few cycles in 105 000. On one page an operation repeats
to the cycle; between pages it does not: an EEPW takes 2622 us on page
5, 2640 us (half an EEPERW) on page 6, 2674 us on page 58 and 2691 to
2700 us on page 12 - 31 to 35 % above the table's typical. It is timed
by an oscillator of the controller's own: halving CLK_PER halves the
count in CLK_PER cycles (53486 -> 26791 on page 58), and OSCHF's tune
(+-24 steps; OSCHF does not even run under the crystal) moves nothing.
Which oscillator, and why the time depends on the page, is not
identified; a write that adapts its pulses to the cells it programs
would show both.

**What an operation costs the rest of the system**: a loop polling
STATUS turns 7000 to 7450 times through a 5.3 ms EEPERW; an interrupt
due 1 ms into it is served 33 cycles late, as on a quiet CPU (and the
operation is running at its stamp); a read of the EEPROM 1 us into it
returns only when the operation is over - the alarm waits with it,
86 800 to 87 400 cycles late - and so does a buffer load. Such an
access returns 70 to 120 us after the operation's own length (107 000
cycles against 105 400); a read right after EEBUSY drops takes 10
cycles. Whether the operation runs longer with an access pending or
the bus is released late is not separated.

**The engine's CPU cost**, measured with interrupts masked: 70 cycles
for a one-byte call, 173 for an 8-byte run, 201 for an 8-byte store run
that finds nothing to do, 78 for `EepromStore::write`; an EEPBCLR with
its key 14, one buffer load 12 - the same in every run.

**The services**, on the store's run path: `NvRecord`, `NvWriter` and
`PersistentPanic` behave as on the DA/DB (an unchanged store writes
nothing, one changed byte costs three, the panic record across a
software reset), an operation per page. The writer's eight bytes onto
erased cells are ONE operation and one interrupt, 2.78 to 2.81 ms, the
main loop's longest stretch between two turns 1539 to 1640 cycles (the
dispatch that scans and starts the run), the PIT's ticks all kept; the
same request through a store without the run verb (the suite's
`EepromByteStore`, the same EEPROM with the verb hidden) is eight
operations and eight interrupts, 21.58 to 21.60 ms.

The panic record's report with interrupts masked, onto an erased page
0: **2.73 ms on the run path** (54 507 to 54 747 cycles) - one operation
of page 0, timed in the same run at 53 399 to 53 663, plus 1000 to 1264
cycles of the reporter's own work - against **21.37 to 21.45 ms on the
byte path**, eight operations to within 0.33 %.

A run's CPU cost, masked, eight differing bytes: **the scan 319
cycles** (the first and last differing byte within the room: eight
EEPROM reads at about nine cycles each and the loop around them, the
call and its returned struct), **`write_run` 510** (the store run reads
the eight bytes again to choose its command, then EEPBCLR, eight loads
and the command: the 201 cycles a store run that finds nothing to do
costs, above, plus the loads at 12 each). The double read, some 80 cycles, is the price of a store that
chooses its own command from the bytes it holds; against a 2.6 ms
operation it is 0.15 %. The byte path spends about the same CPU in
total - eight `EepromStore::write`s at 78 - but in eight dispatches
and eight operations.

A 12-byte record over two pages, stored under a 1 ms alarm: on the run
path the longest gap between two alarm runs is **20 002 cycles** for a
period of 20 000 - nothing held the interrupt, because the record waits
for the EEPROM before every read; on the byte path it is 54 493 to
54 744 cycles: each read of the next byte holds the CPU and the alarm to
the end of the write before it.

**EEWP**: set, every verb refuses; the raw sequence past the driver is
refused by the silicon with WRITEPROTECT, an erase included, the byte
intact; a reset clears it. **FLMAP**: CTRLB is wholly under CCP (a plain
store reads back unchanged), `set_flmap` selects each of the four
sections. **CTRLA**: six bits of CMD are implemented (0x7F reads 0x3F),
and a reserved code raises no error.

### The declared wear budget

`test_avr_nvm z` on the EA, counted in operations per byte (each
counted as a full erase/write cycle, which overstates an EEPW or an
EEPER alone):

| EEPROM bytes | Letters | Operations per run | Endurance | Runs before the spec |
|--------------|---------|--------------------|-----------|----------------------|
| 0..7 (the panic slot) | c, f | 7 on byte 0, at most 5 on the others | 100k | ~14000 |
| 16..23 (the settings record) | c | at most 5 | 100k | ~20000 |
| 32..39 (letter b's first page) | b | at most 8 | 100k | ~12000 |
| 40..47 (letter b's second page) | b | at most 16 (byte 40) | 100k | ~6000 |
| 48..55 (letter e's page) | e | at most 15 (byte 48) | 100k | ~6600 |
| 56 | f | 2 | 100k | ~50000 |
| 96..103 (the writer's run) | c | 6 | 100k | ~16000 |
| 104..119 (letter e's record, pages 13 and 14) | e | 5 | 100k | ~20000 |

No flash and no User Row cycle: the EA's suite writes neither.

## Not covered yet

Driver gaps - the chapter's features this driver does not implement:

- **The AVR EA's flash and User Row self-programming**: DECLINED - a
  64 KB part with a 512-byte EEPROM keeps its small values in the
  EEPROM. The DA/DB's flash and User Row write verbs refuse on the EA at
  compile time with that reason, and `NvmFlash` - with the flash heap
  and journal over it - refuses the part; its rev B1's flash errata
  (DS80001048D 2.4.1, 2.4.2) have no user then. UPDI writes the User
  Row ([userrow.md](userrow.md)).
- **The store contract's run verb on the AVR DA/DB.** The DA/DB's
  `EepromStore` does not offer `NvRunStore`, so `NvRecord`, `NvWriter`
  and `PersistentPanic` serve it a byte at a time. That generation
  writes its EEPROM a byte per operation, so its room would be one and
  the verb would change only the writers' loop (the item below):
  adopted in its family's phase.
- **`NvRecord::store`'s byte path reads the next byte before it waits
  for the previous write** - the path the DA/DB's store takes. The run
  path waits before every read (on the AVR EA such a read is a bus wait
  that holds the CPU and its interrupts to the operation's end,
  measured above); the byte path keeps its shape until the DA/DB adopt
  the run verb in their family's phase.
- **The VLM as a write guard (AVR EA, 11.3.3)**: declined until a
  battery-powered program needs a write refused near the brown-out
  level; `supply_guarded()` covers the erratum on a mains-powered one.
- **No bootloader.** Nothing here writes BOOT (nothing can), and no
  boot loader is built. The door is left open at zero cost: BOOTSIZE 1
  is 512 bytes, which is what an Optiboot-DX fits in.
- **No wear levelling of payload pages and no growing log**, because a
  policy of that kind needs a user with real numbers behind it. What
  does exist is a block store - `NvmFlash` above carries the allocator
  of [design/nv-heap.md](../design/nv-heap.md), "a table that
  survives" - and, over the same `FlashMedia`, the small-value journal
  of [design/nv-journal.md](../design/nv-journal.md), host-tested on
  this family's 512/2 geometry and never mounted on an AVR: this family
  has a real EEPROM for small values (`NvRecord` over `EepromStore`),
  so the journal is born here with its first user.
- **No fuse writing**, because the silicon has none from software; and
  no chip-erase commands, deliberately (above).
- **`NVMCTRL.DATA` and `NVMCTRL.ADDR` are not exposed.** They report
  the last value and address the controller saw and program nothing;
  no verb here needed them. A debugger view of them would be the only
  use.

Implemented but not bench verified:

- **DS80001048D 2.2.1 itself on the AVR EA** - a write below 2.7 V
  failing, and the BOD's reset preventing it: the bench supply is a
  fixed 5 V; a variable supply would measure it.
- **The AVR64EA28 and AVR64EA32**: the same die as the bench's
  AVR64EA48, compiled on every package by the family fixture; no board.
- **The EA's `wrong_section` error**: the driver issues EEPROM commands
  on EEPROM addresses only and cannot produce it.

- **The DA's lower PFM endurance** (DS80000882C 2.17.1, 1k instead of
  10k): the wear budget above already assumes it; measuring it means
  wearing a page out, a thousand erase cycles of the part on the desk.
- **2.7.1's positive control**: a window write outside BOOT, accepted -
  the scratch region is BOOT-mirrored under the bench geometry, and
  BOOTSIZE below 64 would be needed (a fuse change).
- **The AVR DA/DB's `command_collision` and `ongoing_program` error
  codes** (the EA's collision is measured, above). The
  driver's own sequencing waits for the busy flags and passes through
  NOCMD, so it cannot produce either; they have never been seen.
- **A power loss during a write.** The checksum is designed to catch a
  torn record and does so against a corrupted byte, but no test has cut
  the supply mid-write. The data sheet's advice (11.3.3) - enable the
  BOD, use the VLM to refuse a write near the brown-out level - is not
  implemented as policy anywhere.
