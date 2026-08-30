# NVMCTRL - Non-Volatile Memory Controller (SAM C21)

> **PROVISIONAL.** The chapter's programming, protection and description
> surfaces are built and bench-verified; what is deliberately left out is
> either a one-way hazard (the security bit) or waits for another driver.
> The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 27, with the
memory map and the factory areas in ch. 9 (9.3 user row, 9.4 and 9.5
calibration, 9.6 serial number) and the timing and endurance numbers in
tables 45-41..45-44 - and errata DS80000740S, where **NVMCTRL has no
items at all**: 1.14.1 reads "Reserved" with dashes across every silicon
revision, and the revision history records that the one item this module
ever carried ("EEPROM Cache") was deprecated due to resolution. Code
written against older SAMD/SAMC examples may still carry a cache
workaround around RWWEE access; it is not needed here. Drivers:
`samc/nvm.hpp` (`Nvm` + `FlashWaitStates` + the factory views) and
`samc/nvm_flash.hpp` (`RwweePartition`, `RwweeFlash` the
`util/nv_heap.hpp` backend, and `RwweeJournalZone` the
`util/nv_journal.hpp` one). Family fixtures `test/family_samc/nvm.cpp`
and `test/family_samc/journal.cpp` plus their negatives under
`tools/check_samc.sh`; the bench suites are `test_samc_nvm` and
`test_samc_journal`.

## What the silicon does

**Two programmable arrays, and only one of them stops the CPU.** There is
the main array (256 KB at 0x00000000 on the 18A parts) and a dedicated
Read-While-Write EEPROM array, 8 KB at 0x00400000. They share the page
buffer, the ADDR register and the command register; what differs is the
command code and who stalls. Reading the main array while the main array
is programmed or erased stalls the AHB until the operation ends - and
since instruction fetch is a main-array read, that means the CPU. The
main array can be read freely while the RWWEE array is written (27.6.4.1,
27.6.4.2). Measured: through a row erase the CPU completes about 3950
turns of a polling loop on RWWEE and exactly ONE on the main array.

The RWWEE array is also the more durable of the two - 100k cycles against
the main array's 25k (45-43, 45-44) - and in exchange it is not cached,
so reads from it miss and a full-word access takes twice the data phase.
Both halves of that trade point the same way for stored records.

**Erase by row, program by page.** A page is 64 bytes; a row is four
pages, 256 bytes. Erase granularity is the row, program granularity the
page (27.6.2). Those are two different numbers and code that calls both
"page" will be wrong on one target or the other - note that here "page"
is the SMALL one, the opposite of the AVR family's usage.

**The page buffer is loaded by storing into the memory-mapped array**,
and it has two traps plus a limit.

- Stores must be 16 or 32 bits. An 8-bit store raises a system exception
  (27.6.4.3), which on this core is a HardFault. Every store the driver
  makes is 32 bits wide.
- The 64-bit holding register PBLDATA is reset to all ones whenever a
  store crosses a 64-bit boundary, and a section is rewritten wholesale
  from PBLDATA on each store - so a store pattern that leaves a section
  and comes back loses what it left there.
- Table 45-42's footnote, which is nowhere in chapter 27: **at most 8
  consecutive writes are allowed per row** before a row erase becomes
  mandatory. Programming each of a row's four pages once is 4, and safe.

The second trap is usually stated as "write ascending". Measured, the
real rule is narrower and more useful: **the two words of a 64-bit
section must be written back to back**. Ascending satisfies it; so does
descending, which crosses a boundary on every single store and still
comes out byte-exact, because 15-then-14 completes a section before the
crossing. What actually destroys data is interleaving.

**ADDR is a half-word offset from the section base**, not a byte address
and not an absolute one: the effective address is "start address of the
section + 2*ADDR" (27.8.8). Twenty-two bits could have held either
convention, so it was measured rather than trusted - a page-buffer load
at 0x00400100 leaves ADDR = 0x00000080 and one at 0x00020100 leaves
0x00010080. The register is also updated by hardware on every page-buffer
load, which is what made the measurement possible.

**Commands take a key, and errors are sticky.** CMD and the key 0xA5 must
reach CTRLA in one write; a wrong key, or a command issued while another
is running, sets STATUS.PROGE rather than failing loudly. INTFLAG.READY
is low for the duration and commands written while it is low are ignored.
The three error bits (NVME, LOCKE, PROGE) accumulate until written back,
exactly like the AVR's reset flags.

**Sixteen lock regions, and two different kinds of lock.** The main array
is divided into 16 equal regions (16 KB each at 256 KB) with one LOCK bit
apiece, zero meaning locked. The Lock Region and Unlock Region commands
change that state ONLY UNTIL THE NEXT RESET; the state a reset restores
lives in the user row. The RWWEE array has no regions - its rows are
writable regardless of the main array's locks.

**Register access protection.** CTRLA, CTRLB and ADDR are PAC
write-protected (27.5.5). PAC protection is off out of reset and no brio
driver enables it, so nothing here performs an unlock; a future PAC
driver must.

**The factory areas are read-only and nothing loads them for you.** The
user row at 0x00804000 is this family's fuse row (BOOTPROT, EEPROM
emulation size, the brown-out level and action, the watchdog's power-on
enable, always-on, period and window); it is in force from power-on and a
change takes effect only after a reset. The software calibration area at
0x00806020 holds values an application is expected to copy into its
peripherals - ADC0 and ADC1 bias, OSC32K CALIB, and the two OSC48M
calibrations for the 3.3 V and 5 V supply ranges - and the temperature
calibration area at 0x00806030 holds TSENS's. The die's 128-bit serial
number is four words at 0x0080A00C and 0x0080A040..48, unique only when
all 128 bits are used.

**The user row is written by the bench tool and not from firmware, on
purpose.** The commands exist - Erase Auxiliary Row (EAR) and Write
Auxiliary Page (WAP) - and this driver deliberately exposes neither. The
row IS the fuses: it survives a chip erase, and it carries the watchdog's
power-on ALWAYSON bit, the brown-out level and BOOTPROT, so a wrong word
is not undone by reflashing and an application has no business writing
one. Provisioning belongs to the programmer, which on the AVR side means
`tools/bench.py fuses` over UPDI and here means the same verb over SWD:
it reads and decodes every field of table 9-4, and a write is a whole-row
read-modify-write through EAR + WAP with the core halted, the old row
printed first, the new one read back and diffed, the BODCORE calibration
and Reserved bits carried across untouched, no raw bit escape, and
BOOTPROT, LOCK and ALWAYS-ON behind an acknowledgement flag. Nothing on
either side reaches the security bit. See [../bench.md](../bench.md).

Note that the device header's `NVMCTRL_USER_PAGE_OFFSET` is 0x00800000,
the base of the whole auxiliary space, and NOT the user row: the two
differ by 0x4000, and confusing them reads calibration data as fuses.

## Types and verbs

**`NvmConfig`** - the block's configuration.

| Field | Values | Default | Effect |
|---|---|---|---|
| `wait_states` | 0..15 | 0 | CTRLB.RWS, bits 4:1 (not 3:0) |
| `cache` | bool | true | CTRLB.CACHEDIS inverted; the main array only |
| `read_mode` | `no_miss_penalty`, `low_power`, `deterministic` | `no_miss_penalty` | what a cache miss costs |
| `sleep_power` | `wake_on_access`, `wake_on_exit`, `disabled` | `disabled` | CTRLB.SLEEPPRM; the reset value is `wake_on_access` |
| `manual_write` | bool | true | CTRLB.MANW; false lets a store to a page's last word commit the page |

`init<cfg>()` refuses an impossible configuration at compile time,
`init(cfg)` returns false and writes nothing. Impossible means a
sixteenth wait state (the field is four bits and would mask to zero,
leaving the flash too slow for the clock that asked) or SLEEPPRM's
reserved code 0x2.

**`NvmArray`** - `main` or `rwwee`, the argument that picks the command
code and the section ADDR counts from.

**`NvmError`** - `none`, `busy`, `timed_out`, `bad_address`,
`program_error`, `lock_error`, `nvm_error`. Returned by every command
verb; `none` is the only success.

**`NvmStatus`** - STATUS unpacked: `nvm_error`, `lock_error`,
`program_error`, `page_buffer_loaded`, `power_reduced`, `security_bit`.

**`Nvm`** - the block, monostate.

- *Geometry*, all compile-time constants taken from the device header:
  `page_size`, `row_size`, `pages_per_row`, `main_base`/`main_size`/
  `main_end`/`main_pages`, `rwwee_base`/`rwwee_size`/`rwwee_end`/
  `rwwee_pages`, `region_count`/`region_size`, `user_row`, `aux_base`,
  `software_calibration`, `temperature_calibration`. `base_of`, `end_of`,
  `in_array`, `addr_field` and `region_of` are the constexpr helpers.
- *Description*: `param` and its decoded `param_page_size`,
  `param_main_pages`, `param_rwwee_pages`; `geometry_matches` compares
  them with the header's constants, which is how a program checks it was
  built for the silicon it is running on.
- *Configuration*: `init`, `ctrlb`, `cache`, `cache_enabled`,
  `read_mode`, `manual_write`, `config_valid`.
- *Status and interrupts*: `ready`, `flags`, `armed`, `clear_flags`,
  `pending`, `arm`, `disarm`, `status`, `status_bits`, `clear_status`,
  `take_status` (read-and-clear), `security_bit`, `isr` (the handler
  body; the app binds the vector).
- *Commands*: `command` (key, wait, outcome), `outcome`, `address` both
  ways, `clear_page_buffer`, `invalidate_cache`, `enter_power_reduction`,
  `exit_power_reduction`, `power_reduced`.
- *Memory*: `erase_row`, `program_page`, `read`, `read_word`.
- *Protection*: `locks`, `region_locked`, `lock_region`, `unlock_region`.

**`FlashWaitStates`** - `get`, `set`, `for_hz`. The register is
NVMCTRL's, the question is the clock's: nothing may raise the CPU
frequency without setting these first, and 27.5.2 orders them adapted
BEFORE a rise and after a fall. `samc/clock.hpp` is the caller.

**The factory views** - `NvmUserRow`, `NvmCalibration`,
`NvmTemperatureCalibration`, `DeviceSerial`. Each has a static `read()`
and constexpr accessors, so a decoding can be asserted in a fixture
without hardware.

**`RwweePartition`** (`samc/nvm_flash.hpp`) - where the line between the
two storage classes is drawn, in one place. The 8 KB array's 32 rows
split as

| Rows | Bytes | Whose | |
|------|-------|-------|--|
| 0..27 | 7168 | `RwweeFlash` | blocks (`util/nv_heap.hpp`), its map pair in rows 26..27 |
| 28..31 | 1024 | `RwweeJournalZone` | small values (`util/nv_journal.hpp`), two 512-byte halves |

Both bounds are CONSTANTS, unlike every zone on the AVR side, because
nothing the linker places can reach into the RWWEE array - that is the
property the RWWEE choice bought and the partition does not spend it. The
split is anchored at the TOP of the array so that each user's own home is
in turn anchored to the silicon: the heap's map pair at the top of the
heap's share, the journal's halves at the top of the part.

**`RwweeFlash`** and **`RwweeJournalZone`** - the two FlashMedia
realizations over it: `erase_size` 256 and `write_cell` 64 on both,
`flash_end` 0x00401C00 and 0x00402000, one zone each,
`read`/`program`/`erase`/`build_id`. The journal zone delegates the
mechanics to `RwweeFlash` - same array, same commands - and differs only
in its bounds.

## How to use it

**Set the wait states before raising the clock** - which the clock code
already does, and is the reason this type exists:

```cpp
FlashWaitStates::set(FlashWaitStates::for_hz(48'000'000));
```

**Store a record without stopping the program.** This is the RWWEE
array's whole point: the erase and the write below cost about 989 us and
190 us, and the CPU keeps running through both.

```cpp
uint8_t page[Nvm::page_size] = { /* ... */ };
if (Nvm::erase_row(NvmArray::rwwee, Nvm::rwwee_base) == NvmError::none) {
    (void)Nvm::program_page(NvmArray::rwwee, Nvm::rwwee_base, page);
}
```

**Run a block allocator on it**, which is what an application usually
wants instead of raw pages:

```cpp
NvHeap<RwweeFlash, 8, 2> heap;
heap.mount();
if (auto w = heap.alloc(record_id, sizeof settings)) {
    w->append(std::span<const uint8_t>(bytes, sizeof settings));
    w->seal();
}
if (auto block = heap.find(record_id)) {
    block->read(0, std::span<uint8_t>(bytes, sizeof settings));
}
```

**Keep a handful of small values in the attic**, which is the other
storage class and the other partition - the two coexist over one array
and neither knows the other exists:

```cpp
NvJournal<RwweeJournalZone, 6, 32> journal;   // six ids of up to 32 bytes
journal.mount();
journal.save<Calibration>(cal_id, cal);
if (const auto back = journal.load<Calibration>(cal_id)) { /* ... */ }
```

**Read what the factory left**, before configuring a converter or an
oscillator that needs it:

```cpp
const auto cal = NvmCalibration::read();
// cal.adc0_biascomp(), cal.osc32k_calib(), cal.cal48m_3v3() ...
const auto id = DeviceSerial::read();   // all four words, or it is not unique
```

**Check the fuses rather than assuming them.** A bootloader area or an
EEPROM emulation area carved out of the main array changes what is safe
to write:

```cpp
const auto fuses = NvmUserRow::read();
const uint32_t writable_top = Nvm::main_end - fuses.eeprom_bytes();
const uint32_t writable_bottom = fuses.bootprot_bytes();
```

## Bench findings

From `test_samc_nvm` (52 verdicts in `z`, plus letter `m` outside it -
`m` costs main-array endurance and must be asked for by name). Nothing to
wire: everything under test is inside the chip.

- **ADDR is section-relative half-words**, confirmed from firmware and
  independently over SWD: a load at RWWEE + 0x100 leaves 0x80, one at
  main 0x20100 leaves 0x10080.
- **The page-buffer rule is "one 64-bit section at a time", not
  "ascending".** Loading a page ascending and descending both produce a
  byte-exact page; loading all even words and then all odd words leaves
  all eight even words ERASED and all eight odd words intact. That is
  27.6.4.3's own example generalized, and it says the condition is that a
  section's two words be consecutive - which ascending and descending
  both satisfy.
- **RWWEE row erase 989 us, page write 190 us**, stable to the
  microsecond across runs, against the 6 ms and 2.5 ms maxima of table
  45-42.
- **The stall is real and one-sided.** Through an RWWEE row erase the CPU
  completes ~3950 polling turns; through a main-array row erase, ONE.
- **A stalled operation cannot be timed by the software clock, and the
  reason is not the obvious one.** Single main-array erases measured
  between 234 and 1233 us across runs - a whole tick period of spread.
  The tempting explanation, that the SysTick handler misses ticks while
  its own code is unfetchable, is WRONG: eight consecutive erases report
  exactly eight milliseconds of ticks. What happens instead is that the
  stopwatch mixes the software tick counter with the hardware SysTick
  value, and in the moment right after a stall ends SysTick may already
  have wrapped while the handler has not yet run - a reading taken there
  comes out one tick short. The tick is late, not lost, and the damage is
  confined to one reading. Eight erases measure 7252..7822 us against the
  7912 us the same eight cost on RWWEE, which is the same operation timed
  where the CPU survives it.
- **Every malformed request is refused, not half-performed**: an
  unaligned page address, a partial page, an address past the array, a
  row erase at a page boundary, and a main-array address handed to the
  RWWEE array all return `bad_address` and leave the row intact.
- **Region locks work and are temporary.** All sixteen regions are
  unlocked out of reset on this board; locking region 15 makes LOCK read
  0x7FFF, an erase into it returns `lock_error` with the row untouched,
  and unlocking restores 0xFFFF. A reserved command code (0x07) raises
  STATUS.PROGE, and `take_status()` reports it and leaves the status
  clean.
- **`util/nv_heap.hpp` runs unmodified on this target.** It mounts on the
  heap's share of the RWWEE array, places a 200-byte block immediately
  below the two-row map home at the top of that share, as its top-down
  placement intends - seals it, reads it back byte-exact, and finds it
  again after a fresh mount that re-reads the map from flash. The
  concept's `erase_size`/`write_cell` split is what made this fit: the
  two numbers are 256 and 64 here against 512 and 2 on the AVR.
- **`util/nv_journal.hpp` runs unmodified on it too, in the attic beside
  the heap** (`test_samc_journal`, 58 verdicts in `z`, plus `p` and `v`
  outside it). The measurements:
  - a **bare RWWEE page program is 190 us** and a **journal save is 347
    to 357 us** over five, the difference being the journal's own work -
    building a 64-byte entry image and its BITWISE CRC-16 (`util/crc.hpp`
    trades flash for speed on purpose, and this is what that costs where
    the payload is small);
  - **a collection is 5.8 ms**: four row erases (both halves, two rows
    each) plus one page program per live id, and the meter confirms
    exactly that count;
  - the CPU is **not stalled**, measured the way `test_samc_nvm` letter
    `d` measures it - about 2500 polling turns inside one RWWEE row erase
    - and the software timebase keeps up with the cycle stopwatch through
    three consecutive collections (18 ms against 17.5 ms), which is the
    thing a main-array operation makes impossible;
  - **the panic reserve holds on silicon**: after every completed save
    `save_reserved()` of a maximum-size value succeeded and **cost no
    erase**, run against the real page program;
  - **mounting costs no erase and no program**, and takes 158 us for the
    two 512-byte halves;
  - **the coexistence is real**: a 300-byte heap block stays byte-exact
    at the same address while the journal collects repeatedly over it,
    and the heap re-mounts with nothing lost;
  - a **z run spends 168 to 184 row erases** in the four attic rows,
    about 46 cycles of each - which is the price of running the journal
    at the maximum `max_ids` the geometry allows, where a collection
    falls due at every second save.
- **The repartition is a one-time break of on-chip data, and the mount
  says so rather than hiding it.** Moving the heap's ceiling down to
  0x00401C00 moved its map home from rows 30..31 to rows 26..27, so the
  first boot after the change found no map, reported an empty heap, and
  left the old blocks unreadable - the survival-aware mount working as
  designed. The journal's first mount over the same rows found the old
  map bytes, recognized none of them as entries (a different magic), and
  treated them as dirty cells to collect past.
- **This board's fuses are at production defaults**: no bootloader area,
  no EEPROM emulation area, BODVDD level 8, watchdog off. The calibration
  area is programmed (OSC32K CALIB 65, CAL48M 3V3 2229025, 5V 2229024;
  ADC0 and ADC1 bias 7/6 each) and the die serial reads
  F9E78960-51574841-59202020-FF160321.
- **An unbound vector on this target is a silent death**, not a crash:
  the crt's default handler is a spin loop, so a suite that forgets
  `SERCOM5_Handler` simply never prints anything. (The AVR twin of this
  lesson is the opposite shape - an unbound vector there jumps to 0 and
  the suite reboots forever.) Diagnosed by halting over SWD and reading
  IPSR, which named external interrupt 14.

## Not covered yet

Driver gaps (not built):

- **The SSB command** (Set Security Bit). One-way - only a debugger chip
  erase clears it - and its entire effect is to lock the part against the
  debugger. `security_bit()` reads the state; nothing sets it. Same
  ruling that kept CHER and software fuse writes out of the AVR driver.
- **The two commands the device header carries and chapter 27's command
  table does not list**: SF (0xA, "Security Flow") and WL (0xF, "Write
  lockbits"). Undocumented, and WL is permanent.
- **A main-array FlashMedia backend, and this one is a RULING and not a
  gap.** The RWWEE array serves BOTH storage classes - blocks in rows
  0..27, small values in rows 28..31 - so nothing is waiting for room,
  and a main-array backend would import back the three things the RWWEE
  choice deleted: the stall (the CPU completes ONE polling turn inside a
  main-array row erase against about 2500 inside an RWWEE one), a quarter
  of the endurance (25k cycles against 100k, 45-43 and 45-44), and the
  AVR backend's linker-symbol zone arithmetic with the reflash preflight
  that goes with it. It is built when something genuinely needs more than
  8 KB of nonvolatile storage, and not before.
- **The EEPROM emulation area** as a first-class region: the driver reads
  its size out of the fuses but has no verbs that treat those rows
  specially (they are writable regardless of region lock).
- (Both once-wished neighbours now exist: `samc/dsu.hpp` carries the
  CRC32 engine - chip erase deliberately absent there too, the same
  ruling as this file's - and `samc/pac.hpp` the write protection;
  see [dsu.md](dsu.md) and [pac.md](pac.md). Nothing here uses either
  yet.)

Implemented but not bench-verified:

- The cache verbs (`cache`, `read_mode`, `invalidate_cache`): set and
  read back, never measured as a performance or coherence effect.
- Power reduction (`enter_power_reduction`, `exit_power_reduction`,
  `sleep_power`): the block reports PRM, but the sleep modes that give it
  meaning belong to the power pass.
- `Nvm::isr()` from a real handler - every command in the suite is
  awaited by polling READY, and the vector is deliberately unbound.
- Operation on the E and G variants, and on the 15/16/17 flash sizes:
  compile-checked only. The geometry comes from the device header, so a
  smaller part is expected to work, and `geometry_matches()` is the
  runtime check that would catch it not doing so.
