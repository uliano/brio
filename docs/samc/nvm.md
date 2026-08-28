# NVMCTRL - Non-Volatile Memory Controller (SAM C21)

> **PROVISIONAL.** The chapter's programming, protection and description
> surfaces are built and bench-verified; what is deliberately left out is
> either provisioning (writing the fuse row), a one-way hazard (the
> security bit) or waits for another driver. The list is in "Not covered
> yet".

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
`samc/nvm_flash.hpp` (`RwweeFlash`, the `util/nv_heap.hpp` backend).
Family fixture `test/family_samc/nvm.cpp` plus two negatives under
`tools/check_samc.sh`; the bench suite is `test_samc_nvm`.

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

**`RwweeFlash`** (`samc/nvm_flash.hpp`) - the FlashMedia realization:
`erase_size` 256, `write_cell` 64, `flash_end` 0x00402000, one zone
covering the whole array, `read`/`program`/`erase`/`build_id`. The zone
is a CONSTANT, unlike every zone on the AVR side, because nothing the
linker places can reach into the RWWEE array.

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
  RWWEE array, places a 200-byte block at 0x401D00 - immediately below
  the two-row map home at the top of the array, as its top-down placement
  intends - seals it, reads it back byte-exact, and finds it again after
  a fresh mount that re-reads the map from flash. The concept's
  `erase_size`/`write_cell` split is what made this fit: the two numbers
  are 256 and 64 here against 512 and 2 on the AVR.
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

- **Writing the NVM User Row** (the EAR and WAP commands). On this family
  the user row IS the fuses, it survives a chip erase, and it carries the
  watchdog's power-on ALWAYSON bit and the brown-out level - so a wrong
  word is not undone by reflashing. It is read and typed here; writing it
  is provisioning, which on the AVR side lives in the bench tool and goes
  over UPDI, and which here wants a `tools/bench.py` verb over SWD. That
  tool currently refuses `fuses` on a SAM board and says why.
- **The SSB command** (Set Security Bit). One-way - only a debugger chip
  erase clears it - and its entire effect is to lock the part against the
  debugger. `security_bit()` reads the state; nothing sets it. Same
  ruling that kept CHER and software fuse writes out of the AVR driver.
- **The two commands the device header carries and chapter 27's command
  table does not list**: SF (0xA, "Security Flow") and WL (0xF, "Write
  lockbits"). Undocumented, and WL is permanent.
- **A main-array FlashMedia backend.** `RwweeFlash` is the only one, and
  8 KB is its ceiling. A main-array heap would need the AVR backend's
  linker-symbol zone arithmetic and would stall the CPU on every write;
  it waits for something that needs the room.
- **The EEPROM emulation area** as a first-class region: the driver reads
  its size out of the fuses but has no verbs that treat those rows
  specially (they are writable regardless of region lock).
- **A DSU driver** (ch. 13) for chip erase and the CRC32 engine, and a
  **PAC driver** (ch. 11) for the write protection these registers
  advertise.

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
