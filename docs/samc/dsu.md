# DSU - Device Service Unit (SAM C21)

> **PROVISIONAL.** Everything chapter 13 offers to code running ON the
> device is built; what a debug PROBE uses it for is not, and chip erase
> is deliberately absent. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 13 - and
errata DS80000740S, which has **no DSU section**. The one item touching
this chapter's subject matter is **1.8.15 (Device)**: the Program and
Debug Interface Disable lock of 13.10 is not available on E/G/J silicon
up to and including revision F, a feature this driver does not build in
any case. Driver: `samc/dsu.hpp`. Family fixture
`test/family_samc/dsu.cpp` plus two negatives under
`tools/check_samc.sh`; the bench suite is `test_samc_debug`.

The DSU is the block a debug probe talks to before the CPU is running.
Three of its services are useful to the CPU itself, and those are what
this driver builds: **device identification**, a **hardware CRC32 over
any memory the bus matrix reaches**, and **MBIST**, the March LR memory
self-test IEC 60730 Class B asks for.

## What the silicon does

**It comes out of reset PAC write-protected**, alone among the
peripherals of this device (PAC.STATUSB reset value 0x00000002; table
12-3's "Prot at Reset" column has one Y and it is on this row). A bare
store to DSU.ADDR therefore does nothing and raises PAC.INTFLAGB.DSU
instead. `init()` clears the protection through `samc/pac.hpp` and
`release()` puts it back exactly as reset left it.

**One shared command interface.** ADDR, LENGTH and DATA are shared by
CRC32 and MBIST (13.12.1): configure them, then issue a command by one
write to CTRL. While a command runs, further commands are DISCARDED, so
every caller must wait for STATUSA.DONE - which the verbs do rather than
leaving it to the caller. CTRL.SWRST is the only way to cancel one.

**ADDR carries a byte address in bits 31:2 and AMOD in the low two.**
AMOD is one two-bit field with two meanings, one per command (tables 13-3
and 13-5), and zero is what a CPU-side caller wants either way.

**LENGTH's field is a word count in bits 31:2**, so the register value is
four times it - a byte length. Measured: it is also a working COUNTER,
not a latched configuration (see "Bench findings").

**CRC32 is the industry-standard reflected 0xEDB88320.** Seed DATA with
0xFFFFFFFF and complement the result to match any host-side tool; pass a
previous NON-complemented result as the seed to chain ranges. 13.12.3.2
requires STATUSA.BERR to be checked afterwards, and a checksum taken over
a bus error is not a checksum.

**MBIST destroys what it tests.** Phase 0 of the March LR algorithm
writes the whole range to zero and the test ends having written it again;
there is nothing anywhere that saves and restores. On a failure the
engine reports the failing word in ADDR and, in DATA, the failing bit
index and the phase (table 13-4). AMOD 1 pauses at the first failure
instead of exiting.

**Two address ranges.** 13.9: the first 0x100 bytes of the register map
are mirrored at 0x100, so the silicon can tell a CPU access from a
debugger one and restrict the latter when the device is protected by the
NVM security bit. This driver uses the INTERNAL range throughout, which
13.12.2.3 is explicit is right for code on the CPU.

**DID identifies the die**, in the fields every errata row and every
device-pack lookup is keyed by: processor, family, series, die, revision
and device-select. Revision 0 is rev A, so 5 is rev F.

**The CoreSight ROM at offset 0x1000** carries the conceptual 64-bit
peripheral ID that identifies the part as a SAM device implementing a DSU
(PARTNUM 0xCD0), plus two entries pointing at the device's CoreSight
components.

**Two debug communication channels**, DCC0 and DCC1: 32-bit mailboxes
readable and writable from both sides even under the security bit, each
with a dirty bit that sets on write and clears on read. 13.12.4 warns
they are shared with the MBIST logic and must not be used while a memory
test runs.

**Two registers chapter 13 does not describe.** The device header
declares `DSU_STATUSC` at offset 0x03 (a three-bit STATE field) and
`DSU_DCFG[2]` at 0xF0; 13.13's register summary prints "Reserved" at 0x03
and nothing at 0xF0. Both are exposed as raw reads with no interpretation
- the header is the authority on what exists, the data sheet on what it
does, and here the data sheet says nothing.

**No interrupt, no events** (13.5.5 and 13.5.6 are both "Not
applicable"). STATUSA.DONE is polled.

## Types and verbs

`DsuStatus` - the STATUSA bits by name: `done`, `crstext`, `bus_error`,
`fail`, `protection_error`, `all`. Every one is write-one-to-clear.

`DsuAccessMode` - ADDR.AMOD, keeping both of the chapter's namings:
`array_or_exit_on_error` (0), `eeprom_or_pause_on_error` (1).

`DsuDeviceId` - DID decoded: `raw`, `processor`, `family`, `series`,
`die`, `revision`, `devsel`, and `revision_letter()`, which is how the
errata matrix spells it.

`DsuMbistResult` - `done`, `failed`, `bus_error`, `protection_error`, and
on a failure `address`, `data`, `phase`, `bit_index`.

`Dsu` - the monostate resource.

- `pac_id` (33), `regs()`.
- Clocks and protection: `bus_clock` (AHB and APB, both on at reset),
  `init()` (clocks on, protection off - conditionally, because a
  redundant clear is itself an error), `release()` (back under
  protection).
- Identity: `did_raw`, `device_id`.
- Status: `status`, `clear_status`, `done`, `bus_error`, `failed`,
  `protection_error`, `cpu_reset_extended`, `release_cpu_reset`,
  `status_b`, `device_protected`, `debugger_present`,
  `hot_plugging_enabled`, `dcc_dirty`, and the two undocumented reads
  `status_c_raw`, `dcfg_raw`.
- The shared interface: `set_address`, `set_length_words`,
  `address_raw`, `length_raw`, `data` (read and write), `range_valid`
  (constant-evaluable: non-empty and word-aligned), `reset_module`,
  `wait_done`.
- Commands: `crc32` (complemented, the standard answer), `crc32_raw`
  (for chaining), `mbist`.
- The CoreSight ROM: `rom_entry`, `rom_entry_present`,
  `rom_entry_offset`, `rom_end`, `rom_memtype`, `rom_pid`, `rom_cid`,
  `rom_partnum`.

## How to use it

Identify the board:

    Dsu::init();
    const auto id = Dsu::device_id();
    print(sink, "rev ", id.revision_letter(), " devsel ", hex(id.devsel));

Checksum the application image (against a value a host computed the same
way):

    Dsu::init();
    if (const auto crc = Dsu::crc32(0x0000, image_words)) { ... }
    Dsu::release();

Chain two ranges into one checksum:

    const auto a = Dsu::crc32_raw(0x0000, n);
    const auto b = Dsu::crc32_raw(0x4000, m, *a);   // seed with the raw result
    const uint32_t whole = ~*b;

Test a block of SRAM the program has finished with:

    const auto r = Dsu::mbist(reinterpret_cast<uint32_t>(buf), words);
    if (r.done && !r.failed) { ... }        // buf is destroyed either way

## Bench findings

`test_samc_debug` letters d, e and f on the ATSAMC21J18A rev F, wireless.

**The board is the board.** DID reads **0x11010500**: PROCESSOR 1
(Cortex-M0+), FAMILY 2 (5V Industrial - the SAM C), SERIES 1 (the
CAN-bearing series, so a C21 and not a C20), DIE 0, REVISION 5 = **rev
F**, DEVSEL 0x00. The factory 128-bit die serial `samc/nvm.hpp` reads
comes back as **f9e78960-51574841-59202020-ff160321**, matching the
string `tools/bench_boards.py` records for desk position C. The CoreSight
PARTNUM is 0xCD0, table 13-2's "a DSU is present".

**The CRC32 engine is the standard CRC-32**, matched against a table-free
bitwise reference over the same bytes at 16, 256 and 1024 words of flash
and over 1 KB of SRAM this suite filled itself; flipping one byte changes
the answer. Chaining works: two halves seeded from one another equal the
whole. A checksum over an unmapped address (0x50000000) sets STATUSA.BERR
and the driver refuses to return it.

**LENGTH is a counter, which 13.14.5 never says.** Written for 256 words
it reads back 0x400 - the byte count, the field being at bits 31:2 - and
after the command it reads **zero**: the engine consumed it.

**The engine costs 6 cycles per word** (7024..7101 cycles for 4096 bytes
at 48 MHz), against **387 cycles per word** for the bitwise software
reference - about **55x**.

**MBIST costs 1165 cycles per word** (74567 cycles for 64 words) and
passes this SRAM in both exit-on-error and pause-on-error modes. **It
really destroys the range**: a buffer filled with 0xDEADBEEF reads zero
afterwards. Over an unmapped address it reports DONE with BERR set, so a
bus error cannot be mistaken for a pass.

**The debug status, printed and mostly not judged**, because what a probe
did before the program started is not something the program can arrange:
STATUSB reads 0x12 - PROT clear (the device is not locked by the security
bit, which is why every service here is available), DBGPRES set, HPE set,
CRSTEXT clear. DBGPRES is never cleared once set, so it is a
since-power-on fact and not a now fact.

**DCC0 holds what is written to it and its dirty bit behaves as
13.12.4 says**: set by the write, cleared by the read.

**The CoreSight ROM's two entries name the device's two debug
components.** Resolved against the table's own base (DSU + 0x1000),
ENTRY0 is **0xE00FF000** - the Cortex-M0+'s own ROM table - and ENTRY1 is
**0x41008000**, THE MTB. So chapter 13 and section 10.3 are one debug
system, and it is the DSU's ROM that tells a probe the trace buffer is
there. Both entries read EPRES = 1 and both point at real components,
which settles 13.14.10's two contradictory sentences (they both begin "if
the device is not protected"): **EPRES = 1 means present**. CID0..CID3
read 0x0D / 0x10 / 0x05 / 0xB1 - the architected CoreSight preamble with
component class 1, ROM table. MEMTYPE reads 1.

**The two undocumented registers, on this die:** STATUSC reads 0x00, and
DCFG0/DCFG1 read **0xFFFB5900** and **0xFFFFF13F**.

**The protection round trip.** `release()` leaves the DSU protected as
reset does; a write to ADDR is then dropped AND flagged in
PAC.INTFLAGB.DSU; `init()` takes the protection off again.

## Not covered yet

Driver gaps - features of ch. 13 not built:

- **Chip erase.** CTRL.CE erases the whole flash array including the
  EEPROM emulation area and clears the security bit. A firmware-callable
  verb for it would be a verb for destroying the running program; the AVR
  half of this framework made the same call about NVMCTRL's CHER.
- **Everything that is a probe's business rather than the CPU's**: the
  DAP security filter, cold- and hot-plugging as procedures, programming
  through the AHB-AP, and the external address range at 0x100..0x1FFF.
  `STATUSA.CRSTEXT` has a clearing verb but code that can call it is by
  definition already running.
- **The DMA connection of the debug communication channels** (13.5.4),
  which needs a peer on the other side to be worth anything.
- **The Program and Debug Interface Disable of 13.10**, which erratum
  1.8.15 says is not on this silicon anyway.

Implemented but not bench-verified:

- **MBIST's failure path.** Every run on this board passes, so the phase
  and bit-index decoding of figure 13-6, and pause-on-error's
  resume-by-clearing-FAIL, have never been exercised against a real bad
  bit. Nothing on this bench can produce one.
- **A CRC32 seeded from a debugger's value**, and the whole external
  range's AMOD behaviour (13.12.3's ARRAY and EEPROM restrictions), which
  only apply to accesses this code cannot make.
- **Sleep.** 13.5.2 says the DSU keeps operating in Idle; nothing has
  slept with a command running.

Open, and outside this driver:

- **What STATUSC.STATE and DCFG0/1 mean.** They exist, they read
  non-trivial values, and no document in this project describes them.
