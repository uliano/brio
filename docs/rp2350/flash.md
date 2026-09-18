# Flash, the QMI and the XIP cache (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.14 (the
QSPI memory interface: the two memory windows with their timing,
transfer formats and command constants, the four address translation
panes a window, and direct mode), 4.4 (execute-in-place: the four
aliases, the cache and its maintenance window, the streaming interface,
the two performance counters), 4.3 (boot RAM, where the XIP setup
function lives), 5.4.8.3 and 5.4.8.6 to 5.4.8.14 (the bootrom's flash
functions and the two ROM data pointers beside them), and 5.2.7 for how
the bootrom leaves the interface set up before it enters an image.
Appendix E carries one erratum this chapter answers, RP2350-E11, and one
it only states, RP2350-E14. The drivers: `brio/rp2350/flash.hpp` over
`bootrom.hpp`, and `brio/rp2350/nvm_flash.hpp` over that. The reference
suite: `test_rp2350_flash`.

## What the silicon does

THE QMI IS NOT THE RP2040'S SSI RENAMED. It is a new block with two
16 MB MEMORY WINDOWS, one per chip select, and each window carries its
own configuration: a timing register (the clock divisor, the sample
delay, the chip select setup, hold, maximum assertion and minimum
deassertion, a cooldown that lets a following access join the transfer
in flight, and a page break that forbids a burst crossing 256, 1024 or
4096 bytes), a READ format and a WRITE format, and a pair of command
constants for each. A transfer has five phases - prefix, address,
suffix, dummy, data - and EACH PHASE HAS ITS OWN BUS WIDTH, serial, dual
or quad, which is how one register file describes a 03h serial read and
an EBh quad-I/O read alike. The prefix and the suffix are eight bits or
absent; the dummy phase is a count of four-bit units, 0 to 28 bits; and
one bit turns the whole thing into a double-transfer-rate transaction.

ADDRESS TRANSLATION IS PART OF THE MEMORY MAP (12.14.4). Each window is
four PANES of 4 MB, and each pane has a physical BASE and a SIZE, both
counted in 4 kB units. The reset state is the identity: pane n based at
n times 4 MB, 4 MB wide. A pane's SIZE is a bound, not a hint - an
access past it returns a bus error and never reaches the device - and
the mapping wraps modulo the window's 16 MB, which is what the bootrom's
documentation calls a rolling window. The translation sits DOWNSTREAM of
the cache, so the cache is virtual with respect to it and a change of
any pane needs a flush.

DIRECT MODE DISCONNECTS BOTH WINDOWS. One CSR carries the enable, a busy
flag, the two FIFOs' levels and flags, direct mode's own clock divisor
and sample delay, and four chip select controls: two that drive a select
low unconditionally - EVEN WITH THE ENABLE CLEAR, which 12.14.5.2 warns
about in as many words - and two that let the block drive it for as long
as it is busy. A push into DIRECT_TX carries control bits above the
data: the width, eight or sixteen data bits, the output enable for the
bidirectional widths, and NOPUSH, which throws away the answer that
record would otherwise put in the receive FIFO. All control bits zero is
a plain eight-bit byte, so the FIFO can be used as one. THE RECEIVE FIFO
NEVER DROPS A BYTE: a full one stalls the interface instead, which means
a caller may push freely as long as it also pops - and must never wait
for the busy flag without also popping, or the two block each other.

THE CACHE IS 16 kB, two-way, eight-byte lines, one cycle on a hit, and
what it caches is a 26-bit downstream space mirrored four times in the
system map: **cached** at 0x1000_0000, **uncached** at 0x1400_0000,
**maintenance** at 0x1800_0000, and **uncached and untranslated** at
0x1c00_0000. The QMI occupies the lower half of the downstream space
(two windows of 16 MB); the upper half is reserved, and is where a
pinned line can be parked without ever aliasing a real address.

MAINTENANCE IS AN ADDRESS AND NOT A REGISTER. There is no flush bit
here: a write into the maintenance window maintains ONE LINE, and the
three least significant bits of the address say which of five
operations - invalidate by set and way, clean by set and way,
invalidate by address, clean by address, and PIN, which marks a line
so it is never evicted and is this chip's cache-as-SRAM. A pin copies
nothing: it marks the line and writes its tag, and the data has to be
fetched afterwards through the uncached alias. The tag memory lives in
the XIP power domain, so pinned lines survive a processor reset. The
cache has two enables, Secure and Non-secure, a power-down that forces
both clear, a split that gives each security level one way, a bit that
hands maintenance to Non-secure code, four that turn an alias into a bus
error, and TWO THAT MAKE A WINDOW WRITABLE - clear by default, because a
store into a read-only device would appear to succeed in the cache and
then, on eviction, issue a write command that can leave the chip
returning garbage.

**Erratum RP2350-E11 is live on stepping A2 and on A3 and A4.** A clean
by set and way also OVERWRITES THE CLEANED LINE'S TAG with bits 25:13 of
the maintenance write's own address. A sweep driven from the bottom of
the maintenance window would therefore leave every cleaned line claiming
to hold address zero, and the next read of the flash's first bytes would
hit it. `Xip::clean_all()` drives the sweep from the TOP of the window
instead, so the tag it leaves behind names the reserved upper half of
the downstream space, which no QMI access can ask for; the cost is a
miss on the next access to a cleaned address, which is what a clean is
for. The other four operations are unaffected.

THE STREAMING INTERFACE (4.4.3) is a linear run of words the XIP
subsystem fetches in the BACKGROUND, at lower priority than the
program's own accesses and with a nine-cycle cooldown after each cache
miss, into a small FIFO whose DREQ paces a DMA channel. The auxiliary
AHB port gives single-cycle access to that FIFO and to the QMI's two
direct-mode FIFOs, where the APB configuration port takes several
cycles.

### The flash chip, and what the bootrom leaves behind

THERE IS NO SECOND STAGE ON THIS CHIP. The RP2040's first 256 flash
bytes were a checksummed stage whose job was to set the flash interface
up, and whose second job was to be the way back after a flash operation.
Here the bootrom does the setup itself while scanning the flash: it
tries sixteen combinations in turn - EBh, BBh, 0Bh and 03h reads at SCK
divisors 3, 6, 12 and 24 (5.2.7's table 452) - keeps the first that
works, and writes a position-independent XIP SETUP FUNCTION into the
first 256 bytes of BOOT RAM that restores it. Boot RAM is 1 kB on the APB and is physically not
executable, so a program that wants that function must COPY IT INTO SRAM
and call the copy. THE DATASHEET GIVES TWO ROUTES TO THAT ONE ADDRESS -
the ROM data entry 'X','F' names it, and 4.3 and 5.2.7 say it is at the
BASE of boot RAM - so this driver takes the named one when it points
into boot RAM and the base otherwise, and reports which route answered.
Only the first is a promise the ROM keeps across releases.

AND THE WAY BACK IS NOT LOAD-BEARING FOR CORRECTNESS, which is the other
half of the difference from the RP2040. 5.4.8.6 states it: the ROM's
`flash_exit_xip()` sets a basic 03h read mode up as it goes, and the
program and erase functions do not leave XIP inaccessible. So after a
window the flash is READABLE whatever happened; the setup function only
restores the FAST mode. That is why this driver has two ways back and
reports which it took.

THE BOOTROM'S FLASH FUNCTIONS are the same six the RP2040 had, under the
same two-character codes - 'I','F' to reconnect the pads and the QMI,
'E','X' to put the device back in a serial command state, 'R','E' and
'R','P' to erase and program, 'F','C' to flush the cache, 'C','X' for
the slow read mode - plus, new here, 'R','A' to reset the translation
panes, 'X','M' to pick a read mode and 'F','A' to translate a runtime
address into a storage address. They are Secure-Arm and RISC-V both,
found through the per-architecture lookup `bootrom.hpp` does, and none
of them is among the four that 5.4.8.26 says want extra stack on
RISC-V. Beside them sit two ROM DATA pointers, each a pointer TO a
pointer: 'X','F' for the setup function and 'F','D' for the runtime
copy of FLASH_DEVINFO in boot RAM.

FLASH_DEVINFO is what the ROM's higher-level entry points bound their
addresses against, and on a board that has programmed no OTP it is the
ROM's own default: 16 MB on chip select 0, nothing on chip select 1, no
secondary chip select pad, and **no D8h block erase** - which is a
statement about what the ROM will do of its own accord, not about what
the chip can do. This driver reads it and never writes it.

**Erratum RP2350-E14** says that on A2 silicon `connect_internal_flash()`
ignores the chip select 1 pad FLASH_DEVINFO names and always uses GPIO 0.
It bites only a board that has a device on chip select 1; nothing in brio
writes that field, and the bench board has nothing there.

WHAT A WINDOW COSTS, AND WHAT IT ASKS. While the QMI is in direct mode
the windows are gone: an access to them is a bus error, for the
processor, for the debugger and for the other core alike (5.4.8.10). So
every operation runs from `.ram_text` - an input section the linker
script folds into .data, copied to SRAM by the crt - with this core's
interrupts masked for the whole of it. The rest cannot be enforced from
here and is therefore stated: THE OTHER CORE MUST NOT FETCH OR READ
FLASH while a window is open, and no DMA channel may have the window as
a source. A buffer of the caller's that lies in the XIP space is
refused.

AND THE ROM'S CACHE FLUSH IS AN INVALIDATE, not a clean: it drops every
line, dirty ones included. That matters only to a program that has made
a memory window WRITABLE - which only a RAM on the QSPI bus makes sense
of - so every window here cleans the cache first, and only when one of
the two writable bits is set. The clean is a maintenance write and not a
QSPI access, so it happens before the window opens, out of flash like the
rest of the driver.

TWO MORE THINGS ARE SAVED ACROSS A WINDOW, and neither was the RP2040's
concern. `connect_internal_flash()` puts every QSPI PAD CONTROL back to
its reset state, and `flash_exit_xip()` rewrites WINDOW 1's timing, read
format and read command as well as window 0's. This driver snapshots the
six pad registers and window 1's trio and writes them back - the trio
only when FLASH_DEVINFO says nothing is attached to chip select 1, which
is the only case in which the old configuration is still the right one.

THE RAW COMMAND VERB READS AND NOTHING ELSE. The opcodes it will clock
out are an ALLOW-LIST, every one of them a read: 03h, 0Bh, 05h, 35h,
15h, 5Ah, 9Fh, 4Bh and 90h. There is no write enable, no status or
configuration register write, no erase and no page program in it. The
reason is not squeamishness: those registers are one-way on a good many
QSPI parts, and a quad-enable, a lock bit or a four-byte-address mode set
by accident is a board that no longer boots. The erase and program this
driver does offer go through the bootrom's own functions, which take an
address and a length and never an opcode.

### The storage partition

The linker script stops the `flash` region 64 kB short of the chip
(`rp2350/ld/rp2350_16m.ld`, 16320K of 16384K), so nothing the compiler
emits can land in the top 64 kB and the storage's floor is a CONSTANT
rather than a symbol that moves with every build. `__brio_flash_end` is
read back as the proof, and a script edited past the line CLOSES the
medium rather than letting an image grow into its own storage.

    0x00'0000 .. 0xff'0000   the image (the linker's)
    0xff'0000 .. 0x100'0000  QspiFlash, 64 kB: sixteen sectors of 4096

THE CELL IS THE PAGE: the bootrom's program takes whole 256-byte pages,
so the medium's write cell is 256 under an erase size of 4096.

NOTHING STANDS ON THE CONTRACT HERE. Neither the block heap nor the
value journal is instantiated on this family, by the same decision the
CH32V203 records: the NV stack is under review as a whole, and the
question of which programs need a block allocator, which need small
values, and whether a zone at a fixed address is worth a partition every
image pays is prior to any port. What a program gets here is a band of
flash with bounds it cannot write outside of.

## Types and verbs

`Qmi` is the block's shared half: direct mode's enable and busy flag,
both FIFOs' levels and flags, direct mode's own divisor and sample
delay, a push that takes a raw word or a described frame, a pop, the
four FIFO addresses a DMA channel may be pointed at (the APB pair and
the auxiliary AHB pair), a READ-ONLY view of the six QSPI pad controls,
and the verb that puts all eight translation panes back to the identity -
eight stores this file makes itself rather than calling the ROM's
'R','A', and a verb a program whose own code is reached through a
translated pane must not call, because the ground it runs on would move.

`QmiWindow<0|1>` is one memory window: its timing decoded as `QmiTiming`
(and a divisor-only setter, the one change 4.4 allows while the
interface runs), its read and write formats as `QmiTransferFormat`, its
two command pairs as `QmiCommand`, its four panes as `QmiTranslation`
(by BYTES, never by the register's 4 kB units), a translation read off
the live registers, an identity check, and its chip select in both of
the chapter's arrangements. A third window is a compile error, and so is
a fifth pane.

Beside them the chapter's arithmetic as pure functions, which is what a
test judges the silicon against: `qmi_direct_word`, `qmi_timing_word`
and `qmi_timing_from`, `qmi_format_word` and `qmi_format_from`,
`qmi_command_word` and `qmi_command_from`, `qmi_atrans_word` and
`qmi_atrans_from`, `qmi_identity_pane`, `qmi_sck_hz` (the divisor with
zero meaning 256) and `qmi_translate`.

`Xip` is the cache: the two enables together or apart, the power-down,
the split ways (whose setter does the flush the change requires, in the
order it requires), the Non-secure maintenance bit, the four alias
refusals, the two window-writable bits as a template verb, the two
performance counters, the five maintenance operations - all lines
invalidated, all lines cleaned the E11-safe way, one address
invalidated or cleaned, a range invalidated, one address pinned - and
the streaming interface: start, stop, the remaining count, the FIFO's
flags, a pop, and the FIFO's two addresses.

`Flash` is the engine: `init()` finds the ROM's functions and copies the
setup function, `xip_restore_kind()` says which way back this image got,
`device_info()` decodes the runtime FLASH_DEVINFO, `runtime_to_storage()`
is the ROM's own translator, `address()` / `uncached_address()` /
`untranslated_address()` are the three readable aliases, `read()` is a
copy through the cache, `erase()` takes a grain (`FlashEraseGrain`) that
says whether the 64 kB block command may be used, `program()` takes
whole pages from SRAM, `commit_writes()` is the clean every window does
for itself, and `command()` is the raw read - with the opcode
as an argument, refused at run time, or as a template argument, refused
at compile time. Over it `jedec_id()`, `unique_id()`,
`status_register<1|2|3>()` and `read_sfdp()`.

`QspiFlashPartition` states the geometry once and answers
`geometry_matches_silicon()`, plus two constexpr predicates a program
can pin a fixed address with. `QspiFlash` is the `FlashMedia` over it.

There is no interrupt in this chapter: neither the QMI nor the XIP
subsystem has a line.

## How to use it

Read the chip's identity, and learn early whether a window is possible
at all:

```cpp
if (brio::Flash::init()) {
    const auto id = brio::Flash::jedec_id();            // 9Fh
    const auto uid = brio::Flash::unique_id();          // 4Bh
    const auto st = brio::Flash::status_register<1>();  // 05h
}
```

Erase and program the storage partition. The source must be in SRAM, and
the caller owns the duty of keeping the other core off the flash:

```cpp
uint8_t page[brio::QspiFlash::write_cell];              // SRAM, always
const auto zone = brio::QspiFlash::zones()[0];
brio::QspiFlash::erase(zone.floor);                     // one sector
brio::QspiFlash::program(zone.floor, page);             // one page
```

A whole region at once, letting the bootrom take the larger erase
command where the run allows it:

```cpp
brio::Flash::erase(zone.floor, zone.ceiling - zone.floor,
                   brio::FlashEraseGrain::sector_or_block);
```

Read what the chip holds rather than what the cache remembers, and put
the cache back in step by hand when something outside the driver has
changed the flash:

```cpp
const uint8_t* fresh = brio::Flash::uncached_address(offset);
brio::Xip::invalidate_range(offset, bytes);
```

Ask what the QMI was left configured for, and what that costs:

```cpp
const auto timing = brio::QmiWindow<0>::timing();
const uint32_t sck = brio::qmi_sck_hz(SysClock::hz, timing.clkdiv);
const auto mode = brio::QmiWindow<0>::read_command().prefix;   // 03h, 0Bh, BBh or EBh
```

Profile the cache over a piece of work:

```cpp
brio::Xip::reset_counters();
do_the_work();
const uint32_t percent = 100u * brio::Xip::hits() / brio::Xip::accesses();
```

Stream a run of flash words in the background and take them by hand (a
DMA channel takes them on `Dreq::xip_stream` instead):

```cpp
brio::Xip::stream_start(brio::Flash::window + offset, words);
while (const auto w = brio::Xip::stream_pop()) {
    consume(*w);
}
brio::Xip::stream_stop();
```

Read a raw command with the opcode fixed at compile time, which is where
a write opcode is refused:

```cpp
uint8_t sfdp[8];
brio::Flash::read_sfdp(0, sfdp);                        // 5Ah
brio::Flash::command<0x02>({}, sfdp);                   // COMPILE ERROR: 02h writes
```

## Not covered yet

Driver gaps, each with its reason:

- **Reconfiguring a window's format, command or width.** Every setter is
  there and every encoding is checked at compile time, but nothing here
  performs the sequence a live change needs: dropping into `.ram_text`,
  masking interrupts, writing the format, the command and the divisor,
  and issuing whatever the DEVICE needs to agree (a quad enable, a
  continuous read prefix). That sequence is a FLASH CHIP'S, not this
  block's, and it wants a board file that names the chip - which this
  project does not have yet. A program that wants the flash faster today
  gets what the bootrom found.
- **A device on chip select 1.** The second window has every verb the
  first has, and the driver refuses to restore its configuration across
  a window when FLASH_DEVINFO says something is attached to it, because
  the right restore for a PSRAM is that device's own setup function.
  There is no such device on the bench board, and the writable-window
  bit, the write format and the write command that only a RAM makes use
  of are therefore compiled and never exercised. Erratum RP2350-E14
  belongs to this gap.
- **Cache-as-SRAM.** `pin_address()` is the chapter's operation and
  nothing more: pinning a useful amount of cache, filling it through the
  uncached alias and handing it to a program as memory is a facility,
  not a register, and it waits for a program that needs it.
- **The bootrom's higher-level flash entry point** (`flash_op`, 'F','O')
  and the partition table functions around it. It duplicates the bounds
  and alignment checks this driver already makes, and adds a dependency
  on a resident partition table that no image here carries; the two
  low-level functions are what the RP2040's document describes and what
  this one can state exactly.
- **The block heap and the value journal** over `QspiFlash`. Declined
  for this family by the NV stack's own review (CLAUDE.md), which is
  prior to a port; the medium and its bounds are what a program gets.
- **The QFN-60 package.** Nothing in this chapter is bonded differently
  - the QSPI pads are not bank 0's - but no QFN-60 part is on the bench,
  so the statement is a compile check and not a measurement.

Implemented but not bench-verified - the whole of this chapter, which
was written ahead of the bench; each item names the letter of
`test_rp2350_flash` that will measure it:

- **The bootrom's six flash functions found on both architectures**, and
  which way back into execute-in-place an image gets - letter **a**,
  which also reads the chip's identity, its three status registers, the
  SFDP signature and the runtime FLASH_DEVINFO.
- **That a window leaves the QMI where it found it** - letter **b**,
  which reads both windows' whole configuration, runs a window that
  writes nothing, and reads them again, the QSPI pad control included.
  It is the measurement that decides whether the copied setup function
  works at all.
- **The four aliases, the cache's counters and its maintenance** -
  letter **c**: the same bytes through three aliases, the hit ratio of a
  sector read twice, one line invalidated by address and the miss that
  follows, and both full sweeps timed. The clean sweep is also E11's own
  test: the program that issued it must still be able to read the
  flash's first word.
- **The translation arithmetic against the bootrom's own translator** -
  letter **d**, over a hundred addresses on the identity map, then a
  pane the image does not use given a rolling map and restored. It also
  settles one thing the chapter leaves open: WHICH BASE the ROM's
  translator answers in. 5.4.8.13 says "the storage address", which
  reads like an offset in the chip, while 5.4.8.9 expresses every flash
  address from the window's base - so the letter asks for an address it
  knows the answer to, prints which of the two came back, and judges
  everything after it against that.
- **The streaming interface** - letter **e**, whose words are judged
  against the same addresses read through the window. Streaming into a
  DMA channel on `Dreq::xip_stream` is not measured: that wants the DMA
  chapter's engine slots and a second suite's worth of wiring.
- **What a window costs with nothing written** - letter **f**: a raw id
  command timed, the kernel ticks it eats, and a console line written
  across one.
- **Every refusal** - letter **g**: the alignment and bounds checks, a
  source in the XIP space, a write opcode through the run-time face of
  the allow-list, and the medium's own floor and ceiling. Nothing in it
  reaches the chip.
- **A sector erase and a page program, with their durations and the
  ticks they cost** - letter **h**, outside `z` because it spends three
  erase cycles across two sectors. It also proves the cache coherent
  with the chip after a program, and - with a marker in the NEXT sector
  - that the default erase grain really issues the 4 kB command and
  nothing wider, which is the one thing about the block size this driver
  hands the bootrom that reading the datasheet cannot settle.
- **What a page holds when it is programmed twice between erases** -
  letter **i**, outside `z`, one erase cycle. The expectation is the AND
  of the two programs, which is the physics the chapter does not state.
- **Whether the 64 kB block command is worth offering** - letter **w**,
  outside `z`, two erase cycles: the partition erased once with the 4 kB
  command alone and once with the block command offered, both timed,
  with a marker page written at each end between the two. The markers
  are what make the second verdict mean something: a chip with no D8h
  command would ignore it in silence, and an already-erased partition
  would read all ones either way.
