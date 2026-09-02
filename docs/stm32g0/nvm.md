# FLASH - embedded flash memory and the storage over it (STM32G0)

> **PROVISIONAL.** The program and erase engine, the geometry, the option
> bytes as a read-only decode and both storage backends are built and
> bench-verified. What is deliberately left out is either a one-way
> hazard (writing an option byte, the OTP, RDP) or a protection nothing
> in brio sets yet. The list is in "Not covered yet".

Documents of record: RM0444 Rev 6 ch. 3 (3.3.1 organization, 3.3.2 dual
bank and table 12, 3.3.3 ECC, 3.3.6..3.3.9 program/erase/RWW, 3.4 option
bytes, 3.5 protections, 3.6 interrupts, 3.7 registers), datasheet
DS13560 Rev 5 tables 48 and 49 for the timings and the endurance, and
errata **ES0548 Rev 3** on silicon revision Z, where three items touch
this chapter: 2.2.3 (a location holding all ones cannot be re-programmed
to all zeros - LIVE, and unreachable by construction here), 2.2.10
(prefetch failure branching across banks - LIVE, and the reason the
storage is in bank 2 while the code is in bank 1) and 2.2.5 (the PCROP
read weakness, revision A only, and nothing here sets PCROP). Drivers:
`stm32g0/flash.hpp` (`FlashWaitStates`, `FlashAccel`, `Flash`,
`FlashOptions`) and `stm32g0/nvm_flash.hpp` (`MainFlashPartition`,
`MainFlash` the `util/nv_heap.hpp` backend, `MainFlashJournalZone` the
`util/nv_journal.hpp` one). Family fixtures `test/family_stm32g0/flash.cpp`
and `nvm_flash.cpp` plus four negatives under `tools/check_stm32g0.sh`;
the bench suites are `test_stm32_nvm` and `test_stm32_journal`.

## What the silicon does

**One array, two banks, three granularities.** 512 Kbytes in two banks of
256 (table 11); a **page** of 2048 bytes is the erase unit, a **row** of
256 bytes is the fast-programming unit, and a **double word** of 8 bytes
is the ordinary program unit and the ECC unit - 64 data bits with 8 ECC
bits beside them (3.3.1). Nothing smaller can be written at all: a byte
or half-word store raises SIZERR and a double word that is not
double-word aligned raises PGAERR. A cell is programmed **once** between
erases; a second, non-zero write raises PROGERR. That last sentence is
`util/nv_heap.hpp`'s `write_cell` contract stated by the hardware, which
is why the two fit with nothing in between.

**The 512 Kbyte part is always dual-bank** whatever DUAL_BANK says (3.3.2
note 1, and measured). `nSWAP_BANK` decides which physical bank appears
at 0x0800_0000. It changes nothing about programming - the hardware
follows the logical address - but an **erase is bound to the physical
bank** (FLASH_CR.BKER), so it is the one place the swap changes an
outcome. `Flash::erase_bank_of()` is that arithmetic; FLASH_CR.PNB is the
page's index **within its bank**, so the first page of the upper bank is
page 0 and not page 128.

**Read-while-write (3.3.9).** A program or erase in one bank leaves reads
of the other bank running. Inside one bank a read stalls the bus until
the operation finishes (3.3.6) - and instruction fetch is a read, so for
code executing from that bank the CPU stops for the whole 22 ms of a page
erase. This is the property the whole storage design rests on.

**The unlock is a keyed sequence and a wrong one is fatal until the next
reset.** KEY1 then KEY2 into FLASH_KEYR (3.3.6); any wrong sequence locks
FLASH_CR until the next system reset *and* is a bus error, i.e. a
HardFault. Writing a key into an already-unlocked KEYR is itself a wrong
sequence, so `Flash::unlock()` is idempotent by checking LOCK first and
there is no verb in the file that can produce a half sequence.

**FLASH_CR must not be written while CFGBSY stands** - 3.7.5 says such a
write "causes a HardFault exception". Every verb waits first, and the
wait is bounded.

**EOP is not a completion witness.** 3.7.4 bit 0 says EOP "is set only if
the end of operation interrupts are enabled (EOPIE=1)", which contradicts
3.3.8's own step 7 telling the programmer to check it. The bench settles
it (below): 3.7.4 is right. This driver judges an operation by CFGBSY
falling and by the error bits.

**HSI16 is turned on by the engine itself** when PG, FSTPG or STRT is set,
and turned off again unless RCC_CR.HSION was already set (3.3.7, 3.3.8).
Nothing here manages it.

## Types and verbs

`FlashWaitStates` and `FlashAccel` are the bring-up surface described in
[clock.md](clock.md): the LATENCY the clock task sets before it raises
HCLK, plus prefetch, the instruction cache and its flush, the empty-check
bit and the debug-access bit. Both are read-only about the two bits this
stratum leaves at their reset values.

`FlashFlag` is FLASH_SR as one mask vocabulary. Every operation returns a
mask, zero meaning success, because several causes can stand at once. One
bit is **not** silicon: `FlashFlag::refused` (bit 31, which FLASH_SR
reserves) means the driver refused before the flash was touched - a
misaligned address, a locked FLASH_CR, a wait that timed out.

`Flash` is the engine: the geometry as constants and as runtime readings
(`bank_count`, `pages_per_bank`, `banks_swapped`, `erase_bank_of`,
`page_of`), `read`/`read_otp`, the status and its clearing, `unlock` /
`lock`, `erase_page`, `mass_erase` (exposed, never called in this tree),
`program`, `fast_program_row`, the three interrupt enables and the
`FLASH_IRQHandler` body, and the per-bank ECC status. Two diagnostics are
published because a bench suite needs them and an application may want
them: `last_status()` (the whole FLASH_SR the last operation ended on)
and `last_wait_turns()` (how many turns the last operation's completing
wait spent - the number that *measures* read-while-write instead of
assuming it).

`Flash::provoke(Misstep, addr)` is the deliberately-named diagnostic that
performs one of four malformed programs - three of 3.3.8's own, plus the
half-double-word the chapter has no name for - so a suite can see the
silicon raise SIZERR, PGAERR and PGSERR rather than only see the driver's
own bounds check. **The fourth is one-way** - see the findings. Nothing
but a bench suite has any business calling it.

`FlashOptions` decodes FLASH_OPTR, the WRP and PCROP area registers and
FLASH_SECR, and has no setter of any kind. That is the design, not an
omission: RDP Level 2 is irreversible and ES0548 2.2.9 says an
interrupted option write can leave the device with BOOT_LOCK set and the
debug interface gone. Option-byte provisioning belongs to a tool over
SWD, the way fuses do on the other two targets.

`MainFlashPartition` / `MainFlash` / `MainFlashJournalZone`
(`stm32g0/nvm_flash.hpp`) are the storage. See below.

## How to use it

**Reading and erasing a page of the storage bank.** Everything is bounded
and every return is a mask.

```cpp
using namespace brio;
if (Flash::unlock()) {
    const uint32_t err = Flash::erase_page(0x0804'0000UL);
    uint8_t page[16];
    Flash::read(0x0804'0000UL, page);
    (void)Flash::lock();
    if (err != 0u) { /* err names PROGERR / WRPERR / ... or `refused` */ }
}
```

**Programming.** Address and size are multiples of 8 and the cells have
been erased since they were last written; both halves are checked before
the flash is touched.

```cpp
const uint8_t value[16] = { /* ... */ };
if (Flash::unlock()) {
    const uint32_t err = Flash::program(0x0804'0000UL, value);
    (void)Flash::lock();
}
```

**A whole row under one high-voltage ramp.** The 32 double words must
arrive within about 20 us of each other or MISSERR stops the row, so
**the caller masks interrupts**. The driver does not do it for the
caller: a driver that masks behind the application's back decides the
system's latency.

```cpp
uint8_t row[Flash::row_size];
{
    Stm32Platform::CriticalSection cs;
    (void)Flash::fast_program_row(0x0804'0000UL, row);
}
```

**The storage: a block heap and a value journal, both over bank 2.**
Nothing here is target-specific except the two media types.

```cpp
brio::NvHeap<brio::MainFlash, 8, 2> heap;             // blocks
brio::NvJournal<brio::MainFlashJournalZone, 6, 32, 1> journal;  // values

heap.mount();       // read-only: no erase, no program at boot
journal.mount();
if (auto w = heap.alloc(0x1234, 300)) { w->append(table); w->seal(); }
journal.save<Calibration>(1, cal);
```

## The storage design: bank 2 is the attic

The linker script gives the compiler **bank 1 only** (256 K of `rom`), so
nothing the image contains can land in bank 2, and `__brio_rom_end` is
read back at run time as the proof. Three facts point the same way:

- **3.3.9** makes a program or erase of bank 2 invisible to code running
  in bank 1, so an ordinary `NvJournal::save()` is legal from the main
  loop. Inside one bank the CPU would stop for the operation's whole
  duration.
- **ES0548 2.2.10** says the prefetch may fail on a branch across banks,
  with no workaround - and its own note lists "EEPROM emulation or other
  data storage in bank 2" as the safe use of the feature. Code must stay
  in one bank; data in the other is the vendor's sanctioned arrangement.
- the bounds become **constants**, as they are on the samc's RWWEE array
  and unlike the AVR's, where the free flash is bounded by linker symbols
  that move with every build.

The bank splits once, at the top:

| offset from 0x0800_0000 | address | who |
| --- | --- | --- |
| 0x00000..0x3FFFF | 0x0800_0000..0x0803_FFFF | bank 1: the image, and nothing else |
| 0x40000..0x7EFFF | 0x0804_0000..0x0807_EFFF | `MainFlash` - the heap's 252 K, its map pair in the top two pages |
| 0x7F000..0x7FFFF | 0x0807_F000..0x0807_FFFF | `MainFlashJournalZone` - the attic, two 2 K halves |

**The media's addresses are OFFSETS from 0x0800_0000, and that is forced.**
`util/nv_heap.hpp` numbers erase units in a `uint16_t`, so an absolute-address
media must sit below page 65536 - which the samc's RWWEE array at
0x0040_0000 does (page 16384) and this bank does **not**: 0x0804_0000 /
2048 is 65664, one page past the field. The AVR backend already numbers
its flash from zero, so this is the contract's other established
convention and not a new one. It costs one addition per access, and the
family fixture asserts both halves of the reasoning.

**A journal half is one page**, which is the smallest a half may be
(erasing one must not take the other down) and is already 256 write
cells. The invariant `(max_ids + 2) x max_entry_cells <= half_cells` then
reads `(6 + 2) x 6 <= 256` for the bench's six 32-byte values - and holds
up to 40 such ids. That is what the 2048/8 geometry buys over the samc's
256/64, where the same six values needed both rows of a 1 K attic.

**Erratum ES0548 2.2.3 is unreachable by construction.** 3.3.8 makes one
exception to the write-once rule - a location may be overwritten if the
value is all zeros - and the erratum says that exception does not work
when the location holds all ones. Neither `NvHeap` nor `NvJournal` ever
programs a cell twice between erases, so neither can ask for the
exception; `MainFlash::program()` does not special-case zeros either, and
a caller that tries gets the silicon's PROGERR back unedited.

**A part that is not this one is refused, not guessed at.** The device
select macro is `STM32G0B1xx` for the 128, 256 and 512 Kbyte members
alike, so the flash size is knowable only from the size register.
`MainFlashPartition::geometry_matches_silicon()` asks it, together with
the linker's own boundary; when the answer is no, `zones()` reports a
band no user can fit in and both mounts answer `bad_geometry` having
written nothing.

## Bench findings

Measured by `test_stm32_nvm` (z = 85 verdicts over 10 letters, plus `s`
12/12 and `v` 5/5 outside `z`) and `test_stm32_journal` (z = 52 verdicts
over 7 letters, plus `p` and `v`), on the Nucleo-G0B1RE at 64 MHz. Both
suites are wireless.

**The timings, against DS13560 table 48.** A page erase takes **22039 us**
to the microsecond, run after run - table 48's typical is 22.0 ms and its
maximum 40.0. One double word programs in **92 us** and eight in one call
in **705 us**, i.e. **88 us each** against a typical of 85 and a maximum
of 125 - so the high-voltage ramp is per double word and not per call.

**Fast programming beats its own datasheet by a factor.** A whole row of
32 double words under FSTPG takes **683 us** where table 48 gives 1.7 ms
typical; the same 256 bytes the ordinary way take **2808 us**, which
matches that table's 2.7 ms exactly. 683 us over 32 double words is 21 us
each, which is precisely the figure 3.3.8's own note gives for the
interval the data must arrive within ("around 20 us") - so the note is
exact and the datasheet's fast-row typical is conservative.

**EOP: 3.7.4 wins.** With EOPIE clear a successful program leaves
FLASH_SR at **0x0**; with EOPIE set the same program leaves **0x1**, with
no NVIC line enabled to take an interrupt. So the enable bit gates the
FLAG and not only the request, 3.3.8's step 7 is wrong as written, and a
polled driver that waits for EOP waits for ever. The interrupt letter
shows the other half: with EOPIE and ERRIE set and the line enabled the
handler runs once per erase and is given EOP to clear; with the enables
down the same erase raises nothing at all.

**Read-while-write, measured rather than assumed.** During a 22039 us
erase of a bank-2 page, the polling loop in `flash.hpp` - which lives in
bank 1 - completed **174260 turns**, one every 8 core cycles. A stalled
bus would have allowed none. A double-word program leaves about 664 turns
in the same way. The letter states what it cannot do: there is no
control, because the only same-bank erase available is an erase of the
running image.

**PROGERR comes alone.** A second, non-zero write of a written cell
returns exactly PROGERR (0x8) and **not** PGSERR beside it. 3.3.8's
"PGSERR is set also if PROGERR ... is set due to a previous programming
error" is about an error still standing when the next operation starts,
which is why both sequences in the chapter open with "check and clear all
error flags" - `flash.hpp` does that as its own first act, so a clean
caller sees the real cause alone.

**WHETHER A MALFORMED PROGRAM IS RECOVERABLE DEPENDS ON ITS LENGTH, NOT
ON ITS OFFENCE - and no part of chapter 3 says so.** `test_stm32_nvm`'s
letter `s` stages four wrong programs, one per boot, and the answer
splits in two:

| what was sent | FLASH_SR | CFGBSY after |
| --- | --- | --- |
| eight bytes in four HALF-WORD accesses | 0xE0 = SIZERR + PGAERR + PGSERR | down |
| two words straddling the double-word boundary (+4 then +8) | 0x20 = PGAERR alone | down |
| a whole double word with PG and FSTPG clear | 0x80 = PGSERR alone | down |
| a legal FIRST word and no second | **no error bit at all** | **standing** |

A misstep that sends a whole double word's worth of accesses raises its
documented flag and is over. A misstep that sends *half* a double word
raises nothing - from the engine's side nothing has gone wrong yet - and
3.7.4's "CFGBSY ... is cleared after the second word is sent" then never
happens: **no further flash operation of any kind is possible, and a
store into FLASH_CR from there would be a HardFault** (3.7.5). Only a
system reset gets the interface back, which letter `s`'s last leg proves
by erasing and programming again afterwards. `Flash::provoke()` reports
that state as `FlashFlag::refused` and touches nothing.

The first version of this suite paid for the discovery the expensive way:
it provoked SIZERR with a single half-word store inside `z`, wedged the
engine, and rebooted the board in the middle of its own output.

**The option bytes of this board**, read through `FlashOptions`
(FLASH_OPTR = 0xFFFF_FEAA):

| field | value | meaning |
| --- | --- | --- |
| RDP | 0xAA | level 0 - the only level a debugged board can be at |
| BOR_EN | 0 | the configurable brown-out reset is off; POR/PDR levels apply |
| BORR_LEV / BORF_LEV | 3 / 3 | 2.9 V rising, 2.8 V falling, if it were enabled |
| nRST_STOP / STDBY / SHDW | 1 / 1 / 1 | no reset generated when entering any low-power mode |
| IWDG_SW | 1 | software independent watchdog (firmware starts it) |
| IWDG_STOP / IWDG_STDBY | 1 / 1 | the IWDG counter runs in Stop and Standby |
| WWDG_SW | 1 | software window watchdog |
| RAM_PARITY_CHECK | 1 | SRAM parity check **disabled** (the bit is inverted) |
| nBOOT_SEL / nBOOT0 / nBOOT1 | 1 / 1 / 1 | BOOT0 comes from the option bit, main flash boot |
| DUAL_BANK | 1 | set, and inert: a 512 K part is dual-bank regardless |
| nSWAP_BANK | 1 | no swap - the storage really is physical bank 2 |
| NRST_MODE | 3 | bidirectional reset (the legacy default) |
| IRHEN | 1 | internal resets hold the reset pin low |
| WRP1A/B, WRP2A/B | 127..0 | empty: no write protection anywhere |
| PCROP1A, PCROP2A | 511..0 | empty; PCROP_RDP = 0 |
| SEC_SIZE / SEC_SIZE2 | 0 / 0 | no securable area, so FLASH_CR's SEC_PROT bits are inert |
| BOOT_LOCK | 0 | clear - the state ES0548 2.2.9 warns about is not this board's |
| FLASH_ACR.DBG_SWEN | 1 | the debug port reaches the core |

OPTVERR does not stand, so the option loader trusted what it read. The
two watchdog bits are the same two `test_stm32_platform` letter a reads
from the RCC's side: WWDG_SW = 1 is why nothing was feeding a window
watchdog before that suite started.

**util/nv_heap.hpp on the third silicon, unchanged.** A mount costs no
erase and no program. A 300-byte block is placed at the top of the zone,
sealed, found, read back byte for byte and survives a fresh mount; a
`rewrite()` replaces it in place; the map pair really ping-pongs (page
0 -> 1 -> 0) and every mutation lands on the other one. A whole `z` run
costs **15 page erases** (8 through the heap, 7 raw), which against
DS13560 table 49's 10 kcycle minimum leaves the busiest page good for
hundreds of runs.

**util/nv_journal.hpp on the third silicon, unchanged.** Mount is
read-only and takes **4.3..5.3 ms** for 800..1000 media reads over the two
2 K halves - the boot-path cost of walking 512 cells. A save of 16 bytes
is **420 us** and **no erase**: four double words plus the CRC over them.
Fifty-seven such saves fill a half and trigger a collection, which costs
**46.3 ms** for six live ids - two page erases (2 x 22 ms) plus one
program each - and the erase inside it spins **176009** bank-1 turns, so
the no-stall claim holds for the attic too. That is what makes an
ordinary save legal from the main loop on this target. Over eighty
consecutive saves the half never fell below **7** free cells against a
reserve of **6**, and `save_reserved()` spends exactly that in **629 us**
with **zero** erases - which is what makes it legal from a panic handler.
A `z` run costs **10 attic page erases**.

**Coexistence.** A heap block in the lower pages stays byte-exact while
the journal collects twice in the attic above it, and a heap mutation
leaves the journal's values untouched: two storage classes over one bank
and one FLASH_CR, neither reaching the other. The bounds are what
guarantee it - `MainFlash` refuses an address below its floor or in the
attic, and `MainFlashJournalZone` refuses anything below the attic - so
the running image is out of reach by bounds and not only by convention.

**A panic crosses a reset in FLASH - and the FAULT BODY is what writes
it.** `test_stm32_journal` letter `p` panics for real. On a board whose
C_DEBUGEN `tools/bench.py` has cleared, `panic()`'s BKPT escalates into
HardFault *before* `Reporter::report()` is ever reached, and the first
version of the letter measured exactly that: the SRAM breadcrumb present,
the journal empty. So an application that wants a breadcrumb in FLASH on
this target must bind the fault body and write it there - through the
same bounded, erase-free `save_reserved()` path the reporter would have
used, which is what makes it legal with interrupts masked for good and
the board about to reset. The suite's `HardFault_Handler` is that
composition, three lines of application glue over
`kernel/panic.hpp` and `stm32g0/reset.hpp` with neither touched. The
record then comes back with its code and its context byte intact,
`take()` returns it once, and the next ordinary save restores the
reserve. (The samc campaign found the same thing on its own silicon; this
is its confirmation on the third.)

**A reflash does not touch the storage.** OpenOCD's `program <elf> verify`
erases only the sectors the image occupies, and the image occupies bank 1
alone - so the blocks and the values survive flashing a different app and
coming back. Measured both ways: `test_stm32_nvm`'s three heap blocks
came back byte-exact after `test_stm32_journal` had been flashed over
bank 1, and the journal's six values came back after `test_stm32_nvm`
had. That is what the `v` letter of each suite is for, and it is also why
there is no way to wipe the storage from `bench.py`: `--erase` is refused
on this target and nothing in the tree calls `mass_erase()`.

## Not covered yet

Driver gaps:

- **Writing an option byte.** There is no OPTKEYR, OPTSTRT or OBL_LAUNCH
  verb, on purpose (see above). Provisioning wants a `bench.py` verb over
  SWD, the way the samc's user row got one.
- **Setting WRP, PCROP or the securable area, and RDP.** Read-only decode
  only. Each of them is an option-byte write, and RDP Level 2 is one-way.
- **Writing the OTP area.** It is memory-mapped and `read_otp()` reads
  it; a write is one-way per double word (3.3.1) and has no user yet.
- **The bank swap.** `nSWAP_BANK` is read and its consequence for erase is
  implemented and tested; setting it is an option-byte write.
- **A RAM-resident erase path.** Erasing or programming *bank 1* stalls
  the CPU for the operation's whole duration (3.3.6), so a firmware
  updater would have to run from SRAM. Nothing in brio does, and
  `mass_erase()` is exposed but never called.

Implemented but not bench-verified:

- **ECC.** `Flash::ecc()` reads FLASH_ECCR and FLASH_ECCR2 and the
  correction interrupt is wired, but no ECC error has been *provoked*:
  injecting one means writing a double word twice on purpose, which is
  the erratum-2.2.3 path and leaves a cell nobody can trust. The NMI half
  (ECCD, a double error) is stated and unexercised.
- **RDERR and the PCROP read path**, which need a PCROP area to exist.
- **`mass_erase()`** on either bank.
- **A single-bank part.** The G071 and G031 headers compile the driver and
  the family fixture proves the second bank's registers are reached only
  where they exist, but no such board has run this code, and on one of
  them `MainFlashPartition::geometry_matches_silicon()` would (correctly)
  close the storage.
