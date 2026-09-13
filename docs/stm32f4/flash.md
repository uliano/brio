# The embedded flash memory interface (STM32F4)

Documents of record: RM0090 Rev 22 ch. 3 - which describes TWO flash
interfaces, the F405/F407/F415/F417 class's in 3.3 and the F42x/F43x
class's dual-bank one in 3.4 - and its twins RM0390 Rev 6 ch. 3 (the
F446) and RM0383 Rev 4 ch. 3 (the F411); the erase and program times are
the datasheets' (DS10314 table 45 for the F411, DS10693 table 48 and
DocID024030 table 48 for its siblings) and so is the endurance (DS10314
table 47 and its twins: 10 kcycles a sector).
Errata: no item of ES0206 Rev 24, ES0298 Rev 8 or ES0287 Rev 6
is filed against the program/erase engine or the option bytes; the two
that touch the chapter from outside are quoted below and are both
dual-bank only. Driver: `stm32f4/flash.hpp`; the per-part facts come
from `stm32f4/device_tables.hpp`. Bench suite: `test_stm32f4_flash`.
Family fixture `test/family_stm32f4/flash.cpp` plus three negatives
under `brio check stm32f4`.

The read side of this chapter - the wait states the clock task programs
before it raises HCLK, and the ART accelerator - is in the same file and
is documented in [clock.md](clock.md), where its caller is. This page is
the write side.

**There is no FlashMedia here, and no partition in any linker script.**
The geometry is what raises the question: the small sectors are at the
BOTTOM of the bank, under the vector table, and everything above the
first 128 Kbytes is one 128 Kbyte sector, so a fixed storage zone would
cost every image a partitioned linker script for a grain sixty-four
times the STM32G0's page. That decision is the NV stack's own review
(CLAUDE.md, "The NV stack reviewed as a whole"), and until it is taken
this chapter delivers the ENGINE and nothing above it.

## What the silicon does

**One array, uneven sectors, one erase grain.** A bank's sectors are
four of 16 Kbytes, then one of 64, then 128 Kbytes to the end of the
bank - the same rule in all four chapters read (RM0090 tables 5, 6, 8,
9 and 10, RM0390 table 4, RM0383 table 4), which is why the driver
COMPUTES the map from the flash size register instead of tabulating it.
The sector is the only erase unit: there is no page, no row and no
half-word grain. A 512 Kbyte part has eight sectors, a 2 Mbyte one
twenty-four.

**The sector NUMBER is not the sector's position.** On a dual-bank part
bank 1 ends at sector 11 whatever its length and bank 2 starts at
sector 12, and FLASH_CR.SNB encodes 12..23 as 16..27 - RM0090 3.9.8
leaves the four codes between "not allowed". The driver's `sector_at()`
walks the array in address order and `sector()` looks a sector up by the
manual's number; on a single-bank part the two agree.

**Which part has what is the MANUAL's, and the device header is wrong in
both directions.** ST declares one `FLASH_TypeDef` and one set of bit
masks for a whole marketing line, so the F446's header carries
`FLASH_CR_MER2`, `FLASH_OPTCR_DB1M` and `FLASH_OPTCR_BFB2` - three bits
of a second bank RM0390 3.3 says it has not got - while the F411's
header does NOT carry `FLASH_OPTCR_SPRMOD`, a bit RM0383 3.6.5
describes in full. So `flash_facts()` in the reserve is keyed on the
part class and states only what was read: the sector count per bank, the
width of SNB, whether a second bank can exist, whether PCROP and RDERR
do, how many nWRP bits there are and whether OPTCR1 is real. A part
class whose chapter 3 is not on the desk gets `known == false`, and
there the driver refuses every ERASE (it has no map to aim SNB with)
while PROGRAMMING, which needs no map, keeps working.

**Two names in the code are the header's and not the manual's**: bit 1
of FLASH_SR is OPERR in every reference manual and `FLASH_SR_SOP` in
every device header, and bit 15 of FLASH_CR is MER1 in RM0090 3.9.8 and
`FLASH_CR_MER2` in the header. Same bits; the driver's own names
(`FlashFlag::operation_error`, `bank_erase<FlashBank::bank2>`) are the
manual's.

**The program unit is the access width, declared twice.** FLASH_CR.PSIZE
says x8, x16, x32 or x64 and the STORE INSTRUCTION must match it or the
silicon raises PGPERR (3.5.4). That is why the parallelism is a TEMPLATE
parameter of `program()` and `erase_sector()`: it decides which
instruction the compiler emits, where a runtime argument would need a
switch over four widths inside the loop.

**PSIZE is a claim about the supply, not a speed setting.** Table 6 /
table 13: x8 from 1.7 V, x16 from 2.1, x32 from 2.7, and x64 ONLY with
an external 8..9 V supply on the VPP pad. It also decides the ERASE
time, which is not obvious: a 16 Kbyte sector is 400 ms at x8 and 250 at
x32 (DS10314 table 45). The driver REFUSES x64 at compile time - no
board here has the pad, and the datasheet forbids leaving VPP applied
for more than an hour - and takes the other three on trust, no register
being able to check a rail.

**A cell is NOT written once.** 3.5.4: "Successive write operations are
possible without the need of an erase operation when changing bits from
1 to 0. Writing 1 requires a flash memory erase operation." A second
program into a programmed word is therefore accepted, ANDs into it and
raises nothing at all - the opposite of the STM32G0's PROGERR, and the
reason `util/nv_heap.hpp`'s write-once cell contract cannot simply be
carried here. Measured, with the twist the next paragraph describes.

**A read-back through the D-cache is a report of what was WRITTEN.** The
same paragraph of 3.5.4 says a write "modifies the data in the flash
memory AND the data in the cache", and the flash takes the AND while the
cache takes the value: after programming 0x5A5A5A5A over 0xA5A5A5A5 the
cache answers 0x5A5A5A5A and the array holds zero. So on this family the
only honest verification of a program is a read with the caches flushed
(`FlashAccel::flush_caches()`) or with DCEN clear. Measured both ways.

**An erase does not invalidate the caches either.** 3.5.4: "If an erase
operation in flash memory also concerns data in the data or instruction
cache, you have to make sure that these data are rewritten before they
are accessed during code execution. If this cannot be done safely, it is
recommended to flush the caches" - and the reset bits may be written
only while the cache is disabled, which is the whole dance
`FlashAccel::flush_caches()` performs. Nothing in the driver does it
behind a caller's back: an erase does not know which lines it
invalidated, and a driver that resets the instruction cache on its own
decides the system's timing.

**The core is frozen while the flash works.** 3.5: "Any attempt to read
the flash memory while it is being written or erased causes the bus to
stall." On a single-bank part the code IS in the array, so the
instruction fetches stop with it: no interrupt is taken and the kernel
tick loses the whole of a one-second erase. The F42x/F43x class is the
exception - two banks, and a read of one runs while the other is erased
(3.6.5), which is what makes a storage bank thinkable there and nowhere
else. `Flash::last_wait_turns()` counts the poll iterations the CPU
completed while the engine was busy and is what tells the two apart.

**The unlock is a keyed sequence and a wrong one is fatal until the next
reset.** 3.5.1: KEY1 then KEY2 into FLASH_KEYR, and "any wrong sequence
returns a bus error and lock up the FLASH_CR register until the next
reset". Both halves are real: the store raises an IMPRECISE bus error on
this core, and after it even the correct pair is refused. `unlock()` is
therefore idempotent - it writes the keys only when LOCK really stands -
and no verb of the file can produce half a sequence. The option bytes
have their own key pair, their own lock and their own start bit.

**EOP and OPERR exist only while their interrupt enables are set.**
3.8.4 says so of both bits, which contradicts nothing in the erase and
program procedures only because those tell the programmer to wait on BSY
instead. The driver judges an operation by BSY falling and by the error
flags; EOP is reported and never required.

**The option bytes are a second engine, and writing any of them erases
and reprograms the whole configuration sector** (3.6.2's note). RDP is
READ and never written by any verb: level 2 is irreversible and takes
the debug port with it, level 1 costs a mass erase to leave, and
provisioning belongs to a tool over the debug port. The one option the
driver does write is nWRP, the per-sector write protection - reversible
at RDP level 0 with PCROP off, which is figure 4's "options write (RDP
level identical)" - and it reaches the register through a HALF-WORD
store to OPTCR's upper half, so the RDP byte is not in the data path at
all.

**The OTP area** (3.7) is 512 bytes in sixteen blocks of 32 with a lock
byte each, writable once and never erasable. It is memory-mapped and
read like any other flash; the driver offers the read and, on purpose,
no write.

**The errata that touch the chapter are both dual-bank only.** ES0206
2.2.15 and ES0298 2.2.12 are the same item: the data cache may be
corrupted when a read of one bank meets a write of the other, and the
workaround is to drop DCEN around the write and reset the cache after -
`FlashAccel::flush_caches()`. ES0206 2.2.14 is a pad rule: on an
F42x/F43x with the second bank in use, PA12 as a GPIO corrupts data read
from the flash. Neither can be live on a single-bank part.

## Types and verbs

**`FlashParallelism`** - x8, x16, x32, x64, with
`flash_parallelism_bytes()` and `flash_parallelism_min_mv()` (the
datasheet's rail per width; 0 for x64, whose condition is VPP and not a
VDD). x64 is refused at compile time wherever it is named.

**`FlashFlag`** - FLASH_SR as one mask vocabulary: `eop`,
`operation_error` (the manual's OPERR), `write_protect_error`,
`alignment_error`, `parallelism_error`, `sequence_error`,
`read_protect_error` (0 where the part has no PCROP), `busy`, the
`errors` and `clearable` unions, and `refused` - bit 31, which no
silicon flag occupies, meaning "the driver refused before the flash was
touched". Every operation returns a mask of these, zero being success.

**`FlashSector`** - the manual's sector `number`, its `bank`, its `base`
and `size`, and `contains()`.

**`FlashBank`** - which bank an erase names. **`FlashEraseAll`** - the
confirmation `mass_erase()` takes as a template argument.

**`Flash`** - the engine, a monostate. What this part is:
`geometry_known`, `dual_bank_capable`, `size_bytes`, `bank_count`,
`bank_bytes`, `sectors_per_bank`, `sector_count`, `sector_size_in_bank`,
`in_main_flash`. The map: `sector_at` (by position), `sector` (by the
manual's number), `sector_of` (by address). Reading: `read`, `blank`,
`read_otp`, `otp_block_locked`. Status: `status`, `busy`, `errors`,
`clear`, `clear_errors`, `last_status`, `last_wait_turns`, `wait_ready`.
The lock: `locked`, `unlock`, `lock`. Erasing: `erase_sector`,
`bank_erase`, `mass_erase`. Programming: `program`, `program_word`. The
four malformed sequences a suite stages on purpose: `Misstep` and
`provoke`, and beside them `provoke_wrong_key`, the fifth, which ends
the engine's life until the next reset. The interrupt: `irq`,
`interrupts`, `end_of_operation_interrupt`, `error_interrupt`, `isr`.

**`FlashRdpLevel`** and **`FlashOptions`** - the option bytes, decoded:
`raw`, `raw_bank2`, `rdp_code`, `rdp`, `bor_level`, `bor_off`,
`iwdg_software`, `reset_on_stop`, `reset_on_standby`, `pcrop_mode`,
`dual_bank_boot`, `dual_bank_1m`, `nwrp`, `write_protected`,
`pcrop_protected`; and the one write, behind its own engine: `locked`,
`unlock`, `lock`, `protect`.

**`FlashAccel`** gains `flush_caches()` beside the switches the clock
task uses - both caches down, both reset, both put back as they were.

The reserve (`stm32f4/device_tables.hpp`) publishes `FlashFacts` /
`flash_facts()`, the sector shape constants, `flash_sector_number_code`,
`flash_read_error_flag`, `flash_bank2_mass_erase_mask`,
`flash_pcrop_mode_mask`, `flash_dual_bank_option_mask`,
`flash_dual_bank_boot_mask`, the system-memory, OTP and option-byte
addresses, and `flash_irq`.

## How to use it

Erase a sector and program a block into it - the whole round trip, with
the read-back the caches cannot be trusted to give:

```cpp
#include "stm32f4/flash.hpp"
using namespace brio;

const std::optional<FlashSector> s = Flash::sector_of(0x08060000UL);
if (s && Flash::unlock()) {
    if (Flash::erase_sector(s->number) == 0u) {
        FlashAccel::flush_caches();              // the erase invalidated nothing
        Flash::program(s->base, std::span<const uint8_t>{data, sizeof(data)});
        FlashAccel::flush_caches();              // now the read-back is the array's
    }
    Flash::lock();
}
```

At another parallelism - x16 for a rail between 2.1 and 2.7 V - the
width is a template argument and the stores follow it:

```cpp
Flash::program<FlashParallelism::x16>(addr, bytes);
Flash::erase_sector<FlashParallelism::x16>(s->number);
```

Walk the map, which is the only way to know what a part has:

```cpp
for (uint8_t i = 0; i < Flash::sector_count(); ++i) {
    const std::optional<FlashSector> s = Flash::sector_at(i);
    print(sink, "sector ", s->number, " bank ", s->bank, " at ", hex(s->base),
          " of ", s->size / 1024u, " KB", crlf);
}
```

Read the option bytes - what a program needs from them is to KNOW:

```cpp
if (FlashOptions::rdp() != FlashRdpLevel::level0) { /* a debugger will not read us */ }
if (!FlashOptions::iwdg_software()) { /* the IWDG has been running since the boot */ }
```

Write-protect a sector, and take it off again. The verb refuses unless
RDP is 0xAA and PCROP is off, and it costs the configuration sector an
endurance cycle each way:

```cpp
if (FlashOptions::unlock()) {
    FlashOptions::protect(s->number, true);
    // ... a program or erase of that sector now answers WRPERR ...
    FlashOptions::protect(s->number, false);
    FlashOptions::lock();
}
```

Take the errors through the vector instead of the return value:

```cpp
Flash::interrupts(false, true);          // ERRIE alone
Nvic::enable(Flash::irq());
// and in the app's own handler:
extern "C" void FLASH_IRQHandler() { const uint32_t flags = Flash::isr(); ... }
```

## Bench findings

`test_stm32f4_flash` on an STM32F411CE (DEV_ID 0x431, REV_ID 0x1000,
512 Kbytes, 100 MHz from a 25 MHz crystal, 3 wait states, the ART fully
on, a 3.27 V rail). Its `z` set - the letters that cost the board
nothing - is 47 verdicts; the four that erase and the one that locks the
engine are asked for by name.

**The map is the rule, and the rule is right.** Eight sectors in one
bank - 0x0800_0000, 0x0800_4000, 0x0800_8000, 0x0800_C000 of 16 Kbytes,
0x0801_0000 of 64, then 0x0802_0000, 0x0804_0000 and 0x0806_0000 of 128
- tiling 512 Kbytes with no hole, the size register agreeing, and every
sector found by address and by the manual's number alike.

**The lock.** LOCK stands out of reset; the key pair clears it in 1 us;
a second `unlock()` writes no key. An erase attempted while LOCK stands
is refused by the driver before the flash is touched and leaves no
hardware flag at all.

**The four malformed sequences, each aborted with nothing written**
(SR after each, the target word unchanged at 0xFFFFFFFF):

| what was done | SR | flags |
|---------------|----|-------|
| a store into the array with PG clear | 0xC0 | PGSERR **and PGPERR** |
| a byte store while PSIZE says x32 | 0x40 | PGPERR |
| a word store two bytes off its alignment | 0x40 | PGPERR |
| a store into the system memory | 0x10 | WRPERR |

Two of those are worth keeping. A store with PG clear raises PGPERR
BESIDE the PGSERR 3.5.4 promises - the engine judges the width against a
PSIZE that means nothing yet - so a handler that switches on one flag
will mis-diagnose it. And an unaligned word store earns PGPERR and not
the PGAERR one might expect: the core splits the access into pieces that
are no longer words, and the parallelism check is what catches it first.
PGAERR belongs to the 128-bit row crossing, which only x64 can reach.

**The interrupt.** With ERRIE clear a misstep leaves SR 0xC0 and no
OPERR at all - 3.8.4's claim, measured - and no interrupt. With ERRIE
set the same misstep leaves SR 0xC2 and reaches the vector exactly ONCE:
the level goes down with the flags the body clears. An error raised
while the line is MASKED at the NVIC still latches its pending bit, so a
program that masks the line takes the pending bit down before opening it
again or the handler runs on history.

**Erase durations at x32, and the freeze.** The datasheet's typical
figures are in brackets (DS10314 table 45):

| sector | erased in | poll turns the core completed | kernel tick |
|--------|-----------|-------------------------------|-------------|
| 16 Kbytes | 189.2 ms (250) | 393 | 1 ms |
| 64 Kbytes | 528.8 ms (550) | 593 | 1 ms |
| 128 Kbytes | 978.0 ms (1000) | 589 | 1 ms |

The two smaller sizes come in well under the typical figure and the
128 Kbyte one straddles it - 973 to 1056 ms over the runs of a session,
all far from the 2 s maximum. The tick tells the story of the freeze: a
full second of erase advances the kernel's timebase by one millisecond,
because SysTick's handler cannot be FETCHED. The poll turns are the
ART's doing - the wait loop is small enough to be served out of the
instruction cache a few hundred times over a second - and with the
accelerator off the count is exactly ZERO: 972 ms of erase and not one
turn of the loop completed. The erase itself takes the same time either
way (975 ms with the ART against 972 without, and the spread between
two runs with it on is larger than that), the accelerator being the
CPU's side of the interface and not the array's.

**Programming.** 1024 bytes at x32, word by word with a status wait
after each: 3665 us, or 14.3 us a word against the datasheet's 16 us
typical. The block reads back byte for byte and its CRC-16 matches the
source's. EOP is set at the end of an erase - with EOPIE set, which is
what makes the flag exist.

**A second program into a programmed word - the fact the NV review
needs.** Each pair written into a 128-bit row of its own, read once
through the cache and once with the caches flushed:

| written, then written | the D-cache says | the array holds | flags |
|-----------------------|------------------|-----------------|-------|
| 0xA5A5A5A5, 0x5A5A5A5A | 0x5A5A5A5A | **0x00000000** | none |
| 0xFFFF0000, 0x0000FFFF | 0x0000FFFF | **0x00000000** | none |
| 0x12345678, 0xFFFFFFFF | 0xFFFFFFFF | **0x12345678** | none |
| 0x0F0F0F0F, 0x0F0F0F0F | 0x0F0F0F0F | 0x0F0F0F0F | none |

The array takes the AND, exactly as 3.5.4 says, and NOT ONE of the eight
programs raised a flag: there is no write-once cell on this family and
no PROGERR to catch a caller that writes twice. With DCEN clear the read
needs no flush - 0xCCCC0F0F then 0x0F0FCCCC reads 0x0C0C0C0C straight
away. Two consequences for anything built on top: a program's only
witness is a read-back taken with the caches out of the way, and a
storage layout that relies on "a cell can be written once" has to
enforce that itself.

**The write protection round trip.** OPTCR reads 0x00FFAAED at boot -
RDP 0xAA (level 0), BOR level 3 (off), the software watchdog, no reset
on Stop or Standby, PCROP off, every nWRP bit set. Protecting one sector
takes 350 ms - the configuration sector is erased and reprogrammed
whole - and moves exactly the one bit: 0x00FFAAEC to 0x007FAAEC, the RDP
byte and the user bits untouched. A program into the protected sector
answers WRPERR and writes nothing; an erase of it answers WRPERR too.
Taking the protection off puts OPTCR back where it started and the
sector takes a word again. A `protect()` through a locked option engine
is refused.

**The wrong key.** One wrong word into FLASH_KEYR raises exactly one
BusFault with CFSR 0x400 - BFSR.IMPRECISERR, an asynchronous store
error, which is why a program that wants to survive it arms the vector
first. After it LOCK stands, a CORRECT key pair is refused and raises a
bus error of its own, and every erase and program answers "refused"
until the next reset. The board comes back with a reset and nothing
else.

**The OTP area** reads 0xFF throughout with none of its sixteen blocks
locked, and a read past its end is refused.

## Not covered yet

Driver gaps, each with its reason:

- **No FlashMedia backend and no linker partition.** The uneven geometry
  makes a fixed zone expensive and the question belongs to the NV
  stack's own review (CLAUDE.md); until it is taken this chapter is the
  engine alone.
- **No OTP write.** One store is permanent, there is no way back and no
  user; `read_otp()` is the half that is safe.
- **No RDP verb, and no write of BOR_LEV, WDG_SW, nRST_STOP,
  nRST_STDBY, SPRMOD, BFB2 or DB1M.** Every one of them changes how the
  part BOOTS - the watchdog mode needs a system reset to take effect,
  the dual-bank bits renumber the sectors under the erase engine, and
  RDP level 2 is a one-way door that takes the debug port with it.
  Provisioning belongs to a tool over the debug port; firmware needs to
  know these, not to write them.
- **No PCROP.** Setting SPRMOD is exactly the state `protect()` refuses
  in, and leaving it is bound to an RDP 1 -> 0 transition that mass
  erases the part (3.6.5).
- **The sector map of the part classes whose chapter 3 is not on the
  desk** - the F401, F410, F412, F413/F423 and F469/F479. There
  `flash_facts().known` is false and every erase is refused rather than
  aimed at a guessed SNB; programming, which needs no map, works. RM0368,
  RM0401, RM0402, RM0430 and RM0386 would each add one row to the
  reserve.
- **x64 programming.** It needs 8..9 V on the VPP pad, which no board
  here has, and the datasheet forbids leaving that supply applied for
  more than an hour. Refused at compile time.

Implemented but not bench-verified, each with what would measure it:

- **Everything dual bank**: `bank_erase()`, the MER2 bit, the SNB
  encoding of sectors 12..23, OPTCR1's nWRP, the DB1M and BFB2 decoding
  and above all READ-WHILE-WRITE - the erase of one bank while the core
  runs from the other, which is the property that would make a storage
  bank thinkable on this family. An STM32F42x or F43x part measures all
  of it: the suite's letters a and g already branch on the bank and
  print what they find.
- **`mass_erase()`**, exposed and never called: it takes down the
  running image, so only a program that lives in RAM or a boot loader
  can measure it.
- **The x8 and x16 parallelisms.** Only x32 was run. The datasheet's
  erase times differ by up to a factor of two between x8 and x32 (400
  against 250 ms for a 16 Kbyte sector, 2 s against 1 for a 128 Kbyte
  one), so a run at each width on the same sector would measure both
  the time and the claim that the width is otherwise invisible.
- **The instruction cache's half of the stale-line problem.** The
  D-cache's lie is measured above; the I-cache's needs an erase of a
  sector holding code that is about to be executed, which cannot be
  staged from that same code and wants a routine copied to RAM first.
- **`FlashFlag::read_protect_error` (RDERR)**, which only a D-bus read
  of a PCROPed sector can raise - and PCROP is not set on any board
  here.
- **A locked OTP block.** Every block reads erased on the parts here, so
  `otp_block_locked()` is exercised against 0xFF alone; a part with a
  0x00 lock byte would show the other answer.
