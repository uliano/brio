# FMC, the external memory controller (STM32F4)

Documents of record: RM0090 Rev 22 ch. 37 and RM0390 Rev 6 ch. 11 - the
same block described twice, with two fields' difference in opposite
directions (below). RM0383 Rev 4 has no such chapter: the F411 has no
external memory controller. Errata: ES0206 Rev 24 section 2.3 has
fifteen items and ES0298 Rev 8's four; the ones this driver answers as
code or as a default, and the ones it can only state, are named where
they bite. Datasheet DocID024030 Rev 10 table 12 is where AF12 - the
alternate function every FMC signal takes - is stated, and which pad
carries which signal. Driver: `stm32f4/fmc.hpp`; the presence facts are
in `stm32f4/device_tables.hpp`. Bench suite: `test_stm32f4_fmc`, against
the 64-Mbit SDRAM the STM32F429I-DISC1 carries. Family fixture
`test/family_stm32f4/fmc.cpp` plus four negatives under `brio check
stm32f4`.

## What the silicon does

**Two different peripherals answer to this chapter, and the family is
split three ways.** The FSMC is a STATIC controller - NOR, PSRAM, SRAM,
NAND, PC Card - and the FMC is that controller plus an SDRAM half in
banks 5 and 6, plus three fields the FSMC has not (CCLKEN, CPSIZE,
CBURSTRW). They are not the same block and the device headers give them
different names, so the reserve asks two questions and never one:
`fmc_present()` for the F427/F429/F437/F439/F446/F469/F479, and
`fsmc_present()` for the F405/F415/F407/F417, the F412Rx/Vx/Zx and the
F413/F423. The F401, the F410, the F411 and the 48-pin F412 have
neither. This driver is the FMC's alone; nothing in it exists on an
FSMC part, and a program learns which block it has from the reserve
rather than from a driver that answers false.

**The address decoder is live before the memory is.** Six banks of the
Cortex's own map (37.4, figure 457): four NOR/PSRAM sub-banks of 64 MB
from 0x6000 0000 with a chip select each, NAND at 0x7000 0000 and
0x8000 0000, a PC Card at 0x9000 0000, and two SDRAM banks of 256 MB at
0xC000 0000 and 0xD000 0000. The AHB has no way to say "there is
nothing there": an access to a bank that is enabled but has no working
memory behind it completes with whatever the data pads read. Worse,
**a read of an SDRAM bank whose controller has not run its
initialization sequence HANGS THE MACHINE with no fault** (ES0206
2.3.3) - not a bus error, not a hard fault, a stopped core. That single
fact shapes the driver: `initialize()` is one verb that runs the whole
of 37.7.3 or answers false, and nothing hands out a pointer before it.

**What the controller DOES raise an AHB error for** (37.3): a read or
write of a bank that is not enabled, of a NOR bank with FACCEN clear, of
a PC Card with the presence pin low, a write to a write-protected SDRAM
bank, and an access past the SDRAM device into the reserved part of its
256 MB window. On the CPU that error is a hard fault; on a DMA it is a
transfer error that disables the stream.

**The static and dynamic halves cannot be used together.** ES0206 2.3.7:
put the SDRAM in self-refresh before touching a static bank and issue a
Normal command on the way back. There is no code answer to this - it is
a program-wide ordering - and it is why the bench suite's static letter
touches registers only.

**Three errata bracket the SDRAM read FIFO.** 2.3.4 says ENABLE it
(`SDCR1.RBURST`): with the FIFO off, a 16- or 8-bit CPU burst read that
crosses a row boundary and is interrupted reads the next row wrong.
2.3.15 says the FIFO ON can serve a stale line when an interrupted
burst read of one internal bank is followed by a read of another, and
offers three workarounds (turn the FIFO off, precharge all before the
second read, or do a dummy write). 2.3.5 is the broadest: with the FMC
holding stack, heap or variable data, an interrupt taken during a CPU
read may corrupt the read or fault, and reads by any master OTHER than
the CPU are not affected - so "read the FMC with DMA" is one of its two
workarounds and "mask interrupts around the read" the other. The
driver's default is the FIFO ON, because 2.3.4 has no other workaround
while 2.3.15 has three.

**The SDRAM clock is made here, not by the RCC.** `SDCR1.SDCLK` divides
HCLK by two or three for both banks, so a program that changes HCLK
changes the memory clock and every one of the seven SDTR delays under
it. The driver is deliberately not a `ClockUser`: rebasing those delays
means re-running the initialization sequence, which is a program-wide
decision and not a fan-out.

**Fields that are not per bank, though the register pair suggests they
are.** SDCLK, RBURST and RPIPE live in SDCR1 for BOTH banks (SDCR2's
copies are read-only or don't-care); TRP and TRC live in SDTR1 for both.
The driver writes them into bank 1's register whichever bank was asked,
so a two-device board states the slowest device's value and gets it.

**The refresh rate is arithmetic, not a mode.** `SDRTR.COUNT` is the
number of memory clock cycles between auto-refresh requests, and 37.7.5
gives it as `(refresh period / rows) x memory clock - 20`; the twenty
cycles are the margin for a request that lands on an accepted read. The
field is 13 bits, must be at least 41, and one further value is
forbidden: `TWR + TRP + TRC + TRCD + 4`.

**37.7.5's "if the value programmed in the register is 0, no refresh is
carried out" is not what this silicon does** (measured, below): a store
of 0 into COUNT reads back as 40 and the timer keeps running at that
rate, and so does a store of any other value under the floor. The
driver refuses anything below 41 so that no program is built on the
sentence.

**Nor does `SDCR1.SDCLK = 00` stop the memory clock** (measured, below).
37.7.5 calls the code "SDCLK clock disabled", the field reads back
cleared, and FMC_SDCLK's own pad goes on toggling - so the refresh
timer, which is decremented BY that clock, goes on counting too. The
field's real purpose is the one the same paragraph gives it: park the
clock while the divider or HCLK changes, which is what `memory_clock()`
is for (with the PALL 37.7.5 asks for in front of it, and the whole
initialization sequence as the way back). **The one thing that leaves
this device unclocked and unrefreshed is the block's own reset**,
`Fmc::reset()` - after which the sequence has to run again before a
single load or store, and after which the device's own charge is what
keeps the contents (seconds, measured below).

**Clearing the refresh error restarts the refresh timer.** `SDRTR.CRE`
is write-only and shares its register with COUNT, so the only way to
clear `SDSR.RE` is to store SDRTR back - and "as soon as the FMC_SDRTR
register is programmed, the timer starts counting" (37.7.5). A program
that clears the flag often postpones a refresh each time.

**The low-power modes are left by an ACCESS, not only by a command.**
37.7.4: in self-refresh the device keeps itself alive with no clock; in
power-down the CONTROLLER keeps refreshing it, leaving and re-entering
the mode for each refresh. In both, the first access to the bank makes
the controller issue the exit sequence, and the device stays in normal
mode afterwards.

**Two fields separate the two manuals, one each way.** `BCR1.WRAPMOD` is
RM0090's (37.5.6), and RM0090 states it does nothing anyway - no master
of this family emits a wrapping burst; RM0390 11.6.6 leaves bit 10
reserved, the F446/F469/F479 headers declare no mask for it, and the
driver refuses a config that asks for it there. `BCR1.WFDIS` is
RM0390's (11.6.6, bit 21): it DISABLES the write FIFO the whole
controller shares, it governs all four static banks from bank 1's
register, and ES0298 2.3.4 says a read taken in a one-cycle window
after a write returns corrupt data when it is set. Its reset value is
"FIFO enabled", which is that erratum's own workaround, so the driver
has no verb for it - see "Not covered yet". No CMSIS header of the pack
declares a mask for it either.

**The NAND half differs by part.** The F42x/F43x carry NAND banks 2 and
3 and a PC Card bank 4; the F446, F469 and F479 carry NAND bank 3 alone,
under a struct of its own. Both facts come off the header.

## Types and verbs

**`Fmc`** - the block. What this part has (`static_banks`,
`sdram_banks`, `nand_banks`, `has_pc_card`), the one vector every half
shares (`irq_line`), the AHB3 gate every other verb of the file opens
(`clock`) and the reset line that puts every bank back (`reset`, which
is also the only thing that stops the memory clock), the
six windows as constants (`static_window`, `sdram_window`,
`nand_window`, `pc_card_window`), and `spin_at_least` - the cycle
counter the power-up delay is spun on, so the memory can be brought up
before SysTick exists.

**`SdramConfig`** - the geometry and the reading discipline, one field
per SDCRx field: `columns` (8..11 bits), `rows` (11..13), `width` (8,
16 or 32), `internal_banks` (two or four), `cas` (1, 2 or 3 - the
enumerator's value IS the cycle count, because code 0 is reserved),
`write_protect`; then the three that go to SDCR1 whichever bank asks -
`clock` (off, HCLK/2, HCLK/3), `read_burst` (the six-line read FIFO;
default on, ES0206 2.3.4) and `read_pipe` (0, 1 or 2 HCLK).

**`SdramTiming`** - the seven delays IN MEMORY CLOCK CYCLES as a human
counts them, 1..16: `load_mode_to_active` (TMRD), `exit_self_refresh`
(TXSR), `self_refresh` (TRAS), `row_cycle` (TRC, SDTR1's),
`write_recovery` (TWR), `row_precharge` (TRP, SDTR1's),
`row_to_column` (TRCD). The register holds one less than each.

**`SdramInit`** - what the power-up sequence needs beyond those:
`power_up_us` (the delay of step 4), `refresh_cycles` (1..15 consecutive
auto-refreshes), `mode_register` (the DEVICE's word, carried in MRD),
`refresh_count` (SDRTR.COUNT, 0 to leave the timer alone) and
`both_banks` (every command issued to CTB1 and CTB2 at once, for a
two-device board).

**The arithmetic, as free constexpr functions**: `sdram_capacity_bytes`
(rows x columns x internal banks x width), `sdram_clock_hz` (HCLK over
the divider), `sdram_refresh_count` (37.7.5's formula, in 64 bits
because a 64 ms period times 90 MHz does not fit in 32),
`sdram_refresh_count_valid`, `sdram_forbidden_count`,
`sdram_mode_register` (the JEDEC SDR word: burst length, burst type, CAS
latency, single-location writes), and `sdram_config_valid` /
`sdram_timing_valid` / `sdram_init_valid`, which spell every reserved
code and both of TWR's inequalities.

**`FmcSdram<1|2>`** - one bank. `base` and `at<T>(offset)` for the
window; `configure` and `timing` for the two registers; `command` for
one SDCMR store bracketed by the BUSY waits it needs, with
`self_refresh`, `power_down` and `normal` as its named cases; `state`
for SDSR's per-bank field and `busy`/`wait_ready` for the flag they all
share; `write_protect`; `memory_clock` (SDCR1.SDCLK for both banks, with the
PALL before it); `refresh_rate` (with or without the timings, so
the forbidden value can be checked), `refresh_interrupt`,
`refresh_error`, `clear_refresh_error` and `refresh_isr` (the ISR body,
which answers whether the error was this interrupt's cause); and
`initialize`, the whole of 37.7.3 as one verb taking the core clock for
its power-up delay - and the way back from `Fmc::reset()` as well as the
way up from a cold block.

**`StaticConfig` / `StaticTiming`** - every field of BCRx, BTRx and
BWTRx: the memory type, the width, multiplexing, FACCEN, WREN, the two
burst enables, the wait signal's enable, timing and polarity, the
asynchronous wait, WRAPMOD where the part has it, the extended mode, the
CRAM page size and the continuous clock; and ADDSET, ADDHLD, DATAST,
BUSTURN, CLKDIV (2..16 HCLK), DATLAT (2..17 FMC_CLK) and ACCMOD. The
DEFAULTS ARE ERRATA WORKAROUNDS where there is one: the wait signal
enabled and active high (ES0206 2.3.2 and 2.3.9), the bus turnaround
nonzero (2.3.12).

**`FmcNorPsram<1..4>`** - one sub-bank of FMC bank 1: `configure`
(which leaves the bank DISABLED, so a window never answers before its
timings are written), `timing`, `write_timing`, `enable` / `disable` /
`enabled`, the three register read-backs, and `at<T>`.

**`FmcSram<n>` and `FmcNor<n>`** - the two tasks over it. `FmcSram::init`
takes a width and a timing (or two, for the extended mode) and pins the
memory type; `FmcNor` adds `init_multiplexed` and `init_burst` for the
two other shapes a NOR of this size is wired in. Both `release()` by
clearing MBKEN alone - the description and the timings stay, so a bank
re-enabled is the same bank; `Fmc::reset()` is what wipes the block.

**The NAND and PC Card halves have no verbs**: `Fmc::nand_banks`,
`Fmc::has_pc_card` and the two window constants are all this driver says
about them (see "Not covered yet").

## How to use it

An SDRAM brought up before anything else runs - the pads first, because
a command issued into pads that are still GPIO inputs reaches nothing:

```cpp
using Sdram = brio::FmcSdram<2>;                      // SDNE1, 0xD0000000

constexpr brio::SdramConfig geometry{
    .columns = brio::SdramColumns::eight, .rows = brio::SdramRows::twelve,
    .width = brio::SdramWidth::bits16,
    .internal_banks = brio::SdramInternalBanks::four,
    .cas = brio::SdramCas::three, .clock = brio::SdramClock::hclk_div2,
    .read_burst = true};
constexpr brio::SdramTiming timing{.load_mode_to_active = 2, .exit_self_refresh = 7,
                                   .self_refresh = 4, .row_cycle = 6,
                                   .write_recovery = 2, .row_precharge = 2,
                                   .row_to_column = 2};

constexpr uint32_t sdclk = brio::sdram_clock_hz(SysClock::hz, geometry.clock);
constexpr uint32_t count = brio::sdram_refresh_count(sdclk, 64'000, 4096);

board_claim_sdram_pads();                             // AF12, very high speed
const bool up = Sdram::initialize(clock, geometry, timing,
    {.power_up_us = 100, .refresh_cycles = 8,
     .mode_register = brio::sdram_mode_register(brio::SdramCas::three),
     .refresh_count = static_cast<uint16_t>(count)});
```

The memory as memory, once `up` is true - and not one instruction
before:

```cpp
volatile uint16_t* pixels = Sdram::at<uint16_t>(0);
pixels[y * width + x] = colour;
```

Two devices brought up together, which is what `both_banks` is for
(every command reaches CTB1 and CTB2, and the shared fields are stated
with the slower device's values):

```cpp
(void)brio::FmcSdram<1>::configure(geometry);         // its own SDCR1 fields
(void)brio::FmcSdram<2>::initialize(clock, geometry, slowest_timing,
                                    {.refresh_count = count, .both_banks = true});
```

The device asleep and awake again, with its contents:

```cpp
(void)Sdram::self_refresh();                          // the device keeps itself alive
...
(void)Sdram::normal();                                // or just read it: an access exits too
```

The memory clock's field cleared - what 37.7.5 asks for before the
divider or HCLK changes, with the PALL in front of it and the whole
sequence as the way back:

```cpp
(void)Sdram::memory_clock(brio::SdramClock::off);     // PALL, then SDCLK = 00
...
(void)Sdram::initialize(clock, geometry, timing, boot);   // the way back is the sequence
```

The refresh error reported instead of watched for:

```cpp
Sdram::refresh_interrupt(true);
brio::Fmc::enable_interrupt();
extern "C" void FMC_IRQHandler() { if (Sdram::refresh_isr()) { ++missed_refreshes; } }
```

An asynchronous SRAM on the second static sub-bank, with the extended
mode giving writes their own timings:

```cpp
using Ram = brio::FmcSram<2>;                         // FMC_NE2, 0x64000000
constexpr brio::StaticTiming reads{.address_setup = 3, .address_hold = 2,
                                   .data_phase = 6, .bus_turnaround = 1};
constexpr brio::StaticTiming writes{.address_setup = 1, .address_hold = 1,
                                    .data_phase = 3, .bus_turnaround = 1};
if (Ram::init(brio::StaticWidth::bits16, reads, writes)) {
    volatile uint16_t* cells = Ram::at<uint16_t>(0);
}
```

## Bench findings

`test_stm32f4_fmc` on the STM32F429I-DISC1, whose 64-Mbit SDRAM
(4096 rows x 256 columns x 4 internal banks x 16 bits = 8 MB) sits on
bank 2 with the core at 180 MHz and the memory clock at HCLK/2 = 90 MHz.
Every number below is that board's; the marches are byte-exact
everywhere. `ALL: 102 pass, 0 fail`. The same 102 on the
32F469IDISCOVERY, whose 128-Mbit IS42S32400F (the same rows, columns and
internal banks, 32 bits wide, 16 MB) sits on BANK 1 at 0xC0000000 with
SDNE0/SDCKE0 on PH3/PH2 and its upper sixteen data lines on ports H and
I: thirty-two data lines walk clean, the whole 16 MB marches byte-exact
in words, halfwords (373 ms written, 885 read) and bytes (560 / 1462),
and the controller of that part has one NAND bank and no PC Card
(RM0386 12.1), which letter a checks against the reserve.

**What the block holds at reset**, read with the gate opened and nothing
else touched: the AHB3 gate itself is CLOSED; SDCR1 = SDCR2 =
0x0000 02D0 (write protected, CAS 1, no memory clock); SDTR1 = SDTR2 =
0x0FFF FFFF (every delay at its slowest); SDRTR = 0 and SDSR = 0.
BCR1 = 0x0000 30DB - **bank 1 comes up ENABLED**, as a 16-bit
multiplexed NOR - and BCR2..4 = 0x0000 30D2, disabled SRAM; every BTR
and BWTR reads 0x0FFF FFFF.

**The initialization sequence costs 147 us**, of which 100 us is the
power-up delay it is asked to spin: 26499 core cycles at 180 MHz.
Command by command, each including the BUSY wait before and after:
clock enable 180 core cycles, precharge all 158, eight auto-refreshes
250, load mode register 158, normal 153. The mode register written is
0x0230 - burst length 1, sequential, CAS 3, single-location writes.

**The refresh rate that arithmetic produces** for 64 ms over 4096 rows
at 90 MHz is COUNT = 1386: a refresh request every 15.6 us, the whole
array in 63 ms.

**Eight megabytes, three access widths, no error.** Written and read
back, whole:

| access | write | read | write rate | read rate |
|--------|-------|------|------------|-----------|
| 32-bit words | 105 ms | 252 ms | 80 MB/s | 33 MB/s |
| 16-bit halfwords | 187 ms | 443 ms | 45 MB/s | 19 MB/s |
| 8-bit bytes | 279 ms | 735 ms | 30 MB/s | 11 MB/s |

A word written and read back as two halfwords and four bytes gives
0x5678 0x1234 and 78 56 34 12: the byte lanes and the controller's own
splitting agree, little-endian throughout. A checkerboard in both
phases and a moving inversion up and then down over all 2 M words
disturb nothing.

**Every line, one at a time.** Sixteen data lines walked at one address
plus both rails; a unique word at every power-of-two byte offset from 4
to 4 M, all read back afterwards, plus the last word of the device: no
alias, no stuck bit.

**The array holds.** Written, then read back untouched after 1 s and
after 11 s with the refresh running: clean both times, with no refresh
error raised.

**A COUNT of zero does not reach SDRTR.** Stored through the resource's
own register view - the driver refuses it - `SDRTR.COUNT` reads back
**40**, and so it does after a store of 10; the value is stable across
back-to-back reads and two milliseconds later, so it is a clamp and not
a live counter, and 41 and 1386 both read back exactly. The timer
therefore keeps running at 40 memory cycles where 37.7.5 promises no
refresh at all. That is one measurement on one part, and the driver's
answer to it is a refusal rather than a claim.

**Nor does clearing SDCR1.SDCLK stop the clock**, and the pad is what
says so. FMC_SDCLK is an ordinary pad in an alternate function, so its
input buffer reads the wire; two thousand samples a few core cycles
apart see **both levels while the controller runs, both levels with
SDCR1.SDCLK cleared, and one level (low) with the block held in
RCC_AHB3RSTR**. So the field named "SDCLK clock disabled" leaves the
clock running - and with it the refresh timer, which counts on it -
while the block's own reset stops both.

**Unrefreshed, 1 MB of this device starts losing bits between 3.2 and
6.4 seconds.** With the controller held in reset for a span that doubles
from 100 ms, 1 MB written beforehand came back exact through 3.2 s and
showed exactly ONE bad word at 6.4 s: 0xD00F F9FC read 0xF57D F0D4 where
0xF575 F0D4 was written, a single bit that should have been clear. The
sequence brings the device back after the reset with everything else
intact, which is why the sweep can rewrite and try again. That is 50 to
100 times the 64 ms the refresh rate is derived from - a
room-temperature figure from one device, where a retention
specification is a worst case over the whole temperature range, so it
says nothing about what to program. What it says is that a lost refresh
costs seconds and not milliseconds, which is worth knowing when a
program decides what a Stop or a reset really loses. That sweep is the
suite's one by-name letter: it doubles its dark span until a bit moves
and would otherwise lengthen every regression run.

**A CPU read with interrupts live was exact** over 64 KB, as was the
same read masked in 16 KB chunks - which is ES0206 2.3.5's workaround.
That is one measurement on one part and not a refutation of the
erratum: the suite's masked read is what the verdict is on.

**The refresh error could not be provoked.** At COUNT = 41, the floor of
the field - a refresh request every 456 ns against a device whose row
cycle is 6 memory cycles - a 64 KB march completed with SDSR.RE clear,
no interrupt taken and no bad word; so did 100 ms of power-down at the
same rate, where the controller has to leave the mode, precharge,
refresh and go back for every request. The flag and its vector are
implemented and stay unmeasured.

**The timing floor**, each SDTR field lowered alone with the others as
configured, each step followed by a precharge, a self-refresh round trip,
a mode-register load and a 64 KB march:

| field | in use | exact down to | what stopped the sweep |
|-------|--------|---------------|------------------------|
| TRC (row cycle) | 6 | 4 | the march fails at 3 |
| TRAS (self refresh) | 4 | 1 | the sweep's own end |
| TXSR (exit self refresh) | 7 | 1 | the sweep's own end |
| TMRD (load mode to active) | 2 | 1 | the sweep's own end |
| TRCD (row to column) | 2 | 2 | the driver refuses 1 |
| TRP (row precharge) | 2 | 2 | the driver refuses 1 |
| TWR (write recovery) | 2 | 2 | the driver refuses 1 |

Three of the seven are bounded from below by the CHAPTER and not by the
device: at TRCD = 1 or TWR = 1 the configured TRAS breaks
`TWR >= TRAS - TRCD`, and at TRP = 1 the configured TRC breaks
`TWR >= TRC - TRCD - TRP`. TRC is the one field with a measured floor,
and the configured 6 has two cycles of margin over it.

**The throughput ladder**, 4 KB copied word by word by the CPU, in core
cycles at 180 MHz:

| copy | cycles | bytes per 100 cycles |
|------|--------|----------------------|
| SRAM -> SRAM | 7252 | 56 |
| SRAM -> SDRAM | 8199 | 49 |
| SDRAM -> SRAM | 14648 | 27 |
| SDRAM -> SDRAM | 39332 | 10 |
| SRAM -> SDRAM, halfword by halfword | 12294 | 33 |

A read out of the device costs nearly twice what a write into it does -
the write posts into the controller's 16-entry write FIFO and the read
waits out the CAS latency - and a copy that uses the external bus at
both ends costs more than the two one-ended copies together. Moving the
same 4 KB in halfwords instead of words costs half as much again: the
element type is the beat.

**A DMA2 stream beats the CPU into the window and loses to it coming
out.** The same 4 KB, memory to memory, words, 4-beat memory bursts, on
a controller reset before each block: 4206 cycles in (97 bytes per 100
cycles, twice the CPU's rate) and 9660 cycles out (42 per 100, against
the CPU's 27). Byte-exact both ways. This is also the read ES0206 2.3.5
says the CPU cannot make safely.

**Self-refresh and power-down, entered and left with the data.** The
command puts SDSR.MODES2 at 1 and 2 respectively; after 200 ms in either,
a 64 KB slice comes back exact. **A single READ leaves self-refresh with
no command at all** - MODES2 reads 0 straight after it - which is 37.7.4
as written and worth knowing, because it means a stray access is enough
to wake the device.

**The static half, registers only**, with no device wired: a 16-bit SRAM
described into BCR2 = 0x7250 with BTR2 = 0x0131 0623 and its own
BWTR2 = 0x0001 0311 under the extended mode; every field decodes back,
`configure()` leaves MBKEN clear, and the four banks go back to their
reset values afterwards with the SDRAM none the wiser.

## Not covered yet

Driver gaps:

- **The NAND Flash controller (37.6) and the PC Card one** - PCR, SR,
  PMEM, PATT, PIO and the hardware ECC: no NAND and no PC Card socket on
  this desk, and the PC Card's own presence pin means a bank whose
  window faults until a card is in it. The reserve publishes which banks
  the part has so a driver can be written against a real device without
  touching it.
- **The FSMC of the F405 class, the F412 and the F413/F423** - the same
  static banks under other register names: another driver, born with the
  first board that has one.
- **`SYSCFG_MEMRMP` remapping the external memory to address 0** - a
  boot decision the BOOT pins already made; `stm32f4/syscfg.hpp` reads
  the field and does not write it.
- **A linker region over the SDRAM** - no `.sdram` output section and no
  crt hook, so an application reaches the device through a pointer and
  not through named objects. Placing data there needs three things this
  chapter does not provide: a MEMORY entry and an output section in
  `ld/stm32f429zi.ld`, the initialization sequence run BEFORE `.data` is
  copied (so the crt, not `main`), and a decision about erratum 2.3.5,
  which says the CPU must not read the FMC for variable data with
  interrupts live. Born with the first program that needs a framebuffer.
- **The read FIFO turned OFF** - `read_burst = false` is expressible and
  never exercised: ES0206 2.3.4 makes it the wrong setting for a 16-bit
  device, and the driver's default is on.
- **`BCR1.WFDIS`, the write FIFO's off switch on the F446, F469 and
  F479** - declined: its reset value is the only setting ES0298 2.3.4
  allows, no header of the pack declares a mask for it, and a verb whose
  one legal argument is the reset value is worse than none. A program
  that wants the write FIFO gone for a device that cannot take posted
  writes would add it to the reserve keyed on the part class, the way
  the frequency ladders are.

Implemented but not bench-verified:

- **The SDRAM refresh error and its interrupt** - the flag, REIE, the
  CRE clear and the ISR body are all in the driver; neither probe that
  the suite can make (COUNT at the field's floor under a march, and the
  same through a power-down) provoked SDSR.RE on this device. A second
  bus master saturating the controller while the counter is at its
  floor, or a device whose row cycle does not fit the interval, would
  raise it.
- **`both_banks`, the CTB1|CTB2 command of a two-device sequence** -
  each board carries one SDRAM (the DISC1's on bank 2, the
  32F469IDISCOVERY's on bank 1), so the two-bank form of every command
  is compiled and never issued; a board with a device on each chip
  select would measure it.
- **The 8-bit SDRAM width, the 11- and 13-bit row counts, the 9..11-bit
  column counts, CAS 1 and 2, HCLK/3, and the read pipe** - the whole
  option space of SDCRx compiles and is refused where the chapter
  reserves a code, but only the two boards' geometries (16 and 32 bits
  wide, 12 rows, 8 columns, 4 banks, CAS 3, HCLK/2, no pipe) have been
  read through. Another device on either board's pads would measure them.
  `memory_clock()` is exercised as a STOP and a restart at the same
  rate; a change from HCLK/2 to HCLK/3 on a live device - the field's
  actual purpose - would need the seven delays recomputed for the new
  period first, which is the program-wide decision the driver declines
  to make (see "the SDRAM clock is made here").
- **Everything in the NOR/PSRAM controller past the register write** -
  the four asynchronous access modes, the synchronous burst, the
  multiplexed bus, the continuous clock, the CRAM page split, the wait
  signal in either polarity and the bus turnaround: no static memory is
  wired to this board, so letter n proves the fields go where they are
  meant to and nothing about a transaction. A NOR or a PSRAM on
  FMC_NE2..NE4 would measure them, and would be where ES0206 2.3.2,
  2.3.6, 2.3.9, 2.3.11 and 2.3.12 stop being comments and become
  measurements.
- **A framebuffer's worth of traffic from a second master while the CPU
  works** - the LTDC reading the same device the program writes is what
  will decide whether the numbers above hold under contention, and the
  LTDC has no driver yet. The DMA letter is one master at a time.
