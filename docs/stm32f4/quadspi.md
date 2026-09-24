# Quad-SPI memory interface (STM32F4)

Documents of record: RM0386 Rev 6 ch. 13 (the F469/F479's, the chapter
this driver was written from) and its twin RM0390 Rev 6 ch. 12 (the
F446's: one register file under one name, and the same two facts the
header does not carry - the window's address and the DMA cell - stated
by both); the errata's QUADSPI items, ES0321 Rev 14 2.4.1..2.4.5 and
ES0298 Rev 8 2.4.1..2.4.5, the same five in the same words; and, for
the device the bench suite is measured against, Micron's MT25QL128ABA
datasheet Rev. K (its table 18 for the commands, tables 3..11 for the
registers, table 9 for the dummy cycles a clock needs, table 44 for the
clock ceilings). Driver: `stm32f4/quadspi.hpp` (`Quadspi`,
`QspiCommand`, `QspiConfig`, `QspiLines`, `QspiWidth`, `QspiStatus`,
`QspiEvents` and the arithmetic `qspi_clock_hz`, `qspi_prescaler_for`,
`qspi_cs_high_cycles_for`, `qspi_address_bits`), over the reserve's
`quadspi_present`, `quadspi_clock_mask`, `quadspi_reset_mask`,
`quadspi_irq`, `quadspi_window` and `quadspi_dma_placements`
(`stm32f4/device_tables.hpp`). The family fixture is
`test/family_stm32f4/quadspi.cpp` with the negatives that refuse the type
on a part with no block and a configuration past the fields. Bench:
`test_stm32f4_qspi` on the 32F469IDISCOVERY.

## What the silicon does

**A command engine in front of a serial flash, with three faces.** Every
command is up to five phases - an instruction, an address of one to four
bytes, one to four alternate bytes, a run of up to 31 dummy cycles, and
the data - and each phase goes out on one, two or four lines or is
skipped, so that one block speaks single-SPI to a status register and
quad-SPI to the array in the same breath (13.3.3, 13.3.4). What differs
between the three FUNCTIONAL MODES (13.3.11) is who moves the data: in
INDIRECT mode the program writes the registers and pumps a 32-byte FIFO
through the data register, a byte, a halfword or a word at a time; in
AUTOMATIC STATUS-POLLING mode the block repeats a read of up to four
bytes every N clocks on its own, masks the answer and raises SMF when
the masked bits match, ANDed or ORed, stopping there if APMS is set; in
MEMORY-MAPPED mode every load from the window at 0x9000 0000 becomes the
programmed read command, the FIFO turns into a prefetch buffer, and the
flash is memory to the Cortex and to a DMA alike - reads only, and a bus
error for a load outside the flash's size or with the block disabled.
Eight of the pack's twenty-three headers declare the block; the F412,
F413/F423, F446 and F469/F479 have it and the rest have nothing.

**A command starts on the last register it needs.** 13.3.5's rule, and
the one that shapes every verb: the write to CCR starts a command that
has no address and no data to give, the write to AR starts one that has
an address and nothing to send, the first write to DR starts one that
has data to send. ABR never starts anything, so the alternate bytes go
in first; DLR is the length and goes in before CCR. A consequence the
bench found the hard way: WRITING AR IS NOT AN IDLE STORE. Under an
indirect read command a write to AR is a read at that address, and
while BUSY stands a write to AR is ignored (13.5.7) - which is why the
errata's "clear AR before memory-mapped mode" is done here behind an
abort (below).

**The clock is HCLK divided.** CLK = HCLK / (PRESCALER + 1) (13.5.1), an
odd divisor leaving the clock low one cycle longer than high, and the
device's ceiling is met by the divisor a program chooses knowing HCLK:
at 180 MHz the MT25QL128's 133 MHz wants PRESCALER 1 (90 MHz) for every
command but READ 03h, whose own ceiling of 54 MHz wants PRESCALER 3
(45 MHz). SSHIFT samples half a cycle later for a slow board, and must
be clear in DDR mode. DDR itself needs a divisor of two or more.

**The flash's size is part of the contract.** FSIZE + 1 is the number of
address bits (13.3.8); an indirect access at or across that size raises
TEF as soon as it is triggered (13.3.13), and a memory-mapped one is a
bus error to whoever made it. CSHT + 1 is the least number of cycles NCS
stays high between commands, a device parameter in disguise (the
MT25QL128 wants 50 ns after a non-read command, five cycles at 90 MHz).
CKMODE says whether CLK idles low or high.

**BUSY means a different thing per mode** (13.3.14): done and the FIFO
empty in indirect mode; up between the periodic reads of status polling
until a match with APMS or an abort; up from the first memory-mapped
access, prefetching with NCS low, until an abort or a timeout. ABORT is
the one way down that works everywhere, and it flushes the FIFO and sets
TCF. The FIFO, in indirect read mode, fills to 32 and then THE BLOCK
STOPS THE CLOCK until the program reads (13.3.5) - measured: a 64-byte
read left alone parks at FLEVEL 32 with BUSY up and TCF down, and reading
32 bytes lets the other 32 in.

**Five errata, the same on both sheets read, four of them code.**
- *2.4.2, first nibble of data not written after dummy phase*: an
  indirect WRITE with dummy cycles loses its first nibble. `write()`
  refuses a command with dummy cycles; a device that wants latency
  before the data of a write gets it as alternate bytes, the sheet's own
  workaround.
- *2.4.3, wrong data from memory-mapped read after an indirect mode
  operation* when AR's two low bits are set on entry. `map()` clears AR
  and aborts before it writes the memory-mapped command - and aborts
  BEFORE the clear too, because the sheet's recipe assumes an idle block
  and a write to AR is ignored while BUSY stands: staged with AR = 3 and
  a read still stalled in the FIFO, the plain recipe left AR at 3, the
  guarded one leaves it at 0.
- *2.4.4, memory-mapped read operations may fail when timeout counter is
  enabled*: TCEN is never set - `QspiConfig` has no timeout option - and
  NCS is raised by `unmap()`'s abort, the sheet's workaround.
- *2.4.1, extra data written in the FIFO at the end of a read transfer*
  (quad, DDR, PRESCALER 1): `read()` drains whatever the FIFO holds after
  the transfer and discards it, which costs nothing on the transfers the
  item does not touch.
- *2.4.5, memory-mapped access in indirect mode clearing QUADSPI_AR*: a
  load a PROGRAM makes from the window while the block is in indirect
  mode; no code covers it, `window()` is handed out by `map()`, and the
  verbs write AR right before every command they start.

**The poller and the handler must not want the same flag.** TCF is what
an interrupt handler reports and clears; a blocking verb that waited for
TCF would be robbed by that handler and time out - measured: with TCIE
armed over a 16-byte read, the wait ran out, the abort of the failure
path set TCF anew, and the vector was entered twice for one read. The
blocking verbs therefore complete on BUSY falling, and `isr()` clears
the four flags it reports.

## Types and verbs

- `QspiLines` (`none`, `one`, `two`, `four`) and `QspiWidth` (`bits8`,
  `bits16`, `bits24`, `bits32`): the codes of IMODE/ADMODE/ABMODE/DMODE
  and of ADSIZE/ABSIZE (13.5.6).
- `QspiCommand`: one command as the chapter draws it - `instruction` and
  `instruction_lines`, `address_lines` and `address_width`,
  `alternate_lines`, `alternate_width` and `alternate`, `dummy_cycles`,
  `data_lines`, `ddr`, `ddr_hold`, `instruction_once`. The functional
  mode is the verb's, so one read command serves an indirect read and
  the window alike. `qspi_command_ok` is the chapter's rules (at most 31
  dummy cycles, at least one phase), `qspi_read_needs_turnaround` the
  dummy cycle a two- or four-line read must have.
- `QspiConfig`: `prescaler`, `sample_shift`, `fifo_threshold` (1..32
  bytes), `address_bits` (FSIZE + 1, 1..32), `cs_high_cycles` (CSHT + 1,
  1..8), `clock_mode3`; `qspi_config_ok` the fields' bounds. The
  arithmetic beside it: `qspi_clock_hz(hclk, prescaler)`,
  `qspi_prescaler_for(hclk, max_hz)` (the fastest legal divisor),
  `qspi_cs_high_cycles_for(clk, ns)` (0 when eight cycles are not enough),
  `qspi_address_bits(bytes)` (0 for a size that is not a power of two).
- `Quadspi`, the monostate over the block where the header declares it:
  `clock`, `reset_block`; `init<cfg>()` (static_assert) and `init(cfg)`
  (false and nothing written outside the fields) - the gate, an abort of
  anything in flight, CR and DCR, the enable; `release`; the readbacks
  `prescaler`, `fifo_threshold`, `address_bits`, `cs_high_cycles`; the
  status `busy`, `fifo_level`, the five flags and `clear_flags`;
  `abort(spins)`. Then the modes: `command(c, address)` for a command
  with no data phase, `write(c, address, data, len)` and
  `read(c, address, data, len)` in indirect mode, each answering a
  `QspiStatus` (`ok`, `refused` with nothing written, `error` for TEF,
  `timeout` for a wait that ran out - and a failed command aborted so the
  block is at rest); `poll(c, address, bytes, mask, match, interval,
  match_any, status)` for automatic status polling, stopping on the match
  (APMS) and handing back the last bytes read; `map(c)`, `mapped()`,
  `unmap()` and `window()` for memory-mapped mode, `map` refused where the
  manual gives no window; `interrupts(QspiEvents)` and the ISR body
  `isr()`, which reports the five events and clears the four that are
  flags. `ccr_word(c, fmode)` is the command word, canonical - a skipped
  phase leaves its width field zero too.
- The reserve's facts: `quadspi_present()`, the AHB3 gate and reset
  masks, `quadspi_irq()`, `quadspi_window()` (known and 0x9000 0000 of
  256 MB on the F446 and the F469/F479, unknown on the F412 and F413/F423
  whose manuals are not on the desk), `quadspi_dma_placements()` /
  `quadspi_dma_placement_valid` (the one cell, DMA2 stream 7 channel 3,
  known on the same two classes).

## How to use it

The device's words stay with the device: every opcode, dummy count and
register below is the MT25QL128ABA's table 18, spelled as a
`QspiCommand` beside the program that talks to it - the driver knows
phases and lines and nothing of any device.

```cpp
#include "stm32f4/quadspi.hpp"
using namespace brio;

constexpr QspiConfig cfg{
    .prescaler = qspi_prescaler_for(SysClock::hz, 133'000'000u),   // 90 MHz at 180
    .fifo_threshold = 4,
    .address_bits = qspi_address_bits(16u * 1024u * 1024u),        // 24
    .cs_high_cycles = qspi_cs_high_cycles_for(90'000'000u, 50u)};  // 5

constexpr QspiCommand write_enable{.instruction = 0x06};
constexpr QspiCommand read_status{.instruction = 0x05, .data_lines = QspiLines::one};
constexpr QspiCommand page_program{.instruction = 0x02, .address_lines = QspiLines::one,
                                   .data_lines = QspiLines::one};
constexpr QspiCommand quad_io_read{.instruction = 0xEB, .address_lines = QspiLines::four,
                                   .dummy_cycles = 10, .data_lines = QspiLines::four};

// the six pads in their alternate functions first (the board's), then:
Quadspi::init<cfg>();
(void)Quadspi::command(write_enable);
(void)Quadspi::write(page_program, 0x1000, data, 256);
uint32_t last = 0;   // WIP down: RDSR every 16 clocks, bit 0 against 0, stop on the match
(void)Quadspi::poll(read_status, 0, 1, 0x01u, 0x00u, 16, false, last);

if (Quadspi::map(quad_io_read)) {           // then the flash is memory
    const volatile uint8_t* flash = Quadspi::window();
    const uint8_t b = flash[0x1000];
    (void)Quadspi::unmap();                 // before the next indirect command
}
```

An interrupt-driven program binds `QUADSPI_IRQHandler` to a body that
calls `Quadspi::isr()` and arms `interrupts({.transfer_complete = true})`
around the transfer; it does not then wait on the blocking verbs' flags,
which complete on BUSY.

## Bench findings

`test_stm32f4_qspi` on the 32F469IDISCOVERY (STM32F469NI at 180 MHz,
the block at HCLK/2 = 90 MHz) against the Micron MT25QL128ABA the board
wires to bank 1 - CLK PF10, NCS PB6 under the board's 10 k pull-up, IO0
PF8, IO1 PF9, IO2 PF7, IO3 PF6: **ALL: 60 pass, 0 fail**, twice.

- **The block at reset**: the AHB3 gate closed, every one of the eleven
  registers zero (table 94); after `init` CR and DCR read back PRESCALER
  1, FTHRES 3, FSIZE 23, CSHT 4, and the vector is 91.
- **The device answers as its datasheet says**: READ ID 20 BA 18 10 40
  00 and fourteen bytes of unique id (table 16; the extended id's bits
  1:0 = 00, a uniform 64 KB sector device); the SFDP table opens with
  'SFDP', revision 1.6, two parameter headers; the status register 00h,
  the flag status register 80h (ready, no error), the nonvolatile
  configuration register FFFFh (the delivery state, never written), the
  volatile one FBh (dummy cycles at default, XIP off, continuous wrap),
  the enhanced volatile one FFh - the chip as Micron ships it, plus the
  4 KB the suite programs.
- **The last subsector, erased and programmed**: WRITE ENABLE sets the
  latch; the 4 KB SUBSECTOR ERASE at 0xFFF000 completes in 3.7 ms on a
  blank subsector and 15.6..16.7 ms on one holding data (tSSE 50 ms
  typical, 400 maximum), the block's AUTOMATIC STATUS-POLLING mode
  seeing WIP fall and stopping on the match; sixteen PAGE PROGRAMs of 256
  bytes at 183 us a page (188 at most; tPP 120 us typical, 1800 maximum),
  each behind its own WRITE ENABLE; the latch clear afterwards; the 4 KB
  read back byte-exact.
- **Five read commands, one array**: READ 03h at 45 MHz, FAST READ 0Bh,
  DUAL OUTPUT 3Bh, QUAD OUTPUT 6Bh and QUAD I/O EBh at 90 MHz all give
  the same 4 KB, in 731 / 367 / 186 / 114 / 113 us - the one- and two-line
  reads at the wire's pace (4 KB at 90 MHz on one line is 364 us), the
  four-line ones at the pump's (36 MB/s against the wire's 45, the level
  read once a burst and the FIFO taken in words).
- **The dummy cycles against table 9**: quad I/O at 90 MHz is byte-exact
  with the default ten dummy cycles and with the seven the table allows
  (97 MHz); with six (86 MHz in the table) it still read right on this
  part at room temperature - margin the table does not promise, printed
  and not judged. The volatile configuration register takes each count
  and gives it back.
- **The FIFO and the flags**: a read left alone parks at FLEVEL 32 with
  FTF set, BUSY up and TCF down, and the clock stalls until the program
  reads; a word read of DR is four bytes, the first at the bottom; TCF
  rises with the 64th byte out and CTCF clears it; an abort eight bytes
  into a 4 KB read brings BUSY and ABORT down and empties the FIFO, and
  the next read is whole; a read at the flash's size, or crossing it,
  raises TEF and ends as an error, while the last sixteen bytes read
  fine.
- **The interrupt**: TCIE armed over a 16-byte read enters the vector
  ONCE, the body reporting the transfer complete - after the finding
  above about the flag a poller must not share with the handler.
- **Memory-mapped mode**: with quad I/O and ten dummies as the read,
  every byte of the subsector reads through 0x9000 0000 + 0xFFF000 as
  the pattern, halfword and word loads agree, a 4 KB copy in words takes
  92 us (44.5 MB/s - the wire); BUSY stands after an access (the
  prefetch), AR reads 0 once mapped and then the address the bus last
  fetched (the register follows the bus in this mode, it is not the
  program's), `unmap()` brings BUSY down and an indirect read works
  again.
- **Deep power-down**: after B9h the device answers FF FF FF to READ ID;
  after ABh and 30 us the identity is back.

## Not covered yet

Driver gaps:
- **DDR mode and DHHC**: the bits are in the command word and the
  turnaround rule is enforced, but no DDR command was sent - declined for
  this round: the device's DTR reads exist (0Dh, 6Dh, EDh) and would be
  worth a letter with 2.4.1 staged on purpose (quad, DDR, PRESCALER 1).
- **Dual-flash mode (DFM, FSEL, BK2)**: one flash on this board, so the
  second bank's pads and the even-length rule are neither exposed nor
  measured; a board with two devices would.
- **A DMA engine slot** (DMAEN over the request cell the reserve knows):
  the indirect verbs pump the FIFO with the CPU; a `DmaRxEngine`/`DmaTxEngine`
  pair on DMA2 stream 7 channel 3 is the next step when a program needs
  the core elsewhere during a transfer.
- **SIOO with the device's XIP mode**: `instruction_once` is in the word
  and never used, because the MT25QL128's XIP mode is entered through a
  confirmation bit and, left on, outlives a core reset - a device state a
  bench that nobody watches must not leave behind (its rescue sequence
  is the datasheet's, not this driver's).
- **The timeout counter (TCEN, LPTR)**: declined, ES0321/ES0298 2.4.4.
- **The polling interval and status match as interrupts**: `poll()`
  waits; SMIE with `isr()` is compiled and not driven.
- **The bus error of a memory-mapped access outside the flash's size**:
  the manual's promise, not staged - it needs a fault handler that
  returns, which the reset chapter's `Faults` could give a letter.
- **The F446's block**: it compiles on every header that has one and the
  F446 shares this class's window and cell, but the Nucleo-F446RE carries
  no serial flash; a device on its Arduino header would measure it.
- **The F412 and F413/F423**: the block is there, the window and the DMA
  cell are refused for want of RM0402 and RM0430.

Implemented, not bench-verified:
- `clock_mode3` (CKMODE = 1): written and read back, never driven -
  the MT25QL128 accepts mode 0 and mode 3 alike; a letter reading the
  identity under mode 3 would measure it.
- `sample_shift`: the bit is written; a board whose signal delay needs
  it would show a read that is wrong without it.
- `interrupts` beyond TCIE: TEIE, SMIE, FTIE and TOIE are set and
  cleared and never entered.
