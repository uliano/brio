# PIO (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), chapter 11
whole - 11.1.1 (what this chip added to the RP2040's PIO), 11.2 (the
programmer's model: the programs, the control flow, the registers and
shift counters, the FIFOs, the stalls, the pin mapping, the flags), 11.4
(the instruction set and its encoding, including the forms that are
new), 11.5 (the functional details: the side-set, the wrap, the FIFO
join, autopush and autopull, the clock dividers, the GPIO mapping and its
priority, the forced instructions), 11.6 (the examples), 11.7 (the
registers) - and 9.4 for the pin functions. Appendix E carries no erratum
against this block; RP2350-E9, which is the pads', is why no letter of
the suite reads an undriven pin. The driver: `brio/rp2350/pio.hpp` (the
assembler, `Pio<n>` the block, `PioSm<n, sm>` the machine, the four
tasks) over `pin.hpp`, `resets.hpp`, `core.hpp` and `dma_engine.hpp`; and
`util/pwm_channel.hpp` for the PwmChannel the PIO PWM satisfies. The
reference suite: `test_rp2350_pio` - letters a, b, h and j wireless, c,
d, e, g, i, k and l on two of the standing wires, and f on both. Two of
those letters put a [dma.md](dma.md) engine on a machine's FIFO, which is
what the request numbers here are for.

## What the silicon does

THREE BLOCKS of four state machines each - twelve machines against the
RP2040's eight - running sixteen-bit programs from a shared memory of
thirty-two instructions. Nine instructions - JMP, WAIT, IN, OUT, PUSH,
PULL, MOV, IRQ, SET -, every one taking one cycle of the machine's clock
and carrying, in its bits 12:8, a delay of up to 31 cycles and a side-set
of up to five pins, the split between the two the machine's
configuration. A machine has two 32-bit scratch registers, an output and
an input shift register with a counter each (autopull refills the OSR and
autopush empties the ISR at a threshold), a four-entry FIFO each way, a
16.8 clock divider off clk_sys (1 to 65536), four pin ranges - OUT, SET,
IN, side-set - each a base and a count modulo 32, a pin to branch on, and
a sticky option that keeps the last output asserted. Eight flags are
shared by a block's machines, raised, cleared and waited on by IRQ and
WAIT. Each FIFO is a DMA request. On one cycle the highest-numbered
machine writing a pin wins, and a machine's side-set beats its own OUT or
SET. An instruction written to SMx_INSTR executes at once, the program
resuming after - and a DISABLED machine executes those and nothing else,
which is how a program is started (a JMP to its offset) and how pins are
claimed before it runs (a SET). A JMP holds an absolute address, so a
program loaded at an offset has its JMPs relocated. All of that is the
RP2040's chapter too.

WHAT IS THIS CHIP'S OWN (11.1.1), and what a driver has to say about each:

- **A VERSION FIELD.** DBG_CFGINFO's top four bits read 1 here and 0 on
  the RP2040, where they were reserved. It is the only thing in the
  chapter that lets a program ask the silicon whether the rest of this
  list exists.
- **GPIOBASE, and it changes what every pin number means.** A block still
  sees 32 GPIOs at a time; GPIOBASE picks WHICH 32, and only the values 0
  and 16 are writable, so the two windows are GPIO0..31 and GPIO16..47.
  The register's own description is that it relocates GPIO 0 FROM PIO'S
  POINT OF VIEW, so everything a machine names in pin terms - the four
  ranges of PINCTRL, EXECCTRL's JMP_PIN, the index of a WAIT GPIO - is
  counted from there,
  so on the QFN-80 a pad above GP31 is reachable only with the window
  moved, and with it moved a pad below GP16 is not reachable at all.
  Sixteen pads, GP16..GP31, are in both windows.
- **CTRL reaches the NEIGHBOURING BLOCKS.** Three new bits say "do this
  to the machines named by NEXT_PIO_MASK and PREV_PIO_MASK as well":
  enable, disable, restart the clock divider. The blocks are a RING -
  PIO2's next is PIO0, PIO0's previous is PIO2 - and the point of the
  feature is SIMULTANEITY: one bus write starts machines of two blocks on
  the same cycle, which nothing else on this chip arranges.
- **ALL EIGHT FLAGS REACH THE INTERRUPT LINES.** IRQ0_INTE and IRQ1_INTE
  are sixteen bits here: four receive-FIFO sources, four transmit-FIFO
  sources, and the eight flags. On the RP2040 the upper four flags were
  visible to the machines and to no interrupt.
- **SHIFTCTRL.IN_COUNT** masks the IN-mapped pins above a count to zero.
  It matters most to `MOV X, PINS`, which otherwise has no way at all to
  say how many pins it meant and returns the whole rotated bank.
- **THE RECEIVE FIFO AS FOUR REGISTERS.** FJOIN_RX_PUT disables the
  receive queue and hands its four cells to the machine as randomly
  addressed storage (the PUT instruction), with the system reading them
  at RXFn_PUTGETm: a status register a program keeps up to date and
  nobody blocks on. FJOIN_RX_GET is the mirror - the system writes, the
  machine reads (GET) - and is how a program is given a parameter while
  it runs. Both together give the machine four more scratch registers and
  shut the system out. Setting either clears FJOIN_TX and FJOIN_RX, and
  changing any join discards what the FIFOs held.
- **FOUR NEW INSTRUCTION FORMS** (11.4), all of them sharing opcodes with
  instructions the RP2040 had: WAIT on JMP_PIN plus an offset of 0..3
  (source 3, which was reserved), so a WAIT can use the machine's own
  branch pin instead of the IN mapping; MOV to PINDIRS (destination 3,
  reserved), which turns every OUT-mapped pin around in one instruction
  and past a SET's five-pin limit; the IRQ flags as a source for
  `MOV X, STATUS`, so a program can BRANCH on a flag where before it
  could only block on one; and an IRQ index whose two most significant
  bits choose a scope - this block, the previous block, this block
  relative to the machine's number, the next block - with no cycle of
  delay between blocks. That last one is where the RP2040's single
  `relative` modifier went: relative is one of the four scopes, and it
  keeps its old bit pattern.

Each FIFO is still a DMA request, and PIO0's and PIO1's sixteen keep the
numbers they had on the RP2040; PIO2's eight are new behind them, which is
what pushed every other peripheral's request up the table
(`rp2350/dma_engine.hpp`). PIO0, PIO1 and PIO2 are functions 6, 7 and 8,
and EVERY pad of this chip offers all three (9.4).

## Types and verbs

- THE ASSEMBLER: `PioInstr`, `pio_jmp(cond, address)`, `pio_wait(
  polarity, on, index)`, `pio_wait_irq(polarity, flag, scope)`,
  `pio_wait_jmppin(polarity, offset)`, `pio_in(from, bits)`, `pio_out(to,
  bits)`, `pio_push(if_full, block)`, `pio_pull(if_empty, block)`,
  `pio_put(entry)` / `pio_put_y()`, `pio_get(entry)` / `pio_get_y()`,
  `pio_mov(to, from, op)`, `pio_irq(flag, clear, wait, scope)`,
  `pio_set(to, data)`, `pio_nop()`, with `PioJmp`, `PioWaitOn` (whose
  fourth source is `jmppin`), `PioIn`, `PioOut`, `PioMovTo` (whose third
  destination is `pindirs`) / `PioMovOp` / `PioMovFrom`, `PioSetTo` and
  `PioIrqScope` (here, prev, relative, next); `PioSideSet` (count,
  optional, pindirs) and `pio_side_delay(instr, side, value, delay)` /
  `pio_delay`; `pio_relocate(instr, offset)`; `PioProgram<N>` (the code,
  the wrap points, the side-set shape; `valid()`); `PioClockDiv` (16.8, 0
  for 65536) with `pio_clock_div_for(sys_hz, hz)` and `pio_sm_hz`. Every
  encoding in this list is pinned at compile time against the words the
  vendor's own assembler produces.
- `PioSmConfig` (the divider, the wrap points, the side-set, the JMP pin,
  sticky and inline-enable outputs, the STATUS source, the FIFO join,
  both thresholds and directions, autopull and autopush, THE INPUT COUNT,
  the four pin ranges) with `pio_sm_config_valid` and the three register
  words - and every pin field in it is the BLOCK'S index, not a system
  GPIO number; `PioFifoJoin` (none, tx, rx, rx_put, rx_get, rx_putget);
  `PioStatus` (tx_level, rx_level, irq_flag) with `PioStatusIrq` and
  `pio_status_irq(flag, where)` for the STATUS_N value an IRQ source
  wants; `PioInterrupt::rx_not_empty(sm)` / `tx_not_full(sm)` / `flag(f)`
  for f in 0..7 / `all_flags` / `all`; `PioNeighbours` (the two four-bit
  masks a CTRL write carries).
- `Pio<n>` for n in 0..2: `index`, `next_index` / `prev_index` (the
  ring), `reset` / `hold`, `pin_function`, `irq(line)`, `dreq_tx(sm)` /
  `dreq_rx(sm)`; the window: `gpio_base()` read and written,
  `pin_index(gpio)`, `sees(gpio)`, `gpio_window`; the memory:
  `load(program, offset)`, `place`, `add` (first fit), `unload`,
  `used_slots`; the machines: `enable(mask)` / `disable` / `enabled`,
  `restart(mask)`, `restart_clocks(mask)`, and the three that reach the
  neighbours - `enable_across`, `disable_across`,
  `restart_clocks_across`; the status words: `fifo_status`, `fifo_debug`
  / `clear_fifo_debug`, `fifo_levels`, `version` / `memory_size` /
  `machine_count` / `fifo_depth` (DBG_CFGINFO), `pad_out` / `pad_oe`
  (what the block is driving, level and direction), `sync_bypass`; the
  flags: `flags`, `flag(f)`, `clear_flags`, `raise_flags` (IRQ_FORCE,
  visible to the machines); the lines: `interrupts(line, mask, on)`,
  `interrupts_only`, `raw_pending`, `pending(line)`, `force`, `isr(line)`
  (the raised-and-enabled sources, a flag source cleared).
- `PioSm<n, sm>`: `configure(config)` (the machine disabled first),
  `init(program, offset, config)` (the wrap points from the program,
  restarted, the FIFOs drained, the PC at the offset - still disabled),
  `enable(on)`, `restart`, `clock(div)`, `exec(instr)` / `exec_wait` /
  `exec_stalled`, `address()`, `instruction()`, `pin_directions(base,
  count, output)` / `pin_levels(base, count, high)` (a SET on the side
  with PINCTRL borrowed; `base` is the block's index), the FIFOs:
  `tx_full` / `tx_empty` / `rx_full` / `rx_empty`, the levels, `push` /
  `pop` and their bounded waits, `drain_rx` / `drain_tx`, the four FDEBUG
  flags and their clear, `tx_address` / `rx_address` /
  `rx_top_byte_address` (the top byte of an entry for a byte engine),
  `putget(entry)` (the system's end of the PUT and GET modes), `dreq_tx`
  / `dreq_rx`, the two interrupt source masks.
- THE TASKS, which take SYSTEM GPIO numbers and convert them through the
  block's window, refusing a pad the window does not cover:
  `PioUartTx<n, sm, pin>` (init(clock, baud): 11.6.3's program, eight
  cycles a bit, the FIFO joined; `write`, `writable`, `write_wait`,
  `idle`, `tx_address`), `PioUartRx<n, sm, pin>` (init(clock, baud,
  pull): 11.6.4's program with the frame check; `read` (the top byte),
  `readable`, `read_wait`, `frame_error()` - flag 4 + sm -,
  `rx_top_byte_address`), `PioSquareWave<n, sm, pin>` (init(clock, hz): a
  four-cycle wave), `PioPwm<n, sm, pin, period>` (a PwmChannel:
  `init(divider)`, `duty`; a pulse of (period + 1) x 3 machine cycles).
  Every other program is the application's, written with the encoders.

## How to use it

```cpp
using Tx = brio::PioUartTx<0, 0, 13>;         // PIO0, machine 0, GPIO 13
using Rx = brio::PioUartRx<0, 1, 15>;
Tx::init(clock, 115'200);
Rx::init(clock, 115'200);
Tx::write_wait('A');
if (const auto b = Rx::read_wait()) { got(*b); }

using Blink = brio::PioSquareWave<2, 0, 17>;   // the third block
Blink::init(clock, 1'000'000);                 // a 1 MHz wave, the divider solved
```

A pad above GP31, which wants the window moved first - and moving it
moves every machine of the block:

```cpp
brio::Pio<1>::gpio_base(16);                   // now GPIO16..47
using Far = brio::PioSquareWave<1, 0, 40>;
Far::init(clock, 100'000);                     // refuses if the window does not cover GP40
```

A program of one's own, with the receive FIFO as a status register the
program updates and nothing blocks on:

```cpp
constexpr brio::PioProgram<5> counter = [] {
    brio::PioProgram<5> p{};
    p.code = {
        brio::pio_wait(true, brio::PioWaitOn::pin, 0),    // a rising edge
        brio::pio_wait(false, brio::PioWaitOn::pin, 0),   // and its falling one
        brio::pio_mov(brio::PioMovTo::isr, brio::PioMovFrom::y),
        brio::pio_put(0),                       // into cell 0: this never stalls
        brio::pio_jmp(brio::PioJmp::y_dec, 0),  // Y counts the periods, downwards
    };
    return p;
}();
using Sm = brio::PioSm<0, 2>;
const auto at = brio::Pio<0>::add(counter);
brio::PioSmConfig c{};
c.clock = *brio::pio_clock_div_for(clock_hz(clock), 1'000'000);
c.in_base = *brio::Pio<0>::pin_index(15);
c.fifo_join = brio::PioFifoJoin::rx_put;
Sm::init(counter, *at, c);
Sm::enable(true);
const uint32_t periods = 0u - Sm::putget(0);    // read whenever, from anywhere
```

Two blocks started on the same cycle:

```cpp
// PIO0's machine 0 and PIO1's machine 3, in one write to PIO0's CTRL
brio::Pio<0>::restart_clocks_across(0x1, {.next = 0x8});
brio::Pio<0>::enable_across(0x1, {.next = 0x8});
```

## Bench findings

All of them on an RP2350 in the QFN-80 package, **stepping A2**, clk_sys
at 150 MHz, and all of them on BOTH architectures: `test_rp2350_pio`
reports **30 pass, 0 fail** on the Cortex-M33 pair and on the Hazard3
pair, from one source, each on three flash-and-run cycles. No verdict of
this chapter reads differently between the halves.

- **VERSION 1 ON ALL THREE BLOCKS**, with thirty-two instructions, four
  machines and FIFOs four deep, which is the whole of DBG_CFGINFO; PIO0
  out of reset reads CTRL 0, FSTAT 0x0F000F00 (both FIFOs empty on every
  machine) and GPIOBASE 0. The instruction encodings are held against the
  vendor assembler's own words at compile time, the four forms this chip
  added among them, and a program loaded at an offset runs with its JMPs
  relocated: the probe pushes 5, ~3, 3 reversed and the value its
  countdown ended at, and parks.
- **GPIOBASE TAKES 0 AND 16 AND NOTHING ELSE.** With the window at 0 a
  free pad at GP22 is index 22 and GP40 is outside; with it at 16 that
  same pad is index 6, GP13 falls outside and GP47 is index 31. The same
  machine driving that one pad from each window shows it in DBG_PADOE at
  **bit 6 with the window at 16 and bit 22 with it at 0** - the register
  being the BLOCK's view and not the system's.
- **THE FOUR TASKS ON THE WIRE.** The square wave asks for 1 MHz, 100 kHz
  and 10 kHz and a second machine counts **1000050 / 999950 Hz, 100000 Hz
  and 10000 Hz** at the far end. The serial port carries 64 bytes exact
  at 115200, 1 Mbaud and 3 Mbaud in **5566, 641 and 213 us** against the
  wire's own 5555, 640 and 213; a 200 us break raises the receiver's frame
  flag, delivers nothing, and the receiver takes the next byte once the
  line is idle. The PIO PWM at 250, 500 and 750 of 999 is counted at
  **250-251, 500-501 and 749-750 per mille** by a machine at the other end
  of GP17 -> GP9.
- **ALL EIGHT FLAGS REACH A LINE, which four could not on the RP2040.**
  Flag 7 forced alone raises INTS 0x8000 and gives **exactly one**
  interrupt entry - so the clear inside the ISR body is seen before the
  handler returns, and no posted write lands late here; all eight
  together read INTS 0xFF00 and the ISR leaves the flags at zero. An IRQ
  WAIT holds its machine at the instruction after the IRQ until the
  system clears the flag, and the receive FIFO's not-empty source
  delivers 32 bytes in 32 entries.
- **THE RING AND THE NEIGHBOUR MASKS.** PIO0's `irq next 3` lands on PIO1,
  PIO2's on PIO0 - the wrap from the last block to the first - and PIO1's
  `irq prev 5` on PIO0. One write to PIO0's CTRL with a neighbour mask
  starts PIO0's machine 0 and PIO1's machine 3 together (SM_ENABLE 0x1 and
  0x8), and one more stops both.
- **A SAMPLER'S RATE IS ITS OWN, ITS DUTY IS THE WIRE'S.** One instruction
  with autopush fills the joined receive FIFO with eight words - 256
  samples at clk_sys - of a 37.5 MHz square wave: **127 to 128 transitions
  and 37.35 to 37.65 MHz read back**, within one per cent. The ONES are
  192 of 256 and not 128, the words reading 0x77777777 or 0xBBBBBBBB: with
  only four samples to a period, the pad that drives, the jumper and the
  pad that reads do not carry a rise and a fall in the same time, and 6.7
  ns of difference IS a whole sample. The same sampler on a 2.34 MHz wave
  - 64 samples to a period - reads **132 ones of 256**, half of them
  within two per cent, which is what places the distortion on the wire and
  not on the program.
- **THE RECEIVE FIFO AS FOUR REGISTERS.** Under FJOIN_RX_PUT a machine
  writes cell 2 and cell 0 and the system reads back 21 and 6 at
  RXFn_PUTGETm with nothing blocking; under FJOIN_RX_GET the system writes
  0x12345678 and 0x0BADC0DE into cells 1 and 3 and the machine reads them
  out through GET.
- **THE MASKED INPUT AND THE NEW PIN FORMS.** With IN_BASE at GP13,
  SHIFTCTRL's IN_COUNT set to 1 makes `MOV X, PINS` read 0x1 with the pad
  high and 0x0 with it low, and IN_COUNT 3 reads 0x5 - the three pads from
  the base, and the two above GP13 are pads ANOTHER function is driving,
  so a block does read a pad it does not own. `mov pindirs, ~null` turns
  every OUT-mapped pin into an output in one instruction (DBG_PADOE 0 ->
  0x2000) and `mov pindirs, null` back. A WAIT on the machine's own branch
  pin holds while GP15 is low and goes on the moment the wire pulls it
  high.
- **A FIFO IS A DMA REQUEST, which is what makes a PIO program a
  peripheral.** 256 bytes at 1 Mbaud from one machine's transmit FIFO to
  another's receive FIFO, a byte engine at each end and the processor
  touching neither, arrive byte-exact in **2572 to 2573 us** against the
  wire's 2560. And the sampler with a WORD engine on its joined receive
  FIFO collects **8192 samples in 8197 us** at one a microsecond - the
  capture as long as the buffer instead of as long as four entries -
  reading **4096 ones of 8192, 500 per mille exactly**, of a 31.25 kHz
  square wave.

## Not covered yet

Driver gaps, each with its reason:

- The chapter's other examples (the duplex SPI, WS2812, Manchester,
  differential Manchester, I2C, addition): programs an application writes
  with the encoders; the four here are the ones the suite can measure
  with the wires on this desk.
- OUT EXEC and MOV EXEC (an instruction from the data stream): the
  encoders make one, the relocation of a JMP inside it is the
  application's; a program that needs it is the first user.
- The synchronizer bypass and the sticky and inline-enable outputs:
  written by the configuration, no program here depends on them.
- The security half of 11.1.1 - a non-secure block seeing only non-secure
  GPIOs, and the cross-block link severed between blocks of different
  accessibility: everything this stratum runs is Secure, as the bootrom
  hands over, so ACCESSCTRL is read by no driver of this target.

Implemented but not bench-verified, each with what would measure it:

- THE SIMULTANEITY CTRL's neighbour masks exist for. The masks are
  measured - one write starts a machine of each of two blocks - but that
  the two start on the SAME CYCLE is not: two outputs of two blocks
  compared edge by edge would want a wire pair this desk has not got, so
  the letter proves the effect and not the cycle.
- `rx_putget`, both join bits at once: the four cells become the
  machine's private scratch and the system is shut out. A program that
  wants four more registers is its first user, and what would measure it
  is a machine writing a cell the system then fails to read back.
- `pio_status_irq` and the IRQ flags as a `MOV X, STATUS` source: a
  program branching on a flag instead of blocking on it, its two paths
  told apart by what it pushes.
- Two machines in lockstep through `restart_clocks` within one block, and
  `pio_irq` relative from a machine other than 0: the pattern read back
  from two programs of one source.
- WHETHER A WAIT GPIO'S INDEX MOVES WITH THE WINDOW. 11.4.3 calls that
  index absolute and says only that the state machine's input mapping
  does not touch it; GPIOBASE's own description - PIO's GPIO 0 relocated
  - is what says the block's window does. A machine waiting on the same
  index under both windows, with two pads sixteen apart driven in turn,
  would settle it.
- The 65536 divider (`PioClockDiv{0, 0}`): a wave at 572 Hz counted.
- The tasks on machines 2 and 3 of each block, and on the QFN-60 package,
  which the stratum compiles for and no board of this bench carries.
