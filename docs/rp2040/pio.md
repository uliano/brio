# PIO (RP2040)

This document is for a programmer who wants an interface the chip has
no block for - a serial port on any pin, a protocol of one's own, a
logic sampler - or one more of a block it has. The RP2040's
programmable I/O is two blocks of four small state machines, each
running a program of up to thirty-two instructions out of its block's
memory, one instruction a cycle, on the pins the program names.
`brio/rp2040/pio.hpp` gives three things: the nine instructions as
encoders evaluated when the program is compiled (there is no separate
assembler in this tree), `Pio<n>` the block and `PioSm<n, sm>` one
machine, and four programs of the chapter's own as ready-made tasks (a
task, in brio, is a driver layer with state of its own that a program
calls): a serial transmitter and receiver, a square wave, and a PWM
output that is a `PwmChannel` (`util/pwm_channel.hpp`'s interface of
an output with a duty).

The smallest use, a serial link and a square wave, from "How to use it"
below:

```cpp
using Tx = brio::PioUartTx<0, 0, 13>;         // PIO0, machine 0, GPIO 13
using Rx = brio::PioUartRx<0, 1, 15>;
Tx::init(clock, 115'200);
Rx::init(clock, 115'200);
Tx::write_wait('A');
if (const auto b = Rx::read_wait()) { got(*b); }

using Blink = brio::PioSquareWave<1, 0, 17>;
Blink::init(clock, 1'000'000);                 // a 1 MHz wave, the divider solved
```

Line by line: each task names a block, a machine of it and a pin as
template arguments, so `Tx` is a transmitter on PIO0's machine 0
driving GPIO 13 and `Rx` a receiver on machine 1 reading GPIO 15;
`init()` takes the clock's tag ([clock.md](clock.md)) and the rate,
solves the machine's clock divider from clk_sys, loads the task's
program into the block's memory, claims the pin and starts the machine
- false, nothing started, when no divider reaches the rate or the
memory has no room; `write_wait()` waits, bounded, for room in the
transmit FIFO and writes the byte; `read_wait()` waits, bounded, for a
received byte and answers nothing when none came; `Blink` is a square
wave on PIO1's machine 0 at GPIO 17.

What decides correctness even there. Each block is held in reset at
power-up and no task releases it: `Pio<0>::reset()` and
`Pio<1>::reset()` run before the first task's `init()` - the suite
resets both blocks at the start of every letter - and since a reset
also forgets every program loaded, never under running tasks. The
divider comes from the rate the clock type claims for clk_sys, so the
clock's `init()` comes first. Two tasks on one block share its
thirty-two instructions and must not share a machine, which nothing
checks. A plain `write()` into a full FIFO sets the block's TXOVER flag
(a bench finding below), so a caller that does not wait asks
`writable()` first. And a FIFO or flag interrupt enabled with no handler
bound parks the core in the crt's `Default_Handler` spin.

## Documents of record

The RP2040 datasheet (build 3184e62), chapter 3
(the programmable I/O: 3.2 the programmer's model - the programs, the
control flow, the registers and shift counters, the FIFOs, the
stalls, the pin mapping, the flags -, 3.4 the instruction set and its
encoding, 3.5 the functional details - the side-set, the wrap, the
FIFO join, autopush and autopull, the clock dividers, the GPIO
mapping and its priority, the forced instructions -, 3.6 the
examples, 3.7 the registers), 2.19.2 (the pin functions);
util/pwm_channel.hpp for the PwmChannel the PIO PWM satisfies. The
driver: `brio/rp2040/pio.hpp` (the assembler, `Pio<n>` the block,
`PioSm<n, sm>` the machine, the four tasks) over `pin.hpp`,
`resets.hpp` and `dma_engine.hpp`. The reference suite:
`test_rp2040_pio`, wireless and on two wires between the chip's own
pads - wires the bench pair (the WeAct RP2040 the suite runs on,
wired to a WeAct RP2350B) does not carry
([../boards/weact-rp2040.md](../boards/weact-rp2040.md)), so the
six wire letters run by name only, outside z, when they are fitted. A
suite's letter is one command key of its console and the group of
verdicts it runs; `z`, the all-key, runs the set.

## What the silicon does

Two blocks, four state machines each, running sixteen-bit programs
from a shared memory of thirty-two instructions; nine instructions -
JMP, WAIT, IN, OUT, PUSH, PULL, MOV, IRQ, SET -, every one taking one
cycle of the machine's clock and carrying, in its bits 12:8, a delay
of up to 31 cycles and a side-set of up to five pins, the split
between the two the machine's configuration. A machine has two 32-bit
scratch registers, an output and an input shift register with a
counter each (autopull refills the OSR and autopush empties the ISR
at a threshold), a four-entry FIFO each way that can lend its storage
to the other for eight, a 16.8 clock divider off clk_sys (1 to
65536), four pin ranges - OUT, SET, IN, side-set - each a base and a
count modulo 32 over the thirty GPIOs, a pin to branch on, and a
sticky option that keeps the last output asserted. Eight flags are
shared by a block's machines, raised, cleared and waited on by IRQ
and WAIT; the lower four reach either of the block's two interrupt
lines alongside the eight FIFO sources. Each FIFO is a DMA request.
On one cycle the highest-numbered machine writing a pin wins, and a
machine's side-set beats its own OUT or SET. An instruction written
to SMx_INSTR executes at once, the program resuming after: how a
program is started (a JMP to its offset) and how pins are claimed
before it runs (a SET). A JMP holds an absolute address, so a program
loaded at an offset has its JMPs relocated.

Three facts measured that the chapter does not state: a machine held
by IRQ WAIT reads, in SMx_ADDR, the address of the instruction after
the IRQ; the TX FIFO is emptied by executing a non-blocking PULL per
word when autopull is off (an OUT NULL only serves under autopull);
and a byte written by the DMA into the TX FIFO lands as the word's low
byte (the narrow write replicated), which is what the transmitter's
program wants.

## Types and verbs

- THE ASSEMBLER: `PioInstr`, `pio_jmp(cond, address)`, `pio_wait(
  polarity, on, index)` and `pio_wait_irq`, `pio_in(from, bits)`,
  `pio_out(to, bits)`, `pio_push(if_full, block)`, `pio_pull(if_empty,
  block)`, `pio_mov(to, from, op)`, `pio_irq(flag, clear, wait,
  relative)`, `pio_set(to, data)`, `pio_nop()`, with `PioJmp`,
  `PioWaitOn`, `PioIn`, `PioOut`, `PioMovTo` / `PioMovOp` /
  `PioMovFrom`, `PioSetTo`; `PioSideSet` (count, optional, pindirs)
  and `pio_side_delay(instr, side, value, delay)` / `pio_delay`;
  `pio_relocate(instr, offset)`; `PioProgram<N>` (the code, the wrap
  points, the side-set shape; `valid()`); `PioClockDiv` (16.8, 0 for
  65536) with `pio_clock_div_for(sys_hz, hz)` and `pio_sm_hz`.
- `PioSmConfig` (the divider, the wrap points, the side-set, the JMP
  pin, sticky and inline-enable outputs, the STATUS source, the FIFO
  join, both thresholds and directions, autopull and autopush, the
  four pin ranges) with `pio_sm_config_valid` and the three register
  words; `PioInterrupt::rx_not_empty(sm)` / `tx_not_full(sm)` /
  `flag(f)`; `PioFifoJoin`, `PioStatus`.
- `Pio<n>`: `reset` / `hold`, `pin_function`, `irq(line)`, `dreq_tx(sm)`
  / `dreq_rx(sm)`; the memory: `load(program, offset)`, `place`,
  `add` (first fit), `unload`, `used_slots`; `enable(mask)` /
  `disable` / `enabled`, `restart(mask)`, `restart_clocks(mask)` (the
  dividers in lockstep); `fifo_status`, `fifo_debug` /
  `clear_fifo_debug`, `fifo_levels`, `memory_size` / `machine_count`
  / `fifo_depth` (DBG_CFGINFO), `sync_bypass`; the flags: `flags`,
  `flag(f)`, `clear_flags`, `raise_flags` (IRQ_FORCE, visible to the
  machines); the lines: `interrupts(line, mask, on)`,
  `interrupts_only`, `raw_pending`, `pending(line)`, `force`,
  `isr(line)` (the raised-and-enabled sources, a flag source cleared).
- `PioSm<n, sm>`: `configure(config)` (the machine disabled first),
  `init(program, offset, config)` (the wrap points from the program,
  restarted, the FIFOs drained, the PC at the offset - still
  disabled), `enable(on)`, `restart`, `clock(div)`, `exec(instr)` /
  `exec_wait` / `exec_stalled`, `address()`, `instruction()`,
  `pin_directions(base, count, output)` / `pin_levels(base, count,
  high)` (a SET on the side with PINCTRL borrowed), the FIFOs:
  `tx_full` / `tx_empty` / `rx_full` / `rx_empty`, the levels, `push`
  / `pop` and their bounded waits, `drain_rx` / `drain_tx`, the four
  FDEBUG flags and their clear, `tx_address` / `rx_address` /
  `rx_top_byte_address` (the top byte of an entry for a byte engine),
  `dreq_tx` / `dreq_rx`, the two interrupt source masks.
- THE TASKS: `PioUartTx<n, sm, pin>` (init(clock, baud): 3.6.3's
  program, eight cycles a bit, the FIFO joined; `write`, `writable`,
  `write_wait`, `idle`, `tx_address`), `PioUartRx<n, sm, pin>`
  (init(clock, baud, pull): 3.6.4's program with the frame check;
  `read` (the top byte), `readable`, `read_wait`, `frame_error()` -
  flag 4 + sm -, `rx_top_byte_address`), `PioSquareWave<n, sm, pin>`
  (init(clock, hz): a four-cycle wave), `PioPwm<n, sm, pin, period>`
  (a PwmChannel: `init(divider)`, `duty`; a pulse of (period + 1) x 3
  machine cycles). Every other program is the application's, written
  with the encoders.

## How to use it

```cpp
using Tx = brio::PioUartTx<0, 0, 13>;         // PIO0, machine 0, GPIO 13
using Rx = brio::PioUartRx<0, 1, 15>;
Tx::init(clock, 115'200);
Rx::init(clock, 115'200);
Tx::write_wait('A');
if (const auto b = Rx::read_wait()) { got(*b); }

using Blink = brio::PioSquareWave<1, 0, 17>;
Blink::init(clock, 1'000'000);                 // a 1 MHz wave, the divider solved
```

A program of one's own:

```cpp
constexpr brio::PioProgram<1> sampler = [] {
    brio::PioProgram<1> p{};
    p.code = {brio::pio_in(brio::PioIn::pins, 1)};   // one pin into the ISR, autopush at 32
    return p;
}();
using Sm = brio::PioSm<0, 2>;
const auto at = brio::Pio<0>::add(sampler);
brio::PioSmConfig c{};
c.clock = *brio::pio_clock_div_for(clock_hz(clock), 1'000'000);
c.in_base = 15;
c.in_shift_right = false;
c.autopush = true;
c.push_threshold = 32;
c.fifo_join = brio::PioFifoJoin::rx;
Sm::init(sampler, *at, c);
brio::DmaRxEngine<10, uint32_t>::arm(Sm::rx_address(), Sm::dreq_rx);
brio::DmaRxEngine<10, uint32_t>::start(words, 256);
Sm::enable(true);
```

## Bench findings

The reference suite is `test_rp2040_pio`, green on the WeAct board:
two letters wireless in z (13 verdicts), six by name on two wires (GP13
into GP15, GP17 into GP9) with the PWM block's counters at the far ends.

- THE ENCODINGS: the transmitter's four words (0x9FA0 0xF727 0x6001
  0x0642) and the receiver's nine are the vendor's assembled ones;
  DBG_CFGINFO says 32 instructions, 4 machines, FIFOs 4 deep; at reset
  no machine is enabled and every FIFO empty. A program loaded and
  run pushes SET X 5, Y 3 inverted (0xFFFFFFFC) and bit-reversed
  (0xC0000000), and a JMP X-- run to its end leaves X at all ones; SET,
  MOV and PUSH executed on the side of a stopped machine push 9; the
  transmit FIFO is full at four and a fifth push sets TXOVER, a pop of
  the empty receive FIFO sets RXUNDER, joined it takes eight; a flag
  raised through IRQ_FORCE reaches line 0 once.
- THE SQUARE WAVE at 31.25 MHz (the machine at clk_sys), 1 MHz, 100
  kHz and 10 kHz counted by edges on the wire: 31.30 MHz, 999.5 kHz,
  100.0 kHz, 9995 Hz.
- THE SERIAL PORT from machine 0 to machine 1 on one wire: 64 bytes
  at 115200, 1 Mbaud and 3 Mbaud byte-exact in the wire's own time
  (5571, 641, 214 us); a 200 us break raises the receiver's frame
  flag and delivers nothing, and the next byte after the line idles
  is taken. ON THE DMA: 256 bytes at 1 Mbaud poured into the transmit
  FIFO by a byte engine and collected from the receive FIFO's top byte
  by another, byte-exact, in 2.6 ms.
- THE PIO PWM as a PwmChannel: levels 250, 500 and 750 of 999 read
  250, 500 and 749 per mille by level on the wire.
- THE INTERRUPTS: the receive FIFO's not-empty source on line 0 takes
  32 bytes in 32 entries; a program's IRQ 1 reaches line 1 once; IRQ
  WAIT raises the flag and holds the machine (ADDR reading the
  instruction after) until the system clears the flag, then the
  program pushes.
- THE SAMPLER: one instruction with autopush at 1 MHz, 8192 samples of
  a 25 % wave collected by the DMA in 8.2 ms, 250 per mille ones.
- THE CORNERS, wireless (letter h, ten runs of ten):
  - IRQ RELATIVE: `irq 0 rel` raises flag 1, 2, 3 on machines 1, 2, 3
    and nothing else, and `irq 1 rel` on machine 3 raises flag 0 - the
    modulo-4 sum on the two low bits. `wait 1 irq 0 rel` on machine 2
    stays stalled under flags 0, 1 and 3 together and leaves on flag 2,
    which the wait clears.
  - MOV X, STATUS as a branch: a machine that pulls only while STATUS
    (TX level, N = 2) reads zero takes 1, 2 and 3 of four words pushed
    while it was stopped and leaves one; on the RX level (N = 2) STATUS
    reads all ones at levels 0 and 1 and zero at 2.
  - THE 65536 DIVIDER: CLKDIV 0 (`PioClockDiv{0, 0}`) on machine 3
    divides by 65536 - the four-cycle square wave rises 477 times in a
    second on the pad's own input (476.84 Hz; the machine and the timer
    share the crystal).
  - PIO1, MACHINES 2 AND 3: the serial tasks on ONE pad, the receiver
    set up first and the transmitter after, the receiver reading the
    transmitter's output through the pad's input: 64 bytes exact at
    115200 and at 1 Mbaud, no frame error.

## Not covered yet

Driver gaps, each with its reason:

- The chapter's other examples (the duplex SPI, WS2812, Manchester,
  differential Manchester, I2C, addition): programs an application
  writes with the encoders; the four here are the ones the suite can
  measure with the wires and counters on the desk.
- OUT EXEC and MOV EXEC (an instruction from the data stream): the
  encoders make one, the relocation of a JMP inside it is the
  application's; a program that needs it is the first user.
- The synchronizer bypass and the sticky and inline-enable outputs:
  written by the configuration, no program here depends on them.
- Two machines in lockstep through `restart_clocks` and a shared
  flag: no program here depends on it; two square waves on two free
  pads, phase-compared through the pads' own inputs by a sampler
  machine (no wire needed - letter h reads a pad that way), would
  measure it.
