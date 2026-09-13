# PIO (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), chapter 3
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
`test_rp2040_pio`, wireless and on two of the standing wires.

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
by IRQ WAIT reads, in SMx_ADDR, the address of the instruction AFTER
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

The reference suite is `test_rp2040_pio`, green on the Pico and the
WeAct board:
one letter wireless, six on two of the standing wires (GP13 into
GP15, GP17 into GP9) with the PWM block's counters at the far ends.

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
  flag: the wires reach one output at a time; two square waves on
  the two wires phase-compared by a sampler would measure it.

Implemented but not bench-verified, each with what would measure it:

- `pio_wait_irq` with the relative flag and `pio_irq` relative from a
  machine other than 0: two machines of one program synchronized, the
  pattern read back.
- The STATUS source (MOV x, STATUS with its level): a program
  branching on the FIFO level, its words counted.
- The 65536 divider (`PioClockDiv{0, 0}`): a wave at 476 Hz counted.
- The tasks on PIO1 and on machines 2 and 3: the same programs, the
  same wires.
