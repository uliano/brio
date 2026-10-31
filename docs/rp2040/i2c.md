# I2C (RP2040)

This document is for a programmer who has an I2C device on an RP2040 -
a sensor, a converter, an EEPROM - and wants to talk to it through the
bus arbiter ([design/i2c-bus.md](../design/i2c-bus.md) is the model),
or who wants this chip to answer as a client on someone else's bus.
The chip's two I2C blocks are Synopsys's DW_apb_i2c, so the driver is
written once in the IP stratum (`brio/dw_apb_i2c/`) and this stratum
supplies the chip's part and the public names: `DwApbI2c<n>` the
instance, `I2cHost<n, pins, ...>` the host engine `util/i2c_bus.hpp`
drives, `I2cClient<n, pins>` the client role. The arbiter, `I2cBus`,
is an active object (a component of a brio program with its own event
queue, served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)): devices post
it requests, it runs them one tenure at a time on the host - a tenure
being one START to its STOP - and replies to each.

The smallest use, a register read through the arbiter, from "How to
use it" below:

```cpp
constexpr brio::I2cPins pins{.scl = 13, .sda = 12};
using Bus = brio::I2cHost<0, pins>;
using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;

Bus::init(clock);

Bus::Request r{};
r.addr = 0x48;
r.tx = brio::lend<brio::Lease::reply>(reg);   r.tx_len = 1;    // the register
r.rx = brio::lend<brio::Lease::reply>(value); r.rx_len = 2;    // then its value
r.speed = brio::I2cSpeed::fast_400k;
r.reply = brio::reply_to<Sensor, brio::I2cDone>();
brio::post<I2c>(r);

extern "C" void isr_i2c0() {
    if (Bus::isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
```

Line by line: `pins` names I2C0's SCL and SDA pads, GP13 and GP12, which
table 279 gives the instance (a pin it does not give does not
compile); `Bus` is the host engine over I2C0; `I2c` the arbiter over it
- four requests able to wait while one is in flight, `BusPassThrough`
the failure policy (a failed request is answered to its client as it
failed, never retried), and a 20 ms timeout after which
a tenure that never completed is answered `i2c_timeout` and the host
recovered - listed in the program's `Tenuto` like any active object;
`init()` takes the clock task's tag ([clock.md](clock.md)) and solves
the three speeds' SCL counts from clk_sys; the request writes one byte
(the register's address) to the device at 0x48 and reads two back in
the same tenure, at 400 kHz, and names `Sensor` as the active object
that receives the `I2cDone` reply; `isr_i2c0` is I2C0's handler under
the pico-sdk's name, and it tells the arbiter when the host's tenure
completes.

What decides correctness even there. The request lends its buffers
(`brio::lend<Lease::reply>`): `reg` and `value` belong to the tenure
from the post until `Sensor` receives its `I2cDone`, and stay
untouched and in scope until then, which nothing checks. `init()` after
the clock task's: the counts are computed from the rate the clock type
claims for clk_sys, and `init()` answers false when that rate cannot
reach the standard speed (under 2.7 MHz) or the block does not come up;
a request for a speed the clock cannot reach is answered `i2c_rejected`
and moves nothing on the wire, and so is one whose address lies outside
0x08..0x77 - the general call and the 10-bit prefixes among them, which
every host refuses ([../design/i2c-bus.md](../design/i2c-bus.md)). The vector is the program's to bind: the
crt gives `isr_i2c0` a weak alias to `Default_Handler`'s spin, so an
unbound vector parks the core there at the host's first interrupt. The
pull-ups are the board's: the pads take the internal pull-up the
chapter asks for, but the bus needs its external resistors. And a bus
without a timeout (`I2cBus`'s last argument 0) waits for ever on a
tenure whose completion never comes, a client holding SCL among the
causes ("Bench findings", the timed bus).

The page is long because the chapter is. In order: the documents of
record; what the silicon does - the host as a command FIFO, the SCL
timing, the client, two hosts, the pins; the types and verbs; how to
use it, with the engines and a client; the bench findings, against the
partner's peer on the pair's wires, with the host's cost and the
vendor's tenure on the same wire; and what is not covered.

## Documents of record

The RP2040 datasheet (build 3184e62), 4.3 (the
Synopsys DW_apb_i2c: 4.3.5 the behaviour - START and STOP generation,
the combined formats -, 4.3.7 the transmit FIFO and the RESTART / STOP
bits of a command, 4.3.10 the operation modes - the slave flows on
RD_REQ and RX_FULL, the master's command stream, the disable
procedure, the abort -, 4.3.11 the spike filter, 4.3.12 fast-mode-plus,
4.3.14 the SCL counts with their floors and the cycles the block adds,
4.3.15 the DMA interface, 4.3.16 the interrupt table, 4.3.17 the
registers), 2.19.2 (table 279: the pins), 2.1.4 (narrow writes
replicated across a register); docs/design/i2c-bus.md for the Request
and the vocabulary. The driver is Synopsys's and not this chip's, so it
lives in the IP stratum: `brio/dw_apb_i2c/i2c.hpp` holds the resource,
the host engine and the client
([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)), and
`brio/rp2040/i2c.hpp` holds what this chip owes it - the pin table of
table 279 with its pad configuration, the `I2cPad` type that adds the
two open-drain verbs to a `Pin`, the `Rp2040DwApbI2c` chip traits over
`pin.hpp`, `resets.hpp`, `nvic.hpp`, `delay.hpp` and `dma_engine.hpp`,
this chip's device description held against the bits the IP file spells
itself, and the public names `DwApbI2c<n>` (the resource), `I2cHost`
(the engine `I2cBus` drives) and `I2cClient` (the other end), this
chip's aliases of the three templates there. The reference suite:
`test_rp2040_i2c`, the block wireless and then the wire, which is the
desk partner's: I2C1 on GP2 (SDA) / GP3 (SCL) with 1.5 kOhm to this
board's 3V3 on each, to the WeAct RP2350B
([../boards/weact-rp2040.md](../boards/weact-rp2040.md)), whose peer of
the twi_link protocol is commanded over the bus under test; the same
board's own suite, `test_rp2350_i2c`, judges this chip's block as that
peer. A suite's letter is one command key of its console and the group
of verdicts it runs.

## What the silicon does

Two DW_apb_i2c controllers on clk_sys, each a host or a client and
never both, standard, fast and fast-mode-plus (no high-speed mode),
7-bit addresses on both sides and 10-bit ones, two FIFOs of sixteen
entries, one interrupt line per instance with thirteen sources, a DMA
request per FIFO. The component parameter register that would report
the FIFO depths is not implemented and reads zero: the depth is the
chapter's word, and measured.

The host is a command FIFO. Every IC_DATA_CMD entry is a byte to write
or a read command, with a RESTART bit and a STOP bit: the first entry
after idle issues START and the address in IC_TAR, a direction change
(or the RESTART bit) a repeated START, the STOP bit a STOP after that
entry. A transmit FIFO that runs empty with no STOP holds SCL low and
stalls the bus until the next entry (4.3.7). There is no address-only
entry: START, the address and STOP with no data cannot be issued, so
the probe is a one-byte read. Every failure - a NACK on the address
or on a byte, a lost arbitration, an abort asked for - is one
interrupt, TX_ABRT, with the reason in IC_TX_ABRT_SOURCE, and both
FIFOs stay flushed until IC_CLR_TX_ABRT is read. IC_CON, the counts,
the addresses and the hold times take a write only with ENABLE clear;
a disable completes when the bus activity does (IC_ENABLE_STATUS says
when), and a host mid-command with no STOP never disables - the abort
is the way out. TX_EMPTY and RX_FULL are levels against IC_TX_TL and
IC_RX_TL, set and cleared by the hardware; the other eleven sources
clear by reading their own register.

The SCL timing is two counts per speed in ic_clk cycles (HCNT, LCNT;
the standard pair and the fast pair, fast-mode-plus using the fast
pair), the spike filter (SPKLEN), and the SDA hold and setup. The
block adds to what is programmed: the low period is LCNT + 1, the
high period HCNT + SPKLEN + 7 (4.3.14.1), and the floors are HCNT at
least SPKLEN + 5 and LCNT at least SPKLEN + 7 - table 450's own rows
sit exactly there. The specification's minimum high and low times
(4000 / 4700 ns, 600 / 1300, 260 / 500) bound the split, and a clock
too slow to meet them makes the speed unreachable: 2.7 MHz for
standard, 12 for fast, 32 for fast-mode-plus.

The client answers IC_SAR and then: a written byte lands in its
receive FIFO (RX_FULL), a read raises RD_REQ and holds SCL until a byte
is given - the clock stretch is the block's own -, the host's NACK at
the end of a read is RX_DONE, a repeated START while addressed is
RESTART_DET, the STOP is STOP_DET (for its own tenures alone under
STOP_DET_IFADDRESSED). Bytes queued beyond what the host takes are
flushed at its NACK and reported as an abort (ABRT_SLVFLUSH_TXFIFO). A
full receive FIFO holds SCL instead of overflowing
(RX_FIFO_FULL_HLD_CTRL): the client holds SCL as a transmitter, at an
empty transmit FIFO, and as a receiver only with its receive FIFO full.
IC_SLV_DATA_NACK_ONLY makes the client refuse every data byte, written
with the block disabled and the client idle; a refusal mid-tenure is a
disable, which "stops the current transfer at the end of the current
byte and does not acknowledge" (4.3.17, IC_ENABLE) and leaves
SLV_RX_DATA_LOST set. A general call raises GEN_CALL and no STOP_DET
under STOP_DET_IFADDRESSED (the note on IC_CON bit 7). There is no
address-match event.

Two hosts (4.3.8): a host's START waits while the bus is busy - a START
on the wire not yet closed by its STOP - so two hosts whose STARTs wait
on one STOP leave together and the wired-AND decides, the loser aborting
with ARB_LOST. STOP_DET is raised in host mode for every STOP on the
wire, another host's included.

The pins are fixed per instance under function 3 (table 279): I2C0
has SDA on GPIO 0, 4, 8, 12, 16, 20, 24, 28 and SCL on 1, 5, 9, 13,
17, 21, 25, 29; I2C1 has SDA on 2, 6, 10, 14, 18, 22, 26 and SCL on
3, 7, 11, 15, 19, 23, 27. The pads take the pull-up, the slew limit
and the Schmitt trigger 4.3.1.3 asks for; the board's pull-ups do the
pulling. A narrow write to IC_DATA_CMD is replicated across the word
(2.1.4): a command is written as a halfword, never as a byte.

The bus clear of 4.3.13 (nine clocks and a STOP by the master) has no
enable in this chip's IC_CON: the unstick is by hand.

## Types and verbs

- `I2cSpeed` (standard_100k, fast_400k, fast_plus_1m) with
  `i2c_speed_hz`, `i2c_speed_code` (IC_CON.SPEED), `i2c_min_clk_hz`
  (table 450), the specification's minima per speed, `I2cTiming`
  (hcnt, lcnt, spklen, sda_tx_hold, sda_setup, speed),
  `i2c_timing_for(hz, speed)` (the split under the floors with the
  block's added cycles subtracted; nullopt when unreachable),
  `i2c_scl_hz(hz, timing)` (what the counts produce on an ideal wire),
  `I2cPins` (scl, sda) with `i2c_sda_pin` / `i2c_scl_pin` as table
  279 and `i2c_pins_valid`, `i2c_pad_config`, `I2cRole`, `I2cConfig`
  (role, speed, 10-bit host / client, restarts, STOP_DET only if
  addressed, the late TX_EMPTY, the hold when the receive FIFO is
  full) with `i2c_con_of`, `I2cCmd` (restart, stop) with
  `i2c_write_entry` / `i2c_read_entry`, `I2cFlag`, `I2cInterrupt`,
  `I2cAbort` (the abort sources grouped) with `i2c_status_of_abort`
  (the vocabulary's code for a source). Of that list only `I2cPins`,
  `i2c_sda_pin` / `i2c_scl_pin`, `i2c_pins_valid` and `i2c_pad_config`
  are this chip's; every other name is the IP stratum's, spelled the
  same on every family that carries the block.
- `DwApbI2c<n>`: `reset` / `hold` / `released`, `enable`, `disable`
  (bounded on IC_ENABLE_STATUS), `running`, `abort`, the two facts a
  disable reports about a cut client tenure, `configure`, `timing`
  (both the write of a speed's counts and the readback), `sda_rx_hold`,
  `target` (with the general call), `own_address`, `ack_general_call`,
  `nack_data`, the two thresholds, `command_block`, the flags and
  the FIFO counts, `push` / `pop` / `first_data_byte` / `flush_rx`,
  the interrupt mask, the raw and masked status, `clear_pending` (the
  read-to-clear sources by mask), `clear_all`, `abort_source` /
  `flushed_entries` / `clear_abort`, `isr()` (the raised-and-enabled
  sources, nothing cleared), `dma_requests`, `dma_levels`,
  `dreq_tx` / `dreq_rx`, `irq()`.
- `I2cHost<n, pins, TxEngine, RxEngine, Placement>`: the engine `I2cBus`
  (= `BusMaster`) drives. Its `Request` is the other strata's: `addr`,
  `tx` / `tx_len`, `rx` / `rx_len`, `reply`, `speed`. `init(clock)`
  (clk_sys, the three speeds solved), `rebase(hz)`, `speed_ok`,
  `timing_of`, `scl_hz`, `reference_hz`, `idle` (the engine's phase),
  `start` (false when the wire moves; true for the refusals that
  move nothing, i2c_rejected for an unreachable speed or an address
  outside 0x08..0x77 and i2c_bus_error for a block that would not
  disable to take the address), `isr()`, `dma_isr()`, `status()`, `unstick()` (the pulses
  it took, 0 for a healthy wire, 0xFF for one that stays low),
  `recover()` (the block reset and reconfigured), `release()`, and the
  two levels the pump runs at, `tx_level` and `rx_level` (half a FIFO
  each). The pump writes a run into the room the transmit FIFO has and
  refills on TX_EMPTY at its half, takes eight bytes an interrupt on
  RX_FULL and the tail at the STOP, and ends EVERY tenure on its
  STOP_DET (or on the abort); the address and the speed go into the
  block only when they move. The engine slots take a
  `DmaTxEngine<ch, uint16_t>` and a `DmaRxEngine<ch>` on any two
  channels, both or neither, and serve a READ phase of three bytes or
  more: the plain read command poured from a fixed cell, the bytes
  collected by a receive engine armed to report a bus error alone, the
  first entry (RESTART) and the last (STOP) through the pump. A write
  phase stays on the pump.
- `Placement`, the last parameter: util/placement.hpp's tag, `InFlash`
  by default, `InRam` making `start()`, the vector's body and the DMA
  line's one flattened instance each in SRAM
  ([platform.md](platform.md), "A handler in SRAM").
- `I2cClient<n, pins>`: `init(clock, Config)` (address, 10-bit, the
  general call, the fastest speed it will see, the hold at a full receive
  FIFO - on by default), `events` (the sources
  `service()` reports on), `data_ready` / `take`, `data_wanted` /
  `give` / `writable` / `clear_read_request`, `stop_seen` /
  `clear_stop`, `host_nacked` / `clear_nack`, `overrun`,
  `general_call_seen`, `acknowledge(on)` (a disable/enable pair),
  `flush_tx`, `service()` (one `I2cClientEvent` per call:
  byte_received, byte_wanted, stop, restart, nacked, general_call -
  once a call, GEN_CALL masked from its report to the call's first data
  byte -, flushed, overrun, error), `host_reads`, `release`.

## How to use it

```cpp
constexpr brio::I2cPins pins{.scl = 13, .sda = 12};
using Bus = brio::I2cHost<0, pins>;
using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;

Bus::init(clock);

Bus::Request r{};
r.addr = 0x48;
r.tx = brio::lend<brio::Lease::reply>(reg);   r.tx_len = 1;    // the register
r.rx = brio::lend<brio::Lease::reply>(value); r.rx_len = 2;    // then its value
r.speed = brio::I2cSpeed::fast_400k;
r.reply = brio::reply_to<Sensor, brio::I2cDone>();
brio::post<I2c>(r);

extern "C" void isr_i2c0() {
    if (Bus::isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
```

With the engines: `I2cHost<0, pins, DmaTxEngine<4, uint16_t>,
DmaRxEngine<5>>` and `isr_dma_0` calling `Bus::dma_isr()` the same
way. A client:

```cpp
constexpr brio::I2cPins client_pins{.scl = 15, .sda = 14};
using Client = brio::I2cClient<1, client_pins>;
Client::init(clock, {.address = 0x42, .speed = brio::I2cSpeed::fast_400k});
Client::interrupts(Client::events, true);
extern "C" void isr_i2c1() {
    switch (Client::service()) {
        case brio::I2cClientEvent::byte_received: while (Client::data_ready()) { heard(Client::take()); } break;
        case brio::I2cClientEvent::byte_wanted:   Client::give(next_answer()); Client::clear_read_request(); break;
        case brio::I2cClientEvent::stop:          tenure_over(); break;
        default: break;
    }
}
```

## Bench findings

The reference suite is `test_rp2040_i2c`, green on the WeAct board
against the WeAct RP2350B's peer over the pair's I2C wires, on either
of that chip's halves: **64 verdicts** in its key `z`. The same block as the PEER, the RP2040 the
far end, is judged by `test_rp2350_i2c` (docs/rp2350/i2c.md), so every
client finding below is measured from both sides. The cost figures are
`bench_rp2040` letter i's, against the same peer.

- The block's reset state is a host with its client half disabled at
  the fast speed code with restarts enabled (IC_CON 0x65), the spike
  filter at 7, the hold at 1, and the component parameter register
  at zero. The transmit FIFO measured under TX_CMD_BLOCK is sixteen
  deep: full at sixteen, TX_OVER on the seventeenth, flushed by the
  disable. Configuration, timing, target and own address are refused
  while enabled; the disable of an idle block is reported within a
  few microseconds.
- THE WIRE. The scan of 0x08..0x77 finds the peer's 0x6B alone; nobody
  home is `i2c_nack_addr` in 110 to 111 us at 100 kHz - the address's 94 us of
  wire and the disable cycle a new address pays. The three speeds run
  byte-exact both ways against the second chip and measure, on the
  system timer over a 64-byte tenure, 99, 393 and 963 to 966 kHz for
  100, 400 and 1000 asked (the counts give 100, 399.4 and 1000 on an
  ideal wire); the bench's SCL period at 400 kHz is 315.9 cycles against
  the counts' 313, the 1.5 kOhm pull-ups' rise adding 23 ns a period at
  every speed.
- THE TENURE SHAPES against a second chip: a write, a read, a
  write-then-read whose repeated START the peer counts as a tenure of its
  own (RESTART_DET), the probe, reads of 1, 2, 17 and 40 bytes across the
  peer's refills, and a 200-byte write in ONE tenure through the pump in
  **24** interrupts - ceil(184 / 8) refills at half the FIFO and the
  STOP. The peer's tally (count, sum, first byte, tenures) agrees with
  every one.
- THE RESERVED ADDRESSES, refused both ways. A Request to 0x7A or to the
  general call is answered true and `i2c_rejected` by `start()`, no
  entry written, and the peer's client, answering the general call,
  takes only the write to its own address that follows - one tenure,
  two bytes (letter `e`). Commanded to write a general call at this
  board's client, the peer's host refuses it the same way and this
  client, IC_ACK_GENERAL_CALL on, hears nothing (letter `l`).
- THE VOCABULARY: a deaf client is `i2c_nack_addr` on a write, the probe
  and a read; a client refusing from its first data byte
  (IC_SLV_DATA_NACK_ONLY) and from its third (a disable once the second
  is taken, which refuses the byte in flight and sets SLV_RX_DATA_LOST)
  is `i2c_nack_data`, with nothing and two bytes stored. THE CLIENT'S
  STRETCH IS A READ'S: 100 us held before each of eight bytes given
  lengthens this host's read by 746 to 752 us - eight holds less the
  host's own 6 us low phase each overlaps - and leaves a write under the
  same client within 7 us of the plain one (the block holds SCL as a
  receiver only with its receive FIFO full).
- THE ENGINES against the second chip: a 64-byte read at 400 kHz in
  1487 to 1494 us (391 to 393 kHz) with two host interrupts - the second
  engined read in a row; the first is the open item below -, a write-then-read,
  the short reads on the pump, a deaf client on an engined read, and
  eight 255-byte reads at 1 MHz byte-exact with TX_OVER never raised.
- THE ENGINES' FAULT PATH (letter g): a 64-byte engined read whose
  receive run points into a hole in the map (0x40100000) is a bus error
  on the receive channel; `dma_isr()` aborts the tenure and answers
  `i2c_dma_fault` with no host interrupt, the peer having served 3 or 4
  bytes by the abort (21 and 9 of 30 runs over the three images of the
  pair), and the next engined read is clean.
- I2cBus OVER I2cHost: four queued tenures, the NACK in its place, the
  rejection of what a four-deep queue cannot hold, both votes, and THE
  TIMED BUS - the peer holding SCL 30 ms before the first byte of a read
  - answered `i2c_timeout` at the arbiter's 20 ms with SCL still held,
  the recover()ed engine carrying the next two tenures.
- THE STUCK BUS: the peer holding SDA until its fourth SCL fall is freed
  by unstick() in exactly four pulses; a line nine pulses cannot free is
  0xFF in 115 to 118 us (twenty waits of 5 us on SysTick's delay, and the pads
  handed over), the bus working after the far end lets go.
- TWO HOSTS (4.3.8), measured both ways: both STARTs written while a
  START made by hand holds the bus busy, released by the one STOP that
  closes it - this host aiming at 0x6B loses the wired-AND to the peer's
  0x2C at its first address bit (`i2c_arb_lost`), and aiming at 0x11 it
  keeps the bus and the PEER reports the loss. THE HOST'S STOP_DET IS
  EVERY STOP ON THE WIRE: a tenure written inside the window ended
  `i2c_ok` on the window's own STOP before it had clocked a bit, until
  the engine required its transmit FIFO empty and its host state machine
  idle at the STOP (its own STOP follows its last entry; the IP stratum's
  page, and below, the held wire under the preemptive kernel). A new address written
  while another node holds the bus costs a disable of 94 us at 100 kHz.
- THIS BLOCK AS THE PEER HOST'S CLIENT: six bytes written to it exact
  with their STOP; two reads of six served one byte a read request, the
  peer's count, sum and both first bytes the run given, nothing flushed.
- THE CLIENT'S EDGES (letter m, the peer's host at 100 kHz): without the
  hold, 24 bytes written into a receive FIFO nobody drains are all
  acknowledged, the first 16 kept exact and EIGHT `overrun` events
  raised, one a byte lost; with the hold the same write stretches until
  the vector drains it, all 24 exact. A disable after the fourth byte of
  a write refuses the fifth - the far host's data NACK - with
  SLV_DISABLED_WHILE_BUSY and SLV_RX_DATA_LOST both set. A disable at a
  read request the client holds SCL for NEVER COMPLETES: IC_EN stays 1
  through 2 ms and 50 ms more, a byte written after it untaken, neither
  bit set; `release()` lets SCL go and the far host's read finishes on the
  pull-up ([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)). Ten runs
  of z, the same figures in each.
- THE FIRST ENGINED READ IS THE XIP CACHE'S FILL. Letter g's first
  64-byte read at 400 kHz after a boot takes 1632 us with ONE host
  interrupt where the read right after it takes 1492 to 1500 us with two,
  and the same read with every line of the XIP cache flushed first (the
  letter's third, printed) takes 1637 to 1646 us with one - the first
  read after a boot is the cold one, in each of five boots, and the
  letter run again in the same boot reads its first read 1507 to 1513,
  warm. The 140 us are the host's `start()`, its vector's
  and the DMA line's code fetched line by line out of the flash, and the
  late handler finding room for the STOP entry is why one entry stands
  for two. The placement is what takes it away: `bench_rp2040` letter i's
  16-byte engined read at 400 kHz reads 49498 cycles warm and 69460 cold
  with the default image, 54576 cold with the hosts InRam and the vectors
  placed ([platform.md](platform.md), "A handler in SRAM"). The letter
  judges the second read.
- TX_EMPTY IS MASKED BEFORE THE PUSH ([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md),
  the transmit threshold): masked after it, the engined read's second
  read took a third host entry reading IC_INTR_STAT zero after every
  boot - the letter's verdict failed seven passes in seven - and reading
  the mask back after the store changed nothing; masked first, ten passes
  in ten read two entries.
- THE HOST'S COST, `bench_rp2040` letter i against the peer's client,
  the host's vector in the flash (the default image, warm), nothing of
  the far end on this core. Each
  figure the best of five; the fixed cost is wall less the wire's time,
  in clk_sys cycles at 125 MHz:

  | at 400 kHz | wall | interrupts | handler cycles | busy | fixed cost |
  |---|---|---|---|---|---|
  | write of 1 | 6830 (x 1.11) | 1 | 157 | 709 | 700 |
  | write of 255 | 729 200 (x 1.00) | 31 | 8960 | 14 924 | 696 |
  | read of 16 | 49 637 (x 1.01) | 2 | 583 | 1604 | 847 |
  | read of 255 | 729 338 (x 1.00) | 32 | 13 087 | 19 300 | 834 |
  | register read, 1 + 1 | 13 023 (x 1.06) | 1 | 211 | 816 | 763 |
  | read of 255 on the engines | 729 216 (x 1.00) | 3 | 571 | 2406 | 712 |

  A tenure of half a FIFO or less is ONE interrupt, its STOP; a long
  read is one every eight bytes. The fixed cost is the same at the three
  speeds (700, 700, 700 for a one-byte write) because the address and
  the speed stay in the block between tenures: the cycle of disable,
  IC_TAR and enable, which a new address still pays (`i2c.write.readdr`),
  costs 968 cycles more at 100 kHz, 186 at 400 kHz and 73 at 1 MHz - the
  disable waits on the block's own shutdown, which follows the bus
  timing. The longest host entry is 429 cycles, a 255-byte read's. Cold,
  the same one-byte write's `start()` reads 12429 cycles and its entry
  6182 with both in the flash ([platform.md](platform.md), "A handler in
  SRAM").
- THE VENDOR'S TENURE ON THE SAME WIRE: the pico-sdk 2.3.1's polled
  `i2c_write_blocking` / `i2c_read_blocking` (its `i2c.c` compiled from
  the pinned archive, `clock_get_hz` answered with clk_sys, brio's crt,
  clock, console and ruler around it) on I2C1, GP2/GP3, against the same
  peer between the same serve commands; the SDK's own `i2c_init`, then
  brio's counts written over its own, so the SCL period measures the
  same (316.0 cycles at 400 kHz against letter i's 315.9). At 400 kHz,
  fixed cost, the SDK against brio's default image: a one-byte write 398
  against 700, a 255-byte write 388 against 696, a 16-byte read 1053
  against 847, a 255-byte read 16 442 (x 1.02) against 834 (x 1.00), a
  register read of 1 + 1 66 against 763; a refused address 383 against
  822. THE WRITE'S GAP, some 300 cycles (4.6 % of a one-byte write's
  wall), is what the SDK does not do: it takes no interrupt and no wake
  from the idle loop - and holds the core for the whole wire, busy 6528
  cycles against 709. Its short reads' fixed cost is near zero because
  `i2c_read_blocking` returns at the last byte, before the NACK and the
  STOP are on the wire; its long read keeps one read command in flight,
  the bus held a loop turn every byte, where the pump pours them by the
  FIFO.

### The stack under the preemptive kernel (`test_rp2040_rubato_i2c`)

Measured by `test_rp2040_rubato_i2c`: `util/i2c_bus.hpp`'s arbiter
(pending depth 4, the per-bus timeout at 20 ms) over the engined
`I2cHost<1>` on GP2/GP3 (DMA channels 4 and 5 on DMA_IRQ_0, clk_sys at
125 MHz) inside Rubato on core 0, against the WeAct RP2350B's
`twi_peer`, while PWM slice 7's wrap posts to the pack's highest object
every 97, 193 or 389 us and its dispatch spins 300 or 3000 cycles - at
97 us x 3000, 25 % of the core. Every letter runs once with the slice
silent and six times under it, and letter `c` runs letter `o` again with
every pressure fed by core 1's doorbell ([multicore.md](multicore.md)).
**237 verdicts in the key `z`, green in every run - the key three
times, its vocabulary letter `q` ten more; the letters `x`, `e` and `s`, outside it, green
in ten runs each.** The pack, the command
channel through the arbiter and the letters are the I2C suites' shared
body (`avrxt/src/apps/rubato_i2c.hpp`, whose header describes them);
what the two Raspberry Pi chips share over this block - the paths, the
peer's arithmetic, the stretch instrument, letter `s` - is
`rp2040/src/apps/rp_rubato_i2c.hpp`, whose header derives every bound.

**Where a preemption reaches this host: inside `start()`'s fill, and a
stretch only past two byte times.** `start()`'s first entry starts the
tenure, and the block clocks the address and that byte while the fill
goes on: a preemption after the k-th entry leaves the block (1 + k) byte
times before it needs the next, and only a longer one empties the
command FIFO with no STOP in it - the block then holds SCL until
`start()` resumes, the bytes unchanged
([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md), the host's engine).
Letter `s` measures the fill: `start()` takes 497 to 581 cycles for a
one-byte write and 887 to 968 for a write of sixteen, so the fifteen
entries after the first are 309 to 425 cycles (20 to 28 an entry), where
400 writes under ticks independent of them would land 10 to 14 times.
Under 97 us x 3000, the longest preemption of `start()` 3485 cycles: at
400 kHz, where two byte times are 5625 cycles, the slice landed in
`start()` 26 to 50 times in 400 writes, 7 to 17 of them with the host
active on the wire, and **stretched no tenure in ten runs**; at 1 MHz
(two byte times 2250 cycles), 23 to 41 landings, 8 to 21 with the host
active, 9 stretches in 4000 writes; under a 160 us spin at 389 us,
longer than a whole sixteen-byte write at 1 MHz, 47 stretches of 62
landings with the host active - by the arithmetic, the other fifteen
landed after the last entry, whose tenure ends on its own STOP inside
the spin. Every
write `i2c_ok` and the peer's tallies exact, stretched or not. A stretch
is read on the pad: the host active, its FIFO empty and SCL low for
10 us, longer than any low half this host clocks (6 us at 100 kHz) - an
empty FIFO alone is the block still clocking the entry it took. The key
`z`, whose letter `p` runs 1 MHz, stretched 11 tenures in ten runs.

**THE TENURE CYCLE LOCKS TO THE DOORBELL.** Letter `x`, 400 writes of
sixteen at 400 kHz (`start()` 933 to 961 cycles on average, silent):
under the slice at 97 us x 3000 the landings in `start()` are 35 to 50
against the 30 or 31 independent ticks would give; letter `e`, the same
with every tick a doorbell from core 1, reads 180 to 200 - six times
that, the tenure cycle settled at a phase of the doorbell's period - and
`start()` averages 4039 to 4178 cycles there against 1368 to 1616 under
the slice. No stretch in either, at 400 kHz.

**THE HELD WIRE PARKS THE START, ACROSS A NEW ADDRESS AND A NEW SPEED.**
Letter `h`: the peer holds SDA low 60 ms, and the Client's write into
it, whose `start()` first takes the disable, IC_TAR and enable cycle for
its address, is answered `i2c_timeout` 19.8 to 20.1 ms after its
`start()` in every pass of ten runs - its START parked behind the hold,
the re-enabled block still knowing the bus busy; met at 400 kHz after
commands at 100 kHz, the same, ten of ten. The preempting object's flood
of probes is answered `i2c_rejected` while the arbiter's queue is full,
one probe a pass `i2c_timeout` (parked behind the hold in its turn), and
after the release `i2c_ok`, or `i2c_nack_addr` for the one probe a pass
that meets the peer before its client is back. THE ONE-ENTRY TENURE AND
ANOTHER NODE'S STOP: while the host took a STOP for its own with its
transmit FIFO empty alone, the probe parked at the release ended
`i2c_bus_error` on the release's STOP in 15 of 24 pressed passes - its
one entry still in the FIFO at the vector's entry (IC_TXFLR 1,
MST_ACTIVITY clear) and taken by the block, the bus free, by the time
the handler read TFE, its START then going out for a tenure already
answered; with the host state machine's idle required beside the FIFO's
empty, it ended on its own STOP, `i2c_nack_addr`, in 78 pressed passes
of 78. The test costs the one-STOP handler two cycles (the cost table
above).

**Every shape, and three requesters, under every pressure as in the
control.** Letter `o` at 400 kHz against one serve counted to its end:
the probe, reads of 1 and 2 (the pump) and of 3, 8 and 32 (the engines),
write-then-reads of 1+1 and 1+2 (the pump) and 2+8 (its read on the
engines), writes of 1, 2, 3, 8 and 32 (the pump) - every one `i2c_ok`,
every read byte-exact, the peer's count, sum and seventeen address
matches exact (a write-then-read is two). Post to reply over ten runs,
the control's and the worst under pressure: the probe 102 to 148 and 115
to 179 us, the 32-byte write 782 to 798 and 796 to 844, the 32-byte read
784 to 796 and 796 to 855. Letter `w`: the Client's 20 write-then-reads
of 1+6 (two outstanding), a requester below the arbiter with 10 writes
of 2, and in a pressure pass the preempting object's 8 probes - every
reply `i2c_ok`, to its requester, in its order, nothing rejected, no
timeout, the peer's tallies exact, 5.7 to 6.8 ms a pass. Letter `p`: a
read of 16, a 1+4 and a write of 16 byte-exact at 100 kHz, 400 kHz and
1 MHz, and the peer's commanded stretch of 1000 us a byte on a read of 8
in 8827 to 8920 us against the 8762 its eight holds take at least.
Letter `q`: the deaf client's probe and write `i2c_nack_addr`, a write to
0x7A and one to the general call `i2c_rejected` with the peer, answering
the general call, matching only the probe that follows, the third
byte of six refused `i2c_nack_data`, the last written byte of a 2+4
refused `i2c_nack_data` with no read half after it, the next tenure
`i2c_ok` after each.

## Not covered yet

Driver gaps, each with its reason:

- 10-bit addressing on the host side: the resource has the mode and
  IC_TAR takes ten bits; the Request's `addr` is 7 bits on every
  stratum, and a 10-bit device is the first user.
- The START byte and the general call as host verbs: `target(addr,
  general_call)` is in the resource, and `start()` refuses both
  addresses with every reserved one
  ([../design/i2c-bus.md](../design/i2c-bus.md)); a broadcast is a
  feature of its own, born with its first user.
- A write phase on the DMA engines: an entry is a command word with a
  flag, which a byte buffer cannot carry; a halfword staging copy the
  size of the request is declined until a program needs a bulk write
  on the engines.
- The client on the DMA engines: the requests exist; a streaming
  client is born with a program that needs one.
- The receive-side hold (`sda_rx_hold`) and a client's `ten_bit`: a
  verb and a field, waiting for a wire that asks for them.
- Two clients on one address with a third party reading (twi_link's
  `coll`): this desk has no third node on the wire, and a DW block is a
  host or a client, never both.
- This chip as the bus's target, served from an active object, under
  the preemptive kernel: a target whose bytes come from an object (its
  vector posting "the host reads" and leaving SCL held, a dispatch giving
  the byte) is the one place a held-off reader would stretch the clock,
  and no util service serves a target that way (`I2cClient` is served
  from its vector). The partner's `twi_peer` could read it (its
  `host_read`). Born with the first util target object.

Implemented but not bench-verified, each with what would measure it:

- The client's general call (`ack_general_call`): GEN_CALL stood through
  the address's acknowledge when a host's general call reached this
  block - a read of IC_CLR_GEN_CALL inside that bit did not hold, the
  line re-entering two to seventeen times in the 8 us after the address
  at 100 kHz (traced: six entries 1 us apart, then RX_FULL a byte
  later), and no STOP_DET followed (STOP_DET_IFADDRESSED, the note on
  IC_CON bit 7) - which is why `service()` masks the source until the
  call's first data byte and reports one event. No brio host sends a
  general call now, the partner's included, so no suite reaches it; a
  controller that sends one would measure it.

- `rebase()`: this stratum has no dynamic clock yet; measured when the
  clock chapter grows one, a tenure exact after a switch.
