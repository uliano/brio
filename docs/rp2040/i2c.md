# I2C (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 4.3 (the
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
itself, and the PUBLIC NAMES `DwApbI2c<n>` (the resource), `I2cHost`
(the engine `I2cBus` drives) and `I2cClient` (the other end), this
chip's aliases of the three templates there. The reference suite:
`test_rp2040_i2c`, the block wireless and then the wire, which is the
desk partner's: I2C1 on GP2 (SDA) / GP3 (SCL) with 1.5 kOhm to this
board's 3V3 on each, to the WeAct RP2350B
([../boards/weact-rp2040.md](../boards/weact-rp2040.md)), whose peer of
the twi_link protocol is commanded over the bus under test; the same
board's own suite, `test_rp2350_i2c`, judges this chip's block as that
peer.

## What the silicon does

Two DW_apb_i2c controllers on clk_sys, each a host or a client and
never both, standard, fast and fast-mode-plus (no high-speed mode),
7-bit addresses on both sides and 10-bit ones, two FIFOs of sixteen
entries, one interrupt line per instance with thirteen sources, a DMA
request per FIFO. The component parameter register that would report
the FIFO depths is not implemented and reads zero: the depth is the
chapter's word, and measured.

THE HOST IS A COMMAND FIFO. Every IC_DATA_CMD entry is a byte to write
or a read command, with a RESTART bit and a STOP bit: the first entry
after idle issues START and the address in IC_TAR, a direction change
(or the RESTART bit) a repeated START, the STOP bit a STOP after that
entry. A transmit FIFO that runs EMPTY WITH NO STOP holds SCL low and
stalls the bus until the next entry (4.3.7). There is no address-only
entry: START, the address and STOP with no data cannot be issued, so
the probe is a one-byte read. Every failure - a NACK on the address
or on a byte, a lost arbitration, an abort asked for - is ONE
interrupt, TX_ABRT, with the reason in IC_TX_ABRT_SOURCE, and both
FIFOs stay flushed until IC_CLR_TX_ABRT is read. IC_CON, the counts,
the addresses and the hold times take a write only with ENABLE clear;
a disable completes when the bus activity does (IC_ENABLE_STATUS says
when), and a host mid-command with no STOP never disables - the abort
is the way out. TX_EMPTY and RX_FULL are levels against IC_TX_TL and
IC_RX_TL, set and cleared by the hardware; the other eleven sources
clear by reading their own register.

THE SCL TIMING is two counts per speed in ic_clk cycles (HCNT, LCNT;
the standard pair and the fast pair, fast-mode-plus using the fast
pair), the spike filter (SPKLEN), and the SDA hold and setup. The
block ADDS to what is programmed: the low period is LCNT + 1, the
high period HCNT + SPKLEN + 7 (4.3.14.1), and the floors are HCNT at
least SPKLEN + 5 and LCNT at least SPKLEN + 7 - table 450's own rows
sit exactly there. The specification's minimum high and low times
(4000 / 4700 ns, 600 / 1300, 260 / 500) bound the split, and a clock
too slow to meet them makes the speed unreachable: 2.7 MHz for
standard, 12 for fast, 32 for fast-mode-plus.

THE CLIENT answers IC_SAR and then: a written byte lands in its
receive FIFO (RX_FULL), a read raises RD_REQ and HOLDS SCL until a byte
is given - the clock stretch is the block's own -, the host's NACK at
the end of a read is RX_DONE, a repeated START while addressed is
RESTART_DET, the STOP is STOP_DET (for its own tenures alone under
STOP_DET_IFADDRESSED). Bytes queued beyond what the host takes are
flushed at its NACK and reported as an abort (ABRT_SLVFLUSH_TXFIFO). A
full receive FIFO holds SCL instead of overflowing
(RX_FIFO_FULL_HLD_CTRL): the client holds SCL as a TRANSMITTER, at an
empty transmit FIFO, and as a receiver only with its receive FIFO full.
IC_SLV_DATA_NACK_ONLY makes the client refuse every data byte, written
with the block disabled and the client idle; a refusal mid-tenure is a
disable, which "stops the current transfer at the end of the current
byte and does not acknowledge" (4.3.17, IC_ENABLE) and leaves
SLV_RX_DATA_LOST set. A general call raises GEN_CALL and NO STOP_DET
under STOP_DET_IFADDRESSED (the note on IC_CON bit 7). There is no
address-match event.

TWO HOSTS (4.3.8): a host's START waits while the bus is busy - a START
on the wire not yet closed by its STOP - so two hosts whose STARTs wait
on one STOP leave together and the wired-AND decides, the loser aborting
with ARB_LOST. STOP_DET is raised in host mode for every STOP on the
wire, another host's included.

THE PINS are fixed per instance under function 3 (table 279): I2C0
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
  `start` (false when the wire moves; true for the two refusals that
  move nothing, i2c_rejected for an unreachable speed and
  i2c_bus_error for a block that would not disable to take the
  address), `isr()`, `dma_isr()`, `status()`, `unstick()` (the pulses
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
  general call, the fastest speed it will see), `events` (the sources
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
of that chip's halves: **57 verdicts** in its letters a..l. The same block as the PEER, the RP2040 the
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
- THE GENERAL CALL, both ways. A host Request to address 0x00 - IC_TAR
  holding 0 with SPECIAL clear - goes out as the general call: the
  peer's client takes it and its two bytes. This block as the CLIENT,
  IC_ACK_GENERAL_CALL on, takes the peer host's three-byte general call
  - and GEN_CALL STANDS THROUGH THE ADDRESS'S ACKNOWLEDGE: a read of
  IC_CLR_GEN_CALL inside that bit does not hold, the line re-entering
  two to seventeen times in the 8 us after the address at 100 kHz
  (traced: six entries 1 us apart, then RX_FULL a byte later). The
  client's service() masks the source until the call's first data byte
  and reports one event. No STOP_DET follows a general call
  (STOP_DET_IFADDRESSED, the note on IC_CON bit 7), on either chip.
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
  the engine required its transmit FIFO empty at the STOP (its own STOP
  follows its last entry; the IP stratum's page). A new address written
  while another node holds the bus costs a disable of 94 us at 100 kHz.
- THIS BLOCK AS THE PEER HOST'S CLIENT: six bytes written to it exact
  with their STOP; two reads of six served one byte a read request, the
  peer's count, sum and both first bytes the run given, nothing flushed.
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
  | write of 1 | 6830 (x 1.11) | 1 | 155 | 709 | 700 |
  | write of 255 | 729 200 (x 1.00) | 31 | 8958 | 14 924 | 696 |
  | read of 16 | 49 637 (x 1.01) | 2 | 581 | 1604 | 847 |
  | read of 255 | 729 338 (x 1.00) | 32 | 13 085 | 19 300 | 834 |
  | register read, 1 + 1 | 13 023 (x 1.06) | 1 | 209 | 816 | 763 |
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

## Not covered yet

Driver gaps, each with its reason:

- 10-bit addressing on the host side: the resource has the mode and
  IC_TAR takes ten bits; the Request's `addr` is 7 bits on every
  stratum, and a 10-bit device is the first user.
- The START byte as a host verb: `target(addr, general_call)` is in the
  resource; no tenure shape asks for it.
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

Implemented but not bench-verified, each with what would measure it:

- `rebase()`: this stratum has no dynamic clock yet; measured when the
  clock chapter grows one, a tenure exact after a switch.
- The client's `overrun` event: a client configured without the hold
  when its receive FIFO is full.
- `dma_isr()`'s fault path (i2c_dma_fault): a bus error on a channel,
  staged with an address the fabric refuses.
- `client_disabled_while_busy`, read after a disable timed inside a
  host's read (its write-side twin, SLV_RX_DATA_LOST, is measured
  above).
