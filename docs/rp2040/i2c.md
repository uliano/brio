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
and the vocabulary. The driver: `brio/rp2040/i2c.hpp` (`DwApbI2c<n>`
the resource, `I2cHost` the engine `I2cBus` drives, `I2cClient` the
other end) over `pin.hpp`, `resets.hpp`, `dma_engine.hpp` and
`util/i2c_bus.hpp`. The reference suite: `test_rp2040_i2c`, on two
wires between the chip's two instances.

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
(RX_FIFO_FULL_HLD_CTRL). IC_SLV_DATA_NACK_ONLY makes the client
refuse every data byte, written with the block disabled and the
client idle. There is no address-match event.

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
  (the vocabulary's code for a source).
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
- `I2cHost<n, pins, TxEngine, RxEngine>`: the engine `I2cBus`
  (= `BusMaster`) drives. Its `Request` is the other strata's: `addr`,
  `tx` / `tx_len`, `rx` / `rx_len`, `reply`, `speed`. `init(clock)`
  (clk_sys, the three speeds solved), `rebase(hz)`, `speed_ok`,
  `timing_of`, `scl_hz`, `reference_hz`, `idle` (the engine's phase),
  `start` (false when the wire moves; true for the two refusals that
  move nothing, i2c_rejected for an unreachable speed and
  i2c_bus_error for a block that would not disable to take the
  address), `isr()`, `dma_isr()`, `status()`, `unstick()` (the pulses
  it took, 0 for a healthy wire, 0xFF for one that stays low),
  `recover()` (the block reset and reconfigured), `release()`. The
  pump writes entries while the FIFO takes them and refills on
  TX_EMPTY at the FIFO's half, takes bytes on RX_FULL, ends a write on
  STOP_DET and a read on the last byte. The engine slots take a
  `DmaTxEngine<ch, uint16_t>` and a `DmaRxEngine<ch>` on any two
  channels, both or neither, and serve a READ phase of three bytes or
  more: the plain read command poured from a fixed cell, the bytes
  collected, the first entry (RESTART) and the last (STOP) through the
  pump. A write phase stays on the pump.
- `I2cClient<n, pins>`: `init(clock, Config)` (address, 10-bit, the
  general call, the fastest speed it will see), `events` (the sources
  `service()` reports on), `data_ready` / `take`, `data_wanted` /
  `give` / `writable` / `clear_read_request`, `stop_seen` /
  `clear_stop`, `host_nacked` / `clear_nack`, `overrun`,
  `general_call_seen`, `acknowledge(on)` (a disable/enable pair),
  `flush_tx`, `service()` (one `I2cClientEvent` per call:
  byte_received, byte_wanted, stop, restart, nacked, general_call,
  flushed, overrun, error), `host_reads`, `release`.

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

The reference suite is `test_rp2040_i2c`, green on the Pico and the
WeAct board: I2C0 hosting on GP13/GP12 and I2C1 listening at 0x42 on
GP15/GP14, two wires between them with 4.7 kOhm pull-ups, the client
served from its own interrupt, both instances' interrupts on one core,
the roles inverted for the last letter.

- The block's reset state is a host with its client half disabled at
  the fast speed code with restarts enabled (IC_CON 0x65), the spike
  filter at 7, the hold at 1, and the component parameter register
  at zero. The transmit FIFO measured under TX_CMD_BLOCK is sixteen
  deep: full at sixteen, TX_OVER on the seventeenth, flushed by the
  disable. Configuration, timing, target and own address are refused
  while enabled; the disable of an idle block is reported within a
  few microseconds.
- THE COUNTS PRODUCE THE ASKED RATE. With the block's added cycles
  subtracted, a 64-byte tenure on the ruler gives 99 kHz, 391 kHz
  and 960 kHz for the three speeds on this wire - the remainder is
  the pull-ups' rise time. Counts that ignore the added cycles (the
  vendor's arithmetic) would run at 96, 376 and 912 on an ideal wire.
- THE SCAN of 0x08..0x77 finds the one client; nobody home is
  i2c_nack_addr inside 34 us at 400 kHz; the probe is served as one
  read request and one STOP at the client.
- THE SHAPES: a write of eight heard byte-exact with one STOP; a read
  of eight served as eight read requests, the host's NACK seen once;
  a write-then-read with ONE repeated START counted from the far end;
  a 200-byte write in one tenure through the pump in 4.6 ms and 24
  interrupts, no overrun at the client.
- THE VOCABULARY: a deaf address is i2c_nack_addr on a write, on the
  probe and on a read; a client refusing data (IC_SLV_DATA_NACK_ONLY)
  is i2c_nack_data on the first byte, which it does not store; the
  client's clock stretch is priced on the wire (100 us before each of
  eight bytes lengthens a 240 us read to 1027); bytes queued beyond a
  two-byte read are flushed at the host's NACK and reported
  (ABRT_SLVFLUSH_TXFIFO), the next read clean.
- THE ENGINES: a 64-byte read at 400 kHz in 1.7 ms with one host
  interrupt; a write-then-read with the write on the pump; the
  two-byte read and the probe on the pump of an engined host; a deaf
  address answered with the engines put away; eight 255-byte reads at
  1 MHz exact with TX_OVER never raised. THE REQUEST IS A LEVEL the
  DMA banks credits on while the transmit FIFO is empty and spends in
  a burst that fills it - a STOP entry pushed by hand at the block's
  completion was dropped with TX_OVER, which is why the last entry
  goes through the pump on TX_EMPTY.
- THE KERNEL over it, unchanged: four tenure shapes through I2cBus
  with four replies; the NACK of a deaf address delivered in its
  place; six posted into a four-deep queue, one rejected at once and
  every request answered once; an idle bus votes for the sleep, a busy
  one against. THE TIMED BUS: a client holding SCL through a read
  request it never serves is answered i2c_timeout at the arbiter's 20
  ms with SCL still held - no silicon on this chip answers a held
  clock - and the recover()ed engine carries the next tenures clean.
- THE STUCK BUS: unstick() answers 0 on a healthy wire, 0xFF with SDA
  held low by a GPIO through nine clocks and a STOP, 0 again once
  released, and the bus works after it. THE ROLES INVERT on the same
  two wires: sixteen bytes exact both ways.

## Not covered yet

Driver gaps, each with its reason:

- 10-bit addressing on the host side: the resource has the mode and
  IC_TAR takes ten bits; the Request's `addr` is 7 bits on every
  stratum, and a 10-bit device is the first user.
- The general call and the START byte as host verbs: `target(addr,
  general_call)` is in the resource; no tenure shape asks for either.
- Multi-master arbitration on the wire: ARB_LOST is decoded to
  i2c_arb_lost; the self-link's other instance is the client, and a
  second host needs a peer board.
- A write phase on the DMA engines: an entry is a command word with a
  flag, which a byte buffer cannot carry; a halfword staging copy the
  size of the request is declined until a program needs a bulk write
  on the engines.
- The client on the DMA engines: the requests exist; a streaming
  client is born with a program that needs one.
- The receive-side hold (`sda_rx_hold`) and a client's `ten_bit`: a
  verb and a field, waiting for a wire that asks for them.

Implemented but not bench-verified, each with what would measure it:

- `rebase()`: this stratum has no dynamic clock yet; measured when the
  clock chapter grows one, a tenure exact after a switch.
- The client's general call (IC_ACK_GENERAL_CALL, the `general_call`
  event) and the client's `overrun` event: a host tenure to address
  0x00 through the resource's `target(0, true)`, and a client
  configured without the hold when its receive FIFO is full.
- `dma_isr()`'s fault path (i2c_dma_fault): a bus error on a channel,
  staged with an address the fabric refuses.
- The client tenure cut by a disable (`client_disabled_while_busy`,
  `client_rx_data_lost`): a disable timed inside a host's write.
