# I2C (CH32V00x)

The one I2C of RM ch. 15 - host and client, 7- and 10-bit addresses,
dual addressing and the general call, two speeds, clock stretching,
PEC, two DMA requests - and the host engine util/i2c_bus.hpp's arbiter
drives on this silicon as on the other three. The same register
description on both parts: the CH32V003's chapter 13 is the CH32V006's
chapter 15 word for word, the same block at the same address, the
same two DMA channels, the same default pads. Documents of record:
the CH32V00X reference manual V1.5 (15.3 for the host's event
sequences, 15.4 for the target's, 15.5 for the errors, 15.8 for the
DMA, 15.10 for the registers), the CH32V003 reference manual V1.9
(ch. 13, table 13-1), the CH32V006 datasheet V2.0 (table 2-1-1 for the
pads).

## What the silicon does

- **The STM32F1's I2C, an event machine**: SB, ADDR, TxE, RxNE, BTF and
  STOPF on one vector, the five errors (BERR, ARLO, AF, OVR, PECERR)
  on another; each of 15.3's software sequences stretches SCL low
  until it runs (SB: read STAR1, write the address; ADDR: read STAR1
  then STAR2; the two-byte and N-byte receive procedures with their
  ACK and POS choreography).
- **NO RISE-TIME REGISTER, ON EITHER PART.** 15.3's prose names an
  "R16_I2C1_RTR" that table 15-1 does not list, and the CH32V003's
  13.3 does the same against its table 13-1; both blocks end at
  CKCFGR, and so do the vendor's own headers. The SCL timing is CCR
  under F/S and DUTY, and CTLR2.FREQ must hold the bus clock in MHz
  between 8 and 48 (15.10.2) - below 8 MHz no speed is legal.
- **The CCR arithmetic is the F1's** (stated by the vendor's init, not
  by the register description): standard mode SCL = pclk / (2 x CCR),
  fast mode pclk / (3 x CCR) at DUTY 2 or pclk / (25 x CCR) at 16/9.
- **OADDR1's bit 14 is reserved** (measured): the F1 lineage's "keep it
  at one" reads back zero here whatever is written.
- **CKCFGR and FREQ take a write with PE set** (measured): no lock
  behind the enable.
- **The DMA requests are channels 6 (transmit) and 7 (receive)**, table
  8-2, and CTLR2.LAST makes the controller NACK the last byte a
  receive block takes.
- **The default pads** are SCL PC2 and SDA PC1 on both parts (table
  2-1-1 of each datasheet; the remap columns are each part's own,
  pin.md). An alternate-function open-drain pad has no internal pull
  on this family: the pull-ups are the wire's.
- **A START into a busy bus is answered ARLO** (measured): with a peer
  holding SDA low, the START comes back as arbitration lost at once.
  The other strata's peripherals PARK such a START until the bus frees;
  this one answers, and `i2c_arb_lost` is the wire's own word for "not
  our bus".
- **A client holding SCL is answered by nothing** (measured): the F1
  lineage's I2C has no clock-low timeout, and a 45 ms stretch stands
  with no flag raised - the per-bus timeout is the only answer.
- **The slave half raises STOPF on a STOP it was not addressed in**
  (measured): after the START and STOP `unstick()` puts on the wire by
  hand - no address byte completed, ACK clear - STOPF stands with
  nothing in flight, and ITEVTEN routes it to the event vector, which
  it re-enters without end until its sequence clears it.

## Types and verbs

[brio/ch32v00x/i2c.hpp](../../brio/ch32v00x/i2c.hpp), three layers:

- `I2c<1>` is the resource: `timing(I2cTiming)` (FREQ and CKCFGR with
  PE clear), `addresses(I2cAddressConfig)` (7 or 10 bits, a second
  7-bit address under ENDUAL, the general call - refused by
  `i2c_address_config_valid()`), `enable()`/`disable()`,
  `software_reset()` (SWRST pulsed), the control bits as verbs
  (`start()`, `stop()`, `ack()`, `pos()`, `no_stretch()`, `pec()`,
  `dma(on, last)`), the flags and the clearing sequences
  (`clear_addr()` = STAR1 then STAR2, `clear_stopf()` = STAR1 then a
  CTLR1 write, `clear_errors()` write-zero), the three interrupt
  enables. `i2c_timing_for(pclk, speed, duty)` solves CCR rounded UP
  (a bus at most as fast as asked) and refuses outside FREQ's window.
- `I2cHost<1, pins, TxEngine, RxEngine>` is the engine `I2cBus` (=
  `BusMaster`) drives. Its `Request` is the other strata's verbatim -
  `addr`, a tx span, an rx span, the reply, `speed` - one tenure that
  is a write, a read, a write-then-read with a repeated START, or the
  empty probe. `isr()` is the event vector's body, phase by phase; the
  ITBUFEN enable is switched on and off within a tenure so TxE/RxNE
  interrupt only while the byte pump needs them and BTF carries the
  rest. `error_isr()` is the error vector's: a NACK becomes
  `i2c_nack_addr` or `i2c_nack_data` by the phase it landed in, ARLO
  `i2c_arb_lost`, the rest `i2c_bus_error`. A speed the clock cannot
  produce is answered `i2c_rejected` inside `start()` - the one
  synchronous completion, delivered through the arbiter with the wire
  and the vector untouched. Before its START, `start()` waits for the
  last tenure's STOP to leave CTLR1 - no CTLR1 write may happen while
  STOP stands, and a client stretching the clock after the last
  acknowledge holds it there - for up to 5 ms of the dispatch's time,
  then for BUSY to clear for up to 100 us; a clock held past the first
  bound PARKS the tenure (no START, no vector) for the per-bus timeout,
  and a bus still busy after the second gets its START anyway, which
  this silicon answers ARLO. `isr()` clears a STOPF the slave half
  raises in any phase, once no START or STOP stands. `unstick()`
  clocks a stuck client free by hand and `recover()` puts the
  peripheral back (SWRST, the timing rewritten). The engine slots are
  `DmaTxEngine<6>` and `DmaRxEngine<7>`, both or neither: a write
  phase of any length and a read of two bytes or more run on them, the
  one-byte read stays on the pump.
- `I2cClient<1, pins>`: the target side - `init(clock, addresses,
  no_stretch)`, the polled surface (`addressed()`, `answer_address()`
  returning the direction, `take()`/`give()`, `stop_seen()`,
  `host_nacked()`) and two ISR bodies, `service()` and
  `error_service()`, each reporting one `I2cClientEvent`.

## How to use it

```cpp
using Bus = brio::I2cHost<1>;                          // PC2 SCL, PC1 SDA
using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;

Bus::init(clock);                                      // both speeds solved at 48 MHz

Bus::Request r{};
r.addr = 0x48;
r.tx = brio::lend<brio::Lease::reply>(reg); r.tx_len = 1;
r.rx = brio::lend<brio::Lease::reply>(value); r.rx_len = 2;   // write-then-read
r.speed = brio::I2cSpeed::fast_400k;
r.reply = brio::reply_to<Sensor, brio::I2cDone>();
brio::post<I2c>(r);

extern "C" BRIO_CH32_INTERRUPT void i2c1_ev_handler() {
    if (Bus::isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
extern "C" BRIO_CH32_INTERRUPT void i2c1_er_handler() {
    if (Bus::error_isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
```

## Bench findings

The reference suite is `test_ch32_i2c` on the CH32V006K8U6 at 48 MHz:
its wire letters talk to a PEER BOARD running `twi_peer` (the shared
twi_link protocol, the peer's pull-ups, both boards at 3.3 V) and
decline when the wire reads low. On the CH32V003F4P6 it builds as five
group images: 38 verdicts green there against the same SAM C21 peer
on the same two pads (the scan in 13 ms, every tenure shape and
receive procedure byte-exact, the two speeds at 94 and 330 kHz on
the wire, the DMA engines, the held SDA freed by four pulses); the
kernel letter is left out of that build, its image alone 216 bytes
over the part's 15 KB. Against a SAM C21 peer:

- **The reset values are table 15-1's**, both speeds resolve exactly
  at 48 MHz (CCR 240 and 40), the own addresses land as spelled.
- **The wire's rise time, measured from the pad** (the wireless
  letter pulls each line low as an open-drain output, releases it and
  counts the cycles until it reads high): 1 us on either line with
  the peer's 1.5 kOhm pull-ups - and 283 us on a line whose pull-up
  it was NOT on, which a meter at rest reported as 3.3 V all the
  same. A slow line is what the scan's `i2c_arb_lost` on every START
  looks like from the register side, and the probe is the one that
  tells a pull-up from a leakage.
- **OADDR1's bit 14 reads zero** when written: reserved, not the F1's
  fixed one.
- **CKCFGR and FREQ take a write with PE set**: no enable lock.
- **The scan**: 112 addresses probed with the empty request in 14 ms,
  the peer's command address the only answer, every other one
  `i2c_nack_addr`.
- **Every tenure shape is byte-exact** - write, read, write-then-read on
  a repeated START, the four receive procedures by count (1, 2, 3 and
  4 bytes, each counted by the peer), the general call - and so is the
  vocabulary: a NACK on the address and on a commanded byte come back
  as `i2c_nack_addr` and `i2c_nack_data`.
- **Stretching is flow control**: a client stretching every byte of an
  8-byte write by 2 ms makes the tenure 16 ms long and it completes
  `i2c_ok`.
- **The two speeds on the wire**: an 8-byte write at 100 kHz runs at
  about 93 kHz, at 400 kHz about 324 kHz - CCR rounded up, the wire's
  rise time and the interrupt turnaround between bytes all counted -
  byte-exact both ways.
- **The DMA engines** on channels 6 and 7 carry a 16-byte write, a
  16-byte read and a write-then-read; the two-byte read ends on LAST's
  NACK, the one-byte read stays on the pump; no transfer fault.
- **The arbiter over the engine** carries queued tenures in order with
  a NACK delivered in its place, rejects what its queue cannot hold,
  votes for a sleep idle and against it busy. A held SDA is answered
  `i2c_arb_lost` in the same millisecond, the line still low; a clock
  held for 45 ms is answered `i2c_timeout` at the arbiter's 20, and
  the `recover()`ed engine carries the next tenure `i2c_ok`.
- **`unstick()`** clocks four pulses before the peer lets SDA go; the
  STOPF its hand-made STOP leaves is taken by the event vector once.

## Not covered yet

Driver gaps, each with its reason:

- 10-bit addressing on the host side: the resource has the mode and
  the client matches such an address; the host's header-byte sequence
  (EV9) has no user.
- PEC as part of a tenure: the resource has the enable, no request
  shape asks for it.
- The general call as a host verb: a Request to address 0x00 sends it
  (the suite does); no separate verb.
- A wake from Standby on an address match: this family's PWR chapter
  offers none.

Implemented but not bench-verified, each with what would measure it:

- The kernel letter on the CH32V003, which no group image of the
  part holds (the suite's header says which letters each image
  carries): the arbiter over the engine is proven on the CH32V006, and
  that run stands for both parts by decision - the block, the engine
  and the arbiter are the same code on the same registers, and the
  wire letters around it run green on the CH32V003.
- The client side against a foreign host: the peer's host half (its
  `arb` and `coll` commands) addressing this instance.
- A START into a wire ANOTHER HOST is clocking: ARLO is measured only
  against a held SDA; what the START puts on a live wire wants the
  peer's `arb` race and a scope.
