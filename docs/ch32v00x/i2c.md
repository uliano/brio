# I2C (CH32V00x)

The one I2C of RM ch. 15 - host and client, 7- and 10-bit addresses,
dual addressing and the general call, two speeds, clock stretching,
PEC, two DMA requests - and the host engine util/i2c_bus.hpp's arbiter
drives on this silicon as on the other three. Documents of record: the
CH32V00X reference manual V1.5 (15.3 for the host's event sequences,
15.4 for the target's, 15.5 for the errors, 15.8 for the DMA, 15.10
for the registers), the CH32V006 datasheet V2.0 (table 2-1-1 for the
pads).

## What the silicon does

- **The STM32F1's I2C, an event machine**: SB, ADDR, TxE, RxNE, BTF and
  STOPF on one vector, the five errors (BERR, ARLO, AF, OVR, PECERR)
  on another; each of 15.3's software sequences stretches SCL low
  until it runs (SB: read STAR1, write the address; ADDR: read STAR1
  then STAR2; the two-byte and N-byte receive procedures with their
  ACK and POS choreography).
- **NO RISE-TIME REGISTER.** 15.3's prose names an "R16_I2C1_RTR" that
  table 15-1 does not list; the block ends at CKCFGR. The SCL timing
  is CCR under F/S and DUTY, and CTLR2.FREQ must hold the bus clock in
  MHz between 8 and 48 (15.10.2) - below 8 MHz no speed is legal.
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
- **The default pads on the CH32V006** are SCL PC2 and SDA PC1 (table
  2-1-1). An alternate-function open-drain pad has no internal pull on
  this family: the pull-ups are the wire's.

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
  produce is answered `i2c_rejected` ASYNCHRONOUSLY, the event vector
  pended by software, so the arbiter's contract holds. `unstick()`
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
twi_link protocol, the peer's pull-ups) and decline when the wire
reads low. What the desk has measured so far is its wireless letter:

- **The reset values are table 15-1's**, both speeds resolve exactly
  at 48 MHz (CCR 240 and 40), the own addresses land as spelled.
- **OADDR1's bit 14 reads zero** when written: reserved, not the F1's
  fixed one.
- **CKCFGR and FREQ take a write with PE set**: no enable lock.

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

- **Every tenure on the wire**: the scan, the peer's command channel,
  the shapes (write, read, write-then-read, the four receive
  procedures by count, the general call), the vocabulary (NACK on the
  address and on a byte), stretching, the two speeds timed, the DMA
  engines, the arbiter with the peer's wedge answered by the timeout,
  the unstick - the suite's letters b..k against a peer board.
- The client side against a foreign host.
