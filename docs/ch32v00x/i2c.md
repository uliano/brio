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
  nothing in flight, and with ITEVTEN up it re-enters the event vector
  without end until its sequence (a STAR1 read, then a CTLR1 write,
  15.10.6) clears it.
- **BTF stands until the STOP or the repeated START is on the wire**
  (15.10.6: the hardware clears it "after initiating a start or stop
  event"): requested at EVT8_2, either one leaves the event line
  asserted for the half a bit the condition takes to go out - measured,
  one or two vector entries for nothing at 400 and 100 kHz.
- **A repeated START requested while a byte shifts survives that byte's
  NACK** (measured): the controller generates the Sr after the refused
  byte all the same, START clearing and SB rising a few tens of cycles
  after the error vector sees AF; a CTLR1 store that clears START and
  sets STOP while it does so leaves STOP standing over a master that
  holds the bus (at 400 kHz; at 100 kHz the store lands first).

**What the chapter offers a controller's transfer, and what the engine
takes** - the inventory the costs below are read against:

| offer (section) | taken? | why |
|---|---|---|
| the event machine, SB / ADDR / TxE / RxNE / BTF on one vector and the five errors on another (15.3, 15.7) | yes | the silicon's: no byte counter and no FIFO, so the pump is one entry a data byte and the frame a few entries more (SB, ADDR, the end) |
| ITBUFEN as a switch inside a tenure (15.10.2) | yes | TxE and RxNE reach the vector only while the pump needs them, BTF carries the ends |
| ITEVTEN as a switch between tenures (15.10.2) | yes | up at `start()`, down at the end: the BTF that stands while the STOP goes out, and the slave half's STOPF, enter no vector between tenures |
| the END of a write at BTF, EVT8_2 (15.3, figure 15-4) | yes, for a plain write's STOP | the last byte's acknowledge is known there |
| START set while the last written byte shifts, the repeated START generated at its end (15.10.1) | yes, for a write-then-read | a polled CH32 target loses the last written byte when the repeated START comes after BTF ([../ch32vx03/i2c.md](../ch32vx03/i2c.md)); requested on the TxE of that byte - the order WCH's interrupt example uses - it is on the wire as the byte ends (the findings) |
| START cleared by the user code, a requested start withdrawn (15.10.1) | declined | behind a refused last byte the controller generates the repeated START all the same, and the withdrawal races it: won at 100 kHz, lost at 400 kHz with the bus left held (the findings); the refusal is closed from the Sr's SB instead |
| POS and ACK for the two-byte and the N-byte receive (15.3, 15.10.1) | yes | the chapter's procedures by count |
| DMAEN with LAST (15.8, 15.10.2) | yes, the engine slots | a plain write of any length and a read of two or more bytes take no entry a byte; a one-byte read stays on the pump, whose ACK-before-ADDR sequence the DMA path cannot express, and so does the write half of a write-then-read: its repeated START wants the TxE of the last byte, which the transmit block gives only through a completion interrupt the channel is not armed for (a plain write takes no DMA interrupt) |
| PEC (15.9), 10-bit addresses as a host (15.3) | declined | no Request shape asks for them ("Not covered yet") |
| one CTLR1 store for START with POS and ACK down | yes | the address phase's control written once; read after STAR1 it is also the second half of a stale STOPF's clear |

## Types and verbs

[brio/ch32v00x/i2c.hpp](../../brio/ch32v00x/i2c.hpp), three layers:

- `I2c<1>` is the resource: `timing(I2cTiming)` (FREQ and CKCFGR with
  PE clear), `addresses(I2cAddressConfig)` (7 or 10 bits, a second
  7-bit address under ENDUAL, the general call - refused by
  `i2c_address_config_valid()`), `enable()`/`disable()`,
  `software_reset()` (SWRST pulsed), the control bits as verbs
  (`start()`, `stop()`, `ack()`, `pos()`, `no_stretch()`, `pec()`,
  `dma(on, last)`) and as one store each way (`control(set, clear)` on
  CTLR1, `interrupts(set, clear)` on CTLR2), the flags and the clearing
  sequences
  (`clear_addr()` = STAR1 then STAR2, `clear_stopf()` = STAR1 then a
  CTLR1 write, `clear_errors()` write-zero), the three interrupt
  enables. `i2c_timing_for(pclk, speed, duty)` solves CCR rounded UP
  (a bus at most as fast as asked) and refuses outside FREQ's window.
- `I2cHost<1, pins, TxEngine, RxEngine>` is the engine `I2cBus` (=
  `BusMaster`) drives. Its `Request` is the other strata's verbatim -
  `addr`, a tx span, an rx span, the reply, `speed` - one tenure that
  is a write, a read, a write-then-read with a repeated START, or the
  empty probe. `isr()` is the event vector's body, phase by phase; the
  event line is a tenure's (raised by `start()`, dropped at its end, and
  down through a DMA read phase, which the receive block's completion
  ends - over a channel that does not serve, RxNE and BTF would stand
  under it and starve the thread and the bus's timeout) and
  ITBUFEN is switched on and off within it so TxE/RxNE interrupt only
  while the byte pump needs them and BTF carries the rest. The repeated
  START of a write-then-read is requested on the TxE that says the last
  written byte went into the shifter, and goes out as that byte ends.
  `error_isr()` is the error vector's: a NACK becomes `i2c_nack_addr` or
  `i2c_nack_data` by the phase it landed in, ARLO `i2c_arb_lost`, the
  rest `i2c_bus_error`. A NACK of the last written byte, its repeated
  START already requested, is `i2c_nack_data`: the Sr the controller
  generates all the same is waited for, and its SB closes the tenure
  with a void write - the address with the write bit and STOP behind
  it. A speed the clock cannot
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
  raises inside a tenure, once no START or STOP stands. `unstick()`
  clocks a stuck client free by hand and clears the STOPF its own STOP
  leaves, and `recover()` puts the peripheral back (SWRST, the timing
  rewritten); `release()` pulses the block's reset before the gate - the
  vendor's DeInit, the one act that withdraws a request DMAEN raised on
  the CH32V203 ([dma.md](dma.md)). The engine slots are
  `DmaTxEngine<6>` and `DmaRxEngine<7>`, both or neither: a plain write
  of any length and a read of two bytes or more run on them; the
  one-byte read and the write half of a write-then-read stay on the
  pump. A PLAIN WRITE ENDS ON BTF, which the event vector takes anyway,
  with the transmit channel's count read at zero: the controller wrote
  the last byte a byte time before it left the shifter, so the transmit
  engine is armed for its errors alone and a write takes no DMA
  interrupt; a BTF with the count standing is a channel that stopped
  serving, and ends the tenure `i2c_dma_fault` with its STOP; a read ends on the receive block's completion, its one. `dma_isr()` reads the controller's one flag
  register once for both channels.
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
decline when the wire reads low. On the CH32V003F4P6 it builds as six
group images, the first five green there against the same SAM C21 peer on the same
two pads (the scan in 13 ms, every tenure shape and receive procedure
byte-exact, the two speeds at 94 and 330 kHz on the wire, the held SDA
freed by four pulses); the
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
- **The arbiter over the engine** carries queued tenures in order with
  a NACK delivered in its place, rejects what its queue cannot hold,
  votes for a sleep idle and against it busy. A held SDA is answered
  `i2c_arb_lost` in the same millisecond, the line still low; a clock
  held for 45 ms is answered `i2c_timeout` at the arbiter's 20, and
  the `recover()`ed engine carries the next tenure `i2c_ok`.
- **`unstick()`** clocks four pulses before the peer lets SDA go, and
  the STOPF its hand-made STOP leaves is cleared with no entry of the
  event vector.
- **Against an STM32G0 peer** (its `twi_peer`, its 2.2 kOhm pull-ups)
  the whole suite holds too: 56 verdicts, the four receive procedures,
  the vocabulary, the repeated START acknowledged and refused, the
  engines and the kernel letter.

### The repeated START: on the last byte's TxE, and the refusal

A write-then-read turns around on a repeated START, and the engine
requests it on the TxE that says the last written byte went into the
shifter: 15.10.1's START bit repeats the start condition, and requested
while a byte shifts it goes out as that byte ends, with no stretch of its
own (measured below). 15.3 ends a write at EVT8_2 instead -
"TxE=1, BTF=1", SCL held low past the last acknowledge - and WCH's polled
EEPROM example requests its repeated START there; WCH's interrupt example
requests it a byte earlier, as this engine does. What decides is the far
end: a POLLED target of this block's lineage loses the last written byte
when the START comes after BTF (the CH32V303VCT6 and the CH32V203C8T6,
[../ch32vx03/i2c.md](../ch32vx03/i2c.md)), and a register index followed
by a read is exactly that shape. Measured here against the STM32G0
target (`test_ch32_i2c` letter r): one to four bytes written, then four
read, at 100 and 400 kHz, through the pump and through the engines - the
target counts and sums every written byte, the read is its pattern, and
BTF never enters the vector behind the pending START (the pump's tenure
takes 9, 10, 11 and 12 entries at both speeds, where the BTF order took
13 to 16 at 100 kHz and 11 to 14 at 400).

THE REFUSAL. The START is requested before the last byte's acknowledge
is known, so a NACK of that byte arrives with the START standing (CTLR1
0x101 at the error vector) and is `i2c_nack_data`. What the controller
does next was measured from the error vector, STAR1 and CTLR1 polled with
nothing written: it GENERATES THE REPEATED START ALL THE SAME - START
clears and SB rises within a few tens of cycles. A CTLR1 store that
clears START and sets STOP races that generation: at 100 kHz it lands
first and the STOP alone goes out; at 400 kHz it lands while the Sr is
under way, and the controller is left with STOP standing over a master
that holds the bus (CTLR1 0x201, STAR2 MSL, BUSY and TRA, TxE up) and
the next tenure stalls - at every count, pump and engines. So the engine
touches nothing at the NACK: it waits for the Sr's SB and closes the
tenure with the one sequence EV5 allows, the address with the WRITE bit
and STOP requested behind it - a void write, Sr A P on the wire - its
ADDR cleared (or its own NACK taken) before the reply. Measured, one to
four bytes with the last refused, both speeds, pump and engines: sixteen
of sixteen `i2c_nack_data`, CTLR1 back to PE alone and MSL clear within a
millisecond, and the next tenure on the same host `i2c_ok`; the target
saw the void write's address in fifteen and missed it once, which ended
on that address's own NACK. The price of the order is the refusal's:
one void write on the wire; an acknowledged tenure pays nothing for it.

### The host's cost (bench_ch32_i2c, letter i)

The benchmark's I2C letter has an image of its own, `bench_ch32_i2c`:
`bench_ch32` whole is some 39 KB of this part's 40 KB of program flash
and more than its 8 KB of RAM (it builds as two group images here), and
two instantiations of the host and the peer's command channel are about
ten kilobytes more. It runs every tenure shape against the STM32G0 peer's `twi_peer`,
serving at 0x2C for a bounded window and holding its count of the bytes
moved against this side's: writes and reads of 1, 2, 16 and 255 bytes,
the register read (one written, a repeated START, 1, 2 and 16 read),
the probe answered and not, a write to the address nobody answers -
through the pump and through the engines, at both speeds. `wire` is the
bus's own time at the SCL period measured as the slope of two engined
writes (483 cycles at 100 kHz, 99 kHz; 123 at 400 kHz, 390 kHz); `the
rest` = wall - wire is the tenure's fixed cost, the instrument's share in
it (a stamp pair an entry, 44 cycles of it charged to the body, an idle
turn, a ruler read) the same in every column. The vendor's column is the
EVT library's polled master (its `CheckEvent` loops, the EEPROM example's
shape, `ch32v00X_i2c.c`), the CPU spinning through the whole tenure, its
wall ending once the STOP has gone out. At 400 kHz:

| tenure | the rest (cycles), entries | the EVT, polled (busy = wall) |
|---|---|---|
| write 1 | 1418, 4 | 766 |
| write 16 | 1418, 19 | 765 |
| read 1 | 1164, 3 | 517 |
| read 16 | 1508, 17 | 538 |
| register read 1 + 1 | 1967, 7 | 907 |
| probe answered | 945, 2 | 595 |
| write 255 through the engine, x | 1.00 | - |

- **`start()` is 190 cycles** (250 on the ruler, a read of it 60), where
  it was 421: the tenure copies the five fields its entries read and not
  the whole Request, and the address phase's control is one CTLR1 store
  and the event line one CTLR2 store.
- **The pump's entry is 105 cycles a byte** (149 between the stamps, 44
  of them the stamps'), one a byte being this silicon's.
- **The engines** take a 255-byte write or read in three entries of the
  I2C's own (SB, ADDR and the end) at x 1.00, the core busy 3 800 cycles
  of its 285 000; their fixed cost is the pump's or under it (1231
  cycles against 1418 for a 255-byte write).
- **The register read** (one byte written, the repeated START on its
  TxE, one read) takes seven entries and 1967 cycles of fixed cost; under
  the engines its write half is the pump's too (the inventory), and the
  same tenure costs 2051.
- **Against the EVT** the comparison is between two contracts: the
  polled loop's fixed cost is 350 to 1200 cycles under this engine's
  (no vector entry, no idle turn, no instrument stamp inside it) and its
  CPU is the bus's for the whole tenure - 284 000 cycles for a 255-byte
  write at 400 kHz, where this engine's pump spends 110 000 of them and
  its engines 3 800.

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
- The benchmark's I2C letter on the CH32V003: `bench_ch32_i2c` is a
  19 KB image against that part's 15 KB, and splitting it - the pump's
  ops in one image and the engines' in another - is born with the part's
  return to the desk.

Implemented but not bench-verified, each with what would measure it:

- `release()`'s reset pulse, the `i2c_dma_fault` on a BTF with the
  transmit count standing and the read phase with the event line down:
  written from the CH32V203's measurement and staged on no board of
  this family. A transmit channel held dead by USART2 released without
  the pulse ([dma.md](dma.md)'s first item) under a DMA write to the
  peer would stage the first answer; the peer's letters g and h stand
  for the healthy path.
- The DMA engines on the CH32V003: `test_ch32_i2c` letter g in its
  group image against a peer, the board being off the desk (on the
  CH32V006 the letter and `bench_ch32_i2c`'s engined lines are green).
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
- The repeated START's order against a CH32 TARGET: the bus here
  carries an STM32G0 target, which takes the last written byte in
  either order; the loss the order avoids is measured on the CH32V203C8T6
  and the CH32V303VCT6 ([../ch32vx03/i2c.md](../ch32vx03/i2c.md)). A CH32
  board running `twi_peer` on this bus, letter r against it, would
  measure it from this host.
- Letter r on the CH32V003: its own group image, the board being off
  the desk (the engine is the CH32V006's, the same code on the same
  registers).
