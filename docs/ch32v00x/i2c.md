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
- **A START into a tenure ANOTHER CONTROLLER IS CLOCKING is held**
  (measured, raw): set 30 us into the peer's probe of an address nobody
  answers, it raised no ARLO and no BERR; the peer's STOP reached the
  pads 48 us after the store and SB rose 59 us after that STOP, the
  peer's tenure undisturbed (its probe ended `i2c_nack_addr`). So the two
  busy buses get two answers: a line held with no clock is refused, a
  live tenure is waited out.
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
  event"): requested at EVT8_2, a STOP leaves BTF up 513 cycles at 100
  kHz and 225 at 400 kHz, a repeated START 552 and 240 to 264 - about one bit
  and two bits - and SB rises 144 and 63 cycles after BTF falls
  (measured raw, the vectors dark). With the event line up that is a
  vector entry, or more, for nothing.
- **A repeated START requested while a byte shifts survives that byte's
  NACK** (measured raw): the controller generates the Sr after the
  refused byte all the same, SB rising 804 cycles after AF at 100 kHz and
  399 at 400 kHz (the far end a CH32 target); a CTLR1 store that clears
  START and sets STOP at the NACK leaves STOP standing over a master
  that holds the bus at 400 kHz (CTLR1 0x201, STAR2 MSL, BUSY and TRA a
  millisecond later), while at 100 kHz the STOP alone goes out.

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
  register once for both channels. Both engines arm at high, the
  transmit level of [../design/dma.md](../design/dma.md)'s rule: the host
  does not overrun - a byte a starved channel has not moved leaves BTF
  up and SCL held low until it is (15.3's notes), the bus slower and
  nothing lost.
- `I2cClient<1, pins>`: the target side - `init(clock, addresses,
  options)`, `I2cClientOptions` giving up the clock stretch or leaving
  the three enables and the two PFIC lines down for a POLLED client, the
  polled surface (`addressed()`, `answer_address()` returning the
  direction, `take()`/`give()`, `stop_seen()`, `host_nacked()`),
  `flush()` - the PE cycle that drops the byte a target transmitter is
  asked for beyond the host's closing NACK (15.10.1 and 15.10.6: PE
  clear clears the flags and ACK, the addresses and the timing stay) -
  and two ISR bodies, `service()` and `error_service()`, each reporting
  one `I2cClientEvent`.

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
its wire letters talk to the desk's CH32X035F8U6 running `twi_peer` (the
shared twi_link protocol) on I2C1's column 3 - PB5 SCL, PB6 SDA to that
board's PC16 and PC17, a 1.5 kOhm resistor from each line to its PB3,
which its I2C images drive high - and decline when the wire reads low.
On this part the suite is two images (`test_ch32_i2c-1`, the single-host
wire letters, and `-2`, the kernel, the two-host and the raw letters),
the whole being over the 40 KB of program flash; on the CH32V003F4P6 it
builds as eight group images, the kernel and the two-host letters left
out of that build, each alone over the part's 15 KB. What needs no peer:

- **The reset values are table 15-1's**, both speeds resolve exactly
  at 48 MHz (CCR 240 and 40), the own addresses land as spelled.
- **OADDR1's bit 14 reads zero** when written: reserved, not the F1's
  fixed one.
- **CKCFGR and FREQ take a write with PE set**: no enable lock.
- **Against the CH32X035** the whole suite holds: 47 verdicts in the
  first image and 29 in the second, the four receive procedures, the
  vocabulary, the repeated START acknowledged and refused, the engines,
  the kernel letter, the two hosts and the raw sequences, each on the
  first `z` after its flash.
- **THE RATE ASKED, on the wire** (a tenure timed on the ticker's
  cycle count from `start()` to its completion, nine SCL periods a
  byte with the address's, START, STOP and the software inside; the
  bracket: never above the rate asked by more than 2 %, within a
  quarter below it): an 8-byte write on the pump at about 95 kHz for 100
  asked and 342 kHz for 400, on the engines a 16-byte write and read at
  about 97 kHz each and a 16-byte read at about 369 kHz for 400. The
  wire's rise from the pad: SCL 94 to 100 cycles, SDA 110 to 113 (2 us),
over two runs.
- **A port read through its shut gate answers zero** (RM 3.4.7's PB2
  enable): right after a reset nothing has opened port B - the console
  is on port D - and the two lines would read low with the pull-ups on
  them, every wire letter skipping as if no peer were there. The suite
  opens the bus port's gate before it reads the lines; measured, the
  first `z` after a flash is whole.
- **A target of this lineage stretches one byte late**: the CH32X035
  holding every data byte of an 8-byte write for 2 ms lengthened the
  tenure by SEVEN holds (14 ms), the address's and the first six bytes',
  the last two falling after the tenure; and asked for a read, it left
  one byte in its DATAR past this host's closing NACK in each of the two
  read tenures of a serve, dropped by its `flush()` (the peer's
  `counters`), the bytes read exact all the same.
- **Two controllers on one wire** (letter j), both STARTs released by one
  STOP: aimed at 0x6B this host lost to the CH32X035's 0x2C and reported
  `i2c_arb_lost`, the peer's controller keeping the bus; aimed at 0x11 it
  kept the bus (`i2c_nack_addr`, nobody there) and the peer reported the
  loss. A START set into the peer's live tenure is held (the silicon's
  findings above); through the engine, `start()`'s bounded BUSY wait met
  the peer's STOP and returned 93 to 96 us after the call, its own probe of
  nobody `i2c_nack_addr` and the peer's tenure undisturbed.
- **This board as the client of a foreign controller**, polled: written
  six bytes by the CH32X035's host it answered its own address and took
  them byte-exact, ending on the STOP, its stretch one byte late (two
  milliseconds after the first byte was left in DATAR, RxNE and BTF both
  up: the next byte in the shifter, SCL held); READ twice six bytes back
  to back, it served each run byte-exact as a transmitter, one byte
  standing in DATAR at each closing NACK, and `flush()` dropped both -
  the second read opened on its own first byte.

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
by a read is exactly that shape. Measured here against the CH32X035
target, polled (`test_ch32_i2c` letter r): one to four bytes written,
then four read, at 100 and 400 kHz, through the pump and through the
engines - the target counts and sums every written byte, the read is its
pattern, and BTF never enters the vector behind the pending START (the
pump's tenure takes 9, 10, 11 and 12 entries at both speeds, the
engines' 6, 7, 8 and 9). That target does not show the loss itself: the
EVT library's polled master, its repeated START requested at BTF (the
EEPROM example's order), wrote a register index before 24 reads at each
speed and the CH32X035 took every byte - so the order is this engine's
for the targets that lose it, and costs nothing against one that does
not.

THE REFUSAL. The START is requested before the last byte's acknowledge
is known, so a NACK of that byte arrives with the START standing and is
`i2c_nack_data`. What the controller does next was measured raw
(`test_ch32_i2c` letter q, the vectors dark, STAR1 polled with nothing
written): it GENERATES THE REPEATED START ALL THE SAME - SB rises 804
cycles after AF at 100 kHz and 399 at 400 kHz. A CTLR1 store that clears
START and sets STOP at the NACK races that generation: at 100 kHz the
STOP alone goes out; at 400 kHz the controller is left with STOP
standing over a master that holds the bus (CTLR1 0x201, STAR2 MSL, BUSY
and TRA, a millisecond later). So the engine
touches nothing at the NACK: it waits for the Sr's SB and closes the
tenure with the one sequence EV5 allows, the address with the WRITE bit
and STOP requested behind it - a void write, Sr A P on the wire - its
ADDR cleared (or its own NACK taken) before the reply. Measured, one to
four bytes with the last refused, both speeds, pump and engines: sixteen
of sixteen `i2c_nack_data`, CTLR1 back to PE alone and MSL clear within a
millisecond (at 100 kHz the void write's STOP still stands at the reply,
CTLR1 0x201), and the next tenure on the same host - a probe of the same
address - completing on the wire: `i2c_nack_addr` in all sixteen, the
CH32X035 target keeping its acknowledge down after the refused byte until
its window closes, which refused the void write's address too. The price
of the order is the refusal's: one void write on the wire; an
acknowledged tenure pays nothing for it.

### This part as the desk's instrument: `twi_peer`

`twi_peer` (ch32v00x/src/apps/twi_peer.cpp) is the CH32V006's peer on
the twi_link protocol, kind 0x06, on I2C1's column 3 - SCL PB5, SDA PB6,
the desk's I2C wires to the CH32X035, whose PB3 holds the pull-ups. Its
target is POLLED (`I2cClientOptions`, the three enables and the two PFIC
lines left down) and drops the byte it was asked beyond a closing NACK
with `flush()`; its host action is the instance's role switch, writing
(`arb`) or reading (`host_read`, the extended repertoire), and its
`counters` answer what a report has no field for. Against the
CH32X035's `test_x035_i2c` - every one of its 84 verdicts - it answered
108 commands with no nak and no vector entered while it was polled, and
showed the habits of this block as a target, each measured from the far
end:

- **A target transmitter is asked for one byte more than the controller
  takes**: at the closing NACK of a read a byte stands in DATAR, never
  clocked out; `flush()` drops it and the tally gives it back - 181 such
  drops in the run, two in the serve of two reads that letter d counts,
  and no byte of a later read out of its place.
- **The stretch is one byte late**: a 2 ms hold before every byte of an
  8-byte write lengthened the far host's tenure by seven holds, the
  address's and the first six bytes', the last two falling after the
  tenure - the block acknowledges the byte in its shifter and holds SCL
  only after it.
- **The refusal is a bit set in advance, and it stays**: with the
  acknowledge cleared for a refused byte, the far host's next probe of
  the same address inside the action's window was refused too
  (`i2c_nack_addr`); the action's deadline and the command-mode re-init
  behind it put ACK back.
- **Deaf is a re-init** at `deaf_addr`: the far host's write and probe
  answered `i2c_nack_addr`.
- **The host action watches the pads**: its start() run on the STOP that
  closes the far host's hand-made window, the wired-AND decided both ways
  (0x2C over 0x6B, 0x11 over 0x2C), each end reporting the loss it saw;
  addressing the far board's client it wrote six bytes byte-exact, and
  read it twice six bytes back to back, each read its own run.
- **The stuck line** is taken after forty microseconds of quiet wire and
  let go after the falling edges asked: the far host's `unstick()` counted
  four.

### The host's cost (bench_ch32_i2c, letter i)

The benchmark's I2C letter has an image of its own, `bench_ch32_i2c`:
`bench_ch32` whole is some 39 KB of this part's 40 KB of program flash
and more than its 8 KB of RAM (it builds as two group images here), and
two instantiations of the host and the peer's command channel are about
ten kilobytes more. It runs every tenure shape against the CH32X035's
`twi_peer`, serving at 0x2C for a bounded window and holding its count
of the bytes moved against this side's (19 324 bytes at each speed, the
counts and sums agreeing): writes and reads of 1, 2, 16 and 255 bytes,
the register read (one written, a repeated START, 1, 2 and 16 read), the
probe answered and not, a write to the address nobody answers - through
the pump and through the engines, at both speeds. `wire` is the bus's own
time at the SCL period measured as the slope of two engined writes (482
cycles at 100 kHz, 99.6 kHz; 122 at 400 kHz, 393 kHz); `the rest` = wall
- wire is the tenure's fixed cost - the far end's own answer included: a
polled target stretches the clock until its loop has taken each step -
and the instrument's share in it (a stamp pair an entry, 46 cycles of it
charged to the body, an idle turn, a ruler read) the same in every
column. The vendor's column is the EVT library's polled master (its
`CheckEvent` loops, the EEPROM example's shape, `ch32v00X_i2c.c`), built
with this stratum's crt, clock, console and ruler and run on the same
board against the same peer, the CPU spinning through the whole tenure,
its wall ending once the STOP has gone out. At 400 kHz:

| tenure | the rest (cycles), entries | the EVT, polled (busy = wall) |
|---|---|---|
| write 1 | 1373, 4 | 780 |
| write 16 | 1373, 20 | 797 |
| read 1 | 1200, 3 | 707 |
| read 16 | 1512, 18 | 741 |
| register read 1 + 1 | 1970, 7 | 1221 |
| probe answered | 912, 2 | 572 |
| write 255 through the engine, x | 1.00 | 1.00 |

(An entry count is the I2C's two vectors, channels 6 and 7 and the tick:
a tenure longer than a tick period may hold one of the tick's.)

- **`start()` is 193 cycles** (250 on the ruler, a read of it 57): the
  tenure copies the five fields its entries read and not the whole
  Request, and the address phase's control is one CTLR1 store and the
  event line one CTLR2 store.
- **The pump's entry is 105 cycles a byte** (151 between the stamps, 46
  of them the stamps'), one a byte being this silicon's.
- **The engines** take a 255-byte write or read in three entries of the
  I2C's own (SB, ADDR and the end) at x 1.00, the core busy 2 694 cycles
  of its 282 398; their fixed cost is the pump's or under it (1188
  cycles against 1373 for a 255-byte write).
- **The register read** (one byte written, the repeated START on its
  TxE, one read) takes seven entries and 1970 cycles of fixed cost; under
  the engines its write half is the pump's too (the inventory), and the
  same tenure costs 2060.
- **The pump takes no placement**: measured in the same letter through
  a twin host whose event pump was an instance in SRAM behind the same
  vector in the flash, an entry of a 255-byte write took 161 cycles
  between the stamps against 154 inline from the flash, and of a read
  177 against 182 - the call into SRAM and the return into the flash's
  two wait states (RM 18.3.1) eat what SRAM saves. With the VECTOR
  placed by the program in `.fast_text` the inline pump takes 92 and 106
  ([platform.md](platform.md), "A handler in SRAM"), so `I2cHost` takes
  no placement parameter on this family and a program that wants the
  pump from SRAM places its event vector.
- **Against the EVT** the comparison is between two contracts: the
  polled loop's fixed cost is 340 to 770 cycles under this engine's
  (no vector entry, no idle turn, no instrument stamp inside it) and its
  CPU is the bus's for the whole tenure - 281 972 cycles for a 255-byte
  write at 400 kHz, where this engine's pump keeps the core busy for
  74 908 of them and its engines for 2 694.

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
- The collision case (twi_link's `coll`, two clients on one address with
  a third party reading): an instance is a host or a client, and the desk
  has two boards.
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
- The kernel letter and the two-host letter j on the CH32V003, which no
  group image of the part holds (the suite's header says which letters
  each image carries): the arbiter over the engine and the client against
  a foreign controller are proven on the CH32V006, and that run stands
  for both parts by decision - the block, the engine and the arbiter are
  the same code on the same registers, and the wire letters around it run
  green on the CH32V003.
- The client's ISR bodies against a foreign host: the client's polled
  surface serves a foreign host in `test_ch32_i2c` letter j and in
  `twi_peer` (above); `service()` and `error_service()` bound to the two
  vectors are what an image serving the client from its vectors would
  measure.
- The repeated START's order against a target that LOSES the BTF-order
  byte: the desk's CH32X035 takes the last written byte in either order
  (the findings); the loss the order avoids is measured on the
  CH32V203C8T6 and the CH32V303VCT6
  ([../ch32vx03/i2c.md](../ch32vx03/i2c.md)), and one of them on this
  bus, letter r against it, would measure it from this host - neither is
  this part's desk partner.
- What a START into a live tenure puts on the WIRE: the flags and the
  peer's report measure it (held until the STOP, the peer undisturbed);
  the edges themselves want a logic analyser on the two lines.
- Letters r and q on the CH32V003: their own group images, the board
  being off the desk (the engine is the CH32V006's, the same code on the
  same registers).
