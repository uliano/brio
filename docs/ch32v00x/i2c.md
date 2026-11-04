# I2C (CH32V00x)

This document is for a programmer who talks to I2C devices - a sensor,
an EEPROM, a converter - with the part as the bus host, or who makes
the part a client another host addresses. It covers the one I2C of
RM ch. 15 - host and client, 7- and 10-bit addresses, dual addressing
and the general call, two speeds, clock stretching, PEC, two DMA
requests - in three layers: `I2c<1>`, the resource, its registers as
functions; `I2cHost`, the host, which runs one tenure (one START to its
STOP, the bus held for one request) at a time from its two interrupt
vectors, its data one byte an entry (a write's first two in one) or, when the
request names them, on its engines (the DMA channels bound to the host); and
`I2cClient`, the target
side. A program drives the host through util/i2c_bus.hpp's arbiter,
`I2cBus`, an active object of the program (a component with its own event
queue, served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)) that takes the
requests of the requesters (the active objects that post them) one at a time
in arrival order and sends each its reply
([design/i2c-bus.md](../design/i2c-bus.md)). The same register description
on both parts: the CH32V003's chapter 13 is the CH32V006's chapter 15 word
for word, the same peripheral at the same address, the same two DMA
channels, the same default pads.

The smallest use, a register read from the device at address 0x48 (a
write of the register number, then a read of two bytes behind a
repeated START):

```cpp
using Bus = brio::I2cHost<1>;                          // PC2 SCL, PC1 SDA
using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;

BRIO_CH32_VECTOR(i2c1_ev_handler) {
    if (Bus::isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
BRIO_CH32_VECTOR(i2c1_er_handler) {
    if (Bus::error_isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}

// In main(), after the clock task's init() and before the kernel runs:
if (!Bus::init(clock)) {                               // both speeds solved at 48 MHz
    // a clock under 8 MHz: no speed is legal, no bus
}

// In a handler of Sensor. The two buffers are static: they are lent until I2cDone.
static uint8_t reg = 0x00;
static uint8_t value[2];
Bus::Request r{};
r.addr = 0x48;
r.tx = brio::lend<brio::Lease::reply>(&reg); r.tx_len = 1;
r.rx = brio::lend<brio::Lease::reply>(value); r.rx_len = 2;   // write-then-read
r.speed = brio::I2cSpeed::fast_400k;
r.reply = brio::reply_to<Sensor, brio::I2cDone>();
brio::post<I2c>(r);

// In Sensor's dispatch, the reply: value holds the two bytes only on i2c_ok.
brio::match(e, [](const brio::I2cDone& done) {
    if (done.status != brio::i2c_ok) {
        // a code of util/i2c_bus.hpp: what the wire did instead
    }
});
```

An excerpt: it assumes the program's platform `P`, its clock object
`clock` and an active object `Sensor` that handles `I2cDone`, listed with
`I2c` in the program's `Tenuto` (the cooperative kernel,
[design/kernel.md](../design/kernel.md)). `Bus` is the host on the default
pads with no DMA engines. `I2c` is the arbiter over it, with room for four
pending requests (the next is answered `i2c_rejected`, and past ten
outstanding the post itself is dropped with no reply, util/bus_master.hpp),
the failure policy `BusPassThrough` (no retry: every outcome goes to the
requester as it came) and a per-bus timeout of 20 ms. That timeout is the
one answer to a client (a device on the wire) that holds the clock through a
tenure, and it must sit above the longest tenure the program asks for: at nine SCL periods
a byte, 20 ms fits some 200 bytes at 100 kHz, and the longest request the
host takes, a write-then-read of 255 + 255 bytes, is 46 ms of wire at 100
kHz. The program binds both
vectors under the startup code's names, the event vector to `isr()` and
the error vector to `error_isr()`, and posts the host's status to the
arbiter when a tenure ends. `Bus::init()` takes the clock task's tag,
solves the timing of both speeds at the rate the clock type claims, hands
the two pads to the peripheral as open-drain outputs, and answers false
when not even the standard speed can be produced (the peripheral wants a
bus clock of 8 to 48 MHz); a program that goes on after a false has every
request answered `i2c_rejected`, with nothing on the wire. The request
names the 7-bit address, lends the host the register byte to send and the
two-byte buffer to fill (`lend<Lease::reply>`: the buffers stay the
sender's, lent until the reply comes), asks for 400 kHz and says where
the answer goes: an `I2cDone` event posted to `Sensor`, whose one field,
`status`, says what the wire did.

What decides correctness even there. The pull-ups are the wire's: an
open-drain alternate-function pad has no internal pull on this family,
so a bus with no resistors never rises. `init()` after the clock
task's, since the timing is computed from the rate the clock type
claims; while the chip is still at its 8 MHz reset rate the bus runs
six times slower than asked, and nothing refuses it. The buffers are untouched by
the sender from the post until `I2cDone` arrives, and nothing checks
that it keeps away. Both vectors must be bound: an unbound one lands in
the startup code's spin, the event vector at the first START and the
error vector at the first error of any kind (a NACK, a lost
arbitration, a bus error).

`start()`, which the arbiter calls in its own dispatch, waits there, so the
ticker's `init()` belongs before the first request: a bound of 5 ms for the last
tenure's STOP to leave the control register, then a bound of 100 us for the bus
to free. Both are timed on the system counter and each ends a few microseconds
past its name, later if a handler holds the core; with the counter stopped each
is a count of polls, some 26 ms and 800 us at 48 MHz ("The two bounds of
`start()`", below). Beyond that the program has one thing to keep, the per-bus
timeout: it answers a client holding the clock past the first bound, and
`start()` resets by itself a BUSY left up over an idle wire ("A BUSY left
standing", below). On a bus with a second controller a START's fate depends on
that controller's clock ("What the silicon does"). The
event vector moves a request's data one byte an entry when the request
names no DMA (the byte pump, or the pump, below), the entry that clears ADDR
loading a write's first two, and it waits once: in a
pumped read of three bytes or more it polls for the last byte, a bound of
100 000 polls behind which it reports `i2c_ok` whatever DATAR holds. That byte
is in the shifter by then, so no correct program spends the bound ("Not covered
yet").

## Documents of record

The CH32V00X reference manual V1.5 (15.3 for the host's event
sequences, 15.4 for the target's, 15.5 for the errors, 15.8 for the
DMA, 15.10 for the registers), the CH32V003 reference manual V1.9
(ch. 13, table 13-1), the CH32V006 datasheet V2.0 (table 2-1-1 for the
pads). The header is
[brio/ch32v00x/i2c.hpp](../../brio/ch32v00x/i2c.hpp); the reference
suite is `test_ch32_i2c`, and the host's cost is `bench_ch32_i2c`'s (a
suite's letter, below, is one command key of its console and the group
of verdicts it runs, `z` the key
that runs the image's set of letters, a letter outside it run by its
own key).

## What the silicon does

- **The STM32F1's I2C, an event machine**: SB, ADDR, TxE, RxNE, BTF and
  STOPF on one vector, the five errors (BERR, ARLO, AF, OVR, PECERR)
  on another; each of 15.3's software sequences stretches SCL low
  until it runs (EVT5, SB up: read STAR1, write the address; EVT6, ADDR up:
  read STAR1 then STAR2; EVT8_2, TxE and BTF up: the last byte is out,
  STOP or repeated START; the two-byte and N-byte receive procedures with
  their ACK and POS choreography).
- **No rise-time register, on either part.** 15.3's prose names an
  "R16_I2C1_RTR" that table 15-1 does not list, and the CH32V003's
  13.3 does the same against its table 13-1; both peripherals end at
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
  receive DMA block takes. 15.8 sets no length on a DMA transfer either way,
  and forbids ITBUFEN while DMA is used.
- **The default pads** are SCL PC2 and SDA PC1 on both parts (table
  2-1-1 of each datasheet; the remap columns are each part's own,
  pin.md). An alternate-function open-drain pad has no internal pull
  on this family: the pull-ups are the wire's.
- **A START into a line held low is answered ARLO** (measured): with a peer
  holding SDA low and no clock on the wire, the START comes back as
  arbitration lost at once, and the requester gets `i2c_arb_lost`.
- **A START into a tenure another controller is clocking at 100 kHz is held**
  (measured, raw, this peripheral at 100 kHz too): stored 30 us behind the STOP
  that lets the peer's probe of an address nobody answers go, it raised no ARLO
  and no BERR, and SB rose 103 or 114 us after the store, behind the peer's STOP
  (SDA's last rise under a high SCL, 82 us after the store in 16 of 18 runs),
  the peer's tenure undisturbed (its probe ended `i2c_nack_addr`). Swept 20 to 51
  us behind that STOP, SB rose in 1152 of 1152, wherever the store fell against the
  lines. So the two busy buses get two answers: a line held with no clock is
  answered ARLO, a live tenure at this rate is waited out.
- **Under a clock of 50 or 20 kHz that START mostly does not leave** (measured,
  raw, this peripheral a bystander at 100 kHz): swept into the peer's probe at 50
  kHz, 30 to 123 us by 3, it raised SB in 17 of 1152, and at 20 kHz, 30 to 247 us
  by 7, in 163 of 1152. In the others BUSY stood over both lines high with no SB at
  the end of the 3 ms the letter watches. The probe, 180 or 450 us long, was over
  by then, so its STOP did not take BUSY down. This row reads no report from the
  peer; through `start()` at 20 kHz the peer's tenure ended `i2c_nack_addr` in 576
  of 576. Why the rate decides is not established.
- **A client holding SCL is answered by nothing** (measured): the F1
  lineage's I2C has no clock-low timeout, and a 45 ms stretch stands
  with no flag raised - the per-bus timeout is the only answer.
- **The client half raises STOPF on a STOP it was not addressed in**
  (measured): after the START and STOP `unstick()` puts on the wire by
  hand - no address byte completed, ACK clear - STOPF stands with
  nothing in flight, and with ITEVTEN up it re-enters the event vector
  without end until its sequence (a STAR1 read, then a CTLR1 write,
  15.10.6) clears it.
- **BTF stands until the STOP or the repeated START is on the wire**
  (15.10.6: the hardware clears it "after initiating a start or stop
  event"). Measured raw, with the interrupt lines down, each requested at
  EVT8_2: at 100 kHz a STOP leaves BTF up 534 cycles and a repeated START
  492, about one bit, and SB rises 234 cycles after BTF falls; at 400 kHz
  a STOP 234 and a repeated START 168 to 270, one and a half to two and a
  quarter bits, and SB rises 66 or 169 cycles after it. With the event line
  (ITEVTEN) up that is a vector entry, or more, for nothing.
- **A repeated START requested while a byte shifts survives that byte's
  NACK** (measured raw): the controller generates the Sr after the
  refused byte all the same, SB rising 834 cycles after AF at 100 kHz and
  414 at 400 kHz (the far end a CH32 target); a CTLR1 store that clears
  START and sets STOP at the NACK leaves STOP standing over a master
  that holds the bus at 400 kHz (CTLR1 0x201, STAR2 MSL, BUSY and TRA a
  millisecond later), while at 100 kHz the STOP alone goes out.

- **TxE stands at ADDR, and the first two bytes of a write load at once**
  (measured raw, both speeds): STAR1 reads 0x82 at ADDR and 0x80 after its
  clear. The first store goes through the data register into the empty
  shifter, the next read of STAR1 finds TxE up (no read found it down in
  240 tenures), and the second store takes it down, STAR1 0x00, the byte
  waiting in the data register. Two stores with no read between them, and with
  one read whose answer is ignored, were byte-exact at the far end too.
- **BUSY is the wire's, and a STOP takes it down, but for one case** (15.10.7:
  "low level exists in SDA or SCL"; measured): SCL low alone raises it, and it
  stands over both lines high until a STOP is seen ("A BUSY left standing",
  below). The case: this peripheral's own START pending under a foreign clock of 50
  or 20 kHz, the bullet "Under a clock of 50 or 20 kHz" above. With PE clear BUSY
  reads 0, against 15.10.7's "still updated when the interface is disabled", and a
  START made with PE up, PE then cleared or the gate then shut over it, leaves
  nothing once the peripheral is back.

**What the chapter offers a controller's transfer, and what the host
takes** - the inventory the costs below are read against. The pump is the
event vector moving a request's data, one byte an entry; a data half is the
written or the read part of a tenure:

| offer (section) | taken? | why |
|---|---|---|
| the event machine, SB / ADDR / TxE / RxNE / BTF on one vector and the five errors on another (15.3, 15.7) | yes | the silicon's: no byte counter and no FIFO, so the pump is one entry a data byte past a write's first two, and the frame a few entries more (SB, ADDR, the end) |
| ITBUFEN, the buffer line, as a switch inside a tenure (15.10.2) | yes | TxE and RxNE reach the vector only while the pump needs them, BTF carries the ends |
| ITEVTEN, the event line, as a switch between tenures (15.10.2) | yes | up at `start()`, down at the end: the BTF that stands while the STOP goes out, and the client half's STOPF, enter no vector between tenures |
| the two bytes the ADDR entry can load, shifter and data register (15.3, EVT8_1 then EVT8) | yes | TxE stands there and rises again on the next read (the bullet above), so a write of one or of two takes three entries, SB, ADDR and BTF, and the one-byte index of a register read two, SB and ADDR, the second also requesting the repeated START; saved at 400 kHz: 180 to 350 busy cycles a tenure |
| the end of a write at BTF, EVT8_2 (15.3, figure 15-4) | yes, for a plain write's STOP | the last byte's acknowledge is known there |
| START set while the last written byte shifts, the repeated START generated at its end (15.10.1) | yes, for a write-then-read | requested on the TxE of that byte, in the ADDR entry for a one-byte index - the order WCH's interrupt example uses - it goes out as the byte ends, with no BTF entry of its own ("The repeated START", below) |
| START cleared by the user code, a requested start withdrawn (15.10.1) | declined | behind a refused last byte the controller generates the repeated START all the same, and the withdrawal races it: won at 100 kHz, lost at 400 kHz with the bus left held ("The repeated START", below); the refusal is closed from the Sr's SB instead |
| POS and ACK for the two-byte and the N-byte receive (15.3, 15.10.1) | yes | the chapter's procedures by count |
| DMAEN with LAST (15.8, 15.10.2) | yes, when the request names `I2cEngine::dma` | every data half at any length, no entry a byte (below the table) |
| PEC (15.9), 10-bit addresses as a host (15.3) | declined | no Request shape asks for them ("Not covered yet") |
| one CTLR1 store for START with POS and ACK down | yes | the address phase's control written once; read after STAR1 it is also the second half of a stale STOPF's clear |
| SWRST (15.10.1) | `recover()`, and `start()` for a BUSY standing over an idle wire | the chapter's own case, and its condition, "the pin of the I2C bus is released and the bus is idle", watched through the whole of `bus_free_us` ("A BUSY left standing", below) |

Under DMAEN a read of two or more runs under LAST, and a read of one has
its NACK set in the SB entry, before the address, and its STOP in the ADDR
entry, as the pump's are (15.3's one-byte procedure, the channel moving the
byte). Each DMA block's completion interrupt is taken. The transmit DMA block's
hands its half back to the event vector with DMAEN down (15.8: no ITBUFEN
under DMA), so BTF ends a plain write and the TxE of the last byte requests
a write-then-read's repeated START.

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
  enables. `i2c_timing_for(pclk, speed, duty)` solves CCR rounded up
  (a bus at most as fast as asked) and refuses outside FREQ's window.
- `I2cHost<1, pins, TxEngine, RxEngine>` is the host `I2cBus` (=
  `BusMaster`) drives. Its `Request` holds `addr`, `tx` and `tx_len`, `rx`
  and `rx_len`, `reply`, `speed` and `engine`: one tenure that is a write, a
  read, a write-then-read with a repeated START, or the empty probe.

  `isr()` is the event vector's body, phase by phase. `start()` raises the
  event line and the tenure's end drops it. It is also down through a read
  on the engines, which the receive DMA block's completion ends: over a
  channel that does not serve, RxNE and BTF would stand under the line and
  starve the thread and the bus's timeout. The buffer line goes up and down
  inside a tenure, so TxE and RxNE interrupt only while the pump needs them.
  The entry that clears ADDR stores a write's first byte and, on one read of
  STAR1 that finds TxE up, its second, and raises the buffer line only when a
  byte or a repeated START remains. The repeated START of a write-then-read is
  requested on the TxE that says the last written byte went into the shifter,
  in the ADDR entry itself for a one-byte index, and goes out as that byte
  ends.

  `error_isr()` is the error vector's body: a NACK becomes `i2c_nack_addr`
  or `i2c_nack_data` by the phase it landed in, ARLO `i2c_arb_lost`, the
  rest `i2c_bus_error`. A NACK of the last written byte, its repeated START
  already requested, is `i2c_nack_data`: the Sr the controller generates all
  the same is waited for, and its SB closes the tenure with a void write -
  the address with the write bit and STOP behind it.

  Three requests are answered `i2c_rejected` inside `start()`, delivered
  through the arbiter with the wire and the vector untouched: a speed the
  clock cannot produce, an address outside 0x08..0x77 (the general call and
  the 10-bit prefixes among them,
  [../design/i2c-bus.md](../design/i2c-bus.md)), and `I2cEngine::dma` on a
  host with no engines bound (`has_dma` false). An address-phase entry of
  the event vector that finds ADD10, SB or BTF where it waits for ADDR ends
  the tenure `i2c_bus_error` instead of re-entering for ever.

  Before its START, `start()` waits for the last tenure's STOP to leave
  CTLR1, a bound of 5 ms of the dispatch's time (no CTLR1 write may happen while
  STOP stands, and a client stretching the clock after the last acknowledge
  holds it there), then for BUSY to clear, a bound of 100 us. A clock held past
  the first bound is answered there: `start()` returns true with `i2c_timeout` in
  `status()`, nothing written, with or without a per-bus timeout ("The answer to a
  STOP held past its bound", below). BUSY still up after the second, no read having found a
  line low, is reset and counted by `unwedges()`. A line seen low gets the START
  anyway: a line held with no clock answers it ARLO, a tenure at 100 kHz holds
  it until its STOP, and one at 50 or 20 kHz mostly until the per-bus timeout's
  `recover()`.
  `isr()` clears a STOPF the client half raises inside a tenure, once no START
  or STOP stands.

  `unstick()` clocks a stuck client free by hand and clears the STOPF its
  own STOP leaves, and `recover()` puts the peripheral back (SWRST, the
  timing rewritten). `release()` stops the engines, clears PE and closes the
  gate with no reset, and `init()` starts from the peripheral's reset line,
  the vendor's DeInit, as [../design/dma.md](../design/dma.md)'s release
  contract asks.

  The engine parameters are `DmaTxEngine<6>` and `DmaRxEngine<7>`, both or
  neither, with both channels' vectors bound to `dma_isr()`, which reads the
  controller's one flag register once for both. The program calls no init
  of the DMA controller: the host's `init()` arms both engines, and arming
  opens the controller's gate. Which mechanism moves a request's data is the
  request's `engine` field (util/i2c_bus.hpp's `I2cEngine`): `interrupt`, the
  default, moves every byte from the event vector and touches no channel;
  `dma` puts every data half on the engines at any length. A read under
  `dma` ends on the receive DMA block's completion. A write under `dma` ends
  as the pump's does: the transmit DMA block's completion drops DMAEN and hands
  the half back to the event vector, so a plain write ends on BTF and a
  write before a read raises the buffer line and requests its repeated START
  on the TxE of its last byte. A BTF served before that completion reads the
  transmit channel's count: at zero the event vector completes the DMA block
  itself, and a count standing is a channel that has not served a TxE for a
  byte's time, starved by a block above it or dead. That, or a
  channel's transfer error, ends the tenure `i2c_dma_fault` with its STOP;
  this host declares that status in its header, beyond the four wire codes
  of util/i2c_bus.hpp.

  The launch of either engine cannot refuse a half of 1 to 255 bytes. The
  host asks nothing about its channels at run time: the compiler refuses a
  second owner of a channel (in the verb that arms it,
  [../design/dma.md](../design/dma.md), "The check"), and the host runs one
  tenure at a time. An error puts away only the DMA block the tenure has in
  flight. Both engines arm at high, the transmit level of the same page's
  rule: the host does not overrun - a byte a starved receive channel has not moved
  leaves BTF up and SCL held low until it is (15.3's notes), the bus slower
  and nothing lost.
- `I2cClient<1, pins>`: the target side - `init(clock, addresses,
  options)`, `I2cClientOptions` giving up the clock stretch or leaving
  the three enables and the two PFIC lines down for a polled client, the
  polled surface (`addressed()`, `answer_address()` returning the
  direction, `take()`/`give()`, `stop_seen()`, `host_nacked()`),
  `flush()` - the PE cycle that drops the byte a target transmitter is
  asked for beyond the host's closing NACK (15.10.1 and 15.10.6: PE
  clear clears the flags and ACK, the addresses and the timing stay) -
  and two ISR bodies, `service()` and `error_service()`, each reporting
  one `I2cClientEvent`.

## How to use it

The register read at the top of this page is the plain use: a request
that names no engine. A request that asks for the DMA engines, an excerpt
over that setup with the host given its two engines, their vectors bound
and `page` a static buffer of 34 bytes (the program also includes
ch32v00x/dma.hpp, which names the two engine types):

```cpp
// At namespace scope:
using Bus = brio::I2cHost<1, brio::i2c1_default_pins, brio::DmaTxEngine<6>, brio::DmaRxEngine<7>>;
static_assert(Bus::has_dma);

// In a handler of Sensor:
Bus::Request r{};
r.addr = 0x50;
r.tx = brio::lend<brio::Lease::reply>(page); r.tx_len = 34;   // address word + 32 data bytes
r.speed = brio::I2cSpeed::fast_400k;
r.engine = brio::I2cEngine::dma;
r.reply = brio::reply_to<Sensor, brio::I2cDone>();
brio::post<I2c>(r);

// At namespace scope, the two channels' vectors:
BRIO_CH32_VECTOR(dma1_channel6_handler) {
    if (Bus::dma_isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
BRIO_CH32_VECTOR(dma1_channel7_handler) {
    if (Bus::dma_isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
```

`engine` left out is `I2cEngine::interrupt`, the event vector's pump at
any length; `I2cEngine::dma` on a host declared without engines is
answered `i2c_rejected` with nothing on the wire. On a host with engines
either name delivers the same bytes, and the choice costs time or energy:
which costs the core less at which length, and what a working channel costs
a program that sleeps, is "Which engine to name" below.

## Bench findings

The reference suite is `test_ch32_i2c` on the CH32V006K8U6 at 48 MHz. Its
wire letters talk to a CH32X035F8U6 running `twi_peer` (peer firmware on
the shared twi_link protocol), and decline when the wire reads low. This
board uses I2C1's remap code 011, PB5 SCL and PB6 SDA, wired to that
board's PC16 and PC17, with a 1.5 kOhm resistor from each line to its
PB3, which its I2C images drive high. On this part the suite is four
images (`test_ch32_i2c-1`, the single-host wire letters and letter v; `-2`, the
kernel, the two-host, the dead-channel and the raw letters; `-3`, letters
n, l, u and t; `-4`, letter y), the whole being over the 40 KB of program flash. On the
CH32V003F4P6 it builds as nine group images under the part's 15 KB, and no
group holds the kernel, the two-host, the client-from-its-vectors, the
dead-channel, the n, the u, the t or the y letters ("Not covered yet").

- **The reset values are table 15-1's**, and both speeds resolve exactly
  at 48 MHz (CCR 240 and 40).
- **Against the CH32X035** the whole suite holds: 54 verdicts in the
  first image, 43 in the second and 27 in the third, the four receive
  procedures, the vocabulary, the repeated START acknowledged and
  refused (by a device's NACK), the engines, the kernel letter, the two
  hosts and the raw sequences, each on the first `z` after its flash. Letter `d` there: a write to 0x7A and one to
  the general call answered `i2c_rejected`, the peer's client, answering
  the general call, taking only the write to its own address that
  follows - one match, two bytes (ten runs of the letter, three of each
  image's `z`).
- **An address of 0x78 to 0x7B never reaches the wire**: `start()`
  refuses it, as letter `d` judges. So a correct program cannot reach what
  such a write's address byte, 1111 0xx0, would be to this machine: a 10-bit
  header, as on the STM32F1's peripheral 15.3 follows (CTLR1 and OADDR1 carry
  no controller addressing mode), which `unserved()` ends `i2c_bus_error` in
  the address phase.
- **The rate asked, on the wire** (a tenure timed on the ticker's
  cycle count from `start()` to its completion, nine SCL periods a
  byte with the address's, START, STOP and the software inside; the
  bracket: never above the rate asked by more than 2 %, within a
  quarter below it): an 8-byte write on the pump at about 95 kHz for 100
  asked and 347 kHz for 400, on the engines a 16-byte write and read at
  about 97 kHz each and a 16-byte read at 368 or 369 kHz for 400. Each
  line's rise from the pad reads SCL 97 cycles and SDA 109 in every run,
  an upper bound: the count starts before the call that releases the pad.
- **A port read through its shut gate answers zero** (RM 3.4.7's PB2
  enable): right after a reset nothing has opened port B, and the two lines
  would read low with the pull-ups on them. The suite opens the bus port's
  gate before it reads the lines.
- **A target of this lineage stretches one byte late**: the CH32X035
  holding every data byte of an 8-byte write for 2 ms lengthened the
  tenure by seven holds (14 ms), the address's and the first six bytes',
  the last two falling after the tenure; and asked for a read, it left
  one byte in its DATAR past this host's closing NACK in each of the two
  read tenures of a serve (a commanded spell as a target, reported
  afterwards), dropped by its `flush()`, the bytes read exact all the same.
- **Two controllers on one wire** (letter j), both STARTs released by one
  STOP: aimed at 0x6B this host lost to the CH32X035's 0x2C and reported
  `i2c_arb_lost`, the peer's controller keeping the bus; aimed at 0x11 it
  kept the bus (`i2c_nack_addr`, nobody there) and the peer reported the
  loss. The two STARTs are set microseconds apart, so a round can miss the
  race: one set after the other has reached the wire waits for that tenure's
  STOP, and the wire carries two tenures, each a NACK on its address. The
  letter tells a missed round by this board's time to its answer and retries
  it, up to five attempts; a race that came out wrong still fails. In 18 runs
  none of 54 rounds missed. A START set into the peer's 100 kHz tenure is held
  ("What the silicon does"): `start()`'s bounded BUSY wait met the
  peer's STOP and returned 96 or 97 us after the call, its own probe of nobody
  `i2c_nack_addr` and the peer's tenure undisturbed. A tenure that outlasts the
  wait is judged in "A BUSY left standing" below.
- **This board as the client of another chip's controller**, polled:
  written six bytes by the CH32X035's host it answered its own address and
  took them byte-exact, ending on the STOP, its stretch one byte late (two
  milliseconds after the first byte was left in DATAR, RxNE and BTF both
  up: the next byte in the shifter, SCL held); read twice six bytes back
  to back, it served each run byte-exact as a transmitter, one byte
  standing in DATAR at each closing NACK, and `flush()` dropped both -
  the second read opened on its own first byte.
- **The client from its vectors** (letter v): the same two cases with the
  interrupt enables up, `service()` and `error_service()` the bodies of
  i2c1_ev and i2c1_er - six bytes written to it byte-exact (eight event
  entries: ADDR, six RxNE, STOPF), two reads of six served as a
  transmitter (each closing NACK on the error vector, `flush()` there
  dropping the byte asked beyond it, the second read opening on its own
  first byte), seventeen event entries and two error ones, no entry
  finding nothing; the event body 83 or 84 cycles on average at 48 MHz,
  91 at most, stamped inside the vector.
- **The dead channel** (letter p): a timer's compare request raised by a
  match with no channel enabled, the timer's gate then closed with no
  reset - TIM1's channel 3 on DMA channel 6, TIM2's channel 2 on channel
  7 (table 8-2) - is held on this die: the channel takes one stray item at
  its next enable and serves nothing after it ([dma.md](dma.md)). A write
  of 32 on the engines over the dead channel 6 ends `i2c_dma_fault` - BTF
  with the transmit count standing, the far end holding one byte - where
  a host trusting BTF alone would report `i2c_ok`; a read of 16 over
  the dead channel 7 never completes, its event vector entered twice (SB,
  ADDR) and never stormed - the event line is down for the DMA block - so
  only a bound outside the host answers it: under the arbiter the per-bus
  timeout and its `recover()` (the letter's own wait stands in for it), and with
  no per-bus timeout nothing. Each completes intact once the timer is reset.
- **A block above the host's channels** (letter y, image `-4`, twelve runs of eight rows a shape):
  a memory-to-memory block of 30000 words on channel 1 at very_high, which leaves the
  channels below it nothing until it ends, at 100 kHz. Caught at its third byte, a
  write of eight ends `i2c_dma_fault` in every row with the block still running, the
  far end holding four, the event vector entered three times (SB, ADDR, the BTF that
  ends it): BTF with the count standing is answered once and does not re-enter, and
  the vector cannot tell this channel from a dead one. Up before the START the block
  costs a write nothing, no byte having gone to raise BTF: SCL stands and a write of
  two or eight lands whole when it ends. Reads of one, two and eight under it, and a
  read of eight caught at its third byte, are exact in every row with the event
  vector at two entries, the answer behind the block. A read of two starved from its
  start draws more off the target, which counted four, the reply exact. So a block
  above `high` that can outlast a byte (90 us at 100 kHz, 22.5 at 400) can end a write;
  a copy at its default `low` lies below it.
- **The peripheral's own request does not outlive its gate** (letter p): a
  read stalled over that dead channel, RxNE and BTF standing under DMAEN,
  gave channel 7's next enable four items while the peripheral stayed
  live, and none once its gate was closed - with PE up or cleared - or
  after `release()`, which closes the gate. So `init()`'s reset pulse
  finds nothing of this peripheral's to clear after these ways of letting
  go. The pulse stays because the release contract asks it of every DMA
  request on the chip, and a timer's request does outlive its gate.

### The repeated START: on the last byte's TxE, and the refusal

A write-then-read turns around on a repeated START, and the host
requests it on the TxE that says the last written byte went into the
shifter: 15.10.1's START bit repeats the start condition, and requested
while a byte shifts it goes out as that byte ends. 15.3 ends a write at EVT8_2
instead - "TxE=1, BTF=1", SCL held low past the last acknowledge - and WCH's
polled EEPROM example requests its repeated START there; WCH's interrupt
example requests it a byte earlier, as this host does.

Measured here against the CH32X035 target, polled (`test_ch32_i2c` letter
r): one to four bytes written, then four read, at 100 and 400 kHz, through
the pump and through the engines - the target counts and sums every
written byte, the read is its pattern, and BTF never enters the vector
behind the pending START (the pump's tenure takes 7, 8, 9 and 10 entries
at both speeds, a one-byte index's repeated START requested in the entry that
clears ADDR; under `I2cEngine::dma`, both halves on the engines, five
event entries and one completion of each DMA block, whatever the length
written).

No delivery depends on this order on this part. The polled master of WCH's
EVT library (the vendor's example package), its repeated START requested at
BTF, wrote a register index before reads at each speed and the CH32X035 took
every byte. The BTF order's loss was measured on the CH32V203C8T6 and the
CH32V303VCT6 ([../ch32vx03/i2c.md](../ch32vx03/i2c.md)), in three
arrangements: a target polled by the core that also serves the host lost
the byte, a target on the same chip served from its own vector did not, and
a target polled on another board did not. This part's one I2C cannot be both
ends. What the order buys here is the BTF entry it saves, and what it costs
is below.

**The refusal.** The START is requested before the last byte's acknowledge
is known, so a NACK of that byte arrives with the START standing and is
`i2c_nack_data`. What the controller does next was measured raw
(`test_ch32_i2c` letter q, the vectors dark, STAR1 polled with nothing
written): it generates the repeated START all the same - SB rises 834
cycles after AF at 100 kHz and 414 at 400 kHz. A CTLR1 store that clears
START and sets STOP at the NACK races that generation: at 100 kHz the
STOP alone goes out; at 400 kHz the controller is left with STOP
standing over a master that holds the bus (CTLR1 0x201, STAR2 MSL, BUSY
and TRA, a millisecond later). So the host touches nothing at the NACK: it
waits for the Sr's SB and closes the tenure with the one sequence EVT5 allows,
the address with the write bit and STOP requested behind it - a void write,
Sr A P on the wire - its ADDR cleared (or its own NACK taken) before the
reply. Measured, one to four bytes with the last refused, both speeds, under
`interrupt` and under `dma`: sixteen of sixteen `i2c_nack_data`, CTLR1 back
to PE alone and MSL clear within a millisecond (at 100 kHz the void write's
STOP still stands at the reply, CTLR1 0x201), and the next tenure on the same
host - a probe of the same address - completing on the wire: `i2c_nack_addr`
in all sixteen, the CH32X035 target keeping its acknowledge down after the
refused byte until its window closes, which refused the void write's address
too. The price of the order is the refusal's: one void write on the wire. An
acknowledged tenure pays for it only the SB entry's test of its phase.

### The first bytes of a write: at the entry that clears ADDR (`test_ch32_i2c` letter l)

15.3 draws two events behind ADDR's clear: EVT8_1, TxE up with the shifter and
the data register both empty, and EVT8, TxE up with the shifter full. The entry
that clears ADDR serves both, so what the pump would otherwise take in a TxE
entry each costs it a store. Fifteen runs of the letter, each verdict green:

- **The silicon, raw**: the flags of the bullet in "What the silicon does", and
  two bytes stored behind ADDR's clear byte-exact at the peer in all 240
  tenures of each of the three orders (TxE polled, one read ignored, back to
  back), half of them at each speed.
- **Entries, counted and judged**: a write of 1, 2, 3, 4 takes 3, 3, 4 and 5 (SB,
  ADDR, a TxE a byte past the second, BTF), a 1 + 1 five (SB, ADDR with the index
  and the repeated START, SB, ADDR, RxNE) and a 2 + 1 six, at both speeds. One
  more would be the STAR1 read finding TxE down: it left no byte to the buffer
  vector in 180 tenures.
- **The STOP stays at BTF**: a write of 1 or 2 with its last byte refused is
  `i2c_nack_data`, never `i2c_ok`, the acknowledge being known before the STOP
  is requested.
- **The first byte refused, the second committed in the data register**:
  `i2c_nack_data`, the far end's count and sum holding the refused byte alone,
  so the committed one never goes out; STAR1 0x00 and CTLR1 back to PE a
  millisecond later. The pump's own write position is an internal cursor over
  the bytes stored, reported to nobody. No flush is needed: the next tenure's
  address store at SB replaces the byte. A stale byte going out as the address
  would change who answers, so the letter arranges for that to show. With the stale
  byte the peer's command address, the next write, aimed at nobody, is still
  `i2c_nack_addr`; with the stale byte nobody's address, the next write, a whole
  command frame to the peer, is `i2c_ok` and its acknowledgement reads back.

### A BUSY left standing over an idle wire (`test_ch32_i2c` letters u and j)

BUSY can stand over both lines high after a line went low and no STOP followed,
and 15.10.1 names SWRST for it: it "can reset the I2C module when the stop
condition is not detected on the bus but the bus bit is 1", having asked that "the
pin of the I2C bus is released and the bus is idle". `start()` takes it behind a
watch of both lines. Eighteen runs of letter u, each making every staging below
eight times, and ten more right behind a far board's reset, 28 runs in all.

**What brings a correct program to it**, no hand on the wire:

- **A client holding the clock past the per-bus timeout.** The arbiter's
  `recover()` takes SWRST under the low SCL, BUSY rises again over it, and the
  client lets go with no STOP. In every one of 56 rounds, half of them at each
  speed: the stretched request `i2c_timeout` at 20 ms, then both lines high with
  BUSY up, and the next request takes the reset and is on the wire, 262 us after
  its post at 100 kHz and 185 at 400. That costs one timeout, the stretched
  request's own.
- **The pull-ups restarted.** On this bench they hang from the CH32X035's PB3,
  which lets go in a reset: both lines fall and rise together with no STOP seen.
  BUSY stood over both lines high after 9 of 9 far-board resets (the tenth found the image
  just flashed, the peripheral not yet up), and the first
  request behind each was `i2c_ok` with one reset. A board restarted on a live bus,
  its pull-ups with it, is this case.

**What makes it by hand.** On the pads, BUSY standing over both lines high and
still two milliseconds later, every time (224 of 224 each): a START, then SDA let
go under a low SCL, then SCL; SCL alone low and let go, no START at all; both
lines low over `recover()`, SDA let go first; SCL low over `recover()`; a START
over the reset line and the peripheral brought up again. Not standing, every time:
SDA alone low and let go, which is a START and its STOP; both lines low over
`recover()` with SCL let go first, a STOP; PE down over a START, or the gate shut,
with no STOP, both reading 0 at the end; a real tenure cut mid-byte by `recover()`.
No read of 1, 2, 3, 4 or 16 bytes left it, on the pump or on the engines at either
speed (0 of 2240, BUSY read 300 us behind each).

**The watch.** BUSY found up at the end of `start()`'s wait for the STOP sends it
out of line, and each read of the wait also reads both lines. The reset is taken
when BUSY stood for the whole of `bus_free_us` and no read in that time found
either line low; one read would not do, since both lines read high at some instant
of any tenure. The assumption this rests on: **a foreign
controller's clock whose high phase is longer than the bound, a clock under 5 kHz,
is taken for an idle wire.** SMBus's ceiling on that phase is 50 us.

A poll of the watch is 63 or 64 cycles, 1.3 us at 48 MHz (the bounds' section
below). A low phase shorter than that can fall between two reads, and a low phase is
what the watch looks for. At 100 kHz the low phase is 5 us, at 400 kHz in DUTY 2
1.67 us, 80 cycles, so at both speeds every low phase holds a read unless an
interrupt lands there; a tick's entry, 220 cycles, can hide two low phases at 400
kHz of the forty the bound spans.

Judged in letter j, a request posted into a tenure of the peer's controller longer
than the bound, the moment swept by 7 us, this request at 100 kHz. At 100 and 400 kHz
the peer writes 24 bytes that this board's own client half acknowledges, there being
no third chip on the wire. Host and client share CTLR1's ACK bit, and `start()`'s
store clears it for the address phase, so the client half refuses the next byte. At
20 kHz the peer probes an address nobody answers, this peripheral a bystander. The bound ran out in every round and **no reset was taken in
1728 of 1728 rounds** (18 runs, 32 rounds at each of the three rates). At 20 kHz the
peer's tenure ended `i2c_nack_addr` in all 576. At 100 and 400 kHz the letter counts it
clean when the peer reports `i2c_nack_data` or `i2c_ok`, 576 of 576 at each, and
by the arithmetic it cannot be `i2c_ok`: the store falls at most 335 us into a tenure
of 562 at its shortest.

**What a request meets, both ways out**, at either speed:

- **The watch finds both lines high throughout: the reset, then the wire.** SWRST
  leaves CTLR1, CTLR2 and CKCFGR at zero (read back), so `start()` writes the timing,
  PE and the error line again. Over a BUSY staged by hand `start()` returns in at most
  115 us (116 with a PE cycle for the other speed), answering false, the tenure then
  running in the vectors, and the probe is `i2c_ok` at most 224 us after the call at
  100 kHz and 152 at 400, one reset counted, two event entries, BUSY down behind it:
  224 of 224 at each speed over each of the two stagings. The derived bound beside the
  letter is 255 and 165. Through the arbiter, at 100 kHz, one reply, `i2c_ok`, 261
  us after the post, no timeout spent, and the next request `i2c_ok` in 151 to 154.
- **A line seen low in the bound: no reset, and the START goes in anyway.** Behind
  that, "What the silicon does" decides. Under the peer's 24 bytes at 100 and 400 kHz,
  this board's client half addressed, every request ended on the wire: its probe's
  `i2c_nack_addr`, 576 of 576 at each. Under the peer's 20 kHz probe, this peripheral a
  bystander, 569 rounds of 576 had no answer in the 10 ms the letter watches and 7
  ended `i2c_nack_addr`; the per-bus timeout is the answer. A held SDA is
  `i2c_arb_lost` at once.

**With no per-bus timeout** (`timeout_ticks` 0), staged:

- Over a BUSY left standing the request is `i2c_ok`, one reply, 260 to 262 us after
  its post, where the watch finds both lines high.
- A client holding the clock 45 ms and letting go cuts nothing, since no `recover()`
  runs: a write of two bytes is `i2c_ok` 45.5 ms after its post and no BUSY is left.
- A request posted right behind that reply, the client still holding SCL under the
  host's STOP, is answered `i2c_timeout` by `start()` ("The answer to a STOP held past
  its bound", below).
- A request into a foreign tenure under a clock of 50 or 20 kHz, its START mostly held
  past that tenure's end, is not answered: that one needs the per-bus timeout.

**What takes it down**, this peripheral's own START not pending: a START and a STOP
made by hand on the pads and SWRST, 224 of 224 each. Fifty milliseconds alone never
do. A PE cycle reads it down, 224 of 224, and `start()` does not use it: with the
reset taken out of `start()`, a request whose speed change cycled PE over a BUSY left
by a START with no STOP stayed held, STAR2 reading 0 and no answer in 30 ms, in 15
runs of 15, a staging today's letter does not make. `start()` takes SWRST before that
PE cycle, so a program changing speed does nothing.

### The two bounds of `start()` against their names (`test_ch32_i2c` letter t)

Eighteen runs of the letter, all green. The time is the ticker's cycle count around
the call, 20.8 ns a count, a read of it 57 cycles inside each figure, so a figure
reads high by that and never low. Each verdict takes the name as its floor and, as
its ceiling, the name plus the free bus's 273 cycles, one poll, a tick's entry and the
ruler. A floor on a figure that holds those 273 cycles would pass a wait up to 5.7 us
short; what keeps the wait from being short is the sum read below.

| wait | named | measured at 48 MHz | how it was staged |
|---|---|---|---|
| free bus | 0 | 273 cycles on the ruler, 5.7 us | nothing standing: the two first looks |
| `bus_free_us` | 100 us | 5191 to 5301 cycles, 108 to 110 us | SCL held low from this board's pad |
| `bus_free_us` with the reset | 100 us | 5442 cycles, 113 us, the reset's stores 4 of them | BUSY over an idle wire |
| `stop_drain_us` | 5 ms | 240 267 to 240 308 cycles, 5005 or 5006 us | a client holding SCL 45 ms behind a write of two, both speeds; `start()` true with `i2c_timeout`, no START, no vector entered |

Less `start()`'s own 5.7 us the `bus_free_us` wait is 102 to 105 us at the name of 100.
A `start()` that runs out returns before most of what that 5.7 us holds, so the figure at the
name of 5000 is not reduced the same way; the letter does not divide its 5 or 6 us
over the name. The poll is out of line and cold, reached on a bus not free: 42 cycles
waiting for the STOP, 63 or 64 in the watch. On a free bus the two first looks are
one load and a branch each, with no counter read and no call (the host's cost, below).

**The wait reads what the counter moved**, each poll adding the difference with one
period folded in where the counter restarts from zero (it counts up to its compare
and reloads, RM 6.5.4), so a bound spans ticks. It reads a register and consults no
handler, so it holds with interrupts masked. A gap of a whole period between two
polls, a handler a millisecond long, is lost from the sum and the wait ends later,
never earlier.

**Before the ticker runs** nothing moves the counter, so the wait also spends a
budget, the time asked at eight cycles a poll, which a running counter always beats
(the loop is more than eight instructions, on a core that retires one a cycle at
most). With STE cleared by hand, timed on TIM2 at 1 us a count: `bus_free_us` ends
after its 600 polls in 797 or 798 us with SCL low and 802 or 803 us over an idle wire, and
`stop_drain_us` after 30 000 polls in 26.3 ms. The wait ends in every case. A program
that calls `start()` before `Ticker::init()` has bounds that long.

**Why 5 ms and 100 us.** `stop_drain_us` holds the dispatch of every active object
under the arbiter's kernel and is there for a client digesting a write: 5 ms is a
fifth of SMBus's 25 ms clock-low limit and it stands under a 20 ms per-bus timeout with
room for the tenure behind it. `bus_free_us` is there for this host's
own STOP, a bit at 100 kHz, and is the watch's window: twice SMBus's 50 us ceiling on
a clock's high phase, so any such clock shows a low line inside it.

### The answer to a STOP held past its bound (`test_ch32_i2c` letter t)

A client that stretches after a write's last acknowledge holds the host's STOP in
CTLR1, and a request behind it finds that STOP standing. When `stop_drain_us` runs out
`start()` answers that request itself: true, `i2c_timeout` in `status()`, CTLR1
untouched, no vector. The arbiter replies and takes its next request, and no
`recover()` runs under the hold, so the STOP stays requested and goes out when the
client lets go. Nothing of the request reached the wire, so it can be posted again.

| | post to reply | the next request, the hold over |
|---|---|---|
| a 20 ms per-bus timeout, the client holding 15 ms, 100 and 400 kHz | 5032 to 5034 us | on the wire, 150 to 154 us from its post at 100 kHz, 77 at 400 |
| no per-bus timeout, the client holding 45 ms, 100 and 400 kHz | 5032 or 5033 us | the same |

Eighteen runs, 12 rounds a run, all sound: CTLR1's STOP up, START down and BUSY up at the
reply, no reset, no vector, and behind the hold STOP and BUSY down with nothing done.
The ceiling beside the letter is `stop_drain_us`, what `start()` costs around the wait,
a poll, a tick's entry, the ruler and the kernel's own turn, read in the same round as
the post-to-reply of a request refused with nothing on the wire (29 us).

What a program sees of it: a client that stretches past 5 ms after a write loses the
request behind it to `i2c_timeout` however long the per-bus timeout is, and that
request's 5 ms are the dispatch's. A requester that posts again at once meets the same
bound until the client lets go, each try 5 ms of every object's time. Requests waiting
behind the tenure when its reply comes are each answered in that one dispatch, 5 ms
apiece: two behind a 15 ms hold, every reply in 25.5 ms, and a queue of four holds
the kernel 20 ms. A program with such a client spaces its requests by the client's
own time.

### The mechanism a request names (`test_ch32_i2c` letter n)

Four rows against the CH32X035's counting serve, each judged by its
statuses, its bytes, the peer's count and sum and the DMA vectors'
entries, ten runs of the letter and three of its image's `z`, every
verdict green:

- **`dma` at one byte**: a one-byte write, a one-byte read and a register
  read of 1 + 1, every half on the engines - one completion of the
  transmit DMA block a write half, one of the receive DMA block a read half - the
  bytes exact and no fault counted. A read of one byte rides the receive
  DMA block with its NACK prepared before the address and its STOP set in the
  ADDR entry, LAST clear: the controller NACKs it and the channel moves
  it, at 100 kHz here and at 400 kHz in `bench_ch32_i2c`.
- **`interrupt` at sixteen bytes** on the host with engines bound: no DMA
  vector entered, both channels' configuration word and count as
  `init()` left them, 17 event entries for the write and 17 for the
  read.
- **`dma` on the host with no engines**: `i2c_rejected` inside `start()`,
  the vector never entered, MSL clear, the peer seeing only the request
  that followed.
- **The edges under `dma`**: an address nobody answers is
  `i2c_nack_addr` writing and reading; a NACK on the third of six bytes
  written is `i2c_nack_data`, channel 6 put away and one fault counted;
  the next write `i2c_ok`, whole. The repeated START and its refused last
  byte under `dma` are letter `r`'s, the lost arbitration letter `j`'s
  (`i2c_arb_lost` with no transmit DMA block started or thrown away).

### Which engine to name

The host chooses nothing: the program names the mechanism of each request.
On a host with engines bound, either name delivers the same bytes, and the
choice costs time or energy. On the CH32V006K8U6, tenures with no tick inside
compared, the engines cost the core fewer cycles for a write from four bytes and
for a read from three, at both speeds; under that, name the interrupt. **For a write-then-read, a short
register read stays cheaper under `interrupt`:** `dma` pays for two DMA blocks,
so the register read at the top of this page, one byte written and two read, is
2066 cycles on the pump against 2602 at 400 kHz, and the engines win from four
bytes read behind the index. These figures are that part's: the letter has no
image on the CH32V003 ("Not covered yet").

What each costs the core comes from `bench_ch32_i2c` letter i (below), as
`busy`: the cycles outside the idle path's windows
([../design/benchmark.md](../design/benchmark.md), "Busy"), the best of eight
tenures, over three runs of the letter. The lowest figure of a cell is a tenure
with no tick inside; the letter's `irq` counts a tick's entry in a higher one:

| tenure | 400 kHz: pump, engines | 100 kHz: pump, engines |
|---|---|---|
| write 1 | 1292, 1580 | 1429, 1717 |
| write 2 | 1304, 1580 to 2021 | 1441, 1717 |
| write 3 | 1631, 1717 or 1938 | 1631 or 1768, 1854 |
| write 4 | 1922, 1717 | 2059, 1854 |
| read 1 | 1379, 1492 | 1379 or 1516, 1629 |
| read 2 | 1409, 1474 | 1546 or 1957, 1474 or 1832 |
| read 3 | 1847, 1468 | 1984 to 2401, 1605 |
| read 4 | 2168 or 2664, 1468 | 2252 or 2305, 1605 to 1826 |
| register read 1 + 1 | 2036, 2620 | 2173, 2841 or 2978 |
| register read 1 + 2 | 2066 or 2150, 2602 or 2823 | 2066 or 2203, 2602 to 2960 |
| register read 1 + 3 | 2504, 2596 | 2641 or 2725, 2596 to 2817 |
| register read 1 + 4 | 2825, 2596 | 2909 or 3183, 2680 or 2733 |
| register read 1 + 16 | 6677 or 6898, 2596 | 6898 to 7256, 3175 |

The register read of 1 + 3 is 92 cycles in the pump's favour at 400 kHz (2504
against 2596), and at 100 kHz the two overlap (2641 or 2725 against 2596 to 2817). The letter also prints a first estimate from the
slope, which these rows overrule at the short lengths: a write's first two bytes share
an entry, so the pump's line is flat from one to two, and the engines' line steps up by
137 cycles from the third byte at the same entries, which the letter does not explain.

Against the wire: `b`, the pump's busy cycles a byte between its 16- and 255-byte
tenures, which holds each entry's trip and the letter's stamps as well as its body, is
295 cycles of a 1098-cycle byte at 400 kHz (a quarter of the core) and 310 of a
4338-cycle byte at 100 kHz (a fourteenth); the engines spend none a byte.

**The energy is a part's own.** On the CH32V006 the Sleep lets the DMA work
(QingKe V2C), and `idle()` sleeps through a DMA block: `dma` costs no awake
time. On the CH32V003 it does not (measured, [dma.md](dma.md),
[../../brio/ch32v00x/parts/ch32v003.hpp](../../brio/ch32v00x/parts/ch32v003.hpp),
`sleep_stops_bus_masters`), and `idle()` does not sleep while a channel
works ([../../brio/qingke/platform.hpp](../../brio/qingke/platform.hpp)): a
half on the engines keeps the core awake for its whole wire time - a
32-byte write at 400 kHz some 0.75 ms - where the pump would sleep between
its entries, so on that part `dma` buys core cycles at the price of a
sleep.

### This part as another board's peer: `twi_peer`

`twi_peer` (ch32v00x/src/apps/twi_peer.cpp) is the CH32V006's peer on
the twi_link protocol, on I2C1's remap code 011 - SCL PB5, SDA PB6, wired to
the CH32X035, whose PB3 holds the pull-ups. Its target is polled
(`I2cClientOptions`, the three enables and the two PFIC lines left down) and
drops the byte it was asked beyond a closing NACK with `flush()`. Through one
`z` of each of the two images of the CH32X035's `test_x035_i2c`, no command
was answered with a nak and no vector was entered while it was polled. It
showed the habits of this peripheral as a target, each measured from the far
end:

- **A target transmitter is asked for one byte more than the controller
  takes**: at the closing NACK of a read a byte stands in DATAR, never
  clocked out; `flush()` drops it and the tally gives it back - 254 such
  drops in those two `z`, two in the serve of two reads that letter d
  counts, and no byte of a later read out of its place.
- **A refusal is a bit set in advance, and it stays**: with the acknowledge
  cleared for a refused byte, the far host's next probe of the same address
  inside the action's window was refused too (a device's NACK,
  `i2c_nack_addr`); the action's deadline and the re-init behind it put ACK
  back.
- **A counted serve ends on the wire at rest, never on a Stop.** Polled as a
  target transmitter, this peripheral raised no STOPF the loop saw after a
  read (its flush, a PE cycle, falls between the closing NACK and the Stop, so
  whether the Stop alone would raise it is not separated), and a count
  reached by reads alone never ended a serve that waited for a Stop.
  `twi_peer` ends a counted serve once its count has moved and both lines have
  stood high for forty microseconds: armed for eight bytes and 2000 ms, it is
  back in command mode 9 ms after the read, the eighth byte its own (the far
  suite's letter d, ten runs).

### The host's cost (bench_ch32_i2c, letter i)

The benchmark's I2C letter has an image of its own, `bench_ch32_i2c`,
because it carries two instantiations of the host and the peer's command
channel. It runs every tenure shape against the CH32X035's
`twi_peer`, serving at 0x2C for a bounded window and holding its count
of the bytes moved against this side's (20 060 bytes at each speed, the
counts and sums agreeing): writes and reads of 1, 2, 3, 4, 16 and 255 bytes,
the register read (one written, a repeated START, 1, 2, 3, 4 and 16 read), the
probe answered and not, a write to the address nobody answers - through
the pump and through the engines (every request naming
`I2cEngine::dma`), at both speeds, and at each speed a slope estimate
("Which engine to name" above). `wire` is the bus's own
time at the SCL period measured as the slope of two engined writes (482
cycles at 100 kHz, 99.6 kHz; 122 at 400 kHz, 393 kHz). `the rest` = wall
- wire is the tenure's fixed cost. It holds the far end's own answer (a
polled target stretches the clock until its loop has taken each step), and in
the pump's column the instrument's share as well: a stamp pair an entry (42 to
48 cycles of it charged to the body by `bench_ch32_i2c` letter r), an idle
turn, a ruler read. `x` is wall over wire. The vendor's column is the polled
master of WCH's EVT library (its `CheckEvent` loops, the EEPROM example's
shape, `ch32v00X_i2c.c`), built with this stratum's crt, clock, console and
ruler and run on the same board against the same peer, the CPU spinning
through the whole tenure, its wall ending once the STOP has gone out, with no
stamp inside it. At 400 kHz:

| tenure | the pump: the rest (cycles), entries | WCH's library, polled: the rest (its busy is its wall) |
|---|---|---|
| write 1 | 1091, 3 | 780 |
| write 16 | 1091, 18 | 797 |
| read 1 | 1167 or 1168, 3 or 4 | 707 |
| read 16 | 1487 or 1488, 17 or 18 | 741 |
| register read 1 + 1 | 1659 to 1723, 5 | 1221 |
| probe answered | 918, 2 | 572 |
| write 255 on the engines, x | 1.00 | 1.00 |

(An entry count is the I2C's two vectors, channels 6 and 7 and the tick:
any tenure may hold one of the tick's, the longer the likelier.)

- **`start()` is 181 cycles** (238 on the ruler, a read of it 57): the
  tenure copies the five fields its entries read and not the whole
  Request, the two waits' first looks are one load and a branch each on a free
  bus, with no counter read and no call, the address phase's control is one CTLR1
  store and the event line one CTLR2 store; the refusal of `dma` on this host with
  no engines, a load and a compare, is 9 of them. With engines bound the same
  verb is 178 (235): the refusal folds away and the mechanism named is
  one store.
- **The pump's entry is 106 to 112 cycles a byte** (154 between the stamps,
  the slope of the letter's `isr` between its 4- and 16-byte writes at 400
  kHz with the one tick the longer holds taken out, 42 to 48 of them the
  stamps'), one entry a byte being this silicon's past a write's first two.
- **The engines** take a 255-byte write in four entries (SB, ADDR, the
  transmit DMA block's completion and BTF) and a read in three (SB, ADDR and
  the receive DMA block's completion), at x 1.00, the core busy 2 822 and
  2 794 cycles of 282 408 and 282 479 (the tick's entries inside
  them). Their fixed cost in the letter's lines, wall less wire (no column
  of the table above), is over the pump's for a write of any length (1198
  cycles against 1091) and for a read of one or two (1266 and 1275 against 1167 or
  1168 and 1184 or 1236), and under it for a read of three
  bytes or more (1269 against 1487 to 1514). That says when the tenure ends, not what
  the core pays: the core's cost is the busy cycles of "Which engine to
  name". The transmit DMA block's
  completion entry is the price of carrying a write half before a read:
  it hands the half back to the event vector for the BTF or the TxE that
  ends it.
- **The register read** (one byte written, the repeated START on its
  TxE in the entry that clears ADDR, one read) takes five entries and 1659 to 1723
  cycles of fixed cost on the pump; under `dma`, both halves on the engines,
  seven entries and 1855.
- **The pump takes no placement**: `I2cHost` has no placement parameter on
  this family, and a program that wants the pump from SRAM places its
  event vector in `.fast_text`, which carries the inline body with it
  ([platform.md](platform.md), "A handler in SRAM"). A body in SRAM behind
  a vector in the flash would still pay the trip through the flash and its
  two wait states (RM 18.3.1).
- **Against WCH's library** the comparison is between two contracts: the
  polled loop's fixed cost is 294 to 747 cycles under this host's and its
  CPU is the bus's for the whole tenure - 281 972 cycles for a 255-byte
  write at 400 kHz, where this host's pump keeps the core busy for
  76 039 to 76 273 of them and its engines for 2 822.

## Not covered yet

Driver gaps, each with its reason:

- 10-bit addressing on the host side: the resource has the mode and
  the client matches such an address; the host's header-byte sequence
  (15.3's EVT9, the second address byte) has no user.
- PEC as part of a tenure: the resource has the enable, no request
  shape asks for it.
- The general call as a host verb: `start()` refuses address 0x00 with
  every reserved address ([../design/i2c-bus.md](../design/i2c-bus.md));
  a broadcast is a feature of its own, born with its first user.
- A wake from Standby on an address match: this family's PWR chapter
  offers none.
- The collision case (twi_link's `coll`, two clients on one address with
  a third party reading): an instance is a host or a client, and the
  CH32V006K8U6's suite has one other board on its bus.
- A placement parameter on `I2cHost`: declined, the vector placed by the
  program being one attribute that carries the whole body. No suite
  compares the two; a twin host in `bench_ch32_i2c` letter i, its pump in
  SRAM behind the vector in the flash, would.
- The benchmark's I2C letter on the CH32V003: `bench_ch32_i2c` is a
  21 KB image against that part's 15 KB. Splitting it - the pump's ops in
  one image and the engines' in another - waits for a peer on that board's
  bus, whose pads reach none.

Implemented but not bench-verified, each with what would measure it:

- The target's general call (ENGC, GENCALL): armed and read back. No brio
  host sends a general call, so nothing here has exercised it on the wire;
  a controller that sends one would measure it.
- The refusal of an address outside 0x08..0x77 on the CH32V003: the same
  code on the same registers as the CH32V006's, where letter `d` judges it;
  letter `d` in its group image against a peer would measure it on that
  part, whose pads reach none.
- The address phase's end on ADD10 (`unserved()`), on both parts: `start()`
  refuses the address first, so no suite reaches it. An image with that
  refusal compiled out, against a client acknowledging 0x7A, would measure
  it.
- The DMA engines on the CH32V003: `test_ch32_i2c` letter g in its
  group image against a peer, the board's pads reaching none (on the
  CH32V006 the letter and `bench_ch32_i2c`'s engined lines are green).
  Letter n is in no group image of that part, alone over its 15 KB; on
  the CH32V006 it is green, the same code on the same registers, and a group
  of its rows sized for the part would measure it there.
- A foreign clock whose high phase is longer than `bus_free_us`: taken for an idle
  wire by the page's stated assumption. The peer's divider tops out at 85 us of high
  phase at 48 MHz, so no suite makes one; a controller under 5 kHz would measure it. A
  clock past this peripheral's own 400 kHz, whose low phases reads can fall between, is
  unmeasured too.
- Why a START under a foreign clock of 50 or 20 kHz mostly stays held ("What the
  silicon does"): measured at those rates with this peripheral at 100 kHz. The rates
  between 50 and 100 kHz, and this peripheral at 400 kHz under any of them, 100 kHz
  included, are unmeasured; a letter sweeping them would.
- The kernel letter, the two-host letters j and v, the dead-channel
  letter p, the standing BUSY's letter u, the bounds' letter t and the starved channels' letter y on the CH32V003, which no group image of the part holds (the
  suite's header says which letters each image carries): the arbiter over
  the host, the client against another chip's controller and the dead
  channel's guards are proven on the CH32V006, and that run stands for
  both parts by decision - the peripheral, the host and the arbiter are the
  same code on the same registers. On the CH32V003 only the letters that
  need no peer run green (`a` and `i`, 11 verdicts); its wire letters decline.
- What a START into a live tenure puts on the wire: at 100 kHz the flags and
  the peer's report measure it (held until the STOP, the peer undisturbed);
  the edges themselves want a logic analyser on the two lines.
- Letters r, q and l on the CH32V003: their own group images, the board's
  pads reaching no peer (the host is the CH32V006's, the same code on the
  same registers).
- The first bytes when the one read of STAR1 finds TxE down: the buffer vector
  then takes that step, the store or the repeated START, as it does for every
  later byte. No run reached it (letter l's entries count it none), and nothing
  here makes that read miss; an image with the read compiled to its false
  answer would measure the path.
- The last byte of a pumped read of three bytes or more when the wait
  for RxNE runs out: the host waits for it inside the event vector with
  a bound of 100 000 polls, tens of milliseconds at letter t's poll lengths (this
  loop is not timed), every other interrupt held out; when the bound runs out it reads DATAR regardless and reports `i2c_ok` with what it holds
  ([brio/ch32v00x/i2c.hpp](../../brio/ch32v00x/i2c.hpp), `receive_step()`). No
  correct program reaches it: at that point byte N is already in the shifter, so its
  RxNE follows the read of byte N-1 whatever the target does, and `bench_ch32_i2c`'s
  reads stay within 2100 cycles of their wire, against the bound's hundreds of
  thousands. An image with the poll's answer forced false would measure the path, and
  whether the verdict should then be a timeout rather than a success is open.
