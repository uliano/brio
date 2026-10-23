# I2C (CH32X035)

This document is for a programmer who needs an I2C bus: a sensor or a
memory read and written from an active object (a component of a brio
program with its own event queue, served one event at a time by the
kernel), or this chip as a target another controller addresses. It
covers the one I2C of RM ch. 15, host and client, 7- and 10-bit
addresses, dual addressing and the general call, two speeds with fast
mode's two duty shapes, clock stretching, PEC, two DMA requests; and
the host engine `I2cHost`, which the bus arbiter of util/i2c_bus.hpp
drives on this silicon as on every other stratum: an active object
posts a request, the arbiter runs it as one bus tenure (a START, the
address, the bytes, a STOP) and replies with an event when the wire is
done ([design/i2c-bus.md](../design/i2c-bus.md)).

The smallest use, a register read from a sensor at address 0x48:

```cpp
using Bus = brio::I2cHost<1>;   // the part's first column: PC16 SCL, PC17 SDA on the F8
using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;

Bus::init(clock);                // both speeds solved at 48 MHz; false: no block, no column

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

`Bus` is the engine over I2C1 on the part's default column; `I2c` is
the arbiter, an active object listed in the program's `Tenuto` with
four requests of queue and a 20 ms timeout; `init()` takes the clock
task's tag ([clock.md](clock.md)) and answers false for an HCLK under
4 MHz, a column the probe or the USB transceiver owns, or a block that
does not hold its timing (below); the request lends
its two buffers (`brio::lend`) until the reply arrives, so `reg` and
`value` must stay untouched and in scope from the post to the
`I2cDone` event `Sensor` receives; the two `extern "C"` functions are
the block's two interrupt handlers, as in C, under the crt's names and
with `BRIO_CH32_INTERRUPT` in front ([platform.md](platform.md)), each
posting the completion to the arbiter when the engine says the tenure
ended. What decides correctness even there: the clock task first
(`init()` solves the bus timing from the rate the clock type claims);
the two vectors bound; the bus pulled up by resistors of its own, since
a pad under the I2C's alternate function is open drain and the chip's
USB pull-up modes do not act under it (below); and `init()`'s answer
read, because on this series the block may be absent from a die (the
lot note, the first fact below), and a program that ignores a false
posts its requests to a bus that never came up.

The rest of this page, in order: the documents of record; what the
silicon does, with the chapter's offer item by item; the types and
verbs; how to use it; the bench findings, this part as the desk's I2C
instrument and the host's cost among them; what is not covered.

## Documents of record

The CH32X035 reference manual V1.8 (15.3 for the host's event sequences
and figures 15-3 and 15-4, 15.4 for the target's, 15.5 for the errors,
15.6 for the clock stretch, 15.9 for the registers; 8.2.8 and table 8-5
for the pads, 8.3.2.1 for I2C1_RM, 8.3.2.4 for the USB pads' pulls;
figure 9-1 for the DMA requests; table 7-1 for the vectors), the
CH32X035/X033 datasheet V1.7 (table 2-1 with its notes 3 to 7, 1.4.16),
and WCH's EVT library (`ch32x035_i2c.c` and the two I2C examples) as the
vendor's shape. Driver:
[brio/ch32x035/i2c.hpp](../../brio/ch32x035/i2c.hpp), the columns in
[brio/ch32x035/afio.hpp](../../brio/ch32x035/afio.hpp); reference suite
`test_x035_i2c` (a suite's letter, below, is one command key of its
console and the group of verdicts it runs, `z` the key that runs them
all) against the desk's CH32V006K8U6 running its `twi_peer`.

## What the silicon does

- **The block may not be there.** The datasheet withdraws the I2C - and
  the ADC's channels 3, 7, 11 and 15 - from every lot "with a 0 in the
  penultimate 5 digits of the lot number" (table 2-1's note 3, 1.4.16's
  note), and no register names a die's lot. The driver asks the block:
  `I2c<1>::present()` writes CKCFGR with PE clear and reads it back, and
  both `init()`s refuse a block that does not hold what they wrote. On
  the bench die (its marking unreadable) the block is there (measured,
  below).
- **The STM32F1's I2C, an event machine**: SB, ADDR, BTF, ADD10, STOPF,
  RxNE and TxE on one vector (table 7-1's entry 30), the five errors
  (BERR, ARLO, AF, OVR, PECERR) on another (31); each of 15.3's software
  sequences stretches SCL until it runs (EVT5: read STAR1, write the
  address; EVT6: read STAR1 then STAR2; the one-, two- and N-byte
  receive procedures with their ACK and POS choreography).
- **No rise-time register.** Table 15-1 ends at CKCFGR (0x1C) and so does
  the chapter's prose; the word after it holds nothing written to it
  (measured). The SCL timing is CTLR2.FREQ and CKCFGR: FREQ "must be set
  between 000100b and 111100b", 4 to 60 MHz (15.9.2), and every bus of
  the series runs at HCLK, whose ceiling is 48 MHz - so the window's
  ceiling is never reached and its floor is: an HCLK of 3 MHz or less, the
  bottom of the HPRE ladder, has no legal timing.
- **The CCR arithmetic is the F1's**, stated by WCH's I2C_Init and not by
  the register description: standard mode SCL = HCLK / (2 x CCR), fast
  mode HCLK / (3 x CCR) at DUTY 2 or / (25 x CCR) at 16/9. The vendor's
  function rounds CCR down - a bus faster than asked - and this driver
  rounds it up.
- **No SMBus mode, whatever the feature list says.** The datasheet's
  overview says "support SMBus/PMBus", CTLR1's STOP bit is "set by
  hardware when a timeout error is detected" and ITERREN lists TIMEOUT and
  SMBAlert - but CTLR1's bits 3:1 are reserved and read zero when written
  (measured), STAR1 has no TIMEOUT or SMBALERT bit (15:13 reserved), and
  ENARP (bit 4) takes its bit (measured) with nothing in the chapter
  behind it.
- **OADDR1's bit 14 is reserved** (measured: written, it reads zero) -
  the F1 lineage's "keep it at one" is not this register's.
- **CKCFGR and FREQ take a write with PE set** (measured): no lock behind
  the enable.
- **Six columns** in I2C1_RM's eight codes (8.3.2.1): 000 PA10/PA11, 001
  PA13/PA14, 010 PC16/PC17, 011 PC19/PC18, 1x0 PC17/PC16 and 1x1
  PC18/PC19, SCL named first. A pad under the I2C's alternate function is
  open drain by itself (8.2.8, table 8-5) - this series has no
  open-drain GPIO at all.
- **The USB pads' pulls do not reach the I2C.** AFIO_CTLR's UDP_PUE and
  UDM_PUE give PC17 and PC16 a weak, a 10 kOhm or a 1.5 kOhm pull-up, a
  mode "only ... set here", the GPIO configuration deciding (8.3.2.4).
  Measured with the bus's own pull-ups off: every mode, both output-data
  levels, under the alternate function - both lines driven low and
  released stay low; the 1.5 kOhm and 10 kOhm modes pull both pads up as
  pulled inputs. An I2C on PC16/PC17 wants resistors of its own.
- **The DMA requests are channels 6 (transmit) and 7 (receive)**, figure
  9-1 - ORs the I2C shares with USART2's receive and transmit - and
  CTLR2.LAST makes the controller NACK the last byte a receive block
  takes (15.9.2).
- **A START into a held SDA is not refused - it goes out, and the address
  loses.** With the peer holding SDA low, a START set by a block that saw
  the line taken raised SB 20 us later at 100 kHz, MSL and BUSY up, and
  the address written then lost arbitration 16 us on, at its first one
  bit (measured): the engine's reply is `i2c_arb_lost` at once, the
  CH32V006's word - which there came as ARLO on the START itself - through
  another path. The same START set by a block brought up over the line
  already held stays pending: no SB and no ARLO in 50 ms, START standing
  in CTLR1, BUSY up - the case the per-bus timeout answers (measured).
- **The host's own STOP stands in CTLR1 for about a bit** after the
  completion edge - 440 to 448 cycles at 100 kHz, 112 to 116 at 400 kHz,
  at 48 MHz (measured) - and no flag marks it leaving: `start()` waits for
  it, bounded, before anything writes CTLR1. A read-modify-write of CTLR1
  under it (STOP written back as it stood, ACK set) left no trace: STOP
  cleared on its own a millisecond later and the next tenure ran
  (measured) - the F1 lineage's rule is kept, the wait costing the bit.
- **The client half raises STOPF on a STOP it was not addressed in**,
  ACK clear or set (measured: a START and a STOP by hand, no address
  byte, STAR1 0x10), against 15.9.6's "after an acknowledge (if ACK=1)":
  `isr()` clears a STOPF it meets inside a tenure once no START or STOP
  stands, and `unstick()` clears the one its own STOP leaves.
- **A target stretches one byte late** (measured as the client): a byte
  left in DATAR lets the NEXT one into the shifter, and only then is SCL
  held - two milliseconds on, STAR1 reads RxNE and BTF both up. The
  desk's CH32V006 target does the same: a commanded 2 ms hold before every
  byte of an 8-byte write puts seven holds on the wire - the address's and
  the first six bytes' - and the last two fall after the tenure.

### The offer, item by item

What the chapter gives a controller's transfer, and what the engine takes:

| offer (section) | taken? | why |
|---|---|---|
| the event machine on two vectors (15.3, 15.7) | yes | the silicon's: no byte counter, no FIFO, so the pump is one entry a data byte past the first two and the frame a few entries more (SB, ADDR, the end) |
| ITBUFEN as a switch inside a tenure (15.9.2) | yes | TxE and RxNE reach the vector only while the pump needs them, BTF carries the ends |
| ITEVTEN as a switch between tenures (15.9.2) | yes | raised by `start()`, dropped at the completion with the buffer line and the DMA requests in one CTLR2 store: the BTF a STOP leaves standing and the client half's STOPF enter no vector between tenures (measured: no entry after a completion edge) |
| EVT8_1 and EVT8 in one entry (figure 15-3) | yes | the entry that clears ADDR loads the data register and the shifter - a one- or two-byte write takes three entries (SB, ADDR, BTF), a register index before a read none of its own (measured) |
| the END of a write at BTF, EVT8_2 (figure 15-3) | yes, for a plain write | the last byte's acknowledge is known there |
| START set while the last written byte shifts (15.9.1) | yes, for a write-then-read | a polled CH32 target loses the last written byte when the repeated START comes after BTF ([../ch32vx03/i2c.md](../ch32vx03/i2c.md)); the desk's CH32V006 is one, and took every byte in this order (measured) |
| START cleared by the user code, a requested start withdrawn (15.9.1) | declined | behind a refused last byte the controller generates the repeated START all the same and the withdrawal races it (lost on the CH32V006 and the CH32V203); the refusal is closed from the Sr's SB with a void write instead |
| POS and ACK for the two-byte and the N-byte receive (15.3, 15.9.1) | yes | the chapter's procedures by count |
| DMAEN with LAST (15.9.2), channels 6 and 7 (figure 9-1) | yes, the engine slots | a plain write of three bytes or more and a read of two or more; a write ends on BTF with the channel's count at zero, so the transmit engine arms for its errors alone and a write takes no DMA interrupt; a one-byte read stays on the pump (EVT6_1 has no DMA form), and so does a write half before a read (its repeated START wants the last byte's TxE) |
| SWRST (15.9.1) | yes | `recover()`, and `start()` for a BUSY standing over an idle wire, the chapter's own case |
| PEC (15.8), 10-bit addresses as a host (15.3), ENARP | declined | no Request shape asks for them ("Not covered yet") |
| the USB pads' pull modes (8.3.2.4) | declined | they do not act under the alternate function (measured) |

## Types and verbs

[brio/ch32x035/i2c.hpp](../../brio/ch32x035/i2c.hpp), three layers:

- `I2c<1>` is the resource: `present()` (the lot probe), `remap(code)`,
  `timing(I2cTiming)` (FREQ and CKCFGR with PE clear),
  `addresses(I2cAddressConfig)` (refused by `i2c_address_config_valid()`),
  `enable()`/`disable()`, `software_reset()`, the control bits as verbs
  (`start()`, `stop()`, `ack()`, `pos()`, `general_call()`,
  `no_stretch()`, `pec()`, `pec_next()`, `arp()`, `dma(on, last)`) and
  CTLR2 in one store (`control2(freq, enables)`), the flags and the
  sequences that clear them (`clear_addr()`, `clear_stopf()`,
  `clear_errors()`). `i2c_timing_for(hclk, speed, duty)` solves CCR
  rounded up and refuses outside FREQ's window; `i2c1_pins_for(code)` is
  a column, `i2c_pins_valid()` says whether it is a bus on the part (both
  pads bonded and alone on their pins), and `i2c1_default_pins` is the
  part's first column off the debug port - PC16/PC17 on the CH32X035F8,
  PA10/PA11 on the CH32X035R8, C8 and the CH32X033F8, the debug port's
  pair on the F7 and the two 28-pin parts, which have no other.
- `I2cHost<1, pins, TxEngine, RxEngine, Placement>` is the engine `I2cBus` (=
  `BusMaster`) drives, its `Request` the other strata's verbatim. `init()`
  is false for an HCLK under 4 MHz, a column on the debug port's pads
  while the probe owns them, a column on PC16/PC17 while AFIO_CTLR's
  USB_IOEN gives them to the USB transceiver, and a block that does not
  hold its timing. `isr()` is the event vector's body, phase by phase,
run through the host's `Placement` (util/placement.hpp's `InFlash`, the
default, or `InRam`);
  `error_isr()` the error vector's (a NACK is `i2c_nack_addr` or
  `i2c_nack_data` by the phase it landed in, ARLO `i2c_arb_lost`, the
  rest `i2c_bus_error`); `dma_isr()` both channels', INTFR read once. A
  speed the clock cannot produce is answered `i2c_rejected` inside
  `start()`. `start()` waits, bounded, for the last tenure's STOP to leave
  CTLR1 (5 ms, a clock held past it parks the tenure for the per-bus
  timeout) and for BUSY (100 us), then - BUSY still up and both lines
  high - takes SWRST and counts it (`unwedges()`). `unstick()` clocks a
  stuck client free by hand - a line pulled low as a push-pull output over
  a clear bit and released as a floating input, never driven high - and
  `recover()` puts the block back (SWRST, the timing rewritten).
  `release()` pulses the block's reset line before its gate
  ([../design/dma.md](../design/dma.md)'s release contract). `busy()` is
  the engine's phase, for a sleep site. The engine slots are
  `DmaTxEngine<6>` and `DmaRxEngine<7>`, both or neither, at `high` (the
  host stretches SCL behind a starved channel and loses nothing, 15.6).
- `I2cClient<1, pins>`: `init(clock, addresses, I2cClientOptions)` - a
  polled client with `.interrupts = false` -, the polled surface
  (`addressed()`, `answer_address()` returning the direction,
  `take()`/`give()`, `stop_seen()`, `host_nacked()`), `flush()` (the PE
  cycle that drops a byte the host never clocked), and two ISR bodies,
  `service()` and `error_service()`, each reporting one `I2cClientEvent`.

## How to use it

```cpp
using Bus = brio::I2cHost<1>;   // the part's first column: PC16 SCL, PC17 SDA on the F8
using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;

Bus::init(clock);                // both speeds solved at 48 MHz; false: no block, no column

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

On the engines, `I2cHost<1, i2c1_default_pins, DmaTxEngine<6>,
DmaRxEngine<7>>` and both channels' vectors calling `dma_isr()`. The
event pump in SRAM is either the host's `InRam`, its last parameter, or
the vector placed by the program with the host as it is - the faster of
the two (Bench findings, "The host's cost"):

```cpp
extern "C" BRIO_CH32_INTERRUPT [[gnu::section(".fast_text")]] void i2c1_ev_handler() {
    if (Bus::isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
```

## Bench findings

The reference suite is `test_x035_i2c` on the CH32X035F8U6 at 48 MHz
against the desk's CH32V006K8U6 running `twi_peer` on its column 3 (PB5,
PB6), the bus pulled up by a 1.5 kOhm resistor from each line to this
board's PB3, which the suite drives high from its boot - PC17 is the
board's boot pin, and a pull-up to the supply would send every power-up
into WCH's loader. The suite is two images, `test_x035_i2c-1` (the
single-host wire letters) and `-2` (the kernel, the two hosts, the dead
channel, the stuck BUSY, the vectors' client, the cost): `z` is 55 and
52 verdicts, green.

- **The lot probe** answered PRESENT: FREQ, CKCFGR and OADDR1 hold what
  they are given, PE sets, and a START on the idle bus raises SB with MSL
  and BUSY (after 22 polls of STAR1), a STOP behind it giving the bus
  back. The reset values are table 15-1's (all zero).
- **The wire's rise**: each line released from low reads high at the
  first read after the release - under the instrument's 50-cycle floor,
  a fraction of a 400 kHz bit.
- **The scan, the shapes, the vocabulary**: 112 addresses probed in 14 ms,
  the peer's command address the one answering and 111 `i2c_nack_addr`;
  a write, a read and a write-then-read against the peer, its count and
  the repeated START counted at its end; reads of one to four bytes, the
  four receive procedures, each closed at the right byte; the general
  call; `i2c_nack_addr` from a deaf peer and `i2c_nack_data` at a
  commanded byte.
- **The rate asked, on the wire** (a tenure timed from `start()` to its
  completion, nine SCL periods a byte with the address's; the bracket:
  never above the rate asked by more than 2 %, within a quarter below
  it): an 8-byte write on the pump at about 94 kHz for 100 and 336 kHz for
  400, on the engines a 16-byte write and read at 97 kHz and a 16-byte
  read at 363 kHz. Past the chapter, CKCFGR's fast-mode CCR at 16 (1 MHz
  asked) carried an 8-byte write and read byte-exact at about 700 kHz -
  measured, and no speed of the engine's: 15.1 names two.
- **The engines**: a 16-byte write and read, a write-then-read, a
  two-byte read under LAST and a 16-byte read at 400 kHz byte-exact, the
  peer accounting all 59 bytes, no transfer fault; no DMA interrupt for a
  write and one per engined read; a write three event entries (SB, ADDR,
  BTF), a read two.
- **The kernel**: four tenures through `I2cBus` answered in order with
  the NACK in its place, a full queue's rejection, both sleep votes; a
  tenure into an SDA the peer holds answered `i2c_arb_lost` at once and a
  clock held 45 ms answered `i2c_timeout` at the arbiter's 20 ms, the
  same bus AO carrying the next tenure to `i2c_ok` after each. A speed
  the clock cannot make is `i2c_rejected` inside `start()`, the vector
  never entered.
- **Arbitration in both directions**: two controllers' STARTs released by
  one STOP - this host aimed at 0x6B lost to the peer's 0x2C and reported
  `i2c_arb_lost`; aimed at 0x11 it kept the bus (`i2c_nack_addr`, nobody
  there) and the peer reported the loss. **As the peer's client**, polled,
  this board answered its own address in a write tenure and took six bytes
  byte-exact, ending on the STOP; read twice six bytes back to back by the
  peer's controller (twi_link's `host_read`), it served each run
  byte-exact as a transmitter, one byte standing in DATAR at each closing
  NACK, and `flush()` dropped both - the second read opened on its own
  first byte.
- **The peer as a target transmitter** is asked one byte more as well: in
  a serve of two read tenures the CH32V006 left one byte in its DATAR past
  each of this host's closing NACKs and dropped it (its `counters`), the
  bytes read exact.
- **The stuck bus**: the peer holding SDA until four falling edges,
  `unstick()` clocked four pulses, its hand-made START and STOP left no
  STOPF and no entry of the event vector, and the channel came back.
- **The repeated START** of a write-then-read requested on the last
  byte's TxE: one to four bytes written then four read, at 100 and 400
  kHz, pump and engines - the CH32V006, a polled CH32 target, took every
  written byte (its count and sum) and every read byte was its pattern;
  the pump's tenures took 7, 8, 9 and 10 event entries, the engines' 4 to
  7. The last written byte refused: `i2c_nack_data`, CTLR1 back to PE
  alone and MSL clear within a millisecond, and the next tenure - a probe
  of the same address - completed on the wire; it is `i2c_nack_addr`
  because a CH32 target keeps its acknowledge down after a refused byte
  until its window closes (the peer's own header).

- **Fast mode at DUTY 16/9** (letter f): CKCFGR 0xC005 - F/S, DUTY and
  CCR 5, 48 MHz / (25 x 400 kHz) = 4.8 rounded up so the bus never runs
  faster than asked - the chooser's 384 kHz; an 8-byte write at about
  322 kHz with the software inside it (334 at DUTY 2's 400: the same
  0.96), a write and a read byte-exact.
- **A BUSY left standing over an idle wire** (letter u): a START made by
  hand, SDA let go while SCL was held low, then SCL - no STOP condition
  ever on the wire - left BUSY up with both lines high; `start()`'s
  100 us wait ran out, `unwedge_busy()` took SWRST (`unwedges()` one
  more) and the tenure behind it completed `i2c_ok`, BUSY down after. A
  STOP made with the block's gate closed leaves nothing: BUSY read 0 at
  the reopening - the flag follows the wire.
- **The dead channel** (letter p): a timer's compare request raised by a
  match with no channel enabled, the timer's gate then closed with no
  reset - TIM1's channel 3 on DMA channel 6, TIM2's channel 2 on channel
  7 (figure 9-1) - is held: the next owner of the channel takes one stray
  item and serves nothing after it. (The software compare event, SWEVGR's
  CCxG, left the next owner nothing: the match is the request.) A write
  of 32 on the engines over the dead channel 6 ends `i2c_dma_fault` - BTF
  with the transmit count standing, the far end holding one byte - where
  an engine trusting BTF alone would report `i2c_ok`; a read of 16 over
  the dead channel 7 never completes, its event vector entered twice (SB,
  ADDR) and never stormed - the line is down for the block - so the
  caller's bound and `recover()` answer it; each lands whole once the
  timer is reset.
- **The block's own request does not outlive its gate** (letter p): a
  read stalled under DMAEN, RxNE and BTF standing, left the next owner of
  channel 7 four items while the block stayed live, and none once its
  gate was closed - with PE up or cleared - or after `release()`. The
  release contract's reset pulse finds nothing of this block's to clear
  in these lets-go; USART3's request on this die does outlive its gate
  ([dma.md](dma.md), "A released requester"), and the pulse is the
  contract for both.
- **The client from its vectors** (letter v): letter j's two cases with
  the interrupt enables up, `service()` and `error_service()` the bodies
  of i2c1_ev and i2c1_er - six bytes written to it byte-exact (eight
  event entries: ADDR, six RxNE, STOPF), two reads of six served as a
  transmitter (each closing NACK on the error vector, `flush()` there
  dropping the byte asked beyond it, the second read opening on its own
  first byte), seventeen event entries and two error ones, no entry
  finding nothing; the event body 113 to 128 cycles at 48 MHz, stamped
  inside the vector.

### This part as the desk's instrument: `twi_peer`

`twi_peer` (ch32x035/src/apps/twi_peer.cpp) is the CH32X035's peer on the
twi_link protocol, kind 0x07, on column 010 - SCL PC16, SDA PC17, the
desk's I2C wires to the CH32V006's column 3. It drives PB3 high, the
bus's pull-ups, from the supply check at boot (the voltage detector at
3.0 V; below it PB3 and the pads stay floating) to the end of its run,
command mode and every action included. Its target is polled
(`I2cClientOptions`) and drops the byte it was asked beyond a closing NACK
with `flush()`; its host action is the instance's role switch, writing
(`arb`) or reading (`host_read`); `hold_sda` pulls the line down with a
push-pull output over a clear bit, this series having no open-drain GPIO.
Against the CH32V006's `test_ch32_i2c` - its 93 verdicts over the suite's
two images on that part - it answered 111 commands with no nak and no
vector entered while it was polled, and showed this block's habits as a
target, each measured from the far end:

- **A target transmitter is asked for one byte more than the controller
  takes**: at the closing NACK of a read a byte stands in DATAR; `flush()`
  drops it and the tally gives it back - 176 such drops in the runs, two
  in the serve of two reads the far suite counts, no byte of a later read
  out of its place.
- **The stretch is one byte late**: a 2 ms hold before every byte of an
  8-byte write lengthened the far host's tenure by seven holds.
- **The refusal stays**: after a refused byte the far host's void write
  and its next probe of the same address inside the action's window were
  refused too (`i2c_nack_addr`, sixteen of sixteen).
- **The last written byte before a repeated START is taken in either
  order**: the far host's engine requests the Sr on that byte's TxE, and
  WCH's EVT library, built and run on the CH32V006, requests it at BTF -
  this target took every written byte both ways, at 100 and 400 kHz.
- **The host action watches the pads**: its start() run on the STOP that
  closes the far host's hand-made window, the wired-AND decided both ways
  (0x2C over 0x6B, 0x11 over 0x2C); addressing the far board's client it
  wrote six bytes byte-exact and read it twice six bytes back to back.
- **The stuck line** is taken after forty microseconds of quiet wire and
  let go after the falling edges asked: the far host's `unstick()` counted
  four.
- **A counted serve ends on the wire at rest, never on a Stop.** Polled
  as a target transmitter, this block raised no STOPF the loop saw after
  a read (its flush, a PE cycle, falls between the closing NACK and the
  Stop, so whether the Stop alone would raise it is not separated), and a
  count reached by reads alone never ended a serve that waited for a Stop
  - it waited out its deadline. `twi_peer` ends a counted serve once its
  count has moved and both lines have stood high for forty microseconds,
  watched throughout (no two looks a microsecond apart): armed for eight
  bytes and 2000 ms, it is back in command mode 9 ms after the read, the
  eighth byte its own (the far suite's letter d, ten runs).

### The host's cost (`test_x035_i2c`, letter x)

Every shape at 400 kHz against the peer, the SCL period from the slope of
two engined writes (122 cycles, 393 kHz); `wall` from `start()` to the
completion edge, `wire` = (bytes + the address) x 9 x the period, `rest`
= wall - wire; the cycles inside the event vector stamped at its entry
and exit (the vector's own entry and exit, about a hundred cycles with
the hardware prologue, [platform.md](platform.md), outside them; inside
them the suite's storm budget and its switch over three hosts). The
pump in both placements: the default, its body inline in the vector in
the flash, and a twin host with its event pump in SRAM (`InRam`) behind
the same vector:

| tenure | the pump from the flash: rest, entries, cycles an entry | from SRAM: rest, cycles an entry | the engines: rest, entries + DMA |
|---|---|---|---|
| write 1 | 1312, 3, 239 | 1052, 158 | - |
| write 2 | 1282, 3, 251 | - | - |
| write 16 | 1294, 17, 218 | 1010, 146 | 1218, 3 + 0 |
| write 255 | 1276, 256, 205 | 1035, 140 | 1232, 3 + 0 |
| read 1 | 1312, 3, 242 | 1169, 162 | - |
| read 2 | 1398, 3, 261 | 1175, 165 | - |
| read 16 | 1710, 17, 253 | 1272, 157 | 1182, 2 + 1 |
| read 255 | - | - | 1176, 2 + 1 |
| register read 1 + 1 | 2124, 5, 249 | - | - |
| register read 1 + 4 | 2458, 7, 271 | 1947, 166 | 1790, 4 + 1 |
| probe | 1002, 2, 211 | 865, 146 | - |

- **`start()` is 304 to 324 cycles** on the ticker's ruler, the bounded
  waits' first looks included; the listing of a minimal glue (one host,
  `-Os`) has it at 173 instructions, two calls into `delay_us` that run
  only when a wait is not met at once.
- **The pump's entry is 205 cycles a byte from the flash** of a
  1089-cycle byte at 400 kHz - a fifth of the core - for some 340
  instructions in the whole event body, a leaf, the tenure's fields one
  base address: a byte's path is a few dozen instructions from the flash
  behind two wait states at 48 MHz, this core's fetch (the DMA
  controller's own 6 cycles an item from SRAM against 11.5 from the
  flash, [dma.md](dma.md)). The longest entry over every shape is 333
  cycles, the suite's stamps and owner switch included - the family's
  longest handler with every driver in the flash, which the SPI host's
  default hold-off is drawn from ([spi.md](spi.md)).
- **The pump in SRAM, two ways, and only if the program wants the core**:
  the host's `InRam` places the event body (`.fast_text.brio`, 968 bytes
  for a plain host, flattened, a leaf): 140 to 141 cycles an entry over
  the 255-byte write and 157 to 159 over the 16-byte read, the longest
  195 to 207, behind the suite's vector in the flash. A program that places the vector
  (`.fast_text`, [platform.md](platform.md), "A handler in SRAM") carries
  the default host's inline body with it: measured in a variant of the
  suite, 79 and 94 cycles - a fourteenth of the core at 400 kHz - and an
  `InRam` host behind that placed vector the same 79 and 95, the call
  between them costing nothing measurable. So on this path the vector's
  placement is the faster and the cleaner, one attribute where the
  program binds the vector, and the host's `InRam` is for a program that
  keeps its vector in the flash. Either way every shape stays `i2c_ok`
  with the same entries and the wire's own time does not move, since SCL
  stretches behind a late entry and the rate is the bus's: the placement
  buys core time, not the wire.
- **The engines** carry a 255-byte write or read in three event entries
  and none or one DMA interrupt, the write at 1004 thousandths of its
  wire, the core in the vector 627 cycles of 282 320.
- **No vector re-entered after a completion edge**: the event line is the
  tenure's.
- **Against the EVT** (read, not run: no vendor build for this series on
  the desk): WCH's examples are polled - `I2C_CheckEvent` loops of two
  status loads and a compare around every step, the CPU the bus's for the
  whole tenure (282 000 cycles for a 255-byte write at 400 kHz, where this
  pump spends some 47 000 of them in its vector and the engines 648), CCR
  rounded down, and the DMA example's channel 6 at VeryHigh and
  re-initialised whole - DMA_DeInit and DMA_Init - for every block, where
  the engine stores five words.

## Not covered yet

Driver gaps, each with its reason:

- 10-bit addressing on the host side: the resource has the mode and the
  client matches such an address; the host's header-byte sequence (EVT9)
  has no user.
- PEC as part of a tenure, and ENARP: the resource has the bits, no
  Request shape asks for them.
- The general call as a host verb: a Request to address 0x00 sends it
  (the suite does); no separate verb.
- A wake on an address match: this stratum's power chapter has no sleep
  site.
- The collision case (twi_link's `coll`, two clients on one address with
  a third party reading): an instance is a host or a client, and the desk
  has two boards.
- Fast mode Plus: 15.1 names two speeds and the datasheet gives the I2C
  no timing table; the engine has no third speed.

Implemented but not bench-verified, each with what would measure it:

- **The columns other than PC16/PC17**: PA10/PA11, PA13/PA14 and the
  debug port's pair on the other packages compile on every part (`brio
  check ch32x035`) and are refused where the package says; a board of
  each package would run them, and the debug port's pair only with the
  probe given up.
- **The USB_IOEN refusal**: nothing in the stratum sets the bit; the USB
  controller's driver, when it is born, is what would set it and see
  `init()` refuse.
- **The last byte of a read when the wait for its RxNE runs out.** On a
  host with no engines, a read of three or more bytes sets STOP at its
  second BTF, reads byte N-1 and waits for byte N's RxNE with a bound of
  100 000 polls whose answer is discarded: when the bound runs out, DATAR
  is read regardless and the tenure reported `i2c_ok` with what it holds
  (`receive_step()` and `wait_until()` in
  [brio/ch32x035/i2c.hpp](../../brio/ch32x035/i2c.hpp)). Whether a
  measured case reaches it is not recorded. At that BTF byte N is
  already in the shifter, so its RxNE follows the read of N-1 and a
  target stretching SCL cannot hold it back; what would show the bound
  running out is a target that never produces the byte (one that
  releases the bus in the middle of the read), or a review of what the
  code does with the wait's result.
