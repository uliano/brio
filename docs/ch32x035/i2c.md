# I2C (CH32X035)

This document is for a programmer who needs an I2C bus: a sensor or a
memory read and written from an active object (a component of a brio
program with its own event queue, served one event at a time by the
kernel), or this chip as a target another controller addresses. It
covers the one I2C of RM ch. 15, host and client, 7- and 10-bit
addresses, dual addressing and the general call, two speeds with fast
mode's two duty shapes, clock stretching, PEC, two DMA requests; and
the host `I2cHost`, which the bus arbiter of util/i2c_bus.hpp drives
under the cooperative kernel and the preemptive one: an active object
(the requester) posts a request, the arbiter runs it as one bus tenure (one
START to its STOP, the bus held for one request) and replies with an event
when the wire is done ([design/i2c-bus.md](../design/i2c-bus.md)).

The smallest use, a register read from a sensor at address 0x48:

```cpp
using Bus = brio::I2cHost<1>;   // the part's first column: PC16 SCL, PC17 SDA on the F8
using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;
using PullUps = brio::Pin<'B', 3>;   // on the F8 the resistors hang from this pad, never from the supply

BRIO_CH32_VECTOR(i2c1_ev_handler) {
    if (Bus::isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
BRIO_CH32_VECTOR(i2c1_er_handler) {
    if (Bus::error_isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}

// In main(), after the clock task's init() and before the kernel runs:
PullUps::output(true);
if (!Bus::init(clock)) {         // both speeds solved at 48 MHz
    // an HCLK under 4 MHz, a column the probe or USB owns, a die without the I2C: no bus
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
[design/kernel.md](../design/kernel.md)). `Bus` is the host over I2C1 on the
part's default column; `I2c` is the arbiter, with room for four pending
requests (the next is answered `i2c_rejected`, and past ten outstanding the
post itself is dropped with no reply, util/bus_master.hpp), `BusPassThrough`
its failure policy (no retry: every outcome goes to the requester as it
came), and a 20 ms timeout, the answer to a client (a device on the wire)
that holds the clock. That timeout must sit above the longest tenure the
program asks for: at nine SCL periods a byte, 20 ms fits some 200 bytes at
100 kHz, and the longest request the host takes, a write-then-read of 255 +
255 bytes, is 46 ms of wire at 100 kHz. The two `BRIO_CH32_VECTOR` bindings
are the peripheral's two interrupt handlers, `extern "C"` functions as in C
under the crt's names ([platform.md](platform.md)), each posting the
completion to the arbiter when the host says the tenure ended. The request
lends its two buffers (`brio::lend`) until the reply arrives, so `reg` and
`value` stay untouched and in scope from the post to the `I2cDone` event
`Sensor` receives, whose one field, `status`, says what the wire did.

What decides correctness even there:

- **On the CH32X035F8, resistors to the supply on the default column send
  every power-up into WCH's boot loader.** PC17 is SDA there, and PC17 high
  at power-up selects the loader (the evaluation board's reference V1.3, section
  5: the chip "powers up and detects USB_P (PC17) high level to enter
  BOOT"). The F8 has no other column off the debug port, so its pull-ups
  hang from a pad the program drives high once it runs, as above: the
  reference suite uses PB3 with 1.5 kOhm a line, and that pad floats from
  every reset. On a part whose column is not PC16/PC17 the resistors go to
  the supply as usual and the two `PullUps` lines drop out (the CH32X035R8,
  C8 and the CH32X033F8 default to PA10/PA11; PB3 is not bonded on every
  part, and the compiler says so).
- The bus needs resistors of its own on any column: a pad under the I2C's
  alternate function is open drain, and the chip's USB pull-up modes do not
  act under it (below).
- The clock task first, because `init()` solves the bus timing from the
  rate the clock type claims.
- Both vectors bound: an unbound one lands in the crt's spin, the event
  vector at the first START and the error vector at the first error.
- `init()`'s answer read, because on this series the peripheral may be absent
  from a die (the lot note, the first fact below). `init()` answers false
  for an HCLK under 4 MHz, a column the probe or the USB transceiver owns,
  or a peripheral that does not hold its timing. A program that goes on after a
  refused clock or column has every request answered `i2c_rejected`; on a
  die without the peripheral no vector ever runs, so each request waits for the
  per-bus timeout and is answered `i2c_timeout` (read from the code: the
  bench die has the peripheral).
- `start()`, in the arbiter's dispatch, waits: a bound of 5 ms for the last
  tenure's STOP, then a bound of 100 us for BUSY, both timed on the STK (the
  core's system counter, RM 7.5.5, which the ticker runs). Each ends a few
  microseconds past its name, later if a handler holds the core; before the ticker
  runs each is a count of polls, 30 ms and 1 ms ("The two bounds of `start()`",
  below). And on the pump (the event vector moving a request's data) a read of
  three bytes or more waits inside the event vector for its last byte, a bound of
  100 000 polls no correct program spends, then reports `i2c_ok` whatever DATAR
  holds ("Not covered yet").
- No brio program has a bus with a second controller. On one, a request into that
  controller's tenure under a clock of 50 or 20 kHz mostly waits for the per-bus timeout
  ("A request into another controller's tenure", below).

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
all) against a CH32V006K8U6 running its `twi_peer`.

## What the silicon does

- **The peripheral may not be there.** The datasheet withdraws the I2C - and
  the ADC's channels 3, 7, 11 and 15 - from every lot "with a 0 in the
  penultimate 5 digits of the lot number" (table 2-1's note 3, 1.4.16's
  note), and no register names a die's lot. The driver asks the peripheral:
  `I2c<1>::present()` writes CKCFGR with PE clear and reads it back, and
  both `init()`s refuse one that does not hold what they wrote. On the bench
  die (its marking unreadable) it is there (measured, below).
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
  the series runs at HCLK, whose ceiling is 48 MHz. So only the window's
  floor is ever reached: an HCLK of 3 MHz or less, the bottom of the HPRE
  ladder, has no legal timing.
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
  UDM_PUE give PC17 and PC16 a weak, a 10 kOhm or a 1.5 kOhm pull-up, the
  GPIO configuration deciding whether it acts (8.3.2.4).
  Measured with the bus's own pull-ups off: every mode, both output-data
  levels, under the alternate function - both lines driven low and
  released stay low; the 1.5 kOhm and 10 kOhm modes pull both pads up as
  pulled inputs. An I2C on PC16/PC17 wants resistors of its own.
- **The DMA requests are channels 6 (transmit) and 7 (receive)**, figure
  9-1 - ORs the I2C shares with USART2's receive and transmit - and
  CTLR2.LAST makes the controller NACK the last byte a receive DMA block
  takes (15.9.2). The chapter has no DMA section: CTLR2's DMAEN and LAST
  are all it says, with no length named.
- **A START into a held SDA goes out, and the address loses.** With the peer
  holding SDA low, a START set by a peripheral that saw the line taken raised
  SB 40 us later at 100 kHz, MSL and BUSY up, and the address written then
  lost arbitration 16 us on, at its first one bit (measured): the requester
  gets `i2c_arb_lost`, inside any per-bus timeout. A peripheral reset while
  the line was already held, by `init()` or by `recover()`'s SWRST, has not
  seen it taken, and the same START stays pending: no SB and no ARLO in 50 ms,
  START standing in CTLR1, BUSY up - the case the per-bus timeout answers
  (measured).
- **The host's own STOP stands in CTLR1 for about a bit** after the
  completion edge - 460 to 468 cycles at 100 kHz, 108 to 116 at 400 kHz,
  at 48 MHz (measured) - and no flag marks it leaving: `start()` waits for
  it, bounded, before anything writes CTLR1. A read-modify-write of CTLR1
  under it (STOP written back as it stood, ACK set) left no trace: STOP
  cleared on its own a millisecond later and the next tenure ran
  (measured). The host keeps the F1 lineage's rule all the same, no CTLR1
  write while STOP stands, and the wait costs that bit.
- **The client half raises STOPF on a STOP it was not addressed in**,
  ACK clear or set (measured: a START and a STOP by hand, no address
  byte, STAR1 0x10), against 15.9.6's "after an acknowledge (if ACK=1)":
  `isr()` clears a STOPF it meets inside a tenure once no START or STOP
  stands, and `unstick()` clears the one its own STOP leaves.
- **A target stretches one byte late** (measured as the client): a byte
  left in DATAR lets the NEXT one into the shifter, and only then is SCL
  held - two milliseconds on, STAR1 reads RxNE and BTF both up. The
  CH32V006's target does the same: a commanded 2 ms hold before every
  byte of an 8-byte write puts seven holds on the wire - the address's and
  the first six bytes' - and the last two fall after the tenure.

### The offer, item by item

What the chapter gives a controller's transfer, and what the host takes. A
data half is the written or the read part of a tenure:

| offer (section) | taken? | why |
|---|---|---|
| the event machine on two vectors (15.3, 15.7) | yes | the silicon's: no byte counter, no FIFO, so the pump (the event vector moving a request's data) is one entry a data byte past the first two and the frame a few entries more (SB, ADDR, the end) |
| ITBUFEN, the buffer line, as a switch inside a tenure (15.9.2) | yes | TxE and RxNE reach the vector only while the pump needs them, BTF carries the ends |
| ITEVTEN, the event line, as a switch between tenures (15.9.2) | yes | raised by `start()`, dropped at the completion with the buffer line and the DMA requests in one CTLR2 store: the BTF a STOP leaves standing and the client half's STOPF enter no vector between tenures (measured: no entry after a completion edge) |
| EVT8_1 and EVT8 in one entry (figure 15-3) | yes | the entry that clears ADDR loads the data register and the shifter - a one- or two-byte write takes three entries (SB, ADDR, BTF), a register index before a read none of its own (measured) |
| the end of a write at BTF, EVT8_2 (figure 15-3) | yes, for a plain write | the last byte's acknowledge is known there |
| START set while the last written byte shifts (15.9.1) | yes, for a write-then-read | it goes out as the byte ends, with no BTF entry of its own. No delivery here depends on it: the BTF order's one loss was measured with a target polled by the core that also serves the host (a target on its own vector or on another board kept the byte, [../ch32vx03/i2c.md](../ch32vx03/i2c.md)), which this part's one I2C cannot be, and the CH32V006's polled target took every byte in this order (measured) |
| START cleared by the user code, a requested start withdrawn (15.9.1) | declined | behind a refused last byte the controller generates the repeated START all the same and the withdrawal races it (lost on the CH32V006 and the CH32V203); the refusal is closed from the Sr's SB with a void write (the address with the write bit and STOP behind it) instead |
| POS and ACK for the two-byte and the N-byte receive (15.3, 15.9.1) | yes | the chapter's procedures by count |
| DMAEN with LAST (15.9.2), channels 6 and 7 (figure 9-1) | yes, when the request names `I2cEngine::dma` | every data half at any length (below the table) |
| SWRST (15.9.1) | yes | `recover()`, and `start()` for a BUSY standing over an idle wire, the chapter's own case and its condition, "the pins of the I2C bus are released and the bus is idle": BUSY up through the whole of `bus_free_us` and no read finding a line low ("A request into another controller's tenure") |
| PEC (15.8), 10-bit addresses as a host (15.3), ENARP | declined | no Request shape asks for them ("Not covered yet") |
| the USB pads' pull modes (8.3.2.4) | declined | they do not act under the alternate function (measured) |

Under DMAEN a read of two or more runs under LAST, and a read of one has
its NACK set in the SB entry, before the address, and its STOP in the ADDR
entry (EVT6_1, the channel moving the byte). Each DMA block's completion
interrupt is taken. The transmit DMA block's hands its half back to the event
vector with DMAEN down - never with ITBUFEN, since both answer the same TxE
- so BTF ends a plain write and the TxE of the last byte requests a
write-then-read's repeated START.

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
- `I2cHost<1, pins, TxEngine, RxEngine, Placement>` is the host `I2cBus` (=
  `BusMaster`) drives; its `Request` holds `addr`, `tx` and `tx_len`, `rx` and
  `rx_len`, `reply`, `speed` and `engine`. `init()` is false for an HCLK under
  4 MHz, a column on the debug port's pads while the probe owns them, a column
  on PC16/PC17 while AFIO_CTLR's USB_IOEN gives them to the USB transceiver,
  and a peripheral that does not hold its timing. `isr()` is the event
  vector's body, phase by phase, run through the host's `Placement`
  (util/placement.hpp's `InFlash`, the default, or `InRam`); `error_isr()` the
  error vector's (a NACK is `i2c_nack_addr` or `i2c_nack_data` by the phase it
  landed in, ARLO `i2c_arb_lost`, the rest `i2c_bus_error`); `dma_isr()` both
  channels', INTFR read once.

  Three requests are answered `i2c_rejected` inside `start()`: a speed the
  clock cannot produce, an address outside 0x08..0x77 (the general call and
  the 10-bit prefixes among them,
  [../design/i2c-bus.md](../design/i2c-bus.md)), and `I2cEngine::dma` on a host
  with no engines bound (`has_dma` false). An address-phase entry of the event
  vector that finds ADD10, SB or BTF where it waits for ADDR ends the tenure
  `i2c_bus_error` instead of re-entering for ever. `start()` waits, bounded,
  for the last tenure's STOP to leave CTLR1 (5 ms; a clock held past it is answered
  there, `start()` true with `i2c_timeout` in `status()`, nothing written, with or
  without a per-bus timeout) and for BUSY (100 us), then - BUSY up for all of it and no
  read finding a line low - takes SWRST and counts it (`unwedges()`). With no per-bus
  timeout nothing answers a START held under a foreign tenure (read from the code: no
  letter here stages it without one).

  `unstick()` clocks a stuck client free by hand - a line pulled low as a
  push-pull output over a clear bit and released as a floating input, never
  driven high - and `recover()` puts the peripheral back (SWRST, the timing
  rewritten). `release()` stops the engines, clears PE and closes the gate with
  no reset, and `init()` starts from the peripheral's reset line
  ([../design/dma.md](../design/dma.md)'s release contract). `busy()` is the
  host's phase, for a sleep site.

  The engine parameters (engines: the DMA channels bound to the host) are
  `DmaTxEngine<6>` and `DmaRxEngine<7>`, both or neither, at `high` (the host
  stretches SCL behind a starved receive channel and loses nothing, 15.6; a
  transmit channel starved past a byte's end ends the tenure). The program
  calls no init of the DMA controller: the host's `init()` arms both engines,
  and arming opens the controller's gate. Which mechanism moves a request's
  data is the request's `engine` field (util/i2c_bus.hpp's `I2cEngine`):
  `interrupt`, the default, moves every byte from the event vector, the first
  two of a write in the ADDR entry, and touches no channel; `dma` puts every
  data half on the engines at any length. The launch of either engine cannot
  refuse a half of 1 to 255 bytes. The host asks nothing about its channels at
  run time: the compiler refuses a second owner of a channel (in the verb that
  arms it, [../design/dma.md](../design/dma.md), "The check"), and the host runs
  one tenure at a time. An error puts away only the DMA block the tenure has in
  flight. A channel's transfer error, or a transmit count standing at BTF, ends
  the tenure `i2c_dma_fault`, this host's own status beyond the wire codes of
  util/i2c_bus.hpp.
- `I2cClient<1, pins>`: `init(clock, addresses, I2cClientOptions)` - a
  polled client with `.interrupts = false` -, the polled surface
  (`addressed()`, `answer_address()` returning the direction,
  `take()`/`give()`, `stop_seen()`, `host_nacked()`), `flush()` (the PE
  cycle that drops a byte the host never clocked), and two ISR bodies,
  `service()` and `error_service()`, each reporting one `I2cClientEvent`.

## How to use it

The register read at the top of this page is the plain use: a request
that names no engine. A request that asks for the DMA engines, an excerpt
over that setup with the host given its two engines, their vectors bound
and `page` a static buffer of 34 bytes (the program also includes
ch32x035/dma.hpp, which names the two engine types):

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
a program that would sleep, is "Which engine to name" below. The
event pump in SRAM is either the host's `InRam`, its last parameter, or
the vector placed by the program with the host as it is (Bench findings,
"The host's cost"). The vector takes its section on a declaration in
front of the same binding:

```cpp
extern "C" [[gnu::section(".fast_text")]] void i2c1_ev_handler();
BRIO_CH32_VECTOR(i2c1_ev_handler) {
    if (Bus::isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
```

## Bench findings

The reference suite is `test_x035_i2c` on the CH32X035F8U6 at 48 MHz
against a CH32V006K8U6 running `twi_peer` on its remap code 011 (PB5,
PB6), the bus pulled up by a 1.5 kOhm resistor from each line to this
board's PB3, which the suite drives high from its boot. The suite is two images, `test_x035_i2c-1` (the
single-host wire letters) and `-2` (the kernel, the two hosts, the dead
channel, the stuck BUSY, the vectors' client, the cost, the bounds, the live
tenure): `z` is 64 and 61 verdicts, green.

- **The lot probe** answered present: FREQ, CKCFGR and OADDR1 hold what
  they are given, PE sets, and a START on the idle bus raises SB with MSL
  and BUSY (after 23 polls of STAR1), a STOP behind it giving the bus
  back. The reset values are table 15-1's (all zero).
- **The wire's rise**: each line released from low reads high at the
  first read after the release, its count starting after the release's
  return (44 and 45 cycles, no poll): a fraction of a 400 kHz bit.
- **The scan, the shapes, the vocabulary**: 112 addresses probed in 14 ms,
  the peer's command address the one answering and 111 `i2c_nack_addr`;
  a write, a read and a write-then-read against the peer, its count and
  the repeated START counted at its end; reads of one to four bytes, the
  four receive procedures, each closed at the right byte; `i2c_nack_addr`
  from a deaf peer and `i2c_nack_data` at a commanded byte.
- **The rate asked, on the wire** (a tenure timed from `start()` to its
  completion, nine SCL periods a byte with the address's; the bracket:
  never above the rate asked by more than 2 %, within a quarter below
  it): an 8-byte write on the pump at about 94 kHz for 100 and 332 kHz for
  400, on the engines a 16-byte write and read at 96 kHz and a 16-byte
  read at 359 kHz. Past the chapter, CKCFGR's fast-mode CCR at 16 (1 MHz
  asked) carried an 8-byte write and read byte-exact at about 690 kHz -
  measured, and no speed of the host's: 15.1 names two.
- **The engines** (letter g, every request naming `I2cEngine::dma`): a
  16-byte write and read, a write-then-read, a two-byte read under LAST,
  a one-byte read and a 16-byte read at 400 kHz byte-exact, the peer
  accounting all 59 bytes, no transfer fault; one completion interrupt a
  DMA block, the write's handing its half back to the event vector; a write
  three event entries (SB, ADDR, BTF), a read two.
- **The mechanism a request names** (letter n, ten runs and three of its
  image's `z`, every verdict green): `dma` at one byte - a one-byte
  write, a one-byte read and a 1 + 1, every half on the engines, one
  completion a DMA block, no fault; a read of one byte rides the receive
  DMA block with its NACK prepared before the address and its STOP set in
  the ADDR entry, LAST clear. `interrupt` at sixteen bytes on the host
  with engines bound: no DMA entry, both channels' configuration word and
  count as `init()` left them, 17 event entries each way. `dma` on the host with no engines: `i2c_rejected`
  inside `start()`, the vector never entered, nothing on the wire. Under
  `dma` an address nobody answers is `i2c_nack_addr` writing and reading,
  a NACK on the third of six bytes written `i2c_nack_data` with channel 6
  put away and one fault counted, and the next write whole; letter `r`
  carries the repeated START and its refused last byte under `dma`, letter
  `j` a lost arbitration (`i2c_arb_lost`, no transmit DMA block started or
  thrown away, eighteen runs).
- **The kernel**: four tenures through `I2cBus` answered in order with
  the NACK in its place, a full queue's rejection, the sleep vote for an idle
  bus and against a busy one; a tenure into an SDA the peer holds answered
  `i2c_arb_lost` in under a millisecond and a clock held 45 ms answered
  `i2c_timeout` at the arbiter's 20 ms, the same arbiter carrying the next
  tenure to `i2c_ok` after each. A speed the clock cannot make is
  `i2c_rejected` inside `start()`, the vector never entered.
- **Arbitration in both directions**: two controllers' STARTs released by
  one STOP - this host aimed at 0x6B lost to the peer's 0x2C and reported
  `i2c_arb_lost`; aimed at 0x11 it kept the bus (`i2c_nack_addr`, nobody
  there) and the peer reported the loss. The two STARTs are set microseconds
  apart, so a round can miss the race: one set after the other has reached the
  wire waits for that tenure's STOP, and the wire carries two tenures, each a
  NACK on its address. The letter tells a missed round by this board's time to
  its answer and retries it, up to five attempts; a race that came out wrong
  still fails. Over 18 runs 11 of 65 rounds missed and were repeated. **As the peer's client**,
  polled,
  this board answered its own address in a write tenure and took six bytes
  byte-exact, ending on the STOP; read twice six bytes back to back by the
  peer's controller (twi_link's `host_read`), it served each run
  byte-exact as a transmitter, one byte standing in DATAR at each closing
  NACK, and `flush()` dropped both - the second read opened on its own
  first byte.
- **The stuck bus**: the peer holding SDA until four falling edges,
  `unstick()` clocked four pulses, its hand-made START and STOP left no
  STOPF and no entry of the event vector, and the channel came back.
- **The repeated START** of a write-then-read requested on the last
  byte's TxE: one to four bytes written then four read, at 100 and 400
  kHz, under `interrupt` and under `dma` - the CH32V006, a polled CH32 target, took every
  written byte (its count and sum) and every read byte was its pattern;
  the pump's tenures took 7, 8, 9 and 10 event entries; under
  `I2cEngine::dma`, both halves on the engines, five event entries and
  one completion of each DMA block, whatever the length written. The last
  written byte refused by the peer's NACK: `i2c_nack_data`, CTLR1 back to PE
  alone and MSL clear within a millisecond, and the next tenure - a probe
  of the same address - completed on the wire; it is `i2c_nack_addr`
  because a CH32 target keeps its acknowledge down after a refused byte
  until its window closes (the peer's own header).
- **The status a polling thread reads is the vectors'.** `status()` is
  a load of a volatile field the vectors write, so a polled caller that
  has `start()` inlined reads their answer. The arbiter's completion carries
  the status from the vector.
- **The reserved addresses refused by the host** (letter `d`): `start()`
  returned true and left `i2c_rejected` on a write to 0x7A and one to the
  general call (its true is the synchronous completion), and
  the CH32V006's client, answering the general call, took only the write
  to its own address that follows - one match, two bytes (ten runs of
  the letter).
- **An address of 0x78 to 0x7B never reaches the wire**: `start()`
  refuses it, as letter `d` judges. So a correct program cannot reach what
  such a write's address byte, 1111 0xx0, would be to this machine: a 10-bit
  header, which `unserved()` ends `i2c_bus_error` in the address phase.
- **Fast mode at DUTY 16/9** (letter f): CKCFGR 0xC005 - F/S, DUTY and
  CCR 5, 48 MHz / (25 x 400 kHz) = 4.8 rounded up so the bus never runs
  faster than asked - the chooser's 384 kHz; an 8-byte write at about
  321 kHz with the software inside it (332 at DUTY 2's 400: the same
  0.96), a write and a read byte-exact.
- **A BUSY left standing over an idle wire** (letter u): a START made by
  hand, SDA let go while SCL was held low, then SCL - no STOP condition
  ever on the wire - left BUSY up with both lines high; `start()`'s
  100 us wait ran out with no read finding a line low, it took SWRST
  (`unwedges()` one more) and the tenure behind it completed `i2c_ok` at 100
  and at 400 kHz, BUSY down after. A
  STOP made with the peripheral's gate closed leaves nothing: BUSY read 0 at
  the reopening - the flag follows the wire. No hand on the wire, the case is a
  client holding the clock past the per-bus timeout (`recover()`'s SWRST under the
  low SCL, the client then letting go with no STOP) or a board restarted on a live
  bus with its pull-ups. Those two are read from the code here: letter h's tenure
  behind such a timeout is `i2c_ok`, and it prints no reset count.
- **The dead channel** (letter p): a timer's compare request raised by a
  match with no channel enabled, the timer's gate then closed with no
  reset - TIM1's channel 3 on DMA channel 6, TIM2's channel 2 on channel
  7 (figure 9-1) - is held: the channel takes one stray item at its next
  enable and serves nothing after it. (The software compare event, SWEVGR's
  CCxG, left it nothing: the match is the request.) A write
  of 32 on the engines over the dead channel 6 ends `i2c_dma_fault` - BTF
  with the transmit count standing, the far end holding one byte - where
  a host trusting BTF alone would report `i2c_ok`; a read of 16 over
  the dead channel 7 never completes, its event vector entered twice (SB,
  ADDR) and never stormed - the event line is down for the DMA block - so
  only a bound outside the host answers it: under the arbiter the per-bus
  timeout and its `recover()` (the letter's own wait stands in for it), and with
  no per-bus timeout nothing. Each completes intact once the timer is reset.
- **A block above the host's channels** (letter y, twelve runs of eight rows a shape):
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
  read stalled under DMAEN, RxNE and BTF standing, gave channel 7's next
  enable four items while the peripheral stayed live, and none once its
  gate was closed - with PE up or cleared - or after `release()`, which
  closes the gate. So `init()`'s reset pulse finds nothing of this
  peripheral's to clear after these ways of letting go. The pulse stays,
  because USART3's request on this die does outlive its gate
  ([dma.md](dma.md), "A released requester").
- **The client from its vectors** (letter v): letter j's two cases with
  the interrupt enables up, `service()` and `error_service()` the bodies
  of i2c1_ev and i2c1_er - six bytes written to it byte-exact (eight
  event entries: ADDR, six RxNE, STOPF), two reads of six served as a
  transmitter (each closing NACK on the error vector, `flush()` there
  dropping the byte asked beyond it, the second read opening on its own
  first byte), seventeen event entries and two error ones, no entry
  finding nothing; the event body 125 or 126 cycles on average at 48 MHz,
  137 at most, stamped inside the vector.

### This part as another board's peer: `twi_peer`

`twi_peer` (ch32x035/src/apps/twi_peer.cpp) is the CH32X035's peer on the
twi_link protocol, on column 010 - SCL PC16, SDA PC17, wired to the
CH32V006's PB5 and PB6. It drives PB3 high, the bus's pull-ups, from the
supply check at boot (the voltage detector at 3.0 V; below it PB3 and the pads
stay floating) to the end of its run. Its target is polled
(`I2cClientOptions`) and drops the byte it was asked beyond a closing NACK
with `flush()`. Through one `z` of each of the three images of the CH32V006's
`test_ch32_i2c` on that part, no command was answered with a nak and no vector
was entered while it was polled. It showed this peripheral's habits as a
target, each measured from the far end:

- **A target transmitter is asked for one byte more than the controller
  takes**: at the closing NACK of a read a byte stands in DATAR; `flush()`
  drops it and the tally gives it back - 233 such drops in those three `z`,
  two in the serve (a commanded spell as a target, reported afterwards) of two
  reads the far suite counts, no byte of a later read out of its place.
- **The stretch is one byte late**: a 2 ms hold before every byte of an
  8-byte write lengthened the far host's tenure by seven holds.
- **A refusal stays**: after a byte it refused, the far host's void write
  and its next probe of the same address inside the action's window met the
  same NACK (`i2c_nack_addr`, sixteen of sixteen).
- **The last written byte before a repeated START is taken in either
  order**: the far host requests the Sr on that byte's TxE, and WCH's EVT
  library (the vendor's example package), built and run on the CH32V006,
  requests it at BTF - this target took every written byte both ways, at 100
  and 400 kHz.
- **A counted serve ends on the wire at rest, never on a Stop.** Polled as a
  target transmitter, this peripheral raised no STOPF the loop saw after a
  read (its flush, a PE cycle, falls between the closing NACK and the Stop, so
  whether the Stop alone would raise it is not separated), and a count reached
  by reads alone never ended a serve that waited for a Stop. `twi_peer` ends a
  counted serve once its count has moved and both lines have stood high for
  forty microseconds: armed for eight bytes and 2000 ms, it is back in command
  mode 9 ms after the read, the eighth byte its own (the far suite's letter d,
  ten runs).

### The two bounds of `start()` against their names (`test_x035_i2c` letter t)

Eighteen runs, all green. The time is the ticker's cycle count around the call, 20.8
ns a count, a read of it some 60 cycles inside each figure, so a figure reads high by
that and never low. Each verdict takes the name as its floor and, as its ceiling, the
name plus the free bus's own figure, one poll, a tick's entry and the ruler. A floor on
a figure that holds those cycles would pass a wait up to 7 us short; what keeps the wait
from being short is the sum read below.

| wait | named | measured at 48 MHz | how it was staged |
|---|---|---|---|
| free bus | 0 | 304 to 320 cycles, 7 us | nothing standing: the two first looks |
| `bus_free_us` | 100 us | 5276 to 5344 cycles, 109 to 111 us | SCL held low from this board's pad |
| `bus_free_us` with the reset | 100 us | 5468 to 5544 cycles, 113 to 115 us | BUSY over an idle wire |
| `stop_drain_us` | 5 ms | 240 332 to 240 392 cycles, 5007 or 5008 us | a client holding SCL 45 ms behind a write of two, both speeds; `start()` true with `i2c_timeout`, no START, no vector entered |

The wait reads what the STK's low half moved (RM 7.5.5), each poll adding the
difference with one period folded in where the counter restarts from zero, so a bound
spans ticks; it consults no handler, so it holds with interrupts masked, and a gap of
a whole period between two polls is lost from the sum and ends it later, never
earlier. On a free bus the two first looks are one load and a branch each, with no
counter read and no call; the poll is out of line and cold.

**Before the ticker runs** nothing moves the counter, so the wait also spends a
budget, the time asked at eight cycles a poll, which a running counter always beats.
With STE cleared by hand, timed on TIM2 at 1 us a count: `bus_free_us` ends after its
600 polls in 962 or 963 us with SCL low and 1016 or 1017 us over an idle wire, a poll 76 or 77 and 81 cycles, and `stop_drain_us` after 30 000 polls in 30.0 ms, a poll
48 cycles. A program that calls `start()` before `Ticker::init()` has bounds that long.

**Why 5 ms and 100 us.** `stop_drain_us` holds the dispatch of every active object
under the arbiter's kernel and is there for a client digesting a write: 5 ms is a
fifth of SMBus's 25 ms clock-low limit and stands under a 20 ms per-bus timeout with
room for the tenure behind it. `bus_free_us` is there for this host's own STOP, a bit
at 100 kHz, and is the watch's window (below): twice SMBus's 50 us ceiling on a clock's
high phase, so any such clock shows a low line inside it.

### The answer to a STOP held past its bound (`test_x035_i2c` letter t)

A client that stretches after a write's last acknowledge holds the host's STOP in
CTLR1, and a request behind it finds that STOP standing. When `stop_drain_us` runs out
`start()` answers that request itself: true, `i2c_timeout` in `status()`, CTLR1
untouched, no vector. The arbiter replies and takes its next request, and no
`recover()` runs under the hold, so the STOP stays requested and goes out when the
client lets go. Nothing of the request reached the wire, so it can be posted again.

| | post to reply | the next request, the hold over |
|---|---|---|
| a 20 ms per-bus timeout, the client holding 15 ms, 100 and 400 kHz | 5043 or 5044 us | on the wire, 171 or 172 us from its post at 100 kHz, 92 to 99 at 400 |
| no per-bus timeout, the client holding 45 ms, 100 and 400 kHz | 5043 to 5045 us | the same |

Eighteen runs, 12 rounds a run, all sound: CTLR1's STOP up, START down and BUSY up at the
reply, no reset, no vector, and behind the hold STOP and BUSY down with nothing done.
The ceiling beside the letter is `stop_drain_us`, what `start()` costs around the wait,
a poll, a tick's entry, the ruler and the kernel's own turn, read in the same round as
the post-to-reply of a request refused with nothing on the wire (41 to 43 us).

What a program sees of it: a client that stretches past 5 ms after a write loses the
request behind it to `i2c_timeout` however long the per-bus timeout is, and that
request's 5 ms are the dispatch's. A requester that posts again at once meets the same
bound until the client lets go, each try 5 ms of every object's time. Requests waiting
behind the tenure when its reply comes are each answered in that one dispatch, 5 ms
apiece: two behind a 15 ms hold, every reply in 25.4 ms, and a queue of four holds
the kernel 20 ms. A program with such a client spaces its requests by the client's
own time.

### A request into another controller's tenure (`test_x035_i2c` letter w)

`start()` takes SWRST only when BUSY stood for the whole of `bus_free_us` and no read
in that time found a line low: 15.9.1's "Make sure the pins of the I2C bus are released
and the bus is idle before the reset". One read would not do: both lines stand high in
SCL's high phase on a 1 (the last column below). The assumption this rests on: **a
foreign controller's clock whose high phase is longer than the bound, a clock under
5 kHz, is taken for an idle wire.** SMBus's ceiling on that phase is 50 us.

A poll of the watch is 76 to 81 cycles, 1.7 us at 48 MHz, read on a timer with the STK stopped: some 60 reads
in the bound. A low phase shorter than that can fall between two of them, and a low
phase is what the watch looks for. At 100 kHz the low phase is 5 us, at 400 kHz in DUTY 2
1.67 us, 80 cycles, about one poll, 40 of them in the bound; a poll is not a multiple of
that clock's period, so successive reads do not stay in its high phase. A tick's entry,
220 cycles, can hide two low phases at 400 kHz. A clock past 400 kHz is not staged ("Not
covered yet").

Letter w stages it: this board's probe of nobody at 100 kHz posted into a tenure of the
peer's controller longer than the bound, the moment swept by 7 us, 64 rounds a kind, eighteen runs. At 100 and 400 kHz the peer writes 24 bytes of
0x55 that this board's own client half acknowledges from the event vector, there being
no third chip on the wire. Host and client share CTLR1's ACK bit, and `start()`'s store
clears it for the address phase, so the client half refuses the next byte and the peer's tenure ends on the byte behind the request either way. At
20 kHz it probes an address nobody answers, nine clocks of 50 us, this peripheral a
bystander. The bound ran out in every round, and **no reset was taken**.

| the peer's tenure | requests | resets taken | both lines high by arithmetic and read | this request | the peer's tenure |
|---|---|---|---|---|---|
| 24 bytes at 100 kHz | 1152 | 0 | half of the bit x 4 of 9 bits, 22 %; the thread's reads 16.2 % | `i2c_nack_addr` in 1151, `i2c_bus_error` in 1 | `i2c_nack_data` in 1151, `i2c_bus_error` in 1 |
| 24 bytes at 400 kHz | 1152 | 0 | a third of the bit x 4 of 9 bits, 15 %; the thread reads too seldom here to say | `i2c_nack_addr` in 1147, `i2c_bus_error` in 5 | `i2c_nack_data` in 1147, `i2c_bus_error` in 5 |
| probe at 20 kHz | 1152 | 0 | half of the bit x 4 of 9 bits, 22 %; the thread's reads 24.7 % | unanswered in 10 ms in 1049, `i2c_nack_addr` in 103 | `i2c_nack_addr` in 1152 |

At 20 kHz, the only rate where the peer's tenure can tell, this peripheral a bystander,
that tenure is not ended in `i2c_bus_error`, which the letter judges. The rows at 100
and 400 kHz cannot show that: the acknowledge is this peripheral's.

**A START stored into a tenure mostly stays standing under a clock of 50 or 20 kHz.**
Raw, the vectors dark, the store swept inside the peer's probe of nobody with this
peripheral a bystander at 100 kHz, 1152 stores at each rate: at 100 kHz SB rose in all
of them; at 50 kHz in 95 and at 20 kHz in 226. In the others BUSY stood over both lines
high with no SB at the end of the 3 ms the letter watches, the probe, 180 or 450 us
long, over by then. Through `start()` the unanswered rounds above read START in CTLR1
as well, and what answers such a request is the per-bus timeout. Why the rate decides
is not established.

### The host's cost (`test_x035_i2c`, letter x)

Every shape at 400 kHz against the peer, the SCL period from the slope of
two engined writes (121 or 122 cycles, 393 to 396 kHz, the rest below taken at
122 throughout); `wall` from `start()` to the
completion edge, `wire` = (bytes + the address) x 9 x the period, `rest`
= wall - wire; the cycles inside the event vector stamped at its entry
and exit (the vector's own entry and exit, 70 cycles with the hardware
prologue, [platform.md](platform.md), outside them; inside
them the suite's bound on a re-entering vector and its switch over three
hosts). The
pump in both placements: the default, its body inline in the vector in
the flash, and a twin host with its event pump in SRAM (`InRam`) behind
the same vector:

| tenure | the pump from the flash: rest, entries, cycles an entry | from SRAM: rest, cycles an entry | the engines: rest, entries + DMA |
|---|---|---|---|
| write 1 | 1308, 3, 227 or 229 | 1073 to 1093, 148 or 150 | 1280 to 1288, 3 + 1 |
| write 2 | 1310 or 1318, 3, 241 | - | 1242 or 1258, 3 + 1 |
| write 16 | 1298 to 1318, 17, 211 or 214 | 1046 or 1048, 136 | 1270 to 1282, 3 + 1 |
| write 255 | 1320 or 1336, 256, 201 | 1029 or 1031, 132 | 1260 to 1276, 3 + 1 |
| read 1 | 1368 or 1388, 3, 246 | 1054 to 1158, 149 or 151 | 1360 or 1372, 2 + 1 |
| read 2 | 1418 to 1430, 3, 257 | 1088 to 1186, 155 | 1342 or 1346, 2 + 1 |
| read 16 | 1758 to 1766, 17, 243 or 244 | 1268 to 1339, 147 or 148 | 1294 or 1302, 2 + 1 |
| read 255 | 1756 to 1812, 256, 233 | - | 1276 to 1288, 2 + 1 |
| register read 1 + 1 | 2144, 5, 248 | - | 2004 to 2012, 5 + 2 |
| register read 1 + 4 | 2518 to 2526, 7, 261 | 1881 to 1954, 157 or 158 | 2022 or 2034, 5 + 2 |
| probe | 1038 or 1042, 2, 205 | 860, 138 | - |

Each figure is the range over three runs of the letter.

- **`start()` is 296 to 312 cycles** on the ticker's ruler for every
  shape on the pump and the engines, the bounded waits' first looks
  included; the engined read of one byte is 372 to 380 and the pumped read of 255 is 360 or 364 in letter `x`,
  without a reason the letter prints.
- **The pump's entry is 201 cycles a byte from the flash between the stamps**
  over the 255-byte write, 271 with its trip, of a 1098-cycle byte at 400 kHz:
  a quarter of the core. The event body is a leaf, the tenure's fields one
  base address, and a byte's path runs from the flash behind two wait states
  at 48 MHz, this core's fetch (the DMA controller's own 6 cycles an item from
  SRAM against 11.6 from the flash, [dma.md](dma.md)). The longest entry over
  every shape is 321 cycles, the suite's stamps and its switch among
  the hosts included, and the family's longest handler with every driver in
  the flash ([platform.md](platform.md) places it among the handlers); main
  context waits that long, as it does for any polled loop.
- **The pump in SRAM, two ways, and only if the program wants the core**:
  the host's `InRam` places the event body (`.fast_text.brio`, 968 bytes
  for a plain host, flattened, a leaf): 132 cycles an entry over
  the 255-byte write and 147 or 148 over the 16-byte read, the longest
  189, behind the suite's vector in the flash. A program that places the vector
  (`.fast_text`, [platform.md](platform.md), "A handler in SRAM") carries
  the default host's inline body with it, one attribute where the program
  binds the vector, with no call between the two memories; `InRam` is for
  a program that keeps its vector in the flash. Under `InRam` every shape
  stays `i2c_ok` with the same entries and the wire's own time does not
  move, since SCL stretches behind a late entry and the rate is the bus's:
  the placement buys core time, not the wire.
- **The engines** carry a 255-byte write in three event entries and a
  read in two, each with one DMA interrupt (its body 151 to 191 cycles
  between the stamps), the write at 1004 to 1012 thousandths of its wire, the
  core in the event vector 579 or 583 cycles of some 282 350. The
  write's DMA entry, its DMA block's completion, is the price of carrying a
  write half before a read: the completion hands the half back to the
  event vector, which ends a plain write at BTF and requests a
  write-then-read's repeated START on the last byte's TxE.
- **No vector re-entered after a completion edge**: the event line is the
  tenure's.
- **Against WCH's EVT library** (read, not run: no vendor build for this
  series has been made): WCH's examples are polled - `I2C_CheckEvent` loops of two
  status loads and a compare around every step, the CPU the bus's for the
  whole tenure (282 000 cycles for a 255-byte write at 400 kHz, where this
  pump spends some 52 000 of them in its vector and the engines some 730),
  CCR
  rounded down, and the DMA example's channel 6 at VeryHigh and
  re-initialised whole - DMA_DeInit and DMA_Init - for every DMA block, where
  the host stores five words.

### Which engine to name

The host chooses nothing: the program names the mechanism of each request.
On a host with engines bound, either name delivers the same bytes, and the
choice costs time or energy. At 400 kHz, read directly, the engines cost the
core fewer cycles for a read from two bytes, a read of one being level (1249 against
1247 to 1255), and for a write from 3 bytes (the letter has no write of three:
write 2's 1229 and the pump's 271 cycles a byte give some 1500 against 1302 to
1318); under that, name the interrupt. **For a write-then-read, the shortest
register read stays cheaper under `interrupt`:** `dma` pays for two DMA blocks, so 1 + 1 is 1887 or 1891 cycles on
the pump against 2137 to 2153. The register read at the top of this page, 1 +
2, has no row of its own: the slope between 1 + 1 and 1 + 4, 244 cycles a byte, puts
it at some 2133, about what the engines cost, and on this series `dma` costs the
sleep (below), so name the interrupt there too. The same slope puts 1 + 3 at some
2377 against the engines' 2137 to 2153, and from 1 + 4 (2621 against 2137 or 2153) the rows
say it: from there name `dma`, if the core has other work to run. At 100 kHz the
letter gives the estimate only.

What each costs the core comes from letter x at both speeds: a tenure's
cycles between the stamps (`start()`, the event entries and the DMA
entries) and the trip of every entry outside them, 70 cycles under the
hardware prologue ([platform.md](platform.md), the interrupt round trip).
The letter computes a first estimate: `b` the pump's cycles a byte, the
slope between its 16- and 255-byte tenures; `F` its fixed part, its 16-byte
tenure less sixteen `b`; `E` the engines' cost of a one-byte tenure, which
does not grow with the length; `N`, the smallest length with E <= F + N x b:

| at | write: b, F, E | N | read: b, F, E | N |
|---|---|---|---|---|
| 400 kHz | 270 or 271, 743 to 811, 1314 or 1318 | 2 or 3, by the run ((1318 - 743) / 271 = 2.1) | 303, 783 to 791, 1247 to 1255 | 2 ((1255 - 783) / 303 = 1.6) |
| 100 kHz | 270 or 271, 755 to 771, 1354 | 3 ((1354 - 755) / 271 = 2.2) | 302 or 303, 783 to 815, 1179 | 2 ((1179 - 783) / 303 = 1.3) |

The estimate counts a byte the same at every length, and a write's first
two bytes cost the pump less: the entry that clears ADDR loads both, so a
second byte costs it no entry. So the short lengths are read directly, at
400 kHz with the trips:

| tenure | pump | engines |
|---|---|---|
| write 1 | 1193 | 1314 or 1318 |
| write 2 | 1229 | 1306 or 1310 |
| write 16 | 5079 or 5131 | 1302 |
| read 1 | 1249 | 1247 to 1255 |
| read 2 | 1281 | 1219 |
| register read 1 + 1 | 1887 or 1891 | 2137 to 2153 |
| register read 1 + 4 | 2621 | 2137 or 2153 |

Against the wire: the pump spends 271 cycles of a 1098-cycle byte at 400
kHz (a quarter of the core) and 270 of a byte of at least 4320 cycles at
100 kHz (CCR 240, twice, nine times: a sixteenth at most); the engines spend none
a byte.

**The energy.** On this series the Sleep stalls the DMA - measured, against
RM 2.3.2, which has the peripherals run in the Sleep ([dma.md](dma.md)) - so
`idle()` does not sleep while a channel works
([../../brio/qingke/platform.hpp](../../brio/qingke/platform.hpp)): a
half on the engines keeps the core awake for its wire time - the receive
DMA block for the whole read, the transmit DMA block until its last byte is in
DATAR - where the pump would sleep between its entries. `dma` buys core
cycles at the price of a sleep.

### The stack under the preemptive kernel (`test_x035_rubato_i2c`)

Measured by `test_x035_rubato_i2c` and its twin `test_x035_rubato_i2c_flash`.
The two are the same source; the scheduler path, the kernel's own code that
runs a preempting object, is in SRAM in the first and in the flash in the
twin ([platform.md](platform.md), "The scheduler's place"). They run
`util/i2c_bus.hpp`'s arbiter (pending depth 4, the per-bus timeout at
20 ms) over the engined `I2cHost<1>` on PC16/PC17 (channels 6 and 7) inside
Rubato, the preemptive kernel, against the same CH32V006K8U6 `twi_peer`.
Each request names its mechanism by its shape: `dma` for a read of two
bytes or more or a write of three or more, `interrupt` otherwise. The suite's
active objects are the pack. Pressure, an object of the pack, is the highest:
TIM2 posts to it every 97, 193 or 389 us and its dispatch spins 300 or 3000
cycles, at 97 us x 3000 64 % of the core. Below it stand a requester, the
arbiter and a second requester; the Console, the lowest, runs the letters, and
so runs only when every object above it is idle.

Every letter runs once with TIM2 silent (the control) and six times under it.
Key `z` (202 verdicts) ran three times on each image with every verdict green,
its vocabulary letter `q` ten more on each, letter `x` eleven times on each
and the letter `u`, the unpaced flood, by its own name ten times on each.
The pack, the command channel through the arbiter and the letters are the
I2C suites' shared body (`avrxt/src/apps/rubato_i2c.hpp`, whose header
describes them).

**Where a preemption can reach this host: before the START, or in
`start()`'s last instructions after it.** `start()` waits for the last
STOP and for BUSY, then stores START last: a dispatch above the arbiter
that lands in it delays the START, a gap between tenures, and every byte
decision after it is the vectors'. Letter `x` counts it on 400 writes of
16 bytes on the engines: `start()` takes 307 to 309 cycles with TIM2 silent
in the default image and 303 or 304 in the twin, `start()` in the flash in both.
That figure includes the suite's own code in front of the host's `start()`,
23 instructions from the flash that copy the Request and set its `engine`, and it
is taken on this suite's own ruler, which the letter does not compare with
`test_x035_i2c`'s ("The host's cost" above).
Under 97 us x 3000 the interrupt landed inside it 149 to 159 times in 400 in
each of eleven runs of the default image, and 225 to 262 in each of eleven of the
twin, against the 26 that ticks independent of the tenures would give. That
excess is not explained: the letter counts the landings and does not time
their phase. MSL stood at such a landing rarely, a preemption after the START
store that outlasted the START's way onto the wire, the tenure then the
vector's from its first bit:

| image | eleven runs of `x` | three runs of `z` |
|---|---|---|
| default | none | none |
| twin | none | one in one of the runs |

**Every shape under `interrupt` and under `dma`, under every pressure as in
the control.** Letter `o`, at 400 kHz against one serve counted to its last
byte: the probe, reads of 1 (the interrupt) and of 2, 3, 8 and 32
(channel 7 under LAST), write-then-reads of 1+1 (both halves the
interrupt), 1+2 and 2+8 (both halves on the engines, the write half
handed back by channel 6's completion for the repeated START), writes of
1 and 2 (the interrupt, both bytes loaded in the ADDR entry) and of 3, 8
and 32 (channel 6, its completion, then BTF) - every one `i2c_ok`, every
read byte-exact, the peer's count, sum and address matches exact. A
shape's post-to-reply under pressure is the control's plus at most one
dispatch and the path behind it: the 32-byte write 807 or 808 us silent
and at most 886 under pressure with the scheduler path in SRAM, 823 and
at most 1160 with it in the flash; the probe 73 or 74 and at most 213, 86
and at most 480. Letter `p`: a read of 16, a 1+4 and a write of 16
byte-exact at 100 and 400 kHz (the write of 16 in 1596 to 1598 and 442 or 443 us
silent with the scheduler path in SRAM, 1611 to 1613 and 456 to 458 with it in the
flash, against the wire's 1530 and 382), and the peer's commanded stretch of 1000 us a byte
priced from below - a read of 8 in 9266 to 9601 us against its nine holds, a
write of 8 in 7562 to 8134 us against its
seven (the peer one byte late: the hold before each byte it takes
overlaps the next byte's shifting in, and the write's last two fall after
its tenure), under the 20 ms timeout.

**The vocabulary under pressure, with this peer's answers.** Letter `q`:
a deaf client - the peer's client parked at another address, 0x5D - answers
the probe and a four-byte write `i2c_nack_addr`, and the probe of 0x5D
`i2c_ok`; a write to 0x7A and one to the general call answer `i2c_rejected`,
the host's own, the peer, answering the general call, matching only the probe
that follows; the third byte of a six-byte write on the engines refused by the
peer's NACK answers `i2c_nack_data`; the last written byte of a 2+4 refused answers `i2c_nack_data`, the controller let
go by its void write and no read half begun. After each refusal the
CH32V006 keeps its acknowledge down until its action's window closes, so
the probe that follows inside the window is answered `i2c_nack_addr` and
the peer counts one address match for each refused tenure, the void
write's address refused too - in every pass of every run, as
`test_x035_i2c`'s letter `r` finds it - and after the deadline a probe of
the command address is `i2c_ok` (the suite's
`I2cTraits::refusal_holds_address`, which the shared letter's verdicts
follow).

**The arbiter with three requesters of three priorities.** Letter `w`,
at 400 kHz: the requester above the arbiter's twenty write-then-reads of
1+6 (both halves on the engines), two outstanding; the one below it ten
writes of two bytes (the interrupt); Pressure's eight probes in a pressure
pass - every reply `i2c_ok`, to its own requester, in its order (slot k
complete and k + 1 untouched at reply k), nothing rejected, the timeout
never firing, the peer's count, sum and address matches exact; the
letter in 6.97 to 14.26 ms with the scheduler path in SRAM and 7.51 to
25.51 with it in the flash.

**The held wire: `i2c_arb_lost` at the speed in force and at another.**
Letter `h` (the peer holds SDA for 60 ms, the tenure into it, and in a
pressure pass Pressure's flood of probes, paced as below): at the speed
in force the held tenure answered `i2c_arb_lost` in 167 to 328 us from
its `start()` (167 to 363 with the scheduler path in the flash), its
START out into the held line and its address's first one bit read back
low - in 17 passes of 21 with the scheduler path in SRAM and 18 of 21
with it in the flash, over three runs of each image. Of that time 100 us
is `start()`'s wait for BUSY; the letter does not break down the rest.
Where a flood probe met the hold first and its recovery (SWRST, after which
the peripheral's START stays pending) came before the held tenure's START, the
held tenure answered `i2c_timeout`, 19.6 to 20.6 ms from its `start()` and
up to 21.4 ms from its post (the other 4 and 3 passes). Met at 400 kHz
after commands at 100 - the timing written again before the START, a PE
cycle - it is `i2c_arb_lost` too (159 to 245 us): unlike SWRST, a PE cycle
does not make this peripheral forget the line it saw taken.

Up to three of Pressure's requests a pass arrived in the arbiter's
draining window (after a timeout's recovery, before the arbiter serves the
`BusFlushed` marker event it posted itself, an order only the preemptive
kernel produces), every one answered once.

**At 97 us x 3000 a probe at every tick is more than this core can answer,
so the shared letter paces its flood.** A 97 us period is 4656 cycles. Pressure's
spin takes 3000 of them, and its way in, its tick's own dispatch and a probe
answered `i2c_rejected` by an arbiter holding its four waiting - the flood's
common case while the line is held - leave nothing.

The suite's letter `u`, run by name, floods unpaced, a probe at every tick for
2000 ticks, the pressure's vector then stopping the timer. In ten runs of each
image past the arbiter's pending four the arbiter's event queue (the kernel's
queue of the active object, not the pending four) dropped 634 to 645 posts with
the scheduler path in SRAM and 1578 to 1587 in the flash, with no reply; 1356 to
1367 of the 2000 probes were answered, 415 to 424 in the flash, each once
(`i2c_rejected` but for four `i2c_ok` after the release and one
that met the hold, `i2c_arb_lost` or `i2c_timeout`), and the next tenure was
`i2c_ok` and the arbiter idle once the flood was over. The held tenure, whose
timeout is a time event the overload starves, was answered `i2c_timeout` some 193 ms
from its `start()` in every run with the scheduler path in SRAM. With it in the
flash it was answered `i2c_rejected` in three runs and in the other seven not at all,
its post among those dropped: a load this core cannot serve, which a program sizes
away by keeping at most ten requests outstanding (util/bus_master.hpp). The
preemptive kernel's stack stayed within its bound through it: a high-water of 1288
to 1400 bytes with the scheduler path in SRAM and 1288 to 1320 in the flash, against the 2144 the
letter derives from the kernel's count of runs - at most five for the pack's
five objects - and the release listing's frames ([platform.md](platform.md)).

The shared letter `h` therefore posts a probe at a tick only when the Console
has turned since the last (the first probe of a hold excepted): at 97 us x 3000
that is 307 to 309 probes in the pass's 1117 or 1118 ticks with the scheduler path in
SRAM and 164 in 1141 to 1161 in the flash. The arbiter's queue never
overflows, the held tenure is answered in its place, and at least 14136 bytes of
the 20 KB of stack stay untouched by the kernel's ledger in every pass of every
letter (three runs of `z` on each image, the default's figure the lower).

**Dark** (letter `i`): every object idle, the arbiter's vote for the
sleep, no time event armed, TIM2 stopped with its update interrupt and
every DMA enable of DMAINTENR clear, the host's phase idle with CTLR2 at
the error vector alone, channels 6 and 7 with nothing left to move, the
console's DMA enables clear and TIM1's gate shut (the other requesters
ORed into channels 6 and 7), BUSY and MSL clear, both lines high, PB3
driven high. `unwedges()` stayed 0 and the receive engine counted no
fault in every run. The transmit engine's fault count rises by one at
every tenure an error ends while its DMA block is in flight - a NACK on a
byte the engine is writing - and at no other: the error path puts away
only the DMA block the tenure's phase holds.

**Where the code runs.** With the scheduler path in SRAM, `activate()` is
2014 bytes there, with the dispatches of Pressure, the requester below the
arbiter and the console inlined into it, `.data` 2716 bytes against the
twin's 420; the arbiter's dispatch is reached through its state machine's
handler pointer, in the flash in both images, and so is `start()`.

## Not covered yet

Driver gaps, each with its reason:

- 10-bit addressing on the host side: the resource has the mode and the
  client matches such an address; the host's header-byte sequence (EVT9)
  has no user.
- PEC as part of a tenure, and ENARP: the resource has the bits, no
  Request shape asks for them.
- The general call as a host verb: `start()` refuses address 0x00 with
  every reserved address ([../design/i2c-bus.md](../design/i2c-bus.md));
  a broadcast is a feature of its own, born with its first user.
- A wake on an address match: this stratum's power chapter has no sleep
  site.
- The collision case (twi_link's `coll`, two clients on one address with
  a third party reading): an instance is a host or a client, and this
  part's suite has one other board on its bus.
- Fast mode Plus: 15.1 names two speeds and the datasheet gives the I2C
  no timing table; the host has no third speed.
- **This chip as the bus's target, served from an active object, under
  the preemptive kernel.** A target whose bytes come from an object (its
  vector posting "the host reads" and leaving SCL held, a dispatch giving
  the byte) is the one place a held-off reader would stretch the clock;
  no util service serves a target that way (`I2cClient` is served from
  its vectors). The CH32V006's `twi_peer` can read this
  board's target (`host_read`). Born with the first util target object.

Implemented but not bench-verified, each with what would measure it:

- **The target's general call** (ENGC, GENCALL): armed and read back. No
  brio host sends a general call, so nothing here has exercised it on the
  wire; a controller that sends one would measure it.
- **The address phase's end on ADD10** (`unserved()`): `start()` refuses
  the address first, so no suite reaches it. An image with that refusal
  compiled out, against a client acknowledging 0x7A, would measure it.
- **The event vector placed in `.fast_text`**: it compiles beside the
  `InRam` instances in the family fixture (`test/family_ch32x035/i2c.cpp`),
  and no suite runs it. Letter `x` over a placed vector would measure what
  it saves against `InRam`.
- **The columns other than PC16/PC17**: PA10/PA11, PA13/PA14 and the
  debug port's pair on the other packages compile on every part (`brio
  check ch32x035`) and are refused where the package says; a board of
  each package would run them, and the debug port's pair only with the
  probe given up.
- **The USB_IOEN refusal**: nothing in the stratum sets the bit; the USB
  controller's driver, when it is born, is what would set it and see
  `init()` refuse.
- **What a START into a live tenure puts on the wire.** Letter w reads the
  flags and the far controller's report; the edges want a logic analyser on the two
  lines.
- **A foreign clock whose high phase is longer than `bus_free_us`**: taken for an idle
  wire by the page's stated assumption. The peer's divider tops out at 85 us of high
  phase, so no suite makes one; a controller under 5 kHz would measure it. A clock past
  this peripheral's own 400 kHz, whose low phases reads can fall between, is unmeasured
  too.
- **This peripheral at 400 kHz under a foreign clock.** Letter w's request and its raw
  row run at 100 kHz; a letter sweeping the rates would measure it.
- **The last byte of a read when the wait for its RxNE runs out.** On the
  pump (a host with no engines, or a request naming the interrupt), a
  read of three or more bytes sets STOP at its
  second BTF, reads byte N-1 and waits for byte N's RxNE with a bound of
  100 000 polls, tens of milliseconds at the poll length letter t times (this loop is
  not timed), every other interrupt held out, whose answer is discarded: when the bound
  runs out, DATAR is read regardless and the tenure reported `i2c_ok` with what it holds
  (`receive_step()` and `wait_until()` in
  [brio/ch32x035/i2c.hpp](../../brio/ch32x035/i2c.hpp)). No correct program reaches it:
  at that BTF byte N is already in the shifter, so its RxNE follows the read of N-1
  whatever the target does, and letter x's reads stay within 2600 cycles of their wire,
  against the bound's hundreds of thousands. An image with the poll's answer forced
  false would measure the path.
