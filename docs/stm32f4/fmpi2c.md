# FMPI2C (STM32F4)

This document is for a programmer who wants an I2C bus from the second
I2C design of this family: the FMPI2C1 peripheral of the STM32F410, F412,
F413/F423 and F446, beside the classic I2C of [i2c.md](i2c.md). It is
the one to reach for when the bus needs Fast-mode Plus (1 MHz), or a rate
held through a change of the core's clock. Its kernel clock, the input its
timing is divided from, is selectable: the default, its APB clock, moves with
the core and needs `rebase()` at every change; the HSI holds its rate and
refuses Fast-mode Plus. `brio/stm32f4/fmpi2c.hpp` has it in the layers
every brio bus driver has: `FmpI2c<n>`, the resource, one function per
register fact; `FmpI2cHost<n, pins, TxEngine, RxEngine>`, the host task,
which `util/i2c_bus.hpp` drives (a task, in brio, is a driver layer with
state of its own, served from its peripheral's interrupts; `TxEngine` and
`RxEngine` name the DMA streams bound to the host, its engines); and
`FmpI2cClient<n, pins>`, the target role. The arbiter,
`I2cBus`, is an active object (a component of a brio program with its
own event queue, served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)): the active
objects that share the bus post it requests, it hands them to the host
one tenure at a time - a tenure is one START to its STOP, the bus held for
one request - and replies to each with an event
([design/i2c-bus.md](../design/i2c-bus.md)).

The smallest use, a bus host under the arbiter reading one register of a
device, on the Nucleo-F446RE's PC6/PC7:

```cpp
constexpr FmpI2cPins pins{.scl = {'C', 6, PinFunction::af4},
                          .sda = {'C', 7, PinFunction::af4}};
using Bus = FmpI2cHost<1, pins>;
using BusAo = I2cBus<Bus, P, 4, BusPassThrough, ticks_from_ms<P>(20)>;

extern "C" void FMPI2C1_EV_IRQHandler() {
    if (Bus::isr()) { post<BusAo>(TransferDone{Bus::status()}); }
}
extern "C" void FMPI2C1_ER_IRQHandler() {
    if (Bus::error_isr()) { post<BusAo>(TransferDone{Bus::status()}); }
}

// in main(), after the clock task's init(), the APB clock as the kernel clock:
if (!Bus::init(clock)) {
    // not even 100 kHz at this kernel clock: there is no bus
}

// in a handler of MyAo, the request: a write and a read joined by a repeated
// START. The two bytes outlive the handler: they are lent until I2cDone.
post<BusAo>(Bus::Request{
    .addr = 0x1D,
    .tx = lend<Lease::reply>(&reg), .tx_len = 1,
    .rx = lend<Lease::reply>(&id), .rx_len = 1,
    .reply = reply_to<MyAo, I2cDone>(),
    .speed = FmpI2cSpeed::fast_400k,
});

// among the same state's handlers, the reply:
[](const I2cDone& done) {
    if (done.status == i2c_ok) { use(id); }   // else: the code says what the wire did
    return handled();
},
```

An excerpt, its names in `brio::` and written as inside a `using namespace
brio`. It assumes the program's platform `P`, its clock object `clock`, an
active object `MyAo` holding `reg` and `id` as static members, and the
program's `use()`. `pins` names the two pads with the alternate-function
number the datasheet gives FMPI2C1's SCL and SDA on them, which nothing in
the code can check; `Bus` is the host task over the one instance, its two DMA
engine parameters left empty; `BusAo` is the arbiter over it for the platform
`P`, with four requests of queue, `BusPassThrough` (the failure policy that
hands every status to the requester and retries nothing) and a per-bus
timeout of 20 ms, an active object listed in the program's `Tenuto`, the
kernel type that runs a program's active objects; `init()`
takes that clock object (`constexpr SysClock clock;`,
[clock.md](clock.md)), selects the kernel clock (the instance's APB1
clock by default), solves TIMINGR for the three speeds against it and hands the
pads over; the two `extern "C"` functions are the instance's event and
error interrupt handlers, as in C, under ST's names for the vectors, and
each posts the completion to the arbiter when its body says the tenure
is over. The request is one tenure: a write, a read, a write then a read
with a repeated START, or the empty probe (both lengths zero, "How to use it"
below). The active object that posts it, the requester, receives an `I2cDone`
event with the status.

What decides correctness even there:

- **Pull-ups and the part.** Pull-up resistors on both wires are the
  board's and not the chip's. The part must carry the peripheral: on a part
  without it `FmpI2c<n>` does not compile, and `n` must be 1, the one instance
  there is.
- **The timeout.** The timeout given to the arbiter is what ends a tenure a
  wedged device holds, or a read of two or more on a stream other DMA traffic has
  starved outright ("Bench findings"). With no change of speed since the last tenure, a data
  line held low parks the START with no error, and nothing in the host answers
  it. After a change of speed, whose TIMINGR write takes a PE cycle (PE is CR1's
  enable; below), the same hold is answered `i2c_arb_lost` instead ("Bench
  findings"). The timeout must sit above the longest legal tenure: 20 ms fits
  some 200 bytes at 100 kHz, nine periods a byte, so a 255-byte request (23 ms
  of wire) or a device that stretches wants more.
- **`init()` after the clock task's.** On the APB or SYSCLK kernel clock,
  TIMINGR is solved from the rate the clock type claims and not read from the
  hardware, so a bus initialized on a chip still at its 16 MHz reset rate under
  a 180 MHz clock type (45 MHz of APB1) runs at about a third of the rate asked
  - derived from the register arithmetic, not measured - and nothing refuses it
  (on the HSI kernel clock the rate does not depend on the clock type). `init()`
  answers false when not even standard mode can be solved for the kernel clock,
  or when `FmpI2cHostConfig` names a kernel clock outside the three or a digital
  filter past 15.
- **Both vectors bound by the program.** `init()` enables the two NVIC lines,
  and an unbound one is the crt's weak alias of `Default_Handler`, a silent
  spin, where the first event of the first tenure stops the program.
- **The buffers.** The buffers a `Request` names are lent (`brio::lend`) from
  the post to the `I2cDone` the arbiter replies with: untouched and in scope
  until then, which the `Borrowed` type records and nothing checks.
- **What the host refuses.** A speed the kernel clock cannot produce is answered
  `i2c_rejected` before a byte moves (`speed_ok()` says so beforehand), and so
  is an address outside 0x08..0x77 - the general call and the 10-bit prefixes
  among them, which this peripheral would send as plain 7-bit addresses where
  the classic I2C beside it sends a header
  ([../design/i2c-bus.md](../design/i2c-bus.md)).
- **What a device refuses.** An address not acknowledged answers `i2c_nack_addr`
  and a data byte a device NACKs `i2c_nack_data`, whichever mechanism the request's
  `engine` field names: `interrupt`, the default, moves the data on the event
  vector, and `dma` on two DMA streams ("A refused write" below says how the host
  tells them).
- **The two stream vectors, under `dma`.** The streams interrupt for a transfer
  error alone; a program that names the engines binds both, where an unbound one
  is the same silent spin on the first such error. A transfer error ends the
  tenure with a STOP and answers `fmpi2c_dma_fault`, this host's name for
  `i2c_dma_fault` ([../design/i2c-bus.md](../design/i2c-bus.md)): the buffer is at
  an address a DMA master does not reach. The F446 has no core-coupled memory, so
  an ordinary buffer in its SRAM or a written half in its flash does not meet it.
  After it the cells the stream reached hold their bytes and the rest what they
  held, `I2cDone` does not say where that line falls, and the requester posts again.
- **A read under `dma` needs nothing of the program for a late stream.** The reply
  comes with every byte in the buffer: the STOPF entry settles a stream other DMA
  traffic has made late, up to 13 us in the vector ("The receive stream at STOPF"
  below).
- **After a failure.** Every failed tenure leaves the host idle, and the arbiter
  starts the next queued request in the same dispatch; a retry is the requester's,
  by posting again. `i2c_timeout` says no completion came inside the arbiter's
  timeout: the arbiter has already called `recover()`, 23.4.6's PE cycle, which
  releases both lines and resets the state machines and keeps the configuration,
  so a program under a timed arbiter has no call to make. `i2c_arb_lost` needs no
  call either: the host took back what it had started. Neither frees a device
  holding the wire. If the next request fails the same way, `unstick()` (nine clock
  pulses and a STOP by hand) is the program's step, from main context and with no
  tenure in flight, which is the program's to know: no verb says whether the
  arbiter's queue is empty.
- **Fast-mode Plus.** A 1 MHz bus wants the 20 mA pad drive turned on by the
  program (`Bus::fast_plus_drive(true)`, a SYSCFG bit `init()` leaves alone; the
  suites call it after `init()`), which nothing turns on by itself.

## Documents of record

RM0390 Rev 6 ch. 23 (the F446's manual; the F410,
F412 and F413/F423 carry the same peripheral and their manuals have not been
read), DS10693 Rev 11 table 11 for the pads and its I2C
characteristics for the bus edges, RM0390 table 28 for the DMA request
mapping and ch. 9 for the DMA controller (the sections 9.3.x cited here), and ES0298
Rev 8's FMPI2C items 2.12.1 to 2.12.10, each treated under
"What the silicon does". Driver: `stm32f4/fmpi2c.hpp`, over the reserve's presence,
SMBus and request-mapping facts (`stm32f4/device_tables.hpp`, the reserve: the
table of what each part has, read off the device header), under the arbiter
and the status vocabulary of [../design/i2c-bus.md](../design/i2c-bus.md),
with the streams from [dma.md](dma.md), the kernel-clock multiplexer
from [clock.md](clock.md) and the Fast-mode Plus pad drive from the
SYSCFG ([exti.md](exti.md)). The family fixture is
`test/family_stm32f4/fmpi2c.cpp` with the negatives that refuse a second
instance, the peripheral on a part that has not got it, a bus with no SDA,
one pad twice, a pad on an absent port, an engine off the request map,
an engine on a part class whose manual was not read, one engine of two,
and an engine whose element is not a byte. Bench: `test_stm32f4_fmpi2c`
on the Nucleo-F446RE, the one board of this stratum whose part carries
the peripheral, its PC6 and PC7 wired to the same chip's I2C1 on PB8 and PB9
with a 2.2 kOhm pull-up on each line. Letters `m`, `q`, `r`, `u` and `v` make I2C1 a target,
letter `f` runs it as a second host, and it stays silent in the others. The same wires carry the classic I2C's own
suite ([i2c.md](i2c.md)) and `bench_stm32f4`'s letter `i`. A letter of a suite
is one command key of the suite's console and the group of verdicts it runs;
`z` is the key that runs them all but `v`, which runs by its own key, and `t` a
second set, run against a partner board (below).

## What the silicon does

**It is the STM32G0's I2C under another name.** CR1/CR2 with SADD,
NBYTES, RELOAD and AUTOEND; one TIMINGR word of five fields; OAR1 and
OAR2 with the second address's mask; ISR and ICR; PECR; TIMEOUTR;
separate RXDR and TXDR. Register for register it is the peripheral
[../stm32g0/i2c.md](../stm32g0/i2c.md) describes, but for the reserved bit and the
one clear below. The driver has
that driver's verb names and shapes, and each choice in its host's byte path
cites this chapter and is measured here (the inventory below). It is not the
peripheral of [i2c.md](i2c.md), which
is the F1 lineage's event machine and another chapter entirely; a part that has
both has two I2C designs on one die, which is why every name in this file
carries an `Fmp` prefix.

**What differs in the silicon from the STM32G0's, all of it this family's own:** two
vectors instead of one (table 142 splits the seven event conditions from
the six error ones, as every other peripheral of this stratum does), the
kernel-clock multiplexer in RCC_DCKCFGR2 instead of RCC_CCIPR, the Fm+
pad drive in SYSCFG_CFGR as two bits per signal instead of a bit per pad,
DMA streams instead of channels - and no wake from Stop. One difference
under the same words: writing ADDRCF clears a START parked behind a busy bus
here (23.7.8, measured below) and leaves it standing on the STM32G0B1RE
([../stm32g0/i2c.md](../stm32g0/i2c.md)).

**Table 127's row, and its one dash.** 7- and 10-bit addressing, all
three speeds up to Fast-mode Plus at 1 MHz, the independent clock and
SMBus/PMBus are all there; "wakeup from Stop mode" is not. The device
header agrees in the only way it can: no header of the pack declares an
FMPI2C_CR1_WUPEN, and 23.7.1 makes CR1's bit 18 reserved. So the driver
publishes `wakes_from_stop` as false and its `wake_from_stop()` refuses -
the one feature of the STM32G0's register file that this
silicon has not.

**One instance, and two names for it.** Every part that carries the
peripheral carries exactly one. ES0298 2.12.10 records the name as a
documentation erratum: "I2C4 and FMPI2C1 refer to the same peripheral
instance", which is why RM0390 6.3.27 calls the kernel-clock selector
"I2C4 kernel clock source selection" while the rest of the chapter says
FMPI2C1. The header's name is the one used here. The resource is still
`FmpI2c<n>` with an index, so the day a part arrives with two, no user of
the file is renamed.

**The independent clock is the reason to reach for this peripheral.**
RCC_DCKCFGR2.FMPI2C1SEL picks the peripheral's own APB clock (code 0 and
code 3, "same as 00"), SYSCLK (1) or the HSI (2). A bus on the HSI keeps
its rate through every change of the core's, and holds a speed the
kernel-clock floor of ES0298 2.12.2 (below) would refuse at a slow core; a bus on
SYSCLK gets a kernel clock far above what an APB prescaler leaves. A bus on the APB
clock or SYSCLK moves with the core, and `rebase()` solves its timing again. The other I2C of
this family has no such selector: FREQ, CCR and TRISE all divide its own
APB clock and nothing else.

**The timing register is the chapter, and its arithmetic goes both
ways.** TIMINGR is PRESC, SCLL, SCLH, SDADEL and SCLDEL, with
tPRESC = (PRESC + 1) x tI2CCLK and

- tSCLL = (SCLL + 1) x tPRESC, tSCLH = (SCLH + 1) x tPRESC, the low and
  high parts of the master's clock;
- tSCLDEL = (SCLDEL + 1) x tPRESC, the data setup stretch;
- tSDADEL = SDADEL x tPRESC, the data hold delay - **the one field with
  no +1**, and the one place the manual disagrees with itself: 23.7.5's
  field description gives that formula while 23.4.5's prose adds a
  tI2CCLK to it. Tables 134 and 135 print "2 x 250 ns = 500 ns" for
  SDADEL 0x2 at PRESC 1 and 8 MHz, which is the register description's
  formula, and that is what the driver implements.

**tSCL is not tSCLL + tSCLH.** 23.4.9's own formula adds tSYNC1 + tSYNC2
- the SCL slopes, the filter delays and two to three kernel periods of
synchronization at each edge - whose floor is 4 x tI2CCLK with the
filters off and whose value the manual's example tables assume to be 1000
ns in Sm, 750 in Fm and 500..655 in Fm+. Every term but the slopes has a
floor (two kernel periods, DNF x tI2CCLK and tAF(min) an edge; DS10693
states no tAF(min), so zero), and only the slopes are a bus fact. So **a
requested rate is a ceiling**: unless told the wire's measured tSYNC
(`FmpI2cBusTiming::sync_ns`), `fmpi2c_timing_for`, the function that solves TIMINGR
and is called the chooser below, charges that floor alone, the
fastest any wire can make, and a real wire only adds its slopes and runs
slower; told it, the bus runs nearer the rate asked, to the prescaler's
resolution. What the other choices cost is measured below.

**The kernel clock has a floor and it is an erratum's.** ES0298 2.12.2:
below a kernel period that fits inside the transmitter's minimum data
setup time the peripheral samples the previous SDA value, and the erratum
names 4 / 10 / 20 MHz for Sm / Fm / Fm+ against the standard's own
tSU;DAT. There is no register workaround - "increase the FMPI2C kernel
clock frequency" is the whole of it - so the driver refuses a speed the
kernel clock cannot legally make, and answers a Request naming it with
`i2c_rejected` rather than running the bus at a rate nobody asked for.
23.4.3's own two conditions (tI2CCLK < (tLOW - tfilters)/4 and
tI2CCLK < tHIGH) are checked beside it and refuse independently: they are
what drops Fast-mode Plus when the digital filter is deep.

**The controller is a byte counter, not a set of software
sequences.** CR2 carries the address, the direction, NBYTES and the end
policy in one word; the START bit sends everything up to the address, and
from there TXIS and RXNE pace one byte each. AUTOEND sends the STOP when
NBYTES runs out; with AUTOEND clear the transfer ends on TC with SCL
stretched, and setting START there is the repeated START. RELOAD carries
a transfer past 255 bytes through TCR - and ES0298 2.12.4 says a receiver
can lose the byte that raises it, so the host never sets RELOAD
(a Request's spans are eight-bit lengths) and the resource's `reload()`
carries the caution.

**A NACK is an event and not an error.** NACKF sits on the event vector
under NACKIE, and 23.4.9 says that on a NACK "the TXIS flag is not set,
and a STOP condition is automatically sent" - so a tenure nobody answers
ends by itself, and the completion is that STOP. The host leaves NACKIE down and
reads NACKF at the STOPF entry. A byte loaded ahead stays in TXDR through it,
TXE clear, and with TXE clear the next address's ninth pulse sends it to the
shifter (23.4.7): the host empties TXDR at that entry and at a lost arbitration,
and flushes it again before a write's first byte, so nothing of a failed request
leads the next one.

**Four errata are live in the driver.**
- 2.12.3, a spurious bus error in controller mode: "detection of bus
  error has no effect on the I2C-bus transfer in controller mode and any
  such transfer continues normally". The host clears BERR, counts it
  (`spurious_bus_errors()`) and never reports `i2c_bus_error` from it. A
  client's BERR is outside the erratum and is reported.
- 2.12.8, a transmission stalled after its first byte: when the first
  byte is not already in TXDR, it is written less than two kernel cycles
  after its request, and the APB-to-kernel clock ratio is between 1.5 and
  3, the second byte "is not internally detected" and the peripheral stalls
  until a PE cycle or a reset. The erratum's two workarounds are to write
  the first byte into TXDR before the transfer starts, or to choose a
  kernel clock outside the band. The host takes the first on one path, under
  `interrupt` and under `dma`, whatever the two clocks: `start()` flushes TXDR
  and stores the first byte before the START, and the event vector or the
  transmit stream carries the rest. Measured in the band and out of it
  ("ES0298 2.12.8" below). The ratio stays published
  (`fmpi2c_transmit_stall_risk()`) for what does not go through this host: a
  target's first byte, the erratum naming target mode too, and a program's
  own stream on the resource's verbs.
- 2.12.4, the reload-mode byte loss: answered by never using RELOAD.
- 2.12.1, a 10-bit address whose first byte is NACKed, which leaves NACKF
  and START both standing and no new transfer possible. The host's Request
  is 7-bit and cannot reach it; the resource publishes the state
  (`ten_bit_nack_stuck()`) and the escape is 23.4.6's PE cycle, which is
  `cycle()`.
Of the other six, 2.12.2 is the floor above, refused by the chooser, and
2.12.10 is the instance's name. Four are the application's: 2.12.5 and 2.12.6
(a device that is controller and target at once must write CR2 with START low
after every address match, and short of a PE cycle ADDRCF is the only thing that
clears START),
2.12.7 (a target transmitter with NOSTRETCH can send 0xFF without raising OVR)
and 2.12.9 (an SMBus time-out in target mode reports but does not release SDA;
the recovery is a PE cycle).

**The one verb that disables is 23.4.6's procedure.** Clearing PE
releases both lines, resets the state machines, clears CR2's START, STOP,
NACK and PECBYTE and every status flag - and keeps every configuration
register. PE must stay low for at least three APB cycles, and the chapter
prescribes write-0 / check-0 / write-1, so that sequence is `disable()`
plus `enable()` and there is no raw PE clear anywhere in the driver. It
is also the chapter's own deadlock escape, which is why `recover()` is
built on it and reconfigures nothing.

**The write gates are four and not one.** PE = 0 for TIMINGR, NOSTRETCH,
ANFOFF and DNF; START = 0 for CR2's SADD, RD_WRN, ADD10, HEAD10R and
NBYTES; OA1EN = 0 for OAR1's own fields and OA2EN = 0 for OAR2's;
TIMOUTEN = 0 for TIMEOUTA and TIDLE, TEXTEN = 0 for TIMEOUTB. Each verb
checks its own.

**The SMBus part is whole here**, unlike on some instances of the same IP
elsewhere: the two device addresses, the alert, the PEC unit and the
three time-outs. TIMEOUTA watches SCL held low by anybody (TIDLE = 0) or
both lines standing high (TIDLE = 1, bus idle detection); TIMEOUTB
watches this peripheral's own cumulative stretch. 23.7.6 makes the whole
register read back zero where the part is absent, which is a question the
silicon can answer - `smbus_probe()` asks it.

**The pads and the 20 mA drive.** DS10693 table 11 puts FMPI2C1_SCL on
PC6, PD12, PD14 and PF14, FMPI2C1_SDA on PC7, PD13, PD15 and PF15 and
FMPI2C1_SMBA on PD11 and PF13, all at AF4. No device header carries a pin
table, so nothing in the code can check an alternate function: the
datasheet claims it and the bench proves it. Both lines go out open drain
- that is the bus - and the pull-ups are the board's. Fast-mode Plus
wants the 20 mA output drive, which on this family is two bits of
SYSCFG_CFGR, one for the SCL signal and one for SDA (not one per pad):
whichever pad carries the signal takes the drive.

**The DMA requests are one cell each.** RM0390 table 28: FMPI2C1_RX on
DMA1 stream 2 channel 2, FMPI2C1_TX on DMA1 stream 5 channel 2, and
nothing on the other controller. The reserve keys that per part class and
refuses an engine on a class whose manual was not read. Only the data
moves: 23.4.16 says the address "cannot be transferred with DMA", so a
DMA tenure still costs the framing interrupts. The data moves at any
length, one byte included: TXDR is loaded "whenever the TXIS bit is
set", RXDR read whenever RXNE is, and the NBYTES counter ends the
transfer (23.4.16), which names no minimum length. A
write's first byte stored before the START spends one of NBYTES's TXIS
events (23.4.9), and the stream answers the rest.

**What the chapter offers a controller's transfer, and what the host takes** -
the inventory every cost below is read against. A half is a request's written
bytes or its read bytes, and the pump is the event vector moving one data byte
an entry.

| offer (section) | taken? | why |
|---|---|---|
| NBYTES with AUTOEND (23.4.7, 23.7.2) | yes | one CR2 store starts a tenure, the hardware NACKs a read's last byte and sends the STOP, and STOPF ends every shape |
| TC with AUTOEND clear (23.4.7) | yes, for a write-then-read alone | the repeated START is the CR2 store at TC; TCIE is armed for that shape only, TC having no ICR bit |
| RELOAD and TCR past 255 bytes (23.4.7) | resource only | a Request's lengths are eight bits, and ES0298 2.12.4 loses a receiver's byte there |
| one TXDR and one RXDR, no FIFO (23.2, 23.7.10, 23.7.11) | the silicon's | one interrupt per data byte on the pump; the streams carry a longer run |
| the first byte written before the START (TXE stands at rest, 23.7.7) | yes, under `interrupt` and under `dma` | a write of N takes N interrupts, not N + 1, for some 29 cycles more in `start()`; ES0298 2.12.8's first workaround on one path |
| a TXDR flush by writing TXE (23.7.7) | yes, before that store, and at a NACK or a lost arbitration | a byte left in TXDR goes out ahead of the next write ("A refused write"); the store fails on a TXDR not empty (23.7.11) |
| the last byte of a read left in RXDR for the STOPF entry (23.4.7) | yes | RXIE falls as the byte before it is read, and SCL waits on RXNE, so no later byte can pass an unread one; a read of N takes N interrupts, not N + 1 |
| NACKIE (table 142) | no | every NACK ends in the STOP that raises STOPF, whatever AUTOEND holds (23.4.9), measured on every shape the host makes ("A refused write", the read half's address after a repeated START, no RELOAD); an address not acknowledged takes one interrupt, not two |
| the enables at their flags' bit positions (23.7.1, 23.7.7) | yes, on the host's event vector (`armed_events()`) | one mask, where a test per enable (the client's `pending()`) reads some thirty-five instructions; TCR is the flag it leaves out, no host tenure setting RELOAD |
| one CR1 store for a tenure's enables | yes, `enables()` at `start()` and at the tenure's end | at rest ERRIE alone. One read and one store: `start()` runs with no tenure in flight, and at rest the vectors write CR1 only to take down an enable a program armed through the resource, so nothing is lost between the two for a program that arms none; the bits a program set through the resource (ALERTEN, GCEN) are kept |
| TXDMAEN and RXDMAEN (23.4.16) | yes, two engine parameters | the program names `dma` in the Request; a half with a buffer rides a stream, all of a write's but the first byte, which `start()` stores |
| a stream's completion interrupt (`DmaInterrupts`, [dma.md](dma.md)) | no, `errors_only` | a `dma` tenure takes STOPF and nothing per stream |
| SxCR.EN down as a receive DMA block's end (9.3.3, 9.3.14) | yes, at the STOPF entry of a read on the stream | STOPF says the wire is over and not that the stream is ("The receive stream at STOPF" below) |
| the independent kernel clock, the Fm+ drive, both filters | yes | none bears on a transfer's cost but the filters' share of the SCL period (above; the suites run with the digital filter off, and a deep one's refusal is below) |
| TIMEOUTR, the PEC, the alert, 10-bit addressing | resource only | no Request shape asks for them ("Not covered yet"); the arbiter's timeout ends a wedged tenure |
| the wake from Stop | absent | table 127 |

**ST's HAL v1.8.5 for the same operations** (`stm32f4xx_hal_fmpi2c.c`, read as the
oracle of shape, not run on this peripheral). `HAL_FMPI2C_Master_Transmit_IT` stores
the first byte before the START, as this host does, with no flush ahead of it, its
end flushing TXDR instead. Its vector takes NACKF and each RXNE as entries of their
own, RXIE up to the last byte, so a read of N is N + 1 entries. Its DMA receive ends
on the stream's completion interrupt, which arms STOPIE, so the stream's end comes
first by construction, at the price of that entry.

## Types and verbs

`FmpI2c<n>` - the resource. The gate and the reset (`bus_clock`,
`reset`), the kernel-clock multiplexer (`kernel_clock`, `kernel_hz`), the
enable and 23.4.6's procedure (`enable`, `disable`, `cycle`, `enabled`),
the configuration behind PE (`configure`, `timing`, `timing_reg`,
`filters`, `no_stretch`, `byte_control`, `general_call`), the own
addresses (`addresses`, `oar1`, `oar2`, `own_address`, `own_address10`),
the controller's transfer (`transfer`, `transfer_now`, `cr2_word`, `start`,
`starting`, `stop`, `reload`, `nbytes`, `cr2`, `nack_next`, `pec_byte`,
`pec`, `ten_bit_nack_stuck`), the data registers (`data`, `flush_tx`,
`tx_address`, `rx_address`), the status and its clears (`flags`, `flag`,
`clear`, `clear_all`, `busy`, `host_reads`, `matched_address`), the
interrupts and the two vectors' masked views (`interrupt`, `interrupts`,
`enables`, `pending`, `pending_event`, `pending_error`, `armed_events`, `isr`,
`error_isr`), the DMA enables (`dma_transmit`, `dma_receive`, and
`FmpI2cDmaEnable` for `enables`' word), the SMBus time-outs (`timeouts`,
`smbus_probe`), the Fast-mode Plus drive (`fast_plus_drive`,
`fast_plus_scl`, `fast_plus_sda`), the wake that is not there
(`wake_from_stop`), and `release` (the peripheral disabled and its clock
gated, no reset pulse, the host's and the client's `init` starting from
the RCC reset - the release contract, docs/design/dma.md). The constants `number`, `event_irq`,
`error_irq`, `smbus_claimed`, `wakes_from_stop`, `has_ten_bit`,
`has_fast_plus`.

`FmpI2cHost<n, pins, TxEngine, RxEngine>` - the host task, with the
parameter list and the `Request` of [i2c.md](i2c.md)'s `I2cHost`, field
for field, the speed an `FmpI2cSpeed` (`standard_100k`, `fast_400k`,
`fast_plus_1m`). `engine` names what moves the data, and the host never
chooses. `interrupt`, the default, is the event vector at any length. `dma` is
the engines at any length, on a host with both engine parameters named:
`start()` stores a write half's first byte in TXDR and starts the streams before
the START (23.4.16), so a write of one byte starts no stream. `has_dma` says at
compile time whether the engines are there, and `dma` on a host without them is
answered `i2c_rejected` inside `start()` with nothing on the wire. The Request does
not offer a half with no buffer (a null pointer and a length) as a use; if one
comes it stays on the vector under `dma` too: a write sends 0xFF for each byte
and a read discards what it receives, and the request completes. A stream is this
host's alone ([../design/dma.md](../design/dma.md), the ownership rule), so
`start()` does not test whether a stream's start took. Both engines arm at DMA
priority `high`: a byte RXDR still holds keeps SCL "stretched low until
FMPI2C_RXDR is read", and an empty TXDR stretches it after the ninth pulse (RM0390
23.4.7), so a late stream slows the bus and loses nothing. A read's last byte has no
next one to hold: the STOP goes out whatever the stream has done, and the STOPF entry
settles it ("The receive stream at STOPF" below).

`FmpI2cHostConfig`, what `init()` takes besides the clock:

| field | values, default | effect |
|---|---|---|
| `kernel` | `FmpI2cClock::pclk` (default), `sysclk`, `hsi` | the clock TIMINGR is divided from |
| `filters` | `.analog` true by default, `.digital` 0 to 15, 0 by default | the two noise filters; a deep digital filter drops Fast-mode Plus (below) |
| `internal_pull_up` | false by default | the port's weak pull-ups on both pads, for a bench wire with none of its own |
| `bus` | no value by default, or an `FmpI2cBusTiming` | the wire's rise, fall, setup and tSYNC; with none, the standard's limits and the tSYNC floor |

The host's verbs:

- `init` brings the host up; `rebase` solves the three speeds again when the core's
  clock changes (below); `release` stops the streams, gates the clock and gives the
  pads back.
- `speed_ok` says whether this kernel clock can make a speed, `timing_of` and
  `scl_hz` give the word solved and the rate it states, `kernel_hz` the clock that
  word divides and `reference_hz` the APB clock the registers are on.
- `start`, `status`, `isr`, `error_isr` and `dma_isr` are the arbiter's call and the
  bodies of the vectors. `idle` says the host is between tenures.
- `recover` is 23.4.6's PE cycle and stops the streams: it fixes the peripheral, and
  the arbiter calls it on a timeout. `unstick` is nine clock pulses and a STOP by
  hand on the pads: it fixes a wire a device holds. It answers the pulses it took, 0
  on a free wire, 0xFF when the line stayed low.
- `fast_plus_drive` is the 20 mA drive, `spurious_bus_errors` the count of ES0298
  2.12.3's flag, `claim_smba_pad` the alert pad for a program using SMBus through
  the resource.

`FmpI2cClient<n, pins>` - the target side. `service` and `error_service` are the two
vectors' bodies and return what happened. `init`, `release`, `kernel_hz`,
`general_call` and `byte_control` set it up and take it down. The address match is
`addressed`, `host_reads`, `matched_address` and `answer_address`; a byte the host
writes is `data_ready` and `take`, and `answer_byte` refuses a chosen one under byte
control; a byte the host reads is `data_wanted`, `give` and `flush`. The rest is
status: `stop_seen`, `clear_stop`, `host_nacked`, `clear_nack`, `host_reads_last`,
`overrun`, `clear_overrun`, `flags`, `clear`, `interrupt`.

The arithmetic, all constexpr. The standard's limits: `fmpi2c_speed_hz`,
`fmpi2c_bus_timing`, `fmpi2c_low_min_ns`, `fmpi2c_high_min_ns`,
`fmpi2c_data_valid_max_ns`. The floors and conditions on a clock:
`fmpi2c_min_kernel_hz`, `fmpi2c_datasheet_min_pclk_hz`,
`fmpi2c_clock_requirements_met`, `fmpi2c_transmit_stall_risk`. TIMINGR solved:
`fmpi2c_timing_for` (the chooser), `fmpi2c_sync_floor_cycles` and
`fmpi2c_sync_cycles` (the tSYNC it charges). TIMINGR priced: `fmpi2c_scl_hz` (on a
bus: the ceiling unmeasured, the rate itself measured; or against a budget
charged as given, the manual's tables), `fmpi2c_scll_ns`,
`fmpi2c_sclh_ns`, `fmpi2c_sdadel_ns`, `fmpi2c_scldel_ns`,
`fmpi2c_min_stretch_cycles`. A hand-written word judged: `fmpi2c_setup_ok`,
`fmpi2c_hold_ok`, `fmpi2c_hold_upper_ok`. The SMBus time-outs:
`fmpi2c_timeout_code_for`, `fmpi2c_timeout_us`. The register words:
`fmpi2c_timingr`, `fmpi2c_timing_of`, `fmpi2c_cr1`. And the validity predicates
beside each configuration type.

The reserve's own: `fmpi2c_present`, `fmpi2c_base`, `fmpi2c_clock_mask`,
`fmpi2c_event_irq`, `fmpi2c_error_irq`, `fmpi2c_has_wakeup`,
`fmpi2c_smbus_claimed`, `fmpi2c_dma_placements`,
`fmpi2c_dma_placement_valid`.

## How to use it

**A bus host under the arbiter** is the smallest use above: the same shape as
every other bus in brio, and the same `Request`. Moving a program from the other
I2C of this family to this one changes the
host's type, `I2cHost<1, i2c_pins>` becoming `FmpI2cHost<1, fmp_pins>`, its
pads, the two vectors' names and any DMA cells. The `Request` its requesters
build is unchanged but for a speed it names, which is an `FmpI2cSpeed` here.

**The empty probe**, a request with both lengths zero: a START, the address and a
STOP, `i2c_ok` if something acknowledged and `i2c_nack_addr` if nothing did. An
excerpt in the smallest use's setting:

```cpp
Bus::Request probe{};   // both lengths zero, no buffers
probe.addr = 0x1D;
probe.reply = reply_to<MyAo, I2cDone>();
post<BusAo>(probe);
```

**Choosing the kernel clock**, which this peripheral has and its neighbour has
not. An excerpt in the smallest use's setting, its `init()` replaced:

```cpp
if (!Bus::init(clock, FmpI2cHostConfig{.kernel = FmpI2cClock::hsi})) {
    // no bus
}
// 16 MHz whatever the core does - and Fm+ is refused there, because
// ES0298 2.12.2 wants 20 MHz for it. speed_ok() says so before a
// Request does. On a core whose APB1 sits at 24 to 48 MHz this is inside
// ES0298 2.12.8's band, where the erratum names a stall; the host's own paths
// do not meet it (Bench findings, measured at a ratio of 2.81).
```

**A bus whose edges are known.** The defaults are the specification's
worst-case edges for the delays and no measured tSYNC, so each speed asked
is a ceiling the wire stays at or under (about 2, 8 and 15 per cent under
on the bench's wire at 45 MHz). A board that has measured its wire - the
period less the register's tSCLL + tSCLH, which `test_stm32f4_fmpi2c`'s
letter `f` prints - states it, and runs nearer the rate asked (the findings
have how near). `rise_ns` and `fall_ns` are the wire's edges, or table 131's limits
when not measured, `setup_ns` is the standard's tSU;DAT for the mode, and these four
numbers are an illustration, not this bench's. An excerpt, the same `init()`
replaced:

```cpp
if (!Bus::init(clock, FmpI2cHostConfig{
        .bus = FmpI2cBusTiming{.rise_ns = 250, .fall_ns = 60,
                               .setup_ns = 100, .sync_ns = 240}})) {
    // no bus
}
```

**The DMA engines**, on the only two cells the request mapping gives
(controller, stream and channel in the engines' arguments), carry the data
of a Request that names `dma`; one that names nothing still runs on the event
vector. `Dma<1>::init()` resets the controller, so it runs once, before the
first engine of any driver that shares it. The two stream vectors answer a
transfer error, the one thing a stream interrupts for here. An excerpt, this
`Bus` taking the place of the smallest use's, its two I2C vectors as there:

```cpp
using Bus = FmpI2cHost<1, pins, DmaTxEngine<1, 5, 2>, DmaRxEngine<1, 2, 2>>;
static_assert(Bus::has_dma);         // or `dma` is answered i2c_rejected
extern "C" void DMA1_Stream5_IRQHandler() {
    if (Bus::dma_isr()) { post<BusAo>(TransferDone{Bus::status()}); }
}
extern "C" void DMA1_Stream2_IRQHandler() {
    if (Bus::dma_isr()) { post<BusAo>(TransferDone{Bus::status()}); }
}

// in main(), after the clock task's init():
Dma<1>::init();
if (!Bus::init(clock)) {
    // no bus
}
// and every request that should ride them: .engine = I2cEngine::dma
```

**When the core's clock changes.** A program does nothing itself. `rebase()` is
called by the `DynamicClock` the program lists this host under, as one of its users,
before the RCC moves ([clock.md](clock.md)), and `init()` does not compile under a
`DynamicClock` that does not list it. A program on a static `Clock` never meets it.
On the HSI it changes nothing, and otherwise it solves the three speeds for the new
rate and makes the PE cycle: the bus must be idle, which is the program's to arrange
before it switches. A speed the new rate cannot make answers `i2c_rejected` from
then on, and `speed_ok()` says so.

**The SMBus time-outs**, sized from the kernel clock in force. An excerpt, after
`init()`:

```cpp
const uint32_t ker = Bus::kernel_hz();
const auto idle = fmpi2c_timeout_code_for(ker, 50u, true);      // tIDLE
const auto ext  = fmpi2c_timeout_code_for(ker, 8'000u, false);  // tLOW:EXT
if (idle && ext) {                      // nullopt: too long for this kernel clock
    // Refused only for a code past twelve bits, which the chooser never returns.
    (void)FmpI2c<1>::timeouts(*idle, true, true, *ext, true);
}
```

**A target**, whose protocol is the application's. An excerpt: `pins` as in the
smallest use, and `act()` is the program's own function, which takes the
`FmpI2cClientEvent` each body returns and answers it with `take()`, `give()` or
what the event asks:

```cpp
using Peer = FmpI2cClient<1, pins>;
extern "C" void FMPI2C1_EV_IRQHandler() { act(Peer::service()); }
extern "C" void FMPI2C1_ER_IRQHandler() { act(Peer::error_service()); }

// in main(), after the clock task's init():
if (!Peer::init(clock, FmpI2cAddressConfig{.own = 0x2A}, FmpI2cSpeed::fast_400k)) {
    // the timing or the address was refused: there is no target
}
```

## Bench findings

The board is a Nucleo-F446RE at 180 MHz with 45 MHz on APB1, and the two
pads are PC6 and PC7, wired to I2C1's PB8 and PB9 with a 2.2 kOhm
pull-up on each line. Letters a to l but `f` see that bus with I2C1 silent -
what a bus with nobody answering can be made to say - letter `f` runs I2C1 as a
second host beside it, and letters m, q, r, u and v make I2C1 the target. `test_stm32f4_fmpi2c` is 124 verdicts over sixteen
letters in `z`, green in three runs of three, letters `m` and `f` in ten of ten more, `q`,
`r` and `u` in twelve of twelve more, and letter `v`, which `z` does not run, in fifteen of
fifteen. The figures of every section below that names a suite are of one image of it, and of
one image of `bench_stm32f4` and of `test_stm32f4_rubato_fmpi2c`, unless the sentence says otherwise.

**The register file at reset is the chapter's**, and the one non-zero
value is ISR = 0x0001: TXE stands out of reset, so a transmitter that
waits for TXE before its first byte waits for nothing.

**BUSY does not come up set, the opposite of the other I2C of this family.**
There, BUSY watches the peripheral's own line inputs and
reads them low while the pads are not yet in their alternate function, so
a peripheral configured before its pads comes up busy over an idle wire and
its first START never leaves ([i2c.md](i2c.md)'s trap). Here BUSY is set
by a START condition and cleared by a STOP or by PE = 0 - and 23.4.6's PE
cycle is part of every `init()` - so it read clear both before and after
the pads were handed over.

**The wake from Stop is really absent.** A one written into CR1's bit 18,
where the STM32G0's register file keeps WUPEN, reads back zero.

**The SMBus part answered for itself.** A value written into TIMEOUTA
read back, which 23.7.6 says only an instance that has it can do.

**The kernel-clock multiplexer moves the whole arithmetic.** The three
sources measured 45 MHz (APB1), 180 MHz (SYSCLK) and 16 MHz (HSI), each
read back from RCC_DCKCFGR2. Fast-mode Plus is reachable on the first two
and refused on the HSI, the host answering `i2c_rejected`, ES0298 2.12.2 wanting
20 MHz for it. ES0298 2.12.8's band lands the other way round: the HSI
against a 45 MHz APB1 is a ratio of 2.8 and sits inside the 1.5..3 window,
while the APB clock (1.0) and SYSCLK (0.25) do not; the host stalls in it
neither under `interrupt` nor under `dma` (below).

**What the chooser produces at 45 MHz**, and the three speeds it makes:

| speed | PRESC | SCLL | SCLH | SDADEL | SCLDEL | states |
|-------|-------|------|------|--------|--------|--------|
| Sm 100 kHz | 3 | 61 | 49 | 3 | 14 | 99557 Hz |
| Fm 400 kHz | 1 | 38 | 15 | 6 | 9 | 394736 Hz |
| Fm+ 1 MHz | 0 | 25 | 14 | 3 | 8 | 1000000 Hz |

The rate stated is the fastest any wire can make of the word - the
period's low and high parts plus the floor under tSYNC, four kernel periods here. Sm and
Fm, whose rates 45 MHz does not divide, come out under the speed asked for:
the period is rounded up before it is split, which is what keeps a 400 kHz
request at 394736 Hz and not at 401785.

**The speed asked is a ceiling on the wire, and the rate of ST's timing utility
is not.**
SCL was counted on the clock pad's own input buffer while an address
phase ran - the pad is an open-drain alternate function, so its IDR is the
wire - and the period measured less the tSCLL + tSCLH the register
programs is the detection delay 23.4.9 calls tSYNC1 + tSYNC2, judged on
the mean gap of the probe's clocks (nothing stretches them; the shortest
gap moves with the polling loop's phase by up to a hundred core cycles).
On the 2.2 kOhm wire it came out **between about 165 and 395 ns** over
the eight rows - always above the floor the chooser
charges (22, 89 and 250 ns at 180, 45 and 16 MHz), and well under the
1000 / 750 / 500 ns tables 134 and 135 assume. Letter `f`, the mean rate on
the pad:

| kernel | speed | stated (the ceiling) | measured | tSYNC measured |
|--------|-------|----------------------|----------|----------------|
| APB1 45 MHz | Sm | 99557 Hz | 97613 Hz | 289 ns |
| APB1 45 MHz | Fm | 394736 Hz | 368098 Hz | 272 ns |
| APB1 45 MHz | Fm+ | 1000000 Hz | 845070 or 849056 Hz | 267 or 273 ns |
| SYSCLK 180 MHz | Sm | 98955 Hz | 97402 or 97455 Hz | 178 or 183 ns |
| SYSCLK 180 MHz | Fm | 396475 Hz | 375000 Hz | 167 ns |
| SYSCLK 180 MHz | Fm+ | 1000000 Hz | 865384 Hz | 178 ns |
| HSI 16 MHz | Sm | 98765 Hz | 98576 Hz | 269 ns |
| HSI 16 MHz | Fm | 400000 Hz | 378151 Hz | 394 ns |

Not one row runs above the speed asked. The letter reads the period on the
first eight rising edges of the probe, the address and direction bits; the
completion's entry comes after them, and a tick's entry that lands among them
makes the letter count again. These are thirteen runs on one image; another
image's placement moves a row by a few of the loop's turns.

With each APB1 row's measured tSYNC stated to `init()`, the same speeds give 99502,
392156 or 393013 and 989010 Hz on the pad (1809, 459 or 458 and 182 core cycles of
mean gap), where `scl_hz()`, the period formula with that tSYNC, states 99337, 394736
and 1000000 Hz. Each is good to the prescaler's resolution and to the
instrument's.

The instrument: a turn of the loop that counts the edges is 31 to 34 core cycles at
180 MHz (the letter measures it: the cycles between the first and the last dated
edge over the turns between them), each edge is dated to within one turn, so a mean
of seven gaps is good to five cycles - about three per cent at Fm+, where the last
row's 182 cycles lie. The letter allows that above the speed and no more.

A word solved against the manual's budgets runs a wire with faster edges fast: at 45
MHz, table 135's 750 ns give register low and high times of 1288 and 577 ns, and with
this wire's measured 272 ns that is 468 kHz for 400, past fast mode's ceiling (the
arithmetic; no letter runs such a word). The default is the floor for that reason.

**ST's own arithmetic, the reference for the choice above, errs fast.** The HAL
takes a precomputed TIMINGR; ST computes it in CubeMX and in the timing
utility its STM32Cube examples ship (`I2C_GetTiming()`, the
`i2c_timing_utility.c` of STM32CubeH7's and STM32CubeU5's I2C examples,
the same function in their boards' BSP bus files). It charges each of the low
and high parts tAF(min) (50 ns, fixed in its source for every part) + tDNF + 2 x
tI2CCLK, plus a "typical" rise and fall of 640 + 20 ns (Sm), 250 + 100
(Fm) and 60 + 100 (Fm+), and keeps the candidate CLOSEST to the rate
asked inside a band of plus or minus 20 per cent - so it lands on either
side of the rate on its own model, and above it on a wire with faster
edges than it assumes. At 45 MHz it answers 0x30A03234, 0x00F02136 and
0x00400A12: on the fastest wire 107.1 kHz, 483.9 kHz and 1.32 MHz, and on
this one (its low and high parts plus the measured delays) 105.0 kHz, 444 kHz and
1.07 MHz - all three above the rate asked, the Fm one past the mode's
ceiling. The two methods share every inequality (the setup and hold
bounds, 23.4.3's conditions) and differ in that one charge;
`brio/stm32f4/fmpi2c.hpp` pins ST's three words at the bottom.

**The other I2C of this family keeps its stated rate.** The classic I2C1
(CCR, RM0390 27.6.8), run as a host through the same probe on the same
wire and counted on PB8: 100166 Hz, once in thirteen runs 100000, against the 100000 its CCR
states, and 395604 Hz, once 392156, against 394736, within four core cycles of 456 at 180 MHz,
inside a turn of the poll. Its arithmetic is [i2c.md](i2c.md)'s; what the
comparison shows is that its period does not grow by a detection delay
the way this peripheral's does, and that with CCR rounded up its stated rate is
its ceiling too, within one turn of the polling loop.

**A NACK on the address is a complete transaction, in one interrupt.** All 112
addresses from 0x08 to 0x77 answered `i2c_nack_addr` and nothing answered any
other way. The event vector's own reading of ISR at its one entry is 0x0031
(STOPF, NACKF, TXE - the address unanswered, no data byte moved, the STOP the
peripheral sends by itself gone out), with the error vector never raised. After
it BUSY is clear and both lines read high: the STOP landed on the wire.

**The idle time-out needs no peer.** TIMEOUTA with TIDLE set watches both
lines standing high, which is what an empty bus does: armed at 50 us it
raised TIMEOUT after **51.1 or 51.2 us**. It is a level and not an event -
cleared on a bus that is still idle it came back in another 8984 or 8990 core
cycles, 50 us again. The same code with TIDLE clear - the SCL-low watcher
- stayed quiet over the same wait, which is the two modes measured apart.

**And an armed time-out reaches the error vector, where this host sweeps it.**
TIMEOUT is one of the six conditions behind ERRIE,
and the host arms ERRIE for BERR and ARLO: with the vector live the host's
error body clears the flag before a polling loop can see it, so the
measurement above takes the interrupt away first. For an application: a time-out
does not end a tenure here, and a bus that stops answering is the
arbiter's timeout to catch.

**The two other clocks' arithmetic is pinned against the manual.** Every
cell of tables 134 and 135 (8 and 16 MHz, four columns each) prices out
at its own column's frequency, and every cell of tables 138, 139 and 140
gives its own time-out code - with one exception, table 139's 16 MHz row,
where 8 ms needs 62.5 units: this chooser gives at least what was asked
and lands on 63 (0x3E, 8.064 ms) where the table prints 64 (0x3F,
8.192 ms).

**A deep digital filter costs the fast end of the vocabulary**, and by
23.4.3's own condition rather than an arbitrary bound: with DNF at 15 and
a 45 MHz kernel, Sm and Fm stay reachable and Fm+ is refused.

**The Fast-mode Plus drive is two bits, one per signal.** Both set, SCL
alone, both clear - all three read back, and the register exists exactly
where the peripheral does.

**The PE cycle keeps the configuration and the RCC's reset line does
not.** After `cycle()`, TIMINGR, OAR1 and the interrupt enables all stood
unchanged and ISR was back to 0x0001; after `reset()` they were zero.

**The engines came up on the mapping's only two cells** and were torn
down by a tenure that ended in a NACK: TXDMAEN and RXDMAEN both clear
afterwards, and the next tenure ran from a clean slate. A four-byte write
and an eight-byte read both ended `i2c_nack_addr` on the address phase,
which stays on the interrupt under `interrupt` and under `dma` alike.

**The arbiter is unchanged over this second I2C design of the family.**
Four probes through `I2cBus` gave four replies, all `i2c_nack_addr`; six
posted into a four-deep queue gave six replies with one `i2c_rejected`
from the queue; a Fast-mode Plus request on the HSI came back
`i2c_rejected` through the arbiter, answered inside `start()`; and an idle bus
voted for the sleep. Letter `d` judges the host's other
refusal, the address: 0x00, 0x07, 0x78, 0x7A, 0x7B, 0x7C and 0x7F each answered
true and `i2c_rejected` by `start()`, CR2 and the interrupt enables as they
were.

**`unstick()` left a free wire alone**, answering 0 pulses, and the pads
came back to the peripheral after it.

**No bus error was seen in this suite.** ES0298 2.12.3's BERR did not fire
once in a `z` run; the count is printed rather than judged. The preemptive
suite's letter `p` does count one (below).

**The first acknowledged bytes: the host against I2C1** (letter `m`),
I2C1 a target at 0x3A served from its own two vectors - a byte sink and
a pattern source. The suite's target drops the byte it asked for ahead of the
host's closing NACK with a PE cycle, and silences its buffer interrupt outside a
read (TxE stands between tenures on that peripheral, [i2c.md](i2c.md)). The
probe, an absent address, a write of eight, reads of one, two, three,
four and eight bytes and a write of two then a read of four are byte-exact
at 100 and 400 kHz through the pump, and at 400 kHz through the DMA
engines (DMA1 streams 5 and 2, channel 2). That covers the pump, the TC branch
that turns the write half into the read on one CR2 store, the receive
path and the engines moving bytes. The last written byte of a
write-then-read NACKed by the target (one to four written, four times
a count, both speeds): 32 refusals of 32 `i2c_nack_data`, the bus let go
(START and STOP down, BUSY clear, 2 ms on) and the next tenure `i2c_ok`. The STOP
after a NACK is the peripheral's own (23.4.9), and it comes before any repeated
START is asked for, so the host has nothing to take back.

**The mechanism each request names, judged on the streams.** The letter zeroes a
stream's memory address before the tenure, so the address afterwards says whether a
DMA block was started and on which buffer. Under `dma` a write of one byte starts no stream, a
write of two starts the stream on its second byte, a read of one starts its
stream, all byte-exact, and neither stream's vector is entered, each armed for
its errors alone; under `interrupt` the engined host's write of eight and read of
eight start neither; the last written byte NACKed under `dma`, 16 refusals of 16
`i2c_nack_data`, the bus let go, the next tenure `i2c_ok`; and `dma` on the
engineless host is answered `i2c_rejected` inside `start()`, CR2 untouched, BUSY
low and no address match at I2C1 over 200 us, the next write on that host exact.

**The entries, counted** (letter `q`, the event vector's count against the shape's,
read 100 us after the completion so a flag taken down late would show). Under
`interrupt`, the plain host and the engined one alike, the probe one, a write or a
read of N entries, N for one to eight, a write-then-read tx + rx, thirteen shapes in
all;
under `dma` one for a write or a read, two for a write-then-read (the STOPF and
TC), nine shapes, every byte exact on the far end; an address nobody answers one,
written, read or probed. Fifteen runs of fifteen.

**A late entry** (letter `q`, the vector's line held off with the wire running, as
a long handler ahead of it would). A read of N with the line off from the moment
RXIE falls: RXNE and STOPF stood up together in 2400 rows of 2400 (reads of one,
two, three and eight at 100 and 400 kHz), and the entry reads the last byte, N
entries, every byte exact. A NACK with the line off from before the START, the
address's or the first byte's, under `interrupt` and under `dma`: 1800 rows of 1800,
one entry, the right status and the next write whole, TXIS up beside STOPF in all
300 rows that refuse the first of two bytes or more on the vector, which is why
the STOPF test comes before the TXIS one.

**A read half's address refused after the repeated START** (letter `q`, the far end
taking its acknowledge away with the write's last byte, 40 rows a run under both
mechanisms): the NACK comes with AUTOEND set by the second CR2 store, and the
tenure ends in the STOPF that raises `i2c_nack_addr`, two or three entries as the
shape wants, the bus let go, the next tenure `i2c_ok`, 600 of 600. With the
write half's own NACKs under "A refused write" (AUTOEND clear, RELOAD clear) those
are the shapes of a NACK this host can make: it has no RELOAD segment.

**The write after a failed one, byte for byte.** An absent address and each
NACKed position of writes of one, two, three and eight, 18 failures a pass with
no PE cycle after them, each followed by a write of eight compared with what I2C1
kept: none wrong, under `dma` and `interrupt` followed by either, and on the
engineless host, the first byte of a refused address standing in TXDR under
`interrupt` too. The resource's own verb can park a START behind a data line
held low. On the STM32F446RE that START stood until the ICR write the host's
`start()` opens with cleared it (23.7.8; ES0298 2.12.6 says the same), and the
tenure then ran on `start()`'s own CR2 store, which is why the host does not read
that store's answer. Only a program's own use of the resource stands a START there.
Thirteen runs of thirteen.

**The target half answers a real controller.** As the peer of
`test_stm32f4_i2c` on the same wires, `FmpI2cClient<1>` served from its
own two vectors is a register file at 0x2C - the address match, each
byte received and wanted, the controller's closing NACK (its byte asked
for ahead dropped by 23.7.7's TXE flush) all reported by `service()` -
through that suite's 73 verdicts, and CR2's NACK bit set as the byte
before is taken refuses a chosen written byte (23.7.2: "a NACK is sent
after current received byte"), 192 refusals of 192 on time with the core
at 180, 48 and 16 MHz ([i2c.md](i2c.md)).

**What a tenure costs this host** (`bench_stm32f4`'s letter `i` on the
same wires, I2C1 the target served on the same core, its entries counted
apart). The SCL period measured on the pad is 488 core cycles at the 400 kHz row:
368.9 kHz, under the ceiling as above, which carries 41.0 KB/s at nine periods a
byte. At 400 kHz a one-byte write costs 1052 to 1056 cycles beyond the wire, in one
interrupt, and a register read 1+1 costs 1615, in two. A 255-byte write and read take
1.00 times the wire's time on the pump (255 interrupts, 106 cycles of handler a byte
written and 113 read) and on the engines (one interrupt): a write at 40769 and 40759
B/s against 40805, the rate the letter computes for that tenure with its address,
START and STOP. `start()` is 340 to 345 cycles for a write on the pump, 317 for a
read, 312 for a probe and 304 for an address not acknowledged, and the longest
entry is 335 cycles of the event vector. The fixed cost includes the target's, because
the classic I2C stretches SCL from its address match until its vector has run, on the
same core: in the same image I2C1 as the host against this peripheral as the target
pays 210 for the same write ([i2c.md](i2c.md)).

**Which mechanism, by the cycles** (the same letter, every row of one image, three
runs that differ by at most two cycles a row). `irq` is how many entries the two I2C
vectors and the two streams took, and `isr` the cycles inside them between the
meter's stamps, the bench's two timer reads around each entry. `start` is `start()`'s
cycles, counted because on this peripheral the engines are started there. The last
column is start + isr under `dma` less the same under `interrupt`, the entries apart.
At 400 kHz; at 100 kHz every row is within two cycles of these:

| half | `interrupt`: start, irq, isr | `dma`: start, irq, isr | `dma` - `interrupt` |
|---|---|---|---|
| write 1 | 340, 1, 236 | 399, 1, 293 | +116 |
| write 2 | 344, 2, 358 | 457, 1, 346 | +101 |
| write 3 | 344, 3, 464 | 457, 1, 346 | -5 |
| write 16 | 345, 16, 1842 | 457, 1, 346 | -1384 |
| read 1 | 317, 1, 255 | 439, 1, 309 | +176 |
| read 2 | 317, 2, 387 | 443, 1, 309 | +48 |
| read 16 | 317, 16, 1971 | 443, 1, 309 | -1536 |
| register read 1+1 | 344, 2, 445 | 451, 2, 507 | +169 |
| register read 1+2 | 344, 3, 584 | 453, 2, 507 | +32 |
| register read 1+16 | 344, 17, 2165 | 453, 2, 507 | -1549 |

The first byte's place shows in both columns: a write's `start()` pays the TXDR
flush and the store, 344 cycles against the read's 317 on the vector (340 for a write
of one, which arms no TXIE), and under
`dma` a write of two or more 457, which programs a stream too, against 399 for a
write of one, which does not. An address nobody answers takes one interrupt, 263
cycles of handler under `interrupt` and 324 under `dma`, against an answered
probe's 236 and 293.

The pump's figure a byte past the first is the slope, 106 cycles written
and 113 read, one entry each, and an entry's own exception entry and
return - about 24 cycles on the Cortex-M4 (ARM DDI 0439B 3.9.1, twelve
each way at zero wait states), which no stamp sees - comes on top for
every interrupt counted. In core cycles, then:
- **a write half costs the core fewer cycles on the engines from three bytes**:
  -5 on the stamps at three, with two interrupts fewer. At two, +101 on the stamps
  less one entry is still 77 over;
- **a read half from three**: +48 on the stamps at two, less one entry, is still 24
  over. At three the slope makes the stamps -65, before the two entries saved;
- **a register read with a one-byte index from a read of three**: +32 on the stamps at
  1+2, less one entry, is 8 over. At 1+3 the slope makes the stamps -81, before the
  two entries saved.

The table has no rows for a read of three or a 1+3, which are the slope, nor for a
two-byte index, which nothing measures here.
The engines' fixed cost to the core, their program in `start()` and their share of
the entry, is about 223 cycles for a write half of two or more (457 + 346 less
344 + 236) and 180 for a read half (443 + 309 less 317 + 255), against a byte's 110
or so on the pump. The speed moves none of these lengths: the same rows at 100 kHz
say so. The letter has no row at 1 MHz, because its target is the classic I2C1, which
stops at 400 kHz; at 1 MHz a byte is some 1620 core cycles of wire against the pump's
entry of 130 or so, and letter `n`'s write and read against the partner run on the
pump, byte-exact, with no figure for a stretch of the pump's own.

**The release listing's facts.** A data byte's entry on the pump is about thirty
instructions: one load of CR1 and one of ISR, one AND and a mask where a test per
enable reads some thirty-five, the phase, then the data flag's branch, no call. The
buffer pointer and the length are the tenure's and loaded from RAM: once each on a
write entry, twice each on a read entry, where the store of the byte may alias them.
A vector is not masked (no interrupt nests over another, design/kernel.md) and
`start()` masks nothing. At the speed in force `start()` writes CR1 once: one load,
two BICs, an ORR, one store; a change of speed adds a PE cycle first (`apply()`, the
host's private TIMINGR write), two CR1 stores more. The STOPF entry's calls are
`streams_back()` and `finish()`, once a tenure, and `streams_back()` calls the
transmit engine's `progress()` on a write under `dma`. A `dma` tenure's end reads
SxNDTR once for a write and SxCR once for a read, and stores nothing into either
stream. A stream is stopped only on a failed tenure or when it is late at STOPF, and
its `stop()` masks interrupts over its two enable registers.

These figures are of one image, and placement moves them. The pump's path
is the same instructions in images that differ elsewhere, but at 180 MHz the flash
has five wait states behind its accelerator, and where a handler's branches land
changes what it pays. So a
length above is good to about one byte, and a difference of a few dozen cycles
between two images is no finding. The core
sleeps while a stream works: the platform's idle is a WFI that asks no
stream anything, and Sleep keeps the DMA clocked (RM0390
6.3.15: DMA1LPEN is set at reset and written by no verb here) - so on
this family `dma` costs no sleep, only its fixed cycles.

**The receive stream at STOPF.** With no completion entry STOPF is a `dma` read's
one edge, and it says the wire is over: RXNE rose more than a period and a half
earlier, and the stream still had its peripheral port's arbitration and its memory
port's to win (9.3.5). So the entry reads SxCR once before it answers:

- **EN low: on time.** The DMA block is finished with its last byte in memory: a transfer is
  the load, the store and the count, in that order (9.3.3), and EN falls when the
  count reaches zero (9.3.14). Nothing more is done.
- **EN high.** The stream is late, or an address not acknowledged left it nothing to move. One
  read of EN cannot say which, so the entry clears EN and waits for it to read low, and what
  stands then says what happened:
  - **A beat was under way.** "The stream may take time to be disabled (ongoing transfer is
    completed first)", and then EN "is now 0 to confirm the stream interruption" (9.3.15); the bit
    "is actually written to 0 once all the current transfers are finished" (9.3.18 step 1). So the
    beat ends first, and at EN low the byte is in memory.
  - **No beat was under way.** A DMA request the DMA controller's arbitration has not granted is
    not a transfer under way: EN falls with the byte in RXDR and RXNE up, as it does for a byte the
    stream was never asked for. The entry takes RXDMAEN down, then stores that byte itself at the
    place the stream's count gives. An address not acknowledged has no byte at all.

The cost is that one register read on every read on a stream, and a stop with a read-modify-write
of CR1 only on a late stream or an address not acknowledged.

Two ends of this path no correct program reaches, and what the code does there:

- **EN does not fall.** The wait is bounded, 200000 reads of SxCR, and the entry does not read its
  answer: it goes on, the status stays `i2c_ok`, the reply goes out and a beat can follow it. It has
  taken a timer's gate closed over a raised DMA request with a stream at `very_high` left on the cell
  ([dma.md](dma.md)), which the release contract excludes.
- **The byte in neither place.** EN low, the count not at zero, RXNE down: nothing is stored, and
  the status stays `i2c_ok` with that cell as it was. This host has no `i2c_dma_fault` for it, where
  [the STM32G0's](../stm32g0/i2c.md) has. It takes a program's own read of RXDR through the resource
  inside a tenure.

**A load makes a stream late by itself** (`test_stm32f4_fmpi2c` letter `u` on the self-link,
letter `s` against the partner at 1 MHz, letter `v` for the last row). A load, on this page, is other
DMA traffic. DMA1 has no memory-to-memory (9.3.7), so what loads this stream's controller is
peripheral-paced streams: a timer's DMA request at `very_high`, above the host's `high`, a word from
the timer's counter on APB1 to one word of SRAM, on the streams of DMA1 the host does not hold (0, 1,
3, 4, 6 and 7, RM0390 table 28), each every so many counts of the timers' 90 MHz. The copies, on this
page, are four memory-to-memory blocks on DMA2's streams 0, 4, 5 and 7 at `very_high`, 256 words of
SRAM each, started again by the core while it waits for the reply; they meet the stream at the memory,
through the bus matrix. The host's `high` is fixed by the host; the level of a program's own streams and
copies is the program's, and no row here runs this load at a lower one. Late is EN up at the vector's
read of SxCR. The figures are over fifteen runs on 180 MHz with 45 MHz on APB1, and "beats" is what the
paced streams moved in 100 us with the copies idle, the bus's own figure being 2250 (45 MHz, two clocks
an access):

| load | beats | late at 400 kHz, reads of 1, 2, 8 (2250 each) | late at 100 kHz (750 each) | late at 1 MHz (1500 each) |
|---|---|---|---|---|
| nothing else | 0 | 0, 0, 0 | 0, 0, 0 | 0, 0, 0 |
| six streams every 32 | 1690 | 0, 0, 0 | 0, 0, 0 | 0, 0, 0 |
| four every 17 | 2118 | 0, 0, 0 | 0, 0, 0 | 0, 0, 0 |
| three every 8 | 2111 | 0, 0, 0 | 0, 0, 0 | 0, 0, 0 |
| four copies | 0 | 0, 0, 0 | 0, 0, 0 | 0, 0, 0 |
| two every 16, and the copies | 1127 | 0, 0, 0 | 0, 0, 0 | 0, 0, 0 |
| six every 48, and the copies | 1127 | 0, 0, 0 | 0, 0, 0 | 0, 0, 0 |
| two every 8, and the copies | 1501 | 0, 0, 133 | 0, 0, 58 | 620, 1049, 1205 |
| three every 16, and the copies | 1689 | 11, 364, 1973 | 2, 278, 613 | 1499, 1477, 1495 |
| three every 12, and the copies | 1999 | 1922, 1481, 2176 | 304, 643, 733 | not run |

Two streams every 8 ask 2250 beats and move 1501, fewer than three every 8: in a sweep outside the suite
one stream and two both stopped at 1502 however fast they were paced ([dma.md](dma.md)), and why is not
established. In this table's rows no load of paced streams alone made one late and neither did the
copies alone, and the rows that did have both; the load in the next paragraph needs no copies. Every one
of the table's 130500 reads was exact in the vector, ahead of anything
a reply could read, `i2c_ok`, with no byte left in RXDR and the stream still once the reply stood. **So the
late path is reached by itself, 88 reads of eight in a hundred at 400 kHz under letter `u`'s heaviest row
and all but 29 of its 4500 reads at 1 MHz, and the entry holds.** Under three streams every 16 and the
copies, a first byte refused on a write of four: `i2c_nack_data`, and the next write of eight whole,
1500 of 1500 (letter `u`).
The entry's body, between two reads of the cycle counter, is at most 257 cycles on an unloaded
read and 2354 on a late one, the stop's wait for its beat: 13 us in the vector. Letter `v`, run by its own
key and not under `z`, carries the last row and has no 1 MHz column, for what such a load can do to the
core: at 1 MHz one stood the core still once, and no bound on such a load is established
([dma.md](dma.md), "Not covered yet").

**A load past what the bus takes starves the stream outright** (letter `v`, fifteen runs). Three
streams every 5, asking 5400 beats in 100 us against the bus's 2250: the stream is not served at all
while the load stands. A read of one finds its byte in RXDR with EN up, 1500 of 1500. EN then falls with
no beat, and the entry's own read puts the byte in memory, exact, and the read of eight started behind
it, with the load off, is exact, 1500 of 1500. **With the entry changed to leave RXDMAEN up over that
read of RXDR: the read of one exact, and the read of eight behind it wrong 300 of 300 over three runs,
every one `i2c_ok`, its count at zero and its first cell not a byte on the wire.** The cost of the order is a
read-modify-write of CR1 on the late path alone: in the release listing an on-time read takes the branch
on EN low before it. A read of two or more under this load does not end while it stands: its first byte
is not taken, so SCL stays low (23.4.7), and only the arbiter's per-bus timeout answers it; after
`recover()` the next read is whole, 45 of 45.

**The stop on the silicon.** Letter `r` stages what the entry's second bullet relies on. The vector's line
off, it holds RXDMAEN down, lets the DMA request up once RXNE stands and clears EN 0 to 47 turns of a delay loop later (4 to 6
core cycles a turn), 20 stops a turn, with nothing else moving, under a `very_high` stream on the same
controller and under the copies. The first read after EN reads low is the memory cell, then the stream's
flags and its count. Over fifteen runs, 14400 stops under each: in 14100 the stream had the byte, and in
every one of them, at that first read, the byte was in memory, the count at zero and TCIF up, with EN read
high at most once. In the other 300 of each, one stop in 48, the clear came ahead of the DMA request and
the byte stayed in RXDR. That is what these stops landed on, at those steps and loads; no stop was timed
against the stream's two accesses (the second list below). 9.3.13 reads the other way for data in a FIFO:
its flush "continues transferring the remaining data ... (even though stream is effectively disabled)"
and ends with TCIF. This stream is in direct mode, where a byte passes through the FIFO and is
"immediately drained" (9.3.7), and the manual does not say which of the two covers that byte, which is
why the letter asks the silicon.

**The same stop on the host's own path.** A DMA request held down to the STOPF entry and let
up there, 0 to 7 turns ahead of the host's body, makes the entry stop a running stream
over a standing request. On 7200 reads of one, two and eight at 400 kHz (letter `r`,
fifteen runs) and 4500 at 1 MHz (letter `s`, fifteen runs), every one was
exact in the vector, ahead of anything a reply could read, the stream having the byte
every time. Each 400 kHz read is followed by a read of eight, 7200 whole of 7200, and at
1 MHz the partner's count judges the sequence. Over the transmit stream, a first byte of
four refused with the same stage, 2400 writes: `i2c_nack_data`, the next write of eight
whole on I2C1.

**The byte the stream is never asked for.** Taking the DMA request away for the whole last
byte, on 2250 reads of one, two and eight over fifteen runs, the stream was running with
the byte in RXDR at every STOPF entry, and every one came out exact. With the entry changed to stop the
stream and sweep RXNE, it reads 0 exact of 150.

**A clear ahead of the DMA request leaves a stray beat, in one order.** A stray beat is one the next DMA
block's start takes with RXDMAEN clear and nothing on the wire, its count one down. When the clear lands
first, one stop in 48 above, and RXDR is then read with RXDMAEN up, that beat comes 1500 of 1500. With
RXDMAEN taken down ahead of that read, 0 of 1500. The manual does not describe this ([dma.md](dma.md) has
it). A stream a load starves lands there on every late read of one (above), so the entry takes RXDMAEN
down at EN low, ahead of that read.

**How near a stream comes to the wire's end.** Polled with the vector's line off and the DMA request let up
as RXNE rises: the time to EN low is the stream's lag, and the time to STOPF the window it has. Longest
lag and shortest window over fifteen runs at each speed, in core cycles:

| load | 400 kHz: lag, window | 1 MHz: lag, window |
|---|---|---|
| nothing else | 256, 804 | 267, 407 |
| a `very_high` stream, a beat every 10 | 104, 921 | 267, 274 |
| the copies | 315, 411 | 289, 165 |

These are not a margin. The two figures of a cell come from different tenures. The cell
at 1 MHz with nothing else reads 99 and 407 over the twelve runs of the letter alone,
and 267 and 423 inside set `t`, and nothing here says what lengthened it (the second list
below). Under the copies the wait for RXNE also starts copies between its reads, so by the code it can
date RXNE late and shorten that window. Every poll is good to a turn of three register reads, itself not
timed. The chapter states no bound on a stream's lag, and the load table above has tenures that had a
running stream at STOPF, so the entry's read of SxCR is what a read on a stream rests on.

**Against a partner board** (set `t`, letters `n`, `o`, `p` and `s`, 39 verdicts,
green in three runs of three, letters `o`, `p` and `s` in twelve of twelve more). The bullets just below are letter `n`'s 12 verdicts;
letters `o` and `p` have their own paragraphs after them, and `s` is in "A load makes a
stream late by itself" and "The same stop on the host's own path" above. The
partner is an STM32G0 on the same two nodes, running `twi_peer`, the suites'
scriptable target, commanded in band over the bus under test by frames of a
protocol every family shares (twi_link). For `z` it runs its dark console, a
program that stays off the bus, because letter `e` counts 112 NACKs and the
partner's command address would answer one of them.

- **Fast-mode Plus with every byte acknowledged**: a write and a read of
  eight bytes at the 1 MHz row, byte-exact, the partner's target - solved
  for Fm+ - accounting 8 taken and 8 served; the write, on the pump, averages **768 to
  782 kHz** of SCL (81 periods in a stopwatch that holds the request's cost
  and the partner's polled target stretching each byte), never above
  the rate.
- **`unstick()` against a client that really holds SDA**: held by the
  partner's GPIO until four SCL falling edges (latched by its EXTI), the
  line is free at the **fourth pulse** and `unstick()` answers 4; held to
  its deadline, **0xFF**; on the freed wire, 0.
- **The arbiter's per-bus timeout over this host**: a tenure the
  partner stretches 40 ms at its address is answered `i2c_timeout` by
  `I2cBus` at its 20 ms limit, and the same host carries the command
  channel after `recover()`.
- **TIMEOUTA against a held clock**, which letter `g`, on an idle bus, could not
  stage: with the clock-low watcher at 25 ms and the partner holding SCL 40 ms
  at the address, TIMEOUT reaches the error vector **25 ms** into the
  hold, every run - and the tenure goes on: this host sweeps the flag
  (no tenure here arms a time-out of its own), the silicon sends nothing
  of its own on it. This tenure is driven by hand, with no arbiter and no timeout over
  it, so when the holds are over the write completes `i2c_ok`, in 147 ms for two
  bytes: the three holds, at the address and on each byte, are 120 ms of that, and the
  rest is not broken down.
- **`rebase()` derives the APB1 rate** from
  the new SYSCLK (`apb_hz_at`, [clock.md](clock.md)): the fan-out of the `DynamicClock`
  runs before the RCC moves, so a divider read back at that point is the rate being
  left. With that derivation taken out and the divider read back, a switch from 180 to
  48 MHz solves TIMINGR for 12 MHz where 24 runs and the bus goes at **1809
  thousandths** of 100 kHz, and at 16 MHz at 1713, on a kernel clock read as 8 MHz.
  Derived, at core rates of 180, 48 and 16 MHz the bus goes at **954, 942 and 904
  thousandths**, byte-exact against the partner at each.

**ES0298 2.12.8: no stall on the host's path, in the band or out of it.**
The band is measured at one ratio, 2.81; the rest of it rests on the arithmetic
and the erratum's clause below. Letter `o` runs writes of 2, 16 and 255 bytes to the partner's target with
the HSI as the kernel against the 45 MHz APB1, a ratio of 2.81, under `interrupt`
and under `dma`, at 100 and 400 kHz; a stall is a tenure with no completion inside
twice its wire time plus 3 ticks, recovered by `recover()`'s PE cycle, the
partner's count, sum and address matches read back after each batch of a
hundred writes (twenty of the 255). Beside them runs a control the host does
not contain, staged by the letter through the resource's verbs: TXDR left
empty, the stream started on the whole run before the START, STOPF polled -
the sequence the erratum describes. Over 15 runs, three of `t` and twelve of the
letter:

| kernel | writes under | tenures | stalled |
|---|---|---|---|
| HSI, ratio 2.81 | `interrupt` | 6600 | 0 |
| HSI, ratio 2.81 | `dma` | 6600 | 0 |
| HSI, ratio 2.81 | `dma`, writes of one | 1500 | 0 |
| HSI, ratio 2.81 | the control, TXDR left empty | 6600 | 1091, 16.5 per cent |
| APB1, ratio 1.0 | `interrupt`, writes of 16 | 1500 | 0 |
| APB1, ratio 1.0 | `dma`, writes of 1, 2, 16 and 255 | 4800 | 0 |
| SYSCLK, ratio 0.25 | `dma`, writes of 2, 16 and 255 | 3300 | 0 |

Every byte of every host row is on the partner, its count, sum and address
matches exact. The control's rate does not follow the length or the speed:
15.0 to 17.4 per cent at every length and speed, a two-byte write as often as a 255-byte
one, which is the erratum's own account of a fault at the first two bytes.
Every stall leaves the partner holding exactly the first byte of that write,
its count and sum exact across the batch, and each batch's first stall, the one
the letter reads, shows ISR with BUSY alone, SCL low and SDA high and the
transmit stream's count two beats down. The next tenure after the PE cycle is
whole. What the host's path and the control
differ in is the erratum's first clause. Two kernel cycles of the HSI are 125
ns, 22 core cycles at 180 MHz; that a stream's request and its one-word store
through the bridge fall inside them is inferred from the stalls, nothing here
timing that store. With the first byte in TXDR before the
START that clause cannot arise, under either mechanism: the byte goes to the
shifter at the address's acknowledge, and each request after it asks for one byte.
Two more controls show the
letter can fail: a transmit interrupt taken away after `start()` parks a tenure
with SCL low inside the same bound, the partner holding the byte `start()` stored,
and a byte changed after its sum was taken
moves the partner's sum by what was changed. Letter `o` was green in 15 runs of
15. No Request is longer than 255 bytes, so no write runs through RELOAD.

**A refused write, position by position** (letter `p`, 12 verdicts): the partner
NACKs through its target byte control the data byte a command names, or parks
at another address for a NACK on the address. Nineteen rows - the address under
writes of one, two and sixteen and under a 1+4 and a 2+4; the first byte of one,
of a 1+4, of two, of a 2+4 and of sixteen; the second of two, of a 2+4 and of
sixteen; the third of three and of sixteen, the seventh and last of sixteen, the
200th and last of 255 - run under `interrupt` and under `dma`, at both speeds, on
the APB1 and on the HSI, each followed by a request naming either: 304 rows a run,
with no `init()` and no PE cycle between a failure and what follows it, since
either would flush TXDR. Each is judged on `start()`'s answer, on `status()` and
on the partner's count, sum, first byte, address matches and word that it
refused. Every row ended in the STOPF that NACKIE down waits for, AUTOEND set or
clear. Under `dma`, over fifteen runs of fifteen:

| refused | rows | right |
|---|---|---|
| the address | 600 | 600, `i2c_nack_addr` |
| a write of one | 240 | 240, `i2c_nack_data` |
| the first byte of two or more | 360 | 360 |
| the second | 360 | 360 |
| a later byte | 720 | 720 |
| a write of two, the address or either byte (counted above too) | 720 | 720 |

The 2280 rows under `interrupt` are right too, and the partner's count is the
byte asked for in every row.

How the STOPF entry tells them apart, with the transmit stream already stopped, so
that nothing moves between its two reads:
- **The address refused.** The first byte never left TXDR, so TXE is clear
  (23.4.7), and no TXIS rose: the vector stored nothing past it, or the stream's
  count stands at its length.
- **A data byte refused, TXIS served.** TXE reads clear too, TXDR refilled behind
  the byte on the wire, and the vector's place has passed the first byte, or the
  stream's count has moved. SxNDTR is post-decremented when a beat is stored into
  TXDR (9.3.3), not when direct mode preloads it (9.3.13), so it says a refill
  happened.
- **A data byte refused, TXIS not served yet.** The count stands, but the byte left:
  TXE is set. The row that takes the transmit request away after `start()` stages
  this one under `dma`, and the late entry above under `interrupt`: `i2c_nack_data`,
  the first byte refused, the next request whole.

So TXE set says data, and that alone answers a write of one; with TXE clear, nothing
served says the address. ISR is read at every STOPF entry; a NACK adds only the test of
TXE and, under `dma` with TXE clear, one more read of SxNDTR. A read half takes no NACK on its data, so a NACK there is the
address's.

And nothing of a failed request reaches the next one. A byte loaded ahead stays
in TXDR through the NACK and the STOP: the next one under either mechanism, or a
first byte stored before an address nobody answered. So the STOPF entry empties
TXDR at a NACK, with the stream stopped: a flush; a store, because only a store takes
down a TXIS nobody served (23.7.7); and a flush of that store, on the failed path
alone. `start()`
flushes before its own first byte. Letter `m` judges the next write byte for byte
against I2C1. A START lost into a line the partner holds low, with the
first byte stored and the stream armed, answers `i2c_arb_lost`, the stream taken
back and the next request whole, fifteen runs of fifteen. After an address not
acknowledged the next request is a write in the same serve (the partner's commanded
turn as a target), the partner's sum judging it byte for byte; after a NACKed byte
the partner keeps NACKing to its deadline, so the next request is a frame of the
command channel, whose checksum judges it.

**Under the preemptive kernel: `test_stm32f4_rubato_fmpi2c`**, the twin of
[i2c.md](i2c.md)'s `test_stm32f4_rubato_i2c` - the same source over this
peripheral - against the same partner's `twi_peer`: `util/i2c_bus.hpp`'s
arbiter (pending depth 4, the per-bus timeout at 20 ms) over the engined
`FmpI2cHost<1>` on PC6/PC7 (DMA1 streams 5 and 2 on channel 2, the APB1
kernel clock, a ratio of 1.0 outside ES0298 2.12.8's band, the Fm+ pad drive on) inside Rubato, the preemptive kernel type,
while TIM2 posts to the highest of the suite's active objects (together, the
pack) every 97, 193 or 389 us and that object's dispatch spins 300 or 3000
cycles. Every letter runs once with TIM2 silent and six times under it; **209
verdicts, green in three runs of `z`, and letters `x` in three more and `q` in ten more**.
The pack and the letters are the I2C suites' shared body
(`avrxt/src/apps/rubato_i2c.hpp`), and [platform.md](platform.md) places the
suite among the preemptive kernel's.

- **Where a preemption reaches this host: mostly before its one START store.**
  At the speed in force `start()` waits for nothing (a change of speed adds
  `apply()`'s PE cycle); it arms the engines and then writes CR2
  with START last, so a dispatch above the arbiter landing in it delays
  the START. Letter `x`, 400 writes of 16 bytes: `start()` takes 435 or 436
  cycles with TIM2 silent (the longest 591 to 594), in this image and in a loop of
  400 (the bench's 457 above is another image, and placement and the flash
  accelerator move this figure, so the two are not compared). Under 97 us x 3000
  the interrupt landed inside it 16, 16 and 20 times in three runs, against the 10
  that independent ticks would give (400 x 436 / 17460), 0 to 4 of them with
  ISR.BUSY set, which only the instructions after the START store could see. That
  is up to about twice the independent figure; the letter times no phase, so why is
  not established.
- **Every shape on the engines, with TIM2 silent and under each of its six settings**
  (letter `o`, at 400 kHz against one serve counted to its last byte; a serve is
  the partner's commanded turn as a target):
  the shared body's Requests name no mechanism, and the suite names `dma`
  for every one in a wrapper around `start()`, so a read of one rides its stream,
  a write of one is its store in TXDR alone and a longer write has its stream
  carry the rest. The probe, the reads, the write-then-reads and the
  writes are every one `i2c_ok`, every read byte-exact, the partner's
  count, sum and address matches exact. The 32-byte write 819 to 821 us
  silent and at most 825 under TIM2 (the classic I2C's 761: this peripheral's
  400 kHz row is a ceiling, its SCL slower, as above), the probe 40
  and at most 46.
- **Three speeds and the commanded stretch** (letter `p`): a read of 16, a
  1+4 and a write of 16 byte-exact at 100, 400 and 1000 kHz, the write of
  16 in 1589 to 1610, 429 and 193 to 202 us against the wire's
  1530, 382 and 153 - at Fm+ the partner's polled target stretching each
  byte, as above. The partner's stretch of 1000 us a byte at 100 kHz: a
  read of 8 in 9211 to 9228 us against its nine holds, a write of 8 in
  8991 to 9013 against its eight (the write follows the read at once and
  meets most of one hold more, [i2c.md](i2c.md) has the arithmetic).
- **A NACKed data byte is told from an address not acknowledged, under the
  engines** (letter `q`): a probe and a write to an address nothing acknowledges, the partner
  parked elsewhere, the
  write on the transmit engine, answer `i2c_nack_addr` and the next
  tenure - a probe of 0x5D, where the partner parks - `i2c_ok`; the third
  byte of a six-byte write NACKed answers `i2c_nack_data`, and so does
  the last written byte of a 2+4 NACKed. The partner reports the 2+4's two written
  bytes received and two address matches, what a host that lets go with a STOP
  produces. A write to 0x7A and one to the general call
  answer `i2c_rejected`, the partner, answering the general call,
  matching only the probe that follows - 35 verdicts of 35 in ten runs
  of the letter, and the twin's `z` 209 of 209 in three runs with it. How the
  event vector tells the two apart is "A refused write" above; the
  engineless host does it by its pump's position (`test_stm32f4_fmpi2c`'s letter `m`
  above).
- **The arbiter with three requesters of three priorities** (letter
  `w`): every reply `i2c_ok`, to its requester, in its order, nothing
  rejected, no timeout, the partner's tallies exact, in 5.8 to 6.3 ms a
  pass.
- **The held wire: a park at the speed in force, arbitration lost after a
  PE cycle** - the STM32G0's two signatures, this peripheral being its register
  file ([../stm32g0/i2c.md](../stm32g0/i2c.md)). With the partner holding
  SDA low 60 ms, the tenure posted at the speed in force parks behind
  BUSY and is answered `i2c_timeout` by the arbiter, in 19982 to
  20010 us from its `start()`, 20 passes of 21. In the other a probe the suite's highest
  object had posted first parked and timed out, the arbiter's `recover()` cleared BUSY, and the held
  tenure's START lost its address at once: `i2c_arb_lost`, 46 us from its `start()` and
  20047 from its post. In this passage the flood is the suite's
  highest object posting probes to the arbiter at each of TIM2's interrupts, paced by
  the turns of the suite's console object. The same tenure at 400 kHz after
  commands at 100 kHz pays the PE cycle of a change of speed before its START and is
  answered `i2c_arb_lost` in 25 us, every run. Under the flood (180
  to 722 probes a pass, 721 or 722 of 920 to 931 ticks at 97 us) every
  request was answered exactly once. While SDA is held they answer `i2c_rejected`; a
  probe that reaches the wire after a `recover()` nearly always answers `i2c_arb_lost`;
  after the release they answer `i2c_ok` or `i2c_nack_addr`, the next tenure is
  `i2c_ok` and the arbiter votes idle, every pass.
- **The bus error count** rose by one in each pass of letter `p` and
  nowhere else, seven a run, as on the classic I2C beside it
  ([i2c.md](i2c.md) places it at the write that ends the partner's
  counted serve at 100 kHz); "Not covered yet" says what would tell a spurious one
  from a real one.
- **Dark** (letter `i`): every object idle, the arbiter's vote, no time
  event armed, TIM2 stopped with its DMA requests off, the host between
  tenures with ERRIE alone and both DMA request enables clear, both streams
  idle, BUSY clear and PC6 and PC7 reading high on their pads.

## Not covered yet

Driver gaps, each with its reason:
- The PEC unit and the SMBus protocol above the time-outs (the alert
  pin, the address resolution protocol, the host and device default
  addresses): the register bits are all reachable through `configure()`
  and the resource's verbs, but no vocabulary, no `Request` shape and no
  task express an SMBus transaction. Born with the first program that
  speaks SMBus, which wants a peer that does.
- RELOAD past 255 bytes as a Request shape: the descriptor's spans are
  eight-bit lengths on every target of brio, and ES0298 2.12.4 makes the
  hardware's own answer unsafe for a receiver. A transfer longer than a
  page would want a design decision at the util level first
  ([../design/i2c-bus.md](../design/i2c-bus.md)).
- Target byte control (SBC) as a task: the resource has the bit and
  `answer_byte()`, but the protocol that decides whether to acknowledge a
  byte is the application's, as it is for every client in brio.
- 10-bit addressing in a Request: the descriptor carries a 7-bit address
  on every target, so the resource does ten bits and the host does not.
  A device that needs it would be the first.
- The general call and the two SMBus addresses as host verbs: `start()`
  refuses address 0x00 with every reserved address
  ([../design/i2c-bus.md](../design/i2c-bus.md)); a broadcast is a
  feature of its own, born with its first user. The resource enables the
  target side's recognition of them.
- The recovery ladder - when to unstick, when to retry, when to take a
  bus out of service - is the application's or a future policy type's, as
  on every other stratum.
- This chip as the bus's target, served from an active object, under the
  preemptive kernel. A target whose bytes come from an object (its vector
  posting "the host reads" and leaving SCL held, a dispatch giving the
  byte) is the one place a held-off reader would stretch the clock; no
  util service serves a target that way (`FmpI2cClient` is served from
  its vector), and the partner board, the STM32G0B1RE's `twi_peer`, has no
  `host_read` to read this board's target. Born with the first util
  target object.
- `FmpI2cClient::service()` still builds its pending mask enable by enable
  (`pending()`), where the host takes `armed_events()`: a target's entries have no
  cost letter yet. Born with the first measurement of them.

Implemented, not bench-verified (each with what would measure it):
- ST's HAL on this peripheral beside this host: read, not run (the inventory); a
  scratch program on the HAL alone with the same crt, clock and target would take its
  column, as [i2c.md](i2c.md) has for the classic I2C.
- `fmpi2c_dma_fault`: no letter stages a transfer error on this host. A buffer at a
  reserved address under `dma` would; the path is read from the code and from
  [dma.md](dma.md)'s measured transfer error.
- Arbitration lost between two controllers: a race wants two STARTs
  committed inside each other's detection window, and on this bench two
  held STARTs released by one STOP serialize
  ([../stm32g0/i2c.md](../stm32g0/i2c.md), "Two held STARTs released by one
  STOP serialize", measured with this chip's other I2C). What would measure it
  is a peer releasing
  its START by software at a delay swept across this one's. The status's
  own path is measured: a START into a data line the partner holds low
  loses its address and is answered `i2c_arb_lost` (Bench findings).
- What lengthens a stream's lag. No stop in letter `r` was timed against the stream's two accesses,
  and the polled lag at 1 MHz reads 99 cycles over the letter alone and 267 inside set `t` with nothing
  said of why. A stop dated on the stream's peripheral read and memory store, with a bus trace or a
  counter on the bus matrix, would measure them.
- ES0298 2.12.3's spurious BERR told apart from a real one: the count
  moved under the preemptive kernel's suite, once a pass at a write that
  ends the partner's counted serve at 100 kHz (Bench findings), which the
  partner's re-init inside that byte's acknowledge would explain as a
  misplaced STOP - a real one, counted the same. A scope on the two lines
  at that write would separate them.
- The noise filters on the wire: the register is written and read back
  and the refusals the filter delay causes are measured, but what a
  digital filter does to a glitch wants a glitch generator.
- ES0298 2.12.8 in target mode, which the erratum names: the client gives
  its first byte on TXIS and nothing has read this peripheral as a target inside
  the band. The partner's `twi_peer` has no `host_read` (twi_link's extended
  repertoire), and that op on the HSI kernel is what would measure it.
- TIMEOUTB, this peripheral's own cumulative stretch, which only a
  target being read produces: the partner's `twi_peer` has no `host_read`
  (twi_link's extended repertoire) to read this peripheral as a target, and
  that op is what would measure it.
- The Fast-mode Plus drive's effect: the two bits are written and read
  back, but the 20 mA sink shows itself in a fall time, which wants an
  oscilloscope and a bus with stronger pull-ups than the bench's.
- The instance on the other three part classes that carry it (the F410,
  F412 and F413/F423): the driver compiles for all of them, their DMA
  request tables are refused for want of a manual, and no board here
  carries one.
