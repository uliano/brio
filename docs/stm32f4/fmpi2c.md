# FMPI2C (STM32F4)

This document is for a programmer who wants an I2C bus from the second
I2C design of this family: the FMPI2C1 peripheral of the STM32F410, F412,
F413/F423 and F446, beside the classic I2C of [i2c.md](i2c.md). It is
the one to reach for when the bus needs Fast-mode Plus (1 MHz), or a
rate that survives a change of the core's clock, because it has a
kernel clock of its own. `brio/stm32f4/fmpi2c.hpp` has it in the layers
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

// in main(), after the clock task's init(), the APB clock as the kernel:
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
takes the clock task's tag (`constexpr SysClock clock;`,
[clock.md](clock.md)), selects the kernel clock (the instance's APB1
clock by default), solves the three speed rows against it and hands the
pads over; the two `extern "C"` functions are the instance's event and
error interrupt handlers, as in C, under ST's names for the vectors, and
each posts the completion to the arbiter when its body says the tenure
is over. The request is one tenure: a write, a read, a write then a read
with a repeated START, or the empty probe. The active object that posts it,
the requester, receives an `I2cDone` event with the status.

What decides correctness even there:

- **Pull-ups, the pads, the part.** Pull-up resistors on both wires are the
  board's and not the chip's. The part must carry the peripheral: on a part
  without it `FmpI2c<n>` does not compile, and `n` must be 1, the one instance
  there is.
- **The timeout.** The timeout given to the arbiter is what ends a tenure a
  wedged device holds: at the speed in force a held data line parks the START
  with no error and nothing here answers it. After a PE cycle, which a change
  of speed makes, the same hold is answered `i2c_arb_lost` instead ("Bench
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
  or when the kernel clock or the boot configuration is refused.
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
  and a data byte a device NACKs `i2c_nack_data`, under `interrupt` and under
  `dma` alike ("A refused write" below says how the host tells them).
- **Fast-mode Plus.** A 1 MHz bus wants the 20 mA pad drive turned on by the
  program (`Bus::fast_plus_drive(true)`), which nothing turns on by itself.

## Documents of record

RM0390 Rev 6 ch. 23 (the F446's manual; the F410,
F412 and F413/F423 carry the same peripheral and their manuals have not been
read), DS10693 Rev 11 table 11 for the pads and its I2C
characteristics for the bus edges, RM0390 table 28 for the DMA request
mapping, and ES0298's FMPI2C items 2.12.1 to 2.12.10, each treated under "What
the silicon does". Driver: `stm32f4/fmpi2c.hpp`, over the reserve's presence,
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
with a 2.2 kOhm pull-up on each line. I2C1 stays silent in every letter but
one, and is a target in that one. The same wires carry the classic I2C's own
suite ([i2c.md](i2c.md)) and `bench_stm32f4`'s letter `i`. A letter of a suite
is one command key of the suite's console and the group of verdicts it runs;
`z` is the key that runs them all, and `t` a second set, run against a partner
board (below).

## What the silicon does

**It is the STM32G0's I2C under another name.** CR1/CR2 with SADD,
NBYTES, RELOAD and AUTOEND; one TIMINGR word of five fields; OAR1 and
OAR2 with the second address's mask; ISR and ICR; PECR; TIMEOUTR;
separate RXDR and TXDR. Register for register and bit for bit it is the
peripheral [../stm32g0/i2c.md](../stm32g0/i2c.md) describes. The driver has
that driver's verb names and shapes; its host's byte path differs in the places
the inventory below names. It is not the peripheral of [i2c.md](i2c.md), which
is the F1 lineage's event machine and another chapter entirely; a part that has
both has two I2C designs on one die, which is why every name in this file
carries an `Fmp` prefix.

**What differs in the silicon from the STM32G0's, all of it this family's own:** two
vectors instead of one (table 142 splits the seven event conditions from
the six error ones, as every other peripheral of this stratum does), the
kernel-clock multiplexer in RCC_DCKCFGR2 instead of RCC_CCIPR, the Fm+
pad drive in SYSCFG_CFGR as two bits per signal instead of a bit per pad,
DMA streams instead of channels - and no wake from Stop. One difference
under the same words: writing ADDRCF takes a START parked behind a busy bus
here (23.7.8, measured below) and leaves it standing on the STM32G0B1RE
([../stm32g0/i2c.md](../stm32g0/i2c.md)).

**Table 127's row, and its one dash.** 7- and 10-bit addressing, all
three speeds up to Fast-mode Plus at 1 MHz, the independent clock and
SMBus/PMBus are all there; "wakeup from Stop mode" is not. The device
header agrees in the only way it can: no header of the pack declares an
FMPI2C_CR1_WUPEN, and 23.7.1 makes CR1's bit 18 reserved. So the driver
publishes `wakes_from_stop` as false and its `wake_from_stop()` refuses -
the one feature of the STM32G0's identical register file that this
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
erratum's kernel-clock floor would refuse at a slow core; a bus on SYSCLK
gets a kernel far above what an APB prescaler leaves. The other I2C of
this family has no such selector: FREQ, CCR and TRISE all divide its own
APB clock and nothing else.

**The timing register is the chapter, and its arithmetic goes both
ways.** TIMINGR is PRESC, SCLL, SCLH, SDADEL and SCLDEL, with
tPRESC = (PRESC + 1) x tI2CCLK and

- tSCLL = (SCLL + 1) x tPRESC, tSCLH = (SCLH + 1) x tPRESC, the two
  halves of the master's clock;
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
(`FmpI2cBusTiming::sync_ns`), the chooser charges that floor alone, the
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
ends by itself, and the completion is that STOP. A byte loaded ahead stays
in TXDR through it, TXE clear, and with TXE clear the next address's ninth
pulse sends it to the shifter (23.4.7): the host empties TXDR at the NACK and
at a lost arbitration, so nothing of a failed request leads the next one.

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
  kernel clock outside the band. The host takes the first under `dma`, on
  one path whatever the two clocks: `start()` flushes TXDR, stores the first
  byte and starts the transmit stream on the rest, all before the START, and
  a write of one byte starts no stream. Under `interrupt` the erratum asks
  nothing: the first store comes later than the window. Both are measured in
  the band and out of it ("ES0298 2.12.8" below). The ratio stays published
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
after every address match, and ADDRCF is the only thing that clears START),
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
bytes or its read bytes. "No reason" means what it says: neither RM0390 nor the
code gives one.

| offer (section) | taken? | why |
|---|---|---|
| NBYTES with AUTOEND (23.4.7, 23.7.2) | yes | one CR2 store starts a tenure, the hardware NACKs a read's last byte and sends the STOP, and STOPF ends every shape |
| TC with AUTOEND clear (23.4.7) | yes, for a write-then-read alone | the repeated START is the CR2 store at TC; TCIE is armed for that shape only, TC having no ICR bit |
| RELOAD and TCR past 255 bytes (23.4.7) | resource only | a Request's lengths are eight bits, and ES0298 2.12.4 loses a receiver's byte there |
| one TXDR and one RXDR, no FIFO (23.2, 23.7.10, 23.7.11) | the silicon's | one interrupt per data byte, called the pump on this page; the streams carry a longer run |
| the first byte written before the START (TXE stands at rest, 23.7.7) | under `dma`; not under `interrupt`, no reason | ES0298 2.12.8's first workaround under `dma`; the cost of the other is below |
| a TXDR flush by writing TXE (23.7.7) | yes, under `dma` before that store, and at a NACK or a lost arbitration | a byte left in TXDR goes out ahead of the next write ("A refused write") |
| the last byte of a read left in RXDR for the STOPF entry | no, no reason | RXIE stays up: a read of N takes N + 1 interrupts |
| NACKIE (table 142) | yes, no reason | NACKF has an entry of its own, and STOPF ends the tenure on the next |
| the enables at their flags' bit positions (23.7.1, 23.7.7) | no, no reason | `pending()` tests each enable in turn, some thirty-five instructions an entry in the release listing, where one AND would do |
| one CR1 store for a tenure's enables | no, no reason | one read-modify-write of CR1 under `interrupt`, up to five under `dma` (`start()`, `launch_dma()`); TXIE and RXIE stay armed between tenures |
| TXDMAEN and RXDMAEN (23.4.16) | yes, two engine parameters | the program names `dma` in the Request; a half with a buffer rides a stream, a write past its stored first byte |
| a stream's completion interrupt (`DmaInterrupts`, [dma.md](dma.md)) | yes, no reason | each stream a tenure starts takes an entry beyond the STOPF; `errors_only` is on offer and STOPF already proves the run went through |
| the independent kernel clock, the Fm+ drive, both filters | yes | none bears on a transfer's cost but the filters' share of the SCL period (above, measured below) |
| TIMEOUTR, the PEC, the alert, 10-bit addressing | resource only | no Request shape asks for them ("Not covered yet"); the arbiter's timeout ends a wedged tenure |
| the wake from Stop | absent | table 127 |

**Where the host's pump differs from the STM32G0's, with no reason.** The
register file is the same, and the host of
[../stm32g0/i2c.md](../stm32g0/i2c.md) takes three offers this one does not: it
stores a write's first byte before the START under `interrupt` too, it reads a
read's last byte at the STOPF entry, and it keeps NACKIE down and reads NACKF at
that entry. Here the first store under `interrupt` is the vector's, RXIE stays up
for the last byte and NACKF takes an entry of its own. **There is no reason for any
of the three in RM0390 or in the code.** The erratum does not make one: ES0298 2.12.8 asks nothing of a late first
store, and it forbids no early one, which this host's own `dma` path makes. The
split vectors do not make one: NACKF and STOPF are both on the event vector
(table 142). What each costs a correct program, read off `bench_stm32f4`'s letter
`i` ("Which engine, by the cycles" below, and its two probe rows), the entry's own
24 cycles on top of the stamps (an estimate, no such build having run):

- A write under `interrupt` takes N + 1 interrupts for N bytes, against N: one
  more entry of about 126 to 140 cycles a write, less the store's 28 in `start()`.
- A read takes N + 1 interrupts, against N: one more entry of about 135 cycles a
  read.
- An address not acknowledged takes two interrupts, against one: the probe's 372
  cycles of handler, where the acknowledged probe's is 250.

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
`pending`, `pending_event`, `pending_error`, `isr`, `error_isr`), the DMA
enables (`dma_transmit`, `dma_receive`), the SMBus time-outs (`timeouts`,
`smbus_probe`), the Fast-mode Plus drive (`fast_plus_drive`,
`fast_plus_scl`, `fast_plus_sda`), the wake that is not there
(`wake_from_stop`), and `release` (the peripheral disabled and its clock
gated, no reset pulse, the host's and the client's `init` starting from
the RCC reset - the release contract, docs/design/dma.md). The constants `number`, `event_irq`,
`error_irq`, `smbus_claimed`, `wakes_from_stop`, `has_ten_bit`,
`has_fast_plus`.

`FmpI2cHost<n, pins, TxEngine, RxEngine>` - the host task, with the
parameter list and the `Request` of [i2c.md](i2c.md)'s `I2cHost`, field
for field, the speed an `FmpI2cSpeed` (with `fast_plus_1m`). `engine`
names what moves the data, and the host never chooses: `interrupt`, the
default, the event vector at any length; `dma` the engines at any
length, started in `start()` before the START (23.4.16) with a write half's
first byte already stored in TXDR, so that a write of one byte starts no
stream, on a host with both engine parameters named - `has_dma` says so at
compile time, and `dma` on a host without them is answered
`i2c_rejected` inside `start()` with nothing on the wire. The Request does
not offer a half with no buffer (a null pointer and a length) as a use; if one
comes it stays on the vector under `dma` too: a write sends 0xFF for each byte
and a read discards what it receives, and the request completes. A stream is this host's alone
([../design/dma.md](../design/dma.md), the ownership rule), so `start()` does
not ask it whether it started. The field sits in the padding after
`speed`, the Request 28 bytes with it and without it. The verbs: `init`, `rebase`, `speed_ok`, `timing_of`, `scl_hz`,
`kernel_hz`, `reference_hz`,
`spurious_bus_errors`, `fast_plus_drive`, `claim_smba_pad`, `idle`,
`start`, `status`, `isr`, `error_isr`, `dma_isr`, `unstick`, `recover`,
`release`. Both engines arm at DMA priority `high`: a byte RXDR still holds
keeps SCL "stretched low until FMPI2C_RXDR is read", and an empty TXDR
stretches it after the ninth pulse (RM0390 23.4.7) - a starved stream
slows the bus and loses nothing.

`FmpI2cClient<n, pins>` - the target side: `init`, `kernel_hz`,
`general_call`, `byte_control`, `addressed`, `host_reads`,
`matched_address`, `answer_address`, `data_ready`, `take`, `data_wanted`,
`give`, `flush`, `answer_byte`, `stop_seen`, `clear_stop`, `host_nacked`,
`clear_nack`, `overrun`, `clear_overrun`, `flags`, `clear`, `interrupt`,
`service`, `error_service`, `host_reads_last`, `release`.

The arithmetic, all constexpr: `fmpi2c_speed_hz`, `fmpi2c_bus_timing`,
`fmpi2c_low_min_ns`, `fmpi2c_high_min_ns`, `fmpi2c_data_valid_max_ns`,
`fmpi2c_min_kernel_hz`, `fmpi2c_datasheet_min_pclk_hz`,
`fmpi2c_transmit_stall_risk`, `fmpi2c_clock_requirements_met`,
`fmpi2c_timing_for` (the chooser), `fmpi2c_sync_floor_cycles` and
`fmpi2c_sync_cycles` (the tSYNC it charges), `fmpi2c_scl_hz` (on a bus:
the ceiling unmeasured, the rate itself measured; or against a budget
charged as given, the manual's tables), `fmpi2c_scll_ns`,
`fmpi2c_sclh_ns`, `fmpi2c_sdadel_ns`, `fmpi2c_scldel_ns`,
`fmpi2c_min_stretch_cycles`, `fmpi2c_setup_ok`, `fmpi2c_hold_ok`,
`fmpi2c_hold_upper_ok`, `fmpi2c_timeout_code_for`, `fmpi2c_timeout_us`,
`fmpi2c_timingr`, `fmpi2c_timing_of`, `fmpi2c_cr1`, and the validity
predicates beside each configuration type.

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
is a ceiling the wire stays at or under (about 2, 7 and 15 per cent under
on the bench's wire at 45 MHz). A board that has measured its wire - the
period less the register's tSCLL + tSCLH, which `test_stm32f4_fmpi2c`'s
letter `f` prints - states it, and runs nearer the rate asked (the findings
have how near). An excerpt, the same `init()` replaced:

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
first engine of any driver that shares it. An excerpt, this `Bus` taking the
place of the smallest use's, its two I2C vectors as there:

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
pull-up on each line. Letters a to l see that bus with I2C1 silent -
what a bus with nobody answering can be made to say - and letter m makes
I2C1 the target. `test_stm32f4_fmpi2c` is 105 verdicts over thirteen
letters, and `z` was green in three runs of three, letters `m` and `f` in ten of
ten.

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
where the STM32G0's identical register file keeps WUPEN, reads back zero.

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
period's halves plus the floor under tSYNC, four kernel periods here. Sm and
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
On the 2.2 kOhm wire it came out **between about 170 and 400 ns** over
the eight rows - always above the floor the chooser
charges (22, 89 and 250 ns at 180, 45 and 16 MHz), and well under the
1000 / 750 / 500 ns tables 134 and 135 assume. Letter `f`, the mean rate on
the pad:

| kernel | speed | stated (the ceiling) | measured | tSYNC measured |
|--------|-------|----------------------|----------|----------------|
| APB1 45 MHz | Sm | 99557 Hz | 97879 Hz | 261 ns |
| APB1 45 MHz | Fm | 394736 Hz | 371134 Hz | 250 ns |
| APB1 45 MHz | Fm+ | 1000000 Hz | 845070 Hz | 273 ns |
| SYSCLK 180 MHz | Sm | 98955 Hz | 97402 Hz | 183 ns |
| SYSCLK 180 MHz | Fm | 396475 Hz | 375000 Hz | 167 ns |
| SYSCLK 180 MHz | Fm+ | 1000000 Hz | 865384 Hz | 178 ns |
| HSI 16 MHz | Sm | 98765 Hz | 98576 to 98630 Hz | 263 to 269 ns |
| HSI 16 MHz | Fm | 400000 Hz | 378151 Hz | 394 ns |

Not one row runs above the speed asked. The letter reads the period on the
first eight rising edges of the probe, the address and direction bits: the
ninth is where the peripheral raises its NACK, and that entry would date the rise
late or hide it. These are thirteen runs on one image; another image's placement
moves a row by a few of the loop's turns. The APB1 delays stated to
`init()` give 99502 or 99722, 384615 and 1011235 Hz on the pad (1809 or 1805,
468 and 178 core cycles of mean gap). The arithmetic states 99557, 391304 and
1000000 Hz for the same words: the period formula with the wire's own tSYNC, good
to the prescaler's resolution and to this instrument's. A turn of the loop that
counts the edges is 31 to 35 core cycles at 180 MHz (the letter measures
it: the cycles between the first and the last dated edge over the turns
between them), each edge is dated to within one turn, so a mean of seven
gaps is good to about six cycles - three per cent at Fm+, where the last
row's 178 cycles lie. The letter allows exactly that above the speed and no
more, and takes a count again when a tick's entry landed inside it. A word
solved against the manual's budgets runs a wire with faster edges fast: at
45 MHz, table 135's 750 ns give register halves of 1288 and 577 ns, and with this
wire's measured 250 ns that is 473 kHz for 400, past fast mode's ceiling (the
arithmetic; no letter runs such a word). That is why the default is the floor.

**ST's own arithmetic, the reference for the choice above, errs fast.** The HAL
takes a precomputed TIMINGR; ST computes it in CubeMX and in the timing
utility its STM32Cube examples ship (`I2C_GetTiming()`, the
`i2c_timing_utility.c` of STM32CubeH7's and STM32CubeU5's I2C examples,
the same function in their boards' BSP bus files). It charges each half
tAF(min) (50 ns, fixed in its source for every part) + tDNF + 2 x
tI2CCLK, plus a "typical" rise and fall of 640 + 20 ns (Sm), 250 + 100
(Fm) and 60 + 100 (Fm+), and keeps the candidate CLOSEST to the rate
asked inside a band of plus or minus 20 per cent - so it lands on either
side of the rate on its own model, and above it on a wire with faster
edges than it assumes. At 45 MHz it answers 0x30A03234, 0x00F02136 and
0x00400A12: on the fastest wire 107.1 kHz, 483.9 kHz and 1.32 MHz, and on
this one (its halves plus the measured delays) 105.2 kHz, 449 kHz and
1.06 MHz - all three above the rate asked, the Fm one past the mode's
ceiling. The two methods share every inequality (the setup and hold
bounds, 23.4.3's conditions) and differ in that one charge;
`brio/stm32f4/fmpi2c.hpp` pins ST's three words at the bottom.

**The other I2C of this family keeps its stated rate.** The classic I2C1
(CCR, RM0390 27.6.8), run as a host through the same probe on the same
wire and counted on PB8: 100000 Hz against the 100000 its CCR states and
396475 Hz against 394736, two core cycles of 456 at 180 MHz, inside a turn of
the poll. Its arithmetic is [i2c.md](i2c.md)'s; what the
comparison shows is that its period does not grow by a detection delay
the way this peripheral's does, and that with CCR rounded up its stated rate is
its ceiling too, within one turn of the polling loop.

**A NACK on the address is a complete transaction, and its two flags come
in order.** All 112 addresses from 0x08 to 0x77 answered `i2c_nack_addr`
and nothing answered any other way. The event vector's own reading of ISR
shows the sequence: 0x8011 first (BUSY, NACKF, TXE - the address
unanswered, no data byte moved) and 0x0021 second (STOPF, TXE - the STOP
the peripheral sends by itself), on two separate interrupts, with the
error vector never raised. After it BUSY is clear and both lines read
high: the STOP landed on the wire.

**The idle time-out needs no peer.** TIMEOUTA with TIDLE set watches both
lines standing high, which is what an empty bus does: armed at 50 us it
raised TIMEOUT after **51.1 us**. It is a level and not an event -
cleared on a bus that is still idle it came back in another 8985 core
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
a pattern source - its byte asked for ahead of the host's closing NACK
dropped by a PE cycle, its buffer interrupt silenced outside a read
(TxE stands between tenures on that peripheral, [i2c.md](i2c.md)). The
probe, an absent address, a write of eight, reads of one, two, three,
four and eight bytes and a write of two then a read of four, byte-exact
at 100 and 400 kHz through the pump, and at 400 kHz through the DMA
engines (DMA1 streams 5 and 2, channel 2): the byte pump, the TC branch
that turns the write half into the read on one CR2 store, the receive
path and the engines moving bytes. The last written byte of a
write-then-read NACKed by the target (one to four written, four times
a count, both speeds): 32 refusals of 32 `i2c_nack_data`, the bus let go
(START and STOP down, BUSY clear, 2 ms on) and the next tenure `i2c_ok` -
this peripheral has no repeated START to withdraw, the STOP after a NACK
being its own (23.4.9).

**The mechanism each request names, judged on the streams.** A stream's memory
address, zeroed before the tenure, names the tenure's buffer after it only if a
DMA block's start wrote it, and its vector is entered once by the DMA block's
completion. Under `dma` a write of one byte starts no stream and enters no stream
vector, a write of two starts the stream on its second byte, a read of one starts
its stream, all byte-exact; under `interrupt` the engined host's write of eight
and read of eight start neither and enter no stream vector; the last written
byte NACKed under `dma`, 16 refusals of 16 `i2c_nack_data`, the bus let go, the
next tenure `i2c_ok`; and `dma` on the engineless host is answered `i2c_rejected`
inside `start()`, CR2 untouched, BUSY low and no address match at I2C1
over 200 us, the next write on that host exact.

**The write after a failed one, byte for byte.** An absent address and each
NACKed position of writes of one, two, three and eight, 18 failures a pass with
no PE cycle after them, each followed by a write of eight compared with what I2C1
kept: none wrong, under `dma` and `interrupt` followed by either, and on the
engineless host. The resource's own verb can park a START behind a data line
held low. On the STM32F446RE that START stood until the ICR write `start()`
opens with took it (23.7.8; ES0298 2.12.6 says the same), and `start()` over one
answered false and stored its own CR2, which is why the host does not read that
store's answer. Only a program's own use of the resource stands a START there.
Ten runs of ten.

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
apart; the SCL period measured on the pad, 488 core cycles at the 400
kHz row - 368.9 kHz, under the ceiling as above). At 400 kHz: a one-byte
write 1150 to 1154 cycles beyond the wire in two interrupts, a register read 1+1
1722 in four, a 255-byte write and read x 1.00 on the pump (256 interrupts, 140
cycles of handler a byte written and 135 read) and on the engines (two
interrupts), `start()` 307 to 315 cycles on the pump, the longest entry 532
cycles of the event vector. The fixed cost carries the target's: the
classic I2C stretches SCL from its address match until its vector has
run, on the same core; I2C1 as the host against this peripheral as the target
pays 211 for the same write ([i2c.md](i2c.md)).

**Which engine, by the cycles** (the same letter, every row of one image;
`irq` and `isr` the two I2C vectors' and the two streams', between the
meter's stamps, the bench's two timer reads around each entry; on this peripheral the engines are started in `start()`,
so its cycles are counted too). At 400 kHz; at 100 kHz every row is
within three cycles of these:

| half | `interrupt`: start, irq, isr | `dma`: start, irq, isr | `dma` - `interrupt` |
|---|---|---|---|
| write 1 | 312, 2, 403 | 417, 1, 414 | +116 |
| write 2 | 312, 3, 529 | 473, 2, 499 | +131 |
| write 3 | 312, 4, 668 | 473, 2, 499 | -8 |
| write 16 | 312, 17, 2488 | 473, 2, 499 | -1828 |
| read 1 | 307, 2, 391 | 445, 2, 629 | +376 |
| read 2 | 307, 3, 526 | 445, 2, 629 | +241 |
| read 16 | 307, 17, 2424 | 445, 2, 629 | -1657 |
| register read 1+1 | 308, 4, 750 | 501, 3, 866 | +309 |
| register read 1+2 | 308, 5, 885 | 501, 3, 866 | +174 |
| register read 1+16 | 308, 19, 2783 | 501, 3, 866 | -1724 |

The first byte's place shows in the `dma` column. A write of two or more
pays the TXDR flush and the store, seven instructions in the release listing
of which two are stores across the bridge: 473 cycles of `start()` against
the read's 445, whose stream program is the same and whose path holds neither.
A write of one programs no stream and takes no stream vector, 417 cycles and
one interrupt, the STOP's. The `interrupt` column stores nothing before the
START (the inventory above: no reason).

The pump's figure a byte past the first is the slope, 140 cycles written
and 135 read, one entry each, and an entry's own exception entry and
return - about 24 cycles on the Cortex-M4 (ARM DDI 0439B 3.9.1, twelve
each way at zero wait states), which no stamp sees - comes on top for
every interrupt counted. In core cycles, then:
- **a write half costs the core fewer cycles on the engines from three bytes**,
  -8 on the stamps with two interrupts fewer;
- **a read half from four**: the table's +241 at two falls 135 a byte, to +106 at
  three, which two entries fewer (48 cycles) do not cover, and -29 at four;
- **a register read with a one-byte index from a read of three** counting the
  entries: the slope takes +174 at 1+2 to +39 at 1+3, with three interrupts fewer,
  72 cycles.

The table has no rows for a read of three or four, or a 1+3: those are the slope.
The engines' fixed cost to the core, their program in `start()` and their
completion, is about 260 cycles for a write half of two or more (473 + 499 less
312 + 403) and 380 for a read half (445 + 629 less 307 + 391), a read half's end
also stopping its receive stream, against a byte's 140 or so on the pump. The
speed moves none of these lengths.

These figures are of one image, and placement moves them. The pump's path
is the same instructions in images that differ elsewhere, and across four such
images it read 124, 136, 140 and 146
cycles a byte written: at 180 MHz the flash has five wait states behind its
accelerator, and where a handler's branches land changes what it pays. So a
length above is good to about one byte, and a difference of a few dozen cycles
between two images is no finding. The core
sleeps while a stream works: the platform's idle is a WFI that asks no
stream anything, and Sleep keeps the DMA clocked (RM0390
6.3.15: DMA1LPEN is set at reset and written by no verb here) - so on
this family `dma` costs no sleep, only its fixed cycles.

**Against a partner board** (set `t`, letters `n`, `o` and `p`, 33
verdicts): letter `n`'s 12 green in every run of `t`, the other two's below. The
partner is an STM32G0 on the same two nodes, running `twi_peer`, the suites'
scriptable target, commanded in band over the bus under test by frames of a
protocol every family shares (twi_link). For `z` it runs its dark console, a
program that stays off the bus, because letter `e` counts 112 NACKs and the
partner's command address would answer one of them.

- **Fast-mode Plus with every byte acknowledged**: a write and a read of
  eight bytes at the 1 MHz row, byte-exact, the partner's target - solved
  for Fm+ - accounting 8 taken and 8 served; the write averages **761..776
  kHz** of SCL (81 periods in a stopwatch that holds the request's cost
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
- **TIMEOUTA against a held clock**, which letter `g` could not stage:
  with the clock-low watcher at 25 ms and the partner holding SCL 40 ms
  at the address, TIMEOUT reaches the error vector **25 ms** into the
  hold, every run - and the tenure goes on: this host sweeps the flag
  (no tenure here arms a time-out of its own), the silicon sends nothing
  of its own on it, and when the holds are over the write completes
  `i2c_ok` (147 ms for two bytes held 40 ms each and at the
  address).
- **`rebase()` derives the APB1 rate** from
  the new SYSCLK (`apb_hz_at`, [clock.md](clock.md)): the DynamicClock's
  fan-out runs before the RCC moves, so a divider read back there is the
  rate being left. With the divider read back, a switch from 180 to 48 MHz solves
  TIMINGR for 12 MHz where 24 runs and the bus goes at **1811 thousandths** of 100
  kHz, and at 16 MHz at 1694. Derived, the three cores give **955, 942
  and 901 thousandths**, byte-exact against the partner at each.

**ES0298 2.12.8: no stall on the host's paths, in the band or out of it.**
The band is measured at one ratio, 2.81; the rest of it rests on the arithmetic
and the erratum's clause below. Letter `o` runs writes of 2, 16 and 255 bytes to the partner's target with
the HSI as the kernel against the 45 MHz APB1, a ratio of 2.81, on both
engines, at 100 and 400 kHz; a stall is a tenure with no completion inside
twice its wire time plus 3 ticks, recovered by `recover()`'s PE cycle, the
partner's count, sum and address matches read back after each batch of a
hundred writes (twenty of the 255). Beside them runs a control the host does
not contain, staged by the letter through the resource's verbs: TXDR left
empty, the stream started on the whole run before the START, STOPF polled -
the sequence the erratum describes. Over 46 runs:

| kernel | writes on | tenures | stalled |
|---|---|---|---|
| HSI, ratio 2.81 | the event vector | 20240 | 0 |
| HSI, ratio 2.81 | the DMA engines | 20240 | 0 |
| HSI, ratio 2.81 | the DMA engines, writes of one | 4600 | 0 |
| HSI, ratio 2.81 | the control, TXDR left empty | 20240 | 2828, 14.0 per cent |
| APB1, ratio 1.0 | the event vector, writes of 16 | 4600 | 0 |
| APB1, ratio 1.0 | the DMA engines, writes of 1, 2, 16 and 255 | 14720 | 0 |
| SYSCLK, ratio 0.25 | the DMA engines, writes of 2, 16 and 255 | 10120 | 0 |

Every byte of every host row is on the partner, its count, sum and address
matches exact. The control's rate does not follow the length or the speed:
12.8 to 15.4 per cent in every row, a two-byte write as often as a 255-byte
one, which is the erratum's own account of a fault at the first two bytes.
Every stall leaves the partner holding exactly the first byte of that write,
its count and sum exact across the batch, and each batch's first stall, the one
the letter reads, shows ISR with BUSY alone, SCL low and SDA high and the
transmit stream's count two beats down. The next tenure after the PE cycle is
whole. What the host's two paths and the control
differ in is the erratum's first clause. Two kernel cycles of the HSI are 125
ns, 22 core cycles at 180 MHz; that a stream's request and its one-word store
through the bridge fall inside them is inferred from the stalls, nothing here
timing that store. With the first byte in TXDR before the
START that clause cannot arise: the byte goes to the shifter at the address's
acknowledge, and each request after it asks for one byte. The event vector's
first TXDR store comes after a 12-cycle entry and about 60 instructions (the
release listing of `isr()`, the flags masked and each tested in turn), past 380
ns, and on any slower core later still, so `interrupt` is outside the window on
every rate this family's APB1 can have in the band. Two more controls show the
letter can fail: a transmit interrupt taken away after `start()` parks a tenure
with SCL low inside the same bound, and a byte changed after its sum was taken
moves the partner's sum by what was changed. Letter `o` was green in 46 runs of
46. No Request is longer than 255 bytes, so no write runs through RELOAD.

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
refused. Under `dma`, over twelve runs of twelve:

| refused | rows | right |
|---|---|---|
| the address | 480 | 480, `i2c_nack_addr` |
| a write of one | 192 | 192, `i2c_nack_data` |
| the first byte of two or more | 288 | 288 |
| the second | 288 | 288 |
| a later byte | 576 | 576 |
| a write of two, the address or either byte (counted above too) | 576 | 576 |

The 1824 rows on the event vector are right too, and the partner's count is the
byte asked for in every row.

How the vector tells them apart, under `dma`, in three cases:
- **The address refused.** The first byte never left TXDR, so TXE is clear
  (23.4.7), and no TXIS rose, so the stream's count stands at its length.
- **A data byte refused, the stream served.** TXE reads clear too, the stream
  having refilled TXDR behind the byte on the wire, and the count has moved.
  SxNDTR is post-decremented when a beat is stored into TXDR (9.3.3), not when
  direct mode preloads it (9.3.7), so it says a refill happened.
- **A data byte refused, the stream not served yet.** The count stands, but the
  byte left: TXE is set. The row that takes the transmit request away after
  `start()` stages this one and answers `i2c_nack_data`, the first byte refused,
  the next request whole.

So TXE set says data, and that alone answers a write of one; with TXE clear, an
unmoved count says the address. One register read and one stream read, on the
NACK path alone. A read half takes no NACK on its data, so a NACK there is the
address's.

And nothing of a failed request reaches the next one. A byte loaded ahead stays
in TXDR through the NACK and the STOP: the next one under either mechanism, or a
first byte stored before an address nobody answered. So the vector empties TXDR
at the NACK: a flush, a store that takes down a TXIS nobody served, and a flush of
that store, on the failed path alone. With that emptying taken out, 10 of letter
`m`'s 18 failures are wrong on the far end when the failed write ran on
`interrupt`, and 4 of 18 when it ran on `dma`: the next write on the event vector
sends the stale byte ahead of its own and drops its own last. A next write under
`dma` flushes before it stores, so it is whole. With it taken out too, the row with
the request taken away never returns, TXIS re-entering the vector the moment TXIE
is armed again. A START lost into a line the partner holds low, with the
first byte stored and the stream armed, answers `i2c_arb_lost`, the stream taken
back and the next request whole, twelve runs of twelve. After an address not
acknowledged the next request is a write in the same serve (the partner's commanded
turn as a target), the partner's sum judging it byte for byte; after a NACKed byte
the partner keeps NACKing to its deadline, so the next request is a frame of the
command channel, whose checksum judges it, and letter `m` judges the same case
byte for byte against I2C1.

**Under the preemptive kernel: `test_stm32f4_rubato_fmpi2c`**, the twin of
[i2c.md](i2c.md)'s `test_stm32f4_rubato_i2c` - the same source over this
peripheral - against the same partner's `twi_peer`: `util/i2c_bus.hpp`'s
arbiter (pending depth 4, the per-bus timeout at 20 ms) over the engined
`FmpI2cHost<1>` on PC6/PC7 (DMA1 streams 5 and 2 on channel 2, the APB1
kernel clock, the Fm+ pad drive on) inside Rubato, the preemptive kernel type,
while TIM2 posts to the highest of the suite's active objects (together, the
pack) every 97, 193 or 389 us and that object's dispatch spins 300 or 3000
cycles. Every letter runs once with TIM2 silent and six times under it; **209
verdicts, green in three runs of `z`, and letters `x` in three and `q` in ten**.
The pack and the letters are the I2C suites' shared body
(`avrxt/src/apps/rubato_i2c.hpp`), and [platform.md](platform.md) places the
suite among the preemptive kernel's.

- **Where a preemption reaches this host: before its one START store.**
  At the speed in force `start()` waits for nothing (a change of speed adds
  `apply()`'s PE cycle); it arms the engines and then writes CR2
  with START last, so a dispatch above the arbiter landing in it delays
  the START. Letter `x`, 400 writes of 16 bytes: `start()` takes 363
  cycles with TIM2 silent (the longest 525), in this image and in a loop of 400
  (the bench's 473 above is another image, and placement and the flash
  accelerator move this figure, so the two are not compared). Under 97 us x 3000
  the interrupt landed inside it 13 to 27 times in three runs, against the 8.3
  that independent ticks would give (400 x 363 / 17460), none with ISR.BUSY set,
  which only the instructions after the START store could see. That is about two
  and a half times the independent figure; the letter times no phase, so why is
  not established.
- **Every shape on the engines, under every pressure as in the control**
  (letter `o`, at 400 kHz against one serve counted to its last byte; a serve is
  the partner's commanded turn as a target):
  the shared body's Requests name no mechanism, and the suite names `dma`
  for every one in a wrapper around `start()`, so a read of one rides its stream,
  a write of one is its store in TXDR alone and a longer write has its stream
  carry the rest. The probe, the reads, the write-then-reads and the
  writes are every one `i2c_ok`, every read byte-exact, the partner's
  count, sum and address matches exact. The 32-byte write 819 or 820 us
  silent and at most 824 under pressure (the classic I2C's 761: this peripheral's
  400 kHz row is a ceiling, its SCL slower, as above), the probe 40 or 41
  and at most 60.
- **Three speeds and the commanded stretch** (letter `p`): a read of 16, a
  1+4 and a write of 16 byte-exact at 100, 400 and 1000 kHz, the write of
  16 in 1589 to 1593, 429 to 431 and 193 to 219 us against the wire's
  1530, 382 and 153 - at Fm+ the partner's polled target stretching each
  byte, as above. The partner's stretch of 1000 us a byte at 100 kHz: a
  read of 8 in 9214 to 9239 us against its nine holds, a write of 8 in
  9002 to 9014 against its eight (the write follows the read at once and
  meets most of one hold more, [i2c.md](i2c.md) has the arithmetic).
- **A NACKed data byte is told from an address not acknowledged, under the
  engines** (letter `q`): the deaf client's probe and its write, the
  write on the transmit engine, answer `i2c_nack_addr` and the next
  tenure - a probe of 0x5D, where the partner parks - `i2c_ok`; the third
  byte of a six-byte write NACKed answers `i2c_nack_data`, and so does
  the last written byte of a 2+4 NACKed, whose two written bytes the
  partner reports received, its address matches the two a host that
  lets go with a STOP makes; a write to 0x7A and one to the general call
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
  BUSY and is answered `i2c_timeout` by the arbiter, in 19990 to
  20026 us from its `start()`, 21 passes of 21, with the flood paced by the
  turns of the suite's console object. The same tenure at 400 kHz after
  commands at 100 kHz pays `apply()`'s PE cycle before its START and is
  answered `i2c_arb_lost` in 26 us, every run. Under the flood (180
  to 722 probes a pass, 721 or 722 of 930 to 942 ticks at 97 us x 3000) every
  request was answered exactly once - while held `i2c_rejected`, and
  `i2c_arb_lost` for nearly every probe that reached the wire after a
  `recover()`, after the release `i2c_ok` or `i2c_nack_addr`; after
  the release the next tenure was
  `i2c_ok` and the arbiter voted idle, every pass.
- **The bus error count** rose by one in each pass of letter `p` and
  nowhere else, seven a run, as on the classic I2C beside it
  ([i2c.md](i2c.md) places it at the write that ends the partner's
  counted serve at 100 kHz). This suite's APB1 kernel clock, a ratio of 1.0, is
  outside ES0298 2.12.8's band.
- **Dark** (letter `i`): every object idle, the arbiter's vote, no time
  event armed, TIM2 stopped with its DMA requests off, the host between
  tenures with TCIE off and both DMA request enables clear, both streams
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
- The general call and the two SMBus addresses as HOST verbs: `start()`
  refuses address 0x00 with every reserved address
  ([../design/i2c-bus.md](../design/i2c-bus.md)); a broadcast is a
  feature of its own, born with its first user. The resource enables the
  target side's recognition of them.
- The recovery LADDER - when to unstick, when to retry, when to take a
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
- The three offers the STM32G0's host takes and this one does not, and the
  stream's completion interrupt (the inventory under "What the silicon does"):
  no reason, and neither is chosen; taking them moves the lengths under "Which
  engine, by the cycles".

Implemented, not bench-verified (each with what would measure it):
- Arbitration lost between two controllers: a race wants two STARTs
  committed inside each other's detection window, and on this bench two
  held STARTs released by one STOP serialize
  ([../stm32g0/i2c.md](../stm32g0/i2c.md), "Two held STARTs released by one
  STOP serialize", measured with this chip's other I2C). What would measure it
  is a peer releasing
  its START by software at a delay swept across this one's. The status's
  own path is measured: a START into a data line the partner holds low
  loses its address and is answered `i2c_arb_lost` (Bench findings).
- ES0298 2.12.3's spurious BERR told apart from a real one: the count
  moved under the preemptive kernel's suite, once a pass at a write that
  ends the partner's counted serve at 100 kHz (Bench findings), which the
  partner's re-init inside that byte's acknowledge would explain as a
  misplaced STOP - a real one, counted the same. A scope on the two lines
  at that write would separate them.
- The noise filters on the wire: the register is written and read back
  and the refusals the filter delay causes are measured, but what a
  9-period digital filter does to a glitch wants a glitch generator.
- ES0298 2.12.8 in target mode, which the erratum names: the client gives
  its first byte on TXIS and nothing has read this peripheral as a target inside
  the band. The partner's `twi_peer` has no `host_read` (twi_link's extended
  repertoire), and that op on the HSI kernel is what would measure it.
- TIMEOUTB, this peripheral's own cumulative stretch, which only a
  target being read produces: the partner's `twi_peer` has no `host_read`
  (twi_link's extended repertoire) to read this peripheral as a target, and
  that op is what would measure it.
- The Fast-mode Plus drive's EFFECT: the two bits are written and read
  back, but the 20 mA sink shows itself in a fall time, which wants an
  oscilloscope and a bus with real pull-ups.
- The instance on the other three part classes that carry it (the F410,
  F412 and F413/F423): the driver compiles for all of them, their DMA
  request tables are refused for want of a manual, and no board here
  carries one.
