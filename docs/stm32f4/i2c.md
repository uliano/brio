# I2C (STM32F4)

This document is for a programmer who needs an I2C bus on an STM32F4: a
sensor, a touch controller, a converter or an EEPROM on two wires, with
this chip as the controller, or this chip answering as a target.
`brio/stm32f4/i2c.hpp` is the chapter of RM0090 in the layers every brio
bus driver has: `I2c<n>`, the resource, one function per register fact;
`I2cHost<n, pins, TxEngine, RxEngine>`, the host task, which
`util/i2c_bus.hpp` drives (a task, in brio, is a driver layer with state of
its own, served from its peripheral's interrupts; `TxEngine` and `RxEngine`
name the DMA streams bound to the host, its engines); and
`I2cClient<n, pins>`, the target role. A program with more than one
active object on the bus (an active object is a component of a brio
program with its own event queue, served one event at a time by the
kernel, [docs/learn/introduction.md](../learn/introduction.md)) puts the
host under the arbiter `I2cBus`, itself an active object, which takes one
request at a time and replies with an event when its tenure is over (a
tenure is one START to its STOP, the bus held for one request;
[design/i2c-bus.md](../design/i2c-bus.md)). This peripheral is the event
machine of the STM32F1's generation, driven by software sequences; the
STM32F410, F412, F413/F423 and F446 also carry a second I2C design,
[fmpi2c.md](fmpi2c.md).

The smallest use, a bus with one device driven from the loop, the
STM32F429I-DISC1's I2C3 and the STMPE811 on it:

```cpp
volatile bool done = false;                // set by the handlers, read by the loop
constexpr brio::I2cPins pins{.scl = {'A', 8, brio::PinFunction::af4},
                             .sda = {'C', 9, brio::PinFunction::af4}};
using Bus = brio::I2cHost<3, pins>;
extern "C" void I2C3_EV_IRQHandler() { if (Bus::isr()) { done = true; } }
extern "C" void I2C3_ER_IRQHandler() { if (Bus::error_isr()) { done = true; } }

// in main(), after the clock task's init():
if (!Bus::init(clock)) {                   // both speed rows solved for PCLK1
    // not even 100 kHz at this APB1 clock: there is no bus
}
const uint8_t index[1] = {0x00};
uint8_t id[2] = {};
Bus::Request r{};
r.addr = 0x41;                             // the 7-bit address, unshifted
r.tx = brio::lend<brio::Lease::reply>(index);
r.tx_len = 1;
r.rx = brio::lend<brio::Lease::reply>(id);
r.rx_len = 2;
done = false;
if (!Bus::start(r)) {                      // false: on the wire, the handlers end it
    while (!done) {}
}
const uint8_t outcome = Bus::status();     // i2c_ok, or what ended the tenure
```

An excerpt: the flag, `pins`, `Bus` and the two handlers sit at
namespace scope, the rest in `main()`. Line by line: `done` is the
program's own completion flag, `volatile` because a handler writes it
and the loop reads it; `pins` names the two pads with the alternate-function
number the datasheet gives I2C3's SCL and SDA on them, which nothing in
the code can check; `Bus` is the host task over I2C3 on those pads, its
two DMA engine parameters left empty, so every byte moves on the interrupt;
`init()` takes the clock task's tag (`constexpr SysClock clock;`,
[clock.md](clock.md)), hands the pads over and solves the two speed
rows, standard and fast mode, for the APB1 clock; the two `extern "C"`
functions are the instance's event and error interrupt handlers, as in
C, under ST's names for the vectors, and each body returns true on the
interrupt that completes the tenure. `init()` answers whether there is
a bus at all, and the empty branch is where a program decides what it
does without one. The request is one tenure: write the
one byte of `index` (the device's register index), then a repeated
START and a read of two bytes into `id`, the last one NACKed, then the
STOP; `addr` is the 7-bit address, and the speed is left at its default,
`standard_100k`. `brio::lend<brio::Lease::reply>(...)` wraps each buffer
as a loan that lasts until the reply ([design/kernel.md, "Payloads: what
travels inside the event"](../design/kernel.md#4-payloads-what-travels-inside-the-event-kernelborrowedhpp)).
`start()` begins the tenure, and its answer decides the wait: false
means the tenure is on the wire and a handler will end it, so the loop
waits for `done`; true means it ended before a byte moved, so there is
nothing to wait for. Either way `status()` then says how it ended.

What decides correctness even there:

- **Pull-ups.** Pull-up resistors on both wires are the board's and not the
  chip's.
- **`init()` after the clock task's.** FREQ, CCR and TRISE are computed from the
  APB1 rate the clock type claims and not read from the hardware, so a bus
  initialized on a chip still at its 16 MHz reset rate under a 180 MHz clock
  type (45 MHz of APB1) runs at about a third of the rate asked - derived from
  the register arithmetic, not measured - and nothing refuses it. `init()`
  answers false only when not even standard mode can be solved for the APB1
  clock (below 2 MHz, above 50 MHz, or a stated rise time TRISE cannot hold).
- **Both vectors bound by the program.** `init()` enables the two NVIC lines, and
  an unbound one is the crt's weak alias of `Default_Handler`, a silent spin,
  where the first event of the first tenure stops the program.
- **The buffers.** `index` and `id` are lent from `start()` to the completion,
  untouched and in scope until then, which the `Borrowed` type records and
  nothing checks. A half of a request (its written bytes or its read bytes) with
  a length needs its buffer: `start()` does not look, and the vectors would read
  or write through a null pointer.
- **One tenure at a time.** `start()` does not look at whether the previous tenure
  has completed; `I2cBus` is what serializes.
- **The address.** `addr` is a 7-bit address of 0x08 to 0x77, and `start()` refuses
  any other before the wire - the general call and the 10-bit prefixes 0x78 to
  0x7B among them - as every host does
  ([../design/i2c-bus.md](../design/i2c-bus.md)). On this peripheral the refusal
  matters more than tidiness: a write's address byte of the form 1111 0xx0 is sent
  as a 10-bit header and the address phase ends on ADD10 instead of ADDR (27.3.3).
  The host ends such a tenure `i2c_bus_error` rather than spin on it ("Bench
  findings"), but only the refusal keeps it off the wire.
- **`start()`'s answer.** True means nothing of the request moved and no interrupt
  follows, so a loop that waited for `done` after a true would wait for ever - the
  excerpt waits only after a false. `status()` then says why: `i2c_rejected` for a
  speed the clock cannot produce, an address outside 0x08..0x77, or `dma` named on a
  host without engines; `i2c_timeout` for the previous tenure's STOP still standing
  after the bound below.
- **`start()` waits in its caller, bounded.** 5 ms for the previous tenure's
  STOP to leave the wire, then 100 us for BUSY to fall, both on the DWT's cycle counter,
  which `init()` turns on, so no ticker is needed. A STOP still standing
  after 5 ms is that `i2c_timeout`, with a per-bus timeout or without one. CR1 stays
  untouched and `recover()` is not needed: the STOP goes out when the client lets go,
  and the next request runs. Each request `start()` answers this way costs its 5 ms in the
  caller, so under `I2cBus` the requests queued behind a held wire add up inside one
  dispatch. A bus still busy after 100 us sets the START anyway, which waits behind
  BUSY (27.3.3), parked until the line is let go, and only then goes out; only an
  arbiter's per-bus timeout answers it sooner, the excerpt's loop having no bound.
  Under `I2cBus` ("How to use it") the
  20 ms it is given fits some 200 bytes at 100 kHz, nine periods a byte: a 255-byte
  request, 23 ms of wire, or a device that stretches wants more.
- **A read that has an effect, under `dma`.** A device whose read moves a FIFO loses what
  leaves it. A read of three or more on a stream ends under LAST, which acts once the stream
  has taken the last byte but one. A stream held off from that byte's arrival through the
  last byte's acknowledge, one byte on the wire (90 us at 100 kHz, 22 us at 400), has the
  last byte acknowledged and one more clocked off the target; the reply is exact and `i2c_ok`.
  Reading such a device, name `interrupt` or read two at a time, if the program's DMA load
  can hold a stream that long: every read under `interrupt` and a read of one or two under
  `dma` hold the wire and clock nothing more at any load ("A load past what the bus takes").
- **The pads first.** The trap of a bring-up that drives the resource by hand is
  that a peripheral configured before its pads are handed over comes up BUSY over
  an idle wire and never gets a START out ("What the silicon does"); `init()` hands
  the pads over first.

## Documents of record

RM0090 Rev 22 ch. 27 (RM0390 Rev 6 ch. 24 and
RM0383 Rev 4 ch. 18 are its twins: one register description, three
manuals), the datasheets' alternate-function tables for the pads,
RM0090 table 43 with its RM0390 and RM0383 twins for the DMA request
mapping, and the errata's I2C items - ES0206 2.10, ES0298 2.11,
ES0287 2.9, ES0321 2.12, the same six on all four parts: "spurious bus error
detection in controller mode", "SMBus standard not fully supported",
"start cannot be generated after a misplaced Stop", "mismatch on the
setup time for a repeated Start condition", "data valid time violated
without the OVR flag being set", "both SDA and SCL maximum rise times
violated when the VDD_I2C bus voltage is higher than ((VDD + 0.3) /
0.7) V". Driver: `stm32f4/i2c.hpp` (`I2c<n>` the resource,
`I2cHost<n, pins, TxEngine, RxEngine>` the host task,
`I2cClient<n, pins>` the other end, the vocabulary `I2cSpeed`,
`I2cDuty`, `I2cTiming`, `I2cFilter`, `I2cAddressConfig`,
`I2cSmbusConfig`, `I2cFlag`, `I2cPins`, `I2cHostConfig`,
`I2cClientEvent`, and the timing arithmetic), over the reserve's
instance, filter and request-mapping facts
(`stm32f4/device_tables.hpp`, the reserve: the table of what each part
has, read off the device header), under the arbiter and the status
vocabulary of [../design/i2c-bus.md](../design/i2c-bus.md), with the
streams from [dma.md](dma.md). The family fixture is
`test/family_stm32f4/i2c.cpp` with the negatives that refuse an absent
instance, one pad twice, a bus with no SDA, a pad on an absent port, an
engine off the request map, an engine on a part class whose manual was
not read, one engine of two, and an engine whose element is not a byte.
Bench: `test_stm32f4_i2c` on the STM32F429I-DISC1, against the STMPE811
touch-screen controller the board carries on I2C3, and on the
32F469IDISCOVERY, against the FocalTech capacitive touch controller of
its MB1166 display board on I2C1 - whose register map is FocalTech's
"Application Note for FT6x06 CTPM" (v1.0, the same document bound into
the FT6236/FT6336/FT6436 series datasheet v0.3) and whose pins and
timings are the FT6x06 datasheet v0.1 - and on the Nucleo-F446RE, I2C1
on PB8/PB9 wired to the same chip's FMPI2C1 on PC6/PC7 (a 2.2 kOhm
pull-up on each line), the FMPI2C1 a register file the suite serves from
its own vectors. A letter of a suite is one command key of the
suite's console and the group of verdicts it runs; `z` is the key that
runs them all, and `t` on the Nucleo-F446RE a second set, run against
a partner board (below).

## What the silicon does

**The F1 lineage's event machine, and it is a set of software
sequences.** CR1/CR2/OAR1/OAR2/DR/SR1/SR2/CCR/TRISE/FLTR, one byte of
buffer, and a tenure that advances only when software runs the sequence
each flag prescribes - SCL held low meanwhile (27.3.2 and 27.3.3). EV5:
SB up, read SR1 and write the address into DR. EV6: ADDR up, read SR1
then SR2. EV8: TxE, write DR. EV8_2: TxE and BTF together, the last
byte is out, STOP or repeated START. EV7: RxNE, read DR. EV4: STOPF,
read SR1 then write CR1. Nothing here is a clear register: the flag and
the act that consumes it are the same instruction, which is why the
driver's ISR body is written phase by phase and not flag by flag.

**The receive procedure depends on the count** (27.3.3's "controller
receiver", and the three procedures it spells out for when the software
cannot keep up):
- One byte: ACK cleared before ADDR is cleared, STOP set right after;
  the byte lands on RxNE.
- Two bytes: POS set with ACK before the address goes out, ACK cleared
  after ADDR; both bytes come off one BTF, and the STOP goes before
  they are read.
- N > 2: RxNE until three remain, then BTF twice - the first clears ACK
  and reads N-2, the second sets STOP and reads N-1 and N.
That is the whole reason the buffer interrupt (ITBUFEN) is switched on
and off during a tenure. On this page the pump is the interrupt that moves
one byte at a time: TxE and RxNE reach the vector only while it wants them, and
BTF, under ITEVTEN alone, carries the tail.

**Up to three instances, two vectors each, all on APB1.** I2C1 and I2C2
on every part of the family, I2C3 on every one but the F410. Each has
an event vector and an error vector, shared with nothing. FREQ, CCR and
TRISE all divide that bus's clock, so a program asks
`apb_hz(clock, false)` and never SYSCLK - 45 MHz against 180 at this
board's core ([clock.md](clock.md)).

**The rate is three registers and a rise time.** CR2.FREQ states the
APB1 clock in whole megahertz and is what the peripheral builds its data
setup and hold times from; below 2 MHz nothing is legal and below 4 MHz
fast mode is not (27.3.2), and above 50 MHz the field has no encoding.
CCR is the period: `2 x CCR` APB periods in standard mode, `3 x CCR` in
fast mode at DUTY 2 and `25 x CCR` at DUTY 16/9. TRISE is the maximum
SCL rise time in whole APB periods plus one, and it exists because the
peripheral checks the SCL input at the end of that window to see whether a
target is stretching: get it wrong and the frequency moves with the
wire's rise time instead of staying put. All three are
enable-protected. 400 kHz comes out exact only where the APB clock is a
multiple of 1.2 MHz at DUTY 2, or of 10 MHz at DUTY 16/9 - which is
what 27.6.8's note about "a multiple of 10 MHz" is really saying.

**BUSY is not a stored bit, and it watches the peripheral's input.**
27.6.7: set "on detection of SDA or SCL low", cleared "on detection of
a Stop condition", and "still updated when the interface is disabled".
The level it watches is the peripheral's own line input, which reads
low while the pad is not in its alternate function - so a peripheral
configured before its pads comes up BUSY over an idle wire, and since
27.3.3 makes a START wait for BUSY to fall and no STOP is ever going to
arrive, the first tenure never leaves. Measured: at reset, with both
pads reading HIGH as plain inputs, SR2 reads BUSY. The driver's order -
pads first, then SWRST, then the timing and PE - is the answer, and
27.6.1 names SWRST for exactly this ("if the BUSY bit is set and
remains locked due to a glitch on the bus").

**The noise filters are not on every part.** I2C_FLTR - the analog
filter's off switch (it is ON at reset) and a digital filter of up to
15 APB periods - exists on every part of the pack but the F405 class,
and the device header says which: the register is not in that class's
I2C_TypeDef at all, and its bit definitions are gone with it, so the
two accesses that touch it are selected by the preprocessor while the
presence fact is the reserve's. Table
122's ceiling on the digital filter per APB band and mode is published
as arithmetic and is advice: it is what keeps the data hold time inside
the bus specification with the analog filter off, and 27.3.5 itself
allows more to a system that can take the violation.

**Errors, and what a controller must do about each** (27.3.4). A NACK
(AF) leaves the STOP to software. Lost arbitration (ARLO) has already
released the lines and dropped the peripheral back to target mode. A bus
error (BERR) in controller mode leaves things as they were, "the lines are
not released and the state of the current transmission is not affected", so it
is not a reason to abort anything. Overrun and underrun (OVR) are a target's with the
clock stretch given up. All seven error bits are rc_w0: a zero written
into the bit.

**Three errata shape the code, and three more are the application's.**
- *Spurious bus error detection in controller mode* (ES0206 2.10.1).
  BERR can be raised where there was none. Together with the manual's
  own statement above, that makes a controller's BERR a thing to clear
  and count and nothing else - `spurious_bus_errors()` is the counter -
  and it is where this family's host differs from the CH32V00x's,
  whose chapter has no such erratum.
- *Start cannot be generated after a misplaced Stop* (2.10.3), whose
  workaround is SWRST - which is what `recover()` is, so a tenure that
  never answers and is timed out by the arbiter comes back through the
  errata's own escape.
- *Mismatch on the setup time for a repeated Start condition* (2.10.4):
  in standard mode between 88 and 100 kHz the setup time can be
  violated, independently of the APB clock, when the target stretches
  the clock or SCL rises in more than 300 ns. A write-then-read Request
  is a repeated start, so the fact is published beside the bus
  (`repeated_start_setup_at_risk`) rather than hidden; the workaround -
  fast mode, or a standard-mode rate below 88 kHz - is the
  application's to take.
- *SMBus 2.0 is not fully supported* (2.10.2): the peripheral cannot NACK an
  invalid byte or command, so an SMBus host wants the alert pin, the
  alert response address or the host-notify protocol above it.
- *Data valid time violated without OVR being set* (2.10.5): a target
  transmitter with NOSTRETCH set can send a byte late and wrong with no
  flag raised. The client here keeps the clock stretch by default.
- *Both rise times violated above ((VDD + 0.3) / 0.7) V* (2.10.6): a
  5 V I2C bus on a 3.3 V part is out of specification - a board fact.

**The DMA requests are a pair per instance, all on DMA1**, and they
serve a phase of any length (27.3.8): the transmit stream is fed on TxE;
a receive run of two or more ends by itself under CR2.LAST - the peripheral
NACKs the byte after the controller's EOT_1 signal - with the STOP in
the stream's completion handler; and a one-byte receive has a sequence
of its own in the same section, "the NACK must be programmed during EV6
event, i.e. program ACK=0 when ADDR=1, before clearing ADDR flag. Then
the user can program the STOP condition either after clearing ADDR
flag, or in the DMA Transfer Complete interrupt routine". ITBUFEN must
be off while a DMA request serves the same direction.

**What the host takes from the chapter, and what it declines.** Every
item that bears on a tenure's cost, at 180 MHz with APB1 at 45 MHz - the
silicon's own timings below measured on the STM32F429I-DISC1, the STOP's
drain on the Nucleo-F446RE:
- *One data register, no FIFO, no byte counter, no automatic STOP.* One
  interrupt a byte is this peripheral's, on the pump; the figure is the
  handler's cycles a byte (bench findings).
- *EV8_1 takes two bytes at once.* TxE rises 17 cycles after ADDR's clear
  and again 20 cycles after the first byte's store, as that byte moves
  into the shifter: the entry that clears ADDR loads the data register
  and the shifter itself, so a write of one or two bytes, and a register
  index, take no TxE interrupt at all. Used on the pump, under
  `interrupt`; under `dma` the transmit stream takes those bytes too.
- *The data register before the START.* Declined: at EV5 DR is where the
  address goes, so nothing can wait in it across the START.
- *The repeated START during the last written byte.* "Setting the START
  bit causes the interface to generate a ReStart condition at the end of
  the current byte transfer" (27.3.3's note): requested on the TxE that
  says the last byte went into the shifter, the bus turns around with no
  stretch of the host's. Used, with a dummy store into DR right after the
  request. With DR left empty, BTF rises at the end of that byte and stands
  until the Sr is out - 2950 cycles at 100 kHz and 700 at 400, measured -
  a level that would hold the event vector for all that time. With the dummy
  in DR, BTF never rises and the wire carries the index's nine clocks and the
  Sr's own one and nothing more. The Sr raises SB as it would have, and the
  address stored at SB replaces the dummy: the register read back is the
  one asked for (measured, polled; ST's library also stores into DR for this,
  I2C_Flush_DR). A NACK of that last byte arrives
  with the START still pending, and is answered `i2c_nack_data` - but
  nothing is written at the NACK: 27.6.1 forbids any CR1 access while
  START stands ("a risk of setting a second STOP, START or PEC request"),
  and 27.3.4 lets a NACKed controller answer with a repeated START. The
  controller sends the Sr, and its SB is answered with the address and
  the write bit and a STOP behind it - a void write, one address byte on
  the wire, only when a device NACKs a byte (the bench findings measure the
  alternative, a one-store withdrawal, and its hang).
- *START cleared by software - a requested start withdrawn* (27.6.1's
  START bit). Declined: behind a refused last byte it is a CR1 access
  while START stands, which the same register's note forbids, and on the
  wire it hung the event vector (the bench findings).
- *The STOP on the TxE of the last byte* (27.3.3: "when either TxE or BTF
  is set"). Declined: nothing marks a controller's own STOP leaving
  (STOPF is the target half's), so a STOP requested a byte early would
  give no event to complete on but that byte's TxE, and the next start()
  would wait the whole byte. The write ends at BTF (EV8_2).
- *The event vector, a level.* BTF stands after a write's STOP is
  requested until that STOP is on the wire ("TxE and BTF are cleared by
  hardware by the stop condition", figure 243), and a vector left on
  re-entered the handler for all that time: sixteen extra entries a
  write tenure at 100 kHz and three at 400, the thread starved meanwhile
  (measured). ITEVTEN is the tenure's: raised by start() and taken down
  at the completion, with the buffer vector and the DMA requests, in one
  CR2 store (`control2()`). A phase on the DMA streams keeps it down, from ADDR's
  entry to the stream's completion: one bit left out of the store that raises DMAEN,
  because a late stream holds BTF up with nothing for the vector to do ("A load past
  what the bus takes" below).
- *The STOP's drain is the next tenure's.* With no event for the
  controller's own STOP, and CR1 untouchable while STOP stands (27.6.1),
  start() waits for it, in the caller (the arbiter's dispatch). The bound is
  5 ms; a tenure started at once behind a completion pays about a bit period:
  1848 to 1884 cycles at 100 kHz and 323 to 376 at 400 (`bench_stm32f4` letter
  `i`). The alternatives were weighed and
  declined: a deferred START needs an event the chapter has not got, and
  a timer of the bus's own would cost a timer and an interrupt a tenure
  to free about ten microseconds at 100 kHz and two at 400.
- *The three receive procedures* (one byte, two with POS, N with the BTF
  pair). Used. The faster EV7_1 path - RxNE to the end, the NACK and the
  STOP programmed on the second-to-last RxNE - declined: it must complete
  before the last byte's acknowledge, a deadline no handler of a program
  guarantees, and 27.3.3 gives the BTF procedures for exactly that case.
- *The DMA requests and LAST.* Used for every phase of a Request that
  names `dma`, one byte included, a read of two excepted, and a read's end is the stream's ("A read
  under `dma` ends on its stream", in the bench findings). The transmit stream is started in the
  ADDR entry, before ADDR's clear, with DMAEN raised behind it;
  the receive stream at the read's SB, before the address goes out, so
  the entry that arranges ACK and POS for the read knows which procedure
  it arranges them for. A receive run of three or more runs under LAST, EOT_1 being the
  stream's taking of the last byte but one, so a stream late there has the last byte
  acknowledged; a read of two takes the chapter's two-byte procedure on the event vector, no
  stream, because there that lateness is the whole read and the procedure holds the wire
  whatever the load, one entry for the stream's completion; a
  run of one under its own sequence: ACK down at the SB, as the pump's,
  and the STOP after ADDR's clear, the first of 27.3.8's two places for
  it (ST's library takes the second, the completion). The transmit
  stream's completion is 27.3.8's EOT: the requests go off, and the event
  vector, raised there, ends the write on BTF - or, with a read half to follow, the
  buffer vector is armed for the TxE behind the last byte, where the
  repeated START is requested as on the pump. A stream is this host's
  alone ([../design/dma.md](../design/dma.md), the ownership rule), so
  the host does not ask it whether it started.
- *PEC, SMBus, the 10-bit header, the noise filters' effect.* No tenure
  shape asks for them (the gap lists below).
- *The FMPI2C* of the F410/F412/F413/F446 is the STM32G0's I2C, its own
  document ([fmpi2c.md](fmpi2c.md)); on the Nucleo-F446RE it is this
  peripheral's partner on the self-link, and the host each makes is
  priced against the other in `bench_stm32f4`'s letter `i`.

## Types and verbs

**The resource, `I2c<n>`.** `number`, `on_apb2`, `event_irq`,
`error_irq` and `has_filter` are its facts. `bus_clock` (both
directions) and `reset` are the gate and the RCC reset line. `timing`
writes FREQ, CCR and TRISE with PE clear and reads them back; `filter`
writes and reads I2C_FLTR, refused where the part has none, while PE
stands, or past DNF's four bits; `addresses` writes OAR1 and OAR2 from
an `I2cAddressConfig`, refused when the address does not fit its mode
or when a ten-bit address is paired with a second one, and it keeps
OAR1's bit 14 at one as 27.6.3 asks. `enable` / `disable` / `enabled`
are PE; `software_reset` is the SWRST pulse. The control bits are verbs
one for one: `start`, `starting`, `stop`, `stopping`, `ack`, `pos`,
`general_call`, `no_stretch`. SMBus is `smbus` (an `I2cSmbusConfig`),
`alert`, `pec`, `pec_transfer`, `pec_pending`, `pec_value`. The DMA
requests are `dma(on, last)` and `data_address`. Data and status are
`data` (both directions), `status1`, `status2`, `flag`, `busy`, `host`,
`transmitting`, `second_address_matched`, `general_call_matched`,
`smbus_host_matched`, `smbus_default_matched`; the clearing sequences
are `clear_addr` (SR1 then SR2, returning SR2), `clear_stopf` (SR1 then
a CR1 write) and `clear_errors`. The three interrupt enables are
`event_interrupt`, `buffer_interrupt`, `error_interrupt`, and `control2`
writes CR2 whole - FREQ, the three enables, DMAEN and LAST - in one
store.

**The arithmetic.** `I2cSpeed` is `standard_100k` or `fast_400k` - this
peripheral has no Fm+. `I2cDuty` is `ratio_2` or `ratio_16_9`.
`i2c_timing_for(pclk, speed, duty, rise_ns)` solves an `I2cTiming`
(FREQ, CCR with its F/S and DUTY bits, TRISE) or refuses;
`i2c_scl_hz(pclk, timing)` says what a solved row really produces;
`i2c_speed_hz` is the nominal rate; `i2c_digital_filter_max(pclk,
speed)` is table 122; `i2c_repeated_start_at_risk(speed, scl_hz)` is
ES0206 2.10.4's window. `i2c_address_config_valid` and
`i2c_pins_valid` are the two refusals a configuration can fail at
compile time.

**The pads, `I2cPins`.** SCL, SDA and an optional SMBA, each a `PinSel`
carrying the alternate function the datasheet gives that signal on that
pad - AF4 for all three instances, AF9 on the handful of pads the
tables put there. Both lines go out open drain at `very_high`.

**The host task, `I2cHost<n, pins, TxEngine, RxEngine>`.** `Request`
is `{addr, tx, tx_len, rx, rx_len, reply, speed, engine}`, the tenure
shape `util/i2c_bus.hpp` describes, the speed an `I2cSpeed` of this
peripheral's two rows (no Fast-mode Plus): one bus tenure that is a write, a
read, a write-then-read with a repeated START, or the empty probe.
`engine` names what moves the data, and the host never chooses:
`interrupt`, the default, the vectors at any length; `dma` the DMA
engines at any length, one byte included (27.3.8, above), on a host with
both engine parameters named - `has_dma` says so at compile time, and `dma` on
a host without them is answered `i2c_rejected` inside `start()` with
nothing on the wire. The field sits in the padding after `speed`: the
Request is 28 bytes with it and without it, a `static_assert` beside it
says so. "Which engine, by the cycles" below gives the length from which
`dma` costs the core fewer cycles. A buffer in the core-coupled memory cannot
go on the engines: no DMA master reaches it, so a request naming `dma` over one
should end `i2c_dma_fault` (read from [dma.md](dma.md)'s measured transfer
error, not staged on I2C). `init(clock, config)`
takes an `I2cHostConfig` - the duty shape, the two rise times and the
filters - and solves both speed rows; `rebase(hz)` solves them again
after a clock change. `speed_ok`, `timing_of`, `scl_hz`,
`reference_hz`, `digital_filter_max` and
`repeated_start_setup_at_risk` are what it will answer about the rate;
`spurious_bus_errors` and `clear_spurious_bus_errors` are the errata's
counter. `start` begins a tenure and answers false whenever the wire
moves; `status` is the completion code; `isr` and `error_isr` are the
two vectors' bodies and `dma_isr` the streams'. `unstick` clocks a
stuck client off SDA and `recover` puts the peripheral back where
`start` is legal; `release` stops the streams, disables the peripheral, gates
its clock with no reset pulse and gives the pads back - `init` starts from
the peripheral's RCC reset, which withdraws a request a tenure left raised (the
release contract, docs/design/dma.md) - and `claim_smba_pad` hands over the alert pad a
program in SMBus mode wants. Both engines arm at DMA priority `high`: the host
does not overrun - a byte its streams have not moved leaves TxE or RxNE
up and BTF rises, "stretching SCL low" until DR is served (RM0090
27.3.3), so a late stream slows the bus and loses no byte (a read of three or more late
across its last two does clock one more off the target, "A load past what the bus takes" below). A
read of one has no next byte to hold: its wire ends by itself and the answer follows when the stream is served,
and the arbiter's timeout answers for one never served ("A read under `dma` ends on its stream" below).
`I2cClient::release` gates the peripheral the same way, its `init` starting from
the reset.

**The client task, `I2cClient<n, pins>`.** `init(clock, addresses,
no_stretch)`, then the polled surface `addressed`, `answer_address`
(which returns the direction), `second_address_matched`,
`general_call_matched`, `data_ready` / `take`, `data_wanted` / `give`,
`acknowledge`, `stop_seen` / `clear_stop`, `host_nacked` /
`clear_nack`, `overrun` / `clear_overrun`, `flags`, `host_reads`; and
the two ISR bodies `service` and `error_service`, each answering one
`I2cClientEvent`.

## How to use it

A bus with one device, driven from the loop, is the smallest use above, the
shape a bring-up starts from.

The same bus under the kernel's arbiter, which is what a shared bus
wants: the reply comes back as an event and the buffers are lent until
it does. An excerpt: it assumes `pins`, `Bus` and the platform `P` of the
smallest use, and an active object `MyAo` that handles `I2cDone` and holds
`index` and `id` as static members; `BusPassThrough` is the failure policy that
hands every status to the requester and retries nothing. The two handlers
replace the smallest use's:

```cpp
using Arb = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;
extern "C" void I2C3_EV_IRQHandler() {
    if (Bus::isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}
extern "C" void I2C3_ER_IRQHandler() {
    if (Bus::error_isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}

// in a handler of MyAo:
brio::post<Arb>(Bus::Request{.addr = 0x41,
                             .tx = brio::lend<brio::Lease::reply>(index), .tx_len = 1,
                             .rx = brio::lend<brio::Lease::reply>(id), .rx_len = 2,
                             .reply = brio::reply_to<MyAo, brio::I2cDone>(),
                             .speed = brio::I2cSpeed::fast_400k});
```

An address scan is the empty Request, and its answer is the status. An excerpt,
`a` the address the loop has reached:

```cpp
brio::I2cHost<3, pins>::Request probe{};
probe.addr = a;                            // both spans empty
// i2c_ok: somebody answered.  i2c_nack_addr: nobody home.
```

A faster bus, a shaped duty and a wire whose rise times are known -
what an `I2cHostConfig` is for, and the speed travels per request so
one bus can serve a 100 kHz sensor and a 400 kHz converter. An excerpt
in the smallest use's setting, its `init()` replaced and its request
given a speed:

```cpp
if (!Bus::init(clock, {.duty = brio::I2cDuty::ratio_16_9,
                       .standard_rise_ns = 400, .fast_rise_ns = 120,
                       .filter = {.analog = true, .digital = 1}})) {
    // not even 100 kHz at this APB1 clock and rise time: there is no bus
}
r.speed = brio::I2cSpeed::fast_400k;       // refused by start() if unreachable
```

With the DMA engines, on the cells the request mapping gives the
instance (the engines' arguments are controller, stream and channel, and a
cell outside `i2c_dma_placements()` does not compile), a Request that names
`dma` moves every phase without the CPU,
one byte included; one that names nothing still runs on the vectors. The
smallest use again, an excerpt in its setting (`done`, `pins`, the two I2C
vectors, `index` and `id` as there), the streams' two vectors added and
the request naming its engine. `Dma<1>::init()` resets the controller, so it
runs once, before the first engine of any driver that shares it; under the
arbiter the streams' vectors post `TransferDone` as the I2C vectors do:

```cpp
using Bus = brio::I2cHost<3, pins, brio::DmaTxEngine<1, 4, 3>,
                                   brio::DmaRxEngine<1, 2, 3>>;
static_assert(Bus::has_dma);               // or `dma` is answered i2c_rejected
extern "C" void DMA1_Stream4_IRQHandler() { if (Bus::dma_isr()) { done = true; } }
extern "C" void DMA1_Stream2_IRQHandler() { if (Bus::dma_isr()) { done = true; } }

// in main(), after the clock task's init():
brio::Dma<1>::init();
if (!Bus::init(clock)) {
    // not even 100 kHz at this APB1 clock: there is no bus
}
Bus::Request r{};
r.addr = 0x41;
r.tx = brio::lend<brio::Lease::reply>(index);
r.tx_len = 1;
r.rx = brio::lend<brio::Lease::reply>(id);
r.rx_len = 2;
r.engine = brio::I2cEngine::dma;           // the index and the two bytes on the streams
done = false;
if (!Bus::start(r)) {
    while (!done) {}
}
```

The recovery ladder, which is two verbs and a decision:

```cpp
Bus::recover();      // the PERIPHERAL: SWRST, the timing again, PE back
Bus::unstick();      // the WIRE: nine clocks and a STOP by hand;
                     // 0 = it was free, 0xFF = SDA is still low
```

## Bench findings

`test_stm32f4_i2c` on the STM32F429I-DISC1, I2C3 on PA8/PC9 against the
board's STMPE811 touch controller. `z` holds 59 verdicts here. 58 have run on
this board, and the other, letter `c`'s refusal of the reserved addresses, on the
Nucleo-F446RE alone ("Not covered yet").

- **The chapter's reset values are exact** - every register zero but
  TRISE, which is 0x0002 - and the peripheral clock is off at reset.
- **BUSY stands at reset over an idle wire.** Both pads read HIGH as
  plain inputs and SR2 reads BUSY, because the peripheral's own line
  inputs are low while the pads are not in their alternate function.
  Once `init()` has handed the pads over and pulsed SWRST behind them,
  BUSY reads the wire and is clear. This is the trap of the chapter for
  a bring-up: a peripheral configured before its pads never gets its first
  START out, and no error is raised anywhere.
- **The timing rows at 45 MHz of APB1.** Standard mode is EXACT: CCR
  225, TRISE 46 (1000 ns in whole APB periods plus one), 100000 Hz.
  Fast mode is not: 45 / 1.2 is 37.5, so CCR is 38 and the rate is
394736 Hz - rounded up in CCR, which is down in frequency, a bus
  never running faster than asked. The 16/9 duty at CCR 5 gives
  360000 Hz. FREQ reads 45 on every row.
- **SCL on the pad, counted while a 32-byte DMA-carried read runs.**
  The tenure is 35 bytes and 315 clocks, and the pad gives 315 rising
  edges (316 when the polling loop is still watching as the STOP's own
  rise comes). The shortest gap between two edges and the mean over the
whole tenure bracket the bit rate - the mean counts the two address
  phases, whose software sequences stretch SCL low, and the shortest is
  sampled by a polling loop that timestamps a little before the edge it
  sees, in core cycles at 180 MHz:

  | row | CCR | states | shortest gap | mean gap |
  |---|---|---|---|---|
  | Sm 100k | 225 | 100000 Hz | 1730 = 104046 Hz | 1803 = 99833 Hz |
  | Fm 400k duty 2 | 38 | 394736 Hz | 442 = 407239 Hz | 456 = 394736 Hz |
  | Fm 400k 16/9 | 5 | 360000 Hz | 470 = 382978 Hz | 501 = 359281 Hz |

  What the arithmetic states sits inside each bracket - in the fast row
  the mean is the stated rate, the address phases' stretch now under a
  core cycle a clock. The mean is stable run to run; the
  shortest gap moves with the sampling phase (366 to 442 in the duty-2
  row, 413 to 489 in the 16/9 one, across runs of one image) by more
  than the two fast rows differ, which is why the verdict is on the order
  of the three means and not on any one number.
- **The address scan.** Of the 112 addresses from 0x08 to 0x77 exactly
  one answers - 0x41, the STMPE811 the schematic names - and every
  other comes back `i2c_nack_addr`, none any other way. The scan is
  repeatable and a NACK leaves the bus fit for the next tenure.
- **The device answers.** CHIP_ID at 0x00 reads 0x0811 as two bytes MSB
  first through a write-then-read tenure, ID_VER at 0x02 reads 0x03,
  and eight tenures in a row give the same bytes.
- **The three receive procedures agree byte for byte.** The same eight
  registers read one at a time, two at a time (POS and the single BTF)
  and eight in one tenure (RxNE, then the BTF pair at the tail) give
  the same eight bytes; a sixteen-byte read is one tenure and ends
  `i2c_ok`.
- **A register written reads back exactly**, and the device's own soft
  reset goes out over the same bus and brings it back. (One pattern of
  that register the DEVICE masks - a device rule, not a bus one.)
- **The kernel's arbiter is unchanged on this architecture.** Four
  tenures through `I2cBus` give four `i2c_ok` replies; a probe of an
  address nobody answers gives `i2c_nack_addr` to the requester
  untouched; six posted into a four-deep queue are all answered, the
  overflow rejected immediately; an idle bus votes for the sleep.
- **The DMA engines carry both phases, at any length.** An eight-byte
  write-then-read through DMA1 streams 4 and 2 on channel 3 is byte-exact
  against the same read on the pump, and a two-byte write through the
  transmit engine reaches the device. On the Nucleo-F446RE (below) the
same letter shows the mechanism each Request names on the streams
  themselves: under `dma` the one-byte index, a one-byte read and a
  one-byte write each start their stream - its memory address, zeroed
  before the tenure, names the tenure's buffer after it - and enter its
  vector once; under `interrupt` the engined host starts no stream; and
  `dma` on the engineless host is answered `i2c_rejected`, CR1 and CR2
  untouched and MSL clear, the next read on that host exact.
- **SWRST puts the whole register file back**, the configuration
  included (CCR 0x00E1 -> 0, OAR1 -> 0, PE down), which is why
  `recover()` rewrites the timing behind it; the bus works immediately
  after. `unstick()` on a healthy wire returns 0 and drives nothing,
  and the pads come back to the peripheral.
- **No bus error was seen** in the whole suite, spurious or otherwise.
- **The target half, against a controller on another board.** As the
  instrument of another target's I2C suite, `I2cClient<1>` on PB8/PB9
  answered a foreign controller at its own address
  over tens of thousands of tenures in both directions - 25100
  tenures of sixteen bytes in ten seconds at fast mode with no error at
  either end - and 27.3.3's BTF stretch was shown to fall AFTER the
  ninth pulse: a twenty-byte read closes on the controller's NACK, which
  it could not if the target held the clock before it. Two things a
  program over this half must know. **A byte written into DR outside a
  read tenure wedges the next one**: the target holds SCL low for ever
  with SR1 reading zero, SR2 showing BUSY and TRA and DR full, and
  nothing short of the peripheral's RCC reset lets the line go - so a give
  pump must follow the direction of the last address match and not TxE,
  which stands between tenures too. And the byte the shifter is asked
  for one ahead survives a tenure even when TxE stands, so the PE cycle
  that drops it (27.6.1, the address and the timing surviving - measured)
  belongs at the end of every read, with PE held down for a few bus
  cycles and no longer: two adjacent stores are one bus cycle, and a
  microsecond-wide window makes the target deaf to a controller that
  opens its next tenure at once.

`test_stm32f4_i2c` on the Nucleo-F446RE, I2C1 on PB8/PB9 against the
chip's own FMPI2C1 on PC6/PC7 - two wires and a 2.2 kOhm pull-up on each
line - served from its own vectors as a register file (an identity at
0x00, a reset command at 0x03, a scratch register at 0x04, a pointer the
first written byte sets), at 0x2C. `z` is 77 verdicts: every letter of
the DISC1's runs against it, and four more - with a partner board, an
STM32G0 on the same two nodes, on its dark console (a program that stays off
the bus), since letter d's
scan would otherwise meet the partner's command address. A second set,
`t`, is the partner running `twi_peer`, the suites' scriptable target, commanded
in band over the bus under test: letter `n`, 13 verdicts, green in every run of
`t` (below).

- **The scan finds 0x2C alone** of the 112 addresses; the three receive
  procedures agree byte for byte, the scratch register is written and
  read back and the target's reset command restores it, the kernel's
  arbiter and the DMA engines (DMA1 streams 6 and 0 on channel 1, RM0390
  table 28) carry the same tenures, and the recovery verbs behave as on
  the DISC1. SCL on the pad: 1803, 458 and 503 core cycles of mean gap
  for the three rows, the stated rates within the brackets.
- **The last byte NACKed** (letter `m`): the FMPI2C1 target refusing the
  last of one to four written bytes of a write-then-read - its NACK bit
  set as the byte before is taken (23.7.2) - four times a count, through
  the pump and the engines, at 100 and 400 kHz, the core at 180, 48 and 16
  MHz (a DynamicClock over the three, the console, the tick and the two
  hosts its users). The driver closes from the Sr: 192 refusals of 192 answered
  `i2c_nack_data`, the controller let go (START and STOP down, MSL clear,
  2 ms on) and the next tenure `i2c_ok`, the target never late, at all
  three cores; and every count acknowledged whole, byte-exact. The alternative, a
  one-store withdrawal at the NACK, runs clean at 180 MHz and at 48 MHz and
  100 kHz, then at 48 MHz and 400 kHz through the pump hangs the program - the
  event vector re-entering without end, SR1 SB, SR2 MSL and BUSY, CR1 PE alone,
  CR2 the event and error enables, the host in its address phase of the next
  tenure - twice in two runs. The core
  switches are `rebase()`'s fan-out - the two hosts are the DynamicClock's
  users - so these tenures at 48 and 16 MHz run on the timing it
  re-derives; how fast is set `t`'s question (below).
- **`rebase()` derives the APB1 rate** from
  the new SYSCLK (`apb_hz_at`, [clock.md](clock.md)): the fan-out runs
  before the RCC moves, so a divider read back there is the rate being
  left. With the divider read back, a switch from 180 to 48 MHz solves FREQ, CCR
  and TRISE for 12 MHz where 24 runs, and the partner's tenures go at
  **1935 thousandths** of 100 kHz (and 1839 at 16 MHz) - byte-exact, which
  letter `m` cannot see. Derived, **989, 982 and 958
  thousandths** at 180, 48 and 16 MHz, in every run of `t`. The letter reads FREQ and
  CCR back against each rung's APB1 and judges the write never faster than the rate they
  make (1000 thousandths at all three: a period counts 2 x CCR periods of PCLK1 at least,
  24.3.3) nor below a quarter under the rate asked (the framework's window for an I2C
  clock, [overview.md](../design/overview.md)); how much slower inside it - the
  request's own cost and the partner's polled target - is printed.
- **Clock stretching is flow control, against the partner** (set `t`,
  letter `n`): its target holding SCL 2 ms before each byte stretches an
  8-byte write to **16 ms** (eight holds inside the controller's window;
  0 ms on the tick unstretched), and the tenure completes `i2c_ok` - the
  host simply waits.
- **`unstick()` against a client that really holds SDA**: the partner
  holding SDA from its own GPIO until four SCL falling edges (latched by
  its EXTI) is freed at the **fourth pulse**, and `unstick()` answers 4;
  held to its deadline, nine pulses and a STOP leave it low and
  `unstick()` answers **0xFF**, and on the freed wire 0.
- **The arbiter's per-bus timeout over this host**: a tenure the
  partner stretches 40 ms at its address is answered `i2c_timeout` by
  `I2cBus` at its 20 ms limit (20 ms on the tick), and the
  same host carries the command channel again after `recover()`.

- **`start()`'s two bounds are what they say** (letter `q`, ten stages a case, fifteen
  runs). The far pads of the self-link, driven low from here, hold a STOP standing and
  hold BUSY up, and the wall of `start()` is read on the counter the case leaves running:

  | case | the 5000 us drain | the 100 us bus wait |
  |---|---|---|
  | the ticker running | 5001 us | 101 us |
  | SysTick stopped | 5001 us | 101 us |
  | the DWT's counter stopped, by a probe or a program | 12504 us | 251 to 252 us |

  The counter is one load and a difference exact over this span, it runs with no ticker,
  under a mask and in a handler, and whatever lands inside the wait counts. A stopped
  counter leaves the budget: the time asked at 8 cycles a poll, each poll taking some 20
  here, so the wait ends about two and a half times late, and it always ends. A bus already
  free takes the first look alone, in line in `start()`: one load, a shift and a branch per
  wait, in the release listing, and `poll_for_us()` is out of line and cold.

- **A client that holds SCL past the drain, a request behind it** (letter `s`, ten passes
  a row, fifteen runs, on an arbiter with a 20 ms timeout and on one with none alike). The
  far SCL pad, pulled low from the completion's entry, stands for a client that stretches
  after a written byte's acknowledge, as this block's own client does (24.3.2):

  | SCL held from the first tenure's completion | the requests behind it |
  |---|---|
  | 8 ms, one behind | `i2c_timeout` 5006 us behind that completion, nothing of it on the wire |
  | 30 ms, three behind | `i2c_timeout` each, in order, 15011 us for all three in one dispatch |
  | 2 ms, let go inside the bound | `i2c_ok` 3007 us behind that completion, its byte in the register |

  Judged: each answer no earlier than the time asked (start() serves it at least, letter
  `q`); what it takes past that is code with no written bound, printed. In every row the
  first tenure is `i2c_ok` 285 to 287 us from the post, and with no
  `init()` and no `recover()` a read finds the register as the answered requests left it
  and a write and its read-back are exact. FMPI2C1 as the client does not reach this path:
  its hold at its address, 8 ms, delays the first tenure's own completion to 8007 us, no
  STOP stands, and the request behind it is `i2c_ok`. Judged: the completion no earlier than
  the 8000 us the hold lasts (ADDR holds SCL until its vector clears it, 23.4.8); the 7 us
  past it, two vectors and the acknowledge's clock, is printed.

**A read under `dma` ends on its stream** (`test_stm32f4_i2c` letters `o` and `p` on the
self-link, ten and fifteen runs, 180 MHz with 45 MHz on APB1). In that phase the event vector
is down, and `dma_isr()` ends the tenure when the receive stream's count reaches zero;
a run of three or more has its STOP requested there (27.3.8), a run of one at ADDR's clear. A
read of two has no stream and ends on the event vector's BTF.
Three other things can answer while the phase stands, and none says `i2c_ok`: a transfer error
(`i2c_dma_fault`), lost arbitration (`i2c_arb_lost`) and the arbiter's timeout (`i2c_timeout`).
A NACK cannot, this end sending the acknowledges. So the question the
[FMPI2C](fmpi2c.md) settles at its STOPF entry does not arise here: a transfer is the load,
the store and the count, in that order (RM0390 9.3.3), and the flag rises at the count's
zero (9.3.14), so a reply never stands ahead of its last byte. What a late stream changes is
the time of the answer.

The load is [fmpi2c.md](fmpi2c.md)'s, "A load makes a stream late by itself", on the streams
of DMA1 this host does not hold (1, 2, 3, 4, 5 and 7). Reads of one, two and eight, 1500 each
at 400 kHz and 500 each at 100 kHz, a row stopped after three reads with no answer in a run:

| load | beats in 100 us | wire over ahead of a read of one's completion, 400 kHz, 100 kHz | past the letter's first bound, reads of all lengths | no answer in fifty times that |
|---|---|---|---|---|
| nothing else | 0 | 0, 0 | 0 | 0 |
| six streams every 32 | 1690, 1691 | 0, 0 | 0 | 0 |
| four every 17 | 2029 to 2118 | 0, 0 | 0 | 0 |
| three every 13 | 2070 to 2073 | 0, 0 | 0 | 0 |
| four copies | 0 | 0, 0 | 0 | 0 |
| two every 16, and the copies | 1126 | 0, 0 | 0 | 0 |
| six every 48, and the copies | 1126, 1127 | 0, 0 | 0 | 0 |
| two every 8, and the copies | 1500 | 0, 0 | 0 | 0 |
| three every 16, and the copies | 1689 to 1701 | 1499 of 1500, 491 of 500 | 796 of 6000 | 0 |

Every read answered was exact and `i2c_ok`, 54000 in all, with the stream still once the reply
stood. A stream late on a run of three or more stretches SCL until it takes the byte ("stretching
SCL low", 27.3.3); on a read of one the wire ends by itself and the answer follows when the
stream is served, the "wire over" column. The first bound is twice the wire and three ticks and
the last fifty times that; past it the letter answers as an arbiter's timeout would, with
`recover()`, and the next read is whole. The event vector took two entries a read on a stream, SB
and ADDR, at every load, and three a read of two, BTF the third.

**A load past what the bus takes is the case that decides** (letter `r`, run by its own key and not
under `z`, fifteen runs). Three streams every 5, asking 5400 beats in 100 us against the bus's 2250: the
stream is not served at all while the load stands. A read of one's wire ended with the byte in DR, 1500 of 1500. Nothing answered ahead of the
stream, 0 of 1500, and when the load came off the read and the read of eight behind it were exact,
1500 of 1500 each. Three every 12 and the copies, 2243 to 2247 beats: 2998 of the 3000 reads of one had the
wire over first, 3410 of 9000 reads of all lengths ran past the first bound, all answered and all exact;
on a later image, in five runs of ten, two or three reads went unanswered past the wait's last bound
(the row stops at its third), each recovered with the next read whole, every answered read exact - so the
letter judges the answered reads and prints the unanswered ones. What
such a load can do to the core is in [dma.md](dma.md)'s "Not covered yet".

Runs of two and eight on the same three streams every 5, starved from their start or from
the third byte, 50 rows each a run: on a stream SCL is held, BTF standing but for a write starved from its
start, whose first byte never reaches the shifter; the thread runs on, the event vector
takes SB's and ADDR's entries alone, nothing answers while the load stands, and when it comes
off every byte is there, read and written. A read of two is not on a stream: it ends with the load
up, exact, the target's pointer at three, as with no load. A read of eight moves it nine, as
with no load, starved from its start or its third byte. **A read of eight starved across its last two
bytes clocks one more:** the load up from the seventh, 50 of 50, the eighth is acknowledged, since LAST
acts when the stream takes the seventh, the wire waits on BTF, and that take lets a ninth out, NACKed. The
pointer moves ten; the reply is exact and `i2c_ok`, and a client whose read has a side effect has
lost that byte. It takes a stream held off for a byte on the wire at that one place, and every read
of three or more has that place.

**Why the event vector stays down in these phases.** With it up, BTF standing under a stream the
load held re-entered the handler for as long as the load stood: a read under three streams every 8,
the rate that load took depending on the image (2027, 2108 and 2248 beats in 100 us, and at the
last of them every run), counted 1.07 billion entries when a probe looked, the thread that could lift the
load never running, the arbiter's timeout with it. And a write of eight, the load up from its third byte, took that BTF for its end:
`i2c_ok` with four bytes at the target, 50 of 50.

**A stream's store takes BTF down on this block** ("cleared by software by either a read or write in
the DR register", 24.6.6), so BTF at the end of a phase on the stream is the last byte's. Staged in letter `r`, the
eighth byte of a write held behind the stream by that load, BTF standing 50 of 50 rows with SCL held
until the load comes off, and the store goes in over it: with the target refusing that byte,
`i2c_nack_data` 50 of 50, a read of two behind it or none, and from the third byte, from the start and
with no load the same, over fifteen runs.

**A completion held off clocks no further byte.** The chapter asks for the STOP "in the DMA
Transfer Complete interrupt routine" and does not say what the wire does until it comes. Letter
`p`, the stream's vector line off, a poll dating the count's zero, then a hold of up to forty bit
periods counting SCL's rising edges, the line back on, and the wire followed for forty bits at
100 kHz: reads of three and eight at both speeds, 3600 over fifteen runs. At most one rising edge
behind the last byte, the STOP's own; RxNE down at the end; each read and the read of eight behind
it exact. So a handler that runs long, or a mask, holds the bus with no byte clocked past the
NACK: the wire is stretched for that long, and the target sees nothing but the stretch.

**The stray beat ([dma.md](dma.md)) has no path on this host.** No path reads DR with DMAEN
up over a stopped stream: the pump's reads run with DMAEN down, raised only for a phase on the
streams (`begin_receive()`), and `put_engines_away()` stops a stream and takes DMAEN down with no
read of DR between them. Staged by hand on this block all the same, a read of one with DMAEN let
up and EN's clear in the next store: the byte left in DR 1500 of 1500, and with DR then read
with DMAEN up the next block's count moved 0 of 1500. The FMPI2C's request does remember it, 1500 of
1500: the two blocks differ here, and the manuals say nothing of either.

**The host above the wire on the Nucleo-F446RE: `bench_stm32f4`'s letter
`i`** on the same self-link (benchmark.md's grammar; the letter's header
says how the wire is counted: the SCL period measured on PB8 - 1800 core
cycles at 100 kHz and 456 at 400 for I2C1, the stated rates - a START and
an Sr a period, nine a byte, the STOP left to the next tenure's line).
Each peripheral is the host in turn and the other the target at 0x3A, served
from its own vectors on the same core: its entries are counted apart, and
its stretch of SCL while its vector runs is inside the host's fixed cost,
`fixed` being the wall less that wire. `irq` and `isr` are the host's I2C
vectors alone, between the meter's stamps. At 400 kHz, I2C1 the host, on
the pump:

| tenure | brio on the Nucleo-F446RE: irq, isr, fixed | ST's HAL on the STM32F429I-DISC1: irq, isr, fixed |
|---|---|---|
| write of 1 | 3, 409, 183 | 5, 758, 387 |
| write of 2 | 3, 448, 183 | 6, 955, 426 |
| write of 3 | 4, 602, 171 | - |
| register read 1+1 | 5, 714, 568 | 9, 1996, 1171 |
| register read 1+2 | 5, 756, 597 | 10, 1837, 816 |
| register read 1+16 | 19, 2517, 645 | 23, 3843, 792 |
| probe (ACK / NACK) | 2, 241 / 242 | polled, fixed 578 |
| write of 255 on the engines | 4, 559, x 1.00 | 4, 807, x 1.00 |

The HAL's column is ST's v1.8.5 (`HAL_I2C_Master_Transmit_IT`/`_DMA`,
`HAL_I2C_Mem_Read_IT`/`_DMA`) in a scratch program with brio's crt, clock
and ruler, on the STM32F429I-DISC1's I2C3 at the same 180 MHz core and
45 MHz APB1, against the board's STMPE811, which NACKs a read no write opened, so the HAL's
plain reads are not in it. Its register read of 255 on the DMA, one transfer
behind a polled index, is not a row either (15648 core cycles busy). Its `fixed` is
that device's and is not set against brio's, whose target is served on the
same core; the interrupts and the handler's cycles compare.

- **The pump costs 97 cycles of handler a byte written and 118 a byte
  read** (the slope between the 16- and the 255-byte rows; 101 and 125
  between 2 and 16), against the HAL's 148 and 153. A 255-byte write and
  read are x 1.00 on the pump (256 interrupts) and on the engines (4 and
  3).
- **The STOP's drain is the next tenure's**: `start()` called at a
  completion waits for the STOP to leave the wire, 1848 to 1884 cycles at
  100 kHz and 323 to 376 at 400 (the tenure before a write, a read or a
  register read of two), over the 181 it reads on a quiet bus; the HAL's
  own call waits for BUSY the same way (1728 cycles at 100 kHz).
- **The engines' fixed cost per tenure** is their start inside the
  vector - the transmit stream's in the ADDR entry, the receive stream's
  at the read's SB - and their completion entries: a write of 255 is 4
  interrupts and 559 cycles of handler, a read of 255 is 3 and 477, and
  a register read under `dma`, its index on the transmit stream too, 7
  and 991 at 1+16 (the "Which engine" table below).
- **The longest single entry** of each vector over the letter: events 221
  cycles, errors 197, the streams 120.
- **The same code placed elsewhere costs differently**: the FMPI2C1
  host's rows, whose code was the same in two builds of this letter,
  moved by eight to ten cycles a byte between them - the image's layout
  under the flash accelerator. Rows are compared within one image.

**Which engine, by the cycles** (the same letter on the same self-link,
I2C1 the host, every row of one image; `irq` and `isr` the I2C vectors'
and the two streams', between the meter's stamps, the bench's two timer reads
around each entry). The column leaves out `start()`, the streams here being started
in the vectors: 163 to 193 cycles - on the pump the writes and
register reads 166 and the reads 190 to 193, on the engines 163 and 164 and 179
and 180 - so adding it takes 2 to 13 cycles off each difference, and moves no length below.
A read of two under `dma` runs the pump's own procedure, no stream in it, so its row is the
same code in another place, its -8 the image's layout.
At 400 kHz, the 100 kHz rows within a few cycles of these:

| phase | `interrupt`: irq, isr | `dma`: irq, isr | `dma` - `interrupt` |
|---|---|---|---|
| write 1 | 3, 409 | 4, 552 | +143 |
| write 2 | 3, 448 | 4, 559 | +111 |
| write 3 | 4, 602 | 4, 559 | -43 |
| write 16 | 17, 1864 | 4, 559 | -1305 |
| read 1 | 3, 420 | 3, 480 | +60 |
| read 2 | 3, 477 | 3, 469 | -8 |
| read 16 | 17, 2231 | 3, 477 | -1754 |
| register read 1+1 | 5, 714 | 7, 992 | +278 |
| register read 1+2 | 5, 756 | 7, 987 | +231 |
| register read 1+16 | 19, 2517 | 7, 991 | -1526 |

The pump's figure a byte past the second is the slope of the table's
rows from 2 to 16 bytes, 101 cycles written and 125 read, one entry each
(97 and 118 between 16 and 255 bytes, the letter's own per-byte line),
and an entry's own exception entry and
return - about 24 cycles on the Cortex-M4 (ARM DDI 0439B 3.9.1, twelve
each way at zero wait states), which no stamp sees - comes on top for
every interrupt counted. So, in core cycles:
- **a write costs the core fewer cycles on the engines from three
  bytes**: EV8_1 loads the first two in the entry that clears ADDR, so the
  pump takes no interrupt for a write of one or two and the engines add
  their stream's start and its completion entry (+143 and +111, one
  interrupt more);
- **a read from three bytes, a read of two being the same procedure either way**: the engines'
  receive run under LAST ends in one completion entry, at about the
  cost of the pump's two-byte procedure, the same three interrupts. The pump's third byte
  costs it 125 cycles and an entry;
- **a register read with a one-byte index from four read bytes**. A Request has
  one mechanism for both halves, so a short register read is cheaper on the pump:
  under `dma` the index rides the transmit stream too - its completion and the
  TxE behind it, where the repeated START is requested, are two
  interrupts the pump's ADDR entry does without - so the engines' register
  read stays at about 990 cycles and 7 interrupts whatever its length.
  The pump's grows 42 cycles and no interrupt from 1+1 to 1+2 (714 to 756,
  the two-byte procedure), then 125 cycles and one interrupt a byte: at
  1+4 the engines lead by some fifteen cycles, by the slopes (756 + 2 x 125 = 1006 against about 990, 7
  interrupts each), and from 1+5 by about 125 cycles and an entry more with every byte.
The speed moves none of these lengths: the streams and the vectors
cost within a few cycles of the same at both rows, and the wire's time between them is
the core's to spend elsewhere. The core sleeps while a stream works: the
platform's idle is a WFI that asks no stream anything, and Sleep keeps
the DMA clocked (RM0390 6.3.15: DMA1LPEN is set at reset and written by
no verb here) - so on this family `dma` costs no sleep, only its fixed
cycles.

**Under the preemptive kernel: `test_stm32f4_rubato_i2c`** on the
Nucleo-F446RE, against the same partner's `twi_peer` on the same node:
`util/i2c_bus.hpp`'s arbiter (pending depth 4, the per-bus timeout at 20
ms) over the engined `I2cHost<1>` on PB8/PB9 (DMA1 streams 6 and 0 on
channel 1) inside Rubato, the preemptive kernel type, while TIM2 posts to the
highest of the suite's active objects (together, the pack) every 97, 193 or 389
us, and that object's dispatch spins 300 or 3000 cycles - at 97
us x 3000, 17 % of the core. Every letter runs once with TIM2 silent and
six times under it; **202 verdicts, green in three runs of `z`, and letter `x` in
three runs of its own**. The figures of this suite's letters below are from those runs: three runs of
`z` are 21 passes of a letter, 18 of them under pressure.
The pack, the command channel through the arbiter and the letters are
the I2C suites' shared body (`avrxt/src/apps/rubato_i2c.hpp`, whose
header describes them), and [platform.md](platform.md) places the suite
among the preemptive kernel's; its twin over the FMPI2C1 is
[fmpi2c.md](fmpi2c.md)'s.

- **Where a preemption reaches this host: `start()`'s two waits and its
  register writes, all before the START.** `start()` waits for the last
  tenure's STOP to leave the wire and for BUSY, then raises the event
  vector and sets START last, so a dispatch above the arbiter landing in
  it lengthens the gap between tenures and moves nothing on the wire.
  Letter `x`, 400 writes of 16 bytes on the transmit engine: `start()`
  takes 214 cycles with TIM2 silent (at most 330), the suite's naming
  of the mechanism (below) included, in this image, in a loop of 400 (`bench`'s 161
  to 190 are another image, and placement and the flash accelerator move this
  figure, so the two are not compared). Under 97 us x 3000 the interrupt landed
  inside it 3, 2 and 4 times in three runs, and never with MSL set, against the
  4 independent ticks would give (400 x 214 / 17460, the suite's own line). The letter times no phase,
  so this says only that the landings do not sit near zero as on the STM32G0's
  host ([../stm32g0/i2c.md](../stm32g0/i2c.md)); whether this tenure cycle locks
  to the tick is not established. In letter `h`, where the preempting object's
  probes follow one another back to back, the interrupt found MSL set
  inside `start()` 11 times in 18 pressure passes (4, 4 and 3 a run): `start()`
  entered while the last probe's STOP was still on its way out, MSL standing
  until it is - the drain this peripheral cannot queue a START behind.
- **Every shape on every path, under every pressure as in the control**
  (letter `o`, at 400 kHz against one serve, the partner's commanded turn as a
  target, counted to its last byte).
  The shared body's Requests name no mechanism, so the suite names it in a
  wrapper around `start()`: `dma` when either half carries two bytes or
  more, `interrupt` otherwise. The probe; reads of 1 (the pump) and of 2,
  3, 8 and 32 (the receive engine under LAST); write-then-reads of 1+1
  (the pump both halves), 1+2 and 2+8 (both halves on the engines, the
  one-byte index too); writes of 1 (EV8_1, no interrupt for the byte) and
  of 2, 3, 8 and 32 (the transmit engine) - every one `i2c_ok`, every read byte-exact, the
  partner's count, sum and address matches exact. A shape's
  post-to-reply under pressure is the control's plus the dispatches that land
  in it and the kernel's path around each, each dispatch 16.7 us at 97 us x 3000:
  the 32-byte write 761 or 762 us silent and at most 800 under pressure, the probe
  35 and at most 40.
- **The speeds and the commanded stretch** (letter `p`): a read of 16, a
  1+4 and a write of 16 byte-exact at 100 and 400 kHz (the write of 16 in
  1543 to 1548 us and 396 to 402 against the wire's 1530 and 382). The
  partner's stretch of 1000 us a byte at 100 kHz, priced from below: a
  read of 8 in 9195 to 9212 us against its nine holds (the address's and
  one before each byte it gives); a write of 8 in 8992 to 9012 us against
  its eight (the address's and seven: the eighth falls after the
  tenure), the write following the read at once and meeting most of one
  hold more - the one the partner spends, as the read's last byte goes
  out, before the byte it is asked for beyond it (a write of 8 alone took
  8195 us in a run of its own, on a scratch build); under the 20 ms timeout.
- **The vocabulary** (letter `q`): a deaf client - the partner's client
  parked at 0x5D, the letter's own address (the reserved one below) -
  answers the probe and a four-byte write `i2c_nack_addr`, and the probe
  of 0x5D `i2c_ok`; the third byte of six refused answers
  `i2c_nack_data`, the write on the transmit engine, its untransmitted
  tail abandoned and counted (one a pass), the partner reporting the
  refusal and two address matches; the last written byte of a 2+4
  refused answers `i2c_nack_data` and no read half follows, the
  controller closing the tenure with the void write from the repeated
  START it had requested - so the partner counts three address matches
  where a host that lets go with a STOP makes two. The next tenure is
  `i2c_ok` after each.
- **The arbiter with three requesters of three priorities** (letter
  `w`): the suite's requester above the arbiter (20 write-then-reads of 1+6, both
  halves on the engines, two outstanding), a requester below it (10
  writes of two bytes, on the transmit engine), and in a pressure pass the preempting
  object itself (8 probes posted from its tick): every reply `i2c_ok`, to
  its requester, in its order - the upper requester's slot k complete and slot k+1
  untouched at reply k - nothing rejected, no timeout, the partner's
  tallies exact, in 5.3 to 5.9 ms a pass.
- **The held wire parks, and a PE cycle does not change that** (letter
  `h`, the partner holding SDA low from its GPIO for 60 ms): the tenure
  posted into it parks behind BUSY (27.3.3's START waits for BUSY to
  fall) and is answered `i2c_timeout` by the arbiter in 19994 to 20010
  us from its `start()`, 21 passes of 21; the same tenure at 400 kHz after commands
  at 100 kHz, a PE cycle before its START (the three timing registers are
  PE-gated), parks the same way, every run - where the FMPI2C beside it and
  the STM32G0's host forget the held line at a PE cycle and lose
  arbitration at once ([fmpi2c.md](fmpi2c.md)). `recover()`'s SWRST leaves
  the next START parking too while the line is held. Under the flood - a
  probe at a tick whenever the suite's console object has turned since the last, which
  on this core is most ticks: 180 to 720 a pass, 717 to 719 of the 920
  to 932 ticks at 97 us x 3000 - every request was answered exactly once: while
  held `i2c_rejected` (the four-deep queue full) or `i2c_timeout` (one a
  pass: the probe started after the held tenure's timeout, parked in its
  turn), after the release `i2c_ok`, or `i2c_nack_addr` in the moment the
  partner's client was not yet back (in every pressure pass); a request
  arrived in the arbiter's draining window - between a timeout's
  `recover()` and the marker after it, which only a requester above the
  arbiter can reach - in 5 of the 18 pressure passes.
  After the release the next tenure was `i2c_ok` and the arbiter voted
  idle with nothing waiting, every pass.
- **An address of 0x78 to 0x7B is a 10-bit header here, and the host
  ends it.** Measured on I2C1 with `start()`'s refusal compiled out of a
  scratch image, the STM32G0B1RE's client acknowledging 0x7A, a probe, a
  write of 4 and a 1+2 at both speeds. Without the address phase's stray
  branch (the control) each tenure stopped in the event vector - SR1
  ADD10, SR2 MSL and BUSY, the vector re-entered 200 000 times before
  the image's guard cut its line, SCL held low until the per-bus
  timeout's `recover()`. 0xF4 is 0x7A shifted, 1111 0100: this peripheral in
  controller mode sends it as a 10-bit header and sets ADD10 where a
  7-bit address sets ADDR (27.3.3, 27.6.6). With the branch, each tenure
  ended `i2c_bus_error` in two entries of the vector (SB, then ADD10),
  both lines high after, the peer counting one address match, no byte
  and one STOP, and the next tenures sound (a probe of nobody
  `i2c_nack_addr`, then the command address `i2c_ok`). The STOP alone
  does not clear ADD10: in a variant without the DR store after it the
  flag stood on, and the next tenure's START never came out (SR2 0)
  while its vector re-entered on ADD10 until the timeout; with the
  store, SR1 read 0 after every one. A READ of 0x7A is not a header:
  0xF5 went out as a plain 7-bit read and the peer served its two bytes,
  `i2c_ok` - only the write form 1111 0xx0 is one.
- **The refusal, judged.** `test_stm32f4_i2c`'s letter `c`: 0x00, 0x07, 0x78, 0x7A, 0x7B,
  0x7C and 0x7F each answered true and `i2c_rejected` by `start()`, CR1
  and CR2 as they were and MSL never set; the arbiter letter's probe of
  nobody names 0x5D. The shared letter `q` of the preemptive
  kernel's suite judges it through the arbiter with the partner
  answering the general call: `i2c_rejected` for 0x7A and 0x00 and one
  address match on the partner's count, the probe that follows.
- **The cost of the two.** The refusal is three instructions in
  `start()` (a subtraction, a compare and a branch, the address already
  loaded for the tenure). The stray branch adds nothing to a tenure's
  path in the release listing of `test_stm32f4_rubato_i2c`: every
  address phase's test of ADDR is the same two instructions (`lsls`,
  `bpl` not taken), the switch's jump table one entry longer, and the
  stray test, the STOP, the DR store and `finish()` an 18-instruction
  cold function called only from an address-phase entry that found no
  ADDR.
- **A bus error counted, and a real one.** `spurious_bus_errors()` rose by
  one in each pass of letter `p` and nowhere else, seven a run; a probe letter (not kept) placed every one at the 100 kHz
  write of 16 that ends the partner's counted serve, never at the same
  write at 400 kHz nor at the stretched one. That write's last byte is
  where the partner ends its serve and re-inits its I2C, inside the
  byte's acknowledge at 100 kHz - a STOP where none belongs, which is what
  BERR flags - attributed, not seen on a scope. The tenure ran on to
  `i2c_ok` with the byte counted at the partner, as 27.3.4 and the
  driver's counting intend.
- **Dark** (letter `i`): every object idle, the arbiter's vote for the
  sleep, no time event armed, TIM2 stopped with its DMA requests off, CR2
  at ITERREN alone, no START or STOP standing, both streams idle, BUSY and
  MSL clear, PB8 and PB9 reading high on their pads.

`test_stm32f4_i2c` on the 32F469IDISCOVERY, I2C1 on PB8/PB9 under the
board's 1.5 k pull-ups, against the MB1166's FocalTech touch controller,
reset through the panel's shared line PH7. `z` holds 59 verdicts here. 58 have
run on this board, and letter `c`'s refusal of the reserved addresses on the
Nucleo-F446RE alone ("Not covered yet"); one letter more, run by its own key and not
under `z`, wants a finger.

- **The scan finds one address of 112, 0x38**, and the device says
  what it is in its own registers: FOCALTECH_ID (0xA8) reads 0x11 - the
  note's value, the FT6x06 family - FIRMID (0xA6) 0x13, LIB_VER
  0x05 0x01, CIPHER (0xA3) 0x64, RELEASE_CODE_ID (0xAF) 0x01, CTRL 0x01,
  PERIODACTIVE 0x0A; eight identity reads running agree, and a second
  register (TH_GROUP, 0x1C) answers differently.
- **TH_GROUP, the touch threshold, written and read back** three
  times, put back to its boot value, and read at its reset value again
  after the shared reset line's pulse - 300 ms before the device's
  first report, as both datasheets say.
- **A finger on the glass** (the letter by name): in interrupt trigger
  mode (G_MODE 0x01, read back) the touch data block polled every 10 ms
  while a human touched the panel - 771 of 1418 samples with one point,
  the coordinates x 41..474 and y 123..697 in the controller's own
  frame, which is the module's PORTRAIT one (x across the 480, y along
  the 800), the press and contact event flags seen, touch id 0; INT
  idles high with no finger and gave 805 falling edges over the reports,
  a pulse per report as the note's 1.2 draws it; G_MODE put back.

## Not covered yet

Driver gaps, each with its reason:
- The byte a read of three or more clocks when its stream is late across its last two ("A
  load past what the bus takes"). 27.3.3's procedure from byte N-2, a stream of N-3 and two BTF
  entries behind it, would close it at two more entries on every such read; the page's opening
  says what a program does instead.
- 10-bit addressing on the HOST side. The resource has the mode and the
  client matches such an address; the host's header sequence (EV9,
  ADD10) has no user, and the arbiter's Request has no shape for a
  10-bit address either ([../design/i2c-bus.md](../design/i2c-bus.md)
  says so of every stratum).
- PEC as a tenure shape. The resource has ENPEC, the PEC transfer bit
  and the value; no Request asks for a checksummed transfer, and the
  first SMBus device on a board is what would ask.
- SMBus as a protocol: the mode bits, the type, ARP and the alert are
  the resource's verbs, and the alert pad has a claim verb - but the
  protocols themselves are software (27.3.7 says so) and no SMBus
  device is on the bench. The peripheral is not fully SMBus 2.0 compliant
  anyway (ES0206 2.10.2).
- The general call as a HOST verb (a broadcast write to address 0):
  `start()` refuses address 0x00 with every reserved address
  ([../design/i2c-bus.md](../design/i2c-bus.md)); a broadcast is a
  feature of its own, born with its first user. The resource enables the
  target side's recognition.
- A wake from Stop on an address match: no I2C of this family has one -
  neither this peripheral nor the FMPI2C beside it on some parts
  ([fmpi2c.md](fmpi2c.md)) - so there is nothing to build.
- The recovery ladder - when to unstick, when to retry, when to take a
  bus out of service - is the application's or a future policy type's,
  as on every other stratum.
- This chip as the bus's target, served from an active object, under the
  preemptive kernel. A target whose bytes come from an object (its vector
  posting "the host reads" and leaving SCL held, a dispatch giving the
  byte) is the one place a held-off reader would stretch the clock; no
  util service serves a target that way (`I2cClient` is served from its
  vector), and the partner board, the STM32G0B1RE's `twi_peer`, has no
  `host_read` to read this board's target. Born with the first util
  target object.

Implemented, not bench-verified (each with what would measure it):
- `start()`'s budget at another core rate: measured at 180 MHz alone. The poll's length
  in cycles moves with the APB1 ratio, the bound in time does not; letter `q` under
  letter `m`'s DynamicClock would measure it.
- The letters `o`, `p` and `q` on the STM32F429I-DISC1 and the 32F469IDISCOVERY: the self-link
  they stage on is the Nucleo-F446RE's alone.
- The engine a Request names, on the STM32F429I-DISC1 and the
  32F469IDISCOVERY: letter `j`'s traces (the one-byte phases on the
  streams, `interrupt` touching none, `dma` refused on the engineless
  host) and "Which engine, by the cycles" were measured on the
  Nucleo-F446RE's self-link alone, the two boards not being on the bench. A
  run of the suite's `j` and the bench's `i` on each board would close it.
- ST's HAL and brio on one board: the HAL's column of the cost table is
  the STM32F429I-DISC1's against its STMPE811, brio's the Nucleo-F446RE's
  against the chip's own FMPI2C1, so the interrupts and the handler's
  cycles compare and the fixed costs do not. The HAL's scratch program
  on the Nucleo-F446RE's self-link, or the bench's `i` on the DISC1,
  would put them on one board.
- Letter `c`'s refusal of the reserved addresses on the STM32F429I-DISC1
  and the 32F469IDISCOVERY: the same verdict on the same host, run on
  the Nucleo-F446RE alone, the two boards not being on the bench; a run of
  `c` on each would close it.
- The client's second own address (ENDUAL) and its 10-bit matching: the
  instrument above uses one 7-bit address, so the dual address and
  ADDMODE are written, read back and never matched on the wire. A
  controller that addresses both would close it.
- The client's general call (ENGC): written and read back, not taken on the
  wire, since every brio host refuses address 0x00, the partner board's included.
  A controller that sends a general call would measure it.
- I2C2 (compiled everywhere and driven nowhere): I2C1 and I2C3 each
  have a board's touch controller; the 32F469IDISCOVERY's CS43L22 audio
  DAC on I2C2 (PH4/PH5, 100 kHz at most, its identity register 01h
  documented) is the device that would measure the third instance.
- The host's sequences on the 32F469IDISCOVERY (EV8_1's two bytes, the
  repeated START behind a dummy, the event vector the tenure's, the
  refusal closed from the Sr): the board was not on the bench; its suite
  against the FocalTech controller is what would measure it, as the
  suite and the bench's `i` measure it on the Nucleo-F446RE.
- The refusal's close from the Sr on the STM32F429I-DISC1: measured on
  the Nucleo-F446RE's self-link (the same host, the same register
  description); the DISC1's suite, not on the bench, would say its device
  letters are unmoved.
- EV8_1's fallback - TxE not seen within ten SR1 reads of ADDR's clear,
  the byte left to the buffer vector: TxE rose within twenty cycles in
  every tenure measured, so the branch never ran. A slower APB1 against
  a faster core would make it run.
- Arbitration lost (`i2c_arb_lost`): a second controller is on the wire,
  the partner board, and still no race: two held STARTs released
  by one STOP serialize, this controller's going out about 2 us after
  the STOP and the partner's waiting behind it
  ([../stm32g0/i2c.md](../stm32g0/i2c.md), "Two held STARTs released by one STOP
  serialize"). What
  would measure it is a peer releasing its START by software at a delay
  swept across this one's.
- A bus error status (`i2c_bus_error`): the host answers it for an
  overrun, a PEC error or an SMBus time-out or alert inside a controller
  tenure, none of which a plain controller tenure raises, and no
  instrument here makes one.
- ES0298 2.11.1's spurious BERR told apart from a real one: the count
  moved under the preemptive kernel's suite, once a pass at a write that
  ends the partner's counted serve at 100 kHz, the tenure running on to
  `i2c_ok` (Bench findings), which the partner's re-init inside that
  byte's acknowledge would explain as a misplaced STOP - a real one,
  counted the same. A scope on the two lines at that write would
  separate them.
- The noise filters on the wire: the register is written and read back
  and the ceiling arithmetic is checked, but what a 4-period digital
  filter does to a glitch wants a glitch generator.
- The last byte of a read of three or more when its wait runs out: with
  the STOP requested and byte N-1 taken, the event vector polls RxNE up
  to 400 000 times for byte N and, when the bound runs out, reads DR
  regardless and reports `i2c_ok` with what it holds (`receive_step()`
  in `brio/stm32f4/i2c.hpp`). By 27.3.3 byte N is already in the shifter at
  that point ("data N-1 in DR, data N in shift register"), so no client's clock
  stands between the two bytes and a correct wire should not reach the bound;
  whether any run has is not recorded, and nothing here could stage it.
- The 88 kHz workaround for ES0206 2.10.4: the suite reports that a
  100 kHz bus sits inside the errata's window, and every repeated start
  it made succeeded - which is not proof the timing is met, only that
  this device tolerates it. An oscilloscope on Tsu;sta would measure
  it.
