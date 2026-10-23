# I2C (CH32V203, CH32V303)

This document is for a programmer who needs an I2C bus on a CH32V203 or
a CH32V303: a sensor or a memory read and written from an active object
(a component of a brio program with its own event queue, served one
event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)), or this chip
as a target another controller addresses. The block is the STM32F1's
I2C under WCH's register names, up to two two-wire ports, host and
target, 7- and 10-bit addresses with a dual address and the general
call, two speeds with fast mode's two duty shapes, clock stretching,
PEC and the SMBus bits, and two DMA requests each - an event machine
over two vectors per instance. `brio/ch32vx03/i2c.hpp` gives it three
layers: `I2c<n>`, the register block's verbs; `I2cHost<n, pins,
TxEngine, RxEngine>`, the host engine that the bus arbiter of
util/i2c_bus.hpp drives (the arbiter, `I2cBus`, is an active object
that runs the requests posted to it one after another and replies to
each with an event, util/bus_master.hpp): an active object posts a
request, the arbiter runs it as one bus tenure (the time the host holds
the bus, from the START to the STOP) and replies with an `I2cDone`
event when the wire is done
([design/i2c-bus.md](../design/i2c-bus.md)); and `I2cClient<n, pins>`,
the target side, polled or served from its two vectors. The engine
moves bytes one interrupt each through its event vector (the pump), or
on the instance's own two DMA channels when it is given its two
engines.

The smallest use, the first block of "How to use it" below, a register
write-then-read at address 0x2C:

```cpp
using Bus = brio::I2cHost<1>;                       // PB6/PB7, no engines
using Arb = brio::I2cBus<Bus, P, 4, brio::BusPassThrough,
                         brio::ticks_from_ms<P>(20)>;

Bus::init(clock);                                   // PB1 must be 4..60 MHz
post<Arb>(Bus::Request{.addr = 0x2C,
                       .tx = lend<Lease::reply>(out), .tx_len = 4,
                       .rx = lend<Lease::reply>(in),  .rx_len = 2,
                       .reply = reply_to<Probe, brio::I2cDone>(),
                       .speed = brio::I2cSpeed::fast_400k});
```

Line by line: `Bus` is a type alias for the host engine over I2C1 on
its default column, PB6 for SCL and PB7 for SDA, with no DMA engines,
so every byte is an entry of the event vector. `Arb` is the arbiter
over it and over the program's platform type `P`
([platform.md](platform.md)): four requests may wait behind the one in
flight (a further one is answered `i2c_rejected` at once),
`BusPassThrough` hands every outcome to the requester as it is, with no
retry, and `ticks_from_ms<P>(20)` is the per-bus timeout in kernel
ticks - a tenure that has not answered in 20 ms is answered
`i2c_timeout` and the engine `recover()`ed. `Arb` is listed in the
program's `Tenuto` with its other active objects. `Bus::init(clock)`
runs once in `main()`, after the clock task's `init()` and before the
kernel's `run()`; `clock` is the clock task's tag object
([clock.md](clock.md)), whose type states the PB1 rate the timing is
solved from. It solves both speeds, hands the pads to the block open
drain, raises the error vector and enables the instance's two
interrupt lines; the excerpt does not look at its answer, and a program
does (below). The `post` runs in an active object's handler, `Probe`
being the one that receives the reply: the address is the 7-bit one,
unshifted; four bytes of `out` are written, then a repeated START and
two bytes read into `in`; `reply_to<Probe, brio::I2cDone>()` names the
receiver of the `I2cDone`; `speed` asks for 400 kHz. `lend<Lease::reply>`
marks each buffer as lent (kernel/borrowed.hpp): `out` and `in` stay
untouched and in scope from the post to the `I2cDone` that `Probe`
receives, because the vectors read and write them in place, and
nothing checks it. The arbiter calls `Bus::start()` from its own
dispatch, in main context, and the tenure then runs in the vectors.
Left out of the block are the two bindings that end a tenure: the
program binds `i2c1_ev_handler` and `i2c1_er_handler`, the crt's names
for I2C1's two vectors (the crt is the startup code that runs before
`main()`), with `BRIO_CH32_VECTOR(name) { body }`, each body calling
`Bus::isr()` or `Bus::error_isr()` and, when it answers true, posting
`brio::TransferDone{Bus::status()}` to `Arb`, as `test_vx03_i2c`'s
handlers do for the arbiter's tenures.

What decides correctness even there:

- **The clock task first.** `init()` solves CTLR2.FREQ, CKCFGR and RTR
  from the PB1 rate the clock type claims, not from the hardware. A
  host initialized while the chip still runs on its 8 MHz HSI reset
  rate (PB1 8 MHz) under a type that claims 96 MHz (PB1 48 MHz) writes
  FREQ 48 and CCR 240, and SCL runs at a sixth of the rate asked until
  the clock moves, at the rate asked after (CCR is a divisor of PB1,
  written once); derived from the CCR arithmetic, not measured. What
  `init()` refuses is a claimed PB1 outside 19.12.2's 4..60 MHz window
  ("The bus clock has a ceiling" below), never a hardware rate that
  differs from the claim.
- **The ticker running.** `start()`'s two bounded waits and
  `unstick()`'s half bits are timed on the STK through `delay_us`
  (`brio::Ticker::init(clock)`, [platform.md](platform.md)), which
  refuses and spends no time with the counter stopped: before the
  ticker runs, `start()`'s bounds are counts of polls and `unstick()`
  clocks its pulses with no half bit between them (derived from the
  code, not measured).
- **The two vectors bound by the program**, under the crt's names and
  with the one spelling of a binding, `BRIO_CH32_VECTOR` or one of its
  two forms ([platform.md](platform.md)). A vector left unbound runs the
  crt's weak `default_handler`, a spin loop, with no fault and no message:
  an unbound event vector is entered at the first tenure's SB, and the
  core stays there. On the CH32V303's images, built with the F
  extension and the hardware prologue on (the core saving the
  caller-saved integer registers itself; the `ch32vx03/` project's
  `CH32VX03_HPE` option, a CMake option chosen at configure, on by
  default), the binding is a naked trampoline whose body must do no
  floating-point work; the engine's bodies do none.
- **Pull-ups on the wire.** The pads are open drain under the block and
  this family's pad has no pull in any output mode, so the bus's
  resistors own the idle level (the bench carries 2.2 kOhm, below).
- **`init()`'s answer read.** False for a PB1 outside the window, for a
  rise time told to `init()` whose TRISE does not fit RTR's six bits, or
  for a column the part's AFIO refuses; the two interrupt lines then
  stay off, and a request posted after an `init()` that refused the
  clock is answered `i2c_rejected` by `start()`, no speed having been
  solved.
- **With the engines, no sleep while a block moves.** In a sleep of any
  depth on this family no bus master but the core gets a cycle, so a
  DMA channel enabled is counted as a working bus master
  (ch32vx03/bus_activity.hpp, [sleep.md](sleep.md)) and the kernel's
  idle path does not sleep while one is; the engines stop each channel
  at its block's completion ("The DMA rows" below).

What the engine waits for, and where. `start()` runs in the arbiter's
dispatch and waits there, bounded, for the last tenure's STOP to leave
CTLR1 (5 ms) and then for BUSY to clear (100 us). When the first bound
runs out the tenure parks - no START and no event vector - and the
per-bus timeout is what answers it; when the second runs out the
engine resets a BUSY left standing over an idle wire (both lines high,
"BUSY is the wire" below) and sets the START either way. In the entry
that clears ADDR the event vector's body polls TxE in up to two waits
of at most sixteen STAR1 reads each before it leaves a byte, or the
repeated START, to the buffer interrupt; and on a host without engines
a read of three bytes or more ends with a wait for the last byte's RxNE
of at most 100 000 STAR1 polls whose answer the code discards: when it
runs out, DATAR is read regardless and the tenure reported `i2c_ok`
with what it holds, and whether a case reaches it is not recorded.
Without the engines nothing in the driver masks interrupts. With them,
`init()`, `recover()` and `release()` stop the channels, and wherever a
DMA block starts or ends - those three verbs, and the vectors as a block
starts or completes - the channel's entry into or exit from the count
of active bus masters is made under an interrupt guard of a few
instructions
([bus_activity.hpp](../../brio/ch32vx03/bus_activity.hpp)). `init()`,
`unstick()` and `recover()` turn the instance's two interrupt lines off
while they work, and `release()` turns them off for good.

The rest of this page, in order: the documents of record; what the
silicon does (the bus clock's ceiling, the three timing registers, the
software sequences, the repeated START requested while the last byte
goes out, BUSY, a START on a busy bus, what the engine takes from the
chapter and what it declines, the pads, the DMA rows, how many
instances a part has); the types and verbs (the vocabulary, the
resource, the host engine, the client); how to use it, in five uses;
the bench findings, with `bench_vx03`'s letter `i` pricing the host
above the wire; and what is not covered.

## Documents of record

The CH32F/V20x_V30x_V31x reference manual V2.3 (19.1 for what the block
claims, 19.3 and 19.4 for the host and target sequences with figures
19-4 to 19-7, 19.5 for the errors, 19.6 for the clock stretch, 19.7 for
SMBus, 19.9 for the DMA, 19.10 for the PEC, 19.12 for the registers with
tables 19-1 and 19-2 for the two instances' maps, 10.2.11 and table
10-34 for the pads, 11.2.3 with table 11-5 for the DMA rows, 3.4.7 for
the gate and 9.5.1 with table 9-2 for the four vectors), the CH32V203
datasheet V2.8 (table 2-1 for how many I2C a part has, section 3 for
which pads it bonds and where the SMBus alert sits) and the CH32V303
datasheet V3.5 (table 2-1-1 and the pin tables of 3.2). Three device
classes read the chapter - CH32V20x_D6 for every part up to the
CH32V203C8, CH32V20x_D8 for the CH32V203RB and CH32V30x_D8 for the four
CH32V303 - and chapter 19 carries no class note: one block on all three.
Driver: [brio/ch32vx03/i2c.hpp](../../brio/ch32vx03/i2c.hpp). Reference
suite: `test_vx03_i2c` (a suite's letter is one command key of its
console and the group of verdicts it runs; `z` is the key that runs
every letter of the image but `r`, which runs by name only), every
letter on I2C1 and every far end a second
chip running `twi_peer` - the other series of this stratum on the bench,
the CH32V203C8T6 and the CH32V303VCT6 each the other's peer, or a board
of another family; `bench_vx03`'s letter `i` prices the host against the
same far end.

## What the silicon does

### The bus clock has a ceiling, and it is this chapter's

CTLR2.FREQ states PB1 in whole megahertz and 19.12.2 confines the field
to 000100b..111100b, 4 to 60 MHz. The number has to fit, so a PB1 above
60 MHz has no legal timing at all: `i2c_timing_for()` answers nothing
there and `init()` returns false. This stratum's PB1 ceiling is 72 MHz
(the clock chapter), so I2C is the one peripheral that cannot be used at
the top of the tree - a program that wants a bus keeps PB1 at or below
60 MHz: an HCLK of 60 MHz or less, which PB1 takes undivided, or one
from above 72 to 120 MHz, which the clock task halves for PB1
(`pclk1_hz_at()`, ch32vx03/clock.hpp); an HCLK above 60
and up to 72 MHz is PB1's own rate and has no timing either. The
rise-time register says the same from the other side:
standard mode's TRISE is (1000 ns x Fpclk) + 1 and its six bits overflow
just past 60 MHz.

### Three registers make the SCL timing, not two

Unlike the CH32V00x's block this one has a rise-time register (RTR,
19.12.9): six bits, reset value 2, holding the longest rise the block
must plan for as (rise / Tpclk) + 1. So the timing is CTLR2.FREQ,
CKCFGR (CCR under F/S and DUTY) and RTR together, and
`i2c_timing_for()` solves all three. It takes the wire's rise time in
nanoseconds as an argument, because the wire is not a property of the
silicon: told nothing it uses the mode's own maximum, 1000 ns in
standard mode and 300 ns in fast mode.

The CCR arithmetic is the F1 lineage's: standard mode divides PB1 by
2 x CCR, fast mode by 3 x CCR at DUTY 2 and by 25 x CCR at DUTY 16/9,
rounded up so a bus never runs faster than asked. At 48 MHz of PB1 that
is CCR 240 for 100 kHz, 40 for 400 kHz at DUTY 2, and 5 at DUTY 16/9 -
which is 384 kHz and not 400, because five is what rounding up gives.

### The machine is a set of software sequences

Every one of them stretches SCL low until it completes (19.3's notes):
EVT5 (SB up: read STAR1, write the address), EVT6 (ADDR up: read STAR1
then STAR2), EVT8 (TxE: write DATAR), EVT8_2 (TxE and BTF: the last byte
is out, STOP or repeated START), EVT7 (RxNE: read DATAR). The receive
half depends on the count: one byte wants ACK cleared before ADDR is
cleared and STOP set right after; two want POS set with ACK, ACK cleared
after ADDR, and the pair read after one BTF; three or more run on RxNE
until three remain and then wait for BTF twice. All of it lives in the
engine's `isr()`, and the buffer interrupt (ITBUFEN) is switched on and
off inside a tenure because TxE and RxNE are wanted only while the byte
pump needs them.

### The repeated START is requested while the last byte goes out

A write-then-read turns around on a repeated START, and when it is
requested is not free on this silicon. Requested at EVT8_2 - TxE and
BTF, the last byte and its acknowledge done and the host holding SCL low
- a CH32V303VCT6 target loses that last byte: it acknowledges it and
never raises RxNE for it, in silence (measured with the chip's two
controllers on one bus: the last of one, two, three and four written
bytes, at both speeds, whichever instance was the target, while the
same bytes closed by a STOP all arrived - the target polled by the core
that also serves the host's vectors, through its ISR body's order, the
address before a received byte). A target on another board, polled by a
core of its own, did not lose it on either part: the CH32V203C8T6's
under a CH32V303VCT6 host and the CH32V303VCT6's under a CH32V203C8T6
host, one to four written bytes with the START requested as soon as BTF
was seen and 20 us past it, twelve probes each way and the last byte in
every one - on the CH32V203C8T6 whether its target took a standing byte
before answering the address or after. So the loss wants one core
serving both ends: measured on both parts with their own two
controllers (below), and on neither part's target served by a core of
its own. A register write followed by a read of that register is
exactly this shape, and the register number is what such a target
would lose. Requested a byte earlier - on the TxE
that says the last byte went into the shifter, so the peripheral
generates the START at the end of that byte, which is where WCH's own
interrupt example puts it - every byte arrives, at both of those
targets. So the engine requests it there, on the pump and on the DMA
path alike (the transmit block's completion arms the TxE it waits for).
A TxE served later than one byte time still falls back to the BTF
order, which a target of another family takes, and a CH32V203C8T6
target too.

The CH32V203C8T6's two controllers on one bus (I2C1 the host, I2C2 the
target, two wires from PB10/PB11 to PB6/PB7 and a 4.7 kOhm pull-up on
each line) measured both orders again, one to four bytes written and a
read of two, at 100 and 400 kHz, three runs a point:

- **A target that ACKs every byte**: served from its own vector, it took
  every written byte in both orders; polled by the loop that waits for
  the host, it lost the last written byte
  in every write-then-read with the START requested at BTF - the
  CH32V303VCT6's finding, on this part too - and took every one with the
  START requested on the TxE.
- **A target that refuses the last written byte**: at BTF the host
  answered `i2c_nack_data`, STOP alone left standing, and the next tenure
  ran. On the TxE, the NACK arrives with the START still pending - the
  price of the order, which the next paragraphs answer.

The chapter's own order is BTF: 19.3 ends a write at EVT8_2 ("TxE=1,
BTF=1, request to set the stop bit", figure 19-4), and 19.12.1's START
bit, unlike its STOP bit, says nothing of a condition held to the end of
the byte under way. The engine keeps the TxE order all the same, because
the BTF order loses a byte to a polled CH32 target, measured twice, and
answers the refusal instead: the phase between the request and SB is its
own, and a NACK in it is `i2c_nack_data`. BTF never rises behind a
pending START on this silicon (polled from the request to SB, and no
entry of the event vector there in a timeline), so unlike the STM32F4's
block nothing is stored into DATAR to hold it down.

The requested START is not withdrawn. The controller generates the
repeated START after the refused byte all the same, and a CTLR1 store at
the NACK that clears START and sets STOP - START is a bit "the user
code" may clear (19.12.1) - races that generation. Measured with the
chip's two controllers on one bus (I2C2 the target, served from its own
vector, refusing the last of one to four written bytes, four times a
count, through the pump and the engines, at 100 and 400 kHz), with that
one-store withdrawal: every refusal clean with the core at 96 and at 48
MHz; with the core at 8 MHz (the HSI alone, PB1 8 MHz) at 100 kHz, all
32 refusals answered `i2c_nack_data` and left STOP standing over a
controller still master of the bus (CTLR1 0x201, STAR2 0x7 - MSL, BUSY,
TRA) and the next tenure parked, the CH32V006's wedge at a slower core.
So the engine writes nothing at the NACK: it waits for the Sr's SB and
closes the tenure with the one sequence EVT5 allows (19.3) - the address
with the WRITE bit, and STOP requested behind it, "after the current byte
transfer" (19.12.1) - a void write, Sr A P on the wire, its ADDR cleared
or its own NACK taken before the reply. Measured the same way: 192
refusals of 192 answered `i2c_nack_data` with the controller let go
(START and STOP down, MSL clear, 2 ms on) and the next tenure `i2c_ok`,
at all three cores, three runs; at 8 MHz and 400 kHz the
target - served by the same core - is late with its own NACK in half the
engined refusals, which then take the byte whole and end `i2c_ok`, an
instrument's limit counted apart. Against another chip's target -
refusing from its own polled core, its acknowledge down a byte ahead
(the suite's letter `o`) - every refusal of every run answered
`i2c_nack_data`, the controller let go and the next tenure `i2c_ok`, on
both parts as the host (the counts in the findings below). The price is
the refusal's:
one address byte on the wire; an acknowledged tenure pays for it only
the SB entry's test, about ten cycles on a read half at 120 MHz
(`bench_vx03`'s letter `i`).

### BUSY is the wire, and it can be left standing

19.12.7 defines BUSY as "SDA or SCL has a low level", cleared when a
STOP is detected. A tenure that ends without the peripheral seeing its
own STOP therefore leaves BUSY standing over an idle wire - 19.12.1's
own case for SWRST, "when no stop condition is detected on the bus but
the busy bit is 1". The engine answers it: when its bounded wait for
BUSY runs out and both lines read high, it takes the chapter's reset,
rewrites the timing and the interrupt enables, and issues the START. The
wire is the guard - a bus another master really holds is never reset
out from under it.

### A START on a busy bus waits for a tick, a START on a free bus is committed

A START set while BUSY stands does not wait for the STOP alone. With SCL
standing high and SDA held low - a line held down, no clock on the wire
- the pending START reaches the wire at the controller's next tick:
about 85 us after the edge that made the bus busy, or a multiple of 80
us after that. Measured on the CH32V303VCT6, SDA held low by its own
port: set 21 us and 41 us after that edge it left at 85 us, set 151 us
after it at 165, set 259 us after it at 326; a CH32V203C8T6's START set
inside another board's window left 325 us after the window opened. PE
dropped and raised by two consecutive stores does not stop it, though
19.12.1 has a cleared PE clear START - the one PE-low width measured;
writing START back to zero withdraws it, which no verb of this driver
does; and an edge on SCL releases it at once (the first pulse did,
twice). A START set on a free bus, or pending when a STOP frees it, is
committed: SDA falls 5.1 to 5.3 us later, whatever the wire does
meanwhile - SDA pulled low again within half a microsecond of the STOP
did not stop it: SCL fell 5.3 us after the STOP with the controller's
SB up. Two controllers' STARTs therefore meet where both are pending
when one STOP frees the bus, or where one is pending and the other is
set within the 5 us that follow that STOP; a START set into a window
longer than a tick goes out alone, into whatever holds the line. That
is also why a tenure into a wire a foreign chip holds low is answered at
once and not parked: its START leaves at the tick and loses at its first
one bit.

### What the engine takes from the chapter, and what it declines

Every item that bears on a tenure's cost, measured on the CH32V203C8T6
at 120 MHz (PB1 60 MHz), the host's own:

- **One data register, no FIFO, no byte counter, no automatic STOP**:
  one interrupt a byte is this block's on the pump.
- **EVT8_1 takes two bytes at once**: TxE is up within 47 cycles of
  ADDR's clear and again of the first byte's store (the ruler's own read
  inside that figure), so the entry that clears ADDR loads the data
  register and the shifter itself, and requests the repeated START of a
  one-byte index there too. Used.
- **The data register before the START**: declined - EVT5 puts the
  address there.
- **The event vector is a level, and BTF stands after a write's STOP**
  until that STOP is on the wire: a vector left on re-entered four times
  at 100 kHz and once at 400 after every write, the thread starved
  meanwhile (a timeline of the entries, below). ITEVTEN is the tenure's:
  raised by `start()`, taken down at the completion with the buffer
  vector and the DMA requests in one CTLR2 store (`control2()`) - and
  down for a DMA read phase as well, which the receive channel's
  completion ends: over a channel that does not serve, RxNE and BTF would
  stand under it and the vector would starve the thread and the bus's
  timeout.
- **The STOP on the last byte's TxE**: declined - no event marks a
  controller's own STOP leaving, so the write ends at BTF (EVT8_2) and
  the STOP's drain is the next `start()`'s: about a bit period when a
  tenure is started at the completion - 1032 cycles after a write at 100
  kHz, 344 at 400 - spent in the caller. A deferred START would need an
  event this chapter has not got, or a timer and an interrupt a tenure.
- **The receive procedures by count**: used; the faster path that NACKs
  and STOPs on the second-to-last RxNE is declined, its deadline (before
  the last byte's acknowledge) being one no handler of a program
  guarantees.
- **The DMA rows**: a write phase of three bytes or more (two ride
  EVT8_1) and a read phase of two or more run on the engines. They buy no
  CPU time at this bus's rates: while a block is in flight the core may
  not sleep (the bus matrix serves the core alone in a sleep), so a
  255-byte write at 400 kHz keeps it awake for 486 thousand cycles where
  the pump's 256 entries take 51 thousand and let it sleep between them -
  the pump is this family's bulk path for I2C wherever the core has
  nothing else to run.
- **START cleared by the user code - a requested start withdrawn**
  (19.12.1): declined. Behind a refused last byte the controller
  generates the repeated START all the same, and the withdrawal races it
  - lost with the core at 8 MHz (above); the refusal is closed from the
  Sr's SB with a void write instead.
- **The START itself is slow on this silicon**: SB comes 1247 cycles
  after START at 100 kHz and 431 at 400 (10.4 and 3.6 us, polled), the
  largest single term of a short tenure's fixed cost.

### The pads are a column, the alert is not

Table 10-34 gives I2C1 two columns, the default PB6/PB7 and code 1's
PB8/PB9; I2C2 has no remap field at all, one column on PB10/PB11 from
the datasheet's pin table. `I2cPins` carries the code and `init()`
writes it, refused where the part bonds no pad of it. The SMBus alert
pad does not move with the column (PB5 for I2C1, PB12 for I2C2,
datasheet section 3) and is the application's to claim: no verb of this
driver drives it.

### The DMA rows, and what they cannot express

Table 11-5 wires I2C1's transmit request to DMA channel 6 and its
receive to 7, I2C2's to 4 and 5. On this family the channel is the
request, so an engine slot naming any other channel is refused at
compile time. Both engines arm at high priority, the transmit level of
[../design/dma.md](../design/dma.md)'s rule and not a receive's: the
host does not overrun - a byte a starved channel has not moved leaves
BTF up and SCL held low until it is (19.3's notes), the bus slower and
nothing lost. With engines a write phase of three bytes or more (two
ride EVT8_1's own entry) and a read phase of two bytes or more run on
them, the read under CTLR2.LAST so the controller NACKs the block's last
byte; a one-byte read stays on the
pump, because its ACK-before-ADDR sequence has no DMA shape. In sleep
the bus matrix serves the core alone on this family, so a transport with
engines holds the program awake while a block is in flight - and stops
the receive channel when its block completes, because a channel's EN
stays set after a block and an enabled channel is counted as a working
bus master ([dma.md](dma.md)). The transmit channel's completion still
raises its interrupt here, unlike the SPI host's: on this bus nothing
comes back to prove it, and it is the edge the host acts on - the event
vector ends a write on BTF only behind it, and a BTF with the transmit
channel's count not at zero ends the tenure `i2c_dma_fault`, its STOP
requested: a stalled channel and not a written block (measured: with a
request another peripheral left held on channel 6, the target took one
byte of a write of 255, the count standing at 254 at that BTF). A
write-then-read's repeated START is requested on the TxE the completion
arms.

`release()` pulses the reset line before the gate, as `recover()`'s
SWRST does: a request raised under DMAEN is held through DMAEN and PE
cleared, and a tenure abandoned mid-block leaves one - measured, it
stalled the next USART2 transmit on channel 7 after one byte
([dma.md](dma.md), "A released requester"). A release after a completed
tenure leaves none (the completion drops DMAEN before a further request
can rise), so the hazard is the release of an abandoned or stalled
tenure - the state a timeout leaves.

### How many instances a part has

`device::i2c_count`, from the datasheets' tables 2-1 and 2-1-1 and
nothing else: the CH32V203F6 has no I2C (I2C1 lives on PB6/PB7 and that
package bonds neither), the parts up to the CH32V203K8 have I2C1 alone,
and the CH32V203C8 and RB and all four CH32V303 have both. An instance a
part has not got does not compile.

## Types and verbs

### The vocabulary

`I2cSpeed` names the chapter's two speeds and `I2cDuty` fast mode's two
shapes. `I2cTiming` is what the three timing registers hold;
`i2c_timing_for()` solves it from the bus clock, the speed, the duty and
the wire's rise time, and answers nothing when the clock is outside
19.12.2's window or when CCR or TRISE would not fit its field.
`i2c_scl_hz()` says what a timing really produces. `i2c_bus_hz()` and
`i2c_bus_hz_at()` are where the bus rate comes from - PB1, never the
system clock. `I2cPins` is a column with its remap code, built by
`i2c_pins_for()` and judged by `i2c_pins_valid()`. `I2cAddressConfig` is
a target's addresses: one 7- or 10-bit own address, an optional second
7-bit one, the general call.

### The resource

`I2c<n>` is the register block: its gate and its RCC reset, the timing
written with PE clear, the own addresses, the enable, the software
reset, every control bit as a verb (START, STOP, ACK, POS, the general
call, NOSTRETCH, PEC and its next-byte bit, the SMBus mode and type,
ARP, the alert, the DMA enable with LAST), the data register, both
status words with the sequences that clear ADDR, STOPF and the errors,
and the three interrupt enables. It decides nothing.

### The host engine

`I2cHost<n, pins, TxEngine, RxEngine>` is what `util/i2c_bus.hpp`'s
`I2cBus` (= `BusMaster`) drives, with the same Request on every stratum,
verbatim: one tenure is a write, a read, a write-then-read with a
repeated START, or the empty probe. `init()` solves both speeds for the
bus clock and claims the pads open-drain; `rebase()` re-solves them
after a clock change; `speed_ok()`, `timing_of()` and `scl_hz()` report
what a speed resolved to. `start()` returns true only for the one
refusal that moves nothing - a speed the clock cannot produce, answered
`i2c_rejected`. `isr()` and `error_isr()` are the two vectors' bodies
and `dma_isr()` the channels'; `status()` carries the outcome.
`unstick()` is the wire's remedy (nine clocks and a STOP by hand,
counting the pulses a stuck target took to let go), `recover()` the
peripheral's, and `unwedges()` counts how often the stuck-BUSY case
above was reset out of the way.

### The client

`I2cClient<n, pins>` is the target side: `init()` takes the addresses
and `I2cClientOptions` (NOSTRETCH, and whether to arm the three
interrupt enables at all - a polled client wants them down). The polled
surface is `addressed()`, `answer_address()` (which also says whether
the host reads), `data_ready()`/`take()`, `data_wanted()`/`give()`,
`acknowledge()`, `stop_seen()`/`clear_stop()`, `host_nacked()`, and
`flush()`, the PE cycle that throws away a byte the controller never
clocked. `service()` and `error_service()` are the two ISR bodies, each
reporting one `I2cClientEvent`, and a polled client calls `service()`
from its loop instead (the polled target below).

## How to use it

A bus host over the kernel's arbiter:

```cpp
using Bus = brio::I2cHost<1>;                       // PB6/PB7, no engines
using Arb = brio::I2cBus<Bus, P, 4, brio::BusPassThrough,
                         brio::ticks_from_ms<P>(20)>;

Bus::init(clock);                                   // PB1 must be 4..60 MHz
post<Arb>(Bus::Request{.addr = 0x2C,
                       .tx = lend<Lease::reply>(out), .tx_len = 4,
                       .rx = lend<Lease::reply>(in),  .rx_len = 2,
                       .reply = reply_to<Probe, brio::I2cDone>(),
                       .speed = brio::I2cSpeed::fast_400k});
```

With both DMA engines, which are the instance's own two channels:

```cpp
using Dma = brio::I2cHost<1, brio::i2c_default_pins<1>,
                          brio::DmaTxEngine<1, 6>, brio::DmaRxEngine<1, 7>>;
BRIO_CH32_VECTOR(dma1_channel6_handler) { (void)Dma::dma_isr(); }
BRIO_CH32_VECTOR(dma1_channel7_handler) { (void)Dma::dma_isr(); }
```

A wire whose rise time has been measured, told to the block:

```cpp
Bus::init(clock, brio::I2cDuty::ratio_2, 1000);     // 1000 ns of rise
```

A polled target at two addresses, answering the general call:

```cpp
using Peer = brio::I2cClient<1>;
Peer::init(clock, {.own = 0x2C, .second = 0x39, .general_call = true},
           {.no_stretch = false, .interrupts = false});
for (;;) {
    switch (Peer::service()) {
        case brio::I2cClientEvent::addressed:     /* Peer::host_reads() */ break;
        case brio::I2cClientEvent::byte_received: take(Peer::take()); break;
        case brio::I2cClientEvent::byte_wanted:   Peer::give(next()); break;
        case brio::I2cClientEvent::stop:          end_of_tenure(); break;
        default: break;
    }
}
```

The bus back from a stuck target, and the engine back from a lost
tenure:

```cpp
const uint8_t pulses = Bus::unstick();   // 0 = the wire was never stuck
(void)Bus::recover();                    // SWRST, the timing again, PE back
```

## Bench findings

`test_vx03_i2c` on the CH32V203C8T6 and on the CH32V303VCT6, both at 96
MHz of HCLK - PB1 = 48 MHz, which is what the FREQ window allows and what
makes both speeds exact - each against the other running `twi_peer` (the
peer's PLL on its 8 MHz crystal): I2C1 to I2C1 on PB6/PB7, a 2.2 kOhm
pull-up on each line to the CH32V203C8T6 board's 3.3 V, a shared ground
([../boards/weact-ch32v203c8.md](../boards/weact-ch32v203c8.md)). Every
letter both ways, ten runs of `z` in each direction with no failure,
and ten of letter `d` each way since its counted serve (90 verdicts on
the CH32V303VCT6's one image, 53 and 37 in the CH32V203C8T6's two). TIM4's channel 1 is the second instrument: PB6 is I2C1's clock line
and TIM4_CH1 on its default column at once, so SCL is measured on the pad
that carries it, with no scope and no wire. The figures below are the
CH32V203C8T6's where one is given, the CH32V303VCT6's beside it where
they differ.

- **The reset values are table 19-1's, with one exception that is the
  chapter's own**: every register reads zero after the RCC pulse but
  RTR, whose reset value is 2 - the same on both parts.
- **The arithmetic is exact at 48 MHz.** 100 kHz: FREQ 48, CCR 240,
  TRISE 49, and the pad measures a 9979 ns period (9989 on the
  CH32V303VCT6) - 100 kHz. 400 kHz at DUTY 2: CKCFGR 0x8028, TRISE 15,
  and the pad measures 2479 ns - 403 kHz. At DUTY 16/9 the register takes
  CCR 5 and the wire carries 385 to 387 kHz, which is what rounding up
  asks for.
- **THE ENABLE PROTECTION IS NOT THERE.** 19.12.9 says of RTR that it
  "can only be set when PE is cleared". Measured with PE set, on both
  parts: CKCFGR written 0x8028 reads 0x8028, FREQ written 24 reads 24,
  and RTR written 33 reads 33. All three take a write with the
  peripheral enabled. The driver still writes them with PE clear,
  because that is the order 19.3 prescribes and a read-back is not a
  promise about what the block DOES with the value.
- **What the pad shows of the wire.** At 100 kHz the capture reads a
  high time of 4947 ns against the 5000 the register asks for and a low
  of 5031; at 400 kHz, 781 ns of high. The tens of nanoseconds missing
  from each high half are what the pull-ups cost on the rise. Released
  from low, SCL and SDA are high again within 479 ns, an upper bound
  that includes the pad's own mode switch.
- **SCL RUNS AT THE RATE ASKED, read against a clock the bus does not
  share.** TIM4 counts a clock of PB1, the bus FREQ and CCR divide, so a
  period in its ticks moves with a wrong PB1 and cannot tell it; letter
  a reads TIM4's clock on the core's counter first - 96000 kHz, the
  tree's figure - and the three rungs against it in thousandths of the
  rate ASKED: 1002, 1008 and 967 (1001, 1008, 963 on the CH32V303VCT6).
  Against the far chip (letter f), a period captured at the end of a
  write: 1000, 1000 and 960 (963), the far target's polled core
  stretching none of those periods.
- **An address nobody answers is `i2c_nack_addr`**, for a probe and for
  a write alike, and the bus is idle again 6 to 7 us after the engine
  answers - the time the STOP takes to leave the wire.
- **A tenure into a wire a foreign chip holds down is answered
  `i2c_arb_lost`**, at once and by the silicon: the START, still pending
  after the 100 us the engine waits for BUSY, leaves at its tick into the
  held line and loses at the address's first one bit.
- **`unstick()` is a measurement.** With the far chip holding SDA low
  from its own port and letting go after four SCL falling edges (the
  first being that START's), the verb reported 3 pulses and TIM4 counted
  3 rising edges on the pad, both ways. The STOP it makes by hand raises
  STOPF on the instance's own target half, and that flag STANDS IN
  SILENCE: the event vector is the tenure's (raised by `start()`, down at
  the completion), so it never enters for it, and the next tenure's
  first entry takes it.
- **The command channel is two tenures of the engine under test** - a
  write carrying the frame and a read collecting the answer - and it ran
  10 of 10 round trips at 100 kHz each way. That is also where the
  target's BTF stretch falls: AFTER the ninth pulse, or a twenty-byte
  read could never be closed by the controller's NACK.
- **The tenure shapes hold against a second chip.** A write, a read and
  the combined write-then-read all complete `i2c_ok` byte-exact, and the
  far end counts FOUR address matches for the three tenures: the
  repeated START is real, seen from the other side. Reads of one, two,
  three and four bytes - the chapter's four receive procedures - are
  each byte-exact, and a general-call write reaches the far target at
  address 0x00.
- **A target transmitter's tenure ends with no Stop flag.** 19.4's two
  target sequences end differently - the transmitter's in the closing
  NACK (figure 19-6's EVT3_2), the receiver's in STOPF (figure 19-7's
  EVT4) - and `twi_peer`'s polled target, read eight bytes by the far
  controller, took the NACK and then no STOPF for the Stop behind it
  (its flush, a PE cycle, falls between the two, so whether the Stop
  alone would raise the flag is not separated). So `twi_peer` ends a
  counted serve on the wire at rest after its count, never on a Stop: a
  serve armed for eight bytes and a 2000 ms deadline is back in command
  mode 8 ms after the read, the eighth byte its own (letter d, ten runs
  each way).
- **The vocabulary is real statuses across two chips.** A deaf target
  answers `i2c_nack_addr`; a target commanded to refuse the third byte
  answers `i2c_nack_data`.
- **A polled CH32 target's stretch lands where its model puts it.** The
  far end spending 2 ms before each answer (its hold, at the address
  match and before each byte it takes, the receiver one byte late): SCL
  held low 2008 to 2014 us at the address, then 1919 to 1929 us at each
  of six bytes - the hold less the byte its shifter takes meanwhile -
  over twenty runs, and an
  eight-byte write whole in 14 ms of wall time against 6 ms for eight
  unstretched; the seventh byte's hold falls after this host's
  completion and the eighth's after the tenure. The tenure completes
  `i2c_ok`: stretching is flow control.
- **The DMA engines carry the shapes.** A 16-byte write and a 16-byte
  read on channels 6 and 7, the combined tenure, the two-byte read under
  LAST and the one-byte read that stays on the pump all complete
  `i2c_ok` byte-exact, the far end taking the 20 bytes written and asked
  for the 23 its shifter wanted, with no transfer fault on either
  channel.
- **A DMA-served read can leave BUSY standing over an idle wire.** With
  both lines high and no STOP in sight the next START is held for ever,
  which is 19.12.1's own case; the engine's SWRST takes it out of the
  way and the tenure that follows completes. SWRST TAKES CTLR2 WITH IT -
  the interrupt enables have to be written again or the next tenure runs
  with no vector at all.
- **Clearing PE does not let the pads go.** A controller parked at SB -
  a START issued and the address not yet written - holds SCL low, and it
  goes on holding it with PE clear; the block's RCC reset line is what
  frees the wire. That is what the suite's stall report uses when it
  asks which end of the link is holding the clock, and it is worth
  knowing before writing a recovery: 19.12.1's own advice to release the
  pins before a software reset cannot be followed by disabling the
  peripheral.
- **The kernel runs over the engine unchanged.** Four queued tenures
  give four replies in order with the NACK from an absent address in its
  place; a fifth and sixth into a four-deep queue are rejected
  immediately and still answered exactly once; an idle bus votes for a
  sleep and a busy one against it. A target holding the CLOCK for 45 ms
  under a 20 ms per-bus limit is answered `i2c_timeout` at 20 ms - the
  one wedge this silicon cannot answer by itself, since outside SMBus
  mode it has no clock-low time-out - and the recovered engine carries
  the next tenure to `i2c_ok` through the same arbiter.
- **ARBITRATION BETWEEN TWO CH32 CONTROLLERS, in both directions and
  from both sides.** With a START condition made by hand to hold both
  controllers' STARTs and released by a hand-made STOP so they leave
  together: aiming at 0x6B against the far end's 0x2C this controller
  reports `i2c_arb_lost` and the far one goes on to meet no target;
  aiming at 0x11 it keeps the bus (`i2c_nack_addr` here, nobody being at
  0x11) and the far controller reports the loss - the far controller
  setting its START on this board's hand-made STOP and not inside the
  window before it, where its tick would have released it alone (the
  section above).
- **The target half answers a real controller.** With the host released
  and `I2cClient<1>` at its own address, the far controller wrote six
  bytes into it: one address match, six bytes byte-exact, the tenure
  closed on its STOP - and the command channel came back after the role
  swap. The same target with a second address and the general call armed
  took the far controller's two-byte write at each, DUALF and GENCALL
  naming which matched (letter `l`).
- **THE TARGET AS A TRANSMITTER** (letter `l`): the far controller read
  one, two, three, four and eight bytes out of this board's data
  register, two reads back to back each, byte-exact both ways; in every
  read the target was asked for exactly one byte more than the host
  clocked - the byte its shifter wants ahead of the wire - which
  `flush()` dropped after the host's closing NACK, ten of ten.
- **The transmit-empty flag of an idle target reads DOWN**, with or
  without a byte in DATAR, so it cannot tell an unclocked byte from an
  idle port; `flush()` belongs where the question is real, after the
  host's closing NACK. The PE cycle it runs SPARES THE CONFIGURATION:
  both own addresses, CKCFGR, RTR and CTLR2 all read back what they
  held, and only ACK has to be put back up.
- **A far target stuck mid-byte** (letter `n`): a read of zeros from the
  far target cut off a few bit times into its first byte by taking this
  host through its reset line leaves that target holding SDA low with
  SCL released high; `unstick()` reported 4 pulses and TIM4 counted 4
  rising edges on the pad, the bus came back free, and the same two
  chips read four bytes byte-exact right after.
- **THE REFUSED LAST BYTE from another chip** (letter `o`, the far
  target refusing the last of one to four written bytes of a
  write-then-read, four times a count, through the pump and the engines,
  at 100 and 400 kHz, the core at 96, 48 and 8 MHz and at 96 again):
  every refusal answered `i2c_nack_data` with the controller let go
  (START and STOP down, MSL clear, 2 ms on) and the next tenure
  `i2c_ok`, the far end reporting each refusal as its own and every
  control exact - 256 refusals and 64 controls a run, ten runs each way.
- **THE MEAN RATE OF A TENURE at each core**, a write of sixteen
  acknowledged by the far target and timed on the core's counter from
  `start()` to the completion, in thousandths of the rate asked, over
  twenty runs: at 96 MHz 990 and 976 to 977 through the pump at 100 and
  400 kHz, 989 and 974 to 975 through the engines; at 48 MHz 987 to 988
  and 965 to 968, 986 to 987 and 962 to 963; at 8 MHz 958 to 961 and 819
  to 831, 952 to 956 and 802 to 813 - where
  400 kHz is CCR 7 at 8 MHz of PB1, 381 kHz and 952 thousandths by the
  arithmetic. Each sits inside the bracket the suite derives: SCL and the
  counter share HCLK; above, half a period of the 153 counted; below,
  the START's lead and the completion's 2.5 periods plus four vector
  services of 180 core cycles at most (the longest entries below) - 965
  thousandths at 96 MHz and 400 kHz, 768 at 8 MHz. The 8 MHz figures are
  the host's own: an 8 MHz core's 180-cycle service is nine periods at
  400 kHz.
- **A held request on the host's channels** (letters `p` and `q`):
  USART2 released with its transmit engine, then this host's DMA read of
  sixteen from the far target on channel 7 byte-exact, its mean rate 972
  thousandths of 400 kHz; this host's read abandoned with RxNE standing
  under DMAEN and released, then USART2's run out whole on the same
  channel in 1002 thousandths of its wire time. Over a transmit channel
  held dead by TIM3's gated compare request a DMA write of 32 is answered
  `i2c_dma_fault` with the far target holding one byte and the count at
  31 - the BTF check - and over a receive channel held dead by a gated
  USART2 a DMA read of sixteen enters the event vector twice, for the
  START and the address, and is answered by the caller's 50 ms bound and
  `recover()`, the thread running throughout; each holder's reset line
  lets its request go and the same tenure runs whole.
- **The late repeated START** - the suite's by-name letter, both ways:
  the last written byte in all twelve probes against the CH32V303VCT6's
  target polled by its own core, and in all twelve against the
  CH32V203C8T6's (the section above).
- **Ten seconds of back-to-back tenures at fast mode**: 25336 tenures
  (25352 on the CH32V303VCT6), 405 thousand bytes moved, not one failure
  at either end and no byte mismatch.

### The host above the wire: `bench_vx03`'s letter `i`

At 120 MHz (PB1 = 60 MHz; the app's 144 MHz has no I2C timing) against
the far chip's target, its `twi_peer` serving a fixed byte, the SCL
period counted on the pad by TIM4 (1200 core cycles at 100 kHz, 300 at
400: the stated rates exactly). `fixed` is wall less the wire (a START
and an Sr a period, nine a byte, the STOP the next tenure's); `irq` and
`isr` are the host's vectors alone. BEFORE is the engine this driver's
last round started from, EVT is WCH's interrupt example
(EXAM/I2C/I2C_7bit_Interrupt_Mode's host handler, its fixed six-byte
tenure made a parameter, through the library) - both measured on the
CH32V203C8T6 with a target served by the same core, so their `fixed` is
not this table's and is left out. At 400 kHz:

| tenure | BEFORE irq, isr | EVT irq, isr | CH32V203C8T6 irq, isr, fixed | CH32V303VCT6 irq, isr, fixed |
|---|---|---|---|---|
| write of 1 | 5, 711 | 7, 1069 | 3, 404, 835 | 3, 326, 767 |
| write of 2 | 6, 830 | 8, 1243 | 3, 423, 835 | 3, 343, 763 |
| write of 3 | 7, 950 | - | 4, 570, 835 | 4, 462, 763 |
| read of 1 | 3, 474 | - | 3, 404, 822 | 3, 363, 821 |
| register read 1+1 | 7, 1021 | - | 5, 696, 1468 | 5, 582, 1393 |
| register read 1+2 | 7, 1045 | 8, 1383 | 5, 722, 1502 | 5, 606, 1438 |
| register read 1+16 | 21, 3390 | 22, 4561 | 19, 2624, 1690 | 19, 2140, 1604 |
| probe (ACK) | 2, 288 | - | 2, 245, 666 | 2, 202, 625 |
| write of 255, engines | 6, 783 | - | 4, 542, x 1.00 | 4, 525, x 1.00 |
| read of 255, engines | 3, 461 | - | 3, 417, x 1.00 | 3, 411, x 1.00 |

- **The pump's handler: 116 to 117 cycles a byte written and 129 to 130
  a byte read on the CH32V203C8T6, 87 to 90 and 97 to 105 on the
  CH32V303VCT6** (120 and 162 before; the EVT's 174 and 227), the stamps'
  own read of the ruler inside each; the hardware prologue is outside the
  stamps. A 255-byte write is x 1.00 on the pump and the engines alike.
- **WHY THE CH32V303VCT6's HANDLER IS CHEAPER ON THE SAME SOURCE: THE
  V4B PAYS MORE FOR A TAKEN BRANCH.** The two release listings of the
  event handler are the same instructions in the same order but for the
  frame - the V4F's body, behind its trampoline, saves ra and three more
  registers, which is MORE work - so the gap is the cores' and not the
  code's. `bench_vx03`'s letter i prices each class the path uses, at
  the letter's 120 MHz, a loop of 1024 turns the best of four: a load or
  a store of SRAM, of PB2 and of PB1 (6.02 cycles a turn on both, the
  bridge's two over PB2's), of the STK's count, a multiply, eight loads
  back to back of SRAM, of the STK and of PB1 (18.1 to 19.3, 18.2 and
  48.0 to 48.5 on both), a turn of 32 aligned or word-straddling 32-bit
  instructions (36.4 to 37.4 on both) - THE SAME ON THE TWO DIES, a
  loop's placement in the image moving a figure by one on either. The
  class that differs is the TAKEN BRANCH: a turn with a forward branch
  taken cost one to three cycles more than the same turn with it not
  taken on the CH32V203C8T6 (two images), none to one more on the
  CH32V303VCT6; taken to a 32-bit instruction that straddles a word it
  cost two more than to an aligned one on the CH32V203C8T6, one more on
  the CH32V303VCT6 - in loops where any predictor is warm. The pump's
  path is a chain of taken tests - the letter's own watch and trace
  flags, the ruler's retry checks in each stamp, the engine's event flags
  one after another - and the ruler itself, the same straight code with
  its taken tests, reads in 32 cycles on the CH32V203C8T6 and 23 on the
  CH32V303VCT6 (letter r, 144 MHz). The QingKe V4 manual lists BHT, BTB
  and RAS for the whole series and says nothing that sets the two apart.
  Not closed: the gap (27 to 32 cycles an entry) is not summed from a
  count of the taken branches on the byte's path, nor is a branch's cost
  with a cold predictor - the handler's case - measured.
- **Where a short tenure's fixed cost goes**, from the timeline of a
  one-byte write at 400 kHz (every entry of the host's vectors, cycles
  from the `start()` call): the START's SB at 657 - the silicon's
  START, about a period and a half past the model's one; the address's
  ADDR at 3543, its entry loading the byte; BTF at 6459, the STOP
  requested and the tenure complete - the end seen at 6766. A target
  served by the host's own core costs the same tenure some 240 cycles
  more (its three entries and the stretch its ADDR puts on the wire,
  measured with the chip's two controllers on one bus); a target of
  another chip takes the bus without them.
- **What went** from BEFORE: the event vector's re-entries on a standing
  BTF (four extra entries a write at 100 kHz, one at 400), the TxE
  entries EVT8_1 saves, `start()`'s read-modify-writes (264 to 140
  cycles), the calls on the hot path (`receive_step`, `ack`, `stop`,
  `clear_addr` were out of line; none is now).
- **The STOP's drain is the next tenure's**: `start()` at a completion
  waits 1032 cycles after a write at 100 kHz and 344 at 400 on the
  CH32V203C8T6, 1055 and 190 on the CH32V303VCT6.
- **The engines hold the core awake**: busy 486 thousand cycles for a
  255-byte write at 400 kHz against the pump's 51 thousand.
- **The longest single entry** of each host vector over the letter:
  events 165 cycles, errors 166, the channels 125 on the CH32V203C8T6
  (183, 170 and 125 on the CH32V303VCT6; 219, 171 and 152 before).

## Not covered yet

Driver gaps:

- **10-bit addressing on the HOST side.** The tenure
  `docs/design/i2c-bus.md` describes carries a 7-bit address on every
  stratum, so the Request has no shape for a 10-bit one and the header
  sequence EVT9 has no user. The target half matches a 10-bit address
  and the register was measured; a controller that sends the two-byte
  header would close the other half.
- **PEC as a tenure shape.** The enable, the next-byte bit and the PEC
  register are on the resource and read back; no Request asks for a
  checksum. Born with the first device that checks one.
- **SMBus above its bits.** ARP, the alert response, the host notify and
  the 25 ms time-out are bits this driver writes and reads; what they DO
  wants an SMBus device, which this bench has not got.
- **The collision case, two targets sharing one address.** An instance
  is a host or a target and never both, so two boards can only ever be
  one of each: the case needs a third party on the wire.
- **A wake from a sleep on an address match.** The peripheral wake-up
  lines of this family are the PVD's, the RTC alarm's and the USB's
  ([pin.md](pin.md)), and what ends a Stop is one of those or a pad's
  ([sleep.md](sleep.md)): no register of this block asks to be one.

Implemented, not bench-verified (each with what would measure it):

- **I2C2 against another chip.** The block is the same design as
  I2C1's (19.12's one register description, table 19-2's second map),
  and I2C2 was measured as host and as target only against the same
  chip's I2C1, its pads wired to I2C1's on one bus; on the bench pair
  its pads reach no wire (PB10/PB11 carry the serial link on
  the CH32V203C8T6 board, and nothing on the CH32V303VCT6 board's), so no
  letter clocks it against another chip. A board whose PB10/PB11 reach a
  peer's I2C would.
- **NOSTRETCH, and the overrun it admits.** The option is written and
  read back; measuring it wants a controller that will not wait, which
  the peer's engine is not.
- **`rebase()` under a `DynamicClock`.** The fan-out is written and
  compiled; no suite drives this chapter under a dynamic clock, and one
  would have to stay inside the 4..60 MHz window at every rate.
- **I2C1's second column, PB8/PB9.** The pads are bonded on both
  boards' parts and the remap is refused where they are not; both
  boards' bus is on PB6/PB7 (and the evaluation board's PB8 carries a
  timer wire), so the column is compiled and never clocked.
- **The SMBus alert pad.** PB5 for I2C1: the driver drives the bit, not
  the pad, and an alert line wants a device that pulls it.
- **The last byte of a read when the wait for its RxNE runs out.** On a
  host with no engines, a read of three or more bytes sets STOP at its
  second BTF, reads byte N-1 and waits for byte N's RxNE with a bound of
  100 000 polls whose answer is discarded: when the bound runs out, DATAR
  is read regardless and the tenure reported `i2c_ok` with what it holds
  (`receive_step()` and `wait_until()` in
  [brio/ch32vx03/i2c.hpp](../../brio/ch32vx03/i2c.hpp)). Whether a
  measured case reaches it is not recorded. At that BTF byte N is
  already in the shifter, so its RxNE follows the read of N-1 and a
  target stretching SCL cannot hold it back; what would show the bound
  running out is a target that never produces the byte (one that
  releases the bus in the middle of the read), or a review of what the
  code does with the wait's result.
