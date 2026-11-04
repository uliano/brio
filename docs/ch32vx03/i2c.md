# I2C (CH32V203, CH32V303)

This document is for a programmer who needs an I2C bus on a CH32V203 or
a CH32V303: a sensor or a memory read and written from an active object
(a component of a brio program with its own event queue, served one
event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)), or this chip
as a target another controller addresses. The peripheral is the STM32F1's
I2C under WCH's register names, up to two two-wire ports, host and
target, 7- and 10-bit addresses with a dual address and the general
call, two speeds with fast mode's two duty shapes, clock stretching,
PEC and the SMBus bits, and two DMA requests each - an event machine
over two vectors per instance. `brio/ch32vx03/i2c.hpp` gives it three
layers: `I2c<n>`, the register block's verbs; `I2cHost<n, pins,
TxEngine, RxEngine>`, the host that the bus arbiter of
util/i2c_bus.hpp drives (the arbiter, `I2cBus`, is an active object
that runs the requests posted to it one after another and replies to
each with an event, util/bus_master.hpp): an active object (the requester)
posts a request, the arbiter runs it as one bus tenure (one START to its
STOP, the bus held for one request) and replies with an `I2cDone`
event when the wire is done
([design/i2c-bus.md](../design/i2c-bus.md)); and `I2cClient<n, pins>`,
the target side (a client is a device on the wire that answers a host),
polled or served from its two vectors. The host moves bytes one interrupt
each through its event vector (the pump), or on the instance's own two DMA
channels when it is given its two engines (the DMA channels bound to it) and
a request names them: every request says which, `I2cEngine::interrupt`
(the default) or `I2cEngine::dma` ("Which engine, at which length",
below, is the guidance).

The smallest use, a register write-then-read at address 0x2C. An excerpt: it
assumes the program's platform type `P` ([platform.md](platform.md)), its clock
object `clock` ([clock.md](clock.md)), an active object `Probe` that handles
`I2cDone` and the static arrays `out` and `in`:

```cpp
using Bus = brio::I2cHost<1>;                       // PB6 SCL, PB7 SDA, no engines
using Arb = brio::I2cBus<Bus, P, 4, brio::BusPassThrough,
                         brio::ticks_from_ms<P>(20)>;

BRIO_CH32_VECTOR(i2c1_ev_handler) {
    if (Bus::isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}
BRIO_CH32_VECTOR(i2c1_er_handler) {
    if (Bus::error_isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}

// In main(), after the clock task's init() and before the kernel runs:
if (!Bus::init(clock)) {                            // PB1 must be 4..60 MHz
    // no timing at this PB1, or a column the part refuses: no bus
}

// In a handler of Probe. out and in are lent until I2cDone.
brio::post<Arb>(Bus::Request{.addr = 0x2C,
                             .tx = brio::lend<brio::Lease::reply>(out), .tx_len = 4,
                             .rx = brio::lend<brio::Lease::reply>(in),  .rx_len = 2,
                             .reply = brio::reply_to<Probe, brio::I2cDone>(),
                             .speed = brio::I2cSpeed::fast_400k});

// In Probe's dispatch, the reply: in holds the two bytes only on i2c_ok.
brio::match(e, [](const brio::I2cDone& done) {
    if (done.status != brio::i2c_ok) {
        // a code of util/i2c_bus.hpp: what the wire did instead
    }
});
```

`Bus` is a type alias for the host over I2C1 on its default column, with no DMA
engines, so every byte is an entry of the event vector. `Arb` is the arbiter
over it: four requests may wait behind the one in flight (a further one is
answered `i2c_rejected` at once, and past ten outstanding the post itself is
dropped with no reply, util/bus_master.hpp), `BusPassThrough` hands every
outcome to the requester as it is, with no retry, and `ticks_from_ms<P>(20)` is
the per-bus timeout in kernel ticks - a tenure that has not answered in 20 ms
is answered `i2c_timeout` and the host `recover()`ed. The limit has to cover the
longest request the program posts at its speed, with whatever a target
stretches: 20 ms covers this one, and the longest request the host takes, a
write-then-read of 255 + 255 bytes, is 46 ms of wire at 100 kHz. `Arb` is listed
in the program's `Tenuto`, the cooperative kernel
([design/kernel.md](../design/kernel.md)), with its other active objects.

The two bindings run under the crt's names for I2C1's two vectors (the crt is
the startup code that runs before `main()`), each posting the status to `Arb`
when the body answers true. `Bus::init(clock)` runs once in `main()`, after the
clock task's `init()` and before the kernel's `run()`; the type of `clock`
states the PB1 rate the timing is solved from. It solves both speeds, hands the
pads to the peripheral open drain, raises the error vector and enables the
instance's two interrupt lines. The `post` runs in an active object's handler:
the address is the 7-bit one, unshifted; four bytes of `out` are written, then a
repeated START and two bytes read into `in`; `speed` asks for 400 kHz.
`lend<Lease::reply>` marks each buffer as lent (kernel/borrowed.hpp): `out` and
`in` stay untouched and in scope from the post to the `I2cDone` that `Probe`
receives, because the vectors read and write them in place, and nothing checks
it. That event's one field, `status`, says what the wire did. The arbiter calls
`Bus::start()` from its own dispatch, in main context, and the tenure then runs
in the vectors.

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
  `unstick()`'s half bits are timed on the STK
  (`brio::Ticker::init(clock)`, [platform.md](platform.md)). Before the
  ticker runs, `start()`'s bounds are counts of polls, longer than named
  (measured, "The two bounds of `start()`" below), and `unstick()`
  clocks its pulses with no half bit between them, `delay_us` refusing
  with the counter stopped (derived from the code, not measured).
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
  floating-point work; the host's bodies do none.
- **Pull-ups on the wire.** The pads are open drain under the peripheral and
  this family's pad has no pull in any output mode, so the bus's
  resistors own the idle level (the bench carries 2.2 kOhm, below).
- **An address of 0x08 to 0x77.** `start()` answers any other
  `i2c_rejected` before the wire - the general call and the 10-bit
  prefixes 0x78 to 0x7B among them - as every host does
  ([../design/i2c-bus.md](../design/i2c-bus.md)). The refusal is what keeps
  such an address off the wire: this event machine recognises the prefix by
  its value, a write's address byte 1111 0xx0 raising ADD10 instead of ADDR,
  and the host would then end the tenure `i2c_bus_error` rather than spin on
  it ("Bench findings").
- **`init()`'s answer read.** False for a PB1 outside the window, for a
  rise time told to `init()` whose TRISE does not fit RTR's six bits, or
  for a column the part's AFIO refuses; the two interrupt lines then
  stay off. A request posted after an `init()` that refused the clock, or a
  first `init()` that refused the column, is answered `i2c_rejected` by
  `start()`, no speed having been solved; a second `init()` refused at the
  column keeps the first's timing, and `start()` does not refuse.
- **Under `dma`, no sleep while a DMA block moves.** In a sleep of any
  depth on this family no bus master but the core gets a cycle, so a
  DMA channel enabled is counted as a working bus master
  (ch32vx03/bus_activity.hpp, [sleep.md](sleep.md)) and the kernel's
  idle path does not sleep while one is; the engines stop each channel
  at its DMA block's completion ("The DMA rows" below). A request that names
  no engine runs on the pump and touches no channel.
- **`dma` asks for engines bound.** On a host given none, `start()`
  answers a request naming `I2cEngine::dma` `i2c_rejected`, nothing on
  the wire; `Bus::has_dma` says at compile time which a host is.

What the host waits for, and where - three places, each bounded:

- **`start()`, in the arbiter's dispatch**, for the last tenure's STOP
  to leave CTLR1 (up to 5 ms), then for BUSY to clear (up to 100 us).
  When the first bound runs out the tenure parks - no START and no
  event vector - and the per-bus timeout is what answers it, nothing with no
  such timeout; when the second runs out the host resets a BUSY left standing
  over an idle wire (one read finding both lines high, "BUSY is the wire"
  below) and sets the START either way.
- **The event vector's entry that clears ADDR**, for TxE: up to two
  polls of at most sixteen STAR1 reads each, before it leaves a byte, or
  the repeated START, to the buffer interrupt.
- **The end of a read of three bytes or more on the pump**, for the
  last byte's RxNE: at most 100 000 STAR1 polls, whose answer the code
  discards - when they run out, DATAR is read regardless and the tenure
  reported `i2c_ok` with what it holds; a stretching target cannot reach
  it, one that lets the bus go in mid-read would ("Not covered yet").

Without the engines nothing in the driver masks interrupts. With them,
`init()`, `recover()` and `release()` stop the channels, and wherever a
DMA block starts or ends - in those three verbs, and in the vectors - the
channel enters or leaves the count of active bus masters under an interrupt
guard of a few instructions
([bus_activity.hpp](../../brio/ch32vx03/bus_activity.hpp)). Separately,
`init()`, `unstick()` and `recover()` turn the instance's two interrupt lines
off while they work, and `release()` turns them off for good: the PFIC's
enable of those two lines, which masks no other interrupt.

## Documents of record

The CH32F/V20x_V30x_V31x reference manual V2.3 (19.1 for what the peripheral
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
CH32V303 - and chapter 19 carries no class note: one peripheral on all three.
Driver: [brio/ch32vx03/i2c.hpp](../../brio/ch32vx03/i2c.hpp). Reference
suite: `test_vx03_i2c` (a suite's letter is one command key of its
console and the group of verdicts it runs; `z` is the key that runs
every letter of the image but `r`, which runs by name only), every
letter on I2C1 and every far end a second
chip running `twi_peer`, the bench's peer firmware (a polled target, or a
controller on command, told what to do over the same two wires) - the
other series of this stratum on the bench,
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
60 MHz. By HCLK:

| HCLK | PB1 | timing |
|---|---|---|
| up to 60 MHz | HCLK | yes |
| above 60, up to 72 MHz | HCLK | none: above FREQ's window |
| above 72, up to 120 MHz | HCLK / 2 (`pclk1_hz_at()`, ch32vx03/clock.hpp) | yes |
| 144 MHz | 72 MHz | none |

The rise-time register says the same from the other side: standard mode's TRISE is (1000 ns x Fpclk) + 1 and its six bits overflow
just past 60 MHz.

### Three registers make the SCL timing, not two

Unlike the CH32V00x's peripheral this one has a rise-time register (RTR,
19.12.9): six bits, reset value 2, holding the longest rise the peripheral
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
host's `isr()`, and the buffer interrupt (ITBUFEN) is switched on and
off inside a tenure because TxE and RxNE are wanted only while the byte
pump needs them.

### The repeated START is requested while the last byte goes out

A write-then-read turns around on a repeated START, and when it is
requested is not free on this silicon. The chapter's moment is EVT8_2:
TxE and BTF, the last byte and its acknowledge done and the host holding
SCL low. Requested there, the last written byte is lost when the chip's
own two controllers are host and target on one bus, the target polled by
the core that also serves the host's vectors. The target acknowledges the
byte and never raises RxNE for it, in silence. On the CH32V303VCT6 that
was measured for the last of one, two, three and four written bytes, at
both speeds, whichever instance was the target, while the same bytes
closed by a STOP all arrived; the CH32V203C8T6's measurement is below. A
register write followed by a read of that register is exactly this shape,
and the register number is what such a target would lose.

A target on another board, polled by a core of its own, did not lose it on
either part (letter `r` of `test_vx03_i2c`, by name): one to four written bytes
with the START requested as soon as BTF was seen and 20 us past it, eight probes
each way, the last byte in every one. A target on the same chip served from its
own vector did not lose it either (below). So three arrangements were measured:
polled by the core that also serves the host, lost; on the same chip on its own
vector, kept; polled on another board, kept. No letter separates the shared core
from the shared die or the bus timing.

Requested a byte earlier - on the TxE that says the last byte went into the
shifter, so the peripheral generates the START at the end of that byte, which
is where WCH's own interrupt example puts it - every byte arrives in every
arrangement. So the host requests it there, under `interrupt` and under `dma`
alike (the transmit DMA block's completion arms the TxE it waits for). A TxE
served later than one byte time still falls back to the BTF order, which a
target on another board takes; no letter times that case against a target
polled by the same core.

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
the byte under way. The host keeps the TxE order all the same. Between
two boards either order delivers; the BTF order fails a chip whose own
second controller is the target, polled by the core that serves the host,
and the TxE order delivered in every arrangement measured. It costs a
refused last byte more handling, which the host takes on: the phase
between the request and SB is its own, and a NACK in it is `i2c_nack_data`.
BTF never rises behind a
pending START on this silicon (polled from the request to SB, and no
entry of the event vector there in `bench_vx03`'s timeline of a 1+2), so
unlike the STM32F4's peripheral nothing is stored into DATAR to hold it down.

The requested START is not withdrawn. The controller generates the
repeated START after the refused byte all the same, and a CTLR1 store at
the NACK that clears START and sets STOP - START is a bit "the user
code" may clear (19.12.1) - races that generation. Measured with the
chip's two controllers on one bus (I2C2 the target, served from its own
vector, refusing the last of one to four written bytes, four times a
count, under `interrupt` and under `dma`, at 100 and 400 kHz), with that
one-store withdrawal: every refusal clean with the core at 96 and at 48
MHz; with the core at 8 MHz (the HSI alone, PB1 8 MHz) at 100 kHz, all
32 refusals answered `i2c_nack_data` and left STOP standing over a
controller still master of the bus (CTLR1 0x201, STAR2 0x7 - MSL, BUSY,
TRA) and the next tenure parked. So the host writes nothing at the NACK: it
waits for the Sr's SB and
closes the tenure with the one sequence EVT5 allows (19.3) - the address
with the write bit, and STOP requested behind it, "after the current byte
transfer" (19.12.1) - a void write, Sr A P on the wire, its ADDR cleared
or its own NACK taken before the reply. Measured the same way: 192
refusals of 192 answered `i2c_nack_data` with the controller let go
(START and STOP down, MSL clear, 2 ms on) and the next tenure `i2c_ok`,
at all three cores, three runs. Counted apart from those: at 8 MHz and 400 kHz
the target, served by the same core, is late with its own NACK in half the
refusals under `dma`, which then take the byte whole and end `i2c_ok`, a limit
of the instrument. Against another chip's target - refusing from its own polled
core, its acknowledge down a byte ahead (`test_vx03_i2c` letter `o`) - every
refusal of every run answered `i2c_nack_data`, the controller let go and the next
tenure `i2c_ok`, on both parts as the host (the counts in the findings below).
The price is the refusal's:
one address byte on the wire; an acknowledged tenure pays for it only
the SB entry's test, a few instructions.

### BUSY is the wire, and it can be left standing

19.12.7 defines BUSY as "SDA or SCL has a low level", cleared when a
STOP is detected. A tenure that ends without the peripheral seeing its
own STOP therefore leaves BUSY standing over an idle wire - 19.12.1's
own case for SWRST, "when no stop condition is detected on the bus but
the busy bit is 1". The host answers it: when its bounded wait for
BUSY runs out and one read finds both lines high, it takes the chapter's reset,
rewrites the timing and the interrupt enables, and issues the START. That read
also passes in a tenure another controller is clocking, at the fraction of a bit
both lines stand high, and the reset is then taken in that tenure (measured, "A
request into another controller's tenure" below).

### A START on a busy bus waits for a tick, a START on a free bus leaves at once, a START under a clock stays

A START set while BUSY stands does not wait for the STOP alone. With SCL
standing high and SDA held low - a line held down, no clock on the wire
- the pending START still reaches the wire, at a moment the controller
chooses for itself. This page calls that moment the controller's tick; it
has nothing to do with the kernel's tick. Letter `h` of the preemptive
suite (below) prices it: a START into a line held low is answered
`i2c_arb_lost` 170 to 350 us from its `start()`, the host's 100 us wait for
BUSY among them.

A START set on a free bus, or pending when a STOP frees it, leaves within
microseconds. Two controllers' STARTs therefore meet where both are pending
when one STOP frees the bus, the rendezvous `test_vx03_i2c` letter `i` uses,
while a START set into a longer hold goes out alone, into whatever holds the
line. That is also why a tenure into a wire another chip holds low is answered
within those hundreds of microseconds: its START leaves at the tick and loses
at its first one bit. Only the first START into the hold does: one set after
it has lost, inside the same hold, stays pending, and only the arbiter's per-bus
timeout answers it (measured under the preemptive kernel, below).

A START set while another controller's clock is running is the third case: **it
does not leave at that tenure's STOP**, and BUSY stands over both lines high until
the per-bus timeout's `recover()` (measured, "A request into another controller's
tenure" below).

### What the host takes from the chapter, and what it declines

Every item that bears on a tenure's cost, measured on the CH32V203C8T6
at 120 MHz (PB1 60 MHz) by `bench_vx03` letter `i`:

- **One data register, no FIFO, no byte counter, no automatic STOP**:
  one interrupt a byte is this peripheral's on the pump.
- **EVT8_1 takes two bytes at once**: TxE rises right after ADDR's clear
  and again after the first byte's store, so the entry that clears ADDR
  loads the data register and the shifter itself, and requests the
  repeated START of a one-byte index there too. Used: a write of two costs
  the three entries a write of one does.
- **The data register before the START**: declined - EVT5 puts the
  address there.
- **The event vector is a level, and BTF stands after a write's STOP**
  until that STOP is on the wire, so a vector left on would re-enter after
  every write and starve the thread meanwhile. ITEVTEN, the event line, is the
  tenure's: raised by `start()`, taken down at the completion with the buffer
  line and the DMA requests in one CTLR2 store (`control2()`). It is down for a
  read under `dma` as well, which the receive channel's completion ends: over a
  channel that does not serve, RxNE and BTF would stand under it and the vector
  would starve the thread and the bus's timeout.
- **The STOP on the last byte's TxE**: declined - no event marks a
  controller's own STOP leaving, so the write ends at BTF (EVT8_2) and
  the STOP's drain is the next `start()`'s, spent in the caller when a
  tenure is started at the completion: 998 cycles after a write at 100 kHz,
  62 at 400. A deferred START would need an
  event this chapter has not got, or a timer and an interrupt a tenure.
- **The receive procedures by count**: used; the faster path that NACKs
  and STOPs on the second-to-last RxNE is declined, its deadline (before
  the last byte's acknowledge) being one no handler of a program
  guarantees.
- **The DMA rows**: used where a request names them, every data half at
  any length. They buy handler cycles from a few bytes on and cost the
  sleep: while a DMA block is in flight the core may not (the bus matrix
  serves the core alone in a sleep), so a 255-byte write at 400 kHz keeps
  it awake for the whole tenure, 692 thousand cycles at 120 MHz, where the
  pump's 256 entries keep it busy some 50 thousand of them and let it
  sleep between them ("Which engine, at which length", below).
- **START cleared by the user code - a requested start withdrawn**
  (19.12.1): declined. Behind a refused last byte the controller
  generates the repeated START all the same, and the withdrawal races it
  - lost with the core at 8 MHz (above); the refusal is closed from the
  Sr's SB with a void write instead.
- **The START itself is slow on this silicon**: SB's entry comes 1404
  cycles after the `start()` call at 100 kHz and 660 at 400 (11.7 and 5.5
  us), the largest single term of a short tenure's
  fixed cost.

### The pads are a column, the alert is not

Table 10-34 gives I2C1 two columns, the default PB6/PB7 and code 1's
PB8/PB9; I2C2 has no remap field at all, one column on PB10/PB11 from
the datasheet's pin table. `I2cPins` carries the code and `init()`
writes it, refused where the part bonds no pad of it. The SMBus alert
pad does not move with the column (PB5 for I2C1, PB12 for I2C2,
datasheet section 3) and is the application's to claim: no verb of this
driver drives it.

### The DMA rows, and the request that names them

Table 11-5 wires I2C1's transmit request to DMA channel 6 and its
receive to 7, I2C2's to 4 and 5. On this family the channel is the
request, so an engine slot naming any other channel is refused at
compile time. Both engines arm at high priority, the transmit level of
[../design/dma.md](../design/dma.md)'s rule and not a receive's: the
host does not overrun - a byte a starved channel has not moved leaves
BTF up and SCL held low until it is (19.3's notes), the bus slower and
nothing lost.

The program names the mechanism of every request: `I2cEngine::interrupt`,
the default, runs every data byte on the event vector and touches no
channel; `I2cEngine::dma` runs every data half on the engines at any
length, and on a host with no engines bound it is answered `i2c_rejected`
by `start()` with nothing on the wire. The address, the repeated START
and the end of the tenure are the event vector's under either name. No
length is beyond the engines on this peripheral: 19.9 sets none in either
direction; a read of two bytes or more runs under CTLR2.LAST, the
controller NACKing the DMA block's last byte (19.12.2); and a read of one
keeps the chapter's one-byte procedure (19.3's EVT6_1) - ACK cleared
before the address, STOP set right after ADDR's clear, both by the event
vector as on the pump - while the receive channel moves the byte. A data half
is the written or the read part of a tenure; a write half's DMA block starts in the entry that takes ADDR, the channel
before DMAEN, and its completion hands the end back to the event vector:
BTF for a plain write, or, a read half to follow, the TxE that says the
last byte went into the shifter, where the repeated START is requested.

An engine checks the run it is handed when it launches a half (starts a
channel's DMA block), and that check can refuse. The half then stays on the
event vector and the request completes. Both launches are made in the
entry that takes ADDR, SCL held until ADDR's clear (19.3's note 1), so
nothing has moved when the refusal is known. A correct program meets this
on the CH32V303 alone, whose DMA1 keeps a transfer inside one 64 KB page on
the lots [dma.md](dma.md) describes. A written half kept in flash can cross
such a boundary, and so can a read half in SRAM on a CH32V303RC or VC whose
option bytes give it more than 64 KB of SRAM (the split FLASH_OBR reports,
RM 32.4.6). A read of two that falls back has POS set before ADDR's clear,
as 19.12.1 asks.

The channel itself is never what is refused, and the host does not ask
whether its engines are free: their channels are this instance's for the
whole image ([dma.md](dma.md), "One owner a channel"), and every way a
tenure ends gives them back.

In sleep the bus matrix serves the core alone on this family, so a
`dma` tenure holds the program awake while a DMA block is in flight - and stops
the receive channel when its DMA block completes, because a channel's EN
stays set after a DMA block and an enabled channel is counted as a working
bus master ([dma.md](dma.md)). The transmit channel's completion still
raises its interrupt here, unlike the SPI host's: on this bus nothing
comes back to prove it, and it is the edge the host acts on. The event vector
ends a write on BTF only behind it. A BTF with the transmit channel's count not
at zero is a stalled channel and not a written DMA block, and ends the tenure
`i2c_dma_fault` with its STOP requested, as does a channel's transfer error:
this host's own status, declared in its header beyond the four wire codes of
util/i2c_bus.hpp (measured: with a request another peripheral left held on
channel 6, the target took one byte of a write of 255, the count standing at
254 at that BTF). A write-then-read's repeated START is requested on the TxE
the completion arms.

`release()` closes the gate and pulses no reset; `init()` opens it and
pulses the line first. A DMA request raised under DMAEN stays held after DMAEN
and PE are cleared, and a tenure abandoned in the middle of a DMA block leaves
one. The channels
are I2C1's for the whole image, so only this host initialized again can
meet it, and the pulse withdraws it. Without the pulse the next read storms
the event vector and never launches; `test_vx03_i2c` letter `s` runs the read
with the pulse
([dma.md](dma.md), "The same owner over its own held request"). A release
after a completed tenure leaves none (the
completion drops DMAEN before a further request can rise), so the hazard is
the release of an abandoned or stalled tenure - the state a timeout leaves.

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

### The host

`I2cHost<n, pins, TxEngine, RxEngine>` is what `util/i2c_bus.hpp`'s
`I2cBus` (= `BusMaster`) drives. Its `Request` is one tenure: a write, a read, a write-then-read with a
repeated START, or the empty probe. `init()` solves both speeds for the
bus clock and claims the pads open-drain; `rebase()` re-solves them
after a clock change; `speed_ok()`, `timing_of()` and `scl_hz()` report
what a speed resolved to. The Request's `engine` field names the
mechanism of its data halves (`I2cEngine::interrupt` by default,
`I2cEngine::dma`), and `has_dma` says whether the host was given its two
engines. `start()` returns true only for the one refusal that moves
nothing - a speed the clock cannot produce, an address outside
0x08..0x77, or `dma` on a host with no engines, answered `i2c_rejected`.
`isr()` and `error_isr()` are the two vectors' bodies and `dma_isr()` the
channels'; `status()` carries the outcome.
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

The register read at the top of this page is the plain use: a request that
names no engine.

With both DMA engines, which are the instance's own two channels, and a
request that names them - an excerpt beside the smallest use's, the
event and error vectors bound the same way for `DmaBus`; `page` is the
caller's array of 64 bytes. There is no DMA call of the program's to
make: `DmaBus::init()` arms both engines, which opens the controller's gate:

```cpp
// At namespace scope:
using DmaBus = brio::I2cHost<1, brio::i2c_default_pins<1>,
                             brio::DmaTxEngine<1, 6>, brio::DmaRxEngine<1, 7>>;
using DmaArb = brio::I2cBus<DmaBus, P, 4, brio::BusPassThrough,
                            brio::ticks_from_ms<P>(20)>;

BRIO_CH32_VECTOR(dma1_channel6_handler) {
    if (DmaBus::dma_isr()) {
        brio::post<DmaArb>(brio::TransferDone{DmaBus::status()});
    }
}
BRIO_CH32_VECTOR(dma1_channel7_handler) {
    if (DmaBus::dma_isr()) {
        brio::post<DmaArb>(brio::TransferDone{DmaBus::status()});
    }
}

// In a handler of Probe:
brio::post<DmaArb>(DmaBus::Request{.addr = 0x50,
                                   .tx = brio::lend<brio::Lease::reply>(page), .tx_len = 64,
                                   .rx = {}, .rx_len = 0,
                                   .reply = brio::reply_to<Probe, brio::I2cDone>(),
                                   .speed = brio::I2cSpeed::fast_400k,
                                   .engine = brio::I2cEngine::dma});
```

A request on this host that names no engine runs on the pump, as on
`Bus`; `.engine = brio::I2cEngine::dma` on `Bus`, which has no engines,
is answered `i2c_rejected`.

A wire whose rise time has been measured, told to the peripheral:

```cpp
if (!Bus::init(clock, brio::I2cDuty::ratio_2, 1000)) {   // 1000 ns of rise
    // that rise's TRISE does not fit RTR at this PB1: no bus
}
```

A polled target at two addresses, answering the general call. `service()`
reports no closing NACK, so the loop asks for it and drops the byte the shifter
asked beyond it (the bench findings):

```cpp
using Peer = brio::I2cClient<1>;
if (!Peer::init(clock, {.own = 0x2C, .second = 0x39, .general_call = true},
                {.no_stretch = false, .interrupts = false})) {
    // no timing at this PB1, or a column the part refuses: no target
}
for (;;) {
    switch (Peer::service()) {
        case brio::I2cClientEvent::addressed:     /* Peer::host_reads() */ break;
        case brio::I2cClientEvent::byte_received: take(Peer::take()); break;
        case brio::I2cClientEvent::byte_wanted:   Peer::give(next()); break;
        case brio::I2cClientEvent::stop:          end_of_tenure(); break;
        default: break;
    }
    if (Peer::host_nacked()) {
        Peer::clear_nack();
        (void)Peer::flush();   // its answer only says whether a byte stood there
    }
}
```

The bus back from a stuck target, and the host back from a lost tenure:

```cpp
const uint8_t pulses = Bus::unstick();   // 0 = never stuck, 0xFF = SDA still low: a short
(void)Bus::recover();                    // always answers true: SWRST, the timing, PE back
```

## Bench findings

`test_vx03_i2c` on the CH32V203C8T6 and on the CH32V303VCT6, both at 96
MHz of HCLK - PB1 = 48 MHz, inside the FREQ window, both speeds exact - each
against the other running `twi_peer` (the peer's PLL on its 8 MHz crystal): I2C1
to I2C1 on PB6/PB7, a 2.2 kOhm pull-up on each line to the CH32V203C8T6 board's
3.3 V, a shared ground
([../boards/weact-ch32v203c8.md](../boards/weact-ch32v203c8.md)). Every
letter both ways, three runs of `z` in each direction with no failure (95
verdicts on the CH32V303VCT6's one image, 57 and 37 in the CH32V203C8T6's
two, the one more being letter `g`'s write across 64 KB, a rule only the
CH32V303 has). TIM4's channel 1 is the second instrument: PB6 is I2C1's clock line
and TIM4_CH1 on its default column at once, so SCL is measured on the pad
that carries it, with no scope and no wire. The figures below are the
CH32V203C8T6's where one is given, the CH32V303VCT6's beside it where
they differ. In this section a letter with no suite named is `test_vx03_i2c`'s.

- **The reset values are table 19-1's, with one exception that is the
  chapter's own**: every register reads zero after the RCC pulse but
  RTR, whose reset value is 2 - the same on both parts.
- **The arithmetic is exact at 48 MHz.** 100 kHz: FREQ 48, CCR 240,
  TRISE 49, and the pad measures a 9989 ns period - 100 kHz. 400 kHz at
  DUTY 2: CKCFGR 0x8028, TRISE 15, and the pad measures 2489 ns - 401 kHz.
  At DUTY 16/9 the register takes CCR 5 and the wire carries 385 kHz, which
  is what rounding up asks for.
- **The enable protection is not there.** 19.12.9 says of RTR that it
  "can only be set when PE is cleared". Measured with PE set, on both
  parts: CKCFGR written 0x8028 reads 0x8028, FREQ written 24 reads 24,
  and RTR written 33 reads 33. All three take a write with the
  peripheral enabled. The driver still writes them with PE clear,
  because that is the order 19.3 prescribes and a read-back is not a
  promise about what the peripheral does with the value.
- **What the pad shows of the wire.** At 100 kHz the capture reads a
  high time of 4937 ns against the 5000 the register asks for and a low
  of 5052; at 400 kHz, 770 ns of high. The tens of nanoseconds missing
  from each high half are what the pull-ups cost on the rise. Released
  from low, SCL and SDA are high again within 594 ns (604 on the
  CH32V303VCT6), an upper bound that includes the pad's own mode switch.
- **SCL runs at the rate asked, read against a clock the bus does not
  share.** TIM4 counts a clock of PB1, the bus FREQ and CCR divide, so a
  period in its ticks moves with a wrong PB1 and cannot tell it; letter
  a reads TIM4's clock on the core's counter first - 96000 kHz, the
  tree's figure - and the letter's three rungs (100 kHz, 400 kHz at
  DUTY 2, 400 kHz at DUTY 16/9) against it in thousandths of the rate
  asked: 1001, 1004 and 963. Against the far chip (letter f), a period
  captured at the end of a write reads the same three, the far target's
  polled core stretching none of those periods.
- **An address nobody answers is `i2c_nack_addr`**, for a probe and for
  a write alike, and the bus is idle again 6 to 7 us after the host
  answers - the time the STOP takes to leave the wire.
- **A tenure into a wire another chip holds down is answered
  `i2c_arb_lost`**, by the silicon, when it is the first: the START, still
  pending after the 100 us the host waits for BUSY, leaves at its tick into the
  held line and loses at the address's first one bit. A tenure started after
  that loss inside the same hold stays pending and is answered `i2c_timeout` by
  the arbiter (the preemptive kernel's section below).
- **`unstick()` is a measurement.** With the far chip holding SDA low
  from its own port and letting go after four SCL falling edges (the
  first being that START's), the verb reported 3 pulses and TIM4 counted
  3 rising edges on the pad, both ways. The STOP it makes by hand raises
  STOPF on the instance's own target half, and that flag stands in
  silence: the event vector is the tenure's (raised by `start()`, down at
  the completion), so it never enters for it, and the next tenure's
  first entry takes it.
- **The command channel is two tenures of the host under test** - a
  write carrying the frame and a read collecting the answer - and it ran
  10 of 10 round trips at 100 kHz each way. That is also where the
  target's BTF stretch falls: after the ninth pulse, or a twenty-byte
  read could never be closed by the controller's NACK.
- **The tenure shapes hold against a second chip.** A write, a read and
  the combined write-then-read all complete `i2c_ok` byte-exact, and the
  far end counts four address matches for the three tenures: the
  repeated START is real, seen from the other side. Reads of one, two,
  three and four bytes - the chapter's four receive procedures - are
  each byte-exact.
- **A polled target that flushes counts no Stop for a read.** 19.4's two
  target sequences end differently - the transmitter's in the closing NACK
  (figure 19-6's EVT3_2), the receiver's in STOPF (figure 19-7's EVT4).
  `twi_peer`'s target takes the NACK and flushes, a PE cycle, before the Stop
  behind it; whether the Stop alone would raise the flag is not separated. So
  `twi_peer` ends a counted serve (a commanded spell as a target, reported
  afterwards) on the wire at rest after its count, never on a Stop: one serve
  of the preemptive suite's letter `o`, nine tenures closed by a read and six by
  a plain Stop, counts seven Stops. A serve armed for eight bytes and a 2000 ms deadline is back in command mode 8 ms after
  the read, the eighth byte its own (letter `d`, ten runs each way).
- **The vocabulary is real statuses across two chips.** A deaf target
  answers `i2c_nack_addr`; a target commanded to refuse the third byte
  answers `i2c_nack_data`; a write to 0x7A and one to the general call
  are refused by the host, `start()` returning true and `i2c_rejected`, and the peer's
  client, answering the general call, takes only the write to its own
  address that follows - one match, two bytes (letter `d`).
- **An address of 0x78 to 0x7B is a 10-bit header here, and the host ends
  it.** Measured on the CH32V303VCT6 with `start()`'s refusal
  compiled out of a scratch image, the CH32V203C8T6's target
  acknowledging 0x7A, a probe, a write of 4 and a 1+2 at both speeds.
  With the address phase's stray branch taken out each tenure stopped in the event vector - STAR1 ADD10, STAR2 MSL and BUSY, the
  vector re-entered 200 000 times before the image's guard cut its line
  - until the per-bus timeout's `recover()`. With the
  branch each tenure ended `i2c_bus_error` in two entries (SB, then
  ADD10), both lines high after, the peer taking no byte, and the next
  tenures sound (a probe of nobody `i2c_nack_addr`, the command address
  `i2c_ok`). The STOP alone does not clear ADD10, whatever 19.12.6's
  table says of a CTLR1 write after a STAR1 read: in a variant without
  the DATAR store after the STOP the flag stood on and the next tenure's
  START never came out (STAR2 0), its vector re-entering on ADD10 until
  the timeout; with the store, STAR1 read 0 after every one. A read of
  0x7A is no header: 0xF5 went out as a plain 7-bit read, `i2c_ok`, the
  peer serving its two bytes - only the write form 1111 0xx0 is one.
- **The cost of the two guarantees.** The refusal is five instructions
  in `start()` on the QingKe V4B. The stray branch adds nothing to a tenure's
  path in the release listing: it is a cold function called only from an
  address-phase entry that found no ADDR.
- **A polled CH32 target's stretch lands where its model puts it.** The
  far end spending 2 ms before each answer (its hold, at the address
  match and before each byte it takes, the receiver one byte late): SCL
  held low 2010 to 2014 us at the address, then 1920 to 1930 us at each
  of six bytes - the hold less the byte its shifter takes meanwhile -
  over six runs, and one eight-byte write whole at 100 kHz in 14 ms of wall
  time against 6 ms for eight unstretched writes of eight; the seventh byte's hold falls after this host's
  completion and the eighth's after the tenure. The tenure completes
  `i2c_ok`: stretching is flow control.
- **The DMA engines carry the shapes, at every length** (letter `g`,
  every request naming its mechanism). A 16-byte write and a 16-byte read on
  channels 6 and 7, the combined tenure, the two-byte read under LAST and
  the one-byte read - its NACK and STOP the event vector's, its byte the
  receive channel's - all complete `i2c_ok` byte-exact, the far end taking
  the 20 bytes written and asked for 23, the bytes read and the one more its
  shifter wants in each read, with no transfer fault on either channel. Which mechanism ran is read off the
  channels themselves, each count set to a sentinel before the tenure and
  each vector's entries counted: under `dma` a write of one, a read of one
  and a 1+1 each leave their channel's count at zero after one completion
  entry, every byte exact at both ends; under `interrupt` a write of 32
  and a read of 32 through the engined host leave both sentinels standing
  and enter neither channel's vector. On the CH32V303VCT6 a write of 16
  from flash across 0x0801 0000, which DMA1 refuses there, completes on
  the event vector, exact, the channel untouched. `dma` on the
  plain host is answered `i2c_rejected` inside `start()`, the far target
  counting no address match, and the same write on the default completes
  as its one match. Under `dma` an address nobody answers is
  `i2c_nack_addr` for a write and a read, a third byte of six refused
  by a device's NACK with the write on the transmit engine is
  `i2c_nack_data`, and the next
  tenure is `i2c_ok`; letter `i`'s losing arbitration round, run again
  through the DMA host under `dma`, is `i2c_arb_lost` - the bus is lost
  in the address byte, before either engine starts.
- **A DMA-served read can leave BUSY standing over an idle wire.** With
  both lines high and no STOP in sight the next START is held for ever,
  which is 19.12.1's own case; the host's SWRST takes it out of the
  way and the tenure that follows completes. SWRST takes CTLR2 with it -
  the interrupt enables have to be written again or the next tenure runs
  with no vector at all.
- **Clearing PE does not let the pads go.** A controller parked at SB -
  a START issued and the address not yet written - holds SCL low, and it
  goes on holding it with PE clear; the peripheral's RCC reset line is what
  frees the wire. That is what the suite's stall report uses when it
  asks which end of the link is holding the clock, and it is worth
  knowing before writing a recovery: 19.12.1's own advice to release the
  pins before a software reset cannot be followed by disabling the
  peripheral.
- **The kernel runs over the host unchanged.** Four queued tenures
  give four replies in order with the NACK from an absent address in its
  place; a fifth and sixth into a four-deep queue are rejected
  immediately and still answered exactly once; an idle bus votes for a
  sleep and a busy one against it. A target holding the clock for 45 ms
  under a 20 ms per-bus limit is answered `i2c_timeout` at 20 ms - the
  one wedge this silicon cannot answer by itself, since outside SMBus
  mode it has no clock-low time-out - and the recovered host carries
  the next tenure to `i2c_ok` through the same arbiter.
- **Arbitration between two CH32 controllers, in both directions and
  from both sides.** With a START condition made by hand to hold both
  controllers' STARTs and released by a hand-made STOP so they leave
  together: aiming at 0x6B against the far end's 0x2C this controller
  reports `i2c_arb_lost` and the far one goes on to meet no target;
  aiming at 0x11 it keeps the bus (`i2c_nack_addr` here, nobody being at
  0x11) and the far controller reports the loss. The far controller sets its
  START on this board's hand-made STOP: set inside the window before it, its
  tick would release it alone ("A START on a busy bus" above).
- **The target half answers a real controller.** With the host released
  and `I2cClient<1>` at its own address, the far controller wrote six
  bytes into it: one address match, six bytes byte-exact, the tenure
  closed on its STOP - and the command channel came back after the role
  swap. The same target with a second address armed took the far
  controller's two-byte write there, DUALF naming the match; a general
  call commanded of the far controller was refused there (its report
  `i2c_rejected`) and this target, the general call armed, matched
  nothing (letter `l`).
- **The target as a transmitter** (letter `l`): the far controller read
  one, two, three, four and eight bytes out of this board's data
  register, two reads back to back each, byte-exact both ways; in every
  read the target was asked for exactly one byte more than the host
  clocked - the byte its shifter wants ahead of the wire - which
  `flush()` dropped after the host's closing NACK, ten of ten.
- **The transmit-empty flag of an idle target reads down**, with or
  without a byte in DATAR, so it cannot tell an unclocked byte from an
  idle port; `flush()` belongs where the question is real, after the
  host's closing NACK. The PE cycle it runs spares the configuration:
  both own addresses, CKCFGR, RTR and CTLR2 all read back what they
  held, and only ACK has to be put back up.
- **A far target stuck mid-byte** (letter `n`): a read of zeros from the
  far target cut off a few bit times into its first byte by taking this
  host through its reset line leaves that target holding SDA low with
  SCL released high; `unstick()` reported 4 pulses and TIM4 counted 4
  rising edges on the pad, the bus came back free, and the same two
  chips read four bytes byte-exact right after.
- **The refused last byte from another chip** (letter `o`, the far
  target refusing the last of one to four written bytes of a
  write-then-read, four times a count, through the pump and under `dma` -
  every written count on the transmit engine, the two read on the
  receive engine - at 100 and 400 kHz, the core at 96, 48 and 8 MHz and
  at 96 again): every refusal answered `i2c_nack_data` with the
  controller let go (START and STOP down, MSL clear, 2 ms on) and the
  next tenure `i2c_ok`, the far end reporting each refusal as its own and
  every control exact - 256 refusals and 64 controls a run, three runs on
  each board.
- **The mean rate of a tenure at each core**, a write of sixteen
  acknowledged by the far target and timed on the core's counter from
  `start()` to the completion, in thousandths of the rate asked, over
  six runs (three on each board): at 96 MHz 990 and 977 through the pump
  at 100 and 400 kHz, 989 and 974 to 975 through the engines under `dma`;
  at 48 MHz 987 and 964 to 967, 986 and 962 to 963; at 8 MHz 958 to 961
  and 821 to 828, 953 to 956 and 803 to 808 - where
  400 kHz is CCR 7 at 8 MHz of PB1, 381 kHz and 952 thousandths by the
  arithmetic. Each sits inside the bracket the suite derives. SCL and the
  counter share HCLK. The upper bound is half a period of the 153 SCL periods a
  write of sixteen takes (nine for the address and nine a byte). The lower bound
  takes off the START's lead and the completion's 2.5 periods plus four vector
  entries of 200 core cycles at most (a write of 16's longest entries are 143 to
  175, `bench_vx03` letter `i` below): 963 thousandths at 96 MHz and 400 kHz, 753
  at 8 MHz. The 8 MHz figures are the host's own: an 8 MHz core's 200-cycle entry
  is ten periods at 400 kHz.
- **A held request on the host's channels** (letters `q` and `s`): this
  host's DMA read abandoned with RxNE standing under DMAEN and the host
  released, then the same host initialized again, reads sixteen bytes in
  step on channel 7, its count at zero. Over a transmit channel
  held dead by TIM3's gated compare request a DMA write of 32 is answered
  `i2c_dma_fault` with the far target holding one byte and the count at
  31 - the BTF check - and over a receive channel held dead by a gated
  USART2 a DMA read of sixteen enters the event vector twice, for the
  START and the address, and is answered by the caller's 50 ms bound and
  `recover()`, the thread running throughout; each holder's reset line
  lets its request go and the same tenure runs whole.
- **Ten seconds of back-to-back tenures at fast mode**: 25346 tenures
  (25354 on the CH32V303VCT6), 405 thousand bytes moved, not one failure
  at either end and no byte mismatch.

### The two bounds of `start()` against their names (`test_vx03_i2c` letter t)

Fifteen runs on each part at the suite's 96 MHz, all green. The time is the ticker's
cycle count around the call, 10.4 ns a count, a read of it some 60 cycles inside each
figure, so a figure reads high by that and never low. Each verdict's bound is the name
below and, above, the name plus the free bus's own figure, one poll, a tick's entry
and the ruler.

| wait | named | CH32V203C8T6 | CH32V303VCT6 | how it was staged |
|---|---|---|---|---|
| free bus | 0 | 145 or 146 cycles | 135 to 144 | nothing standing: the two first looks |
| `bus_free_us` | 100 us | 9815 or 9816 cycles, 102 us | 9802 to 9818, 102 us | SCL held low from this board's pad |
| `bus_free_us` with the reset | 100 us | 9933 or 9934, 103 us | 9908 to 9938, 103 or 104 us | BUSY over an idle wire |
| `stop_drain_us` | 5 ms | 480 153 to 480 162, 5002 us | 480 137 to 480 160, 5001 or 5002 us | a client holding SCL 45 ms behind a write of two, both speeds; parked, no START, no vector entered |

The wait reads what the STK's low half moved (the QingKe V4 manual ch. 5), each poll
adding the difference with one period folded in where the counter restarts from zero,
so a bound spans ticks; it consults no handler, so it holds with interrupts masked,
and a gap of a whole period between two polls is lost from the sum and ends it later,
never earlier. On a free bus the two first looks are one load and a branch each, with
no counter read and no call; the poll is out of line and cold.

**Before the ticker runs** nothing moves the counter, so the wait also spends a
budget, the time asked at eight cycles a poll, which a running counter always beats.
With STE cleared by hand, timed on TIM3 at 1 us a count: `bus_free_us` ends after its
1200 polls in 327 or 328 us on the CH32V203C8T6 and 252 or 253 on the CH32V303VCT6, and
`stop_drain_us` after 60 000 polls in 16.25 ms and 12.5 ms, a poll 26 and 20 cycles. A
program that calls `start()` before `Ticker::init()` has bounds that long.

**Why 5 ms and 100 us.** `stop_drain_us` holds the dispatch of every active object
under the arbiter's kernel and is there for a client digesting a write: 5 ms is a
fifth of SMBus's 25 ms clock-low limit and stands under a 20 ms per-bus timeout with
room for the tenure behind it. `bus_free_us` is there for this host's own STOP, a bit
at 100 kHz: twice SMBus's 50 us ceiling on a clock's high phase.

### A request into another controller's tenure (`test_vx03_i2c` letter w)

`start()` reads both lines once when its wait for BUSY runs out, and takes SWRST if
both are high. In a tenure another controller is clocking both lines stand high in
SCL's high phase on a 1. Letter w stages it: this board's probe of nobody posted into
a tenure of the far board's controller longer than the bound, the moment swept by 7
us, 64 rounds a kind, eighteen runs on each part. At 100 and 400 kHz the far board
writes 24 bytes of 0x55 that this board's own client half acknowledges from the event
vector, there being no third chip on the wire, and `start()`'s store of CTLR1 drops
that acknowledge, so that tenure ends on the byte behind the request either way. At
20 kHz it probes an address nobody answers, nine clocks of 50 us, this block a
bystander. The bound ran out in every round; of 1152 requests of each kind:

| the far tenure | resets, CH32V203C8T6 | resets, CH32V303VCT6 | both lines high by arithmetic and read (either part) |
|---|---|---|---|
| 24 bytes at 100 kHz | 360, 31.2 % | 325, 28.2 % | half of the bit x 4 of 9 bits, 22 %; the thread's reads 26 to 28 % |
| 24 bytes at 400 kHz | 134, 11.6 % | 156, 13.5 % | a third of the bit x 4 of 9 bits, 15 %; the thread's reads 9 to 14 % |
| probe at 20 kHz | 216, 18.8 % | 216, 18.8 % | half of the bit x 4 of 9 bits, 22 %; the thread's reads 26 to 28 % |

So the reset is reached in a live tenure at about the rate chance gives it. Where the
far tenure can tell, this block a bystander at 20 kHz, **two thirds of the resets put
a START into that tenure and end it in `i2c_bus_error` at the far controller**
(144 of 216 on the CH32V203C8T6, 144 of 216 on the CH32V303VCT6); with no reset it
ended `i2c_nack_addr` in every round. The rows at 100 and 400 kHz cannot show that:
the acknowledge is this block's, so a reset's own NACK is what the far end sees.

**And the request itself mostly gets no answer.** Unanswered in the 10 ms the letter
watches, START and BUSY standing over both lines high: at 100 and 400 kHz, this block
addressed by the far controller, every round but 12 on the CH32V303VCT6's 400 kHz row,
which ended `i2c_bus_error`, those with no reset; at 20 kHz 1008 of 1152 on the
CH32V203C8T6 and 1008 of 1152 on the CH32V303VCT6, the others `i2c_nack_addr`. Raw, the
vectors dark, the store swept inside the far probe with this block a bystander at 100
kHz, 1152 stores at each rate: SB rose in none at 100 and 50 kHz on either part, and at
20 kHz in 144 on the CH32V203C8T6 and 144 on the CH32V303VCT6. In the others the far
tenure ended, both lines rose, and START and BUSY stood with no SB for the 3 ms the
letter watches, so what answers such a request is the per-bus timeout, `recover()`
freeing the block. Why that STOP does not take BUSY down is not established.

### The host above the wire: `bench_vx03`'s letter `i`

At 120 MHz (PB1 = 60 MHz; the app's 144 MHz has no I2C timing) against
the far chip's target, its `twi_peer` serving a fixed byte, the SCL
period counted on the pad by TIM4 (1200 core cycles at 100 kHz, 300 at
400: the stated rates exactly). `fixed` is wall less the wire (one period for a START and for an Sr, nine a
byte; the STOP falls in the next tenure); `irq` and
`isr` are the host's vectors alone (and, under `dma`, its two channels').
`x` is wall over wire. The rows that name no mechanism are the plain host's, on
the pump; the rows marked `dma` are the engined host's with every request naming
it. EVT is WCH's interrupt example, from its example package
(EXAM/I2C/I2C_7bit_Interrupt_Mode's host
handler, its fixed six-byte tenure made a parameter, through the
library), measured on the CH32V203C8T6 with a target served by the same
core, so its `fixed` is not this table's and is left out. At 400 kHz:

| tenure | EVT irq, isr | CH32V203C8T6 irq, isr, fixed | CH32V303VCT6 irq, isr, fixed |
|---|---|---|---|
| write of 1 | 7, 1069 | 3, 385, 819 | 3, 308, 744 |
| write of 2 | 8, 1243 | 3, 406, 819 | 3, 329, 744 |
| write of 3 | - | 4, 548, 819 | 4, 448, 742 |
| read of 1 | - | 3, 387, 808 | 3, 323, 741 or 743 |
| register read 1+1 | - | 5, 663, 1391 to 1442 | 5, 553, 1349 to 1375 |
| register read 1+2 | 8, 1383 | 5, 689, 1478 | 5, 586 or 609, 1395 or 1413 |
| register read 1+16 | 22, 4561 | 19, 2523, 1662 or 1670 | 19, 2034, 1541 to 1564 |
| probe (ACK) | - | 2, 233, 656 | 2, 194, 603 or 605 |
| write of 1, `dma` | - | 4, 538, 857 | 4, 501 or 505, 817 or 821 |
| read of 1, `dma` | - | 3, 430, 808 or 865 | 3, 404, 788 |
| register read 1+1, `dma` | - | 7, 949, 1515 | 7, 900 to 905, 1466 or 1470 |
| register read 1+16, `dma` | - | 7, 957, 1461 to 1486 | 7, 908 to 911, 1470 or 1474 |
| write of 255, `dma` | - | 4, 537 or 538, x 1.00 | 4, 508, x 1.00 |
| read of 255, `dma` | - | 3, 438 or 439, x 1.00 | 3, 415 to 422, x 1.00 |

A range is the spread over three runs of the letter; a single figure held
in all three.

- **The pump's handler: 113 cycles a byte written and 126 a byte read on
  the CH32V203C8T6, 88 or 89 and 98 or 99 on the CH32V303VCT6** (the
  EVT's 174 and 227), the stamps' own read of the ruler
  inside each; the hardware prologue is outside the stamps. A 255-byte
  write is x 1.00 on the pump and the engines alike.
- **The CH32V303VCT6's handler is cheaper on the same source, and the reason
  is not closed.** The two release listings of the event handler are the same
  instructions in the same order but for the frame, and the V4F saves more. At
  the letter's 120 MHz a load or a store of SRAM, of PB2, of PB1 or of the STK, a
  multiply, and a turn of aligned or word-straddling 32-bit instructions cost the
  same on the two dies. A forward branch taken cost two cycles more than not
  taken on the CH32V203C8T6 and none more on the CH32V303VCT6, in loops where any
  predictor is warm, and the pump's path is a chain of taken tests. The gap (24 to
  28 cycles an entry) is not summed from a count of those branches, nor is a
  branch's cost with a cold predictor, the handler's case, measured.
- **Where a short tenure's fixed cost goes**, from the timeline of a
  one-byte write at 400 kHz (every entry of the host's vectors, cycles
  from the `start()` call): the START's SB at 660, against the one period
  (300 cycles) `fixed` allows it; the address's
  ADDR at 3544, its entry loading the byte; BTF at 6454, the STOP
  requested and the tenure complete - the end seen at 6757.
- **The STOP's drain is the next tenure's**: `start()` at a completion
  waits 893 to 998 cycles after a tenure at 100 kHz and 62 to 165 at 400
  on the CH32V203C8T6, 925 to 1009 and 69 to 197 on the CH32V303VCT6, by the
  shape; the wait's poll is some 26 cycles, so the figure is the STOP's own time in
  CTLR1.
- **The engines hold the core awake**: under `dma` the idle path does
  not sleep while a channel works, so a 255-byte write at 400 kHz keeps
  the core awake for its whole 692 thousand cycles, where on the pump it is
  busy 50 thousand and sleeps the rest (the CH32V203C8T6; "busy" and its count
  in "Which engine, at which length", below).
- **The longest single entry** of each host vector over the letter:
  events 163 cycles, errors 166, the channels 124 on the CH32V203C8T6
  (187 or 189, 152 or 155 and 129 to 135 on the CH32V303VCT6). The event vector's longest
  is a read's on the pump on the CH32V203C8T6, and on the CH32V303VCT6 the
  entry that takes ADDR under `dma` - the channel started there, the count of working bus
  masters raised, and for a read of one the STOP set: 158 to 187 cycles
  for a write's, 185 to 189 for a one-byte read's, against 139 to 149 for
  the pump's longest. The letter prints each operation's.

### Which engine, at which length

What a program weighs when it names `I2cEngine::dma` instead of the
default, from `bench_vx03`'s letter `i` at 120 MHz (PB1 60 MHz): the
handlers' cycles of one data half - the host's vectors and, under `dma`,
its channels' - which the letter prints at each speed as F, b and E with
the shortest halves read off their own rows: the pump's rows give the
measured figure of each short half and, for the lengths past them, the
model F + b x n, F the pump's fixed part of a half and b its cost a
byte; E is the engines' cost of a half, which does not grow with n.
`start()` is left out of all three: it is the same under both names (139 or 141
cycles on the CH32V203C8T6 and 124 to 134, by the row, on the CH32V303VCT6,
in this letter's image at 120 MHz, a row with a tick inside it apart). The figures
at 100 and 400 kHz agree to within twenty cycles, the CH32V303VCT6's engines to
within thirty-five on a long read:

| a half of n bytes | CH32V203C8T6 | CH32V303VCT6 |
|---|---|---|
| written, the pump | 385, 406, 548 at n = 1, 2, 3; past them F + b x n, F 209, b 113 | 308, 329, 448; past them F 181 to 201, b 88 or 89 |
| written, the engines (E) | 537 or 538 at every n | 501 to 510 |
| read, the pump | 387, 415 at n = 1, 2; past them F 233, b 126 | 323, 345 to 353; past them F 215 or 235, b 98 or 99 |
| read, the engines (E) | 430 to 450 | 404 to 421 |

The engines cost the core fewer cycles from the first length at which E
is no more than the pump's figure. A written half: from 3 bytes on the
CH32V203C8T6 (548 against 537 at 3, 209 + 4 x 113 = 661 against 538 at
4) and from 4 on the CH32V303VCT6 (448 against 505 at 3, 185 + 4 x 89 =
541 against 505 at 4); the pump's first two written bytes ride the entry
that clears ADDR, so a write of one or two costs it about 400 cycles and
the engines 538 (the CH32V203C8T6). A read half: from 3 bytes on both
(233 + 3 x 126 = 611 against 438; 215 + 3 x 99 = 512 against 416), a
read of two being one BTF entry for both bytes on the pump.

**For a write-then-read, name `interrupt` for a short register read, the
smallest use at the top of this page included.** `dma` pays E twice: 1+1 is 949
cycles under `dma` against 663 on the pump, and 1+16 957 against 2523. The
smallest use, four bytes written and two read, has no row: from 1+2's 689 the
pump's 113 cycles a written byte put it at some 1030, against some 955 under
`dma`, within a tenth, and `dma` costs the sleep (below). Behind a one-byte
index the pump grows some 131 cycles a read byte from 1+2's 689 on the
CH32V203C8T6 (103 from 586 on the CH32V303VCT6), which puts the engines ahead
from a read of 5 bytes (6 on the CH32V303VCT6); that length is arithmetic on the
rows, not a row of its own.

The pump does not hold the wire back at this family's top speed: a byte
at 400 kHz is nine periods of 300 cycles at 120 MHz, 2700 cycles, against
the longest event entry's 163 between its stamps (189 on the
CH32V303VCT6), and the 255-byte write runs at x 1.00 of its wire on the
pump as on the engines. Only a slow core stretches the clock behind its
own service: at 8 MHz of core a 400 kHz byte is some 190 cycles, about
what one service takes, and a write of sixteen falls to 803 to 828
thousandths of the rate asked ("The mean rate of a tenure" above).

The energy is the other half of the choice, and on this family it runs
the other way. A channel at work keeps the core awake ("Under `dma`, no
sleep while a DMA block moves" above): the engines take a long half's
handler cycles off the core and keep it out of the sleep for the whole
DMA block, where the pump's one entry a byte lets it sleep between bytes.
The letter's `busy` is the cycles the core spends outside the idle
path's windows, a handler that ends a window counted back in
([../design/benchmark.md](../design/benchmark.md), "Busy"). A 255-byte
write at 400 kHz is 692 thousand cycles of wall at 120 MHz: on the pump
the core is busy 50 thousand of them and sleeps the rest; under `dma` it
sleeps none - the idle path returns at once while the channel works - and the
letter counts 485 thousand of them busy, the turns of its idle loop around an
idle call that does not sleep, the others inside those calls' windows (the
CH32V203C8T6; on the
CH32V303VCT6 busy 42 thousand on the pump, 500 thousand under `dma`). At 100
kHz, a byte four times as long, the wall is 2.77 million cycles: busy 52
thousand on the pump against 1.94 million under `dma` (44 thousand and 2.00
million on the CH32V303VCT6), the handler cycles saved the same. So `dma`
pays where the core has other work to run during a long half, and costs where
the program would sleep through it.

### The stack under the preemptive kernel (`test_vx03_rubato_i2c`)

Measured by `test_vx03_rubato_i2c`, one source and two images, each
board the other's peer with its unchanged `twi_peer` on the same I2C1
pair: `util/i2c_bus.hpp`'s arbiter (pending depth 4, the per-bus timeout
at 20 ms) over the engined `I2cHost<1>` on PB6/PB7 (DMA1 channels 6 and
7) inside Rubato, the preemptive kernel - every Request naming its mechanism, by
the suite's table of shapes for letter `o` and `dma` for the rest - the core at
96 MHz from the crystal (PB1 48 MHz), while TIM2 posts to the highest
object of the pack (the suite's active objects under the kernel) every 97,
193 or 389 us and its dispatch spins 300 or 3000 cycles - at 97 us x
3000, 32 % of the core. Every letter runs once with TIM2 silent (the control)
and six times under it (the pressure passes); **202 verdicts on each image, green on every run:
three runs of `z` on each image, every letter (`n`, `o`, `p`, `q`, `w`,
`h` and `i`) in each**. The pack, the command channel through the arbiter
and the letters are the I2C suites' shared body
(`avrxt/src/apps/rubato_i2c.hpp`, whose header describes them), and
[platform.md](platform.md) places the suite among the preemptive
kernel's.

**Where a preemption can reach this host: before the START, or in
`start()`'s last instructions after it.** `start()` runs in the
arbiter's dispatch, waits there for the last STOP and for BUSY on the STK's
counter, and ends on the CTLR1 store that sets START; a
dispatch above the arbiter that lands in it spends a wait, ending it no earlier, or
delays the START - a gap between tenures - and every byte decision after the
START is the vectors'. Letter `x` counts it on 400 writes of 16 bytes on
the transmit engine, eleven runs: `start()` takes 186 cycles with TIM2
silent on the CH32V203C8T6 and 173 or 174 on the CH32V303VCT6, the suite's
own naming of the mechanism in front of the host's `start()` included (the
Request copied field by field and a bit of its table loaded); under 97 us x
3000 the interrupt landed inside it 4 to 12 times in 400 on the
CH32V203C8T6 and 3 to 12 on the CH32V303VCT6, against about 8 and 7 for
ticks independent of the tenures (400 x 186 / 9312 and 400 x 174 / 9312, the
period 9312 cycles at 96 MHz): what chance gives. In none of them was STAR2.MSL set: a START reaches the wire
microseconds after its store, and
`start()` returns within a few instructions of it. In the letters of `z`
MSL was set in four passes of letter `h`, of the 36 under pressure: there a
START set into the held line leaves at the controller's tick (above), a
preemption after its store lasted until it had left (the reading of the count,
which the suite does not time), and the tenure was then the vector's from its
first bit. The worst post-to-reply of letter `x`'s writes is 400 us silent and
435 to 436 under pressure: one 3000-cycle dispatch.

**Every shape under `interrupt` and under `dma`, under every pressure as in
the control.**
Letter `o`, at 400 kHz against one serve counted to its last byte: the
probe; under `dma` a read of 1 (its NACK and STOP the event vector's,
its byte the receive channel's), reads of 2 and 8 (the receive engine
under LAST), write-then-reads of 1+1 and 2+8 (both halves on the
engines, the transmit block's completion arming TxE for the repeated
START) and writes of 1, 3 and 32 (the transmit engine); under
`interrupt` reads of 3 and 32, write-then-reads of 1+2 and 3+2 (the
repeated START requested from the entry that clears ADDR, or on the last
byte's TxE) and writes of 2 and 8 - every one `i2c_ok`, every read
byte-exact, the peer's count, sum and address matches exact. A shape's
post-to-reply under pressure is the control's plus about one dispatch of 31
us, at most 37 us over the six runs but for one 3+2's 53:
the 32-byte write 759 us silent and at most 767 under pressure, the probe 38 and at
most 73. Letter `p`: a read of 16, a 1+4 and a write of
16 byte-exact at 100 and 400 kHz (the write of 16 in 1553 to 1554 and 399 us
silent against the wire's 1530 and 382), and the peer's commanded stretch of
1000 us a byte
priced from below by its holds - a read of 8 in 9214 to 9237 us against nine
(the address's and the eight gives'), a write of 8 in 8053 to 8120 us
against seven (the address's and six takes': the receiver one byte late,
the findings above) - under the 20 ms timeout.

**The vocabulary, and the two answers of this peer.** Letter `q`: a
deaf client - the peer's client moved to another address, 0x5D -
answers the probe and a four-byte write `i2c_nack_addr` and the probe of
0x5D `i2c_ok`; a write to 0x7A and one to the general call, the peer's
client answering the general call, are refused by the host (`i2c_rejected`) and
the peer counts one address match, the probe that follows; the third byte of six
refused by the peer's NACK (the write on the transmit engine, under `dma`)
answers `i2c_nack_data`; the last written byte of a 2+4 so refused answers `i2c_nack_data` with the
controller let go through its void write; the peer reports each refusal
as its own, and after each action's deadline the next tenure, a probe
of the command address, is `i2c_ok`. Inside a refusing action's window a
probe of the peer gets a NACK on the address, every pass: `twi_peer`
refuses a data byte with CTLR1.ACK set down in advance, the same bit
acknowledges its own address, and nothing puts it back before the
deadline - so the 2+4's void write meets a refused address too, and the
peer counts each refusing tenure's one address match (the suite's
`I2cTraits::refusal_holds_address`, which the shared letter's verdicts
follow).

**The arbiter with three requesters of three priorities.** Letter `w`:
the suite's requester above the arbiter, its `Client` (20 write-then-reads of
1+6 under `dma`,
both halves on the engines, two outstanding), a requester below it (10
writes of two bytes on the pump, both taken in the entry that clears
ADDR) and in a
pressure pass the preempting object itself (8 probes from its tick):
every reply `i2c_ok`, to its requester, in its order - that requester's slot
k complete and slot k+1 untouched at reply k - nothing rejected, no
timeout, the peer's tallies exact, in 5479 to 6307 us a pass.

**The held wire: the first START loses, the next ones stay pending.** Letter `h`
has the peer hold SDA low for 60 ms. The first START into the hold goes
out at the controller's tick (the section above) and loses its first one
bit: the held tenure is answered `i2c_arb_lost` in 170 to 350 us from
its `start()` when it is the first. A START set after one has lost
inside the same hold does not go out - it stays pending, and the arbiter's 20 ms
answers it `i2c_timeout`: under the flood (a probe at a tick whenever
the Console, the object that reads the suite's commands, has turned since
the last, which at 96 MHz is most ticks: 180 to 718 a pass, 711 to 714 of
some 930 ticks at 97 us x 3000) the first START to meet the hold lost -
the held tenure's in 33 of the 36 pressure passes of the six runs - and
two or three probes after it timed out, one after another, until the
release. In the other three, all at 97 us x 3000, a probe met the hold
first and lost, and the held tenure's START, set after that loss inside
the same hold, stayed pending: the arbiter answered it `i2c_timeout`, 19.7 ms
from its `start()` (the limit is counted in whole ticks, and the first may be
nearly over when it is armed). Met at 400 kHz after the commands at 100 (a PE cycle for
the timing first) the hold is `i2c_arb_lost` in 241 to 253 us, every run.
Every request was answered exactly once. While the line was held the answer
was `i2c_rejected` (the pending FIFO full), `i2c_arb_lost` or `i2c_timeout`;
after the release `i2c_ok`, or `i2c_nack_addr` in the moment the peer's client
was not yet back (one request each in two of the 36 passes). In 5 passes of the
36 a request arrived in the arbiter's draining window, between a timeout's
`recover()` and the `BusFlushed` marker event after it, an interleaving only a
requester above the arbiter can make. After the release the next tenure was
`i2c_ok` and the arbiter voted idle with nothing waiting, every pass.
`unwedges()` stayed at zero in every run.

**Dark** (letter `i`): every object idle, the arbiter's vote for the
sleep, no time event armed, TIM2 stopped with every DMA request enable
clear (its channels 2 and 4 are ORed on channel 7), the host between
tenures with the error vector alone and DMAEN and LAST down, both
channels disabled, USART2's gate closed, BUSY clear and PB6 and PB7
reading high on their pads; on the CH32V303VCT6 mstatus.FS Clean, three
runs of three.

## Not covered yet

Driver gaps:

- **A watch of the lines behind `start()`'s reset**, in place of its one read: it would
  keep the reset out of a live tenure ("A request into another controller's tenure")
  and is the CH32V00x host's. Left open until a bus with two controllers is what this
  host is asked for: no brio program has one, and the decision is owed.
- **An answer for a tenure parked with no per-bus timeout**: `start()` writes nothing
  and nothing answers it. The answer a bound would give is the arbiter's own, and a
  program that may meet a client stretching past 5 ms keeps the timeout.
- **This chip as the bus's target, served from an active object, under
  the preemptive kernel.** A target whose bytes come from an object (its
  vector posting "the host reads" and leaving SCL held, a dispatch giving
  the byte) is the one place a held-off reader would stretch the clock;
  no util service serves a target that way (`I2cClient` is served from
  its vector, and the bench peer's `host_read` reads a target served so).
  Born with the first util target object.
- **10-bit addressing on the host side.** The tenure
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
  ([sleep.md](sleep.md)): no register of this peripheral asks to be one.

Implemented, not bench-verified (each with what would measure it):

- **The target's general call** (ENGC, GENCALL): armed and read back,
  never clocked, because every brio host refuses address 0x00, the bench
  peer's included. A controller that sends a general call would measure
  it.
- **The period of the controller's tick.** Letter `h` bounds the first
  START's wait (170 to 350 us from its `start()`) and no letter times the
  tick itself. A letter that holds SDA low by its own port and sets START at
  several delays after that edge would measure it.
- **I2C2 against another chip.** The peripheral is the same design as
  I2C1's (19.12's one register description, table 19-2's second map),
  and I2C2 was measured as host and as target only against the same
  chip's I2C1, its pads wired to I2C1's on one bus; on the bench pair
  its pads reach no wire (PB10/PB11 carry the serial link on
  the CH32V203C8T6 board, and nothing on the CH32V303VCT6 board's), so no
  letter clocks it against another chip. A board whose PB10/PB11 reach a
  peer's I2C would.
- **NOSTRETCH, and the overrun it admits.** The option is written and
  read back; measuring it wants a controller that will not wait, which
  the peer's host is not.
- **`rebase()` under a `DynamicClock`.** The fan-out is written and
  compiled; no suite drives this chapter under a dynamic clock, and one
  would have to stay inside the 4..60 MHz window at every rate.
- **I2C1's second column, PB8/PB9.** The pads are bonded on both
  boards' parts and the remap is refused where they are not; both
  boards' bus is on PB6/PB7 (and the evaluation board's PB8 carries a
  timer wire), so the column is compiled and never clocked.
- **The SMBus alert pad.** PB5 for I2C1: the driver drives the bit, not
  the pad, and an alert line wants a device that pulls it.
- **A read half the receive engine refuses to launch, under `dma`.**
  The receive engine refuses only a run across a 64 KB boundary on the
  CH32V303's DMA1 ("The DMA rows, and the request that names them"),
  which a buffer in SRAM meets only where the option bytes give a
  CH32V303RC or VC more than 64 KB of SRAM; the read then runs on the event vector, a read
  of two with POS set before ADDR's clear. The bench CH32V303VCT6 keeps
  the factory's 64 KB, so no buffer of it crosses and the branch has not
  run; the transmit side's, a span of flash across 0x0801 0000, was
  measured (letter `g`). A CH32V303 so set,
  with a buffer straddling 0x2001 0000, would measure it - a change of
  option bytes the bench's board does not take.
- **The last byte of a read when the wait for its RxNE runs out.** On
  the pump, a read of three or more bytes sets STOP at its
  second BTF, reads byte N-1 and waits for byte N's RxNE with a bound of
  100 000 polls whose answer is discarded: when the bound runs out, DATAR
  is read regardless and the tenure reported `i2c_ok` with what it holds
  (`receive_step()` and `wait_until()` in
  [brio/ch32vx03/i2c.hpp](../../brio/ch32vx03/i2c.hpp)). At that BTF byte N
  is already in the shifter, so a target stretching SCL cannot hold it back.
  A target that releases the bus in the middle of the read would show the
  bound running out.
