# I2C (STM32G0)

This document is for a programmer who needs a device on an I2C bus: a
sensor read, a register written, several devices sharing two wires
through the bus arbiter ([design/i2c-bus.md](../design/i2c-bus.md) is
the model), or this chip answering as a target. `brio/stm32g0/i2c.hpp`
is RM0444 chapter 32 in the two layers every brio bus driver has:
`I2c<n>`, the instance; `I2cHost<n, pins, TxEngine, RxEngine>`, the
transfer engine `util/i2c_bus.hpp` drives; and `I2cClient<n, pins>`,
the target role. `I2cHost`'s Request is `avrxt/twi.hpp`'s and
`samc21/i2c.hpp`'s verbatim, so
`util/i2c_bus.hpp` and `util/bus_master.hpp` drive this engine exactly
as they are written.

The smallest use, a bus of sensors behind the arbiter:

```cpp
constexpr brio::I2cPins bus_pins{
    .scl = {'B', 8, brio::PinFunction::af6},
    .sda = {'B', 9, brio::PinFunction::af6},
};
using I2cHw = brio::I2cHost<1, bus_pins>;
using Sensors = brio::I2cBus<I2cHw, P, 4, brio::BusPassThrough,
                             brio::ticks_from_ms<P>(20)>;

I2cHw::init(clock, brio::I2cClock::hsi16);   // independent of the core rate

extern "C" void I2C1_IRQHandler() {
    if (I2cHw::isr()) { brio::post<Sensors>(brio::TransferDone{I2cHw::status()}); }
}
```

What decides correctness even there: pull-up resistors on both wires,
which are the board's and not the chip's; the two pads named with the
alternate-function numbers the datasheet gives for that instance, which
nothing in the code can check; `init()` after the clock task's, with a
kernel clock chosen (HSI16 keeps the bus rate whatever the core does);
and the instance's vector bound to `isr()`, with the completion posted
to the bus active object as above. A request's buffers are lent until
the reply event arrives ([design/kernel.md](../design/kernel.md), the
loans), and the per-bus timeout given to the arbiter is what ends a
tenure a wedged device holds, because on this silicon a held data line
is a wait and not an error. The chapter is built whole and measured on
the Nucleo's own self-link and, on the same bus, against the desk
partner, an STM32F446RE - the host under the preemptive kernel too -
and on the two smaller parts against a second STM32G0; what is not
covered is listed at the end, with the reason in each case.

## Documents of record

- RM0444 Rev 6 chapter 32 (I2C), and 5.4.21 for the kernel-clock
  multiplexer, 6.1.3 for the fast-mode-plus drive bits and table 65 for
  the wake lines.
- DS13560 Rev 5: tables 13..16 for the alternate functions, table 11 for
  the pad options (`_f` = Fm+ capable, `_s` = supplied from VDDIO2
  only), table 74 for the minimum kernel clock and table 75 for the
  analog filter's delay.
- ES0548 Rev 3 items 2.10.1 and 2.10.2, both live on this die's
  revision, and 2.2.4 from outside the chapter (it bites the wake).

## What the silicon does

**The three instances are not copies of each other**, and table 165's
split is the one fact the whole driver is shaped around. Every instance
does 7- and 10-bit addressing at all three speeds; the **independent
clock**, the **wake from Stop** and the **SMBus half** belong to I2C1
always, to I2C2 only on the G0B1/G0C1 class, and to I2C3 nowhere. The
device header cannot be asked: all twelve headers of the pack declare
the same 253 `I2C_*` macros with the same values, so `WUPEN`, `SMBHEN`
and the whole of `TIMEOUTR` are there on the G030 as on the G0C1. What
the header does distinguish is `RCC_CCIPR_I2C2SEL_Pos`, which exists on
exactly the parts whose I2C2 has the independent clock - so the reserve
(`stm32g0/device_tables.hpp`, the table of what each part has, read off
the device header) derives all three rows from that one probe, and the silicon is asked as
a second opinion (below).

**TIMINGR is the chapter.** Five fields set the SCL period, the data
hold time and the data setup time, and 32.4.5 gives each an inequality
against the I2C standard's own numbers and the bus's own edges. Three
things about it decide the code:

- **tSCL is not tSCLL + tSCLH.** 32.4.9 adds tSYNC1 + tSYNC2 - the edge
  slopes, the filters and two to three kernel periods of synchronization
  - and the example tables charge 250..1000 ns of it. Every term but the
  slopes has a floor - tAF(min) (DS13560 table 75: 50 ns), DNF x tI2CCLK
  and two kernel periods, per edge - and only the slopes are a BUS fact.
  So **a requested rate is a CEILING**: unless told the wire's measured
  tSYNC (`I2cBusTiming::sync_ns`), the chooser charges that floor alone,
  the fastest any wire can make, and a real wire only adds its slopes and
  runs slower; told it, the bus runs at the rate asked.
- **SDADEL has no +1 and the other three do**: tSDADEL = SDADEL x
  tPRESC, while tSCLDEL, tSCLL and tSCLH are all (field + 1) x tPRESC.
- **The kernel clock has three floors and all three bind**: 32.4.3's
  tI2CCLK < (tLOW - tfilters)/4 and tI2CCLK < tHIGH; DS13560 table 74's
  2 / 9 / 18 MHz; and ES0548 2.10.1's 4 / 10 / 20 MHz, **which is the
  strictest in every mode**. The chooser refuses below the erratum's and
  below 32.4.3's, and states the datasheet's as a constant.

**PE = 0 is the only disable, and it is a reset.** 32.4.6: clearing PE
releases both lines, resets the state machines, clears CR2's START,
STOP, PECBYTE and NACK and every ISR flag - and **keeps** every
configuration register. The chapter asks for write-0 / read-0 / write-1
and `disable()` is that sequence; there is no raw PE clear in the file.
It is also the chapter's own deadlock escape (32.4.9), which is what
`recover()` is built on.

**The clear register does not reach everything.** `I2C_ICR` has a bit for
ADDR, NACKF, STOPF and each of the six errors and nothing else: TXIS,
RXNE, TC and TCR are cleared by an ACCESS (a data-register read or
write, a START, a STOP, an NBYTES write) or by PE. On a level-driven
vector that is a trap: a handler that meets one of those flags and
cannot clear it must DISARM its interrupt - see the findings.

**The vector is shared** where the part has an I2C3: I2C2 and I2C3 sit
on `I2C2_3_IRQn`, so an app binds `BRIO_STM32G0_I2C2_HANDLER` and calls
both instances' bodies. I2C1's line is its own everywhere - and it is
also EXTI line 23, the wake, so arming the wake needs the EXTI's mask
and not just WUPEN.

**What the chapter offers a controller's transfer, and what the engine
takes** - the inventory every cost below is read against:

| offer (section) | taken? | why |
|---|---|---|
| NBYTES with AUTOEND (32.4.9) | yes | one CR2 store starts a tenure and the hardware NACKs a read's last byte and sends the STOP; the tenure's frame costs no interrupt but the STOPF |
| TC with AUTOEND clear (32.4.9, 32.9.2) | yes, for a write-then-read alone | the repeated START is the CR2 store at TC; every other shape never raises TC and its enable stays down |
| RELOAD and TCR past 255 bytes (32.4.9) | resource only | the Request's lengths are eight bits on every target (below, "Not covered yet") |
| one TXDR and one RXDR, no FIFO (32.9.10, 32.9.11) | the silicon's | one interrupt per data byte is this block's pump; the engines below are the run's path |
| the first byte written before the START (TXE stands at rest, 32.9.7) | yes | the TXIS for that byte never rises (the TXIS events count NBYTES, 32.4.9): a one-byte write is the STOPF alone, the vendor's library does the same |
| a TXDR flush by writing TXE (32.9.7) | yes, at every start() | a byte an unanswered address left in TXDR refuses the next write (32.9.11) and would go out in its place |
| the last byte of a read left in RXDR | yes | nothing follows it - the controller NACKs it and stops under AUTOEND - so RXIE goes down after the second-to-last and the STOPF entry reads it: a one-byte read is one interrupt |
| NACKIE (table 181) | declined | a STOP follows every NACK by itself (32.4.9), so the STOPF entry reads NACKF from the status register and one entry serves a refused tenure |
| the six enables at their flags' bit positions (table 181, 32.9.7) | yes | the pending mask is CR1 masked and shifted, no test per enable |
| TXDMAEN and RXDMAEN (32.7) | yes, two engine slots | a half of two bytes or more rides a channel and takes no interrupt; a half of one is the pump's, which takes none for it either |
| the independent kernel clock, the Fm+ drive, the wake, SMBus, the PEC, both filters | yes, measured (below) | none of them bears on a transfer's cost but the filters' share of the SCL period |

## Types and verbs

`I2c<n>` carries the whole register description: the enable and 32.4.6's
disable, the configuration (TIMINGR, both filters, NOSTRETCH, SBC, the
general call, the wake, the SMBus enables and the PEC), the two own
addresses with OA2's seven mask codes, the controller's transfer engine
(`transfer()`, `transfer_now()`, `start()`, `stop()`, `reload()`), the
data registers, the flags with their W1C clears, the interrupts, the two
DMA enables, `enables()` - the seven interrupt enables and the two DMA
request enables (`I2cDmaEnable`) as one word in one store -, the SMBus
time-outs and `smbus_probe()`, the wake with its EXTI line, and the
fast-mode-plus drive in both of SYSCFG's flavours.
Every field the register description gates is a verb that refuses, and
the gates are not one rule but four: PE = 0 for TIMINGR, NOSTRETCH,
ANFOFF, DNF and PECEN; START = 0 for CR2's address, direction and
NBYTES; OA1EN = 0 and OA2EN = 0 for their own registers; TIMOUTEN = 0
and TEXTEN = 0 for the two halves of TIMEOUTR.

The timing arithmetic goes both ways and is `constexpr` throughout:
`i2c_timing_for(kernel_hz, speed, filters, bus)` solves 32.4.5's
conditions for a register value, `i2c_scl_hz(kernel_hz, timing, filters,
bus)` prices one on a bus (the ceiling when `bus` carries no measured
tSYNC, the rate itself when it does) and `i2c_scl_hz(kernel_hz, timing,
sync_ns)` against a budget charged as given (the manual's tables),
`i2c_sync_floor_cycles` / `i2c_sync_cycles` name the floor and what is
charged, and `i2c_scll_ns` / `i2c_sclh_ns` /
`i2c_scldel_ns` / `i2c_sdadel_ns` / `i2c_min_stretch_cycles` name the
four field times and 32.4.8's minimum stretch. `i2c_setup_ok()` and
`i2c_hold_ok()` judge a hand-written TIMINGR against the chapter's two
inequalities. The SMBus time-outs have the same pair,
`i2c_timeout_code_for()` and `i2c_timeout_us()`.

`I2cHost<n, pins, TxEngine, RxEngine>` is the engine: one Request is one
bus tenure in the four shapes I2C devices use (write, read,
write-then-read joined by a repeated START, and the empty probe),
asynchronous whenever the wire moves, with the `i2c_*` codes produced
on the wire; the two failures that move nothing - a speed this kernel
clock cannot produce (`i2c_rejected`) and a START still standing from
a tenure the peripheral never closed (`i2c_bus_error`) - complete
inside `start()` and reach the requester through the arbiter all the
same.
`init(clock, kernel, filters, bus)`, `rebase(hz)` (a ClockUser),
`speed_ok()`, `scl_hz()`, `start()`, `isr()`, `dma_isr()`, `status()`,
`recover()`, `unstick()`, `fast_plus_drive()` and
`spurious_bus_errors()`. A tenure writes its interrupt and DMA enables
in one store at `start()` and puts them back to ERRIE alone at its end,
so nothing one tenure armed or dropped reaches the next. The two DMA
engine slots default to `NoDmaEngine` and every DMA branch folds away.
Named, they carry the DATA of a tenure (32.7: the address cannot be
moved by DMA) - each half of two bytes or more; a half of one byte, or
one with no buffer, stays on the pump - and both are armed for their
errors alone: the tenure's own STOP after NBYTES proves every byte went
through TXDR and RXDR, so a tenure takes the I2C's interrupts and no DMA
one, and `finish()` completes the transmit engine on that proof
([dma.md](dma.md), "Two moments"). BOTH ARM AT `high` (docs/design/dma.md):
a byte the receive channel has not taken leaves RXNE up and holds the
next in the shift register with SCL stretched until RXDR is read, and a
byte the transmit channel has not written stretches SCL after the ninth
pulse until TXDR is (32.4.7) - a starved channel slows the bus and loses
nothing. The resource's `release()` - the host's and the client's -
pulses the block's reset before the gate, so a DMA enable a tenure left
set cannot raise its request again for the next owner of the line
([dma.md](dma.md), "A released requester holds nothing here").

`I2cClient<n, pins>` is the target: `init(clock, addresses, speed,
kernel, filters, no_stretch)`, `addressed()`, `host_reads()`,
`matched_address()`, `answer_address()`, `take()`, `give()`, `flush()`,
`answer_byte()` for byte control, the STOP/NACK/overrun flags,
`wake_from_stop()` and the ISR body. A client is a protocol and the
protocol is the application's, so this half decides nothing.

## How to use it

```cpp
constexpr brio::I2cPins bus_pins{
    .scl = {'B', 8, brio::PinFunction::af6},
    .sda = {'B', 9, brio::PinFunction::af6},
};
using I2cHw = brio::I2cHost<1, bus_pins>;
using Sensors = brio::I2cBus<I2cHw, P, 4, brio::BusPassThrough,
                             brio::ticks_from_ms<P>(20)>;

I2cHw::init(clock, brio::I2cClock::hsi16);   // independent of the core rate

extern "C" void I2C1_IRQHandler() {
    if (I2cHw::isr()) { brio::post<Sensors>(brio::TransferDone{I2cHw::status()}); }
}
```

Each speed asked is a ceiling the wire stays at or under. A bus whose
detection delay has been measured (letter `d` prints it: the period less
the register's tSCLL + tSCLH) states it, and runs at the rate itself:

```cpp
I2cHw::init(clock, brio::I2cClock::pclk, {},
            brio::I2cBusTiming{.rise_ns = 300, .fall_ns = 300,
                               .setup_ns = 100, .sync_ns = 430});
```

A request is the complete script of one tenure:

```cpp
uint8_t reg = 0x0F;
uint8_t id = 0;
brio::post<Sensors>(I2cHw::Request{
    .addr = 0x1D,
    .tx = brio::lend<brio::Lease::reply>(&reg), .tx_len = 1,
    .rx = brio::lend<brio::Lease::reply>(&id),  .rx_len = 1,
    .reply = brio::reply_to<MyAo, brio::I2cDone>(),
    .speed = brio::I2cSpeed::fast_400k,
});
```

A target answers on its own vector:

```cpp
using Peer = brio::I2cClient<2, peer_pins>;
Peer::init(clock, {.own = 0x42}, brio::I2cSpeed::fast_400k,
           brio::I2cClock::hsi16);
extern "C" void BRIO_STM32G0_I2C2_HANDLER() {
    const uint32_t f = Peer::isr();
    if (f & brio::I2cFlag::addr)  { Peer::answer_address(); }
    if (f & brio::I2cFlag::rxne)  { store(Peer::take()); }
    if (f & brio::I2cFlag::txis)  { Peer::give(next()); }
    if (f & brio::I2cFlag::stop)  { Peer::clear_stop(); }
}
```

**A pump must be able to silence everything it is asked about.** ADDR,
NACKF, STOPF and the errors go through the ICR; RXNE is cleared by
reading RXDR; TC and TCR have no clear at all outside a START, a STOP or
an NBYTES write - so a handler that meets one it does not own must
DISARM its interrupt. The driver's own idle sweep does exactly that and
the reason is in the findings.

## Bench findings

Measured by `test_stm32_i2c` on the Nucleo-G0B1RE's desk, 190 verdicts
in two sets, green ten times running in one boot (the letters named
below are the suite's console keys, each the group of verdicts it
runs). The findings down to
"ES0548 2.10.2 did not fire once" are the SELF-LINK's (I2C1 host
PB8/PB9, I2C2 client PA11/PA12, both AF6, 2.2 kOhm pull-ups): `z`,
letters `a`..`m`, **155 verdicts**. The ones under "The peer bus" are a
SECOND CHIP's, the STM32F446RE running `twi_peer` on the same two nodes:
the second all-key `w`, letters `n`..`t`, **35 verdicts**.

**The enable protection is REAL here, which is the opposite of the SPI's
answer.** A raw write with PE set lands on NONE of the four PE-gated
fields - TIMINGR, NOSTRETCH, ANFOFF and DNF all refuse - where
`test_stm32_spi` measures the silicon enforcing not one line of 35.5.7.
Two chapters of one manual, two different dispositions.

**The silicon names its own column of table 165.** 32.9.6 makes
`I2C_TIMEOUTR` "reserved, and its bits forced by hardware to 0" on an
instance without SMBus, so a write that reads back is the peripheral
saying which column it is in: I2C1 yes, I2C2 yes, I2C3 **no** - exactly
what the reserve's stated table says, on a question the device header
cannot answer.

**A repeated START must be ONE CR2 store, START included.** 32.9.2 gives
START two meanings and says nothing about the order of the two writes it
takes to get there, but the order decides: at TC the previous transfer
is finished and UNTERMINATED, so a store that raises AUTOEND before
START is read as a request to end THAT transfer - the peripheral sends
the STOP at once and START, a cycle later, opens a new tenure instead of
restarting this one. On the ISR trace: TC then STOPF with **no
second address match at the client**, where a repeated START owes two.
Written as one store, the client sees two matches and one STOP.

**RXNE must be served BEFORE STOPF.** They stand together - the last
byte of a read is in RXDR at the instant the automatic STOP goes out -
and RXNE is cleared only by reading RXDR. A handler that serves STOPF
first loses that byte AND leaves RXNE standing on a level-driven vector,
which is an endless handler that starves the program so completely that
nothing can report it.

**NACKF and STOPF are two events microseconds apart.** The STOP the
peripheral sends by itself after a NACK raises its own flag later; an
entry that served NACKF and then swept the status register would clear
that STOPF the moment it set, and the tenure would keep its status, lose
its flags and never complete - the symptom an order dependency between
letters, which is what a race usually looks like first. The engine
takes no NACK interrupt at all: the STOPF entry reads NACKF, which
stands until cleared, and names the refusal - the address's when the
preloaded byte never left TXDR (TXE still clear), a byte's when it did.

**Three flags no branch owns.** PECERR, TIMEOUT and ALERT ride the one
ERRIE a plain I2C engine arms for BERR and ARLO. They are swept at the
bottom of the handler, and the sweep touches nothing a branch owns: a
flag that sets between the handler's read of the status and the sweep -
a TXIS, a byte in RXDR, the TC that ends a write-then-read's write half -
is the next entry's to serve. What a sweep cannot reach is disarmed, and
inside a tenure that is TCR alone, which only RELOAD raises and which the
engine never asks for. TC is never disarmed there: RM0444 32.4.7 keeps
SCL stretched as long as TC stands, so a disarmed TCIE would be a lost
repeated START and a tenure left to the per-bus timeout. ERRIE is never
disarmed either: all six error conditions have a clear bit, so an error
that sets after the handler's read is cleared on the next entry, and
ARLO and BERR go on being reported for the life of the program. TCIE is
armed afresh by every tenure's `start()`.

**A held SDA is a PARK and not an error**, the same answer the AVR DA/DB
and the SAM C21 give. A controller whose SDA is held low by
another device raises no ARLO and no BERR: 32.4.9 makes the START wait
for a free bus, a low SDA under a high SCL is a START the monitor has
already seen, so BUSY stands (host ISR `0x8000`, BUSY alone), the
request is parked and there is no completion to report. Which is why
`util/i2c_bus.hpp`'s per-bus timeout is the ARBITER'S on all three
targets.

**ES0548 2.10.1 costs more than the datasheet's floor, and the
independent clock is the cure.** The erratum's 4 / 10 / 20 MHz beats
table 74's 2 / 9 / 18 in every mode. Measured on the ladder: at a 64 MHz
core all three speeds are reachable on PCLK; at 16 MHz Fm+ is refused; at
**2 MHz NO SPEED IS LEGAL ON PCLK AT ALL** - and the same bus runs
byte-exact at 100 kHz with the instance's kernel moved to HSI16. That is
what the independent clock is for, on the wire.

**And the wake and Fm+ are mutually exclusive**, which no table says: a
target's SDADEL and SCLDEL are solved against the fastest bus it expects,
so a client is subject to the same 20 MHz floor - and HSI16 is 16 MHz,
while the wake from Stop accepts HSI16 and nothing else (32.4.16).

**A requested rate is a ceiling, and the oracle's would not be.** This
wire's own tSYNC, measured as the difference between a tenure's average
period and the register's own tSCLL + tSCLH (a 32-byte tenure, duration /
(9 x 33), the target in NOSTRETCH so it holds the clock for nothing, at
64 MHz on PCLK), is **422 to 485 ns** at the three speeds on the desk's
wire (the self-link's two nodes shared with two more boards' taps,
2.2 kOhm pull-ups) - against the 1000 / 750 / 500 ns the manual's tables
charge, so a chooser solved against those runs this wire FAST: 104.7 kHz
for 100, 457 kHz for 400, 1.09 MHz for 1 (the register halves of that
budget, 9062 / 1750 / 499 ns, plus the measured delay). The default therefore charges the FLOOR
under tSYNC - two kernel periods, tAF(min) and DNF x tI2CCLK an edge,
10 kernel cycles (156 ns) at 64 MHz with the analog filter on - and the
three rungs become 100 / 400 / 1000 kHz at most on any wire; on this one
letter `d` measures 96.8, 359.6 and 790.5 kHz, none above its speed.
Stated to `init()`, the measured delay brings the 100 kHz rung to the
rate asked within one per cent (measured); the arithmetic gives 400 and
970 kHz for the other two - the last held under the rate by table 171's
tLOW, which the chooser keeps with the floor's edge rather than trust a
split of the stated delay between the two edges.

**THE RATE ASKED, JUDGED ON THE WIRE.** A self-link's target follows the
controller's clock, so it carries every byte whatever TIMINGR made of
the rate, and only the stopwatch can see a wrong one: every letter that
sets an SCL rate times its tenure (9 SCL periods a byte and the address,
START, STOP and the request's own cost inside the reading) against the
speed the request NAMED and judges it - never above it by more than
2 %, and within a quarter below it, the wire's tSYNC (above) being what
only lengthens a period. On the STM32G0B1RE, in thousandths of the rate
asked:

| letter | tenure | rate asked | reading |
|---|---|---|---|
| `d` | 32 bytes, the target in NOSTRETCH, 64 MHz core on PCLK | 100 k, 400 k, 1 M | 967, 897, 782 |
| `d` | 32 bytes on the 64 MHz and the 16 MHz rung, PCLK | 100 k | 969, 928 |
| `h` | 300 bytes through RELOAD | 400 k | 920 |
| `h` | 64 bytes on the transmit engine | 400 k | 909 |

Each lands under its speed by this wire's tSYNC, which the default
charges at its floor - the gap a stated `sync_ns` closes (the 100 kHz
rung brought within a per cent, above); the 16 MHz rung's 928 is the
same register shape on a kernel a quarter as fast, its fixed costs a
larger share. AT A 2 MHz CORE THE STOPWATCH CANNOT TIME SCL: with the
kernel on HSI16 the tenure takes 0.64 of the 100 kHz wire because the
controller holds SCL low until its TXIS entry refills TXDR and the
target stretches until its own entry reads RXDR (32.4.9, 32.4.17), both
served by the one 2 MHz core - and no pad is spare for SCL's edges. That
rung's rate rests on the register instead: its TIMINGR on HSI16 is
0x1091222B, the very word the 16 MHz rung timed at 928 on a 16 MHz
kernel, and the letter checks the identity.

**ST's own arithmetic is the oracle, and its error runs FAST.** The HAL
takes a precomputed TIMINGR; ST computes it in CubeMX and in the timing
utility its STM32Cube examples ship (`I2C_GetTiming()`, the
`i2c_timing_utility.c` of STM32CubeH7's and STM32CubeU5's I2C examples,
and the same function in their boards' BSP bus files). That method
charges each half tAF(min) (50 ns, fixed in the source for every part) +
tDNF + 2 x tI2CCLK, plus a "typical" rise and fall of 640 + 20 ns (Sm),
250 + 100 (Fm) and 60 + 100 (Fm+), and keeps the candidate CLOSEST to the
rate asked inside a band of plus or minus 20 per cent - so it lands on
either side of the rate on its own model, and on a wire with faster edges
than it assumes, above it. At 64 MHz it answers 0x60702729, 0x10A11626
and 0x00610E1A: on the fastest wire 109.6 kHz, 477.6 kHz and 1.23 MHz;
on this one (its halves plus the measured 406..438 ns) 106.3 kHz, 426.7
kHz and 928 kHz - two rungs of three above the rate asked, the Fm+ one
below it because this wire's 422 ns exceed the 322 its typical edges
make. It also answers an Fm+ word at 16 MHz, where ES0548 2.10.1 asks
20. The two methods agree on everything but that charge -
the setup and hold bounds and the 32.4.3 conditions are the same
inequalities - and `i2c_scl_hz()` pins ST's words at the bottom of
`brio/stm32g0/i2c.hpp`.

**At 1 MHz this core cannot serve a NOSTRETCH target.** The window is
nine microseconds - the byte before the one being missed - and an
interrupt-driven target does not always make it: 32.4.17's automatic
NACK on an overrun follows, and the controller reads `i2c_nack_data`.
With the target stretching, the same 1 MHz bus is byte-exact.

**Clock stretching is linear and free of the data.** Commanded holds of
20 / 50 / 100 us per event lengthen an 8-byte read by nine holds a step
(the address event and eight bytes): from 20 to 50 us the tenure grew
281 us against 270, from 50 to 100 by 469 against 450, byte-exact
throughout. Each tenure runs about 13 us a byte short of the baseline
plus nine holds - 282 / 563 / 1032 us against 422 / 692 / 1142 - because
a commanded hold lands on top of the client's own turnaround, its ISR's
entry and read, which already stretch SCL in the unstretched baseline;
the suite judges the slope, where that constant cancels. With NOSTRETCH set, a
late target raises OVR and **0xFF goes out in the missing byte's place**,
exactly as 32.4.8 says, while the tenure itself completes - an underrun
is not a fault the controller sees.

**Whose hold does each time-out police?** With a control on each side:
the host's own unserved hold trips TIMEOUTA at a 4 ms limit (ISR
`0x1023`), and a **peer's 6 ms hold does not trip it** - nor does
TIMEOUTB, which 32.9.6 makes this controller's own cumulative stretch
(tLOW:MEXT). So 32.4.12's "if SCL is tied low" is this controller's own
hold, and **the SAM C21 answers the same way**: no silicon time-out on
any engine of this project watches a wire a client wedged. TIDLE = 1 is
accepted and reads back, but bus idle detection did not raise TIMEOUT on
a bus idle for 2 ms against a 50 us limit - recorded, not judged.

**The PEC is the standard's CRC-8, pinned against a bitwise reference**:
hardware `0x3D` against `0x3D` computed with C(x) = x8 + x2 + x + 1 over
the address byte and the four data bytes, with the checksum travelling as
a fifth byte the target receives.

**RELOAD past 255 works as the chapter describes**: a 300-byte write goes
out in one tenure (one address match) through exactly one TCR reload,
byte-exact. With AUTOEND clear, TC stands with no STOP on the wire and a
software STOP ends it.

**10-bit addressing**: acknowledged and byte-exact, and **ADDCODE is the
HEADER and not the address** - `0x79` for the 10-bit address `0x155`,
which is 11110 plus the address's two MSBs (32.9.7). A client with
several addresses cannot simply compare ADDCODE with its own.

**OA2's mask is a real range**: OA2 = 0x50 with `low_2` answers 0x50,
0x51, 0x52 and 0x53 and stops at 0x54, and ADDCODE names the address
that matched.

**The digital filter is hold delay.** A deeper DNF never asks for MORE
SDADEL - 32.4.5 subtracts it from the bound - and at 400 kHz on a 64 MHz
kernel SDADEL falls 7, 7, 5, 3 for DNF 0, 1, 4, 8 with the SCL period
unmoved. A filter deep enough to eat the low period is refused by
32.4.3's own condition, not by an arbitrary bound.

**unstick() reads the wire before it clocks it**: 0 on a healthy bus,
`0xFF` when nine clocks and a STOP leave SDA still low (a short, not a
client), and 0 again once the pad is given back.

**`util/i2c_bus.hpp` over this engine**: four tenures queued through
`I2cBus` come back in order with the NACK delivered in its place as a
reply; what will not fit the queue is rejected on the spot and every
request is still answered exactly once; an idle bus votes for the sleep
and a busy one against it; and a tenure into a wedged wire is answered
**i2c_timeout at 20 ms, the arbiter's own limit, with SDA still low at
the reply** - after which `recover()`'s PE cycle lets the same bus AO
carry the next tenure to `i2c_ok`.

**ES0548 2.10.2 did not fire once.** The spurious master BERR was swept 0
times across a whole `z` run. The driver counts it and never reports it -
the erratum's own workaround - so `i2c_bus_error` from the engine means
its own bounded wait ran out and never a flag this erratum can forge.

Two facts about the bench itself: **a long wait inside an
interrupt must be a counted loop**, because a stopwatch built on SysTick's
VAL against the tick count runs backwards when a tick handler cannot
preempt the interrupt reading it; and **a wire letter needs a flushed
marker between its steps**, because an I2C storm starves main so
completely that a letter which prints only at its end reports nothing at
all.

### The host's cost (bench_stm32 letter i)

Letter `i` of `bench_stm32` runs every tenure shape on the self-link
(I2C1 the host on PB8/PB9, I2C2 the client at 0x42 on PA11/PA12 from
its own vector, both kernels on PCLK at 64 MHz, 2.2 kOhm pull-ups):
writes and reads of 1, 2, 16 and 255 bytes, the register read (one
written, a repeated START, 1, 2 and 16 read), the probe answered and
not, a write to the address nobody answers - through the pump and
through the engines (DMA1 channels 2 and 3), at the three speeds, the
data judged after every run. `wire` is the bus's own time at the SCL
period MEASURED as the slope of two engined writes (255 and 16 bytes),
nine periods a byte with the address and one for the START and the
STOP; `the rest` = wall - wire is the fixed cost of a tenure, the
instrument's own share inside it (a stamp pair a handler, an idle turn
or two, the ruler read in the Stopwatch: about 400 cycles, the same in
every column). The vendor's column is ST's HAL v1.4.7 on the same board,
the same client and the same TIMINGR words (`HAL_I2C_Master_Transmit_IT`,
`_Receive_IT`, `Mem_Read_IT`, a zero-length transmit as the probe, the
DMA variants), CubeMX's G0 handler shape on the vector. At the 400 kHz
rung - 369.9 kHz on the desk's wire under the ceiling default for brio,
the HAL's TIMINGR words solved against the manual's budgets:

| tenure | the rest (cycles), interrupts: brio | ST HAL |
|---|---|---|
| write 1 | 1028, 1 | 1362, 1 |
| write 2 | 1031, 2 | 1361, 2 |
| write 16 | 1040, 16 | 1374, 16 |
| read 1 | 1228, 1 | 1545, 2 |
| read 16 | 1248, 16 | 1560, 17 |
| register read 1 + 1 | 1651, 2 | 2289, 4 |
| register read 1 + 16 | 1670, 17 | 2313, 19 |
| probe answered | 1128, 1 | 1393, 1 |
| probe not answered | 816, 1 | - |
| write 255 through the engine, x | 1.00 | 1.00 |

- **The interrupts are the silicon's floor.** N for a write or a read of
  N bytes, two for a one-byte register read, one for a probe either
  way: the preload, the last byte at the STOP and the NACK at the STOP
  each took one out, and the engines take every data byte - an engined
  tenure is one interrupt (two for a register read), the STOPF (and the
  TC).
- **The pump's entry is 127 cycles a byte** (169 between the meter's
  stamps, 42 of them the stamps' own), where it was 163 and ST's is
  168; at 400 kHz a byte is 1193 cycles of wire, so the pump costs a
  tenth of the core and keeps the wire's pace (x 1.00 at 255 bytes).
  The decode is one mask from CR1 (the six enables sit at their flags'
  positions) and one test for the six errors.
- **`start()` is 228 cycles** (290 on the ruler, one read of it 62),
  where it was 371: the tenure copies the five fields its entries read
  and not the whole Request, and the CR2 word is built inline.
- **The engines' fixed cost** is the channel programmed at `start()` (432
  on the ruler where the pump's is 290) and one fewer branch in the
  STOPF entry; from two bytes a half they save an interrupt a byte, and
  a 255-byte write or read runs at x 1.00 at 400 kHz and 1.01 at the
  1 MHz rung.
- **At the 1 MHz rung the pump is the core's limit, not the wire's**:
  both ends of the self-link are this core's interrupts, about 300
  cycles of handler for a byte of 474 - a 255-byte write at x 1.11
  (1.28 before), the engines at 1.01.
- **Against ST's HAL**: every shape within a fifth and ahead - the HAL
  preloads too, but its entry is the generic `I2C_Master_ISR_IT` (168
  cycles a byte, 403 for its STOPF entry against 242), it reads a
  one-byte read through an RXNE entry, and its register read takes four
  interrupts to this engine's two.
- **The pump takes no placement** ([platform.md](platform.md), "Code in
  SRAM"): its entry through util/placement.hpp's trampoline - built and
  measured in a scratch build, the same letter, a 255-byte tenure at
  100 kHz - took a written byte 167 cycles as an `InRam` instance behind
  the vector in the flash against 161 flattened inline in it, and a read
  one 176 against 178: the veneer and the instance's own frame eat what
  the fetch saves on a body this short, so `I2cHost` has no placement
  parameter and an `InRam` given to it does not compile. The engined
  tenure's STOPF entry, once a tenure, came out 39 cycles shorter from
  SRAM. A program that wants the pump out of the flash places its
  VECTOR in `.fast_text`, where the inline pump runs whole with no call:
  124 cycles a written byte and 137 a read one against 169 and 176 from
  the flash (`bench_stm32` with its vectors in `.fast_text`, a scratch
  build).
- **The rungs are measured as the slope**: 98.0 kHz, 369.9 kHz and
  827.1 kHz on the desk's wire under the ceiling default; the HAL's
  column was taken at the words solved against the manual's 1000 / 750
  / 500 ns of tSYNC, 106.5 kHz, 482.8 kHz and 1.216 MHz on the same
  board. `the rest` is a handler's and a channel's cost and the wire's
  share is taken out of it, so the two columns compare across the
  rungs.
- **The 1 MHz rung is not claimed as Fm+.** Released from low, SCL
  reaches the input's threshold 8 cycles and SDA 12 cycles after the
  same steps on a line already high (125 and 190 ns at 64 MHz, the
  poll's turn the resolution) - on an RC edge about 90 and 130 ns from
  30 to 70 per cent, the second over the 120 ns Fm+ allows a rise. The
  data is byte-exact at that rung all the same, on this short link.

### The peer bus (letters n..t)

The same two nodes carry a PEER BOARD running `twi_peer` - the desk's
STM32F446RE, its I2C1 on PB8/PB9 (`ident` fw 0x0400) - beside I2C2's
taps, so this desk carries the self-link AND the peer at once and the
suite runs both: letters b..l, x and y skip only when the boot probe
finds no self-link, and the peer letters - their own all-key, `w`, so
that `z` runs with the far board on its dark console - FAIL when nobody
answers. The two do not disturb each other: the peer's command client answers 0x6B
alone, which no self-link letter sends, and I2C2 is released while a
peer letter runs.

**The wire format is `avrxt/src/apps/twi_link.hpp`, shared by every
architecture's peer.** Here **one command is TWO TENURES of the engine
under test** - a write carrying the frame, then a read collecting the
answer - so the command channel is itself a measurement of this driver.
Ten of ten round trips, and `ident` names the peer's unique-ID word and
its `twi_peer` sanity byte.

**Every tenure shape against a real second chip.** A write, a read whose
bytes are the peer's own pattern byte-exact, and the combined
write-then-read - which the peer counts as **FOUR address matches for
two tenures**, the repeated START seen from the far end exactly as the
self-link's letter `b` sees it from I2C2 - with the peer accounting
every byte, 12 received and 12 served. A general call reaches the peer
at address 0x00.

**The vocabulary is real statuses across two architectures**: a client
parked where nobody calls answers `i2c_nack_addr` both to a write and to
the EMPTY request an address scanner sends, and a commanded refusal of
the third data byte answers `i2c_nack_data`. Commanded stretching is
priced against its own baseline - eight 8-byte tenures unstretched in 6
ms, ONE 8-byte tenure at 2 ms a byte in 16 ms, completing `i2c_ok`: the
controller simply waits, which is what flow control means.

**THE RATE ASKED, BETWEEN TWO CHIPS** (letter `q`): the write's SCL at
**950..951 and 859 thousandths** of 100 and 400 kHz over ten runs -
under the rate, and inside the quarter the verdict allows, the request's
own cost being inside the reading. All three speeds carry a write and a
read byte-exact, Fm+ included - at **722..723 thousandths of 1 MHz**,
which is the far end's doing and not this controller's: the peer's
target is a 400 kHz block that stretches every byte, so the verdict on
the Fm+ rung claims only that the kernel clock can produce it. **AND
ONE TARGET CONFIGURATION SERVES ALL THREE RUNGS** on a target of this
family: a target's `speed` names the row its SDADEL and SCLDEL are
solved against and not a rate it generates (32.4.8), so a G0 target
solved for Fm+ is byte-exact on the Sm and Fm rungs too.

**The arbiter measured against a SECOND CHIP.** Letter
`r` runs `I2cBus` (= `BusMaster`) over `I2cHost` with the peer answering:
four tenures queued come back in order with the NACK from an address
nobody answers delivered in its place, the sixth of six posted into a
four-deep queue is rejected on the spot, an idle bus votes for the sleep
and a busy one against it - `util/i2c_bus.hpp` and `util/bus_master.hpp`
exactly as they are written.

**AND THE WEDGE HAS A SECOND SIGNATURE WHEN A FOREIGN CHIP HOLDS THE
WIRE.** The self-link's letters `c` and `l`
measure a client of this same die holding SDA down: the silicon sees a
PARK and raises nothing (BUSY alone, no ARLO, no BERR), and the
arbiter's `i2c_timeout` at 20 ms is the whole answer. With the peer told
to hold SDA from its own GPIO for 300 ms, the same tenure comes back
**`i2c_arb_lost` in 0 ms, with SDA still low at the reply** - the ENGINE
reporting a wire code of its own where the other arrangement reported
nothing at all. What separates the two is a PE cycle before the START:
this letter's request runs at 400 kHz after commands at 100 kHz, and
the speed change cycles PE, which clears BUSY - measured under the
preemptive kernel below, where the same hold met at the speed in force
parks and times out and met after a speed change or a `recover()` loses
its address at once. The REQUIREMENT is met either way - the requester is answered IN ITS PLACE, never with
silence and never with `i2c_ok`, at the arbiter's limit or sooner, and
the same bus AO carries the next tenure to `i2c_ok` once the line comes
back.

**TWO HELD STARTS RELEASED BY ONE STOP SERIALIZE** (letter `s`). The
third node is this board's I2C2, whose pads tap the same bus: it opens a
tenure nobody answers at about 2 kHz (4.7 ms busy), this controller sets
START behind it - held, 32.4.9 - and the peer, told by `arb` to wait for
BUSY and a lead-in, sets its own, held the same way. I2C2's STOP
releases both. **Neither silicon commits its START early enough to meet
the other's**: a sampled trace of both lines (8 core cycles a sample)
puts this controller's START **5.75, 2.0 and 0.75 us** after the STOP at
100, 400 and 1000 kHz and the peer's within about 2 us, and in twelve
rendezvous over the three rates and two rendezvous rates the second
controller saw the first's START every time, waited, and went second -
no address was ever arbitrated, whichever order the addresses favoured.
What the letter judges is what holds: ten rendezvous of ten end on the
third node's own STOP, both controllers' tenures run whole to the
address nobody answers (`i2c_nack_addr` at both ends), no arbitration is
lost. A live arbitration is in the second list below.

**A RATE CHANGE ON A BUSY BUS SENDS THE START INTO THE OTHER'S TENURE.**
TIMINGR is PE-gated (32.9.5), so a request at a rate other than the one
in force costs `I2cHost` a PE cycle - and PE = 0 CLEARS BUSY (32.9.7), so
this controller forgets that another one holds the bus and the START
that follows goes out at once. Measured in letter `s`'s staging: a
request at 400 kHz or 1 MHz, made while the rendezvous tenure ran with
100 or 400 kHz in force, put this controller's START into I2C2's
tenure in eight stagings of ten in one sweep and in every single run
before it - I2C2 left mid-tenure with both lines low (BUSY, no STOPF),
the peer reporting a lost arbitration. A START requested at the rate
already in force waits behind the busy bus as 32.4.9 says, every time. The letter therefore
carries its commands at the rate it contends at; the driver's answer is
a decision posed (the first list below).

**THE WAKE FROM STOP, BY ANOTHER CHIP'S ADDRESS** (letter `t`). I2C2 -
the instance whose kernel is HSI16 and whose EXTI line 22 is direct - is
the client; the peer's `arb` writes four bytes to it 50 ms after this
board's own rendezvous tenure made the bus busy, and by then this board
is asleep. In Sleep first (the wake source proven in the shallower mode)
and then in **Stop 1**, the address wakes the core: the RTC on the LSE,
which is also the one-second backstop that never fired, reads the Stop
at **46..50 ms against 50 due** over ten runs (its unit 3.9 ms), and the
four bytes the address opened are taken whole, the peer's write
completing `i2c_ok` - the client stretched SCL through the wake. The
three conditions of 32.4.16 (HSI16 kernel, DNF 0, stretching on) are
the ones `wake_from_stop()` grants on, and HSIDIV is 0 at the fast rung
(ES0548 2.2.4).

**TWO THINGS A TARGET OF THIS FAMILY DOES THAT ITS TALLY MUST KNOW**,
both measured from the peer's side and both a property of the target
rather than of the verdict counting it. First, **a target transmitter is
always asked for one byte more than the controller takes**: TXIS rises as soon
as TXDR empties, so while the controller clocks byte k the target has
already loaded k+1, and the byte standing in TXDR when the closing NACK
arrives never reaches the wire - 32.4.8's own `flush()` is what throws
it away, and an instrument that counts it reports nine bytes served
for eight clocked. Second, **the last data byte's stretch falls outside
the controller's own tenure**: a controller's write is over once its
last byte is in the shifter, so a target that holds SCL before every
data byte lengthens the tenure by N-1 holds and not N. The address match
is a stretch too (ADDR holds the clock exactly as RXNE and TXIS do), so
an instrument that spends its commanded hold at ADDR as well puts the
full N back inside the measured window - which is how 2 ms a byte
prices an 8-byte tenure at the 16 ms the model predicts.

**A RULE ABOUT THE VECTOR AND NOT THE WIRE**, shared with the SPI
suite: a peer command re-inits the host, which hands
I2C1's vector back to the bare pump, so an arbiter that owned it must
re-state its claim afterwards or every later completion is consumed by
the wrong branch and answered `i2c_timeout` on a tenure that ran
perfectly.

### The stack under the preemptive kernel (`test_stm32_rubato_i2c`)

Measured by `test_stm32_rubato_i2c`: `util/i2c_bus.hpp`'s arbiter
(pending depth 4, the per-bus timeout at 20 ms) over the engined
`I2cHost<1>` on PB8/PB9 (DMA1 channels 1 and 2, PCLK under the static
PLL at 64 MHz) inside Rubato, against the same STM32F446RE `twi_peer`,
while TIM14 posts to the pack's highest object every 97, 193 or 389 us
and its dispatch spins 300 or 3000 cycles - at 97 us x 3000, 48 % of
the core. Every letter runs once with TIM14 silent and six times under
it; **195 verdicts, green on every run: forty of the letters `n`, `o`,
`p`, `w` and `i`, ten of `q` and `h`**. The pack, the
command channel through the arbiter and the letters are the I2C suites'
shared body (`avrxt/src/apps/rubato_i2c.hpp`, whose header describes
them), and [platform.md](platform.md) places the suite among the
preemptive kernel's.

**Where a preemption can reach this host: before the START, never inside
a tenure.** `start()` writes the tenure's enables and then the one CR2
store that carries START, so a dispatch above the arbiter that lands in
it delays the START: a gap between tenures. Inside a tenure every byte
decision is the vector's, and a late vector is a held SCL (32.4.8,
32.4.9). Letter `x` counts it on 400 writes of 16 bytes on the engines:
`start()` takes 400 cycles with TIM14 silent; under 97 us x 3000 the
interrupt landed inside it 166 to 181 times in eight runs, and of those 3
to 7 with ISR.BUSY already set - which can only be after the START
store, `start()`'s last register write, with the engines armed before
it, so the tenure on the wire was the vector's from its first bit:
nothing the thread does after the START store reaches the wire.

**THE TENURE CYCLE LOCKS TO THE PRESSURE'S PERIOD.** Were the ticks
independent of the tenures, a 400-cycle `start()` would hold one in
400 / 6208 of the cases - about 25 of 400. It holds one in 42 to 45 %: a
histogram of TIM14's count at `start()`'s entry (an instrument added for
the measurement, not kept in the suite) put 290 and 294 of 400 entries,
in two runs, in the last eighth of the period, just before a tick. The
tenure cycle - the write, its STOPF, the reply up the pack and the next
request down - settles at a phase of the pressure's period instead of
drifting across it; which step of that path does the locking is not
established. A program's own periodic interrupt and its bus traffic can
phase-lock the same way; what it costs here is one dispatch's length on
nearly every other tenure, and nothing on the wire.

**Every shape on both paths, under every pressure as in the control.**
Letter `o`, at 400 kHz against one serve counted to its last byte: the
probe, reads of 1 (the pump, the byte taken at STOPF) and of 2, 3, 8 and
32 (the receive engine), write-then-reads of 1+1 (both halves the pump),
1+2 and 2+8, writes of 1 (the preload alone) and of 2, 3, 8 and 32 (the
transmit engine) - every one `i2c_ok`, every read byte-exact, the
peer's count, sum and address matches exact (a write-then-read is two).
A shape's post-to-reply under pressure is the control's plus at most one
dispatch and the path behind it: the 32-byte write 827 us silent and at
most 883 pressed, the probe 58 and at most 114, over three runs. Letter `p`: a read of 16,
a 1+4 and a write of 16 byte-exact at 100 and 400 kHz (the write of 16
in 1601 and 442 us against the wire's 1530 and 382), and the peer's
commanded stretch of 1000 us a byte priced from below - a read of 8 in
9868 to 9975 us against its nine holds, a write of 8 in 8243 to 8357 us
against its eight (the address's and one before every byte the peer
takes or gives, the write's last falling after its tenure), under the
20 ms timeout.

**The vocabulary under pressure, and a refused byte ON THE ENGINE.**
Letter `q`: a deaf client - the peer's client parked at 0x5D, the
letter's own address - answers the probe and a four-byte write
`i2c_nack_addr` (the transmit channel's count never moved), and the
probe of 0x5D `i2c_ok`; the third byte of six refused answers
`i2c_nack_data` - a write of six rides the transmit engine, so this is
the NACK named mid-block from the channel's count, its untransmitted
tail abandoned and counted - the peer reporting the refusal and two
address matches, the tenure's and the probe's after it; the last written
byte of a 2+4 refused answers `i2c_nack_data` with the controller let
go, the peer counting one address match for that tenure, one for the
probe after it and no byte served - no repeated START followed. The next
tenure is `i2c_ok` after each.

**The arbiter with three requesters of three priorities.** Letter `w`:
the Client above the arbiter (20 write-then-reads of 1+6, the read half
on the receive engine, two outstanding), a requester below it (10 writes
of one byte, the preload), and in a pressure pass the preempting object
itself (8 probes posted from its tick): every reply `i2c_ok`, to its
requester, in its order - the Client's slot k complete and slot k+1
untouched at reply k, which Rubato makes exact because the requester
above the arbiter runs its reply inside the arbiter's own dispatch,
before the next waiting request starts - nothing rejected, no timeout,
the peer's tallies exact, in 6.1 to 7.9 ms a pass.

**THE TWO SIGNATURES OF THE HELD WIRE ARE ONE MECHANISM: THE PE CYCLE.**
Letter `h` has the peer hold SDA low for 60 ms. A tenure posted at the
speed in force parks behind the BUSY its own monitor saw and is answered
`i2c_timeout` by the arbiter at 20 ms (19938 to 20139 us from its
`start()`, 20 of 21 passes). In the other one a probe the preempting object
had posted first parked instead and timed out; the arbiter's
`recover()` - 32.4.6's PE cycle - then started the held tenure, and PE =
0 clears BUSY (32.9.7), so its START went out at once into the held line
and lost its address's first one bit: `i2c_arb_lost`, 59 us from
its `start()` and 20023 us from its post. The same tenure posted
at 400 kHz after the commands at 100 kHz costs `apply()` a PE cycle for
TIMINGR before the START, and is answered `i2c_arb_lost` in 36 to 43 us,
every run. The peer-bus letter `r` above meets its hold
the second way (its request at 400 kHz after commands at 100): that is
the second signature, and the self-link's park the first. Under the
flood - after its first probe, one at a tick only when the Console has
turned since the last, which on this core is most ticks: 180 to 722 a
pass, 560 to 565 of the 972 ticks at 97 us x 3000 - every request was
answered exactly once: while held `i2c_rejected` (the FIFO full),
`i2c_timeout` or `i2c_arb_lost`, after the release `i2c_ok`, or
`i2c_nack_addr` in the moment the peer's client was not yet back (6 of
18 pressure passes, one probe each); a request arrived in the arbiter's
draining window, between a timeout's `recover()` and the marker after
it, in 2 of 18 - the interleaving only a requester above the arbiter
can make. After the release the next tenure was `i2c_ok` and the arbiter
voted idle with nothing waiting, every pass.

**Dark** (letter `i`): every object idle, the arbiter's vote for the
sleep, no time event armed, TIM14 stopped, the host between tenures with
ERRIE alone and both DMA request enables clear, the transmit engine
holding nothing, BUSY clear and PB8 and PB9 reading high on their pads.
ES0548 2.10.2's spurious BERR counted 0 over thirty runs.

## On the STM32G071RB

`test_stm32_i2c` runs on the Nucleo-G071RB (DEV_ID 0x460, REV_ID 0x2000)
with the Nucleo-G0B1RE as its peer - the roles of the G0-to-G0 link
exchanged - and scores **54/54**, the same 54 the G0B1RE scores hosting
the other way. The self-link letters skip after the probe, as on the
G0B1RE.

**TABLE 165's COLUMN MOVES WITH THE PART, AND THE SILICON SAYS SO.**
There is no I2C3 (`i2c_present(3)` false, `I2c<3>` does not compile), and
- the finding that matters - **this part's I2C2 HAS NO INDEPENDENT
CLOCK**: the header declares no `RCC_CCIPR_I2C2SEL_Pos`, so
`i2c_has_independent_clock(2)`, `i2c_has_smbus(2)` and
`i2c_wakes_from_stop(2)` are all false, and `smbus_probe()` - 32.9.6's
own question, a TIMEOUTR write that does not read back - answers **I2C1
yes, I2C2 NO** where on the G0B1RE both answer yes. The wake line follows:
`i2c_exti_line(2)` is 0xFF here and 22 there. Letter `a`'s roster,
selector, wake-line and SMBus verdicts are all written as the reserve's
statement for the part rather than as constants.

**AND THE THREE REFUSALS MOVE WITH IT.** The verbs that must refuse on an
instance without the independent clock - `kernel_clock()`, `timeouts()`,
`wake_from_stop()` - are put to I2C3 on a part that has one and to I2C2
on a part that has not, which is the same claim about the same column
asked of whichever instance is in it.

Measured against the peer: the three speeds byte-exact both ways, the
tenure shapes
with the repeated START counted from the far end as two address matches,
the whole vocabulary on the wire, commanded stretching at 2 ms a byte
costing exactly 16 ms for eight, and the kernel letter running `I2cBus`
with an `i2c_timeout` recovered and the next tenure `i2c_ok`.

ES0418's I2C items: **2.11.1** (the tSU;DAT floors) is the G0B1's 2.10.1
and is the driver's refusals, identical on both; **2.11.2** (a spurious
BERR) is the G0B1's 2.10.2 and letter `m` counts none; **2.11.6** (a
transfer stalled when PCLK/I2CCLK falls between 1.5 and 3) is NOT
REACHED, this suite's ratios being 4, 1 and 0.125; and 2.11.3, 2.11.5 and
2.11.7 need the multi-master, NOSTRETCH-target and SMBus-target roles the
SELF-LINK carries, so they are not staged on this board.

## On the STM32G031K8

`test_stm32_i2c` runs ON THE WIRE on the Nucleo-G031K8 (DEV_ID 0x466,
REV_ID 0x1003) and scores **54/54** - the same 54 the SAM C21 and the
G071RB scored - **in both roles**: this board hosting against a Nucleo-64
running `twi_peer`, and this board running the peer while the G0B1RE
hosts. NOTHING IS COMPILED OUT HERE FOR A PACKAGE, unlike the SPI
suite's self-link: the LQFP32 bonds every pad of both links - I2C1's
PB8/PB9 and I2C2's PA11/PA12, which are the Nucleo-32's own A5 and A4 -
so this board asks the wire at boot like any other (`self-link probe:
SCL 0 SDA 0` today) and its eleven self-link letters skip on that
answer, claiming nothing. The image is 52676 bytes of the part's 64 K.

**TABLE 165's COLUMN AGAIN, one part further down.** This part's I2C2
has no `RCC_CCIPR_I2C2SEL`, so - as on the G071 - it has no independent
clock, no SMBus and no wake, and `smbus_probe()` says so on the SILICON:
32.9.6 forces TIMEOUTR to zero on an instance without SMBus, and every
instance the reserve's column grants it answers while every instance it
does not is forced to zero. There is no I2C3 here either, so the "must
refuse" verbs are asked of I2C2.

The timing arithmetic, ES0548 2.10.1's floors (4, 10 and 20 MHz for Sm,
Fm and Fm+) and the enable-protection rule - which on this chapter is
REAL, unlike the SPI's - all hold as they do on the LQFP64 parts. The
console is **LPUART1** on PA2/PA3 at AF6, for [clock.md](clock.md)'s
reason.

## Not covered yet

Driver gaps:

- **This chip as the bus's target, served from an active object, under
  the preemptive kernel.** A target whose bytes come from an object (its
  vector posting "the host reads" and leaving SCL held, a dispatch giving
  the byte) is the one place a held-off reader would stretch the clock;
  no util service serves a target that way (`I2cClient` is served from
  its vector), and the desk peer, the STM32F446RE's `twi_peer`, has no
  `host_read` to read from this board's target. Born with the first util
  target object.

- **No client DMA slots.** `I2cClient` has none, as `SpiClient` has
  none, and a slot waits for a device-shaped user. The TARGET
  side of a DMA tenure is therefore not measured at all: letter `h`
  runs the host's two engines against a client on its byte pump, which
  proves the host's half and says nothing about the client's.
- **The Request is 7-bit**, as on all three targets - 10-bit addressing
  lives at the resource level and `design/i2c-bus.md` records that the
  arbiter's descriptor has no shape for it.
- **A tenure longer than 255 bytes is not a Request either**: the
  lengths are `uint8_t` on all three targets, so RELOAD is a resource
  feature and the suite drives it directly.
- **No SMBus task.** The host and alert addresses, the PEC and the
  time-outs are resource verbs; a `SmbusHost` with the protocol's own
  vocabulary is born with its first device.
- **A rate change while another controller holds the bus.** `start()`
  at a rate other than the one in force cycles PE for TIMINGR, which
  clears BUSY, and the START then goes out into the other controller's
  tenure (measured, the peer bus above). Not settled: waiting for BUSY
  first, bounded, as the vendor's library does before a blocking
  transfer (25 ms),
  or answering the request as lost to another controller, are the two
  shapes, and the choice is posed. On a bus with one controller - every
  program on this desk but letter `s`'s staging - the hazard cannot
  arise.

Implemented but not bench-verified:

- **The held request of an I2C.** `test_stm32_dma` letter `r` measures a
  released requester on USART1, SPI1 and TIM3 with no wire; an I2C's
  TXIS or RXNE stands only inside a tenure on a wire, so its release is
  reset by reading - the self-link's engined letters pass with it, and a
  tenure abandoned mid-block, then released, then followed by the next
  owner of its request line, would measure it.
- **I2C3** - present on this part, exercised in the family fixture and
  in letter a's refusals, but its pads carry no wire here.
- **The LQFP32's own self-link.** The STM32G031K8 bonds both ends of
  it (I2C1 on PB8/PB9, I2C2 on PA11/PA12), so the eleven self-link
  letters are compiled there and probe for it at boot - but that desk
  carries the peer link on the same two pads, the probe answers no, and
  I2C2 AS A CLIENT HAS NEVER RUN ON THAT DIE. Two jumpers from CN3-11
  and CN3-13 to CN4-7 and CN4-8, with the peer wires off, would run
  them.
- **The target half of the PEC**, which table 176 makes "SBC = 1,
  RELOAD = 0, PECBYTE = 1" - the check rides target byte control, whose
  NBYTES the suite's pump does not re-arm.
- **The SMBus ALERT**, which needs a wire to an SMBA pad.
- **The last-resort sweep inside a tenure.** Letter `i`'s staged PECERR
  (on the self-link) is what takes the handler's last resort. Two of
  its promises are staged by no letter: ERRIE surviving it (an ARLO or a
  BERR reported by a tenure after it), and a flag that sets between the
  handler's read and the sweep - the TC of a write-then-read above all -
  served by the next entry instead of being disarmed or read away. The
  second needs the sweep to run in the middle of a tenure, which only a
  spurious BERR (ES0548 2.10.2, swept 0 times in a whole `z` run) or an
  SMBus flag makes it do, and a flag to land in the handler's few
  microseconds: a self-link letter raising an SMBus flag inside a
  write-then-read's write half, repeated until a TC lands there, would
  measure it.
- **The self-link roles on the STM32G071RB and the STM32G031K8**: the
  eleven self-link letters - the own-address match, the NOSTRETCH
  target, 10-bit addressing both ways, the PEC, the time-outs, the wake
  from Stop, the whole stretching census, and with them ES0418's 2.11.3,
  2.11.5 and 2.11.7 - skip on those boards because their PB8/PB9 carry
  the peer link and the probe answers no (above). All are measured on
  the STM32G0B1RE.
- **Arbitration lost against a real second controller.** ARLO is
  seen on silicon (letter `r`'s wedge, held by the peer from its own
  GPIO), but a LIVE RACE - two controllers driving addresses at once and
  the smaller byte winning - is not staged: letter `s` releases this
  controller's held START and the peer's on one STOP and measures them
  SERIALIZING (the second sees the first's START and waits, at every
  rate, 0.75 to 5.75 us after the STOP against the peer's 2 us), so a
  race wants one START committed inside the other's detection window, a
  fraction of a microsecond that nothing on this desk can aim. A peer
  whose START is released by software at a delay the DUT sweeps would
  measure it.

Declined, with the reason (what a wire or an instrument this bench has
not got would measure):

- **The filters' suppression itself.** A glitch needs a source on the
  net, and both ends of both wires are alternate functions with no third
  pad on either. The filters' effect on the TIMING is arithmetic and is
  measured; the spike is not staged.
- **ES0548 2.10.1's wrong sampling.** Reproducing it means a
  transmitter whose tSU;DAT is under one kernel period, which on this
  bench means a bit-banged sender on a pad the peripheral must also own.
  The erratum is carried as the refusal it is instead.
- **SCL at a 2 MHz core.** A tenure there is paced by the controller's
  and the target's vectors on the one core (the 0.64 above), so the
  stopwatch times the software and not the clock; a timer capture of
  SCL's edges on a pad of its own would time it, and both ends of the
  wire are alternate functions. The rate is carried by the register
  instead - the 16 MHz rung's own TIMINGR on the same 16 MHz kernel.
- **The Fm+ drive's electrical effect.** With the drive on and off the
  measured SCL period moves by less than the stopwatch's own resolution
  (921 vs 921 ns): 6.1.3 makes it a pad property and the period is
  TIMINGR's, so what the 20 mA buys is the EDGE, which no instrument on
  this board can see.
