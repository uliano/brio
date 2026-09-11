# I2C (STM32G0)

The chapter is built whole and measured three ways - on the Nucleo's
own self-link, and on the same two pads against two other chips, a SAM
C21 and another STM32G0; what is not covered is listed at the end, with
the reason in each case.

`brio/stm32g0/i2c.hpp` is RM0444 chapter 32 in the two strata every brio
bus driver has: `I2c<n>`, the resource; `I2cHost<n, pins, TxEngine,
RxEngine>`, the transfer engine `util/i2c_bus.hpp` drives; and
`I2cClient<n, pins>`, the target role. `I2cHost`'s Request is
`avrdx/twi.hpp`'s and `samc21/i2c.hpp`'s **verbatim**, so
`util/i2c_bus.hpp` and `util/bus_master.hpp` drive this engine exactly
as they are written.

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
the header DOES distinguish is `RCC_CCIPR_I2C2SEL_Pos`, which exists on
exactly the parts whose I2C2 has the independent clock - so the reserve
derives all three rows from that one probe, and the silicon is asked as
a second opinion (below).

**TIMINGR is the chapter.** Five fields set the SCL period, the data
hold time and the data setup time, and 32.4.5 gives each an inequality
against the I2C standard's own numbers and the bus's own edges. Three
things about it decide the code:

- **tSCL is not tSCLL + tSCLH.** 32.4.9 adds tSYNC1 + tSYNC2 - the edge
  slopes, the filters and two to three kernel periods of synchronization
  - and the example tables charge 250..1000 ns of it. That is a BUS fact
  and not a chip one, so it is an argument (`I2cBusTiming::sync_ns`),
  exactly as the rise time is on the other two targets.
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

## Types and verbs

`I2c<n>` carries the whole register description: the enable and 32.4.6's
disable, the configuration (TIMINGR, both filters, NOSTRETCH, SBC, the
general call, the wake, the SMBus enables and the PEC), the two own
addresses with OA2's seven mask codes, the controller's transfer engine
(`transfer()`, `transfer_now()`, `start()`, `stop()`, `reload()`), the
data registers, the flags with their W1C clears, the interrupts, the two
DMA enables, the SMBus time-outs and `smbus_probe()`, the wake with its
EXTI line, and the fast-mode-plus drive in both of SYSCFG's flavours.
Every field the register description gates is a verb that refuses, and
the gates are not one rule but four: PE = 0 for TIMINGR, NOSTRETCH,
ANFOFF, DNF and PECEN; START = 0 for CR2's address, direction and
NBYTES; OA1EN = 0 and OA2EN = 0 for their own registers; TIMOUTEN = 0
and TEXTEN = 0 for the two halves of TIMEOUTR.

The timing arithmetic goes both ways and is `constexpr` throughout:
`i2c_timing_for(kernel_hz, speed, filters, bus)` solves 32.4.5's
conditions for a register value, `i2c_scl_hz(kernel_hz, timing,
sync_ns)` prices one, and `i2c_scll_ns` / `i2c_sclh_ns` /
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
`spurious_bus_errors()`. The two DMA engine slots default to
`NoDmaEngine` and every DMA branch folds away.

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

Measured by `test_stm32_i2c`. The findings down to "ES0548 2.10.2 did
not fire once" are the SELF-LINK's (I2C1 host PB8/PB9, I2C2 client
PA11/PA12, both AF6, 2.2 kOhm pull-ups): 13 letters in `z`, 142
verdicts. The ones under "The peer bus" are a SECOND CHIP's, on the same
two pads and the same pull-ups: 54 verdicts, against a SAM C21 and
against a second STM32G0.

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

**A NACK branch must return.** The STOP the peripheral sends by itself
arrives as its own interrupt microseconds later; a branch that falls
through to a general sweep clears that STOPF the moment it sets, and the
tenure keeps its status, loses its flags and never completes. The
symptom is an order dependency between letters, which is what a race
usually looks like first.

**Three flags no branch owns.** PECERR, TIMEOUT and ALERT ride the one
ERRIE a plain I2C engine arms for BERR and ARLO. They are swept at the
bottom of the handler, and what a sweep cannot reach is disarmed.

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

**The standard's worst-case edges make a real bus run FAST.** The
chooser charges tSYNC = 1000 / 750 / 500 ns; this wire's own tSYNC,
measured as the difference between the tenure's average period and the
register's own tSCLL + tSCLH, is **438 ns**. So at a nominal 100 kHz the
bus really runs at **105263 Hz - above the mode's own limit** - and
handing the measured budget back to `init()` brings it to **99690 Hz**.
The same hazard `avrdx/twi.hpp` and `samc21/i2c.hpp` record about their
rise-time arguments, seen from the other side; state what the bench
measures.

The rates, at 64 MHz on PCLK, measured over a 32-byte tenure as
duration / (9 x 33) with the target in NOSTRETCH so it holds the clock
for nothing:

| asked | register floor (tSCLL + tSCLH) | measured | standard-edge prediction |
|-------|-------------------------------|----------|--------------------------|
| 100 kHz | 9062 ns | 9500 ns | 10062 ns |
| 400 kHz | 1750 ns | 2156 ns | 2500 ns |
| 1 MHz (stretching) | 499 ns | 921 ns | 1000 ns |

**At 1 MHz this core cannot serve a NOSTRETCH target.** The window is
nine microseconds - the byte before the one being missed - and an
interrupt-driven target does not always make it: 32.4.17's automatic
NACK on an overrun follows, and the controller reads `i2c_nack_data`.
With the target stretching, the same 1 MHz bus is byte-exact.

**Clock stretching is linear and free of the data.** Commanded holds of
20 / 50 / 100 us per event lengthened an 8-byte read to 307 / 588 /
1057 us against 383 / 653 / 1103 predicted (nine holds: the address
event and eight bytes), byte-exact throughout. With NOSTRETCH set, a
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

### The peer bus (letters n..r)

The same two pads and the same two pull-ups reach a PEER BOARD running
`twi_peer` when the jumpers go there instead of to I2C2, and the
topologies EXCLUDE each other: the suite probes for the self-link at
boot and letters b..l, x and y skip themselves when it is absent (letter
m keeps its three wireless verdicts and drops its closing wire leg). On
the peer desk `z` is 54 verdicts (letter `a` wireless, letter `m`
wireless, letters `n`..`r` on the wire), and they hold against a SAM C21
and against a second STM32G0 alike. THE TWO PEERS ARE INTERCHANGEABLE
HERE: every verdict of the five letters holds against both, and the peer's `ident` is what says which one answered.

**The wire format is `avrdx/src/apps/twi_link.hpp`, shared by all three
architectures' peers.** Here **one command is TWO TENURES of the engine
under test** - a write carrying the frame, then a read collecting the
answer - so the
command channel is itself a measurement of this driver. Ten of ten round
trips at a measured 99 kHz, and `ident` names the peer's die serial and
its `twi_peer` sanity byte.

**Every tenure shape against a real second chip.** A write, a read whose
bytes are the peer's own pattern byte-exact, and the combined
write-then-read - which the peer counts as **FOUR address matches for
two tenures**, the repeated START seen from the far end exactly as the
self-link's letter `b` sees it from I2C2. A general call reaches the peer
at address 0x00.

**The vocabulary is real statuses across two architectures**: a client
parked where nobody calls answers `i2c_nack_addr` both to a write and to
the EMPTY request an address scanner sends, and a commanded refusal of
the third data byte answers `i2c_nack_data`. Commanded stretching is
priced against its own baseline - eight 8-byte tenures unstretched in 6
ms, ONE 8-byte tenure at 2 ms a byte in 16 ms, completing `i2c_ok`: the
controller simply waits, which is what flow control means.

**ALL THREE SPEEDS CARRY THE LINK BYTE-EXACT, Fm+ INCLUDED** - 99 kHz,
400 kHz and 1000 kHz against the register, a write and a read exact at
each, against BOTH peers. That is worth stating as a WIRE result and not
a controller one: a megahertz bus here is a property of these two short
jumpers, their 2.2 kOhm pull-ups and the far end's own input path (the
SAM's target has no input filter at all), and the suite's verdict on the
Fm+ rung claims only that the
kernel clock can produce it. **AND ONE TARGET CONFIGURATION SERVES ALL
THREE RUNGS**: a target's `speed` names the row its SDADEL and SCLDEL
are solved against and not a rate it generates (32.4.8), so the G0 peer
sits on the bus solved for Fm+ - the fastest rung the ladder will bring
- and the Sm and Fm rungs are byte-exact through the same delays.

**The arbiter measured against a SECOND CHIP.** Letter
`r` runs `I2cBus` (= `BusMaster`) over `I2cHost` with the SAM answering:
four tenures queued come back in order with the NACK from an address
nobody answers delivered in its place, the sixth of six posted into a
four-deep queue is rejected on the spot, an idle bus votes for the sleep
and a busy one against it - `util/i2c_bus.hpp` and `util/bus_master.hpp`
exactly as they are written.

**AND THE WEDGE HAS A SECOND SIGNATURE WHEN A FOREIGN CHIP HOLDS THE
WIRE - RECORDED, NOT RECONCILED.** The self-link's letters `c` and `l`
measure a client of this same die holding SDA down: the silicon sees a
PARK and raises nothing (BUSY alone, no ARLO, no BERR), and the
arbiter's `i2c_timeout` at 20 ms is the whole answer. With the SAM told
to hold SDA from its own PORT for 300 ms, the same tenure comes back
**`i2c_arb_lost` in 0 ms, with SDA still low at the reply** - the ENGINE
reporting a wire code of its own where the other arrangement reported
nothing at all. The two differ in who holds the line and in how the
hold's start lines up with the controller's own START, and this bench
does not separate those; what it does establish is that the REQUIREMENT
is met either way - the requester is answered IN ITS PLACE, never with
silence and never with `i2c_ok`, at the arbiter's limit or sooner, and
the same bus AO carries the next tenure to `i2c_ok` once the line comes
back.

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

Measured against the peer: the three speeds byte-exact both ways with SCL
at **99 / 400 / 1000 kHz** (the 100 k rung at 99 kHz, the measured
tSYNC budget rather than the standard's worst case), the tenure shapes
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

Implemented but not bench-verified:

- **I2C3** - present on this part, exercised in the family fixture and
  in letter a's refusals, but its pads carry no wire here.
- **The LQFP32's own self-link.** The STM32G031K8 bonds both ends of
  it (I2C1 on PB8/PB9, I2C2 on PA11/PA12), so the eleven self-link
  letters are compiled there and probe for it at boot - but that desk
  carries the peer link on the same two pads, the probe answers no, and
  I2C2 AS A CLIENT HAS NEVER RUN ON THAT DIE. Two jumpers from CN3-11
  and CN3-13 to CN4-7 and CN4-8, with the peer wires off, would run
  them.
- **The wake from Stop on silicon.** Everything around it is measured -
  the three conditions as refusals, WUPEN's readback, the direct EXTI
  line, a bus that carries bytes with it armed - but the wake itself
  needs an address on the wire while the core is stopped, which on the
  SELF-LINK is impossible: both ends are on the same die, and a Stop
  that silences the client silences the controller that would wake it.
  The peer bus removes that obstacle - the peer is a second node and
  `twi_link`'s `arb` op makes it a CONTROLLER, on all three of that
  app's ports - so the wake is stageable and simply not built: it
  wants this board's client on PB8/PB9 while the peer addresses it,
  which is a role swap no letter of this suite does yet. ES0548 2.2.4
  (HSIDIV must be 0) is stated for the same reason.
- **The target half of the PEC**, which table 176 makes "SBC = 1,
  RELOAD = 0, PECBYTE = 1" - the check rides target byte control, whose
  NBYTES the suite's pump does not re-arm.
- **The SMBus ALERT**, which needs a wire to an SMBA pad.
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
  the smaller byte winning - is not staged, and it is the
  cheapest gap on this list: `twi_link`'s `arb` op makes the peer a
  controller (implemented on the stm32g0 port too, exercised by no
  letter), `twilink::low_addr` exists so either end can be made to lose,
  and the rendezvous it needs is a bus both ends see Busy at the same
  instant. The self-link cannot help, since a held START on this silicon
  does not fire when a phantom release comes - the SAM C21's answer
  too.

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
- **The Fm+ drive's electrical effect.** With the drive on and off the
  measured SCL period moves by less than the stopwatch's own resolution
  (921 vs 921 ns): 6.1.3 makes it a pad property and the period is
  TIMINGR's, so what the 20 mA buys is the EDGE, which no instrument on
  this board can see.
