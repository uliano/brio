# FMPI2C (STM32F4)

This document is for a programmer who wants an I2C bus from the second
I2C design of this family: the FMPI2C1 block of the STM32F410, F412,
F413/F423 and F446, beside the classic I2C of [i2c.md](i2c.md). It is
the one to reach for when the bus needs Fast-mode Plus (1 MHz), or a
rate that survives a change of the core's clock, because it has a
kernel clock of its own. `brio/stm32f4/fmpi2c.hpp` has it in the layers
every brio bus driver has: `FmpI2c<n>`, the resource, one function per
register fact; `FmpI2cHost<n, pins, TxEngine, RxEngine>`, the host task,
the transfer engine `util/i2c_bus.hpp` drives (a task, in brio, is a
driver layer with state of its own, served from its peripheral's
interrupts; an engine is the task an arbiter hands one bus transaction
at a time); and `FmpI2cClient<n, pins>`, the target role. The arbiter,
`I2cBus`, is an active object (a component of a brio program with its
own event queue, served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)): the active
objects that share the bus post it requests, it hands them to the host
one tenure at a time - the time the controller holds the bus, from START
to STOP - and replies to each with an event
([design/i2c-bus.md](../design/i2c-bus.md)).

The smallest use, a bus host under the arbiter - the first block of "How
to use it" below, on the Nucleo-F446RE's PC6/PC7:

```cpp
constexpr FmpI2cPins pins{.scl = {'C', 6, PinFunction::af4},
                          .sda = {'C', 7, PinFunction::af4}};
using Bus = FmpI2cHost<1, pins>;
using BusAo = I2cBus<Bus, P, 4>;

Bus::init(clock);                       // the APB clock as kernel, Sm applied

extern "C" void FMPI2C1_EV_IRQHandler() {
    if (Bus::isr()) { post<BusAo>(TransferDone{Bus::status()}); }
}
extern "C" void FMPI2C1_ER_IRQHandler() {
    if (Bus::error_isr()) { post<BusAo>(TransferDone{Bus::status()}); }
}
```

Line by line (the names are in `brio::`, the block written as inside a
`using namespace brio`): `pins` names the two pads with the
alternate-function number the datasheet gives FMPI2C1's SCL and SDA on
them, which nothing in the code can check; `Bus` is the host task over
the one instance, its two DMA engine parameters left empty; `BusAo` is the
arbiter over it for the platform `P`, with four requests of queue, an
active object listed in the program's `Tenuto` like any other; `init()`
takes the clock task's tag (`constexpr SysClock clock;`,
[clock.md](clock.md)), selects the kernel clock (the instance's APB1
clock by default), solves the three speed rows against it and hands the
pads over; the two `extern "C"` functions are the instance's event and
error interrupt handlers, as in C, under ST's names for the vectors, and
each posts the completion to the arbiter when its body says the tenure
is over. A client then posts a `Bus::Request` to `BusAo` - one tenure, a
write, a read, a write then a read with a repeated START, or the empty
probe - and receives an `I2cDone` event with the status.

What decides correctness even there. The part must carry the block: on
a part without it `FmpI2c<n>` does not compile, and `n` must be 1, the
one instance there is. `init()` after the clock task's: on the APB or
SYSCLK kernel clock, TIMINGR is solved from the rate the clock type
claims and not read from the hardware, so a bus initialized on a chip
still at its 16 MHz reset rate under a 180 MHz clock type (45 MHz of
APB1) runs at about a third of the rate asked - derived from the
register arithmetic, not measured - and nothing refuses it (on the HSI
kernel clock the rate does not depend on the clock type). `init()`
answers false when not even standard mode can be solved for the kernel
clock, or when the kernel clock or the boot configuration is refused.
Both vectors bound by the program: `init()` enables the two NVIC lines,
and an unbound one is the crt's weak alias of `Default_Handler`, a
silent spin, where the first event of the first tenure stops the
program. The buffers a `Request` names are lent (`brio::lend`) from the
post to the `I2cDone` the arbiter replies with: untouched and in scope
until then, which the `Borrowed` type records and nothing checks. A
speed the kernel clock cannot produce is answered `i2c_rejected` before
a byte moves (`speed_ok()` says so beforehand), and so is an address
outside 0x08..0x77 - the general call and the 10-bit prefixes among them,
which this block would send as plain 7-bit addresses where the classic
I2C beside it sends a header ([../design/i2c-bus.md](../design/i2c-bus.md));
a Fast-mode Plus bus
wants the 20 mA pad drive turned on by the program
(`Bus::fast_plus_drive(true)`), which nothing turns on by itself. A
refused address and a refused DATA byte are told apart on both paths:
`i2c_nack_addr` when nothing of the half's data reached the wire - the position of the interrupt that moves one byte at a time (the pump, on this
page), or under the DMA engines TXE and the transmit stream's count - and
`i2c_nack_data` otherwise ("Bench findings").

The page is long because the chapter is. In order: the documents of
record; what the silicon does (the block's identity and what differs
from the STM32G0's, the one instance, the kernel clock, the timing
register and its two-way arithmetic, the erratum's kernel-clock floor,
the byte counter, the errata the driver answers, the PE procedure and
the write gates, the SMBus half, the pads, the DMA cells); the types and
verbs; how to use it, one block per shape; the bench findings on the
Nucleo-F446RE, with the host's cost and the timing chooser against the
wire; and what is not covered.

## Documents of record

RM0390 Rev 6 ch. 23 (the F446's manual; the F410,
F412 and F413/F423 carry the same block and their manuals are not on
this desk), DS10693 Rev 11 table 11 for the pads and its I2C
characteristics for the bus edges, RM0390 table 28 for the DMA request
mapping, and ES0298's FMPI2C items - 2.12, ten of them: "10-bit
controller mode: new transfer cannot be launched if first part of the
address is not acknowledged by the target", "wrong data sampling when
data setup time is shorter than one FMPI2C kernel clock period",
"spurious bus error detection in controller mode", "last-received byte
loss in reload mode", "spurious controller transfer upon own target
address match", "START bit is cleared upon setting ADDRCF, not upon
address match", "OVR flag not set in underrun condition", "transmission
stalled after first byte transfer", "SDA held low upon SMBus timeout
expiry in target mode" and "inconsistent FMPI2C peripheral instance
naming". Driver: `stm32f4/fmpi2c.hpp` (`FmpI2c<n>` the resource,
`FmpI2cHost<n, pins, TxEngine, RxEngine>` the bus engine,
`FmpI2cClient<n, pins>` the other end, the vocabulary `FmpI2cSpeed`,
`FmpI2cClock`, `FmpI2cTiming`, `FmpI2cFilters`, `FmpI2cBusTiming`,
`FmpI2cConfig`, `FmpI2cAddressConfig`, `FmpI2cAddressMode`,
`FmpI2cOa2Mask`, `FmpI2cFlag`, `FmpI2cClear`, `FmpI2cInterrupt`,
`FmpI2cPins`, `FmpI2cHostConfig`, `FmpI2cClientEvent`, and the timing
arithmetic both ways), over the reserve's presence, SMBus and
request-mapping facts (`stm32f4/device_tables.hpp`, the reserve: the
table of what each part has, read off the device header), under the arbiter
and the status vocabulary of [../design/i2c-bus.md](../design/i2c-bus.md),
with the streams from [dma.md](dma.md), the kernel-clock multiplexer
from [clock.md](clock.md) and the Fast-mode Plus pad drive from the
SYSCFG block ([exti.md](exti.md)). The family fixture is
`test/family_stm32f4/fmpi2c.cpp` with the negatives that refuse a second
instance, the block on a part that has not got it, a bus with no SDA,
one pad twice, a pad on an absent port, an engine off the request map,
an engine on a part class whose manual was not read, one engine of two,
and an engine whose element is not a byte. Bench: `test_stm32f4_fmpi2c`
on the Nucleo-F446RE, the one board of this stratum whose part carries
the block, its PC6 and PC7 wired to the same chip's I2C1 on PB8 and PB9
with a 2.2 kOhm pull-up on each line - the classic block silent in every
letter but one, and a target in that one; the other end of the classic
block's own suite on the same wires ([i2c.md](i2c.md)), and of
`bench_stm32f4`'s letter `i`. A letter of a suite is one command key of
the suite's console and the group of verdicts it runs; `z` is the key
that runs them all, and `t` a second set, run against the desk partner
(below).

## What the silicon does

**It is the STM32G0's I2C under another name.** CR1/CR2 with SADD,
NBYTES, RELOAD and AUTOEND; one TIMINGR word of five fields; OAR1 and
OAR2 with the second address's mask; ISR and ICR; PECR; TIMEOUTR;
separate RXDR and TXDR. Register for register and bit for bit it is the
block [../stm32g0/i2c.md](../stm32g0/i2c.md) describes, and the driver is
that driver with the same verb names and the same shapes. It is not the
block of [i2c.md](i2c.md), which is the F1 lineage's event machine and
another chapter entirely; a part that has both has two I2C designs on one
die, which is why every name in this file carries an `Fmp` prefix.

**What differs from the STM32G0, all of it this family's own:** two
vectors instead of one (table 142 splits the seven event conditions from
the six error ones, as every other peripheral of this stratum does), the
kernel-clock multiplexer in RCC_DCKCFGR2 instead of RCC_CCIPR, the Fm+
pad drive in SYSCFG_CFGR as two bits per signal instead of a bit per pad,
DMA streams instead of channels - and no wake from Stop.

**Table 127's row, and its one dash.** 7- and 10-bit addressing, all
three speeds up to Fast-mode Plus at 1 MHz, the independent clock and
SMBus/PMBus are all there; "wakeup from Stop mode" is not. The device
header agrees in the only way it can: no header of the pack declares an
FMPI2C_CR1_WUPEN, and 23.7.1 makes CR1's bit 18 reserved. So the driver
publishes `wakes_from_stop` as false and its `wake_from_stop()` refuses -
the one feature of the STM32G0's identical register file that this
silicon has not.

**One instance, and two names for it.** Every part that carries the
block carries exactly one. ES0298 2.12.10 records the name as a
documentation erratum: "I2C4 and FMPI2C1 refer to the same peripheral
instance", which is why RM0390 6.3.27 calls the kernel-clock selector
"I2C4 kernel clock source selection" while the rest of the chapter says
FMPI2C1. The header's name is the one used here. The resource is still
`FmpI2c<n>` with an index, so the day a part arrives with two, no user of
the file is renamed.

**The independent clock is the reason to reach for this block.**
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
states no tAF(min), so zero), and only the slopes are a BUS fact. So **a
requested rate is a ceiling**: unless told the wire's measured tSYNC
(`FmpI2cBusTiming::sync_ns`), the chooser charges that floor alone, the
fastest any wire can make, and a real wire only adds its slopes and runs
slower; told it, the bus runs at the rate asked. What the other choices
cost is measured below.

**The kernel clock has a floor and it is an erratum's.** ES0298 2.12.2:
below a kernel period that fits inside the transmitter's minimum data
setup time the block samples the previous SDA value, and the erratum
names 4 / 10 / 20 MHz for Sm / Fm / Fm+ against the standard's own
tSU;DAT. There is no register workaround - "increase the FMPI2C kernel
clock frequency" is the whole of it - so the driver refuses a speed the
kernel clock cannot legally make, and answers a Request naming it with
`i2c_rejected` rather than running the bus at a rate nobody asked for.
23.4.3's own two conditions (tI2CCLK < (tLOW - tfilters)/4 and
tI2CCLK < tHIGH) are checked beside it and refuse independently: they are
what drops Fast-mode Plus when the digital filter is deep.

**The transfer engine is a byte counter, not a set of software
sequences.** CR2 carries the address, the direction, NBYTES and the end
policy in one word; the START bit sends everything up to the address, and
from there TXIS and RXNE pace one byte each. AUTOEND sends the STOP when
NBYTES runs out; with AUTOEND clear the transfer ends on TC with SCL
stretched, and setting START there IS the repeated START. RELOAD carries
a transfer past 255 bytes through TCR - and ES0298 2.12.4 says a receiver
can lose the byte that raises it, so the host engine never sets RELOAD
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
  3, the second byte "is not internally detected" and the block stalls
  until a PE cycle or a reset. The erratum's two workarounds are to write
  the first byte into TXDR before the transfer starts, or to choose a
  kernel clock outside the band. The host takes the first under `dma`, on
  one path whatever the two clocks: `start()` flushes TXDR, stores the first
  byte and starts the transmit stream on the rest, all before the START, and
  a write of one byte starts no stream. `interrupt` needs neither, its first
  store coming later than the window. Both are measured in the band and out
  of it ("ES0298 2.12.8" below). The ratio stays published
  (`transmit_stall_risk()`) for what does not go through this host: a
  target's first byte, the erratum naming target mode too, and a program's
  own stream on the resource's verbs.
- 2.12.4, the reload-mode byte loss: answered by never using RELOAD.
- 2.12.1, a 10-bit address whose first byte is NACKed, which leaves NACKF
  and START both standing and no new transfer possible. The host's Request
  is 7-bit and cannot reach it; the resource publishes the state
  (`ten_bit_nack_stuck()`) and the escape is 23.4.6's PE cycle, which is
  `cycle()`.
The other six are the application's: 2.12.5 and 2.12.6 (a device that is
controller and target at once must write CR2 with START low after every
address match, and ADDRCF is the only thing that clears START), 2.12.7
(a target transmitter with NOSTRETCH can send 0xFF without raising OVR),
2.12.9 (an SMBus time-out in target mode reports but does not release
SDA; the recovery is a PE cycle), 2.12.2's floor (a refusal here) and
2.12.10 (the name).

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

**The SMBus half is whole here**, unlike on some instances of the same IP
elsewhere: the two device addresses, the alert, the PEC engine and the
three time-outs. TIMEOUTA watches SCL held low by anybody (TIDLE = 0) or
both lines standing high (TIDLE = 1, bus idle detection); TIMEOUTB
watches this peripheral's own cumulative stretch. 23.7.6 makes the whole
register read back zero where the half is absent, which is a question the
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
transfer (23.4.16), with no length below which either does not hold. A
write's first byte stored before the START spends one of NBYTES's TXIS
events (23.4.9), and the stream answers the rest.

## Types and verbs

`FmpI2c<n>` - the resource. The gate and the reset (`bus_clock`,
`reset`), the kernel-clock multiplexer (`kernel_clock`, `kernel_hz`), the
enable and 23.4.6's procedure (`enable`, `disable`, `cycle`, `enabled`),
the configuration behind PE (`configure`, `timing`, `timing_reg`,
`filters`, `no_stretch`, `byte_control`, `general_call`), the own
addresses (`addresses`, `oar1`, `oar2`, `own_address`, `own_address10`),
the transfer engine (`transfer`, `transfer_now`, `cr2_word`, `start`,
`starting`, `stop`, `reload`, `nbytes`, `cr2`, `nack_next`, `pec_byte`,
`pec`, `ten_bit_nack_stuck`), the data registers (`data`, `flush_tx`,
`tx_address`, `rx_address`), the status and its clears (`flags`, `flag`,
`clear`, `clear_all`, `busy`, `host_reads`, `matched_address`), the
interrupts and the two vectors' masked views (`interrupt`, `interrupts`,
`pending`, `pending_event`, `pending_error`, `isr`, `error_isr`), the DMA
enables (`dma_transmit`, `dma_receive`), the SMBus time-outs (`timeouts`,
`smbus_probe`), the Fast-mode Plus drive (`fast_plus_drive`,
`fast_plus_scl`, `fast_plus_sda`), the wake that is not there
(`wake_from_stop`), and `release` (the block disabled and its clock
gated, no reset pulse, the host's and the client's `init` starting from
the RCC reset - the release contract, docs/design/dma.md). The constants `number`, `event_irq`,
`error_irq`, `smbus_claimed`, `wakes_from_stop`, `has_ten_bit`,
`has_fast_plus`.

`FmpI2cHost<n, pins, TxEngine, RxEngine>` - the bus engine, with the
parameter list and the `Request` of [i2c.md](i2c.md)'s `I2cHost`, field
for field, the speed an `FmpI2cSpeed` (with `fast_plus_1m`). `engine`
names what moves the data, and the host never chooses: `interrupt`, the
default, the event vector at any length; `dma` the engines at any
length, started in `start()` before the START (23.4.16) with a write half's
first byte already stored in TXDR, so that a write of one byte starts no
stream, on a host with both engine parameters named - `has_dma` says so at
compile time, and `dma` on a host without them is answered
`i2c_rejected` inside `start()` with nothing on the wire. A half with no
buffer stays on the vector under `dma` too, writing its filler or
draining what it reads, and the request completes. A stream is this
host's alone (the ownership rule, [../design/dma.md](../design/dma.md)),
so `start()` does not ask it whether it started: a program that starts
the host's own engine by hand stalls its next tenure, for the per-bus
timeout to answer. The field sits in the padding after
`speed`, the Request 28 bytes with it and without it. The verbs: `init`, `rebase`, `speed_ok`, `timing_of`, `scl_hz`,
`kernel_hz`, `reference_hz`, `transmit_stall_risk`,
`spurious_bus_errors`, `fast_plus_drive`, `claim_smba_pad`, `idle`,
`start`, `status`, `isr`, `error_isr`, `dma_isr`, `unstick`, `recover`,
`release`. The engines arm at `high`, both: a byte RXDR still holds
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

**A bus host under the arbiter.** The same shape as every other bus in
brio, and the same `Request`:

```cpp
constexpr FmpI2cPins pins{.scl = {'C', 6, PinFunction::af4},
                          .sda = {'C', 7, PinFunction::af4}};
using Bus = FmpI2cHost<1, pins>;
using BusAo = I2cBus<Bus, P, 4>;

Bus::init(clock);                       // the APB clock as kernel, Sm applied

extern "C" void FMPI2C1_EV_IRQHandler() {
    if (Bus::isr()) { post<BusAo>(TransferDone{Bus::status()}); }
}
extern "C" void FMPI2C1_ER_IRQHandler() {
    if (Bus::error_isr()) { post<BusAo>(TransferDone{Bus::status()}); }
}
```

Moving a program from the other I2C of this family to this one is one
type name: `I2cHost<1, i2c_pins>` becomes `FmpI2cHost<1, fmp_pins>`, and
the `Request` its clients build is unchanged but for a speed it names,
which is an `FmpI2cSpeed` here.

**Choosing the kernel clock**, which is what this block has and its
neighbour has not:

```cpp
Bus::init(clock, FmpI2cHostConfig{.kernel = FmpI2cClock::hsi});
// 16 MHz whatever the core does - and Fm+ is refused there, because
// ES0298 2.12.2 wants 20 MHz for it. speed_ok() says so before a
// Request does. On a core whose APB1 sits at 24 to 48 MHz this is inside
// ES0298 2.12.8's band: transmit_stall_risk() answers true, and neither
// engine stalls there (Bench findings).
```

**A bus whose edges are known.** The defaults are the specification's
worst-case edges for the delays and no measured tSYNC, so each speed asked
is a ceiling the wire stays at or under (about 2, 8 and 14 per cent under
on this bench's wire at 45 MHz). A board that has measured its wire - the
period less the register's tSCLL + tSCLH, which `test_stm32f4_fmpi2c`'s
letter `f` prints - states it, and gets the rate asked for:

```cpp
Bus::init(clock, FmpI2cHostConfig{
    .bus = FmpI2cBusTiming{.rise_ns = 250, .fall_ns = 60,
                           .setup_ns = 100, .sync_ns = 240}});
```

**The DMA engines**, on the only two cells the request mapping gives,
carry the data of a Request that names `dma`; one that names nothing
still runs on the event vector:

```cpp
using Bus = FmpI2cHost<1, pins, DmaTxEngine<1, 5, 2>, DmaRxEngine<1, 2, 2>>;
static_assert(Bus::has_dma);         // or `dma` is answered i2c_rejected
Dma<1>::init();                      // the controller is the app's
Bus::init(clock);
extern "C" void DMA1_Stream5_IRQHandler() { if (Bus::dma_isr()) { /* ... */ } }
extern "C" void DMA1_Stream2_IRQHandler() { if (Bus::dma_isr()) { /* ... */ } }
// and every request that should ride them: .engine = I2cEngine::dma
```

**The SMBus time-outs**, sized from the kernel clock in force:

```cpp
const uint32_t ker = Bus::kernel_hz();
const auto idle = fmpi2c_timeout_code_for(ker, 50u, true);      // tIDLE
const auto ext  = fmpi2c_timeout_code_for(ker, 8'000u, false);  // tLOW:EXT
FmpI2c<1>::timeouts(*idle, true, true, *ext, true);
```

**A target**, whose protocol is the application's:

```cpp
using Peer = FmpI2cClient<1, pins>;
Peer::init(clock, FmpI2cAddressConfig{.own = 0x2A}, FmpI2cSpeed::fast_400k);
extern "C" void FMPI2C1_EV_IRQHandler() { act(Peer::service()); }
extern "C" void FMPI2C1_ER_IRQHandler() { act(Peer::error_service()); }
```

## Bench findings

The board is a Nucleo-F446RE at 180 MHz with 45 MHz on APB1, and the two
pads are PC6 and PC7, wired to I2C1's PB8 and PB9 with a 2.2 kOhm
pull-up on each line. Letters a to l see that bus with I2C1 silent -
what a bus with nobody answering can be made to say - and letter m makes
I2C1 the target. `test_stm32f4_fmpi2c` is 104 verdicts over thirteen
letters, and `z` was green in three runs of three, as was letter `f` in twenty of
twenty.

**The register file at reset is the chapter's**, and the one non-zero
value is ISR = 0x0001: TXE stands out of reset, so a transmitter that
waits for TXE before its first byte waits for nothing.

**BUSY DOES NOT COME UP SET, and that is the opposite of the other I2C of
this family.** There, BUSY watches the peripheral's own line inputs and
reads them low while the pads are not yet in their alternate function, so
a block configured before its pads comes up busy over an idle wire and
its first START never leaves ([i2c.md](i2c.md)'s trap). Here BUSY is set
by a START CONDITION and cleared by a STOP or by PE = 0 - and 23.4.6's PE
cycle is part of every `init()` - so it read clear both before and after
the pads were handed over.

**The wake from Stop is really absent.** A one written into CR1's bit 18,
where the STM32G0's identical register file keeps WUPEN, reads back zero.

**The SMBus half answered for itself.** A value written into TIMEOUTA
read back, which 23.7.6 says only an instance that has the half can do.

**The kernel-clock multiplexer moves the whole arithmetic.** The three
sources measured 45 MHz (APB1), 180 MHz (SYSCLK) and 16 MHz (HSI), each
read back from RCC_DCKCFGR2. Fast-mode Plus is reachable on the first two
and REFUSED on the HSI, ES0298 2.12.2 wanting 20 MHz for it - which is
the price of a 1 MHz bus stated as a compile-and-run-time fact rather
than a footnote. ES0298 2.12.8's band lands the other way round: the HSI
against a 45 MHz APB1 is a ratio of 2.8 and sits INSIDE the 1.5..3 window,
while the APB clock (1.0) and SYSCLK (0.25) do not; neither engine of the
host stalls in it (below).

**What the chooser produces at 45 MHz**, and the three speeds it makes:

| speed | PRESC | SCLL | SCLH | SDADEL | SCLDEL | states |
|-------|-------|------|------|--------|--------|--------|
| Sm 100 kHz | 3 | 61 | 49 | 3 | 14 | 99557 Hz |
| Fm 400 kHz | 1 | 38 | 15 | 6 | 9 | 394736 Hz |
| Fm+ 1 MHz | 0 | 25 | 14 | 3 | 8 | 1000000 Hz |

The rate stated is the fastest any wire can make of the word - the
period's halves plus the floor under tSYNC, four kernel periods here - and
neither of the two that the kernel rate does not divide comes out above
the speed asked for: the period is rounded UP before it is split, which
is what keeps a 400 kHz request at 394736 Hz and not at 401785.

**THE SPEED ASKED IS A CEILING ON THE WIRE, AND THE ORACLE'S IS NOT.**
SCL was counted on the clock pad's own input buffer while an address
phase ran - the pad is an open-drain alternate function, so its IDR is the
wire - and the period measured less the tSCLL + tSCLH the register
programs is the detection delay 23.4.9 calls tSYNC1 + tSYNC2, judged on
the MEAN gap of the probe's clocks (nothing stretches them; the shortest
gap moves with the polling loop's phase by up to a hundred core cycles).
On the 2.2 kOhm wire it came out **between about 170 and 400 ns** over
the six rows that count the STOP's rise (three speeds on the APB1, two on
the HSI, standard mode on SYSCLK) - always above the floor the chooser
charges (22, 89 and 250 ns at 180, 45 and 16 MHz), and well under the
1000 / 750 / 500 ns tables 134 and 135 assume. SYSCLK's two faster rows
read more, 328 and 523 ns: their completion arrives before the STOP's
rise, and the NACK's handler, on the core the loop runs on, dates the last
counted rise late. Letter `f`, the mean rate on the pad:

| kernel | speed | stated (the ceiling) | measured | tSYNC measured |
|--------|-------|----------------------|----------|----------------|
| APB1 45 MHz | Sm | 99557 Hz | 97826 Hz | 267 ns |
| APB1 45 MHz | Fm | 394736 Hz | 367346 Hz | 278 ns |
| APB1 45 MHz | Fm+ | 1000000 Hz | 857142 Hz | 256 ns |
| SYSCLK 180 MHz | Sm | 98955 Hz | 97560 Hz | 167 ns |
| SYSCLK 180 MHz | Fm | 396475 Hz | 330882 Hz | 523 ns |
| SYSCLK 180 MHz | Fm+ | 1000000 Hz | 765957 Hz | 328 ns |
| HSI 16 MHz | Sm | 98765 Hz | 98522 Hz | 275 ns |
| HSI 16 MHz | Fm | 400000 Hz | 378151 Hz | 394 ns |

Not one row runs above the speed asked. The APB1 delays STATED to
`init()` give 99722, 403587 and 1000000 Hz on the pad (1805, 446 and 180
core cycles of mean gap) against the 99557, 398230 and 1000000 the
arithmetic states - the period formula with the wire's own tSYNC, to the
prescaler's resolution and to the instrument's. A turn of the loop that
counts the edges is 40 to 43 core cycles at 180 MHz (the letter measures
it: the cycles between the first and the last edge over the turns between
them), each edge is dated to within one turn, so a mean of eight gaps is
good to about five cycles - three per cent at Fm+, where the last row read
178 to 181 cycles from one image to the next at the same TIMINGR. The
letter allows exactly that above the speed and no more. A word solved against
the manual's budgets runs this wire FAST - 107.4 kHz, 497 kHz and 1.36
MHz on APB1, past fast mode's 400 kHz ceiling - which is why the default
is the floor.

**ST's own arithmetic is the oracle, and its error runs FAST.** The HAL
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
this one (its halves plus the measured delays) 105.4 kHz, 450 kHz and
1.10 MHz - all three above the rate asked, the Fm one past the mode's
ceiling. The two methods share every inequality (the setup and hold
bounds, 23.4.3's conditions) and differ in that one charge;
`brio/stm32f4/fmpi2c.hpp` pins ST's three words at the bottom.

**THE OTHER I2C OF THIS FAMILY KEEPS ITS STATED RATE.** The classic I2C1
(CCR, RM0390 27.6.8), run as a host through the same probe on the same
wire and counted on PB8: 100055 Hz against the 100000 its CCR states,
394736 Hz against 394736 - within one core cycle of a period at 180 MHz,
the poll's resolution. Its arithmetic is [i2c.md](i2c.md)'s; what the
comparison shows is that its period does not grow by a detection delay
the way this block's does, and that, CCR rounded up, its stated rate is
its ceiling too. The pad gives nine rising edges at 100 kHz and eight at
400 kHz, 23 runs of 23 each. This host's completion is the AF the ninth
clock raises, inside that rise's high phase, so the letter reads the pad
once more in the pass that sees the completion. At 100 kHz the high phase
is 4 us, 720 core cycles, against the error vector's longest entry of
about 200, and that read finds the rise. At 400 kHz it is 0.6 us, 108
cycles: the handler outlasts it and the line is low again, so nine is
what the letter asks at 100 kHz and eight at 400.

**A NACK on the address is a complete transaction, and its two flags come
in order.** All 112 addresses from 0x08 to 0x77 answered `i2c_nack_addr`
and nothing answered any other way. The event vector's own reading of ISR
shows the sequence: 0x8011 first (BUSY, NACKF, TXE - the address
unanswered, no data byte moved) and 0x0021 second (STOPF, TXE - the STOP
the peripheral sends by itself), on two separate interrupts, with the
ERROR vector never raised. After it BUSY is clear and both lines read
high: the STOP landed on the wire.

**The idle time-out needs no peer.** TIMEOUTA with TIDLE set watches both
lines standing high, which is what an empty bus does: armed at 50 us it
raised TIMEOUT after **51.1 us**. It is a LEVEL dressed as an event -
cleared on a bus that is still idle it came back in another 8985 core
cycles, 50 us again. The same code with TIDLE clear - the SCL-low watcher
- stayed quiet over the same wait, which is the two modes measured apart.

**AND AN ARMED TIME-OUT REACHES THE ERROR VECTOR, which is where this
engine sweeps it.** TIMEOUT is one of the six conditions behind ERRIE,
and the host arms ERRIE for BERR and ARLO: with the vector live the flag
was cleared by the engine's error body twice in the same wait, before a
polling loop could see it at all. Measured while chasing exactly that -
the span above read as fourteen periods whenever the sweep won the race,
which is why the measurement takes the interrupt away first. What it
means for an application is the design decision stated above: a time-out
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
which is the interrupt's whatever carries the data.

**The arbiter is unchanged over this second block of the same family.**
Four probes through `I2cBus` gave four replies, all `i2c_nack_addr`; six
posted into a four-deep queue gave six replies with one `i2c_rejected`
from the queue; a Fast-mode Plus request on the HSI came back
`i2c_rejected` through the arbiter, a synchronous completion of the
engine; and an idle bus voted for the sleep. Letter `d` judges the other:
0x00, 0x07, 0x78, 0x7A, 0x7B, 0x7C and 0x7F each answered true and
`i2c_rejected` by `start()`, CR2 and the interrupt enables as they
were.

**`unstick()` left a free wire alone**, answering 0 pulses, and the pads
came back to the peripheral after it.

**No bus error was ever seen.** ES0298 2.12.3's spurious BERR did not
fire once on this die over the whole suite; the count is printed rather
than judged.

**THE FIRST ACKNOWLEDGED BYTES: the host against I2C1** (letter `m`),
I2C1 a target at 0x3A served from its own two vectors - a byte sink and
a pattern source - its byte asked for ahead of the host's closing NACK
dropped by a PE cycle, its buffer interrupt silenced outside a read
(TxE stands between tenures on that block, [i2c.md](i2c.md)). The
probe, an absent address, a write of eight, reads of one, two, three,
four and eight bytes and a write of two then a read of four, byte-exact
at 100 and 400 kHz through the pump, and at 400 kHz through the DMA
engines (DMA1 streams 5 and 2, channel 2): the byte pump, the TC branch
that turns the write half into the read on one CR2 store, the receive
path and the engines moving bytes. The LAST written byte of a
write-then-read refused by the target (one to four written, four times
a count, both speeds): 32 refusals of 32 `i2c_nack_data`, the bus let go
(START and STOP down, BUSY clear, 2 ms on) and the next tenure `i2c_ok` -
this block has no repeated START to withdraw, the STOP after a NACK
being its own (23.4.9). And THE ENGINE EACH REQUEST NAMES, judged on the
streams - a stream's memory address, zeroed before the tenure, names the
tenure's buffer after it only if a block start wrote it, and its vector
is entered once by the block's completion: under `dma` a write of one
byte starts no stream and enters no stream vector, a write of two starts the
stream on its second byte, a read of one starts its stream, all byte-exact;
under `interrupt` the engined host's write of eight and read of eight start
neither and
enter no stream vector; the last written byte refused under `dma`, 16
refusals of 16 `i2c_nack_data`, the bus let go, the next tenure
`i2c_ok`; and `dma` on the engineless host is answered `i2c_rejected`
inside `start()`, CR2 untouched, BUSY low and no address match at I2C1
over 200 us, the next write on that host exact. AND THE WRITE AFTER A FAILED
ONE, byte for byte: an absent address and each refused position of writes of
one, two, three and eight, 18 failures a pass with no PE cycle after them,
each followed by a write of eight compared with what I2C1 kept: none wrong,
under `dma` and `interrupt` followed by either, and on the engineless host.
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
write 1142 cycles beyond the wire in two interrupts, a register read 1+1
1699 in four, a 255-byte write and read x 1.00 on the pump (256 interrupts, 136
cycles of handler a byte written and 141 read) and on the engines (two
interrupts), `start()` 324 to 330 cycles on the pump, the longest entry 522
cycles of the event vector. The fixed cost carries the target's: the
classic block stretches SCL from its address match until its vector has
run, on the same core; I2C1 as the host against this block as the target
pays 212 for the same write ([i2c.md](i2c.md)).

**Which engine, by the cycles** (the same letter, every row of one image;
`irq` and `isr` the two I2C vectors' and the two streams', between the
meter's stamps; on this block the engines are started in `start()`,
so its cycles are counted too). At 400 kHz; at 100 kHz every row is
within three cycles of these:

| half | `interrupt`: start, irq, isr | `dma`: start, irq, isr | `dma` - `interrupt` |
|---|---|---|---|
| write 1 | 330, 2, 391 | 468, 1, 397 | +144 |
| write 2 | 330, 3, 517 | 520, 2, 488 | +161 |
| write 3 | 330, 4, 653 | 520, 2, 488 | +25 |
| write 16 | 330, 17, 2425 | 520, 2, 488 | -1747 |
| read 1 | 325, 2, 394 | 484, 2, 623 | +388 |
| read 2 | 325, 3, 535 | 484, 2, 623 | +247 |
| read 16 | 325, 17, 2509 | 484, 2, 623 | -1727 |
| register read 1+1 | 326, 4, 745 | 532, 3, 828 | +289 |
| register read 1+2 | 326, 5, 886 | 532, 3, 828 | +148 |
| register read 1+16 | 326, 19, 2860 | 532, 3, 828 | -1826 |

The first byte's place shows in the `dma` column. A write of two or more
pays the TXDR flush and the store, seven instructions in the release listing
of which two are stores across the bridge: 520 cycles of `start()` against
the read's 484, whose stream program is the same and whose path holds neither.
A write of one programs no stream and takes no stream vector, 468 cycles and
one interrupt, the STOP's. The `interrupt` column stores nothing before the
START.

The pump's figure a byte past the first is the slope, 136 cycles written
and 141 read, one entry each, and an entry's own exception entry and
return - about 24 cycles on the Cortex-M4 (ARM DDI 0439B 3.9.1, twelve
each way at zero wait states), which no stamp sees - comes on top for
every interrupt counted. So, in core cycles, **a write half costs the
core fewer cycles on the engines from three bytes** counting the entries
(+25 on the stamps, with two interrupts fewer; from four on the stamps alone),
**a read half from four**, and **a register read with a one-byte index from a
read of three** counting the entries (+7 by the slope, three interrupts fewer).
The engines' fixed cost to the core, their program in `start()` and their
completion, is about 290 cycles for a write half of two or more and 390 for a
read half - a write tenure's end stops no receive stream - against a byte's 140
or so on the pump. The speed moves none of these lengths.

These figures are of one image, and placement moves them. The pump's path
is the same instructions in images that differ elsewhere, its `start()` the
same size to the byte, and across three such images it read 124, 136 and 146
cycles a byte written: at 180 MHz the flash has five wait states behind its
accelerator, and where a handler's branches land changes what it pays. So a
length above is good to about one byte, and a difference of a few dozen cycles
between two images is no finding. The core
sleeps while a stream works: the platform's idle is a WFI that asks no
stream anything, and Sleep keeps the DMA clocked (RM0390
6.3.15: DMA1LPEN is set at reset and written by no verb here) - so on
this family `dma` costs no sleep, only its fixed cycles.

**AGAINST THE DESK PARTNER** (set `t`, letters `n`, `o` and `p`, 33
verdicts): letter `n`'s 12 green ten runs of ten, the other two's below; the
STM32G0 beside this board on the same two nodes,
running `twi_peer` - with it on its dark console for `z`, whose letter e
counts 112 NACKs that its command address would spoil.

- **FAST-MODE PLUS WITH EVERY BYTE ACKNOWLEDGED**: a write and a read of
  eight bytes at the 1 MHz row, byte-exact, the partner's target - solved
  for Fm+ - accounting 8 taken and 8 served; the write averages **762..773
  kHz** of SCL (81 periods in a stopwatch that holds the request's cost
  and the partner's polled target stretching each byte), never above
  the rate.
- **`unstick()` against a client that REALLY holds SDA**: held by the
  partner's GPIO until four SCL falling edges (latched by its EXTI), the
  line is free at the **fourth pulse** and `unstick()` answers 4; held to
  its deadline, **0xFF**; on the freed wire, 0.
- **The arbiter's per-bus timeout over this engine**: a tenure the
  partner stretches 40 ms at its address is answered `i2c_timeout` by
  `I2cBus` at its 20 ms limit, and the same host carries the command
  channel after `recover()`.
- **TIMEOUTA AGAINST A HELD CLOCK** - the half letter g could not stage:
  with the clock-low watcher at 25 ms and the partner holding SCL 40 ms
  at the address, TIMEOUT reaches the error vector **25 ms** into the
  hold, every run - and THE TENURE GOES ON: this engine sweeps the flag
  (no tenure here arms a time-out of its own), the silicon sends nothing
  of its own on it, and when the holds are over the write completes
  `i2c_ok` (146..147 ms for two bytes held 40 ms each and at the
  address).
- **`rebase()` WAS READING THE PAST, and now derives** the APB1 rate from
  the new SYSCLK (`apb_hz_at`, [clock.md](clock.md)): the DynamicClock's
  fan-out runs before the RCC moves, so a divider read back there is the
  rate being left. Read back, a switch from 180 to 48 MHz solved TIMINGR
  for 12 MHz where 24 ran and the bus went at **1811 thousandths** of 100
  kHz, and at 16 MHz at 1694; derived, the three cores give **955, 942
  and 901 thousandths**, byte-exact against the partner at each.

**ES0298 2.12.8: NO STALL ON THE HOST'S PATHS, IN THE BAND OR OUT OF IT.**
Letter `o` runs writes of 2, 16 and 255 bytes to the partner's target with
the HSI as the kernel against the 45 MHz APB1, a ratio of 2.81, on both
engines, at 100 and 400 kHz; a stall is a tenure with no completion inside
twice its wire time plus 3 ticks, recovered by `recover()`'s PE cycle, the
partner's count, sum and address matches read back after each batch of a
hundred writes (twenty of the 255). Beside them runs a CONTROL the host does
not contain, staged by the letter through the resource's verbs: TXDR left
empty, the stream started on the whole run before the START, STOPF polled -
the sequence the erratum describes. Over 46 runs:

| kernel | writes on | tenures | stalled |
|---|---|---|---|
| HSI, ratio 2.81 | the event vector | 20240 | 0 |
| HSI, ratio 2.81 | the DMA engines | 20240 | 0 |
| HSI, ratio 2.81 | the DMA engines, writes of one | 4600 | 0 |
| HSI, ratio 2.81 | the control, TXDR left empty | 20240 | 2798, 13.8 per cent |
| APB1, ratio 1.0 | the event vector, writes of 16 | 4600 | 0 |
| APB1, ratio 1.0 | the DMA engines, writes of 1, 2, 16 and 255 | 14720 | 0 |
| SYSCLK, ratio 0.25 | the DMA engines, writes of 2, 16 and 255 | 10120 | 0 |

Every byte of every host row is on the partner, its count, sum and address
matches exact. The control's rate does not follow the length or the speed:
12.7 to 14.9 per cent in every row, a two-byte write as often as a 255-byte
one, which is the erratum's own account of a fault at the first two bytes.
Every stall of it is the same: ISR showing BUSY alone, SCL low and SDA high,
the transmit stream's count two beats down, and the partner holding exactly
the first byte of that write, its count and sum exact across the batch. The
next tenure after the PE cycle is whole. What the two engines and the control
differ in is the erratum's first clause. Two kernel cycles of the HSI are 125
ns, 22 core cycles at 180 MHz, a window a stream's request and its one-word
store through the bridge fall inside. With the first byte in TXDR before the
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

**A REFUSED WRITE, POSITION BY POSITION** (letter `p`, 12 verdicts): the partner
refuses through its target byte control the data byte a command names, or parks
at another address for a NACK on the address. Nineteen rows - the address under
writes of one, two and sixteen and under a 1+4 and a 2+4; the first byte of one,
of a 1+4, of two, of a 2+4 and of sixteen; the second of two, of a 2+4 and of
sixteen; the third of three and of sixteen, the seventh and last of sixteen, the
200th and last of 255 - run under each engine, at both speeds, on the APB1 and
on the HSI, each followed by a request on either engine: 304 rows a run, with no
`init()` and no PE cycle between a failure and what follows it, since either
would flush TXDR. Each is judged on `start()`'s answer, on `status()` and on the
partner's count, sum, first byte, address matches and word that it refused. Under
`dma`, over twelve runs of twelve:

| refused | rows | right |
|---|---|---|
| the address | 480 | 480, `i2c_nack_addr` |
| a write of one | 192 | 192, `i2c_nack_data` |
| the first byte of two or more | 304 | 304 |
| the second | 278 | 278 |
| a later byte | 570 | 570 |
| either byte of a write of two (counted above too) | 576 | 576 |

The 1824 rows on the event vector are right too. The positions land by the
partner's own count, which is the byte asked for or one sooner, 35 rows of 1824
sooner: its target polls RXNE and TCR on separate passes and decides at TCR on
what it has counted, so a pass that sees RXNE first refuses the byte it has just
counted. The letter judges the sum over that count.

How the vector tells them apart, under `dma`. A byte stays in TXDR with TXE clear
until the ninth pulse of an acknowledged address sends it to the shifter (23.4.7),
so TXE set says the first byte left: a data NACK, and that alone answers a write
of one. TXE clear does not answer two bytes or more: the byte leaving raises TXIS
and the stream refills TXDR while it is on the wire, so TXE reads clear after a
data NACK as after the address's. There the stream's count answers: SxNDTR is
post-decremented when a beat is stored into TXDR (9.3.3), not when direct mode
preloads it (9.3.7), so a count still at the stream's length means no refill. A
stream not served yet leaves its count standing under a byte already gone, and
TXE tells that one: the row that takes the transmit request away after `start()`
answers `i2c_nack_data`, the first byte refused, the next request whole. One
register read and one stream read, on the NACK path alone. A read half takes no
NACK on its data, so a NACK there is the address's.

And nothing of a failed request reaches the next one. The byte loaded ahead, the
next one on either engine or a first byte stored before an address nobody
answered, stays in TXDR through the NACK and the STOP, so the vector empties TXDR
at the NACK: a flush, a store that takes down a TXIS nobody served, and a flush of
that store, on the failed path alone. With that emptying taken out, the next
write on the event vector sends the stale byte ahead of its own and drops its own
last: 10 failures of 18 wrong on the far end of letter `m` after `interrupt`, 4 of
18 after `dma` (the other rows' byte goes with the stream's abandon, and a next
write under `dma` flushes before it stores, so that one is whole); and the row with
the request taken away never returns, TXIS re-entering the vector the moment TXIE
is armed again. A START lost into a line the partner holds low, with the
first byte stored and the stream armed, answers `i2c_arb_lost`, the stream taken
back and the next request whole, twelve runs of twelve. After a refused address the
next request is a write in the same serve, the partner's sum judging it byte for
byte; after a refused byte the partner refuses to its deadline, so the next
request is a command frame, whose checksum judges it, and letter `m` judges the
same case byte for byte against I2C1.

**Under the preemptive kernel: `test_stm32f4_rubato_fmpi2c`**, the twin of
[i2c.md](i2c.md)'s `test_stm32f4_rubato_i2c` - the same source over this
block - against the same partner's `twi_peer`: `util/i2c_bus.hpp`'s
arbiter (pending depth 4, the per-bus timeout at 20 ms) over the engined
`FmpI2cHost<1>` on PC6/PC7 (DMA1 streams 5 and 2 on channel 2, the APB1
kernel clock, the Fm+ pad drive on) inside Rubato, while TIM2 posts to
the pack's highest object every 97, 193 or 389 us and its dispatch spins
300 or 3000 cycles. Every letter runs once with TIM2 silent and six
times under it; **209 verdicts, green on every run: thirty of the
letters `n`, `o`, `p`, `w` and `i`, ten of `h` and of `q`**. The pack
and the letters
are the I2C suites' shared body (`avrxt/src/apps/rubato_i2c.hpp`), and
[platform.md](platform.md) places the suite among the preemptive
kernel's.

- **Where a preemption reaches this host: before its one START store.**
  `start()` waits for nothing; it arms the engines and then writes CR2
  with START last, so a dispatch above the arbiter landing in it delays
  the START. Letter `x`, 400 writes of 16 bytes: `start()` takes 369
  cycles with TIM2 silent (the longest 527; the classic block's takes
  174); under 97 us x 3000 the interrupt landed inside it 14 to 17 times
  in three runs, against the 8.5 that independent ticks would give (400 x
  369 / 17460), and 1 of the 48 with ISR.BUSY set - after the START
  store, `start()`'s last register write, the engines already armed.
  Over twenty runs of the other letters it found BUSY set inside
  `start()` 15 times, the same shadow.
- **Every shape on the engines, under every pressure as in the control**
  (letter `o`, at 400 kHz against one serve counted to its last byte):
  the shared body's Requests name no engine, and the suite names `dma`
  for every one in a wrapper around `start()`, so every data half rides
  the engines, a one-byte read included and a one-byte write its store alone - and the
  probe, the reads, the write-then-reads and the
  writes are every one `i2c_ok`, every read byte-exact, the partner's
  count, sum and address matches exact. The 32-byte write 819 to 821 us
  silent and at most 839 pressed (the classic block's 761: this block's
  400 kHz row is a ceiling, its SCL slower, as above), the probe 41 to 42
  and at most 59.
- **Three speeds and the commanded stretch** (letter `p`): a read of 16, a
  1+4 and a write of 16 byte-exact at 100, 400 and 1000 kHz, the write of
  16 in 1589 to 1611, 428 to 448 and 192 to 218 us against the wire's
  1530, 382 and 153 - at Fm+ the partner's polled target stretching each
  byte, as above. The partner's stretch of 1000 us a byte at 100 kHz: a
  read of 8 in 9208 to 9214 us against its nine holds, a write of 8 in
  8990 to 9025 against its eight (the write follows the read at once and
  meets most of one hold more, [i2c.md](i2c.md) has the arithmetic).
- **A REFUSED DATA BYTE IS TOLD FROM A REFUSED ADDRESS under the
  engines** (letter `q`): the deaf client's probe and its write, the
  write on the transmit engine, answer `i2c_nack_addr` and the next
  tenure - a probe of 0x5D, where the partner parks - `i2c_ok`; the third
  byte of a six-byte write refused answers `i2c_nack_data`, and so does
  the last written byte of a 2+4 refused, whose two written bytes the
  partner reports received, its address matches the two a host that
  lets go with a STOP makes; a write to 0x7A and one to the general call
  answer `i2c_rejected`, the partner, answering the general call,
  matching only the probe that follows - 35 verdicts of 35 in ten runs
  of the letter, and the twin's `z` 209 of 209 in three runs with it. How the event vector tells the two apart is "A REFUSED WRITE" above; the
  engineless host does it by its pump's position (letter `m` above).
- **The arbiter with three requesters of three priorities** (letter
  `w`): every reply `i2c_ok`, to its requester, in its order, nothing
  rejected, no timeout, the partner's tallies exact, in 5.8 to 6.3 ms a
  pass.
- **THE HELD WIRE: A PARK AT THE SPEED IN FORCE, ARBITRATION LOST AFTER A
  PE CYCLE** - the STM32G0's two signatures, this block being its register
  file ([../stm32g0/i2c.md](../stm32g0/i2c.md)). With the partner holding
  SDA low 60 ms, the tenure posted at the speed in force parks behind
  BUSY and is answered `i2c_timeout` by the arbiter (in 19988 to 20033 us,
  138 passes of 140, with a probe posted at every tick; in 19981 to
  20026 us from its `start()`, 20 of 21, with the flood paced by the
  Console's turns); where a probe the preempting object had posted first parked
  and timed out instead, the arbiter's `recover()` - 23.4.6's PE cycle,
  which clears BUSY - then started the held tenure, and its START went out
  into the held line and lost: `i2c_arb_lost` 47 us from its `start()`,
  at 20044 to 20059 us from its post. The same tenure at 400 kHz after
  commands at 100 kHz pays `apply()`'s PE cycle before its START and is
  answered `i2c_arb_lost` in 26 to 29 us, every run. Under the flood (180
  to 722 probes a pass, 721 or 722 of 931 to 941 ticks at 97 us x 3000) every
  request was answered exactly once - while held `i2c_rejected`, and
  `i2c_arb_lost` for nearly every probe that reached the wire after a
  `recover()`, after the release `i2c_ok` or `i2c_nack_addr` - and a
  request arrived in the arbiter's draining window in 2 of 120 pressure
  passes with a probe at every tick, 1 of 60 with the flood paced; after
  the release the next tenure was
  `i2c_ok` and the arbiter voted idle, every pass.
- **The bus error count** rose by one in each pass of letter `p` and
  nowhere else, seven a run, as on the classic block beside it
  ([i2c.md](i2c.md) places it at the write that ends the partner's
  counted serve at 100 kHz); ES0298 2.12.8's stall band reads clear at
  this kernel clock.
- **Dark** (letter `i`): every object idle, the arbiter's vote, no time
  event armed, TIM2 stopped with its DMA requests off, the host between
  tenures with TCIE off and both DMA request enables clear, both streams
  idle, BUSY clear and PC6 and PC7 reading high on their pads.

## Not covered yet

Driver gaps, each with its reason:
- The PEC engine and the SMBus protocol above the time-outs (the alert
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
  its vector), and the desk partner, the STM32G0B1RE's `twi_peer`, has no
  `host_read` to read this board's target. Born with the first util
  target object.

Implemented, not bench-verified (each with what would measure it):
- Arbitration lost between two controllers: a race wants two STARTs
  committed inside each other's detection window, and on this desk two
  held STARTs released by one STOP serialize
  ([../stm32g0/i2c.md](../stm32g0/i2c.md), "TWO HELD STARTS", measured
  with this chip's other I2C). What would measure it is a peer releasing
  its START by software at a delay swept across this one's. The status's
  own path is measured: a START into a data line the partner holds low
  loses its address and is answered `i2c_arb_lost` (Bench findings).
- ES0298 2.12.3's spurious BERR told apart from a real one: the count
  moved under the preemptive kernel's suite, once a pass at a write that
  ends the partner's counted serve at 100 kHz (Bench findings), which the
  partner's re-init inside that byte's acknowledge would explain as a
  misplaced STOP - a real one, counted the same. A scope on the two lines
  at that write would separate them.
- The noise filters ON THE WIRE: the register is written and read back
  and the refusals the filter delay causes are measured, but what a
  9-period digital filter does to a glitch wants a glitch generator.
- ES0298 2.12.8 in target mode, which the erratum names: the client gives
  its first byte on TXIS and nothing has read this block as a target inside
  the band. The partner's `twi_peer` has no `host_read` (twi_link's extended
  repertoire), and that op on the HSI kernel is what would measure it.
- TIMEOUTB, this peripheral's own cumulative stretch, which only a
  target being read produces: the partner's `twi_peer` has no `host_read`
  (twi_link's extended repertoire) to read this block as a target, and
  that op is what would measure it.
- The Fast-mode Plus drive's EFFECT: the two bits are written and read
  back, but the 20 mA sink shows itself in a fall time, which wants an
  oscilloscope and a bus with real pull-ups.
- The instance on the other three part classes that carry it (the F410,
  F412 and F413/F423): the driver compiles for all of them, their DMA
  request tables are refused for want of a manual, and no board here
  carries one.
