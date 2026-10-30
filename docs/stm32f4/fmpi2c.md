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
the one instance, its two DMA engine slots left empty; `BusAo` is the
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
a byte moves (`speed_ok()` says so beforehand); a Fast-mode Plus bus
wants the 20 mA pad drive turned on by the program
(`Bus::fast_plus_drive(true)`), which nothing turns on by itself. With
the DMA engines named, a refused DATA byte is answered `i2c_nack_addr`,
as a refused address is: the event vector tells the two apart by the
byte pump's position in the tenure, which the engines never advance -
measured with the transmit engine carrying the write ("Bench
findings"); the engineless host tells them apart.

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
ends by itself, and the completion is that STOP.

**Four errata are live in the driver.**
- 2.12.3, a spurious bus error in controller mode: "detection of bus
  error has no effect on the I2C-bus transfer in controller mode and any
  such transfer continues normally". The host clears BERR, counts it
  (`spurious_bus_errors()`) and never reports `i2c_bus_error` from it. A
  client's BERR is outside the erratum and is reported.
- 2.12.8, a transmission stalled after its first byte when the first byte
  is not already in TXDR, the second is written within two kernel cycles
  of its request, and the APB-to-kernel clock ratio is between 1.5 and 3.
  The ratio is arithmetic, so it is published rather than hidden
  (`transmit_stall_risk()`), and the two workarounds are the manual's own:
  load TXDR before the transfer starts - which the DMA path does by
  construction, the engines being armed before the START bit - or choose a
  kernel clock outside the band.
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
DMA tenure still costs the framing interrupts.

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
(`wake_from_stop`), and `release` (the block disabled and its RCC reset
pulsed before the gate - the release contract, docs/design/dma.md - for
the host and the client alike). The constants `number`, `event_irq`,
`error_irq`, `smbus_claimed`, `wakes_from_stop`, `has_ten_bit`,
`has_fast_plus`.

`FmpI2cHost<n, pins, TxEngine, RxEngine>` - the bus engine, with the
parameter list and the `Request` of [i2c.md](i2c.md)'s `I2cHost`, field
for field, the speed an `FmpI2cSpeed` (with `fast_plus_1m`): `init`, `rebase`, `speed_ok`, `timing_of`, `scl_hz`,
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
// Request does.
```

**A bus whose edges are known.** The defaults are the specification's
worst-case edges for the delays and no measured tSYNC, so each speed asked
is a ceiling the wire stays at or under (about 2, 7 and 13 per cent under
on this bench's wire at 45 MHz). A board that has measured its wire - the
period less the register's tSCLL + tSCLH, which `test_stm32f4_fmpi2c`'s
letter `f` prints - states it, and gets the rate asked for:

```cpp
Bus::init(clock, FmpI2cHostConfig{
    .bus = FmpI2cBusTiming{.rise_ns = 250, .fall_ns = 60,
                           .setup_ns = 100, .sync_ns = 240}});
```

**The DMA engines**, on the only two cells the request mapping gives:

```cpp
using Bus = FmpI2cHost<1, pins, DmaTxEngine<1, 5, 2>, DmaRxEngine<1, 2, 2>>;
Dma<1>::init();                      // the controller is the app's
Bus::init(clock);
extern "C" void DMA1_Stream5_IRQHandler() { if (Bus::dma_isr()) { /* ... */ } }
extern "C" void DMA1_Stream2_IRQHandler() { if (Bus::dma_isr()) { /* ... */ } }
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
I2C1 the target. `test_stm32f4_fmpi2c` is 95 verdicts over thirteen
letters.

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
against a 45 MHz APB1 is a ratio of 2.8 and sits INSIDE the 1.5..3 window
where a transmission can stall, while the APB clock (1.0) and SYSCLK
(0.25) do not.

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
On the 2.2 kOhm wire it came out **between about 160 and 390 ns** over
the eight rows (three speeds on two kernel clocks, two on the third) -
always above the floor the chooser charges (22, 89 and 250 ns at 180, 45
and 16 MHz), and well under the 1000 / 750 / 500 ns tables 134 and 135
assume. Letter `f`, the mean rate on the pad:

| kernel | speed | stated (the ceiling) | measured | tSYNC measured |
|--------|-------|----------------------|----------|----------------|
| APB1 45 MHz | Sm | 99557 Hz | 97985 Hz | 250 ns |
| APB1 45 MHz | Fm | 394736 Hz | 372670 Hz | 239 ns |
| APB1 45 MHz | Fm+ | 1000000 Hz | 865384 Hz | 245 ns |
| SYSCLK 180 MHz | Sm | 98955 Hz | 97613 Hz | 161 ns |
| SYSCLK 180 MHz | Fm | 396475 Hz | 375782 Hz | 162 ns |
| SYSCLK 180 MHz | Fm+ | 1000000 Hz | 873786 Hz | 167 ns |
| HSI 16 MHz | Sm | 98765 Hz | 98468 Hz | 280 ns |
| HSI 16 MHz | Fm | 400000 Hz | 378947 Hz | 388 ns |

Not one row runs above the speed asked. The APB1 delays STATED to
`init()` give 99778, 389610 and 1005586 Hz on the pad (1803 to 1805, 462
and 179 core cycles of mean gap) against the 99778, 391304 and 1000000
the arithmetic states - the period formula with the wire's own tSYNC, to
the prescaler's resolution and to the instrument's. A turn of the loop
that counts the edges is 26 to 29 core cycles at 180 MHz (the letter
measures it: the cycles between the first and the last edge over the
turns between them), each edge is dated to within one turn, so a mean of
eight gaps is good to about four cycles - two per cent at Fm+, where the
last row read 178 to 181 cycles from one image to the next at the same
TIMINGR. The letter allows exactly that above the speed and no more. A word solved against
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
wire and counted on PB8: 100111 Hz against the 100000 its CCR states,
396475 Hz against 394736 - within two core cycles of a period at 180 MHz,
the poll's resolution. Its arithmetic is [i2c.md](i2c.md)'s; what the
comparison shows is that its period does not grow by a detection delay
the way this block's does, and that, CCR rounded up, its stated rate is
its ceiling too.

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
`i2c_rejected` through the arbiter, which is the one synchronous
completion an I2C engine has; and an idle bus voted for the sleep.

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
being its own (23.4.9).

**The target half answers a real controller.** As the peer of
`test_stm32f4_i2c` on the same wires, `FmpI2cClient<1>` served from its
own two vectors is a register file at 0x2C - the address match, each
byte received and wanted, the controller's closing NACK (its byte asked
for ahead dropped by 23.7.7's TXE flush) all reported by `service()` -
through that suite's 67 verdicts, and CR2's NACK bit set as the byte
before is taken refuses a chosen written byte (23.7.2: "a NACK is sent
after current received byte"), 192 refusals of 192 on time with the core
at 180, 48 and 16 MHz ([i2c.md](i2c.md)).

**What a tenure costs this host** (`bench_stm32f4`'s letter `i` on the
same wires, I2C1 the target served on the same core, its entries counted
apart; the SCL period measured on the pad, 484 core cycles at the 400
kHz row - 371.9 kHz, under the ceiling as above). At 400 kHz: a one-byte
write 1104 cycles beyond the wire in two interrupts, a register read 1+1
1674 in four, a 255-byte write and read x 1.00 on the pump (256 interrupts, 136 cycles
of handler a byte written and 151 read) and on the engines (two
interrupts), `start()` 315 cycles, the longest entry 518 cycles of the
event vector. The fixed cost carries the target's: the classic block
stretches SCL from its address match until its vector has run, on the
same core; I2C1 as the host against this block as the target pays 230
for the same write ([i2c.md](i2c.md)).

**AGAINST THE DESK PARTNER** (set `t`, letter `n`, 12 verdicts, green ten
runs of ten): the STM32G0 beside this board on the same two nodes,
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

**Under the preemptive kernel: `test_stm32f4_rubato_fmpi2c`**, the twin of
[i2c.md](i2c.md)'s `test_stm32f4_rubato_i2c` - the same source over this
block - against the same partner's `twi_peer`: `util/i2c_bus.hpp`'s
arbiter (pending depth 4, the per-bus timeout at 20 ms) over the engined
`FmpI2cHost<1>` on PC6/PC7 (DMA1 streams 5 and 2 on channel 2, the APB1
kernel clock, the Fm+ pad drive on) inside Rubato, while TIM2 posts to
the pack's highest object every 97, 193 or 389 us and its dispatch spins
300 or 3000 cycles. Every letter runs once with TIM2 silent and six
times under it; **174 verdicts, green on every run: thirty of the
letters `n`, `o`, `p`, `w` and `i`, ten of `h`**, the vocabulary letter
`q` kept out of them (below). The pack and the letters
are the I2C suites' shared body (`avrxt/src/apps/rubato_i2c.hpp`), and
[platform.md](platform.md) places the suite among the preemptive
kernel's.

- **Where a preemption reaches this host: before its one START store.**
  `start()` waits for nothing; it arms the engines and then writes CR2
  with START last, so a dispatch above the arbiter landing in it delays
  the START. Letter `x`, 400 writes of 16 bytes: `start()` takes 368
  cycles with TIM2 silent (the longest 520; the classic block's takes
  189); under 97 us x 3000 the interrupt landed inside it 4 to 15 times
  in five runs, against the 8.4 that independent ticks would give (400 x
  368 / 17460), and 5 of the 36 with ISR.BUSY set - after the START
  store, `start()`'s last register write, the engines already armed.
  Over twenty runs of the other letters it found BUSY set inside
  `start()` 15 times, the same shadow.
- **Every shape on the engines, under every pressure as in the control**
  (letter `o`, at 400 kHz against one serve counted to its last byte):
  with the engines named every data half rides them, a one-byte read and
  a one-byte write included - this host keeps no pump for data and no
  crossover - and the probe, the reads, the write-then-reads and the
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
- **THE REFUSED DATA BYTE IS NAMED AS A REFUSED ADDRESS under the
  engines** (letter `q`, run by its name): the deaf client's probe and
  write answer `i2c_nack_addr` and its next tenure - a probe of 0x5D,
  where the partner parks - `i2c_ok`; but the third byte of a six-byte
  write refused answers `i2c_nack_addr` too, and so does the last written
  byte of a 2+4 refused, whose two written bytes the partner reports
  received, its address matches the two a host that lets go with a STOP
  makes - 14 verdicts of 28 failing, every pass of every run, the
  status alone. The event vector names a NACK by the byte pump's position
  in the half (`pos_ == 0`: the address), and with the engines carrying
  the bytes the pump's position never moves. The engineless host tells
  the two apart (letter `m` above); the twin keeps the letter out of its
  all-key until the driver does with its engines (a driver gap below).
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
- The general call and the two SMBus addresses as HOST verbs: the
  resource enables the target side's recognition of them, and no portable
  program needs the other half yet.
- The recovery LADDER - when to unstick, when to retry, when to take a
  bus out of service - is the application's or a future policy type's, as
  on every other stratum.
- The name of a refused data byte with the engines named: the event
  vector answers a NACK `i2c_nack_addr` while the byte pump's position in
  the tenure is zero, and the engines never advance it, so a data byte
  refused under an engine is answered as a refused address (measured,
  Bench findings). Telling the two apart by the streams' own counts, as
  the STM32G0's host does by its channel's, is owed to the driver.
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
- ES0298 2.12.8's stall: the ratio is computed and published, and the
  band was never entered with a transmission long enough to trip it -
  every tenure on the HSI ended on its address. The desk partner accepts
  bytes now; a write to it with this block on the HSI, at the ratio the
  erratum names, would put the erratum on trial.
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
