# FMPI2C (STM32F4)

Documents of record: RM0390 Rev 6 ch. 23 (the F446's manual; the F410,
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
request-mapping facts (`stm32f4/device_tables.hpp`), under the arbiter
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
the block - with NOTHING on the bus.

## What the silicon does

**IT IS THE STM32G0's I2C UNDER ANOTHER NAME.** CR1/CR2 with SADD,
NBYTES, RELOAD and AUTOEND; one TIMINGR word of five fields; OAR1 and
OAR2 with the second address's mask; ISR and ICR; PECR; TIMEOUTR;
separate RXDR and TXDR. Register for register and bit for bit it is the
block [../stm32g0/i2c.md](../stm32g0/i2c.md) describes, and the driver is
that driver with the same verb names and the same shapes. It is NOT the
block of [i2c.md](i2c.md), which is the F1 lineage's event machine and
another chapter entirely; a part that has both has two I2C designs on one
die, which is why every name in this file carries an `Fmp` prefix.

**WHAT DIFFERS FROM THE STM32G0, all of it this family's own:** TWO
VECTORS instead of one (table 142 splits the seven event conditions from
the six error ones, as every other peripheral of this stratum does), the
kernel-clock multiplexer in RCC_DCKCFGR2 instead of RCC_CCIPR, the Fm+
pad drive in SYSCFG_CFGR as TWO BITS PER SIGNAL instead of a bit per pad,
DMA STREAMS instead of channels - and NO WAKE FROM STOP.

**Table 127's row, and its one dash.** 7- and 10-bit addressing, all
three speeds up to Fast-mode Plus at 1 MHz, the independent clock and
SMBus/PMBus are all there; "wakeup from Stop mode" is not. The device
header agrees in the only way it can: no header of the pack declares an
FMPI2C_CR1_WUPEN, and 23.7.1 makes CR1's bit 18 reserved. So the driver
publishes `wakes_from_stop` as false and its `wake_from_stop()` refuses -
the one feature of the STM32G0's identical register file that this
silicon has not.

**ONE INSTANCE, AND TWO NAMES FOR IT.** Every part that carries the
block carries exactly one. ES0298 2.12.10 records the name as a
documentation erratum: "I2C4 and FMPI2C1 refer to the same peripheral
instance", which is why RM0390 6.3.27 calls the kernel-clock selector
"I2C4 kernel clock source selection" while the rest of the chapter says
FMPI2C1. The header's name is the one used here. The resource is still
`FmpI2c<n>` with an index, so the day a part arrives with two, no user of
the file is renamed.

**THE INDEPENDENT CLOCK is the reason to reach for this block.**
RCC_DCKCFGR2.FMPI2C1SEL picks the peripheral's own APB clock (code 0 and
code 3, "same as 00"), SYSCLK (1) or the HSI (2). A bus on the HSI keeps
its rate through every change of the core's, and holds a speed the
erratum's kernel-clock floor would refuse at a slow core; a bus on SYSCLK
gets a kernel far above what an APB prescaler leaves. The other I2C of
this family has no such selector: FREQ, CCR and TRISE all divide its own
APB clock and nothing else.

**THE TIMING REGISTER IS THE CHAPTER, and its arithmetic goes both
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

**tSCL IS NOT tSCLL + tSCLH.** 23.4.9's own formula adds tSYNC1 + tSYNC2
- the SCL slopes, the filter delays and two to three kernel periods of
synchronization at each edge - whose floor is 4 x tI2CCLK and whose value
the manual's example tables assume to be 1000 ns in Sm, 750 in Fm and
500..655 in Fm+. It is a BUS fact and not a chip one, so the driver takes
it as an argument (`FmpI2cBusTiming::sync_ns`) beside the rise and fall
times, defaulting to the tables' own numbers so that the arithmetic
reproduces them. What it costs to get that estimate wrong is measured
below.

**THE KERNEL CLOCK HAS A FLOOR AND IT IS AN ERRATUM'S.** ES0298 2.12.2:
below a kernel period that fits inside the transmitter's minimum data
setup time the block samples the PREVIOUS SDA value, and the erratum
names 4 / 10 / 20 MHz for Sm / Fm / Fm+ against the standard's own
tSU;DAT. There is no register workaround - "increase the FMPI2C kernel
clock frequency" is the whole of it - so the driver REFUSES a speed the
kernel clock cannot legally make, and answers a Request naming it with
`i2c_rejected` rather than running the bus at a rate nobody asked for.
23.4.3's own two conditions (tI2CCLK < (tLOW - tfilters)/4 and
tI2CCLK < tHIGH) are checked beside it and refuse independently: they are
what drops Fast-mode Plus when the digital filter is deep.

**THE TRANSFER ENGINE IS A BYTE COUNTER, not a set of software
sequences.** CR2 carries the address, the direction, NBYTES and the end
policy in one word; the START bit sends everything up to the address, and
from there TXIS and RXNE pace one byte each. AUTOEND sends the STOP when
NBYTES runs out; with AUTOEND clear the transfer ends on TC with SCL
stretched, and setting START there IS the repeated START. RELOAD carries
a transfer past 255 bytes through TCR - and ES0298 2.12.4 says a receiver
can lose the byte that raises it, so the host engine never sets RELOAD
(a Request's spans are eight-bit lengths) and the resource's `reload()`
carries the caution.

**A NACK IS AN EVENT AND NOT AN ERROR.** NACKF sits on the EVENT vector
under NACKIE, and 23.4.9 says that on a NACK "the TXIS flag is not set,
and a STOP condition is automatically sent" - so a tenure nobody answers
ends by itself, and the completion is that STOP.

**FOUR ERRATA ARE LIVE IN THE DRIVER.**
- 2.12.3, a spurious BUS ERROR in controller mode: "detection of bus
  error has no effect on the I2C-bus transfer in controller mode and any
  such transfer continues normally". The host clears BERR, COUNTS it
  (`spurious_bus_errors()`) and never reports `i2c_bus_error` from it. A
  CLIENT's BERR is outside the erratum and is reported.
- 2.12.8, a TRANSMISSION STALLED AFTER ITS FIRST BYTE when the first byte
  is not already in TXDR, the second is written within two kernel cycles
  of its request, and the APB-to-kernel clock ratio is between 1.5 and 3.
  The ratio is arithmetic, so it is published rather than hidden
  (`transmit_stall_risk()`), and the two workarounds are the manual's own:
  load TXDR before the transfer starts - which the DMA path does by
  construction, the engines being armed before the START bit - or choose a
  kernel clock outside the band.
- 2.12.4, the reload-mode byte loss: answered by never using RELOAD.
- 2.12.1, a 10-BIT address whose first byte is NACKed, which leaves NACKF
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

**THE ONE VERB THAT DISABLES IS 23.4.6's PROCEDURE.** Clearing PE
releases both lines, resets the state machines, clears CR2's START, STOP,
NACK and PECBYTE and every status flag - and KEEPS every configuration
register. PE must stay low for at least three APB cycles, and the chapter
prescribes write-0 / check-0 / write-1, so that sequence is `disable()`
plus `enable()` and there is no raw PE clear anywhere in the driver. It
is also the chapter's own deadlock escape, which is why `recover()` is
built on it and reconfigures nothing.

**THE WRITE GATES ARE FOUR AND NOT ONE.** PE = 0 for TIMINGR, NOSTRETCH,
ANFOFF and DNF; START = 0 for CR2's SADD, RD_WRN, ADD10, HEAD10R and
NBYTES; OA1EN = 0 for OAR1's own fields and OA2EN = 0 for OAR2's;
TIMOUTEN = 0 for TIMEOUTA and TIDLE, TEXTEN = 0 for TIMEOUTB. Each verb
checks its own.

**THE SMBus HALF IS WHOLE HERE**, unlike on some instances of the same IP
elsewhere: the two device addresses, the alert, the PEC engine and the
three time-outs. TIMEOUTA watches SCL held low by anybody (TIDLE = 0) or
both lines standing high (TIDLE = 1, bus idle detection); TIMEOUTB
watches this peripheral's OWN cumulative stretch. 23.7.6 makes the whole
register read back zero where the half is absent, which is a question the
silicon can answer - `smbus_probe()` asks it.

**THE PADS AND THE 20 mA DRIVE.** DS10693 table 11 puts FMPI2C1_SCL on
PC6, PD12, PD14 and PF14, FMPI2C1_SDA on PC7, PD13, PD15 and PF15 and
FMPI2C1_SMBA on PD11 and PF13, all at AF4. No device header carries a pin
table, so nothing in the code can check an alternate function: the
datasheet claims it and the bench proves it. Both lines go out OPEN DRAIN
- that IS the bus - and the pull-ups are the board's. Fast-mode Plus
wants the 20 mA output drive, which on this family is two bits of
SYSCFG_CFGR, one for the SCL signal and one for SDA (not one per pad):
whichever pad carries the signal takes the drive.

**THE DMA REQUESTS ARE ONE CELL EACH.** RM0390 table 28: FMPI2C1_RX on
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
(`wake_from_stop`), and `release`. The constants `number`, `event_irq`,
`error_irq`, `smbus_claimed`, `wakes_from_stop`, `has_ten_bit`,
`has_fast_plus`.

`FmpI2cHost<n, pins, TxEngine, RxEngine>` - the bus engine, with the
parameter list and the `Request` of [i2c.md](i2c.md)'s `I2cHost`, field
for field: `init`, `rebase`, `speed_ok`, `timing_of`, `scl_hz`,
`kernel_hz`, `reference_hz`, `transmit_stall_risk`,
`spurious_bus_errors`, `fast_plus_drive`, `claim_smba_pad`, `idle`,
`start`, `status`, `isr`, `error_isr`, `dma_isr`, `unstick`, `recover`,
`release`.

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
`fmpi2c_timing_for` (the chooser), `fmpi2c_scl_hz`, `fmpi2c_scll_ns`,
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
the `Request` its clients build is unchanged.

**Choosing the kernel clock**, which is what this block has and its
neighbour has not:

```cpp
Bus::init(clock, FmpI2cHostConfig{.kernel = FmpI2cClock::hsi});
// 16 MHz whatever the core does - and Fm+ is refused there, because
// ES0298 2.12.2 wants 20 MHz for it. speed_ok() says so before a
// Request does.
```

**A bus whose edges are known.** The defaults are the specification's
worst case and the manual's own tSYNC estimate; a board that has measured
its wire states it, and gets a bus that keeps to the rate asked for:

```cpp
Bus::init(clock, FmpI2cHostConfig{
    .bus = FmpI2cBusTiming{.rise_ns = 250, .fall_ns = 60,
                           .setup_ns = 100, .sync_ns = 400}});
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
pads are PC6 and PC7, which that package bonds and nothing on the board
uses. **THERE IS NO DEVICE ON THIS BUS AND NO BYTE WAS EVER ACKNOWLEDGED
HERE**: what follows is what a bus with nobody on it can be made to say,
and the wire is held up by the PORT's own pull-ups - some tens of
kiloohms - because the board has none. `test_stm32f4_fmpi2c` is 89
verdicts over twelve letters.

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
| Sm 100 kHz | 3 | 56 | 44 | 3 | 14 | 99337 Hz |
| Fm 400 kHz | 1 | 28 | 10 | 6 | 9 | 394736 Hz |
| Fm+ 1 MHz | 0 | 13 | 7 | 3 | 8 | 1000000 Hz |

Neither of the two that the kernel rate does not divide comes out above
the speed asked for: the period is rounded UP before it is split, which
is what keeps a 400 kHz request at 394736 Hz and not at 401785.

**THE MANUAL'S tSYNC BUDGET IS GENEROUS ON THIS BOARD, AND THAT MAKES THE
BUS FASTER THAN THE ARITHMETIC STATES.** SCL was counted on the clock
pad's own input buffer while an address phase ran - the pad is an
open-drain alternate function, so its IDR is the wire - and the period
measured less the tSCLL + tSCLH the register programs is the detection
delay 23.4.9 calls tSYNC1 + tSYNC2. Over eight rows (three speeds on two
kernel clocks, two on the third) it came out **between about 50 and
500 ns**, always positive and always well under the 1000 / 750 / 500 ns
tables 134 and 135 assume - the spread being what a polling loop that
timestamps a little before the edge it sees can resolve against a 9 us
period. So a bus solved against the defaults runs a few per cent fast:
105 kHz measured where the arithmetic states 99.3 kHz, 466 kHz where it
states 394.7 kHz, 1.12 MHz where it states 1.00 MHz. The rates are
printed and not judged - one board, one set of pull-ups - but the
direction is a design fact: `sync_ns` is an argument for exactly this
reason, and an application that must not pass the standard's ceiling
states its own measured one.

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

Implemented, not bench-verified (each with what would measure it):
- **THE WHOLE DATA PATH.** No byte has ever been acknowledged on this
  bus: every tenure this suite ran ended on the address. So the byte
  pump, the repeated START of a write-then-read, `i2c_nack_data`, the TC
  branch, the DMA engines actually MOVING bytes, and the client's
  `service()` reporting anything but `none` are all compiled, reasoned
  from a chapter that another stratum's identical block has proved, and
  unmeasured here. A DEVICE ON THE BUS - any I2C part on two wires to PC6
  and PC7 - would measure all of it in one letter.
- The client role: it needs a controller at the other end of a wire, and
  the boards of this stratum have none between them. A peer board, or a
  wire from this block to one of the part's other I2C instances.
- Arbitration lost (`i2c_arb_lost`): a second controller on the wire.
- A bus error and the spurious-BERR count: the chapter raises BERR on a
  misplaced START or STOP, which wants another controller misbehaving,
  and the erratum's own spurious one did not appear.
- The arbiter's per-bus timeout and `i2c_timeout` over this engine: a
  lost completion has to be staged, which wants a client that stretches
  the clock past the limit.
- `unstick()` against a client that really holds SDA down, and the 0xFF
  it answers when nine clocks and a STOP do not free the line: nothing
  here can hold SDA low without a wire.
- The noise filters ON THE WIRE: the register is written and read back
  and the refusals the filter delay causes are measured, but what a
  9-period digital filter does to a glitch wants a glitch generator.
- ES0298 2.12.8's stall: the ratio is computed and published, and the
  band was never entered with a transmission long enough to trip it -
  every tenure on the HSI ended on its address. A device that accepts
  bytes, at that kernel clock, would put the erratum on trial.
- `rebase()` across a clock change: the fan-out is compiled and not
  exercised; a transaction at every rate of a `DynamicClock` pack
  ([clock.md](clock.md)) is what would measure it.
- The SMBus time-outs that need a peer: TIMEOUTA's clock-low watcher was
  shown NOT to fire on an idle bus, which is half the story - a client
  holding SCL down past 25 ms is the other half. TIMEOUTB watches this
  peripheral's own cumulative stretch, which only a target that is being
  read produces.
- The Fast-mode Plus drive's EFFECT: the two bits are written and read
  back, but the 20 mA sink shows itself in a fall time, which wants an
  oscilloscope and a bus with real pull-ups.
- The instance on the other three part classes that carry it (the F410,
  F412 and F413/F423): the driver compiles for all of them, their DMA
  request tables are refused for want of a manual, and no board here
  carries one.
