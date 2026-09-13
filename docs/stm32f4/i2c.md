# I2C (STM32F4)

Documents of record: RM0090 Rev 22 ch. 27 (RM0390 Rev 6 ch. 24 and
RM0383 Rev 4 ch. 18 are its twins: one register description, three
manuals), the datasheets' alternate-function tables for the pads,
RM0090 table 43 with its RM0390 and RM0383 twins for the DMA request
mapping, and the errata's I2C items - ES0206 2.10, ES0298 2.11,
ES0287 2.9, the same six on all three parts: "spurious bus error
detection in controller mode", "SMBus standard not fully supported",
"start cannot be generated after a misplaced Stop", "mismatch on the
setup time for a repeated Start condition", "data valid time violated
without the OVR flag being set", "both SDA and SCL maximum rise times
violated when the VDD_I2C bus voltage is higher than ((VDD + 0.3) /
0.7) V". Driver: `stm32f4/i2c.hpp` (`I2c<n>` the resource,
`I2cHost<n, pins, TxEngine, RxEngine>` the bus engine,
`I2cClient<n, pins>` the other end, the vocabulary `I2cSpeed`,
`I2cDuty`, `I2cTiming`, `I2cFilter`, `I2cAddressConfig`,
`I2cSmbusConfig`, `I2cFlag`, `I2cPins`, `I2cHostConfig`,
`I2cClientEvent`, and the timing arithmetic), over the reserve's
instance, filter and request-mapping facts
(`stm32f4/device_tables.hpp`), under the arbiter and the status
vocabulary of [../design/i2c-bus.md](../design/i2c-bus.md), with the
streams from [dma.md](dma.md). The family fixture is
`test/family_stm32f4/i2c.cpp` with the negatives that refuse an absent
instance, one pad twice, a bus with no SDA, a pad on an absent port, an
engine off the request map, an engine on a part class whose manual was
not read, one engine of two, and an engine whose element is not a byte.
Bench: `test_stm32f4_i2c` on the STM32F429I-DISC1, against the STMPE811
touch-screen controller the board carries on I2C3.

## What the silicon does

**The F1 lineage's EVENT MACHINE, and it is a set of software
sequences.** CR1/CR2/OAR1/OAR2/DR/SR1/SR2/CCR/TRISE/FLTR, one byte of
buffer, and a tenure that advances only when software runs the sequence
each flag prescribes - SCL held low meanwhile (27.3.2 and 27.3.3). EV5:
SB up, read SR1 and write the address into DR. EV6: ADDR up, read SR1
then SR2. EV8: TxE, write DR. EV8_2: TxE and BTF together, the last
byte is out, STOP or repeated START. EV7: RxNE, read DR. EV4: STOPF,
read SR1 then write CR1. Nothing here is a clear register: the flag and
the act that consumes it are the same instruction, which is why the
driver's ISR body is written phase by phase and not flag by flag.

**THE RECEIVE PROCEDURE DEPENDS ON THE COUNT** (27.3.3's "controller
receiver", and the three procedures it spells out for when the software
cannot keep up):
- ONE byte: ACK cleared BEFORE ADDR is cleared, STOP set right after;
  the byte lands on RxNE.
- TWO bytes: POS set with ACK before the address goes out, ACK cleared
  after ADDR; both bytes come off ONE BTF, and the STOP goes before
  they are read.
- N > 2: RxNE until three remain, then BTF twice - the first clears ACK
  and reads N-2, the second sets STOP and reads N-1 and N.
That is the whole reason the buffer interrupt (ITBUFEN) is switched on
and off DURING a tenure: TxE and RxNE reach the vector only while the
byte pump wants them, and BTF - under ITEVTEN alone - carries the tail.

**Up to three instances, TWO vectors each, all on APB1.** I2C1 and I2C2
on every part of the family, I2C3 on every one but the F410. Each has
an event vector and an error vector, shared with nothing. FREQ, CCR and
TRISE all divide THAT bus's clock, so a program asks
`apb_hz(clock, false)` and never SYSCLK - 45 MHz against 180 at this
board's core ([clock.md](clock.md)).

**The rate is three registers and a rise time.** CR2.FREQ states the
APB1 clock in whole megahertz and is what the block builds its data
setup and hold times from; below 2 MHz nothing is legal and below 4 MHz
fast mode is not (27.3.2), and above 50 MHz the field has no encoding.
CCR is the period: `2 x CCR` APB periods in standard mode, `3 x CCR` in
fast mode at DUTY 2 and `25 x CCR` at DUTY 16/9. TRISE is the maximum
SCL rise time in whole APB periods PLUS ONE, and it exists because the
block checks the SCL input at the end of that window to see whether a
target is stretching: get it wrong and the frequency moves with the
wire's rise time instead of staying put. All three are
enable-protected. 400 kHz comes out EXACT only where the APB clock is a
multiple of 1.2 MHz at DUTY 2, or of 10 MHz at DUTY 16/9 - which is
what 27.6.8's note about "a multiple of 10 MHz" is really saying.

**BUSY IS NOT A STORED BIT, AND IT WATCHES THE PERIPHERAL'S INPUT.**
27.6.7: set "on detection of SDA or SCL low", cleared "on detection of
a Stop condition", and "still updated when the interface is disabled".
The level it watches is the peripheral's own line input, which reads
LOW while the pad is not in its alternate function - so a block
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
presence FACT is the reserve's. Table
122's ceiling on the digital filter per APB band and mode is published
as arithmetic and is ADVICE: it is what keeps the data hold time inside
the bus specification with the analog filter off, and 27.3.5 itself
allows more to a system that can take the violation.

**Errors, and what a controller must do about each** (27.3.4). A NACK
(AF) leaves the STOP to software. Lost arbitration (ARLO) has already
released the lines and dropped the block back to target mode. A bus
error (BERR) in CONTROLLER mode "does not release the lines and the
state of the current transmission is not affected" - it is not a reason
to abort anything. Overrun and underrun (OVR) are a target's with the
clock stretch given up. All seven error bits are rc_w0: a zero written
into the bit.

**Three errata shape the code, and three more are the application's.**
- *Spurious bus error detection in controller mode* (ES0206 2.10.1).
  BERR can be raised where there was none. Together with the manual's
  own statement above, that makes a controller's BERR a thing to CLEAR
  and COUNT and nothing else - `spurious_bus_errors()` is the counter -
  and it is where this family's engine differs from the CH32V00x's,
  whose chapter has no such erratum.
- *Start cannot be generated after a misplaced Stop* (2.10.3), whose
  workaround is SWRST - which is what `recover()` is, so a tenure that
  never answers and is timed out by the arbiter comes back through the
  errata's own escape.
- *Mismatch on the setup time for a repeated Start condition* (2.10.4):
  in standard mode between 88 and 100 kHz the setup time can be
  violated, independently of the APB clock, when the target stretches
  the clock or SCL rises in more than 300 ns. A write-then-read Request
  IS a repeated start, so the fact is published beside the bus
  (`repeated_start_setup_at_risk`) rather than hidden; the workaround -
  fast mode, or a standard-mode rate below 88 kHz - is the
  application's to take.
- *SMBus 2.0 is not fully supported* (2.10.2): the block cannot NACK an
  invalid byte or command, so an SMBus host wants the alert pin, the
  alert response address or the host-notify protocol above it.
- *Data valid time violated without OVR being set* (2.10.5): a TARGET
  TRANSMITTER with NOSTRETCH set can send a byte late and wrong with no
  flag raised. The client here keeps the clock stretch by default.
- *Both rise times violated above ((VDD + 0.3) / 0.7) V* (2.10.6): a
  5 V I2C bus on a 3.3 V part is out of specification - a board fact.

**The DMA requests are a pair per instance, all on DMA1**, and CR2.LAST
is what makes them usable for a read: with it set the block NACKs the
byte after the controller's EOT_1 signal, so a receive run of two or
more ends by itself and the STOP goes in the stream's completion
handler (27.3.8). A ONE-BYTE read has no such expression - its NACK
must be programmed during EV6, before ADDR is cleared - so it stays on
the byte pump. ITBUFEN must be off while a DMA request serves the same
direction.

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
`event_interrupt`, `buffer_interrupt`, `error_interrupt`.

**The arithmetic.** `I2cSpeed` is `standard_100k` or `fast_400k` - this
block has no Fm+. `I2cDuty` is `ratio_2` or `ratio_16_9`.
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
tables put there. Both lines go out OPEN DRAIN at `very_high`; the
pull-ups are external.

**The host task, `I2cHost<n, pins, TxEngine, RxEngine>`.** `Request` is
the other strata's, field for field: `{addr, tx, tx_len, rx, rx_len,
reply, speed}`, one bus tenure that is a write, a read, a write-then-
read with a repeated START, or the empty probe. `init(clock, config)`
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
`start` is legal; `release` gives the pads back, and `claim_smba_pad`
hands over the alert pad a program in SMBus mode wants.

**The client task, `I2cClient<n, pins>`.** `init(clock, addresses,
no_stretch)`, then the polled surface `addressed`, `answer_address`
(which returns the direction), `second_address_matched`,
`general_call_matched`, `data_ready` / `take`, `data_wanted` / `give`,
`acknowledge`, `stop_seen` / `clear_stop`, `host_nacked` /
`clear_nack`, `overrun` / `clear_overrun`, `flags`, `host_reads`; and
the two ISR bodies `service` and `error_service`, each answering one
`I2cClientEvent`.

## How to use it

A bus with one device, driven from the loop - the shape a bring-up
starts from:

```cpp
constexpr brio::I2cPins pins{.scl = {'A', 8, brio::PinFunction::af4},
                             .sda = {'C', 9, brio::PinFunction::af4}};
using Bus = brio::I2cHost<3, pins>;
Bus::init(clock);                          // both speed rows solved for PCLK1
extern "C" void I2C3_EV_IRQHandler() { if (Bus::isr()) { done = true; } }
extern "C" void I2C3_ER_IRQHandler() { if (Bus::error_isr()) { done = true; } }

const uint8_t index[1] = {0x00};
uint8_t id[2] = {};
Bus::Request r{};
r.addr = 0x41;                             // the 7-bit address, unshifted
r.tx = brio::lend<brio::Lease::reply>(index);
r.tx_len = 1;
r.rx = brio::lend<brio::Lease::reply>(id);
r.rx_len = 2;
done = false;
Bus::start(r);                             // false: the completion is the ISR's
```

The same bus under the kernel's arbiter, which is what a shared bus
wants - the reply comes back as an event and the buffers are lent until
it does:

```cpp
using Arb = brio::I2cBus<Bus, P, 4>;
post<Arb>(Bus::Request{.addr = 0x41,
                       .tx = lend<Lease::reply>(index), .tx_len = 1,
                       .rx = lend<Lease::reply>(id), .rx_len = 2,
                       .reply = reply_to<Client, brio::I2cDone>(),
                       .speed = brio::I2cSpeed::fast_400k});
// the vector posts the completion:
//   if (Bus::isr()) { post<Arb>(TransferDone{Bus::status()}); }
```

An address scan is the empty Request, and its answer is the status:

```cpp
brio::I2cHost<3, pins>::Request probe{};
probe.addr = a;                            // both spans empty
// i2c_ok: somebody answered.  i2c_nack_addr: nobody home.
```

A faster bus, a shaped duty and a wire whose rise times are known -
what an `I2cHostConfig` is for, and the speed travels per request so
one bus can serve a 100 kHz sensor and a 400 kHz converter:

```cpp
Bus::init(clock, {.duty = brio::I2cDuty::ratio_16_9,
                  .standard_rise_ns = 400, .fast_rise_ns = 120,
                  .filter = {.analog = true, .digital = 1}});
r.speed = brio::I2cSpeed::fast_400k;
```

With the DMA engines, on the cells the request mapping gives the
instance - a write phase of any length and a read phase of two bytes or
more move without the CPU:

```cpp
using Bus = brio::I2cHost<3, pins, brio::DmaTxEngine<1, 4, 3>,
                                   brio::DmaRxEngine<1, 2, 3>>;
brio::Dma<1>::init();
Bus::init(clock);
extern "C" void DMA1_Stream4_IRQHandler() { if (Bus::dma_isr()) { done = true; } }
extern "C" void DMA1_Stream2_IRQHandler() { if (Bus::dma_isr()) { done = true; } }
```

The recovery ladder, which is two verbs and a decision:

```cpp
Bus::recover();      // the PERIPHERAL: SWRST, the timing again, PE back
Bus::unstick();      // the WIRE: nine clocks and a STOP by hand;
                     // 0 = it was free, 0xFF = SDA is still low
```

## Bench findings

`test_stm32f4_i2c` on the STM32F429I-DISC1, I2C3 on PA8/PC9 against the
board's STMPE811 touch controller. `z` is 63 verdicts.

- **The chapter's reset values are exact** - every register zero but
  TRISE, which is 0x0002 - and the peripheral clock is off at reset.
- **BUSY stands at reset over an idle wire.** Both pads read HIGH as
  plain inputs and SR2 reads BUSY, because the peripheral's own line
  inputs are low while the pads are not in their alternate function.
  Once `init()` has handed the pads over and pulsed SWRST behind them,
  BUSY reads the wire and is clear. This is the trap of the chapter for
  a bring-up: a block configured before its pads never gets its first
  START out, and no error is raised anywhere.
- **The timing rows at 45 MHz of APB1.** Standard mode is EXACT: CCR
  225, TRISE 46 (1000 ns in whole APB periods plus one), 100000 Hz.
  Fast mode is not: 45 / 1.2 is 37.5, so CCR is 38 and the rate is
  394736 Hz - rounded UP in CCR, which is DOWN in frequency, a bus
  never running faster than asked. The 16/9 duty at CCR 5 gives
  360000 Hz. FREQ reads 45 on every row.
- **SCL on the pad, counted while a 32-byte DMA-carried read runs.**
  The tenure is 35 bytes and 315 clocks, and the pad gives 315 rising
  edges (316 when the polling loop is still watching as the STOP's own
  rise comes). The shortest gap between two edges and the mean over the
  whole tenure BRACKET the bit rate - the mean counts the two address
  phases, whose software sequences stretch SCL low, and the shortest is
  sampled by a polling loop that timestamps a little before the edge it
  sees, in core cycles at 180 MHz:

  | row | CCR | states | shortest gap | mean gap |
  |---|---|---|---|---|
  | Sm 100k | 225 | 100000 Hz | 1732 = 103926 Hz | 1809 = 99502 Hz |
  | Fm 400k duty 2 | 38 | 394736 Hz | 415 = 433734 Hz | 461 = 390455 Hz |
  | Fm 400k 16/9 | 5 | 360000 Hz | 430 = 418604 Hz | 506 = 355731 Hz |

  What the arithmetic states sits inside each bracket. The MEAN is
  stable run to run (461 core cycles both times in the fast row); the
  shortest gap moves with the sampling phase (366 and 415 across two
  runs of the same image), which is why the verdict is on the ORDER of
  the three rows and not on any one number.
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
- **The DMA engines carry both phases.** An eight-byte write-then-read
  through DMA1 streams 4 and 2 on channel 3 is byte-exact against the
  same read on the pump; a one-byte read under an engined host falls
  back to the pump and reads the same byte; a two-byte write through
  the transmit engine reaches the device.
- **SWRST puts the WHOLE register file back**, the configuration
  included (CCR 0x00E1 -> 0, OAR1 -> 0, PE down), which is why
  `recover()` rewrites the timing behind it; the bus works immediately
  after. `unstick()` on a healthy wire returns 0 and drives nothing,
  and the pads come back to the peripheral.
- **No bus error was seen** in the whole suite, spurious or otherwise.

## Not covered yet

Driver gaps, each with its reason:
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
  device is on this desk. The block is not fully SMBus 2.0 compliant
  anyway (ES0206 2.10.2).
- The general call as a HOST verb (a broadcast write to address 0):
  the resource enables the target side's recognition, and no portable
  program needs the other half yet.
- FMPI2C1 - the Fast-mode Plus I2C of the F410, F412, F413/F423 and
  F446 - is ANOTHER BLOCK, the STM32G0's register file (one TIMINGR
  word, ISR/ICR, a byte counter, autoend) under another name. It has
  its own chapter (RM0390 ch. 23) and will have its own driver; the
  reserve publishes only what the header knows of it.
- A wake from Stop on an address match: this family's I2C has no such
  wake-up (the FMPI2C does), so there is nothing to build.
- The recovery LADDER - when to unstick, when to retry, when to take a
  bus out of service - is the application's or a future policy type's,
  as on every other stratum.

Implemented, not bench-verified (each with what would measure it):
- `I2cClient` is compiled on every header and driven nowhere: a client
  needs a controller at the other end of a wire, and the boards of this
  stratum have none between them. A peer board, or a wire between two
  instances of one part.
- The instances other than I2C3 (I2C1 and I2C2, compiled everywhere and
  driven nowhere): a device on one of them, or a wire between two.
- A NACK on a DATA byte (`i2c_nack_data`): the device on this bus
  acknowledges every byte it is sent. A device that refuses one - an
  EEPROM mid-write, a client that runs out of buffer - would measure
  it.
- Arbitration lost (`i2c_arb_lost`): a second controller on the wire.
- A bus error (`i2c_bus_error`) and the spurious-BERR count: the
  chapter raises it on a misplaced START or STOP, which wants another
  controller misbehaving, and the errata's own spurious one did not
  appear in this suite.
- `unstick()` against a client that really holds SDA down, and the
  0xFF it answers when nine clocks and a STOP do not free the line: the
  two pads are the device's, so nothing here can hold SDA low without a
  wire to a spare pad.
- The arbiter's per-bus timeout and `i2c_timeout` over this engine: a
  lost completion has to be staged, which wants a client that stretches
  the clock past the limit.
- The noise filters ON THE WIRE: the register is written and read back
  and the ceiling arithmetic is checked, but what a 4-period digital
  filter does to a glitch wants a glitch generator.
- `rebase()` across a clock change: the fan-out is compiled and not
  exercised; what would measure it is a transaction against the device
  at every rate of a `DynamicClock` pack ([clock.md](clock.md)).
- Clock stretching by a target, as flow control: the STMPE811 does not
  stretch measurably at either speed. A slow client would show it.
- The 88 kHz workaround for ES0206 2.10.4: the suite reports that a
  100 kHz bus sits inside the errata's window, and every repeated start
  it made succeeded - which is not proof the timing is met, only that
  this device tolerates it. An oscilloscope on Tsu;sta would measure
  it.
