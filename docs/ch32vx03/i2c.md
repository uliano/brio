# I2C (CH32V203, CH32V303)

Up to two two-wire ports, host and target, 7- and 10-bit addresses with
a dual address and the general call, two speeds with fast mode's two
duty shapes, clock stretching, PEC and the SMBus bits, and two DMA
requests each - the STM32F1's I2C under WCH's register names, an EVENT
MACHINE over two vectors per instance. Documents of record: the
CH32F/V20x_V30x_V31x reference manual V2.3 (19.1 for what the block
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
suite: `test_vx03_i2c`, which talks to a peer board running `twi_peer`
on the CH32V203 and, on the CH32V303 evaluation board, makes the chip's
two controllers talk to each other.

## What the silicon does

### The bus clock has a ceiling, and it is this chapter's

CTLR2.FREQ states PB1 in whole megahertz and 19.12.2 confines the field
to 000100b..111100b, 4 to 60 MHz. The number has to FIT, so a PB1 above
60 MHz has no legal timing at all: `i2c_timing_for()` answers nothing
there and `init()` returns false. This stratum's PB1 ceiling is 72 MHz
(the clock chapter), so I2C is the one peripheral that cannot be used at
the top of the tree - a program that wants a bus keeps HCLK at or below
120 MHz. The rise-time register says the same from the other side:
standard mode's TRISE is (1000 ns x Fpclk) + 1 and its six bits overflow
just past 60 MHz.

### Three registers make the SCL timing, not two

Unlike the CH32V00x's block this one HAS a rise-time register (RTR,
19.12.9): six bits, reset value 2, holding the longest rise the block
must plan for as (rise / Tpclk) + 1. So the timing is CTLR2.FREQ,
CKCFGR (CCR under F/S and DUTY) and RTR together, and
`i2c_timing_for()` solves all three. It takes the WIRE's rise time in
nanoseconds as an argument, because the wire is not a property of the
silicon: told nothing it uses the mode's own maximum, 1000 ns in
standard mode and 300 ns in fast mode.

The CCR arithmetic is the F1 lineage's: standard mode divides PB1 by
2 x CCR, fast mode by 3 x CCR at DUTY 2 and by 25 x CCR at DUTY 16/9,
rounded UP so a bus never runs faster than asked. At 48 MHz of PB1 that
is CCR 240 for 100 kHz, 40 for 400 kHz at DUTY 2, and 5 at DUTY 16/9 -
which is 384 kHz and not 400, because five is what rounding up gives.

### The machine is a set of software sequences

Every one of them stretches SCL low until it completes (19.3's notes):
EVT5 (SB up: read STAR1, write the address), EVT6 (ADDR up: read STAR1
then STAR2), EVT8 (TxE: write DATAR), EVT8_2 (TxE and BTF: the last byte
is out, STOP or repeated START), EVT7 (RxNE: read DATAR). The receive
half DEPENDS ON THE COUNT: one byte wants ACK cleared before ADDR is
cleared and STOP set right after; two want POS set with ACK, ACK cleared
after ADDR, and the pair read after one BTF; three or more run on RxNE
until three remain and then wait for BTF twice. All of it lives in the
engine's `isr()`, and the buffer interrupt (ITBUFEN) is switched on and
off inside a tenure because TxE and RxNE are wanted only while the byte
pump needs them.

### The repeated START is requested while the last byte goes out

A write-then-read turns around on a repeated START, and WHEN it is
requested is not free on this silicon. Requested at EVT8_2 - TxE and
BTF, the last byte and its acknowledge done and the host holding SCL low
- a CH32 TARGET loses that last byte: it acknowledges it and never
raises RxNE for it, in silence (measured on the CH32V303VCT6 with its
two controllers on one bus: the last of one, two, three and four
written bytes, at both speeds, whichever instance was the target, while
the same bytes closed by a STOP all arrived). A register write followed
by a read of that register is exactly this shape, and the register
number is what the target would lose. Requested a byte EARLIER - on the
TxE that says the last byte went into the shifter, so the peripheral
generates the START at the end of that byte, which is where WCH's own
interrupt example puts it - every byte arrives. So the engine requests
it there, on the pump and on the DMA path alike (the transmit block's
completion arms the TxE it waits for). A TxE served later than one byte
time still falls back to the BTF order, which a target of another family
takes - the CH32V203C8T6's peer board did - and a CH32 target does not.

### BUSY is the wire, and it can be left standing

19.12.7 defines BUSY as "SDA or SCL has a low level", cleared when a
STOP is detected. Two things follow. A START set while the bus is busy
is HELD BY THE HARDWARE until the bus frees, which is where two
controllers' STARTs meet and the arbitration decides. And a tenure that
ends without the peripheral seeing its own STOP leaves BUSY standing
over an idle wire - 19.12.1's own case for SWRST, "when no stop
condition is detected on the bus but the busy bit is 1". The engine
answers it: when its bounded wait for BUSY runs out AND BOTH LINES READ
HIGH, it takes the chapter's reset, rewrites the timing and the
interrupt enables, and issues the START. The wire is the guard - a bus
another master really holds is never reset out from under it.

### The pads are a column, the alert is not

Table 10-34 gives I2C1 two columns, the default PB6/PB7 and code 1's
PB8/PB9; I2C2 has no remap field at all, one column on PB10/PB11 from
the datasheet's pin table. `I2cPins` carries the code and `init()`
writes it, refused where the part bonds no pad of it. The SMBus alert
pad does NOT move with the column (PB5 for I2C1, PB12 for I2C2,
datasheet section 3) and is the application's to claim: no verb of this
driver drives it.

### The DMA rows, and what they cannot express

Table 11-5 wires I2C1's transmit request to DMA channel 6 and its
receive to 7, I2C2's to 4 and 5. On this family the channel IS the
request, so an engine slot naming any other channel is refused at
compile time. With engines a write phase of any length and a read phase
of two bytes or more run on them, the read under CTLR2.LAST so the
controller NACKs the block's last byte; a ONE-BYTE read stays on the
pump, because its ACK-before-ADDR sequence has no DMA shape. IN SLEEP
THE BUS MATRIX SERVES THE CORE ALONE on this family, so a transport with
engines holds the program awake.

### How many instances a part has

`device::i2c_count`, from the datasheets' tables 2-1 and 2-1-1 and
nothing else: the CH32V203F6 has NO I2C (I2C1 lives on PB6/PB7 and that
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
`I2cBus` (= `BusMaster`) drives, with the other strata's Request
VERBATIM: one tenure is a write, a read, a write-then-read with a
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
interrupt enables at all - a POLLED client wants them down). The polled
surface is `addressed()`, `answer_address()` (which also says whether
the host reads), `data_ready()`/`take()`, `data_wanted()`/`give()`,
`acknowledge()`, `stop_seen()`/`clear_stop()`, `host_nacked()`, and
`flush()`, the PE cycle that throws away a byte the controller never
clocked. `service()` and `error_service()` are the two ISR bodies, each
reporting one `I2cClientEvent`.

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
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() { (void)Dma::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)Dma::dma_isr(); }
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

`test_vx03_i2c` on the CH32V203C8T6 at 96 MHz of HCLK - PB1 = 48 MHz,
which is what the FREQ window allows and what makes both speeds exact -
against a peer board running `twi_peer` on three wires (SCL, SDA, a
shared ground). TIM4's channel 1 is the second instrument: PB6 is I2C1's
clock line and TIM4_CH1 on its default column at once, so SCL is
measured on the pad that carries it, with no scope and no wire.

- **The reset values are table 19-1's, with one exception that is the
  chapter's own**: every register reads zero after the RCC pulse but
  RTR, whose reset value is 2.
- **The arithmetic is exact at 48 MHz.** 100 kHz: FREQ 48, CCR 240,
  TRISE 49, and the pad measures a 9989 ns period - 100 kHz. 400 kHz at
  DUTY 2: CKCFGR 0x8028, TRISE 15, and the pad measures 2489 ns - 401
  kHz. At DUTY 16/9 the register takes CCR 5 and the wire carries 385
  kHz, which is what rounding up asks for.
- **THE ENABLE PROTECTION IS NOT THERE.** 19.12.9 says of RTR that it
  "can only be set when PE is cleared". Measured with PE set: CKCFGR
  written 0x8028 reads 0x8028, FREQ written 24 reads 24, and RTR written
  33 reads 33. All three take a write with the peripheral enabled. The
  driver still writes them with PE clear, because that is the order 19.3
  prescribes and a read-back is not a promise about what the block DOES
  with the value.
- **What the pad shows of the wire.** At 100 kHz the capture reads a
  high time of 4958 ns against the 5000 the register asks for and a low
  of 5031; at 400 kHz, 791 ns of high against 833. The tens of
  nanoseconds missing from each high half are what the pull-up costs on
  the rise, so this bench carries a real resistor pair and not the tens
  of kiloohms of an internal pull, which would take a microsecond and
  cost fast mode its high half entirely. Released from low, SCL is high
  again within 430 ns and SDA within 480 ns, an upper bound that
  includes the pad's own mode switch.
- **What the wire carries is the TARGET's rate and not the register's.**
  The same three rungs measured against the peer, whose target half is
  polled: 87 kHz, 251 kHz and 245 kHz, with the high halves unchanged
  (4958, 791 and 895 ns) and the LOW halves stretched. Clock stretching
  is flow control and it lands where the chapter says it does.
- **An address nobody answers is `i2c_nack_addr`**, for a probe and for
  a write alike, and the bus is idle again 6 us after the engine answers
  - the time the STOP takes to leave the wire, measured by polling BUSY.
- **A tenure into a wire a foreign chip holds down is answered
  `i2c_arb_lost`**, at once and by the silicon: a START into a held SDA
  reads back a level this controller did not drive.
- **`unstick()` is a measurement.** With the peer holding SDA low from
  its own port and letting go after four SCL falling edges, the verb
  reported 3 pulses and TIM4 counted 3 rising edges on the pad. The STOP
  it makes by hand raises STOPF on the instance's own target half, and
  the event vector takes it exactly once.
- **The command channel is two tenures of the engine under test** - a
  write carrying the frame and a read collecting the answer - and it ran
  10 of 10 round trips at 100 kHz. That is also where the target's BTF
  stretch falls: AFTER the ninth pulse, or a twenty-byte read could
  never be closed by the controller's NACK.
- **The tenure shapes hold against a second chip.** A write, a read and
  the combined write-then-read all complete `i2c_ok` byte-exact, and the
  far end counts FOUR address matches for the three tenures: the
  repeated START is real, seen from the other side. Reads of one, two,
  three and four bytes - the chapter's four receive procedures - are
  each byte-exact.
- **The vocabulary is real statuses across two architectures.** A deaf
  target answers `i2c_nack_addr`; a target commanded to refuse the third
  byte answers `i2c_nack_data`. A target stretching every data byte by
  2 ms turns an 8-byte tenure into 16 ms of wall time against 6 ms for
  eight such tenures unstretched, and the tenure still completes
  `i2c_ok`.
- **The DMA engines carry the shapes.** A 16-byte write and a 16-byte
  read on channels 6 and 7, the combined tenure, the two-byte read under
  LAST and the one-byte read that stays on the pump all complete
  `i2c_ok` byte-exact, with no transfer fault on either channel.
- **A DMA-served read can leave BUSY standing over an idle wire.** With
  both lines high and no STOP in sight the next START is held for ever,
  which is 19.12.1's own case; the engine's SWRST takes it out of the
  way and the tenure that follows completes. The count is printed by the
  suite, and SWRST TAKES CTLR2 WITH IT - the interrupt enables have to
  be written again or the next tenure runs with no vector at all.
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
- **Arbitration was observed in both directions.** With a START
  condition made by hand to hold both controllers' STARTs, and released
  by a hand-made STOP so they leave together: aiming at the higher
  address this controller reports `i2c_arb_lost` and the peer completes;
  aiming at the lower one it keeps the bus (its own address goes out and
  nobody answers it) and the peer reports the loss. The wired-AND
  decides, and both ends say so.
- **The target half answers a real controller.** With the host released
  and `I2cClient<1>` at its own address, the peer's controller wrote six
  bytes into it: one address match, six bytes byte-exact, the tenure
  closed on its STOP - and the command channel came back after the role
  swap.
- **The transmit-empty flag of an idle target reads DOWN**, with or
  without a byte in DATAR, so it cannot tell an unclocked byte from an
  idle port; `flush()` belongs where the question is real, after the
  host's closing NACK. The PE cycle it runs SPARES THE CONFIGURATION:
  both own addresses, CKCFGR, RTR and CTLR2 all read back what they
  held, and only ACK has to be put back up.
- **Ten seconds of back-to-back tenures at fast mode**: 25100 tenures,
  401600 bytes moved, not one failure at either end and no byte
  mismatch.

### On the CH32V303VCT6

`test_vx03_i2c` at the same 96 MHz on WCH's evaluation board (28
verdicts in `z`), which wires I2C2's pads to I2C1's - PB10 to PB6, PB11
to PB7 - with a 4.7 kOhm pull-up on each line: the chip's two
controllers share one bus, and there is no peer board. The letters that
want the peer decline by name when they find the bus pulled up by that
link and no peer answering; the rest measure:

- **The block is the CH32V203's**: the same reset values, RTR 2 alone;
  the same three rungs on the pad during a probe - 9979 ns of period at
  100 kHz (high 4947 ns), 2479 ns at 400 kHz and DUTY 2 (403 kHz, high
  781 ns), 2583 ns at DUTY 16/9 (387 kHz); the same missing enable
  protection, CKCFGR, FREQ and RTR each taking a write with PE set; the
  lines back high within 448 to 479 ns of release on the board's
  resistors (two runs, an upper bound as above); an absent address
  `i2c_nack_addr` with BUSY clear 6 us after the answer; both vectors
  carrying a probe; the dual address and the general call in their
  registers, and the PE cycle sparing the configuration.
- **I2C1 the host, I2C2 the target**, the target POLLED from the loop
  that waits for the host (its clock stretch holds the bus while the
  loop comes round): the probe, an absent address, an eight-byte write,
  reads of one, two, three, four and eight bytes - the host's four
  receive procedures - and a write-then-read with the target addressed
  twice, byte-exact at 100 kHz and at 400 kHz in both duty shapes.
- **THE TARGET AS A TRANSMITTER**: every byte those reads took came out
  of I2C2's own data register, and in each of the five reads the target
  was asked for exactly one byte more than the host clocked - the byte
  its shifter wants ahead of the wire - which `flush()` dropped after the
  host's closing NACK, fifteen of fifteen over the three rungs.
- **The second address and the general call** each open a write tenure
  on the target, and the target's status says which matched (DUALF,
  GENCALL).
- **The DMA host on channels 6 and 7** writes sixteen bytes into the
  chip's own target, reads sixteen back and carries a write of three and
  a read of eight in one tenure, byte-exact at 400 kHz, with no BUSY left
  standing.
- **I2C2 ON A WIRE AS THE HOST**, I2C1 its target: the same shapes,
  byte-exact at both speeds and both duties.
- **The last written byte before a repeated START** (above): with the
  START requested after BTF the target took none of one, one of two, two
  of three and three of four written bytes; requested while the last byte
  shifts, every one - the engine's order now, without a dummy byte in the
  data register, with seven to ten event-vector entries for the whole
  tenure.
- **A target stuck mid-byte, with no foreign chip**: a read of zeros cut
  off four bit times into its first byte by taking the host through its
  reset line leaves I2C2 holding SDA low with SCL released high;
  `unstick()` reported 4 pulses and TIM4 counted 4 rising edges on the
  pad, the bus came back free, and the same two controllers read four
  bytes byte-exact right after.

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

- **The write-then-read's START against a target of another family**,
  now that it is requested before BTF: measured against the CH32V303's
  own two controllers; the CH32V203C8T6's peer board running the tenure
  shapes again is what would measure it there.
- **NOSTRETCH, and the overrun it admits.** The option is written and
  read back; measuring it wants a controller that will not wait, which
  the peer's engine is not.
- **`rebase()` under a `DynamicClock`.** The fan-out is written and
  compiled; no suite drives this chapter under a dynamic clock, and one
  would have to stay inside the 4..60 MHz window at every rate.
- **I2C1's second column, PB8/PB9.** The pads are bonded on both
  boards' parts and the remap is refused where they are not; both
  benches' buses are on PB6/PB7 (and the evaluation board's PB8 carries
  a timer wire), so the column is compiled and never clocked.
- **The SMBus alert pad.** PB5 for I2C1: the driver drives the bit, not
  the pad, and an alert line wants a device that pulls it.
