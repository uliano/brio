# TWI - the two-wire interface (AVR DA/DB)

> **PROVISIONAL.** The chapter's register description is covered in full
> and both bench halves pass. Single board: the route table and its
> refusals, the three bus speeds measured against the chapter's own baud
> equations, a host talking to a client of the same instance on the same
> pins (combined) and on the route's second pin pair (dual), the whole
> address-match space, the host cases M1..M3 and the client cases
> S1..S3, both Smart modes, Quick Command counted in SCL edges, the bus
> state machine driven by a bit-bang injector, both ISR bodies and a
> clock rebase under traffic. Two boards: clock stretching by a foreign
> client, injected address and data NACKs, multi-host arbitration with
> ARBLOST positively observed on both sides, the collision case S4, bus
> recovery against a really stuck SDA, a General Call answered by two
> chips, and the three speeds against a real device. What is left is
> 10-bit addressing beyond the recognition note, the sleep and debug-run
> paths, SMBus input levels as an electrical fact, the timings of 39.16,
> and multi-host as a driver POLICY - arbitration is measured, a policy
> layer is not designed. The list is in "Not covered yet".

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (TWI
chapter 29, PORTMUX chapter 17, electricals 39.16), errata DS80000915F
(2.15.1, 2.15.2) and, for the DA parts, DS80000882C (2.14.1, 2.14.2,
2.14.3 and clarification 3.2). Three errata items shape the code:

- **DB 2.15.1 / DA 2.14.1, the output pin override**: with the TWI
  enabled the override covers the pin DRIVER but not the pin VALUE, so a
  `PORTx.OUT` bit left at '1' on SDA or SCL holds that line high and no
  transaction can start. Listed for DB rev. A4/A5 (fixed in B0) and for
  every DA revision. Both init paths of the driver clear the OUT bits of
  every pin they route - unconditionally, on every revision: it is one
  store per pin, and the alternative is a bus that never starts.
- **DB 2.15.2 / DA 2.14.3, FLUSH non-functional**: writing MCTRLB.FLUSH
  can leave the host stuck in the Unknown bus state, on every silicon
  revision of both families. The resource therefore does not expose
  FLUSH at all. `recover()` is the errata's own work-around: an ENABLE
  cycle on the host, followed by forcing the bus state back to Idle
  (an ENABLE cycle leaves BUSSTATE Unknown).
- **DA 2.14.2, the swapped SDA hold selections**: on every DA revision
  the 50 ns and 300 ns selections of CTRLA.SDAHOLD are swapped in the
  silicon. `TwiSdaHold` names TRUE nanoseconds and the encoding is
  swapped on the DA, so the same enumerator means the same hold time on
  both families. The families are told apart by `MVIO`, which only the
  DB has.

Driver: `avrdx/twi.hpp`. The arbiter above it:
[i2c-bus.md](../design/i2c-bus.md) (`util/bus_master.hpp`,
`util/i2c_bus.hpp`). Reference test: `test_avr_twi`.

## What the silicon does

One peripheral, two independent halves that may run alone or together:

- **host** - writing MADDR issues the START and shifts the address
  packet out; then one interrupt per byte (WIF after a transmit, RIF
  after a receive), with SCL held low while a flag stands. MCTRLB is a
  strobe register: every command executes the acknowledge action first
  and then repeats the START, transfers a byte, or issues the STOP.
- **client** - the address match logic answers a Start condition; APIF
  says "an address packet or a Stop condition arrived", DIF says "a byte
  moved", and SCTRLB is the client's strobe register. SCL is held low
  while a flag stands, which is clock stretching.

Both halves running at once is a first-class configuration:
**combined** when they share the route's main pin pair, **dual** when
DUALCTRL moves the client to the route's second pair and gives it its
own INPUTLVL, SDAHOLD and FMPEN. In combined mode the CTRLA knobs are
literally one register for both halves.

**The bus state machine** (figure 29-4) runs whenever the host is
enabled, in every sleep mode. It is Unknown after reset or after an
ENABLE cycle, Idle when software forces it there or when a Stop or a
time-out is seen, Busy after a START somebody else generated, and Owner
after one of ours. A host told to start on a Busy bus WAITS: the START
is pending and goes out by itself as soon as the bus becomes Idle.
Writing 0x1 to BUSSTATE is the only value software can impose.

**The four host cases** after the address packet (figure 29-5): M1 write
direction ACKed, M2 read direction ACKed, M3 nobody answered (WIF with
RXACK set), M4 arbitration lost or bus error. **The four client cases**
(figure 29-6): S1 addressed for a write, S2 addressed for a read, S3 a
Stop condition (APIF with AP clear, which needs SCTRLA.PIEN), S4 a
collision - the client could not put a high bit or a NACK on SDA.

**The address-match space** is richer than one address: SADDR[7:1] is
the client's own address and SADDR[0] enables the General Call address
0x00; SADDRMASK is either a MASK (ADDREN = 0: the set bits are the
address bits the comparison ignores, so the client answers a RANGE) or a
SECOND exact address (ADDREN = 1); and PMEN answers every address on the
bus. The matched address is stored in SDATA, which is the only way to
learn which one was called.

**Smart mode** exists on both halves and means the same thing: the
acknowledge action rides a DATA access instead of needing a command.
On the host, reading MDATA sends what ACKACT holds - so a multi-byte
read costs one MCMD strobe (the closing STOP) instead of one per byte;
the action is NOT performed on a DATA write, because the host is
sending. On the client, touching SDATA clears DIF and continues the
operation.

**Quick Command** (QCEN) makes the address packet the whole
transaction: no data byte is exchanged, and RIF or WIF is set according
to the direction bit. Software still issues the STOP.

**Two conditions the chapter attaches to CLK_PER**: the bus error
detector (29.5.6) and the client's Stop interrupt (29.5.10) both need
the main clock to be at least four times f_SCL. The driver reports that
condition (`clock_ok()`); it does not enforce it, because a slower clock
is legal and only blinds those two features.

### The routes

PORTMUX.TWIROUTEA, two bits per instance. Unlike SPI and USART there is
**no pinless route**: every code names a real SDA/SCL pair, so an
enabled instance always costs two pins. Each route also names a DUAL
pair - where the client sits under DUALCTRL - and several routes bond
the main pair without the dual one. TWI0 is on every package; TWI1
arrives with the 32-pin step.

| | 28 pins | 32 pins | 48 pins | 64 pins |
|---|---|---|---|---|
| TWI0 DEFAULT | PA2/PA3, dual PC2/PC3 | PA2/PA3, dual PC2/PC3 | PA2/PA3, dual PC2/PC3 | PA2/PA3, dual PC2/PC3 |
| TWI0 ALT1 | PA2/PA3, no dual | PA2/PA3, no dual | PA2/PA3, dual PC6/PC7 | PA2/PA3, dual PC6/PC7 |
| TWI0 ALT2 | PC2/PC3, no dual | PC2/PC3, no dual | PC2/PC3, dual PC6/PC7 | PC2/PC3, dual PC6/PC7 |
| TWI1 DEFAULT | - | PF2/PF3, no dual | PF2/PF3, dual PB2/PB3 | PF2/PF3, dual PB2/PB3 |
| TWI1 ALT1 | - | PF2/PF3, no dual | PF2/PF3, no dual | PF2/PF3, dual PB6/PB7 |
| TWI1 ALT2 | - | - | PB2/PB3, no dual | PB2/PB3, dual PB6/PB7 |

TWI0's DEFAULT and ALT1 carry the SAME main pair and differ only in
where the dual client lands. A route whose dual pair this package does
not bond stays perfectly usable for the host and for a non-dual client;
only the dual client is refused, at compile time in the task and at run
time in the resource.

### The baud arithmetic

MBAUD sets the SCL high and low times and must be written **with the
host disabled** (29.5.7). The chapter's own procedure (29.3.2.2.1) is
three steps, and the driver performs all three:

1. equation 29-3, `BAUD = f_CLK/(2 f_SCL) - (5 + f_CLK tR / 2)`;
2. equation 29-4, `tLOW = (BAUD + 5)/f_CLK - tOF`;
3. if that tLOW is below the mode's floor (4700 / 1300 / 500 ns),
   equation 29-5, `BAUD = f_CLK (tLOW_mode + tOF) - 5`.

Both tR and tOF belong to the BUS, not to the peripheral, so they are
arguments: the default charges the I2C specification's maxima for the
mode (rise 1000/300/120 ns, fall 300/300/120 ns), which is all a driver
can assume, and a bus that declares its own measured numbers gets the
speed back. They only ever appear as terms that lengthen the period, so
smaller means faster - and step 3 keeps the result legal. Charging
nothing for tOF is the one thing that is NOT safe: the SCL low time the
pins really show is the register's `BAUD + 5` clocks MINUS the fall, so
it would land below the floor by exactly tOF (measured, see below).

Fast-mode Plus is a PAD setting as much as a divider one (29.3.3.1: x10
output drive instead of the slew limit), so `TwiSpeed::fast_plus_1m`
and CTRLA.FMPEN are checked against each other: asking for 1 MHz with
FMPEN off is refused rather than run on slew-limited drivers.

## Types and verbs

### Configuration

| Knob | Values | Default | Effect |
|---|---|---|---|
| `TwiRoute` | `def`, `alt1`, `alt2` | `def` | PORTMUX.TWIROUTEA. The values ARE the header's group values; there is no pinless code |
| `TwiSpeed` | `standard_100k`, `fast_400k`, `fast_plus_1m` | `standard_100k` | the bus speed class; MBAUD is solved from it and Fm+ also demands FMPEN |
| `rise_ns` / `fall_ns` | nanoseconds, 0 = the mode's specification maximum | 0 | the bus's own edges, the only inputs equations 29-3 and 29-5 take beyond the clock |
| `TwiInputLevel` | `i2c`, `smbus` | `i2c` | CTRLA/DUALCTRL.INPUTLVL, the input transition level |
| `TwiSdaSetup` | `cycles4`, `cycles8` | `cycles4` | CTRLA.SDASETUP: the clocks the CLIENT stretches SCL to set up its SDA output |
| `TwiSdaHold` | `off`, `ns50`, `ns300`, `ns500` | `off` | SDAHOLD in TRUE nanoseconds (the DA encoding swap undone) |
| `TwiTimeout` | `disabled`, `us50`, `us100`, `us200` | `disabled` | MCTRLA.TIMEOUT, the SMBus inactive-bus supervisor |
| `quick_command` | bool | false | MCTRLA.QCEN: the address packet IS the transaction |
| `host_smart` / `client_smart` | bool | false | SMEN on either half |
| `address`, `general_call`, `address_mask`, `second_address`, `promiscuous` | - | 0 / false | the client's whole match space (SADDR, SADDRMASK, PMEN) |
| `dual` + `dual_input_level` / `dual_sda_hold` / `dual_fm_plus` | bool + the CTRLA knobs | false | DUALCTRL: the client on the route's second pin pair with its own copies |
| `debug_run` | bool | false | DBGCTRL.DBGRUN |

`twi_config_valid<n>` refuses, at compile time through `init<cfg>()` and
at run time through `init(cfg, hz)`: a route this package does not bond,
Fast-mode Plus without FMPEN, a dual client without a bonded dual pair
or without an enabled client, an address wider than seven bits, and an
instance with neither half enabled.

### The resource, `Twi<n>`

The route table as constants (`sda`, `scl`, `dual_sda`, `dual_scl`,
`has_route`, `has_dual`), then the chapter's registers as verbs:

| Register | Verbs |
|---|---|
| CTRLA | `input_level`, `sda_setup`, `sda_hold` (true nanoseconds) and `sda_hold_code` (the raw field, for a caller that wants to see the DA swap), `fm_plus` - each written with the halves briefly down, as 29.3.2.1 asks |
| DUALCTRL | `dual_mode(on, level, hold, fmp)`, refused where the pair is not bonded |
| DBGCTRL | `debug_run` |
| MCTRLA | `host_enable`, `enable_read_interrupt`, `enable_write_interrupt`, `quick_command`, `timeout`, `host_smart` |
| MCTRLB | `host_command(cmd, ack)` - ACKACT and MCMD in one store, as the register description allows - and `ack_action` alone. **No FLUSH**: `recover()` is the errata's ENABLE cycle plus `force_idle()` |
| (the wire) | `unstick(max_pulses = 9)` - the classic recovery, which `recover()` cannot do: both halves down, then up to nine bit-banged SCL pulses over the route's own pins until a stuck client lets SDA go, then a STOP, then the halves back and the bus Idle. Returns the pulse count, or `unstick_failed` when the line never came back |
| MSTATUS | `host_status` and the flags one by one, `clear_host_flags(mask)` as a plain store, `bus_state`, `force_idle` |
| MBAUD | `set_baud` (which performs the disable/write/enable/force-idle dance the register demands), `baud`, and above it `set_speed`, `bus_timing`, `actual_scl_hz`, `rebase`, `clock_ok` |
| MADDR / MDATA | `address_write`, `address_read`, raw `maddr`, `host_write`, `host_read` |
| SCTRLA | `client_enable`, `enable_data_interrupt`, `enable_address_interrupt`, `enable_stop_interrupt`, `promiscuous`, `client_smart` |
| SCTRLB | `client_command(cmd, ack)`, `client_ack_action` |
| SSTATUS | `client_status` and the flags, `clear_client_flags(mask)`, `host_reading` (DIR), `address_match` (AP) |
| SADDR / SADDRMASK / SDATA | `client_address(addr, general_call)`, `address_mask`, `second_address`, `client_write`, `client_read` |

Two ISR bodies, one per vector: `take_host()` for `TWIn_TWIM_vect` and
`take_client()` for `TWIn_TWIS_vect`. Neither clears anything - the
host's flags clear when the handler writes MADDR, MDATA or MCMD and the
client's when it writes SCMD or touches SDATA, so a body that cleared
them would eat the state its caller has to read. `release()` disables
both halves, turns dual mode off, leaves every routed pin an input with
OUT = 0 (the state the erratum wants of the next owner) and returns
PORTMUX to DEFAULT - which is as far back as a peripheral without a
pinless route can go.

### The tasks

**`TwiHost<n, route>`** is the transfer engine the bus AO drives. One
`Request` is ONE bus tenure - `{addr, tx span, rx span, reply, speed}` -
in the four shapes I2C devices actually use (write, read, write-then-read
with a repeated START, probe); `start()` is always asynchronous and
`isr()` is the per-byte pump that reports through `status()`. It is a
`ClockUser`: `rebase` re-derives MBAUD. A speed CHANGE costs an ENABLE
cycle and a force-idle, because MBAUD may not be written under a running
host - paid at `start()`, only when the speed actually moves, never per
byte. `quick_command(true)` turns every request into an address-only
frame.

**`TwiClient<n, route, on_dual_pins>`** is the other end: the whole
address-match space as options, the four cases as verbs - `respond`
(ACK or NACK the address packet), `receive` (S1), `transmit` (S2),
`complete` (S3, and the way to clear a Stop's APIF), `collision` (S4) -
plus `last_address` for a match that was not exact, a bounded polled
surface (`wait_address`, `wait_data`) and the ISR body. `on_dual_pins`
is refused at compile time where the route's dual pair is not bonded.
As a `ClockUser` it keeps CLK_PER for the `max_scl_hz` / `can_follow`
readback of the four-times condition; nothing is reprogrammed.

Both tasks may own the same instance. Whichever initializes second
writes only its own half and the shared CTRLA knobs, so the order does
not matter - but the knobs are one register and the later init decides
them.

## How to use it

**A host on the default route, through the bus AO** (the usual case):

```cpp
using TwiHw = brio::TwiHost<0>;                 // PA2 SDA / PA3 SCL
using I2c = brio::I2cBus<TwiHw, P>;
ISR(TWI0_TWIM_vect) { if (TwiHw::isr()) brio::post<I2c>(brio::TransferDone{TwiHw::status()}); }
...
TwiHw::init(clock, {.speed = brio::TwiSpeed::fast_400k});
brio::post<I2c>(TwiHw::Request{addr, tx, 3, rx, 2, brio::reply_to<Me, brio::I2cDone>()});
```

**A bus whose edges are known** - declare them and get the speed back:

```cpp
TwiHw::init(clock, {.speed = brio::TwiSpeed::standard_100k,
                    .rise_ns = 200, .fall_ns = 150});   // 1.5k pull-ups, short wires
```

**A client answering one address, polled:**

```cpp
using C = brio::TwiClient<0>;
C::init(clock, {.address = 0x42, .stop_interrupt = true});
const auto s = C::isr();
if (s.address_or_stop()) { s.is_address() ? C::respond() : C::complete(); }
else if (s.data()) { s.host_reading() ? C::transmit(next()) : store(C::receive()); }
```

**A client on the route's second pin pair** (the host keeps the main
pair for its own bus):

```cpp
using C = brio::TwiClient<0, brio::TwiRoute::def, true>;   // PC2/PC3
C::init(clock, {.address = 0x42});
```

**An address range, a second address, or everything:**

```cpp
C::init(clock, {.address = 0x40, .address_mask = 0x03});                    // 0x40..0x43
C::init(clock, {.address = 0x42, .address_mask = 0x55, .second_address = true});
C::init(clock, {.address = 0x42, .promiscuous = true});                     // every address
```

**A wedged host** - the errata's remedy, never FLUSH:

```cpp
if (TwiHw::bus_state() == brio::TwiBusState::unknown) TwiHw::recover();
```

**A wedged WIRE** - a different fault and a different remedy. `recover()`
puts the peripheral back in a known state; it cannot make a client that
is holding SDA low let go. Only clocks can:

```cpp
const uint8_t pulses = TwiHw::unstick();          // <= 9 SCL pulses, then a STOP
if (pulses == brio::Twi<0>::unstick_failed) { /* the line is not clocked free */ }
```

Deciding WHEN to call it - noticing that a transaction is stuck at all -
is a policy for the bus AO and is not built ([i2c-bus.md](../design/i2c-bus.md)).

## Bench findings

Measured on rev. A5 at 5 V, CLK_PER 24 MHz, TWI0 on its DEFAULT route,
one open-drain bus with 1.5k pull-ups. `test_avr_twi`: `z` = the
single-board half, 175 verdicts; `y` = the two-board half, 174 verdicts,
with a second AVR128DB48 on the same node running the campaign's
instrument firmware. The desk gives that bus three taps of the DUT -
PA2/PA3 (the host and the combined client), PC2/PC3 (the dual client,
and a plain-GPIO bit-bang injector whenever Dual mode is off) and
PB2/PB3 - plus the second board's own PA2/PA3, so a host, several
clients, a second host and a foreign agitator all sit on one wire.

**The errata's OUT bits are real work, not ceremony.** With `PORTA.OUT`
bits 2 and 3 deliberately set before `init`, the driver clears them and
the bus runs; the test sets them on purpose so the clearing is observed
rather than assumed.

**The baud equations hold, and tOF is the term nobody may drop.** The
SCL period and the SCL low time were measured through a PORTA pin event
into two TCB capture meters (period between rising edges, and the low
pulse width), taking the minimum over a burst so clock stretching cannot
flatter the result:

| speed | MBAUD | period | SCL | tLOW | mode floor |
|---|---|---|---|---|---|
| Sm 100 kHz | 115 | 244 ticks (register floor 240) | 98.36 kHz | 117 of 120 ticks = 4875 ns | 4700 ns |
| Fm 400 kHz | 34 | 82 ticks (floor 78) | 292.7 kHz | 36 of 39 ticks = 1500 ns | 1300 ns |
| Fm+ 1 MHz | 10 | 34 ticks (floor 30) | 705.9 kHz | 15 of 15 ticks = 625 ns | 500 ns |

The period exceeds the register's floor by exactly the rise time and the
low time falls short of `BAUD + 5` by exactly the fall time, which is
how both are read off the measurement: **tR = 166 ns on this bus at
every speed** (it is the pull-up's job, and FMPEN does not help it), and
**tOF = 125 ns in Standard and Fast mode but 0 in Fast-mode Plus** -
the x10 pad drive of FMPEN collapses the fall below the meter's 42 ns
tick. Charging nothing for tOF, as an earlier revision of this driver
did, put the measured tLOW at 4583 ns and 1208 ns, i.e. BELOW the
specified floor at both Sm and Fm: the equations' tOF is the difference.

**Declaring the bus's real edges buys the speed back.** The same
Standard mode with `rise_ns = 206, fall_ns = 165` gives MBAUD 113 and
measures 100.000 kHz with tLOW 4791 ns - still above the floor.

**A host and a client of the SAME instance talk to each other.** Every
Request shape moves real data both ways with the two halves on one pin
pair (combined) and with the client moved to PC2/PC3 through DUALCTRL
(dual): a five-byte write, a four-byte read, a write-then-read whose
repeated START shows up as two address matches in one tenure, and the
probe. The dual case is the proof that DUALCTRL really moves the client
to the second pair, since only the desk's wire closes that circuit.

**The address-match space behaves exactly as 29.3.2.3.1 describes**,
proven by who ACKs a probe: the exact address, and 0x00 only once
SADDR[0] is set; a mask of 0x03 over 0x40 answers 0x40..0x43 and
refuses 0x44 and 0x3F; ADDREN = 1 turns the same field into a second
exact address (0x42 and 0x55 ACK, 0x43 and 0x54 do not); PMEN answers
everything. The address the client was called on is readable from SDATA
- necessary, since with a mask or PMEN the match alone does not say it.

**The chapter's cases, as MSTATUS values.** M1 reads 0x62 (WIF, ACK
received, clock held, bus Owner); M2 reads 0xA2 (RIF, not WIF - the two
are mutually exclusive); M3 reads 0x72 (WIF with RXACK set). A client
that NACKs the second byte of a five-byte write stops the write there:
the engine reports `i2c_nack_data`, the client kept exactly the two
bytes it acknowledged, and the host's closing NACK on a read is visible
on the client side as RXACK.

**Smart mode costs three MCMD strobes fewer per four-byte read** (one
instead of four) and returns the same four bytes; on a WRITE it changes
nothing, because the acknowledge action is not performed on a DATA
write. On the client, a read of SDATA alone takes DIF down.

**Quick Command is one frame, and every transaction carries one extra
SCL rise.** Counted with a pin event into a TCB edge counter: an
ordinary one-byte write shows 19 rising edges and a two-byte write 28,
i.e. `9N + 1` for N nine-bit frames - the extra one is the STOP
condition itself, which needs SCL back high before SDA is released. A
quick command shows 10: one frame and the STOP, whatever the request's
data spans say. QCEN + W raises WIF and QCEN + R raises RIF.

**The bus state machine, driven by a foreign agitator.** An injected
START moves an Idle bus to Busy and it STAYS Busy with no time-out
configured; an injected STOP puts it back. The inactive-bus time-out
measured **32 us at the 50 us setting, 81 us at 100 us and 183 us at
200 us** - each a little early, none of them late. A START directly
followed by a STOP raises BUSERR on the host half AND on the client
half.

**A host told to start on a Busy bus really does hold, and the START
survives the wait.** Its address had not reached the client after 2 ms;
forcing the bus Idle sent it out at once, and so did the 200 us
inactive-bus time-out.

**Where the bus is left matters.** A STOP injected in the MIDDLE of a
byte is itself a protocol violation: the host raises BUSERR and the
transaction it was holding stays pending until BUSERR is cleared and the
bus declared Idle. The same STOP after a complete nine-bit frame is
clean, and the held transaction goes out with no error at all. A foreign
host that walks away mid-byte therefore costs a bus error to the next
one.

**`recover()` works and FLUSH is never needed**: after the ENABLE cycle
the bus reads Idle, the host is enabled, and the next transaction runs.

**One interrupt per byte, and silence otherwise.** A six-byte write
raises exactly 7 host interrupts (the address and six data bytes) and 8
client interrupts (one address, six data, one Stop with PIEN); five
milliseconds of idle bus with both enables on raise none at all.

**A clock rebase re-derives MBAUD and the wire does not notice.** Across
24 -> 12 -> 24 MHz under traffic MBAUD moved 115 -> 55 -> 115 and the
measured SCL period stayed at 98.36 kHz at every step, with every
exchange exact. The `I2cBus`/`BusMaster` stack rides the engine
unchanged: one arbitrated write-then-read against the instance's own
client replied `i2c_ok` with the right bytes in both directions.

### With a second chip on the wire

**Clock stretching is exactly what the client's flag costs.** A foreign
client that delays clearing DIF by a commanded time per byte lengthens
the whole tenure by that time times the number of DATA bytes - the
address packet is answered from APIF, which this experiment does not
hold. A six-byte write at 100 kHz:

| the client's hold | the tenure | the longest SCL low |
|---|---|---|
| none | 16892..17123 ticks (703..713 us) | 208..248 ticks (8.7..10.3 us) |
| 100 us/byte | 32171 ticks (1340 us) | 2712..2739 ticks (113..114 us) |
| 1 ms/byte | 162092..162143 ticks (6753..6755 us) | 24314..24335 ticks (1013 us) |

The elongation measured 14841..15279 ticks against a model of 14400 and
145140..145260 against 144000, i.e. the commanded hold plus the client's
own software turnaround, five to nine microseconds a byte on a polled
loop. The register's own SCL low time is 120 ticks (5 us), so **even an
unstretched exchange with a polled client runs at the client's speed,
not the divider's**: 208 ticks instead of 120. The host meanwhile
reports CLKHOLD whenever ITS own flag stands and the bus reads Owner
throughout - the two holds are different things and both are visible.
An unstretched exchange immediately after a stretched one is back to
nominal.

**Both NACKs, injected by a board that is alive and listening.** A peer
that parks its address elsewhere produces an ADDRESS NACK: the engine
reports `i2c_nack_addr` and MSTATUS reads 0x72 (WIF, CLKHOLD, RXACK, bus
Owner), while the peer's own report says it matched nothing at all. A
peer told to NACK its third byte turns a six-byte write into
`i2c_nack_data` with the same 0x72, and it kept exactly the three bytes
it acknowledged. From the client side the host's closing NACK on a read
is SSTATUS 0xB3 sampled AT that instant (DIF, CLKHOLD, RXACK, DIR, AP) -
RXACK is a live bit and a copy taken after the transaction carries
whatever the last one left behind.

**Multi-host arbitration, made deterministic by the silicon's own
rule.** A software rendezvous cannot put two STARTs on a bus inside the
hundred nanoseconds a genuine race needs; the hardware can. A host told
to start while the bus is Busy HOLDS its START and releases it by itself
when the bus goes Idle (29.3.2.2.3) - so both boards arm a START into a
bus the bit-bang injector has made Busy, and the injected STOP releases
the two of them on the same edge. What follows is real wired-AND
arbitration and the winner is simply the smaller address byte, which is
what lets the experiment choose the loser:

- the loser reads MSTATUS **0x4B** - WIF with ARBLOST, and BUSSTATE
  Busy: the bus belongs to the winner now. Its transaction did not land
  (the addressed client saw no match at all) and the engine reports
  `i2c_arb_lost`;
- the winner reads **0x62** (WIF, CLKHOLD, bus Owner) and its bytes
  arrive intact at the client it addressed;
- the loser's retry on the idle bus simply works.

Over 40 rounds (four runs of ten, with the gap between the two arming
points swept 60/120/250/500/900 us) the tally was 20 losses each way and
not one exception: **ARBLOST positively observed on both boards**, and
the outcome decided by the addresses rather than by timing.

**The collision case S4 is the wired-AND read.** Two clients at ONE
address both acknowledge the same read and both put a byte on SDA, so
the host reads the AND of the two. The client that tried a high bit
against the other's low cannot drive it: it raises COLL and un-drives,
the other does not. With the DUT serving 0xFF against the peer's 0x55
the host reads 0x55 and the DUT collides; swap the two bytes and the
peer collides while the DUT does not - three runs, no exception. COLL
stands after the collision and **clears itself on the next Start
condition**, so a collision that is not read before the next START is
gone.

**A stuck wire needs clocks, not an ENABLE cycle.** With the peer
holding SDA low from PORT, the DUT's host reads MSTATUS 0x03 - BUSSTATE
Busy, because a falling SDA against a high SCL IS a Start condition to
the state machine - and the address it was told to send never goes out,
2 ms of waiting later. `unstick()` then:

| the bus | result |
|---|---|
| healthy and idle | **0 pulses** (SDA was already high), bus Idle after, next exchange fine |
| a client released by the 4th SCL falling edge | **4 pulses**, SDA high again, bus Idle, next exchange fine |
| a line nothing will release | **`unstick_failed`**, SDA still down - the honest answer |

and it leaves both pins inputs with OUT = 0, which is what errata 2.15.1
wants of the next owner.

**A General Call is answered by every client that enabled it, at the
same time.** One write to 0x00 reached both boards' clients in the same
tenure, each reporting the match as address 0x00. With the peer's
SADDR[0] cleared the same call misses it entirely while its exact
address still answers - proven inside ONE action, so there is no doubt
the client was alive throughout. And a **Quick Command against a real
second chip** is the same 10 SCL rising edges as against the instance's
own client, with `i2c_ok`; to an address nobody holds it is
`i2c_nack_addr`.

**Board B costs the bus nothing electrically, and everything in
latency.** With the peer's client enabled on the same node the minimum
SCL period is 244 / 82 / 34 ticks at Sm / Fm / Fm+ - the same numbers
the bus measures with the DUT alone, so the extra tap adds no rise time
this meter can see. What a two-board exchange really pays is the peer's
polled turnaround: on a read tenure the longest SCL low measured 1683 /
3021 / 3413 ticks (70 / 126 / 142 us) at the three speeds. **The data is
exact at all three speeds anyway** - which is the protocol working as
designed: stretching costs time, never correctness. (During those
stretched reads the low-width meter also caught pulses as short as 5
ticks, 208 ns, which the period floor rules out as a clock period; the
mechanism was not isolated and no verdict rests on it.)

**A clock rebase does not notice the second board either.** Across
24 -> 12 -> 24 MHz with the command channel and the peer's client live,
MBAUD moved 115 -> 55 -> 115, `actual_scl_hz` stayed at 100 kHz, and
every exchange across the wire was exact.

**A desk fact, not a silicon one**: TWI1's dual pair PB2/PB3 would make
a third tap of this bus - the suite drives PB2 low and prints whether the
SDA node follows, instead of assuming either way.

## Not covered yet

**Driver gaps** (the chapter's own features this driver does not
implement):

- **10-bit addressing.** The client's match logic recognizes the first
  byte when SADDR[7:3] is 0b11110 and the second byte is software's job
  (29.3.3.6); neither the client task nor the host's `Request` has a
  shape for it.
- **A stuck-bus WATCHDOG.** The mechanical remedy is `unstick()` (above,
  and bench-measured below); what is missing is the POLICY - noticing
  that a transaction is stuck at all. That needs a per-request timeout
  in the bus AO ([i2c-bus.md](../design/i2c-bus.md)).
- **Multi-host as a policy.** Arbitration is measured and the engine
  reports `i2c_arb_lost`, but nothing above the engine decides what to
  do with it: a retry policy, a back-off, a bus AO that knows it shares
  the wire. `i2c_arb_lost` reaches the requester and stops there.
- **A client-side AO.** The client task is a polled/ISR surface; the
  kernel-level usage type (an AO that owns an address and answers
  register reads) is not built.

**Implemented but not bench-verified**:

- CTRLA.SDASETUP: the knob that shapes the client's own setup-time
  stretch is exposed and written, but its two settings are not measured
  apart on the wire;
- SMBus as an ELECTRICAL fact: CTRLA/DUALCTRL.INPUTLVL is exposed and
  written, but the input transition level itself is not measured;
- DBGCTRL.DBGRUN, and the sleep behaviour of 29.3.5 (the address match
  logic runs in every sleep mode and stretches the clock during
  wake-up);
- the timings of electricals 39.16 as numbers;
- TWI1, and every route other than TWI0 DEFAULT/ALT1/ALT2, are compiled
  for all eight packages and refused where they must be, but only TWI0's
  DEFAULT pair has carried traffic on this desk.
