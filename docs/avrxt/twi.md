# TWI - the two-wire interface (AVR DA, DB and EA)

This document is for a programmer who puts an I2C bus on an AVR DA, DB
or EA: a host that talks to sensors, converters and memories, a client
that answers a host on another chip, or both on one instance. `Twi<n>`
is the resource, one instance of the peripheral as a monostate (a class
whose state is static, with no instances): its whole configuration in
one struct and one verb per register fact. Two tasks sit over it (a
task, in brio, is a driver layer with state of its own over a resource,
served from its peripheral's interrupt or called by an active object;
an active object, AO for short, is a component of a brio program with
its own event queue, served one event at a time by the kernel,
[../learn/introduction.md](../learn/introduction.md)). `I2cHost<n,
route>` is the transfer engine: it moves one bus tenure per request
(the time the host holds the bus, from its START to its STOP), one
interrupt per byte, and it is driven by the bus arbiter `I2cBus` of
`util/i2c_bus.hpp`, an active object that queues the requests of every
AO sharing the bus and answers each with an `I2cDone`.
`I2cClient<n, route, on_dual_pins>` is the other end of the wire: the
address-match space as options and the chapter's four client cases as
verbs. [design/i2c-bus.md](../design/i2c-bus.md) is the contract above
the engine.

The smallest use, a host on TWI0's default route through the arbiter,
from "How to use it" below:

```cpp
using I2cHw = brio::I2cHost<0>;                 // PA2 SDA / PA3 SCL
using I2c = brio::I2cBus<I2cHw, P>;
ISR(TWI0_TWIM_vect) { if (I2cHw::isr()) brio::post<I2c>(brio::TransferDone{I2cHw::status()}); }
...
I2cHw::init(clock, {.speed = brio::I2cSpeed::fast_400k});
brio::post<I2c>(I2cHw::Request{addr, brio::lend<brio::Lease::reply>(tx), 3,
                               brio::lend<brio::Lease::reply>(rx), 2,
                               brio::reply_to<Me, brio::I2cDone>(), brio::I2cSpeed::fast_400k});
```

Line by line: `I2cHw` names the engine on TWI0 with its route at the
default, `TwiRoute::def`, which puts SDA on PA2 and SCL on PA3; `I2c`
names the arbiter over that engine, `P` being the platform
(`brio::AvrPlatform`, [platform.md](platform.md)); the `ISR()` line
binds the host vector to `isr()`, which returns true on the interrupt
that ends the tenure, and the handler then posts the engine's
`status()` to the arbiter as a `TransferDone`, which the arbiter
forwards to the requester; `init()` takes the clock task's tag
(`constexpr SysClock clock;`, [clkctrl.md](clkctrl.md)) and the
options, routes the pins, programs MBAUD for the speed, enables the
host with both its interrupts and declares the bus idle, and returns
false when the speed cannot be reached at that clock; the `post` hands
the arbiter one request: the 7-bit address, three bytes to write from
`tx`, two bytes to read into `rx` after a repeated START, the reply
channel to the active object `Me`, and the speed of this tenure. The
reply is an `I2cDone` whose status is `i2c_ok` or the code of what went
wrong (`util/i2c_bus.hpp`).

What decides correctness even there:

- **`init()` after the clock task's `SysClock::init()`.** MBAUD is
  computed from the rate the clock type claims (`clock_hz(clock)`), not
  read from the hardware, so a host initialized while the chip still
  runs at its reset rate drives SCL off by the ratio of the two rates
  until the clock task brings CLK_PER to the claimed rate, and nothing
  refuses it. A `DynamicClock` must list the engine
  among its users, or `init()` fails to compile.
- **The host vector bound by the program.** `init()` enables RIEN and
  WIEN, so the address packet's interrupt arrives at
  `TWI0_TWIM_vect`; a vector left unbound enters avr-libc's
  `__bad_interrupt`, a `jmp 0` (read in the release listing), and that
  first interrupt restarts the program from its reset vector. An
  interrupt-driven client binds `TWIn_TWIS_vect` the same way.
- **The buffers are lent until the reply.** `tx` and `rx` are loans
  (`Lease::reply`): the host interrupt reads `tx` and writes `rx` until
  the `I2cDone` lands, and the requester leaves both alone until then;
  nothing checks it at run time.
- **The speed travels with each request.** A `Request` that names none
  runs at `standard_100k`, whatever `init()` programmed: `start()`
  reprograms MBAUD (an ENABLE cycle) whenever the request's speed is not
  the one in force, which is why the example names it twice.
- **The pull-ups are the board's.** The driver leaves the pins inputs
  with OUT = 0 and the peripheral drives them open drain; it enables no
  internal pull-up and checks none.
- **The arbiter is an active object** and is listed with the others in
  the kernel's list (`brio::Tenuto<P, Me, I2c>::run()`), interrupts
  enabled (`sei()`) before `run()`. With its default `timeout_ticks` of
  0 it never answers a tenure that never completes; a timed `I2cBus`
  answers `i2c_timeout` and recovers the engine ("A wedged tenure" under
  "Bench findings").

The rest of this page, in order: the documents of record with the three
errata items that shape the code; what the silicon does (the two
halves, the bus state machine, the host and client cases, the
address-match space, Smart mode, Quick Command, the two conditions on
CLK_PER, the routes, the baud arithmetic, the AVR EA's SCL clock); the
types and verbs; how to use it; what the silicon offers for the cost and
the host's cost measured; the bench findings, most of the page, with a
second chip on the wire and on the AVR64EA48; what is not covered.

## Documents of record

AVR128DB28/32/48/64 data sheet DS40002247B (TWI
chapter 29, PORTMUX chapter 17, electricals 39.16), errata DS80000915F
(2.15.1, 2.15.2) and, for the DA parts, DS80000882C (2.14.1, 2.14.2,
2.14.3 and clarification 3.2); for the AVR64EA28/32/48 DS40002443A (TWI
chapter 27, PORTMUX 17.3.6, electricals 35.15) and DS80001048D (no TWI
item). Three errata items shape the code - and on the AVR EA the chapter
itself, whose SCL clock is not the DA/DB's (below, "The AVR EA's SCL
clock"):

- **DB 2.15.1 / DA 2.14.1, the output pin override**: with the TWI
  enabled the override covers the pin driver but not the pin value, so a
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
  silicon. `TwiSdaHold` names true nanoseconds and the encoding is
  swapped on the DA, so the same enumerator means the same hold time on
  both families. The swap is keyed on the die
  (`erratum_da_twi_sdahold_swap`, `avrxt/device_tables.hpp`), never on
  a feature that happens to separate the two families.

Driver: `avrxt/twi.hpp`. The arbiter above it:
[i2c-bus.md](../design/i2c-bus.md) (`util/bus_master.hpp`,
`util/i2c_bus.hpp`). Reference test: `test_avr_twi` (a letter is one
command key of the suite's console and the group of verdicts it runs;
`z` runs the single-board set, letters a to j and, on the AVR EA, u and
v, and `y` the set that needs a partner board, letters k to t, against
a second board running `twi_peer`, on this page a SAM C21).

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
after one of ours. A host told to start on a Busy bus waits: the START
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
0x00; SADDRMASK is either a mask (ADDREN = 0: the set bits are the
address bits the comparison ignores, so the client answers a range) or a
second exact address (ADDREN = 1); and PMEN answers every address on the
bus. The matched address is stored in SDATA, which is the only way to
learn which one was called.

**Smart mode** exists on both halves and means the same thing: the
acknowledge action rides a data access instead of needing a command.
On the host, reading MDATA sends what ACKACT holds - so a multi-byte
read costs one MCMD strobe (the closing STOP) instead of one per byte;
the action is not performed on a data write, because the host is
sending. On the client, touching SDATA clears DIF and continues the
operation.

**Quick Command** (QCEN) makes the address packet the whole
transaction: no data byte is exchanged, and RIF or WIF is set according
to the direction bit. Software still issues the STOP.

**Two conditions the chapter attaches to CLK_PER**: the bus error
detector (29.5.6) and the client's Stop interrupt (29.5.10) both need
the main clock to be at least four times f_SCL (`twi_clock_ok`). The
resource's `speed_ok(speed)` asks a stronger question - can the
divider make the speed at the clock and the bus timing in force, a
BAUD the register holds with the period at or above the request - and
that one is enforced: `twi_baud_for` has no solution below BAUD 0 (the
divider cannot go that fast: BAUD = 0 is CLK_PER/10, so any reachable
speed also satisfies the four-times condition) or above 255, the
resource refuses to program such a speed, and the host engine refuses
a request naming it. The answer is cached per speed and recomputed
when the clock or the timing moves, never at a request. The client
side keeps the four-times condition as a readback only (`max_scl_hz`,
`can_follow`): a slower clock is legal there and only blinds those two
features.

### The routes

PORTMUX.TWIROUTEA, two bits per instance. Unlike SPI and USART there is
**no pinless route**: every code names a real SDA/SCL pair, so an
enabled instance always costs two pins. Each route also names a dual
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

The AVR EA (DS40002443A 17.3.6) carries TWI0 alone, on every package,
with a fourth route:

| | 28 pins | 32 pins | 48 pins |
|---|---|---|---|
| TWI0 DEFAULT | PA2/PA3, dual PC2/PC3 | PA2/PA3, dual PC2/PC3 | PA2/PA3, dual PC2/PC3 |
| TWI0 ALT1 | PA2/PA3, no dual | PA2/PA3, no dual | PA2/PA3, dual PC6/PC7 |
| TWI0 ALT2 | PC2/PC3, no dual | PC2/PC3, no dual | PC2/PC3, dual PC6/PC7 |
| TWI0 ALT3 | PA0/PA1, dual PC2/PC3 | PA0/PA1, dual PC2/PC3 | PA0/PA1, dual PC2/PC3 |

`TwiRoute::alt3` exists only on that layout (a name the layout has not
got is a compile error, never a code that writes another route).

TWI0's DEFAULT and ALT1 carry the same main pair and differ only in
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

Both tR and tOF belong to the bus, not to the peripheral, so they are
arguments: the default charges the I2C specification's maxima for the
mode (rise 1000/300/120 ns, fall 300/300/120 ns), which is all a driver
can assume, and a bus that declares its own measured numbers gets the
speed back. They only ever appear as terms that lengthen the period, so
smaller means faster - and step 3 keeps the result legal. Charging
nothing for tOF is the one thing that is not safe: the SCL low time the
pins really show is the register's `BAUD + 5` clocks minus the fall, so
it would land below the floor by exactly tOF (measured, see below).

Fast-mode Plus is a pad setting as much as a divider one (29.3.3.1: x10
output drive instead of the slew limit), so `I2cSpeed::fast_plus_1m`
and CTRLA.FMPEN are checked against each other: asking for 1 MHz with
FMPEN off is refused rather than run on slew-limited drivers.

### The AVR EA's SCL clock

DS40002443A 27.3.2.2.1 keeps the period (equation 1, the DA/DB's 29-2:
`f_SCL = f_CLK / (10 + 2 BAUD + f_CLK tR)`) and changes the duty: the
low phase is `(BAUD + 6)/f_CLK - tOF` in Sm (equation 3.2) and
`(BAUD + 6 + min(SCLDUTY, BAUD))/f_CLK - tOF` in Fm and Fm+ (3.1), with
SCLDUTY defined nowhere, and the new CTRLA.FMEN "adjusts the SCL duty
cycle to comply with I2C specification in Fast mode" for the host
(27.5.1; no effect on a client or under FMPEN). `test_avr_twi` letter u
measured what the silicon does (Bench findings): FMEN moves `min(5,
BAUD)` clocks from the high phase into the low, FMPEN `min(8, BAUD + 2)`
whatever FMEN says, the period stays equation 1's at every MBAUD, and
with FMPEN off MBAUD 0..3 make no regular clock (none at all at 3), with
FMPEN on MBAUD 0 and 1 none. The driver takes it whole:

- `twi_low_ticks(baud, speed)` is the low phase the host drives at a
  speed - BAUD + 6 in Sm, plus `min(5, BAUD)` in Fm, plus `min(8,
  BAUD + 2)` in Fm+ - and `twi_baud_for()` the chapter's procedure with
  it: equation 2 for BAUD, raised to `twi_baud_min()` (4, or 2 under
  FMPEN), then to the smallest BAUD whose low phase covers the mode's
  floor plus the fall charged - equation 4 rewritten for the measured
  duty (the data sheet's "- 3" matches neither its own 3.2 nor the
  silicon). The DA/DB keep equation 29-4's BAUD + 5 at every speed
  (`twi_ea_scl`, device_tables.hpp, keys the die).
- FMEN is used: the engine sets it with the Fm speed (`set_speed()`, and
  `init()` for a host configured at Fm) as it sets FMPEN with Fm+. At
  20 MHz with the specification's edges charged, Fm is MBAUD 21 with it
  - 364 kHz on this desk's bus - and would be MBAUD 26 without, 308 kHz:
  the low phase the floor needs is bought from the high phase instead of
  from the period. `Twi::fm_duty()` is the raw verb, where the bit
  exists.
- 1 MHz needs CLK_PER over 12 MHz (35.15 table 35-21): `twi_baud_for()`
  refuses Fm+ at or below it (`twi_fm_plus_min_clk_hz`). The DA/DB's
  table names 10 MHz and is not enforced ("Not covered yet").

The duty has one consequence the 50/50 clock has not: the floor step's
period at tR = 0 is two CLK_PER under 50/50's, so on a bus whose rise is
under two CLK_PER the SCL runs up to two CLK_PER fast - at the rise the
speed charges it never does (equation 2), and this desk's 150 ns rise
reads 99.5 kHz at 20 MHz and 101.0 kHz at 10 MHz.

## Types and verbs

### Configuration

| Knob | Values | Default | Effect |
|---|---|---|---|
| `TwiRoute` | `def`, `alt1`, `alt2`, and `alt3` on the AVR EA | `def` | PORTMUX.TWIROUTEA. The values ARE the header's group values; there is no pinless code |
| `I2cSpeed` | `standard_100k`, `fast_400k`, `fast_plus_1m` | `standard_100k` | the bus speed class; MBAUD is solved from it and Fm+ also demands FMPEN |
| `rise_ns` / `fall_ns` | nanoseconds, 0 = the mode's specification maximum | 0 | the bus's own edges, the only inputs equations 29-3 and 29-5 take beyond the clock |
| `TwiInputLevel` | `i2c`, `smbus` | `i2c` | CTRLA/DUALCTRL.INPUTLVL, the input transition level |
| `TwiSdaSetup` | `cycles4`, `cycles8` | `cycles8` | CTRLA.SDASETUP: the clocks the CLIENT stretches SCL to set up its SDA output - eight in every default, not the reset value's four (below, "The client's data setup") |
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

The resource is named for the chapter and the tasks for the bus they
speak: this silicon calls the peripheral a TWI, while `I2cHost` and
`I2cClient` are spelled the same on every target brio runs on, so an
application AO reads unchanged when it moves. The pin claim in the
template arguments stays in this silicon's words - `TwiRoute::def` is a
PORTMUX code and nothing else. [The contract the tasks implement](../design/i2c-bus.md).

**`I2cHost<n, route>`** is the transfer engine the bus AO drives. One
`Request` is ONE bus tenure - `{addr, tx span, rx span, reply, speed}` -
in the four shapes I2C devices actually use (write, read, write-then-read
with a repeated START, probe); `start()` is asynchronous whenever the
wire moves and `isr()` is the per-byte pump that reports through
`status()`. The one synchronous completion is the refusal: a request
naming a speed the divider cannot make at the clock in force
(`speed_ok(speed)` is false), or an address outside 0x08..0x77 - the
general call and the 10-bit prefixes among them, which every host
refuses ([../design/i2c-bus.md](../design/i2c-bus.md)) - is answered
`i2c_rejected` inside `start()`, no byte moved, and the arbiter delivers that status like any
outcome of the wire - the contract every stratum's engine follows. It
is a `ClockUser`: `rebase` re-derives MBAUD, and when the speed in force
no longer fits the new clock MBAUD stays as it was and every request at
that speed is refused until the clock moves again. A speed CHANGE costs
an ENABLE cycle and a force-idle, because MBAUD may not be written under
a running host - paid at `start()`, only when the speed actually moves,
never per byte. `quick_command(true)` turns every request into an
address-only frame. Smart mode is the engine's default (`Options::smart`,
the measured reason below), and `i2c_nack_addr` answers any address
nobody acknowledged - a write's, a read's, the repeated START's -
`i2c_nack_data` a written byte refused.

**`I2cClient<n, route, on_dual_pins>`** is the other end: the whole
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
using I2cHw = brio::I2cHost<0>;                 // PA2 SDA / PA3 SCL
using I2c = brio::I2cBus<I2cHw, P>;
ISR(TWI0_TWIM_vect) { if (I2cHw::isr()) brio::post<I2c>(brio::TransferDone{I2cHw::status()}); }
...
I2cHw::init(clock, {.speed = brio::I2cSpeed::fast_400k});
brio::post<I2c>(I2cHw::Request{addr, brio::lend<brio::Lease::reply>(tx), 3,
                               brio::lend<brio::Lease::reply>(rx), 2,
                               brio::reply_to<Me, brio::I2cDone>(), brio::I2cSpeed::fast_400k});
```

**A bus whose edges are known** - declare them and get the speed back:

```cpp
I2cHw::init(clock, {.speed = brio::I2cSpeed::standard_100k,
                    .rise_ns = 200, .fall_ns = 150});   // 1.5k pull-ups, short wires
```

**A client answering one address, polled:**

```cpp
using C = brio::I2cClient<0>;
C::init(clock, {.address = 0x42, .stop_interrupt = true});
const auto s = C::isr();
if (s.address_or_stop()) { s.is_address() ? C::respond() : C::complete(); }
else if (s.data()) { s.host_reading() ? C::transmit(next()) : store(C::receive()); }
```

**A client on the route's second pin pair** (the host keeps the main
pair for its own bus):

```cpp
using C = brio::I2cClient<0, brio::TwiRoute::def, true>;   // PC2/PC3
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
if (I2cHw::bus_state() == brio::TwiBusState::unknown) I2cHw::recover();
```

**A wedged WIRE** - a different fault and a different remedy. `recover()`
puts the peripheral back in a known state; it cannot make a client that
is holding SDA low let go. Only clocks can:

```cpp
const uint8_t pulses = I2cHw::unstick();          // <= 9 SCL pulses, then a STOP
if (pulses == brio::Twi<0>::unstick_failed) { /* the line is not clocked free */ }
```

Noticing that a transaction is stuck at all is the timed arbiter's: an
`I2cBus` with a non-zero `timeout_ticks` answers the requester
`i2c_timeout` and calls `recover()` ("A wedged tenure" under "Bench
findings"). Deciding when to spend `unstick()` on the wire is a policy
for the application and is not built ([i2c-bus.md](../design/i2c-bus.md)).

## What the silicon offers for the cost, and what the engine takes

The chapter's whole offer that bears on what a tenure costs the core
and the bus, each item used or declined:

| item | where | the engine |
|---|---|---|
| one host interrupt per byte, WIF after a write, RIF after a read, SCL held while the flag stands | 29.3.2.2.4, 29.3.2.2.5, 29.3.4 | THE SHAPE: there is no FIFO, no byte counter, no automatic STOP and no DMA request on this TWI, so the byte is software's and the handler's latency to its MDATA access is time the BUS waits, every byte. The handler is written for that latency: the two common cases asked first with one masked compare each, a cursor and a count set up once per phase, every verb it touches inline so the vector carries no call |
| Smart mode, host side (MCTRLA.SMEN) | 29.3.3.3, 29.5.9 | USED, the default: ACKACT = ACK is armed once per reading tenure and every byte but the last is answered by the MDATA read itself; the last sets ACKACT = NACK before the read (with ACK standing the read would acknowledge and clock one more byte in; with NACK it sends nothing) and the STOP command carries it. Nothing on a write: the action is not taken on a DATA write |
| MCTRLB with ACKACT and MCMD in one store | 29.5.5, table 29-2 note 1 | USED: every command is one store |
| the held START on a Busy bus | 29.3.2.2.3 | USED by construction: `start()` writes MADDR and never waits for the bus |
| Quick Command (QCEN) | 29.3.3.5 | EXPOSED (`quick_command(true)`), not the probe's shape: the empty probe costs one interrupt either way |
| FLUSH | 29.5.5, errata 2.15.2 / 2.14.3 | DECLINED: broken on every DA/DB revision; `recover()` is the errata's ENABLE cycle. The AVR EA has no such item, and 1000 strobes on an idle host never left it stuck (letter v); still unexposed - `recover()` is the verb on every die, and a FLUSH in the middle of a tenure is unmeasured |
| the client's Smart mode | 29.3.3.3 | the client task's option; the bench's own client runs it |
| CTRLA.SDASETUP, the client's own setup stretch | 29.5.1, DS40002443A 27.5.1 | USED at EIGHT clocks, every default of the driver: four are shorter than the I2C specification's Standard-mode data setup (250 ns) above 16 MHz, and on the AVR EA's wire at 20 MHz they lose a Smart-mode client in a long read (On the AVR64EA48, "The client's data setup"); the price is four clocks of stretch on a byte whose clock the client releases |
| CTRLA.FMEN, the AVR EA's Fm duty | DS40002443A 27.5.1 | USED with the Fm speed: the floor's low phase bought from the high phase (The AVR EA's SCL clock) |
| the inactive-bus time-out's base, CLKCTRL.MCLKTIMEBASE | DS40002443A 27.3.3.1 | USED: the clock writes TIMEBASE at every init and rate change, so `TwiTimeout` is microseconds; at TIMEBASE 0, its reset value, every setting fires at once (letter v) |

### The host's cost, measured (the benchmark skeleton's letter i)

The benchmark skeleton is the app that measures the drivers' costs
([../design/benchmark.md](../design/benchmark.md)). The bench: TWI0's
host on PA2/PA3 against the SAME instance's client on the dual pair
PC2/PC3, served from its own vector in Smart mode at SDASETUP four clocks (the dual loop of the findings below), 1.5k to 5 V,
CLK_PER 24 MHz. Both handlers run on one
core, so every byte waits TWO handler latencies - the client's before
the acknowledge, the host's after it - and wall minus wire holds both.
The SCL period is measured on PA3 (TCB0 between rising edges): 244, 82
and 34 cycles at 100 kHz, 400 kHz and Fm+ with the default timing
(98.4 kHz, 292.7 kHz, 705.9 kHz; the node's rise is 166 ns, over Fm+'s
120 ns specification, so the divider's own Fm+ figure is the one run).
`wire` is the tenure's SCL rising edges - nine a frame, one for a
repeated START, one for the STOP - times that period; the engine's
completion edge is the STOP's command, a period before the STOP's edge.
Plain bindings (no stamps), the thread spinning on the edge, best of 8,
in CLK_PER cycles; the vendor's column is the data sheet's own polled
sequence (29.3.2.2.3 - 29.3.2.2.5) against the same client:

| tenure | wire | before | after | the data sheet's loop |
|---|---|---|---|---|
| 1-byte write, 100 kHz | 4636 | 5437 | 5392 | 5150 |
| 255-byte write, 100 kHz | 562420 | 587198 (x 1.04) | 579784 (x 1.03) | 576405 (x 1.02) |
| 1-byte write, 400 kHz | 1558 | 2157 | 2067 | 1874 |
| 1 + 1 register read, 400 kHz | 3116 | 4228 | 4125 | 3924 |
| 255-byte write, 400 kHz | 189010 | 220307 (x 1.16) | 212733 (x 1.12) | 210715 (x 1.11) |
| 255-byte read, 400 kHz | 189010 | 223278 (x 1.18) | 216381 (x 1.14) | 215202 (x 1.13) |
| 255-byte write, Fm+ | 78370 | 114193 (x 1.45) | 106403 (x 1.35) | 104344 (x 1.33) |
| 255-byte read, Fm+ | 78370 | 116684 (x 1.48) | 109601 (x 1.39) | 109198 (x 1.39) |

(The reads after are the engine's default, Smart mode; before, the
engine without it.)

What the rows say:

- A written byte costs the bus 29 to 30 cycles less than it did, at
  every speed (the 255-byte writes, divided by their 256 frames): the
  handler's path from the flag to the MDATA store is 31 cycles of body
  after a 12-instruction prologue where it was about 50 after 15, and
  its whole entry 76 cycles where it was about 105 (counted in the
  release listing). The handler carried no call in the benchmark
  skeleton's image either way; in `test_avr_twi` it called `Twi::address_read` and
  `Twi::ack_action`, which that image had outlined for their several
  call sites, and saved the whole call-clobbered set - every verb the
  handler touches is always_inline now, so no image can.
- Smart mode moves the bus on at the MDATA read instead of at the
  command after it: 9 to 14 cycles less SCL held per received byte at
  the three speeds (219862 against 216381 at 400 kHz, 112155 against
  109601 at Fm+), the same bytes. It is the engine's default for that.
- Against the data sheet's polled loop the interrupt engine is within 1
  to 2 per cent on 255 bytes at every speed - the loop busy for the
  whole tenure, the engine's thread free between the bytes - and 170 to
  240 cycles behind on one-byte tenures: the request's fields, the START
  through `start()` and the completion edge, where the loop has none.
- What remains above the wire on a long tenure is the two handlers per
  frame on one core - 85 cycles a frame in the polled loop, where the
  host spins and only the client's handler is left, 92 under the engine
  at 400 kHz. A client on another chip pays its own turnaround instead.

**On the AVR64EA48 at 20 MHz** (rev B1, the same dual loop on PA2/PA3
and PC2/PC3, the same image; plain bindings, the thread spinning, best
of 8, in CLK_PER cycles). The SCL period measured on PA3: 201, 55 and
21 cycles at 100 kHz, 400 kHz and Fm+ (99.5 kHz, 363.6 kHz with FMEN,
952.4 kHz - MBAUD 94, 21 and 4, the register's floor 198, 52 and 18
plus the bus's rise); `wire` is the tenure's SCL rises times that
period:

| tenure | wire | measured | x |
|---|---|---|---|
| 1-byte write, 100 kHz | 3819 | 4236 | 1.11 |
| 255-byte write, 100 kHz | 463305 | 481967 | 1.04 |
| 255-byte read, Smart, 100 kHz | 463305 | 481857 | 1.04 |
| 1 + 1 register read, 100 kHz | 7638 | 8548 | 1.12 |
| 1-byte write, 400 kHz | 1045 | 1451 | 1.39 |
| 255-byte write, 400 kHz | 126775 | 153292 | 1.21 |
| 255-byte read, Smart, 400 kHz | 126775 | 155757 | 1.23 |
| 1 + 1 register read, 400 kHz | 2090 | 2793 | 1.34 |
| 1-byte write, Fm+ | 399 | 808 | 2.03 |
| 255-byte write, Fm+ | 48405 | 77327 | 1.60 |
| 255-byte read, Smart, Fm+ | 48405 | 83038 | 1.72 |
| 1 + 1 register read, Fm+ | 798 | 1507 | 1.89 |

The cycles above the wire are the DB's to within an eighth - the same
core and the same handlers: a 255-byte write at 400 kHz carries 26517
above the wire here, 23723 on the DB at 24 MHz; at Fm+ 28922 against
28033. A long write is wire-bound at 100 kHz and the two
handlers on one core cost a fifth at 400 kHz and three fifths at Fm+,
as on the DB. The client runs SDASETUP at eight clocks here, the
driver's default (Bench findings, "The client's data setup"), which a
long read pays on each byte whose clock the client releases; across ten
runs of letter i each 255-byte row spreads by 0.2 per cent at most.

## Bench findings

Measured on rev. A5 at 5 V, CLK_PER 24 MHz, TWI0 on its DEFAULT route,
one open-drain bus with 1.5k pull-ups - and on an AVR128DA48 rev. A7
(its external 24 MHz clock, 5 V, the same node), where `z` runs 206 of
206 and `y` 136 of 136, twice and three times, its meters reading the
bus rise as the same 166 ns. `test_avr_twi`: `z` = the
single-board half; `y` = the two-board half, with an ATSAMC21J18A on
the same node running the instrument firmware `twi_peer` (its SERCOM3
on PA22/PA23, commanded in band over the bus under test). The desk gives
that bus three taps of the DUT - PA2/PA3 (the host and the combined
client), PC2/PC3 (the dual client, and a plain-GPIO bit-bang injector
whenever Dual mode is off) and PB2/PB3 - plus the peer's pair, so a
host, several clients, a foreign chip and a foreign agitator all sit on
one wire.

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
| Sm 100 kHz | 115 | 244 ticks (register floor 240) | 98.36 kHz | 116 of 120 ticks = 4833 ns | 4700 ns |
| Fm 400 kHz | 34 | 82 ticks (floor 78) | 292.7 kHz | 35 of 39 ticks = 1458 ns | 1300 ns |
| Fm+ 1 MHz | 10 | 34 ticks (floor 30) | 705.9 kHz | 15 of 15 ticks = 625 ns | 500 ns |

The period exceeds the register's floor by exactly the rise time and the
low time falls short of `BAUD + 5` by exactly the fall time, which is
how both are read off the measurement: **tR = 166 ns on this bus at
every speed** (it is the pull-up's job, and FMPEN does not help it), and
**tOF = 166 ns in Standard and Fast mode but 0 in Fast-mode Plus** -
the x10 pad drive of FMPEN collapses the fall below the meter's 42 ns
tick. Charging nothing for tOF would leave the low phase the floor
itself in register ticks (113 and 32) and the pins that less the
measured fall - 4541 ns and 1166 ns, BELOW the specified floor at both
Sm and Fm: the equations' tOF is the difference.

**Declaring the bus's real edges buys the speed back.** The same
Standard mode with `rise_ns = 206, fall_ns = 206` gives MBAUD 113 and
measures 100.000 kHz with tLOW 4750 ns - still above the floor.

**The rate on the wire, against the rate ASKED.** Every letter of
`test_avr_twi` that sets an SCL rate judges the MEAN of the SCL periods
inside the bytes of its run - every capture of the period meter within an
eighth of the shortest, which leaves out the gaps between bytes where the
host holds SCL for its software and a client stretches it - on the
meter's CLK_PER ticks, the generator's own clock, so the input
synchronizer's tick is resolved over the run. The verdict every family's
I2C suites share is: never above the rate asked by more than 2 %, and
within a quarter below it. In thousandths of the rate asked:

| speed, edges charged | MBAUD | mean in-byte SCL | reading | the tLOW floor's rate |
|---|---|---|---|---|
| Sm, the specification's (1000/300 ns) | 115 | 98.22 kHz | 982 | 98.36 kHz |
| Fm, the specification's (300/300 ns) | 34 | 292.68 kHz | 731 | 292.68 kHz |
| Fm+, the specification's (120/120 ns) | 10 | 705.88 kHz | 705 | 705.88 kHz |
| Sm, this bus declared (206/206 ns) | 113 | 99.40 kHz | 994 | 100.00 kHz |
| Fm, this bus declared (206/206 ns) | 32 | 307.69 kHz | 769 | 307.69 kHz |
| Fm+, this bus declared (206/40 ns) | 8 | 800.00 kHz | 800 | 800.00 kHz |
| Sm at 24, 12 and 24 MHz again (a rebase) | 115, 55, 115 | 98.18, 98.36, 98.16 kHz | 981, 983, 981 | 98.36 kHz |

Nothing runs above the rate asked. Fm and Fm+ with the specification's
edges charged run MORE than a quarter below it, and that is this TWI's
clock and not its divider: the SCL clock "is designed to have a 50/50
duty cycle" (29.3.2.2.1), its high phase as long as its low, so step 3
of the chapter's procedure - which lengthens the LOW phase to the mode's
tLOW floor plus the fall charged (equation 29-5) - lengthens the high
phase with it, where the I2C modes ask far less of the high (0.6 us at
Fm against a tLOW of 1.3). At Fm that is 2 x 39 ticks plus the bus's
four of rise, 82 ticks and 292.7 kHz, at Fm+ 2 x 15 + 4 = 34 and
705.9 kHz; at Fm+ the floor's rate reaches the quarter on this bus's
rise only with a fall charged under about 80 ns. So the letters judge the lower bound as the
quarter, or - where the floor's rate is lower - that rate, computed in
the suite from equation 29-5 with the floors the chapter names, less
2 %: every reading sits on the floor's rate exactly, the divider
choosing what the procedure says. A bus that declares its edges gets
Fm and Fm+ back inside the quarter.

The fall the default charges at Sm and Fm is the I2C specification's tf
of 300 ns (`twi_fall_budget_ns`); the data sheet's own figure for this
pad's output fall, tOF in 39.16 (table 39-22), is 250 ns at both. With
250 the floor's rate at Fm would be 2 x 38 + 4 = 80 ticks, 300.0 kHz,
exactly on the quarter.

**A host and a client of the SAME instance talk to each other.** Every
Request shape moves real data both ways with the two halves on one pin
pair (combined) and with the client moved to PC2/PC3 through DUALCTRL
(dual): a five-byte write, a four-byte read, a write-then-read whose
repeated START shows up as two address matches in one tenure, and the
probe. The dual case is the proof that DUALCTRL really moves the client
to the second pair, since only the desk's wire closes that circuit.

**The address-match space behaves exactly as 29.3.2.3.1 describes**,
proven by who ACKs a probe: the exact address; a mask of 0x03 over
0x40 answers 0x40..0x43 and refuses 0x44 and 0x3F; ADDREN = 1 turns the
same field into a second exact address (0x42 and 0x55 ACK, 0x43 and 0x54
do not); PMEN answers everything asked of it, 0x08 to 0x77. The general
call and 0x7A are answered `i2c_rejected` inside `start()` with the
client armed for the general call seeing no address packet - a request
cannot reach SADDR[0] from this host. The address the client was called on is readable from SDATA
- necessary, since with a mask or PMEN the match alone does not say it.

**The chapter's cases, as MSTATUS values.** M1 reads 0x62 (WIF, ACK
received, clock held, bus Owner); M2 reads 0xA2 (RIF, not WIF - the two
are mutually exclusive); M3 reads 0x72 (WIF with RXACK set) - for a
read's address as for a write's, which the engine reports
`i2c_nack_addr` either way (bench_avr letter i's one-byte read to an
address nobody holds). A client
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

**A host told to start on a Busy bus really does hold.** Its address
had not reached the client after 2 ms, and forcing the bus Idle sent it
out at once, reliably.

**The time-out release of a HELD transaction is a race, not a
contract.** When the inactive-bus time-out itself matures with a
transaction held, the pending launch races the bus-state logic at the
maturation instant and BOTH outcomes are observed on the bench: a clean
launch (address out, ACK, WIF), or the launch dying in a **naked
BUSERR** - BUSERR with NO WIF and NO ARBLOST (unlike the chapter's M4
signature), nothing on the wire, the client silent, and CLKHOLD
possibly left standing so a flags-clear alone does not reopen the road.
Which outcome comes out is DETERMINISTIC for a given firmware image and
desk - the phase of the last bus edge against the launcher decides -
which is how a suite can promote one lucky outcome to a false contract
and then "break" when a wire moves. Neither errata document has an item
for this. The reliable half, bench-proven: the time-out DOES release
the bus, and after `recover()` the retried address goes out at once.
Software must treat a naked BUSERR after a held start as "recover and
retry", never as "bus dead".

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

Every figure below is `test_avr_twi y` against the SAM C21's
`twi_peer`, twelve runs of the same image - and three
more runs on the AVR128DA48 (rev A7, its external 24 MHz clock, 5 V)
land inside every range it gives (the 1 ms hold's
tenure 162100 ticks, the unstretched one 16814).

**Clock stretching is exactly what the client's hold costs.** A foreign
client that holds its answer to every DATA byte for a commanded time
(the SAM C21's client stretches SCL at DRDY until its software speaks)
lengthens the whole tenure by that time times the number of data bytes;
the address is answered at once. A six-byte write at 100 kHz:

| the client's hold | the tenure | the longest SCL low |
|---|---|---|
| none | 16814..16817 ticks (700 us) | 183..185 ticks (7.7 us) |
| 100 us/byte | 31303..31329 ticks (1304 us) | 2530..2537 ticks (105 us) |
| 1 ms/byte | 162087..162151 ticks (6753..6756 us) | 24331..24343 ticks (1014 us) |

The elongation measured 14489..14700 ticks against a model of 14400
and 145273..145484 against 144000: the commanded hold plus 0.6 to 2.1
us a byte at 100 us and 8.8 to 10.3 us at 1 ms - an excess that grows
with the hold, so most of it is the peer's own timing of the hold, not
the wire. The register's own SCL low time is 120 ticks (5 us), so **even
an unstretched exchange with a polled client runs at the client's
speed, not the divider's**: 185 ticks instead of 120. The host meanwhile
reports CLKHOLD whenever ITS own flag stands and the bus reads Owner
throughout - the two holds are different things and both are visible.
An unstretched exchange immediately after a stretched one is back to
nominal.

**Both NACKs, injected by a chip that is alive and listening.** A peer
that parks its address elsewhere produces an ADDRESS NACK: the engine
reports `i2c_nack_addr` and MSTATUS reads 0x72 (WIF, CLKHOLD, RXACK, bus
Owner), while the peer's own report says it matched nothing at all. A
peer told to NACK its third byte turns a six-byte write into
`i2c_nack_data` with the same 0x72, and it kept exactly the three bytes
it acknowledged. The host's closing NACK on a read reaches the far
client: the SAM C21's STATUS sampled at that instant reads 0x8C
(CLKHOLD, DIR, RXNACK).

**Multi-host arbitration is staged from the SAM C21's side.** This
host's held START on a Busy bus fires when an injected STOP frees it
(29.3.2.2.3), which is the race letter n stages - but it needs BOTH
held STARTs to fire on the one edge, and the SAM C21's does not after a
phantom hold ([../samc21/i2c.md](../samc21/i2c.md), "A tenure into a
busy bus parks in hardware"), so against that peer the letter is
skipped and says so. The live race between the two chips runs in
`test_samc_i2c` o, both STARTs queued behind one REAL STOP: this
chip's host, as the peer there, raises ARBLOST when the address
decides against it and its tenure completes when the address decides
for it ([../samc21/i2c.md](../samc21/i2c.md), "TWO LIVE HOSTS").

**The collision case S4 is the wired-AND read.** Two clients at ONE
address both acknowledge the same read and both put a byte on SDA, so
the host reads the AND of the two. The client that tried a high bit
against the other's low cannot drive it: it raises COLL and un-drives,
the other does not. With the DUT serving 0xFF against the peer's 0x55
the host reads 0x55 and the DUT collides; swap the two bytes and the
peer collides (the SAM C21's STATUS.COLL; its client leaves the tenure
after the byte that collided) while the DUT does not - every run. COLL
stands after the collision and **clears itself on the next Start
condition**, so a collision that is not read before the next START is
gone.

**A stuck wire needs clocks, not an ENABLE cycle.** With the peer
holding SDA low from PORT - taken after this host's STOP - the DUT's
host reads MSTATUS 0x03, BUSSTATE Busy, because a falling SDA against a
high SCL IS a Start condition to the state machine (letter p judges it),
and the address it was told to send never goes out, 2 ms of waiting
later. A peer that takes SDA BEFORE this host's STOP leaves the host
Owner instead (MSTATUS 0x02, its STOP pending): the remedy below frees
that bus the same way. `unstick()` then:

| the bus | result |
|---|---|
| healthy and idle | **0 pulses** (SDA was already high), bus Idle after, next exchange fine |
| a client released by the 4th SCL falling edge | **4 pulses**, SDA high again, bus Idle, next exchange fine |
| a line nothing will release | **`unstick_failed`**, SDA still down - the honest answer |

and it leaves both pins inputs with OUT = 0, which is what errata 2.15.1
wants of the next owner.

**The reserved addresses stay off a wire two clients listen on.** With
this chip's client and the peer's both answering the general call (the
peer's at the command address too), a write to 0x00 and one to 0x7A
were each answered `i2c_rejected` inside `start()`, this chip's client
matched nothing, and the peer counted one address match - the write to
its exact address that followed, its three bytes. And a **Quick Command
against a real second chip** is the same 10 SCL rising edges as against the
instance's own client, with `i2c_ok`; to an address nobody holds it is
`i2c_nack_addr`.

**The peer costs the bus nothing electrically, and its turnaround in
latency.** With the peer's client on the same node the minimum SCL
period is 244 / 82 / 34 ticks at Sm / Fm / Fm+ - the same numbers the
bus measures with the DUT alone, so the extra tap adds no rise time
this meter can see - and the mean in-byte period of letter r's
exchanges reads 983, 729..731 and 703..705 thousandths of the rate
asked, each on its tLOW floor's rate as with the DUT alone. What a
two-board exchange pays is the peer's polled turnaround: on letter r's
tenures the longest SCL low measured 204 / 234..257 / 170..253 ticks
(8.5 / 9.8..10.7 / 7.1..10.5 us) at the three speeds, against the
register's 120 / 39 / 15. **The data is exact at all three speeds** -
which is the protocol working as designed: stretching costs time, never
correctness.

**A clock rebase does not notice the second chip either.** Across
24 -> 12 -> 24 MHz with the command channel and the peer's client live,
MBAUD moved 115 -> 55 -> 115, `actual_scl_hz` stayed at 100 kHz, the
mean in-byte SCL read 983 thousandths at every step, and every exchange
across the wire was exact.

**A WEDGED TENURE IS THE ARBITER'S TO ANSWER, AND IT DOES (letter t).**
`I2cBus` with a 35 ms `timeout_ticks` over this engine, against the
peer holding SDA low from its PORT for 60 ms: the tenure's START parks
behind the held line (the address never goes out, as above), no
interrupt follows, and the reply comes back `i2c_timeout` on the
arbiter's clock (the RTC PIT at 1024 Hz, 36 ticks) at 35 ms WITH SDA
STILL LOW - the diagnosis is the timeout's, not the release's. The
arbiter's `recover()` (the ENABLE cycle and a forced Idle) runs in that
same dispatch, and once the line is free the same bus AO carries the
next tenure `i2c_ok` with no hand on the engine and no stale completion
to drain. A client stretching every byte by 1 ms under the same limit
completes `i2c_ok` in 9 ms: stretching is flow control, and the limit
sits above a whole tenure.

**The client's address match wakes this chip from STANDBY and from
POWER-DOWN, and the host pays the restart on SCL** (`test_avr_sleep` m,
the SAM C21 as the host timing the tenure, twelve runs): three bytes at
100 kHz take 393.5 us awake and out of a standby with the clock kept
alive, 1865 us out of power-down - the crystal's restart held on SCL
from the match to the first serviced byte. The figures and the rule
(the data interrupts are no wake source) are in
[platform.md](platform.md), "Wake-up latency, measured from the partner
board". On the **AVR64EA48** (rev B1, ten runs of ten) the client over
the DB's engine does the same on TWI0's default pins: 393.4 us awake and
out of standby, about 1.0 ms out of power-down - the crystal's cold
restart (0.6 ms) held on SCL; DS40002443A 27.3.5 promises the stretch.

### On the AVR64EA48

Rev B1 at 5 V, CLK_PER 20 MHz from the board's quartz, TWI0 on its
DEFAULT route on the same kind of node - 1.5k pull-ups to +5 V, the dual
pair PC2/PC3 and the partner SAM C21's PA22/PA23 on it (the board's
page). `test_avr_twi` builds two images there (the group axis):
`test_avr_twi-1` (`z`, with the EA's letters u and v) 225 of 225,
`test_avr_twi-2` (`y`, against the SAM C21's `twi_peer`) 133 of 133,
each in every run (thirteen of each image), letter `d` and letter `q` ten
times more each.

**The duty, measured** (letter u: MBAUD swept, the shortest low and high
pulses and the period on PA3 under NACK traffic, CLK_PER ticks; the bus's
rise is 3 ticks):

| MBAUD | Sm: period / low / high | Fm (FMEN): period / low / high | Fm+ (FMPEN): period / low / high |
|---|---|---|---|
| 0, 1 | 8 / 2 / 6, 10 / 4 / 6 | the same (FMEN does nothing) | no clock |
| 2 | 11 / 5 / 6 | no clock | 17 / 8 / 9 |
| 3 | no clock | no clock | 19 / 10 / 9 |
| 4 | 21 / 6 / 15 | 21 / 9 / 12 | 21 / 12 / 9 |
| 6 | 25 / 8 / 17 | 25 / 13 / 12 | 25 / 16 / 9 |
| 8 | 29 / 10 / 19 | 29 / 15 / 14 | 29 / 18 / 11 |
| 32 | 77 / 34 / 43 | 77 / 39 / 38 | 77 / 42 / 35 |
| 95 | 203 / 97 / 106 | 203 / 102 / 101 | 203 / 105 / 98 |

The period is `10 + 2 BAUD` plus the rise at every MBAUD from 4 (2 under
FMPEN), in all three duties; FMEN lengthens the low by `min(5, BAUD)`
within a tick and FMPEN by `min(8, BAUD + 2)`, FMEN or not; and the low
reads `twi_low_ticks()` less one offset - 4 ticks, the fall and the input
path, read once at MBAUD 95 - within a tick at every MBAUD, ten runs of
ten. The Sm low reads BAUD + 2 against the DA/DB's BAUD + 1 on the same
kind of node (the 24 MHz table above: 116 for 115): the one clock the
chapter's 3.2 adds. Below `twi_baud_min()` the host makes no regular
clock - periods of 8 to 11 ticks, shorter than equation 1's, and none at
all at MBAUD 3 without FMPEN, at 2 and 3 with FMEN, at 0 and 1 with FMPEN.

**The three speeds** (letter b, the default timing; then this bus's edges
declared, 190 ns rise and 240 ns fall):

| speed | MBAUD | period | SCL | tLOW | reading (thousandths) |
|---|---|---|---|---|---|
| Sm | 94 | 201 ticks (floor 198) | 99.50 kHz | 96 of 100 ticks = 4800 ns | 995 |
| Fm, FMEN | 21 | 55 (floor 52) | 363.6 kHz | 28 of 32 = 1400 ns | 909 |
| Fm+ | 4 | 21 (floor 18) | 952.4 kHz | 12 of 16 = 600 ns | 952 |
| Sm, declared | 94 | 201 | 99.50 kHz | 4800 ns | 995 |
| Fm, declared | 20 | 53 | 377.4 kHz | 27 of 31 = 1350 ns | 943 |
| Fm+, declared | 4 | 21 | 952.4 kHz | 600 ns | 952 |
| Sm at 20, 10 and 20 MHz (letter j) | 94, 44, 94 | 201, 99, 201 | 99.50, 101.0, 99.50 kHz | | 995, 1010, 995 |

tR reads 150 ns and tOF 200 ns here. Fm with FMEN runs within a tenth of
the rate asked where the DA/DB's 50/50 clock ran at 0.73 of it; at 10
MHz Sm runs a percent fast (The AVR EA's SCL clock).

**The time-out counts MCLKTIMEBASE** (letter v; the injector's START
leaves the bus Busy, the bus state is polled from the injector's return,
20 us after the START): with the clock's TIMEBASE (20) the three settings
release the bus 31, 81..82 and 181..182 us after the return - 50, 100 and
200 us from the START, within the stopwatch's microsecond; with TIMEBASE
10 at 6, 31 and 82 us - half of each; with TIMEBASE 0, the register's
reset value, at once (0 to 1 us). Letter h's verdicts on 50/100/200 us
read 31/82/181 from the same return.

**SDAHOLD's codes are straight** (letter v: LUT0 high while SCL is low
and SDA high, SCL by its pin event, SDA on LUT0's own IN2 = PA2, the
window's shortest pulse in TCB0 - the host's SDA change after SCL's fall
under NACK traffic): 8, 9, 12 and 20..21 ticks for codes 0..3 - code 1
adds 50 ns, code 2 200 ns, code 3 600..650 ns (table 35-21: 300..900 ns
for code 3, nothing for 1 and 2). No swap: `erratum_da_twi_sdahold_swap`
is the DA's alone.

**No OUT override, and FLUSH did not stick** (letter v): with PORT.OUT =
1 on PA2 and PA3 under the enabled TWI a two-byte write completes and the
client takes both bytes - the DA/DB's 2.15.1 / 2.14.1 has no twin here,
and the init's clearing of OUT (one store a pin) stays, harmless; 1000
FLUSH strobes on an idle host never left BUSSTATE other than Idle and the
bus ran after them.

**A STOP inside a byte is not a bus error here.** Letter h's injected
START, one clock and then a STOP: the DA/DB host raises BUSERR and keeps
its held START pending; the AVR EA's takes the STOP as the end of the
foreign tenure - MSTATUS 0x02, no BUSERR, the bus Owner - and launches the
START it was holding at once (the address reached the client on the next
pass), ten runs of ten. 27.5.6's BUSERR text is the DA/DB's word for word.

**Against the SAM C21's client** (`y`): the three speeds read 994, 909
and 952 thousandths of the rate asked; a 1 ms hold a byte stretches six
bytes by 121033 ticks against a model of 120000 (100 us a byte: 12051
against 12000); the 20 -> 10 -> 20 MHz rebase under two-board traffic
reads 994, 1009 and 994; every other letter as on the DA/DB. Letter n
(multi-host arbitration) is skipped against this peer, as it is on the
DA/DB.

**The client's data setup: four clocks lose a Smart client in a long
read, eight do not** (`bench_avr` letter i's 255-byte read from the dual
client, at 400 kHz; recorded by the SAM C21 on the same node - PA22 and
PA23 read as plain inputs, nothing driven, every change stamped by a
seven-cycle loop at 48 MHz, so a record's time is its edge's to within
146 ns and two edges inside one such turn share a record - with the two
TWI vectors marking their bodies on two link wires the recorder reads
with the bus). Through its stretch after a served byte's acknowledge the
client holds SDA at that byte's bit 6: low after 0x00..0x3F and
0x80..0xBF, released after 0x40..0x7F and 0xC0..0xFF, both read off the
record. In Smart mode the SDATA write is the release: at SDASETUP four
(200 ns at 20 MHz) the new byte's first bit and SCL's rise land in the
same turn of the record. Where the two levels differ the WRONG way - bit
6 set, the next byte's first bit 0, so SDA falls as SCL rises, within
one turn of the record (a START, if SCL rises first) - the client drives
that first bit and the next, then nothing, and its vector is not entered
again: the rest of the read is 0xFF (the host sampling the first bit
either side of the fall, 0x7F or 0xFF for that byte), the host saw no
bus error at any of its 255 entries, and both halves read BUSERR after
the STOP, the request complete with `i2c_ok` - a read carries no check
the engine could apply. The byte is the data's: 65 of a read serving 0,
1, 2 ... (after 0x40), 128 with the top bit inverted (after 0xFF); 100
reads of 100 at 400 kHz lost it there, host Smart mode on or off, the
thread spinning or asleep, the vectors marked or plain. It wants the
CLIENT to release SCL (at 400 kHz its stretch, about 4 us, outlasts the
host's low phase every byte): at 100 kHz the host's own low phase mostly
ends later and SCL rises with SDA long settled - one read of ten lost
its client, at byte 91 after 0x5A - and 48 cycles more in the client's
vector lose ten of ten; at Fm+ the client's pads run FMPEN and ten of
ten are exact. At SDASETUP EIGHT (400 ns) SDA moves a record or more
before SCL, and 420 reads of 420 are exact - 100 kHz, 400 kHz and Fm+,
both patterns, the same variants, the 48 cycles included. Microchip's
bare-metal TWI client example for the AVR128DB48 (the same chapter) runs
Smart mode off and writes SDATA, then SCMD's RESPONSE: 11 reads of 11
exact at four clocks too, the store between the two the setup. The
driver's default is eight on every die (Configuration); `bench_avr`
letter i reads 255 bytes exact at every speed with it (the table above).

**A desk fact, not a silicon one**: TWI1's dual pair PB2/PB3 would make
a third tap of this bus - the suite drives PB2 low and prints whether the
SDA node follows, instead of assuming either way.

## Not covered yet

**Driver gaps** (the chapter's own features this driver does not
implement):

- **The AVR DA/DB's Fm+ clock floor.** DS40002247B's TWI electricals
  give 1 MHz only over 10 MHz of CLK_PER; `twi_fm_plus_min_clk_hz` is 0
  there, so the divider alone decides (it reaches 1 MHz's floor from
  about 9 MHz). Enforcing it moves every DA/DB image that builds the
  engine, owed with a DA/DB board on the desk.
- **The AVR EA's ALT3** (SDA PA0, SCL PA1): in the route table and the
  fixture only - this desk's EA board has its quartz on PA0/PA1, and no
  letter routes there.
- **10-bit addressing.** The client's match logic recognizes the first
  byte when SADDR[7:3] is 0b11110 and the second byte is software's job
  (29.3.3.6); neither the client task nor the host's `Request` has a
  shape for it.
- **The general call as a host verb.** `start()` refuses address 0x00
  with every reserved address ([../design/i2c-bus.md](../design/i2c-bus.md));
  a broadcast is a feature of its own, born with its first user.
- **Multi-host as a policy.** The engine reports `i2c_arb_lost`, but
  nothing above the engine decides what to do with it: a retry policy, a
  back-off, a bus AO that knows it shares the wire. `i2c_arb_lost`
  reaches the requester and stops there.
- **A client-side AO.** The client task is a polled/ISR surface; the
  kernel-level usage type (an AO that owns an address and answers
  register reads) is not built.

**Implemented but not bench-verified**:

- letters `d` and `q`'s refusal of the reserved addresses on the AVR DA
  and DB: run on the AVR64EA48 only, the DA and DB boards being off the
  desk; a run of the two letters on each would close it;

- the client's general call (`general_call`, SADDR[0]): written, and
  taken on the wire no longer - every brio host refuses address 0x00,
  this chip's own and the desk partner's; a controller that sends a
  general call (this suite's injector pins, driven by hand, could) would
  measure it;

- the engine's `i2c_arb_lost` on a lost arbitration (MSTATUS with
  ARBLOST, the transaction not landing) with this chip as the DUT: the
  ARBLOST itself is measured as the SAM C21's peer, the engine's report
  of it is not - letter n's race needs a peer whose held START fires on
  an injected STOP, which the SAM C21's does not; a leg of n queuing
  this host's START and the peer's behind one REAL STOP, as
  `test_samc_i2c` o does from that side, would measure it;
- the AVR DA/DB's client at SDASETUP eight: every DA/DB figure and
  verdict on this page was taken with the client at four clocks (exact
  there - the DB's dual loop at 24 MHz, 167 ns); the default's eight add
  four clocks a byte where the client releases SCL, and the DB's letter
  i and its two `test_avr_twi` images would measure the change - the DA
  board on this desk has no pull-ups without the SAM C21's pair;
- the DA's swapped SDAHOLD codes (DS80000882C 2.14.2): `sda_hold` writes
  the swapped code on a DA and `sda_hold_code` reads it back; the AVR
  EA's letter v measures the codes on the wire (straight there), and the
  same letter on a DA board with this bus's pull-ups would measure the
  swap;
- the AVR EA's Fm+ pads (FMPEN, 3 mA at 0.4 V in table 35-21 where the
  DB's is 5): VOL and tOF at FMPEN 0 and 1 against 1.5k to 5 V - PA3 is
  AIN23, so an ADC leg reading the held SCL would measure it; and Fm+ on
  the dual client's PC6/PC7 (the DB limits them to Sm/Fm, the EA's
  chapter is silent) - the dual pair this desk wires is PC2/PC3;
- the AVR EA's INPUTLVL thresholds: no TWI VIL/VIH row in 35.7 or
  DS80001048D; the DAC ramping the node would find them, which needs a
  wire from PD6 to the bus;
- 1 MHz between 10 and 12 MHz of CLK_PER on the AVR EA: this board's
  clock ladder reaches 20 and 10 MHz (refused at 10 by the divider and the
  12 MHz floor alike), not the band between;
- SMBus as an ELECTRICAL fact: CTRLA/DUALCTRL.INPUTLVL is exposed and
  written, but the input transition level itself is not measured;
- DBGCTRL.DBGRUN;
- the timings of electricals 39.16 as numbers;
- TWI1, and every route other than TWI0 DEFAULT/ALT1/ALT2, are compiled
  for all eight DA/DB packages and the three AVR EA ones and refused
  where they must be, but only TWI0's DEFAULT pair has carried traffic
  on this desk.
