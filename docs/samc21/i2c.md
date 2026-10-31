# SERCOM I2C (SAM C21)

This document is for a programmer who needs an I2C bus on the SAM C21: a
host reading and writing devices (a sensor, an EEPROM, a DAC) through
the bus arbiter, or this chip answering another host as a client.
`samc21/i2c.hpp` is the SERCOM's I2C mode in four parts: `I2cm<n>` and
`I2cs<n>`, the host and the client resources, one function per register
fact of one SERCOM instance; `I2cHost<n, pads, generator>`, the transfer
engine `util/i2c_bus.hpp` drives, a task (in brio, a driver layer with
state of its own, served from its peripheral's interrupt) whose Request
every target's I2cHost carries field for field; and `I2cClient<n, pads,
generator>`, the client role. The arbiter, `I2cBus`
([design/i2c-bus.md](../design/i2c-bus.md) is the model), is an active
object (a component of a brio program with its own event queue, served
one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)) that owns the
bus on behalf of every other active object: a client posts one Request
describing one tenure - one ownership of the bus from its START to its
STOP: a write, a read, or a write then a read joined by a repeated START
- and gets one `I2cDone` reply when it is over, the tenures run one at a
time in the order they arrived.

The smallest use, a device client on the arbitrated bus, is the block
of "How to use" below:

```cpp
using I2cHw = brio::I2cHost<3, my_pads>;          // generator 0 on a clean wire
using I2c = brio::I2cBus<I2cHw, P>;
// main: I2cHw::init(clock, measured_rise_ns); kernel.init_all(); ...
extern "C" void SERCOM3_Handler() {
    if (I2cHw::isr()) { brio::post<I2c>(brio::TransferDone{I2cHw::status()}); }
}
// from an AO: {addr, tx spans as lend<Lease::reply>(...), rx, reply,
// speed} posted to I2c; the I2cDone reply carries i2c_ok or the
// wire's own answer (i2c_nack_addr is the address-scanner's probe
// result).
```

Line by line: `I2cHw` names the engine over SERCOM3 with the pads
`my_pads` (an `I2cPads` constant naming the two pins: SDA is the
SERCOM's PAD[0] and SCL its PAD[1] by the chapter) and the SERCOM's core
clock taken from generator 0, which runs at the CPU rate; `I2c` is the
arbiter over that engine on the platform `P`
([platform.md](platform.md)), with `util/i2c_bus.hpp`'s defaults of four
pending requests and no timeout; `I2cHw::init(clock, measured_rise_ns)`
takes the clock task's tag ([clock.md](clock.md)) and the bus's rise
time in nanoseconds, which enters the baud arithmetic (300 ns when it is
left out), and the rest of `main()` starts the kernel with `I2c` in its
list of active objects (`run()` calls every active object's `init()`,
which is what `init_all()` names); the `extern "C"` function is the
SERCOM3 interrupt handler, as in C, under the name the crt's vector
table gives it, and `I2cHw::isr()` answers true when a tenure has just
completed, the edge on which the handler posts `TransferDone` with the
engine's status to the arbiter; and an active object (AO) fills a
`Request` - the 7-bit address, the bytes to write and the buffer to read
into, both lent with `lend<Lease::reply>(...)`, the reply channel, the
speed - posts it to `I2c` and receives `I2cDone` with `i2c_ok` or the
wire's answer (an address nobody acknowledged is `i2c_nack_addr`, the
probe result an address scanner reads).

What decides correctness even there. `init()` comes after the clock
task's: the three speeds' register pairs are computed from the rate the
clock type claims (or from the `core_hz` a caller states for a divided
generator), not read from the hardware. `init()` answers false when even
the standard 100 kHz speed cannot be produced at that core rate, or a
synchronization of the peripheral, the forcing of the bus state to IDLE
included, did not complete; a faster speed the core cannot make is not
an error at init but a request answered `i2c_rejected` without moving a
byte, and so is a request whose address lies outside 0x08..0x77 - the
general call and the 10-bit prefixes among them, which every host
refuses ([../design/i2c-bus.md](../design/i2c-bus.md)). The pads are handed to the SERCOM with input enable and no pull,
so the bus's pull-up resistors are the board's, and which pins may carry
I2C at all is the data sheet's per-package table, which no header symbol
encodes: the code cannot check the pins named. Every tenure that moves
the wire runs on the interrupt, one per byte, so the SERCOM's vector
must be bound to `isr()` as above; an unbound vector is a weak alias of
the crt's silent spin, where the program stops at the first byte. The
request's buffers are lent: the client leaves them untouched and in
scope from the post until its `I2cDone` arrives, whatever the status,
and nothing checks it. And the wire decides the core rate this engine
can run at: the I2C logic has no input filter, so on a bundled pair with
crosstalk a 48 MHz core reads glitches as false Start and Stop
conditions and only a slowed core works (the headline section below);
generator 0 is the choice for a clean, short wire.

The page is long because the chapter is. In order: the documents of
record; the headline, the wire finding that decides the core rate; what
the silicon does (two register sets, SYSOP, the bus state machine, the
clock hold, the automatic length and smart mode as this silicon runs
them, the offer the engine takes or declines, the SMBus time-outs, the
errata as code); the types and verbs; how to use; the host's cost,
measured; the bench findings beyond the headline; and what is not
covered.

## Documents of record

SAM C20/C21 data sheet DS60001479M ch. 33 (over
the shared SERCOM ch. 30) and errata DS80000740S 1.17.x. Driver:
`samc21/i2c.hpp` over `Sercom<n>`'s instance facts (sercom.md owns the
address ladder; `i2cm_regs()`/`i2cs_regs()` and `gclk_slow_id()` sit
beside the SPI view). Family fixture `test/family_samc21/i2c.cpp` +
seven negatives (the per-bus timeout adds an
eighth at the util level: an engine without `recover()` refused);
bench suite `test_samc_i2c`, bench letter `bench_samc` i
(15 letters - a letter is one command key of a suite's console and the
group of verdicts it runs - two-board: the peer board runs `twi_peer` on an AVR64EA48,
an AVR DA/DB or a second SAM C21, one `twi_link.hpp` wire format between them -
commanded in band over the bus under test), and `test_samc_rubato_i2c`
for the stack under the preemptive kernel (the last of the bench
findings).

## The headline: this peripheral has no input filter, and the wire decides everything

The C21's I2C logic - the host's bus monitor and the client's
Start/Stop/address machinery alike - samples SDA and SCL on
GCLK_SERCOMx_CORE with NO glitch filter. The AVR's TWI filters its
inputs, and that SMBus-grade suppression is what carries this very node
at 1 MHz there; the C21 believes every nanosecond of the wire.
On a clean short bus that is free speed. On a bundled desk - the I2C
pair riding a SEVEN-WIRE BUNDLE between the boards - per-edge crosstalk
(~100 ns class, SCL's driven edges coupling into the released SDA)
reads as false Start/Stop conditions, with three measured
consequences:

- **The host's glitch wall is a CORE-RATE wall, not an SCL wall.** A
  hand-driven tenure (SWD only, no driver code in the loop) dies with
  BUSERR+ARBLOST on its FIRST address at 48, 24 and 12 MHz core - at
  every SCL rate tried, including 25 kHz - while a 6 MHz core is clean
  six-for-six at SCL 400 kHz, and at a 32 kHz core the whole address
  was watched crossing the wire pin by pin and the peer ACKed it. Same
  registers, same wire, only the sampling rate moved: the fine core
  SEES the glitches, the coarse one steps over them.
- **A bundled pair therefore wants a slow core** - generator 6 =
  OSC48M/8 = 6 MHz is a notch above 400 kHz SCL and no more - and
  `I2cHost` has the two knobs that make that expressible: the
  `generator` template argument and `init(clock, rise_ns, core_hz)`'s
  stated core rate (the freqm reference_hz pattern: a divided
  generator's rate is the caller's claim). Fm+ (1 MHz) needs a 12 MHz
  core and is therefore UNREACHABLE behind such a wall: `speed_ok()`
  says so and a Request naming it is answered `i2c_rejected` without
  moving a byte - never run slow in silence.
- **The client cannot dodge the wall by slowing down, because it does
  not own the edges.** As a CLIENT the same silicon matched a
  bit-banged address (seconds per edge, driven from the AVR's PORT over
  UPDI) perfectly - AMATCH up, SCL stretched, registers identical to
  the failing case - and stayed DEAF to the peer's real 100 kHz host at
  BOTH 6 and 48 MHz core, while that host read a clean address NACK
  (MSTATUS 0x72, the same nobody-home signature the AVR's TWI reports).
  A client must follow foreign edges wherever they land; per-edge
  glitches reset its machinery every time.
- **AND THE FIX IS MEASURED, NOT JUST NAMED - with its confound
  stated.** With the I2C pair wired SHORT AND SEPARATE the
  wall is gone: the whole suite runs with BOTH ends' cores at 48 MHz -
  the peer's client serves the command channel at the very rate that is
  stone deaf on the bundle, the client letter takes a foreign 100 kHz
  host's burst byte-exact and rests its data verdicts on it, and
  fast-mode-plus (unreachable behind the bundle's 6 MHz core) runs 25
  tenures x 16 bytes in 6 ms, all i2c_ok. TWO THINGS DIFFER TOGETHER,
  though, and the experiment does not split them: the clean pair is
  both shorter AND out of the bundle. Since the identified aggressor
  was largely SCL's own edges coupling into the released SDA - and SDA
  and SCL are still adjacent, being a pair - the LENGTH of the parallel
  run (which mutual capacitance scales with) is plausibly the dominant
  term and the bundle its multiplier. What is measured: the long
  bundled pair dies above a 6 MHz core; the short separated pair is
  clean at 48. The split between length and separation is not.

## What the silicon does

**Two register sets, really different.** Unlike the SPI's SPIM/SPIS
pair, I2CM and I2CS differ at the same offsets (the client has no BAUD
and no bus state; AMATCH/DRDY/PREC against MB/SB; different STATUS
bits) - so `samc21/i2c.hpp` carries TWO resources, `I2cm<n>` and
`I2cs<n>`, and the role picks which one speaks the truth.

**The host's third SYNCBUSY bit.** Beside SWRST and ENABLE the host
has SYSOP, raised by writing CTRLB.CMD, STATUS.BUSSTATE, ADDR or DATA
while enabled (33.6.6). Every such store in the driver waits first
(the sdadc discipline).

**The bus state machine and its exits.** A freshly enabled host is
UNKNOWN and leaves it by a Stop, by the INACTOUT time-out, or by
software forcing IDLE (33.6.2.3). Measured: with no time-out the state
sits in UNKNOWN until `force_idle()`; whether INACTOUT walks it out by
itself came out both ways on this bench (IDLE on one run, still
UNKNOWN on the next, same code) - the suite records the observation
and relies on `force_idle()`, which the engine's init always spends.

**MB/SB hold SCL.** Both host flags stretch the clock until software
answers with DATA, ADDR, a command or a flag clear - unlimited time to
respond. STATUS.CLKHOLD says so, and erratum 1.17.8 (live) makes that
bit writable against the datasheet - writing it corrupts the clock
hold, so both resources' W1C masks exclude bit 7 by construction
(`I2cmStatus::w1c_all` / `I2csStatus::w1c_all` cannot express it).

**One wire fault raises MB and ERROR together**: an ISR that consumes
MB and leaves ERROR standing lets the level storm the vector with main
starved - measured, with IPSR reading the SERCOM's IRQ and INTFLAG
0x80. So every exit of the engine sweeps all flags and W1C statuses
(`finish()`), and a flag arriving with no tenure in flight is swept by
the idle guard rather than returned to.

**A tenure into a busy bus parks in hardware.** Writing ADDR while
another agent holds the wire parks the START until the bus idles
(33.6.2.4.2). Behind a real foreign tenure it fires at that tenure's
STOP: started 1 ms into the peer's 16-byte write, a tenure waits it out
and completes (letter o). Behind a phantom - the peer pinning SDA low
from a pin, no tenure around it - the bus reads BUSY for the hold's
whole length, not one byte moves, and on this silicon the parked START
does not fire when the hold releases (the phantom Start keeps the state
machine BUSY and even a configured INACTOUT does not walk it back): the
way out is the engine's own re-init, and letter g prints the timeline.

**Writing ADDR is the clear ceremony.** BUSERR, ARBLOST, LENERR and
the time-out statuses auto-clear on the next tenure's ADDR write
(33.10.7), which is why the engine starts a tenure with no ceremony.

**The automatic length, as this silicon runs it** (ADDR.LEN + LENEN,
33.10.9, 33.6.4.1.2; every point traced flag by flag against the peer at
100 kHz; the host does not use it, the table below):

- A WRITE under LEN = n raises MB after the address and after every data
  byte but the n-th, each with the clock held for the next DATA; after
  the n-th it sends the STOP by itself and no flag rises - no MB after
  the last byte, none after the STOP, the bus simply going IDLE. With
  LEN = n + 1 the MB after byte n stands with the clock held and its
  acknowledge read, ready for a STOP or a repeated START.
- A NACK under LEN - the address's or any data byte's before LEN - is
  LENERR + RXNACK + ERROR with no MB, and the STOP already sent: one
  signature for an absent client and for a refused byte.
- A READ under LEN = n raises SB for each byte; the DATA read of the
  n-th sends the NACK and the STOP by themselves. SB then stands for a
  while (some hundred cycles at 100 kHz), and an ADDR written while it
  stands is taken as a repeated START: the next tenure dies in BUSERR +
  ARBLOST, whether or not software swept the flag first. Waiting for SB
  to fall, then writing ADDR, is clean: the host holds the new START
  until its own STOP has left.
- An empty probe under LEN = 0 is the STOP after the address's ACK, again
  with no flag; to nobody, RXNACK and the STOP with no ERROR.
- LEN is eight bits (33.10.9): 255 bytes is the longest counted tenure,
  so a write that wants the MB after its last byte (LEN = n + 1) is at
  most 254 bytes long.

**Smart mode, as this silicon runs it** (CTRLB.SMEN, 33.6.3.2): the DATA
read sends ACKACT and clocks the next byte in whatever ACKACT says -
with ACKACT = NACK the read sent the NACK and still read one more byte
(measured), unlike the AVR TWI's smart mode, which sends nothing on a
NACK. So the last byte is closed by the STOP command with its NACK while
SB holds the clock, synchronized, and only then read (33.10.10's
"reading the last data byte after the stop condition has been sent").
And ACKACT persists: a read's closing NACK leaves it set, and the next
read's first DATA read would NACK that byte - every reading phase puts
ACK back first.

### What the silicon offers for the cost, and what the engine takes

| item | where | the engine |
|---|---|---|
| MB / SB, one interrupt a byte with the clock held | 33.6.2.4, 33.10.6 | THE PUMP, every tenure. One STATUS snapshot a handler, the SYSOP wait inline (one load when the last operation synchronized long ago) |
| smart mode | 33.6.3.2 | USED, always: a pumped read byte is one DATA load, no command and no SYSOP wait |
| the two DMA requests (TX, RX) | 33.6.4.1.2, table 33-2 | DECLINED: erratum 1.10.4 ([dmac.md](dmac.md)) - this stratum drives the DMAC for the Uart's transmitter alone |
| ADDR.LEN + LENEN | 33.10.9 | DECLINED: its use would be to close a DMA phase; the pump closes a read by smart mode and a write by its last MB, every byte software's anyway. The resource keeps the verb (`I2cm::start_address_len`), its behaviour measured above |
| quick command (QCEN) | 33.6.3.4 | DECLINED for the probe: an empty request costs one MB either way. On the wire (letter m) it changes a READ alone: SB after the address's acknowledge, nine SCL rises, where a plain read clocks the byte in first (seventeen); a write raises MB at the acknowledge either way |
| SCLSM = 1 | 33.6.2.4.5 | DECLINED: the engine's decisions are taken before the acknowledge (SCLSM = 0), and erratum 1.17.13 refuses it beside QCEN |
| High-speed mode | 33.6.2.4.6 | REFUSED: errata 1.17.7 and 1.17.9 |
| the SMBus time-outs | 33.6.3.1 | the caller's, over `configuration()` (the finding above); the engine answers a time-out's ERROR with `i2c_bus_error` |
| INACTOUT | 33.6.2.3 | USED: 205 us, the BUSY state's escape |

**The SMBus time-outs bound a client's hold of SCL, as 33.6.3.1 reads -
when they are armed, on a slow clock weighed to 32 kHz.** Under LOWTOUTEN
and SEXTTOEN a client stretching every byte by 40 ms ends the tenure at
29 ms, inside the 25..35 ms window, and the engine answers
`i2c_bus_error` (letter j). They bound the host's own unserviced hold as
well: MB left unserviced trips LOWTOUT at 29 ms measured, with the
chapter's exact signature - STATUS.LOWTOUT + BUSERR and INTFLAG.ERROR
beside the still-standing MB. MEXTTOEN, the host's own extend, polices
that same unserviced hold alone - STATUS.MEXTTOUT + BUSERR and
INTFLAG.ERROR, LOWTOUT never reached - and does not see a client's
stretch: two bytes stretched 15 ms each complete `i2c_ok` under it.
**The three counts are not the chapter's numbers.** Timed in
microseconds on the meter FREQM weighs (33.1 to 33.2 kHz, the factory
trim), every shot of fifteen runs: LOWTOUT at 962 to 964 meter cycles,
SEXTTOEN alone (one 40 ms stretch, counted from its start) at 960 or
961, MEXTTOEN at 482 to 484 (sixteen shots a run, a spread of 30 us).
At 32.768 kHz those are 29.3, 29.3 and 14.7 ms: LOWTOUT inside its
25..35 ms window, but SEXT and MEXT late against the 25 and 10 ms
33.6.3.1 states - MEXT exactly half of the other two - and both errata
documents silent. A design that leans on SMBus's T_LOW:SEXT and
T_LOW:MEXT gets them from this silicon at those counts. Two operational rules the chapter does not
state: the counter arms at configure() - the enables left in CTRLA by an
earlier configuration time nothing until a fresh disable/write/enable
cycle - and a wrong-rate meter is mute, not scaled: with
GCLK_SERCOM_SLOW at 48 MHz the same hold never trips at all (a
clock-domain limit), so a design that enables these time-outs must weigh
its slow clock (the suite prices its OSC32K meter on FREQM, 32.2 kHz
with the factory trim). And a third, the one that hid the first finding
for a while: `I2cm<n>::configure()` disarms every interrupt, so a caller
that reconfigures the resource behind a running engine must start from
`I2cHost::configuration()` - which keeps smart mode, the INACTOUT escape
and the rest of what the engine relies on - and the engine arms ERROR
again with every phase. What the time-outs cannot see is a wire wedged
by SDA: the START parks with no clock moving (letter g), and only the
arbiter's per-bus `timeout_ticks` (util/bus_master.hpp, letter l)
answers that with `i2c_timeout` on the kernel's clock and `recover()`s
the engine; the wire - unstick(), re-probing the clients - stays the
application's recovery ladder (docs/design/i2c-bus.md).

**Erratum 1.17.16 not reproduced in I2C mode either**: SWRST from the
disabled state reset the block with its synchronization completing
(0x30200014 -> 0), matching the SPI-mode measurement. The enable-first
discipline is kept in both resources - the sheet marks every revision
and the cost is one enable.

**The rest of the errata as code.** 1.17.10 (10-bit client addressing
dead): `I2csConfig` has no ten-bit knob at all. 1.17.11 (client error
bits not cleared with AMATCH): `I2cs::clear_errors()` writes them by
hand and `answer_address()` spends it on every match. 1.17.13 (quick
command + SCLSM=1 = bus error): the pair is refused at compile and run
time. 1.17.21 (AACKEN broken on repeated start): no AACKEN knob - an
AMATCH handler is the workaround's own prescription and `I2cClient` is
one. 1.17.22 (client RXNACK invalid at the first DRDY): the software
flag the workaround prescribes is `I2cClient::first_drdy()`, armed by
the acknowledged AMATCH. 1.17.6/7/9 (repeated starts in 10-bit and
High-speed): the engine's only repeated start is the 7-bit
write-to-read turn, which none of them touches; HS itself is refused
by `i2cm_config_valid()` (both its repeated-start halves are broken
with no workaround, and no bench wire here could carry 3.4 MHz).

## Types and verbs

- **`I2cm<n>` / `I2cs<n>`** - the two resources: full register
  surfaces, enable-protected configuration written disabled, bounded
  synchronization waits, the SYSOP discipline (host), the CLKHOLD-free
  W1C masks, `force_idle()`, the erratum sweeps - and the client's
  `end_transaction()` (table 33-3's CMD 0x2: after the host's closing
  NACK of a read tenure the machinery goes back to waiting for a start
  instead of stretching for a byte nobody wants - what a client that
  serves reads needs).
- **`i2c_baud_for(gclk_hz, scl_hz, rise_ns, fast_plus)`** - the
  chapter's own formula solved for BAUD/BAUDLOW, the rise time an
  argument (a budget that ignores it lands T_LOW under the
  specification floor), the Fm+ 1:2 split per the chapter's note,
  rounding that never lands above the request, and
  `i2c_scl_hz()` as the readback. Pinned by static_asserts.
- **`I2cHost<n, pads, generator>`** - the engine
  `util/i2c_bus.hpp` (= BusMaster) drives: the I2cHost Request every
  target shares, field for field ({addr, tx, tx_len, rx, rx_len,
  ReplyTo<I2cDone>, speed}; one tenure = write, read, or write-then-read
  on a repeated START; the empty request is the address probe),
  asynchronous whenever the wire moves, smart mode always on, the i2c_*
  status vocabulary on the wire (nack_addr for any address nobody
  acknowledged, nack_data for a refused written byte, arb_lost,
  bus_error), per-speed register pairs cached with `speed_ok()` and the
  refused-not-slowed rule (a speed the core cannot make, and an address
  outside 0x08..0x77, are answered `i2c_rejected` inside `start()`, the
  synchronous completion, delivered through the arbiter). Every tenure rides the byte pump, one
  interrupt a byte; there is no DMA phase ([dmac.md](dmac.md), erratum
  1.10.4). `configuration(speed)` is the resource
  configuration the engine runs, what a caller that reconfigures the
  resource behind it starts from. `unstick()` - nine open-drain pulses
  and a Stop by hand, which leaves a healthy wire untouched (SDA read
  first; zero pulses is the answer and the action) - and `recover()`,
  the init() tail re-run from the cached configuration: what a timed I2cBus calls on a tenure that never answered,
  and the only way out of a START parked behind a phantom hold on this
  silicon (the release does not fire it - letter g). recover() fixes the peripheral;
  unstick() fixes the wire; neither replaces the other. No placement
  parameter: the event entry is inline in the vector that binds it, and
  a program that wants it in SRAM places that vector in `.fast_text`
  ([platform.md](platform.md), "A handler in SRAM"); the entry as a
  util/placement.hpp `InRam` instance behind a vector in the flash was
  measured and declined (below).
- **`I2cClient<n, pads, generator>`** - the polled surface plus the ISR
  body (the SpiClient position: a client is a protocol and the
  protocol is the application's), with the erratum discipline built
  in: `answer_address()` sweeps 1.17.11's leftovers and arms
  1.17.22's `first_drdy()` gate; `take()`/`give()` ride CTRLB.CMD 0x3
  with the acknowledge action.

## How to use

    using I2cHw = brio::I2cHost<3, my_pads>;          // generator 0 on a clean wire
    using I2c = brio::I2cBus<I2cHw, P>;
    // main: I2cHw::init(clock, measured_rise_ns); kernel.init_all(); ...
    extern "C" void SERCOM3_Handler() {
        if (I2cHw::isr()) { brio::post<I2c>(brio::TransferDone{I2cHw::status()}); }
    }
    // from an AO: {addr, tx spans as lend<Lease::reply>(...), rx, reply,
    // speed} posted to I2c; the I2cDone reply carries i2c_ok or the
    // wire's own answer (i2c_nack_addr is the address-scanner's probe
    // result).

## The host's cost, measured (bench_samc letter i)

SERCOM3 on PA22/PA23 at 48 MHz against an AVR64EA48 running
`twi_peer` as a client at 0x2C on the desk's 5 V node (its CLK_PER the
board's 20 MHz quartz); the SCL period is
the register pair's on this node (481, 121 and 49 cycles at 100 kHz,
400 kHz and Fm+, 166 ns rise); `wire` is the tenure's SCL rising edges
times that period. THE PEER IS A POLLED CLIENT and stretches every byte
by its own turnaround - about 300 cycles a frame at 400 kHz, which a
255-byte read reads almost alone (x 1.25) - so the host's own share is
the difference between the columns, and its CPU is `busy` and `isr`.
Best of 8, CLK_CPU cycles; `plain` is the engine with the thread
spinning on the completion edge and no stamp in the vector, `metered`
the same engine with the thread idling masked between the vector's
edges and every entry stamped (56 cycles an entry inside `isr`):

| tenure | wire | plain | metered: wall (irq, isr, busy) |
|---|---|---|---|
| 1-byte write, 400 kHz | 2299 | 3142 | 3612 (2, 531, 1446) |
| 1 + 1 register read, 400 kHz | 4598 | 5944 | 6415 (4, 933, 2304) |
| 16-byte write, 400 kHz | 18634 | 24270 | 24823 (18, 3376, 8055) |
| 16-byte read, 400 kHz | 18634 | 23449 | 26037 (17, 2795, 6918) |
| 255-byte read, 400 kHz | 278905 | 349971 (x 1.25) | 388644 (263, 40837, 100492) |
| 255-byte write, 400 kHz | 278905 | 362506 (x 1.30) | 363530 (263, 48309, 107947) |
| 255-byte read, 100 kHz | 1108705 | 1136956 (x 1.02) | 1168892 (279, 41477, 105564) |
| 255-byte read, Fm+ | 112945 | 194360 (x 1.72) | 207234 (260, 40717, 99788) |

What the rows say:

- THE PUMP'S BYTE: 183 cycles an entry for a written byte and 156 for
  a read one at 400 kHz (isr over entries, stamps included): one STATUS
  snapshot, the SYSOP wait (a call to the resource's bounded wait), and
  on a read the DATA load alone - smart mode. A short tenure carries the
  START through `start()` (266 to 282 cycles), two interrupt entries
  and exits behind two flash wait states, and the completion edge.
- IN SRAM (`bench_samc_ram`, the vector in `.fast_text`): the pump's
  entry costs 112 cycles for a written byte and 97 for a read one at
  100 kHz (31311 over 280 entries, 27045 over 279), against 175 and 149
  from the flash behind two wait states in the same letter. At Fm+ the
  polled peer's turnaround, not the host, paces a long tenure (x 1.72
  above). The written byte's SYSOP wait stays a call, through a veneer
  back into the flash; the host takes no placement.

## Bench findings (beyond the headline)

- The command channel itself is the proof of the host: every twi_link
  command is TWO tenures of the engine under test (a write carrying
  the frame, a read collecting the answer), and letters b..k ran
  hundreds of them.
- The tenure shapes: write, read and write-then-read all i2c_ok; the
  peer saw the combined tenure as TWO address matches on one tenure
  (the repeated start from the client's side) and accounted all 24
  bytes.
- The vocabulary on the wire: a deaf peer answers the write AND the
  empty probe with i2c_nack_addr; a commanded NACK on the 3rd data
  byte comes back i2c_nack_data with the peer's own report_nacked
  beside it; a write to 0x7A and one to the general call are answered
  true and i2c_rejected by `start()`, and the peer's client, answering
  the general call, takes only the write to its own address that
  follows - one match, two bytes.
- Clock stretching is flow control: a client holding every data byte
  2 ms stretched an 8-byte tenure to exactly 16 ms, data intact,
  i2c_ok.
- The speeds: 25 tenures x 16 bytes in 40 ms at 100 kHz, 12 ms at
  400 kHz and 7 ms at Fm+ - the peer's polled turnaround rides on top of
  the divisor, a polled client pacing the bus exactly as the AVR's TWI
  measures it from the other side.
- THE SCL RATE ON THE WIRE (letter f; [../design/overview.md](../design/overview.md),
  "A loop proves the bytes, never the rate"). A tenure's wall cannot
  time the host's clock here - the polled peer stretches every byte, and
  a clock that runs fast hides under one that waits - and PA23 cannot be
  SERCOM3's pad and an EIC line at once. But its input buffer stays on
  under the SERCOM function, so a loop in SRAM samples SCL through one
  16-byte write over the IOBUS (single-cycle, 28.6.5, the pin's group in
  continuous sampling, 28.8.10), four instructions a sample, and takes
  SysTick's count at every rising edge; the mean of the rise-to-rise
  intervals within a quarter of their lower quartile is the host's unhindered
  period (the holds and stretches fall between bytes and before an
  acknowledge). With the node's 166 ns rise given to `init()`, ten
  runs against the AVR64EA48 peer: 100 kHz at 997 to 1002 thousandths of
  the rate asked (479.0 to 481.0 cycles a period), 400 kHz at 1011 to 1012
  (118.5 to 118.6), Fm+ at 933 to 934 (51.3 to 51.4) - inside the
  bracket, never more than 2 % above the rate asked and within a quarter
  below it. At Fm+ the sampler is at its limit: an edge's own path and a
  period nearly meet, so some 115 of 154 rises are caught, about half the
  intervals then a single period and the rest two or more. Every
  disturbance lengthens an interval, so the reference is the LOWER
  QUARTILE, not the median - which fell on the doubled intervals in 4 of
  15 runs, where the quartile held in 12 of 12 - and the reading is the
  coarsest of the three.
- A RISE BUDGET IS NOT A MARGIN. The formula (33.6.2.4.1) subtracts the
  rise from the period the register pair makes, so a budget longer than
  the bus's rise runs SCL FAST by the difference: the same letter with
  `init(clock, 300)` on this 166 ns node measured 100 kHz at 1011 to
  1012 thousandths and 400 kHz at 1070 to 1075 - 7 % above the rate
  asked, outside the bracket. A program states the rise its bus has.
- unstick(): 0 pulses on a healthy wire, and none SPENT - the verb
  reads SDA before driving anything - against exactly 4 with the peer
  releasing on the 4th falling edge, the same number the AVR's verb
  reports.
- THE KERNEL LETTER: four tenures queued in one dispatch came back
  through their own ReplyTo in order over the real wire, the one aimed
  at an empty address answered i2c_nack_addr IN ITS PLACE; an
  over-full arbiter rejected immediately; an idle bus voted ok on
  PrepareSleep and a busy one refused. `util/i2c_bus.hpp` and
  `util/bus_master.hpp` arbitrate this engine as written: the same
  vocabulary both buses speak on every target.
- THE TIMED BUS (letter l): a tenure into the peer's 60 ms SDA hold -
  a START parked with no clock moving, which no time-out of this silicon
  sees - came
  back `i2c_timeout` on the arbiter's 35 ms clock WITH THE WIRE STILL
  HELD, the engine recover()ed inside the same dispatch, and the same
  bus AO carried the next tenure i2c_ok after the release; a client
  stretching every byte 1 ms completed i2c_ok under the same limit
  (stretching is flow control - the limit sits above the tenure). The
  race legs (a stale timeout, a straggler completion) are the host
  suite's, deterministic on the virtual clock.

- THE CLIENT ROLE (letter h): the peer's host writes ten bytes to this
  board's client, which takes them byte-exact by polling - AMATCH, the
  direction, the data, the Stop - while the peer's host reads every byte
  acknowledged. The choreography's one trap: the tenure that arms the
  peer goes to an address NOBODY holds; aimed at the peer's own parked
  client, that client stretches it through the whole lead-in and the
  peer's held START fires on its STOP, before this board is a client.
- THE QUICK COMMAND (letter m), counted on SCL through the pad's input
  buffer: with QCEN a READ raises SB after nine rises - the address and
  its acknowledge - and the STOP is one more; the plain read clocks the
  byte in first, seventeen rises, then the owed NACK and the STOP. A
  write raises MB after nine either way. The peer saw four matches and
  four Stops, every tenure acknowledged and ended IDLE.
- THE ADDRESS-MATCH WAKE (letter n): this board's client asleep, the
  peer's host writing ten bytes to it 30 ms after the bus went Busy. In
  IDLE0, and in STANDBY with CTRLA.RUNSTDBY, the match is the one
  wake-up - SERCOM3's line the pending one, the RTC backstop silent -
  and the client holds SCL from the match until the polled loop answers,
  so the burst lands whole after the wake-up. Without RUNSTDBY the
  client is deaf in STANDBY (33.6.5, "all receptions will be dropped"):
  no match, the backstop ends the sleep, the peer's host reads an
  address NACK.
- TWO LIVE HOSTS (letter o): this board's tenure T1 to the peer's
  client, the peer's START armed behind it, and T2 started a LAG after
  T1 completes - both STARTs queued behind T1's one STOP. Below a lag of
  some 535 core cycles T2 goes first, above some 575 the peer does and
  T2 waits behind its tenure; between them the two STARTs land inside
  each other's detection and the wired AND arbitrates on the address:
  aiming twi_link's `deaf_addr` above the peer's 0x11, T2 comes back
  `i2c_arb_lost` and the
  peer's tenure completes; aiming 0x08 below it, the peer reports ARBLOST
  and T2 completes. Swept every two cycles across the crossover the
  letter finds by itself: 17 to 19 collisions of 81 attempts each way,
  in every one of fifteen runs, none wedged.

### The stack under the preemptive kernel (`test_samc_rubato_i2c`)

Measured by `test_samc_rubato_i2c`: `util/i2c_bus.hpp`'s arbiter
(pending depth 4, the per-bus timeout at 20 ms) over `I2cHost<3>` on
PA22/PA23 at 48 MHz inside Rubato, the preemptive kernel, against the
same AVR64EA48 `twi_peer`, while TC4 posts to the pack's highest object
every 97, 193 or 389 us and its dispatch spins 300 or 3000 cycles - at
97 us x 3000, 64 % of the core. Every letter runs once with TC4 silent
and six times under it; **209 verdicts, green in each of three runs of
`z`**, and letter `q` in ten runs of its own. The pack, the command channel through the arbiter and the letters
are the I2C suites' shared body (`avrxt/src/apps/rubato_i2c.hpp`, whose
header describes them), and [platform.md](platform.md) places the suite
among the preemptive kernel's. Every serve the body commands asks the
AVR peer for its stop interrupt (`twi_link.hpp`'s `flag_stop_interrupt`):
that peer sees a STOP, and so ends a counted serve on its count, only
with it.

**Where a preemption can reach this host: before the START, never inside
a tenure.** `start()` copies the request's fields, applies the speed and
ends on the ADDR store, so a dispatch above the arbiter that lands in it
delays the START: a gap between tenures. Inside a tenure every byte is
the pump's, and a late vector is a held SCL (MB and SB, 33.10.6). Letter
`x` counts it on 400 writes of 16 bytes: `start()` takes 293 cycles with
TC4 silent; under 97 us x 3000 the interrupt landed inside it 215 to 222
times in three runs, against the 25 that ticks independent of the
tenures would give - the tenure cycle locks to the pressure's period,
exactly one tick landing inside each tenure's completion dispatch (400
of the pass's 3768), which step of the cycle sets the phase not
established - and of those 48 to 54 with the bus monitor already
reading OWNER. BUSSTATE
was never OWNER at `start()`'s entry (a diagnostic build, not kept: 0 of
407 entries), so those landings fell after the ADDR store, in the
instructions that return from `start()`.

**Every shape on the pump, under every pressure as in the control.**
Letter `o`, at 400 kHz: the probe, reads of 1, 2, 3, 8 and 32,
write-then-reads of 1+1, 1+2 and 2+8, writes of 1, 2, 3, 8 and 32 -
every one `i2c_ok`, every read byte-exact, the peer's count, sum and
address matches exact. A shape's post-to-reply under pressure is the
control's plus at most a dispatch and the path behind it: the 32-byte
write 1005 to 1018 us silent and at most 1174 pressed, the probe 78 to
82 and at most 372. Letter `p`: a read of 16, a 1+4 and a write of 16
byte-exact at 100 kHz, 400 kHz and 1 MHz (the read of 16 in 1598 to
1760, 527 to 689 and 309 to 400 us against the wire's 1530, 382 and 153
- the polled peer's turnaround, not the host, sets the faster rungs),
and the peer's
commanded stretch of 1000 us a byte priced from below by its eight
holds: a read of 8 in 8908 to 9029 us, a write of 8 in 9614 to 9944 -
the read's ninth hold, at its closing NACK, falls after the read's STOP
and stretches the address of the write that follows it at once.

**The vocabulary and the arbiter.** Letter `q`: a deaf client - the
peer's client parked at 0x5D - answers the probe and a four-byte write
`i2c_nack_addr` and the probe of 0x5D `i2c_ok`, the third byte of six
refused answers `i2c_nack_data` (the peer reporting the refusal and two
address matches), the last written byte of a 2+4 refused answers
`i2c_nack_data` with no read half (the peer counting two address
matches and no byte served), the next tenure `i2c_ok` after each; a
write to 0x7A and one to the general call answer `i2c_rejected`, the
peer, answering the general call, matching only the probe that
follows.
Letter `w`: the Client above the arbiter (20 write-then-reads of 1+6,
two outstanding), a requester below it (10 writes of two) and, in a
pressure pass, the preempting object itself (8 probes from its tick):
every reply `i2c_ok`, to its requester, in its order, nothing rejected,
no timeout, the peer's tallies exact, in 7.5 to 15.1 ms a pass.

**THE HELD WIRE HAS THREE SIGNATURES HERE.** Letter `h` has the peer
hold SDA low for 60 ms. A tenure posted at the speed in force parks
behind the BUSY its monitor saw ("A tenure into a busy bus parks in
hardware" above) and is answered `i2c_timeout` by the arbiter at 20 ms
(19955 to 20129 us from its `start()`; 123 of 140 passes with a probe
posted at every tick, 18 of 21 in three runs with the flood paced as
below); in the others a probe the preempting object posted first parked
instead and timed out, the arbiter's `recover()` forced the bus state to
IDLE, and the held tenure's START went out into the held line and lost
its address: `i2c_arb_lost`, 192 to 278 us from its `start()` and
20.75 ms at most from its post. The same tenure posted at 400 kHz
after commands at 100 kHz costs `apply()` an enable that re-boots the
state machine and a forced IDLE, and is answered `i2c_arb_lost` in 46 to
61 us, every run. The third is `i2c_bus_error`: ARBLOST with BUSERR
beside it, which the engine reports as the bus error it also is (the
suite's `I2cTraits::held_bus_error`) - met once a pass in 35 of 120
passes flooded at every tick and 5 of 18 flooded paced, always on a
probe that completed after the release, in the moment the peer's TWI is
put back to its command address. Under the flood every request was
answered exactly once: while held `i2c_rejected` (the FIFO full),
`i2c_timeout` or `i2c_arb_lost`, after the release `i2c_ok`,
`i2c_nack_addr` or that bus error; a request arrived in the arbiter's
draining window, between a timeout's `recover()` and the marker after
it, in 26 of 140 passes flooded at every tick and 11 of 18 paced. After
the release the next tenure was `i2c_ok` and the arbiter voted idle with
nothing waiting, every pass.

**THE FLOOD IS PACED, BECAUSE AT 97 us x 3000 A PROBE A TICK IS MORE THAN
THIS CORE SERVES.** A flooding probe costs this core some 3500 cycles
end to end (its post, the arbiter's dispatch of it, a START that loses
at once after a recovery, the completion's vector and dispatch, the
reply into the preempting object); with a 3000-cycle spin that is more
than the 4656 cycles of a 97 us period. Posted at every tick, the
objects above the Console never empty, the Console - the one object
that stops the flood - does not run while the flood lasts, and the pack
is overloaded for good: its queues drop what they cannot hold, while the
stack stays within the pack's five runs of the scheduler (the suite's
letter `u`, outside `z`, floods so for 1000 ticks: 1292 bytes of
high-water, [platform.md](platform.md) has the bound and the
measurement). The shared letter therefore posts a probe at a
tick only when the Console has turned since the last: a turn of the
lowest object says the pack above it was idle, so the flood takes only
the time the core has left (the first probe of a hold excepted, which
needs no turn). At 97 us x 3000 that is 182 to 184 probes in the hold's
60 ms, some 620 ticks (the lighter passes keep a probe at more ticks:
522 to 549 at 97 us x 300); the suite paints and reads the stack around every
pass: 1060 to 1564 bytes over every letter, `h` included (1164 to 1292
there).

**Dark** (letter `i`): every object idle, the arbiter's vote for the
sleep, no time event armed, TC4 stopped, the host between tenures with
ERROR alone armed and no flag standing, its line enabled and not
pending, the bus monitor IDLE and PA22 and PA23 reading high on their
pads.

## Not covered yet

Driver gaps (not built), each with its reason:

- **The DMA phases, on the host or the client, and the host's use of
  ADDR.LEN**: declined - erratum 1.10.4 ([dmac.md](dmac.md)); the pump
  closes a read by smart mode.
- **The 4-wire PINOUT** (CTRLA.PINOUT, an external transceiver's
  shape): no user and no transceiver on the bench.
- **This chip as the bus's target, served from an active object, under
  the preemptive kernel.** A target whose bytes come from an object (its
  vector posting "the host reads" and leaving SCL held, a dispatch giving
  the byte) is the one place a held-off reader would stretch the clock;
  no util service serves a target that way (`I2cClient` is served from
  its vector), and the desk peer, the AVR64EA48's `twi_peer`, has no
  `host_read` to read from this board's target. Born with the first util
  target object.
- **High-speed mode**: REFUSED, because errata 1.17.7 and 1.17.9 break
  its repeated starts with no workaround.
- **10-bit HOST addressing**: the register surface only - the Request
  is 7-bit on every target, and `design/i2c-bus.md` records that the
  arbiter's descriptor has no shape for it.
- **The general call as a HOST verb**: `start()` refuses address 0x00
  with every reserved address ([../design/i2c-bus.md](../design/i2c-bus.md));
  a broadcast is a feature of its own, born with its first user.

Implemented but not bench-verified:

- **The client's general call** (`general_call`, ADDR.GENCEN): written and
  read back, and taken on the wire no longer - every brio host refuses
  address 0x00, the desk partner's included. A controller that sends a
  general call would measure it.

- **The engine against the naive shape and the data sheet's own polled
  loop** (33.6.2.4, smart mode off) on this desk: both are scratch
  programs beside `bench_samc`, and the AVR64EA48 peer paces every
  tenure with its own turnaround, so their columns are owed against it
  before the comparison is stated again.
- **XOSC32K as the time-outs' 32 kHz source**: the letter runs on a
  FREQM-weighed OSC32K with the factory trim; the crystal source needs a
  32.768 kHz crystal, which the bench board has not got fitted (the
  board's page) - a board with one would measure it.

Stated, not enforced: `SercomPadPin`'s pin-reaches-pad claim and table
6-7's I2C-capable list both remain the caller's obligations.
