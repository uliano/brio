# SERCOM I2C (SAM C21)

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 33 (over
the shared SERCOM ch. 30) and errata DS80000740S 1.17.x. Driver:
`samc21/i2c.hpp` over `Sercom<n>`'s instance facts (sercom.md owns the
address ladder; `i2cm_regs()`/`i2cs_regs()` and `gclk_slow_id()` sit
beside the SPI view). Family fixture `test/family_samc21/i2c.cpp` +
seven negatives (the per-bus timeout adds an
eighth at the util level: an engine without `recover()` refused);
bench suite `test_samc_i2c`, bench letter `bench_samc` i
(12 letters, two-board: the peer board runs `twi_peer` on an AVR128DB48
or on a second SAM C21, one `twi_link.hpp` wire format between them -
commanded in band over the bus under test).

## THE HEADLINE: this peripheral has no input filter, and the WIRE
## decides everything

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
while enabled (33.6.6). Every such store in the driver WAITS FIRST
(the sdadc discipline).

**The bus state machine and its exits.** A freshly enabled host is
UNKNOWN and leaves it by a Stop, by the INACTOUT time-out, or by
software forcing IDLE (33.6.2.3). Measured: with no time-out the state
sits in UNKNOWN until `force_idle()`; whether INACTOUT walks it out BY
ITSELF came out BOTH WAYS on this bench (IDLE on one run, still
UNKNOWN on the next, same code) - the suite records the observation
and relies on `force_idle()`, which the engine's init always spends.

**MB/SB hold SCL.** Both host flags stretch the clock until software
answers with DATA, ADDR, a command or a flag clear - unlimited time to
respond. STATUS.CLKHOLD says so, and ERRATUM 1.17.8 (live) makes that
bit WRITABLE against the datasheet - writing it corrupts the clock
hold, so both resources' W1C masks exclude bit 7 BY CONSTRUCTION
(`I2cmStatus::w1c_all` / `I2csStatus::w1c_all` cannot express it).

**One wire fault raises MB AND ERROR together**: an ISR that consumes
MB and leaves ERROR standing lets the level storm the vector with main
starved - measured, with IPSR reading the SERCOM's IRQ and INTFLAG
0x80. So every exit of the engine sweeps all flags and W1C statuses
(`finish()`), and a flag arriving with no tenure in flight is swept by
the idle guard rather than returned to.

**A tenure into a busy bus parks in hardware.** Writing ADDR while
another agent holds the wire parks the START until the bus idles
(33.6.2.4.2). Measured with the peer pinning SDA low: the bus reads
BUSY for the hold's whole length and not one byte moves; ON THIS
SILICON the parked START does NOT fire when the hold releases (the
phantom Start keeps the state machine BUSY and even a configured
INACTOUT does not walk it back) - the way out is the engine's own
re-init, and the suite prints the timeline.

**Writing ADDR is the clear ceremony.** BUSERR, ARBLOST, LENERR and
the time-out statuses auto-clear on the next tenure's ADDR write
(33.10.7), which is why the engine starts a tenure with no ceremony.

**The automatic length, as this silicon runs it** (ADDR.LEN + LENEN,
33.10.9, 33.6.4.1.2; every point traced flag by flag against the peer at
100 kHz; the host does not use it, the table below):

- A WRITE under LEN = n raises MB after the address and after every data
  byte but the n-th, each with the clock held for the next DATA; after
  the n-th it sends the STOP by itself and NO FLAG RISES - no MB after
  the last byte, none after the STOP, the bus simply going IDLE. With
  LEN = n + 1 the MB after byte n stands with the clock held and its
  acknowledge read, ready for a STOP or a repeated START.
- A NACK under LEN - the address's or any data byte's before LEN - is
  LENERR + RXNACK + ERROR with NO MB, and the STOP already sent: one
  signature for an absent client and for a refused byte.
- A READ under LEN = n raises SB for each byte; the DATA read of the
  n-th sends the NACK and the STOP by themselves. SB then STANDS for a
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
read sends ACKACT and clocks the next byte in WHATEVER ACKACT SAYS -
with ACKACT = NACK the read sent the NACK and still read one more byte
(measured), unlike the AVR TWI's smart mode, which sends nothing on a
NACK. So the last byte is closed by the STOP command with its NACK while
SB holds the clock, synchronized, and only then read (33.10.10's
"reading the last data byte after the stop condition has been sent").
And ACKACT PERSISTS: a read's closing NACK leaves it set, and the next
read's first DATA read would NACK that byte - every reading phase puts
ACK back first.

### What the silicon offers for the cost, and what the engine takes

| item | where | the engine |
|---|---|---|
| MB / SB, one interrupt a byte with the clock held | 33.6.2.4, 33.10.6 | THE PUMP, every tenure. One STATUS snapshot a handler, the SYSOP wait inline (one load when the last operation synchronized long ago) |
| smart mode | 33.6.3.2 | USED, always: a pumped read byte is one DATA load, no command and no SYSOP wait |
| the two DMA requests (TX, RX) | 33.6.4.1.2, table 33-2 | DECLINED: erratum 1.10.4 ([dmac.md](dmac.md)) - this stratum drives the DMAC for the Uart's transmitter alone |
| ADDR.LEN + LENEN | 33.10.9 | DECLINED: its use would be to close a DMA phase; the pump closes a read by smart mode and a write by its last MB, every byte software's anyway. The resource keeps the verb (`I2cm::start_address_len`), its behaviour measured above |
| quick command (QCEN) | 33.6.3.4 | DECLINED for the probe: an empty request costs one MB either way |
| SCLSM = 1 | 33.6.2.4.5 | DECLINED: the engine's decisions are taken before the acknowledge (SCLSM = 0), and erratum 1.17.13 refuses it beside QCEN |
| High-speed mode | 33.6.2.4.6 | REFUSED: errata 1.17.7 and 1.17.9 |
| the SMBus time-outs | 33.6.3.1 | the caller's, over `configuration()` (the finding above); the engine answers a time-out's ERROR with `i2c_bus_error` |
| INACTOUT | 33.6.2.3 | USED: 205 us, the BUSY state's escape |

**The SMBus time-outs bound a client's hold of SCL, as 33.6.3.1 reads -
when they are armed, on a slow clock weighed to 32 kHz.** Under LOWTOUTEN
and SEXTTOEN a client stretching every byte by 40 ms ends the tenure at
29 ms, inside the 25..35 ms window, and the engine answers
`i2c_bus_error` (letter j). They bound the host's OWN unserviced hold as
well: MB left unserviced trips LOWTOUT at 30 ms measured, with the
chapter's exact signature - STATUS.LOWTOUT + BUSERR and INTFLAG.ERROR
beside the still-standing MB. TWO OPERATIONAL RULES the chapter does not
state: THE COUNTER ARMS AT configure() - the enables left in CTRLA by an
earlier configuration time nothing until a fresh disable/write/enable
cycle - and A WRONG-RATE METER IS MUTE, not scaled: with
GCLK_SERCOM_SLOW at 48 MHz the same hold never trips at all (a
clock-domain limit), so a design that enables these time-outs must WEIGH
its slow clock (the suite prices its OSC32K meter on FREQM, 32.2 kHz
with the factory trim). And a third, the one that hid the first finding
for a while: `I2cm<n>::configure()` DISARMS EVERY INTERRUPT, so a caller
that reconfigures the resource behind a running engine must start from
`I2cHost::configuration()` - which keeps smart mode, the INACTOUT escape
and the rest of what the engine relies on - and the engine arms ERROR
again with every phase. What the time-outs cannot see is a wire wedged
by SDA: the START parks with no clock moving (letter g), and only the
arbiter's per-bus `timeout_ticks` (util/bus_master.hpp, letter l)
answers that with `i2c_timeout` on the kernel's clock and `recover()`s
the engine; the WIRE - unstick(), re-probing the clients - stays the
application's recovery ladder (docs/design/i2c-bus.md).

**Erratum 1.17.16 NOT REPRODUCED in I2C mode either**: SWRST from the
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
AMATCH handler is the workaround's own prescription and `I2cClient` IS
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
  SERVES reads needs).
- **`i2c_baud_for(gclk_hz, scl_hz, rise_ns, fast_plus)`** - the
  chapter's own formula solved for BAUD/BAUDLOW, the RISE TIME an
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
  refused-not-slowed rule (a speed the core cannot make is answered
  `i2c_rejected` inside `start()`, the one synchronous completion,
  delivered through the arbiter). Every tenure rides the byte pump, one
  interrupt a byte; there is no DMA phase ([dmac.md](dmac.md), erratum
  1.10.4). `configuration(speed)` is the resource
  configuration the engine runs, what a caller that reconfigures the
  resource behind it starts from. `unstick()` - nine open-drain pulses
  and a Stop by hand, which leaves a HEALTHY wire untouched (SDA read
  first; zero pulses is the answer and the action) - and `recover()`,
  the init() tail re-run from the cached configuration: what a timed I2cBus calls on a tenure that never answered,
  and the ONLY way out of a parked START on this silicon (the release
  does not fire it - letter g). recover() fixes the PERIPHERAL;
  unstick() fixes the wire; neither replaces the other.
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

SERCOM3 on PA22/PA23 at 48 MHz against an AVR128DB48 running
`twi_peer` as a client at 0x2C on the desk's 5 V node; the SCL period is
the register pair's on this node (481, 121 and 49 cycles at 100 kHz,
400 kHz and Fm+, 166 ns rise); `wire` is the tenure's SCL rising edges
times that period. THE PEER IS A POLLED CLIENT and stretches every byte
by its own turnaround - about 220 cycles a frame at 400 kHz, which a
255-byte read reads almost alone (x 1.20 for the pump and the data
sheet's loop alike) - so the host's own share is the difference between
the columns, and its CPU is `busy` and `isr`. Plain bindings, the thread
spinning on the edge, best of 8, CLK_CPU cycles; `naive` is the shape
the pump is measured against (one interrupt a byte, smart mode off, the
Request copied whole), `pump` the engine; the vendor's column is the
data sheet's own polled sequence (33.6.2.4, smart mode off) against the
same peer:

| tenure | wire | naive | pump | the data sheet's loop |
|---|---|---|---|---|
| 1-byte write, 400 kHz | 2299 | 3504 | 3212 | 2623 |
| 1 + 1 register read, 400 kHz | 4598 | 6135 | 5900 | 5148 |
| 16-byte write, 400 kHz | 18634 | 26004 | 23599 | 22791 |
| 255-byte read, 400 kHz | 278905 | 374672 (x 1.34) | 336537 (x 1.20) | 336033 (x 1.20) |
| 255-byte write, 400 kHz | 278905 | 384787 (x 1.37) | 347330 (x 1.24) | 345970 (x 1.24) |
| 255-byte read, 100 kHz | 1108705 | 1180721 (x 1.06) | 1137878 (x 1.02) | 1134420 (x 1.02) |
| 255-byte read, Fm+ | 112945 | 222014 (x 1.96) | 183626 (x 1.62) | 183300 (x 1.62) |

And the CPU, metered (the stamps' 56 cycles an entry inside isr), at
400 kHz:

| tenure | naive: irq, isr, busy | pump: irq, isr, busy |
|---|---|---|
| 16-byte write | 18, 4950, 9276 | 17, 3791, 8096 |
| 255-byte write | 265, 72939, 132721 | 264, 55018, 114721 |
| 16-byte read | 17, 5043, 9151 | 17, 3020, 7341 |
| 255-byte read | 264, 79007, 137679 | 263, 45364, 103467 |

What the rows say:

- THE PUMP'S BYTE: 208 cycles an entry for a written byte against the
  naive shape's 275 (isr over entries, stamps included), 172 for a read
  byte against 299: one STATUS snapshot where the naive shape has three
  loads, the SYSOP wait inline, and on a read the DATA load alone -
  smart mode - where a command and a SYSOP wait follow it. The bus feels
  it: a 255-byte read at 400 kHz takes 374672 cycles naive and 336537
  pumped, at the peer's own pace.
- Against the data sheet's polled loop the pump is within half a per
  cent on long tenures and 590 to 750 cycles behind on one- and two-byte
  tenures - the START through `start()` (some 200 cycles), two interrupt
  entries and exits behind two flash wait states, and the completion
  edge - where the loop spends the whole tenure busy.
- IN SRAM (`bench_samc_ram`): the pump's entry costs 118 cycles for a
  written byte and 102 for a read one, against 208 and 172 from flash
  behind two wait states; at Fm+ a 255-byte write's wall drops from
  194073 to 193484, a 1-byte write's from 2062 to 1818.

## Bench findings (beyond the headline)

- The command channel itself is the proof of the host: every twi_link
  command is TWO tenures of the engine under test (a write carrying
  the frame, a read collecting the answer), and letters b..k ran
  hundreds of them.
- The tenure shapes: write, read and write-then-read all i2c_ok; the
  peer saw the combined tenure as TWO address matches on one tenure
  (the repeated start from the client's side) and accounted all 24
  bytes; a GENERAL CALL write reached the peer at address 0x00.
- The vocabulary on the wire: a deaf peer answers the write AND the
  empty probe with i2c_nack_addr; a commanded NACK on the 3rd data
  byte comes back i2c_nack_data with the peer's own report_nacked
  beside it.
- Clock stretching is flow control: a client holding every data byte
  2 ms stretched an 8-byte tenure to exactly 16 ms, data intact,
  i2c_ok.
- The speeds: 25 tenures x 16 bytes in 40 ms at 100 kHz, 12 ms at
  400 kHz and 6 ms at Fm+ - the peer's polled turnaround rides on top of
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
  intervals within a quarter of their median is the host's unhindered
  period (the holds and stretches fall between bytes and before an
  acknowledge). With the node's 166 ns rise given to `init()`: 100 kHz
  at 997 to 998 thousandths of the rate asked (480.6 to 481.3 cycles a
  period), 400 kHz at 1012 (118.5), Fm+ at 934 (51.3) - inside the
  bracket, never more than 2 % above the rate asked and within a quarter
  below it. At Fm+ the sampler is at its limit: an edge's own path and a
  period nearly meet, so some 110 of 152 rises are caught and the reading
  is the coarsest of the three.
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

## Not covered yet

Driver gaps (not built), each with its reason:

- **The DMA phases, on the host or the client, and the host's use of
  ADDR.LEN**: declined - erratum 1.10.4 ([dmac.md](dmac.md)); the pump
  closes a read by smart mode.
- **The 4-wire PINOUT** (CTRLA.PINOUT, an external transceiver's
  shape): no user and no transceiver on the bench.
- **High-speed mode**: REFUSED, because errata 1.17.7 and 1.17.9 break
  its repeated starts with no workaround.
- **10-bit HOST addressing**: the register surface only - the Request
  is 7-bit on every target, and `design/i2c-bus.md` records that the
  arbiter's descriptor has no shape for it.

Implemented but not bench-verified:

- **Multi-host arbitration with both hosts LIVE** - and the two-node
  C21 desk has a REASONED wall in front of it: the AVR's deterministic
  race arms both held STARTs against a bit-banged Busy and releases them
  on one edge, but ON THIS SILICON a START parked behind a phantom-Start
  hold DOES NOT FIRE when the hold releases (measured, letter g's
  timeline) - the rendezvous primitive itself is absent. A real race
  here wants a third node (or the AVR back on the bus as the injector).
  The parked-START behaviour and the ARBLOST/BUSERR classification are
  measured; a live collision is not.
- **MEXTTOEN** on silicon (the host's own cumulative-extend flavour;
  LOWTOUT and SEXT are measured - see the findings - and MEXT shares
  their machinery by the same CTRLA wording).
- **XOSC32K as the time-outs' 32 kHz source**: the letter runs on a
  FREQM-weighed OSC32K with the factory trim; the crystal source waits
  for the 32 kHz pass of [osc32kctrl.md](osc32kctrl.md).
- **The quick command** on silicon: written and read back, no tenure
  has used it.
- **Sleep and RUNSTDBY**, the address-match wake included: no letter
  sleeps this bus.

Stated, not enforced: `SercomPadPin`'s pin-reaches-pad claim and table
6-7's I2C-capable list both remain the caller's obligations.
