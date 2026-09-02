# SERCOM I2C (SAM C21)

> **PROVISIONAL.** Both register sets are built and BOTH roles are
> bench-verified end to end against a real second board - the client
> first against the AVR peer's bit-banged host (the bundle desk), then
> in full against the samc peer's real 100 kHz host on the clean pair,
> where the whole suite runs with BOTH cores at 48 MHz and
> fast-mode-plus on the wire. util's bus vocabulary runs over the host
> engine unchanged. What remains open is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 33 (over
the shared SERCOM ch. 30) and errata DS80000740S 1.17.x. Driver:
`samc/i2c.hpp` over `Sercom<n>`'s instance facts (sercom.md owns the
address ladder; this campaign added `i2cm_regs()`/`i2cs_regs()` and
`gclk_slow_id()` beside the SPI view). Family fixture
`test/family_samc/i2c.cpp` + six negatives; bench suite `test_samc_i2c`
(11 letters, 47 verdicts, two-board: the peer board runs `twi_peer` -
the avrdx original or its samc port, one `twi_link.hpp` wire format
between them - commanded in band over the bus under test).

## THE HEADLINE: this peripheral has no input filter, and the WIRE
## decides everything - measured from both sides now

The C21's I2C logic - the host's bus monitor and the client's
Start/Stop/address machinery alike - samples SDA and SCL on
GCLK_SERCOMx_CORE with NO glitch filter. The AVR's TWI filters its
inputs (its SMBus-grade suppression is why the AVR campaign ran this
very node at 1 MHz); the C21 believes every nanosecond of the wire.
On a clean short bus that is free speed. On THIS desk - the I2C pair
rides a SEVEN-WIRE BUNDLE between the boards - per-edge crosstalk
(~100 ns class, SCL's driven edges coupling into the released SDA)
reads as false Start/Stop conditions, and the consequences were
measured from three sides:

- **The host's glitch wall is a CORE-RATE wall, not an SCL wall.** A
  hand-driven tenure (SWD only, no driver code in the loop) dies with
  BUSERR+ARBLOST on its FIRST address at 48, 24 and 12 MHz core - at
  every SCL rate tried, including 25 kHz - while a 6 MHz core is clean
  six-for-six at SCL 400 kHz, and at a 32 kHz core the whole address
  was watched crossing the wire pin by pin and the peer ACKed it. Same
  registers, same wire, only the sampling rate moved: the fine core
  SEES the glitches, the coarse one steps over them.
- **The suite therefore runs SERCOM3's core from generator 6 = OSC48M/8
  = 6 MHz** - a notch above its top SCL and no more - and `I2cHost`
  grew the two knobs that make that expressible: the `generator`
  template argument (which the driver already had) and `init(clock,
  rise_ns, core_hz)`'s stated core rate (the freqm reference_hz
  pattern: a divided generator's rate is the caller's claim). Fm+
  (1 MHz) needs a 12 MHz core and is therefore UNREACHABLE on this
  desk: `speed_ok()` says so and a Request naming it is answered
  `i2c_rejected` without moving a byte - never run slow in silence.
- **The client cannot dodge the wall by slowing down, because it does
  not own the edges.** As a CLIENT the same silicon matched a
  bit-banged address (seconds per edge, driven from the AVR's PORT over
  UPDI) perfectly - AMATCH up, SCL stretched, registers identical to
  the failing case - and stayed DEAF to the peer's real 100 kHz host at
  BOTH 6 and 48 MHz core, while that host read a clean address NACK
  (MSTATUS 0x72, the AVR campaign's own nobody-home signature). A
  client must follow foreign edges wherever they land; per-edge
  glitches reset its machinery every time.
- **AND THE FIX WAS MEASURED, NOT JUST NAMED - with its confound
  stated.** The SAM-SAM desk wired the I2C pair SHORT AND SEPARATE
  (bench.md), and the wall went with the old wiring: the whole suite
  runs with BOTH ends' cores at 48 MHz -
  the samc peer's client serves the command channel at the very rate
  that was stone deaf on the bundle, the DUT's client letter takes the
  foreign 100 kHz host's burst byte-exact and TIGHTENS BACK into the
  data verdicts it used to decline, and fast-mode-plus (unreachable at
  the bundle's forced 6 MHz core) runs 25 tenures x 16 bytes in 6 ms,
  all i2c_ok - this stratum's first Fm+ on a wire. TWO KNOBS CHANGED
  TOGETHER, though, and the experiment does not split them: the new
  pair is both shorter AND out of the bundle. Since the identified
  aggressor was largely SCL's own edges coupling into the released SDA
  - and SDA and SCL are still adjacent, being a pair - the LENGTH of
  the parallel run (which mutual capacitance scales with) is plausibly
  the dominant knob and the bundle its multiplier. What is measured:
  the long bundled pair dies above a 6 MHz core; the short separated
  pair is clean at 48. The split between length and separation is not.

## What the silicon does

**Two register sets, really different.** Unlike the SPI's SPIM/SPIS
pair, I2CM and I2CS differ at the same offsets (the client has no BAUD
and no bus state; AMATCH/DRDY/PREC against MB/SB; different STATUS
bits) - so `samc/i2c.hpp` carries TWO resources, `I2cm<n>` and
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
respond, the AVR TWI's own design. STATUS.CLKHOLD says so, and ERRATUM
1.17.8 (live) makes that bit WRITABLE against the datasheet - writing
it corrupts the clock hold, so both resources' W1C masks exclude bit 7
BY CONSTRUCTION (`I2cmStatus::w1c_all` / `I2csStatus::w1c_all` cannot
express it).

**One wire fault raises MB AND ERROR together** (measured, the hard
way: the engine's first ISR consumed MB, left ERROR standing, and the
level stormed the vector with main starved - caught by halt-and-dump,
IPSR = the SERCOM's IRQ, INTFLAG = 0x80). The engine's every exit now
sweeps all flags and W1C statuses (`finish()`), and a flag arriving
with no tenure in flight is swept by the idle guard rather than
returned to.

**A tenure into a busy bus parks in hardware.** Writing ADDR while
another agent holds the wire parks the START until the bus idles
(33.6.2.4.2) - the AVR's held-START behaviour, met from the second
architecture. Measured with the peer pinning SDA low: the bus reads
BUSY for the hold's whole length and not one byte moves; ON THIS
SILICON the parked START did NOT fire when the hold released (the
phantom Start keeps the state machine BUSY and even the configured
INACTOUT did not walk it back) - recovery took the engine's own
re-init. The suite's timeline print is the record.

**Writing ADDR is the clear ceremony.** BUSERR, ARBLOST, LENERR and
the time-out statuses auto-clear on the next tenure's ADDR write
(33.10.7), which is why the engine starts a tenure with no ceremony.

**The SMBus time-outs police THIS HOST, not the wire - measured three
ways.** The obvious reading of 33.6.3.1 (enable LOWTOUT/SEXT and a
client that hangs the bus becomes a status) is OVERTURNED at the
bench: 80 ms of client stretch under both enables completes i2c_ok
with no time-out bit ever rising - during a client's hold the host's
CLKHOLD never rises and the counters never start, which the CTRLA
descriptions themselves foretell (every remedy is "the HOST releases
ITS clock hold... a STOP will automatically be transmitted", a STOP
that is physically impossible under a client's hold). What they DO
bound is the host's OWN unserviced hold: MB left unserviced trips
LOWTOUT at 30 ms measured (the 25..35 ms window to the letter), with
the chapter's exact signature - STATUS.LOWTOUT + BUSERR and
INTFLAG.ERROR beside the still-standing MB. TWO OPERATIONAL RULES the
chapter does not state: THE COUNTER ARMS AT configure() - the enables
left in CTRLA by an earlier configuration time nothing until a fresh
disable/write/enable cycle - and A WRONG-RATE METER IS MUTE, not
scaled: with GCLK_SERCOM_SLOW at 48 MHz the same hold never trips at
all (a clock-domain limit), so a design that enables these time-outs
must WEIGH its slow clock (the suite prices its OSC32K meter on FREQM,
32.59 kHz with the factory trim). CONSEQUENCE FOR THE ENGINE'S USERS:
a bus hung BY A CLIENT stays software's to bound (a deadline, then
unstick()/re-init) - the hardware time-outs bound only this host's own
software, a case a live ISR never produces.

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
  instead of stretching for a byte nobody wants; born with the samc
  peer, the first thing in this stratum to SERVE reads).
- **`i2c_baud_for(gclk_hz, scl_hz, rise_ns, fast_plus)`** - the
  chapter's own formula solved for BAUD/BAUDLOW, the RISE TIME an
  argument exactly as on the AVR (a budget that ignores it lands T_LOW
  under the specification floor), the Fm+ 1:2 split per the chapter's
  note, rounding that never lands above the request, and
  `i2c_scl_hz()` as the readback. Pinned by static_asserts.
- **`I2cHost<n, pads, generator>`** - the engine `util/i2c_bus.hpp`
  (= BusMaster) drives: the avrdx TwiHost Request verbatim ({addr, tx,
  tx_len, rx, rx_len, ReplyTo<I2cDone>, speed}; one tenure = write,
  read, or write-then-read on a repeated START; the empty request is
  the address probe), ALWAYS asynchronous, one interrupt per byte
  (MB/SB), the i2c_* status vocabulary on the wire
  (nack_addr/nack_data/arb_lost/bus_error), per-speed register pairs
  cached with `speed_ok()` and the refused-not-slowed rule, and
  `unstick()` - nine open-drain pulses and a Stop by hand, the avrdx
  verb's twin, which leaves a HEALTHY wire untouched (SDA read first;
  zero pulses is the answer and the action).
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
- The speeds: 25 tenures x 16 bytes in 41 ms at 100 kHz and 14 ms at
  400 kHz (the peer's polled turnaround rides on top of the divisor -
  the AVR campaign's own pacing, seen from the other side).
- unstick(): 0 pulses on a healthy wire (and no pulses SPENT - the
  early SDA read is the fix the first version needed), exactly 4
  against the peer releasing on the 4th falling edge, the AVR twin's
  own number.
- THE KERNEL LETTER: four tenures queued in one dispatch came back
  through their own ReplyTo in order over the real wire, the one aimed
  at an empty address answered i2c_nack_addr IN ITS PLACE; an
  over-full arbiter rejected immediately; an idle bus voted ok on
  PrepareSleep and a busy one refused - NOT ONE LINE of
  util/i2c_bus.hpp, util/bus_master.hpp or kernel/ changed. The bus
  vocabulary now has its cross-architecture proof on BOTH buses.

## Not covered yet

- Multi-host arbitration with both hosts LIVE - and the two-node C21
  desk has a REASONED wall in front of it: the AVR campaign's
  deterministic race armed both held STARTs against a bit-banged Busy
  and released them on one edge, but ON THIS SILICON a START parked
  behind a phantom-Start hold DOES NOT FIRE when the hold releases
  (measured, letter g's timeline) - the rendezvous primitive itself is
  absent. A real race here wants a third node (or the AVR back on the
  bus as the injector). The parked-START behaviour and the
  ARBLOST/BUSERR classification are measured; a live collision is not.
- MEXTTOEN on silicon (the host's own cumulative-extend flavour;
  LOWTOUT and SEXT are measured - see the findings - and MEXT shares
  their machinery and their host-side-only scope by the same CTRLA
  wording). XOSC32K remains the one unexercised 32 kHz source (board
  D's crystal is its occasion); the letter runs on a FREQM-weighed
  OSC32K with the factory trim.
- Smart mode and quick command on silicon; DMA (the trigger codes are
  published; engines wait for a user); 4-wire PINOUT; High-speed mode
  (refused - errata); 10-bit HOST addressing (register surface only);
  sleep/RUNSTDBY (the power pass owns the address-match wake).
- `SercomPadPin`'s pin-reaches-pad claim and table 6-7's I2C-capable
  list both remain the caller's stated obligations.
