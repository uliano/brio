# SERCOM SPI (SAM C21)

> **PROVISIONAL.** Both roles are built and bench-verified against a
> real second board (an AVR128DB48 peer - the cross-architecture bench),
> and util's bus vocabulary runs over the host engine unchanged. What is
> deliberately still open is listed under "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 32 (over the
shared SERCOM ch. 30) and errata DS80000740S 1.17.x. Driver:
`samc/spi.hpp` over `Sercom<n>`'s instance facts ([sercom.md](sercom.md)
owns the address ladder, the clocks, the NVIC line and the DMAC trigger
codes - one table, shared by every personality). Family fixture
`test/family_samc/spi.cpp` + seven negatives; bench suite
`test_samc_spi` (7 letters, 61 verdicts, two-board: board A runs
`spi_peer`, commanded in band over the bus under test with the AVR
campaign's own `spi_link.hpp` protocol - one source file, two
architectures).

## What the silicon does

**The two register views are one register set.** The device header
declares `sercom_spim_registers_t` and `sercom_spis_registers_t` with
identical offsets and field positions; the driver uses SPIM for both
roles and the role is CTRLA.MODE and nothing else. That equivalence is
static_asserted field by field at the bottom of spi.hpp, so a device
pack that ever separated them fails the build.

**DOPO is a triple, not a pad.** CTRLA.DOPO's four codes each fix a
whole (DO, SCK, SS) assignment (32.8.1); the three signals cannot be
placed independently. And WHICH SIGNAL IS WHICH DEPENDS ON THE ROLE
(table 32-2): DO is MOSI on a host and MISO on a client, so one fixed
four-wire harness is a host on row 0x0 and a client on row 0x2 - two
different DOPO codes, not one code with directions flipped. The bench
runs both roles over the same seven wires. The header's enumerator
naming trap: `DOPO_PAD1_Val` names CODE 0x1, which puts DO on PAD[2] -
the same trap TXPO has.

**Enable protection and the three SYNCBUSY bits.** CTRLA, CTRLB, BAUD
and ADDR are enable-protected (32.6.2.1): a store while the instance
runs is DISCARDED in silence - measured (a DIPO flip and a BAUD store
under a running host change nothing). SYNCBUSY has SWRST, ENABLE and
CTRLB (32.8.8); a CTRLB write while SYNCBUSY.CTRLB stands is an APB
error, so every CTRLB write in the driver waits first. ENABLING THE
PERIPHERAL ITSELF RAISES SYNCBUSY.CTRLB (32.8.2's RXEN note): the
receiver is really up only once that second synchronization clears,
which is why `Spi<n>::enable()` waits out both.

**RXC is the byte edge, DRE is a condition.** RXC rises when a character
has been fully shifted in - on a full-duplex bus that is exactly "one
character moved, both ways" - and reading DATA is both the capture and
the acknowledgement. DRE merely means the transmit buffer is free. The
host engine arms RXC and never DRE: one interrupt per character.

**A client's DATA write needs three SCK cycles to reach the shifter**
(32.6.2.6.2), and those cycles ELAPSE ONLY WHILE SCK RUNS. So an answer
written in the inter-character gap - where a poll loop reacting to RXC
lands - matures mid-character and reaches the wire ONE CHARACTER LATE.
Measured: the host reads the preloaded first answer exactly, then the
whole stream slips by one (11 of 12 mismatched). The working client pump
is ONE AHEAD: preload b0 (CTRLB.PLOADEN puts a write made while SS is
high straight into the shifter, 32.6.3.2), park b1 in DATA at once, and
on every received character write the next-plus-one. So driven, the peer
read this client's 12-character answer stream byte-exact from the first
character.

**A mode change is a CPOL flip on the wire.** Reprogramming a host from
mode 0/1 to mode 2/3 moves SCK's idle level, and that transition is one
extra edge: landed inside an open select window, a selected client
counts it into the character - measured as an exact ONE-BIT SLIP in both
directions, on modes 2 and 3 only. The engine's own `start()` applies
the request's mode BEFORE asserting the request's cs, so an
engine-owned window never sees it; a caller framing CS by hand must call
`SpiHost::prime()` first (that caller is how the trap was found).

**Hardware SS frames a character, not a transaction.** CTRLB.MSSEN
raises SS "for a minimum of one baud cycle between each data sent"
(32.6.3.5) - measured: four rising edges on the SS pad over a
four-character transfer. A multi-byte protocol frame therefore cannot
ride hardware SS at all; the engine's chip select is an ordinary GPIO
carried in the Request (32.6.3.3's "host with several clients").

**The receive buffer is two deep.** Five characters clocked with DATA
never read: the first two survive, STATUS.BUFOVF rises (with CTRLA.IBON
it rises at the overflow; otherwise it travels with the data), and
INTFLAG.ERROR beside it. Both write-one-to-clear.

**Loop-back is real and goes through the pad.** DIPO may name the pad
DO drives (32.6.3.4): a host then reads its own transmit line back -
32 of 32 bytes identical on the bench, and nine-bit characters
(CTRLB.CHSIZE = 9) loop 0x1FF and 0x100 back whole.

**There is no event surface.** 32.5.6 and 32.6.4.3 are both "Not
applicable" - the first peripheral in this stratum with nothing to
publish under the EVSYS ruling. There is no runtime host demotion
either: the AVR's low-SS-demotes-a-host has no counterpart here (the
role is CTRLA.MODE, written disabled); what the silicon offers instead
is CTRLB.SSDE, a client that flags/wakes on the select edge.

**Errata at rev F (E/G/J row).** 1.17.16 (SWRST inert while ENABLE = 0)
is marked on every revision and DID NOT REPRODUCE in SPI mode: SWRST
from the disabled state resets the block with its synchronization
completing, and even with the core clock channel really disconnected the
reset lands (bounded) once the channel returns. `Spi<n>::reset()` keeps
the enable-first discipline anyway - other SERCOM modes are unmeasured
and the cost is one enable. 1.17.19 (DBGCTRL cleared by SWRST) also DID
NOT REPRODUCE: DBGCTRL read 0x1 across SWRST from both states, exactly
as 32.6.2.2 promises; configure() still writes DBGCTRL last, which is
correct under either answer. 1.17.3 (a preloaded client's first
character is a dummy unless the host holds SS low for the whole
transmission) and 1.17.20 (a preloaded character costs standby current)
are LIVE, cannot be fixed on this side of the wire, and are stated on
`SpiConfig::preload`. 1.17.1 (spurious SSL at enable with SSDE + RXEN)
is REVISION B ONLY - named in the driver precisely because it is the one
item a reader would apply without checking the row.

## Types and verbs

- **`Spi<n>`** - the resource: the whole register surface in one typed
  view (both roles), `SpiConfig` + `configure()`/`configure<cfg>()` with
  every Reserved code and every role-crossed knob refused
  (`spi_config_valid()` names whose rule each clause is), the
  synchronization waits bounded, `reset()` with the erratum discipline,
  the flags, STATUS, DATA. The baud arithmetic is pure and pinned by
  static_asserts: BAUD = f_ref/(2 f_SCK) - 1, eight bits, rounded so the
  produced rate is never above the request.
- **`SpiPads`** - the four pads AND the four pins (the `UartPads`
  precedent: the pad side is checked exactly - `spi_dopo_for()` answers
  with the one DOPO row or nothing - and the pin side as far as a header
  can). Role legality is checked IN THE ROLE (`spi_role_probe`), because
  the same harness is legal as a host and illegal as a client on the
  same row.
- **`SpiHost<n, pads, generator>`** - the engine `util/spi_bus.hpp` (=
  `BusMaster`) drives: the avrdx Request shape (cs/dc `PinRef`s,
  two-phase cmd + full-duplex data, `Borrowed<..., Lease::reply>` spans,
  `ReplyTo<SpiDone>`, per-request BAUD value and `SpiMode`, `polled`
  completion style), `start()`/`isr()` per the bus_master contract,
  `baud_for()`/`sck_hz()` and the optional bus-wide SCK ceiling that
  `rebase()` re-resolves, `prime()` for callers that frame CS by hand.
  Configuration changes are cached: a run of requests to one device
  costs no disable/enable pair at all.
- **`SpiClient<n, pads>`** - the polled surface plus ISR bodies:
  preload, SSDE, address recognition (FORM = 0x2 with AMODE/ADDR),
  `drive_output()` for a dark listener on a shared harness,
  `selected()` as a live pad read (there is no status bit for SS).

## How to use

A device client on the arbitrated bus (identical to the AVR shape - the
point of the campaign):

    using SpiHw = brio::SpiHost<1, my_pads>;
    using SpiBus = brio::SpiBus<SpiHw, P>;
    // in main: SpiHw::init(clock); kernel.init_all(); ...
    // ISR glue:
    extern "C" void SERCOM1_Handler() {
        if (SpiHw::isr()) { brio::post<SpiBus>(brio::TransferDone{brio::spi_ok}); }
    }
    // from an AO: fill a Request (cs, buffers as lend<Lease::reply>(...),
    // baud = *SpiHw::baud_for(6'000'000), mode), post it to SpiBus, and
    // consume the SpiDone reply.

A client answering a stream (the one-ahead pump):

    Peer::init(clock, {.preload = true});
    Peer::write(b0);            // into the shifter, while SS is high
    Peer::write(b1);            // parked in DATA
    // per received character (poll() or the RXC body): write b(k+2).

## Bench findings

- The seven-wire cross-architecture bench: SAM SERCOM1 function C
  (PA16 MOSI, PA17 SCK, PA18 SS, PA19 MISO) against the AVR peer's SPI0
  ALT1 (PE0-PE3), both boards at 5 V. The same wires carry the SAM as
  host (DOPO row 0x0) and as client (row 0x2).
- All four transfer modes x both bit orders byte-exact in both
  directions; a deliberate DORD mismatch is an EXACT two-way bit
  reversal at both ends.
- The rate ladder against the crystal: every BAUD really clocks its
  bits, never short (64-character bursts at 93 kHz to 4 MHz, polled-pump
  overhead 2.6..6.5 us per character, falling with rate).
- Back-to-back characters (no inter-byte gap - one engine request) are
  exact against the peer to 500 kHz always and to 1 MHz on most runs,
  and break by 2 MHz: the boundary is the PEER'S POLLED TURNAROUND
  (5..9 us per byte; a 1 MHz character is 10 us - marginal by
  construction, observed landing on both sides of it - and a 2 MHz one
  is 4 us), well below its CLK_PER/6 = 4 MHz electrical ceiling. The
  AVR campaign's own suites reach that ceiling only by spending an
  inter-byte gap.
- The kernel letter: four requests queued in one dispatch come back
  through their own ReplyTo in order; an over-full arbiter answers
  bus_rejected immediately; an idle bus votes ok on PrepareSleep, a busy
  one refuses. NOT ONE LINE of util/spi_bus.hpp, util/bus_master.hpp or
  kernel/ changed.

## Not covered yet

- DMA-driven SPI (the trigger codes are published by `Sercom<n>`;
  engines wait for a real streaming user - the UART's precedent).
- Sleep: RUNSTDBY on silicon, SSDE as a wake source, erratum 1.17.20's
  standby cost (the power pass owns them).
- On silicon: SSDE/SSL, address recognition (FORM = 0x2 - refusals are
  compile-checked, matching is not bench-verified), 9-bit characters
  through a real two-board transfer (loop-back only), IBON = 0's
  travelling overflow flavour.
- Erratum 1.17.3's dummy-first-character (needs a host that RAISES SS
  mid-transmission on purpose; the peer holds it low, correctly).
- `SercomPadPin`'s pin-reaches-pad claim is still the caller's
  (sercom.md's open device-table question, unchanged here).
- THE PEER'S SELECT-WAIT WEDGE, neutralized but not explained: the
  original `run_exchange` opened its window by spinning on the select
  READ, and about once in five z-runs the peer entered a persistent
  state (until board A reset) where that read never fired while its SPI
  HARDWARE demonstrably shifted - the host read the preloads and then
  the echo, the software window read nothing. On the wire the select
  was real (driven low over SWD, the wedged peer's own status read it
  LOW), so the wedge lived between the pad and that wait. The exchange
  loop now polls RXC directly (a byte can only arrive while selected,
  and apply_cfg has just cleared the buffers - the wait added only the
  mechanism that failed) and samples the select purely as TELEMETRY the
  Report carries (aux1..aux3); ten consecutive z-runs since, no
  recurrence. WHAT the state was remains unhunted - an avrdx-side
  question (spi_peer + the pin read path), waiting for an AVR bench,
  with the telemetry in place to catch it in the act.
