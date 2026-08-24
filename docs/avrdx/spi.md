# SPI - the serial peripheral interface (AVR DA/DB)

> **PROVISIONAL.** The chapter's register description is covered in full
> and both bench halves pass: the single-board one (routes and teardown,
> all seven bit rates measured on SCK, the data path, the transfer
> modes' idle levels, the write collision, buffer mode's four flags, the
> host demotion, both ISR bodies, a clock rebase under an SCK ceiling,
> the transfer engine) and the two-board one against a real client
> (every mode, bit order and buffering regime, the client's SCK ceiling,
> deliberate mismatches, SS mid-byte, client-side loss, a real multi-host
> demotion, the USART's Host SPI mode). What is left is a client-side AO,
> sleep, the routes this desk cannot reach and the electricals as timing.
> The list is in "Not covered yet".

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (SPI
chapter 28, PORTMUX chapter 17, electricals 39.15), errata DS80000915F
(2.11.1 and clarifications 3.5.1, 3.5.2, 3.7.3) and, for the DA parts,
DS80000882C (2.10.1). Three items shape the code:

- **DB 2.11.1, SPI1 ALT2 on 48-pin devices**: the position is
  NON-FUNCTIONAL there (rev. A4/A5; fixed in B0). The 48-pin device
  headers still list it, with MOSI on PB4 and MISO on PB5 and no SCK or
  SS position at all - so the driver refuses it on every 48-pin part of
  both families, at compile time and at run time. This is the first
  place in `avrdx/` where an erratum beats the device header.
- **DA 2.10.1, SSD with the pinless route**: with PORTMUX.SPIROUTE at
  NONE the Client Select line must be disabled (CTRLB.SSD = 1) or Host
  mode does not survive. It is listed for every DA revision and not at
  all for the DB, but the configuration it forbids - a pinless host
  still watching an SS input no pin can hold high - has no use, so it is
  refused on both families.
- **clarification 3.7.3, the timing tables**: the host's SCK ceiling is
  f_CLK_PER/2 and a client's is f_CLK_PER/6 (the chapter's own "two
  peripheral clock periods per SCK phase" would say /4; the tighter
  table wins). `spi_max_host_sck_hz` / `spi_max_client_sck_hz`.
- **clarification 3.5.2** also warns that a client in buffer mode near
  its maximum SCK may not set up data in time for the first sample edge
  of a back-to-back transfer.

Driver: `avrdx/spi.hpp`. The arbiter above it:
[spi-bus.md](../design/spi-bus.md) (`util/bus_master.hpp`,
`util/spi_bus.hpp`). Reference test: `test_avr_spi`.

## What the silicon does

One 8-bit shift register shifting out and in at the same time, a clock
generator used only in host mode, and two roles:

- **host** - a write to DATA starts a transfer; the host drives SCK,
  MOSI and (if it wants to) SS, and reads MISO;
- **client** - SS, SCK and MOSI are inputs and the host sets the pace;
  MISO is driven only while SS is low, the pad tri-stating itself when
  the client is deselected, and the SS pin's rising edge RESETS the
  client's state machine (a partial byte is lost).

Both roles run in one of two buffering regimes, chosen by CTRLB.BUFEN,
and the regime decides which of the TWO INTFLAGS layouts the same
register address carries:

| | normal mode | buffer mode |
|---|---|---|
| transmit | single-buffered: a write during a transfer is IGNORED and sets WRCOL | double: DATA + a transmit buffer, DREIF says there is room |
| receive | double-buffered: read before the next transfer ends or lose it | a two-deep FIFO plus the shifter; BUFOVF marks the loss |
| flags | IF, WRCOL | RXCIF, TXCIF, DREIF, SSIF, BUFOVF |
| clearing | IF: write one to it, or read INTFLAGS then access DATA. WRCOL: ONLY that read-then-DATA sequence | TXCIF/SSIF/BUFOVF: write one. RXCIF and BUFOVF also clear by reading DATA; DREIF clears by WRITING DATA and by nothing else |

A host with CTRLB.SSD = 0 watches its SS pin: an SS INPUT driven low by
somebody else clears CTRLA.MASTER, the instance becomes a client, and IF
(normal) or SSIF (buffer) is raised. Nothing but the application puts it
back. An SS pin configured as an OUTPUT is not watched at all (table
28-2), and with SSD = 1 the pin is free for any other use - which is
what a bus with software chip selects wants.

Seven bit rates: PRESC divides CLK_PER by 4, 16, 64 or 128 and CLK2X
halves that. CLK_PER/64 is reachable twice (PRESC DIV64 alone, PRESC
DIV128 doubled); `SpiClock` names the seven distinct divisions and
`spi_presc_bits` picks the canonical encoding.

The routes are PORTMUX.SPIROUTEA, two bits per instance. Both instances
exist on every package of the family; what varies is how many positions
a package bonds:

| | 28/32 pins | 48 pins | 64 pins |
|---|---|---|---|
| SPI0 DEFAULT | PA4-PA7 | PA4-PA7 | PA4-PA7 |
| SPI0 ALT1 | - | PE0-PE3 | PE0-PE3 |
| SPI0 ALT2 | - | - | PG4-PG7 |
| SPI1 DEFAULT | PC0-PC3 | PC0-PC3 | PC0-PC3 |
| SPI1 ALT1 | - | PC4-PC7 | PC4-PC7 |
| SPI1 ALT2 | - | refused (errata 2.11.1) | PB4-PB7 |
| NONE | pinless | pinless | pinless |

The signals are always in the order MOSI, MISO, SCK, SS. The pinless
route leaves an instance running with no pins: the shift register, the
flags and the host's SCK event generator all work.

The SPI has one interrupt vector for both layouts, one event generator
(the host's SCK level, `EvSpiSck`), no event users, and no DBGRUN or
RUNSTDBY control of its own.

## Types and verbs

| Entity | What it is |
|--------|------------|
| `SpiRoute`, `SpiSignal`, `SpiPin` | the route vocabulary; `spi_route_exists`, `spi_pin`, `spi_package_pins` are the per-package table |
| `SpiRole`, `SpiMode`, `SpiClock` | host/client, the four transfer modes (with `spi_cpol`/`spi_cpha`), the seven divisions |
| `spi_division`, `spi_sck_hz`, `spi_presc_bits`, `spi_clock_of` | the rate arithmetic, both directions |
| `spi_clock_for(clk_per, max_sck)` | the chooser: the fastest division that does not exceed a device's limit |
| `spi_max_host_sck_hz`, `spi_max_client_sck_hz` | the two ceilings of the timing tables |
| `SpiConfig`, `spi_config_valid<n>` | the whole configuration, and what this package and the errata allow |
| `Spi<n>` | the RESOURCE: `init<cfg>()`/`init(cfg)`/`release()`, enable, role and demotion, rate, mode, SSD, buffer mode, DATA, both flag sets with their clear verbs, the interrupt enables, `take_normal()`/`take_buffer()` ISR bodies, `routed()` |
| `SpiHost<n, route>` | the transfer ENGINE: `Request` descriptors, `start()`, `isr()`, an optional SCK ceiling, `rebase()` |
| `SpiClient<n, route>` | the client side: `selected()`, `preload()`, `exchange()`, the buffer-mode readbacks, the ISR bodies, `max_sck_hz()` |

Both tasks are `ClockUser`s. The engine's `rebase` recomputes the
`cs_setup_us` timing base and re-picks the division that honours its
ceiling; the client's only records the peripheral clock, because the
fastest SCK it can follow is CLK_PER/6. Neither may be rebased with a
transfer in flight.

## How to use it

**A bus, through the arbiter** - the normal way, and the only way an
application should touch a shared bus (see
[spi-bus.md](../design/spi-bus.md)):

```cpp
using SpiHw = brio::SpiHost<0>;                 // DEFAULT route
using Bus = brio::SpiBus<SpiHw, P>;
ISR(SPI0_INT_vect) { if (SpiHw::isr()) brio::post<Bus>(brio::TransferDone{brio::spi_ok}); }
SpiHw::init(clock);                             // optionally: init(clock, max_sck_hz)
brio::post<Bus>(SpiHw::Request{
    Cs::ref(), Dc::ref(), cmd, 1, data, nullptr, len,
    brio::reply_to<Me, brio::SpiDone>(),
    brio::SpiClock::div4, brio::SpiMode::mode0, /*polled=*/true});
```

**One instance by hand** - a bare host on a route of its own:

```cpp
using S = brio::Spi<0>;
S::init<brio::SpiConfig{.route = brio::SpiRoute::alt1,
                        .clock = brio::SpiClock::div16,
                        .mode = brio::SpiMode::mode3}>();
const auto in = S::transfer(0x9F);              // one polled byte
```

**A client** - the answer prepared before the host clocks it out:

```cpp
using C = brio::SpiClient<1>;
C::init({.mode = brio::SpiMode::mode0, .buffer_mode = true, .buffer_wait = true});
if (C::selected()) { const auto cmd = C::exchange(status_byte); }
```

**A device's datasheet limit in hertz** - the chooser instead of a
guessed division:

```cpp
const auto c = brio::spi_clock_for(brio::clock_hz(clock), 2'500'000u);  // XPT2046
```

## Bench findings

Measured on rev. A5 at 5 V, CLK_PER 24 MHz, SPI0 on ALT1 (PE0-PE3).
`test_avr_spi`: `z` = the single board, 148 verdicts; `y` = the same
four pins against a second AVR128DB48 running `spi_peer` as a real
client, 92 verdicts. The desk wires PORTE straight through (A.PEn -
B.PEn), so MOSI, MISO, SCK and SS are one four-wire bus between the two
boards.

**The bit rates are exact.** All seven divisions measured through the
SPI's own SCK event into a TCB frequency meter (period between rising
edges, CLK_PER ticks):

| division | SCK at 24 MHz | measured period |
|---|---|---|
| CLK_PER/2 | 12 MHz | 2 ticks |
| CLK_PER/4 | 6 MHz | 4 ticks |
| CLK_PER/8 | 3 MHz | 8 ticks |
| CLK_PER/16 | 1.5 MHz | 16 ticks |
| CLK_PER/32 | 750 kHz | 32 ticks |
| CLK_PER/64 | 375 kHz | 64 ticks |
| CLK_PER/128 | 187.5 kHz | 128 ticks |

Exact at every rate, the 12 MHz one included - a two-tick period still
reaches a TCB through the event system, though the capture ISR then
catches only about one period per byte (the minimum of a burst is the
measurement).

**A host's MISO direction is overridden, and the override is latched at
ENABLE.** With the SPI running, a PORT.DIRSET on the MISO position does
nothing to the pad; the same DIRSET performed while the instance is
disabled drives the pin, and it keeps driving across the next enable.
The driver's ordering (pins first, CTRLA.ENABLE last) is what makes a
client's MISO an output at all.

**MOSI parks HIGH between transfers** - not at the last bit sent and not
at the byte received. A stream of zero bytes therefore costs exactly one
rising edge per byte on the wire.

**The write collision.** A write to DATA during a transfer is ignored,
sets WRCOL, and does not disturb the byte in flight. The two clear
disciplines of the normal layout are NOT interchangeable: a plain store
of one to IF clears IF and leaves WRCOL standing; only the documented
read-INTFLAGS-then-access-DATA sequence clears both.

**Buffer mode.** DREIF is up on an idle transmitter, survives the first
write (straight into the shifter) and falls on the second (into the
buffer): two levels, as the chapter says. A store of one to DREIF does
NOT clear it - it follows DATA alone. TXCIF is left clear by `init` and
rises when shifter and buffer are both empty; it is write-one-to-clear,
and it is a CONDITION, so it must be cleared before a burst if it is to
mean "this burst finished". BUFOVF is not raised by the third undrained
byte, which waits in the shifter: it appears when the NEXT transfer
starts, exactly as the register description says. BUFWR changes nothing
in host mode.

**Host demotion.** An SS pin driven low as an OUTPUT does not demote
anything (table 28-2). An SS INPUT seen low does: MASTER clears, IF
(normal) or SSIF (buffer) is raised, and the instance stays a client
until the application re-arms it - mid-byte as well as between bytes.
With SSD = 1 the pin is ignored. The desk has no second driver on the SS
position, so the "seen low" half of the test is produced by the pin's own
INVEN: the pad stays an input held high by its pull-up while the
peripheral reads a low. After a demotion the instance has been a client,
and a client owns the MISO pad - a recovery therefore has to re-establish
the pin roles (an ENABLE cycle), not just write MASTER back.

**The interrupt.** One vector, one interrupt per byte in normal mode,
with the body's INTFLAGS-then-DATA read clearing IF. In buffer mode the
body must write one to TXCIF/SSIF/BUFOVF and read DATA for RXCIF, or the
vector re-enters immediately; with both halves in place a burst of eight
bytes produces eight interrupts and then silence.

**The engine.** Both completion styles move the same bytes, the chip
select comes back high after each transaction, a command phase reaches
the wire, a zero-length request completes without touching it, and a
request faster than the engine's ceiling is slowed to the ceiling. Under
a 24 -> 12 -> 24 MHz rebase a 1.5 MHz ceiling re-picks CLK_PER/16 ->
CLK_PER/8 -> CLK_PER/16 and the measured SCK stays at 1.5 MHz.

**A caveat of the self-driven-MISO technique** (this desk, not the
silicon): with MISO held HIGH by PORT and a TOGGLING pattern on MOSI,
what comes back is the pattern rather than 0xFF - the two wires run side
by side for 20 cm. It is COUPLING, not the peripheral: with a real
client driving MISO the same reads are exact at every rate inside the
client's ceiling (test `l`, `m`). A MISO held LOW is immune and a
constant MOSI is immune, so every byte-level check in the single-board
half either holds MISO low or sends a constant byte.

### Two boards, one four-wire bus

**Every combination of the client's configuration is exact, both
directions.** Four transfer modes x two bit orders x all three
buffering regimes, at CLK_PER/32, with each end checking a deterministic
stream against what the other end generated (test `l`, 24 combinations).
CPHA is therefore verified on the wire, not only as an idle level.

**The bit order is an EXACT two-way reversal.** An MSb-first host
against an LSb-first client reads the bit-reverse of every byte the
client sent, and the client reads the bit-reverse of every byte the host
sent - zero mismatches in twelve bytes each way (test `n`). This is the
check single-board instrumentation cannot make at all, since a reversed
byte carries exactly as many edges.

**In buffer mode WITHOUT BUFWR the answer stream is led by a DUMMY**,
and the dummy is the shift register's leftover: over the eight
combinations of test `l` that used the regime it measured 0x00 every
time, because `init` leaves the shifter clear. With BUFWR the client's
first write goes straight to the shifter and byte 0 of the window is
already the answer.

**The client's rate ceiling is real, and it is the errata's.** Exact at
CLK_PER/8, /16, /32, /64 and /128 (3 MHz down to 187.5 kHz) against a
client whose own CLK_PER is 24 MHz. At CLK_PER/4 (6 MHz, above the
CLK_PER/6 = 4 MHz ceiling of clarification 3.7.3 though inside the
chapter's own /4) the client miscounted all twelve bytes and the host
three of twelve; at CLK_PER/2 both directions were wrong in all twelve
(test `m`). The failure is asymmetric: the client mis-samples MOSI long
before the host mis-samples MISO.

**A mismatch in CPOL or in CPHA corrupts everything**, twelve bytes of
twelve in both directions for each (test `n`) - neither degrades
gracefully.

**The SS rising edge resets the client mid-byte, and the pad stops
driving with it.** With three clean bytes clocked at CLK_PER/128 and the
select wire raised half way through the fourth, the client kept exactly
the three and never saw the fourth (28.3.2.2.3), while the host's own
byte completed - it is the clock, and SS is only a GPIO to it. What the
host read for that byte was the top bits of the client's loaded answer
and then the released line (0xBF where the answer was 0xB6): MISO is
driven only while SS is low. Test `o`.

**What a client that never drains keeps** (gapless bursts of eight,
DATA untouched, test `p`):

| regime | retained | which |
|---|---|---|
| normal | 1 | the LAST byte - a new one overwrites the unread one |
| buffer | 3 | the FIRST two (the FIFO) plus the LAST (the shifter) |

**BUFOVF needs the client to have transmit data.** The same eight-byte
flood with the client's transmitter idle raised no BUFOVF at all - five
bytes lost in silence - and with the transmitter kept fed it raised it
(test `p`). That is 28.5.5's own clause, "if there is no transmit data,
the Buffer Overflow will not be set before the start of a new serial
transfer", and it means a receive-only client in buffer mode cannot use
the flag to detect its own losses.

**WRCOL is about the BOUNDARY, not about writing twice.** The same
client writing the same marker over the answer it had already loaded
gets opposite results on the two sides of one byte: in the gap the host
leaves between bytes the write is an ordinary write and is OBEYED - the
marker goes out in place of the answer - while a write made after SCK
has left its idle level is IGNORED, the loaded answer goes out intact,
and WRCOL comes up. With constant streams (host 0x5A, client 0xA5) and
marker 0x3C the host reads `A5 A5 3C A5 A5 A5 A5 A5`, the marker sitting
only where the gap write landed (test `p`, byte-identical over five
runs). A client is therefore never protected by WRCOL from its own
mistimed writes; it is only protected from writing *into* a transfer.

That also makes the flag hard to CATCH rather than hard to raise:
reading INTFLAGS and then accessing DATA is the documented clear
sequence, so any poll loop erases its own evidence - WRCOL has to be
sampled between the write that caused it and the next DATA access. A
spin that mixes flag reads with DATA writes can even clear IF the same
way and lose a whole byte's completion.

**A client that MISSES its load sends back the byte it just received.**
The shift register is shared between the two directions, so a client
that writes nothing after a byte has the incoming byte still in it when
the next transfer starts. With constant streams (host 0x5A, client 0xA5)
the host read `A5 A5 A5 A5 5A A5 A5 A5` for a load skipped after byte 3
(test `p`). This is the failure mode of a client too slow for the host's
inter-byte gap, and it is silent unless the host checks the data.

**A REAL host demotion, and what re-arming needs.** With SSD = 0 and the
SS pin an input on its pull-up, the other board driving the shared wire
low cleared MASTER and raised IF (INTFLAGS 0x80), and MASTER stayed
clear. The demotion follows the LEVEL, not an edge: `restore_host()`
called while the other board was still holding the wire down left MASTER
at 0, and only stuck once the wire was released (measured 19 ms later,
against the peer's 20 ms hold). A post-recovery exchange was exact.
Test `q`.

**The command channel is a DIVISION of CLK_PER and follows a rebase by
itself.** Across 24 -> 12 -> 24 MHz on the host with the client left at
24 MHz, the SCK period measured 32 CLK_PER ticks at every step (750 kHz,
375 kHz, 750 kHz) and every exchange stayed exact (test `s`).

**The USART's own Host SPI mode talks to this peripheral exactly**, in
all four phase/polarity combinations and in both bit orders, twelve
bytes each way at 750 kHz (test `r`). `MspiHost`'s `sample_trailing`
(UCPHA) and `invert_sck` (the XCK pin's INVEN) map onto `SpiMode`'s CPHA
and CPOL bit for bit: mode 0..3 = `{invert_sck, sample_trailing}`, and
`lsb_first` (UDORD) onto DORD. Host SPI has no client select, so the
client selects itself with INVEN on its own pulled-up SS pin.

## Not covered yet

**Driver gaps** (what the code does not do):

- no client-side ISR-driven service: `SpiClient` is a polled surface plus
  the resource's ISR bodies, and the AO that would sit on it (the mirror
  of `BusMaster` for a client) is not written - it will be born with its
  first user;
- no wake-from-idle path: the chapter lists it as a feature, this driver
  has no sleep story and neither has the rest of `avrdx/` yet;
- the engine's SCK ceiling clamps a request DOWN silently; it does not
  report that it did.

**Implemented but not bench-verified:**

- MULTI-HOST arbitration as a protocol: the demotion mechanism is
  measured on the wire (above), but two hosts actually contending for
  one bus - and the driver's part in resolving it - is not written and
  not tested;
- SPI1 electrically: its pin positions (PC0-PC3, PC4-PC7) are the
  traffic LEDs of this bench, so SPI1 is exercised on route NONE only -
  the register work, not a wire;
- SPI0 DEFAULT and ALT2 electrically: DEFAULT (PA4-PA7) is cabled on
  this desk to a 3.3 V display module and an MCP3550 while the desk runs
  at 5 V, and ALT2 needs a 64-pin package this bench does not have;
- the errata refusals are proven by what the driver REFUSES, not by what
  the silicon does: nobody has watched SPI1 ALT2 fail on a 48-pin part,
  or a pinless DA host lose Host mode with SSD = 0;
- the electricals of 39.15 / clarification 3.7.3 as timing (setup and
  hold windows, the client's tSOS) - only the frequency ceilings are in
  the code, and clarification 3.5.2's buffer-mode setup warning near the
  maximum SCK has not been provoked: the two-board half runs the client
  well inside its ceiling.
