# SPI - the serial peripheral interface (AVR DA/DB)

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (SPI
chapter 28, PORTMUX chapter 17, electricals 39.15), errata DS80000915F
(2.11.1 and clarifications 3.5.1, 3.5.2, 3.7.3) and, for the DA parts,
DS80000882C (2.10.1). Three items shape the code:

- **DB 2.11.1, SPI1 ALT2 on 48-pin devices**: the position is
  NON-FUNCTIONAL there (rev. A4/A5; fixed in B0). The 48-pin device
  headers still list it, with MOSI on PB4 and MISO on PB5 and no SCK or
  SS position at all - so the driver refuses it on every 48-pin part of
  both families, at compile time and at run time. Here an erratum beats
  the device header.
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
| `SpiHost<n, route>` | the transfer ENGINE, the instance in BUFFER MODE: `Request` descriptors (21 bytes, no padding: two `PinRef`s, three pointers, the lengths, the reply, four settings - the field names every stratum shares), `start()` (a polled request completes inside it reading the lent request and copying nothing; the pump copies its tenure's fields at launch), `isr()` (the pump on RXCIE and TXCIE: the received bytes read by count, two in flight, one interrupt a byte when the handler is on time and one for two when it is late), `status()` (always `spi_ok` here - no DMA path, no fault of its own; the verb keeps the app glue spelled as on every stratum), an optional SCK ceiling with `clock_for(max_sck_hz)` as the chooser at the engine's own clock, `prime(mode, clock)` for a caller framing the select by hand (a CPOL change moves the SCK pad to its new idle level, one edge a selected client counts), `rebase()`, and `recover()` - the verb a timed `SpiBus` calls on a transaction that never answered (util/bus_master.hpp): the pump's interrupt silenced, a demoted host re-armed, the bytes still in flight waited for (bounded), the FIFO read out whatever RXCIF says, the select window closed |
| `SpiClient<n, route>` | the client side: `selected()`, `preload()`, `exchange()`, the buffer-mode readbacks, the ISR bodies, `max_sck_hz()`, and `frames_ahead` (ONE: how many answers a pump must keep queued ahead of the host's clock - the one integer that differs between this family's client pump and the other strata's) |

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
`test_avr_spi`: `z` = the single board, 184 verdicts; `y` = the same
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

**And a frame inside a block takes eight periods at the division ASKED,
plus two cycles.** The period above is one SCK against the driver's own
division table; a board answering itself cannot tell a wrong division by
its data either. So the suite times A FRAME INSIDE A BLOCK on the ruler
(TCB3 and TCB2 cascaded at CLK_PER): a block of 2048 frames less one of
256, the best of three each, so the request's fixed cost cancels -
through the data sheet's own buffer-mode sequence (28.3.2.1.2) as a bare
loop, and through the engine's polled clocks-only shape - against eight
SCK periods at CLK_PER over the division named in the test, in
thousandths of a cycle:

| division | bare sequence | engine | eight SCK + 2 | reading |
|---|---|---|---|---|
| CLK_PER/2 | 18.002 | 18.000 | 18 | 1000 |
| CLK_PER/4 | 34.000 | 34.001 | 34 | 1000 |
| CLK_PER/8 | 66.002 | 66.000 | 66 | 1000 |
| CLK_PER/16 | 130.000 | 130.001 | 130 | 1000 |
| CLK_PER/32 | 258.002 | 258.000 | 258 | 1000 |
| CLK_PER/64 | 514.000 | 514.001 | 514 | 1000 |
| CLK_PER/128 | 1026.002 | 1026.000 | 1026 | 1000 |

The two cycles are buffer mode's, between two frames that leave back to
back: the same at every division, in all four modes (514.001 at
CLK_PER/64 in each), and through the bare sequence as through the
engine - the chapter does not state them. Under a ceiling the frame is
the ceiling's division: 130.001 for CLK_PER/2 asked under 1.5 MHz at
24 MHz, 66.000 of the 12 MHz CLK_PER after a rebase (the ruler counting
the clock in force), 514.000 for CLK_PER/4 asked under 400 kHz. At
CLK_PER/2 a frame leaves the loop three cycles of slack: an interrupt
inside the block costs frames there - the console's transmit vector
while a printed line drained took the engine's frame to 19.2 to 20.6
cycles and the bare sequence's to 18.9 - so the suite times every block
with the console drained and no vector armed.

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
and it is a CONDITION, so it means "this burst finished" only when it was
cleared behind the burst's last write - cleared before the burst, any
idle moment inside it raises it again (The engine's cost). BUFOVF is not raised by the third undrained
byte, which waits in the shifter: it appears when the NEXT transfer
starts, exactly as the register description says. A read that frees a
place lets the waiting byte in before a write made right behind it
starts the next transfer: three bytes received unread, one read and a
write at once, all three kept and BUFOVF clear, at CLK_PER/2, /4 and /16
(24 of 24). A store of one to RXCIF over a byte in the FIFO drops the
flag and leaves the byte: the flag stays down through the next arrival,
and the first read brings it back for the second byte (16 of 16) -
which is what the resource's `init()` does to whatever the FIFO holds. A
read of the EMPTY FIFO returns the last byte read and changes nothing
after it (16 of 16). BUFWR changes nothing in host mode.

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
vector re-enters immediately; with both halves in place and both RXCIE
and TXCIE armed, eight bytes written one at a time produce ten
interrupts - one RXC each, plus a TXC wherever the transmitter fell idle
between them - and then silence.

**The engine.** Both completion styles move the same bytes, the chip
select comes back high after each transaction, a command phase reaches
the wire, a zero-length request completes without touching it, and a
request faster than the engine's ceiling is slowed to the ceiling. Under
a 24 -> 12 -> 24 MHz rebase a 1.5 MHz ceiling re-picks CLK_PER/16 ->
CLK_PER/8 -> CLK_PER/16 and the measured SCK stays at 1.5 MHz. These
verdicts are the engine in buffer mode (below); test `t` adds the engine
through the read below.

**A read that empties the FIFO as a byte enters it is not taken.** With
one byte in the receive FIFO, a DATA read made in the CLK_PER cycle the
next received byte enters it RETURNS the first byte and LEAVES it there;
the arrival queues behind it, and RXCIF falls. The next DATA read returns
the first byte AGAIN and brings the flag back for the second. Until then
the FIFO is full with the flag down: the next arrival raises no RXCIF and
waits in the shifter, and the transfer after it raises BUFOVF. Measured
with the byte in the FIFO received from a MISO held low and the
arriving one from a MISO held high, a read placed at every cycle of the
arrival: one phase of 117 at CLK_PER/4 and one of 357 at CLK_PER/16
returns 0x00, then 0x00 again, then 0xFF (test `t`). 28.5.5 says only
that the flag clears when the buffer is empty, and the errata items
this chapter cites (above) are not about it. Two neighbours of the
cycle are safe, and the engine is built on them: a read that LEAVES a
byte in the FIFO is taken at every phase of an arrival (two bytes held,
the read swept across the third's: 474 of 474), and a read made with
nothing in flight has no arrival to meet. And the transmitter's flags
come no later than the byte: of 650 samples that read INTFLAGS showing
DREIF (two bytes written) or TXCIF and read DATA one load later, none
found the byte not yet there (test `t`, both rates, every phase).

RXCIF is therefore no count, and the engine reads by count (below): the
polled loop never makes the read that empties the FIFO while a byte
shifts, and the pump, which reads right behind each arrival and so can
make it when its handler is held late, recognizes the entry that
follows, TXCIF up and RXCIF down with bytes due, and reads the repeated
byte once more and drops it. Test `t` holds the core with a TCB2 handler of a
fixed length fired at a swept time after each 16-byte request starts,
so that the engine's reads fall at every phase of the arrivals, and
alternates the MISO level between requests, so that a byte read twice
or left behind shows as a wrong level in the next one: 3600 pumped
requests at CLK_PER/16 met the cycle eight times and all completed with
every byte the level MISO held and the receiver empty after, and 3600
polled requests under the same handler likewise.

### The engine's cost

Every number in this section is COUNTED in the release listing of
`bench_avr` (`-Os`, the AVR128DB48, CLK_PER 24 MHz) and, where a
measured figure stands beside it, MEASURED by that app's letter `e` on
rev. A5 at the same rate (benchmark.md's `spi.poll`, `spi.poll.tx`,
`spi.pump` and `spi.req` lines), the letter's lines in its own order -
polled, pumped, requests - on one receiver. The engine is written from
28.3.2.1.2 and 28.5.5 and from the read the silicon does not take (Bench
findings); what the silicon offers the host and how each item is used:

| the silicon's offer (section) | used? | how, and what it buys |
|---|---|---|
| buffer mode's two-level transmitter: a write is legal while DREIF is set, the first into the idle shifter, the next into the buffer (28.3.2.1.2, 1) | YES | two bytes in flight on both completion styles: the wire runs while the software prepares the next byte, so a loop shorter than a byte time is wire-bound. Normal mode (28.3.2.1.1) had one in flight and the bus idle between bytes |
| the two-entry receive FIFO, read at least every second transfer (28.3.2.1.2, 2 and 3); the third byte waits in the shifter and is lost when the NEXT transfer starts (28.5.5, BUFOVF) | YES | read BY COUNT, from what the transmitter's flags prove has arrived, never by RXCIF. The polled loop reads, after each DREIF, the older of the two newest bytes and leaves the newest in the FIFO, so it never makes the read that empties it while a byte shifts: three bytes outstanding, and a loop held late finds the FIFO full and the third held in the shifter, which its next read lets in before the next write starts a transfer (Bench findings). The pump keeps two outstanding and reads right behind each arrival. No byte waits for a transfer to start, so BUFOVF cannot rise, and a late loop or handler idles the wire and loses nothing, at every division |
| TXCIF, set when shifter and buffer are both empty (28.3.2.1.2) | YES | "every byte written has arrived", read only where it was cleared BEHIND the last write while that byte shifted: a polled burst's last write and the clear are masked together (an interrupt between them could let the byte finish and the clear wipe the edge the tail waits for), the pump's launch masks its two writes with the clear behind them (a handler between the writes let the first byte finish and raise TXCIF before the second went - measured, a byte then left over), and the handler clears it behind the first write of every refill |
| RXCIF as the interrupt source (28.5.3 RXCIE, 28.5.5) | YES | the pump's edge and never its count: the handler reads INTFLAGS and takes what DREIF and TXCIF prove |
| TXCIE | YES | the pump's second edge: a handler held late finds the transmitter idle, and the entry where TXCIF is up and RXCIF down while bytes are due is the read that was not taken - the repeated byte is read once more and dropped |
| DREIE | no | DREIF would wake a pump that has nothing left to write; RXCIF and TXCIF carry the data and the completion |
| BUFWR | no | a client bit (28.3.2.1.2: "does not affect Host mode") |
| the SS pin as a multi-host demotion (28.3.2.1.3) | no | SSD is set: the select pad is the engine's GPIO, and the polled spin needs no bound because nothing can take the clock generator from this host |
| IF/WRCOL, the normal layout | no | the host never returns to normal mode; the resource keeps both layouts for its own verbs and for a client |

**The polled loops** (`burst<out, in>`, four shapes, inlined into
`start()` so no test of the spans runs per byte; `start()` is 403
instructions in all, of which one request walks one shape; the first
two bytes and the last are peeled off the loop, the last with its masked
TXCIF clear, so the loop itself tests nothing but its count):

| shape | cycles per byte, steady state (counted) | measured, a byte beyond the sixteenth (256 bytes against 16) | against the wire |
|---|---|---|---|
| write-only (`tx` set, `rx` null - the display's) | 19: INTFLAGS read and DREIF tested, the older byte read and dropped, the count (SBIW, BREQ), LD, the DATA store, two MOVWs gcc keeps the source pointer through; 17 for a command phase, whose pointer stays in Z | 34.0 at CLK_PER/4, 130.45 at CLK_PER/16; 256 bytes x 1.09 and x 1.02 | wire-bound from CLK_PER/4 (32 a byte and the block's 2); at CLK_PER/2 (16 and 2) within a cycle of the wire (counted) |
| full duplex (both set) | 23: the read stored through a second pointer, its increment a 16-bit add beside the store, a third MOVW | 34.0 at CLK_PER/4, 130.55 at CLK_PER/16; 256 bytes x 1.09 and x 1.02 | wire-bound from CLK_PER/4; at CLK_PER/2 about 1.3 times the wire (counted) |
| read-only (`tx` null) | 19: a constant 0xFF written | no line | wire-bound from CLK_PER/4 |
| clocks only (both null) | 15 | 18.000 at CLK_PER/2, 34.001 at CLK_PER/4 (`test_avr_spi` letter b, a frame inside a block) | wire-bound from CLK_PER/2: the wire's 16 and the block's 2 |

The loop keeps its count only through an empty asm the count passes
through: without it gcc rewrites the exit as a compare against a
pointer's end or against the count it came in with and shuffles the
pointers through other registers to do it (25 cycles for full duplex).
The tail reads the last three bytes after TXCIF, with nothing left to
arrive.

The two cycles a byte beyond the wire's 32 and 128 are the block's and
not the loop's: they are the same at both rates, the loop has some
hundred cycles of slack a byte at CLK_PER/16, and the data sheet's own
buffer-mode sequence (28.3.2.1.2) as a bare loop measures the same - 256
bytes full duplex at x 1.09 at CLK_PER/4 (35.2 a byte; 31 counted) and x
1.01 at CLK_PER/16, the engine's loop within a cycle a byte of it, the
data sheet being this family's vendor. Its normal-mode sequence
(28.3.2.1.1: write DATA, spin IF, read DATA, one byte in flight)
measures 58.2 cycles a byte at CLK_PER/4 (x 1.81; 53 counted) and 156
at CLK_PER/16 (x 1.21): what buffer mode buys.

**A polled request's fixed cost** - the price of a DCS command, one
command byte and two of data at CLK_PER/16, the select and the D/C on
real pads (`spi.req`, n = 3): about 222 cycles above the wire's 384,
counted along the executed path: 115 before the first byte (the call,
the entry, the length test, `apply()`'s fold and compare at 35 with no
register access, the two edges at 10 and 14, the setup test, the
command burst's prologue with its last byte taken first and its masked
write), 59 between the phases (the TXCIF wait's last turn, the
command's reply read, the D/C edge, the data burst's prologue), 42
after the last byte (the TXCIF wait's last turn, the two reads, the
select's edge, the return) and the block's two cycles a byte. Measured:
221 (wall 670, less the wire's 384 and the stopwatch's floor of 65 -
letter `r`'s `stopwatch` line); 170 for the command byte alone (n = 1)
and 254 with fifteen bytes of data (n = 16).

**The pump** (`isr()`, on RXCIE and TXCIE): two paths. On time - the
handler entered on RXCIF while the next byte shifts - it reads one byte
and writes one: 102 cycles of body from the INTFLAGS read to the count's
store (the flags, the count and what it may read, the DATA read and the
store through `in_`, the 16-bit `to_write_` tested, the next byte loaded
and written, TXCIF cleared behind it), counted. Late - the transmitter
idle and TXCIF up - it reads both and writes two: some 159 counted, 151
measured (the meter's `isr`, 227 an interrupt, less letter `r`'s `stamp`
of 76 for an empty body). Letter `e`'s lines run late at both rates,
because the bench's meter in the vector costs 217 cycles of wall an
interrupt (`stamp`): 228 cycles of wall a byte at CLK_PER/4 and at
CLK_PER/16 alike (256 bytes: x 7.17 and x 1.79), 130 interrupts for 256
bytes, every completion seen and BUFOVF clear after every run. Without
the meter the handler is on time at CLK_PER/16 - the read 30-odd cycles
after the edge, the next byte 130 away - and costs the 102 and an entry
and exit of some 55 (the hardware's, 15 pushes, their pops, RETI), about
1.2 times the wire's 130 a byte; at CLK_PER/4 two bytes take 68 cycles
and the handler is late at every entry, about 107 a byte, three times
the wire (counted, no line). Polling is the bulk path. A handler that
calls out (the app's `post<Bus>` on the completion edge) saves the whole
caller-clobbered set whatever its body does. What reading by count cost
the pump against reading by the flag is the flags read and the count
around the one DATA read, some 30 cycles of the on-time body; what it
bought, besides surviving the read the silicon does not take, is the
late handler taking two bytes an entry.

**`apply()`**: the request's `mode` and `clock` folded into the two
register bytes the block takes (CTRLA = MASTER, the PRESC/CLK2X bits
of `spi_presc_bits()` - arithmetic on the enum, no table -, ENABLE;
CTRLB = BUFEN, SSD, MODE), compared with the pair the engine last
wrote: 34 cycles and no register access on the unchanged path, the
case of a bus with one device. On a change the two bytes are stored,
and when CPOL moves the SCK pad is preset and the instance disabled
around the store (the quirk under "Two silicon facts" in
[spi-bus.md](../design/spi-bus.md)).

**The pins**: a `PinRef` edge inline is the pointer's two loads, the
null test, the mask's load and one OUTSET/OUTCLR store, 10 cycles from
a request on the stack and 13 from the engine's copy, where the
out-of-line verb cost 22 plus the CALL; its null test is the one branch
an absent D/C pin costs. `cs_setup_us` is tested before the delay is
called; a nonzero setup goes through the stored-byte delay's short
path (a `us` under 256: one MUL, the quarter rounded up,
`_delay_loop_2`'s turns), 39 cycles for 1 us counted in the listing,
38 measured: the request of `spi.req` (n = 3) with a 1 us setup takes
38 cycles more than without one, and 24 more for each further
microsecond (docs/avrdx/platform.md).

**No `dma_min_frames`** on this family: there are no engines, so there
is no fixed cost for the pump's per-byte one to be weighed against.

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
- the engine's SCK ceiling clamps a request DOWN silently; it does not
  report that it did - a stated caveat, since a slower transfer is
  still a correct one;
- `recover()` and the timed `SpiBus` over it are host-tested and
  compile-proven on every package, but no wedge has been staged on AVR
  silicon (a demotion mid-transfer is the staging that would do it -
  the demotion itself is bench-measured, the timed recovery over it is
  not).

**Implemented but not bench-verified:**

- the engine's bytes against a real client: `test_avr_spi`'s letters
  `l` and `m` and a panel's memory read back would judge them, both on
  the wire to a second device (a peer board);
- the rates letter `e` does not run, and the pump without the bench's
  meter: the polled loops at CLK_PER/2 (write-only within a cycle of the
  wire, full duplex about 1.3 times it), the pump on time at CLK_PER/16
  (about 1.2 times the wire), late at CLK_PER/4 (about three times) and
  wire-bound from CLK_PER/32 are counted, and a line at each, the pump's
  in a vector with no meter, would measure them;
- the receive FIFO emptied by `init()` and `recover()` (`flush()`): the
  sequence it reads with - one read whatever RXCIF says, then a read per
  entry the flag shows - is the one test `t` judges on a FIFO left with
  the flag down, but neither verb has been called on such a FIFO, and
  `recover()`'s bounded wait for the bytes still in flight has never
  waited (the wedge above);
- an SPI interrupt as the wake from Idle (28.3.5's feature): the sleep
  story is [platform.md](platform.md)'s and no letter of either suite
  sleeps with a transfer pending;
- MULTI-HOST arbitration as a protocol: the demotion mechanism is
  measured on the wire (above), but two hosts actually contending for
  one bus - and the driver's part in resolving it - is not written and
  not tested;
- SPI1 electrically: it is exercised on route NONE only - the
  register work, not a wire;
- SPI0 DEFAULT and ALT2 electrically: the suite runs on ALT1 and leaves
  DEFAULT (PA4-PA7) unwired, and ALT2 needs a 64-pin package this bench
  does not have;
- the errata refusals are proven by what the driver REFUSES, not by what
  the silicon does: nobody has watched SPI1 ALT2 fail on a 48-pin part,
  or a pinless DA host lose Host mode with SSD = 0;
- the electricals of 39.15 / clarification 3.7.3 as timing (setup and
  hold windows, the client's tSOS) - only the frequency ceilings are in
  the code, and clarification 3.5.2's buffer-mode setup warning near the
  maximum SCK has not been provoked: the two-board half runs the client
  well inside its ceiling.
