# USART - the serial port (AVR DA, DB and EA)

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (USART
chapter 27, PORTMUX chapter 17, electricals 39.14), errata
DS80000915F 2.16.1-2.16.3 and, for the DA parts, DS80000882C
2.15.1-2.15.2; for the AVR64EA28/32/48 DS40002443A (USART chapter 25 -
the DB's chapter register for register - PORTMUX 17.3.3-17.3.4,
electricals 35.13) and DS80001048D 2.5.1 (the DB's 2.16.3, word for
word; the DB's 2.16.1 and 2.16.2 have no EA twin, and brio applies both
workarounds on every part - measured on the EA, below: they cost nothing
there and the half of each this desk can probe holds). Three errata
items shape the code:

- **2.16.1 / 2.15.1, open drain**: with the TXD pin configured as an
  output it drives high whether ODME is set or not. The work around is
  to leave TXD's direction at INPUT and let the pull-up hold the line -
  which contradicts 27.3.1's "the TXD pin is automatically set to
  output by hardware", and is what a one-wire `init()` does.
- **2.16.2 / 2.15.2, start-of-frame**: SFDEN set in Active mode makes a
  read of RXDATA re-trigger the detector and corrupt the frame being
  received. SFDEN is therefore not a configuration field here but a
  pair of verbs to use around the transition to standby.
- **2.16.3, auto-baud**: a set ISFIF leaves the receiver dead and
  clearing the flag does not revive it - RXEN must be written 0 then 1.
  Every auto-baud path goes through that.

Driver: `avrxt/usart.hpp`. The line service above it:
[serial.md](../design/serial.md) (`util/serial_port.hpp`). Reference
test: `test_avr_serial`; every other suite's console exercises the
`Uart` task by existing.

## What the silicon does

One peripheral with four communication modes over one fractional baud
generator, one two-level transmit buffer and one two-level receive
buffer, buffered in both directions so frames can follow each other
with no gap.

Facts that matter to code:

- **The fractional baud generator.** BAUD holds the divisor left
  shifted by six: `BAUD = 64 x f_CLK_PER / (S x f_baud)`, with S = 16
  in normal asynchronous mode, 8 in double speed, 2 in the synchronous
  modes. Valid range 64..65535 - so a rate needs at least
  `S x f_baud` of peripheral clock. In the synchronous modes only
  BAUD[15:6] is the divisor and the six low bits must be zero.
- **Routes, including no route at all.** PORTMUX.USARTROUTEA/B gives
  each instance two bits: DEFAULT, ALT1 and NONE. NONE disconnects
  every pin and the peripheral still runs - the generator, the
  transmitter and its flags all work - but the internal loop-back and
  open-drain do not, because both act on the TXD PAD (measured).
- **The AVR EA's route table has another SHAPE** (DS40002443A 17.3.3-4,
  table 3-1): three USARTs on every package; USART0's field three bits
  wide with routes DEFAULT PA0-PA3, ALT1 PA4-PA7, ALT2 PA2/PA3
  (TXD/RXD only), ALT3 PD4-PD7, ALT4 PC1-PC3 (no XDIR) and NONE = 5;
  USART1 DEFAULT PC0-PC3, ALT1 PC4-PC7 (48 pins), ALT2 PD6/PD7
  (TXD/RXD), NONE 3; USART2 DEFAULT PF0-PF3 (PF0/PF1 alone on 28 pins),
  ALT1 PF4/PF5 (TXD/RXD, 32 and 48 pins), NONE 3 - and USART2's field
  in USARTROUTEB. A route may move the instance to another port; every
  signal of one route sits on one port. brio names routes, never codes:
  `UsartRoute::alt2..alt4` exist only where the layout has them, every
  route's code is the device tables' `usart_route_codes` (the header's
  `_gv` enumerators, the chapter's number where only some packages name
  one), the register and the field are the header's `_gm`/`_gp`, and
  `routed()` maps the code back to the name.
- **Which positions exist is a per-package fact**, and the device
  header's route enums are the authority. Three instances on 28/32
  pins, five on 48, six on 64; and the ALT1 halves appear one signal at
  a time as the package grows - on 48 pins USART3's ALT1 has TXD/RXD on
  PB4/PB5 but no XCK/XDIR, and USART4 has no ALT1 at all. USART2's ALT1
  has no XCK position on any package.
- **The read and write order is the character size's business.** The
  status of a received frame (FERR, PERR, BUFOVF and the ninth data
  bit) lives in RXDATAH, and reading ONE of the two data registers
  shifts the FIFO: RXDATAH first, then RXDATAL - except in 9-bit
  low-byte-first (9BITL), where RXDATAH is the shifting read and
  RXDATAL must come first. Transmission mirrors it.
- **STATUS is a mix.** RXCIF and DREIF are conditions; TXCIF, RXSIF,
  ISFIF and BDF are write-one-to-clear and WFB is WRITE-ONLY (there is
  no reading back whether an arming stands). The W1C bits are cleared
  with a plain store of that one bit - an RMW would read every pending
  flag back as one and clear them all.
- **Three frames fit before anything is lost**: TXDATA, the TX buffer
  and the shift register on the way out; the same depth on the way in.
  BUFOVF is set when a start bit arrives with all three full, and it
  travels with its frame through the FIFO.
- **RXCIF is a level, and RXDATAH carries it.** It stands while unread
  data remain (27.5.5) and is bit 7 of RXDATAH (27.5.2), so the status
  byte read after the shifting read says whether a second frame waits.
- **TXCIF is set only when the shift register empties with nothing
  behind it** (27.3.2.3), and a write to TXDATA does not clear it: one
  left from an earlier idle moment stands through the next run until
  someone writes it 1.
- **Disabling the two directions is not symmetric.** The receiver stops
  at once and its buffer is flushed; the transmitter finishes what is
  in flight and then stops overriding TXD, which PORT gets back as an
  INPUT.
- **Auto-baud** measures eight bit times of a 0x55 sync field after a
  break and writes the result into BAUD. GENAUTO accepts a break of any
  length once WFB is armed; LINAUTO wants a real LIN break and insists
  the sync character be 0x55, and rejects anything else with ISFIF.
- **MPCM is a receiver filter**: with nine data bits the ninth bit,
  with five to eight the first stop bit, marks an address frame, and
  data frames are dropped until one arrives.

### The receive side's offer, and what `Uart` takes of it

The chapter offers the receive path these, each used or declined:

- **The edge: RXCIF, used.** It is the only receive edge in Active mode:
  no idle flag, no receiver time-out, no FIFO threshold exist in chapter
  27. `rxc()` returns the ring's empty -> non-empty transition, so the
  consumer hears the burst's FIRST byte, and a console ends a line at its
  terminator, not at a silence.
- **The start-of-frame detector (RXSIF, SFDEN), declined as an edge.**
  It fires in Standby only (27.5.5, measured below: from Idle never),
  and SFDEN left set in Active mode corrupts the frame a read of RXDATA
  re-triggers (errata 2.16.2 / 2.15.2). It stays the standby wake verb
  pair.
- **An idle detector built from the RXD pad, declined.** The pad's edges
  through EVSYS into a TCB in time-out mode would give a burst's end, at
  the price of a TCB, an event channel and a pin channel per port (PORTE
  and PORTF pins reach channels 4 and 5 only, 16.5) - for a consumer that
  already ends a line at its terminator and a burst edge that is the
  first byte's.
- **The two-level buffer, used whole.** One `rxc()` entry takes every
  frame the buffer holds - the loop ends on RXDATAH's RXCIF - so the
  vector's entry and exit are paid once for two frames when the handler
  is late, which is when it matters.
- **DMA, circular mode: none on this family.**
- **The clears: none to make.** FERR, PERR and BUFOVF travel with their
  frame in RXDATAH and go with its read (27.3.2.4.1, 27.5.2); nothing
  else is read to clear them, so no byte beside the hit one is lost
  (`test_avr_serial` letter I). The vector's two other sources (RXSIE,
  ABEIE) are the resource's to arm; an entry they make finds RXCIF clear
  and takes nothing.
- **The loop-back (LBME), used for the measurements.** It listens to the
  TXD PAD, so with the transmitter off it hears whatever PORT or a timer's
  waveform output drives there - which is how `bench_avr`'s letter u feeds
  the receiver from TCA0 with no CPU in the sender.

## Types and verbs

The resource owns the registers and speaks BAUD register values; the
tasks speak hertz and are `ClockUser`s.

| Entity | Verbs |
|--------|-------|
| `UsartRoute`, `usart_route_exists(n, r)`, `usart_pin(n, r, sig)` | the per-package route table: `def`/`alt1`/`none`, and per signal a `{port, pin, bonded}` |
| `usart_baud_reg(hz, baud, S)` / `usart_actual_baud(hz, reg, S)` / `usart_min_clock_hz(baud, S)` / `usart_samples(mode, rx)` | the baud arithmetic, constexpr; 0 = the register cannot express it |
| `UsartConfig`, `usart_config_valid<n>(cfg)` | the whole configuration in one struct, and what this package refuses |
| `Usart<n>` lifecycle | `init<cfg>()` (static_asserts), `init(cfg)` -> bool, `release()` (route to NONE, pins back to PORT), `enable_rx`/`enable_tx`/`disable`/`flush_rx` |
| `Usart<n>` format and rate | `frame(UsartFormat)`, `bits()`, `mode()`, `baud_reg()`/`baud_reg(v)`, `set_baud(hz, baud)`, `actual_baud(hz)`, `samples()`, `max_host_xck_hz`/`max_client_xck_hz` |
| `Usart<n>` mode knobs | `rx_mode`, `multiprocessor`, `open_drain`, `loop_back`, `rs485`, `arm_start_of_frame`/`disarm_start_of_frame`, `auto_baud_window`, `irda_event_input`/`irda_on(ch)`, `tx_pulse`, `rx_pulse`, `debug_run` |
| `Usart<n>` status | `status()`, `rxc_flag`/`dre_flag`/`txc_flag`/`rxs_flag`/`isf_flag`/`break_flag`, `clear_txc`/`clear_rxs`/`clear_isf`/`clear_break`, `wait_for_break()`, `recover_from_isf()` |
| `Usart<n>` interrupts | `enable_rxc_interrupt`, `enable_txc_interrupt`, `enable_dre_interrupt`, `enable_rxs_interrupt`, `enable_autobaud_error_interrupt` |
| `Usart<n>` data | `receive()` / `receive_as<bits>()` -> `UsartFrame` (ISR body of `USARTn_RXC_vect`), `transmit(v)` / `transmit_as<bits>(v)`, `clear_txc()` (ISR body of `USARTn_TXC_vect`), the bounded `send`/`poll`/`wait`/`wait_line_idle` |
| `Usart<n>` events | `XckEvent` (generator), `IrdaIn` (user) |
| `Uart<n, Route, rx, tx>` | the interrupt-driven transport: `init(clock, baud)`, `rebase(hz)`, `set_baud(hz, baud)` (a new rate under the running port, once `tx_idle()`), `release()`, `can_baud`, `min_hz_for`, `actual_baud(hz)`, `write_byte`/`write_bulk` (a run: as many as fit, its first byte pushed and DREIE set before the rest is copied, the rest copied in parts of at most `copy_part` (32) bytes, each published and DREIE set behind it), `read_byte`/`read_span`/`consume` (the receive run in place), `rx_pending`, `tx_idle` (the WIRE is idle: the ring empty and the last stop bit out - TXCIF, which the transport owns), the error counters, `rx_skips` (the gaps in the stream the receive ring hands out, never cleared - util/serial_port.hpp's epoch: the SkipRing's skips, modulo 2^8, one at the consumer's look after `rxc()` reported a frame refused by a full ring, dropped for FERR or PERR, or a BUFOVF - on the rare paths alone, the RXC vector saving no register for them), ISR bodies `rxc()` (takes every frame the buffer holds, returns the empty -> non-empty edge) and `dre()` |
| `OneWire<n, route>` | `available`, `init(clock, baud, fmt)`, `talk()`/`listen()`, `line()`, `echo_matches(sent)` |
| `Rs485<n, route>` | `available`, `init(clock, baud, fmt, one_wire)`, `drive_enable()`, `guard_bits` |
| `SyncHost<n, route>` / `SyncClient<n, route>` | `available`, `init(...)`, `clock_pin()`, `invert_xck(bool)` (host), `max_xck_hz(hz)` (client) |
| `MspiHost<n, route>` | `available`, `init(clock, sck_hz, {lsb_first, sample_trailing, invert_sck})`, `transfer(byte)`, `release()` (the resource's, plus the XCK pin's INVEN) |
| `IrdaLink<n, route>` | `init(clock, baud, tx_pulse, rx_pulse, fmt)`, `receive_on(channel)`, `max_baud` |
| `AutoBaud<n, route>` | `init(clock, fallback, Kind, window, fmt)`, `arm_break()`, `break_detected`/`clear_break`, `sync_error()`, `recover()`, `measured_baud_reg`/`measured_baud(hz)`, `poll()` |

What the driver refuses, at compile time through `init<cfg>()` and at
run time through `init(cfg)`: an instance or a route this package does
not have; a synchronous or Host SPI mode without an XCK pin; RS-485
without an XDIR pin; loop-back or open drain without a TXD pad (the
pinless route included); a non-normal receiver mode outside
asynchronous operation.

What a byte costs `Uart`, counted in the release listings (DS40002198B,
the AVRxt column; the CPUINT's response of six cycles added):

- **Receive: 75 cycles for a frame that is the entry's only one where the
  binding discards the edge, 83 where it uses it, and 30 for each further
  frame the same entry takes.** `rxc()` reads RXDATAH and
  RXDATAL itself and tests the status byte's three flags with one AND on
  the clean path: the vector bound to it alone saves four registers,
  SREG and RAMPZ and calls nothing. It does NOT go through the
  resource's `receive_as()`: once an image has a second call site of it
  for the instance, its `UsartFrame` is built by an out-of-line
  `frame_of()` into a stack frame, and the vector paid 207 cycles a frame
  - sixteen pushes, a frame, a call and a five-byte copy. The edge is
  fourteen cycles (the ring's count at entry, a flag per push, the test
  at exit). A binding that posts from it (the console below) must be
  FLATTENED: the kernel's `post()` is a template with vague linkage,
  kept out of line wherever an image calls it twice, and a call in the
  vector makes it save the call-clobbered set - sixteen pushes, 108
  cycles for a byte that makes no edge and 150 for the byte that posts.
  `ISR(vector, ISR_FLATTEN)` inlines the post into that vector alone:
  ten pushes, 91 and 128 cycles (measured with the response and the
  RETI, the vector run from a frame looped back to the receiver), 60
  bytes of flash, the kernel unchanged.
- **Transmit: 60 cycles a byte** (`dre()`, four registers, SREG and
  RAMPZ, flattened so no ring verb is a call), 73 for a run's last byte,
  which also clears DREIE and TXCIF.
- **The run's copy: avr-libc's `memcpy`, 7 cycles a byte plus 17**, from
  two bytes up. A loop in C over the free run's pointers is the same 7 at
  best, and inside `write_bulk()` gcc makes it 13 (Z reloaded from a pair
  for each access): there is no length below which the loop wins by more
  than a few cycles, and none above which it wins at all.

## How to use it

A console (the shape every app uses):

```cpp
using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;
ISR(USART2_RXC_vect, ISR_FLATTEN) { if (Serial::rxc()) brio::post<Lines>(brio::RxActivity{}); }
ISR(USART2_DRE_vect) { Serial::dre(); }
...
Serial::init(clock, 460800);
brio::print(serial, "hello ", 42, brio::crlf);
```

The resource directly, when the frame format is not 8N1:

```cpp
using U = brio::Usart<4>;
U::init({.route = brio::UsartRoute::def,
         .bits = brio::UsartBits::nine_low_first,
         .parity = brio::UsartParity::even,
         .baud = brio::usart_baud_reg(brio::clock_hz(clock), 115'200)});
U::send(0x155);
if (const auto f = U::poll(); f && f->clean()) { use(f->data); }
```

A one-wire bus (LBME + ODME + the pull-up, errata 2.16.1 handled):

```cpp
using Line = brio::OneWire<1, brio::UsartRoute::def>;   // PC0 is the bus
Line::init(clock, 19'200);
Line::talk();
Line::send(0xA5);
if (!Line::echo_matches(0xA5)) { collision(); }
Line::listen();                                        // the pull-up holds it
```

RS-485, where the hardware raises XDIR one baud clock before the start
bit and drops it after the stop bit:

```cpp
using Bus = brio::Rs485<0, brio::UsartRoute::alt1>;     // XDIR on PA7
Bus::init(clock, 115'200);
Bus::send('x');
```

Synchronous host and client (XCK out, XCK in):

```cpp
brio::SyncHost<4>::init(clock, 1'000'000);              // XCK PE2 driven
brio::SyncHost<4>::invert_xck(true);                    // the other sampling edge
brio::SyncClient<4>::init();                            // XCK PE2 taken in
```

Host SPI mode (SCK = XCK, MOSI = TXD, MISO = RXD, no client select -
the app drives one with a `Pin`):

```cpp
using Spi = brio::MspiHost<4>;
Spi::init(clock, 4'000'000, {.lsb_first = false, .sample_trailing = false});
const auto in = Spi::transfer(0xA5);
```

IrDA, with the pulse shaped by TXPL and filtered by RXPL:

```cpp
brio::IrdaLink<4>::init(clock, 115'200, /*tx_pulse=*/0, /*rx_pulse=*/3);
brio::IrdaLink<4>::receive_on(brio::EventChannel<2>{});  // input from a channel
```

Auto-baud from a foreign sender, with the erratum's recovery:

```cpp
using Ab = brio::AutoBaud<4>;
Ab::init(clock, 19'200);            // the fallback rate
Ab::arm_break();                    // GENAUTO: any break length
...
if (Ab::sync_error()) Ab::recover();          // errata 2.16.3
else if (Ab::break_detected()) rate = Ab::measured_baud(brio::clock_hz(clock));
```

Waking from standby on a start bit (the SFDEN erratum's discipline).
The rate has to be slow enough that the line is still LOW when the main
clock is back - see the bench findings, which measure exactly that:

```cpp
U::arm_start_of_frame();            // ONLY just before sleeping
U::enable_rxs_interrupt(true);
sleep();
U::disarm_start_of_frame();         // first thing on the way back
```

## Bench findings

`test_avr_serial` runs on an AVR128DB48 rev. A5 (24 MHz crystal, 5 V)
- and, green the same, on an AVR128DA48 rev. A7 (its external 24 MHz
quartz clock, 5 V: `z` 163 of 163 and `y` 118 of 118 twice, `v` `w` `x`
green, the SAM C21 at -28 ppm against it) - and has two halves: `z` is the SINGLE-BOARD set (11 tests,
163 verdicts, nothing to wire) and `y` is the TWO-BOARD set (13 tests,
118 verdicts and one SKIPPED leg against this desk's peer)
against a peer board driven IN BAND over the very link under test
(`usart_link.hpp`). On this desk the peer is a SAM C21, its SERCOM1 on
the four link wires as the crossed pair - PE0 to its RxD, PE1 to its
TxD, PE2 to its XCK, PE3 to a pad its USART does not take - with every
rate from its own 24 MHz crystal. Three more commands stand outside `y`:
`v`, the wiring probe (run beside the peer's own), `w`, the one-wire bus
on this board's TXD pad (a single PE0-PE0 wire to an avrxt peer, or the
crossed pair itself against a samc21 peer, whose RXD pin meets PE0), and
`x`, the clock comparison. The peer names itself in its ident - the
version's high byte names the implementation (`usart_link.hpp`), and `j`
judges only that the label names a board other than this one. A letter
gates on that kind where a fact is the AVR's own or a capability the
peer's silicon has not got, and prints the reason: `x` runs at 3750 baud
against a samc21 peer (exact on both generators) and at 3600 against an
avrxt one (one BAUD both ends), `w` runs on the crossed pair against a
samc21 peer, `s` skips the fixed-pulse case against it (its IrDA coder
sends 3/16 pulses only) and inverts PE1 for its pulses, `v` takes PE3
either open or to the peer's PE3-role pin. A leg the peer REFUSES
(`report_cfg_failed`) is printed SKIPPED and judged nothing. The peer is DARK until
addressed (`usart_link.hpp`, the dark discipline): while it searches for
the wiring its transmitter is released and neither PE0 nor PE1 is
pulled, and it naks a bad checksum only within `engage_ms` of a frame
that checked out. Its Report carries the `span` - 24 MHz ticks from the
first to the last character it received in `sink` and `echo`, read on
TCB0 + TCB1 cascaded at CLK_PER, one event strobe a character - and its
`blast` repeats with the pattern running on.

### The transport's cost and its two contracts

Measured by `bench_avr`'s letter u (its header says how: USART4's loop
in LBME at 115200, 1, 2 and 3 Mbaud, the receive side fed by TCA0's
waveform on the TXD pad with the transmitter off, every vector bound
plain, counted and metered) and by `test_avr_serial`'s letters I and T,
on the AVR128DB48 at 24 MHz. "Plain" is the vector bound to the driver's
body alone, as an app binds it; its `x` is the wire's verdict, its cost
the listing's count above.

| op, plain binding | 115200 | 1 Mbaud | 2 Mbaud | 3 Mbaud |
|---|---|---|---|---|
| `uart.tx` 256 bytes, x | 1.00 | 1.00 | 1.01 | 1.03 |
| `uart.tx` 4096 bytes, x | 0.99 | 1.00 | 1.00 | 1.00 |
| `uart.rx` 16-byte burst, x | 1.00 | 1.06 | 1.21 | 9.7, the consumer starved |
| `uart.rx` 256-byte burst, x | 1.00 | 1.00 | 1.04 | 3.9, the consumer starved |
| receive entries for 256 bytes (counted) | 256 | 254 | 274 | 1.35 frames an entry (401 for 297) |
| the edge after the last stop bit's end, cycles | -3 to +2 | +70 | +76 | +77 |

- **The transmit path is wire-bound at every rate the generator makes.**
  One DRE interrupt a byte (the TX buffer is one frame ahead of the
  shifter and DREIF rises once a frame), 60 cycles of the 80 a frame
  lasts at 3 Mbaud. A 256-byte run's 1.03 there is its first part's copy
  (about three frames once a run); before the copy went in parts of 32,
  one copy of the whole run held the wire silent for 22 frames - 1.20.
- **The receive path keeps the wire at 2 Mbaud with a consumer
  draining it.** A frame alone in its entry costs 75 cycles (above); at
  3 Mbaud a frame lasts 80, so the handler finishes just before the next
  frame and enters again with one frame - the core is the receiver's
  alone, the thread gets about five cycles a frame, and a consumer
  falls behind the ring while the hardware loses almost nothing (BUFOVF
  on 1 to 6 frames in 256 - consistent with the bench's metered tick
  handler, 302 cycles on its longest pass, outlasting two frames). Bursts
  the ring holds arrive whole. The 3 Mbaud
  rows vary run to run for that reason, and the receive ring's skips
  shape them: a look after a frame the full ring refused, or a BUFOVF,
  discards what the ring holds, so the letter, which counts bytes
  consumed of a stream that does not stop, waits for what comes after.
  With the meters on the vector (the metered binding) the handler
  outlasts a frame at 2 and 3 Mbaud, a hardware overrun falls between
  every two looks, every look skips and nothing is handed out at all -
  the overload in which the skip delivers nothing
  ([../design/ring.md](../design/ring.md)).
- **The burst edge reaches the consumer about 80 cycles after RXCIF**,
  which rises inside the stop bit, at its majority samples: at 115200
  that is the stop bit's end, faster it is a few bit times after it. There is no idle edge to wait for (the
  inventory above): the edge is the burst's first byte.
- **Before the receive path was rewritten** (the resource's
  `receive_as()`, 207 cycles a frame), the same rows read: 1 Mbaud 1.02
  at 256 bytes with the core 83 per cent in the handler, 2 Mbaud not
  measured, 3 Mbaud 25 with the hardware losing frames; the edge 95
  cycles after the stop bit at 115200 and 165 to 173 above it.
- **The data sheet's own sequence** (27.3.2.4: RXDATAH, RXDATAL, the byte
  into a 256-entry ring of 8-bit indices, one frame an entry; no vendor
  library exists for this family) is 65 cycles a frame in its listing,
  against `rxc()`'s 75 with the edge discarded and 83 with it used: the
  edge's fourteen and the loop's re-read of RXDATAH, against the 45 a
  second frame saves when the handler is late. Measured the same way: x 1.00, 1.00 and 1.01 at
  115200, 1 and 2 Mbaud for 256 bytes; at 3 Mbaud the same starved
  consumer (2.9). Its metered body is 104 cycles between the stamps,
  `rxc()`'s 114.
- **`tx_idle()` is the wire's**: on the pad (TCB0 capturing PE0's last
  rising edge, the stop bit of a final 0x00) it turned true 37 CLK_PER
  after the last stop bit's end at 115200 (a bit is 208), 47 at 460800
  (52) and 26 at 1 Mbaud (24) - never before it; the poll's own turn is
  about twenty of those cycles.
- **Errors cost exactly the frames they hit**: 64 slots through the task,
  four of them a break driven on the pad by PORT with the transmitter
  off - 60 bytes delivered intact and in order, the frame-error counter
  4, nothing else moved. Six frames into a receiver with its interrupt
  off: the first entry takes all three the buffer kept (the two oldest
  and the newest, which carries BUFOVF) and counts one hardware overrun.
- **A handler longer than two frame times loses a received byte.** The
  buffer holds two frames and the shifter a third, so the RXC entry may
  wait two frame times: 2.1 ms at 9600, 240 cycles at 2 Mbaud, 160 at
  3 Mbaud. The bench app's metered vectors run 282 to 410 cycles on
  their longest pass; the console's longest is its RXC with the post,
  182 with the response.
- **A second of a foreign stream arrives whole** (`test_avr_serial` B):
  the peer's REPEATED blast - a hundred gapless blocks of ten
  milliseconds each, the counting pattern running on across them, so a
  lost frame anywhere is a jump in the sequence - into `Uart` with a
  consumer over `read_span()`/`consume()`, at 115200, 500 000, 1 000 000
  and 1 500 000 baud (the 16-sample ceiling): every byte in sequence at
  every rate, no skip, no ring or hardware overrun, no frame error, the
  peer's count the same.
- **A stalled consumer costs one skip and one jump, the stall's frames
  exactly.** The same stream at 115200 with the consumer away for 10 ms
  of wall time after 2000 bytes: the ring filled, the rest of the stall's
  frames were refused, and the next look discarded what the ring held -
  ONE `rx_skips`, ONE discontinuity of 115 frames (10 ms / 86.8 us =
  115.2), every byte after it in sequence, delivered plus lost equal to
  what the peer sent. Timed with a cycle-counted `delay_us` the same
  stall cost 120 frames: the receive entries that interrupt the delay
  stretch it.
- **`Uart::init()` holds TXD idle for 10 ms with the receiver already
  live.** Initialized while a stream is already arriving, the task opens
  on a ring that overflowed inside its own init - the first look skips,
  the stream a hundred frames on at 115200. A receiver that must catch a
  stream from its first frame is brought up before the far end starts,
  or moved there with `set_baud()`, which waits only for the transmitter
  (letter B takes its command and the ack through the task itself, then
  `set_baud()`s it to the blast's rate).

On the AVR64EA48 at 20 MHz (rev B1, the same image; its loop is USART1
on its DEFAULT route, TXD PC0, where TCA0's PORTC route puts WO0, and
its top rate CLK_PER/8, 2.5 Mbaud - a frame 200 cycles at 1 Mbaud, 100
at 2 Mbaud and 80 at 2.5):

| op, plain binding | 115200 | 1 Mbaud | 2 Mbaud | 2.5 Mbaud |
|---|---|---|---|---|
| `uart.tx` 256 bytes, x | 1.00 | 1.00 | 1.02 | 1.03 |
| `uart.tx` 4096 bytes, x | 0.99 | 1.00 | 1.00 | 1.01 |
| `uart.rx` 16-byte burst, x | 1.00 | 1.10 | 1.34 | 4.30, a FERR |
| `uart.rx` 256-byte burst, x | 1.00 | 1.00 | 1.08, an overrun | 1.69, an overrun and a FERR |
| receive entries for 256 bytes (counted) | 256 | 257 | 438 for 275 | 247 for 258 |
| the edge after the last stop bit's end, cycles | +10 | +70 | +73 | +74 |

The cycles are the DB's - the same core, the same handlers: the
transmitter's DRE entry 95 cycles metered, a lone frame's receive entry
107, the edge some 80 cycles after RXCIF. What moves is the frame they
are weighed against: 2 Mbaud here is 100 cycles a frame (the DB's 120)
and 2.5 Mbaud 80 (the DB's 3 Mbaud), so the receive path keeps the wire
to 1 Mbaud and starves the consumer from 2 Mbaud, where the DB starved
at 3. The metered binding outlasts a frame from 1 Mbaud (200 cycles a
frame against the stamps' 206 an entry) and, from 2 Mbaud, hands out
nothing, as on the DB at 2 and 3.

### The two boards

- **The DUT discovers the link's topology, and never latches it.** The
  suite supports two wirings - the crossed full-duplex pair and a single
  wire between the two TXD pads - and finds out which one the desk has
  by pinging in each in turn (the shared one first, where its own
  transmitter drives only while it talks); a topology is believed while
  the command channel keeps proving it. A peer whose desk fixes the
  crossed pair - the samc21 one - is full duplex always and answers
  nothing in the shared topology, so the DUT settles on the pair.
- **The two crystals agree to the tick.** The peer's start bit at 9600 -
  the only low pulse of a stream of 0xFF - is 2500 CLK_PER ticks against
  a nominal 2500 in this board's crystal time; `x`'s sixteen nine-bit
  lows at 3750 baud average 57600.8 to 57601.0 ticks against 57600.0
  (spread 57600 to 57601): -14 to -17 ppm, the ruler's one tick. Every
  rate below is therefore a FOREIGN sender exact to the crystal, and
  what a reading departs from is this board's own divisor.
- **The wiring probe names the crossed pair from both ends** (`v` beside
  the peer's own probe): PE0 hears the peer's PE1-role pin, PE1 its
  PE0-role pin, PE2 its XCK and PE3 the pad wire 4 reaches, by the edge
  rate each pin hears (47, 23, 95, 191 edges in six seconds here; the
  peer's pads counted 23, 47, 94, 188).
- **A refused leg is skipped, not judged.** A peer bug that refused the
  synchronous client (its enable on an external clock timed out until
  XCK ran, since fixed) took `q`'s four host legs down the refusal path:
  each printed SKIPPED with the refused op and counted nothing, the
  letter's other three verdicts unaffected.

### Facts measured on the wire

- **The loop-back is taken at the TXD PAD.**
  With PORTMUX at NONE the peripheral still transmits - DREIF, TXCIF and
  the timing all behave - but LBME delivers nothing at all, so the driver
  refuses loop-back and open drain on a pinless route.
- **The baud generator is exact to the tick.** Measuring the start bit
  of a stream of 0xFF frames against the divisor's own nominal
  `S x BAUD / 64` CLK_PER cycles:

  | requested | BAUD | actual | nominal ticks | measured |
  |-----------|------|--------|---------------|----------|
  | 9 600 | 10000 | 9 600 | 2500.0 | 2500 |
  | 115 200 | 833 | 115 246 | 208.2 | 208 |
  | 460 800 | 208 | 461 538 | 52.0 | 52 |
  | 1 000 000 | 96 | 1 000 000 | 24.0 | 24 |
  | 460 800 (CLK2X) | 417 | 460 431 | 52.1 | 52 |

  Every measurement is the truncation of the nominal value: the
  fractional part never showed up as a wandering pulse width at these
  rates.
- **And a run of frames takes its time at the rate ASKED.** The pulse
  above is judged against the divisor, which a wrong BAUD would pass; a
  loop-back cannot tell either, its two ends sharing one register. So
  every single-board letter of `test_avr_serial` that moves frames times
  a run of them on the ruler (TCB1 and TCB2 cascaded at CLK_PER, one
  cycle a tick), from the first store to TXCIF after the last, against
  the run's length at the rate asked - a hundredth of a second of
  frames, 16 at least and 1024 at most - in thousandths:

  | rate asked | path | frames | reading |
  |---|---|---|---|
  | 1465 (the register's floor, BAUD 65529) | the resource, polled | 16 | 1000 |
  | 2400, 9600 | the resource | 24, 96 | 1000, 1000 |
  | 115 200, 230 400 | the resource | 1024 | 999, 1000 |
  | 460 800, 921 600 | the resource | 1024 | 998, 998 |
  | 1 000 000, 1 500 000 (BAUD 64, the 16-sample ceiling) | the resource | 1024 | 1000, 1000 |
  | 460 800, 2 000 000, 3 000 000 (BAUD 64, the CLK2X ceiling), CLK2X | the resource | 1024 | 1001, 1000, 1002 |
  | 115 200, all 36 formats (5 to 9 bits, N/E/O, 1 or 2 stops) | the resource, 256 frames each | 256 | 999 to 1000 of each format's own frame length |
  | 115 200 on USART0, USART1 and USART3, each its own generator | the resource | 1024 | 999 each |
  | 115 200 through `Uart` (write_bulk, tx_idle) at 24, 12 and 24 MHz | the task | 1024 | 999, 1001, 999 |
  | 460 800, 1 000 000 through `Uart` | the task | 1024 | 998, 1001 |
  | 19 200 at the BAUD GENAUTO learned from a sync field sent at 19 200 | the resource | 192 | 1000 |
  | 1 MHz XCK in Host SPI mode, eight periods a frame | the resource | 1024 | 1000 |

  The readings are the fractional generator's own rounding and nothing
  else: BAUD 833 at 115 200 runs 0.04 % fast, 208 at 460 800 and 104 at
  921 600 0.16 % fast (998), the round divisors exact. Host SPI leaves
  frames back to back through its double-buffered transmitter, no gap
  between them. A frame's length really holds its parity bit and its
  second stop bit in every format.
- **The resource's polled `send()` does not keep the line busy at
  3 Mbaud.** One frame there is 80 cycles; `send()` - out of line, its
  flag read a call, its spin bound a 32-bit count and `transmit()` a
  switch on the character size - took 92 a frame (a run read 1149
  thousandths of its wire time), where a loop with the flag read and the
  store inline, some 25 cycles, read 1002. `Uart`'s transmit path is
  wire-bound there (above). At 2 Mbaud and below, 120 cycles a frame and
  more, `send()` keeps up: the same runs through it read as the inline
  loop's, within a thousandth.
  The letters' runs wait for the console to drain before they start: its
  DRE vector, some hundred cycles an entry, is longer than the slack a
  frame leaves a polled loop at the top rates.
- **The rate floor is the register's, not the protocol's.** At
  CLK_PER = 24 MHz, 300 baud is unreachable: BAUD would be 320000,
  nearly five times what the register holds. The slowest expressible
  asynchronous rate at this clock is **1465 baud**.
- **The loop-back receiver hears the OUTSIDE.** With the instance
  routed, a receiver in loop-back reads frames another board drives onto
  that pad: the peer bit-banged 0x5A onto this board's TXD while it held
  the pin as an input under its pull-up (the errata 2.16.1 open-drain
  discipline), and the frames arrived clean (three of them, no FERR).
  One-wire collision detection is therefore electrically real.
- **Every rate the register can express works across the wire.** 2400,
  9600, 115 200, 230 400, 460 800, 921 600 and 1 000 000 in normal mode,
  and 2 000 000 in double speed (BAUD 96, the only way to reach it at
  24 MHz): every frame echoed back matched, with zero FERR, PERR and
  BUFOVF at BOTH ends.
- **Every frame format works across the wire too.** 5N1, 6O2, 7E1, 8O2,
  8E2, 9N1 in both nine-bit orders and **9E1** - parity IS available
  with nine data bits - all round-trip clean at 115 200. And SBMODE is a
  transmitter setting only: one board sending two stop bits into a
  receiver configured for one, and the reverse, are both clean, because
  the receiver never looks past the first stop bit.
- **Where FERR really begins.** Table 27-4 recommends -4.19/+4.14 % for
  D = 9 (8N1). With the peer's USART deliberately off rate and sending
  0x00 - the frame whose stop bit has the least margin - this board
  stayed clean at -5 % and +4 % (where one frame of eight was lost) and
  flagged FERR from **+5 %** and **-6 %**. Bit-banged from the peer's
  crystal with a four-bit idle between frames, the negative side matched
  (**clean to -5 %, FERR at -6 %**) and the positive side never flagged
  up to +6 %: a sender that is too FAST finishes early, so the stop-bit
  sample lands on the idle line, which is high anyway. Only a slow
  sender pushes a low data bit under that sample.
- **A start bit has to last most of a bit.** The peer put low pulses of
  6, 30, 150, 625, 1250, 1562, 1875 and 2500 ticks on an otherwise idle
  line, against a 9600 receiver whose bit is 2500 ticks. Nothing at or
  below **1250 ticks (half a bit)** was taken as a start bit; 1562 was.
  That is the majority vote of samples 8, 9 and 10 of 16 seen from
  outside.
- **One displaced bit boundary is not a rate error.** Stretching a
  single data cell of a bit-banged frame shifts everything after it by
  that much and nothing else: the frame survived +50 % and broke at
  **+60 %**. Ten times the tolerance of a UNIFORM rate error - the
  operational-range tables are about the accumulated error, not about
  jitter.
- **The auto-baud window (CTRLD.ABW) measures a bit PAIR.** In LINAUTO,
  with one cell of the 0x55 sync field stretched (which moves the
  interval between two edges by half that percentage), WDW1 accepted to
  +28 % and refused from +32 %, WDW2 accepted to +36 % and refused from
  +40 %, WDW3 accepted every step to +44 %; WDW0's edge is marginal at
  +32 % (refused there, accepted at +36 % in the same run). Halved, that
  is 14-16 %, 18-20 % and more than 22 % of a bit pair - the documented
  15 %, 21 % and 25 % windows, with WDW0's 18 % between them.
- **Auto-baud against a foreign clock works.** The peer sent a break, a
  0x55 sync field and payload at 9600, 57 600, 230 400 and an odd
  123 456 baud from its own crystal. Both GENAUTO (with WFB armed) and
  LINAUTO learned the rate, and the BAUD they wrote was this board's own
  divisor for the rate in every case (10000, 1667, 417, 778): two
  crystals a tick apart leave nothing for the measurement to see. The
  payload then arrived clean at the learned rate.
- **BDF must be latched, not read afterwards.** The break-detected flag
  is cleared by the next DATA frame, and in a real auto-baud frame the
  payload follows the sync field immediately: a test that reads BDF
  after the payload always reads zero. Poll for it.
- **LINAUTO is constrained to the BAUD already in force.** ABW compares
  the sync field against the current divisor, so LIN auto-baud only
  accepts a sender within its window: with BAUD at 19 200 and a 9600
  sync field, BDF never set and ISFIF did. GENAUTO has no such tie - it
  learned every rate from a 19 200 fallback. A LIN receiver's fallback
  must therefore start near the bus rate; a generic one need not.
- **The documented BAUD floor is not enforced.** 27.3.3.2.5 says a sync
  field is only accepted when it measures 0x0064-0xFFFF. A sync field at
  1.2 Mbaud measures BAUD 80, well under 0x0064, and was accepted: BDF
  set and BAUD read back 80.
- **A sync character that is not 0x55 sets ISFIF in LINAUTO** and leaves
  BDF clear; after `recover()` (errata 2.16.3's RXEN toggle) the very
  next sync field is learned.
- **A break from outside is a frame error with data zero.** Twenty bit
  times of low from the peer arrive at a normal receiver as one frame
  with FERR and 0x00. A LIN header from the SAM C21's host at 19200 - a
  break of 13, 17, 21 or 26 bit times, then 0x55 and the identifier -
  arrives as exactly three frames, the break one 0x00 with FERR: however
  long the low, the receiver starts no second frame inside it
  (`test_samc_uart` O, this board the receiving peer).
- **The receive FIFO keeps three frames, and the third is the NEWEST.**
  Eight frames flooded at full rate with nothing read leave exactly
  three readable: the two oldest and then the LAST frame on the line,
  not the third one sent - the shift register keeps taking new frames
  while the buffer stays full. BUFOVF marks that third frame and only
  it, and draining is the whole recovery. Measured both in loop-back and
  from the other board (eight frames blasted, three drained, the last
  0xB7 with BUFOVF).
- **Parity errors are counted, not delivered.** The peer sending odd
  parity into a receiver configured for even flags PERR on every frame,
  and the data is still readable in the resource's `UsartFrame`; through
  the `Uart` task the same six frames arrive as six parity errors and
  zero delivered bytes.
- **The transmit path is three deep and its stages are microseconds
  apart.** With TXDATA, the buffer and the shift register all loaded,
  DREIF stays low for a whole frame (1021 polling iterations at 9600)
  and TXCIF for two (2095); DREIF returns while the line is still busy.
  With only TWO frames loaded the "TXDATA full" state lasts a few dozen
  CPU cycles - long enough to read once, far too short to survive an
  interrupt - so a measurement of it is a race, not a fact.
- **MPCM filters silently, in both flavours.** With nine data bits the
  ninth marks the address frame: only address frames set RXCIF, clearing
  MPCM opens the flow to that address's data frames, and setting it
  again catches the next group's address. The 5..8-bit flavour of
  27.3.4.3 behaves identically - but only the RECEIVER can be configured
  for it. A transmitter with five to eight data bits sends ones in both
  stop positions and so can only ever produce address frames; the sender
  has to use NINE data bits, and its ninth bit lands in the receiver's
  first stop-bit slot.
- **RS-485's XDIR is exactly guard + frame.** Measured on PE3 with a TCB
  pulse-width meter through the event system, with the peer receiving the
  frames: **27 500 CLK_PER ticks at 9600** against an expected 27 500
  (one baud clock of guard plus ten frame bits, exact) and **2291 at
  115 200** against an expected 2288.
- **IRCOM works on a wire, and TXPL is a TRANSMITTER knob only.** Both
  ends in IRCOM at 115 200 round-trip cleanly with the default 3/16
  pulses - against this desk's peer only with PE1's INVEN, because its
  coder sends a 0 as a HIGH pulse on an idle-LOW line where this one
  sends and takes LOW pulses on an idle-high line (uninverted, every
  echo was wrong while the peer still read this board's frames). With
  TXPL = 0xFF the transmitter really does become plain asynchronous - a
  normal asynchronous receiver on the other board read all six frames
  clean - but the IRCOM RECEIVER still expects pulses, and a plain
  asynchronous sender is NOT readable by it (five echoes of six wrong). A
  pulse-coded transmitter into a plain receiver is not readable either.
- **IREI moves the IRCOM receiver to an event channel, and takes it off
  the pin.** With `IrdaLink::receive_on()` on channel 5 fed by PE1's own
  pin event, the peer's 3/16 pulses read clean - through PE1's INVEN,
  which the event sees as every peripheral on the pin does. Fed instead
  from PE3, an input held idle-high by its pull-up, the same six echoes
  stand on PE1 and not one is read: with IREI set the RxD pin is
  disconnected (27.3.3.2.7).
- **RXPL counts PERIPHERAL CLOCK cycles, not receiver samples.** At
  115 200 on a 24 MHz CLK_PER a bit is 208 peripheral clocks and the
  3/16 pulse is 39 of them. Sweeping this board's RXPL against the peer's
  pulse: 0, 6 and 20 pass; **40, 60, 100 and 127 block completely**.
  Rejection starts exactly where RXPL + 1 exceeds the pulse width in
  CLK_PER, not where it exceeds three of sixteen samples.
- **The one-wire bus works, collision detection included.** On PE0's
  wire - here the crossed pair itself, the peer's RXD pin meeting PE0 -
  with this board in LBME + ODME (the pad an input under its pull-up,
  errata 2.16.1) and the peer pulling the wire low or releasing it, the
  two exchanged frames half duplex at 19 200: every frame came back
  through this board's own echo path and the peer answered each one.
  Then, with the peer commanded to transmit into this board's NEXT frame
  150 us after the trigger, the echo of an all-ones frame differed from
  what was sent - the wired-AND of two open-drain transmitters, seen
  exactly where 27.3.3.2.6 says to look for it. A responder cannot
  collide with the frame that triggered it: it only knows a frame began
  once it has all of it.
- **Both synchronous roles work on a real XCK.** With this board as
  SyncHost driving XCK on PE2 and the peer as a client, eight frames
  arrived with an exact checksum and zero errors at **100 kHz and
  1 MHz**, in **both INVEN phases** (host and client must invert
  together: INVEN on the host's output inverts the physical waveform, so
  a client that does not follow samples on the wrong edge). With the
  roles reversed - the peer driving XCK, this board as SyncClient - the
  same eight frames arrived exactly at both rates. **The client's
  CLK_PER/4 ceiling is real**: at an XCK of 12 MHz, twice the 6 MHz
  ceiling at CLK_PER = 24 MHz, this board read five frames whose checksum
  is wrong (0x1ee and 0x4de against an expected 0x31c) - corrupted,
  never clean.
- **A synchronous client has to read as it goes.** Eight frames do not
  fit in a three-deep receive FIFO, and in synchronous mode they arrive
  as fast as the host clocks them: draining after the fact returns three
  frames whatever the wire did - the FIFO depth above, and the reason a
  client direction looks broken unless the reader collects live.
- **A synchronous host's XCK runs while the instance is enabled**,
  transmitting or not: read from the SAM C21 at the other end of PE2,
  its pad pulled the other way, about half of 200 readings over two
  milliseconds were high at an XCK of 100 kHz, with INVEN and without
  (`test_samc_uart` C, this board the peer, a synchronous host idling).
  An idle host's XCK is no level a far end can lean on.
- **Rebase is transparent, under real traffic too.** 24 -> 12 -> 24 MHz
  with the USART among a `DynamicClock`'s users: loop-back traffic stays
  clean across every switch and the bit time holds at 8.7 us (208
  CLK_PER ticks at 24 MHz, 104 at 12 MHz). Streaming 96 counted bytes
  through the `Uart` task across the same two switches, the other board
  received all 96 with an exact checksum and zero FERR, PERR and BUFOVF;
  its span over the stream, 209 609 ticks against 11 503 for 95 gapless
  frames, is this board's paced writes, not the wire.
- **Host SPI runs in loop-back in all four phase/order combinations**
  and the rate is exactly CLK_PER / (2 x BAUD[15:6]).
- **`release()` really releases**: PORTMUX reads NONE again, the TXD
  pin's direction bit is back to input, and `MspiHost::release()` also
  clears the INVEN `invert_sck` put on the XCK PIN - a PORT bit the
  resource's own teardown cannot know about. Left set, it makes an SPI
  host that takes the same position afterwards clock the opposite
  polarity, and its client stops decoding.

### Start-of-frame detection out of a real standby

Measured by `test_avr_sleep` (set `y`, 10 verdicts on this alone) rather
than by `test_avr_serial`: it needs a device that is genuinely asleep,
and that suite owns the sleep modes. The desk's SAM C21 sent one
commanded byte at a commanded rate on the shared one-wire link
while this board waited in Standby with `SFDEN` armed and `RXSIE` on;
every leg below came out the same in twelve runs of twelve. Three
main-clock restart times are swept - a crystal (1.48 ms on this board),
OSCHF with the regulator asleep (313 us) and OSCHF with `PMODE = FULL`
(23.4 us), all measured in platform.md.

- **It works through LBME.** The detector listens to whatever the
  receiver listens to, so on that wiring - one wire from this board's
  TXD pad (PE0) to the peer's - it is the TXD PAD that wakes the device.
  That is a fact of the wiring, not a promise of chapter 27.
- **With the main clock kept alive, a start bit wakes the device at
  every rate and the WAKING FRAME survives**: 9600, 115200 and 460800
  all wake and all deliver the byte intact.
- **THE DETECTOR DOES NOT LATCH AN EDGE. It needs the line still LOW
  when the peripheral clock comes back.** 27.3.4.2 says the transition
  "powers the oscillator up" and that the frame can be received
  "provided that the baud rate is slow enough concerning the oscillator
  start-up time"; the bench splits that into two independent facts by
  sending 0xFF (which holds the line low for the start bit ALONE) and
  0x00 (nine bit times of low) against each restart time:

  | restart | rate | low time | 0xFF | 0x00 |
  |---|---|---|---|---|
  | 23.4 us | 115200 | 8.7 us / 78 us | no wake | WAKES |
  | 313 us | 9600 | 104 us / 938 us | no wake | WAKES |
  | 1.48 ms | 2400 | 417 us / 3.75 ms | no wake | WAKES |

  So a slow-restarting device wakes or not depending on the BYTE's own
  bit pattern - `RXSIF` is set when the clock returns onto a line that
  is still low, whether that low is the start bit or a run of zero data
  bits. A frame of all ones at a fast rate never wakes it at all.
- **The frame survives only if the clock is back before the MIDDLE of
  the start bit**, where the receiver takes its sample: 23.4 us against
  a 52 us half-bit at 9600 gives an intact byte, while every wake that
  landed later delivered a garbled one (0xE0 for a transmitted 0x00).
- **From IDLE, `RXSIF` never fires.** 27.5.5 says the flag is set when
  the device is in Standby, and it means it: the same leg from Idle
  wakes on nothing and simply receives the byte normally through
  `RXCIF`. Start-of-frame detection buys nothing in Idle, where the
  receiver is running anyway.
- **`SFDEN` is a verb, and the discipline holds**: armed immediately
  before the sleep, disarmed as the first thing after it, with no
  `RXDATA` read in between (errata 2.16.2). The frame is read only once
  the flag is down.
- **On the AVR128DA48** (rev A7, 5 V; ten runs of ten, every leg the same
  each time) the main clock is an external clock with no start-up of its
  own, held up in standby only through OSCHF's `RUNSTDBY`
  ([platform.md](platform.md)). The rule above holds at the DA's restart
  times - OSCHF under FULL and OSCHF alone exactly as on the DB - and adds
  a boundary at each end:
  - with OSCHF held up the start bit wakes the device at every rate and
    the frame survives at 9600 and 115200, but at **460800 it arrives one
    bit late, every time** (0x5A read as 0xAD): the external clock's wake,
    about a microsecond more than awake, is level with that rate's
    1.09 us half-bit;
  - with nothing held up the wake pays the regulator: at 2400 a start bit
    alone (417 us of low) wakes it and BOTH frames survive; at 9600 the
    start bit alone (104 us) and 0x5A (208 us of low) do not wake it,
    nine bit times of zeros do and that frame is garbled. That pins the
    rule down further than the DB's legs could: what wakes the device is
    a start bit the receiver CONFIRMS, the line still low HALF A BIT
    after the clock's return - a high there is a false start, and
    27.3.4.2 sends the device back to standby. 0x5A not waking at 9600
    puts the restart above 208 - 52 = 156 us, both 2400 frames surviving
    puts it below 208 us: the start-of-frame path pays 156 to 208 us,
    more than the 131 us a pin edge pays through the same regulator.
- **On the AVR64EA48** (rev B1, 5 V, the board's 20 MHz quartz; USART1
  on ALT1 under LBME, its TXD PC4 jumpered onto the PE0 net; ten runs of
  ten, every leg the same each time) there is no regulator to restart
  ([platform.md](platform.md)), so the restart times are three: a clock
  kept alive back 0.5 us after the edge, OSCHF's 9.2 us, the crystal's
  0.61..0.67 ms from cold. The rule holds at each, and the start-of-frame
  flag is a STANDBY source only (DS40002443A table 13-3):
  - with the clock kept alive the start bit wakes the device at 9600, at
    the link's 115200 and at the console's 460800, and the frame
    survives at all three - 460800 included, where the DA's arrived a
    bit late: 0.5 us is inside that rate's 1.09 us half-bit;
  - OSCHF restarting: at 9600 a start bit alone (104 us of low) wakes it
    and 0x5A, 0xFF and 0x00 all arrive intact (9.2 us against a 52 us
    half-bit); at 115200 0xFF (8.7 us of low) does NOT wake it, 0x00 (78
    us) does and arrives garbled (0x80), and 0x5A (17.4 us of low) wakes
    it one bit late (0xAD) - the restart outlasts the start bit by half
    a microsecond and is past its middle;
  - the crystal restarting: at 9600 0xFF (104 us) does not wake it, 0x00
    (938 us) does and arrives garbled;
  - from IDLE `RXSIF` never fires and the frame is received, at 115200
    and 460800.

- **On the AVR64EA48** (rev B1, 5 V, CLK_PER 20 MHz from the board's
  quartz). The EA has no USART on PORTE, so `test_avr_serial`'s link is
  USART1 on ALT1 - TXD PC4, RXD PC5, XCK PC6, XDIR PC7 - which the board
  jumpers onto PE0..PE3, the wires to the partner SAM C21 (one driver a
  net: the PE pads stay inputs); `usart_peer` and `bench_avr` take the
  same instance (bench_avr's loop USART1's DEFAULT route, PC0, where TCA0's
  PORTC route puts WO0). Three images (the group axis), each ten runs of
  ten: `test_avr_serial-1` (`z`) 163 of 163, `test_avr_serial-2` (`y`, j..q)
  88 of 88 in all but one run (below), `test_avr_serial-3` (`y`, r..u and
  B) 30 of 30, its `w` 6 of 6 and `x` 3 of 3.
  - **Every route of USART0 and USART1 read back** (letter a): released,
    USARTROUTEA reads NONE as 5 in USART0's three-bit field and 3 in
    USART1's [4:3], and USARTROUTEB holds the console's USART2 ALT1 (code
    1); USART0 ALT1 (TXD PA4), ALT3 (PD4) and ALT4 (PC1), USART1 DEFAULT
    (PC0) and ALT1 (PC4) each read back their code with four frames looped
    through their TXD pad, USART0 ALT2 (PA2/PA3, the I2C bus) and USART1
    ALT2 (PD6/PD7, the analog net) read back routed with both directions
    off, which claims no pin; every release reads NONE. USART0's DEFAULT
    (PA0/PA1, the quartz) and USART2 (the console) are not routed.
  - **The generator at 20 MHz** (letter f): the register's floor is now
    1221 baud (BAUD 65520), the 16-sample ceiling 1.25 Mbaud and CLK2X's
    2.5 Mbaud (BAUD 64), every start bit within a tick of the divisor and
    every run 999 to 1002 thousandths of the rate asked; 921600 is BAUD 87,
    0.22 % slow. Across the wire (letter k) every rate from 2400 to 1
    Mbaud and 2 Mbaud in double speed echoes clean; the repeated blast
    (letter B) arrives whole at 115200, 500 000, 1 000 000 and 1 250 000
    baud. The SAM C21's 24 MHz crystal reads -109 ppm against this
    board's quartz (letter x, at 4000 baud: BAUD 20000 here and 375.0 on
    that peer's fractional generator, both exact).
  - **SFDEN set in Active mode corrupts nothing here** (letter F, the EA's
    own): 256 back-to-back frames on the loop, each read three bit times
    after its RXCIF - inside the next frame's reception - arrive exact
    with SFDEN set as without it, three runs and ten (in `z`). DS80001048D
    has no twin of the DB's 2.16.2 and the silicon agrees; the verb pair
    stays, costing nothing.
  - **The transmitter pulls an INPUT pad low** (letter u): the one-wire
    discipline - TXD an input under its pull-up, ODME set - reads its own
    frames back, and the loop-back receiver hears the peer driving the
    pad. Whether a TXD left an OUTPUT drives high under ODME (the DB's
    2.16.1, absent from DS80001048D) is not measured: every way this desk
    has of telling a driven high from a pull-up's drives a net the
    partner or the analog net holds.
  - **`tx_idle()` turns true 15 to 50 CLK_PER after the last stop bit**
    (letter T; 81 with the PIT's tick handler inside the poll): within a
    bit at 115200 (174), not at 460800, where a bit is 44 CLK_PER at 20 MHz
    - the poll's cost is cycles and the bit is time, so the letter judges
    "within one bit" where a bit outlasts the poll.
  - **Auto-baud and the rest** read as on the DA/DB: the GENAUTO/LINAUTO
    legs, the bit-banged operational range (FERR from -6 % and nothing up
    to +6 %), a stretched cell surviving to +50 %, the synchronous roles,
    RS-485's XDIR guard, IrDA both ways against the SAM C21's coder, and a
    20 -> 10 -> 20 MHz rebase under the peer's count.
  - **One extra 0xFF frame** at the synchronous client's 100 kHz leg
    (letter q), in one run of 31: the client counted nine frames where the
    peer clocked eight. Its source is not measured (a scope on the XCK
    net would say whether an edge reached it before the peer's first).

### Two facts about the flags that shape half-duplex code

- **RXCIF is raised half a bit BEFORE the sender's TXCIF** - at the
  middle of the stop bit, where the receiver samples it, while the
  transmitter only reports done at the end. A responder that answers the
  instant it has the frame therefore starts its start bit while the
  other end is still transmitting, and any scheme that stops listening
  while it talks loses the beginning of that answer: two bits were
  swallowed at 2400 baud and the receiver locked onto a later low. A
  turnaround guard of a few bit times fixes it, and `OneWire`'s header
  comment says so.
- **`wait_line_idle()` CLEARS TXCIF**, so two calls in a row do not both
  return at once: the second spins out its whole budget. A half-duplex
  turnaround that waited twice stayed deaf for 60 ms and lost the
  answer every time.

## Not covered yet

Driver gaps (not implemented):

- **LIN above the auto-baud primitive.** `AutoBaud` exposes GENAUTO and
  LINAUTO arming, the flags and the recovery; the protocol itself -
  the protected identifier's own parity rule (27.3.4.1), the
  "in the response space" meaning RXDATAH bit 0 takes in LINAUTO - is
  not interpreted anywhere. The ninth data bit is handed back raw.
- **Only `Uart` is interrupt driven.** `OneWire`, `Rs485`, `SyncHost`,
  `SyncClient`, `MspiHost`, `IrdaLink` and `AutoBaud` are polled, with
  bounded spins. An interrupt-driven or kernel-resident version of any
  of them is born with its first user, as is an MPCM address-matching
  layer over the resource's filter.
- **No turnaround guard in `OneWire`.** The half-bit lead of RXCIF over
  TXCIF is documented in the task's header comment and measured above,
  but `talk()` does not wait: a responder built on this task must add
  the guard itself until a real half-duplex user says how long it wants.
- **No RUNSTDBY**: the peripheral has none. Standby operation is
  SFDEN's business, and that is a verb, not a mode.

Implemented and compile-checked for all eight DA/DB packages and the
three AVR64EA packages, but NOT bench-verified:

- **the AVR EA's USART0 ALT2 and USART1 ALT2 with traffic**: read back
  routed, never driven - their pads are this board's I2C bus and analog
  net; USART0 DEFAULT and USART2's routes are not routed at all (the
  quartz, the console);
- **ODME with TXD left an output, on the AVR EA**: the DB's 2.16.1 has no
  EA twin, the workaround stays (above); an external pull-down or a scope
  on a free TXD pad would say whether the output drives high;
- **the synchronous client's extra frame** (one run of 31, above): a
  scope on the XCK net;

- **`DBGCTRL.DBGRUN`**;
- **IRCOM's fixed transmit pulse across the wire** (TXPLCTRL other than
  0): this desk's peer transmits 3/16 pulses only, and `s` skips the leg
  by its kind - an avrxt peer would measure it;
- **Host SPI against a real SPI client** (`MspiHost` against
  `avrxt/spi.hpp`'s client, `test_avr_spi` r): this desk's SPI peer is a
  SAM C21, whose SERCOM is not a USART while it is an SPI - an avrxt SPI
  peer would measure it;
- **routes**: USART0 ALT1, USART1 default, USART3 ALT1 and USART4
  default were driven, on 48-pin silicon only. USART2 is the bench
  console and the suites never reconfigure it.
