# USART - the serial port (AVR DA/DB)

> **PROVISIONAL.** The chapter's register description is covered in full
> and almost everything is now bench-verified against a second board -
> cross frame formats, injected errors, a foreign auto-baud sender, both
> synchronous roles on a real XCK, RS-485 drive-enable timing, IrDA
> pulses, MPCM and the one-wire bus, Host SPI against a real SPI client,
> and start-of-frame detection out of a real standby. What is still
> unmeasured is a handful of routes and two DBGRUN-class knobs; the LIN
> protocol layer is not built. The list is in "Not covered yet".

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (USART
chapter 27, PORTMUX chapter 17, electricals 39.14), errata
DS80000915F 2.16.1-2.16.3 and, for the DA parts, DS80000882C
2.15.1-2.15.2. Three errata items shape the code:

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

Driver: `avrdx/usart.hpp`. The line service above it:
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
| `Uart<n, Route, rx, tx>` | the interrupt-driven transport: `init(clock, baud)`, `rebase(hz)`, `can_baud`, `min_hz_for`, `actual_baud(hz)`, `write_byte`/`read_byte`/`write`, `rx_pending`/`tx_idle`, the error counters, ISR bodies `rxc()` (returns the empty -> non-empty edge) and `dre()` |
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

## How to use it

A console (the shape every app uses):

```cpp
using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;
ISR(USART2_RXC_vect) { if (Serial::rxc()) brio::post<Lines>(brio::RxActivity{}); }
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

Two suites, one driver. `test_avr_serial` runs on board A (`brio-a`,
AVR128DB48 rev. A5, 24 MHz crystal, 5 V) and has two halves: `z` is the
SINGLE-BOARD set (9 tests, 108 verdicts, 108/108, nothing to wire) and
`y` is the TWO-BOARD set (12 tests, 103 verdicts, 103/103) against board
B running `usart_peer`, an instrument driven IN BAND over the very link
under test. Two more commands stand outside `y` because they depend on
how the desk is jumpered: `v`, the wiring probe, and `w`, the one-wire
bus (6/6 on the shared-line desk; it skips itself on the crossed pair).

### The two boards, and what they cost the measurements

- **The two boards' clocks agree to the tick**: a stream of 0xFF frames
  at 9600 puts board B's start bit - the only low pulse on the line - at
  **2500 CLK_PER ticks against a nominal 2500**, measured in board A's
  crystal time. The auto-baud findings below were measured against a
  genuinely foreign clock: board B on its internal OSCHF, **+0.24 to
  +0.28 % fast** - the offset every learned BAUD reproduced. (Running a
  board on OSCHF - deliberately, or via `xtal_probe` when a crystal is
  in doubt - recreates that condition at will.)
- **The link topology is discovered, not assumed, and never latched.**
  The apps support two wirings - the crossed full-duplex pair and a
  single wire between the two TXD pads - and find out which one the desk
  has by alternating their command-mode configuration until a frame
  arrives. A topology is believed only while the command channel keeps
  proving it: after `rediscover_ms` (3 s) of silence the peer stops
  believing and goes back to alternating, which is what lets the desk be
  re-jumpered under two running firmwares. While it is still guessing
  the peer drives NOTHING - a probe that idled its TXD high would, on a
  shared desk, fight the other board's transmitter for as long as it
  guessed wrong. The figures below were taken on the crossed pair;
  everything except the synchronous roles was also measured half duplex
  through the single shared wire, with LBME at both ends.

### Facts the campaign measured

- **The loop-back is taken at the TXD PAD, and it hears the OUTSIDE.**
  With PORTMUX at NONE the peripheral still transmits - DREIF, TXCIF and
  the timing all behave - but LBME delivers nothing at all, so the driver
  refuses loop-back and open drain on a pinless route. And with the
  instance routed, a receiver in loop-back reads frames that ANOTHER
  BOARD drives onto that pad: board B bit-banged 0x5A onto board A's TXD
  while board A held the pin as an input under its pull-up (the errata
  2.16.1 open-drain discipline) and every frame arrived clean. One-wire
  collision detection is therefore electrically real, not just a
  register feature.
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
- **The rate floor is the register's, not the protocol's.** At
  CLK_PER = 24 MHz, 300 baud is unreachable: BAUD would be 320000,
  nearly five times what the register holds. The slowest expressible
  asynchronous rate at this clock is **1465 baud**.
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
  D = 9 (8N1). With board B's USART deliberately off rate and sending
  0x00 - the frame whose stop bit has the least margin - board A stayed
  clean at +4 % and -5 % and flagged FERR on every frame from **+5 %**
  and **-6 %**. Bit-banged from board B with a four-bit idle between
  frames, the negative side matched (**clean to -4 %, FERR from -5 %**)
  and the positive side never flagged at all up to +6 %: a sender that
  is too FAST finishes early, so the stop-bit sample lands on the idle
  line, which is high anyway. Only a slow sender pushes a low data bit
  under that sample.
- **A start bit has to last most of a bit.** Board B put low pulses of
  6, 30, 150, 625, 1250, 1562, 1875 and 2500 CPU cycles on an otherwise
  idle line, against a 9600 receiver whose bit is 2500 cycles. Nothing
  at or below **625 cycles (a quarter bit)** was ever taken as a start
  bit; the boundary sits at **half a bit**, where it is marginal -
  across runs the narrowest accepted pulse was sometimes 1250 cycles and
  sometimes 1562. That is the majority vote of samples 8, 9 and 10 of 16
  seen from outside.
- **One displaced bit boundary is not a rate error.** Stretching a
  single data cell of a bit-banged frame shifts everything after it by
  that much and nothing else: the frame survived +40 % and broke at
  **+50 %**, half a cell, which is exactly where the stop bit's sample
  point crosses into its neighbour. Ten times the tolerance of a
  UNIFORM rate error, and a useful reminder that the operational-range
  tables are about the accumulated error, not about jitter.
- **The auto-baud window (CTRLD.ABW) measures a bit PAIR.** In LINAUTO,
  with one cell of the 0x55 sync field stretched (which moves the
  interval between two edges by half that percentage), the acceptance
  edge sat between +28 and +32 % for WDW1, +32 and +36 % for WDW0, +40
  and +44 % for WDW2, and beyond +44 % for WDW3. Halved, that is
  14-16 %, 16-18 %, 20-22 % and >22 % of a bit pair - the documented
  15 %, 18 %, 21 % and 25 % windows, measured.
- **Auto-baud against a foreign clock works, and measures it.** Board B
  sent a break, a 0x55 sync field and payload at 9600, 57 600, 230 400
  and an odd 123 456 baud, from its own oscillator. Both GENAUTO (with
  WFB armed) and LINAUTO learned the rate, and the BAUD they wrote was
  consistently **-0.17 to -0.25 %** away from the value board A computes
  for the same nominal rate - exactly the sender's OSCHF offset measured
  above, and nothing else. The payload then arrived clean at the learned
  rate.
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
  times of low from board B's PORT arrive at a normal receiver as one
  frame with FERR and 0x00.
- **The receive FIFO keeps three frames, and the third is the NEWEST.**
  Eight frames flooded at full rate with nothing read leave exactly
  three readable: the two oldest and then the LAST frame on the line,
  not the third one sent - the shift register keeps taking new frames
  while the buffer stays full. BUFOVF marks that third frame and only
  it, and draining is the whole recovery. Measured both in loop-back and
  from the other board.
- **Parity errors are counted, not delivered.** Board B sending odd
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
  pulse-width meter through the event system, with board B receiving the
  frames: **27 500 CLK_PER ticks at 9600** against an expected 27 500
  (one baud clock of guard plus ten frame bits, exact) and **2290-2291
  at 115 200** against an expected 2288.
- **IRCOM works on a wire, and TXPL is a TRANSMITTER knob only.** Both
  ends in IRCOM at 115 200 round-trip cleanly with the default 3/16
  pulses and with a fixed 60-CLK_PER pulse. With TXPL = 0xFF the
  transmitter really does become plain asynchronous - a normal
  asynchronous receiver on the other board read all six frames clean -
  but the IRCOM RECEIVER still expects pulses, and a plain asynchronous
  sender is NOT readable by it. A pulse-coded transmitter into a plain
  receiver is not readable either.
- **RXPL counts PERIPHERAL CLOCK cycles, not receiver samples.** At
  115 200 on a 24 MHz CLK_PER a bit is 208 peripheral clocks and the
  3/16 pulse is 39 of them. Sweeping the DUT's RXPL against that pulse:
  0, 6 and 20 pass; **40, 60, 100 and 127 block completely**. Rejection
  starts exactly where RXPL + 1 exceeds the pulse width in CLK_PER, not
  where it exceeds three of sixteen samples.
- **The one-wire bus works, collision detection included.** With the two
  TXD pads jumpered and both boards in LBME + ODME (pads left as inputs
  under their pull-ups, errata 2.16.1), the DUT and board B exchanged
  frames half duplex at 19 200 with an explicit talk/listen turnaround:
  every frame came back through the DUT's own echo path and board B
  answered each one. Then, with board B commanded to transmit into the
  DUT's NEXT frame 150 us after the trigger, the echo of an all-ones
  frame differed from what was sent - the open-drain wired-AND of two
  transmitters, seen exactly where 27.3.3.2.6 says to look for it. A
  responder cannot collide with the frame that triggered it: it only
  knows a frame began once it has all of it.
- **Both synchronous roles work on a real XCK.** With the DUT as
  SyncHost driving XCK on PE2 and board B as a client, eight frames
  arrived with an exact checksum and zero errors at **100 kHz and
  1 MHz**, in **both INVEN phases** (host and client must invert
  together: INVEN on the host's output inverts the physical waveform, so
  a client that does not follow samples on the wrong edge). With the
  roles reversed - board B driving XCK, the DUT as SyncClient - the same
  eight frames arrived exactly at both rates. **The client's CLK_PER/4
  ceiling is real**: at an XCK of 12 MHz, twice the 6 MHz ceiling at
  CLK_PER = 24 MHz, the DUT reads seven or eight frames whose checksum
  is wrong every time (0x351, 0x452, 0x2b7 against an expected 0x31c) -
  corrupted, never clean, run after run. Note that 12 MHz is the only
  rate above the ceiling this clock can produce: with S = 2 the host's
  divisor is an integer, so the achievable XCK rates near the top are
  CLK_PER/2 = 12 MHz, CLK_PER/4 = 6 MHz, CLK_PER/6 = 4 MHz.
- **A synchronous client has to read as it goes.** Eight frames do not
  fit in a three-deep receive FIFO, and in synchronous mode they arrive
  as fast as the host clocks them: draining after the fact returns three
  frames whatever the wire did. This is not a silicon fact, it is the
  FIFO depth above - but it is the one that made the client direction
  look broken until the suite collected live.
- **Rebase is transparent, under real traffic too.** 24 -> 12 -> 24 MHz
  with the USART among a `DynamicClock`'s users: loop-back traffic stays
  clean across every switch and the bit time holds at 8.7 us (208
  CLK_PER ticks at 24 MHz, 104 at 12 MHz). Streaming 96 counted bytes
  through the `Uart` task across the same two switches, the other board
  received all 96 with an exact checksum and zero FERR, PERR and BUFOVF.
- **Host SPI runs in loop-back in all four phase/order combinations**
  and the rate is exactly CLK_PER / (2 x BAUD[15:6]).
- **Host SPI talks to a real SPI client exactly**, in all four
  phase/polarity combinations and in both bit orders, twelve bytes each
  way at 750 kHz - `MspiHost` on USART4 (TXD PE0, RXD PE1, XCK PE2)
  against `avrdx/spi.hpp`'s client on SPI0 ALT1, straight across the
  desk's PORTE link (`test_avr_spi` test `r`). `sample_trailing` (UCPHA)
  and `invert_sck` map onto the SPI chapter's CPHA and CPOL bit for bit,
  so `SpiMode` 0..3 = `{invert_sck, sample_trailing}`, and `lsb_first`
  (UDORD) onto DORD. There is no client select in this mode: the client
  at the other end has to select itself.
- **`release()` really releases**: PORTMUX reads NONE again, the TXD
  pin's direction bit is back to input, and `MspiHost::release()` also
  clears the INVEN `invert_sck` put on the XCK PIN - a PORT bit the
  resource's own teardown cannot know about. Measured the hard way: with
  it left set, an SPI host that took the same position afterwards
  clocked the opposite polarity and its client stopped decoding.

### Start-of-frame detection out of a real standby

Measured by `test_avr_sleep` (set `y`, 10 verdicts on this alone) rather
than by `test_avr_serial`: it needs a device that is genuinely asleep,
and that suite owns the sleep modes. Board B sends one commanded byte at
a commanded rate on the shared one-wire link while board A waits in
Standby with `SFDEN` armed and `RXSIE` on. Three main-clock restart
times are swept - a crystal (1.77 ms), OSCHF with the regulator asleep
(313 us) and OSCHF with `PMODE = FULL` (23.6 us), all measured in
platform.md.

- **It works through LBME.** The detector listens to whatever the
  receiver listens to, so on this desk - one wire between the two TXD
  pads - it is the TXD PAD that wakes the device. That is a fact of this
  wiring, not a promise of chapter 27.
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
  | 23.6 us | 115200 | 8.7 us / 78 us | no wake | WAKES |
  | 313 us | 9600 | 104 us / 938 us | no wake | WAKES |
  | 1.77 ms | 2400 | 417 us / 3.75 ms | no wake | WAKES |

  So a slow-restarting device wakes or not depending on the BYTE's own
  bit pattern - `RXSIF` is set when the clock returns onto a line that
  is still low, whether that low is the start bit or a run of zero data
  bits. A frame of all ones at a fast rate never wakes it at all.
- **The frame survives only if the clock is back before the MIDDLE of
  the start bit**, where the receiver takes its sample: 23.6 us against
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

### Two facts about the flags that shape half-duplex code

- **RXCIF is raised half a bit BEFORE the sender's TXCIF** - at the
  middle of the stop bit, where the receiver samples it, while the
  transmitter only reports done at the end. A responder that answers the
  instant it has the frame therefore starts its start bit while the
  other end is still transmitting, and any scheme that stops listening
  while it talks loses the beginning of that answer: two bits were
  swallowed at 2400 baud and the receiver locked onto a later low. A
  turnaround guard of a few bit times fixes it, and `OneWire`'s header
  comment now says so.
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

Implemented and compile-checked for all eight DA/DB packages, but NOT
bench-verified:

- **`DBGCTRL.DBGRUN`**;
- **the IRCOM receiver fed from an event channel** (`EVCTRL.IREI`);
- **routes**: USART0 ALT1, USART1 default, USART3 ALT1 and USART4
  default were driven, on 48-pin silicon only. USART2 is the bench
  console and the suites never reconfigure it.
