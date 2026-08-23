# USART - the serial port (AVR DA/DB)

> **PROVISIONAL.** The chapter's register description is covered in
> full and the asynchronous side is bench-verified, but half of what a
> USART does needs a SECOND device on the wire: cross frame formats,
> deliberate error injection, a foreign auto-baud sender, the
> synchronous roles, a real one-wire bus, RS-485 drive-enable timing,
> IrDA pulses. All of that is implemented and compile-checked for the
> whole family, and none of it is measured. The list is in "Not
> covered yet".

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
| `MspiHost<n, route>` | `available`, `init(clock, sck_hz, {lsb_first, sample_trailing, invert_sck})`, `transfer(byte)` |
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

Waking from standby on a start bit (the SFDEN erratum's discipline):

```cpp
U::arm_start_of_frame();            // ONLY just before sleeping
U::enable_rxs_interrupt(true);
sleep();
U::disarm_start_of_frame();         // first thing on the way back
```

## Bench findings

`test_avr_serial` (AVR128DB48 rev. A5, 24 MHz crystal, 5 V; 9 tests,
108 verdicts, 108/108). No wires: the loop-back tests run on USART4 at
its default position with LBME, and the baud measurement reads TXD
(PE0) back through the event system into a TCB pulse-width meter.

- **The loop-back is taken at the TXD PAD.** With PORTMUX at NONE the
  peripheral still transmits - DREIF, TXCIF and the timing all behave -
  but LBME delivers nothing at all: every frame of the format matrix
  failed until the instance was routed. The driver now refuses
  loop-back and open drain on a pinless route.
- **The baud generator is exact to the tick.** Measuring the start bit
  of a stream of 0xFF frames (the only low pulse on the line) against
  the divisor's own nominal `S x BAUD / 64` CLK_PER cycles:

  | requested | BAUD | actual | nominal ticks | measured |
  |-----------|------|--------|---------------|----------|
  | 9 600 | 10000 | 9 600 | 2500.0 | 2500 |
  | 115 200 | 833 | 115 246 | 208.2 | 208 |
  | 460 800 | 208 | 461 538 | 52.0 | 52 |
  | 1 000 000 | 96 | 1 000 000 | 24.0 | 24 |
  | 460 800 (CLK2X) | 417 | 460 431 | 52.1 | 52 |

  Every measurement is the truncation of the nominal value: the
  fractional part never showed up as a wandering pulse width at these
  rates. The suite's tolerance is +-2 ticks.
- **Every frame format round-trips.** All 36 combinations of 5, 6, 7,
  8, 9BITL and 9BITH data bits x no/even/odd parity x one/two stop bits
  carry 0x000, all-ones, 0x155 and 0x0AA masked to the size, and the
  ninth bit, with FERR, PERR and BUFOVF clean throughout.
- **The receive FIFO keeps three frames, and the third is the NEWEST.**
  Six frames sent with nothing read leave exactly three readable: the
  two oldest (the buffer) and then the LAST frame on the line, not the
  third one sent - the shift register keeps taking new frames while the
  buffer stays full. BUFOVF marks that third frame and only it, and
  draining is the whole recovery.
- **DRE and TXC mean different things.** Two frames written back to
  back leave DREIF low for only a few instructions (three slots:
  TXDATA, the buffer, the shifter) and it returns while the line is
  still busy; TXCIF stays low until the last frame has left - at 9600
  about 2300 polling iterations later.
- **MPCM filters silently.** In nine-bit mode a data frame (bit 8 = 0)
  never sets RXCIF; an address frame (bit 8 = 1) arrives; clearing MPCM
  opens the flow to data frames and setting it again closes it.
- **Auto-baud measures a BAUD register value directly.** GENAUTO with
  WFB armed accepts a plain 0x00 frame as the break: at a sender's
  19200 baud (BAUD 5000) the sync field wrote 5003, +0.06 %, with BDF
  set and no error - so the counter's result is a BAUD value, not the
  eight bit times it was measured over. Since transmitter and receiver
  share BAUD, the link keeps round-tripping at the new rate.
- **The receiver must be put in loop-back in the SAME store.**
  Configuring auto-baud first and adding LBME afterwards leaves the
  receiver listening to a floating RXD pin for a moment, and a noise
  edge there consumes the armed WFB: about half the runs saw no break
  at all. Setting `loop_back` in the config that starts the instance is
  deterministic.
- **LINAUTO wants a real break.** A 0x00 frame (nine low bit times) is
  not enough: nothing is detected. Holding TXD low for sixteen bit
  times from PORT with the transmitter disabled IS accepted, and then a
  sync character other than 0x55 sets ISFIF, as documented.
- **Errata 2.16.3 confirmed.** With ISFIF set, clearing the flag,
  restoring RXMODE and restoring BAUD still leave the receiver
  completely deaf; writing RXEN 0 then 1 brings it back on the next
  frame.
- **Rebase is transparent.** 24 -> 12 -> 24 MHz with the USART among a
  `DynamicClock`'s users: loop-back traffic stays clean across every
  switch and the bit time holds at 8.7 us (208 CLK_PER ticks at 24 MHz,
  104 at 12 MHz).
- **Host SPI runs in loop-back in all four phase/order combinations**
  and the rate is exactly CLK_PER / (2 x BAUD[15:6]).
- **`release()` really releases**: PORTMUX reads NONE again and the TXD
  pin's direction bit is back to input, so another peripheral can take
  the position.

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
- **No RUNSTDBY**: the peripheral has none. Standby operation is
  SFDEN's business, and that is a verb, not a mode.

Implemented and compile-checked for all eight DA/DB packages, but NOT
bench-verified (most of it waits for the second board of the protocol
campaign):

- everything that needs a foreign device: cross frame formats, framing
  and parity errors injected on purpose, a break received from outside,
  a real auto-baud sender, double-speed RECEPTION (only the generator
  was measured), the operational-range tables of 27.3.3.2.3;
- the synchronous roles on a real XCK, in both host and client
  direction and both INVEN phases, and the client's CLK_PER/4 ceiling;
- Host SPI electrically - deferred to the SPI campaign, where a real
  client will compare it against `avrdx/spi.hpp`;
- one-wire on an actual shared line (two drivers, collision detection
  through the echo) and the errata 2.16.1 pin-direction work around as
  a measured fact rather than a configured one;
- RS-485: the XDIR guard time and its fall after the stop bit, on a
  scope;
- IRCOM entirely: the 3/16 and fixed pulse shapes, the RXPL filter, the
  event-channel input;
- start-of-frame detection from a real standby (needs a sleeping app),
  and with it the RXSIF path;
- `CTRLD.ABW`, the LIN auto-baud tolerance window: written, never
  exercised, and the 0x0064 lower bound the data sheet gives for an
  accepted sync field is not checked by the driver either;
- `DBGCTRL.DBGRUN`;
- routes: only USART0 ALT1, USART1 default, USART3 ALT1 and USART4
  default were driven, on 48-pin silicon only. USART2 is the bench
  console and the suite never reconfigures it.
