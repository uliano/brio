# The bench

The board, the wiring and the apps as they are TODAY. This page is the
volatile end of the documentation: apps are disposable tools that test
the framework's ideas and will not survive in their current form;
nothing in `docs/design/` or in the target pages depends on them.
Every app documents itself in its own header comment - this table is
only the map.

## Boards

Two boards of the same model sit on the desk, indistinguishable by
hardware (same chip, serial-less CH340): each carries its name in its
USERROW ([avrdx/userrow.md](avrdx/userrow.md)) - **A = `brio-a`**, the
DUT; **B = `brio-b`**, the instrument peer. The suites print the label
in their banner, so a console names its own board.

- MCU: AVR128DB48 (48-pin, 128 KB flash, 16 KB SRAM), silicon rev A5
  on both, see [avrdx/README.md](avrdx/README.md) for toolchain, probe
  and clock.
- Both 24 MHz crystals run, and the two boards **agree to the tick**
  (`test_avr_serial` test `j`: board B's start bit at 9600 measures
  2500/2500 CLK_PER ticks in board A's crystal time). A board whose
  crystal is in doubt gets `xtal_probe` (below); the suites and the
  peer print XTAL/OSCHF in their banner at every reset, so a clock
  fallback is never silent.
- Supply: jumper-selectable **3.3 V / 5 V**; VDDIO2 is powered, so
  PORTC (the MVIO domain) is usable - and could one day talk 3.3 V
  logic while the rest of the chip runs 5 V, no level shifters.
- Clock: 24 MHz crystal on PA0/PA1 (not GPIO). RTC on internal OSC32K
  (no 32 kHz crystal, PF0/PF1 free).
- Serial link: CH340 on **USART2 ALT1, PF4/PF5**, 460800 baud in the
  console apps.
- Programmer/debugger: one Atmel-ICE per board over UPDI, AVR port,
  6-pin ISP header (pin 2 VCC, 5 UPDI, 6 GND); A's probe is
  `J42700051207`, B's `J42700049508`. The probe-to-board pairing is the
  UPDI cable and the console-to-board pairing the USB socket, so both
  are facts about the desk, not about the chips: `tools/bench_boards.py`
  is the truth and the USERROW label in each banner is the cross-check.

## Multi-board bench

The protocol work (USART/SPI/TWI) needs two chips talking: **board A =
the DUT** running a `test_avr_*` suite, **board B = a scriptable
instrument peer** (clock stretching, NACK injection, arbitration, a
foreign sender for auto-baud). The CH340 consoles are **observability
only** - firmware always goes in over UPDI.

Three concerns, deliberately kept apart:

1. **Build** - one PlatformIO env per app x board **TYPE**, generated
   into `apps.ini` by `tools/gen_apps.py`. An app that must build for
   more than the bench chip says so in its header:
   `// pio: boards = db28,db32,db48`, and gets `[env:<app>-db28]` +
   `[env:<app>-db28-debug]` per extra type (`db48` is the default and
   stays the bare `[env:<app>]`; env names may only contain
   `[A-Za-z0-9_-]`, hence the `-db28` suffix). Board files:
   `boards/AVR128DB28.json`, `AVR128DB32.json`, `AVR128DB48.json`.
   Never an env per physical board. `family_probe` is the carrier of
   the matrix and the first firmware to flash onto a new board.
2. **Identity** - `tools/bench_boards.py`, the bench **manifest**: a
   plain dict naming each board on the desk ("A", "B" = desk
   positions), its board type, the USERROW label it is expected to
   carry (`id`), its console and its programmer.
3. **Orchestration** - `tools/bench.py`, which resolves 1 against 2.

Consoles are addressed by **`/dev/serial/by-path`**: these CH340
bridges carry no unique USB serial number (`iSerial` = 0, so every one
of them appears as `usb-1a86_USB_Serial-if00-port0` under
`by-id` and a second board would collide), while the by-path name
encodes the USB topology and is therefore stable per physical socket.
The manifest thus documents the desk's wiring; re-plugging a board into
another socket means editing it. Programmers are the opposite case:
EDBG-class probes do have serials, passed as `-P usb:<serial>` -
mandatory with two identical probes attached, or avrdude takes
whichever enumerates first; SerialUPDI adapters would be addressed by
their by-path port. The probe-to-board pairing is the UPDI cable, so
it survives any USB re-plug and changes only when a cable moves - and
the USERROW label in the suite banner is the cross-check that catches
a miswired desk by eye.

```bash
PY=~/.platformio/penv/bin/python        # pyserial lives in PlatformIO's venv
$PY tools/bench.py list                 # devices, probes, manifest (console present?)
$PY tools/bench.py flash A test_avr_pin # pio run -e <env> + avrdude over UPDI
$PY tools/bench.py run A a              # drive the console, judge "ALL: N pass, M fail"
$PY tools/bench.py console A            # print device path + speed (attach a monitor)
$PY tools/bench.py duo A:a B:script.txt # instrument peer scripted, then the DUT
```

`run` exits nonzero on a timeout or a nonzero fail count, so a suite is
usable from a script. `duo` (console-scripting the peer) is **still
unexercised**: both protocol campaigns command their peer IN-BAND over
the link under test (`src/apps/usart_link.hpp`,
`src/apps/spi_link.hpp`), so the whole matrix runs as a plain `run A`.
`duo` stays for a campaign whose bus cannot carry its own commands.

**Today's end state: board A runs `test_avr_twi`, board B runs
`twi_peer`.** The TWI campaign's two-board half needs no wire of its
own - both boards already tap the desk's one I2C bus - and it commands
board B IN BAND over that bus, so the whole matrix runs as a plain
`run A y`. `test_avr_twi z` still scores its full 175 with the peer
attached and running, because the peer's command-mode client answers one
address exactly and the single-board half never sends it.
The board-to-board PORTE link is NOT fitted at the moment (see below), so
the two-board halves of the USART and SPI suites (`y`) need three or four
jumpers back AND their own peer firmware on board B before they can run;
both suites' single-board halves (`z`) need no wire at all.

## Wiring

**The desk runs at 5 V today** (VDDIO2 from the same rail), which is
what the protocol campaigns need. The 3.5" display module and the
MCP3550/MCP47CVB22 pair have no level shifter on their signals, so they
are DISCONNECTED while the rail is at 5 V and their apps
(`display_*`, `spi_duo`, `spi_paint`, `dac_adc`, `mcp_diag`,
`i2c_scan`) are compile-regressions only; the rows below record the
cabling they expect when the rail goes back to 3.3 V.

| Signal | Pin | Notes |
|--------|-----|-------|
| SPI0 MOSI / MISO / SCK | PA4 / PA5 / PA6 | shared bus |
| Display CS / RS(DC) / RST | PD0 / PD1 / PD2 | ILI9481 |
| SD_CS (module) | PD4 | reserved, unused |
| T_CS (touch) | PD5 | XPT2046, same bus, PEN unused |
| MCP3550 CS | PB0 | |
| TWI0 SDA / SCL | PA2 / PA3 | the I2C bus, today 1.5k pull-ups to **5 V** - see "The I2C bus" below |
| MCP47CVB22 | I2C 0x60 | A0 and LAT/HVC to GND (LAT transparent); VOUT0 -> MCP3550 input |
| LED (blink apps) | PF2 | PF2 -> ~330 ohm -> LED -> GND |
| Traffic bench: LED1 R/G/B, LED2 R/G/B | PB0/1/2, PB3/4/5 | common cathode, TCA1 WO0-5 (PWM from traffic2) |
| Traffic bench: LED3 R/G/B, LED4 R/G/B | PC0/1/2, PC3/4/5 | common cathode, TCA0 WO0-5 |
| Traffic bench: buttons 0..3 | PA2..PA5 | to GND, internal pull-ups |
| Event probes (events0) | PD2 (EVOUTD), PC2 (EVOUTC), PF2 (EVOUTF = LED) | logic analyzer on PD2/PC2 |
| CLKOUT (test_avr_clock) | PA7 | scope: CLK_PER |
| Analog loop (test_avr_analog, sampler) | PD6 (DAC0 OUT) -> PD1 (AIN1), PD6 -> PD7 (VREFA) | two jumper wires, NOT fitted at the moment: the wire-dependent half of test_avr_analog (36 of 81) is expected to fail until they return |
| Board-to-board PORTE link (test_avr_serial + usart_peer, and the SPI campaign) | **not fitted today** - the four wires were removed with the desk's move to the I2C bus | see "the board-to-board link" below |

### The I2C bus (5 V rail)

The TWI campaign's desk is ONE open-drain bus with **1.5k pull-ups to
+5 V**, and both boards run at 5 V with VDDIO2 powered from the same
rail (PORTC, the MVIO domain, is therefore usable at bus level):

| Node | Taps |
|------|------|
| SDA | A.PA2 + A.PC2 + A.PB2 + B.PA2 |
| SCL | A.PA3 + A.PC3 + A.PB3 + B.PA3 |

A.PA2/PA3 is TWI0's DEFAULT (and ALT1) pin pair - the host, and the
client of the combined loop. A.PC2/PC3 is that route's DUAL pair, so the
same instance's client can be moved onto it while the host keeps the main
pair; with Dual mode off that pair is plain GPIO on the same node, which
is what `test_avr_twi h` and `n` bit-bang as a foreign agitator (a
foreign START, a foreign STOP, and the Busy bus that makes two boards'
held STARTs fire on the same edge). B.PA2/PA3 is board B's own TWI0
DEFAULT pair: with `twi_peer` on it that board is a real second device -
a client, and for the arbitration case a second HOST. A.PB2/PB3 is the
fourth tap: TWI1 ALT2's primary pair (and TWI1 DEFAULT's dual pair), so
TWI1 can join the same bus electrically. The suite probes for this tap
and prints its verdict rather than assuming (`test_avr_twi h`); it
currently measures PRESENT.

A dedicated GND wire ties the two boards directly (besides the path
through the two USB cables) - the short return the bus needs at 1 MHz.

### The board-to-board link, and its two wirings

The USART campaign supports two ways of jumpering PORTE, and the two
apps FIND OUT which one is on the desk instead of assuming (each
alternates its command-mode configuration until a frame arrives):

- **the crossed full-duplex pair** - `A.PE0 - B.PE1`, `A.PE1 - B.PE0`,
  `A.PE2 - B.PE2`: TXD into RXD each way plus XCK across. This is the
  only wiring that can carry the SYNCHRONOUS roles, and the suite's
  test `q` skips itself without it;
- **one shared wire** - `A.PE0 - B.PE0`, the two TXD pads tied together:
  the one-wire bus of 27.3.3.2.6. With LBME at both ends it carries
  everything else, half duplex, at every rate the register can express,
  and it is the only wiring test `w` can run on.

**The desk carries NO PORTE link today** - the four wires came off when
the I2C bus went on. Both two-board halves therefore wait for jumpers:
`test_avr_serial y` and `test_avr_spi y` need them back, `w`, `q`, `v`
and `x` with them. Nothing else does: the single-board halves (`z`) of
both suites, and the whole TWI suite, run on one board.

The last wiring the desk carried was STRAIGHT THROUGH, `A.PEn - B.PEn`
for n = 0..3. For the USART campaign that is the shared wiring - A.PE0
and B.PE0 are the two TXD pads tied together - so the suite ran `w`
(6/6) while `q`, the synchronous roles, skipped itself, and `y` scored
103 verdicts instead of the crossed pair's 110. PE3 was wired too, which
the RS-485 test does not want (it wants XDIR DUT-local) but which does
not disturb it, since XDIR only drives into the other board's input.

For the SPI campaign the same four wires are exactly what a host and a
client need on SPI0 ALT1: MOSI PE0, MISO PE1, SCK PE2, SS PE3, straight
across - no re-jumpering between the two campaigns, and no topology to
discover either, since an SPI bus is not symmetric. The same three pins
carry USART4's Host SPI mode (TXD PE0 = MOSI, RXD PE1 = MISO, XCK PE2 =
SCK), which is how `test_avr_spi r` cross-checks the two peripherals
against each other; that mode has no client select, so board B's client
selects itself with INVEN on its own pulled-up PE3.

**Moving between the two is a live operation.** Neither firmware latches
what it found: the peer believes a topology only while the command
channel keeps proving it and goes back to alternating after three
seconds of silence, so re-jumpering under two running boards converges
by itself within a few seconds. Board B's console `0` forces it, and the
DUT names that escape in its own link-failure line.

To see the wiring for yourself rather than trust this page, run the
probe on both boards at once - board B's console command `2` and board
A's suite command `v`. Each drives its own PE0..PE3 at 2, 4, 8 and 16 Hz
for six seconds and listens for six more, and the EDGE COUNT on a pin
names which pin of the other board it is tied to:

```
  A.PE0: 23 edges -> B.PE0
  A.PE1: 45 edges -> B.PE1
  A.PE2: 90 edges -> B.PE2
  A.PE3: 179 edges -> B.PE3
```

Moving between the two costs three jumpers and nothing in the firmware.

CCL collisions on this board: LUT0 owns PA0..PA3 (PA0/PA1 are the
crystal), LUT1 PC0..PC3 (traffic LEDs = TCA0 PORTC WO0..3), LUT2
PD0..PD3 (display wires), LUT3 PF0..PF3; TCB0/TCB1 ALT1 sit on
PF4/PF5 = the console USART2.

Display: 3.5" red module **HST035003-A**, controller **ILI9481**
(320x480, SPI = 18-bit pixels only, panel needs INVON), XPT2046
resistive touch on board (U2; U1 is the LDO), BL tied high. The older
2.4" module (frozen white through every protocol despite verified
signals) is parked as defective-suspect.

MCP3550 facts measured with the analyzer (design consequences in
[design/spi-bus.md](design/spi-bus.md)): t_conv 81 ms with CS low,
~119 ms trigger-to-RDY when CS is high during the conversion (result
held for the next CS fall); needs a few us of CS setup before the
first SCK after waking (dac_adc uses 10; the datasheet only says "an
internal power-up delay must be observed"); latches its SPI mode from
SCK at CS fall (mode 1,1 = SCK high, DS20001950F 5.5); CS low >= 8 us
(tCSL).

## Apps

Two kinds of app share `src/apps/`: demos/steps (`traffic0`, `events0`,
...), disposable, and **bench test suites** named `test_<target>_
<subject>` (`test_avr_analog`), which are the reference tests of the
drivers they cover and must keep passing through every restructuring
(they move to a per-target directory when there are enough of them).

One `src/apps/<app>.cpp` = one `main()` = two envs (`<app>`,
`<app>-debug`) on the default board, plus one such pair per extra board
type it names (see "Multi-board bench" above), generated into
`apps.ini` by `python tools/gen_apps.py`
(VS Code task "PIO: regen apps", then reload the project). An app may
carry `// pio: <option> = <value>` lines in its header comment (e.g.
`// pio: monitor_speed = 115200`) - see "Per-app env options" in
[avrdx/README.md](avrdx/README.md).

Shared code goes into `lib/brio/src/`; any header an app includes is
compiled and linked by the LDF, no filter changes. What does NOT belong
there is bench tooling two apps happen to share - a wire protocol
between a DUT and its instrument is not framework. That kind of header
sits next to the apps in `src/apps/` and is included by its plain name
(`#include "usart_link.hpp"`), which the compiler resolves against the
including file's own directory: no build change, and the layering rule
stays honest because nothing under `lib/brio/src/` knows it exists.

| App | What it does |
|-----|--------------|
| `family_probe` | The smallest firmware meant to run on EVERY package: PA7 toggling at ~2 Hz on the internal oscillator. Carrier of the board matrix (`// pio: boards = db28,db32,db48`) and the first thing flashed onto a new board - PA7 alive means chip, UPDI link and fuses are sane |
| `xtal_probe` | The crystal diagnosis probe, for a board whose 24 MHz crystal will not start: the main clock stays on OSCHF while XOSCHF is started and observed (MCLKSTATUS.EXTS) across every FRQRANGE (the range code also sets the oscillator's drive) x every CSUTHF start-up time, reporting spins-to-stable per combo on the console; `r` repeats, so it stays on the board through a repair session. A "never" everywhere means the fault is on the board, not in the start-up margins (board B's dead joint was found and fixed this way); a healthy crystal starts in tens-to-hundreds of spins in every combination |
| `blink` | The minimal kernel app: Blinker toggles PF2 on its periodic time event, Supervisor cycles the period (500/250/100 ms) every 3 s by posting a command - no delay loops, CPU in IDLE sleep between events |
| `clock_console` | The console on a runtime-variable clock (`DynamicClock<Boot, Serial>` @ 115200): `CLOCK 4M` (Hz, or nM/nk) switches CLK_PER under the running program; the console surviving = the rebase fan-out works, the 1 Hz LED = the RTC timebase never noticed |
| `console` | Interactive command console @ 460800 (HELP, LED, UPTIME, ERR): SerialPort (RX bytes -> line events), Console (parse/route/reply), Blinker (heartbeat FSM + LED commands via posted events), zero polling |
| `spi_loopback` | SPI stack test: jumper PA4(MOSI) -> PA5(MISO), a full-duplex 8-byte transaction per second through SpiBus + the Spi<0> engine, verdict on the console (no jumper = FAIL 0xFF, by design) |
| `display_id` | Reads the display controller's DCS registers (RDDPM, RDDID, ID4, 0xBF device code) and prints the raw answers: the probe that identifies the display controller (the 3.5" module answers as an ILI9481) |
| `display_fill` | ILI9481 full-screen solid fill cycling red/green/blue (18-bit pixels, CASET/PASET + RAMWR/3C row writes, INVON) |
| `spi_duo` | Two devices, one arbitrated bus: ILI9481 fill (960-byte rows @ 6 MHz) + XPT2046 touch polling (3-byte conversions @ 1.5 MHz) through the same SpiBus; touch steers the fill palette, per-request clock switching |
| `spi_paint` | Touch painting on the ILI9481 through the arbitrated bus |
| `dac_adc` | Two buses, one signal: MCP47CVB22 VOUT0 (I2C) into the MCP3550 (SPI) - a 9-step ramp, each step written and read back over I2C (write-then-read, repeated START) and measured by the ADC via two SPI requests (trigger, then read 200 ms later, busy-frame retry). ADC code = DAC code * 512 within a few LSB |
| `mcp_diag` | MCP3550 behaviour probe, bit-banged on PB0/PA6/PA5 with `delay_us`, one experiment per console key, no kernel: t_conv, CS toggling, early clocks, MISO net, RDY trace |
| `i2c_scan` | I2C stack test: address sweep 0x08..0x77 every 2 s on TWI0 through I2cBus + the TwiHost<0> engine, ACKs printed as found (expected: 0x60) |
| `events0` | The event system on the bench (verified): PIT/8192 -> EVOUTF (LED at 4 Hz, no CPU), channel 1 rewired every 10 s by an AO (PIT/64 512 Hz, button PA2 level, off) -> EVOUTD PD2 for the analyzer, software pulses on PC2 |
| `test_avr_clock` | **Bench test suite** (14/14 on the scope): CLKCTRL - OSCHF rates, prescalers, tune, crystal vs OSCHF, 32 kHz main clock, forced clock failure + recovery, PLL/status; CLK_PER on PA7 (CLKOUT) for the scope; console 9600 (talks down to 153.6 kHz; silent only at the 32 kHz main clock) |
| `test_avr_analog` | **Bench test suite** (keep passing): VREF/DAC/ADC, 14 tests / 54 verdicts knob by knob - references cross-check, ramp, OUTEN, settling, resolution, differential, prescalers, accumulation, sampling knobs, event start, window, errata 2.3.2, internal inputs, VREFA from the DAC. Supply measured at start (3.3 V and 5 V). Wire PD6->PD1, PD6->PD7. 54/54 on rev A5 @ 3.3 V |
| `test_avr_timer` | **Bench test suite** (keep passing): TCA/TCB/CCL/AC, 11 tests / 82 verdicts, 82/82 on rev A5 at 3.3 V - FrequencyGenerator/TcaPwm16/Heartbeat on PD0 measured back by the TCB meters through EvPin, OneShotPulse on PB5, Pwm8 on PC0, PulseCounter, 32-bit cascade vs the Ticker, PeriodicTick, Timeout, EventCounter with direction from PC1; CCL LUT4 on PB0/PB1 -> PB3 and a JK flip-flop by timer events; AC thresholds/window against the DAC on PD6. Nothing to wire (PB0/1/3/5 = traffic LEDs flicker). Holds in crystal time (PIT paused): the Ticker's OSC32K measured +0.94 % fast |
| `test_avr_pin` | **Bench test suite** (keep passing): PORT - the pin senses counted on self-driven edges, level_low, INVEN, input_disable, the W1C flag discipline, the multi-pin engine across two ports, the Port mask verbs and the slew bit. 22/22 on rev A5. Nothing to wire |
| `test_avr_platform` | **Bench test suite** (keep passing): the PLATFORM - `AvrPlatform`, `delay.hpp` and `reset.hpp`, 9 tests / 96 verdicts, 96/96 on rev A5. Nothing to wire. Every delay path counted in CLK_PER cycles against a 32-bit TCB pair (the folded path exact to the cycle, the fixed-point runtime path a constant 122 cycles, a DynamicClock dispatched by rate index: 157 with a runtime us - capped at 200 so the old runtime division cannot return - and 6 with a constant one, sub-MHz rates exact), `delay_cycles` and its 16-bit chunking, the critical section's nesting and its three-cycle cost, `idle()` (wake latency, the six-cycle wake-up penalty, and a counter frozen while the CPU sleeps), the lock-free `Ring` under a 20 kHz ISR producer plus the `EventQueue`'s saturating overflow counter, `now()` across low-byte wraps against the crystal - and test `i`, which RESETS THE BOARD FOUR TIMES on purpose (watchdog time-out, a WDT window violation, a software reset from inside a panic reporter, a plain software reset) and carries its own verdicts across them in a `.noinit` token, so `z` still closes with one ALL: line. Takes about three seconds |
| `test_avr_rtc` | **Bench test suite** (keep passing): RTC/PIT, 8 tests / 78 verdicts, 78/78 on rev A5 - the counter's period at three prescalers, the compare phase (CMP + 1 ticks), crystal error correction at +-127 ppm, the synchronization busy flags, the first tick after an enable, the PIT and the counter running together, OSC1K as CLK_RTC. A TCB cascade at CLK_PER is the stopwatch and the RTC's own OVF/CMP events latch it: nothing to wire. Takes about two minutes (the correction is measured by averaging) |
| `test_avr_serial` | **Bench test suite** (keep passing), in two halves. `z` = SINGLE BOARD, 9 tests / 108 verdicts, 108/108 on rev A5 at 5 V: instances and routes including the pinless NONE and the teardown, the whole 36-combination frame-format matrix in loop-back on USART4, the receive FIFO's overflow, the MPCM filter, DRE vs TXC over the three-deep transmit path, the baud generator measured on PE0 through EvPin + a TCB pulse-width meter, a 24 -> 12 -> 24 MHz rebase under traffic, GENAUTO/LINAUTO auto-baud with the errata 2.16.3 recovery, and Host SPI in loop-back. `y` = TWO BOARDS (needs the PORTE link, NOT fitted today), 12 tests / 110 verdicts, 110/110 on the crossed pair and 103/103 on the straight-through wiring, where `q` (the synchronous roles) skips itself: the baud and frame matrices across the wire, injected parity/rate/overflow/break errors, bit-banged waveforms (glitch widths, a uniform rate error, one distorted cell, the four ABW windows), auto-baud against board B's own oscillator, MPCM in both flavours, both synchronous roles on a real XCK with the client's CLK_PER/4 ceiling, RS-485's XDIR guard time, IRCOM pulses and the RXPL filter, a clock rebase under real traffic, and the LBME pad probe. Also three tools outside `y`: `v` = the wiring probe (with board B's console `2`), `w` = the one-wire bus (6/6 when the shared wire is fitted; the first two depend on how the desk is jumpered - `w` skips itself on the crossed pair and `q`, the synchronous roles, skips itself on the shared wire), and `x` = the clock comparison: the peer's 0x00 frames put nine hardware-timed bit times of low on the wire, the DUT's pulse-width meter averages sixteen of them (~60000 ticks each, one-tick spread, the div1 short-read corrected), resolving the two crystals' ratio to a few ppm - the boards measure +161..169 ppm apart. On a SHARED line `z` asks board B to stay quiet, re-armed per test - there the peer sits on the DUT's own loop-back; on the crossed pair it cannot disturb a loop-back measurement and nothing is armed |
| `test_avr_spi` | **Bench test suite** (keep passing), in two halves; nothing to wire for `z`, the desk's PORTE link for `y`. `z` = SINGLE BOARD, 10 tests / 148 verdicts, 148/148 on rev A5 at 5 V: routes and teardown including the pinless NONE and the two refusals (SPI1 ALT2 by errata 2.11.1, a pinless host watching SS by DA errata 2.10.1), all seven bit rates measured on SCK through the SPI's own SCK event into a TCB frequency meter (exact, CLK_PER/2 included), the data path with MISO driven by the board's own PORT and MOSI counted through EvPin, the four transfer modes' idle levels, WRCOL and the two clear disciplines, buffer mode's four flags (DREIF's two levels, TXCIF, BUFOVF's deferred rise), host demotion forced through the SS pin's own INVEN, both ISR bodies, a 24 -> 12 -> 24 MHz rebase under a 1.5 MHz SCK ceiling, and the SpiHost engine with both completion styles. `y` = TWO BOARDS (needs the PORTE link, NOT fitted today), 9 tests / 92 verdicts, 92/92, with board B running `spi_peer` as a real client driven in band over the bus itself: the bring-up and the nak, the client matrix (4 transfer modes x 2 bit orders x 3 buffering regimes, exact both ways), the rates inside the client's CLK_PER/6 ceiling and the corruption above it, CPOL/CPHA mismatches and the bit order as an exact two-way reversal, the select wire raised mid-byte, a client that never drains (the normal-mode survivor, buffer mode's BUFOVF condition, the client's WRCOL, the echo a missed load produces), a REAL host demotion driven by board B, USART4's Host SPI mode against this peripheral, and a rebase under two-board traffic. `z` passes with the peer attached: `spi_peer` is dark until addressed. Everything runs on SPI0 ALT1 (PE0-PE3) - SPI0's DEFAULT route is the 3.3 V display/MCP3550 cabling and must never be driven at 5 V, and SPI1's positions are the traffic LEDs, so SPI1 is exercised on route NONE only |
| `test_avr_twi` | **Bench test suite** (keep passing), in two halves, NOTHING TO WIRE beyond the desk's I2C bus - the two-board half commands board B in band over that same bus. `z` = SINGLE BOARD, 10 tests / 175 verdicts, 175/175 on rev A5 at 5 V: the route table and its refusals with the errata's PORT.OUT hygiene observed on purpose, the three bus speeds measured on SCL against equations 29-2..29-5 through a pin event into two TCB capture meters (period and low width, so the bus's own rise and fall times come out as numbers), the COMBINED loop (a host and a client of the same instance on PA2/PA3) and the DUAL loop (the client moved to PC2/PC3 through DUALCTRL) with every Request shape both ways, the whole address-match space proven by who ACKs, the chapter's cases M1..M3 and S1..S3 with both NACK verdicts, both Smart modes counted in MCMD strobes, Quick Command counted in SCL edges, the bus state machine driven by a bit-bang injector on the dual pair (foreign START/STOP, the three inactive-bus time-outs, BUSERR, a host held on a Busy bus), both ISR bodies with exact interrupt counts, and a 24 -> 12 -> 24 MHz rebase under traffic with one arbitrated request through I2cBus. `y` = TWO BOARDS, 9 tests / 174 verdicts, 174/174, with board B running `twi_peer` as a real second device driven in band over the bus (a write tenure carries a command frame to the peer's address, a read tenure collects the answer - `src/apps/twi_link.hpp`): the bring-up and the nak, clock stretching at 100 us and 1 ms a byte measured on the wire and in a 32-bit TCB stopwatch, an injected address NACK and a data NACK at a chosen byte, multi-host arbitration in BOTH directions (both boards combined, both STARTs released by the same edge, the winner chosen by the smaller address), the collision case S4 with the two boards' clients on one address, `unstick()` against a really stuck SDA and against a healthy bus, a General Call answered by two chips plus a quick command a real chip acknowledges, the three bus speeds against the peer's client, and a rebase under two-board traffic. `z` passes with the peer attached: its command-mode client answers ONE address exactly, with no general call, no mask, no PMEN and no PIEN |
| `spi_peer` | The INSTRUMENT half of the SPI campaign, for board B: one blocking loop that shifts whatever the DUT clocks, decodes a command frame off SPI0 ALT1, acknowledges it and becomes for a bounded moment whatever the DUT needs at the other end - a client in any transfer mode, bit order and buffering regime; a client that never drains, or one that misses a load on purpose; a second driver on the shared select wire (the only way to demote a real host); a self-selecting client for USART Host SPI. It is a DARK LISTENER: it drives MISO for exactly one answer window, entered only after a frame that checked out, so it can stay on the desk while the DUT runs its single-board half. Console (observability only): `?` help, `i` status and counters, `0` back to the dark client, `3` command trace |
| `twi_peer` | The INSTRUMENT half of the TWI campaign, for board B: one blocking loop around the polled TwiClient surface that answers one address, decodes a command frame off the bus, acknowledges it and becomes for a bounded moment whatever the DUT needs - a client that clock-stretches by a commanded time per byte, one that NACKs the n-th byte or is not there at all, one that answers the General Call or stops answering it, a second HOST racing the DUT for the wire, a second client sharing one address (so a read is served by two devices and one of them collides), or a stuck client holding SDA low from PORT until enough SCL edges have gone by. Every action carries a count and a deadline after which the peer restores its command-mode client by itself. Console (observability only): `?` help, `i` status and counters, `0` back to command mode, `3` command trace |
| `usart_peer` | The INSTRUMENT half of the USART campaign, for board B: one blocking loop that decodes a command frame off the link, acknowledges it and becomes for a bounded moment whatever the DUT needs on the other end - echo, silent sink, generator, full-rate flood, break, foreign auto-baud sender, cycle-counted bit-banger (a stretched bit cell, sub-bit glitches), one-wire responder. Every mode-changing command carries a frame count and a deadline after which the peer restores command mode by itself. Console (observability only): `?` help, `i` status and counters, `0` command mode, `1` one-wire standby, `2` wiring probe, `3` command trace |
| `sampler` | The ADC inside the kernel: `AnalogSampler` over ADC0 walking PD1 (the DAC loop), die temperature and VDD/10, published to a Monitor (one line a second) and an Alarm (LED PF2 above a threshold); console DAC/PACE HW|SW|OFF/ALARM/CLOCK/STAT. Bench: 512 samples/s (PIT/64) with no queue drops, 128/s default, CLK_ADC follows CLOCK 4M/24M under sampling |
| `traffic0` | The over-commented AO learning testbed: 4 buttons -> 4 RGB lamps, one AO per role, publish for button facts |
| `traffic1` | The traffic light FSM: timed phases via one re-armed time event, a remembered pedestrian call |
| `traffic2` | traffic1 with PWM lamps (TcaPwm split mode, colour palette): the actuator changes, the AOs do not |
