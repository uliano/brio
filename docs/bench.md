# The bench

The board, the wiring and the apps as they are TODAY. This page is the
volatile end of the documentation: apps are disposable tools that test
the framework's ideas and will not survive in their current form;
nothing in `docs/design/` or in the target pages depends on them.
Every app documents itself in its own header comment - this table is
only the map.

## Boards

Two boards of the same model belong to the desk, indistinguishable by
hardware (same chip, serial-less CH340): each carries its name in its
USERROW ([avrdx/userrow.md](avrdx/userrow.md)) - **A = `brio-a`**, the
DUT; **B = `brio-b`**, the instrument peer (and, since the flash
suites, a DUT in its own right). Both are plugged in today. The suites
print the label in their banner, so a console names its own board.

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
  `J42700051207`, B's `J42700049508` (re-verified by reading each
  board's USERROW label through its own probe). The probe-to-board pairing is the
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
$PY tools/bench.py fuses A              # read the fuses; name=value pairs write them
```

`run` exits nonzero on a timeout or a nonzero fail count, so a suite is
usable from a script. The grammar it parses - the letter menu, the
`PASS`/`FAIL` lines, the per-letter `-> N pass, M fail` and the closing
`ALL:` total - has ONE implementation, `util/testbench.hpp`: a suite
registers its letters with it and prints only its own measurements.

**The three erase regimes, MEASURED on this desk** (AVR128DB48 over
UPDI, avrdude 8.1), because the option names invite exactly the wrong
assumption. The flash of these parts is rated **1k erase/write cycles**
(DS40002247B 39-7, lowered from 10k "based on validation data"), so
which pages an erase touches is a budget question:

| How `flash` is invoked | What avrdude does | What survives |
|------------------------|-------------------|---------------|
| default | PAGE-ERASES each page it is about to write | every other page of the part, and the EEPROM regardless of EESAVE |
| `--erase` | a real chip erase (`-e`) first | nothing: every page, EEPROM included |
| `-D` (not used here) | NO erase at all: programs into pages as they are | everything - but the image is ANDed into whatever was there |

The default is what a reflash costs ~40 page cycles instead of 256, and
it is also what lets an `NvHeap`'s blocks and map pages survive a
reflash ([design/nv-heap.md](design/nv-heap.md)): the tool writes the
image's pages and leaves the rest of the part alone. `-D` is a trap and
the reason it is not used: it disables the erase entirely, so it is
safe only when the bytes already in the chip are the ones being written
(reflashing an unchanged image) and silently corrupts anything else -
observed as an avrdude verification mismatch, and the reason
`__nvheap_build_id` is derived from the sources rather than from the
clock.

**The NvHeap preflight.** Before writing, `flash` reads the chip, looks
for a valid heap map in the last pages of the part, and says plainly
which stored blocks the new image would land on (or, with `--erase`,
that all of them are about to go). It WARNS AND NEVER BLOCKS, and it
stays silent when the chip holds no map.

Bench regression policy under
the same budget: native + `tools/check_family.sh` at every change
(free); after a change to a driver, its own suite; after a
cross-cutting change, ONE canary suite whose mechanism is nearest the
change; every other baseline re-runs opportunistically, when its
wiring is next on the desk anyway. `duo` (console-scripting the peer) is **still
unexercised**: both protocol campaigns command their peer IN-BAND over
the link under test (`src/apps/usart_link.hpp`,
`src/apps/spi_link.hpp`), so the whole matrix runs as a plain `run A`.
`duo` stays for a campaign whose bus cannot carry its own commands.

### Fuses

Fuses are provisioning, not build output: the CPU can read them and
nothing more (DS40002247B 11.3.1.5), they survive every reflash, and
only the programmer writes them. `bench.py fuses <board>` reads the
seven named fuses and spells out the Flash geometry and the EESAVE
state they produce; `bench.py fuses <board> name=value ...` writes
them, reading each one back and reporting the before/after pair.

```bash
$PY tools/bench.py fuses A                        # read, with the geometry spelt out
$PY tools/bench.py fuses A bootsize=128 codesize=0
```

**The standing geometry of board A is `BOOTSIZE = 128`, `CODESIZE = 0`,
`SYSCFG0 = 0xC8`**: BOOT is the first 64 KB, where all the code lives,
and APPCODE is everything above it, which is what makes the Flash
writable from software at all (with the shipping default `BOOTSIZE = 0`
the whole Flash is BOOT and nothing can write any of it). Every image
sets `CPUINT.CTRLA.IVSEL` in `.init3` and is therefore correct under
both geometries - see [avrdx/README.md](avrdx/README.md) and
[avrdx/nvm.md](avrdx/nvm.md). EESAVE is CLEAR, so a chip erase wipes
the EEPROM; `test_avr_nvm` verified both settings and put it back.

**Board B carries the same geometry** (`bootsize=128 codesize=0`),
written for `test_avr_nvheap`: without it the whole flash is BOOT, SPM
writes nothing and the heap's middle zone is empty. Putting the flash
suites on B is also deliberate wear rebalancing - A has spent the
project's page cycles so far.

### End state

**Today's end state: board A runs `test_avr_serial` (under the
standing fuse geometry), board B runs `test_avr_opamp` and still holds
the five live `NvHeap` blocks in flash, both on the desk.** The OPAMP
campaign reflashed B five times over `test_avr_nvheap`'s blocks and
`test_avr_nvheap v` found all five present with EXACT contents
afterwards - the page-selective default erase doing what the table
above says it does. The desk is re-rigged daily, so the console/probe mapping is
re-verified at session start the only way that can be trusted: the
probe by USERROW readback (the id names the board), the console by
resetting the chip over UPDI and watching which port emits the boot
banner. Today's verified mapping (the manifest matches): A = console
`usb-0:1.1` / probe J42700051207, B = console `usb-0:1.4` / probe
J42700049508 (the two ICEs had swapped boards again, caught by the
USERROW readback). Wires fitted: ONE jumper `A.PE0 -
B.PE0` (the shared TXD line) plus the dedicated GND. Every two-board
half is therefore idle: B carries its own suite and not a peer, so
`test_avr_serial x`/`w` need `usart_peer` back on B, and the halves
that also want the full PORTE link (`A.PEn - B.PEn`, n = 0..3) need
that fitted AND their peer firmware (`spi_peer`, `twi_peer`,
`sleep_peer`). Reflashing B is what costs the heap its blocks: the
default page-selective erase keeps them, a `--erase` does not.

Board B's 24 MHz crystal is ALIVE: the crystal was replaced and the
10 pF load capacitors refitted with parts verified by measurement
(the earlier "dead crystal" verdicts were 100 nF parts from a
contaminated 10 pF drawer RF-grounding the pins - a fault invisible
to every DC check). B's banner declares `clk=XTAL 24 MHz`.
`test_avr_serial x` measures board B **+152..+153 ppm fast against
board A** (three runs, 1..2-tick spread). The offset is a property of
the crystal SPECIES, not of an individual part: B's two crystals
(same purchase) read +165 and +152 against A, while load-capacitor
asymmetry between the identical boards is bounded at ~15-20 ppm/pF
of pull and cannot produce it - consistent with A's crystal being a
different fit with a different rated C_L. An absolute per-board
measurement against the host's NTP-disciplined clock is the designed
follow-up if the A-vs-B split ever matters.

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
| Board-to-board PORTE link (test_avr_serial + usart_peer, the SPI campaign, the sleep pass wake tests) | A.PE0-B.PE0, A.PE1-B.PE1, A.PE2-B.PE2, A.PE3-B.PE3 (STRAIGHT THROUGH) | see "the board-to-board link" below; verified by the wiring probe |

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

**The desk carries the STRAIGHT-THROUGH link today**, `A.PEn - B.PEn`
for n = 0..3, verified by the wiring probe in both directions. For the
USART campaign that is the shared wiring - A.PE0 and B.PE0 are the two
TXD pads tied together - so the suite runs `w` (6/6) while `q`, the
synchronous roles, skips itself, and `y` scores 103 verdicts instead
of the crossed pair's 110. PE3 is wired too, which the RS-485 test
does not want (it wants XDIR DUT-local) but which does not disturb it,
since XDIR only drives into the other board's input.

For the SLEEP campaign the four wires take a third set of roles, fixed
and discovered by nothing: PE0 is the shared one-wire command channel
(the USART wiring above, LBME at both ends), PE2 is board B's STIMULUS
into the DUT - and it is a Px2 pin, one of the two fully asynchronous
positions of every port, which is what lets an edge on it wake a chip
with every clock stopped - and PE3 is the DUT's ECHO back into B, where
a pin event captures B's 32-bit stopwatch in hardware. PE1 is spare.

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
| `bus_mv` | The bench voltmeter, for a desk without a multimeter: board A measures its own VDD and VDDIO2 through the internal dividers against the 2.048 V reference (no wires), and two jumpers - PD1 (AIN1) and PD2 (AIN2) onto any two nodes - read those nodes in mV against VDD twice a second, alongside the digital state of the I2C taps (PA2/PA3/PC2/PC3). Poke the wires and watch: it found the pull-ups on a LED pin and two jumpers on the wrong pins in one session |
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
| `test_avr_twi` | **Bench test suite** (keep passing), in two halves, NOTHING TO WIRE beyond the desk's I2C bus - the two-board half commands board B in band over that same bus. `z` = SINGLE BOARD, 10 tests / 176 verdicts, 176/176 on rev A5 at 5 V: the route table and its refusals with the errata's PORT.OUT hygiene observed on purpose, the three bus speeds measured on SCL against equations 29-2..29-5 through a pin event into two TCB capture meters (period and low width, so the bus's own rise and fall times come out as numbers), the COMBINED loop (a host and a client of the same instance on PA2/PA3) and the DUAL loop (the client moved to PC2/PC3 through DUALCTRL) with every Request shape both ways, the whole address-match space proven by who ACKs, the chapter's cases M1..M3 and S1..S3 with both NACK verdicts, both Smart modes counted in MCMD strobes, Quick Command counted in SCL edges, the bus state machine driven by a bit-bang injector on the dual pair (foreign START/STOP, the three inactive-bus time-outs, BUSERR, a host held on a Busy bus), both ISR bodies with exact interrupt counts, and a 24 -> 12 -> 24 MHz rebase under traffic with one arbitrated request through I2cBus. `y` = TWO BOARDS, 9 tests / 174 verdicts, 174/174, with board B running `twi_peer` as a real second device driven in band over the bus (a write tenure carries a command frame to the peer's address, a read tenure collects the answer - `src/apps/twi_link.hpp`): the bring-up and the nak, clock stretching at 100 us and 1 ms a byte measured on the wire and in a 32-bit TCB stopwatch, an injected address NACK and a data NACK at a chosen byte, multi-host arbitration in BOTH directions (both boards combined, both STARTs released by the same edge, the winner chosen by the smaller address), the collision case S4 with the two boards' clients on one address, `unstick()` against a really stuck SDA and against a healthy bus, a General Call answered by two chips plus a quick command a real chip acknowledges, the three bus speeds against the peer's client, and a rebase under two-board traffic. `z` passes with the peer attached: its command-mode client answers ONE address exactly, with no general call, no mask, no PMEN and no PIEN |
| `test_avr_tcd` | **Bench test suite** (keep passing): TCD, 11 tests / 250 verdicts, 250/250 on rev A5 at 5 V, NOTHING TO WIRE. WOA..WOD sit on the DEFAULT route PA4..PA7 and are read back as pin EVENTS into TCB meters; the TCD's own CMPBCLR event is the cycle counter; PD3/PD4 driven from PORT are the two input-event sources. Clocked from CLK_PER with both prescalers at DIV1, one counter tick IS one CLK_PER tick, so the chapter's four cycle formulas and both on-times come out in whole ticks (all exact, zero spread - and dual slope measures 2 x (CMPBCLR + 1), one more than the chapter prints). Also: the three synchronization disciplines observed (ENRDY, CMDRDY, the static-register refusal), the per-route pin claim and teardown, the dead-times isolated on a pin through CMPOVR + CTRLD, `TcdPwm`'s complementary pair exact at every duty, the prescaler product, a 24 -> 12 -> 24 MHz rebase with a TCD on OSCHF (immune) and on CLK_PER (following), the PLL's multipliers measured as 2.000/2.999/2.001 against their own oscillator with PLLS locking only when the TCD requests it, the software and event captures with the chapter's PWM-capture example, six input modes with the async/filter/blanking qualifiers, dithering to the tick, the output plumbing (CTRLD, WOC/WOD selection, the fault levels, DISEOC, AUPDATE) and both interrupt vectors. Errata: 2.14.2 MEASURED on ALT2 (a WOB-only configuration drives nothing until CMPAEN is set), CLKCTRL 2.5.3 and 2.5.4 measured, 2.14.1 and 2.14.3 NOT REPRODUCED on this die (both recorded as such in [avrdx/tcd.md](avrdx/tcd.md)). PF0..PF3 (the ALT2 route) are claimed only inside test `a`; PF4/PF5 (the console) are never touched |
| `test_avr_sleep` | **Bench test suite** (keep passing), in two halves. `z` = SINGLE BOARD, 8 tests / 72 verdicts, 72/72 on rev A5 at 5 V, NOTHING TO WIRE - but PD1/PD2 must be FREE of the bus jumpers, because test `e` drives PD2 from the event system and senses its own edge on the pad. The register surface with the erratum 2.2.4 NOP and the HTLLEN interlock refused both ways (the CCL, and a TWI client raised on TWI1 - PF2/PF3 go nowhere on this board); IDLE through `Sleep::enter`; STANDBY proven real by a 32-bit TCB stopwatch frozen at 178 CLK_PER ticks where awake it counts 2.97 million; the RUNSTDBY matrix (two sources x the oscillator's flag x the peripheral's - only the PERIPHERAL's decides) and a TCB waking the CPU by itself; a PORT pin waking from standby AND from power-down with the pad driven by the device's own EVSYS through a PIT divider; power-down stopping the RTC counter while its PIT still interrupts (and the CNT read at the instant of a wake coming back STALE); the wake-up delay in CLK_RTC ticks; and test `n`, the CCL as a wake-up source - a clocked, filtered LUT wakes from standby, and from power-down whenever its CLOCK is one that mode keeps (OSC32K yes, CLK_PER no). `y` = TWO BOARDS, 6 tests / 49 verdicts, 49/49, with board B running `sleep_peer` over the PORTE link: the awake baseline is 23 ticks of B's 41.7 ns stopwatch, IDLE adds 7 (the data sheet's six cycles), and standby and power-down are swept against every clock configuration - a kept-alive crystal costs 43 ticks, a restarting one 1.77 ms, OSCHF 23.6 us beside a running crystal but 313 us once every oscillator is stopped, of which `PMODE = FULL` removes 290 (the REGULATOR, paid for only when the device really lets go). Also USART start-of-frame out of standby (the detector needs the line still LOW when the clock returns, so the byte's own bit pattern decides) and a TWI address match out of both deep modes, timed by the peer as a stretch on the wire. Every power-down sleep is bracketed by a watchdog and a `.noinit` token, so a mode that failed to wake would say so at the next boot instead of hanging the bench. Owns the RTC block (no Ticker: the PIT period IS the instrument) and event channels 0, 1, 4 and 5 |
| `test_avr_nvm` | **Bench test suite** (keep passing): NVMCTRL and the services over it, 6 tests / 112 verdicts, 112/112 on rev A5 at 5 V, NOTHING TO WIRE - but it NEEDS THE FUSE GEOMETRY above (`bootsize=128 codesize=0`), and says so and skips the Flash legs without it. The Signature Row and the whole geometry incl. the scratch region the driver computes from the linker symbols (65536..98304, 64 pages - gcc puts .rodata in Flash section 3, so the free Flash is a hole in the MIDDLE of the part), FLMAP moved through all four sections and then locked one-way; the EEPROM's two write commands, its byte and multi-byte erases and their times against table 39-7 (65 us a bare write, 10087 us an erase-and-write, and erasing 32 bytes costs exactly what erasing one costs); the typed record with its only-changed-bytes policy (an unchanged store writes ZERO bytes) and the EEREADY-paced writer AO (one interrupt per byte, 14064 main-loop turns and 84 timebase ticks during an 80 ms transfer); Flash erase, word write, ELPM read-back and all five multi-page spans in the scratch region; what an operation costs the system (a page erase halts the CPU for its whole 10 ms, delays an interrupt by 9078 us, and costs the 1024 Hz software timebase nine of its ten ticks); and test `f`, which RESETS THE BOARD THREE TIMES on purpose to prove APPCODEWP, BOOTRP and a panic record stored in the EEPROM, carrying its verdicts across them in a `.noinit` token so `z` still closes with one ALL: line. Outside `z` because each costs more than a test should cost per run: `u` the User Row write path (9/9, one erase cycle of the row - it saves the label, wipes the row and puts it back) and `g` (11/11) which needs a temporary `codesize=223` and OBSERVES errata DS80000915F 2.7.1 - a two-page erase straddling a protected APPDATA boundary raises no error and erases the protected page, while the single-page erase of the same page is refused. The declared wear budget is in [avrdx/nvm.md](avrdx/nvm.md) |
| `test_avr_nvheap` | **Bench test suite** (keep passing): the flash BLOCK ALLOCATOR - `util/nv_heap.hpp` over `avrdx/nvm_flash.hpp` - 4 tests / 51 verdicts, 51/51 on board B, NOTHING TO WIRE, but it NEEDS THE FUSE GEOMETRY (`bootsize=128 codesize=0`). Runs on BOARD B by design (wear rebalancing) and its blocks are MEANT to still be there afterwards. `a` mounts, prints the geometry the linker left (middle zone 0x10000..0x18000 = 64 pages, tail 0x19000..0x20000 = 56 pages of which 54 free, map home 0x1fc00..0x20000, one map version 130 bytes of 512) and round-trips a one-page block and a two-page block with an odd tail; `b` supersedes a block by id and proves the old one is served until the very seal, then watches the two map pages take turns; `c` rewrites a block in place (same address, new length and contents); `d` RESETS THE BOARD and finds the block, the map sequence and the build id intact, carrying its verdicts across the reset in a `.noinit` token so `z` still closes with one ALL: line. Outside `z`: `v`, the reflash choreography's judge - it prints per-id survived/lost and passes on either coherent state, `tables present` (5 of 5 EXACT after a default reflash, twice: page-selective erase and a `-D` rewrite of the identical image) or `clean slate` (all absent, heap empty and mountable, after `--erase`). The wear is about a dozen page erases per `z` run |
| `test_avr_opamp` | **Bench test suite** (keep passing): OPAMP, the DB-only analog signal conditioning block - 9 tests / 96 verdicts, 96/96 on board B at 5 V, NOTHING TO WIRE, because the whole instrument is inside the chip: the DAC's buffered output on PD6 is the source, the ADC reads every op amp OUT pad (PD2, PD5, PE2), a TCB latches the READY event through EVSYS and PD0 supplies the LEVEL the DUMP and DRIVE event users need. Both converters run on VDD, so a DAC code c aims at ADC count 4c whatever the rail is. The register faces and the errata (IRSEL measured WRITABLE on this A5 die: 2.8.2 is rev. A4 only); the voltage follower over a nine-point sweep (0 mV of error at every point); the whole non-inverting ladder, exact to a permille at all eight wipers, and the whole inverting ladder about VDD/2 with its input on MUXBOT = DAC; VDD/2, ground, both op-to-op links (LINKOUT and OP0's LINKWIP) and a two-stage cascade; the chapter's three-op-amp instrumentation amplifier at all seven gains of table 35-14; the internal timer measured ENABLE-event to READY-event in hardware (one SETTLE unit = one TIMEBASE microsecond to the tick, 15 us of warm-up on a cold enable and 21 ticks on a restart, READY issued in EVENT_ENABLED mode ONLY) with a 24 -> 12 -> 24 MHz rebase proving TIMEBASE follows; the offset trim measured, stepped and improved from -490 uV to ~100 uV; and all four event users, including a RUNNING op amp holding its OUT pad against a pull-up with OUTMODE OFF and the DUMP switch turning the floating INN pad into a visible integrator. Claims all of PORTD plus PE1/PE2/PE3; PE0, the wire to the other board, is never touched |
| `test_avr_power` | **Bench test suite** (keep passing): the POWER MANAGER - `util/power.hpp`, `AvrSleepSite`, `TimeEvents::ticks_to_next()` and the one branch of `AvrPlatform::idle()` that makes them work - 5 tests / 44 verdicts, 44/44 on board B at 5 V, NOTHING TO WIRE and no pin claimed but the console. THE ONLY SUITE HERE THAT RUNS THE KERNEL: the object under test is an active object, so the rounds go through real queues, real dispatch and a real `Kernel<P, Probe, Bus, Pm>` pack - only the loop is the suite's, and where a sleep is the point it calls the kernel's own `idle_if_empty()`. A TCB pair cascaded at CLK_PER with `RUNSTDBY` on both halves is the stopwatch (it counts through the standby it is timing), the PIT is both the 1024 Hz timebase and the wake source, and a fake bus engine gives the `BusMaster` something to be busy with. `a` the ladder against `SLPCTRL.CTRLA` at every rung; `b` a real standby round - the loop frozen at exactly 32 turns over 32 ticks where awake it turns ~13500, the mode staying armed across a wake that says nothing, the 10-12 cycle wake, the 157 us round, and the first event afterwards disarming and publishing its `WakeReport`; `c` the deadline guard refusing `deep` one tick from a deadline and accepting it 1000 away; `d` the voters, with a transfer in flight aborting the round and its completion making the same request succeed; `e` the standing restrictions, nested, moved and released. Re-runnable indefinitely: it writes no nonvolatile memory and leaves nothing armed |
| `spi_peer` | The INSTRUMENT half of the SPI campaign, for board B: one blocking loop that shifts whatever the DUT clocks, decodes a command frame off SPI0 ALT1, acknowledges it and becomes for a bounded moment whatever the DUT needs at the other end - a client in any transfer mode, bit order and buffering regime; a client that never drains, or one that misses a load on purpose; a second driver on the shared select wire (the only way to demote a real host); a self-selecting client for USART Host SPI. It is a DARK LISTENER: it drives MISO for exactly one answer window, entered only after a frame that checked out, so it can stay on the desk while the DUT runs its single-board half. Console (observability only): `?` help, `i` status and counters, `0` back to the dark client, `3` command trace |
| `sleep_peer` | The INSTRUMENT half of the SLEEP campaign, for board B, and the only ruler that can time a wake-up: a sleeping chip cannot, because the mode stops the very counter that would. One blocking loop decodes a command frame off the shared PE0 wire (`src/apps/sleep_link.hpp`) and becomes for a bounded moment whatever the DUT needs - a train of stimulus edges on PE2, each zeroing a 32-bit CLK_PER stopwatch that the DUT's echo on PE3 CAPTURES through the event system (no software in the measurement path); one byte at a commanded baud on the same wire, for the DUT's start-of-frame wake; or one host write tenure on the office I2C bus against the DUT's TWI client, timed end to end. Its 24 MHz crystal is dead, so it counts on OSCHF - a per-cent-class reference, ample for microsecond-to-millisecond figures, and its banner and its `ident` answer both say so. Console (observability only): `?` help, `i` status, counters and the stored shot times, `0` back to command mode, `3` command trace |
| `twi_peer` | The INSTRUMENT half of the TWI campaign, for board B: one blocking loop around the polled TwiClient surface that answers one address, decodes a command frame off the bus, acknowledges it and becomes for a bounded moment whatever the DUT needs - a client that clock-stretches by a commanded time per byte, one that NACKs the n-th byte or is not there at all, one that answers the General Call or stops answering it, a second HOST racing the DUT for the wire, a second client sharing one address (so a read is served by two devices and one of them collides), or a stuck client holding SDA low from PORT until enough SCL edges have gone by. Every action carries a count and a deadline after which the peer restores its command-mode client by itself. Console (observability only): `?` help, `i` status and counters, `0` back to command mode, `3` command trace |
| `usart_peer` | The INSTRUMENT half of the USART campaign, for board B: one blocking loop that decodes a command frame off the link, acknowledges it and becomes for a bounded moment whatever the DUT needs on the other end - echo, silent sink, generator, full-rate flood, break, foreign auto-baud sender, cycle-counted bit-banger (a stretched bit cell, sub-bit glitches), one-wire responder. Every mode-changing command carries a frame count and a deadline after which the peer restores command mode by itself. Console (observability only): `?` help, `i` status and counters, `0` command mode, `1` one-wire standby, `2` wiring probe, `3` command trace |
| `sampler` | The ADC inside the kernel: `AnalogSampler` over ADC0 walking PD1 (the DAC loop), die temperature and VDD/10, published to a Monitor (one line a second) and an Alarm (LED PF2 above a threshold); console DAC/PACE HW|SW|OFF/ALARM/CLOCK/STAT. Bench: 512 samples/s (PIT/64) with no queue drops, 128/s default, CLK_ADC follows CLOCK 4M/24M under sampling |
| `traffic0` | The over-commented AO learning testbed: 4 buttons -> 4 RGB lamps, one AO per role, publish for button facts |
| `traffic1` | The traffic light FSM: timed phases via one re-armed time event, a remembered pedestrian call |
| `traffic2` | traffic1 with PWM lamps (TcaPwm split mode, colour palette): the actuator changes, the AOs do not |
