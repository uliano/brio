# The bench

The board, the wiring and the apps as they are TODAY. This page is the
volatile end of the documentation: apps are disposable tools that test
the framework's ideas and will not survive in their current form;
nothing in `docs/design/` or in the target pages depends on them.
Every app documents itself in its own header comment - this table is
only the map.

## Boards

Three boards belong to the desk: two AVR boards of the same model,
indistinguishable by hardware (same chip, serial-less CH340), each
carrying its name in its USERROW
([avrdx/userrow.md](avrdx/userrow.md)) - **A = `brio-a`**, the DUT;
**B = `brio-b`**, the instrument peer (and, since the flash suites, a
DUT in its own right) - and the **SAM C21 board** (its own section
below). The suites print the label in their banner, so a console
names its own board; which consoles are actually plugged in is a
per-session fact (the CH340 by-path names move with the USB socket).

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
- Programmer/debugger: Atmel-ICE over UPDI, AVR port, 6-pin ISP
  header (pin 2 VCC, 5 UPDI, 6 GND); A's probe is `J42700051207`.
  The second ICE, `J42700049508`, has MOVED to the SAM board's SWD -
  flashing board B needs it back on B's UPDI first (and
  `tools/bench_boards.py` updated if the move becomes standing). The
  probe-to-board pairing is the cable and the console-to-board
  pairing the USB socket, so both are facts about the desk, not
  about the chips: `tools/bench_boards.py` is the truth and the
  USERROW label in each banner is the cross-check.

## The SAM C21 board

The user's own C21J rev 1.1 (custom KiCad board, not the Xplained
Pro): ATSAMC21J18A, silicon rev F (DSU DID 0x11010500, read over
SWD). No wires - everything the bring-up uses is on the board.

- CPU on the internal OSC48M at 48 MHz. **The 24 MHz crystal on
  PA14/PA15 now RUNS** - `test_samc_clock` was its first consumer
  (2026-08-28) - and it is what finally put a scale on this board:
  OSC48M measures **5100 ppm SLOW** against it, so every absolute
  frequency measured here before that date was a ratio multiplied by
  a nominal 48 MHz that is half a per cent off. The crystal is armed
  by the suites that want it and handed back at the end of every
  letter; the CPU still boots and runs on OSC48M.
- **VDD is about 5.1 V**, located by `test_samc_supc` through the
  comparator's VDD scaler against the SUPC bandgap at all three of its
  reference levels (5251 / 5141 / 5090 mV). Toolchain, upload and
  debug: [samc/README.md](samc/README.md).
- LED **PB23**, button **PB22** (EXTINT6; measured 2026-08-28: the pad does NOT follow its own internal pull and reads LOW with the internal pull-up on, so the button is not fitted pull-up-to-ground).
- Console: CH340 on **PB30/PB31 = SERCOM5 PAD0/PAD1** (function D),
  115200 8N1. Like every CH340 here it has no USB serial number:
  identify its `/dev/serial/by-path` entry by listening on the
  candidates during an OpenOCD `reset run` and seeing which one
  prints the boot banner.
- Programmer/debugger: Atmel-ICE `J42700049508` on SWD (PA30/PA31) -
  the ICE that used to sit on AVR board B's UPDI. Flashing needs the
  cortex-debug session closed (the ICE is single-client).
- Desk position **C** in the manifest, driven by `tools/bench.py` like
  any other board (`flash`/`run`/`console`); its die serial is
  recorded there as the identity an AVR board has to be given by hand.
- Firmware today: `test_samc_journal`.

## Multi-board bench

The protocol work (USART/SPI/TWI) needs two chips talking: **board A =
the DUT** running a `test_avr_*` suite, **board B = a scriptable
instrument peer** (clock stretching, NACK injection, arbitration, a
foreign sender for auto-baud). The CH340 consoles are **observability
only** - firmware never goes in through them.

Three concerns, deliberately kept apart:

1. **Build** - one CMake target per app x board **TYPE**, auto-discovered
   from each app's own header comment at configure time
   (`CMakeLists.txt`). An app that must build for more than the bench
   chip says so directly: `// build: boards = db28,db32,db48` (`db48`
   is the default when the line is absent). A configure targets exactly
   one package (`AVR_MCU`, one value per configurePreset in
   `CMakePresets.json`) - switching preset switches the board, and only
   the apps whose allow-list includes it become targets. Package facts:
   `cmake/avr-mcus.cmake`.
   Never an env per physical board. `family_probe` is the carrier of
   the matrix and the first firmware to flash onto a new board.
2. **Identity** - `tools/bench_boards.py`, the bench **manifest**: a
   plain dict naming each board on the desk ("A", "B", "C" = desk
   positions, not chips), its board type, the USERROW label it is
   expected to carry (`id`, AVR only), its console and its programmer.
3. **Orchestration** - `tools/bench.py`, which resolves 1 against 2.

**The board TYPE is what carries the architecture.** `db28`/`db32`/
`db48` mean an AVR-Dx built by the `avrdx/` project and written by
avrdude over UPDI; `c21j` means a SAM C21 built by `samc/` and written
by OpenOCD over SWD. Everything above that line is one story on both -
the same console protocol, the same `ALL: N pass, M fail` verdict
grammar, the same campaign shape. Each project writes its own app
roster at configure time (`build-cmake/apps_avrdx.json`,
`build-cmake/apps_samc.json`) and the board type picks which to read:
they must be separate files because app NAMES COLLIDE across the two
source trees (`blink`, `console` and `probe` exist in both), so an app
name alone never identifies an app.

Identity works differently on the two families, and the SAM has it
easier: an AVR-Dx board must be labelled by hand (a string written
once into its USERROW) because nothing else about it is unique, while
every SAM die carries a factory 128-bit serial number no chip erase
can touch. The manifest records the SAM board's serial; comparing it
back needs a driver that reads it, which does not exist yet, so
today the check is the SWD readback spelt out in the manifest.

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
# any python3 with pyserial installed (pip install --user pyserial)
python3 tools/bench.py list                 # devices, probes, manifest (console present?)
python3 tools/bench.py flash A test_avr_pin # cmake --build --target <app> + avrdude over UPDI
python3 tools/bench.py flash C test_samc_dma # ... or + OpenOCD over SWD, by board type
python3 tools/bench.py run C z              # drive the console, judge "ALL: N pass, M fail"
python3 tools/bench.py console A            # print device path + speed (attach a monitor)
python3 tools/bench.py duo A:a B:script.txt # instrument peer scripted, then the DUT
python3 tools/bench.py fuses A              # read the fuses; name=value pairs write them
python3 tools/bench.py fuses C              # ... the SAM's NVM User Row, same verb
```

`flash`, `run` and `fuses` are architecture-blind at the command line:
the board name resolves to a type, the type to a project and a
mechanism. `--erase` is the exception and says so - it is an avrdude
option, refused on SAM where `program ... verify` erases exactly the
sectors it writes.

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

Fuses are provisioning, not build output: they survive every reflash,
and they are the programmer's to write. Both architectures have them
and `bench.py fuses <board>` is one verb over both - it reads and
decodes with no arguments, and writes with `name=value` pairs. What
the board TYPE decides is the memory underneath and therefore the
field names; a name belonging to the other family is refused BY NAME,
because "unknown fuse" would be the wrong diagnosis.

#### AVR-Dx: the FUSE bytes over UPDI

On AVR-Dx the CPU can read the fuses and nothing more (DS40002247B
11.3.1.5). `fuses <board>` reads the seven named bytes and spells out
the Flash geometry and the EESAVE state they produce; a write goes in
over UPDI and each byte is read back and reported as a before/after
pair.

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

#### SAM C21: the NVM User Row over SWD

On SAM C21 the fuses are a flash row like any other: the **NVM User
Row**, read at `0x00804000`, whose first 64 bits table 9-4 of
DS60001479M maps to BOOTPROT, the EEPROM emulation size, the BODVDD
detector, the watchdog and the region LOCK word. The peripherals load
those bits at power-on and at a user reset, so **a change takes effect
at the next reset and not before**, and the row **survives a chip
erase** - a wrong word is not undone by reflashing, which is exactly
why this is provisioning.

```bash
$PY tools/bench.py fuses C                        # read, every field decoded
$PY tools/bench.py fuses C bodvdd_hysteresis=1    # write, then reset, then read back
$PY tools/bench.py fuses C --rewrite              # write the row back unchanged
```

The fields, by their names here:
`bootprot`, `eeprom`, `bodvdd_level`, `bodvdd_disable`,
`bodvdd_action`, `bodvdd_hysteresis`, `wdt_enable`, `wdt_always_on`,
`wdt_period`, `wdt_window`, `wdt_ewoffset`, `wdt_wen`, `lock`. They
are the same fields `samc/nvm.hpp`'s `NvmUserRow` reads from the
firmware side, which is where their meanings are documented
([samc/nvm.md](samc/nvm.md), [samc/reset.md](samc/reset.md),
[samc/supc.md](samc/supc.md)); the two must move together.

**The write is chapter 27's own sequence, driven by hand over SWD with
the core halted**: Erase Auxiliary Row (`CMD 0x05`), then per page Page
Buffer Clear, 32-bit ascending stores into the row's address range to
load the page buffer, and Write Auxiliary Page (`CMD 0x06`) - every
command carrying the `0xA5` CMDEX key in the same 16-bit store as CMD
and every one followed by a wait on `INTFLAG.READY` and a read of
`STATUS`. `ADDR` is a half-word offset from the auxiliary section base
`0x00800000`, so the user row is `ADDR 0x2000`. The shipped OpenOCD's
`at91samd nvmuserrow` helper is NOT used to write: its interface is 64
bits wide while the erase takes a whole 256-byte row, so nothing in its
contract says what becomes of the rest of that row - and the same
command group carries `chip-erase` and `set-security`, which this tool
must never be one typo away from. Its READ half agrees with this one,
which is how the address and the layout were cross-checked.

**The safety rules**, all of them enforced:

- the whole **256-byte row** is read, modified and written back,
  because that is the unit the erase takes; pages that come out
  all-`0xFF` are left as the erase made them, and the tool says which
  pages it wrote;
- the old row is printed in full **before** anything is written;
- the row is read back after the write and **diffed** against what was
  intended; a mismatch is a nonzero exit;
- **only the decoded fields can be written.** The BODCORE calibration
  bits table 9-4 marks DO NOT CHANGE, and every Reserved bit, are
  carried across untouched, and **there is no raw bit escape** - a
  field the tool does not decode is a field it does not write;
- nothing here reaches the security bit or a chip erase in either
  direction;
- `bootprot`, `lock` and setting `wdt_always_on` need
  `--i-know-what-this-does`, because each can hand back a board that no
  longer takes firmware the ordinary way. Only a CHANGE is guarded:
  restating what is already there needs no ceremony.

`--rewrite` erases and writes the row back even when no field changes.
That is the end-to-end proof of the erase/write path at zero risk, and
it is what a new board or a new probe should be greeted with.

Every write ends the session with `reset run` (so the silicon is
running under what the row now says) and then clears
`DHCSR.C_DEBUGEN`, exactly as `flash` does; a read never halts the core
at all and clears the same bit on its way out, so the board is left as
it was found.

**Board C's row is the production default**, `0xB15088FF
0xFFFFF8BB` and `0xFF` for the remaining 248 bytes: no boot
protection, no EEPROM emulation area, BODVDD enabled at level 8 with
action RESET and no hysteresis, the watchdog off, every region
unlocked. The decode was verified against the row's own production
column, and `bodvdd_hysteresis` was taken to 1 and back with
`SUPC.BODVDD` read over SWD after each reset (`0x0008000A` ->
`0x0008000E` -> `0x0008000A`), which is the proof that the row really
is loaded into the peripheral at reset.

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

One `src/apps/<app>.cpp` = one `main()` = one target on the default
board (`db48`), plus that same target on every extra board type it
names (see "Multi-board bench" above) - all of it auto-discovered by
`CMakeLists.txt` at configure time, no generation step and no file to
regenerate. An app may carry `// build: <option> = <value>` lines in
its header comment (e.g. `// build: monitor_speed = 115200`) - see
"Per-app build options" in [avrdx/README.md](avrdx/README.md).

Shared code goes into `brio/`; any header an app includes is
compiled and linked by the LDF, no filter changes. What does NOT belong
there is bench tooling two apps happen to share - a wire protocol
between a DUT and its instrument is not framework. That kind of header
sits next to the apps in `src/apps/` and is included by its plain name
(`#include "usart_link.hpp"`), which the compiler resolves against the
including file's own directory: no build change, and the layering rule
stays honest because nothing under `brio/` knows it exists.

| App | What it does |
|-----|--------------|
| `family_probe` | The smallest firmware meant to run on EVERY package: PA7 toggling at ~2 Hz on the internal oscillator. Carrier of the board matrix (`// build: boards = db28,db32,db48`) and the first thing flashed onto a new board - PA7 alive means chip, UPDI link and fuses are sane |
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
| `test_avr_meter` | **Bench test suite** (keep passing): the METER SAMPLER - `util/meter_sampler.hpp` (the `MeterLatch` that bridges a capture interrupt to the loop and the `MeterSampler` AO that paces PUBLICATION instead of capture) over the real TCB meters - 3 tests / 38 verdicts, 38/38 on board B at 5 V, NOTHING TO WIRE: TCA0 drives its own WO0 pad on PD0 and the event system carries that pad's level back into the TCBs, the closed loop `test_avr_timer` established. Runs the kernel (a real `Kernel<P, Sink, Sampler>` pack; only the loop is the suite's). `a` a known rate at 1/5/20 kHz measured to the tick while the sampler publishes 8 samples a second against 1000, 5000 and 20000 captures - the design's whole point, asserted - and the pace doubled to prove it is a knob; `b` staleness: a stopped generator publishes NOTHING (silence is the signal), one burst is published once, and at 20 kHz against a 128-tick pace `missed()` counts 19901 overwrites where missed + published accounts for every one of the 19910 captures; `c` two sources in one sampler - a frequency meter and a pulse-width meter on the same PD0 waveform, period 24000 and width exactly the duty at 6000/12000/18000, labelled by pack order, and a source nobody feeds staying silent beside one that speaks. Claims PD0 and the console; PE0, the wire to the other board, is never touched. Re-runnable indefinitely: it writes no nonvolatile memory |
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
| `test_samc_tc` | **Bench test suite** (keep passing) for the SAM C21 timer/counters - `samc/tc.hpp` - 6 letters / 77 verdicts, 77/77 three times, NOTHING TO WIRE, and the campaign's point rather than another peripheral: letter f runs a **`MeterSampler` AO inside a real kernel**, fed by a `MeterLatch` that TC2's own capture ISR fills from a pin edge routed through the EIC and EVSYS - 41 captures, 19 published, 19 received, with NOT ONE LINE of `util/` changed for the second architecture. Also: the prescaler ratio measured at exactly 4.00x, COUNT32 by pairing TC0 with TC1 (9.6 million counts in 200 ms, and TC1 saying it is the client), PWM on the board's own LED with its duty sampled off the pad and its FREQUENCY read by a second timer counting its overflow events (937 in one second against 937.5 Hz), capture exact at 2344/937 ticks, and reading CCx proven to BE the capture acknowledgement. Findings in [samc/tc.md](samc/tc.md) |
| `test_samc_ac` | **Bench test suite** (keep passing) for the SAM C21 analog comparators - `samc/ac.hpp` - 6 letters / 94 verdicts, 94/94 three times, NOTHING TO WIRE: a pad driven by PORT is a rail the comparator can measure (the analog input is a direct pad connection, so unlike a digital peripheral function the output driver survives), and the second voltage is each comparator's own 64-step VDD scaler. Window mode reaches ALL THREE WSTATE values by swapping the roles - the scaler becomes the shared signal and the two rail-driven pads become the limits - since no pad on this board can sit between the rails; the four window interrupt selections each fire and each stay silent; a comparator flip and a window transition each move a DMA block through EVSYS; and letter f closes the loop with `samc/eic.hpp`, A PIN EDGE STARTING A SINGLE-SHOT COMPARISON through the AC's SOC0 user on the asynchronous path table 29-3 restricts it to. Findings in [samc/ac.md](samc/ac.md) |
| `tc_readsync_probe` | The one-behind experiment (2026-08-29): reproduce samc/tc.hpp's defective synchronized COUNT read, OBSERVE the crossing raw (the shadow lands half a counter-clock period after SYNCBUSY clears, unadvertised - SYNCBUSY.COUNT never rises for a READSYNC), refute the wait-for-the-rise fix, prove the DOUBLE-READSYNC one, price it (242 vs 117 us at 32 kHz, free at 48 MHz), check the TCC shares everything, and settle the n+1-vs-n+2 question (NO PRIMING: after a double read a lone command returns the previous read's landed snapshot, stale by the whole gap - two commands per call, always). The fix this probe proved lives in tc.hpp/tcc.hpp `read_sync()`; findings in [samc/tc.md](samc/tc.md) |
| `ac_sync_probe` | The AC sync-latency probe on the SAM C21 board (7 letters, 30 verdicts, wireless: PA04 GPIO-driven into COMP0 against its own VDD scaler, the CMP0 pad on PA12 read back through PORT.IN, GCLK_AC slowed to 11.7 kHz so a SysTick stopwatch resolves 1/4000 of a sampling period). Measured: the synchronized output costs the fraction to the next GCLK_AC edge + 2 whole periods (staircase + 1000 randomized shots + the independent OSCULP32K clock agree); findings in [samc/ac.md](samc/ac.md) |
| `test_samc_tcc` | **Bench test suite** (keep passing) for the SAM C21 timers for control applications - `samc/tcc.hpp` - 12 letters / 143 verdicts, 143/143 three times, NOTHING TO WIRE. Five of TCC0's eight outputs reach pads this board leaves free (PA08 = WO0 on function E, PA09 = WO1 on E, PA22 = WO4 on F, PA12 = WO6 on F), PA16 carries EXTINT0 as the fault stimulus, a second TC free-running at 3 MHz is the stopwatch and a third counts events. Measured: the two dead times exact at 899 and 2701 stopwatch ticks against 900 and 2700, and UNMOVED by a fourfold prescaler change (36.8.7's 'GCLK_TCC cycles' confirmed against the obvious alternative); a complementary pair never both high in 400000 paired samples; the dual-slope period exactly 2 x PER (942 overflows in 2 s against 942 predicted, where 2 x (PER+1) would give 937 - the AVR TCD's formula was off by that one and this chapter's is not); dithering buying a fractional period (938/933/928 against 937/932/928); a pin edge through the EIC and an asynchronous channel clamping an output, halting the counter and timestamping itself, with EVCTRL.MCEIx proven to be the gate 36.6.3.5 never names; erratum 1.21.9 measured (a SYNCHRONOUS channel does nothing at all); CTRLA.MSYNC moving the CHANNELS and not COUNT (47 to 937 matches a second while the client's own overflows stay at 47); and two facts the chapter does not carry - a buffered write's SYNCBUSY stands until the UPDATE consumes it, and a CCx read returns the buffered value while the pad still shows the old one. Findings in [samc/tcc.md](samc/tcc.md) |
| `test_samc_clock` | **Bench test suite** (keep passing) for the SAM C21 clock tree's two newest roots - `samc/clock.hpp`'s `Xosc` and `Fdpll` - 6 letters / 108 verdicts, 108/108 three times, NOTHING TO WIRE and nothing to jumper: OSCCTRL claims XIN/XOUT by itself when the oscillator is enabled. The instrument is `samc/freqm.hpp` with the crystal divided by 250 (96 kHz) as the reference, which at REFNUM 255 makes one count about 8 ppm of a 48 MHz measurand. THE BOARD'S 24 MHz CRYSTAL RUNS FOR THE FIRST TIME HERE: it starts in 554..576 us with STARTUP = 4, and weighed against it **OSC48M is 47.755 MHz, 5100 ppm slow** - inside table 45-57's tolerance, and a SCALE that corrects every absolute frequency this stratum reported before (OSCULP32K, read as 33074 Hz against a nominal 48 MHz, is 32907 Hz against the crystal). A clock failure is induced with no wire at all - clearing XTALEN leaves XIN a digital input nothing drives - and STATUS.XOSCFAIL, the latched INTFLAG and the safe-clock switch are all observed, then recovered through SWBEN. The DPLL locks to the crystal in about 40 us and its ratios come out EXACT to the count (127500 for LDR 23, 130156 for LDR 23 + 8/16, 63750 for a 96 MHz DCO divided by four); erratum 1.3.3 is seen directly (INTFLAG.DPLLLDRTO = 1 with STATUS.DPLLLDRTO = 0 in the same reading), and INTFLAG.DPLLLTO turns out to mark the lock TIMER finishing rather than a failure to lock. Letter e RUNS THE CPU FROM THE DPLL and brings it back, with the console alive throughout. Letter f settles GENCTRL's DIVSEL as 2^(DIV+1) by counting one generator against another with both fed by the same oscillator. Findings in [samc/clock.md](samc/clock.md) |
| `test_samc_supc` | **Bench test suite** (keep passing) for the SAM C21 Supply Controller - `samc/supc.hpp` - 3 letters / 44 verdicts, 44/44 three times, NOTHING TO WIRE and NOTHING FORCED: no brown-out is ever provoked, because every threshold the suite sets carries ACTION = none, under which STATUS.BODVDDDET still tracks. Letter b closes the loop three drivers wide - SUPC's VREF produces INTREF, `samc/ac.hpp`'s comparator takes it as its negative input against its own 64-step VDD scaler, and the crossing locates VDD: **5251 / 5141 / 5090 mV at the 1.024 / 2.048 / 4.096 V references**, the crossing step doubling with the reference exactly as a real voltage must. That is the gap [samc/ac.md](samc/ac.md) was carrying, and erratum 1.5.6's spurious comparator flag is observed on the way. Letter c sweeps the BODVDD threshold to find the same supply a second way (first detection at level 56) and derives the **level step as 48.7 mV**, settling table 45-18 against its own stated 60 mV; it also observes enable-protection both ways, catches that a store carrying configuration AND enable together leaves the protected fields untouched, confirms a sampled detector never reports ready, and restores the fuse configuration bit for bit. Letter a cross-checks the BODVDD register against the user row field by field and reads SUPC_BODCORE, which is real and running at an offset ch. 22 marks Reserved. Findings in [samc/supc.md](samc/supc.md) |
| `test_samc_rtc` | **Bench test suite** (keep passing) for the SAM C21 Real-Time Counter - `samc/rtc.hpp` - 8 letters / 125 verdicts, 125/125 (seven runs before the last fix, three after), about 40 seconds, NOTHING TO WIRE. The ruler is the BOARD'S CRYSTAL and not GCLK0: a TC0+TC1 pair runs as a 32-bit stopwatch at 3 MHz off generator 2 sourced from the 24 MHz XOSC, and `samc/freqm.hpp` weighs the RTC's own 32 kHz source against the same crystal - so this is the first suite here whose absolute frequencies carry no RC in the reference (OSCULP32K 33002 Hz, a trimmed OSC32K 33152..33174 Hz). Letter b proves THE COUNTER COUNTS ITS SOURCE tick for tick on all four clock selects (48..740 ppm, the residue being the RC's own wander between the two instruments' windows); letter c gets the prescaler exact to 70..300 ppm across DIV2/DIV32/DIV1024 and shows PRESCALER = OFF dividing by one while SILENCING every periodic interval; letter e moves a DMA block on a COMP0 event and again on a PER3 event, over an asynchronous EVSYS channel with no hardware trigger, and catches MATCHCLR raising the compare AND the overflow together; letter f answers what the read synchronization costs (2.2 us against 0.19 us raw, so it is background synchronization and not a per-read handshake) and what it hides (an unsynchronized COUNT is FROZEN, and a synchronized one trails the counter by a CONSTANT four ticks); letter g measures mode 1's period as PER + 1, catches that a mode change does not clear COUNT, and caught a DRIVER bug - the device header's compare-event group mask is one bit wide in the mode 0 view and two in the mode 1 view, so the natural macro silently drops CMPEO1; letter h walks the calendar over every boundary in one second each, confirms the chapter's own leap rule both ways, wraps year 63 to year 0 with OVF, and times the ALARM at 989 ms AFTER its match. Letter d is a lock-in: seven ABBA blocks with a median estimator put FREQCORR's full swing at 415..620 ppm where 24.6.8.2 predicts 258, and declines the per-step linearity because the trim's whole range is smaller than the RC's own wander. Findings in [samc/rtc.md](samc/rtc.md) |
| `test_samc_adc` | **Bench test suite** (keep passing) for the SAM C21's two analog-to-digital converters - `samc/adc.hpp` - 9 letters / 97 verdicts, 97/97 four times, about 25 seconds, NOTHING TO WIRE: an ADC input is a direct pad connection, so a pad driven by PORT is a rail the converter measures, and every other voltage in the room is internal (the SUPC bandgap, the quarter-scaled analog and core supplies, the three reference divisions of VDDANA). PA08 and PA09 are the only pads reaching BOTH converters, which is what makes letter d - one pad, two converters - possible at all. Letter c holds the crown finding: **MUXPOS INTREF is a flat zero without SUPC.VREF.VREFOE**, a connection chapter 38 never mentions, and with it set the three bandgap levels put VDD at 5276 / 5233 / 5201 mV where the comparator independently said 5251 / 5141 / 5090. Letter f is ruled by the board's crystal and gets six conversion configurations EXACT TO THE STOPWATCH TICK, then catches that the digital correction's 13-cycle single-conversion latency is not there though the correction itself takes exactly 100 counts off for an OFFSETCORR of 100. Letter g settles a documentation dispute by experiment - the window monitor's MODE4 is the COMPLEMENT of MODE3, as the device header says and 38.8.10's table does not. Letter h runs the no-CPU chain in both directions at once (a TC overflow starting the conversion over an asynchronous channel, the DMAC taking the result, a second TC counting the result-ready events) and letter i runs `util/analog_sampler.hpp` INSIDE A REAL KERNEL with not one line of util/ changed. Findings in [samc/adc.md](samc/adc.md) |
| `test_samc_dac` | **Bench test suite** (keep passing) for the SAM C21's digital-to-analog converter - `samc/dac.hpp` - 11 letters / 108 verdicts, 108/108 (three cold runs from a fresh flash and six warm ones), about 40 seconds, NOTHING TO WIRE, and the suite that closes the analog loop. THE GEOMETRIC GIFT of this chapter is that PA02 is DAC/VOUT, ADC0/AIN0 and the AC's AIN4 all at once, so erratum 1.8.9's "wire the DAC VOUT pin externally to an ADC AINx pin" is a wire of ZERO LENGTH and three drivers get the validation they were left holding: `AdcInput::dac`, `Ref::dac` and `AcNegative::dac`. Letter c settles the device header against the datasheet by experiment (REFSEL 0's `INT1V` name is the SAM D21's - the reference follows SUPC.VREF.SEL at 996/2011/4057 mV); letter d answers the question [samc/adc.md](samc/adc.md) could not (the ADC's INTREF REFERENCE path does not need VREFOE, where its bandgap INPUT channel is dead without it); letter f isolates erratum 1.8.9's OUTPUT noise with a control (3 counts of pad spread becoming 71..87 while a second converter samples the DAC internally, and 3 again with that converter free-running on any other input) and CATCHES ERRATUM 1.4.10 BEING WORSE THAN ITS OWN SENTENCE - once ADC1 has run, ADC0 will not enable at all and no reset clears it; letter h reports the transfer curve as the two converters' COMBINED nonlinearity and declines to apportion it; letter j runs a waveform with no CPU in the path (a timer event starting each conversion, the DMAC refilling DATABUF, a second timer counting EMPTY) and letter k reproduces erratum 1.9.2 across a real standby with a control on both sides. Findings in [samc/dac.md](samc/dac.md) |
| `test_samc_sdadc` | **Bench test suite** (keep passing) for the SAM C21's sigma-delta converter - `samc/sdadc.hpp` - 11 letters / 101 verdicts, 101/101 (two warm runs and one cold from a fresh flash), about a minute, NOTHING TO WIRE. This chapter is harder to test wirelessly than the other two converters: the SDADC's input is a PAIR of pads and nothing internal reaches it, so the DAC - which closed the ADC's and the AC's loops on PA02 - CANNOT be an input here and meets this converter only as a REFERENCE. What the board can put across a pair is both pads at one rail (an exact analog ZERO, letter b), the two pads at opposite rails (letter c), and TWO PWM WAVEFORMS OUT OF TCC1, whose WO0 and WO1 ARE PA06 and PA07 - i.e. AINN0 and AINP0 themselves - with the converter's own third-order SINC as the reconstruction filter, tuned so exactly 64 PWM periods fit one decimation window and the fundamental lands on a filter zero (letter d). Letter a asks the silicon which of the chapter's three self-contradictions about enable protection is true and finds a REFCTRL field the chapter never mentions; letter c proves the datapath is TWENTY-FOUR bits wide and the corrections in letter h prove they act in those units; letter e settles the PRESCALER against the device header's SAM D21 enumerators, crystal-ruled, and catches 39.6.2.3's "first valid sample is the third" in the act (a full-scale differential reading 5478 / 27623 / 32767 at SKPCNT 0 / 1 / 2); letter f reproduces erratum 1.8.10 with a control on both sides; letter g cross-checks the sigma-delta against the SAR on the one quantity both can see, a RATIO, to three parts in a thousand; letter j runs the whole chain with no CPU in it and letter k the automatic sequencer over all three pairs - which is over in 385 us, shorter than one console line, so nothing is printed between the START and the third result. Findings in [samc/sdadc.md](samc/sdadc.md) |
| `test_samc_tsens` | **Bench test suite** (keep passing) for the SAM C21's on-die temperature sensor - `samc/tsens.hpp` - 10 letters in `z` / 168 verdicts, 168/168 (three warm runs and two cold from a fresh flash), about two minutes - so run it with `--timeout 180`, above bench.py's 60 s default - plus letter `p` (erratum 1.19.1, 8/8) outside `z` because it might have reset the board. NOTHING TO WIRE - 43.5.1 is "Not applicable", the sensor has no pads - and NO THERMOMETER EXISTS ON THIS BENCH, which is the fact every verdict is designed around: no absolute accuracy is claimed anywhere, every band is preceded by its own noise measurement, and what cannot be answered is printed and DECLINED. Letter d is the crown and the reason the chapter is testable at all: this peripheral is a CLOCK RATIO rather than an ADC channel, so the SAME DIE read with GCLK_TSENS on OSC48M and on a crystal-locked DPLL at a true 48 MHz must differ by the reference's own error mapped through 43.6.1 - FREQM weighs OSC48M against the board's crystal in the same letter (5003 ppm slow), the arithmetic predicts -37 centi-C, and an INTERLEAVED A-B-B-A comparison repeated four times (a linear drift of the die's own temperature cancels exactly, and the MEDIAN is reported - the technique `test_samc_rtc`'s FREQCORR letter had to invent) MEASURES a median of -35 with a cycle-to-cycle spread of 7 against a single reading's spread of 42. Letter b catches the reset value being a trap - GAIN 0 IS 2^24, not none: a 699 ms conversion where 2 x 2^24 periods at 48 MHz predicts 699 ms to the millisecond, amplified two hundredfold - and prices CAL.TCAL/FCAL at 10.08 C. Letter e turns 43.8.13's one sentence into a formula (2 x GAIN + 2020 GCLK periods, the constant the same across a fourfold range) with the crystal as ruler and FREQM converting its ticks. Letter f walks OFFSET down to the rail and finds it at -2^23 to 250 counts in eight million, convicting 43.6.4's "more than 16 bits", and catches that an overflowed VALUE WRAPS rather than saturating - a plausible number of the WRONG SIGN. Letter g settles OUTSIDE as the COMPLEMENT of INSIDE, against 43.8.3's printed band. Letter h runs the chain with no CPU in it and, because table 29-3 grants THIS user all three propagation paths, measures what a sampled path costs: with the pacer's rate held at 1 kHz and only the pulse width and the channel clock changed, a 21 ns pulse reaches an asynchronous channel at 32 kHz but a synchronous one loses it unless the channel runs at the generator's own rate, and widening the pulse does not save a slow synchronous channel either. Letter i BINDS THE VECTOR, closing the gap dac.md and sdadc.md both carry (their suites read the flags and never took the interrupt). Letter p reproduces erratum 1.19.1 and finds it worse than its own sentence - the protected CTRLB write starts nothing AND raises no PAC flag, so it is dropped in complete silence. Findings in [samc/tsens.md](samc/tsens.md) |
| `test_samc_ccl` | **Bench test suite** (keep passing) for the SAM C21 configurable custom logic - `samc/ccl.hpp` - 7 letters / 141 verdicts, 141/141 four times (three warm and one cold from a fresh flash), about two seconds, NOTHING TO WIRE. Three stimuli, all inside the chip: a free pad walked between the rails by its own internal pull (PA16/PA17/PA18 into LUT0, PA08 into LUT1, read back on PA19 and PA11), TC and TCC waveforms the CCL takes internally, and the analog comparator fed from a GPIO-driven PA04. Letter a settles the enable-protection dispute RAW - three documents disagree and the four cells say a LUTCTRL write lands only with BOTH ENABLE bits clear, which caught a real bug in the driver's first version (a store into an enabled LUT is dropped in complete silence) - and confirms erratum 1.7.4 by watching CTRL.SWRST raise PAC INTFLAGC bit 23. Letter c settles TWO documentary disputes by experiment: INSEL 0x8 (TCC) has no enumerator in any device header of this pack and works anyway, and erratum 1.8.3 is revision B (TC0 drives the default TC input, TC4 does not, with the waveform read off TCC0's own pad and a stopped-timer control on each side). Letter e runs tables 37-2 to 37-5 and reproduces erratum 1.7.2 with a control on both sides. Letter f moves a DMA block on a LUT output edge and finds A SOFTWARE EVENT DOES CROSS AN ASYNCHRONOUS CHANNEL into a LUT - sixteen of sixteen, with a disconnected-user control and a second witness in memory - correcting what `test_samc_evsys` concluded from the DMAC alone. Letter g is the headline: at GCLK_CCL slowed to 11.719 kHz, a combinational LUT costs 0.05 periods, a LUT pair as a DFF the fraction and no whole period, one LUT with the synchronizer fraction + 1, the AC's own synchronized output fraction + 2 and the filter fraction + 3 - the answer [samc/ac.md](samc/ac.md) left open. Findings in [samc/ccl.md](samc/ccl.md) |
| `test_samc_debug` | **Bench test suite** (keep passing) for the SAM C21's four small debug-class chapters at once - `samc/pac.hpp` (11), `samc/dsu.hpp` (13), `samc/divas.hpp` (14) and `samc/mtb.hpp` (10.3) - 9 letters in `z` / 117 verdicts, 117/117 five times after the last fix (one cold from a fresh flash, four warm; the eight runs before it caught a flaky verdict in letter h and are why there was one), about 40 seconds; letters `c` (the PAC lock across two real resets, 11/11) and `k` (MTB AUTOHALT/HALTREQ with no debugger, 2/2) sit outside `z`, the first because it reboots the board twice and the second because a wrong answer would halt it. NOTHING TO WIRE: not one of the four blocks has a pad. One suite for four chapters because three of them meet here - the DSU comes out of reset PAC-protected, DIVAS's only error report is a bit in the PAC's AHB flag register, and the MTB's PAC identifier is a number only the PAC's register map states. Letter b is the campaign's headline: sixteen peripherals across all three bridges, each written back to itself with protection off (the control) and on, and ALL SIXTEEN report - which places erratum 1.19.1's silence on TSENS.CTRLB and not on TSENS, and erratum 1.7.4's flag-with-no-protection on the CCL, as the two poles of the same block; it also confirms erratum 1.13.2 WITH A CONTROL (the same illegal access flags on the MCLK and not on the PORT) and erratum 1.13.3 with controls on both sides (the same DIRTGL write lands through the IOBUS unflagged and is dropped and flagged through the APB). Letter a finds STATUSC coming up with BIT 25 set - PERID 89, outside both the chapter's drawing and the header's own register mask - and proves it answers WRCTRL. Letter c answers a question ch. 11 and table 18-1 both leave open: a PAC LOCK IS CLEARED BY A SYSTEM RESET AND BY A WATCHDOG RESET, so it lasts until the next reset of any kind. Letter d is the operational one - DID and the factory die serial checked against what `tools/bench_boards.py` records for board C. Letter e matches the DSU's CRC32 against a bitwise software CRC-32 (55x faster) and catches LENGTH being a working counter. Letter f resolves the CoreSight ROM's two entries to the M0+'s own table and to the MTB. Letters g and h measure DIVAS against gcc's own division (17x on a quotient, 30x with the remainder) and find the IOBUS the SLOWER path. Letters i and j read a self-hosted trace back out of SRAM with no probe attached and settle the MTB's event-user numbers in the device header's favour. Findings in [samc/pac.md](samc/pac.md), [samc/dsu.md](samc/dsu.md), [samc/divas.md](samc/divas.md) and [samc/mtb.md](samc/mtb.md) |
| `test_samc_timer_dma` | **Bench test suite** (keep passing) for the timers' SECOND pass - DMA-driven operation on `samc/tc.hpp` and `samc/tcc.hpp`, whose trigger ids both drivers had published and nothing had ever used, plus the advanced modes both docs still listed as gaps - 10 letters in `z` / 101 verdicts, 101/101 three times (twice warm, once cold from a fresh flash), under four seconds, NOTHING TO WIRE. THE INSTRUMENT IS THE POINT as much as the measurements: a TCC or TC waveform reaches a capture channel through a COMBINATIONAL CCL LUT published as an EVSYS generator, so both its edges are delayed alike and a period and a pulse width come out untouched, with no pad, no pull and no wire - `Lut<0>`'s INSEL "TC" is TC0's WO[0] and its "TCC" is TCC0's, so one fabric carries either. The headline is letter c's ROUND TRIP: one DMA channel plays an eight-entry duty table into TCC0's CCBUF0 while two `DmaPingPongEngine`s drain the capture meter's two registers, and over 192 judged samples the captured widths ARE the played table with a WORST ERROR OF ZERO TICKS and a phase that holds across every lap and block boundary. Letter b finds that a TC CAPTURE'S DMA REQUEST IS NOT A LEVEL waiting to be re-risen - a stream armed with INTFLAG.MCx standing starts anyway, and resumes from a dead stop with the flag up and CHCTRLB untouched - which qualifies the doctrine `samc/dmac.hpp`'s `kick()` carries from the SERCOM and the ADC. Letter e runs the HARDWARE answer to the same problem, WAVE.CICCEN and CIPEREN, against the software loop and records what each costs. Letters f and g close the TC's gaps (MFRQ, MPWM - where CC0 is spent as the period and channel 0's own output degenerates - the 16-bit `TcPwm`, INVEN, PWP/PW/STAMP, PRESCSYNC and ALOCK); h closes the TCC's waveform gaps and settles DUAL-SLOPE CRITICAL's arithmetic at (PER-CC0)+(PER-CC2), exact at three settings; i drives the fault system's second half from an EIC line and finds FCTRLn.FILTERVAL counting GCLK_TCC CYCLES and not prescaled ones; j stages ERRATUM 1.21.7 and records it UNREPRODUCED, with a control that proves the instrument would have seen it. Findings in [samc/tc.md](samc/tc.md), [samc/tcc.md](samc/tcc.md) and [samc/dmac.md](samc/dmac.md) |
| `test_samc_sleepwalk` | **Bench test suite** (keep passing) for what every OTHER peripheral does while the CPU is stopped - the transversal half of the sleep story, where [samc/platform.md](samc/platform.md)'s `test_samc_sleep` owns the Power Manager's own - 10 letters in `z` / 76 verdicts, 76/76 five times (one cold from a fresh flash), about 6 seconds, NOTHING TO WIRE; letter `p` (PM.bus_clock, 5/5) sits outside `z` because 19.5.2 calls turning that clock off one-way and a suite that believed the sentence would spend a reboot. THREE THINGS MAKE IT POSSIBLE and each is a finding. The RULER is a TC pair on the 24 MHz crystal with RUNSTDBY, the one clock that means the same on both sides of a WFI (kernel time STOPS in standby). The STIMULUS is a pad walked by HARDWARE: a timer's square wave through a combinational CCL LUT, whose OUTPUT VALUE crosses an asynchronous EVSYS channel to a PORT EVENT INPUT acting OUT - the one action 28.6.4 says survives a standby, and the reason `samc/pin.hpp` grew the PORT event surface [samc/port.md](samc/port.md) had declared a gap. The WITNESS is a second timer counting events or its own clock through the sleep, read before and after, with the same window spent awake as the control. Letter a judges ERRATUM 1.11.6 and finds it DOES NOT REPRODUCE at revision F (100 of 100 edges detected inside one standby by an asynchronous line, with the SysTick timebase and a dead TGL action as two independent proofs that the device really was asleep); b times the RTC's three wakes; c has FREQM finishing a measurement while the CPU sleeps; d answers whether the FDPLL stops in standby (it does NOT - the count across the sleep equals the count awake tick for tick); e runs a REAL SleepWalking ADC conversion paced by the RTC with no CPU in the loop, all four rows of table 38-4, and refutes erratum 1.4.5; f does the same for the SDADC's table 39-1 and the TSENS's 43-1; g proves 41.6.6 and reproduces erratum 1.9.2 with its control; h runs both sequences of 40.6.14; i settles CCL 37.6.4 exactly; j takes the TCC, an EVSYS channel - where CHANNELn.RUNSTDBY turns out to gate the ASYNCHRONOUS path too - and the BODVDD, whose INTFLAG is measured to be a transition and not a level. Findings in [samc/platform.md](samc/platform.md), "Sleep, peripheral by peripheral", and in each chapter's own document |
| `test_samc_sleep` | **Bench test suite** (keep passing) for the SAM C21 power manager and the depth ladder above it - `samc/sleep.hpp` (PM ch. 19, `Pm` + `SamSleepSite`), `samc/platform_sam.hpp`'s idle hook and `util/power.hpp` UNCHANGED on its second architecture - 8 letters / 87 verdicts, 87/87 three times, about 6 seconds, NOTHING TO WIRE. Two rulers, and the letter that earns the second: the RTC on OSCULP32K counts through a standby without itself sleepwalking (its clock is not a GCLK), while a TC0+TC1 pair on the 24 MHz crystal times single wakes at 42 ns - and letter c establishes which RUNSTDBY bit makes any counter survive a standby at all, five configurations deep, before anything else is believed. Letter a times the SLEEPCFG bridge latency (~5 us, so 19.6.3.3's readback rule is not a formality) and walks the ladder mapping this target had to choose; b prices IDLE0 at nothing and catches IDLE2 costing 24..30 us more; d shows the KERNEL TICK FREEZING (a 499 ms standby advanced it by 0 ms) and puts the standby wake at ~106 us (both absolutes CORRECTED 2026-08-29 after tc.hpp's one-behind READSYNC fix exposed the campaign's originals - 3.5..4.4 and ~17 us - as artifacts of the defective read; [samc/tc.md](samc/tc.md) has the mechanism, [samc/platform.md](samc/platform.md) the corrected numbers); e sweeps six regulator and back-bias combinations and finds NONE of them moves that number, printing its own repeat as the noise floor; f proves a sleepwalking task holds its whole clock chain up and that XOSC keeps running through a standby whatever RUNSTDBY says; g and h run the real kernel with `PowerManager` and two voters, through the vote round, the standing locks, the deadline guard and the first-event-after-wake contract, and end by measuring a time event slept over maturing 199 ms late. Every sleeping letter arms the WATCHDOG first, so a wake that never arrives costs a reboot and a banner instead of a mute board. Findings in [samc/platform.md](samc/platform.md) |
| `test_samc_eic` | **Bench test suite** (keep passing) for the SAM C21 external interrupt controller - `samc/eic.hpp` - 6 letters in `z` / 85 verdicts, 85/85, plus `n` (the NMI, 9/9) and `u` (the button, 4/4) outside it. NOTHING TO WIRE, on a chapter whose subject is EXTERNAL pins: letter b establishes the technique rather than assuming it - PMUXEN takes a pad away from PORT's OUTPUT DRIVER but NOT from its internal pull, whose direction is still the OUT bit, so a free pad walks between the rails on its own and every sense, filter and event path sees a real edge. Findings: a line that asks to be SAMPLED cannot even be ENABLED without an EIC clock while a clockless one can (the pending write completes the moment the clock arrives); a HARDWARE generator DOES cross an asynchronous EVSYS channel, answering what `test_samc_evsys` could not ask; every line generates events, not just EXTINT0-7; the NMI fires with the block disabled; and the crt's core-exception vectors were spelled the CMSIS way rather than the device header's, so an NMI landed in a silent spin. Findings in [samc/eic.md](samc/eic.md) |
| `test_samc_evsys` | **Bench test suite** (keep passing) for the SAM C21 event system - `samc/evsys.hpp` - 4 letters / 37 verdicts, 37/37, NOTHING TO WIRE: the software event supplies the stimulus and `samc/dmac.hpp` supplies the user. The measurement is A DMA TRANSFER THAT HAPPENED - a channel armed with no hardware trigger at all, so only an event can move its bytes. Letter d holds the finding the chapter does not have: a SOFTWARE event on an asynchronous channel does not reach the DMAC (eight move nothing, one on a clocked path moves a block) - a limit the CCL campaign later relocated to the USER's input stage, not the path (a LUT catches 16/16 on the same channel, [samc/ccl.md](samc/ccl.md)). Findings in [samc/evsys.md](samc/evsys.md) |
| `test_samc_osc32k` | **Bench test suite** (keep passing) for the SAM C21 32 kHz oscillators - `samc/osc32kctrl.hpp` - 3 letters / 32 verdicts, 32/32, NOTHING TO WIRE. Its instrument is `samc/freqm.hpp`, so it could not have been written before that driver existed, and letter b is where three drivers meet: nvm.hpp reads the production trim out of the NVM calibration area, osc32kctrl.hpp writes it into the oscillator and freqm.hpp says what it was worth - **47312 Hz untrimmed against 32995 trimmed**, which is 21.5.9's insistence measured. Also measures OSCULP32K's trim as a ~900 Hz-per-step knob (the watchdog rides on it) and shows a missing crystal reported rather than hung on. Findings in [samc/osc32kctrl.md](samc/osc32kctrl.md) |
| `test_samc_freqm` | **Bench test suite** (keep passing) for the SAM C21 frequency meter - `samc/freqm.hpp` - 4 letters / 25 verdicts, 25/25, NOTHING TO WIRE (every clock it measures is inside the chip). Also the first thing in this stratum to run a GCLK generator other than 0 on silicon. Its crown is letter d: OSCULP32K measured here against OSC48M reads 32957 Hz while `test_samc_platform` letter c, through the watchdog and SysTick, implies 32960 - 3 Hz apart, though the clock campaign later showed the two routes SHARE the OSC48M scale (the crystal puts both at ~32907 Hz, [samc/clock.md](samc/clock.md)). Letter c settles a documentation dispute by data (CFGA has no DIVREF: the bit does not even stay written) and measures the reference RC's short-term wander. Findings in [samc/freqm.md](samc/freqm.md) |
| `serial_speed` | A PROBE (not a suite) for the SAM C21 console link: how fast it really goes and what it costs. Menu-driven at any of eight rates from 115200 to 3 Mbaud, with the transport switchable between the plain interrupt path and the DMA engines, a 64 KB transmit burst fed per-byte or in bulk, the same burst RAW (polled DRE, no ring/irq/DMA - the link's own ceiling) and an echo window. The host script checks every byte. Findings in [samc/sercom.md](samc/sercom.md): the wire is good to 3 Mbaud, 2.5 M is a divisor hole, and the per-byte API was the real limit |
| `test_samc_platform` | **Bench test suite** (keep passing) for the SAM C21 reset controller and watchdog - `samc/reset.hpp` - 3 letters in `z` / 34 verdicts, 34/34, NOTHING TO WIRE. The boot story with RCAUSE read as the exclusive one-cause register it is, and the watchdog's power-on state checked field by field against the NVM User Row that supplied it (two drivers describing the same fuses from opposite ends); the watchdog as a configurable timer with nothing allowed to expire; and OSCULP32K measured BY DIFFERENCE through the early-warning interrupt - 1030.4 Hz against a nominal 1024, with the 3 ms arming cost that a single measurement would have hidden. Letter `i` sits OUTSIDE `z` because it reboots the board six times: a wrong CLEAR key with the watchdog stopped and then running, a panic through `ResetReporter`, a deliberate HardFault, a time-out and a window violation - 20/20, with the breadcrumb proven to cross a system reset. Run it with `bench.py run C i --expect="->"`. Findings in [samc/reset.md](samc/reset.md) |
| `test_samc_nvm` | **Bench test suite** (keep passing) for the SAM C21 NVMCTRL - `samc/nvm.hpp` and `samc/nvm_flash.hpp` - 6 letters in `z` / 52 verdicts, 52/52, NOTHING TO WIRE. Geometry and the fuse row against PARAM and the factory areas (the die serial it prints is the one the manifest records for board C); the RWWEE erase/program round trip with every malformed request refused; THE PAGE-BUFFER ORDERING RULE decided by data (ascending and descending both exact, even-then-odd loses all eight low words - so the rule is 'one 64-bit section at a time', not 'ascending'); the cost and the no-stall claim (RWWEE row erase 989 us with ~3950 CPU polling turns inside it); `util/nv_heap.hpp` mounting and round-tripping a block on the RWWEE array, its second silicon; region locks refusing an erase and STATUS.PROGE from an invalid command. Letter `m` sits OUTSIDE `z` because it costs one row of main-array endurance: it measures the other side of the stall (ONE polling turn) and why a stalled operation cannot be timed from flash-resident code. Findings in [samc/nvm.md](samc/nvm.md) |
| `test_samc_journal` | **Bench test suite** (keep passing) for the SMALL-VALUE FLASH JOURNAL - `util/nv_journal.hpp` over `samc/nvm_flash.hpp`'s `RwweeJournalZone` - 7 letters in `z` / 58 verdicts, 58/58 (one cold, two warm), NOTHING TO WIRE. It is the other half of `test_samc_nvm`: the RWWEE array is PARTITIONED, rows 0..27 for the block heap and rows 28..31 for the journal's two ping-pong halves, and letter `e` runs BOTH AT ONCE - a 300-byte heap block stays byte-exact at the same address while the journal collects repeatedly over it, which is the partition's whole point and the thing no host test can say anything about. Letter `a` proves the mount COSTS NO ERASE AND NO PROGRAM (158 us for two 512-byte halves); `b` the byte core, the typed twin, latest-wins and the refusals, with a refused save proven to write nothing at all; `c` the halves ping-ponging on silicon with every value carried through and a fresh mount to prove it; `d` the cost, weighing a JOURNAL SAVE (347..357 us) against a BARE PAGE PROGRAM (190 us) measured in the same window - the difference is the entry image and its bitwise CRC-16 - plus the no-stall claim (about 2500 CPU polling turns inside one RWWEE row erase) and the software timebase keeping up through three collections; `f` THE PANIC RESERVE on real flash, where `save_reserved()` of a maximum-size value succeeds after every completed save and costs NO erase; `w` prints the run's wear (168..184 row erases in four rows, about 46 cycles of each) against a declared budget and zeroes the meter so the letter is re-runnable. OUTSIDE `z`: `p` reboots the board - a real `panic()` whose reporter writes the breadcrumb through the reserve, then a reset, then `take()` at the next boot (11/11), and it records WHICH path wrote it, which on a board with `DHCSR.C_DEBUGEN` cleared is the HardFault body, because `break_here()`'s BKPT escalates before any reporter runs; `v` verifies the survivors after a reflash (6/6 after flashing `blink` over the top and coming back - the journal's sequence number unchanged). Findings in [samc/nvm.md](samc/nvm.md), design in [design/nv-journal.md](design/nv-journal.md) |
| `test_samc_uart` | **Bench test suite** (keep passing) for the SERCOM USART transport - `samc/sercom.hpp` in all FOUR of its shapes, interrupt or DMA on each direction independently - 4 letters in `z` / 27 verdicts, 27/27, plus ELEVEN letters outside `z` that need a peer on the other end of the console wire and are driven by `tools/uart_stress.py`: the transport matrix (e..h), receive-only and transmit-only sustained (i, j), 115200 / 1 M / 3 Mbaud (k), the frame-format matrix (l), a deliberately MISMATCHED frame and the recovery (m), ring pressure at an eager and a lazy harvest cadence (n), and bursty traffic with idle gaps (p) - 62 verdicts, all green twice over. NOTHING TO WIRE: the console itself is the peripheral under test, which is also why most letters need the host - this SERCOM has no second port and this silicon has no loop-back bit, so a byte can only be checked against something outside the chip, and only an outside sender can speak a frame the receiver is not using. The pattern is a 32-bit xorshift both ends generate, so a lost byte is LOCATED and not merely counted. It is the suite that diagnosed the duplex wedge (see [samc/dmac.md](samc/dmac.md)'s erratum-1.10.4 note) and it is what keeps the fix honest. Its z letters are self-contained: the baud arithmetic against half a BAUD step at any rate, every frame format written and read back, the transmit ring's refusal contract, and the engines' register facts | 
| `test_samc_dma` | **Bench test suite** (keep passing), the FIRST on the SAM C21 board and the first user of `util/testbench.hpp` on that target: the DMAC - `samc/dmac.hpp` (block, channels, harvest with the erratum-1.10.4 validation) and the Uart's two OPTIONAL DMA engines - 9 letters in `z` / 112 verdicts, 112/112, NOTHING TO WIRE (the console's own SERCOM5 is the peripheral under test; two letters that need the runner to type sit outside `z`). The end-address quirk decided by data (a naive start-address descriptor shown moving the decoy buffer); the memory-to-memory matrix; software-linked chains at ~59700 blocks/s; harvest at ~10 us with scribbled write-backs REFUSED; TCMPL/TERR/INTPEND dispatch including a real bus error and the first-beat loss that follows it; the console transmitting and receiving through DMA channels; the erratum-1.10.4 hunt (five concurrent channels, the violation counters as the verdict - zero at low trigger density, hundreds refused and zero suffered under the engined stress). Driven by `tools/bench.py` like any AVR suite (`flash C test_samc_dma`, `run C z`) |
| `test_samc_analog` | **Bench test suite** (keep passing) for THE ANALOG COMPLETION - the gaps chapters 38 (ADC), 39 (SDADC), 40 (AC), 41 (DAC) and 43 (TSENS) still carried after their own campaigns and that became testable once the DAC on PA02 was a real swept mid-scale source, `samc/supc.hpp` had the bandgap and the TC event pacers existed - 12 letters in `z` / 136 verdicts, 136/136 four times (one from a fresh flash), about three seconds, NOTHING TO WIRE, and NOT ONE LINE of driver code needed for it. Letter `a` runs the ADC HOST/CLIENT PAIR (38.6.3.1) on silicon for the first time: one trigger on the host gives two results agreeing to zero counts on the pad both converters bond, the CLIENT'S OWN ENABLE BIT DOES NOT STAND (CTRLA reads 0x20 and it converts anyway - 38.6.3.1's "enabled by accessing the CTRLA register of Host ADC" meant literally), interleaving delivers one result per trigger where one converter delivers half of them WITH ITS OVERRUN FLAG CLEAR (a missed trigger is not an overrun), and ONLY ONE OF THE CHAPTER'S THREE RESTART OPTIONS RESTARTS ANYTHING - the parity carries straight through both a SWTRIG.FLUSH and a disable/enable cycle, and only a software reset sets it. Letter `b` walks a six-input SEQUENCE whose slots are 222 counts apart at the closest, all six labelled by SEQSTATE and cheaper per conversion than six triggers; `c` measures DIFFERENTIAL mode against the DAC (half the single-ended difference to three counts, a real zero crossing at the code that IS a quarter of the supply, a NEGATIVE signed WINLT separating two legal results); `d` finds NEITHER R2R NOR OFFCOMP moving the reading by more than 2 counts of 4096 at any of three common modes, and offset compensation SHORTER than the SAMPLEN it replaces. Letter `e` is the campaign's showpiece: DAC DITHERING with the pacer, the ADC accumulation and the sample rate chosen so 1024 samples land on each of the sixteen sub-conversion slots exactly 64 times - the dithered mean then steps 4.1 counts per dither bit where one sixteenth of an LSB is 4.0, against an undithered control that steps a whole LSB with nothing between, and erratum 1.9.1 confirmed revision-B-only by running the staircase in both data placements. Letters `g` to `j` finish the AC: COMP2, COMP3 and WINDOW 1 on a signal that really sits between the limits; THE BANDGAP AS A NEGATIVE INPUT, which turns out NOT to need SUPC.VREF.VREFOE and corrects `ac.hpp`'s own comment; erratum 1.5.6 reproduced rarely with its workaround as the control; THE HYSTERESIS MEASURED at 118 mV high-speed and 113 mV low-power with BOTH edges moving (erratum 1.5.1 confirmed revision-B-only); and 40.6.10's SWAP procedure returning a BOUND rather than a number. Letter `k` closes the SDADC's flush (a negative witness: flush events faster than a decimation window stop every result dead), its window event and its three interrupts; `l` the TSENS inverted start event, both driven by a comparator OUTPUT because only a LEVEL can tell an inversion apart from a pulse. Findings in [samc/adc.md](samc/adc.md), [samc/dac.md](samc/dac.md), [samc/ac.md](samc/ac.md), [samc/sdadc.md](samc/sdadc.md) and [samc/tsens.md](samc/tsens.md) |
| `test_samc_analog_dma` | **Bench test suite** (keep passing) for the two STREAMING DMA engines - `samc/dmac.hpp`'s `DmaLoopEngine` and `DmaPingPongEngine` - and for the element-type generalization of all four (`Elem` is the beat, through one `sizeof` that feeds both BEATSIZE and the end-address arithmetic) - 11 letters in `z` / 78 verdicts, 78/78, NOTHING TO WIRE. The chain letters run with no CPU in the sample path at all: a TC's overflow starts the DAC through EVSYS while one channel refills DATABUF, and the same TC's CC0 match starts ADC0 on the SHARED PA02 PAD while another channel drains RESULT into alternating buffers - one sample of each per 200 us period at 5 kHz. THE HEADLINE is letter `c`: with a 32-entry table and 24-sample blocks, chosen not to divide each other so a lost sample cannot hide in a coincidence, the table entry each block starts on steps by exactly 24 (mod 32) block after block, and every sample sits 3..6 counts from a static calibration whose own noise is 4..6 and whose table step is 120 - so a loop closed in SOFTWARE, from the TCMPL interrupt, loses nothing at any lap or block boundary. Letter `b` measures that noise BEFORE the band is chosen; `d` the rate against the wall clock (4980/s against 5000, both rulers OSC48M, so it checks the arithmetic and not the oscillator); `e` the overrun contract under a sleeping drainer (both buffers held, the lap SKIPPED rather than the held buffer overwritten, the stall counted here and the lost SAMPLES counted by the converter's own OVERRUN); `f` erratum 1.10.4 under concurrent churn, where a sprayed churn reaches 2733..2744 refused readings in ~36000 harvests while a bounded-wait one reaches none in 43000 blocks - and where `abandon()` turns out NOT to be enough, a channel whose ENABLE will not clear needing a reset of the BLOCK; `g` where a trigger's edge comes from (writing TRIGSRC onto a standing request is itself a rise); `h` a scribbled write-back refused and a dead block abandoned and resumed; `i` and `j` the SDADC and TSENS on WORD beats, where a halfword would carry a different number and not a narrower one; `k` the util contract live - a BlockRelay (util/block_stream.hpp, design/block-stream.md) inside a REAL kernel, the ADC's blocks travelling as Lease::dispatch loans and verified INSIDE the loan window, every buffer returned, the engine never skipping a lap (and the letter that re-taught the print lesson: a verdict printed between the stream's start and the pump overran the engine once, deterministically, in every z run - the console ring full of the previous letters' output is milliseconds of blocking, and the engine's slack is two blocks). Findings in [samc/dmac.md](samc/dmac.md), with the per-chapter halves in [samc/dac.md](samc/dac.md), [samc/adc.md](samc/adc.md) and [samc/sdadc.md](samc/sdadc.md) |
| `test_avr_power` | **Bench test suite** (keep passing): the POWER MANAGER - `util/power.hpp`, `AvrSleepSite`, `TimeEvents::ticks_to_next()` and the one branch of `AvrPlatform::idle()` that makes them work - 5 tests / 44 verdicts, 44/44 on board B at 5 V, NOTHING TO WIRE and no pin claimed but the console. THE ONLY SUITE HERE THAT RUNS THE KERNEL: the object under test is an active object, so the rounds go through real queues, real dispatch and a real `Kernel<P, Probe, Bus, Pm>` pack - only the loop is the suite's, and where a sleep is the point it calls the kernel's own `idle_if_empty()`. A TCB pair cascaded at CLK_PER with `RUNSTDBY` on both halves is the stopwatch (it counts through the standby it is timing), the PIT is both the 1024 Hz timebase and the wake source, and a fake bus engine gives the `BusMaster` something to be busy with. `a` the ladder against `SLPCTRL.CTRLA` at every rung; `b` a real standby round - the loop frozen at exactly 32 turns over 32 ticks where awake it turns ~13500, the mode staying armed across a wake that says nothing, the 10-12 cycle wake, the 157 us round, and the first event afterwards disarming and publishing its `WakeReport`; `c` the deadline guard refusing `deep` one tick from a deadline and accepting it 1000 away; `d` the voters, with a transfer in flight aborting the round and its completion making the same request succeed; `e` the standing restrictions, nested, moved and released. Re-runnable indefinitely: it writes no nonvolatile memory and leaves nothing armed |
| `spi_peer` | The INSTRUMENT half of the SPI campaign, for board B: one blocking loop that shifts whatever the DUT clocks, decodes a command frame off SPI0 ALT1, acknowledges it and becomes for a bounded moment whatever the DUT needs at the other end - a client in any transfer mode, bit order and buffering regime; a client that never drains, or one that misses a load on purpose; a second driver on the shared select wire (the only way to demote a real host); a self-selecting client for USART Host SPI. It is a DARK LISTENER: it drives MISO for exactly one answer window, entered only after a frame that checked out, so it can stay on the desk while the DUT runs its single-board half. Console (observability only): `?` help, `i` status and counters, `0` back to the dark client, `3` command trace |
| `sleep_peer` | The INSTRUMENT half of the SLEEP campaign, for board B, and the only ruler that can time a wake-up: a sleeping chip cannot, because the mode stops the very counter that would. One blocking loop decodes a command frame off the shared PE0 wire (`src/apps/sleep_link.hpp`) and becomes for a bounded moment whatever the DUT needs - a train of stimulus edges on PE2, each zeroing a 32-bit CLK_PER stopwatch that the DUT's echo on PE3 CAPTURES through the event system (no software in the measurement path); one byte at a commanded baud on the same wire, for the DUT's start-of-frame wake; or one host write tenure on the office I2C bus against the DUT's TWI client, timed end to end. Its 24 MHz crystal is dead, so it counts on OSCHF - a per-cent-class reference, ample for microsecond-to-millisecond figures, and its banner and its `ident` answer both say so. Console (observability only): `?` help, `i` status, counters and the stored shot times, `0` back to command mode, `3` command trace |
| `twi_peer` | The INSTRUMENT half of the TWI campaign, for board B: one blocking loop around the polled TwiClient surface that answers one address, decodes a command frame off the bus, acknowledges it and becomes for a bounded moment whatever the DUT needs - a client that clock-stretches by a commanded time per byte, one that NACKs the n-th byte or is not there at all, one that answers the General Call or stops answering it, a second HOST racing the DUT for the wire, a second client sharing one address (so a read is served by two devices and one of them collides), or a stuck client holding SDA low from PORT until enough SCL edges have gone by. Every action carries a count and a deadline after which the peer restores its command-mode client by itself. Console (observability only): `?` help, `i` status and counters, `0` back to command mode, `3` command trace |
| `usart_peer` | The INSTRUMENT half of the USART campaign, for board B: one blocking loop that decodes a command frame off the link, acknowledges it and becomes for a bounded moment whatever the DUT needs on the other end - echo, silent sink, generator, full-rate flood, break, foreign auto-baud sender, cycle-counted bit-banger (a stretched bit cell, sub-bit glitches), one-wire responder. Every mode-changing command carries a frame count and a deadline after which the peer restores command mode by itself. Console (observability only): `?` help, `i` status and counters, `0` command mode, `1` one-wire standby, `2` wiring probe, `3` command trace |
| `sampler` | The ADC inside the kernel: `AnalogSampler` over ADC0 walking PD1 (the DAC loop), die temperature and VDD/10, published to a Monitor (one line a second) and an Alarm (LED PF2 above a threshold); console DAC/PACE HW|SW|OFF/ALARM/CLOCK/STAT. Bench: 512 samples/s (PIT/64) with no queue drops, 128/s default, CLK_ADC follows CLOCK 4M/24M under sampling |
| `traffic0` | The over-commented AO learning testbed: 4 buttons -> 4 RGB lamps, one AO per role, publish for button facts |
| `traffic1` | The traffic light FSM: timed phases via one re-armed time event, a remembered pedestrian call |
| `traffic2` | traffic1 with PWM lamps (TcaPwm split mode, colour palette): the actuator changes, the AOs do not |
