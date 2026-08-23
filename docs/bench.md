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
- Supply: jumper-selectable **3.3 V / 5 V**; VDDIO2 is powered, so
  PORTC (the MVIO domain) is usable - and could one day talk 3.3 V
  logic while the rest of the chip runs 5 V, no level shifters.
- Clock: 24 MHz crystal on PA0/PA1 (not GPIO). RTC on internal OSC32K
  (no 32 kHz crystal, PF0/PF1 free).
- Serial link: CH340 on **USART2 ALT1, PF4/PF5**, 460800 baud in the
  console apps.
- Programmer/debugger: one Atmel-ICE per board over UPDI, AVR port,
  6-pin ISP header (pin 2 VCC, 5 UPDI, 6 GND); A's probe is
  `J42700049508`, B's `J42700051207`.

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
usable from a script. `duo` is the campaign's shape and is **not yet
exercised** (board B is on the desk, still running its delivery
firmware).

## Wiring (SPI/I2C experiments, 3.3 V rail)

The 3.5" module has no level shifter on the display signals, so the
bench runs on the 3.3 V rail.

| Signal | Pin | Notes |
|--------|-----|-------|
| SPI0 MOSI / MISO / SCK | PA4 / PA5 / PA6 | shared bus |
| Display CS / RS(DC) / RST | PD0 / PD1 / PD2 | ILI9481 |
| SD_CS (module) | PD4 | reserved, unused |
| T_CS (touch) | PD5 | XPT2046, same bus, PEN unused |
| MCP3550 CS | PB0 | |
| TWI0 SDA / SCL | PA2 / PA3 | 1.5k pull-ups to 3.3 V |
| MCP47CVB22 | I2C 0x60 | A0 and LAT/HVC to GND (LAT transparent); VOUT0 -> MCP3550 input |
| LED (blink apps) | PF2 | PF2 -> ~330 ohm -> LED -> GND |
| Traffic bench: LED1 R/G/B, LED2 R/G/B | PB0/1/2, PB3/4/5 | common cathode, TCA1 WO0-5 (PWM from traffic2) |
| Traffic bench: LED3 R/G/B, LED4 R/G/B | PC0/1/2, PC3/4/5 | common cathode, TCA0 WO0-5 |
| Traffic bench: buttons 0..3 | PA2..PA5 | to GND, internal pull-ups |
| Event probes (events0) | PD2 (EVOUTD), PC2 (EVOUTC), PF2 (EVOUTF = LED) | logic analyzer on PD2/PC2 |
| CLKOUT (test_avr_clock) | PA7 | scope: CLK_PER |
| Analog loop (test_avr_analog, sampler) | PD6 (DAC0 OUT) -> PD1 (AIN1), PD6 -> PD7 (VREFA) | two jumper wires |
| Board-to-board serial link (test_avr_serial, USART campaign) | A.PE0 - B.PE1, A.PE1 - B.PE0, A.PE2 - B.PE2, GND - GND | four jumper wires; board B must hold PORTE as inputs (flash it with `family_probe`) |

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
[avrdx/README.md](avrdx/README.md). Shared code goes into
`lib/brio/src/`; any header an app includes is compiled and linked by
the LDF, no filter changes.

| App | What it does |
|-----|--------------|
| `family_probe` | The smallest firmware meant to run on EVERY package: PA7 toggling at ~2 Hz on the internal oscillator. Carrier of the board matrix (`// pio: boards = db28,db32,db48`) and the first thing flashed onto a new board - PA7 alive means chip, UPDI link and fuses are sane |
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
| `i2c_scan` | I2C stack test: address sweep 0x08..0x77 every 2 s on TWI0 through I2cBus + the Twi<0> engine, ACKs printed as found (expected: 0x60) |
| `events0` | The event system on the bench (verified): PIT/8192 -> EVOUTF (LED at 4 Hz, no CPU), channel 1 rewired every 10 s by an AO (PIT/64 512 Hz, button PA2 level, off) -> EVOUTD PD2 for the analyzer, software pulses on PC2 |
| `test_avr_clock` | **Bench test suite** (14/14 on the scope): CLKCTRL - OSCHF rates, prescalers, tune, crystal vs OSCHF, 32 kHz main clock, forced clock failure + recovery, PLL/status; CLK_PER on PA7 (CLKOUT) for the scope; console 9600 (talks down to 153.6 kHz; silent only at the 32 kHz main clock) |
| `test_avr_analog` | **Bench test suite** (keep passing): VREF/DAC/ADC, 14 tests / 54 verdicts knob by knob - references cross-check, ramp, OUTEN, settling, resolution, differential, prescalers, accumulation, sampling knobs, event start, window, errata 2.3.2, internal inputs, VREFA from the DAC. Supply measured at start (3.3 V and 5 V). Wire PD6->PD1, PD6->PD7. 54/54 on rev A5 @ 3.3 V |
| `test_avr_timer` | **Bench test suite** (keep passing): TCA/TCB/CCL/AC, 11 tests / 82 verdicts, 82/82 on rev A5 at 3.3 V - FrequencyGenerator/TcaPwm16/Heartbeat on PD0 measured back by the TCB meters through EvPin, OneShotPulse on PB5, Pwm8 on PC0, PulseCounter, 32-bit cascade vs the Ticker, PeriodicTick, Timeout, EventCounter with direction from PC1; CCL LUT4 on PB0/PB1 -> PB3 and a JK flip-flop by timer events; AC thresholds/window against the DAC on PD6. Nothing to wire (PB0/1/3/5 = traffic LEDs flicker). Holds in crystal time (PIT paused): the Ticker's OSC32K measured +0.94 % fast |
| `test_avr_pin` | **Bench test suite** (keep passing): PORT - the pin senses counted on self-driven edges, level_low, INVEN, input_disable, the W1C flag discipline, the multi-pin engine across two ports, the Port mask verbs and the slew bit. 22/22 on rev A5. Nothing to wire |
| `test_avr_rtc` | **Bench test suite** (keep passing): RTC/PIT, 8 tests / 78 verdicts, 78/78 on rev A5 - the counter's period at three prescalers, the compare phase (CMP + 1 ticks), crystal error correction at +-127 ppm, the synchronization busy flags, the first tick after an enable, the PIT and the counter running together, OSC1K as CLK_RTC. A TCB cascade at CLK_PER is the stopwatch and the RTC's own OVF/CMP events latch it: nothing to wire. Takes about two minutes (the correction is measured by averaging) |
| `test_avr_serial` | **Bench test suite** (keep passing): USART, 9 tests / 108 verdicts, 108/108 on rev A5 at 5 V - instances and routes including the pinless NONE and the teardown, the whole 36-combination frame-format matrix in loop-back on USART4, the receive FIFO's overflow, the MPCM filter, DRE vs TXC, the baud generator measured on PE0 through EvPin + a TCB pulse-width meter (9600 / 115200 / 460800 / 1 Mbaud and CLK2X), a 24 -> 12 -> 24 MHz rebase under traffic, GENAUTO/LINAUTO auto-baud with the errata 2.16.3 recovery, and Host SPI in loop-back. Nothing to wire; drives PE0/PE2 (the link to board B, which must be holding them as inputs), and briefly PA4, PC0, PB4 for the per-instance smoke. Menu letter per test, `z` runs them all |
| `sampler` | The ADC inside the kernel: `AnalogSampler` over ADC0 walking PD1 (the DAC loop), die temperature and VDD/10, published to a Monitor (one line a second) and an Alarm (LED PF2 above a threshold); console DAC/PACE HW|SW|OFF/ALARM/CLOCK/STAT. Bench: 512 samples/s (PIT/64) with no queue drops, 128/s default, CLK_ADC follows CLOCK 4M/24M under sampling |
| `traffic0` | The over-commented AO learning testbed: 4 buttons -> 4 RGB lamps, one AO per role, publish for button facts |
| `traffic1` | The traffic light FSM: timed phases via one re-armed time event, a remembered pedestrian call |
| `traffic2` | traffic1 with PWM lamps (TcaPwm split mode, colour palette): the actuator changes, the AOs do not |
