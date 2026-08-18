# The bench

The board, the wiring and the apps as they are TODAY. This page is the
volatile end of the documentation: apps are disposable tools that test
the framework's ideas and will not survive in their current form;
nothing in `docs/design/` or in the target pages depends on them.
Every app documents itself in its own header comment - this table is
only the map.

## Board

- MCU: AVR128DB48 (48-pin, 128 KB flash, 16 KB SRAM), see
  [targets/avrdx.md](targets/avrdx.md) for toolchain, probe and clock.
- Supply: jumper-selectable **3.3 V / 5 V**; VDDIO2 is powered, so
  PORTC (the MVIO domain) is usable - and could one day talk 3.3 V
  logic while the rest of the chip runs 5 V, no level shifters.
- Clock: 24 MHz crystal on PA0/PA1 (not GPIO). RTC on internal OSC32K
  (no 32 kHz crystal, PF0/PF1 free).
- Serial link: CH340 on **USART2 ALT1, PF4/PF5**, 460800 baud in the
  console apps.
- Programmer/debugger: Atmel-ICE over UPDI, AVR port, 6-pin ISP header
  (pin 2 VCC, 5 UPDI, 6 GND).

## Wiring (SPI/I2C experiments, 3.3 V rail)

The 3.5" module has no level shifter on the display signals, so the
bench runs on the 3.3 V rail.

| Signal | Pin | Notes |
|--------|-----|-------|
| SPI0 MOSI / MISO / SCK | PA4 / PA5 / PA6 | shared bus |
| Display CS / RS(DC) / RST | PD0 / PD1 / PD2 | ILI9481 |
| SD_CS (module) | PD4 | reserved, unused |
| T_CS (touch) | PD5 | XPT2046, same bus, PEN unused |
| MCP3550 CS | PB0 | was PD3 |
| TWI0 SDA / SCL | PA2 / PA3 | 1.5k pull-ups to 3.3 V |
| MCP47CVB22 | I2C 0x60 | A0 and LAT/HVC to GND (LAT transparent); VOUT0 -> MCP3550 input |
| LED (blink apps) | PF2 | PF2 -> ~330 ohm -> LED -> GND |
| Traffic bench: LED1 R/G/B, LED2 R/G/B | PB0/1/2, PB3/4/5 | common cathode, TCA1 WO0-5 (PWM from traffic2) |
| Traffic bench: LED3 R/G/B, LED4 R/G/B | PC0/1/2, PC3/4/5 | common cathode, TCA0 WO0-5 |
| Traffic bench: buttons 0..3 | PA2..PA5 | to GND, internal pull-ups |

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

One `src/apps/<app>.cpp` = one `main()` = two envs (`<app>`,
`<app>-debug`), generated into `apps.ini` by `python tools/gen_apps.py`
(VS Code task "PIO: regen apps", then reload the project). Shared code
goes into `lib/brio/src/`; any header an app includes is compiled and
linked by the LDF, no filter changes.

| App | What it does |
|-----|--------------|
| `blink` | The minimal kernel app: Blinker toggles PF2 on its periodic time event, Supervisor cycles the period (500/250/100 ms) every 3 s by posting a command - no delay loops, CPU in IDLE sleep between events |
| `clock_console` | The console on a runtime-variable clock (`DynamicClock<Boot, Serial>` @ 115200): `CLOCK <div>` switches CLK_PER under the running program; the console surviving = the rebase fan-out works, the 1 Hz LED = the RTC timebase never noticed |
| `console` | Interactive command console @ 460800 (HELP, LED, UPTIME, ERR): SerialPort (RX bytes -> line events), Console (parse/route/reply), Blinker (heartbeat FSM + LED commands via posted events), zero polling |
| `spi_loopback` | SPI stack test: jumper PA4(MOSI) -> PA5(MISO), a full-duplex 8-byte transaction per second through SpiBus + the Spi<0> engine, verdict on the console (no jumper = FAIL 0xFF, by design) |
| `display_id` | Reads the display controller's DCS registers (RDDPM, RDDID, ID4, 0xBF device code) and prints the raw answers: the probe that unmasked the 3.5" module as an ILI9481 |
| `display_fill` | ILI9481 full-screen solid fill cycling red/green/blue (18-bit pixels, CASET/PASET + RAMWR/3C row writes, INVON) |
| `spi_duo` | Two devices, one arbitrated bus: ILI9481 fill (960-byte rows @ 6 MHz) + XPT2046 touch polling (3-byte conversions @ 1.5 MHz) through the same SpiBus; touch steers the fill palette, per-request clock switching |
| `spi_paint` | Touch painting on the ILI9481 through the arbitrated bus |
| `dac_adc` | Two buses, one signal: MCP47CVB22 VOUT0 (I2C) into the MCP3550 (SPI) - a 9-step ramp, each step written and read back over I2C (write-then-read, repeated START) and measured by the ADC via two SPI requests (trigger, then read 200 ms later, busy-frame retry). ADC code = DAC code * 512 within a few LSB |
| `mcp_diag` | MCP3550 behaviour probe, bit-banged on PB0/PA6/PA5 with `delay_us`, one experiment per console key, no kernel: t_conv, CS toggling, early clocks, MISO net, RDY trace |
| `i2c_scan` | I2C stack test: address sweep 0x08..0x77 every 2 s on TWI0 through I2cBus + the Twi<0> engine, ACKs printed as found (expected: 0x60) |
| `traffic0` | The over-commented AO learning testbed: 4 buttons -> 4 RGB lamps, one AO per role, publish for button facts |
| `traffic1` | The traffic light FSM: timed phases via one re-armed time event, a remembered pedestrian call |
| `traffic2` | traffic1 with PWM lamps (TcaPwm split mode, colour palette): the actuator changes, the AOs do not |
