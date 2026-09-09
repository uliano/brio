# display - an ILI9481 panel and an XPT2046 touch controller on one SPI bus

A breadboard experiment, ASSEMBLED ONCE AND DISMANTLED. Nothing of it
is wired on any bench today. The four apps are kept because they are
brio's first arbitrated-bus application (two devices, one `SpiBus`,
per-request clock and mode) and because the panel is a candidate for
the first portable example when it is cabled again. This directory is
self-contained and nothing under docs/ references it.

## What it was

The 3.5" red module **HST035003-A**: controller **ILI9481** (320x480,
SPI = 18-bit pixels only, the panel needs INVON), an **XPT2046**
resistive touch controller on board (U2; U1 is the LDO), backlight tied
high. An older 2.4" module that stayed white through every protocol
despite verified signals is parked as defective-suspect.

- `display_id` reads the controller's DCS registers (RDDPM, RDDID, ID4,
  the 0xBF device code) and prints the raw answers - the probe that
  identified the controller.
- `display_fill` cycles a full-screen solid fill red/green/blue (CASET/
  PASET + RAMWR/3C row writes, INVON).
- `spi_duo` is the point: ILI9481 fills (960-byte rows at 6 MHz) and
  XPT2046 touch polling (3-byte conversions at 1.5 MHz) through the
  same `SpiBus`, the touch steering the fill palette, the clock switched
  per request.
- `spi_paint` paints on the panel with the pen through the same bus.

## The wiring it needed (AVR128DB48, rail at 3.3 V)

| Signal | Pin |
|--------|-----|
| SPI0 MOSI / MISO / SCK (DEFAULT route) | PA4 / PA5 / PA6 |
| Display CS / RS(DC) / RST | PD0 / PD1 / PD2 |
| SD_CS (module, unused) | PD4 |
| T_CS (touch) | PD5, PEN unused |

The module has no level shifter, so the desk had to run at 3.3 V; the
same DEFAULT route at 5 V would have destroyed it, which is why the
bench suites use SPI0 ALT1 (PE0..PE3) instead.

## Building

Discovered by the avrdx project like any other app
(`experiments/*/avrdx/*.cpp`): `cmake --build --preset
avr128db48-release --target spi_duo` (or the other three).
