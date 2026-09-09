# dac_adc - a DAC written over I2C and read back through an ADC over SPI

A breadboard experiment, ASSEMBLED ONCE AND DISMANTLED. Nothing of it
is wired on any bench today; the two apps here are kept because they
are the first real device clients brio's two bus vocabularies ever had,
and because the facts measured on the parts are worth keeping. This
directory is self-contained and nothing under docs/ references it
(house rule: experiments document themselves).

## What it was

An MCP47CVB22 DAC (I2C, address 0x60, A0 and LAT/HVC to GND so LAT is
transparent) driving its VOUT0 into an MCP3550 22-bit delta-sigma ADC
(SPI). `dac_adc` writes a 9-step ramp over I2C - each step written and
read back in one write-then-read tenure with a repeated START - and
measures each step through the ADC with two SPI requests (a trigger,
then the read 200 ms later, with a busy-frame retry): ADC code = DAC
code x 512 within a few LSB. Both buses are arbitrated by the same
`BusMaster` shape (`I2cBus`, `SpiBus`), which is what the experiment
was for.

`mcp_diag` is the MCP3550 behaviour probe that preceded it: bit-banged
on PB0/PA6/PA5 with `delay_us`, no kernel, one experiment per console
key (t_conv, CS toggling, early clocks, the MISO net, an RDY trace).

## The wiring it needed (AVR128DB48, rail at 3.3 V)

| Signal | Pin |
|--------|-----|
| SPI0 MOSI / MISO / SCK (DEFAULT route) | PA4 / PA5 / PA6 |
| MCP3550 CS | PB0 (`dac_adc`; `mcp_diag` bit-bangs the same pins) |
| MCP3550 SDO/RDY | on MISO, PA5 |
| TWI0 SDA / SCL | PA2 / PA3, 1.5k pull-ups |
| MCP47CVB22 VOUT0 | -> MCP3550 input |

Neither part has a level shifter, so the desk had to run at 3.3 V.

## What the parts taught (the design consequences are in docs/design/spi-bus.md)

MCP3550, on the analyzer: t_conv 81 ms with CS low, ~119 ms from
trigger to RDY when CS is high during the conversion (the result held
for the next CS fall); it latches its SPI mode from the SCK level at
the CS edge (mode 1,1 = SCK high, DS20001950F 5.5), so a host must park
SCK at the device's CPOL before CS falls; CS low >= 8 us (tCSL); and
waking from shutdown it needs a few microseconds of CS setup before
the first SCK - the datasheet only says "an internal power-up delay
must be observed"; measured, SDO drives ~4 us after CS falls, a frame
clocked 1.5 us after CS is lost (0x7FFFFF), 3.5 us is enough, `dac_adc`
uses 10. That measurement is why `SpiHost::Request` carries
`cs_setup_us`. A dry joint on a header pin looks like a protocol fault
(random t_conv, frames decaying to zero): check the wire first.

## Building

Both apps are discovered by the avrdx project like any other
(`experiments/*/avrdx/*.cpp`): `cmake --build --preset
avr128db48-release --target dac_adc` (or `mcp_diag`).
