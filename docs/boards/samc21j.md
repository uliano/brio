# An ATSAMC21J18A board

A self-built board (KiCad, not Microchip's Xplained Pro) around an
ATSAMC21J18A (64 pins, 256 KB of flash, 32 KB of SRAM, an 8 KB RWWEE
array), the bench chip of the `samc21/` stratum
([../samc21/README.md](../samc21/README.md)). Its design files are not
in this repository; a repository of their own is in preparation. What
the board offers a brio program is a Cortex-M0+ on its internal
oscillator with a real crystal to measure against, a serial console, an
SWD header for an Atmel-ICE and a factory die serial as its identity.

## The board

- **Clock**: the CPU boots and runs on the internal OSC48M at 48 MHz.
  A 24 MHz crystal on PA14/PA15 (XIN/XOUT) is what puts a scale on the
  board: OSC48M is half a per cent off nominal, the sign the die's, and
  the suites that want the crystal arm it and hand it back. A
  32.768 kHz crystal on PA00/PA01 (XOSC32K) is optional on the design
  and not fitted on the bench board, so no suite may enable XOSC32K
  there.
- **Supply**: jumper-selectable, 3.3 V or 5 V. At 5 V the rail reads
  about 5.1 V (located by the supply suite through the comparator's VDD
  scaler against the bandgap). The analog suites locate the supply and
  lose those verdicts at 3.3 V, where the 4.096 V bandgap level
  saturates the converters: a supply fact, not a regression.
- **LED and button**: the LED on PB23; the button on PB22 (EXTINT6),
  which does not pull the pad through the internal pull-up.
- **Console**: a CH340 bridge on PB30/PB31, which are SERCOM5 PAD0/PAD1
  under function D, at 115200 8N1; addressed by `/dev/serial/by-path`,
  the socket it is plugged into. The wire carries 3 Mbaud (measured).
- **Probe**: SWD on PA30/PA31, an Atmel-ICE driven by OpenOCD as a
  CMSIS-DAP probe ([../probes/atmel-ice.md](../probes/atmel-ice.md)).
- **Identity**: the factory 128-bit die serial, read over SWD and
  recorded in the manifest; `test_samc_debug` checks it.

## Flashing

`brio flash <board> <app>` writes the board through OpenOCD over SWD;
the board type in the manifest is `c21j`, and on it a suite with a
groups line builds as one image per group ([README.md](README.md),
"How a board joins the bench").

## Documents

The datasheet and the errata by number are in
[../samc21/vendor/README.md](../samc21/vendor/README.md); the target's
page is [../samc21/README.md](../samc21/README.md).
