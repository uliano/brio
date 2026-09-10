# An ATSAMC21J18A board

A self-built board (KiCad, not the Xplained Pro) around an
**ATSAMC21J18A** (64-pin, 256 KB flash, 32 KB SRAM, an 8 KB RWWEE
array), the bench chip of the `samc21/` stratum. Design files: a
repository in preparation.

- **Clock**: the CPU boots and runs on the internal OSC48M at 48 MHz;
  a 24 MHz crystal on PA14/PA15 (XIN/XOUT) is what puts a scale on the
  board - OSC48M is half a per cent off nominal, the sign the die's -
  and the suites that want it arm it and hand it back. A 32.768 kHz
  crystal on PA00/PA01 (XOSC32K) is optional on the design.
- **Supply**: jumper-selectable 3.3 V / 5 V; at 5 V the rail reads
  about 5.1 V (located by the supply suite through the comparator's
  VDD scaler against the bandgap). The analog suites locate the supply
  and lose those verdicts at 3.3 V, where the 4.096 V bandgap level
  saturates the converters - a supply fact, not a regression.
- **LED** PB23; **button** PB22 (EXTINT6), which does not pull the pad
  through the internal pull-up.
- **Console**: a CH340 bridge on PB30/PB31 = SERCOM5 PAD0/PAD1
  (function D), 115200 8N1; addressed by `/dev/serial/by-path`. The
  wire carries 3 Mbaud (measured, `serial_speed`).
- **Probe**: SWD on PA30/PA31, an Atmel-ICE as a CMSIS-DAP probe under
  OpenOCD - [../probes/atmel-ice.md](../probes/atmel-ice.md).
- **Identity**: the factory 128-bit die serial, read over SWD and
  recorded in the manifest; `test_samc_debug` checks it.
- Manifest type `c21j`.

Documents: the datasheet and errata by number in
[../samc21/vendor/README.md](../samc21/vendor/README.md); the target's
page [../samc21/README.md](../samc21/README.md).
