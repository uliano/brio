# ST-LINK

The ST-LINK/V2.1 every Nucleo and the 32F469IDISCOVERY carry on board,
the V2-B of the STM32F429I-DISC1, and the standalone STLINK-V3 (its own
UART bridge as the console): probe and console in ONE USB device with a real serial,
so the bench manifest addresses both by it - the console as
`/dev/serial/by-id/...<serial>-if02`, the probe as `adapter serial
<serial>`.

- `brio flash <board> <app>` runs OpenOCD (`/sw/openocd/bin/openocd`,
  the 0.12.0 release) with `interface/stlink.cfg` + the family's target
  script - `target/stm32g0x.cfg` (the stm32l4x flash driver underneath,
  which serves the G0), `target/stm32f4x.cfg` (the stm32f2x driver,
  which identifies the part and its flash size itself) - + `program
  <app>.elf verify reset exit`.
- **OpenOCD's `stm32g0x.cfg` writes DBGMCU_CR.DBG_STOP at every
  examination**, which keeps HCLK and SysTick running inside a Stop and
  survives every reset but a power-on - a board that was ever examined
  does not really stop. `brio flash` ends every G0 flash with a
  reset-halt, a clear of DBGMCU_CR and a resume, and
  `Pwr::debug_in_stop()` reads the bit for a suite that judges the
  state it finds ([../stm32g0/pwr.md](../stm32g0/pwr.md)).
- **SWD memory reads through the HLA transport are unreliable while
  the core sleeps in WFI** (registers that cannot be zero read zero):
  halt first, read, resume.
- **The debug port can go silent, and only unplugging the board's USB
  cable brings it back.** The symptom: the probe's own status 5, "no
  device connected", against a target that is provably alive (its
  firmware sees the probe's SWCLK/SWDIO edges arrive), through USB
  resets and hub-port power cycles that reset the part. The state
  lives in the ST-LINK half of the board, which a hub port's switch
  does not power down; a replug restores it at once. Rule: a probe
  reporting "no device connected" against a live target is replugged
  as a whole before any other diagnosis.
- **The mass-storage flasher is the fallback** (`stlink_msd` in the
  manifest, with the drive's volume label, e.g. `NODE_G031K8`): the
  `.bin` dropped on the drive is programmed in a few seconds, a bad
  file draws a `FAIL.TXT`, a good image leaves none and its banner is
  the proof it runs. Under it nothing can be halted, read over SWD or
  reset from the host, and DBGMCU_CR is never written.
- **The VCP's ceiling is 921600 baud** (measured; a CH340 carries
  3 Mbaud), which `brio stress` knows.
- **Debugging**: cortex-debug over OpenOCD, the launch entry in
  `.vscode/launch.json` ([../stm32g0/README.md](../stm32g0/README.md)).

- **The STLINK-V3 as a standalone probe** (firmware V3J16M9B5S1, on the
  black pill's four-pin SWD header): the same `interface/stlink.cfg`,
  attach without NRST works on a firmware that leaves the debug pads
  alone; its UART bridge is a second CDC port under the same serial and
  serves as the board's console - its RX to the target's TX. The
  connector's TX/RX labels proved ambiguous on the desk: no banner at
  boot means swap the two wires.
