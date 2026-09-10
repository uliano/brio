# ST-LINK

The ST-LINK/V2.1 every Nucleo carries on board: probe and console in ONE
USB device with a real serial, so the bench manifest addresses both by
it - the console as `/dev/serial/by-id/...<serial>-if02`, the probe as
`adapter serial <serial>`.

- `brio flash <board> <app>` runs OpenOCD (`/sw/openocd/bin/openocd`,
  the 0.12.0 release) with `interface/stlink.cfg` +
  `target/stm32g0x.cfg` (the stm32l4x flash driver underneath, which
  serves the G0) + `program <app>.elf verify reset exit`.
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

Not yet driven by brio: an ST-LINK/V3 as a standalone probe, for a
self-built STM board.
