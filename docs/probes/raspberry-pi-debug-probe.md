# Raspberry Pi Debug Probe

Raspberry Pi's own probe: an RP2040 running the `debugprobe` firmware
(CMSIS-DAP v2, FW 2.0.0 on the ones here) in a case, with a UART
bridge - probe and console in ONE USB device with a real serial, so
the bench manifest addresses both by it: the console as
`/dev/serial/by-id/usb-Raspberry_Pi_Debug_Probe__CMSIS-DAP__<serial>-if01`,
the probe as `adapter serial <serial>`. The same firmware runs on
any RP2040 board (SWD on GP2/GP3, the UART on GP4/GP5).

- **Two JST-SH ports and one cable colour code**, read from the
  probe's side: port **D** carries SWD - orange SWCLK (out of the
  probe), black GND, yellow SWDIO; port **U** carries the UART -
  orange TX (out of the probe), black GND, yellow RX (into the probe).
  So onto a target: D's yellow on SWDIO, orange on SWCLK; U's orange
  on the target's RX, yellow on the target's TX - the one crossing.
  A JST-JST cable fits a Pico H's debug connector straight; the
  JST-to-three-dupont cable fits any header.
- **3.3 V fixed, no VTref, no target-power sense**: it never goes on
  a target above 3.3 V (an RP2040's pins are not 5 V tolerant - the
  SAM C21 board jumpered at 5 V is out), and an unpowered target
  looks exactly like a broken wire ("cannot read the DP"): check the
  target's own USB first.
- `brio flash <board> <app>` runs OpenOCD with `interface/cmsis-dap.cfg`
  + `cmsis_dap_backend usb_bulk` (the probe has no HID interface;
  the Atmel-ICE's `hid` is the manifest's default, this probe's entry
  says `"backend": "usb_bulk"`) + the board type's target script +
  `program <app>.elf verify`, `reset run` and the DHCSR write that
  clears C_DEBUGEN.
- **Multidrop SWD** (an RP2040's two debug ports) needs the probe's
  DAP_SWD_Sequence command (CMSIS-DAP 1.20 / 2.00): this firmware has
  it; an older CMSIS-DAP probe fails the select with "CMSIS-DAP
  command SWD_Sequence failed".
- **The flash chip decides the OpenOCD**: the 0.12.0 release's rp2040
  flash driver programs a Pico's Winbond W25Q16JV (measured) and
  refuses a WeAct board's Zetta ZD25Q16 ("Unknown flash device (ID
  0x001560ba)"); a build from OpenOCD's git identifies both - so a
  programmer entry may name its own `"openocd"` and the shared one
  stays the release ([../rp2040/README.md](../rp2040/README.md)).
- **udev**: no rule for the Raspberry Pi vendor id (2e8a) ships with
  OpenOCD's contrib rules or probe-rs's; one line by vendor id
  (`SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", MODE="0660",
  GROUP="plugdev", TAG+="uaccess"`, and the tty world-readable) is
  what makes the probe and its console usable without root.
- **The UART bridge carries 3 Mbaud byte-exact**, host to board
  (measured with `brio stress` against the RP2040 serial suite: 115200
  to 3 Mbaud, 142 KB at the top rung with no error and no overrun on
  the board), so `brio stress` sets no ceiling on this port and every
  rung is pumped. **It sends 8N1 whatever parity the host asks for**:
  a leg announced at 8E1 arrives as clean 8N1 frames (every byte
  accepted, no frame error on the board), so a parity- or
  frame-error test through this bridge measures the probe and not
  the target - such legs belong on a wire between two boards.
- **Core 0 alone, always** (`set USE_CORE 0` before the target
  script, in `brio flash`, the `-upload` target and the debug launch):
  OpenOCD's rp2040 script takes both cores as an SMP pair by default,
  halts both to program, resets core 1 a second time and leaves it
  debug-enabled - which halts a two-kernel program's core 1 on its
  first BKPT and freezes the chip's timer under core 0
  ([../rp2040/multicore.md](../rp2040/multicore.md)). A core 1 that
  such a session left halted (DHCSR reads 0x00030003) is released
  once with both cores configured: `reset run`, then `mww 0xE000EDF0
  0xA05F0000` on `rp2040.core1` and on `rp2040.core0`.
- **Debugging**: cortex-debug over the same OpenOCD; not yet exercised
  on this desk.

Not yet driven by brio through this probe: any target but the RP2040
(it would drive the SAM C21 at 3.3 V and the STM32G0 Nucleos as any
CMSIS-DAP probe would).
