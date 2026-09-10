# WCH-Link

WCH's own probe for its RISC-V parts, driven by WCH's OpenOCD fork -
mechanism `wch_link` in the manifest. Probe and console in ONE USB
device with a real serial: the manifest addresses the console as
`/dev/serial/by-id/usb-wch.cn_WCH-Link_<serial>-if01`, and the probe
is the only one attached (the fork's adapter selection with two
attached is untested).

- The transport is **SDI**, WCH's own debug interface: 1-wire on the
  target's SWDIO pin (PD1 on the CH32V00x), or 2-wire with SWCLK
  added (PB3). The 1-wire mode is the chip's default and is what the
  bench runs. In RISC-V mode the probe enumerates as `1a86:8010`
  (`8012` is its ARM/DAP mode; the ModeS key switches). The udev rule
  that makes it reachable without root is probe-rs's
  (`69-probe-rs.rules`, plugdev + uaccess), already on this machine.
- `brio flash <board> <app>` runs **WCH's OpenOCD fork**
  (`/sw/wch-openocd/bin/openocd`, their v2.10 out of MounRiver Studio
  2.5.0, on an OpenOCD 0.11.0+dev base) with its own `wch-riscv.cfg`
  (beside the binary, not in a scripts tree) + `program <app>.elf
  verify` + `reset run`. It is a separate program from `/sw/openocd`,
  the release build that drives the Atmel-ICE and the ST-LINKs: the
  fork carries the `wlinke` adapter and the `wch_riscv` target and
  flash driver, and nothing else of brio's bench.
- **The vendor binary is linked against a `libjaylink.so.0` the
  distribution does not carry.** The library travels inside the
  prefix (`lib/`), the real ELF sits in `libexec/`, and `bin/openocd`
  is a six-line launcher that sets `LD_LIBRARY_PATH` - `libexec/` is
  one level under the prefix like `bin/`, so OpenOCD still finds
  `share/openocd/scripts` from `/proc/self/exe`.
- **A probe firmware that predates the part refuses it, and the
  refusal reads exactly like a wiring fault.** WCH's own WCH-LinkE at
  firmware 2.10 answered `WCH-Link failed to connect with riscvchip`
  against a CH32V006 through every wire, speed and power cycle; a
  Muse Lab WCH-LinkE 1.0 at firmware **2.16** attached at once on the
  same wires. The LINK's firmware is what identifies the target part,
  so an older probe does not know 2025 silicon. Rule: on a WCH
  refusal, read the version in the banner the fork prints
  (`WCH-LinkE  mode:RV version N`) and compare it with a probe known
  to work with the part BEFORE re-checking pins. MounRiver ships
  `libmcuupdate.so`, so the older probe can be brought up to date
  rather than retired.
- **A core in debug mode never sleeps** (QingKe V2 manual 5.1), so
  nothing about the idle path's power is measurable with the probe
  halted on it; and the debugger's `step` does not take pending
  interrupts.
- probe-rs (0.31, in ~/.cargo/bin) also sees the probe and is a useful
  second opinion when the fork refuses: `probe-rs list`, `probe-rs
  info`.
