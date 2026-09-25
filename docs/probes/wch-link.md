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
- **The CH549 kind** - the probe a WeAct-branded WCH-Link is, its
  banner `WCH-Link-CH549 mode:RV version 2.12`, the same `1a86:8010` -
  attaches a CH32V303 over the two-wire port and programs it through
  the fork at that firmware. Its serial bridge is not the LinkE's in
  two ways. It refuses a CDC break (the ioctl answers EOPNOTSUPP). And
  it forwards the target's bytes in BLOCKS of 128, a partial block some
  two milliseconds after the line goes idle - measured: a 495-byte
  answer arriving as 128, 128, 128 and 111 bytes at 11.4, 22.5, 33.6
  and 45.5 ms after the key that asked for it - which is harmless
  until the target talks while nobody has the port open: once the port
  has been opened and closed since the probe enumerated, a burst sent
  into the closed port leaves its last bytes in the probe (41 of them,
  both times measured), and from then on EVERY later burst arrives that
  many bytes late, its own tail coming out only when more bytes push
  it. A suite flashed and then run can therefore hold its `ALL:` line
  behind a boot banner nobody read. And CLOSING THE PORT IN THE MIDDLE
  OF A BURST does worse: from then on the bridge serves every later
  burst with its bytes SHUFFLED - a 128-byte block's tail standing in
  for another's, later text appearing a block early - the same way at
  every attempt (measured: the suite's help text, 1710 bytes, clean
  after a fresh enumeration, interleaved after one open-write-close of
  30 ms, and interleaved the same at every read after that). The same
  reset of the probe over USB (`USBDEVFS_RESET` on its device node)
  clears both states and drops the held bytes; `brio flash` sends it
  after every programming, and a tool that closes the port on a timer
  rather than on the prompt should send it too before the next capture
  is trusted.
- **A WCH-LinkE cannot be opened by WCH's OpenOCD fork while its serial
  port is held open** by another program (the CH549 kind can), so a
  letter cannot stream on the console while the debug port reads the
  chip, and a console left open blocks an upload.
- **A core in debug mode never sleeps** (QingKe V2 manual 5.1), so
  nothing about the idle path's power is measurable with the probe
  halted on it; and the debugger's `step` does not take pending
  interrupts.
- probe-rs (0.31, in ~/.cargo/bin) also sees the probe and is a useful
  second opinion when the fork refuses: `probe-rs list`, `probe-rs
  info`.
