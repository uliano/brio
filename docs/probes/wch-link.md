# WCH-Link

WCH's own probe for its RISC-V parts, driven by WCH's OpenOCD fork (a
build of OpenOCD WCH ships with its own adapter and target drivers,
"the fork" below): the programmer kind `wch_link` in the manifest.
Probe and console are one
USB device with a real serial; the manifest addresses the console as
`/dev/serial/by-id/usb-wch.cn_WCH-Link_<serial>-if01`, and the probe by
`adapter serial` when the manifest gives one. Two kinds are on the
desk, the WCH-LinkE and the CH549 kind, and they differ in more than the
name: this page says how the fork drives either, and the traps, which
are mostly the serial bridge's. A probe firmware that predates a part
refuses it in words that read like a wiring fault; a LinkE rewrites the
clock tree of the chip it attaches to; the LinkE's bridge transmits at
the wrong rate on an old firmware and drops runs of the host's bytes
above 115200 on a new one; the CH549's bridge holds bytes back and can
shuffle them.

## The transport and the fork

- The transport is **SDI**, WCH's own debug interface: one wire on the
  target's SWDIO pin (PD1 on the CH32V00x), or two wires with SWCLK
  added (PB3 there). The one-wire mode is the CH32V00x's default and is
  what the bench runs on that family; the CH32V203, CH32V303 and
  CH32X035 have the two-wire port. In RISC-V mode the probe enumerates
  as `1a86:8010` (`8012` is its ARM/DAP mode; the ModeS key on the probe
  switches between the two).
  The udev rule that makes it reachable without root is probe-rs's
  (`69-probe-rs.rules`, plugdev + uaccess), already on this machine.
- `brio flash <board> <app>` runs **WCH's OpenOCD fork**
  (`/sw/wch-openocd/bin/openocd`, their v2.10 out of MounRiver Studio
  2.5.0, on an OpenOCD 0.11.0+dev base) with its own `wch-riscv.cfg`
  (beside the binary, not in a scripts tree), `program <app>.elf
  verify`, then `reset halt` and `resume`, which is what starts the
  program on every WCH family (`reset run` leaves a CH32V203 at its
  reset vector, [../ch32vx03/README.md](../ch32vx03/README.md)). It is a
  separate program from `/sw/openocd`, the release build that drives the
  Atmel-ICE and the ST-LINKs: the fork carries the `wlinke` adapter and
  the `wch_riscv` target and flash driver, and nothing else of brio's
  bench.
- **The vendor binary is linked against a `libjaylink.so.0` the
  distribution does not carry.** The library travels inside the prefix
  (`lib/`), the real ELF sits in `libexec/`, and `bin/openocd` is a
  six-line launcher that sets `LD_LIBRARY_PATH`; `libexec/` is one level
  under the prefix like `bin/`, so OpenOCD still finds
  `share/openocd/scripts` from `/proc/self/exe`.
- After every programming `brio flash` resets the probe over USB
  (`USBDEVFS_RESET` on its device node), which stands in for unplugging
  and replugging it; the bridge traps below are why.
- probe-rs (0.31, in `~/.cargo/bin`) also sees the probe and is a
  useful second opinion when the fork refuses: `probe-rs list`,
  `probe-rs info`.

## A firmware that predates the part refuses it

**The refusal reads exactly like a wiring fault.** WCH's own WCH-LinkE
at firmware 2.10 answered `WCH-Link failed to connect with riscvchip`
against a CH32V006 through every wire, speed and power cycle; a Muse Lab
WCH-LinkE 1.0 at firmware 2.16 attached at once on the same wires. The
probe's firmware is what identifies the target part, so an older probe
does not know newer silicon. **The rule: on a WCH refusal, read the
version in the banner the fork prints (`WCH-LinkE  mode:RV version N`)
and compare it with a probe known to work with the part before
re-checking pins.** MounRiver ships `libmcuupdate.so`, so the older
probe can be brought up to date rather than retired.

## An attach through a LinkE is not free

On a CH32V303 running at 144 MHz every fresh attach of the fork through
a LinkE at firmware 2.10 rewrote RCC_CFGR0 (0x00BCC40A read 0x00B4040A
after it: the console's divisor garbled until the program set the tree
again) and set the debug module's low-power word (CSR 0x7C0) to 0x307,
which keeps the clocks running through Sleep, Stop and Standby until
something clears it (measured; the CH549 kind did neither). A LinkE at
firmware 2.16 rewrites RCC_CFGR0 the same way on a CH32V203C8 at
144 MHz: 0x00B4040A read right after `init`, the hart (RISC-V's word
for a core) still running,
PLLMUL's x18 become x15, the core at 120 MHz under divisors computed for
144, so the console's bytes arrive garbled both ways (a 0x55 from the
host read as 0xEB with the noise flag); measured twice, while the CH549
probe on the CH32V303 beside it left 0x00BCC40A. A `reset halt` and
`resume` lets the program set its tree again, which is how the flash
verb ends; any other attach (a register read, a halt) is ended the same
way before a console is trusted. A suite that sleeps is judged on a
2.10 probe only after the low-power word is written back to zero (on
2.16 that word was not read).

**And the 2.10 LinkE's serial bridge transmits a quarter fast**: asked
115200, the probe's TX clocks the target's receiver at 144000 (a 0x55
read back as 0xA9 at the target's 115200 divisor, exact at a divisor for
144000; asked 92160 the target reads 0x55 at 115200), while its RX
decodes 115200 correctly (the target's banner clean at 115200 asked,
garbage at 92160). No one host setting serves both directions, so that
probe is a debug probe alone: a target's console goes through another
bridge, or the firmware is brought up to date first.

## The LinkE at firmware 2.16 carries 921600 out and 115200 in

Measured with `brio stress`, the host end of the serial suites, against
a CH32V006 at 48 MHz on the probe's own bridge (the benchmark
skeleton's letter w, a letter being one command key of a suite's
console: the board judges a sink of the host's xorshift stream, a
pseudo-random sequence, for a second and a half, and sources a quarter
second of its own for the host to judge, at each of eight rates). The
target's bytes reach the host byte-exact at 115200, 230400, 460800 and
921600, every one of 2880, 5760, 11520 and 23040 in order, and arrive
as garbage from 1 Mbaud (1, 1.5 and 2 Mbaud) or not at all (3 Mbaud):
the bridge does not run those rates. The host's bytes reach the target
whole only at 115200 (12096 of 12096); above it every byte that arrives
is the right one, with no bad byte, no framing error and no overrun on
the target, but whole runs of them never arrive, the bridge dropping 2.6
to 5 per cent of a sustained stream in 50 to 140 gaps a second (1314 of
25070 at 230400, 2028 of 48990 at 460800, 2484 of 95680 at 921600, the
first gap after 193 to 513 bytes), and from 1 Mbaud every frame is a
framing error. A burst shorter than the first gap crosses at those rates
(a command does); a sustained stream does not.

## The CH549 kind

The probe a WeAct-branded WCH-Link is, with the banner
`WCH-Link-CH549 mode:RV version 2.12` and the same `1a86:8010`: it
attaches a CH32V303 over the two-wire port and programs it through the
fork at that firmware.
Its serial bridge is not the LinkE's in two ways.

It refuses a CDC break (the ioctl answers EOPNOTSUPP).

And it forwards the target's bytes in blocks of 128, a partial block
some two milliseconds after the line goes idle (measured: a 495-byte
answer arriving as 128, 128, 128 and 111 bytes at 11.4, 22.5, 33.6 and
45.5 ms after the key that asked for it). That is harmless until the
target talks while nobody has the port open: once the port has been
opened and closed since the probe enumerated, a burst sent into the
closed port leaves its last bytes in the probe (41 of them, both times
measured), and from then on every later burst arrives that many bytes
late, its own tail coming out only when more bytes push it. A suite
flashed and then run can therefore hold its `ALL:` line behind a boot
banner nobody read. **Closing the port in the middle of a burst does
worse**: from then on the bridge serves every later burst with its bytes
shuffled, a 128-byte block's tail standing in for another's, later text
appearing a block early, the same way at every attempt (measured: the
suite's help text, 1710 bytes, clean after a fresh enumeration,
interleaved after one open-write-close of 30 ms, and interleaved the
same at every read after that). A burst into the closed port did as
much with no close in the middle, after a programming that did not
reset the probe, the image's boot banner going into the closed port:
on the CH32V203C8T6's probe a suite's 5833-byte answer arrived shuffled
at every capture, in each 64 bytes the last 18 standing in from 192
bytes later, the same at every attempt and under two builds of the
suite; on the CH32V303VCT6's a 645-byte answer arrived whole but led by
the last 100 bytes of the answer before, its own last 100 held
(measured, both probes the CH549 kind at 2.12). Neither target held
anything back: read over the debug port with the console closed, its
transmit ring was empty with the final prompt in its storage, STATR's
TC set and the transmit interrupt disarmed. So a console on this probe
that loses the end of every answer, or interleaves pieces of different
lines, is the bridge's state until a capture after the reset below says
otherwise. The same reset of the probe over USB
(`USBDEVFS_RESET` on its device node) clears both states and drops the
held bytes; `brio flash` sends it after every programming, and a tool
that closes the port on a timer rather than on the prompt should send it
too before the next capture is trusted.

## Two more rules of a session

- **A WCH-LinkE cannot be opened by WCH's OpenOCD fork while its serial
  port is held open** by another program (the CH549 kind can), so a
  suite's letter cannot stream on the console while the debug port
  reads the chip, and a console left open blocks an upload.
- **A core in debug mode never sleeps** (QingKe V2 manual 5.1), so
  nothing about the idle path's power is measurable with the probe
  halted on it; and the debugger's `step` does not take pending
  interrupts.
