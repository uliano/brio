# Atmel-ICE

Microchip's probe for the AVR (UPDI) and the SAM (SWD) alike - which
family a given ICE is wired to is a fact about the cable, so the bench
manifest records the probe's USB serial per board.

## On the AVR: UPDI, avrdude

- The cable goes in the ICE's **AVR** port, not the SAM one. Symptom of
  the wrong port: Vtarget reads about 1.7 V and the sign-on answers
  `0xa0`.
- `brio flash <board> <app>` runs avrdude (`/sw/avr/bin/avrdude`) with
  `-c atmelice_updi`, `-P usb:<serial>` when the manifest names one, and
  `-B 500kHz` - the ICE's own default UPDI clock, passed so that every
  EDBG-class probe runs at it (`UPDI_CLOCK` in `cli/bench/flash.py`
  says why).
- **Three erase regimes**, and the option names invite exactly the
  wrong assumption. These parts' flash is rated 1k erase/write cycles
  (DS40002247B 39-7), so which pages an erase touches is a budget
  question:

  | How `brio flash` is invoked | What avrdude does | What survives |
  |---|---|---|
  | default | PAGE-ERASES each page it is about to write | every other page, and the EEPROM regardless of EESAVE |
  | `--erase` | a real chip erase (`-e`) first | nothing: every page, EEPROM included |
  | `-D` (never used here) | NO erase at all: programs into pages as they are | everything - but the image is ANDed into what was there |

  The default is what a reflash costs ~40 page cycles instead of 256,
  and what lets an `NvHeap`'s blocks survive a reflash
  ([../design/nv-heap.md](../design/nv-heap.md)): the tool writes the
  image's pages and leaves the rest alone. `-D` is a trap: safe only
  when the bytes already in the chip are the ones being written, it
  silently corrupts anything else (an avrdude verification mismatch is
  how it shows), which is why `brio flash` never passes it and why an
  `NvHeap` image's build id is derived from the sources rather than
  from the clock. `brio flash --erase` is the way to wipe a heap.
- **The AVR EA needs firmware 1.46 or later.** With jtag3-class UPDI
  the ICE runs the NVM sequences itself, and firmware 1.41 does not know
  NVM version 3: it acknowledges every write to an AVR64EA - fuses,
  flash, a chip erase - and loses it (the fuse reads back unchanged, the
  flash reads 0xFF). MPLAB IPE updates the firmware (to 1.46, rel. 11).
- **Fuses** are provisioning, UPDI-only: `brio fuses <board> [name=value
  ...]` reads and writes them (BOOTSIZE, CODESIZE, EESAVE ...) and
  refuses what a board type cannot do - on an AVR EA, before anything is
  read or written: a SYSCFG0 with UPDIPINCFG (bit 4) clear, which this
  probe could not undo (no high-voltage pulse), and the CRC/size
  combinations DS80001048D 2.3.1 and 2.2.4 forbid.
- **Reading a running AVR EA with the debugger**: `pyavrocd -a` attaches
  without a reset (`-d avr64ea48 -i updi -t atmelice -u <serial>`, no
  `-m`: nothing of the fuses is managed) and avr-gdb reads the I/O space
  at 0x800000 + address; a detach lets the program run on. This is how
  a board whose console does not reach the host is still read.
- **Debugging**: PyAvrOCD as the GDB server, launched by the IDE's
  cppdbg entry ([../avrxt/README.md](../avrxt/README.md)). Effectively
  ONE free hardware breakpoint (GDB borrows the second); line
  breakpoints need `-fno-inline`, hence the Debug preset's flags.

## On the SAM: SWD, OpenOCD as a CMSIS-DAP probe

- `interface/cmsis-dap.cfg` + `target/at91samdXX.cfg` (the at91samd
  flash driver probes the geometry from the DSU DID) +
  `program <app>.elf verify reset exit`, with `adapter serial <serial>`.
- **`cmsis_dap_backend hid` on every invocation.** The default usb_bulk
  transport desynchronizes by one packet under sustained traffic behind
  a USB hub (every command gets the previous command's answer,
  programming dies mid-flash) and the state survives soft resets; the
  hid backend was measured no worse anywhere and materially better
  behind a hub. A wedged probe is recovered by two USBDEVFS_RESET
  ioctls five seconds apart, or by a replug; the probes are best on the
  PC's own USB ports, the consoles may stay on a hub.
- **DHCSR.C_DEBUGEN is cleared as the last step of every SAM flash.** A
  core with that bit set HALTS on a BKPT instead of faulting, so
  `panic()`'s closing `break_here()` would park the board in silence;
  attaching a probe sets the bit and a software or watchdog reset does
  not clear it (only a power-on or an external reset does, table
  18-1). With the bit clear the BKPT escalates to HardFault and the
  fault body is the path that runs ([../samc21/reset.md](../samc21/reset.md)).
- The mass-erase / user-row work goes through the same probe: `brio
  fuses <board>` reads and writes the SAM's user row over SWD with a
  whole-row read-modify-write and a read-back verify.
- **Debugging**: cortex-debug over OpenOCD, the launch entry in
  `.vscode/launch.json` ([../samc21/README.md](../samc21/README.md)).
