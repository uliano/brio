#!/usr/bin/env python3
# ============================================================================
#  bench_boards.py - THE BENCH MANIFEST: which physical board is which.
#
#  Three separate concerns, and this file is the middle one:
#    1. BUILD      - a PlatformIO env per app x board TYPE (apps.ini, written
#                    by tools/gen_apps.py). Never an env per physical board.
#    2. IDENTITY   - this file: the boards actually on the desk, each with its
#                    board type, its console and its programmer.
#    3. ORCHESTRATION - tools/bench.py, which reads this manifest, resolves an
#                    env, flashes and drives the consoles.
#
#  A board's NAME here ("A", "B", ...) is what bench.py commands take; it is a
#  desk position, not a chip. Moving a chip to another position or another USB
#  socket means editing this file - nothing else.
#
#  CONSOLES ARE ADDRESSED BY /dev/serial/by-path. The CH340 bridges on these
#  boards have NO unique USB serial number (the descriptor's iSerial is 0: the
#  bench shows a single /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 for
#  the one attached board, and a second board would collide with it). What IS
#  unique and stable is the USB TOPOLOGY: /dev/serial/by-path names the port
#  chain (controller, hub, port), so a given physical USB socket always yields
#  the same path. The manifest therefore documents the wiring of the desk: this
#  board plugs into that socket. Re-plug a board into a different socket and
#  the manifest must be edited - that is the price of bridges without serials,
#  and it is cheaper than guessing which ttyUSB* came up first.
#  (The kernel exposes two aliases for the same tty, "...-usb-..." and
#  "...-usbv2-..."; either works, the plain "usb-" one is used here.)
#  The consoles are OBSERVABILITY ONLY: firmware is never loaded through them,
#  they carry the suite's verdicts.
#
#  PROGRAMMERS. Two families, both driven by avrdude (/sw/avr/bin/avrdude):
#    - EDBG-class probes over UPDI: {"type": "atmelice_updi"} or
#      {"type": "pickit4_updi"}. These DO have USB serial numbers (the bench
#      Atmel-ICE is J42700049508), but avrdude only needs to be told which one
#      when TWO probes of the same kind are attached: set "serial" and bench.py
#      passes -P usb:<serial>; leave it None and avrdude takes the only one.
#    - SerialUPDI adapters: {"type": "serialupdi", "port": ..., "baud": ...}
#      -> avrdude -c serialupdi -P <port> -b <baud>. Their USB-serial chips are
#      addressed by /dev/serial/by-path for exactly the reason above.
#
#  IDENTITY IN THE CHIP. The boards themselves are indistinguishable (same
#  chip, serial-less CH340), so each carries a label in its USERROW: 32
#  bytes of NVM that survive chip erase, written ONCE per board over UPDI:
#      avrdude -c atmelice_updi -p avr128db48 -P usb:<probe-serial> \
#              -U userrow:w:0x62,0x72,0x69,0x6f,0x2d,0x61,0x00:m   # "brio-a"
#  The bench suites read it back (avrdx/userrow.hpp board_id()) and print
#  it in their banner, so a console names its own board. The "id" field
#  below is the label this desk position is EXPECTED to carry - the human
#  (or a future bench.py check) compares banner against manifest.
#
#  TODAY'S REALITY: ONE AVR128DB48 board is on the desk. Board A holds the
#  only hub socket in use (usb-0:1.1) and the only Atmel-ICE that is plugged
#  in; board B and its probe are unplugged. B's entry is kept because it is
#  the desk position, not the cable: plug it back in and check which socket it
#  landed on before trusting the path below. Verified the only way that can be
#  trusted - reset the chip over UPDI and watch which console prints the boot
#  banner, which names the board by its USERROW id.
# ============================================================================

# The board types known to the build: keys of tools/gen_apps.py's BOARDS.
# A board type other than db48 needs the app to carry a "// pio: boards" line,
# otherwise there is no env to flash (bench.py says so).

BOARDS = {
    "A": {
        # The original bench board: AVR128DB48, 24 MHz crystal on PA0/PA1,
        # CH340 on USART2 ALT1 (PF4/PF5). See docs/bench.md.
        # Office desk mapping. The desk is re-rigged daily, so every pair
        # below is re-verified at session start: the probe by USERROW
        # readback (the id names the board), the console by resetting the
        # chip over UPDI and watching which port emits the boot traffic.
        "board": "db48",
        "id": "brio-a",
        "console": "/dev/serial/by-path/pci-0000:67:00.3-usb-0:1.1:1.0-port0",
        "programmer": {"type": "atmelice_updi", "serial": "J42700051207"},
    },
    "B": {
        # The instrument peer: a second AVR128DB48 board with its own
        # Atmel-ICE. The probes have swapped boards more than once, so
        # trust the USERROW readback and not habit.
        "board": "db48",
        "id": "brio-b",
        "console": "/dev/serial/by-path/pci-0000:67:00.3-usb-0:1.4:1.0-port0",
        "programmer": {"type": "atmelice_updi", "serial": "J42700049508"},
    },
}

# Console speed used when the app's env does not set monitor_speed.
DEFAULT_MONITOR_SPEED = 460800

# The avrdude that talks to AVR-Dx over UPDI (the self-built 8.x; PlatformIO's
# bundled 7.1 is too old). Same binary the upload_command in platformio.ini uses.
AVRDUDE = "/sw/avr/bin/avrdude"
