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
#  TODAY'S REALITY IS DEGENERATE: one board, one probe, one console. Board "B"
#  below is commented out - it is the shape the second board takes when it
#  arrives, not a claim that it exists.
# ============================================================================

# The board types known to the build: keys of tools/gen_apps.py's BOARDS.
# A board type other than db48 needs the app to carry a "// pio: boards" line,
# otherwise there is no env to flash (bench.py says so).

BOARDS = {
    "A": {
        # The original bench board: AVR128DB48, 24 MHz crystal on PA0/PA1,
        # CH340 on USART2 ALT1 (PF4/PF5). See docs/bench.md.
        "board": "db48",
        "console": "/dev/serial/by-path/pci-0000:67:00.0-usb-0:2:1.0-port0",
        "programmer": {"type": "atmelice_updi", "serial": None},
    },
    # "B": {
    #     # The instrument peer, from 2026-08-22: a 28-pin part driven by a
    #     # SerialUPDI adapter, its own CH340 console on another USB socket.
    #     "board": "db28",
    #     "console": "/dev/serial/by-path/<fill in from bench.py list>",
    #     "programmer": {
    #         "type": "serialupdi",
    #         "port": "/dev/serial/by-path/<fill in from bench.py list>",
    #         "baud": 230400,
    #     },
    # },
}

# Console speed used when the app's env does not set monitor_speed.
DEFAULT_MONITOR_SPEED = 460800

# The avrdude that talks to AVR-Dx over UPDI (the self-built 8.x; PlatformIO's
# bundled 7.1 is too old). Same binary the upload_command in platformio.ini uses.
AVRDUDE = "/sw/avr/bin/avrdude"
