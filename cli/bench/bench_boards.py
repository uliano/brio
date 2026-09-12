#!/usr/bin/env python3
# ============================================================================
#  bench_boards.py - THE BENCH MANIFEST: which physical board is which.
#
#  This is the EXAMPLE manifest, one entry per board type the build knows.
#  Copy it to private/bench_boards.py, keep the entries for the boards on
#  your desk, and fill in their consoles and probes: bin/brio loads
#  private/bench_boards.py when it exists and this file otherwise, and
#  nothing imports either by name (cli/bench/manifest.py).
#
#  Three separate concerns, and this file is the middle one:
#    1. BUILD         - a CMake target per app x board TYPE, auto-discovered
#                       from each app's own "// build: boards = ..." header
#                       comment. Never a target per physical board.
#    2. IDENTITY      - this file: the boards actually on the desk, each with
#                       its board type, its console and its programmer.
#    3. ORCHESTRATION - bin/brio, which reads this manifest, resolves a
#                       target, flashes and drives the consoles.
#
#  A board's NAME here is what brio commands take (`brio flash A blink`);
#  it is a desk position, not a chip. Moving a chip to another position or
#  another USB socket means editing this file - nothing else.
#
#  BOARD TYPES (the keys of cli/bench/common.py's BOARD_TYPES) decide which
#  project builds for a board and how firmware gets in:
#      db28 db32 db48   AVR DA/DB, written by avrdude over UPDI
#      c21j             SAM C21, written by OpenOCD over SWD (CMSIS-DAP)
#      g0b1re g071rb g031k8   STM32G0 Nucleos, written by OpenOCD over SWD
#                       through the board's own ST-LINK
#      v006k8 v003f4    CH32V006K8 / CH32V003F4, written by WCH's OpenOCD
#                       fork through a WCH-Link (SDI, WCH's own debug transport)
#      pico picow weact2040   RP2040 boards (a Raspberry Pi Pico / Pico H, a
#                       Pico W, a WeAct board), written by OpenOCD over
#                       multidrop SWD through a CMSIS-DAP probe - the
#                       Raspberry Pi Debug Probe
#
#  CONSOLES. A board with a serial-less USB bridge (a CH340 has no unique
#  serial: two of them collide in /dev/serial/by-id) is addressed by
#  /dev/serial/by-path, i.e. by the USB SOCKET it is plugged into; a
#  board whose bridge carries a real serial (a Nucleo's ST-LINK) by
#  /dev/serial/by-id, stable across sockets. Consoles are observability
#  only: firmware never goes in through them.
#
#  PROGRAMMERS, by "type":
#    atmelice_updi / pickit4_updi   an EDBG-class probe over UPDI (avrdude);
#                       "serial" only when two probes of the kind are attached
#    serialupdi         {"type": "serialupdi", "port": ..., "baud": ...}
#    openocd_cmsisdap   OpenOCD driving an Atmel-ICE (or any CMSIS-DAP probe)
#                       over SWD; "serial" = the probe's USB serial; "backend"
#                       = "hid" (the default, the ICE) or "usb_bulk" (a
#                       CMSIS-DAP v2 probe, the Raspberry Pi Debug Probe);
#                       "openocd" = this probe's own OpenOCD binary when the
#                       shared OPENOCD below will not do (an RP2040 board
#                       whose flash chip the 0.12.0 release does not know)
#    openocd_stlink     OpenOCD driving an ST-LINK; "serial" = its USB serial
#                       (the same device carries the Nucleo's console)
#    stlink_msd         the ST-LINK's mass-storage flasher, the fallback when
#                       its debug port does not answer: {"type": "stlink_msd",
#                       "label": "NODE_G031K8"} (the drive's volume label)
#    wch_link           a WCH-Link (RISC-V mode) driven by WCH's OpenOCD fork
#                       (WCH_OPENOCD below); the same device carries a serial
#                       port, which is the CH32 board's console when its
#                       USART is wired to the probe's TX/RX pins
#
#  IDENTITY IN THE CHIP. An "id" is the label an AVR board carries in its
#  USERROW (avrdx/userrow.hpp: written once over UPDI, printed by every
#  suite's banner); a SAM board is told by its factory die serial and an
#  STM32 by its 96-bit unique device ID, both read over SWD - recorded
#  here so a suite can compare the chip in hand against the manifest.
# ============================================================================

BOARDS = {
    "A": {
        # An AVR128DB48 board: console on a CH340 (by-path), an Atmel-ICE
        # over UPDI. "serial" is needed only with two such probes attached.
        "board": "db48",
        "id": "brio-a",
        "console": "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.1:1.0-port0",
        "programmer": {"type": "atmelice_updi", "serial": None},
    },
    "C": {
        # An ATSAMC21J18A board: console on a CH340, an Atmel-ICE as a
        # CMSIS-DAP probe over SWD.
        "board": "c21j",
        "id": None,
        "die_serial": None,
        "console": "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.2:1.0-port0",
        "programmer": {"type": "openocd_cmsisdap", "serial": "J000000000000"},
    },
    "E": {
        # A Nucleo-G0B1RE: the on-board ST-LINK is both the probe and the
        # console, one USB serial for both.
        "board": "g0b1re",
        "id": None,
        "device_uid": None,
        "console": "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_000000000000000000000000-if02",
        "programmer": {"type": "openocd_stlink", "serial": "000000000000000000000000"},
    },
    "F": {
        "board": "g071rb",
        "id": None,
        "device_uid": None,
        "console": "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_111111111111111111111111-if02",
        "programmer": {"type": "openocd_stlink", "serial": "111111111111111111111111"},
    },
    "G": {
        # A Nucleo-32; the same ST-LINK arrangement, and the mass-storage
        # flasher as the fallback: {"type": "stlink_msd", "label": "NODE_G031K8"}.
        "board": "g031k8",
        "id": None,
        "device_uid": None,
        "console": "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_222222222222222222222222-if02",
        "programmer": {"type": "openocd_stlink", "serial": "222222222222222222222222"},
    },
    "K": {
        # A WeAct RP2040 board on a Raspberry Pi Debug Probe: port D on
        # the board's SWD header, port U crossed onto GP0/GP1 (UART0),
        # so the probe's own CDC port (interface -if01, a real USB
        # serial) is the console. CMSIS-DAP v2 = the usb_bulk backend.
        "board": "weact2040",
        "id": None,
        "console": "/dev/serial/by-id/usb-Raspberry_Pi_Debug_Probe__CMSIS-DAP__E666666666666666-if01",
        "programmer": {"type": "openocd_cmsisdap", "serial": "E666666666666666",
                       "backend": "usb_bulk"},
    },
    "I": {
        # A CH32V006K8 board on a WCH-Link: the probe's serial pins wired
        # to the board's USART1 (PD5/PD6), so one USB cable carries the
        # debug transport and the console. The WCH-Link carries a real
        # USB serial, so its CDC port (interface -if01) is addressed
        # by-id, stable across sockets.
        "board": "v006k8",
        "id": None,
        "device_uid": None,
        "console": "/dev/serial/by-id/usb-wch.cn_WCH-Link_333333333333-if01",
        "programmer": {"type": "wch_link", "serial": "333333333333"},
    },
}

# The console speed a suite is driven at when its app declares none
# ("// build: monitor_speed = ..." in the app's header).
DEFAULT_MONITOR_SPEED = 460800

# The tools, by absolute path: the self-built ones under /sw
# (docs/avrdx/README.md, docs/samc21/README.md), never a distribution's.
AVRDUDE = "/sw/avr/bin/avrdude"
OPENOCD = "/sw/openocd/bin/openocd"
# WCH's OpenOCD fork for the WCH-Link (docs/ch32v00x/README.md): a
# different program from the one above, kept apart under /sw.
WCH_OPENOCD = "/sw/wch-openocd/bin/openocd"
