#!/usr/bin/env python3
# ============================================================================
#  bench_boards.py - THE BENCH MANIFEST: which physical board is which.
#
#  Three separate concerns, and this file is the middle one:
#    1. BUILD      - a CMake target per app x board TYPE, auto-discovered
#                    from each app's own "// build: boards = ..." header
#                    comment (CMakeLists.txt). Never a target per physical
#                    board.
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
#  PROGRAMMERS, first cut: WHICH ARCHITECTURE. The board's "board" type is
#  the only statement of what chip sits at a desk position, and bench.py
#  derives from it which project builds for it and how firmware gets in:
#  db28/db32/db48 are AVR-Dx written by avrdude over UPDI, c21j is a SAM C21
#  written by OpenOCD over SWD. The desk-position LETTERS stay pure positions
#  - A and B happen to hold AVR boards today and C a SAM one, and that is a
#  fact about the desk, not about the letters.
#
#  SAM boards: {"type": "openocd_cmsisdap", "serial": ...} -> OpenOCD driving
#  the Atmel-ICE as a CMSIS-DAP probe. The ICE is SINGLE-CLIENT, so flashing
#  fails while a debug session holds it. Note that an Atmel-ICE can drive
#  either family: which one a probe is wired to is a fact about the cable,
#  which is why the serial is recorded per desk position and re-checked at
#  session start like everything else here.
#
#  STM32 boards: {"type": "openocd_stlink", "serial": ...} -> OpenOCD driving
#  a Nucleo's ON-BOARD ST-LINK/V2.1 (interface/stlink.cfg). The probe and
#  the console are ONE USB device here - the ST-LINK is a composite of
#  debug + virtual COM port - and it carries a real USB serial, so the
#  console can be addressed by /dev/serial/by-id (stable across sockets,
#  unlike the CH340s) and the probe by the same serial. Single-client too.
#
#  AVR PROGRAMMERS. Two families, both driven by avrdude (/sw/avr/bin/avrdude):
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
#  TODAY'S REALITY (2026-09-02, evening): the Nucleo-G0B1RE joined the
#  desk as position E (a direct USB port, not the hub) and is the board
#  under bring-up; the two SAM boards below stayed as they were -
#  positions C and D, one Atmel-ICE each on the PC's own USB ports
#  (the hub desyncs their bulk transport under load - docs/bench.md),
#  their CH340 consoles behind the hub, the five-wire SPI link fitted.
#  Both AVR boards are unplugged; A and B are kept because an entry is a
#  desk position, not a cable. NOTE that both ICE serials now appear on
#  SAM positions while A's and B's entries still name them - the probes
#  have moved between boards more than once, which is exactly why every
#  pairing here is re-verified at session start rather than remembered:
#  an AVR board by resetting it over UPDI and watching which console
#  prints the USERROW banner, a SAM board by reading its DSU DID and die
#  serial over SWD and by the reset-provokes-banner check on its console.
# ============================================================================

# The board types known to the build: keys of tools/bench.py's
# BOARD_PRESET/MCU_OF_BOARD, mirroring cmake/avr-mcus.cmake. A board type
# other than db48 needs the app to carry a "// build: boards" line, otherwise
# there is no target to flash (bench.py says so).

BOARDS = {
    "E": {
        # The third architecture (2026-09-02): an ST Nucleo-G0B1RE
        # (MB1360) - STM32G0B1RE, Cortex-M0+, 512 KB dual-bank flash,
        # 144 KB SRAM, DBGMCU_IDCODE 0x10016467 (DEV_ID 0x467 = G0B1/G0C1,
        # REV_ID 0x1001 = silicon revision Z in ES0548's table 2), read
        # over SWD before a line of brio ran on it. LD4 on PA5, B1 on
        # PC13, the ST-LINK's virtual COM port on USART2 PA2/PA3 at
        # 115200 (the console apps' declared speed). Target voltage
        # 3.23 V - the whole desk runs at 3.3 V now (the SAM boards'
        # supply jumpers were moved for it: docs/bench.md).
        #
        # IDENTITY: the 96-bit unique device ID at 0x1FFF7590 (RM0444
        # 41.1), read over SWD at this position - the STM32's answer to
        # the SAM die serial and the AVR USERROW label. Not yet checked
        # by any suite letter (there is no stm32g0 suite yet):
        #   openocd -f interface/stlink.cfg -c "adapter serial <s>" \
        #           -f target/stm32g0x.cfg -c init -c "mdw 0x1FFF7590 3" -c exit
        "board": "g0b1re",
        "id": None,
        "device_uid": "0042004c-56305016-20333343",
        "console": "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0670FF534871754867182752-if02",
        "programmer": {"type": "openocd_stlink", "serial": "0670FF534871754867182752"},
    },
    "A": {
        # The original bench board: AVR128DB48, 24 MHz crystal on PA0/PA1,
        # CH340 on USART2 ALT1 (PF4/PF5). See docs/bench.md.
        # Office desk mapping. The desk is re-rigged daily, so every pair
        # below is re-verified at session start: the probe by USERROW
        # readback (the id names the board), the console by resetting the
        # chip over UPDI and watching which port emits the boot traffic.
        "board": "db48",
        "id": "brio-a",
        # Re-rigged 2026-08-31 (the energy experiment's office bench):
        # identity confirmed by USERROW readback (brio-a) through ICE
        # ...51207, console identified by its clock_console banner
        # answering on this port.
        "console": "/dev/serial/by-path/pci-0000:67:00.0-usb-0:1.2:1.0-port0",
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
    "C": {
        # The SAM C21 board: the user's C21J rev 1.1 (ATSAMC21J18A, silicon
        # rev F), console CH340 on PB30/PB31 = SERCOM5 PAD0/PAD1 at 115200,
        # SWD on PA30/PA31. See docs/samc/README.md.
        #
        # IDENTITY COMES FREE ON THIS FAMILY. Where an AVR-Dx board has to be
        # LABELLED by hand (a string written once into its USERROW, because
        # the CH340s carry no serial), this die carries a factory-programmed
        # 128-bit serial number in the NVM software calibration area
        # (DS60001479M 11.5 / 27.x: word 0 at 0x0080A00C, words 1..3 at
        # 0x0080A040..48) that no chip erase can touch. The value below was
        # read over SWD from the board at this position, together with DSU
        # DID 0x11010500 (DEVSEL 0x00 = C21J18A, revision 5 = rev F).
        #
        # IT IS CHECKED ON THE BOARD: test_samc_debug letter d reads the DID
        # through samc/dsu.hpp and the four serial words through
        # samc/nvm.hpp's DeviceSerial, prints them in exactly the format
        # below, and verdicts them against these two constants. bench.py
        # still does not compare them itself (unlike the AVR "id", which a
        # banner carries), so this remains a by-hand check - just one that
        # now has a suite letter instead of an openocd incantation. The SWD
        # readback stays the fallback for a board with no firmware on it:
        #   openocd -f interface/cmsis-dap.cfg -c "adapter serial <s>" \
        #           -f target/at91samdXX.cfg \
        #           -c "init" -c "reset halt" -c "mdw 0x0080A00C" \
        #           -c "mdw 0x0080A040 3" -c "reset run" -c "exit"
        "board": "c21j",
        "id": None,
        "die_serial": "f9e78960-51574841-59202020-ff160321",
        # Re-plugged 2026-09-02: a USB hub entered the chain (the
        # alternative was unpowering the desk), so every by-path below
        # it changed - the console now sits behind the hub at 1.1.2.
        # The stale-match lesson of the 2026-08-31 re-plug stands: the
        # pairing is re-verified by resetting the chip over SWD and
        # watching this port emit the firmware's banner.
        "console": "/dev/serial/by-path/pci-0000:67:00.0-usb-0:1.1.2:1.0-port0",
        "programmer": {"type": "openocd_cmsisdap", "serial": "J42700049508"},
    },
    "D": {
        # The second SAM C21 board (2026-09-02): same C21J rev 1.1 design
        # as position C - ATSAMC21J18A, DSU DID 0x11010500 (DEVSEL 0x00 =
        # C21J18A, revision 5 = rev F), console CH340 on PB30/PB31 =
        # SERCOM5 at 115200, SWD on PA30/PA31. Driven by the Atmel-ICE
        # that used to sit on position A.
        #
        # UNLIKE BOARD C, this board carries a 32.768 kHz CRYSTAL on
        # PA00/PA01 (XOSC32K) - so far unexercised anywhere in this
        # stratum (every 32 kHz measurement to date used the internal
        # RCs). A future pass gets a real 32 kHz reference from it:
        # XOSC32K bring-up, RTC on a crystal, GCLK_SERCOM_SLOW for the
        # I2C SMBus time-outs, watchdog/OSCULP32K cross-checks.
        #
        # The die serial below was read over SWD at this position (word 0
        # at 0x0080A00C, words 1..3 at 0x0080A040..48 - the same factory
        # identity mechanism as C's, untouchable by chip erase). The
        # console pairing was verified causally: a reset issued through
        # THIS probe provoked traffic on THIS port (the firmware it
        # shipped with speaks an unknown baud; the first brio flash
        # replaces it and the banner check becomes the usual one).
        "board": "c21j",
        "id": None,
        "die_serial": "3a39fd67-51574841-59202020-ff160311",
        "console": "/dev/serial/by-path/pci-0000:67:00.0-usb-0:1.1.1:1.0-port0",
        "programmer": {"type": "openocd_cmsisdap", "serial": "J42700051207"},
    },
}

# Console speed used when the app does not set monitor_speed.
DEFAULT_MONITOR_SPEED = 460800

# The avrdude that talks to AVR-Dx over UPDI (the self-built 8.x - any
# system-packaged one is likely too old for AVR Dx). Same binary
# avrdx/CMakeLists.txt's avr_add_app() uses for the <app>-upload targets.
AVRDUDE = "/sw/avr/bin/avrdude"

# The OpenOCD that talks to SAM over SWD: the oss-cad-suite build, which
# drives the Atmel-ICE as a CMSIS-DAP probe flawlessly where others do not.
# Same binary samc/CMakeLists.txt's SAMC_OPENOCD defaults to.
OPENOCD = "/sw/oss-cad-suite/bin/openocd"
