# ============================================================================
#  probe.cmake - which probe an image's <app>-upload target programs
#  through, and the probe's half of that target's command line.
#
#  Every family's recipe, <family>/cmake/family.cmake, includes this file
#  and calls brio_probe_select() once with the probe kinds it drives; its
#  upload command then takes the probe's arguments from one of the
#  functions below and keeps what is the target's own (the part, the
#  target script, the resets). Nothing else includes it. The smallest use,
#  from a build directory of any family, for a second probe of the
#  family's own kind:
#
#      cmake --preset <preset> -DBRIO_PROBE_SERIAL=<serial>
#
#  and for a board on a probe of another kind, here an AVR on a PICkit 4:
#
#      cmake --preset avr128db48-release -DBRIO_PROBE=pickit4
#
#  THE VARIABLES, cache variables of the build directory, the same in every
#  family:
#    BRIO_PROBE         the probe's kind, one of the names below; empty: the
#                       family's own kind (the first of its row).
#    BRIO_PROBE_SERIAL  the probe's USB serial, which picks one probe when
#                       several of the kind are attached; empty: the only
#                       one attached. `brio list` prints the serials.
#    BRIO_PROBE_PORT    the serial port of a probe that is one (serialupdi,
#                       bmp), which then names the probe; required there,
#                       refused for every other kind.
#    BRIO_PROBE_BAUD    the serial rate of serialupdi; empty: 230400.
#  A probe's serial belongs to the desk, not to the project: it is given
#  on the configure line or in a CMakeUserPresets.json.
#
#  THE KINDS, and what drives each:
#    atmelice    Atmel-ICE: avrdude -c atmelice_updi on the AVRs, OpenOCD
#                interface/cmsis-dap.cfg on the Cortex-M families
#    pickit4     MPLAB PICkit 4: avrdude -c pickit4_updi -B 500kHz
#    pickit5     MPLAB PICkit 5: avrdude -c pickit5_updi -B 500kHz
#    snap        MPLAB Snap: avrdude -c snap_updi -B 500kHz
#    nedbg       the nEDBG of a Curiosity Nano: avrdude -c pkobn_updi -B 500kHz
#    serialupdi  a USB-serial adapter wired for UPDI: avrdude -c serialupdi
#    stlink      ST-LINK: OpenOCD interface/stlink.cfg
#    cmsis-dap   any CMSIS-DAP probe: OpenOCD interface/cmsis-dap.cfg
#    debugprobe  Raspberry Pi Debug Probe: OpenOCD interface/cmsis-dap.cfg
#                on the USB bulk backend
#    jlink       SEGGER J-Link: OpenOCD interface/jlink.cfg over SWD
#    bmp         Black Magic Probe: the toolchain's gdb on the probe's own
#                port, no OpenOCD
#    wchlink     WCH-Link: WCH's OpenOCD fork, whose target script opens
#                the probe itself
#  Which kinds a family drives, its own first:
#    avrxt                          atmelice pickit4 pickit5 snap nedbg serialupdi
#    samc21                         atmelice cmsis-dap jlink bmp
#    stm32g0, stm32f4               stlink cmsis-dap atmelice jlink bmp
#    rp2040                         debugprobe cmsis-dap atmelice bmp
#    rp2350                         debugprobe cmsis-dap
#    ch32v00x, ch32vx03, ch32x035   wchlink
#
#  WHAT WAS RUN ON HARDWARE. A family's own kind gives the command the
#  family's documents describe, and what was verified with it is stated
#  there (docs/<family>/README.md). The PICkit 4 is measured on the bench
#  through `brio flash` (cli/bench/flash.py, the comment above UPDI_CLOCK)
#  and was run through an <app>-upload target on an AVR128DB48, from an
#  application outside the tree: named by kind alone and with its serial,
#  the image written and read back. Every other kind - pickit5, snap,
#  nedbg, serialupdi, cmsis-dap, atmelice on an STM32 or an RP2040, jlink,
#  bmp - and a serial for wchlink through the <app>-upload target is
#  configured, not run: its command was read in a build directory's
#  build.ninja and never run on a board.
# ============================================================================

include_guard(GLOBAL)

set(BRIO_PROBE        "" CACHE STRING "the probe's kind (cmake/probe.cmake lists them); empty: the family's own")
set(BRIO_PROBE_SERIAL "" CACHE STRING "the probe's USB serial; empty: the only one attached")
set(BRIO_PROBE_PORT   "" CACHE STRING "the probe's serial port (serialupdi, bmp)")
set(BRIO_PROBE_BAUD   "" CACHE STRING "serialupdi's rate; empty: 230400")

# THE NAMES THIS FILE DOES NOT READ. A probe's serial given under a
# family's own name - a build directory's cache or a preset that carries
# one - is read by nothing, and the -upload target then programs through
# whichever probe answers first: on a desk with several, another board.
# So any of these names that is set and not empty stops the configure,
# and the message names the variable that is read.
foreach(_brio_probe_name AVR_PROBE_SERIAL SAMC21_PROBE_SERIAL STM32G0_PROBE_SERIAL
                         STM32F4_PROBE_SERIAL RP2040_PROBE_SERIAL RP2350_PROBE_SERIAL)
    if(DEFINED ${_brio_probe_name} AND NOT "${${_brio_probe_name}}" STREQUAL "")
        message(FATAL_ERROR "brio: ${_brio_probe_name}=${${_brio_probe_name}} is read by nothing: "
                            "the probe's serial is BRIO_PROBE_SERIAL "
                            "(-DBRIO_PROBE_SERIAL=${${_brio_probe_name}}; -U${_brio_probe_name} "
                            "removes the entry from a build directory's cache)")
    endif()
endforeach()

# brio_probe_select(DEFAULT <kind> [KINDS <kind>...])
#
# Called once by a family's recipe with the kinds it drives, its own as
# DEFAULT. Sets BRIO_PROBE_KIND in the caller's scope to BRIO_PROBE, or to
# DEFAULT when BRIO_PROBE is empty, and stops the configure on a kind the
# family does not drive and on a variable the kind does not take.
function(brio_probe_select)
    cmake_parse_arguments(PARSE_ARGV 0 A "" "DEFAULT" "KINDS")
    if(NOT A_DEFAULT)
        message(FATAL_ERROR "brio_probe_select: DEFAULT <kind> is required")
    endif()
    set(_kinds ${A_DEFAULT} ${A_KINDS})
    if(BRIO_PROBE STREQUAL "")
        set(_kind ${A_DEFAULT})
    else()
        set(_kind ${BRIO_PROBE})
    endif()
    if(DEFINED BRIO_FAMILY_DIR)
        get_filename_component(_family "${BRIO_FAMILY_DIR}" NAME)
    else()
        set(_family "this family")
    endif()
    if(NOT _kind IN_LIST _kinds)
        list(JOIN _kinds " " _list)
        message(FATAL_ERROR "brio: BRIO_PROBE=${_kind} is not a probe ${_family} drives; "
                            "its kinds are: ${_list} (cmake/probe.cmake says what each is)")
    endif()
    if(_kind STREQUAL "serialupdi" OR _kind STREQUAL "bmp")
        if(BRIO_PROBE_PORT STREQUAL "")
            message(FATAL_ERROR "brio: BRIO_PROBE=${_kind} needs the probe's serial port: "
                                "set BRIO_PROBE_PORT (e.g. -DBRIO_PROBE_PORT=/dev/ttyACM0)")
        endif()
        if(NOT BRIO_PROBE_SERIAL STREQUAL "")
            message(FATAL_ERROR "brio: BRIO_PROBE=${_kind} is named by its port, not by a USB serial: "
                                "leave BRIO_PROBE_SERIAL empty (${BRIO_PROBE_SERIAL}) and set BRIO_PROBE_PORT")
        endif()
    elseif(NOT BRIO_PROBE_PORT STREQUAL "")
        message(FATAL_ERROR "brio: BRIO_PROBE=${_kind} takes no port (BRIO_PROBE_PORT=${BRIO_PROBE_PORT}): "
                            "leave it empty, and name one probe of several with BRIO_PROBE_SERIAL")
    endif()
    if(NOT BRIO_PROBE_BAUD STREQUAL "" AND NOT _kind STREQUAL "serialupdi")
        message(FATAL_ERROR "brio: BRIO_PROBE_BAUD=${BRIO_PROBE_BAUD} is serialupdi's rate and "
                            "BRIO_PROBE=${_kind} takes none: leave BRIO_PROBE_BAUD empty")
    endif()
    set(BRIO_PROBE_KIND ${_kind} PARENT_SCOPE)
endfunction()

# brio_probe_avrdude(<out>)
#
# Sets <out> to the programmer's part of an avrdude command line, a list:
# -c, the UPDI clock and the probe's USB serial or serial port, after
# -p <part> and before the -U operations.
#
# -B 500kHz is the UPDI clock `brio flash` gives every probe on USB
# (UPDI_CLOCK in cli/bench/flash.py, with its measurement). It is the
# Atmel-ICE's own, so that probe's command carries no -B; without it
# avrdude leaves the PICkit 4 at 100 kHz, which writes and verifies
# about two and a half times slower. A SerialUPDI adapter takes its rate
# from BRIO_PROBE_BAUD instead.
function(brio_probe_avrdude out)
    if(BRIO_PROBE_SERIAL STREQUAL "")
        set(_usb -P usb)
    else()
        set(_usb -P usb:${BRIO_PROBE_SERIAL})
    endif()
    if(BRIO_PROBE_KIND STREQUAL "atmelice")
        set(_args -c atmelice_updi ${_usb})
    elseif(BRIO_PROBE_KIND STREQUAL "pickit4")
        set(_args -c pickit4_updi -B 500kHz ${_usb})
    elseif(BRIO_PROBE_KIND STREQUAL "pickit5")
        set(_args -c pickit5_updi -B 500kHz ${_usb})
    elseif(BRIO_PROBE_KIND STREQUAL "snap")
        set(_args -c snap_updi -B 500kHz ${_usb})
    elseif(BRIO_PROBE_KIND STREQUAL "nedbg")
        set(_args -c pkobn_updi -B 500kHz ${_usb})
    elseif(BRIO_PROBE_KIND STREQUAL "serialupdi")
        if(BRIO_PROBE_BAUD STREQUAL "")
            set(_baud 230400)
        else()
            set(_baud ${BRIO_PROBE_BAUD})
        endif()
        set(_args -c serialupdi -P ${BRIO_PROBE_PORT} -b ${_baud})
    else()
        message(FATAL_ERROR "brio_probe_avrdude: BRIO_PROBE_KIND=${BRIO_PROBE_KIND} is not an avrdude programmer")
    endif()
    set(${out} ${_args} PARENT_SCOPE)
endfunction()

# brio_probe_openocd(<out>)
#
# Sets <out> to the interface part of an OpenOCD command line, a list:
# the interface script and its settings, then the probe's USB serial when
# BRIO_PROBE_SERIAL names one. Each -c argument is one element. The
# recipe places it before its target script; for wchlink it is the serial
# alone, since WCH's target script opens the probe itself.
function(brio_probe_openocd out)
    if(BRIO_PROBE_KIND STREQUAL "atmelice" OR BRIO_PROBE_KIND STREQUAL "cmsis-dap")
        set(_args -f interface/cmsis-dap.cfg)
    elseif(BRIO_PROBE_KIND STREQUAL "debugprobe")
        set(_args -f interface/cmsis-dap.cfg -c "cmsis_dap_backend usb_bulk")
    elseif(BRIO_PROBE_KIND STREQUAL "stlink")
        set(_args -f interface/stlink.cfg)
    elseif(BRIO_PROBE_KIND STREQUAL "jlink")
        set(_args -f interface/jlink.cfg -c "transport select swd")
    elseif(BRIO_PROBE_KIND STREQUAL "wchlink")
        set(_args)
    else()
        message(FATAL_ERROR "brio_probe_openocd: BRIO_PROBE_KIND=${BRIO_PROBE_KIND} is not an OpenOCD probe")
    endif()
    if(NOT BRIO_PROBE_SERIAL STREQUAL "")
        list(APPEND _args -c "adapter serial ${BRIO_PROBE_SERIAL}")
    endif()
    set(${out} ${_args} PARENT_SCOPE)
endfunction()

# brio_probe_bmp(<out> <gdb> <elf>)
#
# Sets <out> to the whole command that loads <elf> through a Black Magic
# Probe on BRIO_PROBE_PORT: <gdb> connects to the probe's own gdb server,
# scans the SWD port, attaches to the first target found, loads the image,
# compares every loaded section with the target's memory and ends the
# session with `kill`. <elf> is passed through as given (a generator
# expression).
# This is the sequence the probe's own documentation gives for a scripted
# load. gdb's exit status tells the last command only (measured, gdb
# 17.2): the target fails when `kill` finds nothing attached - no probe on
# the port, no target, a refused attach - and does not fail on a section
# that compare-sections prints as MIS-MATCHED. What gdb prints is the
# verdict there; `brio flash` reads it (cli/bench/flash.py, bmp_flash).
function(brio_probe_bmp out gdb elf)
    set(${out}
        ${gdb} -nx --batch
            -ex "target extended-remote ${BRIO_PROBE_PORT}"
            -ex "monitor swd_scan"
            -ex "attach 1"
            -ex "load"
            -ex "compare-sections"
            -ex "kill"
            ${elf}
        PARENT_SCOPE)
endfunction()
