# ============================================================================
#  avr-mcus.cmake - board type -> avr-gcc mcu name: the one documented home
#  of the mapping. cli/bench/common.py's BOARD_TYPES mirrors these short
#  names by hand - keep both in sync. The memories, per die:
#    AVR128DA / AVR128DB   128 KB flash, 16 KB SRAM, 512 B EEPROM (avrxmega4)
#    AVR64EA               64 KB flash, 6 KB SRAM, 512 B EEPROM (avrxmega2,
#                          FLMAP: the upper 32 KB is where .rodata is read)
#  Nothing in this build checks flash/RAM size: avr_add_app() reports the
#  usage after every link (`avr-size -A -d`) and `brio fit avrdx` reads each
#  image against its linker script's regions.
# ============================================================================

set(AVR_MCU_db28 avr128db28)  # 28-pin
set(AVR_MCU_db32 avr128db32)  # 32-pin
set(AVR_MCU_db48 avr128db48)  # 48-pin (the DB bench board)
set(AVR_MCU_da48 avr128da48)  # 48-pin DA (the REV 1.2 board with an AVR128DA48)
set(AVR_MCU_ea28 avr64ea28)   # 28-pin EA (no board: the package's name)
set(AVR_MCU_ea32 avr64ea32)   # 32-pin EA (no board: the package's name)
set(AVR_MCU_ea48 avr64ea48)   # 48-pin EA (the REV 1.2 board with an AVR64EA48)

set(AVR_KNOWN_BOARDS db28 db32 db48 da48 ea28 ea32 ea48)

# avr_short_board(<mcu> <out_var>) - inverse lookup, "avr128db48" -> "db48".
# Used by the app-discovery loop to test an app's board allow-list against
# the currently configured AVR_MCU.
function(avr_short_board mcu out_var)
    foreach(board IN LISTS AVR_KNOWN_BOARDS)
        if(AVR_MCU_${board} STREQUAL mcu)
            set(${out_var} "${board}" PARENT_SCOPE)
            return()
        endif()
    endforeach()
    message(FATAL_ERROR "avr_short_board: unknown AVR_MCU '${mcu}' (known: ${AVR_KNOWN_BOARDS})")
endfunction()
