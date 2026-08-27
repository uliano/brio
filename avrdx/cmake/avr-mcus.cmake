# ============================================================================
#  avr-mcus.cmake - package -> avr-gcc mcu name, replacing the deleted
#  boards/AVR128DB{28,32,48}.json PlatformIO board manifests. All three
#  packages: 128 KB flash, 16 KB SRAM (microchip.com/en-us/product/AVR128DBxx).
#  Nothing in this build programmatically checks flash/RAM size - avr_add_app()
#  already reports real usage after every link via `avr-size -A -d`; this file
#  is the one documented home for the package->mcu mapping. tools/bench.py's
#  MCU_OF_BOARD dict mirrors these short names by hand - keep both in sync.
# ============================================================================

set(AVR_MCU_db28 avr128db28)  # 28-pin
set(AVR_MCU_db32 avr128db32)  # 32-pin
set(AVR_MCU_db48 avr128db48)  # 48-pin (the bench board)

set(AVR_KNOWN_BOARDS db28 db32 db48)

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
