#!/usr/bin/env bash
# Family compile check for the STM32G0 stratum (part of every driver's
# definition of done, the stm32g0 twin of tools/check_samc.sh).
#
# Positive: every test/family_stm32g0/*.cpp must COMPILE for every device
# header named below - the G0B1 (the bench chip, the family's superset),
# the G071 and the G031: the three parts the desk's Nucleo boards carry,
# of which only the first is plugged in; the headers are the only way
# the other two get checked at all.
# Negative: every test/family_stm32g0/neg/*.cpp must FAIL to compile for
# each variant named on its "// mcu: <list>" line (what must be refused
# must be refused at compile time).
#
# No CMake coupling on purpose (same as check_family.sh): the compiler is
# called directly, the whole sweep takes seconds and needs no hardware.
#
# Usage: tools/check_stm32g0.sh            all TUs, all variants
#        tools/check_stm32g0.sh pin        only TUs/negatives matching "pin"
set -u
cd "$(dirname "$0")/.."

CXX=/sw/arm-none-eabi/bin/arm-none-eabi-g++
FLAGS="-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -std=gnu++23 -Os \
       -Wall -Wextra -Werror -fno-exceptions -fno-rtti -c \
       -Ibrio -Ithird_party/cmsis-device-g0/Include -Ithird_party/cmsis-core"
MCUS="stm32g0b1xx stm32g071xx stm32g031xx"
FILTER="${1:-}"
fail=0

# The device is selected by ST's own define (stm32g0b1xx ->
# STM32G0B1xx), which stm32g0xx.h dispatches on - no device-specs machinery
# and no -mmcu equivalent on this toolchain.
mcu_define() {
    printf '%sxx' "$(echo "${1%xx}" | tr '[:lower:]' '[:upper:]')"
}

for tu in test/family_stm32g0/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    line="$(basename "$tu" .cpp):"
    for mcu in $MCUS; do
        if $CXX -D"$(mcu_define "$mcu")" $FLAGS "$tu" -o /dev/null 2>/tmp/check_stm32g0_err; then
            line="$line $mcu"
        else
            line="$line $mcu:FAIL"
            fail=1
            sed "s/^/    /" /tmp/check_stm32g0_err | head -15
        fi
    done
    echo "POS $line"
done

for tu in test/family_stm32g0/neg/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    mcus="$(sed -n 's|^// mcu:||p' "$tu")"
    if [ -z "$mcus" ]; then
        echo "NEG $(basename "$tu"): missing '// mcu:' line"; fail=1; continue
    fi
    line="$(basename "$tu" .cpp):"
    for mcu in $mcus; do
        if $CXX -D"$(mcu_define "$mcu")" $FLAGS "$tu" -o /dev/null 2>/dev/null; then
            line="$line $mcu:COMPILED(BAD)"
            fail=1
        else
            line="$line $mcu:refused"
        fi
    done
    echo "NEG $line"
done

[ "$fail" -eq 0 ] && echo "check_stm32g0: OK" || echo "check_stm32g0: FAILURES"
exit "$fail"
