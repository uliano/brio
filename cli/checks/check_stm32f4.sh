#!/usr/bin/env bash
# Family compile check for the STM32F4 stratum (part of every driver's
# definition of done, the stm32f4 twin of cli/checks/check_stm32g0.sh).
#
# Positive: every test/family_stm32f4/*.cpp must COMPILE for every device
# header the CMSIS pack ships - ALL TWENTY-THREE, from the F401 to the
# F479. The F429, F446, F411 and F469 are the bench parts; for the
# nineteen headers no board here carries this sweep is the only check
# there is,
# and it is what makes the reserve's presence-keyed derivations a proven
# claim rather than a plausible one - and what proves that a part whose
# frequency ladder the reserve does not know is REFUSED above 16 MHz
# rather than run on a guess.
# Negative: every test/family_stm32f4/neg/*.cpp must FAIL to compile for
# each variant named on its "// mcu: <list>" line (what must be refused
# must be refused at compile time).
#
# No CMake coupling on purpose (same as check_family.sh): the compiler is
# called directly, the whole sweep takes seconds and needs no hardware.
#
# Usage: brio check stm32f4            all TUs, all variants
#        brio check stm32f4 pin        only TUs/negatives matching "pin"
set -u
cd "$(dirname "$0")/../.."

CXX=/sw/arm-none-eabi/bin/arm-none-eabi-g++
# A compiler that is not installed must not pass for a refusal: a
# command that cannot start fails like a TU that failed to compile, so
# every negative would read "refused" and every positive would fail
# for a reason that is not the code's. Say so and stop.
[ -x "$CXX" ] || { echo "$(basename "$0"): the compiler $CXX is not installed - nothing checked" >&2; exit 2; }
FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -std=gnu++23 -Os \
       -Wall -Wextra -Werror -fno-exceptions -fno-rtti -c \
       -Ibrio -Ithird_party/cmsis-device-f4/Include -Ithird_party/cmsis-core"
MCUS="stm32f401xc stm32f401xe stm32f405xx stm32f407xx stm32f410cx stm32f410rx stm32f410tx \
      stm32f411xe stm32f412cx stm32f412rx stm32f412vx stm32f412zx stm32f413xx stm32f415xx \
      stm32f417xx stm32f423xx stm32f427xx stm32f429xx stm32f437xx stm32f439xx stm32f446xx \
      stm32f469xx stm32f479xx"
FILTER="${1:-}"
fail=0

# The device is selected by ST's own define, which stm32f4xx.h dispatches
# on - and the spelling is irregular: STM32F429xx for a whole line,
# STM32F411xE for a size class, STM32F410Cx for a package. The header
# stem's last two characters decide, letter by letter.
mcu_define() {
    local stem="${1#stm32}"
    local body="${stem%??}"
    local tail="${stem: -2}"
    local suffix
    case "$tail" in
        xx) suffix="xx" ;;
        xe) suffix="xE" ;;
        xc) suffix="xC" ;;
        cx) suffix="Cx" ;;
        rx) suffix="Rx" ;;
        tx) suffix="Tx" ;;
        vx) suffix="Vx" ;;
        zx) suffix="Zx" ;;
        *)  suffix="$tail" ;;
    esac
    printf 'STM32%s%s' "$(echo "$body" | tr '[:lower:]' '[:upper:]')" "$suffix"
}

for tu in test/family_stm32f4/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    line="$(basename "$tu" .cpp):"
    for mcu in $MCUS; do
        if $CXX -D"$(mcu_define "$mcu")" $FLAGS "$tu" -o /dev/null 2>/tmp/check_stm32f4_err; then
            line="$line $mcu"
        else
            line="$line $mcu:FAIL"
            fail=1
            sed "s/^/    /" /tmp/check_stm32f4_err | head -15
        fi
    done
    echo "POS $line"
done

for tu in test/family_stm32f4/neg/*.cpp; do
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

[ "$fail" -eq 0 ] && echo "check_stm32f4: OK" || echo "check_stm32f4: FAILURES"
exit "$fail"
