#!/usr/bin/env bash
# Family compile check for the SAM C21 stratum (part of every driver's
# definition of done, the samc twin of tools/check_family.sh).
#
# Positive: every test/family_samc/*.cpp must COMPILE for every device
# header named below - the E/G/J variants of the 18A, i.e. the three pin
# counts. Only one of them has a board on the desk; the headers are the
# only way the other two get checked at all.
# Negative: every test/family_samc/neg/*.cpp must FAIL to compile for
# each variant named on its "// mcu: <list>" line (what must be refused
# must be refused at compile time).
#
# No CMake coupling on purpose (same as check_family.sh): the compiler is
# called directly, the whole sweep takes seconds and needs no hardware.
#
# Usage: tools/check_samc.sh            all TUs, all variants
#        tools/check_samc.sh pin        only TUs/negatives matching "pin"
set -u
cd "$(dirname "$0")/.."

CXX=/sw/arm-none-eabi/bin/arm-none-eabi-g++
FLAGS="-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -std=gnu++23 -Os \
       -Wall -Wextra -Werror -fno-exceptions -fno-rtti -c \
       -Ilib/brio/src -Ithird_party/samc21-dfp/include -Ithird_party/cmsis-core"
MCUS="samc21e18a samc21g18a samc21j18a"
FILTER="${1:-}"
fail=0

# The device is selected by an ordinary define (samc21j18a ->
# __SAMC21J18A__), which sam.h dispatches on - no device-specs machinery
# and no -mmcu equivalent on this toolchain.
mcu_define() {
    printf '__%s__' "$(echo "$1" | tr '[:lower:]' '[:upper:]')"
}

for tu in test/family_samc/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    line="$(basename "$tu" .cpp):"
    for mcu in $MCUS; do
        if $CXX -D"$(mcu_define "$mcu")" $FLAGS "$tu" -o /dev/null 2>/tmp/check_samc_err; then
            line="$line $mcu"
        else
            line="$line $mcu:FAIL"
            fail=1
            sed "s/^/    /" /tmp/check_samc_err | head -15
        fi
    done
    echo "POS $line"
done

for tu in test/family_samc/neg/*.cpp; do
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

[ "$fail" -eq 0 ] && echo "check_samc: OK" || echo "check_samc: FAILURES"
exit "$fail"
