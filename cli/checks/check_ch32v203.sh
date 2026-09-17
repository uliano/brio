#!/usr/bin/env bash
# Family compile check for the CH32V203 stratum (part of every driver's
# definition of done, the twin of cli/checks/check_ch32v00x.sh).
#
# Positive: every test/family_ch32v203/*.cpp must COMPILE for every part
# in PARTS with the project's own flags - the part's definition
# (CH32V203C8 and the like, what device.hpp asks for its table) - or,
# when the TU carries a "// mcu: <list>" line, for those parts alone: a
# TU that names a USART, a pad or a block some package has not got says
# so there. Nine parts, one ISA: unlike the CH32V00x family every part
# of this one is the same QingKe V4B core, so what varies is the part
# definition and nothing else.
# Every positive is compiled BOTH WAYS the project can build an image:
# with the core's hardware prologue/epilogue (-DBRIO_CH32_HPE=1, the
# CH32V203_HPE option, WCH's interrupt attribute) and without it (gcc's
# own prologue), since pfic.hpp's BRIO_CH32_INTERRUPT is one spelling
# with two expansions and a fixture that binds a vector proves both.
# Negative: every test/family_ch32v203/neg/*.cpp must FAIL to compile
# for each part named on its "// mcu: <list>" line.
#
# No CMake coupling on purpose (same as the other six scripts): the
# compiler is called directly, the sweep takes seconds, no hardware.
#
# Usage: brio check ch32v203            all TUs, all parts
#        brio check ch32v203 pin        only TUs/negatives matching "pin"
set -u
cd "$(dirname "$0")/../.."

CXX=/sw/wch-riscv/bin/riscv32-wch-elf-g++
COMMON="-march=rv32imac_xw -mabi=ilp32 -std=gnu++23 -Os -Wall -Wextra -Werror -fno-exceptions -fno-rtti -c -Ibrio"
PARTS="ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb"
FILTER="${1:-}"
fail=0

# The part's own flag: the part DEFINITION, which is the one thing
# ch32v203/CMakeLists.txt derives from CH32V203_MCU that the code sees
# (cmake/ch32v203-parts.cmake is the table).
part_flags() {
    case "$1" in
        ch32v203f6) echo "-DCH32V203F6 $COMMON" ;;
        ch32v203f8) echo "-DCH32V203F8 $COMMON" ;;
        ch32v203g6) echo "-DCH32V203G6 $COMMON" ;;
        ch32v203g8) echo "-DCH32V203G8 $COMMON" ;;
        ch32v203k6) echo "-DCH32V203K6 $COMMON" ;;
        ch32v203k8) echo "-DCH32V203K8 $COMMON" ;;
        ch32v203c6) echo "-DCH32V203C6 $COMMON" ;;
        ch32v203c8) echo "-DCH32V203C8 $COMMON" ;;
        ch32v203rb) echo "-DCH32V203RB $COMMON" ;;
        *) echo "unknown part $1" >&2; exit 2 ;;
    esac
}

for tu in test/family_ch32v203/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    line="$(basename "$tu" .cpp):"
    parts="$(sed -n 's|^// mcu:||p' "$tu")"
    [ -n "$parts" ] || parts="$PARTS"
    for part in $parts; do
        for hpe in 0 1; do
            if $CXX $(part_flags "$part") -DBRIO_CH32_HPE=$hpe "$tu" -o /dev/null 2>/tmp/check_ch32v203_err; then
                line="$line $part/hpe$hpe"
            else
                line="$line $part/hpe$hpe:FAIL"
                fail=1
                sed "s/^/    /" /tmp/check_ch32v203_err | head -15
            fi
        done
    done
    echo "POS $line"
done

for tu in test/family_ch32v203/neg/*.cpp; do
    [ -e "$tu" ] || continue
    case "$tu" in *"$FILTER"*) ;; *) continue ;; esac
    parts="$(sed -n 's|^// mcu:||p' "$tu")"
    if [ -z "$parts" ]; then
        echo "NEG $(basename "$tu"): missing '// mcu:' line"; fail=1; continue
    fi
    line="$(basename "$tu" .cpp):"
    for part in $parts; do
        if $CXX $(part_flags "$part") "$tu" -o /dev/null 2>/dev/null; then
            line="$line $part:COMPILED(BAD)"
            fail=1
        else
            line="$line $part:refused"
        fi
    done
    echo "NEG $line"
done

[ "$fail" -eq 0 ] && echo "check_ch32v203: OK" || echo "check_ch32v203: FAILURES"
exit "$fail"
